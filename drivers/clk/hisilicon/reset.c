// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hisilicon Reset Controller Driver
 *
 * Copyright (c) 2015-2016 HiSilicon Technologies Co., Ltd.
 */

#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include "clk.h"
#include "reset.h"

#define	HISI_RESET_BIT_MASK	0x1f
#define	HISI_RESET_OFFSET_SHIFT	8
#define	HISI_RESET_OFFSET_MASK	0xffff00
#define HISI_RESET_ACTIVE_LOW	BIT(30)

struct hisi_reset_variant {
	bool shared_crg_lock;
	bool active_low_ids;
	bool assign_rcdev_dev;
};

struct hisi_reset_controller {
	spinlock_t	lock;
	void __iomem	*membase;
	resource_size_t	size;
	const struct hisi_reset_variant *variant;
	struct reset_controller_dev	rcdev;
};

static const struct hisi_reset_variant hisi_reset_sysctrl_variant = {
	.shared_crg_lock = true,
};

static const struct hisi_reset_variant hisi_reset_cv200_variant = {
	.shared_crg_lock = true,
	.active_low_ids = true,
	.assign_rcdev_dev = true,
};

static const struct hisi_reset_variant hisi_reset_mv310_variant = {
	.shared_crg_lock = true,
	.assign_rcdev_dev = true,
};
static const struct hisi_reset_variant hisi_reset_default_variant;



#define to_hisi_reset_controller(rcdev)  \
	container_of(rcdev, struct hisi_reset_controller, rcdev)

static int hisi_reset_of_xlate(struct reset_controller_dev *rcdev,
			const struct of_phandle_args *reset_spec)
{
	struct hisi_reset_controller *rstc =
		to_hisi_reset_controller(rcdev);
	u32 offset;
	u32 bit;

	if (reset_spec->args_count != 2)
		return -EINVAL;
	if (reset_spec->args[0] >
	    (HISI_RESET_OFFSET_MASK >> HISI_RESET_OFFSET_SHIFT))
		return -EINVAL;

	if (reset_spec->args[1] &
	    ~(HISI_RESET_BIT_MASK |
	      (rstc->variant->active_low_ids ? HISI_RESET_ACTIVE_LOW : 0)))
		return -EINVAL;

	bit = reset_spec->args[1] & HISI_RESET_BIT_MASK;

	offset = (reset_spec->args[0] << HISI_RESET_OFFSET_SHIFT)
		& HISI_RESET_OFFSET_MASK;
	if (rstc->size < sizeof(u32) ||
	    (offset >> HISI_RESET_OFFSET_SHIFT) > rstc->size - sizeof(u32))
		return -EINVAL;
	if (reset_spec->args[0] & (sizeof(u32) - 1))
		return -EINVAL;

	return offset | bit |
		(rstc->variant->active_low_ids ?
		 reset_spec->args[1] & HISI_RESET_ACTIVE_LOW : 0);
}

static int hisi_reset_assert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct hisi_reset_controller *rstc = to_hisi_reset_controller(rcdev);
	spinlock_t *lock = rstc->variant->shared_crg_lock ?
		&hisi_clk_lock : &rstc->lock;
	unsigned long flags;
	u32 offset, reg;
	u8 bit;

	offset = (id & HISI_RESET_OFFSET_MASK) >> HISI_RESET_OFFSET_SHIFT;
	bit = id & HISI_RESET_BIT_MASK;

	/* Clock gates and reset bits can share CRG RMW registers. */
	spin_lock_irqsave(lock, flags);

	reg = readl(rstc->membase + offset);
	if (id & HISI_RESET_ACTIVE_LOW)
		reg &= ~BIT(bit);
	else
		reg |= BIT(bit);
	writel(reg, rstc->membase + offset);

	spin_unlock_irqrestore(lock, flags);

	return 0;
}

static int hisi_reset_deassert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct hisi_reset_controller *rstc = to_hisi_reset_controller(rcdev);
	spinlock_t *lock = rstc->variant->shared_crg_lock ?
		&hisi_clk_lock : &rstc->lock;
	unsigned long flags;
	u32 offset, reg;
	u8 bit;

	offset = (id & HISI_RESET_OFFSET_MASK) >> HISI_RESET_OFFSET_SHIFT;
	bit = id & HISI_RESET_BIT_MASK;

	/* Clock gates and reset bits can share CRG RMW registers. */
	spin_lock_irqsave(lock, flags);

	reg = readl(rstc->membase + offset);
	if (id & HISI_RESET_ACTIVE_LOW)
		reg |= BIT(bit);
	else
		reg &= ~BIT(bit);
	writel(reg, rstc->membase + offset);

	spin_unlock_irqrestore(lock, flags);

	return 0;
}

static const struct reset_control_ops hisi_reset_ops = {
	.assert		= hisi_reset_assert,
	.deassert	= hisi_reset_deassert,
};

struct hisi_reset_controller *hisi_reset_init(struct platform_device *pdev)
{
	struct hisi_reset_controller *rstc;
	struct resource *res;
	const struct hisi_reset_variant *variant;
	int ret;
	if (of_device_is_compatible(pdev->dev.of_node,
				    "hisilicon,hi3798cv200-crg"))
		variant = &hisi_reset_cv200_variant;
	else if (of_device_is_compatible(pdev->dev.of_node,
					 "hisilicon,hi3798cv200-sysctrl"))
		variant = &hisi_reset_sysctrl_variant;
	else if (of_device_is_compatible(pdev->dev.of_node,
					 "hisilicon,hi3798mv310-crg"))
		variant = &hisi_reset_mv310_variant;
	else
		variant = &hisi_reset_default_variant;

	rstc = devm_kzalloc(&pdev->dev, sizeof(*rstc), GFP_KERNEL);
	if (!rstc)
		return NULL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return NULL;

	rstc->membase = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rstc->membase))
		return NULL;

	spin_lock_init(&rstc->lock);
	rstc->size = resource_size(res);
	rstc->variant = variant;
	rstc->rcdev.owner = THIS_MODULE;
	rstc->rcdev.ops = &hisi_reset_ops;
	if (variant->assign_rcdev_dev)
		rstc->rcdev.dev = &pdev->dev;
	rstc->rcdev.of_node = pdev->dev.of_node;
	rstc->rcdev.of_reset_n_cells = 2;
	rstc->rcdev.of_xlate = hisi_reset_of_xlate;
	ret = reset_controller_register(&rstc->rcdev);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to register reset controller: %d\n", ret);
		return NULL;
	}

	return rstc;
}
EXPORT_SYMBOL_GPL(hisi_reset_init);

void hisi_reset_exit(struct hisi_reset_controller *rstc)
{
	reset_controller_unregister(&rstc->rcdev);
}
EXPORT_SYMBOL_GPL(hisi_reset_exit);
