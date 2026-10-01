// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 Linaro Ltd.
 * Copyright (c) 2014 HiSilicon Limited.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <media/rc-core.h>

#define IR_ENABLE		0x00
#define IR_CONFIG		0x04
#define CNT_LEADS		0x08
#define CNT_LEADE		0x0c
#define CNT_SLEADE		0x10
#define CNT0_B			0x14
#define CNT1_B			0x18
#define IR_BUSY			0x1c
#define IR_DATAH		0x20
#define IR_DATAL		0x24
#define IR_INTM			0x28
#define IR_INTS			0x2c
#define IR_INTC			0x30
#define IR_START		0x34

/* interrupt mask */
#define INTMS_SYMBRCV		(BIT(24) | BIT(8))
#define INTMS_TIMEOUT		(BIT(25) | BIT(9))
#define INTMS_OVERFLOW		(BIT(26) | BIT(10))
#define INT_CLR_OVERFLOW	BIT(18)
#define INT_CLR_TIMEOUT		BIT(17)
#define INT_CLR_RCV		BIT(16)
#define INT_CLR_RCVTIMEOUT	(BIT(16) | BIT(17))
#define IR_INT_MASK_SYMBOL	((u32)~GENMASK(18, 16))

#define IR_CLK_ENABLE		BIT(4)
#define IR_CLK_RESET		BIT(5)

/* IR_ENABLE register bits */
#define IR_ENABLE_EN		BIT(0)
#define IR_ENABLE_EN_EXTRA	BIT(8)

#define IR_CFG_WIDTH_MASK	0xffff
#define IR_CFG_WIDTH_SHIFT	16
#define IR_CFG_FORMAT_MASK	0x3
#define IR_CFG_FORMAT_SHIFT	14
#define IR_CFG_INT_LEVEL_MASK	0x3f
#define IR_CFG_INT_LEVEL_SHIFT	8
/* only support raw mode */
#define IR_CFG_MODE_RAW		BIT(7)
#define IR_CFG_FREQ_MASK	0x7f
#define IR_CFG_FREQ_SHIFT	0
#define IR_CFG_INT_THRESHOLD	1
/* symbol start from low to high, symbol stream end at high*/
#define IR_CFG_SYMBOL_FMT	0
#define IR_CFG_SYMBOL_MAXWIDTH	0x3e80

#define IR_HIX5HD2_NAME		"hix5hd2-ir"

/* Hi3796CV300 needs an additional enable bit in IR_ENABLE. */
#define HIX5HD2_FLAG_EXTRA_ENABLE	BIT(0)

struct hix5hd2_soc_data {
	u32 clk_reg;
	u32 flags;
	/* The interrupt-level field is programmed with the raw level on this SoC */
	bool int_level_raw;
};

static const struct hix5hd2_soc_data hi3796cv300_data = {
	.clk_reg = 0x60,
	.flags = HIX5HD2_FLAG_EXTRA_ENABLE,
};

static const struct hix5hd2_soc_data hi3798cv200_data = {
	.clk_reg = 0x48,
	.int_level_raw = true,
};

static const struct hix5hd2_soc_data hix5hd2_data = {
	.clk_reg = 0x48,
};

struct hix5hd2_ir_priv {
	int			irq;
	void __iomem		*base;
	struct device		*dev;
	struct rc_dev		*rdev;
	struct regmap		*regmap;
	struct clk		*clock;
	unsigned long		rate;
	const struct hix5hd2_soc_data *socdata;
	bool			irq_enabled;
};

static int hix5hd2_ir_power(struct hix5hd2_ir_priv *priv, bool on)
{
	u32 mask = IR_CLK_ENABLE | IR_CLK_RESET;
	u32 val = on ? IR_CLK_ENABLE : IR_CLK_RESET;

	if (!priv->regmap)
		return 0;

	return regmap_update_bits(priv->regmap, priv->socdata->clk_reg,
				  mask, val);
}

static inline void hix5hd2_ir_enable(struct hix5hd2_ir_priv *priv)
{
	u32 val = IR_ENABLE_EN;

	if (priv->socdata->flags & HIX5HD2_FLAG_EXTRA_ENABLE)
		val |= IR_ENABLE_EN_EXTRA;

	writel_relaxed(val, priv->base + IR_ENABLE);
}

static void hix5hd2_ir_disable(struct hix5hd2_ir_priv *priv)
{
	writel(0, priv->base + IR_ENABLE);
}

static void hix5hd2_ir_clear_fifo(struct hix5hd2_ir_priv *priv)
{
	u32 count, i;

	count = readl(priv->base + IR_DATAH);
	for (i = 0; i < count; i++)
		readl(priv->base + IR_DATAL);
}

static void hix5hd2_ir_stop(struct hix5hd2_ir_priv *priv)
{
	writel(~0U, priv->base + IR_INTM);
	hix5hd2_ir_clear_fifo(priv);
	writel(~0U, priv->base + IR_INTC);
	readl(priv->base + IR_INTS);
	hix5hd2_ir_disable(priv);
}

/* A level source with an unknown status must not be handed back to the GIC. */
static void hix5hd2_ir_quench_unknown_irq(struct hix5hd2_ir_priv *priv)
{
	writel(~0U, priv->base + IR_INTM);
	hix5hd2_ir_clear_fifo(priv);
	writel(~0U, priv->base + IR_INTC);
	readl(priv->base + IR_INTS);
	dev_warn_ratelimited(priv->dev,
			     "quenching unknown interrupt status\n");
}

static int hix5hd2_ir_config(struct hix5hd2_ir_priv *priv)
{
	int timeout = 10000;
	u32 val, rate;

	hix5hd2_ir_enable(priv);

	while (readl_relaxed(priv->base + IR_BUSY)) {
		if (timeout--) {
			udelay(1);
		} else {
			dev_err(priv->dev, "IR_BUSY timeout\n");
			return -ETIMEDOUT;
		}
	}

	/* Now only support raw mode, with symbol start from low to high */
	rate = DIV_ROUND_CLOSEST(priv->rate, 1000000);
	if (!rate || rate > IR_CFG_FREQ_MASK + 1)
		return -EINVAL;

	val = (IR_CFG_SYMBOL_MAXWIDTH & IR_CFG_WIDTH_MASK)
	      << IR_CFG_WIDTH_SHIFT;
	val |= (IR_CFG_SYMBOL_FMT & IR_CFG_FORMAT_MASK)
	       << IR_CFG_FORMAT_SHIFT;
	/*
	 * The field holds the interrupt level.  The vendor driver for the
	 * Hi3798CV200 block (ir_s2 in the SDK) has DFT_INT_LEVEL = 1 and
	 * writes that 1 straight into the field; this driver has always
	 * written level - 1, which is a property of the older hix5hd2 block.
	 * Keep the historical encoding where the SoC data does not say
	 * otherwise.
	 */
	val |= ((priv->socdata->int_level_raw ? IR_CFG_INT_THRESHOLD
					      : IR_CFG_INT_THRESHOLD - 1)
		& IR_CFG_INT_LEVEL_MASK) << IR_CFG_INT_LEVEL_SHIFT;
	val |= IR_CFG_MODE_RAW;
	val |= ((rate - 1) & IR_CFG_FREQ_MASK) << IR_CFG_FREQ_SHIFT;
	writel(val, priv->base + IR_CONFIG);

	hix5hd2_ir_clear_fifo(priv);
	writel(~0U, priv->base + IR_INTC);
	readl(priv->base + IR_INTS);
	writel(IR_INT_MASK_SYMBOL, priv->base + IR_INTM);
	/* write arbitrary value to start  */
	writel(0x01, priv->base + IR_START);
	return 0;
}

static void hix5hd2_ir_irq_enable(struct hix5hd2_ir_priv *priv)
{
	if (!priv->irq_enabled) {
		enable_irq(priv->irq);
		priv->irq_enabled = true;
	}
}

static void hix5hd2_ir_irq_disable(struct hix5hd2_ir_priv *priv)
{
	if (priv->irq_enabled) {
		disable_irq(priv->irq);
		priv->irq_enabled = false;
	}
}

static int hix5hd2_ir_open(struct rc_dev *rdev)
{
	struct hix5hd2_ir_priv *priv = rdev->priv;
	int ret;

	ret = clk_prepare_enable(priv->clock);
	if (ret)
		return ret;

	ret = hix5hd2_ir_power(priv, true);
	if (ret)
		goto err_clock;

	ret = hix5hd2_ir_config(priv);
	if (!ret) {
		/* rc_open() is only called after rc-core registered raw reception. */
		hix5hd2_ir_irq_enable(priv);
		return 0;
	}

	hix5hd2_ir_stop(priv);
	hix5hd2_ir_power(priv, false);
err_clock:
	clk_disable_unprepare(priv->clock);
	return ret;
}

static void hix5hd2_ir_close(struct rc_dev *rdev)
{
	struct hix5hd2_ir_priv *priv = rdev->priv;

	hix5hd2_ir_irq_disable(priv);
	hix5hd2_ir_stop(priv);
	hix5hd2_ir_power(priv, false);
	clk_disable_unprepare(priv->clock);
}

static irqreturn_t hix5hd2_ir_rx_interrupt(int irq, void *data)
{
	u32 symb_num, symb_val, symb_time;
	u32 data_l, data_h;
	u32 irq_clear = 0;
	u32 irq_sr, i;
	struct hix5hd2_ir_priv *priv = data;
	bool events = false;
	bool handled = false;

	irq_sr = readl(priv->base + IR_INTS);
	if (irq_sr & INTMS_OVERFLOW) {
		handled = true;
		/*
		 * we must read IR_DATAL first, then we can clean up
		 * IR_INTS availably since logic would not clear
		 * fifo when overflow, drv do the job
		 */
		ir_raw_event_overflow(priv->rdev);
		symb_num = readl(priv->base + IR_DATAH);
		for (i = 0; i < symb_num; i++)
			readl(priv->base + IR_DATAL);

		irq_clear |= INT_CLR_OVERFLOW;
		dev_warn_ratelimited(priv->dev, "overflow, level=%d\n",
				     IR_CFG_INT_THRESHOLD);
	}

	if ((irq_sr & INTMS_SYMBRCV) || (irq_sr & INTMS_TIMEOUT)) {
		struct ir_raw_event ev = {};

		handled = true;
		events = true;
		symb_num = readl(priv->base + IR_DATAH);
		for (i = 0; i < symb_num; i++) {
			symb_val = readl(priv->base + IR_DATAL);
			data_l = ((symb_val & 0xffff) * 10);
			data_h =  ((symb_val >> 16) & 0xffff) * 10;
			symb_time = (data_l + data_h) / 10;

			ev.duration = data_l;
			ev.pulse = true;
			ir_raw_event_store(priv->rdev, &ev);

			if (symb_time < IR_CFG_SYMBOL_MAXWIDTH) {
				ev.duration = data_h;
				ev.pulse = false;
				ir_raw_event_store(priv->rdev, &ev);
			} else {
				ir_raw_event_set_idle(priv->rdev, true);
			}
		}

		if (irq_sr & INTMS_SYMBRCV)
			irq_clear |= INT_CLR_RCV;
		if (irq_sr & INTMS_TIMEOUT)
			irq_clear |= INT_CLR_TIMEOUT;
	}

	if (!handled) {
		hix5hd2_ir_quench_unknown_irq(priv);
		return IRQ_NONE;
	}

	writel(irq_clear, priv->base + IR_INTC);
	/* Flush the W1C write before the GIC unmasks this level interrupt. */
	readl(priv->base + IR_INTS);

	if (events)
		ir_raw_event_handle(priv->rdev);

	return IRQ_HANDLED;
}

static const struct of_device_id hix5hd2_ir_table[] = {
	{ .compatible = "hisilicon,hi3796cv300-ir", .data = &hi3796cv300_data },
	{ .compatible = "hisilicon,hi3798cv200-ir", .data = &hi3798cv200_data },
	{ .compatible = "hisilicon,hix5hd2-ir", .data = &hix5hd2_data },
	{},
};
MODULE_DEVICE_TABLE(of, hix5hd2_ir_table);

static int hix5hd2_ir_probe(struct platform_device *pdev)
{
	struct rc_dev *rdev;
	struct device *dev = &pdev->dev;
	struct hix5hd2_ir_priv *priv;
	struct device_node *node = pdev->dev.of_node;
	const char *map_name;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = dev;

	priv->socdata = device_get_match_data(dev);
	if (!priv->socdata) {
		dev_err(dev, "Unable to initialize IR data\n");
		return -ENODEV;
	}

	if (of_property_present(node, "hisilicon,power-syscon")) {
		priv->regmap = syscon_regmap_lookup_by_phandle(node,
							       "hisilicon,power-syscon");
		if (IS_ERR(priv->regmap))
			return dev_err_probe(dev, PTR_ERR(priv->regmap),
					     "power syscon lookup failed\n");
	} else {
		priv->regmap = NULL;
	}

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return priv->irq;

	rdev = rc_allocate_device(RC_DRIVER_IR_RAW);
	if (!rdev)
		return -ENOMEM;

	priv->clock = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clock)) {
		ret = PTR_ERR(priv->clock);
		dev_err(dev, "clock lookup failed: %d\n", ret);
		goto err;
	}
	priv->rate = clk_get_rate(priv->clock);
	if (!priv->rate) {
		ret = -EINVAL;
		dev_err(dev, "clock has no usable rate\n");
		goto err;
	}

	rdev->allowed_protocols = RC_PROTO_BIT_ALL_IR_DECODER;
	rdev->priv = priv;
	rdev->open = hix5hd2_ir_open;
	rdev->close = hix5hd2_ir_close;
	rdev->driver_name = IR_HIX5HD2_NAME;
	map_name = of_get_property(node, "linux,rc-map-name", NULL);
	rdev->map_name = map_name ?: RC_MAP_EMPTY;
	rdev->device_name = IR_HIX5HD2_NAME;
	rdev->input_phys = IR_HIX5HD2_NAME "/input0";
	rdev->input_id.bustype = BUS_HOST;
	rdev->input_id.vendor = 0x0001;
	rdev->input_id.product = 0x0001;
	rdev->input_id.version = 0x0100;
	rdev->rx_resolution = 10;
	rdev->timeout = IR_CFG_SYMBOL_MAXWIDTH * 10;
	priv->rdev = rdev;

	/* Keep the receiver reset and its level IRQ disabled until opened. */
	ret = hix5hd2_ir_power(priv, false);
	if (ret) {
		dev_err(dev, "failed to reset receiver: %d\n", ret);
		goto err;
	}

	ret = devm_request_irq(dev, priv->irq, hix5hd2_ir_rx_interrupt,
			       IRQF_NO_AUTOEN, pdev->name, priv);
	if (ret) {
		dev_err(dev, "IRQ %d registration failed: %d\n",
			priv->irq, ret);
		goto err;
	}

	ret = rc_register_device(rdev);
	if (ret < 0)
		goto err;

	mutex_lock(&rdev->lock);
	if (rdev->users)
		hix5hd2_ir_irq_enable(priv);
	mutex_unlock(&rdev->lock);

	platform_set_drvdata(pdev, priv);

	return 0;

err:
	rc_free_device(rdev);
	dev_err(dev, "Unable to register device (%d)\n", ret);
	return ret;
}

static void hix5hd2_ir_remove(struct platform_device *pdev)
{
	struct hix5hd2_ir_priv *priv = platform_get_drvdata(pdev);

	rc_unregister_device(priv->rdev);
	rc_free_device(priv->rdev);
}

#ifdef CONFIG_PM_SLEEP
static int hix5hd2_ir_suspend(struct device *dev)
{
	struct hix5hd2_ir_priv *priv = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&priv->rdev->lock);
	if (priv->rdev->users) {
		hix5hd2_ir_irq_disable(priv);
		hix5hd2_ir_stop(priv);
		ret = hix5hd2_ir_power(priv, false);
		clk_disable_unprepare(priv->clock);
	}
	mutex_unlock(&priv->rdev->lock);

	return ret;
}

static int hix5hd2_ir_resume(struct device *dev)
{
	struct hix5hd2_ir_priv *priv = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&priv->rdev->lock);
	if (!priv->rdev->users)
		goto out;

	ret = clk_prepare_enable(priv->clock);
	if (ret)
		goto out;

	ret = hix5hd2_ir_power(priv, true);
	if (!ret)
		ret = hix5hd2_ir_config(priv);
	if (ret) {
		hix5hd2_ir_stop(priv);
		hix5hd2_ir_power(priv, false);
		clk_disable_unprepare(priv->clock);
	} else {
		hix5hd2_ir_irq_enable(priv);
	}

out:
	mutex_unlock(&priv->rdev->lock);
	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(hix5hd2_ir_pm_ops, hix5hd2_ir_suspend,
			 hix5hd2_ir_resume);

static struct platform_driver hix5hd2_ir_driver = {
	.driver = {
		.name = IR_HIX5HD2_NAME,
		.of_match_table = hix5hd2_ir_table,
		.pm     = &hix5hd2_ir_pm_ops,
	},
	.probe = hix5hd2_ir_probe,
	.remove_new = hix5hd2_ir_remove,
};

module_platform_driver(hix5hd2_ir_driver);

MODULE_DESCRIPTION("IR controller driver for HiSilicon STB platforms");
MODULE_AUTHOR("Guoxiong Yan <yanguoxiong@huawei.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:hix5hd2-ir");
