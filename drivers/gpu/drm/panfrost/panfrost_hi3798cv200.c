// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hi3798CV200 Mali-T720 platform setup.
 *
 * SVB selection and HPM setup follow HiSTBLinuxV100R005C00SPC060
 * source/msp/drv/gpu/midgard/drivers/gpu/arm/midgard/platform/devicetree/
 * mali_kbase_config_devicetree.c and mali_kbase_avs.c. The former credits
 * (C) COPYRIGHT 2015 ARM Limited. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>

#include "panfrost_device.h"

#define CV200_SYSCTRL_SVB		0xf80000c4
#define CV200_PMC_HPM_CTRL		0xf8a23068
#define CV200_PMC_HPM_MONITOR		0xf8a23074
#define CV200_HPM_CTRL_ENABLE		BIT(24)
#define CV200_HPM_CTRL_MONITOR_ENABLE	BIT(26)
#define CV200_HPM_MONITOR_PERIOD		BIT(24)
#define CV200_HPM_PERIOD_MASK		GENMASK(31, 24)

struct panfrost_cv200_data {
	void __iomem *sysctrl;
	void __iomem *pmc;
	u32 svb_bin;
};

static int panfrost_cv200_hpm_init(struct panfrost_device *pfdev)
{
	struct panfrost_cv200_data *data = pfdev->vendor_data;
	void __iomem *monitor = data->pmc +
		(CV200_PMC_HPM_MONITOR - CV200_PMC_HPM_CTRL);
	u32 enable = CV200_HPM_CTRL_ENABLE | CV200_HPM_CTRL_MONITOR_ENABLE;
	u32 val;

	/* A 4 ms period, retaining the independent target fields. */
	val = readl(monitor);
	val = (val & ~CV200_HPM_PERIOD_MASK) | CV200_HPM_MONITOR_PERIOD;
	writel(val, monitor);
	if ((readl(monitor) & CV200_HPM_PERIOD_MASK) != CV200_HPM_MONITOR_PERIOD)
		return dev_err_probe(pfdev->dev, -EIO, "HPM period readback failed\n");

	val = readl(data->pmc) | enable;
	writel(val, data->pmc);
	if ((readl(data->pmc) & enable) != enable)
		return dev_err_probe(pfdev->dev, -EIO, "HPM enable readback failed\n");
	return 0;
}

int panfrost_hi3798cv200_resume(struct panfrost_device *pfdev)
{
	struct panfrost_cv200_data *data = pfdev->vendor_data;
	u32 enable = CV200_HPM_CTRL_ENABLE | CV200_HPM_CTRL_MONITOR_ENABLE;
	u32 period;

	if (!of_machine_is_compatible("hisilicon,hi3798cv200"))
		return 0;
	/* OPP voltage-bin selection is software state and survives retention. */
	if ((readl(data->sysctrl) >> 24) != data->svb_bin)
		return dev_err_probe(pfdev->dev, -EIO, "SVB bin lost across standby\n");
	period = readl(data->pmc + CV200_PMC_HPM_MONITOR - CV200_PMC_HPM_CTRL);
	if ((period & CV200_HPM_PERIOD_MASK) == CV200_HPM_MONITOR_PERIOD &&
	    (readl(data->pmc) & enable) == enable)
		return 0;
	/* Reapply only after observed PMC context loss, before devfreq restarts. */
	return panfrost_cv200_hpm_init(pfdev);
}

int panfrost_hi3798cv200_init(struct panfrost_device *pfdev)
{
	struct dev_pm_opp_config config = { };
	struct panfrost_cv200_data *data;
	u32 svb_raw, svb_bin;
	const char *prop_name;
	int ret;

	data = devm_kzalloc(pfdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	pfdev->vendor_data = data;

	/* Map only the fixed always-on registers used by this platform. */
	data->sysctrl = devm_ioremap(pfdev->dev, CV200_SYSCTRL_SVB, sizeof(u32));
	if (!data->sysctrl)
		return dev_err_probe(pfdev->dev, -ENOMEM, "failed to map SVB register\n");

	svb_raw = readl(data->sysctrl);
	svb_bin = (svb_raw >> 24) & 0xff;
	data->svb_bin = svb_bin;
	switch (svb_bin) {
	case 1:
		prop_name = "ff";
		break;
	case 2:
		prop_name = "tt";
		break;
	case 3:
		prop_name = "ss";
		break;
	default:
		dev_warn(pfdev->dev, "unknown SVB bin 0x%02x, using SS\n", svb_bin);
		svb_bin = 3;
		prop_name = "ss";
		break;
	}

	config.prop_name = prop_name;
	ret = devm_pm_opp_set_config(pfdev->dev, &config);
	if (ret)
		return dev_err_probe(pfdev->dev, ret, "failed to select %s\n", prop_name);

	dev_info(pfdev->dev, "SVB raw=0x%08x bin=0x%02x OPP=opp-microvolt-%s\n",
		 svb_raw, svb_bin, prop_name);

	data->pmc = devm_ioremap(pfdev->dev, CV200_PMC_HPM_CTRL,
			CV200_PMC_HPM_MONITOR - CV200_PMC_HPM_CTRL + sizeof(u32));
	if (!data->pmc)
		return dev_err_probe(pfdev->dev, -ENOMEM, "failed to map HPM registers\n");
	ret = panfrost_cv200_hpm_init(pfdev);
	if (!ret)
		dev_info(pfdev->dev, "HPM init result=ok ctrl=0x%08x monitor=0x%08x\n",
			 readl(data->pmc), readl(data->pmc +
			 CV200_PMC_HPM_MONITOR - CV200_PMC_HPM_CTRL));
	return ret;
}
