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
#include <linux/pm_opp.h>

#include "panfrost_device.h"

#define CV200_SYSCTRL_SVB		0xf80000c4
#define CV200_PMC_HPM_CTRL		0xf8a23068
#define CV200_PMC_HPM_MONITOR		0xf8a23074
#define CV200_HPM_CTRL_ENABLE		BIT(24)
#define CV200_HPM_CTRL_MONITOR_ENABLE	BIT(26)
#define CV200_HPM_MONITOR_PERIOD		BIT(24)


int panfrost_hi3798cv200_init(struct panfrost_device *pfdev)
{
	struct dev_pm_opp_config config = { };
	void __iomem *sysctrl;
	void __iomem *pmc;
	u32 svb_raw, svb_bin;
	u32 val, expected;
	const char *prop_name;
	int ret;

	/* Map only the fixed always-on registers used by this platform. */
	sysctrl = devm_ioremap(pfdev->dev, CV200_SYSCTRL_SVB, sizeof(u32));
	if (!sysctrl)
		return dev_err_probe(pfdev->dev, -ENOMEM,
				     "failed to map SVB register\n");

	svb_raw = readl_relaxed(sysctrl);
	svb_bin = (svb_raw >> 24) & 0xff;
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
		return dev_err_probe(pfdev->dev, ret,
				     "failed to select %s\n", prop_name);

	dev_info(pfdev->dev, "SVB raw=0x%08x bin=0x%02x OPP=opp-microvolt-%s\n",
		 svb_raw, svb_bin, prop_name);

	/* HPM_CTRL and HPM_MONITOR are adjacent fixed PMC registers. */
	pmc = devm_ioremap(pfdev->dev, CV200_PMC_HPM_CTRL,
			   CV200_PMC_HPM_MONITOR - CV200_PMC_HPM_CTRL + sizeof(u32));
	if (!pmc)
		return dev_err_probe(pfdev->dev, -ENOMEM,
				     "failed to map HPM registers\n");

	/* Set a 4 ms monitor period while retaining the low 24 bits. */
	val = readl_relaxed(pmc + (CV200_PMC_HPM_MONITOR -
					CV200_PMC_HPM_CTRL));
	val = (val & GENMASK(23, 0)) | CV200_HPM_MONITOR_PERIOD;
	writel_relaxed(val, pmc + (CV200_PMC_HPM_MONITOR -
					CV200_PMC_HPM_CTRL));
	expected = val;
	val = readl_relaxed(pmc + (CV200_PMC_HPM_MONITOR -
					CV200_PMC_HPM_CTRL));
	if (val != expected) {
		ret = -EIO;
		goto hpm_error;
	}

	/* Enable HPM and its monitor without disturbing other control fields. */
	val = readl_relaxed(pmc);
	val |= CV200_HPM_CTRL_ENABLE | CV200_HPM_CTRL_MONITOR_ENABLE;
	writel_relaxed(val, pmc);
	expected = val & (CV200_HPM_CTRL_ENABLE |
				 CV200_HPM_CTRL_MONITOR_ENABLE);
	val = readl_relaxed(pmc);
	if ((val & (CV200_HPM_CTRL_ENABLE |
			   CV200_HPM_CTRL_MONITOR_ENABLE)) != expected) {
		ret = -EIO;
		goto hpm_error;
	}

	dev_info(pfdev->dev, "HPM init result=ok ctrl=0x%08x monitor=0x%08x\n",
		 val, readl_relaxed(pmc + (CV200_PMC_HPM_MONITOR -
						  CV200_PMC_HPM_CTRL)));
	return 0;

hpm_error:
	dev_err(pfdev->dev, "HPM init result=failed (%d)\n", ret);
	return ret;
}
