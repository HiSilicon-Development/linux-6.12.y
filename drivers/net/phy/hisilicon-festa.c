// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon FESTA internal Fast Ethernet PHYs.
 *
 * Patch sequences and PHY IDs are from HiSTBLinuxV100R005C00SPC060,
 * linux-4.4.y/drivers/net/ethernet/hieth/phy.c:187-215,250-306,335-345
 * and include/linux/hisilicon_phy.h:5-13. The tables are copied verbatim
 * from hieth/festa_v330.h and hieth/festa_s28v112.h. Match real MDIO IDs;
 * the SoC compatible is not evidence of a particular PHY revision.
 */
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/phy.h>

#define FESTA_V330_ID		0x20669853
#define FESTA_S28V112_ID		0x20669900
#define FESTA_EXP_DATA		0x1d
#define FESTA_EXP_ADDR		0x1e
#define FESTA_ED_TIME		0x3160

static const u16 festa_v330_patch[] = {
#include "hisilicon-festa-v330.h"
};

static const u16 festa_s28v112_patch[] = {
#include "hisilicon-festa-s28v112.h"
};

static_assert(ARRAY_SIZE(festa_v330_patch) % 2 == 0);
static_assert(ARRAY_SIZE(festa_s28v112_patch) % 2 == 0);

static int festa_exp_write(struct phy_device *phydev, u16 reg, u16 value)
{
	int ret;

	ret = __phy_write(phydev, FESTA_EXP_ADDR, reg);
	if (ret)
		return ret;
	return __phy_write(phydev, FESTA_EXP_DATA, value);
}

static int festa_load_patch(struct phy_device *phydev, const u16 *patch,
			    unsigned int count)
{
	int bmcr, ret, restore;
	unsigned int i;

	phy_lock_mdio_bus(phydev);
	bmcr = __phy_read(phydev, MII_BMCR);
	if (bmcr < 0) {
		ret = bmcr;
		goto unlock;
	}
	ret = __phy_write(phydev, MII_BMCR, bmcr | BMCR_PDOWN);
	if (ret)
		goto unlock;
	for (i = 0; i < count; i += 2) {
		ret = festa_exp_write(phydev, patch[i], patch[i + 1]);
		if (ret)
			break;
		cond_resched();
	}
	/* Leave power-down even on an upload error, without hiding that error. */
	restore = __phy_write(phydev, MII_BMCR, bmcr & ~BMCR_PDOWN);
	if (!ret)
		ret = restore;
unlock:
	phy_unlock_mdio_bus(phydev);
	return ret;
}

static int festa_v330_config_init(struct phy_device *phydev)
{
	return festa_load_patch(phydev, festa_v330_patch,
				ARRAY_SIZE(festa_v330_patch));
}

static int festa_s28v112_config_init(struct phy_device *phydev)
{
	int ret;

	ret = festa_load_patch(phydev, festa_s28v112_patch,
			       ARRAY_SIZE(festa_s28v112_patch));
	if (ret)
		return ret;
	/* BSP phy.h:27-31 and phy.c:304: default ED timeout is 8 minutes. */
	phy_lock_mdio_bus(phydev);
	ret = festa_exp_write(phydev, FESTA_ED_TIME, (8 * 60 * 4 / 10) - 1);
	phy_unlock_mdio_bus(phydev);
	return ret;
}

static struct phy_driver festa_drivers[] = {
	{
		PHY_ID_MATCH_MODEL(FESTA_V330_ID),
		.name = "HiSilicon FESTA v330",
		.flags = PHY_IS_INTERNAL,
		.config_init = festa_v330_config_init,
		.suspend = genphy_suspend,
		.resume = genphy_resume,
		.read_mmd = genphy_read_mmd_unsupported,
		.write_mmd = genphy_write_mmd_unsupported,
	}, {
		PHY_ID_MATCH_MODEL(FESTA_S28V112_ID),
		.name = "HiSilicon FESTA S28V112",
		.flags = PHY_IS_INTERNAL,
		.config_init = festa_s28v112_config_init,
		.suspend = genphy_suspend,
		.resume = genphy_resume,
		.read_mmd = genphy_read_mmd_unsupported,
		.write_mmd = genphy_write_mmd_unsupported,
	},
};
module_phy_driver(festa_drivers);

static const struct mdio_device_id __maybe_unused festa_ids[] = {
	{ PHY_ID_MATCH_MODEL(FESTA_V330_ID) },
	{ PHY_ID_MATCH_MODEL(FESTA_S28V112_ID) },
	{ }
};
MODULE_DEVICE_TABLE(mdio, festa_ids);

MODULE_DESCRIPTION("HiSilicon FESTA internal Ethernet PHY driver");
MODULE_LICENSE("GPL");
