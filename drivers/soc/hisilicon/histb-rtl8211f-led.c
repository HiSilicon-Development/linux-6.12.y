// SPDX-License-Identifier: GPL-2.0-only
/* DT-selected RTL8211F LED wiring, independent of the selected PHY driver. */
#include <linux/bits.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/phy.h>

#define RTL8211F_PHY_ID			0x001cc916
#define RTL8211F_LED_PHY_ADDR		3

#define RTL8211F_PAGE_SELECT		0x1f
#define RTL8211F_LED_PAGE			0x0d04
#define RTL8211F_LEDCR			0x10
#define RTL8211F_EEELCR			0x11
#define RTL8211F_LED_MODE_B		BIT(15)
#define RTL8211F_LED_ACTIVITY		BIT(4)
#define RTL8211F_LED_LINK			(BIT(3) | BIT(1) | BIT(0))
#define RTL8211F_LED_FIELDS		(RTL8211F_LED_ACTIVITY | RTL8211F_LED_LINK)
#define RTL8211F_LED_SHIFT		5
#define RTL8211F_EEE_LED_ENABLE		GENMASK(3, 1)

static int rtl8211f_led_write(struct phy_device *phydev, u32 reg, u16 val)
{
	int ret;

	ret = __phy_write(phydev, reg, val);
	if (ret)
		return ret;

	ret = __phy_read(phydev, reg);
	if (ret < 0)
		return ret;

	return ret == val ? 0 : -EIO;
}

static int rtl8211f_led_fixup(struct phy_device *phydev)
{
	int old_page, lcr, eee, new_lcr, new_eee;
	int ret, rollback_lcr, rollback_eee;
	int page_ret = 0;
	u16 mask;

	if (phydev->phy_id != RTL8211F_PHY_ID ||
	    phydev->mdio.addr != RTL8211F_LED_PHY_ADDR ||
	    !of_property_read_bool(phydev->mdio.dev.of_node,
				   "realtek,led-link-activity"))
		return 0;

	/* Generic PHY has no page callbacks; serialize the whole transaction. */
	phy_lock_mdio_bus(phydev);
	old_page = __phy_read(phydev, RTL8211F_PAGE_SELECT);
	if (old_page < 0) {
		ret = old_page;
		goto unlock;
	}

	ret = rtl8211f_led_write(phydev, RTL8211F_PAGE_SELECT,
				RTL8211F_LED_PAGE);
	if (ret)
		goto restore_page;

	lcr = __phy_read(phydev, RTL8211F_LEDCR);
	if (lcr < 0) {
		ret = lcr;
		goto restore_page;
	}
	eee = __phy_read(phydev, RTL8211F_EEELCR);
	if (eee < 0) {
		ret = eee;
		goto restore_page;
	}

	/* Preserve reserved bits; LED0/1 are link, LED2 is independent activity. */
	mask = RTL8211F_LED_MODE_B | RTL8211F_LED_FIELDS |
	       (RTL8211F_LED_FIELDS << RTL8211F_LED_SHIFT) |
	       (RTL8211F_LED_FIELDS << (2 * RTL8211F_LED_SHIFT));
	new_lcr = (lcr & ~mask) | RTL8211F_LED_MODE_B | RTL8211F_LED_LINK |
		  (RTL8211F_LED_LINK << RTL8211F_LED_SHIFT) |
		  (RTL8211F_LED_ACTIVITY << (2 * RTL8211F_LED_SHIFT));
	new_eee = eee & ~RTL8211F_EEE_LED_ENABLE;

	if (lcr != new_lcr) {
		ret = rtl8211f_led_write(phydev, RTL8211F_LEDCR, new_lcr);
		if (ret)
			goto rollback;
	}
	if (eee != new_eee) {
		ret = rtl8211f_led_write(phydev, RTL8211F_EEELCR, new_eee);
		if (ret)
			goto rollback;
	}
	goto restore_page;

rollback:
	rollback_lcr = rtl8211f_led_write(phydev, RTL8211F_LEDCR, lcr);
	rollback_eee = rtl8211f_led_write(phydev, RTL8211F_EEELCR, eee);
	if (rollback_lcr || rollback_eee)
		phydev_warn(phydev, "RTL8211F LED rollback failed: LCR=%d EEELCR=%d\n",
			    rollback_lcr, rollback_eee);
restore_page:
	page_ret = rtl8211f_led_write(phydev, RTL8211F_PAGE_SELECT, old_page);
unlock:
	phy_unlock_mdio_bus(phydev);
	if (page_ret) {
		/* A wrong page would corrupt subsequent, unrelated PHY operations. */
		phydev_err(phydev, "RTL8211F LED MDIO page restore failed: %d\n", page_ret);
		return page_ret;
	}
	if (ret) {
		/* A cosmetic failure alone must not prevent the link from starting. */
		phydev_warn(phydev, "RTL8211F LED configuration failed: %d\n", ret);
		return 0;
	}

	phydev_info(phydev, "RTL8211F LEDs: link/activity LCR=%04x EEELCR=%04x\n",
		    new_lcr, new_eee);
	return 0;
}

static int __init rtl8211f_led_init(void)
{
	int ret;

	ret = phy_register_fixup_for_uid(RTL8211F_PHY_ID, 0xffffffff,
				       rtl8211f_led_fixup);
	if (ret)
		pr_err("RTL8211F LED fixup registration failed: %d\n", ret);
	return ret;
}
arch_initcall(rtl8211f_led_init);
