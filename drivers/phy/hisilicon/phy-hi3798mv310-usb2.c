// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * HiSilicon Hi3798MV310 direct-MMIO USB2 PHY.
 *
 * The EC6110T 760 U-Boot image supplies the three-port hardware sequence:
 * SHA256 0e4319e28d4ff73247c5c0627740f11c3084281e01b87f4a4a5553111eb0dd37,
 * initialization at 0xc57b80, shutdown at 0xc57dec (load base 0xc00000).
 * The analog values below are this firmware's profile, not MV300 tuning
 * or a claim that every MV310 board uses the same analog parameters.
 * The older HiSTBLinuxV100R005C00SPC060 hi3798mv310/clk-hiusb2.c omits
 * the third port; hi3796mv200/clk-hiusb2.c corroborates its register bits.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#define MV310_USB2_PORTS		3
#define MV310_USB2_PORT_STRIDE	0x400
#define MV310_USB2_PHY_CLK	0x18
#define MV310_PERI_USB_CTRL	0x12c
#define MV310_USB_WORD_IF	BIT(0)
#define MV310_USB_INCR16	BIT(9)

/* Both reference clocks precede the eight controller clocks. */
enum mv310_usb2_clock {
	MV310_CLK_REF,
	MV310_CLK_REF2,
	MV310_CLK_BUS,
	MV310_CLK_OHCI48,
	MV310_CLK_OHCI12,
	MV310_CLK_OTG_UTMI,
	MV310_CLK_HOST_PHY,
	MV310_CLK_UTMI0,
	MV310_CLK_UTMI1,
	MV310_CLK_UTMI2,
	MV310_NUM_CLOCKS,
};

enum mv310_usb2_reset {
	MV310_RST_BUS,
	MV310_RST_UTMI0,
	MV310_RST_UTMI1,
	MV310_RST_UTMI2,
	MV310_RST_HOST_PHY,
	MV310_RST_OTG_PHY,
	MV310_RST_PORT0,
	MV310_RST_PORT1,
	MV310_RST_PORT2,
	MV310_RST_POR,
	MV310_RST_POR2,
	MV310_RST_TEST,
	MV310_RST_TEST2,
	MV310_NUM_RESETS,
};

#define MV310_CTRL_CLOCKS	(MV310_NUM_CLOCKS - MV310_CLK_BUS)
#define MV310_CTRL_RESETS	(MV310_RST_OTG_PHY + 1)
#define MV310_PHY_RESETS		(MV310_RST_TEST - MV310_RST_PORT0)
#define MV310_REF_CLOCKS		MV310_CLK_BUS

struct mv310_usb2_phy;

struct mv310_usb2_port {
	struct mv310_usb2_phy *priv;
	struct phy *phy;
	bool initialized;
	bool powered;
};

struct mv310_usb2_phy {
	struct device *dev;
	void __iomem *base;
	void __iomem *base2;
	struct regmap *peripheral;
	struct clk_bulk_data clocks[MV310_NUM_CLOCKS];
	struct reset_control_bulk_data resets[MV310_NUM_RESETS];
	struct mutex lock;
	unsigned int initialized_ports;
	unsigned int powered_ports;
	struct mv310_usb2_port ports[MV310_USB2_PORTS];
};

static void mv310_usb2_configure(struct mv310_usb2_phy *priv)
{
	static const struct {
		u8 offset;
		u8 pair_value;
		u8 port2_value;
	} settings[] = {
		{ 0x08, 0x5c, 0x5c },
		{ 0x00, 0x1c, 0x1c },
		{ 0x14, 0x84, 0x84 },
		{ 0x10, 0xd7, 0xd7 },
		{ 0x7c, 0x68, 0xe8 },
	};
	int i;

	/* The two-port block has only one clock-register write, at port 0. */
	writel(0x5, priv->base + MV310_USB2_PHY_CLK);
	for (i = 0; i < ARRAY_SIZE(settings); i++) {
		writel(settings[i].pair_value, priv->base + settings[i].offset);
		writel(settings[i].pair_value, priv->base + MV310_USB2_PORT_STRIDE +
		       settings[i].offset);
		udelay(20);
	}

	writel(0x5, priv->base2 + MV310_USB2_PHY_CLK);
	for (i = 0; i < ARRAY_SIZE(settings); i++) {
		writel(settings[i].port2_value, priv->base2 + settings[i].offset);
		udelay(20);
	}
}

static int mv310_usb2_start(struct mv310_usb2_phy *priv)
{
	int ret, cleanup_ret;

	ret = reset_control_bulk_assert(MV310_CTRL_RESETS, priv->resets);
	if (ret)
		return ret;
	udelay(200);

	ret = reset_control_bulk_assert(MV310_PHY_RESETS,
				       &priv->resets[MV310_RST_PORT0]);
	if (ret)
		return ret;
	udelay(200);

	/* MV310 ASIC interface setup; this register is not part of the CRG. */
	ret = regmap_update_bits(priv->peripheral, MV310_PERI_USB_CTRL,
				 MV310_USB_WORD_IF | MV310_USB_INCR16, 0);
	if (ret)
		return ret;
	udelay(100);

	ret = clk_bulk_enable(MV310_REF_CLOCKS, priv->clocks);
	if (ret)
		return ret;
	udelay(300);

	ret = reset_control_bulk_deassert(2, &priv->resets[MV310_RST_POR]);
	if (ret)
		goto err_reset;
	udelay(500);

	ret = reset_control_bulk_deassert(2, &priv->resets[MV310_RST_TEST]);
	if (ret)
		goto err_reset;
	udelay(200);

	mv310_usb2_configure(priv);

	ret = reset_control_bulk_deassert(MV310_USB2_PORTS,
					 &priv->resets[MV310_RST_PORT0]);
	if (ret)
		goto err_reset;
	usleep_range(2000, 2100);

	ret = clk_bulk_enable(MV310_CTRL_CLOCKS, &priv->clocks[MV310_CLK_BUS]);
	if (ret)
		goto err_reset;
	udelay(200);

	ret = reset_control_bulk_deassert(MV310_CTRL_RESETS, priv->resets);
	if (ret)
		goto err_disable_clocks;
	udelay(200);

	return 0;

err_disable_clocks:
	clk_bulk_disable(MV310_CTRL_CLOCKS, &priv->clocks[MV310_CLK_BUS]);
err_reset:
	/* No consumer is active yet; leave controller, ports and POR in reset. */
	cleanup_ret = reset_control_bulk_assert(MV310_RST_TEST, priv->resets);
	if (cleanup_ret)
		dev_err(priv->dev, "failed to reset USB2 after power-on error: %d\n",
			cleanup_ret);
	udelay(100);
	clk_bulk_disable(MV310_REF_CLOCKS, priv->clocks);
	return ret;
}

static int mv310_usb2_stop(struct mv310_usb2_phy *priv)
{
	int ret, cleanup_ret;

	/* The vendor shutdown leaves the OTG PHY system reset unchanged. */
	ret = reset_control_bulk_assert(MV310_RST_OTG_PHY, priv->resets);
	if (ret)
		return ret;
	udelay(200);

	ret = reset_control_bulk_assert(MV310_PHY_RESETS,
				       &priv->resets[MV310_RST_PORT0]);
	if (ret)
		goto err_release_controller;
	udelay(100);

	/* Gate release is the generic-PHY lifecycle, not U-Boot's stop policy. */
	clk_bulk_disable(MV310_CTRL_CLOCKS, &priv->clocks[MV310_CLK_BUS]);
	clk_bulk_disable(MV310_REF_CLOCKS, priv->clocks);
	return 0;

err_release_controller:
	cleanup_ret = reset_control_bulk_deassert(MV310_RST_OTG_PHY, priv->resets);
	if (cleanup_ret)
		dev_err(priv->dev, "failed to restore USB2 controller: %d\n",
			cleanup_ret);
	udelay(200);
	return ret;
}

static int mv310_usb2_init(struct phy *phy)
{
	struct mv310_usb2_port *port = phy_get_drvdata(phy);
	struct mv310_usb2_phy *priv = port->priv;
	int ret = 0;

	mutex_lock(&priv->lock);
	if (!priv->initialized_ports) {
		ret = clk_bulk_prepare(MV310_NUM_CLOCKS, priv->clocks);
		if (ret)
			goto out;
	}
	priv->initialized_ports++;
	port->initialized = true;
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int mv310_usb2_exit(struct phy *phy)
{
	struct mv310_usb2_port *port = phy_get_drvdata(phy);
	struct mv310_usb2_phy *priv = port->priv;
	int ret = 0;

	mutex_lock(&priv->lock);
	if (port->powered) {
		ret = -EBUSY;
		goto out;
	}
	port->initialized = false;
	if (!--priv->initialized_ports)
		clk_bulk_unprepare(MV310_NUM_CLOCKS, priv->clocks);
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int mv310_usb2_power_on(struct phy *phy)
{
	struct mv310_usb2_port *port = phy_get_drvdata(phy);
	struct mv310_usb2_phy *priv = port->priv;
	int ret = 0;

	mutex_lock(&priv->lock);
	if (!port->initialized) {
		ret = -EINVAL;
		goto out;
	}
	if (!priv->powered_ports) {
		ret = mv310_usb2_start(priv);
		if (ret)
			goto out;
	}
	priv->powered_ports++;
	port->powered = true;
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int mv310_usb2_power_off(struct phy *phy)
{
	struct mv310_usb2_port *port = phy_get_drvdata(phy);
	struct mv310_usb2_phy *priv = port->priv;
	int ret = 0;

	mutex_lock(&priv->lock);
	if (priv->powered_ports == 1) {
		ret = mv310_usb2_stop(priv);
		if (ret)
			goto out;
	}
	priv->powered_ports--;
	port->powered = false;
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static const struct phy_ops mv310_usb2_ops = {
	.init = mv310_usb2_init,
	.exit = mv310_usb2_exit,
	.power_on = mv310_usb2_power_on,
	.power_off = mv310_usb2_power_off,
	.owner = THIS_MODULE,
};

static struct phy *mv310_usb2_xlate(struct device *dev,
				   const struct of_phandle_args *args)
{
	struct mv310_usb2_phy *priv = dev_get_drvdata(dev);

	if (args->args_count != 1 || args->args[0] >= MV310_USB2_PORTS)
		return ERR_PTR(-EINVAL);

	return priv->ports[args->args[0]].phy;
}

static int mv310_usb2_probe(struct platform_device *pdev)
{
	static const char * const clock_names[] = {
		"ref", "ref2", "bus", "ohci48", "ohci12", "otg-utmi",
		"host-phy", "utmi0", "utmi1", "utmi2",
	};
	static const char * const reset_names[] = {
		"bus", "utmi0", "utmi1", "utmi2", "host-phy", "otg-phy",
		"port0", "port1", "port2", "por", "por2", "phy-test", "phy-test2",
	};
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct mv310_usb2_phy *priv;
	int i, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->base = devm_platform_ioremap_resource_byname(pdev, "phy01");
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);
	priv->base2 = devm_platform_ioremap_resource_byname(pdev, "phy2");
	if (IS_ERR(priv->base2))
		return PTR_ERR(priv->base2);

	priv->peripheral = syscon_regmap_lookup_by_phandle(dev->of_node,
						"hisilicon,peripheral-syscon");
	if (IS_ERR(priv->peripheral))
		return dev_err_probe(dev, PTR_ERR(priv->peripheral),
				     "failed to get peripheral syscon\n");

	for (i = 0; i < MV310_NUM_CLOCKS; i++)
		priv->clocks[i].id = clock_names[i];
	ret = devm_clk_bulk_get(dev, MV310_NUM_CLOCKS, priv->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	for (i = 0; i < MV310_NUM_RESETS; i++)
		priv->resets[i].id = reset_names[i];
	ret = devm_reset_control_bulk_get_exclusive(dev, MV310_NUM_RESETS,
						  priv->resets);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get resets\n");

	mutex_init(&priv->lock);
	platform_set_drvdata(pdev, priv);

	for (i = 0; i < MV310_USB2_PORTS; i++) {
		struct mv310_usb2_port *port = &priv->ports[i];

		port->priv = priv;
		port->phy = devm_phy_create(dev, dev->of_node, &mv310_usb2_ops);
		if (IS_ERR(port->phy))
			return dev_err_probe(dev, PTR_ERR(port->phy),
					     "failed to create PHY %d\n", i);
		phy_set_drvdata(port->phy, port);
	}

	provider = devm_of_phy_provider_register(dev, mv310_usb2_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id mv310_usb2_of_match[] = {
	{ .compatible = "hisilicon,hi3798mv310-usb2-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, mv310_usb2_of_match);

static struct platform_driver mv310_usb2_driver = {
	.probe = mv310_usb2_probe,
	.driver = {
		.name = "hi3798mv310-usb2-phy",
		.of_match_table = mv310_usb2_of_match,
	},
};
module_platform_driver(mv310_usb2_driver);

MODULE_DESCRIPTION("HiSilicon Hi3798MV310 USB2 PHY driver");
MODULE_LICENSE("GPL");
