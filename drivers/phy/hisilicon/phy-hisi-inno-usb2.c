// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * HiSilicon INNO USB2 PHY Driver.
 *
 * Copyright (c) 2016-2017 HiSilicon Technologies Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#define INNO_PHY_PORT_NUM	2
#define REF_CLK_STABLE_TIME	100	/* unit:us */
#define UTMI_CLK_STABLE_TIME	200	/* unit:us */
#define TEST_CLK_STABLE_TIME	2	/* unit:ms */
#define PHY_CLK_STABLE_TIME	2	/* unit:ms */
#define UTMI_RST_COMPLETE_TIME	2	/* unit:ms */
#define POR_RST_COMPLETE_TIME	300	/* unit:us */

#define PHY_TYPE_0	0
#define PHY_TYPE_1	1

#define PHY_TEST_DATA		GENMASK(7, 0)
#define PHY_TEST_ADDR_OFFSET	8
#define PHY0_TEST_ADDR		GENMASK(15, 8)
#define PHY0_TEST_PORT_OFFSET	16
#define PHY0_TEST_PORT		GENMASK(18, 16)
#define PHY0_TEST_WREN		BIT(21)
#define PHY0_TEST_CLK		BIT(22)	/* rising edge active */
#define PHY0_TEST_RST		BIT(23)	/* low active */
#define PHY1_TEST_ADDR		GENMASK(11, 8)
#define PHY1_TEST_PORT_OFFSET	12
#define PHY1_TEST_PORT		BIT(12)
#define PHY1_TEST_WREN		BIT(13)
#define PHY1_TEST_CLK		BIT(14)	/* rising edge active */
#define PHY1_TEST_RST		BIT(15)	/* low active */

#define PHY_CLK_ENABLE		BIT(2)

struct hisi_inno_phy_port {
	struct reset_control *utmi_rst;
	struct hisi_inno_phy_priv *priv;
	u8 index;
};

struct hisi_inno_phy_priv {
	void __iomem *mmio;
	struct clk *ref_clk;
	struct reset_control *por_rst;
	unsigned int type;
	unsigned int port_count;
	unsigned int active_ports;
	bool is_hi3798cv200;
	struct mutex lock;
	struct hisi_inno_phy_port ports[INNO_PHY_PORT_NUM];
};

static void hisi_inno_phy_write_reg(struct hisi_inno_phy_priv *priv,
				    u8 port, u32 addr, u32 data)
{
	void __iomem *reg = priv->mmio;
	u32 val;
	u32 value;

	if (priv->type == PHY_TYPE_0)
		val = (data & PHY_TEST_DATA) |
		      ((addr << PHY_TEST_ADDR_OFFSET) & PHY0_TEST_ADDR) |
		      ((port << PHY0_TEST_PORT_OFFSET) & PHY0_TEST_PORT) |
		      PHY0_TEST_WREN | PHY0_TEST_RST;
	else
		val = (data & PHY_TEST_DATA) |
		      ((addr << PHY_TEST_ADDR_OFFSET) & PHY1_TEST_ADDR) |
		      ((port << PHY1_TEST_PORT_OFFSET) & PHY1_TEST_PORT) |
		      PHY1_TEST_WREN | PHY1_TEST_RST;
	writel(val, reg);

	value = val;
	if (priv->type == PHY_TYPE_0)
		value |= PHY0_TEST_CLK;
	else
		value |= PHY1_TEST_CLK;
	writel(value, reg);

	writel(val, reg);
}

static void hisi_inno_phy_setup_2p_port0(struct hisi_inno_phy_priv *priv)
{
	/* The vendor host-0 path enables the shared PHY clock first. */
	hisi_inno_phy_write_reg(priv, 0, 0x06, PHY_CLK_ENABLE);
	usleep_range(1000, 1100);

	hisi_inno_phy_write_reg(priv, 0, 0x00, 0x1c);
	udelay(20);

	hisi_inno_phy_write_reg(priv, 0, 0x07, 0x00);
	udelay(20);

	hisi_inno_phy_write_reg(priv, 0, 0x11, 0x40);
	udelay(20);
}

static void hisi_inno_phy_setup_2p_port1(struct hisi_inno_phy_priv *priv)
{
	/* The PHY clock is controlled by the port 0 register 0x06. */
	hisi_inno_phy_write_reg(priv, 0, 0x06, PHY_CLK_ENABLE);
	usleep_range(1000, 1100);

	hisi_inno_phy_write_reg(priv, 1, 0x00, 0x1c);
	udelay(20);

	hisi_inno_phy_write_reg(priv, 0, 0x06, 0x07);
	udelay(20);

	hisi_inno_phy_write_reg(priv, 1, 0x07, 0x00);
	udelay(20);

	hisi_inno_phy_write_reg(priv, 0, 0x0a, 0xab);
	udelay(20);

	hisi_inno_phy_write_reg(priv, 1, 0x11, 0x40);
	udelay(20);

	hisi_inno_phy_write_reg(priv, 1, 0x10, 0x41);
	udelay(20);
}

static void hisi_inno_phy_setup_1p(struct hisi_inno_phy_priv *priv)
{
	hisi_inno_phy_write_reg(priv, 0, 0x06, PHY_CLK_ENABLE);
	usleep_range(1000, 1100);

	hisi_inno_phy_write_reg(priv, 0, 0x00, 0x1c);
	udelay(20);

	hisi_inno_phy_write_reg(priv, 0, 0x05, 0x92);
	hisi_inno_phy_write_reg(priv, 0, 0x06, 0x06);
	udelay(20);

	hisi_inno_phy_write_reg(priv, 0, 0x0a, 0xab);
	udelay(20);

	hisi_inno_phy_write_reg(priv, 0, 0x07, 0x00);
	udelay(20);

	hisi_inno_phy_write_reg(priv, 0, 0x11, 0xc0);
	udelay(20);
}

static void hisi_inno_phy_setup_cv200(struct hisi_inno_phy_port *port)
{
	struct hisi_inno_phy_priv *priv = port->priv;

	if (priv->port_count == 1)
		hisi_inno_phy_setup_1p(priv);
	else if (port->index == 0)
		hisi_inno_phy_setup_2p_port0(priv);
	else
		hisi_inno_phy_setup_2p_port1(priv);
}

static int hisi_inno_phy_init_legacy(struct phy *phy)
{
	struct hisi_inno_phy_port *port = phy_get_drvdata(phy);
	struct hisi_inno_phy_priv *priv = port->priv;
	int ret;

	ret = clk_prepare_enable(priv->ref_clk);
	if (ret)
		return ret;
	udelay(REF_CLK_STABLE_TIME);

	reset_control_deassert(priv->por_rst);
	udelay(POR_RST_COMPLETE_TIME);

	/* Preserve the setup used by the pre-CV200 integrations. */
	hisi_inno_phy_write_reg(priv, 0, 0x06, PHY_CLK_ENABLE);
	msleep(PHY_CLK_STABLE_TIME);

	reset_control_deassert(port->utmi_rst);
	udelay(UTMI_RST_COMPLETE_TIME);

	return 0;
}

static int hisi_inno_phy_init_cv200(struct phy *phy)
{
	struct hisi_inno_phy_port *port = phy_get_drvdata(phy);
	struct hisi_inno_phy_priv *priv = port->priv;
	bool first_port;
	int ret;

	mutex_lock(&priv->lock);
	first_port = !priv->active_ports;

	if (first_port) {
		ret = clk_prepare_enable(priv->ref_clk);
		if (ret)
			goto out_unlock;
		udelay(REF_CLK_STABLE_TIME);
	}

	ret = reset_control_assert(port->utmi_rst);
	if (ret)
		goto err_disable_ref_clk;
	usleep_range(UTMI_RST_COMPLETE_TIME * 1000,
		     UTMI_RST_COMPLETE_TIME * 1000 + 100);

	if (first_port) {
		/* POR and ref_clk are shared by every port in this provider. */
		ret = reset_control_assert(priv->por_rst);
		if (ret)
			goto err_disable_ref_clk;
		udelay(POR_RST_COMPLETE_TIME);

		ret = reset_control_deassert(priv->por_rst);
		if (ret)
			goto err_assert_por;
		udelay(POR_RST_COMPLETE_TIME);
	}

	/* Set up phy registers */
	hisi_inno_phy_setup_cv200(port);

	ret = reset_control_deassert(port->utmi_rst);
	if (ret)
		goto err_assert_utmi;
	usleep_range(UTMI_RST_COMPLETE_TIME * 1000,
		     UTMI_RST_COMPLETE_TIME * 1000 + 100);

	priv->active_ports++;
	mutex_unlock(&priv->lock);
	return 0;

err_assert_utmi:
	reset_control_assert(port->utmi_rst);
err_assert_por:
	if (first_port)
		reset_control_assert(priv->por_rst);
err_disable_ref_clk:
	if (first_port)
		clk_disable_unprepare(priv->ref_clk);
out_unlock:
	mutex_unlock(&priv->lock);
	return ret;
}

static int hisi_inno_phy_exit_legacy(struct phy *phy)
{
	struct hisi_inno_phy_port *port = phy_get_drvdata(phy);
	struct hisi_inno_phy_priv *priv = port->priv;

	reset_control_assert(port->utmi_rst);
	reset_control_assert(priv->por_rst);
	clk_disable_unprepare(priv->ref_clk);

	return 0;
}

static int hisi_inno_phy_exit_cv200(struct phy *phy)
{
	struct hisi_inno_phy_port *port = phy_get_drvdata(phy);
	struct hisi_inno_phy_priv *priv = port->priv;
	int ret;
	int tmp;

	mutex_lock(&priv->lock);
	ret = reset_control_assert(port->utmi_rst);

	if (WARN_ON(!priv->active_ports))
		goto out_unlock;

	priv->active_ports--;
	if (!priv->active_ports) {
		tmp = reset_control_assert(priv->por_rst);
		if (!ret)
			ret = tmp;
		clk_disable_unprepare(priv->ref_clk);
	}

out_unlock:
	mutex_unlock(&priv->lock);
	return ret;
}

static int hisi_inno_phy_init(struct phy *phy)
{
	struct hisi_inno_phy_port *port = phy_get_drvdata(phy);

	if (!port->priv->is_hi3798cv200)
		return hisi_inno_phy_init_legacy(phy);

	return hisi_inno_phy_init_cv200(phy);
}

static int hisi_inno_phy_exit(struct phy *phy)
{
	struct hisi_inno_phy_port *port = phy_get_drvdata(phy);

	if (!port->priv->is_hi3798cv200)
		return hisi_inno_phy_exit_legacy(phy);

	return hisi_inno_phy_exit_cv200(phy);
}

static const struct phy_ops hisi_inno_phy_ops = {
	.init = hisi_inno_phy_init,
	.exit = hisi_inno_phy_exit,
	.owner = THIS_MODULE,
};

static int hisi_inno_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct hisi_inno_phy_priv *priv;
	struct phy_provider *provider;
	int i = 0;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->mmio = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->mmio)) {
		ret = PTR_ERR(priv->mmio);
		return ret;
	}

	priv->ref_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->ref_clk))
		return PTR_ERR(priv->ref_clk);

	priv->por_rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(priv->por_rst))
		return PTR_ERR(priv->por_rst);

	priv->type = (uintptr_t) of_device_get_match_data(dev);
	priv->is_hi3798cv200 =
		of_device_is_compatible(np, "hisilicon,hi3798cv200-usb2-phy");
	mutex_init(&priv->lock);

	for_each_child_of_node_scoped(np, child) {
		struct reset_control *rst;
		struct phy *phy;

		rst = of_reset_control_get_exclusive(child, NULL);
		if (IS_ERR(rst))
			return PTR_ERR(rst);

		priv->ports[i].utmi_rst = rst;
		priv->ports[i].priv = priv;
		priv->ports[i].index = i;

		phy = devm_phy_create(dev, child, &hisi_inno_phy_ops);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		phy_set_bus_width(phy, 8);
		phy_set_drvdata(phy, &priv->ports[i]);
		i++;

		if (i >= INNO_PHY_PORT_NUM) {
			dev_warn(dev, "Support %d ports in maximum\n", i);
			break;
		}
	}
	priv->port_count = i;

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id hisi_inno_phy_of_match[] = {
	{ .compatible = "hisilicon,inno-usb2-phy",
	  .data = (void *) PHY_TYPE_0 },
	{ .compatible = "hisilicon,hi3798cv200-usb2-phy",
	  .data = (void *) PHY_TYPE_0 },
	{ .compatible = "hisilicon,hi3798mv100-usb2-phy",
	  .data = (void *) PHY_TYPE_1 },
	{ },
};
MODULE_DEVICE_TABLE(of, hisi_inno_phy_of_match);

static struct platform_driver hisi_inno_phy_driver = {
	.probe	= hisi_inno_phy_probe,
	.driver = {
		.name	= "hisi-inno-phy",
		.of_match_table	= hisi_inno_phy_of_match,
	}
};
module_platform_driver(hisi_inno_phy_driver);

MODULE_DESCRIPTION("HiSilicon INNO USB2 PHY Driver");
MODULE_LICENSE("GPL v2");
