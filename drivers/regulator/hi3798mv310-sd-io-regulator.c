// SPDX-License-Identifier: GPL-2.0-only
/* MV310 SD0 3.3 V I/O: SPC060 himciv300_set_ldo(), PERI_SD_LDO. */
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

static const struct regulator_ops mv310_sd_io_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc mv310_sd_io_desc = {
	.name = "sd0-io-3v3",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &mv310_sd_io_ops,
	.n_voltages = 1,
	.fixed_uV = 3300000,
	.enable_reg = 0x11c,
	.enable_mask = GENMASK(6, 4),
	.enable_val = BIT(6) | BIT(5),
	.disable_val = 0,
};

static int mv310_sd_io_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config config = { .dev = dev, .of_node = dev->of_node };
	struct regulator_dev *rdev;

	config.regmap = syscon_regmap_lookup_by_phandle(dev->of_node,
						      "hisilicon,peripheral-syscon");
	if (IS_ERR(config.regmap))
		return dev_err_probe(dev, PTR_ERR(config.regmap),
				     "failed to get peripheral syscon\n");

	config.init_data = of_get_regulator_init_data(dev, dev->of_node,
						     &mv310_sd_io_desc);
	rdev = devm_regulator_register(dev, &mv310_sd_io_desc, &config);
	return PTR_ERR_OR_ZERO(rdev);
}

static const struct of_device_id mv310_sd_io_match[] = {
	{ .compatible = "hisilicon,hi3798mv310-sd-io-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(of, mv310_sd_io_match);

static struct platform_driver mv310_sd_io_driver = {
	.probe = mv310_sd_io_probe,
	.driver = {
		.name = "hi3798mv310-sd-io-regulator",
		.of_match_table = mv310_sd_io_match,
	},
};
module_platform_driver(mv310_sd_io_driver);

MODULE_DESCRIPTION("Hi3798MV310 SD0 3.3 V I/O supply");
MODULE_LICENSE("GPL");
