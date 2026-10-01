// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3798CV200 CPU and GPU PWM voltage regulators
 *
 * The PWM period and duty cycle share one 32-bit register. The output
 * voltage spans 650 mV to 1.15 V in 2.5 mV steps.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define HI3798CV200_PWM_MIN_UV		650000
#define HI3798CV200_PWM_UV_STEP		2500
#define HI3798CV200_PWM_N_VOLTAGES	201
#define HI3798CV200_PWM_PERIOD		201
#define HI3798CV200_PWM_PERIOD_MASK	GENMASK(15, 0)
#define HI3798CV200_PWM_DUTY_MASK	GENMASK(31, 16)
#define HI3798CV200_PWM_DUTY_SHIFT	16

struct hi3798cv200_pwm_regulator_data {
	const struct regulator_desc *desc;
	unsigned int settle_time_us;
};

struct hi3798cv200_pwm_regulator {
	void __iomem *base;
	const struct hi3798cv200_pwm_regulator_data *data;
};

static int hi3798cv200_pwm_get_voltage_sel(struct regulator_dev *rdev)
{
	struct hi3798cv200_pwm_regulator *regulator = rdev_get_drvdata(rdev);
	u32 duty, period, value;

	value = readl(regulator->base);
	period = value & HI3798CV200_PWM_PERIOD_MASK;
	duty = FIELD_GET(HI3798CV200_PWM_DUTY_MASK, value);
	if (period != HI3798CV200_PWM_PERIOD)
		return -EIO;
	if (!duty || duty > HI3798CV200_PWM_PERIOD)
		return -EINVAL;

	return HI3798CV200_PWM_PERIOD - duty;
}

static int hi3798cv200_pwm_set_voltage_sel(struct regulator_dev *rdev,
					   unsigned int selector)
{
	struct hi3798cv200_pwm_regulator *regulator = rdev_get_drvdata(rdev);
	u32 duty, old_value, readback, value;

	if (selector >= HI3798CV200_PWM_N_VOLTAGES)
		return -EINVAL;

	duty = HI3798CV200_PWM_PERIOD - selector;
	old_value = readl(regulator->base);
	value = old_value;
	value &= ~(HI3798CV200_PWM_PERIOD_MASK |
		   HI3798CV200_PWM_DUTY_MASK);
	value |= HI3798CV200_PWM_PERIOD;
	value |= duty << HI3798CV200_PWM_DUTY_SHIFT;
	writel(value, regulator->base);

	usleep_range(regulator->data->settle_time_us,
		     regulator->data->settle_time_us + 100);
	readback = readl(regulator->base);
	if ((readback & (HI3798CV200_PWM_PERIOD_MASK |
			 HI3798CV200_PWM_DUTY_MASK)) !=
	    (value & (HI3798CV200_PWM_PERIOD_MASK |
		      HI3798CV200_PWM_DUTY_MASK))) {
		writel(old_value, regulator->base);
		readl(regulator->base);
		return -EIO;
	}

	return 0;
}

static const struct regulator_ops hi3798cv200_pwm_regulator_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = hi3798cv200_pwm_get_voltage_sel,
	.set_voltage_sel = hi3798cv200_pwm_set_voltage_sel,
};

static const struct regulator_desc hi3798cv200_gpu_regulator_desc = {
	.name = "vdd-gpu",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &hi3798cv200_pwm_regulator_ops,
	.min_uV = HI3798CV200_PWM_MIN_UV,
	.uV_step = HI3798CV200_PWM_UV_STEP,
	.n_voltages = HI3798CV200_PWM_N_VOLTAGES,
};

static const struct regulator_desc hi3798cv200_cpu_regulator_desc = {
	.name = "vdd-cpu",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &hi3798cv200_pwm_regulator_ops,
	.min_uV = HI3798CV200_PWM_MIN_UV,
	.uV_step = HI3798CV200_PWM_UV_STEP,
	.n_voltages = HI3798CV200_PWM_N_VOLTAGES,
};

static const struct hi3798cv200_pwm_regulator_data hi3798cv200_gpu_data = {
	.desc = &hi3798cv200_gpu_regulator_desc,
	.settle_time_us = 1000,
};

static const struct hi3798cv200_pwm_regulator_data hi3798cv200_cpu_data = {
	.desc = &hi3798cv200_cpu_regulator_desc,
	/* The vendor DVFS sequence waits 10 ms before changing APLL. */
	.settle_time_us = 10000,
};

static int hi3798cv200_pwm_regulator_probe(struct platform_device *pdev)
{
	struct hi3798cv200_pwm_regulator *regulator;
	struct regulator_config config = { };
	struct regulator_dev *rdev;

	regulator = devm_kzalloc(&pdev->dev, sizeof(*regulator), GFP_KERNEL);
	if (!regulator)
		return -ENOMEM;

	regulator->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regulator->base))
		return PTR_ERR(regulator->base);
	regulator->data = of_device_get_match_data(&pdev->dev);
	if (!regulator->data)
		return -EINVAL;

	config.dev = &pdev->dev;
	config.driver_data = regulator;
	config.of_node = pdev->dev.of_node;
	config.init_data = of_get_regulator_init_data(&pdev->dev,
						      pdev->dev.of_node,
						      regulator->data->desc);
	if (!config.init_data)
		return -EINVAL;

	rdev = devm_regulator_register(&pdev->dev, regulator->data->desc,
				       &config);
	return PTR_ERR_OR_ZERO(rdev);
}

static const struct of_device_id hi3798cv200_pwm_regulator_of_match[] = {
	{
		.compatible = "hisilicon,hi3798cv200-gpu-regulator",
		.data = &hi3798cv200_gpu_data,
	},
	{
		.compatible = "hisilicon,hi3798cv200-cpu-regulator",
		.data = &hi3798cv200_cpu_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, hi3798cv200_pwm_regulator_of_match);

static struct platform_driver hi3798cv200_pwm_regulator_driver = {
	.probe = hi3798cv200_pwm_regulator_probe,
	.driver = {
		.name = "hi3798cv200-pwm-regulator",
		.of_match_table = hi3798cv200_pwm_regulator_of_match,
	},
};
module_platform_driver(hi3798cv200_pwm_regulator_driver);

MODULE_AUTHOR("HiSilicon Technologies Co., Ltd.");
MODULE_DESCRIPTION("HiSilicon Hi3798CV200 CPU/GPU PWM regulators");
MODULE_LICENSE("GPL");
