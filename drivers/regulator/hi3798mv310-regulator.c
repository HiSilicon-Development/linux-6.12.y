// SPDX-License-Identifier: GPL-2.0-only
/* MV310 shared CPU/GPU/Core PWM arbitration with calibrated CPU voltage. */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define MV310_MIN_UV 700000
#define MV310_MAX_UV 1250000
#define MV310_STEP_UV 5000
#define MV310_PWM_CLASS 2
#define MV310_CPU 0
#define MV310_GPU 1

#define MV310_SC_GENM6 0x98
#define MV310_SC_GENM17 0xc4
#define MV310_SC_GENM18 0xc8
#define MV310_SC_GENY3 0xf0c

struct mv310_supply {
	struct device *dev;
	struct notifier_block cpufreq_nb;
	void __iomem *pwm;
	struct mutex lock;
	struct clk *cpu_clk;
	int vote[2];
	bool faulted;
	bool calibration_valid;
	u32 corner_hpm;
	u32 k_800m;
	u32 b_800m;
	u32 max_800m;
	u32 k_1200m;
	u32 b_1200m;
	u32 max_1200m;
	u8 otp_add_800m;
	u8 otp_add_1200m;
};

static int mv310_load_calibration(struct device *dev,
				  struct mv310_supply *s)
{
	struct regmap *sysctrl;
	u32 genm6, genm17, genm18, geny3;
	u8 otp_800m, otp_1200m;
	int ret;

	sysctrl = syscon_regmap_lookup_by_phandle(dev->of_node,
						  "hisilicon,sysctrl");
	if (IS_ERR(sysctrl))
		return PTR_ERR(sysctrl);

	ret = regmap_read(sysctrl, MV310_SC_GENM6, &genm6);
	if (ret)
		return ret;
	ret = regmap_read(sysctrl, MV310_SC_GENM17, &genm17);
	if (ret)
		return ret;
	ret = regmap_read(sysctrl, MV310_SC_GENM18, &genm18);
	if (ret)
		return ret;
	ret = regmap_read(sysctrl, MV310_SC_GENY3, &geny3);
	if (ret)
		return ret;

	ret = nvmem_cell_read_u8(dev, "cpu-800m", &otp_800m);
	if (ret)
		return ret;
	ret = nvmem_cell_read_u8(dev, "cpu-1200m", &otp_1200m);
	if (ret)
		return ret;

	s->corner_hpm = FIELD_GET(GENMASK(9, 0), genm6);
	s->k_800m = geny3 >> 16;
	s->b_800m = geny3 & 0xffff;
	s->max_800m = genm17 >> 16;
	s->k_1200m = genm18 >> 16;
	s->b_1200m = genm18 & 0xffff;
	s->max_1200m = genm6 >> 16;
	s->otp_add_800m = (otp_800m & BIT(7)) ? (otp_800m & 0xf) * 10 : 0;
	s->otp_add_1200m = (otp_1200m & BIT(7)) ? (otp_1200m & 0xf) * 10 : 0;

	if (!s->corner_hpm || s->b_800m < MV310_MIN_UV / 1000 ||
	    s->max_800m < MV310_MIN_UV / 1000 ||
	    s->b_1200m < MV310_MIN_UV / 1000 ||
	    s->max_1200m < MV310_MIN_UV / 1000)
		return -EINVAL;

	s->calibration_valid = true;
	dev_info(dev, "CPU voltage calibration loaded (HPM=%u, OTP=%u/%u mV)\n",
		 s->corner_hpm, s->otp_add_800m, s->otp_add_1200m);
	return 0;
}

static int mv310_cpu_opp_uv(const struct mv310_supply *s, unsigned long rate)
{
	u32 k, b, max, otp_add, low_temp_add;
	u64 drop;

	if (rate > 1600000000)
		return -EINVAL;
	/* No characterized 1.6 GHz curve: retain the conservative ceiling. */
	if (rate == 1600000000 || !s->calibration_valid)
		return MV310_MAX_UV;
	if (rate == 400000000 || rate == 600000000 || rate == 800000000) {
		k = s->k_800m;
		b = s->b_800m;
		max = s->max_800m;
		otp_add = s->otp_add_800m;
		low_temp_add = 50;
	} else if (rate == 1200000000) {
		k = s->k_1200m;
		b = s->b_1200m;
		max = s->max_1200m;
		otp_add = s->otp_add_1200m;
		low_temp_add = 30;
	} else {
		return -EINVAL;
	}

	drop = div_u64((u64)k * s->corner_hpm, 1000);
	if (drop > b)
		return MV310_MAX_UV;
	b = min(b - (u32)drop, max);
	b += otp_add + low_temp_add;
	return clamp_t(u32, b * 1000, MV310_MIN_UV, MV310_MAX_UV);
}

static int mv310_pwm_to_uv(u32 value)
{
	u32 period = value & 0xffff, duty = value >> 16;
	u64 delta;

	/* Boot firmware may use a different period; voltage follows its ratio. */
	if (period < 2 || !duty || duty > period)
		return -ERANGE;
	delta = (u64)(duty - 1) * (MV310_MAX_UV - MV310_MIN_UV) /
		(period - 1);
	if (delta > MV310_MAX_UV - MV310_MIN_UV)
		return -ERANGE;
	return MV310_MAX_UV - delta;
}

static int mv310_write_uv(struct mv310_supply *s, int uv)
{
	u32 period, duty, value;

	if (uv < MV310_MIN_UV || uv > MV310_MAX_UV)
		return -ERANGE;
	period = ((MV310_MAX_UV - MV310_MIN_UV) * MV310_PWM_CLASS /
		  MV310_STEP_UV) + 1;
	duty = ((MV310_MAX_UV - uv) * MV310_PWM_CLASS /
		 MV310_STEP_UV) + 1;
	value = period | duty << 16;
	writel(value, s->pwm);
	if (readl(s->pwm) != value)
		return -EIO;
	usleep_range(1000, 1100);
	return 0;
}

/* These two regulator objects represent votes, not independent power rails. */
static int mv310_get_voltage(struct regulator_dev *rdev)
{
	struct mv310_supply *s = rdev_get_drvdata(rdev);

	return READ_ONCE(s->vote[rdev_get_id(rdev)]);
}

static int mv310_write_vote(struct mv310_supply *s, int id, int uv)
{
	int old, cpu, ret;

	mutex_lock(&s->lock);
	if (s->faulted) {
		ret = -EIO;
		goto unlock;
	}
	cpu = mv310_cpu_opp_uv(s, clk_get_rate(s->cpu_clk));
	if (cpu < 0)
		cpu = MV310_MAX_UV;
	old = s->vote[id];
	s->vote[id] = uv;
	ret = mv310_write_uv(s, max(max(s->vote[0], s->vote[1]), cpu));
	if (ret) {
		s->faulted = true;
		s->vote[id] = old;
		/* Best effort: put the shared rail at its ceiling after write failure. */
		mv310_write_uv(s, MV310_MAX_UV);
	}
unlock:
	mutex_unlock(&s->lock);
	return ret;
}

static int mv310_set_voltage(struct regulator_dev *rdev, int min_uv,
			     int max_uv, unsigned int *selector)
{
	struct mv310_supply *s = rdev_get_drvdata(rdev);
	int uv;

	uv = DIV_ROUND_UP(max(min_uv, MV310_MIN_UV), MV310_STEP_UV) * MV310_STEP_UV;
	if (uv > min(max_uv, MV310_MAX_UV))
		return -EINVAL;

	return mv310_write_vote(s, rdev_get_id(rdev), uv);
}

static int mv310_apply_cpu_vote(struct mv310_supply *s)
{
	int uv = mv310_cpu_opp_uv(s, clk_get_rate(s->cpu_clk));

	if (uv < 0)
		uv = MV310_MAX_UV;

	return mv310_write_vote(s, MV310_CPU, uv);
}

static const struct regulator_ops mv310_supply_ops = {
	.get_voltage = mv310_get_voltage,
	.set_voltage = mv310_set_voltage,
};

static const struct regulator_desc mv310_supplies[] = {
	{
		.name = "mv310-cpu-vote",
		.of_match = "cpu",
		.regulators_node = "regulators",
		.id = MV310_CPU,
		.ops = &mv310_supply_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.continuous_voltage_range = true,
	}, {
		.name = "mv310-gpu-vote",
		.of_match = "gpu",
		.regulators_node = "regulators",
		.id = MV310_GPU,
		.ops = &mv310_supply_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.continuous_voltage_range = true,
	},
};

/* cpufreq-dt owns the OPP table and creates it before this notification. */
static int mv310_set_opps_voltage(struct device *dev,
				  const struct mv310_supply *s)
{
	struct dev_pm_opp *opp;
	unsigned long rate = 0;
	int ret, uv;

	ret = dev_pm_opp_get_opp_count(dev);
	if (ret <= 0)
		return ret ? ret : -ENODEV;

	for (;;) {
		opp = dev_pm_opp_find_freq_ceil(dev, &rate);
		if (IS_ERR(opp))
			return PTR_ERR(opp) == -ERANGE ? 0 : PTR_ERR(opp);
		dev_pm_opp_put(opp);
		uv = mv310_cpu_opp_uv(s, rate);
		if (uv < 0)
			return uv;
		ret = dev_pm_opp_adjust_voltage(dev, rate, uv, uv, MV310_MAX_UV);
		if (ret)
			return ret;
		rate++;
	}
}

static int mv310_cpufreq_policy_notify(struct notifier_block *nb,
				       unsigned long event, void *data)
{
	struct mv310_supply *s = container_of(nb, struct mv310_supply,
						      cpufreq_nb);
	struct cpufreq_policy *policy = data;
	struct device *cpu;
	int ret;

	if (event != CPUFREQ_CREATE_POLICY || !policy || policy->cpu != 0)
		return NOTIFY_DONE;

	cpu = get_cpu_device(policy->cpu);
	if (!cpu) {
		dev_err(s->dev, "CPU0 device is unavailable for OPP calibration\n");
		return NOTIFY_DONE;
	}

	ret = mv310_set_opps_voltage(cpu, s);
	if (ret) {
		dev_err(s->dev, "failed to calibrate CPU OPP voltages: %d\n", ret);
		return NOTIFY_DONE;
	}

	ret = mv310_apply_cpu_vote(s);
	if (ret) {
		dev_err(s->dev, "failed to apply calibrated CPU voltage: %d\n", ret);
		return NOTIFY_DONE;
	}

	dev_info(s->dev, "CPU OPP voltages calibrated before policy startup\n");
	return NOTIFY_OK;
}

static void mv310_unregister_cpufreq_notifier(void *data)
{
	cpufreq_unregister_notifier(data, CPUFREQ_POLICY_NOTIFIER);
}

static int mv310_supply_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config config = {
		.dev = dev,
		.of_node = dev->of_node,
	};
	struct device *cpu;
	struct mv310_supply *s;
	struct regulator_dev *rdev;
	u32 pwm;
	int boot_uv, ret, i;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->dev = dev;
	mutex_init(&s->lock);
	s->pwm = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(s->pwm))
		return PTR_ERR(s->pwm);
	s->cpu_clk = devm_clk_get(dev, "cpu");
	if (IS_ERR(s->cpu_clk))
		return dev_err_probe(dev, PTR_ERR(s->cpu_clk), "CPU clock\n");
	pwm = readl(s->pwm);
	boot_uv = mv310_pwm_to_uv(pwm);
	if (boot_uv < MV310_MIN_UV || boot_uv > MV310_MAX_UV)
		return -ERANGE;
	cpu = get_cpu_device(0);
	if (!cpu)
		return -EPROBE_DEFER;
	ret = mv310_load_calibration(dev, s);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret)
		dev_warn(dev, "CPU calibration unavailable (%d); using %d mV\n",
			 ret, MV310_MAX_UV / 1000);

	/* Keep both consumers at the ceiling until their OPP drivers vote. */
	s->vote[MV310_CPU] = MV310_MAX_UV;
	s->vote[MV310_GPU] = MV310_MAX_UV;
	ret = mv310_write_uv(s, MV310_MAX_UV);
	if (ret)
		return ret;

	s->cpufreq_nb.notifier_call = mv310_cpufreq_policy_notify;
	ret = cpufreq_register_notifier(&s->cpufreq_nb,
					CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		return dev_err_probe(dev, ret, "CPUFreq policy notifier\n");
	ret = devm_add_action_or_reset(dev, mv310_unregister_cpufreq_notifier,
				       &s->cpufreq_nb);
	if (ret)
		return ret;

	config.driver_data = s;
	for (i = 0; i < ARRAY_SIZE(mv310_supplies); i++) {
		rdev = devm_regulator_register(dev, &mv310_supplies[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev), "supply %d\n", i);
	}
	return 0;
}

static const struct of_device_id mv310_supply_match[] = {
	{ .compatible = "hisilicon,hi3798mv310-pwm-regulator" },
	{ }
};

static struct platform_driver mv310_supply_driver = {
	.probe = mv310_supply_probe,
	.driver = {
		.name = "hi3798mv310-shared-supply",
		.of_match_table = mv310_supply_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(mv310_supply_driver);
MODULE_DESCRIPTION("Hi3798MV310 shared CPU/GPU supply with calibrated CPU voltage");
MODULE_LICENSE("GPL");
