// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hi3798CV200 CPU HPM feedback.
 *
 * Only register fields defined by the Hi3798CV200 PM mechanism are used.
 * Hardware AVS remains disabled because
 * PMC218's controller parameters are not described by the device tree, so a
 * bounded software loop applies HPM feedback through the regulator framework
 * while the hardware controller remains disabled.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

#include "hi3798cv200-avs.h"

#define HI3798CV200_PMC30		0x0
#define HI3798CV200_PMC31		0x4
#define HI3798CV200_PMC32		0x8
#define HI3798CV200_PMC33		0xc

#define HI3798CV200_HPM_DIV_MASK	GENMASK(5, 0)
#define HI3798CV200_HPM_ENABLE		BIT(24)
#define HI3798CV200_HPM_MONITOR_ENABLE	BIT(26)
#define HI3798CV200_HPM_RECORD_MASK	GENMASK(9, 0)
#define HI3798CV200_HPM_RECORD1_SHIFT	12
#define HI3798CV200_HPM_TARGET_LOW_MASK	GENMASK(9, 0)
#define HI3798CV200_HPM_TARGET_MASK	GENMASK(21, 12)
#define HI3798CV200_HPM_PERIOD_MASK	GENMASK(31, 24)
#define HI3798CV200_HPM_PERIOD_2MS	BIT(24)

#define HI3798CV200_AVS_HW_ENABLE	BIT(0)
#define HI3798CV200_AVS_INTERVAL_MS	10
#define HI3798CV200_AVS_STABLE_MS	8
#define HI3798CV200_AVS_IDLE_MS		100
#define HI3798CV200_AVS_STEP_UV		10000
#define HI3798CV200_AVS_MAX_TEMP_MC	95000
#define HI3798CV200_AVS_POLICY_CELLS	3
#define HI3798CV200_AVS_MAX_POLICIES	16

struct hi3798cv200_avs_policy {
	unsigned long rate;
	u16 hpm_target;
	bool enable_avs;
};

struct hi3798cv200_cpu_avs {
	struct device *dev;
	void __iomem *hpm;
	void __iomem *control;
	struct delayed_work work;
	/* Serializes transitions, delayed HPM sampling and fail-safe recovery. */
	struct mutex lock;
	struct regulator *regulator;
	struct thermal_zone_device *thermal;
	struct hi3798cv200_avs_policy *policies;
	unsigned int num_policies;
	unsigned long min_uv;
	unsigned long safe_uv;
	u16 target;
	bool enabled;
};

static const struct hi3798cv200_avs_policy *
hi3798cv200_avs_find_policy(struct hi3798cv200_cpu_avs *avs,
			    unsigned long rate)
{
	int i;

	for (i = 0; i < avs->num_policies; i++)
		if (avs->policies[i].rate == rate)
			return &avs->policies[i];

	return NULL;
}

static int hi3798cv200_avs_parse_policies(struct hi3798cv200_cpu_avs *avs,
					  struct device_node *np)
{
	u32 *values;
	int count;
	int ret;
	int i;

	count = of_property_count_u32_elems(np, "hisilicon,hpm-targets");
	if (count <= 0 || count % HI3798CV200_AVS_POLICY_CELLS ||
	    count > HI3798CV200_AVS_MAX_POLICIES * HI3798CV200_AVS_POLICY_CELLS)
		return -EINVAL;

	values = kcalloc(count, sizeof(*values), GFP_KERNEL);
	if (!values)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "hisilicon,hpm-targets", values,
					 count);
	if (ret)
		goto free_values;

	avs->num_policies = count / HI3798CV200_AVS_POLICY_CELLS;
	avs->policies = kcalloc(avs->num_policies, sizeof(*avs->policies),
				GFP_KERNEL);
	if (!avs->policies) {
		ret = -ENOMEM;
		goto free_values;
	}

	for (i = 0; i < avs->num_policies; i++) {
		u32 rate = values[i * HI3798CV200_AVS_POLICY_CELLS];
		u32 target = values[i * HI3798CV200_AVS_POLICY_CELLS + 1];
		u32 enable = values[i * HI3798CV200_AVS_POLICY_CELLS + 2];

		if (!rate || target > FIELD_MAX(HI3798CV200_HPM_TARGET_MASK) - 6 ||
		    enable > 1 || (i && rate <= avs->policies[i - 1].rate)) {
			ret = -EINVAL;
			goto free_policies;
		}

		avs->policies[i].rate = rate;
		avs->policies[i].hpm_target = target;
		avs->policies[i].enable_avs = enable;
	}

	kfree(values);
	return 0;

free_policies:
	kfree(avs->policies);
	avs->policies = NULL;
	avs->num_policies = 0;
free_values:
	kfree(values);
	return ret;
}

static int hi3798cv200_avs_disable_hw(struct hi3798cv200_cpu_avs *avs)
{
	u32 value;

	value = readl(avs->control);
	value &= ~HI3798CV200_AVS_HW_ENABLE;
	writel(value, avs->control);

	return readl(avs->control) & HI3798CV200_AVS_HW_ENABLE ? -EIO : 0;
}

static int hi3798cv200_avs_set_voltage(struct hi3798cv200_cpu_avs *avs,
				       unsigned long uv)
{
	int actual;
	int ret;

	if (!avs->regulator || uv < avs->min_uv || uv > avs->safe_uv)
		return -ERANGE;

	ret = regulator_set_voltage(avs->regulator, uv, uv);
	if (ret)
		return ret;

	actual = regulator_get_voltage(avs->regulator);
	if (actual < 0)
		return actual;

	return actual == uv ? 0 : -EIO;
}

static int
hi3798cv200_avs_restore_safe_locked(struct hi3798cv200_cpu_avs *avs)
{
	int disable_ret;
	int voltage_ret;

	avs->enabled = false;
	disable_ret = hi3798cv200_avs_disable_hw(avs);
	voltage_ret = avs->regulator ?
		hi3798cv200_avs_set_voltage(avs, avs->safe_uv) : -ENODEV;
	if (disable_ret)
		dev_err_ratelimited(avs->dev,
				    "CPU hardware AVS could not be disabled: %d\n",
				    disable_ret);
	if (voltage_ret)
		dev_err_ratelimited(avs->dev,
				    "CPU AVS could not restore %lu uV: %d\n",
				    avs->safe_uv, voltage_ret);

	return disable_ret ?: voltage_ret;
}

static int hi3798cv200_avs_temperature(struct hi3798cv200_cpu_avs *avs,
				       int *temperature)
{
	if (!avs->thermal) {
		avs->thermal = thermal_zone_get_zone_by_name("soc-thermal");
		if (IS_ERR(avs->thermal)) {
			int ret = PTR_ERR(avs->thermal);

			avs->thermal = NULL;
			return ret;
		}
	}

	return thermal_zone_get_temp(avs->thermal, temperature);
}

static int hi3798cv200_avs_read_hpm(struct hi3798cv200_cpu_avs *avs,
				    unsigned int *average)
{
	u32 value;
	unsigned int sum = 0;
	int i;

	for (i = 0; i < 2; i++) {
		value = readl(avs->hpm + HI3798CV200_PMC31);
		sum += FIELD_GET(HI3798CV200_HPM_RECORD_MASK, value);
		sum += FIELD_GET(HI3798CV200_HPM_RECORD_MASK,
				 value >> HI3798CV200_HPM_RECORD1_SHIFT);

		value = readl(avs->hpm + HI3798CV200_PMC32);
		sum += FIELD_GET(HI3798CV200_HPM_RECORD_MASK, value);
		sum += FIELD_GET(HI3798CV200_HPM_RECORD_MASK,
				 value >> HI3798CV200_HPM_RECORD1_SHIFT);

		if (!i)
			usleep_range(4000, 4500);
	}

	*average = sum / 8;
	return *average ? 0 : -EIO;
}

static int hi3798cv200_avs_program_hpm(struct hi3798cv200_cpu_avs *avs,
				       unsigned long rate, u16 target)
{
	unsigned long rate_mhz = rate / 1000000;
	u32 divider;
	u32 value;

	if (rate_mhz < 50 || rate_mhz % 50)
		return -EINVAL;

	divider = rate_mhz / 50 - 1;
	if (!FIELD_FIT(HI3798CV200_HPM_DIV_MASK, divider) || target > 0x3ff ||
	    target + 6 > 0x3ff)
		return -ERANGE;

	value = readl(avs->hpm + HI3798CV200_PMC30);
	value &= ~HI3798CV200_HPM_DIV_MASK;
	value |= FIELD_PREP(HI3798CV200_HPM_DIV_MASK, divider) |
		 HI3798CV200_HPM_ENABLE | HI3798CV200_HPM_MONITOR_ENABLE;
	writel(value, avs->hpm + HI3798CV200_PMC30);
	value = readl(avs->hpm + HI3798CV200_PMC30);
	if (FIELD_GET(HI3798CV200_HPM_DIV_MASK, value) != divider ||
	    !(value & HI3798CV200_HPM_ENABLE) ||
	    !(value & HI3798CV200_HPM_MONITOR_ENABLE))
		return -EIO;

	value = readl(avs->hpm + HI3798CV200_PMC33);
	value &= ~HI3798CV200_HPM_PERIOD_MASK;
	value |= HI3798CV200_HPM_PERIOD_2MS;
	writel(value, avs->hpm + HI3798CV200_PMC33);

	value = readl(avs->hpm + HI3798CV200_PMC33);
	value &= ~(HI3798CV200_HPM_TARGET_LOW_MASK |
		   HI3798CV200_HPM_TARGET_MASK);
	value |= FIELD_PREP(HI3798CV200_HPM_TARGET_LOW_MASK, target + 6) |
		 FIELD_PREP(HI3798CV200_HPM_TARGET_MASK, target);
	writel(value, avs->hpm + HI3798CV200_PMC33);
	value = readl(avs->hpm + HI3798CV200_PMC33);
	if (FIELD_GET(HI3798CV200_HPM_PERIOD_MASK, value) != 1 ||
	    FIELD_GET(HI3798CV200_HPM_TARGET_LOW_MASK, value) != target + 6 ||
	    FIELD_GET(HI3798CV200_HPM_TARGET_MASK, value) != target)
		return -EIO;

	return 0;
}

static void hi3798cv200_avs_work(struct work_struct *work)
{
	struct hi3798cv200_cpu_avs *avs =
		container_of(to_delayed_work(work),
			     struct hi3798cv200_cpu_avs, work);
	unsigned long delay_ms = HI3798CV200_AVS_INTERVAL_MS;
	unsigned long next_uv;
	unsigned int hpm;
	int temperature = -1;
	int voltage = -1;
	int ret;

	mutex_lock(&avs->lock);
	if (!avs->enabled)
		goto unlock;

	ret = hi3798cv200_avs_temperature(avs, &temperature);
	if (ret == -ENODEV || ret == -EPROBE_DEFER) {
		/* The thermal provider can probe after cpufreq. Keep AVS bounded. */
		schedule_delayed_work(&avs->work, msecs_to_jiffies(100));
		goto unlock;
	}
	if (ret)
		goto fault;
	if (temperature >= HI3798CV200_AVS_MAX_TEMP_MC) {
		ret = -EOVERFLOW;
		goto fault;
	}

	voltage = regulator_get_voltage(avs->regulator);
	if (voltage < avs->min_uv || voltage > avs->safe_uv) {
		ret = -ERANGE;
		goto fault;
	}

	ret = hi3798cv200_avs_read_hpm(avs, &hpm);
	if (ret)
		goto fault;

	next_uv = voltage;
	if (hpm <= avs->target + 1)
		next_uv = min_t(unsigned long, voltage + HI3798CV200_AVS_STEP_UV,
				avs->safe_uv);
	else if (hpm >= avs->target + 10)
		next_uv = max_t(unsigned long, voltage - HI3798CV200_AVS_STEP_UV,
				avs->min_uv);
	else
		delay_ms = HI3798CV200_AVS_IDLE_MS;

	if (next_uv != voltage) {
		ret = hi3798cv200_avs_set_voltage(avs, next_uv);
		if (ret)
			goto fault;
	}

	schedule_delayed_work(&avs->work, msecs_to_jiffies(delay_ms));
	goto unlock;

fault:
	hi3798cv200_avs_restore_safe_locked(avs);
	dev_err_ratelimited(avs->dev,
			    "CPU AVS stopped: temp=%d mC voltage=%d uV ret=%d\n",
			    temperature, voltage, ret);
unlock:
	mutex_unlock(&avs->lock);
}

struct hi3798cv200_cpu_avs *
hi3798cv200_cpu_avs_get(struct device *cpu_dev)
{
	struct hi3798cv200_cpu_avs *avs;
	struct device_node *np;
	int ret;

	np = of_find_compatible_node(NULL, NULL,
				     "hisilicon,hi3798cv200-avs");
	if (!np)
		return ERR_PTR(-ENODEV);

	avs = kzalloc(sizeof(*avs), GFP_KERNEL);
	if (!avs) {
		of_node_put(np);
		return ERR_PTR(-ENOMEM);
	}
	avs->dev = cpu_dev;
	mutex_init(&avs->lock);
	INIT_DELAYED_WORK(&avs->work, hi3798cv200_avs_work);
	ret = hi3798cv200_avs_parse_policies(avs, np);
	if (ret) {
		of_node_put(np);
		hi3798cv200_cpu_avs_put(avs);
		return ERR_PTR(ret);
	}

	avs->hpm = of_iomap(np, 0);
	avs->control = of_iomap(np, 1);
	of_node_put(np);
	if (!avs->hpm || !avs->control) {
		hi3798cv200_cpu_avs_put(avs);
		return ERR_PTR(-ENOMEM);
	}

	if (hi3798cv200_avs_disable_hw(avs)) {
		hi3798cv200_cpu_avs_put(avs);
		return ERR_PTR(-EIO);
	}

	return avs;
}
EXPORT_SYMBOL_GPL(hi3798cv200_cpu_avs_get);

void hi3798cv200_cpu_avs_put(struct hi3798cv200_cpu_avs *avs)
{
	if (!avs || IS_ERR(avs))
		return;

	cancel_delayed_work_sync(&avs->work);
	if (avs->control)
		hi3798cv200_avs_disable_hw(avs);
	iounmap(avs->control);
	iounmap(avs->hpm);
	kfree(avs->policies);
	kfree(avs);
}
EXPORT_SYMBOL_GPL(hi3798cv200_cpu_avs_put);

int hi3798cv200_cpu_avs_prepare(struct hi3798cv200_cpu_avs *avs)
{
	int ret;

	cancel_delayed_work_sync(&avs->work);
	mutex_lock(&avs->lock);
	avs->enabled = false;
	ret = hi3798cv200_avs_disable_hw(avs);
	mutex_unlock(&avs->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(hi3798cv200_cpu_avs_prepare);

int hi3798cv200_cpu_avs_complete(struct hi3798cv200_cpu_avs *avs,
				 struct regulator *regulator,
				 unsigned long rate,
				 unsigned long min_uv,
				 unsigned long safe_uv)
{
	const struct hi3798cv200_avs_policy *policy;
	unsigned int hpm;
	int temperature;
	int voltage;
	int failure = 0;
	int ret;

	policy = hi3798cv200_avs_find_policy(avs, rate);
	if (!policy || !regulator || min_uv > safe_uv)
		return -EINVAL;

	cancel_delayed_work_sync(&avs->work);
	mutex_lock(&avs->lock);
	avs->regulator = regulator;
	avs->min_uv = min_uv;
	avs->safe_uv = safe_uv;
	avs->target = policy->hpm_target;
	avs->enabled = false;

	ret = hi3798cv200_avs_disable_hw(avs);
	if (ret)
		goto safe;

	ret = hi3798cv200_avs_program_hpm(avs, rate, policy->hpm_target);
	if (ret)
		goto safe;

	usleep_range(HI3798CV200_AVS_STABLE_MS * USEC_PER_MSEC,
		     HI3798CV200_AVS_STABLE_MS * USEC_PER_MSEC + 500);
	ret = hi3798cv200_avs_read_hpm(avs, &hpm);
	if (ret)
		goto safe;

	ret = hi3798cv200_avs_temperature(avs, &temperature);
	if (ret == -ENODEV || ret == -EPROBE_DEFER) {
		if (!policy->enable_avs) {
			ret = hi3798cv200_avs_set_voltage(avs, min_uv);
			if (ret)
				goto safe;
			goto out;
		}
		/* Retry once the thermal zone is registered, while staying safe. */
		ret = hi3798cv200_avs_restore_safe_locked(avs);
		if (!ret) {
			avs->enabled = true;
			schedule_delayed_work(&avs->work,
					      msecs_to_jiffies(100));
			goto out;
		}
		goto safe;
	}
	if (ret)
		goto safe;
	if (temperature >= HI3798CV200_AVS_MAX_TEMP_MC) {
		ret = -EOVERFLOW;
		goto safe;
	}

	if (!policy->enable_avs) {
		ret = hi3798cv200_avs_set_voltage(avs, min_uv);
		if (ret)
			goto safe;
		goto out;
	}

	voltage = regulator_get_voltage(avs->regulator);
	if (voltage < min_uv) {
		ret = -ERANGE;
		goto safe;
	}
	if (voltage > safe_uv) {
		ret = hi3798cv200_avs_set_voltage(avs, safe_uv);
		if (ret)
			goto safe;
	}

	avs->enabled = true;
	schedule_delayed_work(&avs->work,
			      msecs_to_jiffies(HI3798CV200_AVS_INTERVAL_MS));
	ret = 0;
	goto out;

safe:
	failure = ret;
	ret = hi3798cv200_avs_restore_safe_locked(avs);
	if (ret)
		dev_err_ratelimited(avs->dev,
				    "CPU AVS safe restore failed at %lu Hz: %d (original %d)\n",
				    rate, ret, failure);
	else
		dev_warn_ratelimited(avs->dev,
				     "CPU AVS disabled at %lu Hz after error %d\n",
				     rate, failure);
out:
	mutex_unlock(&avs->lock);
	/* Preserve the transition/setup failure even if recovery succeeded. */
	return failure ?: ret;
}
EXPORT_SYMBOL_GPL(hi3798cv200_cpu_avs_complete);

void hi3798cv200_cpu_avs_fault(struct hi3798cv200_cpu_avs *avs)
{
	cancel_delayed_work_sync(&avs->work);
	mutex_lock(&avs->lock);
	avs->enabled = false;
	if (hi3798cv200_avs_disable_hw(avs))
		dev_err_ratelimited(avs->dev,
				    "CPU hardware AVS could not be disabled\n");

	/* No voltage guess is safe before a verified OPP policy is established. */
	if (avs->regulator && avs->safe_uv &&
	    hi3798cv200_avs_set_voltage(avs, avs->safe_uv))
		dev_err_ratelimited(avs->dev,
				    "CPU HPM could not restore %lu uV\n",
				    avs->safe_uv);
	mutex_unlock(&avs->lock);
}
EXPORT_SYMBOL_GPL(hi3798cv200_cpu_avs_fault);

MODULE_AUTHOR("HiSilicon Technologies Co., Ltd.");
MODULE_DESCRIPTION("Hi3798CV200 software HPM voltage feedback");
MODULE_LICENSE("GPL");
