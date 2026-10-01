// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Copyright (C) 2014 Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#include "cpufreq-dt.h"
#include "hi3798cv200-avs.h"

struct private_data {
	struct list_head node;

	cpumask_var_t cpus;
	struct device *cpu_dev;
	struct clk *verify_clk;
	struct regulator *cpu_regulator;
	struct hi3798cv200_cpu_avs *avs;
	struct dev_pm_opp_supply deferred_supply;
	struct cpufreq_frequency_table *freq_table;
	bool have_static_opps;
	bool voltage_deferred;
	int opp_token;
	int opp_clk_token;
};

static LIST_HEAD(priv_list);

static struct freq_attr *cpufreq_dt_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,   /* Extra space for boost-attr if required */
	NULL,
};

static struct private_data *cpufreq_dt_find_data(int cpu)
{
	struct private_data *priv;

	list_for_each_entry(priv, &priv_list, node) {
		if (cpumask_test_cpu(cpu, priv->cpus))
			return priv;
	}

	return NULL;
}

/* Pick the safest listed OPP when firmware leaves an in-between rate. */
static int cpufreq_dt_initial_index(struct cpufreq_policy *policy,
				    unsigned int actual_khz)
{
	struct cpufreq_frequency_table *pos;
	unsigned int idx, floor_idx = 0, floor_freq = 0;
	unsigned int lowest_idx = 0, lowest_freq = UINT_MAX;
	bool have_floor = false, have_lowest = false;

	cpufreq_for_each_valid_entry_idx(pos, policy->freq_table, idx) {
		if (!have_lowest || pos->frequency < lowest_freq) {
			lowest_idx = idx;
			lowest_freq = pos->frequency;
			have_lowest = true;
		}
		if (pos->frequency <= actual_khz &&
		    (!have_floor || pos->frequency > floor_freq)) {
			floor_idx = idx;
			floor_freq = pos->frequency;
			have_floor = true;
		}
	}

	if (have_floor)
		return floor_idx;
	return have_lowest ? lowest_idx : -EINVAL;
}

static int cpufreq_dt_config_regulators_verified(struct device *dev,
						 struct dev_pm_opp *old_opp,
					 struct dev_pm_opp *new_opp,
					 struct regulator **regulators,
					 unsigned int count)
{
	struct private_data *priv = cpufreq_dt_find_data(dev->id);
	struct dev_pm_opp_supply supply;
	unsigned long actual, target;
	int ret;

	if (!priv || count != 1 || !regulators || !regulators[0])
		return -EINVAL;

	/* AVS borrows the OPP table's sole regulator consumer. */
	priv->cpu_regulator = regulators[0];

	ret = dev_pm_opp_get_supplies(new_opp, &supply);
	if (ret)
		return ret;

	priv->voltage_deferred = false;
	actual = clk_get_rate(priv->verify_clk);
	target = dev_pm_opp_get_freq(new_opp);

	/*
	 * The boot APLL can be above the highest listed OPP. OPP core then has
	 * no matching old entry and treats the first correction as an increase.
	 * Preserve the firmware voltage until the real clock is at or below the
	 * requested rate; the clock callback completes this deferred change.
	 */
	if (actual > target) {
		priv->deferred_supply = supply;
		priv->voltage_deferred = true;
		return 0;
	}

	if (!priv->cpu_regulator)
		return -ENODEV;

	return regulator_set_voltage_triplet(priv->cpu_regulator,
					     supply.u_volt_min,
					     supply.u_volt,
					     supply.u_volt_max);
}

static int cpufreq_dt_config_clk_verified(struct device *dev,
					  struct opp_table *opp_table,
					  struct dev_pm_opp *opp,
					  void *data, bool scaling_down)
{
	struct private_data *priv = cpufreq_dt_find_data(dev->id);
	unsigned long *target = data;
	unsigned long actual;
	int ret;

	if (!priv || !priv->verify_clk || !target)
		return -EINVAL;

	ret = clk_set_rate(priv->verify_clk, *target);
	actual = clk_get_rate(priv->verify_clk);
	if (ret || actual != *target) {
		priv->voltage_deferred = false;
		dev_err_ratelimited(dev,
				    "CPU clock transition to %lu Hz failed: ret=%d actual=%lu Hz\n",
				    *target, ret, actual);
		return ret ?: -EIO;
	}

	if (priv->voltage_deferred) {
		ret = regulator_set_voltage_triplet(
			priv->cpu_regulator,
			priv->deferred_supply.u_volt_min,
			priv->deferred_supply.u_volt,
			priv->deferred_supply.u_volt_max);
		priv->voltage_deferred = false;
		if (ret) {
			dev_err(dev,
				"CPU clock reached %lu Hz but deferred voltage failed: %d\n",
				actual, ret);
			return ret;
		}
	}

	return 0;
}

static int set_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct private_data *priv = policy->driver_data;
	struct dev_pm_opp_supply supply;
	struct dev_pm_opp *opp;
	unsigned long actual;
	unsigned long freq = policy->freq_table[index].frequency * 1000;
	int ret;

	if (!priv->avs)
		return dev_pm_opp_set_rate(priv->cpu_dev, freq);

	ret = hi3798cv200_cpu_avs_prepare(priv->avs);
	if (ret)
		return ret;

	ret = dev_pm_opp_set_rate(priv->cpu_dev, freq);
	actual = clk_get_rate(priv->verify_clk);
	if (ret || actual != freq) {
		hi3798cv200_cpu_avs_fault(priv->avs);
		return ret ?: -EIO;
	}

	opp = dev_pm_opp_find_freq_exact(priv->cpu_dev, freq, true);
	if (IS_ERR(opp)) {
		ret = PTR_ERR(opp);
		goto fault;
	}

	ret = dev_pm_opp_get_supplies(opp, &supply);
	dev_pm_opp_put(opp);
	if (ret)
		goto fault;

	ret = hi3798cv200_cpu_avs_complete(priv->avs,
					   priv->cpu_regulator, actual,
					     supply.u_volt_min,
					     supply.u_volt);
	if (ret) {
		dev_err_ratelimited(priv->cpu_dev,
				    "CPU AVS setup failed at %lu Hz: %d; fail-safe engaged\n",
				    actual, ret);
	}

	return ret;

fault:
	hi3798cv200_cpu_avs_fault(priv->avs);
	return ret;
}

/*
 * An earlier version of opp-v1 bindings used to name the regulator
 * "cpu0-supply", we still need to handle that for backwards compatibility.
 */
static const char *find_supply_name(struct device *dev)
{
	struct device_node *np __free(device_node) = of_node_get(dev->of_node);
	int cpu = dev->id;

	/* This must be valid for sure */
	if (WARN_ON(!np))
		return NULL;

	/* Try "cpu0" for older DTs */
	if (!cpu && of_property_present(np, "cpu0-supply"))
		return "cpu0";

	if (of_property_present(np, "cpu-supply"))
		return "cpu";

	dev_dbg(dev, "no regulator for cpu%d\n", cpu);
	return NULL;
}

static int cpufreq_init(struct cpufreq_policy *policy)
{
	struct private_data *priv;
	struct device *cpu_dev;
	struct clk *cpu_clk;
	unsigned long actual;
	unsigned int transition_latency;
	int index;
	int ret;

	priv = cpufreq_dt_find_data(policy->cpu);
	if (!priv) {
		pr_err("failed to find data for cpu%d\n", policy->cpu);
		return -ENODEV;
	}
	cpu_dev = priv->cpu_dev;

	cpu_clk = clk_get(cpu_dev, NULL);
	if (IS_ERR(cpu_clk)) {
		ret = PTR_ERR(cpu_clk);
		dev_err(cpu_dev, "%s: failed to get clk: %d\n", __func__, ret);
		return ret;
	}

	transition_latency = dev_pm_opp_get_max_transition_latency(cpu_dev);
	if (!transition_latency)
		transition_latency = CPUFREQ_DEFAULT_TRANSITION_LATENCY_NS;

	cpumask_copy(policy->cpus, priv->cpus);
	policy->driver_data = priv;
	policy->clk = cpu_clk;
	policy->freq_table = priv->freq_table;
	policy->suspend_freq = dev_pm_opp_get_suspend_opp_freq(cpu_dev) / 1000;
	policy->cpuinfo.transition_latency = transition_latency;
	policy->dvfs_possible_from_any_cpu = true;

	/*
	 * Firmware may leave the CPU at a listed OPP, so cpufreq core's unknown
	 * initial-frequency correction does not call target_index(). Initialize
	 * the voltage/HPM policy explicitly before a governor starts.
	 */
	if (priv->avs) {
		actual = clk_get_rate(priv->verify_clk);
		if (!actual || actual % 1000) {
			ret = -EINVAL;
			dev_err(cpu_dev,
				"invalid initial CPU clock rate %lu Hz\n", actual);
			goto out_clk_put;
		}

		index = cpufreq_frequency_table_get_index(policy, actual / 1000);
		if (index < 0) {
			index = cpufreq_dt_initial_index(policy, actual / 1000);
			if (index < 0) {
				ret = index;
				dev_err(cpu_dev,
					"initial CPU clock rate %lu Hz has no usable OPP\n",
					actual);
				goto out_clk_put;
			}
			dev_warn(cpu_dev,
				 "clamping unlisted initial CPU rate %lu Hz to %u kHz before AVS\n",
				 actual, policy->freq_table[index].frequency);
		}

		ret = set_target(policy, index);
		if (ret) {
			dev_err(cpu_dev,
				"failed to initialize CPU voltage/HPM at %lu Hz: %d\n",
				actual, ret);
			goto out_clk_put;
		}
	}

	/* Support turbo/boost mode */
	if (policy_has_boost_freq(policy)) {
		/* This gets disabled by core on driver unregister */
		ret = cpufreq_enable_boost_support();
		if (ret)
			goto out_clk_put;
		cpufreq_dt_attr[1] = &cpufreq_freq_attr_scaling_boost_freqs;
	}

	return 0;

out_clk_put:
	clk_put(cpu_clk);

	return ret;
}

static int cpufreq_online(struct cpufreq_policy *policy)
{
	/* We did light-weight tear down earlier, nothing to do here */
	return 0;
}

static int cpufreq_offline(struct cpufreq_policy *policy)
{
	/*
	 * Preserve policy->driver_data and don't free resources on light-weight
	 * tear down.
	 */
	return 0;
}

static void cpufreq_exit(struct cpufreq_policy *policy)
{
	clk_put(policy->clk);
}

static struct cpufreq_driver dt_cpufreq_driver = {
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		 CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = set_target,
	.get = cpufreq_generic_get,
	.init = cpufreq_init,
	.exit = cpufreq_exit,
	.online = cpufreq_online,
	.offline = cpufreq_offline,
	.register_em = cpufreq_register_em_with_opp,
	.name = "cpufreq-dt",
	.attr = cpufreq_dt_attr,
	.suspend = cpufreq_generic_suspend,
};

static int dt_cpufreq_early_init(struct device *dev, int cpu)
{
	struct private_data *priv;
	struct device *cpu_dev;
	bool fallback = false;
	static const char * const single_clk[] = { NULL, NULL };
	const char *reg_name[] = { NULL, NULL };
	struct dev_pm_opp_config opp_config = {
		.clk_names = single_clk,
		.config_clks = cpufreq_dt_config_clk_verified,
		.config_regulators = cpufreq_dt_config_regulators_verified,
	};
	int ret;

	/* Check if this CPU is already covered by some other policy */
	if (cpufreq_dt_find_data(cpu))
		return 0;

	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev)
		return -EPROBE_DEFER;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (!zalloc_cpumask_var(&priv->cpus, GFP_KERNEL))
		return -ENOMEM;

	cpumask_set_cpu(cpu, priv->cpus);
	priv->cpu_dev = cpu_dev;

	/*
	 * The Hi3798CV200 APLL reports completion separately from the CCF
	 * set-rate return path. When an HPM node is present, verify the real clock
	 * before OPP is allowed to lower voltage.
	 */
	priv->avs = hi3798cv200_cpu_avs_get(cpu_dev);
	if (IS_ERR(priv->avs)) {
		ret = PTR_ERR(priv->avs);
		priv->avs = NULL;
		if (ret != -ENODEV) {
			ret = dev_err_probe(cpu_dev, ret,
					    "failed to initialize CPU HPM feedback\n");
			goto free_cpumask;
		}
	}

	if (priv->avs) {
		priv->verify_clk = clk_get(cpu_dev, NULL);
		if (IS_ERR(priv->verify_clk)) {
			ret = dev_err_probe(cpu_dev, PTR_ERR(priv->verify_clk),
					    "failed to get verification clock\n");
			priv->verify_clk = NULL;
			goto put_avs;
		}

		priv->opp_clk_token = dev_pm_opp_set_config(cpu_dev,
							    &opp_config);
		if (priv->opp_clk_token < 0) {
			ret = dev_err_probe(cpu_dev, priv->opp_clk_token,
					    "failed to set verified OPP clock\n");
			priv->opp_clk_token = 0;
			goto put_verify_clk;
		}
	}

	/*
	 * OPP layer will be taking care of regulators now, but it needs to know
	 * the name of the regulator first.
	 */
	reg_name[0] = find_supply_name(cpu_dev);
	if (reg_name[0]) {
		priv->opp_token = dev_pm_opp_set_regulators(cpu_dev, reg_name);
		if (priv->opp_token < 0) {
			ret = dev_err_probe(cpu_dev, priv->opp_token,
					    "failed to set regulators\n");
			priv->opp_token = 0;
			goto clear_opp_clk;
		}
	} else if (priv->avs) {
		ret = dev_err_probe(cpu_dev, -ENODEV,
				    "CPU supply is required for HPM feedback\n");
		goto clear_opp_clk;
	}

	/* Get OPP-sharing information from "operating-points-v2" bindings */
	ret = dev_pm_opp_of_get_sharing_cpus(cpu_dev, priv->cpus);
	if (ret) {
		if (ret != -ENOENT)
			goto out;

		/*
		 * operating-points-v2 not supported, fallback to all CPUs share
		 * OPP for backward compatibility if the platform hasn't set
		 * sharing CPUs.
		 */
		if (dev_pm_opp_get_sharing_cpus(cpu_dev, priv->cpus))
			fallback = true;
	}

	/*
	 * Initialize OPP tables for all priv->cpus. They will be shared by
	 * all CPUs which have marked their CPUs shared with OPP bindings.
	 *
	 * For platforms not using operating-points-v2 bindings, we do this
	 * before updating priv->cpus. Otherwise, we will end up creating
	 * duplicate OPPs for the CPUs.
	 *
	 * OPPs might be populated at runtime, don't fail for error here unless
	 * it is -EPROBE_DEFER.
	 */
	ret = dev_pm_opp_of_cpumask_add_table(priv->cpus);
	if (!ret) {
		priv->have_static_opps = true;
	} else if (ret == -EPROBE_DEFER) {
		goto out;
	}

	/*
	 * The OPP table must be initialized, statically or dynamically, by this
	 * point.
	 */
	ret = dev_pm_opp_get_opp_count(cpu_dev);
	if (ret <= 0) {
		dev_err(cpu_dev, "OPP table can't be empty\n");
		ret = -ENODEV;
		goto out;
	}

	if (fallback) {
		cpumask_setall(priv->cpus);
		ret = dev_pm_opp_set_sharing_cpus(cpu_dev, priv->cpus);
		if (ret)
			dev_err(cpu_dev, "%s: failed to mark OPPs as shared: %d\n",
				__func__, ret);
	}

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &priv->freq_table);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
		goto out;
	}

	list_add(&priv->node, &priv_list);
	return 0;

out:
	if (priv->have_static_opps)
		dev_pm_opp_of_cpumask_remove_table(priv->cpus);
	dev_pm_opp_put_regulators(priv->opp_token);
clear_opp_clk:
	if (priv->opp_clk_token)
		dev_pm_opp_clear_config(priv->opp_clk_token);
put_verify_clk:
	clk_put(priv->verify_clk);
put_avs:
	hi3798cv200_cpu_avs_put(priv->avs);
free_cpumask:
	free_cpumask_var(priv->cpus);
	return ret;
}

static void dt_cpufreq_release(void)
{
	struct private_data *priv, *tmp;

	list_for_each_entry_safe(priv, tmp, &priv_list, node) {
		dev_pm_opp_free_cpufreq_table(priv->cpu_dev, &priv->freq_table);
		if (priv->have_static_opps)
			dev_pm_opp_of_cpumask_remove_table(priv->cpus);
		hi3798cv200_cpu_avs_put(priv->avs);
		dev_pm_opp_put_regulators(priv->opp_token);
		if (priv->opp_clk_token)
			dev_pm_opp_clear_config(priv->opp_clk_token);
		clk_put(priv->verify_clk);
		free_cpumask_var(priv->cpus);
		list_del(&priv->node);
	}
}

static int dt_cpufreq_probe(struct platform_device *pdev)
{
	struct cpufreq_dt_platform_data *data = dev_get_platdata(&pdev->dev);
	int ret, cpu;

	/* Request resources early so we can return in case of -EPROBE_DEFER */
	for_each_possible_cpu(cpu) {
		ret = dt_cpufreq_early_init(&pdev->dev, cpu);
		if (ret)
			goto err;
	}

	if (data) {
		if (data->have_governor_per_policy)
			dt_cpufreq_driver.flags |= CPUFREQ_HAVE_GOVERNOR_PER_POLICY;

		dt_cpufreq_driver.resume = data->resume;
		if (data->suspend)
			dt_cpufreq_driver.suspend = data->suspend;
		if (data->get_intermediate) {
			dt_cpufreq_driver.target_intermediate = data->target_intermediate;
			dt_cpufreq_driver.get_intermediate = data->get_intermediate;
		}
	}

	ret = cpufreq_register_driver(&dt_cpufreq_driver);
	if (ret) {
		dev_err(&pdev->dev, "failed register driver: %d\n", ret);
		goto err;
	}

	return 0;
err:
	dt_cpufreq_release();
	return ret;
}

static void dt_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&dt_cpufreq_driver);
	dt_cpufreq_release();
}

static struct platform_driver dt_cpufreq_platdrv = {
	.driver = {
		.name	= "cpufreq-dt",
	},
	.probe		= dt_cpufreq_probe,
	.remove_new	= dt_cpufreq_remove,
};
module_platform_driver(dt_cpufreq_platdrv);

MODULE_ALIAS("platform:cpufreq-dt");
MODULE_AUTHOR("Viresh Kumar <viresh.kumar@linaro.org>");
MODULE_AUTHOR("Shawn Guo <shawn.guo@linaro.org>");
MODULE_DESCRIPTION("Generic cpufreq driver");
MODULE_LICENSE("GPL");
