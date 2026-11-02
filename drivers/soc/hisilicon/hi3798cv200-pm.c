// SPDX-License-Identifier: GPL-2.0-only
/* DVBIP CV200 resident-monitor deep suspend host (CVPM ABI 1). */
#include <linux/arm-smccc.h>
#include <linux/cpu.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/psci.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/sysfs.h>
#include <linux/timekeeping.h>
#include <uapi/linux/psci.h>

#include <asm/memory.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>

#define CVPM_FID		0xc2003798
#define CVPM_MAGIC		0x4356504d
#define CVPM_ABI		1
#define CVPM_QUERY		0
#define CVPM_CONFIG		1
#define CVPM_ENTER		2
#define CVPM_LAST_WAKE		3
#define CVPM_CANCEL		4
#define CVPM_WAKE_IR		BIT(0)
#define CVPM_WAKE_TIMEOUT	BIT(1)
#define CVPM_WAKE_MASK		(CVPM_WAKE_IR | CVPM_WAKE_TIMEOUT)
#define CVPM_TIMEOUT_DISABLED	U32_MAX

struct cvpm_config {
	u32 mask;
	u32 timeout;
	u32 ir_low;
	u32 ir_high;
};

struct cvpm_stats {
	u64 attempts;
	u64 resumes;
	u64 aborted;
	int last_error;
	int cancel_error;
	int wake_error;
	bool wake_valid;
	bool time_injected;
	u32 cause;
	u32 seconds;
	u32 ir_low;
	u32 ir_high;
};

struct cvpm {
	/* All state shared with sysfs is protected, including noirq updates. */
	spinlock_t lock;
	struct device *dev;
	u32 caps;
	struct cvpm_config config;
	struct cvpm_stats stats;
	bool active;
	bool prepared;
	bool resumed;
};

/* The suspend operations and their owner must outlive device unbinding. */
static struct cvpm cv200_pm = {
	.lock = __SPIN_LOCK_UNLOCKED(cv200_pm.lock),
};
static DEFINE_MUTEX(cvpm_probe_lock);

static void cvpm_call(u32 op, unsigned long x2, unsigned long x3,
		      unsigned long x4, unsigned long x5,
		      struct arm_smccc_1_2_regs *res)
{
	struct arm_smccc_1_2_regs args = {
		.a0 = CVPM_FID,
		.a1 = op,
		.a2 = x2,
		.a3 = x3,
		.a4 = x4,
		.a5 = x5,
	};

	/* The register block is local: no pointer is passed to the monitor. */
	arm_smccc_1_2_smc(&args, res);
}

static int cvpm_status(unsigned long status)
{
	if (!status)
		return 0;
	if (IS_ERR_VALUE(status))
		return (long)status;
	return -EPROTO;
}

static void cvpm_error(int error)
{
	unsigned long flags;

	if (!error)
		return;
	spin_lock_irqsave(&cv200_pm.lock, flags);
	if (!cv200_pm.stats.last_error)
		cv200_pm.stats.last_error = error;
	spin_unlock_irqrestore(&cv200_pm.lock, flags);
}

static int cvpm_validate(const struct cvpm_config *config, u32 caps)
{
	if (!config->mask || config->mask & ~CVPM_WAKE_MASK ||
	    config->mask & ~caps)
		return -EINVAL;
	if (config->mask & CVPM_WAKE_TIMEOUT) {
		if (!config->timeout || config->timeout == CVPM_TIMEOUT_DISABLED)
			return -ERANGE;
	} else if (config->timeout) {
		return -EINVAL;
	}
	return 0;
}

static int cvpm_cancel_prepared(void)
{
	struct arm_smccc_1_2_regs res;
	unsigned long flags;
	bool prepared;
	int error;

	spin_lock_irqsave(&cv200_pm.lock, flags);
	prepared = cv200_pm.prepared;
	spin_unlock_irqrestore(&cv200_pm.lock, flags);
	if (!prepared)
		return 0;

	cvpm_call(CVPM_CANCEL, 0, 0, 0, 0, &res);
	error = cvpm_status(res.a0);
	spin_lock_irqsave(&cv200_pm.lock, flags);
	cv200_pm.stats.cancel_error = error;
	if (!error)
		cv200_pm.prepared = false;
	spin_unlock_irqrestore(&cv200_pm.lock, flags);
	/* A failed CANCEL keeps configuration frozen, including after .end. */
	cvpm_error(error);
	if (error)
		dev_err(cv200_pm.dev, "firmware CANCEL failed: %d; configuration remains frozen\n",
			error);
	return error;
}

static void cvpm_cancel(void)
{
	cvpm_cancel_prepared();
}

static int cvpm_begin(suspend_state_t state)
{
	unsigned long flags;
	int error = 0;

	if (state != PM_SUSPEND_MEM)
		return -EINVAL;
	spin_lock_irqsave(&cv200_pm.lock, flags);
	if (cv200_pm.active || cv200_pm.prepared) {
		error = -EBUSY;
	} else {
		error = cvpm_validate(&cv200_pm.config, cv200_pm.caps);
		if (!error) {
			cv200_pm.active = true;
			cv200_pm.resumed = false;
			cv200_pm.stats.attempts++;
			cv200_pm.stats.last_error = 0;
			cv200_pm.stats.cancel_error = 0;
			cv200_pm.stats.wake_error = -ENODATA;
			cv200_pm.stats.wake_valid = false;
			cv200_pm.stats.time_injected = false;
		}
	}
	spin_unlock_irqrestore(&cv200_pm.lock, flags);
	return error;
}

static int cvpm_prepare(void)
{
	struct arm_smccc_1_2_regs res;
	struct cvpm_config config;
	unsigned long flags;
	int error;

	/* This runs before secondary CPU offlining, with interrupts enabled. */
	spin_lock_irqsave(&cv200_pm.lock, flags);
	config = cv200_pm.config;
	/* Even a partially failed CONFIG must be cancelled on the unwind. */
	cv200_pm.prepared = true;
	spin_unlock_irqrestore(&cv200_pm.lock, flags);
	cvpm_call(CVPM_CONFIG, config.ir_low, config.ir_high,
		  config.mask & CVPM_WAKE_TIMEOUT ? config.timeout :
		  CVPM_TIMEOUT_DISABLED, config.mask, &res);
	error = cvpm_status(res.a0);
	cvpm_error(error);
	return error;
}

static int cvpm_finisher(unsigned long unused)
{
	struct arm_smccc_1_2_regs res;

	cvpm_call(CVPM_ENTER, __pa_symbol(cpu_resume), 0, 0, 0, &res);
	/* Success can only return via cpu_resume, never through this SMC. */
	return res.a0 ? cvpm_status(res.a0) : -EPROTO;
}

static int cvpm_read_wake(void)
{
	struct arm_smccc_1_2_regs res;
	unsigned long flags;
	int error;

	/* Exactly one read per actual resume, before CANCEL or another CONFIG. */
	cvpm_call(CVPM_LAST_WAKE, 0, 0, 0, 0, &res);
	error = cvpm_status(res.a0);
	if (!error && (res.a1 > 5 || res.a2 > U32_MAX ||
		       res.a3 > U32_MAX || res.a4 > U32_MAX))
		error = -EPROTO;
	spin_lock_irqsave(&cv200_pm.lock, flags);
	cv200_pm.stats.wake_error = error;
	cv200_pm.stats.wake_valid = !error;
	if (!error) {
		cv200_pm.stats.cause = res.a1;
		cv200_pm.stats.seconds = res.a2;
		cv200_pm.stats.ir_low = res.a3;
		cv200_pm.stats.ir_high = res.a4;
	}
	spin_unlock_irqrestore(&cv200_pm.lock, flags);
	return error;
}

static int cvpm_enter(suspend_state_t state)
{
	unsigned long flags;
	bool prepared;
	int error;

	spin_lock_irqsave(&cv200_pm.lock, flags);
	prepared = cv200_pm.prepared;
	spin_unlock_irqrestore(&cv200_pm.lock, flags);
	if (state != PM_SUSPEND_MEM || !prepared) {
		error = -EINVAL;
		goto out;
	}
	/* PSCI, not this driver, owns secondary CPU hotplug and power-off. */
	if (smp_processor_id() != 0 || cpu_logical_map(0) != 0 ||
	    num_online_cpus() != 1) {
		error = -EBUSY;
		goto out;
	}
	/* System sleep's cpu_pm syscore callback already brackets this call
	 * with CPU and cluster notifications. They are not nestable.
	 */

	error = cpu_suspend(0, cvpm_finisher);
	/* Read firmware-owned wake registers before any MCU cancellation. */
	if (!error) {
		spin_lock_irqsave(&cv200_pm.lock, flags);
		cv200_pm.resumed = true;
		cv200_pm.stats.resumes++;
		spin_unlock_irqrestore(&cv200_pm.lock, flags);
		error = cvpm_read_wake();
	}
out:
	cvpm_error(error);
	/* Preserve the wake snapshot but propagate a failed disarm to PM core. */
	if (!error)
		error = cvpm_cancel_prepared();
	else
		cvpm_cancel_prepared();
	return error;
}

static void cvpm_wake(void)
{
	struct timespec64 delta = { };
	unsigned long flags;
	bool valid;

	spin_lock_irqsave(&cv200_pm.lock, flags);
	valid = cv200_pm.resumed && cv200_pm.stats.wake_valid;
	delta.tv_sec = cv200_pm.stats.seconds;
	spin_unlock_irqrestore(&cv200_pm.lock, flags);
	/*
	 * platform .wake runs after timekeeping's syscore resume and before
	 * device/RTC resume. Prefer the nonstop or persistent clock if it
	 * already accounted sleep; otherwise use the MCU's whole seconds.
	 * This API also tells RTC resume not to inject the interval again.
	 */
	if (valid && !timekeeping_rtc_skipresume()) {
		timekeeping_inject_sleeptime64(&delta);
		spin_lock_irqsave(&cv200_pm.lock, flags);
		cv200_pm.stats.time_injected = true;
		spin_unlock_irqrestore(&cv200_pm.lock, flags);
	}
	cvpm_cancel();
}

static void cvpm_end(void)
{
	unsigned long flags;

	/* Also handles failures before .prepare and PM debug/test unwinds. */
	cvpm_cancel();
	spin_lock_irqsave(&cv200_pm.lock, flags);
	if (cv200_pm.active && !cv200_pm.resumed)
		cv200_pm.stats.aborted++;
	cv200_pm.active = false;
	spin_unlock_irqrestore(&cv200_pm.lock, flags);
}

static const struct platform_suspend_ops cvpm_suspend_ops = {
	.valid = suspend_valid_only_mem,
	.begin = cvpm_begin,
	.prepare = cvpm_prepare,
	.enter = cvpm_enter,
	.wake = cvpm_wake,
	.finish = cvpm_cancel,
	.recover = cvpm_cancel,
	.end = cvpm_end,
};

/* Four base-0 numbers: enabled_mask timeout_seconds raw_ir_low raw_ir_high. */
static ssize_t wakeup_config_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct cvpm_config config;
	unsigned long flags;

	spin_lock_irqsave(&cv200_pm.lock, flags);
	config = cv200_pm.config;
	spin_unlock_irqrestore(&cv200_pm.lock, flags);
	return sysfs_emit(buf, "0x%x %u 0x%08x 0x%08x\n", config.mask,
			  config.timeout, config.ir_low, config.ir_high);
}

static ssize_t wakeup_config_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct cvpm_config config;
	unsigned long flags;
	char **args;
	int argc, error;

	args = argv_split(GFP_KERNEL, buf, &argc);
	if (!args)
		return -ENOMEM;
	if (argc != 4) {
		error = -EINVAL;
		goto free_args;
	}
	error = kstrtou32(args[0], 0, &config.mask);
	if (!error)
		error = kstrtou32(args[1], 0, &config.timeout);
	if (!error)
		error = kstrtou32(args[2], 0, &config.ir_low);
	if (!error)
		error = kstrtou32(args[3], 0, &config.ir_high);
	if (error)
		goto free_args;
	error = cvpm_validate(&config, cv200_pm.caps);
	if (error)
		goto free_args;

	spin_lock_irqsave(&cv200_pm.lock, flags);
	if (cv200_pm.active || cv200_pm.prepared)
		error = -EBUSY;
	else
		cv200_pm.config = config;
	spin_unlock_irqrestore(&cv200_pm.lock, flags);
free_args:
	argv_free(args);
	return error ? error : count;
}
static DEVICE_ATTR(wakeup_config, 0600, wakeup_config_show, wakeup_config_store);

static ssize_t last_wake_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	static const char * const causes[] = {
		"ir", "key", "timeout", "ethernet", "usb", "gpio",
	};
	struct cvpm_stats stats;
	unsigned long flags;

	spin_lock_irqsave(&cv200_pm.lock, flags);
	stats = cv200_pm.stats;
	spin_unlock_irqrestore(&cv200_pm.lock, flags);
	if (!stats.wake_valid)
		return sysfs_emit(buf, "unavailable error=%d\n", stats.wake_error);
	return sysfs_emit(buf, "cause=%u (%s) seconds=%u ir_low=0x%08x ir_high=0x%08x\n",
			  stats.cause, causes[stats.cause], stats.seconds,
			  stats.ir_low, stats.ir_high);
}
static DEVICE_ATTR(last_wake, 0400, last_wake_show, NULL);

static ssize_t stats_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct cvpm_stats stats;
	unsigned long flags;
	bool active, prepared;

	spin_lock_irqsave(&cv200_pm.lock, flags);
	stats = cv200_pm.stats;
	active = cv200_pm.active;
	prepared = cv200_pm.prepared;
	spin_unlock_irqrestore(&cv200_pm.lock, flags);
	return sysfs_emit(buf,
		"attempts=%llu resumes=%llu aborted=%llu active=%u armed=%u last_error=%d cancel_error=%d wake_error=%d time_injected=%u\n",
		stats.attempts, stats.resumes, stats.aborted, active, prepared,
		stats.last_error, stats.cancel_error, stats.wake_error,
		stats.time_injected);
}
static DEVICE_ATTR(stats, 0400, stats_show, NULL);

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return sysfs_emit(buf, "abi=%u capabilities=0x%x\n", CVPM_ABI,
			  cv200_pm.caps);
}
static DEVICE_ATTR(firmware, 0400, firmware_show, NULL);

static struct attribute *cvpm_attrs[] = {
	&dev_attr_wakeup_config.attr,
	&dev_attr_last_wake.attr,
	&dev_attr_stats.attr,
	&dev_attr_firmware.attr,
	NULL,
};
ATTRIBUTE_GROUPS(cvpm);

static int cvpm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct arm_smccc_1_2_regs res;
	struct arm_smccc_res psci_res;
	struct cvpm_config config = {
		.mask = CVPM_WAKE_TIMEOUT,
		.timeout = 30,
	};
	u32 ir[2];
	int error;

	if (!of_machine_is_compatible("hisilicon,hi3798cv200") ||
	    !of_property_read_bool(dev->of_node, "system-power-controller"))
		return -ENODEV;
	if (arm_smccc_1_1_get_conduit() != SMCCC_CONDUIT_SMC ||
	    !psci_ops.cpu_on || !psci_ops.cpu_off || !psci_ops.affinity_info)
		return dev_err_probe(dev, -EOPNOTSUPP, "SMC PSCI CPU lifecycle required\n");

	mutex_lock(&cvpm_probe_lock);
	if (cv200_pm.dev) {
		error = -EBUSY;
		goto unlock;
	}
	cvpm_call(CVPM_QUERY, 0, 0, 0, 0, &res);
	if (res.a0 != CVPM_MAGIC || res.a1 != CVPM_ABI ||
	    !res.a2 || res.a2 & ~CVPM_WAKE_MASK || res.a3 || res.a4) {
		error = dev_err_probe(dev, -EOPNOTSUPP, "CVPM ABI 1 monitor unavailable\n");
		goto unlock;
	}
	cv200_pm.caps = res.a2;
	/* Never displace the standard PSCI SYSTEM_SUSPEND implementation. */
	arm_smccc_smc(PSCI_1_0_FN_PSCI_FEATURES,
		      PSCI_1_0_FN64_SYSTEM_SUSPEND, 0, 0, 0, 0, 0, 0,
		      &psci_res);
	if ((int)psci_res.a0 != PSCI_RET_NOT_SUPPORTED) {
		error = dev_err_probe(dev, -EBUSY, "PSCI owns system suspend\n");
		goto unlock;
	}
	if (of_find_property(dev->of_node, "hisilicon,wakeup-timeout-sec", NULL)) {
		error = of_property_read_u32(dev->of_node,
					     "hisilicon,wakeup-timeout-sec",
					     &config.timeout);
		if (error)
			goto unlock;
		if (!config.timeout)
			config.mask &= ~CVPM_WAKE_TIMEOUT;
	}
	if (of_find_property(dev->of_node, "hisilicon,ir-power-key", NULL)) {
		if (of_property_count_u32_elems(dev->of_node,
						"hisilicon,ir-power-key") != 2) {
			error = -EINVAL;
			goto unlock;
		}
		error = of_property_read_u32_array(dev->of_node,
						   "hisilicon,ir-power-key", ir, 2);
		if (error)
			goto unlock;
		config.ir_low = ir[0];
		config.ir_high = ir[1];
		config.mask |= CVPM_WAKE_IR;
	}
	error = cvpm_validate(&config, cv200_pm.caps);
	if (error) {
		dev_err(dev, "invalid or unsupported wake configuration: %d\n", error);
		goto unlock;
	}
	cv200_pm.config = config;
	cv200_pm.stats.wake_error = -ENODATA;
	cv200_pm.dev = dev;
	error = sysfs_create_groups(&dev->kobj, cvpm_groups);
	if (error)
		goto clear_owner;
	/* Atomic ownership acquisition: unknown platform operations stay intact. */
	error = suspend_set_ops_if_unset(&cvpm_suspend_ops);
	if (error) {
		sysfs_remove_groups(&dev->kobj, cvpm_groups);
		goto clear_owner;
	}
	/* No remove/module-exit path: global platform callbacks are permanent. */
	get_device(dev);
	platform_set_drvdata(pdev, &cv200_pm);
	dev_info(dev, "CVPM ABI 1 deep suspend, wake mask %#x, timeout %u s\n",
		 config.mask, config.timeout);
	goto unlock;
clear_owner:
	cv200_pm.dev = NULL;
unlock:
	mutex_unlock(&cvpm_probe_lock);
	return error;
}

static const struct of_device_id cvpm_of_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-pm" },
	{ }
};

static struct platform_driver cvpm_driver = {
	.probe = cvpm_probe,
	.driver = {
		.name = "hi3798cv200-pm",
		.of_match_table = cvpm_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(cvpm_driver);
MODULE_DESCRIPTION("DVBIP CV200 resident-monitor deep suspend");
MODULE_LICENSE("GPL");
