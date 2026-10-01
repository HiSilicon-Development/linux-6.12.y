// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon HiSTB architectural WFI CPU idle driver
 */

#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>

#include <asm/proc-fns.h>

static int histb_wfi_enter(struct cpuidle_device *dev,
			   struct cpuidle_driver *drv, int index)
{
	cpu_do_idle();
	return index;
}

static struct cpuidle_driver histb_wfi_driver = {
	.name = "histb-wfi",
	.owner = THIS_MODULE,
	.states = {
		{
			.enter = histb_wfi_enter,
			.exit_latency = 1,
			.target_residency = 1,
			.power_usage = UINT_MAX,
			.name = "WFI",
			.desc = "ARM WFI",
		},
	},
	.safe_state_index = 0,
	.state_count = 1,
};

static int __init histb_wfi_init(void)
{
	if (!of_machine_is_compatible("hisilicon,hi3798cv200"))
		return -ENODEV;

	return cpuidle_register(&histb_wfi_driver, NULL);
}
device_initcall(histb_wfi_init);
