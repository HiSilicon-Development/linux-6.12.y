/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __HI3798CV200_AVS_H
#define __HI3798CV200_AVS_H

#include <linux/err.h>
#include <linux/kconfig.h>

struct device;
struct regulator;

struct hi3798cv200_cpu_avs;

#if IS_ENABLED(CONFIG_ARM_HISTB_CPUFREQ)
struct hi3798cv200_cpu_avs *
hi3798cv200_cpu_avs_get(struct device *cpu_dev);
void hi3798cv200_cpu_avs_put(struct hi3798cv200_cpu_avs *avs);

int hi3798cv200_cpu_avs_prepare(struct hi3798cv200_cpu_avs *avs);
int hi3798cv200_cpu_avs_complete(struct hi3798cv200_cpu_avs *avs,
				 struct regulator *regulator,
				 unsigned long rate,
				 unsigned long min_uv,
				 unsigned long safe_uv);
void hi3798cv200_cpu_avs_fault(struct hi3798cv200_cpu_avs *avs);
#else
static inline struct hi3798cv200_cpu_avs *
hi3798cv200_cpu_avs_get(struct device *cpu_dev)
{
	return ERR_PTR(-ENODEV);
}

static inline void
hi3798cv200_cpu_avs_put(struct hi3798cv200_cpu_avs *avs)
{
}

static inline int
hi3798cv200_cpu_avs_prepare(struct hi3798cv200_cpu_avs *avs)
{
	return -ENODEV;
}

static inline int
hi3798cv200_cpu_avs_complete(struct hi3798cv200_cpu_avs *avs,
			     struct regulator *regulator,
			     unsigned long rate, unsigned long min_uv,
			     unsigned long safe_uv)
{
	return -ENODEV;
}

static inline void
hi3798cv200_cpu_avs_fault(struct hi3798cv200_cpu_avs *avs)
{
}
#endif

#endif
