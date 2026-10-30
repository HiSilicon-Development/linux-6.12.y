/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __CLK_HISTB_CPU_H
#define __CLK_HISTB_CPU_H
#include <linux/types.h>
struct clk;
struct device;
struct clk *hisi_histb_cpu_clk_register(struct device *dev, void __iomem *base,
				      const char *name, unsigned long max_rate);
#endif
