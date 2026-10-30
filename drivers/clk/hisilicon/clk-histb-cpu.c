// SPDX-License-Identifier: GPL-2.0-or-later
/* Shared CV200/MV310 APLL tuning protocol, from the vendor PM BSP. */
#include <linux/device.h>
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include "clk-histb-cpu.h"

#define HISTB_CPU_PLL_CFG0		0x0
#define HISTB_CPU_PLL_CFG1		0x4
#define HISTB_CPU_CLK_CTRL		0x48
#define HISTB_CPU_CLK_SEL_MASK	GENMASK(2, 0)
#define HISTB_CPU_PLL_POSTDIV1_MASK GENMASK(26, 24)
#define HISTB_CPU_PLL_POSTDIV2_MASK GENMASK(30, 28)
#define HISTB_CPU_PLL_FBDIV_MASK	GENMASK(11, 0)
#define HISTB_CPU_PLL_REFDIV_MASK	GENMASK(17, 12)
#define HISTB_APLL_TUNE_INT	0x1a4
#define HISTB_APLL_TUNE_FRAC	0x1a8
#define HISTB_APLL_TUNE_STEP_INT	0x1ac
#define HISTB_APLL_TUNE_STEP_FRAC	0x1b0
#define HISTB_APLL_TUNE_CTRL	0x1b4
#define HISTB_APLL_TUNE_INT_MASK	GENMASK(11, 0)
#define HISTB_APLL_TUNE_FRAC_MASK	GENMASK(23, 0)
#define HISTB_APLL_TUNE_STEP_INT_MASK GENMASK(11, 0)
#define HISTB_APLL_TUNE_STEP_FRAC_MASK GENMASK(23, 0)
#define HISTB_APLL_TUNE_MODE	BIT(2)
#define HISTB_APLL_TUNE_ENABLE	BIT(3)
#define HISTB_APLL_TUNE_STATUS	0x294
#define HISTB_APLL_TUNE_RESULT_MASK GENMASK(11, 0)
#define HISTB_APLL_TUNE_BUSY	BIT(13)
#define HISTB_CPU_PLL_REF_RATE	24000000UL

struct histb_cpu_clk {
	struct clk_hw hw;
	void __iomem *base;
	struct device *dev;
	/* Serializes the shared APLL tuning transaction. */
	struct mutex lock;
	/* A failed transaction requires a reboot before another retune. */
	bool rate_fault;
	unsigned long max_rate;
};

static const unsigned long histb_cpu_rates[] = {
	400000000,
	600000000,
	800000000,
	1200000000,
	1600000000,
};

#define HISTB_CPU_RATE_TOLERANCE	10000000UL
#define HISTB_APLL_POLL_DELAY_US	10
#define HISTB_APLL_IDLE_TIMEOUT_US	1000
#define HISTB_APLL_TUNE_TIMEOUT_US	100000

#define to_histb_cpu_clk(_hw) \
	container_of(_hw, struct histb_cpu_clk, hw)

static int histb_cpu_pll_dividers(struct histb_cpu_clk *cpu,
					u32 *postdiv1, u32 *postdiv2,
					u32 *refdiv)
{
	u32 cfg0 = readl_relaxed(cpu->base + HISTB_CPU_PLL_CFG0);
	u32 cfg1 = readl_relaxed(cpu->base + HISTB_CPU_PLL_CFG1);

	*postdiv1 = FIELD_GET(HISTB_CPU_PLL_POSTDIV1_MASK, cfg0);
	*postdiv2 = FIELD_GET(HISTB_CPU_PLL_POSTDIV2_MASK, cfg0);
	*refdiv = FIELD_GET(HISTB_CPU_PLL_REFDIV_MASK, cfg1);

	return *postdiv1 && *postdiv2 && *refdiv ? 0 : -EINVAL;
}

static unsigned long
histb_cpu_rate_from_fbdiv(struct histb_cpu_clk *cpu, u32 fbdiv)
{
	u32 postdiv1, postdiv2, refdiv;
	unsigned long rate;
	u64 divisor;
	int i;

	if (!fbdiv ||
	    histb_cpu_pll_dividers(cpu, &postdiv1, &postdiv2, &refdiv))
		return 0;

	divisor = (u64)postdiv1 * postdiv2 * refdiv;
	rate = div64_u64((u64)fbdiv * HISTB_CPU_PLL_REF_RATE,
			 divisor);

	for (i = 0; i < ARRAY_SIZE(histb_cpu_rates); i++)
		if (abs_diff(rate, histb_cpu_rates[i]) <
		    HISTB_CPU_RATE_TOLERANCE)
			return histb_cpu_rates[i];

	return rate;
}

static int histb_cpu_apll_snapshot(struct histb_cpu_clk *cpu,
					 u32 *fbdiv,
					 unsigned long *rate)
{
	u32 status;

	status = readl_relaxed(cpu->base + HISTB_APLL_TUNE_STATUS);
	if (status & HISTB_APLL_TUNE_BUSY)
		return -EBUSY;

	*fbdiv = FIELD_GET(HISTB_APLL_TUNE_RESULT_MASK, status);
	if (!*fbdiv)
		*fbdiv = FIELD_GET(HISTB_CPU_PLL_FBDIV_MASK,
				   readl_relaxed(cpu->base +
						 HISTB_CPU_PLL_CFG1));

	*rate = histb_cpu_rate_from_fbdiv(cpu, *fbdiv);
	return *rate ? 0 : -EINVAL;
}

static int histb_cpu_apll_update_bits(
					struct histb_cpu_clk *cpu,
					u32 offset, u32 mask, u32 value,
					u32 *written)
{
	u32 reg;

	if (value & ~mask)
		return -EINVAL;

	reg = readl_relaxed(cpu->base + offset);
	reg = (reg & ~mask) | value;
	writel(reg, cpu->base + offset);

	/* The same-device read drains the posted write and verifies its image. */
	if (readl(cpu->base + offset) != reg)
		return -EIO;

	if (written)
		*written = reg;

	return 0;
}

static int histb_cpu_apll_tune(struct histb_cpu_clk *cpu,
				     u32 fbdiv, u32 *last_status)
{
	u32 ctrl, status;
	int ret;

	ret = readl_poll_timeout(cpu->base + HISTB_APLL_TUNE_STATUS,
				 status, !(status & HISTB_APLL_TUNE_BUSY),
				 HISTB_APLL_POLL_DELAY_US,
				 HISTB_APLL_IDLE_TIMEOUT_US);
	if (ret)
		goto out_status;

	/* Keep configuration fields inert while their new image is assembled. */
	ret = histb_cpu_apll_update_bits(
		cpu, HISTB_APLL_TUNE_CTRL,
		HISTB_APLL_TUNE_MODE | HISTB_APLL_TUNE_ENABLE,
		HISTB_APLL_TUNE_MODE, &ctrl);
	if (ret)
		goto out_status;

	ret = histb_cpu_apll_update_bits(
		cpu, HISTB_APLL_TUNE_INT,
		HISTB_APLL_TUNE_INT_MASK,
		FIELD_PREP(HISTB_APLL_TUNE_INT_MASK, fbdiv), NULL);
	if (ret)
		goto out_status;

	ret = histb_cpu_apll_update_bits(
		cpu, HISTB_APLL_TUNE_FRAC,
		HISTB_APLL_TUNE_FRAC_MASK, 0, NULL);
	if (ret)
		goto out_status;

	ret = histb_cpu_apll_update_bits(
		cpu, HISTB_APLL_TUNE_STEP_INT,
		HISTB_APLL_TUNE_STEP_INT_MASK,
		FIELD_PREP(HISTB_APLL_TUNE_STEP_INT_MASK, 1), NULL);
	if (ret)
		goto out_status;

	ret = histb_cpu_apll_update_bits(
		cpu, HISTB_APLL_TUNE_STEP_FRAC,
		HISTB_APLL_TUNE_STEP_FRAC_MASK, 0, NULL);
	if (ret)
		goto out_status;

	/* All verified configuration writes must reach CRG before tune_en. */
	wmb();
	ret = histb_cpu_apll_update_bits(
		cpu, HISTB_APLL_TUNE_CTRL,
		HISTB_APLL_TUNE_MODE | HISTB_APLL_TUNE_ENABLE,
		HISTB_APLL_TUNE_MODE | HISTB_APLL_TUNE_ENABLE,
		NULL);
	if (ret)
		goto out_status;

	ret = readl_poll_timeout(cpu->base + HISTB_APLL_TUNE_STATUS,
				 status,
				 !(status & HISTB_APLL_TUNE_BUSY) &&
				 FIELD_GET(HISTB_APLL_TUNE_RESULT_MASK,
					   status) == fbdiv,
				 HISTB_APLL_POLL_DELAY_US,
				 HISTB_APLL_TUNE_TIMEOUT_US);

	if (ret) {
		/* Cancel a timed-out request before attempting the rollback. */
		histb_cpu_apll_update_bits(
			cpu, HISTB_APLL_TUNE_CTRL,
			HISTB_APLL_TUNE_MODE |
				HISTB_APLL_TUNE_ENABLE,
			ctrl & (HISTB_APLL_TUNE_MODE |
				HISTB_APLL_TUNE_ENABLE), NULL);
		readl_poll_timeout(cpu->base + HISTB_APLL_TUNE_STATUS,
				   status,
				   !(status & HISTB_APLL_TUNE_BUSY),
				   HISTB_APLL_POLL_DELAY_US,
				   HISTB_APLL_IDLE_TIMEOUT_US);
	}

out_status:
	if (last_status)
		*last_status = status;

	return ret;
}

static unsigned long histb_cpu_clk_recalc_rate(struct clk_hw *hw,
						     unsigned long parent_rate)
{
	struct histb_cpu_clk *cpu = to_histb_cpu_clk(hw);
	u32 fbdiv, selector;

	selector = readl_relaxed(cpu->base + HISTB_CPU_CLK_CTRL) &
		   HISTB_CPU_CLK_SEL_MASK;
	switch (selector) {
	case 1:
		return 200000000;
	case 2:
		return 800000000;
	case 3:
		return 1350000000;
	case 4:
		return 24000000;
	case 5:
		return 1200000000;
	case 6:
		return 400000000;
	case 7:
		return 600000000;
	case 0:
		break;
	default:
		return 0;
	}

	if (histb_cpu_apll_snapshot(cpu, &fbdiv, &parent_rate))
		return 0;

	return parent_rate;
}

static long histb_cpu_clk_round_rate(struct clk_hw *hw,
					   unsigned long rate,
					   unsigned long *parent_rate)
{
	unsigned long best_delta = ULONG_MAX;
	unsigned long best_rate = histb_cpu_rates[0];
	int i;

	for (i = 0; i < ARRAY_SIZE(histb_cpu_rates); i++) {
		unsigned long candidate = histb_cpu_rates[i];
		unsigned long delta = candidate > rate ? candidate - rate :
							   rate - candidate;

		if (candidate > to_histb_cpu_clk(hw)->max_rate)
			continue;
		if (delta < best_delta) {
			best_delta = delta;
			best_rate = candidate;
		}
	}

	return best_rate;
}

static int histb_cpu_clk_set_rate(struct clk_hw *hw,
					unsigned long rate,
					unsigned long parent_rate)
{
	struct histb_cpu_clk *cpu = to_histb_cpu_clk(hw);
	u32 fbdiv, old_fbdiv, postdiv1, postdiv2, refdiv, status;
	unsigned long actual, old_rate, rollback_rate;
	u64 divisor;
	bool supported = false;
	int i, ret, rollback_ret;

	for (i = 0; i < ARRAY_SIZE(histb_cpu_rates); i++)
		if (histb_cpu_rates[i] == rate) {
			supported = true;
			break;
		}
	if (!supported || rate > cpu->max_rate)
		return -EINVAL;

	if (histb_cpu_pll_dividers(cpu, &postdiv1, &postdiv2, &refdiv))
		return -EINVAL;

	divisor = (u64)postdiv1 * postdiv2 * refdiv;
	fbdiv = div64_u64((u64)rate * divisor, HISTB_CPU_PLL_REF_RATE);
	if (!fbdiv || fbdiv > FIELD_MAX(HISTB_CPU_PLL_FBDIV_MASK))
		return -EINVAL;

	actual = div64_u64((u64)fbdiv * HISTB_CPU_PLL_REF_RATE,
			   divisor);
	if (abs_diff(actual, rate) >= HISTB_CPU_RATE_TOLERANCE)
		return -EINVAL;

	mutex_lock(&cpu->lock);
	if ((readl_relaxed(cpu->base + HISTB_CPU_CLK_CTRL) &
	     HISTB_CPU_CLK_SEL_MASK) != 0) {
		ret = -EOPNOTSUPP;
		goto unlock;
	}

	ret = histb_cpu_apll_snapshot(cpu, &old_fbdiv, &old_rate);
	if (ret)
		goto latch_fault;

	if (old_rate == rate) {
		ret = 0;
		goto unlock;
	}

	if (cpu->rate_fault) {
		ret = -EIO;
		goto unlock;
	}

	ret = histb_cpu_apll_tune(cpu, fbdiv, &status);
	if (!ret) {
		ret = histb_cpu_apll_snapshot(cpu, &fbdiv, &actual);
		if (!ret && actual == rate) {
			goto unlock;
		}
		if (!ret)
			ret = -EIO;
	}

	rollback_ret = histb_cpu_apll_tune(cpu, old_fbdiv, &status);
	if (!rollback_ret) {
		rollback_ret = histb_cpu_apll_snapshot(cpu, &old_fbdiv,
							     &rollback_rate);
		if (!rollback_ret && rollback_rate != old_rate)
			rollback_ret = -EIO;
	}

	WRITE_ONCE(cpu->rate_fault, true);
	dev_err_ratelimited(cpu->dev,
			    "CPU APLL %lu Hz transition failed: %d, rollback %s (status=%08x)\n",
		rate, ret, rollback_ret ? "failed" : "succeeded", status);
	goto unlock;

latch_fault:
	WRITE_ONCE(cpu->rate_fault, true);
	dev_err_ratelimited(cpu->dev,
			    "CPU APLL was busy before %lu Hz transition (status=%08x)\n",
		rate, readl_relaxed(cpu->base + HISTB_APLL_TUNE_STATUS));
unlock:
	mutex_unlock(&cpu->lock);
	return ret;
}

static const struct clk_ops histb_cpu_clk_ops = {
	.recalc_rate = histb_cpu_clk_recalc_rate,
	.round_rate = histb_cpu_clk_round_rate,
	.set_rate = histb_cpu_clk_set_rate,
};

struct clk *hisi_histb_cpu_clk_register(struct device *dev, void __iomem *base,
				      const char *name, unsigned long max_rate)
{
	struct clk_init_data init = {
		.name = name,
		.ops = &histb_cpu_clk_ops,
		.flags = CLK_GET_RATE_NOCACHE,
	};
	struct histb_cpu_clk *cpu;

	cpu = devm_kzalloc(dev, sizeof(*cpu), GFP_KERNEL);
	if (!cpu)
		return ERR_PTR(-ENOMEM);
	cpu->base = base;
	cpu->dev = dev;
	cpu->hw.init = &init;
	cpu->max_rate = max_rate;
	mutex_init(&cpu->lock);
	return clk_register(dev, &cpu->hw);
}
EXPORT_SYMBOL_GPL(hisi_histb_cpu_clk_register);

MODULE_DESCRIPTION("HiSilicon HiSTB CPU APLL clock");
MODULE_LICENSE("GPL");
