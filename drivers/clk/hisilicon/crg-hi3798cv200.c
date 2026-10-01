// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hi3798CV200 Clock and Reset Generator Driver
 *
 * Copyright (c) 2016 HiSilicon Technologies Co., Ltd.
 */

#include <dt-bindings/clock/histb-clock.h>
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include "clk.h"
#include "crg.h"
#include "reset.h"

/* hi3798CV200 core CRG */
#define HI3798CV200_INNER_CLK_OFFSET		128
#define HI3798CV200_FIXED_24M			129
#define HI3798CV200_FIXED_25M			130
#define HI3798CV200_FIXED_50M			131
#define HI3798CV200_FIXED_75M			132
#define HI3798CV200_FIXED_100M			133
#define HI3798CV200_FIXED_150M			134
#define HI3798CV200_FIXED_200M			135
#define HI3798CV200_FIXED_250M			136
#define HI3798CV200_FIXED_300M			137
#define HI3798CV200_FIXED_400M			138
#define HI3798CV200_MMC_MUX			139
#define HI3798CV200_ETH_PUB_CLK			140
#define HI3798CV200_ETH_BUS_CLK			141
#define HI3798CV200_ETH_BUS0_CLK		142
#define HI3798CV200_ETH_BUS1_CLK		143
#define HI3798CV200_COMBPHY1_MUX		144
#define HI3798CV200_FIXED_12M			145
#define HI3798CV200_FIXED_48M			146
#define HI3798CV200_FIXED_60M			147
#define HI3798CV200_FIXED_166P5M		148
#define HI3798CV200_SDIO0_MUX			149
#define HI3798CV200_COMBPHY0_MUX		150
#define HI3798CV200_FIXED_933P888M		151
#define HI3798CV200_AIAO_MUX			152
#define HI3798CV200_FIXED_333M			153
#define HI3798CV200_VPSS_MUX			154
#define HI3798CV200_FIXED_270M			155
#define HI3798CV200_VENC_MUX			156

#define HI3798CV200_CRG_NR_CLKS			192

#define HI3798CV200_GPU_CLK_CTRL		0x124
#define HI3798CV200_GPU_CLK_GATE_RESET	0xd4
#define HI3798CV200_GPU_CLK_STATUS	0x154
#define HI3798CV200_GPU_CLK_SEL_MASK	GENMASK(2, 0)
#define HI3798CV200_GPU_CLK_BYPASS	BIT(9)
#define HI3798CV200_GPU_CLK_SWITCH	BIT(10)
#define HI3798CV200_GPU_CLK_GATE		BIT(0)
#define HI3798CV200_GPU_RESET		BIT(4)
#define HI3798CV200_GPU_STATUS_MASK	GENMASK(7, 5)
#define HI3798CV200_GPU_STATUS_SHIFT	5
#define HI3798CV200_CPU_PLL_CFG0		0x0
#define HI3798CV200_CPU_PLL_CFG1		0x4
#define HI3798CV200_CPU_CLK_CTRL		0x48
#define HI3798CV200_CPU_CLK_SEL_MASK	GENMASK(2, 0)
#define HI3798CV200_CPU_PLL_POSTDIV1_MASK GENMASK(26, 24)
#define HI3798CV200_CPU_PLL_POSTDIV2_MASK GENMASK(30, 28)
#define HI3798CV200_CPU_PLL_FBDIV_MASK	GENMASK(11, 0)
#define HI3798CV200_CPU_PLL_REFDIV_MASK	GENMASK(17, 12)
#define HI3798CV200_APLL_TUNE_INT	0x1a4
#define HI3798CV200_APLL_TUNE_FRAC	0x1a8
#define HI3798CV200_APLL_TUNE_STEP_INT	0x1ac
#define HI3798CV200_APLL_TUNE_STEP_FRAC	0x1b0
#define HI3798CV200_APLL_TUNE_CTRL	0x1b4
#define HI3798CV200_APLL_TUNE_INT_MASK	GENMASK(11, 0)
#define HI3798CV200_APLL_TUNE_FRAC_MASK	GENMASK(23, 0)
#define HI3798CV200_APLL_TUNE_STEP_INT_MASK GENMASK(11, 0)
#define HI3798CV200_APLL_TUNE_STEP_FRAC_MASK GENMASK(23, 0)
#define HI3798CV200_APLL_TUNE_MODE	BIT(2)
#define HI3798CV200_APLL_TUNE_ENABLE	BIT(3)
#define HI3798CV200_APLL_TUNE_STATUS	0x294
#define HI3798CV200_APLL_TUNE_RESULT_MASK GENMASK(11, 0)
#define HI3798CV200_APLL_TUNE_BUSY	BIT(13)
#define HI3798CV200_CPU_PLL_REF_RATE	24000000UL
#define HI3798CV200_HPLL_CTRL0		0x28
#define HI3798CV200_HPLL_CTRL1		0x2c
#define HI3798CV200_HPLL_FRAC_MASK	GENMASK(23, 0)
#define HI3798CV200_HPLL_POSTDIV1_MASK	GENMASK(26, 24)
#define HI3798CV200_HPLL_POSTDIV2_MASK	GENMASK(30, 28)
#define HI3798CV200_HPLL_FBDIV_MASK	GENMASK(11, 0)
#define HI3798CV200_HPLL_REFDIV_MASK	GENMASK(17, 12)
#define HI3798CV200_HPLL_PD		BIT(20)
#define HI3798CV200_HPLL_FOUTVCOPD	BIT(21)
#define HI3798CV200_HPLL_FOUT4PHASEPD	BIT(22)
#define HI3798CV200_HPLL_FOUTPOSTDIVPD	BIT(23)
#define HI3798CV200_HPLL_DACPD		BIT(24)
#define HI3798CV200_HPLL_DSMPD		BIT(25)
#define HI3798CV200_HPLL_BYPASS		BIT(26)
#define HI3798CV200_HPLL_POWER_MASK	GENMASK(24, 20)
#define HI3798CV200_HPLL_LOCK_STATUS	0x150
#define HI3798CV200_HPLL_LOCK		BIT(4)
#define HI3798CV200_VO_CLK_CTRL		0xd8
#define HI3798CV200_VO_HD_CLK_SEL	BIT(16)
#define HI3798CV200_VO_HD_CLK_DIV_MASK	GENMASK(19, 18)
#define HI3798CV200_VO_HD_CLK_DIV_2	0
#define HI3798CV200_VO_HD_CLK_DIV_4	1
#define HI3798CV200_VO_HD_CLK_DIV_1	3
#define HI3798CV200_HDMI_CLK_SEL	BIT(25)
#define HI3798CV200_VDP_CLK_SEL		BIT(27)
#define HI3798CV200_VO_HD_HDMI_DIV	BIT(29)
#define HI3798CV200_HDMI_CTRL_CLK	0x10c
#define HI3798CV200_HDMI_ASCLK_SEL	BIT(14)
#define HI3798CV200_HDMI_PIXEL_CTRL	0x278
#define HI3798CV200_HDMI_PIXEL_GATE	BIT(2)
#define HI3798CV200_HDMI_PIXELNX_SEL	BIT(4)
#define HI3798CV200_HDMI_OSCLK_SEL	BIT(5)
#define HI3798CV200_HDMI_IDCLK_SEL	BIT(6)
#define HI3798CV200_VDP_CLK_CTRL		0x34c
#define HI3798CV200_VDP_HD_DIV0		BIT(20)
#define HI3798CV200_VDP_INIT_SEL0	BIT(22)
#define HI3798CV200_VO_HD_BP_CLK_SEL	BIT(23)
#define HI3798CV200_HDMI_PIXEL_MIN_RATE	25000000UL
#define HI3798CV200_HDMI14_MAX_RATE	340000000UL
#define HI3798CV200_HDMI_PIXEL_MAX_RATE	594000000UL
#define HI3798CV200_HPLL_PARENT_RATE	24000000UL
#define HI3798CV200_HPLL_VCO_MIN_RATE	800000000ULL
#define HI3798CV200_HPLL_VCO_MAX_RATE	2400000000ULL
#define HI3798CV200_TSI_COUNT		6
#define HI3798CV200_TSI_GATE_SHIFT	3
#define HI3798CV200_TSI_PHASE_SHIFT	15

struct hi3798cv200_tsi_clk {
	struct clk_hw hw;
	void __iomem *base;
	u8 index;
};

#define to_hi3798cv200_tsi_clk(_hw) \
	container_of(_hw, struct hi3798cv200_tsi_clk, hw)

static int hi3798cv200_tsi_clk_enable(struct clk_hw *hw)
{
	struct hi3798cv200_tsi_clk *tsi = to_hi3798cv200_tsi_clk(hw);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&hisi_clk_lock, flags);
	reg = readl_relaxed(tsi->base + 0xfc);
	reg |= BIT(HI3798CV200_TSI_GATE_SHIFT + tsi->index);
	writel_relaxed(reg, tsi->base + 0xfc);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);

	return 0;
}

static void hi3798cv200_tsi_clk_disable(struct clk_hw *hw)
{
	struct hi3798cv200_tsi_clk *tsi = to_hi3798cv200_tsi_clk(hw);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&hisi_clk_lock, flags);
	reg = readl_relaxed(tsi->base + 0xfc);
	reg &= ~BIT(HI3798CV200_TSI_GATE_SHIFT + tsi->index);
	writel_relaxed(reg, tsi->base + 0xfc);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
}

static int hi3798cv200_tsi_clk_is_enabled(struct clk_hw *hw)
{
	struct hi3798cv200_tsi_clk *tsi = to_hi3798cv200_tsi_clk(hw);

	return !!(readl_relaxed(tsi->base + 0xfc) &
		  BIT(HI3798CV200_TSI_GATE_SHIFT + tsi->index));
}

static int hi3798cv200_tsi_clk_get_phase(struct clk_hw *hw)
{
	struct hi3798cv200_tsi_clk *tsi = to_hi3798cv200_tsi_clk(hw);

	return readl_relaxed(tsi->base + 0xfc) &
	       BIT(HI3798CV200_TSI_PHASE_SHIFT + tsi->index) ? 180 : 0;
}

static int hi3798cv200_tsi_clk_set_phase(struct clk_hw *hw, int degrees)
{
	struct hi3798cv200_tsi_clk *tsi = to_hi3798cv200_tsi_clk(hw);
	unsigned long flags;
	u32 mask, reg;

	if (degrees != 0 && degrees != 180)
		return -EINVAL;

	mask = BIT(HI3798CV200_TSI_PHASE_SHIFT + tsi->index);

	spin_lock_irqsave(&hisi_clk_lock, flags);
	reg = readl_relaxed(tsi->base + 0xfc);
	if (degrees == 180)
		reg |= mask;
	else
		reg &= ~mask;
	writel_relaxed(reg, tsi->base + 0xfc);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);

	return 0;
}

static const struct clk_ops hi3798cv200_tsi_clk_ops = {
	.enable = hi3798cv200_tsi_clk_enable,
	.disable = hi3798cv200_tsi_clk_disable,
	.is_enabled = hi3798cv200_tsi_clk_is_enabled,
	.get_phase = hi3798cv200_tsi_clk_get_phase,
	.set_phase = hi3798cv200_tsi_clk_set_phase,
};

static const char *const hi3798cv200_tsi_clk_names[] = {
	"clk_tsi0", "clk_tsi1", "clk_tsi2",
	"clk_tsi3", "clk_tsi4", "clk_tsi5",
};

static const unsigned int hi3798cv200_tsi_clk_ids[] = {
	HISTB_TSI0_CLK, HISTB_TSI1_CLK, HISTB_TSI2_CLK,
	HISTB_TSI3_CLK, HISTB_TSI4_CLK, HISTB_TSI5_CLK,
};

static int hi3798cv200_tsi_clks_register(struct platform_device *pdev,
					 struct hisi_clock_data *clk_data)
{
	struct hi3798cv200_tsi_clk *tsi;
	int i;

	tsi = devm_kcalloc(&pdev->dev, HI3798CV200_TSI_COUNT, sizeof(*tsi),
			   GFP_KERNEL);
	if (!tsi)
		return -ENOMEM;

	for (i = 0; i < HI3798CV200_TSI_COUNT; i++) {
		struct clk_init_data init = {
			.name = hi3798cv200_tsi_clk_names[i],
			.ops = &hi3798cv200_tsi_clk_ops,
		};
		struct clk *clk;

		tsi[i].base = clk_data->base;
		tsi[i].index = i;
		tsi[i].hw.init = &init;

		clk = clk_register(&pdev->dev, &tsi[i].hw);
		if (IS_ERR(clk)) {
			while (i--) {
				unsigned int id = hi3798cv200_tsi_clk_ids[i];

				clk_unregister(clk_data->clk_data.clks[id]);
			}

			return PTR_ERR(clk);
		}

		clk_data->clk_data.clks[hi3798cv200_tsi_clk_ids[i]] = clk;
	}

	return 0;
}

static void hi3798cv200_tsi_clks_unregister(struct hisi_clock_data *clk_data)
{
	int i;

	for (i = 0; i < HI3798CV200_TSI_COUNT; i++) {
		unsigned int id = hi3798cv200_tsi_clk_ids[i];

		clk_unregister(clk_data->clk_data.clks[id]);
		clk_data->clk_data.clks[id] = NULL;
	}
}

struct hi3798cv200_gpu_clk {
	struct clk_hw hw;
	struct device *dev;
	void __iomem *base;
	struct mutex lock;
	bool rate_fault;
};

struct hi3798cv200_gpu_rate {
	unsigned long rate;
	u8 selector;
};

static const struct hi3798cv200_gpu_rate hi3798cv200_gpu_rates[] = {
	{ 200000000, 7 },
	{ 300000000, 4 },
	{ 400000000, 1 },
	{ 500000000, 6 },
	{ 600000000, 3 },
	{ 675000000, 5 },
};

#define to_hi3798cv200_gpu_clk(_hw) \
	container_of(_hw, struct hi3798cv200_gpu_clk, hw)

static const struct hi3798cv200_gpu_rate *
hi3798cv200_gpu_find_rate(unsigned long rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hi3798cv200_gpu_rates); i++)
		if (hi3798cv200_gpu_rates[i].rate == rate)
			return &hi3798cv200_gpu_rates[i];

	return NULL;
}

static u8 hi3798cv200_gpu_status_selector(struct hi3798cv200_gpu_clk *gpu)
{
	return (readl_relaxed(gpu->base + HI3798CV200_GPU_CLK_STATUS) &
		HI3798CV200_GPU_STATUS_MASK) >> HI3798CV200_GPU_STATUS_SHIFT;
}

static unsigned long hi3798cv200_gpu_selector_rate(u8 selector)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hi3798cv200_gpu_rates); i++)
		if (hi3798cv200_gpu_rates[i].selector == selector)
			return hi3798cv200_gpu_rates[i].rate;

	return 0;
}

/* hisi_clk_lock must be held while updating the switch request. */
static void hi3798cv200_gpu_select_locked(struct hi3798cv200_gpu_clk *gpu,
					  u8 selector)
{
	u32 reg;

	reg = readl_relaxed(gpu->base + HI3798CV200_GPU_CLK_CTRL);
	reg &= ~(HI3798CV200_GPU_CLK_SEL_MASK | HI3798CV200_GPU_CLK_SWITCH);
	reg |= HI3798CV200_GPU_CLK_BYPASS | selector;
	writel_relaxed(reg, gpu->base + HI3798CV200_GPU_CLK_CTRL);
	writel_relaxed(reg | HI3798CV200_GPU_CLK_SWITCH,
		       gpu->base + HI3798CV200_GPU_CLK_CTRL);
}

static int hi3798cv200_gpu_wait_selector(struct hi3798cv200_gpu_clk *gpu,
					 u8 selector)
{
	u32 status;

	return readl_poll_timeout(gpu->base + HI3798CV200_GPU_CLK_STATUS,
				  status,
				  ((status & HI3798CV200_GPU_STATUS_MASK) >>
				   HI3798CV200_GPU_STATUS_SHIFT) == selector,
				  1, 1000);
}

/*
 * CCF does not propagate a provider set_rate error.  A failed down-clock must
 * therefore not leave the GPU at a high rate while OPP lowers its voltage.
 * Restore the old selector first, then use the verified 200 MHz floor if the
 * restored rate is above the request.  If recovery cannot be verified, gate
 * the GPU instead of exposing an unknown high-rate/low-voltage combination.
 */
static int __hi3798cv200_gpu_program_rate(struct hi3798cv200_gpu_clk *gpu,
					  u8 selector, bool fail_safe)
{
	const u8 safe_selector = hi3798cv200_gpu_rates[0].selector;
	unsigned long flags, old_rate, target_rate;
	u8 old_selector, recovery_selector;
	u32 reg;
	int ret, recovery_ret;

	old_selector = hi3798cv200_gpu_status_selector(gpu);
	old_rate = hi3798cv200_gpu_selector_rate(old_selector);
	target_rate = hi3798cv200_gpu_selector_rate(selector);

	spin_lock_irqsave(&hisi_clk_lock, flags);
	hi3798cv200_gpu_select_locked(gpu, selector);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);

	ret = hi3798cv200_gpu_wait_selector(gpu, selector);
	if (!ret) {
		WRITE_ONCE(gpu->rate_fault, false);
		return 0;
	}

	/* First restore the state that was known to be active before the write. */
	spin_lock_irqsave(&hisi_clk_lock, flags);
	hi3798cv200_gpu_select_locked(gpu, old_selector);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
	recovery_ret = hi3798cv200_gpu_wait_selector(gpu, old_selector);
	recovery_selector = old_selector;

	if (fail_safe && (recovery_ret || !old_rate || old_rate > target_rate)) {
		spin_lock_irqsave(&hisi_clk_lock, flags);
		hi3798cv200_gpu_select_locked(gpu, safe_selector);
		spin_unlock_irqrestore(&hisi_clk_lock, flags);
		recovery_selector = safe_selector;
		recovery_ret = hi3798cv200_gpu_wait_selector(gpu, safe_selector);
	}

	/* A second successful request for the target is a valid recovery. */
	if (!recovery_ret && recovery_selector == selector) {
		WRITE_ONCE(gpu->rate_fault, false);
		return 0;
	}

	WRITE_ONCE(gpu->rate_fault, true);
	if (recovery_ret) {
		spin_lock_irqsave(&hisi_clk_lock, flags);
		reg = readl_relaxed(gpu->base + HI3798CV200_GPU_CLK_GATE_RESET);
		reg &= ~HI3798CV200_GPU_CLK_GATE;
		writel_relaxed(reg,
			       gpu->base + HI3798CV200_GPU_CLK_GATE_RESET);
		spin_unlock_irqrestore(&hisi_clk_lock, flags);
	}

	dev_err_ratelimited(gpu->dev,
			    "GPU selector %u timed out; recovery selector %u %s\n",
		selector, recovery_selector,
		recovery_ret ? "unverified, clock gated" : "active");

	return ret;
}

static int hi3798cv200_gpu_program_rate(struct hi3798cv200_gpu_clk *gpu,
					u8 selector)
{
	int ret;

	mutex_lock(&gpu->lock);
	ret = __hi3798cv200_gpu_program_rate(gpu, selector, true);
	mutex_unlock(&gpu->lock);

	return ret;
}

static int hi3798cv200_gpu_clk_prepare(struct clk_hw *hw)
{
	struct hi3798cv200_gpu_clk *gpu = to_hi3798cv200_gpu_clk(hw);
	unsigned long flags;
	bool restore_gate;
	u32 reg;
	int ret;

	/* The first prepare is serialized before its matching enable. */
	mutex_lock(&gpu->lock);
	spin_lock_irqsave(&hisi_clk_lock, flags);
	reg = readl_relaxed(gpu->base + HI3798CV200_GPU_CLK_GATE_RESET);
	restore_gate = !(reg & HI3798CV200_GPU_CLK_GATE);
	if (restore_gate)
		writel_relaxed(reg | HI3798CV200_GPU_RESET |
			       HI3798CV200_GPU_CLK_GATE,
			       gpu->base + HI3798CV200_GPU_CLK_GATE_RESET);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);

	ret = __hi3798cv200_gpu_program_rate(gpu,
					     hi3798cv200_gpu_rates[0].selector,
					      false);

	if (restore_gate) {
		spin_lock_irqsave(&hisi_clk_lock, flags);
		writel_relaxed(reg,
			       gpu->base + HI3798CV200_GPU_CLK_GATE_RESET);
		spin_unlock_irqrestore(&hisi_clk_lock, flags);
	}
	mutex_unlock(&gpu->lock);

	return ret;
}

static int hi3798cv200_gpu_clk_enable(struct clk_hw *hw)
{
	struct hi3798cv200_gpu_clk *gpu = to_hi3798cv200_gpu_clk(hw);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&hisi_clk_lock, flags);

	reg = readl_relaxed(gpu->base + HI3798CV200_GPU_CLK_GATE_RESET);
	reg |= HI3798CV200_GPU_RESET | HI3798CV200_GPU_CLK_GATE;
	writel_relaxed(reg, gpu->base + HI3798CV200_GPU_CLK_GATE_RESET);

	spin_unlock_irqrestore(&hisi_clk_lock, flags);

	return 0;
}

static void hi3798cv200_gpu_clk_disable(struct clk_hw *hw)
{
	struct hi3798cv200_gpu_clk *gpu = to_hi3798cv200_gpu_clk(hw);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&hisi_clk_lock, flags);
	reg = readl_relaxed(gpu->base + HI3798CV200_GPU_CLK_GATE_RESET);
	reg &= ~HI3798CV200_GPU_CLK_GATE;
	writel_relaxed(reg, gpu->base + HI3798CV200_GPU_CLK_GATE_RESET);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
}

static int hi3798cv200_gpu_clk_is_enabled(struct clk_hw *hw)
{
	struct hi3798cv200_gpu_clk *gpu = to_hi3798cv200_gpu_clk(hw);

	return !!(readl_relaxed(gpu->base + HI3798CV200_GPU_CLK_GATE_RESET) &
		  HI3798CV200_GPU_CLK_GATE);
}

static unsigned long hi3798cv200_gpu_clk_recalc_rate(struct clk_hw *hw,
						     unsigned long parent_rate)
{
	struct hi3798cv200_gpu_clk *gpu = to_hi3798cv200_gpu_clk(hw);

	return hi3798cv200_gpu_selector_rate(
		hi3798cv200_gpu_status_selector(gpu));
}

static long hi3798cv200_gpu_clk_round_rate(struct clk_hw *hw,
					   unsigned long rate,
					   unsigned long *parent_rate)
{
	unsigned long best_delta = ULONG_MAX;
	unsigned long best_rate = hi3798cv200_gpu_rates[0].rate;
	int i;

	if (READ_ONCE(to_hi3798cv200_gpu_clk(hw)->rate_fault))
		return best_rate;

	for (i = 0; i < ARRAY_SIZE(hi3798cv200_gpu_rates); i++) {
		unsigned long candidate = hi3798cv200_gpu_rates[i].rate;
		unsigned long delta = candidate > rate ?
			candidate - rate : rate - candidate;

		if (delta < best_delta) {
			best_delta = delta;
			best_rate = candidate;
		}
	}

	return best_rate;
}

static int hi3798cv200_gpu_clk_set_rate(struct clk_hw *hw,
					unsigned long rate,
					unsigned long parent_rate)
{
	struct hi3798cv200_gpu_clk *gpu = to_hi3798cv200_gpu_clk(hw);
	const struct hi3798cv200_gpu_rate *gpu_rate;

	gpu_rate = hi3798cv200_gpu_find_rate(rate);
	if (!gpu_rate)
		return -EINVAL;

	return hi3798cv200_gpu_program_rate(gpu, gpu_rate->selector);
}

static const struct clk_ops hi3798cv200_gpu_clk_ops = {
	.prepare = hi3798cv200_gpu_clk_prepare,
	.enable = hi3798cv200_gpu_clk_enable,
	.disable = hi3798cv200_gpu_clk_disable,
	.is_enabled = hi3798cv200_gpu_clk_is_enabled,
	.recalc_rate = hi3798cv200_gpu_clk_recalc_rate,
	.round_rate = hi3798cv200_gpu_clk_round_rate,
	.set_rate = hi3798cv200_gpu_clk_set_rate,
};

static int hi3798cv200_gpu_clk_register(struct platform_device *pdev,
					struct hisi_clock_data *clk_data)
{
	struct hi3798cv200_gpu_clk *gpu;
	struct clk_init_data init = {
		.name = "clk_gpu",
		.ops = &hi3798cv200_gpu_clk_ops,
		.flags = CLK_GET_RATE_NOCACHE | CLK_SET_RATE_UNGATE,
	};
	struct clk *clk;

	gpu = devm_kzalloc(&pdev->dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;

	gpu->base = clk_data->base;
	gpu->dev = &pdev->dev;
	gpu->hw.init = &init;
	mutex_init(&gpu->lock);

	clk = clk_register(&pdev->dev, &gpu->hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	clk_data->clk_data.clks[HISTB_GPU_CLK] = clk;

	return 0;
}

static void hi3798cv200_gpu_clk_unregister(struct hisi_clock_data *clk_data)
{
	clk_unregister(clk_data->clk_data.clks[HISTB_GPU_CLK]);
	clk_data->clk_data.clks[HISTB_GPU_CLK] = NULL;
}

struct hi3798cv200_cpu_clk {
	struct clk_hw hw;
	void __iomem *base;
	struct device *dev;
	/* Serializes the shared APLL tuning transaction. */
	struct mutex lock;
	/* A failed transaction requires a reboot before another retune. */
	bool rate_fault;
	unsigned long last_good_rate;
};

static const unsigned long hi3798cv200_cpu_rates[] = {
	400000000,
	600000000,
	800000000,
	1200000000,
	1600000000,
};

#define HI3798CV200_CPU_RATE_TOLERANCE	10000000UL
#define HI3798CV200_APLL_POLL_DELAY_US	10
#define HI3798CV200_APLL_IDLE_TIMEOUT_US	1000
#define HI3798CV200_APLL_TUNE_TIMEOUT_US	100000

#define to_hi3798cv200_cpu_clk(_hw) \
	container_of(_hw, struct hi3798cv200_cpu_clk, hw)

static int hi3798cv200_cpu_pll_dividers(struct hi3798cv200_cpu_clk *cpu,
					u32 *postdiv1, u32 *postdiv2,
					u32 *refdiv)
{
	u32 cfg0 = readl_relaxed(cpu->base + HI3798CV200_CPU_PLL_CFG0);
	u32 cfg1 = readl_relaxed(cpu->base + HI3798CV200_CPU_PLL_CFG1);

	*postdiv1 = FIELD_GET(HI3798CV200_CPU_PLL_POSTDIV1_MASK, cfg0);
	*postdiv2 = FIELD_GET(HI3798CV200_CPU_PLL_POSTDIV2_MASK, cfg0);
	*refdiv = FIELD_GET(HI3798CV200_CPU_PLL_REFDIV_MASK, cfg1);

	return *postdiv1 && *postdiv2 && *refdiv ? 0 : -EINVAL;
}

static unsigned long
hi3798cv200_cpu_rate_from_fbdiv(struct hi3798cv200_cpu_clk *cpu, u32 fbdiv)
{
	u32 postdiv1, postdiv2, refdiv;
	unsigned long rate;
	u64 divisor;
	int i;

	if (!fbdiv ||
	    hi3798cv200_cpu_pll_dividers(cpu, &postdiv1, &postdiv2, &refdiv))
		return 0;

	divisor = (u64)postdiv1 * postdiv2 * refdiv;
	rate = div64_u64((u64)fbdiv * HI3798CV200_CPU_PLL_REF_RATE,
			 divisor);

	for (i = 0; i < ARRAY_SIZE(hi3798cv200_cpu_rates); i++)
		if (abs_diff(rate, hi3798cv200_cpu_rates[i]) <
		    HI3798CV200_CPU_RATE_TOLERANCE)
			return hi3798cv200_cpu_rates[i];

	return rate;
}

static int hi3798cv200_cpu_apll_snapshot(struct hi3798cv200_cpu_clk *cpu,
					 u32 *fbdiv,
					 unsigned long *rate)
{
	u32 status;

	status = readl_relaxed(cpu->base + HI3798CV200_APLL_TUNE_STATUS);
	if (status & HI3798CV200_APLL_TUNE_BUSY)
		return -EBUSY;

	*fbdiv = FIELD_GET(HI3798CV200_APLL_TUNE_RESULT_MASK, status);
	if (!*fbdiv)
		*fbdiv = FIELD_GET(HI3798CV200_CPU_PLL_FBDIV_MASK,
				   readl_relaxed(cpu->base +
						 HI3798CV200_CPU_PLL_CFG1));

	*rate = hi3798cv200_cpu_rate_from_fbdiv(cpu, *fbdiv);
	return *rate ? 0 : -EINVAL;
}

static int hi3798cv200_cpu_apll_update_bits(
					struct hi3798cv200_cpu_clk *cpu,
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

static int hi3798cv200_cpu_apll_tune(struct hi3798cv200_cpu_clk *cpu,
				     u32 fbdiv, u32 *last_status)
{
	u32 ctrl, status;
	int ret;

	ret = readl_poll_timeout(cpu->base + HI3798CV200_APLL_TUNE_STATUS,
				 status, !(status & HI3798CV200_APLL_TUNE_BUSY),
				 HI3798CV200_APLL_POLL_DELAY_US,
				 HI3798CV200_APLL_IDLE_TIMEOUT_US);
	if (ret)
		goto out_status;

	/* Keep configuration fields inert while their new image is assembled. */
	ret = hi3798cv200_cpu_apll_update_bits(
		cpu, HI3798CV200_APLL_TUNE_CTRL,
		HI3798CV200_APLL_TUNE_MODE | HI3798CV200_APLL_TUNE_ENABLE,
		HI3798CV200_APLL_TUNE_MODE, &ctrl);
	if (ret)
		goto out_status;

	ret = hi3798cv200_cpu_apll_update_bits(
		cpu, HI3798CV200_APLL_TUNE_INT,
		HI3798CV200_APLL_TUNE_INT_MASK,
		FIELD_PREP(HI3798CV200_APLL_TUNE_INT_MASK, fbdiv), NULL);
	if (ret)
		goto out_status;

	ret = hi3798cv200_cpu_apll_update_bits(
		cpu, HI3798CV200_APLL_TUNE_FRAC,
		HI3798CV200_APLL_TUNE_FRAC_MASK, 0, NULL);
	if (ret)
		goto out_status;

	ret = hi3798cv200_cpu_apll_update_bits(
		cpu, HI3798CV200_APLL_TUNE_STEP_INT,
		HI3798CV200_APLL_TUNE_STEP_INT_MASK,
		FIELD_PREP(HI3798CV200_APLL_TUNE_STEP_INT_MASK, 1), NULL);
	if (ret)
		goto out_status;

	ret = hi3798cv200_cpu_apll_update_bits(
		cpu, HI3798CV200_APLL_TUNE_STEP_FRAC,
		HI3798CV200_APLL_TUNE_STEP_FRAC_MASK, 0, NULL);
	if (ret)
		goto out_status;

	/* All verified configuration writes must reach CRG before tune_en. */
	wmb();
	ret = hi3798cv200_cpu_apll_update_bits(
		cpu, HI3798CV200_APLL_TUNE_CTRL,
		HI3798CV200_APLL_TUNE_MODE | HI3798CV200_APLL_TUNE_ENABLE,
		HI3798CV200_APLL_TUNE_MODE | HI3798CV200_APLL_TUNE_ENABLE,
		NULL);
	if (ret)
		goto out_status;

	ret = readl_poll_timeout(cpu->base + HI3798CV200_APLL_TUNE_STATUS,
				 status,
				 !(status & HI3798CV200_APLL_TUNE_BUSY) &&
				 FIELD_GET(HI3798CV200_APLL_TUNE_RESULT_MASK,
					   status) == fbdiv,
				 HI3798CV200_APLL_POLL_DELAY_US,
				 HI3798CV200_APLL_TUNE_TIMEOUT_US);

	if (ret) {
		/* Cancel a timed-out request before attempting the rollback. */
		hi3798cv200_cpu_apll_update_bits(
			cpu, HI3798CV200_APLL_TUNE_CTRL,
			HI3798CV200_APLL_TUNE_MODE |
				HI3798CV200_APLL_TUNE_ENABLE,
			ctrl & (HI3798CV200_APLL_TUNE_MODE |
				HI3798CV200_APLL_TUNE_ENABLE), NULL);
		readl_poll_timeout(cpu->base + HI3798CV200_APLL_TUNE_STATUS,
				   status,
				   !(status & HI3798CV200_APLL_TUNE_BUSY),
				   HI3798CV200_APLL_POLL_DELAY_US,
				   HI3798CV200_APLL_IDLE_TIMEOUT_US);
	}

out_status:
	if (last_status)
		*last_status = status;

	return ret;
}

static unsigned long hi3798cv200_cpu_clk_recalc_rate(struct clk_hw *hw,
						     unsigned long parent_rate)
{
	struct hi3798cv200_cpu_clk *cpu = to_hi3798cv200_cpu_clk(hw);
	u32 fbdiv, selector;

	selector = readl_relaxed(cpu->base + HI3798CV200_CPU_CLK_CTRL) &
		   HI3798CV200_CPU_CLK_SEL_MASK;
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

	if (hi3798cv200_cpu_apll_snapshot(cpu, &fbdiv, &parent_rate))
		return 0;

	return parent_rate;
}

static long hi3798cv200_cpu_clk_round_rate(struct clk_hw *hw,
					   unsigned long rate,
					   unsigned long *parent_rate)
{
	unsigned long best_delta = ULONG_MAX;
	unsigned long best_rate = hi3798cv200_cpu_rates[0];
	int i;

	for (i = 0; i < ARRAY_SIZE(hi3798cv200_cpu_rates); i++) {
		unsigned long candidate = hi3798cv200_cpu_rates[i];
		unsigned long delta = candidate > rate ? candidate - rate :
							   rate - candidate;

		if (delta < best_delta) {
			best_delta = delta;
			best_rate = candidate;
		}
	}

	return best_rate;
}

static int hi3798cv200_cpu_clk_set_rate(struct clk_hw *hw,
					unsigned long rate,
					unsigned long parent_rate)
{
	struct hi3798cv200_cpu_clk *cpu = to_hi3798cv200_cpu_clk(hw);
	u32 fbdiv, old_fbdiv, postdiv1, postdiv2, refdiv, status;
	unsigned long actual, old_rate, rollback_rate;
	u64 divisor;
	bool supported = false;
	int i, ret, rollback_ret;

	for (i = 0; i < ARRAY_SIZE(hi3798cv200_cpu_rates); i++)
		if (hi3798cv200_cpu_rates[i] == rate) {
			supported = true;
			break;
		}
	if (!supported)
		return -EINVAL;

	if (hi3798cv200_cpu_pll_dividers(cpu, &postdiv1, &postdiv2, &refdiv))
		return -EINVAL;

	divisor = (u64)postdiv1 * postdiv2 * refdiv;
	fbdiv = div64_u64((u64)rate * divisor, HI3798CV200_CPU_PLL_REF_RATE);
	if (!fbdiv || fbdiv > FIELD_MAX(HI3798CV200_CPU_PLL_FBDIV_MASK))
		return -EINVAL;

	actual = div64_u64((u64)fbdiv * HI3798CV200_CPU_PLL_REF_RATE,
			   divisor);
	if (abs_diff(actual, rate) >= HI3798CV200_CPU_RATE_TOLERANCE)
		return -EINVAL;

	mutex_lock(&cpu->lock);
	if ((readl_relaxed(cpu->base + HI3798CV200_CPU_CLK_CTRL) &
	     HI3798CV200_CPU_CLK_SEL_MASK) != 0) {
		ret = -EOPNOTSUPP;
		goto unlock;
	}

	ret = hi3798cv200_cpu_apll_snapshot(cpu, &old_fbdiv, &old_rate);
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

	ret = hi3798cv200_cpu_apll_tune(cpu, fbdiv, &status);
	if (!ret) {
		ret = hi3798cv200_cpu_apll_snapshot(cpu, &fbdiv, &actual);
		if (!ret && actual == rate) {
			cpu->last_good_rate = actual;
			goto unlock;
		}
		if (!ret)
			ret = -EIO;
	}

	rollback_ret = hi3798cv200_cpu_apll_tune(cpu, old_fbdiv, &status);
	if (!rollback_ret) {
		rollback_ret = hi3798cv200_cpu_apll_snapshot(cpu, &old_fbdiv,
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
		rate, readl_relaxed(cpu->base + HI3798CV200_APLL_TUNE_STATUS));
unlock:
	mutex_unlock(&cpu->lock);
	return ret;
}

static const struct clk_ops hi3798cv200_cpu_clk_ops = {
	.recalc_rate = hi3798cv200_cpu_clk_recalc_rate,
	.round_rate = hi3798cv200_cpu_clk_round_rate,
	.set_rate = hi3798cv200_cpu_clk_set_rate,
};

static int hi3798cv200_cpu_clk_register(struct platform_device *pdev,
					struct hisi_clock_data *clk_data)
{
	struct hi3798cv200_cpu_clk *cpu;
	struct clk_init_data init = {
		.name = "clk_cpu",
		.ops = &hi3798cv200_cpu_clk_ops,
		.flags = CLK_GET_RATE_NOCACHE,
	};
	struct clk *clk;

	cpu = devm_kzalloc(&pdev->dev, sizeof(*cpu), GFP_KERNEL);
	if (!cpu)
		return -ENOMEM;

	cpu->base = clk_data->base;
	cpu->dev = &pdev->dev;
	cpu->hw.init = &init;
	mutex_init(&cpu->lock);

	clk = clk_register(&pdev->dev, &cpu->hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	clk_data->clk_data.clks[HISTB_CPU_CLK] = clk;
	return 0;
}

static void hi3798cv200_cpu_clk_unregister(struct hisi_clock_data *clk_data)
{
	clk_unregister(clk_data->clk_data.clks[HISTB_CPU_CLK]);
	clk_data->clk_data.clks[HISTB_CPU_CLK] = NULL;
}

struct hi3798cv200_hdmi_pixel_clk {
	struct clk_hw hw;
	void __iomem *base;
	struct mutex lock;
};

struct hi3798cv200_hdmi_pixel_rate {
	unsigned long request_rate;
	unsigned long rate;
	u32 frac;
	u16 fbdiv;
	u8 refdiv;
	u8 postdiv1;
	u8 postdiv2;
	bool dsmpd;
};

/*
 * These clock plans come from the Hi3798CV200 vendor 4.4 kernel mode table.
 * VESA nominal clocks do not always equal the PLL's actual output, so
 * retain both values. Keep the production divider placement and fractional-
 * mode setting: equivalent arithmetic PLL solutions have not proved
 * equivalent on this hardware.
 */
static const struct hi3798cv200_hdmi_pixel_rate
hi3798cv200_hdmi_pixel_presets[] = {
	/* Requested, actual, frac, fbdiv, refdiv, postdiv1, postdiv2, dsmpd */
	{  25175000,  25200000,        0,  168,  5, 4, 2, false }, /* 24000000/000050a8 */
	{  40000000,  39792000,        0, 1658, 25, 5, 2, false }, /* 25000000/0001967a */
	{  65000000,  64992000,        0, 1354, 25, 5, 1, false }, /* 15000000/0001954a */
	{  74176000,  74175750, 0xe6a7ef,   98,  2, 4, 1, false }, /* 14e6a7ef/00002062 */
	{  74250000,  74250000,        0,   99,  2, 4, 1, false }, /* 14000000/00002063 */
	{  83500000,  83760000,        0, 1047, 25, 3, 1, false }, /* 13000000/00019417 */
	{  85500000,  85480000,        0, 2137, 50, 3, 1, false }, /* 13000000/00032859 */
	{  85800000,  85800000,        0,  429, 10, 3, 1, false }, /* 13000000/0000a1ad */
	{  88750000,  88750000,        0,  355,  6, 4, 1, false }, /* 14000000/00006163 */
	{  97750000,  97750000,        0,  391,  6, 4, 1, false }, /* 14000000/00006187 */
	{ 106500000, 106700000,        0, 1067, 30, 2, 1, false }, /* 12000000/0001e42b */
	{ 108000000, 107952000,        0, 2249, 25, 5, 1, false }, /* 15000000/000198c9 */
	{ 121750000, 121800000,        0,  203,  5, 2, 1, false }, /* 12000000/000050cb */
	{ 146250000, 146375000,        0, 1171, 24, 2, 1, false }, /* 12000000/00018493 */
	{ 148352000, 148351500, 0xe6a7ef,   98,  2, 2, 1, false }, /* 12e6a7ef/00002062 */
	{ 148500000, 148500000,        0,   99,  2, 2, 1, false }, /* 12000000/00002063 */
	{ 154000000, 154000000,        0,  154,  3, 2, 1, false }, /* 12000000/0000309a */
	{ 156750000, 157000000,        0,  157,  3, 2, 1, false }, /* 12000000/0000309d */
	{ 162000000, 162000000,        0,   54,  1, 2, 1, false }, /* 12000000/00001036 */
	{ 234000000, 234000000,        0,   39,  1, 1, 1, false }, /* 11000000/00001027 */
	{ 241500000, 241500000,        0,  161,  2, 2, 1, false }, /* 12000000/000020a1 */
	{ 268500000, 268500000,        0,  179,  2, 2, 1, false }, /* 12000000/000020b3 */
	{ 296703000, 296703000, 0xe6a7ef,   98,  2, 1, 1, false }, /* 11e6a7ef/00002062 */
	{ 297000000, 297000000,        0,   99,  2, 1, 1, false }, /* 11000000/00002063 */
	{ 593407000, 593406000, 0xe6a7ef,   98,  2, 1, 1, false }, /* 11e6a7ef/00002062 */
	{ 594000000, 594000000,        0,   99,  2, 1, 1, false }, /* 11000000/00002063 */
};

/* Divider products and factor preference used by the vendor custom mode. */
static const u8 hi3798cv200_hpll_postdiv_products[] = {
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 15, 16, 18, 20, 21, 24,
	25, 28, 30, 35, 36, 42, 49,
};

#define to_hi3798cv200_hdmi_pixel_clk(_hw) \
	container_of(_hw, struct hi3798cv200_hdmi_pixel_clk, hw)

static bool
hi3798cv200_hdmi_pixel_find_rate(unsigned long rate,
				 unsigned long parent_rate,
				 struct hi3798cv200_hdmi_pixel_rate *best)
{
	u64 best_delta = U64_MAX;
	u8 output_divider;
	unsigned int i;

	if (!parent_rate || rate < HI3798CV200_HDMI_PIXEL_MIN_RATE ||
	    rate > HI3798CV200_HDMI_PIXEL_MAX_RATE)
		return false;

	if (parent_rate == HI3798CV200_HPLL_PARENT_RATE) {
		for (i = 0; i < ARRAY_SIZE(hi3798cv200_hdmi_pixel_presets); i++) {
			if (rate != hi3798cv200_hdmi_pixel_presets[i].request_rate &&
			    rate != hi3798cv200_hdmi_pixel_presets[i].rate)
				continue;

			*best = hi3798cv200_hdmi_pixel_presets[i];
			return true;
		}
	}

	/*
	 * The display channel contributes another fixed divide-by-two. The
	 * vendor clock plan uses VO /2 through HDMI 1.4 rates and VO /1 for
	 * HDMI 2.0 rates, yielding total output divisors of four and two.
	 */
	output_divider = rate > HI3798CV200_HDMI14_MAX_RATE ? 2 : 4;

	for (i = 0; i < ARRAY_SIZE(hi3798cv200_hpll_postdiv_products); i++) {
		u8 product = hi3798cv200_hpll_postdiv_products[i];
		u8 postdiv1, postdiv2, refdiv;
		u64 target_vco;

		/* The vendor stores the first factor in the high divider field. */
		for (postdiv2 = 1; postdiv2 <= 7; postdiv2++) {
			if (!(product % postdiv2) && product / postdiv2 <= 7)
				break;
		}
		if (postdiv2 > 7)
			continue;
		postdiv1 = product / postdiv2;
		target_vco = (u64)rate * output_divider * product;

		if (target_vco <= HI3798CV200_HPLL_VCO_MIN_RATE ||
		    target_vco >= HI3798CV200_HPLL_VCO_MAX_RATE)
			continue;

		for (refdiv = 1; refdiv <= 63; refdiv++) {
			u64 scaled, actual, delta, denominator;
			u32 frac;
			u16 fbdiv;

			scaled = DIV_ROUND_CLOSEST_ULL((target_vco * refdiv) << 24,
						       parent_rate);
			fbdiv = scaled >> 24;
			frac = scaled & HI3798CV200_HPLL_FRAC_MASK;
			if (fbdiv <= 16 || fbdiv >= 2400)
				continue;

			denominator = (u64)refdiv * product * output_divider << 24;
			actual = DIV64_U64_ROUND_CLOSEST((u64)parent_rate * scaled,
							 denominator);
			delta = actual > rate ? actual - rate : rate - actual;

			if (delta > best_delta ||
			    (delta == best_delta && (frac || !best->frac)))
				continue;

			best_delta = delta;
			best->request_rate = rate;
			best->rate = actual;
			best->frac = frac;
			best->fbdiv = fbdiv;
			best->refdiv = refdiv;
			best->postdiv1 = postdiv1;
			best->postdiv2 = postdiv2;
			best->dsmpd = !frac;
		}
	}

	return best_delta != U64_MAX;
}

static int hi3798cv200_hdmi_pixel_clk_set_rate(struct clk_hw *hw,
					       unsigned long rate,
					       unsigned long parent_rate);

static bool
hi3798cv200_hdmi_pixel_plan_matches(struct hi3798cv200_hdmi_pixel_clk *pixel,
				    unsigned long rate,
					    unsigned long parent_rate)
{
	struct hi3798cv200_hdmi_pixel_rate pixel_rate = {};
	unsigned long flags;
	u32 ctrl0, ctrl1, vo, vdp, hdmi_ctrl, pixel_ctrl, status;
	u32 vo_divider;
	bool matches;

	if (!hi3798cv200_hdmi_pixel_find_rate(rate, parent_rate, &pixel_rate))
		return false;

	vo_divider = rate > HI3798CV200_HDMI14_MAX_RATE ?
		HI3798CV200_VO_HD_CLK_DIV_1 : HI3798CV200_VO_HD_CLK_DIV_2;

	spin_lock_irqsave(&hisi_clk_lock, flags);
	ctrl0 = readl_relaxed(pixel->base + HI3798CV200_HPLL_CTRL0);
	ctrl1 = readl_relaxed(pixel->base + HI3798CV200_HPLL_CTRL1);
	vo = readl_relaxed(pixel->base + HI3798CV200_VO_CLK_CTRL);
	vdp = readl_relaxed(pixel->base + HI3798CV200_VDP_CLK_CTRL);
	hdmi_ctrl = readl_relaxed(pixel->base + HI3798CV200_HDMI_CTRL_CLK);
	pixel_ctrl = readl_relaxed(pixel->base + HI3798CV200_HDMI_PIXEL_CTRL);
	status = readl_relaxed(pixel->base + HI3798CV200_HPLL_LOCK_STATUS);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);

	matches = (status & HI3798CV200_HPLL_LOCK) &&
		FIELD_GET(HI3798CV200_HPLL_FRAC_MASK, ctrl0) ==
			pixel_rate.frac &&
		FIELD_GET(HI3798CV200_HPLL_POSTDIV1_MASK, ctrl0) ==
			pixel_rate.postdiv1 &&
		FIELD_GET(HI3798CV200_HPLL_POSTDIV2_MASK, ctrl0) ==
			pixel_rate.postdiv2 &&
		FIELD_GET(HI3798CV200_HPLL_FBDIV_MASK, ctrl1) ==
			pixel_rate.fbdiv &&
		FIELD_GET(HI3798CV200_HPLL_REFDIV_MASK, ctrl1) ==
			pixel_rate.refdiv &&
		!(ctrl1 & (HI3798CV200_HPLL_POWER_MASK |
			   HI3798CV200_HPLL_BYPASS)) &&
		!!(ctrl1 & HI3798CV200_HPLL_DSMPD) == pixel_rate.dsmpd &&
		(vo & (HI3798CV200_VO_HD_CLK_SEL |
		       HI3798CV200_HDMI_CLK_SEL)) ==
			(HI3798CV200_VO_HD_CLK_SEL | HI3798CV200_HDMI_CLK_SEL) &&
		!(vo & (HI3798CV200_VDP_CLK_SEL |
			 HI3798CV200_VO_HD_HDMI_DIV)) &&
		FIELD_GET(HI3798CV200_VO_HD_CLK_DIV_MASK, vo) == vo_divider &&
		(vdp & HI3798CV200_VDP_HD_DIV0) &&
		!(vdp & (HI3798CV200_VDP_INIT_SEL0 |
			 HI3798CV200_VO_HD_BP_CLK_SEL)) &&
		!(hdmi_ctrl & HI3798CV200_HDMI_ASCLK_SEL) &&
		(pixel_ctrl & HI3798CV200_HDMI_OSCLK_SEL) &&
		!(pixel_ctrl & (HI3798CV200_HDMI_PIXELNX_SEL |
				HI3798CV200_HDMI_IDCLK_SEL));

	return matches;
}

static int hi3798cv200_hdmi_pixel_clk_prepare(struct clk_hw *hw)
{
	struct hi3798cv200_hdmi_pixel_clk *pixel =
		to_hi3798cv200_hdmi_pixel_clk(hw);
	struct clk_hw *parent = clk_hw_get_parent(hw);
	unsigned long parent_rate, rate;

	if (!parent)
		return -EINVAL;

	parent_rate = clk_hw_get_rate(parent);
	rate = clk_hw_get_rate(hw);
	if (hi3798cv200_hdmi_pixel_plan_matches(pixel, rate, parent_rate))
		return 0;

	return hi3798cv200_hdmi_pixel_clk_set_rate(hw, rate, parent_rate);
}

static int hi3798cv200_hdmi_pixel_clk_enable(struct clk_hw *hw)
{
	struct hi3798cv200_hdmi_pixel_clk *pixel =
		to_hi3798cv200_hdmi_pixel_clk(hw);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&hisi_clk_lock, flags);
	reg = readl_relaxed(pixel->base + HI3798CV200_HDMI_PIXEL_CTRL);
	reg |= HI3798CV200_HDMI_PIXEL_GATE;
	writel_relaxed(reg, pixel->base + HI3798CV200_HDMI_PIXEL_CTRL);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);

	return 0;
}

static void hi3798cv200_hdmi_pixel_clk_disable(struct clk_hw *hw)
{
	struct hi3798cv200_hdmi_pixel_clk *pixel =
		to_hi3798cv200_hdmi_pixel_clk(hw);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&hisi_clk_lock, flags);
	reg = readl_relaxed(pixel->base + HI3798CV200_HDMI_PIXEL_CTRL);
	reg &= ~HI3798CV200_HDMI_PIXEL_GATE;
	writel_relaxed(reg, pixel->base + HI3798CV200_HDMI_PIXEL_CTRL);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
}

static int hi3798cv200_hdmi_pixel_clk_is_enabled(struct clk_hw *hw)
{
	struct hi3798cv200_hdmi_pixel_clk *pixel =
		to_hi3798cv200_hdmi_pixel_clk(hw);

	return !!(readl_relaxed(pixel->base + HI3798CV200_HDMI_PIXEL_CTRL) &
		  HI3798CV200_HDMI_PIXEL_GATE);
}

static unsigned long
hi3798cv200_hdmi_pixel_clk_recalc_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	struct hi3798cv200_hdmi_pixel_clk *pixel =
		to_hi3798cv200_hdmi_pixel_clk(hw);
	u32 ctrl0, ctrl1, frac, vo_clk_ctrl;
	u32 postdiv1, postdiv2, fbdiv, refdiv;
	u32 output_divider, vo_divider;
	u64 scaled, denominator;

	if (!(readl_relaxed(pixel->base + HI3798CV200_HPLL_LOCK_STATUS) &
	      HI3798CV200_HPLL_LOCK))
		return 0;

	ctrl0 = readl_relaxed(pixel->base + HI3798CV200_HPLL_CTRL0);
	ctrl1 = readl_relaxed(pixel->base + HI3798CV200_HPLL_CTRL1);
	if (ctrl1 & (HI3798CV200_HPLL_POWER_MASK |
		     HI3798CV200_HPLL_BYPASS))
		return 0;

	postdiv1 = FIELD_GET(HI3798CV200_HPLL_POSTDIV1_MASK, ctrl0);
	postdiv2 = FIELD_GET(HI3798CV200_HPLL_POSTDIV2_MASK, ctrl0);
	fbdiv = FIELD_GET(HI3798CV200_HPLL_FBDIV_MASK, ctrl1);
	refdiv = FIELD_GET(HI3798CV200_HPLL_REFDIV_MASK, ctrl1);
	if (!postdiv1 || !postdiv2 || !fbdiv || !refdiv)
		return 0;

	vo_clk_ctrl = readl_relaxed(pixel->base + HI3798CV200_VO_CLK_CTRL);
	vo_divider = FIELD_GET(HI3798CV200_VO_HD_CLK_DIV_MASK, vo_clk_ctrl);
	switch (vo_divider) {
	case 2:
	case HI3798CV200_VO_HD_CLK_DIV_1:
		output_divider = 2;
		break;
	case HI3798CV200_VO_HD_CLK_DIV_2:
		output_divider = 4;
		break;
	case HI3798CV200_VO_HD_CLK_DIV_4:
		output_divider = 8;
		break;
	default:
		return 0;
	}

	frac = ctrl1 & HI3798CV200_HPLL_DSMPD ?
		0 : FIELD_GET(HI3798CV200_HPLL_FRAC_MASK, ctrl0);
	scaled = ((u64)fbdiv << 24) | frac;
	denominator = (u64)refdiv * postdiv1 * postdiv2 *
		      output_divider << 24;

	return DIV64_U64_ROUND_CLOSEST((u64)parent_rate * scaled,
				       denominator);
}

static long
hi3798cv200_hdmi_pixel_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	struct hi3798cv200_hdmi_pixel_rate pixel_rate = {};

	if (!hi3798cv200_hdmi_pixel_find_rate(rate, *parent_rate, &pixel_rate))
		return -EINVAL;

	return pixel_rate.rate;
}

static int hi3798cv200_hdmi_pixel_clk_set_rate(struct clk_hw *hw,
					       unsigned long rate,
					       unsigned long parent_rate)
{
	struct hi3798cv200_hdmi_pixel_clk *pixel =
		to_hi3798cv200_hdmi_pixel_clk(hw);
	struct hi3798cv200_hdmi_pixel_rate pixel_rate = {};
	unsigned long flags;
	u32 old_ctrl0, old_ctrl1, old_vo_clk_ctrl, old_vdp_clk_ctrl;
	u32 old_hdmi_ctrl_clk, old_hdmi_pixel_ctrl;
	bool old_locked;
	u32 reg, status;
	int ret, rollback_ret = 0;

	if (!hi3798cv200_hdmi_pixel_find_rate(rate, parent_rate, &pixel_rate))
		return -EINVAL;

	mutex_lock(&pixel->lock);
	spin_lock_irqsave(&hisi_clk_lock, flags);
	old_ctrl0 = readl_relaxed(pixel->base + HI3798CV200_HPLL_CTRL0);
	old_ctrl1 = readl_relaxed(pixel->base + HI3798CV200_HPLL_CTRL1);
	old_vo_clk_ctrl = readl_relaxed(pixel->base + HI3798CV200_VO_CLK_CTRL);
	old_vdp_clk_ctrl = readl_relaxed(pixel->base + HI3798CV200_VDP_CLK_CTRL);
	old_hdmi_ctrl_clk = readl_relaxed(pixel->base +
					   HI3798CV200_HDMI_CTRL_CLK);
	old_hdmi_pixel_ctrl = readl_relaxed(pixel->base +
					     HI3798CV200_HDMI_PIXEL_CTRL);
	old_locked = !!(readl_relaxed(pixel->base +
					       HI3798CV200_HPLL_LOCK_STATUS) &
				 HI3798CV200_HPLL_LOCK);

	/* Stop the pixel output while HPLL and its downstream routes change. */
	reg = old_hdmi_pixel_ctrl & ~HI3798CV200_HDMI_PIXEL_GATE;
	writel_relaxed(reg, pixel->base + HI3798CV200_HDMI_PIXEL_CTRL);
	readl_relaxed(pixel->base + HI3798CV200_HDMI_PIXEL_CTRL);

	reg = readl_relaxed(pixel->base + HI3798CV200_HPLL_CTRL0);
	reg &= ~(HI3798CV200_HPLL_FRAC_MASK |
		 HI3798CV200_HPLL_POSTDIV1_MASK |
		 HI3798CV200_HPLL_POSTDIV2_MASK);
	reg |= FIELD_PREP(HI3798CV200_HPLL_FRAC_MASK, pixel_rate.frac) |
	       FIELD_PREP(HI3798CV200_HPLL_POSTDIV1_MASK,
			  pixel_rate.postdiv1) |
	       FIELD_PREP(HI3798CV200_HPLL_POSTDIV2_MASK,
			  pixel_rate.postdiv2);
	writel_relaxed(reg, pixel->base + HI3798CV200_HPLL_CTRL0);

	reg = readl_relaxed(pixel->base + HI3798CV200_HPLL_CTRL1);
	reg &= ~(HI3798CV200_HPLL_FBDIV_MASK |
		 HI3798CV200_HPLL_REFDIV_MASK |
		 HI3798CV200_HPLL_POWER_MASK |
		 HI3798CV200_HPLL_DSMPD |
		 HI3798CV200_HPLL_BYPASS);
	reg |= FIELD_PREP(HI3798CV200_HPLL_FBDIV_MASK, pixel_rate.fbdiv) |
	       FIELD_PREP(HI3798CV200_HPLL_REFDIV_MASK, pixel_rate.refdiv);
	if (pixel_rate.dsmpd)
		reg |= HI3798CV200_HPLL_DSMPD;
	writel_relaxed(reg, pixel->base + HI3798CV200_HPLL_CTRL1);

	/* Select HPLL and the non-interlaced, non-YUV420 display path. */
	reg = readl_relaxed(pixel->base + HI3798CV200_VO_CLK_CTRL);
	reg &= ~(HI3798CV200_VO_HD_CLK_DIV_MASK |
		 HI3798CV200_VDP_CLK_SEL |
		 HI3798CV200_VO_HD_HDMI_DIV);
	reg |= HI3798CV200_VO_HD_CLK_SEL | HI3798CV200_HDMI_CLK_SEL |
	       FIELD_PREP(HI3798CV200_VO_HD_CLK_DIV_MASK,
			  rate > HI3798CV200_HDMI14_MAX_RATE ?
			  HI3798CV200_VO_HD_CLK_DIV_1 :
			  HI3798CV200_VO_HD_CLK_DIV_2);
	writel_relaxed(reg, pixel->base + HI3798CV200_VO_CLK_CTRL);

	reg = readl_relaxed(pixel->base + HI3798CV200_VDP_CLK_CTRL);
	reg &= ~(HI3798CV200_VDP_INIT_SEL0 |
		 HI3798CV200_VO_HD_BP_CLK_SEL);
	reg |= HI3798CV200_VDP_HD_DIV0;
	writel_relaxed(reg, pixel->base + HI3798CV200_VDP_CLK_CTRL);

	reg = readl_relaxed(pixel->base + HI3798CV200_HDMI_CTRL_CLK);
	reg &= ~HI3798CV200_HDMI_ASCLK_SEL;
	writel_relaxed(reg, pixel->base + HI3798CV200_HDMI_CTRL_CLK);

	reg = readl_relaxed(pixel->base + HI3798CV200_HDMI_PIXEL_CTRL);
	reg &= ~(HI3798CV200_HDMI_PIXELNX_SEL |
		 HI3798CV200_HDMI_IDCLK_SEL);
	reg |= HI3798CV200_HDMI_OSCLK_SEL;
	reg &= ~HI3798CV200_HDMI_PIXEL_GATE;
	writel_relaxed(reg, pixel->base + HI3798CV200_HDMI_PIXEL_CTRL);

	spin_unlock_irqrestore(&hisi_clk_lock, flags);

	ret = readl_poll_timeout(pixel->base + HI3798CV200_HPLL_LOCK_STATUS,
				 status, status & HI3798CV200_HPLL_LOCK,
				 1, 1000);
	if (ret) {
		spin_lock_irqsave(&hisi_clk_lock, flags);
		writel_relaxed(old_ctrl0,
			       pixel->base + HI3798CV200_HPLL_CTRL0);
		writel_relaxed(old_ctrl1,
			       pixel->base + HI3798CV200_HPLL_CTRL1);
		writel_relaxed(old_vo_clk_ctrl,
			       pixel->base + HI3798CV200_VO_CLK_CTRL);
		writel_relaxed(old_vdp_clk_ctrl,
			       pixel->base + HI3798CV200_VDP_CLK_CTRL);
		writel_relaxed(old_hdmi_ctrl_clk,
			       pixel->base + HI3798CV200_HDMI_CTRL_CLK);
		writel_relaxed(old_hdmi_pixel_ctrl,
			       pixel->base + HI3798CV200_HDMI_PIXEL_CTRL);
		spin_unlock_irqrestore(&hisi_clk_lock, flags);

		if (old_locked)
			rollback_ret = readl_poll_timeout(
				pixel->base + HI3798CV200_HPLL_LOCK_STATUS,
				status, status & HI3798CV200_HPLL_LOCK,
				1, 1000);

		if (rollback_ret)
			pr_err_ratelimited("hi3798cv200-crg: HDMI HPLL rollback did not relock\n");
	} else if (old_hdmi_pixel_ctrl & HI3798CV200_HDMI_PIXEL_GATE) {
		/* Restore an already-running stream only after the new HPLL locks. */
		spin_lock_irqsave(&hisi_clk_lock, flags);
		reg = readl_relaxed(pixel->base + HI3798CV200_HDMI_PIXEL_CTRL);
		reg |= HI3798CV200_HDMI_PIXEL_GATE;
		writel_relaxed(reg, pixel->base + HI3798CV200_HDMI_PIXEL_CTRL);
		spin_unlock_irqrestore(&hisi_clk_lock, flags);
	}
	mutex_unlock(&pixel->lock);

	return ret;
}

static const struct clk_ops hi3798cv200_hdmi_pixel_clk_ops = {
	.prepare = hi3798cv200_hdmi_pixel_clk_prepare,
	.enable = hi3798cv200_hdmi_pixel_clk_enable,
	.disable = hi3798cv200_hdmi_pixel_clk_disable,
	.is_enabled = hi3798cv200_hdmi_pixel_clk_is_enabled,
	.recalc_rate = hi3798cv200_hdmi_pixel_clk_recalc_rate,
	.round_rate = hi3798cv200_hdmi_pixel_clk_round_rate,
	.set_rate = hi3798cv200_hdmi_pixel_clk_set_rate,
};

static int
hi3798cv200_hdmi_pixel_clk_register(struct platform_device *pdev,
				    struct hisi_clock_data *clk_data)
{
	static const char *const parent_names[] = { "clk_osc" };
	struct hi3798cv200_hdmi_pixel_clk *pixel;
	struct clk_init_data init = {
		.name = "clk_hdmi_pixel",
		.ops = &hi3798cv200_hdmi_pixel_clk_ops,
		.parent_names = parent_names,
		.num_parents = ARRAY_SIZE(parent_names),
		/* The HDMI bridge gates this dedicated clock around set_rate(). */
		.flags = CLK_GET_RATE_NOCACHE,
	};
	struct clk *clk;

	pixel = devm_kzalloc(&pdev->dev, sizeof(*pixel), GFP_KERNEL);
	if (!pixel)
		return -ENOMEM;

	pixel->base = clk_data->base;
	pixel->hw.init = &init;
	mutex_init(&pixel->lock);

	clk = clk_register(&pdev->dev, &pixel->hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	clk_data->clk_data.clks[HISTB_HDMI_PIXEL_CLK] = clk;

	return 0;
}

static void
hi3798cv200_hdmi_pixel_clk_unregister(struct hisi_clock_data *clk_data)
{
	clk_unregister(clk_data->clk_data.clks[HISTB_HDMI_PIXEL_CLK]);
	clk_data->clk_data.clks[HISTB_HDMI_PIXEL_CLK] = NULL;
}

static const struct hisi_fixed_rate_clock hi3798cv200_fixed_rate_clks[] = {
	{ HISTB_OSC_CLK, "clk_osc", NULL, 0, 24000000, },
	{ HISTB_APB_CLK, "clk_apb", NULL, 0, 100000000, },
	{ HISTB_AHB_CLK, "clk_ahb", NULL, 0, 200000000, },
	/* Independent external TSI0..3 inputs have no single known rate. */
	{ HISTB_TSI_INDEPENDENT_CLK, "tsi_independent", NULL, 0, 0, },
	{ HI3798CV200_FIXED_12M, "12m", NULL, 0, 12000000, },
	{ HI3798CV200_FIXED_24M, "24m", NULL, 0, 24000000, },
	{ HI3798CV200_FIXED_25M, "25m", NULL, 0, 25000000, },
	{ HI3798CV200_FIXED_48M, "48m", NULL, 0, 48000000, },
	{ HI3798CV200_FIXED_50M, "50m", NULL, 0, 50000000, },
	{ HI3798CV200_FIXED_60M, "60m", NULL, 0, 60000000, },
	{ HI3798CV200_FIXED_75M, "75m", NULL, 0, 75000000, },
	{ HI3798CV200_FIXED_100M, "100m", NULL, 0, 100000000, },
	{ HI3798CV200_FIXED_150M, "150m", NULL, 0, 150000000, },
	{ HI3798CV200_FIXED_166P5M, "166p5m", NULL, 0, 165000000, },
	{ HI3798CV200_FIXED_200M, "200m", NULL, 0, 200000000, },
	{ HI3798CV200_FIXED_250M, "250m", NULL, 0, 250000000, },
	{ HI3798CV200_FIXED_270M, "270m", NULL, 0, 270000000, },
	{ HI3798CV200_FIXED_300M, "300m", NULL, 0, 300000000, },
	{ HI3798CV200_FIXED_333M, "333m", NULL, 0, 333000000, },
	{ HI3798CV200_FIXED_400M, "400m", NULL, 0, 400000000, },
	{ HI3798CV200_FIXED_933P888M, "933p888m", NULL, 0, 933888000, },
};

static const char *const mmc_mux_p[] = {
		"100m", "50m", "25m", "200m", "150m" };
static u32 mmc_mux_table[] = {0, 1, 2, 3, 6};

static const char *const comphy_mux_p[] = {
		"100m", "25m"};
static u32 comphy_mux_table[] = {2, 3};

static const char *const sdio_mux_p[] = {
		"100m", "50m", "150m", "166p5m" };
static u32 sdio_mux_table[] = {0, 1, 2, 3};

static const char *const aiao_mux_p[] = { "100m", "50m" };
static u32 aiao_mux_table[] = { 0, 1 };

static const char *const aiao_mclk_mux_p[] = { "933p888m" };
static u32 aiao_mclk_mux_table[] = { 0 };

static const char *const vpss_mux_p[] = { "333m", "400m", "300m" };
static u32 vpss_mux_table[] = { 0, 1, 2 };

static const char *const venc_mux_p[] = { "270m", "200m" };
static u32 venc_mux_table[] = { 0, 1 };

static const char *const tsi_shared_mux_p[] = {
	"clk_tsi0", "clk_tsi1", "clk_tsi2", "clk_tsi3", "tsi_independent"
};

static u32 tsi_shared_mux_table[] = { 4, 5, 6, 7, 0 };

static struct hisi_mux_clock hi3798cv200_mux_clks[] = {
	{ HI3798CV200_MMC_MUX, "mmc_mux", mmc_mux_p, ARRAY_SIZE(mmc_mux_p),
		CLK_SET_RATE_PARENT, 0xa0, 8, 3, 0, mmc_mux_table, },
	{ HI3798CV200_COMBPHY0_MUX, "combphy0_mux",
		comphy_mux_p, ARRAY_SIZE(comphy_mux_p),
		CLK_SET_RATE_PARENT, 0x188, 2, 2, 0, comphy_mux_table, },
	{ HI3798CV200_COMBPHY1_MUX, "combphy1_mux",
		comphy_mux_p, ARRAY_SIZE(comphy_mux_p),
		CLK_SET_RATE_PARENT, 0x188, 10, 2, 0, comphy_mux_table, },
	{ HI3798CV200_SDIO0_MUX, "sdio0_mux", sdio_mux_p,
		ARRAY_SIZE(sdio_mux_p), CLK_SET_RATE_PARENT,
		0x9c, 8, 2, 0, sdio_mux_table, },
	{ HI3798CV200_AIAO_MUX, "aiao_mux", aiao_mux_p,
		ARRAY_SIZE(aiao_mux_p), CLK_SET_RATE_PARENT,
		0x118, 22, 1, 0, aiao_mux_table, },
	{ HISTB_AIAO_MCLK, "aiao_mclk_mux", aiao_mclk_mux_p,
		ARRAY_SIZE(aiao_mclk_mux_p), CLK_SET_RATE_PARENT,
		0x118, 20, 2, 0, aiao_mclk_mux_table, },
	{ HISTB_TSI_SHARED_CLK, "tsi_shared_mux", tsi_shared_mux_p,
		ARRAY_SIZE(tsi_shared_mux_p), CLK_SET_RATE_PARENT,
		0x100, 24, 3, 0, tsi_shared_mux_table, },
	{ HI3798CV200_VPSS_MUX, "vpss_mux", vpss_mux_p,
		ARRAY_SIZE(vpss_mux_p), CLK_SET_RATE_PARENT,
		0xf0, 8, 2, 0, vpss_mux_table, },
	{ HI3798CV200_VENC_MUX, "venc_mux", venc_mux_p,
		ARRAY_SIZE(venc_mux_p), CLK_SET_RATE_PARENT,
		0x8c, 8, 1, 0, venc_mux_table, },
};

static u32 mmc_phase_regvals[] = {0, 1, 2, 3, 4, 5, 6, 7};
static u32 mmc_phase_degrees[] = {0, 45, 90, 135, 180, 225, 270, 315};

static struct hisi_phase_clock hi3798cv200_phase_clks[] = {
	{ HISTB_MMC_SAMPLE_CLK, "mmc_sample", "clk_mmc_ciu",
		CLK_SET_RATE_PARENT, 0xa0, 12, 3, mmc_phase_degrees,
		mmc_phase_regvals, ARRAY_SIZE(mmc_phase_regvals) },
	{ HISTB_MMC_DRV_CLK, "mmc_drive", "clk_mmc_ciu",
		CLK_SET_RATE_PARENT, 0xa0, 16, 3, mmc_phase_degrees,
		mmc_phase_regvals, ARRAY_SIZE(mmc_phase_regvals) },
};

static const struct hisi_gate_clock hi3798cv200_gate_clks[] = {
	/* UART */
	{ HISTB_UART2_CLK, "clk_uart2", "75m",
		CLK_SET_RATE_PARENT, 0x68, 4, 0, },
	/* I2C */
	{ HISTB_I2C0_CLK, "clk_i2c0", "clk_apb",
		CLK_SET_RATE_PARENT, 0x6C, 4, 0, },
	{ HISTB_I2C1_CLK, "clk_i2c1", "clk_apb",
		CLK_SET_RATE_PARENT, 0x6C, 8, 0, },
	{ HISTB_I2C2_CLK, "clk_i2c2", "clk_apb",
		CLK_SET_RATE_PARENT, 0x6C, 12, 0, },
	{ HISTB_I2C3_CLK, "clk_i2c3", "clk_apb",
		CLK_SET_RATE_PARENT, 0x6C, 16, 0, },
	{ HISTB_I2C4_CLK, "clk_i2c4", "clk_apb",
		CLK_SET_RATE_PARENT, 0x6C, 20, 0, },
	/* SPI */
	{ HISTB_SPI0_CLK, "clk_spi0", "clk_apb",
		CLK_SET_RATE_PARENT, 0x70, 0, 0, },
	/* Smart card interface */
	{ HISTB_SCI_CLK, "clk_sci", "60m",
		CLK_SET_RATE_PARENT, 0x74, 0, 0, },
	/* Video decoder */
	{ HISTB_VDH_CLK, "clk_vdh", NULL,
		0, 0x78, 0, 0, },
	{ HISTB_VDH_DSP_CLK, "clk_vdh_dsp", NULL,
		0, 0x78, 1, 0, },
	{ HISTB_BPD_CLK, "clk_bpd", NULL,
		0, 0x88, 0, 0, },
	/* Video encoder */
	{ HISTB_VENC_CORE_CLK, "clk_venc_core", "venc_mux",
		CLK_SET_RATE_PARENT, 0x8c, 0, 0, },
	{ HISTB_VENC_AXI_CLK, "clk_venc_axi", NULL,
		0, 0x8c, 1, 0, },
	/* JPEG encoder */
	{ HISTB_JPGE_CLK, "clk_jpge", NULL,
		0, 0x90, 0, 0, },
	/* Video post-processing subsystem */
	{ HISTB_VPSS_CLK, "clk_vpss", "vpss_mux",
		CLK_SET_RATE_PARENT, 0xf0, 0, 0, },
	/* SDIO */
	{ HISTB_SDIO0_BIU_CLK, "clk_sdio0_biu", "200m",
			CLK_SET_RATE_PARENT, 0x9c, 0, 0, },
	{ HISTB_SDIO0_CIU_CLK, "clk_sdio0_ciu", "sdio0_mux",
		CLK_SET_RATE_PARENT, 0x9c, 1, 0, },
	/* EMMC */
	{ HISTB_MMC_BIU_CLK, "clk_mmc_biu", "200m",
		CLK_SET_RATE_PARENT, 0xa0, 0, 0, },
	{ HISTB_MMC_CIU_CLK, "clk_mmc_ciu", "mmc_mux",
		CLK_SET_RATE_PARENT, 0xa0, 1, 0, },
	/* PCIE*/
	{ HISTB_PCIE_BUS_CLK, "clk_pcie_bus", "200m",
		CLK_SET_RATE_PARENT, 0x18c, 0, 0, },
	{ HISTB_PCIE_SYS_CLK, "clk_pcie_sys", "100m",
		CLK_SET_RATE_PARENT, 0x18c, 1, 0, },
	{ HISTB_PCIE_PIPE_CLK, "clk_pcie_pipe", "250m",
		CLK_SET_RATE_PARENT, 0x18c, 2, 0, },
	{ HISTB_PCIE_AUX_CLK, "clk_pcie_aux", "24m",
		CLK_SET_RATE_PARENT, 0x18c, 3, 0, },
	/* Ethernet */
	{ HI3798CV200_ETH_PUB_CLK, "clk_pub", NULL,
		CLK_SET_RATE_PARENT, 0xcc, 5, 0, },
	{ HI3798CV200_ETH_BUS_CLK, "clk_bus", "clk_pub",
		CLK_SET_RATE_PARENT, 0xcc, 0, 0, },
	{ HI3798CV200_ETH_BUS0_CLK, "clk_bus_m0", "clk_bus",
		CLK_SET_RATE_PARENT, 0xcc, 1, 0, },
	{ HI3798CV200_ETH_BUS1_CLK, "clk_bus_m1", "clk_bus",
		CLK_SET_RATE_PARENT, 0xcc, 2, 0, },
	{ HISTB_ETH0_MAC_CLK, "clk_mac0", "clk_bus_m0",
		CLK_SET_RATE_PARENT, 0xcc, 3, 0, },
	{ HISTB_ETH0_MACIF_CLK, "clk_macif0", "clk_bus_m0",
		CLK_SET_RATE_PARENT, 0xcc, 24, 0, },
	{ HISTB_ETH1_MAC_CLK, "clk_mac1", "clk_bus_m1",
		CLK_SET_RATE_PARENT, 0xcc, 4, 0, },
	{ HISTB_ETH1_MACIF_CLK, "clk_macif1", "clk_bus_m1",
		CLK_SET_RATE_PARENT, 0xcc, 25, 0, },
	/* COMBPHY0 */
	{ HISTB_COMBPHY0_CLK, "clk_combphy0", "combphy0_mux",
		CLK_SET_RATE_PARENT, 0x188, 0, 0, },
	/* COMBPHY1 */
	{ HISTB_COMBPHY1_CLK, "clk_combphy1", "combphy1_mux",
		CLK_SET_RATE_PARENT, 0x188, 8, 0, },
	/* USB2 */
	{ HISTB_USB2_BUS_CLK, "clk_u2_bus", "clk_ahb",
		CLK_SET_RATE_PARENT, 0xb8, 0, 0, },
	{ HISTB_USB2_PHY_CLK, "clk_u2_phy", "60m",
		CLK_SET_RATE_PARENT, 0xb8, 4, 0, },
	{ HISTB_USB2_12M_CLK, "clk_u2_12m", "12m",
		CLK_SET_RATE_PARENT, 0xb8, 2, 0 },
	{ HISTB_USB2_48M_CLK, "clk_u2_48m", "48m",
		CLK_SET_RATE_PARENT, 0xb8, 1, 0 },
	{ HISTB_USB2_UTMI_CLK, "clk_u2_utmi", "60m",
		CLK_SET_RATE_PARENT, 0xb8, 5, 0 },
	{ HISTB_USB2_OTG_UTMI_CLK, "clk_u2_otg_utmi", "60m",
		CLK_SET_RATE_PARENT, 0xb8, 3, 0 },
	{ HISTB_USB2_PHY1_REF_CLK, "clk_u2_phy1_ref", "24m",
		CLK_SET_RATE_PARENT, 0xbc, 0, 0 },
	{ HISTB_USB2_PHY2_REF_CLK, "clk_u2_phy2_ref", "24m",
		CLK_SET_RATE_PARENT, 0xbc, 2, 0 },
	/* USB3 */
	{ HISTB_USB3_BUS_CLK, "clk_u3_bus", NULL,
		CLK_SET_RATE_PARENT, 0xb0, 0, 0 },
	{ HISTB_USB3_UTMI_CLK, "clk_u3_utmi", NULL,
		CLK_SET_RATE_PARENT, 0xb0, 4, 0 },
	{ HISTB_USB3_PIPE_CLK, "clk_u3_pipe", NULL,
		CLK_SET_RATE_PARENT, 0xb0, 3, 0 },
	{ HISTB_USB3_SUSPEND_CLK, "clk_u3_suspend", NULL,
		CLK_SET_RATE_PARENT, 0xb0, 2, 0 },
	{ HISTB_USB3_BUS_CLK1, "clk_u3_bus1", NULL,
		CLK_SET_RATE_PARENT, 0xb0, 16, 0 },
	{ HISTB_USB3_UTMI_CLK1, "clk_u3_utmi1", NULL,
		CLK_SET_RATE_PARENT, 0xb0, 20, 0 },
	{ HISTB_USB3_PIPE_CLK1, "clk_u3_pipe1", NULL,
		CLK_SET_RATE_PARENT, 0xb0, 19, 0 },
	{ HISTB_USB3_SUSPEND_CLK1, "clk_u3_suspend1", NULL,
		CLK_SET_RATE_PARENT, 0xb0, 18, 0 },
	/* Audio */
	{ HISTB_AIAO_CLK, "clk_aiao", "aiao_mux",
		CLK_SET_RATE_PARENT, 0x118, 0, 0 },
	{ HISTB_ADAC_CLK, "clk_adac", NULL,
		0, 0x114, 0, 0 },
	/* Video output and display processor */
	{ HISTB_VO_BUS_CLK, "clk_vo_bus", NULL,
		0, 0xd8, 0, 0 },
	{ HISTB_VO_SD_CLK, "clk_vo_sd", NULL,
		0, 0xd8, 2, 0 },
	{ HISTB_VO_SDATE_CLK, "clk_vo_sdate", NULL,
		0, 0xd8, 3, 0 },
	{ HISTB_VO_HD_CLK, "clk_vo_hd", NULL,
		0, 0xd8, 4, 0 },
	{ HISTB_VDP_CFG_CLK, "clk_vdp_cfg", NULL,
		0, 0xd8, 31, 0 },
	{ HISTB_VDP_G0_CLK, "clk_vdp_g0", NULL,
		0, 0x34c, 11, 0 },
	{ HISTB_VDP_V0_CLK, "clk_vdp_v0", NULL,
		0, 0x34c, 13, 0 },
	{ HISTB_VDP_HD_CLK, "clk_vdp_hd", NULL,
		0, 0x34c, 14, 0 },
	{ HISTB_VO_BP_CLK, "clk_vo_bp", NULL,
		0, 0x34c, 0, 0 },
	/* HDMI transmitter */
	{ HISTB_HDMI_BUS_CLK, "clk_hdmi_bus", NULL,
		0, 0x10c, 0, 0 },
	{ HISTB_HDMI_CEC_CLK, "clk_hdmi_cec", NULL,
		0, 0x10c, 1, 0 },
	{ HISTB_HDMI_ID_CLK, "clk_hdmi_id", NULL,
		0, 0x10c, 2, 0 },
	{ HISTB_HDMI_MHL_CLK, "clk_hdmi_mhl", NULL,
		0, 0x10c, 3, 0 },
	{ HISTB_HDMI_OS_CLK, "clk_hdmi_os", NULL,
		0, 0x10c, 4, 0 },
	{ HISTB_HDMI_AS_CLK, "clk_hdmi_as", NULL,
		0, 0x10c, 5, 0 },
	{ HISTB_HDMI_X_CLK, "clk_hdmi_x", NULL,
		0, 0x278, 0, 0 },
	{ HISTB_HDMI_PIXEL_NX_CLK, "clk_hdmi_pixel_nx", NULL,
		0, 0x278, 1, 0 },
	/* Demux and transport stream inputs */
	{ HISTB_DMX_BUS_CLK, "clk_dmx_bus", NULL,
		0, 0xfc, 0, 0 },
	{ HISTB_DMX_CLK, "clk_dmx", NULL,
		0, 0xfc, 1, 0 },
	{ HISTB_DMX_27M_CLK, "clk_dmx_27m", NULL,
		0, 0xfc, 2, 0 },
};

static struct hisi_clock_data *hi3798cv200_clk_register(
				struct platform_device *pdev)
{
	struct hisi_clock_data *clk_data;
	int ret;

	clk_data = hisi_clk_alloc(pdev, HI3798CV200_CRG_NR_CLKS);
	if (!clk_data)
		return ERR_PTR(-ENOMEM);

	/* hisi_phase_clock is resource managed */
	ret = hisi_clk_register_phase(&pdev->dev,
				hi3798cv200_phase_clks,
				ARRAY_SIZE(hi3798cv200_phase_clks),
				clk_data);
	if (ret)
		return ERR_PTR(ret);

	ret = hisi_clk_register_fixed_rate(hi3798cv200_fixed_rate_clks,
				     ARRAY_SIZE(hi3798cv200_fixed_rate_clks),
				     clk_data);
	if (ret)
		return ERR_PTR(ret);

	ret = hisi_clk_register_mux(hi3798cv200_mux_clks,
				ARRAY_SIZE(hi3798cv200_mux_clks),
				clk_data);
	if (ret)
		goto unregister_fixed_rate;

	ret = hisi_clk_register_gate(hi3798cv200_gate_clks,
				ARRAY_SIZE(hi3798cv200_gate_clks),
				clk_data);
	if (ret)
		goto unregister_mux;

	ret = hi3798cv200_tsi_clks_register(pdev, clk_data);
	if (ret)
		goto unregister_gate;

	ret = hi3798cv200_cpu_clk_register(pdev, clk_data);
	if (ret)
		goto unregister_tsi;

	ret = hi3798cv200_gpu_clk_register(pdev, clk_data);
	if (ret)
		goto unregister_cpu;

	ret = hi3798cv200_hdmi_pixel_clk_register(pdev, clk_data);
	if (ret)
		goto unregister_gpu;

	ret = of_clk_add_provider(pdev->dev.of_node,
			of_clk_src_onecell_get, &clk_data->clk_data);
	if (ret)
		goto unregister_hdmi_pixel;

	return clk_data;

unregister_hdmi_pixel:
	hi3798cv200_hdmi_pixel_clk_unregister(clk_data);
unregister_gpu:
	hi3798cv200_gpu_clk_unregister(clk_data);
unregister_cpu:
	hi3798cv200_cpu_clk_unregister(clk_data);
unregister_tsi:
	hi3798cv200_tsi_clks_unregister(clk_data);
unregister_gate:
	hisi_clk_unregister_gate(hi3798cv200_gate_clks,
				ARRAY_SIZE(hi3798cv200_gate_clks),
				clk_data);
unregister_mux:
	hisi_clk_unregister_mux(hi3798cv200_mux_clks,
				ARRAY_SIZE(hi3798cv200_mux_clks),
				clk_data);
unregister_fixed_rate:
	hisi_clk_unregister_fixed_rate(hi3798cv200_fixed_rate_clks,
				ARRAY_SIZE(hi3798cv200_fixed_rate_clks),
				clk_data);
	return ERR_PTR(ret);
}

static void hi3798cv200_clk_unregister(struct platform_device *pdev)
{
	struct hisi_crg_dev *crg = platform_get_drvdata(pdev);

	of_clk_del_provider(pdev->dev.of_node);
	hi3798cv200_hdmi_pixel_clk_unregister(crg->clk_data);
	hi3798cv200_gpu_clk_unregister(crg->clk_data);
	hi3798cv200_cpu_clk_unregister(crg->clk_data);
	hi3798cv200_tsi_clks_unregister(crg->clk_data);

	hisi_clk_unregister_gate(hi3798cv200_gate_clks,
				ARRAY_SIZE(hi3798cv200_gate_clks),
				crg->clk_data);
	hisi_clk_unregister_mux(hi3798cv200_mux_clks,
				ARRAY_SIZE(hi3798cv200_mux_clks),
				crg->clk_data);
	hisi_clk_unregister_fixed_rate(hi3798cv200_fixed_rate_clks,
				ARRAY_SIZE(hi3798cv200_fixed_rate_clks),
				crg->clk_data);
}

static const struct hisi_crg_funcs hi3798cv200_crg_funcs = {
	.register_clks = hi3798cv200_clk_register,
	.unregister_clks = hi3798cv200_clk_unregister,
};

/* hi3798CV200 sysctrl CRG */

#define HI3798CV200_SYSCTRL_NR_CLKS 16

static const struct hisi_gate_clock hi3798cv200_sysctrl_gate_clks[] = {
	{ HISTB_IR_CLK, "clk_ir", "24m",
		CLK_SET_RATE_PARENT, 0x48, 4, 0, },
	{ HISTB_TIMER01_CLK, "clk_timer01", "24m",
		CLK_SET_RATE_PARENT, 0x48, 6, 0, },
	{ HISTB_UART0_CLK, "clk_uart0", "75m",
		CLK_SET_RATE_PARENT, 0x48, 10, 0, },
};

static struct hisi_clock_data *hi3798cv200_sysctrl_clk_register(
					struct platform_device *pdev)
{
	struct hisi_clock_data *clk_data;
	int ret;

	clk_data = hisi_clk_alloc(pdev, HI3798CV200_SYSCTRL_NR_CLKS);
	if (!clk_data)
		return ERR_PTR(-ENOMEM);

	ret = hisi_clk_register_gate(hi3798cv200_sysctrl_gate_clks,
				ARRAY_SIZE(hi3798cv200_sysctrl_gate_clks),
				clk_data);
	if (ret)
		return ERR_PTR(ret);

	ret = of_clk_add_provider(pdev->dev.of_node,
			of_clk_src_onecell_get, &clk_data->clk_data);
	if (ret)
		goto unregister_gate;

	return clk_data;

unregister_gate:
	hisi_clk_unregister_gate(hi3798cv200_sysctrl_gate_clks,
				ARRAY_SIZE(hi3798cv200_sysctrl_gate_clks),
				clk_data);
	return ERR_PTR(ret);
}

static void hi3798cv200_sysctrl_clk_unregister(struct platform_device *pdev)
{
	struct hisi_crg_dev *crg = platform_get_drvdata(pdev);

	of_clk_del_provider(pdev->dev.of_node);

	hisi_clk_unregister_gate(hi3798cv200_sysctrl_gate_clks,
				ARRAY_SIZE(hi3798cv200_sysctrl_gate_clks),
				crg->clk_data);
}

static const struct hisi_crg_funcs hi3798cv200_sysctrl_funcs = {
	.register_clks = hi3798cv200_sysctrl_clk_register,
	.unregister_clks = hi3798cv200_sysctrl_clk_unregister,
};

static const struct of_device_id hi3798cv200_crg_match_table[] = {
	{ .compatible = "hisilicon,hi3798cv200-crg",
		.data = &hi3798cv200_crg_funcs },
	{ .compatible = "hisilicon,hi3798cv200-sysctrl",
		.data = &hi3798cv200_sysctrl_funcs },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3798cv200_crg_match_table);

static int hi3798cv200_crg_probe(struct platform_device *pdev)
{
	struct hisi_crg_dev *crg;

	crg = devm_kmalloc(&pdev->dev, sizeof(*crg), GFP_KERNEL);
	if (!crg)
		return -ENOMEM;

	crg->funcs = of_device_get_match_data(&pdev->dev);
	if (!crg->funcs)
		return -ENOENT;

	crg->rstc = hisi_reset_init(pdev);
	if (!crg->rstc)
		return -ENOMEM;

	crg->clk_data = crg->funcs->register_clks(pdev);
	if (IS_ERR(crg->clk_data)) {
		hisi_reset_exit(crg->rstc);
		return PTR_ERR(crg->clk_data);
	}

	platform_set_drvdata(pdev, crg);
	return 0;
}

static void hi3798cv200_crg_remove(struct platform_device *pdev)
{
	struct hisi_crg_dev *crg = platform_get_drvdata(pdev);

	hisi_reset_exit(crg->rstc);
	crg->funcs->unregister_clks(pdev);
}

static struct platform_driver hi3798cv200_crg_driver = {
	.probe          = hi3798cv200_crg_probe,
	.remove		= hi3798cv200_crg_remove,
	.driver         = {
		.name   = "hi3798cv200-crg",
		.of_match_table = hi3798cv200_crg_match_table,
	},
};

static int __init hi3798cv200_crg_init(void)
{
	return platform_driver_register(&hi3798cv200_crg_driver);
}
core_initcall(hi3798cv200_crg_init);

static void __exit hi3798cv200_crg_exit(void)
{
	platform_driver_unregister(&hi3798cv200_crg_driver);
}
module_exit(hi3798cv200_crg_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("HiSilicon Hi3798CV200 CRG Driver");
