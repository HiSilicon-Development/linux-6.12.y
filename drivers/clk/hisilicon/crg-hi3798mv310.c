// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hi3798MV310 peripheral clock and reset generator.
 *
 * Rate selectors and gate masks: SPC060 drivers/hisilicon/clk/hi3798mv310/
 * clk-hi3798mv310.c; USB gates: clk-hiusb2.c in the same directory.
 * Phase taps: drivers/mmc/host/himciv300/himci_hi3798mv310.c.
 * CPU APLL uses the common HiSTB protocol up to 1.6 GHz.
 * Shared CPU/GPU/Core voltage and the eMMC clock-mode bit are not modified.
 */

#include <dt-bindings/clock/hi3798mv310-clock.h>
#include <linux/bitfield.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/clk-provider.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "clk.h"
#include "clk-histb-cpu.h"
#include "reset.h"

#define MV310_MMC_SEL_MASK	GENMASK(10, 8)
#define MV310_EMMC_CLK_MODE	BIT(19)
#define MV310_ETH_GATE_MASK	GENMASK(2, 0)
#define MV310_ETH_GATE_VALUE	(BIT(0) | BIT(1))

struct mv310_mmc_mux {
	struct clk_hw hw;
	void __iomem *reg;
	const unsigned long *rates;
	const unsigned long *mode1_rates;
};

#define to_mv310_mmc_mux(_hw) container_of(_hw, struct mv310_mmc_mux, hw)

/* Zero entries are reserved encodings, not zero-frequency clock parents. */
static const unsigned long mv310_emmc_rates[] = {
	100000000, 50000000, 25000000, 200000000, 0, 0, 150000000, 0,
};
static const unsigned long mv310_emmc_mode1_rates[] = {
	50000000, 25000000, 12500000, 100000000, 150000000, 168750000,
	75000000, 0,
};
static const unsigned long mv310_sd0_rates[] = {
	100000000, 50000000, 150000000, 25000000, 0, 0, 0, 0,
};
static const unsigned long mv310_sdio_rates[] = {
	100000000, 50000000, 150000000, 200000000, 0, 0, 0, 0,
};

static const unsigned long *mv310_mmc_rates(struct mv310_mmc_mux *mux,
					 u32 reg)
{
	if (mux->mode1_rates && (reg & MV310_EMMC_CLK_MODE))
		return mux->mode1_rates;

	return mux->rates;
}

static unsigned long mv310_mmc_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct mv310_mmc_mux *mux = to_mv310_mmc_mux(hw);
	u32 reg = readl(mux->reg);

	return mv310_mmc_rates(mux, reg)[(reg & MV310_MMC_SEL_MASK) >> 8];
}

static long mv310_mmc_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	struct mv310_mmc_mux *mux = to_mv310_mmc_mux(hw);
	const unsigned long *rates = mv310_mmc_rates(mux, readl(mux->reg));
	unsigned long best = 0, minimum = ULONG_MAX;
	int i;

	for (i = 0; i < 8; i++) {
		if (!rates[i])
			continue;
		if (rates[i] < minimum)
			minimum = rates[i];
		if (rates[i] <= rate && rates[i] > best)
			best = rates[i];
	}

	/* The DW MMC controller divides this source for identification clocks. */
	return best ? best : minimum;
}

static int mv310_mmc_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	struct mv310_mmc_mux *mux = to_mv310_mmc_mux(hw);
	const unsigned long *rates;
	unsigned long flags;
	u32 reg;
	int i, ret = -EINVAL;

	spin_lock_irqsave(&hisi_clk_lock, flags);
	reg = readl(mux->reg);
	rates = mv310_mmc_rates(mux, reg);
	for (i = 0; i < 8; i++) {
		if (!rates[i] || rates[i] != rate)
			continue;
		/* Preserve bit19: the same selector has different rates by mode. */
		reg = (reg & ~MV310_MMC_SEL_MASK) | (i << 8);
		writel(reg, mux->reg);
		ret = 0;
		break;
	}
	spin_unlock_irqrestore(&hisi_clk_lock, flags);

	return ret;
}

static const struct clk_ops mv310_mmc_mux_ops = {
	.recalc_rate = mv310_mmc_recalc_rate,
	.round_rate = mv310_mmc_round_rate,
	.set_rate = mv310_mmc_set_rate,
};

static int mv310_register_mmc_mux(struct device *dev, void __iomem *base,
				const char *name, unsigned int offset,
				const unsigned long *rates,
				const unsigned long *mode1_rates)
{
	struct clk_init_data init = {
		.name = name,
		.ops = &mv310_mmc_mux_ops,
		.flags = CLK_GET_RATE_NOCACHE,
	};
	struct mv310_mmc_mux *mux;

	mux = devm_kzalloc(dev, sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return -ENOMEM;

	mux->reg = base + offset;
	mux->rates = rates;
	mux->mode1_rates = mode1_rates;
	mux->hw.init = &init;

	return PTR_ERR_OR_ZERO(devm_clk_register(dev, &mux->hw));
}

/*
 * MMC gate bits0/1 are independently controlled; source-rate tables above
 * apply to CIU, not BIU. The BSP does not specify BIU/USB/FEPHY parent rates,
 * so those gates deliberately have no invented fixed-rate parent.
 */
static const struct hisi_gate_clock mv310_gate_clks[] = {
	{ HI3798MV310_AIAO_CLK, "mv310_aiao", "mv310_aiao_mux",
	  CLK_SET_RATE_PARENT, 0x118, 0, 0 },
	{ HI3798MV310_MMC0_BIU, "mv310_mmc0_biu", NULL, 0, 0xa0, 0, 0 },
	{ HI3798MV310_MMC0_CIU, "mv310_mmc0_ciu", "mv310_mmc0_mux",
	  CLK_SET_RATE_PARENT | CLK_GET_RATE_NOCACHE, 0xa0, 1, 0 },
	{ HI3798MV310_SD0_BIU, "mv310_sd0_biu", NULL, 0, 0x9c, 0, 0 },
	{ HI3798MV310_SD0_CIU, "mv310_sd0_ciu", "mv310_sd0_mux",
	  CLK_SET_RATE_PARENT, 0x9c, 1, 0 },
	{ HI3798MV310_SDIO_BIU, "mv310_sdio_biu", NULL, 0, 0x28c, 0, 0 },
	{ HI3798MV310_SDIO_CIU, "mv310_sdio_ciu", "mv310_sdio_mux",
	  CLK_SET_RATE_PARENT, 0x28c, 1, 0 },
	{ HI3798MV310_USB2_REF, "mv310_usb2_ref", NULL, 0, 0xbc, 0, 0 },
	{ HI3798MV310_USB2_BUS, "mv310_usb2_bus", NULL, 0, 0xb8, 0, 0 },
	{ HI3798MV310_USB2_OHCI48, "mv310_usb2_ohci48", NULL, 0, 0xb8, 1, 0 },
	{ HI3798MV310_USB2_OHCI12, "mv310_usb2_ohci12", NULL, 0, 0xb8, 2, 0 },
	{ HI3798MV310_USB2_OTG_UTMI, "mv310_usb2_otg_utmi", NULL, 0, 0xb8, 3, 0 },
	{ HI3798MV310_USB2_HOST_PHY, "mv310_usb2_host_phy", NULL, 0, 0xb8, 4, 0 },
	{ HI3798MV310_USB2_UTMI0, "mv310_usb2_utmi0", NULL, 0, 0xb8, 5, 0 },
	{ HI3798MV310_USB2_UTMI1, "mv310_usb2_utmi1", NULL, 0, 0xb8, 6, 0 },
	/* FEPHY: BSP hieth/phy.c and mach-hi3798mv310/include/mach/platform.h. */
	{ HI3798MV310_ETH_PHY, "mv310_eth_phy", NULL, 0, 0x388, 0, 0 },
	/* Third PHY/UTMI: actual EC6110-T 760 firmware USB init at 0xc57b80. */
	{ HI3798MV310_USB2_REF2, "mv310_usb2_ref2", NULL, 0, 0xbc, 2, 0 },
	{ HI3798MV310_USB2_UTMI2, "mv310_usb2_utmi2", NULL, 0, 0xb8, 7, 0 },
	/* MV310 CRG54/67/68/211; reserved CV200 video gates do not exist here. */
	{ HI3798MV310_VO_BUS, "mv310_vo_bus", NULL, 0, 0xd8, 0, 0 },
	{ HI3798MV310_VO_HD, "mv310_vo_hd", "mv310_pixel",
	  CLK_SET_RATE_PARENT, 0xd8, 4, 0 },
	{ HI3798MV310_VDP_CFG, "mv310_vdp_cfg", NULL, 0, 0xd8, 30, 0 },
	{ HI3798MV310_VDP_CORE, "mv310_vdp_core", NULL, 0, 0x34c, 14, 0 },
	{ HI3798MV310_HDMI_SSC, "mv310_hdmi_ssc", NULL, 0, 0x10c, 0, 0 },
	{ HI3798MV310_HDMI_BYPASS, "mv310_hdmi_bypass", NULL, 0, 0x10c, 1, 0 },
	{ HI3798MV310_HDMI_OSC, "mv310_hdmi_osc", NULL, 0, 0x10c, 2, 0 },
	{ HI3798MV310_HDMI_OS, "mv310_hdmi_os", NULL, 0, 0x10c, 4, 0 },
	{ HI3798MV310_HDMI_AS, "mv310_hdmi_as", NULL, 0, 0x10c, 5, 0 },
	{ HI3798MV310_HDMI_TMDS, "mv310_hdmi_tmds", NULL, 0, 0x110, 0, 0 },
	/* MV310 CRG31/35/203/205; no standalone JPGE gate on this SoC. */
	{ HI3798MV310_JPGD_CORE, "mv310_jpgd_core", NULL, 0, 0x7c, 0, 0 },
	{ HI3798MV310_JPGD_SMMU, "mv310_jpgd_smmu", NULL, 0, 0x334, 0, 0 },
	{ HI3798MV310_VENC_CORE, "mv310_venc_core", "mv310_venc_mux",
	  CLK_SET_RATE_PARENT, 0x8c, 0, 0 },
	{ HI3798MV310_VENC_AXI, "mv310_venc_axi", NULL, 0, 0x8c, 1, 0 },
	{ HI3798MV310_VENC_SMMU, "mv310_venc_smmu", NULL, 0, 0x32c, 0, 0 },
	/* MV310 PNG decoder: CRG33 and CRG210. */
	{ HI3798MV310_PNG_CORE, "mv310_png_core", NULL, 0, 0x84, 0, 0 },
	{ HI3798MV310_PNG_SMMU, "mv310_png_smmu", NULL, 0, 0x348, 0, 0 },
	{ HI3798MV310_VPSS_CORE, "mv310_vpss_core", NULL, 0, 0xf0, 0, 0 },
};

/* Normal SDR tap angles. DDR/HS400 half-cycle phase interpretation is not used. */
static u32 mv310_phase_degrees[] = { 0, 45, 90, 135, 180, 225, 270, 315 };
static u32 mv310_phase_values[] = { 0, 1, 2, 3, 4, 5, 6, 7 };

static const struct hisi_phase_clock mv310_phase_clks[] = {
	{ HI3798MV310_MMC0_SAMPLE, "mv310_mmc0_sample", "mv310_mmc0_ciu",
	  CLK_SET_RATE_PARENT, 0xa0, 12, 3, mv310_phase_degrees,
	  mv310_phase_values, ARRAY_SIZE(mv310_phase_values) },
	{ HI3798MV310_MMC0_DRIVE, "mv310_mmc0_drive", "mv310_mmc0_ciu",
	  CLK_SET_RATE_PARENT, 0xa0, 16, 3, mv310_phase_degrees,
	  mv310_phase_values, ARRAY_SIZE(mv310_phase_values) },
	{ HI3798MV310_SD0_SAMPLE, "mv310_sd0_sample", "mv310_sd0_ciu",
	  CLK_SET_RATE_PARENT, 0x9c, 12, 3, mv310_phase_degrees,
	  mv310_phase_values, ARRAY_SIZE(mv310_phase_values) },
	{ HI3798MV310_SD0_DRIVE, "mv310_sd0_drive", "mv310_sd0_ciu",
	  CLK_SET_RATE_PARENT, 0x9c, 16, 3, mv310_phase_degrees,
	  mv310_phase_values, ARRAY_SIZE(mv310_phase_values) },
	{ HI3798MV310_SDIO_SAMPLE, "mv310_sdio_sample", "mv310_sdio_ciu",
	  CLK_SET_RATE_PARENT, 0x28c, 12, 3, mv310_phase_degrees,
	  mv310_phase_values, ARRAY_SIZE(mv310_phase_values) },
	{ HI3798MV310_SDIO_DRIVE, "mv310_sdio_drive", "mv310_sdio_ciu",
	  CLK_SET_RATE_PARENT, 0x28c, 16, 3, mv310_phase_degrees,
	  mv310_phase_values, ARRAY_SIZE(mv310_phase_values) },
};

struct mv310_eth_gate {
	struct clk_hw hw;
	void __iomem *reg;
};

#define to_mv310_eth_gate(_hw) container_of(_hw, struct mv310_eth_gate, hw)

static void mv310_eth_gate_update(struct clk_hw *hw, bool enable)
{
	struct mv310_eth_gate *gate = to_mv310_eth_gate(hw);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&hisi_clk_lock, flags);
	reg = readl(gate->reg) & ~MV310_ETH_GATE_MASK;
	if (enable)
		reg |= MV310_ETH_GATE_VALUE;
	writel(reg, gate->reg);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
}

static int mv310_eth_gate_enable(struct clk_hw *hw)
{
	mv310_eth_gate_update(hw, true);
	return 0;
}

static void mv310_eth_gate_disable(struct clk_hw *hw)
{
	mv310_eth_gate_update(hw, false);
}

static int mv310_eth_gate_is_enabled(struct clk_hw *hw)
{
	struct mv310_eth_gate *gate = to_mv310_eth_gate(hw);

	return (readl(gate->reg) & MV310_ETH_GATE_MASK) == MV310_ETH_GATE_VALUE;
}

static const struct clk_ops mv310_eth_gate_ops = {
	.enable = mv310_eth_gate_enable,
	.disable = mv310_eth_gate_disable,
	.is_enabled = mv310_eth_gate_is_enabled,
};

/* The OPP core sequences the shared supply around these mux transitions. */
struct mv310_gpu_clk {
	struct clk_hw hw;
	void __iomem *base;
	bool rate_fault;
};

/* CRG selector order; selector 2 is reserved on MV310. */
static const unsigned long mv310_gpu_rates[] = {
	540000000, 400000000, 0, 600000000,
	300000000, 675000000, 500000000, 200000000,
};

static int mv310_gpu_enable(struct clk_hw *hw)
{
	struct mv310_gpu_clk *gpu = container_of(hw, struct mv310_gpu_clk, hw);
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&hisi_clk_lock, flags);
	if (READ_ONCE(gpu->rate_fault)) {
		spin_unlock_irqrestore(&hisi_clk_lock, flags);
		return -EIO;
	}
	value = readl(gpu->base + 0xd4) | BIT(0) | BIT(8) | BIT(9);
	writel(value, gpu->base + 0xd4);
	readl(gpu->base + 0xd4);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
	return 0;
}

static void mv310_gpu_disable(struct clk_hw *hw)
{
	struct mv310_gpu_clk *gpu = container_of(hw, struct mv310_gpu_clk, hw);
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&hisi_clk_lock, flags);
	value = readl(gpu->base + 0xd4) & ~(BIT(0) | BIT(8) | BIT(9));
	writel(value, gpu->base + 0xd4);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
}

static unsigned long mv310_gpu_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct mv310_gpu_clk *gpu = container_of(hw, struct mv310_gpu_clk, hw);
	return mv310_gpu_rates[readl(gpu->base + 0x124) & 7];
}

static long mv310_gpu_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	unsigned long best = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(mv310_gpu_rates); i++)
		if (mv310_gpu_rates[i] <= rate && mv310_gpu_rates[i] > best)
			best = mv310_gpu_rates[i];
	return best ?: 200000000;
}

static int mv310_gpu_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	struct mv310_gpu_clk *gpu = container_of(hw, struct mv310_gpu_clk, hw);
	unsigned long flags;
	u32 value, gates;
	int i;

	for (i = 0; i < ARRAY_SIZE(mv310_gpu_rates); i++)
		if (mv310_gpu_rates[i] && rate == mv310_gpu_rates[i])
			break;
	if (i == ARRAY_SIZE(mv310_gpu_rates))
		return -EINVAL;
	spin_lock_irqsave(&hisi_clk_lock, flags);
	value = (readl(gpu->base + 0x124) & ~7U) | i;
	writel(value, gpu->base + 0x124);
	value = readl(gpu->base + 0x124) & 7;
	if (value != i) {
		/* CCF ignores set_rate errors: stop before OPP can lower voltage. */
		gates = readl(gpu->base + 0xd4) & ~(BIT(0) | BIT(8) | BIT(9));
		writel(gates, gpu->base + 0xd4);
		readl(gpu->base + 0xd4);
		WRITE_ONCE(gpu->rate_fault, true);
	} else {
		WRITE_ONCE(gpu->rate_fault, false);
	}
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
	if (value != i)
		pr_err_ratelimited("hi3798mv310-crg: GPU selector %d did not latch; clock gated\n", i);
	return value == i ? 0 : -EIO;
}

static const struct clk_ops mv310_gpu_ops = {
	.enable = mv310_gpu_enable,
	.disable = mv310_gpu_disable,
	.recalc_rate = mv310_gpu_recalc_rate,
	.round_rate = mv310_gpu_round_rate,
	.set_rate = mv310_gpu_set_rate,
};

/* SPC060 98mv310: HPLL / CRG211 top divider / VO divider / fixed two. */
#define MV310_HPLL_FRAC		GENMASK(23, 0)
#define MV310_HPLL_POST1		GENMASK(26, 24)
#define MV310_HPLL_POST2		GENMASK(30, 28)
#define MV310_HPLL_FB		GENMASK(11, 0)
#define MV310_HPLL_REF		GENMASK(17, 12)
#define MV310_HPLL_DSMPD		BIT(25)
#define MV310_HPLL_POWER		GENMASK(24, 20)
#define MV310_HPLL_BYPASS		BIT(26)
#define MV310_HPLL_LOCK		BIT(4)
#define MV310_PIXEL_GATE		BIT(16)
#define MV310_VO_DIV		GENMASK(19, 18)
#define MV310_VO_ROUTE		(MV310_VO_DIV | BIT(16) | BIT(27) | BIT(28))
#define MV310_VDP_DIV		GENMASK(28, 24)
#define MV310_VDP_ROUTE		(MV310_VDP_DIV | BIT(20) | BIT(22) | BIT(29))
#define MV310_SSC_ROUTE		(GENMASK(13, 10) | BIT(19))
#define MV310_TMDS_DIV		GENMASK(8, 6)

struct mv310_pixel_clk {
	struct clk_hw hw;
	void __iomem *base;
	struct mutex lock;
};

struct mv310_pixel_plan {
	u32 ctrl0, ctrl1, vo, vdp, ssc, tmds;
	unsigned long rate;
};

#define to_mv310_pixel(_hw) container_of(_hw, struct mv310_pixel_clk, hw)

static bool mv310_pixel_plan(unsigned long rate, unsigned long parent,
			     struct mv310_pixel_plan *plan)
{
	u64 best_delta = U64_MAX;
	u32 top, vo, product, ref, best_frac = U32_MAX;

	if (parent != 24000000 || rate < 25000000 || rate > 594000000)
		return false;
	/* HPLL/2 feeds PHY SSC: pixel * 16/8/4/2 below 50/100/200/340 MHz. */
	top = rate <= 50000000 ? 8 : rate <= 100000000 ? 4 :
	      rate <= 340000000 ? 2 : 1;
	vo = rate > 200000000 ? 1 : 2;
	plan->vo = BIT(16) | FIELD_PREP(MV310_VO_DIV, vo == 1 ? 3 : 0);
	plan->vdp = BIT(20) | FIELD_PREP(MV310_VDP_DIV, top - 1);
	plan->ssc = FIELD_PREP(GENMASK(13, 10), rate > 340000000 ? 4 : 9);
	plan->tmds = FIELD_PREP(MV310_TMDS_DIV, rate <= 50000000 ? 7 :
			       rate <= 100000000 ? 3 : rate <= 200000000 ? 1 : 0);
	for (product = 1; product <= 2; product++) {
		u64 vco = (u64)rate * top * vo * 2 * product;

		if (vco <= 800000000 || vco >= 2400000000ULL)
			continue;
		for (ref = 1; ref <= 63; ref++) {
			u64 scaled, actual, denominator, delta;
			u32 fb, frac;

			scaled = DIV_ROUND_CLOSEST_ULL((vco * ref) << 24, parent);
			fb = scaled >> 24;
			frac = scaled & MV310_HPLL_FRAC;
			if (fb <= 16 || fb >= 2400)
				continue;
			denominator = (u64)ref * product * top * vo * 2 << 24;
			actual = DIV64_U64_ROUND_CLOSEST((u64)parent * scaled,
							denominator);
			delta = actual > rate ? actual - rate : rate - actual;
			if (delta > best_delta ||
			    (delta == best_delta && (frac || !best_frac)))
				continue;
			best_delta = delta;
			best_frac = frac;
			plan->rate = actual;
			plan->ctrl0 = frac | FIELD_PREP(MV310_HPLL_POST1, product) |
				       FIELD_PREP(MV310_HPLL_POST2, 1);
			plan->ctrl1 = FIELD_PREP(MV310_HPLL_FB, fb) |
				       FIELD_PREP(MV310_HPLL_REF, ref) |
				       (frac ? 0 : MV310_HPLL_DSMPD);
			if (!delta && !frac)
				return true;
		}
	}
	return best_delta != U64_MAX;
}

static unsigned long mv310_pixel_recalc(struct clk_hw *hw, unsigned long parent)
{
	struct mv310_pixel_clk *pixel = to_mv310_pixel(hw);
	u32 ctrl0 = readl(pixel->base + 0x28);
	u32 ctrl1 = readl(pixel->base + 0x2c);
	u32 vo = readl(pixel->base + 0xd8);
	u32 vdp = readl(pixel->base + 0x34c);
	u32 post1 = FIELD_GET(MV310_HPLL_POST1, ctrl0);
	u32 post2 = FIELD_GET(MV310_HPLL_POST2, ctrl0);
	u32 ref = FIELD_GET(MV310_HPLL_REF, ctrl1);
	u32 divisor;
	u64 scaled, denominator;

	if (!post1 || !post2 || !ref ||
	    !(readl(pixel->base + 0x150) & MV310_HPLL_LOCK) ||
	    (ctrl1 & (MV310_HPLL_POWER | MV310_HPLL_BYPASS)) ||
	    !(vo & BIT(16)) || (vo & BIT(28)) || (vdp & BIT(29)))
		return 0;
	divisor = FIELD_GET(MV310_VO_DIV, vo);
	divisor = divisor == 0 ? 2 : divisor == 1 ? 4 : 1;
	scaled = (u64)FIELD_GET(MV310_HPLL_FB, ctrl1) << 24;
	if (!(ctrl1 & MV310_HPLL_DSMPD))
		scaled |= FIELD_GET(MV310_HPLL_FRAC, ctrl0);
	denominator = (u64)ref * post1 * post2 * divisor * 2 *
		      (FIELD_GET(MV310_VDP_DIV, vdp) + 1) << 24;
	return DIV64_U64_ROUND_CLOSEST((u64)parent * scaled, denominator);
}

static long mv310_pixel_round(struct clk_hw *hw, unsigned long rate,
			      unsigned long *parent)
{
	struct mv310_pixel_plan plan;

	return mv310_pixel_plan(rate, *parent, &plan) ? plan.rate : -EINVAL;
}

static void mv310_pixel_update(struct mv310_pixel_clk *pixel, u32 offset,
			       u32 mask, u32 value)
{
	writel((readl(pixel->base + offset) & ~mask) | (value & mask),
	       pixel->base + offset);
}

static int mv310_pixel_set(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent)
{
	struct mv310_pixel_clk *pixel = to_mv310_pixel(hw);
	struct mv310_pixel_plan plan;
	const u32 mask0 = MV310_HPLL_FRAC | MV310_HPLL_POST1 | MV310_HPLL_POST2;
	const u32 mask1 = MV310_HPLL_FB | MV310_HPLL_REF | MV310_HPLL_DSMPD |
			  MV310_HPLL_POWER | MV310_HPLL_BYPASS;
	u32 old0, old1, oldvo, oldvdp, oldssc, oldtmds, oldgate, status;
	bool old_locked;
	unsigned long flags;
	int ret;

	if (!mv310_pixel_plan(rate, parent, &plan))
		return -EINVAL;
	mutex_lock(&pixel->lock);
	spin_lock_irqsave(&hisi_clk_lock, flags);
	old0 = readl(pixel->base + 0x28);
	old1 = readl(pixel->base + 0x2c);
	oldvo = readl(pixel->base + 0xd8);
	oldvdp = readl(pixel->base + 0x34c);
	oldssc = readl(pixel->base + 0x10c);
	oldtmds = readl(pixel->base + 0x110);
	oldgate = oldssc & MV310_PIXEL_GATE;
	old_locked = !!(readl(pixel->base + 0x150) & MV310_HPLL_LOCK);
	mv310_pixel_update(pixel, 0x10c, MV310_PIXEL_GATE, 0);
	mv310_pixel_update(pixel, 0x28, mask0, plan.ctrl0);
	mv310_pixel_update(pixel, 0x2c, mask1, plan.ctrl1);
	mv310_pixel_update(pixel, 0xd8, MV310_VO_ROUTE, plan.vo);
	mv310_pixel_update(pixel, 0x34c, MV310_VDP_ROUTE, plan.vdp);
	mv310_pixel_update(pixel, 0x10c, MV310_SSC_ROUTE, plan.ssc);
	mv310_pixel_update(pixel, 0x110, MV310_TMDS_DIV, plan.tmds);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
	ret = readl_poll_timeout(pixel->base + 0x150, status,
				 status & MV310_HPLL_LOCK, 1, 1000);
	spin_lock_irqsave(&hisi_clk_lock, flags);
	if (ret) {
		mv310_pixel_update(pixel, 0x28, mask0, old0);
		mv310_pixel_update(pixel, 0x2c, mask1, old1);
		mv310_pixel_update(pixel, 0xd8, MV310_VO_ROUTE, oldvo);
		mv310_pixel_update(pixel, 0x34c, MV310_VDP_ROUTE, oldvdp);
		mv310_pixel_update(pixel, 0x10c, MV310_SSC_ROUTE, oldssc);
		mv310_pixel_update(pixel, 0x110, MV310_TMDS_DIV, oldtmds);
	}
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
	if (ret && old_locked) {
		/* Do not resume an existing stream until the old PLL relocks. */
		if (readl_poll_timeout(pixel->base + 0x150, status,
				       status & MV310_HPLL_LOCK, 1, 1000))
			oldgate = 0;
	}
	if (ret && !old_locked)
		oldgate = 0;
	spin_lock_irqsave(&hisi_clk_lock, flags);
	mv310_pixel_update(pixel, 0x10c, MV310_PIXEL_GATE, oldgate);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
	mutex_unlock(&pixel->lock);
	return ret;
}

static int mv310_pixel_enable(struct clk_hw *hw)
{
	struct mv310_pixel_clk *pixel = to_mv310_pixel(hw);
	unsigned long flags;

	spin_lock_irqsave(&hisi_clk_lock, flags);
	mv310_pixel_update(pixel, 0x10c, MV310_PIXEL_GATE, MV310_PIXEL_GATE);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
	return 0;
}

static void mv310_pixel_disable(struct clk_hw *hw)
{
	struct mv310_pixel_clk *pixel = to_mv310_pixel(hw);
	unsigned long flags;

	spin_lock_irqsave(&hisi_clk_lock, flags);
	mv310_pixel_update(pixel, 0x10c, MV310_PIXEL_GATE, 0);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
}

static int mv310_pixel_is_enabled(struct clk_hw *hw)
{
	return !!(readl(to_mv310_pixel(hw)->base + 0x10c) & MV310_PIXEL_GATE);
}

static const struct clk_ops mv310_pixel_ops = {
	.enable = mv310_pixel_enable,
	.disable = mv310_pixel_disable,
	.is_enabled = mv310_pixel_is_enabled,
	.recalc_rate = mv310_pixel_recalc,
	.round_rate = mv310_pixel_round,
	.set_rate = mv310_pixel_set,
};

static int mv310_register_pixel(struct device *dev, struct hisi_clock_data *data)
{
	const struct clk_parent_data parent = { .fw_name = "osc" };
	struct clk_init_data init = {
		.name = "mv310_pixel",
		.ops = &mv310_pixel_ops,
		.flags = CLK_GET_RATE_NOCACHE,
		.parent_data = &parent,
		.num_parents = 1,
	};
	struct mv310_pixel_clk *pixel;
	struct clk *clk;
	unsigned long flags;

	pixel = devm_kzalloc(dev, sizeof(*pixel), GFP_KERNEL);
	if (!pixel)
		return -ENOMEM;
	pixel->base = data->base;
	pixel->hw.init = &init;
	mutex_init(&pixel->lock);
	clk = devm_clk_register(dev, &pixel->hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	data->clk_data.clks[HI3798MV310_HDMI_PIXEL] = clk;
	/* Divider reset precedes the VOU consumer's gates and runtime reset. */
	spin_lock_irqsave(&hisi_clk_lock, flags);
	mv310_pixel_update(pixel, 0x34c, BIT(29), 0);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
	return 0;
}

static int mv310_register_venc_mux(struct device *dev, void __iomem *base)
{
	struct clk_parent_data parents[2] = {
		{ },
		{ .fw_name = "osc" },
	};
	struct clk_hw *fixed, *mux;

	/* MV310 product.c: selector 0 is 240 MHz; selector 1 is 24 MHz. */
	fixed = devm_clk_hw_register_fixed_rate(dev, "mv310_venc_240m", NULL,
					      0, 240000000);
	if (IS_ERR(fixed))
		return PTR_ERR(fixed);
	parents[0].hw = fixed;
	mux = devm_clk_hw_register_mux_parent_data_table(dev, "mv310_venc_mux",
		parents, ARRAY_SIZE(parents), CLK_GET_RATE_NOCACHE,
		base + 0x8c, 8, 1, 0, NULL, &hisi_clk_lock);
	return PTR_ERR_OR_ZERO(mux);
}

static int mv310_register_audio(struct device *dev, struct hisi_clock_data *data)
{
	static const char * const parents[] = { "mv310_aiao_100m", "mv310_aiao_50m" };
	struct clk_hw *hw;
	unsigned long flags;
	u32 reg;

	/* BSP hi3798mv310/hi_reg_crg.h:819-827; hal_aiao_priv.c:720-740.
	 * The skip divider is bypassed, then the source selects 50/100 MHz.
	 */
	hw = devm_clk_hw_register_fixed_rate(dev, parents[0], NULL, 0, 100000000);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	hw = devm_clk_hw_register_fixed_rate(dev, parents[1], NULL, 0, 50000000);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	hw = devm_clk_hw_register_mux(dev, "mv310_aiao_mux", parents,
		ARRAY_SIZE(parents), CLK_SET_RATE_PARENT, data->base + 0x118,
		22, 1, 0, &hisi_clk_lock);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	/* 860.160 MHz is the source encoded by the BSP's AIAO_PLL_860MHZ
	 * coefficients: e.g. 256*48k*2^27/860160000 = 0x1d41d4 (truncated).
	 * EPLL is established by firmware; never retune the shared PLL here.
	 */
	hw = devm_clk_hw_register_fixed_rate(dev, "mv310_aiao_mclk", NULL,
					 0, 860160000);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	data->clk_data.clks[HI3798MV310_AIAO_MCLK] = hw->clk;
	spin_lock_irqsave(&hisi_clk_lock, flags);
	reg = readl(data->base + 0x118);
	reg &= ~(BIT(21) | BIT(17) | GENMASK(16, 12));
	writel(reg, data->base + 0x118);
	writel(reg | BIT(17), data->base + 0x118);
	spin_unlock_irqrestore(&hisi_clk_lock, flags);
	return 0;
}

static void mv310_unregister_gates(void *data)
{
	hisi_clk_unregister_gate(mv310_gate_clks, ARRAY_SIZE(mv310_gate_clks), data);
}

static void mv310_unregister_resets(void *data)
{
	hisi_reset_exit(data);
}

static void mv310_del_provider(void *data)
{
	of_clk_del_provider(data);
}

static void mv310_unregister_cpu(void *clk)
{
	clk_unregister(clk);
}

static int hi3798mv310_crg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hisi_reset_controller *rstc;
	struct hisi_clock_data *data;
	struct mv310_eth_gate *eth;
	struct mv310_gpu_clk *gpu;
	struct clk_init_data gpu_init = {
		.name = "mv310_gpu",
		.ops = &mv310_gpu_ops,
		.flags = CLK_GET_RATE_NOCACHE,
	};
	struct clk_init_data eth_init = {
		.name = "mv310_eth_mac",
		.ops = &mv310_eth_gate_ops,
	};
	struct clk *clk;
	int ret;

	rstc = hisi_reset_init(pdev);
	if (!rstc)
		return dev_err_probe(dev, -ENODEV, "cannot register CRG resets\n");
	ret = devm_add_action_or_reset(dev, mv310_unregister_resets, rstc);
	if (ret)
		return ret;

	data = hisi_clk_alloc(pdev, HI3798MV310_NR_CLKS);
	if (!data)
		return -ENOMEM;

	clk = hisi_histb_cpu_clk_register(dev, data->base, "mv310_cpu", 1600000000UL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "cannot register CPU APLL\n");
	data->clk_data.clks[HI3798MV310_CPU_CLK] = clk;
	ret = devm_add_action_or_reset(dev, mv310_unregister_cpu, clk);
	if (ret)
		return ret;

	ret = mv310_register_mmc_mux(dev, data->base, "mv310_mmc0_mux", 0xa0,
				     mv310_emmc_rates, mv310_emmc_mode1_rates);
	if (ret)
		return ret;
	ret = mv310_register_mmc_mux(dev, data->base, "mv310_sd0_mux", 0x9c,
				     mv310_sd0_rates, NULL);
	if (ret)
		return ret;
	ret = mv310_register_mmc_mux(dev, data->base, "mv310_sdio_mux", 0x28c,
				     mv310_sdio_rates, NULL);
	if (ret)
		return ret;

	ret = mv310_register_pixel(dev, data);
	if (ret)
		return ret;
	ret = mv310_register_venc_mux(dev, data->base);
	if (ret)
		return ret;
	ret = mv310_register_audio(dev, data);
	if (ret)
		return ret;

	ret = hisi_clk_register_gate(mv310_gate_clks,
				     ARRAY_SIZE(mv310_gate_clks), data);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, mv310_unregister_gates, data);
	if (ret)
		return ret;

	eth = devm_kzalloc(dev, sizeof(*eth), GFP_KERNEL);
	if (!eth)
		return -ENOMEM;
	eth->reg = data->base + 0xd0;
	eth->hw.init = &eth_init;
	clk = devm_clk_register(dev, &eth->hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	data->clk_data.clks[HI3798MV310_ETH_MAC] = clk;

	gpu = devm_kzalloc(dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;
	gpu->base = data->base;
	gpu->hw.init = &gpu_init;
	clk = devm_clk_register(dev, &gpu->hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	data->clk_data.clks[HI3798MV310_GPU_CLK] = clk;

	ret = hisi_clk_register_phase(dev, mv310_phase_clks,
				      ARRAY_SIZE(mv310_phase_clks), data);
	if (ret)
		return ret;

	ret = of_clk_add_provider(dev->of_node, of_clk_src_onecell_get,
				  &data->clk_data);
	if (ret)
		return ret;

	/* Devres unwinds provider, phases, gates, muxes and resets in that order. */
	return devm_add_action_or_reset(dev, mv310_del_provider, dev->of_node);
}

static const struct of_device_id hi3798mv310_crg_match[] = {
	{ .compatible = "hisilicon,hi3798mv310-crg" },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3798mv310_crg_match);

static struct platform_driver hi3798mv310_crg_driver = {
	.probe = hi3798mv310_crg_probe,
	.driver = {
		.name = "hi3798mv310-crg",
		.of_match_table = hi3798mv310_crg_match,
	},
};
module_platform_driver(hi3798mv310_crg_driver);

MODULE_DESCRIPTION("HiSilicon Hi3798MV310 peripheral clock and reset driver");
MODULE_LICENSE("GPL");
