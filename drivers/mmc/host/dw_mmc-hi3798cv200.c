// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 HiSilicon Technologies Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "dw_mmc.h"
#include "dw_mmc-pltfm.h"

#define ALL_INT_CLR		0x1ffff

#define HI3798CV200_HS400_STROBE_DELAY	GENMASK(15, 8)
#define HI3798CV200_HS400_STROBE_ENABLE	BIT(16)
#define HI3798CV200_HS400_STROBE_MASK	(HI3798CV200_HS400_STROBE_DELAY | \
					 HI3798CV200_HS400_STROBE_ENABLE)
#define HI3798CV200_HS400_STROBE_MIN	1
#define HI3798CV200_HS400_STROBE_MAX	15
#define HI3798CV200_HS400_STROBE_DEFAULT	8
#define HI3798CV200_HS400_TESTS		5
#define HI3798CV200_HS400_RETRIES	3
#define HI3798CV200_HS400_SETTLE_US	1000
#define HI3798CV200_DEFAULT_CLK		100000000UL
#define HI3798CV200_HS400_CLK		150000000UL
#define HI3798CV200_HS400_CDTHRCTL	0x02000005
#define HI3798CV200_HS200_CDTHRCTL	0x02000001

struct hi3798cv200_priv {
	struct clk *sample_clk;
	struct clk *drive_clk;
	struct regmap *sysctrl;
	u32 io_voltage_reg;
	struct regmap *crg;
	u32 strobe_reg;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_hs200;
	struct pinctrl_state *pins_hs400;
	bool inherit_boot_tuning;
	int boot_sample_phase;
	unsigned int boot_strobe_phase;
};

static int hi3798cv200_mmc_select_pins(struct dw_mci *host,
				       struct pinctrl_state *state,
					const char *name)
{
	struct hi3798cv200_priv *priv = host->priv;
	int ret;

	if (!state)
		return 0;

	ret = pinctrl_select_state(priv->pinctrl, state);
	if (ret)
		dev_err_ratelimited(host->dev,
				    "failed to select %s eMMC pins: %d\n", name,
				    ret);
	return ret;
}

static int hi3798cv200_mmc_set_strobe(struct dw_mci *host, unsigned int phase,
				      bool enable)
{
	struct hi3798cv200_priv *priv = host->priv;
	u32 value, expected;
	int ret;

	if (!priv->crg)
		return -ENODEV;
	if (phase < HI3798CV200_HS400_STROBE_MIN ||
	    phase > HI3798CV200_HS400_STROBE_MAX)
		return -EINVAL;

	expected = FIELD_PREP(HI3798CV200_HS400_STROBE_DELAY, phase);
	if (enable)
		expected |= HI3798CV200_HS400_STROBE_ENABLE;
	ret = regmap_update_bits(priv->crg, priv->strobe_reg,
				 HI3798CV200_HS400_STROBE_MASK, expected);
	if (ret)
		return ret;
	ret = regmap_read(priv->crg, priv->strobe_reg, &value);
	if (ret)
		return ret;
	if ((value & HI3798CV200_HS400_STROBE_MASK) != expected)
		return -EIO;
	if (enable)
		usleep_range(HI3798CV200_HS400_SETTLE_US,
			     HI3798CV200_HS400_SETTLE_US + 500);

	return 0;
}

static bool hi3798cv200_hs400_retryable_error(int ret)
{
	switch (ret) {
	case -EILSEQ:
	case -ETIMEDOUT:
	case -EIO:
	case -EBADMSG:
		return true;
	default:
		return false;
	}
}

/*
 * A single CRC/timeout while changing the data-strobe phase is not evidence
 * that the phase is outside the eye.  Let the card and CIU settle, then retry
 * transient transport failures before rejecting the phase.
 */
static int hi3798cv200_hs400_read_ext_csd(struct dw_mci *host,
						struct mmc_card *card)
{
	u8 *ext_csd;
	int attempt, ret = -EIO;

	for (attempt = 0; attempt < HI3798CV200_HS400_RETRIES; attempt++) {
		ext_csd = NULL;
		mci_writel(host, RINTSTS, ALL_INT_CLR);
		ret = mmc_get_ext_csd(card, &ext_csd);
		kfree(ext_csd);
		mci_writel(host, RINTSTS, ALL_INT_CLR);
		if (!ret)
			return 0;
		if (!hi3798cv200_hs400_retryable_error(ret) ||
		    attempt + 1 == HI3798CV200_HS400_RETRIES)
			break;
		usleep_range(HI3798CV200_HS400_SETTLE_US,
			     HI3798CV200_HS400_SETTLE_US + 500);
	}

	return ret;
}

static int hi3798cv200_mmc_set_phase(struct dw_mci *host, struct clk *clk,
				     const char *name, int degrees)
{
	int ret;

	ret = clk_set_phase(clk, degrees);
	if (ret)
		dev_err_ratelimited(host->dev,
				    "failed to set %s phase to %d degrees: %d\n",
				    name, degrees, ret);

	return ret;
}

static int hi3798cv200_mmc_capture_boot_tuning(struct dw_mci *host)
{
	struct hi3798cv200_priv *priv = host->priv;
	unsigned int strobe_phase;
	u32 value;
	int drive_phase;
	int ret;

	if (!priv->crg)
		return dev_err_probe(host->dev, -EINVAL,
				     "boot tuning requires an HS400 strobe register\n");

	priv->boot_sample_phase = clk_get_phase(priv->sample_clk);
	drive_phase = clk_get_phase(priv->drive_clk);
	ret = regmap_read(priv->crg, priv->strobe_reg, &value);
	if (ret)
		return dev_err_probe(host->dev, ret,
				     "failed to read boot HS400 strobe tuning\n");

	strobe_phase = FIELD_GET(HI3798CV200_HS400_STROBE_DELAY, value);
	if (priv->boot_sample_phase < 0 || priv->boot_sample_phase > 315 ||
	    priv->boot_sample_phase % 45 || drive_phase != 90 ||
	    !(value & HI3798CV200_HS400_STROBE_ENABLE) ||
	    strobe_phase < HI3798CV200_HS400_STROBE_MIN ||
	    strobe_phase > HI3798CV200_HS400_STROBE_MAX) {
		dev_err(host->dev,
			"invalid boot HS400 tuning: sample=%d drive=%d strobe=%u enabled=%u\n",
			priv->boot_sample_phase, drive_phase, strobe_phase,
			!!(value & HI3798CV200_HS400_STROBE_ENABLE));
		return -EINVAL;
	}

	priv->boot_strobe_phase = strobe_phase;
	dev_info(host->dev,
		 "inherited boot HS400 tuning: sample=%d drive=%d strobe=%u\n",
		 priv->boot_sample_phase, drive_phase, priv->boot_strobe_phase);

	return 0;
}

static int hi3798cv200_mmc_ddr52_sample_phase(struct dw_mci *host)
{
	struct hi3798cv200_priv *priv = host->priv;
	unsigned int val;
	int ret;

	if (!priv->sysctrl)
		return 225;

	ret = regmap_read(priv->sysctrl, priv->io_voltage_reg, &val);
	if (ret) {
		dev_err_ratelimited(host->dev,
				    "failed to read eMMC I/O voltage: %d\n", ret);
		return ret;
	}

	return val & BIT(0) ? 225 : 135;
}

static void hi3798cv200_mmc_set_bus_rate(struct dw_mci *host,
					 unsigned long requested)
{
	unsigned long actual;
	int ret;

	ret = clk_set_rate(host->ciu_clk, requested);
	actual = clk_get_rate(host->ciu_clk);
	if (!actual) {
		dev_err_ratelimited(host->dev,
				    "eMMC CIU clock stopped while requesting %lu Hz\n",
			requested);
		return;
	}

	host->bus_hz = actual;
	if (ret || actual != requested)
		dev_err_ratelimited(host->dev,
				    "eMMC CIU clock request %lu Hz failed: ret=%d actual=%lu Hz\n",
			requested, ret, actual);
}

static void dw_mci_hi3798cv200_set_ios(struct dw_mci *host, struct mmc_ios *ios)
{
	struct hi3798cv200_priv *priv = host->priv;
	int sample_phase;
	u32 val;

	/* The CV200 controller must keep CCLK running between commands. */
	if (host->slot)
		set_bit(DW_MMC_CARD_NO_LOW_PWR, &host->slot->flags);

	if (ios->timing == MMC_TIMING_MMC_HS200 ||
	    ios->timing == MMC_TIMING_MMC_HS400)
		hi3798cv200_mmc_set_bus_rate(host, HI3798CV200_HS400_CLK);
	else
		hi3798cv200_mmc_set_bus_rate(host, HI3798CV200_DEFAULT_CLK);

	val = mci_readl(host, UHS_REG);
	if (ios->timing == MMC_TIMING_MMC_DDR52 ||
	    ios->timing == MMC_TIMING_MMC_HS400 ||
	    ios->timing == MMC_TIMING_UHS_DDR50)
		val |= SDMMC_UHS_DDR;
	else
		val &= ~SDMMC_UHS_DDR;
	mci_writel(host, UHS_REG, val);

	val = mci_readl(host, ENABLE_SHIFT);
	if (ios->timing == MMC_TIMING_MMC_DDR52)
		val |= SDMMC_ENABLE_PHASE;
	else
		val &= ~SDMMC_ENABLE_PHASE;
	mci_writel(host, ENABLE_SHIFT, val);

	val = mci_readl(host, DDR_REG);
	if (ios->timing == MMC_TIMING_MMC_HS400)
		val |= SDMMC_DDR_HS400;
	else
		val &= ~SDMMC_DDR_HS400;
	mci_writel(host, DDR_REG, val);

	if (ios->timing == MMC_TIMING_MMC_HS ||
	    ios->timing == MMC_TIMING_LEGACY) {
		hi3798cv200_mmc_select_pins(host, priv->pins_default, "default");
		hi3798cv200_mmc_set_phase(host, priv->sample_clk,
					  "sample", 45);
		hi3798cv200_mmc_set_phase(host, priv->drive_clk,
					  "drive", 180);
	} else if (ios->timing == MMC_TIMING_MMC_DDR52) {
		hi3798cv200_mmc_select_pins(host, priv->pins_default, "default");
		sample_phase = hi3798cv200_mmc_ddr52_sample_phase(host);
		if (sample_phase >= 0)
			hi3798cv200_mmc_set_phase(host, priv->sample_clk,
						  "sample", sample_phase);
		hi3798cv200_mmc_set_phase(host, priv->drive_clk,
					  "drive", 180);
	} else if (ios->timing == MMC_TIMING_MMC_HS200) {
		hi3798cv200_mmc_select_pins(host, priv->pins_hs200, "HS200");
		hi3798cv200_mmc_set_phase(host, priv->drive_clk,
					  "drive", 135);
	} else if (ios->timing == MMC_TIMING_MMC_HS400) {
		hi3798cv200_mmc_select_pins(host, priv->pins_hs400, "HS400");
		if (priv->inherit_boot_tuning)
			hi3798cv200_mmc_set_phase(host, priv->sample_clk,
						  "sample",
						  priv->boot_sample_phase);
		hi3798cv200_mmc_set_phase(host, priv->drive_clk,
					  "drive", 90);
		mci_writel(host, CDTHRCTL, HI3798CV200_HS400_CDTHRCTL);
	}
}

static int dw_mci_hi3798cv200_prepare_hs400_tuning(struct dw_mci *host,
						   struct mmc_ios *ios)
{
	struct hi3798cv200_priv *priv = host->priv;
	unsigned long rate;
	unsigned int strobe_phase;
	unsigned int value;
	int ret;

	if (!priv->crg || !priv->sysctrl)
		return -ENODEV;

	ret = regmap_read(priv->sysctrl, priv->io_voltage_reg, &value);
	if (ret)
		return ret;
	if (!(value & BIT(0))) {
		dev_err(host->dev,
			"refusing HS400 because eMMC I/O is not at 1.8 V\n");
		return -EINVAL;
	}

	ret = clk_set_rate(host->ciu_clk, HI3798CV200_HS400_CLK);
	if (ret)
		return ret;
	rate = clk_get_rate(host->ciu_clk);
	if (rate != HI3798CV200_HS400_CLK)
		return -ERANGE;
	host->bus_hz = rate;

	ret = hi3798cv200_mmc_select_pins(host, priv->pins_hs400, "HS400");
	if (ret)
		return ret;
	ret = hi3798cv200_mmc_set_phase(host, priv->drive_clk, "drive", 90);
	if (ret)
		return ret;
	mci_writel(host, CDTHRCTL, HI3798CV200_HS200_CDTHRCTL);
	strobe_phase = priv->inherit_boot_tuning ? priv->boot_strobe_phase :
		HI3798CV200_HS400_STROBE_MIN;
	return hi3798cv200_mmc_set_strobe(host, strobe_phase, false);
}

static int dw_mci_hi3798cv200_execute_hs400_tuning(struct dw_mci *host,
						   struct mmc_card *card)
{
	struct hi3798cv200_priv *priv = host->priv;
	u16 valid = 0;
	unsigned int phase, start = 0, best_len = 0, best_start = 0;
	int ret = 0, i, len;

	if (!priv->crg)
		return -ENODEV;
	if (priv->inherit_boot_tuning) {
		ret = hi3798cv200_mmc_set_phase(host, priv->sample_clk,
						"sample",
						priv->boot_sample_phase);
		if (!ret)
			ret = hi3798cv200_mmc_set_phase(host, priv->drive_clk,
							"drive", 90);
		if (!ret)
			ret = hi3798cv200_mmc_set_strobe(host,
							 priv->boot_strobe_phase, true);
		if (!ret)
			ret = hi3798cv200_hs400_read_ext_csd(host, card);
		if (!ret) {
			dev_info(host->dev,
				 "restored and validated boot HS400 tuning: sample=%d strobe=%u\n",
				 priv->boot_sample_phase, priv->boot_strobe_phase);
			return 0;
		}

		/* A stale hand-off is recoverable: scan the active board instead. */
		dev_warn(host->dev,
			 "inherited HS400 tuning failed validation (%d); actively retuning\n",
			 ret);
		priv->inherit_boot_tuning = false;
		hi3798cv200_mmc_set_strobe(host, HI3798CV200_HS400_STROBE_DEFAULT,
					   false);
		mci_writel(host, RINTSTS, ALL_INT_CLR);
	}

	for (phase = HI3798CV200_HS400_STROBE_MIN;
	     phase <= HI3798CV200_HS400_STROBE_MAX; phase++) {
		ret = hi3798cv200_mmc_set_strobe(host, phase, true);
		if (ret)
			goto tuning_failed;
		for (i = 0; i < HI3798CV200_HS400_TESTS; i++) {
			ret = hi3798cv200_hs400_read_ext_csd(host, card);
			if (ret)
				break;
		}
		if (!ret)
			valid |= BIT(phase);
	}

	for (start = HI3798CV200_HS400_STROBE_MIN;
	     start <= HI3798CV200_HS400_STROBE_MAX; start++) {
		if (!(valid & BIT(start)))
			continue;
		len = 0;
		while (len < HI3798CV200_HS400_STROBE_MAX &&
		       (valid & BIT(HI3798CV200_HS400_STROBE_MIN +
					((start - HI3798CV200_HS400_STROBE_MIN + len) %
					 (HI3798CV200_HS400_STROBE_MAX -
					  HI3798CV200_HS400_STROBE_MIN + 1)))))
			len++;
		if (len > best_len) {
			best_len = len;
			best_start = start;
		}
	}

	if (!best_len)
		goto tuning_failed;
	phase = HI3798CV200_HS400_STROBE_MIN +
		((best_start - HI3798CV200_HS400_STROBE_MIN +
		  (best_len - 1) / 2) %
		 (HI3798CV200_HS400_STROBE_MAX - HI3798CV200_HS400_STROBE_MIN + 1));
	ret = hi3798cv200_mmc_set_strobe(host, phase, true);
	if (ret)
		goto tuning_failed;
	ret = hi3798cv200_hs400_read_ext_csd(host, card);
	if (ret)
		goto tuning_failed;
	dev_info(host->dev, "HS400 data-strobe valid=0x%04x selected=%u\n",
		 valid, phase);
	return 0;

tuning_failed:
	/* Leave the controller in HS200-safe state before the core retries. */
	hi3798cv200_mmc_set_strobe(host,
				   priv->inherit_boot_tuning ?
				   priv->boot_strobe_phase :
				   HI3798CV200_HS400_STROBE_DEFAULT, false);
	mci_writel(host, RINTSTS, ALL_INT_CLR);
	if (!ret)
		ret = -EIO;
	dev_err(host->dev,
		"HS400 data-strobe tuning failed (valid=0x%04x, err=%d)\n",
		valid, ret);
	return ret;
}

static int dw_mci_hi3798cv200_execute_tuning(struct dw_mci_slot *slot,
					     u32 opcode)
{
	static const int degrees[] = { 0, 45, 90, 135, 180, 225, 270, 315 };
	struct dw_mci *host = slot->host;
	struct hi3798cv200_priv *priv = host->priv;
	int raise_point = -1, fall_point = -1;
	int err, prev_err = -1;
	int found = 0;
	int i;

	if (priv->inherit_boot_tuning) {
		err = hi3798cv200_mmc_set_phase(host, priv->sample_clk,
						"sample",
						priv->boot_sample_phase);
		if (!err) {
			mci_writel(host, RINTSTS, ALL_INT_CLR);
			err = mmc_send_tuning(slot->mmc, opcode, NULL);
		}
		mci_writel(host, RINTSTS, ALL_INT_CLR);
		if (!err) {
			dev_info(host->dev,
				 "validated boot HS400 sample phase at %d degrees\n",
				 priv->boot_sample_phase);
			return 0;
		}

		/* Do not make an incorrect bootloader phase fatal to eMMC probe. */
		dev_warn(host->dev,
			 "boot sample phase %d failed HS200 validation (%d); actively retuning\n",
			 priv->boot_sample_phase, err);
		priv->inherit_boot_tuning = false;
	}

	for (i = 0; i < ARRAY_SIZE(degrees); i++) {
		err = hi3798cv200_mmc_set_phase(host, priv->sample_clk,
						"sample", degrees[i]);
		if (err)
			goto clear_interrupts;
		mci_writel(host, RINTSTS, ALL_INT_CLR);

		err = mmc_send_tuning(slot->mmc, opcode, NULL);
		if (!err)
			found = 1;

		if (i > 0) {
			if (err && !prev_err)
				fall_point = i - 1;
			if (!err && prev_err)
				raise_point = i;
		}

		if (raise_point != -1 && fall_point != -1)
			goto tuning_out;

		prev_err = err;
	}

tuning_out:
	if (found) {
		if (raise_point == -1)
			raise_point = 0;
		if (fall_point == -1)
			fall_point = ARRAY_SIZE(degrees) - 1;
		if (fall_point < raise_point) {
			if ((raise_point + fall_point) >
			    (ARRAY_SIZE(degrees) - 1))
				i = fall_point / 2;
			else
				i = (raise_point + ARRAY_SIZE(degrees) - 1) / 2;
		} else {
			i = (raise_point + fall_point) / 2;
		}

		err = hi3798cv200_mmc_set_phase(host, priv->sample_clk,
						"sample", degrees[i]);
		if (err)
			goto clear_interrupts;
		dev_dbg(host->dev, "Tuning clk_sample[%d, %d], set[%d]\n",
			raise_point, fall_point, degrees[i]);
	} else {
		dev_err(host->dev, "No valid clk_sample shift! use default\n");
		err = -EINVAL;
	}

clear_interrupts:
	mci_writel(host, RINTSTS, ALL_INT_CLR);
	return err;
}

static int dw_mci_hi3798cv200_init(struct dw_mci *host)
{
	struct hi3798cv200_priv *priv;
	u32 args[1];
	int ret;

	priv = devm_kzalloc(host->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	host->priv = priv;

	priv->sample_clk = devm_clk_get(host->dev, "ciu-sample");
	if (IS_ERR(priv->sample_clk)) {
		dev_err(host->dev, "failed to get ciu-sample clock\n");
		return PTR_ERR(priv->sample_clk);
	}

	priv->drive_clk = devm_clk_get(host->dev, "ciu-drive");
	if (IS_ERR(priv->drive_clk)) {
		dev_err(host->dev, "failed to get ciu-drive clock\n");
		return PTR_ERR(priv->drive_clk);
	}

	ret = clk_prepare_enable(priv->sample_clk);
	if (ret) {
		dev_err(host->dev, "failed to enable ciu-sample clock\n");
		return ret;
	}

	ret = clk_prepare_enable(priv->drive_clk);
	if (ret) {
		dev_err(host->dev, "failed to enable ciu-drive clock\n");
		goto disable_sample_clk;
	}

	if (of_find_property(host->dev->of_node,
			     "hisilicon,io-voltage-reg", NULL)) {
		priv->sysctrl = syscon_regmap_lookup_by_phandle_args(
			host->dev->of_node, "hisilicon,io-voltage-reg", 1,
			args);
		if (IS_ERR(priv->sysctrl)) {
			ret = dev_err_probe(host->dev, PTR_ERR(priv->sysctrl),
					    "failed to get I/O voltage syscon\n");
			goto disable_drive_clk;
		}
		priv->io_voltage_reg = args[0];
	}

	if (of_find_property(host->dev->of_node,
			     "hisilicon,hs400-strobe-reg", NULL)) {
		priv->crg = syscon_regmap_lookup_by_phandle_args(
			host->dev->of_node, "hisilicon,hs400-strobe-reg", 1,
			args);
		if (IS_ERR(priv->crg)) {
			ret = dev_err_probe(host->dev, PTR_ERR(priv->crg),
					    "failed to get HS400 CRG syscon\n");
			goto disable_drive_clk;
		}
		priv->strobe_reg = args[0];
		priv->pinctrl = devm_pinctrl_get(host->dev);
		if (IS_ERR(priv->pinctrl)) {
			ret = dev_err_probe(host->dev, PTR_ERR(priv->pinctrl),
					    "failed to get eMMC pinctrl\n");
			goto disable_drive_clk;
		}
		priv->pins_default = pinctrl_lookup_state(priv->pinctrl,
							  PINCTRL_STATE_DEFAULT);
		priv->pins_hs200 = pinctrl_lookup_state(priv->pinctrl, "hs200");
		priv->pins_hs400 = pinctrl_lookup_state(priv->pinctrl, "hs400");
		if (IS_ERR(priv->pins_default) || IS_ERR(priv->pins_hs200) ||
		    IS_ERR(priv->pins_hs400)) {
			ret = -ENODEV;
			dev_err(host->dev, "HS400 pinctrl states are required\n");
			goto disable_drive_clk;
		}
	}

	priv->inherit_boot_tuning = of_property_read_bool(host->dev->of_node,
							  "hisilicon,inherit-boot-tuning");
	if (priv->inherit_boot_tuning) {
		ret = hi3798cv200_mmc_capture_boot_tuning(host);
		if (ret) {
			/* The bootloader value is only an optimization, never a requirement. */
			dev_warn(host->dev,
				 "ignoring invalid boot HS400 tuning (%d); Linux will retune\n",
				 ret);
			priv->inherit_boot_tuning = false;
		}
	}

	return 0;

disable_drive_clk:
	clk_disable_unprepare(priv->drive_clk);

disable_sample_clk:
	clk_disable_unprepare(priv->sample_clk);
	return ret;
}

static const struct dw_mci_drv_data hi3798cv200_data = {
	.common_caps = MMC_CAP_CMD23,
	.init = dw_mci_hi3798cv200_init,
	.set_ios = dw_mci_hi3798cv200_set_ios,
	.execute_tuning = dw_mci_hi3798cv200_execute_tuning,
	.prepare_hs400_tuning = dw_mci_hi3798cv200_prepare_hs400_tuning,
	.execute_hs400_tuning = dw_mci_hi3798cv200_execute_hs400_tuning,
	.strict_reset = true,
};

static int dw_mci_hi3798cv200_probe(struct platform_device *pdev)
{
	return dw_mci_pltfm_register(pdev, &hi3798cv200_data);
}

static void dw_mci_hi3798cv200_remove(struct platform_device *pdev)
{
	struct dw_mci *host = platform_get_drvdata(pdev);
	struct hi3798cv200_priv *priv = host->priv;

	clk_disable_unprepare(priv->drive_clk);
	clk_disable_unprepare(priv->sample_clk);

	dw_mci_pltfm_remove(pdev);
}

static const struct of_device_id dw_mci_hi3798cv200_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-dw-mshc", },
	{},
};

MODULE_DEVICE_TABLE(of, dw_mci_hi3798cv200_match);
static struct platform_driver dw_mci_hi3798cv200_driver = {
	.probe = dw_mci_hi3798cv200_probe,
	.remove_new = dw_mci_hi3798cv200_remove,
	.driver = {
		.name = "dwmmc_hi3798cv200",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = dw_mci_hi3798cv200_match,
		.pm = &dw_mci_pltfm_pmops,
	},
};
module_platform_driver(dw_mci_hi3798cv200_driver);

MODULE_DESCRIPTION("HiSilicon Hi3798CV200 Specific DW-MSHC Driver Extension");
MODULE_LICENSE("GPL v2");
