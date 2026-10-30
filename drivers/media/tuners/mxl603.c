// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MaxLinear MXL603 digital TV tuner driver
 *
 * Copyright (C) 2024 Xiaodong Ni <nxiaodong520@gmail.com>
 *
 * Based on nxdong520/avl6381's MXL603 tuner and MxLWare603 API adaptation.
 * Register sequences retain that adaptation's programming values.
 *
 * I2C protocol: write = [reg][val]; read = write [0xFB][reg], then read.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <media/dvb_frontend.h>
#include <media/tuner.h>

#include "mxl603.h"

#define MXL603_REG_PAGE_CHANGE		0x00
#define MXL603_REG_XTAL_CAP_CTRL	0x01
#define MXL603_REG_XTAL_ENABLE_DIV	0x02
#define MXL603_REG_XTAL_CALI_SET	0x03
#define MXL603_REG_IF_FREQ_SEL		0x04
#define MXL603_REG_IF_PATH_GAIN		0x05
#define MXL603_REG_IF_FCW_LOW		0x06
#define MXL603_REG_IF_FCW_HIGH		0x07
#define MXL603_REG_AGC_CONFIG		0x08
#define MXL603_REG_AGC_SET_POINT	0x09
#define MXL603_REG_MAIN_REG_AMP		0x0E
#define MXL603_REG_TUNER_ENABLE		0x0B
#define MXL603_REG_CHAN_TUNE_BW		0x0F
#define MXL603_REG_CHAN_TUNE_LOW	0x10
#define MXL603_REG_CHAN_TUNE_HI		0x11
#define MXL603_REG_START_TUNE		0x12
#define MXL603_REG_CHIP_ID		0x18
#define MXL603_REG_CHIP_VERSION		0x1A
#define MXL603_REG_RFPIN_LOW		0x1D
#define MXL603_REG_RFPIN_HIGH		0x1E
#define MXL603_REG_RF_REF_STATUS	0x2B
#define MXL603_REG_AGC_FLIP		0x5E
#define MXL603_REG_DIG_ANA_IF_CFG_0	0x5A
#define MXL603_REG_DIG_ANA_IF_CFG_1	0x5B
#define MXL603_REG_DIG_ANA_IF_PWR	0x5C
#define MXL603_REG_DIG_ANA_GINJO_LT	0x96
#define MXL603_REG_DFE_CSF_SS_SEL	0xEA
#define MXL603_REG_DFE_REFLUT_BYP	0xEA
#define MXL603_REG_DFE_REFSX_INT_MOD	0xEB
#define MXL603_REG_DFE_DACIF_GAIN	0xDC
#define MXL603_REG_READ_POINTER		0xFB
#define MXL603_REG_AGC_RB		0xB6
#define MXL603_REG_DFE_SEQ_TUNE		0x60
#define MXL603_REG_DFE_SEQ_CDC		0x5F
#define MXL603_REG_AIC_RESET		0xFF

/* signal modes (register semantics) */
#define MXL603_MODE_DVB_C		0
#define MXL603_MODE_ISDBT_ATSC		1
#define MXL603_MODE_DVBT_DTMB		2

/* FCW is 16-bit: floor(64 * RF / 1 MHz) must fit without truncation. */
#define MXL603_MAX_RF_FREQ_HZ		((U16_MAX + 1ULL) * 1000000 / 64 - 1)
/* bandwidth register values */
#define MXL603_BW_CABLE_6MHz		0x00
#define MXL603_BW_CABLE_7MHz		0x01
#define MXL603_BW_CABLE_8MHz		0x02
#define MXL603_BW_TERR_6MHz		0x20
#define MXL603_BW_TERR_7MHz		0x21
#define MXL603_BW_TERR_8MHz		0x22

struct mxl603_reg_val {
	u8 reg;
	u8 mask;
	u8 val;
};

static const struct mxl603_reg_val mxl603_overwrite_defaults_tbl[] = {
	{ 0x14, 0xFF, 0x13 }, { 0x6D, 0xFF, 0x8A }, { 0x6D, 0xFF, 0x0A },
	{ 0xDF, 0xFF, 0x19 }, { 0x45, 0xFF, 0x1B }, { 0xA9, 0xFF, 0x59 },
	{ 0xAA, 0xFF, 0x6A }, { 0xBE, 0xFF, 0x4C }, { 0xCF, 0xFF, 0x25 },
	{ 0xD0, 0xFF, 0x34 }, { 0x77, 0xFF, 0xE7 }, { 0x78, 0xFF, 0xE3 },
	{ 0x6F, 0xFF, 0x51 }, { 0x7B, 0xFF, 0x84 }, { 0x7C, 0xFF, 0x9F },
	{ 0x56, 0xFF, 0x41 }, { 0xCD, 0xFF, 0x64 }, { 0xC3, 0xFF, 0x2C },
	{ 0x9D, 0xFF, 0x61 }, { 0xF7, 0xFF, 0x52 }, { 0x58, 0xFF, 0x81 },
	{ 0x00, 0xFF, 0x01 }, { 0x62, 0xFF, 0x02 }, { 0x00, 0xFF, 0x00 },
};

static const struct mxl603_reg_val mxl603_mode_dvbc[] = {
	{ 0x0C, 0xFF, 0x00 }, { 0x13, 0xFF, 0x04 }, { 0x53, 0xFF, 0x7E },
	{ 0x57, 0xFF, 0x91 }, { 0x5C, 0xFF, 0xB1 }, { 0x62, 0xFF, 0xF2 },
	{ 0x6E, 0xFF, 0x03 }, { 0x6F, 0xFF, 0xD1 }, { 0x87, 0xFF, 0x77 },
	{ 0x88, 0xFF, 0x55 }, { 0x93, 0xFF, 0x33 }, { 0x97, 0xFF, 0x03 },
	{ 0xBA, 0xFF, 0x40 }, { 0x98, 0xFF, 0xAF }, { 0x9B, 0xFF, 0x20 },
	{ 0x9C, 0xFF, 0x1E }, { 0xA0, 0xFF, 0x18 }, { 0xA5, 0xFF, 0x09 },
	{ 0xC2, 0xFF, 0xA9 }, { 0xC5, 0xFF, 0x7C }, { 0xCD, 0xFF, 0x64 },
	{ 0xCE, 0xFF, 0x7C }, { 0xD5, 0xFF, 0x05 }, { 0xD9, 0xFF, 0x00 },
	{ 0xEA, 0xFF, 0x00 }, { 0xDC, 0xFF, 0x1C },
};

static const struct mxl603_reg_val mxl603_mode_dvbt[] = {
	{ 0x0C, 0xFF, 0x00 }, { 0x13, 0xFF, 0x04 }, { 0x53, 0xFF, 0xFE },
	{ 0x57, 0xFF, 0x91 }, { 0x62, 0xFF, 0xC2 }, { 0x6E, 0xFF, 0x01 },
	{ 0x6F, 0xFF, 0x51 }, { 0x87, 0xFF, 0x77 }, { 0x88, 0xFF, 0x55 },
	{ 0x93, 0xFF, 0x22 }, { 0x97, 0xFF, 0x02 }, { 0xBA, 0xFF, 0x30 },
	{ 0x98, 0xFF, 0xAF }, { 0x9B, 0xFF, 0x20 }, { 0x9C, 0xFF, 0x1E },
	{ 0xA0, 0xFF, 0x18 }, { 0xA5, 0xFF, 0x09 }, { 0xC2, 0xFF, 0xA9 },
	{ 0xC5, 0xFF, 0x7C }, { 0xCD, 0xFF, 0x64 }, { 0xCE, 0xFF, 0x7C },
	{ 0xD5, 0xFF, 0x03 }, { 0xD9, 0xFF, 0x04 },
};

struct mxl603_state {
	struct i2c_adapter *i2c;
	u8 addr;
	const struct mxl603_config *config;
	u32 frequency_hz;
	u32 bandwidth_hz;
};

static int mxl603_wr_reg(struct mxl603_state *state, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };
	struct i2c_msg msg = {
		.addr = state->addr,
		.flags = 0,
		.buf = buf,
		.len = 2,
	};
	int ret = i2c_transfer(state->i2c, &msg, 1);

	if (ret == 1)
		return 0;

	dev_warn(&state->i2c->dev, "wr reg 0x%02x failed=%d\n", reg, ret);
	return -EREMOTEIO;
}

static int mxl603_rd_reg(struct mxl603_state *state, u8 reg, u8 *val)
{
	/* Write the register pointer, then read in a separate transfer. */
	u8 ptr[2] = { MXL603_REG_READ_POINTER, reg };
	struct i2c_msg wr = {
		.addr = state->addr,
		.flags = 0,
		.buf = ptr,
		.len = 2,
	};
	struct i2c_msg rd = {
		.addr = state->addr,
		.flags = I2C_M_RD,
		.buf = val,
		.len = 1,
	};
	int ret = i2c_transfer(state->i2c, &wr, 1);

	if (ret == 1)
		ret = i2c_transfer(state->i2c, &rd, 1);

	if (ret == 1)
		return 0;

	dev_warn(&state->i2c->dev, "rd reg 0x%02x failed=%d\n", reg, ret);
	return -EREMOTEIO;
}

static int mxl603_wr_page1(struct mxl603_state *state, u8 reg, u8 val)
{
	int ret, restore_ret;

	ret = mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x01);
	if (!ret)
		ret = mxl603_wr_reg(state, reg, val);
	restore_ret = mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x00);
	return ret ? ret : restore_ret;
}

static int mxl603_program_regs(struct mxl603_state *state,
			       const struct mxl603_reg_val *tab, size_t len)
{
	size_t i;
	int ret = 0;

	for (i = 0; i < len; i++) {
		u8 cur = 0;

		if (tab[i].mask != 0xFF) {
			ret = mxl603_rd_reg(state, tab[i].reg, &cur);
			if (ret)
				return ret;
			cur = (cur & ~tab[i].mask) | tab[i].val;
			ret = mxl603_wr_reg(state, tab[i].reg, cur);
		} else {
			ret = mxl603_wr_reg(state, tab[i].reg, tab[i].val);
		}
		if (ret)
			return ret;
	}
	return 0;
}

static int mxl603_soft_reset(struct mxl603_state *state)
{
	return mxl603_wr_reg(state, MXL603_REG_AIC_RESET, 0x00);
}

static int mxl603_overwrite_defaults(struct mxl603_state *state)
{
	u8 val;
	int ret;

	ret = mxl603_program_regs(state, mxl603_overwrite_defaults_tbl,
				  ARRAY_SIZE(mxl603_overwrite_defaults_tbl));
	if (ret) {
		mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x00);
		return ret;
	}
	ret |= mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x01);
	if (!ret)
		ret = mxl603_rd_reg(state, 0x31, &val);
	if (!ret)
		ret = mxl603_wr_reg(state, 0x31, (val & 0x2F) | 0xD0);
	ret |= mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x00);

	if (state->config->single_supply_3_3v)
		ret |= mxl603_wr_reg(state, MXL603_REG_MAIN_REG_AMP, 0x04);

	return ret;
}

static int mxl603_xtal(struct mxl603_state *state)
{
	const struct mxl603_config *cfg = state->config;
	u8 control;
	int ret;

	control = (u8)((cfg->xtal.xtal_freq_sel << 5) |
		       (cfg->xtal.xtal_cap & 0x1F));
	if (cfg->xtal.clk_out_enable)
		control |= 0x80;
	ret = mxl603_wr_reg(state, MXL603_REG_XTAL_CAP_CTRL, control);

	control = cfg->xtal.clk_out_div ? 0x01 : 0x00;
	if (cfg->xtal.xtal_sharing_mode)
		control |= 0x40;
	ret |= mxl603_wr_reg(state, MXL603_REG_XTAL_ENABLE_DIV, control);

	/* runqida board-specific */
	ret |= mxl603_wr_reg(state, 0x6D, 0x0A);

	if (cfg->xtal.single_supply_3_3v)
		ret |= mxl603_wr_reg(state, MXL603_REG_MAIN_REG_AMP, 0x14);

	return ret;
}

static int mxl603_if_out(struct mxl603_state *state)
{
	const struct mxl603_config *cfg = state->config;
	u8 val, control;
	int ret;

	ret = mxl603_rd_reg(state, MXL603_REG_IF_FREQ_SEL, &val);
	if (ret)
		return ret;

	if (cfg->if_out.manual_freq_set) {
		u16 if_fcw;

		val |= 0x20;
		ret = mxl603_wr_reg(state, MXL603_REG_IF_FREQ_SEL, val);

		if_fcw = (u16)(cfg->if_out.manual_if_out_freq_khz * 8192 /
			       216000);
		ret |= mxl603_wr_reg(state, MXL603_REG_IF_FCW_LOW,
				     (u8)(if_fcw & 0xFF));
		ret |= mxl603_wr_reg(state, MXL603_REG_IF_FCW_HIGH,
				     (u8)((if_fcw >> 8) & 0x0F));
	} else {
		val &= 0xC0;
		val |= cfg->if_out.if_out_freq;
		ret = mxl603_wr_reg(state, MXL603_REG_IF_FREQ_SEL, val);
	}
	if (ret)
		return ret;

	control = cfg->if_out.if_inversion ? (0x3 << 6) : 0;
	control += cfg->if_out.gain_level & 0x0F;
	control |= 0x20; /* enable IF out */

	return mxl603_wr_reg(state, MXL603_REG_IF_PATH_GAIN, control);
}

static int mxl603_agc(struct mxl603_state *state)
{
	const struct mxl603_config *cfg = state->config;
	u8 val;
	int ret;

	ret = mxl603_rd_reg(state, MXL603_REG_AGC_CONFIG, &val);
	if (ret)
		return ret;
	val = (val & 0xF2) | (cfg->agc.agc_type << 2) | 0x01;
	ret = mxl603_wr_reg(state, MXL603_REG_AGC_CONFIG, val);

	if (!ret)
		ret = mxl603_rd_reg(state, MXL603_REG_AGC_SET_POINT, &val);
	if (ret)
		return ret;
	val = (val & 0x80) | cfg->agc.set_point;
	ret = mxl603_wr_reg(state, MXL603_REG_AGC_SET_POINT, val);

	if (!ret)
		ret = mxl603_rd_reg(state, MXL603_REG_AGC_FLIP, &val);
	if (ret)
		return ret;
	val = (val & 0xEF) | (cfg->agc.agc_polarity_inversion ? 0x10 : 0);
	return mxl603_wr_reg(state, MXL603_REG_AGC_FLIP, val);
}

static int mxl603_power(struct mxl603_state *state, bool active)
{
	int ret;

	if (active) {
		ret = mxl603_wr_reg(state, MXL603_REG_TUNER_ENABLE, 0x01);
		ret |= mxl603_wr_reg(state, MXL603_REG_START_TUNE, 0x01);
	} else {
		ret = mxl603_wr_reg(state, MXL603_REG_START_TUNE, 0x00);
		ret |= mxl603_wr_reg(state, MXL603_REG_TUNER_ENABLE, 0x00);
	}

	/* Page-1 setting retained from the community runqida adaptation. */
	if (!ret)
		ret = mxl603_wr_page1(state, 0x60, 0x37);

	return ret;
}

static int mxl603_set_mode(struct mxl603_state *state, u8 signal_mode)
{
	const struct mxl603_config *cfg = state->config;
	u8 dfe;
	int ret;

	if (signal_mode == MXL603_MODE_DVB_C) {
		ret = mxl603_program_regs(state, mxl603_mode_dvbc,
					  ARRAY_SIZE(mxl603_mode_dvbc));
		ret |= mxl603_wr_reg(state, MXL603_REG_DIG_ANA_IF_CFG_0,
				     0xFE);
		ret |= mxl603_wr_reg(state, MXL603_REG_DIG_ANA_IF_CFG_1,
				     0x10);
	} else {
		ret = mxl603_program_regs(state, mxl603_mode_dvbt,
					  ARRAY_SIZE(mxl603_mode_dvbt));
		ret |= mxl603_wr_reg(state, MXL603_REG_DIG_ANA_IF_CFG_0,
				     0xFE);
		ret |= mxl603_wr_reg(state, MXL603_REG_DIG_ANA_IF_CFG_1,
				     0x18);
		ret |= mxl603_wr_reg(state, MXL603_REG_DIG_ANA_IF_PWR,
				     0xF1);
	}

	dfe = cfg->tuner_mode.xtal_freq_sel ? 0x0E : 0x0D;
	ret |= mxl603_wr_reg(state, MXL603_REG_DFE_CSF_SS_SEL, dfe);

	switch (cfg->tuner_mode.if_out_gain_level) {
	case 0x0B:
		dfe = 0x47;
		break;
	case 0x09:
		dfe = 0x44;
		break;
	case 0x08:
		dfe = 0x43;
		break;
	case 0x07:
		dfe = 0x42;
		break;
	case 0x06:
		dfe = 0x41;
		break;
	case 0x05:
		dfe = 0x40;
		break;
	default:
		dfe = 0x00;
		break;
	}
	ret |= mxl603_wr_reg(state, MXL603_REG_DFE_DACIF_GAIN, dfe);

	/* XTAL calibration */
	if (!ret) {
		ret = mxl603_wr_reg(state, MXL603_REG_XTAL_CALI_SET, 0x00);
		ret |= mxl603_wr_reg(state, MXL603_REG_XTAL_CALI_SET, 0x01);
		msleep(50);
	}

	return ret;
}

static int mxl603_set_freq(struct mxl603_state *state, u32 freq_hz,
			   u8 bw_reg, u8 signal_mode)
{
	u64 frequency = freq_hz;
	u32 fcw;
	u8 lt_reg, agc_data, tune_data, cdc_data;
	int ret;

	if (freq_hz > MXL603_MAX_RF_FREQ_HZ)
		return -EINVAL;

	/* abort any running tune */
	ret = mxl603_wr_reg(state, MXL603_REG_START_TUNE, 0x00);
	if (ret)
		return ret;

	/* VCO band select */
	if (freq_hz < 700000000) {
		ret = mxl603_wr_reg(state, 0x7C, 0x1F);
		if (signal_mode == MXL603_MODE_DVB_C) {
			if (!ret)
				ret = mxl603_wr_page1(state, 0x31, 0xC1);
		} else {
			if (!ret)
				ret = mxl603_wr_page1(state, 0x31, 0x81);
		}
	} else {
		ret = mxl603_wr_reg(state, 0x7C, 0x9F);
		if (signal_mode == MXL603_MODE_DVB_C) {
			if (!ret)
				ret = mxl603_wr_page1(state, 0x31, 0xD1);
		} else {
			if (!ret)
				ret = mxl603_wr_page1(state, 0x31, 0x91);
		}
	}
	if (ret)
		return ret;

	/* Settings retained from the community runqida adaptation. */
	ret = mxl603_wr_reg(state, MXL603_REG_DFE_REFLUT_BYP, 0x00);
	ret |= mxl603_wr_reg(state, MXL603_REG_DFE_REFSX_INT_MOD, 0xD8);
	if (ret)
		return ret;

	ret = mxl603_wr_reg(state, MXL603_REG_CHAN_TUNE_BW, bw_reg);
	if (ret)
		return ret;

	/* RF channel FCW = 64 * RF(Hz) / 1 MHz */
	frequency *= 64;
	do_div(frequency, 1000000);
	fcw = (u32)frequency;

	ret = mxl603_wr_reg(state, MXL603_REG_CHAN_TUNE_LOW,
			    (u8)(fcw & 0xFF));
	ret |= mxl603_wr_reg(state, MXL603_REG_CHAN_TUNE_HI,
			     (u8)((fcw >> 8) & 0xFF));
	if (ret)
		return ret;

	/* power up tuner */
	ret = mxl603_wr_reg(state, MXL603_REG_TUNER_ENABLE, 0x01);
	if (ret)
		return ret;

	/* start sequencer settings (page 1) */
	ret = mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x01);
	if (!ret)
		ret = mxl603_rd_reg(state, MXL603_REG_DIG_ANA_GINJO_LT,
				    &lt_reg);
	ret |= mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x00);
	if (!ret)
		ret = mxl603_rd_reg(state, MXL603_REG_AGC_RB, &agc_data);
	if (!ret)
		ret = mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x01);
	if (!ret)
		ret = mxl603_rd_reg(state, MXL603_REG_DFE_SEQ_TUNE,
				    &tune_data);
	if (!ret)
		ret = mxl603_rd_reg(state, MXL603_REG_DFE_SEQ_CDC, &cdc_data);
	if (ret) {
		mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x00);
		return ret;
	}

	if (lt_reg & 0x10) {
		agc_data  = (agc_data & 0xBF) | 0x0E;
		tune_data = (tune_data & 0xC0) | 0x0E;
		cdc_data  = (cdc_data & 0xC0) | 0x0E;
	} else {
		agc_data  = (agc_data | 0x40) & 0xC0;
		tune_data = (tune_data & 0xC0) | 0x37;
		cdc_data  = (cdc_data & 0xC0) | 0x37;
	}

	ret = mxl603_wr_reg(state, MXL603_REG_DFE_SEQ_TUNE, tune_data);
	ret |= mxl603_wr_reg(state, MXL603_REG_DFE_SEQ_CDC, cdc_data);
	ret |= mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x00);
	if (!ret)
		ret = mxl603_wr_reg(state, MXL603_REG_AGC_RB, agc_data);
	if (ret)
		return ret;

	/* start calibration */
	ret = mxl603_wr_reg(state, MXL603_REG_START_TUNE, 0x01);
	if (ret)
		return ret;
	usleep_range(15000, 16000);

	agc_data |= 0x40;
	ret |= mxl603_wr_reg(state, MXL603_REG_AGC_RB, agc_data);

	return ret;
}

static int mxl603_get_rf_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct mxl603_state *state = fe->tuner_priv;
	u8 lo = 0, hi = 0;
	u16 raw;
	s32 rxpwr;	/* 0.01 dBm */
	int ret, gate_ret;

	*strength = 0;

	ret = fe->ops.i2c_gate_ctrl ? fe->ops.i2c_gate_ctrl(fe, 1) : 0;
	if (!ret)
		ret = mxl603_rd_reg(state, MXL603_REG_RFPIN_LOW, &lo);
	if (!ret)
		ret = mxl603_rd_reg(state, MXL603_REG_RFPIN_HIGH, &hi);

	gate_ret = fe->ops.i2c_gate_ctrl ? fe->ops.i2c_gate_ctrl(fe, 0) : 0;
	if (!ret)
		ret = gate_ret;

	if (ret)
		return ret;

	raw = (u16)lo | ((u16)(hi & 0x03) << 8);
	rxpwr = (raw & 0x01FF) * 25;
	if (raw & 0x02)
		rxpwr += 50;
	if (raw & 0x01)
		rxpwr += 25;
	if (raw & 0x0200)
		rxpwr -= 128 * 100;

	/* legacy u16 interface: positive attenuation, two's complement */
	*strength = (u16)(-rxpwr);

	return 0;
}

static int mxl603_set_params(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct mxl603_state *state = fe->tuner_priv;
	u8 bw_reg, signal_mode;
	int ret, gate_ret;

	switch (c->delivery_system) {
	case SYS_ATSC:
		signal_mode = MXL603_MODE_ISDBT_ATSC;
		bw_reg = MXL603_BW_TERR_6MHz;
		break;
	case SYS_DVBC_ANNEX_A:
		signal_mode = MXL603_MODE_DVB_C;
		switch (c->bandwidth_hz) {
		case 6000000:
			bw_reg = MXL603_BW_CABLE_6MHz;
			break;
		case 7000000:
			bw_reg = MXL603_BW_CABLE_7MHz;
			break;
		case 8000000:
			bw_reg = MXL603_BW_CABLE_8MHz;
			break;
		default:
			return -EINVAL;
		}
		break;
	case SYS_DVBT:
	case SYS_DVBT2:
		signal_mode = MXL603_MODE_DVBT_DTMB;
		switch (c->bandwidth_hz) {
		case 6000000:
			bw_reg = MXL603_BW_TERR_6MHz;
			break;
		case 7000000:
			bw_reg = MXL603_BW_TERR_7MHz;
			break;
		case 8000000:
			bw_reg = MXL603_BW_TERR_8MHz;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	ret = fe->ops.i2c_gate_ctrl ? fe->ops.i2c_gate_ctrl(fe, 1) : 0;
	if (!ret)
		ret = mxl603_if_out(state);
	if (!ret)
		ret = mxl603_set_mode(state, signal_mode);
	if (!ret)
		ret = mxl603_set_freq(state, c->frequency, bw_reg,
				      signal_mode);

	gate_ret = fe->ops.i2c_gate_ctrl ? fe->ops.i2c_gate_ctrl(fe, 0) : 0;
	if (!ret)
		ret = gate_ret;

	if (!ret) {
		state->frequency_hz = c->frequency;
		state->bandwidth_hz = c->bandwidth_hz;
	}

	return ret;
}

static int mxl603_init(struct dvb_frontend *fe)
{
	struct mxl603_state *state = fe->tuner_priv;
	u8 lt;
	int ret, cleanup_ret;

	ret = fe->ops.i2c_gate_ctrl ? fe->ops.i2c_gate_ctrl(fe, 1) : 0;
	if (ret)
		goto close_gate;
	ret = mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x00);
	if (!ret)
		ret = mxl603_soft_reset(state);
	if (!ret)
		ret = mxl603_overwrite_defaults(state);
	if (!ret)
		ret = mxl603_xtal(state);
	if (ret)
		goto close_gate;

	/* Disable loop-through before selecting the calibration sequencer. */
	ret = mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x01);
	if (!ret)
		ret = mxl603_rd_reg(state, MXL603_REG_DIG_ANA_GINJO_LT, &lt);
	if (!ret)
		ret = mxl603_wr_reg(state, MXL603_REG_DIG_ANA_GINJO_LT,
				    lt & ~0x10);
	cleanup_ret = mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x00);
	if (!ret)
		ret = cleanup_ret;
	if (!ret)
		ret = mxl603_if_out(state);
	if (!ret)
		ret = mxl603_agc(state);
	if (!ret)
		ret = mxl603_power(state, true);
	if (!ret)
		ret = mxl603_set_mode(state, MXL603_MODE_DVB_C);

close_gate:
	cleanup_ret = fe->ops.i2c_gate_ctrl ? fe->ops.i2c_gate_ctrl(fe, 0) : 0;
	return ret ? ret : cleanup_ret;
}

static int mxl603_sleep(struct dvb_frontend *fe)
{
	struct mxl603_state *state = fe->tuner_priv;
	int ret, gate_ret;

	ret = fe->ops.i2c_gate_ctrl ? fe->ops.i2c_gate_ctrl(fe, 1) : 0;
	if (!ret)
		ret = mxl603_power(state, false);
	gate_ret = fe->ops.i2c_gate_ctrl ? fe->ops.i2c_gate_ctrl(fe, 0) : 0;
	if (!ret)
		ret = gate_ret;

	return ret;
}

static int mxl603_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	*frequency = ((struct mxl603_state *)fe->tuner_priv)->frequency_hz;
	return 0;
}

static int mxl603_get_bandwidth(struct dvb_frontend *fe, u32 *bandwidth)
{
	*bandwidth = ((struct mxl603_state *)fe->tuner_priv)->bandwidth_hz;
	return 0;
}

static void mxl603_release(struct dvb_frontend *fe)
{
	kfree(fe->tuner_priv);
	fe->tuner_priv = NULL;
}

static const struct dvb_tuner_ops mxl603_ops = {
	.info = {
		.name = "MaxLinear MXL603",
		.frequency_min_hz = 1 * MHz,
		.frequency_max_hz = MXL603_MAX_RF_FREQ_HZ,
		.frequency_step_hz  =  25 * kHz,
	},
	.release = mxl603_release,
	.init = mxl603_init,
	.sleep = mxl603_sleep,
	.set_params = mxl603_set_params,
	.get_frequency = mxl603_get_frequency,
	.get_bandwidth = mxl603_get_bandwidth,
	.get_rf_strength = mxl603_get_rf_strength,
};

struct dvb_frontend *mxl603_attach(struct dvb_frontend *fe,
				   struct i2c_adapter *i2c, u8 addr,
				   const struct mxl603_config *config)
{
	struct mxl603_state *state;
	u8 id = 0, ver = 0;
	int ret;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	state->i2c = i2c;
	state->addr = addr;
	state->config = config;

	ret = fe->ops.i2c_gate_ctrl ? fe->ops.i2c_gate_ctrl(fe, 1) : 0;
	if (!ret)
		ret = mxl603_wr_reg(state, MXL603_REG_PAGE_CHANGE, 0x00);
	if (!ret)
		ret = mxl603_soft_reset(state);
	if (!ret)
		ret = mxl603_rd_reg(state, MXL603_REG_CHIP_ID, &id);
	if (!ret)
		ret = mxl603_rd_reg(state, MXL603_REG_CHIP_VERSION, &ver);

	if (fe->ops.i2c_gate_ctrl) {
		int gate_ret = fe->ops.i2c_gate_ctrl(fe, 0);

		if (!ret)
			ret = gate_ret;
	}

	dev_info(&i2c->dev, "MXL603 detected id(%02x) ver(%02x)\n", id, ver);

	if (ret || id == 0 || id == 0xFF) {
		kfree(state);
		return NULL;
	}

	fe->tuner_priv = state;
	memcpy(&fe->ops.tuner_ops, &mxl603_ops, sizeof(fe->ops.tuner_ops));

	return fe;
}
EXPORT_SYMBOL_GPL(mxl603_attach);

MODULE_DESCRIPTION("MaxLinear MXL603 digital TV tuner driver");
MODULE_AUTHOR("Xiaodong Ni <nxiaodong520@gmail.com>");
MODULE_LICENSE("GPL");
