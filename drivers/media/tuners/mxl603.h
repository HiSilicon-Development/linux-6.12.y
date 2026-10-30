/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef MXL603_H_
#define MXL603_H_

#include <linux/dvb/frontend.h>

/*
 * MXL603 tuner attach configuration. Clock, IF and AGC values are provided
 * by the bridge driver for its board wiring.
 */
struct mxl603_config {
	bool single_supply_3_3v;

	struct {
		u8 xtal_freq_sel;	/* 0: 16 MHz, 1: 24 MHz */
		u8 xtal_cap;		/* 1 LSB = 1 pF */
		bool clk_out_enable;
		bool clk_out_div;
		bool single_supply_3_3v;
		bool xtal_sharing_mode;
	} xtal;

	struct {
		u8 if_out_freq;		/* IF_FREQ_SEL <4:0> table index */
		bool if_inversion;
		u8 gain_level;		/* <3:0> */
		bool manual_freq_set;
		u16 manual_if_out_freq_khz;
	} if_out;

	struct {
		u8 agc_type;		/* 0: self, 1: external (closed loop) */
		u8 set_point;
		bool agc_polarity_inversion;
	} agc;

	struct {
		u32 if_out_freq_khz;
		u8 xtal_freq_sel;
		u8 if_out_gain_level;
	} tuner_mode;
};

struct dvb_frontend *mxl603_attach(struct dvb_frontend *fe,
				   struct i2c_adapter *i2c, u8 addr,
				   const struct mxl603_config *config);

#endif
