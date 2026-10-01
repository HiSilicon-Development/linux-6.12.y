/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _MXL214_PRIV_H_
#define _MXL214_PRIV_H_

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mutex.h>

#include <media/dvb_frontend.h>

#define MXL214_NUM_FRONTENDS	4
#define MXL214_FIRMWARE		"mxl214/mxl214.fw"
#define MXL214_NVRAM		"mxl214/nvram50.bin"

struct mxl214_state;

struct mxl214_channel {
	struct dvb_frontend frontend;
	struct mxl214_state *state;
	unsigned int id;
	u64 pre_bit_errors;
	u64 pre_bit_count;
	u64 block_errors;
	u64 block_count;
	unsigned long stats_deadline;
	unsigned long lock_deadline;
	bool stats_started;
	bool lock_candidate;
	bool retune_restart_done;
	bool demod_enabled;
	bool locked;
};

struct mxl214_state {
	struct i2c_client *client;
	struct gpio_desc *reset_gpio;
	struct mutex lock;
	struct mxl214_channel channels[MXL214_NUM_FRONTENDS];
	unsigned int device_id;
	unsigned int lock_timeout_ms;
	bool three_wire;
	bool initialized;
};

struct mxl214_xpt_readback {
	u32 input_enable;
	u32 mux_mode;
	u32 output_enable_raw;
	u32 clock_polarity;
	u32 valid_polarity;
	u32 mode_27mhz;
	u32 pid_mux0;
	u32 pid_mux1;
	u32 ts_clock_enable_raw;
	u32 nco_1_4;
	u32 nco_5_7;
	u32 output_enable;
	u32 ts_clock_enable;
	u32 nco_count_min;
	u16 mpeg_output_enable;
	u16 mpeg_clock_enable;
	u16 xpt_ts_mode;
	u16 pad_sync_drive;
	u16 pad_data_drive;
	u16 pad_valid_drive;
	u16 eco4;
	u16 eco8;
	unsigned int physical_output;
};

int mxl214_xpt_readback(u8 device_id, unsigned int logical_output,
			struct mxl214_xpt_readback *readback);

#endif
