// SPDX-License-Identifier: GPL-2.0-only
/*
 * LME2510C DTMB USB receiver driver
 *
 * Copyright (C) 2024 wwek
 * Based on dvb-cn-usb/src/lme2510.c and src/dvb_usb_lme2510.c,
 * derived from gitee.com/xuyizai/dtmb-linux/kernel/lme2510_dtmb.c.
 */

#include <linux/cache.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/usb.h>

#include "dvb_usb.h"

#define LME2510_DTMB_VID 0x3344
#define LME2510_DTMB_COLD_PID 0x1111
#define LME2510_DTMB_PID_1120 0x1120
#define LME2510_DTMB_PID_1122 0x1122
#define LME2510_DTMB_PID_22D0 0x22d0
#define LME2510_DTMB_PID_22D8 0x22d8
#define LME2510_DTMB_PID_E388 0xe388
#define LME2510_DTMB_OEM_VID 0x1b80

#define LME2510_DTMB_FW_ROM "dvb-usb-lme2510c-dtmb-win-rom.bin"
#define LME2510_DTMB_FW_MAIN "dvb-usb-lme2510c-dtmb-win-fw.bin"
#define LME2510_DTMB_FW_22D0 "dvb-usb-lme2510c-dtmb-win-fw-pid0xd0.bin"

#define LME2510_DTMB_EP_COMMAND_OUT 0x01
#define LME2510_DTMB_EP_COMMAND_IN 0x81
#define LME2510_DTMB_EP_TS_IN 0x88
#define LME2510_DTMB_EP_STATUS_IN 0x8a

#define LME2510_DTMB_DEMOD_SELECTOR 0x32
#define LME2510_DTMB_DEMOD_ALT_SELECTOR 0x36
#define LME2510_DTMB_TUNER_SELECTOR 0xc0

#define LME2510_DTMB_REG_RESET 0x02
#define LME2510_DTMB_REG_MODE 0x07
#define LME2510_DTMB_REG_SIGNAL 0x4b
#define LME2510_DTMB_REG_INVERSION 0x7c
#define LME2510_DTMB_REG_DETECTED 0xa2
#define LME2510_DTMB_REG_LOCK 0xa4

#define LME2510_DTMB_TUNER_ROM_ADDRESS 0x0d
#define LME2510_DTMB_TUNER_ROM_DATA 0x10
#define LME2510_DTMB_TUNER_REFERENCE_MHZ 12
#define LME2510_DTMB_TUNER_INIT_REGISTERS 15

#define LME2510_DTMB_COMMAND_BUFFER_SIZE 64
#define LME2510_DTMB_STATUS_BUFFER_SIZE 512
#define LME2510_DTMB_FIRMWARE_PAYLOAD_SIZE 50
#define LME2510_DTMB_TS_BUFFER_SIZE 4096
#define LME2510_DTMB_STREAM_STALE_MS 2000

struct lme2510_dtmb_status {
	bool valid;
	u8 lock;
	u8 quality_low;
	u8 strength;
	u8 quality_high;
	u8 extra;
};

struct lme2510_dtmb_state {
	struct dvb_usb_device *d;
	struct dvb_frontend frontend;
	struct mutex protocol_lock; /* Serializes command/response transfers. */
	struct mutex tune_lock; /* Serializes frontend and stream transactions. */
	u8 command_buffer[LME2510_DTMB_COMMAND_BUFFER_SIZE] __aligned(ARCH_DMA_MINALIGN);
	u8 demod_read_hint __aligned(ARCH_DMA_MINALIGN);

	struct urb *status_urb;
	u8 status_buffer[LME2510_DTMB_STATUS_BUFFER_SIZE] __aligned(ARCH_DMA_MINALIGN);
	atomic_t streaming;
	bool stream_requested;
	bool transport_active;
	bool disconnected;
	spinlock_t stream_lock; /* Protects asynchronous status data. */
	struct lme2510_dtmb_status stream_status;
	unsigned long stream_status_updated;

	u8 tuner_calibration[5];
	u8 tuner_fraction_high;
	u8 tuner_reg_a;
	u8 detected_parameters;
	bool initialized;
	bool tuned;
};

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);



static int lme2510_dtmb_start_stream_locked(struct dvb_frontend *frontend);
static int lme2510_dtmb_stop_stream_locked(struct lme2510_dtmb_state *state);

static int lme2510_dtmb_exchange(struct lme2510_dtmb_state *state,
				 const u8 *request, size_t request_length,
				 u8 *response, size_t response_length,
				 size_t *response_actual)
{
	struct usb_device *udev = state->d->udev;
	int actual = 0;
	int ret;

	if (!request_length || request_length > sizeof(state->command_buffer) ||
	    response_length > sizeof(state->command_buffer))
		return -EINVAL;

	ret = mutex_lock_interruptible(&state->protocol_lock);
	if (ret)
		return ret;

	memcpy(state->command_buffer, request, request_length);
	ret = usb_bulk_msg(udev,
			   usb_sndbulkpipe(udev, LME2510_DTMB_EP_COMMAND_OUT),
			   state->command_buffer, request_length, &actual,
			   1500);
	if (ret)
		goto unlock;
	if (actual != request_length) {
		ret = -EREMOTEIO;
		goto unlock;
	}

	if (!response_length)
		goto unlock;

	/* Drain a complete command packet, including longer bridge replies. */
	memset(state->command_buffer, 0, sizeof(state->command_buffer));
	ret = usb_bulk_msg(udev,
			   usb_rcvbulkpipe(udev, LME2510_DTMB_EP_COMMAND_IN),
			   state->command_buffer, sizeof(state->command_buffer),
			   &actual, 1500);
	if (ret)
		goto unlock;
	if (!actual) {
		ret = -EREMOTEIO;
		goto unlock;
	}

	if (response)
		memcpy(response, state->command_buffer,
		       min_t(size_t, actual, response_length));
	if (response_actual)
		*response_actual = actual;

unlock:
	mutex_unlock(&state->protocol_lock);
	return ret;
}

static bool lme2510_dtmb_valid_ack(u8 value)
{
	return value == 0x55 || value == 0x88;
}

static int lme2510_dtmb_command(struct lme2510_dtmb_state *state,
				const u8 *command, size_t command_length,
				size_t response_length)
{
	u8 response[8] = {};
	size_t actual = 0;
	int ret;

	if (response_length > sizeof(response))
		return -EINVAL;

	ret = lme2510_dtmb_exchange(state, command, command_length, response,
				    response_length, &actual);
	if (ret)
		return ret;
	if (actual < 1 || !lme2510_dtmb_valid_ack(response[0]))
		return -EREMOTEIO;

	if (command_length == 5 && command[0] == 0x05 &&
	    command[2] == LME2510_DTMB_DEMOD_SELECTOR)
		state->demod_read_hint = command[4];

	return 0;
}

static int lme2510_dtmb_read_reg(struct lme2510_dtmb_state *state, u8 command,
				 u8 selector, u8 reg, u8 suffix, u8 *value)
{
	u8 request[] = { command, command == 0x85 ? 0x02 : 0x03, selector, reg,
			 suffix };
	u8 response[2] = {};
	size_t actual = 0;
	int ret;

	ret = lme2510_dtmb_exchange(state, request, sizeof(request), response,
				    sizeof(response), &actual);
	if (ret)
		return ret;
	if (actual != sizeof(response) || !lme2510_dtmb_valid_ack(response[0]))
		return -EREMOTEIO;

	*value = response[1];
	return 0;
}

static int lme2510_dtmb_demod_read(struct lme2510_dtmb_state *state, u8 reg,
				   u8 *value)
{
	return lme2510_dtmb_read_reg(state, 0x85, LME2510_DTMB_DEMOD_SELECTOR,
				     reg, state->demod_read_hint, value);
}

static int lme2510_dtmb_demod_alt_read(struct lme2510_dtmb_state *state, u8 reg,
				       u8 *value)
{
	return lme2510_dtmb_read_reg(state, 0x85,
				     LME2510_DTMB_DEMOD_ALT_SELECTOR, reg,
				     state->demod_read_hint, value);
}

static int lme2510_dtmb_demod_write_selector(struct lme2510_dtmb_state *state,
					     u8 selector, u8 reg, u8 value)
{
	u8 command[] = { 0x05, 0x04, selector, reg, value };

	return lme2510_dtmb_command(state, command, sizeof(command), 4);
}

static int lme2510_dtmb_demod_write(struct lme2510_dtmb_state *state, u8 reg,
				    u8 value)
{
	u8 selector = LME2510_DTMB_DEMOD_SELECTOR;

	return lme2510_dtmb_demod_write_selector(state, selector, reg, value);
}

static int lme2510_dtmb_demod_alt_write(struct lme2510_dtmb_state *state,
					u8 reg, u8 value)
{
	u8 selector = LME2510_DTMB_DEMOD_ALT_SELECTOR;

	return lme2510_dtmb_demod_write_selector(state, selector, reg, value);
}

static int lme2510_dtmb_demod_update_bits(struct lme2510_dtmb_state *state,
					  u8 reg, u8 mask, u8 value)
{
	u8 old;
	int ret;

	ret = lme2510_dtmb_demod_read(state, reg, &old);
	if (ret)
		return ret;
	return lme2510_dtmb_demod_write(state, reg,
					(old & ~mask) | (value & mask));
}

static int lme2510_dtmb_demod_alt_update_bits(struct lme2510_dtmb_state *state,
					      u8 reg, u8 mask, u8 value)
{
	u8 old;
	int ret;

	ret = lme2510_dtmb_demod_alt_read(state, reg, &old);
	if (ret)
		return ret;
	return lme2510_dtmb_demod_alt_write(state, reg,
					    (old & ~mask) | (value & mask));
}

static int lme2510_dtmb_tuner_write_block(struct lme2510_dtmb_state *state,
					  u8 reg, const u8 *values,
					  size_t count)
{
	u8 command[20];
	u8 response[4] = {};
	size_t actual = 0;
	int ret;

	if (!count || count > 16)
		return -EINVAL;

	command[0] = 0x04;
	command[1] = count + 2;
	command[2] = LME2510_DTMB_TUNER_SELECTOR;
	command[3] = reg;
	memcpy(command + 4, values, count);

	ret = lme2510_dtmb_exchange(state, command, count + 4, response,
				    sizeof(response), &actual);
	if (ret)
		return ret;
	if (actual < 1 || !lme2510_dtmb_valid_ack(response[0]))
		return -EREMOTEIO;
	return 0;
}

static int lme2510_dtmb_tuner_write(struct lme2510_dtmb_state *state, u8 reg,
				    u8 value)
{
	return lme2510_dtmb_tuner_write_block(state, reg, &value, 1);
}

static int lme2510_dtmb_tuner_read(struct lme2510_dtmb_state *state, u8 reg,
				   u8 *value)
{
	return lme2510_dtmb_read_reg(state, 0x84, LME2510_DTMB_TUNER_SELECTOR,
				     reg, 0x01, value);
}

struct lme2510_dtmb_boot {
	struct usb_device *udev;
	struct mutex lock; /* Serializes bootloader transfers. */
	u8 *buffer;
};

static int lme2510_dtmb_boot_exchange(struct lme2510_dtmb_boot *boot,
				      const u8 *request, size_t length,
				      bool launch)
{
	unsigned int pipe;
	int actual = 0;
	int ret;

	if (!length || length > LME2510_DTMB_COMMAND_BUFFER_SIZE)
		return -EINVAL;

	mutex_lock(&boot->lock);
	memcpy(boot->buffer, request, length);
	ret = usb_bulk_msg(boot->udev,
			   usb_sndbulkpipe(boot->udev,
					   LME2510_DTMB_EP_COMMAND_OUT),
			   boot->buffer, length, &actual, 500);
	if (ret)
		goto unlock;
	if (actual != length) {
		ret = -EREMOTEIO;
		goto unlock;
	}

	memset(boot->buffer, 0, LME2510_DTMB_COMMAND_BUFFER_SIZE);
	pipe = usb_rcvbulkpipe(boot->udev, LME2510_DTMB_EP_COMMAND_IN);
	ret = usb_bulk_msg(boot->udev, pipe, boot->buffer,
			   LME2510_DTMB_COMMAND_BUFFER_SIZE, &actual, 200);
	/* Bootloader replies are optional; only launch may disconnect USB. */
	if (ret == -ETIMEDOUT ||
	    (launch && (ret == -ENODEV || ret == -ESHUTDOWN)))
		ret = 0;

unlock:
	mutex_unlock(&boot->lock);
	return ret;
}

static u8 lme2510_dtmb_firmware_checksum(const u8 *data, size_t length)
{
	u8 checksum = 0;

	while (length--)
		checksum += *data++;
	return checksum;
}

static int lme2510_dtmb_upload_segment(struct lme2510_dtmb_boot *boot,
				       const struct firmware *firmware,
				       u8 segment)
{
	u8 packet[LME2510_DTMB_FIRMWARE_PAYLOAD_SIZE + 3];
	size_t offset = 0;
	int ret;

	if (!firmware->size || firmware->size > U16_MAX)
		return -EINVAL;

	while (offset < firmware->size) {
		const u8 *chunk = firmware->data + offset;
		size_t payload = min_t(size_t,
				       LME2510_DTMB_FIRMWARE_PAYLOAD_SIZE,
				       firmware->size - offset);
		bool final = offset + payload == firmware->size;

		packet[0] = final ? segment | 0x80 : segment;
		packet[1] = payload - 1;
		memcpy(packet + 2, chunk, payload);
		packet[payload + 2] =
			lme2510_dtmb_firmware_checksum(chunk, payload);

		ret = lme2510_dtmb_boot_exchange(boot, packet, payload + 3,
						 false);
		if (ret)
			return ret;
		offset += payload;
	}

	return 0;
}

static const char *lme2510_dtmb_main_firmware(struct usb_device *udev)
{
	if (le16_to_cpu(udev->descriptor.idProduct) == LME2510_DTMB_PID_22D0)
		return LME2510_DTMB_FW_22D0;
	return LME2510_DTMB_FW_MAIN;
}

static int lme2510_dtmb_download_firmware(struct usb_device *udev)
{
	const struct firmware *rom;
	const struct firmware *main;
	struct lme2510_dtmb_boot boot = { .udev = udev };
	const char *main_name = lme2510_dtmb_main_firmware(udev);
	static const u8 launch[] = { 0x8a, 0x00 };
	int ret;

	ret = request_firmware(&rom, LME2510_DTMB_FW_ROM, &udev->dev);
	if (ret)
		return dev_err_probe(&udev->dev, ret, "failed to load %s\n",
				     LME2510_DTMB_FW_ROM);

	ret = request_firmware(&main, main_name, &udev->dev);
	if (ret) {
		dev_err_probe(&udev->dev, ret, "failed to load %s\n",
			      main_name);
		goto release_rom;
	}

	boot.buffer = kmalloc(LME2510_DTMB_COMMAND_BUFFER_SIZE, GFP_KERNEL);
	if (!boot.buffer) {
		ret = -ENOMEM;
		goto release_main;
	}
	mutex_init(&boot.lock);

	ret = usb_set_interface(udev, 0, 1);
	if (ret)
		goto free_buffer;
	ret = lme2510_dtmb_upload_segment(&boot, rom, 1);
	if (ret)
		goto free_buffer;
	ret = lme2510_dtmb_upload_segment(&boot, main, 2);
	if (ret)
		goto free_buffer;
	ret = lme2510_dtmb_boot_exchange(&boot, launch, sizeof(launch), true);
	if (!ret)
		msleep(300);

free_buffer:
	kfree(boot.buffer);
release_main:
	release_firmware(main);
release_rom:
	release_firmware(rom);
	return ret;
}

static void lme2510_dtmb_tuner_pll(struct lme2510_dtmb_state *state,
				   u32 frequency_khz, u8 registers[5])
{
	u32 ref_khz = LME2510_DTMB_TUNER_REFERENCE_MHZ * 1000;
	u32 fraction =
		(u32)((((u64)(frequency_khz % ref_khz)) << 20) / ref_khz);
	u32 frequency_mhz = frequency_khz / 1000;
	int lpf_low = state->tuner_calibration[1] & 0x0f;
	int lpf_high = state->tuner_calibration[1] >> 4;
	int lpf;
	u8 band;

	state->tuner_fraction_high &= 0xf0;
	state->tuner_fraction_high |= (fraction >> 16) & 0x0f;
	band = frequency_mhz < 725 ? state->tuner_calibration[0] >> 4 :
				     state->tuner_calibration[0] & 0x0f;
	lpf = lpf_low + (lpf_high - lpf_low) * ((int)frequency_mhz - 470) / 310;

	registers[0] = frequency_khz / ref_khz;
	registers[1] = state->tuner_fraction_high;
	registers[2] = fraction >> 8;
	registers[3] = fraction;
	registers[4] = band << 4 | clamp_t(int, lpf, 0, 15);
}

static u8 lme2510_dtmb_tuner_reg_a(struct lme2510_dtmb_state *state)
{
	int high = (state->tuner_calibration[2] >> 4) - 2;

	state->tuner_reg_a &= 0x0f;
	state->tuner_reg_a |= clamp_t(int, high, 0, 15) << 4;
	return state->tuner_reg_a;
}

static int lme2510_dtmb_demod_reset(struct lme2510_dtmb_state *state)
{
	u8 dummy;
	u8 reset;
	int ret;

	ret = lme2510_dtmb_demod_read(state, 0x00, &dummy);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_read(state, LME2510_DTMB_REG_RESET, &reset);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_write(state, LME2510_DTMB_REG_RESET,
				       reset & ~BIT(0));
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_write(state, LME2510_DTMB_REG_RESET,
				       reset | BIT(0));
	if (!ret)
		usleep_range(5000, 6000);
	return ret;
}

static int lme2510_dtmb_tuner_init(struct lme2510_dtmb_state *state)
{
	u8 registers[LME2510_DTMB_TUNER_INIT_REGISTERS] = {};
	u8 rom_address = LME2510_DTMB_TUNER_ROM_ADDRESS;
	bool all_zero = true;
	int ret;
	int i;

	state->tuner_fraction_high = 0x10;
	state->tuner_reg_a = 0xc3;
	for (i = 0; i < ARRAY_SIZE(state->tuner_calibration); i++) {
		ret = lme2510_dtmb_tuner_write(state, rom_address, i + 1);
		if (ret)
			return ret;
		ret = lme2510_dtmb_tuner_read(state,
					      LME2510_DTMB_TUNER_ROM_DATA,
					      &state->tuner_calibration[i]);
		if (ret)
			return ret;
		all_zero &= !state->tuner_calibration[i];
	}

	ret = lme2510_dtmb_tuner_write(state, LME2510_DTMB_TUNER_ROM_ADDRESS,
				       0x00);
	if (ret)
		return ret;
	if (all_zero)
		return -ENODEV;

	lme2510_dtmb_tuner_pll(state, 474 * 1000, registers);
	registers[4] |= 0xf0;
	registers[5] = 0x01;
	registers[6] = 0x0a;
	registers[7] = 0x08;
	registers[8] = 0x02;
	registers[9] = 0x54;
	registers[10] = lme2510_dtmb_tuner_reg_a(state);
	registers[11] = 0x75;

	return lme2510_dtmb_tuner_write_block(state, 0x00, registers,
					      sizeof(registers));
}

static int lme2510_dtmb_frontend_initialize(struct lme2510_dtmb_state *state)
{
	static const u8 bridge_init[] = { 0x16, 0x01, 0x00 };
	u8 dummy;
	int ret;
	u8 reg;

	ret = lme2510_dtmb_demod_read(state, 0x00, &dummy);
	if (ret)
		return ret;
	ret = lme2510_dtmb_command(state, bridge_init, sizeof(bridge_init), 5);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_read(state, 0x00, &dummy);
	if (ret)
		return ret;
	/* Parallel TS, inverted sampling clock, gated clock. */
	ret = lme2510_dtmb_demod_alt_write(state, 0xc2, 0x2a);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_reset(state);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_write(state, 0x01, 0xe0);
	if (ret)
		return ret;
	ret = lme2510_dtmb_tuner_init(state);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_write(state, 0x01, 0x60);
	if (ret)
		return ret;

	ret = lme2510_dtmb_demod_read(state, LME2510_DTMB_REG_MODE, &reg);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_write(state, LME2510_DTMB_REG_MODE,
				       reg | 0x0c);
	if (ret)
		return ret;
	for (reg = 0x09; reg <= 0x0c; reg++) {
		ret = lme2510_dtmb_demod_write(state, reg, 0x00);
		if (ret)
			return ret;
	}
	ret = lme2510_dtmb_demod_read(state, LME2510_DTMB_REG_MODE, &reg);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_write(state, LME2510_DTMB_REG_MODE,
					reg & 0x7c);
	return ret;
}

static int lme2510_dtmb_tuner_set_frequency(struct lme2510_dtmb_state *state,
					    u32 frequency_khz)
{
	u8 registers[5];
	u8 control;
	int ret;

	lme2510_dtmb_tuner_pll(state, frequency_khz, registers);
	ret = lme2510_dtmb_tuner_write_block(state, 0x00, registers,
					     sizeof(registers));
	if (ret)
		return ret;
	ret = lme2510_dtmb_tuner_write(state, 0x0a,
					lme2510_dtmb_tuner_reg_a(state));
	if (ret)
		return ret;
	ret = lme2510_dtmb_tuner_read(state, 0x04, &control);
	if (ret)
		return ret;
	return lme2510_dtmb_tuner_write(state, 0x04, control | 0xf0);
}


static int lme2510_dtmb_post_tuner_setup(struct lme2510_dtmb_state *state)
{
	int ret;

	ret = lme2510_dtmb_demod_update_bits(state, 0x03, BIT(0), 0);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_update_bits(state, 0x7e, BIT(0), BIT(0));
	if (ret)
		return ret;
	return lme2510_dtmb_demod_alt_update_bits(state, 0xc5, 0x1f, 0);
}

static int lme2510_dtmb_capture_mode(struct lme2510_dtmb_state *state, u8 mode)
{
	u8 reg4;
	u8 reg37;
	int ret;

	ret = lme2510_dtmb_demod_read(state, 0x04, &reg4);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_read(state, 0x37, &reg37);
	if (ret)
		return ret;
	reg4 &= 0xfc;

	switch (mode) {
	case 0:
		reg4 |= 0x02;
		reg37 &= ~BIT(7);
		break;
	case 2:
		reg37 &= ~BIT(7);
		break;
	case 4:
		reg4 |= 0x01;
		reg37 |= BIT(7);
		break;
	default:
		return -EINVAL;
	}

	ret = lme2510_dtmb_demod_write(state, 0x04, reg4);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_write(state, 0x37, reg37);
	if (ret)
		return ret;
	return lme2510_dtmb_demod_reset(state);
}

static int lme2510_dtmb_wait_mask(struct lme2510_dtmb_state *state, u8 reg,
				  u8 mask, u8 expected, unsigned int attempts,
				  unsigned int delay_ms, u8 *last,
				  bool *transport_error)
{
	u8 value = 0;
	int ret;

	if (transport_error)
		*transport_error = false;
	while (attempts--) {
		msleep(delay_ms);
		ret = lme2510_dtmb_demod_read(state, reg, &value);
		if (ret) {
			if (transport_error)
				*transport_error = true;
			return ret;
		}
		if ((value & mask) == expected) {
			if (last)
				*last = value;
			return 0;
		}
	}
	if (last)
		*last = value;
	return -ETIMEDOUT;
}

static int lme2510_dtmb_finalize_lock(struct lme2510_dtmb_state *state)
{
	u8 detected;
	int ret;

	ret = lme2510_dtmb_demod_read(state, LME2510_DTMB_REG_DETECTED,
				      &detected);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_update_bits(state, 0x7e, BIT(0), 0);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_alt_update_bits(state, 0xc5, 0x1f, 0x06);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_reset(state);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_write(state, 0x7d, detected);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_reset(state);
	if (ret)
		return ret;
	state->detected_parameters = detected;
	return 0;
}

static int lme2510_dtmb_acquire(struct lme2510_dtmb_state *state)
{
	static const u8 modes[] = { 0, 2, 4 };
	u8 inversion_reg = LME2510_DTMB_REG_INVERSION;
	u8 lock_reg = LME2510_DTMB_REG_LOCK;
	u8 inversion;
	int ret;
	bool transport_error;
	int attempt;
	int i;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		for (attempt = 0; attempt < 3; attempt++) {
			ret = lme2510_dtmb_capture_mode(state, modes[i]);
			if (ret)
				return ret;
			ret = lme2510_dtmb_wait_mask(state,
						     LME2510_DTMB_REG_SIGNAL,
						     0x80, 0x80, 30, 30,
						     NULL, &transport_error);
			if (ret == -ETIMEDOUT && !transport_error)
				break;
			if (ret)
				return ret;
			ret = lme2510_dtmb_wait_mask(state, lock_reg, 0x03,
						     0x01, 15, 20, NULL,
						     &transport_error);
			if (!ret)
				return lme2510_dtmb_finalize_lock(state);
			if (ret && (ret != -ETIMEDOUT || transport_error))
				return ret;

			ret = lme2510_dtmb_demod_read(state, inversion_reg,
						      &inversion);
			if (ret)
				return ret;
			ret = lme2510_dtmb_demod_write(state, inversion_reg,
						       inversion ^ BIT(7));
			if (ret)
				return ret;
		}
	}

	/* No lock in any mode: report failure, do not fake a tuned state. */
	return -ETIMEDOUT;
}

static int lme2510_dtmb_tune(struct lme2510_dtmb_state *state, u32 frequency_hz)
{
	u32 frequency_khz = frequency_hz / 1000;
	int ret;

	state->detected_parameters = 0;
	state->tuned = false;
	if (frequency_hz < 450 * MHz || frequency_hz > 880 * MHz)
		return -EINVAL;

	ret = lme2510_dtmb_demod_reset(state);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_write(state, 0x01, 0xe0);
	if (ret)
		return ret;
	ret = lme2510_dtmb_tuner_set_frequency(state, frequency_khz);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_write(state, 0x01, 0x60);
	if (ret)
		return ret;
	ret = lme2510_dtmb_post_tuner_setup(state);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_reset(state);
	if (ret)
		return ret;
	ret = lme2510_dtmb_acquire(state);
	if (ret)
		return ret;
	state->tuned = true;
	return 0;
}

static void
lme2510_dtmb_decode_parameters(struct dtv_frontend_properties *properties,
			       u8 detected)
{
	switch ((detected >> 6) & 0x03) {
	case 0:
		properties->transmission_mode = TRANSMISSION_MODE_C1;
		break;
	case 1:
		properties->transmission_mode = TRANSMISSION_MODE_C3780;
		break;
	default:
		properties->transmission_mode = TRANSMISSION_MODE_AUTO;
		break;
	}

	switch (detected & 0x07) {
	case 0:
		properties->modulation = QAM_4_NR;
		break;
	case 1:
		properties->modulation = QPSK;
		break;
	case 2:
		properties->modulation = QAM_16;
		break;
	case 3:
		properties->modulation = QAM_32;
		break;
	case 4:
		properties->modulation = QAM_64;
		break;
	default:
		properties->modulation = QAM_AUTO;
		break;
	}

	switch ((detected >> 3) & 0x03) {
	case 0:
		properties->code_rate_HP = FEC_2_5;
		break;
	case 1:
		properties->code_rate_HP = FEC_3_5;
		break;
	case 2:
		properties->code_rate_HP = FEC_4_5;
		break;
	default:
		properties->code_rate_HP = FEC_AUTO;
		break;
	}

	properties->interleaving = detected & BIT(5) ? INTERLEAVING_720 :
						       INTERLEAVING_240;
}

static u16 lme2510_dtmb_percent(u8 percent)
{
	return min_t(u8, percent, 100) * 0xffffU / 100;
}

/* Firmware/BDA relative quality and AGC curves; these are not dB values. */
static u8 lme2510_dtmb_stream_quality(const struct lme2510_dtmb_status *status,
				       bool locked)
{
	u16 raw = status->quality_high << 8 | status->quality_low;
	u8 low = status->quality_low & 0x07;
	u8 strength;

	if (!locked)
		strength = low + 8;
	else if (raw >= 0x1f00 && raw <= 0x1fff)
		strength = low + 80;
	else if (raw >= 0x0058 && raw <= 0x00ff)
		strength = low + 70;
	else if (raw >= 0x0100 && raw <= 0x0180)
		strength = low + 60;
	else if (raw >= 0x0180 && raw <= 0x0200)
		strength = low + 50;
	else
		strength = 69;

	return min_t(u8, strength, 100);
}

static u8 lme2510_dtmb_stream_strength(u8 raw, bool locked)
{
	u8 scaled = (u32)raw * 100 / 255;
	u8 remainder = raw % 15;

	if (!locked)
		return remainder + 5;
	if (!scaled)
		return remainder + 30;
	if (scaled <= 40)
		return 90 - scaled;
	if (scaled >= 90)
		return 30;
	return 100 - scaled * 3 / 4;
}

static void lme2510_dtmb_status_snapshot(struct lme2510_dtmb_state *state,
					 struct lme2510_dtmb_status *status,
					 unsigned long *updated)
{
	unsigned long flags;

	spin_lock_irqsave(&state->stream_lock, flags);
	*status = state->stream_status;
	if (updated)
		*updated = state->stream_status_updated;
	spin_unlock_irqrestore(&state->stream_lock, flags);
}

static bool
lme2510_dtmb_status_is_live(const struct lme2510_dtmb_status *status,
			    unsigned long updated)
{
	return status->valid && updated &&
	       time_before(jiffies, updated +
			   msecs_to_jiffies(LME2510_DTMB_STREAM_STALE_MS));
}



static void
lme2510_dtmb_set_statistics(struct dtv_frontend_properties *properties,
			    const struct lme2510_dtmb_status *stream_status,
			    u8 signal, bool locked)
{
	u8 quality;
	u8 strength;

	if (stream_status->valid) {
		strength = lme2510_dtmb_stream_strength(stream_status->strength, locked);
		quality =
			lme2510_dtmb_stream_quality(stream_status, locked);
	} else {
		strength = (signal & 0x7f) * 100 / 127;
		quality = 0;
	}

	properties->strength.len = 1;
	properties->strength.stat[0].scale = FE_SCALE_RELATIVE;
	properties->strength.stat[0].uvalue = lme2510_dtmb_percent(strength);
	properties->cnr.len = 1;
	properties->cnr.stat[0].scale = locked && stream_status->valid ?
		FE_SCALE_RELATIVE : FE_SCALE_NOT_AVAILABLE;
	properties->cnr.stat[0].uvalue =
		locked && stream_status->valid ? lme2510_dtmb_percent(quality) : 0;
}

static void lme2510_dtmb_invalidate_stats(struct dvb_frontend *frontend)
{
	struct dtv_frontend_properties *properties = &frontend->dtv_property_cache;

	properties->strength.len = 1;
	properties->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	properties->strength.stat[0].uvalue = 0;
	properties->cnr.len = 1;
	properties->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	properties->cnr.stat[0].uvalue = 0;
}

static int lme2510_dtmb_frontend_init(struct dvb_frontend *frontend)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;
	int ret = 0;

	ret = mutex_lock_interruptible(&state->tune_lock);
	if (ret)
		return ret;
	if (state->disconnected) {
		ret = -ENODEV;
		goto unlock;
	}
	if (!state->status_urb) {
		state->status_urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!state->status_urb) {
			ret = -ENOMEM;
			goto unlock;
		}
	}
	if (!state->initialized) {
		ret = lme2510_dtmb_frontend_initialize(state);
		if (!ret)
			state->initialized = true;
	}

unlock:
	mutex_unlock(&state->tune_lock);
	return ret;
}

static int lme2510_dtmb_frontend_sleep(struct dvb_frontend *frontend)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;
	int ret;

	mutex_lock(&state->tune_lock);
	state->stream_requested = false;
	ret = lme2510_dtmb_stop_stream_locked(state);
	state->initialized = false;
	state->tuned = false;
	mutex_unlock(&state->tune_lock);
	return ret;
}

static int lme2510_dtmb_set_frontend(struct dvb_frontend *frontend)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;
	u32 frequency = frontend->dtv_property_cache.frequency;
	bool stream_requested;
	int ret;

	ret = mutex_lock_interruptible(&state->tune_lock);
	if (ret)
		return ret;
	if (state->disconnected) {
		ret = -ENODEV;
		goto unlock;
	}
	stream_requested = READ_ONCE(state->stream_requested);
	if (stream_requested) {
		ret = lme2510_dtmb_stop_stream_locked(state);
		if (ret)
			goto unlock;
	}
	lme2510_dtmb_invalidate_stats(frontend);
	ret = lme2510_dtmb_tune(state, frequency);
	/* Retuning resets the bridge PID commit; restore active feeds afterward. */
	if (!ret && stream_requested)
		ret = lme2510_dtmb_start_stream_locked(frontend);
unlock:
	mutex_unlock(&state->tune_lock);
	return ret;
}

static int lme2510_dtmb_read_status(struct dvb_frontend *frontend,
				    enum fe_status *status)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;
	struct dtv_frontend_properties *properties = &frontend->dtv_property_cache;
	struct lme2510_dtmb_status stream_status;
	unsigned long updated;
	u8 signal = 0, lock = 0;
	bool locked;
	int ret;

	*status = 0;
	ret = mutex_lock_interruptible(&state->tune_lock);
	if (ret)
		return ret;
	lme2510_dtmb_invalidate_stats(frontend);
	if (state->disconnected) {
		ret = -ENODEV;
		goto unlock;
	}
	lme2510_dtmb_status_snapshot(state, &stream_status, &updated);
	if (atomic_read(&state->streaming)) {
		if (!state->tuned)
			goto unlock;
		/* Keep EP01/81 idle during TS capture; stale status is unknown. */
		if (!lme2510_dtmb_status_is_live(&stream_status, updated))
			goto unlock;
		locked = stream_status.lock != 0 &&
			 !(stream_status.strength == 0xff && stream_status.extra == 0xff);
	} else {
		stream_status.valid = false;
		ret = lme2510_dtmb_demod_read(state, LME2510_DTMB_REG_SIGNAL, &signal);
		if (ret)
			goto unlock;
		ret = lme2510_dtmb_demod_read(state, LME2510_DTMB_REG_LOCK, &lock);
		if (ret)
			goto unlock;
		if (signal & 0x7f)
			*status |= FE_HAS_SIGNAL;
		if (signal & BIT(7))
			*status |= FE_HAS_CARRIER;
		locked = (signal & BIT(7)) && (lock & 0x03) == 0x01;
	}
	if (locked) {
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
			  FE_HAS_SYNC | FE_HAS_LOCK;
		state->tuned = true;
		lme2510_dtmb_decode_parameters(properties, state->detected_parameters);
	}
	lme2510_dtmb_set_statistics(properties, &stream_status, signal, locked);
unlock:
	mutex_unlock(&state->tune_lock);
	return ret;
}

static int lme2510_dtmb_get_frontend(struct dvb_frontend *frontend,
				     struct dtv_frontend_properties *properties)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;

	mutex_lock(&state->tune_lock);
	if (state->tuned)
		lme2510_dtmb_decode_parameters(properties, state->detected_parameters);
	mutex_unlock(&state->tune_lock);
	return 0;
}

static enum dvbfe_algo
lme2510_dtmb_get_frontend_algo(struct dvb_frontend *frontend)
{
	return DVBFE_ALGO_HW;
}

static int lme2510_dtmb_frontend_tune(struct dvb_frontend *frontend,
				    bool re_tune, unsigned int mode_flags,
				    unsigned int *delay, enum fe_status *status)
{
	int ret;

	*delay = HZ / 4;
	*status = 0;
	/* Acquisition is completed by set_frontend; polls must not reset TS. */
	if (re_tune) {
		ret = lme2510_dtmb_set_frontend(frontend);
		if (ret)
			return ret;
	}
	return lme2510_dtmb_read_status(frontend, status);
}

static int lme2510_dtmb_read_signal_strength(struct dvb_frontend *frontend,
					     u16 *strength)
{
	struct dtv_fe_stats *statistics = &frontend->dtv_property_cache.strength;

	*strength = 0;
	if (statistics->stat[0].scale != FE_SCALE_RELATIVE)
		return -EAGAIN;
	*strength = statistics->stat[0].uvalue;
	return 0;
}

static int lme2510_dtmb_read_snr(struct dvb_frontend *frontend, u16 *snr)
{
	struct dtv_fe_stats *statistics = &frontend->dtv_property_cache.cnr;

	*snr = 0;
	if (statistics->stat[0].scale != FE_SCALE_RELATIVE)
		return -EAGAIN;
	*snr = statistics->stat[0].uvalue;
	return 0;
}


static void lme2510_dtmb_parse_status(struct lme2510_dtmb_state *state,
				      const u8 *data, size_t length)
{
	while (length >= 8) {
		if (data[0] == 0xbb) {
			state->stream_status.valid = true;
			state->stream_status.lock = data[2];
			state->stream_status.quality_low = data[3];
			state->stream_status.strength = data[4];
			state->stream_status.quality_high = data[5];
			state->stream_status.extra = data[6];
			state->stream_status_updated = jiffies;
		}
		data += 8;
		length -= 8;
	}
}
static void lme2510_dtmb_status_complete(struct urb *urb)
{
	struct lme2510_dtmb_state *state = urb->context;
	unsigned long flags;
	int ret;

	if (!state)
		return;
	if (!urb->status && urb->actual_length >= 8 &&
	    urb->actual_length <= sizeof(state->status_buffer)) {
		spin_lock_irqsave(&state->stream_lock, flags);
		lme2510_dtmb_parse_status(state, state->status_buffer,
				  urb->actual_length);
		spin_unlock_irqrestore(&state->stream_lock, flags);
	}
	if (!atomic_read(&state->streaming) || urb->status == -ENOENT ||
	    urb->status == -ECONNRESET || urb->status == -ESHUTDOWN ||
	    urb->status == -ENODEV)
		return;

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret && ret != -ENODEV && ret != -ESHUTDOWN)
		dev_warn_ratelimited(&state->d->udev->dev,
				     "status URB resubmit failed: %d\n", ret);
}


static int lme2510_dtmb_start_status(struct lme2510_dtmb_state *state)
{
	struct usb_device *udev = state->d->udev;
	struct usb_host_endpoint *ep = usb_pipe_endpoint(udev,
		usb_rcvintpipe(udev, LME2510_DTMB_EP_STATUS_IN));

	/* Some variants have no asynchronous status endpoint. */
	if (!ep)
		return 0;
	if (!state->status_urb)
		return -ENOMEM;
	if (usb_endpoint_xfer_int(&ep->desc))
		usb_fill_int_urb(state->status_urb, udev,
			usb_rcvintpipe(udev, LME2510_DTMB_EP_STATUS_IN),
			state->status_buffer, sizeof(state->status_buffer),
			lme2510_dtmb_status_complete, state, ep->desc.bInterval);
	else if (usb_endpoint_xfer_bulk(&ep->desc))
		usb_fill_bulk_urb(state->status_urb, udev,
			usb_rcvbulkpipe(udev, LME2510_DTMB_EP_STATUS_IN),
			state->status_buffer, sizeof(state->status_buffer),
			lme2510_dtmb_status_complete, state);
	else
		return -EINVAL;
	return usb_submit_urb(state->status_urb, GFP_KERNEL);
}


static void lme2510_dtmb_reset_stream(struct lme2510_dtmb_state *state)
{
	unsigned long flags;

	spin_lock_irqsave(&state->stream_lock, flags);
	memset(&state->stream_status, 0, sizeof(state->stream_status));
	state->stream_status_updated = 0;
	spin_unlock_irqrestore(&state->stream_lock, flags);
}


static void lme2510_dtmb_stream_complete(struct usb_data_stream *stream,
					 u8 *data, size_t length)
{
	struct dvb_usb_adapter *adapter = stream->user_priv;
	struct lme2510_dtmb_state *state = adap_to_priv(adapter);
	if (!atomic_read(&state->streaming) || !length)
		return;

	dvb_dmx_swfilter(&adapter->demux, data, length);
}




static int lme2510_dtmb_start_transport(struct lme2510_dtmb_state *state)
{
	static const u8 all_pids[] = { 0x03, 0x06, 0x00, 0xff,
				       0x01, 0x1f, 0x20, 0x81 };
	static const u8 start[] = { 0x06, 0x00 };
	int ret;

	/* Even a failed handshake may have enabled part of the bridge. */
	state->transport_active = true;
	ret = lme2510_dtmb_command(state, all_pids, sizeof(all_pids), 5);
	if (ret)
		return ret;
	ret = lme2510_dtmb_command(state, all_pids, sizeof(all_pids), 5);
	if (ret)
		return ret;

	return lme2510_dtmb_command(state, start, sizeof(start), 5);
}

static int lme2510_dtmb_stop_stream_locked(struct lme2510_dtmb_state *state)
{
	static const u8 clear[] = { 0x03, 0x02, 0x20, 0xa0 };
	u8 response;
	size_t actual;
	int ret = 0;

	atomic_set(&state->streaming, 0);
	if (state->status_urb)
		usb_kill_urb(state->status_urb);
	if (state->transport_active && !state->disconnected) {
		ret = lme2510_dtmb_exchange(state, clear, sizeof(clear),
					  &response, sizeof(response), &actual);
		if (!ret) {
			/* In stream mode CMD03 only exits it and replies 0x77. */
			if (response == 0x77)
				ret = lme2510_dtmb_command(state, clear,
							 sizeof(clear), 1);
			else if (!lme2510_dtmb_valid_ack(response))
				ret = -EREMOTEIO;
		}
	}
	if (!ret || state->disconnected)
		state->transport_active = false;
	lme2510_dtmb_reset_stream(state);
	lme2510_dtmb_invalidate_stats(&state->frontend);
	return ret;
}

static int lme2510_dtmb_start_stream_locked(struct dvb_frontend *frontend)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;
	int ret;

	atomic_set(&state->streaming, 0);
	if (state->status_urb)
		usb_kill_urb(state->status_urb);
	lme2510_dtmb_reset_stream(state);
	ret = lme2510_dtmb_start_transport(state);
	if (ret)
		goto err_stop_streaming;
	atomic_set(&state->streaming, 1);
	ret = lme2510_dtmb_start_status(state);
	if (ret)
		goto err_stop_streaming;

	return 0;

err_stop_streaming:
	lme2510_dtmb_stop_stream_locked(state);
	return ret;
}


static int lme2510_dtmb_streaming_ctrl(struct dvb_frontend *frontend,
				       int enable)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;
	int ret;

	ret = mutex_lock_interruptible(&state->tune_lock);
	if (ret)
		return ret;
	if (state->disconnected) {
		ret = -ENODEV;
		goto unlock;
	}

	if (!enable) {
		WRITE_ONCE(state->stream_requested, false);
		ret = lme2510_dtmb_stop_stream_locked(state);
		goto unlock;
	}

	WRITE_ONCE(state->stream_requested, true);
	ret = lme2510_dtmb_start_stream_locked(frontend);
	if (ret)
		WRITE_ONCE(state->stream_requested, false);
unlock:
	mutex_unlock(&state->tune_lock);
	return ret;
}

static int lme2510_dtmb_frontend_detach(struct dvb_usb_adapter *adapter)
{
	struct lme2510_dtmb_state *state = adap_to_priv(adapter);

	/* Framework unwind can precede frontend_attach(). */
	if (!state->d)
		return 0;

	mutex_lock(&state->tune_lock);
	state->stream_requested = false;
	lme2510_dtmb_stop_stream_locked(state);
	usb_free_urb(state->status_urb);
	state->status_urb = NULL;
	mutex_unlock(&state->tune_lock);

	return 0;
}

static const struct dvb_frontend_ops lme2510_dtmb_frontend_ops = {
	.delsys = { SYS_DVBT, SYS_DTMB },
	.info = {
		.name = "LME2510C LGS8GL5 DTMB",
		.frequency_min_hz = 450 * MHz,
		.frequency_max_hz = 880 * MHz,
		.frequency_stepsize_hz = MHz,
		.caps = FE_CAN_FEC_AUTO | FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO | FE_CAN_BANDWIDTH_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO,
	},
	.init = lme2510_dtmb_frontend_init,
	.sleep = lme2510_dtmb_frontend_sleep,
	.set_frontend = lme2510_dtmb_set_frontend,
	.get_frontend = lme2510_dtmb_get_frontend,
	.get_frontend_algo = lme2510_dtmb_get_frontend_algo,
	.tune = lme2510_dtmb_frontend_tune,
	.read_status = lme2510_dtmb_read_status,
	.read_signal_strength = lme2510_dtmb_read_signal_strength,
	.read_snr = lme2510_dtmb_read_snr,
};

static int lme2510_dtmb_frontend_attach(struct dvb_usb_adapter *adapter)
{
	struct lme2510_dtmb_state *state = adap_to_priv(adapter);

	state->d = adap_to_d(adapter);
	mutex_init(&state->protocol_lock);
	mutex_init(&state->tune_lock);
	spin_lock_init(&state->stream_lock);
	atomic_set(&state->streaming, 0);
	state->frontend.ops = lme2510_dtmb_frontend_ops;
	state->frontend.demodulator_priv = state;
	lme2510_dtmb_invalidate_stats(&state->frontend);
	adapter->fe[0] = &state->frontend;
	return 0;
}

static int lme2510_dtmb_identify_state(struct dvb_usb_device *d,
				       const char **firmware_name)
{
	return WARM;
}

static const struct dvb_usb_device_properties lme2510_dtmb_properties = {
	.driver_name = KBUILD_MODNAME,
	.owner = THIS_MODULE,
	.adapter_nr = adapter_nr,
	.size_of_priv = sizeof(struct lme2510_dtmb_state),
	.identify_state = lme2510_dtmb_identify_state,
	.frontend_attach = lme2510_dtmb_frontend_attach,
	.frontend_detach = lme2510_dtmb_frontend_detach,
	.streaming_ctrl = lme2510_dtmb_streaming_ctrl,
	.num_adapters = 1,
	.adapter = {
			{
				.stream_complete = lme2510_dtmb_stream_complete,
				.stream = DVB_USB_STREAM_BULK(LME2510_DTMB_EP_TS_IN, 10,
						     LME2510_DTMB_TS_BUFFER_SIZE),
		},
	},
};

#define LME2510_DTMB_DEVICE(vid, pid, label)                              \
	{ DVB_USB_DEVICE((vid), (pid), &lme2510_dtmb_properties, (label), \
			 NULL) }

static const struct usb_device_id lme2510_dtmb_id_table[] = {
	LME2510_DTMB_DEVICE(LME2510_DTMB_VID, LME2510_DTMB_COLD_PID,
			    "LME2510C DTMB receiver (cold)"),
	LME2510_DTMB_DEVICE(LME2510_DTMB_VID, LME2510_DTMB_PID_1120,
			    "LME2510C DTMB receiver"),
	LME2510_DTMB_DEVICE(LME2510_DTMB_VID, LME2510_DTMB_PID_1122,
			    "LME2510C DTMB receiver"),
	LME2510_DTMB_DEVICE(LME2510_DTMB_VID, LME2510_DTMB_PID_22D0,
			    "LME2510C DTMB receiver"),
	LME2510_DTMB_DEVICE(LME2510_DTMB_VID, LME2510_DTMB_PID_22D8,
			    "LME2510C DTMB receiver"),
	LME2510_DTMB_DEVICE(LME2510_DTMB_VID, LME2510_DTMB_PID_E388,
			    "LME2510C DTMB receiver"),
	LME2510_DTMB_DEVICE(LME2510_DTMB_OEM_VID, LME2510_DTMB_PID_E388,
			    "LME2510C DTMB receiver"),
	{}
};
MODULE_DEVICE_TABLE(usb, lme2510_dtmb_id_table);

static int lme2510_dtmb_clear_halt(struct usb_interface *interface,
				   unsigned int pipe)
{
	int ret = usb_clear_halt(interface_to_usbdev(interface), pipe);

	/* Some runtime firmwares reject CLEAR_FEATURE while accepting commands. */
	return ret == -EPIPE ? 0 : ret;
}

static int lme2510_dtmb_prepare_runtime(struct usb_interface *interface)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	u8 number = interface->cur_altsetting->desc.bInterfaceNumber;
	unsigned int pipe;
	int ret;

	/* Probe only: no TS or status URBs may be queued while resetting pipes. */
	ret = usb_set_interface(udev, number, 1);
	if (ret)
		return ret;
	pipe = usb_sndbulkpipe(udev, LME2510_DTMB_EP_COMMAND_OUT);
	ret = lme2510_dtmb_clear_halt(interface, pipe);
	if (ret)
		return ret;
	pipe = usb_rcvbulkpipe(udev, LME2510_DTMB_EP_COMMAND_IN);
	ret = lme2510_dtmb_clear_halt(interface, pipe);
	if (ret)
		return ret;
	pipe = usb_rcvbulkpipe(udev, LME2510_DTMB_EP_TS_IN);
	return lme2510_dtmb_clear_halt(interface, pipe);
}

static int lme2510_dtmb_runtime_ready(struct usb_interface *interface)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	static const u8 request[] = { 0x85, 0x02, LME2510_DTMB_DEMOD_SELECTOR,
				      0x00, 0x00 };
	u8 *buffer;
	int actual = 0;
	int ret;

	buffer = kmalloc(LME2510_DTMB_COMMAND_BUFFER_SIZE, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	memcpy(buffer, request, sizeof(request));
	ret = usb_bulk_msg(udev,
			   usb_sndbulkpipe(udev, LME2510_DTMB_EP_COMMAND_OUT),
			   buffer, sizeof(request), &actual, 500);
	if (ret)
		goto free_buffer;
	if (actual != sizeof(request)) {
		ret = -EREMOTEIO;
		goto free_buffer;
	}

	memset(buffer, 0, LME2510_DTMB_COMMAND_BUFFER_SIZE);
	ret = usb_bulk_msg(udev,
			   usb_rcvbulkpipe(udev, LME2510_DTMB_EP_COMMAND_IN),
			   buffer, LME2510_DTMB_COMMAND_BUFFER_SIZE, &actual, 500);
	if (!ret && (actual != 2 || !lme2510_dtmb_valid_ack(buffer[0])))
		ret = -EREMOTEIO;

free_buffer:
	kfree(buffer);
	return ret;
}




static int lme2510_dtmb_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	u16 pid = le16_to_cpu(udev->descriptor.idProduct);
	bool cold = pid == LME2510_DTMB_COLD_PID;
	int ret;

	if (interface->cur_altsetting->desc.bInterfaceNumber)
		return -ENODEV;
	if (!cold) {
		ret = lme2510_dtmb_prepare_runtime(interface);
		if (ret)
			return ret;
		ret = lme2510_dtmb_runtime_ready(interface);
		if (!ret)
			return dvb_usbv2_probe(interface, id);
		if (ret != -ETIMEDOUT && ret != -EPIPE && ret != -EREMOTEIO)
			return ret;
	}

	ret = lme2510_dtmb_download_firmware(udev);
	if (ret)
		return ret;

	/* The bootloader disconnects and returns with one of the runtime IDs. */
	return -ENODEV;
}

static void lme2510_dtmb_disconnect(struct usb_interface *interface)
{
	struct dvb_usb_device *d = usb_get_intfdata(interface);
	struct lme2510_dtmb_state *state;

	if (!d)
		return;
	state = d_to_priv(d);
	if (state->d) {
		mutex_lock(&state->tune_lock);
		state->disconnected = true;
		state->stream_requested = false;
		lme2510_dtmb_stop_stream_locked(state);
		mutex_unlock(&state->tune_lock);
	}
	dvb_usbv2_disconnect(interface);
}

static struct usb_driver lme2510_dtmb_driver = {
	.name = KBUILD_MODNAME,
	.id_table = lme2510_dtmb_id_table,
	.probe = lme2510_dtmb_probe,
	.disconnect = lme2510_dtmb_disconnect,
	.suspend = dvb_usbv2_suspend,
	.resume = dvb_usbv2_resume,
	.reset_resume = dvb_usbv2_reset_resume,
	.no_dynamic_id = 1,
	.soft_unbind = 1,
};
module_usb_driver(lme2510_dtmb_driver);

MODULE_AUTHOR("wwek");
MODULE_DESCRIPTION("LME2510C LGS8GL5 MAX2165 DTMB USB receiver driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(LME2510_DTMB_FW_ROM);
MODULE_FIRMWARE(LME2510_DTMB_FW_MAIN);
MODULE_FIRMWARE(LME2510_DTMB_FW_22D0);
