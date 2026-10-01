// SPDX-License-Identifier: GPL-2.0-only
/*
 * LME2510C DTMB USB receiver driver
 *
 * Copyright (C) 2026 wwek <licoolgo@gmail.com>
 */

#include <linux/bitmap.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

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
#define LME2510_DTMB_FIRMWARE_PAYLOAD_SIZE 50
#define LME2510_DTMB_TS_PACKET_SIZE 188
#define LME2510_DTMB_TS_PID_COUNT 8192
#define LME2510_DTMB_TS_BUFFER_SIZE 4096
#define LME2510_DTMB_TS_SYNC_PACKETS 4
#define LME2510_DTMB_TS_CARRY_SIZE \
	(LME2510_DTMB_TS_SYNC_PACKETS * LME2510_DTMB_TS_PACKET_SIZE - 1)
#define LME2510_DTMB_STREAM_VALIDATE_MS 300
#define LME2510_DTMB_STREAM_MONITOR_MS 1000
#define LME2510_DTMB_STREAM_MIN_PACKETS 64
#define LME2510_DTMB_STREAM_VALID_PERCENT 95
#define LME2510_DTMB_STREAM_MAX_TEI_PERCENT 1
#define LME2510_DTMB_STREAM_MAX_CC_PERCENT 2
#define LME2510_DTMB_STREAM_WINDOW_BYTES (64 * 1024)
#define LME2510_DTMB_STREAM_STALE_MS 2000
#define LME2510_DTMB_STATUS_SETTLE_PACKETS 4

struct lme2510_dtmb_status {
	bool valid;
	u8 lock;
	u8 signal_low;
	u8 snr;
	u8 signal_high;
	u8 extra;
};

struct lme2510_dtmb_stream_sample {
	u64 bytes;
	u64 packets;
	u64 tei;
	u64 cc;
};

struct lme2510_dtmb_state {
	struct dvb_usb_device *d;
	struct dvb_frontend frontend;
	struct mutex protocol_lock; /* Serializes command/response transfers. */
	struct mutex tune_lock; /* Serializes frontend and stream transactions. */
	u8 command_buffer[LME2510_DTMB_COMMAND_BUFFER_SIZE];
	u8 demod_read_hint;

	struct urb *status_urb;
	struct delayed_work stream_work;
	u8 status_buffer[LME2510_DTMB_COMMAND_BUFFER_SIZE];
	u8 status_settle_packets;
	atomic_t streaming;
	bool stream_requested;
	spinlock_t stream_lock; /* Protects stream reassembly and status data. */
	u8 stream_buffer[LME2510_DTMB_TS_BUFFER_SIZE +
			 LME2510_DTMB_TS_CARRY_SIZE];
	size_t stream_carry;
	struct lme2510_dtmb_status stream_status;
	u64 stream_bytes;
	u64 stream_packets;
	u64 stream_tei;
	u64 stream_cc_errors;
	u64 stream_window_bytes;
	u64 stream_window_packets;
	u64 stream_window_tei;
	u64 stream_window_cc_errors;
	unsigned long stream_started;
	unsigned long stream_last_packet;
	bool stream_validated;
	bool stream_force_restart;
	u8 stream_cc[LME2510_DTMB_TS_PID_COUNT];
	DECLARE_BITMAP(stream_cc_valid, LME2510_DTMB_TS_PID_COUNT);

	u8 tuner_calibration[5];
	u8 tuner_fraction_high;
	u8 tuner_reg_a;
	u8 detected_parameters;
	u8 tuned_strength;
	u32 tuned_frequency;
	bool initialized;
	bool tuned;
};

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static int lme2510_dtmb_start_stream_locked(struct dvb_frontend *frontend);
static int lme2510_dtmb_stop_stream_locked(struct lme2510_dtmb_state *state);
static void lme2510_dtmb_stream_work(struct work_struct *work);

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

	memset(state->command_buffer, 0, response_length);
	ret = usb_bulk_msg(udev,
			   usb_rcvbulkpipe(udev, LME2510_DTMB_EP_COMMAND_IN),
			   state->command_buffer, response_length, &actual,
			   1500);
	if (ret == -EOVERFLOW && actual > 0)
		ret = 0;
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
				      bool response_optional)
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
	if (response_optional && (ret == -ETIMEDOUT || ret == -EOVERFLOW ||
				  ret == -ENODEV || ret == -ESHUTDOWN))
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
						 true);
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
				   u32 frequency_mhz, u8 registers[5])
{
	u32 fraction =
		((frequency_mhz % LME2510_DTMB_TUNER_REFERENCE_MHZ) << 20) /
		LME2510_DTMB_TUNER_REFERENCE_MHZ;
	int lpf_low = state->tuner_calibration[1] & 0x0f;
	int lpf_high = state->tuner_calibration[1] >> 4;
	int lpf;
	u8 band;

	state->tuner_fraction_high &= 0xf0;
	state->tuner_fraction_high |= (fraction >> 16) & 0x0f;
	band = frequency_mhz < 725 ? state->tuner_calibration[0] >> 4 :
				     state->tuner_calibration[0] & 0x0f;
	lpf = lpf_low + (lpf_high - lpf_low) * ((int)frequency_mhz - 470) / 310;

	registers[0] = frequency_mhz / LME2510_DTMB_TUNER_REFERENCE_MHZ;
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

	lme2510_dtmb_tuner_pll(state, 474, registers);
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
	state->status_settle_packets = LME2510_DTMB_STATUS_SETTLE_PACKETS;
	ret = lme2510_dtmb_demod_read(state, 0x00, &dummy);
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
	return lme2510_dtmb_demod_write(state, LME2510_DTMB_REG_MODE,
					reg & 0x7c);
}

static int lme2510_dtmb_tuner_set_frequency(struct lme2510_dtmb_state *state,
					    u32 frequency_mhz)
{
	u8 registers[5];
	u8 reg;
	int ret;

	lme2510_dtmb_tuner_pll(state, frequency_mhz, registers);
	ret = lme2510_dtmb_tuner_write_block(state, 0x00, registers,
					     sizeof(registers));
	if (ret)
		return ret;
	ret = lme2510_dtmb_tuner_write(state, 0x0a,
				       lme2510_dtmb_tuner_reg_a(state));
	if (ret)
		return ret;
	ret = lme2510_dtmb_tuner_read(state, 0x04, &reg);
	if (ret)
		return ret;
	return lme2510_dtmb_tuner_write(state, 0x04, reg | 0xf0);
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
				  unsigned int delay_ms, u8 *last)
{
	u8 value = 0;
	int ret;

	while (attempts--) {
		msleep(delay_ms);
		ret = lme2510_dtmb_demod_read(state, reg, &value);
		if (ret)
			return ret;
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
	u8 strength;
	int ret;

	ret = lme2510_dtmb_demod_read(state, LME2510_DTMB_REG_DETECTED,
				      &detected);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_read(state, LME2510_DTMB_REG_SIGNAL,
				      &strength);
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
	state->tuned_strength = strength;
	return 0;
}

static int lme2510_dtmb_acquire(struct lme2510_dtmb_state *state)
{
	static const u8 modes[] = { 0, 2, 4 };
	u8 inversion_reg = LME2510_DTMB_REG_INVERSION;
	u8 lock_reg = LME2510_DTMB_REG_LOCK;
	u8 inversion;
	int ret;
	int attempt;
	int i;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		for (attempt = 0; attempt < 3; attempt++) {
			ret = lme2510_dtmb_capture_mode(state, modes[i]);
			if (ret)
				return ret;
			ret = lme2510_dtmb_wait_mask(state,
						     LME2510_DTMB_REG_SIGNAL,
						     BIT(7), BIT(7), 20, 30,
						     NULL);
			if (ret == -ETIMEDOUT)
				break;
			if (ret)
				return ret;
			ret = lme2510_dtmb_wait_mask(state, lock_reg, 0x03,
						     0x01, 10, 20, NULL);
			if (!ret)
				return lme2510_dtmb_finalize_lock(state);
			if (ret != -ETIMEDOUT)
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

	return -ETIMEDOUT;
}

static int lme2510_dtmb_tune(struct lme2510_dtmb_state *state, u32 frequency_hz)
{
	u32 frequency_khz = frequency_hz / 1000;
	int ret;

	frequency_khz = frequency_khz / 1000 * 1000;
	if (frequency_khz < 450000 || frequency_khz > 880000)
		return -EINVAL;

	state->detected_parameters = 0;
	state->tuned_strength = 0;
	state->tuned_frequency = 0;
	state->tuned = false;

	ret = lme2510_dtmb_demod_reset(state);
	if (ret)
		return ret;
	ret = lme2510_dtmb_demod_write(state, 0x01, 0xe0);
	if (ret)
		return ret;
	ret = lme2510_dtmb_tuner_set_frequency(state, frequency_khz / 1000);
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
	if (!ret) {
		state->tuned_frequency = frequency_khz * 1000;
		state->tuned = true;
	}
	return ret;
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
		properties->code_rate_HP = FEC_2_3;
		break;
	case 2:
		properties->code_rate_HP = FEC_3_4;
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

static u8 lme2510_dtmb_stream_strength(const struct lme2510_dtmb_status *status,
				       bool locked)
{
	u16 raw = status->signal_high << 8 | status->signal_low;
	u8 low = status->signal_low & 0x07;
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

static u8 lme2510_dtmb_stream_quality(u8 raw, bool locked)
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
					 struct lme2510_dtmb_status *status)
{
	unsigned long flags;

	spin_lock_irqsave(&state->stream_lock, flags);
	*status = state->stream_status;
	spin_unlock_irqrestore(&state->stream_lock, flags);
}

static bool lme2510_dtmb_stream_is_live(struct lme2510_dtmb_state *state)
{
	unsigned long deadline;
	unsigned long flags;
	bool live;

	spin_lock_irqsave(&state->stream_lock, flags);
	if (!state->stream_validated) {
		live = false;
	} else {
		deadline =
			(state->stream_last_packet ?: state->stream_started) +
			msecs_to_jiffies(LME2510_DTMB_STREAM_STALE_MS);
		live = time_before(jiffies, deadline);
	}
	spin_unlock_irqrestore(&state->stream_lock, flags);

	return live;
}

static void
lme2510_dtmb_set_statistics(struct lme2510_dtmb_state *state,
			    struct dtv_frontend_properties *properties,
			    const struct lme2510_dtmb_status *stream_status,
			    u8 signal, bool locked)
{
	u8 quality;
	u8 strength;

	if (stream_status->valid) {
		strength = lme2510_dtmb_stream_strength(stream_status, locked);
		quality =
			lme2510_dtmb_stream_quality(stream_status->snr, locked);
	} else {
		strength = (signal & 0x7f) * 100 / 127;
		quality = strength;
	}

	properties->strength.len = 1;
	properties->strength.stat[0].scale = FE_SCALE_RELATIVE;
	properties->strength.stat[0].uvalue = lme2510_dtmb_percent(strength);
	properties->cnr.len = 1;
	properties->cnr.stat[0].scale = locked ? FE_SCALE_RELATIVE :
						 FE_SCALE_NOT_AVAILABLE;
	properties->cnr.stat[0].uvalue =
		locked ? lme2510_dtmb_percent(quality) : 0;
}

static int lme2510_dtmb_frontend_init(struct dvb_frontend *frontend)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;
	int ret = 0;

	ret = mutex_lock_interruptible(&state->tune_lock);
	if (ret)
		return ret;
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

static int lme2510_dtmb_set_frontend(struct dvb_frontend *frontend)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;
	u32 frequency = frontend->dtv_property_cache.frequency;
	bool restart_stream;
	int stop_ret;
	int ret;

	cancel_delayed_work_sync(&state->stream_work);
	ret = mutex_lock_interruptible(&state->tune_lock);
	if (ret)
		return ret;
	restart_stream = state->stream_requested;
	if (restart_stream) {
		stop_ret = lme2510_dtmb_stop_stream_locked(state);
		if (stop_ret)
			dev_warn(&state->d->udev->dev,
				 "stream stop failed before retune: %d\n",
				 stop_ret);
	}
	ret = lme2510_dtmb_tune(state, frequency);
	if (!ret && restart_stream)
		ret = lme2510_dtmb_start_stream_locked(frontend);

	mutex_unlock(&state->tune_lock);
	if (!ret && restart_stream)
		mod_delayed_work(system_wq, &state->stream_work,
				 msecs_to_jiffies(LME2510_DTMB_STREAM_VALIDATE_MS));
	return ret;
}

static int lme2510_dtmb_read_status(struct dvb_frontend *frontend,
				    enum fe_status *status)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;
	struct dtv_frontend_properties *properties =
		&frontend->dtv_property_cache;
	struct lme2510_dtmb_status stream_status;
	u8 lock_reg = LME2510_DTMB_REG_LOCK;
	u8 signal = state->tuned_strength;
	u8 lock = 0;
	bool locked;
	int ret = 0;

	lme2510_dtmb_status_snapshot(state, &stream_status);
	if (READ_ONCE(state->stream_requested)) {
		locked = READ_ONCE(state->tuned) &&
			 atomic_read(&state->streaming);
		if (!lme2510_dtmb_stream_is_live(state) &&
		    stream_status.valid && stream_status.lock)
			locked &= !(stream_status.snr == 0xff &&
				    stream_status.extra == 0xff);
	} else {
		ret = mutex_lock_interruptible(&state->tune_lock);
		if (ret)
			return ret;
		ret = lme2510_dtmb_demod_read(state, LME2510_DTMB_REG_SIGNAL,
					      &signal);
		if (!ret)
			ret = lme2510_dtmb_demod_read(state, lock_reg, &lock);
		mutex_unlock(&state->tune_lock);
		if (ret)
			return ret;
		locked = (signal & BIT(7)) && (lock & 0x03) == 0x01;
	}

	*status = 0;
	if (signal & 0x7f)
		*status |= FE_HAS_SIGNAL;
	if (signal & BIT(7))
		*status |= FE_HAS_CARRIER;
	if (locked)
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
			  FE_HAS_SYNC | FE_HAS_LOCK;

	if (locked && state->detected_parameters)
		lme2510_dtmb_decode_parameters(properties,
					       state->detected_parameters);
	lme2510_dtmb_set_statistics(state, properties, &stream_status, signal,
				    locked);
	return 0;
}

static int lme2510_dtmb_get_frontend(struct dvb_frontend *frontend,
				     struct dtv_frontend_properties *properties)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;

	if (state->detected_parameters)
		lme2510_dtmb_decode_parameters(properties,
					       state->detected_parameters);
	return 0;
}

static int
lme2510_dtmb_get_tune_settings(struct dvb_frontend *frontend,
			       struct dvb_frontend_tune_settings *settings)
{
	settings->min_delay_ms = 240;
	settings->step_size = 0;
	settings->max_drift = 0;
	return 0;
}

static int lme2510_dtmb_read_signal_strength(struct dvb_frontend *frontend,
					     u16 *strength)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;
	struct lme2510_dtmb_status status;
	u8 signal = state->tuned_strength;
	u8 percent;
	int ret;

	lme2510_dtmb_status_snapshot(state, &status);
	if (status.valid) {
		percent = lme2510_dtmb_stream_strength(&status, state->tuned);
		*strength = lme2510_dtmb_percent(percent);
		return 0;
	}
	if (!atomic_read(&state->streaming)) {
		ret = mutex_lock_interruptible(&state->tune_lock);
		if (ret)
			return ret;
		ret = lme2510_dtmb_demod_read(state, LME2510_DTMB_REG_SIGNAL,
					      &signal);
		mutex_unlock(&state->tune_lock);
		if (ret)
			return ret;
	}
	*strength = (signal & 0x7f) << 9;
	return 0;
}

static int lme2510_dtmb_read_snr(struct dvb_frontend *frontend, u16 *snr)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;
	struct lme2510_dtmb_status status;
	u8 quality;

	lme2510_dtmb_status_snapshot(state, &status);
	if (!state->tuned) {
		*snr = 0;
		return 0;
	}
	if (status.valid) {
		quality = lme2510_dtmb_stream_quality(status.snr, true);
		*snr = lme2510_dtmb_percent(quality);
	} else {
		*snr = (state->tuned_strength & 0x7f) << 9;
	}
	return 0;
}

static int lme2510_dtmb_read_ber(struct dvb_frontend *frontend, u32 *ber)
{
	*ber = 0;
	return 0;
}

static int lme2510_dtmb_read_ucblocks(struct dvb_frontend *frontend,
				      u32 *ucblocks)
{
	*ucblocks = 0;
	return 0;
}

static void lme2510_dtmb_parse_status(struct lme2510_dtmb_state *state,
				      const u8 *data, size_t length)
{
	if (length < 7 || data[0] != 0xbb)
		return;
	if (state->status_settle_packets) {
		state->status_settle_packets--;
		return;
	}

	state->stream_status.valid = true;
	state->stream_status.lock = data[2];
	state->stream_status.signal_low = data[3];
	state->stream_status.snr = data[4];
	state->stream_status.signal_high = data[5];
	state->stream_status.extra = data[6];
}

static void lme2510_dtmb_status_complete(struct urb *urb)
{
	struct lme2510_dtmb_state *state = urb->context;
	unsigned long flags;
	int ret;

	if (!state)
		return;
	if (!urb->status && urb->actual_length >= 7) {
		spin_lock_irqsave(&state->stream_lock, flags);
		lme2510_dtmb_parse_status(state, state->status_buffer,
					  urb->actual_length);
		spin_unlock_irqrestore(&state->stream_lock, flags);
	}
	if (!atomic_read(&state->streaming))
		return;

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret && ret != -ENODEV && ret != -ESHUTDOWN)
		dev_warn_ratelimited(&state->d->udev->dev,
				     "status URB resubmit failed: %d\n", ret);
}

static int lme2510_dtmb_start_status(struct lme2510_dtmb_state *state)
{
	struct usb_host_interface *alt = state->d->intf->cur_altsetting;
	struct usb_endpoint_descriptor *endpoint = NULL;
	int i;

	if (!state->status_urb)
		return -ENOMEM;

	for (i = 0; i < alt->desc.bNumEndpoints; i++) {
		if (alt->endpoint[i].desc.bEndpointAddress ==
		    LME2510_DTMB_EP_STATUS_IN) {
			endpoint = &alt->endpoint[i].desc;
			break;
		}
	}
	if (!endpoint)
		return -ENODEV;

	if (usb_endpoint_xfer_int(endpoint)) {
		usb_fill_int_urb(state->status_urb, state->d->udev,
				 usb_rcvintpipe(state->d->udev,
						LME2510_DTMB_EP_STATUS_IN),
				 state->status_buffer,
				 sizeof(state->status_buffer),
				 lme2510_dtmb_status_complete, state,
				 endpoint->bInterval);
	} else if (usb_endpoint_xfer_bulk(endpoint)) {
		usb_fill_bulk_urb(state->status_urb, state->d->udev,
				  usb_rcvbulkpipe(state->d->udev,
						  LME2510_DTMB_EP_STATUS_IN),
				  state->status_buffer,
				  sizeof(state->status_buffer),
				  lme2510_dtmb_status_complete, state);
	} else {
		return -EINVAL;
	}

	return usb_submit_urb(state->status_urb, GFP_KERNEL);
}

static void lme2510_dtmb_reset_stream(struct lme2510_dtmb_state *state)
{
	unsigned long flags;

	spin_lock_irqsave(&state->stream_lock, flags);
	state->stream_carry = 0;
	memset(&state->stream_status, 0, sizeof(state->stream_status));
	state->stream_bytes = 0;
	state->stream_packets = 0;
	state->stream_tei = 0;
	state->stream_cc_errors = 0;
	state->stream_window_bytes = 0;
	state->stream_window_packets = 0;
	state->stream_window_tei = 0;
	state->stream_window_cc_errors = 0;
	state->stream_started = 0;
	state->stream_last_packet = 0;
	state->stream_validated = false;
	state->stream_force_restart = false;
	bitmap_zero(state->stream_cc_valid, LME2510_DTMB_TS_PID_COUNT);
	spin_unlock_irqrestore(&state->stream_lock, flags);
}

static void lme2510_dtmb_count_packet(struct lme2510_dtmb_state *state,
				      const u8 *packet)
{
	u16 pid = ((packet[1] & 0x1f) << 8) | packet[2];
	u8 adaptation = (packet[3] >> 4) & 0x03;
	u8 counter = packet[3] & 0x0f;
	bool discontinuity = false;

	if (packet[1] & BIT(7))
		state->stream_tei++;
	if (pid == 0x1fff || !adaptation)
		return;
	if ((adaptation & 0x02) && packet[4] &&
	    packet[4] < LME2510_DTMB_TS_PACKET_SIZE - 5)
		discontinuity = packet[5] & BIT(7);
	if (discontinuity)
		clear_bit(pid, state->stream_cc_valid);
	if (!(adaptation & 0x01))
		return;
	if (test_bit(pid, state->stream_cc_valid) &&
	    counter != ((state->stream_cc[pid] + 1) & 0x0f))
		state->stream_cc_errors++;
	state->stream_cc[pid] = counter;
	set_bit(pid, state->stream_cc_valid);
}

static size_t lme2510_dtmb_find_sync(const u8 *buffer, size_t length,
				     size_t offset)
{
	size_t packet;

	while (offset + (LME2510_DTMB_TS_SYNC_PACKETS - 1) *
				LME2510_DTMB_TS_PACKET_SIZE <
	       length) {
		for (packet = 0; packet < LME2510_DTMB_TS_SYNC_PACKETS;
		     packet++)
			if (buffer[offset +
				   packet * LME2510_DTMB_TS_PACKET_SIZE] !=
			    0x47)
				break;
		if (packet == LME2510_DTMB_TS_SYNC_PACKETS)
			return offset;
		offset++;
	}
	return length;
}

static void lme2510_dtmb_stream_complete(struct usb_data_stream *stream,
					 u8 *data, size_t length)
{
	struct dvb_usb_adapter *adapter = stream->user_priv;
	struct lme2510_dtmb_state *state = adap_to_priv(adapter);
	unsigned long flags;
	u64 cc_before;
	u64 tei_before;
	size_t total;
	size_t scan = 0;
	size_t output = 0;
	size_t sync;
	size_t keep;
	size_t i;
	bool recover = false;

	if (!atomic_read(&state->streaming) || !length ||
	    length > LME2510_DTMB_TS_BUFFER_SIZE)
		return;

	spin_lock_irqsave(&state->stream_lock, flags);
	tei_before = state->stream_tei;
	cc_before = state->stream_cc_errors;
	state->stream_bytes += length;
	total = state->stream_carry + length;
	memcpy(state->stream_buffer + state->stream_carry, data, length);

	for (;;) {
		sync = lme2510_dtmb_find_sync(state->stream_buffer, total,
					      scan);
		if (sync == total)
			break;
		if (sync > scan) {
			lme2510_dtmb_parse_status(state,
						  state->stream_buffer + scan,
						  sync - scan);
		}

		while (sync + LME2510_DTMB_TS_PACKET_SIZE < total &&
		       state->stream_buffer[sync] == 0x47 &&
		       state->stream_buffer[sync + LME2510_DTMB_TS_PACKET_SIZE] ==
			       0x47) {
			memmove(state->stream_buffer + output,
				state->stream_buffer + sync,
				LME2510_DTMB_TS_PACKET_SIZE);
			output += LME2510_DTMB_TS_PACKET_SIZE;
			sync += LME2510_DTMB_TS_PACKET_SIZE;
		}
		scan = sync;
	}

	state->stream_packets += output / LME2510_DTMB_TS_PACKET_SIZE;
	if (output)
		state->stream_last_packet = jiffies;
	for (i = 0; i < output; i += LME2510_DTMB_TS_PACKET_SIZE)
		lme2510_dtmb_count_packet(state, state->stream_buffer + i);
	state->stream_window_bytes += length;
	state->stream_window_packets += output / LME2510_DTMB_TS_PACKET_SIZE;
	state->stream_window_tei += state->stream_tei - tei_before;
	state->stream_window_cc_errors += state->stream_cc_errors - cc_before;
	if (state->stream_validated &&
	    state->stream_window_bytes >= LME2510_DTMB_STREAM_WINDOW_BYTES) {
		if (state->stream_window_packets * LME2510_DTMB_TS_PACKET_SIZE *
				    100 <
			    state->stream_window_bytes *
				    LME2510_DTMB_STREAM_VALID_PERCENT ||
		    state->stream_window_tei * 100 >
			    state->stream_window_packets *
				    LME2510_DTMB_STREAM_MAX_TEI_PERCENT ||
		    state->stream_window_cc_errors * 100 >
			    state->stream_window_packets *
				    LME2510_DTMB_STREAM_MAX_CC_PERCENT) {
			state->stream_validated = false;
			state->stream_force_restart = true;
			recover = true;
		}
		state->stream_window_bytes = 0;
		state->stream_window_packets = 0;
		state->stream_window_tei = 0;
		state->stream_window_cc_errors = 0;
	}
	if (output && state->stream_validated)
		dvb_dmx_swfilter_packets(&adapter->demux, state->stream_buffer,
					 output / LME2510_DTMB_TS_PACKET_SIZE);

	keep = min_t(size_t, total - scan, LME2510_DTMB_TS_CARRY_SIZE);
	if (keep)
		memmove(state->stream_buffer,
			state->stream_buffer + total - keep, keep);
	state->stream_carry = keep;
	spin_unlock_irqrestore(&state->stream_lock, flags);

	if (recover && READ_ONCE(state->stream_requested))
		mod_delayed_work(system_wq, &state->stream_work, 0);
}

static int lme2510_dtmb_stream_command(struct lme2510_dtmb_state *state,
				       const u8 *command, size_t length,
				       size_t response_length)
{
	size_t actual = 0;
	int ret;

	ret = lme2510_dtmb_exchange(state, command, length, NULL,
				    response_length, &actual);
	if (ret)
		return ret;
	return actual ? 0 : -EREMOTEIO;
}

static bool
lme2510_dtmb_stream_is_valid(struct lme2510_dtmb_state *state,
			     struct lme2510_dtmb_stream_sample *sample)
{
	unsigned long flags;

	spin_lock_irqsave(&state->stream_lock, flags);
	sample->bytes = state->stream_bytes;
	sample->packets = state->stream_packets;
	sample->tei = state->stream_tei;
	sample->cc = state->stream_cc_errors;
	spin_unlock_irqrestore(&state->stream_lock, flags);

	if (!sample->bytes || sample->packets < LME2510_DTMB_STREAM_MIN_PACKETS)
		return false;
	if (sample->packets * LME2510_DTMB_TS_PACKET_SIZE * 100 <
	    sample->bytes * LME2510_DTMB_STREAM_VALID_PERCENT)
		return false;
	if (sample->tei * 100 >
	    sample->packets * LME2510_DTMB_STREAM_MAX_TEI_PERCENT)
		return false;
	if (sample->cc * 100 >
	    sample->packets * LME2510_DTMB_STREAM_MAX_CC_PERCENT)
		return false;

	return true;
}

static int lme2510_dtmb_start_transport(struct lme2510_dtmb_state *state)
{
	static const u8 all_pids[] = { 0x03, 0x06, 0x00, 0xff,
				       0x01, 0x1f, 0x20, 0x81 };
	static const u8 start[] = { 0x06, 0x00 };
	int ret;

	ret = lme2510_dtmb_stream_command(state, all_pids, sizeof(all_pids), 1);
	if (ret)
		return ret;
	ret = lme2510_dtmb_stream_command(state, all_pids, sizeof(all_pids), 1);
	if (ret)
		return ret;

	return lme2510_dtmb_stream_command(state, start, sizeof(start),
					   LME2510_DTMB_COMMAND_BUFFER_SIZE);
}

static int lme2510_dtmb_stop_transport(struct lme2510_dtmb_state *state)
{
	static const u8 stop[] = { 0x03, 0x02, 0x20, 0xa0 };

	return lme2510_dtmb_stream_command(state, stop, sizeof(stop), 1);
}

static void lme2510_dtmb_mark_stream_valid(struct lme2510_dtmb_state *state)
{
	unsigned long flags;

	spin_lock_irqsave(&state->stream_lock, flags);
	state->stream_started = jiffies;
	state->stream_validated = true;
	state->stream_force_restart = false;
	state->stream_window_bytes = 0;
	state->stream_window_packets = 0;
	state->stream_window_tei = 0;
	state->stream_window_cc_errors = 0;
	spin_unlock_irqrestore(&state->stream_lock, flags);
}

static int lme2510_dtmb_stop_stream_locked(struct lme2510_dtmb_state *state)
{
	int ret;

	atomic_set(&state->streaming, 0);
	if (state->status_urb)
		usb_kill_urb(state->status_urb);
	ret = lme2510_dtmb_stop_transport(state);
	lme2510_dtmb_reset_stream(state);
	if (ret == -ENODEV || ret == -ESHUTDOWN)
		ret = 0;

	return ret;
}

static int lme2510_dtmb_start_stream_locked(struct dvb_frontend *frontend)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;
	struct device *dev = &state->d->udev->dev;
	int stop_ret;
	int ret;

	atomic_set(&state->streaming, 0);
	lme2510_dtmb_reset_stream(state);
	ret = lme2510_dtmb_start_transport(state);
	if (ret)
		goto err_stop_streaming;
	atomic_set(&state->streaming, 1);
	ret = lme2510_dtmb_start_status(state);
	if (ret)
		dev_warn(dev,
			 "status endpoint failed (%d); validating TS only\n",
			 ret);
	return 0;

err_stop_streaming:
	atomic_set(&state->streaming, 0);
	stop_ret = lme2510_dtmb_stop_transport(state);
	if (!ret)
		ret = stop_ret;
	return ret;
}

static void lme2510_dtmb_stream_work(struct work_struct *work)
{
	struct delayed_work *stream_work = to_delayed_work(work);
	struct lme2510_dtmb_state *state;
	struct lme2510_dtmb_stream_sample sample;
	struct device *dev;
	unsigned long delay = msecs_to_jiffies(LME2510_DTMB_STREAM_VALIDATE_MS);
	bool force_restart;
	bool was_validated;
	bool requeue = false;
	int ret;

	state = container_of(stream_work, struct lme2510_dtmb_state,
			     stream_work);
	dev = &state->d->udev->dev;
	mutex_lock(&state->tune_lock);
	if (!READ_ONCE(state->stream_requested))
		goto unlock;

	was_validated = READ_ONCE(state->stream_validated);
	force_restart = READ_ONCE(state->stream_force_restart);
	if (atomic_read(&state->streaming) && was_validated) {
		if (lme2510_dtmb_stream_is_live(state)) {
			delay = msecs_to_jiffies(LME2510_DTMB_STREAM_MONITOR_MS);
			requeue = true;
			goto unlock;
		}
		force_restart = true;
	}

	if (atomic_read(&state->streaming) && !force_restart &&
	    lme2510_dtmb_stream_is_valid(state, &sample)) {
		lme2510_dtmb_mark_stream_valid(state);
		delay = msecs_to_jiffies(LME2510_DTMB_STREAM_MONITOR_MS);
		requeue = true;
		goto unlock;
	}

	if (atomic_read(&state->streaming)) {
		if (force_restart)
			lme2510_dtmb_stream_is_valid(state, &sample);
		ret = lme2510_dtmb_stop_stream_locked(state);
		if (ret)
			dev_warn_ratelimited(dev,
					     "TS recovery stop failed: %d\n", ret);
	}

	ret = lme2510_dtmb_start_stream_locked(&state->frontend);
	if (ret)
		dev_warn_ratelimited(dev, "TS recovery start failed: %d\n",
				     ret);
	requeue = true;

unlock:
	mutex_unlock(&state->tune_lock);
	if (requeue && READ_ONCE(state->stream_requested))
		mod_delayed_work(system_wq, &state->stream_work, delay);
}

static int lme2510_dtmb_streaming_ctrl(struct dvb_frontend *frontend,
				       int enable)
{
	struct lme2510_dtmb_state *state = frontend->demodulator_priv;
	u32 frequency = READ_ONCE(frontend->dtv_property_cache.frequency);
	u32 frequency_khz = frequency / 1000;
	int ret;

	cancel_delayed_work_sync(&state->stream_work);
	ret = mutex_lock_interruptible(&state->tune_lock);
	if (ret)
		return ret;

	if (!enable) {
		WRITE_ONCE(state->stream_requested, false);
		ret = lme2510_dtmb_stop_stream_locked(state);
		goto unlock;
	}

	WRITE_ONCE(state->stream_requested, true);
	frequency_khz = frequency_khz / 1000 * 1000;
	if (!state->tuned || state->tuned_frequency != frequency_khz * 1000) {
		ret = lme2510_dtmb_tune(state, frequency);
		if (ret)
			goto unlock;
	}
	ret = lme2510_dtmb_start_stream_locked(frontend);

unlock:
	mutex_unlock(&state->tune_lock);
	if (enable && !ret)
		mod_delayed_work(system_wq, &state->stream_work,
				 msecs_to_jiffies(LME2510_DTMB_STREAM_VALIDATE_MS));
	return ret;
}

static int lme2510_dtmb_frontend_detach(struct dvb_usb_adapter *adapter)
{
	struct lme2510_dtmb_state *state = adap_to_priv(adapter);

	WRITE_ONCE(state->stream_requested, false);
	cancel_delayed_work_sync(&state->stream_work);
	atomic_set(&state->streaming, 0);
	if (state->status_urb) {
		usb_kill_urb(state->status_urb);
		usb_free_urb(state->status_urb);
		state->status_urb = NULL;
	}

	return 0;
}

static const struct dvb_frontend_ops lme2510_dtmb_frontend_ops = {
	/* Keep the terrestrial ABI expected by existing DVB applications. */
	.delsys = { SYS_DVBT },
	.info = {
		.name = "LME2510C LGS8GL5 DTMB",
		.frequency_min_hz = 450 * MHz,
		.frequency_max_hz = 880 * MHz,
		.frequency_stepsize_hz = 10000,
		.caps = FE_CAN_FEC_AUTO | FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO | FE_CAN_BANDWIDTH_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO,
	},
	.init = lme2510_dtmb_frontend_init,
	.set_frontend = lme2510_dtmb_set_frontend,
	.get_frontend = lme2510_dtmb_get_frontend,
	.get_tune_settings = lme2510_dtmb_get_tune_settings,
	.read_status = lme2510_dtmb_read_status,
	.read_signal_strength = lme2510_dtmb_read_signal_strength,
	.read_snr = lme2510_dtmb_read_snr,
	.read_ber = lme2510_dtmb_read_ber,
	.read_ucblocks = lme2510_dtmb_read_ucblocks,
};

static int lme2510_dtmb_frontend_attach(struct dvb_usb_adapter *adapter)
{
	struct lme2510_dtmb_state *state = adap_to_priv(adapter);

	state->d = adap_to_d(adapter);
	mutex_init(&state->protocol_lock);
	mutex_init(&state->tune_lock);
	spin_lock_init(&state->stream_lock);
	atomic_set(&state->streaming, 0);
	INIT_DELAYED_WORK(&state->stream_work, lme2510_dtmb_stream_work);

	state->frontend.ops = lme2510_dtmb_frontend_ops;
	state->frontend.demodulator_priv = state;
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

static bool lme2510_dtmb_runtime_ready(struct usb_interface *interface)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	static const u8 request[] = { 0x85, 0x02, LME2510_DTMB_DEMOD_SELECTOR,
				      0x00, 0x00 };
	u8 *buffer;
	int actual = 0;
	bool ready = false;
	int ret;

	buffer = kmalloc(LME2510_DTMB_COMMAND_BUFFER_SIZE, GFP_KERNEL);
	if (!buffer)
		return false;
	memcpy(buffer, request, sizeof(request));
	ret = usb_bulk_msg(udev,
			   usb_sndbulkpipe(udev, LME2510_DTMB_EP_COMMAND_OUT),
			   buffer, sizeof(request), &actual, 500);
	if (ret || actual != sizeof(request))
		goto free_buffer;

	memset(buffer, 0, 2);
	ret = usb_bulk_msg(udev,
			   usb_rcvbulkpipe(udev, LME2510_DTMB_EP_COMMAND_IN),
			   buffer, 2, &actual, 500);
	if (!ret && actual == 2 && lme2510_dtmb_valid_ack(buffer[0]))
		ready = true;

free_buffer:
	kfree(buffer);
	return ready;
}

static int lme2510_dtmb_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	u16 pid = le16_to_cpu(udev->descriptor.idProduct);
	bool cold = pid == LME2510_DTMB_COLD_PID;
	int ret;

	if (!cold) {
		ret = lme2510_dtmb_prepare_runtime(interface);
		if (ret)
			return ret;
		if (lme2510_dtmb_runtime_ready(interface))
			return dvb_usbv2_probe(interface, id);
	}

	ret = lme2510_dtmb_download_firmware(udev);
	if (ret)
		return ret;

	/* The bootloader disconnects and returns with one of the runtime IDs. */
	return -ENODEV;
}

static struct usb_driver lme2510_dtmb_driver = {
	.name = KBUILD_MODNAME,
	.id_table = lme2510_dtmb_id_table,
	.probe = lme2510_dtmb_probe,
	.disconnect = dvb_usbv2_disconnect,
	.suspend = dvb_usbv2_suspend,
	.resume = dvb_usbv2_resume,
	.reset_resume = dvb_usbv2_reset_resume,
	.no_dynamic_id = 1,
	.soft_unbind = 1,
};
module_usb_driver(lme2510_dtmb_driver);

MODULE_AUTHOR("wwek <licoolgo@gmail.com>");
MODULE_DESCRIPTION("LME2510C LGS8GL5 MAX2165 DTMB USB receiver driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(LME2510_DTMB_FW_ROM);
MODULE_FIRMWARE(LME2510_DTMB_FW_MAIN);
MODULE_FIRMWARE(LME2510_DTMB_FW_22D0);
