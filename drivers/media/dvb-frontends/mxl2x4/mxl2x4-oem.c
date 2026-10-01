// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/slab.h>

#include "MxL_HRCLS_Common.h"
#include "MxL_HRCLS_CommonApi.h"
#include "mxl214-priv.h"

static struct mxl214_state *mxl2x4_state(UINT8 dev_id)
{
	if (dev_id >= MXL_HRCLS_MAX_NUM_DEVICES)
		return NULL;
	return MxL_HRCLS_OEM_DataPtr[dev_id];
}

static int mxl2x4_i2c_write(struct mxl214_state *state,
			     const u8 *prefix, size_t prefix_size,
			     const u8 *data, size_t data_size)
{
	struct i2c_msg message;
	u8 *buffer;
	int ret;

	if (!prefix_size || data_size > MXL_HRCLS_OEM_MAX_BLOCK_WRITE_LENGTH)
		return -EINVAL;

	buffer = kmalloc(prefix_size + data_size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	memcpy(buffer, prefix, prefix_size);
	if (data_size)
		memcpy(buffer + prefix_size, data, data_size);

	message.addr = state->client->addr;
	message.flags = 0;
	message.len = prefix_size + data_size;
	message.buf = buffer;
	ret = i2c_transfer(state->client->adapter, &message, 1);
	kfree(buffer);
	if (ret != 1)
		dev_err_ratelimited(&state->client->dev,
				    "I2C write failed at register 0x%02x%02x: %d\n",
				    prefix[0], prefix_size > 1 ? prefix[1] : 0,
				    ret < 0 ? ret : -EIO);

	return ret == 1 ? 0 : ret < 0 ? ret : -EIO;
}

static int mxl2x4_i2c_read_xfer(struct mxl214_state *state,
				 const u8 *command, size_t command_size,
				 u8 *data, size_t data_size, bool report_error)
{
	struct i2c_msg messages[2] = {
		{
			.addr = state->client->addr,
			.flags = 0,
			.len = command_size,
			.buf = (u8 *)command,
		}, {
			.addr = state->client->addr,
			.flags = I2C_M_RD,
			.len = data_size,
			.buf = data,
		},
	};
	int ret;

	if (!command_size || !data_size ||
	    data_size > MXL_HRCLS_OEM_MAX_BLOCK_READ_LENGTH)
		return -EINVAL;

	ret = i2c_transfer(state->client->adapter, messages, 2);
	if (ret != 2 && report_error)
		dev_err_ratelimited(&state->client->dev,
				    "I2C read command %*ph failed: %d\n",
				    (int)command_size, command,
				    ret < 0 ? ret : -EIO);
	return ret == 2 ? 0 : ret < 0 ? ret : -EIO;
}

static int mxl2x4_i2c_read(struct mxl214_state *state,
			    const u8 *command, size_t command_size,
			    u8 *data, size_t data_size)
{
	return mxl2x4_i2c_read_xfer(state, command, command_size, data,
				       data_size, true);
}

static int mxl214_wait_ready(struct mxl214_state *state)
{
	static const unsigned int delays_ms[] = { 50, 100, 350 };
	u8 command[] = { 0xff, 0xfb, 0x60, 0xe8 };
	u8 data[2];
	unsigned int elapsed = 0;
	unsigned int i;
	int ret = -ENXIO;

	for (i = 0; i < ARRAY_SIZE(delays_ms); i++) {
		msleep(delays_ms[i]);
		elapsed += delays_ms[i];
		ret = mxl2x4_i2c_read_xfer(state, command, sizeof(command),
					     data, sizeof(data), false);
		if (!ret) {
			if (i)
				dev_info(&state->client->dev,
					 "I2C became ready %u ms after reset\n",
					 elapsed);
			return 0;
		}
	}

	dev_err(&state->client->dev,
		"I2C did not become ready within %u ms after reset: %d\n",
		elapsed, ret);
	return ret;
}

MXL_STATUS_E MxLWare_HRCLS_OEM_Reset(UINT8 dev_id)
{
	struct mxl214_state *state = mxl2x4_state(dev_id);
	int asserted;
	int deasserted;

	if (!state || !state->reset_gpio)
		return MXL_FAILURE;

	gpiod_set_value_cansleep(state->reset_gpio, 1);
	/* MxL214 needs a real reset interval before its I2C registers respond. */
	msleep(100);
	asserted = gpiod_get_raw_value_cansleep(state->reset_gpio);
	gpiod_set_value_cansleep(state->reset_gpio, 0);
	deasserted = gpiod_get_raw_value_cansleep(state->reset_gpio);
	if (asserted != 0 || deasserted != 1) {
		dev_err(&state->client->dev,
			"reset GPIO raw readback failed: asserted=%d deasserted=%d\n",
			asserted, deasserted);
		return MXL_FAILURE;
	}
	if (mxl214_wait_ready(state))
		return MXL_FAILURE;
	dev_info_ratelimited(&state->client->dev,
			     "reset pulse verified (raw 0 -> 1)\n");
	return MXL_SUCCESS;
}

MXL_STATUS_E MxLWare_HRCLS_OEM_WriteRegister(UINT8 dev_id,
					     UINT16 reg, UINT16 value)
{
	struct mxl214_state *state = mxl2x4_state(dev_id);
	u8 address[2] = { reg >> 8, reg };
	u8 data[2] = { value >> 8, value };

	if (!state)
		return MXL_FAILURE;
	return mxl2x4_i2c_write(state, address, sizeof(address), data,
				 sizeof(data)) ? MXL_FAILURE : MXL_SUCCESS;
}

MXL_STATUS_E MxLWare_HRCLS_OEM_ReadRegister(UINT8 dev_id,
					    UINT16 reg, UINT16 *value)
{
	struct mxl214_state *state = mxl2x4_state(dev_id);
	u8 command[4] = { 0xff, 0xfb, reg >> 8, reg };
	u8 data[2];

	if (!state || !value)
		return MXL_FAILURE;
	if (mxl2x4_i2c_read(state, command, sizeof(command), data,
			       sizeof(data))) {
		*value = 0;
		return MXL_FAILURE;
	}

	*value = data[0] << 8 | data[1];
	return MXL_SUCCESS;
}

MXL_STATUS_E MxLWare_HRCLS_OEM_WriteBlock(UINT8 dev_id, UINT16 reg,
					  UINT16 size, UINT8 *buffer)
{
	struct mxl214_state *state = mxl2x4_state(dev_id);
	u8 address[2] = { reg >> 8, reg };

	if (!state || (!buffer && size))
		return MXL_FAILURE;
	return mxl2x4_i2c_write(state, address, sizeof(address), buffer,
				 size) ? MXL_FAILURE : MXL_SUCCESS;
}

MXL_STATUS_E MxLWare_HRCLS_OEM_ReadBlock(UINT8 dev_id, UINT16 reg,
					 UINT16 size, UINT8 *buffer)
{
	struct mxl214_state *state = mxl2x4_state(dev_id);
	u8 command[4] = { 0xff, 0xfd, reg >> 8, reg };

	if (!state || !buffer)
		return MXL_FAILURE;
	return mxl2x4_i2c_read(state, command, sizeof(command), buffer,
				size) ? MXL_FAILURE : MXL_SUCCESS;
}

MXL_STATUS_E MxLWare_HRCLS_OEM_ReadBlockExt(UINT8 dev_id, UINT16 command_id,
					    UINT16 offset, UINT16 size,
					    UINT8 *buffer)
{
	struct mxl214_state *state = mxl2x4_state(dev_id);
	u8 command[6] = {
		0xff, 0xfd, command_id >> 8, command_id,
		offset >> 8, offset,
	};
	struct i2c_msg message;
	int ret;

	if (!state || !buffer || !size ||
	    size > MXL_HRCLS_OEM_MAX_BLOCK_READ_LENGTH)
		return MXL_FAILURE;
	if (mxl2x4_i2c_write(state, command, sizeof(command), NULL, 0))
		return MXL_FAILURE;

	message.addr = state->client->addr;
	message.flags = I2C_M_RD;
	message.len = size;
	message.buf = buffer;
	ret = i2c_transfer(state->client->adapter, &message, 1);
	return ret == 1 ? MXL_SUCCESS : MXL_FAILURE;
}

MXL_STATUS_E MxLWare_HRCLS_OEM_LoadNVRAMFile(UINT8 dev_id, UINT8 *buffer,
					     UINT32 size)
{
	struct mxl214_state *state = mxl2x4_state(dev_id);
	const struct firmware *firmware;
	int ret;

	if (!state || !buffer || !size)
		return MXL_FAILURE;

	ret = request_firmware(&firmware, MXL214_NVRAM,
			       &state->client->dev);
	if (ret) {
		dev_warn(&state->client->dev, "calibration %s unavailable: %d\n",
			 MXL214_NVRAM, ret);
		return MXL_FAILURE;
	}

	if (firmware->size < size) {
		dev_err(&state->client->dev,
			"calibration %s is too short: %zu < %u\n",
			MXL214_NVRAM, firmware->size, size);
		ret = -EINVAL;
	} else {
		memcpy(buffer, firmware->data, size);
		ret = 0;
	}
	release_firmware(firmware);
	return ret ? MXL_FAILURE : MXL_SUCCESS;
}

MXL_STATUS_E MxLWare_HRCLS_OEM_SaveNVRAMFile(UINT8 dev_id, UINT8 *buffer,
					     UINT32 size)
{
	(void)dev_id;
	(void)buffer;
	(void)size;
	return MXL_NOT_SUPPORTED;
}

void MxLWare_HRCLS_OEM_DelayUsec(UINT32 usec)
{
	if (usec <= 10)
		udelay(usec);
	else
		usleep_range(usec, usec + max_t(UINT32, 10, usec / 10));
}

void MxLWare_HRCLS_OEM_GetCurrTimeInUsec(UINT64 *usec)
{
	if (usec)
		*usec = div_u64(ktime_get_ns(), NSEC_PER_USEC);
}
