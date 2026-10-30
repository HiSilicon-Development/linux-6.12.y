// SPDX-License-Identifier: GPL-2.0-only
/* Shared EM28xx vendor-control, I2C and EEPROM transport. */
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb.h>

#include "em28xx-dtmb.h"

int em28xx_ctrl(struct em28xx_state *st, bool read, u8 request,
			u16 value, u16 index, u8 *buf, u16 len)
{
	u8 *transfer = st->control_buf;
	int ret;

	/* Callers serialize control traffic and disconnect with st->lock. */
	if (st->disconnected)
		return -ENODEV;

	if (!len || len > sizeof(st->control_buf))
		return -EINVAL;
	if (!read)
		memcpy(transfer, buf, len);
	ret = usb_control_msg(st->d->udev,
		read ? usb_rcvctrlpipe(st->d->udev, 0) : usb_sndctrlpipe(st->d->udev, 0),
		request, USB_TYPE_VENDOR | USB_RECIP_DEVICE |
		(read ? USB_DIR_IN : USB_DIR_OUT), value, index, transfer, len, 1000);
	if (read && ret == len)
		memcpy(buf, transfer, len);
	return ret == len ? 0 : (ret < 0 ? ret : -EIO);
}

int em28xx_reg_read(struct em28xx_state *st, u16 reg, u8 *value)
{
	return em28xx_ctrl(st, true, EM28XX_REQ_BRIDGE, 0, reg, value, 1);
}

int em28xx_reg_write(struct em28xx_state *st, u16 reg, u8 value)
{
	return em28xx_ctrl(st, false, EM28XX_REQ_BRIDGE, 0, reg, &value, 1);
}

int em28xx_reg_update_bits(struct em28xx_state *st, u16 reg, u8 mask, u8 value)
{
	u8 old_value;
	int ret = em28xx_reg_read(st, reg, &old_value);

	if (ret)
		return ret;
	return em28xx_reg_write(st, reg, (old_value & ~mask) | (value & mask));
}

static int em28xx_i2c_wait(struct em28xx_state *st)
{
	u8 status;
	int ret, i;

	for (i = 0; i < 8; i++) {
		ret = em28xx_reg_read(st, EM28XX_REG_I2C_STATUS, &status);
		if (ret)
			return ret;
		if (status == 0x00)
			return 0;
		if (status == 0x10)
			return -ENXIO;
		msleep(5);
	}
	return -ETIMEDOUT;
}

static int em28xx_i2c_bus_b_wait(struct em28xx_state *st)
{
	u8 status;
	int ret;

	ret = em28xx_ctrl(st, true, EM28XX_REQ_I2C_BUS_B_STATUS, 0, 0,
			  &status, sizeof(status));
	if (ret)
		return ret;
	return status ? -ENXIO : 0;
}

int em28xx_i2c_write(struct em28xx_state *st, u8 addr, const u8 *buf,
			     u8 len, bool stop)
{
	int ret;

	if (!len || len > 64)
		return -EINVAL;
	if (st->use_bus_b) {
		ret = em28xx_ctrl(st, false, EM28XX_REQ_I2C_BUS_B, 0, addr << 1,
				   (u8 *)buf, len);
		if (ret)
			return ret;
		return em28xx_i2c_bus_b_wait(st);
	}
	ret = em28xx_ctrl(st, false, stop ? EM28XX_REQ_I2C_STOP : EM28XX_REQ_I2C_NOSTOP,
			   0, addr << 1, (u8 *)buf, len);
	if (ret)
		return ret;
	return em28xx_i2c_wait(st);
}

int em28xx_i2c_read(struct em28xx_state *st, u8 addr, u8 *buf, u8 len)
{
	int ret;

	if (!len || len > 64)
		return -EINVAL;
	if (st->use_bus_b) {
		ret = em28xx_ctrl(st, true, EM28XX_REQ_I2C_BUS_B, 0, addr << 1,
				   buf, len);
		if (ret)
			return ret;
		return em28xx_i2c_bus_b_wait(st);
	}
	ret = em28xx_ctrl(st, true, EM28XX_REQ_I2C_STOP, 0, addr << 1, buf, len);
	if (ret)
		return ret;
	return em28xx_i2c_wait(st);
}

int em28xx_select_primary_i2c(struct em28xx_state *st, bool bus1)
{
	return em28xx_reg_write(st, EM28XX_REG_I2C_CLOCK,
				bus1 ? EM28XX_REG_I2C_CLOCK_BUS1 :
				EM28XX_REG_I2C_CLOCK_BUS0);
}

int em28xx_read_windows_eeprom_profile(struct em28xx_state *st,
					       struct em28xx_eeprom_profile *profile)
{
	u8 *raw;
	bool previous_bus_b = st->use_bus_b;
	int ret = 0;
	int offset;

	/* Board EEPROM layout used by these profiles, not an EM28xx requirement:
	 * K24C64 at primary I2C 0x50 with a 16-bit word address. Read-only.
	 */
	raw = kmalloc(EM28XX_EEPROM_SCAN_SIZE, GFP_KERNEL);
	if (!raw)
		return -ENOMEM;
	st->use_bus_b = false;
	ret = em28xx_select_primary_i2c(st, false);
	if (ret)
		goto out;
	for (offset = 0; offset < EM28XX_EEPROM_SCAN_SIZE; offset += 16) {
		u8 address[] = { offset >> 8, offset & 0xff };

		ret = em28xx_i2c_write(st, EM28XX_EEPROM_ADDR, address,
				       sizeof(address), false);
		if (ret)
			goto out;
		ret = em28xx_i2c_read(st, EM28XX_EEPROM_ADDR, raw + offset, 16);
		if (ret)
			goto out;
	}
	for (offset = 0; offset + 0x2e < EM28XX_EEPROM_SCAN_SIZE; offset++) {
		u32 magic = (u32)raw[offset] | ((u32)raw[offset + 1] << 8) |
			((u32)raw[offset + 2] << 16) | ((u32)raw[offset + 3] << 24);

		if (magic != EM28XX_EEPROM_MAGIC)
			continue;
		profile->offset = offset;
		profile->vid = raw[offset + 4] | (raw[offset + 5] << 8);
		profile->pid = raw[offset + 6] | (raw[offset + 7] << 8);
		profile->tuner_profile = raw[offset + 0x0a];
		profile->forced_tuner_addr = raw[offset + 0x2e];
		goto out;
	}
	ret = -ENOENT;
out:
	st->use_bus_b = previous_bus_b;
	kfree(raw);
	return ret;
}

bool em28xx_eeprom_is_dt5236(const struct em28xx_eeprom_profile *profile)
{
	/* emBDA: signature 0x9567eb1a, then DT5236 VID/PID and tuner profile 0x6a. */
	return profile->vid == EM28XX_VID && profile->pid == EM28XX_PID &&
		profile->tuner_profile == 0x6a;
}
