// SPDX-License-Identifier: GPL-2.0-only
/* Aigo DT5236 DTMB profile: LGS8GL5, XC5000 and TS gate. */
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/string.h>
#include <linux/usb.h>

#include "em28xx-dtmb.h"

#define DT5236_LGS_ADDR		0x19
#define DT5236_LGS_ALT_ADDR		0x1b
#define DT5236_XC5000_ADDR		0x61
#define DT5236_XC5000_FIRMWARE	"dvb-fe-xc5000-1.6.114.fw"
/* DVB 前端从 42 MHz 开始公布；Android 会依此完成初始属性协商。 */
#define DT5236_FRONTEND_MIN_HZ		42000000U
#define DT5236_FRONTEND_MAX_HZ		864000000U
#define DT5236_GATE_STOP_DELAY_MS	50
#define DT5236_ALT0_SETTLE_MS		80
static int dt5236_arm_ts_gate(struct em28xx_state *st);
static int dt5236_disarm_ts_gate(struct em28xx_state *st);

static int dt5236_lgs_read_addr(struct em28xx_state *st, u8 addr, u8 reg, u8 *value)
{
	int ret;

	ret = em28xx_i2c_write(st, addr, &reg, 1, false);
	if (ret)
		return ret;
	return em28xx_i2c_read(st, addr, value, 1);
}

static int dt5236_lgs_read(struct em28xx_state *st, u8 reg, u8 *value)
{
	return dt5236_lgs_read_addr(st, DT5236_LGS_ADDR, reg, value);
}

static int dt5236_lgs_write_addr(struct em28xx_state *st, u8 addr, u8 reg, u8 value)
{
	u8 data[] = { reg, value };

	return em28xx_i2c_write(st, addr, data, sizeof(data), true);
}

static int dt5236_lgs_write(struct em28xx_state *st, u8 reg, u8 value)
{
	return dt5236_lgs_write_addr(st, DT5236_LGS_ADDR, reg, value);
}

static int dt5236_lgs_alt_update(struct em28xx_state *st, u8 reg, u8 value)
{
	u8 ignored;
	int ret;

	/* The 0x1b bank requires the read phase before each write. */
	ret = dt5236_lgs_read_addr(st, DT5236_LGS_ALT_ADDR, reg, &ignored);
	if (ret)
		return ret;
	return dt5236_lgs_write_addr(st, DT5236_LGS_ALT_ADDR, reg, value);
}

static int dt5236_lgs_reset(struct em28xx_state *st)
{
	int ret;

	ret = dt5236_lgs_write(st, 0x02, 0x00);
	if (ret)
		return ret;
	ret = dt5236_lgs_write(st, 0x02, 0x01);
	if (ret)
		return ret;
	msleep(100);
	return 0;
}

static int dt5236_lgs_prepare_auto(struct em28xx_state *st)
{
	u8 value;
	int ret;

	ret = dt5236_lgs_read(st, 0x03, &value);
	if (ret)
		return ret;
	ret = dt5236_lgs_write(st, 0x03, value & ~0x01);
	if (ret)
		return ret;
	ret = dt5236_lgs_read(st, 0x7e, &value);
	if (ret)
		return ret;
	ret = dt5236_lgs_write(st, 0x7e, value | 0x01);
	if (ret)
		return ret;
	ret = dt5236_lgs_read_addr(st, DT5236_LGS_ALT_ADDR, 0xc5, &value);
	if (ret)
		return ret;
	return dt5236_lgs_write_addr(st, DT5236_LGS_ALT_ADDR, 0xc5,
					 value & 0xe0);
}

static int dt5236_lgs_i2c_gate(struct em28xx_state *st, bool enable)
{
	return dt5236_lgs_write(st, 0x01,
				enable ? 0x80 | DT5236_XC5000_ADDR : 0x00);
}

static int dt5236_xc_write_once(struct em28xx_state *st, const u8 *buf,
				u8 len)
{
	int ret;

	ret = em28xx_select_primary_i2c(st, true);
	if (ret)
		return ret;
	return em28xx_i2c_write(st, DT5236_XC5000_ADDR, buf, len, true);
}

static int dt5236_xc_write(struct em28xx_state *st, const u8 *buf, u8 len)
{
	int ret;

	ret = dt5236_xc_write_once(st, buf, len);
	if (ret != -ENXIO && ret != -ETIMEDOUT)
		return ret;
	ret = dt5236_lgs_i2c_gate(st, true);
	if (ret)
		return ret;
	ret = dt5236_xc_write_once(st, buf, len);
	if (dt5236_lgs_i2c_gate(st, false) && !ret)
		return -EIO;
	return ret;
}

static int dt5236_xc_read_reg_once(struct em28xx_state *st, u16 reg,
				    u16 *value)
{
	u8 address[] = { reg >> 8, reg & 0xff };
	u8 data[2];
	int ret;

	ret = em28xx_select_primary_i2c(st, true);
	if (ret)
		return ret;
	ret = em28xx_i2c_write(st, DT5236_XC5000_ADDR, address, sizeof(address), false);
	if (ret)
		return ret;
	ret = em28xx_i2c_read(st, DT5236_XC5000_ADDR, data, sizeof(data));
	if (ret)
		return ret;
	*value = (data[0] << 8) | data[1];
	return 0;
}

static int dt5236_xc_read_reg(struct em28xx_state *st, u16 reg, u16 *value)
{
	int ret;

	ret = dt5236_xc_read_reg_once(st, reg, value);
	if (ret != -ENXIO && ret != -ETIMEDOUT)
		return ret;
	ret = dt5236_lgs_i2c_gate(st, true);
	if (ret)
		return ret;
	ret = dt5236_xc_read_reg_once(st, reg, value);
	if (dt5236_lgs_i2c_gate(st, false) && !ret)
		return -EIO;
	return ret;
}

static int dt5236_xc_write_reg(struct em28xx_state *st, u16 reg, u16 value)
{
	u8 data[] = { reg >> 8, reg & 0xff, value >> 8, value & 0xff };
	u16 busy;
	int ret, i;

	ret = dt5236_xc_write(st, data, sizeof(data));
	if (ret)
		return ret;
	for (i = 0; i < 200; i++) {
		ret = dt5236_xc_read_reg(st, 0x0009, &busy);
		if (ret)
			return ret;
		if (!busy)
			return 0;
		msleep(5);
	}
	return -ETIMEDOUT;
}

static int dt5236_xc_load_firmware(struct em28xx_state *st)
{
	const struct firmware *fw;
	size_t offset = 0;
	u16 product, length, busy;
	bool saw_end = false;
	int ret, i;
	u8 boot[] = { 0x00, 0x00, 0x00, 0x00 };

	ret = dt5236_xc_read_reg(st, 0x0008, &product);
	if (!ret && product == 0x1388) {
		st->tuner_firmware_loaded = true;
		return 0;
	}
	ret = request_firmware(&fw, DT5236_XC5000_FIRMWARE, &st->d->udev->dev);
	if (ret)
		return ret;

	while (offset + 2 <= fw->size) {
		length = (fw->data[offset] << 8) | fw->data[offset + 1];
		offset += 2;
		if (length == 0xffff) {
			saw_end = true;
			break;
		}
		if (!length)
			continue;
		if (length & 0x8000) {
			msleep(length & 0x7fff);
			continue;
		}
		if (length < 2 || offset + length > fw->size) {
			ret = -EINVAL;
			goto out;
		}
		/* XC5000 loader blocks repeat their two-byte address per 62-byte chunk. */
		{
			size_t pos = 2;
			u8 transaction[64];
			while (pos < length) {
				u8 payload = min_t(size_t, 62, length - pos);
				transaction[0] = fw->data[offset];
				transaction[1] = fw->data[offset + 1];
				memcpy(&transaction[2], &fw->data[offset + pos], payload);
				ret = dt5236_xc_write(st, transaction, payload + 2);
				if (ret)
					goto out;
				pos += payload;
			}
		}
		offset += length;
	}
	if (!saw_end) {
		ret = -EINVAL;
		goto out;
	}
	msleep(20);
	ret = dt5236_xc_write(st, boot, sizeof(boot));
	if (ret)
		goto out;
	msleep(100);
	for (i = 0; i < 10; i++) {
		ret = dt5236_xc_read_reg(st, 0x0009, &busy);
		if (ret)
			goto out;
		if (!busy) {
			st->tuner_firmware_loaded = true;
			ret = 0;
			goto out;
		}
		msleep(100);
	}
	ret = -ETIMEDOUT;
out:
	release_firmware(fw);
	return ret;
}

static int dt5236_lgs_init(struct em28xx_state *st)
{
	static const struct { u8 reg, value; } clear_table[] = {
		{ 0x03, 0x00 }, { 0x09, 0x00 }, { 0x0a, 0x00 },
		{ 0x0b, 0x00 }, { 0x0c, 0x00 }, { 0x02, 0x00 }, { 0x02, 0x01 },
	};
	static const u8 if_table[] = { 0xa1, 0xbc, 0x86, 0x32 };
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(clear_table); i++) {
		ret = dt5236_lgs_write(st, clear_table[i].reg, clear_table[i].value);
		if (ret)
			return ret;
	}
	ret = dt5236_lgs_alt_update(st, 0xc2, 0x05);
	if (ret)
		return ret;
	ret = dt5236_lgs_reset(st);
	if (ret)
		return ret;
	ret = dt5236_lgs_write(st, 0x07, 0x14);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(if_table); i++) {
		ret = dt5236_lgs_write(st, 0x09 + i, if_table[i]);
		if (ret)
			return ret;
	}

	/* sub_14527: LGS8GL5 lock preparation */
	ret = dt5236_lgs_prepare_auto(st);
	if (ret)
		return ret;
	return dt5236_lgs_reset(st);
}

static int dt5236_lgs_capture(struct em28xx_state *st)
{
	static const u8 modes[] = { 4, 0, 2 };
	u8 reg4, reg37, status, a2, c6;
	int ret, mode, i, pass;

	ret = dt5236_lgs_prepare_auto(st);
	if (ret)
		return ret;
	ret = dt5236_lgs_alt_update(st, 0xc2, 0x05);
	if (ret)
		return ret;
	ret = dt5236_lgs_write(st, 0x07, 0x14);
	if (ret)
		return ret;

	for (mode = 0; mode < ARRAY_SIZE(modes); mode++) {
		ret = dt5236_lgs_read(st, 0x04, &reg4);
		if (ret)
			return ret;
		ret = dt5236_lgs_read(st, 0x37, &reg37);
		if (ret)
			return ret;

		reg4 &= 0xfc;
		switch (modes[mode]) {
		case 0:
			reg4 |= 0x02;
			reg37 &= ~0x80;
			break;
		case 2:
			reg37 &= ~0x80;
			break;
		case 4:
			reg4 |= 0x01;
			reg37 |= 0x80;
			break;
		}

		ret = dt5236_lgs_write(st, 0x04, reg4);
		if (ret)
			return ret;
		ret = dt5236_lgs_write(st, 0x37, reg37);
		if (ret)
			return ret;
		ret = dt5236_lgs_reset(st);
		if (ret)
			return ret;

		for (i = 0; i < 20; i++) {
			ret = dt5236_lgs_read(st, 0x4b, &st->carrier);
			if (ret)
				return ret;
			if (st->carrier & 0x80)
				break;
			msleep(30);
		}
		dev_dbg(&st->d->udev->dev,
			 "DT5236: mode=%d carrier_attempts=%d reg4b=0x%02x\n",
			 modes[mode], i, st->carrier);
		if (!(st->carrier & 0x80))
			continue;
		for (pass = 0; pass < 2; pass++) {
			for (i = 0; i < 15; i++) {
				ret = dt5236_lgs_read(st, 0xa4, &st->fec);
				if (ret)
					return ret;
				dev_dbg(&st->d->udev->dev,
					 "DT5236: mode=%d pass=%d read 0xa4=0x%02x\n",
					 modes[mode], pass, st->fec);
				if ((st->fec & 0x03) == 0x01)
					goto locked;
				msleep(50);
			}
			if (!pass) {
				ret = dt5236_lgs_read(st, 0x7c, &status);
				if (ret)
					return ret;
				ret = dt5236_lgs_write(st, 0x7c, status ^ 0x80);
				if (ret)
					return ret;
			}
		}
	}
	ret = dt5236_lgs_write(st, 0x03, 0x01);
	if (ret)
		return ret;
	goto finish;

locked:
	ret = dt5236_lgs_read(st, 0xa2, &a2);
	if (ret)
		return ret;
	{
		u8 val;
		ret = dt5236_lgs_read(st, 0x7e, &val);
		if (!ret)
			ret = dt5236_lgs_write(st, 0x7e, val & ~0x01);
		if (!ret)
			ret = dt5236_lgs_read_addr(st, DT5236_LGS_ALT_ADDR, 0xc5, &val);
		if (!ret)
			ret = dt5236_lgs_write_addr(st, DT5236_LGS_ALT_ADDR, 0xc5, (val & 0xe0) | 0x06);
		if (!ret)
			ret = dt5236_lgs_reset(st);
		if (ret)
			return ret;
	}
	ret = dt5236_lgs_write(st, 0x7d, a2);
	if (ret)
		return ret;
	ret = dt5236_lgs_reset(st);
	if (ret)
		return ret;
finish:
	ret = dt5236_lgs_reset(st);
	if (ret)
		return ret;
	msleep(100);
	ret = dt5236_lgs_read_addr(st, DT5236_LGS_ALT_ADDR, 0xc6, &c6);
	if (ret)
		return ret;
	ret = dt5236_lgs_write_addr(st, DT5236_LGS_ALT_ADDR, 0xc6, c6 & ~0x80);
	if (ret)
		return ret;
	ret = dt5236_lgs_read_addr(st, DT5236_LGS_ALT_ADDR, 0xc6, &c6);
	if (ret)
		return ret;
	ret = dt5236_lgs_write_addr(st, DT5236_LGS_ALT_ADDR, 0xc6, c6 | 0x40);
	if (ret)
		return ret;
	ret = dt5236_lgs_read_addr(st, DT5236_LGS_ALT_ADDR, 0xc6, &c6);
	if (ret)
		return ret;
	return dt5236_lgs_write_addr(st, DT5236_LGS_ALT_ADDR, 0xc6, c6 & ~0x40);
}

static int dt5236_tune(struct em28xx_state *st, u32 frequency)
{
	u16 rf_code;
	int ret;

	/* Android 的属性协商可能在真正调谐前传入未设置的 frequency。 */
	if (!frequency)
		return -EINVAL;
	if (frequency < DT5236_FRONTEND_MIN_HZ ||
	    frequency > DT5236_FRONTEND_MAX_HZ)
		return -EINVAL;
	rf_code = div_u64((u64)(frequency - 2750000) , 15625);
	dev_dbg(&st->d->udev->dev,
		 "DT5236 tune: freq=%u Hz, rf_code=0x%04x\n", frequency, rf_code);
	ret = dt5236_xc_write_reg(st, 0x0001, 0x800b);
	if (ret) return ret;
	ret = dt5236_xc_write_reg(st, 0x0002, 0x00c0);
	if (ret) return ret;
	ret = dt5236_xc_write_reg(st, 0x0005, 0x1800);
	if (ret) return ret;
	ret = dt5236_xc_write_reg(st, 0x000b, 0x008a);
	if (ret) return ret;
	ret = dt5236_xc_write_reg(st, 0x000d, 0x0000);
	if (ret) return ret;
	ret = dt5236_xc_write_reg(st, 0x0004, 0x0000);
	if (ret) return ret;
	ret = dt5236_xc_write_reg(st, 0x0003, rf_code);
	if (ret) return ret;
	ret = dt5236_xc_write_reg(st, 0x0010, rf_code);
	if (ret) return ret;
	msleep(100);
	return dt5236_lgs_capture(st);
}

static int dt5236_probe_lgs(struct em28xx_state *st)
{
	u8 chip_id;
	int ret;

	ret = dt5236_lgs_read(st, 0x00, &chip_id);
	if (ret)
		return ret;
	return chip_id == 0x0e ? 0 : -ENODEV;
}

static int dt5236_fe_init(struct dvb_frontend *fe)
{
	struct em28xx_state *st = fe->demodulator_priv;
	struct em28xx_eeprom_profile eeprom_profile;
	u16 tuner_id;
	int ret;
	bool board_started = false;

	mutex_lock(&st->lock);
	if (st->disconnected) {
		ret = -ENODEV;
		goto out;
	}
	if (st->initialized) {
		mutex_unlock(&st->lock);
		return 0;
	}
	ret = em28xx_read_windows_eeprom_profile(st, &eeprom_profile);
	if (!ret) {
		if (!em28xx_eeprom_is_dt5236(&eeprom_profile)) {
			dev_info(&st->d->udev->dev,
				 "EM28xx EEPROM profile at 0x%04x is %04x:%04x tuner=0x%02x; refusing DT5236 init\n",
				 eeprom_profile.offset, eeprom_profile.vid, eeprom_profile.pid,
				 eeprom_profile.tuner_profile);
			ret = -ENODEV;
			goto out;
		}
		if (eeprom_profile.forced_tuner_addr != DT5236_XC5000_ADDR) {
			dev_err(&st->d->udev->dev,
				"EM28xx EEPROM DT5236 tuner address 0x%02x is not 0x%02x\n",
				eeprom_profile.forced_tuner_addr, DT5236_XC5000_ADDR);
			ret = -ENODEV;
			goto out;
		}
		st->profile = EM28XX_PROFILE_DT5236_LGS8GL5_XC5000;
		dev_info(&st->d->udev->dev,
			 "EM28xx EEPROM selected DT5236 at 0x%04x (tuner=0x%02x i2c=0x%02x)\n",
			 eeprom_profile.offset, eeprom_profile.tuner_profile,
			 eeprom_profile.forced_tuner_addr);
	} else if (ret == -ENOENT) {
		dev_info(&st->d->udev->dev,
			 "EM28xx EEPROM config absent; validating DT5236 frontend signature\n");
	} else {
		goto out;
	}

	/* This board's GPIO/power sequence is not generic bridge wiring. */
	board_started = true;
	ret = dt5236_arm_ts_gate(st);
	if (ret)
		goto out;
	msleep(60);

	st->use_bus_b = true;
	ret = dt5236_probe_lgs(st);
	dev_info(&st->d->udev->dev, "DT5236 Bus B LGS probe: %d\n", ret);
	if (ret) {
		st->use_bus_b = false;
		ret = em28xx_select_primary_i2c(st, true);
		if (!ret)
			ret = dt5236_probe_lgs(st);
		dev_info(&st->d->udev->dev, "DT5236 primary LGS probe: %d\n", ret);
	}
	if (!ret) {
		ret = dt5236_xc_read_reg(st, 0x0008, &tuner_id);
		dev_info(&st->d->udev->dev,
			 "DT5236 XC5000 product probe: ret=%d id=0x%04x\n",
			 ret, ret ? 0 : tuner_id);
	}
	if (!ret && tuner_id != 0x1388 && tuner_id != 0x2000 && tuner_id != 0x0000)
		ret = -ENODEV;
	if (ret) {
		/* Reject an unverified LGS8GL5/XC5000 frontend. */
		dev_err(&st->d->udev->dev,
			"EM28xx: DT5236 profile not verified (%d); refusing profile fallback\n",
			ret);
		goto out;
	}
	st->profile = EM28XX_PROFILE_DT5236_LGS8GL5_XC5000;
	if (!ret && tuner_id != 0x1388)
		ret = dt5236_xc_load_firmware(st);
	if (!ret)
		ret = dt5236_lgs_init(st);
	if (!ret)
		ret = dt5236_disarm_ts_gate(st);
	if (!ret)
		st->initialized = true;
	if (!ret)
		dev_info(&st->d->udev->dev,
			 "EM28xx profile: Aigo DT5236 (LGS8GL5 + XC5000)\n");
out:
	if (ret) {
		if (board_started)
			dt5236_disarm_ts_gate(st);
		st->initialized = false;
		st->tuner_firmware_loaded = false;
		st->profile = EM28XX_PROFILE_UNKNOWN;
	}
	mutex_unlock(&st->lock);
	return ret;
}
static int dt5236_fe_sleep(struct dvb_frontend *fe)
{
	struct em28xx_state *st = fe->demodulator_priv;
	int ret;

	mutex_lock(&st->lock);
	ret = st->disconnected ? 0 : dt5236_disarm_ts_gate(st);
	st->initialized = false;
	st->tuned = false;
	st->streaming = false;
	st->tuner_firmware_loaded = false;
	st->profile = EM28XX_PROFILE_UNKNOWN;
	fe->dtv_property_cache.strength.len = 0;
	fe->dtv_property_cache.cnr.len = 0;
	mutex_unlock(&st->lock);
	return ret;
}

static int dt5236_set_frontend(struct dvb_frontend *fe)
{
	struct em28xx_state *st = fe->demodulator_priv;
	int ret;

	if (fe->dtv_property_cache.bandwidth_hz &&
	    fe->dtv_property_cache.bandwidth_hz != 8000000)
		return -EINVAL;
	ret = dt5236_fe_init(fe);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	if (st->disconnected || !st->initialized) {
		mutex_unlock(&st->lock);
		return -ENODEV;
	}
	fe->dtv_property_cache.strength.len = 0;
	fe->dtv_property_cache.cnr.len = 0;
	st->tuned = false;
	ret = dt5236_disarm_ts_gate(st);
	if (!ret)
		ret = dt5236_tune(st, fe->dtv_property_cache.frequency);
	if (!ret) {
		st->tuned = true;
		if (st->streaming)
			ret = em28xx_reg_update_bits(st, 0x5f, 0x01, 0x01);
	}
	if (ret) {
		st->tuned = false;
		dt5236_disarm_ts_gate(st);
	}
	mutex_unlock(&st->lock);
	return ret;
}

static int dt5236_get_strength(struct em28xx_state *st, u16 *strength)
{
	u8 level;
	int ret;

	/* The dedicated lgs8gl5 driver uses 0x4b[6:0] for relative strength;
	 * 0x4b[7] is the carrier flag. Generic LGS8Gxx AGC bands do not apply.
	 */
	ret = dt5236_lgs_read(st, 0x4b, &level);
	if (ret)
		return ret;
	*strength = (level & 0x7f) << 9;
	return 0;
}

static int dt5236_get_snr(struct em28xx_state *st, u16 *snr)
{
	u8 noise;
	int ret;

	ret = dt5236_lgs_read(st, 0x95, &noise);
	if (ret)
		return ret;
	/* Native LGS8Gxx inverse average-noise quality, not calibrated dB.
	 * Scale before narrowing so zero noise reaches 100%, not u16 wrap.
	 */
	*snr = (256 - noise) * 0xffff / 256;
	return 0;
}

static int dt5236_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct em28xx_state *st = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u8 carrier, fec;
	u16 snr;
	int ret = 0;

	*status = 0;
	mutex_lock(&st->lock);
	c->strength.len = 0;
	c->cnr.len = 0;
	if (st->disconnected) {
		ret = -ENODEV;
		goto out;
	}
	if (!st->initialized || !st->tuned)
		goto out;
	ret = dt5236_lgs_read(st, 0x4b, &carrier);
	if (!ret)
		ret = dt5236_lgs_read(st, 0xa4, &fec);
	if (ret)
		goto out;
	if (carrier & 0x7f)
		*status |= FE_HAS_SIGNAL;
	if (carrier & 0x80)
		*status |= FE_HAS_CARRIER;
	if (fec & 0x04)
		*status |= FE_HAS_SYNC;
	if (fec & 0x02)
		*status |= FE_HAS_VITERBI;
	/* 0xa4 bit 0 completes parameter detection; bit 2 is data sync. */
	if ((carrier & 0x80) && ((fec & 0x03) == 0x01) && (fec & 0x04))
		*status |= FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
			FE_HAS_SYNC | FE_HAS_LOCK;
	/* A metric read failure must not turn a valid lock into a tune failure. */
	c->strength.len = 1;
	c->strength.stat[0].scale = FE_SCALE_RELATIVE;
	c->strength.stat[0].uvalue = (carrier & 0x7f) << 9;
	if ((*status & FE_HAS_LOCK) && !dt5236_get_snr(st, &snr)) {
		c->cnr.len = 1;
		c->cnr.stat[0].scale = FE_SCALE_RELATIVE;
		c->cnr.stat[0].uvalue = snr;
	}
out:
	mutex_unlock(&st->lock);
	return ret;
}

static int dt5236_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct em28xx_state *st = fe->demodulator_priv;
	int ret = 0;

	*strength = 0;
	mutex_lock(&st->lock);
	if (st->disconnected)
		ret = -ENODEV;
	else if (st->initialized && st->tuned) {
		ret = dt5236_get_strength(st, strength);
	}
	mutex_unlock(&st->lock);
	return ret;
}

static int dt5236_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct em28xx_state *st = fe->demodulator_priv;
	struct dtv_fe_stats *cnr = &fe->dtv_property_cache.cnr;
	enum fe_status status;
	int ret;

	*snr = 0;
	ret = dt5236_read_status(fe, &status);
	if (ret)
		return ret;
	mutex_lock(&st->lock);
	if (cnr->len && cnr->stat[0].scale == FE_SCALE_RELATIVE)
		*snr = cnr->stat[0].uvalue;
	mutex_unlock(&st->lock);
	return 0;
}

static enum dvbfe_algo dt5236_get_frontend_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}

static int dt5236_frontend_tune(struct dvb_frontend *fe, bool re_tune,
				unsigned int mode_flags, unsigned int *delay,
				enum fe_status *status)
{
	int ret;

	*delay = HZ / 4;
	*status = 0;
	/* set_frontend searches all PN modes; status polls must not reset it. */
	if (re_tune) {
		ret = dt5236_set_frontend(fe);
		if (ret)
			return ret;
	}
	return dt5236_read_status(fe, status);
}

static const struct dvb_frontend_ops dt5236_frontend_ops = {
	.delsys = { SYS_DVBT, SYS_DTMB },
	.info = {
		.name = "Aigo DT5236 LGS8GL5 DTMB",
		.frequency_min_hz = DT5236_FRONTEND_MIN_HZ,
		.frequency_max_hz = DT5236_FRONTEND_MAX_HZ,
		.frequency_stepsize_hz = 10000,
		.caps = FE_CAN_FEC_AUTO | FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO | FE_CAN_BANDWIDTH_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO,
	},
	.init = dt5236_fe_init,
	.sleep = dt5236_fe_sleep,
	.set_frontend = dt5236_set_frontend,
	.get_frontend_algo = dt5236_get_frontend_algo,
	.tune = dt5236_frontend_tune,
	.read_status = dt5236_read_status,
	.read_signal_strength = dt5236_read_signal_strength,
	.read_snr = dt5236_read_snr,
};
int dt5236_frontend_attach(struct dvb_usb_adapter *adap)
{
	struct em28xx_state *st = d_to_priv(adap_to_d(adap));

	st->profile = EM28XX_PROFILE_UNKNOWN;
	memcpy(&st->frontend.ops, &dt5236_frontend_ops, sizeof(dt5236_frontend_ops));
	st->frontend.demodulator_priv = st;
	adap->fe[0] = &st->frontend;
	return 0;
}
static int dt5236_arm_ts_gate(struct em28xx_state *st)
{
	/* Captured DT5236 board setup includes GPIO/power values. Their pin
	 * polarity and attached loads must not be inferred for other EM28xx boards.
	 */
	static const struct { u16 reg; u8 value; } sequence[] = {
		{ 0x13, 0x08 }, { 0x12, 0x27 }, { 0x0c, 0x10 }, { 0x27, 0x00 },
		{ 0x10, 0x00 }, { 0x11, 0x11 }, { 0x50, 0x01 }, { 0x5d, 0x05 },
	};
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(sequence); i++) {
		ret = em28xx_reg_write(st, sequence[i].reg, sequence[i].value);
		if (ret)
			return ret;
	}
	return em28xx_reg_update_bits(st, 0x5f, 0x01, 0x01);
}

static int dt5236_disarm_ts_gate(struct em28xx_state *st)
{
	int ret;

	ret = em28xx_reg_update_bits(st, 0x5f, 0x01, 0x00);
	if (!ret)
		msleep(DT5236_GATE_STOP_DELAY_MS);
	return ret;
}

static void dt5236_rollback_stream_start(struct em28xx_state *st, int start_ret)
{
	int ret;

	ret = dt5236_disarm_ts_gate(st);
	if (ret)
		dev_warn(&st->d->udev->dev,
			 "DT5236 stream start rollback: TS gate stop failed: %d\n",
			 ret);
	ret = usb_set_interface(st->d->udev, 0, 0);
	if (ret)
		dev_warn(&st->d->udev->dev,
			 "DT5236 stream start rollback: alt0 restore failed: %d\n",
			 ret);
	dev_err(&st->d->udev->dev, "DT5236 stream start failed: %d\n",
		start_ret);
}

int dt5236_get_stream_config(struct dvb_frontend *fe, u8 *ts_type,
				    struct usb_data_stream_properties *stream)
{
	struct em28xx_state *st = fe->demodulator_priv;
	int ret;

	/* DVB USB v2 在这里尚未提交新 URB。先撤到 alt0，保证上一次停流后
	 * 遗留的 alt1 不会让桥寄存器写入与等时端点状态交错；随后严格按
	 * DT5236 已验证顺序启 gate，再选择 alt1。 */
	mutex_lock(&st->lock);
	if (st->disconnected || !st->initialized ||
	    st->profile != EM28XX_PROFILE_DT5236_LGS8GL5_XC5000) {
		ret = -ENODEV;
		goto out;
	}
	ret = usb_set_interface(st->d->udev, 0, 0);
	if (!ret)
		msleep(DT5236_ALT0_SETTLE_MS);
	if (!ret)
		ret = dt5236_arm_ts_gate(st);
	if (!ret)
		ret = usb_set_interface(st->d->udev, 0, 1);
	if (ret)
		dt5236_rollback_stream_start(st, ret);
out:
	mutex_unlock(&st->lock);
	if (!ret)
		*ts_type = DVB_USB_FE_TS_TYPE_188;
	return ret;
}

int dt5236_streaming_ctrl(struct dvb_frontend *fe, int onoff)
{
	struct em28xx_state *st = fe->demodulator_priv;
	int ret = 0;

	mutex_lock(&st->lock);
	if (st->disconnected) {
		ret = onoff ? -ENODEV : 0;
	} else if (onoff) {
		ret = st->initialized ?
			em28xx_reg_update_bits(st, 0x5f, 0x01, 0x01) : -ENODEV;
		if (!ret)
			st->streaming = true;
	} else {
		st->streaming = false;
		/* URBs are still live here; alt0 belongs to stream preparation. */
		ret = dt5236_disarm_ts_gate(st);
	}
	mutex_unlock(&st->lock);
	return ret;
}
