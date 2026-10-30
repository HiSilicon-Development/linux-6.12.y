/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * EM28xx DTMB 私有接口。
 *
 * 此目录按 DVB USB v2 的常见分层组织：core 只负责桥控制，板级 profile
 * 只负责已验证的前端和 TS gate，USB 注册文件只做 VID:PID 到 profile 的绑定。
 * 不在此模块加入 ATV、AV视频或 FM；它们属于独立的 V4L2/ALSA 路径。
 */
#ifndef DVB_USB_EM28XX_DTMB_H
#define DVB_USB_EM28XX_DTMB_H

#include <linux/cache.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include "dvb_usb.h"

#define EM28XX_DRIVER_NAME		"dvb_usb_em28xx"
#define EM28XX_VID			0xeb1a
#define EM28XX_PID			0x2885

#define EM28XX_REQ_BRIDGE		0x00
#define EM28XX_REQ_I2C_STOP		0x02
#define EM28XX_REQ_I2C_NOSTOP		0x03
#define EM28XX_REQ_I2C_BUS_B		0x06
#define EM28XX_REQ_I2C_BUS_B_STATUS	0x08
#define EM28XX_REG_I2C_STATUS		0x05
#define EM28XX_REG_I2C_CLOCK		0x06
#define EM28XX_REG_I2C_CLOCK_BUS0	0x40
#define EM28XX_REG_I2C_CLOCK_BUS1	0x45

#define EM28XX_EEPROM_ADDR		0x50
#define EM28XX_EEPROM_SCAN_SIZE	512
#define EM28XX_EEPROM_MAGIC		0x9567eb1a

enum em28xx_profile {
	EM28XX_PROFILE_UNKNOWN,
	EM28XX_PROFILE_DT5236_LGS8GL5_XC5000,
};

struct em28xx_state {
	struct dvb_usb_device *d;
	struct dvb_frontend frontend;
	struct mutex lock;
	bool initialized;
	bool disconnected;
	bool streaming;
	bool tuned;
	bool tuner_firmware_loaded;
	bool use_bus_b;
	enum em28xx_profile profile;
	u8 carrier;
	u8 fec;
	/* kmalloc-backed device private storage; protected by st->lock. */
	u8 control_buf[64] __aligned(ARCH_DMA_MINALIGN);
};

struct em28xx_eeprom_profile {
	u16 offset;
	u16 vid;
	u16 pid;
	u8 tuner_profile;
	u8 forced_tuner_addr;
};

int em28xx_ctrl(struct em28xx_state *st, bool read, u8 request,
		u16 value, u16 index, u8 *buf, u16 len);
int em28xx_reg_read(struct em28xx_state *st, u16 reg, u8 *value);
int em28xx_reg_write(struct em28xx_state *st, u16 reg, u8 value);
int em28xx_reg_update_bits(struct em28xx_state *st, u16 reg, u8 mask, u8 value);
int em28xx_i2c_write(struct em28xx_state *st, u8 addr, const u8 *buf,
		     u8 len, bool stop);
int em28xx_i2c_read(struct em28xx_state *st, u8 addr, u8 *buf, u8 len);
int em28xx_select_primary_i2c(struct em28xx_state *st, bool bus1);
int em28xx_read_windows_eeprom_profile(struct em28xx_state *st,
				       struct em28xx_eeprom_profile *profile);
bool em28xx_eeprom_is_dt5236(const struct em28xx_eeprom_profile *profile);

int dt5236_frontend_attach(struct dvb_usb_adapter *adap);
int dt5236_streaming_ctrl(struct dvb_frontend *fe, int onoff);
int dt5236_get_stream_config(struct dvb_frontend *fe, u8 *ts_type,
			     struct usb_data_stream_properties *stream);

#endif
