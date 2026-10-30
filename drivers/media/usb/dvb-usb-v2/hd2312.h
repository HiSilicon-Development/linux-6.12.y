/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Cypress FX2 / HDIC HD2312 DTMB receiver driver
 *
 * Based on hanwckf's HD2312 driver. Firmware-specific adapters share
 * its USB control and frontend protocol.
 *
 * Copyright (C) 2024 hanwckf <hanwckf@vip.qq.com>
 */

#ifndef _HD2312_H_
#define _HD2312_H_

#include <linux/types.h>
#include <linux/mutex.h>
#include <media/dvb_frontend.h>
#include "dvb_usb.h"

/* USB Device Identity */
#define HD2312_USB_VID			0x04b4
#define HD2312_USB_PID			0x1004

/* Protocol Timing Timeouts */
#define HD2312_USB_TIMEOUT_MS		500
#define HD2312_LOCK_TIMEOUT_MS		800

/* Full scale SNR in milli-decibels (30 dB) */
#define HD2312_SNR_FULL_SCALE_MDB	30000

/* Hardware Command Request IDs (SETUPDAT[1] bRequest) */
#define HD2312_REQ_STREAM_ON		0xab
#define HD2312_REQ_STREAM_OFF		0xac
#define HD2312_REQ_POWER_ON		0xad
#define HD2312_REQ_POWER_OFF		0xae
#define HD2312_REQ_CVB_EXTRA_AUTH	0xbe
#define HD2312_REQ_CVB_EXTRA_DATA	0xbf
#define HD2312_REQ_EEPROM_RW		0xd7
#define HD2312_REQ_TUNER_WRITE		0xd8
#define HD2312_REQ_TUNER_READ		0xd9
#define HD2312_REQ_FRONTEND		0xe7
#define HD2312_REQ_SNR			0xe8
#define HD2312_REQ_BER			0xe9
#define HD2312_REQ_LOCK			0xea
#define HD2312_REQ_STRENGTH		0xeb
#define HD2312_REQ_VERSION		0xed
#define HD2312_REQ_FREQUENCY		0xfc

/* Known firmware version profiles used for board naming and activation. */
enum hd2312_vendor {
	HD2312_VENDOR_GENERIC = 0,
	HD2312_VENDOR_LETV    = 0x03, /* LeShi / Letv (ver: 03.08.2044) */
	HD2312_VENDOR_AIWA    = 0x05, /* Aiwa itv720  (ver: 05.08.2044) */
	HD2312_VENDOR_CVB     = 0x06, /* CVB T302A    (ver: 06.08.2044) */
};

struct hd2312_state {
	struct dvb_frontend fe;
	struct dvb_usb_device *d;
	enum hd2312_vendor vendor;
	u8 fw_version[4];
	u32 frequency;
	u16 cached_strength;
	u16 cached_snr;
	u8 status[6];
	u8 statistics[4];
	u8 ctrl_buf[32] __aligned(ARCH_DMA_MINALIGN);
};

#endif /* _HD2312_H_ */
