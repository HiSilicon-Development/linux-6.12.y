// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL6381 DVB-C / DTMB demodulator driver
 *
 * Copyright (C) 2024 Xiaodong Ni <nxiaodong520@gmail.com>
 *
 * Based on nxdong520/avl6381, with I2C-client binding and DVBv5 statistics.
 * DTMB reception uses the vendor DVB-T/DVB-T2 userspace interface.
 *
 * Chip family id 0x63814e24 @ 0x040000.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <media/dvb_frontend.h>

#include "avl6381_priv.h"
#include "avl6381_tables.h"

/* Register map from the nxdong520/avl6381 community driver. */
#define AVL6381_REG_FAMILY_ID		0x040000
#define AVL6381_FAMILY_ID_6381		0x63814e24
#define AVL6381_REG_CHIP_ID		0x108004

#define AVL6381_REG_RXOP_STATUS		0x000204	/* !=0: busy */
#define AVL6381_REG_CORE_RESET		0x38FFFC
#define AVL6381_REG_CHIP_READY		0x110840	/* 1: not ready */
#define AVL6381_REG_READY_MAGIC		0x0000A0
#define AVL6381_READY_MAGIC_VALUE	0x5AA57FF7

/* RxOP mailbox commands */
#define AVL6381_RXOP_INIT_RX		1
#define AVL6381_RXOP_AUTOLOCK		2
#define AVL6381_RXOP_HALT		3
#define AVL6381_RXOP_SLEEP		6
#define AVL6381_RXOP_INIT_SDRAM		8
#define AVL6381_RXOP_INIT_ADC		9
#define AVL6381_RXOP_MODE_SWITCH	10
#define AVL6381_RXOP_DVBC_AUTOLOCK	12

/* DTMB status */
#define AVL6381_DTMB_REG_LOCK		0x0000A6	/* u8, !=0 locked */
#define AVL6381_DTMB_REG_SNR		0x00011C	/* u16 */
#define AVL6381_DTMB_REG_RUNNING	0x000124	/* u8, !=0 running */
#define AVL6381_DTMB_REG_HALT		0x0000A4
#define AVL6381_DTMB_REG_SYMRATE	0x000300
#define AVL6381_DTMB_SYM_RATE_HZ	7560000

/* DVBC status */
#define AVL6381_DVBC_REG_LOCK		0x0001A4	/* u32, ==21 locked */
#define AVL6381_DVBC_REG_SNR		0x0001AE
#define AVL6381_DVBC_REG_RUNNING	0x0001A4
#define AVL6381_DVBC_REG_HALT		0x0001A0
#define AVL6381_DVBC_REG_SNR_TRIG	0x0005D8

#define AVL6381_SIGNAL_MIN_MDB	(-90000)
#define AVL6381_SIGNAL_MAX_MDB	0
#define AVL6381_CNR_MIN_MDB	0
#define AVL6381_CNR_MAX_MDB	30000

#define AVL6381_RELATIVE_MAX	65535

/* PER statistics */
#define AVL6381_REG_PER_ERR		0x149110
#define AVL6381_REG_PER_CNT		0x149114

#define AVL6381_TS_SYM_RATE_HZ		7560000

enum avl6381_mode {
	AVL6381_MODE_DTMB,
	AVL6381_MODE_DVBC,
};

struct avl6381_priv {
	struct i2c_client	*client;
	struct dvb_frontend	fe;
	const struct avl6381_config *config;
	struct mutex		mutex; /* Serializes tuning and status reads. */
	enum fe_delivery_system	delivery_system;
	bool			inited;
};

static u16 avl6381_relative_scale(s64 value, s64 min, s64 max)
{
	if (value <= min)
		return 0;
	if (value >= max)
		return AVL6381_RELATIVE_MAX;

	return div_u64((u64)(value - min) * AVL6381_RELATIVE_MAX,
		       max - min);
}

/*
 * I2C protocol: 3-byte big-endian register address, then 1/2/4-byte
 * big-endian data. Reads are a write of the address followed by a
 * separate read transfer.
 */
static int avl6381_i2c_wr(struct avl6381_priv *priv, const u8 *buf, int len)
{
	struct i2c_msg msg = {
		.addr = priv->client->addr,
		.flags = 0,
		.buf = (u8 *)buf,
		.len = len,
	};
	int ret = i2c_transfer(priv->client->adapter, &msg, 1);

	if (ret == 1)
		return 0;

	dev_warn(&priv->client->dev, "i2c wr failed=%d len=%d\n", ret, len);
	return -EREMOTEIO;
}

static int avl6381_i2c_rd(struct avl6381_priv *priv, u8 *buf, int len)
{
	struct i2c_msg msg = {
		.addr = priv->client->addr,
		.flags = I2C_M_RD,
		.buf = buf,
		.len = len,
	};
	int ret = i2c_transfer(priv->client->adapter, &msg, 1);

	if (ret == 1)
		return 0;

	dev_warn(&priv->client->dev, "i2c rd failed=%d len=%d\n", ret, len);
	return -EREMOTEIO;
}

static int avl6381_wr_reg(struct avl6381_priv *priv, u32 addr, u32 data,
			  int size)
{
	u8 buf[3 + 4];
	u8 *p = buf;
	int i;

	*(p++) = (u8)(addr >> 16);
	*(p++) = (u8)(addr >> 8);
	*(p++) = (u8)(addr);
	for (i = size - 1; i >= 0; i--)
		*(p++) = (u8)(data >> (8 * i));

	return avl6381_i2c_wr(priv, buf, 3 + size);
}

static int avl6381_rd_reg(struct avl6381_priv *priv, u32 addr, u32 *data,
			  int size)
{
	u8 addrbuf[3];
	u8 buf[4];
	int ret, i;

	addrbuf[0] = (u8)(addr >> 16);
	addrbuf[1] = (u8)(addr >> 8);
	addrbuf[2] = (u8)(addr);

	ret = avl6381_i2c_wr(priv, addrbuf, 3);
	if (ret)
		return ret;

	ret = avl6381_i2c_rd(priv, buf, size);
	if (ret)
		return ret;

	*data = 0;
	for (i = 0; i < size; i++)
		*data |= (u32)buf[i] << (8 * (size - 1 - i));

	return 0;
}

#define AVL6381_WR_REG8(_p, _a, _v)  avl6381_wr_reg(_p, _a, _v, 1)
#define AVL6381_WR_REG16(_p, _a, _v) avl6381_wr_reg(_p, _a, _v, 2)
#define AVL6381_WR_REG32(_p, _a, _v) avl6381_wr_reg(_p, _a, _v, 4)
#define AVL6381_RD_REG8(_p, _a, _v)  avl6381_rd_reg(_p, _a, _v, 1)
#define AVL6381_RD_REG16(_p, _a, _v) avl6381_rd_reg(_p, _a, _v, 2)
#define AVL6381_RD_REG32(_p, _a, _v) avl6381_rd_reg(_p, _a, _v, 4)

/*
 * RxOP mailbox: poll status until idle, then issue command in <31:24>.
 */
static int avl6381_rxop_busy(struct avl6381_priv *priv)
{
	u32 val = 0;

	if (AVL6381_RD_REG32(priv, AVL6381_REG_RXOP_STATUS, &val))
		return -EIO;
	return val ? 1 : 0;
}

static int avl6381_rxop_wait_idle(struct avl6381_priv *priv, int retries,
				  int delay_ms)
{
	int ret;

	for (; retries > 0; retries--) {
		ret = avl6381_rxop_busy(priv);
		if (ret < 0)
			return ret;
		if (ret == 0)
			return 0;
		fsleep(delay_ms * USEC_PER_MSEC);
	}
	return -ETIMEDOUT;
}

static int avl6381_send_rxop(struct avl6381_priv *priv, u8 cmd)
{
	int ret = avl6381_rxop_wait_idle(priv, 42, 10);

	if (ret)
		return ret;

	return AVL6381_WR_REG32(priv, AVL6381_REG_RXOP_STATUS,
				(u32)cmd << 24);
}

static int avl6381_core_reset(struct avl6381_priv *priv)
{
	int ret = AVL6381_WR_REG32(priv, AVL6381_REG_CORE_RESET, 0);

	usleep_range(10000, 11000);
	ret |= AVL6381_WR_REG32(priv, AVL6381_REG_CORE_RESET, 1);
	return ret;
}

static int avl6381_chip_ready(struct avl6381_priv *priv)
{
	u32 ready = 0, magic = 0;

	if (AVL6381_RD_REG32(priv, AVL6381_REG_CHIP_READY, &ready))
		return -EIO;
	if (AVL6381_RD_REG32(priv, AVL6381_REG_READY_MAGIC, &magic))
		return -EIO;

	if (ready == 1 || magic != AVL6381_READY_MAGIC_VALUE)
		return 1;	/* not ready yet */
	return 0;
}

static int avl6381_wait_ready(struct avl6381_priv *priv, int retries)
{
	int ret;

	for (; retries > 0; retries--) {
		ret = avl6381_chip_ready(priv);
		if (ret < 0)
			return ret;
		if (ret == 0)
			return 0;
		msleep(20);
	}
	return -ETIMEDOUT;
}

/* Core PLL configuration. */
static int avl6381_set_pll(struct avl6381_priv *priv, const u8 *pll)
{
	int ret;

	ret = AVL6381_WR_REG32(priv, 0x1000C0, pll[4] - 1);
	ret |= AVL6381_WR_REG32(priv, 0x1000C4, pll[5] - 1);
	ret |= AVL6381_WR_REG32(priv, 0x1000D4, pll[6]);
	ret |= AVL6381_WR_REG32(priv, 0x1000C8, pll[7] - 1);
	ret |= AVL6381_WR_REG32(priv, 0x100080, pll[12] - 1);
	ret |= AVL6381_WR_REG32(priv, 0x100084, pll[13] - 1);
	ret |= AVL6381_WR_REG32(priv, 0x100094, pll[14]);
	ret |= AVL6381_WR_REG32(priv, 0x100088, pll[15] - 1);
	ret |= AVL6381_WR_REG32(priv, 0x10008C, pll[16] - 1);
	ret |= AVL6381_WR_REG32(priv, 0x100090, pll[17] - 1);
	ret |= AVL6381_WR_REG32(priv, 0x100000, 0x00);
	ret |= AVL6381_WR_REG32(priv, 0x100000, 0x01);
	usleep_range(5000, 6000);
	ret |= AVL6381_WR_REG32(priv, 0x100018,
		pll[20] | pll[21] << 8 | pll[22] << 16 | (u32)pll[23] << 24);
	ret |= AVL6381_WR_REG32(priv, 0x10001C,
		pll[24] | pll[25] << 8 | pll[26] << 16 | (u32)pll[27] << 24);
	ret |= AVL6381_WR_REG32(priv, 0x100010, 0x01);
	ret |= AVL6381_WR_REG32(priv, 0x100008, 0x01);
	ret |= AVL6381_WR_REG32(priv, 0x100008, 0x00);

	return ret;
}

/*
 * Apply the DTMB firmware blob: TLV stream of
 * [u32 len BE][u8 addr[3]][payload] blocks starting at offset 4.
 */
#define AVL6381_FW_CHUNK_PAYLOAD	47

static int avl6381_load_freeze(struct avl6381_priv *priv)
{
	const u8 *d = avl6381_freeze_dtmb;
	size_t i = 4;
	int ret = 0;

	while (i + 4 <= sizeof(avl6381_freeze_dtmb)) {
		u32 len = ((u32)d[i] << 24) | (d[i + 1] << 16) |
			  (d[i + 2] << 8) | d[i + 3];
		const u8 *blk;
		u32 addr, pos = 3;

		if (!len)
			break;
		if (sizeof(avl6381_freeze_dtmb) - i < 8 ||
		    len > sizeof(avl6381_freeze_dtmb) - i - 8)
			return -EPROTO;
		blk = &d[i + 5];
		addr = (blk[0] << 16) | (blk[1] << 8) | blk[2];

		while (pos < len + 3) {
			u32 this_len = min_t(u32, len + 3 - pos,
					     AVL6381_FW_CHUNK_PAYLOAD);
			u8 buf[3 + AVL6381_FW_CHUNK_PAYLOAD];

			buf[0] = (u8)(addr >> 16);
			buf[1] = (u8)(addr >> 8);
			buf[2] = (u8)addr;
			memcpy(&buf[3], &blk[pos], this_len);

			ret = avl6381_i2c_wr(priv, buf, 3 + this_len);
			if (ret)
				return ret;

			pos += this_len;
			addr += this_len;
		}

		i += len + 8;
	}

	/* interrupt vector table + release */
	ret |= AVL6381_WR_REG32(priv, 0x000228, 0x00280000);
	ret |= AVL6381_WR_REG32(priv, 0x00022C, 0x002D0008);
	ret |= AVL6381_WR_REG32(priv, 0x000230, 0x0028CB00);
	ret |= AVL6381_WR_REG32(priv, 0x000234, 0x002F2C08);
	ret |= AVL6381_WR_REG8(priv, 0x000225, 0x01);
	ret |= AVL6381_WR_REG8(priv, 0x000226, 0x01);
	ret |= AVL6381_WR_REG16(priv, 0x2D0000, 0x0001);
	ret |= AVL6381_WR_REG16(priv, 0x2D0002, 0x0000);
	ret |= AVL6381_WR_REG32(priv, AVL6381_REG_READY_MAGIC, 0);
	ret |= AVL6381_WR_REG32(priv, AVL6381_REG_CHIP_READY, 0);

	return ret;
}

static int avl6381_ibase_init(struct avl6381_priv *priv, const u8 *pll)
{
	int ret = AVL6381_WR_REG32(priv, AVL6381_REG_CHIP_READY, 1);

	ret |= avl6381_set_pll(priv, pll);
	ret |= avl6381_core_reset(priv);
	if (!ret)
		ret = avl6381_load_freeze(priv);
	return ret;
}

static int avl6381_dtmb_init_rx(struct avl6381_priv *priv)
{
	int ret = avl6381_send_rxop(priv, AVL6381_RXOP_INIT_RX);

	if (ret)
		return ret;

	ret = AVL6381_WR_REG32(priv, 0x000338, 0x11E1A300);
	ret |= AVL6381_WR_REG32(priv, 0x000384, 0x0A037A00);
	ret |= AVL6381_WR_REG32(priv, 0x00033C, 0x04C4B400);
	ret |= AVL6381_WR_REG32(priv, 0x000304, 0x016E3600);
	ret |= AVL6381_WR_REG8(priv, 0x000321, 0x01);
	ret |= AVL6381_WR_REG8(priv, 0x000323, 0x01);
	ret |= AVL6381_WR_REG8(priv, 0x000319, 0x00);
	ret |= AVL6381_WR_REG8(priv, 0x00032B, 0x01);
	ret |= AVL6381_WR_REG8(priv, 0x0000A6, 0x00);
	/* spectrum polarity: normal */
	ret |= AVL6381_WR_REG8(priv, 0x000322, 0x00);
	ret |= AVL6381_WR_REG32(priv, 0x000324, 0x004C4B40);

	return ret;
}

static int avl6381_dtmb_init_adc(struct avl6381_priv *priv)
{
	int ret = AVL6381_WR_REG8(priv, 0x000320, 0x00);

	ret |= AVL6381_WR_REG8(priv, 0x0004D7, 0x00);
	ret |= avl6381_send_rxop(priv, AVL6381_RXOP_INIT_ADC);
	return ret;
}

static int avl6381_dvbc_init_rx(struct avl6381_priv *priv)
{
	int ret = avl6381_send_rxop(priv, AVL6381_RXOP_INIT_RX);

	if (ret)
		return ret;

	ret = AVL6381_WR_REG32(priv, 0x000560, 0x0d59f800);
	ret |= AVL6381_WR_REG32(priv, 0x0005a8, 0x0a037a00);
	ret |= AVL6381_WR_REG32(priv, 0x00055c, 0x016e3600);
	ret |= AVL6381_WR_REG32(priv, 0x000580, 0x004c4b40);
	ret |= AVL6381_WR_REG32(priv, 0x000558, 0x0068e778);
	ret |= AVL6381_WR_REG8(priv, 0x00057d, 0x01);
	ret |= AVL6381_WR_REG8(priv, 0x00057f, 0x01);
	ret |= AVL6381_WR_REG8(priv, 0x00057c, 0x00);
	ret |= AVL6381_WR_REG8(priv, 0x000747, 0x00);

	return ret;
}

static int avl6381_init_sdram(struct avl6381_priv *priv)
{
	int ret = AVL6381_WR_REG32(priv, 0x000210, 0x00070A00);

	ret |= AVL6381_WR_REG32(priv, 0x000214, 0x05060600);
	ret |= AVL6381_WR_REG32(priv, 0x000218, 0x03010301);
	ret |= avl6381_send_rxop(priv, AVL6381_RXOP_INIT_SDRAM);
	return ret;
}

static int avl6381_irx_init(struct avl6381_priv *priv,
			    enum fe_delivery_system ds)
{
	int ret;

	if (ds == SYS_DVBC_ANNEX_A) {
		ret = avl6381_dvbc_init_rx(priv);
	} else {
		ret = avl6381_dtmb_init_rx(priv);
		if (!ret)
			ret = avl6381_dtmb_init_adc(priv);
	}
	if (!ret)
		ret = avl6381_init_sdram(priv);
	return ret;
}

/* TS output configuration feeding the IT930x bridge. */
static int avl6381_mpeg_config(struct avl6381_priv *priv,
			       enum fe_delivery_system ds)
{
	bool dvbc = (ds == SYS_DVBC_ANNEX_A);
	int ret;

	if (dvbc) {
		ret = AVL6381_WR_REG8(priv, 0x00056E, 0);
		ret |= AVL6381_WR_REG8(priv, 0x00056F, 1);
		ret |= AVL6381_WR_REG8(priv, 0x00056D, 0);
		ret |= AVL6381_WR_REG8(priv, 0x00056C, 0);
		ret |= AVL6381_WR_REG8(priv, 0x00074E, 0);
		ret |= AVL6381_WR_REG8(priv, 0x000578, 1);
		ret |= AVL6381_WR_REG8(priv, 0x000570, 0);
		ret |= AVL6381_WR_REG8(priv, 0x00074F, 0);
		ret |= AVL6381_WR_REG8(priv, 0x000573, 0);
	} else {
		ret = AVL6381_WR_REG8(priv, 0x000352, 1);
		ret |= AVL6381_WR_REG8(priv, 0x000353, 1);
		ret |= AVL6381_WR_REG8(priv, 0x000351, 0);
		ret |= AVL6381_WR_REG8(priv, 0x000350, 0);
		ret |= AVL6381_WR_REG8(priv, 0x0004E6, 0);
		ret |= AVL6381_WR_REG8(priv, 0x000378, 1);
		ret |= AVL6381_WR_REG8(priv, 0x000354, 0);
		ret |= AVL6381_WR_REG8(priv, 0x0004E7, 0);
		ret |= AVL6381_WR_REG8(priv, 0x000357, 0);
	}

	/*
	 * DTMB-only: disable MPEG continuous mode (same reg both paths
	 * in the vendor sequence)
	 */
	ret |= AVL6381_WR_REG8(priv, 0x00038B, 0);
	ret |= AVL6381_WR_REG32(priv, 0x108030, 0x00000FFF);
	return ret;
}

static int avl6381_tuner_i2c_init(struct avl6381_priv *priv,
				  enum fe_delivery_system ds)
{
	u32 val;
	int ret;

	ret = AVL6381_WR_REG32(priv, 0x118000, 0x01);
	/* close gate: tuner behind demod I2C gateway */
	ret |= AVL6381_WR_REG32(priv, 0x11801C, 0x00000006);
	if (!ret)
		ret = AVL6381_RD_REG32(priv, 0x118004, &val);
	if (!ret)
		ret = AVL6381_WR_REG32(priv, 0x118004, val & ~1U);
	/*
	 * gateway bit-repeat clock divider, vendor values:
	 * DTMB 0x34, DVB-C 0x27.
	 */
	if (ds == SYS_DVBC_ANNEX_A)
		ret |= AVL6381_WR_REG32(priv, 0x118018, 0x27);
	else
		ret |= AVL6381_WR_REG32(priv, 0x118018, 0x34);
	ret |= AVL6381_WR_REG32(priv, 0x118000, 0x00);

	return ret;
}

static int avl6381_agc_config(struct avl6381_priv *priv,
			      enum fe_delivery_system ds)
{
	int ret;

	if (ds == SYS_DVBC_ANNEX_A)
		ret = AVL6381_WR_REG8(priv, 0x00059F, 0);
	else
		ret = AVL6381_WR_REG8(priv, 0x00030B, 0);

	ret |= AVL6381_WR_REG32(priv, 0x108034, 0x00000001);
	return ret;
}

static int avl6381_reset_per(struct avl6381_priv *priv,
			     enum fe_delivery_system ds)
{
	u32 val;
	int ret;

	ret = AVL6381_RD_REG32(priv, 0x149104, &val);
	if (ret)
		return ret;
	ret |= AVL6381_WR_REG32(priv, 0x149104, val | 0x01);

	if (ds == SYS_DVBC_ANNEX_A)
		ret |= AVL6381_WR_REG16(priv, 0x0001A2, 0x0000);
	else
		ret |= AVL6381_WR_REG8(priv, 0x0000A5, 0x00);

	if (ret)
		return ret;
	ret = AVL6381_RD_REG32(priv, 0x149104, &val);
	if (ret)
		return ret;
	ret = AVL6381_WR_REG32(priv, 0x149104, val | 0x08);
	if (ret)
		return ret;
	ret = AVL6381_RD_REG32(priv, 0x149104, &val);
	if (ret)
		return ret;
	ret = AVL6381_WR_REG32(priv, 0x149104, val | 0x01);
	if (ret)
		return ret;
	ret = AVL6381_RD_REG32(priv, 0x149104, &val);
	if (ret)
		return ret;
	ret = AVL6381_WR_REG32(priv, 0x149104, val & 0xFFFFFFFE);

	return ret;
}

static int avl6381_init_per(struct avl6381_priv *priv,
			    enum fe_delivery_system ds)
{
	u32 val;
	int ret;

	ret = AVL6381_WR_REG32(priv, 0x149160, 0x00000001);
	ret |= AVL6381_WR_REG32(priv, 0x14912C, 0x00000001);
	ret |= AVL6381_WR_REG32(priv, 0x149130, 0x0A037A00);
	ret |= AVL6381_WR_REG32(priv, 0x149134, 0x00000000);
	ret |= AVL6381_WR_REG32(priv, 0x149138, 0x00000000);
	ret |= AVL6381_WR_REG32(priv, 0x14913C, 0x00000000);

	if (ret)
		return ret;
	ret = AVL6381_RD_REG32(priv, 0x149160, &val);
	if (ret)
		return ret;
	if (val == 1) {
		ret |= AVL6381_WR_REG32(priv, 0x149128, 0);
		ret |= AVL6381_WR_REG32(priv, 0x149128, 1);
		ret |= AVL6381_WR_REG32(priv, 0x149128, 0);
	}

	ret |= avl6381_reset_per(priv, ds);
	return ret;
}

/* Full post-load RX/MPEG/AGC/PER configuration sequence */
static int avl6381_rx_config(struct avl6381_priv *priv,
			     enum fe_delivery_system ds)
{
	int ret = avl6381_irx_init(priv, ds);

	if (ds != SYS_DVBC_ANNEX_A)
		ret |= AVL6381_WR_REG32(priv, AVL6381_DTMB_REG_SYMRATE,
					AVL6381_DTMB_SYM_RATE_HZ);

	ret |= avl6381_mpeg_config(priv, ds);
	ret |= avl6381_tuner_i2c_init(priv, ds);
	ret |= avl6381_agc_config(priv, ds);
	ret |= avl6381_init_per(priv, ds);
	ret |= AVL6381_WR_REG32(priv, 0x0006F4, 0x0000000A);

	return ret;
}

static int avl6381_initialize(struct avl6381_priv *priv)
{
	u32 chipid = 0;
	int ret;

	if (priv->inited)
		return 0;

	ret = AVL6381_RD_REG32(priv, AVL6381_REG_CHIP_ID, &chipid);
	if (ret)
		return ret;

	priv->delivery_system = SYS_DVBC_ANNEX_A;

	ret = avl6381_ibase_init(priv, avl6381_pll_dvbc);
	if (ret)
		return ret;

	msleep(20);
	ret = avl6381_wait_ready(priv, 20);
	if (ret)
		return ret;

	ret = avl6381_rx_config(priv, SYS_DVBC_ANNEX_A);
	if (!ret)
		priv->inited = true;

	return ret;
}

static int avl6381_halt(struct avl6381_priv *priv, enum fe_delivery_system ds)
{
	u32 val;
	u32 reg = ds == SYS_DVBC_ANNEX_A ? AVL6381_DVBC_REG_HALT :
		AVL6381_DTMB_REG_HALT;
	int ret = avl6381_send_rxop(priv, AVL6381_RXOP_HALT);

	if (ret)
		return ret;
	usleep_range(2000, 3000);
	ret = AVL6381_RD_REG32(priv, reg, &val);
	if (ret)
		return ret;
	return AVL6381_WR_REG32(priv, reg, val & 0xFE);
}

static int avl6381_running_level(struct avl6381_priv *priv,
				 enum fe_delivery_system ds)
{
	u32 val = 0;
	int ret;

	if (ds == SYS_DVBC_ANNEX_A) {
		ret = AVL6381_RD_REG32(priv, AVL6381_DVBC_REG_RUNNING, &val);
		if (ret)
			return ret;
		return val ? 2 : 0;
	}

	ret = AVL6381_RD_REG8(priv, AVL6381_DTMB_REG_RUNNING, &val);
	if (ret)
		return ret;
	return val ? 2 : 0;
}

static int avl6381_autolock_channel(struct avl6381_priv *priv,
				    enum fe_delivery_system ds)
{
	int ret;

	if (ds == SYS_DVBC_ANNEX_A)
		return avl6381_send_rxop(priv, AVL6381_RXOP_DVBC_AUTOLOCK);

	ret = AVL6381_WR_REG8(priv, 0x00021F, 0x01);

	ret |= AVL6381_WR_REG8(priv, 0x0000AD, 0x00);
	ret |= AVL6381_WR_REG32(priv, 0x00010C, 0x00000000);
	ret |= avl6381_send_rxop(priv, AVL6381_RXOP_AUTOLOCK);
	return ret;
}

static int avl6381_stop_rx(struct avl6381_priv *priv,
			   enum fe_delivery_system ds)
{
	int ret, i, level;

	ret = avl6381_halt(priv, ds);
	if (ret)
		return ret;

	/* wait for RX to stop */
	for (i = 0; i < 10; i++) {
		level = avl6381_running_level(priv, ds);
		if (level < 0)
			return level;
		if (level == 0)
			break;
		usleep_range(10000, 11000);
	}
	if (level != 0)
		return -ETIMEDOUT;
	return 0;
}

static int avl6381_autolock(struct avl6381_priv *priv,
			    enum fe_delivery_system ds)
{
	int ret, i, level;

	ret = avl6381_autolock_channel(priv, ds);
	if (ret)
		return ret;

	/* wait for RX to start */
	for (i = 0; i < 10; i++) {
		level = avl6381_running_level(priv, ds);
		if (level < 0)
			return level;
		if (level != 0)
			return 0;
		usleep_range(10000, 11000);
	}
	return -ETIMEDOUT;
}

/*
 * Switch the demod between DTMB and DVBC RX chains. This reloads the core
 * PLL and re-runs the full RX configuration (vendor sequence).
 */
static int avl6381_set_mode(struct avl6381_priv *priv, enum avl6381_mode mode)
{
	enum fe_delivery_system ds = (mode == AVL6381_MODE_DTMB) ?
		SYS_DVBT2 : SYS_DVBC_ANNEX_A;
	u32 val;
	int ret;

	ret = AVL6381_RD_REG32(priv, 0x000200, &val);
	if (ret)
		return ret;
	if (val == (u32)mode) {
		priv->delivery_system = ds;
		return 0;
	}

	ret = avl6381_halt(priv, ds);
	if (ret)
		return ret;

	ret = avl6381_rxop_wait_idle(priv, 22, 20);
	if (ret)
		return ret;

	/* A partial switch must not be mistaken for a configured RX chain. */
	priv->inited = false;
	ret = AVL6381_WR_REG32(priv, 0x110084, 0);
	usleep_range(10000, 11000);
	ret |= AVL6381_WR_REG32(priv, 0x110084, 1);
	ret |= AVL6381_WR_REG32(priv, AVL6381_REG_READY_MAGIC, 0);

	ret |= avl6381_send_rxop(priv, AVL6381_RXOP_MODE_SWITCH);
	if (ret)
		return ret;

	ret = avl6381_rxop_wait_idle(priv, 202, 20);
	if (ret)
		return ret;

	ret = avl6381_wait_ready(priv, 22);
	if (ret)
		return ret;

	ret = AVL6381_WR_REG32(priv, AVL6381_REG_CHIP_READY, 1);
	ret |= avl6381_set_pll(priv, mode == AVL6381_MODE_DTMB ?
			       avl6381_pll_dtmb : avl6381_pll_dvbc);
	msleep(20);
	ret |= AVL6381_WR_REG32(priv, AVL6381_REG_CHIP_READY, 0);

	if (ret)
		return ret;

	ret = avl6381_wait_ready(priv, 200);
	if (ret)
		return ret;

	ret = avl6381_rx_config(priv, ds);
	if (!ret) {
		priv->delivery_system = ds;
		priv->inited = true;
	}

	return ret;
}

static int avl6381_lock_status(struct avl6381_priv *priv, bool *locked)
{
	u32 val = 0;
	int ret;

	if (priv->delivery_system == SYS_DVBC_ANNEX_A) {
		ret = AVL6381_RD_REG32(priv, AVL6381_DVBC_REG_LOCK, &val);
		if (ret)
			return ret;
		*locked = (val == 21);
		return 0;
	}

	ret = AVL6381_RD_REG8(priv, AVL6381_DTMB_REG_LOCK, &val);
	if (ret)
		return ret;
	*locked = (val != 0);
	return 0;
}

static int avl6381_snr_raw(struct avl6381_priv *priv, u32 *snr)
{
	if (priv->delivery_system == SYS_DVBC_ANNEX_A) {
		u32 trig = 0;
		int ret = AVL6381_RD_REG32(priv, AVL6381_DVBC_REG_SNR_TRIG,
					   &trig);

		if (ret)
			return ret;
		if (trig)
			return -EAGAIN;

		ret = AVL6381_RD_REG16(priv, AVL6381_DVBC_REG_SNR, snr);
		ret |= AVL6381_WR_REG32(priv, AVL6381_DVBC_REG_SNR_TRIG, 1);
		msleep(50);
		return ret;
	}

	return AVL6381_RD_REG16(priv, AVL6381_DTMB_REG_SNR, snr);
}

/*
 * I2C gate: the MXL603 sits behind the demod I2C gateway (0x11801C).
 * Preserve the community driver's five writes per transition.
 */
static int avl6381_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct avl6381_priv *priv = fe->demodulator_priv;
	int ret = 0, i;

	for (i = 0; i < 5; i++)
		ret |= AVL6381_WR_REG32(priv, 0x11801C,
					enable ? 0x00000007 : 0x00000006);
	return ret;
}

static int avl6381_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct avl6381_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	bool locked = false;
	u32 snr, per_err, per_cnt;
	u16 strength;
	int ret;

	mutex_lock(&priv->mutex);
	*status = 0;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->pre_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->pre_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	if (!priv->inited) {
		ret = 0;
		goto out;
	}

	ret = avl6381_lock_status(priv, &locked);
	if (ret)
		goto out;
	if (fe->ops.tuner_ops.get_rf_strength &&
	    !fe->ops.tuner_ops.get_rf_strength(fe, &strength)) {
		/* MXL603 reports attenuation as a signed hundredth-dBm value. */
		c->strength.stat[0].scale = FE_SCALE_RELATIVE;
		c->strength.stat[0].uvalue =
			avl6381_relative_scale(-(s64)(s16)strength * 10,
					       AVL6381_SIGNAL_MIN_MDB,
					       AVL6381_SIGNAL_MAX_MDB);
	}
	if (!locked)
		goto out;

	*status = FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
		  FE_HAS_SYNC | FE_HAS_LOCK;
	if (!avl6381_snr_raw(priv, &snr)) {
		c->cnr.stat[0].scale = FE_SCALE_RELATIVE;
		c->cnr.stat[0].uvalue =
			avl6381_relative_scale((s64)snr * 10,
					       AVL6381_CNR_MIN_MDB,
					       AVL6381_CNR_MAX_MDB);
	}
	if (!AVL6381_RD_REG32(priv, AVL6381_REG_PER_ERR, &per_err) &&
	    !AVL6381_RD_REG32(priv, AVL6381_REG_PER_CNT, &per_cnt) && per_cnt) {
		c->block_error.stat[0].scale = FE_SCALE_COUNTER;
		c->block_error.stat[0].uvalue = per_err;
		c->block_count.stat[0].scale = FE_SCALE_COUNTER;
		c->block_count.stat[0].uvalue = per_cnt;
	}
out:
	mutex_unlock(&priv->mutex);
	return ret;
}

static int avl6381_set_frontend(struct dvb_frontend *fe)
{
	struct avl6381_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;

	mutex_lock(&priv->mutex);

	ret = avl6381_initialize(priv);
	if (ret)
		goto out;

	if (priv->config->tuner_select_input)
		ret = priv->config->tuner_select_input(fe, c->delivery_system);
	else
		ret = 0;
	if (ret)
		goto out;

	switch (c->delivery_system) {
	case SYS_DVBT:
	case SYS_DVBT2:
		ret = avl6381_set_mode(priv, AVL6381_MODE_DTMB);
		break;
	case SYS_DVBC_ANNEX_A:
		ret = avl6381_set_mode(priv, AVL6381_MODE_DVBC);
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	if (ret)
		goto out;

	/* Stop acquisition before changing the tuner's shared clock and IF. */
	ret = avl6381_stop_rx(priv, c->delivery_system);
	if (ret)
		goto out;

	if (fe->ops.tuner_ops.set_params) {
		ret = fe->ops.tuner_ops.set_params(fe);
		if (ret)
			goto out;
	}

	if (c->delivery_system != SYS_DVBC_ANNEX_A)
		ret = AVL6381_WR_REG32(priv, AVL6381_DTMB_REG_SYMRATE,
				       AVL6381_DTMB_SYM_RATE_HZ);
	if (ret)
		goto out;

	ret = avl6381_autolock(priv, c->delivery_system);
	if (!ret)
		priv->delivery_system = c->delivery_system;

out:
	mutex_unlock(&priv->mutex);
	return ret;
}

static enum dvbfe_algo avl6381_get_frontend_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}

static int avl6381_tune(struct dvb_frontend *fe, bool re_tune,
			unsigned int mode_flags, unsigned int *delay,
			enum fe_status *status)
{
	int ret;

	*delay = HZ / 5;
	*status = 0;
	/* Firmware acquires and tracks the signal; do not restart it on polls. */
	if (re_tune) {
		ret = avl6381_set_frontend(fe);
		if (ret)
			return ret;
	}
	return avl6381_read_status(fe, status);
}

static int avl6381_init(struct dvb_frontend *fe)
{
	struct avl6381_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;

	c->strength.len = 1;
	c->cnr.len = 1;
	c->pre_bit_error.len = 1;
	c->pre_bit_count.len = 1;
	c->block_error.len = 1;
	c->block_count.len = 1;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->pre_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->pre_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	mutex_lock(&priv->mutex);
	ret = avl6381_initialize(priv);
	mutex_unlock(&priv->mutex);
	return ret;
}

static void avl6381_release(struct dvb_frontend *fe)
{
	struct avl6381_priv *priv = fe->demodulator_priv;

	i2c_unregister_device(priv->client);
	mutex_destroy(&priv->mutex);
	kfree(priv);
}

static const struct dvb_frontend_ops avl6381_ops = {
	.delsys = { SYS_DVBT, SYS_DVBT2, SYS_DVBC_ANNEX_A },
	.info = {
		.name = "Availink AVL6381 DTMB/DVB-C",
		.frequency_min_hz =  42 * MHz,
		.frequency_max_hz = 858 * MHz,
		.frequency_stepsize_hz = 0,
		.frequency_tolerance_hz = 0,
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 45000000,
		.caps = FE_CAN_INVERSION_AUTO |
			FE_CAN_FEC_AUTO |
			FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO |
			FE_CAN_HIERARCHY_AUTO |
			FE_CAN_RECOVER |
			FE_CAN_MUTE_TS,
	},
	.release = avl6381_release,
	.init = avl6381_init,
	.i2c_gate_ctrl = avl6381_i2c_gate_ctrl,
	.read_status = avl6381_read_status,
	.set_frontend = avl6381_set_frontend,
	.get_frontend_algo = avl6381_get_frontend_algo,
	.tune = avl6381_tune,
};

static int avl6381_probe(struct i2c_client *client)
{
	const struct avl6381_config *config = dev_get_platdata(&client->dev);
	struct avl6381_priv *priv;
	u32 fid = 0;
	int ret;

	if (!config || !config->fe)
		return -EINVAL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	priv->config = config;
	priv->delivery_system = SYS_DVBC_ANNEX_A;
	mutex_init(&priv->mutex);
	i2c_set_clientdata(client, priv);

	ret = AVL6381_RD_REG32(priv, AVL6381_REG_FAMILY_ID, &fid);
	if (ret || fid != AVL6381_FAMILY_ID_6381) {
		dev_err(&client->dev, "unsupported chip (family id 0x%08x)\n",
			fid);
		mutex_destroy(&priv->mutex);
		kfree(priv);
		return -ENODEV;
	}

	memcpy(&priv->fe.ops, &avl6381_ops, sizeof(priv->fe.ops));
	priv->fe.demodulator_priv = priv;

	/* The tuner gateway is usable only after firmware/RX initialization. */
	ret = avl6381_initialize(priv);
	if (ret) {
		dev_err(&client->dev, "initialization failed=%d\n", ret);
		mutex_destroy(&priv->mutex);
		kfree(priv);
		return ret;
	}

	*config->fe = &priv->fe;

	dev_info(&client->dev, "Availink AVL6381 found (i2c 0x%02x)\n",
		 client->addr);

	return 0;
}

static void avl6381_remove(struct i2c_client *client)
{
	struct avl6381_priv *priv = i2c_get_clientdata(client);

	*priv->config->fe = NULL;
}

/*
 * Bridge driver entry: instantiate the demod I2C client on the given
 * adapter. The caller-owned config must outlive the frontend; probe
 * publishes the created frontend through config->fe.
 */
struct dvb_frontend *avl6381_attach(const struct avl6381_config *config,
				    struct i2c_adapter *i2c)
{
	struct i2c_board_info board_info = {
		I2C_BOARD_INFO("avl6381", config->i2c_address),
		.platform_data = (void *)config,
	};
	struct i2c_client *client;

	*config->fe = NULL;
	client = i2c_new_client_device(i2c, &board_info);
	if (IS_ERR(client))
		return NULL;
	if (!*config->fe) {
		i2c_unregister_device(client);
		return NULL;
	}

	return *config->fe;
}
EXPORT_SYMBOL_GPL(avl6381_attach);

static const struct i2c_device_id avl6381_id_table[] = {
	{ "avl6381", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, avl6381_id_table);

static struct i2c_driver avl6381_driver = {
	.driver = {
		.name = "avl6381",
		/* The USB bridge owns the published frontend until .release(). */
		.suppress_bind_attrs = true,
	},
	.probe = avl6381_probe,
	.remove = avl6381_remove,
	.id_table = avl6381_id_table,
};
module_i2c_driver(avl6381_driver);

MODULE_DESCRIPTION("Availink AVL6381 DTMB/DVB-C demodulator driver");
MODULE_AUTHOR("Xiaodong Ni <nxiaodong520@gmail.com>");
MODULE_LICENSE("GPL");
