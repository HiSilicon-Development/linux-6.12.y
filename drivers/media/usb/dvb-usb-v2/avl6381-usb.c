// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * dvb-usb-v2 bridge driver for the runqida AVL6381 DTMB stick.
 *
 * Copyright (C) 2009 Antti Palosaari <crope@iki.fi>
 * Copyright (C) 2012 Antti Palosaari <crope@iki.fi>
 * Copyright (C) 2024 Xiaodong Ni <nxiaodong520@gmail.com>
 * Copyright (C) 2024 wwek
 *
 * The stick uses an ITE IT9303 USB controller (04b4:9306 / 048d:9306),
 * an AVL6381 demodulator and a MaxLinear MXL603 tuner on the IT9303
 * internal I2C bus 1. The demod is instantiated as an I2C client (avl6381
 * driver); the tuner is attached through mxl603_attach().
 *
 * Adapted from the community it930x driver and dvb-cn-usb's
 * dvb_usb_it930x.c. The mailbox and firmware download routines originate
 * in Antti Palosaari's af9035 driver.
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <media/dvbdev.h>
#include <media/dvb-usb-ids.h>

#include "dvb_usb.h"
#include "avl6381_priv.h"
#include "mxl603.h"
#define AVL6381_USB_I2C_ADDRESS    0x14
#define MXL603_USB_I2C_ADDRESS     0x60

/*
 * I2C speed register: reg = 1000000000 / (24.4 * 16 * speed).
 * 7 => ~366 kbps, the value the vendor firmware uses.
 */
#define I2C_SPEED_366K             7

#define AVL6381_USB_FIRMWARE       "dvb-usb-it9303-01.fw"

/* Max transfer size done by the I2C transfer functions */
#define MAX_XFER_SIZE              64

/* USB mailbox commands */
#define CMD_MEM_RD                 0x00
#define CMD_MEM_WR                 0x01
#define CMD_FW_DL                  0x21
#define CMD_FW_QUERYINFO           0x22
#define CMD_FW_BOOT                0x23
#define CMD_FW_DL_BEGIN            0x24
#define CMD_FW_DL_END              0x25
#define CMD_FW_SCATTER_WR          0x29
#define CMD_GENERIC_I2C_RD         0x2a
#define CMD_GENERIC_I2C_WR         0x2b

enum avl6381_usb_gpio {
	GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO7, GPIO8,
	GPIO9, GPIO10, GPIO11, GPIO12, GPIO13, GPIO14, GPIO15, GPIO16,
};

enum avl6381_usb_gpio_mode {
	GPIO_MODE_IN,
	GPIO_MODE_OUT,
};

enum avl6381_usb_gpio_enable {
	GPIO_DISABLE,
	GPIO_ENABLE,
};

enum avl6381_usb_gpio_level {
	GPIO_LOW,
	GPIO_HIGH,
};

struct usb_req {
	u8 cmd;
	u8 mbox;
	u8 wlen;
	u8 *wbuf;
	u8 rlen;
	u8 *rbuf;
};

struct state {
#define BUF_LEN 256
	u8 buf[BUF_LEN];
	u8 seq; /* packet sequence number */
	u8 prechip_version;
	u8 chip_version;
	u16 chip_type;
	struct avl6381_config avl6381_cfg;
};

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

/* MXL603 profile for this stick: 24 MHz shared XTAL (clk-out on), 5 MHz
 * IF with spectrum inversion, external (demod-loop) AGC. The 16 MHz
 * tuner-mode XTAL selector is what the vendor driver programs.
 */
static const struct mxl603_config mxl603_cfg = {
	.single_supply_3_3v = true,
	.xtal = {
		.xtal_freq_sel = 1, /* 24 MHz */
		.xtal_cap = 12,
		.clk_out_enable = true,
		.clk_out_div = false,
		.single_supply_3_3v = true,
		.xtal_sharing_mode = false,
	},
	.if_out = {
		.if_out_freq = 6, /* 5 MHz */
		.if_inversion = true,
		.gain_level = 11,
		.manual_freq_set = false,
		.manual_if_out_freq_khz = 5000,
	},
	.agc = {
		.agc_type = 1, /* external, closed loop through the demod */
		.set_point = 66,
		.agc_polarity_inversion = false,
	},
	.tuner_mode = {
		.if_out_freq_khz = 5000,
		.xtal_freq_sel = 0, /* 16 MHz, per vendor programming */
		.if_out_gain_level = 11,
	},
};

static u16 avl6381_usb_checksum(const u8 *buf, size_t len)
{
	size_t i;
	u16 checksum = 0;

	for (i = 1; i < len; i++) {
		if (i % 2)
			checksum += buf[i] << 8;
		else
			checksum += buf[i];
	}
	checksum = ~checksum;

	return checksum;
}

static int avl6381_usb_ctrl_msg(struct dvb_usb_device *d, struct usb_req *req)
{
#define REQ_HDR_LEN 4 /* send header size */
#define ACK_HDR_LEN 3 /* receive header size */
#define CHECKSUM_LEN 2
	struct state *state = d_to_priv(d);
	int ret, wlen, rlen;
	u16 checksum, tmp_checksum;

	mutex_lock(&d->usb_mutex);

	/* buffer overflow check */
	if (req->wlen > (BUF_LEN - REQ_HDR_LEN - CHECKSUM_LEN) ||
	    req->rlen > (BUF_LEN - ACK_HDR_LEN - CHECKSUM_LEN)) {
		dev_err(&d->udev->dev, "too much data wlen=%d rlen=%d\n",
			req->wlen, req->rlen);
		ret = -EINVAL;
		goto exit;
	}

	state->buf[0] = REQ_HDR_LEN + req->wlen + CHECKSUM_LEN - 1;
	state->buf[1] = req->mbox;
	state->buf[2] = req->cmd;
	state->buf[3] = state->seq++;
	if (req->wlen)
		memcpy(&state->buf[REQ_HDR_LEN], req->wbuf, req->wlen);

	wlen = REQ_HDR_LEN + req->wlen + CHECKSUM_LEN;
	rlen = ACK_HDR_LEN + req->rlen + CHECKSUM_LEN;

	/* calc and add checksum */
	checksum = avl6381_usb_checksum(state->buf, state->buf[0] - 1);
	state->buf[state->buf[0] - 1] = (checksum >> 8);
	state->buf[state->buf[0] - 0] = (checksum & 0xff);

	/* no ack for firmware download packets */
	if (req->cmd == CMD_FW_DL)
		rlen = 0;
	ret = dvb_usbv2_generic_rw_locked(d, state->buf, wlen, state->buf, rlen);
	if (ret)
		goto exit;

	if (req->cmd == CMD_FW_DL)
		goto exit;

	/* verify checksum */
	checksum = avl6381_usb_checksum(state->buf, rlen - 2);
	tmp_checksum = (state->buf[rlen - 2] << 8) | state->buf[rlen - 1];
	if (tmp_checksum != checksum) {
		dev_err(&d->udev->dev, "command=%02x checksum mismatch (%04x != %04x)\n",
			req->cmd, tmp_checksum, checksum);
		ret = -EIO;
		goto exit;
	}

	if (state->buf[2]) {
		dev_dbg(&d->udev->dev, "command=%02x failed fw error=%d\n",
			req->cmd, state->buf[2]);
		ret = -EREMOTEIO;
		goto exit;
	}

	/* read request, copy returned data to return buf */
	if (req->rlen)
		memcpy(req->rbuf, &state->buf[ACK_HDR_LEN], req->rlen);
exit:
	mutex_unlock(&d->usb_mutex);
	return ret;
}

/* write multiple registers (mbox selects the 64k register page) */
static int avl6381_usb_wr_regs(struct dvb_usb_device *d, u32 reg,
			       const u8 *val, int len)
{
	u8 wbuf[MAX_XFER_SIZE];
	u8 mbox = (reg >> 16) & 0xff;
	struct usb_req req = { CMD_MEM_WR, mbox, 6 + len, wbuf, 0, NULL };

	if (6 + len > sizeof(wbuf)) {
		dev_warn(&d->udev->dev, "reg wr: len=%d is too big!\n", len);
		return -EOPNOTSUPP;
	}

	wbuf[0] = len;
	wbuf[1] = 2;
	wbuf[2] = 0;
	wbuf[3] = 0;
	wbuf[4] = (reg >> 8) & 0xff;
	wbuf[5] = (reg >> 0) & 0xff;
	memcpy(&wbuf[6], val, len);

	return avl6381_usb_ctrl_msg(d, &req);
}

static int avl6381_usb_rd_regs(struct dvb_usb_device *d, u32 reg,
			       u8 *val, int len)
{
	u8 wbuf[] = { len, 2, 0, 0, (reg >> 8) & 0xff, reg & 0xff };
	u8 mbox = (reg >> 16) & 0xff;
	struct usb_req req = { CMD_MEM_RD, mbox, sizeof(wbuf), wbuf, len, val };

	return avl6381_usb_ctrl_msg(d, &req);
}

static int avl6381_usb_wr_reg(struct dvb_usb_device *d, u32 reg, u8 val)
{
	return avl6381_usb_wr_regs(d, reg, &val, 1);
}

static int avl6381_usb_rd_reg(struct dvb_usb_device *d, u32 reg, u8 *val)
{
	return avl6381_usb_rd_regs(d, reg, val, 1);
}

static int avl6381_usb_wr_reg_mask(struct dvb_usb_device *d, u32 reg, u8 val,
				   u8 mask)
{
	int ret;
	u8 tmp;

	/* no need for read if whole reg is written */
	if (mask != 0xff) {
		ret = avl6381_usb_rd_regs(d, reg, &tmp, 1);
		if (ret)
			return ret;

		val &= mask;
		tmp &= ~mask;
		val |= tmp;
	}

	return avl6381_usb_wr_regs(d, reg, &val, 1);
}

static int avl6381_usb_i2c_master_xfer(struct i2c_adapter *adap,
				       struct i2c_msg msg[], int num)
{
	struct dvb_usb_device *d = i2c_get_adapdata(adap);
	int ret;
	int i;

	for (i = 0; i < num; i++) {
		if (msg[i].addr > 0x7f ||
		    (msg[i].flags & ~(I2C_M_RD | I2C_M_DMA_SAFE)))
			return -EOPNOTSUPP;
	}
	if (num == 2 && msg[0].addr != msg[1].addr)
		return -EOPNOTSUPP;

	if (mutex_lock_interruptible(&d->i2c_mutex) < 0)
		return -EAGAIN;

	/*
	 * IT9303 generic I2C sub header is 3 bytes:
	 *   0: transfer data length (read length for reads)
	 *   1: I2C bus (1 = demod/tuner bus on this board)
	 *   2: I2C address << 1
	 * followed by the write payload.
	 */
	if (num == 2 && !(msg[0].flags & I2C_M_RD) &&
	    (msg[1].flags & I2C_M_RD)) {
		u8 buf[MAX_XFER_SIZE];
		struct usb_req req = { CMD_GENERIC_I2C_RD, 0,
				3 + msg[0].len, buf, msg[1].len, msg[1].buf };

		if (msg[0].len > MAX_XFER_SIZE - 3 ||
		    msg[1].len > MAX_XFER_SIZE) {
			ret = -EOPNOTSUPP;
		} else {
			req.mbox |= ((msg[0].addr & 0x80) >> 3);
			buf[0] = msg[1].len;
			buf[1] = 0x01; /* I2C bus */
			buf[2] = msg[0].addr << 1;
			memcpy(&buf[3], msg[0].buf, msg[0].len);
			ret = avl6381_usb_ctrl_msg(d, &req);
		}
	} else if (num == 1 && !(msg[0].flags & I2C_M_RD)) {
		u8 buf[MAX_XFER_SIZE];
		struct usb_req req = { CMD_GENERIC_I2C_WR, 0,
				3 + msg[0].len, buf, 0, NULL };

		if (msg[0].len > MAX_XFER_SIZE - 3) {
			ret = -EOPNOTSUPP;
		} else {
			req.mbox |= ((msg[0].addr & 0x80) >> 3);
			buf[0] = msg[0].len;
			buf[1] = 0x01; /* I2C bus */
			buf[2] = msg[0].addr << 1;
			memcpy(&buf[3], msg[0].buf, msg[0].len);
			ret = avl6381_usb_ctrl_msg(d, &req);
		}
	} else if (num == 1 && (msg[0].flags & I2C_M_RD)) {
		u8 buf[3];
		struct usb_req req = { CMD_GENERIC_I2C_RD, 0, sizeof(buf),
				buf, msg[0].len, msg[0].buf };

		if (msg[0].len > MAX_XFER_SIZE) {
			ret = -EOPNOTSUPP;
		} else {
			req.mbox |= ((msg[0].addr & 0x80) >> 3);
			buf[0] = msg[0].len;
			buf[1] = 0x01; /* I2C bus */
			buf[2] = msg[0].addr << 1;
			ret = avl6381_usb_ctrl_msg(d, &req);
		}
	} else {
		/*
		 * Supported transactions only:
		 * 1) write + read (repeated start)
		 * 2) write
		 * 3) read
		 */
		ret = -EOPNOTSUPP;
	}

	mutex_unlock(&d->i2c_mutex);

	if (ret < 0)
		return ret;

	return num;
}

static u32 avl6381_usb_i2c_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

static struct i2c_algorithm avl6381_usb_i2c_algo = {
	.master_xfer = avl6381_usb_i2c_master_xfer,
	.functionality = avl6381_usb_i2c_functionality,
};

static int avl6381_usb_set_gpio_mode(struct dvb_usb_device *d,
				     enum avl6381_usb_gpio gpio,
				     enum avl6381_usb_gpio_mode mode)
{
	/* GPIO mode registers (gpioh1 to gpioh16) */
	static const u32 regs[] = {
		0xd8b0, 0xd8b8, 0xd8b4, 0xd8c0, 0xd8bc, 0xd8c8, 0xd8c4,
		0xd8d0, 0xd8cc, 0xd8d8, 0xd8d4, 0xd8e0, 0xd8dc, 0xd8e4,
		0xd8e8, 0xd8ec,
	};

	return avl6381_usb_wr_reg(d, regs[gpio], mode);
}

static int avl6381_usb_enable_gpio(struct dvb_usb_device *d,
				   enum avl6381_usb_gpio gpio,
				   enum avl6381_usb_gpio_enable enable)
{
	/* GPIO enable/disable registers (gpioh1 to gpioh16) */
	static const u32 regs[] = {
		0xd8b1, 0xd8b9, 0xd8b5, 0xd8c1, 0xd8bd, 0xd8c9, 0xd8c5,
		0xd8d1, 0xd8cd, 0xd8d9, 0xd8d5, 0xd8e1, 0xd8dd, 0xd8e5,
		0xd8e9, 0xd8ed,
	};

	return avl6381_usb_wr_reg(d, regs[gpio],
				  enable == GPIO_ENABLE ? 1 : 0);
}

static int avl6381_usb_write_gpio(struct dvb_usb_device *d,
				  enum avl6381_usb_gpio gpio,
				  enum avl6381_usb_gpio_level level)
{
	/* GPIO output registers (gpioh1 to gpioh16) */
	static const u32 regs[] = {
		0xd8af, 0xd8b7, 0xd8b3, 0xd8bf, 0xd8bb, 0xd8c7, 0xd8c3,
		0xd8cf, 0xd8cb, 0xd8d7, 0xd8d3, 0xd8df, 0xd8db, 0xd8e3,
		0xd8e7, 0xd8eb,
	};

	return avl6381_usb_wr_reg(d, regs[gpio],
				  level == GPIO_HIGH ? 1 : 0);
}

/*
 * RF input select: GPIO2/GPIO3 steer the tuner front-end between the
 * terrestrial and the cable input.
 */
static int avl6381_usb_select_input(struct dvb_frontend *fe,
				    enum fe_delivery_system delivery_system)
{
	struct dvb_usb_device *d = fe_to_d(fe);
	int ret;

	ret = avl6381_usb_set_gpio_mode(d, GPIO2, GPIO_MODE_OUT);
	ret |= avl6381_usb_enable_gpio(d, GPIO2, GPIO_ENABLE);
	ret |= avl6381_usb_set_gpio_mode(d, GPIO3, GPIO_MODE_OUT);
	ret |= avl6381_usb_enable_gpio(d, GPIO3, GPIO_ENABLE);

	switch (delivery_system) {
	case SYS_DVBT:
	case SYS_DVBT2:
		ret |= avl6381_usb_write_gpio(d, GPIO2, GPIO_HIGH);
		ret |= avl6381_usb_write_gpio(d, GPIO3, GPIO_LOW);
		break;
	case SYS_DVBC_ANNEX_A:
		ret |= avl6381_usb_write_gpio(d, GPIO2, GPIO_LOW);
		ret |= avl6381_usb_write_gpio(d, GPIO3, GPIO_LOW);
		break;
	default:
		break;
	}

	return ret;
}

static int avl6381_usb_identify_state(struct dvb_usb_device *d,
				      const char **name)
{
	struct state *state = d_to_priv(d);
	int ret;
	u8 wbuf[1] = { 1 };
	u8 rbuf[4];
	struct usb_req req = { CMD_FW_QUERYINFO, 0, sizeof(wbuf), wbuf,
			sizeof(rbuf), rbuf };

	/* configure gpio1, reset & power slave demod */
	ret = avl6381_usb_set_gpio_mode(d, GPIO1, GPIO_MODE_OUT);
	ret |= avl6381_usb_enable_gpio(d, GPIO1, GPIO_ENABLE);
	ret |= avl6381_usb_write_gpio(d, GPIO1, GPIO_LOW);
	if (ret)
		goto err;

	msleep(20);

	ret = avl6381_usb_rd_regs(d, 0x1222, rbuf, 3);
	if (ret < 0)
		goto err;

	state->chip_version = rbuf[0];
	state->chip_type = rbuf[2] << 8 | rbuf[1] << 0;

	ret = avl6381_usb_rd_reg(d, 0x384f, &state->prechip_version);
	if (ret < 0)
		goto err;

	dev_info(&d->udev->dev, "prechip_version=%02x chip_version=%02x chip_type=%04x\n",
		 state->prechip_version, state->chip_version, state->chip_type);

	if (state->chip_type != 0x9306) {
		dev_err(&d->udev->dev, "unsupported chip type %04x\n",
			state->chip_type);
		ret = -ENODEV;
		goto err;
	}

	*name = AVL6381_USB_FIRMWARE;

	ret = avl6381_usb_ctrl_msg(d, &req);
	if (ret < 0)
		goto err;

	usleep_range(7000, 8000);
	ret = avl6381_usb_wr_reg_mask(d, 0xda05, 0x01, 0x01);
	ret |= avl6381_usb_write_gpio(d, GPIO1, GPIO_HIGH);

	/* mp2_sw_rst: reset the TS endpoint path */
	ret |= avl6381_usb_wr_reg(d, 0xda1d, 0x01);
	usleep_range(2000, 3000);
	ret |= avl6381_usb_wr_reg(d, 0xda1d, 0x00);

	usleep_range(8000, 9000);
	ret |= avl6381_usb_wr_reg(d, 0x4976, 0x00);
	ret |= avl6381_usb_wr_reg(d, 0x4bfb, 0x00);
	ret |= avl6381_usb_wr_reg(d, 0x4978, 0x00);
	ret |= avl6381_usb_wr_reg(d, 0x4977, 0x00);

	/* I2C master bus 1,3 clock speed 366k */
	ret |= avl6381_usb_wr_reg(d, 0xf103, I2C_SPEED_366K);
	if (ret)
		goto err;

	dev_info(&d->udev->dev, "reply=%*ph\n", 4, rbuf);
	if (rbuf[0] || rbuf[1] || rbuf[2] || rbuf[3])
		return WARM;

	return COLD;

err:
	dev_dbg(&d->udev->dev, "failed=%d\n", ret);
	return ret;
}

/* classic MCS-51 core download (fw blob starts with 0x01) */
static int avl6381_usb_download_firmware_old(struct dvb_usb_device *d,
					     const struct firmware *fw)
{
	int ret, j, len;
	size_t i;
	u8 wbuf[1];
	struct usb_req req = { 0, 0, 0, NULL, 0, NULL };
	struct usb_req req_fw_dl = { CMD_FW_DL, 0, 0, wbuf, 0, NULL };
	u8 hdr_core;
	u16 hdr_addr, hdr_data_len, hdr_checksum;
	#define MAX_DATA 58
	#define HDR_SIZE 7

	/*
	 * Thanks to Daniel Glöckner <daniel-gl@gmx.net> about that info.
	 *
	 * byte 0: MCS 51 core (1 = Link, 2 = OFDM)
	 * byte 1-2: big endian destination address
	 * byte 3-4: big endian number of data bytes following the header
	 * byte 5-6: big endian header checksum (ignored by the chip)
	 */
	for (i = fw->size; i >= HDR_SIZE;) {
		hdr_core = fw->data[fw->size - i + 0];
		hdr_addr = fw->data[fw->size - i + 1] << 8;
		hdr_addr |= fw->data[fw->size - i + 2] << 0;
		hdr_data_len = fw->data[fw->size - i + 3] << 8;
		hdr_data_len |= fw->data[fw->size - i + 4] << 0;
		hdr_checksum = fw->data[fw->size - i + 5] << 8;
		hdr_checksum |= fw->data[fw->size - i + 6] << 0;

		dev_dbg(&d->udev->dev, "core=%d addr=%04x data_len=%d checksum=%04x\n",
			hdr_core, hdr_addr, hdr_data_len, hdr_checksum);

		if ((hdr_core != 1 && hdr_core != 2) ||
		    hdr_data_len > i - HDR_SIZE)
			return -EINVAL;

		/* download begin packet */
		req.cmd = CMD_FW_DL_BEGIN;
		ret = avl6381_usb_ctrl_msg(d, &req);
		if (ret < 0)
			goto err;

		/* download firmware packet(s) */
		for (j = HDR_SIZE + hdr_data_len; j > 0; j -= MAX_DATA) {
			len = j;
			if (len > MAX_DATA)
				len = MAX_DATA;
			req_fw_dl.wlen = len;
			req_fw_dl.wbuf = (u8 *)&fw->data[fw->size - i +
					HDR_SIZE + hdr_data_len - j];
			ret = avl6381_usb_ctrl_msg(d, &req_fw_dl);
			if (ret < 0)
				goto err;
		}

		/* download end packet */
		req.cmd = CMD_FW_DL_END;
		ret = avl6381_usb_ctrl_msg(d, &req);
		if (ret < 0)
			goto err;

		i -= hdr_data_len + HDR_SIZE;

		dev_dbg(&d->udev->dev, "data uploaded=%zu\n", fw->size - i);
	}

	if (i)
		return -EINVAL;

	return 0;

err:
	dev_dbg(&d->udev->dev, "failed=%d\n", ret);
	return ret;
}

/* scatter download (fw blob uses 0x03 records) */
static int avl6381_usb_download_firmware_new(struct dvb_usb_device *d,
					     const struct firmware *fw)
{
	int ret;
	size_t i, i_prev;
	struct usb_req req_fw_dl = { CMD_FW_SCATTER_WR, 0, 0, NULL, 0, NULL };

	if (fw->size < 7)
		return -EINVAL;
	for (i = 7, i_prev = 0; i <= fw->size; i++) {
		if (i == fw->size ||
		    (i + 2 < fw->size && fw->data[i + 0] == 0x03 &&
		     (fw->data[i + 1] == 0x00 ||
		      fw->data[i + 1] == 0x01) &&
		     fw->data[i + 2] == 0x00)) {
			if (i - i_prev > BUF_LEN - REQ_HDR_LEN - CHECKSUM_LEN)
				return -EINVAL;
			req_fw_dl.wlen = i - i_prev;
			req_fw_dl.wbuf = (u8 *)&fw->data[i_prev];
			i_prev = i;
			ret = avl6381_usb_ctrl_msg(d, &req_fw_dl);
			if (ret < 0)
				goto err;

			dev_dbg(&d->udev->dev, "data uploaded=%zu\n", i);
		}
	}

	return 0;

err:
	dev_dbg(&d->udev->dev, "failed=%d\n", ret);
	return ret;
}

static int avl6381_usb_download_firmware(struct dvb_usb_device *d,
					 const struct firmware *fw)
{
	int ret;
	u8 wbuf[1];
	u8 rbuf[4];
	struct usb_req req = { 0, 0, 0, NULL, 0, NULL };
	struct usb_req req_fw_ver = { CMD_FW_QUERYINFO, 0, 1, wbuf, 4, rbuf };

	dev_dbg(&d->udev->dev, "\n");

	if (!fw->size)
		return -EINVAL;

	if (fw->data[0] == 0x01)
		ret = avl6381_usb_download_firmware_old(d, fw);
	else
		ret = avl6381_usb_download_firmware_new(d, fw);
	if (ret < 0)
		goto err;

	/* firmware loaded, request boot */
	req.cmd = CMD_FW_BOOT;
	ret = avl6381_usb_ctrl_msg(d, &req);
	if (ret < 0)
		goto err;

	/* ensure firmware starts */
	wbuf[0] = 1;
	ret = avl6381_usb_ctrl_msg(d, &req_fw_ver);
	if (ret < 0)
		goto err;

	if (!(rbuf[0] || rbuf[1] || rbuf[2] || rbuf[3])) {
		dev_err(&d->udev->dev, "firmware did not run\n");
		ret = -ENODEV;
		goto err;
	}

	dev_info(&d->udev->dev, "firmware version=%d.%d.%d.%d\n",
		 rbuf[0], rbuf[1], rbuf[2], rbuf[3]);

	return 0;

err:
	dev_dbg(&d->udev->dev, "failed=%d\n", ret);
	return ret;
}

static int avl6381_usb_init(struct dvb_usb_device *d)
{
	int ret;
	u16 frame_size = (d->udev->speed == USB_SPEED_FULL ? 5 : 816) * 188 / 4;
	u8 packet_size = (d->udev->speed == USB_SPEED_FULL ? 64 : 512) / 4;
	u8 buf[3];

	/* I2C master bus 2 clock speed 366k */
	ret = avl6381_usb_wr_reg(d, 0xf6a7, I2C_SPEED_366K);

	/* I2C master bus 1,3 clock speed 366k */
	ret |= avl6381_usb_wr_reg(d, 0xf103, I2C_SPEED_366K);

	/* ignore sync byte: no */
	ret |= avl6381_usb_wr_reg(d, 0xda1a, 0);

	/* dvb-t interrupt: enable */
	ret |= avl6381_usb_wr_reg_mask(d, 0xf41f, 0x04, 0x04);

	/* mpeg full speed */
	ret |= avl6381_usb_wr_reg_mask(d, 0xda10, 0x00, 0x00);

	/* dvb-t mode: enable */
	ret |= avl6381_usb_wr_reg_mask(d, 0xf41a, 0x05, 0x05);

	ret |= avl6381_usb_wr_reg_mask(d, 0xda1d, 0x01, 0x01);

	/* enable ep4 */
	ret |= avl6381_usb_wr_reg_mask(d, 0xdd11, 0x0f, 0x0f);

	/* disable nak of ep4 */
	ret |= avl6381_usb_wr_reg_mask(d, 0xdd13, 0x1b, 0x1b);

	/* enable ep4 */
	ret |= avl6381_usb_wr_reg_mask(d, 0xdd11, 0x2f, 0x2f);

	buf[0] = (frame_size & 0xff);
	buf[1] = ((frame_size >> 8) & 0xff);
	ret |= avl6381_usb_wr_regs(d, 0xdd88, buf, 2);

	/* max bulk packet size */
	ret |= avl6381_usb_wr_reg(d, 0xdd0c, packet_size);

	ret |= avl6381_usb_wr_reg_mask(d, 0xda05, 0x00, 0x01);
	ret |= avl6381_usb_wr_reg_mask(d, 0xda06, 0x00, 0x01);
	ret |= avl6381_usb_wr_reg_mask(d, 0xda1d, 0x00, 0x01);

	/* reverse: no */
	ret |= avl6381_usb_wr_reg(d, 0xd920, 0x00);

	/* power config */
	ret |= avl6381_usb_wr_reg(d, 0xd833, 0x01);
	ret |= avl6381_usb_wr_reg(d, 0xd830, 0x00);
	ret |= avl6381_usb_wr_reg(d, 0xd831, 0x01);
	ret |= avl6381_usb_wr_reg(d, 0xd832, 0x00);
	ret |= avl6381_usb_wr_reg(d, 0x4976, 0x01);

	msleep(20);
	/* ts_in_src: serial */
	ret |= avl6381_usb_wr_reg_mask(d, 0xda58, 0x00, 0x01);
	usleep_range(8000, 9000);
	/* in ts packet length */
	ret |= avl6381_usb_wr_reg(d, 0xda51, 0x00);
	usleep_range(8000, 9000);
	/* ts0_aggre_mode */
	ret |= avl6381_usb_wr_reg(d, 0xda73, 0x01);
	/* ts0_sync_byte */
	ret |= avl6381_usb_wr_reg(d, 0xda78, 0x47);
	msleep(30);
	/* ts0_en */
	ret |= avl6381_usb_wr_reg(d, 0xda4c, 0x01);
	usleep_range(8000, 9000);
	/* ts_fail_ignore */
	ret |= avl6381_usb_wr_reg(d, 0xda5a, 0x1f);

	if (ret < 0)
		goto err;

	return 0;

err:
	dev_dbg(&d->udev->dev, "failed=%d\n", ret);
	return ret;
}

static int avl6381_usb_frontend_attach(struct dvb_usb_adapter *adap)
{
	struct dvb_usb_device *d = adap_to_d(adap);
	struct state *state = d_to_priv(d);
	int ret;

	/* hard-reset the demod through GPIO1 */
	ret = avl6381_usb_set_gpio_mode(d, GPIO1, GPIO_MODE_OUT);
	ret |= avl6381_usb_enable_gpio(d, GPIO1, GPIO_ENABLE);
	ret |= avl6381_usb_write_gpio(d, GPIO1, GPIO_LOW);
	msleep(30);
	ret |= avl6381_usb_write_gpio(d, GPIO1, GPIO_HIGH);
	msleep(150);
	if (ret)
		return ret;

	dev_info(&d->udev->dev, "Checking for Availink AVL6381 demodulator ...\n");

	state->avl6381_cfg.i2c_address = AVL6381_USB_I2C_ADDRESS;
	state->avl6381_cfg.fe = &adap->fe[0];
	state->avl6381_cfg.tuner_select_input = avl6381_usb_select_input;

	adap->fe[0] = dvb_attach(avl6381_attach, &state->avl6381_cfg,
				 &d->i2c_adap);
	if (!adap->fe[0]) {
		dev_err(&d->udev->dev, "no AVL6381 demod found\n");
		return -ENODEV;
	}

	return ret;
}

static int avl6381_usb_tuner_attach(struct dvb_usb_adapter *adap)
{
	struct dvb_usb_device *d = adap_to_d(adap);

	if (dvb_attach(mxl603_attach, adap->fe[0], &d->i2c_adap,
		       MXL603_USB_I2C_ADDRESS, &mxl603_cfg) == NULL)
		return -ENODEV;

	return 0;
}

static int avl6381_usb_get_stream_config(struct dvb_frontend *fe, u8 *ts_type,
					 struct usb_data_stream_properties *stream)
{
	struct dvb_usb_device *d = fe_to_d(fe);

	if (d->udev->speed == USB_SPEED_FULL)
		stream->u.bulk.buffersize = 5 * 188;

	return 0;
}

static const struct dvb_usb_device_properties avl6381_usb_props = {
	.driver_name = KBUILD_MODNAME,
	.owner = THIS_MODULE,
	.adapter_nr = adapter_nr,
	.size_of_priv = sizeof(struct state),

	.generic_bulk_ctrl_endpoint = 0x02,
	.generic_bulk_ctrl_endpoint_response = 0x81,

	.identify_state = avl6381_usb_identify_state,
	.download_firmware = avl6381_usb_download_firmware,

	.i2c_algo = &avl6381_usb_i2c_algo,
	.frontend_attach = avl6381_usb_frontend_attach,
	.tuner_attach = avl6381_usb_tuner_attach,
	.init = avl6381_usb_init,
	.get_stream_config = avl6381_usb_get_stream_config,

	.num_adapters = 1,
	.adapter = {
		{
			/* Pass the full transport stream to the software demux. */
			.stream = DVB_USB_STREAM_BULK(0x84, 4, 816 * 188),
		},
	},
};

static const struct usb_device_id avl6381_usb_id_table[] = {
	{ DVB_USB_DEVICE(USB_VID_CYPRESS, USB_PID_ITETECH_IT9303,
		&avl6381_usb_props, "Runqida AVL6381 DTMB", NULL) },
	{ DVB_USB_DEVICE(0x048d, 0x9306,
		&avl6381_usb_props, "Runqida AVL6381 DTMB (ITE)", NULL) },
	{ }
};
MODULE_DEVICE_TABLE(usb, avl6381_usb_id_table);

static struct usb_driver avl6381_usb_driver = {
	.name = KBUILD_MODNAME,
	.id_table = avl6381_usb_id_table,
	.probe = dvb_usbv2_probe,
	.disconnect = dvb_usbv2_disconnect,
	.suspend = dvb_usbv2_suspend,
	.resume = dvb_usbv2_resume,
	.reset_resume = dvb_usbv2_reset_resume,
	.no_dynamic_id = 0, /* allow manual rebind after unbinding af9035 */
	.soft_unbind = 1,
};

module_usb_driver(avl6381_usb_driver);

/* the demod and tuner modules must be present before the bridge probes */
MODULE_SOFTDEP("pre: avl6381 mxl603");

MODULE_DESCRIPTION("dvb-usb-v2 bridge for the runqida AVL6381 DTMB stick (IT9303)");
MODULE_AUTHOR("Xiaodong Ni <nxiaodong520@gmail.com>");
MODULE_AUTHOR("wwek");
MODULE_AUTHOR("Antti Palosaari <crope@iki.fi>");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(AVL6381_USB_FIRMWARE);
