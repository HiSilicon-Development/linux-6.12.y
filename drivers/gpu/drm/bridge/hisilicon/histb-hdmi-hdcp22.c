// SPDX-License-Identifier: GPL-2.0
/*
 * Hi3798CV200 imagev200 HDCP 2.2 transmitter.
 *
 * Register and sequencing authority: HiSTBLinuxV100R005C00SPC060 imagev200
 * hdmi_hal.c (Hdcp22ProgramLoad), si_mod_tx_hdcp.c, si_drv_tx_regs.h and
 * platform/platform.c. CRA addresses are (BSP address - 0x3000) * 4.
 *
 * Firmware: hisilicon/hi3798cv200-hdcp22.bin, exactly 16384 raw bytes, in
 * ascending program-address order. The BSP c8051_hdcp22_program.bin is a C
 * initializer containing 16384 comma-separated byte literals, NOT the raw
 * firmware file. Obtain the licensed image from the platform supplier; no
 * proprietary bytes are supplied here. Size checking is not authentication:
 * the hardware code checker must finish successfully before authentication.
 *
 * The non-TEE BSP provisions TX 2.2 by loading/checking this MCU image only
 * (drv_hdmi_intf.c, lines 1038-1064). Its separate encrypted-key/cipher path
 * is TX 1.4; cipher's 2.2 key-package path targets RX RAM, not this TX engine.
 * Do not substitute either for TX 2.2 provisioning. Platforms that restrict
 * PRAM/code-check access to a secure world need their vendor secure service;
 * a failed hardware load/check is fatal, never bypassed with ROM/fake auth.
 *
 * The caller supplies PRAM 0xf8cf0000/0x10000 (one byte per 32-bit slot),
 * PERI 0xf8a20000 and AVCTTPT 0xf8a80038/4, and serializes all engine/DDC
 * accesses. MCU firmware performs the cryptographic protocol, including
 * certificate, H', L', V' and M' verification. Host approval additionally
 * requires SRM checking, topology/sequence checks and stream completion.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <drm/display/drm_hdcp.h>
#include <drm/display/drm_hdcp_helper.h>
#include <drm/drm_connector.h>

#include "histb-hdmi-hdcp.h"

#define HDCP22_FIRMWARE "hisilicon/hi3798cv200-hdcp22.bin"
#define HDCP22_IMAGE_SIZE 16384
#define HDCP22_REG(index) (0x2000 + (index) * 4)
#define TPI_COPP_DATA2 (0x1800 + 0x2a * 4)
#define TPI_HW_OPT0 (0x1800 + 0xb8 * 4)
#define SCDC_CTL (0x2400 + 0x20 * 4)
#define LM_DDC (0xcc * 4)
#define SCDC_INTR0 (0x2400 + 0x25 * 4)
#define TPI_DDC_MASTER_EN (0x1800 + 0xf8 * 4)
#define DDC_ADDR (0xed * 4)
#define DDC_SEGM (0xee * 4)
#define DDC_OFFSET (0xef * 4)
#define DDC_DIN_CNT1 (0xf0 * 4)
#define DDC_DIN_CNT2 (0xf1 * 4)
#define DDC_STATUS (0xf2 * 4)
#define DDC_CMD (0xf3 * 4)
#define DDC_DATA (0xf4 * 4)
#define DDC_DOUT_CNT (0xf5 * 4)

#define HDCP22_CTRL0 HDCP22_REG(0x00)
#define HDCP22_CTRL1 HDCP22_REG(0x01)
#define HDCP22_INTR0 HDCP22_REG(0x03)
#define HDCP22_INTR1 HDCP22_REG(0x04)
#define HDCP22_INTR2 HDCP22_REG(0x05)
#define HDCP22_INTR0_MASK HDCP22_REG(0x07)
#define HDCP22_INTR1_MASK HDCP22_REG(0x08)
#define HDCP22_INTR2_MASK HDCP22_REG(0x09)
#define HDCP22_AUTH_STAT HDCP22_REG(0x0c)
#define HDCP22_STATE HDCP22_REG(0x0d)
#define HDCP22_GEN_STATUS HDCP22_REG(0x0e)
#define HDCP22_TP0 HDCP22_REG(0x10)
#define HDCP22_GP_IN0 HDCP22_REG(0x20)
#define HDCP22_GP_OUT0 HDCP22_REG(0x24)
#define HDCP22_RX_ID4 HDCP22_REG(0x2c)
#define HDCP22_RPT_DETAIL HDCP22_REG(0x2d)
#define HDCP22_SMNG_K HDCP22_REG(0x2e)
#define HDCP22_RPT_DEPTH HDCP22_REG(0x2f)
#define HDCP22_RPT_DEVCNT HDCP22_REG(0x30)
#define HDCP22_SEQ_V0 HDCP22_REG(0x31)
#define HDCP22_SEQ_M0 HDCP22_REG(0x34)
#define HDCP22_TX_CTRL HDCP22_REG(0x70)
#define HDCP22_TX_STATUS HDCP22_REG(0x71)
#define HDCP22_SMNG_IN HDCP22_REG(0x72)
#define HDCP22_RCVID_OUT HDCP22_REG(0x73)
#define HDCP22_SRST HDCP22_REG(0xb5)
#define HDCP22_POLL_CS HDCP22_REG(0xb6)
#define HDCP22_CTL0 HDCP22_REG(0xbd)
#define HDCP22_CTL1 HDCP22_REG(0xbe)
#define HDCP22_POLL_VAL HDCP22_REG(0xc3)

#define HDCP22_ENABLE BIT(0)
#define HDCP22_ENCRYPT BIT(7)
#define HDCP22_AUTH_DONE BIT(0)
#define HDCP22_AUTH_FAIL BIT(1)
#define HDCP22_RPT_READY BIT(2)
#define HDCP22_HASH_FAIL BIT(3)
#define HDCP22_CCHK_DONE BIT(4)
#define HDCP22_CCHK_FAIL BIT(5)
#define HDCP22_REAUTH BIT(6)
#define HDCP22_ID_CHANGED BIT(0)
#define HDCP22_AKE_SENT BIT(2)
#define HDCP22_STREAM_DONE BIT(5)
#define HDCP22_H_RECEIVED BIT(2)
#define HDCP22_REPEATER BIT(2)
#define HDCP22_RCVID_READ BIT(0)
#define HDCP22_RCVID_START BIT(1)
#define HDCP22_SMNG_XFER BIT(2)
#define HDCP22_SMNG_WRITE BIT(3)
#define HDCP22_SMNG_START BIT(4)
#define HDCP22_CORE_RESET BIT(0)
#define HDCP22_LOGIC_RESET BIT(5)
#define HDCP22_TPI_ENCRYPT_DISABLE BIT(5)

#define PERI_HDMITX_CTRL 0x8b0
#define PERI_CCHK_DONE BIT(25)
#define PERI_CUPD_START BIT(15)
#define PERI_CUPD_DONE BIT(13)
#define AVCTTPT_CUPD_HW BIT(7)
#define AVCTTPT_PRAM_WR_LOCK BIT(12)
#define AVCTTPT_PRAM_RD_LOCK BIT(13)

struct histb_hdcp22 {
	struct histb_hdcp_io *io;
	unsigned long deadline;
	unsigned long encrypt_after;
	u32 seq_v;
	u32 seq_m;
	u8 receiver_id[HDCP_2_2_RECEIVER_ID_LEN];
	u8 receiver_ids[HDCP_2_2_RECEIVER_IDS_MAX_LEN];
	bool running;
	bool receiver_checked;
	bool repeater_ready;
	bool list_checked;
	bool have_seq_v;
	bool stream_pending;
	bool stream_done;
	bool auth_done;
	bool repeater;
	bool encrypted;
};

static u8 hdcp22_read(struct histb_hdcp22 *hdcp, u32 reg)
{
	return hdcp->io->read(hdcp->io->ctx, reg);
}

static void hdcp22_write(struct histb_hdcp22 *hdcp, u32 reg, u8 value)
{
	hdcp->io->write(hdcp->io->ctx, reg, value);
}

static void hdcp22_update(struct histb_hdcp22 *hdcp, u32 reg, u8 mask,
			  u8 value)
{
	hdcp22_write(hdcp, reg, (hdcp22_read(hdcp, reg) & ~mask) | value);
}

static void hdcp22_disable_encryption(struct histb_hdcp22 *hdcp)
{
	hdcp22_update(hdcp, HDCP22_CTL0, HDCP22_ENCRYPT, 0);
	hdcp22_update(hdcp, TPI_COPP_DATA2, HDCP22_TPI_ENCRYPT_DISABLE,
		      HDCP22_TPI_ENCRYPT_DISABLE);
	hdcp->encrypted = false;
}

void histb_hdcp22_stop(struct histb_hdcp22 *hdcp)
{
	if (!hdcp)
		return;

	hdcp22_disable_encryption(hdcp);
	hdcp22_update(hdcp, HDCP22_CTL0, HDCP22_ENABLE, 0);
	hdcp22_write(hdcp, HDCP22_INTR0_MASK, 0);
	hdcp22_write(hdcp, HDCP22_INTR1_MASK, 0);
	hdcp22_write(hdcp, HDCP22_INTR2_MASK, 0);
	hdcp22_write(hdcp, HDCP22_TX_CTRL, 0);
	hdcp22_update(hdcp, HDCP22_SRST, HDCP22_CORE_RESET,
		      HDCP22_CORE_RESET);
	hdcp22_update(hdcp, HDCP22_SRST, HDCP22_CORE_RESET, 0);
	hdcp22_update(hdcp, HDCP22_POLL_CS, BIT(0), BIT(0));
	hdcp22_update(hdcp, HDCP22_CTRL0, BIT(0) | BIT(2), 0);
	hdcp22_write(hdcp, HDCP22_INTR0, 0xff);
	hdcp22_write(hdcp, HDCP22_INTR1, 0xff);
	hdcp22_write(hdcp, HDCP22_INTR2, 0xff);
	hdcp->running = false;
}

static int hdcp22_fail(struct histb_hdcp22 *hdcp, int error)
{
	histb_hdcp22_stop(hdcp);
	return error;
}

/* No image caching: resume/reset can invalidate PRAM and its code check. */
static int hdcp22_load_program(struct histb_hdcp22 *hdcp)
{
	const struct firmware *fw;
	struct histb_hdcp_io *io = hdcp->io;
	unsigned long timeout;
	u32 value;
	unsigned int i, retry;
	int ret, cleanup_ret;

	ret = request_firmware(&fw, HDCP22_FIRMWARE, io->dev);
	if (ret)
		return ret;
	if (fw->size != HDCP22_IMAGE_SIZE) {
		ret = -EINVAL;
		goto release;
	}

	value = readl(io->avcttpt);
	value &= ~(AVCTTPT_PRAM_WR_LOCK | AVCTTPT_PRAM_RD_LOCK);
	writel(value, io->avcttpt);
	hdcp22_update(hdcp, HDCP22_SRST, HDCP22_LOGIC_RESET,
		      HDCP22_LOGIC_RESET);
	msleep(5);
	hdcp22_update(hdcp, HDCP22_SRST, HDCP22_LOGIC_RESET, 0);
	hdcp22_write(hdcp, HDCP22_INTR0, HDCP22_CCHK_DONE);
	ret = regmap_update_bits(io->peri, PERI_HDMITX_CTRL,
				 PERI_CUPD_START | PERI_CUPD_DONE,
				 PERI_CUPD_START);
	if (ret)
		goto cleanup;
	msleep(2);
	writel(readl(io->avcttpt) | AVCTTPT_CUPD_HW, io->avcttpt);
	msleep(2);

	for (i = 0; i < fw->size; i++) {
		for (retry = 0; retry < 5; retry++) {
			writel(fw->data[i], io->pram + i * 4);
			if (readl(io->pram + i * 4) == fw->data[i])
				break;
		}
		if (retry == 5) {
			u32 peri = 0;
			int peri_ret = regmap_read(io->peri, PERI_HDMITX_CTRL, &peri);

			dev_err_ratelimited(io->dev,
				"HDCP 2.2 PRAM readback failed at byte %u: read=%08x avcttpt=%08x peri=%08x/%d reset=%02x\n",
				i, readl(io->pram + i * 4), readl(io->avcttpt),
				peri, peri_ret, hdcp22_read(hdcp, HDCP22_SRST));
			ret = -EIO;
			goto cleanup;
		}
	}

	writel(readl(io->avcttpt) & ~AVCTTPT_CUPD_HW, io->avcttpt);
	msleep(2);
	ret = regmap_update_bits(io->peri, PERI_HDMITX_CTRL,
				 PERI_CUPD_DONE, PERI_CUPD_DONE);
	if (ret)
		goto cleanup;
	msleep(2);

	/* BSP allows 3000 polls at 5ms for the hardware code checker. */
	timeout = jiffies + msecs_to_jiffies(15000);
	for (;;) {
		ret = regmap_read(io->peri, PERI_HDMITX_CTRL, &value);
		if (ret)
			break;
		if (value & PERI_CCHK_DONE) {
			u8 status = hdcp22_read(hdcp, HDCP22_AUTH_STAT);

			if (!(hdcp22_read(hdcp, HDCP22_INTR0) &
			      HDCP22_CCHK_DONE) || !(status & HDCP22_CCHK_DONE))
				ret = -EIO;
			else if (status & HDCP22_CCHK_FAIL)
				ret = -EKEYREJECTED;
			if (ret)
				dev_err_ratelimited(io->dev,
					"HDCP 2.2 code-check failed: peri=%08x intr=%02x auth=%02x\n",
					value, hdcp22_read(hdcp, HDCP22_INTR0), status);
			break;
		}
		if (time_after_eq(jiffies, timeout)) {
			ret = -ETIMEDOUT;
			dev_err_ratelimited(io->dev,
				"HDCP 2.2 code-check timeout: peri=%08x\n", value);
			break;
		}
		msleep(5);
	}

cleanup:
	writel(readl(io->avcttpt) & ~AVCTTPT_CUPD_HW, io->avcttpt);
	cleanup_ret = regmap_update_bits(io->peri, PERI_HDMITX_CTRL,
					PERI_CUPD_DONE, 0);
	if (!ret)
		ret = cleanup_ret;
	msleep(2);
	cleanup_ret = regmap_update_bits(io->peri, PERI_HDMITX_CTRL,
					PERI_CUPD_START, 0);
	if (!ret)
		ret = cleanup_ret;
	/* Close CPU PRAM access after the update, including all error paths. */
	writel(readl(io->avcttpt) | AVCTTPT_PRAM_WR_LOCK |
	       AVCTTPT_PRAM_RD_LOCK, io->avcttpt);
	hdcp22_write(hdcp, HDCP22_INTR0, HDCP22_CCHK_DONE);
release:
	release_firmware(fw);
	return ret;
}

struct histb_hdcp22 *histb_hdcp22_create(struct histb_hdcp_io *io)
{
	struct histb_hdcp22 *hdcp;

	if (!io || !io->dev || !io->read || !io->write || !io->peri ||
	    !io->pram || !io->avcttpt)
		return ERR_PTR(-EINVAL);

	hdcp = devm_kzalloc(io->dev, sizeof(*hdcp), GFP_KERNEL);
	if (!hdcp)
		return ERR_PTR(-ENOMEM);
	hdcp->io = io;
	return hdcp;
}

/* Native sReadHDCP2XVersion transaction, with bounded conflict/error waits. */
static int hdcp22_sink_capable(struct histb_hdcp22 *hdcp)
{
	u8 scdc = hdcp22_read(hdcp, SCDC_CTL);
	u8 master = hdcp22_read(hdcp, TPI_DDC_MASTER_EN);
	unsigned long deadline = jiffies + msecs_to_jiffies(100);
	u8 status;
	int ret = -ETIMEDOUT;

	while (hdcp22_read(hdcp, DDC_STATUS) & BIT(4)) {
		if (time_after_eq(jiffies, deadline))
			return ret;
		usleep_range(1000, 2000);
	}
	do {
		hdcp22_write(hdcp, SCDC_INTR0, BIT(2));
		hdcp22_update(hdcp, SCDC_CTL, BIT(0), 0);
		hdcp22_update(hdcp, SCDC_CTL, BIT(0), BIT(0));
		if (!(hdcp22_read(hdcp, SCDC_INTR0) & BIT(2)))
			break;
		if (time_after_eq(jiffies, deadline))
			goto restore;
		usleep_range(1000, 2000);
	} while (true);
	hdcp22_update(hdcp, TPI_DDC_MASTER_EN, BIT(7), BIT(7));
	hdcp22_write(hdcp, DDC_STATUS, 0x9f);
	hdcp22_write(hdcp, DDC_ADDR, 0x74);
	hdcp22_write(hdcp, DDC_SEGM, 0);
	hdcp22_write(hdcp, DDC_OFFSET, 0x50);
	hdcp22_write(hdcp, DDC_DIN_CNT2, 0);
	hdcp22_write(hdcp, DDC_DIN_CNT1, 1);
	hdcp22_write(hdcp, DDC_CMD, 0x39);
	hdcp22_write(hdcp, DDC_CMD, 0x32);
	msleep(2);
	for (;;) {
		status = hdcp22_read(hdcp, DDC_STATUS);
		if (status & (BIT(5) | BIT(6))) {
			ret = -ENXIO;
			break;
		}
		if (!(status & BIT(4)) &&
		    (hdcp22_read(hdcp, DDC_DOUT_CNT) & 0x1f) == 1) {
			ret = hdcp22_read(hdcp, DDC_DATA) == 0x04 ? 0 : -ENODEV;
			break;
		}
		if (time_after_eq(jiffies, deadline))
			break;
		usleep_range(1000, 2000);
	}
	hdcp22_write(hdcp, DDC_CMD, 0x3f);
	hdcp22_update(hdcp, TPI_DDC_MASTER_EN, BIT(7), master & BIT(7));
	hdcp22_update(hdcp, DDC_CMD, 0x7f, 0);
restore:
	hdcp22_write(hdcp, SCDC_CTL, scdc);
	return ret;
}

int histb_hdcp22_start(struct histb_hdcp22 *hdcp)
{
	/* sHardwareInit's 24MHz timing configuration, TP0 through TP15. */
	static const u8 timing[] = {
		0x02, 0x8c, 0x01, 0x32, 0x1e, 0x78, 0x02, 0x0a,
		0x0a, 0x14, 0x16, 0xc8, 0x60, 0x10, 0xc8, 0x00,
	};
	static const u8 gp_in[] = { 0x00, 0x22, 0x80, 0x00 };
	unsigned int i;
	int ret;

	histb_hdcp22_stop(hdcp);
	ret = hdcp22_sink_capable(hdcp);
	if (ret)
		return hdcp22_fail(hdcp, ret);
	ret = hdcp22_load_program(hdcp);
	if (ret)
		return hdcp22_fail(hdcp, ret);

	hdcp->receiver_checked = false;
	hdcp->repeater_ready = false;
	hdcp->list_checked = false;
	hdcp->have_seq_v = false;
	hdcp->stream_pending = false;
	hdcp->stream_done = false;
	hdcp->auth_done = false;
	hdcp->seq_v = 0;
	hdcp->seq_m = 0;
	hdcp22_write(hdcp, HDCP22_CTL1, 0);
	hdcp22_write(hdcp, HDCP22_CTL1, BIT(0));
	for (i = 0; i < ARRAY_SIZE(timing); i++)
		hdcp22_write(hdcp, HDCP22_TP0 + i * 4, timing[i]);
	for (i = 0; i < ARRAY_SIZE(gp_in); i++)
		hdcp22_write(hdcp, HDCP22_GP_IN0 + i * 4, gp_in[i]);
	hdcp22_write(hdcp, HDCP22_POLL_VAL, 0x05);
	/* This write is part of the native imagev200 initialization. */
	hdcp22_write(hdcp, HDCP22_GP_OUT0, 0xb0);
	hdcp22_write(hdcp, HDCP22_INTR0, 0xff);
	hdcp22_write(hdcp, HDCP22_INTR1, 0xff);
	hdcp22_write(hdcp, HDCP22_INTR2, 0xff);
	hdcp22_update(hdcp, SCDC_CTL, BIT(1), 0);
	hdcp22_update(hdcp, TPI_HW_OPT0, BIT(1), BIT(1));
	hdcp22_update(hdcp, TPI_HW_OPT0, BIT(1), 0);
	hdcp22_update(hdcp, LM_DDC, BIT(7), BIT(7));
	hdcp22_update(hdcp, HDCP22_POLL_CS, BIT(0), 0);
	hdcp22_write(hdcp, HDCP22_CTL0, HDCP22_ENABLE);
	hdcp22_update(hdcp, HDCP22_CTRL0, BIT(0) | BIT(2), BIT(0) | BIT(2));
	hdcp22_write(hdcp, HDCP22_INTR0_MASK, 0x4f);
	hdcp22_write(hdcp, HDCP22_INTR1_MASK, 0x25);
	hdcp22_write(hdcp, HDCP22_INTR2_MASK, HDCP22_H_RECEIVED);
	hdcp22_write(hdcp, HDCP22_CTRL1, 0xe1);
	hdcp22_update(hdcp, HDCP22_CTRL1, BIT(0), 0);
	hdcp->deadline = jiffies + msecs_to_jiffies(5000);
	hdcp->running = true;
	return 0;
}

/* DRM's SRM helper allows an absent SRM; this engine requires one. */
static int hdcp22_check_revocation(struct histb_hdcp22 *hdcp,
				 struct drm_connector *connector,
				 u8 *ids, unsigned int count)
{
	const struct firmware *srm;
	int ret;

	ret = request_firmware_direct(&srm, "display_hdcp_srm.bin", hdcp->io->dev);
	if (ret)
		return ret;
	ret = histb_hdcp_check_revocation(srm->data, srm->size, ids, count);
	release_firmware(srm);
	return ret;
}

static int hdcp22_check_receiver(struct histb_hdcp22 *hdcp,
				 struct drm_connector *connector)
{
	unsigned int i;
	int ret;

	/* Core registers have the reverse byte order of the SRM IDs. */
	for (i = 0; i < HDCP_2_2_RECEIVER_ID_LEN; i++)
		hdcp->receiver_id[i] = hdcp22_read(hdcp, HDCP22_RX_ID4 - i * 4);
	ret = hdcp22_check_revocation(hdcp, connector, hdcp->receiver_id, 1);
	if (!ret)
		hdcp->receiver_checked = true;
	return ret;
}

static u32 hdcp22_read_seq(struct histb_hdcp22 *hdcp, u32 reg)
{
	return hdcp22_read(hdcp, reg) |
	       hdcp22_read(hdcp, reg + 4) << 8 |
	       hdcp22_read(hdcp, reg + 8) << 16;
}

static int hdcp22_check_topology(struct histb_hdcp22 *hdcp,
				 struct drm_connector *connector)
{
	u8 count = hdcp22_read(hdcp, HDCP22_RPT_DEVCNT);
	u8 depth = hdcp22_read(hdcp, HDCP22_RPT_DEPTH);
	u8 detail = hdcp22_read(hdcp, HDCP22_RPT_DETAIL);
	u32 seq_v = hdcp22_read_seq(hdcp, HDCP22_SEQ_V0);
	unsigned int i, j;
	int ret;

	if (detail & (BIT(2) | BIT(3)) ||
	    count > HDCP_2_2_MAX_DEVICE_COUNT || depth > 4 ||
	    (count && !depth))
		return -EOVERFLOW;
	if (seq_v == HDCP_2_2_SEQ_NUM_MAX ||
	    (!hdcp->have_seq_v && seq_v) ||
	    (hdcp->have_seq_v && seq_v != hdcp->seq_v + 1))
		return -EPROTO;

	hdcp22_update(hdcp, HDCP22_TX_CTRL, HDCP22_RCVID_START,
		      HDCP22_RCVID_START);
	for (i = 0; i < count * HDCP_2_2_RECEIVER_ID_LEN; i++) {
		hdcp->receiver_ids[i] = hdcp22_read(hdcp, HDCP22_RCVID_OUT);
		hdcp22_update(hdcp, HDCP22_TX_CTRL, HDCP22_RCVID_READ,
			      HDCP22_RCVID_READ);
		hdcp22_update(hdcp, HDCP22_TX_CTRL, HDCP22_RCVID_READ, 0);
	}
	hdcp22_update(hdcp, HDCP22_TX_CTRL, HDCP22_RCVID_START, 0);
	for (i = 0; i < count; i++) {
		u8 *id = hdcp->receiver_ids + i * HDCP_2_2_RECEIVER_ID_LEN;

		if (!memcmp(id, hdcp->receiver_id, HDCP_2_2_RECEIVER_ID_LEN))
			return -EPROTO;
		for (j = 0; j < i; j++)
			if (!memcmp(id, hdcp->receiver_ids +
				    j * HDCP_2_2_RECEIVER_ID_LEN,
				    HDCP_2_2_RECEIVER_ID_LEN))
				return -EPROTO;
	}
	ret = hdcp22_check_revocation(hdcp, connector, hdcp->receiver_ids, count);
	if (ret)
		return ret;
	/* Detect a topology update while consuming the MCU's FIFO. */
	if (count != hdcp22_read(hdcp, HDCP22_RPT_DEVCNT) ||
	    depth != hdcp22_read(hdcp, HDCP22_RPT_DEPTH) ||
	    detail != hdcp22_read(hdcp, HDCP22_RPT_DETAIL) ||
	    seq_v != hdcp22_read_seq(hdcp, HDCP22_SEQ_V0))
		return -EAGAIN;
	hdcp->seq_v = seq_v;
	hdcp->have_seq_v = true;
	hdcp->list_checked = true;
	return 0;
}

static int hdcp22_stream_manage(struct histb_hdcp22 *hdcp)
{
	unsigned int i;

	if (hdcp->seq_m == HDCP_2_2_SEQ_NUM_MAX)
		return -EOVERFLOW;
	/* Native MCU transport sends the current value, then advances it. */
	hdcp->seq_m++;
	hdcp22_write(hdcp, HDCP22_SMNG_K, 1);
	hdcp22_update(hdcp, HDCP22_TX_CTRL, HDCP22_SMNG_START,
		      HDCP22_SMNG_START);
	/* Single HDMI stream, type 0 (the connector exposes no type-1 mode). */
	for (i = 0; i < 2; i++) {
		hdcp22_update(hdcp, HDCP22_TX_CTRL, HDCP22_SMNG_WRITE,
			      HDCP22_SMNG_WRITE);
		hdcp22_write(hdcp, HDCP22_SMNG_IN, 0);
		hdcp22_update(hdcp, HDCP22_TX_CTRL, HDCP22_SMNG_WRITE, 0);
	}
	hdcp22_write(hdcp, HDCP22_INTR1, HDCP22_STREAM_DONE);
	hdcp22_update(hdcp, HDCP22_TX_CTRL, HDCP22_SMNG_XFER,
		      HDCP22_SMNG_XFER);
	hdcp22_update(hdcp, HDCP22_TX_CTRL, HDCP22_SMNG_XFER, 0);
	for (i = 0; i < 3; i++)
		hdcp22_write(hdcp, HDCP22_SEQ_M0 + i * 4,
			     hdcp->seq_m >> (i * 8));
	hdcp->stream_pending = true;
	hdcp->deadline = jiffies + msecs_to_jiffies(5000);
	return 0;
}

int histb_hdcp22_poll(struct histb_hdcp22 *hdcp, struct drm_connector *connector)
{
	u8 intr0, intr1, intr2, status, state;
	bool repeater;
	int ret;

	if (!hdcp->running)
		return hdcp22_fail(hdcp, -EINVAL);
	if (!connector)
		return hdcp22_fail(hdcp, -EINVAL);

	intr0 = hdcp22_read(hdcp, HDCP22_INTR0);
	intr1 = hdcp22_read(hdcp, HDCP22_INTR1);
	intr2 = hdcp22_read(hdcp, HDCP22_INTR2);
	status = hdcp22_read(hdcp, HDCP22_AUTH_STAT);
	state = hdcp22_read(hdcp, HDCP22_STATE);
	hdcp22_write(hdcp, HDCP22_INTR0, intr0);
	hdcp22_write(hdcp, HDCP22_INTR1, intr1);
	hdcp22_write(hdcp, HDCP22_INTR2, intr2);

	if ((intr0 | status) & (HDCP22_AUTH_FAIL | HDCP22_CCHK_FAIL |
				HDCP22_REAUTH | HDCP22_HASH_FAIL))
		return hdcp22_fail(hdcp, -EIO);
	if (intr1 & HDCP22_AKE_SENT) {
		/* A new unsolicited AKE after authentication requires a new attempt. */
		if (hdcp->auth_done || hdcp->receiver_checked)
			return hdcp22_fail(hdcp, -EAGAIN);
		for (ret = 0; ret < 3; ret++)
			hdcp22_write(hdcp, HDCP22_SEQ_M0 + ret * 4, 0);
	}
	/* The native H' received event is where the BSP checks the core RX ID. */
	if (intr2 & HDCP22_H_RECEIVED) {
		if (hdcp->receiver_checked)
			return hdcp22_fail(hdcp, -EAGAIN);
		ret = hdcp22_check_receiver(hdcp, connector);
		if (ret)
			return hdcp22_fail(hdcp, ret);
	}
	repeater = hdcp22_read(hdcp, HDCP22_GEN_STATUS) & HDCP22_REPEATER;
	if (hdcp->auth_done && repeater != hdcp->repeater)
		return hdcp22_fail(hdcp, -EPROTO);
	if (intr0 & HDCP22_RPT_READY)
		hdcp->repeater_ready = true;
	if (intr1 & HDCP22_ID_CHANGED) {
		hdcp22_disable_encryption(hdcp);
		hdcp->list_checked = false;
		hdcp->stream_done = false;
		hdcp->stream_pending = false;
		if (!repeater || !hdcp->receiver_checked)
			return hdcp22_fail(hdcp, -EPROTO);
		ret = hdcp22_check_topology(hdcp, connector);
		if (ret)
			return hdcp22_fail(hdcp, ret);
	}
	if (intr1 & HDCP22_STREAM_DONE) {
		if (hdcp->stream_pending &&
		    hdcp22_read(hdcp, HDCP22_TX_STATUS) & BIT(2)) {
			hdcp->stream_pending = false;
			hdcp->stream_done = true;
			hdcp->encrypt_after = jiffies + msecs_to_jiffies(300);
		}
	}
	if (intr0 & HDCP22_AUTH_DONE) {
		if (status != 0x81 || state != 0x2b)
			return hdcp22_fail(hdcp, -EIO);
		hdcp->auth_done = true;
		hdcp->repeater = repeater;
		hdcp->encrypt_after = jiffies +
			msecs_to_jiffies(repeater ? 300 : 200);
	}
	if (repeater && hdcp->receiver_checked && hdcp->repeater_ready &&
	    hdcp->list_checked && !hdcp->stream_pending && !hdcp->stream_done) {
		ret = hdcp22_stream_manage(hdcp);
		if (ret)
			return hdcp22_fail(hdcp, ret);
	}

	if (hdcp->encrypted) {
		if (status != 0x81 || state != 0x2b ||
		    !(hdcp22_read(hdcp, HDCP22_CTL0) & HDCP22_ENCRYPT))
			return hdcp22_fail(hdcp, -EIO);
		return 1;
	}
	if (hdcp->auth_done && hdcp->receiver_checked &&
	    (!repeater || (hdcp->repeater_ready && hdcp->list_checked &&
			   hdcp->stream_done)) &&
	    time_after_eq(jiffies, hdcp->encrypt_after)) {
		if (status != 0x81 || state != 0x2b)
			return hdcp22_fail(hdcp, -EIO);
		hdcp22_update(hdcp, HDCP22_CTL0, HDCP22_ENCRYPT, HDCP22_ENCRYPT);
		hdcp22_update(hdcp, TPI_COPP_DATA2, HDCP22_TPI_ENCRYPT_DISABLE, 0);
		hdcp->encrypted = true;
		return 1;
	}
	if (time_after_eq(jiffies, hdcp->deadline))
		return hdcp22_fail(hdcp, -ETIMEDOUT);
	return 0;
}

MODULE_FIRMWARE(HDCP22_FIRMWARE);
