// SPDX-License-Identifier: GPL-2.0
/*
 * Hi3798CV200 imagev200 HDCP 1.4.
 *
 * Register and sequence authority: the CV200 BSP's drv_hdcp_v100.[ch],
 * drv_symc_v100.[ch], drv_osal_hi3798cv200.h and imagev200/driver/tx/hdcp/
 * si_mod_tx_hdcp.c. CRA addresses are (address - 0x3000) * 4, not MMIO
 * offsets (see imagev200/platform/platform.c).
 *
 * Firmware: hisilicon/hi3798cv200-hdcp14.bin is the licensed, board-specific
 * 332-byte encrypted key package, NOT a raw HDCP key. Bytes 1..3 are 00,H,I;
 * byte 0 & 0xc0 selects the provisioned OTP root (00) or HISI root (80).
 * Cipher0 decrypts the 320 bytes at offset 4 directly into TX14 key RAM.
 * Neither plaintext keys nor a CPU-supplied root key pass through this driver.
 *
 * The caller must reserve cipher0, including against secure-world users, and
 * keep cipher and HDMI clocks running and resets deasserted throughout calls.
 * CV200 cipher is at 0xf9a00000 (0x4000 bytes); its CRG is 0xf8a220c0,
 * reset bit 9 (the BSP marks clock control invalid). Nonsecure operation is
 * supported only when channel0 is nonsecure; secure routing is never bypassed.
 * No PRAM resource is used by HDCP 1.4. The caller serializes DDC/engine access,
 * stops HDCP 2.2 before starting this engine, and polls every 50 ms.
 */
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include <drm/display/drm_hdcp_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

#include "histb-hdmi-hdcp.h"

#define HDCP14_FIRMWARE "hisilicon/hi3798cv200-hdcp14.bin"
#define HDCP14_PACKAGE_SIZE 332
#define HDCP14_PAYLOAD_SIZE 320
#define HDCP14_KSV_SIZE 5
#define HDCP14_MAX_DEVICES 127
#define HDCP14_TIMEOUT_MS 5000

#define CRA(reg) (((reg) - 0x3000) * 4)
#define SYS_STAT CRA(0x3031)
#define SYS_CLOCK_STABLE BIT(0)
#define SYS_HPD BIT(1)
#define LM_DDC CRA(0x30cc)
#define SW_TPI_ENABLE BIT(7)
#define EPCM CRA(0x30f9)
#define LOAD_KSV BIT(5)
#define COPP_QUERY CRA(0x3629)
#define HDCP_CAPABLE BIT(1)
#define HDCP_REPEATER BIT(3)
#define LINK_STATUS GENMASK(5, 4)
#define LOCAL_SECURE BIT(6)
#define GLOBAL_SECURE BIT(7)
#define COPP_CTRL CRA(0x362a)
#define PROTECTION_REQUEST BIT(0)
#define DOUBLE_RI_CHECK BIT(2)
#define KSV_FORWARD BIT(4)
/* Set inhibits encryption; clear only after all software checks pass. */
#define ENCRYPTION_INHIBIT BIT(5)
#define CANCEL_PROTECTION BIT(6)
#define HDCP_PREPARE BIT(7)
#define PROTECTION_MODE (PROTECTION_REQUEST | DOUBLE_RI_CHECK | KSV_FORWARD)
#define BKSV_BASE CRA(0x362b)
#define KSV_V CRA(0x3631)
#define AUTH_STATE GENMASK(7, 6)
#define AUTH_REAUTH 0x40
#define AUTH_AUTHENTICATED 0x80
#define AKSV_BASE CRA(0x3636)
#define TPI_INTR_ENABLE CRA(0x363c)
#define TPI_INTR_STATUS CRA(0x363d)
#define INTR_BKSV_ERROR (BIT(0) | BIT(1))
#define INTR_FIFO_FIRST BIT(3)
#define INTR_LINK_CHANGE BIT(5)
#define INTR_V_READY BIT(6)
#define INTR_AUTH_CHANGE BIT(7)
#define FIFO_STATUS CRA(0x3641)
#define FIFO_BYTES GENMASK(4, 0)
#define FIFO_LAST BIT(7)
#define FIFO_PORT CRA(0x3642)
#define BSTATUS1 CRA(0x3645)
#define BSTATUS2 CRA(0x3646)
#define DEVICE_COUNT GENMASK(6, 0)
#define DEVICE_EXCEEDED BIT(7)
#define CASCADE_EXCEEDED BIT(3)
#define HW_DEBUG5 CRA(0x367d)
#define HW_STATE GENMASK(3, 0)
#define HW_AUTH_FAIL 8
#define HW_OPTION0 CRA(0x36b8)
#define HW_SM_RESET BIT(1)
#define HDCP22_POLL CRA(0x38b6)
#define HDCP22_POLL_ENABLE BIT(0)
#define SCDC_CTRL CRA(0x3920)
#define SCDC_AUTO_REPLY BIT(1)

/* v100 cipher register byte offsets and bitfields. */
#define CIPHER_HDCP_MODE 0x0820
#define CIPHER_HDCP_ENABLE BIT(0)
#define CIPHER_TX_READ BIT(1)
#define CIPHER_ROOT_SELECT GENMASK(3, 2)
#define CIPHER_RX_READ BIT(4)
#define CIPHER_RAM_SELECT GENMASK(6, 5)
#define CIPHER_RX22_READ BIT(7)
#define CIPHER_READ_ENABLE (CIPHER_TX_READ | CIPHER_RX_READ | CIPHER_RX22_READ)
#define CIPHER_SECURITY 0x0824
#define CIPHER_CH0_SECURE BIT(0)
#define CIPHER_CH0_CTRL 0x1000
#define CIPHER_CTRL_FIELDS GENMASK(16, 0)
#define CIPHER_DECRYPT BIT(0)
#define CIPHER_CBC FIELD_PREP_CONST(GENMASK(3, 1), 1)
#define CIPHER_AES FIELD_PREP_CONST(GENMASK(5, 4), 2)
#define CIPHER_IV_LOAD BIT(8)
#define CIPHER_CH0_IV 0x1004
#define CIPHER_CH0_INPUT 0x1014
#define CIPHER_CH0_CONFIG 0x1410
#define CIPHER_START BIT(0)
#define CIPHER_BUSY BIT(1)

enum hdcp14_state {
	HDCP14_OFF,
	HDCP14_PREPARE,
	HDCP14_AUTHENTICATING,
	HDCP14_READ_FIFO,
	HDCP14_WAIT_PART2,
	HDCP14_AUTHENTICATED,
};

struct histb_hdcp14 {
	struct histb_hdcp_io io;
	enum hdcp14_state state;
	unsigned long deadline;
	u8 ksvs[(HDCP14_MAX_DEVICES + 1) * HDCP14_KSV_SIZE]; /* SRM byte order */
	u16 fifo_read;
	u16 fifo_expected;
	u8 topology[2];
	bool repeater;
	bool fifo_first;
	bool fifo_last;
	bool scdc_saved;
	u8 scdc_auto_reply;
};

static u8 hdcp14_read(struct histb_hdcp14 *hdcp, u32 reg)
{
	return hdcp->io.read(hdcp->io.ctx, reg);
}

static void hdcp14_write(struct histb_hdcp14 *hdcp, u32 reg, u8 value)
{
	hdcp->io.write(hdcp->io.ctx, reg, value);
}

static void hdcp14_update(struct histb_hdcp14 *hdcp, u32 reg, u8 mask, u8 val)
{
	hdcp14_write(hdcp, reg, (hdcp14_read(hdcp, reg) & ~mask) | (val & mask));
}

/* Registers and the FIFO expose KSVs LSB-first; SRMs store them MSB-first. */
static void hdcp14_ksv_to_srm(u8 *ksv)
{
	unsigned int i;
	u8 tmp;

	for (i = 0; i < HDCP14_KSV_SIZE / 2; i++) {
		tmp = ksv[i];
		ksv[i] = ksv[HDCP14_KSV_SIZE - 1 - i];
		ksv[HDCP14_KSV_SIZE - 1 - i] = tmp;
	}
}

static void hdcp14_read_ksv(struct histb_hdcp14 *hdcp, u32 base, u8 *ksv)
{
	unsigned int i;

	for (i = 0; i < HDCP14_KSV_SIZE; i++)
		ksv[i] = hdcp14_read(hdcp, base + i * 4);
	hdcp14_ksv_to_srm(ksv);
}

static bool hdcp14_valid_ksv(const u8 *ksv)
{
	unsigned int i, ones = 0;

	for (i = 0; i < HDCP14_KSV_SIZE; i++)
		ones += hweight8(ksv[i]);
	return ones == 20;
}

static int hdcp14_load_keys(struct histb_hdcp14 *hdcp)
{
	const struct firmware *fw;
	void __iomem *cipher = hdcp->io.cipher;
	u32 security, ctrl, mode, value, root;
	unsigned int block, word;
	int ret;

	ret = request_firmware(&fw, HDCP14_FIRMWARE, hdcp->io.dev);
	if (ret)
		return ret;
	if (fw->size != HDCP14_PACKAGE_SIZE || fw->data[1] ||
	    fw->data[2] != 'H' || fw->data[3] != 'I') {
		ret = -EINVAL;
		goto release;
	}
	/* unf_hdcp.c: mpi_hdcp_load_key validates CRC32_1 over bytes0..327.
	 * CRC32_0 describes plaintext and cannot be checked by the host here.
	 */
	if ((crc32_le(~0U, fw->data, HDCP14_PACKAGE_SIZE - 4) ^ ~0U) !=
	    get_unaligned_le32(fw->data + HDCP14_PACKAGE_SIZE - 4)) {
		ret = -EBADMSG;
		goto release;
	}
	switch (fw->data[0] & 0xc0) {
	case 0x00:
		root = 0;
		break;
	case 0x80:
		root = 1;
		break;
	default:
		ret = -EINVAL;
		goto release;
	}

	if (readl(cipher + CIPHER_CH0_CONFIG) & CIPHER_BUSY) {
		ret = -EBUSY;
		goto release;
	}
	mode = readl(cipher + CIPHER_HDCP_MODE);
	if (mode & CIPHER_HDCP_ENABLE) {
		ret = -EBUSY;
		goto release;
	}

	/* Nonsecure channel zero only. Never probe by toggling secure ownership. */
	security = readl(cipher + CIPHER_SECURITY);
	if (security & CIPHER_CH0_SECURE) {
		ret = -EACCES;
		goto release;
	}
	ctrl = readl(cipher + CIPHER_CH0_CTRL);
	value = (mode & ~(CIPHER_ROOT_SELECT | CIPHER_RAM_SELECT |
			 CIPHER_TX_READ)) | CIPHER_HDCP_ENABLE |
		CIPHER_RX_READ | CIPHER_RX22_READ |
		FIELD_PREP(CIPHER_ROOT_SELECT, root);
	writel(value, cipher + CIPHER_HDCP_MODE);
	if ((readl(cipher + CIPHER_HDCP_MODE) & 0xff) != (value & 0xff)) {
		dev_err(hdcp->io.dev, "HDCP14 cipher RAM routing is unavailable\n");
		ret = -EACCES;
		goto restore;
	}
	for (word = 0; word < 16; word += 4)
		writel(0, cipher + CIPHER_CH0_IV + word);
	value = (ctrl & ~CIPHER_CTRL_FIELDS) | CIPHER_DECRYPT |
		CIPHER_CBC | CIPHER_AES | CIPHER_IV_LOAD;
	writel(value, cipher + CIPHER_CH0_CTRL);
	if ((readl(cipher + CIPHER_CH0_CTRL) & CIPHER_CTRL_FIELDS) !=
	    (value & CIPHER_CTRL_FIELDS)) {
		ret = -EACCES;
		goto restore;
	}
	for (block = 0; block < HDCP14_PAYLOAD_SIZE; block += 16) {
		for (word = 0; word < 16; word += 4)
			writel(get_unaligned_le32(fw->data + 4 + block + word),
			       cipher + CIPHER_CH0_INPUT + word);
		value = readl(cipher + CIPHER_CH0_CONFIG);
		writel(value | CIPHER_START, cipher + CIPHER_CH0_CONFIG);
		ret = readl_poll_timeout(cipher + CIPHER_CH0_CONFIG, value,
					 !(value & CIPHER_BUSY), 1, 5000);
		if (ret) {
			dev_err(hdcp->io.dev, "HDCP14 cipher0 timed out; reset required\n");
			goto restore;
		}
		/* Continue CBC chaining from the preceding block, not zero IV. */
		value = readl(cipher + CIPHER_CH0_CTRL);
		writel(value & ~CIPHER_IV_LOAD, cipher + CIPHER_CH0_CTRL);
	}
restore:
	/* Always close the hardware key-write path, including on timeouts. */
	value = readl(cipher + CIPHER_HDCP_MODE);
	value &= ~CIPHER_HDCP_ENABLE;
	value |= CIPHER_READ_ENABLE;
	writel(value, cipher + CIPHER_HDCP_MODE);
	writel(ctrl, cipher + CIPHER_CH0_CTRL);
release:
	release_firmware(fw);
	return ret;
}

void histb_hdcp14_stop(struct histb_hdcp14 *hdcp)
{
	if (hdcp->state == HDCP14_OFF)
		return;

	hdcp14_update(hdcp, COPP_CTRL,
		      ENCRYPTION_INHIBIT | HDCP_PREPARE | CANCEL_PROTECTION,
		      ENCRYPTION_INHIBIT | CANCEL_PROTECTION);
	usleep_range(5000, 6000);
	hdcp14_update(hdcp, COPP_CTRL, PROTECTION_MODE, 0);
	hdcp14_update(hdcp, EPCM, LOAD_KSV, 0);
	hdcp14_write(hdcp, TPI_INTR_ENABLE, 0);
	hdcp14_update(hdcp, HW_OPTION0, HW_SM_RESET, HW_SM_RESET);
	hdcp14_update(hdcp, HW_OPTION0, HW_SM_RESET, 0);
	hdcp14_write(hdcp, TPI_INTR_STATUS, 0xff);
	if (hdcp->scdc_saved) {
		hdcp14_update(hdcp, SCDC_CTRL, SCDC_AUTO_REPLY,
			      hdcp->scdc_auto_reply);
		hdcp->scdc_saved = false;
	}
	hdcp->state = HDCP14_OFF;
	hdcp->fifo_read = 0;
	hdcp->fifo_expected = 0;
	hdcp->fifo_first = false;
	hdcp->fifo_last = false;
	hdcp->repeater = false;
}

static int hdcp14_fail(struct histb_hdcp14 *hdcp, int error)
{
	histb_hdcp14_stop(hdcp);
	return error;
}

struct histb_hdcp14 *histb_hdcp14_create(struct histb_hdcp_io *io)
{
	struct histb_hdcp14 *hdcp;

	if (!io || !io->dev || !io->read || !io->write || !io->cipher)
		return ERR_PTR(-EINVAL);
	hdcp = devm_kzalloc(io->dev, sizeof(*hdcp), GFP_KERNEL);
	if (!hdcp)
		return ERR_PTR(-ENOMEM);
	hdcp->io = *io;
	return hdcp;
}

int histb_hdcp14_start(struct histb_hdcp14 *hdcp)
{
	u8 aksv[HDCP14_KSV_SIZE];
	int ret;

	histb_hdcp14_stop(hdcp);
	hdcp->state = HDCP14_PREPARE;
	hdcp->scdc_auto_reply = hdcp14_read(hdcp, SCDC_CTRL) & SCDC_AUTO_REPLY;
	hdcp->scdc_saved = true;
	hdcp14_update(hdcp, SCDC_CTRL, SCDC_AUTO_REPLY, 0);
	hdcp14_update(hdcp, COPP_CTRL, ENCRYPTION_INHIBIT, ENCRYPTION_INHIBIT);
	if ((hdcp14_read(hdcp, SYS_STAT) & (SYS_HPD | SYS_CLOCK_STABLE)) !=
	    (SYS_HPD | SYS_CLOCK_STABLE))
		return hdcp14_fail(hdcp, -ENOLINK);
	ret = hdcp14_load_keys(hdcp);
	if (ret)
		return hdcp14_fail(hdcp, ret);

	hdcp14_update(hdcp, HDCP22_POLL, HDCP22_POLL_ENABLE, 0);
	hdcp14_update(hdcp, LM_DDC, SW_TPI_ENABLE, 0);
	/* BSP workaround: TPI/DDC reset must not race an SCDC auto reply. */
	hdcp14_update(hdcp, SCDC_CTRL, SCDC_AUTO_REPLY, 0);
	hdcp14_update(hdcp, HW_OPTION0, HW_SM_RESET, HW_SM_RESET);
	hdcp14_update(hdcp, HW_OPTION0, HW_SM_RESET, 0);
	hdcp14_write(hdcp, TPI_INTR_STATUS, 0xff);
	/* LOAD_KSV reads the cipher-loaded RAM; this is not an OTP write. */
	hdcp14_update(hdcp, EPCM, LOAD_KSV, LOAD_KSV);
	msleep(20);
	hdcp14_update(hdcp, EPCM, LOAD_KSV, 0);
	hdcp14_read_ksv(hdcp, AKSV_BASE, aksv);
	if (!hdcp14_valid_ksv(aksv)) {
		dev_err(hdcp->io.dev, "HDCP14 key RAM has no valid AKSV\n");
		return hdcp14_fail(hdcp, -EKEYREJECTED);
	}
	hdcp14_update(hdcp, COPP_CTRL, HDCP_PREPARE, HDCP_PREPARE);
	hdcp->state = HDCP14_PREPARE;
	hdcp->deadline = jiffies + msecs_to_jiffies(HDCP14_TIMEOUT_MS);
	return 0;
}

static int hdcp14_check_ksvs(struct histb_hdcp14 *hdcp,
			     struct drm_connector *connector)
{
	const struct firmware *srm;
	unsigned int count = 1 + hdcp->fifo_expected / HDCP14_KSV_SIZE;
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++)
		if (!hdcp14_valid_ksv(hdcp->ksvs + i * HDCP14_KSV_SIZE))
			return -EKEYREJECTED;
	/* The DRM helper intentionally treats a missing SRM as an empty list.
	 * Require the SRM here so success actually includes a revocation check.
	 */
	ret = request_firmware_direct(&srm, "display_hdcp_srm.bin", connector->dev->dev);
	if (ret)
		return ret;
	ret = histb_hdcp_check_revocation(srm->data, srm->size, hdcp->ksvs, count);
	release_firmware(srm);
	return ret;
}

static int hdcp14_topology(struct histb_hdcp14 *hdcp, bool capture)
{
	u8 status[2];

	status[0] = hdcp14_read(hdcp, BSTATUS1);
	status[1] = hdcp14_read(hdcp, BSTATUS2);
	if ((status[0] & DEVICE_EXCEEDED) || (status[1] & CASCADE_EXCEEDED))
		return -EOVERFLOW;
	if (!capture && (status[0] != hdcp->topology[0] ||
			 status[1] != hdcp->topology[1]))
		return -EAGAIN;
	if (capture) {
		hdcp->topology[0] = status[0];
		hdcp->topology[1] = status[1];
		hdcp->fifo_expected = (status[0] & DEVICE_COUNT) * HDCP14_KSV_SIZE;
	}
	return 0;
}

static int hdcp14_read_fifo(struct histb_hdcp14 *hdcp)
{
	unsigned int chunk, i, bytes;
	u8 status;

	/* A full legal topology is at most 635 bytes; never spin for a sink. */
	for (chunk = 0; chunk <= HDCP14_MAX_DEVICES * HDCP14_KSV_SIZE; chunk++) {
		status = hdcp14_read(hdcp, FIFO_STATUS);
		bytes = status & FIFO_BYTES;
		hdcp->fifo_last |= !!(status & FIFO_LAST);
		if (bytes > hdcp->fifo_expected - hdcp->fifo_read)
			return -EPROTO;
		for (i = 0; i < bytes; i++) {
			hdcp->ksvs[HDCP14_KSV_SIZE + hdcp->fifo_read++] =
				hdcp14_read(hdcp, FIFO_PORT);
			/* A FIFO chunk can end in the middle of a KSV. */
			if (!(hdcp->fifo_read % HDCP14_KSV_SIZE))
				hdcp14_ksv_to_srm(hdcp->ksvs + hdcp->fifo_read);
		}
		if (hdcp->fifo_read == hdcp->fifo_expected && hdcp->fifo_last)
			return 1;
		if (!bytes)
			return 0;
	}
	return -EPROTO;
}

int histb_hdcp14_poll(struct histb_hdcp14 *hdcp, struct drm_connector *connector)
{
	u8 query, intr, auth, hw;
	int ret;

	if (hdcp->state == HDCP14_OFF)
		return hdcp14_fail(hdcp, -EINVAL);
	if (!connector || !connector->dev)
		return hdcp14_fail(hdcp, -EINVAL);
	if ((hdcp14_read(hdcp, SYS_STAT) & (SYS_HPD | SYS_CLOCK_STABLE)) !=
	    (SYS_HPD | SYS_CLOCK_STABLE))
		return hdcp14_fail(hdcp, -ENOLINK);

	intr = hdcp14_read(hdcp, TPI_INTR_STATUS);
	hdcp14_write(hdcp, TPI_INTR_STATUS, intr);
	hdcp->fifo_first |= !!(intr & INTR_FIFO_FIRST);
	query = hdcp14_read(hdcp, COPP_QUERY);
	auth = hdcp14_read(hdcp, KSV_V) & AUTH_STATE;
	hw = hdcp14_read(hdcp, HW_DEBUG5) & HW_STATE;
	if (intr & INTR_BKSV_ERROR)
		return hdcp14_fail(hdcp, -EIO);
	if (hdcp->state != HDCP14_PREPARE &&
	    ((query & LINK_STATUS) || auth == AUTH_REAUTH || hw == HW_AUTH_FAIL))
		return hdcp14_fail(hdcp, -EAGAIN);
	if (hdcp->state >= HDCP14_READ_FIFO &&
	    (!(query & LOCAL_SECURE) ||
	     !!(query & HDCP_REPEATER) != hdcp->repeater))
		return hdcp14_fail(hdcp, -EAGAIN);
	if (hdcp->state != HDCP14_AUTHENTICATED &&
	    time_after_eq(jiffies, hdcp->deadline))
		return hdcp14_fail(hdcp, -ETIMEDOUT);

	switch (hdcp->state) {
	case HDCP14_PREPARE:
		if (!(query & HDCP_CAPABLE))
			return 0;
		hdcp14_read_ksv(hdcp, BKSV_BASE, hdcp->ksvs);
		if (!hdcp14_valid_ksv(hdcp->ksvs))
			return hdcp14_fail(hdcp, -EKEYREJECTED);
		hdcp14_update(hdcp, COPP_CTRL, CANCEL_PROTECTION, CANCEL_PROTECTION);
		usleep_range(5000, 6000);
		hdcp14_update(hdcp, COPP_CTRL, PROTECTION_MODE, 0);
		hdcp14_update(hdcp, COPP_CTRL, CANCEL_PROTECTION, 0);
		hdcp14_update(hdcp, COPP_CTRL, PROTECTION_MODE, PROTECTION_MODE);
		hdcp14_write(hdcp, TPI_INTR_ENABLE, INTR_FIFO_FIRST | INTR_V_READY |
				 INTR_AUTH_CHANGE | INTR_LINK_CHANGE | INTR_BKSV_ERROR);
		hdcp->state = HDCP14_AUTHENTICATING;
		hdcp->deadline = jiffies + msecs_to_jiffies(HDCP14_TIMEOUT_MS);
		return 0;
	case HDCP14_AUTHENTICATING:
		if (!(query & LOCAL_SECURE))
			return 0;
		hdcp->repeater = !!(query & HDCP_REPEATER);
		/* Re-read the actual authenticated BKSV, not just the prep copy. */
		hdcp14_read_ksv(hdcp, BKSV_BASE, hdcp->ksvs);
		if (!hdcp->repeater) {
			if (auth != AUTH_AUTHENTICATED)
				return 0;
			ret = hdcp14_check_ksvs(hdcp, connector);
			if (ret)
				return hdcp14_fail(hdcp, ret);
			hdcp->state = HDCP14_AUTHENTICATED;
			break;
		}
		/* BSTATUS/FIFO may not be available until the first-byte event. */
		hdcp->state = HDCP14_READ_FIFO;
		hdcp->deadline = jiffies + msecs_to_jiffies(HDCP14_TIMEOUT_MS);
		fallthrough;
	case HDCP14_READ_FIFO:
		if (!hdcp->fifo_first &&
		    (query & (LOCAL_SECURE | GLOBAL_SECURE)) !=
		    (LOCAL_SECURE | GLOBAL_SECURE))
			return 0;
		if (!hdcp->fifo_read) {
			ret = hdcp14_topology(hdcp, true);
			if (ret)
				return hdcp14_fail(hdcp, ret);
		}
		ret = hdcp14_topology(hdcp, false);
		if (ret)
			return hdcp14_fail(hdcp, ret);
		/* A zero-device repeater has no FIFO bytes; still require part2. */
		if (hdcp->fifo_expected) {
			ret = hdcp14_read_fifo(hdcp);
			if (ret < 0)
				return hdcp14_fail(hdcp, ret);
			if (!ret)
				return 0;
		}
		ret = hdcp14_check_ksvs(hdcp, connector);
		if (ret)
			return hdcp14_fail(hdcp, ret);
		hdcp->state = HDCP14_WAIT_PART2;
		hdcp->deadline = jiffies + msecs_to_jiffies(HDCP14_TIMEOUT_MS);
		fallthrough;
	case HDCP14_WAIT_PART2:
		ret = hdcp14_topology(hdcp, false);
		if (ret)
			return hdcp14_fail(hdcp, ret);
		if ((query & (LOCAL_SECURE | GLOBAL_SECURE)) !=
		    (LOCAL_SECURE | GLOBAL_SECURE) || auth != AUTH_AUTHENTICATED)
			return 0;
		hdcp->state = HDCP14_AUTHENTICATED;
		break;
	case HDCP14_AUTHENTICATED:
		if (!(query & LOCAL_SECURE) || auth != AUTH_AUTHENTICATED ||
		    !!(query & HDCP_REPEATER) != hdcp->repeater)
			return hdcp14_fail(hdcp, -EAGAIN);
		if (hdcp->repeater) {
			if (!(query & GLOBAL_SECURE) || (intr & INTR_FIFO_FIRST))
				return hdcp14_fail(hdcp, -EAGAIN);
			ret = hdcp14_topology(hdcp, false);
			if (ret)
				return hdcp14_fail(hdcp, ret);
		}
		return 1;
	default:
		return hdcp14_fail(hdcp, -EINVAL);
	}

	/* Firmware/SRM loading can sleep. Do not enable from stale link state. */
	if (time_after_eq(jiffies, hdcp->deadline))
		return hdcp14_fail(hdcp, -ETIMEDOUT);
	query = hdcp14_read(hdcp, COPP_QUERY);
	auth = hdcp14_read(hdcp, KSV_V) & AUTH_STATE;
	intr = hdcp14_read(hdcp, TPI_INTR_STATUS);
	if ((query & LINK_STATUS) || !(query & LOCAL_SECURE) ||
	    auth != AUTH_AUTHENTICATED ||
	    !!(query & HDCP_REPEATER) != hdcp->repeater ||
	    (intr & (INTR_BKSV_ERROR | INTR_LINK_CHANGE)))
		return hdcp14_fail(hdcp, -EAGAIN);
	if ((hdcp14_read(hdcp, SYS_STAT) & (SYS_HPD | SYS_CLOCK_STABLE)) !=
	    (SYS_HPD | SYS_CLOCK_STABLE))
		return hdcp14_fail(hdcp, -ENOLINK);
	if (hdcp->repeater) {
		if (!(query & GLOBAL_SECURE))
			return hdcp14_fail(hdcp, -EAGAIN);
		ret = hdcp14_topology(hdcp, false);
		if (ret)
			return hdcp14_fail(hdcp, ret);
	}
	/* Only this path can turn on encryption. */
	hdcp14_update(hdcp, COPP_CTRL, ENCRYPTION_INHIBIT, 0);
	return 1;
}

MODULE_FIRMWARE(HDCP14_FIRMWARE);
