// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3798CV200 VEDU H.264 encoder
 *
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 */

#include <linux/bitfield.h>
#include <linux/build_bug.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/workqueue.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "histb-venc-rc.h"

#define HISTB_VENC_INTSTAT		0x0000
#define HISTB_VENC_INTMASK		0x0004
#define HISTB_VENC_INTCLR		0x000c
#define HISTB_VENC_START		0x0010
#define HISTB_VENC_MODE			0x0020
#define HISTB_VENC_PICFG0		0x0024
#define HISTB_VENC_PICFG1		0x0028
#define HISTB_VENC_PICFG2		0x002c
#define HISTB_VENC_PICFG3		0x0030
#define HISTB_VENC_PICFG4		0x0034
#define HISTB_VENC_PICFG5		0x0038
#define HISTB_VENC_ENCUPDATE		0x0040
#define HISTB_VENC_IMAGE_SIZE		0x0050
#define HISTB_VENC_PTBITS		0x0058
#define HISTB_VENC_SLICE_HEADER(n)	(0x0060 + (n) * 4)
#define HISTB_VENC_SLICE_HEADER_PARAM	0x0080
#define HISTB_VENC_PTS0			0x0090
#define HISTB_VENC_PTS1			0x0094
#define HISTB_VENC_PTS2			0x0098
#define HISTB_VENC_PTS3			0x009c
#define HISTB_VENC_TIMEOUT		0x00a0
#define HISTB_VENC_OUTSTANDING		0x00a4
#define HISTB_VENC_TIMER		0x00a8
#define HISTB_VENC_IDLE_TIMER		0x00ac
#define HISTB_VENC_VERSION0		0x00f8
#define HISTB_VENC_SRC_Y		0x0100
#define HISTB_VENC_SRC_C		0x0104
#define HISTB_VENC_SRC_V		0x0108
#define HISTB_VENC_RECON_Y		0x0110
#define HISTB_VENC_RECON_C		0x0114
#define HISTB_VENC_REF_Y		0x0120
#define HISTB_VENC_REF_C		0x0124
#define HISTB_VENC_SRC_STRIDE		0x0130
#define HISTB_VENC_RECON_STRIDE		0x0134
#define HISTB_VENC_STREAM_ADDR		0x0140
#define HISTB_VENC_STREAM_RPTR_ADDR	0x0144
#define HISTB_VENC_STREAM_WPTR_ADDR	0x0148
#define HISTB_VENC_STREAM_LEN		0x014c
#define HISTB_VENC_ME_CFG0		0x0200
#define HISTB_VENC_ME_CFG1		0x0204
#define HISTB_VENC_ME_RECT(n)		(0x0208 + (n) * 4)
#define HISTB_VENC_ME_THRESHOLD0	0x0220
#define HISTB_VENC_ME_THRESHOLD1	0x0224
#define HISTB_VENC_ME_RDO		0x0228
#define HISTB_VENC_CREF_MODE		0x022c
#define HISTB_VENC_MD_CFG		0x0230
#define HISTB_VENC_MCTF_CFG0		0x02a0
#define HISTB_VENC_ROI_CFG		0x0300
#define HISTB_VENC_QP_THRESHOLD		0x0350
#define HISTB_VENC_RC_CFG		0x0354
#define HISTB_VENC_QP_DELTA0		0x0358
#define HISTB_VENC_QP_DELTA1		0x035c
#define HISTB_VENC_QP_DELTA2		0x0360
#define HISTB_VENC_TARGET_BITS		0x0364
#define HISTB_VENC_PICTURE_BITS		0x0374
#define HISTB_VENC_MEAN_QP		0x0388
#define HISTB_VENC_PICINFO1		0x0a24
#define HISTB_VENC_PICINFO6		0x0a38
#define HISTB_VENC_PICINFO7		0x0a3c
#define HISTB_VENC_PICINFO8		0x0a40
#define HISTB_VENC_MODE_LAMBDA(n)	(0x03a0 + (n) * 4)
#define HISTB_VENC_OSD_CFG		0x0500
#define HISTB_VENC_SECURE		0x05d8
#define HISTB_VENC_CHANNEL_BYPASS	0x05dc
#define HISTB_VENC_SCALE(n)		(0x5e00 + (n) * 4)
#define HISTB_VENC_SMMU_CTRL		0xf000
#define HISTB_VENC_SMMU_INTMASK_NS	0xf020

#define HISTB_VENC_INT_DONE		BIT(0)
#define HISTB_VENC_INT_BUF_FULL		BIT(1)
#define HISTB_VENC_INT_BITS_OVERFLOW	BIT(3)
#define HISTB_VENC_INT_TIMEOUT		BIT(27)
#define HISTB_VENC_INT_CFG_ERROR	BIT(30)
#define HISTB_VENC_INT_ALL		(HISTB_VENC_INT_DONE | \
					 HISTB_VENC_INT_BUF_FULL | \
					 HISTB_VENC_INT_BITS_OVERFLOW | \
					 HISTB_VENC_INT_TIMEOUT | \
					 HISTB_VENC_INT_CFG_ERROR)
#define HISTB_VENC_INT_ERRORS		(HISTB_VENC_INT_ALL & \
					 ~HISTB_VENC_INT_DONE)

#define HISTB_VENC_MODE_ENABLE		BIT(1)
#define HISTB_VENC_MODE_TIMEOUT		GENMASK(11, 10)
#define HISTB_VENC_MODE_CLOCK_GATE	GENMASK(14, 13)
#define HISTB_VENC_MODE_MEMORY_GATE	BIT(15)
#define HISTB_VENC_MODE_PACKAGE_SEL	GENMASK(24, 17)
#define HISTB_VENC_PACKAGE_U_V		1

#define HISTB_VENC_PICFG0_PTBITS_ENABLE	BIT(8)
#define HISTB_VENC_PICFG0_I_PICTURE	BIT(13)
#define HISTB_VENC_PICFG0_TRANSFORM	GENMASK(15, 14)
#define HISTB_VENC_PICFG0_NAL_REF_IDC	GENMASK(17, 16)
#define HISTB_VENC_PICFG0_ENTROPY_CABAC	BIT(19)
#define HISTB_VENC_PICFG2_IPCM		BIT(0)
#define HISTB_VENC_PICFG2_INTRA_4X4	BIT(1)
#define HISTB_VENC_PICFG2_INTRA_16X16	BIT(3)
#define HISTB_VENC_PICFG2_LOW_POWER	BIT(31)
#define HISTB_VENC_PICFG3_INTER_MODES	GENMASK(4, 0)
#define HISTB_VENC_PICFG3_FRAC_LOW_POWER BIT(29)
#define HISTB_VENC_PICFG3_INTP_LOW_POWER BIT(30)
#define HISTB_VENC_PICFG3_EXTENDED_EDGE	BIT(31)
#define HISTB_VENC_IMAGE_WIDTH		GENMASK(12, 0)
#define HISTB_VENC_IMAGE_HEIGHT		GENMASK(28, 16)
#define HISTB_VENC_SLICE_IDR		BIT(1)
#define HISTB_VENC_SLICE_PARAM_WORDS	GENMASK(14, 8)
#define HISTB_VENC_SLICE_REORDER_WORDS	GENMASK(21, 16)
#define HISTB_VENC_SLICE_MARKING_WORDS	GENMASK(27, 22)
#define HISTB_VENC_OUTSTANDING_WRITE	GENMASK(2, 0)
#define HISTB_VENC_OUTSTANDING_READ	GENMASK(11, 8)
#define HISTB_VENC_ME_H_SEARCH		GENMASK(3, 0)
#define HISTB_VENC_ME_V_SEARCH		GENMASK(6, 4)
#define HISTB_VENC_ME_FRAC_THRESHOLD	GENMASK(19, 16)
#define HISTB_VENC_QP_MIN		GENMASK(5, 0)
#define HISTB_VENC_QP_MAX		GENMASK(13, 8)
#define HISTB_VENC_QP_START		GENMASK(21, 16)
#define HISTB_VENC_RC_QP_DELTA		GENMASK(5, 0)
#define HISTB_VENC_RC_MADP_DELTA	GENMASK(15, 8)
#define HISTB_VENC_LAMBDA_LOW		GENMASK(11, 0)
#define HISTB_VENC_LAMBDA_HIGH		GENMASK(23, 12)
#define HISTB_VENC_SMMU_BYPASS		BIT(0)

#define HISTB_VENC_MIN_WIDTH		176
#define HISTB_VENC_MIN_HEIGHT		144
#define HISTB_VENC_MAX_WIDTH		1920
#define HISTB_VENC_MAX_HEIGHT		1088
#define HISTB_VENC_MAX_STRIDE		8192
#define HISTB_VENC_DEFAULT_WIDTH	1280
#define HISTB_VENC_DEFAULT_HEIGHT	720
#define HISTB_VENC_DEFAULT_QP		26
#define HISTB_VENC_DEFAULT_BITRATE	(4 * 1024 * 1024)
#define HISTB_VENC_DEFAULT_FPS		25
#define HISTB_VENC_DEFAULT_MIN_QP	10
#define HISTB_VENC_DEFAULT_MAX_QP	50
#define HISTB_VENC_MIN_BITRATE		(32 * 1024)
#define HISTB_VENC_MAX_BITRATE		(50 * 1024 * 1024)
#define HISTB_VENC_CORE_RATE		200000000UL
#define HISTB_VENC_STREAM_META_SIZE	256
#define HISTB_VENC_STREAM_DESC_SIZE	64
#define HISTB_VENC_STREAM_ALIGN		64
#define HISTB_VENC_STREAM_MIN_SIZE	SZ_256K
#define HISTB_VENC_STREAM_MAX_SIZE	(20 * SZ_1M)
#define HISTB_VENC_HEADER_MAX		64
#define HISTB_VENC_WATCHDOG_MS		1000

struct histb_venc_stream_desc {
	__le32 packet_len;
	__le32 invalid_bytes;
	u8 type;
	u8 bottom_field;
	u8 field;
	u8 last_slice;
	__le32 channel;
	__le32 pts[4];
	__le32 reserved[8];
};

static_assert(sizeof(struct histb_venc_stream_desc) ==
	      HISTB_VENC_STREAM_DESC_SIZE);

struct histb_venc_q_data {
	struct v4l2_pix_format pix;
	u32 sequence;
};

struct histb_venc_dev;

struct histb_venc_ctx {
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler ctrl_handler;
	struct histb_venc_dev *venc;
	struct histb_venc_q_data src;
	struct histb_venc_q_data dst;
	struct v4l2_rect crop;
	void *recon_cpu;
	dma_addr_t recon_dma;
	size_t recon_size;
	size_t recon_frame_size;
	u32 i_qp;
	u32 p_qp;
	u32 gop_size;
	u32 bitrate;
	u32 profile;
	u32 min_qp;
	u32 max_qp;
	struct v4l2_fract timeperframe;
	struct histb_venc_rc rc;
	struct histb_venc_rc_decision rc_decision;
	u32 frame_num;
	u32 idr_pic_id;
	bool frame_rc;
	bool rc_initialized;
	bool rc_dirty;
	bool rc_job_pending;
	bool force_idr;
	bool curr_intra;
	bool job_aborting;
};

struct histb_venc_dev {
	struct device *dev;
	void __iomem *regs;
	struct clk_bulk_data clocks[2];
	struct reset_control *reset;
	int irq;

	struct v4l2_device v4l2_dev;
	struct video_device vfd;
	struct v4l2_m2m_dev *m2m_dev;
	/* Serializes V4L2 ioctls and both vb2 queues. */
	struct mutex lock;

	/* Protects the active hardware context against IRQ and watchdog races. */
	spinlock_t irqlock;
	struct histb_venc_ctx *curr_ctx;
	/* Serializes synchronous setup/launch with IRQ and streamoff teardown. */
	struct mutex launch_lock;
	struct delayed_work watchdog_work;
	u64 watchdog_deadline_ns;
	bool engine_faulted;
	/* Count each active hardware job once, shared with sysfs readers. */
	bool job_count_pending;
	u64 jobs;
	u64 errors;
};

static const struct v4l2_event histb_venc_eos_event = {
	.type = V4L2_EVENT_EOS,
};

struct histb_venc_bit_writer {
	u8 *data;
	size_t size;
	size_t bitpos;
	bool overflow;
};

struct histb_venc_reg_bits {
	u32 value;
	u8 count;
};

static const u16 histb_venc_mode_lambda[] = {
	1, 1, 1, 2, 2, 3, 3, 4, 5, 7,
	9, 11, 14, 17, 22, 27, 34, 43, 54, 69,
	86, 109, 137, 173, 218, 274, 345, 435, 548, 691,
	870, 1097, 1382, 1741, 2193, 2763, 3482, 4095, 4095, 4095,
};

static const u8 histb_venc_scaling_list[] = {
	6, 10, 13, 16, 18, 23, 25, 27,
	10, 11, 16, 18, 23, 25, 27, 29,
	13, 16, 18, 23, 25, 27, 29, 31,
	16, 18, 23, 25, 27, 29, 31, 33,
	18, 23, 25, 27, 29, 31, 33, 36,
	23, 25, 27, 29, 31, 33, 36, 38,
	25, 27, 29, 31, 33, 36, 38, 40,
	27, 29, 31, 33, 36, 38, 40, 42,
	9, 13, 15, 17, 19, 21, 22, 24,
	13, 13, 17, 19, 21, 22, 24, 25,
	15, 17, 19, 21, 22, 24, 25, 27,
	17, 19, 21, 22, 24, 25, 27, 28,
	19, 21, 22, 24, 25, 27, 28, 30,
	21, 22, 24, 25, 27, 28, 30, 32,
	22, 24, 25, 27, 28, 30, 32, 33,
	24, 25, 27, 28, 30, 32, 33, 35,
};

static struct histb_venc_q_data *
histb_venc_get_q_data(struct histb_venc_ctx *ctx, enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->src;

	return &ctx->dst;
}

static void histb_venc_put_bits(struct histb_venc_bit_writer *writer,
				u32 value, unsigned int count)
{
	int bit;

	if (!count)
		return;

	for (bit = count - 1; bit >= 0; bit--) {
		size_t byte = writer->bitpos / 8;
		unsigned int shift = 7 - writer->bitpos % 8;

		if (byte >= writer->size) {
			writer->overflow = true;
			return;
		}
		writer->data[byte] |= ((value >> bit) & 1) << shift;
		writer->bitpos++;
	}
}

static void histb_venc_put_ue(struct histb_venc_bit_writer *writer, u32 value)
{
	u32 code = value + 1;
	unsigned int bits = fls(code);

	histb_venc_put_bits(writer, 0, bits - 1);
	histb_venc_put_bits(writer, code, bits);
}

static void histb_venc_put_se(struct histb_venc_bit_writer *writer, s32 value)
{
	u32 code = value <= 0 ? -2 * value : 2 * value - 1;

	histb_venc_put_ue(writer, code);
}

static int histb_venc_finish_nal(struct histb_venc_bit_writer *writer)
{
	histb_venc_put_bits(writer, 1, 1);
	while (writer->bitpos % 8)
		histb_venc_put_bits(writer, 0, 1);

	return writer->overflow ? -ENOSPC : DIV_ROUND_UP(writer->bitpos, 8);
}

static u8 histb_venc_level_idc(unsigned int width, unsigned int height)
{
	unsigned int macroblocks = DIV_ROUND_UP(width, 16) *
				   DIV_ROUND_UP(height, 16);

	if (macroblocks <= 99)
		return 10;
	if (macroblocks <= 396)
		return 20;
	if (macroblocks <= 792)
		return 21;
	if (macroblocks <= 1620)
		return 30;
	if (macroblocks <= 3600)
		return 31;
	if (macroblocks <= 5120)
		return 32;

	return 41;
}

static int histb_venc_build_sps(struct histb_venc_ctx *ctx, u8 *data,
				unsigned int size)
{
	struct histb_venc_bit_writer writer = { .data = data, .size = size };
	unsigned int width_mb = DIV_ROUND_UP(ctx->src.pix.width, 16);
	unsigned int height_mb = DIV_ROUND_UP(ctx->src.pix.height, 16);
	unsigned int crop_right = (width_mb * 16 - ctx->crop.width) / 2;
	unsigned int crop_bottom = (height_mb * 16 - ctx->crop.height) / 2;
	bool cropped = crop_right || crop_bottom;

	memset(data, 0, size);
	histb_venc_put_bits(&writer, 1, 32);
	histb_venc_put_bits(&writer, 0x67, 8);
	histb_venc_put_bits(&writer, 66, 8);
	histb_venc_put_bits(&writer, 0, 8);
	histb_venc_put_bits(&writer,
			    histb_venc_level_idc(ctx->src.pix.width,
						 ctx->src.pix.height), 8);
	histb_venc_put_ue(&writer, 0);
	histb_venc_put_ue(&writer, 0);
	histb_venc_put_ue(&writer, 2);
	histb_venc_put_ue(&writer, 1);
	histb_venc_put_bits(&writer, 0, 1);
	histb_venc_put_ue(&writer, width_mb - 1);
	histb_venc_put_ue(&writer, height_mb - 1);
	histb_venc_put_bits(&writer, 1, 1);
	histb_venc_put_bits(&writer, width_mb * height_mb >= 1620, 1);
	histb_venc_put_bits(&writer, cropped, 1);
	if (cropped) {
		histb_venc_put_ue(&writer, 0);
		histb_venc_put_ue(&writer, crop_right);
		histb_venc_put_ue(&writer, 0);
		histb_venc_put_ue(&writer, crop_bottom);
	}
	histb_venc_put_bits(&writer, 0, 1);

	return histb_venc_finish_nal(&writer);
}

static int histb_venc_build_pps(u8 *data, unsigned int size)
{
	struct histb_venc_bit_writer writer = { .data = data, .size = size };

	memset(data, 0, size);
	histb_venc_put_bits(&writer, 1, 32);
	histb_venc_put_bits(&writer, 0x68, 8);
	histb_venc_put_ue(&writer, 0);
	histb_venc_put_ue(&writer, 0);
	histb_venc_put_bits(&writer, 0, 1);
	histb_venc_put_bits(&writer, 0, 1);
	histb_venc_put_ue(&writer, 0);
	histb_venc_put_ue(&writer, 0);
	histb_venc_put_ue(&writer, 0);
	histb_venc_put_bits(&writer, 0, 1);
	histb_venc_put_bits(&writer, 0, 2);
	histb_venc_put_se(&writer, 0);
	histb_venc_put_se(&writer, 0);
	histb_venc_put_se(&writer, 0);
	histb_venc_put_bits(&writer, 1, 1);
	histb_venc_put_bits(&writer, 0, 1);
	histb_venc_put_bits(&writer, 0, 1);

	return histb_venc_finish_nal(&writer);
}

static void histb_venc_reg_put_bits(struct histb_venc_reg_bits *bits,
				    u32 value, unsigned int count)
{
	if (!count)
		return;

	bits->value = bits->value << count | (value & GENMASK(count - 1, 0));
	bits->count += count;
}

static void histb_venc_reg_put_ue(struct histb_venc_reg_bits *bits, u32 value)
{
	u32 code = value + 1;
	unsigned int count = fls(code);

	histb_venc_reg_put_bits(bits, 0, count - 1);
	histb_venc_reg_put_bits(bits, code, count);
}

static u32 histb_venc_build_slice_header(struct histb_venc_ctx *ctx,
					 u32 header[8], bool intra)
{
	struct histb_venc_reg_bits params = {};
	struct histb_venc_reg_bits reorder = {};
	struct histb_venc_reg_bits marking = {};

	memset(header, 0, sizeof(u32) * 8);
	histb_venc_reg_put_ue(&params, intra ? 2 : 0);
	histb_venc_reg_put_ue(&params, 0);
	histb_venc_reg_put_bits(&params, intra ? 0 : ctx->frame_num, 4);
	if (intra)
		histb_venc_reg_put_ue(&params, ctx->idr_pic_id++ & 0xf);
	else
		histb_venc_reg_put_bits(&params, 0, 1);
	histb_venc_reg_put_bits(&reorder, 0, 1);
	histb_venc_reg_put_bits(&marking, 0, intra ? 2 : 1);
	header[0] = params.value;
	header[4] = reorder.value;
	header[6] = marking.value;

	return (intra ? HISTB_VENC_SLICE_IDR : 0) |
		FIELD_PREP(HISTB_VENC_SLICE_PARAM_WORDS, params.count - 1) |
		FIELD_PREP(HISTB_VENC_SLICE_REORDER_WORDS,
			   reorder.count - 1) |
		FIELD_PREP(HISTB_VENC_SLICE_MARKING_WORDS,
			   marking.count - 1);
}

static void histb_venc_account_job(struct histb_venc_dev *venc, bool error)
{
	unsigned long flags;

	spin_lock_irqsave(&venc->irqlock, flags);
	if (venc->job_count_pending) {
		venc->job_count_pending = false;
		venc->jobs++;
		if (error)
			venc->errors++;
	}
	spin_unlock_irqrestore(&venc->irqlock, flags);
}

static ssize_t histb_venc_jobs_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct histb_venc_dev *venc = dev_get_drvdata(dev);
	unsigned long flags;
	u64 jobs;

	spin_lock_irqsave(&venc->irqlock, flags);
	jobs = venc->jobs;
	spin_unlock_irqrestore(&venc->irqlock, flags);

	return sysfs_emit(buf, "%llu\n", jobs);
}

static ssize_t histb_venc_errors_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct histb_venc_dev *venc = dev_get_drvdata(dev);
	unsigned long flags;
	u64 errors;

	spin_lock_irqsave(&venc->irqlock, flags);
	errors = venc->errors;
	spin_unlock_irqrestore(&venc->irqlock, flags);

	return sysfs_emit(buf, "%llu\n", errors);
}

static DEVICE_ATTR_RO(histb_venc_jobs);
static DEVICE_ATTR_RO(histb_venc_errors);

static struct attribute *histb_venc_attrs[] = {
	&dev_attr_histb_venc_jobs.attr,
	&dev_attr_histb_venc_errors.attr,
	NULL,
};

static const struct attribute_group histb_venc_attr_group = {
	.attrs = histb_venc_attrs,
};

static int histb_venc_reset_engine(struct histb_venc_dev *venc)
{
	int ret;

	lockdep_assert_held(&venc->launch_lock);
	writel(0, venc->regs + HISTB_VENC_INTMASK);
	writel(HISTB_VENC_INT_ALL, venc->regs + HISTB_VENC_INTCLR);
	ret = reset_control_assert(venc->reset);
	if (ret)
		goto fault;
	udelay(5);
	clk_bulk_disable_unprepare(ARRAY_SIZE(venc->clocks), venc->clocks);
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(venc->clocks), venc->clocks);
	if (ret)
		goto fault;
	ret = reset_control_deassert(venc->reset);
	if (ret)
		goto fault;
	udelay(10);
	writel(0, venc->regs + HISTB_VENC_INTMASK);
	writel(HISTB_VENC_INT_ALL, venc->regs + HISTB_VENC_INTCLR);
	WRITE_ONCE(venc->engine_faulted, false);

	return 0;

fault:
	/* A failed recovery must not leave a deasserted engine able to DMA. */
	reset_control_assert(venc->reset);
	clk_bulk_disable_unprepare(ARRAY_SIZE(venc->clocks), venc->clocks);
	WRITE_ONCE(venc->engine_faulted, true);
	return ret;
}

static int histb_venc_configure_rc(struct histb_venc_ctx *ctx)
{
	struct histb_venc_rc_config config = {
		.bitrate = ctx->bitrate,
		.fps_num = ctx->timeperframe.denominator,
		.fps_den = ctx->timeperframe.numerator,
		.gop_size = ctx->gop_size,
		.width = ctx->src.pix.width,
		.height = ctx->src.pix.height,
		.min_qp = ctx->min_qp,
		.max_qp = ctx->max_qp,
	};
	int ret;

	ret = histb_venc_rc_init(&ctx->rc, &config);
	if (ret) {
		ctx->rc_initialized = false;
		return ret;
	}

	ctx->rc_initialized = true;
	ctx->rc_dirty = false;
	ctx->rc_job_pending = false;
	return 0;
}

static int histb_venc_prepare_rc(struct histb_venc_ctx *ctx)
{
	u64 target_bits;
	u32 qp;
	int ret;

	if (ctx->frame_rc) {
		if (!ctx->rc_initialized || ctx->rc_dirty) {
			ret = histb_venc_configure_rc(ctx);
			if (ret)
				return ret;
		}

		ret = histb_venc_rc_prepare(&ctx->rc, ctx->force_idr,
					    &ctx->rc_decision);
		if (ret)
			return ret;
		ctx->rc_job_pending = true;
	} else {
		target_bits = div_u64((u64)ctx->bitrate *
				      ctx->timeperframe.numerator,
				      ctx->timeperframe.denominator);
		if (!target_bits || target_bits > U32_MAX)
			return -ERANGE;
		ctx->rc_decision.intra = ctx->force_idr ||
			!((u32)ctx->src.sequence % ctx->gop_size);
		qp = ctx->rc_decision.intra ? ctx->i_qp : ctx->p_qp;
		ctx->rc_decision.target_bits = target_bits;
		ctx->rc_decision.start_qp = qp;
		ctx->rc_decision.min_qp = qp;
		ctx->rc_decision.max_qp = qp;
		ctx->rc_decision.token = 0;
		ctx->rc_job_pending = false;
	}

	ctx->curr_intra = ctx->rc_decision.intra;
	return 0;
}

static void histb_venc_cancel_rc(struct histb_venc_ctx *ctx, bool dropped)
{
	if (!ctx->rc_job_pending)
		return;

	if (dropped)
		histb_venc_rc_drop(&ctx->rc, ctx->rc_decision.token);
	else
		histb_venc_rc_abort(&ctx->rc, ctx->rc_decision.token);
	ctx->rc_job_pending = false;
}

static int histb_venc_collect_stream(struct histb_venc_ctx *ctx,
				     struct vb2_v4l2_buffer *dst)
{
	struct histb_venc_dev *venc = ctx->venc;
	struct histb_venc_stream_desc *desc;
	dma_addr_t dma = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	void *cpu = vb2_plane_vaddr(&dst->vb2_buf, 0);
	u8 sps[HISTB_VENC_HEADER_MAX], pps[HISTB_VENC_HEADER_MAX];
	unsigned int size = vb2_plane_size(&dst->vb2_buf, 0);
	unsigned int packet_len, invalid, payload, prefix, total;
	bool intra = ctx->curr_intra;
	int sps_len, pps_len;

	if (!cpu)
		return -EFAULT;

	dma_sync_single_for_cpu(venc->dev, dma, size, DMA_BIDIRECTIONAL);
	desc = cpu + HISTB_VENC_STREAM_META_SIZE;
	packet_len = le32_to_cpu(desc->packet_len);
	invalid = le32_to_cpu(desc->invalid_bytes);
	if (packet_len < HISTB_VENC_STREAM_DESC_SIZE ||
	    packet_len > size - HISTB_VENC_STREAM_META_SIZE ||
	    invalid > packet_len - HISTB_VENC_STREAM_DESC_SIZE ||
	    desc->type != (intra ? 5 : 1) || !desc->last_slice)
		return -EIO;

	payload = packet_len - HISTB_VENC_STREAM_DESC_SIZE - invalid;
	if (intra) {
		sps_len = histb_venc_build_sps(ctx, sps, sizeof(sps));
		pps_len = histb_venc_build_pps(pps, sizeof(pps));
		if (sps_len < 0 || pps_len < 0)
			return -EINVAL;
		prefix = sps_len + pps_len;
	} else {
		sps_len = 0;
		pps_len = 0;
		prefix = 0;
	}
	total = prefix + payload;
	if (total > size)
		return -ENOSPC;

	memmove(cpu + prefix,
		cpu + HISTB_VENC_STREAM_META_SIZE +
		HISTB_VENC_STREAM_DESC_SIZE, payload);
	if (intra) {
		memcpy(cpu, sps, sps_len);
		memcpy(cpu + sps_len, pps, pps_len);
	}
	vb2_set_plane_payload(&dst->vb2_buf, 0, total);

	return 0;
}

static void histb_venc_finish_job(struct histb_venc_ctx *ctx,
				  enum vb2_buffer_state state)
{
	struct histb_venc_dev *venc = ctx->venc;
	struct vb2_v4l2_buffer *src, *dst;
	bool last;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (WARN_ON(!src || !dst)) {
		v4l2_m2m_job_finish(venc->m2m_dev, ctx->fh.m2m_ctx);
		return;
	}

	if (state == VB2_BUF_STATE_DONE && histb_venc_collect_stream(ctx, dst))
		state = VB2_BUF_STATE_ERROR;
	if (state != VB2_BUF_STATE_DONE)
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);

	src->sequence = ctx->src.sequence++;
	dst->sequence = ctx->dst.sequence++;
	v4l2_m2m_buf_copy_metadata(src, dst, false);
	dst->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
			V4L2_BUF_FLAG_BFRAME);
	if (state == VB2_BUF_STATE_DONE) {
		if (ctx->curr_intra) {
			dst->flags |= V4L2_BUF_FLAG_KEYFRAME;
			ctx->frame_num = 1;
		} else {
			dst->flags |= V4L2_BUF_FLAG_PFRAME;
			ctx->frame_num = (ctx->frame_num + 1) & 0xf;
		}
		ctx->force_idr = false;
	} else {
		ctx->force_idr = true;
		ctx->frame_num = 0;
	}
	last = state == VB2_BUF_STATE_DONE &&
	       v4l2_m2m_is_last_draining_src_buf(ctx->fh.m2m_ctx, src);
	if (last) {
		dst->flags |= V4L2_BUF_FLAG_LAST;
		v4l2_m2m_mark_stopped(ctx->fh.m2m_ctx);
	}
	v4l2_m2m_buf_done_and_job_finish(venc->m2m_dev, ctx->fh.m2m_ctx,
					 state);
	if (last)
		v4l2_event_queue_fh(&ctx->fh, &histb_venc_eos_event);
}

static void histb_venc_watchdog(struct work_struct *work)
{
	struct histb_venc_dev *venc =
		container_of(to_delayed_work(work), struct histb_venc_dev,
			     watchdog_work);
	struct histb_venc_ctx *ctx;
	unsigned long flags;
	u64 deadline, now;
	unsigned long delay;

	mutex_lock(&venc->launch_lock);
	now = ktime_get_ns();
	spin_lock_irqsave(&venc->irqlock, flags);
	ctx = venc->curr_ctx;
	deadline = venc->watchdog_deadline_ns;
	if (ctx && deadline && now < deadline) {
		delay = max_t(unsigned long,
			      nsecs_to_jiffies(deadline - now), 1);
		spin_unlock_irqrestore(&venc->irqlock, flags);
		mod_delayed_work(system_wq, &venc->watchdog_work, delay);
		mutex_unlock(&venc->launch_lock);
		return;
	}
	if (ctx) {
		venc->curr_ctx = NULL;
		venc->watchdog_deadline_ns = 0;
	}
	spin_unlock_irqrestore(&venc->irqlock, flags);
	if (!ctx) {
		mutex_unlock(&venc->launch_lock);
		return;
	}

	dev_err(venc->dev, "H.264 encode timed out\n");
	ctx->job_aborting = false;
	histb_venc_cancel_rc(ctx, false);
	if (histb_venc_reset_engine(venc))
		dev_err(venc->dev, "failed to reset H.264 encoder\n");
	histb_venc_account_job(venc, true);
	histb_venc_finish_job(ctx, VB2_BUF_STATE_ERROR);
	mutex_unlock(&venc->launch_lock);
}

static irqreturn_t histb_venc_irq_thread(int irq, void *data)
{
	struct histb_venc_dev *venc = data;
	struct histb_venc_ctx *ctx;
	struct histb_venc_rc_feedback feedback;
	unsigned long flags;
	bool aborted, failed;
	u32 status;
	int ret;

	mutex_lock(&venc->launch_lock);
	status = readl(venc->regs + HISTB_VENC_INTSTAT) & HISTB_VENC_INT_ALL;
	if (!status) {
		mutex_unlock(&venc->launch_lock);
		return IRQ_NONE;
	}

	spin_lock_irqsave(&venc->irqlock, flags);
	ctx = venc->curr_ctx;
	if (ctx) {
		venc->curr_ctx = NULL;
		venc->watchdog_deadline_ns = 0;
	}
	spin_unlock_irqrestore(&venc->irqlock, flags);
	if (!ctx) {
		writel(status, venc->regs + HISTB_VENC_INTCLR);
		writel(0, venc->regs + HISTB_VENC_INTMASK);
		mutex_unlock(&venc->launch_lock);
		return IRQ_HANDLED;
	}

	cancel_delayed_work(&venc->watchdog_work);
	aborted = ctx->job_aborting;
	ctx->job_aborting = false;
	failed = (status & HISTB_VENC_INT_ERRORS) ||
		 !(status & HISTB_VENC_INT_DONE);
	if (!failed && !aborted) {
		histb_venc_rc_feedback_from_raw(&feedback,
						readl(venc->regs + HISTB_VENC_PICTURE_BITS),
			readl(venc->regs + HISTB_VENC_MEAN_QP),
			readl(venc->regs + HISTB_VENC_TIMER),
			readl(venc->regs + HISTB_VENC_IDLE_TIMER),
			readl(venc->regs + HISTB_VENC_PICINFO1),
			readl(venc->regs + HISTB_VENC_PICINFO6),
			readl(venc->regs + HISTB_VENC_PICINFO7),
			readl(venc->regs + HISTB_VENC_PICINFO8));
		if (ctx->frame_rc) {
			ret = histb_venc_rc_commit(&ctx->rc,
						   ctx->rc_decision.token,
						   &feedback);
			if (ret) {
				histb_venc_rc_abort(&ctx->rc,
						    ctx->rc_decision.token);
				dev_err(venc->dev,
					"invalid H.264 RC feedback: %d\n", ret);
				failed = true;
				ctx->rc_initialized = false;
			}
			ctx->rc_job_pending = false;
		}
	}

	writel(status, venc->regs + HISTB_VENC_INTCLR);
	writel(0, venc->regs + HISTB_VENC_INTMASK);

	if (failed) {
		dev_err(venc->dev, "H.264 encode failed, status %#x\n", status);
		histb_venc_cancel_rc(ctx,
				     status & (HISTB_VENC_INT_BUF_FULL |
				  HISTB_VENC_INT_BITS_OVERFLOW));
		ret = histb_venc_reset_engine(venc);
		if (ret)
			dev_err(venc->dev,
				"failed to reset H.264 encoder: %d\n", ret);
		histb_venc_account_job(venc, true);
		histb_venc_finish_job(ctx, VB2_BUF_STATE_ERROR);
	} else if (aborted) {
		histb_venc_cancel_rc(ctx, false);
		histb_venc_account_job(venc, true);
		histb_venc_finish_job(ctx, VB2_BUF_STATE_ERROR);
	} else {
		histb_venc_account_job(venc, false);
		histb_venc_finish_job(ctx, VB2_BUF_STATE_DONE);
	}
	mutex_unlock(&venc->launch_lock);

	return IRQ_HANDLED;
}

static void histb_venc_abort(struct histb_venc_ctx *ctx)
{
	struct histb_venc_dev *venc = ctx->venc;

	mutex_lock(&venc->launch_lock);
	/* Let an active frame drain; hardware errors and watchdog own reset. */
	ctx->job_aborting = true;
	mutex_unlock(&venc->launch_lock);
}

static void histb_venc_program_scaling(struct histb_venc_dev *venc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(histb_venc_scaling_list) / 4; i++) {
		u32 value = histb_venc_scaling_list[i * 4] |
			    histb_venc_scaling_list[i * 4 + 1] << 8 |
			    histb_venc_scaling_list[i * 4 + 2] << 16 |
			    histb_venc_scaling_list[i * 4 + 3] << 24;

		writel_relaxed(value, venc->regs + HISTB_VENC_SCALE(i));
	}
}

static void histb_venc_program_frame(struct histb_venc_ctx *ctx,
				     dma_addr_t src_dma, dma_addr_t dst_dma,
				     unsigned int dst_size,
				     u64 timestamp)
{
	struct histb_venc_dev *venc = ctx->venc;
	unsigned int width = ctx->src.pix.width;
	unsigned int height = ctx->src.pix.height;
	unsigned int stride = ctx->src.pix.bytesperline;
	unsigned int stream_len = (dst_size - HISTB_VENC_STREAM_META_SIZE) &
				  ~(HISTB_VENC_STREAM_ALIGN - 1);
	unsigned int recon_index = ctx->src.sequence & 1;
	dma_addr_t recon = ctx->recon_dma +
			   recon_index * ctx->recon_frame_size;
	dma_addr_t reference = ctx->recon_dma +
			       !recon_index * ctx->recon_frame_size;
	u32 slice_header[8], slice_param, me_h, me_v;
	u32 mode, qp, value;
	bool intra = ctx->curr_intra;
	unsigned int i;

	mode = HISTB_VENC_MODE_ENABLE |
		FIELD_PREP(HISTB_VENC_MODE_TIMEOUT, 3) |
		FIELD_PREP(HISTB_VENC_MODE_CLOCK_GATE, 2) |
		HISTB_VENC_MODE_MEMORY_GATE |
		FIELD_PREP(HISTB_VENC_MODE_PACKAGE_SEL,
			   HISTB_VENC_PACKAGE_U_V);
	writel_relaxed(mode, venc->regs + HISTB_VENC_MODE);
	writel_relaxed(HISTB_VENC_PICFG0_PTBITS_ENABLE |
			 (intra ? HISTB_VENC_PICFG0_I_PICTURE : 0) |
			 FIELD_PREP(HISTB_VENC_PICFG0_TRANSFORM, 1) |
			 FIELD_PREP(HISTB_VENC_PICFG0_NAL_REF_IDC, 3),
			 venc->regs + HISTB_VENC_PICFG0);
	writel_relaxed(0, venc->regs + HISTB_VENC_PICFG1);
	writel_relaxed(HISTB_VENC_PICFG2_IPCM |
			 HISTB_VENC_PICFG2_INTRA_4X4 |
			 HISTB_VENC_PICFG2_INTRA_16X16 |
			 HISTB_VENC_PICFG2_LOW_POWER,
			 venc->regs + HISTB_VENC_PICFG2);
	writel_relaxed(HISTB_VENC_PICFG3_INTER_MODES |
			 HISTB_VENC_PICFG3_FRAC_LOW_POWER |
			 HISTB_VENC_PICFG3_INTP_LOW_POWER |
			 HISTB_VENC_PICFG3_EXTENDED_EDGE,
			 venc->regs + HISTB_VENC_PICFG3);
	writel_relaxed(0, venc->regs + HISTB_VENC_PICFG4);
	writel_relaxed(16 | 235 << 8 | 16 << 16 | 240 << 24,
		       venc->regs + HISTB_VENC_PICFG5);
	writel_relaxed(0, venc->regs + HISTB_VENC_ENCUPDATE);
	writel_relaxed(FIELD_PREP(HISTB_VENC_IMAGE_WIDTH, width - 1) |
			 FIELD_PREP(HISTB_VENC_IMAGE_HEIGHT, height - 1),
			 venc->regs + HISTB_VENC_IMAGE_SIZE);
	writel_relaxed(4000000, venc->regs + HISTB_VENC_PTBITS);

	slice_param = histb_venc_build_slice_header(ctx, slice_header, intra);
	for (i = 0; i < ARRAY_SIZE(slice_header); i++)
		writel_relaxed(slice_header[i],
			       venc->regs + HISTB_VENC_SLICE_HEADER(i));
	writel_relaxed(slice_param, venc->regs + HISTB_VENC_SLICE_HEADER_PARAM);
	writel_relaxed(lower_32_bits(timestamp), venc->regs + HISTB_VENC_PTS0);
	writel_relaxed(upper_32_bits(timestamp), venc->regs + HISTB_VENC_PTS1);
	writel_relaxed(0, venc->regs + HISTB_VENC_PTS2);
	writel_relaxed(0, venc->regs + HISTB_VENC_PTS3);
	writel_relaxed(10000000, venc->regs + HISTB_VENC_TIMEOUT);
	writel_relaxed(FIELD_PREP(HISTB_VENC_OUTSTANDING_WRITE, 1) |
			 FIELD_PREP(HISTB_VENC_OUTSTANDING_READ, 1),
			 venc->regs + HISTB_VENC_OUTSTANDING);

	writel_relaxed(lower_32_bits(src_dma), venc->regs + HISTB_VENC_SRC_Y);
	writel_relaxed(lower_32_bits(src_dma + stride * height),
		       venc->regs + HISTB_VENC_SRC_C);
	writel_relaxed(0, venc->regs + HISTB_VENC_SRC_V);
	writel_relaxed(lower_32_bits(recon), venc->regs + HISTB_VENC_RECON_Y);
	writel_relaxed(lower_32_bits(recon + stride * height),
		       venc->regs + HISTB_VENC_RECON_C);
	writel_relaxed(lower_32_bits(reference), venc->regs + HISTB_VENC_REF_Y);
	writel_relaxed(lower_32_bits(reference + stride * height),
		       venc->regs + HISTB_VENC_REF_C);
	writel_relaxed(stride | stride << 16,
		       venc->regs + HISTB_VENC_SRC_STRIDE);
	writel_relaxed(stride | stride << 16,
		       venc->regs + HISTB_VENC_RECON_STRIDE);
	writel_relaxed(lower_32_bits(dst_dma + HISTB_VENC_STREAM_META_SIZE),
		       venc->regs + HISTB_VENC_STREAM_ADDR);
	writel_relaxed(lower_32_bits(dst_dma),
		       venc->regs + HISTB_VENC_STREAM_RPTR_ADDR);
	writel_relaxed(lower_32_bits(dst_dma + 16),
		       venc->regs + HISTB_VENC_STREAM_WPTR_ADDR);
	writel_relaxed(stream_len, venc->regs + HISTB_VENC_STREAM_LEN);

	if (width > 1280) {
		me_h = 5;
		me_v = 0;
	} else if (width > 720) {
		me_h = 5;
		me_v = 1;
	} else if (width > 352) {
		me_h = 5;
		me_v = 2;
	} else {
		me_h = 3;
		me_v = 2;
	}
	writel_relaxed(FIELD_PREP(HISTB_VENC_ME_H_SEARCH, me_h) |
			 FIELD_PREP(HISTB_VENC_ME_V_SEARCH, me_v) |
			 FIELD_PREP(HISTB_VENC_ME_FRAC_THRESHOLD, 15),
			 venc->regs + HISTB_VENC_ME_CFG0);
	writel_relaxed(GENMASK(3, 0), venc->regs + HISTB_VENC_ME_CFG1);
	writel_relaxed(2 | 2 << 8, venc->regs + HISTB_VENC_ME_RECT(0));
	writel_relaxed(8 | 8 << 8, venc->regs + HISTB_VENC_ME_RECT(1));
	writel_relaxed(13 | 13 << 8 | BIT(16) | BIT(24),
		       venc->regs + HISTB_VENC_ME_RECT(2));
	writel_relaxed(4 | 4 << 8, venc->regs + HISTB_VENC_ME_RECT(3));
	writel_relaxed(1500 << 16, venc->regs + HISTB_VENC_ME_THRESHOLD0);
	writel_relaxed(4096, venc->regs + HISTB_VENC_ME_THRESHOLD1);
	writel_relaxed(0, venc->regs + HISTB_VENC_ME_RDO);
	writel_relaxed(1, venc->regs + HISTB_VENC_CREF_MODE);
	writel_relaxed(BIT(16) | ((u32)-1024 & GENMASK(13, 0)),
		       venc->regs + HISTB_VENC_MD_CFG);
	writel_relaxed(4 << 8 | 4 << 12 | 2000 << 16,
		       venc->regs + HISTB_VENC_MCTF_CFG0);

	qp = FIELD_PREP(HISTB_VENC_QP_MIN, ctx->rc_decision.min_qp) |
	     FIELD_PREP(HISTB_VENC_QP_MAX, ctx->rc_decision.max_qp) |
	     FIELD_PREP(HISTB_VENC_QP_START, ctx->rc_decision.start_qp);
	writel_relaxed(qp, venc->regs + HISTB_VENC_QP_THRESHOLD);
	writel_relaxed(FIELD_PREP(HISTB_VENC_RC_QP_DELTA, 0) |
			 FIELD_PREP(HISTB_VENC_RC_MADP_DELTA, (u8)-8),
			 venc->regs + HISTB_VENC_RC_CFG);
	writel_relaxed(0x09070707, venc->regs + HISTB_VENC_QP_DELTA0);
	writel_relaxed(0x19120e0c, venc->regs + HISTB_VENC_QP_DELTA1);
	writel_relaxed(0xffffffff, venc->regs + HISTB_VENC_QP_DELTA2);
	writel_relaxed(ctx->rc_decision.target_bits,
		       venc->regs + HISTB_VENC_TARGET_BITS);
	for (i = 0; i < ARRAY_SIZE(histb_venc_mode_lambda) / 2; i++) {
		value = FIELD_PREP(HISTB_VENC_LAMBDA_LOW,
				   histb_venc_mode_lambda[i * 2]) |
			FIELD_PREP(HISTB_VENC_LAMBDA_HIGH,
				   histb_venc_mode_lambda[i * 2 + 1]);
		writel_relaxed(value,
			       venc->regs + HISTB_VENC_MODE_LAMBDA(i));
	}
	histb_venc_program_scaling(venc);
	writel_relaxed(0, venc->regs + HISTB_VENC_ROI_CFG);
	writel_relaxed(0, venc->regs + HISTB_VENC_OSD_CFG);
	writel_relaxed(0, venc->regs + HISTB_VENC_SECURE);
	writel_relaxed(0xff, venc->regs + HISTB_VENC_CHANNEL_BYPASS);
	writel_relaxed(HISTB_VENC_SMMU_BYPASS,
		       venc->regs + HISTB_VENC_SMMU_CTRL);
	writel_relaxed(0, venc->regs + HISTB_VENC_SMMU_INTMASK_NS);
}

static void histb_venc_job_abort(void *priv)
{
	histb_venc_abort(priv);
}

static void histb_venc_device_run(void *priv)
{
	struct histb_venc_ctx *ctx = priv;
	struct histb_venc_dev *venc = ctx->venc;
	struct vb2_v4l2_buffer *src, *dst;
	dma_addr_t src_dma, dst_dma;
	void *dst_cpu;
	unsigned long flags;
	unsigned int dst_size;

	mutex_lock(&venc->launch_lock);
	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (WARN_ON(!src || !dst)) {
		ctx->job_aborting = false;
		v4l2_m2m_job_finish(venc->m2m_dev, ctx->fh.m2m_ctx);
		mutex_unlock(&venc->launch_lock);
		return;
	}
	if (ctx->job_aborting) {
		ctx->job_aborting = false;
		histb_venc_finish_job(ctx, VB2_BUF_STATE_ERROR);
		mutex_unlock(&venc->launch_lock);
		return;
	}
	if (READ_ONCE(venc->engine_faulted)) {
		histb_venc_finish_job(ctx, VB2_BUF_STATE_ERROR);
		mutex_unlock(&venc->launch_lock);
		return;
	}

	src_dma = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	dst_dma = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	dst_cpu = vb2_plane_vaddr(&dst->vb2_buf, 0);
	dst_size = vb2_plane_size(&dst->vb2_buf, 0);
	if (!dst_cpu || dst_size <= HISTB_VENC_STREAM_META_SIZE ||
	    upper_32_bits(src_dma) || upper_32_bits(dst_dma) ||
	    upper_32_bits(ctx->recon_dma) ||
	    !IS_ALIGNED(src_dma, 16) ||
	    !IS_ALIGNED(dst_dma, HISTB_VENC_STREAM_ALIGN)) {
		histb_venc_finish_job(ctx, VB2_BUF_STATE_ERROR);
		mutex_unlock(&venc->launch_lock);
		return;
	}
	if (histb_venc_prepare_rc(ctx)) {
		histb_venc_finish_job(ctx, VB2_BUF_STATE_ERROR);
		mutex_unlock(&venc->launch_lock);
		return;
	}

	memset(dst_cpu, 0, HISTB_VENC_STREAM_META_SIZE);
	dma_sync_single_for_device(venc->dev, dst_dma, dst_size,
				   DMA_BIDIRECTIONAL);

	spin_lock_irqsave(&venc->irqlock, flags);
	if (WARN_ON(venc->curr_ctx)) {
		spin_unlock_irqrestore(&venc->irqlock, flags);
		histb_venc_cancel_rc(ctx, false);
		histb_venc_finish_job(ctx, VB2_BUF_STATE_ERROR);
		mutex_unlock(&venc->launch_lock);
		return;
	}
	venc->curr_ctx = ctx;
	venc->job_count_pending = true;
	venc->watchdog_deadline_ns = ktime_get_ns() +
		(u64)HISTB_VENC_WATCHDOG_MS * NSEC_PER_MSEC;
	spin_unlock_irqrestore(&venc->irqlock, flags);

	writel(0, venc->regs + HISTB_VENC_INTMASK);
	writel(HISTB_VENC_INT_ALL, venc->regs + HISTB_VENC_INTCLR);
	histb_venc_program_frame(ctx, src_dma, dst_dma, dst_size,
				 src->vb2_buf.timestamp);
	writel(HISTB_VENC_INT_ALL, venc->regs + HISTB_VENC_INTMASK);
	mod_delayed_work(system_wq, &venc->watchdog_work,
			 msecs_to_jiffies(HISTB_VENC_WATCHDOG_MS));
	/* Publish DMA metadata and register programming before starting VEDU. */
	wmb();
	writel(0, venc->regs + HISTB_VENC_START);
	writel(1, venc->regs + HISTB_VENC_START);
	mutex_unlock(&venc->launch_lock);
}

static const struct v4l2_m2m_ops histb_venc_m2m_ops = {
	.device_run = histb_venc_device_run,
	.job_abort = histb_venc_job_abort,
};

static void histb_venc_return_buffers(struct histb_venc_ctx *ctx,
				      enum v4l2_buf_type type)
{
	struct vb2_v4l2_buffer *buf;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(type))
			buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!buf)
			break;
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	}
}

static int histb_venc_alloc_recon(struct histb_venc_ctx *ctx)
{
	struct histb_venc_dev *venc = ctx->venc;
	size_t frame_size = ctx->src.pix.bytesperline *
			    ctx->src.pix.height * 3 / 2;
	size_t size = ALIGN(frame_size, HISTB_VENC_STREAM_ALIGN) * 2;

	frame_size = ALIGN(frame_size, HISTB_VENC_STREAM_ALIGN);
	if (ctx->recon_cpu && ctx->recon_size == size) {
		ctx->recon_frame_size = frame_size;
		return 0;
	}
	if (ctx->recon_cpu)
		dma_free_coherent(venc->dev, ctx->recon_size, ctx->recon_cpu,
				  ctx->recon_dma);

	ctx->recon_cpu = dma_alloc_coherent(venc->dev, size, &ctx->recon_dma,
					    GFP_KERNEL);
	if (!ctx->recon_cpu) {
		ctx->recon_size = 0;
		return -ENOMEM;
	}
	ctx->recon_size = size;
	ctx->recon_frame_size = frame_size;

	return 0;
}

static int histb_venc_queue_setup(struct vb2_queue *vq,
				  unsigned int *nbuffers,
				  unsigned int *nplanes,
				  unsigned int sizes[],
				  struct device *alloc_devs[])
{
	struct histb_venc_ctx *ctx = vb2_get_drv_priv(vq);
	struct histb_venc_q_data *q_data =
		histb_venc_get_q_data(ctx, vq->type);

	if (*nplanes)
		return sizes[0] < q_data->pix.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = q_data->pix.sizeimage;

	return 0;
}

static int histb_venc_buf_prepare(struct vb2_buffer *vb)
{
	struct histb_venc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct histb_venc_q_data *q_data =
		histb_venc_get_q_data(ctx, vb->vb2_queue->type);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (vb2_plane_size(vb, 0) < q_data->pix.sizeimage)
		return -EINVAL;
	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type) &&
	    !vb2_plane_vaddr(vb, 0))
		return -EFAULT;

	vbuf->field = V4L2_FIELD_NONE;
	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		vb2_set_plane_payload(vb, 0, q_data->pix.sizeimage);
	else
		vb2_set_plane_payload(vb, 0, 0);

	return 0;
}

static void histb_venc_buf_queue(struct vb2_buffer *vb)
{
	struct histb_venc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (!V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) &&
	    vb2_is_streaming(vb->vb2_queue) &&
	    v4l2_m2m_dst_buf_is_last(ctx->fh.m2m_ctx)) {
		vbuf->sequence = ctx->dst.sequence++;
		v4l2_m2m_last_buffer_done(ctx->fh.m2m_ctx, vbuf);
		v4l2_event_queue_fh(&ctx->fh, &histb_venc_eos_event);
		return;
	}

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int histb_venc_start_streaming(struct vb2_queue *vq,
				      unsigned int count)
{
	struct histb_venc_ctx *ctx = vb2_get_drv_priv(vq);
	struct histb_venc_q_data *q_data =
		histb_venc_get_q_data(ctx, vq->type);
	int ret;

	q_data->sequence = 0;
	mutex_lock(&ctx->venc->launch_lock);
	ctx->job_aborting = false;
	mutex_unlock(&ctx->venc->launch_lock);
	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		ctx->rc_initialized = false;
		ctx->rc_dirty = true;
		ctx->force_idr = true;
	}
	v4l2_m2m_update_start_streaming_state(ctx->fh.m2m_ctx, vq);
	ret = histb_venc_alloc_recon(ctx);
	if (ret)
		goto return_buffers;

	ret = pm_runtime_resume_and_get(ctx->venc->dev);
	if (ret < 0)
		goto return_buffers;

	return 0;

return_buffers:
	histb_venc_return_buffers(ctx, vq->type);
	return ret;
}

static void histb_venc_stop_streaming(struct vb2_queue *vq)
{
	struct histb_venc_ctx *ctx = vb2_get_drv_priv(vq);

	histb_venc_abort(ctx);
	v4l2_m2m_update_stop_streaming_state(ctx->fh.m2m_ctx, vq);
	histb_venc_return_buffers(ctx, vq->type);
	pm_runtime_mark_last_busy(ctx->venc->dev);
	pm_runtime_put_autosuspend(ctx->venc->dev);
}

static const struct vb2_ops histb_venc_qops = {
	.queue_setup = histb_venc_queue_setup,
	.buf_prepare = histb_venc_buf_prepare,
	.buf_queue = histb_venc_buf_queue,
	.start_streaming = histb_venc_start_streaming,
	.stop_streaming = histb_venc_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int histb_venc_queue_init(void *priv, struct vb2_queue *src_vq,
				 struct vb2_queue *dst_vq)
{
	struct histb_venc_ctx *ctx = priv;
	struct histb_venc_dev *venc = ctx->venc;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	src_vq->ops = &histb_venc_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &venc->lock;
	src_vq->dev = venc->dev;
	src_vq->dma_dir = DMA_TO_DEVICE;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	dst_vq->ops = &histb_venc_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &venc->lock;
	dst_vq->dev = venc->dev;
	dst_vq->dma_dir = DMA_BIDIRECTIONAL;

	return vb2_queue_init(dst_vq);
}

static int histb_venc_querycap(struct file *file, void *priv,
			       struct v4l2_capability *cap)
{
	strscpy(cap->driver, "histb-venc", sizeof(cap->driver));
	strscpy(cap->card, "HiSilicon Hi3798CV200 H.264 encoder",
		sizeof(cap->card));

	return 0;
}

static int histb_venc_encoder_cmd(struct file *file, void *priv,
				  struct v4l2_encoder_cmd *cmd)
{
	struct histb_venc_ctx *ctx = priv;
	int ret;

	ret = v4l2_m2m_ioctl_try_encoder_cmd(file, priv, cmd);
	if (ret)
		return ret;

	ret = v4l2_m2m_ioctl_encoder_cmd(file, priv, cmd);
	if (ret)
		return ret;

	if (cmd->cmd == V4L2_ENC_CMD_START) {
		vb2_clear_last_buffer_dequeued(
			v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
	} else if (v4l2_m2m_has_stopped(ctx->fh.m2m_ctx)) {
		v4l2_event_queue_fh(&ctx->fh, &histb_venc_eos_event);
	}

	return 0;
}

static int histb_venc_subscribe_event(struct v4l2_fh *fh,
				      const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

static int histb_venc_enum_fmt(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	if (V4L2_TYPE_IS_OUTPUT(f->type))
		f->pixelformat = V4L2_PIX_FMT_NV12;
	else
		f->pixelformat = V4L2_PIX_FMT_H264;

	return 0;
}

static int histb_venc_enum_framesizes(struct file *file, void *priv,
				      struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index ||
	    (fsize->pixel_format != V4L2_PIX_FMT_NV12 &&
	     fsize->pixel_format != V4L2_PIX_FMT_H264))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = HISTB_VENC_MIN_WIDTH;
	fsize->stepwise.max_width = HISTB_VENC_MAX_WIDTH;
	fsize->stepwise.step_width = 16;
	fsize->stepwise.min_height = HISTB_VENC_MIN_HEIGHT;
	fsize->stepwise.max_height = HISTB_VENC_MAX_HEIGHT;
	fsize->stepwise.step_height = 16;

	return 0;
}

static void histb_venc_try_raw_format(struct v4l2_pix_format *pix)
{
	u32 width = pix->width;
	u32 height = pix->height;
	u32 bytesperline;

	v4l_bound_align_image(&width, HISTB_VENC_MIN_WIDTH,
			      HISTB_VENC_MAX_WIDTH, 4,
			      &height, HISTB_VENC_MIN_HEIGHT,
			      HISTB_VENC_MAX_HEIGHT, 4, 0);
	bytesperline = clamp_t(u32, pix->bytesperline, width,
			       HISTB_VENC_MAX_STRIDE);
	bytesperline = ALIGN(bytesperline, 16);

	pix->width = width;
	pix->height = height;
	pix->pixelformat = V4L2_PIX_FMT_NV12;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = bytesperline;
	pix->sizeimage = bytesperline * height * 3 / 2;
	pix->colorspace = V4L2_COLORSPACE_REC709;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static void histb_venc_try_h264_format(struct histb_venc_ctx *ctx,
				       struct v4l2_pix_format *pix)
{
	u32 min_size = max_t(u32,
			     ALIGN(ctx->src.pix.width * ctx->src.pix.height +
				   HISTB_VENC_STREAM_META_SIZE,
				   HISTB_VENC_STREAM_ALIGN),
			     HISTB_VENC_STREAM_MIN_SIZE);

	pix->width = ctx->src.pix.width;
	pix->height = ctx->src.pix.height;
	pix->pixelformat = V4L2_PIX_FMT_H264;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = 0;
	pix->sizeimage = clamp(pix->sizeimage, min_size,
			       HISTB_VENC_STREAM_MAX_SIZE);
	pix->colorspace = ctx->src.pix.colorspace;
	pix->ycbcr_enc = ctx->src.pix.ycbcr_enc;
	pix->quantization = ctx->src.pix.quantization;
	pix->xfer_func = ctx->src.pix.xfer_func;
}

static int histb_venc_try_fmt(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct histb_venc_ctx *ctx = priv;

	if (V4L2_TYPE_IS_OUTPUT(f->type))
		histb_venc_try_raw_format(&f->fmt.pix);
	else
		histb_venc_try_h264_format(ctx, &f->fmt.pix);

	return 0;
}

static int histb_venc_g_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct histb_venc_ctx *ctx = priv;

	f->fmt.pix = histb_venc_get_q_data(ctx, f->type)->pix;

	return 0;
}

static int histb_venc_s_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct histb_venc_ctx *ctx = priv;
	struct vb2_queue *vq;
	u32 visible_width = f->fmt.pix.width;
	u32 visible_height = f->fmt.pix.height;
	int ret;

	ret = histb_venc_try_fmt(file, priv, f);
	if (ret)
		return ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq))
		return -EBUSY;

	histb_venc_get_q_data(ctx, f->type)->pix = f->fmt.pix;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		struct v4l2_pix_format dst = ctx->dst.pix;
		u32 width;
		u32 height;

		width = clamp_t(u32, ALIGN(visible_width, 2),
				HISTB_VENC_MIN_WIDTH, ctx->src.pix.width);
		height = clamp_t(u32, ALIGN(visible_height, 2),
				 HISTB_VENC_MIN_HEIGHT, ctx->src.pix.height);
		if (ALIGN(width, 16) != ctx->src.pix.width)
			width = ctx->src.pix.width;
		if (ALIGN(height, 16) != ctx->src.pix.height)
			height = ctx->src.pix.height;
		ctx->crop.left = 0;
		ctx->crop.top = 0;
		ctx->crop.width = width;
		ctx->crop.height = height;
		ctx->rc_initialized = false;
		ctx->rc_dirty = true;
		ctx->force_idr = true;

		histb_venc_try_h264_format(ctx, &dst);
		ctx->dst.pix = dst;
	}

	return 0;
}

static int histb_venc_g_selection(struct file *file, void *priv,
				  struct v4l2_selection *selection)
{
	struct histb_venc_ctx *ctx = priv;

	if (selection->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	selection->r.left = 0;
	selection->r.top = 0;
	switch (selection->target) {
	case V4L2_SEL_TGT_CROP:
		selection->r.width = ctx->crop.width;
		selection->r.height = ctx->crop.height;
		break;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		selection->r.width = ctx->src.pix.width;
		selection->r.height = ctx->src.pix.height;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int histb_venc_s_selection(struct file *file, void *priv,
				  struct v4l2_selection *selection)
{
	struct histb_venc_ctx *ctx = priv;
	struct vb2_queue *vq;
	u32 width;
	u32 height;

	if (selection->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ||
	    selection->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	vq = v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx);
	if (vb2_is_busy(vq))
		return -EBUSY;

	width = ALIGN(selection->r.width, 2);
	height = ALIGN(selection->r.height, 2);
	if (selection->r.left || selection->r.top ||
	    width < HISTB_VENC_MIN_WIDTH || height < HISTB_VENC_MIN_HEIGHT ||
	    width > ctx->src.pix.width || height > ctx->src.pix.height ||
	    ALIGN(width, 16) != ctx->src.pix.width ||
	    ALIGN(height, 16) != ctx->src.pix.height)
		return -EINVAL;

	ctx->crop.left = 0;
	ctx->crop.top = 0;
	ctx->crop.width = width;
	ctx->crop.height = height;
	selection->r = ctx->crop;

	return 0;
}

static int histb_venc_g_parm(struct file *file, void *priv,
			     struct v4l2_streamparm *parm)
{
	struct histb_venc_ctx *ctx = priv;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	memset(&parm->parm.output, 0, sizeof(parm->parm.output));
	parm->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	parm->parm.output.timeperframe = ctx->timeperframe;
	return 0;
}

static int histb_venc_s_parm(struct file *file, void *priv,
			     struct v4l2_streamparm *parm)
{
	struct histb_venc_ctx *ctx = priv;
	struct v4l2_fract requested;
	u32 fps;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	requested = parm->parm.output.timeperframe;
	if (!requested.numerator || !requested.denominator ||
	    requested.denominator % requested.numerator)
		return -EINVAL;
	fps = requested.denominator / requested.numerator;
	if (!fps || fps > 60)
		return -ERANGE;

	ctx->timeperframe.numerator = 1;
	ctx->timeperframe.denominator = fps;
	ctx->rc_dirty = true;
	return histb_venc_g_parm(file, priv, parm);
}

static const struct v4l2_ioctl_ops histb_venc_ioctl_ops = {
	.vidioc_querycap = histb_venc_querycap,
	.vidioc_enum_fmt_vid_cap = histb_venc_enum_fmt,
	.vidioc_g_fmt_vid_cap = histb_venc_g_fmt,
	.vidioc_try_fmt_vid_cap = histb_venc_try_fmt,
	.vidioc_s_fmt_vid_cap = histb_venc_s_fmt,
	.vidioc_enum_fmt_vid_out = histb_venc_enum_fmt,
	.vidioc_g_fmt_vid_out = histb_venc_g_fmt,
	.vidioc_try_fmt_vid_out = histb_venc_try_fmt,
	.vidioc_s_fmt_vid_out = histb_venc_s_fmt,
	.vidioc_enum_framesizes = histb_venc_enum_framesizes,
	.vidioc_g_parm = histb_venc_g_parm,
	.vidioc_s_parm = histb_venc_s_parm,
	.vidioc_g_selection = histb_venc_g_selection,
	.vidioc_s_selection = histb_venc_s_selection,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_encoder_cmd = histb_venc_encoder_cmd,
	.vidioc_try_encoder_cmd = v4l2_m2m_ioctl_try_encoder_cmd,
	.vidioc_subscribe_event = histb_venc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int histb_venc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct histb_venc_ctx *ctx =
		container_of(ctrl->handler, struct histb_venc_ctx, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		ctx->i_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
		ctx->p_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
		if (ctrl->val > ctx->max_qp)
			return -ERANGE;
		ctx->min_qp = ctrl->val;
		ctx->rc_dirty = true;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		if (ctrl->val < ctx->min_qp)
			return -ERANGE;
		ctx->max_qp = ctrl->val;
		ctx->rc_dirty = true;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		ctx->gop_size = ctrl->val;
		ctx->rc_dirty = true;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		ctx->bitrate = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		ctx->profile = ctrl->val;
		ctx->rc_dirty = true;
		break;
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		if (ctx->rc_job_pending)
			return -EBUSY;
		ctx->frame_rc = ctrl->val;
		ctx->rc_initialized = false;
		ctx->rc_dirty = true;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
		if (ctrl->val != V4L2_MPEG_VIDEO_BITRATE_MODE_CBR)
			return -EINVAL;
		break;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		ctx->force_idr = true;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops histb_venc_ctrl_ops = {
	.s_ctrl = histb_venc_s_ctrl,
};

static void histb_venc_set_default_formats(struct histb_venc_ctx *ctx)
{
	ctx->src.pix.width = HISTB_VENC_DEFAULT_WIDTH;
	ctx->src.pix.height = HISTB_VENC_DEFAULT_HEIGHT;
	ctx->src.pix.bytesperline = HISTB_VENC_DEFAULT_WIDTH;
	histb_venc_try_raw_format(&ctx->src.pix);
	ctx->dst.pix.width = HISTB_VENC_DEFAULT_WIDTH;
	ctx->dst.pix.height = HISTB_VENC_DEFAULT_HEIGHT;
	histb_venc_try_h264_format(ctx, &ctx->dst.pix);
	ctx->crop.left = 0;
	ctx->crop.top = 0;
	ctx->crop.width = ctx->src.pix.width;
	ctx->crop.height = ctx->src.pix.height;
}

static int histb_venc_open(struct file *file)
{
	struct histb_venc_dev *venc = video_drvdata(file);
	struct histb_venc_ctx *ctx;
	struct v4l2_m2m_ctx *m2m_ctx;
	struct v4l2_ctrl *ctrl;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->venc = venc;
	ctx->i_qp = HISTB_VENC_DEFAULT_QP;
	ctx->p_qp = HISTB_VENC_DEFAULT_QP;
	ctx->gop_size = 50;
	ctx->bitrate = HISTB_VENC_DEFAULT_BITRATE;
	ctx->profile = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH;
	ctx->min_qp = HISTB_VENC_DEFAULT_MIN_QP;
	ctx->max_qp = HISTB_VENC_DEFAULT_MAX_QP;
	ctx->timeperframe.numerator = 1;
	ctx->timeperframe.denominator = HISTB_VENC_DEFAULT_FPS;
	ctx->frame_rc = true;
	ctx->rc_dirty = true;
	ctx->force_idr = true;
	histb_venc_set_default_formats(ctx);
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 12);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &histb_venc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
			  0, 51, 1, HISTB_VENC_DEFAULT_QP);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &histb_venc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
			  0, 51, 1, HISTB_VENC_DEFAULT_QP);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &histb_venc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
			  0, 51, 1, HISTB_VENC_DEFAULT_MIN_QP);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &histb_venc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
			  0, 51, 1, HISTB_VENC_DEFAULT_MAX_QP);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &histb_venc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_GOP_SIZE, 1, 300, 1, 50);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &histb_venc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_BITRATE,
			  HISTB_VENC_MIN_BITRATE, HISTB_VENC_MAX_BITRATE, 1,
			  HISTB_VENC_DEFAULT_BITRATE);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &histb_venc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, 0, 1, 1, 1);
	v4l2_ctrl_new_std_menu(&ctx->ctrl_handler, &histb_venc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CBR,
			       ~BIT(V4L2_MPEG_VIDEO_BITRATE_MODE_CBR),
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &histb_venc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 0, 0, 0, 0);
	v4l2_ctrl_new_std_menu(&ctx->ctrl_handler, NULL,
			       V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			       V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
			       0, V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE);
	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler, NULL,
				 V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR,
				 0, 1, 1, 1);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (ctx->ctrl_handler.error) {
		ret = ctx->ctrl_handler.error;
		goto free_fh;
	}
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;

	m2m_ctx = v4l2_m2m_ctx_init(venc->m2m_dev, ctx, histb_venc_queue_init);
	ctx->fh.m2m_ctx = m2m_ctx;
	if (IS_ERR(m2m_ctx)) {
		ret = PTR_ERR(m2m_ctx);
		goto free_ctrls;
	}

	v4l2_fh_add(&ctx->fh);
	ret = v4l2_ctrl_handler_setup(&ctx->ctrl_handler);
	if (ret)
		goto del_fh;

	return 0;

del_fh:
	v4l2_fh_del(&ctx->fh);
	v4l2_m2m_ctx_release(m2m_ctx);
free_ctrls:
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
free_fh:
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int histb_venc_release(struct file *file)
{
	struct histb_venc_ctx *ctx =
		container_of(file->private_data, struct histb_venc_ctx, fh);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	if (ctx->recon_cpu)
		dma_free_coherent(ctx->venc->dev, ctx->recon_size,
				  ctx->recon_cpu, ctx->recon_dma);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations histb_venc_fops = {
	.owner = THIS_MODULE,
	.open = histb_venc_open,
	.release = histb_venc_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static int histb_venc_runtime_resume(struct device *dev)
{
	struct histb_venc_dev *venc = dev_get_drvdata(dev);
	int ret;

	WRITE_ONCE(venc->engine_faulted, true);
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(venc->clocks), venc->clocks);
	if (ret)
		return ret;

	ret = reset_control_deassert(venc->reset);
	if (ret) {
		reset_control_assert(venc->reset);
		clk_bulk_disable_unprepare(ARRAY_SIZE(venc->clocks), venc->clocks);
		return ret;
	}
	udelay(10);

	writel(0, venc->regs + HISTB_VENC_INTMASK);
	writel(HISTB_VENC_INT_ALL, venc->regs + HISTB_VENC_INTCLR);
	enable_irq(venc->irq);
	WRITE_ONCE(venc->engine_faulted, false);

	return 0;
}

static int histb_venc_runtime_suspend(struct device *dev)
{
	struct histb_venc_dev *venc = dev_get_drvdata(dev);
	bool faulted = READ_ONCE(venc->engine_faulted);

	WRITE_ONCE(venc->engine_faulted, true);
	disable_irq(venc->irq);
	if (!faulted) {
		writel(0, venc->regs + HISTB_VENC_INTMASK);
		writel(HISTB_VENC_INT_ALL, venc->regs + HISTB_VENC_INTCLR);
	}
	reset_control_assert(venc->reset);
	clk_bulk_disable_unprepare(ARRAY_SIZE(venc->clocks), venc->clocks);

	return 0;
}

static int __maybe_unused histb_venc_suspend(struct device *dev)
{
	struct histb_venc_dev *venc = dev_get_drvdata(dev);
	int ret;

	v4l2_m2m_suspend(venc->m2m_dev);
	ret = pm_runtime_force_suspend(dev);
	if (ret)
		v4l2_m2m_resume(venc->m2m_dev);

	return ret;
}

static int __maybe_unused histb_venc_resume(struct device *dev)
{
	struct histb_venc_dev *venc = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (!ret)
		v4l2_m2m_resume(venc->m2m_dev);

	return ret;
}

static const struct dev_pm_ops histb_venc_pm_ops = {
	SET_RUNTIME_PM_OPS(histb_venc_runtime_suspend,
			   histb_venc_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(histb_venc_suspend, histb_venc_resume)
};

static int histb_venc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_venc_dev *venc;
	int ret;

	venc = devm_kzalloc(dev, sizeof(*venc), GFP_KERNEL);
	if (!venc)
		return -ENOMEM;

	venc->dev = dev;
	venc->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(venc->regs))
		return PTR_ERR(venc->regs);

	venc->clocks[0].id = "core";
	venc->clocks[1].id = "axi";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(venc->clocks), venc->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");
	ret = clk_set_rate(venc->clocks[0].clk, HISTB_VENC_CORE_RATE);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to select original core clock\n");
	if (clk_get_rate(venc->clocks[0].clk) != HISTB_VENC_CORE_RATE)
		return dev_err_probe(dev, -EIO,
				     "core clock did not reach %lu Hz\n",
				     HISTB_VENC_CORE_RATE);

	venc->reset = devm_reset_control_get_exclusive(dev, "venc");
	if (IS_ERR(venc->reset))
		return dev_err_probe(dev, PTR_ERR(venc->reset),
				     "failed to get reset\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	venc->irq = platform_get_irq(pdev, 0);
	if (venc->irq < 0)
		return venc->irq;

	mutex_init(&venc->lock);
	mutex_init(&venc->launch_lock);
	spin_lock_init(&venc->irqlock);
	INIT_DELAYED_WORK(&venc->watchdog_work, histb_venc_watchdog);
	platform_set_drvdata(pdev, venc);

	ret = devm_request_threaded_irq(dev, venc->irq, NULL,
					histb_venc_irq_thread,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					dev_name(dev), venc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	ret = v4l2_device_register(dev, &venc->v4l2_dev);
	if (ret)
		return ret;

	venc->m2m_dev = v4l2_m2m_init(&histb_venc_m2m_ops);
	if (IS_ERR(venc->m2m_dev)) {
		ret = PTR_ERR(venc->m2m_dev);
		goto unregister_v4l2;
	}

	strscpy(venc->vfd.name, "histb-venc", sizeof(venc->vfd.name));
	venc->vfd.fops = &histb_venc_fops;
	venc->vfd.ioctl_ops = &histb_venc_ioctl_ops;
	venc->vfd.v4l2_dev = &venc->v4l2_dev;
	venc->vfd.lock = &venc->lock;
	venc->vfd.release = video_device_release_empty;
	venc->vfd.vfl_dir = VFL_DIR_M2M;
	venc->vfd.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	venc->vfd.entity.function = MEDIA_ENT_F_PROC_VIDEO_ENCODER;
	video_set_drvdata(&venc->vfd, venc);

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
	ret = devm_device_add_group(dev, &histb_venc_attr_group);
	if (ret)
		goto disable_runtime_pm;

	ret = video_register_device(&venc->vfd, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto disable_runtime_pm;

	dev_info(dev, "registered as /dev/%s\n",
		 video_device_node_name(&venc->vfd));
	return 0;

disable_runtime_pm:
	pm_runtime_disable(dev);
	v4l2_m2m_release(venc->m2m_dev);
unregister_v4l2:
	v4l2_device_unregister(&venc->v4l2_dev);
	return ret;
}

static void histb_venc_remove(struct platform_device *pdev)
{
	struct histb_venc_dev *venc = platform_get_drvdata(pdev);

	v4l2_m2m_suspend(venc->m2m_dev);
	synchronize_irq(venc->irq);
	cancel_delayed_work_sync(&venc->watchdog_work);
	video_unregister_device(&venc->vfd);
	pm_runtime_disable(venc->dev);
	if (!pm_runtime_status_suspended(venc->dev))
		histb_venc_runtime_suspend(venc->dev);
	v4l2_m2m_release(venc->m2m_dev);
	v4l2_device_unregister(&venc->v4l2_dev);
}

static const struct of_device_id histb_venc_of_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-venc" },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_venc_of_match);

static struct platform_driver histb_venc_driver = {
	.probe = histb_venc_probe,
	.remove_new = histb_venc_remove,
	.driver = {
		.name = "histb-venc",
		.of_match_table = histb_venc_of_match,
		.pm = &histb_venc_pm_ops,
	},
};
module_platform_driver(histb_venc_driver);

MODULE_AUTHOR("HiSilicon Technologies Co., Ltd.");
MODULE_DESCRIPTION("HiSilicon Hi3798CV200 H.264 encoder");
MODULE_LICENSE("GPL");
