// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon HiSTB stand-alone JPEG encoder
 *
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/workqueue.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-jpeg.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#define HISTB_JPGE_INTSTAT		0x0000
#define HISTB_JPGE_INTMASK		0x0004
#define HISTB_JPGE_INTCLR		0x000c
#define HISTB_JPGE_PICFG		0x0010
#define HISTB_JPGE_ECSCFG		0x0014
#define HISTB_JPGE_START		0x001c
#define HISTB_JPGE_IMAGE_SIZE		0x0020
#define HISTB_JPGE_SRC_Y		0x0030
#define HISTB_JPGE_SRC_C		0x0034
#define HISTB_JPGE_SRC_V		0x0038
#define HISTB_JPGE_SRC_STRIDE		0x003c
#define HISTB_JPGE_STREAM_ADDR		0x0040
#define HISTB_JPGE_STREAM_RPTR		0x0044
#define HISTB_JPGE_STREAM_WPTR		0x0048
#define HISTB_JPGE_STREAM_LEN		0x004c
#define HISTB_JPGE_PTS0			0x0070
#define HISTB_JPGE_OUTSTD		0x00a4
#define HISTB_JPGE_QTABLE(n)		(0x1000 + (n) * 4)

#define HISTB_JPGE_INT_DONE		BIT(0)
#define HISTB_JPGE_INT_BUF_FULL		BIT(1)
#define HISTB_JPGE_INT_TIMEOUT		BIT(2)
#define HISTB_JPGE_INT_CFG_ERR		BIT(3)
#define HISTB_JPGE_INT_ALL		GENMASK(3, 0)
#define HISTB_JPGE_INT_ERRORS		(HISTB_JPGE_INT_BUF_FULL | \
					 HISTB_JPGE_INT_TIMEOUT | \
					 HISTB_JPGE_INT_CFG_ERR)

#define HISTB_JPGE_PICFG_STORE		GENMASK(1, 0)
#define HISTB_JPGE_PICFG_SAMPLE		GENMASK(3, 2)
#define HISTB_JPGE_PICFG_CLK_GATE	GENMASK(15, 14)
#define HISTB_JPGE_PICFG_MEM_CLK_GATE	BIT(16)
#define HISTB_JPGE_PICFG_PACKAGE	GENMASK(27, 20)

#define HISTB_JPGE_IMAGE_WIDTH		GENMASK(12, 0)
#define HISTB_JPGE_IMAGE_HEIGHT		GENMASK(28, 16)

#define HISTB_JPGE_MIN_DIMENSION	16
#define HISTB_JPGE_MAX_DIMENSION	4096
#define HISTB_JPGE_DEFAULT_WIDTH	1280
#define HISTB_JPGE_DEFAULT_HEIGHT	720
#define HISTB_JPGE_DEFAULT_QUALITY	75

/*
 * The hardware writes a 64-byte stream descriptor at STREAM_ADDR and starts
 * the entropy-coded scan immediately afterwards. A 704-byte JPEG header keeps
 * STREAM_ADDR at a 64-byte aligned offset while letting the scan begin directly
 * after the header.
 */
#define HISTB_JPGE_HEADER_SIZE		704
#define HISTB_JPGE_STREAM_META_OFFSET	(HISTB_JPGE_HEADER_SIZE - 64)
#define HISTB_JPGE_STREAM_DATA_OFFSET	HISTB_JPGE_HEADER_SIZE
#define HISTB_JPGE_STREAM_ADDR_ALIGN	64
#define HISTB_JPGE_STREAM_SIZE_ALIGN	256
#define HISTB_JPGE_WATCHDOG_MS		5000

struct histb_jpge_stream_header {
	__le32 packet_len;
	__le32 invalid_bytes;
	u8 reserved[56];
};

struct histb_jpge_q_data {
	struct v4l2_pix_format pix;
	u32 sequence;
};

struct histb_jpge_dev;

struct histb_jpge_ctx {
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler ctrl_handler;
	struct histb_jpge_dev *jpge;
	struct histb_jpge_q_data src;
	struct histb_jpge_q_data dst;
	u8 luma_qtable[V4L2_JPEG_PIXELS_IN_BLOCK];
	u8 chroma_qtable[V4L2_JPEG_PIXELS_IN_BLOCK];
	u32 quality;
	u32 active_stream_len;
	bool job_aborting;
	/*
	 * V4L2_ENC_CMD_STOP was received.  The next capture buffer completed
	 * carries V4L2_BUF_FLAG_LAST so the client's drain terminates.  The
	 * driver used to implement no encoder_cmd at all, so VIDIOC_ENCODER_CMD
	 * returned ENOTTY, FFmpeg fell back to streaming off the output queue
	 * only, and the capture buffers left in the driver were never
	 * completed - the client then blocked in poll forever after the last
	 * frame was encoded correctly.
	 */
	bool draining;
};

struct histb_jpge_dev {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	struct reset_control *reset;
	int irq;

	struct v4l2_device v4l2_dev;
	struct video_device vfd;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex lock; /* Serializes V4L2 ioctls and vb2 queues. */

	spinlock_t irqlock; /* Protects curr_ctx against IRQ and timeout races. */
	struct histb_jpge_ctx *curr_ctx;
	/* Serializes setup and launch with completion and teardown. */
	struct mutex launch_lock;
	struct delayed_work watchdog_work;
	u64 watchdog_deadline_ns;
	bool quiescing;
	bool engine_faulted;
};

struct histb_jpge_header_writer {
	u8 *buf;
	size_t pos;
};

static struct histb_jpge_q_data *
histb_jpge_get_q_data(struct histb_jpge_ctx *ctx, enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->src;

	return &ctx->dst;
}

static void histb_jpge_put_u8(struct histb_jpge_header_writer *writer, u8 value)
{
	writer->buf[writer->pos++] = value;
}

static void histb_jpge_put_be16(struct histb_jpge_header_writer *writer,
				u16 value)
{
	histb_jpge_put_u8(writer, value >> 8);
	histb_jpge_put_u8(writer, value);
}

static void histb_jpge_put_marker(struct histb_jpge_header_writer *writer,
				  u8 marker)
{
	histb_jpge_put_u8(writer, 0xff);
	histb_jpge_put_u8(writer, marker);
}

static void histb_jpge_put_data(struct histb_jpge_header_writer *writer,
				const u8 *data, size_t len)
{
	memcpy(writer->buf + writer->pos, data, len);
	writer->pos += len;
}

static u8 histb_jpge_scale_quantizer(u8 quantizer, unsigned int scale)
{
	unsigned int value;

	value = DIV_ROUND_CLOSEST((unsigned int)quantizer * scale, 100);
	return clamp_t(unsigned int, value, 1, 255);
}

static void histb_jpge_prepare_qtables(struct histb_jpge_ctx *ctx)
{
	unsigned int scale;
	unsigned int i;

	if (ctx->quality < 50)
		scale = 5000 / ctx->quality;
	else
		scale = 200 - 2 * ctx->quality;

	for (i = 0; i < V4L2_JPEG_PIXELS_IN_BLOCK; i++) {
		u8 luma = v4l2_jpeg_ref_table_luma_qt[i];
		u8 chroma = v4l2_jpeg_ref_table_chroma_qt[i];

		ctx->luma_qtable[i] = histb_jpge_scale_quantizer(luma, scale);
		ctx->chroma_qtable[i] = histb_jpge_scale_quantizer(chroma, scale);
	}
}

static void histb_jpge_write_dqt(struct histb_jpge_header_writer *writer,
				 const u8 *table, u8 id)
{
	unsigned int i;

	histb_jpge_put_marker(writer, 0xdb);
	histb_jpge_put_be16(writer, 67);
	histb_jpge_put_u8(writer, id);
	for (i = 0; i < V4L2_JPEG_PIXELS_IN_BLOCK; i++) {
		unsigned int index = v4l2_jpeg_zigzag_scan_index[i];

		histb_jpge_put_u8(writer, table[index]);
	}
}

static void histb_jpge_write_dht(struct histb_jpge_header_writer *writer,
				 const u8 *table, size_t len, u8 id)
{
	histb_jpge_put_marker(writer, 0xc4);
	histb_jpge_put_be16(writer, len + 3);
	histb_jpge_put_u8(writer, id);
	histb_jpge_put_data(writer, table, len);
}

static int histb_jpge_build_header(struct histb_jpge_ctx *ctx, void *buffer)
{
	struct histb_jpge_header_writer writer = { .buf = buffer };
	const u8 *chroma_ac = v4l2_jpeg_ref_table_chroma_ac_ht;
	const u8 *chroma_dc = v4l2_jpeg_ref_table_chroma_dc_ht;
	const u8 *luma_ac = v4l2_jpeg_ref_table_luma_ac_ht;
	const u8 *luma_dc = v4l2_jpeg_ref_table_luma_dc_ht;
	unsigned int i;

	/* SOI and JFIF APP0. */
	histb_jpge_put_marker(&writer, 0xd8);
	histb_jpge_put_marker(&writer, 0xe0);
	histb_jpge_put_be16(&writer, 16);
	histb_jpge_put_data(&writer, (const u8 *)"JFIF\0", 5);
	histb_jpge_put_u8(&writer, 1);
	histb_jpge_put_u8(&writer, 1);
	histb_jpge_put_u8(&writer, 0);
	histb_jpge_put_be16(&writer, 1);
	histb_jpge_put_be16(&writer, 1);
	histb_jpge_put_u8(&writer, 0);
	histb_jpge_put_u8(&writer, 0);

	/* The block has separate luma, Cb and Cr quantization memories. */
	histb_jpge_write_dqt(&writer, ctx->luma_qtable, 0);
	histb_jpge_write_dqt(&writer, ctx->chroma_qtable, 1);
	histb_jpge_write_dqt(&writer, ctx->chroma_qtable, 2);

	/* Baseline SOF0, 8-bit YCbCr 4:2:0. */
	histb_jpge_put_marker(&writer, 0xc0);
	histb_jpge_put_be16(&writer, 17);
	histb_jpge_put_u8(&writer, 8);
	histb_jpge_put_be16(&writer, ctx->src.pix.height);
	histb_jpge_put_be16(&writer, ctx->src.pix.width);
	histb_jpge_put_u8(&writer, 3);
	histb_jpge_put_u8(&writer, 1);
	histb_jpge_put_u8(&writer, 0x22);
	histb_jpge_put_u8(&writer, 0);
	histb_jpge_put_u8(&writer, 2);
	histb_jpge_put_u8(&writer, 0x11);
	histb_jpge_put_u8(&writer, 1);
	histb_jpge_put_u8(&writer, 3);
	histb_jpge_put_u8(&writer, 0x11);
	histb_jpge_put_u8(&writer, 2);

	histb_jpge_write_dht(&writer, luma_dc, V4L2_JPEG_REF_HT_DC_LEN, 0x00);
	histb_jpge_write_dht(&writer, luma_ac, V4L2_JPEG_REF_HT_AC_LEN, 0x10);
	histb_jpge_write_dht(&writer, chroma_dc, V4L2_JPEG_REF_HT_DC_LEN, 0x01);
	histb_jpge_write_dht(&writer, chroma_ac, V4L2_JPEG_REF_HT_AC_LEN, 0x11);

	/* Pad the header so the hardware descriptor starts on a cache line. */
	histb_jpge_put_marker(&writer, 0xfe);
	histb_jpge_put_be16(&writer, 10);
	for (i = 0; i < 8; i++)
		histb_jpge_put_u8(&writer, 0);

	/* SOS; the entropy-coded scan is written immediately after this. */
	histb_jpge_put_marker(&writer, 0xda);
	histb_jpge_put_be16(&writer, 12);
	histb_jpge_put_u8(&writer, 3);
	histb_jpge_put_u8(&writer, 1);
	histb_jpge_put_u8(&writer, 0x00);
	/*
	 * The engine emits Cr before Cb into the scan, so the scan order is
	 * 1, 3, 2 - component 3 (Cr) first.  Measured with a source whose Cb
	 * is a constant 0x40 and Cr a constant 0xc0: the decode came back
	 * with U = 191, V = 65, i.e. swapped and clean (zero variance in
	 * both), which is exactly this ordering.  The SOF component list
	 * itself stays 1, 2, 3 with Cb = 2 and Cr = 3.
	 */
	histb_jpge_put_u8(&writer, 3);
	histb_jpge_put_u8(&writer, 0x11);
	histb_jpge_put_u8(&writer, 2);
	histb_jpge_put_u8(&writer, 0x11);
	histb_jpge_put_u8(&writer, 0);
	histb_jpge_put_u8(&writer, 63);
	histb_jpge_put_u8(&writer, 0);

	return writer.pos == HISTB_JPGE_HEADER_SIZE ? 0 : -EINVAL;
}

static void histb_jpge_program_qtables(struct histb_jpge_ctx *ctx)
{
	struct histb_jpge_dev *jpge = ctx->jpge;
	unsigned int i;

	for (i = 0; i < 16; i++) {
		unsigned int index = (i & 1) * 32 + (i >> 1);
		u32 luma, chroma;

		luma = ctx->luma_qtable[index] |
		       ctx->luma_qtable[index + 8] << 8 |
		       ctx->luma_qtable[index + 16] << 16 |
		       ctx->luma_qtable[index + 24] << 24;
		chroma = ctx->chroma_qtable[index] |
			 ctx->chroma_qtable[index + 8] << 8 |
			 ctx->chroma_qtable[index + 16] << 16 |
			 ctx->chroma_qtable[index + 24] << 24;

		writel_relaxed(luma, jpge->regs + HISTB_JPGE_QTABLE(i));
		writel_relaxed(chroma, jpge->regs + HISTB_JPGE_QTABLE(i + 16));
		writel_relaxed(chroma, jpge->regs + HISTB_JPGE_QTABLE(i + 32));
	}
}

static bool histb_jpge_take_job(struct histb_jpge_dev *jpge,
				struct histb_jpge_ctx *ctx)
{
	unsigned long flags;
	bool taken = false;

	spin_lock_irqsave(&jpge->irqlock, flags);
	if (jpge->curr_ctx == ctx) {
		jpge->curr_ctx = NULL;
		jpge->watchdog_deadline_ns = 0;
		taken = true;
	}
	spin_unlock_irqrestore(&jpge->irqlock, flags);

	return taken;
}

static int histb_jpge_reset_engine(struct histb_jpge_dev *jpge)
{
	int ret;

	WRITE_ONCE(jpge->engine_faulted, true);
	writel(0, jpge->regs + HISTB_JPGE_INTMASK);
	writel(HISTB_JPGE_INT_ALL, jpge->regs + HISTB_JPGE_INTCLR);
	ret = reset_control_reset(jpge->reset);
	if (!ret)
		WRITE_ONCE(jpge->engine_faulted, false);

	return ret;
}

static int histb_jpge_capture_begin_cpu(struct histb_jpge_ctx *ctx,
					struct vb2_buffer *vb)
{
	struct dma_buf *dbuf = vb->planes[0].dbuf;

	if (dbuf)
		return dma_buf_begin_cpu_access(dbuf, DMA_BIDIRECTIONAL);

	dma_sync_single_for_cpu(ctx->jpge->dev,
				vb2_dma_contig_plane_dma_addr(vb, 0),
				vb2_plane_size(vb, 0), DMA_BIDIRECTIONAL);
	return 0;
}

static int histb_jpge_capture_end_cpu(struct vb2_buffer *vb)
{
	struct dma_buf *dbuf = vb->planes[0].dbuf;

	if (dbuf)
		return dma_buf_end_cpu_access(dbuf, DMA_BIDIRECTIONAL);

	return 0;
}

static void histb_jpge_finish_job(struct histb_jpge_ctx *ctx,
				  enum vb2_buffer_state state)
{
	struct histb_jpge_dev *jpge = ctx->jpge;
	struct vb2_v4l2_buffer *src, *dst;
	struct histb_jpge_stream_header *stream_header;
	void *dst_cpu;
	u32 packet_len, invalid_bytes, payload, scan_len;
	int end_ret, ret;
	bool cpu_access = false;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (WARN_ON(!src || !dst)) {
		v4l2_m2m_job_finish(jpge->m2m_dev, ctx->fh.m2m_ctx);
		return;
	}

	if (state == VB2_BUF_STATE_DONE) {
		dst_cpu = vb2_plane_vaddr(&dst->vb2_buf, 0);
		if (!dst_cpu) {
			state = VB2_BUF_STATE_ERROR;
			goto done;
		}
		ret = histb_jpge_capture_begin_cpu(ctx, &dst->vb2_buf);
		if (ret) {
			state = VB2_BUF_STATE_ERROR;
			goto done;
		}
		cpu_access = true;

		stream_header = dst_cpu + HISTB_JPGE_STREAM_META_OFFSET;
		packet_len = le32_to_cpu(stream_header->packet_len);
		invalid_bytes = le32_to_cpu(stream_header->invalid_bytes);
		if (packet_len < sizeof(*stream_header) ||
		    packet_len > ctx->active_stream_len ||
		    invalid_bytes > packet_len - sizeof(*stream_header)) {
			state = VB2_BUF_STATE_ERROR;
			goto done;
		}

		scan_len = packet_len - sizeof(*stream_header) - invalid_bytes;
		if (scan_len > vb2_plane_size(&dst->vb2_buf, 0) -
			       HISTB_JPGE_HEADER_SIZE ||
		    histb_jpge_build_header(ctx, dst_cpu)) {
			state = VB2_BUF_STATE_ERROR;
			goto done;
		}

		payload = HISTB_JPGE_HEADER_SIZE + scan_len;
		vb2_set_plane_payload(&dst->vb2_buf, 0, payload);
	}

done:
	if (cpu_access) {
		end_ret = histb_jpge_capture_end_cpu(&dst->vb2_buf);
		if (end_ret)
			state = VB2_BUF_STATE_ERROR;
	}
	if (state != VB2_BUF_STATE_DONE)
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);

	src->sequence = ctx->src.sequence++;
	dst->sequence = ctx->dst.sequence++;
	v4l2_m2m_buf_copy_metadata(src, dst, false);
	dst->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
			V4L2_BUF_FLAG_BFRAME);
	if (state == VB2_BUF_STATE_DONE)
		dst->flags |= V4L2_BUF_FLAG_KEYFRAME;
	if (ctx->draining) {
		dst->flags |= V4L2_BUF_FLAG_LAST;
		ctx->draining = false;
		v4l2_m2m_mark_stopped(ctx->fh.m2m_ctx);
	}
	v4l2_m2m_buf_done_and_job_finish(jpge->m2m_dev, ctx->fh.m2m_ctx, state);
}

static void histb_jpge_watchdog(struct work_struct *work)
{
	struct histb_jpge_dev *jpge =
		container_of(to_delayed_work(work), struct histb_jpge_dev,
			     watchdog_work);
	struct histb_jpge_ctx *ctx;
	unsigned long flags;
	unsigned long delay;
	u64 deadline, now;

	mutex_lock(&jpge->launch_lock);
	now = ktime_get_ns();
	spin_lock_irqsave(&jpge->irqlock, flags);
	ctx = jpge->curr_ctx;
	deadline = jpge->watchdog_deadline_ns;
	if (ctx && deadline && now < deadline) {
		delay = max_t(unsigned long,
			      nsecs_to_jiffies(deadline - now), 1);
		spin_unlock_irqrestore(&jpge->irqlock, flags);
		mod_delayed_work(system_wq, &jpge->watchdog_work, delay);
		mutex_unlock(&jpge->launch_lock);
		return;
	}
	if (ctx) {
		jpge->curr_ctx = NULL;
		jpge->watchdog_deadline_ns = 0;
	}
	spin_unlock_irqrestore(&jpge->irqlock, flags);
	if (!ctx) {
		mutex_unlock(&jpge->launch_lock);
		return;
	}

	dev_err(jpge->dev, "JPEG encode timed out\n");
	ctx->job_aborting = false;
	if (histb_jpge_reset_engine(jpge))
		dev_err(jpge->dev, "failed to reset JPEG encoder\n");
	histb_jpge_finish_job(ctx, VB2_BUF_STATE_ERROR);
	mutex_unlock(&jpge->launch_lock);
}

static irqreturn_t histb_jpge_irq_thread(int irq, void *data)
{
	struct histb_jpge_dev *jpge = data;
	struct histb_jpge_ctx *ctx;
	u32 status;

	mutex_lock(&jpge->launch_lock);
	status = readl(jpge->regs + HISTB_JPGE_INTSTAT) & HISTB_JPGE_INT_ALL;
	if (!status) {
		mutex_unlock(&jpge->launch_lock);
		return IRQ_NONE;
	}

	writel(status, jpge->regs + HISTB_JPGE_INTCLR);
	writel(0, jpge->regs + HISTB_JPGE_INTMASK);

	ctx = v4l2_m2m_get_curr_priv(jpge->m2m_dev);
	if (!ctx || !histb_jpge_take_job(jpge, ctx)) {
		mutex_unlock(&jpge->launch_lock);
		return IRQ_HANDLED;
	}

	cancel_delayed_work(&jpge->watchdog_work);
	ctx->job_aborting = false;
	if ((status & HISTB_JPGE_INT_ERRORS) || !(status & HISTB_JPGE_INT_DONE)) {
		dev_err(jpge->dev, "JPEG encode failed, status %#x\n", status);
		if (histb_jpge_reset_engine(jpge))
			dev_err(jpge->dev, "failed to reset JPEG encoder\n");
		histb_jpge_finish_job(ctx, VB2_BUF_STATE_ERROR);
	} else {
		histb_jpge_finish_job(ctx, VB2_BUF_STATE_DONE);
	}
	mutex_unlock(&jpge->launch_lock);

	return IRQ_HANDLED;
}

static void histb_jpge_abort(struct histb_jpge_ctx *ctx)
{
	struct histb_jpge_dev *jpge = ctx->jpge;

	mutex_lock(&jpge->launch_lock);
	if (histb_jpge_take_job(jpge, ctx)) {
		cancel_delayed_work(&jpge->watchdog_work);
		ctx->job_aborting = false;
		if (histb_jpge_reset_engine(jpge))
			dev_err(jpge->dev, "failed to reset JPEG encoder\n");
		histb_jpge_finish_job(ctx, VB2_BUF_STATE_ERROR);
	} else if (v4l2_m2m_get_curr_priv(jpge->m2m_dev) == ctx) {
		ctx->job_aborting = true;
	}
	mutex_unlock(&jpge->launch_lock);
}

static void histb_jpge_job_abort(void *priv)
{
	histb_jpge_abort(priv);
}

static void histb_jpge_device_run(void *priv)
{

	struct histb_jpge_ctx *ctx = priv;
	struct histb_jpge_dev *jpge = ctx->jpge;
	struct vb2_v4l2_buffer *src, *dst;
	dma_addr_t src_dma, dst_dma, stream_dma;
	unsigned long flags;
	unsigned int dst_size, stream_len;
	u32 image_size, picfg;

	mutex_lock(&jpge->launch_lock);
	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (WARN_ON(!src || !dst)) {
		ctx->job_aborting = false;
		v4l2_m2m_job_finish(jpge->m2m_dev, ctx->fh.m2m_ctx);
		mutex_unlock(&jpge->launch_lock);
		return;
	}
	if (READ_ONCE(jpge->quiescing) || ctx->job_aborting ||
	    READ_ONCE(jpge->engine_faulted)) {
		ctx->job_aborting = false;
		histb_jpge_finish_job(ctx, VB2_BUF_STATE_ERROR);
		mutex_unlock(&jpge->launch_lock);
		return;
	}

	src_dma = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	dst_dma = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	dst_size = vb2_plane_size(&dst->vb2_buf, 0);
	if (dst_size <= HISTB_JPGE_STREAM_DATA_OFFSET ||
	    upper_32_bits(src_dma) || upper_32_bits(dst_dma) ||
	    !IS_ALIGNED(src_dma, 16)) {
		histb_jpge_finish_job(ctx, VB2_BUF_STATE_ERROR);
		mutex_unlock(&jpge->launch_lock);
		return;
	}

	histb_jpge_prepare_qtables(ctx);

	stream_dma = dst_dma + HISTB_JPGE_STREAM_META_OFFSET;
	if (upper_32_bits(stream_dma) ||
	    !IS_ALIGNED(stream_dma, HISTB_JPGE_STREAM_ADDR_ALIGN)) {
		histb_jpge_finish_job(ctx, VB2_BUF_STATE_ERROR);
		mutex_unlock(&jpge->launch_lock);
		return;
	}
	stream_len = (dst_size - HISTB_JPGE_STREAM_META_OFFSET) &
		     ~(HISTB_JPGE_STREAM_SIZE_ALIGN - 1);
	if (stream_len < HISTB_JPGE_STREAM_SIZE_ALIGN) {
		histb_jpge_finish_job(ctx, VB2_BUF_STATE_ERROR);
		mutex_unlock(&jpge->launch_lock);
		return;
	}

	spin_lock_irqsave(&jpge->irqlock, flags);
	if (WARN_ON(jpge->curr_ctx)) {
		spin_unlock_irqrestore(&jpge->irqlock, flags);
		histb_jpge_finish_job(ctx, VB2_BUF_STATE_ERROR);
		mutex_unlock(&jpge->launch_lock);
		return;
	}
	ctx->active_stream_len = stream_len;
	jpge->curr_ctx = ctx;
	jpge->watchdog_deadline_ns = ktime_get_ns() +
		(u64)HISTB_JPGE_WATCHDOG_MS * NSEC_PER_MSEC;
	spin_unlock_irqrestore(&jpge->irqlock, flags);

	writel(0, jpge->regs + HISTB_JPGE_INTMASK);
	writel(HISTB_JPGE_INT_ALL, jpge->regs + HISTB_JPGE_INTCLR);

	picfg = FIELD_PREP(HISTB_JPGE_PICFG_STORE, 0) |
		 FIELD_PREP(HISTB_JPGE_PICFG_SAMPLE, 0) |
		 FIELD_PREP(HISTB_JPGE_PICFG_CLK_GATE, 2) |
		 HISTB_JPGE_PICFG_MEM_CLK_GATE |
		 FIELD_PREP(HISTB_JPGE_PICFG_PACKAGE, 0xd8);
	writel_relaxed(picfg, jpge->regs + HISTB_JPGE_PICFG);
	writel_relaxed(0, jpge->regs + HISTB_JPGE_ECSCFG);
	image_size = FIELD_PREP(HISTB_JPGE_IMAGE_WIDTH,
				ctx->src.pix.width - 1) |
		     FIELD_PREP(HISTB_JPGE_IMAGE_HEIGHT,
				ctx->src.pix.height - 1);
	writel_relaxed(image_size, jpge->regs + HISTB_JPGE_IMAGE_SIZE);
	writel_relaxed(src_dma, jpge->regs + HISTB_JPGE_SRC_Y);
	writel_relaxed(src_dma + ctx->src.pix.bytesperline *
			 ctx->src.pix.height, jpge->regs + HISTB_JPGE_SRC_C);
	writel_relaxed(0, jpge->regs + HISTB_JPGE_SRC_V);
	writel_relaxed(ctx->src.pix.bytesperline |
			 ctx->src.pix.bytesperline << 16,
			 jpge->regs + HISTB_JPGE_SRC_STRIDE);
	writel_relaxed(stream_dma, jpge->regs + HISTB_JPGE_STREAM_ADDR);
	writel_relaxed(stream_dma - 16, jpge->regs + HISTB_JPGE_STREAM_RPTR);
	writel_relaxed(stream_dma - 32, jpge->regs + HISTB_JPGE_STREAM_WPTR);
	writel_relaxed(stream_len, jpge->regs + HISTB_JPGE_STREAM_LEN);
	writel_relaxed(0, jpge->regs + HISTB_JPGE_PTS0);
	writel_relaxed(2, jpge->regs + HISTB_JPGE_OUTSTD);
	histb_jpge_program_qtables(ctx);

	writel(HISTB_JPGE_INT_ALL, jpge->regs + HISTB_JPGE_INTMASK);
	mod_delayed_work(system_wq, &jpge->watchdog_work,
			 msecs_to_jiffies(HISTB_JPGE_WATCHDOG_MS));
	/* Publish DMA addresses and table programming before starting JPGE. */
	wmb();
	writel(0, jpge->regs + HISTB_JPGE_START);
	writel(1, jpge->regs + HISTB_JPGE_START);
	mutex_unlock(&jpge->launch_lock);
}

static const struct v4l2_m2m_ops histb_jpge_m2m_ops = {
	.device_run = histb_jpge_device_run,
	.job_abort = histb_jpge_job_abort,
};

static void histb_jpge_return_buffers(struct histb_jpge_ctx *ctx,
				      enum v4l2_buf_type type);

static int histb_jpge_queue_setup(struct vb2_queue *vq,
				  unsigned int *nbuffers,
				  unsigned int *nplanes,
				  unsigned int sizes[],
				  struct device *alloc_devs[])
{

	struct histb_jpge_ctx *ctx = vb2_get_drv_priv(vq);
	struct histb_jpge_q_data *q_data =
		histb_jpge_get_q_data(ctx, vq->type);

	if (*nplanes)
		return sizes[0] < q_data->pix.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = q_data->pix.sizeimage;

	return 0;
}

static int histb_jpge_buf_prepare(struct vb2_buffer *vb)
{

	struct histb_jpge_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct histb_jpge_q_data *q_data =
		histb_jpge_get_q_data(ctx, vb->vb2_queue->type);
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

static void histb_jpge_buf_queue(struct vb2_buffer *vb)
{
	struct histb_jpge_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int histb_jpge_start_streaming(struct vb2_queue *vq,
				      unsigned int count)
{

	struct histb_jpge_ctx *ctx = vb2_get_drv_priv(vq);
	struct histb_jpge_q_data *q_data =
		histb_jpge_get_q_data(ctx, vq->type);
	int ret;

	q_data->sequence = 0;
	mutex_lock(&ctx->jpge->launch_lock);
	ctx->job_aborting = false;
	mutex_unlock(&ctx->jpge->launch_lock);
	ret = pm_runtime_resume_and_get(ctx->jpge->dev);
	if (ret < 0) {
		histb_jpge_return_buffers(ctx, vq->type);
		return ret;
	}

	return 0;
}

static void histb_jpge_return_buffers(struct histb_jpge_ctx *ctx,
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

static void histb_jpge_stop_streaming(struct vb2_queue *vq)
{
	struct histb_jpge_ctx *ctx = vb2_get_drv_priv(vq);

	histb_jpge_abort(ctx);
	histb_jpge_return_buffers(ctx, vq->type);
	pm_runtime_mark_last_busy(ctx->jpge->dev);
	pm_runtime_put_autosuspend(ctx->jpge->dev);
}

static const struct vb2_ops histb_jpge_qops = {
	.queue_setup = histb_jpge_queue_setup,
	.buf_prepare = histb_jpge_buf_prepare,
	.buf_queue = histb_jpge_buf_queue,
	.start_streaming = histb_jpge_start_streaming,
	.stop_streaming = histb_jpge_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int histb_jpge_queue_init(void *priv, struct vb2_queue *src_vq,
				 struct vb2_queue *dst_vq)
{
	struct histb_jpge_ctx *ctx = priv;
	struct histb_jpge_dev *jpge = ctx->jpge;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	src_vq->ops = &histb_jpge_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &jpge->lock;
	src_vq->dev = jpge->dev;
	src_vq->dma_dir = DMA_TO_DEVICE;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	dst_vq->ops = &histb_jpge_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &jpge->lock;
	dst_vq->dev = jpge->dev;
	dst_vq->dma_dir = DMA_BIDIRECTIONAL;

	return vb2_queue_init(dst_vq);
}

static int histb_jpge_querycap(struct file *file, void *priv,
			       struct v4l2_capability *cap)
{
	strscpy(cap->driver, "histb-jpge", sizeof(cap->driver));
	strscpy(cap->card, "HiSilicon HiSTB JPEG encoder",
		sizeof(cap->card));

	return 0;
}

static int histb_jpge_enum_fmt(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		if (f->index)
			return -EINVAL;
		f->pixelformat = V4L2_PIX_FMT_NV12;
	} else {
		if (f->index > 1)
			return -EINVAL;
		f->pixelformat = f->index ? V4L2_PIX_FMT_MJPEG :
					       V4L2_PIX_FMT_JPEG;
	}

	return 0;
}

static int histb_jpge_enum_framesizes(struct file *file, void *priv,
				      struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index ||
	    (fsize->pixel_format != V4L2_PIX_FMT_NV12 &&
	     fsize->pixel_format != V4L2_PIX_FMT_JPEG &&
	     fsize->pixel_format != V4L2_PIX_FMT_MJPEG))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = HISTB_JPGE_MIN_DIMENSION;
	fsize->stepwise.max_width = HISTB_JPGE_MAX_DIMENSION;
	fsize->stepwise.step_width = 16;
	fsize->stepwise.min_height = HISTB_JPGE_MIN_DIMENSION;
	fsize->stepwise.max_height = HISTB_JPGE_MAX_DIMENSION;
	fsize->stepwise.step_height = 2;

	return 0;
}

static void histb_jpge_try_raw_format(struct v4l2_pix_format *pix)
{
	const u32 min = HISTB_JPGE_MIN_DIMENSION;
	const u32 max = HISTB_JPGE_MAX_DIMENSION;
	u32 width = pix->width;
	u32 height = pix->height;
	u32 bytesperline;

	v4l_bound_align_image(&width, min, max, 4, &height, min, max, 1, 0);
	bytesperline = clamp_t(u32, pix->bytesperline, width, U16_MAX & ~15);
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

static void histb_jpge_try_jpeg_format(struct histb_jpge_ctx *ctx,
				       struct v4l2_pix_format *pix)
{
	u32 min_size;

	min_size = ALIGN(ctx->src.pix.width * ctx->src.pix.height * 2 +
			 HISTB_JPGE_HEADER_SIZE,
			 HISTB_JPGE_STREAM_SIZE_ALIGN);
	pix->width = ctx->src.pix.width;
	pix->height = ctx->src.pix.height;
	if (pix->pixelformat != V4L2_PIX_FMT_MJPEG)
		pix->pixelformat = V4L2_PIX_FMT_JPEG;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = 0;
	pix->sizeimage = max(pix->sizeimage, min_size);
	pix->colorspace = ctx->src.pix.colorspace;
	pix->ycbcr_enc = ctx->src.pix.ycbcr_enc;
	pix->quantization = ctx->src.pix.quantization;
	pix->xfer_func = ctx->src.pix.xfer_func;
}

static int histb_jpge_try_fmt(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct histb_jpge_ctx *ctx = priv;

	if (V4L2_TYPE_IS_OUTPUT(f->type))
		histb_jpge_try_raw_format(&f->fmt.pix);
	else
		histb_jpge_try_jpeg_format(ctx, &f->fmt.pix);

	return 0;
}

static int histb_jpge_g_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct histb_jpge_ctx *ctx = priv;

	f->fmt.pix = histb_jpge_get_q_data(ctx, f->type)->pix;

	return 0;
}

static int histb_jpge_s_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct histb_jpge_ctx *ctx = priv;
	struct vb2_queue *vq;
	int ret;

	ret = histb_jpge_try_fmt(file, priv, f);
	if (ret)
		return ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq))
		return -EBUSY;

	histb_jpge_get_q_data(ctx, f->type)->pix = f->fmt.pix;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		struct v4l2_pix_format dst = ctx->dst.pix;

		histb_jpge_try_jpeg_format(ctx, &dst);
		ctx->dst.pix = dst;
	}

	return 0;
}

/*
 * Complete a capture buffer with V4L2_BUF_FLAG_LAST so a client that has
 * issued V4L2_ENC_CMD_STOP can leave poll().
 *
 * The engine only reports completion for a job it is running.  When the
 * client stops the stream after the last frame there is no further source
 * buffer, so no further job, so no further interrupt - the drain has
 * nothing to ride on and the client blocks in poll forever.  This is the
 * case FFmpeg hits with `-frames:v N`: the last frame is encoded
 * correctly, then the process hangs.  Measured on the board: both jobs
 * completed with INTSTAT = 0x1 (DONE) and the interrupt count stopped
 * rising while the encoder thread sat in do_sys_poll.
 *
 * When a job is in flight, or a source buffer is still queued, the normal
 * completion path emits LAST instead and this does nothing.
 */
static void histb_jpge_drain_now(struct histb_jpge_ctx *ctx, bool was_draining)
{
	struct histb_jpge_dev *jpge = ctx->jpge;
	struct vb2_v4l2_buffer *dst;
	unsigned long flags;

	if (was_draining)
		return;

	/*
	 * Nothing is in flight and nothing is queued, so no job will run and
	 * no interrupt will arrive to carry V4L2_BUF_FLAG_LAST.  This is the
	 * same situation v4l2_update_last_buf_state() handles for a driver
	 * that reaches it (v4l2-mem2mem.c:676-693), and it is handled the same
	 * way: take the buffer off the queue with v4l2_m2m_dst_buf_remove() so
	 * vb2 marks it ACTIVE, then complete it with
	 * v4l2_m2m_last_buffer_done(), which adds LAST and marks the m2m
	 * context stopped.  Completing it with v4l2_m2m_buf_done() instead
	 * leaves the buffer in VB2_BUF_STATE_QUEUED and vb2_buffer_done()
	 * warns (videobuf2-core.c:1187).
	 */
	mutex_lock(&jpge->launch_lock);
	spin_lock_irqsave(&jpge->irqlock, flags);
	if (jpge->curr_ctx || v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx)) {
		spin_unlock_irqrestore(&jpge->irqlock, flags);
		mutex_unlock(&jpge->launch_lock);
		return;
	}
	spin_unlock_irqrestore(&jpge->irqlock, flags);

	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	if (dst) {
		ctx->draining = false;
		v4l2_m2m_last_buffer_done(ctx->fh.m2m_ctx, dst);
	}
	mutex_unlock(&jpge->launch_lock);
}

static int histb_jpge_encoder_cmd(struct file *file, void *priv,
				  struct v4l2_encoder_cmd *cmd)
{
	struct histb_jpge_ctx *ctx = priv;
	bool was_draining;
	int ret;

	ret = v4l2_m2m_ioctl_try_encoder_cmd(file, priv, cmd);
	if (ret)
		return ret;

	switch (cmd->cmd) {
	case V4L2_ENC_CMD_STOP:
		was_draining = ctx->draining;
		ctx->draining = true;
		histb_jpge_drain_now(ctx, was_draining);
		break;
	case V4L2_ENC_CMD_START:
		ctx->draining = false;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ioctl_ops histb_jpge_ioctl_ops = {
	.vidioc_querycap = histb_jpge_querycap,
	.vidioc_enum_fmt_vid_cap = histb_jpge_enum_fmt,
	.vidioc_g_fmt_vid_cap = histb_jpge_g_fmt,
	.vidioc_try_fmt_vid_cap = histb_jpge_try_fmt,
	.vidioc_s_fmt_vid_cap = histb_jpge_s_fmt,
	.vidioc_enum_fmt_vid_out = histb_jpge_enum_fmt,
	.vidioc_g_fmt_vid_out = histb_jpge_g_fmt,
	.vidioc_try_fmt_vid_out = histb_jpge_try_fmt,
	.vidioc_s_fmt_vid_out = histb_jpge_s_fmt,
	.vidioc_enum_framesizes = histb_jpge_enum_framesizes,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_encoder_cmd = histb_jpge_encoder_cmd,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int histb_jpge_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct histb_jpge_ctx *ctx =
		container_of(ctrl->handler, struct histb_jpge_ctx, ctrl_handler);

	if (ctrl->id != V4L2_CID_JPEG_COMPRESSION_QUALITY)
		return -EINVAL;

	ctx->quality = ctrl->val;

	return 0;
}

static const struct v4l2_ctrl_ops histb_jpge_ctrl_ops = {
	.s_ctrl = histb_jpge_s_ctrl,
};

static void histb_jpge_set_default_formats(struct histb_jpge_ctx *ctx)
{
	ctx->src.pix.width = HISTB_JPGE_DEFAULT_WIDTH;
	ctx->src.pix.height = HISTB_JPGE_DEFAULT_HEIGHT;
	ctx->src.pix.bytesperline = HISTB_JPGE_DEFAULT_WIDTH;
	histb_jpge_try_raw_format(&ctx->src.pix);
	ctx->dst.pix.width = HISTB_JPGE_DEFAULT_WIDTH;
	ctx->dst.pix.height = HISTB_JPGE_DEFAULT_HEIGHT;
	histb_jpge_try_jpeg_format(ctx, &ctx->dst.pix);
}

static int histb_jpge_open(struct file *file)
{
	struct histb_jpge_dev *jpge = video_drvdata(file);
	struct histb_jpge_ctx *ctx;
	struct v4l2_m2m_dev *m2m_dev = jpge->m2m_dev;
	struct v4l2_m2m_ctx *m2m_ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->jpge = jpge;
	ctx->quality = HISTB_JPGE_DEFAULT_QUALITY;
	histb_jpge_set_default_formats(ctx);
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 1);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &histb_jpge_ctrl_ops,
			  V4L2_CID_JPEG_COMPRESSION_QUALITY,
			  1, 100, 1, HISTB_JPGE_DEFAULT_QUALITY);
	if (ctx->ctrl_handler.error) {
		ret = ctx->ctrl_handler.error;
		goto free_fh;
	}
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;

	m2m_ctx = v4l2_m2m_ctx_init(m2m_dev, ctx, histb_jpge_queue_init);
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

static int histb_jpge_release(struct file *file)
{
	struct histb_jpge_ctx *ctx =
		container_of(file->private_data, struct histb_jpge_ctx, fh);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations histb_jpge_fops = {
	.owner = THIS_MODULE,
	.open = histb_jpge_open,
	.release = histb_jpge_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static int histb_jpge_runtime_resume(struct device *dev)
{
	struct histb_jpge_dev *jpge = dev_get_drvdata(dev);
	int ret;

	WRITE_ONCE(jpge->engine_faulted, true);
	ret = clk_prepare_enable(jpge->clk);
	if (ret)
		return ret;

	ret = reset_control_deassert(jpge->reset);
	if (ret) {
		reset_control_assert(jpge->reset);
		clk_disable_unprepare(jpge->clk);
		return ret;
	}

	writel(0, jpge->regs + HISTB_JPGE_INTMASK);
	writel(HISTB_JPGE_INT_ALL, jpge->regs + HISTB_JPGE_INTCLR);
	enable_irq(jpge->irq);
	WRITE_ONCE(jpge->engine_faulted, false);

	return 0;
}

static int histb_jpge_runtime_suspend(struct device *dev)
{
	struct histb_jpge_dev *jpge = dev_get_drvdata(dev);
	bool faulted = READ_ONCE(jpge->engine_faulted);

	WRITE_ONCE(jpge->engine_faulted, true);
	disable_irq(jpge->irq);
	if (!faulted) {
		writel(0, jpge->regs + HISTB_JPGE_INTMASK);
		writel(HISTB_JPGE_INT_ALL, jpge->regs + HISTB_JPGE_INTCLR);
	}
	reset_control_assert(jpge->reset);
	clk_disable_unprepare(jpge->clk);

	return 0;
}

static int __maybe_unused histb_jpge_suspend(struct device *dev)
{
	struct histb_jpge_dev *jpge = dev_get_drvdata(dev);
	int ret;

	WRITE_ONCE(jpge->quiescing, true);
	v4l2_m2m_suspend(jpge->m2m_dev);
	ret = pm_runtime_force_suspend(dev);
	if (ret) {
		WRITE_ONCE(jpge->quiescing, false);
		v4l2_m2m_resume(jpge->m2m_dev);
	}

	return ret;
}

static int __maybe_unused histb_jpge_resume(struct device *dev)
{
	struct histb_jpge_dev *jpge = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (!ret) {
		WRITE_ONCE(jpge->quiescing, false);
		v4l2_m2m_resume(jpge->m2m_dev);
	}

	return ret;
}

static const struct dev_pm_ops histb_jpge_pm_ops = {
	SET_RUNTIME_PM_OPS(histb_jpge_runtime_suspend,
			   histb_jpge_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(histb_jpge_suspend, histb_jpge_resume)
};

static int histb_jpge_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_jpge_dev *jpge;
	int ret;

	jpge = devm_kzalloc(dev, sizeof(*jpge), GFP_KERNEL);
	if (!jpge)
		return -ENOMEM;

	jpge->dev = dev;
	jpge->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(jpge->regs))
		return PTR_ERR(jpge->regs);

	jpge->clk = devm_clk_get(dev, "core");
	if (IS_ERR(jpge->clk))
		return dev_err_probe(dev, PTR_ERR(jpge->clk),
				     "failed to get core clock\n");

	jpge->reset = devm_reset_control_get_exclusive(dev, "jpge");
	if (IS_ERR(jpge->reset))
		return dev_err_probe(dev, PTR_ERR(jpge->reset),
				     "failed to get reset\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	jpge->irq = platform_get_irq(pdev, 0);
	if (jpge->irq < 0)
		return jpge->irq;

	mutex_init(&jpge->lock);
	mutex_init(&jpge->launch_lock);
	spin_lock_init(&jpge->irqlock);
	INIT_DELAYED_WORK(&jpge->watchdog_work, histb_jpge_watchdog);
	platform_set_drvdata(pdev, jpge);

	ret = devm_request_threaded_irq(dev, jpge->irq, NULL,
					histb_jpge_irq_thread,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					dev_name(dev), jpge);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	ret = v4l2_device_register(dev, &jpge->v4l2_dev);
	if (ret)
		return ret;

	jpge->m2m_dev = v4l2_m2m_init(&histb_jpge_m2m_ops);
	if (IS_ERR(jpge->m2m_dev)) {
		ret = PTR_ERR(jpge->m2m_dev);
		goto unregister_v4l2;
	}

	strscpy(jpge->vfd.name, "histb-jpge", sizeof(jpge->vfd.name));
	jpge->vfd.fops = &histb_jpge_fops;
	jpge->vfd.ioctl_ops = &histb_jpge_ioctl_ops;
	jpge->vfd.v4l2_dev = &jpge->v4l2_dev;
	jpge->vfd.lock = &jpge->lock;
	jpge->vfd.release = video_device_release_empty;
	jpge->vfd.vfl_dir = VFL_DIR_M2M;
	jpge->vfd.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	jpge->vfd.entity.function = MEDIA_ENT_F_PROC_VIDEO_ENCODER;
	video_set_drvdata(&jpge->vfd, jpge);

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);

	ret = video_register_device(&jpge->vfd, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto disable_runtime_pm;

	dev_info(dev, "registered as /dev/%s\n",
		 video_device_node_name(&jpge->vfd));

	return 0;

disable_runtime_pm:
	pm_runtime_disable(dev);
	v4l2_m2m_release(jpge->m2m_dev);
unregister_v4l2:
	v4l2_device_unregister(&jpge->v4l2_dev);

	return ret;
}

static void histb_jpge_remove(struct platform_device *pdev)
{
	struct histb_jpge_dev *jpge = platform_get_drvdata(pdev);

	WRITE_ONCE(jpge->quiescing, true);
	v4l2_m2m_suspend(jpge->m2m_dev);
	synchronize_irq(jpge->irq);
	cancel_delayed_work_sync(&jpge->watchdog_work);
	video_unregister_device(&jpge->vfd);
	pm_runtime_disable(jpge->dev);
	if (!pm_runtime_status_suspended(jpge->dev))
		histb_jpge_runtime_suspend(jpge->dev);
	v4l2_m2m_release(jpge->m2m_dev);
	v4l2_device_unregister(&jpge->v4l2_dev);
}

static const struct of_device_id histb_jpge_of_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-jpge" },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_jpge_of_match);

static struct platform_driver histb_jpge_driver = {
	.probe = histb_jpge_probe,
	.remove_new = histb_jpge_remove,
	.driver = {
		.name = "histb-jpge",
		.of_match_table = histb_jpge_of_match,
		.pm = &histb_jpge_pm_ops,
	},
};
module_platform_driver(histb_jpge_driver);

MODULE_AUTHOR("HiSilicon Technologies Co., Ltd.");
MODULE_DESCRIPTION("HiSilicon HiSTB JPEG encoder");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(DMA_BUF);
