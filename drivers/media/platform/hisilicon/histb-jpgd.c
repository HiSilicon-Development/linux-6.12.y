// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon HiSTB stand-alone JPEG decoder (JPGD)
 *
 * Register programming and table packing follow the HiSTB JPEG HAL shared by
 * Hi3798CV200 and Hi3798MV310. Only complete baseline sequential frames run.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
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
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#define HISTB_JPGD_REG_START			0x0000
#define HISTB_JPGD_REG_RESUME			0x0004
#define HISTB_JPGD_REG_PICVLDNUM		0x0008
#define HISTB_JPGD_REG_STRIDE			0x000c
#define HISTB_JPGD_REG_PICSIZE			0x0010
#define HISTB_JPGD_REG_PICTYPE			0x0014
#define HISTB_JPGD_REG_TIME			0x0018
#define HISTB_JPGD_REG_STADDR			0x0020
#define HISTB_JPGD_REG_ENDADDR			0x0024
#define HISTB_JPGD_REG_STADD			0x0028
#define HISTB_JPGD_REG_ENDADD			0x002c
#define HISTB_JPGD_REG_YSTADDR			0x0030
#define HISTB_JPGD_REG_UVSTADDR			0x0034
#define HISTB_JPGD_REG_SCALE			0x0040
#define HISTB_JPGD_REG_OUTTYPE			0x0044
#define HISTB_JPGD_REG_INT			0x0100
#define HISTB_JPGD_REG_INTMASK			0x0104
#define HISTB_JPGD_REG_DEBUG			0x0108
#define HISTB_JPGD_REG_SAMPLINGFACTOR		0x0124
#define HISTB_JPGD_REG_DRI			0x0128
#define HISTB_JPGD_REG_MMU_BYPASS		0x0134
#define HISTB_JPGD_REG_QUANT			0x0200
#define HISTB_JPGD_REG_HDCTABLE			0x0300
#define HISTB_JPGD_REG_HACMINTABLE		0x0340
#define HISTB_JPGD_REG_HACBASETABLE		0x0360
#define HISTB_JPGD_REG_HACSYMTABLE		0x0400
/* HACSYMTABLE contains 176 32-bit words, ending at offset 0x6bc. */
#define HISTB_JPGD_REG_SIZE			0x06c0

#define HISTB_JPGD_INT_FINISH			BIT(0)
#define HISTB_JPGD_INT_ERROR			BIT(1)
#define HISTB_JPGD_INT_CONTINUE			BIT(2)
#define HISTB_JPGD_INT_LOWDELAY			BIT(5)
#define HISTB_JPGD_INT_STREAM_ERR		BIT(6)
#define HISTB_JPGD_INT_ALL			(HISTB_JPGD_INT_FINISH | \
						 HISTB_JPGD_INT_ERROR | \
						 HISTB_JPGD_INT_CONTINUE | \
						 HISTB_JPGD_INT_STREAM_ERR)

#define HISTB_JPGD_MIN_DIMENSION		1
/* The selected BSP hardware parser accepts source dimensions up to 8095. */
#define HISTB_JPGD_MAX_DIMENSION		8095
#define HISTB_JPGD_DEFAULT_WIDTH		1280
#define HISTB_JPGD_DEFAULT_HEIGHT		720
#define HISTB_JPGD_WATCHDOG_MS			3000

#include "histb-jpgd-jpeg.h"

struct histb_jpgd_buffer {
	struct v4l2_m2m_buffer m2m;
	struct histb_jpgd_image image;
	int parse_error;
};

struct histb_jpgd_fmt {
	u32 fourcc;
};

/* Native SP is Cr,Cb: BSP jpeg_hdec_debug.c extracts U from odd bytes.
 * OUTTYPE bit 2 downsamples color inputs to 4:2:0; no packed YUYV mode.
 */
static const struct histb_jpgd_fmt histb_jpgd_formats[] = {
	{ .fourcc = V4L2_PIX_FMT_NV21 },
	{ .fourcc = V4L2_PIX_FMT_NV61 },
	{ .fourcc = V4L2_PIX_FMT_GREY },
};

static const struct histb_jpgd_fmt *histb_jpgd_find_format(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(histb_jpgd_formats); i++) {
		if (histb_jpgd_formats[i].fourcc == fourcc)
			return &histb_jpgd_formats[i];
	}

	return &histb_jpgd_formats[0];
}

struct histb_jpgd_q_data {
	struct v4l2_pix_format pix;
	size_t size;
};

struct histb_jpgd_dev {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct video_device vfd;
	void __iomem *regs;
	struct clk_bulk_data clocks[2];
	struct reset_control *core_reset;
	struct reset_control *smmu_reset;
	int irq;

	struct mutex dev_mutex;
	struct histb_jpgd_ctx *active;
	unsigned long deadline;
	struct mutex launch_lock;
	bool removing;
	bool clock_enabled;
	struct workqueue_struct *workqueue;

	struct delayed_work watchdog_work;
};

struct histb_jpgd_ctx {
	struct v4l2_fh fh;
	struct histb_jpgd_dev *jpgd;

	struct histb_jpgd_q_data src;
	struct histb_jpgd_q_data dst;

	struct histb_jpgd_parser parser;
	struct work_struct finish_work;
	enum vb2_buffer_state finish_state;
	bool aborting;
};

static inline struct histb_jpgd_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct histb_jpgd_ctx, fh);
}


static struct histb_jpgd_q_data *
histb_jpgd_get_q_data(struct histb_jpgd_ctx *ctx, enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->src;
	return &ctx->dst;
}

static int histb_jpgd_reset_engine(struct histb_jpgd_dev *jpgd)
{
	int ret;

	writel(0x7f, jpgd->regs + HISTB_JPGD_REG_INTMASK);
	ret = reset_control_assert(jpgd->core_reset);
	if (ret)
		return ret;
	udelay(5);
	ret = reset_control_deassert(jpgd->core_reset);
	if (ret)
		return ret;
	udelay(10);
	writel(0x7f, jpgd->regs + HISTB_JPGD_REG_INTMASK);
	writel(0x7f, jpgd->regs + HISTB_JPGD_REG_INT);
	return 0;
}

/* launch_lock excludes other register users, including the threaded IRQ.
 * If reset fails, clock gating is the final DMA stop before buffers return.
 */
static int histb_jpgd_recover_engine(struct histb_jpgd_dev *jpgd)
{
	int ret = histb_jpgd_reset_engine(jpgd);

	if (ret) {
		dev_err(jpgd->dev, "JPEG reset failed: %d; engine stopped\n", ret);
		disable_irq_nosync(jpgd->irq);
		clk_bulk_disable_unprepare(ARRAY_SIZE(jpgd->clocks), jpgd->clocks);
		jpgd->clock_enabled = false;
	}
	return ret;
}

/* Queue every completion source, including immediate launch errors, so close
 * can join the entire M2M completion helper before freeing the context.
 */
static void histb_jpgd_finish_work(struct work_struct *work)
{
	struct histb_jpgd_ctx *ctx = container_of(work, struct histb_jpgd_ctx,
					       finish_work);
	struct histb_jpgd_dev *jpgd = ctx->jpgd;
	enum vb2_buffer_state state = ctx->finish_state;
	struct vb2_v4l2_buffer *src, *dst;

	src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	if (dst) {
		dst->vb2_buf.timestamp = src ? src->vb2_buf.timestamp : 0;
		vb2_set_plane_payload(&dst->vb2_buf, 0,
			state == VB2_BUF_STATE_DONE ? ctx->dst.size : 0);
	}
	if (src)
		v4l2_m2m_buf_done(src, state);
	if (dst)
		v4l2_m2m_buf_done(dst, state);
	v4l2_m2m_job_finish(jpgd->m2m_dev, ctx->fh.m2m_ctx);
}

static void histb_jpgd_finish_job(struct histb_jpgd_ctx *ctx,
				  enum vb2_buffer_state state)
{
	ctx->finish_state = state;
	queue_work(ctx->jpgd->workqueue, &ctx->finish_work);
}

static void histb_jpgd_watchdog(struct work_struct *work)
{
	struct histb_jpgd_dev *jpgd = container_of(to_delayed_work(work),
				struct histb_jpgd_dev, watchdog_work);
	struct histb_jpgd_ctx *ctx;
	unsigned long now;

	mutex_lock(&jpgd->launch_lock);
	now = jiffies;
	ctx = jpgd->active;
	if (ctx && time_before(now, jpgd->deadline)) {
		/* An already-running old timeout must not kill the next job. */
		mod_delayed_work(system_wq, &jpgd->watchdog_work,
				 jpgd->deadline - now);
		ctx = NULL;
	} else if (ctx) {
		jpgd->active = NULL;
		histb_jpgd_recover_engine(jpgd);
		dev_err(jpgd->dev, "JPEG decode timed out\n");
	}
	mutex_unlock(&jpgd->launch_lock);
	if (ctx)
		histb_jpgd_finish_job(ctx, VB2_BUF_STATE_ERROR);
}

static irqreturn_t histb_jpgd_irq(int irq, void *data)
{
	struct histb_jpgd_dev *jpgd = data;
	struct histb_jpgd_ctx *ctx;
	enum vb2_buffer_state state;
	u32 status;

	mutex_lock(&jpgd->launch_lock);
	if (!jpgd->clock_enabled) {
		mutex_unlock(&jpgd->launch_lock);
		return IRQ_NONE;
	}
	status = readl(jpgd->regs + HISTB_JPGD_REG_INT) & HISTB_JPGD_INT_ALL;
	if (!status) {
		mutex_unlock(&jpgd->launch_lock);
		return IRQ_NONE;
	}
	writel(status, jpgd->regs + HISTB_JPGD_REG_INT);
	ctx = jpgd->active;
	jpgd->active = NULL;
	cancel_delayed_work(&jpgd->watchdog_work);
	/* CONTINUE with the complete input and EOF set is a truncated stream. */
	state = status & (HISTB_JPGD_INT_ERROR | HISTB_JPGD_INT_STREAM_ERR |
			  HISTB_JPGD_INT_CONTINUE) ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE;
	if (state == VB2_BUF_STATE_ERROR)
		histb_jpgd_recover_engine(jpgd);
	else
		writel(0x7f, jpgd->regs + HISTB_JPGD_REG_INTMASK);
	mutex_unlock(&jpgd->launch_lock);
	if (ctx)
		histb_jpgd_finish_job(ctx, state);
	return IRQ_HANDLED;
}

static void histb_jpgd_job_abort(void *priv)
{
	struct histb_jpgd_ctx *ctx = priv;
	struct histb_jpgd_dev *jpgd = ctx->jpgd;
	bool active;

	mutex_lock(&jpgd->launch_lock);
	ctx->aborting = true;
	active = jpgd->active == ctx;
	if (active) {
		jpgd->active = NULL;
		cancel_delayed_work(&jpgd->watchdog_work);
		histb_jpgd_recover_engine(jpgd);
	}
	mutex_unlock(&jpgd->launch_lock);
	if (active)
		histb_jpgd_finish_job(ctx, VB2_BUF_STATE_ERROR);
}

static void histb_jpgd_write_table(struct histb_jpgd_dev *jpgd,
				 unsigned int reg, const u32 *table, size_t words)
{
	unsigned int i;

	for (i = 0; i < words; i++)
		writel(table[i], jpgd->regs + reg + i * 4);
}

static void histb_jpgd_device_run(void *priv)
{
	struct histb_jpgd_ctx *ctx = priv;
	struct histb_jpgd_dev *jpgd = ctx->jpgd;
	struct vb2_v4l2_buffer *src, *dst;
	struct histb_jpgd_buffer *buffer;
	struct histb_jpgd_image *img;
	dma_addr_t src_dma, dst_dma;
	u32 stride = ctx->dst.pix.bytesperline;
	u32 y_size = stride * ALIGN(ctx->dst.pix.height, 16);

	mutex_lock(&jpgd->launch_lock);
	if (jpgd->removing || !jpgd->clock_enabled || ctx->aborting)
		goto error;
	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	buffer = container_of(src, struct histb_jpgd_buffer, m2m.vb);
	img = &buffer->image;
	if (buffer->parse_error ||
	    ctx->dst.pix.width != (ctx->dst.pix.pixelformat == V4L2_PIX_FMT_GREY ?
				   img->width : ALIGN(img->width, 2)) ||
	    ctx->dst.pix.height != ALIGN(img->height, 16) ||
	    ((ctx->dst.pix.pixelformat == V4L2_PIX_FMT_GREY) != !img->picture_type) ||
	    (ctx->dst.pix.pixelformat == V4L2_PIX_FMT_NV61 && img->picture_type != 4))
		goto error;
	src_dma = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	dst_dma = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	if (!IS_ALIGNED(src_dma, 128) || !IS_ALIGNED(dst_dma, 64) ||
	    img->stream_end > round_down(vb2_plane_size(&src->vb2_buf, 0), 128) ||
	    src_dma > U32_MAX ||
	    vb2_plane_size(&src->vb2_buf, 0) > U32_MAX - src_dma ||
	    dst_dma > U32_MAX || ctx->dst.size > U32_MAX - dst_dma)
		goto error;

	if (histb_jpgd_recover_engine(jpgd))
		goto error;
	/* BSP physical per-client bypass; no private SMMU page tables. */
	writel(0x1ff, jpgd->regs + HISTB_JPGD_REG_MMU_BYPASS);
	histb_jpgd_write_table(jpgd, HISTB_JPGD_REG_QUANT, img->quant, 64);
	histb_jpgd_write_table(jpgd, HISTB_JPGD_REG_HDCTABLE, img->dc, 12);
	histb_jpgd_write_table(jpgd, HISTB_JPGD_REG_HACMINTABLE, img->ac_min, 8);
	histb_jpgd_write_table(jpgd, HISTB_JPGD_REG_HACBASETABLE, img->ac_base, 8);
	histb_jpgd_write_table(jpgd, HISTB_JPGD_REG_HACSYMTABLE, img->ac_symbol, 176);
	writel(img->picture_type, jpgd->regs + HISTB_JPGD_REG_PICTYPE);
	writel(img->sampling, jpgd->regs + HISTB_JPGD_REG_SAMPLINGFACTOR);
	writel(img->restart_interval, jpgd->regs + HISTB_JPGD_REG_DRI);
	writel(img->mcu_size, jpgd->regs + HISTB_JPGD_REG_PICSIZE);
	writel(stride << 16 | stride, jpgd->regs + HISTB_JPGD_REG_STRIDE);
	writel(lower_32_bits(dst_dma), jpgd->regs + HISTB_JPGD_REG_YSTADDR);
	writel(lower_32_bits(img->picture_type ? dst_dma + y_size : dst_dma),
	       jpgd->regs + HISTB_JPGD_REG_UVSTADDR);
	writel(0x3c, jpgd->regs + HISTB_JPGD_REG_SCALE);
	writel(ctx->dst.pix.pixelformat == V4L2_PIX_FMT_NV21 ? 4 : 0,
	       jpgd->regs + HISTB_JPGD_REG_OUTTYPE);
	writel(lower_32_bits(src_dma), jpgd->regs + HISTB_JPGD_REG_STADDR);
	writel(lower_32_bits(src_dma + round_down(vb2_plane_size(&src->vb2_buf, 0), 128)),
	       jpgd->regs + HISTB_JPGD_REG_ENDADDR);
	writel(lower_32_bits(src_dma + img->entropy_offset), jpgd->regs + HISTB_JPGD_REG_STADD);
	writel(lower_32_bits(src_dma + img->stream_end), jpgd->regs + HISTB_JPGD_REG_ENDADD);
	writel(0, jpgd->regs + 0x154); /* BSP ERR_STREAM_EN */
	jpgd->active = ctx;
	jpgd->deadline = jiffies + msecs_to_jiffies(HISTB_JPGD_WATCHDOG_MS);
	mod_delayed_work(system_wq, &jpgd->watchdog_work,
			 msecs_to_jiffies(HISTB_JPGD_WATCHDOG_MS));
	/* INTMASK is active-high masking: BSP masks only low-delay IRQ. */
	writel(0x20, jpgd->regs + HISTB_JPGD_REG_INTMASK);
	writel(2, jpgd->regs + HISTB_JPGD_REG_RESUME);
	writel(1, jpgd->regs + HISTB_JPGD_REG_START);
	mutex_unlock(&jpgd->launch_lock);
	return;
error:
	mutex_unlock(&jpgd->launch_lock);
	histb_jpgd_finish_job(ctx, VB2_BUF_STATE_ERROR);
}

static int histb_jpgd_queue_setup(struct vb2_queue *vq,
				  unsigned int *nbuffers, unsigned int *nplanes,
				  unsigned int sizes[], struct device *alloc_devs[])
{
	struct histb_jpgd_ctx *ctx = vb2_get_drv_priv(vq);
	struct histb_jpgd_q_data *q_data = histb_jpgd_get_q_data(ctx, vq->type);

	if (*nplanes)
		return *nplanes != 1 || sizes[0] < q_data->size ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = q_data->size;

	return 0;
}

static int histb_jpgd_buf_prepare(struct vb2_buffer *vb)
{
	struct histb_jpgd_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct histb_jpgd_q_data *q = histb_jpgd_get_q_data(ctx, vb->vb2_queue->type);
	struct histb_jpgd_buffer *buffer = container_of(to_vb2_v4l2_buffer(vb),
					struct histb_jpgd_buffer, m2m.vb);
	struct dma_buf *dbuf = vb->planes[0].dbuf;
	void *vaddr;
	int ret = 0;

	if (vb2_plane_size(vb, 0) < q->size || vb->planes[0].data_offset)
		return -EINVAL;
	if (!V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		vb2_set_plane_payload(vb, 0, 0);
		return 0;
	}
	if (dbuf) {
		ret = dma_buf_begin_cpu_access(dbuf, DMA_TO_DEVICE);
		if (ret)
			return ret;
	}
	vaddr = vb2_plane_vaddr(vb, 0);
	if (!vaddr) {
		ret = -EFAULT;
	} else {
		/* Keep syntax with the queued buffer. Bad input is dequeued ERROR. */
		buffer->parse_error = histb_jpgd_parse(&ctx->parser, &buffer->image,
				vaddr, vb2_get_plane_payload(vb, 0));
	}
	if (dbuf) {
		int end_ret = dma_buf_end_cpu_access(dbuf, DMA_TO_DEVICE);

		if (!ret)
			ret = end_ret;
	}
	return ret;
}

static void histb_jpgd_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct histb_jpgd_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int histb_jpgd_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct histb_jpgd_ctx *ctx = vb2_get_drv_priv(vq);

	mutex_lock(&ctx->jpgd->launch_lock);
	ctx->aborting = false;
	mutex_unlock(&ctx->jpgd->launch_lock);
	return 0;
}

static void histb_jpgd_stop_streaming(struct vb2_queue *vq)
{
	struct histb_jpgd_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *vbuf;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

		if (!vbuf)
			break;
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops histb_jpgd_qops = {
	.queue_setup = histb_jpgd_queue_setup,
	.buf_prepare = histb_jpgd_buf_prepare,
	.buf_queue = histb_jpgd_buf_queue,
	.start_streaming = histb_jpgd_start_streaming,
	.stop_streaming = histb_jpgd_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int histb_jpgd_querycap(struct file *file, void *priv,
			       struct v4l2_capability *cap)
{
	strscpy(cap->driver, "histb-jpgd", sizeof(cap->driver));
	strscpy(cap->card, "HiSilicon HiSTB JPEG decoder", sizeof(cap->card));
	return 0;
}

static int histb_jpgd_enum_fmt(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		if (f->index > 0)
			return -EINVAL;
		f->pixelformat = V4L2_PIX_FMT_JPEG;
		return 0;
	}

	if (f->index >= ARRAY_SIZE(histb_jpgd_formats))
		return -EINVAL;

	f->pixelformat = histb_jpgd_formats[f->index].fourcc;
	return 0;
}

static int histb_jpgd_g_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct histb_jpgd_ctx *ctx = fh_to_ctx(priv);
	struct histb_jpgd_q_data *q_data = histb_jpgd_get_q_data(ctx, f->type);

	f->fmt.pix = q_data->pix;
	return 0;
}

static int histb_jpgd_try_fmt(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	u32 max_dimension = V4L2_TYPE_IS_OUTPUT(f->type) ?
		HISTB_JPGD_MAX_DIMENSION : ALIGN(HISTB_JPGD_MAX_DIMENSION, 16);

	pix->width = clamp_t(u32, pix->width, HISTB_JPGD_MIN_DIMENSION,
			     max_dimension);
	pix->height = clamp_t(u32, pix->height, HISTB_JPGD_MIN_DIMENSION,
			      max_dimension);
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_SRGB;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		pix->pixelformat = V4L2_PIX_FMT_JPEG;
		pix->bytesperline = 0;
		pix->sizeimage = clamp_t(u32, pix->sizeimage, 128,
				HISTB_JPGD_MAX_DIMENSION * HISTB_JPGD_MAX_DIMENSION * 4);
		pix->sizeimage = ALIGN(pix->sizeimage, 128);
		return 0;
	}
	pix->pixelformat = histb_jpgd_find_format(pix->pixelformat)->fourcc;
	if (pix->pixelformat != V4L2_PIX_FMT_GREY)
		pix->width = ALIGN(pix->width, 2);
	/* Expose padded coded height: chroma starts at bytesperline * height. */
	pix->height = ALIGN(pix->height, 16);
	pix->bytesperline = ALIGN(pix->width, 64);
	pix->sizeimage = pix->bytesperline * pix->height;
	if (pix->pixelformat == V4L2_PIX_FMT_NV21)
		pix->sizeimage += pix->sizeimage / 2;
	else if (pix->pixelformat == V4L2_PIX_FMT_NV61)
		pix->sizeimage *= 2;
	return 0;
}

static int histb_jpgd_s_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct histb_jpgd_ctx *ctx = fh_to_ctx(priv);
	struct histb_jpgd_q_data *q_data = histb_jpgd_get_q_data(ctx, f->type);
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	int ret;

	if (vb2_is_busy(vq))
		return -EBUSY;

	ret = histb_jpgd_try_fmt(file, priv, f);
	if (ret)
		return ret;

	q_data->pix = f->fmt.pix;
	q_data->size = f->fmt.pix.sizeimage;

	return 0;
}

static const struct v4l2_ioctl_ops histb_jpgd_ioctl_ops = {
	.vidioc_querycap = histb_jpgd_querycap,
	.vidioc_enum_fmt_vid_cap = histb_jpgd_enum_fmt,
	.vidioc_enum_fmt_vid_out = histb_jpgd_enum_fmt,
	.vidioc_g_fmt_vid_cap = histb_jpgd_g_fmt,
	.vidioc_g_fmt_vid_out = histb_jpgd_g_fmt,
	.vidioc_try_fmt_vid_cap = histb_jpgd_try_fmt,
	.vidioc_try_fmt_vid_out = histb_jpgd_try_fmt,
	.vidioc_s_fmt_vid_cap = histb_jpgd_s_fmt,
	.vidioc_s_fmt_vid_out = histb_jpgd_s_fmt,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int histb_jpgd_queue_init(void *priv, struct vb2_queue *src_vq,
				 struct vb2_queue *dst_vq)
{
	struct histb_jpgd_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct histb_jpgd_buffer);
	src_vq->ops = &histb_jpgd_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->jpgd->dev_mutex;
	src_vq->dev = ctx->jpgd->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &histb_jpgd_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->jpgd->dev_mutex;
	dst_vq->dev = ctx->jpgd->dev;

	return vb2_queue_init(dst_vq);
}

static int histb_jpgd_open(struct file *file)
{
	struct histb_jpgd_dev *jpgd = video_drvdata(file);
	struct histb_jpgd_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	mutex_lock(&jpgd->dev_mutex);
	if (jpgd->removing || !video_is_registered(&jpgd->vfd)) {
		mutex_unlock(&jpgd->dev_mutex);
		kfree(ctx);
		return -ENODEV;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	ctx->jpgd = jpgd;
	INIT_WORK(&ctx->finish_work, histb_jpgd_finish_work);

	ctx->src.pix.pixelformat = V4L2_PIX_FMT_JPEG;
	ctx->src.pix.width = HISTB_JPGD_DEFAULT_WIDTH;
	ctx->src.pix.height = HISTB_JPGD_DEFAULT_HEIGHT;
	ctx->src.size = HISTB_JPGD_DEFAULT_WIDTH * HISTB_JPGD_DEFAULT_HEIGHT;
	ctx->src.pix.sizeimage = ctx->src.size;
	ctx->src.pix.field = V4L2_FIELD_NONE;

	ctx->dst.pix.pixelformat = V4L2_PIX_FMT_NV21;
	ctx->dst.pix.width = HISTB_JPGD_DEFAULT_WIDTH;
	ctx->dst.pix.height = HISTB_JPGD_DEFAULT_HEIGHT;
	ctx->dst.pix.bytesperline = HISTB_JPGD_DEFAULT_WIDTH;
	ctx->dst.size = HISTB_JPGD_DEFAULT_WIDTH * HISTB_JPGD_DEFAULT_HEIGHT * 3 / 2;
	ctx->dst.pix.sizeimage = ctx->dst.size;
	ctx->dst.pix.field = V4L2_FIELD_NONE;
	ctx->dst.pix.colorspace = V4L2_COLORSPACE_JPEG;
	ctx->dst.pix.quantization = V4L2_QUANTIZATION_FULL_RANGE;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(jpgd->m2m_dev, ctx,
					    histb_jpgd_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		mutex_unlock(&jpgd->dev_mutex);
		kfree(ctx);
		return ret;
	}

	v4l2_fh_add(&ctx->fh);
	mutex_unlock(&jpgd->dev_mutex);
	return 0;
}

static int histb_jpgd_release(struct file *file)
{
	struct histb_jpgd_ctx *ctx = fh_to_ctx(file->private_data);
	struct histb_jpgd_dev *jpgd = ctx->jpgd;

	mutex_lock(&jpgd->dev_mutex);
	v4l2_m2m_streamoff(file, ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	flush_work(&ctx->finish_work);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	mutex_unlock(&jpgd->dev_mutex);
	return 0;
}

static const struct v4l2_file_operations histb_jpgd_fops = {
	.owner = THIS_MODULE,
	.open = histb_jpgd_open,
	.release = histb_jpgd_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static const struct v4l2_m2m_ops histb_jpgd_m2m_ops = {
	.device_run = histb_jpgd_device_run,
	.job_abort = histb_jpgd_job_abort,
};

static const struct of_device_id histb_jpgd_of_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-jpgd" },
	{ .compatible = "hisilicon,hi3798mv310-jpgd" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, histb_jpgd_of_match);

static int histb_jpgd_power_on(struct histb_jpgd_dev *jpgd)
{
	int ret;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(jpgd->clocks), jpgd->clocks);
	if (ret)
		return ret;
	jpgd->clock_enabled = true;
	ret = reset_control_assert(jpgd->core_reset);
	if (ret)
		goto off;
	ret = reset_control_assert(jpgd->smmu_reset);
	if (ret)
		goto off;
	udelay(5);
	ret = reset_control_deassert(jpgd->smmu_reset);
	if (ret)
		goto off;
	ret = reset_control_deassert(jpgd->core_reset);
	if (ret)
		goto off;
	udelay(10);
	ret = histb_jpgd_reset_engine(jpgd);
	if (!ret) {
		/* Restore the physical-DMA baseline before permitting another job. */
		writel(0x1ff, jpgd->regs + HISTB_JPGD_REG_MMU_BYPASS);
		return 0;
	}
off:
	reset_control_assert(jpgd->core_reset);
	reset_control_assert(jpgd->smmu_reset);
	clk_bulk_disable_unprepare(ARRAY_SIZE(jpgd->clocks), jpgd->clocks);
	jpgd->clock_enabled = false;
	return ret;
}

static int histb_jpgd_runtime_resume(struct device *dev)
{
	struct histb_jpgd_dev *jpgd = dev_get_drvdata(dev);
	int ret;

	ret = histb_jpgd_power_on(jpgd);
	if (!ret)
		enable_irq(jpgd->irq);
	return ret;
}

static int histb_jpgd_runtime_suspend(struct device *dev)
{
	struct histb_jpgd_dev *jpgd = dev_get_drvdata(dev);
	int ret;

	if (!jpgd->clock_enabled)
		return 0;
	disable_irq(jpgd->irq);
	writel(0x7f, jpgd->regs + HISTB_JPGD_REG_INTMASK);
	writel(0x7f, jpgd->regs + HISTB_JPGD_REG_INT);
	ret = reset_control_assert(jpgd->core_reset);
	if (ret)
		return ret;
	ret = reset_control_assert(jpgd->smmu_reset);
	if (ret)
		return ret;
	clk_bulk_disable_unprepare(ARRAY_SIZE(jpgd->clocks), jpgd->clocks);
	jpgd->clock_enabled = false;
	return 0;
}

static int __maybe_unused histb_jpgd_suspend(struct device *dev)
{
	struct histb_jpgd_dev *jpgd = dev_get_drvdata(dev);
	int ret;

	/* The IRQ/watchdog/finish worker never needs dev_mutex. Keep contexts
	 * alive, but do not hold launch_lock while waiting for their completion.
	 */
	mutex_lock(&jpgd->dev_mutex);
	v4l2_m2m_suspend(jpgd->m2m_dev);
	synchronize_irq(jpgd->irq);
	cancel_delayed_work_sync(&jpgd->watchdog_work);
	flush_workqueue(jpgd->workqueue);
	ret = pm_runtime_force_suspend(dev);
	mutex_unlock(&jpgd->dev_mutex);
	return ret;
}

static int __maybe_unused histb_jpgd_resume(struct device *dev)
{
	struct histb_jpgd_dev *jpgd = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (!ret)
		v4l2_m2m_resume(jpgd->m2m_dev);
	return ret;
}

static const struct dev_pm_ops histb_jpgd_pm_ops = {
	SET_RUNTIME_PM_OPS(histb_jpgd_runtime_suspend,
			   histb_jpgd_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(histb_jpgd_suspend, histb_jpgd_resume)
};

/* The video device holds a v4l2_device reference until its last fd closes. */
static void histb_jpgd_device_release(struct v4l2_device *v4l2_dev)
{
	struct histb_jpgd_dev *jpgd = container_of(v4l2_dev,
						struct histb_jpgd_dev, v4l2_dev);

	v4l2_m2m_release(jpgd->m2m_dev);
	destroy_workqueue(jpgd->workqueue);
	put_device(jpgd->dev);
	kfree(jpgd);
}

static int histb_jpgd_probe(struct platform_device *pdev)
{
	struct histb_jpgd_dev *jpgd;
	struct video_device *vfd;
	struct resource *res;
	int ret;

	/* Every hardware channel bypasses the SMMU and needs physical DMA. */
	if (device_iommu_mapped(&pdev->dev))
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "JPGD requires physical DMA bypass\n");
	jpgd = kzalloc(sizeof(*jpgd), GFP_KERNEL);
	if (!jpgd)
		return -ENOMEM;
	jpgd->dev = get_device(&pdev->dev);
	mutex_init(&jpgd->dev_mutex);
	mutex_init(&jpgd->launch_lock);
	INIT_DELAYED_WORK(&jpgd->watchdog_work, histb_jpgd_watchdog);
	jpgd->workqueue = alloc_ordered_workqueue("histb-jpgd", WQ_MEM_RECLAIM);
	if (!jpgd->workqueue) {
		ret = -ENOMEM;
		goto err_free;
	}
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		goto err_free;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || resource_size(res) < HISTB_JPGD_REG_SIZE) {
		ret = -EINVAL;
		goto err_free;
	}
	jpgd->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(jpgd->regs)) {
		ret = PTR_ERR(jpgd->regs);
		goto err_free;
	}
	jpgd->irq = platform_get_irq(pdev, 0);
	if (jpgd->irq < 0) {
		ret = jpgd->irq;
		goto err_free;
	}
	jpgd->clocks[0].id = "core";
	jpgd->clocks[1].id = "smmu";
	ret = devm_clk_bulk_get(&pdev->dev, ARRAY_SIZE(jpgd->clocks), jpgd->clocks);
	if (ret)
		goto err_free;
	jpgd->core_reset = devm_reset_control_get_exclusive(&pdev->dev, "core");
	if (IS_ERR(jpgd->core_reset)) {
		ret = PTR_ERR(jpgd->core_reset);
		goto err_free;
	}
	jpgd->smmu_reset = devm_reset_control_get_exclusive(&pdev->dev, "smmu");
	if (IS_ERR(jpgd->smmu_reset)) {
		ret = PTR_ERR(jpgd->smmu_reset);
		goto err_free;
	}
	ret = histb_jpgd_power_on(jpgd);
	if (ret)
		goto err_free;
	ret = devm_request_threaded_irq(&pdev->dev, jpgd->irq, NULL,
				 histb_jpgd_irq, IRQF_ONESHOT, "histb-jpgd", jpgd);
	if (ret)
		goto err_clock;
	ret = v4l2_device_register(&pdev->dev, &jpgd->v4l2_dev);
	if (ret)
		goto err_irq;
	jpgd->m2m_dev = v4l2_m2m_init(&histb_jpgd_m2m_ops);
	if (IS_ERR(jpgd->m2m_dev)) {
		ret = PTR_ERR(jpgd->m2m_dev);
		goto err_v4l2;
	}
	jpgd->v4l2_dev.release = histb_jpgd_device_release;
	vfd = &jpgd->vfd;
	strscpy(vfd->name, "histb-jpgd", sizeof(vfd->name));
	vfd->fops = &histb_jpgd_fops;
	vfd->ioctl_ops = &histb_jpgd_ioctl_ops;
	vfd->minor = -1;
	vfd->release = video_device_release_empty;
	vfd->lock = &jpgd->dev_mutex;
	vfd->v4l2_dev = &jpgd->v4l2_dev;
	vfd->vfl_dir = VFL_DIR_M2M;
	vfd->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(vfd, jpgd);
	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;
	platform_set_drvdata(pdev, jpgd);
	/* Preserve binding-time power ownership; system sleep forces it off. */
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	return 0;

err_m2m:
	v4l2_m2m_release(jpgd->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&jpgd->v4l2_dev);
err_irq:
	devm_free_irq(&pdev->dev, jpgd->irq, jpgd);
err_clock:
	if (jpgd->clock_enabled) {
		reset_control_assert(jpgd->core_reset);
		reset_control_assert(jpgd->smmu_reset);
		clk_bulk_disable_unprepare(ARRAY_SIZE(jpgd->clocks), jpgd->clocks);
	}
err_free:
	if (jpgd->workqueue)
		destroy_workqueue(jpgd->workqueue);
	put_device(jpgd->dev);
	kfree(jpgd);
	return dev_err_probe(&pdev->dev, ret, "cannot initialize JPGD\n");
}

static void histb_jpgd_remove(struct platform_device *pdev)
{
	struct histb_jpgd_dev *jpgd = platform_get_drvdata(pdev);
	struct histb_jpgd_ctx *ctx;

	mutex_lock(&jpgd->dev_mutex);
	mutex_lock(&jpgd->launch_lock);
	jpgd->removing = true;
	ctx = jpgd->active;
	jpgd->active = NULL;
	if (jpgd->clock_enabled)
		histb_jpgd_reset_engine(jpgd);
	mutex_unlock(&jpgd->launch_lock);
	disable_irq(jpgd->irq);
	cancel_delayed_work_sync(&jpgd->watchdog_work);
	if (ctx)
		histb_jpgd_finish_job(ctx, VB2_BUF_STATE_ERROR);
	v4l2_m2m_suspend(jpgd->m2m_dev);
	flush_workqueue(jpgd->workqueue);
	v4l2_device_disconnect(&jpgd->v4l2_dev);
	video_unregister_device(&jpgd->vfd);
	v4l2_device_unregister(&jpgd->v4l2_dev);
	devm_free_irq(&pdev->dev, jpgd->irq, jpgd);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	if (jpgd->clock_enabled) {
		reset_control_assert(jpgd->core_reset);
		reset_control_assert(jpgd->smmu_reset);
		clk_bulk_disable_unprepare(ARRAY_SIZE(jpgd->clocks), jpgd->clocks);
	}
	mutex_unlock(&jpgd->dev_mutex);
	v4l2_device_put(&jpgd->v4l2_dev);
}

static struct platform_driver histb_jpgd_driver = {
	.probe = histb_jpgd_probe,
	.remove = histb_jpgd_remove,
	.driver = {
		.name = "histb-jpgd",
		.of_match_table = histb_jpgd_of_match,
		.pm = &histb_jpgd_pm_ops,
	},
};

module_platform_driver(histb_jpgd_driver);

MODULE_DESCRIPTION("HiSilicon HiSTB standalone JPEG decoder (JPGD)");
MODULE_AUTHOR("hiper25 <whydonit@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(DMA_BUF);
