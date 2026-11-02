// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hi3798MV310 PNG decoder.
 *
 * Register/stream contract: HiSTBLinuxV100R005C00SPC060-Hi3798MV310,
 * drv/png/{src/png_hal.c,src/png_osi.c,include/png_hal.h} and
 * api/png/png-hal/src_hard/png_accelerate.c (hipng_read_stream and
 * hipng_set_transform). This is not the CV200 decoder.
 *
 * One complete PNG file per OUTPUT buffer, fourcc 'PNG ', and one RGBA
 * CAPTURE buffer per image. Configure both queues to the IHDR dimensions
 * before allocating buffers. CAPTURE is V4L2_PIX_FMT_RGBA32: bytes R,G,B,A,
 * straight alpha, 16-byte aligned stride. The libpng BSP consumer maps
 * PNG_BGR/PNG_SWAP_ALPHA to the corresponding hardware bits; neither is
 * enabled here. No gamma, background compositing or premultiplication.
 *
 * Supported: noninterlaced, 8-bit gray, RGB, gray+alpha and RGBA, including
 * gray/RGB tRNS. Palette expansion needs a separate TDE engine in the BSP;
 * indexed, packed-gray, 16-bit and Adam7 images are explicitly rejected.
 * All chunks (including ancillary chunks and PLTE/tRNS) are bounds/CRC
 * checked before hardware sees any bytes. Only consecutive IDAT chunks,
 * including their length, type and CRC, enter the hardware stream.
 *
 * The hardware width/height fields are 13 bits, hence 1..8191 (just under
 * 64 Mi pixels), not the BSP libpng wrapper's separate <4096 policy.
 * Scratch: 32 KiB RDC dictionary, ALIGN(width * source_channels, 16)
 * filter row, and a 256 KiB stream segment; no full-frame scratch copy.
 * The 272 MiB input limit permits worst-case uncompressed RGBA at maximum
 * dimensions plus bounded PNG framing/metadata. All six MMU bypass bits
 * are set: this driver requires physically contiguous DMA32, no IOMMU.
 *
 * DT: hisilicon,hi3798mv310-png; core registers >=0x120 bytes;
 * IRQ SPI96; mandatory clocks/resets named "core" and "smmu". The SMMU
 * clock/reset must be present even though all engine channels bypass it.
 */
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#define PNG_FOURCC             v4l2_fourcc('P', 'N', 'G', ' ')
#define PNG_MAX_DIM            8191U
#define PNG_MAX_STREAM         (272U * SZ_1M)
#define PNG_SEGMENT_SIZE       SZ_256K
#define PNG_RDC_SIZE           SZ_32K
#define PNG_FILTER_SIZE        ALIGN(PNG_MAX_DIM * 4, 16)
#define PNG_TIMEOUT_MS         10000

#define PNG_START              0x000
#define PNG_RESUME             0x004
#define PNG_STRIDE             0x008
#define PNG_SIZE               0x00c
#define PNG_TYPE               0x010
#define PNG_TRANSFORM          0x014
#define PNG_BUFFER_START       0x020
#define PNG_BUFFER_END         0x024
#define PNG_STREAM_START       0x028
#define PNG_STREAM_END         0x02c
#define PNG_TARGET_START       0x04c
#define PNG_TARGET_END         0x050
#define PNG_TRANS_COLOR0       0x078
#define PNG_TRANS_COLOR1       0x07c
#define PNG_RDC_START          0x080
#define PNG_FILTER_START       0x088
#define PNG_FILTER_END         0x08c
#define PNG_AXI                0x090
#define PNG_TIMEOUT            0x094
#define PNG_BYPASS             0x098
#define PNG_INT                0x100
#define PNG_INT_MASK           0x104
#define PNG_ERROR_MODE         0x108
#define PNG_RESET_BUSY         0x110
#define PNG_INT_DONE            BIT(0)
#define PNG_INT_RESUME          BIT(1)
#define PNG_INT_ERROR          (BIT(2) | GENMASK(31, 16))
#define PNG_GRAY_TO_RGB         BIT(2)
#define PNG_ADD_ALPHA           BIT(3)
#define PNG_USE_TRNS            BIT(18)

#define PNG_CHUNK(a, b, c, d) (((u32)(a) << 24) | ((u32)(b) << 16) | \
                               ((u32)(c) << 8) | (d))
#define PNG_IHDR PNG_CHUNK('I', 'H', 'D', 'R')
#define PNG_IDAT PNG_CHUNK('I', 'D', 'A', 'T')
#define PNG_IEND PNG_CHUNK('I', 'E', 'N', 'D')
#define PNG_PLTE PNG_CHUNK('P', 'L', 'T', 'E')
#define PNG_TRNS PNG_CHUNK('t', 'R', 'N', 'S')

struct histb_png_image {
	u32 width;
	u32 height;
	u32 idat_offset;
	u32 idat_length;
	u8 depth;
	u8 color;
	u8 channels;
	bool interlace;
	bool transparent;
	u16 red;
	u16 green;
	u16 blue;
};

struct histb_png_dev {
	struct device *dev;
	void __iomem *regs;
	struct clk_bulk_data clocks[2];
	struct reset_control *core_reset;
	struct reset_control *smmu_reset;
	int irq;
	struct mutex mutex;
	struct v4l2_device v4l2;
	struct video_device video;
	struct v4l2_m2m_dev *m2m;
	struct workqueue_struct *workqueue;
	wait_queue_head_t wait;
	atomic_t status;
	bool removing;
	bool clock_enabled;
	void *segment;
	dma_addr_t segment_dma;
	void *dictionary;
	dma_addr_t dictionary_dma;
	void *filter;
	dma_addr_t filter_dma;
};

struct histb_png_ctx {
	struct v4l2_fh fh;
	struct histb_png_dev *png;
	struct v4l2_pix_format output;
	struct v4l2_pix_format capture;
	struct work_struct work;
	bool aborting;
	u32 sequence;
};

static struct histb_png_ctx *png_ctx(void *priv)
{
	return container_of(priv, struct histb_png_ctx, fh);
}

static bool png_chunk_letter(u8 c)
{
	return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

static int png_parse(const u8 *data, size_t size, struct histb_png_image *im)
{
	static const u8 signature[8] = { 137, 'P', 'N', 'G', 13, 10, 26, 10 };
	size_t pos = 8;
	u32 palette = 0;
	bool ihdr = false, idat = false, ended_idat = false, trns = false;
	bool iend = false;
	u64 compressed = 0;

	memset(im, 0, sizeof(*im));
	if (size < 8 || size > PNG_MAX_STREAM || memcmp(data, signature, 8))
		return -EINVAL;
	while (pos < size) {
		const u8 *chunk = data + pos;
		const u8 *p;
		u32 len, type, crc;
		unsigned int i;

		if (size - pos < 12)
			return -EINVAL;
		p = chunk + 8;
		len = get_unaligned_be32(chunk);
		type = get_unaligned_be32(chunk + 4);
		if (len > INT_MAX || len > size - pos - 12)
			return -EINVAL;
		for (i = 4; i < 8; i++)
			if (!png_chunk_letter(chunk[i]))
				return -EINVAL;
		if (chunk[6] & 0x20) /* PNG reserved bit must be zero. */
			return -EINVAL;
		crc = crc32_le(~0U, chunk + 4, len + 4) ^ ~0U;
		if (crc != get_unaligned_be32(p + len))
			return -EBADMSG;
		if (!ihdr && type != PNG_IHDR)
			return -EINVAL;
		if (idat && type != PNG_IDAT)
			ended_idat = true;

		switch (type) {
		case PNG_IHDR:
			if (ihdr || len != 13)
				return -EINVAL;
			im->width = get_unaligned_be32(p);
			im->height = get_unaligned_be32(p + 4);
			im->depth = p[8];
			im->color = p[9];
			if (!im->width || im->width > PNG_MAX_DIM ||
			    !im->height || im->height > PNG_MAX_DIM ||
			    p[10] || p[11] || p[12] > 1)
				return -EINVAL;
			im->interlace = p[12];
			switch (im->color) {
			case 0:
				im->channels = 1;
				if (im->depth != 1 && im->depth != 2 &&
				    im->depth != 4 && im->depth != 8 && im->depth != 16)
					return -EINVAL;
				break;
			case 3:
				im->channels = 1;
				if (im->depth != 1 && im->depth != 2 &&
				    im->depth != 4 && im->depth != 8)
					return -EINVAL;
				break;
			case 2:
				im->channels = 3;
				break;
			case 4:
				im->channels = 2;
				break;
			case 6:
				im->channels = 4;
				break;
			default:
				return -EINVAL;
			}
			if (im->color != 0 && im->color != 3 &&
			    im->depth != 8 && im->depth != 16)
				return -EINVAL;
			ihdr = true;
			break;
		case PNG_PLTE:
			if (palette || idat || trns || !len || len % 3 || len > 768 ||
			    im->color == 0 || im->color == 4)
				return -EINVAL;
			palette = len / 3;
			if (im->color == 3 && palette > (1U << im->depth))
				return -EINVAL;
			break;
		case PNG_TRNS:
			if (trns || idat)
				return -EINVAL;
			if (im->color == 3) {
				if (!palette || !len || len > palette)
					return -EINVAL;
			} else if (im->color == 0) {
				if (len != 2)
					return -EINVAL;
				im->red = im->green = im->blue = get_unaligned_be16(p);
				if (im->blue >= (1U << im->depth))
					return -EINVAL;
			} else if (im->color == 2) {
				if (len != 6)
					return -EINVAL;
				im->red = get_unaligned_be16(p);
				im->green = get_unaligned_be16(p + 2);
				im->blue = get_unaligned_be16(p + 4);
				if (im->red >= (1U << im->depth) ||
				    im->green >= (1U << im->depth) ||
				    im->blue >= (1U << im->depth))
					return -EINVAL;
			} else {
				return -EINVAL;
			}
			trns = true;
			im->transparent = true;
			break;
		case PNG_IDAT:
			if (ended_idat || (im->color == 3 && !palette))
				return -EINVAL;
			if (!idat)
				im->idat_offset = pos;
			idat = true;
			compressed += len;
			im->idat_length += len + 12;
			break;
		case PNG_IEND:
			if (len || !idat || !compressed || pos + 12 != size)
				return -EINVAL;
			iend = true;
			break;
		case PNG_CHUNK('a', 'c', 'T', 'L'):
		case PNG_CHUNK('f', 'c', 'T', 'L'):
		case PNG_CHUNK('f', 'd', 'A', 'T'):
			return -EOPNOTSUPP; /* Do not silently decode only one APNG frame. */
		default:
			if (!(chunk[4] & 0x20))
				return -EOPNOTSUPP;
			break;
		}
		pos += len + 12;
	}
	if (!iend)
		return -EINVAL;
	if (im->depth != 8 || im->color == 3 || im->interlace)
		return -EOPNOTSUPP;
	return 0;
}

static irqreturn_t png_irq(int irq, void *priv)
{
	struct histb_png_dev *png = priv;
	u32 status = readl(png->regs + PNG_INT);

	if (!status)
		return IRQ_NONE;
	writel(status, png->regs + PNG_INT);
	atomic_or(status, &png->status);
	wake_up(&png->wait);
	return IRQ_HANDLED;
}

/* Segment boundaries may split chunk headers, data or CRC (BSP contract). */
static void png_feed(struct histb_png_dev *png, const u8 *data, u32 length)
{
	memcpy(png->segment, data, length);
	if (length < PNG_SEGMENT_SIZE)
		memset(png->segment + length, 0, ALIGN(length, 16) - length);
	dma_wmb();
	writel(lower_32_bits(png->segment_dma), png->regs + PNG_BUFFER_START);
	writel(lower_32_bits(png->segment_dma) + PNG_SEGMENT_SIZE - 1,
	       png->regs + PNG_BUFFER_END);
	writel(lower_32_bits(png->segment_dma), png->regs + PNG_STREAM_START);
	writel(lower_32_bits(png->segment_dma) + length - 1,
	       png->regs + PNG_STREAM_END);
}

static bool png_cancelled(struct histb_png_ctx *ctx)
{
	return READ_ONCE(ctx->aborting) || READ_ONCE(ctx->png->removing);
}

static int png_decode(struct histb_png_ctx *ctx, const u8 *data,
		      const struct histb_png_image *im, struct vb2_v4l2_buffer *dst)
{
	struct histb_png_dev *png = ctx->png;
	dma_addr_t target = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	u32 transform = 0, row = ALIGN(im->width * im->channels, 16);
	u32 offset = im->idat_offset, remaining = im->idat_length;
	unsigned long deadline = jiffies + msecs_to_jiffies(PNG_TIMEOUT_MS);
	u32 length, status;
	int ret, reset_ret;

	if (!target || !IS_ALIGNED(target, 16) ||
	    target > U32_MAX || ctx->capture.sizeimage - 1 > U32_MAX - target)
		return -EINVAL;
	if (png_cancelled(ctx))
		return -ECANCELED;
	if (!png->clock_enabled)
		return -EIO;
	ret = reset_control_deassert(png->core_reset);
	if (ret)
		return ret;
	/* Registers and IRQ state were quiesced before the preceding job finished. */
	atomic_set(&png->status, 0);
	writel(~0U, png->regs + PNG_INT);
	writel(0x20441, png->regs + PNG_AXI);
	writel(0x80008, png->regs + PNG_TIMEOUT);
	writel(0xffff0000, png->regs + PNG_ERROR_MODE);
	writel(0x3f, png->regs + PNG_BYPASS);
	writel(im->width << 16 | im->height, png->regs + PNG_SIZE);
	writel(im->color << 8 | im->depth, png->regs + PNG_TYPE);
	if (im->color == 0 || im->color == 4)
		transform |= PNG_GRAY_TO_RGB;
	if (im->color == 0 || im->color == 2) {
		transform |= PNG_ADD_ALPHA;
		if (im->transparent)
			transform |= PNG_USE_TRNS;
	}
	writel(transform, png->regs + PNG_TRANSFORM);
	writel(im->red << 16 | im->green, png->regs + PNG_TRANS_COLOR0);
	writel(0xffff0000 | im->blue, png->regs + PNG_TRANS_COLOR1);
	writel(lower_32_bits(png->dictionary_dma), png->regs + PNG_RDC_START);
	writel(lower_32_bits(png->filter_dma), png->regs + PNG_FILTER_START);
	writel(lower_32_bits(png->filter_dma) + row - 1, png->regs + PNG_FILTER_END);
	writel(lower_32_bits(target), png->regs + PNG_TARGET_START);
	writel(lower_32_bits(target) + ctx->capture.sizeimage - 1,
	       png->regs + PNG_TARGET_END);
	writel(ctx->capture.bytesperline, png->regs + PNG_STRIDE);
	length = min(remaining, (u32)PNG_SEGMENT_SIZE);
	png_feed(png, data + offset, length);
	offset += length;
	remaining -= length;
	writel(~0U, png->regs + PNG_INT_MASK);
	writel(1, png->regs + PNG_START);

	for (;;) {
		long timeout;

		if (png_cancelled(ctx)) {
			ret = -ECANCELED;
			break;
		}
		timeout = deadline - jiffies;
		if (timeout <= 0) {
			ret = -ETIMEDOUT;
			break;
		}
		timeout = wait_event_timeout(png->wait,
			atomic_read(&png->status) || png_cancelled(ctx), timeout);
		if (!timeout) {
			ret = -ETIMEDOUT;
			break;
		}
		if (png_cancelled(ctx)) {
			ret = -ECANCELED;
			break;
		}
		status = atomic_xchg(&png->status, 0);
		if (status & PNG_INT_ERROR) {
			dev_dbg(png->dev, "decode error interrupt %#x\n", status);
			ret = -EBADMSG;
			break;
		}
		if (status & PNG_INT_DONE) {
			/* Hardware must not finish before all submitted IDAT bytes. */
			ret = remaining ? -EBADMSG : 0;
			break;
		}
		if (status & PNG_INT_RESUME) {
			/*
			 * Exhausting the final segment raises RESUME before the
			 * output pipeline drains and raises DONE. Do not reset
			 * or replay input: await DONE/ERROR under the same deadline.
			 */
			if (!remaining)
				continue;
			length = min(remaining, (u32)PNG_SEGMENT_SIZE);
			png_feed(png, data + offset, length);
			offset += length;
			remaining -= length;
			writel(1, png->regs + PNG_RESUME);
		}
	}
	/* Stop DMA before returning buffers, including error/abort/timeout paths. */
	writel(0, png->regs + PNG_INT_MASK);
	reset_ret = reset_control_assert(png->core_reset);
	if (!reset_ret)
		reset_ret = readl_poll_timeout(png->regs + PNG_RESET_BUSY, status,
					     !(status & BIT(0)), 10, 20000);
	if (reset_ret) {
		dev_err(png->dev, "PNG reset failed: %d; engine stopped\n", reset_ret);
		/* Join the hard IRQ before removing its register clock. An engine
		 * with an unconfirmed reset must not DMA into returned buffers.
		 */
		disable_irq(png->irq);
		clk_bulk_disable_unprepare(ARRAY_SIZE(png->clocks), png->clocks);
		png->clock_enabled = false;
		ret = reset_ret;
	} else {
		synchronize_irq(png->irq);
	}
	return ret;
}

static void png_work(struct work_struct *work)
{
	struct histb_png_ctx *ctx = container_of(work, struct histb_png_ctx, work);
	struct histb_png_dev *png = ctx->png;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	struct histb_png_image image;
	struct dma_buf *dmabuf = src->vb2_buf.planes[0].dbuf;
	const u8 *data;
	bool cpu_access = false;
	int ret = -ECANCELED;

	if (png_cancelled(ctx))
		goto done;
	if (dmabuf) {
		ret = dma_buf_begin_cpu_access(dmabuf, DMA_FROM_DEVICE);
		if (ret)
			goto done;
		cpu_access = true;
	}
	data = vb2_plane_vaddr(&src->vb2_buf, 0);
	if (!data) {
		ret = -EFAULT;
		goto done;
	}
	ret = png_parse(data, vb2_get_plane_payload(&src->vb2_buf, 0), &image);
	if (ret)
		goto done;
	if (image.width != ctx->capture.width || image.height != ctx->capture.height ||
	    image.width != ctx->output.width || image.height != ctx->output.height) {
		ret = -EINVAL;
		goto done;
	}
	ret = png_decode(ctx, data, &image, dst);

done:
	if (cpu_access) {
		int end_ret = dma_buf_end_cpu_access(dmabuf, DMA_FROM_DEVICE);

		if (!ret)
			ret = end_ret;
	}
	dst->vb2_buf.timestamp = src->vb2_buf.timestamp;
	dst->timecode = src->timecode;
	dst->flags &= ~V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
	dst->flags |= src->flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
	src->sequence = dst->sequence = ctx->sequence++;
	dst->field = V4L2_FIELD_NONE;
	vb2_set_plane_payload(&dst->vb2_buf, 0, ret ? 0 : ctx->capture.sizeimage);
	if (ret && ret != -ECANCELED)
		dev_dbg(png->dev, "PNG job failed: %d\n", ret);
	/* release joins this worker before freeing its M2M context. */
	v4l2_m2m_buf_done_and_job_finish(png->m2m, ctx->fh.m2m_ctx,
				      ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
}

static void png_device_run(void *priv)
{
	struct histb_png_ctx *ctx = priv;

	queue_work(ctx->png->workqueue, &ctx->work);
}

static void png_job_abort(void *priv)
{
	struct histb_png_ctx *ctx = priv;

	WRITE_ONCE(ctx->aborting, true);
	wake_up(&ctx->png->wait);
}

static int png_queue_setup(struct vb2_queue *q, unsigned int *count,
			   unsigned int *planes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct histb_png_ctx *ctx = vb2_get_drv_priv(q);
	u32 size = V4L2_TYPE_IS_OUTPUT(q->type) ? ctx->output.sizeimage :
		ctx->capture.sizeimage;

	if (*planes)
		return *planes == 1 && sizes[0] >= size ? 0 : -EINVAL;
	*planes = 1;
	sizes[0] = size;
	return 0;
}

static int png_buffer_prepare(struct vb2_buffer *vb)
{
	struct histb_png_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	bool output = V4L2_TYPE_IS_OUTPUT(vb->type);
	u32 size = output ? ctx->output.sizeimage : ctx->capture.sizeimage;

	if (vb2_plane_size(vb, 0) < size || vb->planes[0].data_offset)
		return -EINVAL;
	if (output) {
		if (!vb2_get_plane_payload(vb, 0) ||
		    vb2_get_plane_payload(vb, 0) > size)
			return -EINVAL;
	} else {
		vb2_set_plane_payload(vb, 0, 0);
	}
	return 0;
}

static void png_buffer_queue(struct vb2_buffer *vb)
{
	struct histb_png_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int png_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct histb_png_ctx *ctx = vb2_get_drv_priv(q);

	/* A previous STREAMOFF cannot leave an abort latched for the next job. */
	WRITE_ONCE(ctx->aborting, false);
	return 0;
}

static void png_stop_streaming(struct vb2_queue *q)
{
	struct histb_png_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *buf;

	/* The M2M cancel hook already waited for worker/DMA completion. */
	for (;;) {
		buf = V4L2_TYPE_IS_OUTPUT(q->type) ?
			v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
			v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!buf)
			break;
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops png_queue_ops = {
	.queue_setup = png_queue_setup,
	.buf_prepare = png_buffer_prepare,
	.buf_queue = png_buffer_queue,
	.start_streaming = png_start_streaming,
	.stop_streaming = png_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int png_queue_init(void *priv, struct vb2_queue *src, struct vb2_queue *dst)
{
	struct histb_png_ctx *ctx = priv;
	struct vb2_queue *queues[] = { src, dst };
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(queues); i++) {
		struct vb2_queue *q = queues[i];

		q->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE : V4L2_BUF_TYPE_VIDEO_OUTPUT;
		q->io_modes = VB2_MMAP | VB2_DMABUF;
		q->drv_priv = ctx;
		q->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
		q->ops = &png_queue_ops;
		q->mem_ops = &vb2_dma_contig_memops;
		q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		q->lock = &ctx->png->mutex;
		q->dev = ctx->png->dev;
		q->gfp_flags = GFP_DMA32;
		ret = vb2_queue_init(q);
		if (ret)
			return ret;
	}
	return 0;
}

static int png_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, "histb-png", sizeof(cap->driver));
	strscpy(cap->card, "Hi3798MV310 PNG decoder", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(png_ctx(priv)->png->dev));
	return 0;
}

static int png_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		f->pixelformat = PNG_FOURCC;
		f->flags = V4L2_FMT_FLAG_COMPRESSED;
		strscpy(f->description, "PNG (8-bit, noninterlaced)", sizeof(f->description));
	} else {
		f->pixelformat = V4L2_PIX_FMT_RGBA32;
	}
	return 0;
}

static int png_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width = clamp_t(u32, pix->width, 1, PNG_MAX_DIM);
	pix->height = clamp_t(u32, pix->height, 1, PNG_MAX_DIM);
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->xfer_func = V4L2_XFER_FUNC_SRGB;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->flags = 0;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		pix->pixelformat = PNG_FOURCC;
		pix->bytesperline = 0;
		pix->sizeimage = clamp_t(u32, pix->sizeimage, 67, PNG_MAX_STREAM);
	} else {
		pix->pixelformat = V4L2_PIX_FMT_RGBA32;
		pix->bytesperline = ALIGN(pix->width * 4, 16);
		pix->sizeimage = pix->bytesperline * pix->height;
	}
	return 0;
}

static int png_get_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct histb_png_ctx *ctx = png_ctx(priv);

	f->fmt.pix = V4L2_TYPE_IS_OUTPUT(f->type) ? ctx->output : ctx->capture;
	return 0;
}

static int png_set_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct histb_png_ctx *ctx = png_ctx(priv);
	int ret;

	if (vb2_is_busy(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type)))
		return -EBUSY;
	ret = png_try_fmt(file, priv, f);
	if (ret)
		return ret;
	if (V4L2_TYPE_IS_OUTPUT(f->type))
		ctx->output = f->fmt.pix;
	else
		ctx->capture = f->fmt.pix;
	return 0;
}

static int png_enum_framesizes(struct file *file, void *priv,
			       struct v4l2_frmsizeenum *f)
{
	if (f->index || (f->pixel_format != PNG_FOURCC &&
			 f->pixel_format != V4L2_PIX_FMT_RGBA32))
		return -EINVAL;
	f->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	f->stepwise.min_width = f->stepwise.min_height = 1;
	f->stepwise.max_width = f->stepwise.max_height = PNG_MAX_DIM;
	f->stepwise.step_width = f->stepwise.step_height = 1;
	return 0;
}

static const struct v4l2_ioctl_ops png_ioctl_ops = {
	.vidioc_querycap = png_querycap,
	.vidioc_enum_fmt_vid_cap = png_enum_fmt,
	.vidioc_enum_fmt_vid_out = png_enum_fmt,
	.vidioc_enum_framesizes = png_enum_framesizes,
	.vidioc_g_fmt_vid_cap = png_get_fmt,
	.vidioc_g_fmt_vid_out = png_get_fmt,
	.vidioc_try_fmt_vid_cap = png_try_fmt,
	.vidioc_try_fmt_vid_out = png_try_fmt,
	.vidioc_s_fmt_vid_cap = png_set_fmt,
	.vidioc_s_fmt_vid_out = png_set_fmt,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

static int png_open(struct file *file)
{
	struct histb_png_dev *png = video_drvdata(file);
	struct histb_png_ctx *ctx;
	struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT };
	int ret;

	if (mutex_lock_interruptible(&png->mutex))
		return -ERESTARTSYS;
	if (READ_ONCE(png->removing)) {
		ret = -ENODEV;
		goto unlock;
	}
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto unlock;
	}
	ctx->png = png;
	INIT_WORK(&ctx->work, png_work);
	v4l2_fh_init(&ctx->fh, &png->video);
	file->private_data = &ctx->fh;
	fmt.fmt.pix.width = 640;
	fmt.fmt.pix.height = 480;
	fmt.fmt.pix.sizeimage = SZ_1M;
	png_try_fmt(file, &ctx->fh, &fmt);
	ctx->output = fmt.fmt.pix;
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	png_try_fmt(file, &ctx->fh, &fmt);
	ctx->capture = fmt.fmt.pix;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(png->m2m, ctx, png_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		goto unlock;
	}
	v4l2_fh_add(&ctx->fh);
	ret = 0;
unlock:
	mutex_unlock(&png->mutex);
	return ret;
}

static int png_release(struct file *file)
{
	struct histb_png_ctx *ctx = png_ctx(file->private_data);
	struct histb_png_dev *png = ctx->png;

	mutex_lock(&png->mutex);
	/* Cancel scheduling while the M2M context still exists, then join the
	 * completion helper's tail before releasing that context.
	 */
	v4l2_m2m_streamoff(file, ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	flush_work(&ctx->work);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	mutex_unlock(&png->mutex);
	return 0;
}

static const struct v4l2_file_operations png_fops = {
	.owner = THIS_MODULE,
	.open = png_open,
	.release = png_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = v4l2_m2m_fop_poll,
	.mmap = v4l2_m2m_fop_mmap,
};

static const struct v4l2_m2m_ops png_m2m_ops = {
	.device_run = png_device_run,
	.job_abort = png_job_abort,
};

static void png_device_release(struct v4l2_device *v4l2)
{
	struct histb_png_dev *png = container_of(v4l2, struct histb_png_dev, v4l2);

	v4l2_m2m_release(png->m2m);
	destroy_workqueue(png->workqueue);
	put_device(png->dev);
	kfree(png);
}

static int png_power_on(struct histb_png_dev *png)
{
	int ret;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(png->clocks), png->clocks);
	if (ret)
		return ret;
	png->clock_enabled = true;
	ret = reset_control_assert(png->core_reset);
	if (ret)
		goto off;
	ret = reset_control_assert(png->smmu_reset);
	if (ret)
		goto off;
	udelay(2);
	ret = reset_control_deassert(png->smmu_reset);
	if (!ret)
		/* Idle baseline: core remains reset. Each job restores the BSP
		 * AXI/timeout/bypass and complete image state before PNG_START.
		 */
		return 0;
off:
	reset_control_assert(png->core_reset);
	reset_control_assert(png->smmu_reset);
	clk_bulk_disable_unprepare(ARRAY_SIZE(png->clocks), png->clocks);
	png->clock_enabled = false;
	return ret;
}

static int png_runtime_resume(struct device *dev)
{
	struct histb_png_dev *png = dev_get_drvdata(dev);
	int ret;

	ret = png_power_on(png);
	if (!ret)
		enable_irq(png->irq);
	return ret;
}

static int png_runtime_suspend(struct device *dev)
{
	struct histb_png_dev *png = dev_get_drvdata(dev);
	u32 status;
	int ret;

	if (!png->clock_enabled)
		return 0;
	disable_irq(png->irq);
	writel(0, png->regs + PNG_INT_MASK);
	ret = reset_control_assert(png->core_reset);
	if (ret)
		return ret;
	ret = readl_poll_timeout(png->regs + PNG_RESET_BUSY, status,
				 !(status & BIT(0)), 10, 20000);
	if (ret)
		return ret;
	ret = reset_control_assert(png->smmu_reset);
	if (ret)
		return ret;
	clk_bulk_disable_unprepare(ARRAY_SIZE(png->clocks), png->clocks);
	png->clock_enabled = false;
	return 0;
}

static int __maybe_unused png_suspend(struct device *dev)
{
	struct histb_png_dev *png = dev_get_drvdata(dev);
	int ret;

	/* Workers and IRQ completion do not take the ioctl mutex. Drain an
	 * image (or its explicit timeout/error), not just the M2M scheduling bit.
	 */
	mutex_lock(&png->mutex);
	v4l2_m2m_suspend(png->m2m);
	flush_workqueue(png->workqueue);
	synchronize_irq(png->irq);
	ret = pm_runtime_force_suspend(dev);
	mutex_unlock(&png->mutex);
	return ret;
}

static int __maybe_unused png_resume(struct device *dev)
{
	struct histb_png_dev *png = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (!ret)
		v4l2_m2m_resume(png->m2m);
	return ret;
}

static const struct dev_pm_ops png_pm_ops = {
	SET_RUNTIME_PM_OPS(png_runtime_suspend, png_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(png_suspend, png_resume)
};

static int histb_png_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_png_dev *png;
	struct resource *res;
	int ret;

	/* Hardware bypass addresses are physical; translated DMA is not usable. */
	if (device_iommu_mapped(dev))
		return dev_err_probe(dev, -EINVAL, "PNG requires DMA32 physical bypass\n");
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	png = kzalloc(sizeof(*png), GFP_KERNEL);
	if (!png)
		return -ENOMEM;
	/* Queue DMA allocations can outlive unbind while video fds remain open. */
	png->dev = get_device(dev);
	mutex_init(&png->mutex);
	init_waitqueue_head(&png->wait);
	atomic_set(&png->status, 0);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || resource_size(res) < 0x120) {
		ret = -EINVAL;
		goto free_png;
	}
	png->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(png->regs)) {
		ret = PTR_ERR(png->regs);
		goto free_png;
	}
	png->clocks[0].id = "smmu";
	png->clocks[1].id = "core";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(png->clocks), png->clocks);
	if (ret)
		goto free_png;
	png->core_reset = devm_reset_control_get_exclusive(dev, "core");
	if (IS_ERR(png->core_reset)) {
		ret = PTR_ERR(png->core_reset);
		goto free_png;
	}
	png->smmu_reset = devm_reset_control_get_exclusive(dev, "smmu");
	if (IS_ERR(png->smmu_reset)) {
		ret = PTR_ERR(png->smmu_reset);
		goto free_png;
	}
	ret = png_power_on(png);
	if (ret)
		goto free_png;
	png->irq = platform_get_irq(pdev, 0);
	if (png->irq < 0) {
		ret = png->irq;
		goto clocks_off;
	}
	png->segment = dmam_alloc_coherent(dev, PNG_SEGMENT_SIZE, &png->segment_dma,
					 GFP_KERNEL | GFP_DMA32);
	png->dictionary = dmam_alloc_coherent(dev, PNG_RDC_SIZE, &png->dictionary_dma,
					    GFP_KERNEL | GFP_DMA32);
	png->filter = dmam_alloc_coherent(dev, PNG_FILTER_SIZE, &png->filter_dma,
					GFP_KERNEL | GFP_DMA32);
	if (!png->segment || !png->dictionary || !png->filter) {
		ret = -ENOMEM;
		goto clocks_off;
	}
	ret = devm_request_irq(dev, png->irq, png_irq, 0, dev_name(dev), png);
	if (ret)
		goto clocks_off;
	png->workqueue = alloc_ordered_workqueue("histb-png", WQ_MEM_RECLAIM);
	if (!png->workqueue) {
		ret = -ENOMEM;
		goto free_irq;
	}
	ret = v4l2_device_register(dev, &png->v4l2);
	if (ret)
		goto destroy_queue;
	png->m2m = v4l2_m2m_init(&png_m2m_ops);
	if (IS_ERR(png->m2m)) {
		ret = PTR_ERR(png->m2m);
		goto unregister_v4l2;
	}
	png->v4l2.release = png_device_release;
	strscpy(png->video.name, "histb-png", sizeof(png->video.name));
	png->video.fops = &png_fops;
	png->video.ioctl_ops = &png_ioctl_ops;
	png->video.lock = &png->mutex;
	png->video.v4l2_dev = &png->v4l2;
	png->video.vfl_dir = VFL_DIR_M2M;
	png->video.release = video_device_release_empty;
	png->video.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(&png->video, png);
	/* Video core holds v4l2 while registered or file handles remain open. */
	ret = video_register_device(&png->video, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_m2m_release(png->m2m);
		goto unregister_v4l2;
	}
	platform_set_drvdata(pdev, png);
	/* Preserve binding-time power ownership; system sleep forces it off. */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	dev_info(dev, "MV310 PNG hardware decoder registered as /dev/video%d\n",
		 png->video.num);
	return 0;

unregister_v4l2:
	v4l2_device_unregister(&png->v4l2);
destroy_queue:
	destroy_workqueue(png->workqueue);
free_irq:
	devm_free_irq(dev, png->irq, png);
clocks_off:
	if (png->clock_enabled) {
		reset_control_assert(png->core_reset);
		reset_control_assert(png->smmu_reset);
		clk_bulk_disable_unprepare(ARRAY_SIZE(png->clocks), png->clocks);
	}
free_png:
	put_device(png->dev);
	kfree(png);
	return dev_err_probe(dev, ret, "failed to initialize PNG decoder\n");
}

static void histb_png_remove(struct platform_device *pdev)
{
	struct histb_png_dev *png = platform_get_drvdata(pdev);

	mutex_lock(&png->mutex);
	WRITE_ONCE(png->removing, true);
	v4l2_device_disconnect(&png->v4l2);
	wake_up(&png->wait);
	v4l2_m2m_suspend(png->m2m);
	flush_workqueue(png->workqueue);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	if (png->clock_enabled) {
		reset_control_assert(png->core_reset);
		reset_control_assert(png->smmu_reset);
	}
	devm_free_irq(png->dev, png->irq, png);
	if (png->clock_enabled)
		clk_bulk_disable_unprepare(ARRAY_SIZE(png->clocks), png->clocks);
	mutex_unlock(&png->mutex);
	video_unregister_device(&png->video);
	v4l2_device_unregister(&png->v4l2);
	v4l2_device_put(&png->v4l2);
}

static const struct of_device_id histb_png_of_match[] = {
	{ .compatible = "hisilicon,hi3798mv310-png" },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_png_of_match);

static struct platform_driver histb_png_driver = {
	.probe = histb_png_probe,
	.remove = histb_png_remove,
	.driver = {
		.name = "histb-png",
		.of_match_table = histb_png_of_match,
		.pm = &png_pm_ops,
	},
};
module_platform_driver(histb_png_driver);

MODULE_DESCRIPTION("HiSilicon Hi3798MV310 PNG hardware decoder");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(DMA_BUF);
