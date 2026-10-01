// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3798CV200 video post-processing subsystem
 *
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>

#include "histb-vpss.h"

#define HISTB_VPSS_CTRL			0x000
#define HISTB_VPSS_CTRL2		0x004
#define HISTB_VPSS_CTRL3		0x008
#define HISTB_VPSS_IMG_SIZE		0x018
#define HISTB_VPSS_ZME_OUTPUT_SIZE	0x02c
#define HISTB_VPSS_LB_Y_ADDR		0x094
#define HISTB_VPSS_LB_C_ADDR		0x098
#define HISTB_VPSS_LB_STRIDE		0x09c
#define HISTB_VPSS_INT_MASK		0x0fc
#define HISTB_VPSS_INPUT_CTRL		0x130
#define HISTB_VPSS_INPUT_Y_ADDR		0x134
#define HISTB_VPSS_INPUT_C_ADDR		0x138
#define HISTB_VPSS_INPUT_STRIDE		0x13c
#define HISTB_VPSS_OUTPUT_CTRL		0x150
#define HISTB_VPSS_OUTPUT_SIZE		0x154
#define HISTB_VPSS_OUTPUT_Y_ADDR	0x158
#define HISTB_VPSS_OUTPUT_C_ADDR	0x15c
#define HISTB_VPSS_OUTPUT_STRIDE	0x160
#define HISTB_VPSS_ZME_LH_COEF_ADDR	0x200
#define HISTB_VPSS_ZME_LV_COEF_ADDR	0x204
#define HISTB_VPSS_ZME_CH_COEF_ADDR	0x208
#define HISTB_VPSS_ZME_CV_COEF_ADDR	0x20c
#define HISTB_VPSS_RCH_BYPASS		0x280
#define HISTB_VPSS_WCH_BYPASS		0x284
/* De-interlacer field registers, in groups of four: control, Y, C, stride. */
#define HISTB_VPSS_DEI_CUR_CTRL		0x100
#define HISTB_VPSS_DEI_CURYADDR		0x104
#define HISTB_VPSS_DEI_CURCADDR		0x108
#define HISTB_VPSS_DEI_CURSTRIDE	0x10c
#define HISTB_VPSS_DEI_REF_CTRL		0x110
#define HISTB_VPSS_DEI_REFYADDR		0x114
#define HISTB_VPSS_DEI_REFCADDR		0x118
#define HISTB_VPSS_DEI_REFSTRIDE	0x11c
#define HISTB_VPSS_DEI_NXT1_CTRL	0x120
#define HISTB_VPSS_DEI_NXT1YADDR	0x124
#define HISTB_VPSS_DEI_NXT1CADDR	0x128
#define HISTB_VPSS_DEI_NXT1STRIDE	0x12c
#define HISTB_VPSS_DEI_NXT2_CTRL	0x130
#define HISTB_VPSS_DEI_NXT2YADDR	0x134
#define HISTB_VPSS_DEI_NXT2CADDR	0x138
#define HISTB_VPSS_DEI_NXT2STRIDE	0x13c
#define HISTB_VPSS_DEI_ADDR		0x258
#define HISTB_VPSS_DIECTRL		0x1000

/* VPSS_CTRL */
#define HISTB_VPSS_CTRL_DEI_EN		BIT(7)
#define HISTB_VPSS_CTRL_MCDI_EN		BIT(8)
#define HISTB_VPSS_CTRL_MEDS_EN		BIT(9)
#define HISTB_VPSS_CTRL_IFMD_EN		BIT(25)
#define HISTB_VPSS_CTRL_BFIELD_FIRST	BIT(29)
#define HISTB_VPSS_CTRL_BFIELD_MODE	BIT(30)

/* VPSS_DIECTRL */
#define HISTB_VPSS_DIE_EDGE_SMOOTH_EN	BIT(20)
#define HISTB_VPSS_DIE_L_MODE		GENMASK(27, 26)
#define HISTB_VPSS_DIE_C_MODE		GENMASK(25, 24)

/* Field control: bit for "this field is in the decoder's tile format". */
#define HISTB_VPSS_DEI_TILE_FORMAT	BIT(0)

/* The BSP bypasses the de-interlacer above these (vpss_in_3798cv200.c). */
#define HISTB_VPSS_DEI_MAX_WIDTH	1920
#define HISTB_VPSS_DEI_MAX_HEIGHT	1088

#define HISTB_VPSS_ZME_ADDR		0x240
#define HISTB_VPSS_NEXT			0x2fc
#define HISTB_VPSS_START		0x300
#define HISTB_VPSS_INT_STATE		0x304
#define HISTB_VPSS_INT_CLEAR		0x308
#define HISTB_VPSS_MISC			0x314
#define HISTB_VPSS_TIMEOUT		0x31c

#define HISTB_VPSS_ZME_HSP		0x2000
#define HISTB_VPSS_ZME_HL_OFFSET	0x2004
#define HISTB_VPSS_ZME_HC_OFFSET	0x2008
#define HISTB_VPSS_ZME_VSP		0x200c
#define HISTB_VPSS_ZME_VSR		0x2010
#define HISTB_VPSS_ZME_V_OFFSET		0x2014

#define HISTB_VPSS_CTRL_OUTPUT_EN	BIT(3)
#define HISTB_VPSS_CTRL_FOUR_PIX	BIT(20)
#define HISTB_VPSS_CTRL3_ZME_EN		BIT(3)
#define HISTB_VPSS_CTRL2_INPUT_10BIT	BIT(21)
#define HISTB_VPSS_INPUT_TILE		BIT(4)
#define HISTB_VPSS_OUTPUT_10BIT		BIT(6)
#define HISTB_VPSS_OUTPUT_DITHER	BIT(7)
#define HISTB_VPSS_OUTPUT_UV_INVERT	BIT(8)
#define HISTB_VPSS_OUTPUT_DITHER_ROUND	BIT(10)

#define HISTB_VPSS_ZME_H_RATIO_PRECISION	BIT(20)
#define HISTB_VPSS_ZME_V_RATIO_PRECISION	BIT(12)
#define HISTB_VPSS_ZME_H_LUMA_EN		BIT(31)
#define HISTB_VPSS_ZME_H_CHROMA_EN	BIT(30)
#define HISTB_VPSS_ZME_H_LUMA_FIR_EN	BIT(26)
#define HISTB_VPSS_ZME_H_CHROMA_FIR_EN	BIT(25)
#define HISTB_VPSS_ZME_V_LUMA_EN		BIT(31)
#define HISTB_VPSS_ZME_V_CHROMA_EN	BIT(30)
#define HISTB_VPSS_ZME_V_LUMA_FIR_EN	BIT(24)
#define HISTB_VPSS_ZME_V_CHROMA_FIR_EN	BIT(23)
#define HISTB_VPSS_ZME_FMT_420_IN	BIT(19)
#define HISTB_VPSS_ZME_FMT_420_OUT	BIT(21)

#define HISTB_VPSS_INT_EOF		BIT(0)
#define HISTB_VPSS_INT_TIMEOUT		BIT(1)
#define HISTB_VPSS_INT_BUS_WRITE	BIT(2)
#define HISTB_VPSS_INT_EOF_END		BIT(3)
#define HISTB_VPSS_INT_DCMP		BIT(5)
#define HISTB_VPSS_INT_BUS_READ		BIT(6)
#define HISTB_VPSS_INT_IP_USED		BIT(7)
#define HISTB_VPSS_INT_IP_USING	BIT(8)
#define HISTB_VPSS_INT_DONE		(HISTB_VPSS_INT_EOF_END | \
					 HISTB_VPSS_INT_IP_USED)
#define HISTB_VPSS_INT_ERROR		(HISTB_VPSS_INT_TIMEOUT | \
					 HISTB_VPSS_INT_BUS_WRITE | \
					 HISTB_VPSS_INT_DCMP | \
					 HISTB_VPSS_INT_BUS_READ | \
					 HISTB_VPSS_INT_IP_USING)
#define HISTB_VPSS_INT_ALL		GENMASK(8, 0)

#define HISTB_VPSS_NODE_SIZE		SZ_64K
#define HISTB_VPSS_ZME_COEF_SIZE		SZ_4K
#define HISTB_VPSS_ZME_COEF_SLOT_SIZE	0x100
#define HISTB_VPSS_ZME_HL_COEF_OFFSET	0x000
#define HISTB_VPSS_ZME_HC_COEF_OFFSET	HISTB_VPSS_ZME_COEF_SLOT_SIZE
#define HISTB_VPSS_ZME_VL_COEF_OFFSET	(2 * HISTB_VPSS_ZME_COEF_SLOT_SIZE)
#define HISTB_VPSS_ZME_VC_COEF_OFFSET	(3 * HISTB_VPSS_ZME_COEF_SLOT_SIZE)
#define HISTB_VPSS_MISC_DEFAULT	0x03006466
#define HISTB_VPSS_JOB_TIMEOUT_MS	1000
#define HISTB_VPSS_CORE_RATE		400000000UL

struct histb_vpss {
	struct device *dev;
	void __iomem *regs;
	struct clk *clock;
	struct reset_control *reset;
	int irq;

	__le32 *node;
	dma_addr_t node_dma;
	__le32 *zme_coef;
	dma_addr_t zme_coef_dma;
	void *staging_cpu;
	dma_addr_t staging_dma;
	size_t staging_size;

	/* Serializes the single hardware pipeline and its staging buffer. */
	struct mutex lock;
	struct completion completion;
	u32 irq_state;
};

/* CV200 2x FIR tables: 17 symmetric phases for 8-tap luma and 4-tap chroma. */
static const s16 histb_vpss_zme_hl8_coef[17][8] = {
	{ 4, -22, 40, 468, 40, -22, 4, 0 },
	{ 3, -18, 26, 468, 54, -26, 5, 0 },
	{ 2, -14, 14, 466, 68, -30, 6, 0 },
	{ 2, -11, 2, 462, 84, -34, 7, 0 },
	{ 1, -7, -9, 457, 100, -38, 8, 0 },
	{ 1, -4, -18, 450, 116, -42, 9, 0 },
	{ 1, -2, -27, 443, 133, -46, 10, 0 },
	{ 0, 2, -35, 434, 151, -50, 10, 0 },
	{ 0, 4, -42, 425, 168, -54, 11, 0 },
	{ 0, 6, -49, 414, 186, -57, 12, 0 },
	{ 0, 8, -54, 401, 204, -60, 13, 0 },
	{ 0, 10, -58, 387, 222, -62, 13, 0 },
	{ 0, 11, -62, 374, 240, -65, 14, 0 },
	{ 0, 12, -65, 359, 258, -66, 14, 0 },
	{ 0, 13, -67, 344, 276, -68, 14, 0 },
	{ 0, 14, -68, 327, 293, -68, 14, 0 },
	{ 0, 14, -68, 310, 310, -68, 14, 0 },
};

static const s16 histb_vpss_zme_4t_coef[17][4] = {
	{ 103, 335, 103, -29 },
	{ 92, 335, 112, -27 },
	{ 84, 335, 121, -28 },
	{ 75, 334, 131, -28 },
	{ 67, 332, 141, -28 },
	{ 59, 329, 152, -28 },
	{ 51, 326, 162, -27 },
	{ 43, 323, 173, -27 },
	{ 36, 319, 183, -26 },
	{ 30, 313, 194, -25 },
	{ 23, 308, 204, -23 },
	{ 17, 301, 215, -21 },
	{ 12, 295, 225, -20 },
	{ 6, 288, 235, -17 },
	{ 2, 280, 244, -14 },
	{ -3, 271, 254, -10 },
	{ -7, 263, 263, -7 },
};

static void histb_vpss_node_write(struct histb_vpss *vpss, u32 reg,
				  u32 value)
{
	vpss->node[reg / sizeof(*vpss->node)] = cpu_to_le32(value);
}

static u32 histb_vpss_pack_10bit(s16 c0, s16 c1, s16 c2)
{
	return ((u16)c0 & GENMASK(9, 0)) |
	       (((u16)c1 & GENMASK(9, 0)) << 10) |
	       (((u16)c2 & GENMASK(9, 0)) << 20);
}

static u32 histb_vpss_pack_16bit(s16 c0, s16 c1)
{
	return (u16)c0 | (u32)(u16)c1 << 16;
}

static void histb_vpss_load_zme_coefficients(struct histb_vpss *vpss)
{
	__le32 *hl = (__le32 *)((u8 *)vpss->zme_coef +
				      HISTB_VPSS_ZME_HL_COEF_OFFSET);
	__le32 *hc = (__le32 *)((u8 *)vpss->zme_coef +
				      HISTB_VPSS_ZME_HC_COEF_OFFSET);
	__le32 *vl = (__le32 *)((u8 *)vpss->zme_coef +
				      HISTB_VPSS_ZME_VL_COEF_OFFSET);
	__le32 *vc = (__le32 *)((u8 *)vpss->zme_coef +
				      HISTB_VPSS_ZME_VC_COEF_OFFSET);
	u32 phase;

	memset(vpss->zme_coef, 0, HISTB_VPSS_ZME_COEF_SIZE);
	for (phase = 0; phase < ARRAY_SIZE(histb_vpss_zme_hl8_coef); phase++) {
		const s16 *h = histb_vpss_zme_hl8_coef[phase];
		const s16 *c = histb_vpss_zme_4t_coef[phase];

		hl[phase * 3] = cpu_to_le32(histb_vpss_pack_10bit(h[0], h[1], h[2]));
		hl[phase * 3 + 1] =
			cpu_to_le32(histb_vpss_pack_10bit(h[3], h[4], h[5]));
		hl[phase * 3 + 2] =
			cpu_to_le32(histb_vpss_pack_10bit(h[6], h[7], 0));
		hc[phase * 2] = cpu_to_le32(histb_vpss_pack_16bit(c[0], c[1]));
		hc[phase * 2 + 1] =
			cpu_to_le32(histb_vpss_pack_16bit(c[2], c[3]));
		vl[phase * 2] = hc[phase * 2];
		vl[phase * 2 + 1] = hc[phase * 2 + 1];
		vc[phase * 2] = hc[phase * 2];
		vc[phase * 2 + 1] = hc[phase * 2 + 1];
	}
}

static int histb_vpss_reset(struct histb_vpss *vpss)
{
	int ret;

	disable_irq(vpss->irq);
	writel(HISTB_VPSS_INT_ALL, vpss->regs + HISTB_VPSS_INT_MASK);
	writel(HISTB_VPSS_INT_ALL, vpss->regs + HISTB_VPSS_INT_CLEAR);
	ret = reset_control_assert(vpss->reset);
	if (ret)
		goto enable_irq;
	udelay(1);
	ret = reset_control_deassert(vpss->reset);
	if (ret)
		goto enable_irq;
	udelay(1);
	writel(HISTB_VPSS_MISC_DEFAULT, vpss->regs + HISTB_VPSS_MISC);
	writel(U32_MAX, vpss->regs + HISTB_VPSS_TIMEOUT);
	writel(0xfe, vpss->regs + HISTB_VPSS_INT_MASK);
	writel(HISTB_VPSS_INT_ALL, vpss->regs + HISTB_VPSS_INT_CLEAR);

enable_irq:
	enable_irq(vpss->irq);
	return ret;
}

static irqreturn_t histb_vpss_irq(int irq, void *data)
{
	struct histb_vpss *vpss = data;
	u32 state;

	state = readl(vpss->regs + HISTB_VPSS_INT_STATE) & HISTB_VPSS_INT_ALL;
	if (!state)
		return IRQ_NONE;

	writel(state, vpss->regs + HISTB_VPSS_INT_CLEAR);
	vpss->irq_state |= state;
	if (state & (HISTB_VPSS_INT_DONE | HISTB_VPSS_INT_ERROR))
		complete(&vpss->completion);

	return IRQ_HANDLED;
}

static size_t histb_vpss_output_stride(const struct histb_vpss_frame *frame)
{
	if (frame->output_ten_bit)
		return ALIGN(DIV_ROUND_UP((size_t)frame->width * 10, 8), 64);

	return ALIGN(frame->width, 64);
}

static int histb_vpss_prepare_staging(struct histb_vpss *vpss,
				      const struct histb_vpss_frame *frame)
{
	size_t stride = histb_vpss_output_stride(frame);
	size_t size;

	if (check_mul_overflow(stride, (size_t)frame->height * 3 / 2, &size))
		return -EOVERFLOW;
	if (vpss->staging_cpu && vpss->staging_size >= size)
		return 0;

	if (vpss->staging_cpu)
		dma_free_noncoherent(vpss->dev, vpss->staging_size,
				     vpss->staging_cpu, vpss->staging_dma,
				     DMA_FROM_DEVICE);
	vpss->staging_cpu = dma_alloc_noncoherent(vpss->dev, size,
						  &vpss->staging_dma,
						     DMA_FROM_DEVICE, GFP_KERNEL);
	if (!vpss->staging_cpu) {
		vpss->staging_size = 0;
		return -ENOMEM;
	}
	vpss->staging_size = size;
	if (upper_32_bits(vpss->staging_dma)) {
		dma_free_noncoherent(vpss->dev, vpss->staging_size,
				     vpss->staging_cpu, vpss->staging_dma,
				     DMA_FROM_DEVICE);
		vpss->staging_cpu = NULL;
		vpss->staging_size = 0;
		return -ERANGE;
	}

	return 0;
}

static __le64 histb_vpss_unpack_group(const u8 *src)
{
	u8 lsb[4];
	u8 msb[4];
	u64 packed;

	lsb[0] = src[0];
	msb[0] = src[1] & 3;
	lsb[1] = (src[1] >> 2) | ((src[2] & 3) << 6);
	msb[1] = (src[2] >> 2) & 3;
	lsb[2] = (src[2] >> 4) | ((src[3] & 0xf) << 4);
	msb[2] = (src[3] >> 4) & 3;
	lsb[3] = (src[3] >> 6) | ((src[4] & 0x3f) << 2);
	msb[3] = src[4] >> 6;

	packed = ((u64)(lsb[0] | (msb[0] << 8)) << 6) |
		 ((u64)(lsb[1] | (msb[1] << 8)) << 22) |
		 ((u64)(lsb[2] | (msb[2] << 8)) << 38) |
		 ((u64)(lsb[3] | (msb[3] << 8)) << 54);

	return cpu_to_le64(packed);
}

static void histb_vpss_unpack_p010(struct histb_vpss *vpss,
				   const struct histb_vpss_frame *frame)
{
	const u8 *src = vpss->staging_cpu;
	u8 *dst = frame->output_cpu;
	u32 packed_stride = histb_vpss_output_stride(frame);
	u32 plane, row, column;

	for (plane = 0; plane < 2; plane++) {
		u32 rows = plane ? frame->height / 2 : frame->height;
		u32 src_base = plane ? packed_stride * frame->height : 0;
		u32 dst_base = plane ? frame->output_stride * frame->height : 0;

		for (row = 0; row < rows; row++) {
			const u8 *src_row = src + src_base + row * packed_stride;
			u8 *dst_row = dst + dst_base + row * frame->output_stride;

			for (column = 0; column < frame->width; column += 4)
				put_unaligned(histb_vpss_unpack_group(
						      src_row + column * 5 / 4),
					      (__le64 *)(dst_row + column * 2));
		}
	}
}

static void histb_vpss_copy_nv12(struct histb_vpss *vpss,
				 const struct histb_vpss_frame *frame)
{
	const u8 *src = vpss->staging_cpu;
	u8 *dst = frame->output_cpu;
	u32 source_stride = histb_vpss_output_stride(frame);
	u32 plane, row;

	for (plane = 0; plane < 2; plane++) {
		u32 rows = plane ? frame->height / 2 : frame->height;
		u32 src_base = plane ? source_stride * frame->height : 0;
		u32 dst_base = plane ? frame->output_stride * frame->height : 0;

		for (row = 0; row < rows; row++)
			memcpy(dst + dst_base + row * frame->output_stride,
			       src + src_base + row * source_stride,
			       frame->width);
	}
}

static void histb_vpss_build_node(struct histb_vpss *vpss,
				  const struct histb_vpss_frame *frame,
				  dma_addr_t output_dma, u32 output_stride)
{
	u32 input_width = frame->input_width ?: frame->width;
	u32 input_height = frame->input_height ?: frame->height;
	bool scale = input_width != frame->width ||
		input_height != frame->height;
	u32 aligned_height = ALIGN(frame->height, frame->input_height_align);
	u32 input_chroma = frame->input_stride * aligned_height;
	u32 input_low_y = input_chroma * 3 / 2;
	u32 input_low_c = input_low_y + frame->input_stride / 4 * aligned_height;
	u32 output_ctrl = HISTB_VPSS_OUTPUT_UV_INVERT;

	memset(vpss->node, 0, HISTB_VPSS_NODE_SIZE);
	histb_vpss_node_write(vpss, HISTB_VPSS_CTRL,
			      HISTB_VPSS_CTRL_OUTPUT_EN |
				HISTB_VPSS_CTRL_FOUR_PIX);
	if (frame->input_ten_bit)
		histb_vpss_node_write(vpss, HISTB_VPSS_CTRL2,
				      HISTB_VPSS_CTRL2_INPUT_10BIT);
	histb_vpss_node_write(vpss, HISTB_VPSS_CTRL3,
			      scale ? HISTB_VPSS_CTRL3_ZME_EN : 0);
	histb_vpss_node_write(vpss, HISTB_VPSS_IMG_SIZE,
			      (input_height - 1) << 16 | (input_width - 1));
	if (scale) {
		u32 hratio = div_u64((u64)input_width *
				     HISTB_VPSS_ZME_H_RATIO_PRECISION,
				     frame->width);
		u32 vratio = div_u64((u64)input_height *
				     HISTB_VPSS_ZME_V_RATIO_PRECISION,
				     frame->height);

		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_OUTPUT_SIZE,
				      (frame->height - 1) << 16 |
				      (frame->width - 1));
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_ADDR,
				      lower_32_bits(vpss->node_dma +
						    HISTB_VPSS_ZME_HSP));
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_LH_COEF_ADDR,
				      lower_32_bits(vpss->zme_coef_dma +
						    HISTB_VPSS_ZME_HL_COEF_OFFSET));
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_LV_COEF_ADDR,
				      lower_32_bits(vpss->zme_coef_dma +
						    HISTB_VPSS_ZME_VL_COEF_OFFSET));
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_CH_COEF_ADDR,
				      lower_32_bits(vpss->zme_coef_dma +
						    HISTB_VPSS_ZME_HC_COEF_OFFSET));
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_CV_COEF_ADDR,
				      lower_32_bits(vpss->zme_coef_dma +
						    HISTB_VPSS_ZME_VC_COEF_OFFSET));
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_HSP,
				      HISTB_VPSS_ZME_H_LUMA_EN |
				      HISTB_VPSS_ZME_H_CHROMA_EN |
				      HISTB_VPSS_ZME_H_LUMA_FIR_EN |
				      HISTB_VPSS_ZME_H_CHROMA_FIR_EN |
				      hratio);
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_HL_OFFSET, 0);
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_HC_OFFSET, 0);
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_VSP,
				      HISTB_VPSS_ZME_V_LUMA_EN |
				      HISTB_VPSS_ZME_V_CHROMA_EN |
				      HISTB_VPSS_ZME_V_LUMA_FIR_EN |
				      HISTB_VPSS_ZME_V_CHROMA_FIR_EN |
				      HISTB_VPSS_ZME_FMT_420_IN |
				      HISTB_VPSS_ZME_FMT_420_OUT);
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_VSR, vratio);
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_V_OFFSET, 0);
	}
	if (frame->input_ten_bit) {
		histb_vpss_node_write(vpss, HISTB_VPSS_LB_Y_ADDR,
				      lower_32_bits(frame->input_dma + input_low_y));
		histb_vpss_node_write(vpss, HISTB_VPSS_LB_C_ADDR,
				      lower_32_bits(frame->input_dma + input_low_c));
		histb_vpss_node_write(vpss, HISTB_VPSS_LB_STRIDE,
				      frame->input_stride / 4);
	}
	if (frame->output_ten_bit)
		output_ctrl |= HISTB_VPSS_OUTPUT_10BIT;
	else if (scale)
		output_ctrl |= HISTB_VPSS_OUTPUT_DITHER |
			       HISTB_VPSS_OUTPUT_DITHER_ROUND;
	else
		output_ctrl |= HISTB_VPSS_OUTPUT_DITHER;
	histb_vpss_node_write(vpss, HISTB_VPSS_INT_MASK, 0xff);
	histb_vpss_node_write(vpss, HISTB_VPSS_INPUT_CTRL,
			      HISTB_VPSS_INPUT_TILE);
	histb_vpss_node_write(vpss, HISTB_VPSS_INPUT_Y_ADDR,
			      lower_32_bits(frame->input_dma));
	histb_vpss_node_write(vpss, HISTB_VPSS_INPUT_C_ADDR,
			      lower_32_bits(frame->input_dma + input_chroma));
	histb_vpss_node_write(vpss, HISTB_VPSS_INPUT_STRIDE,
			      frame->input_stride << 16 | frame->input_stride);
	histb_vpss_node_write(vpss, HISTB_VPSS_OUTPUT_CTRL, output_ctrl);
	histb_vpss_node_write(vpss, HISTB_VPSS_OUTPUT_SIZE,
			      (frame->height - 1) << 16 |
				(ALIGN(frame->width, 4) - 1));
	histb_vpss_node_write(vpss, HISTB_VPSS_OUTPUT_Y_ADDR,
			      lower_32_bits(output_dma));
	histb_vpss_node_write(vpss, HISTB_VPSS_OUTPUT_C_ADDR,
			      lower_32_bits(output_dma + output_stride * frame->height));
	histb_vpss_node_write(vpss, HISTB_VPSS_OUTPUT_STRIDE,
			      output_stride << 16 | output_stride);
	histb_vpss_node_write(vpss, HISTB_VPSS_RCH_BYPASS, U32_MAX);
	histb_vpss_node_write(vpss, HISTB_VPSS_WCH_BYPASS, U32_MAX);
	histb_vpss_node_write(vpss, HISTB_VPSS_NEXT, 0);
	histb_vpss_node_write(vpss, HISTB_VPSS_MISC,
			      HISTB_VPSS_MISC_DEFAULT);
}

static void histb_vpss_dei_field(struct histb_vpss *vpss, u32 ctrl_off,
				 u32 y_off, u32 c_off, u32 stride_off,
				 const struct histb_vpss_dei_frame *frame,
				 dma_addr_t dma, u32 stride)
{
	writel(frame->tile ? HISTB_VPSS_DEI_TILE_FORMAT : 0,
	       vpss->regs + ctrl_off);
	writel(lower_32_bits(dma), vpss->regs + y_off);
	writel(lower_32_bits(dma + stride * frame->height),
	       vpss->regs + c_off);
	writel(stride | stride << 16, vpss->regs + stride_off);
}

/*
 * Hardware de-interlace: four consecutive fields in, one progressive frame
 * out.  The block lives in the VPSS at DIECTRL (0x1000) and is switched on
 * with VPSS_CTRL bit 7; it shares the input field registers with the ZME
 * path this driver already uses, which is why it cannot run as an
 * independent node at the same time as a scaler job.
 *
 * Thresholds are left at the block's own defaults.  The vendor tuning values
 * live in a binary PQ table (PQ_HAL_SetDeiRegist reads a DEI_PARAMETER_S
 * filled from PQ_FILE_HEADER_S), so there is no compiled-in set to copy;
 * only edge smoothing is enabled explicitly, as the BSP does.
 */
int histb_vpss_dei(struct histb_vpss *vpss,
		   const struct histb_vpss_dei_frame *frame,
		   dma_addr_t output_dma)
{
	u32 stride = frame ? frame->stride : 0;
	u32 ctrl;
	unsigned long timeout;
	int ret;

	if (!vpss || !frame || !frame->width || !frame->height || !stride)
		return -EINVAL;
	if (frame->width > HISTB_VPSS_DEI_MAX_WIDTH ||
	    frame->height * 2 > HISTB_VPSS_DEI_MAX_HEIGHT)
		return -EINVAL;
	if (stride & 15 || stride < frame->width * (frame->ten_bit ? 2 : 1))
		return -EINVAL;
	if (!frame->ref_dma || !frame->cur_dma || !frame->nxt1_dma ||
	    !frame->nxt2_dma || !output_dma)
		return -EINVAL;
	if (upper_32_bits(frame->ref_dma) || upper_32_bits(frame->cur_dma) ||
	    upper_32_bits(frame->nxt1_dma) || upper_32_bits(frame->nxt2_dma) ||
	    upper_32_bits(output_dma))
		return -EINVAL;

	mutex_lock(&vpss->lock);

	ret = pm_runtime_resume_and_get(vpss->dev);
	if (ret < 0)
		goto unlock;

	/* Luma and chroma output of the de-interlacer. */
	writel(lower_32_bits(output_dma), vpss->regs + HISTB_VPSS_LB_Y_ADDR);
	writel(lower_32_bits(output_dma + stride *
			     (frame->height * 2)),
	       vpss->regs + HISTB_VPSS_LB_C_ADDR);
	writel(stride, vpss->regs + HISTB_VPSS_LB_STRIDE);
	writel(FIELD_PREP(GENMASK(15, 0), frame->width - 1) |
	       FIELD_PREP(GENMASK(31, 16), frame->height * 2 - 1),
	       vpss->regs + HISTB_VPSS_IMG_SIZE);

	histb_vpss_dei_field(vpss, HISTB_VPSS_DEI_REF_CTRL,
			     HISTB_VPSS_DEI_REFYADDR, HISTB_VPSS_DEI_REFCADDR,
			     HISTB_VPSS_DEI_REFSTRIDE, frame, frame->ref_dma,
			     stride);
	histb_vpss_dei_field(vpss, HISTB_VPSS_DEI_CUR_CTRL,
			     HISTB_VPSS_DEI_CURYADDR, HISTB_VPSS_DEI_CURCADDR,
			     HISTB_VPSS_DEI_CURSTRIDE, frame, frame->cur_dma,
			     stride);
	histb_vpss_dei_field(vpss, HISTB_VPSS_DEI_NXT1_CTRL,
			     HISTB_VPSS_DEI_NXT1YADDR, HISTB_VPSS_DEI_NXT1CADDR,
			     HISTB_VPSS_DEI_NXT1STRIDE, frame, frame->nxt1_dma,
			     stride);
	histb_vpss_dei_field(vpss, HISTB_VPSS_DEI_NXT2_CTRL,
			     HISTB_VPSS_DEI_NXT2YADDR, HISTB_VPSS_DEI_NXT2CADDR,
			     HISTB_VPSS_DEI_NXT2STRIDE, frame, frame->nxt2_dma,
			     stride);

	/* 4-field mode, as the vendor HAL selects (`SetMode(..., 1)`). */
	writel(FIELD_PREP(HISTB_VPSS_DIE_L_MODE, 1) |
	       FIELD_PREP(HISTB_VPSS_DIE_C_MODE, 1) |
	       HISTB_VPSS_DIE_EDGE_SMOOTH_EN, vpss->regs + HISTB_VPSS_DIECTRL);

	ctrl = readl(vpss->regs + HISTB_VPSS_CTRL);
	ctrl |= HISTB_VPSS_CTRL_DEI_EN;
	ctrl &= ~HISTB_VPSS_CTRL_BFIELD_FIRST;
	if (!frame->top_field_first)
		ctrl |= HISTB_VPSS_CTRL_BFIELD_FIRST;
	ctrl &= ~HISTB_VPSS_CTRL_BFIELD_MODE;
	writel(ctrl, vpss->regs + HISTB_VPSS_CTRL);

	reinit_completion(&vpss->completion);
	vpss->irq_state = 0;
	writel(HISTB_VPSS_INT_ALL, vpss->regs + HISTB_VPSS_INT_CLEAR);
	writel(HISTB_VPSS_MISC_DEFAULT, vpss->regs + HISTB_VPSS_MISC);
	writel(0, vpss->regs + HISTB_VPSS_NEXT);
	wmb();
	writel(1, vpss->regs + HISTB_VPSS_START);

	timeout = wait_for_completion_timeout(&vpss->completion,
					      msecs_to_jiffies(500));
	ret = timeout ? 0 : -ETIMEDOUT;

	writel(0, vpss->regs + HISTB_VPSS_CTRL);
	pm_runtime_put(vpss->dev);
unlock:
	mutex_unlock(&vpss->lock);
	return ret;
}

int histb_vpss_detile(struct histb_vpss *vpss,
		      const struct histb_vpss_frame *frame)
{
	u32 input_width;
	u32 input_height;
	bool direct_output;
	dma_addr_t output_dma;
	u32 output_stride;
	unsigned long timeout;
	int ret;

	if (!vpss || !frame || !frame->width || !frame->height ||
	    !frame->input_stride || frame->width & 3 ||
	    upper_32_bits(frame->input_dma))
		return -EINVAL;
	input_width = frame->input_width ?: frame->width;
	input_height = frame->input_height ?: frame->height;
	if (!input_width || !input_height || input_width > frame->width ||
	    input_height > frame->height || input_width & 3)
		return -EINVAL;
	if (frame->output_stride <
	    frame->width * (frame->output_ten_bit ? 2 : 1) ||
	    upper_32_bits(frame->output_dma))
		return -EINVAL;
	direct_output = !frame->output_ten_bit && frame->output_dma;
	if ((!direct_output && !frame->output_cpu) ||
	    (direct_output &&
	     frame->output_stride < histb_vpss_output_stride(frame)))
		return -EINVAL;

	mutex_lock(&vpss->lock);
	if (direct_output) {
		output_dma = frame->output_dma;
		output_stride = frame->output_stride;
	} else {
		ret = histb_vpss_prepare_staging(vpss, frame);
		if (ret)
			goto unlock;
		output_dma = vpss->staging_dma;
		output_stride = histb_vpss_output_stride(frame);
	}

	ret = pm_runtime_resume_and_get(vpss->dev);
	if (ret < 0)
		goto unlock;

	histb_vpss_build_node(vpss, frame, output_dma, output_stride);
	reinit_completion(&vpss->completion);
	vpss->irq_state = 0;
	if (!direct_output)
		dma_sync_single_for_device(vpss->dev, vpss->staging_dma,
					   vpss->staging_size,
					   DMA_FROM_DEVICE);
	writel(HISTB_VPSS_INT_ALL, vpss->regs + HISTB_VPSS_INT_CLEAR);
	writel(HISTB_VPSS_MISC_DEFAULT, vpss->regs + HISTB_VPSS_MISC);
	dma_wmb();
	writel(lower_32_bits(vpss->node_dma), vpss->regs + HISTB_VPSS_NEXT);
	/* The engine must observe PNEXT only after the complete DDR node. */
	wmb();
	writel(1, vpss->regs + HISTB_VPSS_START);

	timeout = wait_for_completion_timeout(&vpss->completion,
					      msecs_to_jiffies(HISTB_VPSS_JOB_TIMEOUT_MS));
	if (!timeout) {
		dev_err(vpss->dev, "post-processing timed out\n");
		ret = -ETIMEDOUT;
	} else if (vpss->irq_state & HISTB_VPSS_INT_ERROR) {
		dev_err(vpss->dev, "post-processing failed, state=%#x\n",
			vpss->irq_state);
		ret = -EIO;
	} else {
		if (!direct_output) {
			dma_sync_single_for_cpu(vpss->dev, vpss->staging_dma,
						vpss->staging_size, DMA_FROM_DEVICE);
			if (frame->output_ten_bit)
				histb_vpss_unpack_p010(vpss, frame);
			else
				histb_vpss_copy_nv12(vpss, frame);
		}
		ret = 0;
	}
	if (ret)
		histb_vpss_reset(vpss);
	pm_runtime_mark_last_busy(vpss->dev);
	pm_runtime_put_autosuspend(vpss->dev);

unlock:
	mutex_unlock(&vpss->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(histb_vpss_detile);

struct histb_vpss *histb_vpss_get(struct device *consumer)
{
	struct platform_device *pdev;
	struct device_node *node;
	struct histb_vpss *vpss;

	node = of_parse_phandle(consumer->of_node, "hisilicon,vpss", 0);
	if (!node)
		return ERR_PTR(-ENODEV);
	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	vpss = platform_get_drvdata(pdev);
	if (!vpss) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	if (!device_link_add(consumer, &pdev->dev,
			     DL_FLAG_AUTOREMOVE_CONSUMER | DL_FLAG_PM_RUNTIME)) {
		put_device(&pdev->dev);
		return ERR_PTR(-ENOMEM);
	}

	return vpss;
}
EXPORT_SYMBOL_GPL(histb_vpss_get);

void histb_vpss_put(struct histb_vpss *vpss)
{
	if (vpss)
		put_device(vpss->dev);
}
EXPORT_SYMBOL_GPL(histb_vpss_put);

static int histb_vpss_runtime_resume(struct device *dev)
{
	struct histb_vpss *vpss = dev_get_drvdata(dev);
	int ret;

	ret = reset_control_assert(vpss->reset);
	if (ret)
		return ret;
	udelay(1);
	ret = clk_set_rate(vpss->clock, HISTB_VPSS_CORE_RATE);
	if (ret)
		return ret;
	if (clk_get_rate(vpss->clock) != HISTB_VPSS_CORE_RATE) {
		dev_err(vpss->dev, "VPSS core clock did not reach %lu Hz\n",
			HISTB_VPSS_CORE_RATE);
		return -EIO;
	}
	ret = clk_prepare_enable(vpss->clock);
	if (ret)
		return ret;
	udelay(1);
	ret = reset_control_deassert(vpss->reset);
	if (ret) {
		clk_disable_unprepare(vpss->clock);
		return ret;
	}
	udelay(1);
	writel(HISTB_VPSS_MISC_DEFAULT, vpss->regs + HISTB_VPSS_MISC);
	writel(U32_MAX, vpss->regs + HISTB_VPSS_TIMEOUT);
	writel(0xfe, vpss->regs + HISTB_VPSS_INT_MASK);
	writel(HISTB_VPSS_INT_ALL, vpss->regs + HISTB_VPSS_INT_CLEAR);
	enable_irq(vpss->irq);
	return 0;
}

static int histb_vpss_runtime_suspend(struct device *dev)
{
	struct histb_vpss *vpss = dev_get_drvdata(dev);
	int ret;

	disable_irq(vpss->irq);
	writel(HISTB_VPSS_INT_ALL, vpss->regs + HISTB_VPSS_INT_MASK);
	writel(HISTB_VPSS_INT_ALL, vpss->regs + HISTB_VPSS_INT_CLEAR);
	ret = reset_control_assert(vpss->reset);
	if (ret) {
		enable_irq(vpss->irq);
		return ret;
	}
	clk_disable_unprepare(vpss->clock);
	return 0;
}

static const struct dev_pm_ops histb_vpss_pm_ops = {
	SET_RUNTIME_PM_OPS(histb_vpss_runtime_suspend,
			   histb_vpss_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static int histb_vpss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_vpss *vpss;
	int ret;

	vpss = devm_kzalloc(dev, sizeof(*vpss), GFP_KERNEL);
	if (!vpss)
		return -ENOMEM;
	vpss->dev = dev;
	vpss->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vpss->regs))
		return PTR_ERR(vpss->regs);
	vpss->clock = devm_clk_get(dev, "core");
	if (IS_ERR(vpss->clock))
		return dev_err_probe(dev, PTR_ERR(vpss->clock),
				     "failed to get core clock\n");
	vpss->reset = devm_reset_control_get_exclusive(dev, "core");
	if (IS_ERR(vpss->reset))
		return dev_err_probe(dev, PTR_ERR(vpss->reset),
				     "failed to get reset\n");
	vpss->irq = platform_get_irq(pdev, 0);
	if (vpss->irq < 0)
		return vpss->irq;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	vpss->node = dma_alloc_coherent(dev, HISTB_VPSS_NODE_SIZE,
					&vpss->node_dma, GFP_KERNEL);
	if (!vpss->node)
		return -ENOMEM;
	if (upper_32_bits(vpss->node_dma)) {
		ret = -ERANGE;
		goto free_node;
	}
	vpss->zme_coef = dma_alloc_coherent(dev, HISTB_VPSS_ZME_COEF_SIZE,
					    &vpss->zme_coef_dma, GFP_KERNEL);
	if (!vpss->zme_coef) {
		ret = -ENOMEM;
		goto free_node;
	}
	if (upper_32_bits(vpss->zme_coef_dma)) {
		ret = -ERANGE;
		goto free_zme_coef;
	}
	histb_vpss_load_zme_coefficients(vpss);
	mutex_init(&vpss->lock);
	init_completion(&vpss->completion);
	platform_set_drvdata(pdev, vpss);

	ret = devm_request_irq(dev, vpss->irq, histb_vpss_irq,
			       IRQF_NO_AUTOEN, dev_name(dev), vpss);
	if (ret) {
		ret = dev_err_probe(dev, ret, "failed to request IRQ\n");
		goto free_zme_coef;
	}

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
	dev_info(dev, "registered native tile post-processor\n");
	return 0;

free_zme_coef:
	dma_free_coherent(dev, HISTB_VPSS_ZME_COEF_SIZE,
			  vpss->zme_coef, vpss->zme_coef_dma);
free_node:
	dma_free_coherent(dev, HISTB_VPSS_NODE_SIZE,
			  vpss->node, vpss->node_dma);
	return ret;
}

static void histb_vpss_remove(struct platform_device *pdev)
{
	struct histb_vpss *vpss = platform_get_drvdata(pdev);

	pm_runtime_disable(vpss->dev);
	if (!pm_runtime_status_suspended(vpss->dev))
		histb_vpss_runtime_suspend(vpss->dev);
	if (vpss->staging_cpu)
		dma_free_noncoherent(vpss->dev, vpss->staging_size,
				     vpss->staging_cpu, vpss->staging_dma,
				     DMA_FROM_DEVICE);
	dma_free_coherent(vpss->dev, HISTB_VPSS_ZME_COEF_SIZE,
			  vpss->zme_coef, vpss->zme_coef_dma);
	dma_free_coherent(vpss->dev, HISTB_VPSS_NODE_SIZE,
			  vpss->node, vpss->node_dma);
}

static const struct of_device_id histb_vpss_of_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-vpss" },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_vpss_of_match);

static struct platform_driver histb_vpss_driver = {
	.probe = histb_vpss_probe,
	.remove_new = histb_vpss_remove,
	.driver = {
		.name = "histb-vpss",
		.of_match_table = histb_vpss_of_match,
		.pm = &histb_vpss_pm_ops,
	},
};
module_platform_driver(histb_vpss_driver);

MODULE_AUTHOR("HiSilicon Technologies Co., Ltd.");
MODULE_DESCRIPTION("HiSilicon Hi3798CV200 video post-processing engine");
MODULE_LICENSE("GPL");
