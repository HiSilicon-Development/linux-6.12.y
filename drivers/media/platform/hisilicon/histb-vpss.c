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
#define HISTB_VPSS_DEI_PARAM_BASE	0x1000

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
#define HISTB_VPSS_DIE_OUT_SEL_C	BIT(28)
#define HISTB_VPSS_DIE_OUT_SEL_L	BIT(29)

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

	phys_addr_t regs_phys;
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

static u32 histb_vpss_node_read(struct histb_vpss *vpss, u32 reg)
{
	return le32_to_cpu(vpss->node[reg / sizeof(*vpss->node)]);
}

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

struct histb_vpss_dei_default {
	u16 reg;
	u8 msb;
	u8 lsb;
	s32 value;
};

/* CV200 de-interlacer defaults, from the vendor table
 * hal/3798cv200/pq_hal_table_default.c (403 HI_PQ_MODULE_DEI rows).
 * Format: register, msb, lsb, value, name. */
static const struct histb_vpss_dei_default histb_vpss_dei_defaults[] = {
	{ 0x1000, 22, 22,      0 }, /* mc_only */
	{ 0x1000, 21, 21,      0 }, /* ma_only */
	{ 0x1000, 29, 29,      0 }, /* die_out_sel_l */
	{ 0x1000, 28, 28,      0 }, /* die_out_sel_c */
	{ 0x1000, 16, 16,      0 }, /* stinfo_stop */
	{ 0x1000, 26, 27,      1 }, /* die_l_mode */
	{ 0x1000, 24, 25,      1 }, /* die_c_mode */
	{ 0x1000, 17, 17,      0 }, /* die_rst */
	{ 0x1004, 24, 31,      8 }, /* chroma_mf_offset */
	{ 0x1004,  7,  7,      1 }, /* rec_mode_en */
	{ 0x1004,  6,  6,      0 }, /* chroma_mf_max */
	{ 0x1004,  5,  5,      0 }, /* luma_mf_max */
	{ 0x1004,  4,  4,      1 }, /* motion_iir_en */
	{ 0x1004,  2,  2,      0 }, /* luma_scesdf_max */
	{ 0x1004,  1,  1,      1 }, /* frame_motion_smooth_en */
	{ 0x1004,  0,  0,      0 }, /* recmode_frmfld_blend_mode */
	{ 0x1008, 16, 31,   -320 }, /* ver_min_inten */
	{ 0x1008,  8, 11,      2 }, /* dir_inten_ver */
	{ 0x100c,  0,  7,      2 }, /* range_scale */
	{ 0x1010, 16, 19,      8 }, /* ck1_gain */
	{ 0x1010,  8, 11,      2 }, /* ck1_range_gain */
	{ 0x1010,  0,  7,     30 }, /* ck1_max_range */
	{ 0x1014, 16, 19,      8 }, /* ck2_gain */
	{ 0x1014,  8, 11,      2 }, /* ck2_range_gain */
	{ 0x1014,  0,  7,     30 }, /* ck2_max_range */
	{ 0x1024, 16, 21,      3 }, /* dir14_mult */
	{ 0x1024,  8, 13,      5 }, /* dir13_mult */
	{ 0x1024,  0,  5,      5 }, /* dir12_mult */
	{ 0x1020, 24, 29,      6 }, /* dir11_mult */
	{ 0x1020, 16, 21,      7 }, /* dir10_mult */
	{ 0x1020,  8, 13,      8 }, /* dir9_mult */
	{ 0x1020,  0,  5,      9 }, /* dir8_mult */
	{ 0x101c, 24, 29,     11 }, /* dir7_mult */
	{ 0x101c, 16, 21,     12 }, /* dir6_mult */
	{ 0x101c,  8, 13,     15 }, /* dir5_mult */
	{ 0x101c,  0,  5,     18 }, /* dir4_mult */
	{ 0x1018, 24, 29,     27 }, /* dir3_mult */
	{ 0x1018, 16, 21,     32 }, /* dir2_mult */
	{ 0x1018,  8, 13,     24 }, /* dir1_mult */
	{ 0x1018,  0,  5,     40 }, /* dir0_mult */
	{ 0x1030, 24, 27,      8 }, /* intp_scale_ratio_15 */
	{ 0x1030, 20, 23,      8 }, /* intp_scale_ratio_14 */
	{ 0x1030, 16, 19,      8 }, /* intp_scale_ratio_13 */
	{ 0x1030, 12, 15,      8 }, /* intp_scale_ratio_12 */
	{ 0x1030,  8, 11,      8 }, /* intp_scale_ratio_11 */
	{ 0x1030,  4,  7,      8 }, /* intp_scale_ratio_10 */
	{ 0x1030,  0,  3,      8 }, /* intp_scale_ratio_9 */
	{ 0x102c, 28, 31,      8 }, /* intp_scale_ratio_8 */
	{ 0x102c, 24, 27,      7 }, /* intp_scale_ratio_7 */
	{ 0x102c, 20, 23,      7 }, /* intp_scale_ratio_6 */
	{ 0x102c, 16, 19,      6 }, /* intp_scale_ratio_5 */
	{ 0x102c, 12, 15,      6 }, /* intp_scale_ratio_4 */
	{ 0x102c,  8, 11,      5 }, /* intp_scale_ratio_3 */
	{ 0x102c,  4,  7,      5 }, /* intp_scale_ratio_2 */
	{ 0x102c,  0,  3,      6 }, /* intp_scale_ratio_1 */
	{ 0x1034, 16, 31,   5000 }, /* strength_thd */
	{ 0x1034, 13, 13,      0 }, /* hor_edge_en */
	{ 0x1034, 12, 12,      0 }, /* edge_mode */
	{ 0x1034,  8, 11,      4 }, /* dir_thd */
	{ 0x1034,  0,  6,     32 }, /* bc_gain */
	{ 0x1038, 12, 19,      0 }, /* fld_motion_coring */
	{ 0x1038,  4, 11,      0 }, /* jitter_coring */
	{ 0x1038,  0,  3,      0 }, /* jitter_gain */
	{ 0x103c, 29, 29,      0 }, /* long_motion_shf */
	{ 0x103c, 28, 28,      1 }, /* fld_motion_wnd_mode */
	{ 0x103c, 24, 27,      8 }, /* fld_motion_gain */
	{ 0x103c, 16, 21,     -2 }, /* fld_motion_curve_slope */
	{ 0x103c,  8, 15,    255 }, /* fld_motion_thd_high */
	{ 0x103c,  0,  7,      0 }, /* fld_motion_thd_low */
	{ 0x1044, 24, 30,     64 }, /* max_motion_iir_ratio */
	{ 0x1044, 16, 22,     32 }, /* min_motion_iir_ratio */
	{ 0x1044,  8, 15,    255 }, /* motion_diff_thd_5 */
	{ 0x1044,  0,  7,    255 }, /* motion_diff_thd_4 */
	{ 0x1040, 24, 31,    255 }, /* motion_diff_thd_3 */
	{ 0x1040, 16, 23,    208 }, /* motion_diff_thd_2 */
	{ 0x1040,  8, 15,    144 }, /* motion_diff_thd_1 */
	{ 0x1040,  0,  7,     16 }, /* motion_diff_thd_0 */
	{ 0x1048, 18, 23,      0 }, /* motion_iir_curve_slope_3 */
	{ 0x1048, 12, 17,      0 }, /* motion_iir_curve_slope_2 */
	{ 0x1048,  6, 11,      2 }, /* motion_iir_curve_slope_1 */
	{ 0x1048,  0,  5,      1 }, /* motion_iir_curve_slope_0 */
	{ 0x104c, 24, 30,     64 }, /* motion_iir_curve_ratio_4 */
	{ 0x104c, 16, 22,     64 }, /* motion_iir_curve_ratio_3 */
	{ 0x104c,  8, 14,     64 }, /* motion_iir_curve_ratio_2 */
	{ 0x104c,  0,  6,     48 }, /* motion_iir_curve_ratio_1 */
	{ 0x1048, 24, 30,     32 }, /* motion_iir_curve_ratio_0 */
	{ 0x1050, 21, 21,      0 }, /* his_motion_info_write_mode */
	{ 0x1050, 20, 20,      0 }, /* his_motion_write_mode */
	{ 0x1050, 19, 19,      1 }, /* his_motion_using_mode */
	{ 0x1050, 18, 18,      1 }, /* his_motion_en */
	{ 0x1050, 17, 17,      0 }, /* pre_info_en */
	{ 0x1050, 16, 16,      1 }, /* ppre_info_en */
	{ 0x1050, 12, 13,      0 }, /* rec_mode_frm_motion_step_1 */
	{ 0x1050,  8,  9,      0 }, /* rec_mode_frm_motion_step_0 */
	{ 0x1050,  4,  6,      2 }, /* rec_mode_fld_motion_step_1 */
	{ 0x1050,  0,  2,      2 }, /* rec_mode_fld_motion_step_0 */
	{ 0x1054, 26, 26,      0 }, /* med_blend_en */
	{ 0x1054, 25, 25,      0 }, /* reserved_1 */
	{ 0x1054, 24, 24,      1 }, /* mor_flt_en */
	{ 0x1054, 10, 23,      0 }, /* reserved_2 */
	{ 0x1054,  8,  9,      0 }, /* mor_flt_size */
	{ 0x1054,  0,  7,      0 }, /* mor_flt_thd */
	{ 0x105c, 16, 16,      0 }, /* comb_chk_en */
	{ 0x105c,  8, 12,     30 }, /* comb_chk_md_thd */
	{ 0x105c,  0,  6,     64 }, /* comb_chk_edge_thd */
	{ 0x1058, 24, 31,    160 }, /* comb_chk_upper_limit */
	{ 0x1058, 16, 23,     10 }, /* comb_chk_lower_limit */
	{ 0x1058,  8, 15,     15 }, /* comb_chk_min_vthd */
	{ 0x1058,  0,  7,    255 }, /* comb_chk_min_hthd */
	{ 0x1064, 24, 30,     64 }, /* frame_motion_smooth_ratio_max */
	{ 0x1064, 16, 22,      0 }, /* frame_motion_smooth_ratio_min */
	{ 0x1064,  8, 15,    255 }, /* frame_motion_smooth_thd5 */
	{ 0x1064,  0,  7,    255 }, /* frame_motion_smooth_thd4 */
	{ 0x1060, 24, 31,    255 }, /* frame_motion_smooth_thd3 */
	{ 0x1060, 16, 23,    255 }, /* frame_motion_smooth_thd2 */
	{ 0x1060,  8, 15,     72 }, /* frame_motion_smooth_thd1 */
	{ 0x1060,  0,  7,      8 }, /* frame_motion_smooth_thd0 */
	{ 0x1068, 18, 23,      0 }, /* frame_motion_smooth_slope3 */
	{ 0x1068, 12, 17,      0 }, /* frame_motion_smooth_slope2 */
	{ 0x1068,  6, 11,      0 }, /* frame_motion_smooth_slope1 */
	{ 0x1068,  0,  5,      8 }, /* frame_motion_smooth_slope0 */
	{ 0x106c, 24, 30,     64 }, /* frame_motion_smooth_ratio4 */
	{ 0x106c, 16, 22,     64 }, /* frame_motion_smooth_ratio3 */
	{ 0x106c,  8, 14,     64 }, /* frame_motion_smooth_ratio2 */
	{ 0x106c,  0,  6,     64 }, /* frame_motion_smooth_ratio1 */
	{ 0x1068, 24, 30,      0 }, /* frame_motion_smooth_ratio0 */
	{ 0x1074, 24, 30,     64 }, /* frame_field_blend_ratio_max */
	{ 0x1074, 16, 22,      0 }, /* frame_field_blend_ratio_min */
	{ 0x1074,  8, 15,    255 }, /* frame_field_blend_thd5 */
	{ 0x1074,  0,  7,    255 }, /* frame_field_blend_thd4 */
	{ 0x1070, 24, 31,    255 }, /* frame_field_blend_thd3 */
	{ 0x1070, 16, 23,    255 }, /* frame_field_blend_thd2 */
	{ 0x1070,  8, 15,     72 }, /* frame_field_blend_thd1 */
	{ 0x1070,  0,  7,      8 }, /* frame_field_blend_thd0 */
	{ 0x1078, 18, 23,      0 }, /* frame_field_blend_slope3 */
	{ 0x1078, 12, 17,      0 }, /* frame_field_blend_slope2 */
	{ 0x1078,  6, 11,      0 }, /* frame_field_blend_slope1 */
	{ 0x1078,  0,  5,      8 }, /* frame_field_blend_slope0 */
	{ 0x107c, 24, 30,     64 }, /* frame_field_blend_ratio4 */
	{ 0x107c, 16, 22,     64 }, /* frame_field_blend_ratio3 */
	{ 0x107c,  8, 14,     64 }, /* frame_field_blend_ratio2 */
	{ 0x107c,  0,  6,     64 }, /* frame_field_blend_ratio1 */
	{ 0x1078, 24, 30,      0 }, /* frame_field_blend_ratio0 */
	{ 0x1080, 16, 23,    128 }, /* motion_adjust_gain_chr */
	{ 0x1080,  8, 13,      0 }, /* motion_adjust_coring */
	{ 0x1080,  0,  7,    128 }, /* motion_adjust_gain */
	{ 0x1098, 12, 23,    134 }, /* edge_norm_11 */
	{ 0x1098,  0, 11,    135 }, /* edge_norm_10 */
	{ 0x1094, 12, 23,    143 }, /* edge_norm_9 */
	{ 0x1094,  0, 11,    142 }, /* edge_norm_8 */
	{ 0x1090, 12, 23,    132 }, /* edge_norm_7 */
	{ 0x1090,  0, 11,    140 }, /* edge_norm_6 */
	{ 0x108c, 12, 23,    132 }, /* edge_norm_5 */
	{ 0x108c,  0, 11,    134 }, /* edge_norm_4 */
	{ 0x1088, 12, 23,    115 }, /* edge_norm_3 */
	{ 0x1088,  0, 11,    136 }, /* edge_norm_2 */
	{ 0x1084, 12, 23,    227 }, /* edge_norm_1 */
	{ 0x1084,  0, 11,      0 }, /* edge_norm_0 */
	{ 0x1094, 24, 31,     32 }, /* inter_diff_thd0 */
	{ 0x1098, 24, 31,     64 }, /* inter_diff_thd1 */
	{ 0x109c, 24, 31,    255 }, /* inter_diff_thd2 */
	{ 0x109c, 12, 23,     32 }, /* edge_scale */
	{ 0x109c,  0, 11,     16 }, /* edge_coring */
	{ 0x10a4, 24, 31,     16 }, /* mc_strength_maxg */
	{ 0x10a4, 16, 23,     16 }, /* mc_strength_ming */
	{ 0x1090, 24, 31,     64 }, /* mc_strength_g3 */
	{ 0x10a4,  8, 15,     64 }, /* mc_strength_g2 */
	{ 0x10a4,  0,  7,     16 }, /* mc_strength_g1 */
	{ 0x10a0, 24, 31,     16 }, /* mc_strength_g0 */
	{ 0x108c, 24, 31,      0 }, /* mc_strength_k3 */
	{ 0x10a0, 16, 23,      0 }, /* mc_strength_k2 */
	{ 0x10a0,  8, 15,      6 }, /* mc_strength_k1 */
	{ 0x10a0,  0,  7,      0 }, /* mc_strength_k0 */
	{ 0x10a8, 24, 30,     64 }, /* k_c_mcbld */
	{ 0x10a8, 16, 22,      8 }, /* k_c_mcw */
	{ 0x10a8,  8, 14,     64 }, /* k_y_mcbld */
	{ 0x10a8,  0,  6,     16 }, /* k_y_mcw */
	{ 0x10ac, 16, 27,   1023 }, /* g0_mcw_adj */
	{ 0x10ac,  8, 15,     64 }, /* k0_mcw_adj */
	{ 0x10ac,  0,  7,     64 }, /* x0_mcw_adj */
	{ 0x10b0,  0,  7,    128 }, /* k1_mcw_adj */
	{ 0x10b0, 24, 31,      0 }, /* k1_mcbld */
	{ 0x10b0, 16, 23,      0 }, /* k0_mcbld */
	{ 0x10b0,  8, 15,      0 }, /* x0_mcbld */
	{ 0x10b4,  0, 11,      0 }, /* g0_mcbld */
	{ 0x10b4, 20, 20,      1 }, /* mc_lai_bldmode */
	{ 0x10b4, 12, 16,      0 }, /* k_curw_mcbld */
	{ 0x10b8, 16, 25,     16 }, /* ma_gbm_thd0 */
	{ 0x10b8,  0,  9,     48 }, /* ma_gbm_thd1 */
	{ 0x10bc, 16, 25,     80 }, /* ma_gbm_thd2 */
	{ 0x10bc,  0,  9,    112 }, /* ma_gbm_thd3 */
	{ 0x10c0, 28, 28,      1 }, /* mtfilten_gmd */
	{ 0x10c0, 20, 25,     28 }, /* mtth3_gmd */
	{ 0x10c0, 12, 17,     20 }, /* mtth2_gmd */
	{ 0x10c0,  4,  8,     12 }, /* mtth1_gmd */
	{ 0x10c0,  0,  3,      4 }, /* mtth0_gmd */
	{ 0x10c4, 12, 19,     96 }, /* k_mag_gmd */
	{ 0x10c4,  4, 10,     22 }, /* k_difh_gmd */
	{ 0x10c4,  0,  3,      4 }, /* k_maxmag_gmd */
	{ 0x10fc, 24, 27,      3 }, /* k_rgdifycore */
	{ 0x10fc, 14, 23,   1023 }, /* g_rgdifycore */
	{ 0x10fc, 10, 13,      7 }, /* core_rgdify */
	{ 0x10fc,  0,  9,    511 }, /* lmt_rgdify */
	{ 0x1100, 23, 25,      1 }, /* coef_sadlpf */
	{ 0x1100, 15, 21,     32 }, /* kmv_rgsad */
	{ 0x1100,  9, 14,     63 }, /* k_tpdif_rgsad */
	{ 0x1100,  0,  8,    255 }, /* g_tpdif_rgsad */
	{ 0x1104, 19, 26,     48 }, /* thmag_rgmv */
	{ 0x1104, 10, 18,    128 }, /* th_saddif_rgmv */
	{ 0x1104,  0,  9,    256 }, /* th_0mvsad_rgmv */
	{ 0x1108, 10, 13,      3 }, /* core_mag_rg */
	{ 0x1108,  0,  9,    255 }, /* lmt_mag_rg */
	{ 0x110c, 21, 25,      2 }, /* core_mv_rgmvls */
	{ 0x110c, 16, 20,     16 }, /* k_mv_rgmvls */
	{ 0x110c,  9, 15,    -64 }, /* core_mag_rgmvls */
	{ 0x110c,  5,  8,     12 }, /* k_mag_rgmvls */
	{ 0x110c,  1,  4,      8 }, /* th_mvadj_rgmvls */
	{ 0x110c,  0,  0,      1 }, /* en_mvadj_rgmvls */
	{ 0x1110, 15, 18,      8 }, /* k_sad_rgls */
	{ 0x1110,  9, 14,     40 }, /* th_mag_rgls */
	{ 0x1110,  4,  8,      8 }, /* th_sad_rgls */
	{ 0x1110,  0,  3,      8 }, /* k_sadcore_rgmv */
	{ 0x1114,  8,  8,      0 }, /* force_mven */
	{ 0x1114,  0,  7,      0 }, /* force_mvx */
	{ 0x1118, 16, 18,      6 }, /* th_blkmvx_mvdlt */
	{ 0x1118, 12, 15,      4 }, /* th_rgmvx_mvdlt */
	{ 0x1118,  8, 11,      8 }, /* th_ls_mvdlt */
	{ 0x1118,  4,  7,      1 }, /* th_vblkdist_mvdlt */
	{ 0x1118,  0,  3,      4 }, /* th_hblkdist_mvdlt */
	{ 0x111c, 23, 25,      2 }, /* k_sadcore_mvdlt */
	{ 0x111c, 18, 22,     12 }, /* th_mag_mvdlt */
	{ 0x111c, 12, 17,     16 }, /* g_mag_mvdlt */
	{ 0x111c,  5, 11,     96 }, /* thl_sad_mvdlt */
	{ 0x111c,  0,  4,     16 }, /* thh_sad_mvdlt */
	{ 0x1120, 20, 24,     20 }, /* k_rglsw */
	{ 0x1120, 14, 19,     32 }, /* k_simimvw */
	{ 0x1120,  8, 13,     15 }, /* gh_core_simimv */
	{ 0x1120,  5,  7,      0 }, /* gl_core_simimv */
	{ 0x1120,  0,  4,      8 }, /* k_core_simimv */
	{ 0x1124, 23, 27,     16 }, /* k_core_vsaddif */
	{ 0x1124, 19, 22,      8 }, /* k_rgsadadj_mcw */
	{ 0x1124, 10, 18,     64 }, /* core_rgsadadj_mcw */
	{ 0x1124,  4,  9,     16 }, /* k_mvy_mcw */
	{ 0x1124,  1,  3,      3 }, /* core_mvy_mcw */
	{ 0x1124,  0,  0,      1 }, /* rgtb_en_mcw */
	{ 0x1128, 19, 26,     24 }, /* core_rgmag_mcw */
	{ 0x1128, 18, 18,      0 }, /* mode_rgysad_mcw */
	{ 0x1128, 12, 17,      8 }, /* k_vsaddifw */
	{ 0x1128,  5, 11,     64 }, /* gh_core_vsad_dif */
	{ 0x1128,  0,  4,      8 }, /* gl_core_vsaddif */
	{ 0x112c, 18, 25,     64 }, /* g0_rgmag_mcw */
	{ 0x112c,  9, 17,    256 }, /* k0_rgmag_mcw */
	{ 0x112c,  0,  8,     64 }, /* x0_rgmag_mcw */
	{ 0x1130, 17, 26,    512 }, /* x0_rgsad_mcw */
	{ 0x1130,  9, 16,     96 }, /* core_rgsad_mcw */
	{ 0x1130,  0,  8,    320 }, /* k1_rgmag_mcw */
	{ 0x1134, 17, 25,    128 }, /* k1_rgsad_mcw */
	{ 0x1134,  9, 16,    255 }, /* g0_rgsad_mcw */
	{ 0x1134,  0,  8,    160 }, /* k0_rgsad_mcw */
	{ 0x1138, 24, 29,     24 }, /* k_rgsad_mcw */
	{ 0x1138, 16, 23,    122 }, /* x_rgsad_mcw */
	{ 0x1138,  8, 15,     64 }, /* k0_smrg_mcw */
	{ 0x1138,  0,  7,     16 }, /* x0_smrg_mcw */
	{ 0x113c, 23, 29,     32 }, /* k1_tpmvdist_mcw */
	{ 0x113c, 15, 22,      0 }, /* g0_tpmvdist_mcw */
	{ 0x113c,  8, 14,     64 }, /* k0_tpmvdist_mcw */
	{ 0x113c,  0,  7,    255 }, /* x0_tpmvdist_mcw */
	{ 0x1140, 11, 13,      2 }, /* k_core_tpmvdist_mcw */
	{ 0x1140,  8, 10,      2 }, /* b_core_tpmvdist_mcw */
	{ 0x1140,  4,  7,      4 }, /* k_avgmvdist_mcw */
	{ 0x1140,  0,  3,      4 }, /* k_minmvdist_mcw */
	{ 0x1144, 27, 30,     15 }, /* k_tbdif_mcw */
	{ 0x1144, 23, 26,      8 }, /* k0_max_mag_mcw */
	{ 0x1144, 19, 22,      8 }, /* k1_max_mag_mcw */
	{ 0x1144, 15, 18,      8 }, /* k_max_dif_mcw */
	{ 0x1144, 11, 14,      8 }, /* k_max_core_mcw */
	{ 0x1144,  5, 10,     32 }, /* k_difvcore_mcw */
	{ 0x1144,  0,  4,     18 }, /* k_difhcore_mcw */
	{ 0x1148, 21, 24,      6 }, /* k1_mag_wnd_mcw */
	{ 0x1148, 15, 20,     24 }, /* g0_mag_wnd_mcw */
	{ 0x1148, 11, 14,      6 }, /* k0_mag_wnd_mcw */
	{ 0x1148,  4, 10,     32 }, /* x0_mag_wnd_mcw */
	{ 0x1148,  0,  3,      0 }, /* k_tbmag_mcw */
	{ 0x114c, 21, 28,     16 }, /* g0_sad_wnd_mcw */
	{ 0x114c, 16, 20,     16 }, /* k0_sad_wnd_mcw */
	{ 0x114c,  9, 15,      8 }, /* x0_sad_wnd_mcw */
	{ 0x114c,  0,  8,    288 }, /* g1_mag_wnd_mcw */
	{ 0x1150,  5, 13,    288 }, /* g1_sad_wnd_mcw */
	{ 0x1150,  0,  4,     16 }, /* k1_sad_wnd_mcw */
	{ 0x1154, 24, 26,      0 }, /* b_hvdif_dw */
	{ 0x1154, 20, 22,      0 }, /* b_bhvdif_dw */
	{ 0x1154, 12, 18,     64 }, /* k_bhvdif_dw */
	{ 0x1154,  8, 11,      5 }, /* core_bhvdif_dw */
	{ 0x1154,  4,  7,     15 }, /* gain_lpf_dw */
	{ 0x1154,  0,  3,     12 }, /* k_max_hvdif_dw */
	{ 0x1158, 20, 25,     56 }, /* b_mv_dw */
	{ 0x1158, 16, 19,     -2 }, /* core_mv_dw */
	{ 0x1158,  8, 12,     20 }, /* k_difv_dw */
	{ 0x1158,  0,  4,     16 }, /* core_hvdif_dw */
	{ 0x115c, 25, 30,      8 }, /* k1_hvdif_dw */
	{ 0x115c, 16, 24,    128 }, /* g0_hvdif_dw */
	{ 0x115c,  9, 14,      8 }, /* k0_hvdif_dw */
	{ 0x115c,  0,  8,    256 }, /* x0_hvdif_dw */
	{ 0x1160, 24, 31,     64 }, /* k1_mv_dw */
	{ 0x1160, 16, 21,     32 }, /* g0_mv_dw */
	{ 0x1160,  8, 13,     16 }, /* k0_mv_dw */
	{ 0x1160,  0,  4,      8 }, /* x0_mv_dw */
	{ 0x1164, 24, 29,     32 }, /* k1_mt_dw */
	{ 0x1164, 16, 23,     64 }, /* g0_mt_dw */
	{ 0x1164,  8, 13,     32 }, /* k0_mt_dw */
	{ 0x1164,  0,  6,     32 }, /* x0_mt_dw */
	{ 0x1168, 24, 31,      0 }, /* b_mt_dw */
	{ 0x1168, 12, 16,     20 }, /* k1_mv_mt */
	{ 0x1168,  8,  9,      1 }, /* x0_mv_mt */
	{ 0x1168,  0,  4,     31 }, /* g0_mv_mt */
	{ 0x116c, 28, 28,      1 }, /* mclpf_mode */
	{ 0x116c, 20, 26,      8 }, /* k_pxlmag_mcw */
	{ 0x116c, 16, 18,      2 }, /* x_pxlmag_mcw */
	{ 0x116c, 15, 15,      0 }, /* rs_pxlmag_mcw */
	{ 0x116c, 10, 14,     16 }, /* gain_mclpfh */
	{ 0x116c,  5,  9,     16 }, /* gain_dn_mclpfv */
	{ 0x116c,  0,  4,     16 }, /* gain_up_mclpfv */
	{ 0x1170,  0,  5,      0 }, /* g_pxlmag_mcw */
	{ 0x1174,  5, 11,     15 }, /* k_c_vertw */
	{ 0x1174,  0,  4,      8 }, /* k_y_vertw */
	{ 0x1178, 25, 29,      0 }, /* k_fstmt_mc */
	{ 0x1178, 20, 24,      0 }, /* x_fstmt_mc */
	{ 0x1178, 14, 19,     16 }, /* k1_mv_mc */
	{ 0x1178, 11, 13,      2 }, /* x0_mv_mc */
	{ 0x1178,  8, 10,      4 }, /* bdv_mcpos */
	{ 0x1178,  5,  7,      4 }, /* bdh_mcpos */
	{ 0x1178,  0,  4,      8 }, /* k_delta */
	{ 0x117c, 26, 30,     48 }, /* k_hfcore_mc */
	{ 0x117c, 21, 25,      8 }, /* x_hfcore_mc */
	{ 0x117c, 15, 20,      0 }, /* g_slmt_mc */
	{ 0x117c, 10, 14,      0 }, /* k_slmt_mc */
	{ 0x117c,  5,  9,      0 }, /* x_slmt_mc */
	{ 0x117c,  0,  4,      0 }, /* g_fstmt_mc */
	{ 0x1180, 18, 29,      0 }, /* r0_mc */
	{ 0x1180,  6, 17,      0 }, /* c0_mc */
	{ 0x1180,  0,  5,      0 }, /* g_hfcore_mc */
	{ 0x1184, 24, 29,     32 }, /* mcmvrange */
	{ 0x1184, 12, 23,   4095 }, /* r1_mc */
	{ 0x1184,  0, 11,   4095 }, /* c1_mc */
	{ 0x1188,  6, 12,     48 }, /* k_frcount_mc */
	{ 0x1188,  1,  5,      8 }, /* x_frcount_mc */
	{ 0x1188,  0,  0,      0 }, /* scenechange_mc */
	{ 0x118c, 24, 31,      0 }, /* mcendc */
	{ 0x118c, 16, 23,      0 }, /* mcendr */
	{ 0x118c,  8, 15,      0 }, /* mcstartc */
	{ 0x118c,  0,  7,      0 }, /* mcstartr */
	{ 0x1190, 18, 23,      2 }, /* movegain */
	{ 0x1190, 12, 17,      4 }, /* movecorig */
	{ 0x1190,  6, 11,      2 }, /* movethdl */
	{ 0x1190,  0,  5,     12 }, /* movethdh */
	{ 0x1194, 15, 15,      0 }, /* mc_numt_blden */
	{ 0x1194,  7, 14,     32 }, /* numt_gain */
	{ 0x1194,  1,  6,      0 }, /* numt_coring */
	{ 0x1194,  0,  0,      0 }, /* numt_lpf_en */
	{ 0x1198,  5, 16,      0 }, /* demo_border */
	{ 0x1198,  3,  4,      2 }, /* demo_mode_r */
	{ 0x1198,  1,  2,      2 }, /* demo_mode_l */
	{ 0x1198,  0,  0,      0 }, /* demo_en */
};

int histb_vpss_dei(struct histb_vpss *vpss,
		   const struct histb_vpss_dei_frame *frame,
		   dma_addr_t output_dma)
{
	u32 stride = frame ? frame->stride : 0;
	u32 field_height;
	u32 ctrl;
	unsigned int i;
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

	field_height = frame->height / 2;

	mutex_lock(&vpss->lock);

	ret = pm_runtime_resume_and_get(vpss->dev);
	if (ret < 0)
		goto unlock;

	/*
	 * Everything goes into the node image, not into the register window:
	 * the engine reads its register set from the descriptor that
	 * VPSS_NEXT points at, which is how the scaler path works too.
	 * Writing the live registers instead leaves NEXT at zero, the engine
	 * finds no node to run and the job times out - measured as
	 * diesta = 0, intstat = 0, next = 0 with every other field correct.
	 */
	memset(vpss->node, 0, HISTB_VPSS_NODE_SIZE);

	ctrl = histb_vpss_node_read(vpss, HISTB_VPSS_CTRL);
	ctrl |= HISTB_VPSS_CTRL_DEI_EN | HISTB_VPSS_CTRL_OUTPUT_EN;
	if (!frame->top_field_first)
		ctrl |= HISTB_VPSS_CTRL_BFIELD_FIRST;
	else
		ctrl &= ~HISTB_VPSS_CTRL_BFIELD_FIRST;
	histb_vpss_node_write(vpss, HISTB_VPSS_CTRL, ctrl);

	/* Output of the de-interlaced frame. */
	histb_vpss_node_write(vpss, HISTB_VPSS_LB_Y_ADDR,
			      lower_32_bits(output_dma));
	histb_vpss_node_write(vpss, HISTB_VPSS_LB_C_ADDR,
			      lower_32_bits(output_dma + stride * frame->height));
	histb_vpss_node_write(vpss, HISTB_VPSS_LB_STRIDE, stride);
	histb_vpss_node_write(vpss, HISTB_VPSS_IMG_SIZE,
			      (frame->height - 1) << 16 | (frame->width - 1));

	/* The four fields, in the same group-of-four layout as the registers. */
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_REF_CTRL,
			      frame->tile ? HISTB_VPSS_DEI_TILE_FORMAT : 0);
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_REFYADDR,
			      lower_32_bits(frame->ref_dma));
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_REFCADDR,
			      lower_32_bits(frame->ref_dma + stride * field_height));
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_REFSTRIDE,
			      stride | stride << 16);

	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_CUR_CTRL,
			      frame->tile ? HISTB_VPSS_DEI_TILE_FORMAT : 0);
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_CURYADDR,
			      lower_32_bits(frame->cur_dma));
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_CURCADDR,
			      lower_32_bits(frame->cur_dma + stride * field_height));
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_CURSTRIDE,
			      stride | stride << 16);

	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_NXT1_CTRL,
			      frame->tile ? HISTB_VPSS_DEI_TILE_FORMAT : 0);
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_NXT1YADDR,
			      lower_32_bits(frame->nxt1_dma));
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_NXT1CADDR,
			      lower_32_bits(frame->nxt1_dma + stride * field_height));
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_NXT1STRIDE,
			      stride | stride << 16);

	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_NXT2_CTRL,
			      frame->tile ? HISTB_VPSS_DEI_TILE_FORMAT : 0);
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_NXT2YADDR,
			      lower_32_bits(frame->nxt2_dma));
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_NXT2CADDR,
			      lower_32_bits(frame->nxt2_dma + stride * field_height));
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_NXT2STRIDE,
			      stride | stride << 16);

	/*
	 * Bring the node up the way the vendor's field-node builder does
	 * (VPSS_HAL_SetFieldNode in vpss_hal_3798cv200.c), because the block
	 * checks state this driver was previously not setting at all:
	 * pixel format and bit width in CTRL2, crop enable in CTRL3, and the
	 * explicit disable of the replay/TNR/SNR stages in CTRL.  Writing CTRL
	 * from scratch without those bits leaves the pipeline in a state the
	 * de-interlacer will not start from.
	 */
	histb_vpss_node_write(vpss, HISTB_VPSS_MISC, HISTB_VPSS_MISC_DEFAULT);
	histb_vpss_node_write(vpss, HISTB_VPSS_INT_MASK, 0xff);
	/* CTRL2: input pixel format (NV12) and read bit width. */
	histb_vpss_node_write(vpss, HISTB_VPSS_CTRL2,
			      frame->ten_bit ? HISTB_VPSS_CTRL2_INPUT_10BIT : 0);
	/* CTRL3: no input crop. */
	histb_vpss_node_write(vpss, HISTB_VPSS_CTRL3, 0);

	/*
	 * Apply the vendor's de-interlacer defaults.  These are not invented:
	 * every row comes from the CV200 table in
	 * pq_hal_table_default.c (HI_PQ_MODULE_DEI), which is what the BSP
	 * loads when no binary parameter file is present.  The block will not
	 * produce a result without them - with every other field correct it
	 * sat at diesta = 0xb7bab9b9 and never raised a completion interrupt.
	 *
	 * Each row is a bitfield: read the 32-bit word, splice in the value
	 * and write it back.  Several registers carry many fields, so they
	 * must accumulate rather than be overwritten.
	 */
	for (i = 0; i < ARRAY_SIZE(histb_vpss_dei_defaults); i++) {
		const struct histb_vpss_dei_default *d =
			&histb_vpss_dei_defaults[i];
		u32 mask = GENMASK(d->msb, d->lsb);
		u32 old = histb_vpss_node_read(vpss, d->reg);

		histb_vpss_node_write(vpss, d->reg,
				      (old & ~mask) |
				      (((u32)d->value << d->lsb) & mask));
	}

	/* The block reads its own tuning block from inside the register map. */
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_ADDR,
			      lower_32_bits(vpss->regs_phys +
					    HISTB_VPSS_DEI_PARAM_BASE));

	histb_vpss_node_write(vpss, HISTB_VPSS_INT_MASK, HISTB_VPSS_INT_ALL);
	histb_vpss_node_write(vpss, HISTB_VPSS_NEXT, 0);

	reinit_completion(&vpss->completion);
	vpss->irq_state = 0;
	writel(HISTB_VPSS_INT_ALL, vpss->regs + HISTB_VPSS_INT_CLEAR);
	writel(HISTB_VPSS_MISC_DEFAULT, vpss->regs + HISTB_VPSS_MISC);
	writel(lower_32_bits(vpss->node_dma), vpss->regs + HISTB_VPSS_NEXT);
	wmb();
	writel(1, vpss->regs + HISTB_VPSS_START);

	timeout = wait_for_completion_timeout(&vpss->completion,
					      msecs_to_jiffies(500));
	if (!timeout)
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
EXPORT_SYMBOL_GPL(histb_vpss_dei);

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
	{
		struct resource *res = platform_get_resource(pdev,
							     IORESOURCE_MEM, 0);

		vpss->regs_phys = res ? res->start : 0;
	}
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
