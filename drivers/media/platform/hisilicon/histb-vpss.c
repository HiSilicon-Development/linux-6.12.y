// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3798CV200 video post-processing subsystem
 *
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 */

#include <linux/build_bug.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/overflow.h>
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
#define HISTB_VPSS_REE_Y_ADDR		0x188
#define HISTB_VPSS_REE_C_ADDR		0x18c
#define HISTB_VPSS_REE_STRIDE		0x190
#define HISTB_VPSS_PRJV_CUR_ADDR	0x194
#define HISTB_VPSS_PRJH_CUR_ADDR	0x198
#define HISTB_VPSS_PRJCUR_STRIDE	0x19c
#define HISTB_VPSS_RGMV_CUR_ADDR	0x1a0
#define HISTB_VPSS_RGMV_NX1_ADDR	0x1a4
#define HISTB_VPSS_RGMV_STRIDE		0x1a8
#define HISTB_VPSS_BLKMV_CUR_ADDR	0x1ac
#define HISTB_VPSS_BLKMV_REF_ADDR	0x1b0
#define HISTB_VPSS_BLKMV_STRIDE	0x1b4
#define HISTB_VPSS_CUE_Y_ADDR		0x1bc
#define HISTB_VPSS_CUE_C_ADDR		0x1c0
#define HISTB_VPSS_CUE_STRIDE		0x1c4
#define HISTB_VPSS_PRJV_NX2_ADDR	0x1c8
#define HISTB_VPSS_PRJH_NX2_ADDR	0x1cc
#define HISTB_VPSS_PRJNX2_STRIDE	0x1d0
#define HISTB_VPSS_RGMV_NX2_ADDR	0x1d4
#define HISTB_VPSS_RGMVNX2_STRIDE	0x1d8
#define HISTB_VPSS_BLKMV_NX1_ADDR	0x1dc
#define HISTB_VPSS_BLKMVNX1_STRIDE	0x1e0
#define HISTB_VPSS_CTRL_MCDI_EN		BIT(8)
#define HISTB_VPSS_CTRL_MEDS_EN		BIT(9)
#define HISTB_VPSS_STT_W_ADDR		0x170
#define HISTB_VPSS_TNR_ADDR		0x210
#define HISTB_VPSS_TNR_CLUT_ADDR		0x214
#define HISTB_VPSS_VHD0_CTRL		0x150
#define HISTB_VPSS_VHD0_SIZE		0x154
#define HISTB_VPSS_VHD0_Y_ADDR		0x158
#define HISTB_VPSS_VHD0_C_ADDR		0x15c
#define HISTB_VPSS_VHD0_STRIDE		0x160
#define HISTB_VPSS_CTRL2_VHD0_FORMAT	GENMASK(15, 12)
/* The hardware uses one 4:2:0 code for NV12/NV21; UV invert selects order. */
#define HISTB_VPSS_FMT_420		0
/*
 * DEI tuning block contents, taken verbatim from the CV200 default PQ table
 * (pq_hal_table_default.c, HI_PQ_MODULE_DEI):
 *   0x1000 die_l_mode=0 die_c_mode=0 ma_only=0 mc_only=0 ...
 *   0x1004 chroma_mf_offset=8 rec_mode_en=1 motion_iir_en=1 frame_motion_smooth_en=1
 *   0x1008 ver_min_inten=-320 dir_inten_ver=2
 *   0x100c range_scale=2
 *   0x1010 ck1_gain=8 ck1_range_gain=2 ck1_max_range=30
 */
#define HISTB_VPSS_NODE_ID		0x0f8
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
#define HISTB_VPSS_DEI_PARAM_BASE	0x1000
/* VPSS_DIESTA: the de-interlacer's state, height and chroma counters. */
#define HISTB_VPSS_DIESTA		0x1028
#define HISTB_VPSS_DIECTRL		0x1000
#define HISTB_VPSS_DIE_RST		BIT(17)	/* DIECTRL.die_rst */
/* Port path registers VPSS_HAL_SetPortCfg() always programs. */
#define HISTB_VPSS_VHD0CROP_POS		0x024
#define HISTB_VPSS_VHD0CROP_SIZE	0x028
#define HISTB_VPSS_VHD0LBA_DSIZE	0x030
#define HISTB_VPSS_VHD0LBA_VFPOS	0x034
#define HISTB_VPSS_VHD0LBA_BK		0x038
#define HISTB_VPSS_CTRL3_VHD0_CROP_EN	BIT(17)
#define HISTB_VPSS_CTRL3_VHD0_LBA_EN	BIT(20)
/* System registers the engine may or may not copy out of the node. */
#define HISTB_VPSS_PFCNT		0x310
#define HISTB_VPSS_MACCFG		0x318
#define HISTB_VPSS_FTCONFIG		0x320
#define HISTB_VPSS_BUSCTRL		0x344
#define HISTB_VPSS_STT_WB_SIZE		(32 * 1024 * 5)

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
#define HISTB_VPSS_DEI_TILE_FORMAT	BIT(4)	/* VPSS_NXT2_CTRL.nxt2_tile_format */
#define HISTB_VPSS_DEI_DCMP_EN		BIT(31)	/* nxt2_dcmp_en: must stay clear */
#define HISTB_VPSS_CTRL2_IN_FORMAT	GENMASK(20, 16)
#define HISTB_VPSS_CTRL2_IN_PIX_BITW	BIT(21)
#define HISTB_VPSS_CTRL2_REF_NXT_BITW	BIT(22)
#define HISTB_VPSS_CTRL2_RFR_PIX_BITW	BIT(23)
#define HISTB_VPSS_CTRL3_PRE_VFIR_MODE	GENMASK(5, 4)
#define HISTB_VPSS_CTRL3_PRE_VFIR_EN	BIT(6)
#define HISTB_VPSS_CTRL3_IN_CROP_EN	BIT(16)
#define HISTB_VPSS_REFSIZE		0x014
#define HISTB_VPSS_RFR_Y_ADDR		0x144
#define HISTB_VPSS_RFR_C_ADDR		0x148
#define HISTB_VPSS_RFR_STRIDE		0x14c
#define HISTB_VPSS_CTRL_PROT		GENMASK(24, 23)	/* prot is 2 bits */
#define HISTB_VPSS_CTRL_IMG_PRO_MODE	GENMASK(28, 27)
#define HISTB_VPSS_CTRL_IGBM_EN		BIT(26)
#define HISTB_VPSS_CTRL_BFIELD		BIT(31)
#define HISTB_VPSS_DEI_ADDR		0x258
/*
 * Algorithm parameter block addresses.  VPSS_HAL_SetAlgParaAddr() in the
 * BSP programs both of these for every node it builds, so every node that
 * enables dbm_en carries a valid address; the address it writes is the
 * offset of the block's first register inside the node image, minus one
 * word, because the block fetcher counts the register that holds the
 * address as the first word of the block.
 */
#define HISTB_VPSS_SNR_ADDR		0x218
#define HISTB_VPSS_DBM_ADDR		0x21c
#define HISTB_VPSS_DB_PARA		0x2500
#define HISTB_VPSS_SNR_PARA		0x3000
#define HISTB_VPSS_RAW_INT		0x30c
#define HISTB_VPSS_EOF_CNT		0x330
/*
 * VPSS_DIECTRL, at offset 0x1000 inside the node image; the block is handed
 * that offset through VPSS_DEI_ADDR and fetches the tuning block itself.
 * U_VPSS_DIECTRL:
 *   edge_smooth_ratio[15:8] edge_smooth_en[20] ma_only[21] mc_only[22]
 *   die_c_mode[25:24] die_l_mode[27:26] die_out_sel_c[28] die_out_sel_l[29]
 */

#define HISTB_VPSS_ST_RD_ADDR		0x164
#define HISTB_VPSS_ST_WR_ADDR		0x168
#define HISTB_VPSS_ST_STRIDE		0x16c
#define HISTB_VPSS_DEI_ST_SLOTS		3
#define HISTB_VPSS_CTRL_TNR_EN		BIT(18)
#define HISTB_VPSS_CTRL_SNR_EN		BIT(17)
#define HISTB_VPSS_CTRL_RFR_EN		BIT(19)
#define HISTB_VPSS_CTRL_DBM_EN		BIT(11)
#define HISTB_VPSS_DEI_IN_FMT_NV12_TILE	3

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
#define HISTB_VPSS_INPUT_TILE		BIT(4)
#define HISTB_VPSS_OUTPUT_10BIT		BIT(6)
#define HISTB_VPSS_OUTPUT_DITHER	BIT(7)
#define HISTB_VPSS_OUTPUT_UV_INVERT	BIT(8)
#define HISTB_VPSS_OUTPUT_DITHER_ROUND	BIT(10)

#define HISTB_VPSS_ZME_H_RATIO_PRECISION	BIT(20)
#define HISTB_VPSS_ZME_V_RATIO_PRECISION	BIT(12)
#define HISTB_VPSS_ZME_MAX_REDUCTION	16
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
#define HISTB_VPSS_ZME_HL_UP_OFFSET	0x000
#define HISTB_VPSS_ZME_HL_EXACT_OFFSET	0x100
#define HISTB_VPSS_ZME_HL_MID_OFFSET	0x200
#define HISTB_VPSS_ZME_HL_LOW_OFFSET	0x300
#define HISTB_VPSS_ZME_4T_EXACT_OFFSET	0x400
#define HISTB_VPSS_ZME_4T_MID_OFFSET	0x500
#define HISTB_VPSS_ZME_4T_LOW_OFFSET	0x600
#define HISTB_VPSS_ZME_HL8_BYTES	(17 * 3 * sizeof(__le32))
#define HISTB_VPSS_ZME_4T_BYTES		(17 * 2 * sizeof(__le32))
#define HISTB_VPSS_MISC_DEFAULT	0x03006466
#define HISTB_VPSS_JOB_TIMEOUT_MS	1000
#define HISTB_VPSS_CORE_RATE		400000000UL

static_assert(HISTB_VPSS_ZME_HL8_BYTES <= HISTB_VPSS_ZME_COEF_SLOT_SIZE);
static_assert(HISTB_VPSS_ZME_4T_BYTES <= HISTB_VPSS_ZME_COEF_SLOT_SIZE);
static_assert(HISTB_VPSS_ZME_4T_LOW_OFFSET +
	      HISTB_VPSS_ZME_COEF_SLOT_SIZE <= HISTB_VPSS_ZME_COEF_SIZE);

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

	/* Motion-statistics ring the de-interlacer reads from and writes to. */
	void *dei_st_cpu;
	dma_addr_t dei_st_dma;
	size_t dei_st_size;
	u32 dei_st_seq;

	/* Statistics write-back target: VPSS_STT_W_ADDR (BSP: u32stt_w_phy_addr). */
	void *stt_cpu;
	dma_addr_t stt_dma;
	size_t stt_size;

	/* Serializes the single hardware pipeline and its staging buffer. */
	struct mutex lock;
	struct completion completion;
	u32 irq_state;
};

/* CV200 VPSS FIR tables: 17 stored phases; hardware mirrors the other 15. */
static const s16 histb_vpss_zme_hl8_up[17][8] = {
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

static const s16 histb_vpss_zme_hl8_exact[17][8] = {
	{ 0, 0, 0, 511, 0, 0, 0, 0 },
	{ -1, 3, -12, 511, 14, -4, 1, 0 },
	{ -2, 6, -23, 509, 28, -8, 2, 0 },
	{ -2, 9, -33, 503, 44, -12, 3, 0 },
	{ -3, 11, -41, 496, 61, -16, 4, 0 },
	{ -3, 13, -48, 488, 79, -21, 5, -1 },
	{ -3, 14, -54, 477, 98, -25, 7, -2 },
	{ -4, 16, -59, 465, 118, -30, 8, -2 },
	{ -4, 17, -63, 451, 138, -35, 9, -1 },
	{ -4, 18, -66, 437, 158, -39, 10, -2 },
	{ -4, 18, -68, 421, 180, -44, 11, -2 },
	{ -4, 18, -69, 404, 201, -48, 13, -3 },
	{ -4, 18, -70, 386, 222, -52, 14, -2 },
	{ -4, 18, -70, 368, 244, -56, 15, -3 },
	{ -4, 18, -69, 348, 265, -59, 16, -3 },
	{ -4, 18, -67, 329, 286, -63, 16, -3 },
	{ -3, 17, -65, 307, 307, -65, 17, -3 },
};

static const s16 histb_vpss_zme_hl8_mid[17][8] = {
	{ -16, 0, 145, 254, 145, 0, -16, 0 },
	{ -16, -2, 140, 253, 151, 3, -17, 0 },
	{ -15, -5, 135, 253, 157, 5, -18, 0 },
	{ -14, -7, 129, 252, 162, 8, -18, 0 },
	{ -13, -9, 123, 252, 167, 11, -19, 0 },
	{ -13, -11, 118, 250, 172, 15, -19, 0 },
	{ -12, -12, 112, 250, 177, 18, -20, -1 },
	{ -11, -14, 107, 247, 183, 21, -20, -1 },
	{ -10, -15, 101, 245, 188, 25, -21, -1 },
	{ -9, -16, 96, 243, 192, 29, -21, -2 },
	{ -8, -18, 90, 242, 197, 33, -22, -2 },
	{ -8, -19, 85, 239, 202, 37, -22, -2 },
	{ -7, -19, 80, 236, 206, 41, -22, -3 },
	{ -7, -20, 75, 233, 210, 46, -22, -3 },
	{ -6, -21, 69, 230, 215, 50, -22, -3 },
	{ -5, -21, 65, 226, 219, 55, -22, -5 },
	{ -5, -21, 60, 222, 222, 60, -21, -5 },
};

static const s16 histb_vpss_zme_hl8_low[17][8] = {
	{ -18, 18, 144, 226, 144, 19, -17, -4 },
	{ -17, 16, 139, 226, 148, 21, -17, -4 },
	{ -17, 13, 135, 227, 153, 24, -18, -5 },
	{ -17, 11, 131, 226, 157, 27, -18, -5 },
	{ -17, 9, 126, 225, 161, 30, -17, -5 },
	{ -16, 6, 122, 225, 165, 33, -17, -6 },
	{ -16, 4, 118, 224, 169, 37, -17, -7 },
	{ -16, 2, 113, 224, 173, 40, -17, -7 },
	{ -15, 0, 109, 222, 177, 43, -17, -7 },
	{ -15, -1, 104, 220, 181, 47, -16, -8 },
	{ -14, -3, 100, 218, 185, 51, -16, -9 },
	{ -14, -5, 96, 217, 188, 54, -15, -9 },
	{ -14, -6, 91, 214, 192, 58, -14, -9 },
	{ -13, -7, 87, 212, 195, 62, -14, -10 },
	{ -13, -9, 83, 210, 198, 66, -13, -10 },
	{ -12, -10, 79, 207, 201, 70, -12, -11 },
	{ -12, -11, 74, 205, 205, 74, -11, -12 },
};

static const s16 histb_vpss_zme_4t_exact[17][4] = {
	{ 0, 511, 0, 0 }, { -19, 511, 21, -1 },
	{ -37, 509, 42, -2 }, { -51, 504, 64, -5 },
	{ -64, 499, 86, -9 }, { -74, 492, 108, -14 },
	{ -82, 484, 129, -19 }, { -89, 474, 152, -25 },
	{ -94, 463, 174, -31 }, { -97, 451, 196, -38 },
	{ -98, 438, 217, -45 }, { -98, 424, 238, -52 },
	{ -98, 409, 260, -59 }, { -95, 392, 280, -65 },
	{ -92, 376, 300, -72 }, { -88, 358, 320, -78 },
	{ -83, 339, 339, -83 },
};

static const s16 histb_vpss_zme_4t_mid[17][4] = {
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

static const s16 histb_vpss_zme_4t_low[17][4] = {
	{ 120, 281, 120, -9 }, { 113, 281, 127, -9 },
	{ 106, 280, 134, -8 }, { 99, 279, 141, -7 },
	{ 92, 277, 148, -5 }, { 85, 275, 156, -4 },
	{ 79, 273, 162, -2 }, { 72, 270, 170, 0 },
	{ 66, 267, 177, 2 }, { 61, 263, 184, 4 },
	{ 56, 259, 191, 6 }, { 50, 255, 198, 9 },
	{ 44, 251, 205, 12 }, { 40, 246, 211, 15 },
	{ 34, 241, 218, 19 }, { 31, 235, 224, 22 },
	{ 26, 230, 230, 26 },
};

static u32 histb_vpss_node_read(struct histb_vpss *vpss, u32 reg)
{
	return le32_to_cpu(vpss->node[reg / sizeof(*vpss->node)]);
}

/*
 * Diagnostic / hypothesis sweep for the de-interlacer.  Each bit adds one
 * element the vendor's node builder writes and this driver does not, or one
 * probe that tells a stalled pipeline from one that never started:
 *
 *   0x01  CTRL.four_pix_en, which the detile node - the node this engine is
 *         known to complete - sets and the de-interlace node does not.
 *   0x02  VPSS_FTCONFIG.node_rst_en, the only enable bit in the top level
 *         that no BSP code writes and this driver never touched.
 *   0x04  reserved; the VHD0 crop is now part of the normal port setup.
 *   0x08  give VPSS_STT_W_ADDR a real buffer, as the BSP's
 *         u32stt_w_phy_addr always is, instead of the zero this driver wrote.
 *   0x10  arm the logic watchdog (VPSS_TIMEOUT) and unmask its interrupt, so a
 *         pipeline that starts and then stalls reports itself instead of
 *         sitting silent behind a 0xffffffff timeout.
 *   0x20  pulse DIECTRL.die_rst in the live window before the start.
 *   0x40  write sentinels into the sys/status words of the node and dump them
 *         back afterwards: it says which registers the engine's node load
 *         actually overwrites, i.e. whether diesta is hardware state at all.
 *   0x80  dump the same register set after a detile job succeeds, which is
 *         the reference for "the engine ran this node to completion".
 */
static unsigned int dei_opt;
module_param(dei_opt, uint, 0644);
MODULE_PARM_DESC(dei_opt, "VPSS de-interlacer hypothesis bits (0x1 four_pix, 0x2 node_rst_en, 0x8 stt write-back, 0x10 watchdog, 0x20 die_rst pulse, 0x40 node sentinel probe, 0x80 detile reference dump)");

#define HISTB_VPSS_OPT_FOUR_PIX		BIT(0)
#define HISTB_VPSS_OPT_NODE_RST_EN	BIT(1)
#define HISTB_VPSS_OPT_STT_WB		BIT(3)
#define HISTB_VPSS_OPT_WATCHDOG		BIT(4)
#define HISTB_VPSS_OPT_DIE_RST		BIT(5)
#define HISTB_VPSS_OPT_SENTINEL		BIT(6)
#define HISTB_VPSS_OPT_DETILE_REF	BIT(7)
#define HISTB_VPSS_OPT_LONG_WAIT	BIT(8)

static void histb_vpss_dump_state(struct histb_vpss *vpss, const char *tag)
{
	static const u32 words[] = {
		HISTB_VPSS_CTRL, HISTB_VPSS_CTRL2, HISTB_VPSS_CTRL3,
		HISTB_VPSS_REFSIZE, HISTB_VPSS_IMG_SIZE,
		HISTB_VPSS_VHD0CROP_POS, HISTB_VPSS_VHD0CROP_SIZE,
		HISTB_VPSS_VHD0LBA_DSIZE, HISTB_VPSS_VHD0LBA_VFPOS,
		HISTB_VPSS_VHD0LBA_BK,
		HISTB_VPSS_STT_W_ADDR, HISTB_VPSS_ST_RD_ADDR,
		HISTB_VPSS_ST_WR_ADDR, HISTB_VPSS_ST_STRIDE,
		HISTB_VPSS_NEXT, HISTB_VPSS_START, HISTB_VPSS_INT_STATE,
		HISTB_VPSS_RAW_INT, HISTB_VPSS_PFCNT, HISTB_VPSS_MACCFG,
		HISTB_VPSS_TIMEOUT, HISTB_VPSS_FTCONFIG, HISTB_VPSS_EOF_CNT,
		HISTB_VPSS_BUSCTRL, HISTB_VPSS_DIECTRL, HISTB_VPSS_DIESTA,
		HISTB_VPSS_DEI_ADDR, HISTB_VPSS_MISC, HISTB_VPSS_INT_MASK,
		HISTB_VPSS_OUTPUT_CTRL, HISTB_VPSS_OUTPUT_SIZE,
	};
	unsigned int i, q;

	dev_err(vpss->dev, "vpss[%s] state:\n", tag);
	for (i = 0; i < ARRAY_SIZE(words); i++)
		dev_err(vpss->dev, "vpss[%s] r[%03x]=%08x\n", tag,
			words[i], readl(vpss->regs + words[i]));
	for (q = 0x4000; q < 0x4100; q += 16)
		dev_err(vpss->dev, "vpss[%s] dbg[%04x]=%08x %08x %08x %08x\n",
			tag, q, readl(vpss->regs + q), readl(vpss->regs + q + 4),
			readl(vpss->regs + q + 8), readl(vpss->regs + q + 12));
}

static int histb_vpss_prepare_stt_wb(struct histb_vpss *vpss)
{
	if (vpss->stt_cpu)
		return 0;

	vpss->stt_cpu = dma_alloc_coherent(vpss->dev, HISTB_VPSS_STT_WB_SIZE,
					   &vpss->stt_dma, GFP_KERNEL);
	if (!vpss->stt_cpu)
		return -ENOMEM;
	vpss->stt_size = HISTB_VPSS_STT_WB_SIZE;
	memset(vpss->stt_cpu, 0, vpss->stt_size);
	if (upper_32_bits(vpss->stt_dma)) {
		dma_free_coherent(vpss->dev, vpss->stt_size, vpss->stt_cpu,
				  vpss->stt_dma);
		vpss->stt_cpu = NULL;
		vpss->stt_size = 0;
		return -ERANGE;
	}

	return 0;
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

static void histb_vpss_load_zme_hl8(struct histb_vpss *vpss, u32 offset,
				     const s16 coef[17][8])
{
	__le32 *dst = (__le32 *)((u8 *)vpss->zme_coef + offset);
	u32 phase;

	for (phase = 0; phase < 17; phase++) {
		const s16 *c = coef[phase];

		dst[phase * 3] =
			cpu_to_le32(histb_vpss_pack_10bit(c[0], c[1], c[2]));
		dst[phase * 3 + 1] =
			cpu_to_le32(histb_vpss_pack_10bit(c[3], c[4], c[5]));
		dst[phase * 3 + 2] =
			cpu_to_le32(histb_vpss_pack_10bit(c[6], c[7], 0));
	}
}

static void histb_vpss_load_zme_4t(struct histb_vpss *vpss, u32 offset,
				    const s16 coef[17][4])
{
	__le32 *dst = (__le32 *)((u8 *)vpss->zme_coef + offset);
	u32 phase;

	for (phase = 0; phase < 17; phase++) {
		const s16 *c = coef[phase];

		dst[phase * 2] = cpu_to_le32(histb_vpss_pack_16bit(c[0], c[1]));
		dst[phase * 2 + 1] =
			cpu_to_le32(histb_vpss_pack_16bit(c[2], c[3]));
	}
}

static void histb_vpss_load_zme_coefficients(struct histb_vpss *vpss)
{
	memset(vpss->zme_coef, 0, HISTB_VPSS_ZME_COEF_SIZE);
	histb_vpss_load_zme_hl8(vpss, HISTB_VPSS_ZME_HL_UP_OFFSET,
				 histb_vpss_zme_hl8_up);
	histb_vpss_load_zme_hl8(vpss, HISTB_VPSS_ZME_HL_EXACT_OFFSET,
				 histb_vpss_zme_hl8_exact);
	histb_vpss_load_zme_hl8(vpss, HISTB_VPSS_ZME_HL_MID_OFFSET,
				 histb_vpss_zme_hl8_mid);
	histb_vpss_load_zme_hl8(vpss, HISTB_VPSS_ZME_HL_LOW_OFFSET,
				 histb_vpss_zme_hl8_low);
	histb_vpss_load_zme_4t(vpss, HISTB_VPSS_ZME_4T_EXACT_OFFSET,
				histb_vpss_zme_4t_exact);
	histb_vpss_load_zme_4t(vpss, HISTB_VPSS_ZME_4T_MID_OFFSET,
				histb_vpss_zme_4t_mid);
	histb_vpss_load_zme_4t(vpss, HISTB_VPSS_ZME_4T_LOW_OFFSET,
				histb_vpss_zme_4t_low);
}

static u32 histb_vpss_zme_hl8_offset(u32 input, u32 output)
{
	if (output > input)
		return HISTB_VPSS_ZME_HL_UP_OFFSET;
	if (output == input)
		return HISTB_VPSS_ZME_HL_EXACT_OFFSET;
	if ((u64)output * 2 >= input)
		return HISTB_VPSS_ZME_HL_MID_OFFSET;

	return HISTB_VPSS_ZME_HL_LOW_OFFSET;
}

static u32 histb_vpss_zme_4t_offset(u32 input, u32 output)
{
	if (output == input)
		return HISTB_VPSS_ZME_4T_EXACT_OFFSET;
	if (output > input || (u64)output * 2 >= input)
		return HISTB_VPSS_ZME_4T_MID_OFFSET;

	return HISTB_VPSS_ZME_4T_LOW_OFFSET;
}

/*
 * The BSP's VPSS_HAL_SetPortCfg() runs VHD0 through the ZME for every port,
 * including a 1:1 de-interlace output.  For a 1:1 conversion the vendor PQ
 * algorithm enables the four scaler channels but puts every FIR in copy
 * mode: bZmeMdHL/HC/VL/VC are all false when the ratios are exactly 1.0 and
 * the phase is zero.  Enabling the FIRs here exposes their vertical lead-in
 * at the top of every bottom-field result.  Program the vendor copy path and
 * the full-frame crop/LBA window.
 */
static void histb_vpss_config_dei_port(struct histb_vpss *vpss, u32 width,
				       u32 height)
{
	u32 hratio = HISTB_VPSS_ZME_H_RATIO_PRECISION;
	u32 vratio = HISTB_VPSS_ZME_V_RATIO_PRECISION;
	u32 hl_offset = histb_vpss_zme_hl8_offset(width, width);
	u32 vc_offset = histb_vpss_zme_4t_offset(height, height);
	u32 hc_offset = histb_vpss_zme_4t_offset(width, width);
	u32 ctrl3 = histb_vpss_node_read(vpss, HISTB_VPSS_CTRL3);

	histb_vpss_node_write(vpss, HISTB_VPSS_ZME_OUTPUT_SIZE,
			      (height - 1) << 16 | (width - 1));
	histb_vpss_node_write(vpss, HISTB_VPSS_ZME_ADDR,
			      lower_32_bits(vpss->node_dma + HISTB_VPSS_ZME_HSP));
	histb_vpss_node_write(vpss, HISTB_VPSS_ZME_LH_COEF_ADDR,
			      lower_32_bits(vpss->zme_coef_dma + hl_offset));
	histb_vpss_node_write(vpss, HISTB_VPSS_ZME_LV_COEF_ADDR,
			      lower_32_bits(vpss->zme_coef_dma + vc_offset));
	histb_vpss_node_write(vpss, HISTB_VPSS_ZME_CH_COEF_ADDR,
			      lower_32_bits(vpss->zme_coef_dma + hc_offset));
	histb_vpss_node_write(vpss, HISTB_VPSS_ZME_CV_COEF_ADDR,
			      lower_32_bits(vpss->zme_coef_dma + vc_offset));
	histb_vpss_node_write(vpss, HISTB_VPSS_ZME_HSP,
			      HISTB_VPSS_ZME_H_LUMA_EN |
			      HISTB_VPSS_ZME_H_CHROMA_EN | hratio);
	histb_vpss_node_write(vpss, HISTB_VPSS_ZME_HL_OFFSET, 0);
	histb_vpss_node_write(vpss, HISTB_VPSS_ZME_HC_OFFSET, 0);
	histb_vpss_node_write(vpss, HISTB_VPSS_ZME_VSP,
			      HISTB_VPSS_ZME_V_LUMA_EN |
			      HISTB_VPSS_ZME_V_CHROMA_EN |
			      HISTB_VPSS_ZME_FMT_420_IN |
			      HISTB_VPSS_ZME_FMT_420_OUT);
	histb_vpss_node_write(vpss, HISTB_VPSS_ZME_VSR, vratio);
	histb_vpss_node_write(vpss, HISTB_VPSS_ZME_V_OFFSET, 0);

	histb_vpss_node_write(vpss, HISTB_VPSS_VHD0CROP_POS, 0);
	histb_vpss_node_write(vpss, HISTB_VPSS_VHD0CROP_SIZE,
			      (height - 1) << 16 | (width - 1));
	histb_vpss_node_write(vpss, HISTB_VPSS_VHD0LBA_DSIZE,
			      (height - 1) << 16 | (width - 1));
	histb_vpss_node_write(vpss, HISTB_VPSS_VHD0LBA_VFPOS, 0);
	histb_vpss_node_write(vpss, HISTB_VPSS_VHD0LBA_BK, 0x02080200);
	histb_vpss_node_write(vpss, HISTB_VPSS_CTRL3,
			      ctrl3 | HISTB_VPSS_CTRL3_ZME_EN |
			      HISTB_VPSS_CTRL3_VHD0_CROP_EN |
			      HISTB_VPSS_CTRL3_VHD0_LBA_EN);
}

/*
 * Assert and release the block's hardware reset - the same pulse
 * VPSS_HAL_SetClockEn(TRUE) performs through PERI_CRG60.vpss_srst_req before
 * every BSP job, so every BSP node is programmed into logic that has just
 * come out of reset.  Pulsing it before a de-interlace job does not make the
 * de-interlacer start: measured with the pulse on every DEI submission, the
 * engine still took the node and diesta stayed 0, so the block does not need
 * a reset release to be able to run.
 */
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

/*
 * CV200's Main10 tile surface is two NV12-sized regions: the high eight
 * bits, followed by the packed low two bits.  Keep the arithmetic here in
 * one place so the node builder and its caller agree on the addresses of the
 * low-bit planes, and so a malformed frame cannot wrap an address register.
 */
static int histb_vpss_input_size(const struct histb_vpss_frame *frame,
				 size_t *size)
{
	u32 input_height = frame->input_height ?: frame->height;
	u32 aligned_height;
	size_t high;
	size_t low;

	if (!frame->input_height_align)
		return -EINVAL;
	if (frame->input_ten_bit && frame->input_stride % 4)
		return -EINVAL;

	aligned_height = ALIGN(input_height, frame->input_height_align);
	if (check_mul_overflow((size_t)frame->input_stride, aligned_height,
			       &high))
		return -EOVERFLOW;
	if (check_mul_overflow(high, 3, &high))
		return -EOVERFLOW;
	high /= 2;
	if (!frame->input_ten_bit) {
		*size = high;
		return 0;
	}

	if (check_mul_overflow((size_t)frame->input_stride / 4,
			       aligned_height, &low))
		return -EOVERFLOW;
	if (check_mul_overflow(low, 3, &low))
		return -EOVERFLOW;
	low /= 2;
	if (check_add_overflow(high, low, size))
		return -EOVERFLOW;

	return 0;
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

/*
 * The de-interlacer keeps per-field motion statistics in DDR: it reads the
 * previous field's block through VPSS_STRADDR and writes the current one to
 * VPSS_STWADDR, at VPSS_STSTRIDE.  The vendor sizes one block as
 * ((width + 3) / 4 * 2) rounded up to 16 and then to the 64-byte Y stride,
 * times half the woven frame height, and rotates three of them
 * (vpss_sttinf.c: VPSS_STTINFO_CalDieBufSize / VPSS_STTINFO_DieInit).
 *
 * The BSP is called with the woven frame height and halves it there, so the
 * block holds one line of statistics per *field* line plus the chroma half
 * again: stride * field_height.  Sizing it at stride * (field_height / 2),
 * which is what this driver did, makes every block a third of the size the
 * block actually writes and pushes the third write past the end of the
 * allocation.
 */
static u32 histb_vpss_dei_st_stride(u32 width)
{
	return ALIGN(((width + 3) / 4 * 2 + 15) / 16 * 16, 64);
}

static int histb_vpss_prepare_dei_st(struct histb_vpss *vpss, u32 width,
				     u32 field_height)
{
	size_t slot = (size_t)histb_vpss_dei_st_stride(width) * field_height;
	size_t size = slot * HISTB_VPSS_DEI_ST_SLOTS;

	if (!slot)
		return -EINVAL;
	if (vpss->dei_st_cpu && vpss->dei_st_size >= size)
		return 0;

	if (vpss->dei_st_cpu)
		dma_free_coherent(vpss->dev, vpss->dei_st_size,
				  vpss->dei_st_cpu, vpss->dei_st_dma);
	vpss->dei_st_cpu = dma_alloc_coherent(vpss->dev, size,
					      &vpss->dei_st_dma, GFP_KERNEL);
	if (!vpss->dei_st_cpu) {
		vpss->dei_st_size = 0;
		return -ENOMEM;
	}
	vpss->dei_st_size = size;
	vpss->dei_st_seq = 0;
	memset(vpss->dei_st_cpu, 0, size);
	if (upper_32_bits(vpss->dei_st_dma)) {
		dma_free_coherent(vpss->dev, size, vpss->dei_st_cpu,
				  vpss->dei_st_dma);
		vpss->dei_st_cpu = NULL;
		vpss->dei_st_size = 0;
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
	/*
	 * This is the CV200 BSP's single-node UHD path in Linux form.  The
	 * vendor selects VPSS_HAL_NODE_UHD for any source wider than 1920 and
	 * then programs the ordinary port ZME in that same node.  FOUR_PIX,
	 * tile input, the low-bit plane and output dither are therefore one
	 * operation for 4K Main10 -> 1080p NV12; there is no second "UHD"
	 * scaler to submit here.
	 *
	 * Do not set DCMP for every UHD frame.  The BSP only enables it for
	 * NV12/NV21 compressed-tile surfaces and supplies the corresponding
	 * header addresses/stride.  VDEC currently exports an uncompressed
	 * tile surface, so leaving nxt2_dcmp_en clear is required.
	 */
	u32 input_width = frame->input_width ?: frame->width;
	u32 input_height = frame->input_height ?: frame->height;
	bool scale = input_width != frame->width ||
		input_height != frame->height;
	u32 aligned_height = ALIGN(input_height, frame->input_height_align);
	u32 input_chroma = frame->input_stride * aligned_height;
	u32 input_low_y = input_chroma * 3 / 2;
	u32 input_low_c = input_low_y + frame->input_stride / 4 * aligned_height;
	u32 output_ctrl = HISTB_VPSS_OUTPUT_UV_INVERT;

	memset(vpss->node, 0, HISTB_VPSS_NODE_SIZE);
	histb_vpss_node_write(vpss, HISTB_VPSS_CTRL,
			      HISTB_VPSS_CTRL_OUTPUT_EN |
				HISTB_VPSS_CTRL_FOUR_PIX);
	/* NV12/NV12_TILE is CV200 format code 0; do not inherit stale state. */
	histb_vpss_node_write(vpss, HISTB_VPSS_CTRL2,
			      frame->input_ten_bit ? HISTB_VPSS_CTRL2_IN_PIX_BITW : 0);
	/*
	 * CV200's SetPortCfg() always places VHD0 behind the port crop/LBA
	 * window.  The old Linux UHD node only enabled ZME, leaving these bits
	 * at reset values; that bypasses the same port state used by the BSP
	 * and makes the scaler path depend on stale hardware state.
	 */
	histb_vpss_node_write(vpss, HISTB_VPSS_CTRL3,
			      scale ? HISTB_VPSS_CTRL3_ZME_EN |
			      HISTB_VPSS_CTRL3_VHD0_CROP_EN |
			      HISTB_VPSS_CTRL3_VHD0_LBA_EN : 0);
	histb_vpss_node_write(vpss, HISTB_VPSS_IMG_SIZE,
			      (input_height - 1) << 16 | (input_width - 1));
	if (scale) {
		histb_vpss_node_write(vpss, HISTB_VPSS_VHD0CROP_POS, 0);
		histb_vpss_node_write(vpss, HISTB_VPSS_VHD0CROP_SIZE,
				      (input_height - 1) << 16 | (input_width - 1));
		histb_vpss_node_write(vpss, HISTB_VPSS_VHD0LBA_DSIZE,
				      (frame->height - 1) << 16 |
				      (frame->width - 1));
		histb_vpss_node_write(vpss, HISTB_VPSS_VHD0LBA_VFPOS, 0);
		histb_vpss_node_write(vpss, HISTB_VPSS_VHD0LBA_BK, 0x02080200);
	}
	if (scale) {
		u32 hratio = div_u64((u64)input_width *
				     HISTB_VPSS_ZME_H_RATIO_PRECISION,
				     frame->width);
		u32 vratio = div_u64((u64)input_height *
				     HISTB_VPSS_ZME_V_RATIO_PRECISION,
				     frame->height);
		u32 hl_offset = histb_vpss_zme_hl8_offset(input_width,
							     frame->width);
		u32 hc_offset = histb_vpss_zme_4t_offset(input_width,
							    frame->width);
		u32 v_offset = histb_vpss_zme_4t_offset(input_height,
							   frame->height);

		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_OUTPUT_SIZE,
				      (frame->height - 1) << 16 |
				      (frame->width - 1));
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_ADDR,
				      lower_32_bits(vpss->node_dma +
						    HISTB_VPSS_ZME_HSP));
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_LH_COEF_ADDR,
				      lower_32_bits(vpss->zme_coef_dma +
						    hl_offset));
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_LV_COEF_ADDR,
				      lower_32_bits(vpss->zme_coef_dma +
						    v_offset));
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_CH_COEF_ADDR,
				      lower_32_bits(vpss->zme_coef_dma +
						    hc_offset));
		histb_vpss_node_write(vpss, HISTB_VPSS_ZME_CV_COEF_ADDR,
				      lower_32_bits(vpss->zme_coef_dma +
						    v_offset));
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

/*
 * One field of the de-interlacer's four-deep history.
 *
 * A field lives inside a frame-sized surface.  VPSS_IMG_CopyAddr() advances
 * the bottom field's luma and chroma addresses by one pitch; the caller has
 * already applied that offset to @dma.  CTRL.bfield_mode still identifies the
 * parity of the field being processed.  The chroma plane sits a whole woven
 * frame height away from the luma address of the same parity.
 */
static void histb_vpss_dei_field(struct histb_vpss *vpss, u32 ctrl_off,
				 u32 y_off, u32 c_off, u32 stride_off,
				 dma_addr_t dma, u32 field_stride,
				 u32 chroma_off)
{
	histb_vpss_node_write(vpss, ctrl_off, 0 /* linear, not tiled */);
	histb_vpss_node_write(vpss, y_off, lower_32_bits(dma));
	histb_vpss_node_write(vpss, c_off, lower_32_bits(dma + chroma_off));
	histb_vpss_node_write(vpss, stride_off,
			      field_stride | field_stride << 16);
}

/*
 * Hardware de-interlace: four consecutive fields in, one progressive frame
 * out.  The block lives in the VPSS at DIECTRL (0x1000) and is switched on
 * with VPSS_CTRL bit 7; it shares the input field registers with the ZME
 * path this driver already uses, which is why it cannot run as an
 * independent node at the same time as a scaler job.
 *
 * The tuning words below are the vendor's default PQ register image, folded
 * the way the BSP applies it: PQ_TABLE_LoadPhyList() walks the CV200 table
 * (pq_hal_table_default.c) row by row - one row per bitfield - and
 * read-modify-writes the row's value into the node word at the row's own
 * offset, so the whole table lands in the register image in table order.
 * VPSS_REG_ResetAppReg() then copies that image over every node it hands to
 * the engine (memcpy(node, pstPqCfg, sizeof(VPSS_REG_S))), which is why a
 * node carries the tuning words of *every* block the VPSS owns and not just
 * those of the block the job uses: DEI and FMD at 0x1000, DB/DM/DR/DS at
 * 0x2500, SNR at 0x3000, TNR at 0x3800.
 *
 * The rows are the ones drv_pq.c registers as REG_TYPE_VPSS - DEI, FMD, DB,
 * DM, DR, DS, SNR and TNR - because that is what makes
 * PQ_HAL_WriteRegister() resolve the address against the VPSS register
 * struct.  The remaining modules (SHARPNESS, ACM/COLOR, DCI, ARTDS, the
 * VDP's CSC) are REG_TYPE_VDP and are not part of the node.
 *
 * 2116 rows collapse to the 352 words below.  DM and SNR have per source
 * size rows (SD/HD/UHD); this image holds the source independent rows plus
 * the SD variants, and histb_vpss_pq_hd carries the nine words the FHD set
 * changes.  The block only runs up to 1920x1088, so those are the only two
 * sets reachable from here.
 */
struct histb_vpss_pq_default {
	u32 reg;
	u32 value;
};

static const struct histb_vpss_pq_default histb_vpss_pq_defaults[] = {
	{ 0x0000, 0x00000800u }, { 0x1000, 0x05002000u }, { 0x1004, 0x08000092u }, { 0x1008, 0xfec00200u },
	{ 0x100c, 0x00000002u }, { 0x1010, 0x0008021eu }, { 0x1014, 0x0008021eu }, { 0x1018, 0x1b201828u },
	{ 0x101c, 0x0b0c0f12u }, { 0x1020, 0x06070809u }, { 0x1024, 0x00030505u }, { 0x102c, 0x87766556u },
	{ 0x1030, 0x08888888u }, { 0x1034, 0x13880420u }, { 0x1038, 0x00000000u }, { 0x103c, 0x183eff00u },
	{ 0x1040, 0xffd09010u }, { 0x1044, 0x4020ffffu }, { 0x1048, 0x20000081u }, { 0x104c, 0x40404030u },
	{ 0x1050, 0x000d0022u }, { 0x1054, 0x01000000u }, { 0x1058, 0xa00a0fffu }, { 0x105c, 0x00001e40u },
	{ 0x1060, 0xffff4808u }, { 0x1064, 0x4000ffffu }, { 0x1068, 0x00000008u }, { 0x106c, 0x40404040u },
	{ 0x1070, 0xffff4808u }, { 0x1074, 0x4000ffffu }, { 0x1078, 0x00000008u }, { 0x107c, 0x40404040u },
	{ 0x1080, 0x00800080u }, { 0x1084, 0x000e3000u }, { 0x1088, 0x00073088u }, { 0x108c, 0x00084086u },
	{ 0x1090, 0x4008408cu }, { 0x1094, 0x2008f08eu }, { 0x1098, 0x40086087u }, { 0x109c, 0xff020010u },
	{ 0x10a0, 0x10000600u }, { 0x10a4, 0x10104010u }, { 0x10a8, 0x40084010u }, { 0x10ac, 0x03ff4040u },
	{ 0x10b0, 0x00000080u }, { 0x10b4, 0x00100000u }, { 0x10b8, 0x00100030u }, { 0x10bc, 0x00500070u },
	{ 0x10c0, 0x11c140c4u }, { 0x10c4, 0x00060164u }, { 0x10c8, 0x20201008u }, { 0x10cc, 0x00143214u },
	{ 0x10d0, 0x00000006u }, { 0x10d4, 0x01e00000u }, { 0x10d8, 0x00402008u }, { 0x10dc, 0x0010105au },
	{ 0x10e0, 0x00000010u }, { 0x10e4, 0x80402010u }, { 0x10e8, 0x20100804u }, { 0x10ec, 0x00200f14u },
	{ 0x10f0, 0x00000020u }, { 0x10f4, 0x00000000u }, { 0x10f8, 0x00000000u }, { 0x10fc, 0x03ffddffu },
	{ 0x1100, 0x00907effu }, { 0x1104, 0x01820100u }, { 0x1108, 0x00000cffu }, { 0x110c, 0x00508191u },
	{ 0x1110, 0x00045088u }, { 0x1114, 0x00000000u }, { 0x1118, 0x00064814u }, { 0x111c, 0x01310c10u },
	{ 0x1120, 0x01480f08u }, { 0x1124, 0x08410107u }, { 0x1128, 0x00c08808u }, { 0x112c, 0x01020040u },
	{ 0x1130, 0x0400c140u }, { 0x1134, 0x0101fea0u }, { 0x1138, 0x187a4010u }, { 0x113c, 0x100040ffu },
	{ 0x1140, 0x00001244u }, { 0x1144, 0x7c444412u }, { 0x1148, 0x00cc3200u }, { 0x114c, 0x02101120u },
	{ 0x1150, 0x00002410u }, { 0x1154, 0x000405fcu }, { 0x1158, 0x038e1410u }, { 0x115c, 0x10801100u },
	{ 0x1160, 0x40201008u }, { 0x1164, 0x20402020u }, { 0x1168, 0x0001411fu }, { 0x116c, 0x10824210u },
	{ 0x1170, 0x00000000u }, { 0x1174, 0x000001e8u }, { 0x1178, 0x00041488u }, { 0x117c, 0x41000000u },
	{ 0x1180, 0x00000000u }, { 0x1184, 0x20ffffffu }, { 0x1188, 0x00000c10u }, { 0x118c, 0x00000000u },
	{ 0x1190, 0x0008408cu }, { 0x1194, 0x00001000u }, { 0x1198, 0x00000014u }, { 0x2500, 0x0225a43fu },
	{ 0x258c, 0x00000110u }, { 0x2590, 0x00063210u }, { 0x2594, 0x0c8531d0u }, { 0x2598, 0x00000044u },
	{ 0x259c, 0x16c6b9f0u }, { 0x25a0, 0x02329d2au }, { 0x25a4, 0x0000012au }, { 0x25b4, 0x18c6b9f0u },
	{ 0x25b8, 0x0a63a54bu }, { 0x25bc, 0x04221085u }, { 0x25c0, 0x00110842u }, { 0x25c4, 0x00000000u },
	{ 0x25c8, 0x21084210u }, { 0x25cc, 0x00010990u }, { 0x25d0, 0x00000000u }, { 0x25d4, 0x0c8531d0u },
	{ 0x25d8, 0x00000044u }, { 0x25dc, 0x1c180c08u }, { 0x25e0, 0x78502824u }, { 0x25e4, 0x0e0681e0u },
	{ 0x25e8, 0x06338070u }, { 0x25f0, 0x2881c200u }, { 0x25f4, 0x00000f2cu }, { 0x25f8, 0x00001020u },
	{ 0x25fc, 0x00403420u }, { 0x2600, 0x00028000u }, { 0x2604, 0x00000000u }, { 0x2608, 0x00000000u },
	{ 0x260c, 0x00000000u }, { 0x2610, 0x88888884u }, { 0x2614, 0x00246888u }, { 0x2618, 0x00002460u },
	{ 0x261c, 0x00000000u }, { 0x2620, 0x00000008u }, { 0x2624, 0x00000000u }, { 0x2628, 0x00000000u },
	{ 0x2640, 0x00202020u }, { 0x2644, 0x00101010u }, { 0x2648, 0xca752100u }, { 0x264c, 0xfffeeddcu },
	{ 0x2650, 0x00101014u }, { 0x2654, 0x03c0b45au }, { 0x2658, 0x00030010u }, { 0x265c, 0x00070050u },
	{ 0x2660, 0x000b0090u }, { 0x2664, 0x000000d0u }, { 0x2668, 0x00030010u }, { 0x266c, 0x00070050u },
	{ 0x2670, 0x000b0090u }, { 0x2674, 0x000000d0u }, { 0x2678, 0x00030010u }, { 0x267c, 0x00070050u },
	{ 0x2680, 0x000b0090u }, { 0x2684, 0x000000d0u }, { 0x2688, 0x10101006u }, { 0x2700, 0x00021498u },
	{ 0x2704, 0x02115000u }, { 0x2708, 0x20100804u }, { 0x270c, 0x00481240u }, { 0x2710, 0x56888888u },
	{ 0x2714, 0x68888888u }, { 0x2718, 0x00000011u }, { 0x271c, 0xa4480000u }, { 0x2720, 0x00100802u },
	{ 0x2724, 0x2400781eu }, { 0x2728, 0x0280c819u }, { 0x272c, 0x0a000000u }, { 0x2730, 0x0000a014u },
	{ 0x2734, 0x00000000u }, { 0x2738, 0x00006775u }, { 0x273c, 0x00060202u }, { 0x2740, 0x01802008u },
	{ 0x2760, 0x00406006u }, { 0x2764, 0x01040584u }, { 0x2768, 0x0062c287u }, { 0x276c, 0x00010506u },
	{ 0x2770, 0x0005502du }, { 0x2774, 0x000c050au }, { 0x2778, 0x00b45b00u }, { 0x277c, 0x00082605u },
	{ 0x2780, 0x000a2d05u }, { 0x2784, 0x005a5a08u }, { 0x2788, 0x00140505u }, { 0x278c, 0x01401008u },
	{ 0x2790, 0x0040120au }, { 0x2794, 0x027102d0u }, { 0x2798, 0x01600000u }, { 0x279c, 0x00000020u },
	{ 0x27a0, 0x00000001u }, { 0x27a4, 0x00004c16u }, { 0x27b8, 0x04600000u }, { 0x2800, 0x00000018u },
	{ 0x2804, 0x03206406u }, { 0x2808, 0x0020c278u }, { 0x280c, 0x0c441208u }, { 0x2810, 0x23002820u },
	{ 0x2814, 0x00803c11u }, { 0x2818, 0x00014121u }, { 0x281c, 0x000008f0u }, { 0x2820, 0x00005000u },
	{ 0x2824, 0x00302820u }, { 0x2828, 0x000c1430u }, { 0x282c, 0x000c1430u }, { 0x2830, 0x02207838u },
	{ 0x2834, 0x05032008u }, { 0x2838, 0x00007850u }, { 0x283c, 0x1045b878u }, { 0x2840, 0x001e1f20u },
	{ 0x2844, 0x03211402u }, { 0x2848, 0x00000c14u }, { 0x284c, 0x00308064u }, { 0x2850, 0x005a5080u },
	{ 0x2854, 0x00ffab00u }, { 0x2858, 0x0000003eu }, { 0x285c, 0x000000c8u }, { 0x2860, 0x01428190u },
	{ 0x2864, 0x000a0322u }, { 0x2868, 0x010228f8u }, { 0x3000, 0x08001046u }, { 0x3004, 0x0400a008u },
	{ 0x3008, 0x3ff3fc80u }, { 0x300c, 0x00102040u }, { 0x3010, 0x00002040u }, { 0x3014, 0x00000000u },
	{ 0x3018, 0x00000000u }, { 0x301c, 0x0000007fu }, { 0x3020, 0x01f03c07u }, { 0x3024, 0x0ff1fc3fu },
	{ 0x3028, 0x000ffdffu }, { 0x302c, 0x0048a100u }, { 0x3030, 0x0003fb9eu }, { 0x3034, 0x00406080u },
	{ 0x3038, 0x00112030u }, { 0x303c, 0x00000fc0u }, { 0x3040, 0x00002ce4u }, { 0x3044, 0x00000000u },
	{ 0x3048, 0x00001882u }, { 0x304c, 0x0000571bu }, { 0x3050, 0x00000421u }, { 0x3054, 0x00004ab8u },
	{ 0x3058, 0x00000020u }, { 0x305c, 0x000000ffu }, { 0x3060, 0x00000000u }, { 0x3064, 0x00000087u },
	{ 0x3068, 0x00000000u }, { 0x306c, 0x00000022u }, { 0x3070, 0x00000003u }, { 0x3074, 0x00000400u },
	{ 0x3078, 0x00000007u }, { 0x307c, 0x00000007u }, { 0x3080, 0x00000000u }, { 0x3084, 0x00000000u },
	{ 0x3088, 0x00000000u }, { 0x308c, 0x00000000u }, { 0x3090, 0x00000000u }, { 0x3094, 0x00000000u },
	{ 0x3098, 0x00000000u }, { 0x309c, 0x00000000u }, { 0x30a0, 0x00000000u }, { 0x30a4, 0x00000000u },
	{ 0x30a8, 0x00000000u }, { 0x30ac, 0x00000000u }, { 0x30b0, 0x00000000u }, { 0x30b4, 0x00000000u },
	{ 0x30b8, 0x00000000u }, { 0x30bc, 0x00000000u }, { 0x30c0, 0x00000000u }, { 0x30c4, 0x00000000u },
	{ 0x30c8, 0x00000000u }, { 0x30cc, 0x00000000u }, { 0x30d0, 0x00000000u }, { 0x3800, 0x04202bc2u },
	{ 0x3804, 0x8003fc00u }, { 0x3808, 0x0600e005u }, { 0x380c, 0x0003fc90u }, { 0x3810, 0x04008000u },
	{ 0x3814, 0x0ff3fc80u }, { 0x3818, 0x0000501au }, { 0x381c, 0x0000380cu }, { 0x3820, 0x0003fc00u },
	{ 0x3824, 0x0600e005u }, { 0x3828, 0x0003fc90u }, { 0x382c, 0x04008000u }, { 0x3830, 0x0ff3fc80u },
	{ 0x3834, 0x0000501au }, { 0x3838, 0x0000380cu }, { 0x383c, 0x000ffc00u }, { 0x3840, 0x02008078u },
	{ 0x3844, 0x00008020u }, { 0x3848, 0x10009018u }, { 0x384c, 0x3fffffffu }, { 0x3850, 0x0000038bu },
	{ 0x3854, 0x00000000u }, { 0x3858, 0x000ffc00u }, { 0x385c, 0x02008020u }, { 0x3860, 0x00008020u },
	{ 0x3864, 0x0c819001u }, { 0x3868, 0x3ff6412cu }, { 0x386c, 0x00000000u }, { 0x3870, 0x00000000u },
	{ 0x3874, 0x02190182u }, { 0x3878, 0x02190182u }, { 0x3974, 0x00008000u }, { 0x3978, 0x00200000u },
	{ 0x397c, 0x00008000u }, { 0x3980, 0x0140140au }, { 0x3984, 0x0000801bu }, { 0x3988, 0x02001000u },
	{ 0x398c, 0x03f0e82eu }, { 0x3990, 0x000027ecu }, { 0x3994, 0x00001c08u }, { 0x3998, 0x00008000u },
	{ 0x399c, 0x0140140au }, { 0x39a0, 0x0000801bu }, { 0x39a4, 0x02001000u }, { 0x39a8, 0x03f0e82eu },
	{ 0x39ac, 0x000027bau }, { 0x39b0, 0x00001c08u }, { 0x39b4, 0x69cffc20u }, { 0x39b8, 0x00000282u },
	{ 0x39bc, 0x00008000u }, { 0x39c0, 0x00000000u }, { 0x39c4, 0x00004000u }, { 0x39c8, 0x00008000u },
	{ 0x39cc, 0x00000000u }, { 0x39d0, 0x00008000u }, { 0x39d4, 0x00000000u }, { 0x39d8, 0x00006400u },
	{ 0x39dc, 0x00007000u }, { 0x39e0, 0x00004000u }, { 0x39e4, 0x02200006u }, { 0x39e8, 0x00000c00u },
	{ 0x3e40, 0x06070707u }, { 0x3e44, 0x08070706u }, { 0x3e48, 0x0c0b0a09u }, { 0x3e4c, 0x11100f0du },
	{ 0x3e50, 0x15141312u }, { 0x3e54, 0x19181716u }, { 0x3e58, 0x1d1c1b1au }, { 0x3e5c, 0x1f1f1f1eu },
	{ 0x3e60, 0x06070707u }, { 0x3e64, 0x08070706u }, { 0x3e68, 0x0c0b0a09u }, { 0x3e6c, 0x11100f0du },
	{ 0x3e70, 0x15141312u }, { 0x3e74, 0x19181716u }, { 0x3e78, 0x1d1c1b1au }, { 0x3e7c, 0x1f1f1f1eu },
};

/*
 * HI_PQ_MODULE_SNR's FHD rows: diredgesmoothen (0x3000 bit 14) and the eight
 * winmeanhordiffthrdir* window thresholds, which the HD set raises from 0 to
 * 500.  Row order matches the table's, so they are applied over the image
 * above.
 */
static const struct histb_vpss_pq_default histb_vpss_pq_hd[] = {
	{ 0x3000, 0x08005046u }, { 0x3098, 0x0007d000u }, { 0x30a0, 0x0007d000u }, { 0x30a8, 0x0007d000u },
	{ 0x30b0, 0x0007d000u }, { 0x30b8, 0x0007d000u }, { 0x30c0, 0x0007d000u }, { 0x30c8, 0x0007d000u },
	{ 0x30d0, 0x0007d000u },
};

int histb_vpss_dei(struct histb_vpss *vpss,
		   const struct histb_vpss_dei_frame *frame,
		   dma_addr_t output_dma)
{
	u32 stride = frame ? frame->stride : 0;
	u32 output_stride = frame ? frame->output_stride : 0;
	u32 field_stride;
	u32 chroma_off;
	u32 st_stride;
	u32 st_slot;
	u32 ctrl;
	u32 pnext_ack;
	unsigned int i;
	unsigned long timeout;
	int ret;

	if (!vpss || !frame || !frame->width || !frame->height || !stride ||
	    !output_stride)
		return -EINVAL;
	if (frame->width > HISTB_VPSS_DEI_MAX_WIDTH ||
	    frame->height * 2 > HISTB_VPSS_DEI_MAX_HEIGHT)
		return -EINVAL;
	if (stride & 15 || stride < frame->width * (frame->ten_bit ? 2 : 1))
		return -EINVAL;
	if (output_stride & 15 ||
	    output_stride < frame->width * (frame->ten_bit ? 2 : 1))
		return -EINVAL;
	if (stride > U32_MAX / 2)
		return -EINVAL;
	if (!frame->ref_dma || !frame->cur_dma || !frame->nxt1_dma ||
	    !frame->nxt2_dma || !output_dma)
		return -EINVAL;
	if (upper_32_bits(frame->ref_dma) || upper_32_bits(frame->cur_dma) ||
	    upper_32_bits(frame->nxt1_dma) || upper_32_bits(frame->nxt2_dma) ||
	    upper_32_bits(output_dma))
		return -EINVAL;

	/*
	 * frame->height is the height of one field; the block reads a field out
	 * of the woven frame that holds it, so the image it is given is the
	 * whole frame.  VPSS_HAL_SetH265DeiCfg() - the vendor's de-interlace
	 * configuration - halves this on the way in:
	 *
	 *   SetImgSize(width, stInInfo.u32Height * 2, ...)
	 *   SetImgStride(LAST/CUR/NEXT1/NEXT2, u32Stride_Y, ...)
	 *
	 * i.e. the image size counts the woven frame's lines and the line step
	 * is the frame pitch, with bfield/bfield_first/bfield_mode telling the
	 * block which of the two fields to take from it.  Passing the field
	 * height instead makes the block treat half a field as its input.
	 */
	field_stride = stride;
	chroma_off = stride * frame->height * 2;

	mutex_lock(&vpss->lock);

	ret = pm_runtime_resume_and_get(vpss->dev);
	if (ret < 0)
		goto unlock;

	ret = histb_vpss_prepare_dei_st(vpss, frame->width, frame->height);
	if (ret < 0)
		goto put;
	if ((dei_opt & HISTB_VPSS_OPT_STT_WB) && histb_vpss_prepare_stt_wb(vpss)) {
		ret = -ENOMEM;
		goto put;
	}

	st_stride = histb_vpss_dei_st_stride(frame->width);
	st_slot = st_stride * frame->height;

	/*
	 * Everything goes into the node image, not into the register window:
	 * the engine reads its register set from the descriptor that
	 * VPSS_NEXT points at, which is how the scaler path works too.
	 * Writing the live registers instead leaves NEXT at zero, the engine
	 * finds no node to run and the job times out - measured as
	 * diesta = 0, intstat = 0, next = 0 with every other field correct.
	 */
	memset(vpss->node, 0, HISTB_VPSS_NODE_SIZE);

	/*
	 * VPSS_REG_ResetAppReg(): the BSP starts every node as a copy of the
	 * whole PQ register image, so the de-interlacer is handed a node whose
	 * DB, SNR and TNR parameter regions are filled in as well.  Those
	 * regions are what the blocks' own state machines read, and leaving
	 * them at zero is the one thing that still differed from the BSP with
	 * the whole DEI block byte for byte correct: the engine loaded the
	 * node, the DEI words appeared in the register file, and the block
	 * stayed idle - diesta (0x1028) = 0, rawint = 0, eof count = 0.
	 *
	 * Everything written after this point is one of the HAL's SetXXX()
	 * calls, which VPSS_HAL_SetNode_H265_Step2_Dei() issues *after*
	 * VPSS_REG_ResetAppReg(), so they override individual fields of this
	 * image and never the other way round.
	 */
	for (i = 0; i < ARRAY_SIZE(histb_vpss_pq_defaults); i++)
		histb_vpss_node_write(vpss, histb_vpss_pq_defaults[i].reg,
				      histb_vpss_pq_defaults[i].value);
	/* The SNR tuning is the one part of the table the source size picks. */
	if (frame->width > 720)
		for (i = 0; i < ARRAY_SIZE(histb_vpss_pq_hd); i++)
			histb_vpss_node_write(vpss, histb_vpss_pq_hd[i].reg,
					      histb_vpss_pq_hd[i].value);

	/*
	 * VPSS_HAL_SetH265DeiCfg() - the vendor's de-interlace configuration
	 * for this block - is what the control word below reproduces:
	 *
	 *   SetImgReadMod(TRUE)   CTRL.bfield[31]
	 *   EnDei(TRUE)           CTRL.dei_en[7]
	 *   SetDeiTopFirst()      CTRL.bfield_first[29]
	 *   SetDeiFieldMode()     CTRL.bfield_mode[30]
	 *   SetMode(4-field)      DIECTRL.die_l_mode/die_c_mode = 1
	 *   SetDeiParaAddr()      VPSS_DEI_ADDR = node + 0x1000
	 *   SetIglbEn(TRUE)       CTRL.igbm_en[26]
	 *   SetIfmdEn(TRUE)       CTRL.ifmd_en[25]
	 *
	 * and the port, dbm and four_pix bits come from the same node's
	 * SetPortCfg()/SetDbmEn()/ResetAppReg().
	 */
	ctrl = histb_vpss_node_read(vpss, HISTB_VPSS_CTRL);
	ctrl |= HISTB_VPSS_CTRL_DEI_EN | HISTB_VPSS_CTRL_OUTPUT_EN;
	/*
	 * FOUR_PIX is what histb_vpss_build_node() - the detile/scaler path,
	 * which the engine completes on every frame - sets and the DEI path
	 * did not.  The de-interlacer's motion estimator works on four-pixel
	 * groups, so it is a plausible start condition.
	 */
	/*
	 * four_pix_en stays clear.  This bit is set by the detile node, whose
	 * input is a tiled surface, and the de-interlace node never has it:
	 * VPSS_HAL_SetNode_H265_Step2_Dei() starts from ResetAppReg() - which
	 * zeroes it - and only turns on prot, dei_en, bfield, ifmd_en, igbm_en,
	 * dbm_en and the port, and the de-interlacer's own input here is a
	 * linear surface.  The bit is the four-pixel group mode of the input
	 * fetch, so enabling it for linear data leaves the fetch waiting for a
	 * grouping the memory never produces.
	 */
	ctrl &= ~(u32)HISTB_VPSS_CTRL_FOUR_PIX;
	if (dei_opt & HISTB_VPSS_OPT_FOUR_PIX)
		ctrl |= HISTB_VPSS_CTRL_FOUR_PIX;
	/*
	 * MCDI stays off, which is what the vendor's own de-interlace node
	 * does: VPSS_HAL_SetNode_H265_Step2_Dei() selects the same 4-field
	 * mode through SetH265DeiCfg() and never calls SetMcDeiCfg(), so its
	 * node runs with mcdi_en and meds_en clear and with no motion-vector
	 * buffers programmed at all.  Only the 5-field node - whose block
	 * motion, region motion and projection buffers are sized by
	 * VPSS_STTINFO_CalDieBufSize() and rotated by the MCDI allocator -
	 * turns MCDI on.  This driver turned it on while pointing every one
	 * of the seven motion buffers at the statistics ring with a stride
	 * they were never sized for, and the frame never completed: no eof,
	 * no ip_used, no error flag, and the next job could not start.
	 */
	ctrl &= ~(u32)(HISTB_VPSS_CTRL_MCDI_EN | HISTB_VPSS_CTRL_MEDS_EN);
	ctrl |= HISTB_VPSS_CTRL_BFIELD;		/* SetImgReadMod(HI_TRUE) */
	ctrl |= HISTB_VPSS_CTRL_IGBM_EN;	/* SetIglbEn(HI_TRUE) */
	ctrl |= HISTB_VPSS_CTRL_IFMD_EN;	/* SetIfmdEn(HI_TRUE) */
	if (!frame->top_field_first)
		ctrl |= HISTB_VPSS_CTRL_BFIELD_FIRST;
	else
		ctrl &= ~HISTB_VPSS_CTRL_BFIELD_FIRST;
	if (frame->bottom_field)
		ctrl |= HISTB_VPSS_CTRL_BFIELD_MODE;	/* SetDeiFieldMode */
	else
		ctrl &= ~HISTB_VPSS_CTRL_BFIELD_MODE;
	histb_vpss_node_write(vpss, HISTB_VPSS_CTRL, ctrl);

	/* VPSS_REG_SetStRdAddr/SetStWrAddr/SetStStride - motion statistics. */
	histb_vpss_node_write(vpss, HISTB_VPSS_ST_RD_ADDR,
			      lower_32_bits(vpss->dei_st_dma +
					    (vpss->dei_st_seq %
					     HISTB_VPSS_DEI_ST_SLOTS) * st_slot));
	histb_vpss_node_write(vpss, HISTB_VPSS_ST_WR_ADDR,
			      lower_32_bits(vpss->dei_st_dma +
					    ((vpss->dei_st_seq + 1) %
					     HISTB_VPSS_DEI_ST_SLOTS) * st_slot));
	histb_vpss_node_write(vpss, HISTB_VPSS_ST_STRIDE, st_stride);
	vpss->dei_st_seq++;

	/*
	 * The de-interlaced frame leaves through the port - VHD0YADDR/CADDR/
	 * STRIDE/SIZE and CTRL.vhd0_en below - which is what both of the BSP's
	 * de-interlace nodes do.  VPSS_NX2Y_LB_ADDR/NX2C_LB_ADDR/NX2_LB_STRIDE
	 * are the luma/chroma low bit planes of a 10-bit *input*, so they stay
	 * clear for an 8-bit one; this driver used to point them at the output
	 * buffer.
	 */
	/*
	 * SetImgSize() of the de-interlace node: the image is the woven frame
	 * the four fields live in, not one field of it.  With bfield set, the
	 * block takes the field it wants out of that frame; giving it the field
	 * height instead makes it process half a field's worth of lines.
	 */
	histb_vpss_node_write(vpss, HISTB_VPSS_IMG_SIZE,
			      ((frame->height * 2) - 1) << 16 |
			      (frame->width - 1));

	/* The four fields, in the same group-of-four layout as the registers. */
	histb_vpss_dei_field(vpss, HISTB_VPSS_DEI_REF_CTRL,
			     HISTB_VPSS_DEI_REFYADDR, HISTB_VPSS_DEI_REFCADDR,
			     HISTB_VPSS_DEI_REFSTRIDE, frame->ref_dma,
			     field_stride, chroma_off);
	histb_vpss_dei_field(vpss, HISTB_VPSS_DEI_CUR_CTRL,
			     HISTB_VPSS_DEI_CURYADDR, HISTB_VPSS_DEI_CURCADDR,
			     HISTB_VPSS_DEI_CURSTRIDE, frame->cur_dma,
			     field_stride, chroma_off);
	histb_vpss_dei_field(vpss, HISTB_VPSS_DEI_NXT1_CTRL,
			     HISTB_VPSS_DEI_NXT1YADDR, HISTB_VPSS_DEI_NXT1CADDR,
			     HISTB_VPSS_DEI_NXT1STRIDE, frame->nxt1_dma,
			     field_stride, chroma_off);
	histb_vpss_dei_field(vpss, HISTB_VPSS_DEI_NXT2_CTRL,
			     HISTB_VPSS_DEI_NXT2YADDR, HISTB_VPSS_DEI_NXT2CADDR,
			     HISTB_VPSS_DEI_NXT2STRIDE, frame->nxt2_dma,
			     field_stride, chroma_off);

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
	/*
	 * H.265 Step2 programs NEXT2_FIELD here.  The register helper maps
	 * that channel to ref_nxt_pix_bitw (bit 22), not in_pix_bitw (bit 21).
	 */
	histb_vpss_node_write(vpss, HISTB_VPSS_CTRL2,
			      frame->ten_bit ? HISTB_VPSS_CTRL2_REF_NXT_BITW : 0);
	/* CTRL3: no input crop. */
	histb_vpss_node_write(vpss, HISTB_VPSS_CTRL3, 0);

	/* The block reads its own tuning block from inside the register map. */
	/*
	 * VPSS_REG_SetDeiParaAddr(): the address the block fetches its tuning
	 * block from.  The BSP computes it as
	 *
	 *   u32AppPhy + VPSS_REG_SIZE_CALC(VPSS_CTRL, VPSS_DIECTRL)
	 *             - sizeof(HI_U32)
	 *
	 * and VPSS_REG_SIZE_CALC() adds sizeof(HI_U32) back, so the value is
	 * the plain offset of VPSS_DIECTRL inside the node - 0x1000, the first
	 * word of the block below.  This driver pointed the block 12 bytes
	 * further in, which makes it read die_l_mode/die_c_mode and every
	 * threshold after them from the wrong words: the visible register file
	 * still looks right - the node carries the whole block - but the
	 * block's own copy is misaligned, no de-interlace mode gets selected
	 * and its state machine never leaves idle.  Measured before the fix:
	 * DIESTA = 0, RAWINT = 0, EOF count = 0 with the node loaded and every
	 * register in the block byte-for-byte correct.
	 */
	histb_vpss_node_write(vpss, HISTB_VPSS_DEI_ADDR,
			      lower_32_bits(vpss->node_dma +
					    HISTB_VPSS_DEI_PARAM_BASE));

	/* --- remaining BSP calls our driver never made --- */
	{
		u32 c = histb_vpss_node_read(vpss, HISTB_VPSS_CTRL);

		/* prot[24:23]=0 (not secure), rfr/tnr/snr off, dbm for big frames */
		c &= ~(u32)(HISTB_VPSS_CTRL_PROT | HISTB_VPSS_CTRL_RFR_EN |
			    HISTB_VPSS_CTRL_TNR_EN | HISTB_VPSS_CTRL_SNR_EN);
		/*
		 * dbm_en stays clear.  The BSP turns it on for frames above
		 * 128x64, and with the node now carrying the whole PQ table the
		 * de-blocking block would have the same parameter block the BSP
		 * gives it - so this was measured with dbm_en set (ctrl
		 * 0x86000888) as well as clear.  Neither starts the
		 * de-interlacer: diesta stayed 0 with every DB word at 0x2500-
		 * 0x2688 and every SNR word at 0x3000-0x30d0 correct in the
		 * register file.  The stage is left off because its
		 * VPSS_DB_BORD_FLAG array (0x2900-0x2afc) has no PQ table rows
		 * either, so enabling it buys nothing and leaves a second block
		 * running on half-unknown configuration.
		 */
		c &= ~(u32)HISTB_VPSS_CTRL_DBM_EN;
		histb_vpss_node_write(vpss, HISTB_VPSS_CTRL, c);

		/* SetDcmpEn(FALSE) - no compressed frame store. */
		histb_vpss_node_write(vpss, HISTB_VPSS_INPUT_CTRL, 0);

		/*
		 * VPSS_REG_SetAlgParaAddr(): every BSP node builder calls it,
		 * and it is what gives the DBM and SNR blocks a parameter
		 * block to fetch.  The value is the plain offset of the
		 * block's first register inside the node - VPSS_DB_CTRL at
		 * 0x2500 and VPSS_SNR_ENABLE at 0x3000 (VPSS_REG_SIZE_CALC()
		 * adds back the word it subtracts).
		 */
		histb_vpss_node_write(vpss, HISTB_VPSS_DBM_ADDR,
				      lower_32_bits(vpss->node_dma +
						    HISTB_VPSS_DB_PARA));
		histb_vpss_node_write(vpss, HISTB_VPSS_SNR_ADDR,
				      lower_32_bits(vpss->node_dma +
						    HISTB_VPSS_SNR_PARA));
	}

	/* SetRefWidth / SetRefHight encode both dimensions as value minus one. */
	histb_vpss_node_write(vpss, HISTB_VPSS_REFSIZE,
				      ((((frame->height * 2) - 1) & 0x1fff) << 16) |
				      ((frame->width - 1) & 0x1fff));

	/* SetSttWrAddr / SetTnrAddr / SetTnrClutAddr: the block wants these
	 * offsets inside the node even when TNR is disabled.  VPSS_REG_SIZE_
	 * CALC() nets out to the plain register offset, so these are 0x3800
	 * and 0x3b00, not four bytes short of them. */
	histb_vpss_node_write(vpss, HISTB_VPSS_STT_W_ADDR,
			      (dei_opt & HISTB_VPSS_OPT_STT_WB) && vpss->stt_cpu
			      ? lower_32_bits(vpss->stt_dma) : 0);
	histb_vpss_config_dei_port(vpss, frame->width, frame->height * 2);
	histb_vpss_node_write(vpss, HISTB_VPSS_TNR_ADDR,
			      lower_32_bits(vpss->node_dma + 0x3800));
	histb_vpss_node_write(vpss, HISTB_VPSS_TNR_CLUT_ADDR,
			      lower_32_bits(vpss->node_dma + 0x3b00));
	/*
	 * The engine loads this node into its own register file, so the node's
	 * INTMASK is what actually governs interrupt delivery - the live
	 * window copy is overwritten when the node is fetched.  The readback
	 * showed intmask=0xff even after the live window was written with
	 * 0xfe, which is exactly that overwrite.  Use 0xfe, i.e. eof unmasked
	 * (0 = enabled), matching what the HAL leaves in the live window.
	 */
	histb_vpss_node_write(vpss, HISTB_VPSS_INT_MASK, 0xfe);
	histb_vpss_node_write(vpss, HISTB_VPSS_NEXT, 0);

	reinit_completion(&vpss->completion);
	vpss->irq_state = 0;
	writel(HISTB_VPSS_INT_ALL, vpss->regs + HISTB_VPSS_INT_CLEAR);
	writel(HISTB_VPSS_MISC_DEFAULT, vpss->regs + HISTB_VPSS_MISC);
	/*
	 * VPSS_REG_SetIntMask(base, 0xfe) - the HAL unmasks exactly one bit
	 * (eof) in the live window after every ResetAppReg.  0 = enabled, so
	 * 0xfe leaves bit 0 clear.  This driver never set the live window at
	 * all, and the node copy is 0xff (all masked), so no completion
	 * interrupt could ever arrive.
	 */
	writel(0xfe, vpss->regs + HISTB_VPSS_INT_MASK);
	/*
	 * VPSS_REG_StartLogic() writes PNEXT and START into the *node*, not the
	 * live window: it casts its pu32PhyAddr argument - the node address -
	 * to VPSS_REG_S * and stores through that.  Writing the live window
	 * instead leaves the engine looking for a start bit inside the node
	 * that was never set, so it never begins and never raises a completion
	 * interrupt.  Measured before the fix: start=0x0 and pnext=0x0 in the
	 * live window while ctrl=0x86000088 (read from the node) was correct.
	 */
	/*
	 * VPSS_REG_SetNodeID(): the BSP always stamps a node id before the
	 * node is submitted.  With it left at zero the engine loads the node
	 * (its CTRL is visible on readback) and then does nothing at all -
	 * no completion interrupt, no error bit - which is exactly what we
	 * measured.  Use 1, a non-zero user node id.
	 */
	/*
	 * VPSS_REG_SetRchSmmuBypass/WchSmmuBypass: the BSP unconditionally
	 * programs both to 0xffffffff (bypass) for every node it builds.  A
	 * non-bypassed channel with no page tables installed would stall every
	 * bus access the engine makes, which matches the readout: the engine
	 * loads the node, then neither completes nor reports an error.
	 */
	histb_vpss_node_write(vpss, HISTB_VPSS_RCH_BYPASS, 0xffffffff);
	histb_vpss_node_write(vpss, HISTB_VPSS_WCH_BYPASS, 0xffffffff);

	/*
	 * Port/VHD0 output: the de-interlaced frame is one progressive frame of
	 * twice the field height, written to the caller's surface at the
	 * caller's pitch.  VPSS_HAL_SetPortCfg() is the vendor's version -
	 * SetFrmSize(h, w), SetFrmAddr(Y, C), SetFrmStride(outYStride, ...),
	 * SetFrmFormat(format), EnPort(TRUE) - and it uses the *destination*
	 * stride, not the input's.
	 *
	 * This driver wrote the frame with twice the input pitch and a doubled
	 * stride (borrowed from VPSS_HAL_SetNode_H265_Step1_Interlace, which
	 * writes the *interlaced* surface, not the de-interlaced one).  A
	 * 1088 line frame at a 4096 pitch is 4.4 MB, and the capture surface it
	 * aims at is 1920 x 1088 x 1.5 = 3.1 MB: the port's last writes leave
	 * the buffer entirely, and with both SMMU channels bypassed an access
	 * with no backing never completes, so the frame neither finishes nor
	 * reports an error and the block stays busy.
	 */
	histb_vpss_node_write(vpss, HISTB_VPSS_VHD0_SIZE,
			      ((frame->height * 2 - 1) << 16) |
			      (frame->width - 1));
	histb_vpss_node_write(vpss, HISTB_VPSS_VHD0_Y_ADDR, lower_32_bits(output_dma));
	histb_vpss_node_write(vpss, HISTB_VPSS_VHD0_C_ADDR,
			      lower_32_bits(output_dma +
					    output_stride * frame->height * 2));
	histb_vpss_node_write(vpss, HISTB_VPSS_VHD0_STRIDE,
			      output_stride << 16 | output_stride);
	{
		u32 c2 = histb_vpss_node_read(vpss, HISTB_VPSS_CTRL2) &
			 ~(u32)HISTB_VPSS_CTRL2_VHD0_FORMAT;
		histb_vpss_node_write(vpss, HISTB_VPSS_CTRL2,
				      c2 | (HISTB_VPSS_FMT_420 << 12));
	}
	histb_vpss_node_write(vpss, HISTB_VPSS_VHD0_CTRL,
			      HISTB_VPSS_OUTPUT_DITHER |
			      HISTB_VPSS_OUTPUT_DITHER_ROUND);

	/*
	 * With MCDI off the block never touches the block-motion, region-motion
	 * or projection buffers, so they stay clear: VPSS_HAL_SetNode_H265_
	 * Step2_Dei() leaves them at zero too, and only VPSS_HAL_SetMcDeiCfg()
	 * fills them in, from buffers sized by the MCDI allocator.
	 */

	/*
	 * VPSS_REG_SetNodeId(): the node id is the node's *index* inside the
	 * block's node chain - VPSS_BaseOpt_...() and the split-MCDI builder
	 * both stamp it as `pstVirAddrNodeX->VPSS_NODEID.u32 = u32Idx` - so a
	 * job made of one node carries id 0.  This driver had been stamping 1
	 * on the guess that any non-zero id would do; the engine then had a
	 * chain that starts at 1, loaded the node, waited for the node before
	 * it and never finished the list: no eof, no ip_used, no error flag,
	 * and the block stayed busy.  The detile node - which completes -
	 * leaves the id at zero.
	 */
	histb_vpss_node_write(vpss, HISTB_VPSS_NODE_ID, 0);
	histb_vpss_node_write(vpss, HISTB_VPSS_NEXT, 0);
	histb_vpss_node_write(vpss, HISTB_VPSS_START, 1);
	/* Keep the live window's port copy consistent as the BSP does. */
	writel(histb_vpss_node_read(vpss, HISTB_VPSS_CTRL3),
	       vpss->regs + HISTB_VPSS_CTRL3);
	writel(0, vpss->regs + HISTB_VPSS_VHD0CROP_POS);
	writel(histb_vpss_node_read(vpss, HISTB_VPSS_VHD0CROP_SIZE),
	       vpss->regs + HISTB_VPSS_VHD0CROP_SIZE);
	if (dei_opt & HISTB_VPSS_OPT_NODE_RST_EN) {
		histb_vpss_node_write(vpss, HISTB_VPSS_FTCONFIG, 1);
		writel(1, vpss->regs + HISTB_VPSS_FTCONFIG);
	}
	if (dei_opt & HISTB_VPSS_OPT_WATCHDOG) {
		/*
		 * Bypass the 0xffffffff this driver leaves in VPSS_TIMEOUT and
		 * unmask the logic timeout: if the pipeline starts and then
		 * stalls, the watchdog says so instead of timing out silently.
		 */
		u32 to = 0x800000;
		u32 mask = 0x7e;

		histb_vpss_node_write(vpss, HISTB_VPSS_TIMEOUT, to);
		histb_vpss_node_write(vpss, HISTB_VPSS_INT_MASK, mask);
		writel(to, vpss->regs + HISTB_VPSS_TIMEOUT);
		writel(mask, vpss->regs + HISTB_VPSS_INT_MASK);
	}
	if (dei_opt & HISTB_VPSS_OPT_SENTINEL) {
		/*
		 * Sentinels in the words the engine may or may not copy out of
		 * the node: whatever comes back as the sentinel was overwritten
		 * by the node load, so it is not hardware state.  The values are
		 * chosen to sit in bits the interrupt logic ignores (INTSTATE
		 * and RAWINT) or to be harmless counters.  DIESTA's is
		 * cur_state=0/cur_cstate=0 with a non-zero line count - the
		 * state the vendor's own tables seed it with, plus a marker.
		 */
		histb_vpss_node_write(vpss, HISTB_VPSS_INT_STATE, 0x00010000);
		histb_vpss_node_write(vpss, HISTB_VPSS_INT_CLEAR, 0x00010000);
		histb_vpss_node_write(vpss, HISTB_VPSS_RAW_INT, 0x00020000);
		histb_vpss_node_write(vpss, HISTB_VPSS_PFCNT, 0x00001111);
		histb_vpss_node_write(vpss, HISTB_VPSS_MACCFG, 0x00040000);
		histb_vpss_node_write(vpss, HISTB_VPSS_EOF_CNT, 0x00002222);
		histb_vpss_node_write(vpss, HISTB_VPSS_BUSCTRL, 0x00030000);
		histb_vpss_node_write(vpss, HISTB_VPSS_DIESTA, 0x00012300);
	}
	if (dei_opt & HISTB_VPSS_OPT_DIE_RST) {
		u32 diectrl = readl(vpss->regs + HISTB_VPSS_DIECTRL);

		writel(diectrl | HISTB_VPSS_DIE_RST,
		       vpss->regs + HISTB_VPSS_DIECTRL);
		udelay(2);
		writel(diectrl & ~(u32)HISTB_VPSS_DIE_RST,
		       vpss->regs + HISTB_VPSS_DIECTRL);
		udelay(2);
	}
	/* Keep the live window consistent too: harmless, and it is what the
	 * scaler path already does. */
	writel(lower_32_bits(vpss->node_dma), vpss->regs + HISTB_VPSS_NEXT);
	wmb();
	writel(1, vpss->regs + HISTB_VPSS_START);
	/*
	 * PNEXT is a live register the engine takes the job from - the
	 * vendor's VPSS_DRV_Set_VPSS_PNEXT() writes it and reads it straight
	 * back to check the write landed.  Sample it here, before the wait:
	 * the timeout dump below reads it 500 ms later, when the value is
	 * either the same or long cleared, so it cannot tell "the engine took
	 * the job" from "the store never landed" - this can.
	 */
	pnext_ack = readl(vpss->regs + HISTB_VPSS_NEXT);

	timeout = wait_for_completion_timeout(&vpss->completion,
					      msecs_to_jiffies(
						      (dei_opt & HISTB_VPSS_OPT_LONG_WAIT)
						      ? 3000 : 500));
	if (!timeout) {
		{
			unsigned int q;

			dev_err(vpss->dev,
				"dei timed out: intstate=%#x intmask=%#x pnext=%#x ack=%#x start=%#x ctrl=%#x\n",
				readl(vpss->regs + HISTB_VPSS_INT_STATE),
				readl(vpss->regs + HISTB_VPSS_INT_MASK),
				readl(vpss->regs + HISTB_VPSS_NEXT),
				pnext_ack,
				readl(vpss->regs + HISTB_VPSS_START),
				readl(vpss->regs + HISTB_VPSS_CTRL));
			/*
			 * INTSTATE is the masked view and this driver leaves
			 * only eof unmasked, so it hides the block's own error
			 * flags; RAWINT is the unmasked one and it is the only
			 * way to tell "the engine ran and faulted" from "the
			 * engine never started".  EOFCNT counts completed
			 * frames.  0x100-0x13c are the four field slots the
			 * engine latched out of the node.
			 */
			dev_err(vpss->dev,
				"dei raw=%#x eofcnt=%#x ctrl2=%#x ctrl3=%#x imgsize=%#x refsize=%#x nodeid=%#x\n",
				readl(vpss->regs + HISTB_VPSS_RAW_INT),
				readl(vpss->regs + HISTB_VPSS_EOF_CNT),
				readl(vpss->regs + HISTB_VPSS_CTRL2),
				readl(vpss->regs + HISTB_VPSS_CTRL3),
				readl(vpss->regs + HISTB_VPSS_IMG_SIZE),
				readl(vpss->regs + HISTB_VPSS_REFSIZE),
				readl(vpss->regs + HISTB_VPSS_NODE_ID));
			dev_err(vpss->dev,
				"dei stra=%#x stwa=%#x stst=%#x stt=%#x dbm=%#x snr=%#x tnr=%#x dei=%#x zme=%#x misc=%#x to=%#x\n",
				readl(vpss->regs + HISTB_VPSS_ST_RD_ADDR),
				readl(vpss->regs + HISTB_VPSS_ST_WR_ADDR),
				readl(vpss->regs + HISTB_VPSS_ST_STRIDE),
				readl(vpss->regs + HISTB_VPSS_STT_W_ADDR),
				readl(vpss->regs + HISTB_VPSS_DBM_ADDR),
				readl(vpss->regs + HISTB_VPSS_SNR_ADDR),
				readl(vpss->regs + HISTB_VPSS_TNR_ADDR),
				readl(vpss->regs + HISTB_VPSS_DEI_ADDR),
				readl(vpss->regs + HISTB_VPSS_ZME_ADDR),
				readl(vpss->regs + HISTB_VPSS_MISC),
				readl(vpss->regs + HISTB_VPSS_TIMEOUT));
			for (q = 0x100; q < 0x140; q += 16)
				dev_err(vpss->dev,
					"fld[%03x]=%08x %08x %08x %08x\n", q,
					readl(vpss->regs + q), readl(vpss->regs + q + 4),
					readl(vpss->regs + q + 8), readl(vpss->regs + q + 12));
			for (q = 0x1000; q < 0x10d0; q += 16)
				dev_err(vpss->dev,
					"die[%03x]=%08x %08x %08x %08x\n", q,
					readl(vpss->regs + q), readl(vpss->regs + q + 4),
					readl(vpss->regs + q + 8), readl(vpss->regs + q + 12));
			for (q = 0x1100; q < 0x11a0; q += 16)
				dev_err(vpss->dev,
					"die[%03x]=%08x %08x %08x %08x\n", q,
					readl(vpss->regs + q), readl(vpss->regs + q + 4),
					readl(vpss->regs + q + 8), readl(vpss->regs + q + 12));
			/* diesta: the de-interlacer's own state machine. */
			dev_err(vpss->dev, "dei diesta=%#x\n",
				readl(vpss->regs + HISTB_VPSS_DIESTA));
			histb_vpss_dump_state(vpss, "dei");
			/*
			 * The DB/DM/DR/DS, SNR and TNR regions of the node image.
			 * The engine reflects the node it loads into this
			 * register file at the node's own offsets - which is how
			 * the DEI words above became readable at all - so these
			 * lines say both whether the PQ image reached the block
			 * and whether the offsets the PQ table uses are the
			 * block's real ones.
			 */
			{
				static const u32 pq_dump[] = {
					0x2000, 0x2500, 0x2600, 0x2700,
					0x2800, 0x3000, 0x3800, 0x3b00
				};
				unsigned int d;

				for (d = 0; d < ARRAY_SIZE(pq_dump); d++)
					for (q = pq_dump[d]; q < pq_dump[d] + 0x20;
					     q += 16)
						dev_err(vpss->dev,
							"pq[%04x]=%08x %08x %08x %08x\n",
							q, readl(vpss->regs + q),
							readl(vpss->regs + q + 4),
							readl(vpss->regs + q + 8),
							readl(vpss->regs + q + 12));
			}
			/*
			 * The vendor dumps VPSS_DEBUG0 when its logic times
			 * out; DEBUG22 carries vpss_zme_done and DEBUG27 the
			 * per-channel command counters, which is where a stuck
			 * DDR access shows up.
			 */
			for (q = 0x4000; q < 0x4070; q += 16)
				dev_err(vpss->dev,
					"dbg[%04x]=%08x %08x %08x %08x\n", q,
					readl(vpss->regs + q), readl(vpss->regs + q + 4),
					readl(vpss->regs + q + 8), readl(vpss->regs + q + 12));
		}
		ret = -ETIMEDOUT;
	} else if (vpss->irq_state & HISTB_VPSS_INT_ERROR) {
		dev_err(vpss->dev, "dei failed, state=%#x raw=%#x eofcnt=%#x\n",
			vpss->irq_state,
			readl(vpss->regs + HISTB_VPSS_RAW_INT),
			readl(vpss->regs + HISTB_VPSS_EOF_CNT));
		histb_vpss_dump_state(vpss, "dei-err");
		ret = -EIO;
	} else {
		ret = 0;
	}

	writel(0, vpss->regs + HISTB_VPSS_CTRL);
put:
	pm_runtime_put(vpss->dev);
unlock:
	mutex_unlock(&vpss->lock);
	return ret;
}


int histb_vpss_detile(struct histb_vpss *vpss,
		      const struct histb_vpss_frame *frame)
{
	size_t required_input_size;
	u32 input_width;
	u32 input_height;
	bool direct_output;
	dma_addr_t output_dma;
	u32 output_stride;
	unsigned long timeout;
	int ret;

	if (!vpss || !frame || !frame->width || !frame->height ||
	    !frame->input_stride || frame->width & 3 ||
	    !frame->input_height_align ||
	    frame->input_stride < (frame->input_width ?: frame->width) ||
	    upper_32_bits(frame->input_dma))
		return -EINVAL;
	input_width = frame->input_width ?: frame->width;
	input_height = frame->input_height ?: frame->height;
	if (!input_width || !input_height || input_width & 3 ||
	    input_height & 1 || frame->height & 1 ||
	    (u64)input_width >=
		(u64)frame->width * HISTB_VPSS_ZME_MAX_REDUCTION ||
	    (u64)input_height >=
		(u64)frame->height * HISTB_VPSS_ZME_MAX_REDUCTION)
		return -EINVAL;
	ret = histb_vpss_input_size(frame, &required_input_size);
	if (ret)
		return ret;
	if (frame->input_size && frame->input_size < required_input_size)
		return -EINVAL;
	if ((u64)frame->input_dma + required_input_size > U32_MAX + 1ULL)
		return -ERANGE;
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
		if (dei_opt & HISTB_VPSS_OPT_DETILE_REF)
			histb_vpss_dump_state(vpss, "detile");
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
	if (vpss->dei_st_cpu)
		dma_free_coherent(vpss->dev, vpss->dei_st_size,
				  vpss->dei_st_cpu, vpss->dei_st_dma);
	if (vpss->stt_cpu)
		dma_free_coherent(vpss->dev, vpss->stt_size,
				  vpss->stt_cpu, vpss->stt_dma);
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
