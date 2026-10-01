// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3798CV200 VDH video decoder
 *
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/log2.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/dma-map-ops.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm_runtime.h>
#include <linux/printk.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-vp9.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include "histb-vdec-mpeg4.h"
#include "histb-vdec-vc1.h"
#include "histb-vpss.h"

static const struct v4l2_event histb_vdec_eos_event = {
	.type = V4L2_EVENT_EOS,
};

#define HISTB_VDEC_START		0x0000
#define HISTB_VDEC_BASIC_CFG0		0x0008
#define HISTB_VDEC_BASIC_CFG1		0x000c
#define HISTB_VDEC_AVM_ADDR		0x0010
#define HISTB_VDEC_VAM_ADDR		0x0014
#define HISTB_VDEC_STREAM_BASE		0x0018
#define HISTB_VDEC_STATE		0x001c
#define HISTB_VDEC_INT_STATE		0x0020
#define HISTB_VDEC_INT_MASK		0x0024
#define HISTB_VDEC_VCTRL_STATE		0x0028
#define HISTB_VDEC_TIMEOUT_FIRST	0x003c
#define HISTB_VDEC_TIMEOUT_LAST		0x0054
#define HISTB_VDEC_STORE_PARAM		0x005c
#define HISTB_VDEC_CURRENT_Y		0x0060
#define HISTB_VDEC_Y_STRIDE		0x0064
#define HISTB_VDEC_CHROMA_OFFSET	0x0068
#define HISTB_VDEC_HEAD_INFO_OFFSET	0x006c
#define HISTB_VDEC_LINE_NUM_ADDR	0x0070
#define HISTB_VDEC_Y_STRIDE_2BIT		0x0074
#define HISTB_VDEC_Y_OFFSET_2BIT	0x0078
#define HISTB_VDEC_CHROMA_OFFSET_2BIT	0x007c
#define HISTB_VDEC_PPFD_ADDR		0x0080
#define HISTB_VDEC_PPFD_SIZE		0x0084
#define HISTB_VDEC_DNR_MBINFO_ADDR	0x0090
#define HISTB_VDEC_VC1_BPD_STRIDE	0x0074
#define HISTB_VDEC_VC1_MVTYPE_ADDR	0x0078
#define HISTB_VDEC_VC1_SKIP_ADDR		0x007c
#define HISTB_VDEC_VC1_DIRECT_ADDR	0x0080
#define HISTB_VDEC_VC1_ACPRED_ADDR	0x0084
#define HISTB_VDEC_VC1_OVERFLAGS_ADDR	0x0088
#define HISTB_VDEC_VC1_FIELDTX_ADDR	0x008c
#define HISTB_VDEC_VC1_FORWARD_ADDR	0x0090
#define HISTB_VDEC_REF_PIC_TYPE		0x0094
#define HISTB_VDEC_FF_APT_ENABLE	0x0098
#define HISTB_VDEC_DOWN_CLK_CFG		0x009c

#define HISTB_VDEC_SCD_AVS_FLAG		0xc000
#define HISTB_VDEC_SCD_EMAR_CFG		0xc004
#define HISTB_VDEC_SCD_EMAR_BASE	0x0000a1f7
#define HISTB_VDEC_SCD_EMAR_ENABLE	BIT(16)
#define HISTB_VDEC_SCD_VDH_SELRST	0xc008
#define HISTB_VDEC_SCD_CLOCK_GATE	0xc00c
#define HISTB_VDEC_SCD_INT_MASK		0xc81c
#define HISTB_VDEC_SCD_RESET_CLOCK	0xc880
#define HISTB_VDEC_SCD_DSP_CTRL		0xc100
#define HISTB_VDEC_SCD_DSP_STATE	0xc104
#define HISTB_VDEC_SCD_DSP_CODE_WORDS	0xc108
#define HISTB_VDEC_SCD_DSP_CODE_ADDR	0xc10c

#define HISTB_VDEC_SMMU_CTRL		0xf000
#define HISTB_VDEC_SMMU_INT_STATE_S	0xf018
#define HISTB_VDEC_SMMU_INT_CLEAR_S	0xf01c
#define HISTB_VDEC_SMMU_INT_MASK_NS	0xf020
#define HISTB_VDEC_SMMU_INT_STATE_NS	0xf028
#define HISTB_VDEC_SMMU_INT_CLEAR_NS	0xf02c
#define HISTB_VDEC_SMMU_FAULT_WR_S	0xf330
#define HISTB_VDEC_SMMU_FAULT_WR_NS	0xf340
#define HISTB_VDEC_SMMU_FAULT_RD_S	0xf350
#define HISTB_VDEC_SMMU_FAULT_RD_NS	0xf360
#define HISTB_VDEC_SMMU_ERR_RD_ADDR	0xf304
#define HISTB_VDEC_SMMU_ERR_WR_ADDR	0xf308
#define HISTB_VDEC_SMMU_PGTABLE_ADDR	0xf20c

#define HISTB_VDEC_SMMU_IOVA_SIZE	0xfff00000ULL
#define HISTB_VDEC_SMMU_DMA_SIZE	SZ_1G
#define HISTB_VDEC_SMMU_PAGE_SIZE	SZ_4K
#define HISTB_VDEC_SMMU_PGTABLE_ENTRIES \
	(HISTB_VDEC_SMMU_IOVA_SIZE / HISTB_VDEC_SMMU_PAGE_SIZE)
#define HISTB_VDEC_SMMU_PGTABLE_SIZE	SZ_4M
#define HISTB_VDEC_SMMU_PGTABLE_ALIGN	SZ_1M
#define HISTB_VDEC_SMMU_ERROR_SIZE	0x200U
#define HISTB_VDEC_SMMU_ERROR_ALIGN	0x100U
/*
 * The two error buffers are carved out of the page-table allocation.
 * Allocated on their own they land below 1 MiB, which the vendor domain
 * reserves, and histb_vdec_smmu_dma_range_valid() then rejects them.
 */
#define HISTB_VDEC_SMMU_ERROR_POOL \
	(2 * HISTB_VDEC_SMMU_ERROR_SIZE + HISTB_VDEC_SMMU_ERROR_ALIGN)

#define HISTB_CRG_VDH_CLOCK		0x0078
#define HISTB_CRG_BPD_CLOCK		0x0088
#define HISTB_CRG_RESET_STATUS		0x0174
#define HISTB_CRG_VDH_CLK_SEL		GENMASK(9, 8)
#define HISTB_CRG_VDH_CLK_SKIP		GENMASK(16, 12)
#define HISTB_CRG_VDH_CLK_LOAD		BIT(17)
#define HISTB_CRG_VDH_RESET		BIT(4)
#define HISTB_CRG_SCD_RESET		BIT(5)
#define HISTB_CRG_MFD_RESET		BIT(6)
#define HISTB_CRG_BPD_RESET		BIT(4)
#define HISTB_CRG_VDH_RESET_OK		BIT(0)
#define HISTB_CRG_SCD_RESET_OK		BIT(1)
#define HISTB_CRG_MFD_RESET_OK		BIT(2)
#define HISTB_CRG_BPD_RESET_OK		BIT(3)
#define HISTB_CRG_RESET_TIMEOUT_US	30000

#define HISTB_VDEC_INT_DONE		BIT(0)
#define HISTB_VDEC_STATE_DECODE_DONE	BIT(17)
#define HISTB_VDEC_STATE_DECODE_ERROR	BIT(18)
#define HISTB_VDEC_STATE_CABAC_END_ERROR	0x00060001
#define HISTB_VDEC_STATE_VP9_END		0x000e0000
#define HISTB_VDEC_INT_ENABLE_DONE	((u32)~HISTB_VDEC_INT_DONE)
#define HISTB_VDEC_TIMEOUT_VALUE	0x00300c03
#define HISTB_VDEC_BASIC_CFG0_H264	0x41000000
#define HISTB_VDEC_BASIC_CFG0_VC1	0x41000000
#define HISTB_VDEC_BASIC_CFG0_HEVC	0x01000000
#define HISTB_VDEC_BASIC_CFG0_MPEG2	0x41400000
#define HISTB_VDEC_BASIC_CFG0_VP8	0x00000000
#define HISTB_VDEC_BASIC_CFG0_VP9	0x00000000
#define HISTB_VDEC_BASIC_CFG0_AVS	0x41400000
#define HISTB_VDEC_BASIC_CFG1_I_SLICE	0x0001c000
#define HISTB_VDEC_BASIC_CFG1_NEW_PIC	BIT(14)
#define HISTB_VDEC_BASIC_CFG1_VC1	0x0001c001
#define HISTB_VDEC_BASIC_CFG1_MBAFF	BIT(4)
#define HISTB_VDEC_BASIC_CFG1_HEVC	0x0003c00d
#define HISTB_VDEC_BASIC_CFG1_MPEG2	0x00004003
#define HISTB_VDEC_BASIC_CFG1_MAX_SLICE_GROUP	GENMASK(27, 16)
#define HISTB_VDEC_BASIC_CFG1_VP8	0x0000c00c
#define HISTB_VDEC_BASIC_CFG1_VP9	0x0003c00e
#define HISTB_VDEC_BASIC_CFG1_AVS	(6 | BIT(14) | BIT(16))
#define HISTB_VDEC_BASIC_CFG1_AVS_MV_OUT	BIT(15)
#define HISTB_VDEC_BASIC_CFG1_AVS_MMU_EN	BIT(12)

#define HISTB_VDEC_PIC_WIDTH		GENMASK(8, 0)
#define HISTB_VDEC_PIC_STRUCTURE	GENMASK(15, 14)
#define HISTB_VDEC_PIC_HEIGHT		GENMASK(24, 16)
#define HISTB_VDEC_PIC_420		BIT(25)
#define HISTB_VDEC_PIC_CONSTRAINED_INTRA BIT(26)
#define HISTB_VDEC_PIC_ENTROPY_CODING	BIT(27)
#define HISTB_VDEC_PIC_TRANSFORM_8X8	BIT(28)
#define HISTB_VDEC_PIC_REFERENCE	BIT(31)

#define HISTB_VDEC_SLICE_FIRST_MB	GENMASK(19, 0)
#define HISTB_VDEC_SLICE_CABAC_INIT	GENMASK(25, 24)
#define HISTB_VDEC_SLICE_QP		GENMASK(31, 26)
#define HISTB_VDEC_SLICE_HW_TYPE	GENMASK(1, 0)
#define HISTB_VDEC_SLICE_TYPE_I		0
#define HISTB_VDEC_SLICE_TYPE_P		1
#define HISTB_VDEC_SLICE_TYPE_B		2
#define HISTB_VDEC_SLICE_LIST0_SIZE	GENMASK(7, 2)
#define HISTB_VDEC_SLICE_LIST1_SIZE	GENMASK(13, 8)
#define HISTB_VDEC_SLICE_DIRECT_8X8	BIT(14)
#define HISTB_VDEC_SLICE_DIRECT_SPATIAL	BIT(15)
#define HISTB_VDEC_SLICE_LIST0_ACTIVE	GENMASK(20, 16)
#define HISTB_VDEC_SLICE_LIST1_ACTIVE	GENMASK(25, 21)
#define HISTB_VDEC_SLICE_REF_MBAFF	BIT(0)
#define HISTB_VDEC_SLICE_REF_PAIR	BIT(1)
#define HISTB_VDEC_SLICE_REF_TOP		BIT(2)
#define HISTB_VDEC_SLICE_REF_LONG_TERM	BIT(3)
#define HISTB_VDEC_SLICE_CHROMA_QP2	GENMASK(4, 0)
#define HISTB_VDEC_SLICE_CHROMA_QP1	GENMASK(9, 5)
#define HISTB_VDEC_SLICE_WEIGHT		GENMASK(17, 16)
#define HISTB_VDEC_SLICE_DEBLOCK	GENMASK(1, 0)
#define HISTB_VDEC_SLICE_BETA		GENMASK(11, 8)
#define HISTB_VDEC_SLICE_ALPHA		GENMASK(19, 16)

/*
 * H.264 allows at most 16 reference frames, but MVC decodes one view per
 * picture with both views sharing this DPB, so the APC has to hold both
 * views' references: the vendor's MVC_MAX_FRAME_STORE is 40.  The array is
 * one of pointers, so the extra slots cost nothing worth counting, and the
 * loops that walk it treat an empty slot as absent.
 *
 * dpb_to_apc stays sized by the H.264 limit, because that array maps the
 * client's DPB entries, which the standard caps at 16 regardless of view.
 */
#define HISTB_VDEC_H264_DPB_SIZE	40
#define HISTB_VDEC_H264_DPB_MAP_SIZE	16
#define HISTB_VDEC_APC_INVALID		(-1)

#define HISTB_VDEC_UP_CABAC_END_ERROR	0x14230000

#define HISTB_VDEC_MIN_WIDTH		64U
#define HISTB_VDEC_MIN_HEIGHT		64U
#define HISTB_VDEC_H264_MAX_WIDTH	4096U
#define HISTB_VDEC_H264_MAX_HEIGHT	2304U
#define HISTB_VDEC_MPEG2_MAX_WIDTH	1920U
#define HISTB_VDEC_MPEG2_MAX_HEIGHT	1152U
#define HISTB_VDEC_MPEG4_MAX_WIDTH	1920U
#define HISTB_VDEC_MPEG4_MAX_HEIGHT	1088U
#define HISTB_VDEC_VC1_MAX_WIDTH	2048U
#define HISTB_VDEC_VC1_MAX_HEIGHT	2048U
#define HISTB_VDEC_VP8_MAX_WIDTH		1920U
#define HISTB_VDEC_VP8_MAX_HEIGHT	1088U
#define HISTB_VDEC_VP9_MAX_WIDTH		4096U
#define HISTB_VDEC_VP9_MAX_HEIGHT	2304U
#define HISTB_VDEC_HEVC_MAX_WIDTH	4096U
#define HISTB_VDEC_HEVC_MAX_HEIGHT	2304U
#define HISTB_VDEC_AVS_MAX_WIDTH	1920U
#define HISTB_VDEC_AVS_MAX_HEIGHT	1088U
#define HISTB_VDEC_DEFAULT_WIDTH	1280U
#define HISTB_VDEC_DEFAULT_HEIGHT	720U
#define HISTB_VDEC_MIN_BITSTREAM	SZ_4K
#define HISTB_VDEC_MAX_BITSTREAM	SZ_8M
#define HISTB_VDEC_DEFAULT_BITSTREAM	SZ_1M
#define HISTB_VDEC_WATCHDOG_MS		2000
#define HISTB_VDEC_VC1_BPD_TIMEOUT_MS	4000
#define HISTB_VDEC_VC1_BPD_MODE_ROWSKIP	5
#define HISTB_VDEC_RAW_STREAM_GUARD	64U

#define HISTB_VDEC_H264_MSG_SLOT_SIZE	SZ_1K
#define HISTB_VDEC_H264_MAX_SLICES	136
#define HISTB_VDEC_HEVC_MSG_SLOT_WORDS	320
#define HISTB_VDEC_HEVC_MSG_SLOT_SIZE	\
	(HISTB_VDEC_HEVC_MSG_SLOT_WORDS * sizeof(__le32))
#define HISTB_VDEC_UP_MSG_SLOT		0
#define HISTB_VDEC_PIC_MSG_SLOT		4
#define HISTB_VDEC_SLICE_MSG_SLOT	5
#define HISTB_VDEC_H264_MSG_SIZE		ALIGN((HISTB_VDEC_SLICE_MSG_SLOT + \
	HISTB_VDEC_H264_MAX_SLICES) * HISTB_VDEC_H264_MSG_SLOT_SIZE, SZ_4K)
#define HISTB_VDEC_QMATRIX_WORD		64
#define HISTB_VDEC_QMATRIX_WORDS	56
#define HISTB_VDEC_H264_CABAC_SIZE	5120
#define HISTB_VDEC_H264_CABAC_FIRMWARE	"hisilicon/histb-h264-cabac.bin"
#define HISTB_VDEC_HEVC_CABAC_SIZE	928
#define HISTB_VDEC_HEVC_CABAC_FIRMWARE	"hisilicon/histb-hevc-cabac.bin"
#define HISTB_VDEC_AVSP_FIRMWARE		"hisilicon/histb-avsp.bin"
#define HISTB_VDEC_AVSP_FIRMWARE_SIZE	17920U
#define HISTB_VDEC_AVSP_FIRMWARE_ALIGN	128U
#define HISTB_VDEC_AVS_PROFILE_JIZHUN	0x20
#define HISTB_VDEC_AVS_PROFILE_GUANGDIAN	0x48
#define HISTB_VDEC_AVS_START_SLICE_MAX	0xaf
#define HISTB_VDEC_AVS_PIC_MSG_WORDS	80U
#define HISTB_VDEC_AVS_PIC_MSG_SIZE	\
	(HISTB_VDEC_AVS_PIC_MSG_WORDS * sizeof(__le32))
#define HISTB_VDEC_AVS_SLICE_MSG_WORDS	64U
#define HISTB_VDEC_AVS_SLICE_MSG_SIZE	\
	(HISTB_VDEC_AVS_SLICE_MSG_WORDS * sizeof(__le32))
#define HISTB_VDEC_AVS_MAX_SLICES	200U
#define HISTB_VDEC_AVS_UP_MSG_SIZE	\
	(HISTB_VDEC_AVS_MAX_SLICES * 4U * sizeof(__le32))
#define HISTB_VDEC_AVS_PIC_MSG_OFFSET	ALIGN(HISTB_VDEC_AVS_UP_MSG_SIZE, SZ_4K)
#define HISTB_VDEC_AVS_SLICE_MSG_OFFSET	\
	(HISTB_VDEC_AVS_PIC_MSG_OFFSET + HISTB_VDEC_AVS_PIC_MSG_SIZE)
#define HISTB_VDEC_AVS_MSG_SIZE	\
	ALIGN(HISTB_VDEC_AVS_SLICE_MSG_OFFSET + \
	      HISTB_VDEC_AVS_MAX_SLICES * HISTB_VDEC_AVS_SLICE_MSG_SIZE, SZ_4K)
#define HISTB_VDEC_MAX_UP_REPORTS	200
/*
 * An 8 MiB-and-up capture buffer is a 4K geometry: 12.4 MB each out of a
 * 192 MiB CMA area.  Eleven of them is what H.264 4K uses and decodes
 * with; seventeen does not fit.
 */
#define HISTB_VDEC_MAX_LARGE_CAPTURE_BUFFERS	11
#define HISTB_VDEC_HEVC_MAX_ENTRY_POINTS	256
#define HISTB_VDEC_HEVC_MAX_SLICES	HISTB_VDEC_MAX_UP_REPORTS
#define HISTB_VDEC_HEVC_MAX_TILE_COLUMNS	10
#define HISTB_VDEC_HEVC_MAX_TILE_ROWS	11
#define HISTB_VDEC_HEVC_TILE_INFO_SIZE	2048
#define HISTB_VDEC_MPEG2_MAX_SLICES	1024
#define HISTB_VDEC_MPEG2_UP_MSG_SIZE	\
	(HISTB_VDEC_MPEG2_MAX_SLICES * 4 * sizeof(__le32))
#define HISTB_VDEC_MPEG2_PIC_MSG_SIZE	256
#define HISTB_VDEC_MPEG2_SLICE_MSG_SIZE	32
#define HISTB_VDEC_MPEG2_PIC_MSG_OFFSET	\
	ALIGN(HISTB_VDEC_MPEG2_UP_MSG_SIZE, SZ_4K)
#define HISTB_VDEC_MPEG2_SLICE_MSG_OFFSET	\
	(HISTB_VDEC_MPEG2_PIC_MSG_OFFSET + HISTB_VDEC_MPEG2_PIC_MSG_SIZE)
#define HISTB_VDEC_MPEG2_MSG_SIZE	\
	ALIGN(HISTB_VDEC_MPEG2_SLICE_MSG_OFFSET + \
	      HISTB_VDEC_MPEG2_MAX_SLICES * HISTB_VDEC_MPEG2_SLICE_MSG_SIZE, \
	      SZ_4K)
#define HISTB_VDEC_MPEG4_PIC_MSG_OFFSET	\
	(HISTB_VDEC_PIC_MSG_SLOT * HISTB_VDEC_H264_MSG_SLOT_SIZE)
#define HISTB_VDEC_MPEG4_SLICE_MSG_OFFSET	\
	(HISTB_VDEC_MPEG4_PIC_MSG_OFFSET + \
	 HISTB_MPEG4_PIC_MSG_WORDS * sizeof(__le32))
#define HISTB_VDEC_MPEG4_MSG_SIZE	\
	ALIGN(HISTB_VDEC_MPEG4_SLICE_MSG_OFFSET + \
	      HISTB_MPEG4_MAX_SLICE_MSGS * HISTB_MPEG4_SLICE_MSG_WORDS * \
	      sizeof(__le32), SZ_4K)
#define HISTB_VDEC_MPEG4_ITRANS_SIZE	45056U
#define HISTB_VDEC_VC1_PIC_MSG_OFFSET	\
	(HISTB_VDEC_PIC_MSG_SLOT * HISTB_VDEC_H264_MSG_SLOT_SIZE)
#define HISTB_VDEC_VC1_SLICE_MSG_OFFSET	\
	((HISTB_VDEC_PIC_MSG_SLOT + 1) * HISTB_VDEC_H264_MSG_SLOT_SIZE)
#define HISTB_VDEC_VC1_MSG_SIZE		\
	ALIGN(HISTB_VDEC_VC1_SLICE_MSG_OFFSET + \
	      HISTB_VC1_MAX_SLICES * HISTB_VC1_SLICE_MSG_WORDS * \
	      sizeof(__le32), SZ_4K)
#define HISTB_VDEC_VC1_BPD_SIZE		SZ_16K
#define HISTB_VDEC_VC1_INTENSITY_SIZE	HISTB_VC1_INTENSITY_WORK_SIZE
#define HISTB_VDEC_VP8_PROB_SIZE	2752
#define HISTB_VDEC_VP8_SEG_SIZE		SZ_32K
#define HISTB_VDEC_VP8_PARTITIONS	8
#define HISTB_VDEC_VP9_DPB_SIZE		8
#define HISTB_VDEC_VP9_LOGIC_IDS		9
#define HISTB_VDEC_VP9_PROB_SIZE		SZ_8K
#define HISTB_VDEC_VP9_COUNT_SIZE		(11 * SZ_1K)
#define HISTB_VDEC_VP9_SEG_SIZE		SZ_256K
#define HISTB_VDEC_VP9_SEG_STRIDE		2048
#define HISTB_VDEC_VP9_PMV_SIZE			ALIGN(256 * 144 * 64 + 16, 128)
#define HISTB_VDEC_VP9_MSG_WORDS		64
#define HISTB_VDEC_VP9_TILE_WORDS		64
#define HISTB_VDEC_VP9_MAX_TILES		1024
#define HISTB_VDEC_VP9_COEF_CONTEXTS	(4 * 2 * 2 * 6 * 6)
#define HISTB_VDEC_HEVC_MSG_SIZE		ALIGN((HISTB_VDEC_SLICE_MSG_SLOT + \
	HISTB_VDEC_HEVC_MAX_SLICES) * HISTB_VDEC_HEVC_MSG_SLOT_SIZE, SZ_4K)

enum histb_vdec_buffer_id {
	HISTB_VDEC_BUF_MSG,
	HISTB_VDEC_BUF_H264_CABAC,
	HISTB_VDEC_BUF_HEVC_CABAC,
	HISTB_VDEC_BUF_SED_TOP,
	HISTB_VDEC_BUF_PMV_TOP,
	HISTB_VDEC_BUF_PMV_LEFT,
	HISTB_VDEC_BUF_RCN_TOP,
	HISTB_VDEC_BUF_COLMB,
	HISTB_VDEC_BUF_TILE_SEG,
	HISTB_VDEC_BUF_APC_MV,
	HISTB_VDEC_BUF_SAO_LEFT,
	HISTB_VDEC_BUF_SAO_TOP,
	HISTB_VDEC_BUF_DBLK_LEFT,
	HISTB_VDEC_BUF_DBLK_TOP,
	HISTB_VDEC_BUF_PPFD,
	HISTB_VDEC_BUF_AVS_DNR_MBINFO,
	HISTB_VDEC_BUF_VP8_PROB,
	HISTB_VDEC_BUF_VP8_SEG,
	HISTB_VDEC_BUF_VP9_PROB,
	HISTB_VDEC_BUF_VP9_COUNT,
	HISTB_VDEC_BUF_VP9_PMV,
	HISTB_VDEC_BUF_VP9_SEG,
	HISTB_VDEC_BUF_VP9_SEG_WORK,
	HISTB_VDEC_BUF_MPEG4_ITRANS,
	HISTB_VDEC_BUF_MPEG4_DNR_MBINFO,
	HISTB_VDEC_BUF_MPEG4_PMV_TOP,
	HISTB_VDEC_BUF_MPEG4_SED_TOP,
	HISTB_VDEC_BUF_VC1_BPD,
	HISTB_VDEC_BUF_VC1_INTENSITY,
	HISTB_VDEC_BUF_COUNT,
};

static const size_t histb_vdec_buffer_sizes[HISTB_VDEC_BUF_COUNT] = {
	[HISTB_VDEC_BUF_MSG] = SZ_8K,
	[HISTB_VDEC_BUF_H264_CABAC] = SZ_8K,
	[HISTB_VDEC_BUF_HEVC_CABAC] = SZ_4K,
	[HISTB_VDEC_BUF_SED_TOP] = SZ_512K,
	[HISTB_VDEC_BUF_PMV_TOP] = SZ_512K,
	[HISTB_VDEC_BUF_PMV_LEFT] = SZ_512K,
	[HISTB_VDEC_BUF_RCN_TOP] = SZ_512K,
	[HISTB_VDEC_BUF_COLMB] = SZ_512K,
	[HISTB_VDEC_BUF_TILE_SEG] = SZ_256K,
	[HISTB_VDEC_BUF_APC_MV] = SZ_4K,
	[HISTB_VDEC_BUF_SAO_LEFT] = SZ_512K,
	[HISTB_VDEC_BUF_SAO_TOP] = SZ_512K,
	[HISTB_VDEC_BUF_DBLK_LEFT] = SZ_512K,
	[HISTB_VDEC_BUF_DBLK_TOP] = SZ_512K,
	[HISTB_VDEC_BUF_PPFD] = SZ_256K,
	[HISTB_VDEC_BUF_AVS_DNR_MBINFO] = SZ_32K,
	[HISTB_VDEC_BUF_VP8_PROB] = ALIGN(HISTB_VDEC_VP8_PROB_SIZE, SZ_4K),
	[HISTB_VDEC_BUF_VP8_SEG] = HISTB_VDEC_VP8_SEG_SIZE,
	[HISTB_VDEC_BUF_VP9_PROB] = HISTB_VDEC_VP9_PROB_SIZE,
	[HISTB_VDEC_BUF_VP9_COUNT] = HISTB_VDEC_VP9_COUNT_SIZE,
	[HISTB_VDEC_BUF_VP9_PMV] = HISTB_VDEC_VP9_PMV_SIZE,
	[HISTB_VDEC_BUF_VP9_SEG] = HISTB_VDEC_VP9_SEG_SIZE,
	[HISTB_VDEC_BUF_VP9_SEG_WORK] = HISTB_VDEC_VP9_SEG_SIZE,
	[HISTB_VDEC_BUF_MPEG4_ITRANS] = HISTB_VDEC_MPEG4_ITRANS_SIZE,
	[HISTB_VDEC_BUF_MPEG4_DNR_MBINFO] = SZ_64K,
	[HISTB_VDEC_BUF_MPEG4_PMV_TOP] = SZ_1M,
	[HISTB_VDEC_BUF_MPEG4_SED_TOP] = SZ_1M,
	[HISTB_VDEC_BUF_VC1_BPD] = HISTB_VDEC_VC1_BPD_SIZE,
	[HISTB_VDEC_BUF_VC1_INTENSITY] = HISTB_VDEC_VC1_INTENSITY_SIZE,
};

struct histb_vdec_dma_buffer {
	void *cpu;
	dma_addr_t dma;
	size_t size;
};

struct histb_vdec_hevc_stream {
	u32 data_offset;
	u32 valid_bits;
};

struct histb_vdec_avs_stream {
	u32 data_offset;
	u32 size;
};

struct histb_vdec_hevc_tiles {
	u32 columns;
	u32 rows;
	u32 column_bd[HISTB_VDEC_HEVC_MAX_TILE_COLUMNS + 1];
	u32 row_bd[HISTB_VDEC_HEVC_MAX_TILE_ROWS + 1];
};

struct histb_vdec_mpeg2_slice {
	u32 data_offset;
	u32 valid_bits;
	u32 start_mb;
	u8 bit_offset;
	u8 quantiser_scale;
	bool intra;
};

struct histb_vdec_avs_metadata {
	bool valid;
	bool anchor;
	bool top_field_first;
	u8 picture_structure;
	u8 picture_coding_type;
	u16 picture_distance;
};

struct histb_vdec_decoded_buffer {
	/* Must be first for the v4l2-mem2mem ready-queue helpers. */
	struct v4l2_m2m_buffer base;
	struct histb_vdec_dma_buffer tile;
	struct histb_vdec_dma_buffer pmv;
	struct histb_vdec_avs_metadata avs;
	bool tile_noncoherent;
	bool pmv_noncoherent;
	/* A decoded surface remains bad until userspace QBUF starts a new life. */
	bool error_tainted;
	/* MPEG-2 field pairs are stored in one full-frame reconstruction surface. */
	bool mpeg2_field_saved;
	s8 apc_slot;
	/* VP9 stores codec DPB ownership separately from the H.264 APC. */
	bool vp9_dpb_valid;
	s8 vp9_logic_id;
	u16 vp9_width;
	u16 vp9_height;
};

struct histb_vdec_avs_reference {
	struct histb_vdec_decoded_buffer *decoded;
	const struct histb_vdec_avs_metadata *metadata;
};

struct histb_vdec_mpeg4_anchor {
	struct histb_vdec_dma_buffer tile;
	struct histb_vdec_dma_buffer pmv;
	bool tile_noncoherent;
};

struct histb_vdec_vc1_anchor {
	struct histb_vdec_dma_buffer tile;
	struct histb_vdec_dma_buffer pmv;
	struct histb_vc1_intensity_map intensity;
	bool tile_noncoherent;
	bool halfpel;
	bool range_reduction;
	u8 fcm;
	u8 ref_dist;
	u8 res_pic;
};

enum histb_vdec_vc1_display_state {
	HISTB_VDEC_VC1_DISPLAY_NONE,
	HISTB_VDEC_VC1_DISPLAY_BUFFER,
	HISTB_VDEC_VC1_DISPLAY_DROPPED,
};

enum histb_vdec_job_phase {
	HISTB_VDEC_PHASE_IDLE,
	HISTB_VDEC_PHASE_BPD,
	HISTB_VDEC_PHASE_VDH,
};

struct histb_vdec_vp9_counts {
	struct v4l2_vp9_frame_symbol_counts map;
	u32 eob[HISTB_VDEC_VP9_COEF_CONTEXTS];
	u32 tx16[2][4];
};

struct histb_vdec_q_data {
	struct v4l2_pix_format pix;
	u32 sequence;
};

struct histb_vdec_dev;

struct histb_vdec_ctx {
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler ctrl_handler;
	struct histb_vdec_dev *vdec;
	struct histb_vdec_q_data src;
	struct histb_vdec_q_data dst;
	/* Serializes MPEG-4 AU/display completion with decoder commands. */
	struct mutex mpeg4_lock;
	struct histb_vdec_dma_buffer buffers[HISTB_VDEC_BUF_COUNT];
	/* AVS only: one allocation carries every entry of buffers[]. */
	struct histb_vdec_dma_buffer buf_arena;
	struct histb_vdec_dma_buffer bitstream;
	struct histb_vdec_dma_buffer vc1_stream;
	struct histb_vdec_decoded_buffer *apc[HISTB_VDEC_H264_DPB_SIZE];
	u32 total_mbs;
	u32 frame_flags;
	bool h264_partial;
	bool h264_new_frame;
	bool h264_new_surface;
	bool h264_last_slice;
	bool h264_field_picture;
	bool h264_bottom_field;
	bool h264_second_field;
	bool h264_field_transaction;
	u32 h264_first_mb;
	u32 h264_last_first_mb;
	size_t h264_stream_bytes;
	u16 h264_slices;
	bool h264_controls_valid;
	struct v4l2_ctrl_h264_decode_params h264_decode;
	struct v4l2_ctrl_h264_sps h264_sps;
	struct v4l2_ctrl_h264_pps h264_pps;
	struct v4l2_ctrl_h264_scaling_matrix h264_scaling;
	__le32 h264_picture_message[HISTB_VDEC_QMATRIX_WORD +
		HISTB_VDEC_QMATRIX_WORDS];
	struct histb_vdec_decoded_buffer *h264_pending_field;
	u64 h264_pending_timestamp;
	u32 h264_pending_sequence;
	u16 h264_pending_frame_num;
	u16 h264_pending_width_mbs;
	u16 h264_pending_height_map_units;
	u16 h264_field_frame_num;
	u16 h264_field_width_mbs;
	u16 h264_field_height_map_units;
	bool h264_pending_bottom;
	bool h264_pending_failed;
	bool h264_cabac_loaded;
	bool hevc_cabac_loaded;
	bool avs;
	u16 avs_slices;
	u32 avs_total_mbs;
	u32 avs_basic_cfg1;
	u8 avs_profile;
	bool hevc;
	bool hevc_main10;
	bool hevc_scaling_list;
	u16 hevc_slices;
	u16 mpeg2_slices;
	bool mpeg2;
	bool mpeg2_reference_started;
	bool mpeg2_picture_started;
	bool mpeg2_picture_valid;
	bool mpeg2_field_picture;
	bool mpeg2_second_field;
	bool mpeg2_pending_failed;
	u8 mpeg2_picture_structure;
	u8 mpeg2_picture_coding_type;
	u8 mpeg2_pending_structure;
	u8 mpeg2_pending_coding_type;
	u16 mpeg2_picture_width_mbs;
	u16 mpeg2_picture_height_mbs;
	u16 mpeg2_pending_width_mbs;
	u16 mpeg2_pending_frame_height_mbs;
	u32 mpeg2_pending_sequence;
	u32 mpeg2_pending_frame_flags;
	u32 mpeg2_ref_pic_type;
	u64 mpeg2_pending_timestamp;
	struct histb_vdec_decoded_buffer *mpeg2_pending_field;
	/* MPEG-2 B pictures can straddle an I-picture boundary. */
	struct histb_vdec_decoded_buffer *mpeg2_anchor[2];
	bool mpeg4;
	bool mpeg4_pending_valid;
	u16 mpeg4_slices;
	struct histb_mpeg4_parser mpeg4_parser;
	struct histb_mpeg4_parser mpeg4_pending_parser;
	struct histb_mpeg4_frame mpeg4_pending_frame;
	struct histb_mpeg4_regs mpeg4_pending_regs;
	struct histb_vdec_decoded_buffer *mpeg4_pending_output;
	struct vb2_v4l2_buffer *mpeg4_au_src;
	struct vb2_v4l2_buffer *mpeg4_display_pending;
	u32 mpeg4_au_bytes;
	u32 mpeg4_au_offset;
	u32 mpeg4_pending_next_offset;
	bool mpeg4_au_active;
	bool mpeg4_pending_source_done;
	struct histb_mpeg4_display_state mpeg4_display_state;
	struct histb_vdec_mpeg4_anchor mpeg4_ref[2];
	/* Serializes VC-1 reference and delayed-display ownership. */
	struct mutex vc1_lock;
	struct completion vc1_setup_idle;
	bool vc1;
	bool vc1_annex_l;
	bool vc1_pending_valid;
	bool vc1_need_intra;
	bool vc1_smp_sequence_valid;
	u8 vc1_smp_rounding;
	u8 vc1_smp_res_pic;
	struct histb_vc1_sequence vc1_sequence;
	struct histb_vc1_entry_point vc1_entry;
	struct histb_vc1_smp_sequence vc1_smp_sequence;
	struct histb_vc1_field_transaction vc1_field_transaction;
	struct histb_vc1_parsed_picture vc1_pending_picture;
	struct histb_vdec_decoded_buffer *vc1_pending_output;
	u32 vc1_pending_stream_base;
	u8 vc1_pending_ref_pic_type;
	u16 vc1_slices;
	struct vb2_v4l2_buffer *vc1_au_src;
	struct vb2_v4l2_buffer *vc1_display_pending;
	enum histb_vdec_vc1_display_state vc1_display_state;
	u32 vc1_au_bytes;
	u32 vc1_au_offset;
	u32 vc1_pending_next_offset;
	bool vc1_au_active;
	bool vc1_pending_source_done;
	struct histb_vdec_vc1_anchor vc1_ref[2];
	struct histb_vc1_intensity_map vc1_pending_earlier_intensity;
	bool vc1_pending_intensity_valid;
	u8 vc1_pending_halfpel;
	bool vp8;
	bool vp9;
	bool vp9_have_state;
	struct histb_vdec_decoded_buffer *vp9_dpb[HISTB_VDEC_VP9_DPB_SIZE];
	struct histb_vdec_decoded_buffer *vp9_pending_dpb[HISTB_VDEC_VP9_DPB_SIZE];
	struct histb_vdec_decoded_buffer *vp9_pending_output;
	u8 vp9_pending_refresh;
	s8 vp9_pending_logic_id;
	u8 vp9_pending_ref_idx[3];
	u8 vp9_pending_sign_bias;
	u16 vp9_prev_width;
	u16 vp9_prev_height;
	u16 vp9_pending_width;
	u16 vp9_pending_height;
	bool vp9_pending_valid;
	struct v4l2_ctrl_vp9_frame vp9_pending_frame;
	struct v4l2_ctrl_vp9_compressed_hdr vp9_pending_compressed_hdr;
	struct v4l2_vp9_segmentation vp9_seg;
	struct v4l2_vp9_frame_context vp9_frame_ctx[V4L2_VP9_NUM_FRAME_CTX];
	struct v4l2_vp9_frame_context vp9_pending_frame_ctx[V4L2_VP9_NUM_FRAME_CTX];
	struct v4l2_vp9_frame_context vp9_pending_probs;
	struct histb_vdec_vp9_counts vp9_counts;
	u32 vp9_last_frame_flags;
	bool vp8_have_state;
	u8 vp8_last_frame_type;
	u8 vp8_last_filter_type;
	u8 vp8_last_sharpness;
	u8 vp8_pending_frame_type;
	u8 vp8_pending_filter_type;
	u8 vp8_pending_sharpness;
	bool cabac;
	bool mbaff;
};

struct histb_vdec_dev {
	struct device *dev;
	void __iomem *regs;
	struct regmap *crg;
	struct histb_vpss *vpss;
	struct clk_bulk_data clocks[3];
	struct reset_control_bulk_data resets[4];
	int irq;
	struct histb_vdec_dma_buffer avsp_firmware;
	/* CV200 AVS requires VDH translation even when DMA addresses are identity. */
	struct histb_vdec_dma_buffer smmu_pt_raw;
	struct histb_vdec_dma_buffer smmu_err_rd_raw;
	struct histb_vdec_dma_buffer smmu_err_wr_raw;
	void *smmu_pt_cpu;
	void *smmu_err_rd_cpu;
	void *smmu_err_wr_cpu;
	dma_addr_t smmu_pt_dma;
	dma_addr_t smmu_err_rd_dma;
	dma_addr_t smmu_err_wr_dma;
	bool smmu_identity_ready;
	bool smmu_identity_needs_reset;
	bool avsp_available;
	bool avsp_loaded;

	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	struct video_device vfd;
	struct v4l2_m2m_dev *m2m_dev;
	/* Serializes ioctls and both videobuf2 queues. */
	struct mutex lock;

	/* Protects the active context from IRQ and watchdog races. */
	spinlock_t irqlock;
	/* Serializes the final cancellation check with hardware doorbells. */
	struct mutex launch_lock;
	struct histb_vdec_ctx *curr_ctx;
	enum histb_vdec_job_phase phase;
	bool job_cancelled;
	struct histb_vdec_ctx *post_ctx;
	bool post_abort;
	struct delayed_work watchdog_work;
	struct work_struct postprocess_work;
	struct completion postprocess_idle;
	/* Count each active hardware job once, shared with sysfs readers. */
	bool job_count_pending;
	u64 jobs;
	u64 errors;
};

struct histb_vdec_capture_mem {
	const struct vb2_mem_ops *ops;
	void *priv;
	bool cpu_synced;
};

/*
 * Cacheable capture buffers.
 *
 * The queue used to hand NV12 output to userspace from
 * dma_alloc_coherent(), which on arm64 with DMA_DIRECT_REMAP is uncached
 * memory.  Every consumer read of a decoded frame then went straight to
 * LPDDR: ffmpeg's av_frame_copy() alone accounted for 78% of the CPU time in
 * a 1080p decode, and the hardware path ended up slower than the software
 * one, which writes into ordinary cached memory.
 *
 * These buffers are allocated with dma_alloc_noncoherent() instead, the same
 * primitive the reconstruction surfaces already use: physically contiguous
 * (VPSS writes by DMA address, and the SMMU is configured as an identity
 * mapping, so contiguity is required) but cacheable, with the cache
 * maintained explicitly around the handover.
 */
struct histb_vdec_cached_buf {
	struct device *dev;
	void *cpu;
	dma_addr_t dma;
	size_t size;
	unsigned int refs;
	bool noncoherent;
};

static void *histb_vdec_cached_alloc(struct vb2_buffer *vb,
				     struct device *dev, unsigned long size)
{
	struct histb_vdec_cached_buf *buf;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->dev = dev;
	buf->size = size;
	buf->cpu = dma_alloc_noncoherent(dev, size, &buf->dma,
					 DMA_BIDIRECTIONAL, GFP_KERNEL);
	buf->noncoherent = buf->cpu != NULL;
	if (!buf->cpu)
		buf->cpu = dma_alloc_coherent(dev, size, &buf->dma, GFP_KERNEL);
	if (!buf->cpu) {
		kfree(buf);
		return ERR_PTR(-ENOMEM);
	}
	buf->refs = 1;

	return buf;
}

static void histb_vdec_cached_free(struct histb_vdec_cached_buf *buf)
{
	if (buf->noncoherent)
		dma_free_noncoherent(buf->dev, buf->size, buf->cpu, buf->dma,
				     DMA_BIDIRECTIONAL);
	else
		dma_free_coherent(buf->dev, buf->size, buf->cpu, buf->dma);
	kfree(buf);
}

static void histb_vdec_cached_put(void *buf_priv)
{
	struct histb_vdec_cached_buf *buf = buf_priv;

	if (!buf)
		return;
	if (--buf->refs)
		return;

	histb_vdec_cached_free(buf);
}

static unsigned int histb_vdec_cached_num_users(void *buf_priv)
{
	return buf_priv ? ((struct histb_vdec_cached_buf *)buf_priv)->refs : 0;
}

static void *histb_vdec_cached_vaddr(struct vb2_buffer *vb, void *buf_priv)
{
	return buf_priv ? ((struct histb_vdec_cached_buf *)buf_priv)->cpu : NULL;
}

/* Called before the hardware is pointed at the buffer. */
static void histb_vdec_cached_prepare(void *buf_priv)
{
	struct histb_vdec_cached_buf *buf = buf_priv;

	if (buf && buf->noncoherent)
		dma_sync_single_for_device(buf->dev, buf->dma, buf->size,
					   DMA_TO_DEVICE);
}

/* Called before the CPU reads what the hardware wrote. */
static void histb_vdec_cached_finish(void *buf_priv)
{
	struct histb_vdec_cached_buf *buf = buf_priv;

	if (buf && buf->noncoherent)
		dma_sync_single_for_cpu(buf->dev, buf->dma, buf->size,
					DMA_FROM_DEVICE);
}

/*
 * dma_buf export for the cacheable capture buffers.
 *
 * ffmpeg's v4l2request refuses a capture queue whose buffers cannot be
 * exported - `Failed to export capture buffer` in its log - and then falls
 * back to copying every frame out of the decoder, which is the single largest
 * cost in the whole pipeline.  vb2_dma_contig_memops has an export path but
 * allocates uncached memory; vb2_dma_sg allocates cacheable pages but is not
 * contiguous, and VPSS walks the surface as base + offset, so it cannot be
 * used here.  Hence a small export implementation for these buffers.
 *
 * The buffer is physically contiguous, so the sg table has a single entry.
 */
static struct sg_table *histb_vdec_cached_dmabuf_map(
		struct dma_buf_attachment *attach,
		enum dma_data_direction dir)
{
	struct histb_vdec_cached_buf *buf = attach->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}

	sg_set_page(sgt->sgl, NULL, 0, 0);
	sg_dma_address(sgt->sgl) = buf->dma;
	sg_dma_len(sgt->sgl) = buf->size;

	return sgt;
}

static void histb_vdec_cached_dmabuf_unmap(struct dma_buf_attachment *attach,
					   struct sg_table *sgt,
					   enum dma_data_direction dir)
{
	sg_free_table(sgt);
	kfree(sgt);
}

static int histb_vdec_cached_dmabuf_mmap(struct dma_buf *dbuf,
					 struct vm_area_struct *vma)
{
	struct histb_vdec_cached_buf *buf = dbuf->priv;
	unsigned long size = vma->vm_end - vma->vm_start;

	if (size > buf->size)
		return -EINVAL;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start, buf->dma >> PAGE_SHIFT,
			       size, vma->vm_page_prot);
}

static int histb_vdec_cached_mmap(void *buf_priv, struct vm_area_struct *vma)
{
	struct histb_vdec_cached_buf *buf = buf_priv;

	if (!buf)
		return -ENODEV;
	/*
	 * No attributes: 6.12 has no DMA_ATTR_NON_CONSISTENT, and the
	 * noncoherent allocation was made with dma_alloc_noncoherent(), whose
	 * mapping is what dma_mmap_attrs() with attrs 0 reproduces.  This is
	 * the same pair vb2_dma_contig uses.
	 */
	return dma_mmap_attrs(buf->dev, vma, buf->cpu, buf->dma, buf->size, 0);
}

static void histb_vdec_cached_dmabuf_release(struct dma_buf *dbuf)
{
	struct histb_vdec_cached_buf *buf = dbuf->priv;

	/*
	 * dma_buf_release() frees the dma_buf itself, so this must not.  Doing
	 * so produced a double free, and because the second free ran on an
	 * object the core had already torn down, dma_resv_fini() decremented a
	 * zero refcount and the kernel reported "refcount_t: underflow;
	 * use-after-free" before oopsing in dma_resv_fini().
	 *
	 * The export holds one reference on the buffer.  Dropping it here is
	 * enough when vb2 still holds its own; if vb2 has already been put,
	 * this was the last reference and the memory has to go.
	 */
	if (--buf->refs == 0)
		histb_vdec_cached_free(buf);
}

static const struct dma_buf_ops histb_vdec_cached_dmabuf_ops = {
	.map_dma_buf = histb_vdec_cached_dmabuf_map,
	.unmap_dma_buf = histb_vdec_cached_dmabuf_unmap,
	.mmap = histb_vdec_cached_dmabuf_mmap,
	.release = histb_vdec_cached_dmabuf_release,
};

static struct dma_buf *histb_vdec_cached_dmabuf_export(
		struct vb2_buffer *vb, void *buf_priv, unsigned long flags)
{
	struct histb_vdec_cached_buf *buf = buf_priv;
	struct dma_buf *dbuf;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	exp_info.ops = &histb_vdec_cached_dmabuf_ops;
	exp_info.size = buf->size;
	exp_info.flags = flags;
	exp_info.priv = buf;

	dbuf = dma_buf_export(&exp_info);
	if (IS_ERR(dbuf))
		return dbuf;

	buf->refs++;
	return dbuf;
}

static const struct vb2_mem_ops histb_vdec_cached_memops = {
	.alloc = histb_vdec_cached_alloc,
	.put = histb_vdec_cached_put,
	.num_users = histb_vdec_cached_num_users,
	.vaddr = histb_vdec_cached_vaddr,
	.prepare = histb_vdec_cached_prepare,
	.finish = histb_vdec_cached_finish,
	.get_dmabuf = histb_vdec_cached_dmabuf_export,
	.mmap = histb_vdec_cached_mmap,
};


static const struct vb2_mem_ops *
histb_vdec_capture_backend(const struct vb2_buffer *vb)
{
	struct histb_vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type) &&
	    vb->vb2_queue->memory == VB2_MEMORY_MMAP)
		return &histb_vdec_cached_memops;

	return ctx->dst.pix.pixelformat == V4L2_PIX_FMT_NV12 ?
		&vb2_dma_contig_memops : &vb2_vmalloc_memops;
}

static void *histb_vdec_capture_alloc(struct vb2_buffer *vb,
				      struct device *dev, unsigned long size)
{
	struct histb_vdec_capture_mem *mem;

	mem = kzalloc(sizeof(*mem), GFP_KERNEL);
	if (!mem)
		return ERR_PTR(-ENOMEM);
	mem->ops = histb_vdec_capture_backend(vb);
	mem->priv = mem->ops->alloc(vb, dev, size);
	if (IS_ERR(mem->priv)) {
		void *ret = mem->priv;

		kfree(mem);
		return ret;
	}

	return mem;
}

static void histb_vdec_capture_put(void *buf_priv)
{
	struct histb_vdec_capture_mem *mem = buf_priv;

	if (mem->ops->put)
		mem->ops->put(mem->priv);
	kfree(mem);
}

static struct dma_buf *
histb_vdec_capture_get_dmabuf(struct vb2_buffer *vb, void *buf_priv,
			      unsigned long flags)
{
	struct histb_vdec_capture_mem *mem = buf_priv;

	/*
	 * Not every backend exports a dma_buf - vb2_vmalloc_memops and the
	 * cacheable backend below do not - so this cannot be forwarded
	 * unconditionally.  Calling through a NULL function pointer is an
	 * immediate oops; returning an error lets the caller fall back.
	 */
	if (!mem->ops->get_dmabuf)
		return ERR_PTR(-EOPNOTSUPP);

	return mem->ops->get_dmabuf(vb, mem->priv, flags);
}

static void histb_vdec_capture_prepare(void *buf_priv)
{
	struct histb_vdec_capture_mem *mem = buf_priv;

	mem->cpu_synced = false;
	if (mem->ops->prepare)
		mem->ops->prepare(mem->priv);
}

static void histb_vdec_capture_finish(void *buf_priv)
{
	struct histb_vdec_capture_mem *mem = buf_priv;

	if (mem->cpu_synced) {
		mem->cpu_synced = false;
		return;
	}
	if (mem->ops->finish)
		mem->ops->finish(mem->priv);
}

static int histb_vdec_capture_begin_cpu_access(struct vb2_buffer *vb)
{
	struct histb_vdec_capture_mem *mem;

	if (!vb || !vb->num_planes || !vb->planes[0].mem_priv)
		return -EINVAL;
	mem = vb->planes[0].mem_priv;
	if (!mem->cpu_synced) {
		if (mem->ops->finish)
			mem->ops->finish(mem->priv);
		mem->cpu_synced = true;
	}
	return 0;
}

static void *histb_vdec_capture_attach_dmabuf(struct vb2_buffer *vb,
					      struct device *dev,
					     struct dma_buf *dbuf,
					     unsigned long size)
{
	struct histb_vdec_capture_mem *mem;

	mem = kzalloc(sizeof(*mem), GFP_KERNEL);
	if (!mem)
		return ERR_PTR(-ENOMEM);
	mem->ops = histb_vdec_capture_backend(vb);
	if (!mem->ops->attach_dmabuf)
		return ERR_PTR(-ENODEV);
	mem->priv = mem->ops->attach_dmabuf(vb, dev, dbuf, size);
	if (IS_ERR(mem->priv)) {
		void *ret = mem->priv;

		kfree(mem);
		return ret;
	}

	return mem;
}

static void histb_vdec_capture_detach_dmabuf(void *buf_priv)
{
	struct histb_vdec_capture_mem *mem = buf_priv;

	if (mem->ops->detach_dmabuf)
		mem->ops->detach_dmabuf(mem->priv);
	kfree(mem);
}

static int histb_vdec_capture_map_dmabuf(void *buf_priv)
{
	struct histb_vdec_capture_mem *mem = buf_priv;

	if (!mem->ops->map_dmabuf)
		return -ENODEV;

	return mem->ops->map_dmabuf(mem->priv);
}

static void histb_vdec_capture_unmap_dmabuf(void *buf_priv)
{
	struct histb_vdec_capture_mem *mem = buf_priv;

	if (mem->ops->unmap_dmabuf)
		mem->ops->unmap_dmabuf(mem->priv);
}

static void *histb_vdec_capture_vaddr(struct vb2_buffer *vb, void *buf_priv)
{
	struct histb_vdec_capture_mem *mem = buf_priv;

	if (!mem->ops->vaddr)
		return NULL;

	return mem->ops->vaddr(vb, mem->priv);
}

static void *histb_vdec_capture_cookie(struct vb2_buffer *vb, void *buf_priv)
{
	struct histb_vdec_capture_mem *mem = buf_priv;

	if (!mem->ops->cookie)
		return NULL;
	return mem->ops->cookie(vb, mem->priv);
}

static unsigned int histb_vdec_capture_num_users(void *buf_priv)
{
	struct histb_vdec_capture_mem *mem = buf_priv;

	return mem->ops->num_users(mem->priv);
}

static int histb_vdec_capture_mmap(void *buf_priv, struct vm_area_struct *vma)
{
	struct histb_vdec_capture_mem *mem = buf_priv;

	if (!mem->ops->mmap)
		return -ENODEV;

	return mem->ops->mmap(mem->priv, vma);
}

static int histb_vdec_capture_begin_cpu_access(struct vb2_buffer *vb);

static const struct vb2_mem_ops histb_vdec_capture_memops = {
	.alloc = histb_vdec_capture_alloc,
	.put = histb_vdec_capture_put,
	.get_dmabuf = histb_vdec_capture_get_dmabuf,
	.prepare = histb_vdec_capture_prepare,
	.finish = histb_vdec_capture_finish,
	.attach_dmabuf = histb_vdec_capture_attach_dmabuf,
	.detach_dmabuf = histb_vdec_capture_detach_dmabuf,
	.map_dmabuf = histb_vdec_capture_map_dmabuf,
	.unmap_dmabuf = histb_vdec_capture_unmap_dmabuf,
	.vaddr = histb_vdec_capture_vaddr,
	.cookie = histb_vdec_capture_cookie,
	.num_users = histb_vdec_capture_num_users,
	.mmap = histb_vdec_capture_mmap,
};

static bool histb_vdec_is_vc1_format(u32 pixelformat)
{
	return pixelformat == V4L2_PIX_FMT_VC1_ANNEX_G ||
	       pixelformat == V4L2_PIX_FMT_VC1_ANNEX_L;
}

static bool histb_vdec_is_stateless_format(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_H264_SLICE:
	case V4L2_PIX_FMT_HEVC_SLICE:
	case V4L2_PIX_FMT_MPEG1_SLICE:
	case V4L2_PIX_FMT_MPEG2_SLICE:
	case V4L2_PIX_FMT_VP8_FRAME:
	case V4L2_PIX_FMT_VP9_FRAME:
	case V4L2_PIX_FMT_AVS_SLICE:
		return true;
	default:
		return false;
	}
}

static u32 histb_vdec_tile_stride(u32 width)
{
	u32 blocks = width / 256;

	if (!(width % 256))
		return width;

	return (blocks + (blocks & 1 ? 2 : 1)) * 256;
}

static u32 histb_vdec_surface_stride(struct histb_vdec_ctx *ctx)
{
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_HEVC_SLICE ||
	    ctx->src.pix.pixelformat == V4L2_PIX_FMT_AVS_SLICE ||
	    ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG4 ||
	    histb_vdec_is_vc1_format(ctx->src.pix.pixelformat) ||
	    ctx->src.pix.pixelformat == V4L2_PIX_FMT_VP9_FRAME)
		return ALIGN(ctx->dst.pix.width, 256);

	return histb_vdec_tile_stride(ctx->dst.pix.width);
}

static u32 histb_vdec_surface_height_align(struct histb_vdec_ctx *ctx)
{
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG2_SLICE &&
	    ctx->mpeg2_field_picture)
		return 32;
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_HEVC_SLICE ||
	    ctx->src.pix.pixelformat == V4L2_PIX_FMT_VP9_FRAME)
		return 64;
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_AVS_SLICE ||
	    ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG4 ||
	    histb_vdec_is_vc1_format(ctx->src.pix.pixelformat))
		return 32;

	return 16;
}

static u32 histb_vdec_avs_height_mbs(u16 height, bool progressive)
{
	return progressive ? DIV_ROUND_UP(height, 16) :
		2 * DIV_ROUND_UP(height, 32);
}

static size_t histb_vdec_ebsp_to_rbsp(u8 *data, size_t size)
{
	size_t read, write = 0;
	unsigned int zeroes = 0;

	for (read = 0; read < size; read++) {
		u8 byte = data[read];

		if (zeroes == 2 && byte == 3 && read + 1 < size &&
		    data[read + 1] <= 3) {
			zeroes = 0;
			continue;
		}
		data[write++] = byte;
		if (byte)
			zeroes = 0;
		else
			zeroes = min(zeroes + 1, 2U);
	}
	/* VDH fetches full bus bursts around the valid stream-bit boundary. */
	memset(data + write, 0, size - write);
	return write;
}

static int histb_vdec_append_h264_bitstream(struct histb_vdec_ctx *ctx,
					     struct vb2_v4l2_buffer *src,
					     unsigned long *payload,
					     dma_addr_t *slice_dma,
					     u32 *tail_bits)
{
	struct histb_vdec_dma_buffer *stream = &ctx->bitstream;
	struct histb_vdec_dev *vdec = ctx->vdec;
	struct histb_vdec_dma_buffer replacement = { };
	size_t ebsp_size = *payload;
	size_t allocation, data_end, offset, required, tail_bytes;
	u8 *src_cpu;

	offset = ALIGN(ctx->h264_stream_bytes, 16);
	if (check_add_overflow(offset, ebsp_size, &data_end) ||
	    check_add_overflow(data_end, HISTB_VDEC_RAW_STREAM_GUARD,
			       &required) ||
	    required > HISTB_VDEC_MAX_BITSTREAM)
		return -E2BIG;
	if (stream->size < required) {
		allocation = ALIGN(required, SZ_64K);
		replacement.cpu = dma_alloc_coherent(vdec->dev, allocation,
						     &replacement.dma, GFP_KERNEL);
		if (!replacement.cpu)
			return -ENOMEM;
		if (upper_32_bits(replacement.dma) ||
		    !IS_ALIGNED(replacement.dma, 16)) {
			dma_free_coherent(vdec->dev, allocation, replacement.cpu,
					  replacement.dma);
			return -ERANGE;
		}
		replacement.size = allocation;
		if (stream->cpu) {
			memcpy(replacement.cpu, stream->cpu,
			       ctx->h264_stream_bytes);
			dma_free_coherent(vdec->dev, stream->size, stream->cpu,
					  stream->dma);
		}
		*stream = replacement;
	}

	src_cpu = vb2_plane_vaddr(&src->vb2_buf, 0);
	if (!src_cpu)
		return -EOPNOTSUPP;
	src_cpu += src->vb2_buf.planes[0].data_offset;
	memset(stream->cpu + ctx->h264_stream_bytes, 0,
	       offset - ctx->h264_stream_bytes);
	memcpy(stream->cpu + offset, src_cpu, ebsp_size);
	*payload = histb_vdec_ebsp_to_rbsp(stream->cpu + offset, ebsp_size);
	/*
	 * VDH fetches full bus bursts around the valid stream-bit boundary, and
	 * the backwards scan below needs a zero byte to stop at.  Both needs
	 * are served by a short tail; zeroing the whole remaining allocation
	 * instead costs one uncached pass over `stream->size - offset -
	 * *payload` bytes per frame, and stream->size only ever grows, so a
	 * single large IDR makes every later frame clear megabytes of uncached
	 * memory.  bitstream_tail_trim selects the short tail for A/B
	 * measurement.
	 */
	memset(stream->cpu + offset + *payload, 0,
	       stream->size - offset - *payload);
	tail_bytes = *payload;
	while (tail_bytes && !((u8 *)stream->cpu)[offset + tail_bytes - 1])
		tail_bytes--;
	if (!tail_bytes)
		return -EINVAL;
	*tail_bits = (*payload - tail_bytes) * 8 +
		__ffs(((u8 *)stream->cpu)[offset + tail_bytes - 1]) + 1;
	*slice_dma = stream->dma + offset;
	ctx->h264_stream_bytes = offset + *payload;
	return 0;
}

struct histb_vdec_bitreader {
	const u8 *data;
	size_t size;
	u32 bitpos;
};

struct histb_vdec_vp9_header {
	u8 refresh_frame_flags;
	u8 ref_frame_idx[3];
	u8 sign_bias;
	u8 frame_context_idx;
	u8 reset_frame_context;
	u8 interpolation_filter;
	u16 width;
	u16 height;
	u16 render_width;
	u16 render_height;
	bool key_frame;
	bool show_frame;
	bool error_resilient;
	bool intra_only;
	bool allow_high_precision_mv;
	bool refresh_frame_context;
	bool frame_parallel_decoding_mode;
};

static bool
histb_vdec_decoded_buffer_valid(struct histb_vdec_decoded_buffer *decoded);

static void
histb_vdec_propagate_error_taint(struct histb_vdec_decoded_buffer *decoded,
					 struct histb_vdec_decoded_buffer *reference)
{
	if (reference && reference != decoded && reference->error_tainted)
		decoded->error_tainted = true;
}

static struct histb_vdec_decoded_buffer *
histb_vdec_find_reference(struct histb_vdec_ctx *ctx, u64 timestamp);
static int histb_vdec_alloc_decoded_buffers(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded);
static void histb_vdec_reclaim_decoded_buffers(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *output);
static dma_addr_t histb_vdec_buffer_dma(struct histb_vdec_ctx *ctx,
					enum histb_vdec_buffer_id id);
static __le32 *histb_vdec_msg_slot(struct histb_vdec_ctx *ctx,
					   unsigned int slot);
static int histb_vdec_run_vc1_bpd(struct histb_vdec_ctx *ctx,
				  const struct histb_vc1_bpd_regs *regs,
				  u32 available_bits,
				  struct histb_vc1_bpd_result *result);
static int histb_vdec_resize_vc1_stream(struct histb_vdec_ctx *ctx,
					 size_t required);
static int histb_vdec_load_avsp(struct histb_vdec_dev *vdec);
static int
histb_vdec_validate_avs(struct histb_vdec_ctx *ctx,
			const struct v4l2_ctrl_avs_sequence *sequence,
			const struct v4l2_ctrl_avs_picture *picture,
			const struct v4l2_ctrl_avs_slice_params *slices,
			unsigned int num_slices,
			const struct v4l2_ctrl_avs_decode_params *decode,
			const u8 *payload_data, unsigned long payload);

struct histb_vdec_mpeg2_vlc {
	u16 code;
	u8 bits;
	u8 increment;
};

static const struct histb_vdec_mpeg2_vlc histb_vdec_mpeg2_mbaddr_vlc[] = {
	{ 0x1, 1, 1 }, { 0x3, 3, 2 }, { 0x2, 3, 3 },
	{ 0x3, 4, 4 }, { 0x2, 4, 5 }, { 0x3, 5, 6 },
	{ 0x2, 5, 7 }, { 0x7, 7, 8 }, { 0x6, 7, 9 },
	{ 0xb, 8, 10 }, { 0xa, 8, 11 }, { 0x9, 8, 12 },
	{ 0x8, 8, 13 }, { 0x7, 8, 14 }, { 0x6, 8, 15 },
	{ 0x17, 10, 16 }, { 0x16, 10, 17 }, { 0x15, 10, 18 },
	{ 0x14, 10, 19 }, { 0x13, 10, 20 }, { 0x12, 10, 21 },
	{ 0x23, 11, 22 }, { 0x22, 11, 23 }, { 0x21, 11, 24 },
	{ 0x20, 11, 25 }, { 0x1f, 11, 26 }, { 0x1e, 11, 27 },
	{ 0x1d, 11, 28 }, { 0x1c, 11, 29 }, { 0x1b, 11, 30 },
	{ 0x1a, 11, 31 }, { 0x19, 11, 32 }, { 0x18, 11, 33 },
};

static int histb_vdec_read_bits(struct histb_vdec_bitreader *br, u8 bits,
				u32 *value)
{
	u32 result = 0;
	u8 i;

	if (!bits || bits > 24 || br->bitpos + bits > br->size * 8)
		return -EINVAL;
	for (i = 0; i < bits; i++) {
		result <<= 1;
		result |= (br->data[br->bitpos / 8] >>
			   (7 - br->bitpos % 8)) & 1;
		br->bitpos++;
	}
	*value = result;
	return 0;
}

static int histb_vdec_vp9_read_bit(struct histb_vdec_bitreader *br, bool *value)
{
	u32 bit;
	int ret;

	ret = histb_vdec_read_bits(br, 1, &bit);
	if (!ret)
		*value = bit;
	return ret;
}

static int histb_vdec_vp9_read_size(struct histb_vdec_bitreader *br,
				    u16 *width, u16 *height)
{
	u32 value;

	if (histb_vdec_read_bits(br, 16, &value))
		return -EINVAL;
	*width = value + 1;
	if (histb_vdec_read_bits(br, 16, &value))
		return -EINVAL;
	*height = value + 1;
	return 0;
}

static int histb_vdec_vp9_read_render_size(struct histb_vdec_bitreader *br,
					   u16 width, u16 height,
					   u16 *render_width,
					   u16 *render_height)
{
	bool different;

	if (histb_vdec_vp9_read_bit(br, &different))
		return -EINVAL;
	if (different)
		return histb_vdec_vp9_read_size(br, render_width, render_height);
	*render_width = width;
	*render_height = height;
	return 0;
}

static int histb_vdec_vp9_read_sync_code(struct histb_vdec_bitreader *br)
{
	u32 sync;

	return histb_vdec_read_bits(br, 24, &sync) || sync != 0x498342 ?
		-EINVAL : 0;
}

static int histb_vdec_vp9_read_color_config(struct histb_vdec_bitreader *br,
					    u8 profile)
{
	u32 color_space;
	bool full_range;

	/* The first candidate accepts only profile-0 8-bit 4:2:0. */
	if (profile)
		return -EINVAL;
	if (histb_vdec_read_bits(br, 3, &color_space))
		return -EINVAL;
	/* SRGB is specified only for the 4:4:4 profiles. */
	if (color_space == 7)
		return -EINVAL;
	return histb_vdec_vp9_read_bit(br, &full_range);
}

static int histb_vdec_parse_vp9_uncompressed_header(
		const u8 *data, size_t payload,
		const struct v4l2_ctrl_vp9_frame *frame,
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_vp9_header *header)
{
	struct histb_vdec_bitreader br = {
		.data = data,
		.size = min_t(size_t, payload, frame->uncompressed_header_size),
	};
	u32 marker, profile_low, profile_high, value;
	bool show_existing, found_size = false;
	unsigned int i;

	memset(header, 0, sizeof(*header));
	if (!br.size || frame->uncompressed_header_size > payload ||
	    frame->compressed_header_size > payload - frame->uncompressed_header_size)
		return -EINVAL;
	if (histb_vdec_read_bits(&br, 2, &marker) || marker != 2 ||
	    histb_vdec_read_bits(&br, 1, &profile_low) ||
	    histb_vdec_read_bits(&br, 1, &profile_high))
		return -EINVAL;
	value = profile_low | profile_high << 1;
	if (value == 3) {
		bool reserved;

		if (histb_vdec_vp9_read_bit(&br, &reserved) || reserved)
			return -EINVAL;
	}
	if (value != frame->profile || value != 0 || frame->bit_depth != 8)
		return -EINVAL;
	if (histb_vdec_vp9_read_bit(&br, &show_existing) || show_existing)
		return -EINVAL;
	if (histb_vdec_vp9_read_bit(&br, &header->key_frame) ||
	    histb_vdec_vp9_read_bit(&br, &header->show_frame) ||
	    histb_vdec_vp9_read_bit(&br, &header->error_resilient))
		return -EINVAL;
	header->key_frame = !header->key_frame;

	if (header->key_frame) {
		header->refresh_frame_flags = 0xff;
		if (histb_vdec_vp9_read_sync_code(&br) ||
		    histb_vdec_vp9_read_color_config(&br, frame->profile) ||
		    histb_vdec_vp9_read_size(&br, &header->width, &header->height) ||
		    histb_vdec_vp9_read_render_size(&br, header->width,
						 header->height,
						 &header->render_width,
						 &header->render_height))
			return -EINVAL;
	} else {
		if (!header->show_frame &&
		    histb_vdec_vp9_read_bit(&br, &header->intra_only))
			return -EINVAL;
		if (!header->error_resilient) {
			if (histb_vdec_read_bits(&br, 2, &value))
				return -EINVAL;
			header->reset_frame_context = value;
		}
		if (header->intra_only) {
			if (histb_vdec_vp9_read_sync_code(&br) ||
			    histb_vdec_read_bits(&br, 8, &value))
				return -EINVAL;
			header->refresh_frame_flags = value;
			if (histb_vdec_vp9_read_size(&br, &header->width,
						      &header->height) ||
			    histb_vdec_vp9_read_render_size(&br, header->width,
							 header->height,
							 &header->render_width,
							 &header->render_height))
				return -EINVAL;
		} else {
			if (histb_vdec_read_bits(&br, 8, &value))
				return -EINVAL;
			header->refresh_frame_flags = value;
			for (i = 0; i < ARRAY_SIZE(header->ref_frame_idx); i++) {
				bool bias;

				if (histb_vdec_read_bits(&br, 3, &value) ||
				    histb_vdec_vp9_read_bit(&br, &bias))
					return -EINVAL;
				header->ref_frame_idx[i] = value;
				header->sign_bias |= bias << i;
			}
			for (i = 0; i < ARRAY_SIZE(header->ref_frame_idx); i++) {
				bool use_ref;
				struct histb_vdec_decoded_buffer *ref;

				if (histb_vdec_vp9_read_bit(&br, &use_ref))
					return -EINVAL;
				if (!use_ref)
					continue;
				ref = ctx->vp9_dpb[header->ref_frame_idx[i]];
				if (!ref || !ref->vp9_dpb_valid)
					return -EINVAL;
				header->width = ref->vp9_width;
				header->height = ref->vp9_height;
				found_size = true;
				break;
			}
			if (!found_size && histb_vdec_vp9_read_size(&br,
							      &header->width,
							      &header->height))
				return -EINVAL;
			if (histb_vdec_vp9_read_render_size(&br, header->width,
							 header->height,
							 &header->render_width,
							 &header->render_height) ||
			    histb_vdec_vp9_read_bit(&br,
						     &header->allow_high_precision_mv) ||
			    histb_vdec_vp9_read_bit(&br, &found_size))
				return -EINVAL;
			if (found_size) {
				header->interpolation_filter =
					V4L2_VP9_INTERP_FILTER_SWITCHABLE;
			} else {
				if (histb_vdec_read_bits(&br, 2, &value))
					return -EINVAL;
				/* The bitstream places smooth before the regular filter. */
				header->interpolation_filter = value ^ (value <= 1);
			}
		}
	}

	if (!header->error_resilient) {
		if (histb_vdec_vp9_read_bit(&br, &header->refresh_frame_context) ||
		    histb_vdec_vp9_read_bit(&br,
					     &header->frame_parallel_decoding_mode))
			return -EINVAL;
	} else {
		header->frame_parallel_decoding_mode = true;
	}
	if (histb_vdec_read_bits(&br, 2, &value))
		return -EINVAL;
	header->frame_context_idx = value;

	if (br.bitpos > frame->uncompressed_header_size * 8 ||
	    header->width != frame->frame_width_minus_1 + 1 ||
	    header->height != frame->frame_height_minus_1 + 1 ||
	    header->render_width != frame->render_width_minus_1 + 1 ||
	    header->render_height != frame->render_height_minus_1 + 1 ||
	    header->key_frame != !!(frame->flags & V4L2_VP9_FRAME_FLAG_KEY_FRAME) ||
	    header->show_frame != !!(frame->flags & V4L2_VP9_FRAME_FLAG_SHOW_FRAME) ||
	    header->error_resilient !=
		!!(frame->flags & V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT) ||
	    header->intra_only != !!(frame->flags & V4L2_VP9_FRAME_FLAG_INTRA_ONLY) ||
	    header->allow_high_precision_mv !=
		!!(frame->flags & V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV) ||
	    header->refresh_frame_context !=
		!!(frame->flags & V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX) ||
	    header->frame_parallel_decoding_mode !=
		!!(frame->flags & V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE) ||
	    header->frame_context_idx != frame->frame_context_idx ||
	    header->reset_frame_context != frame->reset_frame_context ||
	    (!header->key_frame && !header->intra_only &&
	     (header->interpolation_filter != frame->interpolation_filter ||
	      header->sign_bias != frame->ref_frame_sign_bias)))
		return -EINVAL;

	return 0;
}

static int histb_vdec_stage_vp9_state(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		const struct v4l2_ctrl_vp9_frame *frame,
		const struct v4l2_ctrl_vp9_compressed_hdr *compressed,
		const struct histb_vdec_vp9_header *header)
{
	const u64 timestamps[] = { frame->last_frame_ts, frame->golden_frame_ts,
				   frame->alt_frame_ts };
	u32 known_flags = V4L2_VP9_FRAME_FLAG_KEY_FRAME |
		V4L2_VP9_FRAME_FLAG_SHOW_FRAME |
		V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT |
		V4L2_VP9_FRAME_FLAG_INTRA_ONLY |
		V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV |
		V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX |
		V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE |
		V4L2_VP9_FRAME_FLAG_X_SUBSAMPLING |
		V4L2_VP9_FRAME_FLAG_Y_SUBSAMPLING |
		V4L2_VP9_FRAME_FLAG_COLOR_RANGE_FULL_SWING;
	u8 frame_ctx_idx;
	bool used_logic_id[HISTB_VDEC_VP9_LOGIC_IDS] = { };
	unsigned int i;

	if ((frame->flags & ~known_flags) || frame->profile ||
	    frame->bit_depth != 8 ||
	    !(frame->flags & V4L2_VP9_FRAME_FLAG_X_SUBSAMPLING) ||
	    !(frame->flags & V4L2_VP9_FRAME_FLAG_Y_SUBSAMPLING) ||
	    frame->frame_width_minus_1 + 1 != ctx->src.pix.width ||
	    frame->frame_height_minus_1 + 1 != ctx->src.pix.height ||
	    frame->frame_width_minus_1 + 1 > HISTB_VDEC_VP9_MAX_WIDTH ||
	    frame->frame_height_minus_1 + 1 > HISTB_VDEC_VP9_MAX_HEIGHT ||
	    frame->tile_cols_log2 > 6 || frame->tile_rows_log2 > 2 ||
	    frame->reference_mode > V4L2_VP9_REFERENCE_MODE_SELECT ||
	    compressed->tx_mode > V4L2_VP9_TX_MODE_SELECT ||
	    memchr_inv(frame->reserved, 0, sizeof(frame->reserved)))
		return -EINVAL;

	if (!header->key_frame && !header->intra_only) {
		for (i = 0; i < ARRAY_SIZE(header->ref_frame_idx); i++) {
			struct histb_vdec_decoded_buffer *ref;

			ref = histb_vdec_find_reference(ctx, timestamps[i]);
			if (!ref || !histb_vdec_decoded_buffer_valid(ref) ||
			    !ref->vp9_dpb_valid ||
			    ref->vp9_logic_id < 0 ||
			    ref->vp9_logic_id >= HISTB_VDEC_VP9_LOGIC_IDS ||
			    ctx->vp9_dpb[header->ref_frame_idx[i]] != ref ||
			    2ULL * header->width < ref->vp9_width ||
			    2ULL * header->height < ref->vp9_height ||
			    header->width > 16ULL * ref->vp9_width ||
			    header->height > 16ULL * ref->vp9_height)
				return -EINVAL;
		}
	}
	if (!header->key_frame)
		for (i = 0; i < ARRAY_SIZE(ctx->vp9_dpb); i++)
			if (ctx->vp9_dpb[i] == decoded)
				return -EBUSY;

	memcpy(ctx->vp9_pending_dpb, ctx->vp9_dpb,
	       sizeof(ctx->vp9_pending_dpb));
	for (i = 0; i < ARRAY_SIZE(ctx->vp9_pending_dpb); i++)
		if (header->refresh_frame_flags & BIT(i))
			ctx->vp9_pending_dpb[i] = decoded;
	for (i = 0; i < ARRAY_SIZE(ctx->vp9_dpb); i++) {
		struct histb_vdec_decoded_buffer *ref = ctx->vp9_dpb[i];

		if (!ref)
			continue;
		if (ref->vp9_logic_id < 0 ||
		    ref->vp9_logic_id >= HISTB_VDEC_VP9_LOGIC_IDS)
			return -EINVAL;
		used_logic_id[ref->vp9_logic_id] = true;
	}
	for (i = 0; i < ARRAY_SIZE(used_logic_id); i++)
		if (!used_logic_id[i])
			break;
	if (i == ARRAY_SIZE(used_logic_id))
		return -ENOSPC;

	memcpy(ctx->vp9_pending_frame_ctx, ctx->vp9_frame_ctx,
	       sizeof(ctx->vp9_pending_frame_ctx));
	frame_ctx_idx = v4l2_vp9_reset_frame_ctx(frame,
						 ctx->vp9_pending_frame_ctx);
	ctx->vp9_pending_probs = ctx->vp9_pending_frame_ctx[frame_ctx_idx];
	v4l2_vp9_fw_update_probs(&ctx->vp9_pending_probs, compressed, frame);
	ctx->vp9_pending_frame = *frame;
	if (!(frame->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_UPDATE_DATA)) {
		const struct v4l2_vp9_segmentation *previous = &ctx->vp9_seg;

		if (header->key_frame || header->intra_only ||
		    header->error_resilient) {
			memset(ctx->vp9_pending_frame.seg.feature_data, 0,
			       sizeof(ctx->vp9_pending_frame.seg.feature_data));
			memset(ctx->vp9_pending_frame.seg.feature_enabled, 0,
			       sizeof(ctx->vp9_pending_frame.seg.feature_enabled));
			ctx->vp9_pending_frame.seg.flags &=
				~V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE;
		} else {
			memcpy(ctx->vp9_pending_frame.seg.feature_data,
			       previous->feature_data,
			       sizeof(ctx->vp9_pending_frame.seg.feature_data));
			memcpy(ctx->vp9_pending_frame.seg.feature_enabled,
			       previous->feature_enabled,
			       sizeof(ctx->vp9_pending_frame.seg.feature_enabled));
			ctx->vp9_pending_frame.seg.flags =
				(ctx->vp9_pending_frame.seg.flags &
				 ~V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE) |
				(previous->flags &
				 V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE);
		}
	}
	ctx->vp9_pending_compressed_hdr = *compressed;
	ctx->vp9_pending_frame.frame_context_idx = frame_ctx_idx;
	ctx->vp9_pending_output = decoded;
	ctx->vp9_pending_refresh = header->refresh_frame_flags;
	ctx->vp9_pending_logic_id = i;
	memcpy(ctx->vp9_pending_ref_idx, header->ref_frame_idx,
	       sizeof(ctx->vp9_pending_ref_idx));
	ctx->vp9_pending_sign_bias = header->sign_bias;
	ctx->vp9_pending_width = header->width;
	ctx->vp9_pending_height = header->height;
	ctx->vp9_pending_valid = true;

	memcpy(ctx->buffers[HISTB_VDEC_BUF_VP9_SEG_WORK].cpu,
	       ctx->buffers[HISTB_VDEC_BUF_VP9_SEG].cpu,
	       HISTB_VDEC_VP9_SEG_SIZE);
	if (header->key_frame || header->intra_only || header->error_resilient)
		memset(ctx->buffers[HISTB_VDEC_BUF_VP9_SEG_WORK].cpu, 0,
		       HISTB_VDEC_VP9_SEG_SIZE);
	memset(ctx->buffers[HISTB_VDEC_BUF_VP9_COUNT].cpu, 0,
	       ctx->buffers[HISTB_VDEC_BUF_VP9_COUNT].size);

	return 0;
}

static void histb_vdec_vp9_build_logical_probs(
		const struct v4l2_vp9_frame_context *probs, u8 logical[2087])
{
	u8 *p = logical;

	memcpy(p, probs->coef, sizeof(probs->coef));
	p += sizeof(probs->coef);
	memcpy(p, probs->y_mode, sizeof(probs->y_mode));
	p += sizeof(probs->y_mode);
	memcpy(p, probs->uv_mode, sizeof(probs->uv_mode));
	p += sizeof(probs->uv_mode);
	memcpy(p, v4l2_vp9_kf_partition_probs,
	       sizeof(v4l2_vp9_kf_partition_probs));
	p += sizeof(v4l2_vp9_kf_partition_probs);
	memcpy(p, probs->partition, sizeof(probs->partition));
	p += sizeof(probs->partition);
	memcpy(p, probs->interp_filter, sizeof(probs->interp_filter));
	p += sizeof(probs->interp_filter);
	memcpy(p, probs->inter_mode, sizeof(probs->inter_mode));
	p += sizeof(probs->inter_mode);
	memcpy(p, probs->is_inter, sizeof(probs->is_inter));
	p += sizeof(probs->is_inter);
	memcpy(p, probs->comp_mode, sizeof(probs->comp_mode));
	p += sizeof(probs->comp_mode);
	memcpy(p, probs->single_ref, sizeof(probs->single_ref));
	p += sizeof(probs->single_ref);
	memcpy(p, probs->comp_ref, sizeof(probs->comp_ref));
	p += sizeof(probs->comp_ref);
	memcpy(p, probs->tx32, sizeof(probs->tx32));
	p += sizeof(probs->tx32);
	memcpy(p, probs->tx16, sizeof(probs->tx16));
	p += sizeof(probs->tx16);
	memcpy(p, probs->tx8, sizeof(probs->tx8));
	p += sizeof(probs->tx8);
	memcpy(p, probs->skip, sizeof(probs->skip));
	p += sizeof(probs->skip);
	memcpy(p, probs->mv.joint, sizeof(probs->mv.joint));
	p += sizeof(probs->mv.joint);
	memcpy(p, probs->mv.sign, sizeof(probs->mv.sign));
	p += sizeof(probs->mv.sign);
	memcpy(p, probs->mv.classes, sizeof(probs->mv.classes));
	p += sizeof(probs->mv.classes);
	memcpy(p, probs->mv.class0_bit, sizeof(probs->mv.class0_bit));
	p += sizeof(probs->mv.class0_bit);
	memcpy(p, probs->mv.bits, sizeof(probs->mv.bits));
	p += sizeof(probs->mv.bits);
	memcpy(p, probs->mv.class0_fr, sizeof(probs->mv.class0_fr));
	p += sizeof(probs->mv.class0_fr);
	memcpy(p, probs->mv.fr, sizeof(probs->mv.fr));
	p += sizeof(probs->mv.fr);
	memcpy(p, probs->mv.class0_hp, sizeof(probs->mv.class0_hp));
	p += sizeof(probs->mv.class0_hp);
	memcpy(p, probs->mv.hp, sizeof(probs->mv.hp));
}

static void histb_vdec_vp9_convert_coef_probs(u8 *dst, const u8 *src)
{
	static const u16 dst_offsets[] = {
		0, 32, 64, 96, 128, 160, 192,
		256, 288, 320, 352, 384, 416, 448,
	};
	static const u16 src_offsets[] = {
		0, 18, 36, 54, 72, 90, 108,
		108, 126, 144, 162, 180, 198, 216,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dst_offsets); i++)
		memcpy(dst + dst_offsets[i], src + src_offsets[i], 18);
}

static void histb_vdec_pack_vp9_probabilities(struct histb_vdec_ctx *ctx)
{
	const struct v4l2_vp9_frame_context *p = &ctx->vp9_pending_probs;
	const struct v4l2_vp9_segmentation *seg =
		&ctx->vp9_pending_frame.seg;
	u8 *table = ctx->buffers[HISTB_VDEC_BUF_VP9_PROB].cpu;
	u8 *tail = table + 4096;
	u8 *logical = table + 4608;
	const u8 *partition;
	unsigned int i;

	memset(table, 0, ctx->buffers[HISTB_VDEC_BUF_VP9_PROB].size);
	histb_vdec_vp9_build_logical_probs(p, logical);
	for (i = 0; i < 8; i++)
		histb_vdec_vp9_convert_coef_probs(table + i * 512,
						     logical + i * 216);

	for (i = 0; i < ARRAY_SIZE(p->y_mode); i++)
		memcpy(tail + i * 16, p->y_mode[i], sizeof(p->y_mode[i]));
	for (i = 0; i < ARRAY_SIZE(p->uv_mode); i++)
		memcpy(tail + 64 + i * 16, p->uv_mode[i],
		       sizeof(p->uv_mode[i]));
	memcpy(tail + 256, seg->tree_probs, sizeof(seg->tree_probs));
	memcpy(tail + 264, seg->pred_probs, sizeof(seg->pred_probs));
	partition = ctx->vp9_pending_frame.flags &
		(V4L2_VP9_FRAME_FLAG_KEY_FRAME | V4L2_VP9_FRAME_FLAG_INTRA_ONLY) ?
		(const u8 *)v4l2_vp9_kf_partition_probs :
		(const u8 *)p->partition;
	memcpy(tail + 268, partition, sizeof(p->partition));
	memcpy(tail + 316, p->skip, sizeof(p->skip));
	memcpy(tail + 320, p->tx8, sizeof(p->tx8));
	memcpy(tail + 322, p->tx16, sizeof(p->tx16));
	memcpy(tail + 326, p->tx32, sizeof(p->tx32));
	memcpy(tail + 332, p->is_inter, sizeof(p->is_inter));
	memcpy(tail + 336, p->comp_mode, 4);
	tail[340] = p->comp_mode[4];
	memcpy(tail + 341, p->comp_ref, 3);
	memcpy(tail + 344, &p->comp_ref[3], 2);
	memcpy(tail + 348, p->single_ref, sizeof(p->single_ref));
	tail[358] = p->inter_mode[0][0];
	memcpy(tail + 360, (const u8 *)p->inter_mode + 1,
	       sizeof(p->inter_mode) - 1);
	memcpy(tail + 380, p->interp_filter, sizeof(p->interp_filter));
	tail[388] = p->mv.sign[0];
	tail[389] = p->mv.sign[1];
	memcpy(tail + 392, p->mv.joint, sizeof(p->mv.joint));
	memcpy(tail + 396, p->mv.classes, sizeof(p->mv.classes));
	tail[416] = p->mv.class0_bit[0];
	tail[417] = p->mv.class0_bit[1];
	memcpy(tail + 420, p->mv.bits, sizeof(p->mv.bits));
	memcpy(tail + 440, p->mv.class0_fr, sizeof(p->mv.class0_fr));
	memcpy(tail + 452, p->mv.fr[0], sizeof(p->mv.fr[0]));
	memcpy(tail + 456, p->mv.fr[1], sizeof(p->mv.fr[1]));
	tail[460] = p->mv.class0_hp[0];
	tail[461] = p->mv.class0_hp[1];
	tail[462] = p->mv.hp[0];
	tail[463] = p->mv.hp[1];
}

static u32 histb_vdec_vp9_signed_magnitude(s32 value, u8 bits)
{
	u32 magnitude = min_t(u32, abs(value), BIT(bits - 1) - 1);

	return magnitude | (value < 0 ? BIT(bits - 1) : 0);
}

static void histb_vdec_vp9_loop_filter_levels(
		const struct v4l2_ctrl_vp9_frame *frame, u8 levels[8][4][2])
{
	const struct v4l2_vp9_loop_filter *lf = &frame->lf;
	const struct v4l2_vp9_segmentation *seg = &frame->seg;
	unsigned int segment, ref, mode;

	memset(levels, 0, 8 * 4 * 2);
	for (segment = 0; segment < 8; segment++) {
		s32 base = lf->level;

		if ((seg->flags & V4L2_VP9_SEGMENTATION_FLAG_ENABLED) &&
		    v4l2_vp9_seg_feat_enabled(seg->feature_enabled,
					       V4L2_VP9_SEG_LVL_ALT_L, segment)) {
			s32 alt = seg->feature_data[segment][V4L2_VP9_SEG_LVL_ALT_L];

			base = seg->flags &
				V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE ?
				alt : base + alt;
		}
		base = clamp_t(s32, base, 0, 63);
		if (!(lf->flags & V4L2_VP9_LOOP_FILTER_FLAG_DELTA_ENABLED))
			continue;

		levels[segment][0][0] = clamp_t(s32,
			base + ((s32)lf->ref_deltas[0] << (base >> 5)), 0, 63);
		for (ref = 1; ref < 4; ref++)
			for (mode = 0; mode < 2; mode++) {
				s32 level = base +
					((s32)lf->ref_deltas[ref] << (base >> 5));

				level += (s32)lf->mode_deltas[mode] << (base >> 5);
				levels[segment][ref][mode] =
					clamp_t(s32, level, 0, 63);
			}
	}
}

static u32 histb_vdec_vp9_scale_fp(u32 ref, u32 current_size)
{
	return ((u64)ref << 14) / current_size;
}

static int histb_vdec_prepare_vp9_tiles(struct histb_vdec_ctx *ctx,
					const struct v4l2_ctrl_vp9_frame *frame,
					dma_addr_t src_dma, size_t payload)
{
	__le32 *messages = ctx->buffers[HISTB_VDEC_BUF_TILE_SEG].cpu;
	dma_addr_t messages_dma = histb_vdec_buffer_dma(ctx,
							 HISTB_VDEC_BUF_TILE_SEG);
	const u8 *stream = ctx->bitstream.cpu;
	u32 mi_cols = DIV_ROUND_UP(frame->frame_width_minus_1 + 1, 8);
	u32 mi_rows = DIV_ROUND_UP(frame->frame_height_minus_1 + 1, 8);
	u32 sb_cols = DIV_ROUND_UP(mi_cols, 8);
	u32 sb_rows = DIV_ROUND_UP(mi_rows, 8);
	u32 cols = BIT(frame->tile_cols_log2);
	u32 rows = BIT(frame->tile_rows_log2);
	u32 total = cols * rows;
	u32 tile_sb_offset = 0;
	size_t offset = frame->uncompressed_header_size +
			frame->compressed_header_size;
	unsigned int row, col, index = 0;

	if (!total || total > HISTB_VDEC_VP9_MAX_TILES || offset >= payload)
		return -EINVAL;
	memset(messages, 0, HISTB_VDEC_VP9_MAX_TILES *
	       HISTB_VDEC_VP9_TILE_WORDS * sizeof(*messages));
	for (row = 0; row < rows; row++) {
		u32 row_start = min_t(u32, ((row * sb_rows) >>
						frame->tile_rows_log2) * 8, mi_rows);
		u32 row_end = min_t(u32, (((row + 1) * sb_rows) >>
						frame->tile_rows_log2) * 8, mi_rows);

		for (col = 0; col < cols; col++, index++) {
			__le32 *msg = messages + index * HISTB_VDEC_VP9_TILE_WORDS;
			u32 col_start = min_t(u32, ((col * sb_cols) >>
						frame->tile_cols_log2) * 8, mi_cols);
			u32 col_end = min_t(u32, (((col + 1) * sb_cols) >>
						frame->tile_cols_log2) * 8, mi_cols);
			u32 sb_row_start = DIV_ROUND_UP(row_start, 8);
			u32 sb_row_end = DIV_ROUND_UP(row_end, 8);
			u32 sb_col_start = DIV_ROUND_UP(col_start, 8);
			u32 sb_col_end = DIV_ROUND_UP(col_end, 8);
			u32 tile_sb_rows = sb_row_end - sb_row_start;
			u32 tile_sb_cols = sb_col_end - sb_col_start;
			size_t tile_size;
			dma_addr_t data_dma, aligned;

			if (!row_end || !col_end || row_start >= row_end ||
			    col_start >= col_end)
				return -EINVAL;
			if (index + 1 < total) {
				if (payload - offset < 4)
					return -EINVAL;
				tile_size = get_unaligned_be32(stream + offset);
				offset += 4;
				if (!tile_size || tile_size > payload - offset)
					return -EINVAL;
			} else {
				tile_size = payload - offset;
				if (!tile_size)
					return -EINVAL;
			}
			data_dma = src_dma + offset;
			aligned = round_down(data_dma, 16);
			if (tile_size * 8ULL + 128 > U32_MAX)
				return -E2BIG;
			msg[0] = cpu_to_le32(lower_32_bits(aligned));
			msg[1] = cpu_to_le32((data_dma - aligned) * 8);
			msg[2] = cpu_to_le32(tile_size * 8 + 128);
			msg[6] = cpu_to_le32((sb_row_end - 1) |
						 sb_row_start << 16);
			msg[7] = cpu_to_le32((sb_col_end - 1) |
						 sb_col_start << 16);
			msg[8] = cpu_to_le32((sb_row_end - 1) * sb_cols +
						 sb_col_end - 1);
			msg[9] = cpu_to_le32(tile_sb_offset +
						 tile_sb_rows * tile_sb_cols - 1);
			msg[10] = cpu_to_le32(sb_row_start * sb_cols +
						  sb_col_start);
			msg[11] = cpu_to_le32(tile_sb_offset);
			if (index + 1 < total)
				msg[63] = cpu_to_le32(lower_32_bits(messages_dma +
					(index + 1) * HISTB_VDEC_VP9_TILE_WORDS *
					sizeof(*messages)));
			tile_sb_offset += tile_sb_rows * tile_sb_cols;
			offset += tile_size;
		}
	}
	return offset == payload ? 0 : -EINVAL;
}

static int histb_vdec_prepare_vp9_messages(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		const struct v4l2_ctrl_vp9_frame *frame,
		const struct histb_vdec_vp9_header *header,
		dma_addr_t src_dma, size_t payload)
{
	__le32 *pic = histb_vdec_msg_slot(ctx, HISTB_VDEC_PIC_MSG_SLOT);
	struct histb_vdec_decoded_buffer *refs[3];
	u8 levels[8][4][2];
	bool intra;
	u32 value, stride, height, i;

	if (decoded->apc_slot != HISTB_VDEC_APC_INVALID)
		return -EINVAL;
	histb_vdec_reclaim_decoded_buffers(ctx, decoded);
	if (histb_vdec_alloc_decoded_buffers(ctx, decoded))
		return -ENOMEM;
	decoded->error_tainted = false;
	intra = frame->flags & (V4L2_VP9_FRAME_FLAG_KEY_FRAME |
				V4L2_VP9_FRAME_FLAG_INTRA_ONLY);
	for (i = 0; i < ARRAY_SIZE(refs); i++) {
		refs[i] = intra ? NULL : ctx->vp9_dpb[header->ref_frame_idx[i]];
		if (!refs[i])
			refs[i] = decoded;
		if (!intra)
			histb_vdec_propagate_error_taint(decoded, refs[i]);
	}

	memset(ctx->buffers[HISTB_VDEC_BUF_MSG].cpu, 0,
	       ctx->buffers[HISTB_VDEC_BUF_MSG].size);
	histb_vdec_pack_vp9_probabilities(ctx);
	if (histb_vdec_prepare_vp9_tiles(ctx, frame, src_dma, payload))
		return -EINVAL;
	histb_vdec_vp9_loop_filter_levels(frame, levels);

	value = !(frame->flags & V4L2_VP9_FRAME_FLAG_KEY_FRAME);
	value |= (ctx->vp9_have_state &&
		  !(ctx->vp9_last_frame_flags &
		    V4L2_VP9_FRAME_FLAG_KEY_FRAME)) << 1;
	value |= (ctx->vp9_have_state &&
		  (ctx->vp9_last_frame_flags &
		   V4L2_VP9_FRAME_FLAG_SHOW_FRAME)) << 3;
	value |= !!(frame->flags & V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT) << 4;
	value |= !!(frame->flags & V4L2_VP9_FRAME_FLAG_X_SUBSAMPLING) << 5;
	value |= !!(frame->flags & V4L2_VP9_FRAME_FLAG_Y_SUBSAMPLING) << 6;
	value |= !!(frame->flags & V4L2_VP9_FRAME_FLAG_INTRA_ONLY) << 7;
	value |= !!(frame->flags & V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX) << 8;
	value |= !!(frame->flags & V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE) << 9;
	value |= (ctx->vp9_pending_compressed_hdr.tx_mode & 0x7) << 10;
	value |= (frame->reference_mode & 0x3) << 13;
	value |= (frame->bit_depth & 0xf) << 15;
	pic[0] = cpu_to_le32(value);
	value = frame->flags & (V4L2_VP9_FRAME_FLAG_KEY_FRAME |
				V4L2_VP9_FRAME_FLAG_INTRA_ONLY) ?
		0 : frame->interpolation_filter & 0x7;
	value |= !!(frame->flags & V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV) << 3;
	value |= (frame->ref_frame_sign_bias & 0x7) << 5;
	value |= (refs[0] == decoded ? ctx->vp9_pending_logic_id :
		  refs[0]->vp9_logic_id) << 8;
	value |= (refs[1] == decoded ? ctx->vp9_pending_logic_id :
		  refs[1]->vp9_logic_id) << 12;
	value |= (refs[2] == decoded ? ctx->vp9_pending_logic_id :
		  refs[2]->vp9_logic_id) << 16;
	value |= (ctx->vp9_prev_width != header->width ||
		  ctx->vp9_prev_height != header->height) << 20;
	pic[1] = cpu_to_le32(value);
	pic[2] = cpu_to_le32((DIV_ROUND_UP(header->width, 8) - 1) |
		(DIV_ROUND_UP(header->height, 8) - 1) << 16);
	pic[3] = cpu_to_le32(frame->lf.sharpness | frame->lf.level << 8);
	for (i = 0; i < 16; i++)
		pic[4 + i] = cpu_to_le32(levels[i / 2][(i % 2) * 2][0] |
			levels[i / 2][(i % 2) * 2][1] << 8 |
			levels[i / 2][(i % 2) * 2 + 1][0] << 16 |
			levels[i / 2][(i % 2) * 2 + 1][1] << 24);
	pic[20] = cpu_to_le32(frame->quant.base_q_idx |
		histb_vdec_vp9_signed_magnitude(frame->quant.delta_q_y_dc, 5) << 8 |
		histb_vdec_vp9_signed_magnitude(frame->quant.delta_q_uv_dc, 5) << 16 |
		histb_vdec_vp9_signed_magnitude(frame->quant.delta_q_uv_ac, 5) << 24);
	pic[21] = cpu_to_le32(
		!!(frame->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_ENABLED) |
		!!(frame->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_UPDATE_MAP) << 1 |
		!!(frame->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE) << 2 |
		!!(frame->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_TEMPORAL_UPDATE) << 3);
	value = 0;
	for (i = 0; i < 8; i++)
		value |= (frame->seg.feature_enabled[i] & 0xf) << (i * 4);
	pic[22] = cpu_to_le32(value);
	for (i = 0; i < 8; i++)
		pic[23 + i / 3] |= cpu_to_le32(
			(frame->seg.feature_data[i][V4L2_VP9_SEG_LVL_ALT_Q] & 0x1ff) <<
			((i % 3) * 9));
	for (i = 0; i < 8; i++)
		pic[26 + i / 4] |= cpu_to_le32(
			(frame->seg.feature_data[i][V4L2_VP9_SEG_LVL_ALT_L] & 0xff) <<
			((i % 4) * 8));
	value = 0;
	for (i = 0; i < 8; i++) {
		value |= (frame->seg.feature_data[i][V4L2_VP9_SEG_LVL_REF_FRAME] & 3) <<
			(i * 2);
		value |= v4l2_vp9_seg_feat_enabled(frame->seg.feature_enabled,
			V4L2_VP9_SEG_LVL_SKIP, i) << (16 + i);
	}
	pic[28] = cpu_to_le32(value);
	pic[29] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_VP9_SEG_WORK)));
	pic[30] = cpu_to_le32(HISTB_VDEC_VP9_SEG_STRIDE);
	pic[32] = cpu_to_le32(lower_32_bits(decoded->tile.dma));
	pic[33] = cpu_to_le32(lower_32_bits(refs[2]->tile.dma));
	pic[34] = cpu_to_le32(lower_32_bits(refs[1]->tile.dma));
	pic[35] = cpu_to_le32(lower_32_bits(refs[0]->tile.dma));
	pic[36] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_SED_TOP)));
	pic[37] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_PMV_TOP)));
	pic[38] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_RCN_TOP)));
	pic[39] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_VP9_PROB)));
	pic[40] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_DBLK_TOP)));
	pic[41] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_VP9_COUNT)));
	pic[42] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_VP9_PMV)));
	pic[43] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_DBLK_LEFT)));
	for (i = 0; i < ARRAY_SIZE(refs); i++) {
		u32 ref_width = refs[i] == decoded ? header->width : refs[i]->vp9_width;
		u32 ref_height = refs[i] == decoded ? header->height : refs[i]->vp9_height;
		u32 ref_base = 54 + i * 3;
		u32 xscale = histb_vdec_vp9_scale_fp(ref_width, header->width);
		u32 yscale = histb_vdec_vp9_scale_fp(ref_height, header->height);
		u32 xstep = (xscale * 16) >> 14;
		u32 ystep = (yscale * 16) >> 14;
		u32 head_size;

		pic[45 + i] = cpu_to_le32(yscale | xscale << 16);
		pic[48] |= cpu_to_le32((ystep & 0x3f) << (i * 8));
		pic[49] |= cpu_to_le32((xstep & 0x3f) << (i * 8));
		pic[50 + i] = cpu_to_le32(ref_width | ref_height << 16);
		stride = ALIGN(ref_width, 256);
		height = ALIGN(ref_height, 64);
		head_size = DIV_ROUND_UP(ref_width, 512) *
			DIV_ROUND_UP(ref_height, 64) * 512;
		pic[ref_base] = cpu_to_le32(stride * 16);
		pic[ref_base + 1] = cpu_to_le32(stride * height);
		pic[ref_base + 2] = cpu_to_le32(head_size);
	}
	pic[53] = cpu_to_le32(ctx->vp9_prev_width |
				 ctx->vp9_prev_height << 16);
	pic[63] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_TILE_SEG)));

	return 0;
}

static int histb_vdec_mpeg2_mbaddr_increment(
		struct histb_vdec_bitreader *br, u32 *increment)
{
	u32 total = 0;

	for (;;) {
		unsigned int i;
		u32 value;

		for (i = 0; i < ARRAY_SIZE(histb_vdec_mpeg2_mbaddr_vlc); i++) {
			struct histb_vdec_bitreader probe = *br;
			const struct histb_vdec_mpeg2_vlc *vlc =
				&histb_vdec_mpeg2_mbaddr_vlc[i];

			if (histb_vdec_read_bits(&probe, vlc->bits, &value) ||
			    value != vlc->code)
				continue;
			*br = probe;
			if (check_add_overflow(total, (u32)vlc->increment, &total))
				return -EOVERFLOW;
			*increment = total;
			return 0;
		}

		/* macroblock_escape adds 33; macroblock_stuffing adds nothing. */
		if (histb_vdec_read_bits(br, 11, &value))
			return -EINVAL;
		if (value == 0x8) {
			if (check_add_overflow(total, 33U, &total))
				return -EOVERFLOW;
		} else if (value != 0xf) {
			return -EINVAL;
		}
	}
}

static int histb_vdec_copy_raw_bitstream(struct histb_vdec_ctx *ctx,
					 struct vb2_v4l2_buffer *src,
					 unsigned long payload,
					 dma_addr_t *stream_dma)
{
	struct histb_vdec_dma_buffer *stream = &ctx->bitstream;
	struct histb_vdec_dev *vdec = ctx->vdec;
	size_t allocation, required;
	u8 *src_cpu;

	if (check_add_overflow((size_t)payload,
			       (size_t)HISTB_VDEC_RAW_STREAM_GUARD, &required) ||
	    required > SIZE_MAX - (SZ_64K - 1))
		return -EOVERFLOW;
	allocation = ALIGN(required, SZ_64K);
	if (stream->size < allocation) {
		struct histb_vdec_dma_buffer replacement = { };

		replacement.cpu = dma_alloc_coherent(vdec->dev, allocation,
						     &replacement.dma, GFP_KERNEL);
		if (!replacement.cpu)
			return -ENOMEM;
		if (upper_32_bits(replacement.dma) ||
		    !IS_ALIGNED(replacement.dma, 16)) {
			dma_free_coherent(vdec->dev, allocation, replacement.cpu,
					  replacement.dma);
			return -ERANGE;
		}
		replacement.size = allocation;
		if (stream->cpu)
			dma_free_coherent(vdec->dev, stream->size, stream->cpu,
					  stream->dma);
		*stream = replacement;
	}

	src_cpu = vb2_plane_vaddr(&src->vb2_buf, 0);
	if (!src_cpu)
		return -EOPNOTSUPP;
	src_cpu += src->vb2_buf.planes[0].data_offset;
	memcpy(stream->cpu, src_cpu, payload);
	memset(stream->cpu + payload, 0, stream->size - payload);
	*stream_dma = stream->dma;
	return 0;
}

static int histb_vdec_validate_vp8_tag(
		const struct v4l2_ctrl_vp8_frame *frame,
		const u8 *data, size_t payload)
{
	u32 profile, tag, width, height;
	bool key_frame;

	if (payload < 3)
		return -EINVAL;
	tag = get_unaligned_le24(data);
	key_frame = !(tag & BIT(0));
	profile = frame->version |
		  (!!(frame->flags & V4L2_VP8_FRAME_FLAG_EXPERIMENTAL) << 2);
	if (key_frame != V4L2_VP8_FRAME_IS_KEY_FRAME(frame) ||
	    ((tag >> 1) & 0x7) != profile ||
	    !!(tag & BIT(4)) !=
		!!(frame->flags & V4L2_VP8_FRAME_FLAG_SHOW_FRAME) ||
	    (tag >> 5) != frame->first_part_size)
		return -EINVAL;
	if (!key_frame)
		return 0;
	if (payload < 10 || data[3] != 0x9d || data[4] != 0x01 ||
	    data[5] != 0x2a)
		return -EINVAL;
	width = get_unaligned_le16(data + 6);
	height = get_unaligned_le16(data + 8);
	if ((width & GENMASK(13, 0)) != frame->width ||
	    (height & GENMASK(13, 0)) != frame->height ||
	    (width >> 14) != frame->horizontal_scale ||
	    (height >> 14) != frame->vertical_scale)
		return -EINVAL;

	return 0;
}

static int histb_vdec_copy_vp8_bitstream(
		struct histb_vdec_ctx *ctx, struct vb2_v4l2_buffer *src,
		unsigned long payload, const struct v4l2_ctrl_vp8_frame *frame,
		dma_addr_t *stream_dma)
{
	int ret;

	ret = histb_vdec_copy_raw_bitstream(ctx, src, payload, stream_dma);
	if (ret)
		return ret;
	memmove(ctx->bitstream.cpu + 16, ctx->bitstream.cpu, payload);
	memset(ctx->bitstream.cpu, 0, 16);
	*stream_dma += 16;

	return histb_vdec_validate_vp8_tag(frame, ctx->bitstream.cpu + 16,
					   payload);
}

static int histb_vdec_copy_hevc_bitstream(
		struct histb_vdec_ctx *ctx, struct vb2_v4l2_buffer *src,
		unsigned long payload,
		const struct v4l2_ctrl_hevc_slice_params *slices,
		unsigned int num_slices, dma_addr_t *stream_dma,
		struct histb_vdec_hevc_stream *streams)
{
	struct histb_vdec_dma_buffer *stream = &ctx->bitstream;
	struct histb_vdec_dev *vdec = ctx->vdec;
	size_t src_offset = 0, dst_offset = 0;
	u8 *src_cpu;
	unsigned int i;

	if (stream->size < payload) {
		struct histb_vdec_dma_buffer replacement = { };
		size_t allocation = ALIGN(payload, SZ_64K);

		replacement.cpu = dma_alloc_coherent(vdec->dev, allocation,
						     &replacement.dma, GFP_KERNEL);
		if (!replacement.cpu)
			return -ENOMEM;
		if (upper_32_bits(replacement.dma) ||
		    !IS_ALIGNED(replacement.dma, 16)) {
			dma_free_coherent(vdec->dev, allocation, replacement.cpu,
					  replacement.dma);
			return -ERANGE;
		}
		replacement.size = allocation;
		if (stream->cpu)
			dma_free_coherent(vdec->dev, stream->size, stream->cpu,
					  stream->dma);
		*stream = replacement;
	}

	src_cpu = vb2_plane_vaddr(&src->vb2_buf, 0);
	if (!src_cpu)
		return -EOPNOTSUPP;
	src_cpu += src->vb2_buf.planes[0].data_offset;
	for (i = 0; i < num_slices; i++) {
		const struct v4l2_ctrl_hevc_slice_params *slice = &slices[i];
		size_t nal_size = DIV_ROUND_UP(slice->bit_size, 8);
		size_t rbsp_size;
		u64 skipped_bits;

		if (!nal_size || nal_size > payload - src_offset ||
		    slice->data_byte_offset >= nal_size)
			return -EINVAL;
		memcpy(stream->cpu + dst_offset, src_cpu + src_offset, nal_size);
		rbsp_size = histb_vdec_ebsp_to_rbsp(stream->cpu + dst_offset,
						    nal_size);
		skipped_bits = ((u64)(nal_size - rbsp_size) +
			slice->data_byte_offset) * 8;
		if (skipped_bits >= slice->bit_size)
			return -EINVAL;
		streams[i].data_offset = dst_offset + slice->data_byte_offset;
		streams[i].valid_bits = slice->bit_size - skipped_bits;
		src_offset += nal_size;
		dst_offset += rbsp_size;
	}
	if (src_offset != payload)
		return -EINVAL;
	memset(stream->cpu + dst_offset, 0, stream->size - dst_offset);
	*stream_dma = stream->dma;

	return 0;
}

static size_t histb_vdec_tile_size(struct histb_vdec_ctx *ctx)
{
	u32 stride = histb_vdec_surface_stride(ctx);
	u32 height;
	size_t size;

	/*
	 * A field pair may need one extra macroblock row even though S_FMT keeps
	 * the visible frame height (for example, 240 lines use a 256-line surface).
	 */
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG1_SLICE ||
	    ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG2_SLICE)
		height = ALIGN(ctx->dst.pix.height, 32);
	else
		height = ALIGN(ctx->dst.pix.height,
			       histb_vdec_surface_height_align(ctx));
	size = stride * height * 3 / 2;

	/* CV200 stores Main10 as an 8-bit tile plane followed by packed low bits. */
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_HEVC_SLICE &&
	    ctx->hevc_main10)
		size += (stride / 4) * height * 3 / 2;

	return size;
}

static size_t histb_vdec_pmv_size(struct histb_vdec_ctx *ctx)
{
	u32 width_mbs = DIV_ROUND_UP(ctx->dst.pix.width, 16);
	u32 height_mbs;
	size_t size;

	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG2_SLICE)
		height_mbs = 2 * DIV_ROUND_UP(ctx->dst.pix.height, 32);
	else if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_AVS_SLICE)
		height_mbs = histb_vdec_avs_height_mbs(ctx->dst.pix.height, false);
	else
		height_mbs = DIV_ROUND_UP(ctx->dst.pix.height, 16);

	/* VP9 uses the single context-level work slot allocated with the codec. */
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_VP9_FRAME)
		return 0;
	else if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_HEVC_SLICE)
		size = width_mbs * height_mbs * 64 + 16;
	else
		size = width_mbs * height_mbs * 64;

	return ALIGN(size, 128);
}

static int histb_vdec_set_decoded_buffer_sizes(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded)
{
	size_t tile_size = histb_vdec_tile_size(ctx);
	size_t pmv_size = histb_vdec_pmv_size(ctx);

	/*
	 * The surface geometry is fixed for the lifetime of a setup: it is
	 * derived from the capture format and a per-stream flag such as
	 * mpeg2_field_picture.  HEVC renegotiates the capture format on
	 * every job, so releasing the surfaces here made the decoder
	 * pipeline fail to reset on each frame (-110, ETIMEDOUT) and the
	 * stream fell back to ~0.5x realtime.  Keep rejecting a geometry
	 * change instead: it means the capture format moved underneath a
	 * running setup, which the client must resolve with S_FMT.
	 */
	if ((decoded->tile.cpu && decoded->tile.size != tile_size) ||
	    (decoded->pmv.cpu && decoded->pmv.size != pmv_size)) {
		dev_err_ratelimited(ctx->vdec->dev,
				    "surface geometry changed: tile %zu/%zu pmv %zu/%zu\n",
				    decoded->tile.size, tile_size,
				    decoded->pmv.size, pmv_size);
		return -EINVAL;
	}

	decoded->tile.size = tile_size;
	decoded->pmv.size = pmv_size;
	return 0;
}

static void
histb_vdec_free_decoded_buffers(struct histb_vdec_ctx *ctx,
				struct histb_vdec_decoded_buffer *decoded)
{
	struct device *dev = ctx->vdec->dev;

	if (decoded->pmv.cpu)
		dma_free_noncoherent(dev, decoded->pmv.size, decoded->pmv.cpu,
				  decoded->pmv.dma,
					     DMA_BIDIRECTIONAL);
	if (decoded->tile.cpu) {
		if (decoded->tile_noncoherent)
			dma_free_noncoherent(dev, decoded->tile.size,
					     decoded->tile.cpu,
					     decoded->tile.dma,
					     DMA_BIDIRECTIONAL);
		else
			dma_free_coherent(dev, decoded->tile.size,
					  decoded->tile.cpu,
					  decoded->tile.dma);
	}
	decoded->pmv.cpu = NULL;
	decoded->pmv.dma = 0;
	decoded->tile.cpu = NULL;
	decoded->tile.dma = 0;
	decoded->tile_noncoherent = true;
	memset(&decoded->avs, 0, sizeof(decoded->avs));
	decoded->vp9_dpb_valid = false;
	decoded->vp9_logic_id = HISTB_VDEC_APC_INVALID;
}

static int
histb_vdec_alloc_decoded_buffers(struct histb_vdec_ctx *ctx,
				 struct histb_vdec_decoded_buffer *decoded)
{
	struct device *dev = ctx->vdec->dev;
	if (decoded->tile.cpu && (!decoded->pmv.size || decoded->pmv.cpu))
		return 0;
	if (decoded->tile.cpu || decoded->pmv.cpu)
		histb_vdec_free_decoded_buffers(ctx, decoded);

	/*
	 * Cacheable surfaces at every size.  This used to switch to
	 * dma_alloc_coherent() at 3840 and above, i.e. the 4K decode output
	 * (~12 MB per frame) was mapped uncached while every smaller size was
	 * cacheable.  The explicit dma_sync_single_for_device() below covers
	 * the device view, so the coherent allocation buys nothing and costs a
	 * large multiple of the decode time - measured 6 fps for 4K HEVC
	 * Main10 against a 50 fps stream.
	 */
	decoded->tile_noncoherent = true;
	if (decoded->tile_noncoherent)
		decoded->tile.cpu = dma_alloc_noncoherent(dev, decoded->tile.size,
							  &decoded->tile.dma,
							  DMA_BIDIRECTIONAL,
							  GFP_KERNEL);
	else
		decoded->tile.cpu = dma_alloc_coherent(dev, decoded->tile.size,
						       &decoded->tile.dma, GFP_KERNEL);
	if (!decoded->tile.cpu)
		return -ENOMEM;
	if (decoded->pmv.size) {
		/* Same reasoning as the tile surface: the engine only ever reads
		 * it through the bus, and the CPU never touches it mid-decode. */
		decoded->pmv.cpu = dma_alloc_noncoherent(dev, decoded->pmv.size,
							 &decoded->pmv.dma,
							 DMA_BIDIRECTIONAL,
							 GFP_KERNEL);
		if (!decoded->pmv.cpu) {
			histb_vdec_free_decoded_buffers(ctx, decoded);
			return -ENOMEM;
		}
		decoded->pmv_noncoherent = true;
	}
	if (upper_32_bits(decoded->tile.dma) ||
	    (decoded->pmv.size && upper_32_bits(decoded->pmv.dma))) {
		histb_vdec_free_decoded_buffers(ctx, decoded);
		return -ERANGE;
	}

	memset(decoded->tile.cpu, 0, decoded->tile.size);
	if (decoded->tile_noncoherent)
		dma_sync_single_for_device(dev, decoded->tile.dma,
					   decoded->tile.size,
					   DMA_BIDIRECTIONAL);
	if (decoded->pmv.cpu)
		memset(decoded->pmv.cpu, 0, decoded->pmv.size);
	return 0;
}

static void histb_vdec_free_mpeg4_anchor(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_mpeg4_anchor *anchor)
{
	struct histb_vdec_decoded_buffer decoded = {
		.tile = anchor->tile,
		.pmv = anchor->pmv,
		.tile_noncoherent = anchor->tile_noncoherent,
	};

	histb_vdec_free_decoded_buffers(ctx, &decoded);
	memset(anchor, 0, sizeof(*anchor));
}

static void histb_vdec_discard_mpeg4_state(struct histb_vdec_ctx *ctx)
{
	ctx->mpeg4_pending_valid = false;
	ctx->mpeg4_pending_output = NULL;
}

static void histb_vdec_reset_mpeg4_au(struct histb_vdec_ctx *ctx)
{
	ctx->mpeg4_au_src = NULL;
	ctx->mpeg4_au_bytes = 0;
	ctx->mpeg4_au_offset = 0;
	ctx->mpeg4_pending_next_offset = 0;
	ctx->mpeg4_au_active = false;
	ctx->mpeg4_pending_source_done = false;
}

static void histb_vdec_reset_mpeg4_state(struct histb_vdec_ctx *ctx)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ctx->mpeg4_ref); i++)
		histb_vdec_free_mpeg4_anchor(ctx, &ctx->mpeg4_ref[i]);
	WARN_ON(ctx->mpeg4_display_pending);
	ctx->mpeg4_display_state.pending = 0;
	ctx->mpeg4_slices = 0;
	histb_mpeg4_parser_reset(&ctx->mpeg4_parser);
	histb_vdec_discard_mpeg4_state(ctx);
	histb_vdec_reset_mpeg4_au(ctx);
}

static void histb_vdec_commit_mpeg4_state(struct histb_vdec_ctx *ctx)
{
	struct histb_vdec_decoded_buffer *decoded = ctx->mpeg4_pending_output;
	struct histb_vdec_mpeg4_anchor *earlier = &ctx->mpeg4_ref[0];
	struct histb_vdec_mpeg4_anchor *latest = &ctx->mpeg4_ref[1];
	struct histb_vdec_decoded_buffer old_latest = {
		.tile = latest->tile,
		.pmv = latest->pmv,
		.tile_noncoherent = latest->tile_noncoherent,
	};

	if (!ctx->mpeg4_pending_valid || !decoded)
		return;
	if (ctx->mpeg4_pending_frame.vop.coding_type == HISTB_MPEG4_B_VOP) {
		ctx->mpeg4_parser = ctx->mpeg4_pending_parser;
		histb_vdec_discard_mpeg4_state(ctx);
		return;
	}

	histb_vdec_free_mpeg4_anchor(ctx, earlier);
	if (old_latest.pmv.cpu) {
		dma_free_coherent(ctx->vdec->dev, old_latest.pmv.size,
				  old_latest.pmv.cpu, old_latest.pmv.dma);
		memset(&old_latest.pmv, 0, sizeof(old_latest.pmv));
	}
	earlier->tile = old_latest.tile;
	earlier->tile_noncoherent = old_latest.tile_noncoherent;

	memset(latest, 0, sizeof(*latest));
	latest->tile = decoded->tile;
	latest->pmv = decoded->pmv;
	latest->tile_noncoherent = decoded->tile_noncoherent;
	memset(&decoded->tile, 0, sizeof(decoded->tile));
	memset(&decoded->pmv, 0, sizeof(decoded->pmv));
	decoded->tile_noncoherent = false;

	ctx->mpeg4_parser = ctx->mpeg4_pending_parser;
	histb_vdec_discard_mpeg4_state(ctx);
}

static void histb_vdec_sync_mpeg4_anchor_for_device(
		struct histb_vdec_ctx *ctx,
		const struct histb_vdec_mpeg4_anchor *anchor)
{
	if (anchor->tile.cpu && anchor->tile_noncoherent)
			dma_sync_single_for_device(ctx->vdec->dev, anchor->tile.dma,
						   anchor->tile.size,
						   DMA_BIDIRECTIONAL);
}

static void histb_vdec_free_vc1_anchor(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_vc1_anchor *anchor)
{
	struct histb_vdec_decoded_buffer decoded = {
		.tile = anchor->tile,
		.pmv = anchor->pmv,
		.tile_noncoherent = anchor->tile_noncoherent,
	};

	histb_vdec_free_decoded_buffers(ctx, &decoded);
	memset(anchor, 0, sizeof(*anchor));
}

static void histb_vdec_discard_vc1_current_locked(
		struct histb_vdec_ctx *ctx, bool fatal)
{
	ctx->vc1_pending_valid = false;
	ctx->vc1_pending_output = NULL;
	ctx->vc1_pending_intensity_valid = false;
	ctx->vc1_pending_halfpel = 0;
	ctx->vc1_pending_ref_pic_type = 0;
	if (fatal)
		ctx->vc1_need_intra = true;
}

static void histb_vdec_discard_vc1_runtime_error(
		struct histb_vdec_ctx *ctx, bool force_fatal)
{
	bool nonreference;
	bool fatal;

	mutex_lock(&ctx->vc1_lock);
	nonreference = ctx->vc1_pending_valid &&
		ctx->vc1_pending_picture.fcm != HISTB_VC1_FIELD_INTERLACED &&
		(ctx->vc1_pending_picture.ptype == HISTB_VC1_PICTURE_B ||
		 ctx->vc1_pending_picture.ptype == HISTB_VC1_PICTURE_BI_HW);
	fatal = force_fatal || !nonreference;
	histb_vdec_discard_vc1_current_locked(ctx, fatal);
	mutex_unlock(&ctx->vc1_lock);
}

static void histb_vdec_reset_vc1_au(struct histb_vdec_ctx *ctx);

static bool histb_vdec_commit_vc1_state(struct histb_vdec_ctx *ctx)
{
	struct histb_vdec_decoded_buffer *decoded = ctx->vc1_pending_output;
	struct histb_vdec_vc1_anchor *earlier = &ctx->vc1_ref[0];
	struct histb_vdec_vc1_anchor *latest = &ctx->vc1_ref[1];
	bool field_pair, nonreference, recovery;
	u8 old_ref_dist = latest->ref_dist;
	struct histb_vdec_decoded_buffer old_latest = {
		.tile = latest->tile,
		.pmv = latest->pmv,
		.tile_noncoherent = latest->tile_noncoherent,
	};

	if (!ctx->vc1_pending_valid || !decoded)
		return false;
	field_pair = ctx->vc1_pending_picture.fcm == HISTB_VC1_FIELD_INTERLACED;
	if (WARN_ON(field_pair && ctx->vc1_field_transaction.state !=
		    HISTB_VC1_FIELD_TXN_COMPLETE)) {
		histb_vdec_discard_vc1_current_locked(ctx, true);
		histb_vdec_reset_vc1_au(ctx);
		return false;
	}
	recovery = field_pair &&
		(ctx->vc1_field_transaction.first_ptype == HISTB_VC1_PICTURE_I ||
		 ctx->vc1_field_transaction.second_ptype == HISTB_VC1_PICTURE_I);
	nonreference = ctx->vc1_pending_picture.ptype == HISTB_VC1_PICTURE_B ||
		ctx->vc1_pending_picture.ptype == HISTB_VC1_PICTURE_BI_HW;
	if (WARN_ON(!nonreference && !ctx->vc1_pending_intensity_valid)) {
		histb_vdec_discard_vc1_current_locked(ctx, true);
		histb_vdec_reset_vc1_au(ctx);
		return false;
	}
	if (WARN_ON(ctx->vc1_pending_next_offset <= ctx->vc1_au_offset ||
		    ctx->vc1_pending_next_offset > ctx->vc1_au_bytes)) {
		histb_vdec_discard_vc1_current_locked(ctx, true);
		histb_vdec_reset_vc1_au(ctx);
		return false;
	}
	ctx->vc1_au_offset = ctx->vc1_pending_next_offset;
	if (ctx->vc1_annex_l) {
		ctx->vc1_smp_rounding = ctx->vc1_pending_picture.rounding_control;
		ctx->vc1_smp_res_pic = ctx->vc1_pending_picture.res_pic;
	}
	if (nonreference) {
		histb_vdec_discard_vc1_current_locked(ctx, false);
		return true;
	}

	/*
	 * The running transaction owns current tile+PMV while latest owns the
	 * preceding tile+PMV and earlier retains only its tile: three surfaces,
	 * two PMV allocations.  Ownership changes only after hardware success.
	 */
	histb_vdec_free_vc1_anchor(ctx, earlier);
	if (old_latest.pmv.cpu) {
		dma_free_coherent(ctx->vdec->dev, old_latest.pmv.size,
				  old_latest.pmv.cpu, old_latest.pmv.dma);
		memset(&old_latest.pmv, 0, sizeof(old_latest.pmv));
	}
	earlier->tile = old_latest.tile;
	earlier->tile_noncoherent = old_latest.tile_noncoherent;
	earlier->intensity = ctx->vc1_pending_earlier_intensity;
	earlier->halfpel = latest->halfpel;
	earlier->range_reduction = latest->range_reduction;
	earlier->fcm = latest->fcm;
	earlier->ref_dist = latest->ref_dist;
	earlier->res_pic = latest->res_pic;

	memset(latest, 0, sizeof(*latest));
	latest->tile = decoded->tile;
	latest->pmv = decoded->pmv;
	latest->tile_noncoherent = decoded->tile_noncoherent;
	histb_vc1_intensity_init(&latest->intensity);
	latest->halfpel = ctx->vc1_pending_halfpel;
	latest->range_reduction = ctx->vc1_pending_picture.range_reduction;
	latest->fcm = ctx->vc1_pending_picture.fcm;
	latest->res_pic = ctx->vc1_pending_picture.res_pic;
	latest->ref_dist = field_pair ?
		(ctx->vc1_entry.refdist_flag ?
		 ctx->vc1_field_transaction.field[0].ref_dist : old_ref_dist) : 0;
	memset(&decoded->tile, 0, sizeof(decoded->tile));
	memset(&decoded->pmv, 0, sizeof(decoded->pmv));
	decoded->tile_noncoherent = false;
	if (ctx->vc1_pending_picture.ptype == HISTB_VC1_PICTURE_I || recovery)
		ctx->vc1_need_intra = false;
	histb_vc1_field_transaction_reset(&ctx->vc1_field_transaction);
	histb_vdec_discard_vc1_current_locked(ctx, false);
	return true;
}

static bool histb_vdec_commit_vc1_first_field(struct histb_vdec_ctx *ctx)
{
	if (!ctx->vc1_pending_valid || !ctx->vc1_pending_output ||
	    ctx->vc1_pending_picture.fcm != HISTB_VC1_FIELD_INTERLACED ||
	    ctx->vc1_pending_picture.is_second_field ||
	    ctx->vc1_field_transaction.state !=
		HISTB_VC1_FIELD_TXN_FIRST_COMPLETE ||
	    ctx->vc1_pending_source_done ||
	    ctx->vc1_pending_next_offset <= ctx->vc1_au_offset ||
	    ctx->vc1_pending_next_offset >= ctx->vc1_au_bytes)
		return false;

	ctx->vc1_au_offset = ctx->vc1_pending_next_offset;
	ctx->vc1_pending_next_offset = 0;
	ctx->vc1_slices = 0;
	histb_vdec_discard_vc1_current_locked(ctx, false);
	return true;
}

static void histb_vdec_sync_vc1_anchor_for_device(
		struct histb_vdec_ctx *ctx,
		const struct histb_vdec_vc1_anchor *anchor)
{
	if (anchor->tile.cpu && anchor->tile_noncoherent)
		dma_sync_single_for_device(ctx->vdec->dev, anchor->tile.dma,
					   anchor->tile.size,
					   DMA_BIDIRECTIONAL);
}

static void histb_vdec_sync_tile_for_device(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded)
{
	if (decoded && decoded->tile.cpu && decoded->tile_noncoherent)
		dma_sync_single_for_device(ctx->vdec->dev, decoded->tile.dma,
					   decoded->tile.size,
					   DMA_BIDIRECTIONAL);
}

static int histb_vdec_copy_vc1_skipped_surface(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded)
{
	struct histb_vdec_vc1_anchor *latest = &ctx->vc1_ref[1];

	if (!decoded || !decoded->tile.cpu || !decoded->pmv.cpu ||
	    !latest->tile.cpu || !latest->pmv.cpu ||
	    decoded->tile.size != latest->tile.size ||
	    decoded->pmv.size != latest->pmv.size) {
		dev_err_ratelimited(ctx->vdec->dev,
			"VC-1 SKIP copy invalid: tile=%p/%zu ref=%p/%zu pmv=%p/%zu ref=%p/%zu\n",
			decoded ? decoded->tile.cpu : NULL,
			decoded ? decoded->tile.size : 0,
			latest->tile.cpu, latest->tile.size,
			decoded ? decoded->pmv.cpu : NULL,
			decoded ? decoded->pmv.size : 0,
			latest->pmv.cpu, latest->pmv.size);
		return -EINVAL;
	}
	if (latest->tile_noncoherent)
		dma_sync_single_for_cpu(ctx->vdec->dev, latest->tile.dma,
					latest->tile.size, DMA_BIDIRECTIONAL);
	if (decoded->tile_noncoherent)
		dma_sync_single_for_cpu(ctx->vdec->dev, decoded->tile.dma,
					decoded->tile.size, DMA_BIDIRECTIONAL);
	memcpy(decoded->tile.cpu, latest->tile.cpu, decoded->tile.size);
	memcpy(decoded->pmv.cpu, latest->pmv.cpu, decoded->pmv.size);
	histb_vdec_sync_vc1_anchor_for_device(ctx, latest);
	histb_vdec_sync_tile_for_device(ctx, decoded);

	return 0;
}

static struct histb_vdec_decoded_buffer *
histb_vdec_decoded_buffer(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct histb_vdec_decoded_buffer, base.vb);
}

static bool
histb_vdec_decoded_buffer_valid(struct histb_vdec_decoded_buffer *decoded);

static struct histb_vdec_decoded_buffer *
histb_vdec_find_reference(struct histb_vdec_ctx *ctx, u64 timestamp)
{
	struct vb2_queue *queue = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	struct vb2_buffer *buffer;
	unsigned int i;

	/* MPEG-1/2 keeps references after userspace may dequeue their capture. */
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG1_SLICE ||
	    ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG2_SLICE) {
		for (i = 0; i < ARRAY_SIZE(ctx->apc); i++) {
			struct histb_vdec_decoded_buffer *ref = ctx->apc[i];

			if (ref && ref->base.vb.vb2_buf.timestamp == timestamp &&
			    histb_vdec_decoded_buffer_valid(ref))
				return ref;
		}
	}

	buffer = vb2_find_buffer(queue, timestamp);
	if (buffer)
		return container_of(buffer, struct histb_vdec_decoded_buffer,
				   base.vb.vb2_buf);

	return NULL;
}

static void histb_vdec_reset_mpeg2_field_pair(struct histb_vdec_ctx *ctx)
{
	ctx->mpeg2_picture_started = false;
	ctx->mpeg2_picture_valid = false;
	ctx->mpeg2_field_picture = false;
	ctx->mpeg2_second_field = false;
	ctx->mpeg2_pending_failed = false;
	ctx->mpeg2_picture_structure = V4L2_MPEG2_PIC_FRAME;
	ctx->mpeg2_picture_coding_type = 0;
	ctx->mpeg2_pending_structure = V4L2_MPEG2_PIC_FRAME;
	ctx->mpeg2_pending_coding_type = 0;
	ctx->mpeg2_picture_width_mbs = 0;
	ctx->mpeg2_picture_height_mbs = 0;
	ctx->mpeg2_pending_width_mbs = 0;
	ctx->mpeg2_pending_frame_height_mbs = 0;
	ctx->mpeg2_pending_sequence = 0;
	ctx->mpeg2_pending_frame_flags = 0;
	ctx->mpeg2_ref_pic_type = 0;
	ctx->mpeg2_pending_timestamp = 0;
	ctx->mpeg2_pending_field = NULL;
}

static void histb_vdec_reset_apc(struct histb_vdec_ctx *ctx)
{
	unsigned int i;

	ctx->mpeg2_reference_started = false;
	histb_vdec_reset_mpeg2_field_pair(ctx);
	ctx->mpeg2_anchor[0] = NULL;
	ctx->mpeg2_anchor[1] = NULL;
	for (i = 0; i < ARRAY_SIZE(ctx->apc); i++) {
		if (ctx->apc[i]) {
			ctx->apc[i]->apc_slot = HISTB_VDEC_APC_INVALID;
			ctx->apc[i]->mpeg2_field_saved = false;
		}
		ctx->apc[i] = NULL;
	}
}

static u32 histb_vdec_mpeg2_frame_height_mbs(
		const struct v4l2_ctrl_mpeg2_sequence *sequence)
{
	if (sequence->flags & V4L2_MPEG2_SEQ_FLAG_PROGRESSIVE)
		return DIV_ROUND_UP(sequence->vertical_size, 16);

	return 2 * DIV_ROUND_UP(sequence->vertical_size, 32);
}

static u32 histb_vdec_mpeg2_picture_height_mbs(
		const struct v4l2_ctrl_mpeg2_sequence *sequence,
		const struct v4l2_ctrl_mpeg2_picture *picture)
{
	if (picture->picture_structure != V4L2_MPEG2_PIC_FRAME)
		return histb_vdec_mpeg2_frame_height_mbs(sequence) / 2;

	return DIV_ROUND_UP(sequence->vertical_size, 16);
}

static u32 histb_vdec_mpeg2_frame_flags(
		const struct v4l2_ctrl_mpeg2_picture *picture)
{
	if (picture->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_B)
		return V4L2_H264_DECODE_PARAM_FLAG_BFRAME;
	if (picture->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_P)
		return V4L2_H264_DECODE_PARAM_FLAG_PFRAME;

	return 0;
}

static int histb_vdec_begin_mpeg2_picture(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded, u64 timestamp,
		const struct v4l2_ctrl_mpeg2_sequence *sequence,
		const struct v4l2_ctrl_mpeg2_picture *picture)
{
	u32 width_mbs = DIV_ROUND_UP(sequence->horizontal_size, 16);
	u32 frame_height_mbs = histb_vdec_mpeg2_frame_height_mbs(sequence);
	bool field_picture = picture->picture_structure != V4L2_MPEG2_PIC_FRAME;
	bool second_field = false;

	if (field_picture && ctx->mpeg2_pending_field) {
		second_field = ctx->mpeg2_pending_field == decoded &&
			ctx->mpeg2_pending_timestamp == timestamp &&
			ctx->mpeg2_pending_width_mbs == width_mbs &&
			ctx->mpeg2_pending_frame_height_mbs == frame_height_mbs &&
			ctx->mpeg2_pending_structure != picture->picture_structure;
	}
	if (!second_field && ctx->mpeg2_pending_field) {
		ctx->dst.sequence++;
		histb_vdec_reset_mpeg2_field_pair(ctx);
	}

	ctx->mpeg2_picture_started = true;
	ctx->mpeg2_picture_valid = false;
	ctx->mpeg2_field_picture = field_picture;
	ctx->mpeg2_second_field = second_field;
	ctx->mpeg2_picture_structure = picture->picture_structure;
	ctx->mpeg2_picture_coding_type = picture->picture_coding_type;
	ctx->mpeg2_picture_width_mbs = width_mbs;
	ctx->mpeg2_picture_height_mbs =
		histb_vdec_mpeg2_picture_height_mbs(sequence, picture);
	if (second_field && ctx->mpeg2_pending_failed)
		return -EIO;

	return 0;
}

static void histb_vdec_finish_mpeg2_field_transaction(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		struct vb2_v4l2_buffer *dst,
		enum vb2_buffer_state state, bool surface_started)
{
	if (!ctx->mpeg2_picture_started) {
		if (state != VB2_BUF_STATE_DONE && ctx->mpeg2_pending_field) {
			dst->sequence = ctx->mpeg2_pending_sequence;
			ctx->dst.sequence++;
			histb_vdec_reset_mpeg2_field_pair(ctx);
		} else {
			dst->sequence = ctx->dst.sequence++;
		}
		return;
	}
	if (!ctx->mpeg2_field_picture) {
		dst->sequence = ctx->dst.sequence++;
		decoded->mpeg2_field_saved = false;
		histb_vdec_reset_mpeg2_field_pair(ctx);
		return;
	}
	if (ctx->mpeg2_second_field) {
		dst->sequence = ctx->mpeg2_pending_sequence;
		ctx->dst.sequence++;
		decoded->mpeg2_field_saved = ctx->mpeg2_picture_valid;
		histb_vdec_reset_mpeg2_field_pair(ctx);
		return;
	}

	ctx->mpeg2_pending_field = decoded;
	ctx->mpeg2_pending_timestamp = decoded->base.vb.vb2_buf.timestamp;
	ctx->mpeg2_pending_width_mbs = ctx->mpeg2_picture_width_mbs;
	ctx->mpeg2_pending_frame_height_mbs = 2 * ctx->mpeg2_picture_height_mbs;
	ctx->mpeg2_pending_structure = ctx->mpeg2_picture_structure;
	ctx->mpeg2_pending_coding_type = ctx->mpeg2_picture_coding_type;
	ctx->mpeg2_pending_sequence = ctx->dst.sequence;
	ctx->mpeg2_pending_frame_flags = ctx->frame_flags;
	ctx->mpeg2_pending_failed = state != VB2_BUF_STATE_DONE &&
		!surface_started;
	dst->sequence = ctx->mpeg2_pending_sequence;
	ctx->mpeg2_picture_started = false;
	ctx->mpeg2_picture_valid = false;
}

static bool histb_vdec_h264_self_field_entry(
		const struct histb_vdec_ctx *ctx,
		const struct histb_vdec_decoded_buffer *decoded,
		const struct v4l2_h264_dpb_entry *entry)
{
	u8 pending_field = ctx->h264_pending_bottom ?
		V4L2_H264_BOTTOM_FIELD_REF : V4L2_H264_TOP_FIELD_REF;

	if (!ctx->h264_second_field || ctx->h264_pending_failed ||
	    ctx->h264_pending_field != decoded ||
	    entry->reference_ts != ctx->h264_pending_timestamp ||
	    !(entry->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) ||
	    !(entry->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID) ||
	    !(entry->flags & V4L2_H264_DPB_ENTRY_FLAG_FIELD) ||
	    !(entry->fields & pending_field))
		return false;

	return (entry->flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM) ||
	       entry->frame_num == ctx->h264_pending_frame_num;
}

static bool histb_vdec_h264_current_apc_valid(
		const struct histb_vdec_ctx *ctx,
		const struct histb_vdec_decoded_buffer *decoded,
		const struct v4l2_ctrl_h264_decode_params *decode,
		const s8 dpb_to_apc[HISTB_VDEC_H264_DPB_MAP_SIZE])
{
	unsigned int i, matches = 0;

	if (decoded->apc_slot == HISTB_VDEC_APC_INVALID)
		return true;
	if (!ctx->h264_second_field)
		return false;

	for (i = 0; i < ARRAY_SIZE(decode->dpb); i++) {
		if (dpb_to_apc[i] != decoded->apc_slot)
			continue;
		if (!histb_vdec_h264_self_field_entry(ctx, decoded,
							  &decode->dpb[i]))
			return false;
		matches++;
	}

	return matches == 1;
}

static int
histb_vdec_sync_apc(struct histb_vdec_ctx *ctx,
		    struct histb_vdec_decoded_buffer *decoded,
		    const struct v4l2_ctrl_h264_decode_params *decode,
		    s8 dpb_to_apc[HISTB_VDEC_H264_DPB_MAP_SIZE])
{
	struct histb_vdec_decoded_buffer *active[HISTB_VDEC_H264_DPB_SIZE] = {};
	bool keep[HISTB_VDEC_H264_DPB_SIZE] = {};
	unsigned int i, slot;

	memset(dpb_to_apc, HISTB_VDEC_APC_INVALID, HISTB_VDEC_H264_DPB_SIZE);
	for (i = 0; i < ARRAY_SIZE(decode->dpb); i++) {
		unsigned int j;

		if (!(decode->dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE))
			continue;

		if (histb_vdec_h264_self_field_entry(ctx, decoded,
							      &decode->dpb[i]))
			active[i] = decoded;
		else
			active[i] = histb_vdec_find_reference(
				ctx, decode->dpb[i].reference_ts);
		if (!active[i] || !histb_vdec_decoded_buffer_valid(active[i]))
			return -EINVAL;
		for (j = 0; j < i; j++)
			if (active[j] == active[i])
				return -EINVAL;
		if (active[i]->apc_slot >= 0 &&
		    active[i]->apc_slot < ARRAY_SIZE(ctx->apc) &&
		    ctx->apc[active[i]->apc_slot] == active[i])
			keep[active[i]->apc_slot] = true;
		else
			active[i]->apc_slot = HISTB_VDEC_APC_INVALID;
	}

	for (slot = 0; slot < ARRAY_SIZE(ctx->apc); slot++) {
		if (!ctx->apc[slot] || keep[slot])
			continue;
		ctx->apc[slot]->apc_slot = HISTB_VDEC_APC_INVALID;
		ctx->apc[slot] = NULL;
	}

	for (i = 0; i < ARRAY_SIZE(decode->dpb); i++) {
		if (!active[i])
			continue;
		if (active[i]->apc_slot == HISTB_VDEC_APC_INVALID) {
			for (slot = 0; slot < ARRAY_SIZE(ctx->apc); slot++)
				if (!ctx->apc[slot])
					break;
			if (slot == ARRAY_SIZE(ctx->apc))
				return -ENOSPC;
			ctx->apc[slot] = active[i];
			active[i]->apc_slot = slot;
		}
		dpb_to_apc[i] = active[i]->apc_slot;
	}

	return 0;
}

static int
histb_vdec_sync_hevc_apc(struct histb_vdec_ctx *ctx,
			 const struct v4l2_ctrl_hevc_decode_params *decode,
			 s8 dpb_to_apc[V4L2_HEVC_DPB_ENTRIES_NUM_MAX])
{
	struct histb_vdec_decoded_buffer *active[V4L2_HEVC_DPB_ENTRIES_NUM_MAX] = {};
	bool keep[V4L2_HEVC_DPB_ENTRIES_NUM_MAX] = {};
	unsigned int i, slot;

	memset(dpb_to_apc, HISTB_VDEC_APC_INVALID,
	       V4L2_HEVC_DPB_ENTRIES_NUM_MAX);
	for (i = 0; i < decode->num_active_dpb_entries; i++) {
		unsigned int j;

		active[i] = histb_vdec_find_reference(ctx, decode->dpb[i].timestamp);
		if (!active[i] || !histb_vdec_decoded_buffer_valid(active[i]))
			return -EINVAL;
		for (j = 0; j < i; j++)
			if (active[j] == active[i])
				return -EINVAL;
		if (active[i]->apc_slot >= 0 &&
		    active[i]->apc_slot < ARRAY_SIZE(ctx->apc) &&
		    ctx->apc[active[i]->apc_slot] == active[i])
			keep[active[i]->apc_slot] = true;
		else
			active[i]->apc_slot = HISTB_VDEC_APC_INVALID;
	}

	for (slot = 0; slot < ARRAY_SIZE(ctx->apc); slot++) {
		if (!ctx->apc[slot] || keep[slot])
			continue;
		ctx->apc[slot]->apc_slot = HISTB_VDEC_APC_INVALID;
		histb_vdec_free_decoded_buffers(ctx, ctx->apc[slot]);
		ctx->apc[slot] = NULL;
	}
	for (i = 0; i < decode->num_active_dpb_entries; i++) {
		if (active[i]->apc_slot == HISTB_VDEC_APC_INVALID) {
			for (slot = 0; slot < ARRAY_SIZE(ctx->apc); slot++)
				if (!ctx->apc[slot])
					break;
			if (slot == ARRAY_SIZE(ctx->apc))
				return -ENOSPC;
			ctx->apc[slot] = active[i];
			active[i]->apc_slot = slot;
		}
		dpb_to_apc[i] = active[i]->apc_slot;
	}

	return 0;
}

static int histb_vdec_sync_mpeg2_apc(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		const struct v4l2_ctrl_mpeg2_picture *picture,
		struct histb_vdec_decoded_buffer **backward,
		struct histb_vdec_decoded_buffer **forward)
{
	struct histb_vdec_decoded_buffer *active[2] = { };
	bool self_forward;
	unsigned int count = 0, i, slot;

	*backward = NULL;
	*forward = NULL;
	if (picture->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_B) {
		*backward = histb_vdec_find_reference(ctx,
						      picture->backward_ref_ts);
		if (!*backward || !histb_vdec_decoded_buffer_valid(*backward))
			return -EINVAL;
		if (*backward == decoded)
			return -EINVAL;
		active[count++] = *backward;
	}
	if (picture->picture_coding_type != V4L2_MPEG2_PIC_CODING_TYPE_I) {
		self_forward = ctx->mpeg2_second_field &&
			!ctx->mpeg2_pending_failed &&
			!ctx->mpeg2_anchor[0] && !ctx->mpeg2_anchor[1] &&
			ctx->mpeg2_pending_field == decoded &&
			picture->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_P &&
			ctx->mpeg2_pending_coding_type ==
			V4L2_MPEG2_PIC_CODING_TYPE_I;
		/* A cold I/P field pair has no prior full-frame forward anchor. */
		*forward = self_forward ? decoded :
			histb_vdec_find_reference(ctx, picture->forward_ref_ts);
		if (!*forward || !histb_vdec_decoded_buffer_valid(*forward)) {
			return -EINVAL;
		} else if (*forward == decoded) {
			if (!self_forward)
				return -EINVAL;
		} else if (*forward != decoded) {
			active[count++] = *forward;
		}
	}

	for (i = 0; i < count; i++) {
		if (active[i]->apc_slot >= 0 &&
		    active[i]->apc_slot < ARRAY_SIZE(ctx->apc) &&
		    ctx->apc[active[i]->apc_slot] == active[i])
			continue;
		active[i]->apc_slot = HISTB_VDEC_APC_INVALID;
		for (slot = 0; slot < ARRAY_SIZE(ctx->apc); slot++)
			if (!ctx->apc[slot])
				break;
		if (slot == ARRAY_SIZE(ctx->apc))
			return -ENOSPC;
		ctx->apc[slot] = active[i];
		active[i]->apc_slot = slot;
	}
	return 0;
}

static int histb_vdec_commit_mpeg2_reference(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded)
{
	struct histb_vdec_decoded_buffer *oldest;
	unsigned int slot;

	if (ctx->frame_flags & V4L2_H264_DECODE_PARAM_FLAG_BFRAME)
		return 0;
	if (!histb_vdec_decoded_buffer_valid(decoded))
		return -EINVAL;
	if (decoded->apc_slot != HISTB_VDEC_APC_INVALID)
		return -EINVAL;

	/*
	 * MPEG-2 B pictures around a GOP boundary use both the previous
	 * anchor and the newly decoded I/P anchor.  Retain those two surfaces;
	 * the older one is no longer part of the normative two-anchor window.
	 */
	oldest = ctx->mpeg2_anchor[0];
	if (oldest) {
		if (oldest->apc_slot < 0 ||
		    oldest->apc_slot >= ARRAY_SIZE(ctx->apc) ||
		    ctx->apc[oldest->apc_slot] != oldest)
			return -EINVAL;
		ctx->apc[oldest->apc_slot] = NULL;
		oldest->apc_slot = HISTB_VDEC_APC_INVALID;
		histb_vdec_free_decoded_buffers(ctx, oldest);
	}

	for (slot = 0; slot < ARRAY_SIZE(ctx->apc); slot++)
		if (!ctx->apc[slot])
			break;
	if (slot == ARRAY_SIZE(ctx->apc))
		return -ENOSPC;

	ctx->apc[slot] = decoded;
	decoded->apc_slot = slot;
	ctx->mpeg2_anchor[0] = ctx->mpeg2_anchor[1];
	ctx->mpeg2_anchor[1] = decoded;
	return 0;
}

static int histb_vdec_sync_vp8_apc(
		struct histb_vdec_ctx *ctx,
		const struct v4l2_ctrl_vp8_frame *frame,
		struct histb_vdec_decoded_buffer **last,
		struct histb_vdec_decoded_buffer **golden,
		struct histb_vdec_decoded_buffer **alt)
{
	struct histb_vdec_decoded_buffer *active[3] = { };
	struct histb_vdec_decoded_buffer **refs[] = { last, golden, alt };
	u64 timestamps[] = { frame->last_frame_ts, frame->golden_frame_ts,
			     frame->alt_frame_ts };
	bool keep[HISTB_VDEC_H264_DPB_SIZE] = { };
	unsigned int active_count = 0, i, slot;

	*last = NULL;
	*golden = NULL;
	*alt = NULL;
	if (!V4L2_VP8_FRAME_IS_KEY_FRAME(frame)) {
		for (i = 0; i < ARRAY_SIZE(refs); i++) {
			unsigned int j;

			*refs[i] = histb_vdec_find_reference(ctx, timestamps[i]);
			if (!*refs[i] ||
			    !histb_vdec_decoded_buffer_valid(*refs[i]))
				return -EINVAL;
			for (j = 0; j < active_count; j++)
				if (active[j] == *refs[i])
					break;
			if (j == active_count)
				active[active_count++] = *refs[i];
		}
	}

	for (i = 0; i < active_count; i++) {
		if (active[i]->apc_slot >= 0 &&
		    active[i]->apc_slot < ARRAY_SIZE(ctx->apc) &&
		    ctx->apc[active[i]->apc_slot] == active[i])
			keep[active[i]->apc_slot] = true;
		else
			active[i]->apc_slot = HISTB_VDEC_APC_INVALID;
	}
	for (slot = 0; slot < ARRAY_SIZE(ctx->apc); slot++) {
		if (!ctx->apc[slot] || keep[slot])
			continue;
		ctx->apc[slot]->apc_slot = HISTB_VDEC_APC_INVALID;
		histb_vdec_free_decoded_buffers(ctx, ctx->apc[slot]);
		ctx->apc[slot] = NULL;
	}
	for (i = 0; i < active_count; i++) {
		if (active[i]->apc_slot != HISTB_VDEC_APC_INVALID)
			continue;
		for (slot = 0; slot < ARRAY_SIZE(ctx->apc); slot++)
			if (!ctx->apc[slot])
				break;
		if (slot == ARRAY_SIZE(ctx->apc))
			return -ENOSPC;
		ctx->apc[slot] = active[i];
		active[i]->apc_slot = slot;
	}

	return 0;
}

static void
histb_vdec_reclaim_decoded_buffers(struct histb_vdec_ctx *ctx,
				   struct histb_vdec_decoded_buffer *output)
{
	struct vb2_queue *queue = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	unsigned int i;

	for (i = 0; i < queue->max_num_buffers; i++) {
		struct vb2_buffer *buffer = queue->bufs[i];
		struct histb_vdec_decoded_buffer *decoded;

		if (!buffer)
			continue;
		decoded = container_of(buffer, struct histb_vdec_decoded_buffer,
				       base.vb.vb2_buf);
		/* H.264 references may leave the request DPB and reappear after MMCO. */
		if (decoded == output ||
		    ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE ||
		    (ctx->src.pix.pixelformat == V4L2_PIX_FMT_AVS_SLICE &&
		     decoded->avs.valid) ||
		    decoded->vp9_dpb_valid ||
		    decoded->apc_slot != HISTB_VDEC_APC_INVALID)
			continue;
		histb_vdec_free_decoded_buffers(ctx, decoded);
	}
}

static int histb_vdec_configure_vdh_clock(struct histb_vdec_dev *vdec)
{
	u32 mask = HISTB_CRG_VDH_CLK_SEL | HISTB_CRG_VDH_CLK_SKIP |
		   HISTB_CRG_VDH_CLK_LOAD;
	int ret;

	/* The vendor HAL pulses LOAD after selecting the full-rate VDH clock. */
	ret = regmap_update_bits(vdec->crg, HISTB_CRG_VDH_CLOCK, mask, 0);
	if (ret)
		return ret;

	return regmap_update_bits(vdec->crg, HISTB_CRG_VDH_CLOCK,
				  HISTB_CRG_VDH_CLK_LOAD,
				  HISTB_CRG_VDH_CLK_LOAD);
}

static void histb_vdec_account_job(struct histb_vdec_dev *vdec, bool error)
{
	unsigned long flags;

	spin_lock_irqsave(&vdec->irqlock, flags);
	if (vdec->job_count_pending) {
		vdec->job_count_pending = false;
		vdec->jobs++;
		if (error)
			vdec->errors++;
	}
	spin_unlock_irqrestore(&vdec->irqlock, flags);
}

static ssize_t histb_vdec_jobs_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct histb_vdec_dev *vdec = dev_get_drvdata(dev);
	unsigned long flags;
	u64 jobs;

	spin_lock_irqsave(&vdec->irqlock, flags);
	jobs = vdec->jobs;
	spin_unlock_irqrestore(&vdec->irqlock, flags);

	return sysfs_emit(buf, "%llu\n", jobs);
}

static ssize_t histb_vdec_errors_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct histb_vdec_dev *vdec = dev_get_drvdata(dev);
	unsigned long flags;
	u64 errors;

	spin_lock_irqsave(&vdec->irqlock, flags);
	errors = vdec->errors;
	spin_unlock_irqrestore(&vdec->irqlock, flags);

	return sysfs_emit(buf, "%llu\n", errors);
}

static DEVICE_ATTR_RO(histb_vdec_jobs);
static DEVICE_ATTR_RO(histb_vdec_errors);

static struct attribute *histb_vdec_attrs[] = {
	&dev_attr_histb_vdec_jobs.attr,
	&dev_attr_histb_vdec_errors.attr,
	NULL,
};

static const struct attribute_group histb_vdec_attr_group = {
	.attrs = histb_vdec_attrs,
};

static const u8 histb_vdec_row_map_y[128] = {
	0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15,
	12, 0, 13, 1, 14, 2, 15, 3, 8, 4, 9, 5, 10, 6, 11, 7,
	4, 12, 5, 13, 6, 14, 7, 15, 0, 8, 1, 9, 2, 10, 3, 11,
	8, 4, 9, 5, 10, 6, 11, 7, 12, 0, 13, 1, 14, 2, 15, 3,
	8, 0, 9, 1, 10, 2, 11, 3, 12, 4, 13, 5, 14, 6, 15, 7,
	0, 12, 1, 13, 2, 14, 3, 15, 4, 8, 5, 9, 6, 10, 7, 11,
	12, 4, 13, 5, 14, 6, 15, 7, 8, 0, 9, 1, 10, 2, 11, 3,
	4, 8, 5, 9, 6, 10, 7, 11, 0, 12, 1, 13, 2, 14, 3, 15,
};

static const u8 histb_vdec_row_map_uv[64] = {
	0, 4, 1, 5, 2, 6, 3, 7, 4, 0, 5, 1, 6, 2, 7, 3,
	4, 0, 5, 1, 6, 2, 7, 3, 0, 4, 1, 5, 2, 6, 3, 7,
	4, 0, 5, 1, 6, 2, 7, 3, 0, 4, 1, 5, 2, 6, 3, 7,
	0, 4, 1, 5, 2, 6, 3, 7, 4, 0, 5, 1, 6, 2, 7, 3,
};

static u8 histb_vdec_row_map_y_2bit(u32 index)
{
	u32 group;
	u32 row;

	if (index >= 128)
		return histb_vdec_row_map_y[index - 128];
	group = index / 32;
	row = index % 32;
	if (group == 0 || group == 3)
		return (row >> 1) + ((row & 1) << 4);

	return (row >> 1) + (!(row & 1) << 4);
}

static u8 histb_vdec_row_map_uv_2bit(u32 index)
{
	return (index >> 1) + ((index & 1) << 3);
}

static __le64 histb_vdec_pack_p010(u32 high, u8 low, bool swap_uv)
{
	u64 packed;

	if (swap_uv) {
		high = ((high & 0x00ff00ff) << 8) |
		       ((high & 0xff00ff00) >> 8);
		low = ((low & 0x33) << 2) | ((low & 0xcc) >> 2);
	}
	packed = ((u64)(high & 0xff) << 8) |
		 ((u64)(low & 0x03) << 6) |
		 ((u64)((high >> 8) & 0xff) << 24) |
		 ((u64)((low >> 2) & 0x03) << 22) |
		 ((u64)((high >> 16) & 0xff) << 40) |
		 ((u64)((low >> 4) & 0x03) << 38) |
		 ((u64)((high >> 24) & 0xff) << 56) |
		 ((u64)((low >> 6) & 0x03) << 54);

	return cpu_to_le64(packed);
}

static __le16 histb_vdec_p010_sample(const u8 *high, u8 low, u32 offset,
				     bool swap_uv)
{
	u32 source = swap_uv ? offset ^ 1 : offset;

	return cpu_to_le16((high[source] << 8) |
		(((low >> (2 * (source & 3))) & 3) << 6));
}

static void histb_vdec_detile_nv12(struct histb_vdec_ctx *ctx,
				   const u8 *src, u8 *dst, bool swap_uv)
{
	u32 width = ctx->dst.pix.width;
	u32 height = ctx->dst.pix.height;
	u32 stride = histb_vdec_surface_stride(ctx);
	u32 chroma_base = stride *
		ALIGN(height, histb_vdec_surface_height_align(ctx));
	u32 row, column;

	for (row = 0; row < height; row++) {
		for (column = 0; column < width; column += 64) {
			u32 tile_x = column / 64;
			u32 index = (row & 15) + 16 *
				(((row / 16) & 1) * 4 + (tile_x & 3));
			u32 source = stride * 16 * (row / 16) +
				1024 * tile_x + 64 * histb_vdec_row_map_y[index];
			u32 length = min_t(u32, width - column, 64);

			memcpy(dst + row * width + column, src + source, length);
		}
	}

	for (row = 0; row < height / 2; row++) {
		for (column = 0; column < width; column += 64) {
			u32 tile_x = column / 64;
			u32 index = (row & 7) + 8 *
				(((row / 8) & 1) * 4 + (tile_x & 3));
			u32 source = chroma_base + stride * 8 * (row / 8) +
				512 * tile_x + 64 * histb_vdec_row_map_uv[index];
			u32 length = min_t(u32, width - column, 64);
			u32 offset;

			if (!swap_uv) {
				memcpy(dst + width * height + row * width + column,
				       src + source, length);
				continue;
			}
			for (offset = 0; offset < length; offset += 2) {
				dst[width * height + row * width + column + offset] =
					src[source + offset + 1];
				dst[width * height + row * width + column + offset + 1] =
					src[source + offset];
			}
		}
	}
}

static void histb_vdec_detile_p010(struct histb_vdec_ctx *ctx,
				   const u8 *src, __le16 *dst)
{
	u32 width = ctx->dst.pix.width;
	u32 height = ctx->dst.pix.height;
	u32 stride = histb_vdec_surface_stride(ctx);
	u32 aligned_height = ALIGN(height, histb_vdec_surface_height_align(ctx));
	u32 chroma_base = stride * aligned_height;
	u32 low_base = chroma_base * 3 / 2;
	u32 low_chroma_base = low_base + (stride / 4) * aligned_height;
	u32 row, column;

	for (row = 0; row < height; row++) {
		for (column = 0; column < width; column += 64) {
			u32 tile_x = column / 64;
			u32 high_index = (row & 15) + 16 *
				(((row / 16) & 1) * 4 + (tile_x & 3));
			u32 high_source = stride * 16 * (row / 16) +
				1024 * tile_x +
				64 * histb_vdec_row_map_y[high_index];
			u32 low_index = (row & 31) + ((tile_x & 3) << 5);
			u32 low_source = low_base + stride * 8 * (row / 32) +
				512 * tile_x +
				16 * histb_vdec_row_map_y_2bit(low_index);
			u32 length = min_t(u32, width - column, 64);
			u32 offset;

			for (offset = 0; offset + 4 <= length; offset += 4)
				put_unaligned(histb_vdec_pack_p010(
					get_unaligned_le32(src + high_source + offset),
					src[low_source + offset / 4], false),
					(__le64 *)(dst + row * width + column + offset));
			for (; offset < length; offset++)
				dst[row * width + column + offset] =
					histb_vdec_p010_sample(src + high_source,
							       src[low_source + offset / 4], offset,
						false);
		}
	}

	for (row = 0; row < height / 2; row++) {
		for (column = 0; column < width; column += 64) {
			u32 tile_x = column / 64;
			u32 high_index = (row & 7) + 8 *
				(((row / 8) & 1) * 4 + (tile_x & 3));
			u32 high_source = chroma_base + stride * 8 * (row / 8) +
				512 * tile_x +
				64 * histb_vdec_row_map_uv[high_index];
			u32 low_index = row & 15;
			u32 low_source = low_chroma_base + stride * 4 * (row / 16) +
				256 * tile_x +
				16 * histb_vdec_row_map_uv_2bit(low_index);
			u32 length = min_t(u32, width - column, 64);
			u32 offset;

			for (offset = 0; offset + 4 <= length; offset += 4)
				put_unaligned(histb_vdec_pack_p010(
					get_unaligned_le32(src + high_source + offset),
					src[low_source + offset / 4], true),
					(__le64 *)(dst + width * height + row * width +
						 column + offset));
			for (; offset < length; offset++)
				dst[width * height + row * width + column + offset] =
					histb_vdec_p010_sample(src + high_source,
							       src[low_source + offset / 4], offset,
						true);
		}
	}
}

static int histb_vdec_apply_vc1_range_map(
			struct histb_vdec_ctx *ctx, struct vb2_v4l2_buffer *dst_buf,
			u8 *dst)
{
	int ret;

	if (!ctx->vc1 || ctx->vc1_annex_l ||
	    (!ctx->vc1_entry.range_map_y_flag &&
	     !ctx->vc1_entry.range_map_uv_flag))
		return 0;
	if (ctx->dst.pix.pixelformat != V4L2_PIX_FMT_NV12 || !dst)
		return -EOPNOTSUPP;
	ret = histb_vdec_capture_begin_cpu_access(&dst_buf->vb2_buf);
	if (ret)
		return ret;
	ret = histb_vc1_range_map_nv12(dst, ctx->dst.pix.bytesperline,
					ctx->dst.pix.width, ctx->dst.pix.height,
					&ctx->vc1_entry);
	return ret == HISTB_VC1_OK ? 0 : -EINVAL;
}

static void histb_vdec_scale_plane_nearest(u8 *plane, u32 stride,
					   u32 input_width, u32 input_height,
					   u32 output_width, u32 output_height,
					   u32 bytes_per_sample)
{
	u32 y;

	for (y = output_height; y--;) {
		u32 input_y = div_u64((u64)y * input_height, output_height);
		u8 *output = plane + y * stride;
		const u8 *input = plane + input_y * stride;
		u32 x;

		for (x = output_width; x--;) {
			u32 input_x = div_u64((u64)x * input_width,
						  output_width);

			memcpy(output + x * bytes_per_sample,
			       input + input_x * bytes_per_sample,
			       bytes_per_sample);
		}
	}
}

static void histb_vdec_scale_vc1_respic_nv12(u8 *dst, u32 stride,
					      u32 input_width, u32 input_height,
					      u32 output_width, u32 output_height)
{
	u8 *chroma = dst + stride * output_height;

	histb_vdec_scale_plane_nearest(dst, stride, input_width, input_height,
				       output_width, output_height, 1);
	histb_vdec_scale_plane_nearest(chroma, stride, input_width / 2,
				       input_height / 2, output_width / 2,
				       output_height / 2, 2);
}

/*
 * The DMA address of a capture buffer, for whichever backend allocated it.
 *
 * vb2_dma_contig_plane_dma_addr() reads vb->planes[].mem_priv as a
 * struct vb2_dma_contig_buf.  That is only true for the dma-contig backend;
 * the cacheable backend stores a struct histb_vdec_capture_mem there instead,
 * so calling the dma-contig accessor on it dereferences the wrong struct and
 * reads a NULL dma_addr_t, which is what produced the
 * "Unable to handle kernel NULL pointer dereference" at
 * histb_vdec_copy_capture+0x88 in the postprocess workqueue.
 */
static dma_addr_t histb_vdec_capture_dma(struct vb2_buffer *vb,
					 unsigned int plane)
{
	struct histb_vdec_capture_mem *mem = vb->planes[plane].mem_priv;

	if (mem->ops == &histb_vdec_cached_memops)
		return ((struct histb_vdec_cached_buf *)mem->priv)->dma;

	/*
	 * vb2_vmalloc_memops has no device address at all - it is plain CPU
	 * memory - so it must not reach the dma-contig accessor either.  No
	 * caller asks for an address on a vmalloc buffer today (the only call
	 * site is guarded by NV12, which selects dma-contig), but reading the
	 * wrong struct here is exactly the fault that was just fixed, and
	 * leaving the guard implicit would reintroduce it the first time
	 * someone asks for a P010 capture address.
	 */
	if (mem->ops == &vb2_vmalloc_memops)
		return 0;

	return vb2_dma_contig_plane_dma_addr(vb, plane);
}

static int histb_vdec_copy_capture(struct histb_vdec_ctx *ctx)
{
	struct histb_vdec_decoded_buffer *decoded;
	struct histb_vpss_frame frame;
	struct vb2_v4l2_buffer *dst_buf;
	dma_addr_t dst_dma;
	u8 *dst;
	int ret;

	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (!dst_buf)
		return -EPIPE;
	decoded = histb_vdec_decoded_buffer(dst_buf);
	dst = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
	if (!decoded->tile.cpu)
		return -EOPNOTSUPP;
	dst_dma = 0;
	if (ctx->dst.pix.pixelformat == V4L2_PIX_FMT_NV12) {
		dst_dma = histb_vdec_capture_dma(&dst_buf->vb2_buf, 0);
		dst_dma += dst_buf->vb2_buf.planes[0].data_offset;
	}
	if (dst)
		dst += dst_buf->vb2_buf.planes[0].data_offset;

	/*
	 * The VDH completion IRQ can precede the final 4K FSP store bursts.
	 * Poll for the state to settle instead of always paying a fixed 50 ms:
	 * in the common case it is already final, and 50 ms out of the ~166 ms
	 * a 4K frame costs was pure waste.  Bail out as soon as DECODE_DONE is
	 * visible and the state stops changing.
	 */
	if (ctx->dst.pix.width >= 3840) {
		u32 prev = 0, cur = 0;
		unsigned int spin;

		for (spin = 0; spin < 60; spin++) {
			cur = readl(ctx->vdec->regs + HISTB_VDEC_STATE);
			if (spin && cur == prev &&
			    (cur & HISTB_VDEC_STATE_DECODE_DONE))
				break;
			prev = cur;
			usleep_range(500, 1000);
		}
	}
	dma_rmb();
	/* VC-1 reconstruction surfaces are allocated non-coherent.  The VDH
	 * completion only orders the engine; it does not invalidate the VPSS
	 * device view of the surface. */
	if (decoded->tile_noncoherent)
		dma_sync_single_for_device(ctx->vdec->dev, decoded->tile.dma,
					decoded->tile.size, DMA_BIDIRECTIONAL);
	memset(&frame, 0, sizeof(frame));
	frame.input_dma = decoded->tile.dma;
	if (ctx->dst.pix.pixelformat == V4L2_PIX_FMT_NV12)
		frame.output_dma = dst_dma;
	frame.output_cpu = dst;
	/*
	 * Input and output are the same size: this driver does not scale.
	 * Splitting them to engage the VPSS ZME was tried (histb_vdec_core)
	 * and made things worse - 1080p transcoding fell from 0.91x to 0.73x
	 * and 4K produced almost no output, because ctx->src.pix is not yet
	 * the decoded size at this point.  Do not reintroduce it without
	 * first proving ctx->src is populated.
	 */
	frame.input_width = ctx->dst.pix.width;
	frame.input_height = ctx->dst.pix.height;
	frame.width = ctx->dst.pix.width;
	frame.height = ctx->dst.pix.height;
	if (ctx->vc1 && ctx->vc1_annex_l &&
	    ctx->vc1_pending_picture.res_pic) {
		frame.input_width = ctx->vc1_pending_picture.coded_width;
		frame.input_height = ctx->vc1_pending_picture.coded_height;
	}
	frame.input_stride = histb_vdec_surface_stride(ctx);
	frame.input_height_align = histb_vdec_surface_height_align(ctx);
	frame.output_stride = ctx->dst.pix.bytesperline;
	frame.input_ten_bit = ctx->hevc_main10;
	frame.output_ten_bit =
		ctx->dst.pix.pixelformat == V4L2_PIX_FMT_P010;
	ret = histb_vpss_detile(ctx->vdec->vpss, &frame);
	if (!ret) {
		/*
		 * VPSS wrote the output surface by DMA.  With cacheable capture
		 * buffers those writes sit in the cache as dirty lines the CPU
		 * may not see, so invalidate before anyone reads the frame.
		 * begin_cpu_access() is idempotent, so the call inside
		 * apply_vc1_range_map() is harmless when it also fires.
		 */
		ret = histb_vdec_capture_begin_cpu_access(&dst_buf->vb2_buf);
		if (ret)
			return ret;
		return histb_vdec_apply_vc1_range_map(ctx, dst_buf, dst);
	}
	dev_warn_ratelimited(ctx->vdec->dev,
			     "VPSS post-process failed (%d), using CPU fallback\n",
			     ret);
	if (!dst)
		return ret;
	ret = histb_vdec_capture_begin_cpu_access(&dst_buf->vb2_buf);
	if (ret)
		return ret;
	if (decoded->tile_noncoherent)
		dma_sync_single_for_cpu(ctx->vdec->dev, decoded->tile.dma,
					decoded->tile.size,
					DMA_BIDIRECTIONAL);
	/* Order reconstruction-surface DMA writes after the observed VDH IRQ. */
	dma_rmb();
	if (ctx->dst.pix.pixelformat == V4L2_PIX_FMT_P010)
		histb_vdec_detile_p010(ctx, decoded->tile.cpu, (__le16 *)dst);
	else {
		histb_vdec_detile_nv12(ctx, decoded->tile.cpu, dst, true);
		if (frame.input_width != frame.width ||
		    frame.input_height != frame.height)
			histb_vdec_scale_vc1_respic_nv12(dst,
							 ctx->dst.pix.bytesperline,
							 frame.input_width,
							 frame.input_height,
							 frame.width, frame.height);
	}
	return histb_vdec_apply_vc1_range_map(ctx, dst_buf, dst);
}

static inline struct histb_vdec_ctx *fh_to_histb_vdec_ctx(void *priv)
{
	return container_of(priv, struct histb_vdec_ctx, fh);
}

static struct histb_vdec_q_data *
histb_vdec_get_q_data(struct histb_vdec_ctx *ctx, enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->src;

	return &ctx->dst;
}

static dma_addr_t histb_vdec_buffer_dma(struct histb_vdec_ctx *ctx,
					enum histb_vdec_buffer_id id)
{
	return ctx->buffers[id].dma;
}

static __le32 *histb_vdec_msg_slot(struct histb_vdec_ctx *ctx,
				   unsigned int slot)
{
	return ctx->buffers[HISTB_VDEC_BUF_MSG].cpu +
	       slot * HISTB_VDEC_H264_MSG_SLOT_SIZE;
}

static __le32 *histb_vdec_hevc_msg_slot(struct histb_vdec_ctx *ctx,
					unsigned int slot)
{
	__le32 *base = ctx->buffers[HISTB_VDEC_BUF_MSG].cpu;
	unsigned int words = slot;

	words *= HISTB_VDEC_HEVC_MSG_SLOT_WORDS;
	return base + words;
}

static __le32 *histb_vdec_avs_pic_msg(struct histb_vdec_ctx *ctx)
{
	u8 *base = ctx->buffers[HISTB_VDEC_BUF_MSG].cpu;

	return (__le32 *)(base + HISTB_VDEC_AVS_PIC_MSG_OFFSET);
}

static __le32 *histb_vdec_avs_slice_msg(struct histb_vdec_ctx *ctx,
					unsigned int slice)
{
	u8 *base = ctx->buffers[HISTB_VDEC_BUF_MSG].cpu;

	return (__le32 *)(base + HISTB_VDEC_AVS_SLICE_MSG_OFFSET +
			  slice * HISTB_VDEC_AVS_SLICE_MSG_SIZE);
}

static bool histb_vdec_dma_buffers_valid(struct histb_vdec_ctx *ctx)
{
	unsigned int i;

	for (i = 0; i < HISTB_VDEC_BUF_COUNT; i++)
		if (ctx->buffers[i].size &&
		    (!ctx->buffers[i].cpu || upper_32_bits(ctx->buffers[i].dma)))
			return false;

	return true;
}

static bool histb_vdec_smmu_dma_range_valid(dma_addr_t dma, size_t size)
{
	dma_addr_t end;

	if (!size || size > HISTB_VDEC_SMMU_DMA_SIZE || dma < SZ_1M ||
	    check_add_overflow(dma, (dma_addr_t)(size - 1), &end))
		return false;

	return end < HISTB_VDEC_SMMU_DMA_SIZE;
}

static bool
histb_vdec_smmu_dma_buffer_valid(const struct histb_vdec_dma_buffer *buffer)
{
	return buffer->cpu &&
	       histb_vdec_smmu_dma_range_valid(buffer->dma, buffer->size);
}

static bool histb_vdec_smmu_dma_buffers_valid(struct histb_vdec_ctx *ctx)
{
	unsigned int i;

	for (i = 0; i < HISTB_VDEC_BUF_COUNT; i++)
		if (ctx->buffers[i].size &&
		    !histb_vdec_smmu_dma_buffer_valid(&ctx->buffers[i]))
			return false;

	return true;
}

static int
histb_vdec_smmu_aligned_view(const struct histb_vdec_dma_buffer *raw,
			     size_t alignment, size_t size, void **cpu,
			     dma_addr_t *dma)
{
	dma_addr_t aligned;
	dma_addr_t end;
	size_t offset;

	if (!raw->cpu || !raw->size || !size || !alignment)
		return -EINVAL;

	aligned = ALIGN(raw->dma, alignment);
	if (aligned < raw->dma)
		return -ERANGE;
	offset = (size_t)(aligned - raw->dma);
	if (offset > raw->size || size > raw->size - offset)
		return -ERANGE;
	end = aligned + size - 1;
	if (end < aligned || upper_32_bits(end))
		return -ERANGE;

	*cpu = (u8 *)raw->cpu + offset;
	*dma = aligned;
	return 0;
}

static void histb_vdec_release_smmu_identity(void *data)
{
	struct histb_vdec_dev *vdec = data;

	vdec->smmu_identity_ready = false;
	vdec->smmu_identity_needs_reset = false;

	if (vdec->smmu_pt_raw.cpu)
		dma_free_coherent(vdec->dev, vdec->smmu_pt_raw.size,
				  vdec->smmu_pt_raw.cpu, vdec->smmu_pt_raw.dma);
	if (vdec->smmu_err_rd_raw.cpu)
		dma_free_coherent(vdec->dev, vdec->smmu_err_rd_raw.size,
				  vdec->smmu_err_rd_raw.cpu,
				  vdec->smmu_err_rd_raw.dma);
	if (vdec->smmu_err_wr_raw.cpu)
		dma_free_coherent(vdec->dev, vdec->smmu_err_wr_raw.size,
				  vdec->smmu_err_wr_raw.cpu,
				  vdec->smmu_err_wr_raw.dma);

	memset(&vdec->smmu_pt_raw, 0, sizeof(vdec->smmu_pt_raw));
	memset(&vdec->smmu_err_rd_raw, 0, sizeof(vdec->smmu_err_rd_raw));
	memset(&vdec->smmu_err_wr_raw, 0, sizeof(vdec->smmu_err_wr_raw));
	vdec->smmu_pt_cpu = NULL;
	vdec->smmu_err_rd_cpu = NULL;
	vdec->smmu_err_wr_cpu = NULL;
	vdec->smmu_pt_dma = 0;
	vdec->smmu_err_rd_dma = 0;
	vdec->smmu_err_wr_dma = 0;
}

static int histb_vdec_prepare_smmu_identity(struct histb_vdec_dev *vdec)
{
	size_t pt_alloc;
	unsigned int i;
	int ret;

	BUILD_BUG_ON(HISTB_VDEC_SMMU_PGTABLE_ENTRIES * sizeof(__le32) >
		     HISTB_VDEC_SMMU_PGTABLE_SIZE);
	if (vdec->smmu_identity_ready)
		return 0;

	/*
	 * One allocation carries the identity page table and both error
	 * buffers.  Allocating the 512-byte error buffers separately puts them
	 * below 1 MiB on this platform, and the vendor domain reserves the
	 * first and last MiB, so histb_vdec_smmu_dma_range_valid() rejects them
	 * and AVS never starts streaming.  Carving them out of the page-table
	 * allocation keeps all three inside the aperture the engine can address.
	 */
	if (check_add_overflow((size_t)HISTB_VDEC_SMMU_PGTABLE_SIZE,
			       (size_t)HISTB_VDEC_SMMU_PGTABLE_ALIGN - 1,
			       &pt_alloc) ||
	    check_add_overflow(pt_alloc, (size_t)HISTB_VDEC_SMMU_ERROR_POOL,
			       &pt_alloc))
		return -EOVERFLOW;

	vdec->smmu_pt_raw.size = pt_alloc;
	vdec->smmu_pt_raw.cpu =
		dma_alloc_coherent(vdec->dev, pt_alloc,
				   &vdec->smmu_pt_raw.dma, GFP_KERNEL);
	if (!vdec->smmu_pt_raw.cpu) {
		ret = -ENOMEM;
		goto fail;
	}
	ret = histb_vdec_smmu_aligned_view(&vdec->smmu_pt_raw,
					   HISTB_VDEC_SMMU_PGTABLE_ALIGN,
					   HISTB_VDEC_SMMU_PGTABLE_SIZE,
					   &vdec->smmu_pt_cpu,
					   &vdec->smmu_pt_dma);
	if (ret)
		goto fail;

	/*
	 * The error buffers live immediately past the page table, inside the
	 * allocation whose range has already been checked against the engine's
	 * aperture.  The raw descriptors stay empty so the release path, which
	 * frees whatever they name, does not free the carve separately.
	 */
	vdec->smmu_err_rd_cpu = (u8 *)vdec->smmu_pt_cpu +
		HISTB_VDEC_SMMU_PGTABLE_SIZE;
	vdec->smmu_err_rd_dma = vdec->smmu_pt_dma +
		HISTB_VDEC_SMMU_PGTABLE_SIZE;
	vdec->smmu_err_wr_cpu = (u8 *)vdec->smmu_err_rd_cpu +
		HISTB_VDEC_SMMU_ERROR_SIZE;
	vdec->smmu_err_wr_dma = vdec->smmu_err_rd_dma +
		HISTB_VDEC_SMMU_ERROR_SIZE;
	if (vdec->smmu_err_wr_dma + HISTB_VDEC_SMMU_ERROR_SIZE >
	    vdec->smmu_pt_raw.dma + vdec->smmu_pt_raw.size) {
		ret = -ENOMEM;
		goto fail;
	}

	if (!histb_vdec_smmu_dma_range_valid(vdec->smmu_pt_dma,
					     HISTB_VDEC_SMMU_PGTABLE_SIZE) ||
	    !histb_vdec_smmu_dma_range_valid(vdec->smmu_err_rd_dma,
					     HISTB_VDEC_SMMU_ERROR_SIZE) ||
	    !histb_vdec_smmu_dma_range_valid(vdec->smmu_err_wr_dma,
					     HISTB_VDEC_SMMU_ERROR_SIZE)) {
		dev_err(vdec->dev,
			"identity setup outside the engine aperture: pt=0x%llx rd=0x%llx wr=0x%llx, window 0x%lx..0x%lx\n",
			(unsigned long long)vdec->smmu_pt_dma,
			(unsigned long long)vdec->smmu_err_rd_dma,
			(unsigned long long)vdec->smmu_err_wr_dma,
			(unsigned long)SZ_1M,
			(unsigned long)HISTB_VDEC_SMMU_DMA_SIZE);
		ret = -ERANGE;
		goto fail;
	}

	memset(vdec->smmu_pt_raw.cpu, 0, vdec->smmu_pt_raw.size);
	/*
	 * The vendor domain reserves the first and last MiB and uses one
	 * little-endian 32-bit PTE per 4 KiB page. DMA is already physical on
	 * this platform, so an identity table supplies the VDH translation mode
	 * AVS requires. AVS-facing DMA allocations are constrained to the SoC DMA
	 * aperture.
	 */
	for (i = SZ_1M / HISTB_VDEC_SMMU_PAGE_SIZE;
	     i < HISTB_VDEC_SMMU_DMA_SIZE / HISTB_VDEC_SMMU_PAGE_SIZE; i++)
		((__le32 *)vdec->smmu_pt_cpu)[i] =
			cpu_to_le32(i * HISTB_VDEC_SMMU_PAGE_SIZE | 1);
	memset(vdec->smmu_err_rd_cpu, 0, HISTB_VDEC_SMMU_ERROR_SIZE);
	memset(vdec->smmu_err_wr_cpu, 0, HISTB_VDEC_SMMU_ERROR_SIZE);
	dma_wmb();
	vdec->smmu_identity_ready = true;
	vdec->smmu_identity_needs_reset = true;
	return 0;

fail:
	histb_vdec_release_smmu_identity(vdec);
	return ret;
}

static void histb_vdec_program_smmu(struct histb_vdec_dev *vdec, bool enable)
{
	if (!enable) {
		writel(0, vdec->regs + HISTB_VDEC_SMMU_CTRL);
		return;
	}

	if (WARN_ON(!vdec->smmu_identity_ready)) {
		writel(0, vdec->regs + HISTB_VDEC_SMMU_CTRL);
		return;
	}

	writel(lower_32_bits(vdec->smmu_pt_dma),
	       vdec->regs + HISTB_VDEC_SMMU_PGTABLE_ADDR);
	writel(lower_32_bits(vdec->smmu_err_rd_dma),
	       vdec->regs + HISTB_VDEC_SMMU_ERR_RD_ADDR);
	writel(lower_32_bits(vdec->smmu_err_wr_dma),
	       vdec->regs + HISTB_VDEC_SMMU_ERR_WR_ADDR);
	/* Publish the table and error sinks before enabling translation. */
	wmb();
	writel(8, vdec->regs + HISTB_VDEC_SMMU_CTRL);
}

static int histb_vdec_mpeg4_result_to_errno(int result)
{
	switch (result) {
	case HISTB_MPEG4_OK:
		return 0;
	case HISTB_MPEG4_NEED_MORE:
		return -ENODATA;
	case HISTB_MPEG4_UNSUPPORTED:
		return -EOPNOTSUPP;
	default:
		return -EINVAL;
	}
}

static int histb_vdec_vc1_result_to_errno(int result)
{
	switch (result) {
	case HISTB_VC1_OK:
		return 0;
	case HISTB_VC1_NOT_READY:
		return -ENODATA;
	case HISTB_VC1_UNSUPPORTED:
		return -EOPNOTSUPP;
	case HISTB_VC1_HW_ERROR:
		return -EIO;
	default:
		return -EINVAL;
	}
}

struct histb_vdec_vc1_view_storage {
	struct histb_vc1_parse_view view;
	u8 *shadow;
	u32 *u2r;
};

static int histb_vdec_open_vc1_view(const u8 *raw, u32 raw_size,
				    struct histb_vdec_vc1_view_storage *storage)
{
	int ret;

	if (!raw || !raw_size || !storage)
		return -EINVAL;
	memset(storage, 0, sizeof(*storage));
	storage->shadow = kmalloc(raw_size, GFP_KERNEL);
	storage->u2r = kcalloc((size_t)raw_size + 1,
			       sizeof(*storage->u2r), GFP_KERNEL);
	if (!storage->shadow || !storage->u2r) {
		ret = -ENOMEM;
		goto free_storage;
	}
	ret = histb_vc1_unescape_unit(raw, raw_size, storage->shadow, raw_size,
				      storage->u2r, raw_size + 1, &storage->view);
	if (ret) {
		ret = histb_vdec_vc1_result_to_errno(ret);
		goto free_storage;
	}
	return 0;

free_storage:
	kfree(storage->u2r);
	kfree(storage->shadow);
	memset(storage, 0, sizeof(*storage));
	return ret;
}

static void histb_vdec_close_vc1_view(
		struct histb_vdec_vc1_view_storage *storage)
{
	kfree(storage->u2r);
	kfree(storage->shadow);
	memset(storage, 0, sizeof(*storage));
}

static int histb_vdec_stage_vc1_bpd(
		struct histb_vdec_ctx *ctx,
		const struct histb_vc1_parse_view *view, u32 start_u_bit,
		struct histb_vc1_bpd_regs *regs, u32 *available_bits)
{
	struct histb_vdec_dma_buffer *stream = &ctx->vc1_stream;
	dma_addr_t cursor;
	u32 adjusted_offset;
	int ret;

	if (!view || !view->unescaped || !view->unescaped_size || !regs ||
	    !available_bits || start_u_bit >= view->unescaped_size * 8U)
		return -EINVAL;
	ret = histb_vdec_resize_vc1_stream(ctx, view->unescaped_size);
	if (ret)
		return ret;
	memset(stream->cpu, 0, stream->size);
	memcpy(stream->cpu, view->unescaped, view->unescaped_size);
	cursor = stream->dma + (start_u_bit >> 3);
	if (upper_32_bits(cursor))
		return -ERANGE;
	adjusted_offset = ((cursor & 15) << 3) | (start_u_bit & 7);
	regs->cfg[0] = (regs->cfg[0] & ~0xffU) | adjusted_offset;
	regs->cfg[1] = lower_32_bits(cursor) & ~15U;
	*available_bits = view->unescaped_size * 8U - start_u_bit;
	return 0;
}

static int histb_vdec_parse_vc1_picture_unit(
		struct histb_vdec_ctx *ctx, const struct histb_vc1_unit *unit,
		const struct histb_vc1_sequence *sequence,
		const struct histb_vc1_entry_point *entry,
		const struct histb_vc1_bpd_layout *layout,
		struct histb_vc1_parsed_picture *picture)
{
	struct histb_vdec_vc1_view_storage storage;
	struct histb_vc1_bpd_result bpd_result;
	struct histb_vc1_bpd_regs bpd_regs;
	dma_addr_t payload_dma = ctx->bitstream.dma + unit->payload_offset;
	const u8 *payload = ctx->bitstream.cpu + unit->payload_offset;
	u32 available_bits;
	int ret;

	if (upper_32_bits(payload_dma))
		return -ERANGE;
	ret = histb_vdec_open_vc1_view(payload, unit->payload_size, &storage);
	if (ret)
		return ret;
	ret = histb_vc1_parse_progressive_b_picture_view(
		&storage.view, 0, lower_32_bits(payload_dma), sequence, entry,
		layout, picture);
	if (ret == HISTB_VC1_NEEDS_BPD) {
		bpd_regs = picture->bpd_regs;
		ret = histb_vdec_stage_vc1_bpd(ctx, &storage.view,
						picture->bpd_start_u_bits,
						&bpd_regs, &available_bits);
		if (ret)
			goto close_view;
		ret = histb_vdec_run_vc1_bpd(ctx, &bpd_regs, available_bits,
						      &bpd_result);
		if (ret)
			goto close_view;
		ret = histb_vdec_vc1_result_to_errno(
			histb_vc1_resume_progressive_b_picture_view(
				&storage.view, entry, &bpd_result, picture));
	} else {
		ret = histb_vdec_vc1_result_to_errno(ret);
	}

close_view:
	histb_vdec_close_vc1_view(&storage);
	return ret;
}

static int histb_vdec_parse_vc1_slice_unit(
		struct histb_vdec_ctx *ctx, const struct histb_vc1_unit *unit,
		const struct histb_vc1_sequence *sequence,
		const struct histb_vc1_entry_point *entry,
		const struct histb_vc1_bpd_layout *layout,
		struct histb_vc1_parsed_slice *slice)
{
	struct histb_vdec_vc1_view_storage storage;
	struct histb_vc1_bpd_result bpd_result;
	struct histb_vc1_bpd_regs bpd_regs;
	dma_addr_t payload_dma = ctx->bitstream.dma + unit->payload_offset;
	const u8 *payload = ctx->bitstream.cpu + unit->payload_offset;
	u32 available_bits;
	int ret;

	if (upper_32_bits(payload_dma))
		return -ERANGE;
	ret = histb_vdec_open_vc1_view(payload, unit->payload_size, &storage);
	if (ret)
		return ret;
	ret = histb_vc1_parse_progressive_b_slice_view(
		&storage.view, lower_32_bits(payload_dma), sequence, entry,
		layout, slice);
	if (ret == HISTB_VC1_NEEDS_BPD) {
		bpd_regs = slice->picture.bpd_regs;
		ret = histb_vdec_stage_vc1_bpd(ctx, &storage.view,
						slice->picture.bpd_start_u_bits,
						&bpd_regs, &available_bits);
		if (ret)
			goto close_view;
		ret = histb_vdec_run_vc1_bpd(ctx, &bpd_regs, available_bits,
					      &bpd_result);
		if (ret)
			goto close_view;
		ret = histb_vdec_vc1_result_to_errno(
			histb_vc1_resume_progressive_b_slice_view(
				&storage.view, entry, &bpd_result, slice));
	} else {
		ret = histb_vdec_vc1_result_to_errno(ret);
	}

close_view:
	histb_vdec_close_vc1_view(&storage);
	return ret;
}

static int histb_vdec_parse_vc1_field_picture_unit(
		struct histb_vdec_ctx *ctx, const struct histb_vc1_unit *unit,
		const struct histb_vc1_sequence *sequence,
		const struct histb_vc1_entry_point *entry,
		const struct histb_vc1_bpd_layout *layout, bool second,
		struct histb_vc1_field_transaction *transaction)
{
	struct histb_vdec_vc1_view_storage storage;
	struct histb_vc1_bpd_result bpd_result;
	struct histb_vc1_bpd_regs bpd_regs;
	struct histb_vc1_parsed_picture *picture;
	dma_addr_t payload_dma = ctx->bitstream.dma + unit->payload_offset;
	const u8 *payload = ctx->bitstream.cpu + unit->payload_offset;
	u32 available_bits;
	u8 expected_state;
	int parser_ret, ret;

	if (upper_32_bits(payload_dma))
		return -ERANGE;
	ret = histb_vdec_open_vc1_view(payload, unit->payload_size, &storage);
	if (ret)
		return ret;
	parser_ret = second ?
		histb_vc1_parse_second_field_picture_view(
			&storage.view, 0, lower_32_bits(payload_dma), sequence,
			entry, layout, transaction) :
		histb_vc1_parse_first_field_picture_view(
			&storage.view, 0, lower_32_bits(payload_dma), sequence,
			entry, layout, transaction);
	if (parser_ret == HISTB_VC1_NEEDS_BPD) {
		picture = &transaction->field[second];
		bpd_regs = picture->bpd_regs;
		ret = histb_vdec_stage_vc1_bpd(ctx, &storage.view,
						picture->bpd_start_u_bits,
						&bpd_regs, &available_bits);
		if (ret)
			goto close_view;
		ret = histb_vdec_run_vc1_bpd(ctx, &bpd_regs, available_bits,
					      &bpd_result);
		if (ret)
			goto close_view;
		parser_ret = histb_vc1_resume_field_picture_view(
			&storage.view, entry, &bpd_result, transaction);
	}
	ret = histb_vdec_vc1_result_to_errno(parser_ret);
	if (ret)
		goto close_view;
	expected_state = second ? HISTB_VC1_FIELD_TXN_COMPLETE :
		HISTB_VC1_FIELD_TXN_FIRST_COMPLETE;
	if (transaction->state != expected_state)
		ret = -EINVAL;

close_view:
	histb_vdec_close_vc1_view(&storage);
	return ret;
}

static int histb_vdec_parse_vc1_field_slice_unit(
		struct histb_vdec_ctx *ctx, const struct histb_vc1_unit *unit,
		const struct histb_vc1_entry_point *entry,
		const struct histb_vc1_field_transaction *transaction,
		u8 field_index, struct histb_vc1_parsed_slice *slice)
{
	struct histb_vdec_vc1_view_storage storage;
	dma_addr_t payload_dma = ctx->bitstream.dma + unit->payload_offset;
	const u8 *payload = ctx->bitstream.cpu + unit->payload_offset;
	int parser_ret, ret;

	if (upper_32_bits(payload_dma))
		return -ERANGE;
	ret = histb_vdec_open_vc1_view(payload, unit->payload_size, &storage);
	if (ret)
		return ret;
	parser_ret = histb_vc1_parse_field_slice_view(
		&storage.view, entry, transaction, field_index, slice);
	ret = histb_vdec_vc1_result_to_errno(parser_ret);
	histb_vdec_close_vc1_view(&storage);
	return ret;
}

#define HISTB_VDEC_VC1_AU_DONE	1

static int histb_vdec_resize_vc1_stream(struct histb_vdec_ctx *ctx,
					 size_t required)
{
	struct histb_vdec_dma_buffer *stream = &ctx->vc1_stream;
	struct histb_vdec_dma_buffer replacement = { };
	size_t allocation;

	if (required > HISTB_VDEC_MAX_BITSTREAM ||
	    check_add_overflow(required, (size_t)HISTB_VDEC_RAW_STREAM_GUARD,
			       &required) || required > SIZE_MAX - (SZ_64K - 1))
		return -EOVERFLOW;
	allocation = ALIGN(required, SZ_64K);
	if (stream->size >= allocation)
		return 0;
	replacement.cpu = dma_alloc_coherent(ctx->vdec->dev, allocation,
					     &replacement.dma, GFP_KERNEL);
	if (!replacement.cpu)
		return -ENOMEM;
	if (upper_32_bits(replacement.dma) || !IS_ALIGNED(replacement.dma, 16)) {
		dma_free_coherent(ctx->vdec->dev, allocation, replacement.cpu,
				  replacement.dma);
		return -ERANGE;
	}
	replacement.size = allocation;
	if (stream->cpu)
		dma_free_coherent(ctx->vdec->dev, stream->size, stream->cpu,
				  stream->dma);
	*stream = replacement;
	return 0;
}

static int histb_vdec_stage_vc1_slices(struct histb_vdec_ctx *ctx,
		struct histb_vc1_slice_part *parts, u32 count)
{
	struct histb_vdec_dma_buffer *stream = &ctx->vc1_stream;
	u32 bitstream_dma;
	size_t required = 0, write_offset = 0;
	unsigned int i;
	int ret;

	if (!ctx || !parts || !count || count > HISTB_VC1_MAX_SLICES ||
	    upper_32_bits(ctx->bitstream.dma))
		return -EINVAL;
	bitstream_dma = lower_32_bits(ctx->bitstream.dma);
	for (i = 0; i < count; i++) {
		u32 payload_size;

		if (parts[i].payload_dma < bitstream_dma ||
		    parts[i].payload_raw_bits & 7)
			return -ERANGE;
		payload_size = parts[i].payload_raw_bits >> 3;
		if (!payload_size || check_add_overflow(required,
						      ALIGN(payload_size, 16), &required))
			return -EOVERFLOW;
	}
	ret = histb_vdec_resize_vc1_stream(ctx, required);
	if (ret)
		return ret;
	memset(stream->cpu, 0, stream->size);

	for (i = 0; i < count; i++) {
		struct histb_vdec_vc1_view_storage storage;
		u32 payload_offset = parts[i].payload_dma - bitstream_dma;
		u32 payload_size = parts[i].payload_raw_bits >> 3;

		if (payload_offset > ctx->vc1_au_bytes ||
		    payload_size > ctx->vc1_au_bytes - payload_offset)
			return -ERANGE;
		ret = histb_vdec_open_vc1_view(ctx->bitstream.cpu + payload_offset,
						 payload_size, &storage);
		if (ret)
			return ret;
		if (parts[i].data_u_bit >= storage.view.unescaped_size * 8U ||
		    write_offset > stream->size - storage.view.unescaped_size ||
		    upper_32_bits(stream->dma + write_offset)) {
			histb_vdec_close_vc1_view(&storage);
			return -EINVAL;
		}
		memcpy(stream->cpu + write_offset, storage.view.unescaped,
		       storage.view.unescaped_size);
		parts[i].payload_dma = lower_32_bits(stream->dma + write_offset);
		parts[i].payload_raw_bits = storage.view.unescaped_size * 8U;
		parts[i].data_raw_bit = parts[i].data_u_bit;
		write_offset += ALIGN(storage.view.unescaped_size, 16);
		histb_vdec_close_vc1_view(&storage);
	}

	return 0;
}

static int histb_vdec_apply_vc1_field_intensity(
		const struct histb_vc1_parsed_picture *picture, bool second_field,
		bool have_latest, struct histb_vc1_intensity_map *latest_map,
		struct histb_vc1_intensity_map *current_map)
{
	bool active[HISTB_VC1_INTENSITY_PARITIES] = { };
	u8 scale[HISTB_VC1_INTENSITY_PARITIES] = { 32, 32 };
	u8 shift[HISTB_VC1_INTENSITY_PARITIES] = { };
	unsigned int parity;
	int ret;

	if (!picture || !latest_map || !current_map ||
	    picture->current_parity >= HISTB_VC1_INTENSITY_PARITIES)
		return -EINVAL;
	if (picture->ptype != HISTB_VC1_PICTURE_P ||
	    picture->mv_mode != HISTB_VC1_MV_INTENSITY_COMP)
		return 0;
	switch (picture->intcomp_field) {
	case HISTB_VC1_INTENSITY_BOTH_FIELDS:
		active[0] = true;
		active[1] = true;
		scale[0] = picture->lum_scale;
		shift[0] = picture->lum_shift;
		scale[1] = picture->lum_scale2;
		shift[1] = picture->lum_shift2;
		break;
	case HISTB_VC1_INTENSITY_TOP_FIELD:
		active[0] = true;
		scale[0] = picture->lum_scale;
		shift[0] = picture->lum_shift;
		break;
	case HISTB_VC1_INTENSITY_BOTTOM_FIELD:
		active[1] = true;
		scale[1] = picture->lum_scale;
		shift[1] = picture->lum_shift;
		break;
	default:
		return -EINVAL;
	}

	for (parity = 0; parity < HISTB_VC1_INTENSITY_PARITIES; parity++) {
		struct histb_vc1_intensity_map *target;

		if (!active[parity])
			continue;
		/* The second field can predict from both the preceding full-frame
		 * anchor and the opposite field already reconstructed in current. */
		target = !second_field ||
			(have_latest && parity == picture->current_parity) ?
			latest_map : current_map;
		ret = histb_vc1_intensity_compose_parity(
			target, parity, scale[parity], shift[parity], target);
		if (ret)
			return histb_vdec_vc1_result_to_errno(ret);
	}
	return 0;
}

static int histb_vdec_prepare_vc1_intensity_state(
		struct histb_vdec_ctx *ctx,
		const struct histb_vc1_parsed_picture *picture)
{
	const struct histb_vdec_vc1_anchor *earlier = &ctx->vc1_ref[0];
	const struct histb_vdec_vc1_anchor *latest = &ctx->vc1_ref[1];
	struct histb_vc1_intensity_work *work =
		ctx->buffers[HISTB_VDEC_BUF_VC1_INTENSITY].cpu;
	const struct histb_vc1_intensity_map *transformed = NULL;
	u8 halfpel = 0;
	int ret;

	BUILD_BUG_ON(sizeof(*work) != HISTB_VDEC_VC1_INTENSITY_SIZE);
	ctx->vc1_pending_intensity_valid = false;
	ctx->vc1_pending_halfpel = 0;
	if (!picture || !work ||
	    ctx->buffers[HISTB_VDEC_BUF_VC1_INTENSITY].size < sizeof(*work))
		return -EINVAL;

	ret = histb_vc1_picture_halfpel(picture, latest->halfpel, &halfpel);
	if (ret)
		return histb_vdec_vc1_result_to_errno(ret);
	if (picture->fcm == HISTB_VC1_FIELD_INTERLACED) {
		struct histb_vc1_intensity_map *latest_map =
			&work->ref[0];
		struct histb_vc1_intensity_map *current_map =
			&work->ref[2];
		unsigned int i;

		/* Use the coherent work slot as scratch: field maps are large enough
		 * that keeping two copies on the kernel stack trips frame-size checks. */
		for (i = 0; i < HISTB_VC1_INTENSITY_REFS; i++)
			histb_vc1_intensity_init(&work->ref[i]);
		if (latest->tile.cpu)
			*latest_map = latest->intensity;
		/* The first field's map is recomputed for the second job.  No map is
		 * published to an anchor until the complete field pair commits. */
		if (picture->is_second_field) {
			ret = histb_vdec_apply_vc1_field_intensity(
				&ctx->vc1_field_transaction.field[0], false,
				latest->tile.cpu, latest_map, current_map);
			if (ret)
				return ret;
		}
		ret = histb_vdec_apply_vc1_field_intensity(
			picture, picture->is_second_field, latest->tile.cpu,
			latest_map, current_map);
		if (ret)
			return ret;
		ctx->vc1_pending_earlier_intensity = *latest_map;
		ctx->vc1_pending_intensity_valid = true;
		if (picture->ptype == HISTB_VC1_PICTURE_P) {
			work->ref[0] = latest->tile.cpu ?
				*latest_map : *current_map;
		} else if (picture->ptype == HISTB_VC1_PICTURE_B) {
			if (!earlier->tile.cpu || !latest->tile.cpu)
				return -EINVAL;
			work->ref[0] = earlier->intensity;
			work->ref[1] = *latest_map;
		}
		ctx->vc1_pending_halfpel = halfpel;
		return 0;
	}

	if (picture->ptype == HISTB_VC1_PICTURE_I) {
		if (latest->tile.cpu)
			ctx->vc1_pending_earlier_intensity = latest->intensity;
		else
			histb_vc1_intensity_init(
				&ctx->vc1_pending_earlier_intensity);
		ctx->vc1_pending_intensity_valid = true;
	} else if (picture->ptype == HISTB_VC1_PICTURE_P) {
		if (!latest->tile.cpu)
			return -EINVAL;
		ctx->vc1_pending_earlier_intensity = latest->intensity;
		if (picture->mv_mode == HISTB_VC1_MV_INTENSITY_COMP) {
			ret = histb_vc1_intensity_compose(
				&latest->intensity, picture->lum_scale,
				picture->lum_shift,
				&ctx->vc1_pending_earlier_intensity);
			if (ret)
				return histb_vdec_vc1_result_to_errno(ret);
		}
		transformed = &ctx->vc1_pending_earlier_intensity;
		ctx->vc1_pending_intensity_valid = true;
	}

	ret = histb_vc1_pack_intensity_work(
		picture->ptype,
		earlier->tile.cpu ? &earlier->intensity : NULL,
		latest->tile.cpu ? &latest->intensity : NULL,
		transformed, work);
	if (ret) {
		ctx->vc1_pending_intensity_valid = false;
		return histb_vdec_vc1_result_to_errno(ret);
	}
	ctx->vc1_pending_halfpel = halfpel;
	return 0;
}

static int histb_vdec_prepare_vc1_picture_message(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded)
{
	const struct histb_vc1_parsed_picture *syntax =
		&ctx->vc1_pending_picture;
	const struct histb_vdec_vc1_anchor *earlier = &ctx->vc1_ref[0];
	const struct histb_vdec_vc1_anchor *latest = &ctx->vc1_ref[1];
	struct histb_vc1_picture picture = { };
	struct histb_vc1_pic_msg message;
	__le32 *words;
	dma_addr_t bpd_dma, message_dma;
	u32 forward_ref_dist, total_ref_dist;
	u16 coded_width = ctx->vc1_annex_l ?
		syntax->coded_width : ctx->vc1_entry.coded_width;
	u16 coded_height = ctx->vc1_annex_l ?
		syntax->coded_height : ctx->vc1_entry.coded_height;
	unsigned int i;
	int ret;

	if (!ctx->vc1_pending_valid || !decoded || syntax->skipped)
		return -EINVAL;
	message_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	bpd_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_VC1_BPD);
	if (upper_32_bits(message_dma) || upper_32_bits(bpd_dma) ||
	    upper_32_bits(decoded->tile.dma) || upper_32_bits(decoded->pmv.dma))
		return -ERANGE;
	if (syntax->ptype == HISTB_VC1_PICTURE_P &&
	    upper_32_bits(latest->tile.dma))
		return -ERANGE;
	if (syntax->ptype == HISTB_VC1_PICTURE_B &&
	    (upper_32_bits(earlier->tile.dma) ||
	     upper_32_bits(latest->tile.dma) || upper_32_bits(latest->pmv.dma)))
		return -ERANGE;
	picture.ptype = syntax->ptype;
	picture.profile = ctx->vc1_annex_l ? ctx->vc1_smp_sequence.profile :
		HISTB_VC1_PROFILE_ADVANCED;
	picture.fcm = syntax->fcm;
	picture.loopfilter = ctx->vc1_entry.loopfilter;
	picture.is_second_field = syntax->is_second_field;
	picture.current_parity = syntax->current_parity;
	picture.num_ref = syntax->num_ref;
	picture.rounding_control = syntax->rounding_control;
	picture.fast_uv_mc = ctx->vc1_entry.fast_uv_mc;
	picture.overlap = ctx->vc1_entry.overlap;
	picture.condover = syntax->condover;
	picture.pquant = syntax->pquant;
	picture.pqindex = syntax->pqindex;
	picture.altpquant = syntax->altpquant;
	picture.halfqp = syntax->halfqp;
	picture.b_uniform = syntax->pquantizer;
	picture.use_alt_qp = syntax->use_alt_qp;
	picture.dquant = ctx->vc1_entry.dquant;
	picture.dqprofile = syntax->dqprofile;
	picture.dqbi_level = syntax->dqbi_level;
	picture.dquant_frame = syntax->dquant_frame;
	picture.quant_mode = syntax->quant_mode;
	picture.mv_mode = syntax->mv_mode;
	picture.mv_mode2 = syntax->mv_mode2;
	picture.current_halfpel = ctx->vc1_pending_halfpel;
	if (syntax->ptype == HISTB_VC1_PICTURE_P)
		picture.colocated_halfpel = ctx->vc1_pending_halfpel;
	else if (syntax->ptype == HISTB_VC1_PICTURE_B)
		picture.colocated_halfpel = latest->halfpel;
	picture.mv_range = syntax->mv_range;
	picture.ref_dist = syntax->ref_dist;
	if (syntax->fcm == HISTB_VC1_FIELD_INTERLACED &&
	    syntax->ptype == HISTB_VC1_PICTURE_B) {
		/* Field-B carries BFRACTION but inherits the distance between the
		 * committed past/future anchors from the latest reference pair. */
		total_ref_dist = latest->ref_dist;
		forward_ref_dist = ((u32)syntax->bfraction * total_ref_dist) >> 8;
		picture.ref_dist = total_ref_dist;
		picture.forward_ref_dist = forward_ref_dist;
		picture.backward_ref_dist = total_ref_dist > forward_ref_dist ?
			total_ref_dist - forward_ref_dist - 1 : 0;
	}
	picture.dmv_range = syntax->dmv_range;
	picture.ref_field = syntax->ref_field;
	picture.trans_dc_table = syntax->trans_dc_table;
	picture.variable_transform = ctx->vc1_entry.variable_transform;
	picture.ttmbf = syntax->ttmbf;
	picture.trans_ac_frame = syntax->trans_ac_frame;
	picture.trans_ac_frame2 = syntax->trans_ac_frame2;
	picture.ttfrm = syntax->ttfrm;
	picture.forward_mb_raw = syntax->forward_mb_raw;
	picture.direct_mb_raw = syntax->direct_mb_raw;
	picture.mvtype_mb_raw = syntax->mvtype_mb_raw;
	picture.fieldtx_raw = syntax->fieldtx_raw;
	picture.skip_mb_raw = syntax->skip_mb_raw;
	picture.acpred_raw = syntax->acpred_raw;
	picture.overflags_raw = syntax->overflags_raw;
	picture.mv_table = syntax->mv_table;
	picture.cbp_table = syntax->cbp_table;
	picture.mb_mode_table = syntax->mb_mode_table;
	picture.two_mv_bp_table = syntax->two_mv_bp_table;
	picture.four_mv_bp_table = syntax->four_mv_bp_table;
	picture.four_mv_switch = syntax->four_mv_switch;
	picture.b_fraction = syntax->bfraction_index;
	picture.scale_factor = syntax->bfraction;
	picture.range_map_y_flag = ctx->vc1_entry.range_map_y_flag;
	picture.range_map_y = ctx->vc1_entry.range_map_y;
	picture.range_map_uv_flag = ctx->vc1_entry.range_map_uv_flag;
	picture.range_map_uv = ctx->vc1_entry.range_map_uv;
	if (ctx->vc1_annex_l) {
		picture.range_reduction = syntax->range_reduction;
		if (syntax->ptype == HISTB_VC1_PICTURE_P)
			picture.range_reduction0 = latest->range_reduction;
		else if (syntax->ptype == HISTB_VC1_PICTURE_B) {
			picture.range_reduction0 = earlier->range_reduction;
			picture.range_reduction1 = latest->range_reduction;
		}
		picture.postproc = 0;
		picture.codec_version = 5;
	} else {
		/* CV200 maps Advanced Profile codec version 8 to this HAL code info. */
		picture.postproc = 1;
		picture.codec_version = 6;
	}
	picture.picture_structure = syntax->fcm;
	picture.mb_width = DIV_ROUND_UP(coded_width, 16);
	picture.mb_height = DIV_ROUND_UP(coded_height, 16);
	if (syntax->fcm == HISTB_VC1_FIELD_INTERLACED)
		picture.mb_height = DIV_ROUND_UP(picture.mb_height, 2);
	/* SMP RESPIC changes the coded MB grid, not the full FSP display canvas. */
	picture.display_width = ctx->vc1_annex_l ?
		ctx->vc1_smp_sequence.coded_width : min_t(u16,
		ctx->vc1_sequence.display_width ?: coded_width, coded_width);
	picture.display_height = ctx->vc1_annex_l ?
		ctx->vc1_smp_sequence.coded_height : min_t(u16,
		ctx->vc1_sequence.display_height ?: coded_height, coded_height);
	picture.total_slices = ctx->vc1_slices;
	picture.bpd_mb_width = picture.mb_width;
	picture.current_picture_addr = lower_32_bits(decoded->tile.dma);
	picture.current_colmb_addr = lower_32_bits(decoded->pmv.dma);
	if (syntax->ptype == HISTB_VC1_PICTURE_P) {
		picture.forward_ref_addr = lower_32_bits(
			latest->tile.cpu ? latest->tile.dma : decoded->tile.dma);
		picture.backward_ref_addr = lower_32_bits(decoded->tile.dma);
		picture.backward_colmb_addr = lower_32_bits(decoded->pmv.dma);
		picture.forward_fcm = latest->tile.cpu ? latest->fcm : syntax->fcm;
		picture.backward_fcm = syntax->fcm;
	} else if (syntax->ptype == HISTB_VC1_PICTURE_B) {
		picture.forward_ref_addr = lower_32_bits(earlier->tile.dma);
		picture.backward_ref_addr = lower_32_bits(latest->tile.dma);
		picture.backward_colmb_addr = lower_32_bits(latest->pmv.dma);
		picture.forward_fcm = earlier->fcm;
		picture.backward_fcm = latest->fcm;
	} else {
		picture.forward_ref_addr = lower_32_bits(decoded->tile.dma);
		picture.backward_ref_addr = lower_32_bits(decoded->tile.dma);
		picture.backward_colmb_addr = lower_32_bits(decoded->pmv.dma);
		picture.forward_fcm = syntax->fcm;
		picture.backward_fcm = syntax->fcm;
	}
	picture.sed_top_addr = lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_SED_TOP));
	picture.pmv_top_addr = lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_PMV_TOP));
	picture.itrans_top_addr = lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_RCN_TOP));
	picture.dblk_top_addr = lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_DBLK_TOP));
	picture.intensity_table_addr = lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_VC1_INTENSITY));
	picture.bpd_stride = ((picture.bpd_mb_width + 127) >> 7) << 4;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		picture.bitplane_addr[i] = lower_32_bits(bpd_dma) + i * 2048;
	picture.slice_info_addr = lower_32_bits(message_dma) +
		HISTB_VDEC_VC1_SLICE_MSG_OFFSET;
	/* VC1_CfgVDH packs IMAGE.top_fld_type for current/ref-list1/ref-list0.
	 * CV200 marks field-interlaced images as type 1; progressive and frame-
	 * interlaced images retain type 0. */
	ctx->vc1_pending_ref_pic_type =
		(picture.fcm == HISTB_VC1_FIELD_INTERLACED ? BIT(4) : 0) |
		(picture.forward_fcm == HISTB_VC1_FIELD_INTERLACED ? BIT(2) : 0) |
		(picture.backward_fcm == HISTB_VC1_FIELD_INTERLACED ? BIT(0) : 0);

	ret = histb_vc1_build_pic_msg(&picture, &message);
	if (ret)
		return histb_vdec_vc1_result_to_errno(ret);
	words = ctx->buffers[HISTB_VDEC_BUF_MSG].cpu +
		HISTB_VDEC_VC1_PIC_MSG_OFFSET;
	for (i = 0; i < ARRAY_SIZE(message.d); i++)
		words[i] = cpu_to_le32(message.d[i]);
	return 0;
}

static void histb_vdec_set_vc1_smp_headers(
		struct histb_vdec_ctx *ctx,
		const struct histb_vc1_smp_sequence *smp)
{
	memset(&ctx->vc1_sequence, 0, sizeof(ctx->vc1_sequence));
	ctx->vc1_sequence.max_coded_width = smp->coded_width;
	ctx->vc1_sequence.max_coded_height = smp->coded_height;
	ctx->vc1_sequence.display_width = smp->coded_width;
	ctx->vc1_sequence.display_height = smp->coded_height;

	memset(&ctx->vc1_entry, 0, sizeof(ctx->vc1_entry));
	ctx->vc1_entry.loopfilter = smp->loopfilter;
	ctx->vc1_entry.fast_uv_mc = smp->fast_uv_mc;
	ctx->vc1_entry.extended_mv = smp->extended_mv;
	ctx->vc1_entry.dquant = smp->dquant;
	ctx->vc1_entry.variable_transform = smp->variable_transform;
	ctx->vc1_entry.overlap = smp->overlap;
	ctx->vc1_entry.quantizer_mode = smp->quantizer_mode;
	ctx->vc1_entry.coded_width = smp->coded_width;
	ctx->vc1_entry.coded_height = smp->coded_height;
}

static int histb_vdec_prepare_vc1_smp_syntax(
		struct histb_vdec_ctx *ctx, struct histb_vdec_decoded_buffer *decoded,
		struct vb2_v4l2_buffer *src, unsigned long payload,
		dma_addr_t *src_dma)
{
	struct histb_vc1_bpd_layout layout = { };
	struct histb_vc1_bpd_result bpd_result;
	struct histb_vc1_parsed_picture picture;
	struct histb_vc1_slice_msg *slice_messages;
	struct histb_vc1_slice_part part = { };
	struct histb_vc1_slice slice;
	struct histb_vc1_smp_sequence smp;
	dma_addr_t bpd_dma, message_dma;
	u16 mb_width, mb_height;
	u32 stream_base;
	unsigned int i, row;
	bool bpd_completed = false;
	bool allow_tail_guard = false;
	int parser_ret, ret;

	if (!payload || payload > U32_MAX)
		return -EINVAL;
	if (!ctx->vc1_au_active) {
		ret = histb_vdec_copy_raw_bitstream(ctx, src, payload, src_dma);
		if (ret)
			return ret;
		ctx->vc1_au_src = src;
		ctx->vc1_au_bytes = payload;
		ctx->vc1_au_offset = 0;
		ctx->vc1_au_active = true;
	} else {
		if (ctx->vc1_au_src != src || ctx->vc1_au_bytes != payload)
			return -EINVAL;
		*src_dma = ctx->bitstream.dma;
	}
	if (!histb_vdec_dma_buffers_valid(ctx) || upper_32_bits(*src_dma))
		return -ERANGE;

	if (!ctx->vc1_smp_sequence_valid) {
		if (payload != 4)
			return -EINVAL;
		parser_ret = histb_vc1_parse_smp_sequence(
			ctx->bitstream.cpu, payload, ctx->src.pix.width,
			ctx->src.pix.height, &smp);
		ret = histb_vdec_vc1_result_to_errno(parser_ret);
		if (ret)
			return ret;
		ctx->vc1_smp_sequence = smp;
		ctx->vc1_smp_sequence_valid = true;
		ctx->vc1_smp_res_pic = 0;
		histb_vdec_set_vc1_smp_headers(ctx, &smp);
		ctx->vc1_au_offset = payload;
		ctx->vc1_pending_source_done = true;
		return HISTB_VDEC_VC1_AU_DONE;
	}

	smp = ctx->vc1_smp_sequence;
	bpd_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_VC1_BPD);
	if (upper_32_bits(bpd_dma))
		return -ERANGE;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		layout.plane_addr[i] = lower_32_bits(bpd_dma) + i * 2048;
	parser_ret = histb_vc1_parse_smp_picture(
		ctx->bitstream.cpu, payload, lower_32_bits(*src_dma), &smp,
		ctx->vc1_smp_rounding, ctx->vc1_smp_res_pic, &layout, &picture);
	if (parser_ret == HISTB_VC1_NEEDS_BPD) {
		ret = histb_vdec_run_vc1_bpd(ctx, &picture.bpd_regs,
							      picture.available_raw_bits,
							      &bpd_result);
		if (ret)
			return ret;
		parser_ret = histb_vc1_resume_smp_picture(
			ctx->bitstream.cpu, payload, &smp, &bpd_result, &picture);
		if (parser_ret == HISTB_VC1_OK)
			bpd_completed = true;
	}
	ret = histb_vdec_vc1_result_to_errno(parser_ret);
	if (ret)
		return ret;
	if (!picture.complete)
		return -EOPNOTSUPP;
	if (ctx->vc1_need_intra && picture.ptype != HISTB_VC1_PICTURE_I)
		return -EINVAL;
	if (picture.ptype == HISTB_VC1_PICTURE_P && !ctx->vc1_ref[1].tile.cpu)
		return -EINVAL;
	if (picture.ptype == HISTB_VC1_PICTURE_B &&
	    (!ctx->vc1_ref[0].tile.cpu || !ctx->vc1_ref[1].tile.cpu ||
	     !ctx->vc1_ref[1].pmv.cpu ||
	     (ctx->vc1_annex_l &&
	      (ctx->vc1_ref[0].res_pic != picture.res_pic ||
	       ctx->vc1_ref[1].res_pic != picture.res_pic)) ||
	     ctx->vc1_display_state == HISTB_VDEC_VC1_DISPLAY_NONE))
		return -EINVAL;

	ret = histb_vdec_set_decoded_buffer_sizes(ctx, decoded);
	if (ret)
		return ret;
	histb_vdec_reclaim_decoded_buffers(ctx, decoded);
	ret = histb_vdec_alloc_decoded_buffers(ctx, decoded);
	if (ret)
		return ret;
	if (!histb_vdec_dma_buffers_valid(ctx))
		return -ERANGE;
	mb_width = DIV_ROUND_UP(picture.coded_width, 16);
	mb_height = DIV_ROUND_UP(picture.coded_height, 16);
	part.row = 0;
	part.payload_dma = lower_32_bits(*src_dma);
	part.payload_raw_bits = payload * 8U;
	part.data_raw_bit = picture.header_raw_bits;
	part.data_u_bit = picture.header_u_bits;
	/*
	 * CV200 rejects a zero-length VC-1 DN entry, but a compressed SKIPMB
	 * P-picture may legally end immediately after its picture header.  The
	 * copied stream has a zero-filled guard; expose exactly one guard bit to
	 * the DN builder without changing parser/BPD bounds or accepting a
	 * truncated non-SKIP picture.
	 */
	if (picture.ptype == HISTB_VC1_PICTURE_P && !picture.skipped &&
	    !picture.skip_mb_raw && picture.fcm == HISTB_VC1_PROGRESSIVE &&
	    bpd_completed &&
	    picture.bpd_mode[HISTB_VC1_BPD_SKIPMB] ==
		HISTB_VDEC_VC1_BPD_MODE_ROWSKIP &&
	    picture.bpd_start_raw_bits == picture.bpd_start_u_bits &&
	    bpd_result.eaten_bits == mb_height + 4 &&
	    picture.bpd_start_raw_bits + mb_height + 4 <= payload * 8U) {
		u32 bit = picture.bpd_start_raw_bits;

		/* ROWSKIP's first bit is INVERT; all row flags must be zero. */
		allow_tail_guard = !!(((u8 *)ctx->bitstream.cpu)[bit >> 3] &
			BIT(7 - (bit & 7)));
		for (row = 0; allow_tail_guard && row < mb_height; row++) {
			bit = picture.bpd_start_raw_bits + 4 + row;
			if (((u8 *)ctx->bitstream.cpu)[bit >> 3] &
			    BIT(7 - (bit & 7)))
				allow_tail_guard = false;
		}
	}
	if (allow_tail_guard && picture.header_raw_bits == payload * 8U &&
	    picture.header_u_bits == payload * 8U)
		part.payload_raw_bits++;
	parser_ret = histb_vc1_finalize_slices(&part, 1, mb_width, mb_height,
						 &slice, 1);
	if (parser_ret)
		return histb_vdec_vc1_result_to_errno(parser_ret);
	message_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG) +
		HISTB_VDEC_VC1_SLICE_MSG_OFFSET;
	if (upper_32_bits(message_dma))
		return -ERANGE;
	slice_messages = ctx->buffers[HISTB_VDEC_BUF_MSG].cpu +
		HISTB_VDEC_VC1_SLICE_MSG_OFFSET;
	parser_ret = histb_vc1_build_slice_messages(
		&slice, 1, mb_width, mb_height, lower_32_bits(message_dma),
		slice_messages, 1, &stream_base);
	if (parser_ret)
		return histb_vdec_vc1_result_to_errno(parser_ret);
	histb_vdec_set_vc1_smp_headers(ctx, &smp);
	ctx->vc1_pending_picture = picture;
	ctx->vc1_pending_output = decoded;
	ctx->vc1_pending_stream_base = stream_base;
	ctx->vc1_slices = 1;
	ctx->vc1_pending_next_offset = payload;
	ctx->vc1_pending_source_done = true;
	ctx->vc1_pending_valid = true;
	ret = histb_vdec_prepare_vc1_intensity_state(ctx, &picture);
	if (ret) {
		histb_vdec_discard_vc1_current_locked(ctx, false);
		return ret;
	}
	ret = histb_vdec_prepare_vc1_picture_message(ctx, decoded);
	if (ret)
		histb_vdec_discard_vc1_current_locked(ctx, false);
	return ret;
}

static int histb_vdec_prepare_vc1_syntax(
		struct histb_vdec_ctx *ctx, struct histb_vdec_decoded_buffer *decoded,
		struct vb2_v4l2_buffer *src, unsigned long payload,
		dma_addr_t *src_dma)
{
	struct histb_vc1_sequence sequence = ctx->vc1_sequence;
	struct histb_vc1_entry_point entry = ctx->vc1_entry;
	struct histb_vc1_entry_point parsed_entry;
	struct histb_vc1_field_transaction field_transaction =
		ctx->vc1_field_transaction;
	struct histb_vc1_parsed_picture picture;
	struct histb_vc1_bpd_layout layout = { };
	struct histb_vc1_slice_part *parts = NULL;
	struct histb_vc1_slice *slices = NULL;
	struct histb_vc1_slice_msg *slice_messages;
	struct histb_vc1_unit unit;
	struct histb_vdec_vc1_view_storage storage;
	dma_addr_t bpd_dma, message_dma;
	u32 cursor, next_offset, part_count = 0, stream_base;
	u16 mb_width, mb_height;
	unsigned int i;
	bool field_current_ref, field_picture = false, second_field, source_done;
	bool entry_seen;
	u8 field_index = 0;
	int parser_ret, ret;

	if (!payload || payload > U32_MAX)
		return -EINVAL;
	if (!ctx->vc1_au_active) {
		ret = histb_vdec_copy_raw_bitstream(ctx, src, payload, src_dma);
		if (ret)
			return ret;
		ctx->vc1_au_src = src;
		ctx->vc1_au_bytes = payload;
		ctx->vc1_au_offset = 0;
		ctx->vc1_au_active = true;
	} else {
		if (ctx->vc1_au_src != src || ctx->vc1_au_bytes != payload)
			return -EINVAL;
		*src_dma = ctx->bitstream.dma;
	}
	if (!histb_vdec_dma_buffers_valid(ctx) || upper_32_bits(*src_dma))
		return -ERANGE;
	bpd_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_VC1_BPD);
	if (upper_32_bits(bpd_dma))
		return -ERANGE;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		layout.plane_addr[i] = lower_32_bits(bpd_dma) + i * 2048;

	cursor = ctx->vc1_au_offset;
	/* Entry-point inheritance is scoped to one sequence header.  When an
	 * OUTPUT starts with a continuation, the committed entry remains the
	 * previous sequence's baseline and must still participate in validation.
	 */
	entry_seen = !!entry.coded_width;
	second_field = field_transaction.state ==
		HISTB_VC1_FIELD_TXN_FIRST_COMPLETE;
	for (;;) {
		parser_ret = histb_vc1_next_unit(ctx->bitstream.cpu,
						 ctx->vc1_au_bytes, &cursor, &unit);
		if (parser_ret)
			return histb_vdec_vc1_result_to_errno(parser_ret);
		if (second_field && unit.type != HISTB_VC1_CODE_FIELD)
			return -EINVAL;
		switch (unit.type) {
		case HISTB_VC1_CODE_SEQUENCE:
			/* A new sequence starts a new entry-point inheritance domain. */
			memset(&entry, 0, sizeof(entry));
			entry_seen = false;
			ret = histb_vdec_open_vc1_view(
				ctx->bitstream.cpu + unit.payload_offset,
				unit.payload_size, &storage);
			if (ret)
				return ret;
			parser_ret = histb_vc1_parse_sequence_view(&storage.view,
								     &sequence);
			histb_vdec_close_vc1_view(&storage);
			if (parser_ret)
				return histb_vdec_vc1_result_to_errno(parser_ret);
			break;
		case HISTB_VC1_CODE_ENTRY_POINT:
			if (!sequence.max_coded_width)
				return -EINVAL;
			ret = histb_vdec_open_vc1_view(
				ctx->bitstream.cpu + unit.payload_offset,
				unit.payload_size, &storage);
			if (ret)
				return ret;
			parser_ret = histb_vc1_parse_entry_point_view(
				&storage.view, &sequence, &parsed_entry);
			histb_vdec_close_vc1_view(&storage);
			if (parser_ret)
				return histb_vdec_vc1_result_to_errno(parser_ret);
			if (entry_seen &&
			    histb_vc1_validate_entry_range_transition(
				    &entry, &parsed_entry) != HISTB_VC1_OK)
				return -EINVAL;
			entry = parsed_entry;
			entry_seen = true;
			break;
		case HISTB_VC1_CODE_FRAME:
			goto frame_found;
		case HISTB_VC1_CODE_END_OF_SEQUENCE:
			ctx->vc1_sequence = sequence;
			ctx->vc1_entry = entry;
			ctx->vc1_au_offset = ctx->vc1_au_bytes;
			ctx->vc1_pending_source_done = true;
			return HISTB_VDEC_VC1_AU_DONE;
		case 0x1b ... 0x1f:
			break;
		case HISTB_VC1_CODE_FIELD:
			if (!second_field)
				return -EOPNOTSUPP;
			goto field_found;
		default:
			return -EINVAL;
		}
	}

frame_found:
	if (!entry.coded_width ||
	    ALIGN(entry.coded_width, 16) != ctx->src.pix.width ||
	    ALIGN(entry.coded_height, 16) != ctx->src.pix.height)
		return -EINVAL;
	if (sequence.interlace) {
		histb_vc1_field_transaction_reset(&field_transaction);
		ret = histb_vdec_parse_vc1_field_picture_unit(
			ctx, &unit, &sequence, &entry, &layout, false,
			&field_transaction);
		if (!ret) {
			field_picture = true;
			picture = field_transaction.field[0];
		} else if (ret != -EOPNOTSUPP) {
			return ret;
		}
	}
	if (!field_picture)
		ret = histb_vdec_parse_vc1_picture_unit(
			ctx, &unit, &sequence, &entry, &layout, &picture);
	if (ret)
		return ret;
	goto picture_found;

field_found:
	if (!entry.coded_width ||
	    ALIGN(entry.coded_width, 16) != ctx->src.pix.width ||
	    ALIGN(entry.coded_height, 16) != ctx->src.pix.height)
		return -EINVAL;
	field_index = 1;
	ret = histb_vdec_parse_vc1_field_picture_unit(
		ctx, &unit, &sequence, &entry, &layout, true,
		&field_transaction);
	if (ret)
		return ret;
	field_picture = true;
	picture = field_transaction.field[1];

picture_found:
	/* Intensity compensation is represented by the parsed map transaction
	 * and D26/D28 message fields prepared below.  Let the hardware acceptance
	 * path decide validity instead of rejecting the public syntax here.
	 */
	/* An IP pair is a valid random-access picture.  Its second P field can
	 * select the already decoded first I field even when NUMREF is set; the
	 * current surface is also the hardware's safe dummy for the other slot.
	 */
	field_current_ref = field_picture && field_index &&
		field_transaction.first_ptype == HISTB_VC1_PICTURE_I;
	if (ctx->vc1_need_intra && picture.ptype != HISTB_VC1_PICTURE_I &&
	    !field_current_ref)
		return -EINVAL;
	if (picture.ptype == HISTB_VC1_PICTURE_P &&
	    !ctx->vc1_ref[1].tile.cpu && !field_current_ref)
		return -EINVAL;
	if (picture.ptype == HISTB_VC1_PICTURE_B &&
	    (!ctx->vc1_ref[0].tile.cpu || !ctx->vc1_ref[1].tile.cpu ||
	     !ctx->vc1_ref[1].pmv.cpu ||
	     ctx->vc1_display_state == HISTB_VDEC_VC1_DISPLAY_NONE))
		return -EINVAL;
	ret = histb_vdec_set_decoded_buffer_sizes(ctx, decoded);
	if (ret)
		return ret;
	histb_vdec_reclaim_decoded_buffers(ctx, decoded);
	ret = histb_vdec_alloc_decoded_buffers(ctx, decoded);
	if (ret)
		return ret;
	if (!histb_vdec_dma_buffers_valid(ctx))
		return -ERANGE;

	next_offset = cursor;
	if (!picture.skipped) {
		parts = kcalloc(HISTB_VC1_MAX_SLICES, sizeof(*parts), GFP_KERNEL);
		slices = kcalloc(HISTB_VC1_MAX_SLICES, sizeof(*slices), GFP_KERNEL);
		if (!parts || !slices) {
			ret = -ENOMEM;
			goto free_slices;
		}
		parts[0].row = 0;
		parts[0].payload_dma = lower_32_bits(ctx->bitstream.dma) +
			unit.payload_offset;
		parts[0].payload_raw_bits = unit.payload_size * 8U;
		parts[0].data_raw_bit = picture.header_raw_bits;
		parts[0].data_u_bit = picture.header_u_bits;
		part_count = 1;
	}
	while (next_offset < ctx->vc1_au_bytes) {
		struct histb_vc1_parsed_slice parsed_slice;
		u32 scan = next_offset;

		parser_ret = histb_vc1_next_unit(ctx->bitstream.cpu,
						 ctx->vc1_au_bytes, &scan, &unit);
		if (parser_ret)
			break;
		if (unit.type != HISTB_VC1_CODE_SLICE) {
			if (unit.type == HISTB_VC1_CODE_END_OF_SEQUENCE)
				next_offset = ctx->vc1_au_bytes;
			break;
		}
		if (picture.skipped || part_count >= HISTB_VC1_MAX_SLICES) {
			ret = -EINVAL;
			goto free_slices;
		}
		if (field_picture)
			ret = histb_vdec_parse_vc1_field_slice_unit(
				ctx, &unit, &entry, &field_transaction,
				field_index, &parsed_slice);
		else
			ret = histb_vdec_parse_vc1_slice_unit(
				ctx, &unit, &sequence, &entry, &layout,
				&parsed_slice);
		if (ret)
			goto free_slices;
		parts[part_count].row = parsed_slice.address;
		parts[part_count].payload_dma = lower_32_bits(ctx->bitstream.dma) +
			unit.payload_offset;
		parts[part_count].payload_raw_bits = unit.payload_size * 8U;
		parts[part_count].data_raw_bit = parsed_slice.header_raw_bits;
		parts[part_count].data_u_bit = parsed_slice.header_u_bits;
		part_count++;
		next_offset = scan;
	}
	source_done = next_offset >= ctx->vc1_au_bytes;
	mb_width = DIV_ROUND_UP(entry.coded_width, 16);
	mb_height = DIV_ROUND_UP(entry.coded_height, 16);
	if (field_picture)
		mb_height = DIV_ROUND_UP(mb_height, 2);
	if (!picture.skipped) {
		ret = histb_vdec_stage_vc1_slices(ctx, parts, part_count);
		if (ret)
			goto free_slices;
		ret = histb_vc1_finalize_slices(parts, part_count, mb_width, mb_height,
						 slices, HISTB_VC1_MAX_SLICES);
		if (ret) {
			ret = histb_vdec_vc1_result_to_errno(ret);
			goto free_slices;
		}
		message_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG) +
			HISTB_VDEC_VC1_SLICE_MSG_OFFSET;
		if (upper_32_bits(message_dma)) {
			ret = -ERANGE;
			goto free_slices;
		}
		slice_messages = ctx->buffers[HISTB_VDEC_BUF_MSG].cpu +
			HISTB_VDEC_VC1_SLICE_MSG_OFFSET;
		ret = histb_vc1_build_slice_messages(
			slices, part_count, mb_width, mb_height,
			lower_32_bits(message_dma), slice_messages,
			HISTB_VC1_MAX_SLICES, &stream_base);
		if (ret) {
			ret = histb_vdec_vc1_result_to_errno(ret);
			goto free_slices;
		}
	}

	ctx->vc1_sequence = sequence;
	ctx->vc1_entry = entry;
	if (field_picture)
		ctx->vc1_field_transaction = field_transaction;
	ctx->vc1_pending_picture = picture;
	ctx->vc1_pending_output = decoded;
	ctx->vc1_pending_stream_base = picture.skipped ? 0 : stream_base;
	ctx->vc1_slices = part_count;
	ctx->vc1_pending_next_offset = next_offset;
	ctx->vc1_pending_source_done = source_done;
	ctx->vc1_pending_valid = true;
	ret = histb_vdec_prepare_vc1_intensity_state(ctx, &picture);
	if (ret) {
		histb_vdec_discard_vc1_current_locked(ctx, false);
		goto free_slices;
	}
	if (!picture.skipped) {
		ret = histb_vdec_prepare_vc1_picture_message(ctx, decoded);
		if (ret) {
			histb_vdec_discard_vc1_current_locked(ctx, false);
			goto free_slices;
		}
	}
	ret = 0;

free_slices:
	kfree(slices);
	kfree(parts);
	return ret;
}

#define HISTB_VDEC_MPEG4_AU_DONE	1

static void histb_vdec_begin_mpeg4_implicit_drain_locked(
		struct histb_vdec_ctx *ctx, struct vb2_v4l2_buffer *src)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;

	lockdep_assert_held(&ctx->mpeg4_lock);
	if (m2m_ctx->is_draining || m2m_ctx->has_stopped)
		return;
	m2m_ctx->last_src_buf = src;
	m2m_ctx->next_buf_last = false;
	m2m_ctx->is_draining = true;
}

static int histb_vdec_prepare_mpeg4(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		struct vb2_v4l2_buffer *src, unsigned long payload,
		dma_addr_t *src_dma)
{
	struct histb_mpeg4_parser parser = ctx->mpeg4_parser;
	struct histb_mpeg4_hw_picture picture = { };
	struct histb_mpeg4_pic_msg pic_msg;
	struct histb_mpeg4_slice_msg *slice_msgs;
	struct histb_mpeg4_slice *slices;
	struct histb_mpeg4_frame frame;
	struct histb_vdec_mpeg4_anchor *earlier = &ctx->mpeg4_ref[0];
	struct histb_vdec_mpeg4_anchor *latest = &ctx->mpeg4_ref[1];
	__le32 *pic;
	__le32 *slice_words;
	dma_addr_t msg_dma;
	u32 stride;
	u32 message_count;
	u32 next_offset;
	u32 slice_count;
	unsigned int i, j;
	int ret;

	if (payload > U32_MAX)
		return -EOVERFLOW;
	if (!ctx->mpeg4_au_active) {
		ret = histb_vdec_copy_raw_bitstream(ctx, src, payload, src_dma);
		if (ret)
			return ret;
		ctx->mpeg4_au_src = src;
		ctx->mpeg4_au_bytes = payload;
		ctx->mpeg4_au_offset = 0;
		ctx->mpeg4_au_active = true;
	} else if (ctx->mpeg4_au_src != src) {
		/*
		 * A stateful client feeds one access unit per queue job and
		 * never re-submits a buffer, so a new source means a new
		 * unit: take a fresh copy and parse it from the start.
		 * Re-submitting the same buffer keeps the original meaning
		 * of continuing the unit already in ctx->bitstream, which is
		 * what the stateless-style client does.
		 */
		ret = histb_vdec_copy_raw_bitstream(ctx, src, payload, src_dma);
		if (ret)
			return ret;
		ctx->mpeg4_au_src = src;
		ctx->mpeg4_au_bytes = payload;
		ctx->mpeg4_au_offset = 0;
		/*
		 * The timing state is the one part of the parser that must
		 * not survive into a new access unit.  time_base accumulates
		 * monotonically and the B-frame reorder check derives
		 * to_future = last_non_b_time - time from it, so a unit
		 * parsed on top of the previous unit's timing makes every
		 * timestamp look inconsistent and every VOP gets skipped:
		 * the client sees a clean decode that produces no frames.
		 * Keep the VOL - the client does not repeat it in every
		 * unit - and clear only the timing.
		 */
		ctx->mpeg4_parser.last_non_b_time = 0;
		ctx->mpeg4_parser.last_time_base = 0;
		ctx->mpeg4_parser.time_base = 0;
		ctx->mpeg4_parser.time_pp = 0;
		ctx->mpeg4_parser.have_non_b_time = 0;
	} else {
		*src_dma = ctx->bitstream.dma;
	}

	for (;;) {
		parser = ctx->mpeg4_parser;
		ret = histb_mpeg4_parse_frame_at(&parser, ctx->bitstream.cpu,
						 ctx->mpeg4_au_bytes,
						 ctx->mpeg4_au_offset, &frame,
						 &next_offset);
		if (ret == HISTB_MPEG4_NO_VOP) {
			ctx->mpeg4_parser = parser;
			ctx->mpeg4_au_offset = ctx->mpeg4_au_bytes;
			ctx->mpeg4_pending_source_done = true;
			return HISTB_VDEC_MPEG4_AU_DONE;
		}
		if (ret == HISTB_MPEG4_END_OF_STREAM) {
			ctx->mpeg4_parser = parser;
			ctx->mpeg4_au_offset = next_offset;
			ctx->mpeg4_pending_source_done = true;
			histb_vdec_begin_mpeg4_implicit_drain_locked(ctx, src);
			return HISTB_VDEC_MPEG4_AU_DONE;
		}
		if (ret) {
			dev_err_ratelimited(ctx->vdec->dev,
					    "MPEG-4 parse failed: %d (off %u/%u)\n",
					    ret, ctx->mpeg4_au_offset,
					    ctx->mpeg4_au_bytes);
			return histb_vdec_mpeg4_result_to_errno(ret);
		}
		if (next_offset <= ctx->mpeg4_au_offset ||
		    next_offset > ctx->mpeg4_au_bytes) {
			dev_err_ratelimited(ctx->vdec->dev,
					    "MPEG-4 offset stall: next %u from %u (len %u)\n",
					    next_offset, ctx->mpeg4_au_offset,
					    ctx->mpeg4_au_bytes);
			return -EINVAL;
		}
		if (frame.vop.coded &&
		    frame.vop.coding_type != HISTB_MPEG4_N_VOP)
			break;
		ctx->mpeg4_parser = parser;
		ctx->mpeg4_au_offset = next_offset;
	}
	ret = histb_mpeg4_validate_stateful_frame(&parser,
						  ctx->bitstream.cpu,
						  ctx->mpeg4_au_bytes, &frame);
	if (ret) {
		dev_err_ratelimited(ctx->vdec->dev,
				    "MPEG-4 feature blocked: %u (parse result %d)\n",
				    parser.blocker, ret);
		return histb_vdec_mpeg4_result_to_errno(ret);
	}
	if (ALIGN(frame.vol.width, 16) != ALIGN(ctx->src.pix.width, 16) ||
	    ALIGN(frame.vol.height, 16) != ALIGN(ctx->src.pix.height, 16)) {
		dev_err_ratelimited(ctx->vdec->dev,
				    "MPEG-4 size mismatch: vol %ux%u vs fmt %ux%u\n",
				    frame.vol.width, frame.vol.height,
				    ctx->src.pix.width, ctx->src.pix.height);
		return -EINVAL;
	}
	if ((frame.vop.coding_type == HISTB_MPEG4_P_VOP ||
	     frame.vop.coding_type == HISTB_MPEG4_S_VOP) && !latest->tile.cpu) {
		dev_err_ratelimited(ctx->vdec->dev,
				    "MPEG-4 %u-VOP with no reference\n",
				    frame.vop.coding_type);
		return -EINVAL;
	}
	if (frame.vop.coding_type == HISTB_MPEG4_B_VOP &&
	    (!earlier->tile.cpu || !latest->tile.cpu || !latest->pmv.cpu)) {
		dev_err_ratelimited(ctx->vdec->dev,
				    "MPEG-4 B-VOP refs: earlier %d latest %d pmv %d\n",
				    !!earlier->tile.cpu, !!latest->tile.cpu,
				    !!latest->pmv.cpu);
		return -EINVAL;
	}

	ret = histb_vdec_set_decoded_buffer_sizes(ctx, decoded);
	if (ret)
		return ret;
	histb_vdec_reclaim_decoded_buffers(ctx, decoded);
	ret = histb_vdec_alloc_decoded_buffers(ctx, decoded);
	if (ret)
		return ret;
	if (!histb_vdec_dma_buffers_valid(ctx)) {
		dev_err_ratelimited(ctx->vdec->dev,
				    "MPEG-4 DMA buffers invalid for %ux%u\n",
				    frame.vol.width, frame.vol.height);
		return -EINVAL;
	}

	msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	stride = ALIGN(frame.vol.width, 256);
	picture.frame = frame;
	picture.backward_pmv_addr =
		lower_32_bits(frame.vop.coding_type == HISTB_MPEG4_B_VOP ?
			      latest->pmv.dma : decoded->pmv.dma);
	picture.backward_ref_addr =
		lower_32_bits(frame.vop.coding_type == HISTB_MPEG4_B_VOP ?
			      latest->tile.dma : decoded->tile.dma);
	picture.current_picture_addr = lower_32_bits(decoded->tile.dma);
	picture.current_pmv_addr = lower_32_bits(decoded->pmv.dma);
	picture.avm_addr = lower_32_bits(msg_dma +
					 HISTB_VDEC_MPEG4_PIC_MSG_OFFSET);
	picture.dnr_mbinfo_addr = lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_MPEG4_DNR_MBINFO));
	picture.display_picture_addr = lower_32_bits(decoded->tile.dma);
	picture.forward_ref_addr = lower_32_bits(
		frame.vop.coding_type == HISTB_MPEG4_B_VOP ?
		earlier->tile.dma :
		(frame.vop.coding_type == HISTB_MPEG4_P_VOP ||
		 frame.vop.coding_type == HISTB_MPEG4_S_VOP) ?
		latest->tile.dma : decoded->tile.dma);
	picture.itrans_top_addr = lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_MPEG4_ITRANS));
	picture.pmv_top_addr = lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_MPEG4_PMV_TOP));
	picture.sed_top_addr = lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_MPEG4_SED_TOP));
	picture.slice_msg_addr = lower_32_bits(msg_dma +
					       HISTB_VDEC_MPEG4_SLICE_MSG_OFFSET);
	picture.vam_addr = lower_32_bits(msg_dma);
	picture.y_stride = stride * 16;
	picture.uv_offset = stride * ALIGN(frame.vol.height, 32);

	slices = kcalloc(HISTB_MPEG4_MAX_SLICES, sizeof(*slices), GFP_KERNEL);
	slice_msgs = kcalloc(HISTB_MPEG4_MAX_SLICE_MSGS, sizeof(*slice_msgs),
			     GFP_KERNEL);
	if (!slices || !slice_msgs) {
		ret = -ENOMEM;
		goto free_slices;
	}
	ret = histb_mpeg4_parse_slices(&frame, ctx->bitstream.cpu,
					ctx->mpeg4_au_bytes,
					lower_32_bits(*src_dma), slices,
					HISTB_MPEG4_MAX_SLICES, &slice_count);
	if (ret) {
		ret = histb_vdec_mpeg4_result_to_errno(ret);
		goto free_slices;
	}
	/* CV200's report/repair table has 200 entries; fail before launch. */
	if (slice_count > HISTB_VDEC_MAX_UP_REPORTS) {
		ret = -E2BIG;
		goto free_slices;
	}
	ret = histb_mpeg4_build_pic_msg(&picture, &pic_msg);
	if (ret) {
		ret = histb_vdec_mpeg4_result_to_errno(ret);
		goto free_slices;
	}
	ret = histb_mpeg4_build_regs(&picture, slices, slice_count,
					     &ctx->mpeg4_pending_regs);
	if (ret) {
		ret = histb_vdec_mpeg4_result_to_errno(ret);
		goto free_slices;
	}
	ret = histb_mpeg4_build_slice_messages(
		&frame, slices, slice_count, ctx->mpeg4_pending_regs.stream_base,
		lower_32_bits(msg_dma + HISTB_VDEC_MPEG4_SLICE_MSG_OFFSET),
		slice_msgs, HISTB_MPEG4_MAX_SLICE_MSGS, &message_count);
	if (ret) {
		ret = histb_vdec_mpeg4_result_to_errno(ret);
		goto free_slices;
	}

	memset(ctx->buffers[HISTB_VDEC_BUF_MSG].cpu, 0,
	       ctx->buffers[HISTB_VDEC_BUF_MSG].size);
	pic = ctx->buffers[HISTB_VDEC_BUF_MSG].cpu +
	      HISTB_VDEC_MPEG4_PIC_MSG_OFFSET;
	slice_words = ctx->buffers[HISTB_VDEC_BUF_MSG].cpu +
		      HISTB_VDEC_MPEG4_SLICE_MSG_OFFSET;
	for (i = 0; i < ARRAY_SIZE(pic_msg.d); i++)
		pic[i] = cpu_to_le32(pic_msg.d[i]);
	for (i = 0; i < message_count; i++)
		for (j = 0; j < ARRAY_SIZE(slice_msgs[i].d); j++)
			slice_words[i * HISTB_MPEG4_SLICE_MSG_WORDS + j] =
				cpu_to_le32(slice_msgs[i].d[j]);

	ctx->mpeg4_pending_parser = parser;
	ctx->mpeg4_pending_frame = frame;
	ctx->mpeg4_pending_next_offset = next_offset;
	ctx->mpeg4_pending_source_done = false;
	ctx->mpeg4_pending_output = decoded;
	ctx->mpeg4_pending_valid = true;
	ctx->mpeg4_slices = slice_count;
	ret = 0;

free_slices:
	kfree(slice_msgs);
	kfree(slices);
	return ret;
}

static bool
histb_vdec_decoded_buffer_valid(struct histb_vdec_decoded_buffer *decoded)
{
	return decoded->tile.cpu && (!decoded->pmv.size || decoded->pmv.cpu) &&
	       !upper_32_bits(decoded->tile.dma) &&
	       (!decoded->pmv.size || !upper_32_bits(decoded->pmv.dma));
}

static const void *histb_vdec_ctrl_data(struct histb_vdec_ctx *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->ctrl_handler, id);

	return ctrl ? ctrl->p_cur.p : NULL;
}

static int
histb_vdec_validate_avs(struct histb_vdec_ctx *ctx,
			const struct v4l2_ctrl_avs_sequence *sequence,
			const struct v4l2_ctrl_avs_picture *picture,
			const struct v4l2_ctrl_avs_slice_params *slices,
			unsigned int num_slices,
			const struct v4l2_ctrl_avs_decode_params *decode,
			const u8 *payload_data, unsigned long payload)
{
	static const u32 picture_flags =
		V4L2_AVS_PICTURE_FLAG_PROGRESSIVE_FRAME |
		V4L2_AVS_PICTURE_FLAG_TOP_FIELD_FIRST |
		V4L2_AVS_PICTURE_FLAG_REPEAT_FIRST_FIELD |
		V4L2_AVS_PICTURE_FLAG_FIXED_QP |
		V4L2_AVS_PICTURE_FLAG_SKIP_MODE |
		V4L2_AVS_PICTURE_FLAG_LOOP_FILTER_DISABLE |
		V4L2_AVS_PICTURE_FLAG_LOOP_FILTER_PARAMS |
		V4L2_AVS_PICTURE_FLAG_REFERENCE |
		V4L2_AVS_PICTURE_FLAG_NO_FORWARD_REFERENCE |
		V4L2_AVS_PICTURE_FLAG_ADVANCED_PRED_DISABLE |
		V4L2_AVS_PICTURE_FLAG_WEIGHTING_QUANT |
		V4L2_AVS_PICTURE_FLAG_CHROMA_QP_DISABLE |
		V4L2_AVS_PICTURE_FLAG_AEC |
		V4L2_AVS_PICTURE_FLAG_P_FIELD_ENHANCED |
		V4L2_AVS_PICTURE_FLAG_B_FIELD_ENHANCED;
	u32 width_mbs, height_mbs, total_mbs;
	unsigned int i;

	if (!sequence || !picture || !slices || !decode || !payload_data ||
	    payload < 4 || payload > HISTB_VDEC_MAX_BITSTREAM ||
	    (u64)payload * 8 > GENMASK(24, 0) ||
	    !num_slices || num_slices > HISTB_VDEC_AVS_MAX_SLICES ||
	    memchr_inv(sequence->reserved, 0, sizeof(sequence->reserved)) ||
	    memchr_inv(picture->reserved, 0, sizeof(picture->reserved)) ||
	    decode->reserved ||
	    (sequence->profile_id != HISTB_VDEC_AVS_PROFILE_JIZHUN &&
	     sequence->profile_id != HISTB_VDEC_AVS_PROFILE_GUANGDIAN) ||
	    sequence->horizontal_size != ctx->src.pix.width ||
	    sequence->vertical_size != ctx->src.pix.height ||
	    sequence->horizontal_size < HISTB_VDEC_MIN_WIDTH ||
	    sequence->vertical_size < HISTB_VDEC_MIN_HEIGHT ||
	    sequence->horizontal_size > HISTB_VDEC_AVS_MAX_WIDTH ||
	    sequence->vertical_size > HISTB_VDEC_AVS_MAX_HEIGHT ||
	    sequence->chroma_format != 1 || sequence->sample_precision != 1 ||
	    (sequence->flags & ~(V4L2_AVS_SEQUENCE_FLAG_PROGRESSIVE |
				  V4L2_AVS_SEQUENCE_FLAG_LOW_DELAY)) ||
	    (picture->flags & ~picture_flags) ||
	    picture->picture_coding_type > V4L2_AVS_PICTURE_TYPE_B ||
	    picture->picture_structure > V4L2_AVS_PICTURE_STRUCTURE_FRAME ||
	    picture->picture_qp > 63 ||
	    picture->alpha_c_offset < -16 || picture->alpha_c_offset > 15 ||
	    picture->beta_offset < -16 || picture->beta_offset > 15 ||
	    (decode->flags &
	     ~((u32)V4L2_AVS_DECODE_PARAM_FLAG_BACKWARD_REF |
	       V4L2_AVS_DECODE_PARAM_FLAG_FORWARD_REF0 |
	       V4L2_AVS_DECODE_PARAM_FLAG_FORWARD_REF1)))
		return -EINVAL;

	/* A progressive sequence cannot switch to interlaced picture syntax. */
	if ((sequence->flags & V4L2_AVS_SEQUENCE_FLAG_PROGRESSIVE) &&
	    (picture->picture_structure != V4L2_AVS_PICTURE_STRUCTURE_FRAME ||
	     !(picture->flags & V4L2_AVS_PICTURE_FLAG_PROGRESSIVE_FRAME)))
		return -EINVAL;
	if (picture->picture_structure == V4L2_AVS_PICTURE_STRUCTURE_FIELD &&
	    (picture->flags & V4L2_AVS_PICTURE_FLAG_PROGRESSIVE_FRAME))
		return -EINVAL;

	/*
	 * Field-enhanced syntax is the Guangdian AVS+ extension and is meaningful
	 * only for the matching P/B field picture.
	 */
	if (picture->flags & (V4L2_AVS_PICTURE_FLAG_P_FIELD_ENHANCED |
			      V4L2_AVS_PICTURE_FLAG_B_FIELD_ENHANCED)) {
		if (sequence->profile_id != HISTB_VDEC_AVS_PROFILE_GUANGDIAN ||
		    picture->picture_structure != V4L2_AVS_PICTURE_STRUCTURE_FIELD ||
		    (picture->flags & V4L2_AVS_PICTURE_FLAG_P_FIELD_ENHANCED &&
		     picture->picture_coding_type != V4L2_AVS_PICTURE_TYPE_P) ||
		    (picture->flags & V4L2_AVS_PICTURE_FLAG_B_FIELD_ENHANCED &&
		     picture->picture_coding_type != V4L2_AVS_PICTURE_TYPE_B) ||
		    (picture->flags & V4L2_AVS_PICTURE_FLAG_P_FIELD_ENHANCED &&
		     picture->flags & V4L2_AVS_PICTURE_FLAG_B_FIELD_ENHANCED))
			return -EINVAL;
	}

	/* AVS+ fields do not exist in the JiZhun picture syntax. */
	if ((sequence->profile_id == HISTB_VDEC_AVS_PROFILE_JIZHUN &&
	     picture->bbv_delay > GENMASK(15, 0)) ||
	    picture->bbv_delay > GENMASK(22, 0))
		return -EINVAL;

	if (sequence->profile_id == HISTB_VDEC_AVS_PROFILE_JIZHUN &&
	    ((picture->flags & (V4L2_AVS_PICTURE_FLAG_WEIGHTING_QUANT |
			       V4L2_AVS_PICTURE_FLAG_CHROMA_QP_DISABLE |
			       V4L2_AVS_PICTURE_FLAG_AEC |
			       V4L2_AVS_PICTURE_FLAG_P_FIELD_ENHANCED |
			       V4L2_AVS_PICTURE_FLAG_B_FIELD_ENHANCED)) ||
	     picture->chroma_qp_delta_u || picture->chroma_qp_delta_v))
		return -EINVAL;

	if (picture->picture_coding_type == V4L2_AVS_PICTURE_TYPE_I) {
		if (decode->flags)
			return -EINVAL;
	} else if (picture->picture_coding_type == V4L2_AVS_PICTURE_TYPE_P) {
		if ((decode->flags & V4L2_AVS_DECODE_PARAM_FLAG_BACKWARD_REF) ||
		    !(decode->flags & V4L2_AVS_DECODE_PARAM_FLAG_FORWARD_REF0))
			return -EINVAL;
	} else if (!(decode->flags & V4L2_AVS_DECODE_PARAM_FLAG_BACKWARD_REF) ||
		   !(decode->flags & V4L2_AVS_DECODE_PARAM_FLAG_FORWARD_REF0)) {
		return -EINVAL;
	}

	width_mbs = DIV_ROUND_UP(sequence->horizontal_size, 16);
	height_mbs = histb_vdec_avs_height_mbs(sequence->vertical_size,
					       sequence->flags &
					       V4L2_AVS_SEQUENCE_FLAG_PROGRESSIVE);
	if (check_mul_overflow(width_mbs, height_mbs, &total_mbs) ||
	    !total_mbs || slices[0].slice_start_mb)
		return -EINVAL;

	for (i = 0; i < num_slices; i++) {
		u32 size, end;
		u8 code;

		if (slices[i].reserved || (slices[i].bit_size & 7) ||
		    slices[i].bit_size < 32 ||
		    slices[i].data_byte_offset > payload - 4 ||
		    slices[i].slice_start_mb >= total_mbs ||
		    (i && slices[i].slice_start_mb <=
			    slices[i - 1].slice_start_mb))
			return -EINVAL;
		size = slices[i].bit_size / 8;
		if ((size - 3) * 8U > GENMASK(24, 0))
			return -ERANGE;
		if (size > payload - slices[i].data_byte_offset ||
		    check_add_overflow(slices[i].data_byte_offset, size, &end) ||
		    (i + 1 < num_slices && end !=
			    slices[i + 1].data_byte_offset))
			return -EINVAL;
		if (payload_data[slices[i].data_byte_offset] != 0 ||
		    payload_data[slices[i].data_byte_offset + 1] != 0 ||
		    payload_data[slices[i].data_byte_offset + 2] != 1)
			return -EINVAL;
		code = payload_data[slices[i].data_byte_offset + 3];
		if (code > HISTB_VDEC_AVS_START_SLICE_MAX ||
		    (u64)code * width_mbs != slices[i].slice_start_mb)
			return -EINVAL;
	}

	return 0;
}

static int
histb_vdec_copy_avs_bitstream(struct histb_vdec_ctx *ctx,
			      struct vb2_v4l2_buffer *src,
			      unsigned long payload,
			      const struct v4l2_ctrl_avs_slice_params *slices,
			      unsigned int num_slices,
			      dma_addr_t *stream_dma,
			      struct histb_vdec_avs_stream *streams)
{
	struct histb_vdec_dma_buffer *stream = &ctx->bitstream;
	struct histb_vdec_dev *vdec = ctx->vdec;
	struct histb_vdec_dma_buffer replacement = { };
	u8 *src_cpu;
	unsigned int i;
	size_t allocation;

	if (payload > SIZE_MAX - HISTB_VDEC_RAW_STREAM_GUARD)
		return -E2BIG;
	allocation = payload + HISTB_VDEC_RAW_STREAM_GUARD;
	if (allocation > HISTB_VDEC_MAX_BITSTREAM)
		return -E2BIG;
	if (stream->size < allocation) {
		allocation = ALIGN(allocation, SZ_64K);
		replacement.cpu = dma_alloc_coherent(vdec->dev, allocation,
						     &replacement.dma, GFP_KERNEL);
		if (!replacement.cpu)
			return -ENOMEM;
		replacement.size = allocation;
		if (!histb_vdec_smmu_dma_buffer_valid(&replacement) ||
		    !IS_ALIGNED(replacement.dma, 16)) {
			dma_free_coherent(vdec->dev, allocation, replacement.cpu,
					  replacement.dma);
			return -ERANGE;
		}
		if (stream->cpu)
			dma_free_coherent(vdec->dev, stream->size, stream->cpu,
					  stream->dma);
		*stream = replacement;
	}
	if (!histb_vdec_smmu_dma_buffer_valid(stream))
		return -ERANGE;
	src_cpu = vb2_plane_vaddr(&src->vb2_buf, 0);
	if (!src_cpu)
		return -EOPNOTSUPP;
	src_cpu += src->vb2_buf.planes[0].data_offset;
	memcpy(stream->cpu, src_cpu, payload);
	memset(stream->cpu + payload, 0, stream->size - payload);
	for (i = 0; i < num_slices; i++) {
		streams[i].data_offset = slices[i].data_byte_offset;
		streams[i].size = slices[i].bit_size / 8;
	}
	*stream_dma = stream->dma;
	return 0;
}

static bool
histb_vdec_avs_ref_valid(const struct histb_vdec_avs_reference *reference)
{
	return reference->decoded && reference->metadata;
}

static bool
histb_vdec_avs_decoded_smmu_valid(const struct histb_vdec_decoded_buffer *decoded)
{
	return histb_vdec_smmu_dma_buffer_valid(&decoded->tile) &&
	       (!decoded->pmv.size ||
		histb_vdec_smmu_dma_buffer_valid(&decoded->pmv));
}

static u32
histb_vdec_avs_ref_distance(const struct histb_vdec_avs_reference *reference)
{
	return histb_vdec_avs_ref_valid(reference) ?
		reference->metadata->picture_distance : 0;
}

static u32
histb_vdec_avs_ref_structure(const struct histb_vdec_avs_reference *reference)
{
	return histb_vdec_avs_ref_valid(reference) &&
	       reference->metadata->picture_structure ==
	       V4L2_AVS_PICTURE_STRUCTURE_FRAME ? 3 : 1;
}

static u32
histb_vdec_avs_ref_tff(const struct histb_vdec_avs_reference *reference)
{
	if (!histb_vdec_avs_ref_valid(reference))
		return 0;
	if (reference->metadata->picture_structure ==
	    V4L2_AVS_PICTURE_STRUCTURE_FIELD)
		return 1;
	return reference->metadata->top_field_first;
}

static dma_addr_t
histb_vdec_avs_ref_tile(const struct histb_vdec_avs_reference *reference)
{
	return histb_vdec_avs_ref_valid(reference) ?
		round_down(reference->decoded->tile.dma, 16) : 0;
}

static dma_addr_t
histb_vdec_avs_ref_pmv(const struct histb_vdec_avs_reference *reference)
{
	return histb_vdec_avs_ref_valid(reference) ?
		round_down(reference->decoded->pmv.dma, 16) : 0;
}

static int
histb_vdec_avs_reference(struct histb_vdec_ctx *ctx,
			 struct histb_vdec_decoded_buffer *output,
			 u64 timestamp,
			 struct histb_vdec_avs_reference *reference)
{
	struct histb_vdec_decoded_buffer *decoded;

	decoded = histb_vdec_find_reference(ctx, timestamp);
	if (!decoded || decoded == output ||
	    !histb_vdec_decoded_buffer_valid(decoded) ||
	    !decoded->avs.valid || !decoded->avs.anchor)
		return -EINVAL;

	histb_vdec_propagate_error_taint(output, decoded);
	histb_vdec_sync_tile_for_device(ctx, decoded);
	reference->decoded = decoded;
	reference->metadata = &decoded->avs;
	return 0;
}

static int
histb_vdec_avs_build_references(struct histb_vdec_ctx *ctx,
				struct histb_vdec_decoded_buffer *output,
				const struct v4l2_ctrl_avs_picture *picture,
				const struct v4l2_ctrl_avs_decode_params *decode,
				struct histb_vdec_avs_reference refs[3])
{
	unsigned int i, j;
	int ret;

	memset(refs, 0, sizeof(*refs) * 3);
	if (picture->picture_coding_type == V4L2_AVS_PICTURE_TYPE_I &&
	    picture->picture_structure == V4L2_AVS_PICTURE_STRUCTURE_FIELD) {
		/*
		 * CV200 promotes a field-I current surface to pRef0 while building
		 * the transaction. It is not a persistent DPB reference.
		 */
		refs[0].decoded = output;
		refs[0].metadata = &output->avs;
	}
	if (picture->picture_coding_type == V4L2_AVS_PICTURE_TYPE_B &&
	    (decode->flags & V4L2_AVS_DECODE_PARAM_FLAG_BACKWARD_REF)) {
		ret = histb_vdec_avs_reference(ctx, output,
					       decode->backward_ref_ts, &refs[0]);
		if (ret)
			return ret;
	}
	if (decode->flags & V4L2_AVS_DECODE_PARAM_FLAG_FORWARD_REF0) {
		i = picture->picture_coding_type == V4L2_AVS_PICTURE_TYPE_B ? 1 : 0;
		ret = histb_vdec_avs_reference(ctx, output,
					       decode->forward_ref_ts[0], &refs[i]);
		if (ret)
			return ret;
	}
	if (decode->flags & V4L2_AVS_DECODE_PARAM_FLAG_FORWARD_REF1) {
		i = picture->picture_coding_type == V4L2_AVS_PICTURE_TYPE_B ? 2 : 1;
		ret = histb_vdec_avs_reference(ctx, output,
					       decode->forward_ref_ts[1], &refs[i]);
		if (ret)
			return ret;
	}

	for (i = 0; i < 3; i++)
		for (j = i + 1; refs[i].decoded && j < 3; j++)
			if (refs[i].decoded == refs[j].decoded)
				return -EINVAL;

	return 0;
}

static u32 histb_vdec_avs_wrap_distance(s32 distance)
{
	return distance & GENMASK(8, 0);
}

static u32 histb_vdec_avs_scale(u32 numerator, u32 denominator)
{
	return numerator * (denominator ? 512 / denominator : 1);
}

static u32 histb_vdec_avs_direct_scale(u32 denominator)
{
	return denominator ? 16384 / denominator : 32;
}

static u32 histb_vdec_avs_mv_scale(s32 numerator, s32 denominator)
{
	return histb_vdec_avs_scale(histb_vdec_avs_wrap_distance(numerator),
				     histb_vdec_avs_wrap_distance(denominator)) &
		GENMASK(17, 0);
}

static void
histb_vdec_avs_field_direct_config(const struct histb_vdec_avs_reference refs[3],
				   u32 r0, u32 direct[8])
{
	unsigned int k;

	if (histb_vdec_avs_ref_valid(&refs[1])) {
		for (k = 0; k < 4; k++) {
			const struct histb_vdec_avs_reference *reference =
				&refs[1 + k / 2];
			u32 distance;
			u32 parity = (k + 1) & 1;

			if (!histb_vdec_avs_ref_valid(reference))
				continue;
			distance = 2 * histb_vdec_avs_ref_distance(reference) +
				   parity;
			distance = histb_vdec_avs_wrap_distance(r0 - distance);
			direct[k] = histb_vdec_avs_direct_scale(distance);
		}
		memcpy(direct + 4, direct, 4 * sizeof(*direct));
		return;
	}

	for (k = 0; k < 4; k++) {
		const struct histb_vdec_avs_reference *reference =
			&refs[(k + 1) / 2];
		u32 distance;
		u32 parity = k & 1;

		if (!histb_vdec_avs_ref_valid(reference))
			continue;
		distance = 2 * histb_vdec_avs_ref_distance(reference) + parity;
		distance = histb_vdec_avs_wrap_distance(r0 + 1 - distance);
		direct[k + 4] = histb_vdec_avs_direct_scale(distance);
	}
}

static __le32 histb_vdec_pack4(u8 a, u8 b, u8 c, u8 d);

static void
histb_vdec_avs_pmv_config(const struct histb_vdec_avs_metadata *metadata,
			  const struct histb_vdec_avs_reference refs[3],
			  u32 words[32])
{
	u32 current_distance = 2 * metadata->picture_distance;
	u32 idx[8] = { };
	u32 mv[32] = { };
	u32 direct[8] = { };
	u32 bblk[8] = { };
	u32 sym[4] = { };
	u32 r0 = 2 * histb_vdec_avs_ref_distance(&refs[0]);
	u32 r1 = 2 * histb_vdec_avs_ref_distance(&refs[1]);
	s32 numerator, denominator;
	unsigned int i, j;

	memset(words, 0, sizeof(u32) * 32);
	if (metadata->picture_structure == V4L2_AVS_PICTURE_STRUCTURE_FIELD) {
		unsigned int f, k;

		switch (metadata->picture_coding_type) {
		case V4L2_AVS_PICTURE_TYPE_I:
			idx[4] = r0;
			mv[16] = 512;
			break;
		case V4L2_AVS_PICTURE_TYPE_P:
			if (histb_vdec_avs_ref_valid(&refs[1]))
				idx[3] = r1;
			idx[2] = idx[3] + 1;
			idx[7] = idx[2];
			idx[1] = r0;
			idx[6] = idx[1];
			idx[0] = r0 + 1;
			idx[5] = idx[0];
			idx[4] = current_distance;
			for (f = 0; f < 2; f++)
				for (i = 0; i < 4; i++)
					for (j = 0; j < 4; j++) {
						numerator = current_distance + f -
							    idx[4 * f + i];
						denominator = current_distance + f -
							      idx[4 * f + j];
						mv[16 * f + 4 * i + j] =
							histb_vdec_avs_mv_scale(numerator,
										denominator);
					}
			break;
		case V4L2_AVS_PICTURE_TYPE_B:
			if (histb_vdec_avs_ref_valid(&refs[1]))
				idx[1] = r1;
			idx[0] = idx[1] + 1;
			idx[4] = idx[0];
			idx[5] = idx[1];
			idx[2] = r0;
			idx[6] = idx[2];
			idx[3] = r0 + 1;
			idx[7] = idx[3];
			for (f = 0; f < 2; f++) {
				for (i = 0; i < 2; i++)
					for (j = 0; j < 2; j++) {
						numerator = current_distance + f -
							    idx[4 * f + i];
						denominator = current_distance + f -
							      idx[4 * f + j];
						mv[8 * f + 2 * i + j] =
							histb_vdec_avs_mv_scale(numerator,
										denominator);
					}
				for (i = 2; i < 4; i++)
					for (j = 2; j < 4; j++) {
						numerator = idx[4 * f + i] -
							    (current_distance + f);
						denominator = idx[4 * f + j] -
							      (current_distance + f);
						mv[8 * f + 4 + 2 * (i - 2) + j - 2] =
							histb_vdec_avs_mv_scale(numerator,
										denominator);
					}
			}
			histb_vdec_avs_field_direct_config(refs, r0, direct);
			for (f = 0; f < 2; f++) {
				u32 picture_distance = current_distance + f;

				for (k = 0; k < 2; k++) {
					numerator = idx[4 * f + 3 - k] -
						    picture_distance;
					denominator = picture_distance - idx[4 * f + k];
					sym[2 * f + k] =
						histb_vdec_avs_mv_scale(numerator,
									denominator);
				}
			}
			bblk[0] = histb_vdec_avs_wrap_distance(current_distance - idx[0]);
			bblk[1] = histb_vdec_avs_wrap_distance(current_distance - idx[1]);
			bblk[2] = histb_vdec_avs_wrap_distance(idx[2] - current_distance);
			bblk[3] = histb_vdec_avs_wrap_distance(idx[3] - current_distance);
			bblk[4] = histb_vdec_avs_wrap_distance(current_distance + 1 - idx[4]);
			bblk[5] = histb_vdec_avs_wrap_distance(current_distance + 1 - idx[5]);
			bblk[6] = histb_vdec_avs_wrap_distance(idx[6] - current_distance - 1);
			bblk[7] = histb_vdec_avs_wrap_distance(idx[7] - current_distance - 1);
			break;
		}

		if (metadata->picture_coding_type != V4L2_AVS_PICTURE_TYPE_B) {
			memcpy(words, mv, sizeof(mv));
			return;
		}
		for (i = 0; i < 16; i++)
			words[i] = mv[i];
		for (i = 0; i < 8; i++)
			words[16 + i] = direct[i] & GENMASK(14, 0);
		for (i = 0; i < 4; i++)
			words[24 + i] = (bblk[2 * i] & GENMASK(8, 0)) |
				((bblk[2 * i + 1] & GENMASK(8, 0)) << 9);
		for (i = 0; i < 4; i++)
			words[28 + i] = sym[i];
		return;
	}

	if (metadata->picture_coding_type == V4L2_AVS_PICTURE_TYPE_P) {
		idx[0] = 2 * histb_vdec_avs_ref_distance(&refs[0]);
		idx[1] = 2 * histb_vdec_avs_ref_distance(&refs[1]);
		for (i = 0; i < 2; i++) {
			for (j = 0; j < 2; j++) {
				numerator = histb_vdec_avs_wrap_distance(current_distance -
									 idx[i]);
				denominator = histb_vdec_avs_wrap_distance(current_distance -
									   idx[j]);
				words[4 * i + j] =
					histb_vdec_avs_scale(numerator, denominator) &
					GENMASK(17, 0);
			}
		}
		return;
	}
	if (metadata->picture_coding_type != V4L2_AVS_PICTURE_TYPE_B)
		return;

	idx[0] = 2 * histb_vdec_avs_ref_distance(&refs[1]);
	idx[2] = 2 * histb_vdec_avs_ref_distance(&refs[0]);
	numerator = histb_vdec_avs_wrap_distance(current_distance - idx[0]);
	words[0] = histb_vdec_avs_scale(numerator, numerator) & GENMASK(17, 0);
	numerator = histb_vdec_avs_wrap_distance(idx[2] - current_distance);
	words[4] = histb_vdec_avs_scale(numerator, numerator) & GENMASK(17, 0);
	for (i = 0; i < 2; i++) {
		u32 forward_distance;

		if (!histb_vdec_avs_ref_valid(&refs[i + 1]))
			continue;
		forward_distance = 2 * histb_vdec_avs_ref_distance(&refs[i + 1]);
		denominator = histb_vdec_avs_wrap_distance(idx[2] -
								    forward_distance);
		words[16 + i] = histb_vdec_avs_direct_scale(denominator) &
				GENMASK(14, 0);
	}
	words[24] = histb_vdec_avs_wrap_distance(current_distance - idx[0]);
	words[25] = histb_vdec_avs_wrap_distance(idx[2] - current_distance);
	if (histb_vdec_avs_ref_valid(&refs[1])) {
		numerator = histb_vdec_avs_wrap_distance(idx[2] - current_distance);
		denominator = histb_vdec_avs_wrap_distance(current_distance - idx[0]);
		words[28] = histb_vdec_avs_scale(numerator, denominator) &
			GENMASK(17, 0);
	}
}

static void
histb_vdec_write_avs_wq_matrix(__le32 *pic,
			       const struct v4l2_ctrl_avs_picture *picture)
{
	unsigned int row;

	for (row = 0; row < 16; row++)
		pic[64 + row] = cpu_to_le32(0x80808080);
	if (!(picture->flags & V4L2_AVS_PICTURE_FLAG_WEIGHTING_QUANT))
		return;
	for (row = 0; row < 8; row++) {
		const u16 *matrix = &picture->weighting_quant_matrix[row * 8];
		__le32 even = histb_vdec_pack4(matrix[0], matrix[2], matrix[4],
						       matrix[6]);
		__le32 odd = histb_vdec_pack4(matrix[1], matrix[3], matrix[5],
						      matrix[7]);

		pic[64 + row * 2] = even;
		pic[65 + row * 2] = odd;
	}
}

static int
histb_vdec_prepare_avs_slice(struct histb_vdec_ctx *ctx,
			     const struct v4l2_ctrl_avs_slice_params *slice,
			     const struct histb_vdec_avs_stream *stream,
			     dma_addr_t src_dma, u32 end_mb,
			     unsigned int index, unsigned int num_slices)
{
	__le32 *msg = histb_vdec_avs_slice_msg(ctx, index);
	dma_addr_t msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	dma_addr_t stream_base = round_down(src_dma, 16);
	u64 delta = src_dma + stream->data_offset + 4 - stream_base;
	u32 bit_offset = (delta & 15) * 8;
	u32 bytes_pos = delta & ~15U;
	u32 valid_bits = (stream->size - 4) * 8;

	/*
	 * The VDH consumes the vertical start-code byte after 00 00 01.  The
	 * vendor HAL represents an aligned fourth byte through the preceding
	 * word and a 120-bit offset instead of allowing a zero bit offset.
	 */
	if (bit_offset <= 7) {
		if (bytes_pos < 4)
			return -ERANGE;
		bytes_pos -= 4;
		bit_offset += 120;
	} else {
		bit_offset -= 8;
	}
	valid_bits += 8;
	if (delta > U32_MAX || bytes_pos > GENMASK(23, 0) ||
	    valid_bits > GENMASK(24, 0) || bit_offset > GENMASK(6, 0))
		return -ERANGE;
	msg[0] = cpu_to_le32(valid_bits | bit_offset << 25);
	msg[1] = cpu_to_le32(bytes_pos & GENMASK(23, 4));
	msg[4] = cpu_to_le32(slice->slice_start_mb | end_mb << 16);
	if (index + 1 < num_slices)
		msg[63] = cpu_to_le32(lower_32_bits(msg_dma +
			HISTB_VDEC_AVS_SLICE_MSG_OFFSET +
			(index + 1) * HISTB_VDEC_AVS_SLICE_MSG_SIZE));
	return 0;
}

static int
histb_vdec_prepare_avs_messages(struct histb_vdec_ctx *ctx,
				struct histb_vdec_decoded_buffer *decoded,
				const struct v4l2_ctrl_avs_sequence *sequence,
				const struct v4l2_ctrl_avs_picture *picture,
				const struct v4l2_ctrl_avs_decode_params *decode,
				const struct v4l2_ctrl_avs_slice_params *slices,
				unsigned int num_slices, dma_addr_t src_dma,
				const struct histb_vdec_avs_stream *streams)
{
	struct histb_vdec_avs_metadata metadata = {
		.anchor = picture->picture_coding_type != V4L2_AVS_PICTURE_TYPE_B,
		.top_field_first = picture->flags &
			V4L2_AVS_PICTURE_FLAG_TOP_FIELD_FIRST,
		.picture_structure = picture->picture_structure,
		.picture_coding_type = picture->picture_coding_type,
		.picture_distance = picture->picture_distance,
	};
	struct histb_vdec_avs_reference refs[3];
	const struct histb_vdec_avs_reference *backward, *forward0, *forward1;
	__le32 *pic = histb_vdec_avs_pic_msg(ctx);
	dma_addr_t msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	dma_addr_t recon, current_pmv, pmv_top, rcn_top, dblk_top, sed_top;
	u32 pmv[32] = { };
	u32 width_mbs = DIV_ROUND_UP(sequence->horizontal_size, 16);
	u32 height_mbs = histb_vdec_avs_height_mbs(sequence->vertical_size,
							  sequence->flags &
							  V4L2_AVS_SEQUENCE_FLAG_PROGRESSIVE);
	u32 value;
	unsigned int i;
	int ret;

	decoded->error_tainted = false;
	decoded->avs = metadata;
	decoded->avs.valid = false;
	ret = histb_vdec_avs_build_references(ctx, decoded, picture, decode, refs);
	if (ret)
		return ret;
	if (decoded->apc_slot != HISTB_VDEC_APC_INVALID)
		return -EINVAL;
	histb_vdec_reclaim_decoded_buffers(ctx, decoded);
	ret = histb_vdec_alloc_decoded_buffers(ctx, decoded);
	if (ret)
		return ret;
	if (!histb_vdec_avs_decoded_smmu_valid(decoded))
		return -ERANGE;
	for (i = 0; i < ARRAY_SIZE(refs); i++)
		if (histb_vdec_avs_ref_valid(&refs[i]) &&
		    !histb_vdec_avs_decoded_smmu_valid(refs[i].decoded))
			return -ERANGE;
	recon = round_down(decoded->tile.dma, 16);
	current_pmv = round_down(decoded->pmv.dma, 16);
	pmv_top = round_down(histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_PMV_TOP),
			     16);
	rcn_top = round_down(histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_RCN_TOP),
			     16);
	dblk_top = round_down(histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_DBLK_TOP),
			      16);
	sed_top = round_down(histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_SED_TOP),
			     16);
	memset(ctx->buffers[HISTB_VDEC_BUF_MSG].cpu, 0,
	       ctx->buffers[HISTB_VDEC_BUF_MSG].size);

	pic[0] = cpu_to_le32((width_mbs - 1) | (height_mbs - 1) << 16 |
		(sequence->chroma_format & 3) << 25 |
		(sequence->sample_precision & 3) << 27);
	value = !!(picture->flags & V4L2_AVS_PICTURE_FLAG_PROGRESSIVE_FRAME) |
		((picture->picture_structure ? 3 : 1) << 1) |
		(picture->picture_coding_type & 3) << 3 |
		!!(picture->flags & V4L2_AVS_PICTURE_FLAG_TOP_FIELD_FIRST) << 5 |
		!!(picture->flags & V4L2_AVS_PICTURE_FLAG_REPEAT_FIRST_FIELD) << 6 |
		!!(picture->flags & V4L2_AVS_PICTURE_FLAG_FIXED_QP) << 7 |
		(picture->picture_qp & 0x3f) << 8 |
		!!(picture->flags & V4L2_AVS_PICTURE_FLAG_SKIP_MODE) << 14 |
		!!(picture->flags & V4L2_AVS_PICTURE_FLAG_ADVANCED_PRED_DISABLE) << 15 |
		!!(picture->flags & V4L2_AVS_PICTURE_FLAG_REFERENCE) << 16 |
		!!(picture->flags & V4L2_AVS_PICTURE_FLAG_NO_FORWARD_REFERENCE) << 17;
	pic[1] = cpu_to_le32(value);
	value = !!(picture->flags & V4L2_AVS_PICTURE_FLAG_LOOP_FILTER_DISABLE) |
		!!(picture->flags & V4L2_AVS_PICTURE_FLAG_LOOP_FILTER_PARAMS) << 1 |
		(picture->alpha_c_offset & 0x1f) << 2 |
		(picture->beta_offset & 0x1f) << 7;
	pic[2] = cpu_to_le32(value);
	backward = &refs[0];
	if (picture->picture_coding_type == V4L2_AVS_PICTURE_TYPE_B) {
		forward0 = &refs[1];
		forward1 = &refs[2];
	} else {
		forward0 = &refs[0];
		forward1 = &refs[1];
	}
	pic[3] = cpu_to_le32(histb_vdec_avs_ref_tff(forward0) |
		histb_vdec_avs_ref_structure(forward0) << 1 |
		histb_vdec_avs_ref_tff(forward1) << 3 |
		histb_vdec_avs_ref_structure(forward1) << 4 |
		histb_vdec_avs_ref_tff(backward) << 6 |
		histb_vdec_avs_ref_structure(backward) << 7);
	pic[4] = cpu_to_le32(lower_32_bits(histb_vdec_avs_ref_tile(backward)));
	pic[5] = cpu_to_le32(lower_32_bits(histb_vdec_avs_ref_tile(forward0)));
	pic[6] = cpu_to_le32(lower_32_bits(histb_vdec_avs_ref_tile(forward1)));
	pic[7] = cpu_to_le32(lower_32_bits(recon));
	value = histb_vdec_avs_ref_structure(&refs[0]) << 24;
	if (histb_vdec_avs_ref_valid(&refs[0]))
		value |= !!refs[0].metadata->picture_coding_type;
	pic[8] = cpu_to_le32(value);
	value = !!(picture->flags & V4L2_AVS_PICTURE_FLAG_WEIGHTING_QUANT) |
		!!(picture->flags & V4L2_AVS_PICTURE_FLAG_CHROMA_QP_DISABLE) << 1 |
		(picture->chroma_qp_delta_v & 0x3f) << 2 |
		(picture->chroma_qp_delta_u & 0x3f) << 8 |
		!!(picture->flags & V4L2_AVS_PICTURE_FLAG_AEC) << 14 |
		!!(picture->flags & V4L2_AVS_PICTURE_FLAG_P_FIELD_ENHANCED) << 15 |
		!!(picture->flags & V4L2_AVS_PICTURE_FLAG_B_FIELD_ENHANCED) << 16;
	pic[12] = cpu_to_le32(value);
	pic[13] = cpu_to_le32(lower_32_bits(pmv_top));
	pic[14] = cpu_to_le32(lower_32_bits(rcn_top));
	pic[16] = cpu_to_le32(lower_32_bits(histb_vdec_avs_ref_pmv(&refs[0])));
	histb_vdec_avs_pmv_config(&metadata, refs, pmv);
	for (i = 0; i < ARRAY_SIZE(pmv); i++)
		pic[17 + i] = cpu_to_le32(pmv[i]);
	pic[49] = cpu_to_le32(lower_32_bits(current_pmv));
	pic[50] = cpu_to_le32(lower_32_bits(dblk_top));
	pic[51] = cpu_to_le32(lower_32_bits(sed_top));
	pic[63] = cpu_to_le32(lower_32_bits(msg_dma +
		HISTB_VDEC_AVS_SLICE_MSG_OFFSET));
	histb_vdec_write_avs_wq_matrix(pic, picture);

	for (i = 0; i < num_slices; i++) {
		u32 end_mb = i + 1 < num_slices ?
			slices[i + 1].slice_start_mb - 1 :
			width_mbs * height_mbs - 1;

		ret = histb_vdec_prepare_avs_slice(ctx, &slices[i], &streams[i],
						   src_dma, end_mb, i, num_slices);
		if (ret)
			return ret;
	}
	return 0;
}

static void
histb_vdec_write_qmatrix(__le32 *pic,
			 const struct v4l2_ctrl_h264_scaling_matrix *scaling,
			 bool custom)
{
	unsigned int list, word, dst = HISTB_VDEC_QMATRIX_WORD;

	if (!custom) {
		for (; dst < HISTB_VDEC_QMATRIX_WORD +
			     HISTB_VDEC_QMATRIX_WORDS; dst++)
			pic[dst] = cpu_to_le32(0x10101010);
		return;
	}

	for (list = 0; list < ARRAY_SIZE(scaling->scaling_list_4x4); list++) {
		const u8 *matrix = scaling->scaling_list_4x4[list];

		for (word = 0; word < 16; word += 4) {
			pic[dst++] = cpu_to_le32(matrix[word] |
						   matrix[word + 2] << 8 |
						   matrix[word + 1] << 16 |
						   matrix[word + 3] << 24);
		}
	}

	/* 4:2:0 uses only the Intra Y and Inter Y 8x8 matrices. */
	for (list = 0; list < 2; list++) {
		const u8 *matrix = scaling->scaling_list_8x8[list];

		for (word = 0; word < 64; word += 8) {
			pic[dst++] = cpu_to_le32(matrix[word] |
						   matrix[word + 2] << 8 |
						   matrix[word + 4] << 16 |
						   matrix[word + 6] << 24);
			pic[dst++] = cpu_to_le32(matrix[word + 1] |
						   matrix[word + 3] << 8 |
						   matrix[word + 5] << 16 |
						   matrix[word + 7] << 24);
		}
	}

	WARN_ON_ONCE(dst != HISTB_VDEC_QMATRIX_WORD +
			    HISTB_VDEC_QMATRIX_WORDS);
}

static __le32 histb_vdec_pack4(u8 a, u8 b, u8 c, u8 d)
{
	return cpu_to_le32((u32)a | (u32)b << 8 | (u32)c << 16 |
			   (u32)d << 24);
}

static __le32 histb_vdec_pack4_at(const u8 *matrix, u8 a, u8 b, u8 c, u8 d)
{
	return histb_vdec_pack4(matrix[a], matrix[b], matrix[c], matrix[d]);
}

static const u8 histb_vdec_mpeg2_zigzag[64] = {
	0, 1, 8, 16, 9, 2, 3, 10,
	17, 24, 32, 25, 18, 11, 4, 5,
	12, 19, 26, 33, 40, 48, 41, 34,
	27, 20, 13, 6, 7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36,
	29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46,
	53, 60, 61, 54, 47, 55, 62, 63,
};

static void histb_vdec_write_mpeg2_qmatrix(
		__le32 *pic, const struct v4l2_ctrl_mpeg2_quantisation *control)
{
	u8 intra[64], non_intra[64];
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(histb_vdec_mpeg2_zigzag); i++) {
		u8 raster = histb_vdec_mpeg2_zigzag[i];

		intra[raster] = control->intra_quantiser_matrix[i];
		non_intra[raster] = control->non_intra_quantiser_matrix[i];
	}
	for (i = 0; i < 8; i++) {
		pic[16 + 2 * i] = histb_vdec_pack4(intra[i], intra[16 + i],
						   intra[32 + i], intra[48 + i]);
		pic[17 + 2 * i] = histb_vdec_pack4(intra[8 + i],
						   intra[24 + i], intra[40 + i], intra[56 + i]);
		pic[32 + 2 * i] = histb_vdec_pack4(non_intra[i],
						   non_intra[16 + i], non_intra[32 + i],
				non_intra[48 + i]);
		pic[33 + 2 * i] = histb_vdec_pack4(non_intra[8 + i],
						   non_intra[24 + i], non_intra[40 + i],
				non_intra[56 + i]);
	}
}

static int histb_vdec_parse_mpeg2_slices(
		const u8 *data, size_t payload, u32 width_mbs, u32 total_mbs,
		struct histb_vdec_mpeg2_slice *slices, unsigned int *num_slices)
{
	size_t start = 0;
	u32 previous_mb = 0;
	unsigned int count = 0;

	while (start < payload) {
		struct histb_vdec_bitreader br;
		size_t end, byte_position, tail;
		u32 code, qscale, intra_flag, intra = 0, increment;
		u32 vertical_position, start_mb, value;
		bool sequence_end = false;

		if (count == HISTB_VDEC_MPEG2_MAX_SLICES || start + 4 > payload ||
		    data[start] || data[start + 1] || data[start + 2] != 1)
			return -EINVAL;
		code = data[start + 3];
		if (code < 1 || code > 0xaf)
			return -EINVAL;

		for (end = start + 4; end + 3 < payload; end++)
			if (!data[end] && !data[end + 1] && data[end + 2] == 1)
				break;
		if (end + 3 >= payload)
			end = payload;
		else if (data[end + 3] < 1 || data[end + 3] > 0xaf) {
			if (data[end + 3] != 0xb7)
				return -EINVAL;
			for (tail = end + 4; tail < payload; tail++)
				if (data[tail])
					return -EINVAL;
			sequence_end = true;
		}

		br.data = data + start;
		br.size = end - start;
		br.bitpos = 32;
		vertical_position = code;
		if (histb_vdec_read_bits(&br, 5, &qscale) || !qscale ||
		    histb_vdec_read_bits(&br, 1, &intra_flag))
			return -EINVAL;
		if (intra_flag) {
			if (histb_vdec_read_bits(&br, 1, &intra) ||
			    histb_vdec_read_bits(&br, 7, &value))
				return -EINVAL;
			for (;;) {
				if (histb_vdec_read_bits(&br, 1, &value))
					return -EINVAL;
				if (!value)
					break;
				if (histb_vdec_read_bits(&br, 8, &value))
					return -EINVAL;
			}
		}
		if (histb_vdec_mpeg2_mbaddr_increment(&br, &increment))
			return -EINVAL;

		start_mb = (vertical_position - 1) * width_mbs + increment - 1;
		if (start_mb >= total_mbs || (count && start_mb <= previous_mb))
			return -EINVAL;
		byte_position = start + br.bitpos / 8;
		slices[count].data_offset = round_down(byte_position, 16);
		slices[count].bit_offset = (byte_position & 15) * 8 +
			(br.bitpos & 7);
		/*
		 * VDH consumes each slice from its first macroblock through the end
		 * of the containing packet. The next down-message limits the decoded
		 * macroblock range; shortening this field at the next start code makes
		 * the engine report WAIT_STREAM before following that message.
		 */
		slices[count].valid_bits =
			(sequence_end ? end : payload) * 8 -
			(start * 8 + br.bitpos);
		if (slices[count].data_offset > GENMASK(23, 0) ||
		    !slices[count].valid_bits ||
		    slices[count].valid_bits > GENMASK(23, 0))
			return -EINVAL;
		slices[count].start_mb = start_mb;
		slices[count].quantiser_scale = qscale;
		slices[count].intra = intra;
		previous_mb = start_mb;
		count++;
		if (sequence_end)
			break;
		start = end;
	}

	if (!count || slices[0].start_mb)
		return -EINVAL;
	*num_slices = count;
	return 0;
}

static int
histb_vdec_parse_mpeg1_slices(const u8 *data, size_t payload,
				      u32 width_mbs, u32 total_mbs,
		struct histb_vdec_mpeg2_slice *slices, unsigned int *num_slices)
{
	size_t start = 0;
	u32 previous_mb = 0;
	unsigned int count = 0;

	while (start < payload) {
		struct histb_vdec_bitreader br;
		size_t end, byte_position, tail;
		u32 code, qscale, extra, increment, vertical_position, start_mb;
		bool sequence_end = false;

		if (count == HISTB_VDEC_MPEG2_MAX_SLICES || start + 4 > payload ||
		    data[start] || data[start + 1] || data[start + 2] != 1)
			return -EINVAL;
		code = data[start + 3];
		if (code < 1 || code > 0xaf)
			return -EINVAL;
		for (end = start + 4; end + 3 < payload; end++) {
			if (!data[end] && !data[end + 1] && data[end + 2] == 1)
				break;
		}
		if (end + 3 >= payload) {
			end = payload;
		} else if (data[end + 3] < 1 || data[end + 3] > 0xaf) {
			if (data[end + 3] != 0xb7)
				return -EINVAL;
			for (tail = end + 4; tail < payload; tail++) {
				if (data[tail])
					return -EINVAL;
			}
			sequence_end = true;
		}
		br.data = data + start;
		br.size = end - start;
		br.bitpos = 32;
		vertical_position = code;
		if (histb_vdec_read_bits(&br, 5, &qscale) || !qscale)
			return -EINVAL;
		/* MPEG-1 has extra_bit_slice, not MPEG-2's intra_slice_flag. */
		for (;;) {
			if (histb_vdec_read_bits(&br, 1, &extra))
				return -EINVAL;
			if (!extra)
				break;
			if (histb_vdec_read_bits(&br, 8, &extra))
				return -EINVAL;
		}
		if (histb_vdec_mpeg2_mbaddr_increment(&br, &increment))
			return -EINVAL;
		start_mb = (vertical_position - 1) * width_mbs + increment - 1;
		if (start_mb >= total_mbs || (count && start_mb <= previous_mb))
			return -EINVAL;
		byte_position = start + br.bitpos / 8;
		slices[count].data_offset = round_down(byte_position, 16);
		slices[count].bit_offset = (byte_position & 15) * 8 +
			(br.bitpos & 7);
		slices[count].valid_bits =
			(sequence_end ? end : payload) * 8 -
			(start * 8 + br.bitpos);
		if (slices[count].data_offset > GENMASK(23, 0) ||
		    !slices[count].valid_bits ||
		    slices[count].valid_bits > GENMASK(23, 0))
			return -EINVAL;
		slices[count].start_mb = start_mb;
		slices[count].quantiser_scale = qscale;
		slices[count].intra = 0;
		previous_mb = start_mb;
		count++;
		if (sequence_end)
			break;
		start = end;
	}
	if (!count || slices[0].start_mb)
		return -EINVAL;
	*num_slices = count;
	return 0;
}

static int histb_vdec_validate_mpeg2(
		struct histb_vdec_ctx *ctx,
		const struct v4l2_ctrl_mpeg2_sequence *sequence,
		const struct v4l2_ctrl_mpeg2_picture *picture,
		const struct v4l2_ctrl_mpeg2_quantisation *quantisation)
{
	bool field_picture;
	bool progressive_sequence;
	u8 profile;
	unsigned int i;

	if (!sequence || !picture || !quantisation)
		return -EINVAL;
	field_picture = picture->picture_structure != V4L2_MPEG2_PIC_FRAME;
	progressive_sequence = sequence->flags &
		V4L2_MPEG2_SEQ_FLAG_PROGRESSIVE;
	if (sequence->chroma_format != 1 ||
	    picture->picture_structure < V4L2_MPEG2_PIC_TOP_FIELD ||
	    picture->picture_structure > V4L2_MPEG2_PIC_FRAME ||
	    (progressive_sequence &&
	     (field_picture ||
	      !(picture->flags & V4L2_MPEG2_PIC_FLAG_PROGRESSIVE))) ||
	    (field_picture &&
	     (picture->flags & V4L2_MPEG2_PIC_FLAG_PROGRESSIVE)) ||
	    picture->picture_coding_type < V4L2_MPEG2_PIC_CODING_TYPE_I ||
	    picture->picture_coding_type > V4L2_MPEG2_PIC_CODING_TYPE_B ||
	    picture->intra_dc_precision > 3)
		return -EINVAL;
	if (sequence->profile_and_level_indication & BIT(7))
		return -EOPNOTSUPP;
	profile = (sequence->profile_and_level_indication >> 4) & 0x7;
	/* Only the non-scalable High, Main and Simple profiles are in scope. */
	if (profile != 1 && profile != 4 && profile != 5)
		return -EOPNOTSUPP;
	if (profile == 5 && picture->picture_coding_type ==
			    V4L2_MPEG2_PIC_CODING_TYPE_B)
		return -EINVAL;
	if (ALIGN(sequence->horizontal_size, 16) !=
			ALIGN(ctx->src.pix.width, 16) ||
	    ALIGN(sequence->vertical_size, 16) !=
			ALIGN(ctx->src.pix.height, 16) ||
	    ctx->src.pix.width > HISTB_VDEC_MPEG2_MAX_WIDTH ||
	    ctx->src.pix.height > HISTB_VDEC_MPEG2_MAX_HEIGHT)
		return -EINVAL;
	for (i = 0; i < sizeof(picture->f_code); i++)
		if (!((const u8 *)picture->f_code)[i] ||
		    ((const u8 *)picture->f_code)[i] > 15)
			return -EINVAL;
	return 0;
}

static int
histb_vdec_validate_mpeg1(struct histb_vdec_ctx *ctx,
				   const struct v4l2_ctrl_mpeg1_sequence *sequence,
		const struct v4l2_ctrl_mpeg1_picture *picture,
		const struct v4l2_ctrl_mpeg1_quantisation *quantisation)
{
	if (!sequence || !picture || !quantisation ||
	    picture->picture_coding_type < V4L2_MPEG1_PIC_CODING_TYPE_I ||
	    picture->picture_coding_type > V4L2_MPEG1_PIC_CODING_TYPE_B ||
	    picture->flags & ~(V4L2_MPEG1_PIC_FLAG_FULL_PEL_FORWARD |
				V4L2_MPEG1_PIC_FLAG_FULL_PEL_BACKWARD) ||
	    !picture->f_code[0] || picture->f_code[0] > 7 ||
	    !picture->f_code[1] || picture->f_code[1] > 7 ||
	    ALIGN(sequence->horizontal_size, 16) !=
			ALIGN(ctx->src.pix.width, 16) ||
	    ALIGN(sequence->vertical_size, 16) !=
			ALIGN(ctx->src.pix.height, 16) ||
	    ctx->src.pix.width > HISTB_VDEC_MPEG2_MAX_WIDTH ||
	    ctx->src.pix.height > HISTB_VDEC_MPEG2_MAX_HEIGHT)
		return -EINVAL;
	return 0;
}

static bool histb_vdec_vp8_delta_fits(s8 delta)
{
	return delta >= -15 && delta <= 15;
}

static int histb_vdec_validate_vp8(
		struct histb_vdec_ctx *ctx,
		const struct v4l2_ctrl_vp8_frame *frame,
		unsigned long payload)
{
	u64 control_end, token_offset, token_bytes = 0;
	u64 known_flags = V4L2_VP8_FRAME_FLAG_KEY_FRAME |
		V4L2_VP8_FRAME_FLAG_EXPERIMENTAL |
		V4L2_VP8_FRAME_FLAG_SHOW_FRAME |
		V4L2_VP8_FRAME_FLAG_MB_NO_SKIP_COEFF |
		V4L2_VP8_FRAME_FLAG_SIGN_BIAS_GOLDEN |
		V4L2_VP8_FRAME_FLAG_SIGN_BIAS_ALT;
	u32 header_bytes;
	unsigned int i;

	if (!frame || frame->version > 3 || frame->flags & ~known_flags ||
	    frame->segment.flags & ~(V4L2_VP8_SEGMENT_FLAG_ENABLED |
		V4L2_VP8_SEGMENT_FLAG_UPDATE_MAP |
		V4L2_VP8_SEGMENT_FLAG_UPDATE_FEATURE_DATA |
		V4L2_VP8_SEGMENT_FLAG_DELTA_VALUE_MODE) ||
	    frame->lf.flags & ~(V4L2_VP8_LF_ADJ_ENABLE |
		V4L2_VP8_LF_DELTA_UPDATE |
		V4L2_VP8_LF_FILTER_TYPE_SIMPLE) ||
	    frame->horizontal_scale > 3 || frame->vertical_scale > 3 ||
	    !frame->coder_state.range || frame->coder_state.bit_count > 7 ||
	    frame->quant.y_ac_qi > 127 || frame->lf.level > 63 ||
	    frame->lf.sharpness_level > 7 ||
	    !is_power_of_2(frame->num_dct_parts) ||
	    frame->num_dct_parts > HISTB_VDEC_VP8_PARTITIONS ||
	    ALIGN(frame->width, 16) != ALIGN(ctx->src.pix.width, 16) ||
	    ALIGN(frame->height, 16) != ALIGN(ctx->src.pix.height, 16) ||
	    ctx->src.pix.width > HISTB_VDEC_VP8_MAX_WIDTH ||
	    ctx->src.pix.height > HISTB_VDEC_VP8_MAX_HEIGHT ||
	    !histb_vdec_vp8_delta_fits(frame->quant.y_dc_delta) ||
	    !histb_vdec_vp8_delta_fits(frame->quant.y2_dc_delta) ||
	    !histb_vdec_vp8_delta_fits(frame->quant.y2_ac_delta) ||
	    !histb_vdec_vp8_delta_fits(frame->quant.uv_dc_delta) ||
	    !histb_vdec_vp8_delta_fits(frame->quant.uv_ac_delta))
		return -EINVAL;

	header_bytes = V4L2_VP8_FRAME_IS_KEY_FRAME(frame) ? 10 : 3;
	control_end = (u64)header_bytes + frame->first_part_size;
	if (control_end > payload ||
	    (u64)frame->first_part_header_bits + 8 >=
		(u64)frame->first_part_size * 8 ||
	    (u64)frame->first_part_size * 8 -
		(frame->first_part_header_bits + 8) > GENMASK(24, 0))
		return -EINVAL;
	token_offset = control_end + (frame->num_dct_parts - 1) * 3;
	if (token_offset > payload)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(frame->dct_part_sizes); i++) {
		if (i >= frame->num_dct_parts) {
			if (frame->dct_part_sizes[i])
				return -EINVAL;
			continue;
		}
		if (!frame->dct_part_sizes[i] ||
		    (u64)frame->dct_part_sizes[i] * 8 > GENMASK(24, 0))
			return -EINVAL;
		token_bytes += frame->dct_part_sizes[i];
	}
	for (i = 0; i < ARRAY_SIZE(frame->segment.quant_update); i++)
		if (frame->segment.quant_update[i] == -128 ||
		    frame->segment.lf_update[i] < -63 ||
		    frame->segment.lf_update[i] > 63 ||
		    frame->lf.ref_frm_delta[i] < -63 ||
		    frame->lf.ref_frm_delta[i] > 63 ||
		    frame->lf.mb_mode_delta[i] < -63 ||
		    frame->lf.mb_mode_delta[i] > 63)
			return -EINVAL;
	if (token_bytes > payload - token_offset)
		return -EINVAL;
	if (!V4L2_VP8_FRAME_IS_KEY_FRAME(frame) && !ctx->vp8_have_state)
		return -EINVAL;

	return 0;
}

static void
histb_vdec_write_hevc_qmatrix(__le32 *pic,
			      const struct v4l2_ctrl_hevc_scaling_matrix *scaling)
{
	const u8 *dc16 = scaling->scaling_list_dc_coef_16x16;
	const u8 *dc32 = scaling->scaling_list_dc_coef_32x32;
	unsigned int list, row, base;

	memset(&pic[64], 0, 256 * sizeof(*pic));
	for (list = 0; list < ARRAY_SIZE(scaling->scaling_list_8x8); list++) {
		const u8 *matrix = scaling->scaling_list_8x8[list];

		base = 64 + 16 * list;
		for (row = 0; row < 8; row++) {
			pic[base + 2 * row] = histb_vdec_pack4_at(matrix, row,
								  16 + row, 32 + row, 48 + row);
			pic[base + 2 * row + 1] = histb_vdec_pack4_at(matrix,
								      8 + row, 24 + row, 40 + row, 56 + row);
		}
	}
	for (list = 0; list < ARRAY_SIZE(scaling->scaling_list_16x16); list++) {
		const u8 *matrix = scaling->scaling_list_16x16[list];

		base = 160 + 16 * list;
		for (row = 0; row < 8; row++) {
			pic[base + 2 * row] = histb_vdec_pack4_at(matrix, row,
								  8 + row, 16 + row, 24 + row);
			pic[base + 2 * row + 1] = histb_vdec_pack4_at(matrix,
								      32 + row, 40 + row, 48 + row, 56 + row);
		}
	}
	for (list = 0; list < ARRAY_SIZE(scaling->scaling_list_32x32); list++) {
		const u8 *matrix = scaling->scaling_list_32x32[list];

		base = 256 + 16 * list;
		for (row = 0; row < 8; row++) {
			pic[base + 2 * row] = histb_vdec_pack4_at(matrix, row,
								  8 + row, 16 + row, 24 + row);
			pic[base + 2 * row + 1] = histb_vdec_pack4_at(matrix,
								      32 + row, 40 + row, 48 + row, 56 + row);
		}
	}
	for (list = 0; list < ARRAY_SIZE(scaling->scaling_list_4x4); list++) {
		const u8 *matrix = scaling->scaling_list_4x4[list];

		base = 288 + 4 * list;
		for (row = 0; row < 4; row++)
			pic[base + row] = histb_vdec_pack4_at(matrix, row,
							      8 + row, 4 + row, 12 + row);
	}
	pic[312] = histb_vdec_pack4(dc16[0], dc16[1], dc16[2], dc16[3]);
	pic[313] = histb_vdec_pack4(dc16[4], dc16[5], dc32[0], dc32[1]);
}

static int
histb_vdec_validate_h264(struct histb_vdec_ctx *ctx,
			 const struct v4l2_ctrl_h264_decode_params *decode,
			 const struct v4l2_ctrl_h264_sps *sps,
			 const struct v4l2_ctrl_h264_pps *pps,
			 const struct v4l2_ctrl_h264_slice_params *slice,
			 const struct v4l2_ctrl_h264_pred_weights *pred_weights,
			 unsigned long payload, bool new_frame)
{
	u32 width_mbs, map_height, frame_height, picture_height, units, qp;
	unsigned int active = 0, i, references_l0 = 0, references_l1 = 0;
	bool pframe, bframe, field_pic, bottom_field, mbaff;

	if (!decode || !sps || !pps || !slice || decode->nal_ref_idc > 3)
		return -EINVAL;
	field_pic = decode->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC;
	bottom_field = decode->flags & V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD;

	/*
	 * Baseline, Main and High, plus the two MVC profiles.  The MVC
	 * front-end is not wired in yet, so accepting 118 and 128 only means
	 * the parameters reach the same validation a non-MVC picture gets;
	 * the multivew extension itself is still parsed and discarded.
	 */
	if ((sps->profile_idc != 66 && sps->profile_idc != 77 &&
	     sps->profile_idc != 100 && sps->profile_idc != 118 &&
	     sps->profile_idc != 128) ||
	    sps->level_idc > 51 ||
	    sps->chroma_format_idc != 1 || sps->bit_depth_luma_minus8 ||
	    sps->bit_depth_chroma_minus8 ||
	    (sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE) ||
	    ((sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) &&
	     (sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD)) ||
	    (bottom_field && !field_pic) ||
	    (field_pic &&
	     (sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)))
		return -EINVAL;
	width_mbs = sps->pic_width_in_mbs_minus1 + 1;
	map_height = sps->pic_height_in_map_units_minus1 + 1;
	frame_height = map_height *
		((sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) ? 1 : 2);
	picture_height = field_pic ? map_height : frame_height;
	mbaff = !field_pic &&
		(sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD);
	units = width_mbs * (mbaff ? map_height : picture_height);
	if (width_mbs * 16 != ctx->src.pix.width ||
	    frame_height * 16 != ctx->src.pix.height ||
	    slice->first_mb_in_slice >= units || decode->reserved ||
	    slice->reserved)
		return -EINVAL;
	if (new_frame) {
		if (slice->first_mb_in_slice || ctx->h264_partial)
			return -EINVAL;
	} else if (!ctx->h264_partial ||
		   slice->first_mb_in_slice <= ctx->h264_last_first_mb) {
		return -EINVAL;
	}

	pframe = decode->flags & V4L2_H264_DECODE_PARAM_FLAG_PFRAME;
	bframe = decode->flags & V4L2_H264_DECODE_PARAM_FLAG_BFRAME;
	if (pps->num_slice_groups_minus1 || pps->weighted_bipred_idc > 2 ||
	    (pps->flags & V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT))
		return -EINVAL;

	if ((pframe && bframe) ||
	    (!pframe && !bframe &&
	     slice->slice_type != V4L2_H264_SLICE_TYPE_I) ||
	    (pframe && slice->slice_type != V4L2_H264_SLICE_TYPE_P) ||
	    (bframe && slice->slice_type != V4L2_H264_SLICE_TYPE_B) ||
	    slice->colour_plane_id ||
	    slice->redundant_pic_cnt ||
	    slice->header_bit_size >= payload * 8ULL)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(decode->dpb); i++) {
		const struct v4l2_h264_dpb_entry *entry = &decode->dpb[i];
		u32 allowed_flags = V4L2_H264_DPB_ENTRY_FLAG_VALID |
			V4L2_H264_DPB_ENTRY_FLAG_ACTIVE |
			V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM |
			V4L2_H264_DPB_ENTRY_FLAG_FIELD;

		if (memchr_inv(entry->reserved, 0, sizeof(entry->reserved)) ||
		    (entry->flags & ~allowed_flags))
			return -EINVAL;
		if (!(entry->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE))
			continue;
		/* Request userspace has already applied MMCO/reordering.  The VDH
		 * still needs the long-term bit in each packed slice reference. */
		if (!(entry->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID) ||
		    (entry->fields & ~V4L2_H264_FRAME_REF) || !entry->fields ||
		    (!(entry->flags & V4L2_H264_DPB_ENTRY_FLAG_FIELD) &&
		     entry->fields != V4L2_H264_FRAME_REF))
			return -EINVAL;
		active++;
	}

	/* Non-IDR I pictures may retain an active DPB for later pictures. */
	if (pframe || bframe)
		references_l0 = slice->num_ref_idx_l0_active_minus1 + 1;
	if (bframe)
		references_l1 = slice->num_ref_idx_l1_active_minus1 + 1;
	if (pframe || bframe) {
		/*
		 * Linux exposes 16 H.264 DPB entries and 32-entry reference
		 * lists. The message fields below are wide enough for the full
		 * public range; keep rejecting malformed references by index and
		 * ACTIVE membership, but do not impose the old eight-entry limit.
		 */
		if (!active || active > ARRAY_SIZE(decode->dpb) ||
		    references_l0 > ARRAY_SIZE(slice->ref_pic_list0) ||
		    (!bframe && slice->num_ref_idx_l1_active_minus1))
			return -EINVAL;
		for (i = 0; i < references_l0; i++) {
			const struct v4l2_h264_reference *ref =
				&slice->ref_pic_list0[i];
			const struct v4l2_h264_dpb_entry *entry;

			if (ref->index >= ARRAY_SIZE(decode->dpb))
				return -EINVAL;
			entry = &decode->dpb[ref->index];
			if (!(entry->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) ||
			    !ref->fields ||
			    (ref->fields & ~V4L2_H264_FRAME_REF) ||
			    (ref->fields & ~entry->fields) ||
			    (field_pic && ref->fields == V4L2_H264_FRAME_REF) ||
			    (!field_pic && ref->fields != V4L2_H264_FRAME_REF))
				return -EINVAL;
		}
	}
	if (bframe) {
		if (references_l1 > ARRAY_SIZE(slice->ref_pic_list1))
			return -EINVAL;
		for (i = 0; i < references_l1; i++) {
			const struct v4l2_h264_reference *ref =
				&slice->ref_pic_list1[i];
			const struct v4l2_h264_dpb_entry *entry;

			if (ref->index >= ARRAY_SIZE(decode->dpb))
				return -EINVAL;
			entry = &decode->dpb[ref->index];
			if (!(entry->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) ||
			    !ref->fields ||
			    (ref->fields & ~V4L2_H264_FRAME_REF) ||
			    (ref->fields & ~entry->fields) ||
			    (field_pic && ref->fields == V4L2_H264_FRAME_REF) ||
			    (!field_pic && ref->fields != V4L2_H264_FRAME_REF))
				return -EINVAL;
		}
	}
	if (V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(pps, slice)) {
		unsigned int list, references[2] = {
			references_l0, references_l1,
		};

		if (!pred_weights || pred_weights->luma_log2_weight_denom > 7 ||
		    pred_weights->chroma_log2_weight_denom > 7)
			return -EINVAL;
		for (list = 0; list < ARRAY_SIZE(references); list++) {
			const struct v4l2_h264_weight_factors *factors =
				&pred_weights->weight_factors[list];

			for (i = 0; i < references[list]; i++) {
				if (factors->luma_weight[i] < S8_MIN ||
				    factors->luma_weight[i] > S8_MAX ||
				    factors->luma_offset[i] < S8_MIN ||
				    factors->luma_offset[i] > S8_MAX ||
				    factors->chroma_weight[i][0] < S8_MIN ||
				    factors->chroma_weight[i][0] > S8_MAX ||
				    factors->chroma_offset[i][0] < S8_MIN ||
				    factors->chroma_offset[i][0] > S8_MAX ||
				    factors->chroma_weight[i][1] < S8_MIN ||
				    factors->chroma_weight[i][1] > S8_MAX ||
				    factors->chroma_offset[i][1] < S8_MIN ||
				    factors->chroma_offset[i][1] > S8_MAX)
					return -EINVAL;
			}
		}
	}

	qp = 26 + pps->pic_init_qp_minus26 + slice->slice_qp_delta;
	if (qp > 51)
		return -EINVAL;

	return 0;
}

static u32 histb_vdec_h264_weight_word(s16 weight, s16 offset, u16 denom)
{
	return ((u32)weight & GENMASK(8, 0)) << 3 |
	       ((u32)offset & GENMASK(7, 0)) << 12 |
	       (denom & GENMASK(2, 0));
}

static u32 histb_vdec_h264_chroma_v_word(s16 weight, s16 offset)
{
	return ((u32)weight & GENMASK(8, 0)) |
	       ((u32)offset & GENMASK(7, 0)) << 9;
}

static void histb_vdec_write_h264_weights(
		__le32 *slice_msg,
		const struct v4l2_ctrl_h264_pred_weights *pred_weights,
		unsigned int list, unsigned int references)
{
	const struct v4l2_h264_weight_factors *factors =
		&pred_weights->weight_factors[list];
	unsigned int luma_base = list ? 96 : 64;
	unsigned int chroma_u_base = list ? 160 : 128;
	unsigned int chroma_v_base = list ? 224 : 192;
	unsigned int i;

	for (i = 0; i < references; i++) {
		slice_msg[luma_base + i] = cpu_to_le32(histb_vdec_h264_weight_word(
			factors->luma_weight[i], factors->luma_offset[i],
			pred_weights->luma_log2_weight_denom));
		slice_msg[chroma_u_base + i] = cpu_to_le32(
			histb_vdec_h264_weight_word(factors->chroma_weight[i][0],
				factors->chroma_offset[i][0],
				pred_weights->chroma_log2_weight_denom));
		slice_msg[chroma_v_base + i] = cpu_to_le32(
			histb_vdec_h264_chroma_v_word(factors->chroma_weight[i][1],
				factors->chroma_offset[i][1]));
	}
}

static bool histb_vdec_hevc_current_long_term(
		const struct v4l2_ctrl_hevc_decode_params *decode, u8 dpb_index)
{
	unsigned int i;

	for (i = 0; i < decode->num_poc_lt_curr; i++)
		if (decode->poc_lt_curr[i] == dpb_index)
			return true;

	return false;
}

static int
histb_vdec_validate_hevc(struct histb_vdec_ctx *ctx,
			 const struct v4l2_ctrl_hevc_decode_params *decode,
			     const struct v4l2_ctrl_hevc_sps *sps,
			     const struct v4l2_ctrl_hevc_pps *pps,
			     const struct v4l2_ctrl_hevc_slice_params *slice,
			     const u32 *entry_offsets,
			     unsigned int num_entry_offsets,
			     unsigned long payload)
{
	u64 unsupported_sps = V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE;
	u32 min_cb_log2, ctb_log2, ctb_width, ctb_height;
	u32 min_pcm_log2, max_pcm_log2;
	u64 entry_bytes = 0;
	unsigned int i;
	unsigned int references_l0, references_l1;
	bool weighted;
	s32 chroma_denom, qp, qp_bd_offset;

	if (!decode || !sps || !pps || !slice ||
	    sps->chroma_format_idc != 1 ||
	    sps->bit_depth_luma_minus8 != sps->bit_depth_chroma_minus8 ||
	    (sps->bit_depth_luma_minus8 != 0 &&
	     sps->bit_depth_luma_minus8 != 2) ||
	    (ctx->dst.pix.pixelformat == V4L2_PIX_FMT_P010 &&
	     sps->bit_depth_luma_minus8 != 2) ||
	    (sps->flags & unsupported_sps) ||
	    ((slice->flags &
	      V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT) &&
	     !(pps->flags &
	       V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED)) ||
	    (slice->slice_type != V4L2_HEVC_SLICE_TYPE_I &&
	     slice->slice_type != V4L2_HEVC_SLICE_TYPE_P &&
	     slice->slice_type != V4L2_HEVC_SLICE_TYPE_B) ||
	    slice->data_byte_offset >= payload ||
	    slice->bit_size <= slice->data_byte_offset * 8 ||
	    slice->bit_size > payload * 8 ||
	    decode->num_active_dpb_entries > ARRAY_SIZE(decode->dpb) ||
	    (slice->slice_type == V4L2_HEVC_SLICE_TYPE_I &&
	     (!(decode->flags & V4L2_HEVC_DECODE_PARAM_FLAG_IRAP_PIC) ||
	      ((decode->flags & V4L2_HEVC_DECODE_PARAM_FLAG_IDR_PIC) &&
	       decode->num_active_dpb_entries))) ||
	    (slice->slice_type != V4L2_HEVC_SLICE_TYPE_I &&
	     ((decode->flags & V4L2_HEVC_DECODE_PARAM_FLAG_IRAP_PIC) ||
	      !decode->num_active_dpb_entries)))
		return -EINVAL;
	weighted = (slice->slice_type == V4L2_HEVC_SLICE_TYPE_P &&
		    (pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED)) ||
		   (slice->slice_type == V4L2_HEVC_SLICE_TYPE_B &&
		    (pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED));
	chroma_denom = slice->pred_weight_table.luma_log2_weight_denom +
		       slice->pred_weight_table.delta_chroma_log2_weight_denom;
	if (weighted && (slice->pred_weight_table.luma_log2_weight_denom > 7 ||
			 chroma_denom < 0 || chroma_denom > 7))
		return -EINVAL;
	references_l0 = slice->slice_type == V4L2_HEVC_SLICE_TYPE_I ? 0 :
		slice->num_ref_idx_l0_active_minus1 + 1;
	references_l1 = slice->slice_type == V4L2_HEVC_SLICE_TYPE_B ?
		slice->num_ref_idx_l1_active_minus1 + 1 : 0;
	/* Reference lists may select the same DPB entry more than once. */
	if (references_l0 > ARRAY_SIZE(slice->ref_idx_l0) ||
	    references_l1 > ARRAY_SIZE(slice->ref_idx_l1))
		return -EINVAL;
	if (decode->num_poc_st_curr_before >
		decode->num_active_dpb_entries ||
	    decode->num_poc_st_curr_before >
		ARRAY_SIZE(decode->poc_st_curr_before) ||
	    decode->num_poc_st_curr_after >
		decode->num_active_dpb_entries ||
	    decode->num_poc_st_curr_after >
		ARRAY_SIZE(decode->poc_st_curr_after) ||
	    decode->num_poc_lt_curr > decode->num_active_dpb_entries ||
	    decode->num_poc_lt_curr > ARRAY_SIZE(decode->poc_lt_curr) ||
	    (decode->num_poc_lt_curr &&
	     !(sps->flags & V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT)))
		return -EINVAL;
	/* CV200 accepts field pictures as DPB references; the reconstructed
	 * surface remains a complete frame, so no separate APC slot is needed. */
	for (i = 0; i < references_l0; i++) {
		if (slice->ref_idx_l0[i] >= decode->num_active_dpb_entries ||
		    ((decode->dpb[slice->ref_idx_l0[i]].flags &
		      V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE) &&
		     !histb_vdec_hevc_current_long_term(decode,
						       slice->ref_idx_l0[i])))
			return -EINVAL;
	}
	for (i = 0; i < references_l1; i++) {
		if (slice->ref_idx_l1[i] >= decode->num_active_dpb_entries ||
		    ((decode->dpb[slice->ref_idx_l1[i]].flags &
		      V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE) &&
		     !histb_vdec_hevc_current_long_term(decode,
						       slice->ref_idx_l1[i])))
			return -EINVAL;
	}
	for (i = 0; i < decode->num_active_dpb_entries; i++) {
		if (decode->dpb[i].flags &
		    ~V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE)
			return -EINVAL;
		if ((decode->dpb[i].flags &
		     V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE) &&
		    !(sps->flags &
		      V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT))
			return -EINVAL;
	}
	for (i = 0; i < decode->num_poc_lt_curr; i++) {
		u8 index = decode->poc_lt_curr[i];
		unsigned int j;

		if (index >= decode->num_active_dpb_entries ||
		    !(decode->dpb[index].flags &
		      V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE))
			return -EINVAL;
		for (j = 0; j < i; j++)
			if (decode->poc_lt_curr[j] == index)
				return -EINVAL;
	}
	if (slice->slice_type != V4L2_HEVC_SLICE_TYPE_I &&
	    (slice->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED) &&
	    ((slice->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0) ?
	     slice->collocated_ref_idx >= references_l0 :
	     slice->collocated_ref_idx >= references_l1))
		return -EINVAL;

	min_cb_log2 = sps->log2_min_luma_coding_block_size_minus3 + 3;
	ctb_log2 = min_cb_log2 +
		sps->log2_diff_max_min_luma_coding_block_size;
	if (ctb_log2 < 4 || ctb_log2 > 6 ||
	    sps->pic_width_in_luma_samples != ctx->dst.pix.width ||
	    sps->pic_height_in_luma_samples != ctx->dst.pix.height)
		return -EINVAL;
	if (sps->flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED) {
		min_pcm_log2 =
			sps->log2_min_pcm_luma_coding_block_size_minus3 + 3;
		max_pcm_log2 = min_pcm_log2 +
			sps->log2_diff_max_min_pcm_luma_coding_block_size;
		if (sps->pcm_sample_bit_depth_luma_minus1 + 1 >
			    sps->bit_depth_luma_minus8 + 8 ||
		    sps->pcm_sample_bit_depth_chroma_minus1 + 1 >
			    sps->bit_depth_chroma_minus8 + 8 ||
		    max_pcm_log2 > min_t(u32, ctb_log2, 5))
			return -EINVAL;
	}
	ctb_width = DIV_ROUND_UP(sps->pic_width_in_luma_samples,
				 BIT(ctb_log2));
	ctb_height = DIV_ROUND_UP(sps->pic_height_in_luma_samples,
				  BIT(ctb_log2));
	if (!ctb_width || !ctb_height || ctb_width * ctb_height > BIT(20))
		return -EINVAL;
	if (slice->slice_segment_addr >= ctb_width * ctb_height)
		return -EINVAL;
	if (slice->num_entry_point_offsets) {
		if (!(pps->flags & (V4L2_HEVC_PPS_FLAG_TILES_ENABLED |
				    V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED)) ||
		    !entry_offsets ||
		    num_entry_offsets != slice->num_entry_point_offsets)
			return -EINVAL;
		for (i = 0; i < num_entry_offsets; i++)
			entry_bytes += entry_offsets[i];
		if (entry_bytes > DIV_ROUND_UP(slice->bit_size, 8) -
				  slice->data_byte_offset)
			return -EINVAL;
	}
	qp = 26 + pps->init_qp_minus26 + slice->slice_qp_delta;
	qp_bd_offset = 6 * sps->bit_depth_luma_minus8;
	if (qp < -qp_bd_offset || qp > 51)
		return -EINVAL;

	return 0;
}

static int histb_vdec_hevc_init_tiles(
		const struct v4l2_ctrl_hevc_pps *pps, u32 ctb_width,
		u32 ctb_height, struct histb_vdec_hevc_tiles *tiles)
{
	bool enabled = pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED;
	bool uniform = pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING;
	u32 i;

	memset(tiles, 0, sizeof(*tiles));
	tiles->columns = enabled ? pps->num_tile_columns_minus1 + 1 : 1;
	tiles->rows = enabled ? pps->num_tile_rows_minus1 + 1 : 1;
	if (!tiles->columns || tiles->columns > HISTB_VDEC_HEVC_MAX_TILE_COLUMNS ||
	    tiles->columns > ctb_width ||
	    !tiles->rows || tiles->rows > HISTB_VDEC_HEVC_MAX_TILE_ROWS ||
	    tiles->rows > ctb_height)
		return -EINVAL;

	if (!enabled) {
		tiles->column_bd[1] = ctb_width;
		tiles->row_bd[1] = ctb_height;
	} else if (uniform) {
		for (i = 0; i <= tiles->columns; i++)
			tiles->column_bd[i] = i * ctb_width / tiles->columns;
		for (i = 0; i <= tiles->rows; i++)
			tiles->row_bd[i] = i * ctb_height / tiles->rows;
	} else {
		for (i = 0; i + 1 < tiles->columns; i++) {
			u32 width = pps->column_width_minus1[i] + 1;

			if (width > ctb_width - tiles->column_bd[i] -
				    (tiles->columns - i - 1))
				return -EINVAL;
			tiles->column_bd[i + 1] = tiles->column_bd[i] +
				width;
		}
		tiles->column_bd[tiles->columns] = ctb_width;
		for (i = 0; i + 1 < tiles->rows; i++) {
			u32 height = pps->row_height_minus1[i] + 1;

			if (height > ctb_height - tiles->row_bd[i] -
				     (tiles->rows - i - 1))
				return -EINVAL;
			tiles->row_bd[i + 1] = tiles->row_bd[i] +
				height;
		}
		tiles->row_bd[tiles->rows] = ctb_height;
	}

	if (tiles->column_bd[tiles->columns] != ctb_width ||
	    tiles->row_bd[tiles->rows] != ctb_height)
		return -EINVAL;

	return 0;
}

static u32 histb_vdec_hevc_rs_to_ts(const struct histb_vdec_hevc_tiles *tiles,
				    u32 ctb_width, u32 rs)
{
	u32 x = rs % ctb_width;
	u32 y = rs / ctb_width;
	u32 column, row, ts;

	for (column = 0; column + 1 < tiles->columns &&
	     x >= tiles->column_bd[column + 1]; column++)
		;
	for (row = 0; row + 1 < tiles->rows &&
	     y >= tiles->row_bd[row + 1]; row++)
		;

	ts = tiles->row_bd[row] * ctb_width;
	for (x = 0; x < column; x++)
		ts += (tiles->column_bd[x + 1] - tiles->column_bd[x]) *
		      (tiles->row_bd[row + 1] - tiles->row_bd[row]);
	ts += (y - tiles->row_bd[row]) *
	      (tiles->column_bd[column + 1] - tiles->column_bd[column]);
	ts += rs % ctb_width - tiles->column_bd[column];

	return ts;
}

static u32 histb_vdec_hevc_ts_to_rs(const struct histb_vdec_hevc_tiles *tiles,
				    u32 ctb_width, u32 ts)
{
	u32 column, row;

	for (row = 0; row < tiles->rows; row++) {
		u32 height = tiles->row_bd[row + 1] - tiles->row_bd[row];

		for (column = 0; column < tiles->columns; column++) {
			u32 width = tiles->column_bd[column + 1] -
				tiles->column_bd[column];
			u32 area = width * height;

			if (ts < area)
				return (tiles->row_bd[row] + ts / width) *
					ctb_width + tiles->column_bd[column] +
					ts % width;
			ts -= area;
		}
	}

	return 0;
}

static void histb_vdec_write_hevc_tile_info(
		struct histb_vdec_ctx *ctx,
		const struct histb_vdec_hevc_tiles *tiles, u32 ctb_log2)
{
	u8 *data = ctx->buffers[HISTB_VDEC_BUF_TILE_SEG].cpu;
	__le32 *column_pos = (__le32 *)(data + 1024);
	__le32 *row_pos = (__le32 *)(data + 1104);
	u32 scale = BIT(ctb_log2 - 4);
	u32 i, j;

	memset(data, 0, HISTB_VDEC_HEVC_TILE_INFO_SIZE);
	for (i = 0; i < tiles->columns; i++) {
		for (j = tiles->column_bd[i] * scale;
		     j < tiles->column_bd[i + 1] * scale; j++)
			data[j] = i;
		column_pos[i] = cpu_to_le32(tiles->column_bd[i] |
			((tiles->column_bd[i + 1] - 1) << 16));
	}
	for (i = 0; i < tiles->rows; i++) {
		for (j = tiles->row_bd[i] * scale;
		     j < tiles->row_bd[i + 1] * scale; j++)
			data[512 + j] = i;
		row_pos[i] = cpu_to_le32(tiles->row_bd[i] |
			((tiles->row_bd[i + 1] - 1) << 16));
	}
}

static int histb_vdec_validate_hevc_frame(
			struct histb_vdec_ctx *ctx,
		const struct v4l2_ctrl_hevc_decode_params *decode,
		const struct v4l2_ctrl_hevc_sps *sps,
		const struct v4l2_ctrl_hevc_pps *pps,
		const struct v4l2_ctrl_hevc_slice_params *slices,
		unsigned int num_slices, const u32 *entry_offsets,
			unsigned int num_entry_offsets, unsigned long payload)
{
	const struct v4l2_ctrl_hevc_slice_params *independent = NULL;
	struct histb_vdec_hevc_tiles tiles;
	u32 min_cb_log2, ctb_log2, ctb_width, ctb_height;
	size_t payload_offset = 0;
	unsigned int entry_offset = 0;
	unsigned int i;

	if (!num_slices || num_slices > HISTB_VDEC_HEVC_MAX_SLICES ||
	    slices[0].slice_segment_addr ||
	    (slices[0].flags &
	     V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT))
		return -EINVAL;
	min_cb_log2 = sps->log2_min_luma_coding_block_size_minus3 + 3;
	ctb_log2 = min_cb_log2 +
		sps->log2_diff_max_min_luma_coding_block_size;
	ctb_width = DIV_ROUND_UP(sps->pic_width_in_luma_samples, BIT(ctb_log2));
	ctb_height = DIV_ROUND_UP(sps->pic_height_in_luma_samples, BIT(ctb_log2));
	if (histb_vdec_hevc_init_tiles(pps, ctb_width, ctb_height, &tiles))
		return -EINVAL;
	for (i = 0; i < num_slices; i++) {
		const struct v4l2_ctrl_hevc_slice_params *slice = &slices[i];
		u64 inherited_flags = ~V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT;
		bool dependent = slice->flags &
			V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT;
		size_t nal_size = DIV_ROUND_UP(slice->bit_size, 8);
		int ret;

		if (!nal_size || nal_size > payload - payload_offset ||
		    (i && histb_vdec_hevc_rs_to_ts(&tiles, ctb_width,
						    slice->slice_segment_addr) <=
			  histb_vdec_hevc_rs_to_ts(&tiles, ctb_width,
						   slices[i - 1].slice_segment_addr)) ||
		    entry_offset + slice->num_entry_point_offsets >
			  num_entry_offsets)
			return -EINVAL;
		ret = histb_vdec_validate_hevc(ctx, decode, sps, pps, slice,
					       entry_offsets ? entry_offsets + entry_offset : NULL,
			slice->num_entry_point_offsets, nal_size);
		if (ret)
			return ret;
		if (!dependent) {
			independent = slice;
		} else if (!independent ||
			   independent->slice_type != slice->slice_type ||
			   independent->colour_plane_id != slice->colour_plane_id ||
			   independent->slice_pic_order_cnt != slice->slice_pic_order_cnt ||
			   independent->num_ref_idx_l0_active_minus1 !=
				slice->num_ref_idx_l0_active_minus1 ||
			   independent->num_ref_idx_l1_active_minus1 !=
				slice->num_ref_idx_l1_active_minus1 ||
			   independent->collocated_ref_idx != slice->collocated_ref_idx ||
			   independent->five_minus_max_num_merge_cand !=
				slice->five_minus_max_num_merge_cand ||
			   independent->slice_qp_delta != slice->slice_qp_delta ||
			   independent->slice_cb_qp_offset != slice->slice_cb_qp_offset ||
			   independent->slice_cr_qp_offset != slice->slice_cr_qp_offset ||
			   independent->slice_beta_offset_div2 !=
				slice->slice_beta_offset_div2 ||
			   independent->slice_tc_offset_div2 !=
				slice->slice_tc_offset_div2 ||
			   ((independent->flags ^ slice->flags) & inherited_flags) ||
			   memcmp(independent->ref_idx_l0, slice->ref_idx_l0,
				  sizeof(slice->ref_idx_l0)) ||
			   memcmp(independent->ref_idx_l1, slice->ref_idx_l1,
				  sizeof(slice->ref_idx_l1)) ||
			   memcmp(&independent->pred_weight_table,
				  &slice->pred_weight_table,
				  sizeof(slice->pred_weight_table))) {
			return -EINVAL;
		}
		payload_offset += nal_size;
		entry_offset += slice->num_entry_point_offsets;
	}

	if (payload_offset != payload ||
	    (entry_offset && entry_offset != num_entry_offsets))
		return -EINVAL;

	return 0;
}

static void histb_vdec_write_hevc_weighted_list(
		__le32 *slice_msg,
		const struct v4l2_hevc_pred_weight_table *weights,
		unsigned int list, unsigned int references)
{
	const s8 *delta_luma;
	const s8 *luma_offset;
	const s8 (*delta_chroma)[2];
	const s8 (*chroma_offset)[2];
	s32 luma_denom = weights->luma_log2_weight_denom;
	s32 chroma_denom = luma_denom +
		weights->delta_chroma_log2_weight_denom;
	unsigned int i;

	if (list) {
		delta_luma = weights->delta_luma_weight_l1;
		luma_offset = weights->luma_offset_l1;
		delta_chroma = weights->delta_chroma_weight_l1;
		chroma_offset = weights->chroma_offset_l1;
	} else {
		delta_luma = weights->delta_luma_weight_l0;
		luma_offset = weights->luma_offset_l0;
		delta_chroma = weights->delta_chroma_weight_l0;
		chroma_offset = weights->chroma_offset_l0;
	}

	for (i = 0; i < references; i++) {
		s32 luma_weight = BIT(luma_denom) + delta_luma[i];
		s32 chroma_weight[2];
		u32 index = list * 16 + i;
		unsigned int component;

		for (component = 0; component < 2; component++) {
			chroma_weight[component] = BIT(chroma_denom) +
				delta_chroma[i][component];
		}

		slice_msg[64 + index] = cpu_to_le32(
			((luma_offset[i] & 0xff) << 12) |
			((luma_weight & 0x1ff) << 3) | (luma_denom & 7));
		slice_msg[128 + index] = cpu_to_le32(
			((chroma_offset[i][0] & 0xff) << 12) |
			((chroma_weight[0] & 0x1ff) << 3) |
			(chroma_denom & 7));
		slice_msg[160 + list * 16 + i] = cpu_to_le32(
			(chroma_weight[1] & 0x1ff) |
			((chroma_offset[i][1] & 0xff) << 9));
	}
}

static int histb_vdec_prepare_hevc_slice(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		const struct v4l2_ctrl_hevc_decode_params *decode,
		const struct v4l2_ctrl_hevc_sps *sps,
		const struct v4l2_ctrl_hevc_pps *pps,
		const struct v4l2_ctrl_hevc_slice_params *slice,
		const s8 *dpb_to_apc, dma_addr_t src_dma,
		const struct histb_vdec_hevc_stream *stream_info,
		const struct histb_vdec_hevc_tiles *tiles, u32 ctb_width,
		u32 ctb_end_rs, u32 ctb_end_ts, u32 independent_addr,
		unsigned int slice_index,
		unsigned int num_slices)
{
	__le32 *slice_msg = histb_vdec_hevc_msg_slot(ctx,
		HISTB_VDEC_SLICE_MSG_SLOT + slice_index);
	dma_addr_t msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	dma_addr_t stream = src_dma + stream_info->data_offset;
	bool pframe = slice->slice_type == V4L2_HEVC_SLICE_TYPE_P;
	bool bframe = slice->slice_type == V4L2_HEVC_SLICE_TYPE_B;
	bool temporal_mvp;
	unsigned int references_l0 = pframe || bframe ?
		slice->num_ref_idx_l0_active_minus1 + 1 : 0;
	unsigned int references_l1 = bframe ?
		slice->num_ref_idx_l1_active_minus1 + 1 : 0;
	bool low_delay = true;
	u32 qp = 26 + pps->init_qp_minus26 + slice->slice_qp_delta;
	u32 value;
	unsigned int i;

	temporal_mvp = slice->flags &
		V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED;

	slice_msg[0] = cpu_to_le32(lower_32_bits(round_down(stream, 16) -
						   round_down(src_dma, 16)));
	slice_msg[1] = cpu_to_le32((lower_32_bits(stream) << 3) & 0x7f);
	slice_msg[2] = cpu_to_le32(stream_info->valid_bits);
	for (i = 0; i < references_l0; i++) {
		s8 slot = dpb_to_apc[slice->ref_idx_l0[i]];
		struct histb_vdec_decoded_buffer *ref;

		if (slot < 0)
			return -EINVAL;
		ref = ctx->apc[slot];
		if (!ref || !histb_vdec_decoded_buffer_valid(ref))
			return -EINVAL;
		histb_vdec_propagate_error_taint(decoded, ref);
		if (decode->dpb[slice->ref_idx_l0[i]].pic_order_cnt_val >
		    decode->pic_order_cnt_val)
			low_delay = false;
		if (decode->dpb[slice->ref_idx_l0[i]].flags &
		    V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE)
			slice_msg[13] |= cpu_to_le32(BIT(i));
		slice_msg[15 + i / 8] |= cpu_to_le32((u32)slot <<
								 (4 * (i % 8)));
	}
	for (i = 0; i < references_l1; i++) {
		s8 slot = dpb_to_apc[slice->ref_idx_l1[i]];
		struct histb_vdec_decoded_buffer *ref;

		if (slot < 0)
			return -EINVAL;
		ref = ctx->apc[slot];
		if (!ref || !histb_vdec_decoded_buffer_valid(ref))
			return -EINVAL;
		histb_vdec_propagate_error_taint(decoded, ref);
		if (decode->dpb[slice->ref_idx_l1[i]].pic_order_cnt_val >
		    decode->pic_order_cnt_val)
			low_delay = false;
		if (decode->dpb[slice->ref_idx_l1[i]].flags &
		    V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE)
			slice_msg[14] |= cpu_to_le32(BIT(i));
		slice_msg[17 + i / 8] |= cpu_to_le32((u32)slot <<
								 (4 * (i % 8)));
	}
	slice_msg[6] = cpu_to_le32((low_delay ? BIT(27) : 0) |
		(!!(slice->flags &
		    V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT) << 26) |
		((qp & 0x7f) << 19) |
		(!!(slice->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_CABAC_INIT) << 18) |
		independent_addr);
	value = (5 - slice->five_minus_max_num_merge_cand) << 24;
	if (pframe || bframe)
		value |= ((references_l1 ? references_l1 - 1 : 0) << 20) |
			 ((references_l0 - 1) << 16) |
			 (references_l1 << 8) | (references_l0 << 2) |
			 (bframe ? 2 : 1) |
			 (!!(slice->flags &
			     V4L2_HEVC_SLICE_PARAMS_FLAG_MVD_L1_ZERO) << 14) |
			 (temporal_mvp << 15);
	if (temporal_mvp && (pframe || bframe)) {
		unsigned int colocated_idx = slice->collocated_ref_idx;
		unsigned int colocated_dpb;
		struct histb_vdec_decoded_buffer *ref;
		bool colocated_l0 = slice->flags &
			V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0;
		s8 slot;

		if (colocated_l0) {
			if (colocated_idx >= references_l0)
				return -EINVAL;
			colocated_dpb = slice->ref_idx_l0[colocated_idx];
		} else {
			if (colocated_idx >= references_l1)
				return -EINVAL;
			colocated_dpb = slice->ref_idx_l1[colocated_idx];
		}
		slot = dpb_to_apc[colocated_dpb];
		if (slot < 0)
			return -EINVAL;
		ref = ctx->apc[slot];
		if (!ref || !histb_vdec_decoded_buffer_valid(ref))
			return -EINVAL;
		histb_vdec_propagate_error_taint(decoded, ref);
		value |= ((u32)(bframe ? slot : 0) << 28) |
			 (colocated_l0 << 27);
		slice_msg[23] = cpu_to_le32(lower_32_bits(ref->pmv.dma));
	} else if (temporal_mvp) {
		struct histb_vdec_decoded_buffer *ref = ctx->apc[0];

		if (!ref || !histb_vdec_decoded_buffer_valid(ref))
			return -EINVAL;
		histb_vdec_propagate_error_taint(decoded, ref);
		slice_msg[23] = cpu_to_le32(lower_32_bits(ref->pmv.dma));
	} else {
		slice_msg[23] = cpu_to_le32(lower_32_bits(
			histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_COLMB)));
	}
	slice_msg[7] = cpu_to_le32(value);
	slice_msg[8] = cpu_to_le32(slice->slice_segment_addr);
	slice_msg[9] = cpu_to_le32(
		(slice->slice_segment_addr / ctb_width) << 16 |
		(slice->slice_segment_addr % ctb_width));
	slice_msg[10] = cpu_to_le32(histb_vdec_hevc_rs_to_ts(
		tiles, ctb_width, slice->slice_segment_addr));
	slice_msg[11] = cpu_to_le32((slice->slice_cb_qp_offset & 0x1f) << 8 |
				      (slice->slice_cr_qp_offset & 0x1f));
	value = ((slice->slice_tc_offset_div2 & 0xf) << 16) |
		((slice->slice_beta_offset_div2 & 0xf) << 8) |
		(!!(slice->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA) << 3) |
		(!!(slice->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA) << 2) |
		(!!(slice->flags &
		    V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED) << 1) |
		!!(slice->flags &
		   V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED);
	slice_msg[12] = cpu_to_le32(value);
	slice_msg[24] = cpu_to_le32(0x400a |
		((3 * sps->bit_depth_luma_minus8) << 8));
	slice_msg[43] = cpu_to_le32(ctb_end_rs);
	slice_msg[44] = cpu_to_le32(ctb_end_ts);
	if (slice_index + 1 < num_slices)
		slice_msg[63] = cpu_to_le32(lower_32_bits(msg_dma +
			(HISTB_VDEC_SLICE_MSG_SLOT + slice_index + 1) *
			HISTB_VDEC_HEVC_MSG_SLOT_SIZE));
	if (pframe && (pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED))
		histb_vdec_write_hevc_weighted_list(slice_msg,
						    &slice->pred_weight_table, 0, references_l0);
	if (bframe && (pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED)) {
		histb_vdec_write_hevc_weighted_list(slice_msg,
						    &slice->pred_weight_table, 0, references_l0);
		histb_vdec_write_hevc_weighted_list(slice_msg,
						    &slice->pred_weight_table, 1, references_l1);
	}

	return 0;
}

static int
histb_vdec_prepare_hevc_messages(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		const struct v4l2_ctrl_hevc_decode_params *decode,
		const struct v4l2_ctrl_hevc_sps *sps,
		const struct v4l2_ctrl_hevc_pps *pps,
		const struct v4l2_ctrl_hevc_slice_params *slices,
		unsigned int num_slices,
		const struct v4l2_ctrl_hevc_scaling_matrix *scaling,
		dma_addr_t src_dma,
		const struct histb_vdec_hevc_stream *streams)
{
	__le32 *pic = histb_vdec_hevc_msg_slot(ctx, HISTB_VDEC_PIC_MSG_SLOT);
	dma_addr_t msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	u32 min_cb_log2 = sps->log2_min_luma_coding_block_size_minus3 + 3;
	u32 ctb_log2 = min_cb_log2 +
		sps->log2_diff_max_min_luma_coding_block_size;
	u32 min_tb_log2 = sps->log2_min_luma_transform_block_size_minus2 + 2;
	u32 max_tb_log2 = min_tb_log2 +
		sps->log2_diff_max_min_luma_transform_block_size;
	bool pcm_enabled = sps->flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED;
	u32 min_pcm_log2 = pcm_enabled ?
		sps->log2_min_pcm_luma_coding_block_size_minus3 + 3 : 0;
	u32 max_pcm_log2 = pcm_enabled ? min_pcm_log2 +
		sps->log2_diff_max_min_pcm_luma_coding_block_size : 0;
	u32 pcm_bit_depth_luma = pcm_enabled ?
		sps->pcm_sample_bit_depth_luma_minus1 + 1 : 0;
	u32 pcm_bit_depth_chroma = pcm_enabled ?
		sps->pcm_sample_bit_depth_chroma_minus1 + 1 : 0;
	u32 ctb_width = DIV_ROUND_UP(sps->pic_width_in_luma_samples,
				     BIT(ctb_log2));
	u32 ctb_height = DIV_ROUND_UP(sps->pic_height_in_luma_samples,
				      BIT(ctb_log2));
	u32 ctb_end_ts = ctb_width * ctb_height - 1;
	struct histb_vdec_hevc_tiles tiles;
	s8 dpb_to_apc[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	u32 independent_addr = 0;
	u32 value, i;

	if (histb_vdec_sync_hevc_apc(ctx, decode, dpb_to_apc))
		return -EINVAL;
	if (histb_vdec_hevc_init_tiles(pps, ctb_width, ctb_height, &tiles))
		return -EINVAL;
	if (decoded->apc_slot != HISTB_VDEC_APC_INVALID)
		return -EINVAL;
	histb_vdec_reclaim_decoded_buffers(ctx, decoded);
	if (histb_vdec_alloc_decoded_buffers(ctx, decoded))
		return -ENOMEM;
	memset(ctx->buffers[HISTB_VDEC_BUF_MSG].cpu, 0,
	       ctx->buffers[HISTB_VDEC_BUF_MSG].size);
	memset(ctx->buffers[HISTB_VDEC_BUF_APC_MV].cpu, 0,
	       V4L2_HEVC_DPB_ENTRIES_NUM_MAX * sizeof(__le32));
	histb_vdec_write_hevc_tile_info(ctx, &tiles, ctb_log2);
	value = (!!(sps->flags &
		    V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED) << 25) |
		(!!(sps->flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED) << 24) |
		(!!(pcm_enabled &&
		     (sps->flags & V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED)) << 23) |
		(!!(sps->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET) << 22) |
		(!!(sps->flags & V4L2_HEVC_SPS_FLAG_AMP_ENABLED) << 21) |
		(sps->chroma_format_idc << 19) |
		(pcm_enabled << 18) |
		((ctb_height - 1) << 9) | (ctb_width - 1);
	pic[0] = cpu_to_le32(value);
	pic[1] = cpu_to_le32((u32)decode->pic_order_cnt_val);
	value = (pps->log2_parallel_merge_level_minus2 << 29) |
		(!!(pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED) << 28) |
		(!!(pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED) << 27) |
		(sps->max_transform_hierarchy_depth_inter << 24) |
		(sps->max_transform_hierarchy_depth_intra << 21) |
		(min_pcm_log2 << 18) | (max_pcm_log2 << 15) |
		(min_tb_log2 << 12) | (max_tb_log2 << 9) |
		(sps->log2_diff_max_min_luma_coding_block_size << 6) |
		(min_cb_log2 << 3) | ctb_log2;
	pic[2] = cpu_to_le32(value);
	value = (6 * sps->bit_depth_chroma_minus8) << 22 |
		(6 * sps->bit_depth_luma_minus8) << 16 |
		(pcm_bit_depth_chroma << 12) |
		(pcm_bit_depth_luma << 8) |
		((sps->bit_depth_chroma_minus8 + 8) << 4) |
		(sps->bit_depth_luma_minus8 + 8);
	pic[3] = cpu_to_le32(value);
	pic[4] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(ctx,
								 HISTB_VDEC_BUF_SED_TOP)));
	pic[5] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(ctx,
								 HISTB_VDEC_BUF_PMV_TOP)));
	pic[6] = cpu_to_le32(lower_32_bits(decoded->pmv.dma));
	pic[7] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(ctx,
								 HISTB_VDEC_BUF_RCN_TOP)));
	for (i = 0; i < ARRAY_SIZE(ctx->apc); i++) {
		struct histb_vdec_decoded_buffer *ref = ctx->apc[i];
		dma_addr_t ref_dma;
		unsigned int first_valid;

		if (!ref) {
			for (first_valid = 0; first_valid < ARRAY_SIZE(ctx->apc);
			     first_valid++)
				if (ctx->apc[first_valid]) {
					ref = ctx->apc[first_valid];
					break;
				}
		}
		ref_dma = ref ? ref->tile.dma : decoded->tile.dma;
		pic[8 + i] = cpu_to_le32(lower_32_bits(round_down(ref_dma, 16)));
	}
	pic[25] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(ctx,
								  HISTB_VDEC_BUF_HEVC_CABAC)));
	for (i = 0; i < decode->num_active_dpb_entries; i++) {
		s8 slot = dpb_to_apc[i];

		if (slot >= 0)
			pic[26 + slot] =
				cpu_to_le32((u32)decode->dpb[i].pic_order_cnt_val);
	}
	pic[43] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(ctx,
								  HISTB_VDEC_BUF_PMV_LEFT)));
	pic[54] = cpu_to_le32(sps->pic_height_in_luma_samples << 16 |
				 sps->pic_width_in_luma_samples);
	pic[55] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(ctx,
								  HISTB_VDEC_BUF_TILE_SEG)));
	pic[56] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(ctx,
								  HISTB_VDEC_BUF_SAO_LEFT)));
	pic[57] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(ctx,
								  HISTB_VDEC_BUF_DBLK_LEFT)));
	pic[58] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(ctx,
								  HISTB_VDEC_BUF_SAO_TOP)));
	pic[59] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(ctx,
								  HISTB_VDEC_BUF_DBLK_TOP)));
	value = (!!(pps->flags & V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED) << 24) |
		(!!(pps->flags & V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED) << 23) |
		(!!(pps->flags & V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED) << 22) |
		(!!(pps->flags & V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED) << 21) |
		(!!(pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING) << 20) |
		(!!(pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) << 19) |
		((ctb_log2 - pps->diff_cu_qp_delta_depth) << 16) |
		(pps->diff_cu_qp_delta_depth << 13) |
		(!!(pps->flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED) << 12) |
		(!!(pps->flags & V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED) << 11) |
		(!!(pps->flags & V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED) << 10) |
		((pps->pps_cb_qp_offset & 0x1f) << 5) |
		(pps->pps_cr_qp_offset & 0x1f);
	pic[60] = cpu_to_le32(value);
	pic[61] = cpu_to_le32(pps->num_tile_rows_minus1 << 16 |
				 pps->num_tile_columns_minus1);
	pic[63] = cpu_to_le32(lower_32_bits(msg_dma +
		HISTB_VDEC_SLICE_MSG_SLOT * HISTB_VDEC_HEVC_MSG_SLOT_SIZE));
	if (sps->flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED)
		histb_vdec_write_hevc_qmatrix(pic, scaling);
	else
		/* The hardware ignores this region, but keep deterministic defaults. */
		for (i = 64; i < 314; i++)
			pic[i] = cpu_to_le32(0x10101010);

	for (i = 0; i < num_slices; i++) {
		u32 end_ts = i + 1 < num_slices ?
			histb_vdec_hevc_rs_to_ts(&tiles, ctb_width,
						 slices[i + 1].slice_segment_addr) - 1 : ctb_end_ts;
		u32 end_rs = histb_vdec_hevc_ts_to_rs(&tiles, ctb_width, end_ts);
		int ret;

		if (!(slices[i].flags &
		      V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT))
			independent_addr = slices[i].slice_segment_addr;
		ret = histb_vdec_prepare_hevc_slice(ctx, decoded, decode, sps, pps,
						    &slices[i], dpb_to_apc, src_dma, &streams[i],
			&tiles, ctb_width, end_rs, end_ts, independent_addr,
			i, num_slices);
		if (ret)
			return ret;
	}

	return 0;
}

static int histb_vdec_prepare_mpeg2_messages(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		const struct v4l2_ctrl_mpeg2_sequence *sequence,
		const struct v4l2_ctrl_mpeg2_picture *picture,
		const struct v4l2_ctrl_mpeg2_quantisation *quantisation,
		const struct histb_vdec_mpeg2_slice *slices,
		unsigned int num_slices, size_t payload)
{
	struct histb_vdec_decoded_buffer *backward, *forward;
	__le32 *pic = ctx->buffers[HISTB_VDEC_BUF_MSG].cpu +
		      HISTB_VDEC_MPEG2_PIC_MSG_OFFSET;
	__le32 *slice_msg = ctx->buffers[HISTB_VDEC_BUF_MSG].cpu +
			    HISTB_VDEC_MPEG2_SLICE_MSG_OFFSET;
	dma_addr_t msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	dma_addr_t recon, pmv;
	u32 width_mbs = DIV_ROUND_UP(sequence->horizontal_size, 16);
	u32 height_mbs = histb_vdec_mpeg2_picture_height_mbs(sequence, picture);
	u32 total_mbs = width_mbs * height_mbs;
	u32 value;
	unsigned int i;

	if (decoded->apc_slot != HISTB_VDEC_APC_INVALID)
		return -EINVAL;
	if (histb_vdec_sync_mpeg2_apc(ctx, decoded, picture,
					   &backward, &forward))
		return -EINVAL;
	if (decoded->apc_slot != HISTB_VDEC_APC_INVALID)
		return -EINVAL;
	/* The MPEG-2 controls name the two actual prediction anchors explicitly. */
	if (!ctx->mpeg2_second_field)
		decoded->error_tainted = false;
	histb_vdec_propagate_error_taint(decoded, backward);
	histb_vdec_propagate_error_taint(decoded, forward);
	if (!ctx->mpeg2_second_field) {
		decoded->mpeg2_field_saved = false;
		histb_vdec_reclaim_decoded_buffers(ctx, decoded);
		if (histb_vdec_alloc_decoded_buffers(ctx, decoded))
			return -ENOMEM;
		/* A first field only overwrites half the full-frame surface. */
		if (ctx->mpeg2_field_picture) {
			if (decoded->tile_noncoherent)
				dma_sync_single_for_cpu(ctx->vdec->dev,
						decoded->tile.dma,
						decoded->tile.size,
						DMA_BIDIRECTIONAL);
			memset(decoded->tile.cpu, 0, decoded->tile.size);
			if (decoded->tile_noncoherent)
				dma_sync_single_for_device(ctx->vdec->dev,
						   decoded->tile.dma,
						   decoded->tile.size,
						   DMA_BIDIRECTIONAL);
			memset(decoded->pmv.cpu, 0, decoded->pmv.size);
		}
	} else if (!decoded->tile.cpu || !decoded->pmv.cpu) {
		return -EINVAL;
	}
	recon = decoded->tile.dma;
	pmv = decoded->pmv.dma;
	if (payload * 8ULL + 24 > GENMASK(23, 0))
		return -E2BIG;
	memset(ctx->buffers[HISTB_VDEC_BUF_MSG].cpu, 0,
	       ctx->buffers[HISTB_VDEC_BUF_MSG].size);

	pic[0] = cpu_to_le32((width_mbs - 1) | ((height_mbs - 1) << 16));
	value = !!(picture->flags & V4L2_MPEG2_PIC_FLAG_FRAME_PRED_DCT);
	value |= picture->picture_structure << 8;
	value |= ctx->mpeg2_second_field << 10;
	value |= !!(picture->flags & V4L2_MPEG2_PIC_FLAG_CONCEALMENT_MV) << 16;
	value |= picture->picture_coding_type << 24;
	pic[1] = cpu_to_le32(value);
	value = picture->f_code[1][1] | picture->f_code[1][0] << 8 |
		picture->f_code[0][1] << 16 | picture->f_code[0][0] << 24;
	if (picture->flags & V4L2_MPEG2_PIC_FLAG_TOP_FIELD_FIRST)
		value |= BIT(31);
	pic[2] = cpu_to_le32(value);
	value = picture->intra_dc_precision;
	value |= !!(picture->flags & V4L2_MPEG2_PIC_FLAG_Q_SCALE_TYPE) << 8;
	value |= !!(picture->flags & V4L2_MPEG2_PIC_FLAG_INTRA_VLC) << 16;
	value |= !!(picture->flags & V4L2_MPEG2_PIC_FLAG_ALT_SCAN) << 24;
	pic[3] = cpu_to_le32(value);
	pic[4] = cpu_to_le32(lower_32_bits(round_down(
		backward ? backward->tile.dma : recon, 16)));
	pic[5] = cpu_to_le32(lower_32_bits(round_down(
		forward ? forward->tile.dma : recon, 16)));
	pic[6] = cpu_to_le32(lower_32_bits(round_down(recon, 16)));
	pic[7] = cpu_to_le32(lower_32_bits(round_down(pmv, 16)));
	pic[8] = 0;
	pic[9] = cpu_to_le32(payload * 8 + 24);
	pic[48] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_PMV_TOP)));
	pic[63] = cpu_to_le32(lower_32_bits(msg_dma +
					     HISTB_VDEC_MPEG2_SLICE_MSG_OFFSET));
	histb_vdec_write_mpeg2_qmatrix(pic, quantisation);
	ctx->mpeg2_ref_pic_type =
		(forward && ((forward == decoded && ctx->mpeg2_second_field) ||
			     forward->mpeg2_field_saved) ? 1 : 0) |
		(backward && backward->mpeg2_field_saved ? 1 : 0) << 2;

	for (i = 0; i < num_slices; i++) {
		__le32 *msg = slice_msg + i * 8;
		u32 end_mb = i + 1 < num_slices ?
			slices[i + 1].start_mb - 1 : total_mbs - 1;

		msg[0] = cpu_to_le32(slices[i].valid_bits |
					 slices[i].bit_offset << 24);
		msg[1] = cpu_to_le32(slices[i].data_offset);
		msg[4] = cpu_to_le32(slices[i].quantiser_scale |
					 slices[i].intra << 6);
		msg[5] = cpu_to_le32(slices[i].start_mb);
		msg[6] = cpu_to_le32(end_mb);
		if (i + 1 < num_slices)
			msg[7] = cpu_to_le32(lower_32_bits(msg_dma +
				HISTB_VDEC_MPEG2_SLICE_MSG_OFFSET +
				(i + 1) * HISTB_VDEC_MPEG2_SLICE_MSG_SIZE));
	}

	return 0;
}

static int
histb_vdec_prepare_mpeg1_messages(struct histb_vdec_ctx *ctx,
					   struct histb_vdec_decoded_buffer *decoded,
		const struct v4l2_ctrl_mpeg2_sequence *sequence,
		const struct v4l2_ctrl_mpeg2_picture *picture,
		const struct v4l2_ctrl_mpeg2_quantisation *quantisation,
		const struct v4l2_ctrl_mpeg1_picture *mpeg1_picture,
		const struct histb_vdec_mpeg2_slice *slices,
		unsigned int num_slices, size_t payload)
{
	__le32 *pic = ctx->buffers[HISTB_VDEC_BUF_MSG].cpu +
		HISTB_VDEC_MPEG2_PIC_MSG_OFFSET;
	u32 value;
	int ret;

	ret = histb_vdec_prepare_mpeg2_messages(
		ctx, decoded, sequence, picture, quantisation, slices, num_slices,
		payload);
	if (ret)
		return ret;

	/*
	 * CV200 MP2_DEC_PARAM_S maps Mpeg1Flag to word 0 bit 25, and the
	 * forward/backward full-pel flags to word 1 bits 3/4 respectively.
	 *
	 * Bit 25 was measured, not read: with bit 9 the engine ignores the flag
	 * entirely and decodes an MPEG-1 stream through the MPEG-2 path, which
	 * returns blocks that differ from the software decoder by 58/255 on
	 * average with a correlation of 0.02.  Sweeping every bit of word 0 and
	 * comparing block DC against the software decoder gives bit 25 with a
	 * correlation of exactly 1.000 and a byte difference of 1..3 grey
	 * levels, which is the same rounding spread MPEG-2 shows.
	 * See MPEG1-FLAG-BIT-25.md.
	 */
	value = le32_to_cpu(pic[0]) | BIT(25);
	pic[0] = cpu_to_le32(value);
	value = le32_to_cpu(pic[1]);
	if (mpeg1_picture->flags & V4L2_MPEG1_PIC_FLAG_FULL_PEL_FORWARD)
		value |= BIT(3);
	if (mpeg1_picture->flags & V4L2_MPEG1_PIC_FLAG_FULL_PEL_BACKWARD)
		value |= BIT(4);
	pic[1] = cpu_to_le32(value);
	return 0;
}

static void histb_vdec_pack_vp8_probabilities(
		struct histb_vdec_ctx *ctx,
		const struct v4l2_ctrl_vp8_frame *frame)
{
	const struct v4l2_vp8_entropy *entropy = &frame->entropy;
	u8 *table = ctx->buffers[HISTB_VDEC_BUF_VP8_PROB].cpu;
	unsigned int block, band, context;

	memset(table, 0, ctx->buffers[HISTB_VDEC_BUF_VP8_PROB].size);
	table[0] = frame->prob_skip_false;
	memcpy(table + 1, frame->segment.segment_probs, 3);
	table[4] = frame->prob_intra;
	table[5] = frame->prob_last;
	table[6] = frame->prob_gf;
	memcpy(table + 16, entropy->y_mode_probs,
	       sizeof(entropy->y_mode_probs));
	memcpy(table + 20, entropy->uv_mode_probs,
	       sizeof(entropy->uv_mode_probs));

	for (block = 0; block < ARRAY_SIZE(entropy->mv_probs); block++) {
		u8 *first = table + 704 + block * 32;
		u8 *second = first + 16;

		first[0] = entropy->mv_probs[block][1];
		first[1] = entropy->mv_probs[block][0];
		memcpy(first + 2, &entropy->mv_probs[block][2], 7);
		second[0] = entropy->mv_probs[block][1];
		memcpy(second + 1, &entropy->mv_probs[block][9], 10);
	}

	for (block = 0; block < ARRAY_SIZE(entropy->coeff_probs); block++)
		for (band = 0;
		     band < ARRAY_SIZE(entropy->coeff_probs[block]); band++)
			for (context = 0;
			     context < ARRAY_SIZE(entropy->coeff_probs[block][band]);
			     context++) {
				u8 *row = table + 768 +
					(((block * 8 + band) * 3 + context) * 16);

				memcpy(row, entropy->coeff_probs[block][band][context],
				       V4L2_VP8_COEFF_PROB_CNT);
			}
}

static u32 histb_vdec_vp8_quant_deltas(
		const struct v4l2_vp8_quantization *quant)
{
	const s8 deltas[] = { quant->y_dc_delta, quant->y2_dc_delta,
		quant->y2_ac_delta, quant->uv_dc_delta, quant->uv_ac_delta };
	u32 value = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(deltas); i++) {
		u32 magnitude = abs((int)deltas[i]);

		value |= (deltas[i] < 0) << (i * 5);
		value |= magnitude << (i * 5 + 1);
	}

	return value;
}

static u32 histb_vdec_vp8_stream_word(u64 bit_offset, u64 bit_length)
{
	return lower_32_bits(bit_length) |
	       (lower_32_bits(bit_offset) & 0x7f) << 25;
}

static int histb_vdec_prepare_vp8_messages(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		const struct v4l2_ctrl_vp8_frame *frame, dma_addr_t src_dma,
		size_t payload)
{
	struct histb_vdec_decoded_buffer *last, *golden, *alt;
	__le32 *pic = histb_vdec_msg_slot(ctx, HISTB_VDEC_PIC_MSG_SLOT);
	dma_addr_t msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	dma_addr_t control, control_aligned, recon, stream_base;
	u64 control_start_bits, control_end_bits, token_offset;
	u32 control_bit_offset, control_bit_length, control_byte_offset;
	u32 frame_type = V4L2_VP8_FRAME_IS_KEY_FRAME(frame) ? 0 : 1;
	u32 width_mbs = DIV_ROUND_UP(frame->width, 16);
	u32 height_mbs = DIV_ROUND_UP(frame->height, 16);
	u32 value, cumulative = 0;
	unsigned int i;

	if (histb_vdec_sync_vp8_apc(ctx, frame, &last, &golden, &alt))
		return -EINVAL;
	if (decoded->apc_slot != HISTB_VDEC_APC_INVALID)
		return -EINVAL;
	histb_vdec_reclaim_decoded_buffers(ctx, decoded);
	if (histb_vdec_alloc_decoded_buffers(ctx, decoded))
		return -ENOMEM;
	/* QBUF may happen while queued requests still reference this surface. */
	decoded->error_tainted = false;
	recon = decoded->tile.dma;
	stream_base = round_down(src_dma, 16) - 16;
	if (!last)
		last = decoded;
	if (!golden)
		golden = decoded;
	if (!alt)
		alt = decoded;
	histb_vdec_propagate_error_taint(decoded, last);
	histb_vdec_propagate_error_taint(decoded, golden);
	histb_vdec_propagate_error_taint(decoded, alt);

	memset(ctx->buffers[HISTB_VDEC_BUF_MSG].cpu, 0,
	       ctx->buffers[HISTB_VDEC_BUF_MSG].size);
	histb_vdec_pack_vp8_probabilities(ctx, frame);

	pic[0] = cpu_to_le32(frame_type |
		(ctx->vp8_have_state ? ctx->vp8_last_frame_type << 1 : 0));
	/*
	 * CV200 D1 is 2 * full_pixel - use_bilinear + 1.  The original
	 * setup_version path treats experimental profiles 4..7 like version 0.
	 */
	pic[1] = cpu_to_le32(
		(frame->flags & V4L2_VP8_FRAME_FLAG_EXPERIMENTAL) ? 1 :
		frame->version == 0 ? 1 : frame->version == 3 ? 2 : 0);
	pic[2] = cpu_to_le32((width_mbs - 1) | ((height_mbs - 1) << 16));
	pic[3] = cpu_to_le32(frame->prob_skip_false |
		!!(frame->flags & V4L2_VP8_FRAME_FLAG_MB_NO_SKIP_COEFF) << 8 |
		ilog2(frame->num_dct_parts) << 9);
	value = !!(frame->segment.flags & V4L2_VP8_SEGMENT_FLAG_ENABLED);
	value |= !!(frame->segment.flags & V4L2_VP8_SEGMENT_FLAG_UPDATE_MAP) << 1;
	value |= !(frame->segment.flags &
		   V4L2_VP8_SEGMENT_FLAG_DELTA_VALUE_MODE) << 2;
	value |= !!(frame->lf.flags & V4L2_VP8_LF_ADJ_ENABLE) << 3;
	pic[4] = cpu_to_le32(value);
	value = !!(frame->lf.flags & V4L2_VP8_LF_FILTER_TYPE_SIMPLE);
	value |= (ctx->vp8_have_state ? ctx->vp8_last_filter_type : 0) << 1;
	value |= frame->lf.level << 3;
	value |= frame->lf.sharpness_level << 9;
	value |= (ctx->vp8_have_state ? ctx->vp8_last_sharpness : 0) << 12;
	pic[5] = cpu_to_le32(value);
	pic[6] = cpu_to_le32(histb_vdec_vp8_quant_deltas(&frame->quant));
	pic[7] = cpu_to_le32(frame->quant.y_ac_qi);
	pic[8] = cpu_to_le32((u32)frame->coder_state.value << 16);
	pic[9] = cpu_to_le32(frame->coder_state.range);

	control_start_bits = ((u64)(V4L2_VP8_FRAME_IS_KEY_FRAME(frame) ?
		10 : 3) * 8) + frame->first_part_header_bits + 8;
	control_end_bits = (u64)(V4L2_VP8_FRAME_IS_KEY_FRAME(frame) ?
		10 : 3) * 8 + (u64)frame->first_part_size * 8;
	control = src_dma + control_start_bits / 8;
	control_aligned = round_down(control, 16);
	control_bit_offset = (control - control_aligned) * 8 +
		(control_start_bits & 7);
	control_bit_length = control_end_bits - control_start_bits;
	control_byte_offset = control_aligned - stream_base;
	if (control_bit_length > GENMASK(24, 0) ||
	    control_byte_offset > GENMASK(23, 0))
		return -EINVAL;
	pic[16] = cpu_to_le32(histb_vdec_vp8_stream_word(
		control_bit_offset, control_bit_length));
	pic[17] = cpu_to_le32(control_byte_offset);

	pic[20] = histb_vdec_pack4((u8)frame->segment.quant_update[0],
				   (u8)frame->segment.quant_update[1],
		(u8)frame->segment.quant_update[2],
		(u8)frame->segment.quant_update[3]);
	pic[21] = histb_vdec_pack4((u8)frame->segment.lf_update[0],
				   (u8)frame->segment.lf_update[1],
		(u8)frame->segment.lf_update[2],
		(u8)frame->segment.lf_update[3]);
	pic[22] = histb_vdec_pack4((u8)frame->lf.ref_frm_delta[0],
				   (u8)frame->lf.ref_frm_delta[1],
		(u8)frame->lf.ref_frm_delta[2],
		(u8)frame->lf.ref_frm_delta[3]);
	pic[23] = histb_vdec_pack4((u8)frame->lf.mb_mode_delta[0],
				   (u8)frame->lf.mb_mode_delta[1],
		(u8)frame->lf.mb_mode_delta[2],
		(u8)frame->lf.mb_mode_delta[3]);
	pic[24] = histb_vdec_pack4(0, 0,
				   !!(frame->flags & V4L2_VP8_FRAME_FLAG_SIGN_BIAS_GOLDEN),
		!!(frame->flags & V4L2_VP8_FRAME_FLAG_SIGN_BIAS_ALT));
	pic[25] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_VP8_SEG)));
	pic[32] = cpu_to_le32(lower_32_bits(round_down(recon, 16)));
	pic[33] = cpu_to_le32(lower_32_bits(round_down(alt->tile.dma, 16)));
	pic[34] = cpu_to_le32(lower_32_bits(round_down(golden->tile.dma, 16)));
	pic[35] = cpu_to_le32(lower_32_bits(round_down(last->tile.dma, 16)));
	pic[36] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_SED_TOP)));
	pic[37] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_PMV_TOP)));
	pic[38] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_RCN_TOP)));
	pic[39] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_VP8_PROB)));
	pic[40] = cpu_to_le32(lower_32_bits(histb_vdec_buffer_dma(
		ctx, HISTB_VDEC_BUF_DBLK_TOP)));
	pic[63] = cpu_to_le32(lower_32_bits(msg_dma +
		HISTB_VDEC_PIC_MSG_SLOT * HISTB_VDEC_H264_MSG_SLOT_SIZE + 256));

	token_offset = (V4L2_VP8_FRAME_IS_KEY_FRAME(frame) ? 10 : 3) +
		frame->first_part_size + (frame->num_dct_parts - 1) * 3;
	for (i = 0; i < frame->num_dct_parts; i++) {
		dma_addr_t token = src_dma + token_offset + cumulative;
		dma_addr_t aligned = round_down(token, 16);
		u32 byte_offset = aligned - stream_base;

		if (byte_offset > GENMASK(23, 0))
			return -EINVAL;

		pic[64 + i * 4] = cpu_to_le32(histb_vdec_vp8_stream_word(
			(token - aligned) * 8,
			(u64)frame->dct_part_sizes[i] * 8));
		pic[65 + i * 4] = cpu_to_le32(byte_offset);
		cumulative += frame->dct_part_sizes[i];
	}
	if (token_offset + cumulative > payload)
		return -EINVAL;

	ctx->vp8_pending_frame_type = frame_type;
	ctx->vp8_pending_filter_type =
		!!(frame->lf.flags & V4L2_VP8_LF_FILTER_TYPE_SIMPLE);
	ctx->vp8_pending_sharpness = frame->lf.sharpness_level;

	return 0;
}

static u32 histb_vdec_h264_slice_ref_flags(
		const struct v4l2_h264_dpb_entry *entry,
		const struct v4l2_h264_reference *reference,
		bool field_pic, bool sps_mbaff)
{
	u32 flags = 0;

	if (entry->flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM)
		flags |= HISTB_VDEC_SLICE_REF_LONG_TERM;
	if (field_pic) {
		if (entry->flags & V4L2_H264_DPB_ENTRY_FLAG_FIELD)
			flags |= HISTB_VDEC_SLICE_REF_PAIR;
		else if (sps_mbaff)
			flags |= HISTB_VDEC_SLICE_REF_MBAFF;
		if (reference->fields == V4L2_H264_TOP_FIELD_REF)
			flags |= HISTB_VDEC_SLICE_REF_TOP;
	} else if (entry->flags & V4L2_H264_DPB_ENTRY_FLAG_FIELD) {
		flags |= HISTB_VDEC_SLICE_REF_PAIR;
		if (entry->fields != V4L2_H264_FRAME_REF)
			flags |= HISTB_VDEC_SLICE_REF_MBAFF;
	} else if (sps_mbaff) {
		flags |= HISTB_VDEC_SLICE_REF_MBAFF;
	}

	return flags;
}

static int
histb_vdec_prepare_messages(struct histb_vdec_ctx *ctx,
			    struct histb_vdec_decoded_buffer *decoded,
			    const struct v4l2_ctrl_h264_decode_params *decode,
			    const struct v4l2_ctrl_h264_sps *sps,
			    const struct v4l2_ctrl_h264_pps *pps,
			    const struct v4l2_ctrl_h264_scaling_matrix *scaling,
			    const struct v4l2_ctrl_h264_slice_params *slice,
			    const struct v4l2_ctrl_h264_pred_weights *pred_weights,
			    dma_addr_t stream_base_dma, dma_addr_t src_dma,
			    unsigned long payload, u32 tail_bits,
			    unsigned int slice_index, bool new_picture,
			    bool new_surface)
{
	__le32 *pic = histb_vdec_msg_slot(ctx, HISTB_VDEC_PIC_MSG_SLOT);
	__le32 *slice_msg = histb_vdec_msg_slot(
		ctx, HISTB_VDEC_SLICE_MSG_SLOT + slice_index);
	dma_addr_t stream_base = round_down(stream_base_dma, 16);
	s64 header_bits = slice->header_bit_size;
	dma_addr_t bytes_pos;
	dma_addr_t current_pmv;
	dma_addr_t recon;
	dma_addr_t sed_top = histb_vdec_buffer_dma(ctx,
						   HISTB_VDEC_BUF_SED_TOP);
	dma_addr_t pmv_top = histb_vdec_buffer_dma(ctx,
						   HISTB_VDEC_BUF_PMV_TOP);
	dma_addr_t rcn_top = histb_vdec_buffer_dma(ctx,
						   HISTB_VDEC_BUF_RCN_TOP);
	dma_addr_t dblk_top = histb_vdec_buffer_dma(ctx,
						    HISTB_VDEC_BUF_DBLK_TOP);
	dma_addr_t cabac = histb_vdec_buffer_dma(ctx,
						HISTB_VDEC_BUF_H264_CABAC);
	dma_addr_t slice_dma = histb_vdec_buffer_dma(ctx,
						   HISTB_VDEC_BUF_MSG) +
		(HISTB_VDEC_SLICE_MSG_SLOT + slice_index) *
		HISTB_VDEC_H264_MSG_SLOT_SIZE;
	u64 valid_bits = payload * 8ULL - slice->header_bit_size;
	u32 width_mbs = sps->pic_width_in_mbs_minus1 + 1;
	u32 map_height = sps->pic_height_in_map_units_minus1 + 1;
	u32 frame_height = map_height *
		((sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) ? 1 : 2);
	u32 picture_height;
	u32 qp = 26 + pps->pic_init_qp_minus26 + slice->slice_qp_delta;
	u32 pic0, value;
	bool pframe = decode->flags & V4L2_H264_DECODE_PARAM_FLAG_PFRAME;
	bool bframe = decode->flags & V4L2_H264_DECODE_PARAM_FLAG_BFRAME;
	bool field_pic = decode->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC;
	bool bottom_field = decode->flags &
		V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD;
	bool sps_mbaff = sps->flags &
		V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD;
	bool mbaff = !field_pic && sps_mbaff;
	s8 dpb_to_apc[HISTB_VDEC_H264_DPB_SIZE];
	unsigned int i, references_l0 = 0, references_l1 = 0;

	picture_height = field_pic ? map_height : frame_height;
	if (new_picture) {
		ctx->h264_decode = *decode;
		ctx->h264_sps = *sps;
		ctx->h264_pps = *pps;
		ctx->h264_scaling = *scaling;
		ctx->h264_controls_valid = true;
	} else {
		const u8 *saved = NULL, *actual = NULL;
		size_t bytes = 0;
		const char *name = NULL;

		if (!ctx->h264_controls_valid)
			return -EINVAL;
		if (memcmp(&ctx->h264_decode, decode, sizeof(*decode))) {
			saved = (const u8 *)&ctx->h264_decode;
			actual = (const u8 *)decode;
			bytes = sizeof(*decode);
			name = "decode";
		} else if (memcmp(&ctx->h264_sps, sps, sizeof(*sps))) {
			saved = (const u8 *)&ctx->h264_sps;
			actual = (const u8 *)sps;
			bytes = sizeof(*sps);
			name = "sps";
		} else if (memcmp(&ctx->h264_pps, pps, sizeof(*pps))) {
			saved = (const u8 *)&ctx->h264_pps;
			actual = (const u8 *)pps;
			bytes = sizeof(*pps);
			name = "pps";
		} else if (memcmp(&ctx->h264_scaling, scaling,
				 sizeof(*scaling))) {
			saved = (const u8 *)&ctx->h264_scaling;
			actual = (const u8 *)scaling;
			bytes = sizeof(*scaling);
			name = "scaling";
		}
		if (name) {
			for (i = 0; i < bytes; i++)
				if (saved[i] != actual[i]) {
					dev_err(ctx->vdec->dev,
						"H.264 %s byte %u changed across slices: %02x/%02x\n",
						name, i, saved[i], actual[i]);
					break;
				}
			return -EINVAL;
		}
	}
	if (histb_vdec_sync_apc(ctx, decoded, decode, dpb_to_apc)) {
		histb_vdec_reset_apc(ctx);
		return -EINVAL;
	}
	if (!histb_vdec_h264_current_apc_valid(ctx, decoded, decode,
						    dpb_to_apc)) {
		histb_vdec_reset_apc(ctx);
		return -EINVAL;
	}
	if (new_picture && new_surface) {
		histb_vdec_reclaim_decoded_buffers(ctx, decoded);
		if (histb_vdec_alloc_decoded_buffers(ctx, decoded))
			return -ENOMEM;
		/* A first field only overwrites half the reconstruction surface. */
		if (field_pic) {
			if (decoded->tile_noncoherent)
				dma_sync_single_for_cpu(ctx->vdec->dev,
						decoded->tile.dma,
						decoded->tile.size,
						DMA_BIDIRECTIONAL);
			memset(decoded->tile.cpu, 0, decoded->tile.size);
			if (decoded->tile_noncoherent)
				dma_sync_single_for_device(ctx->vdec->dev,
						   decoded->tile.dma,
						   decoded->tile.size,
						   DMA_BIDIRECTIONAL);
			memset(decoded->pmv.cpu, 0, decoded->pmv.size);
		}
	} else if (!decoded->tile.cpu || !decoded->pmv.cpu) {
		return -EINVAL;
	}
	current_pmv = decoded->pmv.dma;
	if (field_pic && bottom_field)
		current_pmv += decoded->pmv.size / 2;
	recon = decoded->tile.dma;
	if (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE)
		header_bits = ALIGN(header_bits, 8);
	if (pframe || bframe)
		references_l0 = slice->num_ref_idx_l0_active_minus1 + 1;
	if (bframe)
		references_l1 = slice->num_ref_idx_l1_active_minus1 + 1;

	if (header_bits < 0 || header_bits >= payload * 8ULL)
		return -EINVAL;
	bytes_pos = src_dma + header_bits / 8;
	valid_bits = payload * 8ULL - header_bits;
	if (tail_bits >= valid_bits)
		return -EINVAL;
	valid_bits -= tail_bits;
	if (valid_bits > U32_MAX)
		return -EOVERFLOW;
	if (new_picture)
		memset(ctx->buffers[HISTB_VDEC_BUF_MSG].cpu, 0,
		       ctx->buffers[HISTB_VDEC_BUF_MSG].size);
	else
		memset(slice_msg, 0, HISTB_VDEC_H264_MSG_SLOT_SIZE);

	pic0 = FIELD_PREP(HISTB_VDEC_PIC_WIDTH, width_mbs - 1) |
	       FIELD_PREP(HISTB_VDEC_PIC_HEIGHT, picture_height - 1);
	if (field_pic)
		pic0 |= FIELD_PREP(HISTB_VDEC_PIC_STRUCTURE,
				   bottom_field ? 2 : 1);
	else if (mbaff)
		pic0 |= FIELD_PREP(HISTB_VDEC_PIC_STRUCTURE, 3);
	if (sps->chroma_format_idc == 1)
		pic0 |= HISTB_VDEC_PIC_420;
	if (pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED)
		pic0 |= HISTB_VDEC_PIC_CONSTRAINED_INTRA;
	if (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE)
		pic0 |= HISTB_VDEC_PIC_ENTROPY_CODING;
	if (pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE)
		pic0 |= HISTB_VDEC_PIC_TRANSFORM_8X8;
	if (decode->nal_ref_idc)
		pic0 |= HISTB_VDEC_PIC_REFERENCE;

	pic[0] = cpu_to_le32(pic0);
	pic[1] = cpu_to_le32(lower_32_bits(round_down(recon, 16)));
	if (field_pic)
		pic[2] = cpu_to_le32((u32)(bottom_field ?
			decode->bottom_field_order_cnt : decode->top_field_order_cnt));
	else
		pic[2] = cpu_to_le32((u32)min(decode->top_field_order_cnt,
						      decode->bottom_field_order_cnt));
	if (mbaff) {
		pic[3] = cpu_to_le32((u32)decode->top_field_order_cnt);
		pic[4] = cpu_to_le32((u32)decode->bottom_field_order_cnt);
	}
	pic[5] = cpu_to_le32(lower_32_bits(sed_top));
	pic[6] = cpu_to_le32(lower_32_bits(pmv_top));
	pic[7] = cpu_to_le32(lower_32_bits(current_pmv));
	pic[8] = cpu_to_le32(lower_32_bits(rcn_top));
	if (new_picture)
		pic[9] = cpu_to_le32(lower_32_bits(slice_dma));
	for (i = 0; i < ARRAY_SIZE(ctx->apc); i++) {
		struct histb_vdec_decoded_buffer *ref = ctx->apc[i];
		dma_addr_t ref_dma;

		if (!ref)
			ref = ctx->apc[0];
		ref_dma = ref ? ref->tile.dma : recon;
		pic[10 + i] = cpu_to_le32(lower_32_bits(round_down(ref_dma, 16)));
	}
	pic[26] = cpu_to_le32(lower_32_bits(cabac));
	for (i = 0; i < ARRAY_SIZE(decode->dpb); i++) {
		s8 slot = dpb_to_apc[i];

		if (slot < 0)
			continue;
		pic[27 + slot * 2] =
			cpu_to_le32((u32)decode->dpb[i].top_field_order_cnt);
		pic[28 + slot * 2] =
			cpu_to_le32((u32)decode->dpb[i].bottom_field_order_cnt);
	}
	pic[59] = cpu_to_le32(lower_32_bits(dblk_top));
	histb_vdec_write_qmatrix(pic, scaling,
				 pps->flags &
				 V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT);
	if (new_picture) {
		memcpy(ctx->h264_picture_message, pic,
		       sizeof(ctx->h264_picture_message));
	} else if (memcmp(ctx->h264_picture_message, pic,
			  sizeof(ctx->h264_picture_message))) {
		for (i = 0; i < ARRAY_SIZE(ctx->h264_picture_message); i++)
			if (ctx->h264_picture_message[i] != pic[i]) {
				dev_err(ctx->vdec->dev,
					"H.264 picture word %u changed across slices: %08x/%08x\n",
					i, le32_to_cpu(ctx->h264_picture_message[i]),
					le32_to_cpu(pic[i]));
				break;
			}
		return -EINVAL;
	}

	slice_msg[0] = cpu_to_le32(lower_32_bits(round_down(bytes_pos, 16) -
						   stream_base));
	value = ((lower_32_bits(bytes_pos) << 3) |
		 (header_bits & 7)) & 0x7f;
	slice_msg[1] = cpu_to_le32(value);
	slice_msg[2] = cpu_to_le32(valid_bits);
	value = FIELD_PREP(HISTB_VDEC_SLICE_FIRST_MB,
			   slice->first_mb_in_slice) |
		FIELD_PREP(HISTB_VDEC_SLICE_CABAC_INIT,
			   slice->cabac_init_idc) |
		FIELD_PREP(HISTB_VDEC_SLICE_QP, qp);
	slice_msg[6] = cpu_to_le32(value);
	value = FIELD_PREP(HISTB_VDEC_SLICE_HW_TYPE,
			   bframe ? HISTB_VDEC_SLICE_TYPE_B :
			   pframe ? HISTB_VDEC_SLICE_TYPE_P :
			   HISTB_VDEC_SLICE_TYPE_I);
	if (pframe || bframe) {
		value |= FIELD_PREP(HISTB_VDEC_SLICE_LIST0_SIZE,
				    references_l0) |
			 FIELD_PREP(HISTB_VDEC_SLICE_LIST0_ACTIVE,
				    slice->num_ref_idx_l0_active_minus1);
	}
	if (bframe) {
		value |= FIELD_PREP(HISTB_VDEC_SLICE_LIST1_SIZE,
				    references_l1) |
			 FIELD_PREP(HISTB_VDEC_SLICE_LIST1_ACTIVE,
				    slice->num_ref_idx_l1_active_minus1);
		if (slice->flags & V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED)
			value |= HISTB_VDEC_SLICE_DIRECT_SPATIAL;
	}
	if (sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE)
		value |= HISTB_VDEC_SLICE_DIRECT_8X8;
	slice_msg[7] = cpu_to_le32(value);
	for (i = 0; i < references_l0; i++) {
		const struct v4l2_h264_reference *reference =
			&slice->ref_pic_list0[i];
		const struct v4l2_h264_dpb_entry *entry =
			&decode->dpb[reference->index];
		u8 dpb = reference->index;
		u32 shift = 5 * (i % 4);
		u32 ref_code;
		u32 ref_flags;
		s8 slot = dpb_to_apc[dpb];

		if (slot < 0 || !ctx->apc[slot])
			return -EINVAL;
		ref_code = slot << 1;
		if (field_pic && reference->fields == V4L2_H264_BOTTOM_FIELD_REF)
			ref_code |= 1;
		value = le32_to_cpu(slice_msg[20 + i / 4]);
		value |= ref_code << shift;
		slice_msg[20 + i / 4] = cpu_to_le32(value);
		ref_flags = histb_vdec_h264_slice_ref_flags(entry, reference,
							       field_pic, sps_mbaff);
		value = le32_to_cpu(slice_msg[12 + i / 8]);
		value |= ref_flags << (4 * (i % 8));
		slice_msg[12 + i / 8] = cpu_to_le32(value);
		histb_vdec_propagate_error_taint(decoded, ctx->apc[slot]);
	}
	for (i = 0; i < references_l1; i++) {
		const struct v4l2_h264_reference *reference =
			&slice->ref_pic_list1[i];
		const struct v4l2_h264_dpb_entry *entry =
			&decode->dpb[reference->index];
		u8 dpb = reference->index;
		u32 shift = 5 * (i % 4);
		u32 ref_code;
		u32 ref_flags;
		s8 slot = dpb_to_apc[dpb];

		if (slot < 0 || !ctx->apc[slot])
			return -EINVAL;
		ref_code = slot << 1;
		if (field_pic && reference->fields == V4L2_H264_BOTTOM_FIELD_REF)
			ref_code |= 1;
		value = le32_to_cpu(slice_msg[28 + i / 4]);
		value |= ref_code << shift;
		slice_msg[28 + i / 4] = cpu_to_le32(value);
		ref_flags = histb_vdec_h264_slice_ref_flags(entry, reference,
							       field_pic, sps_mbaff);
		value = le32_to_cpu(slice_msg[16 + i / 8]);
		value |= ref_flags << (4 * (i % 8));
		slice_msg[16 + i / 8] = cpu_to_le32(value);
		histb_vdec_propagate_error_taint(decoded, ctx->apc[slot]);
	}
	if (bframe) {
		const struct v4l2_h264_reference *reference =
			&slice->ref_pic_list1[0];
		const struct v4l2_h264_dpb_entry *entry =
			&decode->dpb[reference->index];
		u8 dpb = reference->index;
		s8 slot = dpb_to_apc[dpb];
		struct histb_vdec_decoded_buffer *colocated;
		dma_addr_t colocated_pmv;

		if (slot < 0)
			return -EINVAL;
		colocated = ctx->apc[slot];
		if (!colocated || !histb_vdec_decoded_buffer_valid(colocated))
			return -EINVAL;
		colocated_pmv = round_down(colocated->pmv.dma, 16);
		if (field_pic &&
		    (entry->flags & V4L2_H264_DPB_ENTRY_FLAG_FIELD) &&
		    reference->fields == V4L2_H264_BOTTOM_FIELD_REF)
			colocated_pmv += colocated->pmv.size / 2;
		slice_msg[8] = cpu_to_le32(lower_32_bits(colocated_pmv));
		if (!field_pic &&
		    (entry->flags & V4L2_H264_DPB_ENTRY_FLAG_FIELD) &&
		    entry->fields == V4L2_H264_FRAME_REF)
			slice_msg[9] = cpu_to_le32(lower_32_bits(
				round_down(colocated->pmv.dma, 16) +
				colocated->pmv.size / 2));
	}
	for (i = 0; i < ARRAY_SIZE(ctx->apc); i++) {
		unsigned int polarity;

		for (polarity = 0; polarity < 2; polarity++) {
			u32 map = 0;
			u32 map_entry = i * 2 + polarity;
			unsigned int j;

			for (j = 0; j < references_l0; j++) {
				const struct v4l2_h264_reference *reference =
					&slice->ref_pic_list0[j];
				u8 dpb = reference->index;

				if (dpb_to_apc[dpb] != i)
					continue;
				if (field_pic && reference->fields !=
					    (polarity ? V4L2_H264_BOTTOM_FIELD_REF :
					     V4L2_H264_TOP_FIELD_REF))
					continue;
				map = j;
				break;
			}
			value = le32_to_cpu(slice_msg[36 + map_entry / 4]);
			value |= map << (5 * (map_entry % 4));
			slice_msg[36 + map_entry / 4] = cpu_to_le32(value);
		}
	}
	value = FIELD_PREP(HISTB_VDEC_SLICE_CHROMA_QP1,
			   pps->chroma_qp_index_offset & 0x1f) |
		FIELD_PREP(HISTB_VDEC_SLICE_CHROMA_QP2,
			   pps->second_chroma_qp_index_offset & 0x1f);
	if (pframe && (pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED))
		value |= FIELD_PREP(HISTB_VDEC_SLICE_WEIGHT, 1);
	else if (bframe)
		value |= FIELD_PREP(HISTB_VDEC_SLICE_WEIGHT,
				    pps->weighted_bipred_idc);
	slice_msg[10] = cpu_to_le32(value);
	value = FIELD_PREP(HISTB_VDEC_SLICE_DEBLOCK,
			   slice->disable_deblocking_filter_idc) |
		FIELD_PREP(HISTB_VDEC_SLICE_BETA,
			   slice->slice_beta_offset_div2 & 0xf) |
		FIELD_PREP(HISTB_VDEC_SLICE_ALPHA,
			   slice->slice_alpha_c0_offset_div2 & 0xf);
	slice_msg[11] = cpu_to_le32(value);
	value = width_mbs * picture_height - 1;
	slice_msg[44] = cpu_to_le32(value);
	if (V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(pps, slice)) {
		histb_vdec_write_h264_weights(slice_msg, pred_weights, 0,
					       references_l0);
		if (bframe)
			histb_vdec_write_h264_weights(slice_msg, pred_weights, 1,
						       references_l1);
	}
	if (slice_index) {
		__le32 *previous = histb_vdec_msg_slot(
			ctx, HISTB_VDEC_SLICE_MSG_SLOT + slice_index - 1);

		previous[44] = cpu_to_le32(slice->first_mb_in_slice - 1);
		previous[63] = cpu_to_le32(lower_32_bits(slice_dma));
	}
	slice_msg[63] = 0;

	return 0;
}

static bool histb_vdec_take_job(struct histb_vdec_dev *vdec,
				struct histb_vdec_ctx *ctx)
{
	unsigned long flags;
	bool taken = false;

	spin_lock_irqsave(&vdec->irqlock, flags);
	if (vdec->curr_ctx == ctx) {
		vdec->curr_ctx = NULL;
		vdec->phase = HISTB_VDEC_PHASE_IDLE;
		vdec->job_cancelled = false;
		taken = true;
	}
	spin_unlock_irqrestore(&vdec->irqlock, flags);

	return taken;
}

static bool histb_vdec_take_job_in_phase(struct histb_vdec_dev *vdec,
					  struct histb_vdec_ctx *ctx,
					  enum histb_vdec_job_phase phase)
{
	unsigned long flags;
	bool valid = false;

	spin_lock_irqsave(&vdec->irqlock, flags);
	if (vdec->curr_ctx == ctx) {
		valid = vdec->phase == phase && !vdec->job_cancelled;
		vdec->curr_ctx = NULL;
		vdec->phase = HISTB_VDEC_PHASE_IDLE;
		vdec->job_cancelled = false;
	}
	spin_unlock_irqrestore(&vdec->irqlock, flags);

	return valid;
}

static int histb_vdec_pulse_crg_reset(struct histb_vdec_dev *vdec,
				      unsigned int reg, u32 request,
				      u32 ack, u32 extra_mask,
				      u32 extra_value)
{
	u32 status;
	int clear_ret, ret;

	ret = regmap_update_bits(vdec->crg, reg, request | extra_mask,
				 request | extra_value);
	if (ret)
		return ret;

	/* The CV200 HAL holds each reset request for 30 us before testing ACK. */
	udelay(30);
	ret = regmap_read_poll_timeout(vdec->crg, HISTB_CRG_RESET_STATUS,
				       status, status & ack, 30,
				       HISTB_CRG_RESET_TIMEOUT_US);
	clear_ret = regmap_update_bits(vdec->crg, reg, request | extra_mask, 0);
	if (ret)
		dev_err(vdec->dev,
			"decoder reset request %#x did not complete: status=%08x\n",
			request, status);

	return ret ?: clear_ret;
}

static bool histb_vdec_job_in_phase(struct histb_vdec_dev *vdec,
				    struct histb_vdec_ctx *ctx,
				    enum histb_vdec_job_phase phase)
{
	unsigned long flags;
	bool active;

	spin_lock_irqsave(&vdec->irqlock, flags);
	active = vdec->curr_ctx == ctx && vdec->phase == phase &&
		 !vdec->job_cancelled;
	spin_unlock_irqrestore(&vdec->irqlock, flags);

	return active;
}

static int histb_vdec_switch_job_phase(struct histb_vdec_dev *vdec,
				       struct histb_vdec_ctx *ctx,
				       enum histb_vdec_job_phase from,
				       enum histb_vdec_job_phase to)
{
	unsigned long flags;
	int ret = -ECANCELED;

	spin_lock_irqsave(&vdec->irqlock, flags);
	if (vdec->curr_ctx == ctx && vdec->phase == from &&
	    !vdec->job_cancelled) {
		vdec->phase = to;
		ret = 0;
	}
	spin_unlock_irqrestore(&vdec->irqlock, flags);

	return ret;
}

static int histb_vdec_reset_vc1_bpd(struct histb_vdec_dev *vdec)
{
	void __iomem *bpd = vdec->regs + HISTB_VC1_BPD_OFFSET;
	int ret;

	writel(0, bpd + HISTB_VC1_BPD_START);
	ret = histb_vdec_pulse_crg_reset(vdec, HISTB_CRG_BPD_CLOCK,
					 HISTB_CRG_BPD_RESET,
					 HISTB_CRG_BPD_RESET_OK, 0, 0);
	if (ret)
		return ret;

	/* Matching BPD_Reset masks every source except completion. */
	writel(~HISTB_VC1_BPD_STATE_DONE, bpd + HISTB_VC1_BPD_INT_MASK);
	/* Matching BPD_CfgReg clears stale state with the same two writes. */
	writel(~0U, bpd + HISTB_VC1_BPD_INT_STATE);
	writel(~HISTB_VC1_BPD_STATE_DONE, bpd + HISTB_VC1_BPD_INT_STATE);
	return 0;
}

static int histb_vdec_run_vc1_bpd(struct histb_vdec_ctx *ctx,
				  const struct histb_vc1_bpd_regs *regs,
				  u32 available_bits,
				  struct histb_vc1_bpd_result *result)
{
	struct histb_vdec_dev *vdec = ctx->vdec;
	void __iomem *bpd = vdec->regs + HISTB_VC1_BPD_OFFSET;
	u64 start_ns;
	u32 out0, out1, state;
	unsigned int i;
	int reset_ret, ret;

	if (!regs || !available_bits || !result)
		return -EINVAL;
	mutex_lock(&vdec->launch_lock);
	if (!histb_vdec_job_in_phase(vdec, ctx, HISTB_VDEC_PHASE_BPD)) {
		mutex_unlock(&vdec->launch_lock);
		return -ECANCELED;
	}
	ret = histb_vdec_reset_vc1_bpd(vdec);
	if (ret) {
		mutex_unlock(&vdec->launch_lock);
		return ret;
	}
	for (i = 0; i < HISTB_VC1_BPD_CFG_WORDS; i++)
		writel(regs->cfg[i], bpd + HISTB_VC1_BPD_CFG0 + i * sizeof(u32));

	wmb();
	writel(0, bpd + HISTB_VC1_BPD_START);
	writel(1, bpd + HISTB_VC1_BPD_START);
	writel(0, bpd + HISTB_VC1_BPD_START);
	mutex_unlock(&vdec->launch_lock);
	start_ns = ktime_get_ns();
	for (;;) {
		if (!histb_vdec_job_in_phase(vdec, ctx, HISTB_VDEC_PHASE_BPD)) {
			ret = -ECANCELED;
			break;
		}
		state = readl(bpd + HISTB_VC1_BPD_STATE);
		if (state & HISTB_VC1_BPD_STATE_ERROR) {
			ret = -EIO;
			break;
		}
		if (state & HISTB_VC1_BPD_STATE_DONE) {
			out0 = readl(bpd + HISTB_VC1_BPD_OUT0);
			out1 = readl(bpd + HISTB_VC1_BPD_OUT1);
			ret = histb_vc1_parse_bpd_result(state, out0, out1,
							 available_bits, result);
			if (ret == HISTB_VC1_OK)
				return 0;
			ret = ret == HISTB_VC1_HW_ERROR ? -EIO : -EINVAL;
			break;
		}
		if (ktime_get_ns() - start_ns >=
		    HISTB_VDEC_VC1_BPD_TIMEOUT_MS * NSEC_PER_MSEC) {
			ret = -ETIMEDOUT;
			break;
		}
		usleep_range(50, 100);
	}

	/* BPD setup owns reset until its synchronous poll has unwound. */
	mutex_lock(&vdec->launch_lock);
	reset_ret = histb_vdec_reset_vc1_bpd(vdec);
	mutex_unlock(&vdec->launch_lock);
	if (reset_ret) {
		dev_err(vdec->dev, "failed to reset VC-1 BPD after %d: %d\n",
			ret, reset_ret);
		ret = reset_ret;
	}
	return ret;
}

static int histb_vdec_reset_engine(struct histb_vdec_dev *vdec)
{
	u32 scd_int_mask;
	int err, ret = 0;

	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(0, vdec->regs + HISTB_VDEC_START);
	/* Return the SMMU control register to its baseline before reset. */
	histb_vdec_program_smmu(vdec, false);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_MASK_NS);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_S);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_NS);
	/*
	 * The vendor recovery sequence resets SCD before the decode engine and
	 * waits for the CRG acknowledgment of every domain.  The generic reset
	 * controller only toggles the request bits, which is not sufficient after
	 * a stalled VDH transaction.
	 */
	writel(3, vdec->regs + HISTB_VDEC_SCD_RESET_CLOCK);
	scd_int_mask = readl(vdec->regs + HISTB_VDEC_SCD_INT_MASK);
	err = histb_vdec_pulse_crg_reset(vdec, HISTB_CRG_VDH_CLOCK,
					 HISTB_CRG_SCD_RESET,
					 HISTB_CRG_SCD_RESET_OK, 0, 0);
	if (err && !ret)
		ret = err;
	writel((readl(vdec->regs + HISTB_VDEC_SCD_INT_MASK) & ~BIT(0)) |
	       (scd_int_mask & BIT(0)),
	       vdec->regs + HISTB_VDEC_SCD_INT_MASK);
	writel(2, vdec->regs + HISTB_VDEC_SCD_RESET_CLOCK);

	err = histb_vdec_pulse_crg_reset(vdec, HISTB_CRG_VDH_CLOCK,
					 HISTB_CRG_MFD_RESET,
					 HISTB_CRG_MFD_RESET_OK,
					 HISTB_CRG_VDH_CLK_SEL,
					 FIELD_PREP(HISTB_CRG_VDH_CLK_SEL, 2));
	if (err && !ret)
		ret = err;
	err = histb_vdec_pulse_crg_reset(vdec, HISTB_CRG_BPD_CLOCK,
					 HISTB_CRG_BPD_RESET,
					 HISTB_CRG_BPD_RESET_OK, 0, 0);
	if (err && !ret)
		ret = err;
	err = histb_vdec_pulse_crg_reset(vdec, HISTB_CRG_VDH_CLOCK,
					 HISTB_CRG_VDH_RESET,
					 HISTB_CRG_VDH_RESET_OK,
					 HISTB_CRG_VDH_CLK_SEL,
					 FIELD_PREP(HISTB_CRG_VDH_CLK_SEL, 2));
	if (err && !ret)
		ret = err;
	err = histb_vdec_configure_vdh_clock(vdec);
	if (err && !ret)
		ret = err;

	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_MASK_NS);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_S);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_NS);
	vdec->avsp_loaded = false;
	if (ret)
		dev_err(vdec->dev, "failed to reset decoder pipeline: %d\n", ret);

	return ret;
}

static int histb_vdec_reset_vdh_domain(struct histb_vdec_dev *vdec)
{
	int ret;

	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(0, vdec->regs + HISTB_VDEC_START);
	histb_vdec_program_smmu(vdec, false);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_MASK_NS);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_S);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_NS);

	ret = histb_vdec_pulse_crg_reset(vdec, HISTB_CRG_VDH_CLOCK,
					 HISTB_CRG_VDH_RESET,
					 HISTB_CRG_VDH_RESET_OK,
					 HISTB_CRG_VDH_CLK_SEL,
					 FIELD_PREP(HISTB_CRG_VDH_CLK_SEL, 2));
	if (!ret)
		ret = histb_vdec_configure_vdh_clock(vdec);
	vdec->avsp_loaded = false;

	return ret;
}

static void histb_vdec_fill_capture_buffer(struct histb_vdec_ctx *ctx,
					   struct vb2_v4l2_buffer *src,
					   struct vb2_v4l2_buffer *dst,
					   enum vb2_buffer_state state,
					   u32 frame_flags)
{
	v4l2_m2m_buf_copy_metadata(src, dst, true);
	if (state == VB2_BUF_STATE_DONE) {
		vb2_set_plane_payload(&dst->vb2_buf, 0, ctx->dst.pix.sizeimage);
		dst->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
				V4L2_BUF_FLAG_BFRAME);
		if (frame_flags & V4L2_H264_DECODE_PARAM_FLAG_BFRAME)
			dst->flags |= V4L2_BUF_FLAG_BFRAME;
		else if (frame_flags & V4L2_H264_DECODE_PARAM_FLAG_PFRAME)
			dst->flags |= V4L2_BUF_FLAG_PFRAME;
		else
			dst->flags |= V4L2_BUF_FLAG_KEYFRAME;
	} else {
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
	}
}

static void histb_vdec_reset_h264_slices(struct histb_vdec_ctx *ctx)
{
	ctx->h264_partial = false;
	ctx->h264_new_frame = true;
	ctx->h264_last_slice = true;
	ctx->h264_first_mb = 0;
	ctx->h264_last_first_mb = 0;
	ctx->h264_stream_bytes = 0;
	ctx->h264_slices = 0;
	ctx->h264_controls_valid = false;
	memset(ctx->h264_picture_message, 0,
	       sizeof(ctx->h264_picture_message));
}

static void histb_vdec_reset_h264_field_pair(struct histb_vdec_ctx *ctx)
{
	ctx->h264_pending_field = NULL;
	ctx->h264_pending_timestamp = 0;
	ctx->h264_pending_sequence = 0;
	ctx->h264_pending_frame_num = 0;
	ctx->h264_pending_width_mbs = 0;
	ctx->h264_pending_height_map_units = 0;
	ctx->h264_field_frame_num = 0;
	ctx->h264_field_width_mbs = 0;
	ctx->h264_field_height_map_units = 0;
	ctx->h264_pending_bottom = false;
	ctx->h264_pending_failed = false;
	ctx->h264_field_picture = false;
	ctx->h264_bottom_field = false;
	ctx->h264_second_field = false;
	ctx->h264_field_transaction = false;
	ctx->h264_new_surface = true;
}

static int histb_vdec_begin_h264_picture(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded, u64 timestamp,
		const struct v4l2_ctrl_h264_decode_params *decode,
		const struct v4l2_ctrl_h264_sps *sps)
{
	bool field_pic = decode->flags &
		V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC;
	bool bottom_field = decode->flags &
		V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD;
	u16 width_mbs = sps->pic_width_in_mbs_minus1 + 1;
	u16 height_map_units = sps->pic_height_in_map_units_minus1 + 1;
	bool second_field = ctx->h264_pending_field && field_pic &&
		ctx->h264_pending_field == decoded &&
		ctx->h264_pending_timestamp == timestamp &&
		ctx->h264_pending_frame_num == decode->frame_num &&
		ctx->h264_pending_bottom != bottom_field &&
		ctx->h264_pending_width_mbs == width_mbs &&
		ctx->h264_pending_height_map_units == height_map_units;

	if (ctx->h264_pending_field && !second_field) {
		ctx->dst.sequence++;
		histb_vdec_reset_h264_field_pair(ctx);
	}

	ctx->h264_field_picture = field_pic;
	ctx->h264_bottom_field = bottom_field;
	ctx->h264_second_field = second_field;
	ctx->h264_new_surface = !second_field;
	ctx->h264_field_frame_num = decode->frame_num;
	ctx->h264_field_width_mbs = width_mbs;
	ctx->h264_field_height_map_units = height_map_units;
	ctx->h264_field_transaction = true;
	if (second_field && ctx->h264_pending_failed)
		return -EIO;
	if (!second_field)
		decoded->error_tainted = false;
	return 0;
}

static void histb_vdec_finish_h264_field_transaction(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		struct vb2_v4l2_buffer *dst,
		enum vb2_buffer_state state)
{
	if (!ctx->h264_field_transaction) {
		if (state != VB2_BUF_STATE_DONE && ctx->h264_pending_field) {
			dst->sequence = ctx->h264_pending_sequence;
			ctx->dst.sequence++;
			histb_vdec_reset_h264_field_pair(ctx);
		} else {
			dst->sequence = ctx->dst.sequence++;
		}
		return;
	}

	if (!ctx->h264_field_picture) {
		dst->sequence = ctx->dst.sequence++;
		histb_vdec_reset_h264_field_pair(ctx);
		return;
	}

	if (ctx->h264_second_field) {
		dst->sequence = ctx->h264_pending_sequence;
		ctx->dst.sequence++;
		histb_vdec_reset_h264_field_pair(ctx);
		return;
	}

	ctx->h264_pending_field = decoded;
	ctx->h264_pending_timestamp = decoded->base.vb.vb2_buf.timestamp;
	ctx->h264_pending_sequence = ctx->dst.sequence;
	ctx->h264_pending_frame_num = ctx->h264_field_frame_num;
	ctx->h264_pending_width_mbs = ctx->h264_field_width_mbs;
	ctx->h264_pending_height_map_units = ctx->h264_field_height_map_units;
	ctx->h264_pending_bottom = ctx->h264_bottom_field;
	ctx->h264_pending_failed = state != VB2_BUF_STATE_DONE;
	dst->sequence = ctx->h264_pending_sequence;
	ctx->h264_field_transaction = false;
}

static void histb_vdec_prepare_capture_buffer(struct histb_vdec_ctx *ctx,
					      struct vb2_v4l2_buffer *src,
					      struct vb2_v4l2_buffer *dst,
					      enum vb2_buffer_state state,
					      u32 frame_flags)
{
	src->sequence = ctx->src.sequence++;
	histb_vdec_fill_capture_buffer(ctx, src, dst, state, frame_flags);
}

static void histb_vdec_complete_capture_buffer(struct histb_vdec_ctx *ctx,
		struct vb2_v4l2_buffer *dst,
		enum vb2_buffer_state state, bool last)
{
	dst->sequence = ctx->dst.sequence++;
	if (last && state == VB2_BUF_STATE_DONE) {
		v4l2_m2m_last_buffer_done(ctx->fh.m2m_ctx, dst);
		v4l2_event_queue_fh(&ctx->fh, &histb_vdec_eos_event);
	} else {
		v4l2_m2m_buf_done(dst, state);
	}
}

static void histb_vdec_complete_empty_last(struct histb_vdec_ctx *ctx)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *dst;
	unsigned int i;

	dst = v4l2_m2m_dst_buf_remove(m2m_ctx);
	if (!dst) {
		m2m_ctx->next_buf_last = true;
		return;
	}
	for (i = 0; i < dst->vb2_buf.num_planes; i++)
		vb2_set_plane_payload(&dst->vb2_buf, i, 0);
	histb_vdec_complete_capture_buffer(ctx, dst, VB2_BUF_STATE_DONE, true);
}

static void histb_vdec_return_mpeg4_display(struct histb_vdec_ctx *ctx,
		enum vb2_buffer_state state, bool last)
{
	struct vb2_v4l2_buffer *dst = ctx->mpeg4_display_pending;

	if (WARN_ON(ctx->mpeg4_display_state.pending != !!dst))
		ctx->mpeg4_display_state.pending = !!dst;
	if (!dst)
		return;
	ctx->mpeg4_display_pending = NULL;
	ctx->mpeg4_display_state.pending = 0;
	if (state != VB2_BUF_STATE_DONE)
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
	histb_vdec_complete_capture_buffer(ctx, dst, state, last);
}

static void histb_vdec_return_vc1_display(struct histb_vdec_ctx *ctx,
		enum vb2_buffer_state state, bool last)
{
	struct vb2_v4l2_buffer *dst = ctx->vc1_display_pending;

	if (ctx->vc1_display_state == HISTB_VDEC_VC1_DISPLAY_NONE)
		return;
	if (ctx->vc1_display_state == HISTB_VDEC_VC1_DISPLAY_DROPPED) {
		WARN_ON(dst);
		ctx->vc1_display_state = HISTB_VDEC_VC1_DISPLAY_NONE;
		return;
	}
	if (WARN_ON(!dst)) {
		ctx->vc1_display_state = HISTB_VDEC_VC1_DISPLAY_NONE;
		return;
	}
	ctx->vc1_display_pending = NULL;
	ctx->vc1_display_state = HISTB_VDEC_VC1_DISPLAY_NONE;
	if (state != VB2_BUF_STATE_DONE)
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
	histb_vdec_complete_capture_buffer(ctx, dst, state, last);
}

static void histb_vdec_fail_mpeg4_state_locked(struct histb_vdec_ctx *ctx)
{
	lockdep_assert_held(&ctx->mpeg4_lock);
	histb_vdec_return_mpeg4_display(ctx, VB2_BUF_STATE_ERROR, false);
	histb_vdec_reset_mpeg4_state(ctx);
}

static void histb_vdec_fail_mpeg4_state(struct histb_vdec_ctx *ctx)
{
	mutex_lock(&ctx->mpeg4_lock);
	histb_vdec_fail_mpeg4_state_locked(ctx);
	mutex_unlock(&ctx->mpeg4_lock);
}

static void histb_vdec_reset_vc1_au(struct histb_vdec_ctx *ctx)
{
	histb_vc1_field_transaction_reset(&ctx->vc1_field_transaction);
	ctx->vc1_au_src = NULL;
	ctx->vc1_au_bytes = 0;
	ctx->vc1_au_offset = 0;
	ctx->vc1_pending_next_offset = 0;
	ctx->vc1_au_active = false;
	ctx->vc1_pending_source_done = false;
}

static void histb_vdec_reset_vc1_state_locked(struct histb_vdec_ctx *ctx)
{
	unsigned int i;

	histb_vdec_return_vc1_display(ctx, VB2_BUF_STATE_ERROR, false);
	for (i = 0; i < ARRAY_SIZE(ctx->vc1_ref); i++)
		histb_vdec_free_vc1_anchor(ctx, &ctx->vc1_ref[i]);
	histb_vdec_reset_vc1_au(ctx);
	memset(&ctx->vc1_sequence, 0, sizeof(ctx->vc1_sequence));
	memset(&ctx->vc1_entry, 0, sizeof(ctx->vc1_entry));
	memset(&ctx->vc1_smp_sequence, 0, sizeof(ctx->vc1_smp_sequence));
	memset(&ctx->vc1_pending_picture, 0,
	       sizeof(ctx->vc1_pending_picture));
	ctx->vc1_annex_l = false;
	ctx->vc1_smp_sequence_valid = false;
	ctx->vc1_smp_rounding = 0;
	ctx->vc1_smp_res_pic = 0;
	ctx->vc1_need_intra = false;
	histb_vdec_discard_vc1_current_locked(ctx, false);
}

static void histb_vdec_discard_vc1_current(struct histb_vdec_ctx *ctx,
					   bool fatal)
{
	mutex_lock(&ctx->vc1_lock);
	histb_vdec_discard_vc1_current_locked(ctx, fatal);
	if (fatal)
		histb_vdec_reset_vc1_au(ctx);
	mutex_unlock(&ctx->vc1_lock);
}

static void histb_vdec_reset_vc1_state(struct histb_vdec_ctx *ctx)
{
	mutex_lock(&ctx->vc1_lock);
	histb_vdec_reset_vc1_state_locked(ctx);
	mutex_unlock(&ctx->vc1_lock);
}

static void histb_vdec_error_ready_vc1_buffers(struct histb_vdec_ctx *ctx,
		struct vb2_v4l2_buffer *src, struct vb2_v4l2_buffer *dst)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *removed;

	if (src) {
		removed = v4l2_m2m_src_buf_remove_by_idx(m2m_ctx,
							 src->vb2_buf.index);
		if (removed)
			v4l2_m2m_buf_done(removed, VB2_BUF_STATE_ERROR);
	}
	if (dst) {
		removed = v4l2_m2m_dst_buf_remove_by_idx(m2m_ctx,
							 dst->vb2_buf.index);
		if (removed)
			v4l2_m2m_buf_done(removed, VB2_BUF_STATE_ERROR);
	}
	histb_vdec_return_vc1_display(ctx, VB2_BUF_STATE_ERROR, false);
	histb_vdec_reset_vc1_au(ctx);
}

static void histb_vdec_finish_vc1_job_no_pm(struct histb_vdec_ctx *ctx,
					    bool capture_ok)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct histb_vc1_display_state display_state;
	struct histb_vc1_display_plan display_plan;
	struct vb2_v4l2_buffer *src, *dst, *removed;
	bool source_done = ctx->vc1_pending_source_done;
	bool draining, empty_last = false, pending_dropped;
	u8 ptype = ctx->vc1_pending_picture.ptype;
	int ret;

	src = v4l2_m2m_next_src_buf(m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(m2m_ctx);
	if (WARN_ON(!src || !dst)) {
		histb_vdec_error_ready_vc1_buffers(
			ctx, src ?: ctx->vc1_au_src, dst);
		v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
		return;
	}
	draining = source_done &&
		v4l2_m2m_is_last_draining_src_buf(m2m_ctx, src);
	display_state.pending = ctx->vc1_display_state !=
		HISTB_VDEC_VC1_DISPLAY_NONE;
	ret = histb_vc1_plan_display(&display_state, ptype,
				      !display_state.pending, draining,
				      &display_plan);
	if (WARN_ON(ret != HISTB_VC1_OK)) {
		histb_vdec_error_ready_vc1_buffers(ctx, src, dst);
		histb_vdec_reset_vc1_au(ctx);
		v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
		return;
	}

	removed = v4l2_m2m_dst_buf_remove(m2m_ctx);
	if (WARN_ON(removed != dst)) {
		if (removed)
			v4l2_m2m_buf_done(removed, VB2_BUF_STATE_ERROR);
		histb_vdec_error_ready_vc1_buffers(ctx, src, dst);
		v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
		return;
	}
	/* Copy current source metadata exactly once before delaying display. */
	histb_vdec_fill_capture_buffer(ctx, src, dst,
				       capture_ok ? VB2_BUF_STATE_DONE :
				       VB2_BUF_STATE_ERROR, ctx->frame_flags);
	if (source_done) {
		removed = v4l2_m2m_src_buf_remove(m2m_ctx);
		if (WARN_ON(removed != src)) {
			if (removed)
				v4l2_m2m_buf_done(removed, VB2_BUF_STATE_ERROR);
			histb_vdec_complete_capture_buffer(ctx, dst,
							   VB2_BUF_STATE_ERROR, false);
			histb_vdec_error_ready_vc1_buffers(ctx, src, dst);
			v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
			return;
		}
		src->sequence = ctx->src.sequence++;
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
		histb_vdec_reset_vc1_au(ctx);
	}

	pending_dropped = ctx->vc1_display_state ==
		HISTB_VDEC_VC1_DISPLAY_DROPPED;
	if (display_plan.current_before_pending && display_plan.release_current) {
		histb_vdec_complete_capture_buffer(
			ctx, dst, capture_ok ? VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR,
			capture_ok && display_plan.current_last);
		if (!capture_ok && display_plan.current_last)
			empty_last = true;
	}
	if (display_plan.release_pending) {
		histb_vdec_return_vc1_display(
			ctx, VB2_BUF_STATE_DONE, display_plan.pending_last);
		if (pending_dropped && display_plan.pending_last)
			empty_last = true;
	}
	if (!display_plan.current_before_pending &&
	    display_plan.release_current) {
		histb_vdec_complete_capture_buffer(
			ctx, dst, capture_ok ? VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR,
			capture_ok && display_plan.current_last);
		if (!capture_ok && display_plan.current_last)
			empty_last = true;
	}
	if (display_plan.hold_current) {
		if (capture_ok) {
			ctx->vc1_display_pending = dst;
			ctx->vc1_display_state = HISTB_VDEC_VC1_DISPLAY_BUFFER;
		} else {
			histb_vdec_complete_capture_buffer(ctx, dst,
							   VB2_BUF_STATE_ERROR, false);
			ctx->vc1_display_pending = NULL;
			ctx->vc1_display_state = HISTB_VDEC_VC1_DISPLAY_DROPPED;
		}
	}
	if (empty_last)
		histb_vdec_complete_empty_last(ctx);
	v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
}

static void histb_vdec_finish_vc1_job(struct histb_vdec_ctx *ctx,
				       bool capture_ok)
{
	pm_runtime_mark_last_busy(ctx->vdec->dev);
	pm_runtime_put_autosuspend(ctx->vdec->dev);
	histb_vdec_finish_vc1_job_no_pm(ctx, capture_ok);
}

static int histb_vdec_advance_mpeg4_au(struct histb_vdec_ctx *ctx)
{
	struct histb_mpeg4_parser parser;
	struct histb_mpeg4_frame frame;
	u32 next_offset;
	u32 offset = ctx->mpeg4_pending_next_offset;
	int ret;

	if (!ctx->mpeg4_au_active || offset > ctx->mpeg4_au_bytes)
		return -EINVAL;
	while (offset < ctx->mpeg4_au_bytes) {
		parser = ctx->mpeg4_parser;
		ret = histb_mpeg4_parse_frame_at(&parser, ctx->bitstream.cpu,
						 ctx->mpeg4_au_bytes, offset,
						 &frame, &next_offset);
		if (ret == HISTB_MPEG4_NO_VOP) {
			ctx->mpeg4_parser = parser;
			offset = ctx->mpeg4_au_bytes;
			break;
		}
		if (ret == HISTB_MPEG4_END_OF_STREAM) {
			ctx->mpeg4_parser = parser;
			ctx->mpeg4_au_offset = next_offset;
			ctx->mpeg4_pending_source_done = true;
			histb_vdec_begin_mpeg4_implicit_drain_locked(
				ctx, ctx->mpeg4_au_src);
			return 0;
		}
		if (ret)
			return histb_vdec_mpeg4_result_to_errno(ret);
		if (next_offset <= offset || next_offset > ctx->mpeg4_au_bytes)
			return -EINVAL;
		if (frame.vop.coded &&
		    frame.vop.coding_type != HISTB_MPEG4_N_VOP) {
			ctx->mpeg4_au_offset = offset;
			ctx->mpeg4_pending_source_done = false;
			return 0;
		}
		ctx->mpeg4_parser = parser;
		offset = next_offset;
	}

	ctx->mpeg4_au_offset = ctx->mpeg4_au_bytes;
	ctx->mpeg4_pending_source_done = true;
	return 0;
}

static void histb_vdec_finish_mpeg4_job_no_pm(struct histb_vdec_ctx *ctx,
		const struct histb_mpeg4_display_plan *plan, bool pending_after)
{
	struct vb2_v4l2_buffer *src, *dst, *removed_src, *removed_dst;
	bool source_done = ctx->mpeg4_pending_source_done;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (WARN_ON(!src || !dst)) {
		v4l2_m2m_job_finish(ctx->vdec->m2m_dev, ctx->fh.m2m_ctx);
		return;
	}
	removed_dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	if (WARN_ON(removed_dst != dst)) {
		if (removed_dst)
			v4l2_m2m_buf_done(removed_dst, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(ctx->vdec->m2m_dev, ctx->fh.m2m_ctx);
		return;
	}

	histb_vdec_fill_capture_buffer(ctx, src, dst, VB2_BUF_STATE_DONE,
				       ctx->frame_flags);
	if (source_done) {
		removed_src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		if (WARN_ON(removed_src != src)) {
			if (removed_src)
				v4l2_m2m_buf_done(removed_src, VB2_BUF_STATE_ERROR);
			histb_vdec_complete_capture_buffer(ctx, dst,
							   VB2_BUF_STATE_ERROR,
							   false);
			v4l2_m2m_job_finish(ctx->vdec->m2m_dev,
						    ctx->fh.m2m_ctx);
			return;
		}
		src->sequence = ctx->src.sequence++;
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
		histb_vdec_reset_mpeg4_au(ctx);
	}

	if (plan->current_before_pending && plan->release_current)
		histb_vdec_complete_capture_buffer(ctx, dst, VB2_BUF_STATE_DONE,
						  plan->current_last);
	if (plan->release_pending)
		histb_vdec_return_mpeg4_display(ctx, VB2_BUF_STATE_DONE,
						plan->pending_last);
	if (!plan->current_before_pending && plan->release_current)
		histb_vdec_complete_capture_buffer(ctx, dst, VB2_BUF_STATE_DONE,
						  plan->current_last);
	if (plan->hold_current) {
		ctx->mpeg4_display_pending = dst;
		ctx->mpeg4_display_state.pending = 1;
	}
	WARN_ON(ctx->mpeg4_display_state.pending != pending_after ||
		ctx->mpeg4_display_state.pending !=
		!!ctx->mpeg4_display_pending);

	v4l2_m2m_job_finish(ctx->vdec->m2m_dev, ctx->fh.m2m_ctx);
}

static void histb_vdec_finish_mpeg4_job(struct histb_vdec_ctx *ctx,
		const struct histb_mpeg4_display_plan *plan, bool pending_after)
{
	pm_runtime_mark_last_busy(ctx->vdec->dev);
	pm_runtime_put_autosuspend(ctx->vdec->dev);
	histb_vdec_finish_mpeg4_job_no_pm(ctx, plan, pending_after);
}

static void histb_vdec_finish_mpeg4_au_no_capture(struct histb_vdec_ctx *ctx)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct histb_mpeg4_display_plan plan;
	struct histb_mpeg4_display_state state;
	struct vb2_v4l2_buffer *src, *removed_src;
	bool draining;
	int ret;

	lockdep_assert_held(&ctx->mpeg4_lock);
	src = v4l2_m2m_next_src_buf(m2m_ctx);
	if (WARN_ON(!src || src != ctx->mpeg4_au_src)) {
		v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
		return;
	}
	draining = v4l2_m2m_is_last_draining_src_buf(m2m_ctx, src);
	removed_src = v4l2_m2m_src_buf_remove(m2m_ctx);
	if (WARN_ON(removed_src != src)) {
		if (removed_src)
			v4l2_m2m_buf_done(removed_src, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
		return;
	}
	src->sequence = ctx->src.sequence++;
	v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
	histb_vdec_reset_mpeg4_au(ctx);

	if (draining && ctx->mpeg4_display_pending) {
		state = ctx->mpeg4_display_state;
		ret = histb_mpeg4_plan_drain(&state, &plan);
		if (!ret && plan.release_pending && plan.pending_last)
			histb_vdec_return_mpeg4_display(ctx, VB2_BUF_STATE_DONE,
							true);
		else {
			histb_vdec_fail_mpeg4_state_locked(ctx);
			histb_vdec_complete_empty_last(ctx);
		}
	} else if (draining) {
		histb_vdec_complete_empty_last(ctx);
	}
	v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
}

static void histb_vdec_finish_vc1_au_no_capture(struct histb_vdec_ctx *ctx)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src, *removed;
	bool draining;

	src = v4l2_m2m_next_src_buf(m2m_ctx);
	if (WARN_ON(!src || src != ctx->vc1_au_src)) {
		histb_vdec_error_ready_vc1_buffers(ctx, ctx->vc1_au_src, NULL);
		v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
		return;
	}
	draining = v4l2_m2m_is_last_draining_src_buf(m2m_ctx, src);
	removed = v4l2_m2m_src_buf_remove(m2m_ctx);
	if (WARN_ON(removed != src)) {
		if (removed)
			v4l2_m2m_buf_done(removed, VB2_BUF_STATE_ERROR);
		histb_vdec_error_ready_vc1_buffers(ctx, src, NULL);
		v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
		return;
	}
	src->sequence = ctx->src.sequence++;
	v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
	histb_vdec_reset_vc1_au(ctx);

	if (draining && ctx->vc1_display_state == HISTB_VDEC_VC1_DISPLAY_BUFFER) {
		histb_vdec_return_vc1_display(ctx, VB2_BUF_STATE_DONE, true);
	} else if (draining) {
		histb_vdec_return_vc1_display(ctx, VB2_BUF_STATE_DONE, false);
		histb_vdec_complete_empty_last(ctx);
	}
	v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
}

static void histb_vdec_finish_vc1_error_no_pm(struct histb_vdec_ctx *ctx)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src, *dst, *removed;
	bool draining;

	mutex_lock(&ctx->vc1_lock);
	src = v4l2_m2m_next_src_buf(m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(m2m_ctx);
	draining = src && v4l2_m2m_is_last_draining_src_buf(m2m_ctx, src);
	if (dst) {
		removed = v4l2_m2m_dst_buf_remove(m2m_ctx);
		if (removed) {
			if (src)
				histb_vdec_fill_capture_buffer(
					ctx, src, removed, VB2_BUF_STATE_ERROR,
					ctx->frame_flags);
			else
				vb2_set_plane_payload(&removed->vb2_buf, 0, 0);
			histb_vdec_complete_capture_buffer(
				ctx, removed, VB2_BUF_STATE_ERROR, false);
		}
	}
	if (src) {
		removed = v4l2_m2m_src_buf_remove(m2m_ctx);
		if (removed) {
			removed->sequence = ctx->src.sequence++;
			v4l2_m2m_buf_done(removed, VB2_BUF_STATE_ERROR);
		}
	}
	histb_vdec_reset_vc1_au(ctx);
	if (draining &&
	    ctx->vc1_display_state == HISTB_VDEC_VC1_DISPLAY_BUFFER) {
		histb_vdec_return_vc1_display(ctx, VB2_BUF_STATE_DONE, true);
	} else if (draining) {
		histb_vdec_return_vc1_display(ctx, VB2_BUF_STATE_DONE, false);
		histb_vdec_complete_empty_last(ctx);
	}
	mutex_unlock(&ctx->vc1_lock);
	v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
}

static void histb_vdec_finish_vc1_error(struct histb_vdec_ctx *ctx)
{
	pm_runtime_mark_last_busy(ctx->vdec->dev);
	pm_runtime_put_autosuspend(ctx->vdec->dev);
	histb_vdec_finish_vc1_error_no_pm(ctx);
}

static void histb_vdec_finish_job_no_pm(struct histb_vdec_ctx *ctx,
					enum vb2_buffer_state state)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src, *dst;
	struct histb_vdec_decoded_buffer *decoded;
	enum vb2_buffer_state decode_state = state;
	enum vb2_buffer_state capture_state;
	bool mpeg2_commit_reference;
	bool mpeg2_reference_started;
	bool draining;

	src = v4l2_m2m_next_src_buf(m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(m2m_ctx);
	if (WARN_ON(!src || !dst)) {
		v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
		return;
	}
	draining = v4l2_m2m_is_last_draining_src_buf(m2m_ctx, src);
	mpeg2_reference_started = ctx->mpeg2_reference_started;
	ctx->mpeg2_reference_started = false;
	mpeg2_commit_reference = ctx->mpeg2 && ctx->mpeg2_picture_valid &&
		(!ctx->mpeg2_field_picture || ctx->mpeg2_second_field);
	if (decode_state != VB2_BUF_STATE_DONE &&
	    ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE) {
		src->flags &= ~V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF;
		histb_vdec_reset_h264_slices(ctx);
		histb_vdec_reset_apc(ctx);
	}

	histb_vdec_prepare_capture_buffer(ctx, src, dst, decode_state,
					  ctx->frame_flags);
	decoded = histb_vdec_decoded_buffer(dst);
	if (decode_state != VB2_BUF_STATE_DONE)
		decoded->error_tainted = true;
	if (mpeg2_commit_reference &&
	    (decode_state == VB2_BUF_STATE_DONE || mpeg2_reference_started) &&
	    histb_vdec_commit_mpeg2_reference(ctx, decoded)) {
		decode_state = VB2_BUF_STATE_ERROR;
		decoded->error_tainted = true;
		histb_vdec_prepare_capture_buffer(ctx, src, dst, decode_state,
						  ctx->frame_flags);
	}
	capture_state = decoded->error_tainted ? VB2_BUF_STATE_ERROR :
		VB2_BUF_STATE_DONE;
	if (ctx->avs && capture_state != VB2_BUF_STATE_DONE)
		decoded->avs.valid = false;
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE)
		histb_vdec_finish_h264_field_transaction(ctx, decoded, dst,
								 capture_state);
	else if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG2_SLICE)
		histb_vdec_finish_mpeg2_field_transaction(ctx, decoded, dst,
							   decode_state,
							   mpeg2_reference_started);
	else
		dst->sequence = ctx->dst.sequence++;
	if (draining && decode_state == VB2_BUF_STATE_DONE) {
		dst->flags |= V4L2_BUF_FLAG_LAST;
		v4l2_m2m_mark_stopped(m2m_ctx);
	}
	v4l2_m2m_buf_done_and_job_finish_states(ctx->vdec->m2m_dev,
						 m2m_ctx, decode_state, capture_state);
	if (!draining)
		return;
	if (decode_state == VB2_BUF_STATE_DONE)
		v4l2_event_queue_fh(&ctx->fh, &histb_vdec_eos_event);
	else
		histb_vdec_complete_empty_last(ctx);
}

static void histb_vdec_finish_h264_slice(struct histb_vdec_ctx *ctx)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(m2m_ctx);
	bool draining;

	if (WARN_ON(!src || !dst)) {
		histb_vdec_reset_h264_slices(ctx);
		histb_vdec_reset_h264_field_pair(ctx);
		histb_vdec_reset_apc(ctx);
		v4l2_m2m_job_finish(ctx->vdec->m2m_dev, m2m_ctx);
		return;
	}
	draining = v4l2_m2m_is_last_draining_src_buf(m2m_ctx, src);
	if (draining) {
		histb_vdec_decoded_buffer(dst)->error_tainted = true;
		src->flags &= ~V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF;
		src->sequence = ctx->src.sequence++;
		dst->sequence = ctx->dst.sequence++;
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
		histb_vdec_reset_h264_slices(ctx);
		histb_vdec_reset_h264_field_pair(ctx);
		histb_vdec_reset_apc(ctx);
		v4l2_m2m_buf_done_and_job_finish(ctx->vdec->m2m_dev, m2m_ctx,
						      VB2_BUF_STATE_ERROR);
		histb_vdec_complete_empty_last(ctx);
		return;
	}
	if (ctx->h264_new_frame)
		v4l2_m2m_buf_copy_metadata(src, dst, true);
	src->sequence = ctx->src.sequence++;
	v4l2_m2m_buf_done_and_job_finish(ctx->vdec->m2m_dev, m2m_ctx,
					      VB2_BUF_STATE_DONE);
}

static void histb_vdec_finish_job(struct histb_vdec_ctx *ctx,
				  enum vb2_buffer_state state)
{
	pm_runtime_mark_last_busy(ctx->vdec->dev);
	pm_runtime_put_autosuspend(ctx->vdec->dev);
	histb_vdec_finish_job_no_pm(ctx, state);
}

static void histb_vdec_reset_vp8_state(struct histb_vdec_ctx *ctx)
{
	ctx->vp8_have_state = false;
	ctx->vp8_last_frame_type = 0;
	ctx->vp8_last_filter_type = 0;
	ctx->vp8_last_sharpness = 0;
	if (ctx->buffers[HISTB_VDEC_BUF_VP8_SEG].cpu)
		memset(ctx->buffers[HISTB_VDEC_BUF_VP8_SEG].cpu, 0,
		       ctx->buffers[HISTB_VDEC_BUF_VP8_SEG].size);
}

static void histb_vdec_discard_vp9_state(struct histb_vdec_ctx *ctx)
{
	ctx->vp9_pending_valid = false;
	ctx->vp9_pending_output = NULL;
	ctx->vp9_pending_refresh = 0;
	ctx->vp9_pending_logic_id = HISTB_VDEC_APC_INVALID;
	memset(ctx->vp9_pending_dpb, 0, sizeof(ctx->vp9_pending_dpb));
}

static void histb_vdec_reset_vp9_state(struct histb_vdec_ctx *ctx)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ctx->vp9_dpb); i++) {
		if (ctx->vp9_dpb[i]) {
			ctx->vp9_dpb[i]->vp9_dpb_valid = false;
			ctx->vp9_dpb[i]->vp9_logic_id =
				HISTB_VDEC_APC_INVALID;
		}
		ctx->vp9_dpb[i] = NULL;
	}
	ctx->vp9_have_state = false;
	ctx->vp9_last_frame_flags = 0;
	ctx->vp9_prev_width = 0;
	ctx->vp9_prev_height = 0;
	memset(&ctx->vp9_seg, 0, sizeof(ctx->vp9_seg));
	for (i = 0; i < ARRAY_SIZE(ctx->vp9_frame_ctx); i++)
		ctx->vp9_frame_ctx[i] = v4l2_vp9_default_probs;
	histb_vdec_discard_vp9_state(ctx);
	if (ctx->buffers[HISTB_VDEC_BUF_VP9_SEG].cpu)
		memset(ctx->buffers[HISTB_VDEC_BUF_VP9_SEG].cpu, 0,
		       ctx->buffers[HISTB_VDEC_BUF_VP9_SEG].size);
	if (ctx->buffers[HISTB_VDEC_BUF_VP9_SEG_WORK].cpu)
		memset(ctx->buffers[HISTB_VDEC_BUF_VP9_SEG_WORK].cpu, 0,
		       ctx->buffers[HISTB_VDEC_BUF_VP9_SEG_WORK].size);
	if (ctx->buffers[HISTB_VDEC_BUF_VP9_PMV].cpu)
		memset(ctx->buffers[HISTB_VDEC_BUF_VP9_PMV].cpu, 0,
		       ctx->buffers[HISTB_VDEC_BUF_VP9_PMV].size);
}

enum histb_vdec_vp9_count_offset {
	HISTB_VDEC_VP9_COUNT_PARTITION = 2304,
	HISTB_VDEC_VP9_COUNT_Y_MODE = 2368,
	HISTB_VDEC_VP9_COUNT_UV_MODE = 2408,
	HISTB_VDEC_VP9_COUNT_FILTER = 2508,
	HISTB_VDEC_VP9_COUNT_INTER_MODE = 2520,
	HISTB_VDEC_VP9_COUNT_INTRA_INTER = 2548,
	HISTB_VDEC_VP9_COUNT_COMP = 2556,
	HISTB_VDEC_VP9_COUNT_SINGLE_REF = 2566,
	HISTB_VDEC_VP9_COUNT_COMP_REF = 2586,
	HISTB_VDEC_VP9_COUNT_TX8 = 2596,
	HISTB_VDEC_VP9_COUNT_TX16 = 2600,
	HISTB_VDEC_VP9_COUNT_TX32 = 2606,
	HISTB_VDEC_VP9_COUNT_SKIP = 2614,
	HISTB_VDEC_VP9_COUNT_MV_JOINT = 2620,
	HISTB_VDEC_VP9_COUNT_SIGN = 2624,
	HISTB_VDEC_VP9_COUNT_CLASSES = 2628,
	HISTB_VDEC_VP9_COUNT_CLASS0 = 2650,
	HISTB_VDEC_VP9_COUNT_BITS = 2654,
	HISTB_VDEC_VP9_COUNT_CLASS0_FP = 2694,
	HISTB_VDEC_VP9_COUNT_FP = 2710,
	HISTB_VDEC_VP9_COUNT_CLASS0_HP = 2718,
	HISTB_VDEC_VP9_COUNT_HP = 2722,
};

static void histb_vdec_restore_vp9_counts(struct histb_vdec_ctx *ctx)
{
	struct histb_vdec_vp9_counts *storage = &ctx->vp9_counts;
	struct v4l2_vp9_frame_symbol_counts *counts = &storage->map;
	u32 *raw = ctx->buffers[HISTB_VDEC_BUF_VP9_COUNT].cpu;
	unsigned int tx, block, ref, band, context, index = 0;

	dma_rmb();
	for (tx = 0; tx < 4; tx++)
		for (block = 0; block < 2; block++)
			for (ref = 0; ref < 2; ref++)
				for (band = 0; band < 6; band++)
					for (context = 0; context < 6;
					     context++, index++) {
						u32 *packed = raw + index * 4;
						u32 value[4];

						memcpy(value, packed, sizeof(value));
						storage->eob[index] =
							value[0] & GENMASK(25, 0);
						packed[0] = (value[0] >> 26) |
							((value[1] & GENMASK(19, 0)) << 6);
						packed[1] = (value[1] >> 20) |
							((value[2] & GENMASK(13, 0)) << 12);
						packed[2] = (value[2] >> 14) |
							((value[3] & GENMASK(7, 0)) << 18);
						packed[3] = value[3] >> 8;
						counts->coeff[tx][block][ref][band][context] =
							(u32 (*)[3])packed;
						counts->eob[tx][block][ref][band][context][0] =
							&storage->eob[index];
						counts->eob[tx][block][ref][band][context][1] =
							&packed[3];
					}

	counts->partition = (u32 (*)[16][4])
		(raw + HISTB_VDEC_VP9_COUNT_PARTITION);
	counts->y_mode = (u32 (*)[4][10])(raw + HISTB_VDEC_VP9_COUNT_Y_MODE);
	counts->uv_mode = (u32 (*)[10][10])(raw + HISTB_VDEC_VP9_COUNT_UV_MODE);
	counts->filter = (u32 (*)[4][3])(raw + HISTB_VDEC_VP9_COUNT_FILTER);
	counts->mv_mode = (u32 (*)[7][4])
		(raw + HISTB_VDEC_VP9_COUNT_INTER_MODE);
	counts->intra_inter = (u32 (*)[4][2])
		(raw + HISTB_VDEC_VP9_COUNT_INTRA_INTER);
	counts->comp = (u32 (*)[5][2])(raw + HISTB_VDEC_VP9_COUNT_COMP);
	counts->single_ref = (u32 (*)[5][2][2])
		(raw + HISTB_VDEC_VP9_COUNT_SINGLE_REF);
	counts->comp_ref = (u32 (*)[5][2])
		(raw + HISTB_VDEC_VP9_COUNT_COMP_REF);
	counts->tx8p = (u32 (*)[2][2])(raw + HISTB_VDEC_VP9_COUNT_TX8);
	memcpy(storage->tx16[0], raw + HISTB_VDEC_VP9_COUNT_TX16,
	       3 * sizeof(u32));
	memcpy(storage->tx16[1], raw + HISTB_VDEC_VP9_COUNT_TX16 + 3,
	       3 * sizeof(u32));
	storage->tx16[0][3] = 0;
	storage->tx16[1][3] = 0;
	counts->tx16p = &storage->tx16;
	counts->tx32p = (u32 (*)[2][4])(raw + HISTB_VDEC_VP9_COUNT_TX32);
	counts->skip = (u32 (*)[3][2])(raw + HISTB_VDEC_VP9_COUNT_SKIP);
	counts->mv_joint = (u32 (*)[4])
		(raw + HISTB_VDEC_VP9_COUNT_MV_JOINT);
	counts->sign = (u32 (*)[2][2])(raw + HISTB_VDEC_VP9_COUNT_SIGN);
	counts->classes = (u32 (*)[2][11])
		(raw + HISTB_VDEC_VP9_COUNT_CLASSES);
	counts->class0 = (u32 (*)[2][2])(raw + HISTB_VDEC_VP9_COUNT_CLASS0);
	counts->bits = (u32 (*)[2][10][2])(raw + HISTB_VDEC_VP9_COUNT_BITS);
	counts->class0_fp = (u32 (*)[2][2][4])
		(raw + HISTB_VDEC_VP9_COUNT_CLASS0_FP);
	counts->fp = (u32 (*)[2][4])(raw + HISTB_VDEC_VP9_COUNT_FP);
	counts->class0_hp = (u32 (*)[2][2])
		(raw + HISTB_VDEC_VP9_COUNT_CLASS0_HP);
	counts->hp = (u32 (*)[2][2])(raw + HISTB_VDEC_VP9_COUNT_HP);
}

static void histb_vdec_adapt_vp9_context(struct histb_vdec_ctx *ctx)
{
	struct v4l2_ctrl_vp9_frame *frame = &ctx->vp9_pending_frame;
	struct v4l2_vp9_frame_context *probs;
	bool intra = frame->flags & (V4L2_VP9_FRAME_FLAG_KEY_FRAME |
				     V4L2_VP9_FRAME_FLAG_INTRA_ONLY);
	u8 index = frame->frame_context_idx;
	struct {
		u8 tx8[2][1];
		u8 tx16[2][2];
		u8 tx32[2][3];
		u8 skip[3];
	} tx_skip;

	if (!(frame->flags & V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX))
		return;
	if (frame->flags & V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE) {
		ctx->vp9_pending_frame_ctx[index] = ctx->vp9_pending_probs;
		return;
	}

	probs = &ctx->vp9_pending_frame_ctx[index];
	if (intra) {
		memcpy(tx_skip.tx8, ctx->vp9_pending_probs.tx8,
		       sizeof(tx_skip.tx8));
		memcpy(tx_skip.tx16, ctx->vp9_pending_probs.tx16,
		       sizeof(tx_skip.tx16));
		memcpy(tx_skip.tx32, ctx->vp9_pending_probs.tx32,
		       sizeof(tx_skip.tx32));
		memcpy(tx_skip.skip, ctx->vp9_pending_probs.skip,
		       sizeof(tx_skip.skip));
		memcpy(probs->tx8, tx_skip.tx8, sizeof(probs->tx8));
		memcpy(probs->tx16, tx_skip.tx16, sizeof(probs->tx16));
		memcpy(probs->tx32, tx_skip.tx32, sizeof(probs->tx32));
		memcpy(probs->skip, tx_skip.skip, sizeof(probs->skip));
	}

	histb_vdec_restore_vp9_counts(ctx);
	v4l2_vp9_adapt_coef_probs(probs, &ctx->vp9_counts.map,
				  !ctx->vp9_have_state ||
				  (ctx->vp9_last_frame_flags &
				   V4L2_VP9_FRAME_FLAG_KEY_FRAME), intra);
	if (!intra)
		v4l2_vp9_adapt_noncoef_probs(
			probs, &ctx->vp9_counts.map, frame->reference_mode,
			frame->interpolation_filter,
			ctx->vp9_pending_compressed_hdr.tx_mode, frame->flags);
}

static void histb_vdec_commit_vp9_state(struct histb_vdec_ctx *ctx)
{
	struct histb_vdec_dma_buffer tmp;
	unsigned int i;

	if (!ctx->vp9_pending_valid || !ctx->vp9_pending_output)
		return;
	histb_vdec_adapt_vp9_context(ctx);
	for (i = 0; i < ARRAY_SIZE(ctx->vp9_dpb); i++) {
		struct histb_vdec_decoded_buffer *old = ctx->vp9_dpb[i];
		unsigned int pending;

		if (!old)
			continue;
		for (pending = 0; pending < ARRAY_SIZE(ctx->vp9_pending_dpb);
		     pending++)
			if (ctx->vp9_pending_dpb[pending] == old)
				break;
		if (pending == ARRAY_SIZE(ctx->vp9_pending_dpb)) {
			old->vp9_dpb_valid = false;
			old->vp9_logic_id = HISTB_VDEC_APC_INVALID;
		}
	}
	memcpy(ctx->vp9_dpb, ctx->vp9_pending_dpb, sizeof(ctx->vp9_dpb));
	ctx->vp9_pending_output->vp9_logic_id = ctx->vp9_pending_logic_id;
	for (i = 0; i < ARRAY_SIZE(ctx->vp9_dpb); i++)
		if (ctx->vp9_dpb[i])
			ctx->vp9_dpb[i]->vp9_dpb_valid = true;
	if (!ctx->vp9_pending_refresh) {
		ctx->vp9_pending_output->vp9_dpb_valid = false;
		ctx->vp9_pending_output->vp9_logic_id =
			HISTB_VDEC_APC_INVALID;
	}
	ctx->vp9_pending_output->vp9_width = ctx->vp9_pending_width;
	ctx->vp9_pending_output->vp9_height = ctx->vp9_pending_height;
	memcpy(ctx->vp9_frame_ctx, ctx->vp9_pending_frame_ctx,
	       sizeof(ctx->vp9_frame_ctx));
	ctx->vp9_prev_width = ctx->vp9_pending_width;
	ctx->vp9_prev_height = ctx->vp9_pending_height;
	ctx->vp9_last_frame_flags = ctx->vp9_pending_frame.flags;
	ctx->vp9_seg = ctx->vp9_pending_frame.seg;
	ctx->vp9_have_state = true;

	/* The VDH wrote the work map; make it the committed map atomically. */
	tmp = ctx->buffers[HISTB_VDEC_BUF_VP9_SEG];
	ctx->buffers[HISTB_VDEC_BUF_VP9_SEG] =
		ctx->buffers[HISTB_VDEC_BUF_VP9_SEG_WORK];
	ctx->buffers[HISTB_VDEC_BUF_VP9_SEG_WORK] = tmp;
	histb_vdec_discard_vp9_state(ctx);
}

static void histb_vdec_commit_vp8_state(struct histb_vdec_ctx *ctx)
{
	ctx->vp8_last_frame_type = ctx->vp8_pending_frame_type;
	ctx->vp8_last_filter_type = ctx->vp8_pending_filter_type;
	ctx->vp8_last_sharpness = ctx->vp8_pending_sharpness;
	ctx->vp8_have_state = true;
}

static const char *histb_vdec_codec_name(const struct histb_vdec_ctx *ctx)
{
	if (ctx->avs)
		return "AVS";
	if (ctx->vc1)
		return "VC-1";
	if (ctx->vp9)
		return "VP9";
	if (ctx->vp8)
		return "VP8";
	if (ctx->mpeg4)
		return "MPEG-4 Part 2";
	if (ctx->mpeg2)
		return "MPEG-2";
	if (ctx->hevc)
		return "HEVC";

	return "H.264";
}

static void histb_vdec_watchdog(struct work_struct *work)
{
	struct histb_vdec_dev *vdec =
		container_of(to_delayed_work(work), struct histb_vdec_dev,
			     watchdog_work);
	struct histb_vdec_ctx *ctx;
	unsigned long flags;
	bool cancelled = false;

	mutex_lock(&vdec->launch_lock);
	spin_lock_irqsave(&vdec->irqlock, flags);
	ctx = vdec->curr_ctx;
	if (ctx)
		cancelled = vdec->job_cancelled;
	spin_unlock_irqrestore(&vdec->irqlock, flags);
	if (ctx && !histb_vdec_take_job(vdec, ctx))
		ctx = NULL;
	mutex_unlock(&vdec->launch_lock);
	if (!ctx)
		return;

	if (!cancelled)
		dev_err(vdec->dev, "%s decode timed out\n",
			histb_vdec_codec_name(ctx));
	mutex_lock(&vdec->launch_lock);
	histb_vdec_reset_engine(vdec);
	mutex_unlock(&vdec->launch_lock);
	if (ctx->vp8) {
		if (cancelled)
			histb_vdec_reset_vp8_state(ctx);
		else
			histb_vdec_commit_vp8_state(ctx);
	}
	if (ctx->vp9) {
		/* A watchdog has no validated UP/DONE report to describe a
		 * reconstructable surface.  Never publish its pending DPB state. */
		histb_vdec_discard_vp9_state(ctx);
	}
	if (ctx->mpeg4)
		histb_vdec_fail_mpeg4_state(ctx);
	if (ctx->vc1)
		histb_vdec_discard_vc1_runtime_error(ctx, cancelled);
	histb_vdec_account_job(vdec, true);
	if (ctx->vc1)
		histb_vdec_finish_vc1_error(ctx);
	else
		histb_vdec_finish_job(ctx, VB2_BUF_STATE_ERROR);
}

static void histb_vdec_postprocess(struct work_struct *work)
{
	struct histb_vdec_dev *vdec =
		container_of(work, struct histb_vdec_dev, postprocess_work);
	struct histb_vdec_ctx *ctx;
	struct histb_mpeg4_display_plan mpeg4_display_plan;
	struct histb_mpeg4_display_state mpeg4_display_state;
	struct vb2_v4l2_buffer *mpeg4_src;
	unsigned long flags;
	bool aborted;
	bool mpeg2_first_field = false;
	bool vc1_first_field = false;
	bool vc1_commit_ok = false;
	bool mpeg4_draining;
	bool mpeg4_first_anchor = false;
	int ret;

	spin_lock_irqsave(&vdec->irqlock, flags);
	ctx = vdec->post_ctx;
	spin_unlock_irqrestore(&vdec->irqlock, flags);
	if (!ctx)
		return;

	vc1_first_field = ctx->vc1 && ctx->vc1_pending_valid &&
		ctx->vc1_pending_picture.fcm == HISTB_VC1_FIELD_INTERLACED &&
		!ctx->vc1_pending_picture.is_second_field;
	mpeg2_first_field = ctx->mpeg2 && ctx->mpeg2_picture_valid &&
		ctx->mpeg2_field_picture && !ctx->mpeg2_second_field;
	ret = vc1_first_field || mpeg2_first_field ?
		0 : histb_vdec_copy_capture(ctx);
	spin_lock_irqsave(&vdec->irqlock, flags);
	aborted = vdec->post_abort;
	spin_unlock_irqrestore(&vdec->irqlock, flags);
	if (ret) {
		spin_lock_irqsave(&vdec->irqlock, flags);
		vdec->errors++;
		spin_unlock_irqrestore(&vdec->irqlock, flags);
	}
	if (ctx->vp8) {
		/* Keep the VDH-updated state when only capture conversion failed. */
		if (!aborted)
			histb_vdec_commit_vp8_state(ctx);
		else
			histb_vdec_reset_vp8_state(ctx);
	}
	if (ctx->vp9) {
		if (!ret && !aborted)
			histb_vdec_commit_vp9_state(ctx);
		else
			histb_vdec_discard_vp9_state(ctx);
	}
	if (ctx->avs) {
		struct vb2_v4l2_buffer *dst;
		struct histb_vdec_decoded_buffer *decoded;

		dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
		if (dst) {
			decoded = histb_vdec_decoded_buffer(dst);
			/*
			 * Publish reference ownership atomically with the final abort
			 * check so streamoff cannot expose a stale surface.
			 */
			spin_lock_irqsave(&vdec->irqlock, flags);
			aborted = vdec->post_abort;
			decoded->avs.valid = !ret && !aborted &&
					     !decoded->error_tainted;
			spin_unlock_irqrestore(&vdec->irqlock, flags);
		}
	}
	if (ctx->vc1) {
		mutex_lock(&ctx->vc1_lock);
		/* Abort takes vc1_lock before publishing post_abort.  Re-read the
		 * flag here so a stop racing copy_capture cannot commit this frame. */
		spin_lock_irqsave(&vdec->irqlock, flags);
		aborted = vdec->post_abort;
		spin_unlock_irqrestore(&vdec->irqlock, flags);
		if (!aborted && vc1_first_field)
			vc1_commit_ok = histb_vdec_commit_vc1_first_field(ctx);
		else if (!aborted)
			vc1_commit_ok = histb_vdec_commit_vc1_state(ctx);
		else
			histb_vdec_reset_vc1_state_locked(ctx);
		if (!aborted && vc1_commit_ok) {
			if (vc1_first_field) {
				pm_runtime_mark_last_busy(vdec->dev);
				pm_runtime_put_autosuspend(vdec->dev);
				v4l2_m2m_job_finish(vdec->m2m_dev,
						    ctx->fh.m2m_ctx);
			} else {
				histb_vdec_finish_vc1_job(ctx, !ret);
			}
			mutex_unlock(&ctx->vc1_lock);
		} else {
			if (!aborted && vc1_first_field) {
				histb_vdec_discard_vc1_current_locked(ctx, true);
				histb_vdec_reset_vc1_au(ctx);
			}
			mutex_unlock(&ctx->vc1_lock);
			histb_vdec_finish_vc1_error(ctx);
		}
		spin_lock_irqsave(&vdec->irqlock, flags);
		if (vdec->post_ctx == ctx) {
			vdec->post_ctx = NULL;
			vdec->post_abort = false;
		}
		spin_unlock_irqrestore(&vdec->irqlock, flags);
		complete(&vdec->postprocess_idle);
		return;
	}
	if (ctx->mpeg4) {
		mutex_lock(&ctx->mpeg4_lock);
		if (!ret && !aborted) {
			mpeg4_first_anchor =
				ctx->mpeg4_pending_frame.vop.coding_type !=
				HISTB_MPEG4_B_VOP &&
				!ctx->mpeg4_ref[1].tile.cpu;
			mpeg4_src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
			histb_vdec_commit_mpeg4_state(ctx);
			ret = histb_vdec_advance_mpeg4_au(ctx);
			if (!ret) {
				mpeg4_display_state = ctx->mpeg4_display_state;
				mpeg4_draining = ctx->mpeg4_pending_source_done &&
					v4l2_m2m_is_last_draining_src_buf(
							ctx->fh.m2m_ctx, mpeg4_src);
				ret = histb_mpeg4_plan_display(&mpeg4_display_state,
						ctx->mpeg4_pending_frame.vop.coding_type,
						ctx->mpeg4_pending_frame.vol.low_delay,
						mpeg4_first_anchor, mpeg4_draining,
						&mpeg4_display_plan);
			}
			if (ret)
				histb_vdec_fail_mpeg4_state_locked(ctx);
		} else {
			histb_vdec_fail_mpeg4_state_locked(ctx);
		}
	}
	if (ctx->mpeg4 && !ret && !aborted) {
		histb_vdec_finish_mpeg4_job(ctx, &mpeg4_display_plan,
					    mpeg4_display_state.pending);
		mutex_unlock(&ctx->mpeg4_lock);
		spin_lock_irqsave(&vdec->irqlock, flags);
		if (vdec->post_ctx == ctx) {
			vdec->post_ctx = NULL;
			vdec->post_abort = false;
		}
		spin_unlock_irqrestore(&vdec->irqlock, flags);
		/* Do not let the next device_run overtake display/DPB completion. */
		complete(&vdec->postprocess_idle);
		return;
	}
	if (ctx->mpeg4)
		mutex_unlock(&ctx->mpeg4_lock);
	histb_vdec_finish_job(ctx, !ret && !aborted ?
				      VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR);
	spin_lock_irqsave(&vdec->irqlock, flags);
	if (vdec->post_ctx == ctx) {
		vdec->post_ctx = NULL;
		vdec->post_abort = false;
	}
	spin_unlock_irqrestore(&vdec->irqlock, flags);
	/* finish_job may schedule the next request immediately.  Keep the
	 * postprocess owner held until capture and codec state are complete. */
	complete(&vdec->postprocess_idle);
}

static bool histb_vdec_is_cabac_end_error(struct histb_vdec_ctx *ctx,
						  u32 state, const __le32 *up_msg,
						  u32 smmu_state_secure,
						  u32 smmu_state_nonsecure)
{
	u32 up0 = le32_to_cpu(up_msg[0]) & ~BIT(31);
	u32 up1 = le32_to_cpu(up_msg[1]) & GENMASK(15, 0);
	u32 up2 = le32_to_cpu(up_msg[2]) & GENMASK(15, 0);
	u32 up3 = le32_to_cpu(up_msg[3]);
	unsigned int i;

	/*
	 * VP8 uses the boolean decoder rather than H.264 CABAC, but CV200
	 * reports its normal end through the same DECODE_ERROR state. The
	 * original VFMW validates the decoded macroblock range in UP words 1/2;
	 * words 0/3 are not part of its VP8 completion decision.
	 */
	if (ctx->vp8)
		return ctx->total_mbs &&
		       state == HISTB_VDEC_STATE_CABAC_END_ERROR &&
		       !up1 && up2 == ctx->total_mbs - 1 &&
		       !smmu_state_secure && !smmu_state_nonsecure;

	if (ctx->mpeg2 && ctx->src.pix.pixelformat != V4L2_PIX_FMT_MPEG1_SLICE) {
		u32 next_mb = 0;

		/*
		 * The original VFMW completion path consumes only the low 16 bits
		 * of UP words 1/2 as the decoded macroblock range.  Word 0 also
		 * carries picture-dependent status, so it is not a completion
		 * discriminator for MPEG-2 I/P/B pictures.
		 */
		if (!ctx->mpeg2_slices ||
		    state != ((HISTB_VDEC_STATE_CABAC_END_ERROR &
				~GENMASK(16, 0)) | ctx->mpeg2_slices) ||
		    smmu_state_secure || smmu_state_nonsecure)
			return false;
		for (i = 0; i < ctx->mpeg2_slices; i++) {
			u32 start = le32_to_cpu(up_msg[i * 4 + 1]) & GENMASK(15, 0);
			u32 end = le32_to_cpu(up_msg[i * 4 + 2]) & GENMASK(15, 0);

			if (start != next_mb || end < start ||
			    end >= ctx->total_mbs)
				return false;
			next_mb = end + 1;
		}

		return next_mb == ctx->total_mbs;
	}

	/*
	 * MPEG-1 shares this path but not this report format.  See
	 * MPEG1-SHARES-THE-MPEG2-COMPLETION-RULE.md: the engine reports slice 0
	 * as macroblocks 0..7 and slice 1 as 315..333 for a 1620-macroblock
	 * picture, while the driver already knows the real start macroblock of
	 * every slice.  Applying the MPEG-2 continuity rule rejected the whole
	 * picture, which threw the frame away and left the client a zero buffer.
	 */
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG1_SLICE) {
		if (state != ((HISTB_VDEC_STATE_CABAC_END_ERROR &
				~GENMASK(16, 0)) | ctx->mpeg2_slices) ||
		    smmu_state_secure || smmu_state_nonsecure)
			return false;

		return true;
	}

	if (ctx->hevc) {
		u32 next_ctb = 0;

		if (state != ((HISTB_VDEC_STATE_CABAC_END_ERROR &
				~GENMASK(16, 0)) | ctx->hevc_slices) ||
		    smmu_state_secure || smmu_state_nonsecure)
			return false;
		for (i = 0; i < ctx->hevc_slices; i++) {
			u32 start = le32_to_cpu(up_msg[i * 4 + 1]) & GENMASK(15, 0);
			u32 end = le32_to_cpu(up_msg[i * 4 + 2]) & GENMASK(15, 0);

			up0 = le32_to_cpu(up_msg[i * 4]) & ~BIT(31);
			up3 = le32_to_cpu(up_msg[i * 4 + 3]);
			if ((up0 != 0 && up0 != 0x00020000 && up0 != 0x00210000 &&
			     up0 != 0x00230000) || start != next_ctb || end < start ||
			    end >= ctx->total_mbs ||
			    (up3 != 0 && up3 != BIT(28) && up3 != BIT(29)))
				return false;
			next_ctb = end + 1;
		}

		return next_ctb == ctx->total_mbs;
	}

	/*
	 * CV200 reports its CABAC end marker together with DECODE_ERROR even when
	 * the complete picture was reconstructed. For progressive pictures word 2
	 * is the last decoded macroblock, so reject a truncated slice here.
	 */
	return ctx->cabac && state == HISTB_VDEC_STATE_CABAC_END_ERROR &&
	       up0 == HISTB_VDEC_UP_CABAC_END_ERROR &&
	       (ctx->mbaff ||
		up2 == ctx->total_mbs - 1) &&
	       le32_to_cpu(up_msg[3]) == 0 &&
	       !smmu_state_secure && !smmu_state_nonsecure;
}

static bool histb_vdec_h264_slice_message_valid(struct histb_vdec_ctx *ctx,
						 u32 state, const __le32 *up_msg,
						 u32 smmu_state_secure,
						 u32 smmu_state_nonsecure)
{
	u32 reports = state & GENMASK(16, 0);
	u32 completion = state & ~GENMASK(16, 0);
	/* MBAFF slice addresses use MB pairs, but UP reports expanded MBs. */
	u32 units = ctx->total_mbs;
	u32 next_mb = 0;
	unsigned int i;

	if (!units || !ctx->h264_slices ||
	    reports != ctx->h264_slices ||
	    (completion != HISTB_VDEC_STATE_DECODE_DONE &&
	     completion != (HISTB_VDEC_STATE_CABAC_END_ERROR &
			    ~GENMASK(16, 0))) ||
	    smmu_state_secure || smmu_state_nonsecure)
		return false;

	for (i = 0; i < reports; i++) {
		u32 start = le32_to_cpu(up_msg[i * 4 + 1]) & GENMASK(15, 0);
		u32 end = le32_to_cpu(up_msg[i * 4 + 2]) & GENMASK(15, 0);

		if (start != next_mb || end < start || end >= units ||
		    le32_to_cpu(up_msg[i * 4 + 3]))
			return false;
		next_mb = end + 1;
	}

	return next_mb == units;
}

static unsigned int histb_vdec_slice_count(const struct histb_vdec_ctx *ctx)
{
	if (ctx->avs)
		return ctx->avs_slices;
	if (ctx->mpeg2)
		return ctx->mpeg2_slices;
	if (ctx->mpeg4)
		return ctx->mpeg4_slices;
	if (ctx->hevc)
		return ctx->hevc_slices;
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE)
		return ctx->h264_slices;

	return 1;
}

static bool
histb_vdec_avs_up_message_valid(const struct histb_vdec_ctx *ctx, u32 state,
				const __le32 *up_msg, u32 smmu_state_secure,
				u32 smmu_state_nonsecure)
{
	u32 reports = state & GENMASK(16, 0);
	u32 completion = state & ~GENMASK(16, 0);
	u32 covered = 0;
	unsigned int i;

	/*
	 * VFMW treats DONE|ERROR as a repair report. It is deliverable only when
	 * the reported macroblock ranges still cover the complete picture.
	 */
	if (!ctx->avs || !ctx->avs_slices ||
	    reports != ctx->avs_slices ||
	    (completion != HISTB_VDEC_STATE_DECODE_DONE &&
	     completion != (HISTB_VDEC_STATE_DECODE_DONE |
			    HISTB_VDEC_STATE_DECODE_ERROR)) ||
	    smmu_state_secure || smmu_state_nonsecure)
		return false;

	for (i = 0; i < reports; i++) {
		u32 start = le32_to_cpu(up_msg[i * 4 + 1]) & GENMASK(15, 0);
		u32 end = le32_to_cpu(up_msg[i * 4 + 2]) & GENMASK(15, 0);

		if (start != covered || end < start || end >= ctx->avs_total_mbs)
			return false;
		covered = end + 1;
	}

	return covered == ctx->avs_total_mbs;
}

static bool histb_vdec_vp9_up_message_valid(const struct histb_vdec_ctx *ctx,
						     u32 state,
						     const __le32 *up_msg,
						     u32 smmu_state_secure,
						     u32 smmu_state_nonsecure)
{
	u32 total_sbs = DIV_ROUND_UP(ctx->dst.pix.width, 64) *
			DIV_ROUND_UP(ctx->dst.pix.height, 64);
	u32 reports = state & GENMASK(16, 0);
	u32 covered = 0;
	unsigned int i;

	if ((state & ~GENMASK(16, 0)) != HISTB_VDEC_STATE_VP9_END ||
	    smmu_state_secure || smmu_state_nonsecure ||
	    !total_sbs || !reports || reports > HISTB_VDEC_MAX_UP_REPORTS)
		return false;

	for (i = 0; i < reports; i++) {
		u32 start = le32_to_cpu(up_msg[i * 4 + 1]) & GENMASK(19, 0);
		u32 end = le32_to_cpu(up_msg[i * 4 + 2]) & GENMASK(19, 0);

		/* The original report parser consumes only the range words. */
		if (start != covered || start > end || end >= total_sbs)
			return false;
		covered = end + 1;
	}

	return covered == total_sbs;
}

static bool histb_vdec_mpeg4_up_message_valid(
			const struct histb_vdec_ctx *ctx, u32 state, const __le32 *up_msg,
			u32 smmu_state_secure, u32 smmu_state_nonsecure)
{
	u32 reports = state & GENMASK(16, 0);
	u32 completion = state & ~GENMASK(16, 0);
	u32 covered = 0;
	unsigned int i;

	/*
	 * CV200 reports a complete MPEG-4 picture through DECODE_ERROR on the
	 * live hardware. The original parser still consumes up to 200 range
	 * reports in this state; without its repair pass, accept only a complete
	 * ordered union covering the whole picture.
	 */
	if (!ctx->total_mbs || !reports || reports > HISTB_VDEC_MAX_UP_REPORTS ||
	    (completion != HISTB_VDEC_STATE_DECODE_DONE &&
	     completion != (HISTB_VDEC_STATE_CABAC_END_ERROR &
			    ~GENMASK(16, 0))) ||
	    smmu_state_secure || smmu_state_nonsecure)
		return false;

	for (i = 0; i < reports; i++) {
		u32 start = le32_to_cpu(up_msg[i * 4 + 1]) & GENMASK(15, 0);
		u32 end = le32_to_cpu(up_msg[i * 4 + 2]) & GENMASK(15, 0);

		if (start > end || end >= ctx->total_mbs || start > covered)
			return false;
		if (end >= covered)
			covered = end + 1;
	}

	return covered == ctx->total_mbs;
}

static irqreturn_t histb_vdec_irq_thread(int irq, void *data)
{
	struct histb_vdec_dev *vdec = data;
	struct histb_vdec_ctx *ctx;
	enum histb_vdec_job_phase phase = HISTB_VDEC_PHASE_IDLE;
	__le32 *up_msg;
	unsigned long flags;
	u32 expected_state, state, status;
	bool cabac_end_error, cancelled = false, decode_failed, message_error;
	bool phase_error = false, post_busy = false;
	bool post_published = false;
	u32 smmu_state_secure, smmu_state_nonsecure;

	/* Serialize status sampling/acknowledgement with the launch path.  Without
	 * this, a thread that sampled an old INT_DONE can observe PHASE_VDH after
	 * the BPD->VDH transition and complete the wrong job. */
	mutex_lock(&vdec->launch_lock);
	status = readl(vdec->regs + HISTB_VDEC_INT_STATE);
	if (!(status & HISTB_VDEC_INT_DONE)) {
		mutex_unlock(&vdec->launch_lock);
		return IRQ_NONE;
	}

	state = readl(vdec->regs + HISTB_VDEC_STATE);
	smmu_state_secure = readl(vdec->regs + HISTB_VDEC_SMMU_INT_STATE_S);
	smmu_state_nonsecure = readl(vdec->regs + HISTB_VDEC_SMMU_INT_STATE_NS);

	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);

	spin_lock_irqsave(&vdec->irqlock, flags);
	ctx = vdec->curr_ctx;
	if (ctx) {
		phase = vdec->phase;
		if (phase == HISTB_VDEC_PHASE_BPD) {
			/* A stale VDH interrupt cannot take the synchronous BPD owner. */
			spin_unlock_irqrestore(&vdec->irqlock, flags);
			mutex_unlock(&vdec->launch_lock);
			return IRQ_HANDLED;
		}
		cancelled = vdec->job_cancelled;
		phase_error = phase != HISTB_VDEC_PHASE_VDH;
		vdec->curr_ctx = NULL;
		vdec->phase = HISTB_VDEC_PHASE_IDLE;
		vdec->job_cancelled = false;
		if (cancelled || phase_error) {
			/* Abort or a wrong-phase IRQ owns error completion, never display. */
		} else if (WARN_ON(vdec->post_ctx)) {
			post_busy = true;
		} else {
			vdec->post_ctx = ctx;
			vdec->post_abort = false;
			reinit_completion(&vdec->postprocess_idle);
			post_published = true;
		}
	}
	spin_unlock_irqrestore(&vdec->irqlock, flags);
	mutex_unlock(&vdec->launch_lock);
	if (!ctx)
		return IRQ_HANDLED;
	if (cancelled || phase_error) {
		cancel_delayed_work(&vdec->watchdog_work);
		mutex_lock(&vdec->launch_lock);
		histb_vdec_reset_engine(vdec);
		mutex_unlock(&vdec->launch_lock);
		if (ctx->vp8)
			histb_vdec_reset_vp8_state(ctx);
		if (ctx->vp9)
			histb_vdec_discard_vp9_state(ctx);
		if (ctx->mpeg4)
			histb_vdec_fail_mpeg4_state(ctx);
		if (ctx->vc1)
			histb_vdec_discard_vc1_current(ctx, true);
		histb_vdec_account_job(vdec, true);
		if (ctx->vc1)
			histb_vdec_finish_vc1_error(ctx);
		else
			histb_vdec_finish_job(ctx, VB2_BUF_STATE_ERROR);
		return IRQ_HANDLED;
	}
	if (post_busy) {
		cancel_delayed_work(&vdec->watchdog_work);
		if (ctx->vp8)
			histb_vdec_reset_vp8_state(ctx);
		if (ctx->vp9)
			histb_vdec_discard_vp9_state(ctx);
		if (ctx->mpeg4)
			histb_vdec_fail_mpeg4_state(ctx);
		if (ctx->vc1)
			histb_vdec_discard_vc1_current(ctx, true);
		histb_vdec_account_job(vdec, true);
		if (ctx->vc1)
			histb_vdec_finish_vc1_error(ctx);
		else
			histb_vdec_finish_job(ctx, VB2_BUF_STATE_ERROR);
		return IRQ_HANDLED;
	}

	up_msg = histb_vdec_msg_slot(ctx, HISTB_VDEC_UP_MSG_SLOT);
	expected_state = ctx->avs ?
		HISTB_VDEC_STATE_DECODE_DONE | ctx->avs_slices :
		ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE ?
			HISTB_VDEC_STATE_DECODE_DONE |
			ctx->h264_slices : ctx->mpeg4 ?
			HISTB_VDEC_STATE_DECODE_DONE | 1 : ctx->mpeg2 ?
			(HISTB_VDEC_STATE_CABAC_END_ERROR & ~GENMASK(16, 0)) |
			ctx->mpeg2_slices : ctx->hevc ?
			(HISTB_VDEC_STATE_CABAC_END_ERROR & ~GENMASK(16, 0)) |
			ctx->hevc_slices : HISTB_VDEC_STATE_CABAC_END_ERROR;
	message_error =
		(ctx->avs &&
		 !histb_vdec_avs_up_message_valid(ctx, state, up_msg,
						 smmu_state_secure,
						 smmu_state_nonsecure)) ||
		(ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE &&
		 !histb_vdec_h264_slice_message_valid(ctx, state, up_msg,
						     smmu_state_secure,
						     smmu_state_nonsecure)) ||
		(ctx->vp9 &&
		 !histb_vdec_vp9_up_message_valid(ctx, state, up_msg,
							 smmu_state_secure,
							 smmu_state_nonsecure)) ||
		(ctx->mpeg4 &&
		 !histb_vdec_mpeg4_up_message_valid(ctx, state, up_msg,
							 smmu_state_secure,
							 smmu_state_nonsecure)) ||
		(ctx->vc1 && histb_vc1_validate_up_reports(
			state,
			smmu_state_secure, smmu_state_nonsecure,
			ctx->total_mbs,
			(const struct histb_vc1_up_report *)up_msg,
			HISTB_VC1_MAX_UP_REPORTS) != HISTB_VC1_OK);
	cabac_end_error = ctx->avs ? !message_error :
		ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE ?
		!message_error : ctx->vp9 ? !message_error : ctx->mpeg4 ?
		!message_error : ctx->vc1 ? !message_error :
		histb_vdec_is_cabac_end_error(ctx, state, up_msg,
					       smmu_state_secure,
					       smmu_state_nonsecure);
	decode_failed = message_error || !(state & HISTB_VDEC_STATE_DECODE_DONE) ||
		((state & HISTB_VDEC_STATE_DECODE_ERROR) && !cabac_end_error);

	/* Account for hardware completion before CPU postprocessing. */
	histb_vdec_account_job(vdec, decode_failed);

	if (decode_failed) {
		unsigned int i;

		dev_err(vdec->dev,
			"%s decode failed: state=%08x expected=%08x slices=%u units=%u int=%08x vctrl=%08x up=%08x/%08x/%08x/%08x h264=%u/%u/%u/%u\n",
			histb_vdec_codec_name(ctx), state, expected_state,
			histb_vdec_slice_count(ctx), ctx->total_mbs, status,
			readl(vdec->regs + HISTB_VDEC_VCTRL_STATE),
			le32_to_cpu(up_msg[0]), le32_to_cpu(up_msg[1]),
			le32_to_cpu(up_msg[2]), le32_to_cpu(up_msg[3]),
			ctx->h264_new_frame, ctx->h264_last_slice,
			ctx->h264_partial, ctx->h264_first_mb);
		for (i = 1;
		     (ctx->avs && i < ctx->avs_slices) ||
		     (ctx->hevc && i < ctx->hevc_slices) ||
		     (ctx->vp9 &&
		      i < min_t(u32, state & GENMASK(16, 0),
				HISTB_VDEC_MAX_UP_REPORTS)) ||
		     (ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE &&
		      i < ctx->h264_slices);
		     i++)
			dev_err(vdec->dev,
				"%s UP slice %u: %08x/%08x/%08x/%08x\n",
				histb_vdec_codec_name(ctx), i,
				le32_to_cpu(up_msg[i * 4]),
				le32_to_cpu(up_msg[i * 4 + 1]),
				le32_to_cpu(up_msg[i * 4 + 2]),
				le32_to_cpu(up_msg[i * 4 + 3]));
		dev_err(vdec->dev,
			"SMMU state s=%08x ns=%08x err=%08x/%08x fault-r=%08x/%08x fault-w=%08x/%08x\n",
			readl(vdec->regs + HISTB_VDEC_SMMU_INT_STATE_S),
			readl(vdec->regs + HISTB_VDEC_SMMU_INT_STATE_NS),
			readl(vdec->regs + HISTB_VDEC_SMMU_ERR_RD_ADDR),
			readl(vdec->regs + HISTB_VDEC_SMMU_ERR_WR_ADDR),
			readl(vdec->regs + HISTB_VDEC_SMMU_FAULT_RD_S),
			readl(vdec->regs + HISTB_VDEC_SMMU_FAULT_RD_NS),
			readl(vdec->regs + HISTB_VDEC_SMMU_FAULT_WR_S),
			readl(vdec->regs + HISTB_VDEC_SMMU_FAULT_WR_NS));
		mutex_lock(&vdec->launch_lock);
		histb_vdec_reset_engine(vdec);
		mutex_unlock(&vdec->launch_lock);
	}
	cancel_delayed_work(&vdec->watchdog_work);
	if (!decode_failed &&
	    ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE)
		histb_vdec_reset_h264_slices(ctx);
	if (!decode_failed) {
		if (!queue_work(system_long_wq, &vdec->postprocess_work)) {
			if (ctx->vp8)
				histb_vdec_commit_vp8_state(ctx);
			if (ctx->vp9)
				histb_vdec_discard_vp9_state(ctx);
			if (ctx->mpeg4)
				histb_vdec_fail_mpeg4_state(ctx);
			if (ctx->vc1)
				histb_vdec_discard_vc1_runtime_error(ctx, false);
			if (ctx->vc1)
				histb_vdec_finish_vc1_error(ctx);
			else
				histb_vdec_finish_job(ctx, VB2_BUF_STATE_ERROR);
			if (post_published) {
				spin_lock_irqsave(&vdec->irqlock, flags);
				if (vdec->post_ctx == ctx) {
					vdec->post_ctx = NULL;
					vdec->post_abort = false;
				}
				spin_unlock_irqrestore(&vdec->irqlock, flags);
				complete(&vdec->postprocess_idle);
			}
			spin_lock_irqsave(&vdec->irqlock, flags);
			vdec->errors++;
			spin_unlock_irqrestore(&vdec->irqlock, flags);
		}
	} else {
		if (ctx->vp8)
			histb_vdec_commit_vp8_state(ctx);
		if (ctx->vp9) {
			/* A validated UP report plus DONE is a runtime failure: keep
			 * the reconstructed surface available, but let taint
			 * propagation mark dependants ERROR.  Invalid reports or a
			 * non-completing VDH job have no trustworthy DPB state. */
			if (!message_error &&
			    (state & HISTB_VDEC_STATE_DECODE_DONE))
				histb_vdec_commit_vp9_state(ctx);
			else
				histb_vdec_discard_vp9_state(ctx);
		}
		if (ctx->mpeg4)
			histb_vdec_fail_mpeg4_state(ctx);
		if (ctx->vc1)
			histb_vdec_discard_vc1_runtime_error(ctx, false);
		if (ctx->vc1)
			histb_vdec_finish_vc1_error(ctx);
		else
			histb_vdec_finish_job(ctx, VB2_BUF_STATE_ERROR);
		if (post_published) {
			spin_lock_irqsave(&vdec->irqlock, flags);
			if (vdec->post_ctx == ctx) {
				vdec->post_ctx = NULL;
				vdec->post_abort = false;
			}
			spin_unlock_irqrestore(&vdec->irqlock, flags);
			complete(&vdec->postprocess_idle);
		}
	}

	return IRQ_HANDLED;
}

static void histb_vdec_abort(struct histb_vdec_ctx *ctx)
{
	struct histb_vdec_dev *vdec = ctx->vdec;
	enum histb_vdec_job_phase phase = HISTB_VDEC_PHASE_IDLE;
	unsigned long flags;
	bool active, post, taken = false;

	spin_lock_irqsave(&vdec->irqlock, flags);
	active = vdec->curr_ctx == ctx;
	if (active) {
		phase = vdec->phase;
		vdec->job_cancelled = true;
	}
	spin_unlock_irqrestore(&vdec->irqlock, flags);
	if (ctx->vc1) {
		wait_for_completion(&ctx->vc1_setup_idle);
		/* Both queues may stream off; keep the completed barrier observable. */
		complete(&ctx->vc1_setup_idle);
	}
	/* The synchronous BPD caller is the sole buffer and PM owner. */
	if (active && phase == HISTB_VDEC_PHASE_BPD) {
		return;
	}
	if (active) {
		/* Wait out an in-progress register launch before resetting the engine. */
		mutex_lock(&vdec->launch_lock);
		taken = histb_vdec_take_job(vdec, ctx);
		mutex_unlock(&vdec->launch_lock);
	}
	if (!taken) {
		/* An IRQ may have won ownership while abort waited for launch_lock. */
		if (active)
			synchronize_irq(vdec->irq);
		if (ctx->vc1) {
			mutex_lock(&ctx->vc1_lock);
			spin_lock_irqsave(&vdec->irqlock, flags);
			post = vdec->post_ctx == ctx;
			if (post)
				vdec->post_abort = true;
			spin_unlock_irqrestore(&vdec->irqlock, flags);
			mutex_unlock(&ctx->vc1_lock);
		} else {
			spin_lock_irqsave(&vdec->irqlock, flags);
			post = vdec->post_ctx == ctx;
			if (post)
				vdec->post_abort = true;
			spin_unlock_irqrestore(&vdec->irqlock, flags);
		}
		if (!post)
			return;
		/* Let this context's IRQ finish queueing its postprocess work. */
		synchronize_irq(vdec->irq);
		cancel_delayed_work_sync(&vdec->watchdog_work);
		cancel_work_sync(&vdec->postprocess_work);
		spin_lock_irqsave(&vdec->irqlock, flags);
		post = vdec->post_ctx == ctx;
		if (post) {
			vdec->post_ctx = NULL;
			vdec->post_abort = false;
		}
		spin_unlock_irqrestore(&vdec->irqlock, flags);
		if (post) {
			if (ctx->vp8)
				histb_vdec_reset_vp8_state(ctx);
			if (ctx->vp9)
				histb_vdec_discard_vp9_state(ctx);
			if (ctx->mpeg4)
				histb_vdec_fail_mpeg4_state(ctx);
			if (ctx->vc1)
				histb_vdec_reset_vc1_state(ctx);
			histb_vdec_finish_job(ctx, VB2_BUF_STATE_ERROR);
			complete(&vdec->postprocess_idle);
		}
		return;
	}

	cancel_delayed_work_sync(&vdec->watchdog_work);
	histb_vdec_reset_engine(vdec);
	if (ctx->vp8)
		histb_vdec_reset_vp8_state(ctx);
	if (ctx->vp9)
		histb_vdec_discard_vp9_state(ctx);
	if (ctx->mpeg4)
		histb_vdec_fail_mpeg4_state(ctx);
	if (ctx->vc1)
		histb_vdec_reset_vc1_state(ctx);
	/* Do not let a stale threaded IRQ observe the next m2m job. */
	synchronize_irq(vdec->irq);
	histb_vdec_account_job(vdec, true);
	histb_vdec_finish_job(ctx, VB2_BUF_STATE_ERROR);
}

static void histb_vdec_program_mpeg2_registers(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		dma_addr_t src_dma, u32 total_mbs)
{
	struct histb_vdec_dev *vdec = ctx->vdec;
	dma_addr_t msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	dma_addr_t pic_msg = msg_dma + HISTB_VDEC_MPEG2_PIC_MSG_OFFSET;
	dma_addr_t up_msg = msg_dma +
		HISTB_VDEC_UP_MSG_SLOT * HISTB_VDEC_H264_MSG_SLOT_SIZE;
	u32 max_slice_group = DIV_ROUND_UP(ctx->mpeg2_slices, 64) - 1;
	u32 offset;

	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	writel(0, vdec->regs + HISTB_VDEC_SMMU_CTRL);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_MASK_NS);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_S);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_NS);
	writel(HISTB_VDEC_BASIC_CFG0_MPEG2 | (total_mbs - 1),
	       vdec->regs + HISTB_VDEC_BASIC_CFG0);
	writel(HISTB_VDEC_BASIC_CFG1_MPEG2 |
	       FIELD_PREP(HISTB_VDEC_BASIC_CFG1_MAX_SLICE_GROUP,
			  max_slice_group),
	       vdec->regs + HISTB_VDEC_BASIC_CFG1);
	writel(lower_32_bits(pic_msg), vdec->regs + HISTB_VDEC_AVM_ADDR);
	writel(lower_32_bits(up_msg), vdec->regs + HISTB_VDEC_VAM_ADDR);
	writel(lower_32_bits(round_down(src_dma, 16)),
	       vdec->regs + HISTB_VDEC_STREAM_BASE);
	writel(0, vdec->regs + HISTB_VDEC_SCD_AVS_FLAG);
	writel(1, vdec->regs + HISTB_VDEC_SCD_VDH_SELRST);
	writel(HISTB_VDEC_SCD_EMAR_BASE |
	       (DIV_ROUND_UP(ctx->dst.pix.width, 16) <= 256 ?
		HISTB_VDEC_SCD_EMAR_ENABLE : 0),
	       vdec->regs + HISTB_VDEC_SCD_EMAR_CFG);
	for (offset = HISTB_VDEC_TIMEOUT_FIRST;
	     offset <= HISTB_VDEC_TIMEOUT_LAST; offset += 4)
		writel(HISTB_VDEC_TIMEOUT_VALUE, vdec->regs + offset);
	writel(lower_32_bits(round_down(decoded->tile.dma, 16)),
	       vdec->regs + HISTB_VDEC_CURRENT_Y);
	writel(histb_vdec_tile_stride(ctx->dst.pix.width) * 16,
	       vdec->regs + HISTB_VDEC_Y_STRIDE);
	writel(histb_vdec_tile_stride(ctx->dst.pix.width) *
	       ALIGN(ctx->dst.pix.height,
		     histb_vdec_surface_height_align(ctx)),
	       vdec->regs + HISTB_VDEC_CHROMA_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_HEAD_INFO_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_LINE_NUM_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_Y_STRIDE_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_Y_OFFSET_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_CHROMA_OFFSET_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_PPFD_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_PPFD_SIZE);
	writel(ctx->mpeg2_ref_pic_type,
	       vdec->regs + HISTB_VDEC_REF_PIC_TYPE);
	writel(0, vdec->regs + HISTB_VDEC_FF_APT_ENABLE);
	writel(0xaaaaaaaa, vdec->regs + HISTB_VDEC_DOWN_CLK_CFG);
	writel(HISTB_VDEC_INT_ENABLE_DONE, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(3, vdec->regs + HISTB_VDEC_SCD_CLOCK_GATE);
	usleep_range(30, 60);
	/* Publish coherent messages and register writes before starting VDH. */
	wmb();
	writel(0, vdec->regs + HISTB_VDEC_START);
	writel(1, vdec->regs + HISTB_VDEC_START);
	writel(0, vdec->regs + HISTB_VDEC_START);
}

static void histb_vdec_program_mpeg4_registers(struct histb_vdec_ctx *ctx)
{
	const struct histb_mpeg4_regs *regs = &ctx->mpeg4_pending_regs;
	struct histb_vdec_dev *vdec = ctx->vdec;
	u32 offset;

	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	writel(0, vdec->regs + HISTB_VDEC_SMMU_CTRL);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_MASK_NS);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_S);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_NS);
	writel(regs->basic_cfg0, vdec->regs + HISTB_VDEC_BASIC_CFG0);
	writel(regs->basic_cfg1, vdec->regs + HISTB_VDEC_BASIC_CFG1);
	writel(regs->avm_addr, vdec->regs + HISTB_VDEC_AVM_ADDR);
	writel(regs->vam_addr, vdec->regs + HISTB_VDEC_VAM_ADDR);
	writel(regs->stream_base, vdec->regs + HISTB_VDEC_STREAM_BASE);
	writel(0, vdec->regs + HISTB_VDEC_SCD_AVS_FLAG);
	writel(1, vdec->regs + HISTB_VDEC_SCD_VDH_SELRST);
	writel(HISTB_VDEC_SCD_EMAR_BASE |
	       (regs->scd_emar ? HISTB_VDEC_SCD_EMAR_ENABLE : 0),
	       vdec->regs + HISTB_VDEC_SCD_EMAR_CFG);
	for (offset = HISTB_VDEC_TIMEOUT_FIRST;
	     offset <= HISTB_VDEC_TIMEOUT_LAST; offset += 4)
		writel(regs->fixed_cfg, vdec->regs + offset);
	writel(regs->current_picture_addr, vdec->regs + HISTB_VDEC_CURRENT_Y);
	writel(regs->y_stride, vdec->regs + HISTB_VDEC_Y_STRIDE);
	writel(regs->uv_offset, vdec->regs + HISTB_VDEC_CHROMA_OFFSET);
	writel(regs->prcnum, vdec->regs + HISTB_VDEC_HEAD_INFO_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_LINE_NUM_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_Y_STRIDE_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_Y_OFFSET_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_CHROMA_OFFSET_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_PPFD_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_PPFD_SIZE);
	writel(regs->dnr_mbinfo_addr, vdec->regs + HISTB_VDEC_DNR_MBINFO_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_REF_PIC_TYPE);
	writel(0, vdec->regs + HISTB_VDEC_FF_APT_ENABLE);
	writel(0xaaaaaaaa, vdec->regs + HISTB_VDEC_DOWN_CLK_CFG);
	writel(HISTB_VDEC_INT_ENABLE_DONE, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(3, vdec->regs + HISTB_VDEC_SCD_CLOCK_GATE);
	usleep_range(30, 60);
	wmb();
	writel(0, vdec->regs + HISTB_VDEC_START);
	writel(1, vdec->regs + HISTB_VDEC_START);
	writel(0, vdec->regs + HISTB_VDEC_START);
}

static void histb_vdec_program_vc1_registers(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded)
{
	struct histb_vdec_dev *vdec = ctx->vdec;
	dma_addr_t msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	dma_addr_t bpd_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_VC1_BPD);
	u32 bpd_base = lower_32_bits(bpd_dma);
	u32 height = ALIGN(ctx->dst.pix.height, 32);
	u32 stride = histb_vdec_surface_stride(ctx);
	u32 bpd_stride = ((DIV_ROUND_UP(ctx->vc1_pending_picture.coded_width, 16) +
			   127) >> 7) << 4;
	u32 offset;

	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	writel(0, vdec->regs + HISTB_VDEC_SMMU_CTRL);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_MASK_NS);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_S);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_NS);
	writel(HISTB_VDEC_BASIC_CFG0_VC1 | (ctx->total_mbs - 1),
	       vdec->regs + HISTB_VDEC_BASIC_CFG0);
	writel(HISTB_VDEC_BASIC_CFG1_VC1,
	       vdec->regs + HISTB_VDEC_BASIC_CFG1);
	writel(lower_32_bits(msg_dma + HISTB_VDEC_VC1_PIC_MSG_OFFSET),
	       vdec->regs + HISTB_VDEC_AVM_ADDR);
	writel(lower_32_bits(msg_dma), vdec->regs + HISTB_VDEC_VAM_ADDR);
	writel(ctx->vc1_pending_stream_base,
	       vdec->regs + HISTB_VDEC_STREAM_BASE);
	writel(0, vdec->regs + HISTB_VDEC_SCD_AVS_FLAG);
	writel(1, vdec->regs + HISTB_VDEC_SCD_VDH_SELRST);
	writel(HISTB_VDEC_SCD_EMAR_BASE |
	       (DIV_ROUND_UP(ctx->dst.pix.width, 16) <= 256 ?
		HISTB_VDEC_SCD_EMAR_ENABLE : 0),
	       vdec->regs + HISTB_VDEC_SCD_EMAR_CFG);
	for (offset = HISTB_VDEC_TIMEOUT_FIRST;
	     offset <= HISTB_VDEC_TIMEOUT_LAST; offset += 4)
		writel(HISTB_VDEC_TIMEOUT_VALUE, vdec->regs + offset);

	/* VC-1 is uncompressed 8-bit output; clear unrelated stale paths. */
	writel(0, vdec->regs + HISTB_VDEC_STORE_PARAM);
	writel(lower_32_bits(decoded->tile.dma),
	       vdec->regs + HISTB_VDEC_CURRENT_Y);
	writel(stride * 16, vdec->regs + HISTB_VDEC_Y_STRIDE);
	writel(stride * height, vdec->regs + HISTB_VDEC_CHROMA_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_HEAD_INFO_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_LINE_NUM_ADDR);
	writel(bpd_stride, vdec->regs + HISTB_VDEC_VC1_BPD_STRIDE);
	writel(bpd_base, vdec->regs + HISTB_VDEC_VC1_MVTYPE_ADDR);
	writel(bpd_base + 2048,
	       vdec->regs + HISTB_VDEC_VC1_SKIP_ADDR);
	writel(bpd_base + 2 * 2048,
	       vdec->regs + HISTB_VDEC_VC1_DIRECT_ADDR);
	writel(bpd_base + 3 * 2048,
	       vdec->regs + HISTB_VDEC_VC1_ACPRED_ADDR);
	writel(bpd_base + 4 * 2048,
	       vdec->regs + HISTB_VDEC_VC1_OVERFLAGS_ADDR);
	writel(bpd_base + 5 * 2048,
	       vdec->regs + HISTB_VDEC_VC1_FIELDTX_ADDR);
	writel(bpd_base + 6 * 2048,
	       vdec->regs + HISTB_VDEC_VC1_FORWARD_ADDR);
	writel(ctx->vc1_pending_ref_pic_type,
	       vdec->regs + HISTB_VDEC_REF_PIC_TYPE);
	writel(0, vdec->regs + HISTB_VDEC_FF_APT_ENABLE);
	writel(0xaaaaaaaa, vdec->regs + HISTB_VDEC_DOWN_CLK_CFG);
	writel(HISTB_VDEC_INT_ENABLE_DONE, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(3, vdec->regs + HISTB_VDEC_SCD_CLOCK_GATE);
	usleep_range(30, 60);
	wmb();
	writel(0, vdec->regs + HISTB_VDEC_START);
	writel(1, vdec->regs + HISTB_VDEC_START);
	writel(0, vdec->regs + HISTB_VDEC_START);
}

static void histb_vdec_program_vp8_registers(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		dma_addr_t src_dma, u32 total_mbs)
{
	struct histb_vdec_dev *vdec = ctx->vdec;
	dma_addr_t msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	dma_addr_t pic_msg = msg_dma +
		HISTB_VDEC_PIC_MSG_SLOT * HISTB_VDEC_H264_MSG_SLOT_SIZE;
	dma_addr_t up_msg = msg_dma +
		HISTB_VDEC_UP_MSG_SLOT * HISTB_VDEC_H264_MSG_SLOT_SIZE;
	u32 offset;

	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	writel(0, vdec->regs + HISTB_VDEC_SMMU_CTRL);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_MASK_NS);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_S);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_NS);
	writel(HISTB_VDEC_BASIC_CFG0_VP8 | (total_mbs - 1),
	       vdec->regs + HISTB_VDEC_BASIC_CFG0);
	writel(HISTB_VDEC_BASIC_CFG1_VP8,
	       vdec->regs + HISTB_VDEC_BASIC_CFG1);
	writel(lower_32_bits(pic_msg), vdec->regs + HISTB_VDEC_AVM_ADDR);
	writel(lower_32_bits(up_msg), vdec->regs + HISTB_VDEC_VAM_ADDR);
	writel(lower_32_bits(round_down(src_dma, 16) - 16),
	       vdec->regs + HISTB_VDEC_STREAM_BASE);
	writel(0, vdec->regs + HISTB_VDEC_SCD_AVS_FLAG);
	writel(1, vdec->regs + HISTB_VDEC_SCD_VDH_SELRST);
	writel(HISTB_VDEC_SCD_EMAR_BASE |
	       (DIV_ROUND_UP(ctx->dst.pix.width, 16) <= 256 ?
		HISTB_VDEC_SCD_EMAR_ENABLE : 0),
	       vdec->regs + HISTB_VDEC_SCD_EMAR_CFG);
	for (offset = HISTB_VDEC_TIMEOUT_FIRST;
	     offset <= HISTB_VDEC_TIMEOUT_LAST; offset += 4)
		writel(HISTB_VDEC_TIMEOUT_VALUE, vdec->regs + offset);
	writel(lower_32_bits(round_down(decoded->tile.dma, 16)),
	       vdec->regs + HISTB_VDEC_CURRENT_Y);
	writel(histb_vdec_tile_stride(ctx->dst.pix.width) * 16,
	       vdec->regs + HISTB_VDEC_Y_STRIDE);
	writel(histb_vdec_tile_stride(ctx->dst.pix.width) *
	       ALIGN(ctx->dst.pix.height, 16),
	       vdec->regs + HISTB_VDEC_CHROMA_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_HEAD_INFO_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_LINE_NUM_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_Y_STRIDE_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_Y_OFFSET_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_CHROMA_OFFSET_2BIT);
	writel(lower_32_bits(histb_vdec_buffer_dma(ctx,
						   HISTB_VDEC_BUF_PPFD)),
	       vdec->regs + HISTB_VDEC_PPFD_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_PPFD_SIZE);
	writel(0, vdec->regs + HISTB_VDEC_REF_PIC_TYPE);
	writel(2, vdec->regs + HISTB_VDEC_FF_APT_ENABLE);
	writel(0xaaaaaaaa, vdec->regs + HISTB_VDEC_DOWN_CLK_CFG);
	writel(HISTB_VDEC_INT_ENABLE_DONE, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(3, vdec->regs + HISTB_VDEC_SCD_CLOCK_GATE);
	usleep_range(30, 60);
	/* Publish the coherent message and bitstream contents before VDH starts. */
	wmb();
	writel(0, vdec->regs + HISTB_VDEC_START);
	writel(1, vdec->regs + HISTB_VDEC_START);
	writel(0, vdec->regs + HISTB_VDEC_START);
}

static void histb_vdec_program_vp9_registers(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		dma_addr_t src_dma, u32 total_mis)
{
	struct histb_vdec_dev *vdec = ctx->vdec;
	dma_addr_t msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	dma_addr_t pic_msg = msg_dma +
		HISTB_VDEC_PIC_MSG_SLOT * HISTB_VDEC_H264_MSG_SLOT_SIZE;
	dma_addr_t up_msg = msg_dma +
		HISTB_VDEC_UP_MSG_SLOT * HISTB_VDEC_H264_MSG_SLOT_SIZE;
	u32 offset;

	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	writel(0, vdec->regs + HISTB_VDEC_SMMU_CTRL);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_MASK_NS);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_S);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_NS);
	writel(HISTB_VDEC_BASIC_CFG0_VP9 | (total_mis - 1),
	       vdec->regs + HISTB_VDEC_BASIC_CFG0);
	writel(HISTB_VDEC_BASIC_CFG1_VP9,
	       vdec->regs + HISTB_VDEC_BASIC_CFG1);
	writel(lower_32_bits(pic_msg), vdec->regs + HISTB_VDEC_AVM_ADDR);
	writel(lower_32_bits(up_msg), vdec->regs + HISTB_VDEC_VAM_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_STREAM_BASE);
	writel(0, vdec->regs + HISTB_VDEC_SCD_AVS_FLAG);
	writel(1, vdec->regs + HISTB_VDEC_SCD_VDH_SELRST);
	writel(HISTB_VDEC_SCD_EMAR_BASE |
	       (DIV_ROUND_UP(ctx->dst.pix.width, 8) <= 256 ?
		HISTB_VDEC_SCD_EMAR_ENABLE : 0),
	       vdec->regs + HISTB_VDEC_SCD_EMAR_CFG);
	for (offset = HISTB_VDEC_TIMEOUT_FIRST;
	     offset <= HISTB_VDEC_TIMEOUT_LAST; offset += 4)
		writel(HISTB_VDEC_TIMEOUT_VALUE, vdec->regs + offset);
	writel(ALIGN(ctx->dst.pix.height, 64),
	       vdec->regs + HISTB_VDEC_STORE_PARAM);
	writel(lower_32_bits(round_down(decoded->tile.dma, 16)),
	       vdec->regs + HISTB_VDEC_CURRENT_Y);
	writel(histb_vdec_surface_stride(ctx) * 16,
	       vdec->regs + HISTB_VDEC_Y_STRIDE);
	writel(histb_vdec_surface_stride(ctx) *
	       ALIGN(ctx->dst.pix.height, histb_vdec_surface_height_align(ctx)),
	       vdec->regs + HISTB_VDEC_CHROMA_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_HEAD_INFO_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_LINE_NUM_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_Y_STRIDE_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_Y_OFFSET_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_CHROMA_OFFSET_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_PPFD_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_PPFD_SIZE);
	writel(0, vdec->regs + HISTB_VDEC_REF_PIC_TYPE);
	writel(2, vdec->regs + HISTB_VDEC_FF_APT_ENABLE);
	writel(0xaaaaaaaa, vdec->regs + HISTB_VDEC_DOWN_CLK_CFG);
	writel(HISTB_VDEC_INT_ENABLE_DONE, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(3, vdec->regs + HISTB_VDEC_SCD_CLOCK_GATE);
	usleep_range(30, 60);
	/* Publish the coherent message and bitstream contents before VDH starts. */
	wmb();
	writel(0, vdec->regs + HISTB_VDEC_START);
	writel(1, vdec->regs + HISTB_VDEC_START);
	writel(0, vdec->regs + HISTB_VDEC_START);
}

static void histb_vdec_program_registers(struct histb_vdec_ctx *ctx,
						 struct histb_vdec_decoded_buffer *decoded,
						 dma_addr_t src_dma,
						 u32 total_mbs, bool mbaff,
						 bool field_pic)
{
	struct histb_vdec_dev *vdec = ctx->vdec;
	dma_addr_t msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	dma_addr_t pic_msg = msg_dma +
		HISTB_VDEC_PIC_MSG_SLOT * HISTB_VDEC_H264_MSG_SLOT_SIZE;
	dma_addr_t up_msg = msg_dma +
		HISTB_VDEC_UP_MSG_SLOT * HISTB_VDEC_H264_MSG_SLOT_SIZE;
	u32 offset;

	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	writel(0, vdec->regs + HISTB_VDEC_SMMU_CTRL);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_MASK_NS);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_S);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_NS);

	writel(HISTB_VDEC_BASIC_CFG0_H264 | (total_mbs - 1),
	       vdec->regs + HISTB_VDEC_BASIC_CFG0);
	writel((HISTB_VDEC_BASIC_CFG1_I_SLICE &
		~HISTB_VDEC_BASIC_CFG1_NEW_PIC) |
	       (ctx->h264_new_frame ? HISTB_VDEC_BASIC_CFG1_NEW_PIC : 0) |
	       (mbaff ? HISTB_VDEC_BASIC_CFG1_MBAFF : 0),
	       vdec->regs + HISTB_VDEC_BASIC_CFG1);
	writel(lower_32_bits(pic_msg), vdec->regs + HISTB_VDEC_AVM_ADDR);
	writel(lower_32_bits(up_msg), vdec->regs + HISTB_VDEC_VAM_ADDR);
	writel(lower_32_bits(round_down(src_dma, 16)),
	       vdec->regs + HISTB_VDEC_STREAM_BASE);
	writel(0, vdec->regs + HISTB_VDEC_SCD_AVS_FLAG);
	writel(1, vdec->regs + HISTB_VDEC_SCD_VDH_SELRST);
	writel(HISTB_VDEC_SCD_EMAR_BASE |
	       (DIV_ROUND_UP(ctx->dst.pix.width, 16) <= 256 ?
		HISTB_VDEC_SCD_EMAR_ENABLE : 0),
	       vdec->regs + HISTB_VDEC_SCD_EMAR_CFG);
	for (offset = HISTB_VDEC_TIMEOUT_FIRST;
	     offset <= HISTB_VDEC_TIMEOUT_LAST; offset += 4)
		writel(HISTB_VDEC_TIMEOUT_VALUE, vdec->regs + offset);

	writel(lower_32_bits(round_down(decoded->tile.dma, 16)),
	       vdec->regs + HISTB_VDEC_CURRENT_Y);
	writel(histb_vdec_tile_stride(ctx->dst.pix.width) * 16,
	       vdec->regs + HISTB_VDEC_Y_STRIDE);
	writel(histb_vdec_tile_stride(ctx->dst.pix.width) *
	       ALIGN(ctx->dst.pix.height, 16),
	       vdec->regs + HISTB_VDEC_CHROMA_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_HEAD_INFO_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_LINE_NUM_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_Y_STRIDE_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_Y_OFFSET_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_CHROMA_OFFSET_2BIT);
	writel(lower_32_bits(histb_vdec_buffer_dma(ctx,
						   HISTB_VDEC_BUF_PPFD)),
	       vdec->regs + HISTB_VDEC_PPFD_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_PPFD_SIZE);
	writel(0, vdec->regs + HISTB_VDEC_REF_PIC_TYPE);
	/* g_StructTrans selects 0 for field/MBAFF and 2 for frame pictures. */
	writel((field_pic || mbaff) ? 0 : 2,
	       vdec->regs + HISTB_VDEC_FF_APT_ENABLE);
	writel(0xaaaaaaaa, vdec->regs + HISTB_VDEC_DOWN_CLK_CFG);
	writel(HISTB_VDEC_INT_ENABLE_DONE, vdec->regs + HISTB_VDEC_INT_MASK);

	writel(3, vdec->regs + HISTB_VDEC_SCD_CLOCK_GATE);
	usleep_range(30, 60);
	/* Make coherent messages and all register writes visible before start. */
	wmb();
	writel(0, vdec->regs + HISTB_VDEC_START);
	writel(1, vdec->regs + HISTB_VDEC_START);
	writel(0, vdec->regs + HISTB_VDEC_START);
}

static void
histb_vdec_program_avs_registers(struct histb_vdec_ctx *ctx,
				 struct histb_vdec_decoded_buffer *decoded,
				 dma_addr_t src_dma, u32 total_mbs)
{
	struct histb_vdec_dev *vdec = ctx->vdec;
	dma_addr_t msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	dma_addr_t pic_msg = msg_dma + HISTB_VDEC_AVS_PIC_MSG_OFFSET;
	dma_addr_t up_msg = msg_dma;
	dma_addr_t dnr_mbinfo;
	u32 stride = histb_vdec_surface_stride(ctx);
	u32 chroma_offset;
	u32 offset;

	chroma_offset = stride * ALIGN(ctx->dst.pix.height,
					      histb_vdec_surface_height_align(ctx));
	dnr_mbinfo = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_AVS_DNR_MBINFO);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	histb_vdec_program_smmu(vdec, true);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_MASK_NS);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_S);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_NS);
	writel(HISTB_VDEC_BASIC_CFG0_AVS | (total_mbs - 1),
	       vdec->regs + HISTB_VDEC_BASIC_CFG0);
	writel(ctx->avs_basic_cfg1 | HISTB_VDEC_BASIC_CFG1_AVS_MMU_EN,
	       vdec->regs + HISTB_VDEC_BASIC_CFG1);
	writel(lower_32_bits(pic_msg), vdec->regs + HISTB_VDEC_AVM_ADDR);
	writel(lower_32_bits(up_msg), vdec->regs + HISTB_VDEC_VAM_ADDR);
	writel(lower_32_bits(round_down(src_dma, 16)),
	       vdec->regs + HISTB_VDEC_STREAM_BASE);
	writel(1, vdec->regs + HISTB_VDEC_SCD_AVS_FLAG);
	writel(1, vdec->regs + HISTB_VDEC_SCD_VDH_SELRST);
	writel(HISTB_VDEC_SCD_EMAR_BASE |
	       (DIV_ROUND_UP(ctx->dst.pix.width, 16) <= 120 ?
		HISTB_VDEC_SCD_EMAR_ENABLE : 0),
	       vdec->regs + HISTB_VDEC_SCD_EMAR_CFG);
	for (offset = HISTB_VDEC_TIMEOUT_FIRST;
	     offset <= HISTB_VDEC_TIMEOUT_LAST; offset += 4)
		writel(HISTB_VDEC_TIMEOUT_VALUE, vdec->regs + offset);

	writel(lower_32_bits(round_down(decoded->tile.dma, 16)),
	       vdec->regs + HISTB_VDEC_CURRENT_Y);
	writel(stride * 16, vdec->regs + HISTB_VDEC_Y_STRIDE);
	writel(chroma_offset, vdec->regs + HISTB_VDEC_CHROMA_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_HEAD_INFO_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_STORE_PARAM);
	writel(0, vdec->regs + HISTB_VDEC_LINE_NUM_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_Y_STRIDE_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_Y_OFFSET_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_CHROMA_OFFSET_2BIT);
	writel(0, vdec->regs + HISTB_VDEC_PPFD_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_PPFD_SIZE);
	writel(lower_32_bits(dnr_mbinfo),
	       vdec->regs + HISTB_VDEC_DNR_MBINFO_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_REF_PIC_TYPE);
	writel(0, vdec->regs + HISTB_VDEC_FF_APT_ENABLE);
	writel(0xaaaaaaaa, vdec->regs + HISTB_VDEC_DOWN_CLK_CFG);
	writel(HISTB_VDEC_INT_ENABLE_DONE, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(3, vdec->regs + HISTB_VDEC_SCD_CLOCK_GATE);
	usleep_range(30, 60);
	/* Publish coherent messages and all register writes before VDH starts. */
	wmb();
	writel(0, vdec->regs + HISTB_VDEC_START);
	writel(1, vdec->regs + HISTB_VDEC_START);
	writel(0, vdec->regs + HISTB_VDEC_START);
}

static void histb_vdec_program_hevc_registers(
		struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		dma_addr_t src_dma, u32 total_ctbs)
{
	struct histb_vdec_dev *vdec = ctx->vdec;
	dma_addr_t msg_dma = histb_vdec_buffer_dma(ctx, HISTB_VDEC_BUF_MSG);
	dma_addr_t pic_msg = msg_dma +
		HISTB_VDEC_PIC_MSG_SLOT * HISTB_VDEC_HEVC_MSG_SLOT_SIZE;
	dma_addr_t up_msg = msg_dma +
		HISTB_VDEC_UP_MSG_SLOT * HISTB_VDEC_HEVC_MSG_SLOT_SIZE;
	u32 offset;

	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	writel(0, vdec->regs + HISTB_VDEC_SMMU_CTRL);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_MASK_NS);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_S);
	writel(7, vdec->regs + HISTB_VDEC_SMMU_INT_CLEAR_NS);
	writel(HISTB_VDEC_BASIC_CFG0_HEVC |
	       (ctx->hevc_scaling_list ? BIT(30) : 0) | (total_ctbs - 1),
	       vdec->regs + HISTB_VDEC_BASIC_CFG0);
	writel(HISTB_VDEC_BASIC_CFG1_HEVC,
	       vdec->regs + HISTB_VDEC_BASIC_CFG1);
	writel(lower_32_bits(pic_msg), vdec->regs + HISTB_VDEC_AVM_ADDR);
	writel(lower_32_bits(up_msg), vdec->regs + HISTB_VDEC_VAM_ADDR);
	writel(lower_32_bits(round_down(src_dma, 16)),
	       vdec->regs + HISTB_VDEC_STREAM_BASE);
	writel(0, vdec->regs + HISTB_VDEC_SCD_AVS_FLAG);
	writel(1, vdec->regs + HISTB_VDEC_SCD_VDH_SELRST);
	writel(HISTB_VDEC_SCD_EMAR_BASE |
	       (DIV_ROUND_UP(ctx->dst.pix.width, 16) <= 256 ?
		HISTB_VDEC_SCD_EMAR_ENABLE : 0),
	       vdec->regs + HISTB_VDEC_SCD_EMAR_CFG);
	for (offset = HISTB_VDEC_TIMEOUT_FIRST;
	     offset <= HISTB_VDEC_TIMEOUT_LAST; offset += 4)
		writel(HISTB_VDEC_TIMEOUT_VALUE, vdec->regs + offset);
	writel(lower_32_bits(round_down(decoded->tile.dma, 16)),
	       vdec->regs + HISTB_VDEC_CURRENT_Y);
	writel(histb_vdec_surface_stride(ctx) * 16,
	       vdec->regs + HISTB_VDEC_Y_STRIDE);
	writel(histb_vdec_surface_stride(ctx) *
	       ALIGN(ctx->dst.pix.height, histb_vdec_surface_height_align(ctx)),
	       vdec->regs + HISTB_VDEC_CHROMA_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_HEAD_INFO_OFFSET);
	writel(0, vdec->regs + HISTB_VDEC_STORE_PARAM);
	writel(0, vdec->regs + HISTB_VDEC_LINE_NUM_ADDR);
	if (ctx->hevc_main10) {
		u32 stride = histb_vdec_surface_stride(ctx);
		u32 height = ALIGN(ctx->dst.pix.height,
				   histb_vdec_surface_height_align(ctx));

		writel(stride * 8,
		       vdec->regs + HISTB_VDEC_Y_STRIDE_2BIT);
		writel(stride * height * 3 / 2,
		       vdec->regs + HISTB_VDEC_Y_OFFSET_2BIT);
		writel((stride / 4) * height,
		       vdec->regs + HISTB_VDEC_CHROMA_OFFSET_2BIT);
	} else {
		writel(0, vdec->regs + HISTB_VDEC_Y_STRIDE_2BIT);
		writel(0, vdec->regs + HISTB_VDEC_Y_OFFSET_2BIT);
		writel(0, vdec->regs + HISTB_VDEC_CHROMA_OFFSET_2BIT);
	}
	writel(0, vdec->regs + HISTB_VDEC_PPFD_ADDR);
	writel(0, vdec->regs + HISTB_VDEC_PPFD_SIZE);
	writel(0, vdec->regs + HISTB_VDEC_REF_PIC_TYPE);
	writel(2, vdec->regs + HISTB_VDEC_FF_APT_ENABLE);
	writel(0xaaaaaaaa, vdec->regs + HISTB_VDEC_DOWN_CLK_CFG);
	writel(HISTB_VDEC_INT_ENABLE_DONE, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(3, vdec->regs + HISTB_VDEC_SCD_CLOCK_GATE);
	usleep_range(30, 60);
	/* Publish coherent message writes before ringing the VDH doorbell. */
	wmb();
	writel(0, vdec->regs + HISTB_VDEC_START);
	writel(1, vdec->regs + HISTB_VDEC_START);
	writel(0, vdec->regs + HISTB_VDEC_START);
}

static void histb_vdec_complete_request(struct histb_vdec_ctx *ctx,
					struct media_request *request)
{
	if (request)
		v4l2_ctrl_request_complete(request, &ctx->ctrl_handler);
}

static void histb_vdec_run_vc1_job(struct histb_vdec_ctx *ctx,
		struct histb_vdec_decoded_buffer *decoded,
		struct vb2_v4l2_buffer *src, unsigned long payload)
{
	struct histb_vdec_dev *vdec = ctx->vdec;
	dma_addr_t src_dma = 0;
	unsigned long flags;
	unsigned int i;
	bool active = false, cancelled, commit_ok, pm_active = false;
	bool reset_bpd = false;
	u32 total_mbs;
	int capture_ret, reset_ret, ret;

	ctx->vc1 = true;
	ctx->vc1_annex_l =
		ctx->src.pix.pixelformat == V4L2_PIX_FMT_VC1_ANNEX_L;
	ctx->hevc = false;
	ctx->hevc_main10 = false;
	ctx->hevc_scaling_list = false;
	ctx->hevc_slices = 0;
	ctx->mpeg2 = false;
	ctx->mpeg2_slices = 0;
	ctx->mpeg4 = false;
	ctx->vp8 = false;
	ctx->vp9 = false;
	ctx->cabac = false;
	ctx->mbaff = false;

	reinit_completion(&ctx->vc1_setup_idle);
	mutex_lock(&vdec->launch_lock);
	spin_lock_irqsave(&vdec->irqlock, flags);
	if (!vdec->curr_ctx) {
		vdec->curr_ctx = ctx;
		vdec->phase = HISTB_VDEC_PHASE_BPD;
		vdec->job_cancelled = false;
		vdec->job_count_pending = true;
		active = true;
	} else {
		ret = -EBUSY;
	}
	spin_unlock_irqrestore(&vdec->irqlock, flags);
	mutex_unlock(&vdec->launch_lock);
	if (!active)
		goto finish_no_pm;

	ret = pm_runtime_resume_and_get(vdec->dev);
	if (ret < 0)
		goto finish_active;
	pm_active = true;
	ret = histb_vdec_configure_vdh_clock(vdec);
	if (ret) {
		dev_err(vdec->dev, "failed to configure VC-1 VDH clock: %d\n", ret);
		goto finish_active;
	}

	ret = ctx->vc1_annex_l ?
		histb_vdec_prepare_vc1_smp_syntax(ctx, decoded, src, payload,
						   &src_dma) :
		histb_vdec_prepare_vc1_syntax(ctx, decoded, src, payload, &src_dma);
	if (ret == HISTB_VDEC_VC1_AU_DONE) {
		mutex_lock(&vdec->launch_lock);
		histb_vdec_take_job(vdec, ctx);
		mutex_unlock(&vdec->launch_lock);
		pm_runtime_mark_last_busy(vdec->dev);
		pm_runtime_put_autosuspend(vdec->dev);
		mutex_lock(&ctx->vc1_lock);
		histb_vdec_finish_vc1_au_no_capture(ctx);
		mutex_unlock(&ctx->vc1_lock);
		complete(&ctx->vc1_setup_idle);
		return;
	}
	if (ret)
		goto finish_active;

	total_mbs = DIV_ROUND_UP(ctx->vc1_annex_l ?
		ctx->vc1_pending_picture.coded_width : ctx->vc1_entry.coded_width, 16);
	if (ctx->vc1_pending_picture.fcm == HISTB_VC1_FIELD_INTERLACED)
		total_mbs *= DIV_ROUND_UP(
			DIV_ROUND_UP(ctx->vc1_entry.coded_height, 16), 2);
	else
		total_mbs *= DIV_ROUND_UP(ctx->vc1_annex_l ?
			ctx->vc1_pending_picture.coded_height :
			ctx->vc1_entry.coded_height, 16);
	if (!total_mbs) {
		ret = -EINVAL;
		goto finish_active;
	}
	ctx->total_mbs = total_mbs;
	if (ctx->vc1_pending_picture.fcm == HISTB_VC1_FIELD_INTERLACED &&
	    (ctx->vc1_field_transaction.first_ptype == HISTB_VC1_PICTURE_P ||
	     ctx->vc1_field_transaction.second_ptype == HISTB_VC1_PICTURE_P)) {
		ctx->frame_flags = V4L2_H264_DECODE_PARAM_FLAG_PFRAME;
	} else {
		switch (ctx->vc1_pending_picture.ptype) {
		case HISTB_VC1_PICTURE_I:
			ctx->frame_flags = 0;
			break;
		case HISTB_VC1_PICTURE_P:
			ctx->frame_flags = V4L2_H264_DECODE_PARAM_FLAG_PFRAME;
			break;
		default:
			ctx->frame_flags = V4L2_H264_DECODE_PARAM_FLAG_BFRAME;
			break;
		}
	}
	if (ctx->vc1_pending_picture.skipped) {
		ret = histb_vdec_copy_vc1_skipped_surface(ctx, decoded);
		capture_ret = ret ? ret : histb_vdec_copy_capture(ctx);

		mutex_lock(&ctx->vc1_lock);
		mutex_lock(&vdec->launch_lock);
		cancelled = !histb_vdec_take_job_in_phase(
			vdec, ctx, HISTB_VDEC_PHASE_BPD);
		mutex_unlock(&vdec->launch_lock);
		if (cancelled && !ret)
			ret = -ECANCELED;
		commit_ok = !ret && histb_vdec_commit_vc1_state(ctx);
		if (!commit_ok)
			dev_err_ratelimited(vdec->dev,
				"VC-1 SKIP fallback failed: copy=%d capture=%d cancelled=%u\n",
				ret, capture_ret, cancelled);
		if (!commit_ok) {
			histb_vdec_discard_vc1_current_locked(ctx, true);
			histb_vdec_reset_vc1_au(ctx);
		}
		if (commit_ok)
			histb_vdec_finish_vc1_job(ctx, !capture_ret);
		mutex_unlock(&ctx->vc1_lock);
		if (!commit_ok)
			histb_vdec_finish_vc1_error(ctx);
		complete(&ctx->vc1_setup_idle);
		return;
	}
	histb_vdec_sync_tile_for_device(ctx, decoded);
	for (i = 0; i < ARRAY_SIZE(ctx->vc1_ref); i++)
		histb_vdec_sync_vc1_anchor_for_device(ctx, &ctx->vc1_ref[i]);
	mutex_lock(&vdec->launch_lock);
	/* Keep the job in BPD phase while acknowledging any VDH status left by
	 * the previous job.  The IRQ thread takes launch_lock before sampling it. */
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	ret = histb_vdec_switch_job_phase(vdec, ctx,
			HISTB_VDEC_PHASE_BPD, HISTB_VDEC_PHASE_VDH);
	if (!ret) {
		schedule_delayed_work(&vdec->watchdog_work,
				      msecs_to_jiffies(HISTB_VDEC_WATCHDOG_MS));
		histb_vdec_program_vc1_registers(ctx, decoded);
	}
	mutex_unlock(&vdec->launch_lock);
	if (ret)
		goto finish_active;
	complete(&ctx->vc1_setup_idle);
	return;

	finish_active:
	mutex_lock(&vdec->launch_lock);
	spin_lock_irqsave(&vdec->irqlock, flags);
	reset_bpd = vdec->curr_ctx == ctx &&
		vdec->phase == HISTB_VDEC_PHASE_BPD;
	spin_unlock_irqrestore(&vdec->irqlock, flags);
	if (reset_bpd && pm_active) {
		reset_ret = histb_vdec_reset_vc1_bpd(vdec);
		if (reset_ret)
			ret = reset_ret;
	}
	histb_vdec_take_job(vdec, ctx);
	mutex_unlock(&vdec->launch_lock);
	histb_vdec_discard_vc1_current(ctx, true);
	if (pm_active) {
		pm_runtime_mark_last_busy(vdec->dev);
		pm_runtime_put_autosuspend(vdec->dev);
	}
finish_no_pm:
	dev_err_ratelimited(vdec->dev, "VC-1 setup failed: %d\n", ret);
	histb_vdec_finish_vc1_error_no_pm(ctx);
	complete(&ctx->vc1_setup_idle);
}

static void histb_vdec_device_run(void *priv)
{
	struct histb_vdec_ctx *ctx = priv;
	struct histb_vdec_dev *vdec = ctx->vdec;
	const struct v4l2_ctrl_h264_decode_params *decode;
	const struct v4l2_ctrl_h264_slice_params *slice;
	const struct v4l2_ctrl_h264_scaling_matrix *scaling;
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const struct v4l2_ctrl_h264_pred_weights *pred_weights;
	struct histb_vdec_decoded_buffer *decoded;
	struct vb2_v4l2_buffer *src, *dst;
	struct media_request *request;
	struct histb_vdec_hevc_stream *hevc_streams = NULL;
	struct histb_vdec_avs_stream *avs_streams = NULL;
	struct histb_vdec_mpeg2_slice *mpeg2_slices = NULL;
	dma_addr_t src_dma;
	unsigned long payload, flags, bytesused;
	bool mpeg4_state_failed = false;
	bool mpeg4_staged = false;
	bool vp9_staged = false;
	u32 total_mbs;
	int ret;

	wait_for_completion(&vdec->postprocess_idle);
	/* The completion is signalled by the worker after finish_job, but the
	 * work callback has not returned yet.  Flush that final interval before
	 * allowing a new IRQ/postprocess transaction to reuse the owner. */
	flush_work(&vdec->postprocess_work);
	complete(&vdec->postprocess_idle);

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (WARN_ON(!src || !dst)) {
		v4l2_m2m_job_finish(vdec->m2m_dev, ctx->fh.m2m_ctx);
		return;
	}
	decoded = histb_vdec_decoded_buffer(dst);
	ctx->mpeg2_reference_started = false;
	ctx->mpeg2_picture_started = false;
	ctx->mpeg2_picture_valid = false;

	request = src->vb2_buf.req_obj.req;
	if (request) {
		ret = v4l2_ctrl_request_setup(request, &ctx->ctrl_handler);
		if (ret)
			goto finish_request;
	} else if (ctx->src.pix.pixelformat != V4L2_PIX_FMT_MPEG4 &&
		   !histb_vdec_is_vc1_format(ctx->src.pix.pixelformat)) {
		ret = -EINVAL;
		goto finish_request;
	}
	bytesused = vb2_get_plane_payload(&src->vb2_buf, 0);
	if (src->vb2_buf.planes[0].data_offset > bytesused) {
		dev_err_ratelimited(vdec->dev,
				    "plane data_offset %u exceeds bytesused %lu\n",
				    src->vb2_buf.planes[0].data_offset,
				    bytesused);
		ret = -EINVAL;
		goto finish_request;
	}
	payload = bytesused - src->vb2_buf.planes[0].data_offset;
	ctx->avs = false;
	ctx->avs_slices = 0;
	ctx->avs_total_mbs = 0;
	ctx->avs_basic_cfg1 = 0;
	ctx->avs_profile = 0;
	ctx->vp9 = false;
	ctx->mpeg4 = false;
	ctx->vc1 = false;
	/*
	 * Isolating stale VDH/FSP state between HEVC pictures.
	 *
	 * This used to pulse the CRG MFD reset and wait for its acknowledge on
	 * every picture, and the acknowledge never arrives: the ACK register
	 * reports the VDH, SCD and BPD resets as done (bits 0/1/3) while the
	 * MFD bit (2) stays clear, so each frame burned the full 30 ms poll
	 * budget and aborted with -ETIMEDOUT.  HEVC then decoded at 0.592x
	 * with 1401 timeouts over three runs, while h264/mpeg2/vp8/vp9 on the
	 * same path never reset and were clean at 3.55-3.83x.
	 *
	 * The vendor pulses the same bit with the same 30 ms budget, but only
	 * when a channel is torn down or opened, never per picture, and it
	 * guards the pulse with the VDH clock enable.  Per picture the vendor
	 * instead uses the selective reset that this driver already writes as
	 * SCD_VDH_SELRST - so the per-picture MFD reset here was redundant as
	 * well as failing.
	 *
	 * Kept behind a parameter rather than deleted, because no git history
	 * survives that would say what it was added for: if disabling it
	 * regresses a stream, that is the evidence needed to investigate
	 * properly, and the diagnostics below report the clock state.
	 */
	/*
	 * There used to be a CRG MFD reset pulse here, once per HEVC picture,
	 * waiting on an acknowledge bit that never asserts: the ACK register
	 * reports the VDH, SCD and BPD resets done while the MFD bit stays
	 * clear, so every frame burned the full 30 ms poll budget and aborted
	 * with -ETIMEDOUT.  HEVC decoded at 0.592x with 1401 timeouts over
	 * three runs while h264/mpeg2/vp8/vp9, which never reset, were clean
	 * at 3.55-3.83x.
	 *
	 * The vendor pulses the same bit with the same budget only at channel
	 * teardown and open, guarded by the VDH clock enable; per picture it
	 * uses the selective reset this driver already writes as
	 * SCD_VDH_SELRST.  Removing the pulse took HEVC to 3.47x with zero
	 * timeouts and a decoded frame hash identical to the previous
	 * behaviour.
	 */
	if (histb_vdec_is_vc1_format(ctx->src.pix.pixelformat)) {
		histb_vdec_complete_request(ctx, request);
		histb_vdec_run_vc1_job(ctx, decoded, src, payload);
		return;
	}
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG4) {
		mutex_lock(&ctx->mpeg4_lock);
		ret = histb_vdec_prepare_mpeg4(ctx, decoded, src, payload,
						&src_dma);
		if (ret == HISTB_VDEC_MPEG4_AU_DONE) {
			ctx->mpeg4 = true;
			histb_vdec_finish_mpeg4_au_no_capture(ctx);
			mutex_unlock(&ctx->mpeg4_lock);
			return;
		}
		if (ret) {
			histb_vdec_fail_mpeg4_state_locked(ctx);
			mpeg4_state_failed = true;
			mutex_unlock(&ctx->mpeg4_lock);
			goto finish_request;
		}
		mpeg4_staged = true;
		total_mbs = DIV_ROUND_UP(ctx->mpeg4_pending_frame.vol.width, 16) *
			    DIV_ROUND_UP(ctx->mpeg4_pending_frame.vol.height, 16);
		ctx->total_mbs = total_mbs;
		switch (ctx->mpeg4_pending_frame.vop.coding_type) {
		case HISTB_MPEG4_B_VOP:
			ctx->frame_flags = V4L2_H264_DECODE_PARAM_FLAG_BFRAME;
			break;
		case HISTB_MPEG4_P_VOP:
		case HISTB_MPEG4_S_VOP:
			ctx->frame_flags = V4L2_H264_DECODE_PARAM_FLAG_PFRAME;
			break;
		default:
			ctx->frame_flags = 0;
			break;
		}
		ctx->hevc = false;
		ctx->hevc_main10 = false;
		ctx->hevc_scaling_list = false;
		ctx->hevc_slices = 0;
		ctx->mpeg2 = false;
		ctx->mpeg2_slices = 0;
		ctx->mpeg4 = true;
		ctx->vp8 = false;
		ctx->cabac = false;
		ctx->mbaff = false;
		mutex_unlock(&ctx->mpeg4_lock);
		goto start_hardware;
	}
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_AVS_SLICE) {
		struct v4l2_ctrl *avs_slice_ctrl;
		const struct v4l2_ctrl_avs_sequence *avs_sequence;
		const struct v4l2_ctrl_avs_picture *avs_picture;
		const struct v4l2_ctrl_avs_slice_params *avs_slices;
		const struct v4l2_ctrl_avs_decode_params *avs_decode;
		const u8 *payload_data;
		unsigned int num_slices;

		avs_sequence = histb_vdec_ctrl_data(ctx,
						    V4L2_CID_STATELESS_AVS_SEQUENCE);
		avs_picture = histb_vdec_ctrl_data(ctx,
						   V4L2_CID_STATELESS_AVS_PICTURE);
		avs_slice_ctrl = v4l2_ctrl_find(&ctx->ctrl_handler,
						V4L2_CID_STATELESS_AVS_SLICE_PARAMS);
		avs_slices = avs_slice_ctrl ? avs_slice_ctrl->p_cur.p : NULL;
		num_slices = avs_slice_ctrl ? avs_slice_ctrl->elems : 0;
		avs_decode = histb_vdec_ctrl_data(ctx,
						  V4L2_CID_STATELESS_AVS_DECODE_PARAMS);
		payload_data = vb2_plane_vaddr(&src->vb2_buf, 0);
		if (payload_data)
			payload_data += src->vb2_buf.planes[0].data_offset;
		ret = histb_vdec_validate_avs(ctx, avs_sequence, avs_picture,
					      avs_slices, num_slices, avs_decode,
					      payload_data, payload);
		if (ret)
			goto finish_request;
		ret = histb_vdec_set_decoded_buffer_sizes(ctx, decoded);
		if (ret)
			goto finish_request;
		if (!histb_vdec_dma_buffers_valid(ctx)) {
			ret = -EINVAL;
			goto finish_request;
		}
		if (!histb_vdec_smmu_dma_buffers_valid(ctx)) {
			ret = -ERANGE;
			goto finish_request;
		}
		avs_streams = kcalloc(num_slices, sizeof(*avs_streams), GFP_KERNEL);
		if (!avs_streams) {
			ret = -ENOMEM;
			goto finish_request;
		}
		ret = histb_vdec_copy_avs_bitstream(ctx, src, payload, avs_slices,
						    num_slices, &src_dma, avs_streams);
		if (ret)
			goto finish_request;
		ret = histb_vdec_prepare_avs_messages(ctx, decoded, avs_sequence,
						      avs_picture, avs_decode, avs_slices,
						      num_slices, src_dma, avs_streams);
		if (ret)
			goto finish_request;

		total_mbs = DIV_ROUND_UP(avs_sequence->horizontal_size, 16) *
			histb_vdec_avs_height_mbs(avs_sequence->vertical_size,
						  avs_sequence->flags &
						  V4L2_AVS_SEQUENCE_FLAG_PROGRESSIVE);
		ctx->total_mbs = total_mbs;
		ctx->avs_total_mbs = total_mbs;
		ctx->avs_slices = num_slices;
		ctx->avs_profile = avs_sequence->profile_id;
		ctx->avs_basic_cfg1 = HISTB_VDEC_BASIC_CFG1_AVS;
		if (avs_picture->picture_coding_type != V4L2_AVS_PICTURE_TYPE_B)
			ctx->avs_basic_cfg1 |= HISTB_VDEC_BASIC_CFG1_AVS_MV_OUT;
		switch (avs_picture->picture_coding_type) {
		case V4L2_AVS_PICTURE_TYPE_P:
			ctx->frame_flags = V4L2_H264_DECODE_PARAM_FLAG_PFRAME;
			break;
		case V4L2_AVS_PICTURE_TYPE_B:
			ctx->frame_flags = V4L2_H264_DECODE_PARAM_FLAG_BFRAME;
			break;
		default:
			ctx->frame_flags = 0;
			break;
		}
		ctx->avs = true;
		ctx->hevc = false;
		ctx->hevc_main10 = false;
		ctx->hevc_scaling_list = false;
		ctx->hevc_slices = 0;
		ctx->mpeg2 = false;
		ctx->mpeg2_slices = 0;
		ctx->vp8 = false;
		ctx->vp9 = false;
		ctx->cabac = false;
		ctx->mbaff = false;
		histb_vdec_complete_request(ctx, request);
		kfree(avs_streams);
		avs_streams = NULL;
		goto start_hardware;
	}
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_HEVC_SLICE) {
		struct v4l2_ctrl *hevc_entry_ctrl, *hevc_slice_ctrl;
		const struct v4l2_ctrl_hevc_decode_params *hevc_decode;
		const struct v4l2_ctrl_hevc_slice_params *hevc_slice;
		const struct v4l2_ctrl_hevc_sps *hevc_sps;
		const struct v4l2_ctrl_hevc_pps *hevc_pps;
		const struct v4l2_ctrl_hevc_scaling_matrix *hevc_scaling;
		unsigned int num_slices;

		hevc_decode = histb_vdec_ctrl_data(ctx,
						   V4L2_CID_STATELESS_HEVC_DECODE_PARAMS);
		hevc_slice_ctrl = v4l2_ctrl_find(&ctx->ctrl_handler,
						 V4L2_CID_STATELESS_HEVC_SLICE_PARAMS);
		hevc_slice = hevc_slice_ctrl ? hevc_slice_ctrl->p_cur.p : NULL;
		num_slices = hevc_slice_ctrl ? hevc_slice_ctrl->elems : 0;
		hevc_sps = histb_vdec_ctrl_data(ctx,
						V4L2_CID_STATELESS_HEVC_SPS);
		hevc_pps = histb_vdec_ctrl_data(ctx,
						V4L2_CID_STATELESS_HEVC_PPS);
		hevc_scaling = histb_vdec_ctrl_data(
			ctx, V4L2_CID_STATELESS_HEVC_SCALING_MATRIX);
		hevc_entry_ctrl = v4l2_ctrl_find(&ctx->ctrl_handler,
						 V4L2_CID_STATELESS_HEVC_ENTRY_POINT_OFFSETS);
		if (!hevc_decode || !hevc_slice || !num_slices || !hevc_sps ||
		    !hevc_pps || !hevc_scaling) {
			ret = -EINVAL;
			goto finish_request;
		}
		ret = histb_vdec_validate_hevc_frame(ctx, hevc_decode, hevc_sps,
						     hevc_pps, hevc_slice, num_slices,
					      hevc_entry_ctrl ?
					      hevc_entry_ctrl->p_cur.p_u32 : NULL,
					      hevc_entry_ctrl ?
					      hevc_entry_ctrl->elems : 0,
					      payload);
		if (ret) {
			dev_dbg(vdec->dev, "HEVC validation failed: %d\n", ret);
			goto finish_request;
		}
		ctx->hevc = true;
		ctx->hevc_main10 = hevc_sps->bit_depth_luma_minus8 == 2;
		ctx->hevc_scaling_list = hevc_sps->flags &
			V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED;
		ctx->hevc_slices = num_slices;
		ctx->mpeg2 = false;
		ctx->mpeg2_slices = 0;
		ctx->vp8 = false;
		ctx->cabac = false;
		ctx->mbaff = false;
		ret = histb_vdec_set_decoded_buffer_sizes(ctx, decoded);
		if (ret)
			goto finish_request;
		if (!histb_vdec_dma_buffers_valid(ctx)) {
			ret = -EINVAL;
			goto finish_request;
		}
		hevc_streams = kcalloc(num_slices, sizeof(*hevc_streams), GFP_KERNEL);
		if (!hevc_streams) {
			ret = -ENOMEM;
			goto finish_request;
		}
		ret = histb_vdec_copy_hevc_bitstream(ctx, src, payload,
						     hevc_slice, num_slices, &src_dma, hevc_streams);
		if (ret) {
			dev_dbg(vdec->dev, "HEVC bitstream copy failed: %d\n", ret);
			goto finish_request;
		}
		ret = histb_vdec_prepare_hevc_messages(ctx, decoded,
						       hevc_decode, hevc_sps, hevc_pps, hevc_slice,
				num_slices, hevc_scaling, src_dma, hevc_streams);
		if (ret) {
			dev_dbg(vdec->dev, "HEVC message preparation failed: %d\n",
				ret);
			goto finish_request;
		}
		kfree(hevc_streams);
		hevc_streams = NULL;
		total_mbs = DIV_ROUND_UP(hevc_sps->pic_width_in_luma_samples,
					 BIT(hevc_sps->log2_min_luma_coding_block_size_minus3 + 3 +
			    hevc_sps->log2_diff_max_min_luma_coding_block_size)) *
			DIV_ROUND_UP(hevc_sps->pic_height_in_luma_samples,
				     BIT(hevc_sps->log2_min_luma_coding_block_size_minus3 + 3 +
			    hevc_sps->log2_diff_max_min_luma_coding_block_size));
		ctx->total_mbs = total_mbs;
		ctx->frame_flags = 0;
		histb_vdec_complete_request(ctx, request);
		goto start_hardware;
	}
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG1_SLICE) {
		const struct v4l2_ctrl_mpeg1_sequence *mpeg1_sequence;
		const struct v4l2_ctrl_mpeg1_picture *mpeg1_picture;
		const struct v4l2_ctrl_mpeg1_quantisation *mpeg1_quantisation;
		struct v4l2_ctrl_mpeg2_sequence sequence = {
			.chroma_format = 1,
			.flags = V4L2_MPEG2_SEQ_FLAG_PROGRESSIVE,
		};
		struct v4l2_ctrl_mpeg2_picture picture = {
			.flags = V4L2_MPEG2_PIC_FLAG_FRAME_PRED_DCT |
				 V4L2_MPEG2_PIC_FLAG_PROGRESSIVE,
			.picture_structure = V4L2_MPEG2_PIC_FRAME,
		};
		struct v4l2_ctrl_mpeg2_quantisation quantisation = {};
		unsigned int num_slices;

		mpeg1_sequence = histb_vdec_ctrl_data(ctx,
						      V4L2_CID_STATELESS_MPEG1_SEQUENCE);
		mpeg1_picture = histb_vdec_ctrl_data(ctx,
						     V4L2_CID_STATELESS_MPEG1_PICTURE);
		mpeg1_quantisation = histb_vdec_ctrl_data(ctx,
							  V4L2_CID_STATELESS_MPEG1_QUANTISATION);
		ret = histb_vdec_validate_mpeg1(
			ctx, mpeg1_sequence, mpeg1_picture, mpeg1_quantisation);
		if (ret)
			goto finish_request;
		sequence.horizontal_size = mpeg1_sequence->horizontal_size;
		sequence.vertical_size = mpeg1_sequence->vertical_size;
		sequence.vbv_buffer_size = mpeg1_sequence->vbv_buffer_size;
		picture.backward_ref_ts = mpeg1_picture->backward_ref_ts;
		picture.forward_ref_ts = mpeg1_picture->forward_ref_ts;
		picture.picture_coding_type = mpeg1_picture->picture_coding_type;
		picture.f_code[0][0] = mpeg1_picture->f_code[0];
		picture.f_code[0][1] = mpeg1_picture->f_code[0];
		picture.f_code[1][0] = mpeg1_picture->f_code[1];
		picture.f_code[1][1] = mpeg1_picture->f_code[1];
		memcpy(quantisation.intra_quantiser_matrix,
		       mpeg1_quantisation->intra_quantiser_matrix,
		       sizeof(quantisation.intra_quantiser_matrix));
		memcpy(quantisation.non_intra_quantiser_matrix,
		       mpeg1_quantisation->non_intra_quantiser_matrix,
		       sizeof(quantisation.non_intra_quantiser_matrix));
		ret = histb_vdec_begin_mpeg2_picture(
			ctx, decoded, src->vb2_buf.timestamp, &sequence, &picture);
		if (ret)
			goto finish_request;
		ret = histb_vdec_set_decoded_buffer_sizes(ctx, decoded);
		if (ret)
			goto finish_request;
		if (!histb_vdec_dma_buffers_valid(ctx)) {
			ret = -EINVAL;
			goto finish_request;
		}
		ret = histb_vdec_copy_raw_bitstream(ctx, src, payload, &src_dma);
		if (ret)
			goto finish_request;
		mpeg2_slices = kcalloc(HISTB_VDEC_MPEG2_MAX_SLICES,
				       sizeof(*mpeg2_slices), GFP_KERNEL);
		if (!mpeg2_slices) {
			ret = -ENOMEM;
			goto finish_request;
		}
		total_mbs = ctx->mpeg2_picture_width_mbs *
			ctx->mpeg2_picture_height_mbs;
		ret = histb_vdec_parse_mpeg1_slices(
			ctx->bitstream.cpu, payload,
			DIV_ROUND_UP(sequence.horizontal_size, 16), total_mbs,
			mpeg2_slices, &num_slices);
		if (ret)
			goto finish_request;
		ret = histb_vdec_prepare_mpeg1_messages(ctx, decoded, &sequence,
							&picture, &quantisation,
							mpeg1_picture, mpeg2_slices,
							num_slices, payload);
		if (ret)
			goto finish_request;
		ctx->mpeg2_picture_valid = true;
		kfree(mpeg2_slices);
		mpeg2_slices = NULL;
		ctx->total_mbs = total_mbs;
		switch (mpeg1_picture->picture_coding_type) {
		case V4L2_MPEG1_PIC_CODING_TYPE_P:
			ctx->frame_flags = V4L2_H264_DECODE_PARAM_FLAG_PFRAME;
			break;
		case V4L2_MPEG1_PIC_CODING_TYPE_B:
			ctx->frame_flags = V4L2_H264_DECODE_PARAM_FLAG_BFRAME;
			break;
		default:
			ctx->frame_flags = 0;
			break;
		}
		ctx->hevc = false;
		ctx->hevc_main10 = false;
		ctx->hevc_scaling_list = false;
		ctx->hevc_slices = 0;
		ctx->mpeg2 = true;
		ctx->mpeg2_slices = num_slices;
		ctx->vp8 = false;
		ctx->cabac = false;
		ctx->mbaff = false;
		histb_vdec_complete_request(ctx, request);
		goto start_hardware;
	}
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG2_SLICE) {
		const struct v4l2_ctrl_mpeg2_sequence *mpeg2_sequence;
		const struct v4l2_ctrl_mpeg2_picture *mpeg2_picture;
		const struct v4l2_ctrl_mpeg2_quantisation *mpeg2_quantisation;
		unsigned int num_slices;

		mpeg2_sequence = histb_vdec_ctrl_data(ctx,
						      V4L2_CID_STATELESS_MPEG2_SEQUENCE);
		mpeg2_picture = histb_vdec_ctrl_data(ctx,
						     V4L2_CID_STATELESS_MPEG2_PICTURE);
		mpeg2_quantisation = histb_vdec_ctrl_data(ctx,
							  V4L2_CID_STATELESS_MPEG2_QUANTISATION);
		ret = histb_vdec_validate_mpeg2(ctx, mpeg2_sequence,
						mpeg2_picture, mpeg2_quantisation);
		if (ret)
			goto finish_request;
		ret = histb_vdec_begin_mpeg2_picture(ctx, decoded,
						      src->vb2_buf.timestamp,
						      mpeg2_sequence, mpeg2_picture);
		if (ret)
			goto finish_request;
		ret = histb_vdec_set_decoded_buffer_sizes(ctx, decoded);
		if (ret)
			goto finish_request;
		if (!histb_vdec_dma_buffers_valid(ctx)) {
			ret = -EINVAL;
			goto finish_request;
		}
		ret = histb_vdec_copy_raw_bitstream(ctx, src, payload, &src_dma);
		if (ret)
			goto finish_request;
		mpeg2_slices = kcalloc(HISTB_VDEC_MPEG2_MAX_SLICES,
				       sizeof(*mpeg2_slices), GFP_KERNEL);
		if (!mpeg2_slices) {
			ret = -ENOMEM;
			goto finish_request;
		}
		total_mbs = ctx->mpeg2_picture_width_mbs *
			ctx->mpeg2_picture_height_mbs;
		ret = histb_vdec_parse_mpeg2_slices(ctx->bitstream.cpu, payload,
						    DIV_ROUND_UP(mpeg2_sequence->horizontal_size, 16), total_mbs,
			mpeg2_slices, &num_slices);
		if (ret)
			goto finish_request;
		ret = histb_vdec_prepare_mpeg2_messages(ctx, decoded,
							mpeg2_sequence, mpeg2_picture, mpeg2_quantisation,
			mpeg2_slices, num_slices, payload);
		if (ret)
			goto finish_request;
		ctx->mpeg2_picture_valid = true;
		kfree(mpeg2_slices);
		mpeg2_slices = NULL;
		ctx->total_mbs = total_mbs;
		ctx->frame_flags = ctx->mpeg2_second_field ?
			ctx->mpeg2_pending_frame_flags :
			histb_vdec_mpeg2_frame_flags(mpeg2_picture);
		ctx->hevc = false;
		ctx->hevc_main10 = false;
		ctx->hevc_scaling_list = false;
		ctx->hevc_slices = 0;
		ctx->mpeg2 = true;
		ctx->mpeg2_slices = num_slices;
		ctx->vp8 = false;
		ctx->cabac = false;
		ctx->mbaff = false;
		histb_vdec_complete_request(ctx, request);
		goto start_hardware;
	}
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_VP8_FRAME) {
		const struct v4l2_ctrl_vp8_frame *vp8_frame;

		vp8_frame = histb_vdec_ctrl_data(ctx,
						 V4L2_CID_STATELESS_VP8_FRAME);
		ret = histb_vdec_validate_vp8(ctx, vp8_frame, payload);
		if (ret)
			goto finish_request;
		ret = histb_vdec_set_decoded_buffer_sizes(ctx, decoded);
		if (ret)
			goto finish_request;
		if (!histb_vdec_dma_buffers_valid(ctx)) {
			ret = -EINVAL;
			goto finish_request;
		}
		ret = histb_vdec_copy_vp8_bitstream(ctx, src, payload,
						    vp8_frame, &src_dma);
		if (ret)
			goto finish_request;
		ret = histb_vdec_prepare_vp8_messages(ctx, decoded, vp8_frame,
						      src_dma, payload);
		if (ret)
			goto finish_request;
		total_mbs = DIV_ROUND_UP(vp8_frame->width, 16) *
			DIV_ROUND_UP(vp8_frame->height, 16);
		ctx->total_mbs = total_mbs;
		ctx->frame_flags = V4L2_VP8_FRAME_IS_KEY_FRAME(vp8_frame) ? 0 :
			V4L2_H264_DECODE_PARAM_FLAG_PFRAME;
		ctx->hevc = false;
		ctx->hevc_main10 = false;
		ctx->hevc_scaling_list = false;
		ctx->hevc_slices = 0;
		ctx->mpeg2 = false;
		ctx->mpeg2_slices = 0;
		ctx->vp8 = true;
		ctx->cabac = false;
		ctx->mbaff = false;
		histb_vdec_complete_request(ctx, request);
		goto start_hardware;
	}
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_VP9_FRAME) {
		const struct v4l2_ctrl_vp9_compressed_hdr *compressed;
		const struct v4l2_ctrl_vp9_frame *frame;
		struct histb_vdec_vp9_header header;

		frame = histb_vdec_ctrl_data(ctx,
					     V4L2_CID_STATELESS_VP9_FRAME);
		compressed = histb_vdec_ctrl_data(
			ctx, V4L2_CID_STATELESS_VP9_COMPRESSED_HDR);
		if (!frame || !compressed) {
			ret = -EINVAL;
			goto finish_request;
		}
		ret = histb_vdec_set_decoded_buffer_sizes(ctx, decoded);
		if (ret)
			goto finish_request;
		if (!histb_vdec_dma_buffers_valid(ctx)) {
			ret = -EINVAL;
			goto finish_request;
		}
		ret = histb_vdec_copy_raw_bitstream(ctx, src, payload, &src_dma);
		if (ret)
			goto finish_request;
		ret = histb_vdec_parse_vp9_uncompressed_header(
			ctx->bitstream.cpu, payload, frame, ctx, &header);
		if (ret)
			goto finish_request;
		ret = histb_vdec_stage_vp9_state(ctx, decoded, frame,
						 compressed, &header);
		if (ret)
			goto finish_request;
		vp9_staged = true;
		ret = histb_vdec_prepare_vp9_messages(ctx, decoded,
					       &ctx->vp9_pending_frame,
					       &header, src_dma, payload);
		if (ret)
			goto finish_request;
		total_mbs = DIV_ROUND_UP(header.width, 64) *
			DIV_ROUND_UP(header.height, 64);
		ctx->total_mbs = total_mbs;
		ctx->frame_flags = header.key_frame ? 0 :
			V4L2_H264_DECODE_PARAM_FLAG_PFRAME;
		ctx->hevc = false;
		ctx->hevc_main10 = false;
		ctx->hevc_scaling_list = false;
		ctx->hevc_slices = 0;
		ctx->mpeg2 = false;
		ctx->mpeg2_slices = 0;
		ctx->vp8 = false;
		ctx->vp9 = true;
		ctx->cabac = false;
		ctx->mbaff = false;
		histb_vdec_complete_request(ctx, request);
		goto start_hardware;
	}

	decode = histb_vdec_ctrl_data(ctx,
				      V4L2_CID_STATELESS_H264_DECODE_PARAMS);
	sps = histb_vdec_ctrl_data(ctx, V4L2_CID_STATELESS_H264_SPS);
	pps = histb_vdec_ctrl_data(ctx, V4L2_CID_STATELESS_H264_PPS);
	scaling = histb_vdec_ctrl_data(ctx,
				       V4L2_CID_STATELESS_H264_SCALING_MATRIX);
	slice = histb_vdec_ctrl_data(ctx,
				     V4L2_CID_STATELESS_H264_SLICE_PARAMS);
	pred_weights = histb_vdec_ctrl_data(
		ctx, V4L2_CID_STATELESS_H264_PRED_WEIGHTS);
	ctx->h264_new_frame =
		!ctx->h264_partial && slice && !slice->first_mb_in_slice;
	ctx->h264_last_slice =
		!(src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF);
	ctx->h264_first_mb = slice ? slice->first_mb_in_slice : 0;
	ret = histb_vdec_validate_h264(ctx, decode, sps, pps, slice,
				      pred_weights, payload, ctx->h264_new_frame);
	if (ret)
		goto finish_request;
	if (ctx->h264_slices >= HISTB_VDEC_H264_MAX_SLICES) {
		ret = -E2BIG;
		goto finish_request;
	}
	if (ctx->h264_new_frame) {
		ret = histb_vdec_begin_h264_picture(
			ctx, decoded, src->vb2_buf.timestamp, decode, sps);
		if (ret)
			goto finish_request;
	}

	if (!histb_vdec_dma_buffers_valid(ctx)) {
		ret = -EINVAL;
		goto finish_request;
	}
	{
		u32 tail_bits;

		ret = histb_vdec_append_h264_bitstream(ctx, src, &payload,
						       &src_dma, &tail_bits);
		if (ret)
			goto finish_request;
		/* CABAC retains rbsp_stop_one_bit and removes trailing zero bits. */
		if (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE)
			tail_bits--;
		ret = histb_vdec_prepare_messages(ctx, decoded, decode, sps, pps,
					  scaling, slice, pred_weights,
					  ctx->bitstream.dma, src_dma,
					  payload, tail_bits, ctx->h264_slices,
					  ctx->h264_new_frame,
					  ctx->h264_new_surface);
	}
	if (ret)
		goto finish_request;
	total_mbs = (sps->pic_width_in_mbs_minus1 + 1) *
		    (sps->pic_height_in_map_units_minus1 + 1);
	if (!(decode->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC) &&
	    !(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY))
		total_mbs *= 2;
	ctx->total_mbs = total_mbs;
	ctx->frame_flags = decode->flags;
	ctx->hevc = false;
	ctx->hevc_main10 = false;
	ctx->hevc_scaling_list = false;
	ctx->hevc_slices = 0;
	ctx->mpeg2 = false;
	ctx->mpeg2_slices = 0;
	ctx->vp8 = false;
	ctx->cabac = pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE;
	ctx->mbaff =
		!(decode->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC) &&
		(sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD);
	ctx->h264_slices++;

	histb_vdec_complete_request(ctx, request);
	if (!ctx->h264_last_slice) {
		ctx->h264_partial = true;
		ctx->h264_last_first_mb = ctx->h264_first_mb;
		histb_vdec_finish_h264_slice(ctx);
		return;
	}
	ctx->h264_new_frame = true;
	src_dma = ctx->bitstream.dma;

start_hardware:
	ret = pm_runtime_resume_and_get(vdec->dev);
	if (ret < 0) {
		if (ctx->vp9)
			histb_vdec_discard_vp9_state(ctx);
		if (ctx->mpeg4)
			histb_vdec_fail_mpeg4_state(ctx);
		histb_vdec_finish_job_no_pm(ctx, VB2_BUF_STATE_ERROR);
		return;
	}
	ret = histb_vdec_configure_vdh_clock(vdec);
	if (ret) {
		dev_err(vdec->dev, "failed to configure VDH clock: %d\n", ret);
		if (ctx->vp9)
			histb_vdec_discard_vp9_state(ctx);
		if (ctx->mpeg4)
			histb_vdec_fail_mpeg4_state(ctx);
		histb_vdec_finish_job(ctx, VB2_BUF_STATE_ERROR);
		return;
	}
	if (ctx->avs && vdec->smmu_identity_needs_reset) {
		/*
		 * Initialize a new VDH page-table context before GlbResetX. The
		 * normal AVS launch installs the same table again after the reset.
		 */
		mutex_lock(&vdec->launch_lock);
		histb_vdec_program_smmu(vdec, true);
		ret = histb_vdec_reset_vdh_domain(vdec);
		if (!ret)
			vdec->smmu_identity_needs_reset = false;
		mutex_unlock(&vdec->launch_lock);
		if (ret) {
			dev_err(vdec->dev,
				"failed to initialize AVS SMMU domain: %d\n", ret);
			histb_vdec_finish_job(ctx, VB2_BUF_STATE_ERROR);
			return;
		}
	}
	if (ctx->avs) {
		ret = histb_vdec_load_avsp(vdec);
		if (ret) {
			dev_err(vdec->dev, "failed to load AVSP firmware: %d\n", ret);
			histb_vdec_finish_job(ctx, VB2_BUF_STATE_ERROR);
			return;
		}
	}

	/*
	 * Reconstruction surfaces use cached non-coherent CPU
	 * mappings so the later tile conversion is not limited by uncached reads.
	 * Hand the current surface and every stable APC reference back to the VDH
	 * before programming addresses and ringing the doorbell.
	 */
	histb_vdec_sync_tile_for_device(ctx, decoded);
	{
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(ctx->apc); i++)
			if (ctx->apc[i] != decoded)
				histb_vdec_sync_tile_for_device(ctx, ctx->apc[i]);
		for (i = 0; ctx->vp9 && i < ARRAY_SIZE(ctx->vp9_dpb); i++)
			if (ctx->vp9_dpb[i] != decoded)
				histb_vdec_sync_tile_for_device(ctx, ctx->vp9_dpb[i]);
		for (i = 0; ctx->mpeg4 && i < ARRAY_SIZE(ctx->mpeg4_ref); i++)
			histb_vdec_sync_mpeg4_anchor_for_device(
				ctx, &ctx->mpeg4_ref[i]);
		for (i = 0; ctx->vc1 && i < ARRAY_SIZE(ctx->vc1_ref); i++)
			histb_vdec_sync_vc1_anchor_for_device(
				ctx, &ctx->vc1_ref[i]);
	}

	spin_lock_irqsave(&vdec->irqlock, flags);
	if (WARN_ON(vdec->curr_ctx)) {
		spin_unlock_irqrestore(&vdec->irqlock, flags);
		if (ctx->vp9)
			histb_vdec_discard_vp9_state(ctx);
		if (ctx->mpeg4)
			histb_vdec_fail_mpeg4_state(ctx);
		histb_vdec_finish_job(ctx, VB2_BUF_STATE_ERROR);
		return;
	}
	vdec->curr_ctx = ctx;
	vdec->phase = HISTB_VDEC_PHASE_VDH;
	vdec->job_cancelled = false;
	vdec->job_count_pending = true;
	spin_unlock_irqrestore(&vdec->irqlock, flags);

	mutex_lock(&vdec->launch_lock);
	if (!histb_vdec_job_in_phase(vdec, ctx, HISTB_VDEC_PHASE_VDH)) {
		mutex_unlock(&vdec->launch_lock);
		return;
	}
	schedule_delayed_work(&vdec->watchdog_work,
			      msecs_to_jiffies(HISTB_VDEC_WATCHDOG_MS));
	if (ctx->avs) {
		histb_vdec_program_avs_registers(ctx, decoded, src_dma,
						 total_mbs);
	} else if (ctx->vp8) {
		if (!ctx->vp8_pending_frame_type)
			memset(ctx->buffers[HISTB_VDEC_BUF_VP8_SEG].cpu, 0,
			       ctx->buffers[HISTB_VDEC_BUF_VP8_SEG].size);
		histb_vdec_program_vp8_registers(ctx, decoded, src_dma,
						 total_mbs);
	} else if (ctx->vp9) {
		histb_vdec_program_vp9_registers(ctx, decoded, src_dma,
						 total_mbs);
	} else if (ctx->mpeg2) {
		ctx->mpeg2_reference_started = true;
		histb_vdec_program_mpeg2_registers(ctx, decoded, src_dma,
							   total_mbs);
	} else if (ctx->mpeg4)
		histb_vdec_program_mpeg4_registers(ctx);
	else if (ctx->hevc)
		histb_vdec_program_hevc_registers(ctx, decoded, src_dma,
							  total_mbs);
	else
		histb_vdec_program_registers(ctx, decoded, src_dma, total_mbs,
						     ctx->mbaff,
						     ctx->h264_field_picture);
	mutex_unlock(&vdec->launch_lock);
	return;

finish_request:
	dev_err(vdec->dev, "%4.4s request setup failed: %d\n",
			    (char *)&ctx->src.pix.pixelformat, ret);
	kfree(avs_streams);
	kfree(hevc_streams);
	kfree(mpeg2_slices);
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_AVS_SLICE) {
		decoded->avs.valid = false;
		ctx->avs = false;
		ctx->avs_slices = 0;
		ctx->avs_total_mbs = 0;
	}
	if (vp9_staged)
		histb_vdec_discard_vp9_state(ctx);
	if (!mpeg4_state_failed &&
	    (mpeg4_staged || ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG4))
		histb_vdec_fail_mpeg4_state(ctx);
	histb_vdec_complete_request(ctx, request);
	histb_vdec_finish_job_no_pm(ctx, VB2_BUF_STATE_ERROR);
}

static void histb_vdec_job_abort(void *priv)
{
	histb_vdec_abort(priv);
}

static const struct v4l2_m2m_ops histb_vdec_m2m_ops = {
	.device_run = histb_vdec_device_run,
	.job_abort = histb_vdec_job_abort,
};

static void histb_vdec_return_buffers(struct histb_vdec_ctx *ctx,
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
		if (V4L2_TYPE_IS_OUTPUT(type) && buf->vb2_buf.req_obj.req)
			v4l2_ctrl_request_complete(buf->vb2_buf.req_obj.req,
						   &ctx->ctrl_handler);
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	}
}

static void histb_vdec_free_buffers(struct histb_vdec_ctx *ctx)
{
	unsigned int i;

	histb_vdec_reset_mpeg4_state(ctx);
	histb_vdec_reset_vc1_state(ctx);
	if (ctx->bitstream.cpu) {
		dma_free_coherent(ctx->vdec->dev, ctx->bitstream.size,
				  ctx->bitstream.cpu, ctx->bitstream.dma);
		memset(&ctx->bitstream, 0, sizeof(ctx->bitstream));
	}
	if (ctx->vc1_stream.cpu) {
		dma_free_coherent(ctx->vdec->dev, ctx->vc1_stream.size,
				  ctx->vc1_stream.cpu, ctx->vc1_stream.dma);
		memset(&ctx->vc1_stream, 0, sizeof(ctx->vc1_stream));
	}

	for (i = 0; i < HISTB_VDEC_BUF_COUNT; i++) {
		struct histb_vdec_dma_buffer *buffer = &ctx->buffers[i];

		if (buffer->cpu && !ctx->buf_arena.cpu)
			dma_free_coherent(ctx->vdec->dev, buffer->size,
					  buffer->cpu, buffer->dma);
		memset(buffer, 0, sizeof(*buffer));
	}
	if (ctx->buf_arena.cpu) {
		dma_free_coherent(ctx->vdec->dev, ctx->buf_arena.size,
				  ctx->buf_arena.cpu, ctx->buf_arena.dma);
		memset(&ctx->buf_arena, 0, sizeof(ctx->buf_arena));
	}
	ctx->h264_cabac_loaded = false;
	ctx->hevc_cabac_loaded = false;
	ctx->avs = false;
	ctx->avs_slices = 0;
	ctx->avs_total_mbs = 0;
	ctx->avs_basic_cfg1 = 0;
	ctx->avs_profile = 0;
	histb_vdec_reset_vp8_state(ctx);
	histb_vdec_reset_vp9_state(ctx);
}

static size_t histb_vdec_buffer_size(struct histb_vdec_ctx *ctx,
				     enum histb_vdec_buffer_id id)
{
	if ((id == HISTB_VDEC_BUF_VC1_BPD ||
	     id == HISTB_VDEC_BUF_VC1_INTENSITY) &&
	    !histb_vdec_is_vc1_format(ctx->src.pix.pixelformat))
		return 0;
	if (id == HISTB_VDEC_BUF_AVS_DNR_MBINFO &&
	    ctx->src.pix.pixelformat != V4L2_PIX_FMT_AVS_SLICE)
		return 0;
	if ((id == HISTB_VDEC_BUF_MPEG4_DNR_MBINFO ||
	     id == HISTB_VDEC_BUF_MPEG4_ITRANS ||
	     id == HISTB_VDEC_BUF_MPEG4_PMV_TOP ||
	     id == HISTB_VDEC_BUF_MPEG4_SED_TOP) &&
	    ctx->src.pix.pixelformat != V4L2_PIX_FMT_MPEG4)
		return 0;
	if ((ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG1_SLICE ||
	     ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG2_SLICE) &&
	    id == HISTB_VDEC_BUF_MSG)
		return HISTB_VDEC_MPEG2_MSG_SIZE;
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_AVS_SLICE &&
	    id == HISTB_VDEC_BUF_MSG)
		return HISTB_VDEC_AVS_MSG_SIZE;
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG4 &&
	    id == HISTB_VDEC_BUF_MSG)
		return HISTB_VDEC_MPEG4_MSG_SIZE;
	if (histb_vdec_is_vc1_format(ctx->src.pix.pixelformat) &&
	    id == HISTB_VDEC_BUF_MSG)
		return HISTB_VDEC_VC1_MSG_SIZE;
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE &&
	    id == HISTB_VDEC_BUF_MSG)
		return HISTB_VDEC_H264_MSG_SIZE;
	if (ctx->src.pix.pixelformat != V4L2_PIX_FMT_HEVC_SLICE)
		return histb_vdec_buffer_sizes[id];

	switch (id) {
	case HISTB_VDEC_BUF_MSG:
		return HISTB_VDEC_HEVC_MSG_SIZE;
	case HISTB_VDEC_BUF_SED_TOP:
	case HISTB_VDEC_BUF_PMV_TOP:
	case HISTB_VDEC_BUF_RCN_TOP:
	case HISTB_VDEC_BUF_SAO_TOP:
	case HISTB_VDEC_BUF_DBLK_TOP:
		return max_t(size_t, histb_vdec_buffer_sizes[id],
			     256 * HISTB_VDEC_HEVC_MAX_WIDTH);
	case HISTB_VDEC_BUF_PMV_LEFT:
	case HISTB_VDEC_BUF_SAO_LEFT:
	case HISTB_VDEC_BUF_DBLK_LEFT:
		return max_t(size_t, histb_vdec_buffer_sizes[id],
			     256 * HISTB_VDEC_HEVC_MAX_HEIGHT);
	default:
		return histb_vdec_buffer_sizes[id];
	}
}

static int histb_vdec_load_cabac(struct histb_vdec_ctx *ctx,
				 enum histb_vdec_buffer_id id,
				 const char *name, size_t expected_size,
				 bool *loaded)
{
	struct device *dev = ctx->vdec->dev;
	const struct firmware *firmware;
	int ret;

	if (*loaded)
		return 0;
	ret = request_firmware_direct(&firmware, name, dev);
	if (ret) {
		dev_err(dev, "failed to load %s: %d\n", name, ret);
		return ret;
	}
	if (firmware->size != expected_size) {
		dev_err(dev, "%s has invalid size %zu (expected %zu)\n",
			name, firmware->size, expected_size);
		ret = -EINVAL;
		goto release_firmware;
	}
	memset(ctx->buffers[id].cpu, 0, ctx->buffers[id].size);
	memcpy(ctx->buffers[id].cpu, firmware->data, firmware->size);
	*loaded = true;

release_firmware:
	release_firmware(firmware);
	return ret;
}

static void histb_vdec_release_avsp(void *data)
{
	struct histb_vdec_dev *vdec = data;
	struct histb_vdec_dma_buffer *buffer = &vdec->avsp_firmware;

	if (buffer->cpu)
		dma_free_coherent(vdec->dev, buffer->size, buffer->cpu,
				  buffer->dma);
	memset(buffer, 0, sizeof(*buffer));
	vdec->avsp_available = false;
	vdec->avsp_loaded = false;
}

static int histb_vdec_prepare_avsp(struct histb_vdec_dev *vdec)
{
	struct histb_vdec_dma_buffer *buffer = &vdec->avsp_firmware;
	const struct firmware *firmware;
	int ret;

	if (vdec->avsp_available)
		return 0;

	ret = request_firmware_direct(&firmware, HISTB_VDEC_AVSP_FIRMWARE,
				      vdec->dev);
	if (ret) {
		dev_err(vdec->dev, "failed to load %s: %d\n",
			HISTB_VDEC_AVSP_FIRMWARE, ret);
		return ret;
	}
	if (firmware->size != HISTB_VDEC_AVSP_FIRMWARE_SIZE) {
		dev_err(vdec->dev, "%s has invalid size %zu (expected %u)\n",
			HISTB_VDEC_AVSP_FIRMWARE, firmware->size,
			HISTB_VDEC_AVSP_FIRMWARE_SIZE);
		ret = -EINVAL;
		goto release_firmware;
	}

	buffer->size = HISTB_VDEC_AVSP_FIRMWARE_SIZE;
	buffer->cpu = dma_alloc_coherent(vdec->dev, buffer->size,
					 &buffer->dma, GFP_KERNEL);
	if (!buffer->cpu) {
		ret = -ENOMEM;
		goto release_firmware;
	}
	if (!histb_vdec_smmu_dma_buffer_valid(buffer) ||
	    !IS_ALIGNED(buffer->dma, HISTB_VDEC_AVSP_FIRMWARE_ALIGN)) {
		dev_err(vdec->dev, "AVSP DMA address %pad is not usable\n",
			&buffer->dma);
		dma_free_coherent(vdec->dev, buffer->size, buffer->cpu,
				  buffer->dma);
		memset(buffer, 0, sizeof(*buffer));
		ret = -ERANGE;
		goto release_firmware;
	}

	memcpy(buffer->cpu, firmware->data, firmware->size);
	vdec->avsp_available = true;
	vdec->avsp_loaded = false;
	ret = 0;

release_firmware:
	release_firmware(firmware);
	return ret;
}

static int histb_vdec_load_avsp(struct histb_vdec_dev *vdec)
{
	u32 state;
	int ret;

	if (vdec->avsp_loaded)
		return 0;
	if (!vdec->avsp_available)
		return -ENOENT;

	writel(HISTB_VDEC_SCD_EMAR_BASE | HISTB_VDEC_SCD_EMAR_ENABLE,
	       vdec->regs + HISTB_VDEC_SCD_EMAR_CFG);
	writel(1, vdec->regs + HISTB_VDEC_SCD_AVS_FLAG);
	writel(HISTB_VDEC_AVSP_FIRMWARE_SIZE / sizeof(u32),
	       vdec->regs + HISTB_VDEC_SCD_DSP_CODE_WORDS);
	writel(lower_32_bits(vdec->avsp_firmware.dma),
	       vdec->regs + HISTB_VDEC_SCD_DSP_CODE_ADDR);
	/* Make the copied firmware and its descriptor visible before DSP start. */
	wmb();
	writel(0, vdec->regs + HISTB_VDEC_SCD_DSP_CTRL);
	writel(5, vdec->regs + HISTB_VDEC_SCD_DSP_CTRL);
	usleep_range(30, 60);
	ret = readl_poll_timeout(vdec->regs + HISTB_VDEC_SCD_DSP_STATE,
				 state, state & BIT(0), 30,
				 HISTB_CRG_RESET_TIMEOUT_US);
	if (ret) {
		dev_err(vdec->dev,
			"AVSP firmware did not become ready: %d state=%08x\n",
			ret, state);
		return ret;
	}

	vdec->avsp_loaded = true;
	return 0;
}

static int histb_vdec_alloc_buffers(struct histb_vdec_ctx *ctx)
{
	struct device *dev = ctx->vdec->dev;
	unsigned int i;
	int ret = -ENOMEM;

	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_AVS_SLICE) {
		ret = histb_vdec_prepare_smmu_identity(ctx->vdec);
		if (ret)
			return ret;
	}

	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_AVS_SLICE) {
		dma_addr_t base_dma, next;
		void *base_cpu;
		size_t total = 0;

		/*
		 * AVS is the one format whose engine-side buffers must all sit
		 * inside the SMMU identity window, which begins at 1 MiB and ends
		 * at HISTB_VDEC_SMMU_DMA_SIZE.  Allocated one at a time the small
		 * ones - a few KiB - are served by the buddy allocator and land
		 * below 1 MiB, which the vendor domain reserves, so
		 * histb_vdec_smmu_dma_buffer_valid() rejects them and every
		 * request fails with -ERANGE before the engine is programmed.
		 * One arena carries all of them instead: at this size the
		 * allocation is served by CMA, whose region lies inside the
		 * window, and the carve keeps each entry there.  The floor keeps
		 * the total above the size at which CMA is chosen.
		 */
		for (i = 0; i < HISTB_VDEC_BUF_COUNT; i++)
			total += histb_vdec_buffer_size(ctx, i) + SZ_4K;
		total = max_t(size_t, total, SZ_4M);

		ctx->buf_arena.size = total;
		ctx->buf_arena.cpu = dma_alloc_coherent(dev, total,
						       &ctx->buf_arena.dma,
						       GFP_KERNEL);
		if (!ctx->buf_arena.cpu)
			goto free_buffers;
		memset(ctx->buf_arena.cpu, 0, total);

		if (ctx->buf_arena.dma < SZ_1M ||
		    ctx->buf_arena.dma + total > HISTB_VDEC_SMMU_DMA_SIZE) {
			dev_err(dev,
				"AVS buffer arena 0x%llx+%zu is outside the SMMU identity window\n",
				(unsigned long long)ctx->buf_arena.dma, total);
			goto free_buffers;
		}

		base_dma = ctx->buf_arena.dma;
		base_cpu = ctx->buf_arena.cpu;
		next = base_dma;
		for (i = 0; i < HISTB_VDEC_BUF_COUNT; i++) {
			size_t size = histb_vdec_buffer_size(ctx, i);

			if (!size)
				continue;
			next = ALIGN(next, SZ_4K);
			ctx->buffers[i].size = size;
			ctx->buffers[i].dma = next;
			ctx->buffers[i].cpu =
				(u8 *)base_cpu + (next - base_dma);
			next += size;
		}
	} else if (!ctx->buffers[HISTB_VDEC_BUF_MSG].cpu) {
		for (i = 0; i < HISTB_VDEC_BUF_COUNT; i++) {
			struct histb_vdec_dma_buffer *buffer = &ctx->buffers[i];
			size_t size = histb_vdec_buffer_size(ctx, i);

			if (!size)
				continue;
			buffer->size = size;
			buffer->cpu = dma_alloc_coherent(dev, size, &buffer->dma,
							 GFP_KERNEL);
			if (!buffer->cpu)
				goto free_buffers;
			memset(buffer->cpu, 0, size);
		}
	}

	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_AVS_SLICE)
		ret = histb_vdec_prepare_avsp(ctx->vdec);
	else if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_HEVC_SLICE)
		ret = histb_vdec_load_cabac(ctx, HISTB_VDEC_BUF_HEVC_CABAC,
					    HISTB_VDEC_HEVC_CABAC_FIRMWARE,
					    HISTB_VDEC_HEVC_CABAC_SIZE,
					    &ctx->hevc_cabac_loaded);
	else if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE)
		ret = histb_vdec_load_cabac(ctx, HISTB_VDEC_BUF_H264_CABAC,
					    HISTB_VDEC_H264_CABAC_FIRMWARE,
					    HISTB_VDEC_H264_CABAC_SIZE,
					    &ctx->h264_cabac_loaded);
	else
		ret = 0;
	if (!ret)
		return 0;

free_buffers:
	histb_vdec_free_buffers(ctx);
	return ret ?: -ENOMEM;
}

static int histb_vdec_queue_setup(struct vb2_queue *vq,
				  unsigned int *nbuffers,
				  unsigned int *nplanes,
				  unsigned int sizes[],
				  struct device *alloc_devs[])
{
	struct histb_vdec_ctx *ctx = vb2_get_drv_priv(vq);
	struct histb_vdec_q_data *q_data =
		histb_vdec_get_q_data(ctx, vq->type);

	/*
	 * A 3840x2160 capture buffer is 12.4 MB and the whole CMA area is
	 * 192 MiB, so the number of them decides whether the queue can be
	 * filled at all.  Measured: H.264 4K asks for 11 and decodes, VP9 4K
	 * asks for 17 - 211 MB - and every frame fails with
	 * "cma: __cma_alloc: alloc failed".  It is a capacity limit, not
	 * fragmentation: the earlier reading of "3531 pages free, 3038
	 * requested" was a snapshot taken after the first few buffers had
	 * already been taken.  Cap the count for the large geometries so the
	 * queue fits, and say so, because silently reducing a user's request
	 * is exactly the kind of thing that hides a problem.
	 */
	if (!V4L2_TYPE_IS_OUTPUT(vq->type)) {
		/*
		 * The count arrives one buffer at a time through CREATE_BUFS,
		 * so capping *nbuffers does not bound it - the queue's own
		 * limit does.  Seen from the board: H.264 4K ends up with 11
		 * capture buffers and decodes, VP9 4K ends up with 17 and every
		 * frame fails with "cma: __cma_alloc: alloc failed", because 17
		 * x 12.4 MB is 211 MB against a 192 MiB CMA area.  Capacity,
		 * not fragmentation.  Bound it for the large geometries only.
		 */
		if (q_data->pix.sizeimage > SZ_8M &&
		    vq->max_num_buffers > HISTB_VDEC_MAX_LARGE_CAPTURE_BUFFERS)
			vq->max_num_buffers = HISTB_VDEC_MAX_LARGE_CAPTURE_BUFFERS;

		if (q_data->pix.sizeimage > SZ_8M &&
		    *nbuffers > HISTB_VDEC_MAX_LARGE_CAPTURE_BUFFERS) {
			dev_info_ratelimited(ctx->vdec->dev,
					     "capture queue asks for %u buffers of %u bytes; limiting to %u to fit the 192 MiB CMA area\n",
					     *nbuffers, q_data->pix.sizeimage,
					     HISTB_VDEC_MAX_LARGE_CAPTURE_BUFFERS);
			*nbuffers = HISTB_VDEC_MAX_LARGE_CAPTURE_BUFFERS;
		}
	}

	if (*nplanes)
		return *nplanes != 1 || sizes[0] < q_data->pix.sizeimage ?
		       -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = q_data->pix.sizeimage;
	return 0;
}

static int histb_vdec_buf_prepare(struct vb2_buffer *vb)
{
	struct histb_vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct histb_vdec_q_data *q_data =
		histb_vdec_get_q_data(ctx, vb->vb2_queue->type);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	unsigned long data_offset = vb->planes[0].data_offset;
	unsigned long payload = vb2_get_plane_payload(vb, 0);

	if (data_offset > vb2_plane_size(vb, 0) ||
	    vb2_plane_size(vb, 0) - data_offset < q_data->pix.sizeimage)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) &&
	    (payload <= data_offset || payload > vb2_plane_size(vb, 0)))
		return -EINVAL;

	vbuf->field = V4L2_FIELD_NONE;
	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type))
		vb2_set_plane_payload(vb, 0, 0);

	return 0;
}

static int histb_vdec_buf_init(struct vb2_buffer *vb)
{
	struct histb_vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct histb_vdec_decoded_buffer *decoded;

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		return 0;

	decoded = container_of(vb, struct histb_vdec_decoded_buffer,
			       base.vb.vb2_buf);
	decoded->apc_slot = HISTB_VDEC_APC_INVALID;
	decoded->error_tainted = false;
	decoded->mpeg2_field_saved = false;
	memset(&decoded->avs, 0, sizeof(decoded->avs));
	decoded->vp9_dpb_valid = false;
	decoded->vp9_logic_id = HISTB_VDEC_APC_INVALID;
	decoded->vp9_width = 0;
	decoded->vp9_height = 0;
	decoded->tile.size = histb_vdec_tile_size(ctx);
	decoded->pmv.size = histb_vdec_pmv_size(ctx);
	return 0;
}

static void histb_vdec_buf_cleanup(struct vb2_buffer *vb)
{
	struct histb_vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct histb_vdec_decoded_buffer *decoded;

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		return;

	decoded = container_of(vb, struct histb_vdec_decoded_buffer,
			       base.vb.vb2_buf);
	if (decoded->apc_slot >= 0 &&
	    decoded->apc_slot < ARRAY_SIZE(ctx->apc) &&
	    ctx->apc[decoded->apc_slot] == decoded)
		ctx->apc[decoded->apc_slot] = NULL;
	if (ctx->mpeg2_anchor[0] == decoded)
		ctx->mpeg2_anchor[0] = NULL;
	if (ctx->mpeg2_anchor[1] == decoded)
		ctx->mpeg2_anchor[1] = NULL;
	if (ctx->mpeg2_pending_field == decoded)
		histb_vdec_reset_mpeg2_field_pair(ctx);
	decoded->apc_slot = HISTB_VDEC_APC_INVALID;
	decoded->error_tainted = false;
	decoded->mpeg2_field_saved = false;
	memset(&decoded->avs, 0, sizeof(decoded->avs));
	if (ctx->h264_pending_field == decoded)
		histb_vdec_reset_h264_field_pair(ctx);
	{
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(ctx->vp9_dpb); i++)
			if (ctx->vp9_dpb[i] == decoded)
				ctx->vp9_dpb[i] = NULL;
	}
	decoded->vp9_dpb_valid = false;
	decoded->vp9_logic_id = HISTB_VDEC_APC_INVALID;
	histb_vdec_free_decoded_buffers(ctx, decoded);
	memset(&decoded->tile, 0, sizeof(decoded->tile));
	memset(&decoded->pmv, 0, sizeof(decoded->pmv));
}

static int histb_vdec_buf_out_validate(struct vb2_buffer *vb)
{
	to_vb2_v4l2_buffer(vb)->field = V4L2_FIELD_NONE;
	return 0;
}

static void histb_vdec_buf_queue(struct vb2_buffer *vb)
{
	struct histb_vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct histb_vdec_decoded_buffer *decoded;
	unsigned int i;

	if (!V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		decoded = container_of(vb, struct histb_vdec_decoded_buffer,
				       base.vb.vb2_buf);
		decoded->avs.valid = false;
		/* Stateless reference formats clear this only for a new target. */
		if (ctx->src.pix.pixelformat != V4L2_PIX_FMT_H264_SLICE &&
		    ctx->src.pix.pixelformat != V4L2_PIX_FMT_MPEG1_SLICE &&
		    ctx->src.pix.pixelformat != V4L2_PIX_FMT_MPEG2_SLICE &&
		    ctx->src.pix.pixelformat != V4L2_PIX_FMT_VP8_FRAME &&
		    ctx->src.pix.pixelformat != V4L2_PIX_FMT_VP9_FRAME)
			decoded->error_tainted = false;
	}

	if (!V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) &&
	    vb2_is_streaming(vb->vb2_queue) &&
	    v4l2_m2m_dst_buf_is_last(ctx->fh.m2m_ctx)) {
		for (i = 0; i < vb->num_planes; i++)
			vb2_set_plane_payload(vb, i, 0);
		vbuf->field = V4L2_FIELD_NONE;
		vbuf->sequence = ctx->dst.sequence++;
		v4l2_m2m_last_buffer_done(ctx->fh.m2m_ctx, vbuf);
		v4l2_event_queue_fh(&ctx->fh, &histb_vdec_eos_event);
		return;
	}

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static void histb_vdec_buf_request_complete(struct vb2_buffer *vb)
{
	struct histb_vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (vb->req_obj.req)
		v4l2_ctrl_request_complete(vb->req_obj.req, &ctx->ctrl_handler);
}

static int histb_vdec_start_streaming(struct vb2_queue *vq,
				      unsigned int count)
{
	struct histb_vdec_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	int ret = 0;

	v4l2_m2m_update_start_streaming_state(m2m_ctx, vq);
	histb_vdec_get_q_data(ctx, vq->type)->sequence = 0;
	if (V4L2_TYPE_IS_OUTPUT(vq->type) &&
	    ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE) {
		histb_vdec_reset_h264_slices(ctx);
		histb_vdec_reset_h264_field_pair(ctx);
		histb_vdec_reset_apc(ctx);
	}
	if (V4L2_TYPE_IS_OUTPUT(vq->type) &&
	    (ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG1_SLICE ||
	     ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG2_SLICE))
		histb_vdec_reset_apc(ctx);
	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		ret = histb_vdec_alloc_buffers(ctx);
	if (!ret && V4L2_TYPE_IS_OUTPUT(vq->type))
		histb_vdec_reset_vp9_state(ctx);
	if (!ret && V4L2_TYPE_IS_OUTPUT(vq->type)) {
		mutex_lock(&ctx->mpeg4_lock);
		histb_vdec_reset_mpeg4_state(ctx);
		mutex_unlock(&ctx->mpeg4_lock);
	}
	if (!ret && V4L2_TYPE_IS_OUTPUT(vq->type))
		histb_vdec_reset_vc1_state(ctx);
	if (!ret && V4L2_TYPE_IS_OUTPUT(vq->type) &&
	    ctx->src.pix.pixelformat == V4L2_PIX_FMT_AVS_SLICE) {
		ctx->avs = false;
		ctx->avs_slices = 0;
		ctx->avs_total_mbs = 0;
		ctx->avs_basic_cfg1 = 0;
		ctx->avs_profile = 0;
	}
	if (!ret && V4L2_TYPE_IS_OUTPUT(vq->type)) {
		v4l2_m2m_clear_state(m2m_ctx);
		vb2_clear_last_buffer_dequeued(v4l2_m2m_get_dst_vq(m2m_ctx));
	}
	if (ret)
		histb_vdec_return_buffers(ctx, vq->type);

	return ret;
}

static void histb_vdec_stop_streaming(struct vb2_queue *vq)
{
	struct histb_vdec_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	bool preserve_mpeg4;
	bool stopped;

	preserve_mpeg4 = ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG4 &&
		V4L2_TYPE_IS_CAPTURE(vq->type) &&
		v4l2_m2m_has_stopped(m2m_ctx);
	histb_vdec_abort(ctx);
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE) {
		histb_vdec_reset_h264_slices(ctx);
		histb_vdec_reset_h264_field_pair(ctx);
		histb_vdec_reset_apc(ctx);
	}
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG1_SLICE ||
	    ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG2_SLICE)
		histb_vdec_reset_apc(ctx);
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG4) {
		if (preserve_mpeg4) {
			mutex_lock(&ctx->mpeg4_lock);
			histb_vdec_return_mpeg4_display(ctx,
							VB2_BUF_STATE_ERROR, false);
			mutex_unlock(&ctx->mpeg4_lock);
		} else {
			histb_vdec_fail_mpeg4_state(ctx);
		}
	}
	if (histb_vdec_is_vc1_format(ctx->src.pix.pixelformat))
		histb_vdec_reset_vc1_state(ctx);
	histb_vdec_return_buffers(ctx, vq->type);
	if (V4L2_TYPE_IS_CAPTURE(vq->type))
		histb_vdec_reset_apc(ctx);
	histb_vdec_reset_vp8_state(ctx);
	if (V4L2_TYPE_IS_CAPTURE(vq->type))
		histb_vdec_reset_vp9_state(ctx);
	stopped = v4l2_m2m_has_stopped(m2m_ctx);
	v4l2_m2m_update_stop_streaming_state(m2m_ctx, vq);
	if (V4L2_TYPE_IS_OUTPUT(vq->type) && !stopped &&
	    v4l2_m2m_has_stopped(m2m_ctx))
		v4l2_event_queue_fh(&ctx->fh, &histb_vdec_eos_event);
}

static const struct vb2_ops histb_vdec_qops = {
	.queue_setup = histb_vdec_queue_setup,
	.buf_init = histb_vdec_buf_init,
	.buf_prepare = histb_vdec_buf_prepare,
	.buf_cleanup = histb_vdec_buf_cleanup,
	.buf_out_validate = histb_vdec_buf_out_validate,
	.buf_queue = histb_vdec_buf_queue,
	.buf_request_complete = histb_vdec_buf_request_complete,
	.start_streaming = histb_vdec_start_streaming,
	.stop_streaming = histb_vdec_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int histb_vdec_queue_init(void *priv, struct vb2_queue *src_vq,
				 struct vb2_queue *dst_vq)
{
	struct histb_vdec_ctx *ctx = priv;
	struct histb_vdec_dev *vdec = ctx->vdec;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &histb_vdec_qops;
	src_vq->mem_ops = &vb2_vmalloc_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &vdec->lock;
	src_vq->dev = vdec->dev;
	src_vq->dma_dir = DMA_TO_DEVICE;
	src_vq->supports_requests = true;
	src_vq->requires_requests = true;
	src_vq->subsystem_flags =
		VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct histb_vdec_decoded_buffer);
	dst_vq->ops = &histb_vdec_qops;
	dst_vq->mem_ops = &histb_vdec_capture_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &vdec->lock;
	dst_vq->dev = vdec->dev;
	dst_vq->dma_dir = DMA_FROM_DEVICE;
	dst_vq->bidirectional = true;

	return vb2_queue_init(dst_vq);
}

static int histb_vdec_querycap(struct file *file, void *priv,
			       struct v4l2_capability *cap)
{
	strscpy(cap->driver, "histb-vdec", sizeof(cap->driver));
	strscpy(cap->card, "HiSilicon Hi3798CV200 VDH decoder",
		sizeof(cap->card));
	strscpy(cap->bus_info, "platform:histb-vdec", sizeof(cap->bus_info));
	return 0;
}

static int histb_vdec_enum_fmt(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	if (!V4L2_TYPE_IS_OUTPUT(f->type)) {
		switch (f->index) {
		case 0:
			f->pixelformat = V4L2_PIX_FMT_NV12;
			break;
		case 1:
			f->pixelformat = V4L2_PIX_FMT_P010;
			break;
		default:
			return -EINVAL;
		}
		return 0;
	}

	switch (f->index) {
	case 0:
		f->pixelformat = V4L2_PIX_FMT_H264_SLICE;
		break;
	case 1:
		f->pixelformat = V4L2_PIX_FMT_HEVC_SLICE;
		break;
	case 2:
		f->pixelformat = V4L2_PIX_FMT_MPEG2_SLICE;
		break;
	case 3:
		f->pixelformat = V4L2_PIX_FMT_MPEG4;
		break;
	case 4:
		f->pixelformat = V4L2_PIX_FMT_VP8_FRAME;
		break;
	case 5:
		f->pixelformat = V4L2_PIX_FMT_VP9_FRAME;
		break;
	case 6:
		f->pixelformat = V4L2_PIX_FMT_VC1_ANNEX_G;
		break;
	case 7:
		f->pixelformat = V4L2_PIX_FMT_VC1_ANNEX_L;
		break;
	case 8:
		f->pixelformat = V4L2_PIX_FMT_AVS_SLICE;
		break;
	/*
	 * Three formats are deliberately not enumerated.  Their absence here is
	 * the whole interface: a client that cannot see a format will not
	 * select it, which is the only reliable way to keep it out of a
	 * deployment.
	 *
	 *   MPEG-1 (V4L2_PIX_FMT_MPEG1_SLICE)
	 *       The engine accepts a picture message that is correct field by
	 *       field - the geometry, the Mpeg1Flag at word 0 bit 25, the
	 *       QSCALE_TYPE and INTRA_VLC flags, the zigzag table - and
	 *       returns DC-only blocks.  See MPEG1-MESSAGE-IS-CORRECT.md.
	 *
	 *   RealVideo 8 and 9 (V4L2_PIX_FMT_RV30, V4L2_PIX_FMT_RV40)
	 *       The hardware has the HALs (vdm_hal_real8.S, vdm_hal_real9.S)
	 *       and the driver implements the path, but no clip can be built
	 *       to verify it: this ffmpeg has no RV30/RV40 encoder, and the
	 *       slice messages the front end builds carry bit_len = 0.  An
	 *       unverifiable decoder in a broadcast deployment is a liability,
	 *       so it is not offered.
	 *
	 * DivX3 is not listed for the same reason as RealVideo and one more:
	 * this CV200 checkout ships no vdm_hal_divx3.S at all.  The HAL build
	 * directory holds real8, real9, vp6, vp8, vp9, vc1, avs, h264, hevc,
	 * mpeg2 and mpeg4 only, and vfmw_make.cfg guards the DivX3 objects
	 * behind a findstring that never matches, so the build falls back to
	 * a no-op stub.  There is no hardware path to remove.
	 */
	default:
		return -EINVAL;
	}

	return 0;
}

static int histb_vdec_enum_framesizes(struct file *file, void *priv,
				      struct v4l2_frmsizeenum *fsize)
{
	u32 max_width, max_height;
	/*
	 * Only H.264 keeps a 16-pixel step.  Its validator compares the
	 * negotiated width against pic_width_in_mbs_minus1 * 16, so a size off
	 * the macroblock grid cannot decode anyway and should be refused here.
	 * Every other codec states its size in real pixels and is validated
	 * against the aligned value, so 1920x1080 and 854x480 have to be
	 * offered: a 16 step hid them and made ffmpeg fall back to software.
	 */
	u32 step = 1;

	if (fsize->index)
		return -EINVAL;
	if (fsize->pixel_format == V4L2_PIX_FMT_HEVC_SLICE) {
		max_width = HISTB_VDEC_HEVC_MAX_WIDTH;
		max_height = HISTB_VDEC_HEVC_MAX_HEIGHT;
		step = 1;
	} else if (fsize->pixel_format == V4L2_PIX_FMT_H264_SLICE) {
		max_width = HISTB_VDEC_H264_MAX_WIDTH;
		max_height = HISTB_VDEC_H264_MAX_HEIGHT;
		step = 16;
	} else if (fsize->pixel_format == V4L2_PIX_FMT_MPEG1_SLICE ||
		   fsize->pixel_format == V4L2_PIX_FMT_MPEG2_SLICE) {
		max_width = HISTB_VDEC_MPEG2_MAX_WIDTH;
		max_height = HISTB_VDEC_MPEG2_MAX_HEIGHT;
	} else if (fsize->pixel_format == V4L2_PIX_FMT_AVS_SLICE) {
		max_width = HISTB_VDEC_AVS_MAX_WIDTH;
		max_height = HISTB_VDEC_AVS_MAX_HEIGHT;
		step = 1;
	} else if (fsize->pixel_format == V4L2_PIX_FMT_MPEG4) {
		max_width = HISTB_VDEC_MPEG4_MAX_WIDTH;
		max_height = HISTB_VDEC_MPEG4_MAX_HEIGHT;
	} else if (fsize->pixel_format == V4L2_PIX_FMT_VP8_FRAME) {
		max_width = HISTB_VDEC_VP8_MAX_WIDTH;
		max_height = HISTB_VDEC_VP8_MAX_HEIGHT;
	} else if (fsize->pixel_format == V4L2_PIX_FMT_VP9_FRAME) {
		max_width = HISTB_VDEC_VP9_MAX_WIDTH;
		max_height = HISTB_VDEC_VP9_MAX_HEIGHT;
		step = 1;
	} else if (histb_vdec_is_vc1_format(fsize->pixel_format)) {
		max_width = HISTB_VDEC_VC1_MAX_WIDTH;
		max_height = HISTB_VDEC_VC1_MAX_HEIGHT;
	} else if (fsize->pixel_format == V4L2_PIX_FMT_NV12 ||
		   fsize->pixel_format == V4L2_PIX_FMT_P010) {
		max_width = HISTB_VDEC_HEVC_MAX_WIDTH;
		max_height = HISTB_VDEC_HEVC_MAX_HEIGHT;
		step = 1;
	} else {
		return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = HISTB_VDEC_MIN_WIDTH;
	fsize->stepwise.max_width = max_width;
	fsize->stepwise.step_width = step;
	fsize->stepwise.min_height = HISTB_VDEC_MIN_HEIGHT;
	fsize->stepwise.max_height = max_height;
	fsize->stepwise.step_height = step;
	return 0;
}

static void histb_vdec_try_output_format(struct v4l2_pix_format *pix)
{
	u32 max_width = HISTB_VDEC_H264_MAX_WIDTH;
	u32 max_height = HISTB_VDEC_H264_MAX_HEIGHT;
	u32 alignment = 4;
	u32 width = pix->width;
	u32 height = pix->height;
	u32 pixelformat = pix->pixelformat;

	if (pixelformat == V4L2_PIX_FMT_HEVC_SLICE) {
		max_width = HISTB_VDEC_HEVC_MAX_WIDTH;
		max_height = HISTB_VDEC_HEVC_MAX_HEIGHT;
		alignment = 0;
	} else if (pixelformat == V4L2_PIX_FMT_MPEG1_SLICE ||
		   pixelformat == V4L2_PIX_FMT_MPEG2_SLICE) {
		max_width = HISTB_VDEC_MPEG2_MAX_WIDTH;
		max_height = HISTB_VDEC_MPEG2_MAX_HEIGHT;
	} else if (pixelformat == V4L2_PIX_FMT_AVS_SLICE) {
		max_width = HISTB_VDEC_AVS_MAX_WIDTH;
		max_height = HISTB_VDEC_AVS_MAX_HEIGHT;
		alignment = 0;
	} else if (pixelformat == V4L2_PIX_FMT_MPEG4) {
		max_width = HISTB_VDEC_MPEG4_MAX_WIDTH;
		max_height = HISTB_VDEC_MPEG4_MAX_HEIGHT;
	} else if (pixelformat == V4L2_PIX_FMT_VP8_FRAME) {
		max_width = HISTB_VDEC_VP8_MAX_WIDTH;
		max_height = HISTB_VDEC_VP8_MAX_HEIGHT;
	} else if (pixelformat == V4L2_PIX_FMT_VP9_FRAME) {
		max_width = HISTB_VDEC_VP9_MAX_WIDTH;
		max_height = HISTB_VDEC_VP9_MAX_HEIGHT;
		/* VP9 controls carry the exact frame size; storage is aligned later. */
		alignment = 0;
	} else if (histb_vdec_is_vc1_format(pixelformat)) {
		max_width = HISTB_VDEC_VC1_MAX_WIDTH;
		max_height = HISTB_VDEC_VC1_MAX_HEIGHT;
	} else {
		pixelformat = V4L2_PIX_FMT_H264_SLICE;
	}

	v4l_bound_align_image(&width, HISTB_VDEC_MIN_WIDTH,
			      max_width, alignment, &height,
			      HISTB_VDEC_MIN_HEIGHT, max_height,
			      alignment, 0);
	pix->width = width;
	pix->height = height;
	pix->pixelformat = pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = 0;
	pix->sizeimage = clamp_t(u32, pix->sizeimage,
				 HISTB_VDEC_MIN_BITSTREAM,
				 HISTB_VDEC_MAX_BITSTREAM);
	pix->colorspace = V4L2_COLORSPACE_REC709;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static void histb_vdec_try_capture_format(struct histb_vdec_ctx *ctx,
					  struct v4l2_pix_format *pix)
{
	bool p010 = ctx->src.pix.pixelformat == V4L2_PIX_FMT_HEVC_SLICE &&
		pix->pixelformat == V4L2_PIX_FMT_P010;

	pix->width = ctx->src.pix.width;
	pix->height = ctx->src.pix.height;
	pix->pixelformat = p010 ? V4L2_PIX_FMT_P010 : V4L2_PIX_FMT_NV12;
	pix->field = V4L2_FIELD_NONE;
	/* Both public capture formats are linear; CV200 tile storage is internal. */
	pix->bytesperline = p010 ? pix->width * 2 : ALIGN(pix->width, 64);
	pix->sizeimage = pix->bytesperline * pix->height * 3 / 2;
	pix->colorspace = ctx->src.pix.colorspace;
	pix->ycbcr_enc = ctx->src.pix.ycbcr_enc;
	pix->quantization = ctx->src.pix.quantization;
	pix->xfer_func = ctx->src.pix.xfer_func;
}

static int histb_vdec_try_fmt(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct histb_vdec_ctx *ctx = fh_to_histb_vdec_ctx(priv);

	if (V4L2_TYPE_IS_OUTPUT(f->type))
		histb_vdec_try_output_format(&f->fmt.pix);
	else
		histb_vdec_try_capture_format(ctx, &f->fmt.pix);

	return 0;
}

static int histb_vdec_g_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct histb_vdec_ctx *ctx = fh_to_histb_vdec_ctx(priv);

	f->fmt.pix = histb_vdec_get_q_data(ctx, f->type)->pix;
	return 0;
}

static int histb_vdec_s_fmt(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct histb_vdec_ctx *ctx = fh_to_histb_vdec_ctx(priv);
	struct vb2_queue *peer_vq, *vq;
	int ret;

	ret = histb_vdec_try_fmt(file, priv, f);
	if (ret)
		return ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq))
		return -EBUSY;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		peer_vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);
		if (vb2_is_busy(peer_vq))
			return -EBUSY;
	}

	histb_vdec_get_q_data(ctx, f->type)->pix = f->fmt.pix;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		struct v4l2_pix_format capture = ctx->dst.pix;

		vq->requires_requests =
			f->fmt.pix.pixelformat != V4L2_PIX_FMT_MPEG4 &&
			!histb_vdec_is_vc1_format(f->fmt.pix.pixelformat);
		if (f->fmt.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE) {
			vq->subsystem_flags |=
				VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF;
		} else {
			vq->subsystem_flags &=
				~VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF;
		}
		histb_vdec_reset_h264_slices(ctx);
		histb_vdec_reset_h264_field_pair(ctx);
		histb_vdec_free_buffers(ctx);
		histb_vdec_try_capture_format(ctx, &capture);
		ctx->dst.pix = capture;
	}

	return 0;
}

static int histb_vdec_try_decoder_cmd(struct file *file, void *priv,
				      struct v4l2_decoder_cmd *dc)
{
	struct histb_vdec_ctx *ctx = fh_to_histb_vdec_ctx(priv);

	if (histb_vdec_is_stateless_format(ctx->src.pix.pixelformat))
		return v4l2_m2m_ioctl_stateless_try_decoder_cmd(file, priv, dc);

	return v4l2_m2m_ioctl_try_decoder_cmd(file, priv, dc);
}

static int histb_vdec_decoder_cmd(struct file *file, void *priv,
				  struct v4l2_decoder_cmd *dc)
{
	struct histb_vdec_ctx *ctx = fh_to_histb_vdec_ctx(priv);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct histb_mpeg4_display_plan plan;
	struct histb_mpeg4_display_state state;
	struct vb2_v4l2_buffer *last_dst;
	unsigned long flags;
	bool active, held, queued_src;
	int ret;

	ret = histb_vdec_try_decoder_cmd(file, priv, dc);
	if (ret)
		return ret;
	if (dc->cmd == V4L2_DEC_CMD_FLUSH) {
		/*
		 * FLUSH only releases a capture buffer held by the stateless m2m
		 * protocol.  It must not discard timestamp-addressed DPB state.
		 */
		queued_src = !!v4l2_m2m_last_src_buf(m2m_ctx);
		last_dst = v4l2_m2m_last_dst_buf(m2m_ctx);
		held = last_dst && last_dst->is_held;
		ret = v4l2_m2m_ioctl_stateless_decoder_cmd(file, priv, dc);
		if (ret)
			return ret;

		/* A held H.264 slice aggregate has no request left to finish it. */
		spin_lock_irqsave(&ctx->vdec->irqlock, flags);
		active = ctx->vdec->curr_ctx == ctx || ctx->vdec->post_ctx == ctx;
		spin_unlock_irqrestore(&ctx->vdec->irqlock, flags);
		if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE &&
		    !active && !queued_src && held) {
			histb_vdec_reset_h264_slices(ctx);
			histb_vdec_reset_h264_field_pair(ctx);
		}
		return 0;
	}
	if (!vb2_is_streaming(v4l2_m2m_get_src_vq(m2m_ctx)) ||
	    !vb2_is_streaming(v4l2_m2m_get_dst_vq(m2m_ctx)))
		return 0;
	if (dc->cmd == V4L2_DEC_CMD_START) {
		bool mpeg4 = ctx->src.pix.pixelformat == V4L2_PIX_FMT_MPEG4;

		if (mpeg4)
			mutex_lock(&ctx->mpeg4_lock);
		ret = v4l2_m2m_decoder_cmd(file, m2m_ctx, dc);
		if (!ret) {
			vb2_clear_last_buffer_dequeued(
				v4l2_m2m_get_dst_vq(m2m_ctx));
			if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE) {
				if (ctx->h264_pending_field)
					ctx->dst.sequence++;
				histb_vdec_reset_h264_field_pair(ctx);
			}
		}
		if (mpeg4)
			mutex_unlock(&ctx->mpeg4_lock);
		if (!ret)
			v4l2_m2m_try_schedule(m2m_ctx);
		return ret;
	}
	if (ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE &&
	    ctx->h264_partial && !v4l2_m2m_num_src_bufs_ready(m2m_ctx)) {
		struct vb2_v4l2_buffer *dst;
		unsigned int i;

		if (m2m_ctx->is_draining)
			return -EBUSY;
		if (m2m_ctx->has_stopped)
			return 0;
		m2m_ctx->last_src_buf = NULL;
		m2m_ctx->is_draining = true;
		dst = v4l2_m2m_dst_buf_remove(m2m_ctx);
		if (dst) {
			histb_vdec_decoded_buffer(dst)->error_tainted = true;
			dst->is_held = false;
			for (i = 0; i < dst->vb2_buf.num_planes; i++)
				vb2_set_plane_payload(&dst->vb2_buf, i, 0);
			dst->sequence = ctx->dst.sequence++;
			v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
		}
		histb_vdec_reset_h264_slices(ctx);
		histb_vdec_reset_h264_field_pair(ctx);
		histb_vdec_reset_apc(ctx);
		histb_vdec_complete_empty_last(ctx);
		return 0;
	}
	if (histb_vdec_is_vc1_format(ctx->src.pix.pixelformat)) {
		mutex_lock(&ctx->vc1_lock);
		if (dc->cmd != V4L2_DEC_CMD_STOP ||
		    v4l2_m2m_num_src_bufs_ready(m2m_ctx)) {
			ret = v4l2_m2m_decoder_cmd(file, m2m_ctx, dc);
			goto unlock_vc1;
		}
		if (m2m_ctx->is_draining) {
			ret = -EBUSY;
			goto unlock_vc1;
		}
		if (m2m_ctx->has_stopped) {
			ret = 0;
			goto unlock_vc1;
		}

		m2m_ctx->last_src_buf = NULL;
		m2m_ctx->is_draining = true;
		if (ctx->vc1_display_state == HISTB_VDEC_VC1_DISPLAY_BUFFER &&
		    ctx->vc1_display_pending) {
			histb_vdec_return_vc1_display(ctx, VB2_BUF_STATE_DONE, true);
		} else {
			histb_vdec_return_vc1_display(ctx, VB2_BUF_STATE_DONE, false);
			histb_vdec_complete_empty_last(ctx);
		}
		ret = 0;

unlock_vc1:
		mutex_unlock(&ctx->vc1_lock);
		return ret;
	}
	if (ctx->src.pix.pixelformat != V4L2_PIX_FMT_MPEG4) {
		bool stopped = m2m_ctx->has_stopped;

		ret = v4l2_m2m_decoder_cmd(file, m2m_ctx, dc);
		if (!ret && !stopped && m2m_ctx->has_stopped)
			v4l2_event_queue_fh(&ctx->fh, &histb_vdec_eos_event);
		if (!ret && dc->cmd == V4L2_DEC_CMD_STOP && m2m_ctx->has_stopped &&
		    ctx->src.pix.pixelformat == V4L2_PIX_FMT_H264_SLICE) {
			if (ctx->h264_pending_field)
				ctx->dst.sequence++;
			histb_vdec_reset_h264_field_pair(ctx);
		}
		return ret;
	}

	mutex_lock(&ctx->mpeg4_lock);
	if (dc->cmd != V4L2_DEC_CMD_STOP ||
	    !ctx->mpeg4_display_pending ||
	    v4l2_m2m_num_src_bufs_ready(m2m_ctx)) {
		bool stopped = m2m_ctx->has_stopped;

		ret = v4l2_m2m_decoder_cmd(file, m2m_ctx, dc);
		if (!ret && !stopped && m2m_ctx->has_stopped)
			v4l2_event_queue_fh(&ctx->fh, &histb_vdec_eos_event);
		goto unlock;
	}

	if (m2m_ctx->is_draining) {
		ret = -EBUSY;
		goto unlock;
	}
	if (m2m_ctx->has_stopped) {
		ret = 0;
		goto unlock;
	}
	state = ctx->mpeg4_display_state;
	ret = histb_mpeg4_plan_drain(&state, &plan);
	if (ret || !plan.release_pending || !plan.pending_last) {
		ret = -EINVAL;
		goto unlock;
	}
	m2m_ctx->last_src_buf = NULL;
	m2m_ctx->is_draining = true;
	histb_vdec_return_mpeg4_display(ctx, VB2_BUF_STATE_DONE, true);
	WARN_ON(ctx->mpeg4_display_state.pending != state.pending);
	ret = 0;

unlock:
	mutex_unlock(&ctx->mpeg4_lock);
	return ret;
}

static int histb_vdec_subscribe_event(
		struct v4l2_fh *fh,
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

static const struct v4l2_ioctl_ops histb_vdec_ioctl_ops = {
	.vidioc_querycap = histb_vdec_querycap,
	.vidioc_enum_fmt_vid_cap = histb_vdec_enum_fmt,
	.vidioc_g_fmt_vid_cap = histb_vdec_g_fmt,
	.vidioc_try_fmt_vid_cap = histb_vdec_try_fmt,
	.vidioc_s_fmt_vid_cap = histb_vdec_s_fmt,
	.vidioc_enum_fmt_vid_out = histb_vdec_enum_fmt,
	.vidioc_g_fmt_vid_out = histb_vdec_g_fmt,
	.vidioc_try_fmt_vid_out = histb_vdec_try_fmt,
	.vidioc_s_fmt_vid_out = histb_vdec_s_fmt,
	.vidioc_enum_framesizes = histb_vdec_enum_framesizes,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_decoder_cmd = histb_vdec_decoder_cmd,
	.vidioc_try_decoder_cmd = histb_vdec_try_decoder_cmd,
	.vidioc_subscribe_event = histb_vdec_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_ctrl_config histb_vdec_ctrls[] = {
	{ .id = V4L2_CID_STATELESS_VP8_FRAME },
	{ .id = V4L2_CID_STATELESS_VP9_FRAME },
	{ .id = V4L2_CID_STATELESS_VP9_COMPRESSED_HDR },
	{ .id = V4L2_CID_STATELESS_AVS_SEQUENCE },
	{ .id = V4L2_CID_STATELESS_AVS_PICTURE },
	{
		.id = V4L2_CID_STATELESS_AVS_SLICE_PARAMS,
		.dims = { HISTB_VDEC_AVS_MAX_SLICES },
	},
	{ .id = V4L2_CID_STATELESS_AVS_DECODE_PARAMS },
	{
		/* Full range with an explicit check in histb_vdec_request_validate():
		 * pinning min == max makes the core clamp a client that asks for
		 * something else, which is a silent behaviour change.  AVS has a single mode. */
		.id = V4L2_CID_STATELESS_AVS_DECODE_MODE,
		.min = V4L2_STATELESS_AVS_DECODE_MODE_FRAME_BASED,
		.max = V4L2_STATELESS_AVS_DECODE_MODE_FRAME_BASED,
		.def = V4L2_STATELESS_AVS_DECODE_MODE_FRAME_BASED,
	},
	{
		/* Full range with an explicit check in histb_vdec_request_validate():
		 * pinning min == max makes the core clamp a client that asks for
		 * something else, which is a silent behaviour change.  AVS has a single start-code form. */
		.id = V4L2_CID_STATELESS_AVS_START_CODE,
		.min = V4L2_STATELESS_AVS_START_CODE_PREFIX,
		.max = V4L2_STATELESS_AVS_START_CODE_PREFIX,
		.def = V4L2_STATELESS_AVS_START_CODE_PREFIX,
	},
	{ .id = V4L2_CID_STATELESS_MPEG2_SEQUENCE },
	{ .id = V4L2_CID_STATELESS_MPEG2_PICTURE },
	{ .id = V4L2_CID_STATELESS_MPEG2_QUANTISATION },
	{ .id = V4L2_CID_STATELESS_MPEG1_SEQUENCE },
	{ .id = V4L2_CID_STATELESS_MPEG1_PICTURE },
	{ .id = V4L2_CID_STATELESS_MPEG1_QUANTISATION },
	{ .id = V4L2_CID_STATELESS_H264_DECODE_PARAMS },
	{ .id = V4L2_CID_STATELESS_H264_SPS },
	{ .id = V4L2_CID_STATELESS_H264_PPS },
	{ .id = V4L2_CID_STATELESS_H264_SCALING_MATRIX },
	{ .id = V4L2_CID_STATELESS_H264_PRED_WEIGHTS },
	{ .id = V4L2_CID_STATELESS_H264_SLICE_PARAMS },
	{
		/* Full range with an explicit check in histb_vdec_request_validate():
		 * pinning min == max makes the core clamp a client that asks for
		 * something else, which is a silent behaviour change.  Only SLICE_BASED is supported. */
		.id = V4L2_CID_STATELESS_H264_DECODE_MODE,
		.min = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
		.max = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
		.def = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
	},
	{
		/* Full range with an explicit check in histb_vdec_request_validate():
		 * pinning min == max makes the core clamp a client that asks for
		 * something else, which is a silent behaviour change.  Only annex-B input is supported. */
		.id = V4L2_CID_STATELESS_H264_START_CODE,
		.min = V4L2_STATELESS_H264_START_CODE_NONE,
		.max = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.def = V4L2_STATELESS_H264_START_CODE_NONE,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.min = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.max = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
		.def = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.min = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		/* The datasheet states level 5.0 (4Kx2K@30); the BSP header
		 * says 4.1.  The datasheet is the public statement of the part
		 * and outranks a software guard. */
		.max = V4L2_MPEG_VIDEO_H264_LEVEL_5_0,
		.def = V4L2_MPEG_VIDEO_H264_LEVEL_5_0,
	},
	{ .id = V4L2_CID_STATELESS_HEVC_SPS },
	{ .id = V4L2_CID_STATELESS_HEVC_PPS },
	{
		.id = V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
		.dims = { HISTB_VDEC_HEVC_MAX_SLICES },
	},
	{ .id = V4L2_CID_STATELESS_HEVC_SCALING_MATRIX },
	{ .id = V4L2_CID_STATELESS_HEVC_DECODE_PARAMS },
	{
		.id = V4L2_CID_STATELESS_HEVC_ENTRY_POINT_OFFSETS,
		.dims = { HISTB_VDEC_HEVC_MAX_ENTRY_POINTS },
		.max = U32_MAX,
		.step = 1,
	},
	/*
	 * The four HEVC menu controls (decode mode, start code, profile, level)
	 * used to be listed here.  They were never read by the driver - the
	 * capability is expressed by the stateless controls above - and they
	 * could not be registered at all: only cfg->type is given, so the core
	 * has no qmenu for a menu-type control and v4l2_ctrl_new_custom() fails
	 * with -ERANGE.  That error lands in ctrl_handler.error, which
	 * histb_vdec_init_ctrls() returns, so open() failed with -ERANGE and no
	 * client could reach the hardware at all.  Verified with a per-control
	 * diagnostic: indices 31-34, name=HEVC Decode Mode/Start Code/Profile/
	 * Level, type=0, err=-34, while all 31 controls before them registered.
	 */
	{
		.name = "HEVC Decode Mode",
		.id = V4L2_CID_STATELESS_HEVC_DECODE_MODE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
		.min = V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED,
		.max = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
		.def = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
	},
	{
		.name = "HEVC Start Code",
		.id = V4L2_CID_STATELESS_HEVC_START_CODE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
		.min = V4L2_STATELESS_HEVC_START_CODE_NONE,
		.max = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B,
		.def = V4L2_STATELESS_HEVC_START_CODE_NONE,
	},
};

static int histb_vdec_init_ctrls(struct histb_vdec_ctx *ctx)
{
	unsigned int i;
	int ret;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler,
			       ARRAY_SIZE(histb_vdec_ctrls));
	for (i = 0; i < ARRAY_SIZE(histb_vdec_ctrls); i++)
		v4l2_ctrl_new_custom(&ctx->ctrl_handler,
				     &histb_vdec_ctrls[i], ctx);
	if (ctx->ctrl_handler.error) {
		int ret = ctx->ctrl_handler.error;

		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		return ret;
	}

	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	ret = v4l2_ctrl_handler_setup(&ctx->ctrl_handler);
	if (ret)
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);

	return ret;
}

static void histb_vdec_set_default_formats(struct histb_vdec_ctx *ctx)
{
	ctx->src.pix.width = HISTB_VDEC_DEFAULT_WIDTH;
	ctx->src.pix.height = HISTB_VDEC_DEFAULT_HEIGHT;
	ctx->src.pix.sizeimage = HISTB_VDEC_DEFAULT_BITSTREAM;
	histb_vdec_try_output_format(&ctx->src.pix);
	ctx->dst.pix.width = HISTB_VDEC_DEFAULT_WIDTH;
	ctx->dst.pix.height = HISTB_VDEC_DEFAULT_HEIGHT;
	histb_vdec_try_capture_format(ctx, &ctx->dst.pix);
}

static int histb_vdec_open(struct file *file)
{
	struct histb_vdec_dev *vdec = video_drvdata(file);
	struct histb_vdec_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->vdec = vdec;
	mutex_init(&ctx->mpeg4_lock);
	mutex_init(&ctx->vc1_lock);
	init_completion(&ctx->vc1_setup_idle);
	complete(&ctx->vc1_setup_idle);
	histb_vdec_set_default_formats(ctx);
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;

	ret = histb_vdec_init_ctrls(ctx);
	if (ret)
		goto free_fh;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(vdec->m2m_dev, ctx,
					    histb_vdec_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto free_ctrls;
	}

	v4l2_fh_add(&ctx->fh);
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
free_fh:
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int histb_vdec_release(struct file *file)
{
	struct histb_vdec_ctx *ctx =
		container_of(file->private_data, struct histb_vdec_ctx, fh);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	histb_vdec_free_buffers(ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations histb_vdec_fops = {
	.owner = THIS_MODULE,
	.open = histb_vdec_open,
	.release = histb_vdec_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static int histb_vdec_request_validate(struct media_request *request)
{
	struct media_request_object *obj;
	struct v4l2_ctrl_handler *hdl;
	struct histb_vdec_ctx *ctx = NULL;
	static const u32 mpeg2_controls[] = {
		V4L2_CID_STATELESS_MPEG2_SEQUENCE,
		V4L2_CID_STATELESS_MPEG2_PICTURE,
	};
	static const u32 mpeg1_controls[] = {
		V4L2_CID_STATELESS_MPEG1_SEQUENCE,
		V4L2_CID_STATELESS_MPEG1_PICTURE,
	};
	static const u32 vp8_controls[] = {
		V4L2_CID_STATELESS_VP8_FRAME,
	};
	static const u32 vp9_controls[] = {
		V4L2_CID_STATELESS_VP9_FRAME,
		V4L2_CID_STATELESS_VP9_COMPRESSED_HDR,
	};
	static const u32 avs_controls[] = {
		V4L2_CID_STATELESS_AVS_SEQUENCE,
		V4L2_CID_STATELESS_AVS_PICTURE,
		V4L2_CID_STATELESS_AVS_SLICE_PARAMS,
		V4L2_CID_STATELESS_AVS_DECODE_PARAMS,
	};
	static const u32 h264_controls[] = {
		V4L2_CID_STATELESS_H264_SPS,
		V4L2_CID_STATELESS_H264_PPS,
		V4L2_CID_STATELESS_H264_SCALING_MATRIX,
		V4L2_CID_STATELESS_H264_SLICE_PARAMS,
		V4L2_CID_STATELESS_H264_DECODE_PARAMS,
	};
	static const u32 hevc_controls[] = {
		V4L2_CID_STATELESS_HEVC_SPS,
		V4L2_CID_STATELESS_HEVC_PPS,
		V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
		V4L2_CID_STATELESS_HEVC_DECODE_PARAMS,
	};
	const u32 *required = NULL;
	unsigned int required_count = 0;
	unsigned int count = vb2_request_buffer_cnt(request);
	unsigned int i;

	if (!count)
		return -ENOENT;
	if (count > 1)
		return -EINVAL;
	list_for_each_entry(obj, &request->objects, list) {
		struct vb2_buffer *vb;

		if (!vb2_request_object_is_buffer(obj))
			continue;
		vb = container_of(obj, struct vb2_buffer, req_obj);
		ctx = vb2_get_drv_priv(vb->vb2_queue);
		break;
	}
	if (!ctx)
		return -ENOENT;
	switch (ctx->src.pix.pixelformat) {
	case V4L2_PIX_FMT_MPEG1_SLICE:
		required = mpeg1_controls;
		required_count = ARRAY_SIZE(mpeg1_controls);
		break;
	case V4L2_PIX_FMT_MPEG2_SLICE:
		required = mpeg2_controls;
		required_count = ARRAY_SIZE(mpeg2_controls);
		break;
	case V4L2_PIX_FMT_VP8_FRAME:
		required = vp8_controls;
		required_count = ARRAY_SIZE(vp8_controls);
		break;
	case V4L2_PIX_FMT_VP9_FRAME:
		required = vp9_controls;
		required_count = ARRAY_SIZE(vp9_controls);
		break;
	case V4L2_PIX_FMT_AVS_SLICE:
		required = avs_controls;
		required_count = ARRAY_SIZE(avs_controls);
		break;
	case V4L2_PIX_FMT_H264_SLICE:
		required = h264_controls;
		required_count = ARRAY_SIZE(h264_controls);
		break;
	case V4L2_PIX_FMT_HEVC_SLICE:
		required = hevc_controls;
		required_count = ARRAY_SIZE(hevc_controls);
		break;
	default:
		break;
	}
	if (required_count) {
		hdl = v4l2_ctrl_request_hdl_find(request, &ctx->ctrl_handler);
		if (!hdl)
			return -ENOENT;
		for (i = 0; i < required_count; i++) {
			if (!v4l2_ctrl_request_hdl_ctrl_find(hdl, required[i])) {
				v4l2_ctrl_request_hdl_put(hdl);
				return -ENOENT;
			}
		}
		/*
		 * The mode and start-code controls are registered with their full
		 * range so that a client asking for an unsupported combination is
		 * told, rather than silently clamped to what the driver assumes.
		 */
		{
			static const struct {
				u32 id;
				s32 want;
			} fixed[] = {
				{ V4L2_CID_STATELESS_H264_DECODE_MODE,
				  V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED },
				{ V4L2_CID_STATELESS_H264_START_CODE,
				  V4L2_STATELESS_H264_START_CODE_NONE },
				{ V4L2_CID_STATELESS_HEVC_DECODE_MODE,
				  V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED },
				{ V4L2_CID_STATELESS_HEVC_START_CODE,
				  V4L2_STATELESS_HEVC_START_CODE_NONE },
			};
			unsigned int k;

			for (k = 0; k < ARRAY_SIZE(fixed); k++) {
				struct v4l2_ctrl *c;

				c = v4l2_ctrl_request_hdl_ctrl_find(hdl, fixed[k].id);
				if (c && c->p_cur.p && c->p_cur.p_s32 &&
				    c->p_cur.p_s32[0] != fixed[k].want) {
					dev_err(ctx->vdec->dev,
						"control 0x%x: %d is not supported, only %d\n",
						fixed[k].id, c->p_cur.p_s32[0],
						fixed[k].want);
					v4l2_ctrl_request_hdl_put(hdl);
					return -EINVAL;
				}
			}
		}
		v4l2_ctrl_request_hdl_put(hdl);
	}

	return vb2_request_validate(request);
}

static const struct media_device_ops histb_vdec_media_ops = {
	.req_validate = histb_vdec_request_validate,
	.req_queue = v4l2_m2m_request_queue,
};

static int histb_vdec_runtime_resume(struct device *dev)
{
	struct histb_vdec_dev *vdec = dev_get_drvdata(dev);
	int ret;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(vdec->clocks), vdec->clocks);
	if (ret)
		return ret;

	ret = reset_control_bulk_deassert(ARRAY_SIZE(vdec->resets),
					  vdec->resets);
	if (ret) {
		clk_bulk_disable_unprepare(ARRAY_SIZE(vdec->clocks),
					   vdec->clocks);
		return ret;
	}

	/* AVS enables its identity table immediately before each task. */
	histb_vdec_program_smmu(vdec, false);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	enable_irq(vdec->irq);
	return 0;
}

static int histb_vdec_runtime_suspend(struct device *dev)
{
	struct histb_vdec_dev *vdec = dev_get_drvdata(dev);

	vdec->avsp_loaded = false;
	if (vdec->smmu_identity_ready)
		vdec->smmu_identity_needs_reset = true;

	disable_irq(vdec->irq);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_MASK);
	writel(0, vdec->regs + HISTB_VDEC_START);
	histb_vdec_program_smmu(vdec, false);
	writel(~0U, vdec->regs + HISTB_VDEC_INT_STATE);
	reset_control_bulk_assert(ARRAY_SIZE(vdec->resets), vdec->resets);
	clk_bulk_disable_unprepare(ARRAY_SIZE(vdec->clocks), vdec->clocks);
	return 0;
}

static int __maybe_unused histb_vdec_suspend(struct device *dev)
{
	struct histb_vdec_dev *vdec = dev_get_drvdata(dev);
	int ret;

	v4l2_m2m_suspend(vdec->m2m_dev);
	ret = pm_runtime_force_suspend(dev);
	if (ret)
		v4l2_m2m_resume(vdec->m2m_dev);

	return ret;
}

static int __maybe_unused histb_vdec_resume(struct device *dev)
{
	struct histb_vdec_dev *vdec = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (!ret)
		v4l2_m2m_resume(vdec->m2m_dev);

	return ret;
}

static const struct dev_pm_ops histb_vdec_pm_ops = {
	SET_RUNTIME_PM_OPS(histb_vdec_runtime_suspend,
			   histb_vdec_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(histb_vdec_suspend, histb_vdec_resume)
};

static int histb_vdec_register_video(struct histb_vdec_dev *vdec)
{
	int ret;

	ret = v4l2_device_register(vdec->dev, &vdec->v4l2_dev);
	if (ret)
		return ret;

	vdec->m2m_dev = v4l2_m2m_init(&histb_vdec_m2m_ops);
	if (IS_ERR(vdec->m2m_dev)) {
		ret = PTR_ERR(vdec->m2m_dev);
		goto unregister_v4l2;
	}

	vdec->mdev.dev = vdec->dev;
	strscpy(vdec->mdev.model, "HiSilicon Hi3798CV200 VDH",
		sizeof(vdec->mdev.model));
	strscpy(vdec->mdev.bus_info, "platform:histb-vdec",
		sizeof(vdec->mdev.bus_info));
	media_device_init(&vdec->mdev);
	vdec->mdev.ops = &histb_vdec_media_ops;
	vdec->v4l2_dev.mdev = &vdec->mdev;

	strscpy(vdec->vfd.name, "histb-vdec", sizeof(vdec->vfd.name));
	vdec->vfd.fops = &histb_vdec_fops;
	vdec->vfd.ioctl_ops = &histb_vdec_ioctl_ops;
	vdec->vfd.v4l2_dev = &vdec->v4l2_dev;
	vdec->vfd.lock = &vdec->lock;
	vdec->vfd.release = video_device_release_empty;
	vdec->vfd.vfl_dir = VFL_DIR_M2M;
	vdec->vfd.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(&vdec->vfd, vdec);

	ret = video_register_device(&vdec->vfd, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto cleanup_media;

	ret = v4l2_m2m_register_media_controller(vdec->m2m_dev, &vdec->vfd,
						 MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (ret)
		goto unregister_video;

	ret = media_device_register(&vdec->mdev);
	if (ret)
		goto unregister_media_controller;

	return 0;

unregister_media_controller:
	v4l2_m2m_unregister_media_controller(vdec->m2m_dev);
unregister_video:
	video_unregister_device(&vdec->vfd);
cleanup_media:
	media_device_cleanup(&vdec->mdev);
	v4l2_m2m_release(vdec->m2m_dev);
unregister_v4l2:
	v4l2_device_unregister(&vdec->v4l2_dev);
	return ret;
}

static void histb_vdec_unregister_video(struct histb_vdec_dev *vdec)
{
	media_device_unregister(&vdec->mdev);
	v4l2_m2m_unregister_media_controller(vdec->m2m_dev);
	video_unregister_device(&vdec->vfd);
	media_device_cleanup(&vdec->mdev);
	v4l2_m2m_release(vdec->m2m_dev);
	v4l2_device_unregister(&vdec->v4l2_dev);
}

static void histb_vdec_put_vpss(void *data)
{
	histb_vpss_put(data);
}

/*
 * True when the reserved memory region named by the device's memory-region
 * property starts beyond the addresses the device is allowed to use, i.e.
 * when binding it would hand every coherent allocation an address the device
 * cannot reach.
 */
static bool histb_vdec_pool_unreachable(struct device *dev)
{
	struct device_node *np;
	struct resource res;
	dma_addr_t limit;
	bool unreachable = false;

	if (!dev->of_node)
		return false;
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np)
		return false;

	limit = dev->bus_dma_limit;
	if (!limit || limit > dma_get_mask(dev))
		limit = dma_get_mask(dev);
	if (limit && of_address_to_resource(np, 0, &res) == 0)
		unreachable = (dma_addr_t)res.start > limit;

	of_node_put(np);

	return unreachable;
}

static int histb_vdec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_vdec_dev *vdec;
	unsigned int num_resets;
	int ret;

	vdec = devm_kzalloc(dev, sizeof(*vdec), GFP_KERNEL);
	if (!vdec)
		return -ENOMEM;

	vdec->dev = dev;
	/*
	 * The decoder needs large physically contiguous allocations, which
	 * fail from the shared CMA pool once the desktop has filled it with
	 * movable pages.  Use the board's dedicated decoder pool when the
	 * device tree provides one.
	 */
	/*
	 * The device can only reach what its DMA aperture allows (measured:
	 * bus_dma_limit 0x3fffffff for this node).  A memory-region that lies
	 * outside it cannot be used for DMA at all, yet
	 * of_reserved_mem_device_init() binds it unconditionally, and every
	 * coherent allocation then comes from beyond the limit and fails.
	 *
	 * On this board that is exactly the case: vdec-pool@50000000 sits at
	 * 1.25 GiB while the decoder is limited to 1 GiB, because the pool was
	 * moved past the aperture on the reasoning that the Mali - which does
	 * live outside soc - no longer needed the low window.  The pool belongs
	 * to the decoder, which never left soc, so it cannot reach it.
	 *
	 * Refuse a pool the device cannot address and let the shared CMA serve
	 * the allocations instead.  That is what makes hardware decode work
	 * here: with the pool bound, every h264 frame is refused with -EINVAL
	 * because the MSG buffer's dma_alloc_coherent() fails; without it,
	 * decode is byte-identical to the software decoder.
	 */
	ret = histb_vdec_pool_unreachable(dev);
	if (ret)
		dev_warn(dev,
			 "reserved decoder pool is outside the DMA aperture (limit 0x%llx); using CMA instead\n",
			 (unsigned long long)dev->bus_dma_limit);
	else {
		ret = of_reserved_mem_device_init(dev);
		if (ret && ret != -ENODEV)
			return dev_err_probe(dev, ret,
					     "failed to get the decoder memory pool\n");
	}
	vdec->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vdec->regs))
		return PTR_ERR(vdec->regs);
	vdec->crg = syscon_regmap_lookup_by_compatible(
					"hisilicon,hi3798cv200-crg");
	if (IS_ERR(vdec->crg))
		return dev_err_probe(dev, PTR_ERR(vdec->crg),
				     "failed to get CRG syscon\n");

	vdec->clocks[0].id = "vdh";
	vdec->clocks[1].id = "dsp";
	vdec->clocks[2].id = "bpd";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(vdec->clocks), vdec->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	vdec->resets[0].id = "vdh";
	vdec->resets[1].id = "scd";
	vdec->resets[2].id = "mfd";
	vdec->resets[3].id = "bpd";
	num_resets = ARRAY_SIZE(vdec->resets);
	ret = devm_reset_control_bulk_get_exclusive(dev, num_resets,
						    vdec->resets);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get resets\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");
	ret = devm_add_action_or_reset(dev, histb_vdec_release_smmu_identity,
				       vdec);
	if (ret)
		return ret;
	vdec->vpss = histb_vpss_get(dev);
	if (IS_ERR(vdec->vpss))
		return dev_err_probe(dev, PTR_ERR(vdec->vpss),
				     "failed to get VPSS\n");
	ret = devm_add_action_or_reset(dev, histb_vdec_put_vpss, vdec->vpss);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, histb_vdec_release_avsp, vdec);
	if (ret)
		return ret;

	vdec->irq = platform_get_irq_byname(pdev, "vdh");
	if (vdec->irq < 0)
		return vdec->irq;

	mutex_init(&vdec->lock);
	mutex_init(&vdec->launch_lock);
	spin_lock_init(&vdec->irqlock);
	INIT_DELAYED_WORK(&vdec->watchdog_work, histb_vdec_watchdog);
	INIT_WORK(&vdec->postprocess_work, histb_vdec_postprocess);
	init_completion(&vdec->postprocess_idle);
	complete(&vdec->postprocess_idle);
	platform_set_drvdata(pdev, vdec);

	ret = devm_request_threaded_irq(dev, vdec->irq, NULL,
					histb_vdec_irq_thread,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					dev_name(dev), vdec);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request VDH IRQ\n");

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
	ret = devm_device_add_group(dev, &histb_vdec_attr_group);
	if (ret)
		goto disable_runtime_pm;

	ret = histb_vdec_register_video(vdec);
	if (ret)
		goto disable_runtime_pm;

	dev_info(dev, "registered as /dev/%s\n",
		 video_device_node_name(&vdec->vfd));
	return 0;

disable_runtime_pm:
	pm_runtime_disable(dev);
	return ret;
}

static void histb_vdec_remove(struct platform_device *pdev)
{
	struct histb_vdec_dev *vdec = platform_get_drvdata(pdev);

	v4l2_m2m_suspend(vdec->m2m_dev);
	synchronize_irq(vdec->irq);
	cancel_delayed_work_sync(&vdec->watchdog_work);
	cancel_work_sync(&vdec->postprocess_work);
	histb_vdec_unregister_video(vdec);
	pm_runtime_disable(vdec->dev);
	if (!pm_runtime_status_suspended(vdec->dev))
		histb_vdec_runtime_suspend(vdec->dev);
	of_reserved_mem_device_release(vdec->dev);
}

static const struct of_device_id histb_vdec_of_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-vdec" },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_vdec_of_match);

static struct platform_driver histb_vdec_driver = {
	.probe = histb_vdec_probe,
	.remove_new = histb_vdec_remove,
	.driver = {
		.name = "histb-vdec",
		.of_match_table = histb_vdec_of_match,
		.pm = &histb_vdec_pm_ops,
	},
};
module_platform_driver(histb_vdec_driver);

MODULE_AUTHOR("HiSilicon Technologies Co., Ltd.");
MODULE_DESCRIPTION("HiSilicon Hi3798CV200 VDH video decoder");
/* Build tag so a deployment can be proven to have taken effect. */
#define HISTB_VDEC_BUILD_TAG "dvbip-20260915-mvcprof"
MODULE_VERSION(HISTB_VDEC_BUILD_TAG);
/* dma_buf_export() lives in the DMA_BUF symbol namespace. */
MODULE_IMPORT_NS(DMA_BUF);
MODULE_FIRMWARE(HISTB_VDEC_H264_CABAC_FIRMWARE);
MODULE_FIRMWARE(HISTB_VDEC_HEVC_CABAC_FIRMWARE);
MODULE_FIRMWARE(HISTB_VDEC_AVSP_FIRMWARE);
MODULE_LICENSE("GPL");
