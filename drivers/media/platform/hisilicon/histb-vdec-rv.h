/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 *
 * RealVideo 8 / RealVideo 9 syntax and HiVDHV4R3C1 message contract.
 *
 * Field names, offsets and widths follow the vendor VFMW contract as
 * extracted in Work/1001/bsp-gap/RV-SPEC.md:
 *   - picture message: 64 words / 256 bytes, RV-SPEC.md section 2.2 (RV8)
 *     and 2.3 (RV9)
 *   - slice message: one 256-byte slot per slice, RV-SPEC.md section 2.4
 *   - register set and format nibble: RV-SPEC.md sections 3.1 - 3.3
 *
 * The vendor sources for this HAL (vdm_hal_real8.c / real8.c) are not in the
 * BSP drop, so every value that RV-SPEC.md lists as an unknown is marked at
 * its use site with the question number from that document rather than
 * silently invented.
 */
#ifndef HISTB_VDEC_RV_H
#define HISTB_VDEC_RV_H

#include <linux/types.h>
#include <linux/videodev2.h>

/* CV200/real8.h:35-39, CV200/real9.h:34-37 (RV-SPEC.md section 2.1). */
#define HISTB_RV_PIC_MSG_WORDS		64
#define HISTB_RV_PIC_MSG_SIZE		(HISTB_RV_PIC_MSG_WORDS * sizeof(__u32))
#define HISTB_RV_SLICE_MSG_WORDS	64
#define HISTB_RV_SLICE_MSG_SIZE		(HISTB_RV_SLICE_MSG_WORDS * sizeof(__u32))
/* The HAL places the slice area at the picture message plus one slot. */
#define HISTB_RV_SLICE_MSG_OFFSET	HISTB_RV_PIC_MSG_SIZE
#define HISTB_RV_MAX_SLICES		256
#define HISTB_RV_MAX_SLICE_MSGS		(HISTB_RV_MAX_SLICES + 1)
#define HISTB_RV_SLICE_FRAGMENTS	2
/* CV200/real8.h:390-399: the vendor input format list is RV8_VIDEO_FORMAT. */
#define HISTB_RV_MAX_WIDTH		1920U
#define HISTB_RV_MAX_HEIGHT		1088U
#define HISTB_RV_MIN_RPR_SIZES		8

/* Upstream V4L2 fourccs (include/uapi/linux/videodev2.h:771-772): RV30 is
 * RealVideo 8, RV40 is RealVideo 9.  The numeric wire format is
 * VFMW_REAL8 / VFMW_REAL9 (RV-SPEC.md section 1.1) and is also the value
 * programmed into BASIC_CFG1 bits[3:0]. */
#define HISTB_RV_FMT_RV8		V4L2_PIX_FMT_RV30
#define HISTB_RV_FMT_RV9		V4L2_PIX_FMT_RV40

enum histb_rv_result {
	HISTB_RV_OK = 0,
	HISTB_RV_NEED_MORE = -1,
	HISTB_RV_INVALID = -2,
	HISTB_RV_UNSUPPORTED = -3,
	HISTB_RV_NO_PIC = -4,
	HISTB_RV_END_OF_STREAM = -5,
};

/* CV200/real8.h:120-126, CV200/real9.h:116-122. */
enum histb_rv_picture_type {
	HISTB_RV_INTRAPIC = 0,
	HISTB_RV_INTERPIC = 1,
	HISTB_RV_TRUEBPIC = 2,
	HISTB_RV_FRUPIC = 3,
};

/* VID_STD_E values 8 and 9 (RV-SPEC.md section 1.1). */
enum histb_rv_std {
	HISTB_RV_STD_NONE = 0,
	HISTB_RV_STD_REAL8 = 8,
	HISTB_RV_STD_REAL9 = 9,
};

enum histb_rv_blocker {
	HISTB_RV_BLOCK_NONE = 0,
	HISTB_RV_BLOCK_CODING_TYPE,
	HISTB_RV_BLOCK_GEOMETRY,
	HISTB_RV_BLOCK_NO_SLICE,
	HISTB_RV_BLOCK_RPR,
	HISTB_RV_BLOCK_FRAGMENTS,
	HISTB_RV_BLOCK_VENDOR_WORKAROUND,
};

/* RV8_PictureHeader (CV200/real8.h:128-156) with the parse outputs that
 * RV-SPEC.md section 5.1 assigns to the front-end. */
struct histb_rv_picture_header {
	__u8 linear_en;
	__u8 compress_en;
	__u8 fid;
	__u8 pic_coding_type;
	__u8 rounding;
	__u8 deblocking_filter_passthrough;
	__u8 pquant;
	__u8 osvquant;
	__u8 cp_fmt;
	__u8 rpr;
	__u8 pic_size_code;
	__u8 pic_size_code_hi;
	__u16 pic_width_in_pixel;
	__u16 pic_height_in_pixel;
	__u16 pic_width_in_mb;
	__u16 pic_height_in_mb;
	__u16 total_mbs;
	__u16 tr_wrap;
	__u16 tr;
	__u16 trb;
	__u16 dbquant;
	__u16 ratio0;
	__u16 ratio1;
	__u32 stream_base_addr;
	__u32 payload_bit_offset;
	__u32 payload_bits;
};

/* RV8_SliceHeader (CV200/real8.h:158-172). */
struct histb_rv_slice_header {
	__u8 slice_qp;
	__u8 osvquant;
	__u8 dblk_filter_passthrough;
	__u8 slice_type;
	__u8 is_last_seg;
	__u8 is_last_seg1;
	__u16 reserved;
	__u32 first_mb_in_slice;
	__u32 last_mb_in_slice;
	__u8 bit_offset[HISTB_RV_SLICE_FRAGMENTS];
	__u32 bit_len[HISTB_RV_SLICE_FRAGMENTS];
	__u32 bit_stream_addr[HISTB_RV_SLICE_FRAGMENTS];
};

/* RV8_CODECINF (CV200/real8.h:184-281): state that survives a picture. */
struct histb_rv_parser {
	__u8 std;
	__u8 blocker;
	__u8 have_prev_qp;
	__u8 prev_pic_qp;
	__u8 prev_pic_mb0_qp;
	__u8 pctsz;
	__u8 m_num_sizes;
	__u8 reserved;
	__u16 pwidth;
	__u16 pheight;
	__u16 pwidth_prev;
	__u16 pheight_prev;
	__u16 m_pctsz_size[HISTB_RV_MIN_RPR_SIZES];
};

/* One decoded picture's parsed state. */
struct histb_rv_frame {
	struct histb_rv_picture_header header;
	__u8 std;
	__u8 reserved[3];
};

/* The address inputs of RV8_DEC_PARAM_S (CV200/decparam.h:1163-1201) minus
 * the slice array, which the front-end emits separately. */
struct histb_rv_hw_picture {
	struct histb_rv_frame frame;
	__u8 std;
	__u8 compress_en;
	__u8 vdh_mmu_en;
	__u8 fst_slc_grp;
	__u8 linear_en;
	__u8 reserved[3];
	__u32 ddr_stride;
	__u32 uv_offset;
	__u32 head_info_size;
	__u32 cur_pic_phy_addr;
	__u32 disp_frame_phy_addr;
	__u32 fwd_ref_phy_addr;
	__u32 bwd_ref_phy_addr;
	__u32 curr_pmv_phy_addr;
	__u32 col_pmv_phy_addr;
	__u32 sed_top_addr;
	__u32 pmv_top_addr;
	__u32 rcn_top_addr;
	__u32 dblk_top_addr;
	__u32 dnr_mbinfo_addr;
	__u32 pic_msg_addr;
	__u32 slice_msg_addr;
	__u32 vam_addr;
	__u32 avm_addr;
	__u32 stream_base_addr;
	__u32 rpr_num_sizes;
	__u32 rpr_sizes[HISTB_RV_MIN_RPR_SIZES];
};

/* RV8HAL_V4R3C1_WriteReg (RV-SPEC.md section 3.3). */
struct histb_rv_regs {
	__u32 basic_cfg0;
	__u32 basic_cfg1;
	__u32 avm_addr;
	__u32 vam_addr;
	__u32 stream_base;
	__u32 current_picture_addr;
	__u32 y_stride;
	__u32 uv_offset;
	__u32 head_info_size;
	__u32 fixed_cfg;
	__u32 dnr_mbinfo_addr;
	__u32 int_state_reset;
	__u8 scd_emar;
	__u8 reserved[3];
};

/* RV8HAL_V4R3C1_WritePicMsg, words 0-6 / 16-24 / 63 (RV-SPEC.md 2.2/2.3). */
struct histb_rv_pic_msg {
	__u32 d[HISTB_RV_PIC_MSG_WORDS];
};

/* One RV8_SLC_PARAM_S element (CV200/decparam.h:1150-1161). */
struct histb_rv_slice {
	__u8 bit_offset[HISTB_RV_SLICE_FRAGMENTS];
	__u8 dblk_filter_passthrough;
	__u8 osvquant;
	__u8 sliceqp;
	__u8 reserved;
	__u32 bit_len[HISTB_RV_SLICE_FRAGMENTS];
	__u32 dma_addr[HISTB_RV_SLICE_FRAGMENTS];
	__u32 first_mb_in_slice;
	__u32 last_mb_in_slice;
};

/* One 256-byte slice slot; words 6-62 are reserved and stay zero. */
struct histb_rv_slice_msg {
	__u32 d[HISTB_RV_SLICE_MSG_WORDS];
};

/* Display hand-off.  RV-SPEC.md Q5 leaves the vendor's FSP_SetDisplay policy
 * unresolved; the port reuses the pending/current model of the MPEG-4
 * front-end, which drives the same TR/TRB B-picture timing. */
struct histb_rv_display_state {
	__u8 pending;
};

struct histb_rv_display_plan {
	__u8 release_pending;
	__u8 release_current;
	__u8 hold_current;
	__u8 current_before_pending;
	__u8 pending_last;
	__u8 current_last;
};

void histb_rv_parser_reset(struct histb_rv_parser *parser, __u8 std);
int histb_rv_parse_frame(struct histb_rv_parser *parser, const __u8 *data,
			 __u32 bytes, struct histb_rv_frame *frame);
int histb_rv_parse_frame_at(struct histb_rv_parser *parser, const __u8 *data,
			    __u32 bytes, __u32 offset,
			    struct histb_rv_frame *frame,
			    __u32 *next_offset);
int histb_rv_validate_stateful_frame(struct histb_rv_parser *parser,
				     const __u8 *data, __u32 bytes,
				     const struct histb_rv_frame *frame);
int histb_rv_build_pic_msg(const struct histb_rv_hw_picture *picture,
			   struct histb_rv_pic_msg *msg);
int histb_rv_build_regs(const struct histb_rv_hw_picture *picture,
			const struct histb_rv_slice *slices, __u32 slice_count,
			struct histb_rv_regs *regs);
int histb_rv_parse_slices(const struct histb_rv_frame *frame,
			  const __u8 *data, __u32 bytes, __u32 buffer_dma,
			  struct histb_rv_slice *slices, __u32 slice_capacity,
			  __u32 *slice_count);
int histb_rv_make_single_slice(const struct histb_rv_frame *frame,
			       __u32 buffer_dma,
			       struct histb_rv_slice *slice);
int histb_rv_build_slice_msg(const struct histb_rv_frame *frame,
			     const struct histb_rv_slice *slice,
			     struct histb_rv_slice_msg *msg);
int histb_rv_build_slice_messages(const struct histb_rv_frame *frame,
				  const struct histb_rv_slice *slices,
				  __u32 slice_count, __u32 message_dma,
				  struct histb_rv_slice_msg *messages,
				  __u32 message_capacity,
				  __u32 *message_count);
int histb_rv_plan_display(struct histb_rv_display_state *state,
			  __u8 coding_type, __u8 first_anchor, __u8 draining,
			  struct histb_rv_display_plan *plan);

#endif /* HISTB_VDEC_RV_H */
