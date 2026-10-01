/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 */
#ifndef HISTB_VDEC_MPEG4_H
#define HISTB_VDEC_MPEG4_H

#include <linux/types.h>

#define HISTB_MPEG4_MATRIX_SIZE		64
#define HISTB_MPEG4_PIC_MSG_WORDS	64
#define HISTB_MPEG4_MAX_SLICES		512
#define HISTB_MPEG4_MAX_SLICE_MSGS	(HISTB_MPEG4_MAX_SLICES + 1)
#define HISTB_MPEG4_SLICE_FRAGMENTS	2
#define HISTB_MPEG4_SLICE_MSG_WORDS	8

enum histb_mpeg4_result {
	HISTB_MPEG4_OK = 0,
	HISTB_MPEG4_NEED_MORE = -1,
	HISTB_MPEG4_INVALID = -2,
	HISTB_MPEG4_UNSUPPORTED = -3,
	HISTB_MPEG4_NO_VOP = -4,
	HISTB_MPEG4_END_OF_STREAM = -5,
};

enum histb_mpeg4_picture_type {
	HISTB_MPEG4_I_VOP = 0,
	HISTB_MPEG4_P_VOP = 1,
	HISTB_MPEG4_B_VOP = 2,
	HISTB_MPEG4_S_VOP = 3,
	HISTB_MPEG4_N_VOP = 4,
};

/* The original VFMW contract uses 1 for short headers and 2 for MPEG-4. */
enum histb_mpeg4_header_type {
	HISTB_MPEG4_SHORT_HEADER = 1,
	HISTB_MPEG4_VOP_HEADER = 2,
};

enum histb_mpeg4_blocker {
	HISTB_MPEG4_BLOCK_NONE = 0,
	HISTB_MPEG4_BLOCK_CHROMA_FORMAT,
	HISTB_MPEG4_BLOCK_NON_RECTANGULAR,
	HISTB_MPEG4_BLOCK_OBMC,
	HISTB_MPEG4_BLOCK_SPRITE,
	HISTB_MPEG4_BLOCK_NON_8_BIT,
	HISTB_MPEG4_BLOCK_COMPLEXITY_ESTIMATION,
	HISTB_MPEG4_BLOCK_RESYNC_PACKETS,
	HISTB_MPEG4_BLOCK_DATA_PARTITIONING,
	HISTB_MPEG4_BLOCK_NEWPRED,
	HISTB_MPEG4_BLOCK_REDUCED_RESOLUTION,
	HISTB_MPEG4_BLOCK_SCALABILITY,
	HISTB_MPEG4_BLOCK_S_VOP,
	HISTB_MPEG4_BLOCK_N_VOP,
	HISTB_MPEG4_BLOCK_MULTIPLE_VOPS,
	HISTB_MPEG4_BLOCK_VENDOR_WORKAROUND,
};

struct histb_mpeg4_vol {
	__u8 profile_and_level;
	__u8 video_object_layer_verid;
	__u8 quant_type;
	__u8 interlaced;
	__u8 low_delay;
	__u8 vol_control_parameters;
	__u8 quarter_sample;
	__u8 resync_marker_disable;
	__u8 sprite_brightness_change;
	__u8 sprite_enable;
	__u8 sprite_warping_accuracy;
	__u8 sprite_warping_points;
	__u8 vop_time_increment_bits;
	__u8 fixed_vop_rate;
	__u16 width;
	__u16 height;
	__u16 vop_time_increment_resolution;
	__u16 fixed_vop_time_increment;
	__u8 intra_quant_matrix[HISTB_MPEG4_MATRIX_SIZE];
	__u8 nonintra_quant_matrix[HISTB_MPEG4_MATRIX_SIZE];
};

struct histb_mpeg4_vop {
	__u8 coding_type;
	__u8 coded;
	__u8 rounding_type;
	__u8 intra_dc_vlc_thr;
	__u8 top_field_first;
	__u8 alternate_vertical_scan;
	__u8 quant;
	__u8 fcode_forward;
	__u8 fcode_backward;
	__u32 time;
	__u32 time_bp;
	__u32 time_pp;
	__u32 payload_bit_offset;
	__u32 payload_bits;
	__s32 gmc_du[2];
	__s32 gmc_dv[2];
	__s32 gmc_uo;
	__s32 gmc_vo;
	__s32 gmc_uco;
	__s32 gmc_vco;
	__u8 gmc_points;
	__u8 divx_500_b413;
};

struct histb_mpeg4_parser {
	struct histb_mpeg4_vol vol;
	__u8 have_vol;
	__u8 have_non_b_time;
	__u8 blocker;
	__u8 divx_packed;
	__u16 divx_version;
	__u16 divx_build;
	__u32 last_non_b_time;
	__u32 last_time_base;
	__u32 time_base;
	__u32 time_pp;
};

struct histb_mpeg4_frame {
	struct histb_mpeg4_vol vol;
	struct histb_mpeg4_vop vop;
};

struct histb_mpeg4_hw_picture {
	struct histb_mpeg4_frame frame;
	__u8 bug_edge_extend;
	__u8 bug_qpel_chroma;
	__u8 bug_qpel_chroma2;
	__u8 compression;
	__u8 vdh_mmu;
	__u8 reserved[3];
	__u32 backward_pmv_addr;
	__u32 backward_ref_addr;
	__u32 current_picture_addr;
	__u32 current_pmv_addr;
	__u32 avm_addr;
	__u32 dnr_mbinfo_addr;
	__u32 display_picture_addr;
	__u32 forward_ref_addr;
	__u32 itrans_top_addr;
	__u32 pmv_top_addr;
	__u32 sed_top_addr;
	__u32 prcnum;
	__u32 slice_msg_addr;
	__u32 vam_addr;
	__u32 y_stride;
	__u32 uv_offset;
};

struct histb_mpeg4_regs {
	__u32 basic_cfg0;
	__u32 basic_cfg1;
	__u32 avm_addr;
	__u32 vam_addr;
	__u32 stream_base;
	__u32 current_picture_addr;
	__u32 y_stride;
	__u32 uv_offset;
	__u32 fixed_cfg;
	__u32 prcnum;
	__u32 dnr_mbinfo_addr;
	__u8 scd_emar;
	__u8 reserved[3];
};

struct histb_mpeg4_pic_msg {
	__u32 d[HISTB_MPEG4_PIC_MSG_WORDS];
};

struct histb_mpeg4_slice {
	__u8 bit_offset[HISTB_MPEG4_SLICE_FRAGMENTS];
	__u8 fcode_forward;
	__u8 fcode_backward;
	__u8 intra_dc_vlc_thr;
	__u8 coding_type;
	__u8 quant;
	__u16 reserved;
	__u32 bit_len[HISTB_MPEG4_SLICE_FRAGMENTS];
	__u32 dma_addr[HISTB_MPEG4_SLICE_FRAGMENTS];
	__u32 mb_start;
};

struct histb_mpeg4_slice_msg {
	__u32 d[HISTB_MPEG4_SLICE_MSG_WORDS];
};

struct histb_mpeg4_display_state {
	__u8 pending;
};

struct histb_mpeg4_display_plan {
	__u8 release_pending;
	__u8 release_current;
	__u8 hold_current;
	__u8 current_before_pending;
	__u8 pending_last;
	__u8 current_last;
};

void histb_mpeg4_parser_reset(struct histb_mpeg4_parser *parser);
int histb_mpeg4_parse_frame(struct histb_mpeg4_parser *parser,
				    const __u8 *data, __u32 bytes,
				    struct histb_mpeg4_frame *frame);
int histb_mpeg4_parse_frame_at(struct histb_mpeg4_parser *parser,
			       const __u8 *data, __u32 bytes, __u32 offset,
			       struct histb_mpeg4_frame *frame,
			       __u32 *next_offset);
int histb_mpeg4_validate_stateful_frame(struct histb_mpeg4_parser *parser,
					 const __u8 *data, __u32 bytes,
					 const struct histb_mpeg4_frame *frame);
int histb_mpeg4_build_pic_msg(const struct histb_mpeg4_hw_picture *picture,
			      struct histb_mpeg4_pic_msg *msg);
int histb_mpeg4_build_regs(const struct histb_mpeg4_hw_picture *picture,
			   const struct histb_mpeg4_slice *slices,
			   __u32 slice_count,
			   struct histb_mpeg4_regs *regs);
int histb_mpeg4_build_slice_msg(const struct histb_mpeg4_frame *frame,
					const struct histb_mpeg4_slice *slice,
					__u32 stream_base,
					struct histb_mpeg4_slice_msg *msg);
int histb_mpeg4_build_slice_messages(
				const struct histb_mpeg4_frame *frame,
				const struct histb_mpeg4_slice *slices,
				__u32 slice_count, __u32 stream_base,
				__u32 message_dma,
				struct histb_mpeg4_slice_msg *messages,
				__u32 message_capacity, __u32 *message_count);
int histb_mpeg4_make_single_slice(const struct histb_mpeg4_frame *frame,
					  __u32 buffer_dma,
					  struct histb_mpeg4_slice *slice);
int histb_mpeg4_parse_slices(const struct histb_mpeg4_frame *frame,
			     const __u8 *data, __u32 bytes, __u32 buffer_dma,
			     struct histb_mpeg4_slice *slices,
			     __u32 slice_capacity, __u32 *slice_count);
int histb_mpeg4_plan_display(struct histb_mpeg4_display_state *state,
			     __u8 coding_type, __u8 low_delay,
			     __u8 first_anchor, __u8 draining,
			     struct histb_mpeg4_display_plan *plan);
int histb_mpeg4_plan_drain(struct histb_mpeg4_display_state *state,
			   struct histb_mpeg4_display_plan *plan);

#endif
