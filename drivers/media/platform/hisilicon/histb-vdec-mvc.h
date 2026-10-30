/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 *
 * H.264 MVC syntax shared by the Hi3798MV310 request ingress.
 * View/reference lifetime belongs to histb-vdec-mvc-ingress, not a separate DPB.
 */
#ifndef HISTB_VDEC_MVC_H
#define HISTB_VDEC_MVC_H

#include <linux/types.h>

/* product/Hi3798CV200/mvc.h:87-93 */
#define HISTB_MVC_MAX_FRAME_STORE	40
/*
 * mvc.h:93 "最大支持view数" -- the vendor syntax layer is two-view only and
 * MVC_ProcessSUBSPSMvcExt rejects num_views_minus1 > 1 (mvc.S:14308).
 */
#define HISTB_MVC_MAX_NUM_VIEWS		2
/* num_views_minus1 <= 1, so a view has at most one inter-view reference. */
#define HISTB_MVC_MAX_REFS		HISTB_MVC_MAX_NUM_VIEWS

/* mvc.h:60-61 */
#define HISTB_MVC_MAX_BYTES_START	(512 * 1024)

/* mvc.h:74-75 */
#define HISTB_MVC_PROFILE_MULTIVIEW_HIGH	118
#define HISTB_MVC_PROFILE_STEREO_HIGH		128

/* mvc.h:95-113, restricted to the types the front-end classifies. */
#define HISTB_MVC_NAL_TYPE_SLICE	1
#define HISTB_MVC_NAL_TYPE_IDR		5
#define HISTB_MVC_NAL_TYPE_SUBSPS	15
#define HISTB_MVC_NAL_TYPE_SLICEEXT	20

enum histb_mvc_result {
	HISTB_MVC_OK = 0,
	HISTB_MVC_INVALID,
	HISTB_MVC_NEED_MORE,
	HISTB_MVC_UNSUPPORTED,
};


/* mvc.h:187-198, MVC_NALUMVCEXT_S. */
struct histb_mvc_nal_ext {
	__u8 is_valid;
	__u8 non_idr_flag;
	__u8 priority_id;
	__u8 temporal_id;
	__u8 anchor_pic_flag;
	__u8 inter_view_flag;
	__u8 reserved_one_bit;
	__s32 view_id;
};

/*
 * The subset of the ordinary sequence parameter set this front-end keeps.
 * A subset SPS embeds a complete ordinary SPS (mvc.h:358), so a dependent
 * view's geometry comes from here and not from a separate type-7 SPS.
 */
struct histb_mvc_sps {
	__u8 profile_idc;
	__u8 level_idc;
	__u8 chroma_format_idc;
	__u8 frame_mbs_only_flag;
	__u8 mb_adaptive_frame_field_flag;
	__u8 delta_pic_order_always_zero_flag;
	__u8 seq_parameter_set_id;
	__u8 pic_order_cnt_type;
	__u8 log2_max_frame_num_minus4;
	__u8 max_num_ref_frames;
	__u32 pic_width_in_mbs_minus1;
	__u32 pic_height_in_map_units_minus1;
	__u32 frame_crop_left_offset;
	__u32 frame_crop_right_offset;
	__u32 frame_crop_top_offset;
	__u32 frame_crop_bottom_offset;
};

/* mvc.h:339-361, MVC_SUBSPS_S without the 328 KiB MVC VUI table. */
struct histb_mvc_subsps {
	__u8 is_valid;
	__u8 bit_equal_to_one;
	__u32 num_views_minus1;
	__s32 view_id[HISTB_MVC_MAX_NUM_VIEWS];
	__u32 num_anchor_refs[2][HISTB_MVC_MAX_NUM_VIEWS];
	__u32 anchor_ref[2][HISTB_MVC_MAX_NUM_VIEWS][HISTB_MVC_MAX_REFS];
	__u32 num_non_anchor_refs[2][HISTB_MVC_MAX_NUM_VIEWS];
	__u32 non_anchor_ref[2][HISTB_MVC_MAX_NUM_VIEWS][HISTB_MVC_MAX_REFS];
	struct histb_mvc_sps sps;
};

/*
 * MVC_SLICE_S view fields (mvc.h:413-464 tail): the only place in the vendor
 * decode chain that records which view a slice belongs to.
 */
struct histb_mvc_view {
	__s32 view_id;
	__u32 voidx;
	__u8 anchor_pic_flag;
	__u8 inter_view_flag;
	__u8 nal_unit_type;
};

int histb_mvc_parse_subsps(struct histb_mvc_subsps *subsps,
			   const __u8 *rbsp, __u32 bytes);
int histb_mvc_parse_mvc_extension(struct histb_mvc_nal_ext *ext,
				  const __u8 *rbsp, __u32 bytes);

#endif /* HISTB_VDEC_MVC_H */
