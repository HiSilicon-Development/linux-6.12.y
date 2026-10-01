/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 *
 * H.264 MVC (Multiview Video Coding) front-end for the Hi3798CV200 VDH.
 *
 * The layout, the NAL handling and the inter-view reference construction
 * follow the vendor VFMW MVC syntax layer (product/Hi3798CV200/mvc.h and
 * core/syntax/mvc.S).  The vendor HAL adds nothing to the H.264 picture
 * message, the slice message or the register set for MVC: one view is
 * decoded per picture, one hardware launch per view, both views share one
 * DPB, and the inter-view picture is appended to the ordinary L0/L1
 * reference lists with a reserved PMV store.  This front-end therefore only
 * produces view/reference state for the existing H.264 message builder; it
 * never builds a message of its own.
 */
#ifndef HISTB_VDEC_MVC_H
#define HISTB_VDEC_MVC_H

#include <linux/types.h>

/* product/Hi3798CV200/mvc.h:87-93 */
#define HISTB_MVC_MAX_DPB_LEN		33
#define HISTB_MVC_MAX_LIST_SIZE		33
#define HISTB_MVC_MAX_FRAME_STORE	40
#define HISTB_MVC_MAX_PMV_STORE		18
#define HISTB_MVC_MAX_SUBSPS		32
/*
 * mvc.h:93 "最大支持view数" -- the vendor syntax layer is two-view only and
 * MVC_ProcessSUBSPSMvcExt rejects num_views_minus1 > 1 (mvc.S:14308).
 */
#define HISTB_MVC_MAX_NUM_VIEWS		2
/* num_views_minus1 <= 1, so a view has at most one inter-view reference. */
#define HISTB_MVC_MAX_REFS		HISTB_MVC_MAX_NUM_VIEWS

/* mvc.h:60-61 */
#define HISTB_MVC_MAX_BYTES_START	(512 * 1024)
#define HISTB_MVC_GET_ONE_NALU_SIZE	(4 * 1024)

/* mvc.h:74-75 */
#define HISTB_MVC_PROFILE_MULTIVIEW_HIGH	118
#define HISTB_MVC_PROFILE_STEREO_HIGH		128

/* mvc.h:83-85 */
#define HISTB_MVC_SLICE_TYPE_P		0
#define HISTB_MVC_SLICE_TYPE_B		1
#define HISTB_MVC_SLICE_TYPE_I		2

/* mvc.h:95-113, restricted to the types the front-end classifies. */
#define HISTB_MVC_NAL_TYPE_SLICE	1
#define HISTB_MVC_NAL_TYPE_IDR		5
#define HISTB_MVC_NAL_TYPE_SEI		6
#define HISTB_MVC_NAL_TYPE_SPS		7
#define HISTB_MVC_NAL_TYPE_PPS		8
#define HISTB_MVC_NAL_TYPE_AUD		9
#define HISTB_MVC_NAL_TYPE_PREFIX	14
#define HISTB_MVC_NAL_TYPE_SUBSPS	15
#define HISTB_MVC_NAL_TYPE_AUXILIARY	19
#define HISTB_MVC_NAL_TYPE_SLICEEXT	20
#define HISTB_MVC_NAL_TYPE_EOPIC	30

/* mvc.h:112 and mvc.S:24986-25042: ASCII "HSPICEND", vendor private. */
#define HISTB_MVC_EOPIC_SIGNATURE	0x4853504943454e44ULL

#define HISTB_MVC_INVALID_VIEW_ID	(-1)
#define HISTB_MVC_INVALID_SLOT		(-1)

enum histb_mvc_result {
	HISTB_MVC_OK = 0,
	HISTB_MVC_INVALID,
	HISTB_MVC_NEED_MORE,
	HISTB_MVC_UNSUPPORTED,
};

/*
 * Picture structure.  The numbering is the vendor MVC_CURRPIC_S.structure
 * numbering (mvc.h:621), not the hardware PIC_STRUCTURE encoding.
 */
enum histb_mvc_structure {
	HISTB_MVC_STRUCTURE_FRAME = 0,
	HISTB_MVC_STRUCTURE_TOP,
	HISTB_MVC_STRUCTURE_BOTTOM,
};

/*
 * mvc.S:5236 indexes MVC_FRAMESTORE_S.inter_view_flag[2] with
 * (CurrPic.structure == 2); the same index selects anchor_pic_flag[2].  The
 * flag arrays are therefore *not* indexed by picture structure even though
 * the header comment says so, and this macro mirrors the code.
 */
#define HISTB_MVC_FLAG_INDEX(structure)	\
	((structure) == HISTB_MVC_STRUCTURE_BOTTOM ? 1 : 0)

/* The per-reference field selection the slice message needs for a frame. */
enum histb_mvc_fields {
	HISTB_MVC_FIELDS_FRAME = 0,
	HISTB_MVC_FIELDS_TOP,
	HISTB_MVC_FIELDS_BOTTOM,
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
	__u8 mvcinfo_flag;
	__u8 svc_extension_flag;
	__u8 nal_unit_type;
	__u8 interviewlist_xsize[2];
};

/* mvc.h:583-618, MVC_FRAMESTORE_S reduced to the fields the port can act on. */
struct histb_mvc_framestore {
	__u8 in_use;
	__u8 is_reference;
	__u8 is_used;
	__u8 svc_extension_flag;
	__u8 anchor_pic_flag[2];
	__u8 inter_view_flag[2];
	__u32 frame_num;
	__s32 poc;
	/*
	 * MVC_DecPOC (mvc.h:605): a second, never-reset POC used only for
	 * inter-view matching, because MMCO 5 resets the base view's ordinary
	 * POC and the inter-view reference would then be missed.
	 */
	__s32 decode_poc[3];
	__u32 pmv_address_idc;
	__s32 view_id;
	__u32 voidx;
	__u32 frame_store_id;
	__s32 apc_slot;
	/* Opaque struct histb_vdec_decoded_buffer *, owned by the core. */
	void *buffer;
};

/* One resolved inter-view reference appended to an ordinary L0/L1 list. */
struct histb_mvc_interview {
	struct histb_mvc_framestore *fs;
	void *buffer;
	__s32 apc_slot;
	__u32 pmv_address_idc;
	__s32 poc;
	__u8 fields;
	__u8 inter_view_flag;
};

/* The current picture as the core hands it to the front-end. */
struct histb_mvc_picture {
	void *buffer;
	__u32 frame_store_id;
	__u32 frame_num;
	__s32 view_id;
	__s32 poc;
	__s32 decode_poc;
	__u8 structure;
	__u8 is_reference;
	__u8 anchor_pic_flag;
	__u8 inter_view_flag;
	__u8 svc_extension_flag;
};

struct histb_mvc_state {
	/* Set once a type-14/15/20/30 NAL has been classified. */
	__u8 enabled;
	__u8 have_view;
	__u32 total_subsps;
	__u32 curr_subsps;
	struct histb_mvc_subsps subsps[HISTB_MVC_MAX_SUBSPS];
	/* mvc.h:752-847, MVC_CTX_S subset. */
	struct histb_mvc_nal_ext curr_nal;
	/*
	 * The last prefix NAL (type 14) MVC header.  The vendor handler
	 * (mvc.S:25165-25172) logs the NAL and discards it without touching
	 * CurrNalMvcInfo; the header is kept here only so a capture can show
	 * what was dropped.
	 */
	struct histb_mvc_nal_ext prefix;
	struct histb_mvc_view view;
	struct histb_mvc_framestore fs[HISTB_MVC_MAX_FRAME_STORE];
	__u32 dpb_size;
	__u32 total_pmv_num;
	struct histb_mvc_interview interview[2][HISTB_MVC_MAX_NUM_VIEWS];
	__u32 interview_size[2];
	__u8 interview_ready;
	__u32 prev_frame_num[HISTB_MVC_MAX_NUM_VIEWS];
	__u32 eopic_count;
	__u32 prefix_count;
	__u32 slice_ext_count;
	__u32 subsps_count;
};

void histb_mvc_reset(struct histb_mvc_state *mvc);
void histb_mvc_set_pmv_num(struct histb_mvc_state *mvc, __u32 pmv_num);

int histb_mvc_parse_subsps(struct histb_mvc_subsps *subsps,
			   const __u8 *rbsp, __u32 bytes);
int histb_mvc_parse_mvc_extension(struct histb_mvc_nal_ext *ext,
				  const __u8 *rbsp, __u32 bytes);
int histb_mvc_note_slice(struct histb_mvc_state *mvc, const __u8 *data,
			 __u32 bytes);

bool histb_mvc_stream_is_mvc(const struct histb_mvc_state *mvc);
const struct histb_mvc_view *histb_mvc_current_view(
					const struct histb_mvc_state *mvc);
const struct histb_mvc_subsps *histb_mvc_current_subsps(
					const struct histb_mvc_state *mvc);

int histb_mvc_store_picture(struct histb_mvc_state *mvc,
			    const struct histb_mvc_picture *picture);
void histb_mvc_release_picture(struct histb_mvc_state *mvc, void *buffer);
void histb_mvc_note_frame_num(struct histb_mvc_state *mvc, __u32 frame_num);

int histb_mvc_build_interview_list(struct histb_mvc_state *mvc,
				   __s32 this_poc, __u8 structure);
__u32 histb_mvc_interview_size(const struct histb_mvc_state *mvc,
			       unsigned int list);
const struct histb_mvc_interview *histb_mvc_interview(
				const struct histb_mvc_state *mvc,
				unsigned int list, unsigned int index);

/*
 * mvc.S:5052-5056, the one genuinely MVC-specific HAL behaviour: an MVC
 * access unit's clock-skip macroblock count is the H.264 count times two
 * (HAL64/vdm_hal.S:5018 selects .L639, which ends in "lsl w22, w22, 1").
 * The port has no clock-skip register and decodes one view per launch, so
 * the per-picture macroblock count stays un-doubled (MVC-SPEC Q7); this
 * helper exists so a clock-skip consumer can obtain the vendor value.
 */
static inline __u32 histb_mvc_clock_skip_macroblock_count(__u32 total_mbs)
{
	return total_mbs * 2;
}

#endif /* HISTB_VDEC_MVC_H */
