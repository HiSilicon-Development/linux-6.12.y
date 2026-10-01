// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 *
 * H.264 MVC front-end for the Hi3798CV200 VDH decoder.
 *
 * The vendor HAL adds nothing to the H.264 picture message, slice message or
 * register set for MVC (MVC-SPEC.md section 3.1).  The two views are bound
 * only through the reference lists: the inter-view picture is an ordinary
 * frame store in the shared DPB, appended to the current picture's L0/L1 by
 * MVC_GenPiclistfromFrmlist_Interview with a reserved PMV store, and nothing
 * downstream knows it is a second view.
 *
 * So this file produces view and reference state only.  It never builds a
 * message; the existing H.264 message builder consumes what is here.
 *
 * Every algorithm below cites the spec section it comes from.  Where the
 * spec records an unknown, that is marked at the site rather than papered
 * over with a plausible value.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "histb-vdec-mvc.h"

/*
 * ---- RBSP bit reader -------------------------------------------------
 *
 * H.264 Annex B emulation prevention is the core's business - it hands this
 * front-end an RBSP.  The subset SPS and the MVC extension are both
 * Exp-Golomb coded, so a plain MSB-first reader is all that is needed.
 */
struct histb_mvc_bits {
	const __u8 *data;
	__u32 size_bits;
	__u32 pos;
	bool overrun;
};

static void bits_init(struct histb_mvc_bits *br, const __u8 *data, __u32 bytes)
{
	br->data = data;
	br->size_bits = bytes * 8;
	br->pos = 0;
	br->overrun = false;
}

static __u32 bits_get(struct histb_mvc_bits *br, unsigned int count)
{
	__u32 value = 0;
	unsigned int i;

	if (br->overrun || br->pos + count > br->size_bits) {
		br->overrun = true;
		return 0;
	}

	for (i = 0; i < count; i++) {
		value = (value << 1) |
			((br->data[br->pos >> 3] >> (7 - (br->pos & 7))) & 1);
		br->pos++;
	}

	return value;
}

static bool bits_flag(struct histb_mvc_bits *br)
{
	return bits_get(br, 1) != 0;
}

/* ue(v): H.264 9.1. Exp-Golomb, leading zero count then that many bits. */
static __u32 bits_ue(struct histb_mvc_bits *br)
{
	unsigned int leading_zeros = 0;
	__u32 value;

	while (!br->overrun && leading_zeros < 32 && bits_get(br, 1) == 0)
		leading_zeros++;

	if (br->overrun || leading_zeros >= 32) {
		br->overrun = true;
		return 0;
	}

	if (leading_zeros == 0)
		return 0;

	value = bits_get(br, leading_zeros);
	if (br->overrun)
		return 0;

	return (1u << leading_zeros) - 1 + value;
}

/* se(v): H.264 9.1.1. */
static __s32 bits_se(struct histb_mvc_bits *br)
{
	__u32 code = bits_ue(br);
	__s32 value = (__s32)((code + 1) >> 1);

	return (code & 1) ? value : -value;
}

/*
 * ---- NAL classification ----------------------------------------------
 *
 * mvc.S:24893 branches on nal_unit_type == 20 before its jump table, so the
 * set of types this front-end has to recognise is small and explicit.
 */
static bool mvc_nal_type_is_mvc(__u32 type)
{
	switch (type) {
	case HISTB_MVC_NAL_TYPE_PREFIX:
	case HISTB_MVC_NAL_TYPE_SUBSPS:
	case HISTB_MVC_NAL_TYPE_SLICEEXT:
	case HISTB_MVC_NAL_TYPE_EOPIC:
		return true;
	default:
		return false;
	}
}

void histb_mvc_reset(struct histb_mvc_state *mvc)
{
	if (!mvc)
		return;

	memset(mvc, 0, sizeof(*mvc));
	mvc->curr_nal.view_id = HISTB_MVC_INVALID_VIEW_ID;
	mvc->view.view_id = HISTB_MVC_INVALID_VIEW_ID;
	mvc->view.interviewlist_xsize[0] = 0;
	mvc->view.interviewlist_xsize[1] = 0;
}

void histb_mvc_set_pmv_num(struct histb_mvc_state *mvc, __u32 pmv_num)
{
	if (mvc)
		mvc->total_pmv_num = pmv_num;
}

/*
 * ---- The MVC NAL extension header (24 bits) --------------------------
 *
 * mvc.S:25273-25299, reproduced in MVC-SPEC.md section 5.3.  The bit
 * assignment is H.264 Annex G's nal_unit_header_mvc_extension():
 * MSB-first svc_extension_flag, non_idr_flag, priority_id[6], view_id[10],
 * temporal_id[3], anchor_pic_flag, inter_view_flag, reserved_one_bit.
 */
int histb_mvc_parse_mvc_extension(struct histb_mvc_nal_ext *ext,
				  const __u8 *rbsp, __u32 bytes)
{
	struct histb_mvc_bits br;
	unsigned int svc_extension_flag;
	u8 bits[3];

	if (!ext || !rbsp)
		return HISTB_MVC_INVALID;

	/* The header is three bytes; anything shorter cannot contain it. */
	if (bytes < sizeof(bits))
		return HISTB_MVC_NEED_MORE;

	bits[0] = rbsp[0];
	bits[1] = rbsp[1];
	bits[2] = rbsp[2];
	memset(&br, 0, sizeof(br));
	bits_init(&br, bits, sizeof(bits));

	svc_extension_flag = bits_flag(&br) ? 1 : 0;
	ext->non_idr_flag = bits_flag(&br) ? 1 : 0;
	ext->priority_id = bits_get(&br, 6);
	ext->view_id = bits_get(&br, 10);
	ext->temporal_id = bits_get(&br, 3);
	ext->anchor_pic_flag = bits_flag(&br) ? 1 : 0;
	ext->inter_view_flag = bits_flag(&br) ? 1 : 0;
	ext->reserved_one_bit = bits_flag(&br) ? 1 : 0;

	if (br.overrun)
		return HISTB_MVC_INVALID;

	/*
	 * mvc.S:25279: svc_extension_flag != 0 means SVC, not MVC, and the
	 * vendor falls through to the ordinary H.264 slice path.  Report it as
	 * unsupported rather than pretending it is a view.
	 */
	if (svc_extension_flag)
		return HISTB_MVC_UNSUPPORTED;

	ext->is_valid = 1;

	return HISTB_MVC_OK;
}

/*
 * ---- Subset SPS (NAL type 15) ----------------------------------------
 *
 * MVC-SPEC.md section 5.2: MVC_DecSubSPS runs the ordinary
 * sequence_parameter_set_data() body and then, for an MVC profile, the
 * extension parsed here.  A subset SPS embeds a complete ordinary SPS
 * (mvc.h:358), so the dependent view's geometry comes from this structure
 * and not from a separate type-7 SPS.
 */
static int mvc_parse_ordinary_sps(struct histb_mvc_sps *sps,
				  struct histb_mvc_bits *br)
{
	u32 chroma_format_idc = 1;

	sps->profile_idc = bits_get(br, 8);
	bits_get(br, 8);			/* constraint set flags */
	sps->level_idc = bits_get(br, 8);
	sps->seq_parameter_set_id = bits_ue(br);

	if (br->overrun)
		return HISTB_MVC_INVALID;

	/* High profiles carry the chroma and scaling-list block. */
	switch (sps->profile_idc) {
	case 100: case 110: case 122: case 244: case 44:
	case 83: case 86: case 118: case 128: case 138: case 139: case 134:
		chroma_format_idc = bits_ue(br);
		if (chroma_format_idc == 3)
			bits_flag(br);		/* separate_colour_plane_flag */
		(void)bits_ue(br);		/* bit_depth_luma_minus8 */
		(void)bits_ue(br);		/* bit_depth_chroma_minus8 */
		bits_flag(br);			/* qpprime_y_zero_transform_bypass */
		if (bits_flag(br)) {		/* seq_scaling_matrix_present */
			unsigned int i, n = (chroma_format_idc == 3) ? 12 : 8;

			for (i = 0; i < n; i++) {
				if (!bits_flag(br))
					continue;
				/* scaling list: sized by the list index */
				if (i < 6) {
					u32 size = 16, last = 8, next = 8, j;

					for (j = 0; j < size && !br->overrun; j++) {
						if (next)
							next = (last + bits_se(br) + 256) % 256;
						if (j == size - 1)
							break;
						last = next ? next : last;
					}
				} else {
					u32 size = 64, last = 8, next = 8, j;

					for (j = 0; j < size && !br->overrun; j++) {
						if (next)
							next = (last + bits_se(br) + 256) % 256;
						last = next ? next : last;
					}
				}
			}
		}
		break;
	default:
		break;
	}

	sps->chroma_format_idc = chroma_format_idc;

	sps->log2_max_frame_num_minus4 = (u8)bits_ue(br);
	sps->pic_order_cnt_type = bits_ue(br);
	if (sps->pic_order_cnt_type == 0) {
		bits_ue(br);			/* log2_max_pic_order_cnt_lsb */
	} else if (sps->pic_order_cnt_type == 1) {
		sps->delta_pic_order_always_zero_flag = bits_flag(br) ? 1 : 0;
		bits_se(br);			/* offset_for_non_ref_pic */
		bits_se(br);			/* offset_for_top_to_bottom_field */
		{
			u32 i, cycles = bits_ue(br);

			for (i = 0; i < cycles && !br->overrun; i++)
				bits_se(br);
		}
	}

	sps->max_num_ref_frames = bits_ue(br);
	(void)bits_flag(br);			/* gaps_in_frame_num_value_allowed */

	sps->pic_width_in_mbs_minus1 = bits_ue(br);
	sps->pic_height_in_map_units_minus1 = bits_ue(br);
	sps->frame_mbs_only_flag = bits_flag(br) ? 1 : 0;
	if (!sps->frame_mbs_only_flag)
		sps->mb_adaptive_frame_field_flag = bits_flag(br) ? 1 : 0;
	bits_flag(br);				/* direct_8x8_inference_flag */

	if (bits_flag(br)) {			/* frame_cropping_flag */
		sps->frame_crop_left_offset = bits_ue(br);
		sps->frame_crop_right_offset = bits_ue(br);
		sps->frame_crop_top_offset = bits_ue(br);
		sps->frame_crop_bottom_offset = bits_ue(br);
	}

	/* vui_parameters_present_flag and anything after it is not needed. */

	return br->overrun ? HISTB_MVC_INVALID : HISTB_MVC_OK;
}

int histb_mvc_parse_subsps(struct histb_mvc_subsps *subsps,
			   const __u8 *rbsp, __u32 bytes)
{
	struct histb_mvc_bits br;
	int ret;

	if (!subsps || !rbsp)
		return HISTB_MVC_INVALID;

	memset(subsps, 0, sizeof(*subsps));
	bits_init(&br, rbsp, bytes);

	ret = mvc_parse_ordinary_sps(&subsps->sps, &br);
	if (ret != HISTB_MVC_OK)
		return ret;

	/*
	 * Only a multiview or stereo profile carries the extension; anything
	 * else is an ordinary SPS that happens to be in a type-15 NAL.
	 */
	if (subsps->sps.profile_idc != HISTB_MVC_PROFILE_MULTIVIEW_HIGH &&
	    subsps->sps.profile_idc != HISTB_MVC_PROFILE_STEREO_HIGH)
		return HISTB_MVC_UNSUPPORTED;

	/*
	 * MVC_ProcessSUBSPSMvcExt, mvc.S:14288-14753, bit order per
	 * MVC-SPEC.md section 5.2.
	 */
	subsps->bit_equal_to_one = bits_flag(&br) ? 1 : 0;
	subsps->num_views_minus1 = bits_ue(&br);

	/* mvc.S:14308 rejects more than two views, matching MAX_NUM_VIEWS. */
	if (br.overrun || subsps->num_views_minus1 >= HISTB_MVC_MAX_NUM_VIEWS)
		return HISTB_MVC_UNSUPPORTED;

	{
		u32 i, v, list;

		for (i = 0; i <= subsps->num_views_minus1; i++)
			subsps->view_id[i] = (__s32)bits_ue(&br);

		/*
		 * mvc.S:14324-14325 zeroes the counts for view 0 before the
		 * per-view loop, so only views 1..num_views_minus1 carry
		 * references - a view cannot reference itself.
		 */
		for (list = 0; list < 2; list++) {
			subsps->num_anchor_refs[list][0] = 0;
			subsps->num_non_anchor_refs[list][0] = 0;
		}

		for (v = 1; v <= subsps->num_views_minus1; v++) {
			for (list = 0; list < 2; list++) {
				u32 count = bits_ue(&br);
				u32 j;

				if (count > HISTB_MVC_MAX_REFS)
					return HISTB_MVC_UNSUPPORTED;
				subsps->num_anchor_refs[list][v] = count;
				for (j = 0; j < count; j++)
					subsps->anchor_ref[list][v][j] =
						bits_ue(&br);
			}
			for (list = 0; list < 2; list++) {
				u32 count = bits_ue(&br);
				u32 j;

				if (count > HISTB_MVC_MAX_REFS)
					return HISTB_MVC_UNSUPPORTED;
				subsps->num_non_anchor_refs[list][v] = count;
				for (j = 0; j < count; j++)
					subsps->non_anchor_ref[list][v][j] =
						bits_ue(&br);
			}
		}
	}

	if (br.overrun)
		return HISTB_MVC_INVALID;

	/*
	 * num_level_values_signalled_minus1 and the level tables
	 * (mvc.S:14700-14753) are not consumed: nothing in the port's message
	 * path reads them.  The reader stops here deliberately.
	 */
	subsps->is_valid = 1;

	return HISTB_MVC_OK;
}

/*
 * ---- Slice classification and per-slice view state -------------------
 *
 * MVC-SPEC.md section 5.3: a type-20 NAL's 24-bit extension header is
 * consumed, nal_unit_type is rewritten to IDR or non-IDR, and the view
 * fields are taken from the header.  A base-view slice (any other type)
 * takes its view id from the subset SPS instead, per section 5.4 - which is
 * why the prefix NAL can be discarded.
 */
int histb_mvc_note_slice(struct histb_mvc_state *mvc, const __u8 *data,
			 __u32 bytes)
{
	__u32 nal_type, nal_ref_idc;

	if (!mvc || !data || bytes < 1)
		return HISTB_MVC_INVALID;

	nal_type = data[0] & 0x1f;
	nal_ref_idc = (data[0] >> 5) & 0x3;

	mvc->view.nal_unit_type = nal_type;
	mvc->view.interviewlist_xsize[0] = 0;
	mvc->view.interviewlist_xsize[1] = 0;

	if (mvc_nal_type_is_mvc(nal_type))
		mvc->enabled = 1;

	switch (nal_type) {
	case HISTB_MVC_NAL_TYPE_SLICEEXT: {
		struct histb_mvc_nal_ext ext;
		int ret;

		ret = histb_mvc_parse_mvc_extension(&ext, data + 1, bytes - 1);
		if (ret != HISTB_MVC_OK)
			return ret;

		mvc->curr_nal = ext;
		mvc->view.mvcinfo_flag = 1;
		mvc->view.view_id = ext.view_id;
		mvc->view.anchor_pic_flag = ext.anchor_pic_flag;
		mvc->view.inter_view_flag = ext.inter_view_flag;
		/* A type-20 NAL with the SVC bit set was already rejected. */
		mvc->view.svc_extension_flag = 0;
		/*
		 * mvc.S:25419-25422: the type is rewritten to IDR when
		 * non_idr_flag is clear, otherwise to an ordinary slice.
		 */
		mvc->view.nal_unit_type = ext.non_idr_flag ?
			HISTB_MVC_NAL_TYPE_SLICE : HISTB_MVC_NAL_TYPE_IDR;
		mvc->slice_ext_count++;

		/* consumed, per mvc.S:25416 */
		mvc->curr_nal.is_valid = 0;
		break;
	}
	case HISTB_MVC_NAL_TYPE_PREFIX: {
		struct histb_mvc_nal_ext ext;
		int ret;

		/*
		 * mvc.S:25165-25172 recognises the prefix NAL and discards it
		 * without touching CurrNalMvcInfo.  Kept for capture only.
		 */
		ret = histb_mvc_parse_mvc_extension(&ext, data + 1, bytes - 1);
		if (ret == HISTB_MVC_OK)
			mvc->prefix = ext;
		mvc->prefix_count++;
		mvc->curr_nal.view_id = HISTB_MVC_INVALID_VIEW_ID;
		break;
	}
	case HISTB_MVC_NAL_TYPE_EOPIC:
		mvc->eopic_count++;
		return HISTB_MVC_OK;
	case HISTB_MVC_NAL_TYPE_SUBSPS:
		mvc->subsps_count++;
		return HISTB_MVC_OK;
	case HISTB_MVC_NAL_TYPE_SLICE:
	case HISTB_MVC_NAL_TYPE_IDR: {
		/*
		 * Base view.  mvc.S:25372-25407/.L4498: view_id comes from
		 * the first occupied subset SPS slot, or stays -1 when there
		 * is none.  Annex G then makes the flags implicit, which is
		 * why discarding the prefix NAL is correct.
		 */
		__s32 view_id = HISTB_MVC_INVALID_VIEW_ID;

		if (mvc->total_subsps && mvc->curr_subsps < mvc->total_subsps &&
		    mvc->subsps[mvc->curr_subsps].is_valid)
			view_id = mvc->subsps[mvc->curr_subsps].view_id[0];

		mvc->view.view_id = view_id;
		if (view_id != HISTB_MVC_INVALID_VIEW_ID) {
			mvc->view.mvcinfo_flag = 1;
			mvc->view.inter_view_flag = 1;
			mvc->view.anchor_pic_flag =
				(nal_type == HISTB_MVC_NAL_TYPE_IDR) ? 1 : 0;
			mvc->view.svc_extension_flag = 0;
			mvc->have_view = 1;
		}
		break;
	}
	default:
		break;
	}

	(void)nal_ref_idc;

	return HISTB_MVC_OK;
}

bool histb_mvc_stream_is_mvc(const struct histb_mvc_state *mvc)
{
	return mvc && mvc->enabled && mvc->have_view;
}

const struct histb_mvc_view *histb_mvc_current_view(
					const struct histb_mvc_state *mvc)
{
	if (!mvc || !mvc->have_view)
		return NULL;

	return &mvc->view;
}

const struct histb_mvc_subsps *histb_mvc_current_subsps(
					const struct histb_mvc_state *mvc)
{
	if (!mvc || !mvc->total_subsps || mvc->curr_subsps >= mvc->total_subsps)
		return NULL;

	return &mvc->subsps[mvc->curr_subsps];
}

/*
 * ---- The DPB ---------------------------------------------------------
 *
 * One frame store per decoded picture; both views share it
 * (MVC-SPEC.md section 4.1).
 */
int histb_mvc_store_picture(struct histb_mvc_state *mvc,
			    const struct histb_mvc_picture *picture)
{
	struct histb_mvc_framestore *fs;
	unsigned int i, index;
	int idx = HISTB_MVC_INVALID_SLOT;

	if (!mvc || !picture)
		return HISTB_MVC_INVALID;

	/*
	 * Reuse a slot already holding this buffer - the core may hand the
	 * same buffer back for the second view of a pair.
	 */
	for (i = 0; i < HISTB_MVC_MAX_FRAME_STORE; i++) {
		if (mvc->fs[i].in_use && mvc->fs[i].buffer == picture->buffer) {
			idx = i;
			break;
		}
	}

	if (idx < 0) {
		for (i = 0; i < HISTB_MVC_MAX_FRAME_STORE; i++) {
			if (!mvc->fs[i].in_use) {
				idx = i;
				break;
			}
		}
	}

	if (idx < 0)
		return HISTB_MVC_UNSUPPORTED;	/* DPB full */

	fs = &mvc->fs[idx];
	memset(fs, 0, sizeof(*fs));

	fs->in_use = 1;
	fs->is_reference = picture->is_reference;
	fs->svc_extension_flag = picture->svc_extension_flag;
	fs->frame_num = picture->frame_num;
	fs->poc = picture->poc;
	fs->view_id = picture->view_id;
	fs->buffer = picture->buffer;
	fs->frame_store_id = picture->frame_store_id;
	fs->apc_slot = HISTB_MVC_INVALID_SLOT;

	/*
	 * mvc.S:5236 indexes inter_view_flag[2] and anchor_pic_flag[2] by
	 * (structure == BOTTOM), not by a per-structure slot.
	 */
	index = HISTB_MVC_FLAG_INDEX(picture->structure);
	fs->inter_view_flag[index] = picture->inter_view_flag;
	fs->anchor_pic_flag[index] = picture->anchor_pic_flag;

	/*
	 * decode_poc is indexed by picture structure (mvc.S:5223/5307), and
	 * the frame case uses its own slot.  A single view per launch means
	 * one struct is enough here; the array exists to match the vendor
	 * layout.
	 */
	fs->decode_poc[picture->structure > HISTB_MVC_STRUCTURE_BOTTOM ?
		       HISTB_MVC_STRUCTURE_FRAME : picture->structure] =
		picture->decode_poc;

	if (mvc->dpb_size < HISTB_MVC_MAX_FRAME_STORE)
		mvc->dpb_size++;

	return (int)idx;
}

void histb_mvc_release_picture(struct histb_mvc_state *mvc, void *buffer)
{
	unsigned int i;

	if (!mvc || !buffer)
		return;

	for (i = 0; i < HISTB_MVC_MAX_FRAME_STORE; i++) {
		if (mvc->fs[i].in_use && mvc->fs[i].buffer == buffer) {
			memset(&mvc->fs[i], 0, sizeof(mvc->fs[i]));
			if (mvc->dpb_size)
				mvc->dpb_size--;
		}
	}
}

void histb_mvc_note_frame_num(struct histb_mvc_state *mvc, __u32 frame_num)
{
	unsigned int view = 0;

	if (!mvc)
		return;

	if (mvc->view.view_id >= 0 && mvc->total_subsps) {
		const struct histb_mvc_subsps *s = histb_mvc_current_subsps(mvc);
		unsigned int i;

		if (s) {
			for (i = 0; i <= s->num_views_minus1 &&
				    i < HISTB_MVC_MAX_NUM_VIEWS; i++) {
				if (s->view_id[i] == mvc->view.view_id) {
					view = i;
					break;
				}
			}
		}
	}

	if (view < HISTB_MVC_MAX_NUM_VIEWS)
		mvc->prev_frame_num[view] = frame_num;
}

/*
 * ---- The inter-view reference list -----------------------------------
 *
 * mvc_append_interview_list, mvc.S:5137-5327, as analysed in MVC-SPEC.md
 * section 4.2.  Steps 1-5 of that analysis are implemented in order below.
 */
static int mvc_view_order_index(const struct histb_mvc_state *mvc,
				const struct histb_mvc_subsps *s,
				__s32 view_id)
{
	unsigned int i;

	/* mvc.S:5158-5176, MVC_GetVOIdx */
	if (s->num_views_minus1 >= 1 && s->view_id[0] == view_id)
		return 0;

	for (i = 1; i <= s->num_views_minus1 && i < HISTB_MVC_MAX_NUM_VIEWS;
	     i++) {
		if (s->view_id[i] == view_id)
			return (int)i;
	}

	/* Not a view this subset SPS knows about. */
	return -1;
}

int histb_mvc_build_interview_list(struct histb_mvc_state *mvc,
				   __s32 this_poc, __u8 structure)
{
	const struct histb_mvc_subsps *s;
	unsigned int list, count;
	unsigned int emitted = 0;
	unsigned int flag_index;
	int voIdx;

	if (!mvc)
		return HISTB_MVC_INVALID;

	mvc->interview_size[0] = 0;
	mvc->interview_size[1] = 0;
	mvc->interview_ready = 0;

	if (!mvc->have_view)
		return HISTB_MVC_UNSUPPORTED;

	s = histb_mvc_current_subsps(mvc);
	if (!s || !s->is_valid)
		return HISTB_MVC_UNSUPPORTED;

	/* Step 1: which view order index is the current slice? */
	voIdx = mvc_view_order_index(mvc, s, mvc->view.view_id);
	if (voIdx <= 0)
		return HISTB_MVC_UNSUPPORTED;	/* base view has no inter-view refs */

	flag_index = HISTB_MVC_FLAG_INDEX(structure);

	for (list = 0; list < 2; list++) {
		const __u32 *refs;
		u32 ref_count;

		/* Step 2: anchor or non-anchor table, mvc.S:5183/5314. */
		if (mvc->view.anchor_pic_flag) {
			ref_count = s->num_anchor_refs[list][voIdx];
			refs = s->anchor_ref[list][voIdx];
		} else {
			ref_count = s->num_non_anchor_refs[list][voIdx];
			refs = s->non_anchor_ref[list][voIdx];
		}

		if (!ref_count)
			continue;

		count = 0;

		/*
		 * Step 3: walk the DPB backwards from dpb_size - 1
		 * (mvc.S:5198-5206).
		 */
		{
			int i;

			for (i = (int)HISTB_MVC_MAX_FRAME_STORE - 1;
			     i >= 0 && count < count + 1; i--) {
				struct histb_mvc_framestore *fs = &mvc->fs[i];
				u32 r;
				bool listed = false;

				if (!fs->in_use)
					continue;

				/* inter_view_flag for this structure (mvc.S:5236) */
				if (!fs->inter_view_flag[flag_index])
					continue;

				/*
				 * decode_poc for this structure must match the
				 * current picture's thispoc (mvc.S:5226-5241).
				 * Using the decoder POC rather than fs->poc is
				 * the whole point: MMCO 5 resets the base
				 * view's ordinary POC and the match would be
				 * missed (mvc.h:605).
				 */
				if (fs->decode_poc[structure >
						  HISTB_MVC_STRUCTURE_BOTTOM ?
						  HISTB_MVC_STRUCTURE_FRAME :
						  structure] != this_poc)
					continue;

				/* view_id must be in the chosen list (mvc.S:5241) */
				for (r = 0; r < ref_count; r++) {
					if (refs[r] == (__u32)fs->view_id) {
						listed = true;
						break;
					}
				}
				if (!listed)
					continue;

				/* de-duplicate against what was emitted */
				{
					u32 e;

					for (e = 0; e < count; e++) {
						if (mvc->interview[list][e].fs == fs)
							break;
					}
					if (e != count)
						continue;
				}

				if (count >= HISTB_MVC_MAX_NUM_VIEWS)
					break;

				/* Step 4: copy the match and reserve a PMV store. */
				mvc->interview[list][count].fs = fs;
				mvc->interview[list][count].buffer = fs->buffer;
				mvc->interview[list][count].apc_slot = fs->apc_slot;
				/*
				 * mvc.S:5272-5273: reserved as TotalPmvNum - 1,
				 * i.e. the last PMV store.  With no PMV
				 * configured yet, record 0 and let the core
				 * fix it up - see MVC-SPEC Q7.
				 */
				fs->pmv_address_idc = mvc->total_pmv_num ?
					mvc->total_pmv_num - 1 : 0;
				mvc->interview[list][count].pmv_address_idc =
					fs->pmv_address_idc;
				mvc->interview[list][count].poc = this_poc;
				mvc->interview[list][count].fields =
					HISTB_MVC_FIELDS_FRAME;
				mvc->interview[list][count].inter_view_flag =
					fs->inter_view_flag[flag_index];
				count++;
				emitted++;
			}
		}

		mvc->interview_size[list] = count;

		/* Step 5: stop once every view has been emitted (mvc.S:5275). */
		if (emitted >= HISTB_MVC_MAX_NUM_VIEWS)
			break;
	}

	mvc->view.interviewlist_xsize[0] = (__u8)mvc->interview_size[0];
	mvc->view.interviewlist_xsize[1] = (__u8)mvc->interview_size[1];
	mvc->interview_ready = (mvc->interview_size[0] ||
				mvc->interview_size[1]) ? 1 : 0;

	return HISTB_MVC_OK;
}

__u32 histb_mvc_interview_size(const struct histb_mvc_state *mvc,
			       unsigned int list)
{
	if (!mvc || list > 1 || !mvc->interview_ready)
		return 0;

	return mvc->interview_size[list];
}

const struct histb_mvc_interview *histb_mvc_interview(
				const struct histb_mvc_state *mvc,
				unsigned int list, unsigned int index)
{
	if (!mvc || list > 1 || !mvc->interview_ready)
		return NULL;

	if (index >= mvc->interview_size[list])
		return NULL;

	return &mvc->interview[list][index];
}
