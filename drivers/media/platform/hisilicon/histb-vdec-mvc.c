// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 *
 * H.264 MVC subset-SPS and NAL-extension parsing for the Hi3798MV310.
 * The request ingress owns view/reference lifetimes and supplies the parsed
 * syntax to the existing H.264 message builder. No alternate DPB is kept here.
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
	br->size_bits = bytes <= U32_MAX / 8 ? bytes * 8 : 0;
	br->pos = 0;
	br->overrun = bytes > U32_MAX / 8;
}

static __u32 bits_get(struct histb_mvc_bits *br, unsigned int count)
{
	__u32 value = 0;
	unsigned int i;

	if (br->overrun || count > 32 || count > br->size_bits - br->pos) {
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
	if (!ext->reserved_one_bit)
		return HISTB_MVC_INVALID;

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
static void mvc_skip_hrd(struct histb_mvc_bits *br)
{
	u32 i, count = bits_ue(br);

	if (count > 31) {
		br->overrun = true;
		return;
	}
	bits_get(br, 8);
	for (i = 0; i <= count && !br->overrun; i++) {
		bits_ue(br);
		bits_ue(br);
		bits_flag(br);
	}
	bits_get(br, 20);
}

static void mvc_skip_vui(struct histb_mvc_bits *br)
{
	bool nal_hrd, vcl_hrd;

	if (bits_flag(br) && bits_get(br, 8) == 255) {
		bits_get(br, 16);
		bits_get(br, 16);
	}
	if (bits_flag(br))
		bits_flag(br);
	if (bits_flag(br)) {
		bits_get(br, 4);
		if (bits_flag(br))
			bits_get(br, 24);
	}
	if (bits_flag(br)) {
		bits_ue(br);
		bits_ue(br);
	}
	if (bits_flag(br)) {
		bits_get(br, 32);
		bits_get(br, 32);
		bits_flag(br);
	}
	nal_hrd = bits_flag(br);
	if (nal_hrd)
		mvc_skip_hrd(br);
	vcl_hrd = bits_flag(br);
	if (vcl_hrd)
		mvc_skip_hrd(br);
	if (nal_hrd || vcl_hrd)
		bits_flag(br);
	bits_flag(br);
	if (bits_flag(br)) {
		unsigned int i;

		bits_flag(br);
		for (i = 0; i < 6; i++)
			bits_ue(br);
	}
}

static int mvc_parse_ordinary_sps(struct histb_mvc_sps *sps,
				  struct histb_mvc_bits *br)
{
	u32 chroma_format_idc = 1, value;

	sps->profile_idc = bits_get(br, 8);
	bits_get(br, 8);			/* constraint set flags */
	sps->level_idc = bits_get(br, 8);
	value = bits_ue(br);
	if (value > 31)
		return HISTB_MVC_INVALID;
	sps->seq_parameter_set_id = value;

	if (br->overrun)
		return HISTB_MVC_INVALID;

	/* High profiles carry the chroma and scaling-list block. */
	switch (sps->profile_idc) {
	case 100: case 110: case 122: case 244: case 44:
	case 83: case 86: case 118: case 128: case 138: case 139: case 134:
		chroma_format_idc = bits_ue(br);
		if (chroma_format_idc > 3)
			return HISTB_MVC_INVALID;
		if (chroma_format_idc == 3)
			bits_flag(br);		/* separate_colour_plane_flag */
		if (bits_ue(br) > 6 || bits_ue(br) > 6)
			return HISTB_MVC_INVALID;
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

	value = bits_ue(br);
	if (value > 12)
		return HISTB_MVC_INVALID;
	sps->log2_max_frame_num_minus4 = value;
	value = bits_ue(br);
	if (value > 2)
		return HISTB_MVC_INVALID;
	sps->pic_order_cnt_type = value;
	if (sps->pic_order_cnt_type == 0) {
		if (bits_ue(br) > 12)
			return HISTB_MVC_INVALID;
	} else if (sps->pic_order_cnt_type == 1) {
		sps->delta_pic_order_always_zero_flag = bits_flag(br) ? 1 : 0;
		bits_se(br);			/* offset_for_non_ref_pic */
		bits_se(br);			/* offset_for_top_to_bottom_field */
		{
			u32 i, cycles = bits_ue(br);

			if (cycles > 255)
				return HISTB_MVC_INVALID;

			for (i = 0; i < cycles && !br->overrun; i++)
				bits_se(br);
		}
	}

	value = bits_ue(br);
	if (value > 16)
		return HISTB_MVC_INVALID;
	sps->max_num_ref_frames = value;
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

	if (bits_flag(br))
		mvc_skip_vui(br);

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
	 * MVC_ProcessSUBSPSMvcExt, mvc.S:14288-14668, bit order per
	 * MVC-SPEC.md section 5.2.
	 */
	subsps->bit_equal_to_one = bits_flag(&br) ? 1 : 0;
	if (!subsps->bit_equal_to_one)
		return HISTB_MVC_INVALID;
	subsps->num_views_minus1 = bits_ue(&br);

	/* mvc.S:14308 rejects more than two views, matching MAX_NUM_VIEWS. */
	if (br.overrun || subsps->num_views_minus1 >= HISTB_MVC_MAX_NUM_VIEWS)
		return HISTB_MVC_UNSUPPORTED;

	{
		u32 i, v, list;

		for (i = 0; i <= subsps->num_views_minus1; i++) {
			subsps->view_id[i] = (__s32)bits_ue(&br);
			if (subsps->view_id[i] < 0 || subsps->view_id[i] > 1023 ||
			    (i && subsps->view_id[i] == subsps->view_id[0]))
				return HISTB_MVC_INVALID;
		}

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
	 * are not consumed: nothing in the port's message
	 * path reads them.  The reader stops here deliberately.
	 */
	subsps->is_valid = 1;

	return HISTB_MVC_OK;
}
