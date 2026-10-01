// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hi3798CV200 VC-1 Advanced Profile message and bit-plane contracts.
 *
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 *
 * Source mapping is recorded next to every public input in the header.  The
 * packers below follow VC1HAL_V4R3C1_CfgDnMsg, CfgSliceMsg and BPD_CfgReg.
 * The independent parser restores the original Annex G syntax-to-BPD
 * boundary.  This file intentionally does not register a V4L2 format.
 */

#include "histb-vdec-vc1.h"

#define VC1_BPD_AXI_CFG		0x00000330U
#define VC1_SLICE_LENGTH_MAX	0x01ffffffU

/* VC1 Table 40, base-256 scale used by the CV200 motion predictor. */
static const __s16 vc1_bfraction_scale[HISTB_VC1_BFRACTION_COUNT] = {
	128, 85, 170, 64, 192, 51, 102, 153, 204, 43, 215,
	37, 74, 111, 148, 185, 222, 32, 96, 160, 224,
};

static void vc1_zero(void *ptr, __u32 bytes)
{
	__u8 *p = ptr;

	while (bytes--)
		*p++ = 0;
}

static int vc1_bool(__u8 value)
{
	return value <= 1;
}

static int vc1_fcm_valid(__u8 value)
{
	return value == HISTB_VC1_PROGRESSIVE ||
	       value == HISTB_VC1_FRAME_INTERLACED ||
	       value == HISTB_VC1_FIELD_INTERLACED;
}

int histb_vc1_decode_bfraction(__u8 prefix, __u8 suffix,
				       struct histb_vc1_bfraction *fraction)
{
	__u8 index;

	if (!fraction || prefix > 7 || (prefix < 7 && suffix))
		return HISTB_VC1_INVALID;
	index = prefix == 7 ? 7 + suffix : prefix;
	if (index >= HISTB_VC1_BFRACTION_COUNT)
		return HISTB_VC1_INVALID;
	fraction->index = index;
	fraction->scale = vc1_bfraction_scale[index];
	return HISTB_VC1_OK;
}

static __u8 vc1_clip_u8(__s32 value)
{
	if (value < 0)
		return 0;
	if (value > 255)
		return 255;
	return value;
}

static __u8 vc1_range_map_sample(__u8 sample, __u8 range)
{
	return vc1_clip_u8(((((sample - 128) * (range + 9) + 4) >> 3) +
			    128));
}

int histb_vc1_validate_entry_range_transition(
			const struct histb_vc1_entry_point *previous,
			const struct histb_vc1_entry_point *current_entry)
{
	if (!previous || !current_entry)
		return HISTB_VC1_INVALID;
	if (current_entry->closed_entry)
		return HISTB_VC1_OK;
	if (previous->range_map_y_flag != current_entry->range_map_y_flag ||
	    previous->range_map_y != current_entry->range_map_y ||
	    previous->range_map_uv_flag != current_entry->range_map_uv_flag ||
	    previous->range_map_uv != current_entry->range_map_uv)
		return HISTB_VC1_INVALID;
	return HISTB_VC1_OK;
}

int histb_vc1_range_map_nv12(__u8 *data, __u32 stride, __u16 width,
			      __u16 height,
			      const struct histb_vc1_entry_point *entry)
{
	__u32 row, column, chroma_offset;

	if (!data || !entry || !width || !height || (height & 1) ||
	    stride < width || !vc1_bool(entry->range_map_y_flag) ||
	    !vc1_bool(entry->range_map_uv_flag) || entry->range_map_y > 7 ||
	    entry->range_map_uv > 7 || stride > (~0U / height))
		return HISTB_VC1_INVALID;
	chroma_offset = stride * height;
	if (entry->range_map_y_flag)
		for (row = 0; row < height; row++)
			for (column = 0; column < width; column++)
				data[row * stride + column] = vc1_range_map_sample(
					data[row * stride + column],
					entry->range_map_y);
	if (entry->range_map_uv_flag)
		for (row = 0; row < height / 2; row++)
			for (column = 0; column < width; column++)
				data[chroma_offset + row * stride + column] =
					vc1_range_map_sample(
					data[chroma_offset + row * stride + column],
					entry->range_map_uv);
	return HISTB_VC1_OK;
}

void histb_vc1_intensity_init(struct histb_vc1_intensity_map *map)
{
	unsigned int parity, component, value;

	if (!map)
		return;
	for (parity = 0; parity < HISTB_VC1_INTENSITY_PARITIES; parity++)
		for (component = 0;
		     component < HISTB_VC1_INTENSITY_COMPONENTS; component++)
			for (value = 0; value < HISTB_VC1_INTENSITY_VALUES; value++)
				map->value[parity][component][value] = value;
}

int histb_vc1_intensity_compose(const struct histb_vc1_intensity_map *input,
				__u8 lum_scale, __u8 lum_shift,
				struct histb_vc1_intensity_map *output)
{
	__s32 shift, scale, luma_bias;
	unsigned int parity, value;

	if (!input || !output || lum_scale > 63 || lum_shift > 63)
		return HISTB_VC1_INVALID;
	shift = lum_shift >= 32 ? (__s32)lum_shift - 64 : lum_shift;
	scale = lum_scale ? (__s32)lum_scale + 32 : -64;
	luma_bias = (lum_scale ? shift : 255 - 2 * shift) * 64;
	for (parity = 0; parity < HISTB_VC1_INTENSITY_PARITIES; parity++) {
		for (value = 0; value < HISTB_VC1_INTENSITY_VALUES; value++) {
			__s32 old_luma = input->value[parity][0][value];
			__s32 old_chroma = input->value[parity][1][value];

			output->value[parity][0][value] = vc1_clip_u8(
				(scale * old_luma + luma_bias + 32) >> 6);
			output->value[parity][1][value] = vc1_clip_u8(
				(scale * (old_chroma - 128) + 128 * 64 + 32) >> 6);
		}
	}
	return HISTB_VC1_OK;
}

int histb_vc1_intensity_compose_parity(
				const struct histb_vc1_intensity_map *input,
				__u8 parity, __u8 lum_scale, __u8 lum_shift,
				struct histb_vc1_intensity_map *output)
{
	__s32 shift, scale, luma_bias;
	unsigned int value;

	if (!input || !output || parity >= HISTB_VC1_INTENSITY_PARITIES ||
	    lum_scale > 63 || lum_shift > 63)
		return HISTB_VC1_INVALID;
	if (output != input)
		*output = *input;
	shift = lum_shift >= 32 ? (__s32)lum_shift - 64 : lum_shift;
	scale = lum_scale ? (__s32)lum_scale + 32 : -64;
	luma_bias = (lum_scale ? shift : 255 - 2 * shift) * 64;
	for (value = 0; value < HISTB_VC1_INTENSITY_VALUES; value++) {
		__s32 old_luma = input->value[parity][0][value];
		__s32 old_chroma = input->value[parity][1][value];

		output->value[parity][0][value] = vc1_clip_u8(
			(scale * old_luma + luma_bias + 32) >> 6);
		output->value[parity][1][value] = vc1_clip_u8(
			(scale * (old_chroma - 128) + 128 * 64 + 32) >> 6);
	}
	return HISTB_VC1_OK;
}

int histb_vc1_pack_intensity_work(
			__u8 ptype,
			const struct histb_vc1_intensity_map *earlier,
			const struct histb_vc1_intensity_map *latest,
			const struct histb_vc1_intensity_map *transformed_latest,
			struct histb_vc1_intensity_work *work)
{
	unsigned int i;

	if (!work || ptype > HISTB_VC1_PICTURE_BI_HW)
		return HISTB_VC1_INVALID;
	for (i = 0; i < HISTB_VC1_INTENSITY_REFS; i++)
		histb_vc1_intensity_init(&work->ref[i]);
	if (ptype == HISTB_VC1_PICTURE_P) {
		if (!latest)
			return HISTB_VC1_INVALID;
		work->ref[0] = transformed_latest ? *transformed_latest : *latest;
	} else if (ptype == HISTB_VC1_PICTURE_B) {
		if (!earlier || !latest)
			return HISTB_VC1_INVALID;
		work->ref[0] = *earlier;
		work->ref[1] = *latest;
	}
	return HISTB_VC1_OK;
}

int histb_vc1_pack_field_intensity_work(
			__u8 ptype,
			const struct histb_vc1_intensity_map *earlier,
			const struct histb_vc1_intensity_map *latest,
			const struct histb_vc1_intensity_map *current_map,
			struct histb_vc1_intensity_work *work)
{
	unsigned int i;

	if (!work || ptype > HISTB_VC1_PICTURE_BI_HW)
		return HISTB_VC1_INVALID;
	for (i = 0; i < HISTB_VC1_INTENSITY_REFS; i++)
		histb_vc1_intensity_init(&work->ref[i]);
	if (ptype == HISTB_VC1_PICTURE_P) {
		if (!current_map)
			return HISTB_VC1_INVALID;
		work->ref[0] = latest ? *latest : *current_map;
		work->ref[2] = *current_map;
	} else if (ptype == HISTB_VC1_PICTURE_B) {
		if (!earlier || !latest)
			return HISTB_VC1_INVALID;
		work->ref[0] = *earlier;
		work->ref[1] = *latest;
	}
	return HISTB_VC1_OK;
}

int histb_vc1_effective_halfpel(__u8 mv_mode, __u8 mv_mode2,
				__u8 *halfpel)
{
	__u8 effective;

	if (!halfpel || mv_mode > HISTB_VC1_MV_INTENSITY_COMP || mv_mode2 > 3)
		return HISTB_VC1_INVALID;
	effective = mv_mode == HISTB_VC1_MV_INTENSITY_COMP ? mv_mode2 : mv_mode;
	*halfpel = effective == HISTB_VC1_MV_1MV_HALFPEL ||
		   effective == HISTB_VC1_MV_1MV_HALFPEL_BILINEAR;
	return HISTB_VC1_OK;
}

int histb_vc1_picture_halfpel(const struct histb_vc1_parsed_picture *picture,
			      __u8 reference_halfpel, __u8 *halfpel)
{
	if (!picture || !halfpel || !vc1_bool(reference_halfpel) ||
	    picture->ptype > HISTB_VC1_PICTURE_BI_HW)
		return HISTB_VC1_INVALID;
	if (picture->skipped) {
		if (picture->ptype != HISTB_VC1_PICTURE_P)
			return HISTB_VC1_INVALID;
		*halfpel = reference_halfpel;
		return HISTB_VC1_OK;
	}
	if (picture->ptype == HISTB_VC1_PICTURE_P ||
	    picture->ptype == HISTB_VC1_PICTURE_B)
		return histb_vc1_effective_halfpel(picture->mv_mode,
						  picture->mv_mode2, halfpel);
	*halfpel = 0;
	return HISTB_VC1_OK;
}

static int vc1_validate_picture(const struct histb_vc1_picture *p)
{
	unsigned int i;

	if (p->profile > HISTB_VC1_PROFILE_ADVANCED ||
	    !vc1_fcm_valid(p->fcm) ||
	    p->picture_structure != p->fcm)
		return HISTB_VC1_INVALID;
	if (p->profile != HISTB_VC1_PROFILE_ADVANCED &&
	    p->fcm != HISTB_VC1_PROGRESSIVE)
		return HISTB_VC1_UNSUPPORTED;
	if (p->ptype > HISTB_VC1_PICTURE_BI_HW ||
	    p->mb_width < 3 || p->mb_width > HISTB_VC1_MAX_MB_DIMENSION ||
	    p->mb_height < 3 || p->mb_height > HISTB_VC1_MAX_MB_DIMENSION ||
	    !p->display_width ||
	    p->display_width > HISTB_VC1_MAX_MB_DIMENSION * 16U ||
	    !p->display_height ||
	    p->display_height > HISTB_VC1_MAX_MB_DIMENSION * 16U ||
	    !p->total_slices || p->total_slices > HISTB_VC1_MAX_SLICES)
		return HISTB_VC1_INVALID;
	if (p->profile == HISTB_VC1_PROFILE_ADVANCED &&
	    (p->display_width > p->mb_width * 16U ||
	     p->display_height > p->mb_height *
		(p->fcm == HISTB_VC1_FIELD_INTERLACED ? 32U : 16U)))
		return HISTB_VC1_INVALID;
	if (!vc1_bool(p->loopfilter) || !vc1_bool(p->is_second_field) ||
	    !vc1_bool(p->current_parity) || !vc1_bool(p->num_ref) ||
	    !vc1_fcm_valid(p->forward_fcm) ||
	    !vc1_fcm_valid(p->backward_fcm) ||
	    !vc1_bool(p->rounding_control) || !vc1_bool(p->fast_uv_mc) ||
	    !vc1_bool(p->overlap) || p->condover > 3 || p->pquant > 31 ||
	    p->pqindex > 31 || p->altpquant > 31 || !vc1_bool(p->halfqp) ||
	    !vc1_bool(p->b_uniform) || !vc1_bool(p->use_alt_qp) ||
	    p->dquant > 3 || p->dqprofile > 3 || !vc1_bool(p->dqbi_level) ||
	    !vc1_bool(p->dquant_frame) || p->quant_mode > 15 ||
	    p->mv_mode > 7 || p->mv_mode2 > 3 ||
	    !vc1_bool(p->current_halfpel) || !vc1_bool(p->colocated_halfpel) ||
	    p->mv_range > 3 || p->ref_dist > 31 || p->dmv_range > 3 ||
	    !vc1_bool(p->ref_field) || !vc1_bool(p->trans_dc_table) ||
	    !vc1_bool(p->variable_transform) || !vc1_bool(p->ttmbf) ||
	    p->trans_ac_frame > 3 || p->trans_ac_frame2 > 3 || p->ttfrm > 3)
		return HISTB_VC1_INVALID;
	if (!vc1_bool(p->forward_mb_raw) || !vc1_bool(p->direct_mb_raw) ||
	    !vc1_bool(p->mvtype_mb_raw) || !vc1_bool(p->fieldtx_raw) ||
	    !vc1_bool(p->skip_mb_raw) || !vc1_bool(p->acpred_raw) ||
	    !vc1_bool(p->overflags_raw) || p->mv_table > 7 ||
	    p->cbp_table > 7 || p->b_fraction > 127 ||
	    p->mb_mode_table > 7 || p->two_mv_bp_table > 3 ||
	    p->four_mv_bp_table > 3 || !vc1_bool(p->four_mv_switch) ||
	    !vc1_bool(p->range_map_y_flag) || p->range_map_y > 7 ||
	    !vc1_bool(p->range_map_uv_flag) || p->range_map_uv > 7 ||
	    !vc1_bool(p->range_reduction) || !vc1_bool(p->range_reduction0) ||
	    !vc1_bool(p->range_reduction1) || !vc1_bool(p->postproc) ||
	    p->codec_version > 7)
		return HISTB_VC1_INVALID;
	if (p->fcm != HISTB_VC1_FIELD_INTERLACED &&
	    (p->is_second_field || p->current_parity || p->ref_field))
		return HISTB_VC1_UNSUPPORTED;
	if (!p->current_picture_addr || !p->current_colmb_addr ||
	    !p->sed_top_addr || !p->pmv_top_addr || !p->itrans_top_addr ||
	    !p->dblk_top_addr || !p->intensity_table_addr ||
	    p->bpd_mb_width != p->mb_width || p->bpd_mb_width < 3 ||
	    p->bpd_mb_width > HISTB_VC1_MAX_MB_DIMENSION ||
	    p->bpd_stride != ((__u32)(p->bpd_mb_width + 127) >> 7) << 4 ||
	    !p->slice_info_addr)
		return HISTB_VC1_INVALID;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		if (!p->bitplane_addr[i])
			return HISTB_VC1_INVALID;
	if (p->ptype == HISTB_VC1_PICTURE_P && !p->forward_ref_addr)
		return HISTB_VC1_INVALID;
	if (p->ptype == HISTB_VC1_PICTURE_B &&
	    (!p->forward_ref_addr || !p->backward_ref_addr ||
	     !p->backward_colmb_addr))
		return HISTB_VC1_INVALID;
	return HISTB_VC1_OK;
}

int histb_vc1_build_pic_msg(const struct histb_vc1_picture *p,
			     struct histb_vc1_pic_msg *msg)
{
	int ret;
	unsigned int i;

	if (!p || !msg)
		return HISTB_VC1_INVALID;
	ret = vc1_validate_picture(p);
	if (ret)
		return ret;

	vc1_zero(msg, sizeof(*msg));
	msg->d[0] = p->ptype | ((__u32)p->profile << 4) |
		    ((__u32)p->fcm << 14);
	msg->d[1] = (p->mb_width - 1) | ((__u32)(p->mb_height - 1) << 16);
	msg->d[2] = p->loopfilter | ((__u32)p->is_second_field << 1) |
		    ((__u32)p->current_parity << 2) | ((__u32)p->num_ref << 3) |
		    ((__u32)p->forward_fcm << 4) | ((__u32)p->backward_fcm << 6);
	msg->d[3] = p->rounding_control | ((__u32)p->fast_uv_mc << 1) |
		    ((__u32)p->overlap << 2) | ((__u32)p->condover << 4);
	msg->d[4] = p->pquant | ((__u32)p->pqindex << 8) |
		    ((__u32)p->altpquant << 16) | ((__u32)p->halfqp << 24);
	msg->d[5] = p->b_uniform | ((__u32)p->use_alt_qp << 1) |
		    ((__u32)p->dquant << 2) | ((__u32)p->dqprofile << 8) |
		    ((__u32)p->dqbi_level << 12) |
		    ((__u32)p->dquant_frame << 14) |
		    ((__u32)p->quant_mode << 16);
	msg->d[6] = p->mv_mode | ((__u32)p->mv_mode2 << 4);
	msg->d[7] = p->current_halfpel | ((__u32)p->colocated_halfpel << 1) |
		    ((__u32)p->mv_range << 2) | ((__u32)p->ref_dist << 4) |
		    ((__u32)p->dmv_range << 12) | ((__u32)p->ref_field << 14);
	msg->d[8] = p->trans_dc_table | ((__u32)p->variable_transform << 1) |
		    ((__u32)p->ttmbf << 2) | ((__u32)p->trans_ac_frame << 4) |
		    ((__u32)p->trans_ac_frame2 << 6) | ((__u32)p->ttfrm << 8);
	msg->d[9] = p->forward_mb_raw | ((__u32)p->direct_mb_raw << 1) |
		    ((__u32)p->mvtype_mb_raw << 2) | ((__u32)p->fieldtx_raw << 3) |
		    ((__u32)p->skip_mb_raw << 4) | ((__u32)p->acpred_raw << 5) |
		    ((__u32)p->overflags_raw << 6);
	msg->d[10] = p->mv_table | ((__u32)p->cbp_table << 8) |
		     ((__u32)p->b_fraction << 16);
	msg->d[11] = p->mb_mode_table | ((__u32)p->two_mv_bp_table << 4) |
		     ((__u32)p->four_mv_bp_table << 8) |
		     ((__u32)p->four_mv_switch << 12);
	msg->d[12] = p->scale_factor;
	msg->d[13] = p->forward_ref_dist;
	msg->d[14] = p->backward_ref_dist;
	msg->d[15] = p->total_slices | ((__u32)p->range_map_y_flag << 20) |
		     ((__u32)p->range_map_y << 21) |
		     ((__u32)p->range_map_uv_flag << 24) |
		     ((__u32)p->range_map_uv << 25);
	msg->d[16] = p->current_picture_addr;
	msg->d[17] = p->forward_ref_addr;
	msg->d[18] = p->backward_ref_addr;
	msg->d[19] = p->current_colmb_addr;
	msg->d[20] = p->backward_colmb_addr;
	msg->d[21] = p->sed_top_addr;
	msg->d[22] = p->pmv_top_addr;
	msg->d[23] = p->itrans_top_addr;
	msg->d[24] = p->dblk_top_addr;
	msg->d[26] = p->intensity_table_addr;
	msg->d[27] = p->display_width | ((__u32)p->display_height << 16);
	msg->d[28] = ((__u32)p->range_reduction << 20) |
		     ((__u32)p->range_reduction0 << 21) |
		     ((__u32)p->range_reduction1 << 22) |
		     ((__u32)p->postproc << 24) |
		     ((__u32)p->codec_version << 25);
	msg->d[29] = p->bpd_stride;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		msg->d[30 + i] = p->bitplane_addr[i];
	msg->d[63] = p->slice_info_addr;
	return HISTB_VC1_OK;
}

static int vc1_validate_fragment(const struct histb_vc1_slice_fragment *f,
				  int required)
{
	if (!f->bit_len)
		return !required && !f->dma_addr && !f->bit_offset ?
			HISTB_VC1_OK : HISTB_VC1_INVALID;
	if (!f->dma_addr || f->bit_len > VC1_SLICE_LENGTH_MAX ||
	    f->bit_offset > 7)
		return HISTB_VC1_INVALID;
	return HISTB_VC1_OK;
}

int histb_vc1_build_slice_messages(const struct histb_vc1_slice *slices,
				    __u32 count, __u16 mb_width,
				    __u16 mb_height, __u32 message_dma,
				    struct histb_vc1_slice_msg *messages,
				    __u32 capacity, __u32 *stream_base)
{
	__u32 total_mbs = (__u32)mb_width * mb_height;
	__u32 base = ~0U;
	unsigned int i, fragment;

	if (!slices || !messages || !stream_base || !count ||
	    count > HISTB_VC1_MAX_SLICES || count > capacity ||
	    mb_width < 3 || mb_width > HISTB_VC1_MAX_MB_DIMENSION ||
	    mb_height < 3 || mb_height > HISTB_VC1_MAX_MB_DIMENSION ||
	    (message_dma & (sizeof(*messages) - 1)) ||
	    message_dma > ~0U - (count - 1) * sizeof(*messages))
		return HISTB_VC1_INVALID;

	for (i = 0; i < count; i++) {
		if (vc1_validate_fragment(&slices[i].fragment[0], 1) ||
		    vc1_validate_fragment(&slices[i].fragment[1], 0) ||
		    slices[i].start_mb != (i ? slices[i - 1].end_mb + 1 : 0) ||
		    slices[i].end_mb < slices[i].start_mb ||
		    slices[i].end_mb >= total_mbs)
			return HISTB_VC1_INVALID;
		for (fragment = 0; fragment < HISTB_VC1_SLICE_FRAGMENTS;
		     fragment++) {
			__u32 aligned;

			if (!slices[i].fragment[fragment].bit_len)
				continue;
			aligned = slices[i].fragment[fragment].dma_addr & ~15U;
			if (aligned < base)
				base = aligned;
		}
	}
	if (slices[count - 1].end_mb != total_mbs - 1 || base == ~0U)
		return HISTB_VC1_INVALID;

	vc1_zero(messages, count * sizeof(*messages));
	for (i = 0; i < count; i++) {
		for (fragment = 0; fragment < HISTB_VC1_SLICE_FRAGMENTS;
		     fragment++) {
			const struct histb_vc1_slice_fragment *f =
				&slices[i].fragment[fragment];
			__u32 aligned, adjusted, word = fragment * 2;

			if (!f->bit_len)
				continue;
			aligned = f->dma_addr & ~15U;
			adjusted = f->bit_offset + (f->dma_addr & 15) * 8;
			if (adjusted > 127 || aligned < base)
				return HISTB_VC1_INVALID;
			messages[i].d[word] = f->bit_len | (adjusted << 25);
			messages[i].d[word + 1] = aligned - base;
		}
		messages[i].d[4] = slices[i].start_mb |
				   ((__u32)slices[i].end_mb << 16);
		messages[i].d[63] = i + 1 < count ?
			message_dma + (i + 1) * sizeof(*messages) : 0;
	}
	*stream_base = base;
	return HISTB_VC1_OK;
}

int histb_vc1_build_bpd_regs(const struct histb_vc1_bpd_config *c,
			      struct histb_vc1_bpd_regs *regs)
{
	__u32 adjusted;
	unsigned int i;

	if (!c || !regs)
		return HISTB_VC1_INVALID;
	if (c->profile > HISTB_VC1_PROFILE_ADVANCED ||
	    !vc1_fcm_valid(c->picture_structure) ||
	    (c->profile != HISTB_VC1_PROFILE_ADVANCED &&
	     c->picture_structure != HISTB_VC1_PROGRESSIVE))
		return HISTB_VC1_UNSUPPORTED;
	if (!c->stream_addr || c->bit_offset > 7 ||
	    c->mb_width < 3 || c->mb_width > HISTB_VC1_MAX_MB_DIMENSION ||
	    c->mb_height < 3 || c->mb_height > HISTB_VC1_MAX_MB_DIMENSION ||
	    !vc1_bool(c->mv_mode_enable) || !vc1_bool(c->overflags_enable) ||
	    c->ptype > HISTB_VC1_PICTURE_BI_HW)
		return HISTB_VC1_INVALID;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		if (!c->bitplane_addr[i] || (c->bitplane_addr[i] & 15))
			return HISTB_VC1_INVALID;
	adjusted = c->bit_offset + (c->stream_addr & 15) * 8;
	if (adjusted > 255)
		return HISTB_VC1_INVALID;

	vc1_zero(regs, sizeof(*regs));
	regs->cfg[0] = adjusted | ((__u32)(c->mb_width % 3) << 8) |
		       ((__u32)(c->mb_height % 3) << 10) |
		       ((__u32)c->mv_mode_enable << 12) |
		       ((__u32)c->overflags_enable << 13) |
		       ((__u32)c->ptype << 14) |
		       ((__u32)c->picture_structure << 16) |
		       ((__u32)c->profile << 18);
	/* The Linux path supplies direct DMA addresses and does not initialise
	 * the private BPD MMU used by the vendor firmware.
	 */
	regs->cfg[1] = c->stream_addr & ~15U;
	regs->cfg[2] = (c->mb_width - 1) | ((__u32)(c->mb_height - 1) << 16);
	/* BPD_CfgReg uses roundup(mb_width, 128) / 8 bytes. */
	regs->cfg[3] = ((c->mb_width + 127) >> 7) << 4;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		regs->cfg[4 + i] = c->bitplane_addr[i];
	regs->cfg[11] = VC1_BPD_AXI_CFG;
	return HISTB_VC1_OK;
}

int histb_vc1_parse_bpd_result(__u32 state, __u32 out0, __u32 out1,
			       __u32 available_bits,
			       struct histb_vc1_bpd_result *result)
{
	unsigned int i;

	if (!result || !available_bits)
		return HISTB_VC1_INVALID;
	if (state & HISTB_VC1_BPD_STATE_ERROR)
		return HISTB_VC1_HW_ERROR;
	if (!(state & HISTB_VC1_BPD_STATE_DONE))
		return HISTB_VC1_NOT_READY;
	/* BPD_GetParam rejects OUT0 equal to or beyond the available bit count. */
	if (out0 >= available_bits)
		return HISTB_VC1_INVALID;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		if (((out1 >> (i * 4)) & 15) > 7)
			return HISTB_VC1_INVALID;

	vc1_zero(result, sizeof(*result));
	result->eaten_bits = out0;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		result->mode[i] = (out1 >> (i * 4)) & 15;
	result->condover = (out1 >> 28) & 3;
	return HISTB_VC1_OK;
}

struct vc1_bitreader {
	const __u8 *data;
	__u32 size;
	__u32 bit;
};

static int vc1_get_bits(struct vc1_bitreader *br, __u8 count, __u32 *value)
{
	__u32 result = 0;
	unsigned int i;

	if (!count || count > 32 || br->bit > br->size * 8U ||
	    count > br->size * 8U - br->bit)
		return HISTB_VC1_INVALID;
	for (i = 0; i < count; i++) {
		result <<= 1;
		result |= (br->data[br->bit >> 3] >> (7 - (br->bit & 7))) & 1;
		br->bit++;
	}
	*value = result;
	return HISTB_VC1_OK;
}

static int vc1_skip_bits(struct vc1_bitreader *br, __u32 count)
{
	if (br->bit > br->size * 8U || count > br->size * 8U - br->bit)
		return HISTB_VC1_INVALID;
	br->bit += count;
	return HISTB_VC1_OK;
}

static int vc1_get_unary(struct vc1_bitreader *br, __u8 stop,
			  __u8 maximum, __u32 *value)
{
	__u32 bit;
	unsigned int i;

	for (i = 0; i < maximum; i++) {
		if (vc1_get_bits(br, 1, &bit))
			return HISTB_VC1_INVALID;
		if (bit == stop) {
			*value = i;
			return HISTB_VC1_OK;
		}
	}
	*value = maximum;
	return HISTB_VC1_OK;
}

static int vc1_decode012(struct vc1_bitreader *br, __u32 *value)
{
	__u32 bit;

	if (vc1_get_bits(br, 1, &bit))
		return HISTB_VC1_INVALID;
	if (!bit) {
		*value = 0;
		return HISTB_VC1_OK;
	}
	if (vc1_get_bits(br, 1, &bit))
		return HISTB_VC1_INVALID;
	*value = bit + 1;
	return HISTB_VC1_OK;
}

int histb_vc1_next_unit(const __u8 *data, __u32 size, __u32 *cursor,
			 struct histb_vc1_unit *unit)
{
	__u32 start, end;

	if (!data || !cursor || !unit || *cursor > size)
		return HISTB_VC1_INVALID;
	if (size < 4)
		return HISTB_VC1_NOT_READY;
	for (start = *cursor; start <= size - 4; start++)
		if (!data[start] && !data[start + 1] && data[start + 2] == 1)
			break;
	if (start > size - 4)
		return HISTB_VC1_NOT_READY;
	for (end = start + 4; end <= size - 3; end++)
		if (!data[end] && !data[end + 1] && data[end + 2] == 1)
			break;
	if (end > size - 3)
		end = size;

	unit->type = data[start + 3];
	unit->start_offset = start;
	unit->payload_offset = start + 4;
	unit->payload_size = end - unit->payload_offset;
	*cursor = end;
	return HISTB_VC1_OK;
}

void histb_vc1_identity_view(const __u8 *data, __u32 size,
			      struct histb_vc1_parse_view *view)
{
	if (!view)
		return;
	view->raw = data;
	view->unescaped = data;
	view->u2r = 0;
	view->raw_size = size;
	view->unescaped_size = size;
}

int histb_vc1_unescape_unit(const __u8 *raw, __u32 raw_size,
			     __u8 *unescaped, __u32 unescaped_capacity,
			     __u32 *u2r, __u32 u2r_capacity,
			     struct histb_vc1_parse_view *view)
{
	__u32 r, u = 0;

	if (!raw || !raw_size || !unescaped || !u2r || !view ||
	    unescaped_capacity < raw_size || u2r_capacity <= raw_size)
		return HISTB_VC1_INVALID;
	for (r = 0; r < raw_size; r++) {
		if (r >= 2 && raw[r] == 3 && !raw[r - 2] && !raw[r - 1] &&
		    r + 1 < raw_size && raw[r + 1] <= 3)
			continue;
		unescaped[u] = raw[r];
		u2r[u++] = r;
	}
	u2r[u] = raw_size;
	view->raw = raw;
	view->unescaped = unescaped;
	view->u2r = u2r;
	view->raw_size = raw_size;
	view->unescaped_size = u;
	return HISTB_VC1_OK;
}

static int vc1_view_valid(const struct histb_vc1_parse_view *view)
{
	return view && view->raw && view->unescaped && view->raw_size &&
		view->unescaped_size && view->unescaped_size <= view->raw_size;
}

int histb_vc1_u_to_raw_bit(const struct histb_vc1_parse_view *view,
			    __u32 semantic_bit, __u32 *raw_bit)
{
	__u32 byte, bit, raw_byte;

	if (!vc1_view_valid(view) || !raw_bit ||
	    semantic_bit > view->unescaped_size * 8U)
		return HISTB_VC1_INVALID;
	if (!view->u2r) {
		*raw_bit = semantic_bit;
		return HISTB_VC1_OK;
	}
	byte = semantic_bit >> 3;
	bit = semantic_bit & 7;
	raw_byte = view->u2r[byte];
	if (raw_byte > view->raw_size ||
	    (byte < view->unescaped_size && raw_byte >= view->raw_size))
		return HISTB_VC1_INVALID;
	*raw_bit = raw_byte * 8U + bit;
	return HISTB_VC1_OK;
}

int histb_vc1_raw_to_u_bit(const struct histb_vc1_parse_view *view,
			    __u32 raw_bit, __u32 *semantic_bit)
{
	__u32 raw_byte, bit, low, high;

	if (!vc1_view_valid(view) || !semantic_bit ||
	    raw_bit > view->raw_size * 8U)
		return HISTB_VC1_INVALID;
	if (!view->u2r) {
		*semantic_bit = raw_bit;
		return HISTB_VC1_OK;
	}
	raw_byte = raw_bit >> 3;
	bit = raw_bit & 7;
	low = 0;
	high = view->unescaped_size + 1;
	while (low < high) {
		__u32 middle = low + (high - low) / 2;

		if (view->u2r[middle] < raw_byte)
			low = middle + 1;
		else
			high = middle;
	}
	if (low > view->unescaped_size)
		return HISTB_VC1_INVALID;
	if (view->u2r[low] != raw_byte) {
		/* An EPB starts a valid semantic byte boundary; its interior does not. */
		if (bit || !low || raw_byte <= view->u2r[low - 1] ||
		    raw_byte >= view->u2r[low])
			return HISTB_VC1_INVALID;
		*semantic_bit = low * 8U;
		return HISTB_VC1_OK;
	}
	*semantic_bit = low * 8U + bit;
	return HISTB_VC1_OK;
}

int histb_vc1_parse_sequence_view(const struct histb_vc1_parse_view *view,
				   struct histb_vc1_sequence *output)
{
	static const __u8 frame_rate_nr[] = { 24, 25, 30, 50, 60, 48, 72 };
	static const __u16 frame_rate_dr[] = { 1000, 1001 };
	struct histb_vc1_sequence parsed;
	struct histb_vc1_sequence *sequence = &parsed;
	struct vc1_bitreader br;
	__u32 raw_profile, reserved, value, i;

	if (!vc1_view_valid(view) || view->unescaped_size > (~0U >> 3) ||
	    !output)
		return HISTB_VC1_INVALID;
	br.data = view->unescaped;
	br.size = view->unescaped_size;
	br.bit = 0;
	vc1_zero(sequence, sizeof(*sequence));
	if (vc1_get_bits(&br, 2, &raw_profile) || raw_profile != 3)
		return HISTB_VC1_UNSUPPORTED;
	sequence->profile = HISTB_VC1_PROFILE_ADVANCED;
#define VC1_SEQ_GET(member, count) \
	do { \
		if (vc1_get_bits(&br, count, &value)) \
			return HISTB_VC1_INVALID; \
		sequence->member = value; \
	} while (0)
	VC1_SEQ_GET(level, 3);
	VC1_SEQ_GET(chroma_format, 2);
	if (sequence->level >= 5 || sequence->chroma_format != 1)
		return HISTB_VC1_UNSUPPORTED;
	VC1_SEQ_GET(frame_rate_quant, 3);
	VC1_SEQ_GET(bit_rate_quant, 5);
	VC1_SEQ_GET(postproc_flag, 1);
	VC1_SEQ_GET(max_coded_width, 12);
	VC1_SEQ_GET(max_coded_height, 12);
	sequence->max_coded_width = (sequence->max_coded_width + 1) * 2;
	sequence->max_coded_height = (sequence->max_coded_height + 1) * 2;
	if (!sequence->max_coded_width || !sequence->max_coded_height ||
	    sequence->max_coded_width > HISTB_VC1_MAX_MB_DIMENSION * 16U ||
	    sequence->max_coded_height > HISTB_VC1_MAX_MB_DIMENSION * 16U)
		return HISTB_VC1_UNSUPPORTED;
	VC1_SEQ_GET(broadcast, 1);
	VC1_SEQ_GET(interlace, 1);
	VC1_SEQ_GET(tfcntr_flag, 1);
	VC1_SEQ_GET(finterp_flag, 1);
	if (vc1_get_bits(&br, 1, &reserved) || !reserved)
		return HISTB_VC1_INVALID;
	VC1_SEQ_GET(psf, 1);
	if (sequence->psf)
		return HISTB_VC1_UNSUPPORTED;
	VC1_SEQ_GET(display_ext, 1);
	sequence->display_width = sequence->max_coded_width;
	sequence->display_height = sequence->max_coded_height;
	if (sequence->display_ext) {
		VC1_SEQ_GET(display_width, 14);
		VC1_SEQ_GET(display_height, 14);
		sequence->display_width++;
		sequence->display_height++;
		VC1_SEQ_GET(aspect_ratio_flag, 1);
		if (sequence->aspect_ratio_flag) {
			VC1_SEQ_GET(aspect_ratio, 4);
			if (sequence->aspect_ratio == 15) {
				VC1_SEQ_GET(aspect_width, 8);
				VC1_SEQ_GET(aspect_height, 8);
				sequence->aspect_width++;
				sequence->aspect_height++;
			}
		}
		VC1_SEQ_GET(frame_rate_flag, 1);
		if (sequence->frame_rate_flag) {
			VC1_SEQ_GET(frame_rate_ind, 1);
			if (sequence->frame_rate_ind) {
				if (vc1_get_bits(&br, 16, &value))
					return HISTB_VC1_INVALID;
				sequence->frame_rate_nr = value + 1;
				sequence->frame_rate_dr = 32;
			} else {
				__u32 nr, dr;

				if (vc1_get_bits(&br, 8, &nr) ||
				    vc1_get_bits(&br, 4, &dr) || !nr || nr > 7 ||
				    !dr || dr > 2)
					return HISTB_VC1_INVALID;
				sequence->frame_rate_nr = frame_rate_nr[nr - 1] * 1000U;
				sequence->frame_rate_dr = frame_rate_dr[dr - 1];
			}
		}
		VC1_SEQ_GET(color_format_flag, 1);
		if (sequence->color_format_flag) {
			VC1_SEQ_GET(color_primaries, 8);
			VC1_SEQ_GET(transfer_characteristics, 8);
			VC1_SEQ_GET(matrix_coefficients, 8);
		}
	}
	VC1_SEQ_GET(hrd_param_flag, 1);
	if (sequence->hrd_param_flag) {
		VC1_SEQ_GET(hrd_bucket_count, 5);
		VC1_SEQ_GET(hrd_rate_exponent, 4);
		VC1_SEQ_GET(hrd_buffer_exponent, 4);
		for (i = 0; i < sequence->hrd_bucket_count; i++)
			if (vc1_skip_bits(&br, 32))
				return HISTB_VC1_INVALID;
	}
#undef VC1_SEQ_GET
	sequence->header_bits = br.bit;
	*output = parsed;
	return HISTB_VC1_OK;
}

int histb_vc1_parse_sequence(const __u8 *data, __u32 size,
			      struct histb_vc1_sequence *output)
{
	struct histb_vc1_parse_view view;

	histb_vc1_identity_view(data, size, &view);
	return histb_vc1_parse_sequence_view(&view, output);
}

int histb_vc1_parse_entry_point_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_sequence *sequence,
			struct histb_vc1_entry_point *output)
{
	struct histb_vc1_entry_point parsed;
	struct histb_vc1_entry_point *entry = &parsed;
	struct vc1_bitreader br;
	__u32 value, i;

	if (!vc1_view_valid(view) || view->unescaped_size > (~0U >> 3) ||
	    !sequence || !output ||
	    sequence->profile != HISTB_VC1_PROFILE_ADVANCED)
		return HISTB_VC1_INVALID;
	br.data = view->unescaped;
	br.size = view->unescaped_size;
	br.bit = 0;
	vc1_zero(entry, sizeof(*entry));
#define VC1_ENTRY_GET(member, count) \
	do { \
		if (vc1_get_bits(&br, count, &value)) \
			return HISTB_VC1_INVALID; \
		entry->member = value; \
	} while (0)
	VC1_ENTRY_GET(broken_link, 1);
	VC1_ENTRY_GET(closed_entry, 1);
	VC1_ENTRY_GET(panscan_flag, 1);
	VC1_ENTRY_GET(refdist_flag, 1);
	VC1_ENTRY_GET(loopfilter, 1);
	VC1_ENTRY_GET(fast_uv_mc, 1);
	VC1_ENTRY_GET(extended_mv, 1);
	VC1_ENTRY_GET(dquant, 2);
	VC1_ENTRY_GET(variable_transform, 1);
	VC1_ENTRY_GET(overlap, 1);
	VC1_ENTRY_GET(quantizer_mode, 2);
	if (sequence->hrd_param_flag)
		for (i = 0; i < sequence->hrd_bucket_count; i++)
			if (vc1_skip_bits(&br, 8))
				return HISTB_VC1_INVALID;
	VC1_ENTRY_GET(coded_size_flag, 1);
	if (entry->coded_size_flag) {
		VC1_ENTRY_GET(coded_width, 12);
		VC1_ENTRY_GET(coded_height, 12);
		entry->coded_width = (entry->coded_width + 1) * 2;
		entry->coded_height = (entry->coded_height + 1) * 2;
	} else {
		entry->coded_width = sequence->max_coded_width;
		entry->coded_height = sequence->max_coded_height;
	}
	if (!entry->coded_width || !entry->coded_height ||
	    entry->coded_width > sequence->max_coded_width ||
	    entry->coded_height > sequence->max_coded_height ||
	    (entry->coded_width + 15) / 16 > HISTB_VC1_MAX_MB_DIMENSION ||
	    (entry->coded_height + 15) / 16 > HISTB_VC1_MAX_MB_DIMENSION)
		return HISTB_VC1_UNSUPPORTED;
	if (entry->extended_mv)
		VC1_ENTRY_GET(extended_dmv, 1);
	VC1_ENTRY_GET(range_map_y_flag, 1);
	if (entry->range_map_y_flag)
		VC1_ENTRY_GET(range_map_y, 3);
	VC1_ENTRY_GET(range_map_uv_flag, 1);
	if (entry->range_map_uv_flag)
		VC1_ENTRY_GET(range_map_uv, 3);
#undef VC1_ENTRY_GET
	entry->header_bits = br.bit;
	*output = parsed;
	return HISTB_VC1_OK;
}

int histb_vc1_parse_entry_point(const __u8 *data, __u32 size,
				 const struct histb_vc1_sequence *sequence,
				 struct histb_vc1_entry_point *output)
{
	struct histb_vc1_parse_view view;

	histb_vc1_identity_view(data, size, &view);
	return histb_vc1_parse_entry_point_view(&view, sequence, output);
}

static int vc1_parse_panscan(struct vc1_bitreader *br,
			      const struct histb_vc1_sequence *sequence,
			      const struct histb_vc1_entry_point *entry,
			      struct histb_vc1_parsed_picture *picture)
{
	__u32 present, windows = 1;

	if (sequence->interlace)
		windows = sequence->broadcast ? picture->repeat_first_field + 2 : 2;
	else if (sequence->broadcast)
		windows = picture->repeat_frame + 1;
	picture->pan_scan_windows = windows;
	if (!entry->panscan_flag)
		return HISTB_VC1_OK;
	if (vc1_get_bits(br, 1, &present))
		return HISTB_VC1_INVALID;
	picture->pan_scan_present = present;
	if (present && vc1_skip_bits(br, windows * 64))
		return HISTB_VC1_INVALID;
	return HISTB_VC1_OK;
}

static int vc1_prepare_bpd(struct vc1_bitreader *br,
			    const struct histb_vc1_parse_view *view,
			    __u32 payload_dma,
			    const struct histb_vc1_sequence *sequence,
			    const struct histb_vc1_entry_point *entry,
			    const struct histb_vc1_bpd_layout *layout,
			    struct histb_vc1_parsed_picture *picture)
{
	struct histb_vc1_bpd_config *config = &picture->bpd_config;
	__u32 raw_bit, byte_offset;
	unsigned int i;
	int ret;

	ret = histb_vc1_u_to_raw_bit(view, br->bit, &raw_bit);
	if (ret)
		return ret;
	byte_offset = raw_bit >> 3;
	if (byte_offset > ~payload_dma)
		return HISTB_VC1_INVALID;
	config->stream_addr = payload_dma + byte_offset;
	config->bit_offset = raw_bit & 7;
	config->mb_width = (entry->coded_width + 15) / 16;
	config->mb_height = (entry->coded_height + 15) / 16;
	if (picture->fcm == HISTB_VC1_FIELD_INTERLACED)
		config->mb_height = (config->mb_height + 1) / 2;
	config->ptype = picture->ptype;
	config->picture_structure = picture->fcm;
	config->profile = sequence->profile;
	/* MVTYPE is present only in P pictures.  I pictures do not parse MVMODE,
	 * so their zero-initialised field must not enable the plane.
	 */
	config->mv_mode_enable = (picture->ptype == HISTB_VC1_PICTURE_P &&
		(picture->fcm == HISTB_VC1_FRAME_INTERLACED ||
		 (picture->fcm == HISTB_VC1_PROGRESSIVE &&
		  (picture->mv_mode == HISTB_VC1_MV_MIXED ||
		   (picture->mv_mode == HISTB_VC1_MV_INTENSITY_COMP &&
		    picture->mv_mode2 == HISTB_VC1_MV_MIXED))))) ||
		(picture->ptype == HISTB_VC1_PICTURE_B &&
		 picture->fcm == HISTB_VC1_FIELD_INTERLACED &&
		 picture->mv_mode == HISTB_VC1_MV_MIXED);
	/* Matching BPD_CfgReg sets CFG0[13] from OVERLAP && PQUANT <= 8. */
	config->overflags_enable = entry->overlap && picture->pquant <= 8;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		config->bitplane_addr[i] = layout->plane_addr[i];
	ret = histb_vc1_build_bpd_regs(config, &picture->bpd_regs);
	if (ret)
		return ret;
	picture->header_u_bits = br->bit;
	picture->header_raw_bits = raw_bit;
	picture->bpd_start_u_bits = br->bit;
	picture->bpd_start_raw_bits = raw_bit;
	picture->available_raw_bits = view->raw_size * 8U - raw_bit;
	return HISTB_VC1_NEEDS_BPD;
}

static int vc1_parse_progressive_picture_view_internal(
			const struct histb_vc1_parse_view *view,
			__u32 start_u_bit, __u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_picture *output,
			int allow_b)
{
	static const __u8 implicit_pquant[32] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 6, 7, 8, 9, 10, 11, 12,
		13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
		25, 27, 29, 31,
	};
	static const __u8 mv_mode[2][5] = {
		{ 3, 1, 2, 4, 0 }, /* PQUANT > 12, LC1 + unary ordering */
		{ 1, 0, 2, 4, 3 }, /* PQUANT <= 12, LC2 + unary ordering */
	};
	static const __u8 mv_mode2[2][4] = {
		{ 3, 1, 2, 0 }, /* PQUANT > 12, LC4 */
		{ 1, 0, 2, 3 }, /* PQUANT <= 12, LC5 */
	};
	struct histb_vc1_parsed_picture parsed;
	struct histb_vc1_parsed_picture *picture = &parsed;
	struct vc1_bitreader br;
	struct histb_vc1_bfraction fraction;
	__u32 value, ptype_code, mode_code;
	__u8 fraction_prefix, fraction_suffix = 0;
	unsigned int lowquant;
	int ret;

	if (!vc1_view_valid(view) || view->unescaped_size > (~0U >> 3) ||
	    !payload_dma || !sequence || !entry || !layout || !output ||
	    start_u_bit >= view->unescaped_size * 8U ||
	    sequence->profile != HISTB_VC1_PROFILE_ADVANCED)
		return HISTB_VC1_INVALID;
	br.data = view->unescaped;
	br.size = view->unescaped_size;
	br.bit = start_u_bit;
	vc1_zero(picture, sizeof(*picture));
	if (sequence->interlace) {
		if (vc1_decode012(&br, &value))
			return HISTB_VC1_INVALID;
		switch (value) {
		case 0:
			picture->fcm = HISTB_VC1_PROGRESSIVE;
			break;
		case 1:
			picture->fcm = HISTB_VC1_FRAME_INTERLACED;
			break;
		case 2:
			picture->fcm = HISTB_VC1_FIELD_INTERLACED;
			break;
		default:
			return HISTB_VC1_INVALID;
		}
		if (picture->fcm == HISTB_VC1_FIELD_INTERLACED)
			return HISTB_VC1_UNSUPPORTED;
	}
	if (vc1_get_unary(&br, 0, 4, &ptype_code))
		return HISTB_VC1_INVALID;
	switch (ptype_code) {
	case 0:
		picture->ptype = HISTB_VC1_PICTURE_P;
		break;
	case 1:
		picture->ptype = HISTB_VC1_PICTURE_B;
		break;
	case 2:
		picture->ptype = HISTB_VC1_PICTURE_I;
		break;
	case 3:
		picture->ptype = HISTB_VC1_PICTURE_BI_HW;
		break;
	case 4:
		picture->ptype = HISTB_VC1_PICTURE_P;
		picture->skipped = 1;
		break;
	default:
		return HISTB_VC1_INVALID;
	}
	if (picture->fcm == HISTB_VC1_FRAME_INTERLACED &&
	    picture->ptype == HISTB_VC1_PICTURE_BI_HW)
		return HISTB_VC1_UNSUPPORTED;
	if (sequence->tfcntr_flag && vc1_skip_bits(&br, 8))
		return HISTB_VC1_INVALID;
	if (sequence->broadcast) {
		if (sequence->interlace) {
			if (vc1_get_bits(&br, 1, &value))
				return HISTB_VC1_INVALID;
			picture->top_field_first = value;
			if (vc1_get_bits(&br, 1, &value))
				return HISTB_VC1_INVALID;
			picture->repeat_first_field = value;
		} else {
			if (vc1_get_bits(&br, 2, &value))
				return HISTB_VC1_INVALID;
			picture->repeat_frame = value;
		}
	} else {
		picture->top_field_first = 1;
	}
	ret = vc1_parse_panscan(&br, sequence, entry, picture);
	if (ret)
		return ret;
	if (picture->skipped) {
		if (picture->fcm != HISTB_VC1_PROGRESSIVE)
			return HISTB_VC1_UNSUPPORTED;
		ret = histb_vc1_u_to_raw_bit(view, br.bit,
					      &picture->header_raw_bits);
		if (ret)
			return ret;
		picture->header_u_bits = br.bit;
		*output = parsed;
		return HISTB_VC1_OK;
	}
	if (vc1_get_bits(&br, 1, &value))
		return HISTB_VC1_INVALID;
	picture->rounding_control = value;
	if (sequence->interlace) {
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->uv_samp = value;
	}
	if (sequence->finterp_flag) {
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->interp_frame = value;
	}
	if (picture->ptype == HISTB_VC1_PICTURE_B &&
	    picture->fcm != HISTB_VC1_FRAME_INTERLACED) {
		if (vc1_get_bits(&br, 3, &value))
			return HISTB_VC1_INVALID;
		fraction_prefix = value;
		if (fraction_prefix == 7) {
			if (vc1_get_bits(&br, 4, &value))
				return HISTB_VC1_INVALID;
			fraction_suffix = value;
		}
		ret = histb_vc1_decode_bfraction(fraction_prefix,
						 fraction_suffix, &fraction);
		if (ret)
			return ret;
		picture->bfraction_index = fraction.index;
		picture->bfraction = fraction.scale;
	} else if (picture->ptype == HISTB_VC1_PICTURE_BI_HW) {
		/* Direct BI is identified by PTYPE alone; BFRACTION is absent. */
		picture->bfraction_index = 0;
		picture->bfraction = 0;
	}
	/* The public I/P entry keeps B/BI behind an explicit lifecycle gate. */
	if (picture->ptype == HISTB_VC1_PICTURE_B ||
	    picture->ptype == HISTB_VC1_PICTURE_BI_HW) {
		if (!allow_b)
			return HISTB_VC1_UNSUPPORTED;
	}
	if (vc1_get_bits(&br, 5, &value) || !value)
		return HISTB_VC1_INVALID;
	picture->pqindex = value;
	picture->pquant = entry->quantizer_mode ? value : implicit_pquant[value];
	if (picture->pqindex < 9) {
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->halfqp = value;
	}
	switch (entry->quantizer_mode) {
	case 0:
		picture->pquantizer = picture->pqindex < 9;
		break;
	case 1:
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->pquantizer = value;
		break;
	case 2:
		picture->pquantizer = 0;
		break;
	case 3:
		picture->pquantizer = 1;
		break;
	}
	if (sequence->postproc_flag) {
		if (vc1_get_bits(&br, 2, &value))
			return HISTB_VC1_INVALID;
		picture->postproc = value;
	}
	/* Frame-interlaced B carries BFRACTION after the common quantizer
	 * fields.  Progressive B carries it before them. */
	if (picture->ptype == HISTB_VC1_PICTURE_B &&
	    picture->fcm == HISTB_VC1_FRAME_INTERLACED) {
		if (vc1_get_bits(&br, 3, &value))
			return HISTB_VC1_INVALID;
		fraction_prefix = value;
		if (fraction_prefix == 7) {
			if (vc1_get_bits(&br, 4, &value))
				return HISTB_VC1_INVALID;
			fraction_suffix = value;
		}
		ret = histb_vc1_decode_bfraction(fraction_prefix,
						 fraction_suffix, &fraction);
		if (ret)
			return ret;
		picture->bfraction_index = fraction.index;
		picture->bfraction = fraction.scale;
	}
	if (picture->ptype == HISTB_VC1_PICTURE_P) {
		if (entry->extended_mv) {
			if (vc1_get_unary(&br, 0, 3, &value))
				return HISTB_VC1_INVALID;
			picture->mv_range = value;
		}
		if (picture->fcm == HISTB_VC1_FRAME_INTERLACED) {
			if (entry->extended_dmv) {
				if (vc1_get_unary(&br, 0, 3, &value))
					return HISTB_VC1_INVALID;
				picture->dmv_range = value;
			}
			if (vc1_get_bits(&br, 1, &value))
				return HISTB_VC1_INVALID;
			picture->four_mv_switch = value;
			if (vc1_get_bits(&br, 1, &value))
				return HISTB_VC1_INVALID;
			picture->frame_intensity_comp = value;
			if (picture->frame_intensity_comp) {
				if (vc1_get_bits(&br, 6, &value))
					return HISTB_VC1_INVALID;
				picture->lum_scale = value;
				if (vc1_get_bits(&br, 6, &value))
					return HISTB_VC1_INVALID;
				picture->lum_shift = value;
			}
		} else {
			lowquant = picture->pquant <= 12;
			if (vc1_get_unary(&br, 1, 4, &mode_code))
				return HISTB_VC1_INVALID;
			picture->mv_mode = mv_mode[lowquant][mode_code];
			if (picture->mv_mode == HISTB_VC1_MV_INTENSITY_COMP) {
				if (vc1_get_unary(&br, 1, 3, &mode_code))
					return HISTB_VC1_INVALID;
				picture->mv_mode2 = mv_mode2[lowquant][mode_code];
				if (vc1_get_bits(&br, 6, &value))
					return HISTB_VC1_INVALID;
				picture->lum_scale = value;
				if (vc1_get_bits(&br, 6, &value))
					return HISTB_VC1_INVALID;
				picture->lum_shift = value;
			}
		}
	} else if (picture->ptype == HISTB_VC1_PICTURE_B) {
		if (entry->extended_mv) {
			if (vc1_get_unary(&br, 0, 3, &value))
				return HISTB_VC1_INVALID;
			picture->mv_range = value;
		}
		if (picture->fcm == HISTB_VC1_FRAME_INTERLACED) {
			if (entry->extended_dmv) {
				if (vc1_get_unary(&br, 0, 3, &value))
					return HISTB_VC1_INVALID;
				picture->dmv_range = value;
			}
			if (vc1_get_bits(&br, 1, &value))
				return HISTB_VC1_INVALID;
			picture->frame_intensity_comp = value;
			picture->mv_mode = HISTB_VC1_MV_1MV;
			picture->four_mv_switch = 0;
		} else {
			if (vc1_get_bits(&br, 1, &value))
				return HISTB_VC1_INVALID;
			picture->mv_mode = value ? HISTB_VC1_MV_1MV :
				HISTB_VC1_MV_1MV_HALFPEL_BILINEAR;
		}
	}
	ret = vc1_prepare_bpd(&br, view, payload_dma, sequence, entry, layout,
			      picture);
	if (ret == HISTB_VC1_NEEDS_BPD)
		*output = parsed;
	return ret;
}

int histb_vc1_parse_progressive_picture_view(
			const struct histb_vc1_parse_view *view,
			__u32 start_u_bit, __u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_picture *output)
{
	return vc1_parse_progressive_picture_view_internal(
		view, start_u_bit, payload_dma, sequence, entry, layout, output,
		0);
}

int histb_vc1_parse_progressive_b_picture_view(
			const struct histb_vc1_parse_view *view,
			__u32 start_u_bit, __u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_picture *output)
{
	return vc1_parse_progressive_picture_view_internal(
		view, start_u_bit, payload_dma, sequence, entry, layout, output,
		1);
}

int histb_vc1_parse_progressive_picture(
			const __u8 *data, __u32 size, __u32 start_bit,
			__u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_picture *output)
{
	struct histb_vc1_parse_view view;

	histb_vc1_identity_view(data, size, &view);
	return histb_vc1_parse_progressive_picture_view(
		&view, start_bit, payload_dma, sequence, entry, layout, output);
}

static int vc1_parse_progressive_slice_view_internal(
			const struct histb_vc1_parse_view *view,
			__u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_slice *output, int allow_b)
{
	struct histb_vc1_parsed_slice parsed;
	struct histb_vc1_parsed_slice *slice = &parsed;
	struct vc1_bitreader br;
	__u32 value, mb_height;
	int ret;

	if (!vc1_view_valid(view) || view->unescaped_size > (~0U >> 3) ||
	    !sequence || !entry || !layout || !output)
		return HISTB_VC1_INVALID;
	br.data = view->unescaped;
	br.size = view->unescaped_size;
	br.bit = 0;
	vc1_zero(slice, sizeof(*slice));
	if (vc1_get_bits(&br, 9, &value))
		return HISTB_VC1_INVALID;
	slice->address = value;
	mb_height = (entry->coded_height + 15) / 16;
	if (slice->address >= mb_height)
		return HISTB_VC1_INVALID;
	if (vc1_get_bits(&br, 1, &value))
		return HISTB_VC1_INVALID;
	slice->picture_header_flag = value;
	slice->header_u_bits = br.bit;
	ret = histb_vc1_u_to_raw_bit(view, br.bit, &slice->header_raw_bits);
	if (ret)
		return ret;
	if (!slice->picture_header_flag) {
		*output = parsed;
		return HISTB_VC1_OK;
	}
	if (allow_b)
		ret = histb_vc1_parse_progressive_b_picture_view(
			view, br.bit, payload_dma, sequence, entry, layout,
			&slice->picture);
	else
		ret = histb_vc1_parse_progressive_picture_view(
			view, br.bit, payload_dma, sequence, entry, layout,
			&slice->picture);
	if (ret == HISTB_VC1_OK || ret == HISTB_VC1_NEEDS_BPD) {
		slice->header_u_bits = slice->picture.header_u_bits;
		slice->header_raw_bits = slice->picture.header_raw_bits;
		*output = parsed;
	}
	return ret;
}

int histb_vc1_parse_progressive_slice_view(
			const struct histb_vc1_parse_view *view,
			__u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_slice *output)
{
	return vc1_parse_progressive_slice_view_internal(
		view, payload_dma, sequence, entry, layout, output, 0);
}

int histb_vc1_parse_progressive_b_slice_view(
			const struct histb_vc1_parse_view *view,
			__u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_slice *output)
{
	return vc1_parse_progressive_slice_view_internal(
		view, payload_dma, sequence, entry, layout, output, 1);
}

int histb_vc1_parse_progressive_slice(
			const __u8 *data, __u32 size, __u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_slice *output)
{
	struct histb_vc1_parse_view view;

	histb_vc1_identity_view(data, size, &view);
	return histb_vc1_parse_progressive_slice_view(
		&view, payload_dma, sequence, entry, layout, output);
}

static int vc1_parse_alt_quant(struct vc1_bitreader *br,
				struct histb_vc1_parsed_picture *picture)
{
	__u32 value, alt;

	picture->use_alt_qp = 1;
	if (vc1_get_bits(br, 3, &value))
		return HISTB_VC1_INVALID;
	picture->pqdiff = value;
	if (picture->pqdiff == 7) {
		if (vc1_get_bits(br, 5, &value))
			return HISTB_VC1_INVALID;
		picture->abs_pq = value;
		picture->altpquant = value;
		return HISTB_VC1_OK;
	}
	alt = picture->pquant + 1U + picture->pqdiff;
	if (alt > 31)
		return HISTB_VC1_INVALID;
	picture->altpquant = alt;
	return HISTB_VC1_OK;
}

static int vc1_parse_vop_dquant(struct vc1_bitreader *br, __u8 dquant,
					 struct histb_vc1_parsed_picture *picture)
{
	__u32 value;

	if (dquant == 2) {
		picture->dquant_frame = 1;
		picture->quant_mode = 1;
		return vc1_parse_alt_quant(br, picture);
	}
	if (vc1_get_bits(br, 1, &value))
		return HISTB_VC1_INVALID;
	picture->dquant_frame = value;
	if (!picture->dquant_frame)
		return HISTB_VC1_OK;
	if (vc1_get_bits(br, 2, &value))
		return HISTB_VC1_INVALID;
	picture->dqprofile = value;
	switch (picture->dqprofile) {
	case 0:
		picture->quant_mode = 1;
		break;
	case 1:
		if (vc1_get_bits(br, 2, &value))
			return HISTB_VC1_INVALID;
		picture->quant_mode = value + 2;
		break;
	case 2:
		if (vc1_get_bits(br, 2, &value))
			return HISTB_VC1_INVALID;
		picture->quant_mode = value + 6;
		break;
	case 3:
		if (vc1_get_bits(br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->dqbi_level = value;
		picture->quant_mode = value ? 10 : 11;
		if (!picture->dqbi_level)
			return HISTB_VC1_OK;
		break;
	}
	return vc1_parse_alt_quant(br, picture);
}

int histb_vc1_parse_smp_sequence(const __u8 *struct_c, __u32 size,
				 __u16 coded_width, __u16 coded_height,
				 struct histb_vc1_smp_sequence *output)
{
	struct histb_vc1_smp_sequence parsed;
	struct vc1_bitreader br;
	__u32 value, res_y411, res_sprite, res_x8, res_fasttx;
	__u32 res_transtab, res_rtm;

	if (!struct_c || size < 4 || !output || coded_width <= 31 ||
	    coded_height <= 31 ||
	    (coded_width + 15) / 16 > HISTB_VC1_MAX_MB_DIMENSION ||
	    (coded_height + 15) / 16 > HISTB_VC1_MAX_MB_DIMENSION)
		return HISTB_VC1_INVALID;
	br.data = struct_c;
	br.size = 4;
	br.bit = 0;
	vc1_zero(&parsed, sizeof(parsed));
#define VC1_SMP_SEQ_GET(member, count) \
	do { \
		if (vc1_get_bits(&br, count, &value)) \
			return HISTB_VC1_INVALID; \
		parsed.member = value; \
	} while (0)
	VC1_SMP_SEQ_GET(profile, 2);
	if (vc1_get_bits(&br, 1, &res_y411) ||
	    vc1_get_bits(&br, 1, &res_sprite))
		return HISTB_VC1_INVALID;
	if (parsed.profile > HISTB_VC1_PROFILE_MAIN || res_y411 || res_sprite)
		return HISTB_VC1_UNSUPPORTED;
	VC1_SMP_SEQ_GET(frame_rate_quant, 3);
	VC1_SMP_SEQ_GET(bit_rate_quant, 5);
	VC1_SMP_SEQ_GET(loopfilter, 1);
	if (vc1_get_bits(&br, 1, &res_x8))
		return HISTB_VC1_INVALID;
	VC1_SMP_SEQ_GET(multires, 1);
	if (vc1_get_bits(&br, 1, &res_fasttx))
		return HISTB_VC1_INVALID;
	VC1_SMP_SEQ_GET(fast_uv_mc, 1);
	VC1_SMP_SEQ_GET(extended_mv, 1);
	VC1_SMP_SEQ_GET(dquant, 2);
	VC1_SMP_SEQ_GET(variable_transform, 1);
	if (vc1_get_bits(&br, 1, &res_transtab))
		return HISTB_VC1_INVALID;
	VC1_SMP_SEQ_GET(overlap, 1);
	VC1_SMP_SEQ_GET(sync_marker, 1);
	VC1_SMP_SEQ_GET(range_reduction, 1);
	VC1_SMP_SEQ_GET(max_b_frames, 3);
	VC1_SMP_SEQ_GET(quantizer_mode, 2);
	VC1_SMP_SEQ_GET(finterp_flag, 1);
	if (vc1_get_bits(&br, 1, &res_rtm))
		return HISTB_VC1_INVALID;
#undef VC1_SMP_SEQ_GET
	/* CV200 handles Simple RES_RTM=0 internally.  Main has no exposed
	 * message field for the legacy rounding semantics, so keep it gated. */
	if (res_x8 || !res_fasttx || res_transtab ||
	    (parsed.profile == HISTB_VC1_PROFILE_MAIN && !res_rtm))
		return HISTB_VC1_UNSUPPORTED;
	if (parsed.profile == HISTB_VC1_PROFILE_SIMPLE &&
	    (parsed.loopfilter || !parsed.fast_uv_mc || parsed.extended_mv ||
	     parsed.range_reduction || parsed.max_b_frames))
		return HISTB_VC1_UNSUPPORTED;
	parsed.coded_width = coded_width;
	parsed.coded_height = coded_height;
	parsed.header_bits = br.bit;
	*output = parsed;
	return HISTB_VC1_OK;
}

int histb_vc1_smp_respic_dimensions(
				 const struct histb_vc1_smp_sequence *sequence,
				 __u8 res_pic, __u16 *coded_width,
				 __u16 *coded_height)
{
	__u32 mb_width, mb_height;

	if (!sequence || !coded_width || !coded_height || res_pic > 3 ||
	    (!sequence->multires && res_pic))
		return HISTB_VC1_INVALID;
	mb_width = (sequence->coded_width + 15U) / 16U;
	mb_height = (sequence->coded_height + 15U) / 16U;
	if (res_pic == 1 || res_pic == 3)
		mb_width = (mb_width + 1) / 2;
	if (res_pic == 2 || res_pic == 3)
		mb_height = (mb_height + 1) / 2;
	if (mb_width < 3 || mb_height < 3 ||
	    mb_width > HISTB_VC1_MAX_MB_DIMENSION ||
	    mb_height > HISTB_VC1_MAX_MB_DIMENSION)
		return HISTB_VC1_UNSUPPORTED;
	*coded_width = mb_width * 16U;
	*coded_height = mb_height * 16U;
	return HISTB_VC1_OK;
}

static int vc1_parse_smp_bfraction(struct vc1_bitreader *br,
					   struct histb_vc1_parsed_picture *picture)
{
	struct histb_vc1_bfraction fraction;
	__u32 prefix, suffix = 0;
	int ret;

	if (vc1_get_bits(br, 3, &prefix))
		return HISTB_VC1_INVALID;
	if (prefix == 7 && vc1_get_bits(br, 4, &suffix))
		return HISTB_VC1_INVALID;
	if (prefix == 7 && suffix == 14)
		return HISTB_VC1_INVALID;
	if (prefix == 7 && suffix == 15) {
		picture->ptype = HISTB_VC1_PICTURE_BI_HW;
		picture->bfraction_index = 22;
		picture->bfraction = 0;
		return HISTB_VC1_OK;
	}
	ret = histb_vc1_decode_bfraction(prefix, suffix, &fraction);
	if (ret)
		return ret;
	picture->bfraction_index = fraction.index;
	picture->bfraction = fraction.scale;
	return HISTB_VC1_OK;
}

static int vc1_parse_smp_quant(struct vc1_bitreader *br,
			       const struct histb_vc1_smp_sequence *sequence,
			       struct histb_vc1_parsed_picture *picture)
{
	static const __u8 implicit_pquant[32] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 6, 7, 8, 9, 10, 11, 12,
		13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
		25, 27, 29, 31,
	};
	__u32 value;

	if (vc1_get_bits(br, 5, &value) || !value)
		return HISTB_VC1_INVALID;
	picture->pqindex = value;
	picture->pquant = sequence->quantizer_mode ? value : implicit_pquant[value];
	if (picture->pqindex < 9) {
		if (vc1_get_bits(br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->halfqp = value;
	}
	switch (sequence->quantizer_mode) {
	case 0:
		picture->pquantizer = picture->pqindex < 9;
		break;
	case 1:
		if (vc1_get_bits(br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->pquantizer = value;
		break;
	case 2:
		picture->pquantizer = 0;
		break;
	case 3:
		picture->pquantizer = 1;
		break;
	default:
		return HISTB_VC1_INVALID;
	}
	if (sequence->extended_mv) {
		if (vc1_get_unary(br, 0, 3, &value))
			return HISTB_VC1_INVALID;
		picture->mv_range = value;
	}
	if (sequence->multires &&
	    (picture->ptype == HISTB_VC1_PICTURE_I ||
	     picture->ptype == HISTB_VC1_PICTURE_P)) {
		if (vc1_get_bits(br, 2, &value))
			return HISTB_VC1_INVALID;
		picture->res_pic = value;
	}
	return histb_vc1_smp_respic_dimensions(sequence, picture->res_pic,
						 &picture->coded_width,
						 &picture->coded_height);
}

static int vc1_prepare_smp_bpd(
				struct vc1_bitreader *br, __u32 payload_dma,
				const struct histb_vc1_smp_sequence *sequence,
				const struct histb_vc1_bpd_layout *layout,
				struct histb_vc1_parsed_picture *picture)
{
	struct histb_vc1_bpd_config *config = &picture->bpd_config;
	__u32 byte_offset = br->bit >> 3;
	unsigned int i;
	int ret;

	if (byte_offset > ~payload_dma)
		return HISTB_VC1_INVALID;
	config->stream_addr = payload_dma + byte_offset;
	config->bit_offset = br->bit & 7;
	/* Bitplanes describe the current RESPIC-coded picture, not the larger
	 * display canvas retained by the reference surfaces. */
	config->mb_width = (picture->coded_width + 15) / 16;
	config->mb_height = (picture->coded_height + 15) / 16;
	config->ptype = picture->ptype;
	config->picture_structure = HISTB_VC1_PROGRESSIVE;
	config->profile = sequence->profile;
	config->mv_mode_enable = picture->ptype == HISTB_VC1_PICTURE_P &&
		(picture->mv_mode == HISTB_VC1_MV_MIXED ||
		 (picture->mv_mode == HISTB_VC1_MV_INTENSITY_COMP &&
		  picture->mv_mode2 == HISTB_VC1_MV_MIXED));
	config->overflags_enable = 0;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		config->bitplane_addr[i] = layout->plane_addr[i];
	ret = histb_vc1_build_bpd_regs(config, &picture->bpd_regs);
	if (ret)
		return ret;
	picture->bpd_start_u_bits = br->bit;
	picture->bpd_start_raw_bits = br->bit;
	picture->header_u_bits = br->bit;
	picture->header_raw_bits = br->bit;
	picture->available_raw_bits = br->size * 8U - br->bit;
	return HISTB_VC1_NEEDS_BPD;
}

int histb_vc1_parse_smp_picture(
				const __u8 *data, __u32 size, __u32 payload_dma,
				const struct histb_vc1_smp_sequence *sequence,
				__u8 committed_rounding, __u8 committed_res_pic,
				const struct histb_vc1_bpd_layout *layout,
				struct histb_vc1_parsed_picture *output)
{
	static const __u8 mv_mode[2][5] = {
		{ 3, 1, 2, 4, 0 },
		{ 1, 0, 2, 4, 3 },
	};
	static const __u8 mv_mode2[2][4] = {
		{ 3, 1, 2, 0 },
		{ 1, 0, 2, 3 },
	};
	struct histb_vc1_parsed_picture parsed;
	struct vc1_bitreader br;
	__u32 value, mode_code;
	unsigned int lowquant;
	int ret;

	if (!data || !size || size > (~0U >> 3) || !payload_dma || !sequence ||
	    !layout || !output || !vc1_bool(committed_rounding) ||
	    committed_res_pic > 3 ||
	    sequence->profile > HISTB_VC1_PROFILE_MAIN ||
	    sequence->header_bits != 32 ||
	    (!sequence->multires && committed_res_pic))
		return HISTB_VC1_INVALID;
	br.data = data;
	br.size = size;
	br.bit = 0;
	vc1_zero(&parsed, sizeof(parsed));
	parsed.fcm = HISTB_VC1_PROGRESSIVE;
	parsed.res_pic = committed_res_pic;
	if (sequence->finterp_flag) {
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		parsed.interp_frame = value;
	}
	if (vc1_skip_bits(&br, 2))
		return HISTB_VC1_INVALID;
	if (sequence->range_reduction) {
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		parsed.range_reduction = value;
	}
	if (vc1_get_bits(&br, 1, &value))
		return HISTB_VC1_INVALID;
	if (value) {
		parsed.ptype = HISTB_VC1_PICTURE_P;
	} else if (!sequence->max_b_frames) {
		parsed.ptype = HISTB_VC1_PICTURE_I;
	} else {
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		parsed.ptype = value ? HISTB_VC1_PICTURE_I :
			HISTB_VC1_PICTURE_B;
	}
	if (parsed.ptype == HISTB_VC1_PICTURE_B) {
		ret = vc1_parse_smp_bfraction(&br, &parsed);
		if (ret)
			return ret;
	}
	if ((parsed.ptype == HISTB_VC1_PICTURE_I ||
	     parsed.ptype == HISTB_VC1_PICTURE_BI_HW) &&
	    vc1_skip_bits(&br, 7))
		return HISTB_VC1_INVALID;
	if (parsed.ptype == HISTB_VC1_PICTURE_I ||
	    parsed.ptype == HISTB_VC1_PICTURE_BI_HW)
		parsed.rounding_control = 1;
	else if (parsed.ptype == HISTB_VC1_PICTURE_P)
		parsed.rounding_control = committed_rounding ^ 1;
	else
		parsed.rounding_control = committed_rounding;
	ret = vc1_parse_smp_quant(&br, sequence, &parsed);
	if (ret)
		return ret;
	/* ST 421 keeps every predictive picture in the resolution epoch selected
	 * by its preceding I picture.  A new I may select another RESPIC value. */
	if (parsed.ptype == HISTB_VC1_PICTURE_P &&
	    parsed.res_pic != committed_res_pic)
		return HISTB_VC1_INVALID;
	if (parsed.ptype == HISTB_VC1_PICTURE_I ||
	    parsed.ptype == HISTB_VC1_PICTURE_BI_HW) {
		if (vc1_decode012(&br, &value))
			return HISTB_VC1_INVALID;
		parsed.trans_ac_frame = value;
		if (vc1_decode012(&br, &value))
			return HISTB_VC1_INVALID;
		parsed.trans_ac_frame2 = value;
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		parsed.trans_dc_table = value;
		parsed.direct_mb_raw = 1;
		parsed.mvtype_mb_raw = 1;
		parsed.skip_mb_raw = 1;
		parsed.header_u_bits = br.bit;
		parsed.header_raw_bits = br.bit;
		parsed.complete = 1;
		*output = parsed;
		return HISTB_VC1_OK;
	}
	lowquant = parsed.pquant <= 12;
	if (parsed.ptype == HISTB_VC1_PICTURE_P) {
		if (vc1_get_unary(&br, 1, 4, &mode_code))
			return HISTB_VC1_INVALID;
		parsed.mv_mode = mv_mode[lowquant][mode_code];
		if (parsed.mv_mode == HISTB_VC1_MV_INTENSITY_COMP) {
			if (vc1_get_unary(&br, 1, 3, &mode_code))
				return HISTB_VC1_INVALID;
			parsed.mv_mode2 = mv_mode2[lowquant][mode_code];
			if (vc1_get_bits(&br, 6, &value))
				return HISTB_VC1_INVALID;
			parsed.lum_scale = value;
			if (vc1_get_bits(&br, 6, &value))
				return HISTB_VC1_INVALID;
			parsed.lum_shift = value;
		}
	} else {
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		parsed.mv_mode = value ? HISTB_VC1_MV_1MV :
			HISTB_VC1_MV_1MV_HALFPEL_BILINEAR;
	}
	ret = vc1_prepare_smp_bpd(&br, payload_dma, sequence, layout, &parsed);
	if (ret == HISTB_VC1_NEEDS_BPD)
		*output = parsed;
	return ret;
}

int histb_vc1_resume_smp_picture(
				const __u8 *data, __u32 size,
				const struct histb_vc1_smp_sequence *sequence,
				const struct histb_vc1_bpd_result *bpd_result,
				struct histb_vc1_parsed_picture *picture)
{
	struct histb_vc1_parsed_picture next;
	struct vc1_bitreader br;
	__u32 value, end_bit;
	unsigned int i;
	int ret;

	if (!data || !size || size > (~0U >> 3) || !sequence || !bpd_result ||
	    !picture || picture->complete ||
	    (picture->ptype != HISTB_VC1_PICTURE_P &&
	     picture->ptype != HISTB_VC1_PICTURE_B) ||
	    picture->bpd_start_u_bits >= size * 8U ||
	    picture->bpd_start_raw_bits != picture->bpd_start_u_bits ||
	    picture->available_raw_bits != size * 8U - picture->bpd_start_u_bits ||
	    bpd_result->eaten_bits >= picture->available_raw_bits)
		return HISTB_VC1_INVALID;
	for (i = 0; i <= HISTB_VC1_BPD_DIRECTMB; i++)
		if (bpd_result->mode[i] > 7)
			return HISTB_VC1_INVALID;
	end_bit = picture->bpd_start_u_bits + bpd_result->eaten_bits;
	next = *picture;
	br.data = data;
	br.size = size;
	br.bit = end_bit;
	for (i = 0; i <= HISTB_VC1_BPD_DIRECTMB; i++)
		next.bpd_mode[i] = bpd_result->mode[i];
	next.mvtype_mb_raw = !bpd_result->mode[HISTB_VC1_BPD_MVTYPEMB];
	next.skip_mb_raw = !bpd_result->mode[HISTB_VC1_BPD_SKIPMB];
	next.direct_mb_raw = !bpd_result->mode[HISTB_VC1_BPD_DIRECTMB];
	if (vc1_get_bits(&br, 2, &value))
		return HISTB_VC1_INVALID;
	next.mv_table = value;
	if (vc1_get_bits(&br, 2, &value))
		return HISTB_VC1_INVALID;
	next.cbp_table = value;
	if (sequence->dquant) {
		ret = vc1_parse_vop_dquant(&br, sequence->dquant, &next);
		if (ret)
			return ret;
	}
	if (sequence->variable_transform) {
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		next.ttmbf = value;
		if (next.ttmbf) {
			if (vc1_get_bits(&br, 2, &value))
				return HISTB_VC1_INVALID;
			next.ttfrm = value;
		}
	}
	if (vc1_decode012(&br, &value))
		return HISTB_VC1_INVALID;
	next.trans_ac_frame = value;
	if (vc1_get_bits(&br, 1, &value))
		return HISTB_VC1_INVALID;
	next.trans_dc_table = value;
	next.header_u_bits = br.bit;
	next.header_raw_bits = br.bit;
	next.complete = 1;
	*picture = next;
	return HISTB_VC1_OK;
}

static int vc1_field_pair_types(__u8 fptype, __u8 *first, __u8 *second)
{
	if (!first || !second || fptype > 7)
		return HISTB_VC1_INVALID;
	if (fptype & 4) {
		*first = fptype & 2 ? HISTB_VC1_PICTURE_BI_HW :
			HISTB_VC1_PICTURE_B;
		*second = fptype & 1 ? HISTB_VC1_PICTURE_BI_HW :
			HISTB_VC1_PICTURE_B;
	} else {
		*first = fptype & 2 ? HISTB_VC1_PICTURE_P : HISTB_VC1_PICTURE_I;
		*second = fptype & 1 ? HISTB_VC1_PICTURE_P : HISTB_VC1_PICTURE_I;
	}
	return HISTB_VC1_OK;
}

static int vc1_parse_field_bfraction(struct vc1_bitreader *br,
				     struct histb_vc1_parsed_picture *picture)
{
	struct histb_vc1_bfraction fraction;
	__u32 value;
	__u8 prefix, suffix = 0;
	int ret;

	if (vc1_get_bits(br, 3, &value))
		return HISTB_VC1_INVALID;
	prefix = value;
	if (prefix == 7) {
		if (vc1_get_bits(br, 4, &value))
			return HISTB_VC1_INVALID;
		suffix = value;
	}
	ret = histb_vc1_decode_bfraction(prefix, suffix, &fraction);
	if (ret)
		return ret;
	picture->bfraction_index = fraction.index;
	picture->bfraction = fraction.scale;
	return HISTB_VC1_OK;
}

static int vc1_parse_field_quant(struct vc1_bitreader *br,
				 const struct histb_vc1_sequence *sequence,
				 const struct histb_vc1_entry_point *entry,
				 struct histb_vc1_parsed_picture *picture)
{
	static const __u8 implicit_pquant[32] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 6, 7, 8, 9, 10, 11, 12,
		13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
		25, 27, 29, 31,
	};
	__u32 value;

	if (vc1_get_bits(br, 5, &value) || !value)
		return HISTB_VC1_INVALID;
	picture->pqindex = value;
	picture->pquant = entry->quantizer_mode ? value : implicit_pquant[value];
	if (picture->pqindex < 9) {
		if (vc1_get_bits(br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->halfqp = value;
	}
	switch (entry->quantizer_mode) {
	case 0:
		picture->pquantizer = picture->pqindex < 9;
		break;
	case 1:
		if (vc1_get_bits(br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->pquantizer = value;
		break;
	case 2:
		picture->pquantizer = 0;
		break;
	case 3:
		picture->pquantizer = 1;
		break;
	default:
		return HISTB_VC1_INVALID;
	}
	if (sequence->postproc_flag) {
		if (vc1_get_bits(br, 2, &value))
			return HISTB_VC1_INVALID;
		picture->postproc = value;
	}
	return HISTB_VC1_OK;
}

static int vc1_parse_field_refdist(struct vc1_bitreader *br, __u8 *ref_dist)
{
	__u32 value, extension;

	if (vc1_get_bits(br, 2, &value))
		return HISTB_VC1_INVALID;
	if (value == 3) {
		if (vc1_get_unary(br, 0, 14, &extension))
			return HISTB_VC1_INVALID;
		value += extension;
	}
	if (value > 16)
		return HISTB_VC1_INVALID;
	*ref_dist = value;
	return HISTB_VC1_OK;
}

static int vc1_complete_field_p_picture(
			struct vc1_bitreader *br,
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			struct histb_vc1_parsed_picture *picture)
{
	static const __u8 mv_mode[2][5] = {
		{ 3, 1, 2, 4, 0 },
		{ 1, 0, 2, 4, 3 },
	};
	static const __u8 mv_mode2[2][4] = {
		{ 3, 1, 2, 0 },
		{ 1, 0, 2, 3 },
	};
	__u32 value, mode_code;
	unsigned int lowquant;
	int ret;

	if (vc1_get_bits(br, 1, &value))
		return HISTB_VC1_INVALID;
	picture->num_ref = value;
	if (!picture->num_ref) {
		if (vc1_get_bits(br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->ref_field = value;
	}
	if (entry->extended_mv) {
		if (vc1_get_unary(br, 0, 3, &value))
			return HISTB_VC1_INVALID;
		picture->mv_range = value;
	}
	if (entry->extended_dmv) {
		if (vc1_get_unary(br, 0, 3, &value))
			return HISTB_VC1_INVALID;
		picture->dmv_range = value;
	}
	lowquant = picture->pquant <= 12;
	if (vc1_get_unary(br, 1, 4, &mode_code))
		return HISTB_VC1_INVALID;
	picture->mv_mode = mv_mode[lowquant][mode_code];
	if (picture->mv_mode == HISTB_VC1_MV_INTENSITY_COMP) {
		if (vc1_get_unary(br, 1, 3, &mode_code))
			return HISTB_VC1_INVALID;
		picture->mv_mode2 = mv_mode2[lowquant][mode_code];
		if (vc1_get_bits(br, 1, &value))
			return HISTB_VC1_INVALID;
		if (value) {
			picture->intcomp_field = HISTB_VC1_INTENSITY_BOTH_FIELDS;
		} else {
			if (vc1_get_bits(br, 1, &value))
				return HISTB_VC1_INVALID;
			picture->intcomp_field = value ?
				HISTB_VC1_INTENSITY_BOTTOM_FIELD :
				HISTB_VC1_INTENSITY_TOP_FIELD;
		}
		picture->lum_scale = 32;
		picture->lum_shift = 0;
		picture->lum_scale2 = 32;
		picture->lum_shift2 = 0;
		if (vc1_get_bits(br, 6, &value))
			return HISTB_VC1_INVALID;
		picture->lum_scale = value;
		if (vc1_get_bits(br, 6, &value))
			return HISTB_VC1_INVALID;
		picture->lum_shift = value;
		if (picture->intcomp_field == HISTB_VC1_INTENSITY_BOTH_FIELDS) {
			if (vc1_get_bits(br, 6, &value))
				return HISTB_VC1_INVALID;
			picture->lum_scale2 = value;
			if (vc1_get_bits(br, 6, &value))
				return HISTB_VC1_INVALID;
			picture->lum_shift2 = value;
		}
	}
	if (vc1_get_bits(br, 3, &value))
		return HISTB_VC1_INVALID;
	picture->mb_mode_table = value;
	if (vc1_get_bits(br, 2 + picture->num_ref, &value))
		return HISTB_VC1_INVALID;
	picture->mv_table = value;
	if (vc1_get_bits(br, 3, &value))
		return HISTB_VC1_INVALID;
	picture->cbp_table = value;
	if (picture->mv_mode == HISTB_VC1_MV_MIXED ||
	    (picture->mv_mode == HISTB_VC1_MV_INTENSITY_COMP &&
	     picture->mv_mode2 == HISTB_VC1_MV_MIXED)) {
		if (vc1_get_bits(br, 2, &value))
			return HISTB_VC1_INVALID;
		picture->four_mv_bp_table = value;
	}
	if (entry->dquant) {
		ret = vc1_parse_vop_dquant(br, entry->dquant, picture);
		if (ret)
			return ret;
	}
	if (entry->variable_transform) {
		if (vc1_get_bits(br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->ttmbf = value;
		if (picture->ttmbf) {
			if (vc1_get_bits(br, 2, &value))
				return HISTB_VC1_INVALID;
			picture->ttfrm = value;
		}
	}
	if (vc1_decode012(br, &value))
		return HISTB_VC1_INVALID;
	picture->trans_ac_frame = value;
	if (vc1_get_bits(br, 1, &value))
		return HISTB_VC1_INVALID;
	picture->trans_dc_table = value;
	/* Field-P has no BPD pass.  The original syntax object leaves all seven
	 * plane modes at VC1_RAW (zero), and WritePicMsg converts each mode to a
	 * set D9 raw flag so the VDH consumes the per-MB syntax from the stream. */
	picture->forward_mb_raw = 1;
	picture->direct_mb_raw = 1;
	picture->mvtype_mb_raw = 1;
	picture->fieldtx_raw = 1;
	picture->skip_mb_raw = 1;
	picture->acpred_raw = 1;
	picture->overflags_raw = 1;
	ret = histb_vc1_u_to_raw_bit(view, br->bit, &picture->header_raw_bits);
	if (ret)
		return ret;
	picture->header_u_bits = br->bit;
	picture->complete = 1;
	return HISTB_VC1_OK;
}

static int vc1_prepare_field_b_picture(
			struct vc1_bitreader *br,
			const struct histb_vc1_parse_view *view,
			__u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_picture *picture)
{
	static const __u8 mv_mode2[2][4] = {
		{ 3, 1, 2, 0 },
		{ 1, 0, 2, 3 },
	};
	__u32 value, mode_code;
	unsigned int lowquant;

	picture->num_ref = 1;
	if (entry->extended_mv) {
		if (vc1_get_unary(br, 0, 3, &value))
			return HISTB_VC1_INVALID;
		picture->mv_range = value;
	}
	if (entry->extended_dmv) {
		if (vc1_get_unary(br, 0, 3, &value))
			return HISTB_VC1_INVALID;
		picture->dmv_range = value;
	}
	lowquant = picture->pquant <= 12;
	if (vc1_get_unary(br, 1, 3, &mode_code))
		return HISTB_VC1_INVALID;
	picture->mv_mode = mv_mode2[lowquant][mode_code];
	return vc1_prepare_bpd(br, view, payload_dma, sequence, entry, layout,
			       picture);
}

static int vc1_parse_field_child(
			struct vc1_bitreader *br,
			const struct histb_vc1_parse_view *view,
			__u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_picture *picture)
{
	int ret;

	ret = vc1_parse_field_quant(br, sequence, entry, picture);
	if (ret)
		return ret;
	if (picture->ptype == HISTB_VC1_PICTURE_I ||
	    picture->ptype == HISTB_VC1_PICTURE_BI_HW)
		return vc1_prepare_bpd(br, view, payload_dma, sequence, entry,
				       layout, picture);
	if (picture->ptype == HISTB_VC1_PICTURE_P)
		return vc1_complete_field_p_picture(br, view, entry, picture);
	if (picture->ptype == HISTB_VC1_PICTURE_B)
		return vc1_prepare_field_b_picture(br, view, payload_dma, sequence,
					   entry, layout, picture);
	return HISTB_VC1_INVALID;
}

void histb_vc1_field_transaction_reset(
			struct histb_vc1_field_transaction *transaction)
{
	if (transaction)
		vc1_zero(transaction, sizeof(*transaction));
}

int histb_vc1_parse_first_field_picture_view(
			const struct histb_vc1_parse_view *view,
			__u32 start_u_bit, __u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_field_transaction *transaction)
{
	struct histb_vc1_field_transaction next;
	struct histb_vc1_parsed_picture *picture = &next.field[0];
	struct vc1_bitreader br;
	__u32 value;
	int ret;

	if (!vc1_view_valid(view) || view->unescaped_size > (~0U >> 3) ||
	    !payload_dma || !sequence || !entry || !layout || !transaction ||
	    transaction->state != HISTB_VC1_FIELD_TXN_EMPTY ||
	    start_u_bit >= view->unescaped_size * 8U ||
	    sequence->profile != HISTB_VC1_PROFILE_ADVANCED ||
	    !sequence->interlace)
		return HISTB_VC1_INVALID;
	vc1_zero(&next, sizeof(next));
	br.data = view->unescaped;
	br.size = view->unescaped_size;
	br.bit = start_u_bit;
	if (vc1_decode012(&br, &value))
		return HISTB_VC1_INVALID;
	if (value != 2)
		return HISTB_VC1_UNSUPPORTED;
	picture->fcm = HISTB_VC1_FIELD_INTERLACED;
	if (vc1_get_bits(&br, 3, &value))
		return HISTB_VC1_INVALID;
	next.fptype = value;
	ret = vc1_field_pair_types(next.fptype, &next.first_ptype,
				   &next.second_ptype);
	if (ret)
		return ret;
	picture->fptype = next.fptype;
	picture->ptype = next.first_ptype;
	picture->field_index = 0;
	picture->is_second_field = 0;
	if (sequence->tfcntr_flag && vc1_skip_bits(&br, 8))
		return HISTB_VC1_INVALID;
	if (sequence->broadcast) {
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->top_field_first = value;
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->repeat_first_field = value;
	} else {
		picture->top_field_first = 1;
	}
	ret = vc1_parse_panscan(&br, sequence, entry, picture);
	if (ret)
		return ret;
	if (vc1_get_bits(&br, 1, &value))
		return HISTB_VC1_INVALID;
	picture->rounding_control = value;
	if (vc1_get_bits(&br, 1, &value))
		return HISTB_VC1_INVALID;
	picture->uv_samp = value;
	if (next.fptype & 4) {
		ret = vc1_parse_field_bfraction(&br, picture);
		if (ret)
			return ret;
	} else if (entry->refdist_flag) {
		ret = vc1_parse_field_refdist(&br, &picture->ref_dist);
		if (ret)
			return ret;
	}
	picture->current_parity = !(picture->top_field_first ^ 0);
	ret = vc1_parse_field_child(&br, view, payload_dma, sequence, entry,
				    layout, picture);
	if (ret != HISTB_VC1_OK && ret != HISTB_VC1_NEEDS_BPD)
		return ret;
	next.state = ret == HISTB_VC1_NEEDS_BPD ?
		HISTB_VC1_FIELD_TXN_FIRST_BPD :
		HISTB_VC1_FIELD_TXN_FIRST_COMPLETE;
	*transaction = next;
	return ret;
}

int histb_vc1_parse_second_field_picture_view(
			const struct histb_vc1_parse_view *view,
			__u32 start_u_bit, __u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_field_transaction *transaction)
{
	struct histb_vc1_field_transaction next;
	struct histb_vc1_parsed_picture *first, *picture;
	struct vc1_bitreader br;
	int ret;

	if (!vc1_view_valid(view) || view->unescaped_size > (~0U >> 3) ||
	    !payload_dma || !sequence || !entry || !layout || !transaction ||
	    transaction->state != HISTB_VC1_FIELD_TXN_FIRST_COMPLETE ||
	    start_u_bit >= view->unescaped_size * 8U ||
	    sequence->profile != HISTB_VC1_PROFILE_ADVANCED ||
	    !sequence->interlace || !transaction->field[0].complete ||
	    transaction->field[0].fcm != HISTB_VC1_FIELD_INTERLACED)
		return HISTB_VC1_INVALID;
	next = *transaction;
	first = &next.field[0];
	picture = &next.field[1];
	vc1_zero(picture, sizeof(*picture));
	picture->ptype = next.second_ptype;
	picture->fcm = HISTB_VC1_FIELD_INTERLACED;
	picture->fptype = next.fptype;
	picture->field_index = 1;
	picture->is_second_field = 1;
	picture->top_field_first = first->top_field_first;
	picture->repeat_first_field = first->repeat_first_field;
	picture->pan_scan_present = first->pan_scan_present;
	picture->pan_scan_windows = first->pan_scan_windows;
	picture->rounding_control = first->rounding_control;
	picture->uv_samp = first->uv_samp;
	picture->ref_dist = first->ref_dist;
	picture->bfraction_index = first->bfraction_index;
	picture->bfraction = first->bfraction;
	picture->current_parity = !(picture->top_field_first ^ 1);
	br.data = view->unescaped;
	br.size = view->unescaped_size;
	br.bit = start_u_bit;
	ret = vc1_parse_field_child(&br, view, payload_dma, sequence, entry,
				    layout, picture);
	if (ret != HISTB_VC1_OK && ret != HISTB_VC1_NEEDS_BPD)
		return ret;
	next.state = ret == HISTB_VC1_NEEDS_BPD ?
		HISTB_VC1_FIELD_TXN_SECOND_BPD : HISTB_VC1_FIELD_TXN_COMPLETE;
	*transaction = next;
	return ret;
}

int histb_vc1_resume_field_picture_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_field_transaction *transaction)
{
	struct histb_vc1_field_transaction next;
	struct histb_vc1_parsed_picture *picture;
	struct vc1_bitreader br;
	__u32 value, end_u_bit;
	unsigned int field_index, i;
	int ret;

	if (!vc1_view_valid(view) || view->unescaped_size > (~0U >> 3) ||
	    !entry || !bpd_result || !transaction ||
	    (transaction->state != HISTB_VC1_FIELD_TXN_FIRST_BPD &&
	     transaction->state != HISTB_VC1_FIELD_TXN_SECOND_BPD))
		return HISTB_VC1_INVALID;
	field_index = transaction->state == HISTB_VC1_FIELD_TXN_SECOND_BPD;
	picture = &transaction->field[field_index];
	if ((picture->ptype != HISTB_VC1_PICTURE_I &&
	     picture->ptype != HISTB_VC1_PICTURE_B &&
	     picture->ptype != HISTB_VC1_PICTURE_BI_HW) ||
	    picture->fcm != HISTB_VC1_FIELD_INTERLACED || picture->complete ||
	    picture->bpd_start_u_bits >= view->unescaped_size * 8U ||
	    picture->bpd_start_raw_bits >= view->raw_size * 8U ||
	    picture->available_raw_bits !=
		view->raw_size * 8U - picture->bpd_start_raw_bits ||
	    bpd_result->eaten_bits >= view->unescaped_size * 8U -
		picture->bpd_start_u_bits ||
	    bpd_result->condover > 3)
		return HISTB_VC1_INVALID;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		if (bpd_result->mode[i] > 7)
			return HISTB_VC1_INVALID;
	/* BPD skips Annex-G EPBs and reports semantic bits consumed. */
	end_u_bit = picture->bpd_start_u_bits + bpd_result->eaten_bits;

	next = *transaction;
	picture = &next.field[field_index];
	br.data = view->unescaped;
	br.size = view->unescaped_size;
	br.bit = end_u_bit;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		picture->bpd_mode[i] = bpd_result->mode[i];
	picture->condover = bpd_result->condover;
	picture->mvtype_mb_raw = !bpd_result->mode[HISTB_VC1_BPD_MVTYPEMB];
	picture->skip_mb_raw = !bpd_result->mode[HISTB_VC1_BPD_SKIPMB];
	picture->direct_mb_raw = !bpd_result->mode[HISTB_VC1_BPD_DIRECTMB];
	picture->acpred_raw = !bpd_result->mode[HISTB_VC1_BPD_ACPRED];
	picture->overflags_raw = !bpd_result->mode[HISTB_VC1_BPD_OVERFLAGS];
	picture->fieldtx_raw = !bpd_result->mode[HISTB_VC1_BPD_FIELDTX];
	picture->forward_mb_raw = !bpd_result->mode[HISTB_VC1_BPD_FORWARDMB];
	if (picture->ptype == HISTB_VC1_PICTURE_I ||
	    picture->ptype == HISTB_VC1_PICTURE_BI_HW) {
		if (vc1_decode012(&br, &value))
			return HISTB_VC1_INVALID;
		picture->trans_ac_frame = value;
		if (vc1_decode012(&br, &value))
			return HISTB_VC1_INVALID;
		picture->trans_ac_frame2 = value;
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->trans_dc_table = value;
		if (entry->dquant) {
			ret = vc1_parse_vop_dquant(&br, entry->dquant, picture);
			if (ret)
				return ret;
		}
	} else {
		if (vc1_get_bits(&br, 3, &value))
			return HISTB_VC1_INVALID;
		picture->mb_mode_table = value;
		if (vc1_get_bits(&br, 3, &value))
			return HISTB_VC1_INVALID;
		picture->mv_table = value;
		if (vc1_get_bits(&br, 3, &value))
			return HISTB_VC1_INVALID;
		picture->cbp_table = value;
		if (picture->mv_mode == HISTB_VC1_MV_MIXED) {
			if (vc1_get_bits(&br, 2, &value))
				return HISTB_VC1_INVALID;
			picture->four_mv_bp_table = value;
		}
		if (entry->dquant) {
			ret = vc1_parse_vop_dquant(&br, entry->dquant, picture);
			if (ret)
				return ret;
		}
		if (entry->variable_transform) {
			if (vc1_get_bits(&br, 1, &value))
				return HISTB_VC1_INVALID;
			picture->ttmbf = value;
			if (picture->ttmbf) {
				if (vc1_get_bits(&br, 2, &value))
					return HISTB_VC1_INVALID;
				picture->ttfrm = value;
			}
		}
		if (vc1_decode012(&br, &value))
			return HISTB_VC1_INVALID;
		picture->trans_ac_frame = value;
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		picture->trans_dc_table = value;
	}
	ret = histb_vc1_u_to_raw_bit(view, br.bit, &picture->header_raw_bits);
	if (ret)
		return ret;
	picture->header_u_bits = br.bit;
	picture->complete = 1;
	next.state = field_index ? HISTB_VC1_FIELD_TXN_COMPLETE :
		HISTB_VC1_FIELD_TXN_FIRST_COMPLETE;
	*transaction = next;
	return HISTB_VC1_OK;
}

int histb_vc1_parse_field_slice_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_field_transaction *transaction,
			__u8 field_index,
			struct histb_vc1_parsed_slice *output)
{
	struct histb_vc1_parsed_slice parsed;
	struct vc1_bitreader br;
	__u32 value, mb_height;
	int ret;

	if (!vc1_view_valid(view) || view->unescaped_size > (~0U >> 3) ||
	    !entry || !transaction || !output || field_index > 1 ||
	    !transaction->field[field_index].complete ||
	    transaction->field[field_index].fcm != HISTB_VC1_FIELD_INTERLACED)
		return HISTB_VC1_INVALID;
	br.data = view->unescaped;
	br.size = view->unescaped_size;
	br.bit = 0;
	vc1_zero(&parsed, sizeof(parsed));
	if (vc1_get_bits(&br, 9, &value))
		return HISTB_VC1_INVALID;
	parsed.address = value;
	mb_height = ((entry->coded_height + 15) / 16 + 1) / 2;
	if (parsed.address >= mb_height)
		return HISTB_VC1_INVALID;
	if (vc1_get_bits(&br, 1, &value))
		return HISTB_VC1_INVALID;
	parsed.picture_header_flag = value;
	if (parsed.picture_header_flag)
		return HISTB_VC1_UNSUPPORTED;
	parsed.header_u_bits = br.bit;
	ret = histb_vc1_u_to_raw_bit(view, br.bit, &parsed.header_raw_bits);
	if (ret)
		return ret;
	*output = parsed;
	return HISTB_VC1_OK;
}

static int vc1_resume_progressive_picture_view_internal(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_picture *picture,
			int allow_b)
{
	struct histb_vc1_parsed_picture next;
	struct vc1_bitreader br;
	__u32 value, end_u_bit;
	unsigned int i;
	int ret;

	if (!vc1_view_valid(view) || view->unescaped_size > (~0U >> 3) ||
	    !entry || !bpd_result ||
	    !picture || picture->skipped || picture->complete ||
	    (picture->ptype != HISTB_VC1_PICTURE_I &&
	     picture->ptype != HISTB_VC1_PICTURE_P &&
	     (!allow_b || (picture->ptype != HISTB_VC1_PICTURE_B &&
		    picture->ptype != HISTB_VC1_PICTURE_BI_HW))) ||
	    picture->bpd_start_u_bits >= view->unescaped_size * 8U ||
	    picture->bpd_start_raw_bits >= view->raw_size * 8U ||
	    picture->available_raw_bits !=
		view->raw_size * 8U - picture->bpd_start_raw_bits ||
	    bpd_result->eaten_bits >= view->unescaped_size * 8U -
		picture->bpd_start_u_bits ||
	    bpd_result->condover > 3)
		return HISTB_VC1_INVALID;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		if (bpd_result->mode[i] > 7)
			return HISTB_VC1_INVALID;
	/* BPD skips Annex-G EPBs and reports semantic bits consumed. */
	end_u_bit = picture->bpd_start_u_bits + bpd_result->eaten_bits;

	next = *picture;
	br.data = view->unescaped;
	br.size = view->unescaped_size;
	br.bit = end_u_bit;
	for (i = 0; i < HISTB_VC1_BPD_PLANES; i++)
		next.bpd_mode[i] = bpd_result->mode[i];
	next.condover = bpd_result->condover;
	next.mvtype_mb_raw = !bpd_result->mode[HISTB_VC1_BPD_MVTYPEMB];
	next.skip_mb_raw = !bpd_result->mode[HISTB_VC1_BPD_SKIPMB];
	next.direct_mb_raw = !bpd_result->mode[HISTB_VC1_BPD_DIRECTMB];
	next.acpred_raw = !bpd_result->mode[HISTB_VC1_BPD_ACPRED];
	next.overflags_raw = !bpd_result->mode[HISTB_VC1_BPD_OVERFLAGS];
	next.fieldtx_raw = !bpd_result->mode[HISTB_VC1_BPD_FIELDTX];
	next.forward_mb_raw = !bpd_result->mode[HISTB_VC1_BPD_FORWARDMB];

	if (next.ptype == HISTB_VC1_PICTURE_I ||
	    next.ptype == HISTB_VC1_PICTURE_BI_HW) {
		if (vc1_decode012(&br, &value))
			return HISTB_VC1_INVALID;
		next.trans_ac_frame = value;
		if (vc1_decode012(&br, &value))
			return HISTB_VC1_INVALID;
		next.trans_ac_frame2 = value;
		if (vc1_get_bits(&br, 1, &value))
			return HISTB_VC1_INVALID;
		next.trans_dc_table = value;
	} else {
		if (next.fcm == HISTB_VC1_FRAME_INTERLACED) {
			if (vc1_get_bits(&br, 2, &value))
				return HISTB_VC1_INVALID;
			next.mb_mode_table = value;
			if (vc1_get_bits(&br, 2, &value))
				return HISTB_VC1_INVALID;
			next.mv_table = value;
			if (vc1_get_bits(&br, 3, &value))
				return HISTB_VC1_INVALID;
			next.cbp_table = value;
			if (vc1_get_bits(&br, 2, &value))
				return HISTB_VC1_INVALID;
			next.two_mv_bp_table = value;
			if (next.four_mv_switch ||
			    next.ptype == HISTB_VC1_PICTURE_B) {
				if (vc1_get_bits(&br, 2, &value))
					return HISTB_VC1_INVALID;
				next.four_mv_bp_table = value;
			}
		} else {
			if (vc1_get_bits(&br, 2, &value))
				return HISTB_VC1_INVALID;
			next.mv_table = value;
			if (vc1_get_bits(&br, 2, &value))
				return HISTB_VC1_INVALID;
			next.cbp_table = value;
		}
	}
	if (entry->dquant) {
		ret = vc1_parse_vop_dquant(&br, entry->dquant, &next);
		if (ret)
			return ret;
	}
	if (next.ptype == HISTB_VC1_PICTURE_P ||
	    next.ptype == HISTB_VC1_PICTURE_B) {
		if (entry->variable_transform) {
			if (vc1_get_bits(&br, 1, &value))
				return HISTB_VC1_INVALID;
			next.ttmbf = value;
			if (next.ttmbf) {
				if (vc1_get_bits(&br, 2, &value))
					return HISTB_VC1_INVALID;
				next.ttfrm = value;
			}
		}
		if (next.ptype == HISTB_VC1_PICTURE_P ||
		    next.ptype == HISTB_VC1_PICTURE_B) {
			if (vc1_decode012(&br, &value))
				return HISTB_VC1_INVALID;
			next.trans_ac_frame = value;
			if (vc1_get_bits(&br, 1, &value))
				return HISTB_VC1_INVALID;
			next.trans_dc_table = value;
		}
	}
	ret = histb_vc1_u_to_raw_bit(view, br.bit, &next.header_raw_bits);
	if (ret)
		return ret;
	next.header_u_bits = br.bit;
	next.complete = 1;
	*picture = next;
	return HISTB_VC1_OK;
}

int histb_vc1_resume_progressive_picture_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_picture *picture)
{
	return vc1_resume_progressive_picture_view_internal(
		view, entry, bpd_result, picture, 0);
}

int histb_vc1_resume_progressive_b_picture_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_picture *picture)
{
	return vc1_resume_progressive_picture_view_internal(
		view, entry, bpd_result, picture, 1);
}

int histb_vc1_plan_display(struct histb_vc1_display_state *state,
			   __u8 ptype, __u8 first_anchor, __u8 draining,
			   struct histb_vc1_display_plan *plan)
{
	if (!state || !plan || ptype > HISTB_VC1_PICTURE_BI_HW ||
	    (ptype == HISTB_VC1_PICTURE_B && !state->pending) ||
	    (ptype == HISTB_VC1_PICTURE_BI_HW && !state->pending))
		return HISTB_VC1_INVALID;
	vc1_zero(plan, sizeof(*plan));

	if (ptype == HISTB_VC1_PICTURE_B ||
	    ptype == HISTB_VC1_PICTURE_BI_HW) {
		plan->release_current = 1;
		if (draining) {
			plan->release_pending = 1;
			plan->current_before_pending = 1;
			plan->pending_last = 1;
			state->pending = 0;
		}
		return HISTB_VC1_OK;
	}

	if (first_anchor) {
		if (draining) {
			plan->release_current = 1;
			plan->current_last = 1;
			state->pending = 0;
		} else {
			plan->hold_current = 1;
			state->pending = 1;
		}
		return HISTB_VC1_OK;
	}

	plan->release_pending = state->pending;
	if (draining) {
		plan->release_current = 1;
		plan->current_last = 1;
		state->pending = 0;
	} else {
		plan->hold_current = 1;
		state->pending = 1;
	}
	return HISTB_VC1_OK;
}

int histb_vc1_plan_drain(struct histb_vc1_display_state *state,
			 struct histb_vc1_display_plan *plan)
{
	if (!state || !plan)
		return HISTB_VC1_INVALID;
	vc1_zero(plan, sizeof(*plan));
	if (state->pending) {
		plan->release_pending = 1;
		plan->pending_last = 1;
		state->pending = 0;
	}
	return HISTB_VC1_OK;
}

int histb_vc1_resume_progressive_picture(
			const __u8 *data, __u32 size,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_picture *picture)
{
	struct histb_vc1_parse_view view;

	histb_vc1_identity_view(data, size, &view);
	return histb_vc1_resume_progressive_picture_view(
		&view, entry, bpd_result, picture);
}

static int vc1_resume_progressive_slice_view_internal(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_slice *slice, int allow_b)
{
	struct histb_vc1_parsed_slice next;
	int ret;

	if (!slice || !slice->picture_header_flag)
		return HISTB_VC1_INVALID;
	next = *slice;
	if (allow_b)
		ret = histb_vc1_resume_progressive_b_picture_view(
			view, entry, bpd_result, &next.picture);
	else
		ret = histb_vc1_resume_progressive_picture_view(
			view, entry, bpd_result, &next.picture);
	if (ret)
		return ret;
	next.header_u_bits = next.picture.header_u_bits;
	next.header_raw_bits = next.picture.header_raw_bits;
	*slice = next;
	return HISTB_VC1_OK;
}

int histb_vc1_resume_progressive_slice_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_slice *slice)
{
	return vc1_resume_progressive_slice_view_internal(
		view, entry, bpd_result, slice, 0);
}

int histb_vc1_resume_progressive_b_slice_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_slice *slice)
{
	return vc1_resume_progressive_slice_view_internal(
		view, entry, bpd_result, slice, 1);
}

int histb_vc1_resume_progressive_slice(
			const __u8 *data, __u32 size,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_slice *slice)
{
	struct histb_vc1_parse_view view;

	histb_vc1_identity_view(data, size, &view);
	return histb_vc1_resume_progressive_slice_view(
		&view, entry, bpd_result, slice);
}

int histb_vc1_finalize_slices(const struct histb_vc1_slice_part *parts,
			       __u32 count, __u16 mb_width, __u16 mb_height,
			       struct histb_vc1_slice *slices,
			       __u32 slice_capacity)
{
	__u32 total_mbs = (__u32)mb_width * mb_height;
	unsigned int i;

	if (!parts || !slices || !count || count > HISTB_VC1_MAX_SLICES ||
	    count > slice_capacity || mb_width < 3 ||
	    mb_width > HISTB_VC1_MAX_MB_DIMENSION || mb_height < 3 ||
	    mb_height > HISTB_VC1_MAX_MB_DIMENSION || parts[0].row)
		return HISTB_VC1_INVALID;
	for (i = 0; i < count; i++) {
		__u32 byte_offset, bit_len;

		if (parts[i].row >= mb_height ||
		    (i && parts[i].row <= parts[i - 1].row) ||
		    !parts[i].payload_dma ||
		    parts[i].data_raw_bit >= parts[i].payload_raw_bits)
			return HISTB_VC1_INVALID;
		byte_offset = parts[i].data_raw_bit >> 3;
		bit_len = parts[i].payload_raw_bits - parts[i].data_raw_bit;
		if (byte_offset > ~parts[i].payload_dma ||
		    bit_len > VC1_SLICE_LENGTH_MAX)
			return HISTB_VC1_INVALID;
	}

	vc1_zero(slices, count * sizeof(*slices));
	for (i = 0; i < count; i++) {
		slices[i].fragment[0].dma_addr = parts[i].payload_dma +
			(parts[i].data_raw_bit >> 3);
		slices[i].fragment[0].bit_offset = parts[i].data_raw_bit & 7;
		slices[i].fragment[0].bit_len = parts[i].payload_raw_bits -
			parts[i].data_raw_bit;
		slices[i].start_mb = (__u32)parts[i].row * mb_width;
		slices[i].end_mb = i + 1 < count ?
			(__u32)parts[i + 1].row * mb_width - 1 : total_mbs - 1;
	}
	return HISTB_VC1_OK;
}

int histb_vc1_validate_up_reports(__u32 state, __u32 smmu_state_secure,
				  __u32 smmu_state_nonsecure,
				  __u32 total_mbs,
				  const struct histb_vc1_up_report *reports,
				  __u32 report_capacity)
{
	__u32 report_count = state & 0x1ffffU;
	__u32 completion = state & ~0x1ffffU;
	__u32 covered = 0, previous_start = 0;
	unsigned int i;

	if (smmu_state_secure || smmu_state_nonsecure ||
	    (completion != HISTB_VC1_STATE_DECODE_DONE &&
	     completion != (HISTB_VC1_STATE_DECODE_DONE |
			    HISTB_VC1_STATE_DECODE_ERROR)))
		return HISTB_VC1_HW_ERROR;
	if (!total_mbs || !reports || !report_count ||
	    report_count > HISTB_VC1_MAX_UP_REPORTS ||
	    report_count > report_capacity)
		return HISTB_VC1_INVALID;
	for (i = 0; i < report_count; i++) {
		__u32 start = reports[i].d[1] & 0xffffU;
		__u32 end = reports[i].d[2] & 0xffffU;

		if (start > end || end >= total_mbs || start > covered ||
		    (i && start < previous_start) || end + 1 < covered)
			return HISTB_VC1_INVALID;
		previous_start = start;
		if (end >= covered)
			covered = end + 1;
	}
	return covered == total_mbs ? HISTB_VC1_OK : HISTB_VC1_INVALID;
}
