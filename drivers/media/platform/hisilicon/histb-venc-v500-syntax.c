// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hi3798MV310 V500 syntax, from venc_v2.0 drv_venc_efl.c's H265e_Make*
 * and H264e_MakeSlcHdr. Bitstream bytes and register words are deliberately
 * separate: the BSP's 32-bit pointer/endian assumptions do not apply here.
 */

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "histb-venc-v500-syntax.h"

struct v500_bits {
	u8 data[64];
	unsigned int count;
	bool overflow;
};

struct v500_annexb {
	u8 *data;
	size_t capacity;
	size_t count;
	bool overflow;
};

static bool v500_dimensions_valid(u32 width, u32 height)
{
	return width >= 8 && width <= 1920 && height >= 8 && height <= 1088 &&
	       !(width & 1) && !(height & 1);
}

static void v500_put(struct v500_bits *bits, u32 value, unsigned int count)
{
	int bit;

	if (bits->overflow)
		return;
	if (bits->count + count > sizeof(bits->data) * 8) {
		bits->overflow = true;
		return;
	}
	for (bit = (int)count - 1; bit >= 0; bit--) {
		bits->data[bits->count / 8] |=
			((value >> bit) & 1) << (7 - bits->count % 8);
		bits->count++;
	}
}

static void v500_ue(struct v500_bits *bits, u32 value)
{
	u32 code = value + 1;
	unsigned int count = fls(code);

	v500_put(bits, 0, count - 1);
	v500_put(bits, code, count);
}

static void v500_se(struct v500_bits *bits, s32 value)
{
	v500_ue(bits, value <= 0 ? -2 * value : 2 * value - 1);
}

static void v500_trailing(struct v500_bits *bits)
{
	v500_put(bits, 1, 1);
	v500_put(bits, 0, (8 - bits->count % 8) % 8);
}

static void v500_byte(struct v500_annexb *out, u8 value)
{
	if (out->count < out->capacity)
		out->data[out->count] = value;
	else
		out->overflow = true;
	out->count++;
}

static int v500_nal(struct v500_annexb *out, u8 type,
		    const struct v500_bits *rbsp)
{
	unsigned int i, zeros = 0;

	if (rbsp->overflow)
		return -ENOSPC;
	v500_byte(out, 0);
	v500_byte(out, 0);
	v500_byte(out, 0);
	v500_byte(out, 1);
	/* forbidden_zero_bit=0, nuh_layer_id=0, nuh_temporal_id_plus1=1. */
	v500_byte(out, type << 1);
	v500_byte(out, 1);
	for (i = 0; i < rbsp->count / 8; i++) {
		u8 value = rbsp->data[i];

		if (zeros == 2 && value <= 3) {
			v500_byte(out, 3);
			zeros = 0;
		}
		v500_byte(out, value);
		zeros = value ? 0 : zeros + 1;
	}
	return out->overflow ? -ENOSPC : 0;
}

static void v500_profile_tier_level(struct v500_bits *bits, bool progressive)
{
	v500_put(bits, 0, 2); /* general_profile_space */
	v500_put(bits, 0, 1); /* general_tier_flag: Main tier */
	v500_put(bits, 1, 5); /* general_profile_idc: Main (8 bit) */
	v500_put(bits, 0x60000000, 32); /* compatible with Main and Main10 */
	v500_put(bits, progressive, 1);
	v500_put(bits, 0, 1); /* general_interlaced_source_flag */
	v500_put(bits, 0, 1); /* general_non_packed_constraint_flag */
	v500_put(bits, 0, 1); /* general_frame_only_constraint_flag */
	v500_put(bits, 0, 32); /* reserved constraint bits */
	v500_put(bits, 0, 12);
	/* BSP writes zero here, which is not a defined HEVC level. */
	v500_put(bits, 153, 8); /* Main level 5.1 */
}

static void v500_vps(struct v500_bits *bits)
{
	v500_put(bits, 0, 4); /* vps_video_parameter_set_id */
	v500_put(bits, 3, 2); /* vps_base_layer_internal/available_flag */
	v500_put(bits, 0, 6); /* vps_max_layers_minus1 */
	v500_put(bits, 0, 3); /* vps_max_sub_layers_minus1 */
	v500_put(bits, 1, 1); /* vps_temporal_id_nesting_flag */
	v500_put(bits, 0xffff, 16);
	v500_profile_tier_level(bits, true);
	v500_put(bits, 1, 1); /* vps_sub_layer_ordering_info_present_flag */
	v500_ue(bits, 1); /* vps_max_dec_pic_buffering_minus1 */
	v500_ue(bits, 0); /* vps_max_num_reorder_pics */
	v500_ue(bits, 0); /* vps_max_latency_increase_plus1 */
	v500_put(bits, 0, 6); /* vps_max_layer_id */
	v500_ue(bits, 0); /* vps_num_layer_sets_minus1 */
	v500_put(bits, 0, 1); /* vps_timing_info_present_flag */
	v500_put(bits, 0, 1); /* vps_extension_flag */
	v500_trailing(bits);
}

static void v500_sps(struct v500_bits *bits, u32 width, u32 height)
{
	u32 coded_width = ALIGN(width, 8);
	u32 coded_height = ALIGN(height, 8);
	bool cropped = coded_width != width || coded_height != height;

	v500_put(bits, 0, 4); /* sps_video_parameter_set_id */
	v500_put(bits, 0, 3); /* sps_max_sub_layers_minus1 */
	v500_put(bits, 1, 1); /* sps_temporal_id_nesting_flag */
	v500_profile_tier_level(bits, false);
	v500_ue(bits, 0); /* sps_seq_parameter_set_id */
	v500_ue(bits, 1); /* chroma_format_idc: 4:2:0 */
	v500_ue(bits, coded_width);
	v500_ue(bits, coded_height);
	v500_put(bits, cropped, 1); /* conformance_window_flag */
	if (cropped) {
		v500_ue(bits, 0); /* conf_win_left_offset */
		v500_ue(bits, (coded_width - width) / 2);
		v500_ue(bits, 0); /* conf_win_top_offset */
		v500_ue(bits, (coded_height - height) / 2);
	}
	v500_ue(bits, 0); /* bit_depth_luma_minus8 */
	v500_ue(bits, 0); /* bit_depth_chroma_minus8 */
	v500_ue(bits, 12); /* log2_max_pic_order_cnt_lsb_minus4: 16-bit POC */
	v500_put(bits, 1, 1); /* sps_sub_layer_ordering_info_present_flag */
	v500_ue(bits, 1); /* sps_max_dec_pic_buffering_minus1 */
	v500_ue(bits, 0); /* sps_max_num_reorder_pics */
	v500_ue(bits, 0); /* sps_max_latency_increase_plus1 */
	v500_ue(bits, 0); /* log2_min_luma_coding_block_size_minus3: CU8 */
	v500_ue(bits, 2); /* V500: log2_diff_max_min_luma_coding_block_size */
	v500_ue(bits, 0); /* log2_min_luma_transform_block_size_minus2: TU4 */
	v500_ue(bits, 2); /* log2_diff_max_min_luma_transform_block_size: TU16 */
	v500_ue(bits, 0); /* max_transform_hierarchy_depth_inter */
	v500_ue(bits, 0); /* max_transform_hierarchy_depth_intra */
	v500_put(bits, 0, 1); /* scaling_list_enabled_flag */
	v500_put(bits, 0, 1); /* amp_enabled_flag */
	/* BSP emits literal 1, despite its unused bSAOEnabledFlag being zero. */
	v500_put(bits, 1, 1); /* sample_adaptive_offset_enabled_flag */
	v500_put(bits, 1, 1); /* pcm_enabled_flag */
	v500_put(bits, 7, 4); /* pcm_sample_bit_depth_luma_minus1 */
	v500_put(bits, 7, 4); /* pcm_sample_bit_depth_chroma_minus1 */
	v500_ue(bits, 0); /* log2_min_pcm_luma_coding_block_size_minus3 */
	v500_ue(bits, 0); /* log2_diff_max_min_pcm_luma_coding_block_size */
	v500_put(bits, 0, 1); /* pcm_loop_filter_disabled_flag */
	v500_ue(bits, 1); /* num_short_term_ref_pic_sets */
	v500_ue(bits, 1); /* num_negative_pics */
	v500_ue(bits, 0); /* num_positive_pics */
	v500_ue(bits, 0); /* delta_poc_s0_minus1: immediately previous picture */
	v500_put(bits, 1, 1); /* used_by_curr_pic_s0_flag */
	v500_put(bits, 0, 1); /* long_term_ref_pics_present_flag */
	v500_put(bits, 1, 1); /* sps_temporal_mvp_enabled_flag */
	v500_put(bits, 0, 1); /* strong_intra_smoothing_enabled_flag */
	v500_put(bits, 0, 1); /* vui_parameters_present_flag */
	v500_put(bits, 0, 1); /* sps_extension_present_flag */
	v500_trailing(bits);
}

static void v500_pps(struct v500_bits *bits)
{
	v500_ue(bits, 0); /* pps_pic_parameter_set_id */
	v500_ue(bits, 0); /* pps_seq_parameter_set_id */
	v500_put(bits, 0, 1); /* dependent_slice_segments_enabled_flag */
	v500_put(bits, 0, 1); /* output_flag_present_flag */
	v500_put(bits, 0, 3); /* num_extra_slice_header_bits */
	v500_put(bits, 0, 1); /* sign_data_hiding_enabled_flag */
	/* BSP emits literal 1: its cabac_init_present_flag=0 comment is stale. */
	v500_put(bits, 1, 1); /* cabac_init_present_flag */
	v500_ue(bits, 0); /* num_ref_idx_l0_default_active_minus1 */
	v500_ue(bits, 0); /* num_ref_idx_l1_default_active_minus1 */
	v500_se(bits, 0); /* init_qp_minus26 */
	v500_put(bits, 0, 1); /* constrained_intra_pred_flag */
	v500_put(bits, 0, 1); /* transform_skip_enabled_flag */
	v500_put(bits, 1, 1); /* cu_qp_delta_enabled_flag */
	v500_ue(bits, 1); /* V500: diff_cu_qp_delta_depth */
	v500_se(bits, 0); /* pps_cb_qp_offset */
	v500_se(bits, 0); /* pps_cr_qp_offset */
	v500_put(bits, 0, 1); /* pps_slice_chroma_qp_offsets_present_flag */
	v500_put(bits, 0, 1); /* weighted_pred_flag */
	v500_put(bits, 0, 1); /* weighted_bipred_flag */
	v500_put(bits, 0, 1); /* transquant_bypass_enabled_flag */
	v500_put(bits, 0, 1); /* tiles_enabled_flag */
	v500_put(bits, 0, 1); /* entropy_coding_sync_enabled_flag */
	v500_put(bits, 1, 1); /* pps_loop_filter_across_slices_enabled_flag */
	v500_put(bits, 1, 1); /* deblocking_filter_control_present_flag */
	v500_put(bits, 1, 1); /* deblocking_filter_override_enabled_flag */
	v500_put(bits, 1, 1); /* pps_deblocking_filter_disabled_flag */
	v500_put(bits, 0, 1); /* pps_scaling_list_data_present_flag */
	v500_put(bits, 0, 1); /* lists_modification_present_flag */
	v500_ue(bits, 0); /* log2_parallel_merge_level_minus2 */
	v500_put(bits, 0, 1); /* slice_segment_header_extension_present_flag */
	v500_put(bits, 0, 1); /* pps_extension_present_flag */
	v500_trailing(bits);
}

int histb_venc_v500_hevc_parameter_sets(u32 width, u32 height, u8 *dst,
				       size_t capacity, size_t *bytes)
{
	struct v500_annexb out = { .data = dst, .capacity = capacity };
	struct v500_bits bits = {};
	int ret;

	if (!bytes)
		return -EINVAL;
	*bytes = 0;
	if (!dst || !v500_dimensions_valid(width, height))
		return -EINVAL;

	v500_vps(&bits);
	ret = v500_nal(&out, 32, &bits);
	if (ret)
		return ret;
	memset(&bits, 0, sizeof(bits));
	v500_sps(&bits, width, height);
	ret = v500_nal(&out, 33, &bits);
	if (ret)
		return ret;
	memset(&bits, 0, sizeof(bits));
	v500_pps(&bits);
	ret = v500_nal(&out, 34, &bits);
	if (ret)
		return ret;
	*bytes = out.count;
	return 0;
}

static int v500_register_bits(const struct v500_bits *bits, u32 *words,
			      unsigned int word_count)
{
	unsigned int bit;

	if (bits->overflow || bits->count > word_count * 32)
		return -ENOSPC;
	for (bit = 0; bit < bits->count; bit++)
		words[bit / 32] = (words[bit / 32] << 1) |
			((bits->data[bit / 8] >> (7 - bit % 8)) & 1);
	return 0;
}

int histb_venc_v500_hevc_slice(const struct histb_venc_v500_picture *picture,
			     struct histb_venc_v500_slice *slice)
{
	struct v500_bits bits = {};
	int ret;

	if (!slice || !picture)
		return -EINVAL;
	memset(slice, 0, sizeof(*slice));
	if (!v500_dimensions_valid(picture->width, picture->height) ||
	    picture->qp > 51)
		return -EINVAL;

	/* first_slice_segment_in_pic_flag/address are generated by hardware. */
	if (picture->intra)
		v500_put(&bits, 0, 1); /* no_output_of_prior_pics_flag */
	v500_ue(&bits, 0); /* slice_pic_parameter_set_id */
	ret = v500_register_bits(&bits, &slice->part1, 1);
	if (ret)
		return ret;
	slice->part1_bits = bits.count;
	slice->slc_hdr_bits[0] = bits.count;

	memset(&bits, 0, sizeof(bits));
	v500_ue(&bits, picture->intra ? 2 : 1); /* slice_type: I or P */
	if (!picture->intra) {
		v500_put(&bits, picture->poc & 0xffff, 16);
		v500_put(&bits, 1, 1); /* short_term_ref_pic_set_sps_flag */
		v500_put(&bits, 1, 1); /* slice_temporal_mvp_enabled_flag */
	}
	v500_put(&bits, 1, 1); /* slice_sao_luma_flag */
	v500_put(&bits, 1, 1); /* slice_sao_chroma_flag */
	if (!picture->intra) {
		v500_put(&bits, 1, 1); /* num_ref_idx_active_override_flag */
		v500_ue(&bits, 0); /* num_ref_idx_l0_active_minus1 */
		v500_put(&bits, 0, 1); /* cabac_init_flag */
		/* P infers collocated_from_l0_flag=1; just one reference. */
		v500_ue(&bits, 3); /* five_minus_max_num_merge_cand: two candidates */
	}
	v500_se(&bits, (s32)picture->qp - 26); /* slice_qp_delta */
	/* Override PPS's disabled filter, exactly as MakeSliceHeader does. */
	v500_put(&bits, 1, 1); /* deblocking_filter_override_flag */
	v500_put(&bits, 0, 1); /* slice_deblocking_filter_disabled_flag */
	v500_se(&bits, 0); /* slice_beta_offset_div2 */
	v500_se(&bits, 0); /* slice_tc_offset_div2 */
	v500_put(&bits, 1, 1); /* slice_loop_filter_across_slices_enabled_flag */
	/* No trailing/alignment bits: V500 appends byte_alignment(). */
	ret = v500_register_bits(&bits, slice->cabac, ARRAY_SIZE(slice->cabac));
	if (ret)
		return ret;
	slice->slc_hdr_bits[1] = bits.count;
	return 0;
}

int histb_venc_v500_h264_slice(u32 frame_num, u8 idr_pic_id, bool intra,
			     struct histb_venc_v500_slice *slice)
{
	struct v500_bits bits = {};
	int ret;

	if (!slice)
		return -EINVAL;
	memset(slice, 0, sizeof(*slice));
	v500_ue(&bits, intra ? 2 : 0); /* slice_type: I or P */
	v500_ue(&bits, 0); /* pic_parameter_set_id */
	v500_put(&bits, intra ? 0 : frame_num & 0xf, 4);
	if (intra)
		v500_ue(&bits, idr_pic_id & 0xf);
	else
		v500_put(&bits, 0, 1); /* num_ref_idx_active_override_flag */
	ret = v500_register_bits(&bits, slice->slc_hdr_stream,
				 ARRAY_SIZE(slice->slc_hdr_stream));
	if (ret)
		return ret;
	/* Reorder flag=0; IDR marking=00, P adaptive marking flag=0. */
	slice->slc_hdr_bits[0] = bits.count | (1U << 8) |
				 ((intra ? 2U : 1U) << 16);
	return 0;
}
