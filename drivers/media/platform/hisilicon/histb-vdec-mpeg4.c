// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hi3798CV200 MPEG-4 Part 2 syntax and HiVDHV4R3C1 message contract.
 *
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 *
 * The syntax fields and message layout follow the vendor VFMW MP4_DEC_PARAM_S
 * and
 * MP4HAL_V4R3C1_{CfgDnMsg,WriteSlicMsg} contract.  Features whose parser-to-
 * hardware path is not closed return HISTB_MPEG4_UNSUPPORTED.
 */

#include "histb-vdec-mpeg4.h"

#define HISTB_MPEG4_ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

#define MPEG4_START_VOS		0xb0
#define MPEG4_START_VOS_END	0xb1
#define MPEG4_START_USER_DATA	0xb2
#define MPEG4_START_VOP		0xb6
#define MPEG4_START_VOL_MIN	0x20
#define MPEG4_START_VOL_MAX	0x2f

struct histb_mpeg4_bits {
	const __u8 *data;
	__u32 size_bits;
	__u32 pos;
	int error;
};

static const __u8 mpeg4_default_intra[HISTB_MPEG4_MATRIX_SIZE] = {
	8, 17, 18, 19, 21, 23, 25, 27,
	17, 18, 19, 21, 23, 25, 27, 28,
	20, 21, 22, 23, 24, 26, 28, 30,
	21, 22, 23, 24, 26, 28, 30, 32,
	22, 23, 24, 26, 28, 30, 32, 35,
	23, 24, 26, 28, 30, 32, 35, 38,
	25, 26, 28, 30, 32, 35, 38, 41,
	27, 28, 30, 32, 35, 38, 41, 45,
};

static const __u8 mpeg4_default_nonintra[HISTB_MPEG4_MATRIX_SIZE] = {
	16, 17, 18, 19, 20, 21, 22, 23,
	17, 18, 19, 20, 21, 22, 23, 24,
	18, 19, 20, 21, 22, 23, 24, 25,
	19, 20, 21, 22, 23, 24, 26, 27,
	20, 21, 22, 23, 25, 26, 27, 28,
	21, 22, 23, 24, 26, 27, 28, 30,
	22, 23, 24, 26, 27, 28, 30, 31,
	23, 24, 25, 27, 28, 30, 31, 33,
};

static const __u8 mpeg4_zigzag[HISTB_MPEG4_MATRIX_SIZE] = {
	0, 1, 8, 16, 9, 2, 3, 10,
	17, 24, 32, 25, 18, 11, 4, 5,
	12, 19, 26, 33, 40, 48, 41, 34,
	27, 20, 13, 6, 7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36,
	29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46,
	53, 60, 61, 54, 47, 55, 62, 63,
};

static void mpeg4_zero(void *ptr, __u32 bytes)
{
	__u8 *p = ptr;

	while (bytes--)
		*p++ = 0;
}

static void mpeg4_copy(__u8 *dst, const __u8 *src, __u32 bytes)
{
	while (bytes--)
		*dst++ = *src++;
}

static __u32 mpeg4_get_bits(struct histb_mpeg4_bits *bits, __u32 count)
{
	__u32 value = 0;

	if (count > 32 || count > bits->size_bits - bits->pos) {
		bits->error = HISTB_MPEG4_NEED_MORE;
		return 0;
	}

	while (count--) {
		value <<= 1;
		value |= (bits->data[bits->pos >> 3] >>
			  (7 - (bits->pos & 7))) & 1;
		bits->pos++;
	}

	return value;
}

static int mpeg4_marker(struct histb_mpeg4_bits *bits)
{
	if (mpeg4_get_bits(bits, 1) != 1)
		return bits->error ? bits->error : HISTB_MPEG4_INVALID;
	return HISTB_MPEG4_OK;
}

static int mpeg4_unsupported(struct histb_mpeg4_parser *parser,
			     enum histb_mpeg4_blocker blocker)
{
	parser->blocker = blocker;
	return HISTB_MPEG4_UNSUPPORTED;
}

static __u8 mpeg4_log2_bits(__u32 value)
{
	__u8 bits = 0;

	do {
		bits++;
		value >>= 1;
	} while (value);

	return bits;
}

static int mpeg4_get_sprite_length(struct histb_mpeg4_bits *bits)
{
	static const __u16 codes[] = {
		0, 2, 3, 4, 5, 6, 14, 30, 62, 126, 254, 510, 1022, 2046, 4094,
	};
	static const __u8 sizes[] = {
		2, 3, 3, 3, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
	};
	__u32 saved = bits->pos;
	unsigned int i;

	/* The production VFMW stopped at 12; its table and softlib define 15. */
	for (i = 0; i < HISTB_MPEG4_ARRAY_SIZE(codes); i++) {
		__u32 value;

		bits->pos = saved;
		bits->error = 0;
		value = mpeg4_get_bits(bits, sizes[i]);
		if (bits->error)
			break;
		if (value == codes[i])
			return i;
	}
	bits->pos = saved;
	if (!bits->error)
		bits->error = HISTB_MPEG4_INVALID;
	return bits->error;
}

static int mpeg4_get_sprite_component(struct histb_mpeg4_bits *bits,
				       __s32 *component)
{
	int length = mpeg4_get_sprite_length(bits);
	__u32 value;

	if (length < 0)
		return length;
	if (!length) {
		*component = 0;
		return HISTB_MPEG4_OK;
	}
	value = mpeg4_get_bits(bits, length);
	if (bits->error)
		return bits->error;
	if (!(value & (1U << (length - 1))))
		*component = -(__s32)(value ^ ((1U << length) - 1));
	else
		*component = value;
	return HISTB_MPEG4_OK;
}

static __s64 mpeg4_round_div(__s64 value, __s32 divisor)
{
	__s64 half = divisor >> 1;

	return value > 0 ? (value + half) / divisor :
		(value - half) / divisor;
}

static int mpeg4_store_s32(__s32 *dst, __s64 value)
{
	if (value < (-2147483647LL - 1) || value > 2147483647LL)
		return HISTB_MPEG4_INVALID;
	*dst = value;
	return HISTB_MPEG4_OK;
}

static int mpeg4_calculate_gmc(const struct histb_mpeg4_vol *vol,
			       const __s32 d[3][2], __u8 divx_500_b413,
			       struct histb_mpeg4_vop *vop)
{
	__s64 sprite_ref[3][2], virtual_ref[2][2];
	__s64 du[2], dv[2], uo, vo, uco, vco;
	__s32 width = vol->width, height = vol->height;
	__s32 points = vol->sprite_warping_points;
	__s32 accuracy = vol->sprite_warping_accuracy;
	__s32 a = 2 << accuracy, rho = 3 - accuracy, r = 16 / a;
	__s32 alpha = 0, beta = 0, min_ab, shift_y, shift_c;
	__s32 w2, h2, w3, h3;
	unsigned int i;

	while ((1U << alpha) < (__u32)width)
		alpha++;
	while ((1U << beta) < (__u32)height)
		beta++;
	w2 = 1U << alpha;
	h2 = 1U << beta;

	if (divx_500_b413) {
		sprite_ref[0][0] = d[0][0];
		sprite_ref[0][1] = d[0][1];
		sprite_ref[1][0] = (__s64)a * width + d[0][0] + d[1][0];
		sprite_ref[1][1] = d[0][1] + d[1][1];
		sprite_ref[2][0] = d[0][0] + d[2][0];
		sprite_ref[2][1] = (__s64)a * height + d[0][1] + d[2][1];
	} else {
		__s32 scale = a >> 1;

		sprite_ref[0][0] = (__s64)scale * d[0][0];
		sprite_ref[0][1] = (__s64)scale * d[0][1];
		sprite_ref[1][0] = (__s64)scale * (2 * width + d[0][0] + d[1][0]);
		sprite_ref[1][1] = (__s64)scale * (d[0][1] + d[1][1]);
		sprite_ref[2][0] = (__s64)scale * (d[0][0] + d[2][0]);
		sprite_ref[2][1] = (__s64)scale * (2 * height + d[0][1] + d[2][1]);
	}

	virtual_ref[0][0] = 16LL * w2 + mpeg4_round_div(
		(__s64)(width - w2) * (r * sprite_ref[0][0]) +
		(__s64)w2 * (r * sprite_ref[1][0] - 16LL * width), width);
	virtual_ref[0][1] = mpeg4_round_div(
		(__s64)(width - w2) * (r * sprite_ref[0][1]) +
		(__s64)w2 * (r * sprite_ref[1][1]), width);
	virtual_ref[1][0] = mpeg4_round_div(
		(__s64)(height - h2) * (r * sprite_ref[0][0]) +
		(__s64)h2 * (r * sprite_ref[2][0]), height);
	virtual_ref[1][1] = 16LL * h2 + mpeg4_round_div(
		(__s64)(height - h2) * (r * sprite_ref[0][1]) +
		(__s64)h2 * (r * sprite_ref[2][1] - 16LL * height), height);

	switch (points) {
	case 0:
		uo = vo = uco = vco = 0;
		du[0] = dv[1] = a;
		du[1] = dv[0] = 0;
		shift_y = shift_c = 0;
		break;
	case 1:
		uo = sprite_ref[0][0];
		vo = sprite_ref[0][1];
		uco = (sprite_ref[0][0] >> 1) | (sprite_ref[0][0] & 1);
		vco = (sprite_ref[0][1] >> 1) | (sprite_ref[0][1] & 1);
		du[0] = dv[1] = a;
		du[1] = dv[0] = 0;
		shift_y = shift_c = 0;
		break;
	case 2:
		uo = sprite_ref[0][0] * (1LL << (alpha + rho)) +
		     (1LL << (alpha + rho - 1));
		vo = sprite_ref[0][1] * (1LL << (alpha + rho)) +
		     (1LL << (alpha + rho - 1));
		du[0] = -r * sprite_ref[0][0] + virtual_ref[0][0];
		du[1] = r * sprite_ref[0][1] - virtual_ref[0][1];
		dv[0] = -r * sprite_ref[0][1] + virtual_ref[0][1];
		dv[1] = du[0];
		uco = du[0] + du[1] + 2LL * w2 * r * sprite_ref[0][0] -
		      16LL * w2 + (1LL << (alpha + rho + 1));
		vco = dv[0] + dv[1] + 2LL * w2 * r * sprite_ref[0][1] -
		      16LL * w2 + (1LL << (alpha + rho + 1));
		shift_y = alpha + rho;
		shift_c = alpha + rho + 2;
		break;
	case 3:
		min_ab = alpha < beta ? alpha : beta;
		w3 = w2 >> min_ab;
		h3 = h2 >> min_ab;
		uo = sprite_ref[0][0] *
		     (1LL << (alpha + beta + rho - min_ab)) +
		     (1LL << (alpha + beta + rho - min_ab - 1));
		vo = sprite_ref[0][1] *
		     (1LL << (alpha + beta + rho - min_ab)) +
		     (1LL << (alpha + beta + rho - min_ab - 1));
		du[0] = (-r * sprite_ref[0][0] + virtual_ref[0][0]) * h3;
		du[1] = (-r * sprite_ref[0][0] + virtual_ref[1][0]) * w3;
		dv[0] = (-r * sprite_ref[0][1] + virtual_ref[0][1]) * h3;
		dv[1] = (-r * sprite_ref[0][1] + virtual_ref[1][1]) * w3;
		uco = du[0] + du[1] + 2LL * w2 * h3 * r * sprite_ref[0][0] -
		      16LL * w2 * h3 +
		      (1LL << (alpha + beta + rho - min_ab + 1));
		vco = dv[0] + dv[1] + 2LL * w2 * h3 * r * sprite_ref[0][1] -
		      16LL * w2 * h3 +
		      (1LL << (alpha + beta + rho - min_ab + 1));
		shift_y = alpha + beta + rho - min_ab;
		shift_c = shift_y + 2;
		break;
	default:
		return HISTB_MPEG4_INVALID;
	}

	if (du[0] == ((__s64)a << shift_y) && !du[1] && !dv[0] &&
	    dv[1] == ((__s64)a << shift_y)) {
		uo >>= shift_y;
		vo >>= shift_y;
		uco >>= shift_c;
		vco >>= shift_c;
		du[0] = dv[1] = a;
		du[1] = dv[0] = 0;
		vop->gmc_points = 1;
	} else {
		shift_y = 16 - shift_y;
		shift_c = 16 - shift_c;
		if (shift_y < 0 || shift_c < 0)
			return HISTB_MPEG4_INVALID;
		uo *= 1LL << shift_y;
		vo *= 1LL << shift_y;
		uco *= 1LL << shift_c;
		vco *= 1LL << shift_c;
		for (i = 0; i < 2; i++) {
			du[i] *= 1LL << shift_y;
			dv[i] *= 1LL << shift_y;
		}
		vop->gmc_points = points;
	}

	return mpeg4_store_s32(&vop->gmc_du[0], du[0]) ||
	       mpeg4_store_s32(&vop->gmc_du[1], du[1]) ||
	       mpeg4_store_s32(&vop->gmc_dv[0], dv[0]) ||
	       mpeg4_store_s32(&vop->gmc_dv[1], dv[1]) ||
	       mpeg4_store_s32(&vop->gmc_uo, uo) ||
	       mpeg4_store_s32(&vop->gmc_vo, vo) ||
	       mpeg4_store_s32(&vop->gmc_uco, uco) ||
	       mpeg4_store_s32(&vop->gmc_vco, vco) ?
		HISTB_MPEG4_INVALID : HISTB_MPEG4_OK;
}

static int mpeg4_read_matrix(struct histb_mpeg4_bits *bits, __u8 *matrix)
{
	__u32 i;
	__u8 last = 0;

	for (i = 0; i < HISTB_MPEG4_MATRIX_SIZE; i++) {
		__u8 value = mpeg4_get_bits(bits, 8);

		if (bits->error)
			return bits->error;
		matrix[mpeg4_zigzag[i]] = value;
		if (!value) {
			while (++i < HISTB_MPEG4_MATRIX_SIZE)
				matrix[mpeg4_zigzag[i]] = last;
			return HISTB_MPEG4_OK;
		}
		last = value;
	}

	return HISTB_MPEG4_OK;
}

static int mpeg4_parse_vol(struct histb_mpeg4_parser *parser,
			   const __u8 *data, __u32 bytes)
{
	struct histb_mpeg4_bits bits = {
		.data = data,
		.size_bits = bytes * 8,
	};
	struct histb_mpeg4_vol vol;
	__u32 verid;
	__u32 shape;
	__u32 value;
	int ret;

	mpeg4_zero(&vol, sizeof(vol));
	vol.profile_and_level = parser->vol.profile_and_level;
	mpeg4_copy(vol.intra_quant_matrix, mpeg4_default_intra,
		   HISTB_MPEG4_MATRIX_SIZE);
	mpeg4_copy(vol.nonintra_quant_matrix, mpeg4_default_nonintra,
		   HISTB_MPEG4_MATRIX_SIZE);

	(void)mpeg4_get_bits(&bits, 1); /* random_accessible_vol */
	(void)mpeg4_get_bits(&bits, 8); /* video_object_type_indication */
	if (mpeg4_get_bits(&bits, 1)) {
		verid = mpeg4_get_bits(&bits, 4);
		(void)mpeg4_get_bits(&bits, 3);
	} else {
		verid = 1;
	}
	if (bits.error)
		return bits.error;
	if (!verid)
		return HISTB_MPEG4_INVALID;
	vol.video_object_layer_verid = verid;

	value = mpeg4_get_bits(&bits, 4);
	if (value == 15) {
		(void)mpeg4_get_bits(&bits, 8);
		(void)mpeg4_get_bits(&bits, 8);
	}

	vol.vol_control_parameters = mpeg4_get_bits(&bits, 1);
	if (vol.vol_control_parameters) {
		value = mpeg4_get_bits(&bits, 2);
		if (value != 1)
			return bits.error ? bits.error :
				mpeg4_unsupported(parser,
						  HISTB_MPEG4_BLOCK_CHROMA_FORMAT);
		vol.low_delay = mpeg4_get_bits(&bits, 1);
		if (mpeg4_get_bits(&bits, 1)) {
			(void)mpeg4_get_bits(&bits, 15);
			ret = mpeg4_marker(&bits);
			if (ret)
				return ret;
			(void)mpeg4_get_bits(&bits, 15);
			ret = mpeg4_marker(&bits);
			if (ret)
				return ret;
			(void)mpeg4_get_bits(&bits, 15);
			ret = mpeg4_marker(&bits);
			if (ret)
				return ret;
			(void)mpeg4_get_bits(&bits, 3);
			(void)mpeg4_get_bits(&bits, 11);
			ret = mpeg4_marker(&bits);
			if (ret)
				return ret;
			(void)mpeg4_get_bits(&bits, 15);
			ret = mpeg4_marker(&bits);
			if (ret)
				return ret;
		}
	}

	shape = mpeg4_get_bits(&bits, 2);
	if (bits.error)
		return bits.error;
	if (shape)
		return mpeg4_unsupported(parser,
					 HISTB_MPEG4_BLOCK_NON_RECTANGULAR);
	ret = mpeg4_marker(&bits);
	if (ret)
		return ret;

	value = mpeg4_get_bits(&bits, 16);
	if (bits.error)
		return bits.error;
	if (!value)
		return HISTB_MPEG4_INVALID;
	vol.vop_time_increment_resolution = value;
	vol.vop_time_increment_bits = value <= 1 ? 1 : mpeg4_log2_bits(value - 1);
	ret = mpeg4_marker(&bits);
	if (ret)
		return ret;

	vol.fixed_vop_rate = mpeg4_get_bits(&bits, 1);
	if (vol.fixed_vop_rate)
		vol.fixed_vop_time_increment =
			mpeg4_get_bits(&bits, vol.vop_time_increment_bits);
	ret = mpeg4_marker(&bits);
	if (ret)
		return ret;
	vol.width = mpeg4_get_bits(&bits, 13);
	ret = mpeg4_marker(&bits);
	if (ret)
		return ret;
	vol.height = mpeg4_get_bits(&bits, 13);
	ret = mpeg4_marker(&bits);
	if (ret)
		return ret;
	if (!vol.width || !vol.height || vol.width > 8192 || vol.height > 8192)
		return HISTB_MPEG4_INVALID;

	vol.interlaced = mpeg4_get_bits(&bits, 1);
	if (mpeg4_get_bits(&bits, 1) != 1) /* obmc_disable */
		return bits.error ? bits.error :
			mpeg4_unsupported(parser, HISTB_MPEG4_BLOCK_OBMC);
	value = mpeg4_get_bits(&bits, verid == 1 ? 1 : 2);
	if (bits.error)
		return bits.error;
	if (value == 1)
		return mpeg4_unsupported(parser, HISTB_MPEG4_BLOCK_SPRITE);
	if (value > 2)
		return HISTB_MPEG4_INVALID;
	vol.sprite_enable = value;
	if (vol.sprite_enable == 2) {
		vol.sprite_warping_points = mpeg4_get_bits(&bits, 6);
		vol.sprite_warping_accuracy = mpeg4_get_bits(&bits, 2);
		vol.sprite_brightness_change = mpeg4_get_bits(&bits, 1);
		if (bits.error)
			return bits.error;
		/* The CV200 implementation has coefficient cases only for 0..3. */
		if (vol.sprite_warping_points > 3)
			return HISTB_MPEG4_INVALID;
	}
	if (mpeg4_get_bits(&bits, 1)) /* not_8_bit */
		return mpeg4_unsupported(parser, HISTB_MPEG4_BLOCK_NON_8_BIT);

	vol.quant_type = mpeg4_get_bits(&bits, 1);
	if (vol.quant_type) {
		if (mpeg4_get_bits(&bits, 1)) {
			ret = mpeg4_read_matrix(&bits, vol.intra_quant_matrix);
			if (ret)
				return ret;
		}
		if (mpeg4_get_bits(&bits, 1)) {
			ret = mpeg4_read_matrix(&bits, vol.nonintra_quant_matrix);
			if (ret)
				return ret;
		}
	}
	if (verid != 1)
		vol.quarter_sample = mpeg4_get_bits(&bits, 1);
	if (!mpeg4_get_bits(&bits, 1)) /* complexity_estimation_disable */
		return bits.error ? bits.error :
			mpeg4_unsupported(parser,
					  HISTB_MPEG4_BLOCK_COMPLEXITY_ESTIMATION);
	vol.resync_marker_disable = mpeg4_get_bits(&bits, 1);
	if (mpeg4_get_bits(&bits, 1)) /* data_partitioned */
		return bits.error ? bits.error :
			mpeg4_unsupported(parser,
					  HISTB_MPEG4_BLOCK_DATA_PARTITIONING);
	if (verid != 1) {
		if (mpeg4_get_bits(&bits, 1)) /* newpred_enable */
			return bits.error ? bits.error :
				mpeg4_unsupported(parser, HISTB_MPEG4_BLOCK_NEWPRED);
		if (mpeg4_get_bits(&bits, 1)) /* reduced_resolution_vop_enable */
			return bits.error ? bits.error :
				mpeg4_unsupported(parser,
						  HISTB_MPEG4_BLOCK_REDUCED_RESOLUTION);
	}
	if (mpeg4_get_bits(&bits, 1)) /* scalability */
		return bits.error ? bits.error :
			mpeg4_unsupported(parser, HISTB_MPEG4_BLOCK_SCALABILITY);
	if (bits.error)
		return bits.error;

	parser->vol = vol;
	parser->have_vol = 1;
	return HISTB_MPEG4_OK;
}

static int mpeg4_parse_vop(struct histb_mpeg4_parser *parser,
			   const __u8 *data, __u32 bytes,
			   __u32 absolute_bit_offset,
			   struct histb_mpeg4_frame *frame)
{
	struct histb_mpeg4_bits bits = {
		.data = data,
		.size_bits = bytes * 8,
	};
	struct histb_mpeg4_vop vop;
	__s32 trajectory[3][2] = { };
	__u32 time_incr = 0;
	__u32 time_increment;
	__u32 time;
	int ret;

	if (!parser->have_vol)
		return HISTB_MPEG4_INVALID;
	mpeg4_zero(&vop, sizeof(vop));
	vop.coding_type = mpeg4_get_bits(&bits, 2);
	if (vop.coding_type == HISTB_MPEG4_S_VOP &&
	    parser->vol.sprite_enable != 2)
		return bits.error ? bits.error : HISTB_MPEG4_INVALID;
	while (mpeg4_get_bits(&bits, 1)) {
		if (bits.error)
			return bits.error;
		if (++time_incr == 6400)
			return HISTB_MPEG4_INVALID;
	}
	ret = mpeg4_marker(&bits);
	if (ret)
		return ret;
	time_increment = mpeg4_get_bits(&bits,
					 parser->vol.vop_time_increment_bits);
	ret = mpeg4_marker(&bits);
	if (ret)
		return ret;
	vop.coded = mpeg4_get_bits(&bits, 1);
	if (bits.error)
		return bits.error;
	if (vop.coding_type != HISTB_MPEG4_B_VOP) {
		parser->last_time_base = parser->time_base;
		parser->time_base += time_incr;
		time = parser->time_base *
			parser->vol.vop_time_increment_resolution + time_increment;
		vop.time_pp = parser->have_non_b_time ?
			time - parser->last_non_b_time : 0;
		parser->time_pp = vop.time_pp;
		parser->last_non_b_time = time;
		parser->have_non_b_time = 1;
	} else {
		__u32 to_future;

		time = (parser->last_time_base + time_incr) *
			parser->vol.vop_time_increment_resolution + time_increment;
		if (!parser->have_non_b_time || time > parser->last_non_b_time)
			return HISTB_MPEG4_INVALID;
		vop.time_pp = parser->time_pp;
		to_future = parser->last_non_b_time - time;
		if (!vop.time_pp || to_future >= vop.time_pp)
			return HISTB_MPEG4_INVALID;
		vop.time_bp = vop.time_pp - to_future;
		if (vop.time_bp >= vop.time_pp)
			return HISTB_MPEG4_INVALID;
	}
	vop.time = time;
	if (!vop.coded) {
		vop.coding_type = HISTB_MPEG4_N_VOP;
		vop.payload_bit_offset = absolute_bit_offset + bits.pos;
		frame->vol = parser->vol;
		frame->vop = vop;
		return HISTB_MPEG4_OK;
	}

	if (vop.coding_type == HISTB_MPEG4_P_VOP ||
	    vop.coding_type == HISTB_MPEG4_S_VOP)
		vop.rounding_type = mpeg4_get_bits(&bits, 1);
	vop.intra_dc_vlc_thr = mpeg4_get_bits(&bits, 3);
	if (parser->vol.interlaced) {
		vop.top_field_first = mpeg4_get_bits(&bits, 1);
		vop.alternate_vertical_scan = mpeg4_get_bits(&bits, 1);
	}
	if (vop.coding_type == HISTB_MPEG4_S_VOP) {
		__u8 divx_500_b413 = parser->divx_version == 500 &&
			parser->divx_build == 413;
		unsigned int i;

		for (i = 0; i < parser->vol.sprite_warping_points; i++) {
			ret = mpeg4_get_sprite_component(&bits, &trajectory[i][0]);
			if (ret)
				return ret;
			if (!divx_500_b413 && mpeg4_marker(&bits))
				return bits.error ? bits.error : HISTB_MPEG4_INVALID;
			ret = mpeg4_get_sprite_component(&bits, &trajectory[i][1]);
			if (ret)
				return ret;
			ret = mpeg4_marker(&bits);
			if (ret)
				return ret;
		}
		vop.divx_500_b413 = divx_500_b413;
		ret = mpeg4_calculate_gmc(&parser->vol, trajectory,
					  divx_500_b413, &vop);
		if (ret)
			return ret;
	}
	vop.quant = mpeg4_get_bits(&bits, 5);
	if (!vop.quant)
		return bits.error ? bits.error : HISTB_MPEG4_INVALID;
	if (vop.coding_type != HISTB_MPEG4_I_VOP) {
		vop.fcode_forward = mpeg4_get_bits(&bits, 3);
		if (!vop.fcode_forward)
			return bits.error ? bits.error : HISTB_MPEG4_INVALID;
	}
	if (vop.coding_type == HISTB_MPEG4_B_VOP) {
		vop.fcode_backward = mpeg4_get_bits(&bits, 3);
		if (!vop.fcode_backward)
			return bits.error ? bits.error : HISTB_MPEG4_INVALID;
		/* Match VFMW's workaround for encoders that omit VOL control data. */
		if (parser->vol.low_delay &&
		    !parser->vol.vol_control_parameters)
			parser->vol.low_delay = 0;
	}
	if (bits.error)
		return bits.error;
	vop.payload_bit_offset = absolute_bit_offset + bits.pos;
	vop.payload_bits = bytes * 8 - bits.pos;
	frame->vol = parser->vol;
	frame->vop = vop;
	return HISTB_MPEG4_OK;
}

static int mpeg4_parse_uint(const __u8 *data, __u32 bytes, __u32 *pos,
			    __u16 *value)
{
	__u32 number = 0;
	__u32 digits = 0;

	while (*pos < bytes && data[*pos] == ' ')
		(*pos)++;
	while (*pos < bytes && data[*pos] >= '0' && data[*pos] <= '9') {
		number = number * 10 + data[(*pos)++] - '0';
		if (number > 65535)
			return HISTB_MPEG4_INVALID;
		digits++;
	}
	if (!digits)
		return HISTB_MPEG4_INVALID;
	*value = number;
	return HISTB_MPEG4_OK;
}

static void mpeg4_parse_userdata(struct histb_mpeg4_parser *parser,
				 const __u8 *data, __u32 bytes)
{
	__u16 build, version;
	__u32 pos = 4;
	int ret;

	if (bytes < 5 || data[0] != 'D' || data[1] != 'i' ||
	    data[2] != 'v' || data[3] != 'X')
		return;
	ret = mpeg4_parse_uint(data, bytes, &pos, &version);
	if (ret)
		return;
	if (pos < bytes && data[pos] == 'b') {
		pos++;
	} else if (pos + 5 <= bytes && data[pos] == 'B' &&
		   data[pos + 1] == 'u' && data[pos + 2] == 'i' &&
		   data[pos + 3] == 'l' && data[pos + 4] == 'd') {
		pos += 5;
	} else {
		return;
	}
	ret = mpeg4_parse_uint(data, bytes, &pos, &build);
	if (ret)
		return;
	parser->divx_version = version;
	parser->divx_build = build;
	parser->divx_packed = pos < bytes && data[pos] == 'p';
}

static __u32 mpeg4_find_start_code(const __u8 *data, __u32 bytes, __u32 from)
{
	__u32 i;

	for (i = from; i + 3 < bytes; i++)
		if (!data[i] && !data[i + 1] && data[i + 2] == 1)
			return i;
	return bytes;
}

void histb_mpeg4_parser_reset(struct histb_mpeg4_parser *parser)
{
	mpeg4_zero(parser, sizeof(*parser));
}

int histb_mpeg4_parse_frame_at(struct histb_mpeg4_parser *parser,
			       const __u8 *data, __u32 bytes, __u32 offset,
			       struct histb_mpeg4_frame *frame,
			       __u32 *next_offset)
{
	__u32 start;

	if (!parser || !data || !frame || !next_offset || offset > bytes)
		return HISTB_MPEG4_INVALID;
	mpeg4_zero(frame, sizeof(*frame));
	parser->blocker = HISTB_MPEG4_BLOCK_NONE;
	*next_offset = offset;
	start = mpeg4_find_start_code(data, bytes, offset);
	while (start < bytes) {
		__u32 next = mpeg4_find_start_code(data, bytes, start + 4);
		__u32 payload = start + 4;
		__u8 code = data[start + 3];
		int ret;

		if (code == MPEG4_START_VOS) {
			if (payload >= next)
				return HISTB_MPEG4_NEED_MORE;
			parser->vol.profile_and_level = data[payload];
		} else if (code == MPEG4_START_VOS_END) {
			for (; payload < bytes; payload++)
				if (data[payload])
					return HISTB_MPEG4_INVALID;
			*next_offset = start + 4;
			return HISTB_MPEG4_END_OF_STREAM;
		} else if (code >= MPEG4_START_VOL_MIN &&
			   code <= MPEG4_START_VOL_MAX) {
			ret = mpeg4_parse_vol(parser, data + payload,
					      next - payload);
			if (ret)
				return ret;
		} else if (code == MPEG4_START_USER_DATA) {
			mpeg4_parse_userdata(parser, data + payload, next - payload);
		} else if (code == MPEG4_START_VOP) {
			int ret = mpeg4_parse_vop(parser, data + payload,
						   next - payload, payload * 8, frame);

			if (!ret)
				*next_offset = next;
			return ret;
		}
		start = next;
	}

	*next_offset = bytes;
	return HISTB_MPEG4_NO_VOP;
}

int histb_mpeg4_parse_frame(struct histb_mpeg4_parser *parser,
			    const __u8 *data, __u32 bytes,
			    struct histb_mpeg4_frame *frame)
{
	__u32 next_offset;

	return histb_mpeg4_parse_frame_at(parser, data, bytes, 0, frame,
					  &next_offset);
}

int histb_mpeg4_validate_stateful_frame(struct histb_mpeg4_parser *parser,
					 const __u8 *data, __u32 bytes,
					 const struct histb_mpeg4_frame *frame)
{
	__u32 start;
	__u32 vops = 0;

	if (!parser || !data || !frame)
		return HISTB_MPEG4_INVALID;
	if (!frame->vop.coded || frame->vop.coding_type == HISTB_MPEG4_N_VOP)
		return mpeg4_unsupported(parser, HISTB_MPEG4_BLOCK_N_VOP);
	start = mpeg4_find_start_code(data, bytes, 0);
	while (start < bytes) {
		if (data[start + 3] == MPEG4_START_VOP)
			vops++;
		start = mpeg4_find_start_code(data, bytes, start + 4);
	}
	if (!vops)
		return HISTB_MPEG4_INVALID;

	return HISTB_MPEG4_OK;
}

int histb_mpeg4_plan_display(struct histb_mpeg4_display_state *state,
			     __u8 coding_type, __u8 low_delay,
			     __u8 first_anchor, __u8 draining,
			     struct histb_mpeg4_display_plan *plan)
{
	if (!state || !plan || coding_type > HISTB_MPEG4_S_VOP)
		return HISTB_MPEG4_INVALID;
	mpeg4_zero(plan, sizeof(*plan));

	if (low_delay || (first_anchor && coding_type != HISTB_MPEG4_B_VOP)) {
		plan->release_pending = state->pending;
		plan->release_current = 1;
		plan->current_last = draining;
		state->pending = 0;
		return HISTB_MPEG4_OK;
	}

	if (coding_type == HISTB_MPEG4_B_VOP) {
		if (!state->pending)
			return HISTB_MPEG4_INVALID;
		plan->release_current = 1;
		if (draining) {
			plan->release_pending = 1;
			plan->current_before_pending = 1;
			plan->pending_last = 1;
			state->pending = 0;
		}
		return HISTB_MPEG4_OK;
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
	return HISTB_MPEG4_OK;
}

int histb_mpeg4_plan_drain(struct histb_mpeg4_display_state *state,
			   struct histb_mpeg4_display_plan *plan)
{
	if (!state || !plan)
		return HISTB_MPEG4_INVALID;
	mpeg4_zero(plan, sizeof(*plan));
	if (state->pending) {
		plan->release_pending = 1;
		plan->pending_last = 1;
		state->pending = 0;
	}
	return HISTB_MPEG4_OK;
}

static __u32 mpeg4_pack_matrix_word(const __u8 *matrix, __u32 column,
				     __u32 half)
{
	__u32 row = half * 8;

	return matrix[row + column] |
	       ((__u32)matrix[row + 16 + column] << 8) |
	       ((__u32)matrix[row + 32 + column] << 16) |
	       ((__u32)matrix[row + 48 + column] << 24);
}

static __s32 mpeg4_gmc_translation(__s32 offset, __u8 accuracy,
				    __u8 quarter_sample, __u8 divx_500_b413)
{
	__s64 value = (__s64)offset * (1U << quarter_sample);
	__s32 divisor = 1U << accuracy;

	if (divx_500_b413 && accuracy >= quarter_sample)
		return offset / (1U << (accuracy - quarter_sample));
	if (value > 0)
		value += divisor >> 1;
	else
		value += (divisor >> 1) - 1;
	return value >> accuracy;
}

int histb_mpeg4_build_pic_msg(const struct histb_mpeg4_hw_picture *picture,
			      struct histb_mpeg4_pic_msg *msg)
{
	const struct histb_mpeg4_frame *frame;
	const struct histb_mpeg4_vol *vol;
	const struct histb_mpeg4_vop *vop;
	__u32 width_mb;
	__u32 height_mb;
	__u32 mb_bits;
	__u32 i;

	if (!picture || !msg)
		return HISTB_MPEG4_INVALID;
	frame = &picture->frame;
	vol = &frame->vol;
	vop = &frame->vop;
	width_mb = (vol->width + 15) >> 4;
	height_mb = (vol->height + 15) >> 4;
	if (!width_mb || !height_mb || width_mb > 512 || height_mb > 512 ||
	    vop->coding_type > HISTB_MPEG4_S_VOP || !vop->coded)
		return HISTB_MPEG4_INVALID;
	mb_bits = mpeg4_log2_bits(width_mb * height_mb);
	mpeg4_zero(msg, sizeof(*msg));
	msg->d[0] = 0;
	msg->d[1] = ((__u32)(vol->quant_type & 1) << 31) |
		    ((__u32)(vop->quant & 31) << 26);
	msg->d[2] = (vol->interlaced & 1) |
		    ((__u32)(vop->top_field_first & 1) << 1) |
		    ((__u32)(vop->alternate_vertical_scan & 1) << 2) |
		    ((__u32)(vop->coding_type == HISTB_MPEG4_B_VOP ? 0 :
			      vop->rounding_type & 1) << 3) |
		    ((__u32)(vol->quarter_sample & 1) << 4) |
		    ((__u32)(vop->coding_type & 3) << 5) |
		    ((__u32)(vol->resync_marker_disable & 1) << 7) |
		    ((__u32)(vop->intra_dc_vlc_thr & 7) << 8) |
		    ((__u32)(vop->fcode_forward & 7) << 11) |
		    ((__u32)(vop->fcode_backward & 7) << 14) |
		    ((__u32)(vol->vop_time_increment_bits & 31) << 17) |
		    ((mb_bits & 15) << 22) |
		    ((__u32)(vol->sprite_enable & 3) << 26) |
		    ((__u32)(vol->sprite_warping_accuracy & 3) << 28) |
		    ((__u32)(vop->gmc_points & 3) << 30);
	if (vop->coding_type == HISTB_MPEG4_B_VOP)
		msg->d[3] = (vop->time_bp * 2 & 0xffff) |
			    (vop->time_pp << 17);
	msg->d[4] = (width_mb - 1) | ((height_mb - 1) << 16);
	msg->d[5] = vol->width | ((__u32)vol->height << 16);
	if (vop->coding_type == HISTB_MPEG4_S_VOP && vop->gmc_points == 1) {
		msg->d[6] = mpeg4_gmc_translation(vop->gmc_uo,
						    vol->sprite_warping_accuracy,
						    vol->quarter_sample,
						    vop->divx_500_b413);
		msg->d[7] = mpeg4_gmc_translation(vop->gmc_vo,
						    vol->sprite_warping_accuracy,
						    vol->quarter_sample,
						    vop->divx_500_b413);
	}
	msg->d[12] = picture->display_picture_addr & ~15U;
	msg->d[13] = picture->forward_ref_addr & ~15U;
	msg->d[14] = picture->backward_ref_addr & ~15U;
	msg->d[15] = picture->current_pmv_addr & ~15U;
	msg->d[16] = picture->backward_pmv_addr & ~15U;
	msg->d[17] = picture->itrans_top_addr & ~15U;
	msg->d[18] = picture->pmv_top_addr & ~15U;
	msg->d[19] = (picture->bug_qpel_chroma & 1) |
		     ((__u32)(picture->bug_qpel_chroma2 & 1) << 1) |
		     ((__u32)(picture->bug_edge_extend & 1) << 2) |
		     ((__u32)(vop->coding_type == HISTB_MPEG4_S_VOP &&
			       vop->gmc_points == 1 ? 1 :
			       (picture->bug_edge_extend & 1)) << 3);
	if (vop->coding_type == HISTB_MPEG4_S_VOP) {
		msg->d[20] = vop->gmc_du[0];
		msg->d[21] = vop->gmc_du[1];
		msg->d[22] = vop->gmc_dv[0];
		msg->d[23] = vop->gmc_dv[1];
		msg->d[24] = vop->gmc_uo;
		msg->d[25] = vop->gmc_vo;
		msg->d[26] = vop->gmc_uco;
		msg->d[27] = vop->gmc_vco;
	}
	if (vol->quant_type) {
		for (i = 0; i < 8; i++) {
			msg->d[28 + i * 2] =
				mpeg4_pack_matrix_word(vol->intra_quant_matrix, i, 0);
			msg->d[29 + i * 2] =
				mpeg4_pack_matrix_word(vol->intra_quant_matrix, i, 1);
			msg->d[44 + i * 2] =
				mpeg4_pack_matrix_word(vol->nonintra_quant_matrix, i, 0);
			msg->d[45 + i * 2] =
				mpeg4_pack_matrix_word(vol->nonintra_quant_matrix, i, 1);
		}
	}
	msg->d[60] = picture->sed_top_addr & ~15U;
	msg->d[63] = picture->slice_msg_addr & ~15U;
	return HISTB_MPEG4_OK;
}

int histb_mpeg4_build_regs(const struct histb_mpeg4_hw_picture *picture,
			   const struct histb_mpeg4_slice *slices,
			   __u32 slice_count,
			   struct histb_mpeg4_regs *regs)
{
	__u32 width_mb;
	__u32 height_mb;
	__u32 stream_base = ~0U;
	__u32 i;

	if (!picture || !slices || !slice_count || !regs)
		return HISTB_MPEG4_INVALID;
	width_mb = (picture->frame.vol.width + 15) >> 4;
	height_mb = (picture->frame.vol.height + 15) >> 4;
	if (!width_mb || !height_mb || width_mb > 512 || height_mb > 512)
		return HISTB_MPEG4_INVALID;
	for (i = 0; i < slice_count; i++) {
		unsigned int fragment;

		for (fragment = 0; fragment < HISTB_MPEG4_SLICE_FRAGMENTS;
		     fragment++) {
			__u32 addr;

			if (!slices[i].bit_len[fragment])
				continue;
			addr = slices[i].dma_addr[fragment] & ~15U;
			if (addr < stream_base)
				stream_base = addr;
		}
	}
	if (stream_base == ~0U)
		return HISTB_MPEG4_INVALID;

	mpeg4_zero(regs, sizeof(*regs));
	regs->basic_cfg0 = 0x42400000 | ((width_mb * height_mb - 1) & 0xfffff);
	regs->basic_cfg1 = ((__u32)(picture->compression & 1) << 30) |
			   ((__u32)(picture->vdh_mmu & 1) << 12) | 0x1c002;
	regs->avm_addr = picture->avm_addr & ~15U;
	regs->vam_addr = picture->vam_addr & ~15U;
	regs->stream_base = stream_base;
	regs->current_picture_addr = picture->current_picture_addr & ~15U;
	regs->y_stride = picture->y_stride;
	regs->uv_offset = picture->uv_offset;
	regs->fixed_cfg = 0x00300c03;
	regs->prcnum = picture->prcnum;
	regs->dnr_mbinfo_addr = picture->dnr_mbinfo_addr;
	regs->scd_emar = width_mb <= 120;
	return HISTB_MPEG4_OK;
}

static __u32 mpeg4_resync_marker_bits(const struct histb_mpeg4_frame *frame)
{
	__u32 fcode;

	switch (frame->vop.coding_type) {
	case HISTB_MPEG4_I_VOP:
		return 17;
	case HISTB_MPEG4_P_VOP:
	case HISTB_MPEG4_S_VOP:
		return 16 + frame->vop.fcode_forward;
	case HISTB_MPEG4_B_VOP:
		fcode = frame->vop.fcode_forward > frame->vop.fcode_backward ?
			frame->vop.fcode_forward : frame->vop.fcode_backward;
		return 16 + fcode > 18 ? 16 + fcode : 18;
	default:
		return 0;
	}
}

static int mpeg4_bit_at(const __u8 *data, __u32 size_bits, __u32 pos)
{
	if (pos >= size_bits)
		return -1;
	return (data[pos >> 3] >> (7 - (pos & 7))) & 1;
}

static int mpeg4_find_resync_marker(const __u8 *data, __u32 end_bits,
				    __u32 from, __u32 marker_bits,
				    __u32 *marker_pos)
{
	__u32 pos;

	if (!data || !marker_pos || marker_bits < 17 || marker_bits > 32)
		return HISTB_MPEG4_INVALID;
	pos = (from + 7) & ~7U;
	for (; pos + marker_bits <= end_bits; pos += 8) {
		__u32 bit;

		for (bit = 0; bit + 1 < marker_bits; bit++)
			if (mpeg4_bit_at(data, end_bits, pos + bit))
				break;
		if (bit + 1 == marker_bits &&
		    mpeg4_bit_at(data, end_bits, pos + marker_bits - 1) == 1) {
			*marker_pos = pos;
			return HISTB_MPEG4_OK;
		}
	}

	return HISTB_MPEG4_NO_VOP;
}

static int mpeg4_parse_video_packet_header(
				const struct histb_mpeg4_frame *frame,
				const __u8 *data, __u32 end_bits,
				__u32 marker_pos, __u32 marker_bits,
				__u32 buffer_dma,
				struct histb_mpeg4_slice *slice)
{
	struct histb_mpeg4_bits bits = {
		.data = data,
		.size_bits = end_bits,
		.pos = marker_pos + marker_bits,
	};
	__u32 total_mbs = ((__u32)frame->vol.width + 15) / 16 *
			   (((__u32)frame->vol.height + 15) / 16);
	__u32 mb_bits = total_mbs > 1 ? mpeg4_log2_bits(total_mbs - 1) : 1;
	__u32 header_extension;
	__u32 byte_offset;
	__u32 value;

	if (!total_mbs || !slice || bits.pos > end_bits)
		return HISTB_MPEG4_INVALID;
	mpeg4_zero(slice, sizeof(*slice));
	slice->mb_start = mpeg4_get_bits(&bits, mb_bits);
	if (slice->mb_start >= total_mbs)
		return bits.error ? bits.error : HISTB_MPEG4_INVALID;
	slice->quant = mpeg4_get_bits(&bits, 5);
	header_extension = mpeg4_get_bits(&bits, 1);
	slice->coding_type = frame->vop.coding_type;
	slice->intra_dc_vlc_thr = frame->vop.intra_dc_vlc_thr;
	slice->fcode_forward = frame->vop.fcode_forward;
	slice->fcode_backward = frame->vop.fcode_backward;

	if (header_extension) {
		do {
			value = mpeg4_get_bits(&bits, 1);
			if (bits.error)
				return bits.error;
		} while (value);
		if (mpeg4_marker(&bits))
			return bits.error ? bits.error : HISTB_MPEG4_INVALID;
		(void)mpeg4_get_bits(&bits, frame->vol.vop_time_increment_bits);
		if (mpeg4_marker(&bits))
			return bits.error ? bits.error : HISTB_MPEG4_INVALID;
		slice->coding_type = mpeg4_get_bits(&bits, 2);
		slice->intra_dc_vlc_thr = mpeg4_get_bits(&bits, 3);
		if (slice->coding_type > HISTB_MPEG4_S_VOP)
			return HISTB_MPEG4_INVALID;
		if (slice->coding_type != HISTB_MPEG4_I_VOP &&
		    !mpeg4_get_bits(&bits, 3))
			return bits.error ? bits.error : HISTB_MPEG4_INVALID;
		if (slice->coding_type == HISTB_MPEG4_B_VOP &&
		    !mpeg4_get_bits(&bits, 3))
			return bits.error ? bits.error : HISTB_MPEG4_INVALID;
	}
	if (bits.error || bits.pos >= end_bits)
		return bits.error ? bits.error : HISTB_MPEG4_INVALID;

	byte_offset = bits.pos >> 3;
	slice->bit_offset[0] = bits.pos & 7;
	slice->dma_addr[0] = buffer_dma + byte_offset;
	return HISTB_MPEG4_OK;
}

int histb_mpeg4_parse_slices(const struct histb_mpeg4_frame *frame,
			     const __u8 *data, __u32 bytes, __u32 buffer_dma,
			     struct histb_mpeg4_slice *slices,
			     __u32 slice_capacity, __u32 *slice_count)
{
	__u32 end_bits;
	__u32 marker_bits;
	__u32 marker_pos;
	__u32 current_start;
	__u32 count = 1;
	int ret;

	if (!frame || !data || !slices || !slice_count || !slice_capacity ||
	    frame->vop.payload_bit_offset > bytes * 8 ||
	    frame->vop.payload_bits > bytes * 8 - frame->vop.payload_bit_offset)
		return HISTB_MPEG4_INVALID;
	ret = histb_mpeg4_make_single_slice(frame, buffer_dma, &slices[0]);
	if (ret || frame->vol.resync_marker_disable) {
		if (!ret)
			*slice_count = 1;
		return ret;
	}

	end_bits = frame->vop.payload_bit_offset + frame->vop.payload_bits;
	marker_bits = mpeg4_resync_marker_bits(frame);
	current_start = frame->vop.payload_bit_offset;
	for (;;) {
		ret = mpeg4_find_resync_marker(data, end_bits, current_start + 1,
						 marker_bits, &marker_pos);
		if (ret == HISTB_MPEG4_NO_VOP)
			break;
		if (ret || count >= slice_capacity)
			return ret ? ret : HISTB_MPEG4_INVALID;
		if (marker_pos <= current_start)
			return HISTB_MPEG4_INVALID;
		slices[count - 1].bit_len[0] = marker_pos - current_start;
		ret = mpeg4_parse_video_packet_header(frame, data, end_bits,
						      marker_pos, marker_bits,
						      buffer_dma, &slices[count]);
		if (ret)
			return ret;
		if (slices[count].mb_start <= slices[count - 1].mb_start)
			return HISTB_MPEG4_INVALID;
		current_start = (slices[count].dma_addr[0] - buffer_dma) * 8 +
				slices[count].bit_offset[0];
		count++;
	}
	if (end_bits <= current_start)
		return HISTB_MPEG4_INVALID;
	slices[count - 1].bit_len[0] = end_bits - current_start;
	*slice_count = count;
	return HISTB_MPEG4_OK;
}

int histb_mpeg4_make_single_slice(const struct histb_mpeg4_frame *frame,
				  __u32 buffer_dma,
				  struct histb_mpeg4_slice *slice)
{
	__u32 byte_offset;

	if (!frame || !slice || !frame->vop.coded || !frame->vop.payload_bits)
		return HISTB_MPEG4_INVALID;
	byte_offset = frame->vop.payload_bit_offset >> 3;
	mpeg4_zero(slice, sizeof(*slice));
	slice->bit_offset[0] = frame->vop.payload_bit_offset & 7;
	slice->fcode_forward = frame->vop.fcode_forward;
	slice->fcode_backward = frame->vop.fcode_backward;
	slice->intra_dc_vlc_thr = frame->vop.intra_dc_vlc_thr;
	slice->coding_type = frame->vop.coding_type;
	slice->quant = frame->vop.quant;
	slice->bit_len[0] = frame->vop.payload_bits;
	slice->dma_addr[0] = buffer_dma + byte_offset;
	return HISTB_MPEG4_OK;
}

int histb_mpeg4_build_slice_msg(const struct histb_mpeg4_frame *frame,
				const struct histb_mpeg4_slice *slice,
				__u32 stream_base,
				struct histb_mpeg4_slice_msg *msg)
{
	__u32 width_mb;
	__u32 height_mb;
	unsigned int fragment;

	if (!frame || !slice || !msg || !slice->bit_len[0])
		return HISTB_MPEG4_INVALID;
	width_mb = (frame->vol.width + 15) >> 4;
	height_mb = (frame->vol.height + 15) >> 4;
	if (!width_mb || !height_mb || slice->mb_start >= width_mb * height_mb)
		return HISTB_MPEG4_INVALID;
	mpeg4_zero(msg, sizeof(*msg));
	for (fragment = 0; fragment < HISTB_MPEG4_SLICE_FRAGMENTS;
	     fragment++) {
		__u32 address_bits;
		__u32 aligned;
		__u32 word = fragment * 2;

		if (!slice->bit_len[fragment])
			continue;
		if (slice->bit_len[fragment] > 0xffffff)
			return HISTB_MPEG4_INVALID;
		aligned = slice->dma_addr[fragment] & ~15U;
		if (aligned < stream_base)
			return HISTB_MPEG4_INVALID;
		address_bits = slice->bit_offset[fragment] +
			       (slice->dma_addr[fragment] & 15) * 8;
		if (address_bits > 127)
			return HISTB_MPEG4_INVALID;
		msg->d[word] = (slice->bit_len[fragment] & 0xffffff) |
			       (address_bits << 24);
		msg->d[word + 1] = aligned - stream_base;
	}
	msg->d[4] = (slice->quant & 31) |
		    ((__u32)(slice->coding_type & 3) << 5) |
		    ((__u32)(slice->intra_dc_vlc_thr & 7) << 7) |
		    ((__u32)(slice->fcode_forward & 7) << 10) |
		    ((__u32)(slice->fcode_backward & 7) << 13);
	msg->d[5] = slice->mb_start & 0xfffff;
	msg->d[6] = width_mb * height_mb - 1;
	return HISTB_MPEG4_OK;
}

int histb_mpeg4_build_slice_messages(
				const struct histb_mpeg4_frame *frame,
				const struct histb_mpeg4_slice *slices,
				__u32 slice_count, __u32 stream_base,
				__u32 message_dma,
				struct histb_mpeg4_slice_msg *messages,
				__u32 message_capacity, __u32 *message_count)
{
	__u32 output = slices && slice_count && slices[0].mb_start ? 1 : 0;
	__u32 total_mbs;
	__u32 i;
	int ret;

	if (!frame || !slices || !slice_count || !messages || !message_count ||
	    slice_count > HISTB_MPEG4_MAX_SLICES ||
	    output + slice_count > message_capacity)
		return HISTB_MPEG4_INVALID;
	total_mbs = ((__u32)frame->vol.width + 15) / 16 *
		    (((__u32)frame->vol.height + 15) / 16);
	if (!total_mbs)
		return HISTB_MPEG4_INVALID;

	for (i = 0; i < slice_count; i++, output++) {
		struct histb_mpeg4_slice_msg *msg = &messages[output];
		__u32 end = i + 1 < slice_count ?
			    slices[i + 1].mb_start - 1 : total_mbs - 1;

		if (i && slices[i].mb_start <= slices[i - 1].mb_start)
			return HISTB_MPEG4_INVALID;
		ret = histb_mpeg4_build_slice_msg(frame, &slices[i],
						  stream_base, msg);
		if (ret)
			return ret;
		if (end < slices[i].mb_start || end >= total_mbs)
			return HISTB_MPEG4_INVALID;
		msg->d[6] = end;
		msg->d[7] = i + 1 < slice_count ?
			    message_dma + (output + 1) * sizeof(*msg) : 0;
	}

	output = slice_count + (slices[0].mb_start ? 1 : 0);
	if (slices[0].mb_start) {
		messages[0] = messages[1];
		messages[0].d[0] &= 0xff000000;
		messages[0].d[5] = 0;
		messages[0].d[6] = slices[0].mb_start - 1;
		messages[0].d[7] = message_dma + sizeof(messages[0]);
	}
	*message_count = output;
	return HISTB_MPEG4_OK;
}
