/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Hi3798MV310 AVS2, reconstructed from the shipped ARM32 VFMW v5.1 kernel.
 * Source kernel SHA256: 8adb8e7580bebce1716396271dbed3575fb870b536124608388f20f4404ce4a3.
 * Syntax: avs2_dec_decode c0762860; DPB: c0760bf0/c07614cc;
 * hardware messages: c0765d84/c0765ef8. No AVS/AVSP firmware is used.
 */
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/string.h>

#include "histb-vdec-avs2.h"

static const u8 wq_m8x8_idx_model0[] = {
	0, 0, 0, 4, 4, 4, 5, 5, 0, 0, 3, 3, 3, 3, 5, 5,
	0, 3, 2, 2, 1, 1, 5, 5, 4, 3, 2, 2, 1, 5, 5, 5,
	4, 3, 1, 1, 5, 5, 5, 5, 4, 3, 1, 5, 5, 5, 5, 5,
	5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
};

static const u8 wq_m4x4_idx_model0[] = {
	0, 4, 3, 5, 4, 2, 1, 5, 3, 1, 1, 5, 5, 5, 5, 5,
};

static const u8 wq_m8x8_idx_model1[] = {
	0, 0, 0, 4, 4, 4, 5, 5, 0, 0, 4, 4, 4, 4, 5, 5,
	0, 3, 2, 2, 2, 1, 5, 5, 3, 3, 2, 2, 1, 5, 5, 5,
	3, 3, 2, 1, 5, 5, 5, 5, 3, 3, 1, 5, 5, 5, 5, 5,
	5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
};

static const u8 wq_m4x4_idx_model1[] = {
	0, 4, 4, 5, 3, 2, 2, 5, 3, 2, 1, 5, 5, 5, 5, 5,
};

static const u8 wq_m8x8_idx_model2[] = {
	0, 0, 0, 4, 4, 3, 5, 5, 0, 0, 4, 4, 3, 2, 5, 5,
	0, 4, 4, 3, 2, 1, 5, 5, 4, 4, 3, 2, 1, 5, 5, 5,
	4, 3, 2, 1, 5, 5, 5, 5, 3, 2, 1, 5, 5, 5, 5, 5,
	5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
};

static const u8 wq_m4x4_idx_model2[] = {
	0, 4, 3, 5, 4, 3, 2, 5, 3, 2, 1, 5, 5, 5, 5, 5,
};

static const u8 wqm_4x4_default[] = {
	64, 64, 64, 68, 64, 64, 68, 72, 64, 68, 76, 80, 72, 76, 84, 96,
};

static const u8 wqm_8x8_default[] = {
	64, 64, 64, 64, 68, 68, 72, 76, 64, 64, 64, 68, 72, 76, 84, 92,
	64, 64, 68, 72, 76, 80, 88, 100, 64, 68, 72, 80, 84, 92, 100, 112,
	68, 72, 80, 84, 92, 104, 112, 128, 76, 80, 84, 92, 104, 116, 132, 152,
	96, 100, 104, 116, 124, 140, 164, 188, 104, 108, 116, 128, 152, 172, 192, 216,
};

static const u8 weight_quant_param_default[] = {
	64, 49, 53, 58, 58, 64,
};

static const u8 weight_quant_param_base1[] = {
	67, 71, 71, 80, 80, 106,
};

static const u8 alf_region_table[] = {
	0, 1, 4, 5, 15, 2, 3, 6, 14, 11, 10, 7, 13, 12, 9, 8,
};


struct avs2_bits {
	const u8 *data;
	u32 size, byte;
	u8 bit;
	bool epb;
	int error;
};

static u32 avs2_get(struct avs2_bits *b, unsigned int n)
{
	u32 value = 0;

	while (n--) {
		unsigned int bits;

		if (b->byte >= b->size) {
			b->error = -EINVAL;
			return 0;
		}
		bits = b->epb && b->byte >= 2 && !b->data[b->byte - 1] &&
		       !b->data[b->byte - 2] && b->data[b->byte] == 2 ? 6 : 8;
		value = (value << 1) | ((b->data[b->byte] >> (7 - b->bit)) & 1);
		if (++b->bit == bits) {
			b->byte++;
			b->bit = 0;
		}
	}
	return value;
}

static u32 avs2_ue(struct avs2_bits *b)
{
	unsigned int zeros = 0;

	while (!b->error && !avs2_get(b, 1)) {
		if (++zeros > 30) {
			b->error = -EINVAL;
			return 0;
		}
	}
	return ((1U << zeros) - 1) + avs2_get(b, zeros);
}

static s32 avs2_se(struct avs2_bits *b)
{
	u32 value = avs2_ue(b);

	return value & 1 ? (s32)((value + 1) >> 1) : -(s32)(value >> 1);
}

static bool avs2_tool(const struct histb_avs2_sequence *s, unsigned int tool)
{
	return !!(s->tools & BIT(tool));
}

static int avs2_rcs(struct avs2_bits *b, struct histb_avs2_rcs *r)
{
	unsigned int i;

	r->referred = avs2_get(b, 1);
	r->count = avs2_get(b, 3);
	for (i = 0; i < r->count; i++)
		r->delta[i] = avs2_get(b, 6);
	r->remove_count = avs2_get(b, 3);
	for (i = 0; i < r->remove_count; i++)
		r->remove[i] = avs2_get(b, 6);
	avs2_get(b, 1);
	return b->error;
}

static int avs2_matrix(struct avs2_bits *b, u8 *matrix, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		u32 value = avs2_ue(b);

		if (!value || value > 255)
			return -EINVAL;
		matrix[i] = value;
	}
	return b->error;
}

static int avs2_sequence(struct avs2_bits *b, struct histb_avs2_sequence *s)
{
	u32 value, pixels, limit;
	unsigned int i;
	int ret;

	memset(s, 0, sizeof(*s));
	s->profile = avs2_get(b, 8);
	if (s->profile != 0x12 && s->profile != 0x20 && s->profile != 0x22)
		return -EOPNOTSUPP;
	s->level = avs2_get(b, 8);
	if (!s->level)
		return -EINVAL;
	s->progressive = avs2_get(b, 1);
	s->field_coded = avs2_get(b, 1);
	if (s->progressive && s->field_coded)
		return -EINVAL;
	s->display_width = avs2_get(b, 14);
	s->display_height = avs2_get(b, 14);
	if (!s->display_width || !s->display_height ||
	    s->display_width > 4096 || s->display_height > 2304)
		return -EOPNOTSUPP;
	s->width = ALIGN(s->display_width, 8);
	s->height = ALIGN(s->display_height, 8);
	s->chroma_format = avs2_get(b, 2);
	s->sample_depth = 6 + 2 * avs2_get(b, 3);
	s->bit_depth = s->profile == 0x22 ? 6 + 2 * avs2_get(b, 3) : 8;
	if (s->chroma_format != 1 ||
	    (s->sample_depth != 8 && s->sample_depth != 10) ||
	    (s->bit_depth != 8 && s->bit_depth != 10) ||
	    s->sample_depth > s->bit_depth)
		return -EOPNOTSUPP;
	s->aspect_ratio = avs2_get(b, 4);
	s->frame_rate_code = avs2_get(b, 4);
	avs2_get(b, 18); /* bit_rate_lower */
	avs2_get(b, 1);
	avs2_get(b, 12); /* bit_rate_upper */
	s->low_delay = avs2_get(b, 1);
	avs2_get(b, 1);
	s->temporal_id_enable = avs2_get(b, 1);
	avs2_get(b, 18); /* bbv_buffer_size */
	s->lcu_log2 = avs2_get(b, 3);
	if (s->lcu_log2 < 4 || s->lcu_log2 > 6)
		return -EOPNOTSUPP;
	s->weight_quant = avs2_get(b, 1);
	if (s->weight_quant) {
		s->load_wq = avs2_get(b, 1);
		if (s->load_wq) {
			ret = avs2_matrix(b, s->wq4, 16);
			if (ret)
				return ret;
			ret = avs2_matrix(b, s->wq8, 64);
			if (ret)
				return ret;
		}
	}
	for (i = HISTB_AVS2_SCENE; i <= HISTB_AVS2_W_SKIP; i++)
		s->tools |= avs2_get(b, 1) << i;
	for (i = HISTB_AVS2_AMP; i <= HISTB_AVS2_PMVR; i++)
		s->tools |= avs2_get(b, 1) << i;
	avs2_get(b, 1);
	value = avs2_get(b, 6);
	if (value > HISTB_AVS2_RCS)
		return -EINVAL;
	s->rcs_count = value;
	for (i = 0; i < s->rcs_count; i++) {
		ret = avs2_rcs(b, &s->rcs[i]);
		if (ret)
			return ret;
	}
	if (!s->low_delay)
		s->reorder_delay = avs2_get(b, 5);
	s->cross_slice_filter = avs2_get(b, 1);
	pixels = s->width * s->height;
	if (s->level < 0x23) {
		s->max_dpb = 15;
	} else {
		limit = s->level < 0x4b ? 0xcc0000 :
			s->level < 0x5b ? 0x3600000 : 0xcc00000;
		s->max_dpb = clamp_t(u32, min_t(u32, limit / pixels, 16) - 1,
				    pixels < 0x1fe001 ? 7 : 6,
				    pixels < 0x1fe001 ? 15 : 10);
	}
	return b->error;
}

static int avs2_wq(struct avs2_bits *b, const struct histb_avs2_sequence *s,
		   struct histb_avs2_picture *p)
{
	const u8 *m4, *m8, *base;
	u8 params[6];
	unsigned int index, model, mode, i;

	p->wq_shift = 2;
	p->weight_quant = s->weight_quant ? avs2_get(b, 1) : 0;
	if (!p->weight_quant) {
		memset(p->wq4, 64, sizeof(p->wq4));
		memset(p->wq8, 64, sizeof(p->wq8));
		return b->error;
	}
	index = avs2_get(b, 2);
	if (!index) {
		memcpy(p->wq4, s->load_wq ? s->wq4 : wqm_4x4_default, 16);
		memcpy(p->wq8, s->load_wq ? s->wq8 : wqm_8x8_default, 64);
		return b->error;
	}
	if (index == 2) {
		int ret = avs2_matrix(b, p->wq4, 16);

		return ret ? ret : avs2_matrix(b, p->wq8, 64);
	}
	if (index != 1)
		return -EINVAL;
	avs2_get(b, 1);
	mode = avs2_get(b, 2);
	model = avs2_get(b, 2);
	if (mode > 2 || model > 2)
		return -EINVAL;
	base = mode == 1 ? weight_quant_param_base1 : weight_quant_param_default;
	for (i = 0; i < 6; i++) {
		int value = base[i] + (mode ? avs2_se(b) : 0);

		if (value < 1 || value > 255)
			return -EINVAL;
		params[i] = value;
	}
	m4 = model == 0 ? wq_m4x4_idx_model0 :
		model == 1 ? wq_m4x4_idx_model1 : wq_m4x4_idx_model2;
	m8 = model == 0 ? wq_m8x8_idx_model0 :
		model == 1 ? wq_m8x8_idx_model1 : wq_m8x8_idx_model2;
	for (i = 0; i < 16; i++)
		p->wq4[i] = params[m4[i]];
	for (i = 0; i < 64; i++)
		p->wq8[i] = params[m8[i]];
	return b->error;
}

static int avs2_alf_coeff(struct avs2_bits *b, s16 *coeff)
{
	int sum = 0, value;
	unsigned int i;

	for (i = 0; i < 8; i++) {
		value = avs2_se(b);
		if (value < -64 || value > 63)
			return -EINVAL;
		coeff[i] = value;
		sum += value;
	}
	value = avs2_se(b);
	if (value < -1088 || value > 1071)
		return -EINVAL;
	coeff[8] = value + 64 - 2 * sum;
	return b->error;
}

static int avs2_alf(struct avs2_bits *b, const struct histb_avs2_sequence *s,
		    struct histb_avs2_picture *p)
{
	s16 coeff[16][9];
	u8 region[16] = { 0 };
	unsigned int count, group, boundary = 0, i;
	int ret;

	if (!avs2_tool(s, HISTB_AVS2_ALF))
		return 0;
	for (i = 0; i < 3; i++)
		p->alf_enable[i] = avs2_get(b, 1);
	if (p->alf_enable[0]) {
		count = avs2_ue(b) + 1;
		if (!count || count > 16)
			return -EINVAL;
		for (group = 0; group < count; group++) {
			if (group) {
				u32 distance = count == 16 ? 1 : avs2_ue(b);

				if (!distance || distance > 15 || boundary + distance > 15)
					return -EINVAL;
				boundary += distance;
			}
			for (i = boundary; i < 16; i++)
				region[i] = group;
			ret = avs2_alf_coeff(b, coeff[group]);
			if (ret)
				return ret;
		}
		for (i = 0; i < 16; i++)
			memcpy(p->alf_luma[i], coeff[region[alf_region_table[i]]],
			       sizeof(p->alf_luma[i]));
	}
	for (i = 0; i < 2; i++) {
		if (!p->alf_enable[i + 1])
			continue;
		ret = avs2_alf_coeff(b, p->alf_chroma[i]);
		if (ret)
			return ret;
	}
	return b->error;
}

static int avs2_picture(struct avs2_bits *b, const struct histb_avs2_sequence *s,
			struct histb_avs2_picture *p, bool intra)
{
	unsigned int coding = 0, scene = 0, scene_output = 0, scene_pred = 0;
	u32 index;
	int value, ret;

	memset(p, 0, sizeof(*p));
	p->intra = intra;
	b->epb = true;
	avs2_get(b, 32); /* bbv_delay; selected profiles never have 0x48 extension */
	if (intra) {
		if (avs2_get(b, 1))
			avs2_get(b, 24);
		if (!avs2_tool(s, HISTB_AVS2_SCENE)) {
			scene = avs2_get(b, 1);
			if (scene)
				scene_output = avs2_get(b, 1);
		}
	} else {
		coding = avs2_get(b, 2);
		if (!coding)
			return -EINVAL;
		if (!avs2_tool(s, HISTB_AVS2_SCENE) && coding == 1)
			scene_pred = avs2_get(b, 1);
		if (!avs2_tool(s, HISTB_AVS2_SCENE) && coding != 2 && !scene_pred)
			p->scene_reference = avs2_get(b, 1);
	}
	p->doi = avs2_get(b, 8);
	if (s->temporal_id_enable)
		p->temporal_id = avs2_get(b, 3);
	if (!s->low_delay && (!intra || !scene || scene_output)) {
		index = avs2_ue(b);
		if (index > 63)
			return -EINVAL;
		p->output_delay = index;
	}
	if (avs2_get(b, 1)) {
		index = avs2_get(b, 5);
		if (index >= s->rcs_count)
			return -EINVAL;
		p->rcs = s->rcs[index];
	} else {
		ret = avs2_rcs(b, &p->rcs);
		if (ret)
			return ret;
	}
	if (s->low_delay)
		avs2_ue(b); /* bbv_check_times */
	p->progressive = avs2_get(b, 1);
	if (s->progressive && !p->progressive)
		return -EINVAL;
	if (s->field_coded && p->progressive)
		return -EINVAL;
	p->structure = p->progressive ? 1 : avs2_get(b, 1);
	p->top_first = avs2_get(b, 1);
	p->repeat_first = avs2_get(b, 1);
	p->top_field = 1;
	if (s->field_coded) {
		p->top_field = avs2_get(b, 1);
		avs2_get(b, 1);
	}
	p->fixed_qp = avs2_get(b, 1);
	p->qp = avs2_get(b, 7);
	if (p->qp > 8 * s->bit_depth - 1)
		return -EINVAL;
	if (!intra) {
		if (coding != 2 || p->structure != 1)
			p->skip = avs2_get(b, 1);
		p->random_access = avs2_get(b, 1);
	}
	p->loop_disable = avs2_get(b, 1);
	if (!p->loop_disable) {
		p->loop_parameters = avs2_get(b, 1);
		if (p->loop_parameters) {
			value = avs2_se(b);
			if (value < -8 || value > 8)
				return -EINVAL;
			p->alpha = value;
			value = avs2_se(b);
			if (value < -8 || value > 8)
				return -EINVAL;
			p->beta = value;
		}
	}
	p->chroma_disable = avs2_get(b, 1);
	if (!p->chroma_disable) {
		value = avs2_se(b);
		if (value < -16 || value > 16)
			return -EINVAL;
		p->chroma_cb = value;
		value = avs2_se(b);
		if (value < -16 || value > 16)
			return -EINVAL;
		p->chroma_cr = value;
	}
	ret = avs2_wq(b, s, p);
	if (ret)
		return ret;
	ret = avs2_alf(b, s, p);
	if (ret)
		return ret;
	p->type = intra ? (scene ? (scene_output ? HISTB_AVS2_G : HISTB_AVS2_GB) :
			   HISTB_AVS2_I) : (scene_pred ? HISTB_AVS2_S : coding);
	if (!intra && !p->rcs.count && p->type != HISTB_AVS2_S)
		return -EINVAL;
	return b->error;
}

static u32 avs2_start(const u8 *data, u32 size, u32 pos)
{
	while (pos < size && size - pos >= 4) {
		if (!data[pos] && !data[pos + 1] && data[pos + 2] == 1)
			return pos;
		pos++;
	}
	return size;
}

void histb_avs2_init(struct histb_avs2_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->last_doi = -1;
}

u16 histb_avs2_busy_slots(const struct histb_avs2_ctx *ctx)
{
	u16 mask = 0;
	unsigned int i;

	for (i = 0; i < HISTB_AVS2_SLOTS; i++) {
		const struct histb_avs2_frame *f = &ctx->frame[i];

		if (f->reference || f->scene || f->output)
			mask |= BIT(i);
	}
	return mask;
}

int histb_avs2_parse(struct histb_avs2_ctx *ctx, const u8 *data, u32 size,
		     struct histb_avs2_au *au)
{
	u32 pos, end, next;
	bool valid = ctx->sequence_valid;
	int ret = 0;

	if (!data || !size || size > U32_MAX / 8 || ctx->pending)
		return -EINVAL;
	memset(au, 0, sizeof(*au));
	au->sequence = ctx->sequence;
	au->size = size;
	pos = avs2_start(data, size, 0);
	if (pos != 0)
		return -EINVAL;
	while (pos < size) {
		struct avs2_bits b;
		u8 code = data[pos + 3];

		next = avs2_start(data, size, pos + 4);
		end = next;
		b = (struct avs2_bits) { .data = data + pos + 4, .size = end - pos - 4 };
		if (code == 0xb0) {
			if (au->picture_present)
				return -EINVAL;
			ret = avs2_sequence(&b, &au->sequence);
			if (ret)
				return ret;
			au->sequence_present = true;
			au->reset_before = true;
			valid = true;
		} else if (code == 0xb3 || code == 0xb6) {
			if (!valid || au->picture_present)
				return -EINVAL;
			ret = avs2_picture(&b, &au->sequence, &au->picture, code == 0xb3);
			if (ret)
				return ret;
			au->picture_present = true;
		} else if (code < 0x90) {
			struct histb_avs2_slice *sl;
			u32 x, y, width, height;

			if (!au->picture_present || au->slice_count >= HISTB_AVS2_SLICES)
				return -EINVAL;
			y = code;
			if (au->sequence.height > (0x90U << au->sequence.lcu_log2))
				y += avs2_get(&b, 3) << 7;
			x = avs2_get(&b, 8);
			if (au->sequence.width > (0xffU << au->sequence.lcu_log2))
				x += avs2_get(&b, 2) << 8;
			width = DIV_ROUND_UP(au->sequence.width, 1U << au->sequence.lcu_log2);
			height = DIV_ROUND_UP(au->sequence.height, 1U << au->sequence.lcu_log2);
			if (b.error || x >= width || y >= height || end - pos < 4)
				return -EINVAL;
			sl = &au->slice[au->slice_count];
			sl->first_lcu = y * width + x;
			if ((!au->slice_count && sl->first_lcu) ||
			    (au->slice_count && sl->first_lcu <= sl[-1].first_lcu))
				return -EINVAL;
			sl->x = x;
			sl->y = y;
			sl->offset = pos;
			sl->size = end - pos;
			au->slice_count++;
		} else if (code == 0xb1) {
			au->sequence_end = true;
		} else if (code == 0xb7) {
			au->video_edit = true;
			au->reset_before |= !au->picture_present;
		} else if (code != 0xb2 && code != 0xb5) {
			return -EINVAL;
		}
		pos = next;
	}
	if (au->picture_present && !au->slice_count)
		return -EINVAL;
	if (au->sequence_present) {
		ctx->sequence = au->sequence;
		ctx->sequence_valid = true;
	}
	if (au->reset_before) {
		if (histb_avs2_busy_slots(ctx))
			ctx->reset_required = true;
		else
			histb_avs2_clear_dpb(ctx);
	}
	return au->picture_present ? 1 : 0;
}

static int avs2_find_ref(const struct histb_avs2_ctx *ctx, s32 doi)
{
	unsigned int i;

	for (i = 0; i < HISTB_AVS2_SLOTS; i++) {
		const struct histb_avs2_frame *f = &ctx->frame[i];

		if (f->reference && f->doi == doi && f->type != HISTB_AVS2_GB)
			return i;
	}
	return -ENOENT;
}

static int avs2_scene_ref(const struct histb_avs2_ctx *ctx)
{
	unsigned int i;

	for (i = 0; i < HISTB_AVS2_SLOTS; i++)
		if (ctx->frame[i].scene)
			return i;
	return -ENOENT;
}

int histb_avs2_prepare(struct histb_avs2_ctx *ctx, const struct histb_avs2_au *au,
		       u8 slot, u64 timestamp, struct histb_avs2_job *job)
{
	const struct histb_avs2_picture *p = &au->picture;
	const struct histb_avs2_sequence *s = &au->sequence;
	struct histb_avs2_frame *f = &job->frame;
	s32 epoch = ctx->doi_epoch, doi;
	unsigned int i, regular;
	bool reuse = false;
	int ref;

	if (ctx->pending || !ctx->sequence_valid || !au->picture_present ||
	    !au->slice_count || slot >= HISTB_AVS2_SLOTS)
		return -EINVAL;
	if (ctx->reset_required)
		return -EPIPE;
	if (ctx->last_doi >= 0 && p->doi < ctx->last_doi) {
		if (epoch > S32_MAX - 256)
			return -EOVERFLOW;
		epoch += 256;
	}
	if (epoch > S32_MAX - p->doi - p->output_delay)
		return -EOVERFLOW;
	doi = epoch + p->doi;
	if (p->type == HISTB_AVS2_GB) {
		for (i = 0; i < HISTB_AVS2_SLOTS; i++) {
			if (ctx->frame[i].scene && ctx->frame[i].type == HISTB_AVS2_GB &&
			    ctx->frame[i].doi == doi) {
				slot = i;
				reuse = true;
				break;
			}
		}
	}
	if (!reuse && (histb_avs2_busy_slots(ctx) & BIT(slot)))
		return -EBUSY;
	memset(job, 0, sizeof(*job));
	job->au = au;
	job->slot = slot;
	f->doi = doi;
	f->poi = doi + p->output_delay - s->reorder_delay;
	f->output_at = doi + p->output_delay;
	f->type = p->type;
	f->top_field = p->top_field;
	f->structure = p->structure;
	f->progressive = p->progressive;
	f->top_first = p->top_first;
	f->reference = p->rcs.referred;
	f->scene = p->type == HISTB_AVS2_G || p->type == HISTB_AVS2_GB;
	f->output = p->type != HISTB_AVS2_GB;
	f->timestamp = timestamp;
	job->ref_count = p->intra ? 0 : p->type == HISTB_AVS2_S ? 1 : p->rcs.count;
	if (job->ref_count > HISTB_AVS2_REFS)
		return -EINVAL;
	regular = job->ref_count;
	if (!p->intra && (p->scene_reference || p->type == HISTB_AVS2_S)) {
		if (!regular)
			return -EINVAL;
		regular--;
		ref = avs2_scene_ref(ctx);
		if (ref < 0)
			return ref;
		job->ref[regular] = ref;
		job->references |= BIT(ref);
	}
	for (i = 0; i < regular; i++) {
		if (!p->rcs.delta[i])
			return -EINVAL;
		ref = avs2_find_ref(ctx, doi - p->rcs.delta[i]);
		if (ref < 0)
			return ref;
		job->ref[i] = ref;
		job->references |= BIT(ref);
	}
	for (i = 0; !p->intra && i < HISTB_AVS2_REFS; i++) {
		s32 distance = 1;

		if (i < job->ref_count) {
			const struct histb_avs2_frame *r = &ctx->frame[job->ref[i]];
			s64 current_index = 2LL * f->poi + (s->field_coded && !f->top_field);
			s64 ref_index = 2LL * r->poi + (s->field_coded && !r->top_field);

			f->ref_top_field[i] = r->top_field;
			job->ref_flags[i] = r->top_field | r->structure << 1 | r->top_first << 3;
			distance = (r->poi < f->poi ? current_index - ref_index :
				    ref_index - current_index) % 512;
			if (!distance)
				distance = 1;
		}
		f->distance[i] = distance;
		f->reciprocal[i] = 0x4000 / distance;
		job->distance[i] = distance;
		job->reciprocal[i] = f->reciprocal[i];
		if (job->ref_count) {
			const struct histb_avs2_frame *col = &ctx->frame[job->ref[0]];

			job->col_top_field[i] = col->ref_top_field[i];
			job->col_distance[i] = col->distance[i];
			job->col_reciprocal[i] = col->reciprocal[i];
		}
	}
	if (p->intra)
		job->col_top_field[0] = p->top_field;
	if (p->scene_reference) {
		if (job->ref_count == 1) {
			for (i = 0; i < HISTB_AVS2_REFS; i++) {
				job->distance[i] = 1;
				job->col_distance[i] = 1;
				job->reciprocal[i] = 0x4000;
				job->col_reciprocal[i] = 0x4000;
			}
		} else {
			i = job->ref_count - 1;
			job->distance[i] = 1;
			job->reciprocal[i] = 0x4000;
		}
	}
	for (i = 0; i < p->rcs.remove_count; i++) {
		s32 removed_doi = doi - p->rcs.remove[i];
		unsigned int j;

		for (j = 0; j < HISTB_AVS2_SLOTS; j++)
			if (ctx->frame[j].doi == removed_doi)
				job->remove |= BIT(j);
	}
	ctx->pending = true;
	return 0;
}

void histb_avs2_complete(struct histb_avs2_ctx *ctx,
		       const struct histb_avs2_job *job, bool success)
{
	unsigned int i;

	if (!ctx->pending)
		return;
	ctx->pending = false;
	if (!success)
		return;
	for (i = 0; i < HISTB_AVS2_SLOTS; i++) {
		struct histb_avs2_frame *f = &ctx->frame[i];

		if (job->remove & BIT(i))
			f->reference = false;
		if (job->frame.scene)
			f->scene = false;
	}
	ctx->frame[job->slot] = job->frame;
	ctx->last_doi = job->au->picture.doi;
	ctx->doi_epoch = job->frame.doi - ctx->last_doi;
	ctx->current_doi = job->frame.doi;
}

int histb_avs2_next_output(struct histb_avs2_ctx *ctx, bool drain,
			  struct histb_avs2_output *output)
{
	int best = -1;
	unsigned int i;

	if (ctx->pending)
		return -EBUSY;
	for (i = 0; i < HISTB_AVS2_SLOTS; i++) {
		const struct histb_avs2_frame *f = &ctx->frame[i];

		if (!f->output || (!drain && f->output_at > ctx->current_doi))
			continue;
		if (best < 0 || f->poi < ctx->frame[best].poi)
			best = i;
	}
	if (best < 0)
		return 0;
	output->slot = best;
	output->timestamp = ctx->frame[best].timestamp;
	output->poi = ctx->frame[best].poi;
	output->type = ctx->frame[best].type;
	output->top_field = ctx->frame[best].top_field;
	output->structure = ctx->frame[best].structure;
	output->progressive = ctx->frame[best].progressive;
	output->top_first = ctx->frame[best].top_first;
	ctx->frame[best].output = false;
	return 1;
}

void histb_avs2_clear_dpb(struct histb_avs2_ctx *ctx)
{
	memset(ctx->frame, 0, sizeof(ctx->frame));
	ctx->doi_epoch = 0;
	ctx->last_doi = -1;
	ctx->current_doi = 0;
	ctx->pending = false;
	ctx->reset_required = false;
}

static void avs2_pack_alf(u32 *dest, const s16 *coeff)
{
	dest[0] = ((u32)coeff[0] & 0x7f) | (((u32)coeff[1] & 0x7f) << 7) |
		  (((u32)coeff[2] & 0x7f) << 14) | (((u32)coeff[3] & 0x7f) << 21) |
		  ((u32)coeff[8] << 28);
	dest[1] = ((u32)coeff[4] & 0x7f) | (((u32)coeff[5] & 0x7f) << 7) |
		  (((u32)coeff[6] & 0x7f) << 14) | (((u32)coeff[7] & 0x7f) << 21) |
		  (((u32)coeff[8] & 0x70) << 24);
}

int histb_avs2_build_messages(const struct histb_avs2_job *job,
		 const struct histb_avs2_surface surface[HISTB_AVS2_SLOTS],
		 const struct histb_avs2_scratch *scratch, u32 stream_dma,
		 u32 *pic, u32 *slices)
{
	const struct histb_avs2_au *au = job->au;
	const struct histb_avs2_sequence *s = &au->sequence;
	const struct histb_avs2_picture *p = &au->picture;
	const struct histb_avs2_surface *cur = &surface[job->slot];
	u32 lcu_width = DIV_ROUND_UP(s->width, 1U << s->lcu_log2);
	u32 lcu_height = DIV_ROUND_UP(s->height, 1U << s->lcu_log2);
	u32 structure = s->field_coded ? (p->top_field ? 1 : 2) : 0;
	unsigned int i;

	if (!pic || !slices || !au->slice_count ||
	    au->slice_count > HISTB_AVS2_SLICES ||
	    (stream_dma & 15) || au->size > U32_MAX - stream_dma ||
	    ((cur->luma | cur->pmv | scratch->sed_top | scratch->pmv_top |
	      scratch->rcn_top | scratch->sao_top | scratch->dblk_top |
	      scratch->slices) & 15) ||
	    scratch->slices > U32_MAX - au->slice_count * 256)
		return -EINVAL;
	memset(pic, 0, HISTB_AVS2_PIC_WORDS * sizeof(*pic));
	memset(slices, 0, au->slice_count * HISTB_AVS2_SLICE_WORDS * sizeof(*slices));
	pic[0] = (lcu_width - 1) | ((lcu_height - 1) << 16);
	pic[1] = p->top_field << 20 | p->qp << 9 | p->fixed_qp << 8 |
		 p->type << 3 | structure << 1 | s->bit_depth << 16;
	pic[2] = s->cross_slice_filter << 19 |
		 avs2_tool(s, HISTB_AVS2_NSQT) << 18 | s->lcu_log2 |
		 s->chroma_format << 16 | avs2_tool(s, HISTB_AVS2_NSIP) << 15 |
		 avs2_tool(s, HISTB_AVS2_MH_SKIP) << 14 |
		 avs2_tool(s, HISTB_AVS2_W_SKIP) << 13 |
		 avs2_tool(s, HISTB_AVS2_DH_PRED) << 12 | job->ref_count << 9 |
		 avs2_tool(s, HISTB_AVS2_AMP) << 8 |
		 (p->alf_enable[0] | p->alf_enable[1] << 1 | p->alf_enable[2] << 2) << 5 |
		 avs2_tool(s, HISTB_AVS2_SAO) << 4 | avs2_tool(s, HISTB_AVS2_ALF) << 3;
	pic[3] = avs2_tool(s, HISTB_AVS2_SECONDARY) << 15 | s->weight_quant << 14 |
		 ((u32)p->chroma_cr & 63) << 8 | ((u32)p->chroma_cb & 63) << 2 |
		 p->chroma_disable << 1 | p->weight_quant;
	pic[4] = ((u32)p->beta & 31) << 7 | ((u32)p->alpha & 31) << 2 |
		 p->loop_parameters << 1 | p->loop_disable;
	pic[5] = p->top_field << 3 | p->top_first;
	for (i = 0; i < HISTB_AVS2_REFS; i++) {
		u32 addr = p->intra && !i ? cur->luma : 0;

		if (i < job->ref_count) {
			addr = surface[job->ref[i]].luma;
			if (addr & 15)
				return -EINVAL;
		}
		pic[6 + i] = addr >> 4;
		if (i < job->ref_count || (p->intra && !i)) {
			u32 flags = p->intra ? p->top_field | p->structure << 1 |
				p->top_first << 3 : job->ref_flags[i];

			pic[5] |= (flags & 1) << (7 + 4 * i) |
				 ((flags >> 1) & 3) << (5 + 4 * i) |
				 ((flags >> 3) & 1) << (4 + 4 * i);
		}
	}
	pic[13] = s->width | s->height << 16;
	pic[14] = (DIV_ROUND_UP(s->width, 8) - 1) |
		  (DIV_ROUND_UP(s->height, 8) - 1) << 16;
	pic[16] = scratch->pmv_top >> 4;
	pic[17] = scratch->rcn_top >> 4;
	pic[18] = scratch->sao_top >> 4;
	pic[19] = scratch->dblk_top >> 4;
	pic[20] = cur->pmv >> 4;
	if (job->ref_count) {
		if (surface[job->ref[0]].pmv & 15)
			return -EINVAL;
		pic[21] = surface[job->ref[0]].pmv >> 4;
	}
	for (i = 0; i < 16; i++)
		avs2_pack_alf(pic + 0x16 + i * 2, p->alf_luma[i]);
	for (i = 0; i < 2; i++)
		avs2_pack_alf(pic + 0x36 + i * 2, p->alf_chroma[i]);
	pic[0x3f] = scratch->slices >> 4;
	for (i = 0; i < 4; i++) {
		pic[0x40 + i] = ((u32)job->distance[2 * i] & 0x3ff) |
			((u32)job->distance[2 * i + 1] & 0x3ff) << 16;
		pic[0x44 + i] = ((u32)job->col_distance[2 * i] & 0x3ff) |
			((u32)job->col_distance[2 * i + 1] & 0x3ff) << 16;
		pic[0x48 + i] = ((u32)job->reciprocal[2 * i] & 0x7fff) |
			((u32)job->reciprocal[2 * i + 1] & 0x7fff) << 16;
		pic[0x4c + i] = ((u32)job->col_reciprocal[2 * i] & 0x7fff) |
			((u32)job->col_reciprocal[2 * i + 1] & 0x7fff) << 16;
	}
	pic[0x59] = p->wq_shift << 9 | avs2_tool(s, HISTB_AVS2_PMVR) << 8 |
		    p->scene_reference << 7;
	for (i = 0; i < HISTB_AVS2_REFS; i++)
		pic[0x59] |= job->col_top_field[i] << i;
	for (i = 0; i < 4; i++) {
		pic[0x5a + i * 2] = p->wq4[i] | p->wq4[i + 8] << 8;
		pic[0x5b + i * 2] = p->wq4[i + 4] | p->wq4[i + 12] << 8;
	}
	for (i = 0; i < 8; i++) {
		pic[0x62 + i * 4] = p->wq8[i] | p->wq8[i + 16] << 8;
		pic[0x63 + i * 4] = p->wq8[i + 32] | p->wq8[i + 48] << 8;
		pic[0x64 + i * 4] = p->wq8[i + 8] | p->wq8[i + 24] << 8;
		pic[0x65 + i * 4] = p->wq8[i + 40] | p->wq8[i + 56] << 8;
		pic[0x82 + i * 4] = p->wq8[i] | p->wq8[i + 8] << 8;
		pic[0x83 + i * 4] = p->wq8[i + 16] | p->wq8[i + 24] << 8;
		pic[0x84 + i * 4] = p->wq8[i + 32] | p->wq8[i + 40] << 8;
		pic[0x85 + i * 4] = p->wq8[i + 48] | p->wq8[i + 56] << 8;
	}
	memcpy(pic + 0xc0, pic + 0x5a, 0x120);
	for (i = 0; i < au->slice_count; i++) {
		const struct histb_avs2_slice *sl = &au->slice[i];
		u32 *msg = slices + i * HISTB_AVS2_SLICE_WORDS;
		if (sl->offset > au->size || sl->size < 3 ||
		    sl->size > au->size - sl->offset ||
		    sl->offset > U32_MAX - 3 ||
		    sl->size - 3 > U32_MAX / 8 ||
		    sl->x >= lcu_width || sl->y >= lcu_height ||
		    sl->first_lcu != sl->y * lcu_width + sl->x ||
		    (i && sl->first_lcu <= au->slice[i - 1].first_lcu) ||
		    (i + 1 < au->slice_count &&
		     au->slice[i + 1].first_lcu <= sl->first_lcu))
			return -EINVAL;
		msg[0] = (sl->offset + 3) >> 4;
		msg[1] = ((sl->offset + 3) & 15) << 3;
		msg[2] = (sl->size - 3) * 8;
		msg[7] = sl->first_lcu;
		msg[8] = sl->x | sl->y << 16;
		msg[9] = i + 1 < au->slice_count ? au->slice[i + 1].first_lcu - 1 :
			 lcu_width * lcu_height - 1;
		msg[63] = i + 1 < au->slice_count ?
			(scratch->slices + (i + 1) * 256) >> 4 : 0;
	}
	return 0;
}
