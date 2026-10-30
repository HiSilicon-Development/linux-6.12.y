// SPDX-License-Identifier: GPL-2.0-only
/*
 * MPEG-1 complete-AU ingress for the MV310's native MPEG1/2 VDH path.
 * Hardware authority: Android9 kernel SHA256
 * 8adb8e7580bebce1716396271dbed3575fb870b536124608388f20f4404ce4a3,
 * mp2_hal_cfg_dn_msg@c0752c10, mp2_hal_write_slice_msg@c0752474,
 * mp2_hal_start_dec@c0753040. D0[25]=parameter+0xb0 Mpeg1Flag;
 * D1[27]=parameter[2] Mp1FwdmvFullPel, D1[28]=parameter[1]
 * Mp1BwdmvFullPel. These independent bits leave all four f_codes unchanged.
 * The selected BSP decparam.h MP2_DEC_PARAM_S supplies the field names;
 * addresses and bit placements come from the selected native artifact, not
 * the comparative CV200 ABI. No different MPEG1 launch standard is selected.
 */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "histb-vdec-mpeg1.h"
#include "histb-vdec-v5r6.h"

static const u8 zigzag[64] = {
	0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
	12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

static const u8 default_intra[64] = {
	8, 16, 19, 22, 26, 27, 29, 34,
	16, 16, 22, 24, 27, 29, 34, 37,
	19, 22, 26, 27, 29, 34, 34, 38,
	22, 22, 26, 27, 29, 34, 37, 40,
	22, 26, 27, 29, 32, 35, 40, 48,
	26, 27, 29, 32, 35, 40, 48, 58,
	26, 27, 29, 34, 38, 46, 56, 69,
	27, 29, 35, 38, 46, 56, 69, 83,
};

static const struct {
	u16 code;
	u8 bits;
} mbaddr[33] = {
	{1, 1}, {3, 3}, {2, 3}, {3, 4}, {2, 4}, {3, 5}, {2, 5},
	{7, 7}, {6, 7}, {11, 8}, {10, 8}, {9, 8}, {8, 8}, {7, 8},
	{6, 8}, {23, 10}, {22, 10}, {21, 10}, {20, 10}, {19, 10},
	{18, 10}, {35, 11}, {34, 11}, {33, 11}, {32, 11}, {31, 11},
	{30, 11}, {29, 11}, {28, 11}, {27, 11}, {26, 11}, {25, 11},
	{24, 11},
};

struct bits {
	const u8 *data;
	u32 pos, end;
	bool error;
};

static u32 get(struct bits *b, u8 n)
{
	u32 v = 0;

	if (b->error || n > 32 || n > b->end - b->pos) {
		b->error = true;
		return 0;
	}
	while (n--) {
		v = (v << 1) | ((b->data[b->pos / 8] >>
					(7 - (b->pos & 7))) & 1);
		b->pos++;
	}
	return v;
}

static int padding(struct bits *b)
{
	while (!b->error && b->pos < b->end)
		if (get(b, 1))
			return -EINVAL;
	return b->error ? -EINVAL : 0;
}

static u32 next_code(const u8 *data, u32 start, u32 size)
{
	u32 i;

	for (i = start; i + 3 < size; i++)
		if (!data[i] && !data[i + 1] && data[i + 2] == 1)
			return i;
	return size;
}

static int sequence_header(struct bits *b, struct histb_mpeg1_sequence *s)
{
	u32 i, load;

	memset(s, 0, sizeof(*s));
	s->control.horizontal_size = get(b, 12);
	s->control.vertical_size = get(b, 12);
	s->aspect_ratio = get(b, 4);
	s->frame_rate_code = get(b, 4);
	s->bit_rate = get(b, 18);
	if (get(b, 1) != 1)
		return -EINVAL;
	s->vbv_buffer_size = get(b, 10);
	s->constrained = get(b, 1);
	s->control.chroma_format = 1;
	s->control.flags = V4L2_MPEG2_SEQ_FLAG_PROGRESSIVE;
	load = get(b, 1);
	for (i = 0; i < 64; i++) {
		u8 v = load ? get(b, 8) : default_intra[zigzag[i]];

		if (!v)
			return -EINVAL;
		s->quantisation.intra_quantiser_matrix[i] = v;
		s->quantisation.chroma_intra_quantiser_matrix[i] = v;
	}
	load = get(b, 1);
	for (i = 0; i < 64; i++) {
		u8 v = load ? get(b, 8) : 16;

		if (!v)
			return -EINVAL;
		s->quantisation.non_intra_quantiser_matrix[i] = v;
		s->quantisation.chroma_non_intra_quantiser_matrix[i] = v;
	}
	if (!s->control.horizontal_size || !s->control.vertical_size ||
	    !s->aspect_ratio || s->aspect_ratio == 15 ||
	    !s->frame_rate_code || s->frame_rate_code > 8 || !s->bit_rate)
		return -EINVAL;
	if (s->control.horizontal_size > 1920 || s->control.vertical_size > 1152)
		return -EOPNOTSUPP;
	return padding(b);
}

static int extra_information(struct bits *b)
{
	while (!b->error && get(b, 1))
		get(b, 8);
	return b->error ? -EINVAL : 0;
}

static int picture_header(struct bits *b, struct histb_mpeg1_au *au)
{
	struct v4l2_ctrl_mpeg2_picture *p = &au->picture;
	u32 code;

	au->temporal_reference = get(b, 10);
	p->picture_coding_type = get(b, 3);
	au->vbv_delay = get(b, 16);
	if (!p->picture_coding_type || p->picture_coding_type > 4)
		return -EINVAL;
	if (p->picture_coding_type == 4)
		return -EOPNOTSUPP;
	p->picture_structure = V4L2_MPEG2_PIC_FRAME;
	p->flags = V4L2_MPEG2_PIC_FLAG_PROGRESSIVE |
		V4L2_MPEG2_PIC_FLAG_FRAME_PRED_DCT;
	memset(p->f_code, 1, sizeof(p->f_code));
	if (p->picture_coding_type >= V4L2_MPEG2_PIC_CODING_TYPE_P) {
		au->full_pel_forward = get(b, 1);
		code = get(b, 3);
		if (!code)
			return -EINVAL;
		p->f_code[0][0] = code;
		p->f_code[0][1] = code;
	}
	if (p->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_B) {
		au->full_pel_backward = get(b, 1);
		code = get(b, 3);
		if (!code)
			return -EINVAL;
		p->f_code[1][0] = code;
		p->f_code[1][1] = code;
	}
	if (extra_information(b))
		return -EINVAL;
	return padding(b);
}

static int address_increment(struct bits *b, u32 limit, u32 *increment)
{
	u32 sum = 0, code;
	unsigned int i;

	while (!b->error) {
		for (i = 0; i < ARRAY_SIZE(mbaddr); i++) {
			struct bits probe = *b;

			code = get(&probe, mbaddr[i].bits);
			if (!probe.error && code == mbaddr[i].code) {
				*b = probe;
				*increment = sum + i + 1;
				return *increment <= limit ? 0 : -EINVAL;
			}
		}
		code = get(b, 11);
		if (code == 8) {
			sum += 33;
			if (sum >= limit)
				return -EINVAL;
		} else if (code != 15) {
			return -EINVAL;
		}
	}
	return -EINVAL;
}

static int slice_header(struct bits *b, u8 row, struct histb_mpeg1_au *au)
{
	u32 width = DIV_ROUND_UP(au->sequence.control.horizontal_size, 16);
	u32 height = DIV_ROUND_UP(au->sequence.control.vertical_size, 16);
	struct histb_mpeg1_slice *s;
	u32 increment;

	if (au->slice_count == HISTB_MPEG1_SLICES)
		return -E2BIG;
	if (row > height)
		return -EINVAL;
	s = &au->slice[au->slice_count];
	s->quantiser_scale = get(b, 5);
	/* MPEG1 has only extra_bit_slice here, NOT MPEG2 intra_slice_flag. */
	if (!s->quantiser_scale || extra_information(b) ||
	    address_increment(b, width, &increment))
		return -EINVAL;
	s->start_mb = (row - 1) * width + increment - 1;
	if ((!au->slice_count && s->start_mb) ||
	    (au->slice_count && s->start_mb <= s[-1].start_mb) ||
	    b->pos >= b->end)
		return -EINVAL;
	s->data_offset = (b->pos / 8) & ~15U;
	s->bit_offset = b->pos & 127;
	/* Keep the first macroblock_type bit; only MBA increment was consumed. */
	s->valid_bits = b->pos;
	au->slice_count++;
	return 0;
}

void histb_mpeg1_init(struct histb_mpeg1_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->older = HISTB_MPEG1_NO_SLOT;
	ctx->newer = HISTB_MPEG1_NO_SLOT;
	ctx->pending_slot = HISTB_MPEG1_NO_SLOT;
	ctx->need_intra = true;
}

int histb_mpeg1_parse(struct histb_mpeg1_ctx *ctx, const u8 *data, u32 size,
		     struct histb_mpeg1_au *au)
{
	u32 start, end, i;
	int ret;

	if (!data || !size || size > HISTB_MPEG1_MAX_BYTES || ctx->pending)
		return -EINVAL;
	memset(au, 0, sizeof(*au));
	au->size = size;
	au->sequence = ctx->sequence;
	start = next_code(data, 0, size);
	for (i = 0; i < start; i++)
		if (data[i])
			return -EINVAL;
	if (start == size)
		return -EINVAL;
	while (start < size) {
		u8 code = data[start + 3];
		struct bits b = { .data = data, .pos = (start + 4) * 8 };

		end = next_code(data, start + 4, size);
		b.end = end * 8;
		if (au->sequence_end)
			return -EINVAL;
		if (code >= 1 && code <= 0xaf) {
			if (!au->picture_present)
				return -EINVAL;
			if (!au->slice_count)
				au->slice_offset = start;
			ret = slice_header(&b, code, au);
			if (ret)
				return ret;
			au->slice_end = end;
		} else if (code == 0xb7) {
			if (au->picture_present && !au->slice_count)
				return -EINVAL;
			au->sequence_end = true;
			if (padding(&b))
				return -EINVAL;
		} else {
			if (au->slice_count)
				return -EINVAL;
			switch (code) {
			case 0xb3:
				if (au->sequence_present || au->picture_present || au->gop_present)
					return -EINVAL;
				ret = sequence_header(&b, &au->sequence);
				if (ret)
					return ret;
				au->sequence_present = true;
				au->reset_before = ctx->sequence_valid &&
					(ctx->sequence.control.horizontal_size !=
					 au->sequence.control.horizontal_size ||
					 ctx->sequence.control.vertical_size !=
					 au->sequence.control.vertical_size);
				break;
			case 0xb8:
				if (au->gop_present || au->picture_present)
					return -EINVAL;
				au->gop_time_code = get(&b, 25);
				au->closed_gop = get(&b, 1);
				au->broken_link = get(&b, 1);
				if (!(au->gop_time_code & BIT(12)) ||
				    ((au->gop_time_code >> 19) & 31) > 23 ||
				    ((au->gop_time_code >> 13) & 63) > 59 ||
				    ((au->gop_time_code >> 6) & 63) > 59 ||
				    padding(&b))
					return -EINVAL;
				au->gop_present = true;
				au->reset_before |= au->broken_link;
				break;
			case 0:
				if (au->picture_present ||
				    (!ctx->sequence_valid && !au->sequence_present))
					return -EINVAL;
				ret = picture_header(&b, au);
				if (ret)
					return ret;
				au->picture_present = true;
				break;
			case 0xb2:
				break; /* MPEG user_data is not picture entropy. */
			case 0xb5:
				return -EOPNOTSUPP;
			default:
				return -EINVAL;
			}
		}
		start = end;
	}
	if (au->picture_present && !au->slice_count)
		return -EINVAL;
	for (i = 0; i < au->slice_count; i++) {
		/* Native slice messages span the remaining packet, not one slice.
		 * D5/D6 delimit macroblocks; truncating at each start code stalls VDH.
		 */
		au->slice[i].valid_bits = au->slice_end * 8 - au->slice[i].valid_bits;
		if (!au->slice[i].valid_bits || au->slice[i].valid_bits > 0xffffff)
			return -E2BIG;
	}
	if (au->sequence_present) {
		ctx->sequence = au->sequence;
		ctx->sequence_valid = true;
	}
	return au->picture_present ? 1 : 0;
}

u16 histb_mpeg1_busy_slots(const struct histb_mpeg1_ctx *ctx)
{
	u16 mask = 0;
	unsigned int i;

	for (i = 0; i < HISTB_MPEG1_SLOTS; i++)
		if (ctx->frame[i].output || ctx->older == i || ctx->newer == i ||
		    (ctx->pending && ctx->pending_slot == i))
			mask |= BIT(i);
	return mask;
}

int histb_mpeg1_prepare(struct histb_mpeg1_ctx *ctx,
		       const struct histb_mpeg1_au *au, u8 slot, u64 timestamp,
		       struct histb_mpeg1_job *job)
{
	u8 type = au->picture.picture_coding_type;

	if (ctx->pending)
		return -EBUSY;
	if (!au->picture_present || !au->slice_count || slot >= HISTB_MPEG1_SLOTS)
		return -EINVAL;
	if (au->reset_before && histb_mpeg1_busy_slots(ctx))
		return -EPIPE;
	if (histb_mpeg1_busy_slots(ctx) & BIT(slot))
		return -ENOSPC;
	if (type != V4L2_MPEG2_PIC_CODING_TYPE_I &&
	    (ctx->need_intra || ctx->newer == HISTB_MPEG1_NO_SLOT))
		return -ENODATA;
	if (type == V4L2_MPEG2_PIC_CODING_TYPE_B &&
	    ctx->older == HISTB_MPEG1_NO_SLOT)
		return -ENODATA;
	memset(job, 0, sizeof(*job));
	job->au = au;
	job->slot = slot;
	job->timestamp = timestamp;
	job->picture = au->picture;
	job->forward = type == V4L2_MPEG2_PIC_CODING_TYPE_I ?
		HISTB_MPEG1_NO_SLOT : type == V4L2_MPEG2_PIC_CODING_TYPE_B ?
		ctx->older : ctx->newer;
	job->backward = type == V4L2_MPEG2_PIC_CODING_TYPE_B ?
		ctx->newer : HISTB_MPEG1_NO_SLOT;
	if (job->forward != HISTB_MPEG1_NO_SLOT)
		job->picture.forward_ref_ts = ctx->frame[job->forward].timestamp;
	if (job->backward != HISTB_MPEG1_NO_SLOT)
		job->picture.backward_ref_ts = ctx->frame[job->backward].timestamp;
	ctx->pending = true;
	ctx->pending_slot = slot;
	return 0;
}

static void ready(struct histb_mpeg1_ctx *ctx, u8 slot)
{
	struct histb_mpeg1_frame *f;

	if (slot == HISTB_MPEG1_NO_SLOT)
		return;
	f = &ctx->frame[slot];
	if (f->output && !f->ready) {
		f->ready = true;
		f->output_order = ctx->output_order++;
	}
}

void histb_mpeg1_complete(struct histb_mpeg1_ctx *ctx,
			 const struct histb_mpeg1_job *job, bool success)
{
	struct histb_mpeg1_frame *f;
	u8 type = job->picture.picture_coding_type;

	if (!ctx->pending || ctx->pending_slot != job->slot)
		return;
	ctx->pending = false;
	ctx->pending_slot = HISTB_MPEG1_NO_SLOT;
	if (!success) {
		if (type != V4L2_MPEG2_PIC_CODING_TYPE_B) {
			ready(ctx, ctx->newer);
			ctx->older = HISTB_MPEG1_NO_SLOT;
			ctx->newer = HISTB_MPEG1_NO_SLOT;
			ctx->need_intra = true;
		}
		return;
	}
	f = &ctx->frame[job->slot];
	memset(f, 0, sizeof(*f));
	f->timestamp = job->timestamp;
	f->temporal_reference = job->au->temporal_reference;
	f->type = type;
	f->output = true;
	if (type == V4L2_MPEG2_PIC_CODING_TYPE_B) {
		ready(ctx, job->slot);
	} else {
		ready(ctx, ctx->newer);
		ctx->older = ctx->newer;
		ctx->newer = job->slot;
		ctx->need_intra = false;
	}
}

int histb_mpeg1_next_output(struct histb_mpeg1_ctx *ctx, bool drain,
			   struct histb_mpeg1_output *output)
{
	u8 slot = HISTB_MPEG1_NO_SLOT;
	unsigned int i;

	if (drain) {
		if (ctx->pending)
			return -EBUSY;
		ready(ctx, ctx->newer);
	}
	for (i = 0; i < HISTB_MPEG1_SLOTS; i++)
		if (ctx->frame[i].output && ctx->frame[i].ready &&
		    (slot == HISTB_MPEG1_NO_SLOT || ctx->frame[i].output_order <
		     ctx->frame[slot].output_order))
			slot = i;
	if (slot == HISTB_MPEG1_NO_SLOT)
		return 0;
	output->slot = slot;
	output->timestamp = ctx->frame[slot].timestamp;
	output->temporal_reference = ctx->frame[slot].temporal_reference;
	output->type = ctx->frame[slot].type;
	ctx->frame[slot].output = false;
	return 1;
}

void histb_mpeg1_clear_dpb(struct histb_mpeg1_ctx *ctx)
{
	if (ctx->pending)
		return;
	memset(ctx->frame, 0, sizeof(ctx->frame));
	ctx->older = HISTB_MPEG1_NO_SLOT;
	ctx->newer = HISTB_MPEG1_NO_SLOT;
	ctx->need_intra = true;
	ctx->output_order = 0;
}

void histb_mpeg1_patch_picture(const struct histb_mpeg1_au *au, __le32 *pic)
{
	u32 d1 = le32_to_cpu(pic[1]);

	pic[0] = cpu_to_le32(le32_to_cpu(pic[0]) | BIT(25));
	d1 &= ~(BIT(27) | BIT(28));
	d1 |= (u32)au->full_pel_forward << 27;
	d1 |= (u32)au->full_pel_backward << 28;
	pic[1] = cpu_to_le32(d1);
}

int histb_mpeg1_build_messages(const struct histb_mpeg1_job *job,
	const struct histb_mpeg1_surface surface[HISTB_MPEG1_SLOTS],
	u32 stream_dma, u32 slice_dma, u32 pmv_top,
	__le32 pic[HISTB_MPEG1_PIC_WORDS], __le32 *slices)
{
	const struct histb_mpeg1_au *au = job->au;
	const struct v4l2_ctrl_mpeg2_picture *p = &job->picture;
	u32 w = DIV_ROUND_UP(au->sequence.control.horizontal_size, 16);
	u32 h = DIV_ROUND_UP(au->sequence.control.vertical_size, 16);
	u32 recon, pmv, forward, backward;
	u8 intra[64], non_intra[64];
	unsigned int i, j;

	if (job->slot >= HISTB_MPEG1_SLOTS || !au->slice_count || !w || !h ||
	    au->slice_count > HISTB_MPEG1_SLICES || au->size > HISTB_MPEG1_MAX_BYTES)
		return -EINVAL;
	recon = surface[job->slot].luma;
	pmv = surface[job->slot].pmv;
	forward = recon;
	backward = recon;
	if (job->forward != HISTB_MPEG1_NO_SLOT) {
		if (job->forward >= HISTB_MPEG1_SLOTS)
			return -EINVAL;
		forward = surface[job->forward].luma;
	}
	if (job->backward != HISTB_MPEG1_NO_SLOT) {
		if (job->backward >= HISTB_MPEG1_SLOTS)
			return -EINVAL;
		backward = surface[job->backward].luma;
	}
	if (!recon || !pmv || !forward || !backward || !pmv_top || !slice_dma ||
	    ((recon | pmv | forward | backward | stream_dma | slice_dma | pmv_top) & 15) ||
	    (u64)stream_dma + au->size > (1ULL << 32) ||
	    (u64)slice_dma + au->slice_count * 32 > (1ULL << 32))
		return -EINVAL;
	memset(pic, 0, HISTB_MPEG1_PIC_WORDS * sizeof(*pic));
	memset(slices, 0, au->slice_count * HISTB_MPEG1_SLICE_WORDS * sizeof(*slices));
	pic[0] = cpu_to_le32((w - 1) | ((h - 1) << 16));
	pic[1] = cpu_to_le32(1 | (V4L2_MPEG2_PIC_FRAME << 8) |
			      (u32)p->picture_coding_type << 24);
	pic[2] = cpu_to_le32(p->f_code[1][1] | (u32)p->f_code[1][0] << 8 |
		(u32)p->f_code[0][1] << 16 | (u32)p->f_code[0][0] << 24);
	pic[4] = cpu_to_le32(backward);
	pic[5] = cpu_to_le32(forward);
	pic[6] = cpu_to_le32(recon);
	pic[7] = cpu_to_le32(pmv);
	pic[48] = cpu_to_le32(pmv_top);
	pic[63] = cpu_to_le32(slice_dma);
	for (i = 0; i < 64; i++) {
		intra[zigzag[i]] = au->sequence.quantisation.intra_quantiser_matrix[i];
		non_intra[zigzag[i]] = au->sequence.quantisation.non_intra_quantiser_matrix[i];
	}
	for (i = 0; i < 8; i++) {
		u32 a = 0, b = 0, c = 0, d = 0;

		for (j = 0; j < 4; j++) {
			a |= (u32)intra[i + j * 16] << (j * 8);
			b |= (u32)intra[i + 8 + j * 16] << (j * 8);
			c |= (u32)non_intra[i + j * 16] << (j * 8);
			d |= (u32)non_intra[i + 8 + j * 16] << (j * 8);
		}
		pic[16 + i * 2] = cpu_to_le32(a);
		pic[17 + i * 2] = cpu_to_le32(b);
		pic[32 + i * 2] = cpu_to_le32(c);
		pic[33 + i * 2] = cpu_to_le32(d);
	}
	histb_mpeg1_patch_picture(au, pic);
	for (i = 0; i < au->slice_count; i++) {
		const struct histb_mpeg1_slice *s = &au->slice[i];
		__le32 *m = slices + i * HISTB_MPEG1_SLICE_WORDS;

		m[0] = cpu_to_le32(s->valid_bits | (u32)s->bit_offset << 24);
		m[1] = cpu_to_le32(s->data_offset);
		m[4] = cpu_to_le32(s->quantiser_scale);
		m[5] = cpu_to_le32(s->start_mb);
		m[6] = cpu_to_le32(i + 1 < au->slice_count ?
			au->slice[i + 1].start_mb - 1 : w * h - 1);
		if (i + 1 < au->slice_count)
			m[7] = cpu_to_le32(slice_dma + (i + 1) * 32);
		histb_v5r6_encode_mpeg2_slice(m);
	}
	histb_v5r6_encode_mpeg2_picture(pic);
	return 0;
}
