// SPDX-License-Identifier: GPL-2.0-only
/*
 * Evidence-backed CBR-like rate-control calculations for the
 * HiSilicon Hi3798CV200 VEDU H.264 encoder.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/overflow.h>
#include <linux/string.h>

#include "histb-venc-rc.h"

#define HISTB_VENC_RC_MAX_QP		51
#define HISTB_VENC_RC_MAX_FPS		60
#define HISTB_VENC_RC_MAX_GOP		1000
#define HISTB_VENC_RC_I_WEIGHT		5

static void histb_venc_rc_push_u32(u32 *history, u32 value)
{
	unsigned int i;

	for (i = HISTB_VENC_RC_HISTORY_LENGTH - 1; i; i--)
		history[i] = history[i - 1];
	history[0] = value;
}

static void histb_venc_rc_push_u8(u8 *history, u8 value)
{
	unsigned int i;

	for (i = HISTB_VENC_RC_HISTORY_LENGTH - 1; i; i--)
		history[i] = history[i - 1];
	history[0] = value;
}

static u32 histb_venc_rc_average_u32(const u32 *history)
{
	u64 sum = 0;
	u32 samples = 0;
	unsigned int i;

	for (i = 0; i < HISTB_VENC_RC_HISTORY_LENGTH; i++) {
		if (!history[i])
			continue;
		sum += history[i];
		samples++;
	}

	return samples ? div64_u64(sum, samples) : 0;
}

static u8 histb_venc_rc_average_u8(const u8 *history)
{
	u32 sum = 0;
	u32 samples = 0;
	unsigned int i;

	for (i = 0; i < HISTB_VENC_RC_HISTORY_LENGTH; i++) {
		if (!history[i])
			continue;
		sum += history[i];
		samples++;
	}

	return samples ? sum / samples : 0;
}

static void histb_venc_rc_fifo_write(struct histb_venc_rc *rc, u32 bits)
{
	if (rc->bit_fifo_sum >= rc->bit_fifo[rc->bit_fifo_head])
		rc->bit_fifo_sum -= rc->bit_fifo[rc->bit_fifo_head];
	else
		rc->bit_fifo_sum = 0;

	rc->bit_fifo_sum += bits;
	rc->bit_fifo[rc->bit_fifo_head] = bits;
	rc->bit_fifo_head++;
	if (rc->bit_fifo_head >= rc->bit_fifo_len)
		rc->bit_fifo_head = 0;
}

static u8 histb_venc_rc_initial_qp(u32 target_bits, u32 width, u32 height)
{
	u64 samples = (u64)width * height * 3;
	u64 bits_per_point;

	/* Vendor formula: 100 * bits * 2 / (width * height * 3). */
	bits_per_point = div64_u64((u64)target_bits * 200, samples);
	if (bits_per_point > 170)
		return 7;
	if (bits_per_point > 120)
		return 15;
	if (bits_per_point > 80)
		return 20;
	if (bits_per_point > 40)
		return 25;
	if (bits_per_point > 15)
		return 30;
	if (bits_per_point > 5)
		return 38;
	if (bits_per_point > 2)
		return 40;

	return 43;
}

static int histb_venc_rc_u32_target(u64 target, u32 *result)
{
	if (!target || target > (u64)~0U)
		return -ERANGE;

	*result = target;
	return 0;
}

static u8 histb_venc_rc_clamp_qp(const struct histb_venc_rc *rc, int qp)
{
	return clamp_t(int, qp, rc->cfg.min_qp, rc->cfg.max_qp);
}

static u32 histb_venc_rc_intra_weight(const struct histb_venc_rc *rc)
{
	u32 intra_ratio;
	s32 times;

	if (!rc->have_i)
		return HISTB_VENC_RC_I_WEIGHT;

	intra_ratio = rc->average_intra_mbs * 100 / rc->mb_count;
	times = 9 - (12 * (s32)intra_ratio + 50) / 100;
	if (rc->previous_mean_qp > 40)
		times -= ((s32)rc->average_mean_qp - 40) / 2;
	if (rc->still_move_count > 12)
		times -= 2;
	if (rc->still_frame_count >= 30)
		times += rc->still_frame_count2 >= 30 ? 6 : 4;

	return clamp_t(s32, times, 3, 5);
}

static u64 histb_venc_rc_intra_target(const struct histb_venc_rc *rc)
{
	u32 weight = histb_venc_rc_intra_weight(rc);
	u64 target;

	target = div64_u64(rc->gop_bits, weight + rc->cfg.gop_size - 1);
	target *= weight;
	if (rc->have_i)
		target = min(target, (u64)rc->last_i_bits * 3 / 2);

	return target;
}

static int histb_venc_rc_intra_qp(const struct histb_venc_rc *rc,
				  u32 target_bits)
{
	s64 qp;

	if (!rc->have_i)
		return histb_venc_rc_initial_qp(target_bits, rc->cfg.width,
						 rc->cfg.height);

	if (target_bits > rc->last_i_bits) {
		qp = rc->last_i_qp -
		     div64_s64(6LL * (target_bits - rc->last_i_bits),
			       rc->last_i_bits);
	} else {
		qp = rc->last_i_qp + 1 +
		     div64_s64(6LL * (rc->last_i_bits - target_bits),
			       target_bits);
	}
	qp = clamp_t(s64, qp, rc->last_i_qp - 6, rc->last_i_qp + 8);
	qp = max_t(s64, qp, (s64)rc->previous_qp - 6);

	return qp;
}

static u64 histb_venc_rc_inter_target(const struct histb_venc_rc *rc)
{
	u32 frames_left = rc->cfg.gop_size - rc->frame_in_gop;
	u64 target;

	if (rc->p_target_bits) {
		target = rc->p_target_bits;
	} else {
		if (rc->gop_bits_left > 0)
			target = div64_u64(rc->gop_bits_left, frames_left);
		else
			target = 0;

		/* The original remaining-GOP AvePBits floor is 0.5x. */
		target = max(target, (u64)rc->average_frame_bits / 2);
	}

	/* The first P picture in a GOP bypasses the FIFO adjustment. */
	if (rc->frame_in_gop > 1) {
		if (rc->bit_fifo_sum * 100 > (u64)rc->cfg.bitrate * 110)
			target = target * 9 / 10;
		else if (rc->bit_fifo_sum * 100 <
			 (u64)rc->cfg.bitrate * 90)
			target = target * 11 / 10;
	}

	return target;
}

static int histb_venc_rc_inter_qp(const struct histb_venc_rc *rc)
{
	s64 qp;
	u64 ratio;

	if (!rc->have_p)
		return rc->previous_qp + 3;

	/* The first P picture of a later GOP reuses the previous P history. */
	if (rc->frame_in_gop == 1)
		return max(rc->last_p_qp, rc->previous_qp);

	if ((u64)rc->previous_picture_bits * 100 <
	    (u64)rc->previous_target_bits * 105) {
		if ((u64)rc->previous_picture_bits * 100 >
		    (u64)rc->previous_target_bits * 90) {
			qp = rc->previous_qp;
		} else {
			ratio = div64_u64((u64)(rc->previous_target_bits -
						 rc->previous_picture_bits) * 100,
					  rc->previous_target_bits);
			if (ratio >= 40)
				qp = rc->previous_qp - 3;
			else if (ratio > 20)
				qp = rc->previous_qp - 2;
			else
				qp = rc->previous_qp - 1;
			qp = min_t(s64, rc->previous_mean_qp, qp);
		}
	} else {
		ratio = div64_u64((u64)(rc->previous_picture_bits -
					 rc->previous_target_bits) * 100,
				  rc->previous_target_bits);
		ratio = min_t(u64, ratio, 200);
		qp = rc->previous_qp + ratio * 5 / 100 + 1;
		qp = min_t(s64, qp, HISTB_VENC_RC_MAX_QP);
	}

	if ((int)rc->previous_mean_qp - rc->previous_qp > 5)
		qp++;

	return clamp_t(s64, qp, (s64)rc->previous_qp - 3,
		       (s64)rc->previous_qp + 3);
}

static void
histb_venc_rc_update_activity(struct histb_venc_rc *rc,
			      const struct histb_venc_rc_decision *decision,
			      const struct histb_venc_rc_feedback *feedback,
			      u32 intra_mbs)
{
	u32 activity_mbs;
	u32 decrement;

	rc->last_intra_mbs = intra_mbs;
	rc->last_timer = feedback->timer;
	rc->last_idle_timer = feedback->idle_timer;

	histb_venc_rc_push_u8(rc->mean_qp_history, feedback->mean_qp);
	rc->average_mean_qp =
		histb_venc_rc_average_u8(rc->mean_qp_history);

	/* The first P after an I uses the preceding P-picture average. */
	activity_mbs = decision->intra ? rc->average_intra_mbs : intra_mbs;
	histb_venc_rc_push_u32(rc->intra_mb_history, activity_mbs);
	rc->average_intra_mbs =
		histb_venc_rc_average_u32(rc->intra_mb_history);
	rc->frame_intra_ratio = activity_mbs * 100 / rc->mb_count;
	rc->intra_ratio_sum += rc->frame_intra_ratio;

	if (rc->frame_intra_ratio >= 8) {
		rc->still_move_count =
			min_t(u8, rc->still_move_count + 1, 20);
	} else {
		decrement = 1;
		if (rc->frame_intra_ratio <= 1)
			decrement++;
		if (!rc->frame_intra_ratio)
			decrement++;
		rc->still_move_count =
			rc->still_move_count > decrement ?
			rc->still_move_count - decrement : 0;
	}

	if (rc->average_mean_qp < 35) {
		rc->still_frame_count =
			min_t(u8, rc->still_frame_count + 1, 40);
		if (rc->average_mean_qp < 30) {
			rc->still_frame_count2 =
				min_t(u8, rc->still_frame_count2 + 1, 40);
		} else {
			rc->still_frame_count2 = rc->still_frame_count2 > 5 ?
				rc->still_frame_count2 - 5 : 0;
		}
	} else {
		rc->still_frame_count = rc->still_frame_count > 5 ?
			rc->still_frame_count - 5 : 0;
	}

	if (rc->intra_ratio_frames == 1000) {
		rc->intra_ratio_sum = 0;
		rc->intra_ratio_frames = 0;
	}
	rc->intra_ratio_frames++;
}

int histb_venc_rc_init(struct histb_venc_rc *rc,
		       const struct histb_venc_rc_config *config)
{
	u64 average_frame_bits;
	u64 fps_limit;
	u32 fps;
	unsigned int i;

	if (!rc || !config || !config->bitrate || !config->fps_num ||
	    !config->fps_den || !config->gop_size || !config->width ||
	    !config->height || config->min_qp > config->max_qp ||
	    config->max_qp > HISTB_VENC_RC_MAX_QP ||
	    config->gop_size > HISTB_VENC_RC_MAX_GOP)
		return -EINVAL;

	fps_limit = (u64)HISTB_VENC_RC_MAX_FPS * config->fps_den;
	if (config->fps_num > fps_limit)
		return -ERANGE;
	/* The original RC API and its FIFO use integral input/output rates. */
	if (config->fps_num % config->fps_den)
		return -EOPNOTSUPP;
	fps = config->fps_num / config->fps_den;
	if (!fps || fps >= HISTB_VENC_RC_FIFO_CAPACITY)
		return -ERANGE;

	average_frame_bits = div64_u64((u64)config->bitrate * config->fps_den,
				       config->fps_num);
	if (!average_frame_bits || average_frame_bits > (u64)~0U)
		return -ERANGE;

	memset(rc, 0, sizeof(*rc));
	rc->cfg = *config;
	rc->average_frame_bits = average_frame_bits;
	rc->mb_count = DIV_ROUND_UP(config->width, 16) *
		       DIV_ROUND_UP(config->height, 16);
	rc->bit_fifo_len = fps;
	rc->bit_fifo_mean = config->bitrate / fps;
	for (i = 0; i < rc->bit_fifo_len; i++) {
		rc->bit_fifo[i] = rc->average_frame_bits;
		rc->bit_fifo_sum += rc->average_frame_bits;
	}
	/* RcStart increments both scores before the first picture. */
	rc->still_frame_count = 1;
	rc->still_frame_count2 = 1;
	rc->intra_ratio_frames = 1;
	if (check_mul_overflow(average_frame_bits, (u64)config->gop_size,
			       &rc->gop_bits))
		return -ERANGE;
	rc->gop_bits_left = rc->gop_bits;

	return 0;
}

int histb_venc_rc_prepare(struct histb_venc_rc *rc, bool force_intra,
			  struct histb_venc_rc_decision *decision)
{
	struct histb_venc_rc_decision next = {};
	u64 target;
	int qp;
	int ret;

	if (!rc || !decision)
		return -EINVAL;
	if (rc->pending)
		return -EBUSY;

	next.intra = !rc->committed_frames || !rc->frame_in_gop || force_intra;
	if (next.intra) {
		target = histb_venc_rc_intra_target(rc);
		ret = histb_venc_rc_u32_target(target, &next.target_bits);
		if (ret)
			return ret;
		qp = histb_venc_rc_intra_qp(rc, next.target_bits);
	} else {
		target = histb_venc_rc_inter_target(rc);
		ret = histb_venc_rc_u32_target(target, &next.target_bits);
		if (ret)
			return ret;
		qp = histb_venc_rc_inter_qp(rc);
		if (rc->cfg.gop_size > 10 &&
		    rc->frame_in_gop > rc->cfg.gop_size - 10) {
			qp = rc->previous_qp;
			next.target_bits = rc->previous_target_bits;
			if (!(rc->frame_in_gop % 2)) {
				qp++;
				next.target_bits =
					(u64)next.target_bits * 95 / 100;
			}
		}
	}

	if (rc->bit_fifo_sum * 100 > (u64)rc->cfg.bitrate * 105)
		qp++;
	else if (rc->bit_fifo_sum * 100 < (u64)rc->cfg.bitrate * 95)
		qp--;

	next.start_qp = histb_venc_rc_clamp_qp(rc, qp);
	next.min_qp = rc->cfg.min_qp;
	next.max_qp = rc->cfg.max_qp;
	next.token = ++rc->next_token;
	if (!next.token)
		next.token = ++rc->next_token;

	rc->pending_decision = next;
	rc->pending = true;
	*decision = next;

	return 0;
}

void histb_venc_rc_feedback_from_raw(struct histb_venc_rc_feedback *feedback,
				     u32 picture_bits, u32 mean_qp,
				     u32 timer, u32 idle_timer,
				     u32 picinfo1, u32 picinfo6,
				     u32 picinfo7, u32 picinfo8)
{
	if (!feedback)
		return;

	feedback->picture_bits = picture_bits;
	feedback->mean_qp = mean_qp & HISTB_VENC_RC_MEAN_QP_MASK;
	feedback->timer = timer;
	feedback->idle_timer = idle_timer;
	feedback->ipcm_mbs = picinfo1 & HISTB_VENC_RC_MB_COUNT_MASK;
	feedback->intra16_mbs = picinfo6 & HISTB_VENC_RC_MB_COUNT_MASK;
	feedback->intra8_mbs = picinfo7 & HISTB_VENC_RC_MB_COUNT_MASK;
	feedback->intra4_mbs = picinfo8 & HISTB_VENC_RC_MB_COUNT_MASK;
}

int histb_venc_rc_commit(struct histb_venc_rc *rc, u32 token,
			 const struct histb_venc_rc_feedback *feedback)
{
	const struct histb_venc_rc_decision *decision;
	u32 intra_mbs;

	if (!rc || !feedback || !rc->pending ||
	    rc->pending_decision.token != token)
		return -EINVAL;
	if (!feedback->picture_bits ||
	    feedback->mean_qp > HISTB_VENC_RC_MAX_QP ||
	    feedback->ipcm_mbs > HISTB_VENC_RC_MB_COUNT_MASK ||
	    feedback->intra16_mbs > HISTB_VENC_RC_MB_COUNT_MASK ||
	    feedback->intra8_mbs > HISTB_VENC_RC_MB_COUNT_MASK ||
	    feedback->intra4_mbs > HISTB_VENC_RC_MB_COUNT_MASK)
		return -ERANGE;

	intra_mbs = feedback->ipcm_mbs + feedback->intra16_mbs +
		    feedback->intra8_mbs + feedback->intra4_mbs;
	if (intra_mbs > rc->mb_count)
		return -ERANGE;

	decision = &rc->pending_decision;
	if (decision->intra) {
		rc->gop_bits_left = rc->gop_bits;
		rc->frame_in_gop = 0;
		rc->p_target_bits = 0;
		rc->last_i_bits = feedback->picture_bits;
		rc->last_i_qp = decision->start_qp;
		rc->have_i = true;
	} else {
		if (!rc->p_target_bits)
			rc->p_target_bits = decision->target_bits;
		rc->last_p_qp = decision->start_qp;
		rc->have_p = true;
	}

	rc->gop_bits_left -= feedback->picture_bits;
	rc->frame_in_gop++;
	if (rc->frame_in_gop >= rc->cfg.gop_size)
		rc->frame_in_gop = 0;
	rc->committed_frames++;
	rc->previous_target_bits = decision->target_bits;
	rc->previous_picture_bits = feedback->picture_bits;
	rc->previous_qp = decision->start_qp;
	rc->previous_mean_qp = feedback->mean_qp;
	histb_venc_rc_fifo_write(rc, feedback->picture_bits);
	histb_venc_rc_update_activity(rc, decision, feedback, intra_mbs);
	rc->pending = false;

	return 0;
}

int histb_venc_rc_abort(struct histb_venc_rc *rc, u32 token)
{
	if (!rc || !rc->pending || rc->pending_decision.token != token)
		return -EINVAL;

	rc->pending = false;
	return 0;
}

int histb_venc_rc_drop(struct histb_venc_rc *rc, u32 token)
{
	if (!rc || !rc->pending || rc->pending_decision.token != token)
		return -EINVAL;

	/* BufferFull/PTBITS failures consume a zero-bit FIFO slot after RcStart. */
	if (rc->committed_frames)
		histb_venc_rc_fifo_write(rc, 0);
	rc->pending = false;

	return 0;
}
