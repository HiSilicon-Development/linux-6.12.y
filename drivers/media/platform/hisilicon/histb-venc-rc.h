/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HISTB_VENC_RC_H
#define HISTB_VENC_RC_H

#include <linux/types.h>

/* Hi3798CV200 VEDU feedback registers consumed by the original RC path. */
#define HISTB_VENC_RC_REG_TIMER		0x00a8
#define HISTB_VENC_RC_REG_IDLE_TIMER	0x00ac
#define HISTB_VENC_RC_REG_TARGET_BITS	0x0364
#define HISTB_VENC_RC_REG_PICTURE_BITS	0x0374
#define HISTB_VENC_RC_REG_MEAN_QP	0x0388
#define HISTB_VENC_RC_REG_PICINFO1	0x0a24
#define HISTB_VENC_RC_REG_PICINFO6	0x0a38
#define HISTB_VENC_RC_REG_PICINFO7	0x0a3c
#define HISTB_VENC_RC_REG_PICINFO8	0x0a40

#define HISTB_VENC_RC_MEAN_QP_MASK	0x3f
#define HISTB_VENC_RC_MB_COUNT_MASK	0x7ffff
#define HISTB_VENC_RC_FIFO_CAPACITY	65
#define HISTB_VENC_RC_HISTORY_LENGTH	6

/**
 * struct histb_venc_rc_config - evidence-backed CV200 CBR-like policy
 * @bitrate: target stream rate in bits per second
 * @fps_num: output frames-per-second numerator
 * @fps_den: output frames-per-second denominator
 * @gop_size: distance between scheduled intra pictures
 * @width: coded picture width
 * @height: coded picture height
 * @min_qp: lowest hardware QP permitted by the policy
 * @max_qp: highest hardware QP permitted by the policy
 *
 * The frame rate is @fps_num / @fps_den.  The evidence-backed FIFO path
 * requires an integral rate and models equal input/output rates, as every
 * V4L2 mem2mem input is an output job.  This interface deliberately has no VBR
 * selector: the matching vendor engine forced its VBR flag off.
 */
struct histb_venc_rc_config {
	u32 bitrate;
	u32 fps_num;
	u32 fps_den;
	u32 gop_size;
	u32 width;
	u32 height;
	u8 min_qp;
	u8 max_qp;
};

/**
 * struct histb_venc_rc_decision - parameters prepared for one hardware job
 * @token: transaction identifier required by commit or abort
 * @target_bits: per-picture VEDU TARGETBITS value
 * @start_qp: per-picture VEDU start QP
 * @min_qp: per-picture VEDU minimum QP
 * @max_qp: per-picture VEDU maximum QP
 * @intra: this job starts a new GOP
 */
struct histb_venc_rc_decision {
	u32 token;
	u32 target_bits;
	u8 start_qp;
	u8 min_qp;
	u8 max_qp;
	bool intra;
};

/**
 * struct histb_venc_rc_feedback - successful CV200 picture feedback
 * @picture_bits: complete VEDU_PICBITS value
 * @timer: complete VEDU_TIMER value
 * @idle_timer: complete VEDU_IDLE_TIMER value
 * @ipcm_mbs: PICINFO1 IPCM macroblock count
 * @intra16_mbs: PICINFO6 Intra16 macroblock count
 * @intra8_mbs: PICINFO7 Intra8 macroblock count
 * @intra4_mbs: PICINFO8 Intra4 macroblock count
 * @mean_qp: VEDU_MEANQP bits 5:0
 */
struct histb_venc_rc_feedback {
	u32 picture_bits;
	u32 timer;
	u32 idle_timer;
	u32 ipcm_mbs;
	u32 intra16_mbs;
	u32 intra8_mbs;
	u32 intra4_mbs;
	u8 mean_qp;
};

/**
 * struct histb_venc_rc - committed and pending rate-control state
 * @cfg: active immutable configuration
 * @average_frame_bits: target bitrate divided by output frame rate
 * @gop_bits: nominal bit budget for one GOP
 * @gop_bits_left: nominal GOP budget less committed picture bits
 * @p_target_bits: constant P-picture budget selected by the first P picture
 * @frame_in_gop: index of the next picture in the committed GOP
 * @committed_frames: successfully encoded picture count
 * @previous_target_bits: target used for the last successful picture
 * @previous_picture_bits: actual bits from the last successful picture
 * @previous_qp: start QP used for the last successful picture
 * @previous_mean_qp: hardware mean QP from the last successful picture
 * @last_i_bits: actual bits from the last successful intra picture
 * @last_i_qp: start QP from the last successful intra picture
 * @last_p_qp: start QP from the last successful inter picture
 * @mb_count: coded macroblocks used to validate PICINFO feedback
 * @bit_fifo: original one-second encoded-bit history
 * @bit_fifo_sum: sum of the active entries in @bit_fifo
 * @bit_fifo_mean: original per-input-frame FIFO mean
 * @bit_fifo_len: active FIFO length; equal to the integral output frame rate
 * @bit_fifo_head: next FIFO slot replaced by successful or dropped output
 * @intra_mb_history: previous six P-picture NumIMB samples
 * @mean_qp_history: previous six successful MeanQP samples
 * @average_intra_mbs: non-zero average of @intra_mb_history
 * @average_mean_qp: non-zero average of @mean_qp_history
 * @last_intra_mbs: most recent raw PICINFO1+6+7+8 result
 * @frame_intra_ratio: last accepted P-picture NumIMB percentage
 * @intra_ratio_sum: accumulated P-picture NumIMB percentages
 * @intra_ratio_frames: sequence counter used with @intra_ratio_sum
 * @still_move_count: original saturating 0..20 movement score
 * @still_frame_count: original saturating 0..40 low-QP score
 * @still_frame_count2: original saturating 0..40 very-low-QP score
 * @last_timer: most recent VEDU_TIMER result
 * @last_idle_timer: most recent VEDU_IDLE_TIMER result
 * @have_i: an intra-picture history sample is available
 * @have_p: an inter-picture history sample is available
 * @pending: one prepared hardware job has not been committed or aborted
 * @next_token: monotonically increasing transaction identifier
 * @pending_decision: immutable parameters for the outstanding job
 */
struct histb_venc_rc {
	struct histb_venc_rc_config cfg;
	u32 average_frame_bits;
	u64 gop_bits;
	s64 gop_bits_left;
	u32 p_target_bits;
	u32 frame_in_gop;
	u64 committed_frames;
	u32 previous_target_bits;
	u32 previous_picture_bits;
	u8 previous_qp;
	u8 previous_mean_qp;
	u32 last_i_bits;
	u8 last_i_qp;
	u8 last_p_qp;
	u32 mb_count;
	u32 bit_fifo[HISTB_VENC_RC_FIFO_CAPACITY];
	u64 bit_fifo_sum;
	u32 bit_fifo_mean;
	u8 bit_fifo_len;
	u8 bit_fifo_head;
	u32 intra_mb_history[HISTB_VENC_RC_HISTORY_LENGTH];
	u8 mean_qp_history[HISTB_VENC_RC_HISTORY_LENGTH];
	u32 average_intra_mbs;
	u8 average_mean_qp;
	u32 last_intra_mbs;
	u8 frame_intra_ratio;
	u32 intra_ratio_sum;
	u16 intra_ratio_frames;
	u8 still_move_count;
	u8 still_frame_count;
	u8 still_frame_count2;
	u32 last_timer;
	u32 last_idle_timer;
	bool have_i;
	bool have_p;
	bool pending;
	u32 next_token;
	struct histb_venc_rc_decision pending_decision;
};

int histb_venc_rc_init(struct histb_venc_rc *rc,
		       const struct histb_venc_rc_config *config);
int histb_venc_rc_prepare(struct histb_venc_rc *rc, bool force_intra,
			  struct histb_venc_rc_decision *decision);
void histb_venc_rc_feedback_from_raw(struct histb_venc_rc_feedback *feedback,
				     u32 picture_bits, u32 mean_qp,
				     u32 timer, u32 idle_timer,
				     u32 picinfo1, u32 picinfo6,
				     u32 picinfo7, u32 picinfo8);
int histb_venc_rc_commit(struct histb_venc_rc *rc, u32 token,
			 const struct histb_venc_rc_feedback *feedback);
int histb_venc_rc_abort(struct histb_venc_rc *rc, u32 token);
int histb_venc_rc_drop(struct histb_venc_rc *rc, u32 token);

#endif
