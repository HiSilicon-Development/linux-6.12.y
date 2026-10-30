/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HISTB_VDEC_MPEG1_H
#define HISTB_VDEC_MPEG1_H

#include <linux/types.h>
#include <linux/v4l2-controls.h>

#define HISTB_MPEG1_SLICES 512
#define HISTB_MPEG1_SLOTS 8
#define HISTB_MPEG1_NO_SLOT 255
#define HISTB_MPEG1_PIC_WORDS 64
#define HISTB_MPEG1_SLICE_WORDS 8
#define HISTB_MPEG1_MAX_BYTES 0x1ffffcU

struct histb_mpeg1_sequence {
	struct v4l2_ctrl_mpeg2_sequence control;
	struct v4l2_ctrl_mpeg2_quantisation quantisation;
	u32 bit_rate;
	u16 vbv_buffer_size;
	u8 aspect_ratio, frame_rate_code, constrained;
};

struct histb_mpeg1_slice {
	u32 data_offset, valid_bits, start_mb;
	u8 bit_offset, quantiser_scale;
};

struct histb_mpeg1_au {
	struct histb_mpeg1_sequence sequence;
	struct v4l2_ctrl_mpeg2_picture picture;
	u32 size, slice_offset, slice_end, gop_time_code;
	u16 temporal_reference, vbv_delay, slice_count;
	u8 full_pel_forward, full_pel_backward;
	bool sequence_present, picture_present, sequence_end, gop_present;
	bool closed_gop, broken_link, reset_before;
	struct histb_mpeg1_slice slice[HISTB_MPEG1_SLICES];
};

struct histb_mpeg1_frame {
	u64 timestamp, output_order;
	u16 temporal_reference;
	u8 type;
	bool output, ready;
};

struct histb_mpeg1_ctx {
	struct histb_mpeg1_sequence sequence;
	struct histb_mpeg1_frame frame[HISTB_MPEG1_SLOTS];
	u64 output_order;
	u8 older, newer, pending_slot;
	bool sequence_valid, pending, need_intra;
};

struct histb_mpeg1_job {
	const struct histb_mpeg1_au *au;
	struct v4l2_ctrl_mpeg2_picture picture;
	u64 timestamp;
	u8 slot, forward, backward;
};

struct histb_mpeg1_output {
	u64 timestamp;
	u16 temporal_reference;
	u8 slot, type;
};

struct histb_mpeg1_surface {
	u32 luma, pmv;
};

/* No allocation. One OUTPUT buffer contains one complete elementary-stream AU,
 * with optional sequence/GOP/user headers. Header-only buffers return 0,
 * pictures return 1, errors are -errno. Parsing is transactional. MPEG-2
 * extension syntax and MPEG-1 D pictures return -EOPNOTSUPP, never misdecode.
 * Controls use zigzag matrix order and f_code[direction][horizontal/vertical].
 * Full-pel flags MUST NOT be folded into f_code or simulated in software.
 *
 * On reset_before/source change: drain next_output(true), clear_dpb(), negotiate
 * CAPTURE geometry, then prepare the SAME AU. Do not parse a queued AU twice.
 * prepare reserves a free slot; complete must run exactly once even on failed
 * submission. Hold every busy slot AND all caller-owned output surfaces.
 * next_output transfers only the output hold; prediction anchors stay busy.
 * Drain after the final picture on sequence_end/decoder STOP, then clear_dpb.
 * clear_dpb requires !pending and prior draining (or explicit stream abort).
 * Raw AU storage and job must survive prepare/build/complete.
 */
void histb_mpeg1_init(struct histb_mpeg1_ctx *ctx);
int histb_mpeg1_parse(struct histb_mpeg1_ctx *ctx, const u8 *data, u32 size,
		     struct histb_mpeg1_au *au);
u16 histb_mpeg1_busy_slots(const struct histb_mpeg1_ctx *ctx);
int histb_mpeg1_prepare(struct histb_mpeg1_ctx *ctx,
		       const struct histb_mpeg1_au *au, u8 slot, u64 timestamp,
		       struct histb_mpeg1_job *job);
void histb_mpeg1_complete(struct histb_mpeg1_ctx *ctx,
			 const struct histb_mpeg1_job *job, bool success);
int histb_mpeg1_next_output(struct histb_mpeg1_ctx *ctx, bool drain,
			   struct histb_mpeg1_output *output);
void histb_mpeg1_clear_dpb(struct histb_mpeg1_ctx *ctx);

/* Native MV310 V5R6 only. All physical addresses are 32-bit and 16-byte
 * aligned. stream_dma points at the complete AU, not slice_offset. Arrays
 * contain PIC_WORDS and slice_count * SLICE_WORDS LITTLE-ENDIAN words.
 * slice_dma is the separate slice-array physical address; pmv_top is the
 * existing MPEG2 scratch allocation. The builder encodes addresses exactly
 * once with histb_v5r6_encode_mpeg2_*; do not encode its output again.
 * Use native MPEG2 launch: cfg0=0x41400000|(total_mbs-1), cfg1=0x34003,
 * AVS=0, EMAR enabled, uncompressed 8-bit MPEG2 reconstruction geometry.
 */
int histb_mpeg1_build_messages(const struct histb_mpeg1_job *job,
	const struct histb_mpeg1_surface surface[HISTB_MPEG1_SLOTS],
	u32 stream_dma, u32 slice_dma, u32 pmv_top,
	__le32 pic[HISTB_MPEG1_PIC_WORDS], __le32 *slices);

/* Alternative to the standalone builder: existing prepare_mpeg2_messages may
 * consume sequence.control, job.picture, sequence.quantisation and adapted
 * slice descriptors (intra=0), but it MUST receive full-AU size/offsets and
 * patch this native syntax after fresh packing. This does not encode addresses.
 */
void histb_mpeg1_patch_picture(const struct histb_mpeg1_au *au, __le32 *pic);

#endif
