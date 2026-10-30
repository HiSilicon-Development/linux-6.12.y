/* SPDX-License-Identifier: GPL-2.0-only */
/* Hi3798MV310 VFMW v5.1 AVS2 access-unit and VDH message contract. */
#ifndef HISTB_VDEC_AVS2_H
#define HISTB_VDEC_AVS2_H

#include <linux/types.h>

#define HISTB_AVS2_SLOTS 16
#define HISTB_AVS2_REFS 7
#define HISTB_AVS2_RCS 32
#define HISTB_AVS2_SLICES 199
#define HISTB_AVS2_PIC_WORDS 264
#define HISTB_AVS2_SLICE_WORDS 64

/* Ordering is the actual v5.1 syntax/HAL ordering, not AVS/AVSP. */
enum histb_avs2_picture_type {
	HISTB_AVS2_I, HISTB_AVS2_P, HISTB_AVS2_B, HISTB_AVS2_F,
	HISTB_AVS2_S, HISTB_AVS2_G, HISTB_AVS2_GB,
};

enum histb_avs2_tool {
	HISTB_AVS2_SCENE, HISTB_AVS2_MH_SKIP, HISTB_AVS2_DH_PRED,
	HISTB_AVS2_W_SKIP, HISTB_AVS2_AMP, HISTB_AVS2_NSQT,
	HISTB_AVS2_NSIP, HISTB_AVS2_SECONDARY, HISTB_AVS2_SAO,
	HISTB_AVS2_ALF, HISTB_AVS2_PMVR,
};

struct histb_avs2_rcs {
	u8 referred, count, delta[HISTB_AVS2_REFS];
	u8 remove_count, remove[HISTB_AVS2_REFS];
};

struct histb_avs2_sequence {
	u16 width, height, display_width, display_height;
	u16 tools;
	u8 profile, level, progressive, field_coded, chroma_format;
	u8 sample_depth, bit_depth, aspect_ratio, frame_rate_code;
	u8 low_delay, temporal_id_enable, lcu_log2, weight_quant;
	u8 load_wq, wq4[16], wq8[64], rcs_count, reorder_delay;
	u8 cross_slice_filter, max_dpb;
	struct histb_avs2_rcs rcs[HISTB_AVS2_RCS];
};

struct histb_avs2_picture {
	struct histb_avs2_rcs rcs;
	u8 type, intra, scene_reference, random_access, doi, temporal_id;
	u8 output_delay, progressive, structure, top_first, repeat_first, top_field;
	u8 fixed_qp, qp, skip, loop_disable, loop_parameters, chroma_disable;
	s8 alpha, beta, chroma_cb, chroma_cr;
	u8 weight_quant, wq_shift, wq4[16], wq8[64], alf_enable[3];
	s16 alf_luma[16][9], alf_chroma[2][9];
};

struct histb_avs2_slice {
	u32 offset, size, first_lcu;
	u16 x, y;
};

struct histb_avs2_au {
	struct histb_avs2_sequence sequence;
	struct histb_avs2_picture picture;
	u32 size;
	u16 slice_count;
	bool sequence_present, picture_present, sequence_end, video_edit;
	bool reset_before;
	struct histb_avs2_slice slice[HISTB_AVS2_SLICES];
};

struct histb_avs2_frame {
	s32 doi, poi, output_at;
	s16 distance[HISTB_AVS2_REFS], reciprocal[HISTB_AVS2_REFS];
	u64 timestamp;
	u8 type, top_field, structure, top_first, progressive;
	u8 ref_top_field[HISTB_AVS2_REFS];
	bool reference, scene, output;
};

struct histb_avs2_ctx {
	struct histb_avs2_sequence sequence;
	struct histb_avs2_frame frame[HISTB_AVS2_SLOTS];
	s32 doi_epoch, last_doi, current_doi;
	bool sequence_valid, pending, reset_required;
};

struct histb_avs2_job {
	const struct histb_avs2_au *au;
	struct histb_avs2_frame frame;
	s16 distance[8], col_distance[8], reciprocal[8], col_reciprocal[8];
	u16 references, remove;
	u8 slot, ref_count, ref[HISTB_AVS2_REFS];
	u8 ref_flags[HISTB_AVS2_REFS], col_top_field[HISTB_AVS2_REFS];
};

struct histb_avs2_surface {
	u32 luma, pmv;
};

/* avs2_hal_fill_mix_ram: B = ALIGN(allocation, 1024), requiring B + 0x17000.
 * sed=B, pmv=B+0x1000, rcn=B+0x4000, dblk=B+0x7000, sao=B+0x13000.
 * slices is a separate 16-byte-aligned array of slice_count * 256 bytes.
 */
struct histb_avs2_scratch {
	u32 sed_top, pmv_top, rcn_top, sao_top, dblk_top, slices;
};

/* structure: 1 frame, 0 field. A frame is progressive iff progressive is 1;
 * otherwise top_first selects interlaced TB/BT. For fields top_field selects
 * TOP (1) or BOTTOM (0). Preserve these per-picture flags across reordering.
 */
struct histb_avs2_output {
	u64 timestamp;
	s32 poi;
	u8 slot, type, top_field, structure, top_first, progressive;
};

/* Context/AU/job storage belongs to the caller; none of these functions allocates.
 * One OUTPUT buffer is one complete Annex-B AU (headers may precede the picture).
 * Header-only buffers return 0, a complete picture returns 1, errors are -errno.
 * The AU and its raw, contiguous DMA buffer must survive prepare/build/complete.
 * Sequence/edit reset_before barriers return -EPIPE from prepare while old
 * surfaces remain busy. Drain next_output, clear_dpb, and retry the same AU.
 * prepare normally uses the supplied free slot; repeated GB pictures reuse
 * their existing background slot. The hardware destination is always job.slot.
 * On prepare success call complete exactly once, including submission failure.
 * The core holds all busy_slots surfaces, including undisplayed nonreferences.
 * next_output transfers the display hold to the caller; reference holds remain.
 * Caller-held display surfaces must also be excluded from free-slot selection.
 * Drain at sequence_end/video_edit/EOS before releasing the remaining DPB.
 * Non-decoding user-data/extension units are consumed, not exposed as HDR,
 * colour-description, display-centre, copyright or ROI metadata controls.
 */
void histb_avs2_init(struct histb_avs2_ctx *ctx);
int histb_avs2_parse(struct histb_avs2_ctx *ctx, const u8 *data, u32 size,
		     struct histb_avs2_au *au);
u16 histb_avs2_busy_slots(const struct histb_avs2_ctx *ctx);
int histb_avs2_prepare(struct histb_avs2_ctx *ctx, const struct histb_avs2_au *au,
		       u8 slot, u64 timestamp, struct histb_avs2_job *job);
void histb_avs2_complete(struct histb_avs2_ctx *ctx,
		       const struct histb_avs2_job *job, bool success);
int histb_avs2_next_output(struct histb_avs2_ctx *ctx, bool drain,
			  struct histb_avs2_output *output);
void histb_avs2_clear_dpb(struct histb_avs2_ctx *ctx);
/* All DMA addresses are 32-bit, physical-bypass addresses aligned to 16 bytes.
 * Uncompressed reconstruction only: stride=ALIGN(width,256), height=ALIGN(h,64).
 * 10-bit surfaces have high-byte Y/C followed by quarter-stride low-bit Y/C.
 * pic has PIC_WORDS words; slices has au->slice_count * SLICE_WORDS words.
 * The builder initializes these complete messages; the core performs DMA
 * ownership transitions and programs current luma/strides/low-bit offsets,
 * stream_dma, and message/report addresses in the VDH HAL. Picture words
 * 16..19 contain PMV/RCN/SAO/DBLK top addresses (not SED); no separate DBLK
 * register write is required by the observed AVS2 prepare/start path.
 */
int histb_avs2_build_messages(const struct histb_avs2_job *job,
		 const struct histb_avs2_surface surface[HISTB_AVS2_SLOTS],
		 const struct histb_avs2_scratch *scratch, u32 stream_dma,
		 u32 *pic, u32 *slices);

#endif
