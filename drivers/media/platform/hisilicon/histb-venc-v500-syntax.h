/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HISTB_VENC_V500_SYNTAX_H
#define HISTB_VENC_V500_SYNTAX_H

#include <linux/types.h>

/* Fits the V500 parameter-set register image's 48 words. */
#define HISTB_VENC_V500_HEVC_PS_MAX_BYTES 192

/* Progressive Main, 8-bit 4:2:0, one previous-picture reference, I/IDR or P. */
struct histb_venc_v500_picture {
	u32 width;
	u32 height;
	/* POC since the last IDR; the hardware/header use the low 16 bits. */
	u32 poc;
	bool intra;
	u8 qp;
};

/*
 * CPU-endian register values, not a byte stream. Each complete word contains
 * 32 MSB-first syntax bits; the final partial word is right-aligned, like BSP
 * H264e_MakeSlcHdr/H265e_MakeSliceHead. Hardware adds the NAL header, slice
 * address, alignment and emulation prevention; do not add these to the fields.
 */
struct histb_venc_v500_slice {
	u32 slc_hdr_stream[4];
	u32 reorder_stream[2];
	u32 marking_stream[2];
	/* H.264: header | reorder << 8 | marking << 16; second word zero.
	 * HEVC: part1 bit count, part2 bit count, respectively.
	 */
	u32 slc_hdr_bits[2];
	u32 cabac[8];
	u32 part1;
	u32 part1_bits;
};

/*
 * Concatenated VPS/SPS/PPS with four-byte Annex B start codes and escaped EBSP.
 * Width/height must be even, 8..1920 and 8..1088, respectively; coded dimensions
 * are rounded to eight and the conformance window preserves the visible size.
 * Main level 5.1 is signalled (rather than the BSP's invalid level zero).
 * CUQP is enabled, base QP is 26; TMVP, SAO and 8x8 IPCM match V500 defaults.
 * Returns zero or -EINVAL/-ENOSPC. *bytes is zero on error; dst may be partial
 * after -ENOSPC. No Main10, B, tiles, WPP or weighted prediction are supported.
 */
int histb_venc_v500_hevc_parameter_sets(u32 width, u32 height, u8 *dst,
				       size_t capacity, size_t *bytes);
int histb_venc_v500_hevc_slice(const struct histb_venc_v500_picture *picture,
			     struct histb_venc_v500_slice *slice);

/*
 * SPS/PPS are supplied by the frontend: PPS/SPS id zero, four-bit frame_num,
 * pic_order_cnt_type=2 and one L0 reference. All intra pictures are IDR, with
 * frame_num zero; idr_pic_id and inter frame_num wrap modulo 16 as in the BSP.
 * The hardware supplies QP/deblocking/CABAC syntax outside these portions.
 */
int histb_venc_v500_h264_slice(u32 frame_num, u8 idr_pic_id, bool intra,
			     struct histb_venc_v500_slice *slice);

#endif
