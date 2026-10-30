/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HISTB_VDEC_V5R6_H
#define HISTB_VDEC_V5R6_H

#include <linux/io.h>
#include <linux/types.h>

struct regmap;

/* Addresses are DMA byte addresses until the backend writes the hardware. */
struct histb_v5r6_registers {
	u32 basic_cfg0;
	u32 basic_cfg1;
	dma_addr_t picture;
	dma_addr_t report;
	dma_addr_t stream;
	dma_addr_t reconstruction;
	u32 y_stride;
	u32 chroma_offset;
	u32 low_stride;
	u32 low_y_offset;
	u32 low_chroma_offset;
	bool low_bits;
	bool avs;
	bool emar;
};

int histb_v5r6_configure_clock(struct regmap *crg);
int histb_v5r6_reset(void __iomem *regs);
void histb_v5r6_start(void __iomem *regs,
		     const struct histb_v5r6_registers *config);
void histb_v5r6_encode_h264_picture(__le32 *picture, bool new_picture);
void histb_v5r6_encode_h264_slice(__le32 *slice);
void histb_v5r6_encode_hevc_picture(__le32 *picture);
void histb_v5r6_encode_hevc_slice(__le32 *slice);
/*
 * Fresh CV200-compatible logical messages, in little-endian word order and
 * with physical byte addresses. Convert once, after all words are packed.
 * MPEG2 picture/slice slots contain 64/8 words; AVS slots contain 80/64 words.
 * AVS tail_offset is the byte delta from STREAM_BASE to a zero-filled stream
 * tail of at least 4096 bytes; it is not an absolute or encoded address.
 */
void histb_v5r6_encode_mpeg2_picture(__le32 *picture);
void histb_v5r6_encode_mpeg2_slice(__le32 *slice);
void histb_v5r6_encode_avs_picture(__le32 *picture,
				   unsigned int slice_count);
void histb_v5r6_encode_avs_slice(__le32 *slice, u32 tail_offset);
int histb_v5r6_reset_bpd(void __iomem *regs);

#endif
