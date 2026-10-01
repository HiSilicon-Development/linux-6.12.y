/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HISTB_VPSS_H
#define HISTB_VPSS_H

#include <linux/types.h>

struct device;
struct histb_vpss;

struct histb_vpss_frame {
	dma_addr_t input_dma;
	/* Size of the complete decoder tile surface, when known. */
	size_t input_size;
	dma_addr_t output_dma;
	void *output_cpu;
	u32 input_width;
	u32 input_height;
	u32 width;
	u32 height;
	u32 input_stride;
	u32 input_height_align;
	u32 output_stride;
	bool input_ten_bit;
	bool output_ten_bit;
};

struct histb_vpss *histb_vpss_get(struct device *consumer);
void histb_vpss_put(struct histb_vpss *vpss);
int histb_vpss_detile(struct histb_vpss *vpss,
		      const struct histb_vpss_frame *frame);

/*
 * Hardware de-interlacing, the DI block inside the VPSS.
 *
 * It consumes four consecutive display-ordered fields and emits CUR, the
 * second field of LAST/CUR/NEXT1/NEXT2, as one progressive frame.
 * Field address registers sit at 0x100..0x13c in groups of four - control,
 * Y, C, stride - with LAST at 0x110, CUR at 0x100, NEXT1 at 0x120 and NEXT2
 * at 0x130.
 *
 * A field lives in a woven frame-sized surface. The bottom field's luma
 * and chroma addresses advance by one frame pitch, matching BSP
 * VPSS_IMG_CopyAddr(); bfield_mode identifies CUR's parity. Chroma starts
 * one full woven-frame height beyond the same-parity luma address.
 *
 * The input is linear NV12.  Decoder tile surfaces must be detiled first.
 */
struct histb_vpss_dei_frame {
	dma_addr_t ref_dma;	/* oldest field, for motion estimation */
	dma_addr_t cur_dma;
	dma_addr_t nxt1_dma;
	dma_addr_t nxt2_dma;	/* newest field */
	u32 width;
	u32 height;		/* height of one field */
	u32 stride;		/* frame pitch of the input surfaces */
	u32 output_stride;	/* pitch the de-interlaced frame is written at */
	bool top_field_first;
	bool ten_bit;
	bool	bottom_field;	/* HI_DRV_FIELD_BOTTOM */
	bool tile;
};

/*
 * De-interlace @frame into @output_dma.  Returns 0, or a negative errno.
 *
 * Inputs wider than 1920 or taller than 1088 are rejected rather than
 * silently left interlaced: the BSP forces bProgressive for anything larger
 * (vpss_in_3798cv200.c:178-330), so the caller must scale down first.
 */
int histb_vpss_dei(struct histb_vpss *vpss,
		   const struct histb_vpss_dei_frame *frame,
		   dma_addr_t output_dma);

#endif
