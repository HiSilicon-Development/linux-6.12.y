/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HISTB_VPSS_H
#define HISTB_VPSS_H

#include <linux/types.h>

struct device;
struct histb_vpss;
/* Zero retains the existing CV200 decoder surface contract. */
enum histb_vpss_input_layout {
	HISTB_VPSS_INPUT_CV200,
	HISTB_VPSS_INPUT_MV310_TILE,
	HISTB_VPSS_INPUT_MV310_LINEAR,
};


struct histb_vpss_frame {
	dma_addr_t input_dma;
	/* Complete input/output allocations; mandatory for all DMA and CPU writes. */
	size_t input_size;
	size_t output_size;
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
	bool input_packed_yuv;
	bool output_packed_yuv;
	bool hdr10_to_sdr;
	/*
	 * MV310: uncompressed NV12/NV21, all offsets and strides in bytes.
	 * TILE uses high-eight-bit Y/C plus separate low-two-bit Y/C for
	 * ten-bit input. LINEAR ten-bit samples are packed little-endian,
	 * four samples per five bytes; eight-bit samples are byte-linear.
	 * Plane offsets are relative to input_dma and must fit input_size.
	 * input_height_align is a power of two (TILE >= 16, LINEAR >= 1);
	 * C has half the aligned Y height. No CV200 layout is inferred.
	 *
	 * NV12 output_dma requires a 64-byte-aligned output_stride. P010
	 * requires output_cpu and at least width * 2 bytes per output row:
	 * CF/OUT detile to packed linear 10-bit, then software packs words.
	 * LINEAR ten-bit -> NV12 also requires output_cpu for truncation.
	 * CPU-written outputs require caller-managed DMA cache ownership.
	 * output_size covers output_stride * height * 3 / 2 for NV12/P010,
	 * or output_stride * height for packed YUV output.
	 * Ten-bit output and LINEAR ten-bit -> NV12 are native-size only;
	 * eight-bit output from TILE can be resized through the MV310 ZME.
	 * Packed 4:2:2, compressed tiles, HDR tone mapping and MV310 DEI are
	 * not implemented by this progressive conversion entry point.
	 */
	enum histb_vpss_input_layout input_layout;
	u32 input_y_offset;
	u32 input_c_offset;
	u32 input_c_stride;
	u32 input_low_y_offset;
	u32 input_low_c_offset;
	u32 input_low_stride;
	bool input_nv21;
};
struct histb_vpss_pq_settings {
	u8 brightness;
	u8 contrast;
	u8 saturation;
	u8 hue;
	u8 sharpness;
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
 * The current DEI path consumes linear NV12 capture surfaces.
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
	u32 target_width;
	u32 target_height;
	bool top_field_first;
	bool ten_bit;
	bool	bottom_field;	/* HI_DRV_FIELD_BOTTOM */
	bool tile;
	bool mcdi_en;
	bool tnr_en;
	struct histb_vpss_pq_settings pq;
};

/*
 * De-interlace @frame into @output_dma.  Returns 0, or a negative errno.
 *
 * Widths above 1920 and woven-frame heights above 1088 are rejected,
 * following the BSP's progressive bypass policy in vpss_in_3798cv200.c.
 * The height member counts field lines, so its maximum is 544.
 * All DMA surfaces must fit the shared media SMMU identity window.
 */
int histb_vpss_dei(struct histb_vpss *vpss,
		   const struct histb_vpss_dei_frame *frame,
		   dma_addr_t output_dma);

#endif
