/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __HISTB_VDP_ZME_H__
#define __HISTB_VDP_ZME_H__

#include <linux/types.h>

#define HISTB_VDP_CV200_ZME_SIZE 1024

/**
 * histb_vdp_cv200_build_zme - build the normal CV200 full-tap FIR DMA image
 * @buffer: writable CPU buffer of at least HISTB_VDP_CV200_ZME_SIZE bytes
 * @src_width: nonzero input luma width in pixels
 * @src_height: nonzero input luma height in pixels
 * @dst_width: nonzero scaler-output luma width in pixels
 * @dst_height: nonzero scaler-output luma height in pixels
 *
 * For progressive linear NV12/NV21 (4:2:0) input and the BSP's 4:2:2 scaler
 * output only. Dimensions describe this scaler stage, not a subsequent scaler.
 * The caller validates dimensions and the hardware's supported scaling limits;
 * availability of a coefficient class does not establish a supported range.
 *
 * Clears the whole image and writes LH8 at 0 (192 bytes), CH4 at 256 (96 bytes),
 * LV6 at 512 (144 bytes), and CV4 at 768 (96 bytes). Stored phases are 0..16
 * followed by the BSP's zero padding row 17. Each little-endian 16-byte record
 * holds twelve signed 10-bit coefficients followed by eight zero bits; the
 * arithmetic normalization is Q9, not Q10. No MMIO or allocation is performed.
 *
 * Hardware contract for this image: zme_in_fmt=1 (420), zme_out_fmt=0 (422),
 * hfir_order=1 (VH), vtap_reduce=0, vfir_1tap_en=0; enable both luma/chroma
 * scaler paths and their FIRs in each direction. V0 mid-filter enables are
 * zero in the BSP. For progressive input/output the horizontal luma/chroma
 * phases and vertical luma phases are zero; vertical chroma phases are -512
 * in Q12 units, including the bottom-phase register. These phases describe
 * the BSP's centered 420 chroma convention, not arbitrary chroma siting.
 *
 * The shared luma-based hardware ratios are floor((src_width / 2) * 2^20 /
 * dst_width) * 2 horizontally (BSP integer operation order, even input width)
 * and floor(src_height * 2^12 / dst_height) vertically. Hardware derives the
 * chroma ratio from the format; do not halve the programmed vertical ratio.
 * Filter-class selection instead uses output/input integer percentages: HC
 * equals HL, and VC equals twice the already-truncated VL percentage.
 *
 * The caller owns DMA mapping/visibility and inactive-bank lifetime. Program
 * HL/HC/VL/VC coefficient addresses to DMA base + 0/256/512/768 respectively
 * (16-byte aligned), then trigger both H coefficient-update bits and both V
 * coefficient-update bits before the plane register update. Do not overwrite
 * coefficients that hardware may still fetch. This is not an MV310 image.
 */
void histb_vdp_cv200_build_zme(void *buffer, u32 src_width, u32 src_height,
			     u32 dst_width, u32 dst_height);

#endif /* __HISTB_VDP_ZME_H__ */
