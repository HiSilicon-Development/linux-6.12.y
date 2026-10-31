/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __HISTB_VDP_HDR_H__
#define __HISTB_VDP_HDR_H__

#include <linux/io.h>
#include <linux/types.h>

/*
 * CV200 linear 10-bit SP420 is NV15, not P010 and not tiled high8/low2.
 * Pitches are bytes; each group of four samples occupies five bytes.
 * For P010 import, allocate/reuse a DMA shadow outside these helpers, then
 * convert Y (height rows) and interleaved CbCr (height / 2 rows) separately.
 * Destination pitch >= DIV_ROUND_UP(width, 4) * 5, source pitch >= width * 2.
 * Width must be even; allocation/object bounds and overflow are caller checks.
 * A partial final group is padded with zero samples, never read past width.
 * Only the ten significant P010 bits are packed; this is not tone mapping.
 * No DMA ownership, mapping, allocation, cache sync or register update here.
 */
void histb_vdp_cv200_p010_to_nv15(u8 *dst, u32 dst_pitch,
			       const u8 *src, u32 src_pitch,
			       u32 width, u32 rows);

/*
 * Convert nonlinear BT.2020 non-constant-luminance limited YCbCr to full RGB.
 * PQ/HLG remain PQ/HLG: no EOTF/OETF, gamut conversion or tone mapping.
 * Call after the normal CV200 V0 identity-chain setup; that setup must bypass
 * VDM gamma/degamma/norm, GDM and CVM tone mapping and leave RGB output unclipped
 * (DHD HDMI clip 0..1023). The parent owns HDR_UPD and the plane/CRTC latch.
 * Restrict HDR composition to already-HDR video: SDR graphics cannot be mixed
 * into this nonlinear RGB domain without their own real GDM conversion.
 * To restore SDR, run the existing SDR identity-chain and SDR CSC programming;
 * never retain this BT.2020 matrix for BT.601/709 frames.
 */
void histb_vdp_cv200_hdr_bt2020_csc(void __iomem *base);

#endif
