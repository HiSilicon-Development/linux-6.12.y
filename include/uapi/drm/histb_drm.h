/* SPDX-License-Identifier: MIT */
/*
 * Copyright (c) 2026 HiSTB DRM contributors
 */
#ifndef _UAPI_HISTB_DRM_H_
#define _UAPI_HISTB_DRM_H_

#include "drm_fourcc.h"

/*
 * PROJECT-LOCAL EXPERIMENTAL namespace, NOT an upstream-assigned DRM vendor ID.
 * Do not exchange this modifier outside matching HiSTB kernel/userspace builds.
 * Formal upstream registration is required before publishing a stable ABI; a
 * future registered namespace must be negotiated as a distinct modifier.
 */
#define DRM_FORMAT_MOD_VENDOR_HISTB 0x7e

/*
 * Uncompressed Hi3798CV200 VDH HEVC Main10 reconstruction surface, consumed
 * directly by VDP V0 SP_TILE_64 (ifmt 7, data_width 1). This is NOT linear
 * NV15 or P010, NOT the older SP_TILE format, and NOT compressed tile data.
 *
 * Canonical pair: DRM_FORMAT_YUV420_10BIT / DRM_FORMAT_MOD_HISTB_VDH_10BIT.
 * Exactly ONE memory plane and ONE DMA object, with offset[0] == 0. Chroma is
 * interleaved Cr,Cb (VDH order; VDP uv_order = 0). Each component is split
 * into its high eight and low two bits, in the CV200 VDH tile ordering
 * (64-byte high-eight tile rows; separate two-bit tile rows encode four
 * adjacent samples per byte). Both portions retain VDH ordering, not rows.
 *
 * AddFB2 width/height MUST be the full VDH CODED dimensions, not the visible
 * rectangle or a cropped AVFrame height. Consumers must obtain coded geometry
 * from the producer; it cannot be recovered from a DMA object's total size
 * (decoder tail guards may follow the surface). Visible geometry/crop belongs
 * in the plane source rectangle. Never infer coded height from visible height,
 * even when their difference is small: crossing a 64-line boundary changes all
 * subsequent regions. A consumer without exact coded geometry must reject.
 *
 * Let S = ALIGN(coded_width, 256), H = ALIGN(coded_height, 64). pitch[0] is S,
 * in BYTES of the high-eight region per logical row (not 10-bit packed pitch).
 * All offsets below are bytes from the single object's start:
 *
 *   high-eight Y:     offset 0,          size S * H
 *   high-eight CrCb:  offset S * H,      size S * H / 2
 *   low-two Y:        offset S * H * 3/2,size (S / 4) * H
 *   low-two CrCb:     offset S * H * 7/4,size (S / 4) * H / 2
 *
 * Minimum object size is S * H * 15/8. A producer may append a tail guard;
 * it does not change region offsets. All four region addresses must be
 * 64-byte aligned and the complete surface must fit the 32-bit DMA aperture.
 * No auxiliary memory planes, headers, compression metadata or second object.
 *
 * Current KMS implementation: CV200 V0 only, progressive 2D, even coded
 * dimensions; even source width/height/y and source x a multiple of 64 pixels.
 * No rotation/reflection. Tile crop registers select the source rectangle;
 * region base addresses never incorporate a linear source-x/source-y offset.
 * V1, graphics and MV310 cannot consume this pair. Explicit
 * DRM_MODE_FB_MODIFIERS is mandatory; implicit/LINEAR YUV420_10BIT is invalid.
 */
#define DRM_FORMAT_MOD_HISTB_VDH_10BIT fourcc_mod_code(HISTB, 1)

#endif /* _UAPI_HISTB_DRM_H_ */
