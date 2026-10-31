// SPDX-License-Identifier: GPL-2.0-only
/*
 * Independently authored CV200 pixel helpers. Hardware/numeric provenance:
 * HiSTBLinuxV100R005C00SPC060/source/msp/drv/:
 *
 * vpss/vpss_v4_0/vpss_osal.c:367-427 decodes linear 10-bit samples as an
 * LSB-first bitstream, four samples per five bytes; 1802-1832 sizes linear
 * SP420 at ALIGN(DIV_ROUND_UP(width * 10, 8), 16) bytes/row. CV200 VPSS HAL
 * hal/hi3798cv200/vpss_hal_3798cv200.c:1204-1208 selects output format/depth.
 * vo/vdp_v4_0/hal/common/drv_xdr_adapter.c:989-1015 transfers the frame's
 * format, depth, addresses and byte strides to HDR scanout. In 98cv200/,
 * vdp_drv_hihdr.c:819-823 sets fetch format/depth; vdp_hal_vid.c:242-269
 * writes byte addresses/strides and 443-455 writes data_width. Separate
 * low2 addresses belong to tiled fetch (vdp_drv_vid.c:673-691); they are
 * not the representation of linear NV15.
 *
 * pq/pq_v4_0/mng/pq_mng_csc.c:773-793 supplies BT.2020 limited YCbCr to
 * full RGB coefficients and DC. vo/vdp_v4_0/hal/98cv200/vdp_drv_vid.c:
 * 1435-1438 selects that conversion, 1586-1626 specifies Q10 * 8 -> Q13
 * and 10-bit input DC * 16 -> internal 14-bit, with zero output DC.
 * The vendor files credit Hisilicon Technologies Co., Ltd.
 *
 * Already-HDR scanout is distinct from SDR-to-HDR: vdp_drv_hihdr.c:
 * 1182-1231 bypasses the video transfer/tone-map stages for matching
 * HDR10/HDR10 or HLG/HLG. This implementation additionally applies the
 * documented nonlinear YCbCr-to-RGB matrix for an RGB HDMI input bus.
 * Neither HDR transfer needs a LUT in this route. SDR graphics GDM, HDR
 * tone mapping and HDR10/HLG cross-conversion are deliberately not enabled.
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/unaligned.h>

#include "histb-vdp-hdr.h"

#define CV200_V0_CSC_IDC		0x0880
#define CV200_VDM_CTRL		0x3400
#define CV200_VDM_YUV2RGB_CTRL	0x3500
#define CV200_VDM_YUV2RGB_COEF	0x3504
#define CV200_VDM_YUV2RGB_SCALE	0x354c
#define CV200_VDM_YUV2RGB_IN_DC	0x3550
#define CV200_VDM_YUV2RGB_OUT_DC	0x355c

void histb_vdp_cv200_p010_to_nv15(u8 *dst, u32 dst_pitch,
			       const u8 *src, u32 src_pitch,
			       u32 width, u32 rows)
{
	u32 y;

	for (y = 0; y < rows; y++) {
		const u8 *in = src;
		u8 *out = dst;
		u32 x = 0;

		for (; x + 4 <= width; x += 4) {
			u32 a = get_unaligned_le16(in) >> 6;
			u32 b = get_unaligned_le16(in + 2) >> 6;
			u32 c = get_unaligned_le16(in + 4) >> 6;
			u32 d = get_unaligned_le16(in + 6) >> 6;

			put_unaligned_le32(a | b << 10 | c << 20 | d << 30,
					   out);
			out[4] = d >> 2;
			in += 8;
			out += 5;
		}
		/* SP420 has even width: the only partial group has two samples. */
		if (x < width) {
			u32 a = get_unaligned_le16(in) >> 6;
			u32 b = get_unaligned_le16(in + 2) >> 6;

			put_unaligned_le32(a | b << 10, out);
			out[4] = 0;
		}
		src += src_pitch;
		dst += dst_pitch;
	}
}

void histb_vdp_cv200_hdr_bt2020_csc(void __iomem *base)
{
	static const s16 q10[9] = {
		1192, 0, 1719,
		1192, -192, -666,
		1192, 2193, 0,
	};
	static const s16 dc[3] = { -1024, -8192, -8192 };
	unsigned int i;

	for (i = 0; i < 9; i++)
		writel((q10[i] * 8) & 0xffff,
		       base + CV200_VDM_YUV2RGB_COEF + i * 4);
	writel(13, base + CV200_VDM_YUV2RGB_SCALE);
	for (i = 0; i < 3; i++) {
		writel(dc[i] & 0x3fff, base + CV200_VDM_YUV2RGB_IN_DC + i * 4);
		writel(0, base + CV200_VDM_YUV2RGB_OUT_DC + i * 4);
	}
	writel(0, base + CV200_V0_CSC_IDC);
	writel(BIT(0) | BIT(1), base + CV200_VDM_YUV2RGB_CTRL);
	/* VDM + YCbCr input + internal 14-bit; transfer/normalization OFF. */
	writel(BIT(0) | BIT(4) | BIT(6), base + CV200_VDM_CTRL);
}
