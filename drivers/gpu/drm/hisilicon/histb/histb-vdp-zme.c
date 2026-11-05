// SPDX-License-Identifier: GPL-2.0-only
/*
 * Independently authored CV200 ZME coefficient selection/serialization code.
 *
 * Numerical hardware filter parameters follow HiSTBLinuxV100R005C00SPC060,
 * source/msp/drv/pq/pq_v4_0/mng/pq_mng_zme_coef.h:
 *   LH8: lines 45-157; LV6: 607-742; CH4/CV4: 877-902;
 *   active VDP provider g_pVZmeCoef: 1916-1926, not the VPSS provider.
 * Source table names are retained below for traceability. Lanc3, Lanc2, 3M
 * luma and both chroma tables also occur in this kernel's GPL-2.0-only
 * drivers/media/platform/hisilicon/histb-vpss.c and are reused from there.
 *
 * The BSP coefficient header has no license notice. The neighboring BSP
 * pq_mng_zme.c credits Copyright (C) 2014 Hisilicon Technologies Co., Ltd.,
 * All rights reserved, and carries a proprietary, permission-required notice.
 * This records provenance, not a vendor GPL grant. No executable BSP code is
 * incorporated here; the GPL identifier describes the independently written
 * driver implementation.
 *
 * Selection/transport specification: source/msp/drv/vo/vdp_v4_0/hal/98cv200/
 * vdp_drv_vid.c (VDP_DRV_GetVidZmeStrategy and VDP_DRV_SetVidZme) and
 * vdp_drv_pq_zme.c (DRV_VDP_TranlateZoomCoef).
 */

#include <linux/math64.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "histb-vdp-zme.h"

/* Phase-major signed Q9 values; row 17 is the required zero padding row. */
/* s16ZmeCoef_8T32P_Lanc3 */
static const s16 cv200_lh8_lanc3[18 * 8] = {
	4, -22, 40, 468, 40, -22, 4, 0,
	3, -18, 26, 468, 54, -26, 5, 0,
	2, -14, 14, 466, 68, -30, 6, 0,
	2, -11, 2, 462, 84, -34, 7, 0,
	1, -7, -9, 457, 100, -38, 8, 0,
	1, -4, -18, 450, 116, -42, 9, 0,
	1, -2, -27, 443, 133, -46, 10, 0,
	0, 2, -35, 434, 151, -50, 10, 0,
	0, 4, -42, 425, 168, -54, 11, 0,
	0, 6, -49, 414, 186, -57, 12, 0,
	0, 8, -54, 401, 204, -60, 13, 0,
	0, 10, -58, 387, 222, -62, 13, 0,
	0, 11, -62, 374, 240, -65, 14, 0,
	0, 12, -65, 359, 258, -66, 14, 0,
	0, 13, -67, 344, 276, -68, 14, 0,
	0, 14, -68, 327, 293, -68, 14, 0,
	0, 14, -68, 310, 310, -68, 14, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
};

/* s16ZmeCoef_8T32P_Lanc2 */
static const s16 cv200_lh8_lanc2[18 * 8] = {
	-16, 0, 145, 254, 145, 0, -16, 0,
	-16, -2, 140, 253, 151, 3, -17, 0,
	-15, -5, 135, 253, 157, 5, -18, 0,
	-14, -7, 129, 252, 162, 8, -18, 0,
	-13, -9, 123, 252, 167, 11, -19, 0,
	-13, -11, 118, 250, 172, 15, -19, 0,
	-12, -12, 112, 250, 177, 18, -20, -1,
	-11, -14, 107, 247, 183, 21, -20, -1,
	-10, -15, 101, 245, 188, 25, -21, -1,
	-9, -16, 96, 243, 192, 29, -21, -2,
	-8, -18, 90, 242, 197, 33, -22, -2,
	-8, -19, 85, 239, 202, 37, -22, -2,
	-7, -19, 80, 236, 206, 41, -22, -3,
	-7, -20, 75, 233, 210, 46, -22, -3,
	-6, -21, 69, 230, 215, 50, -22, -3,
	-5, -21, 65, 226, 219, 55, -22, -5,
	-5, -21, 60, 222, 222, 60, -21, -5,
	0, 0, 0, 0, 0, 0, 0, 0,
};

/* s16ZmeCoef_8T32P_3M_a19 */
static const s16 cv200_lh8_3m_a19[18 * 8] = {
	-18, 18, 144, 226, 144, 19, -17, -4,
	-17, 16, 139, 226, 148, 21, -17, -4,
	-17, 13, 135, 227, 153, 24, -18, -5,
	-17, 11, 131, 226, 157, 27, -18, -5,
	-17, 9, 126, 225, 161, 30, -17, -5,
	-16, 6, 122, 225, 165, 33, -17, -6,
	-16, 4, 118, 224, 169, 37, -17, -7,
	-16, 2, 113, 224, 173, 40, -17, -7,
	-15, 0, 109, 222, 177, 43, -17, -7,
	-15, -1, 104, 220, 181, 47, -16, -8,
	-14, -3, 100, 218, 185, 51, -16, -9,
	-14, -5, 96, 217, 188, 54, -15, -9,
	-14, -6, 91, 214, 192, 58, -14, -9,
	-13, -7, 87, 212, 195, 62, -14, -10,
	-13, -9, 83, 210, 198, 66, -13, -10,
	-12, -10, 79, 207, 201, 70, -12, -11,
	-12, -11, 74, 205, 205, 74, -11, -12,
	0, 0, 0, 0, 0, 0, 0, 0,
};

/* s16ZmeCoef_8T32P_2M_a05 */
static const s16 cv200_lh8_2m_a05[18 * 8] = {
	14, 66, 113, 133, 113, 66, 14, -7,
	12, 65, 112, 133, 114, 68, 15, -7,
	11, 63, 111, 132, 115, 70, 17, -7,
	10, 62, 110, 132, 116, 71, 18, -7,
	8, 60, 108, 132, 118, 73, 20, -7,
	7, 58, 107, 132, 119, 75, 21, -7,
	6, 56, 106, 132, 120, 76, 23, -7,
	5, 55, 105, 131, 121, 78, 24, -7,
	4, 53, 103, 131, 122, 80, 26, -7,
	3, 51, 102, 131, 122, 81, 28, -6,
	2, 50, 101, 130, 123, 83, 29, -6,
	1, 48, 99, 131, 124, 84, 31, -6,
	0, 46, 98, 129, 125, 86, 33, -5,
	-1, 45, 97, 128, 126, 88, 34, -5,
	-2, 43, 95, 130, 126, 89, 36, -5,
	-3, 41, 94, 128, 127, 91, 38, -4,
	-3, 39, 92, 128, 128, 92, 39, -3,
	0, 0, 0, 0, 0, 0, 0, 0,
};

/* s16ZmeCoef_8T32P_1_5M_a05 */
static const s16 cv200_lh8_1_5m_a05[18 * 8] = {
	39, 69, 93, 102, 93, 69, 39, 8,
	38, 68, 92, 102, 93, 70, 40, 9,
	37, 67, 91, 102, 93, 71, 41, 10,
	36, 66, 91, 101, 94, 71, 42, 11,
	35, 65, 90, 102, 94, 72, 43, 11,
	34, 64, 89, 102, 94, 73, 44, 12,
	33, 63, 88, 101, 95, 74, 45, 13,
	32, 62, 88, 100, 95, 75, 46, 14,
	31, 62, 87, 100, 95, 75, 47, 15,
	30, 61, 86, 99, 96, 76, 48, 16,
	29, 60, 86, 98, 96, 77, 49, 17,
	28, 59, 85, 98, 96, 78, 50, 18,
	27, 58, 84, 99, 97, 78, 50, 19,
	26, 57, 83, 99, 97, 79, 51, 20,
	25, 56, 83, 98, 97, 80, 52, 21,
	24, 55, 82, 97, 98, 81, 53, 22,
	23, 54, 81, 98, 98, 81, 54, 23,
	0, 0, 0, 0, 0, 0, 0, 0,
};

/* s16ZmeCoef_6T32P_6M_a25 */
static const s16 cv200_lv6_6m_a25[18 * 6] = {
	-34, 78, 415, 78, -34, 9,
	-32, 66, 415, 90, -36, 9,
	-29, 54, 413, 103, -38, 9,
	-26, 43, 411, 116, -41, 9,
	-23, 33, 406, 129, -42, 9,
	-21, 24, 401, 143, -44, 9,
	-18, 14, 396, 157, -46, 9,
	-16, 6, 389, 171, -47, 9,
	-14, -2, 382, 185, -48, 9,
	-11, -9, 374, 199, -49, 8,
	-9, -16, 365, 213, -49, 8,
	-7, -21, 354, 228, -49, 7,
	-5, -27, 345, 242, -49, 6,
	-3, -32, 333, 256, -48, 6,
	-1, -36, 322, 269, -46, 4,
	0, -39, 309, 283, -44, 3,
	2, -42, 296, 296, -42, 2,
	0, 0, 0, 0, 0, 0,
};

/* s16ZmeCoef_6T32P_5M_a25 */
static const s16 cv200_lv6_5m_a25[18 * 6] = {
	-31, 104, 362, 104, -31, 4,
	-30, 94, 362, 114, -32, 4,
	-29, 84, 361, 125, -32, 3,
	-28, 75, 359, 136, -33, 3,
	-27, 66, 356, 147, -33, 3,
	-25, 57, 353, 158, -33, 2,
	-24, 49, 349, 169, -33, 2,
	-22, 41, 344, 180, -32, 1,
	-20, 33, 339, 191, -31, 0,
	-19, 26, 333, 203, -30, -1,
	-17, 19, 327, 214, -29, -2,
	-16, 13, 320, 225, -27, -3,
	-14, 7, 312, 236, -25, -4,
	-13, 1, 305, 246, -22, -5,
	-11, -4, 295, 257, -19, -6,
	-10, -8, 286, 267, -16, -7,
	-9, -12, 277, 277, -12, -9,
	0, 0, 0, 0, 0, 0,
};

/* s16ZmeCoef_6T32P_4M_a20 */
static const s16 cv200_lv6_4m_a20[18 * 6] = {
	-20, 130, 297, 130, -20, -5,
	-21, 122, 298, 138, -19, -6,
	-22, 115, 297, 146, -17, -7,
	-22, 108, 296, 153, -16, -7,
	-23, 101, 295, 161, -14, -8,
	-23, 93, 294, 169, -12, -9,
	-24, 87, 292, 177, -10, -10,
	-24, 80, 289, 185, -7, -11,
	-24, 73, 286, 193, -4, -12,
	-23, 66, 283, 200, -1, -13,
	-23, 60, 279, 208, 2, -14,
	-23, 54, 276, 215, 5, -15,
	-22, 48, 271, 222, 9, -16,
	-21, 42, 266, 229, 13, -17,
	-21, 37, 261, 236, 17, -18,
	-21, 32, 255, 242, 22, -18,
	-20, 27, 249, 249, 27, -20,
	0, 0, 0, 0, 0, 0,
};

/* s16ZmeCoef_6T32P_3M_a15 */
static const s16 cv200_lv6_3m_a15[18 * 6] = {
	16, 136, 217, 136, 16, -9,
	13, 132, 217, 141, 18, -9,
	11, 128, 217, 145, 21, -10,
	9, 124, 216, 149, 24, -10,
	7, 119, 216, 153, 27, -10,
	5, 115, 216, 157, 30, -11,
	3, 111, 215, 161, 33, -11,
	1, 107, 214, 165, 36, -11,
	0, 102, 213, 169, 39, -11,
	-2, 98, 211, 173, 43, -11,
	-3, 94, 209, 177, 46, -11,
	-4, 90, 207, 180, 50, -11,
	-5, 85, 206, 184, 53, -11,
	-6, 81, 203, 187, 57, -10,
	-7, 77, 201, 190, 61, -10,
	-8, 73, 198, 193, 65, -9,
	-9, 69, 196, 196, 69, -9,
	0, 0, 0, 0, 0, 0,
};

/* s16ZmeCoef_6T32P_2M_a05 */
static const s16 cv200_lv6_2m_a05[18 * 6] = {
	66, 115, 138, 115, 66, 12,
	64, 114, 136, 116, 68, 14,
	63, 113, 134, 117, 70, 15,
	61, 111, 135, 118, 71, 16,
	59, 110, 133, 119, 73, 18,
	57, 108, 134, 120, 74, 19,
	55, 107, 133, 121, 76, 20,
	53, 105, 133, 121, 78, 22,
	51, 104, 133, 122, 79, 23,
	49, 102, 132, 123, 81, 25,
	47, 101, 132, 124, 82, 26,
	45, 99, 131, 125, 84, 28,
	44, 98, 130, 125, 85, 30,
	42, 96, 130, 126, 87, 31,
	40, 95, 128, 127, 89, 33,
	38, 93, 129, 127, 90, 35,
	36, 92, 128, 128, 92, 36,
	0, 0, 0, 0, 0, 0,
};

/* s16ZmeCoef_6T32P_1_5M_a05 */
static const s16 cv200_lv6_1_5m_a05[18 * 6] = {
	80, 105, 116, 105, 80, 26,
	79, 104, 115, 105, 81, 28,
	77, 103, 116, 106, 81, 29,
	76, 102, 115, 106, 82, 31,
	74, 101, 115, 106, 83, 33,
	73, 100, 114, 106, 84, 35,
	71, 99, 114, 107, 84, 37,
	70, 98, 113, 107, 85, 39,
	68, 98, 113, 107, 86, 40,
	67, 97, 112, 108, 86, 42,
	65, 96, 112, 108, 87, 44,
	63, 95, 112, 108, 88, 46,
	62, 94, 112, 108, 88, 48,
	60, 93, 111, 109, 89, 50,
	58, 93, 111, 109, 90, 51,
	57, 92, 110, 110, 90, 53,
	55, 91, 110, 110, 91, 55,
	0, 0, 0, 0, 0, 0,
};

/* s16ZmeCoef_4T32P_5M_a15 */
static const s16 cv200_c4_5m_a15[18 * 4] = {
	103, 335, 103, -29,
	92, 335, 112, -27,
	84, 335, 121, -28,
	75, 334, 131, -28,
	67, 332, 141, -28,
	59, 329, 152, -28,
	51, 326, 162, -27,
	43, 323, 173, -27,
	36, 319, 183, -26,
	30, 313, 194, -25,
	23, 308, 204, -23,
	17, 301, 215, -21,
	12, 295, 225, -20,
	6, 288, 235, -17,
	2, 280, 244, -14,
	-3, 271, 254, -10,
	-7, 263, 263, -7,
	0, 0, 0, 0,
};

/* s16ZmeCoef_4T32P_4M_a15 */
static const s16 cv200_c4_4m_a15[18 * 4] = {
	120, 281, 120, -9,
	113, 281, 127, -9,
	106, 280, 134, -8,
	99, 279, 141, -7,
	92, 277, 148, -5,
	85, 275, 156, -4,
	79, 273, 162, -2,
	72, 270, 170, 0,
	66, 267, 177, 2,
	61, 263, 184, 4,
	56, 259, 191, 6,
	50, 255, 198, 9,
	44, 251, 205, 12,
	40, 246, 211, 15,
	34, 241, 218, 19,
	31, 235, 224, 22,
	26, 230, 230, 26,
	0, 0, 0, 0,
};

enum histb_zme_ratio {
	HISTB_ZME_GT1,
	HISTB_ZME_EQ1,
	HISTB_ZME_GE075,
	HISTB_ZME_GE050,
	HISTB_ZME_GE033,
	HISTB_ZME_GE025,
	HISTB_ZME_LT025,
};

struct histb_zme_coefficients {
	const s16 *lh8;
	const s16 *lv6;
	const s16 *c4;
};

/* Normal full-tap columns of the active PQ g_pVZmeCoef provider. */
static const struct histb_zme_coefficients cv200_coefficients[] = {
	[HISTB_ZME_GT1] = {
		cv200_lh8_lanc3, cv200_lv6_6m_a25, cv200_c4_5m_a15,
	},
	[HISTB_ZME_EQ1] = {
		cv200_lh8_lanc3, cv200_lv6_6m_a25, cv200_c4_5m_a15,
	},
	[HISTB_ZME_GE075] = {
		cv200_lh8_lanc2, cv200_lv6_5m_a25, cv200_c4_5m_a15,
	},
	[HISTB_ZME_GE050] = {
		cv200_lh8_lanc2, cv200_lv6_4m_a20, cv200_c4_5m_a15,
	},
	[HISTB_ZME_GE033] = {
		cv200_lh8_3m_a19, cv200_lv6_3m_a15, cv200_c4_4m_a15,
	},
	[HISTB_ZME_GE025] = {
		cv200_lh8_2m_a05, cv200_lv6_2m_a05, cv200_c4_4m_a15,
	},
	[HISTB_ZME_LT025] = {
		cv200_lh8_1_5m_a05, cv200_lv6_1_5m_a05, cv200_c4_4m_a15,
	},
};

static enum histb_zme_ratio histb_zme_ratio(u64 percent)
{
	if (percent > 100)
		return HISTB_ZME_GT1;
	if (percent == 100)
		return HISTB_ZME_EQ1;
	if (percent >= 75)
		return HISTB_ZME_GE075;
	if (percent >= 50)
		return HISTB_ZME_GE050;
	if (percent >= 33)
		return HISTB_ZME_GE033;
	if (percent >= 25)
		return HISTB_ZME_GE025;
	return HISTB_ZME_LT025;
}

/* Each record concatenates twelve signed 10-bit fields, then eight zeros. */
static void histb_zme_pack(u8 *dst, const s16 *coef, unsigned int records)
{
	while (records--) {
		u32 c0 = coef[0] & 0x3ff;
		u32 c1 = coef[1] & 0x3ff;
		u32 c2 = coef[2] & 0x3ff;
		u32 c3 = coef[3] & 0x3ff;
		u32 c4 = coef[4] & 0x3ff;
		u32 c5 = coef[5] & 0x3ff;
		u32 c6 = coef[6] & 0x3ff;
		u32 c7 = coef[7] & 0x3ff;
		u32 c8 = coef[8] & 0x3ff;
		u32 c9 = coef[9] & 0x3ff;
		u32 c10 = coef[10] & 0x3ff;
		u32 c11 = coef[11] & 0x3ff;

		put_unaligned_le32(c0 | c1 << 10 | c2 << 20 | c3 << 30,
				   dst);
		put_unaligned_le32(c3 >> 2 | c4 << 8 | c5 << 18 | c6 << 28,
				   dst + 4);
		put_unaligned_le32(c6 >> 4 | c7 << 6 | c8 << 16 | c9 << 26,
				   dst + 8);
		put_unaligned_le32(c9 >> 6 | c10 << 4 | c11 << 14, dst + 12);
		coef += 12;
		dst += 16;
	}
}

void histb_vdp_cv200_build_zme(void *buffer, u32 src_width, u32 src_height,
			     u32 dst_width, u32 dst_height)
{
	u64 h_percent = div_u64((u64)dst_width * 100, src_width);
	u64 v_percent = div_u64((u64)dst_height * 100, src_height);
	enum histb_zme_ratio h = histb_zme_ratio(h_percent);
	enum histb_zme_ratio v = histb_zme_ratio(v_percent);
	/* Match the BSP: double the truncated luma percentage for 420 -> 422. */
	enum histb_zme_ratio cv = histb_zme_ratio(v_percent * 2);
	u8 *dst = buffer;

	memset(buffer, 0, HISTB_VDP_CV200_ZME_SIZE);
	histb_zme_pack(dst, cv200_coefficients[h].lh8, 12);
	histb_zme_pack(dst + 256, cv200_coefficients[h].c4, 6);
	histb_zme_pack(dst + 512, cv200_coefficients[v].lv6, 9);
	histb_zme_pack(dst + 768, cv200_coefficients[cv].c4, 6);
}

/* GP0 transport follows the selected CV200 CONFIG_GFX_PQ provider:
 * pq_v4_0/mng/3798cv200/pq_mng_gfxzme.c:49-59,201-280 (SPC060).
 * The weights below are generated from linear interpolation, not BSP rows.
 */
void histb_vdp_cv200_build_gp0_bilinear(void *buffer)
{
	u8 *dst = buffer;
	unsigned int phase, channel;

	memset(buffer, 0, HISTB_VDP_CV200_GP0_COEF_SIZE);
	for (phase = 0; phase < 8; phase++) {
		u16 right = phase ? phase * 64 : 1;

		for (channel = 0; channel < 2; channel++) {
			u8 *row = dst + channel * 128 + phase * 16;

			put_unaligned_le16(512 - right, row + 3 * 2);
			put_unaligned_le16(right, row + 4 * 2);
		}
	}
	for (phase = 0; phase < 16; phase++) {
		u16 right = phase ? phase * 32 : 1;

		for (channel = 0; channel < 2; channel++) {
			u8 *row = dst + HISTB_VDP_CV200_GP0_VCOEF_OFFSET +
				  phase * 16 + channel * 8;

			put_unaligned_le16(512 - right, row + 1 * 2);
			put_unaligned_le16(right, row + 2 * 2);
		}
	}
}
