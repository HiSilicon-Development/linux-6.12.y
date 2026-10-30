// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hi3798MV310 HiVEDUV500R001 hardware backend.
 * Recovered from the selected BSP venc_v2.0/32bit/3798mv310/drv_venc_hal.S.
 * Offsets below identify the BSP's private tuning object, not CV200 MMIO.
 * DMA addresses remain 32-bit; CPU pointers are native-width on ARM64.
 */
#include <linux/build_bug.h>
#include <linux/slab.h>
#include <linux/string.h>
#include "histb-venc-v500.h"

static_assert(sizeof(struct histb_v500_config) == 0x2a4);
static const u32 v500_defaults[0x45b] = {
	[0x2] = 0x3,
	[0x3] = 0x500,
	[0x4] = 0x2d0,
	[0x7] = 0x2800,
	[0xb] = 0x3,
	[0xe] = 0x1,
	[0x82] = 0x1c,
	[0x83] = 0x10,
	[0x84] = 0x30,
	[0xaa] = 0x1,
	[0xab] = 0x1,
	[0xac] = 0x3,
	[0xad] = 0x40,
	[0xae] = 0x4,
	[0xaf] = 0x10,
	[0xb0] = 0x4,
	[0xb1] = 0x10,
	[0xb9] = 0x10,
	[0xba] = 0x4,
	[0xbb] = 0x1,
	[0xbc] = 0x100,
	[0xbf] = 0x2,
	[0xc3] = 0x4,
	[0xc4] = 0x33,
	[0xca] = 0x7,
	[0xcb] = 0x7,
	[0xcc] = 0x7,
	[0xcd] = 0x9,
	[0xce] = 0xb,
	[0xcf] = 0xe,
	[0xd0] = 0x12,
	[0xd1] = 0x19,
	[0xd2] = 0xff,
	[0xd3] = 0xff,
	[0xd4] = 0xff,
	[0xd5] = 0x1,
	[0xd6] = 0x1,
	[0xd7] = 0x1,
	[0xd8] = 0x2,
	[0xd9] = 0x2,
	[0xda] = 0x3,
	[0xdb] = 0x3,
	[0xdc] = 0x4,
	[0xdd] = 0x5,
	[0xde] = 0x7,
	[0xdf] = 0x9,
	[0xe0] = 0xb,
	[0xe1] = 0xe,
	[0xe2] = 0x11,
	[0xe3] = 0x16,
	[0xe4] = 0x1b,
	[0xe5] = 0x22,
	[0xe6] = 0x2b,
	[0xe7] = 0x36,
	[0xe8] = 0x45,
	[0xe9] = 0x56,
	[0xea] = 0x6d,
	[0xeb] = 0x89,
	[0xec] = 0xad,
	[0xed] = 0xda,
	[0xee] = 0x112,
	[0xef] = 0x159,
	[0xf0] = 0x1b3,
	[0xf1] = 0x224,
	[0xf2] = 0x2b3,
	[0xf3] = 0x366,
	[0xf4] = 0x449,
	[0xf5] = 0x566,
	[0xf6] = 0x6cd,
	[0xf7] = 0x891,
	[0xf8] = 0xacb,
	[0xf9] = 0xd9a,
	[0xfa] = 0xfff,
	[0xfb] = 0xfff,
	[0xfc] = 0xfff,
	[0xfd] = 0x1,
	[0xfe] = 0x1,
	[0xff] = 0x1,
	[0x100] = 0x1,
	[0x101] = 0x2,
	[0x102] = 0x2,
	[0x103] = 0x2,
	[0x104] = 0x2,
	[0x105] = 0x3,
	[0x106] = 0x3,
	[0x107] = 0x3,
	[0x108] = 0x4,
	[0x109] = 0x4,
	[0x10a] = 0x4,
	[0x10b] = 0x5,
	[0x10c] = 0x6,
	[0x10d] = 0x6,
	[0x10e] = 0x7,
	[0x10f] = 0x8,
	[0x110] = 0x9,
	[0x111] = 0xa,
	[0x112] = 0xb,
	[0x113] = 0xd,
	[0x114] = 0xe,
	[0x115] = 0x10,
	[0x116] = 0x12,
	[0x117] = 0x14,
	[0x118] = 0x17,
	[0x119] = 0x19,
	[0x11a] = 0x1d,
	[0x11b] = 0x20,
	[0x11c] = 0x24,
	[0x11d] = 0x28,
	[0x11e] = 0x2d,
	[0x11f] = 0x33,
	[0x120] = 0x39,
	[0x121] = 0x40,
	[0x122] = 0x48,
	[0x123] = 0x51,
	[0x124] = 0x5b,
	[0x126] = 0x3,
	[0x127] = 0x19,
	[0x128] = 0x80,
	[0x129] = 0x3,
	[0x12a] = 0x8,
	[0x12b] = 0x17a,
	[0x12c] = 0x40,
	[0x12e] = 0x7,
	[0x14a] = 0x1,
	[0x14b] = 0x4,
	[0x14c] = 0x100,
	[0x14d] = 0x80,
	[0x14e] = 0x4,
	[0x14f] = 0x80,
	[0x150] = 0x3,
	[0x151] = 0x8,
	[0x152] = 0x20,
	[0x153] = 0x8,
	[0x154] = 0x20,
	[0x196] = 0x1,
	[0x1b9] = 0x20,
	[0x1ba] = 0x6,
	[0x1f4] = 0x1,
	[0x201] = 0x1,
	[0x21d] = 0x1,
	[0x21e] = 0x1,
	[0x21f] = 0x1,
	[0x220] = 0x1,
	[0x221] = 0x1,
	[0x222] = 0x1,
	[0x223] = 0x1,
	[0x224] = 0x1,
	[0x225] = 0x1,
	[0x226] = 0x1,
	[0x227] = 0x1,
	[0x228] = 0x1,
	[0x229] = 0x1,
	[0x22a] = 0x1,
	[0x22b] = 0x1,
	[0x22c] = 0x1,
	[0x22d] = 0x8,
	[0x22e] = 0x2,
	[0x22f] = 0x3,
	[0x235] = 0x30,
	[0x236] = 0x30,
	[0x237] = 0x30,
	[0x238] = 0x33,
	[0x239] = 0x2c,
	[0x23a] = 0xa,
	[0x23b] = 0x4,
	[0x23c] = 0x8,
	[0x23d] = 0x33,
	[0x240] = 0xa20,
	[0x241] = 0x5,
	[0x244] = 0x4,
	[0x246] = 0x4,
	[0x247] = 0x4,
	[0x248] = 0x4,
	[0x249] = 0x4,
	[0x24a] = 0x5555,
	[0x24b] = 0x5555,
	[0x24c] = 0x32,
	[0x24d] = 0x3c,
	[0x24e] = 0x46,
	[0x24f] = 0x50,
	[0x250] = 0x78,
	[0x251] = 0x8c,
	[0x252] = 0xa0,
	[0x253] = 0xb4,
	[0x254] = 0x32,
	[0x255] = 0x3c,
	[0x256] = 0x46,
	[0x257] = 0x50,
	[0x258] = 0x78,
	[0x259] = 0x8c,
	[0x25a] = 0xa0,
	[0x25b] = 0xb4,
	[0x25f] = 0x1,
	[0x260] = 0x1,
	[0x262] = 0x1,
	[0x271] = 0x4,
	[0x281] = 0xa,
	[0x282] = 0xa,
	[0x284] = 0xa,
	[0x285] = 0xa,
	[0x289] = 0x1,
	[0x28a] = 0x14,
	[0x28b] = 0x32,
	[0x28c] = 0x96,
	[0x28d] = 0xc,
	[0x28e] = 0x8,
	[0x28f] = 0x4,
	[0x290] = 0xc,
	[0x291] = 0x8,
	[0x292] = 0x4,
	[0x293] = 0x2,
	[0x294] = 0x4,
	[0x295] = 0x6,
	[0x296] = 0x4,
	[0x297] = 0x1e,
	[0x29a] = 0x83,
	[0x29b] = 0xa0,
	[0x29c] = 0x64,
	[0x29d] = 0x83,
	[0x2aa] = 0xa,
	[0x2ab] = 0x14,
	[0x2ad] = 0x2ff,
	[0x331] = 0x3,
	[0x334] = 0x1,
	[0x336] = 0x2,
	[0x337] = 0x6,
	[0x338] = 0x8,
	[0x339] = 0x8,
	[0x33a] = 0x8,
	[0x33b] = 0x8,
	[0x33c] = 0x10,
	[0x33d] = 0x10,
	[0x33e] = 0x18,
	[0x33f] = 0x18,
	[0x343] = 0x1ff,
	[0x344] = 0xf,
	[0x345] = 0xf,
	[0x346] = 0x5,
	[0x347] = 0x5,
	[0x34c] = 0x3c,
	[0x350] = 0x5,
	[0x355] = 0x8,
	[0x356] = 0x8,
	[0x359] = 0x14,
	[0x365] = 0x1,
	[0x366] = 0x1,
	[0x36e] = 0x1,
	[0x36f] = 0x1,
	[0x370] = 0x1,
	[0x373] = 0x1,
	[0x388] = 0x7,
	[0x389] = 0x7,
	[0x38c] = 0x1,
	[0x38e] = 0x1,
	[0x394] = 0x1,
	[0x398] = 0x1,
	[0x39c] = 0x1,
	[0x39d] = 0x1,
	[0x39e] = 0x3,
	[0x3a0] = 0x1,
	[0x3bb] = 0xffffffff,
	[0x3bc] = 0x7,
	[0x3bd] = 0xffffffff,
	[0x3be] = 0x7,
	[0x3bf] = 0xffffffff,
	[0x3c0] = 0x7,
	[0x3c1] = 0xffffffff,
	[0x3c2] = 0x7,
	[0x3c3] = 0x1,
	[0x3c4] = 0x1,
	[0x3c5] = 0x1,
	[0x3c6] = 0x1,
	[0x3c9] = 0x1,
	[0x3ca] = 0x1,
	[0x3cb] = 0x64,
	[0x3cc] = 0x1f4,
	[0x3cd] = 0x5,
	[0x3ce] = 0x5,
	[0x3cf] = 0xf,
	[0x3d0] = 0xf,
	[0x3d1] = 0x1,
	[0x3d2] = 0x1,
	[0x3d6] = 0x3,
	[0x3dd] = 0x1,
	[0x3e6] = 0xf,
	[0x3e7] = 0xf,
	[0x3ee] = 0x2,
	[0x3f0] = 0x3,
	[0x3f1] = 0x3,
	[0x3f4] = 0x2,
	[0x3fc] = 0x1,
	[0x40f] = 0xff,
	[0x411] = 0xff,
	[0x42e] = 0x2,
	[0x42f] = 0x2,
	[0x432] = 0x1,
	[0x434] = 0x1,
	[0x437] = 0xf,
	[0x438] = 0x9,
	[0x43e] = 0xab,
	[0x43f] = 0x55,
	[0x440] = 0x7,
	[0x441] = 0xf,
	[0x443] = 0x1,
	[0x457] = 0x2,
	[0x458] = 0x1,
	[0x459] = 0x2625a00,
	[0x45a] = 0x3d0900,
};

static const u32 g_h264LambdaSsd[40] = {
	1, 1, 1, 2, 2, 3, 3, 4,
	5, 7, 9, 11, 14, 17, 22, 27,
	34, 43, 54, 69, 86, 109, 137, 173,
	218, 274, 345, 435, 548, 691, 870, 1097,
	1382, 1741, 2193, 2763, 3482, 4095, 4095, 4095,
};

static const u32 g_h264LambdaSad[40] = {
	1, 1, 1, 1, 2, 2, 2, 2,
	3, 3, 3, 4, 4, 4, 5, 6,
	6, 7, 8, 9, 10, 11, 13, 14,
	16, 18, 20, 23, 25, 29, 32, 36,
	40, 45, 51, 57, 64, 72, 81, 91,
};

static const u32 g_h265LambdaSsd[50] = {
	18, 22, 28, 36, 45, 57, 72, 91,
	115, 145, 183, 231, 291, 367, 463, 583,
	735, 926, 1167, 1470, 1853, 2334, 2941, 3706,
	4669, 5883, 7412, 9338, 11766, 14824, 18677, 23532,
	29649, 37355, 47065, 59298, 74711, 94130, 118596, 131071,
	188260, 237192, 262142, 0, 0, 0, 0, 0,
	0, 0,
};

static const u32 g_h265LambdaSad[50] = {
	193, 216, 243, 273, 306, 344, 386, 433,
	487, 546, 613, 688, 773, 867, 974, 1093,
	1227, 1377, 1546, 1735, 1948, 2186, 2454, 2755,
	3092, 3471, 3896, 4373, 4908, 5510, 6184, 6942,
	7792, 8746, 9817, 11020, 12369, 13884, 15584, 17493,
	19636, 22040, 23170, 0, 0, 0, 0, 0,
	0, 0,
};

/* Private BSP tuning offsets are byte offsets into an aligned u32 image.
 * Keep unknown tuning semantics offset-named rather than inventing meanings.
 */
#define V500_TUNE(words, byte_offset) ((words)[(byte_offset) / sizeof(u32)])

static inline u32 v500_tuning_bits(const u32 *tune, unsigned int offset,
                                  u32 mask, unsigned int shift)
{
	return (V500_TUNE(tune, offset) & mask) << shift;
}

/* ARM32 register LSL: counts 32..255 produce zero, not C undefined shifts. */
static u32 v500_lsl(u32 value, u32 count)
{
	return count < 32 ? value << count : 0;
}

static u32 v500_div(u32 numerator, u32 denominator)
{
	if (!denominator)
		return 0;

	return numerator / denominator;
}

void histb_v500_configure(void __iomem *regs, struct histb_v500_config *config,
			 u32 *image, u32 *tune)
{
	u32 value, mode, tuning_flag2, tuning_flag3, tuning_flag4, search_depth, packed_value;
	u32 block_size, blocks_x, blocks_y, index, search_range = 0;
	bool is_planar;

	memcpy(tune, v500_defaults, sizeof(v500_defaults));
	memcpy(tune, config, sizeof(*config));
	memcpy(tune + 0xd5, config->control.protocol ? g_h265LambdaSsd :
		g_h264LambdaSsd, 40 * sizeof(u32));
	memcpy(tune + 0xfd, config->control.protocol ? g_h265LambdaSad :
		g_h264LambdaSad, 40 * sizeof(u32));
	/* Protocol-specific search/transform defaults from CfgReg. */
	if (!config->control.protocol) {
		tune[0xc3] = 4; tune[0xca] = 7; tune[0xcb] = 7;
		tune[0xcc] = 7; tune[0x12c] = 0x20;
		tune[0xcd] = 9; tune[0xce] = 11; tune[0xcf] = 14;
		tune[0xd0] = 18; tune[0xd1] = 25;
	} else {
		tune[0xc3] = 2; tune[0xc9] = 3; tune[0xca] = 3;
		tune[0xcb] = 5; tune[0xcc] = 5; tune[0x12c] = 0x60;
		tune[0xcd] = 10; tune[0xce] = 15; tune[0xcf] = 20;
		tune[0xd0] = 255; tune[0xd1] = 255;
	}
	tune[0xd2] = 255; tune[0xd3] = 255; tune[0xd4] = 255;
	if (config->control.low_delay != 0) {
		if (config->control.width < 0x780) {
			if (config->control.width < 0x2d0) {
				search_depth = 1;
			} else {
				search_depth = 2;
			}
			V500_TUNE(tune, 0x4b4) = search_depth;
		} else {
			V500_TUNE(tune, 0x4b4) = 3;
		}
	}
	packed_value = config->control.profile;
	if (config->control.protocol == 0) {
		V500_TUNE(tune, 0x4c0) = (u32)(packed_value == 0);
		V500_TUNE(tune, 0x4bc) = (u32)(packed_value != 3);
		if (packed_value != 3) {
			search_depth = 5;
		} else {
			search_depth = 7;
		}
		V500_TUNE(tune, 0x4b8) = search_depth;
	} else {
		V500_TUNE(tune, 0x4c0) = 0;
		V500_TUNE(tune, 0x4bc) = 0;
		V500_TUNE(tune, 0x4b8) = 7;
	}
	if ((config->control.width - 0x80U < 0x80) && (config->control.protocol == 0)) {
		config->rc.max_qp = 0x1c;
		config->rc.min_qp = 0x1c;
		config->rc.qp = 0x1c;
		V500_TUNE(tune, 0x30c) = 0;
		config->control.split_enable = 0;
		V500_TUNE(tune, 0x10c8) = 0;
		V500_TUNE(tune, 0xf28) = 0;
		V500_TUNE(tune, 0xf24) = 0;
		V500_TUNE(tune, 0xe28) = 0;
		V500_TUNE(tune, 0xd2c) = 0;
		V500_TUNE(tune, 0x99c) = 0;
		V500_TUNE(tune, 0x10ac) = 0;
	}
	writel(config->image_dma, regs + 4 * (0x46));
	writel(0xcf, regs);
	writel(V500_TUNE(tune, 0xdb4), regs + 4 * (4));
	writel(V500_TUNE(tune, 0xd8c) << 0x1f | v500_tuning_bits(tune, 0xd90, 1, 0x1e) |
		(((V500_TUNE(tune, 0xdbc) - 1U) & 0x1ff)) << 0x10 |
		((V500_TUNE(tune, 0xdc0) - 1U) & 0x1ff), regs + 4 * (5));
	writel(V500_TUNE(tune, 0x1164), regs + 4 * (6));
	writel(v500_tuning_bits(tune, 0x1160, 1, 0x15) | v500_tuning_bits(tune, 0x115c, 3, 0x13)
		| 0x70000 | v500_tuning_bits(tune, 0x10c8, 1, 2) |
		v500_tuning_bits(tune, 0x10e4, 1, 0x17) | v500_tuning_bits(tune, 0x10cc, 1, 3) |
		v500_tuning_bits(tune, 0x10d0, 1, 1) | (V500_TUNE(tune, 0x10d4) & 1), regs + 4 * (0xd));
	writel(v500_tuning_bits(tune, 0x1100, 0xf, 4) | (V500_TUNE(tune, 0x1104) & 0xf), regs + 4 * (0xe));
	writel(0x91b1, regs + 4 * (0x11));
	block_size = config->control.protocol ? 32 : 16;
	blocks_x = v500_div(config->control.width - 1U + block_size, block_size);
	blocks_y = v500_div(config->control.height - 1U + block_size, block_size);
	value = v500_div(config->rc.target_bits, blocks_x * blocks_y);

	writel((value & 0xfffff), regs + 4 * (0x93));
	writel((V500_TUNE(tune, 0xf58) & 0xf), regs + 4 * (0x94));
	packed_value = config->control.protocol;
	value = V500_TUNE(tune, 0xf84) << 0x1f;
	if (packed_value == 2) {
		value = value | 0x4000000;
	} else if (packed_value != 0) {
		value = value | 0xc000000;
	}
	writel(value | v500_tuning_bits(tune, 0xe5c, 1, 0x1e) |
		v500_tuning_bits(tune, 0x4bc, 3, 0x12) | v500_tuning_bits(tune, 0xe4c, 1, 0x15) |
		v500_tuning_bits(tune, 0xe50, 1, 0x11) | (config->rc.intra & 1) << 0x10 |
		v500_tuning_bits(tune, 0x4c0, 1, 0xc) |
		(u32)(config->rc.intra == 0) << 10 | v500_tuning_bits(tune, 0x524, 1, 9)
		| v500_tuning_bits(tune, 0xe3c, 1, 8) | v500_tuning_bits(tune, 0xe70, 1, 7) |
		v500_tuning_bits(tune, 0xe74, 1, 6) | (config->control.low_delay & 1) << 5 |
		(u32)(packed_value != 2) << 2 | (u32)(config->control.ddr_mode != 0) << 4 | 2 |
		(V500_TUNE(tune, 0xda8) & 1), regs + 4 * (7));
	image[0] = v500_tuning_bits(tune, 0xf1c, 0x1ff, 0x10) | (V500_TUNE(tune, 0xf20) & 0x1ff);
	image[2] = (config->control.split_enable & 1) << 0x11 | 0x10000 |
		(config->control.split_size & 0xffff);
	image[3] = (config->rc.qp & 0x3f) << 10 | v500_tuning_bits(tune, 0xdc4, 0x1f, 5)
		| (V500_TUNE(tune, 0xdc8) & 0x1f) | v500_tuning_bits(tune, 0x99c, 1, 0x12) |
		v500_tuning_bits(tune, 0x98c, 1, 0x13);
	image[4] = v500_tuning_bits(tune, 0xe64, 3, 8) | v500_tuning_bits(tune, 0xe68, 0xf, 4) |
		(V500_TUNE(tune, 0xe6c) & 0xf);
	image[1] = (((config->control.height - 1U) & 0x1fff)) << 0x10 |
		((config->control.width - 1U) & 0x1fff);
	if (V500_TUNE(tune, 0xff0) == 1) {
		value = 3;
	} else {
		value = (V500_TUNE(tune, 0xe78) & 3);
	}
	image[5] = value;
	image[6] = (V500_TUNE(tune, 0xe80) & 1);
	image[7] = v500_tuning_bits(tune, 0xe20, 0xf, 0xc) | v500_tuning_bits(tune, 0xe24, 0xf, 8) |
		v500_tuning_bits(tune, 0xe30, 1, 4) | (V500_TUNE(tune, 0x4b8) & 7) |
		v500_tuning_bits(tune, 0xe34, 1, 5);
	image[8] = v500_tuning_bits(tune, 0xe00, 1, 0xc) | v500_tuning_bits(tune, 0xe08, 3, 4) | 3;
	image[9] = v500_tuning_bits(tune, 0xf44, 1, 4) | v500_tuning_bits(tune, 0xe28, 1, 2) |
		v500_tuning_bits(tune, 0xe2c, 1, 3) | v500_tuning_bits(tune, 0xf24, 1, 1) |
		(V500_TUNE(tune, 0xf28) & 1);
	image[10] = V500_TUNE(tune, 0xf2c) << 0x10 | (V500_TUNE(tune, 0xf30) & 0xffff);
	image[0xb] = V500_TUNE(tune, 0xf34) << 0x18 | v500_tuning_bits(tune, 0xf38, 0xff, 0x10) |
		v500_tuning_bits(tune, 0xf3c, 0xff, 8) | (V500_TUNE(tune, 0xf40) & 0xff);
	image[0xc] = v500_tuning_bits(tune, 0x308, 1, 1) | v500_tuning_bits(tune, 0x10bc, 0xff, 0x10)
		| V500_TUNE(tune, 0x10b8) << 0x18;
	value = V500_TUNE(tune, 0x2a8);
	mode = V500_TUNE(tune, 0x2ac);
	tuning_flag2 = V500_TUNE(tune, 0x528);
	tuning_flag3 = V500_TUNE(tune, 0xf48);
	tuning_flag4 = V500_TUNE(tune, 0xf4c);
	image[0xe] = 2;
	image[0xd] = ((value & 1)) << 2 | ((mode & 1)) << 1 | (tuning_flag2 & 1) | ((tuning_flag3 & 1)) << 3 |
		((tuning_flag4 & 1)) << 4;
	image[0xf] = (V500_TUNE(tune, 0x97c) & 1) | v500_tuning_bits(tune, 0x980, 1, 1) |
		v500_tuning_bits(tune, 0x304, 1, 2) | v500_tuning_bits(tune, 0x998, 1, 4) |
		v500_tuning_bits(tune, 0x978, 1, 9) | v500_tuning_bits(tune, 0x300, 1, 8) |
		v500_tuning_bits(tune, 0x9ac, 1, 3) | v500_tuning_bits(tune, 0x9e4, 1, 0xf) |
		v500_tuning_bits(tune, 0x970, 1, 0xe) | v500_tuning_bits(tune, 0x974, 1, 0xd) |
		v500_tuning_bits(tune, 0x9e0, 1, 0xc) | v500_tuning_bits(tune, 0x9dc, 1, 0xb) |
		v500_tuning_bits(tune, 0x9d8, 1, 10) | v500_tuning_bits(tune, 0x9a0, 1, 0x12);
	image[0x10] = v500_tuning_bits(tune, 0x10ac, 1, 8) | (V500_TUNE(tune, 0x10b0) & 0xff);
	image[0x11] = (V500_TUNE(tune, 0x10b4) & 1);
	image[0x12] =
		(V500_TUNE(tune, 0x1158) & 1) | v500_tuning_bits(tune, 0x1154, 1, 1) |
		v500_tuning_bits(tune, 0x1150, 1, 2) | v500_tuning_bits(tune, 0x114c, 1, 3) |
		v500_tuning_bits(tune, 0x1148, 1, 4) | v500_tuning_bits(tune, 0x1144, 1, 5) |
		v500_tuning_bits(tune, 0x1140, 1, 6) | v500_tuning_bits(tune, 0x1118, 1, 7);
	image[0x13] =
		(V500_TUNE(tune, 0x113c) & 1) | v500_tuning_bits(tune, 0x1138, 1, 1) |
		v500_tuning_bits(tune, 0x1134, 1, 2) | v500_tuning_bits(tune, 0x1130, 1, 3) |
		v500_tuning_bits(tune, 0x1110, 1, 4) | v500_tuning_bits(tune, 0x1114, 1, 5) |
		v500_tuning_bits(tune, 0x112c, 1, 6) | v500_tuning_bits(tune, 0x1128, 1, 7) |
		v500_tuning_bits(tune, 0x111c, 1, 8) | v500_tuning_bits(tune, 0x1124, 1, 9) |
		v500_tuning_bits(tune, 0x1120, 1, 10);
	if (config->control.protocol == 2) {
		search_range = 0x13001b;
	} else if (config->control.protocol == 0) {
		search_range = 0xf001f;
	}
	if (config->control.ddr_mode == 0) {
		writel(search_range, regs + 4 * (0x1f));
	} else {
		image[0x14] = search_range;
	}
	writel(0x10001, regs + 4 * (0x20));
	image[0x16] = config->buffer.pme_info[config->buffer.pme_index];
	value = (config->buffer.pme_index + 2) % 3;
	image[0x17] = config->buffer.pme_info[value];
	value = (config->buffer.pme_index + 1) % 3;
	image[0x18] = config->buffer.pme_info[value];
	image[0x19] = config->buffer.tunnel;
	image[0x1a] = config->buffer.source_y;
	image[0x1b] = config->buffer.source_c;
	image[0x1c] = config->buffer.source_v;
	image[0x1f] = config->buffer.source_stride_y << 0x10 | (config->buffer.source_stride_c & 0xffff);
	image[0x20] = config->buffer.recon_y[config->buffer.recon_index];
	image[0x21] = config->buffer.recon_c[config->buffer.recon_index];
	image[0x22] = (config->buffer.recon_stride_y & 0xffff) | config->buffer.recon_stride_c << 0x10;
	image[0x26] = config->buffer.recon_y[!config->buffer.recon_index];
	image[0x27] = config->buffer.recon_c[!config->buffer.recon_index];
	image[0x28] = (config->buffer.recon_stride_y << 16) |
		(config->buffer.recon_stride_c & 0xffff);
	image[0x2c] = config->buffer.pme[!config->buffer.recon_index];
	image[0x34] = config->buffer.pme[config->buffer.recon_index];
	image[0x36] = config->buffer.tmv[config->buffer.recon_index];
	image[0x37] = config->buffer.tmv[!config->buffer.recon_index];
	image[0x38] = config->buffer.stream[0];
	image[0x39] = config->buffer.stream_wp;
	image[0x3a] = config->buffer.stream_rp;
	mode = config->buffer.store;
	value = v500_tuning_bits(tune, 0x1008, 1, 0xe);
	if (mode == 0) {
		value = value | (u32)(config->buffer.package == 0) << 0x10;
	} else {
		is_planar = mode == 2;
		if (is_planar) {
			mode = 3;
		}
		value = value | (config->buffer.package & 0xff) << 0x10;
		if (!is_planar) {
			if (mode == 1) {
				mode = 6;
			} else {
				mode = 0;
			}
		}
	}
	value = value | ((mode & 0xf)) << 6;
	if (config->control.ddr_mode == 0) {
		writel(value, regs + 4 * (0x47));
	} else {
		image[0x3b] = value;
	}
	image[0x40] = 0;
	image[0x53] =
		v500_tuning_bits(tune, 0x2b0, 0x1f, 0x10) | (V500_TUNE(tune, 0x534) & 0xff) |
		v500_tuning_bits(tune, 0x6ac, 0xf, 0x18);
	image[0x54] =
		(V500_TUNE(tune, 0x7b4) & 7) | v500_tuning_bits(tune, 0x7b8, 0x7f, 3) |
		v500_tuning_bits(tune, 0x7bc, 7, 10) | v500_tuning_bits(tune, 0x7c4, 0xf, 0x12) |
		v500_tuning_bits(tune, 0x7cc, 0xf, 0x1a);
	image[0x55] =
		(V500_TUNE(tune, 0x70c) & 1) | v500_tuning_bits(tune, 0x710, 7, 1) |
		v500_tuning_bits(tune, 0x71c, 0x3f, 0xf) | v500_tuning_bits(tune, 0x720, 0x1f, 0x15);
	image[0x56] =
		v500_tuning_bits(tune, 0x540, 0x1f, 0x10) | (V500_TUNE(tune, 0x2b4) & 0xff) |
		v500_tuning_bits(tune, 0x6b4, 0xf, 0x18);
	image[0x57] =
		(V500_TUNE(tune, 0x7d0) & 7) | v500_tuning_bits(tune, 0x7d4, 0x7f, 3) |
		v500_tuning_bits(tune, 0x7d8, 7, 10) | v500_tuning_bits(tune, 0x7e0, 0xf, 0x12) |
		v500_tuning_bits(tune, 0x7e8, 0xf, 0x1a);
	image[0x58] =
		(V500_TUNE(tune, 0x724) & 1) | v500_tuning_bits(tune, 0x728, 7, 1) |
		v500_tuning_bits(tune, 0x734, 0x3f, 0xf) | v500_tuning_bits(tune, 0x738, 0x1f, 0x15);
	image[0x59] =
		v500_tuning_bits(tune, 0x2c0, 0x1f, 0x10) | (V500_TUNE(tune, 0x2c4) & 0xff) |
		v500_tuning_bits(tune, 0x6c4, 0xf, 0x18);
	image[0x5a] =
		(V500_TUNE(tune, 0x804) & 7) | v500_tuning_bits(tune, 0x808, 0x7f, 3) |
		v500_tuning_bits(tune, 0x80c, 7, 10) | v500_tuning_bits(tune, 0x814, 0xf, 0x12) |
		v500_tuning_bits(tune, 0x81c, 0xf, 0x1a);
	image[0x5b] =
		(V500_TUNE(tune, 0x754) & 1) | v500_tuning_bits(tune, 0x758, 7, 1) |
		v500_tuning_bits(tune, 0x764, 0x3f, 0xf) | v500_tuning_bits(tune, 0x768, 0x1f, 0x15);
	image[0x5c] =
		(V500_TUNE(tune, 0x2e4) & 0xff) | v500_tuning_bits(tune, 0x2e8, 0x1f, 0x10) |
		v500_tuning_bits(tune, 0x6cc, 0xf, 0x18);
	image[0x5d] =
		(V500_TUNE(tune, 0x658) & 7) | v500_tuning_bits(tune, 0x65c, 0x7f, 3) |
		v500_tuning_bits(tune, 0x660, 7, 10) | v500_tuning_bits(tune, 0x668, 0xf, 0x12) |
		v500_tuning_bits(tune, 0x670, 0xf, 0x1a);
	image[0x5e] =
		(V500_TUNE(tune, 0x76c) & 1) | v500_tuning_bits(tune, 0x770, 7, 1) |
		v500_tuning_bits(tune, 0x77c, 0x3f, 0xf) | v500_tuning_bits(tune, 0x780, 0x1f, 0x15);
	image[0x5f] =
		v500_tuning_bits(tune, 0x2b8, 0x1f, 0x10) | (V500_TUNE(tune, 0x2bc) & 0xff) |
		v500_tuning_bits(tune, 0x6bc, 0xf, 0x18);
	image[0x60] =
		(V500_TUNE(tune, 0x2ec) & 7) | v500_tuning_bits(tune, 0x7ec, 0x7f, 3) |
		v500_tuning_bits(tune, 0x7f0, 7, 10) | v500_tuning_bits(tune, 0x7f8, 0xf, 0x12) |
		v500_tuning_bits(tune, 0x800, 0xf, 0x1a);
	image[0x61] =
		(V500_TUNE(tune, 0x73c) & 1) | v500_tuning_bits(tune, 0x740, 7, 1) |
		v500_tuning_bits(tune, 0x74c, 0x3f, 0xf) | v500_tuning_bits(tune, 0x750, 0x1f, 0x15);
	image[0x62] =
		(V500_TUNE(tune, 0x6f4) & 0xff) | v500_tuning_bits(tune, 0x6f8, 0x1f, 0x10) |
		v500_tuning_bits(tune, 0x6d4, 0xf, 0x18);
	image[99] = (V500_TUNE(tune, 0x674) & 7) | v500_tuning_bits(tune, 0x678, 0x7f, 3) |
		v500_tuning_bits(tune, 0x67c, 7, 10) | v500_tuning_bits(tune, 0x684, 0xf, 0x12) |
		v500_tuning_bits(tune, 0x68c, 0xf, 0x1a);
	image[100] = (V500_TUNE(tune, 0x784) & 1) | v500_tuning_bits(tune, 0x788, 7, 1) |
		v500_tuning_bits(tune, 0x794, 0x3f, 0xf) |
		v500_tuning_bits(tune, 0x798, 0x1f, 0x15);
	image[0x65] =
		(V500_TUNE(tune, 0x704) & 0xff) | v500_tuning_bits(tune, 0x708, 0x1f, 0x10) |
		v500_tuning_bits(tune, 0x6dc, 0xf, 0x18);
	image[0x66] =
		(V500_TUNE(tune, 0x690) & 7) | v500_tuning_bits(tune, 0x694, 0x7f, 3) |
		v500_tuning_bits(tune, 0x698, 7, 10) | v500_tuning_bits(tune, 0x6a0, 0xf, 0x12) |
		v500_tuning_bits(tune, 0x6a8, 0xf, 0x1a);
	image[0x67] =
		(V500_TUNE(tune, 0x79c) & 1) | v500_tuning_bits(tune, 0x7a0, 7, 1) |
		v500_tuning_bits(tune, 0x7ac, 0x3f, 0xf) | v500_tuning_bits(tune, 0x7b0, 0x1f, 0x15);
	image[0x68] =
		(V500_TUNE(tune, 0x560) & 0xf) | v500_tuning_bits(tune, 0x55c, 0xf, 4) |
		v500_tuning_bits(tune, 0x558, 0xf, 8) | v500_tuning_bits(tune, 0x56c, 0xf, 0x10) |
		v500_tuning_bits(tune, 0x568, 0xf, 0x14) | v500_tuning_bits(tune, 0x2c8, 0xf, 0x18);
	image[0x69] =
		(V500_TUNE(tune, 0x5b0) & 0xf) | v500_tuning_bits(tune, 0x5ac, 0xf, 4) |
		v500_tuning_bits(tune, 0x2d4, 0xf, 8) | v500_tuning_bits(tune, 0x5c0, 0xf, 0x10) |
		v500_tuning_bits(tune, 0x5bc, 0xf, 0x14) | v500_tuning_bits(tune, 0x5b8, 0xf, 0x18);
	image[0x6a] =
		(V500_TUNE(tune, 0x604) & 0xf) | v500_tuning_bits(tune, 0x600, 0xf, 4) |
		v500_tuning_bits(tune, 0x2e0, 0xf, 8) | v500_tuning_bits(tune, 0x614, 0xf, 0x10) |
		v500_tuning_bits(tune, 0x610, 0xf, 0x14) | v500_tuning_bits(tune, 0x60c, 0xf, 0x18);
	image[0x6b] =
		v500_tuning_bits(tune, 0x574, 0xf, 8) | v500_tuning_bits(tune, 0x578, 0xf, 4) |
		(V500_TUNE(tune, 0x57c) & 0xf) | v500_tuning_bits(tune, 0x584, 0xf, 0x18) |
		v500_tuning_bits(tune, 0x2cc, 0xf, 0x14) | v500_tuning_bits(tune, 0x588, 0xf, 0x10);
	image[0x6c] =
		v500_tuning_bits(tune, 0x5c8, 0xf, 8) | v500_tuning_bits(tune, 0x2d8, 0xf, 4) |
		(V500_TUNE(tune, 0x5cc) & 0xf) | v500_tuning_bits(tune, 0x5d4, 0xf, 0x18) |
		v500_tuning_bits(tune, 0x5d8, 0xf, 0x14) | v500_tuning_bits(tune, 0x5dc, 0xf, 0x10);
	image[0x6d] =
		v500_tuning_bits(tune, 0x61c, 0xf, 8) | v500_tuning_bits(tune, 0x620, 0xf, 4) |
		(V500_TUNE(tune, 0x624) & 0xf) | v500_tuning_bits(tune, 0x62c, 0xf, 0x18) |
		v500_tuning_bits(tune, 0x630, 0xf, 0x14) | v500_tuning_bits(tune, 0x634, 0xf, 0x10);
	image[0x6e] =
		v500_tuning_bits(tune, 0x590, 0xf, 8) | v500_tuning_bits(tune, 0x594, 0xf, 4) |
		(V500_TUNE(tune, 0x598) & 0xf) | v500_tuning_bits(tune, 0x5a0, 0xf, 0x18) |
		v500_tuning_bits(tune, 0x2d0, 0xf, 0x14) | v500_tuning_bits(tune, 0x5a4, 0xf, 0x10);
	image[0x6f] =
		v500_tuning_bits(tune, 0x5e4, 0xf, 8) | v500_tuning_bits(tune, 0x2dc, 0xf, 4) |
		(V500_TUNE(tune, 0x5e8) & 0xf) | v500_tuning_bits(tune, 0x5f0, 0xf, 0x18) |
		v500_tuning_bits(tune, 0x5f4, 0xf, 0x14) | v500_tuning_bits(tune, 0x5f8, 0xf, 0x10);
	image[0x70] =
		v500_tuning_bits(tune, 0x63c, 0xf, 8) | v500_tuning_bits(tune, 0x640, 0xf, 4) |
		(V500_TUNE(tune, 0x644) & 0xf) | v500_tuning_bits(tune, 0x64c, 0xf, 0x18) |
		v500_tuning_bits(tune, 0x650, 0xf, 0x14) | v500_tuning_bits(tune, 0x654, 0xf, 0x10);
	image[0x71] =
		(V500_TUNE(tune, 0x820) & 0xf) | v500_tuning_bits(tune, 0x824, 0xf, 4) |
		v500_tuning_bits(tune, 0x828, 0xf, 8) | v500_tuning_bits(tune, 0x82c, 0xf, 0xc) |
		v500_tuning_bits(tune, 0x830, 0xf, 0x10) | v500_tuning_bits(tune, 0x834, 0xf, 0x14);
	image[0x72] =
		(V500_TUNE(tune, 0x2f0) & 0xfff) | v500_tuning_bits(tune, 0x2f4, 0xfff, 0xc) |
		V500_TUNE(tune, 0x838) << 0x18;
	image[0x73] =
		(V500_TUNE(tune, 0x83c) & 0xfff) | v500_tuning_bits(tune, 0x840, 0xfff, 0xc) |
		V500_TUNE(tune, 0x844) << 0x18;
	image[0x74] =
		(V500_TUNE(tune, 0x2f8) & 0xfff) | v500_tuning_bits(tune, 0x848, 0xfff, 0xc) |
		V500_TUNE(tune, 0x84c) << 0x18;
	image[0x75] =
		(V500_TUNE(tune, 0x850) & 0xff) | v500_tuning_bits(tune, 0x854, 0xff, 8) |
		v500_tuning_bits(tune, 0x858, 0xff, 0x10);
	image[0x78] =
		v500_tuning_bits(tune, 0x1034, 1, 0xe) | v500_tuning_bits(tune, 0x1038, 1, 0xf) |
		(config->control.low_delay & 1) << 0x1c | v500_tuning_bits(tune, 0x4b4, 0xff, 0x14);
	image[0x79] =
		(V500_TUNE(tune, 0x1048) & 0xff) | v500_tuning_bits(tune, 0x1044, 0xff, 8) |
		v500_tuning_bits(tune, 0x1040, 0xff, 0x10) | V500_TUNE(tune, 0x103c) << 0x18;
	image[0x7a] = (V500_TUNE(tune, 0xa24) & 1);
	image[0x7b] = (V500_TUNE(tune, 0xa28) & 0xffff);
	image[0x7c] = (V500_TUNE(tune, 0xa2c) & 0xffff);
	image[0x7d] = (V500_TUNE(tune, 0xa30) & 0xffff);
	image[0x7e] = v500_tuning_bits(tune, 0xa60, 1, 0x10) | (V500_TUNE(tune, 0xa64) & 0xffff);
	image[0x7f] =
		v500_tuning_bits(tune, 0xa34, 0x1ff, 0x12) | v500_tuning_bits(tune, 0xa38, 0x1ff, 9) |
		(V500_TUNE(tune, 0xa3c) & 0x1ff);
	image[0x80] =
		v500_tuning_bits(tune, 0xa40, 0xff, 0x10) | v500_tuning_bits(tune, 0xa44, 0xff, 8) |
		(V500_TUNE(tune, 0xa48) & 0xff);
	image[0x81] =
		v500_tuning_bits(tune, 0xa4c, 0xf, 8) | v500_tuning_bits(tune, 0xa50, 0xf, 4) |
		(V500_TUNE(tune, 0xa54) & 0xf) | v500_tuning_bits(tune, 0xa5c, 0xff, 0xc) |
		v500_tuning_bits(tune, 0xa58, 0xff, 0x14);
	image[0x83] =
		(V500_TUNE(tune, 0xd1c) & 0xff) | v500_tuning_bits(tune, 0xd18, 0xff, 8) |
		v500_tuning_bits(tune, 0xd14, 0xff, 0x10) | V500_TUNE(tune, 0xd10) << 0x18;
	image[0x84] =
		V500_TUNE(tune, 0xa68) << 0x18 | v500_tuning_bits(tune, 0xa6c, 0xff, 0x10) |
		v500_tuning_bits(tune, 0xa70, 0xff, 8) | (V500_TUNE(tune, 0xa74) & 0xff);
	image[0x85] = (V500_TUNE(tune, 0xaa8) & 0xff) | v500_tuning_bits(tune, 0xd38, 1, 0x11);
	image[0x86] = (V500_TUNE(tune, 0xab4) & 0xffff) | V500_TUNE(tune, 0xab8) << 0x10;
	image[0x87] =
		v500_tuning_bits(tune, 0x494, 0x1ff, 0x16) | v500_tuning_bits(tune, 0x498, 0x1f, 0x11) |
		v500_tuning_bits(tune, 0x49c, 0xff, 9) | (V500_TUNE(tune, 0x4b0) & 0x1ff);
	image[0x88] = v500_tuning_bits(tune, 0x4ac, 0x3fff, 8) | (V500_TUNE(tune, 0xcc4) & 0xff);
	image[0x89] =
		v500_tuning_bits(tune, 0x4a0, 0xff, 0x10) | v500_tuning_bits(tune, 0x4a4, 0xff, 8) |
		(V500_TUNE(tune, 0x4a8) & 0xff);
	image[0x8a] =
		v500_tuning_bits(tune, 0xccc, 1, 0x12) | v500_tuning_bits(tune, 0xcd0, 3, 0xc) |
		v500_tuning_bits(tune, 0xcd8, 0xf, 4) | (V500_TUNE(tune, 0xcdc) & 0xf);
	image[0x8b] = (V500_TUNE(tune, 0xce0) & 0x3ff) | v500_tuning_bits(tune, 0xce4, 0x1ff, 0x10);
	image[0x8c] = (V500_TUNE(tune, 0xce8) & 0x3ff) | v500_tuning_bits(tune, 0xcec, 0x1ff, 0x10);
	image[0x8d] = (V500_TUNE(tune, 0xcf0) & 0x3ff) | v500_tuning_bits(tune, 0xcf4, 0x1ff, 0x10);
	image[0x8e] = (V500_TUNE(tune, 0xcf8) & 0x3ff) | v500_tuning_bits(tune, 0xcfc, 0x1ff, 0x10);
	image[0x8f] = (V500_TUNE(tune, 0xd00) & 0xffff);
	image[0x90] = (V500_TUNE(tune, 0xd20) & 0x1fff) | v500_tuning_bits(tune, 0xd24, 1, 0xd);
	image[0x91] = (V500_TUNE(tune, 0xd28) & 0x3ffff);
	image[0x92] = (V500_TUNE(tune, 0xd2c) & 0x3ffff);
	image[0x93] =
		v500_tuning_bits(tune, 0xd44, 0x7f, 0x10) | v500_tuning_bits(tune, 0xd48, 0x7f, 0x17);
	image[0x94] =
		v500_tuning_bits(tune, 0xd34, 0xff, 0xc) | v500_tuning_bits(tune, 0xd4c, 0xf, 8) |
		(V500_TUNE(tune, 0xd50) & 0xff) | v500_tuning_bits(tune, 0xd54, 0xff, 0x14) |
		V500_TUNE(tune, 0xd58) << 0x1c;
	image[0x95] =
		(V500_TUNE(tune, 0xd3c) & 0xff) | v500_tuning_bits(tune, 0xd40, 0xff, 8) |
		v500_tuning_bits(tune, 0xd30, 0xff, 0x10);
	image[0x96] =
		(V500_TUNE(tune, 0xd5c) & 0xff) | v500_tuning_bits(tune, 0xd60, 0xff, 8) |
		V500_TUNE(tune, 0xd64) << 0x10;
	image[0x97] =
		V500_TUNE(tune, 0xa78) << 0x18 | v500_tuning_bits(tune, 0xa7c, 0xff, 0x10) |
		v500_tuning_bits(tune, 0xa80, 0xff, 8) | (V500_TUNE(tune, 0xa84) & 0xff);
	image[0x98] =
		V500_TUNE(tune, 0xa88) << 0x18 | v500_tuning_bits(tune, 0xa8c, 0xff, 0x10) |
		v500_tuning_bits(tune, 0xa90, 0xff, 8) | (V500_TUNE(tune, 0xa94) & 0xff);
	image[0x99] =
		V500_TUNE(tune, 0xa98) << 0x1c | v500_tuning_bits(tune, 0xa9c, 0x1f, 0x17) |
		v500_tuning_bits(tune, 0xaa0, 0x1ff, 0xe) | (V500_TUNE(tune, 0xaa4) & 0x3fff);
	image[0x9c] =
		(config->rc.max_qp & 0x3f) << 8 | v500_tuning_bits(tune, 0xdcc, 1, 0x10) |
		(config->rc.min_qp & 0x3f);
	if (config->control.protocol == 0) {
		value = config->control.height >> 4;
	} else {
		value = config->control.height >> 5;
	}
	value = v500_div(config->rc.target_bits,value);
	image[0x9d] = ((value & 0x1ffffff)) << 6 | 2;
	image[0x9e] = (V500_TUNE(tune, 0x900) & 0xffff);
	image[0x9f] =
		v500_tuning_bits(tune, 0x8e4, 0x3f, 8) | v500_tuning_bits(tune, 0x8cc, 0xf, 0x10) |
		(V500_TUNE(tune, 0x8e8) & 0x3f);
	image[0xa0] =
		v500_tuning_bits(tune, 0x8d8, 0x3f, 8) | v500_tuning_bits(tune, 0x8c4, 0xf, 0x10) |
		(V500_TUNE(tune, 0xa14) & 0x3f);
	image[0xa1] =
		v500_tuning_bits(tune, 0x8d8, 0x3f, 8) | v500_tuning_bits(tune, 0x8c8, 0xf, 0x10) |
		(V500_TUNE(tune, 0xa08) & 0x3f);
	image[0xa2] =
		v500_tuning_bits(tune, 0x8d4, 0x3f, 8) | v500_tuning_bits(tune, 0x8c0, 0xf, 0x10) |
		(V500_TUNE(tune, 0xa10) & 0x3f);
	image[0xa3] =
		v500_tuning_bits(tune, 0x310, 0x3f, 8) | v500_tuning_bits(tune, 0x2fc, 0xf, 0x10) |
		(V500_TUNE(tune, 0xa04) & 0x3f);
	image[0xa4] =
		v500_tuning_bits(tune, 0x8d0, 0xf, 0x10) | v500_tuning_bits(tune, 0x8e0, 0x3f, 8) |
		(V500_TUNE(tune, 0xa0c) & 0x3f);
	image[0xa5] =
		v500_tuning_bits(tune, 0x8ec, 0xf, 0x10) | v500_tuning_bits(tune, 0x8f0, 0xf, 0x14) |
		v500_tuning_bits(tune, 0x8f4, 0x3f, 8) | (V500_TUNE(tune, 0xa18) & 0x3f);
	image[0xa6] =
		V500_TUNE(tune, 0x320) << 0x18 | v500_tuning_bits(tune, 0x31c, 0xff, 0x10) |
		v500_tuning_bits(tune, 0x318, 0xff, 8) | (V500_TUNE(tune, 0x314) & 0xff);
	image[0xa7] =
		V500_TUNE(tune, 0x330) << 0x18 | v500_tuning_bits(tune, 0x32c, 0xff, 0x10) |
		v500_tuning_bits(tune, 0x328, 0xff, 8) | (V500_TUNE(tune, 0x324) & 0xff);
	image[0xa8] =
		V500_TUNE(tune, 0x340) << 0x18 | v500_tuning_bits(tune, 0x33c, 0xff, 0x10) |
		v500_tuning_bits(tune, 0x338, 0xff, 8) | (V500_TUNE(tune, 0x334) & 0xff);
	image[0xa9] =
		V500_TUNE(tune, 0x350) << 0x18 | v500_tuning_bits(tune, 0x34c, 0xff, 0x10) |
		v500_tuning_bits(tune, 0x348, 0xff, 8) | (V500_TUNE(tune, 0x344) & 0xff);
	image[0xaa] =
		(V500_TUNE(tune, 0x874) & 3) | v500_tuning_bits(tune, 0x878, 3, 2) |
		v500_tuning_bits(tune, 0x87c, 3, 4) | v500_tuning_bits(tune, 0x880, 3, 6) |
		v500_tuning_bits(tune, 0x884, 3, 8) | v500_tuning_bits(tune, 0x888, 3, 10) |
		v500_tuning_bits(tune, 0x88c, 3, 0xc) | v500_tuning_bits(tune, 0x890, 3, 0xe) |
		v500_tuning_bits(tune, 0x894, 3, 0x10) | v500_tuning_bits(tune, 0x898, 3, 0x12) |
		v500_tuning_bits(tune, 0x89c, 3, 0x14) | v500_tuning_bits(tune, 0x8a0, 3, 0x16) |
		v500_tuning_bits(tune, 0x8a4, 3, 0x18) | v500_tuning_bits(tune, 0x8a8, 3, 0x1a) |
		v500_tuning_bits(tune, 0x8ac, 3, 0x1c) | V500_TUNE(tune, 0x8b0) << 0x1e;
	image[0xab] = (V500_TUNE(tune, 0x8b4) & 0x1f);
	image[0xac] = (V500_TUNE(tune, 0x8f8) & 0xf) | v500_tuning_bits(tune, 0x8fc, 0xf, 4);
	if (config->control.protocol == 2 && config->rc.avbr &&
	!config->rc.intra) {
		for (index = 0; index < 40; index++)
		writel(g_h265LambdaSsd[index + 3] & 0xfffff,
			regs + 4 * (index + 0x251));
		for (index = 0; index < 40; index++)
		writel(g_h265LambdaSad[index + 3] & 0xfffff,
			regs + 4 * (index + 0x279));
	} else {
		for (index = 0; index < 80; index++)
		image[0xad + index] = tune[0xd5 + index] & 0xfffff;
	}
	image[0xfd] =
		v500_tuning_bits(tune, 0x9d0, 3, 8) | v500_tuning_bits(tune, 0x9d4, 1, 0xc) |
		(V500_TUNE(tune, 0x914) & 0x1f);
	image[0xfe] =
		v500_tuning_bits(tune, 0x910, 0xf, 0x10) | v500_tuning_bits(tune, 0x9c4, 0xf, 0xc) |
		v500_tuning_bits(tune, 0x908, 3, 9) | v500_tuning_bits(tune, 0x90c, 1, 8) |
		(V500_TUNE(tune, 0x904) & 0x3f);
	image[0xff] = V500_TUNE(tune, 0x9b0);
	image[0x100] = V500_TUNE(tune, 0x9b4);
	image[0x101] =
		(V500_TUNE(tune, 0x9b8) & 0xffff) | v500_tuning_bits(tune, 0x9cc, 1, 0x15) |
		v500_tuning_bits(tune, 0x9c0, 3, 0x12) | v500_tuning_bits(tune, 0x9bc, 3, 0x10);
	image[0x104] =
		v500_tuning_bits(tune, 0x10c4, 3, 4) | v500_tuning_bits(tune, 0xe38, 1, 1) |
		v500_tuning_bits(tune, 0x10d8, 1, 2);
	image[0x105] = V500_TUNE(tune, 0xf98) << 0x10 | (V500_TUNE(tune, 0xf9c) & 0xffff);
	image[0x106] = v500_tuning_bits(tune, 0x10dc, 0x3f, 6) | (V500_TUNE(tune, 0x10e0) & 0x3f);
	image[0x107] = V500_TUNE(tune, 0xf88) << 0x1f | (V500_TUNE(tune, 0xf8c) & 0x1ffff);
	image[0x108] = v500_tuning_bits(tune, 0xf90, 0x7ff, 0x10) | (V500_TUNE(tune, 0xf94) & 0x3ff);
	if ((config->rc.avbr == 0) || (config->rc.intra != 0)) {
		value = v500_tuning_bits(tune, 0xff8, 1, 3) | v500_tuning_bits(tune, 0xffc, 1, 2) |
			v500_tuning_bits(tune, 0x1000, 1, 1) | (V500_TUNE(tune, 0xff4) & 1);
	} else {
		value = 0xf;
	}
	image[0x109] = value;
	if ((config->rc.avbr == 0) || (config->rc.intra != 0)) {
		value = v500_tuning_bits(tune, 0x1004, 1, 4);
	} else {
		value = 0x10;
	}
	image[0x10a] = value;
	image[0x10b] = (V500_TUNE(tune, 0xea0) & 1);
	image[0x10c] =
		v500_tuning_bits(tune, 0xea8, 1, 0xf) | v500_tuning_bits(tune, 0xeac, 1, 0xe) |
		v500_tuning_bits(tune, 0xeb0, 1, 0xd) | v500_tuning_bits(tune, 0xeb4, 1, 0xc) |
		v500_tuning_bits(tune, 0xeb8, 0xf, 8) | v500_tuning_bits(tune, 0xebc, 0xf, 4) |
		(V500_TUNE(tune, 0xec0) & 0xf);
	image[0x10d] = (V500_TUNE(tune, 0xec4) & 0xf);
	image[0x10e] =
		(V500_TUNE(tune, 0xed0) & 0xff) | v500_tuning_bits(tune, 0xecc, 0xff, 8) |
		v500_tuning_bits(tune, 0xec8, 0xff, 0x10);
	value = V500_TUNE(tune, 0xed8);
	image[0x110] = 0xffffffff;
	image[0x111] = 0xffffffff;
	image[0x113] = 0xffffffff;
	image[0x10f] = (value & 0xffff);
	image[0x112] = 7;
	image[0x114] = 7;
	image[0x115] =
		v500_tuning_bits(tune, 0xf18, 1, 3) | v500_tuning_bits(tune, 0xf14, 1, 2) |
		v500_tuning_bits(tune, 0xf10, 1, 1) | 1;
	image[0x116] = V500_TUNE(tune, 0x100c);
	image[0x117] = V500_TUNE(tune, 0x1010);
	image[0x118] = V500_TUNE(tune, 0x1014);
	image[0x119] = V500_TUNE(tune, 0x1018);
	image[0x11a] = V500_TUNE(tune, 0x101c);
	image[0x11b] = V500_TUNE(tune, 0x1020);
	image[0x11c] = (V500_TUNE(tune, 0xff0) & 1);
	image[0x11d] = (V500_TUNE(tune, 0x10f8) & 0xff) | v500_tuning_bits(tune, 0x10fc, 0xff, 8);
	image[0x11e] = (V500_TUNE(tune, 0x85c) & 0xffff) | V500_TUNE(tune, 0x860) << 0x10;
	image[0x11f] = V500_TUNE(tune, 0x86c) << 0x10;
	packed_value = 0x2601;
	image[0x120] = (V500_TUNE(tune, 0x864) & 0xffff) | V500_TUNE(tune, 0x868) << 0x10;
	image[0x121] = (V500_TUNE(tune, 0x870) & 7);
	if (config->rc.intra == 0) {
		packed_value = 0x201;
	}
	image[0x125] = v500_tuning_bits(tune, 0xfd0, 7, 8) | packed_value << 0x10;
	image[0x126] = (config->slice.bits[0] & 0x1f) | config->slice.bits[1] << 0x10;
	image[0x127] =
		v500_lsl(config->slice.part1, (0x10U - config->slice.bits[0]) & 0xff) & 0xffff;
	image[0x130] = (V500_TUNE(tune, 0x51c) & 0x1f) | V500_TUNE(tune, 0x520) << 0x10;
	image[0x131] = v500_lsl(V500_TUNE(tune, 0x518), (0x10U - V500_TUNE(tune, 0x51c)) & 0xff) & 0xffff;
	packed_value = config->control.protocol;
	if (packed_value == 2) {
		if (config->slice.bits[1] < 0x20) {
			packed_value = v500_lsl(config->slice.cabac[0], (0x20 - config->slice.bits[1]) & 0xff);
		} else {
			packed_value = config->slice.cabac[0];
		}
		image[0x128] = packed_value;
		if (config->slice.bits[1] < 0x40) {
			packed_value = v500_lsl(config->slice.cabac[1], (0x40 - config->slice.bits[1]) & 0xff);
		} else {
			packed_value = config->slice.cabac[1];
		}
		image[0x129] = packed_value;
		if (0x40 < config->slice.bits[1]) {
			image[0x12a] =
				v500_lsl(config->slice.cabac[2], (0x60 - config->slice.bits[1]) & 0xff);
		}
		image[0x12b] = config->slice.cabac[3];
		image[0x12c] = config->slice.cabac[4];
		image[0x12d] = config->slice.cabac[5];
		image[0x12e] = config->slice.cabac[6];
		image[0x12f] = config->slice.cabac[7];
		packed_value = config->control.protocol;
	}
	if (packed_value == 0) {
		value = config->slice.bits[0];
		image[0x142] =
			(((value - 1) & 0x7f)) << 0x10 | ((((value >> 8) - 1) & 0x3f)) << 8 | (((value >> 0x10) - 1) & 0x3f);
		image[0x13a] = config->slice.header[0];
		image[0x13b] = config->slice.header[1];
		image[0x13c] = config->slice.header[2];
		image[0x13d] = config->slice.header[3];
		image[0x13e] = config->slice.reorder[0];
		image[0x13f] = config->slice.reorder[1];
		image[0x140] = config->slice.marking[0];
		image[0x141] = config->slice.marking[1];
	}
	image[0x143] = v500_tuning_bits(tune, 0xfec, 0xffffff, 1);
	packed_value = config->control.protocol;
	if (packed_value == 2) {
		if (V500_TUNE(tune, 0x520) < 0x20) {
			packed_value = v500_lsl(V500_TUNE(tune, 0x4f8), (0x20 - V500_TUNE(tune, 0x520)) & 0xff);
		} else {
			packed_value = V500_TUNE(tune, 0x4f8);
		}
		image[0x132] = packed_value;
		if (V500_TUNE(tune, 0x520) < 0x40) {
			packed_value = v500_lsl(V500_TUNE(tune, 0x4fc), (0x40 - V500_TUNE(tune, 0x520)) & 0xff);
		} else {
			packed_value = V500_TUNE(tune, 0x4fc);
		}
		image[0x133] = packed_value;
		if (0x40 < V500_TUNE(tune, 0x520)) {
			image[0x134] =
				v500_lsl(V500_TUNE(tune, 0x500), (0x60 - V500_TUNE(tune, 0x520)) & 0xff);
		}
		image[0x135] = V500_TUNE(tune, 0x504);
		image[0x136] = V500_TUNE(tune, 0x508);
		image[0x137] = V500_TUNE(tune, 0x50c);
		image[0x138] = V500_TUNE(tune, 0x510);
		image[0x139] = V500_TUNE(tune, 0x514);
		packed_value = config->control.protocol;
	}
	if (packed_value == 0) {
		value = V500_TUNE(tune, 0x4d4);
		image[0x14c] =
			(((value - 1) & 0x7f)) << 0x10 | ((((value >> 8) - 1) & 0x3f)) << 8 | (((value >> 0x10) - 1) & 0x3f);
		image[0x144] = V500_TUNE(tune, 0x4d8);
		image[0x145] = V500_TUNE(tune, 0x4dc);
		image[0x146] = V500_TUNE(tune, 0x4e0);
		image[0x147] = V500_TUNE(tune, 0x4e4);
		image[0x148] = V500_TUNE(tune, 0x4e8);
		image[0x149] = V500_TUNE(tune, 0x4ec);
		image[0x14a] = V500_TUNE(tune, 0x4f0);
		image[0x14b] = V500_TUNE(tune, 0x4f4);
	}
	image[0x14d] = (config->control.pt_bits & 1);
	image[0x14e] = V500_TUNE(tune, 0x1168);
	value = config->control.parameter_set ? 0x10000 : 0;
	mode = value | 0x100;
	if (value) {
		mode |= (config->buffer.parameter_size & 0xff);
		for (index = 0; index < 48; index++)
		image[0x171 + index] = config->buffer.parameter_words[index];
	}
	image[0x170] = mode;
	for (index = 0; index < 16; index++) {
		image[0x15f + index] = config->buffer.stream[index];
		image[0x14f + index] = config->buffer.stream_size[index];
	}
	image[0x16f] = config->buffer.parameter_set;

	writel((V500_TUNE(tune, 0x1108) & 1) | v500_tuning_bits(tune, 0x110c, 1, 3) | 0x30000, regs + 4 * (0x3c00));
	writel(0, regs + 4 * (0x3c02));
	writel(0, regs + 4 * (0x3c04));
	writel(0, regs + 4 * (0x3c07));
	writel(0, regs + 4 * (0x3c08));
	writel(0, regs + 4 * (0x3c0b));
	writel(0x9070001, regs + 4 * (0x3d4c));
	writel(0, regs + 4 * (1));
	writel(config->smmu.ns_table, regs + 4 * (0x3c82));
	writel(config->smmu.ns_table, regs + 4 * (0x3c83));
	writel(config->smmu.ns_error_read, regs + 4 * (0x3cbc));
	writel(config->smmu.ns_error_write, regs + 4 * (0x3cbd));
	writel(config->smmu.ns_error_read, regs + 4 * (0x3cc1));
	writel(config->smmu.ns_error_write, regs + 4 * (0x3cc2));
}

/* V500 HAL selects the HiSTB SMMU global control at engine + 0xf000.
 * Bit 0 is the SMMU global bypass; no CV200 channel/TTBR offsets are used.
 * All source, reference, motion, stream and image addresses are physical DMA.
 */
void histb_v500_bypass(void __iomem *regs)
{
	writel(0x30001, regs + 0xf000);
}

int histb_v500_alloc(struct device *dev, struct histb_v500_memory *memory,
		     unsigned int width, unsigned int height, bool hevc)
{
	u32 mv = DIV_ROUND_UP(1920, 32) * DIV_ROUND_UP(1920, 32) * 32;
	u32 pme = mv * 4;
	u32 info = hevc ? (DIV_ROUND_UP(1920, 64) + 1) * 128 :
			  (DIV_ROUND_UP(1920, 16) + 1) * 32;
	u32 offset, base;
	unsigned int i;

	if (!width || width > 1920 || !height || height > 1088)
		return -EINVAL;

	if (memory->cpu && memory->stride == ALIGN(width, 16) &&
	    memory->luma_size == ALIGN(width, 16) * ALIGN(height, 32) &&
	    memory->hevc == hevc)
		return 0;
	histb_v500_free(dev, memory);
	memory->stride = ALIGN(width, 16);
	memory->luma_size = memory->stride * ALIGN(height, 32);
	memory->hevc = hevc;
	offset = memory->luma_size * 3;
	/* BSP capacity-1080P motion buffers are sized to 1920 square. */
	for (i = 0; i < 2; i++) {
		memory->tmv[i] = offset;
		offset += mv;
	}
	for (i = 0; i < 2; i++) {
		memory->pme[i] = offset;
		offset += pme;
	}
	for (i = 0; i < 3; i++) {
		memory->pme_info[i] = offset;
		offset += info;
	}
	memory->pointers_offset = ALIGN(offset, 4096);
	memory->image_offset = memory->pointers_offset + 4096;
	/* Image plus the HAL's 0x116c-byte native tuning workspace. */
	memory->size = memory->image_offset + 4096 + ALIGN(0x116c, 4096);
	memory->cpu = dma_alloc_coherent(dev, memory->size, &memory->dma,
					 GFP_KERNEL);
	if (!memory->cpu) {
		memset(memory, 0, sizeof(*memory));
		return -ENOMEM;
	}
	if (upper_32_bits(memory->dma) ||
	    upper_32_bits((u64)memory->dma + memory->size - 1)) {
		histb_v500_free(dev, memory);
		return -ERANGE;
	}
	memset(memory->cpu, 0, memory->size);
	base = lower_32_bits(memory->dma);
	for (i = 0; i < 2; i++) {
		memory->tmv[i] += base;
		memory->pme[i] += base;
	}
	for (i = 0; i < 3; i++)
		memory->pme_info[i] += base;
	return 0;
}

void histb_v500_free(struct device *dev, struct histb_v500_memory *memory)
{
	if (memory->cpu)
		dma_free_coherent(dev, memory->size, memory->cpu, memory->dma);
	memset(memory, 0, sizeof(*memory));
}
