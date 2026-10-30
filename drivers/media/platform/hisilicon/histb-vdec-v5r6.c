// SPDX-License-Identifier: GPL-2.0-only
/*
 * MV310 VDH V5R6 backend, recovered from the shipped ARM32 VFMW v5.1 HAL.
 * Authority: Android kernel SHA256
 * 8adb8e7580bebce1716396271dbed3575fb870b536124608388f20f4404ce4a3,
 * h264_hal_set_pic_msg / set_slice_msg, hevc_hal_set_pic_msg / set_slice_msg,
 * hal_cfg_vdm_reg, vdmhal_reset_vdm and vdmhal_start_hw_decode.
 * These address encodings are not those of the CV200 V4R3 engine.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/regmap.h>

#include "histb-vdec-v5r6.h"

#define V5R6_INT_MASK		0x0024
#define V5R6_WRAPPER_STATE	0xf804
#define V5R6_RESET_REQUEST	0xf80c
#define V5R6_RESET_ACK		0xf810

static void histb_v5r6_address_word(__le32 *word)
{
	*word = cpu_to_le32(le32_to_cpu(*word) >> 4);
}

int histb_v5r6_configure_clock(struct regmap *crg)
{
	u32 mask = GENMASK(9, 8) | GENMASK(16, 12) | BIT(17);
	int ret;

	/* pdt's open callback selects 2; start clears skip and pulses LOAD. */
	ret = regmap_update_bits(crg, 0x78, mask, FIELD_PREP(GENMASK(9, 8), 2));
	if (ret)
		return ret;
	return regmap_update_bits(crg, 0x78, BIT(17), BIT(17));
}

int histb_v5r6_reset(void __iomem *regs)
{
	u32 mask = readl(regs + V5R6_INT_MASK);
	u32 request, ack;
	int ret;

	if (!(readl(regs + V5R6_WRAPPER_STATE) & GENMASK(1, 0)))
		return 0;
	request = readl(regs + V5R6_RESET_REQUEST);
	wmb();
	writel(request | BIT(1), regs + V5R6_RESET_REQUEST);
	ret = readl_poll_timeout(regs + V5R6_RESET_ACK, ack,
				 ack & BIT(1), 30, 30000);
	writel(request & ~BIT(1), regs + V5R6_RESET_REQUEST);
	wmb();
	writel(mask, regs + V5R6_INT_MASK);
	return ret;
}

void histb_v5r6_start(void __iomem *regs,
		     const struct histb_v5r6_registers *config)
{
	/* Physical DMA bypass: neither message addresses nor references are IOVAs. */
	writel(~0U, regs + V5R6_INT_MASK);
	writel(0, regs + 0xf000);
	writel(7, regs + 0xf020);
	writel(7, regs + 0xf01c);
	writel(7, regs + 0xf02c);
	writel(config->avs, regs + 0xc000);
	writel(1, regs + 0xc008);
	writel(config->basic_cfg0, regs + 0x0008);
	writel(config->basic_cfg1, regs + 0x000c);
	writel(lower_32_bits(config->picture >> 4), regs + 0x0010);
	writel(lower_32_bits(config->report >> 4), regs + 0x0014);
	writel(lower_32_bits(config->stream >> 4), regs + 0x0018);
	writel(config->emar ? readl(regs + 0xc004) | BIT(0) : 0,
	       regs + 0xc004);
	writel(lower_32_bits(config->reconstruction >> 4), regs + 0x0060);
	writel(config->y_stride, regs + 0x0064);
	writel(config->chroma_offset, regs + 0x0068);
	writel(0, regs + 0x006c);
	if (config->low_bits) {
		writel(config->low_stride, regs + 0x0074);
		writel(config->low_y_offset, regs + 0x0078);
		writel(config->low_chroma_offset, regs + 0x007c);
	}
	writel(~0U, regs + 0x0020);
	writel(2, regs + 0xc00c);
	writel(0xaaaaaaaa, regs + 0x009c);
	writel(~1U, regs + V5R6_INT_MASK);
	usleep_range(30, 60);
	wmb();
	writel(0, regs);
	writel(1, regs);
	writel(0, regs);
}

void histb_v5r6_encode_h264_picture(__le32 *picture, bool new_picture)
{
	unsigned int i;

	histb_v5r6_address_word(&picture[1]);
	for (i = 5; i <= 8; i++)
		histb_v5r6_address_word(&picture[i]);
	/* Slice-mode requests retain the already encoded first-slice pointer. */
	if (new_picture)
		histb_v5r6_address_word(&picture[9]);
	for (i = 10; i <= 26; i++)
		histb_v5r6_address_word(&picture[i]);
	histb_v5r6_address_word(&picture[59]);
}

void histb_v5r6_encode_h264_slice(__le32 *slice)
{
	histb_v5r6_address_word(&slice[0]);
	histb_v5r6_address_word(&slice[3]);
	histb_v5r6_address_word(&slice[8]);
	histb_v5r6_address_word(&slice[9]);
	histb_v5r6_address_word(&slice[63]);
}

void histb_v5r6_encode_hevc_picture(__le32 *picture)
{
	unsigned int i;

	/* V5R6 has no external SED-top picture address at D4. */
	picture[4] = 0;
	for (i = 5; i <= 23; i++)
		histb_v5r6_address_word(&picture[i]);
	histb_v5r6_address_word(&picture[25]);
	histb_v5r6_address_word(&picture[43]);
	for (i = 55; i <= 59; i++)
		histb_v5r6_address_word(&picture[i]);
	histb_v5r6_address_word(&picture[63]);
}

void histb_v5r6_encode_hevc_slice(__le32 *slice)
{
	histb_v5r6_address_word(&slice[0]);
	histb_v5r6_address_word(&slice[3]);
	histb_v5r6_address_word(&slice[23]);
	/* Native V5R6 slice message explicitly initializes this PMV control. */
	slice[37] = cpu_to_le32(BIT(27));
	histb_v5r6_address_word(&slice[63]);
}

void histb_v5r6_encode_mpeg2_picture(__le32 *picture)
{
	unsigned int i;

	/* mp2_hal_cfg_dn_msg: syntax D0-D3 and matrices D16-D47 match. */
	for (i = 4; i <= 7; i++)
		histb_v5r6_address_word(&picture[i]);
	histb_v5r6_address_word(&picture[48]);
	histb_v5r6_address_word(&picture[63]);
}

void histb_v5r6_encode_mpeg2_slice(__le32 *slice)
{
	/* mp2_hal_write_slice_msg: 24-bit relative 16-byte addresses. */
	slice[1] = cpu_to_le32((le32_to_cpu(slice[1]) >> 4) & 0xffffff);
	slice[3] = cpu_to_le32((le32_to_cpu(slice[3]) >> 4) & 0xffffff);
	histb_v5r6_address_word(&slice[7]);
}

void histb_v5r6_encode_avs_picture(__le32 *picture,
				   unsigned int slice_count)
{
	u32 syntax = le32_to_cpu(picture[1]);
	unsigned int i;

	/* avs_hal_start_dec forces frame processing for a single slice. */
	if (slice_count == 1)
		syntax = (syntax & ~GENMASK(2, 1)) | GENMASK(2, 1);
	picture[1] = cpu_to_le32(syntax | BIT(18));
	for (i = 4; i <= 7; i++)
		histb_v5r6_address_word(&picture[i]);
	picture[9] = cpu_to_le32(0x60001);
	picture[10] = cpu_to_le32(0x80019);
	picture[11] = cpu_to_le32(0x30004);
	histb_v5r6_address_word(&picture[13]);
	histb_v5r6_address_word(&picture[14]);
	histb_v5r6_address_word(&picture[16]);
	/* D17-D48 PMV coefficients and D64-D79 WQ matrices are not addresses. */
	histb_v5r6_address_word(&picture[49]);
	histb_v5r6_address_word(&picture[50]);
	/* V5R6 has no CV200 external SED-top pointer here. */
	picture[51] = 0;
	picture[57] = 0;
	picture[58] = cpu_to_le32(0xffffffff);
	picture[59] = cpu_to_le32(0x1ffff);
	picture[60] = cpu_to_le32(0x410007);
	picture[61] = cpu_to_le32(0xfff);
	histb_v5r6_address_word(&picture[63]);
}

void histb_v5r6_encode_avs_slice(__le32 *slice, u32 tail_offset)
{
	/* avs_hal_write_slice_msg's contiguous-input second fragment. */
	histb_v5r6_address_word(&slice[1]);
	slice[2] = cpu_to_le32(0x8000 | ((tail_offset & 15) << 28));
	slice[3] = cpu_to_le32(tail_offset >> 4);
	histb_v5r6_address_word(&slice[63]);
}

int histb_v5r6_reset_bpd(void __iomem *regs)
{
	u32 mask = readl(regs + 0xd034);
	u32 request = readl(regs + V5R6_RESET_REQUEST);
	u32 ack;
	int ret;

	/* bpd_reset requests wrapper bit3; the caller owns BPD state/mask. */
	wmb();
	writel(request | BIT(3), regs + V5R6_RESET_REQUEST);
	ret = readl_poll_timeout(regs + V5R6_RESET_ACK, ack,
				 ack & BIT(3), 30, 30000);
	writel(request & ~BIT(3), regs + V5R6_RESET_REQUEST);
	wmb();
	writel(mask, regs + 0xd034);
	return ret;
}
