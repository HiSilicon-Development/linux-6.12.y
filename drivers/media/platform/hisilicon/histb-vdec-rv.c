// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 *
 * RealVideo 8 / RealVideo 9 front-end for the Hi3798CV200 VDH decoder.
 *
 * Split of work follows RV-SPEC.md section 5: this front-end supplies the
 * picture header fields, the quantiser state and, per slice, a
 * (bit_stream_addr, bit_offset, bit_len) triple per fragment.  It never
 * touches coefficients - entropy decode, motion compensation, inverse
 * transform and in-loop deblocking are all the hardware's job.
 *
 * What is settled and implemented here: the 64-word picture message
 * (RV-SPEC.md 2.2/2.3), the 256-byte slice slot with its D63 link and the
 * last_mb_in_slice computation (2.4), the register set and the format nibble
 * (3.1-3.3), and the "references must never be zero" rule (4.3).
 *
 * What is not settled: the bit-level syntax of the picture and slice
 * headers.  `real8.c`/`real9.c` are absent from this BSP drop and ship only
 * as assembly, so the exact meaning of each field is RV-SPEC Q1.  The
 * readers below implement the field widths that the BSP's BsShow/BsSkip
 * sequence does establish, and stop rather than invent the rest.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "histb-vdec-rv.h"

/*
 * ---- big-endian bit reader -------------------------------------------
 *
 * RealVideo codes its headers MSB-first from the start of the access unit.
 * The bit widths below come from the BsShow/BsSkip sequence that
 * RV-SPEC.md section 5.1 recovers from SYN/real8.S:1184-1378.
 */
struct histb_rv_bits {
	const __u8 *data;
	__u32 bits;
	__u32 pos;
	bool overrun;
};

static void rv_bits_init(struct histb_rv_bits *br, const __u8 *data, __u32 bytes)
{
	br->data = data;
	br->bits = bytes * 8;
	br->pos = 0;
	br->overrun = false;
}

static __u32 rv_bits_get(struct histb_rv_bits *br, unsigned int count)
{
	__u32 value = 0;
	unsigned int i;

	if (br->overrun || br->pos + count > br->bits) {
		br->overrun = true;
		return 0;
	}

	for (i = 0; i < count; i++) {
		value = (value << 1) |
			((br->data[br->pos >> 3] >> (7 - (br->pos & 7))) & 1);
		br->pos++;
	}

	return value;
}

__maybe_unused static void rv_bits_skip(struct histb_rv_bits *br, unsigned int count)
{
	if (br->pos + count > br->bits) {
		br->overrun = true;
		return;
	}
	br->pos += count;
}

/*
 * The variable-length code used by the picture and slice headers.
 *
 * `gs_VLCDecodeTable` is not a guess: it is a 256-byte object defined in the
 * BSP's own assembly, and the loop that consumes it is readable.  From
 * real8.S:68-106:
 *
 *     peek 8 bits -> t = table[peek]
 *     value_bits = t & 0x0F
 *     length     = t >> 4
 *     accumulate length, splice value_bits in above the bits decoded so far,
 *     skip `length` bits, and repeat until the accumulated length is odd.
 *
 * The table below is that object, transcribed verbatim (it is byte-identical
 * across the chips that carry RealVideo in this BSP drop).
 */
static const __u8 gs_VLCDecodeTable[256] = {
	0x80, 0x81, 0x70, 0x70, 0x82, 0x83, 0x71, 0x71,
	0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50,
	0x84, 0x85, 0x72, 0x72, 0x86, 0x87, 0x73, 0x73,
	0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51,
	0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
	0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
	0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
	0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
	0x88, 0x89, 0x74, 0x74, 0x8a, 0x8b, 0x75, 0x75,
	0x52, 0x52, 0x52, 0x52, 0x52, 0x52, 0x52, 0x52,
	0x8c, 0x8d, 0x76, 0x76, 0x8e, 0x8f, 0x77, 0x77,
	0x53, 0x53, 0x53, 0x53, 0x53, 0x53, 0x53, 0x53,
	0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
	0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
	0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
	0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
};

/*
 * Real8_CB_GetVLCBits (real8.S:68-106).  Returns the accumulated length in
 * bits, which is also what BsSkip consumes; the decoded value is the sum of
 * the value nibbles of each segment, shifted by the running width.
 */
static __u32 rv_bits_vlc(struct histb_rv_bits *br, __u32 *value_out)
{
	__u32 total = 0;
	__u32 value = 0;
	unsigned int guard = 0;

	for (guard = 0; guard < 64; guard++) {
		__u32 peek, entry, len, val;

		peek = rv_bits_get(br, 8);
		if (br->overrun)
			break;

		/* put the eight bits back: the table entry decides the length */
		br->pos -= 8;

		entry = gs_VLCDecodeTable[peek & 0xff];
		len = entry >> 4;
		val = entry & 0x0f;

		if (!len)
			break;

		value = (value << len) | val;
		rv_bits_skip(br, len);
		total += len;

		/* the hardware loop stops when the accumulated length is odd */
		if (total & 1)
			break;
	}

	if (value_out)
		*value_out = value;

	return total;
}

/* ---- parser state ---------------------------------------------------- */

void histb_rv_parser_reset(struct histb_rv_parser *parser, __u8 std)
{
	if (!parser)
		return;

	memset(parser, 0, sizeof(*parser));
	parser->std = std;
	parser->blocker = HISTB_RV_BLOCK_NONE;
}

/*
 * ---- picture header --------------------------------------------------
 *
 * Real8_CB_GetPictureHeader, SYN/real8.S:1184-1378.  The observable
 * structure, per RV-SPEC.md section 5.1:
 *
 *   - 24-bit start-code field, skipped, compared against 1
 *   - a VLC tag that must be > 30 with bit 0 clear
 *   - two dimension fields extracted with ubfx, 5 and 8 bits wide
 *   - the coding type selected from the VLC tag
 *   - CPFMT defaults to 144x176 when a flag bit is clear
 *   - dimensions converted to macroblocks with (w+15)>>4, (h+15)>>4
 *
 * The meaning of each individual field is RV-SPEC Q1, so fields that the
 * assembly does not pin down are left at their defaults and the frame is
 * marked blocked rather than guessed at.
 */
static int rv_parse_picture_header(struct histb_rv_parser *parser,
				   struct histb_rv_bits *br,
				   struct histb_rv_frame *frame)
{
	struct histb_rv_picture_header *h = &frame->header;
	__u32 start_code, tag;

	start_code = rv_bits_get(br, 24);
	if (br->overrun)
		return HISTB_RV_NEED_MORE;

	/* :1206-1211 compares the skipped 24-bit field against 1. */
	if (start_code != 1)
		return HISTB_RV_INVALID;

	{
		__u32 vlc_value = 0;

		rv_bits_vlc(br, &vlc_value);
		tag = vlc_value;
	}
	if (br->overrun)
		return HISTB_RV_NEED_MORE;

	/*
	 * The coding type is selected from the VLC tag; Real8_CB_GetPictureHeader
	 * maps three tag values onto it (:1229-1240, :1260-1280).  The tag is now
	 * decodable, but which of the three values means which type is still not
	 * pinned down, so the frame stays blocked on that point alone rather than
	 * guessing an interpretation.
	 */
	h->pic_coding_type = tag & 0x3;
	h->pic_coding_type = HISTB_RV_INTRAPIC;

	/* Dimension fields, widths from the ubfx at :1222-1226. */
	{
		__u32 hi = rv_bits_get(br, 5);
		__u32 lo = rv_bits_get(br, 8);

		if (br->overrun)
			return HISTB_RV_NEED_MORE;

		/*
		 * CPFMT default 144x176 when the flag is clear (:1259-1262).
		 * Which bit is the flag is Q1, so the parsed pair is used
		 * directly and the default is not applied.
		 */
		h->pic_size_code_hi = (__u8)hi;
		h->pic_size_code = (__u8)lo;
		h->pic_width_in_pixel = (__u16)((hi << 8) | lo);
		h->pic_height_in_pixel = 0;
	}

	/*
	 * Real8_CB_SetDimensions (:1254-1255) derives the rest.  Height is
	 * not available from the fields above without Q1, so the frame is
	 * marked blocked and validate_stateful_frame() will refuse it.
	 */
	h->pic_width_in_mb = (__u16)((h->pic_width_in_pixel + 15) >> 4);
	h->pic_height_in_mb = (__u16)((h->pic_height_in_pixel + 15) >> 4);
	h->total_mbs = (__u16)(h->pic_width_in_mb * h->pic_height_in_mb);

	parser->blocker = HISTB_RV_BLOCK_CODING_TYPE;

	return HISTB_RV_OK;
}

/* Real8_CB_FindNextSliceStartCode, SYN/real8.S:709-760. */
static __u32 rv_find_next_start_code(const __u8 *data, __u32 bytes, __u32 from)
{
	__u32 i;

	for (i = from; i + 3 < bytes; i++) {
		if (data[i] == 0 && data[i + 1] == 0 &&
		    data[i + 2] == 1 && data[i + 3] == 0)
			return i;
	}

	return bytes;
}

int histb_rv_parse_frame_at(struct histb_rv_parser *parser, const __u8 *data,
			    __u32 bytes, __u32 offset,
			    struct histb_rv_frame *frame,
			    __u32 *next_offset)
{
	struct histb_rv_bits br;
	int ret;

	if (!parser || !data || !frame)
		return HISTB_RV_INVALID;

	if (offset >= bytes)
		return HISTB_RV_NEED_MORE;

	memset(frame, 0, sizeof(*frame));
	frame->std = parser->std;

	rv_bits_init(&br, data + offset, bytes - offset);
	br.pos = 0;

	ret = rv_parse_picture_header(parser, &br, frame);
	if (ret != HISTB_RV_OK)
		return ret;

	if (next_offset)
		*next_offset = rv_find_next_start_code(data, bytes,
						       offset + (br.pos >> 3));

	return HISTB_RV_OK;
}

int histb_rv_parse_frame(struct histb_rv_parser *parser, const __u8 *data,
			 __u32 bytes, struct histb_rv_frame *frame)
{
	__u32 next = 0;
	int ret;

	ret = histb_rv_parse_frame_at(parser, data, bytes, 0, frame, &next);
	if (ret != HISTB_RV_OK)
		return ret;

	if (next <= 0 || next >= bytes) {
		parser->blocker = HISTB_RV_BLOCK_NO_SLICE;
		return HISTB_RV_INVALID;
	}

	return HISTB_RV_OK;
}

/*
 * ---- validation before launch ----------------------------------------
 *
 * RV-SPEC.md section 4.3: the reference addresses in picture-message words
 * 17 and 18 must always be valid, decodable dma addresses - never zero.
 * The vendor points both at the FSP null frame store before the first
 * anchor picture (SYN/real8.S:161-171), and the port's MPEG-4 front-end
 * achieves the same by refusing to launch a P/B picture without the
 * references it needs.  This is that check.
 */
int histb_rv_validate_stateful_frame(struct histb_rv_parser *parser,
				     const __u8 *data, __u32 bytes,
				     const struct histb_rv_frame *frame)
{
	if (!parser || !frame)
		return HISTB_RV_INVALID;

	if (parser->blocker != HISTB_RV_BLOCK_NONE)
		return HISTB_RV_UNSUPPORTED;

	switch (frame->header.pic_coding_type) {
	case HISTB_RV_INTRAPIC:
	case HISTB_RV_INTERPIC:
	case HISTB_RV_TRUEBPIC:
	case HISTB_RV_FRUPIC:
		break;
	default:
		parser->blocker = HISTB_RV_BLOCK_CODING_TYPE;
		return HISTB_RV_UNSUPPORTED;
	}

	if (!frame->header.total_mbs) {
		parser->blocker = HISTB_RV_BLOCK_GEOMETRY;
		return HISTB_RV_UNSUPPORTED;
	}

	(void)data;
	(void)bytes;

	return HISTB_RV_OK;
}

/*
 * ---- the 64-word picture message -------------------------------------
 *
 * RV-SPEC.md section 2.2 (RV8, HAL/vdm_hal_real8.S:331-492) and 2.3 (RV9).
 * Both are 64 words; only the source offsets differ, which is why the
 * fields below are taken from the port's hw_picture rather than from a
 * vendor struct.  Words 7-15 and 25-62 are never written by the HAL and are
 * reserved-and-must-be-zero (2.2), so the message is zeroed first.
 */
int histb_rv_build_pic_msg(const struct histb_rv_hw_picture *picture,
			   struct histb_rv_pic_msg *msg)
{
	const struct histb_rv_picture_header *h;
	bool rv9;

	if (!picture || !msg)
		return HISTB_RV_INVALID;

	memset(msg, 0, sizeof(*msg));

	h = &picture->frame.header;
	rv9 = (picture->std == HISTB_RV_STD_REAL9);

	/* word 0 [1:0] = PicCodingType */
	msg->d[0] = h->pic_coding_type & 0x3;
	/* word 1 [8:0] = PicWidthInMb-1, [24:16] = PicHeightInMb-1 */
	msg->d[1] = ((h->pic_width_in_mb ? h->pic_width_in_mb - 1 : 0) & 0x1ff) |
		    (((h->pic_height_in_mb ? h->pic_height_in_mb - 1 : 0) & 0x1ff) << 16);
	/* word 2 [15:0] = Ratio0, [31:16] = Ratio1 */
	msg->d[2] = (h->ratio0 & 0xffff) | ((__u32)h->ratio1 << 16);
	/* word 3 always 0 (the memset left it so) */
	/* word 4 [4:0] = PQUANT */
	msg->d[4] = h->pquant & 0x1f;
	/* word 5 [4:0] = PrevPicQP */
	msg->d[5] = h->pquant & 0x1f;	/* filled by the core from the parser */
	/* word 6 [4:0] = PrevPicMb0QP; RV9 adds bit 5 for <= 99 macroblocks */
	msg->d[6] = 0;
	if (rv9) {
		/*
		 * HAL/vdm_hal_real9.S:413-424 sets bit 5 when
		 * PicWidthInMb * PicHeightInMb <= 99.  The bit's meaning is
		 * not documented in the BSP - RV-SPEC Q6 - so it is
		 * reproduced because the HAL does it, not because it is
		 * understood.
		 */
		if (h->total_mbs && h->total_mbs <= 99)
			msg->d[6] |= (1u << 5);
	}
	/* words 16-18: display and reference planes, low nibble masked off */
	msg->d[16] = picture->disp_frame_phy_addr & ~15u;
	msg->d[17] = picture->fwd_ref_phy_addr & ~15u;
	msg->d[18] = picture->bwd_ref_phy_addr & ~15u;
	/* words 19-20: PMV planes, not masked */
	msg->d[19] = picture->curr_pmv_phy_addr;
	msg->d[20] = picture->col_pmv_phy_addr;
	/* words 21-24: workspace tops */
	msg->d[21] = picture->sed_top_addr & ~15u;
	msg->d[22] = picture->pmv_top_addr & ~15u;
	msg->d[23] = picture->rcn_top_addr & ~15u;
	msg->d[24] = picture->dblk_top_addr;
	/* word 63: physical address of the slice-message area */
	msg->d[63] = (picture->slice_msg_addr & ~15u) + 256;

	return HISTB_RV_OK;
}

/*
 * ---- registers -------------------------------------------------------
 *
 * RV-SPEC.md 3.1 (BASIC_CFG1 with the format nibble), 3.2 (BASIC_CFG0) and
 * 3.3 (the full list).  BASIC_CFG0 is 0x01000000 | (macroblock count - 1)
 * for both revisions; the nibble in BASIC_CFG1 is 8 for RV8 and 9 for RV9
 * and is the same numeric field as VID_STD_E.
 */
int histb_rv_build_regs(const struct histb_rv_hw_picture *picture,
			const struct histb_rv_slice *slices, __u32 slice_count,
			struct histb_rv_regs *regs)
{
	const struct histb_rv_picture_header *h;
	__u32 mbs;

	if (!picture || !regs)
		return HISTB_RV_INVALID;

	memset(regs, 0, sizeof(*regs));

	h = &picture->frame.header;
	mbs = h->total_mbs;

	regs->basic_cfg0 = 0x01000000u | ((mbs ? mbs - 1 : 0) & 0xfffffu);
	regs->basic_cfg1 = ((__u32)picture->std & 0xf) |		/* [3:0] */
			   ((picture->ddr_stride >> 6) & 0x7f) << 4 |	/* [10:4] */
			   ((__u32)picture->vdh_mmu_en << 12) |		/* 12 */
			   ((__u32)picture->fst_slc_grp << 14) |	/* 14 */
			   (1u << 15) |					/* constant 1 */
			   ((__u32)picture->compress_en << 30);		/* 30 */

	regs->avm_addr = picture->pic_msg_addr & ~15u;
	regs->vam_addr = picture->vam_addr & ~15u;
	regs->stream_base = picture->stream_base_addr;
	regs->current_picture_addr = picture->cur_pic_phy_addr & ~15u;
	regs->y_stride = picture->ddr_stride;
	regs->uv_offset = picture->uv_offset;
	regs->head_info_size = picture->head_info_size;
	regs->dnr_mbinfo_addr = picture->dnr_mbinfo_addr;

	/*
	 * RV-SPEC.md 3.3: the seven identical 0x00300C03 timeout values and
	 * the zeroed REF_PIC_TYPE / FF_APT_EN match what the port already
	 * writes for MPEG-4, so no new constants are introduced here.
	 */
	regs->fixed_cfg = 0x00300c03u;
	regs->int_state_reset = 0xffffffffu;	/* register 0x20, RV-SPEC Q8 */
	regs->scd_emar = (h->pic_width_in_mb < 256) ? 1 : 0;

	(void)slices;
	(void)slice_count;

	return HISTB_RV_OK;
}

/*
 * ---- slices ----------------------------------------------------------
 *
 * RV-SPEC.md 2.4: the slice area starts at picture message + 256 and each
 * slot is 64 words.  The HAL deduplicates by skipping any element whose
 * first_mb_in_slice is not greater than the previous emitted one
 * (HAL/vdm_hal_real8.S:613-622), and computes last_mb_in_slice itself.
 */
int histb_rv_make_single_slice(const struct histb_rv_frame *frame,
			       __u32 buffer_dma,
			       struct histb_rv_slice *slice)
{
	if (!frame || !slice)
		return HISTB_RV_INVALID;

	memset(slice, 0, sizeof(*slice));
	slice->first_mb_in_slice = 0;
	slice->last_mb_in_slice = frame->header.total_mbs ?
		frame->header.total_mbs - 1 : 0;
	slice->dma_addr[0] = buffer_dma;
	slice->bit_len[0] = 0;

	return HISTB_RV_OK;
}

int histb_rv_parse_slices(const struct histb_rv_frame *frame,
			  const __u8 *data, __u32 bytes, __u32 buffer_dma,
			  struct histb_rv_slice *slices, __u32 slice_capacity,
			  __u32 *slice_count)
{
	if (!frame || !slices || !slice_count || !slice_capacity)
		return HISTB_RV_INVALID;

	/*
	 * Writing out one slice covering the whole picture is the degenerate
	 * case and the one the MPEG-4 front-end uses when a stream carries
	 * no per-slice framing.  Real slice-boundary parsing needs the
	 * slice-header syntax, which is RV-SPEC Q1.
	 */
	if (histb_rv_make_single_slice(frame, buffer_dma, &slices[0]) !=
	    HISTB_RV_OK)
		return HISTB_RV_INVALID;

	*slice_count = 1;

	(void)data;
	(void)bytes;

	return HISTB_RV_OK;
}

/*
 * RV8HAL_V4R3C1_WriteSliceMsg (HAL/vdm_hal_real8.S:561-819).  Per slot, with
 * base = picmsg + 256 + i*256:
 *
 *   word 0  StreamLength[0][23:0] | StreamBitOffset[0][6:0] << 24
 *   word 1  StreamPhyAddr[0]
 *   word 2  StreamLength[1][23:0] | StreamBitOffset[1][6:0] << 24
 *   word 3  StreamPhyAddr[1]
 *   word 4  (dblk | osvquant << 1 | sliceqp << 3) << 16
 *   word 5  first_mb_in_slice[15:0] | last_mb_in_slice << 16
 *   word 63 physical address of the next slot, or 0 for the last
 */
int histb_rv_build_slice_msg(const struct histb_rv_frame *frame,
			     const struct histb_rv_slice *slice,
			     struct histb_rv_slice_msg *msg)
{
	if (!slice || !msg)
		return HISTB_RV_INVALID;

	memset(msg, 0, sizeof(*msg));

	msg->d[0] = (slice->bit_len[0] & 0xffffff) |
		    ((__u32)(slice->bit_offset[0] & 0x7f) << 24);
	msg->d[1] = slice->dma_addr[0];
	msg->d[2] = (slice->bit_len[1] & 0xffffff) |
		    ((__u32)(slice->bit_offset[1] & 0x7f) << 24);
	msg->d[3] = slice->dma_addr[1];
	msg->d[4] = ((__u32)(slice->dblk_filter_passthrough & 1) |
		     ((__u32)(slice->osvquant & 3) << 1) |
		     ((__u32)(slice->sliceqp & 0x1f) << 3)) << 16;
	msg->d[5] = (slice->first_mb_in_slice & 0xffff) |
		    ((slice->last_mb_in_slice & 0xffff) << 16);

	(void)frame;

	return HISTB_RV_OK;
}

int histb_rv_build_slice_messages(const struct histb_rv_frame *frame,
				  const struct histb_rv_slice *slices,
				  __u32 slice_count, __u32 message_dma,
				  struct histb_rv_slice_msg *messages,
				  __u32 message_capacity,
				  __u32 *message_count)
{
	__u32 i;

	if (!frame || !slices || !messages || !message_count || !slice_count)
		return HISTB_RV_INVALID;

	if (slice_count > message_capacity)
		return HISTB_RV_UNSUPPORTED;

	for (i = 0; i < slice_count; i++) {
		int ret = histb_rv_build_slice_msg(frame, &slices[i],
						   &messages[i]);

		if (ret != HISTB_RV_OK)
			return ret;

		/*
		 * The HAL computes last_mb_in_slice itself: for intermediate
		 * slices it is the next slice's first_mb_in_slice - 1, for
		 * the last it is total_mbs - 1 (HAL/vdm_hal_real8.S:737-760,
		 * :788-804).  The port must do the same because nothing else
		 * fills the field.
		 */
		if (i + 1 < slice_count) {
			messages[i].d[5] =
				(slices[i].first_mb_in_slice & 0xffff) |
				(((slices[i + 1].first_mb_in_slice ?
				   slices[i + 1].first_mb_in_slice - 1 : 0)
				  & 0xffff) << 16);
		}

		/* D63 links to the next slot, zero on the last. */
		messages[i].d[63] = (i + 1 < slice_count) ?
			(message_dma + (i + 1) * HISTB_RV_SLICE_MSG_SIZE) : 0;
	}

	*message_count = slice_count;

	return HISTB_RV_OK;
}

/*
 * ---- display ordering ------------------------------------------------
 *
 * Deliberately conservative.  RV-SPEC.md Q5: the vendor drives output
 * through FSP_SetDisplay/FSP_GetDisplay with RealVideo's own TR/TRB B-picture
 * timing, and the analysis states plainly that this is not reducible to the
 * MPEG-4 pending/current model without further reverse engineering.  So no
 * reorder policy is invented here: an anchor picture that precedes a pending
 * one releases it, and nothing else is decided.
 */
int histb_rv_plan_display(struct histb_rv_display_state *state,
			  __u8 coding_type, __u8 first_anchor, __u8 draining,
			  struct histb_rv_display_plan *plan)
{
	if (!state || !plan)
		return HISTB_RV_INVALID;

	memset(plan, 0, sizeof(*plan));

	if (draining) {
		/* Flush whatever is held: B pictures can no longer follow. */
		plan->release_pending = state->pending;
		plan->release_current = 1;
		state->pending = 0;
		return HISTB_RV_OK;
	}

	if (coding_type == HISTB_RV_INTRAPIC) {
		/*
		 * An anchor picture ends the B run that the pending picture
		 * belongs to, so it is released first and the anchor is held.
		 */
		plan->release_pending = state->pending;
		plan->current_before_pending = 0;
		plan->hold_current = 1;
		state->pending = 1;
		return HISTB_RV_OK;
	}

	if (coding_type == HISTB_RV_TRUEBPIC || coding_type == HISTB_RV_FRUPIC) {
		/*
		 * B pictures are displayed in the order they arrive relative
		 * to the pending anchor.  The exact rule is Q5, so the
		 * pending picture is left pending and this one is held.
		 */
		plan->hold_current = 1;
		plan->pending_last = state->pending;
		return HISTB_RV_OK;
	}

	/* P pictures: no reordering observed in the vendor's sequence. */
	plan->hold_current = 1;
	(void)first_anchor;

	return HISTB_RV_OK;
}
