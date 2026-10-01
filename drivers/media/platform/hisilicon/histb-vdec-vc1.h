/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 */
#ifndef HISTB_VDEC_VC1_H
#define HISTB_VDEC_VC1_H

#include <linux/types.h>

#define HISTB_VC1_BPD_CFG_WORDS		12
#define HISTB_VC1_BPD_PLANES		7
#define HISTB_VC1_MAX_MB_DIMENSION	128
#define HISTB_VC1_MAX_SLICES		256
#define HISTB_VC1_PIC_MSG_WORDS		64
#define HISTB_VC1_SLICE_FRAGMENTS	2
#define HISTB_VC1_SLICE_MSG_WORDS	64
#define HISTB_VC1_MAX_UP_REPORTS	200
#define HISTB_VC1_STATE_DECODE_DONE	(1U << 17)
#define HISTB_VC1_STATE_DECODE_ERROR	(1U << 18)
#define HISTB_VC1_BFRACTION_COUNT	21
#define HISTB_VC1_INTENSITY_REFS		3
#define HISTB_VC1_INTENSITY_PARITIES	2
#define HISTB_VC1_INTENSITY_COMPONENTS	2
#define HISTB_VC1_INTENSITY_VALUES	256
#define HISTB_VC1_INTENSITY_MAP_SIZE	\
	(HISTB_VC1_INTENSITY_PARITIES * HISTB_VC1_INTENSITY_COMPONENTS * \
	 HISTB_VC1_INTENSITY_VALUES)
#define HISTB_VC1_INTENSITY_WORK_SIZE	\
	(HISTB_VC1_INTENSITY_REFS * HISTB_VC1_INTENSITY_MAP_SIZE)

/* SMPTE 421M Annex G start-code values used by CV200 VC1_GetStartCode. */
#define HISTB_VC1_CODE_END_OF_SEQUENCE	0x0a
#define HISTB_VC1_CODE_SLICE		0x0b
#define HISTB_VC1_CODE_FIELD		0x0c
#define HISTB_VC1_CODE_FRAME		0x0d
#define HISTB_VC1_CODE_ENTRY_POINT	0x0e
#define HISTB_VC1_CODE_SEQUENCE		0x0f

/* CV200 bitplane.h maps these registers at VDH + 0xd000. */
#define HISTB_VC1_BPD_OFFSET		0xd000
#define HISTB_VC1_BPD_START		0x000
#define HISTB_VC1_BPD_CFG0		0x004
#define HISTB_VC1_BPD_CFG11		0x030
#define HISTB_VC1_BPD_INT_MASK		0x034
#define HISTB_VC1_BPD_STATE		0x040
#define HISTB_VC1_BPD_INT_STATE		0x044
#define HISTB_VC1_BPD_VCTRL_STATE	0x048
#define HISTB_VC1_BPD_OUT0		0x050
#define HISTB_VC1_BPD_OUT1		0x054
#define HISTB_VC1_BPD_STATE_DONE		(1U << 0)
#define HISTB_VC1_BPD_STATE_ERROR	(1U << 1)
#define HISTB_VC1_BPD_STATE_RESET_BUSY	(1U << 2)

enum histb_vc1_result {
	HISTB_VC1_OK = 0,
	HISTB_VC1_NOT_READY = 1,
	HISTB_VC1_NEEDS_BPD = 2,
	HISTB_VC1_INVALID = -1,
	HISTB_VC1_UNSUPPORTED = -2,
	HISTB_VC1_HW_ERROR = -3,
};

enum histb_vc1_profile {
	HISTB_VC1_PROFILE_SIMPLE = 0,
	HISTB_VC1_PROFILE_MAIN = 1,
	HISTB_VC1_PROFILE_ADVANCED = 2,
};

enum histb_vc1_picture_type {
	HISTB_VC1_PICTURE_I = 0,
	HISTB_VC1_PICTURE_P = 1,
	HISTB_VC1_PICTURE_B = 2,
	/* VC1_WritePicMsg maps BI to the VDH value 3. */
	HISTB_VC1_PICTURE_BI_HW = 3,
};

enum histb_vc1_picture_structure {
	HISTB_VC1_PROGRESSIVE = 0,
	HISTB_VC1_FRAME_INTERLACED = 2,
	HISTB_VC1_FIELD_INTERLACED = 3,
};

enum histb_vc1_field_transaction_state {
	HISTB_VC1_FIELD_TXN_EMPTY = 0,
	HISTB_VC1_FIELD_TXN_FIRST_BPD,
	HISTB_VC1_FIELD_TXN_FIRST_COMPLETE,
	HISTB_VC1_FIELD_TXN_SECOND_BPD,
	HISTB_VC1_FIELD_TXN_COMPLETE,
};

enum histb_vc1_bpd_plane {
	HISTB_VC1_BPD_MVTYPEMB = 0,
	HISTB_VC1_BPD_SKIPMB,
	HISTB_VC1_BPD_DIRECTMB,
	HISTB_VC1_BPD_ACPRED,
	HISTB_VC1_BPD_OVERFLAGS,
	HISTB_VC1_BPD_FIELDTX,
	HISTB_VC1_BPD_FORWARDMB,
};

enum histb_vc1_mv_mode {
	/* CV200 vc1.h VC1_MVMODE_EN, not the differently ordered FFmpeg enum. */
	HISTB_VC1_MV_MIXED = 0,
	HISTB_VC1_MV_1MV,
	HISTB_VC1_MV_1MV_HALFPEL,
	HISTB_VC1_MV_1MV_HALFPEL_BILINEAR,
	HISTB_VC1_MV_INTENSITY_COMP,
};

enum histb_vc1_intcomp_field {
	/* CV200 VC1_INTCOMPFIELD_EN values returned by VC1_DecIntCompField. */
	HISTB_VC1_INTENSITY_BOTH_FIELDS = 0,
	HISTB_VC1_INTENSITY_TOP_FIELD,
	HISTB_VC1_INTENSITY_BOTTOM_FIELD,
};

struct histb_vc1_unit {
	__u8 type; /* Annex G 00 00 01 xx value returned by VC1_GetStartCode */
	__u32 start_offset; /* byte offset of 00 00 01 xx in the submitted buffer */
	__u32 payload_offset; /* first byte after the four-byte start code */
	__u32 payload_size; /* bytes up to, but excluding, the next start code */
};

struct histb_vc1_parse_view {
	const __u8 *raw;
	const __u8 *unescaped;
	const __u32 *u2r;
	__u32 raw_size;
	__u32 unescaped_size;
};

/*
 * CV200 VC1_Vfmw_ParseAdvSeqHdr fields.  The parser validates raw profile 3
 * and exports the HAL profile value 2 through profile.
 */
struct histb_vc1_sequence {
	__u8 profile; /* raw PROFILE 3 -> VC1_VFMW_PROFILE_ADVANCED/HAL value 2 */
	__u8 level; /* LEVEL, 3 bits; CV200 rejects reserved values 5..7 */
	__u8 chroma_format; /* CHROMAFORMAT, 2 bits; CV200 supports 4:2:0 value 1 */
	__u8 frame_rate_quant; /* FRMRTQ_POSTPROC, 3 bits */
	__u8 bit_rate_quant; /* BITRTQ_POSTPROC, 5 bits */
	__u8 postproc_flag; /* POSTPROCFLAG */
	__u8 broadcast; /* PULLDOWN/BROADCAST */
	__u8 interlace; /* INTERLACE; progressive pictures still use FCM 0 */
	__u8 tfcntr_flag; /* TFCNTRFLAG */
	__u8 finterp_flag; /* FINTERPFLAG */
	__u8 psf; /* PSF; phase P1 rejects value 1 like the original parser */
	__u8 display_ext; /* DISPLAY_EXT */
	__u8 aspect_ratio_flag; /* ASPECT_RATIO_FLAG */
	__u8 aspect_ratio; /* ASPECT_RATIO, 4 bits */
	__u8 frame_rate_flag; /* FRAMERATE_FLAG */
	__u8 frame_rate_ind; /* FRAMERATEIND */
	__u8 color_format_flag; /* COLOR_FORMAT_FLAG */
	__u8 color_primaries; /* COLOR_PRIM, 8 bits */
	__u8 transfer_characteristics; /* TRANSFER_CHAR, 8 bits */
	__u8 matrix_coefficients; /* MATRIX_COEF, 8 bits */
	__u8 hrd_param_flag; /* HRD_PARAM_FLAG */
	__u8 hrd_bucket_count; /* HRD_NUM_LEAKY_BUCKETS, 5 bits */
	__u8 hrd_rate_exponent; /* BIT_RATE_EXPONENT, 4 bits */
	__u8 hrd_buffer_exponent; /* BUFFER_SIZE_EXPONENT, 4 bits */
	__u16 max_coded_width; /* (MAX_CODED_WIDTH + 1) * 2 */
	__u16 max_coded_height; /* (MAX_CODED_HEIGHT + 1) * 2 */
	__u16 display_width; /* DISPLAY_HORIZ_SIZE + 1, or coded width */
	__u16 display_height; /* DISPLAY_VERT_SIZE + 1, or coded height */
	__u16 aspect_width; /* ASPECT_HORIZ_SIZE + 1 for extended aspect 15 */
	__u16 aspect_height; /* ASPECT_VERT_SIZE + 1 for extended aspect 15 */
	__u32 frame_rate_nr; /* FRAMERATENR or FRAMERATEEXP numerator */
	__u32 frame_rate_dr; /* FRAMERATEDR or fixed 32 denominator */
	__u32 header_bits; /* exact BsGet cursor after VC1_Vfmw_ParseAdvSeqHdr */
};

/* Four-byte WMV3/Annex L Struct C parsed by CV200 Vc1SMPSeqHdr. */
struct histb_vc1_smp_sequence {
	__u8 profile; /* Simple=0 or Main=1 */
	__u8 frame_rate_quant; /* FRMRTQ_POSTPROC, 3 bits */
	__u8 bit_rate_quant; /* BITRTQ_POSTPROC, 5 bits */
	__u8 loopfilter;
	__u8 multires;
	__u8 fast_uv_mc;
	__u8 extended_mv;
	__u8 dquant;
	__u8 variable_transform;
	__u8 overlap;
	__u8 sync_marker;
	__u8 range_reduction;
	__u8 max_b_frames;
	__u8 quantizer_mode;
	__u8 finterp_flag;
	__u16 coded_width; /* RCV/container dimensions, not encoded in Struct C */
	__u16 coded_height;
	__u32 header_bits; /* always 32 for the supported Struct C form */
};

/* CV200 VC1_Vfmw_ParseAdvEntHdr fields retained by picture/BPD setup. */
struct histb_vc1_entry_point {
	__u8 broken_link;
	__u8 closed_entry;
	__u8 panscan_flag;
	__u8 refdist_flag;
	__u8 loopfilter;
	__u8 fast_uv_mc;
	__u8 extended_mv;
	__u8 dquant;
	__u8 variable_transform;
	__u8 overlap;
	__u8 quantizer_mode;
	__u8 coded_size_flag;
	__u8 extended_dmv;
	__u8 range_map_y_flag;
	__u8 range_map_y;
	__u8 range_map_uv_flag;
	__u8 range_map_uv;
	__u16 coded_width;
	__u16 coded_height;
	__u32 header_bits;
};

struct histb_vc1_bpd_layout {
	/* VC1DEC_Init supplies seven 16-byte-aligned planes at 2048-byte steps. */
	__u32 plane_addr[HISTB_VC1_BPD_PLANES];
};

struct histb_vc1_bpd_config {
	__u32 stream_addr; /* bitreader DMA -> BPD CFG1 after 16-byte alignment */
	__u32 bitplane_addr[HISTB_VC1_BPD_PLANES]; /* VFMW BPD buffers -> CFG4-10 */
	__u16 mb_width; /* Advanced syntax coded MB width -> BPD CFG2[15:0] */
	__u16 mb_height; /* Advanced syntax coded MB height -> BPD CFG2[31:16] */
	__u8 bit_offset; /* bitreader offset plus DMA low bits -> BPD CFG0[7:0] */
	__u8 mv_mode_enable; /* syntax/BPD decision -> BPD CFG0[12] */
	__u8 overflags_enable; /* syntax/BPD decision -> BPD CFG0[13] */
	__u8 ptype; /* syntax/BPD picture type -> BPD CFG0[15:14] */
	__u8 picture_structure; /* syntax picstructure -> BPD CFG0[17:16] */
	__u8 profile; /* syntax profile -> BPD CFG0[19:18] */
};

struct histb_vc1_bpd_regs {
	/* CFG0-CFG11 in bitplane.h; CFG11 is the original AXI 0/3/3 value. */
	__u32 cfg[HISTB_VC1_BPD_CFG_WORDS];
};

/*
 * Prefix parsed by VC1_Vfmw_ParseAdvFramePicHdr and the I/P/B child parser up
 * to the exact BPD_Drv call, plus the validated post-BPD picture fields.
 */
struct histb_vc1_parsed_picture {
	__u8 ptype; /* CV200/HAL I=0, P=1, B=2, BI=3 */
	__u8 skipped; /* unary PTYPE value 4; original returns before BPD */
	__u8 fcm; /* decode012 syntax 0/1/2 mapped to CV200 values 0/2/3 */
	__u8 rounding_control; /* RNDCTRL */
	__u8 uv_samp; /* UVSAMP when sequence INTERLACE is set */
	__u8 interp_frame; /* INTERPFRM when FINTERPFLAG is set */
	__u8 bfraction_index; /* Advanced B/field-BI BFRACTION VLC index, 0..20 */
	__s16 bfraction; /* BFRACTION-derived CV200 ScaleFactor, base 256 */
	__u8 repeat_frame; /* RPTFRM for progressive PULLDOWN */
	__u8 top_field_first; /* TFF for INTERLACE+BROADCAST */
	__u8 repeat_first_field; /* RFF for INTERLACE+BROADCAST */
	__u8 pan_scan_present; /* PS_PRESENT when entry PANSCANFLAG is set */
	__u8 pan_scan_windows; /* original NumOfPanScanWindows derivation */
	__u8 fptype; /* field-pair FPTYPE, retained across the FIELD unit */
	__u8 field_index; /* 0 for FRAME/first field, 1 for FIELD/second field */
	__u8 is_second_field; /* field_index copied to CV200 D2[1] */
	__u8 current_parity; /* CurParity: 0 top, 1 bottom */
	__u8 num_ref; /* field-interlace P NUMREF */
	__u8 ref_field; /* field-interlace P REFFIELD when NUMREF is zero */
	__u8 ref_dist; /* field-pair REFDIST, shared by both fields */
	__u8 pqindex; /* PQINDEX, 5 bits and nonzero */
	__u8 pquant; /* CV200 implicit table or explicit PQINDEX */
	__u8 halfqp; /* HALFQP when PQINDEX < 9 */
	__u8 pquantizer; /* explicit-frame selector retained by original syntax */
	__u8 postproc; /* POSTPROC, 2 bits when POSTPROCFLAG is set */
	__u8 res_pic; /* Simple/Main RESPIC: 0 full, 1 half-W, 2 half-H, 3 both */
	__u8 range_reduction; /* Simple/Main RANGEREDFRM */
	__u8 mv_range; /* VC1_DecMvrange result for progressive P/B */
	__u8 dmv_range; /* VC1_DecMvrange result for B EXTENDED_DMV */
	__u8 mv_mode; /* CV200 VC1_MVMODE_EN result */
	__u8 mv_mode2; /* VC1_DecMvmode2 result for intensity compensation */
	__u8 lum_scale; /* LUMSCALE for intensity compensation */
	__u8 lum_shift; /* LUMSHIFT for intensity compensation */
	__u8 intcomp_field; /* field P: both/top/bottom VC1_INTCOMPFIELD_EN */
	__u8 lum_scale2; /* field P BOTH: bottom-field LUMSCALE2 */
	__u8 lum_shift2; /* field P BOTH: bottom-field LUMSHIFT2 */
	__u8 frame_intensity_comp; /* frame-interlace P INTCOMP flag */
	__u8 bpd_mode[HISTB_VC1_BPD_PLANES]; /* BPD OUT1 plane-mode nibbles */
	__u8 condover; /* BPD OUT1[29:28] */
	__u8 forward_mb_raw; /* FORWARDMB mode equals original VC1_RAW value 0 */
	__u8 direct_mb_raw; /* DIRECTMB mode equals original VC1_RAW value 0 */
	__u8 mvtype_mb_raw; /* MVTYPEMB mode equals original VC1_RAW value 0 */
	__u8 fieldtx_raw; /* FIELDTX mode equals original VC1_RAW value 0 */
	__u8 skip_mb_raw; /* SKIPMB mode equals original VC1_RAW value 0 */
	__u8 acpred_raw; /* ACPRED mode equals original VC1_RAW value 0 */
	__u8 overflags_raw; /* OVERFLAGS mode equals original VC1_RAW value 0 */
	__u8 mv_table; /* progressive/frame P MVTAB, 2 bits after BPD */
	__u8 cbp_table; /* P CBPTAB, 2 bits progressive or 3 bits frame */
	__u8 mb_mode_table; /* frame-interlace P MBMODETAB */
	__u8 two_mv_bp_table; /* frame-interlace P 2MVBPTAB */
	__u8 four_mv_bp_table; /* frame-interlace P 4MVBPTAB */
	__u8 four_mv_switch; /* frame-interlace P 4MVSWITCH */
	__u8 dquant_frame; /* VC1_VopDQuant DQUANTFRM */
	__u8 dqprofile; /* VC1_VopDQuant DQPROFILE */
	__u8 dqbi_level; /* VC1_VopDQuant DQBILEVEL */
	__u8 use_alt_qp; /* VC1_VopDQuant UseAltQp */
	__u8 pqdiff; /* VC1_VopDQuant PQDIFF */
	__u8 abs_pq; /* VC1_VopDQuant ABSPQ */
	__u8 altpquant; /* VC1_VopDQuant derived ALTPQUANT */
	__u8 quant_mode; /* VC1_VopDQuant derived VC1_QUANTMODE */
	__u8 ttmbf; /* progressive P TTMBF */
	__u8 ttfrm; /* progressive P TTFRM */
	__u8 trans_ac_frame; /* VC1_DecTransacfrm */
	__u8 trans_ac_frame2; /* second I-picture VC1_DecTransacfrm */
	__u8 trans_dc_table; /* TRANSDCTAB */
	__u8 complete; /* set only after a validated BPD result and post fields */
	__u16 coded_width; /* current resolution after Simple/Main RESPIC */
	__u16 coded_height;
	__u32 bpd_start_u_bits; /* semantic cursor before BPD */
	__u32 bpd_start_raw_bits; /* mapped raw cursor passed to BPD_Drv */
	__u32 header_u_bits; /* final semantic cursor, or SKIP return cursor */
	__u32 header_raw_bits; /* mapped raw cursor consumed by slice hardware */
	__u32 available_raw_bits; /* raw payload bits remaining at BPD call */
	struct histb_vc1_bpd_config bpd_config;
	struct histb_vc1_bpd_regs bpd_regs;
};

/*
 * A field-interlaced frame is one parser transaction and two hardware
 * pictures.  FIELD_TXN_COMPLETE is the only state in which both fields are
 * available for a frame-level reference/display commit.
 */
struct histb_vc1_field_transaction {
	__u8 state;
	__u8 fptype;
	__u8 first_ptype;
	__u8 second_ptype;
	struct histb_vc1_parsed_picture field[2];
};

struct histb_vc1_parsed_slice {
	__u16 address; /* SLICE_ADDR, 9 bits; a macroblock row in CV200 syntax */
	__u8 picture_header_flag; /* PIC_HEADER_FLAG immediately after address */
	__u32 header_u_bits; /* semantic cursor, 10 without repeated header */
	__u32 header_raw_bits; /* mapped raw cursor for slice hardware */
	struct histb_vc1_parsed_picture picture;
};

struct histb_vc1_slice_part {
	__u16 row; /* implicit FRAME row 0, then strictly increasing SLICE_ADDR */
	__u32 payload_dma; /* original Annex G payload DMA, without start code */
	__u32 payload_raw_bits; /* trusted raw unit boundary in bits */
	__u32 data_raw_bit; /* final mapped raw picture/slice header cursor */
	__u32 data_u_bit; /* semantic cursor in the unescaped payload */
};

struct histb_vc1_up_report {
	/* Original report builder consumes only D1/D2 low 16-bit MB ranges. */
	__u32 d[4];
};

/*
 * Evidence chain for every member below:
 *
 *  - Android OMX decoder_vfmw.c selects STD_VC1 for Advanced Profile.
 *  - CV200 syntax/vc1.h and VC1_WritePicMsg populate the identically named
 *    VC1_DEC_PARAM_S member, including FSP-derived frame and reference DMA.
 *  - HiVDHV4R3C1/vdm_hal_v4r3c1_vc1.h and CfgDnMsg pack that member into
 *    the D word and bit range named in each comment.
 *
 * This is deliberately the original HAL-boundary contract.  It is not a
 * substitute VC-1 parser and it does not synthesize fields absent from VFMW.
 */
struct histb_vc1_picture {
	__u8 ptype; /* syntax ptype -> DEC_PARAM ptype -> D0[1:0] */
	__u8 profile; /* syntax profile Simple/Main/Advanced -> D0[5:4] */
	__u8 fcm; /* syntax fcm -> DEC_PARAM fcm -> D0[15:14] */
	__u8 loopfilter; /* syntax LOOPFILTER -> DEC_PARAM loopfilter -> D2[0] */
	__u8 is_second_field; /* syntax field state -> issecondfld -> D2[1] */
	__u8 current_parity; /* syntax field parity -> curparity -> D2[2] */
	__u8 num_ref; /* syntax NUMREF -> numref -> D2[3] */
	__u8 forward_fcm; /* forward FSP picture fcm -> fwd_fcm -> D2[5:4] */
	__u8 backward_fcm; /* backward FSP picture fcm -> bwd_fcm -> D2[7:6] */
	__u8 rounding_control; /* syntax RNDCTRL -> rndctrl -> D3[0] */
	__u8 fast_uv_mc; /* entry FASTUVMC -> fastuvmc -> D3[1] */
	__u8 overlap; /* entry OVERLAP -> overlap -> D3[2] */
	__u8 condover; /* BPD/syntax CONDOVER -> condover -> D3[5:4] */
	__u8 pquant; /* picture PQUANT -> pquant -> D4[4:0] */
	__u8 pqindex; /* picture PQINDEX -> pqindex -> D4[12:8] */
	__u8 altpquant; /* picture ALTPQUANT -> altpquant -> D4[20:16] */
	__u8 halfqp; /* picture HALFQP -> halfqp -> D4[24] */
	__u8 b_uniform; /* picture BUNIFORM -> buniform -> D5[0] */
	__u8 use_alt_qp; /* picture use-alt flag -> usealtqp -> D5[1] */
	__u8 dquant; /* entry DQUANT -> dquant -> D5[3:2] */
	__u8 dqprofile; /* picture DQPROFILE -> dqprofile -> D5[9:8] */
	__u8 dqbi_level; /* picture DQBILEVEL -> dqbilevel -> D5[12] */
	__u8 dquant_frame; /* picture DQUANTFRM -> dquantfrm -> D5[14] */
	__u8 quant_mode; /* sequence quantizer mode -> quantmode -> D5[19:16] */
	__u8 mv_mode; /* picture MVMODE -> mvmode -> D6[2:0] */
	__u8 mv_mode2; /* picture MVMODE2 -> mvmode2 -> D6[5:4] */
	__u8 current_halfpel; /* current FSP halfpel -> curishalfpel -> D7[0] */
	__u8 colocated_halfpel; /* colocated FSP halfpel -> colishalfpel -> D7[1] */
	__u8 mv_range; /* picture MVRANGE -> mvrange -> D7[3:2] */
	__u8 ref_dist; /* picture REFDIST -> refdist -> D7[8:4] */
	__u8 dmv_range; /* picture DMVRANGE -> dmvrange -> D7[13:12] */
	__u8 ref_field; /* picture REFFIELD -> reffiled -> D7[14] */
	__u8 trans_dc_table; /* picture TRANSDCTAB -> transdctab -> D8[0] */
	__u8 variable_transform; /* entry VSTRANSFORM -> vstransform -> D8[1] */
	__u8 ttmbf; /* picture TTMBF -> ttmbf -> D8[2] */
	__u8 trans_ac_frame; /* picture TRANSACFRM -> transacfrm -> D8[5:4] */
	__u8 trans_ac_frame2; /* picture TRANSACFRM2 -> transacfrm2 -> D8[7:6] */
	__u8 ttfrm; /* picture TTFRM -> ttfrm -> D8[9:8] */
	__u8 forward_mb_raw; /* BPD FORWARDMB raw flag -> DEC_PARAM -> D9[0] */
	__u8 direct_mb_raw; /* BPD DIRECTMB raw flag -> DEC_PARAM -> D9[1] */
	__u8 mvtype_mb_raw; /* BPD MVTYPEMB raw flag -> DEC_PARAM -> D9[2] */
	__u8 fieldtx_raw; /* BPD FIELDTX raw flag -> DEC_PARAM -> D9[3] */
	__u8 skip_mb_raw; /* BPD SKIPMB raw flag -> DEC_PARAM -> D9[4] */
	__u8 acpred_raw; /* BPD ACPRED raw flag -> DEC_PARAM -> D9[5] */
	__u8 overflags_raw; /* BPD OVERFLAGS raw flag -> DEC_PARAM -> D9[6] */
	__u8 mv_table; /* picture MVTAB -> mvtab -> D10[2:0] */
	__u8 cbp_table; /* picture CBPTAB -> cbptab -> D10[10:8] */
	__u8 b_fraction; /* picture BFRACTION -> bfraction -> D10[22:16] */
	__u8 mb_mode_table; /* picture MBMODETAB -> mbmodetab -> D11[2:0] */
	__u8 two_mv_bp_table; /* picture 2MVBPTAB -> twomvbptab -> D11[5:4] */
	__u8 four_mv_bp_table; /* picture 4MVBPTAB -> fourmvbptab -> D11[9:8] */
	__u8 four_mv_switch; /* picture 4MVSWITCH -> fourmvswtich -> D11[12] */
	__u8 range_map_y_flag; /* entry RANGEMAPYFLAG -> DEC_PARAM -> D15[20] */
	__u8 range_map_y; /* entry RANGEMAPY -> DEC_PARAM -> D15[23:21] */
	__u8 range_map_uv_flag; /* entry RANGEMAPUVFLAG -> DEC_PARAM -> D15[24] */
	__u8 range_map_uv; /* entry RANGEMAPUV -> DEC_PARAM -> D15[27:25] */
	__u8 range_reduction; /* sequence RANGERED -> RANGEREDFRM -> D28[20] */
	__u8 range_reduction0; /* forward FSP range flag -> RANGEREDFRM0 -> D28[21] */
	__u8 range_reduction1; /* backward FSP range flag -> RANGEREDFRM1 -> D28[22] */
	__u8 postproc; /* sequence/entry POSTPROC -> PostCresent -> D28[24] */
	__u8 codec_version; /* OMX codec version -> CodecVersion -> D28[27:25] */
	__u8 picture_structure; /* syntax picstructure -> BPD CFG0[17:16] */
	__u16 mb_width; /* FSP coded MB width; WritePicMsg minus one -> D1[7:0] */
	__u16 mb_height; /* FSP coded MB height; WritePicMsg minus one -> D1[23:16] */
	__u16 display_width; /* FSP display width -> DispPicWidth -> D27[15:0] */
	__u16 display_height; /* FSP display height -> DispPicHeight -> D27[31:16] */
	__u16 total_slices; /* syntax SlcNum -> totalslicenum -> D15[15:0] */
	__u16 bpd_mb_width; /* current coded picture width used by BPD and D29 */
	__u32 scale_factor; /* field syntax ScaleFactor -> DEC_PARAM -> D12 */
	__u32 forward_ref_dist; /* field/reference syntax FRFD -> DEC_PARAM -> D13 */
	__u32 backward_ref_dist; /* field/reference syntax BRFD -> DEC_PARAM -> D14 */
	__u32 current_picture_addr; /* current FSP luma DMA -> DEC_PARAM -> D16 */
	__u32 forward_ref_addr; /* forward FSP luma DMA -> DEC_PARAM -> D17 */
	__u32 backward_ref_addr; /* backward FSP luma DMA -> DEC_PARAM -> D18 */
	__u32 current_colmb_addr; /* current FSP colmb DMA -> DEC_PARAM -> D19 */
	__u32 backward_colmb_addr; /* backward FSP colmb DMA -> DEC_PARAM -> D20 */
	__u32 sed_top_addr; /* VFMW Vahb SED top DMA -> hardware memory -> D21 */
	__u32 pmv_top_addr; /* VFMW Vahb PMV top DMA -> hardware memory -> D22 */
	__u32 itrans_top_addr; /* VFMW Vahb ITRANS top DMA -> hardware memory -> D23 */
	__u32 dblk_top_addr; /* VFMW Vahb DBLK top DMA -> hardware memory -> D24 */
	__u32 intensity_table_addr; /* syntax intensity table DMA -> DEC_PARAM -> D26 */
	__u32 bpd_stride; /* stride derived from bpd_mb_width -> D29 */
	__u32 bitplane_addr[HISTB_VC1_BPD_PLANES]; /* BPD planes -> D30-D36 */
	__u32 slice_info_addr; /* VFMW slice-message DMA -> SliceInfoPhyAddr -> D63 */
};

struct histb_vc1_pic_msg {
	__u32 d[HISTB_VC1_PIC_MSG_WORDS];
};

struct histb_vc1_slice_fragment {
	__u32 dma_addr; /* VC1_SLCSTREAM Phy_addr{,2} -> slice D1/D3 offset */
	__u32 bit_len; /* VC1_SLCSTREAM Len{,2} -> slice D0/D2[24:0] */
	__u8 bit_offset; /* VC1_SLCSTREAM BitOffset{,2} -> D0/D2[31:25] */
};

struct histb_vc1_slice {
	struct histb_vc1_slice_fragment fragment[HISTB_VC1_SLICE_FRAGMENTS];
	__u16 start_mb; /* CfgSliceMsg running start MB -> slice D4[15:0] */
	__u16 end_mb; /* VC1_SLCSTREAM SlcEndMbn -> slice D4[31:16] */
};

struct histb_vc1_slice_msg {
	/* CfgSliceMsg uses a 64-DWORD/256-byte slot and D63 link. */
	__u32 d[HISTB_VC1_SLICE_MSG_WORDS];
};

struct histb_vc1_bpd_result {
	__u32 eaten_bits; /* BPD OUT0 semantic bits, excluding skipped EPBs */
	__u8 mode[HISTB_VC1_BPD_PLANES]; /* OUT1 nibbles in plane enum order */
	__u8 condover; /* BPD OUT1[29:28] */
};

/* VC1 Table 40 valid Advanced Profile B fractions, scaled to base 256. */
struct histb_vc1_bfraction {
	__u8 index;
	__s16 scale;
};

struct histb_vc1_intensity_map {
	/* BSP layout per reference: top Y/C followed by bottom Y/C. */
	__u8 value[HISTB_VC1_INTENSITY_PARITIES]
		  [HISTB_VC1_INTENSITY_COMPONENTS]
		  [HISTB_VC1_INTENSITY_VALUES];
};

struct histb_vc1_intensity_work {
	struct histb_vc1_intensity_map ref[HISTB_VC1_INTENSITY_REFS];
};

struct histb_vc1_display_state {
	__u8 pending;
};

struct histb_vc1_display_plan {
	__u8 release_pending;
	__u8 release_current;
	__u8 hold_current;
	__u8 current_before_pending;
	__u8 pending_last;
	__u8 current_last;
};

int histb_vc1_build_pic_msg(const struct histb_vc1_picture *picture,
			     struct histb_vc1_pic_msg *msg);
int histb_vc1_build_slice_messages(const struct histb_vc1_slice *slices,
				    __u32 slice_count, __u16 mb_width,
				    __u16 mb_height, __u32 message_dma,
				    struct histb_vc1_slice_msg *messages,
				    __u32 message_capacity,
				    __u32 *stream_base);
int histb_vc1_build_bpd_regs(const struct histb_vc1_bpd_config *config,
			      struct histb_vc1_bpd_regs *regs);
int histb_vc1_parse_bpd_result(__u32 state, __u32 out0, __u32 out1,
			       __u32 available_bits,
			       struct histb_vc1_bpd_result *result);
int histb_vc1_next_unit(const __u8 *data, __u32 size, __u32 *cursor,
			 struct histb_vc1_unit *unit);
void histb_vc1_identity_view(const __u8 *data, __u32 size,
			      struct histb_vc1_parse_view *view);
int histb_vc1_unescape_unit(const __u8 *raw, __u32 raw_size,
			     __u8 *unescaped, __u32 unescaped_capacity,
			     __u32 *u2r, __u32 u2r_capacity,
			     struct histb_vc1_parse_view *view);
int histb_vc1_u_to_raw_bit(const struct histb_vc1_parse_view *view,
			    __u32 semantic_bit, __u32 *raw_bit);
int histb_vc1_raw_to_u_bit(const struct histb_vc1_parse_view *view,
			    __u32 raw_bit, __u32 *semantic_bit);
int histb_vc1_parse_sequence(const __u8 *data, __u32 size,
			      struct histb_vc1_sequence *sequence);
int histb_vc1_parse_sequence_view(const struct histb_vc1_parse_view *view,
				   struct histb_vc1_sequence *sequence);
int histb_vc1_parse_entry_point(const __u8 *data, __u32 size,
				 const struct histb_vc1_sequence *sequence,
				 struct histb_vc1_entry_point *entry);
int histb_vc1_parse_entry_point_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_sequence *sequence,
				struct histb_vc1_entry_point *entry);
int histb_vc1_validate_entry_range_transition(
			const struct histb_vc1_entry_point *previous,
			const struct histb_vc1_entry_point *current_entry);
int histb_vc1_range_map_nv12(__u8 *data, __u32 stride, __u16 width,
			      __u16 height,
			      const struct histb_vc1_entry_point *entry);
int histb_vc1_parse_smp_sequence(const __u8 *struct_c, __u32 size,
				 __u16 coded_width, __u16 coded_height,
				 struct histb_vc1_smp_sequence *sequence);
int histb_vc1_smp_respic_dimensions(
				 const struct histb_vc1_smp_sequence *sequence,
				 __u8 res_pic, __u16 *coded_width,
				 __u16 *coded_height);
int histb_vc1_parse_smp_picture(
				 const __u8 *data, __u32 size, __u32 payload_dma,
				 const struct histb_vc1_smp_sequence *sequence,
				 __u8 committed_rounding, __u8 committed_res_pic,
				 const struct histb_vc1_bpd_layout *layout,
				 struct histb_vc1_parsed_picture *picture);
int histb_vc1_resume_smp_picture(
				const __u8 *data, __u32 size,
				const struct histb_vc1_smp_sequence *sequence,
				const struct histb_vc1_bpd_result *bpd_result,
				struct histb_vc1_parsed_picture *picture);
int histb_vc1_parse_progressive_picture(
			const __u8 *data, __u32 size, __u32 start_bit,
			__u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_picture *picture);
int histb_vc1_parse_progressive_picture_view(
			const struct histb_vc1_parse_view *view,
			__u32 start_u_bit, __u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_picture *picture);
int histb_vc1_parse_progressive_b_picture_view(
			const struct histb_vc1_parse_view *view,
			__u32 start_u_bit, __u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_picture *picture);
void histb_vc1_field_transaction_reset(
			struct histb_vc1_field_transaction *transaction);
int histb_vc1_parse_first_field_picture_view(
			const struct histb_vc1_parse_view *view,
			__u32 start_u_bit, __u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_field_transaction *transaction);
int histb_vc1_parse_second_field_picture_view(
			const struct histb_vc1_parse_view *view,
			__u32 start_u_bit, __u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_field_transaction *transaction);
int histb_vc1_resume_field_picture_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_field_transaction *transaction);
int histb_vc1_parse_field_slice_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_field_transaction *transaction,
			__u8 field_index,
			struct histb_vc1_parsed_slice *slice);
int histb_vc1_decode_bfraction(__u8 prefix, __u8 suffix,
			       struct histb_vc1_bfraction *fraction);
void histb_vc1_intensity_init(struct histb_vc1_intensity_map *map);
int histb_vc1_intensity_compose(const struct histb_vc1_intensity_map *input,
				__u8 lum_scale, __u8 lum_shift,
				struct histb_vc1_intensity_map *output);
int histb_vc1_intensity_compose_parity(
				const struct histb_vc1_intensity_map *input,
				__u8 parity, __u8 lum_scale, __u8 lum_shift,
				struct histb_vc1_intensity_map *output);
int histb_vc1_pack_intensity_work(
			__u8 ptype,
			const struct histb_vc1_intensity_map *earlier,
			const struct histb_vc1_intensity_map *latest,
			const struct histb_vc1_intensity_map *transformed_latest,
			struct histb_vc1_intensity_work *work);
int histb_vc1_pack_field_intensity_work(
			__u8 ptype,
			const struct histb_vc1_intensity_map *earlier,
			const struct histb_vc1_intensity_map *latest,
			const struct histb_vc1_intensity_map *current_map,
			struct histb_vc1_intensity_work *work);
int histb_vc1_effective_halfpel(__u8 mv_mode, __u8 mv_mode2,
				__u8 *halfpel);
int histb_vc1_picture_halfpel(const struct histb_vc1_parsed_picture *picture,
			      __u8 reference_halfpel, __u8 *halfpel);
int histb_vc1_parse_progressive_slice(
			const __u8 *data, __u32 size, __u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_slice *slice);
int histb_vc1_parse_progressive_slice_view(
			const struct histb_vc1_parse_view *view,
			__u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_slice *slice);
int histb_vc1_parse_progressive_b_slice_view(
			const struct histb_vc1_parse_view *view,
			__u32 payload_dma,
			const struct histb_vc1_sequence *sequence,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_layout *layout,
			struct histb_vc1_parsed_slice *slice);
int histb_vc1_resume_progressive_picture(
			const __u8 *data, __u32 size,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_picture *picture);
int histb_vc1_resume_progressive_picture_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_picture *picture);
int histb_vc1_resume_progressive_b_picture_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_picture *picture);
int histb_vc1_plan_display(struct histb_vc1_display_state *state,
			   __u8 ptype, __u8 first_anchor, __u8 draining,
			   struct histb_vc1_display_plan *plan);
int histb_vc1_plan_drain(struct histb_vc1_display_state *state,
			 struct histb_vc1_display_plan *plan);
int histb_vc1_resume_progressive_slice(
			const __u8 *data, __u32 size,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_slice *slice);
int histb_vc1_resume_progressive_slice_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_slice *slice);
int histb_vc1_resume_progressive_b_slice_view(
			const struct histb_vc1_parse_view *view,
			const struct histb_vc1_entry_point *entry,
			const struct histb_vc1_bpd_result *bpd_result,
			struct histb_vc1_parsed_slice *slice);
int histb_vc1_finalize_slices(const struct histb_vc1_slice_part *parts,
			       __u32 count, __u16 mb_width, __u16 mb_height,
			       struct histb_vc1_slice *slices,
			       __u32 slice_capacity);
int histb_vc1_validate_up_reports(__u32 state, __u32 smmu_state_secure,
				  __u32 smmu_state_nonsecure,
				  __u32 total_mbs,
				  const struct histb_vc1_up_report *reports,
				  __u32 report_capacity);

#endif
