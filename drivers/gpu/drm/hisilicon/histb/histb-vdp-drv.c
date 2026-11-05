// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM driver for the HiSilicon Hi3798CV200/MV310 display processors.
 *
 * Register programming follows HiSTBLinuxV100R005C00SPC060
 * source/msp/drv/vo/vdp_v4_0/hal/{98cv200,98mv310}/ and the MV310
 * source/common/drv/include/hi3798mv310/hi_reg_vdp.h register map.
 * The vendor HAL files credit Hisilicon Technologies Co., Ltd.
 */

#include <linux/align.h>
#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-buf.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/seq_file.h>
#include <linux/overflow.h>
#include <linux/slab.h>

#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/bridge/histb-hdmi.h>
#include <drm/drm_blend.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_fourcc.h>
#include <drm/histb_drm.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_prime.h>
#include <drm/drm_print.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#include "histb-vdp-zme.h"
#include "histb-vdp-hdr.h"

#define HISTB_VDP_VOINTSTA		0x0004
#define HISTB_VDP_VOMSKINTSTA		0x0008
#define HISTB_VDP_VOINTMSK		0x000c
#define HISTB_VDP_VO_MUX		0x0100
#define HISTB_VDP_VOCTRL		0x0000
#define HISTB_VDP_VOMIDORDER		0x0028
#define HISTB_VDP_VOLOWPOWER_CTRL	0x002c
#define HISTB_VDP_VOAXICTRL		0x0030
#define HISTB_VDP_VOAXICTRL1		0x0034
#define HISTB_VDP_VOMASTERSEL		0x0038
#define HISTB_VDP_VERSION1		0x0020
#define HISTB_VDP_VERSION2		0x0024

#define HISTB_VDP_VDM_CTRL		0x3400
#define HISTB_VDP_VDM_YUV2RGB_CTRL	0x3500
#define HISTB_VDP_VDM_YUV2RGB_COEF00	0x3504
#define HISTB_VDP_VDM_YUV2RGB_COEF01	0x3508
#define HISTB_VDP_VDM_YUV2RGB_COEF02	0x350c
#define HISTB_VDP_VDM_YUV2RGB_COEF03	0x3510
#define HISTB_VDP_VDM_YUV2RGB_COEF04	0x3514
#define HISTB_VDP_VDM_YUV2RGB_COEF05	0x3518
#define HISTB_VDP_VDM_YUV2RGB_COEF06	0x351c
#define HISTB_VDP_VDM_YUV2RGB_COEF07	0x3520
#define HISTB_VDP_VDM_YUV2RGB_COEF08	0x3524
#define HISTB_VDP_VDM_YUV2RGB_SCALE2P	0x354c
#define HISTB_VDP_VDM_YUV2RGB_IN_DC00	0x3550
#define HISTB_VDP_VDM_YUV2RGB_IN_DC01	0x3554
#define HISTB_VDP_VDM_YUV2RGB_IN_DC02	0x3558
#define HISTB_VDP_VDM_YUV2RGB_OUT_DC00	0x355c
#define HISTB_VDP_VDM_YUV2RGB_OUT_DC01	0x3560
#define HISTB_VDP_VDM_YUV2RGB_OUT_DC02	0x3564

#define HISTB_VDP_GDM_CTRL		0x3700
#define HISTB_VDP_GDM_RGB2LMS_CTRL	0x3800
#define HISTB_VDP_CVM_CTRL		0x3900
#define HISTB_VDP_CVM1_RGB2YUV_CTRL	0x3d00
#define HISTB_VDP_CVM2_RGB2YUV_CTRL	0x3e00
#define HISTB_VDP_CVM2_RGB2YUV_COEF0	0x3e04
#define HISTB_VDP_CVM2_RGB2YUV_SCALE2P	0x3e28
#define HISTB_VDP_CVM2_RGB2YUV_MIN	0x3e2c
#define HISTB_VDP_CVM2_RGB2YUV_MAX	0x3e30
#define HISTB_VDP_CVM2_RGB2YUV_OUT_DC0	0x3e34
#define HISTB_VDP_CVM2_RGB2YUV_OUT_DC1	0x3e38
#define HISTB_VDP_CVM2_RGB2YUV_OUT_DC2	0x3e3c
#define HISTB_VDP_HDR_CTRL		0x3f80
#define HISTB_VDP_HDR_UPD		0x3f84

#define HISTB_VDP_G0_CTRL		0x7000
#define HISTB_VDP_G0_UPD		0x7004
#define HISTB_VDP_G0_ADDR		0x7010
#define HISTB_VDP_G0_NADDR		0x7018
#define HISTB_VDP_G0_STRIDE		0x701c
#define HISTB_VDP_G0_IRESO		0x7020
#define HISTB_VDP_G0_SFPOS		0x7024
#define HISTB_VDP_G0_SMMU_BYPASS0	0x7028
#define HISTB_VDP_G0_CBMPARA		0x7030
#define HISTB_VDP_G0_DCMP_ADDR		0x7050
#define HISTB_VDP_G0_DCMP_NADDR		0x7054
#define HISTB_VDP_G0_DCMP_OFFSET	0x7058
#define HISTB_VDP_G0_DFPOS		0x7080
#define HISTB_VDP_G0_DLPOS		0x7084
#define HISTB_VDP_G0_VFPOS		0x7088
#define HISTB_VDP_G0_VLPOS		0x708c
#define HISTB_VDP_G0_ALPHA		0x7094
#define HISTB_VDP_G0_CSC_IDC		0x70c0

#define HISTB_VDP_GP0_CTRL		0x8000
#define HISTB_VDP_GP0_UPD		0x8004
#define HISTB_VDP_GP0_ORESO		0x8008
#define HISTB_VDP_GP0_IRESO		0x800c
#define HISTB_VDP_GP0_HCOEFAD		0x8010
#define HISTB_VDP_GP0_VCOEFAD		0x8014
#define HISTB_VDP_GP0_PARAUP		0x8018
#define HISTB_VDP_GP0_GALPHA		0x8020
#define HISTB_VDP_GP0_DFPOS		0x8100
#define HISTB_VDP_GP0_DLPOS		0x8104
#define HISTB_VDP_GP0_VFPOS		0x8108
#define HISTB_VDP_GP0_VLPOS		0x810c
#define HISTB_VDP_GP0_CSC_IDC		0x8120
#define HISTB_VDP_GP0_ZME_HSP		0x8140
#define HISTB_VDP_GP0_ZME_HLOFFSET	0x8144
#define HISTB_VDP_GP0_ZME_HCOFFSET	0x8148
#define HISTB_VDP_GP0_ZME_VSP		0x814c
#define HISTB_VDP_GP0_ZME_VSR		0x8150
#define HISTB_VDP_GP0_ZME_VOFFSET		0x8154
#define HISTB_VDP_GP0_ZME_LTICTRL		0x8160
#define HISTB_VDP_GP0_ZME_CTICTRL		0x8180

#define HISTB_VDP_MIXG0_BKALPHA		0xb204
#define HISTB_VDP_MIXG0_MIX		0xb208
#define HISTB_VDP_MIXV0_MIX		0xb008
#define HISTB_VDP_CBM_BKG1		0xb400
#define HISTB_VDP_CBM_MIX1		0xb408

#define HISTB_VDP_V0_CTRL		0x0800
#define HISTB_VDP_V0_UPD		0x0804
#define HISTB_VDP_V0_PRERD		0x0820
#define HISTB_VDP_V0_IRESO		0x0828
#define HISTB_VDP_V0_ORESO		0x082c
#define HISTB_VDP_V0_CBMPARA		0x0838
#define HISTB_VDP_V0_CPOS		0x0844
#define HISTB_VDP_V0_DFPOS		0x0860
#define HISTB_VDP_V0_DLPOS		0x0864
#define HISTB_VDP_V0_VFPOS		0x0868
#define HISTB_VDP_V0_VLPOS		0x086c
#define HISTB_VDP_V0_BK			0x0870
#define HISTB_VDP_V0_ALPHA		0x0874
#define HISTB_VDP_V0_MUTE_BK		0x0878
#define HISTB_VDP_V0_CSC_IDC		0x0880
#define HISTB_VDP_V0_NADDR		0x0e00
#define HISTB_VDP_V0_NCADDR		0x0e04
#define HISTB_VDP_V0_TILE_STRIDE	0x0e08
#define HISTB_VDP_V0_LADDROFFSET	0x0e40
#define HISTB_VDP_V0_CADDROFFSET	0x0e44
#define HISTB_VDP_V0_TILE_LOFFSET	0x0e48
#define HISTB_VDP_V0_TILE_COFFSET	0x0e4c
#define HISTB_VDP_V0_MULTI_MODE		0x0e30
#define HISTB_VDP_V0_16REGIONENL	0x0f00
#define HISTB_VDP_V0_SMMU_BYPASS0	0x09a0
#define HISTB_VDP_V0_SMMU_BYPASS1	0x09a4
#define HISTB_VDP_V0_SMMU_BYPASS2	0x09a8
#define HISTB_VDP_V0_SMMU_BYPASS3	0x09ac
#define HISTB_VDP_V0_P0RESO		0x0a00
#define HISTB_VDP_V0_P0LADDR		0x0a04
#define HISTB_VDP_V0_P0CADDR		0x0a08
#define HISTB_VDP_V0_P0STRIDE		0x0a0c
#define HISTB_VDP_V0_P0VFPOS		0x0a10
#define HISTB_VDP_V0_P0VLPOS		0x0a14
#define HISTB_VDP_V0_P0CTRL		0x0a18
#define HISTB_VDP_V0_PARAUP		0x0840
#define HISTB_VDP_V0_HLCOEFAD		0x0850
#define HISTB_VDP_V0_HCCOEFAD		0x0854
#define HISTB_VDP_V0_VLCOEFAD		0x0858
#define HISTB_VDP_V0_VCCOEFAD		0x085c
#define HISTB_VDP_V0_HSP		0x08c0
#define HISTB_VDP_V0_HLOFFSET		0x08c4
#define HISTB_VDP_V0_HCOFFSET		0x08c8
#define HISTB_VDP_V0_VSP		0x08d8
#define HISTB_VDP_V0_VSR		0x08dc
#define HISTB_VDP_V0_VOFFSET		0x08e0
#define HISTB_VDP_V0_VBOFFSET		0x08e4

#define HISTB_VDP_VDM_RGB2LMS_CTRL	0x3600
#define HISTB_VDP_VDM_RGB2LMS_COEF	0x3604
#define HISTB_VDP_VDM_RGB2LMS_SCALE2P	0x3628
#define HISTB_VDP_VDM_RGB2LMS_MIN		0x362c
#define HISTB_VDP_VDM_RGB2LMS_MAX		0x3630
#define HISTB_VDP_CVM_CTRL		0x3900
#define HISTB_VDP_CVM1_LMS2IPT_CTRL	0x3a00
#define HISTB_VDP_CVM1_LMS2IPT_COEF	0x3a04
#define HISTB_VDP_CVM1_LMS2IPT_SCALE2P	0x3a28
#define HISTB_VDP_CVM1_LMS2IPT_MIN	0x3a2c
#define HISTB_VDP_CVM1_LMS2IPT_MAX	0x3a30
#define HISTB_VDP_CVM1_LMS2IPT_OUT_DC	0x3a34

#define HISTB_VDP_VP0_CTRL		0x6000
#define HISTB_VDP_VP0_UPD		0x6004
#define HISTB_VDP_VP0_IRESO		0x6008
#define HISTB_VDP_VP0_GALPHA		0x6020
#define HISTB_VDP_VP0_DFPOS		0x6040
#define HISTB_VDP_VP0_DLPOS		0x6044
#define HISTB_VDP_VP0_VFPOS		0x6048
#define HISTB_VDP_VP0_VLPOS		0x604c
#define HISTB_VDP_VP0_ALPHA		0x6054

#define HISTB_VDP_DHD0_CTRL		0xc000
#define HISTB_VDP_DHD0_VSYNC		0xc004
#define HISTB_VDP_DHD0_HSYNC1		0xc008
#define HISTB_VDP_DHD0_HSYNC2		0xc00c
#define HISTB_VDP_DHD0_VPLUS		0xc010
#define HISTB_VDP_DHD0_PWR		0xc014
#define HISTB_VDP_DHD0_VTTHD		0xc01c
#define HISTB_VDP_DHD0_SYNC_INV		0xc020
#define HISTB_VDP_DHD0_DATA_SEL		0xc02c
#define HISTB_VDP_DHD0_CSC_IDC		0xc040
#define HISTB_VDP_DHD0_HDMI_CLIP_L	0xc090
#define HISTB_VDP_DHD0_HDMI_CLIP_H	0xc094

#define HISTB_VDP_DHD0_VBLANK		BIT(0)
#define HISTB_VDP_DHD0_UNDERFLOW	BIT(3)
#define HISTB_VDP_MASTER_ERRORS		GENMASK(31, 30)
#define HISTB_VDP_ERROR_IRQS		(HISTB_VDP_DHD0_UNDERFLOW | HISTB_VDP_MASTER_ERRORS)

#define HISTB_VDP_G0_IFMT		GENMASK(7, 0)
#define HISTB_VDP_G0_BITEXT		GENMASK(9, 8)
#define HISTB_VDP_G0_REQ_CTRL		GENMASK(15, 14)
#define HISTB_VDP_G0_FETCH_POLICY	GENMASK(18, 14)
#define HISTB_VDP_G0_READ_PROGRESSIVE	BIT(26)
#define HISTB_VDP_G0_UPDATE_PROGRESSIVE	BIT(27)
#define HISTB_VDP_G0_NONSECURE		BIT(30)
#define HISTB_VDP_G0_ENABLE		BIT(31)
#define HISTB_VDP_G0_PIXEL_ALPHA	BIT(12)
#define HISTB_VDP_G0_STRIDE_MASK	GENMASK(15, 0)
#define HISTB_VDP_G0_STRIDE_UNIT	16

#define HISTB_VDP_GP0_READ_PROGRESSIVE	BIT(31)
#define HISTB_VDP_GP0_CSC_ENABLE	BIT(22)
#define HISTB_VDP_GP0_MUX1_SEL		GENMASK(1, 0)
#define HISTB_VDP_GP0_MUX2_SEL		GENMASK(5, 4)
#define HISTB_VDP_GP0_AOUT_SEL		GENMASK(9, 8)
#define HISTB_VDP_GP0_BOUT_SEL		GENMASK(13, 12)
/* CV200 Linux HIFB/PQ channel A: ZME -> CSC, mux1=3, mux2=0, A=2, B=1. */
#define HISTB_VDP_GP0_ROUTE_ZME_CSC	(FIELD_PREP(HISTB_VDP_GP0_MUX1_SEL, 3) | \
					 FIELD_PREP(HISTB_VDP_GP0_MUX2_SEL, 0) | \
					 FIELD_PREP(HISTB_VDP_GP0_AOUT_SEL, 2) | \
					 FIELD_PREP(HISTB_VDP_GP0_BOUT_SEL, 1))
#define HISTB_VDP_GP0_ZME_H_ENABLE	BIT(31)
#define HISTB_VDP_GP0_ZME_V_ENABLE	BIT(31)
#define HISTB_VDP_GP0_ZME_H_RATIO	GENMASK(23, 0)
#define HISTB_VDP_GP0_ZME_VH		BIT(24)
#define HISTB_VDP_GP0_ZME_H_FIR		(BIT(25) | BIT(26))
#define HISTB_VDP_GP0_ZME_V_FIR		(BIT(23) | BIT(24))
#define HISTB_VDP_GP0_ZME_MID		GENMASK(30, 28)
#define HISTB_VDP_GP0_ZME_V_RATIO	GENMASK(15, 0)

#define HISTB_VDP_V0_IFMT		GENMASK(3, 0)
#define HISTB_VDP_V0_REQ_CTRL		GENMASK(7, 5)
#define HISTB_VDP_V0_NONSECURE		BIT(10)
#define HISTB_VDP_V0_UV_ORDER		BIT(11)
#define HISTB_VDP_V0_CHM_RMODE		GENMASK(13, 12)
#define HISTB_VDP_V0_LM_RMODE		GENMASK(15, 14)
#define HISTB_VDP_V0_IFIR_MODE		GENMASK(19, 18)
#define HISTB_VDP_V0_FLIP		BIT(26)
#define HISTB_VDP_V0_MUTE		BIT(27)
#define HISTB_VDP_V0_DATA_WIDTH		GENMASK(29, 28)
#define HISTB_VDP_V0_SURFACE_C_EN	BIT(30)
#define HISTB_VDP_V0_SURFACE_EN		BIT(31)
#define HISTB_VDP_V0_IFMT_SP_420	3
#define HISTB_VDP_V0_IFMT_SP_TILE_64	7
#define HISTB_VDP_V0_RMODE_PROGRESSIVE	1
#define HISTB_VDP_V0_STRIDE_UNIT	16

#define HISTB_VDP_GDM_ENABLES		GENMASK(3, 0)
#define HISTB_VDP_GDM_RGB2LMS_ENABLE	BIT(0)
#define HISTB_VDP_CVM_ENABLE		BIT(0)
#define HISTB_VDP_CVM_SELECT		BIT(1)
#define HISTB_VDP_CVM_PIPE_ENABLES	GENMASK(5, 0)
#define HISTB_VDP_CVM2_RGB2YUV_ENABLE	BIT(0)
#define HISTB_VDP_CVM2_CSC_PRECISION	15
#define HISTB_VDP_CVM2_CSC_SCALE_ENABLE	BIT(5)
#define HISTB_VDP_CVM2_CSC_UNITY		BIT(14)
#define HISTB_VDP_HDR_ENABLES		GENMASK(31, 30)
#define HISTB_VDP_HDR_REGUP		BIT(0)

#define HISTB_VDP_DHD0_REGUP		BIT(0)
#define HISTB_VDP_DHD0_PROGRESSIVE	BIT(4)
#define HISTB_VDP_DHD0_HDMI_MODE	BIT(14)
#define HISTB_VDP_DHD0_TWOCHN_DEBUG	BIT(15)
#define HISTB_VDP_DHD0_CSC_ENABLE	BIT(22)
#define HISTB_VDP_DHD0_P2I_ENABLE	BIT(28)
#define HISTB_VDP_DHD0_CBAR_SELECT	BIT(29)
#define HISTB_VDP_DHD0_CBAR_ENABLE	BIT(30)
#define HISTB_VDP_DHD0_CBAR_MASK	(HISTB_VDP_DHD0_CBAR_SELECT | \
					 HISTB_VDP_DHD0_CBAR_ENABLE)
#define HISTB_VDP_DHD0_ENABLE		BIT(31)

#define HISTB_VDP_DATE_HSYNC_INV	BIT(13)

#define HISTB_VDP_DHD0_CLIP_FULL	GENMASK(29, 0)

#define HISTB_VDP_VO_MUX_HDMI_SEL	BIT(3)

/* CV200 global pixel-interleaver controls. */
#define HISTB_VDP_VOCTRL_TWOCHN_EN	BIT(24)
#define HISTB_VDP_VOCTRL_TWOCHN_MODE	BIT(25)
#define HISTB_VDP_VOCTRL_CLK_GATE_EN	BIT(31)
#define HISTB_VDP_VOLOWPOWER_DEFAULT	0x00005b2b

#define HISTB_VDP_G0_SMMU_BYPASS_2D	BIT(0)
#define HISTB_VDP_G0_SMMU_BYPASS_3D_READ BIT(1)
#define HISTB_VDP_G0_CSC_ENABLE		BIT(22)
#define HISTB_VDP_G0_OFL_MASTER		BIT(29)
#define HISTB_VDP_VOMASTERSEL_G0		BIT(14)
#define HISTB_VDP_VOMIDORDER_MID_ENABLE	BIT(3)

/* Values used by the CV200 BSP VDP_DRIVER_Initial() AXI setup. */
#define HISTB_VDP_VOAXICTRL_FIELDS	GENMASK(11, 0)
#define HISTB_VDP_VOAXICTRL_MULTI0	BIT(15)
#define HISTB_VDP_VOAXICTRL_FIELDS1	GENMASK(27, 16)
#define HISTB_VDP_VOAXICTRL_MULTI1	BIT(31)
#define HISTB_VDP_VOAXICTRL_MASK	(HISTB_VDP_VOAXICTRL_FIELDS | \
					 HISTB_VDP_VOAXICTRL_MULTI0 | \
					 HISTB_VDP_VOAXICTRL_FIELDS1 | \
					 HISTB_VDP_VOAXICTRL_MULTI1)
#define HISTB_VDP_VOAXICTRL_VALUE	0x07ff87ff
#define HISTB_VDP_VOAXICTRL1_FIELDS	(GENMASK(11, 0) | GENMASK(27, 16))
#define HISTB_VDP_VOAXICTRL1_VALUE	0x07ff07ff

#define HISTB_VDP_SIZE(w, h)		(((w) - 1) | (((h) - 1) << 12))
#define HISTB_VDP_POS(x, y)		((x) | ((y) << 12))

#define HISTB_VDP_MIN_WIDTH		320
#define HISTB_VDP_MIN_HEIGHT		240
#define HISTB_VDP_MAX_WIDTH		4096
#define HISTB_VDP_MAX_HEIGHT		2160

/* MV310 splits graphics fetching from G0 and moves GP0 and DHD0. */
#define MV310_G0_CTRL			0x5000
#define MV310_G0_UPD			0x5004
#define MV310_G0_DFPOS			0x5080
#define MV310_G0_DLPOS			0x5084
#define MV310_G0_VFPOS			0x5088
#define MV310_G0_VLPOS			0x508c
#define MV310_G0_ALPHA			0x5094
#define MV310_GP0_CTRL			0x6800
#define MV310_GP0_UPD			0x6804
#define MV310_GP0_IRESO			0x6810
#define MV310_GP0_DFPOS			0x6880
#define MV310_GP0_DLPOS			0x6884
#define MV310_GP0_VFPOS			0x6888
#define MV310_GP0_VLPOS			0x688c
#define MV310_MIXG0_BKG			0x6900
#define MV310_MIXG0_BKALPHA		0x6904
#define MV310_MIXG0_MIX			0x6908
#define MV310_GP0_ZME_HINFO		0x6a00
#define MV310_GP0_ZME_HSP		0x6a04
#define MV310_GP0_ZME_VINFO		0x6a10
#define MV310_GP0_ZME_VSP		0x6a14
#define MV310_MST_OUTSTANDING		0x8000
#define MV310_MST_CTRL			0x8004
#define MV310_MST_RCHN_PRIO0		0x8010
#define MV310_GFX_DTV_MODE		0x8980
#define MV310_GFX_READ_CTRL		0x8a00
#define MV310_GFX_MAC_CTRL		0x8a04
#define MV310_GFX_OUT_CTRL		0x8a08
#define MV310_GFX_SMMU_BYPASS		0x8a20
#define MV310_GFX_1555_ALPHA		0x8a28
#define MV310_GFX_SRC_INFO		0x8a40
#define MV310_GFX_SRC_RESO		0x8a44
#define MV310_GFX_SRC_CROP		0x8a48
#define MV310_GFX_IRESO			0x8a4c
#define MV310_GFX_ADDR_H			0x8a50
#define MV310_GFX_ADDR_L			0x8a54
#define MV310_GFX_NADDR_H		0x8a58
#define MV310_GFX_NADDR_L		0x8a5c
#define MV310_GFX_STRIDE			0x8a60
#define MV310_CBM_BKG1			0x0200
#define MV310_CBM_MIX1			0x0208
#define MV310_VO_MUX			0xc004
#define MV310_DHD0_CTRL			0xc200
#define MV310_DHD0_CTRL1			0xc204
#define MV310_DHD0_VSYNC			0xc208
#define MV310_DHD0_VSYNC2		0xc20c
#define MV310_DHD0_HSYNC1		0xc210
#define MV310_DHD0_HSYNC2		0xc214
#define MV310_DHD0_VPLUS			0xc218
#define MV310_DHD0_VPLUS2		0xc21c
#define MV310_DHD0_PWR			0xc220
#define MV310_DHD0_VTTHD			0xc228
#define MV310_INTF_HDMI_CTRL		0xc600
#define MV310_INTF_HDMI_SYNC_INV		0xc604
#define MV310_INTF_HDMI_CLIP_L		0xc608
#define MV310_INTF_HDMI_CLIP_H		0xc60c
#define MV310_G0_UPDATE_PROGRESSIVE	BIT(29)
#define MV310_GFX_BITEXT			GENMASK(9, 8)
#define MV310_GFX_PIXEL_ALPHA		BIT(1)
#define MV310_GFX_ALPHA_FULL		BIT(0)
#define MV310_GP0_MUX1			GENMASK(20, 19)
#define MV310_GP0_MUX2			GENMASK(22, 21)
#define MV310_GP0_BOUT			GENMASK(18, 17)
#define MV310_GP0_UPDATE_PROGRESSIVE	BIT(29)
#define MV310_GP0_READ_PROGRESSIVE	BIT(30)
#define MV310_GP0_ENABLE			BIT(31)
#define MV310_INTF_HDMI_RGB		BIT(6)
#define MV310_HDMI_HSYNC_INV		BIT(1)
#define MV310_HDMI_VSYNC_INV		BIT(2)
#define MV310_POS(x, y)			((x) | ((y) << 16))
#define MV310_SIZE(w, h)			MV310_POS((w) - 1, (h) - 1)

/* CV200 HIFB maps logical HD2 to physical G3; there is no G2 gate. */
#define HISTB_VDP_GFX_COUNT	3
#define HISTB_VDP_VIDEO_COUNT	2
#define HISTB_VDP_PLANE_COUNT	5
#define HISTB_VDP_GFX_OFFSET	0x200
#define HISTB_VDP_VIDEO_OFFSET	0x800

static const u8 histb_vdp_gfx_ids[] = { 0, 1, 3 };

struct histb_vdp_overlay {
	struct drm_plane plane;
	u8 id;
};

struct histb_vdp_zme {
	void *cpu;
	dma_addr_t dma;
	u32 src_width, src_height, dst_width, dst_height;
	u8 bank;
	bool valid;
};

struct histb_vdp_mixer {
	u32 graphics, video, output;
	u32 gp0_input, gp0_output, gp0_hsp, gp0_vsp, gp0_vsr;
	u8 graphics_count, graphics_alpha;
};

struct histb_vdp_crtc_state {
	struct drm_crtc_state base;
	struct histb_vdp_mixer mixer;
	u32 plane_updates;
	bool hdr;
	bool inputs_failed;
};

struct histb_vdp_p010_fb {
	struct drm_framebuffer base;
	void *packed;
	dma_addr_t dma;
	size_t size;
	u32 pitch;
	u8 bank;
};

struct histb_vdp_tile_layout {
	u32 address[4];
	u32 pitch;
	u32 low_pitch;
};

struct histb_vdp_soc_data {
	u32 g0_ctrl;
	u32 g0_upd;
	u32 gp0_upd;
	u32 dhd0_ctrl;
	u32 max_width;
	u32 max_height;
	bool mv310;
};


static const struct histb_vdp_soc_data histb_cv200_data = {
	.g0_ctrl = HISTB_VDP_G0_CTRL,
	.g0_upd = HISTB_VDP_G0_UPD,
	.gp0_upd = HISTB_VDP_GP0_UPD,
	.dhd0_ctrl = HISTB_VDP_DHD0_CTRL,
	.max_width = HISTB_VDP_MAX_WIDTH,
	.max_height = HISTB_VDP_MAX_HEIGHT,
};

static const struct histb_vdp_soc_data histb_mv310_data = {
	.g0_ctrl = MV310_G0_CTRL,
	.g0_upd = MV310_G0_UPD,
	.gp0_upd = MV310_GP0_UPD,
	.dhd0_ctrl = MV310_DHD0_CTRL,
	/* BSP 98mv310/vdp_mac_define.c declares GFX0 capacity as 4096x2160. */
	.max_width = HISTB_VDP_MAX_WIDTH,
	.max_height = HISTB_VDP_MAX_HEIGHT,
	.mv310 = true,
};

enum histb_vdp_gfx_format {
	HISTB_VDP_GFX_RGB565 = 0x42,
	HISTB_VDP_GFX_XRGB8888 = 0x60,
	HISTB_VDP_GFX_ARGB8888 = 0x68,
};

struct histb_vdp {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_crtc_helper_funcs crtc_helper_funcs;
	struct drm_plane_helper_funcs primary_helper_funcs;
	struct histb_vdp_overlay overlays[HISTB_VDP_PLANE_COUNT - 1];
	struct drm_connector *connector;
	struct histb_vdp_zme zme[HISTB_VDP_VIDEO_COUNT];
	void *gp0_coefficients;
	dma_addr_t gp0_coefficients_dma;
	u32 display_width, display_height;
	bool video_enabled[HISTB_VDP_VIDEO_COUNT];
	void __iomem *base;
	const struct histb_vdp_soc_data *soc;
	struct clk_bulk_data clks[10];
	unsigned int num_clks;
	unsigned int close_delay_us;
	struct reset_control *reset;
	/* Serializes register snapshots against reset and clock removal. */
	struct mutex power_lock;
	atomic64_t underflows;
	atomic64_t master_errors;
	/* Protects interrupt state while the register clock is running. */
	spinlock_t irq_lock;
	int irq;
	bool resources_enabled;
	bool powered;
	bool vblank_enabled;
};

/*
 * Both SoCs use a 32-bit, non-IOMMU AXI scanout path here. A PRIME
 * buffer exported by a renderer may be physically scattered, so retain the
 * imported attachment for the source while scanning out a private,
 * physically contiguous DMA shadow.
 */
struct histb_vdp_shadow_object {
	struct drm_gem_dma_object dma;
	struct sg_table *import_sgt;
	size_t source_size;
};

static const struct drm_gem_object_funcs histb_vdp_shadow_object_funcs;

static const u32 histb_vdp_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static const u32 histb_vdp_v0_formats[] = {
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
};

static const u32 histb_vdp_hdr_formats[] = {
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
	DRM_FORMAT_NV15,
	DRM_FORMAT_P010,
	DRM_FORMAT_YUV420_10BIT,
};

static const u64 histb_vdp_v0_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_HISTB_VDH_10BIT,
	DRM_FORMAT_MOD_INVALID,
};


static inline struct histb_vdp *to_histb_vdp(struct drm_device *drm)
{
	return container_of(drm, struct histb_vdp, drm);
}

static struct histb_vdp_crtc_state *
to_histb_vdp_crtc_state(struct drm_crtc_state *state)
{
	return container_of(state, struct histb_vdp_crtc_state, base);
}

static struct drm_plane *histb_vdp_plane(struct histb_vdp *vdp, unsigned int id)
{
	return id ? &vdp->overlays[id - 1].plane : &vdp->pipe.plane;
}

static unsigned int histb_vdp_plane_id(struct histb_vdp *vdp,
				     struct drm_plane *plane)
{
	if (plane == &vdp->pipe.plane)
		return 0;
	return container_of(plane, struct histb_vdp_overlay, plane)->id;
}

static bool histb_vdp_plane_visible(const struct drm_plane_state *state)
{
	return state && state->fb && state->crtc && state->visible && state->alpha;
}

static bool histb_vdp_tiled(const struct drm_framebuffer *fb)
{
	return fb->format->format == DRM_FORMAT_YUV420_10BIT &&
	       fb->modifier == DRM_FORMAT_MOD_HISTB_VDH_10BIT;
}

static bool histb_vdp_ten_bit(const struct drm_framebuffer *fb)
{
	return fb->format->format == DRM_FORMAT_NV15 ||
	       fb->format->format == DRM_FORMAT_P010 || histb_vdp_tiled(fb);
}

static inline struct histb_vdp_shadow_object *
to_histb_vdp_shadow_object(struct drm_gem_object *obj)
{
	if (obj->funcs != &histb_vdp_shadow_object_funcs)
		return NULL;

	return container_of(to_drm_gem_dma_obj(obj),
				    struct histb_vdp_shadow_object, dma);
}

static inline bool histb_vdp_dma_address_valid(dma_addr_t address)
{
	return !upper_32_bits(address) &&
	       IS_ALIGNED(address, HISTB_VDP_G0_STRIDE_UNIT);
}

static u64 histb_vdp_plane_address(const struct drm_plane_state *state,
				 unsigned int index)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_dma_object *obj = drm_fb_dma_get_gem_obj(fb, index);
	u32 hsub = index ? fb->format->hsub : 1;
	u32 vsub = index ? fb->format->vsub : 1;
	u32 block = drm_format_info_block_width(fb->format, index);
	u32 bytes = fb->format->char_per_block[index];

	return (u64)obj->dma_addr + fb->offsets[index] +
		(u64)((state->src.y1 >> 16) / vsub) * fb->pitches[index] +
		((state->src.x1 >> 16) / hsub / block) * bytes;
}

/*
 * CV200 VDH high-eight/low-two reconstruction, not packed linear NV15.
 * The framebuffer dimensions are the CODED dimensions (see histb_drm.h).
 * A decoder tail guard is allowed, so object size cannot supply coded height.
 */
static int histb_vdp_tile_layout(struct drm_framebuffer *fb,
				 struct histb_vdp_tile_layout *layout)
{
	struct drm_gem_dma_object *obj;
	u64 luma, offsets[4], sizes[4], address, total, object_size;
	unsigned int i;

	if (!histb_vdp_tiled(fb) || !(fb->flags & DRM_MODE_FB_MODIFIERS) ||
	    fb->flags & DRM_MODE_FB_INTERLACED || fb->format->num_planes != 1 ||
	    fb->width < 2 || fb->width > HISTB_VDP_MAX_WIDTH ||
	    fb->height < 2 || fb->height > HISTB_VDP_MAX_HEIGHT ||
	    (fb->width | fb->height) & 1 || fb->offsets[0] ||
	    fb->pitches[0] != ALIGN(fb->width, 256))
		return -EINVAL;
	obj = drm_fb_dma_get_gem_obj(fb, 0);
	/* A fragmented import's linear shadow is never a native tile surface. */
	if (!obj || to_histb_vdp_shadow_object(&obj->base))
		return -EINVAL;
	object_size = obj->base.size;
	if (obj->base.import_attach)
		object_size = min_t(u64, object_size,
			obj->base.import_attach->dmabuf->size);
	luma = (u64)fb->pitches[0] * ALIGN(fb->height, 64);
	offsets[0] = 0;
	offsets[1] = luma;
	offsets[2] = luma * 3 / 2;
	offsets[3] = luma * 7 / 4;
	sizes[0] = luma;
	sizes[1] = luma / 2;
	sizes[2] = luma / 4;
	sizes[3] = luma / 8;
	total = luma * 15 / 8;
	if (total > object_size)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(offsets); i++) {
		if (offsets[i] > object_size ||
		    sizes[i] > object_size - offsets[i] ||
		    check_add_overflow((u64)obj->dma_addr, offsets[i], &address) ||
		    !IS_ALIGNED(address, 64) || address > U32_MAX ||
		    sizes[i] - 1 > U32_MAX - address)
			return -EINVAL;
		layout->address[i] = address;
	}
	layout->pitch = fb->pitches[0];
	layout->low_pitch = fb->pitches[0] / 4;
	return 0;
}

static void histb_vdp_shadow_object_free(struct drm_gem_object *obj)
{
	struct histb_vdp_shadow_object *shadow =
		to_histb_vdp_shadow_object(obj);

	if (shadow->dma.vaddr)
		dma_free_wc(obj->dev->dev, obj->size, shadow->dma.vaddr,
			     shadow->dma.dma_addr);

	if (obj->import_attach)
		drm_prime_gem_destroy(obj, shadow->import_sgt);

	drm_gem_object_release(obj);
	kfree(shadow);
}

/* drm_gem_vmap() holds obj->resv, which is the imported dma-buf reservation. */
static int histb_vdp_shadow_object_vmap(struct drm_gem_object *obj,
					struct iosys_map *map)
{
	struct dma_buf_attachment *attach = obj->import_attach;

	if (!attach)
		return -EINVAL;

	return dma_buf_vmap(attach->dmabuf, map);
}

static void histb_vdp_shadow_object_vunmap(struct drm_gem_object *obj,
					   struct iosys_map *map)
{
	struct dma_buf_attachment *attach = obj->import_attach;

	if (attach)
		dma_buf_vunmap(attach->dmabuf, map);
}

static const struct drm_gem_object_funcs histb_vdp_shadow_object_funcs = {
	.free = histb_vdp_shadow_object_free,
	.print_info = drm_gem_dma_object_print_info,
	.get_sg_table = drm_gem_dma_object_get_sg_table,
	.vmap = histb_vdp_shadow_object_vmap,
	.vunmap = histb_vdp_shadow_object_vunmap,
	.mmap = drm_gem_dma_object_mmap,
	.vm_ops = &drm_gem_dma_vm_ops,
};

static struct drm_gem_object *
histb_vdp_prime_import_sg_table(struct drm_device *dev,
				struct dma_buf_attachment *attach,
				struct sg_table *sgt)
{
	struct histb_vdp_shadow_object *shadow;
	struct drm_gem_object *obj;
	size_t size;
	int ret;

	/* Preserve the normal zero-copy path whenever the AXI address is linear. */
	if (drm_prime_get_contiguous_size(sgt) >= attach->dmabuf->size)
		return drm_gem_dma_prime_import_sg_table_vmap(dev, attach, sgt);

	size = PAGE_ALIGN(attach->dmabuf->size);
	if (!size)
		return ERR_PTR(-EINVAL);

	shadow = kzalloc(sizeof(*shadow), GFP_KERNEL);
	if (!shadow)
		return ERR_PTR(-ENOMEM);

	obj = &shadow->dma.base;
	obj->funcs = &histb_vdp_shadow_object_funcs;
	drm_gem_private_object_init(dev, obj, size);
	shadow->import_sgt = sgt;
	shadow->source_size = attach->dmabuf->size;
	shadow->dma.map_noncoherent = false;

	ret = drm_gem_create_mmap_offset(obj);
	if (ret)
		goto err_release;

	shadow->dma.vaddr = dma_alloc_wc(dev->dev, size,
					&shadow->dma.dma_addr,
					GFP_KERNEL | __GFP_NOWARN);
	if (!shadow->dma.vaddr) {
		ret = -ENOMEM;
		goto err_release;
	}

	if (!histb_vdp_dma_address_valid(shadow->dma.dma_addr)) {
		ret = -EINVAL;
		dma_free_wc(dev->dev, size, shadow->dma.vaddr,
				shadow->dma.dma_addr);
		shadow->dma.vaddr = NULL;
		goto err_release;
	}

	return obj;

err_release:
	drm_gem_object_release(obj);
	kfree(shadow);
	return ERR_PTR(ret);
}

/* Mappings live until the old plane state is retired after scanout. */
static int histb_vdp_copy_shadow(struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_shadow_plane_state *shadow_state;
	unsigned int i;
	bool copy = false;
	int ret;

	if (!fb)
		return 0;
	if (histb_vdp_tiled(fb))
		return -EINVAL;

	for (i = 0; i < fb->format->num_planes; i++) {
		struct drm_gem_object *obj = drm_gem_fb_get_obj(fb, i);
		struct histb_vdp_shadow_object *shadow;
		size_t height, size;

		if (!obj)
			return -EINVAL;
		shadow = to_histb_vdp_shadow_object(obj);
		if (!shadow)
			continue;
		copy = true;
		height = drm_format_info_plane_height(fb->format, fb->height, i);
		size = (height - 1) * fb->pitches[i] +
			drm_format_info_min_pitch(fb->format, i,
				drm_format_info_plane_width(fb->format, fb->width, i));
		if (size > shadow->source_size ||
		    fb->offsets[i] > shadow->source_size - size)
			return -EINVAL;
	}
	if (!copy)
		return 0;

	shadow_state = to_drm_shadow_plane_state(state);
	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (ret)
		return ret;
	for (i = 0; i < fb->format->num_planes; i++) {
		struct histb_vdp_shadow_object *shadow =
			to_histb_vdp_shadow_object(drm_gem_fb_get_obj(fb, i));
		size_t height, size;

		if (!shadow)
			continue;
		height = drm_format_info_plane_height(fb->format, fb->height, i);
		size = (height - 1) * fb->pitches[i] +
			drm_format_info_min_pitch(fb->format, i,
				drm_format_info_plane_width(fb->format, fb->width, i));
		iosys_map_memcpy_from((u8 *)shadow->dma.vaddr + fb->offsets[i],
				      &shadow_state->data[i], 0, size);
	}
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
	dma_wmb();
	return 0;
}

static u32 histb_vdp_gfx_format(u32 format)
{
	switch (format) {
	case DRM_FORMAT_RGB565:
		return HISTB_VDP_GFX_RGB565;
	case DRM_FORMAT_XRGB8888:
		return HISTB_VDP_GFX_XRGB8888;
	case DRM_FORMAT_ARGB8888:
	default:
		return HISTB_VDP_GFX_ARGB8888;
	}
}

static bool
histb_vdp_plane_config_unchanged(const struct drm_plane_state *old_state,
				 const struct drm_plane_state *new_state)
{
	const struct drm_framebuffer *old_fb;
	const struct drm_framebuffer *new_fb;

	if (!old_state || !old_state->fb || !new_state->fb)
		return false;

	old_fb = old_state->fb;
	new_fb = new_state->fb;

	return old_fb->format->format == new_fb->format->format &&
	       old_fb->modifier == new_fb->modifier &&
	       old_fb->pitches[0] == new_fb->pitches[0] &&
	       old_fb->width == new_fb->width &&
	       old_fb->height == new_fb->height &&
	       old_state->src_x == new_state->src_x &&
	       old_state->src_y == new_state->src_y &&
	       old_state->src_w == new_state->src_w &&
	       old_state->src_h == new_state->src_h &&
	       old_state->crtc_x == new_state->crtc_x &&
	       old_state->crtc_y == new_state->crtc_y &&
	       old_state->crtc_w == new_state->crtc_w &&
	       old_state->crtc_h == new_state->crtc_h &&
	       old_state->alpha == new_state->alpha &&
	       old_state->pixel_blend_mode == new_state->pixel_blend_mode &&
	       drm_rect_equals(&old_state->src, &new_state->src) &&
	       drm_rect_equals(&old_state->dst, &new_state->dst);
}

/*
 * G0 and GP0 have separate shadow latches, but the CV200 graphics path only
 * observes a complete pair.  Read back each request to flush the posted AXI
 * write before issuing the second request.
 */
static void histb_vdp_latch_gfx(struct histb_vdp *vdp)
{
	unsigned int i;

	dma_wmb();
	if (vdp->soc->mv310) {
		writel(1, vdp->base + vdp->soc->g0_upd);
		readl(vdp->base + vdp->soc->g0_upd);
	} else {
		for (i = 0; i < HISTB_VDP_GFX_COUNT; i++) {
			u32 reg = HISTB_VDP_G0_UPD + histb_vdp_gfx_ids[i] * HISTB_VDP_GFX_OFFSET;

			writel(1, vdp->base + reg);
			readl(vdp->base + reg);
		}
	}
	writel(1, vdp->base + vdp->soc->gp0_upd);
	readl(vdp->base + vdp->soc->gp0_upd);
}

static void histb_mv310_program_plane(struct histb_vdp *vdp,
				     struct drm_plane_state *state,
				     const struct drm_plane_state *old_state,
				     dma_addr_t address)
{
	struct drm_framebuffer *fb = state->fb;
	u32 width = state->src_w >> 16;
	u32 height = state->src_h >> 16;
	u32 first = MV310_POS(state->crtc_x, state->crtc_y);
	u32 last = MV310_POS(state->crtc_x + width - 1,
			     state->crtc_y + height - 1);

	/* HIFB v2 uses the FDR address pair, with a 32-bit physical DMA API. */
	writel(0, vdp->base + MV310_GFX_ADDR_H);
	writel(lower_32_bits(address), vdp->base + MV310_GFX_ADDR_L);
	if (histb_vdp_plane_config_unchanged(old_state, state))
		return;

	/* Linear 2D, no compression, color key, premultiply or test pattern. */
	writel(0, vdp->base + MV310_GFX_DTV_MODE);
	writel(1, vdp->base + MV310_GFX_READ_CTRL);
	/* BSP: 16-beat requests, one burst, online AXI master. */
	writel(0, vdp->base + MV310_GFX_MAC_CTRL);
	writel(GENMASK(3, 0), vdp->base + MV310_GFX_SMMU_BYPASS);
	writel(FIELD_PREP(MV310_GFX_BITEXT, 3) | MV310_GFX_ALPHA_FULL |
	       (fb->format->has_alpha ? MV310_GFX_PIXEL_ALPHA : 0),
	       vdp->base + MV310_GFX_OUT_CTRL);
	writel(0xffff, vdp->base + MV310_GFX_1555_ALPHA);
	writel(histb_vdp_gfx_format(fb->format->format),
	       vdp->base + MV310_GFX_SRC_INFO);
	writel(MV310_SIZE(width, height), vdp->base + MV310_GFX_SRC_RESO);
	writel(0, vdp->base + MV310_GFX_SRC_CROP);
	writel(MV310_SIZE(width, height), vdp->base + MV310_GFX_IRESO);
	writel(0, vdp->base + MV310_GFX_NADDR_H);
	writel(0, vdp->base + MV310_GFX_NADDR_L);
	writel(fb->pitches[0] / HISTB_VDP_G0_STRIDE_UNIT,
	       vdp->base + MV310_GFX_STRIDE);
	writel(first, vdp->base + MV310_G0_DFPOS);
	writel(last, vdp->base + MV310_G0_DLPOS);
	writel(first, vdp->base + MV310_G0_VFPOS);
	writel(last, vdp->base + MV310_G0_VLPOS);
	writel(0xff, vdp->base + MV310_G0_ALPHA);
	writel((state->alpha >> 8) | MV310_G0_UPDATE_PROGRESSIVE |
	       HISTB_VDP_G0_NONSECURE | HISTB_VDP_G0_ENABLE,
	       vdp->base + MV310_G0_CTRL);
}

static void histb_vdp_destroy_crtc_state(struct drm_simple_display_pipe *pipe,
					  struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(state);
	kfree(to_histb_vdp_crtc_state(state));
}

static void histb_vdp_reset_crtc(struct drm_simple_display_pipe *pipe)
{
	struct histb_vdp_crtc_state *state;

	if (pipe->crtc.state)
		histb_vdp_destroy_crtc_state(pipe, pipe->crtc.state);
	pipe->crtc.state = NULL;
	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return;
	__drm_atomic_helper_crtc_reset(&pipe->crtc, &state->base);
	state->mixer.graphics = 1;
	state->mixer.output = 2;
	state->mixer.graphics_alpha = 0xff;
}

static struct drm_crtc_state *
histb_vdp_duplicate_crtc_state(struct drm_simple_display_pipe *pipe)
{
	struct histb_vdp_crtc_state *old = to_histb_vdp_crtc_state(pipe->crtc.state);
	struct histb_vdp_crtc_state *state = kzalloc(sizeof(*state), GFP_KERNEL);

	if (!state)
		return NULL;
	__drm_atomic_helper_crtc_duplicate_state(&pipe->crtc, &state->base);
	state->mixer = old->mixer;
	state->hdr = old->hdr;
	return &state->base;
}

static void histb_vdp_gfx_disable(struct histb_vdp *vdp, unsigned int id)
{
	u32 reg = vdp->soc->mv310 ? MV310_G0_CTRL :
		HISTB_VDP_G0_CTRL + histb_vdp_gfx_ids[id] * HISTB_VDP_GFX_OFFSET;

	writel(readl(vdp->base + reg) & ~HISTB_VDP_G0_ENABLE, vdp->base + reg);
}

static int histb_vdp_program_gfx(struct histb_vdp *vdp, unsigned int id,
				struct drm_plane_state *state,
				const struct drm_plane_state *old_state,
				const struct histb_vdp_mixer *mixer, bool copy)
{
	struct drm_framebuffer *fb = state->fb;
	dma_addr_t address;
	u32 off = histb_vdp_gfx_ids[id] * HISTB_VDP_GFX_OFFSET;
	u32 width, height, x, y, ctrl;
	u8 alpha = mixer->graphics_count > 1 ? state->alpha >> 8 : 0xff;
	int ret;

	if (!histb_vdp_plane_visible(state)) {
		histb_vdp_gfx_disable(vdp, id);
		return 0;
	}
	if (copy) {
		ret = histb_vdp_copy_shadow(state);
		if (ret)
			return ret;
	}
	address = histb_vdp_plane_address(state, 0);
	if (vdp->soc->mv310) {
		histb_mv310_program_plane(vdp, state, old_state, address);
		return 0;
	}
	width = drm_rect_width(&state->src) >> 16;
	height = drm_rect_height(&state->src) >> 16;
	x = state->dst.x1;
	y = state->dst.y1;
	writel(lower_32_bits(address), vdp->base + HISTB_VDP_G0_ADDR + off);
	/* Only a lone G input may use GP0 to scale premultiplied RGB and alpha. */
	writel(alpha | BIT(8) |
	       (fb->format->has_alpha && state->pixel_blend_mode != DRM_MODE_BLEND_PIXEL_NONE ?
		HISTB_VDP_G0_PIXEL_ALPHA : 0) |
	       (state->pixel_blend_mode == DRM_MODE_BLEND_PREMULTI ? BIT(13) : 0),
	       vdp->base + HISTB_VDP_G0_CBMPARA + off);
	if (old_state && old_state->visible &&
	    histb_vdp_plane_config_unchanged(old_state, state))
		return 0;

	writel(readl(vdp->base + HISTB_VDP_VOMASTERSEL) &
	       ~BIT(14 + histb_vdp_gfx_ids[id]), vdp->base + HISTB_VDP_VOMASTERSEL);
	writel(0, vdp->base + HISTB_VDP_G0_NADDR + off);
	writel(HISTB_VDP_G0_SMMU_BYPASS_2D | HISTB_VDP_G0_SMMU_BYPASS_3D_READ,
	       vdp->base + HISTB_VDP_G0_SMMU_BYPASS0 + off);
	writel(0, vdp->base + HISTB_VDP_G0_DCMP_ADDR + off);
	writel(0, vdp->base + HISTB_VDP_G0_DCMP_NADDR + off);
	writel(0, vdp->base + HISTB_VDP_G0_DCMP_OFFSET + off);
	writel(fb->pitches[0] / HISTB_VDP_G0_STRIDE_UNIT,
	       vdp->base + HISTB_VDP_G0_STRIDE + off);
	writel(HISTB_VDP_SIZE(width, height), vdp->base + HISTB_VDP_G0_IRESO + off);
	writel(0, vdp->base + HISTB_VDP_G0_SFPOS + off);
	writel(HISTB_VDP_POS(x, y), vdp->base + HISTB_VDP_G0_DFPOS + off);
	writel(HISTB_VDP_POS(x + width - 1, y + height - 1),
	       vdp->base + HISTB_VDP_G0_DLPOS + off);
	writel(HISTB_VDP_POS(x, y), vdp->base + HISTB_VDP_G0_VFPOS + off);
	writel(HISTB_VDP_POS(x + width - 1, y + height - 1),
	       vdp->base + HISTB_VDP_G0_VLPOS + off);
	writel(0, vdp->base + HISTB_VDP_G0_CSC_IDC + off);
	/* BSP assigns request grouping only to G0; HIFB preserves each bank's
	 * burst length, FIFO policy and the secondary request grouping.
	 */
	ctrl = readl(vdp->base + HISTB_VDP_G0_CTRL + off) & HISTB_VDP_G0_FETCH_POLICY;
	if (!id)
		ctrl = (ctrl & ~HISTB_VDP_G0_REQ_CTRL) |
			FIELD_PREP(HISTB_VDP_G0_REQ_CTRL, 3);
	ctrl |= FIELD_PREP(HISTB_VDP_G0_IFMT, histb_vdp_gfx_format(fb->format->format)) |
		FIELD_PREP(HISTB_VDP_G0_BITEXT, 3) |
		HISTB_VDP_G0_READ_PROGRESSIVE | HISTB_VDP_G0_UPDATE_PROGRESSIVE |
		HISTB_VDP_G0_NONSECURE | HISTB_VDP_G0_ENABLE;
	writel(ctrl, vdp->base + HISTB_VDP_G0_CTRL + off);
	writel(0xff, vdp->base + HISTB_VDP_G0_ALPHA + off);
	return 0;
}

/* CV200 routes V0 SDR YCbCr conversion through VDM, not the legacy CSC. */
static void histb_vdp_program_video_csc(struct histb_vdp *vdp, unsigned int id,
					const struct drm_plane_state *state)
{
	static const s16 matrix[2][2][9] = {
		[DRM_COLOR_YCBCR_BT601] = {
			[DRM_COLOR_YCBCR_LIMITED_RANGE] = {
				1192, 0, 1634, 1192, -400, -833, 1192, 2066, 0,
			},
			[DRM_COLOR_YCBCR_FULL_RANGE] = {
				1024, 0, 1436, 1024, -352, -731, 1024, 1815, 0,
			},
		},
		[DRM_COLOR_YCBCR_BT709] = {
			[DRM_COLOR_YCBCR_LIMITED_RANGE] = {
				1192, 0, 1836, 1192, -218, -547, 1192, 2166, 0,
			},
			[DRM_COLOR_YCBCR_FULL_RANGE] = {
				1024, 0, 1613, 1024, -191, -479, 1024, 1901, 0,
			},
		},
	};
	const s16 *coef = matrix[state->color_encoding][state->color_range];
	s32 y_dc = state->color_range == DRM_COLOR_YCBCR_LIMITED_RANGE ? -1024 : 0;
	unsigned int i;
	u32 coef_reg = id ? 0x3528 : HISTB_VDP_VDM_YUV2RGB_COEF00;
	u32 dc_reg = id ? 0x3568 : HISTB_VDP_VDM_YUV2RGB_IN_DC00;
	u32 out_reg = id ? 0x3574 : HISTB_VDP_VDM_YUV2RGB_OUT_DC00;

	/* PQ Q10 coefficients become Q13; DC channels are Y, Cb, Cr. */
	for (i = 0; i < 9; i++)
		writel((coef[i] * 8) & 0xffff,
		       vdp->base + coef_reg + i * 4);
	writel(13, vdp->base + HISTB_VDP_VDM_YUV2RGB_SCALE2P);
	writel(y_dc & 0x3fff, vdp->base + dc_reg);
	writel((-8192) & 0x3fff, vdp->base + dc_reg + 4);
	writel((-8192) & 0x3fff, vdp->base + dc_reg + 8);
	for (i = 0; i < 3; i++)
		writel(0, vdp->base + out_reg + i * 4);
	writel(0, vdp->base + HISTB_VDP_V0_CSC_IDC + id * HISTB_VDP_VIDEO_OFFSET);
	/* The SDR provider selects 14-bit YCbCr with transfer functions bypassed. */
	writel(BIT(0) | BIT(4) | BIT(6), vdp->base + HISTB_VDP_VDM_CTRL);
}

static void histb_vdp_v0_chain_init(struct histb_vdp *vdp)
{
	unsigned int i;

	for (i = 0; i < 9; i++) {
		writel(i % 4 == 0 ? 1 : 0,
		       vdp->base + HISTB_VDP_VDM_RGB2LMS_COEF + i * 4);
		writel(i % 4 == 0 ? 0x4000 : 0,
		       vdp->base + HISTB_VDP_CVM1_LMS2IPT_COEF + i * 4);
	}
	writel(0, vdp->base + HISTB_VDP_VDM_RGB2LMS_SCALE2P);
	writel(1, vdp->base + HISTB_VDP_VDM_RGB2LMS_CTRL);
	writel(readl(vdp->base + HISTB_VDP_CVM_CTRL) & ~BIT(1),
	       vdp->base + HISTB_VDP_CVM_CTRL);
	writel(15 | BIT(5), vdp->base + HISTB_VDP_CVM1_LMS2IPT_SCALE2P);
	writel(0, vdp->base + HISTB_VDP_CVM1_LMS2IPT_MIN);
	writel(0x7fff, vdp->base + HISTB_VDP_CVM1_LMS2IPT_MAX);
	for (i = 0; i < 3; i++)
		writel(0, vdp->base + HISTB_VDP_CVM1_LMS2IPT_OUT_DC + i * 4);
	writel(1, vdp->base + HISTB_VDP_CVM1_LMS2IPT_CTRL);
}

static void histb_vdp_program_zme(struct histb_vdp *vdp, unsigned int id,
				  u32 sw, u32 sh, u32 dw, u32 dh)
{
	struct histb_vdp_zme *zme = &vdp->zme[id];
	u32 off = id * HISTB_VDP_VIDEO_OFFSET;
	dma_addr_t address = zme->dma;
	u32 hsp, vsp, vratio;

	if (zme->valid && (id || (zme->src_width == sw && zme->src_height == sh &&
				 zme->dst_width == dw && zme->dst_height == dh)))
		return;
	if (!id) {
		zme->bank ^= 1;
		address += zme->bank * HISTB_VDP_CV200_ZME_SIZE;
		histb_vdp_cv200_build_zme(zme->cpu + zme->bank * HISTB_VDP_CV200_ZME_SIZE,
					sw, sh, dw, dh);
		hsp = div_u64((u64)(sw / 2) << 20, dw) * 2 |
			BIT(24) | BIT(25) | BIT(26) | BIT(30) | BIT(31);
		vsp = FIELD_PREP(GENMASK(20, 19), 1) |
			BIT(23) | BIT(24) | BIT(30) | BIT(31);
		vratio = div_u64((u64)sh << 12, dh);
	} else {
		/* CV200 V1 is unity scanout with vertical chroma reconstruction only. */
		hsp = BIT(20) | BIT(24);
		vsp = FIELD_PREP(GENMASK(20, 19), 1) | BIT(23) | BIT(24) | BIT(30);
		vratio = BIT(12);
	}
	dma_wmb();
	writel(lower_32_bits(address), vdp->base + HISTB_VDP_V0_HLCOEFAD + off);
	writel(lower_32_bits(address + 256), vdp->base + HISTB_VDP_V0_HCCOEFAD + off);
	writel(lower_32_bits(address + 512), vdp->base + HISTB_VDP_V0_VLCOEFAD + off);
	writel(lower_32_bits(address + 768), vdp->base + HISTB_VDP_V0_VCCOEFAD + off);
	writel(0xf, vdp->base + HISTB_VDP_V0_PARAUP + off);
	writel(hsp, vdp->base + HISTB_VDP_V0_HSP + off);
	writel(vsp, vdp->base + HISTB_VDP_V0_VSP + off);
	writel(vratio, vdp->base + HISTB_VDP_V0_VSR + off);
	writel(0, vdp->base + HISTB_VDP_V0_HLOFFSET + off);
	writel(0, vdp->base + HISTB_VDP_V0_HCOFFSET + off);
	writel(id ? 0 : (u16)-512, vdp->base + HISTB_VDP_V0_VOFFSET + off);
	writel(id ? 0 : (u16)-512, vdp->base + HISTB_VDP_V0_VBOFFSET + off);
	zme->src_width = sw;
	zme->src_height = sh;
	zme->dst_width = dw;
	zme->dst_height = dh;
	zme->valid = true;
}

static void histb_vdp_video_disable(struct histb_vdp *vdp, unsigned int id)
{
	u32 off = id * HISTB_VDP_VIDEO_OFFSET;

	vdp->video_enabled[id] = false;
	writel(readl(vdp->base + HISTB_VDP_V0_CTRL + off) &
	       ~(HISTB_VDP_V0_SURFACE_EN | HISTB_VDP_V0_SURFACE_C_EN),
	       vdp->base + HISTB_VDP_V0_CTRL + off);
	writel(0, vdp->base + HISTB_VDP_V0_16REGIONENL + off);
	if (id)
		writel(0, vdp->base + HISTB_VDP_V0_P0CTRL + off);
}

static int histb_vdp_copy_p010(struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct histb_vdp_p010_fb *packed = container_of(fb, struct histb_vdp_p010_fb, base);
	struct drm_shadow_plane_state *shadow = to_drm_shadow_plane_state(state);
	u8 bank = packed->bank ^ 1;
	u8 *dst = packed->packed + bank * packed->size;
	int ret;

	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (ret)
		return ret;
	histb_vdp_cv200_p010_to_nv15(dst, packed->pitch, shadow->data[0].vaddr,
				   fb->pitches[0], fb->width, fb->height);
	histb_vdp_cv200_p010_to_nv15(dst + packed->pitch * fb->height, packed->pitch,
				   shadow->data[1].vaddr, fb->pitches[1],
				   fb->width, fb->height / 2);
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
	dma_wmb();
	packed->bank = bank;
	return 0;
}

static u32 histb_vdp_video_pitch(struct drm_framebuffer *fb, unsigned int index)
{
	if (fb->format->format == DRM_FORMAT_P010)
		return container_of(fb, struct histb_vdp_p010_fb, base)->pitch;
	return fb->pitches[index];
}

static u64 histb_vdp_video_address(struct drm_plane_state *state, unsigned int index)
{
	struct drm_framebuffer *fb = state->fb;
	struct histb_vdp_p010_fb *packed;

	if (fb->format->format != DRM_FORMAT_P010)
		return histb_vdp_plane_address(state, index);
	packed = container_of(fb, struct histb_vdp_p010_fb, base);
	return (u64)packed->dma + packed->bank * packed->size +
		(index ? (u64)packed->pitch * fb->height : 0) +
		(u64)((state->src.y1 >> 16) / (index ? 2 : 1)) * packed->pitch +
		(state->src.x1 >> 16) / 4 * 5;
}

/*
 * Selected BSP SPC060-Hi3798MV310, CV200 branch:
 * vdec_v2.1/drv_vdec_intf_k.c ConvertFrm_GetPixFormat/GetBufAddr exports
 * NV21_TILE and high-eight y_stride/16, low-two stride_2bit/32. CV200
 * VDH programs those strides as S*16 and S*8 respectively: S and S/4 here.
 * vo/vdp_v4_0/hal/common/drv_win_hal.c maps NV21_TILE to SP_TILE_64.
 * Its drv_xdr_adapter.c JudgeCmpAndUvorderByPixFormat sets uv_order=0 for
 * NV21_TILE (raw Cr,Cb), matching the VDH CPU detiler's explicit UV swap.
 * 98cv200/inc/vdp_define.h fixes SP_TILE_64=7 and VDP_DATA_WTH_10=1.
 * 98cv200/vdp_hal_intf.c forwards stAddress/stAddress_lb without repacking;
 * vdp_drv_vid.c SetVidCrop/SetVidAddr keeps all four bases intact, selects
 * tile crop in pixels, and uses NADDR/NCADDR for the low-two regions.
 * vdp_hal_vid.c SetSrcCropReso sets chroma y to luma y/2; SetTileStride
 * writes byte strides directly. Selected common hi3798cv200/hi_reg_vdp.h
 * defines 16-bit halves at e08/e48/e4c (not the MV310 register layout).
 */
static int histb_vdp_tile_source_check(const struct drm_plane_state *state)
{
	const struct drm_rect *src = &state->src;
	u32 width = drm_rect_width(src), height = drm_rect_height(src);

	/* Only the 64-pixel aligned hardware crop, no secondary horizontal crop. */
	if (state->rotation != DRM_MODE_ROTATE_0 ||
	    src->x1 < 0 || src->y1 < 0 || !width || !height ||
	    src->x2 > ((u64)state->fb->width << 16) ||
	    src->y2 > ((u64)state->fb->height << 16) ||
	    (src->x1 & ((64 << 16) - 1)) ||
	    ((src->y1 | width | height) & ((2 << 16) - 1)))
		return -EINVAL;
	return 0;
}

static int histb_vdp_program_video(struct histb_vdp *vdp, unsigned int id,
				  struct drm_plane_state *state,
				  const struct drm_plane_state *old_state,
				  bool hdr, bool copy)
{
	struct drm_framebuffer *fb = state->fb;
	struct histb_vdp_tile_layout tile;
	u32 sw, sh, dw, dh, x, y, off = id * HISTB_VDP_VIDEO_OFFSET;
	u32 pitch, cpitch, ctrl, i;
	bool was_enabled = vdp->video_enabled[id];
	bool tiled = fb && histb_vdp_tiled(fb);
	int ret;

	if (!histb_vdp_plane_visible(state)) {
		histb_vdp_video_disable(vdp, id);
		return 0;
	}
	if (tiled) {
		if (id || vdp->soc->mv310 ||
		    state->crtc->state->adjusted_mode.flags &
		    (DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_3D_MASK))
			return -EINVAL;
		ret = histb_vdp_tile_source_check(state);
		if (ret)
			return ret;
		ret = histb_vdp_tile_layout(fb, &tile);
		if (ret)
			return ret;
	}
	if (copy && !tiled) {
		ret = fb->format->format == DRM_FORMAT_P010 ?
			histb_vdp_copy_p010(state) : histb_vdp_copy_shadow(state);
		if (ret) {
			histb_vdp_video_disable(vdp, id);
			return ret;
		}
	}
	sw = drm_rect_width(&state->src) >> 16;
	sh = drm_rect_height(&state->src) >> 16;
	dw = drm_rect_width(&state->dst);
	dh = drm_rect_height(&state->dst);
	x = state->dst.x1;
	y = state->dst.y1;
	pitch = tiled ? tile.pitch : histb_vdp_video_pitch(fb, 0);
	cpitch = tiled ? tile.pitch : histb_vdp_video_pitch(fb, 1);
	vdp->video_enabled[id] = true;
	writel(tiled ? tile.address[0] : lower_32_bits(histb_vdp_video_address(state, 0)),
	       vdp->base + HISTB_VDP_V0_P0LADDR + off);
	writel(tiled ? tile.address[1] : lower_32_bits(histb_vdp_video_address(state, 1)),
	       vdp->base + HISTB_VDP_V0_P0CADDR + off);
	if (!id) {
		writel(tiled ? tile.address[2] : 0, vdp->base + HISTB_VDP_V0_NADDR);
		writel(tiled ? tile.address[3] : 0, vdp->base + HISTB_VDP_V0_NCADDR);
	}
	if (was_enabled && histb_vdp_plane_config_unchanged(old_state, state) &&
	    old_state->fb->pitches[1] == fb->pitches[1] &&
	    old_state->color_encoding == state->color_encoding &&
	    old_state->color_range == state->color_range)
		return 0;

	histb_vdp_v0_chain_init(vdp);
	writel(BIT(31), vdp->base + HISTB_VDP_V0_PRERD + off);
	writel(1, vdp->base + HISTB_VDP_V0_16REGIONENL + off);
	writel(0, vdp->base + HISTB_VDP_V0_MULTI_MODE + off);
	writel(0xf, vdp->base + HISTB_VDP_V0_SMMU_BYPASS0 + off);
	writel(0, vdp->base + HISTB_VDP_V0_SMMU_BYPASS1 + off);
	writel(0, vdp->base + HISTB_VDP_V0_SMMU_BYPASS2 + off);
	writel(0, vdp->base + HISTB_VDP_V0_SMMU_BYPASS3 + off);
	writel(pitch | cpitch << 16, vdp->base + HISTB_VDP_V0_P0STRIDE + off);
	if (!id) {
		u32 sx = tiled ? state->src.x1 >> 16 : 0;
		u32 sy = tiled ? state->src.y1 >> 16 : 0;

		writel(tiled ? tile.low_pitch | tile.low_pitch << 16 : 0,
		       vdp->base + HISTB_VDP_V0_TILE_STRIDE);
		writel(sx | sy << 16, vdp->base + HISTB_VDP_V0_TILE_LOFFSET);
		writel(sx | (sy / 2) << 16, vdp->base + HISTB_VDP_V0_TILE_COFFSET);
		writel(0, vdp->base + HISTB_VDP_V0_LADDROFFSET);
		writel(0, vdp->base + HISTB_VDP_V0_CADDROFFSET);
	}
	writel(HISTB_VDP_SIZE(sw, sh), vdp->base + HISTB_VDP_V0_IRESO + off);
	writel(HISTB_VDP_SIZE(dw, dh), vdp->base + HISTB_VDP_V0_ORESO + off);
	writel((sw - 1) << 12, vdp->base + HISTB_VDP_V0_CPOS + off);
	histb_vdp_program_zme(vdp, id, sw, sh, dw, dh);
	writel(HISTB_VDP_POS(x, y), vdp->base + HISTB_VDP_V0_DFPOS + off);
	writel(HISTB_VDP_POS(x + dw - 1, y + dh - 1),
	       vdp->base + HISTB_VDP_V0_DLPOS + off);
	writel(HISTB_VDP_POS(x, y), vdp->base + HISTB_VDP_V0_VFPOS + off);
	writel(HISTB_VDP_POS(x + dw - 1, y + dh - 1),
	       vdp->base + HISTB_VDP_V0_VLPOS + off);
	writel(0x04080200, vdp->base + HISTB_VDP_V0_BK + off);
	writel(0x04080200, vdp->base + HISTB_VDP_V0_MUTE_BK + off);
	writel(state->alpha >> 8, vdp->base + HISTB_VDP_V0_CBMPARA + off);
	/* Transparent letterbox must not hide a lower sibling video layer. */
	writel(0, vdp->base + HISTB_VDP_V0_ALPHA + off);
	if (id) {
		for (i = 1; i < 16; i++)
			writel(0, vdp->base + HISTB_VDP_V0_P0CTRL + off + i * 0x20);
		writel(sw - 1, vdp->base + HISTB_VDP_V0_P0RESO + off);
		writel(HISTB_VDP_POS(x, y), vdp->base + HISTB_VDP_V0_P0VFPOS + off);
		writel(HISTB_VDP_POS(x + sw - 1, y + sh - 1),
		       vdp->base + HISTB_VDP_V0_P0VLPOS + off);
		writel(1, vdp->base + HISTB_VDP_V0_P0CTRL + off);
	}
	if (hdr)
		histb_vdp_cv200_hdr_bt2020_csc(vdp->base);
	else
		histb_vdp_program_video_csc(vdp, id, state);
	ctrl = FIELD_PREP(HISTB_VDP_V0_IFMT, tiled ? HISTB_VDP_V0_IFMT_SP_TILE_64 :
			  HISTB_VDP_V0_IFMT_SP_420) |
		FIELD_PREP(HISTB_VDP_V0_REQ_CTRL, 3) | HISTB_VDP_V0_NONSECURE |
		(!tiled && fb->format->format != DRM_FORMAT_NV21 ? HISTB_VDP_V0_UV_ORDER : 0) |
		FIELD_PREP(HISTB_VDP_V0_CHM_RMODE, HISTB_VDP_V0_RMODE_PROGRESSIVE) |
		FIELD_PREP(HISTB_VDP_V0_LM_RMODE, HISTB_VDP_V0_RMODE_PROGRESSIVE) |
		FIELD_PREP(HISTB_VDP_V0_IFIR_MODE, 3) |
		FIELD_PREP(HISTB_VDP_V0_DATA_WIDTH, histb_vdp_ten_bit(fb) ? 1 : 0) |
		HISTB_VDP_V0_SURFACE_EN;
	writel(ctrl, vdp->base + HISTB_VDP_V0_CTRL + off);
	return 0;
}

static void histb_vdp_plane_atomic_update(struct drm_plane *plane,
					 struct drm_atomic_state *state)
{
	struct histb_vdp *vdp = to_histb_vdp(plane->dev);
	struct drm_crtc_state *crtc_state =
		drm_atomic_get_new_crtc_state(state, &vdp->pipe.crtc);
	struct histb_vdp_crtc_state *cs = to_histb_vdp_crtc_state(crtc_state);
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_plane_state *old = drm_atomic_get_old_plane_state(state, plane);
	unsigned int id = histb_vdp_plane_id(vdp, plane);
	bool copy = cs->plane_updates & BIT(id);
	int ret;

	/* A modeset already programmed every input before enabling scanout. */
	if (!vdp->powered || drm_atomic_crtc_needs_modeset(crtc_state))
		return;
	if (id < HISTB_VDP_GFX_COUNT)
		ret = histb_vdp_program_gfx(vdp, id, ps, old, &cs->mixer, copy);
	else
		ret = histb_vdp_program_video(vdp, id - HISTB_VDP_GFX_COUNT,
					      ps, old, cs->hdr, copy);
	if (ret) {
		cs->inputs_failed = true;
		drm_err(&vdp->drm, "failed to program %s: %pe\n",
			plane->name, ERR_PTR(ret));
	}
}

static void histb_vdp_pipe_update(struct drm_simple_display_pipe *pipe,
				  struct drm_plane_state *old_state)
{
	/* Unlike the installed new state, the retired state owns this commit. */
	histb_vdp_plane_atomic_update(&pipe->plane, old_state->state);
}

static int histb_vdp_overlay_check(struct drm_plane *plane,
				   struct drm_atomic_state *state)
{
	struct histb_vdp *vdp = to_histb_vdp(plane->dev);
	unsigned int id = histb_vdp_plane_id(vdp, plane);
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *cs;
	struct drm_framebuffer *fb = ps->fb;
	const struct drm_format_info *format;
	u32 width, height, align;
	unsigned int i, count;
	int ret;

	if (!ps->crtc)
		return fb ? -EINVAL : 0;
	cs = drm_atomic_get_crtc_state(state, ps->crtc);
	if (IS_ERR(cs))
		return PTR_ERR(cs);
	ret = drm_atomic_helper_check_plane_state(ps, cs,
		id == 3 ? 1 << 13 : DRM_PLANE_NO_SCALING,
		id == 3 ? 8 << 16 : DRM_PLANE_NO_SCALING, true, true);
	if (ret || !ps->visible)
		return ret;
	align = id < HISTB_VDP_GFX_COUNT ? 0xffff : (2 << 16) - 1;
	if ((ps->src.x1 | ps->src.y1 | drm_rect_width(&ps->src) |
	     drm_rect_height(&ps->src)) & align)
		return -EINVAL;
	width = drm_rect_width(&ps->src) >> 16;
	height = drm_rect_height(&ps->src) >> 16;
	if (histb_vdp_tiled(fb)) {
		struct histb_vdp_tile_layout tile;

		if (vdp->soc->mv310 || id != HISTB_VDP_GFX_COUNT ||
		    cs->adjusted_mode.flags & (DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_3D_MASK))
			return -EINVAL;
		ret = histb_vdp_tile_source_check(ps);
		return ret ? ret : histb_vdp_tile_layout(fb, &tile);
	}
	if (id < HISTB_VDP_GFX_COUNT && (width > 1920 || height > 1080))
		return -EINVAL;
	if (id >= HISTB_VDP_GFX_COUNT && histb_vdp_ten_bit(fb) &&
	    ((ps->src.x1 >> 16) & 3))
		return -EINVAL;
	format = fb->format->format == DRM_FORMAT_P010 ?
		drm_format_info(DRM_FORMAT_NV15) : fb->format;
	count = format->num_planes;
	for (i = 0; i < count; i++) {
		u32 pitch = id < HISTB_VDP_GFX_COUNT ? fb->pitches[i] :
			histb_vdp_video_pitch(fb, i);
		u32 rows = drm_format_info_plane_height(format, height, i);
		u64 address = id < HISTB_VDP_GFX_COUNT ?
			histb_vdp_plane_address(ps, i) : histb_vdp_video_address(ps, i);
		u64 end = address + (u64)(rows - 1) * pitch +
			drm_format_info_min_pitch(format, i,
				drm_format_info_plane_width(format, width, i)) - 1;

		if (!IS_ALIGNED(pitch, 16) || !IS_ALIGNED(address, 16) || end > U32_MAX ||
		    (id < HISTB_VDP_GFX_COUNT ? pitch / 16 : pitch) > U16_MAX)
			return -EINVAL;
	}
	return 0;
}

static int histb_vdp_begin_fb_access(struct drm_plane *plane,
				     struct drm_plane_state *state)
{
	struct drm_shadow_plane_state *shadow = to_drm_shadow_plane_state(state);
	int ret;

	/* Native scanout never vmaps or starts a CPU-access transaction. */
	if (state->fb && histb_vdp_tiled(state->fb))
		return 0;
	ret = drm_gem_begin_shadow_fb_access(plane, state);

	if (ret)
		return ret;
	if (state->fb && state->fb->format->format == DRM_FORMAT_P010 &&
	    (shadow->data[0].is_iomem || shadow->data[1].is_iomem)) {
		drm_gem_end_shadow_fb_access(plane, state);
		return -EOPNOTSUPP;
	}
	return 0;
}

static void histb_vdp_end_fb_access(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
	if (!state->fb || !histb_vdp_tiled(state->fb))
		drm_gem_end_shadow_fb_access(plane, state);
}

static bool histb_vdp_format_mod_supported(struct drm_plane *plane,
					 u32 format, u64 modifier)
{
	struct histb_vdp *vdp = to_histb_vdp(plane->dev);

	if (format == DRM_FORMAT_YUV420_10BIT)
		return !vdp->soc->mv310 &&
		       histb_vdp_plane_id(vdp, plane) == HISTB_VDP_GFX_COUNT &&
		       modifier == DRM_FORMAT_MOD_HISTB_VDH_10BIT;
	return modifier == DRM_FORMAT_MOD_LINEAR;
}


static const struct drm_plane_helper_funcs histb_vdp_overlay_helper_funcs = {
	.prepare_fb = drm_gem_plane_helper_prepare_fb,
	.begin_fb_access = histb_vdp_begin_fb_access,
	.end_fb_access = histb_vdp_end_fb_access,
	.atomic_check = histb_vdp_overlay_check,
	.atomic_update = histb_vdp_plane_atomic_update,
};

static const struct drm_plane_funcs histb_vdp_overlay_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.format_mod_supported = histb_vdp_format_mod_supported,
	DRM_GEM_SHADOW_PLANE_FUNCS,
};

static int histb_vdp_commit_inputs(struct histb_vdp *vdp,
				   struct drm_atomic_state *state,
				   struct histb_vdp_crtc_state *cs)
{
	unsigned int id, count = vdp->soc->mv310 ? 1 : HISTB_VDP_PLANE_COUNT;

	for (id = 0; id < count; id++) {
		struct drm_plane *plane = histb_vdp_plane(vdp, id);
		struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
		int ret;

		if (!ps)
			continue;
		if (id < HISTB_VDP_GFX_COUNT)
			ret = histb_vdp_program_gfx(vdp, id, ps, NULL, &cs->mixer, true);
		else
			ret = histb_vdp_program_video(vdp, id - HISTB_VDP_GFX_COUNT,
					      ps, NULL, cs->hdr, true);
		if (ret) {
			drm_err(&vdp->drm, "failed to program %s: %pe\n", plane->name, ERR_PTR(ret));
			return ret;
		}
	}
	return 0;
}

static void histb_vdp_program_gp0(struct histb_vdp *vdp,
				 const struct histb_vdp_mixer *mixer,
				 const struct histb_vdp_mixer *old)
{
	if (old && mixer->gp0_input == old->gp0_input &&
	    mixer->gp0_output == old->gp0_output &&
	    mixer->gp0_hsp == old->gp0_hsp && mixer->gp0_vsp == old->gp0_vsp &&
	    mixer->gp0_vsr == old->gp0_vsr)
		return;

	/* CONFIG_GFX_PQ live GP0 path, SPC060 pq_mng_gfxzme.c:1323-1361.
	 * The canvas is in G-layer coordinates, not output-mode coordinates.
	 * A NULL old state forces every write after modeset/reset/resume.
	 */
	writel(mixer->gp0_input, vdp->base + HISTB_VDP_GP0_IRESO);
	writel(mixer->gp0_output, vdp->base + HISTB_VDP_GP0_ORESO);
	writel(0, vdp->base + HISTB_VDP_GP0_DFPOS);
	writel(mixer->gp0_output, vdp->base + HISTB_VDP_GP0_DLPOS);
	writel(0, vdp->base + HISTB_VDP_GP0_VFPOS);
	writel(mixer->gp0_output, vdp->base + HISTB_VDP_GP0_VLPOS);
	writel(lower_32_bits(vdp->gp0_coefficients_dma),
	       vdp->base + HISTB_VDP_GP0_HCOEFAD);
	writel(lower_32_bits(vdp->gp0_coefficients_dma +
			    HISTB_VDP_CV200_GP0_VCOEF_OFFSET),
	       vdp->base + HISTB_VDP_GP0_VCOEFAD);
	writel(0, vdp->base + HISTB_VDP_GP0_ZME_HLOFFSET);
	writel(0, vdp->base + HISTB_VDP_GP0_ZME_HCOFFSET);
	writel(0, vdp->base + HISTB_VDP_GP0_ZME_VOFFSET);
	/* Our bilinear image has no vendor detail-enhancement coefficients. */
	writel(readl(vdp->base + HISTB_VDP_GP0_ZME_LTICTRL) & ~BIT(31),
	       vdp->base + HISTB_VDP_GP0_ZME_LTICTRL);
	writel(readl(vdp->base + HISTB_VDP_GP0_ZME_CTICTRL) & ~BIT(31),
	       vdp->base + HISTB_VDP_GP0_ZME_CTICTRL);
	writel(mixer->gp0_hsp, vdp->base + HISTB_VDP_GP0_ZME_HSP);
	writel(mixer->gp0_vsp, vdp->base + HISTB_VDP_GP0_ZME_VSP);
	writel(mixer->gp0_vsr, vdp->base + HISTB_VDP_GP0_ZME_VSR);
	writel(BIT(0) | BIT(1), vdp->base + HISTB_VDP_GP0_PARAUP);
}

static void histb_vdp_flush_inputs(struct histb_vdp *vdp,
				   const struct histb_vdp_mixer *mixer,
				   const struct histb_vdp_mixer *old_mixer)
{
	unsigned int i;
	u32 csc = 0;

	if (vdp->soc->mv310) {
		histb_vdp_latch_gfx(vdp);
		return;
	}
	histb_vdp_program_gp0(vdp, mixer, old_mixer);
	if (!old_mixer || mixer->graphics != old_mixer->graphics)
		writel(mixer->graphics, vdp->base + HISTB_VDP_MIXG0_MIX);
	if (!old_mixer || mixer->graphics_alpha != old_mixer->graphics_alpha)
		writel(mixer->graphics_alpha, vdp->base + HISTB_VDP_GP0_GALPHA);
	if (!old_mixer || mixer->video != old_mixer->video)
		writel(mixer->video, vdp->base + HISTB_VDP_MIXV0_MIX);
	if (!old_mixer || mixer->output != old_mixer->output)
		writel(mixer->output, vdp->base + HISTB_VDP_CBM_MIX1);
	/* Modesets pass no old mixer, restoring VP0 after timing changes/reset. */
	if (mixer->video && (!old_mixer || mixer->video != old_mixer->video)) {
		writel(HISTB_VDP_SIZE(vdp->display_width, vdp->display_height),
		       vdp->base + HISTB_VDP_VP0_IRESO);
		writel(0, vdp->base + HISTB_VDP_VP0_CTRL);
		writel(0, vdp->base + HISTB_VDP_VP0_DFPOS);
		writel(HISTB_VDP_POS(vdp->display_width - 1, vdp->display_height - 1),
		       vdp->base + HISTB_VDP_VP0_DLPOS);
		writel(0, vdp->base + HISTB_VDP_VP0_VFPOS);
		writel(HISTB_VDP_POS(vdp->display_width - 1, vdp->display_height - 1),
		       vdp->base + HISTB_VDP_VP0_VLPOS);
		writel(0xff, vdp->base + HISTB_VDP_VP0_GALPHA);
		writel(0xff, vdp->base + HISTB_VDP_VP0_ALPHA);
	}
	for (i = 0; i < HISTB_VDP_VIDEO_COUNT; i++) {
		if (vdp->video_enabled[i])
			csc |= BIT(i + 1);
		writel(1, vdp->base + HISTB_VDP_V0_UPD + i * HISTB_VDP_VIDEO_OFFSET);
	}
	writel(csc ? csc | BIT(0) : 0, vdp->base + HISTB_VDP_VDM_YUV2RGB_CTRL);
	writel(1, vdp->base + HISTB_VDP_VP0_UPD);
	writel(HISTB_VDP_HDR_REGUP, vdp->base + HISTB_VDP_HDR_UPD);
	histb_vdp_latch_gfx(vdp);
}


static void histb_vdp_program_axi(struct histb_vdp *vdp)
{
	u32 value;

	value = readl(vdp->base + HISTB_VDP_VOMIDORDER);
	value |= HISTB_VDP_VOMIDORDER_MID_ENABLE;
	writel(value, vdp->base + HISTB_VDP_VOMIDORDER);

	value = readl(vdp->base + HISTB_VDP_VOAXICTRL);
	value = (value & ~(HISTB_VDP_VOAXICTRL_MASK)) |
		HISTB_VDP_VOAXICTRL_VALUE;
	writel(value, vdp->base + HISTB_VDP_VOAXICTRL);

	value = readl(vdp->base + HISTB_VDP_VOAXICTRL1);
	value = (value & ~HISTB_VDP_VOAXICTRL1_FIELDS) |
		(HISTB_VDP_VOAXICTRL1_VALUE & HISTB_VDP_VOAXICTRL1_FIELDS);
	writel(value, vdp->base + HISTB_VDP_VOAXICTRL1);
}

static void histb_mv310_program_timing(struct histb_vdp *vdp,
				      const struct drm_display_mode *mode)
{
	u32 hfront = mode->hsync_start - mode->hdisplay;
	u32 hsync = mode->hsync_end - mode->hsync_start;
	u32 hblank = mode->htotal - mode->hsync_start;
	u32 vfront = mode->vsync_start - mode->vdisplay;
	u32 vsync = mode->vsync_end - mode->vsync_start;
	u32 vblank = mode->vtotal - mode->vsync_start;
	u32 last = MV310_POS(mode->hdisplay - 1, mode->vdisplay - 1);
	u32 inv = 0;

	/* MV310 master layout is unrelated to CV200 VOAXICTRL. */
	writel((readl(vdp->base + MV310_MST_OUTSTANDING) & ~0xff) | 0x2f,
	       vdp->base + MV310_MST_OUTSTANDING);
	writel((readl(vdp->base + MV310_MST_CTRL) & ~GENMASK(8, 0)) |
	       BIT(8) | BIT(4) | 1, vdp->base + MV310_MST_CTRL);
	writel(readl(vdp->base + MV310_MST_RCHN_PRIO0) | BIT(2),
	       vdp->base + MV310_MST_RCHN_PRIO0);
	/* No CV200 two-channel interleaver: MV310 BSP leaves it disabled. */
	writel(HISTB_VDP_VOCTRL_CLK_GATE_EN, vdp->base + HISTB_VDP_VOCTRL);
	writel(HISTB_VDP_DHD0_PROGRESSIVE | HISTB_VDP_DHD0_REGUP,
	       vdp->base + MV310_DHD0_CTRL);
	writel(0, vdp->base + MV310_DHD0_CTRL1);
	writel(MV310_POS(mode->hdisplay - 1, hblank - 1),
	       vdp->base + MV310_DHD0_HSYNC1);
	writel(hfront - 1, vdp->base + MV310_DHD0_HSYNC2);
	writel(MV310_POS(mode->vdisplay - 1, vblank - 1),
	       vdp->base + MV310_DHD0_VSYNC);
	writel(vfront - 1, vdp->base + MV310_DHD0_VSYNC2);
	writel(0, vdp->base + MV310_DHD0_VPLUS);
	writel(0, vdp->base + MV310_DHD0_VPLUS2);
	writel(MV310_POS(hsync - 1, vsync - 1),
	       vdp->base + MV310_DHD0_PWR);
	/* VTTHD1 line zero, frame mode: one vblank interrupt per frame. */
	writel(0, vdp->base + MV310_DHD0_VTTHD);
	writel(readl(vdp->base + MV310_VO_MUX) & ~BIT(12),
	       vdp->base + MV310_VO_MUX);
	/* RGB 4:4:4; no YUV swap, chroma subsampling or optional CSC. */
	writel(MV310_INTF_HDMI_RGB, vdp->base + MV310_INTF_HDMI_CTRL);
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		inv |= MV310_HDMI_HSYNC_INV;
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		inv |= MV310_HDMI_VSYNC_INV;
	writel(inv, vdp->base + MV310_INTF_HDMI_SYNC_INV);
	writel(0, vdp->base + MV310_INTF_HDMI_CLIP_L);
	writel(HISTB_VDP_DHD0_CLIP_FULL, vdp->base + MV310_INTF_HDMI_CLIP_H);

	writel(0, vdp->base + MV310_MIXG0_BKG);
	writel(0, vdp->base + MV310_MIXG0_BKALPHA);
	writel(1, vdp->base + MV310_MIXG0_MIX);
	writel(0, vdp->base + MV310_CBM_BKG1);
	writel(2, vdp->base + MV310_CBM_MIX1);
	/* VDP_GP_ORDER_NULL: direct RGB, bypass both ZME and CSC/HDR. */
	writel(0xff | FIELD_PREP(MV310_GP0_MUX1, 3) |
	       FIELD_PREP(MV310_GP0_MUX2, 2) |
	       FIELD_PREP(MV310_GP0_BOUT, 1) |
	       MV310_GP0_UPDATE_PROGRESSIVE | MV310_GP0_READ_PROGRESSIVE |
	       MV310_GP0_ENABLE, vdp->base + MV310_GP0_CTRL);
	writel(MV310_SIZE(mode->hdisplay, mode->vdisplay),
	       vdp->base + MV310_GP0_IRESO);
	writel(mode->hdisplay - 1, vdp->base + MV310_GP0_ZME_HINFO);
	writel((mode->vdisplay - 1) | BIT(18),
	       vdp->base + MV310_GP0_ZME_VINFO);
	writel(0, vdp->base + MV310_GP0_ZME_HSP);
	writel(0, vdp->base + MV310_GP0_ZME_VSP);
	writel(0, vdp->base + MV310_GP0_DFPOS);
	writel(last, vdp->base + MV310_GP0_DLPOS);
	writel(0, vdp->base + MV310_GP0_VFPOS);
	writel(last, vdp->base + MV310_GP0_VLPOS);
	writel(1, vdp->base + MV310_GP0_UPD);
	readl(vdp->base + MV310_GP0_UPD);
}

static void histb_vdp_dhd0_stage(struct histb_vdp *vdp, u32 set, u32 clear)
{
	u32 ctrl;

	ctrl = readl(vdp->base + vdp->soc->dhd0_ctrl);
	ctrl &= ~clear;
	ctrl |= set | HISTB_VDP_DHD0_REGUP;
	writel(ctrl, vdp->base + vdp->soc->dhd0_ctrl);
}

static void histb_vdp_dhd0_update(struct histb_vdp *vdp)
{
	histb_vdp_dhd0_stage(vdp, 0, 0);
}

static void histb_vdp_program_rgb_identity_path(struct histb_vdp *vdp)
{
	unsigned int i;
	u32 value;

	/*
	 * The simple DRM pipe submits RGB. Clear optional color transforms,
	 * but keep the CV200 graphics precision conversion below active.
	 */
	writel(readl(vdp->base + HISTB_VDP_GDM_CTRL) &
	       ~HISTB_VDP_GDM_ENABLES,
	       vdp->base + HISTB_VDP_GDM_CTRL);
	writel(readl(vdp->base + HISTB_VDP_GDM_RGB2LMS_CTRL) &
	       ~HISTB_VDP_GDM_RGB2LMS_ENABLE,
	       vdp->base + HISTB_VDP_GDM_RGB2LMS_CTRL);
	writel(readl(vdp->base + HISTB_VDP_HDR_CTRL) &
	       ~HISTB_VDP_HDR_ENABLES,
	       vdp->base + HISTB_VDP_HDR_CTRL);
	writel(readl(vdp->base + HISTB_VDP_G0_CSC_IDC) &
	       ~HISTB_VDP_G0_CSC_ENABLE,
	       vdp->base + HISTB_VDP_G0_CSC_IDC);
	writel(readl(vdp->base + HISTB_VDP_GP0_CSC_IDC) &
	       ~HISTB_VDP_GP0_CSC_ENABLE,
	       vdp->base + HISTB_VDP_GP0_CSC_IDC);
	writel(readl(vdp->base + HISTB_VDP_DHD0_CSC_IDC) &
	       ~HISTB_VDP_DHD0_CSC_ENABLE,
	       vdp->base + HISTB_VDP_DHD0_CSC_IDC);

	/* Clear every CVM processing latch, but preserve output metadata bits. */
	value = readl(vdp->base + HISTB_VDP_CVM_CTRL);
	value &= ~HISTB_VDP_CVM_PIPE_ENABLES;
	writel(value, vdp->base + HISTB_VDP_CVM_CTRL);
	writel(readl(vdp->base + HISTB_VDP_CVM1_RGB2YUV_CTRL) & ~BIT(0),
	       vdp->base + HISTB_VDP_CVM1_RGB2YUV_CTRL);
	writel(readl(vdp->base + HISTB_VDP_CVM2_RGB2YUV_CTRL) &
	       ~HISTB_VDP_CVM2_RGB2YUV_ENABLE,
	       vdp->base + HISTB_VDP_CVM2_RGB2YUV_CTRL);
	/*
	 * CV200 HIFB uses the HDR/CVM2 block even for RGB-to-RGB CSC. Its
	 * Q10 identity coefficient is scaled by 2^(15-10) / 2, giving Q14
	 * unity. Bypassing CVM2 does not preserve the graphics sample width:
	 * adjacent pixel lanes overflow differently before the DHD mixer.
	 */
	for (i = 0; i < 9; i++)
		writel(i % 4 == 0 ? HISTB_VDP_CVM2_CSC_UNITY : 0,
		       vdp->base + HISTB_VDP_CVM2_RGB2YUV_COEF0 + i * 4);
	writel(HISTB_VDP_CVM2_CSC_PRECISION |
	       HISTB_VDP_CVM2_CSC_SCALE_ENABLE,
	       vdp->base + HISTB_VDP_CVM2_RGB2YUV_SCALE2P);
	writel(0, vdp->base + HISTB_VDP_CVM2_RGB2YUV_MIN);
	writel(0x7fff, vdp->base + HISTB_VDP_CVM2_RGB2YUV_MAX);
	writel(0, vdp->base + HISTB_VDP_CVM2_RGB2YUV_OUT_DC0);
	writel(0, vdp->base + HISTB_VDP_CVM2_RGB2YUV_OUT_DC1);
	writel(0, vdp->base + HISTB_VDP_CVM2_RGB2YUV_OUT_DC2);
	writel(value | HISTB_VDP_CVM_ENABLE,
	       vdp->base + HISTB_VDP_CVM_CTRL);
	writel(readl(vdp->base + HISTB_VDP_CVM2_RGB2YUV_CTRL) |
	       HISTB_VDP_CVM2_RGB2YUV_ENABLE,
	       vdp->base + HISTB_VDP_CVM2_RGB2YUV_CTRL);
	writel(HISTB_VDP_HDR_REGUP, vdp->base + HISTB_VDP_HDR_UPD);
}

static void histb_vdp_program_timing(struct histb_vdp *vdp,
				     const struct drm_display_mode *mode)
{
	u32 hfront = mode->hsync_start - mode->hdisplay;
	u32 hsync = mode->hsync_end - mode->hsync_start;
	u32 hback = mode->htotal - mode->hsync_end;
	u32 vfront = mode->vsync_start - mode->vdisplay;
	u32 vsync = mode->vsync_end - mode->vsync_start;
	u32 vback = mode->vtotal - mode->vsync_end;
	u32 sync_inv = HISTB_VDP_DATE_HSYNC_INV;
	u32 ctrl;

	/* CV200's blanking fields include the sync pulse width. */
	writel((mode->hdisplay - 1) | ((hback + hsync - 1) << 16),
	       vdp->base + HISTB_VDP_DHD0_HSYNC1);
	writel((hfront - 1), vdp->base + HISTB_VDP_DHD0_HSYNC2);
	writel((mode->vdisplay - 1) | ((vback + vsync - 1) << 12) |
	       ((vfront - 1) << 22), vdp->base + HISTB_VDP_DHD0_VSYNC);
	writel(0, vdp->base + HISTB_VDP_DHD0_VPLUS);
	writel((hsync - 1) | ((vsync - 1) << 16),
	       vdp->base + HISTB_VDP_DHD0_PWR);
	writel(0, vdp->base + HISTB_VDP_DHD0_VTTHD);

	/* Match the same-device DHD default while HDMI HS/VS stay non-inverted. */
	writel(sync_inv, vdp->base + HISTB_VDP_DHD0_SYNC_INV);

	/* DHD0 is source zero for the HDMI path. */
	writel(readl(vdp->base + HISTB_VDP_VO_MUX) & ~BIT(3),
	       vdp->base + HISTB_VDP_VO_MUX);
	writel(0, vdp->base + HISTB_VDP_DHD0_DATA_SEL);

	/* The DRM framebuffer is RGB; keep the VDP path an explicit identity. */
	histb_vdp_program_rgb_identity_path(vdp);
	writel(0, vdp->base + HISTB_VDP_DHD0_HDMI_CLIP_L);
	writel(HISTB_VDP_DHD0_CLIP_FULL,
	       vdp->base + HISTB_VDP_DHD0_HDMI_CLIP_H);

	/* Route G0 through GP0 and make GP0 the only DHD0 mixer input. */
	writel(1, vdp->base + HISTB_VDP_MIXG0_MIX);
	writel(0, vdp->base + HISTB_VDP_MIXG0_BKALPHA);
	writel(0, vdp->base + HISTB_VDP_CBM_BKG1);
	writel(2, vdp->base + HISTB_VDP_CBM_MIX1);

	writel(HISTB_VDP_GP0_READ_PROGRESSIVE |
	       HISTB_VDP_GP0_ROUTE_ZME_CSC,
	       vdp->base + HISTB_VDP_GP0_CTRL);
	writel(HISTB_VDP_HDR_REGUP, vdp->base + HISTB_VDP_HDR_UPD);

	/* Stock stages HDMI mode before the separate interface-enable update. */
	ctrl = readl(vdp->base + HISTB_VDP_DHD0_CTRL);
	ctrl &= ~(HISTB_VDP_DHD0_ENABLE | HISTB_VDP_DHD0_CBAR_MASK);
	ctrl |= HISTB_VDP_DHD0_PROGRESSIVE | HISTB_VDP_DHD0_HDMI_MODE |
		HISTB_VDP_DHD0_TWOCHN_DEBUG;
	writel(ctrl | HISTB_VDP_DHD0_REGUP,
	       vdp->base + HISTB_VDP_DHD0_CTRL);
}

static int histb_vdp_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct histb_vdp *vdp = to_histb_vdp(pipe->crtc.dev);
	unsigned long flags;

	spin_lock_irqsave(&vdp->irq_lock, flags);
	vdp->vblank_enabled = true;
	if (vdp->powered)
		writel(readl(vdp->base + HISTB_VDP_VOINTMSK) |
		       HISTB_VDP_DHD0_VBLANK |
		       HISTB_VDP_ERROR_IRQS,
		       vdp->base + HISTB_VDP_VOINTMSK);
	spin_unlock_irqrestore(&vdp->irq_lock, flags);

	return 0;
}

static void histb_vdp_disable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct histb_vdp *vdp = to_histb_vdp(pipe->crtc.dev);
	unsigned long flags;

	spin_lock_irqsave(&vdp->irq_lock, flags);
	vdp->vblank_enabled = false;
	if (vdp->powered)
		writel(readl(vdp->base + HISTB_VDP_VOINTMSK) &
		       ~HISTB_VDP_DHD0_VBLANK,
		       vdp->base + HISTB_VDP_VOINTMSK);
	spin_unlock_irqrestore(&vdp->irq_lock, flags);
}

/* Both BSPs expose VOU reset completion through the two VDPVERSION words. */
static int histb_vdp_wait_reset_finished(struct histb_vdp *vdp)
{
	u32 version1;

	return readl_poll_timeout(vdp->base + HISTB_VDP_VERSION1, version1,
				  !version1 &&
				  !readl(vdp->base + HISTB_VDP_VERSION2),
				  2000, 80000);
}

static int histb_vdp_resources_enable(struct histb_vdp *vdp)
{
	int ret;

	guard(mutex)(&vdp->power_lock);
	if (vdp->resources_enabled)
		return 0;

	ret = clk_bulk_prepare_enable(vdp->num_clks, vdp->clks);
	if (ret)
		return ret;

	/* Start from a deterministic hardware state, independent of firmware. */
	ret = reset_control_assert(vdp->reset);
	if (ret) {
		clk_bulk_disable_unprepare(vdp->num_clks, vdp->clks);
		return ret;
	}
	mb();
	udelay(10);
	ret = histb_vdp_wait_reset_finished(vdp);
	if (ret) {
		drm_err(&vdp->drm,
			"VDP reset did not complete before deassert: VDPVERSION1=%08x VDPVERSION2=%08x\n",
			readl(vdp->base + HISTB_VDP_VERSION1),
			readl(vdp->base + HISTB_VDP_VERSION2));
		clk_bulk_disable_unprepare(vdp->num_clks, vdp->clks);
		return ret;
	}

	ret = reset_control_deassert(vdp->reset);
	if (ret) {
		clk_bulk_disable_unprepare(vdp->num_clks, vdp->clks);
		return ret;
	}
	mb();
	udelay(10);

	vdp->resources_enabled = true;
	vdp->zme[0].valid = false;
	vdp->zme[1].valid = false;
	/* Only CV200's BSP specifies this SRAM low-power value. */
	if (!vdp->soc->mv310)
		writel(HISTB_VDP_VOLOWPOWER_DEFAULT,
		       vdp->base + HISTB_VDP_VOLOWPOWER_CTRL);
	return 0;
}

static void histb_vdp_resources_disable(struct histb_vdp *vdp)
{
	guard(mutex)(&vdp->power_lock);

	if (!vdp->resources_enabled)
		return;

	if (!reset_control_assert(vdp->reset)) {
		/* The BSP waits for both version words before removing the clocks. */
		mb();
		udelay(10);
		if (histb_vdp_wait_reset_finished(vdp))
			drm_warn(&vdp->drm,
				 "VDP reset completion timeout while disabling: VDPVERSION1=%08x VDPVERSION2=%08x\n",
				 readl(vdp->base + HISTB_VDP_VERSION1),
				 readl(vdp->base + HISTB_VDP_VERSION2));
	}
	clk_bulk_disable_unprepare(vdp->num_clks, vdp->clks);
	vdp->resources_enabled = false;
}

static void histb_vdp_crtc_atomic_enable(struct drm_crtc *crtc,
					struct drm_atomic_state *state)
{
	struct histb_vdp *vdp = to_histb_vdp(crtc->dev);
	struct drm_simple_display_pipe *pipe = &vdp->pipe;
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	unsigned long flags;
	int ret;

	vdp->display_width = crtc_state->adjusted_mode.hdisplay;
	vdp->display_height = crtc_state->adjusted_mode.vdisplay;
	memset(vdp->video_enabled, 0, sizeof(vdp->video_enabled));
	ret = histb_vdp_resources_enable(vdp);
	if (ret) {
		drm_err(&vdp->drm, "failed to enable VDP resources: %pe\n",
			ERR_PTR(ret));
		return;
	}

	/* Cache the active period before a later commit replaces the CRTC mode. */
	vdp->close_delay_us = DIV_ROUND_UP_ULL(
		(u64)crtc_state->adjusted_mode.htotal *
		crtc_state->adjusted_mode.vtotal * 200000,
		(u64)crtc_state->adjusted_mode.clock * 99);

	/* Preserve the vblank request cached before the register clock started. */
	writel(HISTB_VDP_ERROR_IRQS |
	       (vdp->vblank_enabled ? HISTB_VDP_DHD0_VBLANK : 0),
	       vdp->base + HISTB_VDP_VOINTMSK);
	writel(~0, vdp->base + HISTB_VDP_VOMSKINTSTA);
	if (vdp->soc->mv310) {
		histb_mv310_program_timing(vdp, &crtc_state->adjusted_mode);
		goto program_plane;
	}
	histb_vdp_program_axi(vdp);
	/*
	 * CV200's production VO HAL ignores the caller's value and hard-codes
	 * twochn_mode=1 together with DHD0 twochn_debug=1. Program that state
	 * before the DHD0 staging sequence, as done by the vendor driver.
	 */
	writel(readl(vdp->base + HISTB_VDP_VOCTRL) |
		HISTB_VDP_VOCTRL_CLK_GATE_EN |
		HISTB_VDP_VOCTRL_TWOCHN_EN |
		HISTB_VDP_VOCTRL_TWOCHN_MODE,
	       vdp->base + HISTB_VDP_VOCTRL);
	/* Match stock's disabled DHD0 default/update boundary. */
	histb_vdp_dhd0_stage(vdp,
			     HISTB_VDP_DHD0_PROGRESSIVE |
			     HISTB_VDP_DHD0_TWOCHN_DEBUG,
			     HISTB_VDP_DHD0_ENABLE |
			     HISTB_VDP_DHD0_HDMI_MODE |
			     HISTB_VDP_DHD0_P2I_ENABLE |
			     HISTB_VDP_DHD0_CBAR_MASK);
	histb_vdp_program_timing(vdp, &crtc_state->adjusted_mode);
	/* Interface enable is a separate update, followed by stock's second request. */
	histb_vdp_dhd0_stage(vdp,
			     HISTB_VDP_DHD0_ENABLE |
			     HISTB_VDP_DHD0_PROGRESSIVE |
			     HISTB_VDP_DHD0_HDMI_MODE |
			     HISTB_VDP_DHD0_TWOCHN_DEBUG,
			     HISTB_VDP_DHD0_CBAR_MASK);
	histb_vdp_dhd0_update(vdp);
program_plane:
	ret = histb_vdp_commit_inputs(vdp, state,
				      to_histb_vdp_crtc_state(crtc_state));
	if (ret) {
		drm_err(&vdp->drm, "failed to program initial G0 plane: %pe\n",
			 ERR_PTR(ret));
		histb_vdp_resources_disable(vdp);
		return;
	}
	histb_vdp_flush_inputs(vdp, &to_histb_vdp_crtc_state(crtc_state)->mixer, NULL);
	if (vdp->soc->mv310) {
		/* Latch G0/GP0 before the separate DHD interface-enable request. */
		histb_vdp_dhd0_stage(vdp, HISTB_VDP_DHD0_ENABLE, 0);
		histb_vdp_dhd0_update(vdp);
	}
	spin_lock_irqsave(&vdp->irq_lock, flags);
	vdp->powered = true;
	if (vdp->vblank_enabled)
		writel(HISTB_VDP_DHD0_VBLANK | HISTB_VDP_ERROR_IRQS,
		       vdp->base + HISTB_VDP_VOINTMSK);
	spin_unlock_irqrestore(&vdp->irq_lock, flags);

	enable_irq(vdp->irq);

	/* Publish vblank availability only after the hardware IRQ is armed. */
	drm_crtc_vblank_on(&pipe->crtc);
}

static void histb_vdp_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct histb_vdp *vdp = to_histb_vdp(pipe->crtc.dev);
	struct drm_crtc *crtc = &pipe->crtc;
	unsigned long flags;
	unsigned int i;

	if (!vdp->powered)
		return;
	/* Stop the core from queueing vblank work before the pipe goes away. */
	drm_crtc_vblank_off(crtc);

	if (!vdp->soc->mv310) {
		for (i = 0; i < HISTB_VDP_VIDEO_COUNT; i++) {
			histb_vdp_video_disable(vdp, i);
			writel(1, vdp->base + HISTB_VDP_V0_UPD + i * HISTB_VDP_VIDEO_OFFSET);
		}
		for (i = 0; i < HISTB_VDP_GFX_COUNT; i++)
			histb_vdp_gfx_disable(vdp, i);
		writel(1, vdp->base + HISTB_VDP_VP0_UPD);
		writel(0, vdp->base + HISTB_VDP_CBM_MIX1);
	}
	spin_lock_irqsave(&vdp->irq_lock, flags);
	writel(0, vdp->base + HISTB_VDP_VOINTMSK);
	writel(HISTB_VDP_DHD0_VBLANK,
	       vdp->base + HISTB_VDP_VOMSKINTSTA);
	writel(readl(vdp->base + vdp->soc->g0_ctrl) &
	       ~HISTB_VDP_G0_ENABLE, vdp->base + vdp->soc->g0_ctrl);
	if (vdp->soc->mv310)
		writel(readl(vdp->base + MV310_GP0_CTRL) & ~MV310_GP0_ENABLE,
		       vdp->base + MV310_GP0_CTRL);
	/* G0 is latched through GP0 on the production graphics path. */
	histb_vdp_latch_gfx(vdp);
	writel((readl(vdp->base + vdp->soc->dhd0_ctrl) &
		~(HISTB_VDP_DHD0_ENABLE | HISTB_VDP_DHD0_CBAR_MASK)) |
	       HISTB_VDP_DHD0_REGUP, vdp->base + vdp->soc->dhd0_ctrl);
	spin_unlock_irqrestore(&vdp->irq_lock, flags);

	/*
	 * CV200 BSP requires two old-mode periods for DHD disable to settle.
	 * Do not return to bridge post_disable (which gates the pixel clock)
	 * while REGUP can still be pending. Allow the bridge's 1% clock-rate
	 * tolerance, including fractional-refresh modes and slower PLL plans.
	 */
	usleep_range(vdp->close_delay_us, vdp->close_delay_us + 1000);

	disable_irq(vdp->irq);

	spin_lock_irqsave(&vdp->irq_lock, flags);
	vdp->powered = false;
	spin_unlock_irqrestore(&vdp->irq_lock, flags);

}

static void histb_vdp_encoder_atomic_disable(struct drm_encoder *encoder,
					     struct drm_atomic_state *state)
{
	struct drm_simple_display_pipe *pipe =
		container_of(encoder, struct drm_simple_display_pipe, encoder);

	/* Bridge post_disable gates the pixel clock before the CRTC callback. */
	histb_vdp_pipe_disable(pipe);
}

static const struct drm_encoder_helper_funcs histb_vdp_encoder_helper_funcs = {
	.atomic_disable = histb_vdp_encoder_atomic_disable,
};

static enum drm_mode_status
histb_vdp_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
			  const struct drm_display_mode *mode)
{
	struct histb_vdp *vdp = to_histb_vdp(pipe->crtc.dev);
	u32 hfront, hsync, hback;
	u32 vfront, vsync, vback;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;
	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;
	if (mode->clock <= 0)
		return MODE_CLOCK_LOW;
	if (vdp->soc->mv310) {
		if (mode->clock > 594000)
			return MODE_CLOCK_HIGH;
		if ((mode->flags & ~(DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NHSYNC |
				     DRM_MODE_FLAG_PVSYNC | DRM_MODE_FLAG_NVSYNC)) ||
		    mode->vscan > 1 || drm_mode_vrefresh(mode) > 60)
			return MODE_BAD;
	}

	if (mode->hdisplay < HISTB_VDP_MIN_WIDTH ||
	    mode->hdisplay > vdp->soc->max_width)
		return MODE_BAD_HVALUE;
	if (mode->vdisplay < HISTB_VDP_MIN_HEIGHT ||
	    mode->vdisplay > vdp->soc->max_height)
		return MODE_BAD_VVALUE;

	if (mode->hsync_start <= mode->hdisplay ||
	    mode->hsync_end <= mode->hsync_start ||
	    mode->htotal <= mode->hsync_end)
		return MODE_H_ILLEGAL;
	if (mode->vsync_start <= mode->vdisplay ||
	    mode->vsync_end <= mode->vsync_start ||
	    mode->vtotal <= mode->vsync_end)
		return MODE_V_ILLEGAL;

	hfront = mode->hsync_start - mode->hdisplay;
	hsync = mode->hsync_end - mode->hsync_start;
	hback = mode->htotal - mode->hsync_end;
	vfront = mode->vsync_start - mode->vdisplay;
	vsync = mode->vsync_end - mode->vsync_start;
	vback = mode->vtotal - mode->vsync_end;

	if (hfront > 0x10000 || hback + hsync > 0x10000)
		return MODE_BAD_HVALUE;
	if (hsync > 0x10000)
		return MODE_HSYNC_WIDE;
	if (vfront > (vdp->soc->mv310 ? 0x10000 : 0x400) ||
	    vback + vsync > (vdp->soc->mv310 ? 0x10000 : 0x400))
		return MODE_BAD_VVALUE;
	if (vsync > 0x100)
		return MODE_VSYNC_WIDE;

	return MODE_OK;
}

static int histb_vdp_pipe_check(struct drm_simple_display_pipe *pipe,
				struct drm_plane_state *plane_state,
				struct drm_crtc_state *crtc_state)
{
	struct histb_vdp *vdp = to_histb_vdp(pipe->crtc.dev);
	const struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	struct drm_framebuffer *fb = plane_state->fb;
	u64 address, end;
	u32 width, height;
	int ret;

	ret = drm_atomic_helper_check_plane_state(plane_state, crtc_state,
						  vdp->soc->mv310 ? DRM_PLANE_NO_SCALING : 1 << 14,
						  vdp->soc->mv310 ? DRM_PLANE_NO_SCALING : 4 << 16,
						  false, true);
	if (ret || !plane_state->visible)
		return ret;
	if (histb_vdp_pipe_mode_valid(pipe, mode) != MODE_OK)
		return -EINVAL;
	width = drm_rect_width(&plane_state->src) >> 16;
	height = drm_rect_height(&plane_state->src) >> 16;
	if (width != mode->hdisplay || height != mode->vdisplay) {
		/* Deliberately limit scaling to HD canvases and HD/UHD output.
		 * HIFB advertises 1920x1080 input (drv_hifb_config.h:353-354).
		 * Live PQ chooses VH/four taps for input <=1920; do not infer
		 * wider-input support from register widths or its tiny-ZME path.
		 */
		if (vdp->soc->mv310 || width < 1280 || height < 720 ||
		    width > 1920 || height > 1080 || mode->hdisplay < 1280 ||
		    mode->vdisplay < 720 || mode->hdisplay > 3840 ||
		    mode->vdisplay > 2160)
			return -EINVAL;
	}

	if (!IS_ALIGNED(fb->pitches[0], HISTB_VDP_G0_STRIDE_UNIT) ||
	    fb->pitches[0] / HISTB_VDP_G0_STRIDE_UNIT >
		FIELD_MAX(HISTB_VDP_G0_STRIDE_MASK))
		return -EINVAL;

	if ((plane_state->src.x1 | plane_state->src.y1 |
	     drm_rect_width(&plane_state->src) | drm_rect_height(&plane_state->src)) &
	    0xffff)
		return -EINVAL;
	address = histb_vdp_plane_address(plane_state, 0);
	end = address + (u64)((drm_rect_height(&plane_state->src) >> 16) - 1) *
		fb->pitches[0] + drm_format_info_min_pitch(fb->format, 0,
						   drm_rect_width(&plane_state->src) >> 16) - 1;
	if (!IS_ALIGNED(address, 16) || end > U32_MAX)
		return -EINVAL;

	return 0;
}

static int histb_vdp_primary_atomic_check(struct drm_plane *plane,
					  struct drm_atomic_state *state)
{
	struct histb_vdp *vdp = to_histb_vdp(plane->dev);
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(state, &vdp->pipe.crtc);

	return histb_vdp_pipe_check(&vdp->pipe, ps, cs);
}

static int histb_vdp_check_gp0(struct histb_vdp_mixer *mixer,
			       const struct drm_display_mode *mode,
			       const struct drm_plane_state *primary)
{
	u32 width, height, hratio, vratio;

	mixer->gp0_output = HISTB_VDP_SIZE(mode->hdisplay, mode->vdisplay);
	mixer->gp0_input = mixer->gp0_output;
	mixer->gp0_hsp = BIT(20) | HISTB_VDP_GP0_ZME_VH;
	mixer->gp0_vsr = BIT(12);
	if (!primary)
		return 0;
	width = drm_rect_width(&primary->src) >> 16;
	height = drm_rect_height(&primary->src) >> 16;
	if (width == mode->hdisplay && height == mode->vdisplay)
		return 0;
	/* GP0 scales the composition of G0/G1/G3, not one member. The full
	 * atomic snapshot includes unchanged graphics; VP0 video is independent.
	 */
	if (mixer->graphics_count != 1)
		return -EINVAL;
	hratio = div_u64((u64)width << 20, mode->hdisplay);
	vratio = div_u64((u64)height << 12, mode->vdisplay);
	if (!hratio || hratio > FIELD_MAX(HISTB_VDP_GP0_ZME_H_RATIO) ||
	    !vratio || vratio > FIELD_MAX(HISTB_VDP_GP0_ZME_V_RATIO))
		return -EINVAL;
	mixer->gp0_input = HISTB_VDP_SIZE(width, height);
	mixer->gp0_hsp = hratio | HISTB_VDP_GP0_ZME_VH |
		HISTB_VDP_GP0_ZME_H_FIR | HISTB_VDP_GP0_ZME_H_ENABLE;
	mixer->gp0_vsp = HISTB_VDP_GP0_ZME_V_FIR | HISTB_VDP_GP0_ZME_V_ENABLE;
	/* Match the live PQ median policy, including alpha, in each direction. */
	if (width < mode->hdisplay)
		mixer->gp0_hsp |= HISTB_VDP_GP0_ZME_MID;
	if (height < mode->vdisplay)
		mixer->gp0_vsp |= HISTB_VDP_GP0_ZME_MID;
	mixer->gp0_vsr = vratio;
	return 0;
}


static const struct drm_simple_display_pipe_funcs histb_vdp_pipe_funcs = {
	.mode_valid = histb_vdp_pipe_mode_valid,
	.disable = histb_vdp_pipe_disable,
	.update = histb_vdp_pipe_update,
	.check = histb_vdp_pipe_check,
	.enable_vblank = histb_vdp_enable_vblank,
	.disable_vblank = histb_vdp_disable_vblank,
	.reset_crtc = histb_vdp_reset_crtc,
	.duplicate_crtc_state = histb_vdp_duplicate_crtc_state,
	.destroy_crtc_state = histb_vdp_destroy_crtc_state,
	DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS,
};

static int histb_vdp_atomic_check(struct drm_device *drm,
				  struct drm_atomic_state *state)
{
	struct histb_vdp *vdp = to_histb_vdp(drm);
	struct drm_crtc *crtc = &vdp->pipe.crtc;
	struct drm_crtc_state *cs;
	struct drm_connector_state *connector;
	struct histb_vdp_crtc_state *new;
	struct drm_plane *plane;
	struct drm_plane_state *ps;
	struct drm_plane_state *planes[HISTB_VDP_PLANE_COUNT] = {};
	unsigned int id, rank, g = 0, v = 0, changes = 0;
	int i, group = -1, first = -1, ret;
	u32 updates = 0;
	u8 alpha = 0xff;

	/* Capture caller-requested updates before adding the shared processing group. */
	for_each_new_plane_in_state(state, plane, ps, i)
		updates |= BIT(histb_vdp_plane_id(vdp, plane));
	cs = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(cs))
		return PTR_ERR(cs);
	new = to_histb_vdp_crtc_state(cs);
	new->plane_updates = updates;
	new->inputs_failed = false;
	connector = drm_atomic_get_new_connector_state(state, vdp->connector);
	if (connector)
		new->hdr = !!connector->hdr_output_metadata;
	if (new->hdr != to_histb_vdp_crtc_state(crtc->state)->hdr)
		cs->mode_changed = true;
	if (!vdp->soc->mv310) {
		/* GP0/VP0 constraints need an atomic snapshot of all affected inputs. */
		ret = drm_atomic_add_affected_planes(state, crtc);
		if (ret)
			return ret;
	}
	ret = drm_atomic_helper_check(drm, state);
	if (ret)
		return ret;
	new->mixer = (struct histb_vdp_mixer) { .graphics_alpha = 0xff };
	if (!cs->active)
		return 0;
	if (vdp->soc->mv310)
		return new->hdr ? -EINVAL : 0;
	for (id = 0; id < HISTB_VDP_PLANE_COUNT; id++) {
		ps = drm_atomic_get_new_plane_state(state, histb_vdp_plane(vdp, id));
		if (histb_vdp_plane_visible(ps))
			planes[id] = ps;
	}
	/* Only group-contiguous order is representable by GP0/VP0 -> DHD0. */
	for (rank = 0; rank < HISTB_VDP_PLANE_COUNT; rank++) {
		for (id = 0; id < HISTB_VDP_PLANE_COUNT; id++) {
			int next = id < HISTB_VDP_GFX_COUNT ? 0 : 1;

			ps = planes[id];
			if (!ps || ps->normalized_zpos != rank)
				continue;
			if (group != next) {
				if (group >= 0 && ++changes > 1)
					return -EINVAL;
				group = next;
				if (first < 0)
					first = next;
			}
			if (!next) {
				new->mixer.graphics |= (histb_vdp_gfx_ids[id] + 1) << (4 * g++);
				alpha = ps->alpha >> 8;
			} else {
				new->mixer.video |= (id - HISTB_VDP_GFX_COUNT + 1) << (4 * v++);
				if (ps->color_encoding == DRM_COLOR_YCBCR_BT2020 && !new->hdr)
					return -EINVAL;
			}
		}
	}
	new->mixer.graphics_count = g;
	ret = histb_vdp_check_gp0(&new->mixer, &cs->adjusted_mode, planes[0]);
	if (ret)
		return ret;
	if (g == 1)
		new->mixer.graphics_alpha = alpha;
	if (g > 1) {
		for (id = 0; id < HISTB_VDP_GFX_COUNT; id++) {
			ps = planes[id];
			if (ps && ps->fb->format->has_alpha &&
			    ps->pixel_blend_mode == DRM_MODE_BLEND_PREMULTI && ps->alpha != 0xffff)
				return -EINVAL;
		}
	}
	new->mixer.output = g && v ? (first ? 1 | (2 << 4) : 2 | (1 << 4)) :
			   g ? 2 : v ? 1 : 0;
	if (new->hdr) {
		ps = planes[HISTB_VDP_GFX_COUNT];
		if (g || v != 1 || !ps || !histb_vdp_ten_bit(ps->fb) ||
		    ps->color_encoding != DRM_COLOR_YCBCR_BT2020 ||
		    ps->color_range != DRM_COLOR_YCBCR_LIMITED_RANGE || ps->alpha != 0xffff)
			return -EINVAL;
	}
	return 0;
}

static void histb_vdp_p010_fb_destroy(struct drm_framebuffer *fb)
{
	struct histb_vdp_p010_fb *packed = container_of(fb, struct histb_vdp_p010_fb, base);

	if (packed->packed)
		dma_free_wc(fb->dev->dev, 2 * packed->size, packed->packed, packed->dma);
	drm_gem_fb_destroy(fb);
}

static const struct drm_framebuffer_funcs histb_vdp_p010_fb_funcs = {
	.destroy = histb_vdp_p010_fb_destroy,
	.create_handle = drm_gem_fb_create_handle,
};

static struct drm_framebuffer *
histb_vdp_fb_create(struct drm_device *drm, struct drm_file *file,
		    const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct histb_vdp *vdp = to_histb_vdp(drm);
	struct histb_vdp_p010_fb *packed;
	struct drm_framebuffer *fb;
	struct histb_vdp_tile_layout tile;
	u64 size;
	int ret;

	if (mode_cmd->pixel_format == DRM_FORMAT_YUV420_10BIT) {
		if (vdp->soc->mv310 ||
		    !(mode_cmd->flags & DRM_MODE_FB_MODIFIERS) ||
		    mode_cmd->modifier[0] != DRM_FORMAT_MOD_HISTB_VDH_10BIT)
			return ERR_PTR(-EINVAL);
		fb = drm_gem_fb_create(drm, file, mode_cmd);
		if (IS_ERR(fb))
			return fb;
		ret = histb_vdp_tile_layout(fb, &tile);
		if (ret) {
			drm_framebuffer_put(fb);
			return ERR_PTR(ret);
		}
		return fb;
	}
	/* No non-linear interpretation of the existing RGB/NV12/NV15/P010 ABI. */
	if (mode_cmd->modifier[0] != DRM_FORMAT_MOD_LINEAR)
		return ERR_PTR(-EINVAL);
	if (!IS_ALIGNED(mode_cmd->pitches[0], HISTB_VDP_G0_STRIDE_UNIT) ||
	    mode_cmd->pitches[0] / HISTB_VDP_G0_STRIDE_UNIT > U16_MAX ||
	    !IS_ALIGNED(mode_cmd->offsets[0], HISTB_VDP_G0_STRIDE_UNIT))
		return ERR_PTR(-EINVAL);
	if (mode_cmd->pixel_format != DRM_FORMAT_P010)
		return drm_gem_fb_create(drm, file, mode_cmd);
	if (vdp->soc->mv310 || (mode_cmd->width | mode_cmd->height) & 1 ||
	    mode_cmd->modifier[0] != DRM_FORMAT_MOD_LINEAR ||
	    mode_cmd->modifier[1] != DRM_FORMAT_MOD_LINEAR)
		return ERR_PTR(-EINVAL);
	packed = kzalloc(sizeof(*packed), GFP_KERNEL);
	if (!packed)
		return ERR_PTR(-ENOMEM);
	packed->pitch = ALIGN(DIV_ROUND_UP(mode_cmd->width, 4) * 5, 16);
	size = (u64)packed->pitch * mode_cmd->height / 2 * 3;
	if (packed->pitch > U16_MAX || size > SIZE_MAX / 2) {
		kfree(packed);
		return ERR_PTR(-EINVAL);
	}
	packed->size = size;
	ret = drm_gem_fb_init_with_funcs(drm, &packed->base, file, mode_cmd,
				       &histb_vdp_p010_fb_funcs);
	if (ret) {
		kfree(packed);
		return ERR_PTR(ret);
	}
	/* A framebuffer owns two packed banks until its last scanout state retires. */
	packed->packed = dma_alloc_wc(drm->dev, 2 * packed->size, &packed->dma, GFP_KERNEL);
	if (!packed->packed) {
		drm_gem_fb_destroy(&packed->base);
		return ERR_PTR(-ENOMEM);
	}
	memset(packed->packed, 0, 2 * packed->size);
	return &packed->base;
}

static const struct drm_mode_config_funcs histb_vdp_mode_config_funcs = {
	.fb_create = histb_vdp_fb_create,
	.atomic_check = histb_vdp_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};

/* Call only with live resources (atomic tail or the power-locked snapshot). */
static void histb_vdp_print_regs(struct drm_printer *p, struct histb_vdp *vdp)
{
	unsigned int i;

	drm_printf(p, "VDP g0=%08x dhd0=%08x irq=%08x mask=%08x\n",
		   readl(vdp->base + vdp->soc->g0_ctrl),
		   readl(vdp->base + vdp->soc->dhd0_ctrl),
		   readl(vdp->base + HISTB_VDP_VOINTSTA),
		   readl(vdp->base + HISTB_VDP_VOINTMSK));
	if (vdp->soc->mv310)
		return;
	drm_printf(p, "VDP VO=%08x uf=%08x master=%08x/%08x mixg=%08x\n",
		   readl(vdp->base + HISTB_VDP_VOCTRL),
		   readl(vdp->base + 0x0050),
		   readl(vdp->base + HISTB_VDP_VOMASTERSEL),
		   readl(vdp->base + 0x003c),
		   readl(vdp->base + HISTB_VDP_MIXG0_MIX));
	for (i = 0; i < HISTB_VDP_GFX_COUNT; i++) {
		u32 off = histb_vdp_gfx_ids[i] * HISTB_VDP_GFX_OFFSET;

		drm_printf(p, "VDP G%u=%08x addr=%08x stride=%08x in=%08x src=%08x df=%08x/%08x vf=%08x/%08x\n",
			   histb_vdp_gfx_ids[i],
			   readl(vdp->base + HISTB_VDP_G0_CTRL + off),
			   readl(vdp->base + HISTB_VDP_G0_ADDR + off),
			   readl(vdp->base + HISTB_VDP_G0_STRIDE + off),
			   readl(vdp->base + HISTB_VDP_G0_IRESO + off),
			   readl(vdp->base + HISTB_VDP_G0_SFPOS + off),
			   readl(vdp->base + HISTB_VDP_G0_DFPOS + off),
			   readl(vdp->base + HISTB_VDP_G0_DLPOS + off),
			   readl(vdp->base + HISTB_VDP_G0_VFPOS + off),
			   readl(vdp->base + HISTB_VDP_G0_VLPOS + off));
	}
	drm_printf(p, "VDP V0=%08x in=%08x out=%08x y=%08x c=%08x stride=%08x region=%08x p0=%08x\n",
		   readl(vdp->base + HISTB_VDP_V0_CTRL),
		   readl(vdp->base + HISTB_VDP_V0_IRESO),
		   readl(vdp->base + HISTB_VDP_V0_ORESO),
		   readl(vdp->base + HISTB_VDP_V0_P0LADDR),
		   readl(vdp->base + HISTB_VDP_V0_P0CADDR),
		   readl(vdp->base + HISTB_VDP_V0_P0STRIDE),
		   readl(vdp->base + HISTB_VDP_V0_16REGIONENL),
		   readl(vdp->base + HISTB_VDP_V0_P0CTRL));
	drm_printf(p, "VDP ZME h=%08x v=%08x ratio=%08x hcoef=%08x vcoef=%08x\n",
		   readl(vdp->base + HISTB_VDP_V0_HSP),
		   readl(vdp->base + HISTB_VDP_V0_VSP),
		   readl(vdp->base + HISTB_VDP_V0_VSR),
		   readl(vdp->base + HISTB_VDP_V0_HLCOEFAD),
		   readl(vdp->base + HISTB_VDP_V0_VLCOEFAD));
	drm_printf(p, "VDP VDM=%08x csc=%08x scale=%08x dc=%08x/%08x/%08x rgb=%08x/%08x/%08x/%08x CVM=%08x/%08x/%08x\n",
		   readl(vdp->base + HISTB_VDP_VDM_CTRL),
		   readl(vdp->base + HISTB_VDP_VDM_YUV2RGB_CTRL),
		   readl(vdp->base + HISTB_VDP_VDM_YUV2RGB_SCALE2P),
		   readl(vdp->base + HISTB_VDP_VDM_YUV2RGB_IN_DC00),
		   readl(vdp->base + HISTB_VDP_VDM_YUV2RGB_IN_DC00 + 4),
		   readl(vdp->base + HISTB_VDP_VDM_YUV2RGB_IN_DC00 + 8),
		   readl(vdp->base + HISTB_VDP_VDM_RGB2LMS_CTRL),
		   readl(vdp->base + HISTB_VDP_VDM_RGB2LMS_SCALE2P),
		   readl(vdp->base + HISTB_VDP_VDM_RGB2LMS_MIN),
		   readl(vdp->base + HISTB_VDP_VDM_RGB2LMS_MAX),
		   readl(vdp->base + HISTB_VDP_CVM_CTRL),
		   readl(vdp->base + HISTB_VDP_CVM1_LMS2IPT_CTRL),
		   readl(vdp->base + HISTB_VDP_CVM1_LMS2IPT_SCALE2P));
	drm_printf(p, "VDP VP0=%08x/%08x/%08x CBM=%08x G0alpha=%08x GP0=%08x mixalpha=%08x\n",
		   readl(vdp->base + HISTB_VDP_VP0_CTRL),
		   readl(vdp->base + HISTB_VDP_VP0_IRESO),
		   readl(vdp->base + HISTB_VDP_VP0_GALPHA),
		   readl(vdp->base + HISTB_VDP_CBM_MIX1),
		   readl(vdp->base + HISTB_VDP_G0_CBMPARA),
		   readl(vdp->base + HISTB_VDP_GP0_CTRL),
		   readl(vdp->base + HISTB_VDP_MIXG0_BKALPHA));
}

/* Plane-only V0 commits need the same latch/event boundary as G0 commits. */
static void histb_vdp_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct drm_device *drm = state->dev;
	struct histb_vdp *vdp = to_histb_vdp(drm);
	struct drm_crtc *crtc = &vdp->pipe.crtc;
	struct drm_crtc_state *new_state = drm_atomic_get_new_crtc_state(state, crtc);
	unsigned long flags;

	drm_atomic_helper_commit_modeset_disables(drm, state);
	/* Clocks and vblank must be ready before either plane is programmed. */
	drm_atomic_helper_commit_modeset_enables(drm, state);
	drm_atomic_helper_commit_planes(drm, state, DRM_PLANE_COMMIT_ACTIVE_ONLY);
	if (new_state && new_state->active && vdp->powered &&
	    !drm_atomic_crtc_needs_modeset(new_state)) {
		struct histb_vdp_crtc_state *cs = to_histb_vdp_crtc_state(new_state);
		struct histb_vdp_crtc_state *old = to_histb_vdp_crtc_state(
			drm_atomic_get_old_crtc_state(state, crtc));

		if (!cs->inputs_failed)
			histb_vdp_flush_inputs(vdp, &cs->mixer,
					       old->inputs_failed ? NULL : &old->mixer);
	}
	spin_lock_irqsave(&drm->event_lock, flags);
	if (new_state && new_state->event) {
		struct drm_pending_vblank_event *event = new_state->event;

		new_state->event = NULL;
		if (new_state->active && vdp->powered && !drm_crtc_vblank_get(crtc))
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
	}
	spin_unlock_irqrestore(&drm->event_lock, flags);
	drm_atomic_helper_fake_vblank(state);
	drm_atomic_helper_commit_hw_done(state);
	drm_atomic_helper_wait_for_vblanks(drm, state);
	drm_atomic_helper_cleanup_planes(drm, state);
}

static const struct drm_mode_config_helper_funcs histb_vdp_mode_config_helpers = {
	.atomic_commit_tail = histb_vdp_atomic_commit_tail,
};

DEFINE_DRM_GEM_DMA_FOPS(histb_vdp_fops);

static int histb_vdp_dumb_create(struct drm_file *file,
				 struct drm_device *drm,
				 struct drm_mode_create_dumb *args)
{
	u64 pitch = ALIGN(DIV_ROUND_UP_ULL((u64)args->width * args->bpp, 8),
			  HISTB_VDP_G0_STRIDE_UNIT);

	if (pitch > U32_MAX)
		return -EINVAL;

	args->pitch = pitch;
	args->size = pitch * args->height;

	return drm_gem_dma_dumb_create_internal(file, drm, args);
}

static const struct drm_driver histb_vdp_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &histb_vdp_fops,
	.dumb_create = histb_vdp_dumb_create,
	.gem_prime_import_sg_table = histb_vdp_prime_import_sg_table,
	.name = "histb-vdp",
	.desc = "HiSilicon HiSTB video display processor",
	.date = "20260801",
};

static int histb_vdp_regs_show(struct seq_file *m, void *unused)
{
	struct drm_debugfs_entry *entry = m->private;
	struct drm_device *drm = entry->dev;
	struct histb_vdp *vdp = to_histb_vdp(drm);
	int idx;
	struct drm_printer p = drm_seq_file_printer(m);

	if (!drm_dev_enter(drm, &idx))
		return -ENODEV;
	mutex_lock(&vdp->power_lock);
	seq_printf(m, "underflows=%lld master_errors=%lld\n",
		   atomic64_read(&vdp->underflows), atomic64_read(&vdp->master_errors));
	if (!vdp->resources_enabled || !READ_ONCE(vdp->powered)) {
		seq_puts(m, "VDP inactive (clocks/reset off)\n");
		goto out;
	}
	histb_vdp_print_regs(&p, vdp);
out:
	mutex_unlock(&vdp->power_lock);
	drm_dev_exit(idx);
	return 0;
}

static const struct drm_debugfs_info histb_vdp_debugfs_files[] = {
	{ "vdp_regs", histb_vdp_regs_show, 0 },
};

static irqreturn_t histb_vdp_irq(int irq, void *data)
{
	struct histb_vdp *vdp = data;
	u32 status;

	status = readl(vdp->base + HISTB_VDP_VOINTSTA) &
		 readl(vdp->base + HISTB_VDP_VOINTMSK);
	if (!status)
		return IRQ_NONE;

	/* Acknowledge every enabled source before running the vblank callbacks. */
	writel(status, vdp->base + HISTB_VDP_VOMSKINTSTA);
	if (status & HISTB_VDP_DHD0_UNDERFLOW) {
		atomic64_inc(&vdp->underflows);
		dev_warn_ratelimited(vdp->drm.dev, "DHD0 underflow\n");
	}
	if (status & HISTB_VDP_MASTER_ERRORS) {
		atomic64_inc(&vdp->master_errors);
		drm_err_ratelimited(&vdp->drm, "VDP master error IRQ: %08x\n", status);
	}
	if (status & HISTB_VDP_DHD0_VBLANK)
		drm_crtc_handle_vblank(&vdp->pipe.crtc);

	return IRQ_HANDLED;
}

static int histb_vdp_plane_properties(struct histb_vdp *vdp,
				      struct drm_plane *plane, unsigned int id)
{
	u32 zpos = vdp->soc->mv310 ? 1 : id < HISTB_VDP_GFX_COUNT ? id + 2 : id - 3;
	u32 max_zpos = vdp->soc->mv310 ? 1 : HISTB_VDP_PLANE_COUNT - 1;
	int ret;

	ret = drm_plane_create_alpha_property(plane);
	if (ret)
		return ret;
	ret = drm_plane_create_zpos_property(plane, zpos, 0, max_zpos);
	if (ret || vdp->soc->mv310)
		return ret;
	if (id < HISTB_VDP_GFX_COUNT)
		return drm_plane_create_blend_mode_property(plane,
			BIT(DRM_MODE_BLEND_PIXEL_NONE) | BIT(DRM_MODE_BLEND_PREMULTI) |
			BIT(DRM_MODE_BLEND_COVERAGE));
	return drm_plane_create_color_properties(plane,
		BIT(DRM_COLOR_YCBCR_BT601) | BIT(DRM_COLOR_YCBCR_BT709) |
		(id == 3 ? BIT(DRM_COLOR_YCBCR_BT2020) : 0),
		BIT(DRM_COLOR_YCBCR_LIMITED_RANGE) | BIT(DRM_COLOR_YCBCR_FULL_RANGE),
		DRM_COLOR_YCBCR_BT709, DRM_COLOR_YCBCR_LIMITED_RANGE);
}

static int histb_vdp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_bridge *bridge;
	struct platform_device *bridge_pdev;
	struct histb_vdp *vdp;
	struct drm_device *drm;
	unsigned int i;
	int ret;

	vdp = devm_drm_dev_alloc(dev, &histb_vdp_driver,
				 struct histb_vdp, drm);
	if (IS_ERR(vdp))
		return PTR_ERR(vdp);

	vdp->soc = of_device_get_match_data(dev);
	drm = &vdp->drm;
	spin_lock_init(&vdp->irq_lock);
	mutex_init(&vdp->power_lock);
	atomic64_set(&vdp->underflows, 0);
	atomic64_set(&vdp->master_errors, 0);

	vdp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vdp->base))
		return PTR_ERR(vdp->base);

	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, PTR_ERR(bridge),
				     "failed to find output bridge\n");

	/* Atomic replay needs the transmitter's AUX/reset domain restored first.
	 * OF graph discovery alone is insufficient with fw_devlink=off.
	 */
	bridge_pdev = of_find_device_by_node(bridge->of_node);
	if (!bridge_pdev)
		return -EPROBE_DEFER;
	if (!device_link_add(dev, &bridge_pdev->dev,
			     DL_FLAG_AUTOREMOVE_CONSUMER)) {
		put_device(&bridge_pdev->dev);
		return -ENOMEM;
	}
	put_device(&bridge_pdev->dev);

	vdp->clks[0].id = "bus";
	vdp->clks[1].id = "hd";
	vdp->clks[2].id = "config";
	vdp->clks[3].id = "core";
	vdp->clks[4].id = "video";
	vdp->clks[5].id = "graphics";
	vdp->clks[6].id = "bp";
	vdp->clks[7].id = "graphics1";
	vdp->clks[8].id = "graphics3";
	vdp->clks[9].id = "video1";
	vdp->num_clks = vdp->soc->mv310 ? 4 : ARRAY_SIZE(vdp->clks);
	ret = devm_clk_bulk_get(dev, vdp->num_clks, vdp->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get VDP clocks\n");


	vdp->reset = devm_reset_control_get_exclusive(dev, "vdp");
	if (IS_ERR(vdp->reset))
		return dev_err_probe(dev, PTR_ERR(vdp->reset),
				     "failed to get VDP reset\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	if (!vdp->soc->mv310) {
		for (i = 0; i < HISTB_VDP_VIDEO_COUNT; i++) {
			struct histb_vdp_zme *zme = &vdp->zme[i];
			size_t size = (i ? 1 : 2) * HISTB_VDP_CV200_ZME_SIZE;

			zme->cpu = dmam_alloc_coherent(dev, size, &zme->dma, GFP_KERNEL);
			if (!zme->cpu)
				return -ENOMEM;
			if (i)
				histb_vdp_cv200_build_zme(zme->cpu, 2, 2, 2, 2);
		}
		vdp->gp0_coefficients = dmam_alloc_coherent(dev,
			HISTB_VDP_CV200_GP0_COEF_SIZE, &vdp->gp0_coefficients_dma,
			GFP_KERNEL);
		if (!vdp->gp0_coefficients)
			return -ENOMEM;
		if (!IS_ALIGNED(vdp->gp0_coefficients_dma, 64) ||
		    vdp->gp0_coefficients_dma + HISTB_VDP_CV200_GP0_COEF_SIZE - 1 > U32_MAX)
			return dev_err_probe(dev, -EINVAL, "invalid GP0 coefficient DMA range\n");
		histb_vdp_cv200_build_gp0_bilinear(vdp->gp0_coefficients);
	}

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = HISTB_VDP_MIN_WIDTH;
	drm->mode_config.min_height = HISTB_VDP_MIN_HEIGHT;
	drm->mode_config.max_width = vdp->soc->max_width;
	drm->mode_config.max_height = vdp->soc->max_height;
	drm->mode_config.normalize_zpos = true;
	drm->mode_config.preferred_depth = 32;
	drm->mode_config.funcs = &histb_vdp_mode_config_funcs;
	drm->mode_config.helper_private = &histb_vdp_mode_config_helpers;

	ret = drm_simple_display_pipe_init(drm, &vdp->pipe,
					   &histb_vdp_pipe_funcs,
					   histb_vdp_formats,
					   ARRAY_SIZE(histb_vdp_formats),
					   NULL, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "failed to create display pipe\n");
	/* The simple-pipe enable hook has no atomic-state argument after swap. */
	vdp->crtc_helper_funcs = *vdp->pipe.crtc.helper_private;
	vdp->crtc_helper_funcs.atomic_enable = histb_vdp_crtc_atomic_enable;
	drm_crtc_helper_add(&vdp->pipe.crtc, &vdp->crtc_helper_funcs);
	drm_encoder_helper_add(&vdp->pipe.encoder,
			       &histb_vdp_encoder_helper_funcs);
	if (!vdp->soc->mv310) {
		/* The simple helper enforces unity before calling pipe .check. */
		vdp->primary_helper_funcs = *vdp->pipe.plane.helper_private;
		vdp->primary_helper_funcs.atomic_check = histb_vdp_primary_atomic_check;
		drm_plane_helper_add(&vdp->pipe.plane, &vdp->primary_helper_funcs);
	}

	ret = histb_vdp_plane_properties(vdp, &vdp->pipe.plane, 0);
	if (ret)
		return ret;
	if (!vdp->soc->mv310) {
		static const char * const names[] = { "g1", "g3", "v0", "v1" };

		for (i = 1; i < HISTB_VDP_PLANE_COUNT; i++) {
			struct histb_vdp_overlay *overlay = &vdp->overlays[i - 1];
			const u32 *formats = i < HISTB_VDP_GFX_COUNT ? histb_vdp_formats :
				i == 3 ? histb_vdp_hdr_formats : histb_vdp_v0_formats;
			unsigned int count = i < HISTB_VDP_GFX_COUNT ? ARRAY_SIZE(histb_vdp_formats) :
				i == 3 ? ARRAY_SIZE(histb_vdp_hdr_formats) : ARRAY_SIZE(histb_vdp_v0_formats);

			overlay->id = i;
			ret = drm_universal_plane_init(drm, &overlay->plane, 1,
					       &histb_vdp_overlay_funcs, formats, count,
					       i == HISTB_VDP_GFX_COUNT ? histb_vdp_v0_modifiers : NULL,
					       DRM_PLANE_TYPE_OVERLAY, "%s", names[i - 1]);
			if (ret)
				return ret;
			drm_plane_helper_add(&overlay->plane, &histb_vdp_overlay_helper_funcs);
			ret = histb_vdp_plane_properties(vdp, &overlay->plane, i);
			if (ret)
				return ret;
		}
	}

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize vblank\n");

	ret = drm_bridge_attach(&vdp->pipe.encoder, bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return dev_err_probe(dev, ret, "failed to attach bridge\n");

	vdp->connector = drm_bridge_connector_init(drm, &vdp->pipe.encoder);
	if (IS_ERR(vdp->connector))
		return dev_err_probe(dev, PTR_ERR(vdp->connector),
				     "failed to create connector\n");
#if IS_REACHABLE(CONFIG_DRM_HISI_HISTB_HDMI)
	if (!vdp->soc->mv310) {
		ret = histb_hdmi_connector_init(bridge, vdp->connector);
		if (ret)
			return ret;
	}
#endif

	ret = drm_connector_attach_encoder(vdp->connector, &vdp->pipe.encoder);
	if (ret)
		return ret;

	vdp->irq = platform_get_irq(pdev, 0);
	if (vdp->irq < 0)
		return vdp->irq;

	ret = devm_request_irq(dev, vdp->irq, histb_vdp_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), vdp);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request VDP IRQ\n");

	drm_mode_config_reset(drm);
	ret = drmm_kms_helper_poll_init(drm);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to initialize connector HPD handling\n");
	platform_set_drvdata(pdev, drm);
	drm_debugfs_add_files(drm, histb_vdp_debugfs_files,
			      ARRAY_SIZE(histb_vdp_debugfs_files));

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_fbdev_dma_setup(drm, 32);

	return 0;
}

static void histb_vdp_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	struct histb_vdp *vdp = to_histb_vdp(drm);

	drm_dev_unregister(drm);
	drm_atomic_helper_shutdown(drm);
	histb_vdp_resources_disable(vdp);
}

static void histb_vdp_shutdown(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	struct histb_vdp *vdp = to_histb_vdp(drm);

	drm_atomic_helper_shutdown(drm);
	histb_vdp_resources_disable(vdp);
}

static int histb_vdp_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct histb_vdp *vdp = to_histb_vdp(drm);
	int ret;

	ret = drm_mode_config_helper_suspend(drm);
	if (!ret)
		histb_vdp_resources_disable(vdp);

	return ret;
}

static int histb_vdp_resume(struct device *dev)
{
	return drm_mode_config_helper_resume(dev_get_drvdata(dev));
}

static DEFINE_SIMPLE_DEV_PM_OPS(histb_vdp_pm_ops,
				histb_vdp_suspend, histb_vdp_resume);

static const struct of_device_id histb_vdp_of_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-vdp", .data = &histb_cv200_data },
	{ .compatible = "hisilicon,hi3798mv310-vdp", .data = &histb_mv310_data },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_vdp_of_match);

static struct platform_driver histb_vdp_platform_driver = {
	.probe = histb_vdp_probe,
	.remove_new = histb_vdp_remove,
	.shutdown = histb_vdp_shutdown,
	.driver = {
		.name = "histb-vdp",
		.pm = pm_sleep_ptr(&histb_vdp_pm_ops),
		.of_match_table = histb_vdp_of_match,
	},
};
module_platform_driver(histb_vdp_platform_driver);

MODULE_DESCRIPTION("HiSilicon Hi3798CV200/MV310 VDP DRM driver");
MODULE_IMPORT_NS(DMA_BUF);
MODULE_LICENSE("GPL");
