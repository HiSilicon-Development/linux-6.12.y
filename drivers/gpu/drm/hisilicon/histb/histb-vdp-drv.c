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
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_fourcc.h>
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
#define HISTB_VDP_GP0_GALPHA		0x8020
#define HISTB_VDP_GP0_DFPOS		0x8100
#define HISTB_VDP_GP0_DLPOS		0x8104
#define HISTB_VDP_GP0_VFPOS		0x8108
#define HISTB_VDP_GP0_VLPOS		0x810c
#define HISTB_VDP_GP0_CSC_IDC		0x8120
#define HISTB_VDP_GP0_ZME_HSP		0x8140
#define HISTB_VDP_GP0_ZME_VSP		0x814c

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
	struct drm_plane v0_plane;
	struct drm_connector *connector;
	void *zme_coef;
	dma_addr_t zme_coef_dma;
	u32 zme_src_width, zme_src_height;
	u32 zme_dst_width, zme_dst_height;
	u32 display_width, display_height;
	u32 g0_zpos, v0_zpos;
	u8 zme_bank;
	bool zme_valid;
	bool v0_enabled;
	void __iomem *base;
	const struct histb_vdp_soc_data *soc;
	struct clk_bulk_data clks[7];
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


static inline struct histb_vdp *to_histb_vdp(struct drm_device *drm)
{
	return container_of(drm, struct histb_vdp, drm);
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

	return (u64)obj->dma_addr + fb->offsets[index] +
		(u64)((state->src.y1 >> 16) / vsub) * fb->pitches[index] +
		((state->src.x1 >> 16) / hsub) * fb->format->cpp[index];
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
			drm_format_info_min_pitch(fb->format, i, fb->width);
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
			drm_format_info_min_pitch(fb->format, i, fb->width);
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
	/* Keep the framebuffer writes ordered before both posted latch requests. */
	dma_wmb();
	writel(1, vdp->base + vdp->soc->g0_upd);
	readl(vdp->base + vdp->soc->g0_upd);
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

static int histb_vdp_program_plane(struct histb_vdp *vdp,
				   struct drm_plane_state *state,
				   const struct drm_plane_state *old_state)
{
	struct drm_framebuffer *fb = state->fb;
	dma_addr_t address;
	u32 format = histb_vdp_gfx_format(fb->format->format);
	u32 width = state->src_w >> 16;
	u32 height = state->src_h >> 16;
	u32 x = state->crtc_x;
	u32 y = state->crtc_y;
	u32 ctrl;
	u8 alpha = state->alpha >> 8;
	int ret;

	vdp->g0_zpos = state->normalized_zpos;
	ret = histb_vdp_copy_shadow(state);
	if (ret)
		return ret;

	address = drm_fb_dma_get_gem_addr(fb, state, 0);
	if (!histb_vdp_dma_address_valid(address)) {
		drm_err(&vdp->drm, "invalid G0 DMA address %pad\n", &address);
		return -EINVAL;
	}

	if (vdp->soc->mv310) {
		histb_mv310_program_plane(vdp, state, old_state, address);
		return 0;
	}

	writel(lower_32_bits(address), vdp->base + HISTB_VDP_G0_ADDR);
	/* GP0 applies global alpha after G0's per-pixel premultiplication. */
	writel(alpha, vdp->base + HISTB_VDP_GP0_GALPHA);
	/*
	 * A same-configuration page flip only changes the GFX shadow address.
	 * Avoid restaging live format, geometry and fetch controls before the
	 * paired GFX/GP update requests.
	 */
	if (histb_vdp_plane_config_unchanged(old_state, state)) {
		return 0;
	}

	/* G0 is the scanout layer and uses AXI master 0 in the CV200 BSP. */
	writel(readl(vdp->base + HISTB_VDP_VOMASTERSEL) &
	       ~HISTB_VDP_VOMASTERSEL_G0,
	       vdp->base + HISTB_VDP_VOMASTERSEL);
	/* Keep all unused fetch paths deterministic after the firmware handoff. */
	writel(0, vdp->base + HISTB_VDP_G0_NADDR);
	writel(HISTB_VDP_G0_SMMU_BYPASS_2D |
	       HISTB_VDP_G0_SMMU_BYPASS_3D_READ,
	       vdp->base + HISTB_VDP_G0_SMMU_BYPASS0);
	writel(0, vdp->base + HISTB_VDP_G0_DCMP_ADDR);
	writel(0, vdp->base + HISTB_VDP_G0_DCMP_NADDR);
	writel(0, vdp->base + HISTB_VDP_G0_DCMP_OFFSET);
	writel(fb->pitches[0] / HISTB_VDP_G0_STRIDE_UNIT,
	       vdp->base + HISTB_VDP_G0_STRIDE);
	writel(HISTB_VDP_SIZE(width, height),
	       vdp->base + HISTB_VDP_G0_IRESO);
	writel(0, vdp->base + HISTB_VDP_G0_SFPOS);
	writel(HISTB_VDP_POS(x, y), vdp->base + HISTB_VDP_G0_DFPOS);
	writel(HISTB_VDP_POS(x + width - 1, y + height - 1),
	       vdp->base + HISTB_VDP_G0_DLPOS);
	writel(HISTB_VDP_POS(x, y), vdp->base + HISTB_VDP_G0_VFPOS);
	writel(HISTB_VDP_POS(x + width - 1, y + height - 1),
	       vdp->base + HISTB_VDP_G0_VLPOS);

	ctrl = FIELD_PREP(HISTB_VDP_G0_IFMT, format) |
	       FIELD_PREP(HISTB_VDP_G0_BITEXT, 3) |
	       FIELD_PREP(HISTB_VDP_G0_REQ_CTRL, 3) |
	       HISTB_VDP_G0_READ_PROGRESSIVE |
	       HISTB_VDP_G0_UPDATE_PROGRESSIVE |
	       HISTB_VDP_G0_NONSECURE |
	       HISTB_VDP_G0_ENABLE;
	ctrl &= ~HISTB_VDP_G0_OFL_MASTER;
	writel(ctrl, vdp->base + HISTB_VDP_G0_CTRL);
	writel(0xff | BIT(8) |
	       (fb->format->has_alpha && state->pixel_blend_mode != DRM_MODE_BLEND_PIXEL_NONE ?
		HISTB_VDP_G0_PIXEL_ALPHA : 0) |
	       (state->pixel_blend_mode == DRM_MODE_BLEND_PREMULTI ? BIT(13) : 0),
	       vdp->base + HISTB_VDP_G0_CBMPARA);
	writel(0xff, vdp->base + HISTB_VDP_G0_ALPHA);

	return 0;
}

/* CV200 routes V0 SDR YCbCr conversion through VDM, not the legacy CSC. */
static void histb_vdp_program_v0_csc(struct histb_vdp *vdp,
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

	/* PQ Q10 coefficients become Q13; DC channels are Y, Cb, Cr. */
	for (i = 0; i < 9; i++)
		writel((coef[i] * 8) & 0xffff,
		       vdp->base + HISTB_VDP_VDM_YUV2RGB_COEF00 + i * 4);
	writel(13, vdp->base + HISTB_VDP_VDM_YUV2RGB_SCALE2P);
	writel(y_dc & 0x3fff, vdp->base + HISTB_VDP_VDM_YUV2RGB_IN_DC00);
	writel((-8192) & 0x3fff, vdp->base + HISTB_VDP_VDM_YUV2RGB_IN_DC00 + 4);
	writel((-8192) & 0x3fff, vdp->base + HISTB_VDP_VDM_YUV2RGB_IN_DC00 + 8);
	writel(0, vdp->base + HISTB_VDP_VDM_YUV2RGB_OUT_DC00);
	writel(0, vdp->base + HISTB_VDP_VDM_YUV2RGB_OUT_DC01);
	writel(0, vdp->base + HISTB_VDP_VDM_YUV2RGB_OUT_DC02);
	writel(0, vdp->base + HISTB_VDP_V0_CSC_IDC);
	writel(BIT(0) | BIT(1), vdp->base + HISTB_VDP_VDM_YUV2RGB_CTRL);
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

static void histb_vdp_v0_program_zme(struct histb_vdp *vdp, u32 sw, u32 sh,
				     u32 dw, u32 dh)
{
	dma_addr_t address;
	u32 hratio = div_u64((u64)(sw / 2) << 20, dw) * 2;
	u32 vratio = div_u64((u64)sh << 12, dh);

	if (vdp->zme_valid && vdp->zme_src_width == sw &&
	    vdp->zme_src_height == sh && vdp->zme_dst_width == dw &&
	    vdp->zme_dst_height == dh)
		return;

	/* The preceding commit retires before its inactive DMA bank is reused. */
	vdp->zme_bank ^= 1;
	address = vdp->zme_coef_dma + vdp->zme_bank * HISTB_VDP_CV200_ZME_SIZE;
	histb_vdp_cv200_build_zme(vdp->zme_coef +
				vdp->zme_bank * HISTB_VDP_CV200_ZME_SIZE,
				sw, sh, dw, dh);
	dma_wmb();
	writel(lower_32_bits(address), vdp->base + HISTB_VDP_V0_HLCOEFAD);
	writel(lower_32_bits(address + 256), vdp->base + HISTB_VDP_V0_HCCOEFAD);
	writel(lower_32_bits(address + 512), vdp->base + HISTB_VDP_V0_VLCOEFAD);
	writel(lower_32_bits(address + 768), vdp->base + HISTB_VDP_V0_VCCOEFAD);
	writel(0xf, vdp->base + HISTB_VDP_V0_PARAUP);
	/* Full-tap progressive 420 -> 422, with the BSP's centered chroma phase. */
	writel(hratio | BIT(24) | BIT(25) | BIT(26) | BIT(30) | BIT(31),
	       vdp->base + HISTB_VDP_V0_HSP);
	writel(FIELD_PREP(GENMASK(20, 19), 1) |
	       BIT(23) | BIT(24) | BIT(30) | BIT(31),
	       vdp->base + HISTB_VDP_V0_VSP);
	writel(vratio, vdp->base + HISTB_VDP_V0_VSR);
	writel(0, vdp->base + HISTB_VDP_V0_HLOFFSET);
	writel(0, vdp->base + HISTB_VDP_V0_HCOFFSET);
	writel((u16)-512, vdp->base + HISTB_VDP_V0_VOFFSET);
	writel((u16)-512, vdp->base + HISTB_VDP_V0_VBOFFSET);
	vdp->zme_src_width = sw;
	vdp->zme_src_height = sh;
	vdp->zme_dst_width = dw;
	vdp->zme_dst_height = dh;
	vdp->zme_valid = true;
}

static void histb_vdp_v0_disable(struct histb_vdp *vdp)
{
	vdp->v0_enabled = false;
	if (vdp->soc->mv310)
		return;
	writel(readl(vdp->base + HISTB_VDP_V0_CTRL) &
	       ~(HISTB_VDP_V0_SURFACE_EN | HISTB_VDP_V0_SURFACE_C_EN),
	       vdp->base + HISTB_VDP_V0_CTRL);
	writel(0, vdp->base + HISTB_VDP_V0_16REGIONENL);
}

static void histb_vdp_program_v0(struct histb_vdp *vdp,
				 struct drm_plane_state *state,
				 const struct drm_plane_state *old_state)
{
	struct drm_framebuffer *fb = state->fb;
	u32 sw = drm_rect_width(&state->src) >> 16;
	u32 sh = drm_rect_height(&state->src) >> 16;
	u32 dw = drm_rect_width(&state->dst);
	u32 dh = drm_rect_height(&state->dst);
	u32 x = state->dst.x1, y = state->dst.y1;
	u32 ctrl;
	bool was_enabled = vdp->v0_enabled;
	int ret;

	if (!state->visible) {
		histb_vdp_v0_disable(vdp);
		return;
	}
	ret = histb_vdp_copy_shadow(state);
	if (ret) {
		drm_err_ratelimited(&vdp->drm, "V0 shadow copy failed: %pe\n",
				    ERR_PTR(ret));
		histb_vdp_v0_disable(vdp);
		return;
	}
	vdp->v0_enabled = true;
	vdp->v0_zpos = state->normalized_zpos;
	writel(lower_32_bits(histb_vdp_plane_address(state, 0)),
	       vdp->base + HISTB_VDP_V0_P0LADDR);
	writel(lower_32_bits(histb_vdp_plane_address(state, 1)),
	       vdp->base + HISTB_VDP_V0_P0CADDR);
	if (was_enabled && histb_vdp_plane_config_unchanged(old_state, state) &&
	    old_state->fb->pitches[1] == fb->pitches[1] &&
	    old_state->color_encoding == state->color_encoding &&
	    old_state->color_range == state->color_range)
		return;

	histb_vdp_v0_chain_init(vdp);
	writel(BIT(31), vdp->base + HISTB_VDP_V0_PRERD);
	writel(1, vdp->base + HISTB_VDP_V0_16REGIONENL);
	writel(0, vdp->base + HISTB_VDP_V0_MULTI_MODE);
	writel(0xf, vdp->base + HISTB_VDP_V0_SMMU_BYPASS0);
	writel(0, vdp->base + HISTB_VDP_V0_SMMU_BYPASS1);
	writel(0, vdp->base + HISTB_VDP_V0_SMMU_BYPASS2);
	writel(0, vdp->base + HISTB_VDP_V0_SMMU_BYPASS3);
	/* Linear NV12/NV21 strides are byte counts, unlike tiled decoder metadata. */
	writel(fb->pitches[0] | fb->pitches[1] << 16,
	       vdp->base + HISTB_VDP_V0_P0STRIDE);
	writel(HISTB_VDP_SIZE(sw, sh), vdp->base + HISTB_VDP_V0_IRESO);
	writel(HISTB_VDP_SIZE(dw, dh), vdp->base + HISTB_VDP_V0_ORESO);
	writel((sw - 1) << 12, vdp->base + HISTB_VDP_V0_CPOS);
	histb_vdp_v0_program_zme(vdp, sw, sh, dw, dh);
	writel(HISTB_VDP_POS(x, y), vdp->base + HISTB_VDP_V0_DFPOS);
	writel(HISTB_VDP_POS(x + dw - 1, y + dh - 1),
	       vdp->base + HISTB_VDP_V0_DLPOS);
	writel(HISTB_VDP_POS(x, y), vdp->base + HISTB_VDP_V0_VFPOS);
	writel(HISTB_VDP_POS(x + dw - 1, y + dh - 1),
	       vdp->base + HISTB_VDP_V0_VLPOS);
	writel(0x04080200, vdp->base + HISTB_VDP_V0_BK);
	writel(0x04080200, vdp->base + HISTB_VDP_V0_MUTE_BK);
	writel(state->alpha >> 8, vdp->base + HISTB_VDP_V0_CBMPARA);
	writel(0xff, vdp->base + HISTB_VDP_V0_ALPHA);
	histb_vdp_program_v0_csc(vdp, state);
	ctrl = FIELD_PREP(HISTB_VDP_V0_IFMT, HISTB_VDP_V0_IFMT_SP_420) |
		FIELD_PREP(HISTB_VDP_V0_REQ_CTRL, 3) |
		HISTB_VDP_V0_NONSECURE |
		(fb->format->format == DRM_FORMAT_NV12 ? HISTB_VDP_V0_UV_ORDER : 0) |
		FIELD_PREP(HISTB_VDP_V0_CHM_RMODE, HISTB_VDP_V0_RMODE_PROGRESSIVE) |
		FIELD_PREP(HISTB_VDP_V0_LM_RMODE, HISTB_VDP_V0_RMODE_PROGRESSIVE) |
		FIELD_PREP(HISTB_VDP_V0_IFIR_MODE, 3) |
		HISTB_VDP_V0_SURFACE_EN;
	writel(ctrl, vdp->base + HISTB_VDP_V0_CTRL);

	/* VP0 spans the output canvas; V0 carries the positioned/scaled rectangle. */
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

static int histb_vdp_v0_plane_atomic_check(struct drm_plane *plane,
					   struct drm_atomic_state *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state;
	struct drm_framebuffer *fb = new_state->fb;
	unsigned int i;
	int ret;

	if (!new_state->crtc)
		return fb ? -EINVAL : 0;
	crtc_state = drm_atomic_get_crtc_state(state, new_state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);
	ret = drm_atomic_helper_check_plane_state(new_state, crtc_state,
					  1 << 13, 8 << 16, true, true);
	if (ret || !new_state->visible)
		return ret;
	/* 420 needs even integral source coordinates after clipping. */
	if ((new_state->src.x1 | new_state->src.y1 |
	     drm_rect_width(&new_state->src) | drm_rect_height(&new_state->src)) &
	    ((2 << 16) - 1))
		return -EINVAL;
	for (i = 0; i < 2; i++) {
		u32 hsub = i ? fb->format->hsub : 1;
		u32 vsub = i ? fb->format->vsub : 1;
		u32 width = (drm_rect_width(&new_state->src) >> 16) / hsub;
		u32 height = (drm_rect_height(&new_state->src) >> 16) / vsub;
		u64 address = histb_vdp_plane_address(new_state, i);
		u64 end = address + (u64)(height - 1) * fb->pitches[i] +
			width * fb->format->cpp[i] - 1;

		if (!IS_ALIGNED(fb->pitches[i], HISTB_VDP_V0_STRIDE_UNIT) ||
		    fb->pitches[i] > U16_MAX || !IS_ALIGNED(address, 16) ||
		    end > U32_MAX)
			return -EINVAL;
	}
	return 0;
}

static void histb_vdp_v0_plane_atomic_update(struct drm_plane *plane,
					     struct drm_atomic_state *state)
{
	struct histb_vdp *vdp = to_histb_vdp(plane->dev);
	struct drm_plane_state *new_state;

	new_state = drm_atomic_get_new_plane_state(state, plane);
	if (!new_state || !new_state->fb || !vdp->powered)
		return;

	histb_vdp_program_v0(vdp, new_state,
			     drm_atomic_get_old_plane_state(state, plane));
}

static void histb_vdp_v0_plane_atomic_disable(struct drm_plane *plane,
					      struct drm_atomic_state *state)
{
	struct histb_vdp *vdp = to_histb_vdp(plane->dev);

	if (vdp->powered)
		histb_vdp_v0_disable(vdp);
}

static const struct drm_plane_helper_funcs histb_vdp_v0_plane_helper_funcs = {
	.prepare_fb = drm_gem_plane_helper_prepare_fb,
	DRM_GEM_SHADOW_PLANE_HELPER_FUNCS,
	.atomic_check = histb_vdp_v0_plane_atomic_check,
	.atomic_update = histb_vdp_v0_plane_atomic_update,
	.atomic_disable = histb_vdp_v0_plane_atomic_disable,
};

static const struct drm_plane_funcs histb_vdp_v0_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	DRM_GEM_SHADOW_PLANE_FUNCS,
};

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

	/*
	 * The simple pipe always scans a 1:1 framebuffer. Stock Linux PQ clears
	 * both scaler enables in this case while retaining the HIFB ZME -> CSC
	 * topology. Do this explicitly instead of inheriting Fastboot state.
	 */
	writel(readl(vdp->base + HISTB_VDP_GP0_ZME_HSP) &
	       ~HISTB_VDP_GP0_ZME_H_ENABLE,
	       vdp->base + HISTB_VDP_GP0_ZME_HSP);
	writel(readl(vdp->base + HISTB_VDP_GP0_ZME_VSP) &
	       ~HISTB_VDP_GP0_ZME_V_ENABLE,
	       vdp->base + HISTB_VDP_GP0_ZME_VSP);
	writel(HISTB_VDP_GP0_READ_PROGRESSIVE |
	       HISTB_VDP_GP0_ROUTE_ZME_CSC,
	       vdp->base + HISTB_VDP_GP0_CTRL);
	writel(HISTB_VDP_SIZE(mode->hdisplay, mode->vdisplay),
	       vdp->base + HISTB_VDP_GP0_IRESO);
	writel(HISTB_VDP_SIZE(mode->hdisplay, mode->vdisplay),
	       vdp->base + HISTB_VDP_GP0_ORESO);
	writel(0xff, vdp->base + HISTB_VDP_GP0_GALPHA);
	writel(0, vdp->base + HISTB_VDP_GP0_DFPOS);
	writel(HISTB_VDP_POS(mode->hdisplay - 1, mode->vdisplay - 1),
	       vdp->base + HISTB_VDP_GP0_DLPOS);
	writel(0, vdp->base + HISTB_VDP_GP0_VFPOS);
	writel(HISTB_VDP_POS(mode->hdisplay - 1, mode->vdisplay - 1),
	       vdp->base + HISTB_VDP_GP0_VLPOS);
	writel(1, vdp->base + HISTB_VDP_GP0_UPD);
	readl(vdp->base + HISTB_VDP_GP0_UPD);
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
	vdp->zme_valid = false;
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

static void histb_vdp_pipe_enable(struct drm_simple_display_pipe *pipe,
				  struct drm_crtc_state *crtc_state,
				  struct drm_plane_state *plane_state)
{
	struct histb_vdp *vdp = to_histb_vdp(pipe->crtc.dev);
	unsigned long flags;
	int ret;

	vdp->display_width = crtc_state->adjusted_mode.hdisplay;
	vdp->display_height = crtc_state->adjusted_mode.vdisplay;
	vdp->v0_enabled = false;
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
	ret = histb_vdp_program_plane(vdp, plane_state, NULL);
	if (ret) {
		drm_err(&vdp->drm, "failed to program initial G0 plane: %pe\n",
			 ERR_PTR(ret));
		histb_vdp_resources_disable(vdp);
		return;
	}
	if (vdp->soc->mv310) {
		/* Latch G0/GP0 before the separate DHD interface-enable request. */
		histb_vdp_latch_gfx(vdp);
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

	if (!vdp->powered)
		return;
	/* Stop the core from queueing vblank work before the pipe goes away. */
	drm_crtc_vblank_off(crtc);

	histb_vdp_v0_disable(vdp);
	if (!vdp->soc->mv310) {
		writel(1, vdp->base + HISTB_VDP_V0_UPD);
		writel(1, vdp->base + HISTB_VDP_VP0_UPD);
		writel(2, vdp->base + HISTB_VDP_CBM_MIX1);
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
	const struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	struct drm_framebuffer *fb = plane_state->fb;
	u64 address, end;
	int ret;

	if (histb_vdp_pipe_mode_valid(pipe, mode) != MODE_OK)
		return -EINVAL;

	ret = drm_atomic_helper_check_plane_state(plane_state, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, true);
	if (ret)
		return ret;

	if (!fb)
		return 0;

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

static void histb_vdp_pipe_update(struct drm_simple_display_pipe *pipe,
				  struct drm_plane_state *old_state)
{
	struct histb_vdp *vdp = to_histb_vdp(pipe->crtc.dev);
	int ret;

	/* Initial G0 was already staged by pipe_enable before interface enable. */
	if (!vdp->powered || drm_atomic_crtc_needs_modeset(pipe->crtc.state))
		return;
	ret = histb_vdp_program_plane(vdp, pipe->plane.state, old_state);
	if (ret)
		drm_err(&vdp->drm, "failed to update G0 plane: %pe\n", ERR_PTR(ret));
}

static const struct drm_simple_display_pipe_funcs histb_vdp_pipe_funcs = {
	.mode_valid = histb_vdp_pipe_mode_valid,
	.enable = histb_vdp_pipe_enable,
	.disable = histb_vdp_pipe_disable,
	.check = histb_vdp_pipe_check,
	.update = histb_vdp_pipe_update,
	.enable_vblank = histb_vdp_enable_vblank,
	.disable_vblank = histb_vdp_disable_vblank,
	DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS,
};

static struct drm_framebuffer *
histb_vdp_fb_create(struct drm_device *drm, struct drm_file *file,
		    const struct drm_mode_fb_cmd2 *mode_cmd)
{
	if (!IS_ALIGNED(mode_cmd->pitches[0], HISTB_VDP_G0_STRIDE_UNIT) ||
	    mode_cmd->pitches[0] / HISTB_VDP_G0_STRIDE_UNIT >
		FIELD_MAX(HISTB_VDP_G0_STRIDE_MASK) ||
	    !IS_ALIGNED(mode_cmd->offsets[0], HISTB_VDP_G0_STRIDE_UNIT)) {
		dev_err(drm->dev,
			"fb_create rejected: %p4cc %ux%u pitch=%u offset=%u modifier=0x%llx flags=0x%x\n",
			&mode_cmd->pixel_format, mode_cmd->width, mode_cmd->height,
			mode_cmd->pitches[0], mode_cmd->offsets[0],
			(unsigned long long)mode_cmd->modifier[0], mode_cmd->flags);
		return ERR_PTR(-EINVAL);
	}

	return drm_gem_fb_create(drm, file, mode_cmd);
}

static const struct drm_mode_config_funcs histb_vdp_mode_config_funcs = {
	.fb_create = histb_vdp_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

/* Call only with live resources (atomic tail or the power-locked snapshot). */
static void histb_vdp_print_regs(struct drm_printer *p, struct histb_vdp *vdp)
{
	drm_printf(p, "VDP g0=%08x dhd0=%08x irq=%08x mask=%08x\n",
		   readl(vdp->base + vdp->soc->g0_ctrl),
		   readl(vdp->base + vdp->soc->dhd0_ctrl),
		   readl(vdp->base + HISTB_VDP_VOINTSTA),
		   readl(vdp->base + HISTB_VDP_VOINTMSK));
	if (vdp->soc->mv310)
		return;
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
	if (new_state && new_state->active && vdp->powered) {
		if (!vdp->soc->mv310) {
			u32 mixer = 2;

			if (vdp->v0_enabled)
				mixer = vdp->v0_zpos > vdp->g0_zpos ? 2 | (1 << 4) :
					1 | (2 << 4);
			writel(vdp->v0_enabled, vdp->base + HISTB_VDP_MIXV0_MIX);
			writel(mixer, vdp->base + HISTB_VDP_CBM_MIX1);
			writel(1, vdp->base + HISTB_VDP_V0_UPD);
			writel(1, vdp->base + HISTB_VDP_VP0_UPD);
			writel(HISTB_VDP_HDR_REGUP, vdp->base + HISTB_VDP_HDR_UPD);
		}
		histb_vdp_latch_gfx(vdp);
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

static int histb_vdp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_bridge *bridge;
	struct histb_vdp *vdp;
	struct drm_device *drm;
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

	vdp->clks[0].id = "bus";
	vdp->clks[1].id = "hd";
	vdp->clks[2].id = "config";
	vdp->clks[3].id = "core";
	vdp->clks[4].id = "video";
	vdp->clks[5].id = "graphics";
	vdp->num_clks = vdp->soc->mv310 ? 4 : 6;
	ret = devm_clk_bulk_get(dev, vdp->num_clks, vdp->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get VDP clocks\n");

	if (!vdp->soc->mv310) {
		/* Older CV200 DTs omit the gate between DHD and HDMI. */
		vdp->clks[6].id = "bp";
		vdp->clks[6].clk = devm_clk_get_optional(dev, "bp");
		if (IS_ERR(vdp->clks[6].clk))
			return dev_err_probe(dev, PTR_ERR(vdp->clks[6].clk),
					     "failed to get VDP output clock\n");
		if (vdp->clks[6].clk)
			vdp->num_clks++;
	}

	vdp->reset = devm_reset_control_get_exclusive(dev, "vdp");
	if (IS_ERR(vdp->reset))
		return dev_err_probe(dev, PTR_ERR(vdp->reset),
				     "failed to get VDP reset\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	if (!vdp->soc->mv310) {
		vdp->zme_coef = dmam_alloc_coherent(dev, 2 * HISTB_VDP_CV200_ZME_SIZE,
						 &vdp->zme_coef_dma, GFP_KERNEL);
		if (!vdp->zme_coef)
			return -ENOMEM;
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
	drm_encoder_helper_add(&vdp->pipe.encoder,
			       &histb_vdp_encoder_helper_funcs);

	ret = drm_plane_create_alpha_property(&vdp->pipe.plane);
	if (ret)
		return ret;
	ret = drm_plane_create_zpos_property(&vdp->pipe.plane, 1, 0, 1);
	if (ret)
		return ret;
	if (!vdp->soc->mv310) {
		ret = drm_universal_plane_init(drm, &vdp->v0_plane, 1,
					       &histb_vdp_v0_plane_funcs,
					       histb_vdp_v0_formats,
					       ARRAY_SIZE(histb_vdp_v0_formats),
					       NULL, DRM_PLANE_TYPE_OVERLAY, "v0");
		if (ret)
			return ret;
		drm_plane_helper_add(&vdp->v0_plane, &histb_vdp_v0_plane_helper_funcs);
		ret = drm_plane_create_alpha_property(&vdp->v0_plane);
		if (ret)
			return ret;
		ret = drm_plane_create_zpos_property(&vdp->v0_plane, 0, 0, 1);
		if (ret)
			return ret;
		ret = drm_plane_create_color_properties(&vdp->v0_plane,
				BIT(DRM_COLOR_YCBCR_BT601) | BIT(DRM_COLOR_YCBCR_BT709),
				BIT(DRM_COLOR_YCBCR_LIMITED_RANGE) | BIT(DRM_COLOR_YCBCR_FULL_RANGE),
				DRM_COLOR_YCBCR_BT709, DRM_COLOR_YCBCR_LIMITED_RANGE);
		if (ret)
			return ret;
		ret = drm_plane_create_blend_mode_property(&vdp->pipe.plane,
				BIT(DRM_MODE_BLEND_PIXEL_NONE) |
				BIT(DRM_MODE_BLEND_PREMULTI) | BIT(DRM_MODE_BLEND_COVERAGE));
		if (ret)
			return ret;
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
