// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon HiSTB HDMI transmitter bridge
 *
 * TX and VideoPath register sequences follow HiSTBLinuxV100R005C00SPC060
 * source/msp/drv/hdmi/hdmi_2_0/hal/ctrl/imagev200/driver/tx/
 * tx_hdmi/si_mod_tx_hdmi.c and videopath/si_mod_videopath.c.
 *
 * The transmitter contains a private DDC controller.  Keep its register
 * protocol local to this driver and expose sink discovery through the DRM
 * bridge helpers.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hdmi.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include "histb-hdmi-audio.h"
#include "histb-hdmi-hdcp.h"
#if IS_ENABLED(CONFIG_CEC_CORE)
#include <media/cec.h>
#endif

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_managed.h>
#include <drm/drm_modes.h>
#include <drm/drm_print.h>
#include <drm/drm_property.h>
#include <drm/drm_device.h>
#include <drm/drm_modeset_lock.h>
#include <drm/display/drm_hdcp_helper.h>
#include <drm/bridge/histb-hdmi.h>
#include <drm/display/drm_hdmi_state_helper.h>
#include <drm/display/drm_scdc_helper.h>

#define HISTB_HDMI_REG(page, reg)	((((page) << 8) | (reg)) << 2)

#define HISTB_HDMI_DDC_ADDR		HISTB_HDMI_REG(0, 0xed)
#define HISTB_HDMI_DDC_SEGMENT		HISTB_HDMI_REG(0, 0xee)
#define HISTB_HDMI_DDC_OFFSET		HISTB_HDMI_REG(0, 0xef)
#define HISTB_HDMI_DDC_COUNT_LOW	HISTB_HDMI_REG(0, 0xf0)
#define HISTB_HDMI_DDC_COUNT_HIGH	HISTB_HDMI_REG(0, 0xf1)
#define HISTB_HDMI_DDC_STATUS		HISTB_HDMI_REG(0, 0xf2)
#define HISTB_HDMI_DDC_CMD		HISTB_HDMI_REG(0, 0xf3)
#define HISTB_HDMI_DDC_DATA		HISTB_HDMI_REG(0, 0xf4)
#define HISTB_HDMI_DDC_FIFO_COUNT	HISTB_HDMI_REG(0, 0xf5)
#define HISTB_HDMI_DDC_DELAY_CNT	HISTB_HDMI_REG(0, 0xf6)

#define HISTB_HDMI_TPI_HPD_RSEN		HISTB_HDMI_REG(6, 0x3b)
#define HISTB_HDMI_TPI_DDC_MASTER_EN	HISTB_HDMI_REG(6, 0xf8)

#define HISTB_HDMI_FUNC_SEL		HISTB_HDMI_REG(0, 0x0b)
#define HISTB_HDMI_HOST_CTRL2		HISTB_HDMI_REG(0, 0x09)
#define HISTB_HDMI_CLKPWD		HISTB_HDMI_REG(0, 0x0d)
#define HISTB_HDMI_PWD_SRST		HISTB_HDMI_REG(0, 0x10)
#define HISTB_HDMI_SYS_MISC		HISTB_HDMI_REG(0, 0x13)
#define HISTB_HDMI_CLK_RATIO		HISTB_HDMI_REG(0, 0x65)
#define HISTB_HDMI_P2T_CTRL		HISTB_HDMI_REG(0, 0x66)
#define HISTB_HDMI_PKT_FILTER1		HISTB_HDMI_REG(0, 0x76)
#define HISTB_HDMI_AUDIO_PKT_ENABLE	BIT(3)
#define HISTB_HDMI_DIPT_CTRL		HISTB_HDMI_REG(0, 0x79)
#define HISTB_HDMI_SYS_STAT		HISTB_HDMI_REG(0, 0x31)
#define HISTB_HDMI_HTPLG_T2		HISTB_HDMI_REG(0, 0x8a)
#define HISTB_HDMI_HTPLG_T1		HISTB_HDMI_REG(0, 0x8b)
#define HISTB_HDMI_TEST_TXCTRL		HISTB_HDMI_REG(0, 0xf7)
#define HISTB_HDMI_INTR1		HISTB_HDMI_REG(0, 0x8f)
#define HISTB_HDMI_INTR1_MASK		HISTB_HDMI_REG(0, 0x95)
#define HISTB_HDMI_TX_REG_ZONE		HISTB_HDMI_REG(2, 0x22)
#define HISTB_HDMI_TX_ZONEL_CTRL4	HISTB_HDMI_REG(2, 0x24)
#define HISTB_HDMI_INFOFRAME_SELECT	HISTB_HDMI_REG(6, 0xbf)
#define HISTB_HDMI_INFOFRAME_DATA	HISTB_HDMI_REG(6, 0xc0)
#define HISTB_HDMI_INFOFRAME_CTRL	HISTB_HDMI_REG(6, 0xdf)
#define HISTB_HDMI_PHY_TOP_CTRL0	HISTB_HDMI_REG(7, 0xb0)
#define HISTB_HDMI_PHY_TOP_CTRL1	HISTB_HDMI_REG(7, 0xb1)
#define HISTB_HDMI_PHY_DP_CTRL0		HISTB_HDMI_REG(7, 0xb2)
#define HISTB_HDMI_PHY_DATA_SWING	HISTB_HDMI_REG(7, 0xb3)
#define HISTB_HDMI_PHY_CLK_SWING	HISTB_HDMI_REG(7, 0xb4)
#define HISTB_HDMI_PHY_SRC_TERM		HISTB_HDMI_REG(7, 0xb5)
#define HISTB_HDMI_PHY_VNB		HISTB_HDMI_REG(7, 0xb6)
#define HISTB_HDMI_PHY_CLK_FINE		HISTB_HDMI_REG(7, 0xb7)
#define HISTB_HDMI_PHY_PLL_CTRL0	HISTB_HDMI_REG(7, 0xb8)
#define HISTB_HDMI_PHY_PLL_BAND		HISTB_HDMI_REG(7, 0xb9)
#define HISTB_HDMI_PHY_PLL_CTRL2	HISTB_HDMI_REG(7, 0xba)
#define HISTB_HDMI_PHY_PLL_LOOP		HISTB_HDMI_REG(7, 0xba)
#define HISTB_HDMI_PHY_PLL_VCO		HISTB_HDMI_REG(7, 0xbc)
#define HISTB_HDMI_PHY_PLL_PI		HISTB_HDMI_REG(7, 0xbe)
#define HISTB_HDMI_PHY_BGR_BIAS		HISTB_HDMI_REG(7, 0xbf)
#define HISTB_HDMI_PHY_INPUT_BGR	HISTB_HDMI_REG(7, 0xc0)
#define HISTB_HDMI_PHY_RISE_TIME	HISTB_HDMI_REG(7, 0xc7)
#define HISTB_HDMI_PHY_DATA_FINE	HISTB_HDMI_REG(7, 0xc9)
#define HISTB_HDMI_PHY_FALL_TIME	HISTB_HDMI_REG(7, 0xca)
#define HISTB_HDMI_PHY_PLL_CHARGE	HISTB_HDMI_REG(7, 0xcb)
#define HISTB_HDMI_PHY_DATA_DRV		HISTB_HDMI_REG(7, 0xdc)
#define HISTB_HDMI_PHY_PLL_LDO		HISTB_HDMI_REG(7, 0xe0)
#define HISTB_HDMI_PHY_PLL_ZONE		HISTB_HDMI_REG(7, 0xe2)
#define HISTB_HDMI_PHY_TMDS_CTRL	HISTB_HDMI_REG(7, 0xf8)
#define HISTB_HDMI_SCR_CTRL		HISTB_HDMI_REG(9, 0x00)
#define HISTB_HDMI_TXC_DATA_DIV		HISTB_HDMI_REG(9, 0x09)
#define HISTB_HDMI_SCDC_CTL		HISTB_HDMI_REG(9, 0x20)
#define HISTB_HDMI_SCDC_INTR0		HISTB_HDMI_REG(9, 0x25)

#define HISTB_HDMI_AUD_EN		HISTB_HDMI_REG(0xa, 0x13)
#define  HISTB_HDMI_AUD_IN_EN		BIT(0)
#define HISTB_HDMI_AIP_RST		HISTB_HDMI_REG(0xa, 0x2c)
#define  HISTB_HDMI_AIP_RST_AUDIO		BIT(0)
#define  HISTB_HDMI_AIP_RST_FIFO		BIT(1)
#define HISTB_HDMI_ACR_CTRL		HISTB_HDMI_REG(0xa, 0x01)
#define  HISTB_HDMI_ACR_CTS_REQ_EN	BIT(1)
#define HISTB_HDMI_N_SVAL		HISTB_HDMI_REG(0xa, 0x03)
#define HISTB_HDMI_AUD_MODE		HISTB_HDMI_REG(0xa, 0x14)
#define HISTB_HDMI_I2S_IN_MAP		HISTB_HDMI_REG(0xa, 0x1c)
#define HISTB_HDMI_ASRC			HISTB_HDMI_REG(0xa, 0x23)
#define HISTB_HDMI_I2S_IN_CTRL		HISTB_HDMI_REG(0xa, 0x1d)
#define HISTB_HDMI_I2S_CHST3		HISTB_HDMI_REG(0xa, 0x21)
#define HISTB_HDMI_I2S_CHST4		HISTB_HDMI_REG(0xa, 0x22)
#define HISTB_HDMI_AIP_HDMI2MHL	HISTB_HDMI_REG(0xa, 0x2d)
#define HISTB_HDMI_AUDP_TXCTRL	HISTB_HDMI_REG(0xa, 0x2f)
#define HISTB_HDMI_I2S_IN_SIZE		HISTB_HDMI_REG(0xa, 0x24)
#define HISTB_HDMI_TPI_DOWN_SMPL_CTRL	HISTB_HDMI_REG(0xa, 0x61)
#define HISTB_HDMI_TPI_AUD_CONFIG	HISTB_HDMI_REG(0xa, 0x62)
#define HISTB_HDMI_TPI_AUD_FS		HISTB_HDMI_REG(0xa, 0x63)
#define HISTB_HDMI_CEC_CONFIG_CPI	HISTB_HDMI_REG(0xf, 0x8e)
#define HISTB_HDMI_CEC_DEBUG3		HISTB_HDMI_REG(0xf, 0x87)
#define HISTB_HDMI_CEC_TX_INIT		HISTB_HDMI_REG(0xf, 0x88)
#define HISTB_HDMI_CEC_TX_DEST		HISTB_HDMI_REG(0xf, 0x89)
#define HISTB_HDMI_CEC_TX_COMMAND		HISTB_HDMI_REG(0xf, 0x8f)
#define HISTB_HDMI_CEC_TX_OPERAND		HISTB_HDMI_REG(0xf, 0x90)
#define HISTB_HDMI_CEC_TRANSMIT		HISTB_HDMI_REG(0xf, 0x9f)
#define HISTB_HDMI_CEC_RETRY		HISTB_HDMI_REG(0xf, 0xa0)
#define HISTB_HDMI_CEC_CAPTURE		HISTB_HDMI_REG(0xf, 0xa2)
#define HISTB_HDMI_CEC_INT_ENABLE0	HISTB_HDMI_REG(0xf, 0xa4)
#define HISTB_HDMI_CEC_INT_ENABLE1	HISTB_HDMI_REG(0xf, 0xa5)
#define HISTB_HDMI_CEC_STATUS0		HISTB_HDMI_REG(0xf, 0xa6)
#define HISTB_HDMI_CEC_STATUS1		HISTB_HDMI_REG(0xf, 0xa7)
#define HISTB_HDMI_CEC_RX_CONTROL		HISTB_HDMI_REG(0xf, 0xac)
#define HISTB_HDMI_CEC_RX_COUNT		HISTB_HDMI_REG(0xf, 0xad)
#define HISTB_HDMI_CEC_RX_HEADER		HISTB_HDMI_REG(0xf, 0xae)
#define HISTB_HDMI_CEC_RX_COMMAND		HISTB_HDMI_REG(0xf, 0xaf)
#define HISTB_HDMI_CEC_RX_OPERAND		HISTB_HDMI_REG(0xf, 0xb0)
#define HISTB_HDMI_CEC_OP_ABORT		HISTB_HDMI_REG(0xf, 0xc0)
#define HISTB_HDMI_CEC_RESET		BIT(7)
#define HISTB_HDMI_CEC_NACK_HEADER		BIT(1)
#define HISTB_HDMI_CEC_FLUSH_TX		BIT(7)
#define HISTB_HDMI_CEC_TX_DONE		BIT(5)
#define HISTB_HDMI_CEC_TX_NACK		BIT(1)
#define HISTB_HDMI_PATTERN_GEN_CTRL0	HISTB_HDMI_REG(0xe, 0x60)

#define HISTB_HDMI_VP_INPUT_FORMAT	HISTB_HDMI_REG(0xb, 0x14)
#define HISTB_HDMI_VP_SOFT_RESET	HISTB_HDMI_REG(0xb, 0x0c)
#define HISTB_HDMI_VP_INPUT_MUTE		HISTB_HDMI_REG(0xb, 0x10)
#define HISTB_HDMI_VP_INPUT_SYNC		HISTB_HDMI_REG(0xb, 0x12)
#define HISTB_HDMI_VP_INPUT_MAPPING	HISTB_HDMI_REG(0xb, 0x16)
#define HISTB_HDMI_VP_INPUT_MASK		HISTB_HDMI_REG(0xb, 0x18)
#define HISTB_HDMI_VP_INPUT_SYNC_ADJ	HISTB_HDMI_REG(0xb, 0x1c)
#define HISTB_HDMI_VP_DEGEN_CONFIG	HISTB_HDMI_REG(0xb, 0x20)
#define HISTB_HDMI_VP_DEC656_CONFIG	HISTB_HDMI_REG(0xb, 0x30)
#define HISTB_HDMI_VP_OUTPUT_MUTE	HISTB_HDMI_REG(0xb, 0x40)
#define HISTB_HDMI_VP_OUTPUT_SYNC	HISTB_HDMI_REG(0xb, 0x42)
#define HISTB_HDMI_VP_OUTPUT_MAPPING	HISTB_HDMI_REG(0xb, 0x44)
#define HISTB_HDMI_VP_OUTPUT_MASK	HISTB_HDMI_REG(0xb, 0x46)
#define HISTB_HDMI_VP_OUTPUT_FORMAT	HISTB_HDMI_REG(0xb, 0x48)
#define HISTB_HDMI_VP_OUTPUT_BLANK	HISTB_HDMI_REG(0xb, 0x50)
#define HISTB_HDMI_VP_FDET_CONFIG	HISTB_HDMI_REG(0xb, 0x80)
#define HISTB_HDMI_VP_FDET_STATUS	HISTB_HDMI_REG(0xb, 0x81)
#define HISTB_HDMI_VP_FDET_PIXEL_COUNT	HISTB_HDMI_REG(0xb, 0x8c)
#define HISTB_HDMI_VP_FDET_LINE_COUNT	HISTB_HDMI_REG(0xb, 0x8e)
#define HISTB_HDMI_VP_FDET_IRQ_MASK	HISTB_HDMI_REG(0xb, 0xb0)
#define HISTB_HDMI_VP_FDET_IRQ_STATUS	HISTB_HDMI_REG(0xb, 0xb4)
#define HISTB_HDMI_VP_EMBD_SYNC		HISTB_HDMI_REG(0xb, 0xc0)
#define HISTB_HDMI_VP_CSC0_420_422	HISTB_HDMI_REG(0xd, 0x08)
#define HISTB_HDMI_VP_CSC0_422_444	HISTB_HDMI_REG(0xd, 0x0c)
#define HISTB_HDMI_VP_CSC0_CONFIG	HISTB_HDMI_REG(0xd, 0x20)
#define HISTB_HDMI_VP_CSC1_CONFIG	HISTB_HDMI_REG(0xd, 0xa0)
#define HISTB_HDMI_VP_CSC1_444_422	HISTB_HDMI_REG(0xd, 0xc0)
#define HISTB_HDMI_VP_CSC1_422_420	HISTB_HDMI_REG(0xd, 0xc2)
#define HISTB_HDMI_VP_CSC1_DITHER	HISTB_HDMI_REG(0xd, 0xc4)

#define HISTB_HDMI_DDC_FIFO_COUNT_MASK	GENMASK(4, 0)
#define HISTB_HDMI_DDC_NO_ACK		BIT(5)
#define HISTB_HDMI_DDC_BUS_LOW		BIT(6)
#define HISTB_HDMI_DDC_ERROR		(HISTB_HDMI_DDC_NO_ACK | \
					 HISTB_HDMI_DDC_BUS_LOW)
#define HISTB_HDMI_DDC_IN_PROGRESS	BIT(4)
#define HISTB_HDMI_PATTERN_TPG_ENABLE	BIT(0)
#define HISTB_HDMI_PATTERN_COLORBAR_ENABLE	BIT(4)
#define HISTB_HDMI_PATTERN_MASK	(HISTB_HDMI_PATTERN_TPG_ENABLE | \
					 HISTB_HDMI_PATTERN_COLORBAR_ENABLE)

#define HISTB_HDMI_TPI_DDC_MASTER	BIT(7)
#define HISTB_HDMI_HPD_CONNECTED	(BIT(0) | BIT(2))
#define HISTB_HDMI_INTR_POL_LOW		BIT(7)
#define HISTB_HDMI_INTR_HPD_RSEN	(BIT(5) | BIT(6))
#define HISTB_HDMI_SYS_STAT_P_STABLE	BIT(0)
#define HISTB_HDMI_SYS_STAT_RSEN	BIT(2)

#define HISTB_HDMI_SW_RESET		BIT(0)
#define HISTB_HDMI_DEEP_COLOR_MASK	GENMASK(1, 0)
#define HISTB_HDMI_DEEP_COLOR_PACKET	BIT(7)
#define HISTB_HDMI_MODE			BIT(1)
#define HISTB_HDMI_SCRAMBLE_MASK	(BIT(0) | BIT(5))
#define HISTB_HDMI_SCDC_ACCESS		BIT(0)
#define HISTB_HDMI_SCDC_DDC_CONFLICT	BIT(2)
#define HISTB_HDMI_INFOFRAME_SEL_MASK	GENMASK(3, 0)
#define HISTB_HDMI_INFOFRAME_ENABLE	BIT(7)
#define HISTB_HDMI_INFOFRAME_REPEAT	BIT(6)
#define HISTB_HDMI_INFOFRAME_ACTIVE	BIT(5)
#define HISTB_HDMI_PHY_DEPTH_MASK	GENMASK(1, 0)
#define HISTB_HDMI_PHY_TMDS_ENABLE	BIT(4)
#define HISTB_HDMI_PHY_OUTPUT_ENABLE	(BIT(7) | GENMASK(2, 0))
#define HISTB_HDMI_VP_SYNC_VPOS		BIT(0)
#define HISTB_HDMI_VP_SYNC_HPOS		BIT(1)
#define HISTB_HDMI_VP_AUTO_ADJ_DISABLE	BIT(0)
#define HISTB_HDMI_VP_420_ENABLE	BIT(10)
#define HISTB_HDMI_VP_CONVERTER_ENABLE	BIT(0)
#define HISTB_HDMI_VP_CONVERTER_BYPASS	BIT(1)
#define HISTB_HDMI_VP_FILTER_DISABLE	BIT(2)
#define HISTB_HDMI_VP_CSC_MODE_MASK	GENMASK(1, 0)
#define HISTB_HDMI_VP_CSC_ENABLE	BIT(0)
#define HISTB_HDMI_VP_CSC_OUT_STD	GENMASK(3, 2)
#define HISTB_HDMI_VP_CSC_OUT_PC	BIT(4)
#define HISTB_HDMI_VP_CSC_OUT_RGB	BIT(5)
#define HISTB_HDMI_VP_CSC_IN_STD	GENMASK(7, 6)
#define HISTB_HDMI_VP_CSC_IN_PC		BIT(8)
#define HISTB_HDMI_VP_CSC_IN_RGB		BIT(9)
#define HISTB_HDMI_VP_CSC_SAT_DISABLE	BIT(11)
#define HISTB_HDMI_VP_CSC_CONFIG_MASK	GENMASK(11, 0)
#define HISTB_HDMI_VP_DITHER_MODE	GENMASK(1, 0)
#define HISTB_HDMI_VP_DITHER_10_TO_8	2
#define HISTB_HDMI_VP_DITHER_NO_CHANGE	3

/* The VDP RGB identity path presents G, B, R to the VideoPath Y, Cb, Cr. */
#define HISTB_HDMI_VP_INPUT_MAP_RGB	0x0440
#define HISTB_HDMI_VP_OUTPUT_MAP_444	0x0440
#define HISTB_HDMI_VP_OUTPUT_MAP_422	0x0580
#define HISTB_HDMI_PERI_AUDIO_MASK	GENMASK(4, 0)
#define HISTB_HDMI_PERI_AUDIO_I2S	BIT(1)

#define HISTB_HDMI_INFOFRAME_BYTES	31
#define HISTB_HDMI14_MAX_TMDS_RATE	340000000ULL
#define HISTB_HDMI_MAX_TMDS_RATE	594000000ULL

#define HISTB_HDMI_DDC_EDID_ADDR	0xa0
#define HISTB_HDMI_DDC_SCDC_ADDR	0xa8
#define HISTB_HDMI_DDC_CMD_READ		0x02
#define HISTB_HDMI_DDC_CMD_READ_SEGMENT	0x04
#define HISTB_HDMI_DDC_CMD_WRITE	0x06
#define HISTB_HDMI_DDC_CMD_FIFO_CLEAR	0x09
#define HISTB_HDMI_DDC_CMD_ABORT	0x0f
#define HISTB_HDMI_DDC_CMD_SCDC		(BIT(4) | BIT(5))

#define HISTB_HDMI_SCDC_I2C_ADDR	0x54
#define HISTB_HDMI_SCDC_MAX_TRANSFER	16
#define HISTB_HDMI_SCDC_SOURCE_VERSION	1

#define HISTB_HDMI_DDC_TIMEOUT_US	100000
#define HISTB_HDMI_DDC_BYTE_TIMEOUT_US	20000
#define HISTB_HDMI_HPD_POLL_START_MS	150
#define HISTB_HDMI_HPD_POLL_PERIOD_MS	10

#define HISTB_HDMI_CTRL_RESET_COUNT	2
#define HISTB_HDMI_SW_RESET_COUNT	2

static const char * const histb_hdmi_clk_names[] = {
	"bus", "cec", "id", "mhl", "os", "as", "x", "pixel-nx",
	"pixel",
};

#define HISTB_HDMI_PIXEL_CLK_INDEX	(ARRAY_SIZE(histb_hdmi_clk_names) - 1)
#define HISTB_HDMI_PIXELNX_CLK_INDEX	(HISTB_HDMI_PIXEL_CLK_INDEX - 1)
#define HISTB_HDMI_AUX_CLK_COUNT	HISTB_HDMI_PIXEL_CLK_INDEX

static const char * const histb_hdmi_reset_names[] = {
	"bus", "core", "phy",
};

struct histb_hdmi_phy_config {
	u8 data_swing;
	u8 clk_swing;
	u8 src_termination;
	u8 vnb;
	u8 clk_fine;
	u8 pll_band;
	u8 pll_loop;
	u8 pll_vco;
	u8 pll_pi;
	u8 bgr_bias;
	u8 data_fine;
	u8 data_drive;
	u8 pll_ldo;
	u8 pll_zone;
	u8 pll_charge;
	u8 input_bgr;
	u8 tx_reg_zone;
	u8 rise_time;
	u8 fall_time;
};

/* CV200 STB hdmi_hal.c:412-420, selected by HalHdmiPhyParamGet():
 * 74.25, 165, 190, 297 and 340 MHz are distinct TMDS boundaries.
 */
static const struct histb_hdmi_phy_config histb_hdmi_phy_configs[] = {
	{
		.data_swing = 0x19, .clk_swing = 0x1f,
		.src_termination = 0xd5, .vnb = 0x02, .clk_fine = 0x01,
		.pll_band = 0x14, .pll_loop = 0x32, .pll_vco = 0x68,
		.pll_pi = 0x40, .bgr_bias = 0xc4, .data_fine = 0x01,
		.data_drive = 0x31, .pll_ldo = 0x06, .pll_zone = 0x04,
		.pll_charge = 0x68, .input_bgr = 0x0f, .tx_reg_zone = 0x03,
		.rise_time = 0x00, .fall_time = 0x10,
	}, {
		.data_swing = 0x19, .clk_swing = 0x1f,
		.src_termination = 0xd5, .vnb = 0x02, .clk_fine = 0x01,
		.pll_band = 0x14, .pll_loop = 0x32, .pll_vco = 0x68,
		.pll_pi = 0x40, .bgr_bias = 0xc4, .data_fine = 0x01,
		.data_drive = 0x31, .pll_ldo = 0x06, .pll_zone = 0x04,
		.pll_charge = 0x68, .input_bgr = 0x0f, .tx_reg_zone = 0x02,
		.rise_time = 0x00, .fall_time = 0x10,
	}, {
		.data_swing = 0x19, .clk_swing = 0x1f,
		.src_termination = 0xd5, .vnb = 0x02, .clk_fine = 0x01,
		.pll_band = 0x14, .pll_loop = 0x32, .pll_vco = 0x68,
		.pll_pi = 0x40, .bgr_bias = 0xc4, .data_fine = 0x01,
		.data_drive = 0x31, .pll_ldo = 0x06, .pll_zone = 0x04,
		.pll_charge = 0x68, .input_bgr = 0x0f, .tx_reg_zone = 0x01,
		.rise_time = 0x00, .fall_time = 0x10,
	}, {
		.data_swing = 0x20, .clk_swing = 0x1f,
		.src_termination = 0xd5, .vnb = 0x02, .clk_fine = 0x01,
		.pll_band = 0x14, .pll_loop = 0x32, .pll_vco = 0x68,
		.pll_pi = 0x40, .bgr_bias = 0xc4, .data_fine = 0x01,
		.data_drive = 0x31, .pll_ldo = 0x06, .pll_zone = 0x04,
		.pll_charge = 0x68, .input_bgr = 0x0f, .tx_reg_zone = 0x01,
		.rise_time = 0x00, .fall_time = 0x10,
	}, {
		.data_swing = 0x20, .clk_swing = 0x1f,
		.src_termination = 0xd5, .vnb = 0x02, .clk_fine = 0x01,
		.pll_band = 0x14, .pll_loop = 0x32, .pll_vco = 0x68,
		.pll_pi = 0x40, .bgr_bias = 0xc4, .data_fine = 0x01,
		.data_drive = 0x31, .pll_ldo = 0x06, .pll_zone = 0x04,
		.pll_charge = 0x68, .input_bgr = 0x0f, .tx_reg_zone = 0x01,
		.rise_time = 0x00, .fall_time = 0x90,
	}, {
		.data_swing = 0x36, .clk_swing = 0x12,
		.src_termination = 0x7f, .vnb = 0x02, .clk_fine = 0x03,
		.pll_band = 0x14, .pll_loop = 0x32, .pll_vco = 0x68,
		.pll_pi = 0x40, .bgr_bias = 0xc4, .data_fine = 0x01,
		.data_drive = 0x31, .pll_ldo = 0x06, .pll_zone = 0x04,
		.pll_charge = 0x68, .input_bgr = 0x0f, .tx_reg_zone = 0x00,
		.rise_time = 0x00, .fall_time = 0x00,
	},
};

struct histb_hdmi {
	struct device *dev;
	void __iomem *base;
	struct clk_bulk_data clks[ARRAY_SIZE(histb_hdmi_clk_names)];
	struct reset_control_bulk_data resets[ARRAY_SIZE(histb_hdmi_reset_names)];
	struct regmap *perictrl;
	u32 perictrl_offset;
	struct mutex ddc_lock; /* Serializes access to the internal DDC master. */
	struct mutex audio_lock; /* Serializes ACR/I2S updates with bridge state. */
	struct mutex packet_lock; /* Shared indexed AVI/audio/DRM packet port. */
	struct histb_hdmi_audio audio;
	struct i2c_adapter ddc;
	struct drm_bridge bridge;
	struct drm_connector *active_connector;
	struct drm_connector *hdcp_connector;
	struct drm_device *connector_dev;
	struct delayed_work hpd_work;
	struct delayed_work hdcp_work;
	struct histb_hdcp_io hdcp_io;
	struct histb_hdcp14 *hdcp14;
	struct histb_hdcp22 *hdcp22;
	struct reset_control *cipher_reset;
	unsigned int hdcp_protocol;
	bool cipher_active;
	bool hdcp_requested;
	/* audio_lock protects the attachment and every HDCP enqueue. */
	bool attached;
	bool removing;
	int tx_irq;
	int last_reported_status;
	bool hpd_enabled;
	bool hpd_polling;
	bool aux_enabled;
	bool pixel_enabled;
	bool mode_ready;
	bool high_tmds;
	bool scdc_enabled;
#if IS_ENABLED(CONFIG_CEC_CORE)
	struct cec_adapter *cec_adap;
	struct delayed_work cec_work;
	unsigned long cec_tx_deadline;
	u16 cec_log_addr_mask;
	bool cec_enabled;
	bool cec_tx_pending;
	bool cec_suspended;
	bool cec_tx_failed;
#endif
};


static inline struct histb_hdmi *bridge_to_histb_hdmi(struct drm_bridge *bridge)
{
	return container_of(bridge, struct histb_hdmi, bridge);
}

static u8 histb_hdmi_readb(struct histb_hdmi *hdmi, u32 reg)
{
	return readl(hdmi->base + reg);
}

static void histb_hdmi_writeb(struct histb_hdmi *hdmi, u32 reg, u8 value)
{
	writel(value, hdmi->base + reg);
}

static void histb_hdmi_updateb(struct histb_hdmi *hdmi, u32 reg,
			       u8 mask, u8 value)
{
	u8 regval = histb_hdmi_readb(hdmi, reg);

	histb_hdmi_writeb(hdmi, reg, (regval & ~mask) | (value & mask));
}

static void histb_hdmi_writew(struct histb_hdmi *hdmi, u32 reg, u16 value)
{
	histb_hdmi_writeb(hdmi, reg, value);
	histb_hdmi_writeb(hdmi, reg + sizeof(u32), value >> 8);
}

static u16 histb_hdmi_readw(struct histb_hdmi *hdmi, u32 reg)
{
	return histb_hdmi_readb(hdmi, reg) |
	       histb_hdmi_readb(hdmi, reg + sizeof(u32)) << 8;
}
#if IS_ENABLED(CONFIG_CEC_CORE)

/* All CEC MMIO is serialized with HDMI resets by audio_lock. Completion
 * and receive callbacks run outside it: the CEC core takes adap->lock.
 */
static void histb_hdmi_cec_hw_config(struct histb_hdmi *hdmi)
{
	bool enable = hdmi->cec_enabled && !hdmi->cec_suspended;
	unsigned int i;

	if (hdmi->cec_tx_pending) {
		hdmi->cec_tx_pending = false;
		hdmi->cec_tx_failed = true;
	}

	if (!hdmi->aux_enabled)
		return;
	histb_hdmi_updateb(hdmi, HISTB_HDMI_REG(0, 0x12),
			   HISTB_HDMI_CEC_RESET, enable ? 0 : HISTB_HDMI_CEC_RESET);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_CEC_CONFIG_CPI, BIT(4),
			   enable ? 0 : BIT(4));
	histb_hdmi_updateb(hdmi, HISTB_HDMI_CEC_DEBUG3,
			   HISTB_HDMI_CEC_NACK_HEADER | HISTB_HDMI_CEC_FLUSH_TX,
			   HISTB_HDMI_CEC_FLUSH_TX |
			   (enable ? 0 : HISTB_HDMI_CEC_NACK_HEADER));
	/* Use the BSP's process-context status polling, not an unserviced IRQ. */
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_INT_ENABLE0, 0);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_INT_ENABLE1, 0);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_STATUS0, 0x66);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_STATUS1, 0x1f);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_RX_CONTROL, BIT(1));
	if (!enable)
		return;
	/* Let the CEC framework implement replies, including Feature Abort. */
	for (i = 0; i < 32; i++)
		histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_OP_ABORT + i * 4, 0);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_RETRY, 0);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_CAPTURE,
			  hdmi->cec_log_addr_mask);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_CAPTURE + 4,
			  hdmi->cec_log_addr_mask >> 8);
	mod_delayed_work(system_wq, &hdmi->cec_work, msecs_to_jiffies(10));
}

static void histb_hdmi_cec_work(struct work_struct *work)
{
	struct histb_hdmi *hdmi = container_of(to_delayed_work(work),
					      struct histb_hdmi, cec_work);
	struct cec_msg received[3] = {};
	unsigned int count = 0, i, operands;
	u8 status0, status1, tx_status = 0, rx_count;

	mutex_lock(&hdmi->audio_lock);
	if (!hdmi->cec_enabled || hdmi->cec_suspended || !hdmi->aux_enabled)
		goto unlock;
	status0 = histb_hdmi_readb(hdmi, HISTB_HDMI_CEC_STATUS0);
	status1 = histb_hdmi_readb(hdmi, HISTB_HDMI_CEC_STATUS1);
	if (status1 & HISTB_HDMI_CEC_TX_NACK)
		histb_hdmi_updateb(hdmi, HISTB_HDMI_CEC_DEBUG3,
				   HISTB_HDMI_CEC_FLUSH_TX, HISTB_HDMI_CEC_FLUSH_TX);
	if (hdmi->cec_tx_failed) {
		tx_status = CEC_TX_STATUS_ERROR;
		hdmi->cec_tx_failed = false;
	}
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_STATUS0, status0 & 0x66);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_STATUS1, status1 & 0x1f);
	if (hdmi->cec_tx_pending) {
		if (status1 & HISTB_HDMI_CEC_TX_NACK)
			tx_status = CEC_TX_STATUS_NACK;
		else if (status0 & HISTB_HDMI_CEC_TX_DONE)
			tx_status = CEC_TX_STATUS_OK;
		else if (time_after_eq(jiffies, hdmi->cec_tx_deadline)) {
			histb_hdmi_updateb(hdmi, HISTB_HDMI_CEC_DEBUG3,
				HISTB_HDMI_CEC_FLUSH_TX, HISTB_HDMI_CEC_FLUSH_TX);
			tx_status = CEC_TX_STATUS_ERROR;
		}
		if (tx_status)
			hdmi->cec_tx_pending = false;
	}
	while (count < ARRAY_SIZE(received)) {
		rx_count = histb_hdmi_readb(hdmi, HISTB_HDMI_CEC_RX_COUNT);
		if (!(rx_count & GENMASK(5, 4)))
			break;
		operands = rx_count & 0xf;
		if (!(rx_count & BIT(7)) && operands <= CEC_MAX_MSG_SIZE - 2) {
			struct cec_msg *msg = &received[count++];

			msg->len = operands + 2;
			msg->msg[0] = histb_hdmi_readb(hdmi, HISTB_HDMI_CEC_RX_HEADER);
			msg->msg[1] = histb_hdmi_readb(hdmi, HISTB_HDMI_CEC_RX_COMMAND);
			for (i = 0; i < operands; i++)
				msg->msg[i + 2] = histb_hdmi_readb(hdmi,
					HISTB_HDMI_CEC_RX_OPERAND + i * 4);
		} else {
			dev_warn_ratelimited(hdmi->dev, "invalid CEC RX frame: %02x\n",
					     rx_count);
			/* Bound invalid-frame draining to this hardware FIFO. */
			histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_RX_CONTROL, BIT(1));
			break;
		}
		histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_RX_CONTROL, BIT(0));
	}
	mod_delayed_work(system_wq, &hdmi->cec_work, msecs_to_jiffies(10));
unlock:
	mutex_unlock(&hdmi->audio_lock);
	if (tx_status)
		cec_transmit_attempt_done(hdmi->cec_adap, tx_status);
	for (i = 0; i < count; i++)
		cec_received_msg(hdmi->cec_adap, &received[i]);
}

static int histb_hdmi_cec_adap_enable(struct cec_adapter *adap, bool enable)
{
	struct histb_hdmi *hdmi = cec_get_drvdata(adap);
	guard(mutex)(&hdmi->audio_lock);

	if (enable && (!hdmi->aux_enabled || hdmi->cec_suspended || hdmi->removing))
		return -EHOSTDOWN;
	hdmi->cec_enabled = enable;
	hdmi->cec_tx_pending = false;
	hdmi->cec_tx_failed = false;
	hdmi->cec_log_addr_mask = 0;
	histb_hdmi_cec_hw_config(hdmi);
	if (!enable)
		/* adap->lock is held: never wait on a worker notifying the core. */
		cancel_delayed_work(&hdmi->cec_work);
	return 0;
}

static int histb_hdmi_cec_adap_log_addr(struct cec_adapter *adap, u8 logical_addr)
{
	struct histb_hdmi *hdmi = cec_get_drvdata(adap);
	guard(mutex)(&hdmi->audio_lock);

	hdmi->cec_log_addr_mask = logical_addr == CEC_LOG_ADDR_INVALID ? 0 :
		BIT(logical_addr);
	if (!hdmi->aux_enabled || hdmi->cec_suspended)
		return 0;
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_CAPTURE, hdmi->cec_log_addr_mask);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_CAPTURE + 4,
			  hdmi->cec_log_addr_mask >> 8);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_TX_INIT,
		logical_addr == CEC_LOG_ADDR_INVALID ? CEC_LOG_ADDR_UNREGISTERED : logical_addr);
	return 0;
}

static int histb_hdmi_cec_adap_transmit(struct cec_adapter *adap, u8 attempts,
					u32 signal_free_time, struct cec_msg *msg)
{
	struct histb_hdmi *hdmi = cec_get_drvdata(adap);
	unsigned int i, free_us = CEC_FREE_TIME_TO_USEC(signal_free_time);

	if (!msg->len || msg->len > CEC_MAX_MSG_SIZE)
		return -EINVAL;
	usleep_range(free_us, free_us + 1000);
	guard(mutex)(&hdmi->audio_lock);
	if (!hdmi->cec_enabled || hdmi->cec_suspended || !hdmi->aux_enabled)
		return -EHOSTDOWN;
	if (hdmi->cec_tx_pending)
		return -EBUSY;
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_STATUS0, 0x64);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_STATUS1, HISTB_HDMI_CEC_TX_NACK);
	/* One hardware attempt; the CEC framework accounts for retries. */
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_RETRY, 0);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_TX_INIT, cec_msg_initiator(msg));
	hdmi->cec_tx_pending = true;
	hdmi->cec_tx_deadline = jiffies + msecs_to_jiffies(1000);
	if (msg->len == 1) {
		histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_TX_DEST,
				  BIT(7) | cec_msg_destination(msg));
	} else {
		histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_TX_DEST, cec_msg_destination(msg));
		histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_TX_COMMAND, msg->msg[1]);
		for (i = 0; i < msg->len - 2; i++)
			histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_TX_OPERAND + i * 4,
					  msg->msg[i + 2]);
		histb_hdmi_writeb(hdmi, HISTB_HDMI_CEC_TRANSMIT,
				  BIT(4) | (msg->len - 2));
	}
	mod_delayed_work(system_wq, &hdmi->cec_work, msecs_to_jiffies(10));
	return 0;
}

static void histb_hdmi_cec_pause(struct histb_hdmi *hdmi)
{
	bool pending;

	mutex_lock(&hdmi->audio_lock);
	hdmi->cec_suspended = true;
	pending = hdmi->cec_tx_pending || hdmi->cec_tx_failed;
	hdmi->cec_tx_pending = false;
	hdmi->cec_tx_failed = false;
	histb_hdmi_cec_hw_config(hdmi);
	mutex_unlock(&hdmi->audio_lock);
	cancel_delayed_work_sync(&hdmi->cec_work);
	if (pending && !IS_ERR_OR_NULL(hdmi->cec_adap))
		cec_transmit_attempt_done(hdmi->cec_adap, CEC_TX_STATUS_ERROR);
}

static void histb_hdmi_cec_release(void *data)
{
	struct histb_hdmi *hdmi = data;

	histb_hdmi_cec_pause(hdmi);
	cec_unregister_adapter(hdmi->cec_adap);
	hdmi->cec_adap = NULL;
}

static const struct cec_adap_ops histb_hdmi_cec_ops = {
	.adap_enable = histb_hdmi_cec_adap_enable,
	.adap_log_addr = histb_hdmi_cec_adap_log_addr,
	.adap_transmit = histb_hdmi_cec_adap_transmit,
};
#endif

static void histb_hdmi_write24(struct histb_hdmi *hdmi, u32 reg, u32 value)
{
	histb_hdmi_writeb(hdmi, reg, value);
	histb_hdmi_writeb(hdmi, reg + sizeof(u32), value >> 8);
	histb_hdmi_writeb(hdmi, reg + 2 * sizeof(u32), value >> 16);
}

static void histb_hdmi_updatew(struct histb_hdmi *hdmi, u32 reg,
			       u16 mask, u16 value)
{
	u16 regval = histb_hdmi_readw(hdmi, reg);

	histb_hdmi_writew(hdmi, reg, (regval & ~mask) | (value & mask));
}

static void histb_hdmi_reset_video_path(struct histb_hdmi *hdmi)
{
	/* Restore the 4:4:4 path after a prior mode. */
	histb_hdmi_updatew(hdmi, HISTB_HDMI_VP_INPUT_FORMAT,
			   HISTB_HDMI_VP_420_ENABLE, 0);
	histb_hdmi_updatew(hdmi, HISTB_HDMI_VP_OUTPUT_FORMAT,
			   HISTB_HDMI_VP_420_ENABLE, 0);

	histb_hdmi_updateb(hdmi, HISTB_HDMI_VP_CSC0_420_422,
			   HISTB_HDMI_VP_CONVERTER_ENABLE |
			   HISTB_HDMI_VP_CONVERTER_BYPASS,
			   HISTB_HDMI_VP_CONVERTER_BYPASS);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_VP_CSC0_422_444,
			   HISTB_HDMI_VP_CONVERTER_ENABLE, 0);
	histb_hdmi_updatew(hdmi, HISTB_HDMI_VP_CSC0_CONFIG,
			   HISTB_HDMI_VP_CSC_MODE_MASK, 0);

	histb_hdmi_updatew(hdmi, HISTB_HDMI_VP_CSC1_CONFIG,
			   HISTB_HDMI_VP_CSC_CONFIG_MASK, 0);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_VP_CSC1_444_422,
			   HISTB_HDMI_VP_CONVERTER_ENABLE |
			   HISTB_HDMI_VP_FILTER_DISABLE, 0);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_VP_CSC1_422_420,
			   HISTB_HDMI_VP_CONVERTER_ENABLE |
			   HISTB_HDMI_VP_CONVERTER_BYPASS,
			   HISTB_HDMI_VP_CONVERTER_BYPASS);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_VP_CSC1_DITHER,
			   HISTB_HDMI_VP_DITHER_MODE,
			   HISTB_HDMI_VP_DITHER_NO_CHANGE);

	/* Software-selected 1:1 IDCK:PCLK and PCLKNX:PCLK ratios. */
	histb_hdmi_writeb(hdmi, HISTB_HDMI_CLK_RATIO, 0x85);
}

/* Recreate the vendor VideoPath baseline before a new pixel stream starts. */
static void histb_hdmi_video_path_hw_init(struct histb_hdmi *hdmi)
{
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_SOFT_RESET, 0x07);
	usleep_range(1000, 2000);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_SOFT_RESET, 0x00);
	usleep_range(1000, 2000);

	/* InSPA is the CV200 VDP input; all unused bus transforms stay bypassed. */
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_INPUT_MUTE, 0x00);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_INPUT_SYNC, 0x00);
	histb_hdmi_writew(hdmi, HISTB_HDMI_VP_INPUT_MASK, 0x0000);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_INPUT_SYNC_ADJ,
			  HISTB_HDMI_VP_AUTO_ADJ_DISABLE);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_DEGEN_CONFIG, 0x00);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_DEC656_CONFIG, 0x00);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_OUTPUT_MUTE, 0x00);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_OUTPUT_SYNC, 0x00);
	histb_hdmi_writew(hdmi, HISTB_HDMI_VP_OUTPUT_MASK, 0x0000);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_OUTPUT_BLANK, 0x00);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_FDET_CONFIG, 0x00);
	histb_hdmi_write24(hdmi, HISTB_HDMI_VP_FDET_IRQ_MASK, 0x000000);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_EMBD_SYNC, 0x00);

	histb_hdmi_reset_video_path(hdmi);
}

static u8 histb_hdmi_depth_pack_mode(unsigned int bpc)
{
	switch (bpc) {
	case 10:
		return 1;
	case 12:
		return 2;
	case 8:
	default:
		return 0;
	}
}

static void histb_hdmi_configure_output_depth(struct histb_hdmi *hdmi,
					      unsigned int bpc)
{
	u8 pack_mode = histb_hdmi_depth_pack_mode(bpc);
	u8 p2t = pack_mode;

	if (bpc > 8)
		p2t |= HISTB_HDMI_DEEP_COLOR_PACKET;

	histb_hdmi_updateb(hdmi, HISTB_HDMI_P2T_CTRL,
			   HISTB_HDMI_DEEP_COLOR_MASK |
			   HISTB_HDMI_DEEP_COLOR_PACKET, p2t);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_PHY_TOP_CTRL1,
			   HISTB_HDMI_PHY_DEPTH_MASK, pack_mode);
}

static enum hdmi_colorimetry
histb_hdmi_colorimetry(const struct drm_display_mode *mode)
{
	return mode->vdisplay <= 576 ? HDMI_COLORIMETRY_ITU_601 :
				      HDMI_COLORIMETRY_ITU_709;
}

static void histb_hdmi_configure_video_path(struct histb_hdmi *hdmi,
					    const struct drm_display_mode *mode,
					    enum hdmi_colorspace format,
					    unsigned int bpc, bool bt2020)
{
	u16 output_mapping = HISTB_HDMI_VP_OUTPUT_MAP_444;
	u16 csc_config = HISTB_HDMI_VP_CSC_ENABLE |
			 HISTB_HDMI_VP_CSC_IN_PC | HISTB_HDMI_VP_CSC_IN_RGB |
			 HISTB_HDMI_VP_CSC_SAT_DISABLE;
	u8 standard = bt2020 ? 2 :
		histb_hdmi_colorimetry(mode) == HDMI_COLORIMETRY_ITU_601 ? 1 : 0;
	u8 dither = bpc == 8 ? HISTB_HDMI_VP_DITHER_10_TO_8 :
				     HISTB_HDMI_VP_DITHER_NO_CHANGE;

	histb_hdmi_reset_video_path(hdmi);

	/* imagev200 sVideoPathCscConfig(): 709=0, 601=1, 2020 NCL=2.
	 * The VDP supplies nonlinear full-range RGB; both standard selectors
	 * must match the AVI colourimetry, without changing its PQ/HLG transfer.
	 */
	csc_config |= FIELD_PREP(HISTB_HDMI_VP_CSC_IN_STD, standard) |
		      FIELD_PREP(HISTB_HDMI_VP_CSC_OUT_STD, standard);
	if (format != HDMI_COLORSPACE_RGB)
		histb_hdmi_updatew(hdmi, HISTB_HDMI_VP_CSC1_CONFIG,
				   HISTB_HDMI_VP_CSC_CONFIG_MASK, csc_config);

	switch (format) {
	case HDMI_COLORSPACE_RGB:
		/* The VDP RGB identity path feeds the transmitter without CSC. */
		break;
	case HDMI_COLORSPACE_YUV444:
		break;
	case HDMI_COLORSPACE_YUV422:
		histb_hdmi_updateb(hdmi, HISTB_HDMI_VP_CSC1_444_422,
				   HISTB_HDMI_VP_CONVERTER_ENABLE |
				   HISTB_HDMI_VP_FILTER_DISABLE,
				   HISTB_HDMI_VP_CONVERTER_ENABLE |
				   HISTB_HDMI_VP_FILTER_DISABLE);
		output_mapping = HISTB_HDMI_VP_OUTPUT_MAP_422;
		break;
	case HDMI_COLORSPACE_YUV420:
		histb_hdmi_updateb(hdmi, HISTB_HDMI_VP_CSC1_444_422,
				   HISTB_HDMI_VP_CONVERTER_ENABLE |
				   HISTB_HDMI_VP_FILTER_DISABLE,
				   HISTB_HDMI_VP_CONVERTER_ENABLE |
				   HISTB_HDMI_VP_FILTER_DISABLE);
		histb_hdmi_updateb(hdmi, HISTB_HDMI_VP_CSC1_422_420,
				   HISTB_HDMI_VP_CONVERTER_ENABLE |
				   HISTB_HDMI_VP_CONVERTER_BYPASS,
				   HISTB_HDMI_VP_CONVERTER_ENABLE);
		histb_hdmi_updatew(hdmi, HISTB_HDMI_VP_OUTPUT_FORMAT,
				   HISTB_HDMI_VP_420_ENABLE,
				   HISTB_HDMI_VP_420_ENABLE);
		/* 4:2:0 uses a half-rate PCLKNX while IDCK remains 1:1. */
		histb_hdmi_writeb(hdmi, HISTB_HDMI_CLK_RATIO, 0x81);
		break;
	default:
		dev_err(hdmi->dev, "unsupported HDMI output format %u\n",
			format);
		return;
	}

	histb_hdmi_updateb(hdmi, HISTB_HDMI_VP_CSC1_DITHER,
			   HISTB_HDMI_VP_DITHER_MODE, dither);
	histb_hdmi_writew(hdmi, HISTB_HDMI_VP_OUTPUT_MAPPING,
			  output_mapping);
}

static void histb_hdmi_tmds_enable(struct histb_hdmi *hdmi, bool enable)
{
	if (!enable) {
		histb_hdmi_updateb(hdmi, HISTB_HDMI_PHY_TMDS_CTRL,
				   HISTB_HDMI_PHY_TMDS_ENABLE, 0);
		histb_hdmi_updateb(hdmi, HISTB_HDMI_PHY_DP_CTRL0,
				   HISTB_HDMI_PHY_OUTPUT_ENABLE, 0);
		return;
	}

	histb_hdmi_updateb(hdmi, HISTB_HDMI_PHY_TMDS_CTRL,
			   HISTB_HDMI_PHY_TMDS_ENABLE,
			   HISTB_HDMI_PHY_TMDS_ENABLE);
	usleep_range(1000, 2000);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_PHY_DP_CTRL0,
			   HISTB_HDMI_PHY_OUTPUT_ENABLE,
			   HISTB_HDMI_PHY_OUTPUT_ENABLE);
}

static void histb_hdmi_validate_link(struct histb_hdmi *hdmi)
{
	u8 sys_stat = 0;
	u8 tmds_ctrl = 0;
	u8 dp_ctrl = 0;
	u8 fdet_status = 0;
	u16 pixel_count = 0;
	u16 line_count = 0;
	unsigned int i;

	/* Allow the first complete VDP frame to reach the transmitter. */
	for (i = 0; i < 25; i++) {
		sys_stat = histb_hdmi_readb(hdmi, HISTB_HDMI_SYS_STAT);
		tmds_ctrl = histb_hdmi_readb(hdmi, HISTB_HDMI_PHY_TMDS_CTRL);
		dp_ctrl = histb_hdmi_readb(hdmi, HISTB_HDMI_PHY_DP_CTRL0);
		fdet_status = histb_hdmi_readb(hdmi, HISTB_HDMI_VP_FDET_STATUS);
		pixel_count = histb_hdmi_readw(hdmi, HISTB_HDMI_VP_FDET_PIXEL_COUNT);
		line_count = histb_hdmi_readw(hdmi, HISTB_HDMI_VP_FDET_LINE_COUNT);

		if ((sys_stat & HISTB_HDMI_SYS_STAT_P_STABLE) && pixel_count &&
		    line_count)
			break;
		usleep_range(1000, 2000);
	}

	if (!(sys_stat & HISTB_HDMI_SYS_STAT_P_STABLE))
		dev_warn_ratelimited(hdmi->dev,
			 "HDMI TX clock is not stable after TMDS enable (SYS_STAT=0x%02x)\n",
			 sys_stat);
	if (!(tmds_ctrl & HISTB_HDMI_PHY_TMDS_ENABLE) ||
	    (dp_ctrl & HISTB_HDMI_PHY_OUTPUT_ENABLE) !=
	    HISTB_HDMI_PHY_OUTPUT_ENABLE)
		dev_warn_ratelimited(hdmi->dev,
			 "HDMI TMDS output controls are not enabled (tmds=0x%02x dp=0x%02x)\n",
			 tmds_ctrl, dp_ctrl);
	if (!pixel_count || !line_count)
		dev_warn_ratelimited(hdmi->dev,
			 "HDMI VideoPath FDET has no active DE (status=0x%02x pixels=%u lines=%u)\n",
			 fdet_status, pixel_count, line_count);
	if (!(sys_stat & HISTB_HDMI_SYS_STAT_RSEN))
		dev_warn_ratelimited(hdmi->dev,
			 "HDMI receiver sense is low (SYS_STAT=0x%02x)\n",
			 sys_stat);
}

static void histb_hdmi_infoframe_enable(struct histb_hdmi *hdmi,
					bool enable)
{
	/* imagev200 requires clearing repeat before the send request. */
	if (!enable) {
		histb_hdmi_updateb(hdmi, HISTB_HDMI_INFOFRAME_CTRL,
				   HISTB_HDMI_INFOFRAME_REPEAT, 0);
		histb_hdmi_updateb(hdmi, HISTB_HDMI_INFOFRAME_CTRL,
				   HISTB_HDMI_INFOFRAME_ENABLE, 0);
		return;
	}
	histb_hdmi_updateb(hdmi, HISTB_HDMI_INFOFRAME_CTRL,
			   HISTB_HDMI_INFOFRAME_ENABLE | HISTB_HDMI_INFOFRAME_REPEAT,
			   HISTB_HDMI_INFOFRAME_ENABLE | HISTB_HDMI_INFOFRAME_REPEAT);
}

static int histb_hdmi_infoframe_wait(struct histb_hdmi *hdmi, bool active)
{
	u8 ctrl;
	int ret;

	ret = read_poll_timeout(histb_hdmi_readb, ctrl,
				!!(ctrl & HISTB_HDMI_INFOFRAME_ACTIVE) == active,
				1000, 50000, false, hdmi,
				HISTB_HDMI_INFOFRAME_CTRL);
	if (ret)
		dev_warn(hdmi->dev,
			 "infoframe failed to become %s (ctrl=0x%02x)\n",
			 active ? "active" : "inactive", ctrl);
	return ret;
}

static int histb_hdmi_infoframe_select(struct histb_hdmi *hdmi,
				       enum hdmi_infoframe_type type)
{
	u8 selector;

	switch (type) {
	case HDMI_INFOFRAME_TYPE_AVI:
		selector = 0;
		break;
	case HDMI_INFOFRAME_TYPE_AUDIO:
		selector = 2;
		break;
	case HDMI_INFOFRAME_TYPE_SPD:
		selector = 3;
		break;
	case HDMI_INFOFRAME_TYPE_VENDOR:
		selector = 5;
		break;
	case HDMI_INFOFRAME_TYPE_DRM:
		/* CV200 sInfoframesSet() uses GEN3, not the generic GEN4 comment. */
		selector = 8;
		break;
	default:
		return -EINVAL;
	}

	histb_hdmi_updateb(hdmi, HISTB_HDMI_INFOFRAME_SELECT,
			   HISTB_HDMI_INFOFRAME_SEL_MASK, selector);

	return 0;
}

static void histb_hdmi_disable_infoframes(struct histb_hdmi *hdmi)
{
	static const u8 selectors[] = { 0, 2, 3, 5, 8 };
	unsigned int i;
	guard(mutex)(&hdmi->packet_lock);

	for (i = 0; i < ARRAY_SIZE(selectors); i++) {
		histb_hdmi_updateb(hdmi, HISTB_HDMI_INFOFRAME_SELECT,
				   HISTB_HDMI_INFOFRAME_SEL_MASK, selectors[i]);
		histb_hdmi_infoframe_enable(hdmi, false);
	}

	/* The CV200 transmitter expects the selector to remain on AVI. */
	histb_hdmi_updateb(hdmi, HISTB_HDMI_INFOFRAME_SELECT,
			   HISTB_HDMI_INFOFRAME_SEL_MASK, 0);
	(void)histb_hdmi_readb(hdmi, HISTB_HDMI_INFOFRAME_SELECT);
}

/* Match the SPC060 PWD/SW reset and restore the AVI packet context. */
static void histb_hdmi_pwd_srst(struct histb_hdmi *hdmi)
{
	u8 value;

	histb_hdmi_updateb(hdmi, HISTB_HDMI_PWD_SRST,
			   HISTB_HDMI_SW_RESET, HISTB_HDMI_SW_RESET);
	value = histb_hdmi_readb(hdmi, HISTB_HDMI_PWD_SRST);
	if (!(value & HISTB_HDMI_SW_RESET))
		dev_warn(hdmi->dev,
			 "HDMI PWD_SRST assert did not read back (value=0x%02x)\n",
			 value);

	udelay(10);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_PWD_SRST,
			   HISTB_HDMI_SW_RESET, 0);
	value = histb_hdmi_readb(hdmi, HISTB_HDMI_PWD_SRST);
	if (value & HISTB_HDMI_SW_RESET)
		dev_warn(hdmi->dev,
			 "HDMI PWD_SRST clear did not read back (value=0x%02x)\n",
			 value);

	histb_hdmi_updateb(hdmi, HISTB_HDMI_INFOFRAME_SELECT,
			   HISTB_HDMI_INFOFRAME_SEL_MASK, 0);
	value = histb_hdmi_readb(hdmi, HISTB_HDMI_INFOFRAME_SELECT);
	if (value & HISTB_HDMI_INFOFRAME_SEL_MASK)
		dev_warn(hdmi->dev,
			 "HDMI AVI infoframe selector did not read back (value=0x%02x)\n",
			 value);

	/* SPC060 resends AVI after PWD_SRST; DRM refreshes its payload later. */
	histb_hdmi_infoframe_enable(hdmi, true);
	(void)histb_hdmi_readb(hdmi, HISTB_HDMI_INFOFRAME_CTRL);
}

static void histb_hdmi_phy_configure(struct histb_hdmi *hdmi, u64 tmds_rate)
{
	const struct histb_hdmi_phy_config *config;

	if (tmds_rate < 74250000)
		config = &histb_hdmi_phy_configs[0];
	else if (tmds_rate < 165000000)
		config = &histb_hdmi_phy_configs[1];
	else if (tmds_rate < 190000000)
		config = &histb_hdmi_phy_configs[2];
	else if (tmds_rate < 297000000)
		config = &histb_hdmi_phy_configs[3];
	else if (tmds_rate < 340000000)
		config = &histb_hdmi_phy_configs[4];
	else
		config = &histb_hdmi_phy_configs[5];

	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_DATA_SWING,
			  config->data_swing);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_CLK_SWING, config->clk_swing);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_SRC_TERM,
			  config->src_termination);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_VNB, config->vnb);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_CLK_FINE, config->clk_fine);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_PLL_BAND, config->pll_band);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_PLL_LOOP, config->pll_loop);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_PLL_VCO, config->pll_vco);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_PLL_PI, config->pll_pi);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_BGR_BIAS, config->bgr_bias);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_DATA_FINE, config->data_fine);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_DATA_DRV, config->data_drive);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_PLL_LDO, config->pll_ldo);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_PLL_ZONE, config->pll_zone);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_PLL_CHARGE,
			  config->pll_charge);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_INPUT_BGR, config->input_bgr);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_TX_REG_ZONE, config->tx_reg_zone);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_RISE_TIME, config->rise_time);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_FALL_TIME, config->fall_time);
}

static void histb_hdmi_audio_stop(void *data)
{
	struct histb_hdmi *hdmi = data;

	if (!hdmi->aux_enabled)
		return;
	histb_hdmi_updateb(hdmi, HISTB_HDMI_AUDP_TXCTRL, BIT(7), BIT(7));
	/* CTS/N requests have their own gate, independent of audio sample packets. */
	histb_hdmi_updateb(hdmi, HISTB_HDMI_ACR_CTRL,
			   HISTB_HDMI_ACR_CTS_REQ_EN, 0);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_AUD_EN, HISTB_HDMI_AUD_IN_EN, 0);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_PKT_FILTER1,
			   HISTB_HDMI_AUDIO_PKT_ENABLE, 0);
	guard(mutex)(&hdmi->packet_lock);
	histb_hdmi_infoframe_select(hdmi, HDMI_INFOFRAME_TYPE_AUDIO);
	histb_hdmi_infoframe_enable(hdmi, false);
	histb_hdmi_infoframe_select(hdmi, HDMI_INFOFRAME_TYPE_AVI);
}

/* Values from the CV200 vendor sHardwareInit() sequence. */
static void histb_hdmi_tx_hw_init(struct histb_hdmi *hdmi)
{
	u8 ddc_count;

	histb_hdmi_writeb(hdmi, HISTB_HDMI_TEST_TXCTRL, 0x02);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_HTPLG_T2, 0x4b);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_HTPLG_T1, 0x64);

	/* DDC speed 0x15: bit 7 selects the extended counter range. */
	ddc_count = histb_hdmi_readb(hdmi, HISTB_HDMI_DDC_FIFO_COUNT);
	ddc_count &= ~BIT(7);
	ddc_count |= (0x15 >> 1) & BIT(7);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_FIFO_COUNT, ddc_count);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_DELAY_CNT, 0x15);

	histb_hdmi_audio_stop(hdmi);

	/* Normal scanout must come from VDP, never the internal test generator. */
	histb_hdmi_updateb(hdmi, HISTB_HDMI_PATTERN_GEN_CTRL0,
			   HISTB_HDMI_PATTERN_MASK, 0);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_CEC_CONFIG_CPI, BIT(4), 0x00);
}

static int histb_hdmi_bus_core_reset(struct histb_hdmi *hdmi,
					     unsigned int assert_us,
					     unsigned int settle_us)
{
	int ret;

	/* The vendor reset sequence never asserts the HDMI PHY reset. */
	ret = reset_control_bulk_assert(HISTB_HDMI_CTRL_RESET_COUNT,
						hdmi->resets);
	if (ret)
		return ret;
	usleep_range(assert_us, assert_us + 1000);

	ret = reset_control_bulk_deassert(HISTB_HDMI_CTRL_RESET_COUNT,
					  hdmi->resets);
	if (ret)
		return ret;
	usleep_range(settle_us, settle_us + 1000);

	return 0;
}

static int histb_hdmi_vendor_reset_sequence(struct histb_hdmi *hdmi)
{
	int ret;
	unsigned int i;

	/* Match the initial vendor hardware-reset settle before register writes. */
	ret = histb_hdmi_bus_core_reset(hdmi, 1000, 1000);
	if (ret)
		return ret;

	/* HalHdmiSwReset is a TX PWD_SRST pulse, not another CRG reset. */
	for (i = 0; i < HISTB_HDMI_SW_RESET_COUNT; i++) {
		histb_hdmi_pwd_srst(hdmi);
		if (i + 1 < HISTB_HDMI_SW_RESET_COUNT)
			usleep_range(1000, 2000);
	}

	return 0;
}

static int histb_hdmi_hw_init(struct histb_hdmi *hdmi)
{
	int ret;

	/* Keep the public peripheral mux on the standard I2S source. */
	ret = regmap_update_bits(hdmi->perictrl, hdmi->perictrl_offset,
				 HISTB_HDMI_PERI_AUDIO_MASK,
				 HISTB_HDMI_PERI_AUDIO_I2S);
	if (ret)
		return ret;

	histb_hdmi_writeb(hdmi, HISTB_HDMI_CLKPWD, 0x06);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_FUNC_SEL, 0x01);
	/* The DT describes the TX interrupt as level-high. */
	histb_hdmi_updateb(hdmi, HISTB_HDMI_HOST_CTRL2,
			   HISTB_HDMI_INTR_POL_LOW, 0);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_INTR1_MASK,
		READ_ONCE(hdmi->hpd_enabled) && !hdmi->hpd_polling ?
		HISTB_HDMI_INTR_HPD_RSEN : 0);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_INTR1,
			  HISTB_HDMI_INTR_HPD_RSEN);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_SYS_MISC, 0x00);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DIPT_CTRL, 0x06);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_TX_ZONEL_CTRL4, 0x04);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_TOP_CTRL0, 0x90);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_PLL_CTRL0, 0x82);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_PLL_CTRL2, 0x30);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_PLL_ZONE, 0x04);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_PHY_PLL_LDO, 0x06);
	histb_hdmi_tx_hw_init(hdmi);
	histb_hdmi_video_path_hw_init(hdmi);
	histb_hdmi_tmds_enable(hdmi, false);
	histb_hdmi_disable_infoframes(hdmi);
#if IS_ENABLED(CONFIG_CEC_CORE)
	histb_hdmi_cec_hw_config(hdmi);
#endif

	return 0;
}

static void histb_hdmi_configure_mode(struct histb_hdmi *hdmi,
				      const struct drm_display_mode *mode,
				      u64 tmds_rate, bool is_hdmi,
				      enum hdmi_colorspace format,
				      unsigned int bpc, bool bt2020)
{
	u8 sync = 0;

	histb_hdmi_tmds_enable(hdmi, false);
	histb_hdmi_phy_configure(hdmi, tmds_rate);

	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		sync |= HISTB_HDMI_VP_SYNC_VPOS;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		sync |= HISTB_HDMI_VP_SYNC_HPOS;
	histb_hdmi_pwd_srst(hdmi);
#if IS_ENABLED(CONFIG_CEC_CORE)
	/* PWD_SRST resets CPI too: restore it after the last mode reset. */
	histb_hdmi_cec_hw_config(hdmi);
#endif
	histb_hdmi_configure_output_depth(hdmi, bpc);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_OUTPUT_SYNC, sync);

	/* The full-range RGB input pin order does not depend on the sink format. */
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_INPUT_SYNC_ADJ,
			  HISTB_HDMI_VP_AUTO_ADJ_DISABLE);
	histb_hdmi_writew(hdmi, HISTB_HDMI_VP_INPUT_MAPPING,
			  HISTB_HDMI_VP_INPUT_MAP_RGB);
	histb_hdmi_configure_video_path(hdmi, mode, format, bpc, bt2020);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_OUTPUT_MUTE, 0);

	/* Source scrambling is enabled only after the sink accepts SCDC. */
	hdmi->high_tmds = tmds_rate > HISTB_HDMI14_MAX_TMDS_RATE;
	histb_hdmi_writeb(hdmi, HISTB_HDMI_TXC_DATA_DIV,
			  hdmi->high_tmds ? 0x02 : 0x00);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_SCR_CTRL,
			   HISTB_HDMI_SCRAMBLE_MASK, 0);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_TEST_TXCTRL,
			   HISTB_HDMI_MODE,
			   is_hdmi ? HISTB_HDMI_MODE : 0);
	histb_hdmi_disable_infoframes(hdmi);
	drm_dbg_kms(hdmi->bridge.dev,
		 "CV200 HDMI mode %ux%u@%u %s %u bpc tmds=%llu sync=%02x in=%04x in_map=%04x out=%04x out_map=%04x ratio=%02x csc=%04x dither=%02x pack=%02x phy_depth=%02x\n",
		 mode->hdisplay, mode->vdisplay, drm_mode_vrefresh(mode),
		 drm_hdmi_connector_get_output_format_name(format), bpc,
		 tmds_rate, histb_hdmi_readb(hdmi, HISTB_HDMI_VP_OUTPUT_SYNC),
		 histb_hdmi_readw(hdmi, HISTB_HDMI_VP_INPUT_FORMAT),
		 histb_hdmi_readw(hdmi, HISTB_HDMI_VP_INPUT_MAPPING),
		 histb_hdmi_readw(hdmi, HISTB_HDMI_VP_OUTPUT_FORMAT),
		 histb_hdmi_readw(hdmi, HISTB_HDMI_VP_OUTPUT_MAPPING),
		 histb_hdmi_readb(hdmi, HISTB_HDMI_CLK_RATIO),
		 histb_hdmi_readw(hdmi, HISTB_HDMI_VP_CSC1_CONFIG),
		 histb_hdmi_readb(hdmi, HISTB_HDMI_VP_CSC1_DITHER),
		 histb_hdmi_readb(hdmi, HISTB_HDMI_P2T_CTRL),
		 histb_hdmi_readb(hdmi, HISTB_HDMI_PHY_TOP_CTRL1));
}

static int histb_hdmi_ddc_enable(struct histb_hdmi *hdmi)
{
	u8 status;
	int ret;

	ret = read_poll_timeout(histb_hdmi_readb, status,
				!(status & HISTB_HDMI_DDC_IN_PROGRESS), 100,
				HISTB_HDMI_DDC_TIMEOUT_US, false, hdmi,
				HISTB_HDMI_DDC_STATUS);
	if (ret)
		return ret;

	histb_hdmi_updateb(hdmi, HISTB_HDMI_TPI_DDC_MASTER_EN,
			   HISTB_HDMI_TPI_DDC_MASTER,
			   HISTB_HDMI_TPI_DDC_MASTER);

	return 0;
}

static void histb_hdmi_ddc_disable(struct histb_hdmi *hdmi)
{
	histb_hdmi_updateb(hdmi, HISTB_HDMI_TPI_DDC_MASTER_EN,
			   HISTB_HDMI_TPI_DDC_MASTER, 0);
}

static void histb_hdmi_ddc_abort(struct histb_hdmi *hdmi, u8 command_flags)
{
	u8 status;

	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_CMD,
			  HISTB_HDMI_DDC_CMD_ABORT | command_flags);
	status = histb_hdmi_readb(hdmi, HISTB_HDMI_DDC_STATUS);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_STATUS,
			  status & ~HISTB_HDMI_DDC_ERROR);
	/* The vendor sequence clears these bits only after an SCDC cycle. */
	if (command_flags)
		histb_hdmi_updateb(hdmi, HISTB_HDMI_DDC_CMD,
				   GENMASK(6, 0), 0);
}

static int histb_hdmi_read_edid_block(void *data, u8 *buf,
				      unsigned int block, size_t len)
{
	struct histb_hdmi *hdmi = data;
	u8 command, count, offset, segment, status;
	int ret;
	size_t i;

	if (len != EDID_LENGTH || block > U8_MAX)
		return -EINVAL;

	segment = block >> 1;
	offset = (block & 1) ? EDID_LENGTH : 0;
	command = block < 2 ? HISTB_HDMI_DDC_CMD_READ :
				    HISTB_HDMI_DDC_CMD_READ_SEGMENT;

	ret = histb_hdmi_ddc_enable(hdmi);
	if (ret)
		return ret;

	histb_hdmi_ddc_abort(hdmi, 0);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_ADDR,
			  HISTB_HDMI_DDC_EDID_ADDR);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_SEGMENT, segment);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_OFFSET, offset);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_COUNT_HIGH, len >> 8);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_COUNT_LOW, len);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_CMD,
			  HISTB_HDMI_DDC_CMD_FIFO_CLEAR);
	usleep_range(1000, 2000);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_CMD, command);

	for (i = 0; i < len; i++) {
		ret = read_poll_timeout(histb_hdmi_readb, count,
					(count & HISTB_HDMI_DDC_FIFO_COUNT_MASK) ||
					(histb_hdmi_readb(hdmi,
						HISTB_HDMI_DDC_STATUS) &
					 HISTB_HDMI_DDC_ERROR),
					5, HISTB_HDMI_DDC_BYTE_TIMEOUT_US, false,
					hdmi, HISTB_HDMI_DDC_FIFO_COUNT);
		if (ret)
			goto abort;

		status = histb_hdmi_readb(hdmi, HISTB_HDMI_DDC_STATUS);
		if (status & HISTB_HDMI_DDC_NO_ACK) {
			ret = -ENXIO;
			goto abort;
		}
		if (status & HISTB_HDMI_DDC_BUS_LOW) {
			ret = -EIO;
			goto abort;
		}

		buf[i] = histb_hdmi_readb(hdmi, HISTB_HDMI_DDC_DATA);
		udelay(5);
	}

	ret = read_poll_timeout(histb_hdmi_readb, status,
				!(status & HISTB_HDMI_DDC_IN_PROGRESS), 10,
				HISTB_HDMI_DDC_BYTE_TIMEOUT_US, false, hdmi,
				HISTB_HDMI_DDC_STATUS);
	if (ret)
		goto abort;

	goto release;

abort:
	histb_hdmi_ddc_abort(hdmi, 0);
release:
	histb_hdmi_ddc_disable(hdmi);

	return ret;
}

static int histb_hdmi_ddc_status_error(u8 status)
{
	if (status & HISTB_HDMI_DDC_NO_ACK)
		return -ENXIO;
	if (status & HISTB_HDMI_DDC_BUS_LOW)
		return -EIO;

	return 0;
}

static int histb_hdmi_scdc_select(struct histb_hdmi *hdmi)
{
	unsigned int retries;

	for (retries = 0; retries < 10; retries++) {
		/* Clear a prior conflict, then request an SCDC DDC cycle. */
		histb_hdmi_writeb(hdmi, HISTB_HDMI_SCDC_INTR0,
				  HISTB_HDMI_SCDC_DDC_CONFLICT);
		histb_hdmi_updateb(hdmi, HISTB_HDMI_SCDC_CTL,
				   HISTB_HDMI_SCDC_ACCESS, 0);
		histb_hdmi_updateb(hdmi, HISTB_HDMI_SCDC_CTL,
				   HISTB_HDMI_SCDC_ACCESS,
				   HISTB_HDMI_SCDC_ACCESS);
		if (!(histb_hdmi_readb(hdmi, HISTB_HDMI_SCDC_INTR0) &
		      HISTB_HDMI_SCDC_DDC_CONFLICT))
			return 0;
		usleep_range(10, 20);
	}

	return -EBUSY;
}

static int histb_hdmi_scdc_transfer(struct histb_hdmi *hdmi, u8 offset,
				    u8 *buf, size_t len, bool read)
{
	u8 command, count, status;
	size_t i;
	int ret;

	if (!len || len > HISTB_HDMI_SCDC_MAX_TRANSFER)
		return -EINVAL;

	ret = histb_hdmi_ddc_enable(hdmi);
	if (ret)
		return ret;

	ret = histb_hdmi_scdc_select(hdmi);
	if (ret)
		goto release;

	histb_hdmi_ddc_abort(hdmi, HISTB_HDMI_DDC_CMD_SCDC);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_ADDR,
			  HISTB_HDMI_DDC_SCDC_ADDR);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_SEGMENT, 0);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_OFFSET, offset);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_COUNT_HIGH, len >> 8);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_COUNT_LOW, len);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_CMD,
			  HISTB_HDMI_DDC_CMD_FIFO_CLEAR |
			  HISTB_HDMI_DDC_CMD_SCDC);

	if (read) {
		command = HISTB_HDMI_DDC_CMD_READ |
			  HISTB_HDMI_DDC_CMD_SCDC;
		histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_CMD, command);

		for (i = 0; i < len; i++) {
			ret = read_poll_timeout(histb_hdmi_readb, count,
						(count &
						 HISTB_HDMI_DDC_FIFO_COUNT_MASK) ||
						(histb_hdmi_readb(hdmi,
						 HISTB_HDMI_DDC_STATUS) &
						 HISTB_HDMI_DDC_ERROR),
						5,
						HISTB_HDMI_DDC_BYTE_TIMEOUT_US,
						false, hdmi,
						HISTB_HDMI_DDC_FIFO_COUNT);
			if (ret)
				goto abort;

			status = histb_hdmi_readb(hdmi, HISTB_HDMI_DDC_STATUS);
			ret = histb_hdmi_ddc_status_error(status);
			if (ret)
				goto abort;

			buf[i] = histb_hdmi_readb(hdmi, HISTB_HDMI_DDC_DATA);
			udelay(5);
		}
	} else {
		for (i = 0; i < len; i++)
			histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_DATA, buf[i]);

		command = HISTB_HDMI_DDC_CMD_WRITE |
			  HISTB_HDMI_DDC_CMD_SCDC;
		histb_hdmi_writeb(hdmi, HISTB_HDMI_DDC_CMD, command);
	}

	ret = read_poll_timeout(histb_hdmi_readb, status,
				!(status & HISTB_HDMI_DDC_IN_PROGRESS) ||
				(status & HISTB_HDMI_DDC_ERROR), 10,
				HISTB_HDMI_DDC_BYTE_TIMEOUT_US, false, hdmi,
				HISTB_HDMI_DDC_STATUS);
	if (ret)
		goto abort;
	ret = histb_hdmi_ddc_status_error(status);

abort:
	histb_hdmi_ddc_abort(hdmi, HISTB_HDMI_DDC_CMD_SCDC);
release:
	histb_hdmi_ddc_disable(hdmi);

	return ret;
}

static int histb_hdmi_ddc_master_xfer(struct i2c_adapter *adapter,
				      struct i2c_msg *msgs, int num)
{
	struct histb_hdmi *hdmi = i2c_get_adapdata(adapter);
	int ret;

	guard(mutex)(&hdmi->ddc_lock);
	if (!READ_ONCE(hdmi->aux_enabled))
		return -EHOSTDOWN;

	if (num == 1 && msgs[0].addr == HISTB_HDMI_SCDC_I2C_ADDR &&
	    !(msgs[0].flags & ~I2C_M_DMA_SAFE) && msgs[0].len >= 2) {
		ret = histb_hdmi_scdc_transfer(hdmi, msgs[0].buf[0],
					       msgs[0].buf + 1,
					       msgs[0].len - 1, false);
		return ret ? ret : num;
	}

	if (num == 2 && msgs[0].addr == HISTB_HDMI_SCDC_I2C_ADDR &&
	    msgs[1].addr == HISTB_HDMI_SCDC_I2C_ADDR &&
	    !(msgs[0].flags & ~I2C_M_DMA_SAFE) && msgs[0].len == 1 &&
	    (msgs[1].flags & ~I2C_M_DMA_SAFE) == I2C_M_RD && msgs[1].len) {
		ret = histb_hdmi_scdc_transfer(hdmi, msgs[0].buf[0],
					       msgs[1].buf, msgs[1].len, true);
		return ret ? ret : num;
	}

	return -EOPNOTSUPP;
}

static u32 histb_hdmi_ddc_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm histb_hdmi_ddc_algorithm = {
	.master_xfer = histb_hdmi_ddc_master_xfer,
	.functionality = histb_hdmi_ddc_functionality,
};

static enum drm_connector_status histb_hdmi_bridge_detect(struct drm_bridge *bridge)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	u8 hpd_rsen;

	if (!READ_ONCE(hdmi->aux_enabled))
		return connector_status_unknown;
	hpd_rsen = histb_hdmi_readb(hdmi, HISTB_HDMI_TPI_HPD_RSEN);

	return hpd_rsen & HISTB_HDMI_HPD_CONNECTED ?
		       connector_status_connected : connector_status_disconnected;
}

/* Called with audio_lock held. Never revive audio on a disabled display or
 * trust a cached ELD while HPD is low. A live link may survive an HPD pulse
 * without another atomic_enable, so PCM startup must restore that case too.
 */
static int histb_hdmi_audio_restore(void *data)
{
	struct histb_hdmi *hdmi = data;
	struct drm_connector *connector = hdmi->active_connector;

	lockdep_assert_held(&hdmi->audio_lock);
	if (!hdmi->aux_enabled || !hdmi->pixel_enabled || !hdmi->mode_ready ||
	    !connector || histb_hdmi_bridge_detect(&hdmi->bridge) !=
			 connector_status_connected ||
	    !connector->display_info.is_hdmi || !connector->display_info.has_audio)
		return -ENODEV;

	return histb_audio_enable(&hdmi->audio, connector);
}

/* Stop under audio_lock, before dropping pixel/auxiliary clocks. A queued
 * property worker observes hdcp_requested=false after the modeset unlocks.
 */
static void histb_hdmi_hdcp_stop(struct histb_hdmi *hdmi)
{
	guard(mutex)(&hdmi->ddc_lock);
	if (hdmi->hdcp_protocol == 22)
		histb_hdcp22_stop(hdmi->hdcp22);
	else if (hdmi->hdcp_protocol == 14)
		histb_hdcp14_stop(hdmi->hdcp14);
	hdmi->hdcp_protocol = 0;
	/* Cipher reset covers other secure-world channels: never assert it here. */
	hdmi->cipher_active = false;
}

static int histb_hdmi_hdcp_start(struct histb_hdmi *hdmi)
{
	int ret = -ENODEV;

	lockdep_assert_held(&hdmi->audio_lock);
	guard(mutex)(&hdmi->ddc_lock);
	if (hdmi->hdcp22) {
		ret = histb_hdcp22_start(hdmi->hdcp22);
		if (!ret) {
			hdmi->hdcp_protocol = 22;
			drm_dbg_kms(hdmi->bridge.dev,
				    "HDCP 2.2 program code-check passed; authentication started\n");
			return 0;
		}
		drm_dbg_kms(hdmi->bridge.dev, "HDCP 2.2 startup failed: %pe\n",
			    ERR_PTR(ret));
		histb_hdcp22_stop(hdmi->hdcp22);
	}
	/* Content Type 0 permits a genuine 1.4 fallback, never plaintext
	 * masquerading as Enabled. No Content Type 1 property is exposed.
	 */
	if (!hdmi->hdcp14)
		return ret;
	ret = reset_control_deassert(hdmi->cipher_reset);
	if (ret)
		return ret;
	/* CV200 BSP sets clk_valid=0: CRG48[12] is a selector, not a gate.
	 * Preserve the boot clock selection; only own the real cipher reset.
	 */
	hdmi->cipher_active = true;
	ret = histb_hdcp14_start(hdmi->hdcp14);
	if (!ret)
		hdmi->hdcp_protocol = 14;
	else {
		histb_hdcp14_stop(hdmi->hdcp14);
		hdmi->cipher_active = false;
	}
	return ret;
}

static void histb_hdmi_hdcp_queue(struct histb_hdmi *hdmi, unsigned long delay)
{
	lockdep_assert_held(&hdmi->audio_lock);
	if (hdmi->attached && hdmi->hdcp_connector)
		mod_delayed_work(system_wq, &hdmi->hdcp_work, delay);
}

static void histb_hdmi_hdcp_work(struct work_struct *work)
{
	struct histb_hdmi *hdmi = container_of(to_delayed_work(work),
					      struct histb_hdmi, hdcp_work);
	struct drm_connector *connector;
	struct drm_device *drm;
	unsigned long delay = msecs_to_jiffies(50);
	bool retry = false;
	int ret;

	/* Detach drains this worker before bridge.dev or the connector can go
	 * away. Never wait for connection_mutex: teardown may already own it.
	 */
	mutex_lock(&hdmi->audio_lock);
	if (!hdmi->attached || !hdmi->hdcp_connector) {
		mutex_unlock(&hdmi->audio_lock);
		return;
	}
	drm = hdmi->bridge.dev;
	mutex_unlock(&hdmi->audio_lock);
	if (!ww_mutex_trylock(&drm->mode_config.connection_mutex.mutex, NULL)) {
		mutex_lock(&hdmi->audio_lock);
		histb_hdmi_hdcp_queue(hdmi, delay);
		mutex_unlock(&hdmi->audio_lock);
		return;
	}
	mutex_lock(&hdmi->audio_lock);
	connector = hdmi->hdcp_connector;
	if (!hdmi->attached || !connector)
		goto unlock;
	if (!hdmi->hdcp_requested || !hdmi->aux_enabled || !hdmi->mode_ready ||
	    histb_hdmi_bridge_detect(&hdmi->bridge) != connector_status_connected) {
		histb_hdmi_hdcp_stop(hdmi);
		if (connector->state->content_protection ==
		    DRM_MODE_CONTENT_PROTECTION_ENABLED)
			drm_hdcp_update_content_protection(connector,
					DRM_MODE_CONTENT_PROTECTION_DESIRED);
		retry = hdmi->hdcp_requested && hdmi->mode_ready && hdmi->aux_enabled;
		delay = msecs_to_jiffies(500);
		goto unlock;
	}
	retry = true;
	if (!hdmi->hdcp_protocol) {
		ret = histb_hdmi_hdcp_start(hdmi);
		if (ret)
			goto failed;
	}
	mutex_lock(&hdmi->ddc_lock);
	ret = hdmi->hdcp_protocol == 22 ?
		histb_hdcp22_poll(hdmi->hdcp22, connector) :
		histb_hdcp14_poll(hdmi->hdcp14, connector);
	mutex_unlock(&hdmi->ddc_lock);
	if (ret < 0)
		goto failed;
	if (ret == 1 && connector->state->content_protection ==
	    DRM_MODE_CONTENT_PROTECTION_DESIRED)
		drm_hdcp_update_content_protection(connector,
					DRM_MODE_CONTENT_PROTECTION_ENABLED);
	else if (!ret && connector->state->content_protection ==
		 DRM_MODE_CONTENT_PROTECTION_ENABLED)
		drm_hdcp_update_content_protection(connector,
					DRM_MODE_CONTENT_PROTECTION_DESIRED);
	goto unlock;

failed:
	histb_hdmi_hdcp_stop(hdmi);
	if (connector->state->content_protection == DRM_MODE_CONTENT_PROTECTION_ENABLED)
		drm_hdcp_update_content_protection(connector,
					DRM_MODE_CONTENT_PROTECTION_DESIRED);
	dev_warn_ratelimited(hdmi->dev, "HDCP authentication unavailable/failed: %d\n", ret);
	delay = msecs_to_jiffies(2000);
unlock:
	if (retry)
		histb_hdmi_hdcp_queue(hdmi, delay);
	mutex_unlock(&hdmi->audio_lock);
	drm_modeset_unlock(&drm->mode_config.connection_mutex);
}

/* Registered after the managed connector, so it runs before its cleanup. */
static void histb_hdmi_connector_release(struct drm_device *drm, void *data);

int histb_hdmi_connector_init(struct drm_bridge *bridge,
			      struct drm_connector *connector)
{
	struct histb_hdmi *hdmi;
	int ret;

	if (!of_device_is_compatible(bridge->of_node, "hisilicon,hi3798cv200-hdmi"))
		return 0;
	hdmi = bridge_to_histb_hdmi(bridge);
	ret = drm_mode_create_hdmi_colorspace_property(connector,
					BIT(DRM_MODE_COLORIMETRY_BT2020_RGB) |
					BIT(DRM_MODE_COLORIMETRY_BT2020_YCC));
	if (ret)
		return ret;
	ret = drm_connector_attach_colorspace_property(connector);
	if (ret)
		return ret;
	ret = drm_connector_attach_hdr_output_metadata_property(connector);
	if (ret)
		return ret;
	if (hdmi->hdcp14 || hdmi->hdcp22) {
		ret = drm_connector_attach_content_protection_property(connector, false);
		if (ret)
			return ret;
	}
	mutex_lock(&hdmi->audio_lock);
	hdmi->connector_dev = connector->dev;
	if (hdmi->hdcp14 || hdmi->hdcp22)
		hdmi->hdcp_connector = connector;
	mutex_unlock(&hdmi->audio_lock);
#if IS_ENABLED(CONFIG_CEC_CORE)
	{
		struct cec_connector_info info;

		cec_fill_conn_info_from_drm(&info, connector);
		cec_s_conn_info(hdmi->cec_adap, &info);
	}
#endif
	return drmm_add_action_or_reset(connector->dev,
					histb_hdmi_connector_release, hdmi);
}
EXPORT_SYMBOL_GPL(histb_hdmi_connector_init);

static bool histb_hdmi_scdc_enable(struct histb_hdmi *hdmi,
				   struct drm_connector *connector);

static void histb_hdmi_hpd_work(struct work_struct *work)
{
	struct histb_hdmi *hdmi = container_of(to_delayed_work(work),
					      struct histb_hdmi, hpd_work);
	enum drm_connector_status status;
	bool status_changed;
	u8 intr1;

	if (!READ_ONCE(hdmi->hpd_enabled))
		return;

	if (hdmi->hpd_polling) {
		/* R002 polls the aggregate interrupt and TPI state in process context. */
		intr1 = histb_hdmi_readb(hdmi, HISTB_HDMI_INTR1) &
			HISTB_HDMI_INTR_HPD_RSEN;

		/* INTR1 is write-one-to-clear; do this even without a GIC IRQ. */
		if (intr1)
			histb_hdmi_writeb(hdmi, HISTB_HDMI_INTR1, intr1);
	}

	status = histb_hdmi_bridge_detect(&hdmi->bridge);
	status_changed = READ_ONCE(hdmi->last_reported_status) != status;
	if (status != connector_status_connected) {
		mutex_lock(&hdmi->audio_lock);
		hdmi->scdc_enabled = false;
		histb_audio_disable(&hdmi->audio);
		histb_hdmi_hdcp_stop(hdmi);
		histb_hdmi_hdcp_queue(hdmi, 0);
		mutex_unlock(&hdmi->audio_lock);
	}
#if IS_ENABLED(CONFIG_CEC_CORE)
	if (status != connector_status_connected)
		cec_phys_addr_invalidate(hdmi->cec_adap);
#endif
	if (status == connector_status_connected) {
		mutex_lock(&hdmi->audio_lock);
		/* A TV can lose SCDC while KMS keeps the same active mode. */
		if (status_changed && hdmi->mode_ready && hdmi->pixel_enabled &&
		    hdmi->high_tmds && hdmi->active_connector) {
			histb_hdmi_tmds_enable(hdmi, false);
			hdmi->scdc_enabled = false;
			if (histb_hdmi_scdc_enable(hdmi, hdmi->active_connector))
				histb_hdmi_tmds_enable(hdmi, true);
			else
				dev_err(hdmi->dev,
					"HDMI SCDC hotplug restore failed; keeping TMDS disabled\n");
		}
		/* An already-running PCM keeps its configuration across HPD loss. */
		if (!hdmi->audio.active && hdmi->mode_ready && hdmi->pixel_enabled &&
		    (!hdmi->high_tmds || hdmi->scdc_enabled) &&
		    histb_hdmi_audio_restore(hdmi) && hdmi->audio.configured)
			dev_warn_ratelimited(hdmi->dev,
					     "HDMI PCM hotplug restore failed\n");
		mutex_unlock(&hdmi->audio_lock);
	}
	if (status_changed) {
		WRITE_ONCE(hdmi->last_reported_status, status);
		drm_bridge_hpd_notify(&hdmi->bridge, status);
	}

	if (hdmi->hpd_polling && READ_ONCE(hdmi->hpd_enabled))
		schedule_delayed_work(&hdmi->hpd_work,
				      msecs_to_jiffies(HISTB_HDMI_HPD_POLL_PERIOD_MS));
}

static irqreturn_t histb_hdmi_irq(int irq, void *data)
{
	struct histb_hdmi *hdmi = data;
	u8 intr1;

	/* A shared IRQ can be dispatched while the auxiliary clocks are off. */
	if (!READ_ONCE(hdmi->aux_enabled))
		return IRQ_NONE;

	intr1 = histb_hdmi_readb(hdmi, HISTB_HDMI_INTR1) &
		 HISTB_HDMI_INTR_HPD_RSEN;
	if (!intr1)
		return IRQ_NONE;

	/* INTR1 uses write-one-to-clear status bits. */
	histb_hdmi_writeb(hdmi, HISTB_HDMI_INTR1, intr1);

	if (READ_ONCE(hdmi->hpd_enabled))
		schedule_delayed_work(&hdmi->hpd_work, 0);

	return IRQ_HANDLED;
}

static void histb_hdmi_bridge_hpd_enable(struct drm_bridge *bridge)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);

	guard(mutex)(&hdmi->audio_lock);

	if (!hdmi->attached || !hdmi->aux_enabled)
		return;
	WRITE_ONCE(hdmi->hpd_enabled, true);
	WRITE_ONCE(hdmi->last_reported_status, -1);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_INTR1,
			  HISTB_HDMI_INTR_HPD_RSEN);
	if (hdmi->hpd_polling)
		histb_hdmi_writeb(hdmi, HISTB_HDMI_INTR1_MASK, 0);
	else
		histb_hdmi_writeb(hdmi, HISTB_HDMI_INTR1_MASK,
				  HISTB_HDMI_INTR_HPD_RSEN);

	/* Report an existing sink; R002 starts its periodic poll after 150 ms. */
	if (hdmi->hpd_polling)
		schedule_delayed_work(&hdmi->hpd_work,
				      msecs_to_jiffies(HISTB_HDMI_HPD_POLL_START_MS));
	else
		schedule_delayed_work(&hdmi->hpd_work, 0);
}

static void histb_hdmi_bridge_hpd_disable(struct drm_bridge *bridge)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);

	WRITE_ONCE(hdmi->hpd_enabled, false);
	if (hdmi->aux_enabled)
		histb_hdmi_writeb(hdmi, HISTB_HDMI_INTR1_MASK, 0);
	/* The core holds hpd_mutex, which hpd_work needs for notification. */
	cancel_delayed_work(&hdmi->hpd_work);
}

static void histb_hdmi_quiesce(struct histb_hdmi *hdmi)
{
	/* Close every HDCP enqueue path before draining either worker. Atomic
	 * callbacks use audio_lock too, but never wait for a worker here.
	 */
	mutex_lock(&hdmi->audio_lock);
	hdmi->attached = false;
	hdmi->hdcp_requested = false;
	mutex_unlock(&hdmi->audio_lock);

	/* Clear the core's connector callback, mask the source, then drain the
	 * IRQ and HPD worker before the final HDCP stop/cancel. Do not hold
	 * hpd_mutex, audio_lock or connection_mutex across the drains. Encoder
	 * cleanup and the managed connector release call us outside modeset locks.
	 */
	drm_bridge_hpd_disable(&hdmi->bridge);
	if (hdmi->tx_irq >= 0 && !hdmi->hpd_polling)
		synchronize_irq(hdmi->tx_irq);
	cancel_delayed_work_sync(&hdmi->hpd_work);
	mutex_lock(&hdmi->audio_lock);
	histb_hdmi_hdcp_stop(hdmi);
	histb_audio_disable(&hdmi->audio);
	mutex_unlock(&hdmi->audio_lock);
	cancel_delayed_work_sync(&hdmi->hdcp_work);
#if IS_ENABLED(CONFIG_CEC_CORE)
	cec_phys_addr_invalidate(hdmi->cec_adap);
	cec_s_conn_info(hdmi->cec_adap, NULL);
#endif

	mutex_lock(&hdmi->audio_lock);
	hdmi->hdcp_connector = NULL;
	hdmi->active_connector = NULL;
	mutex_unlock(&hdmi->audio_lock);
}

static void histb_hdmi_connector_release(struct drm_device *drm, void *data)
{
	struct histb_hdmi *hdmi = data;

	hdmi->connector_dev = NULL;
	histb_hdmi_quiesce(hdmi);
}

static int histb_hdmi_bridge_attach(struct drm_bridge *bridge,
				  enum drm_bridge_attach_flags flags)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	guard(mutex)(&hdmi->audio_lock);

	if (hdmi->removing)
		return -ENODEV;
	hdmi->attached = true;
	return 0;
}

static void histb_hdmi_bridge_detach(struct drm_bridge *bridge)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);

	/* Remove the managed callback as well: the provider may be removed
	 * before the DRM device's last reference is dropped.
	 */
	if (hdmi->connector_dev)
		drmm_release_action(hdmi->connector_dev,
				    histb_hdmi_connector_release, hdmi);
	else
		histb_hdmi_quiesce(hdmi);
}

static const struct drm_edid *
histb_hdmi_bridge_edid_read(struct drm_bridge *bridge,
			    struct drm_connector *connector)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	const struct drm_edid *edid;

	{
		guard(mutex)(&hdmi->ddc_lock);

		if (!READ_ONCE(hdmi->aux_enabled))
			return NULL;
		edid = drm_edid_read_custom(connector, histb_hdmi_read_edid_block, hdmi);
	}
#if IS_ENABLED(CONFIG_CEC_CORE)
	cec_s_phys_addr_from_edid(hdmi->cec_adap, edid ? drm_edid_raw(edid) : NULL);
#endif

	return edid;
}

static enum drm_mode_status
histb_hdmi_bridge_mode_valid(struct drm_bridge *bridge,
			     const struct drm_display_info *info,
			     const struct drm_display_mode *mode)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	unsigned long rate = mode->clock * 1000UL;
	unsigned long min_tmds_rate = rate;
	unsigned long rate_delta;
	long rounded_rate;

	if (mode->flags & (DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLSCAN))
		return MODE_NO_INTERLACE;
	if (mode->clock < 25000)
		return MODE_CLOCK_LOW;
	if (mode->clock > 594000)
		return MODE_CLOCK_HIGH;
	/* A 594 MHz pixel mode can use a 297 MHz TMDS link at 8-bit 4:2:0.
	 * Mode probing has no selected format yet; reject only when even the
	 * lowest rate advertised for this timing needs unsupported scrambling.
	 */
	if (info && drm_mode_is_420(info, mode))
		min_tmds_rate /= 2;
	if (min_tmds_rate > HISTB_HDMI14_MAX_TMDS_RATE &&
	    (!info || !info->is_hdmi || !info->hdmi.scdc.supported ||
	     !info->hdmi.scdc.scrambling.supported))
		return MODE_CLOCK_HIGH;

	rounded_rate = clk_round_rate(hdmi->clks[HISTB_HDMI_PIXEL_CLK_INDEX].clk,
				      rate);
	if (rounded_rate < 0)
		return MODE_CLOCK_RANGE;

	/* Accept clock plans within one percent of the requested pixel rate. */
	rate_delta = rounded_rate > rate ? rounded_rate - rate :
					       rate - rounded_rate;
	if (rate_delta > rate / 100)
		return MODE_CLOCK_RANGE;

	return MODE_OK;
}

static enum drm_mode_status
histb_hdmi_tmds_char_rate_valid(const struct drm_bridge *bridge,
				const struct drm_display_mode *mode,
				unsigned long long tmds_rate)
{
	if (tmds_rate < 25000000)
		return MODE_CLOCK_LOW;
	if (tmds_rate > HISTB_HDMI_MAX_TMDS_RATE)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static int histb_hdmi_clear_infoframe(struct drm_bridge *bridge,
				      enum hdmi_infoframe_type type)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	int ret;
	guard(mutex)(&hdmi->packet_lock);

	ret = histb_hdmi_infoframe_select(hdmi, type);
	if (ret)
		return ret;

	histb_hdmi_infoframe_enable(hdmi, false);
	ret = histb_hdmi_infoframe_wait(hdmi, false);
	histb_hdmi_infoframe_select(hdmi, HDMI_INFOFRAME_TYPE_AVI);
	return ret;
}

static int histb_hdmi_write_infoframe(struct drm_bridge *bridge,
				      enum hdmi_infoframe_type type,
				      const u8 *buffer, size_t len)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	unsigned int i;
	int ret;
	guard(mutex)(&hdmi->packet_lock);

	if (!buffer || !len || len > HISTB_HDMI_INFOFRAME_BYTES)
		return -EINVAL;

	ret = histb_hdmi_infoframe_select(hdmi, type);
	if (ret)
		return ret;

	histb_hdmi_infoframe_enable(hdmi, false);
	ret = histb_hdmi_infoframe_wait(hdmi, false);
	if (ret)
		goto restore_selector;
	for (i = 0; i < HISTB_HDMI_INFOFRAME_BYTES; i++)
		histb_hdmi_writeb(hdmi, HISTB_HDMI_INFOFRAME_DATA +
				  i * sizeof(u32), i < len ? buffer[i] : 0);
	histb_hdmi_infoframe_enable(hdmi, true);
	ret = histb_hdmi_infoframe_wait(hdmi, true);
	if (ret)
		histb_hdmi_infoframe_enable(hdmi, false);
restore_selector:
	histb_hdmi_infoframe_select(hdmi, HDMI_INFOFRAME_TYPE_AVI);
	return ret;
}
static int histb_hdmi_audio_program(void *data,
				    const struct hdmi_codec_params *params)
{
	struct histb_hdmi *hdmi = data;
	u8 buffer[HDMI_INFOFRAME_SIZE(AUDIO)];
	u32 n = histb_audio_n(params->sample_rate);
	u8 len = params->iec.status[4] & 0x0f;
	bool hbr = histb_audio_hbr(params);
	u8 lanes = GENMASK(DIV_ROUND_UP(params->channels, 2) + 3, 4);
	int i, ret;

	/* product/Hi3798CV200/hdmi_platform.h:48-53 and imagev200
	 * hdmi_hal.c:1774-1795 select HBRA, not the LPCM I2S input ID.
	 */
	ret = regmap_update_bits(hdmi->perictrl, hdmi->perictrl_offset,
				 HISTB_HDMI_PERI_AUDIO_MASK,
				 hbr ? BIT(3) : HISTB_HDMI_PERI_AUDIO_I2S);
	if (ret)
		return ret;
	ret = hdmi_audio_infoframe_pack_only(&params->cea, buffer, sizeof(buffer));
	if (ret < 0)
		return ret;
	ret = histb_hdmi_write_infoframe(&hdmi->bridge, HDMI_INFOFRAME_TYPE_AUDIO,
					 buffer, ret);
	if (ret)
		return ret;
	/* imagev200/driver/tx/tx_hdmi/si_mod_tx_hdmi.c:1917-2049:
	 * HBR AUD_MODE=4, layout=0, ASRC[7:4]=0. LPCM enables one bit
	 * per SD pair and uses layout 1 above stereo. I2S FIFO map 0xe4
	 * is identity (si_drv_tx_regs.h:12058-12071). CTS follows the
	 * physical 192 kHz frame clock for HBR; its four packed pairs
	 * represent a 768 kHz two-channel IEC61937 carrier.
	 */
	histb_hdmi_writeb(hdmi, HISTB_HDMI_AUD_MODE, 0);
	/* Keep CTS/N requests off until N, I2S and the input FIFO are ready. */
	histb_hdmi_writeb(hdmi, HISTB_HDMI_ACR_CTRL, 0x0c);
	for (i = 0; i < 5; i++)
		histb_hdmi_writeb(hdmi, HISTB_HDMI_REG(0xa, 0x1e + i),
				   params->iec.status[i]);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_I2S_IN_SIZE, len);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_I2S_IN_CTRL, 0x60);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_I2S_IN_MAP, 0xe4);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_ASRC, GENMASK(7, 4),
			   hbr ? 0 : BIT(4));
	for (i = 0; i < 3; i++)
		histb_hdmi_writeb(hdmi, HISTB_HDMI_N_SVAL + i * 4, n >> (i * 8));
	histb_hdmi_writeb(hdmi, HISTB_HDMI_AUD_MODE, hbr ? BIT(2) : lanes);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_AUDP_TXCTRL, BIT(1),
			   !hbr && params->channels > 2 ? BIT(1) : 0);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_AIP_HDMI2MHL, 0);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_TPI_DOWN_SMPL_CTRL, BIT(2), 0);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_TPI_AUD_CONFIG, 0);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_TPI_AUD_FS,
			   0x80 | (params->iec.status[3] & 0x0f) |
			   ((params->iec.status[3] >> 2) & 0x30));
	histb_hdmi_writeb(hdmi, HISTB_HDMI_AIP_RST,
			   HISTB_HDMI_AIP_RST_AUDIO | HISTB_HDMI_AIP_RST_FIFO);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_AIP_RST, 0);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_AUD_EN,
			   HISTB_HDMI_AUD_IN_EN, HISTB_HDMI_AUD_IN_EN);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_ACR_CTRL,
			   HISTB_HDMI_ACR_CTS_REQ_EN, HISTB_HDMI_ACR_CTS_REQ_EN);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_PKT_FILTER1,
			   HISTB_HDMI_AUDIO_PKT_ENABLE, HISTB_HDMI_AUDIO_PKT_ENABLE);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_AUDP_TXCTRL, BIT(7), 0);
	return 0;
}

static void histb_hdmi_source_scrambling_set(struct histb_hdmi *hdmi,
					     bool enable)
{
	histb_hdmi_writeb(hdmi, HISTB_HDMI_TXC_DATA_DIV,
			  enable ? 0x02 : 0x00);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_SCR_CTRL,
			   HISTB_HDMI_SCRAMBLE_MASK,
			   enable ? HISTB_HDMI_SCRAMBLE_MASK : 0);
}

static bool histb_hdmi_scdc_enable(struct histb_hdmi *hdmi,
				   struct drm_connector *connector)
{
	u8 sink_version;

	if (!connector->ddc)
		return false;

	if (!drm_scdc_readb(connector->ddc, SCDC_SINK_VERSION, &sink_version))
		drm_scdc_writeb(connector->ddc, SCDC_SOURCE_VERSION,
				min_t(u8, sink_version,
				      HISTB_HDMI_SCDC_SOURCE_VERSION));

	if (!drm_scdc_set_high_tmds_clock_ratio(connector, true))
		return false;
	if (!drm_scdc_set_scrambling(connector, true)) {
		drm_scdc_set_high_tmds_clock_ratio(connector, false);
		return false;
	}

	histb_hdmi_source_scrambling_set(hdmi, true);
	hdmi->scdc_enabled = true;

	return true;
}

static void histb_hdmi_scdc_disable(struct histb_hdmi *hdmi)
{
	struct drm_connector *connector = hdmi->active_connector;

	histb_hdmi_source_scrambling_set(hdmi, false);
	if (!hdmi->scdc_enabled)
		return;
	hdmi->scdc_enabled = false;
	if (!connector)
		return;

	drm_scdc_set_scrambling(connector, false);
	drm_scdc_set_high_tmds_clock_ratio(connector, false);
}

/* Leave the always-on auxiliary domain usable after a failed modeset. */
static void histb_hdmi_abort_mode(struct histb_hdmi *hdmi)
{
	int ret;

	if (!hdmi->aux_enabled)
		return;

	histb_hdmi_tmds_enable(hdmi, false);
	histb_hdmi_scdc_disable(hdmi);
	histb_hdmi_disable_infoframes(hdmi);

	/* Clear any partial TX/PHY state, but keep the auxiliary clocks enabled. */
	ret = reset_control_bulk_assert(ARRAY_SIZE(hdmi->resets), hdmi->resets);
	if (ret) {
		dev_warn(hdmi->dev,
			 "failed to assert HDMI resets after modeset error: %pe\n",
			 ERR_PTR(ret));
		return;
	}
	usleep_range(1000, 2000);
	ret = reset_control_bulk_deassert(ARRAY_SIZE(hdmi->resets),
					 hdmi->resets);
	if (ret)
		dev_warn(hdmi->dev,
			 "failed to release HDMI resets after modeset error: %pe\n",
			 ERR_PTR(ret));
	else {
		usleep_range(1000, 2000);
#if IS_ENABLED(CONFIG_CEC_CORE)
		histb_hdmi_cec_hw_config(hdmi);
#endif
	}
}

/* drm_display_info in this kernel does not retain CTA Colorimetry DB bits.
 * Read the immutable connector EDID blob while the modeset lock is held.
 */
static bool histb_hdmi_sink_bt2020(struct drm_connector *connector, bool rgb)
{
	const struct drm_property_blob *blob = connector->edid_blob_ptr;
	const u8 *edid, *cta;
	size_t block, pos, end, length;

	if (!blob || blob->length < EDID_LENGTH)
		return false;
	edid = blob->data;
	for (block = 1; block <= edid[126] &&
	     block < blob->length / EDID_LENGTH; block++) {
		cta = edid + block * EDID_LENGTH;
		if (cta[0] != 0x02 || cta[1] < 3)
			continue;
		end = cta[2] ? cta[2] : 127;
		if (end < 4 || end > 127)
			continue;
		for (pos = 4; pos < end; pos += length + 1) {
			length = cta[pos] & 0x1f;
			if (pos + length + 1 > end)
				break;
			/* CTA extended tag 5: BT.2020 RGB bit 7, NCL YCC bit 6. */
			if ((cta[pos] >> 5) == 7 && length >= 3 &&
			    cta[pos + 1] == 5 && (cta[pos + 2] & BIT(rgb ? 7 : 6)))
				return true;
		}
	}
	return false;
}

static int histb_hdmi_bridge_atomic_check(struct drm_bridge *bridge,
					  struct drm_bridge_state *bridge_state,
					  struct drm_crtc_state *crtc_state,
					  struct drm_connector_state *conn_state)
{
	struct hdmi_avi_infoframe *avi;
	int ret;
	struct drm_connector_state *old_state =
		drm_atomic_get_old_connector_state(bridge_state->base.state,
						   conn_state->connector);
	const struct hdr_output_metadata *metadata;
	const struct hdr_metadata_infoframe *type1;
	const struct hdr_static_metadata *sink =
		&conn_state->connector->hdr_sink_metadata.hdmi_type1;
	unsigned int i;

	/* The VDP supplies nonlinear BT.2020 RGB. TX either bypasses CSC for
	 * RGB or uses its native BT.2020 NCL RGB-to-limited-YCbCr conversion.
	 */
	if (conn_state->hdr_output_metadata) {
		if (conn_state->hdr_output_metadata->length != sizeof(*metadata))
			return -EINVAL;
		metadata = conn_state->hdr_output_metadata->data;
		type1 = &metadata->hdmi_metadata_type1;
		if (metadata->metadata_type || type1->metadata_type ||
		    (type1->eotf != HDMI_EOTF_SMPTE_ST2084 &&
		     type1->eotf != HDMI_EOTF_BT_2100_HLG) ||
		    !(sink->eotf & BIT(type1->eotf)) ||
		    !(sink->metadata_type & BIT(0)) ||
		    !conn_state->connector->display_info.is_hdmi ||
		    (conn_state->colorspace != DRM_MODE_COLORIMETRY_BT2020_RGB &&
		     conn_state->colorspace != DRM_MODE_COLORIMETRY_BT2020_YCC))
			return -EINVAL;
		for (i = 0; i < ARRAY_SIZE(type1->display_primaries); i++)
			if (type1->display_primaries[i].x > 50000 ||
			    type1->display_primaries[i].y > 50000)
				return -EINVAL;
		if (type1->white_point.x > 50000 || type1->white_point.y > 50000 ||
		    (type1->max_display_mastering_luminance &&
		     type1->min_display_mastering_luminance >
		     (u32)type1->max_display_mastering_luminance * 10000) ||
		    (type1->max_cll && type1->max_fall > type1->max_cll))
			return -EINVAL;
	} else if (conn_state->colorspace != DRM_MODE_COLORIMETRY_DEFAULT) {
		/* No wide-gamut SDR conversion exists in this VDP/TX route. */
		return -EINVAL;
	}

	ret = drm_atomic_helper_connector_hdmi_check(conn_state->connector,
						     bridge_state->base.state);
	if (ret)
		return ret;
	if (conn_state->hdmi.tmds_char_rate > HISTB_HDMI14_MAX_TMDS_RATE &&
	    (!conn_state->connector->display_info.hdmi.scdc.supported ||
	     !conn_state->connector->display_info.hdmi.scdc.scrambling.supported))
		return -EINVAL;
	if (conn_state->hdr_output_metadata) {
		bool rgb = conn_state->hdmi.output_format == HDMI_COLORSPACE_RGB;

		if (conn_state->hdmi.output_bpc < 10 ||
		    rgb != (conn_state->colorspace == DRM_MODE_COLORIMETRY_BT2020_RGB) ||
		    !histb_hdmi_sink_bt2020(conn_state->connector, rgb))
			return -EINVAL;
	}
	/* Bridge packets are programmed during enable. Metadata-only updates
	 * must therefore traverse disable/enable too, including HDR -> SDR.
	 */
	if (old_state &&
	    (!drm_connector_atomic_hdr_metadata_equal(old_state, conn_state) ||
	     old_state->colorspace != conn_state->colorspace ||
	     old_state->content_protection != conn_state->content_protection))
		crtc_state->mode_changed = true;

	avi = &conn_state->hdmi.infoframes.avi.data.avi;
	if (conn_state->hdmi.output_format == HDMI_COLORSPACE_RGB) {
		if (conn_state->hdmi.broadcast_rgb ==
		    DRM_HDMI_BROADCAST_RGB_LIMITED)
			return -EINVAL;

		conn_state->hdmi.is_limited_range = false;
		avi->quantization_range = HDMI_QUANTIZATION_RANGE_FULL;
	} else {
		conn_state->hdmi.is_limited_range = true;
		avi->colorimetry = conn_state->hdr_output_metadata ?
			HDMI_COLORIMETRY_EXTENDED :
			histb_hdmi_colorimetry(&crtc_state->adjusted_mode);
		if (conn_state->hdr_output_metadata)
			avi->extended_colorimetry = HDMI_EXTENDED_COLORIMETRY_BT2020;
		avi->quantization_range = HDMI_QUANTIZATION_RANGE_DEFAULT;
		avi->ycc_quantization_range =
			HDMI_YCC_QUANTIZATION_RANGE_LIMITED;
	}

	return 0;
}

static void
histb_hdmi_bridge_atomic_pre_enable(struct drm_bridge *bridge,
				    struct drm_bridge_state *old_bridge_state)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	struct drm_atomic_state *state = old_bridge_state->base.state;
	struct drm_connector_state *conn_state;
	struct drm_connector *connector;
	struct drm_crtc_state *crtc_state;
	struct clk *pixel_clk;
	unsigned long pixel_rate, actual_rate;
	long rounded_rate;
	int ret;

	guard(mutex)(&hdmi->audio_lock);
	connector = drm_atomic_get_new_connector_for_encoder(state,
							     bridge->encoder);
	if (!connector) {
		drm_err(bridge->dev, "failed to find HDMI connector state\n");
		return;
	}

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (!conn_state || !conn_state->crtc) {
		drm_err(bridge->dev, "missing active HDMI connector state\n");
		return;
	}

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (!crtc_state) {
		drm_err(bridge->dev, "missing HDMI CRTC state\n");
		return;
	}

	pixel_clk = hdmi->clks[HISTB_HDMI_PIXEL_CLK_INDEX].clk;
	pixel_rate = crtc_state->adjusted_mode.clock * 1000UL;
	rounded_rate = clk_round_rate(pixel_clk, pixel_rate);
	if (rounded_rate <= 0) {
		drm_err(bridge->dev, "pixel clock %lu Hz has no valid PLL plan\n",
			pixel_rate);
		return;
	}

	ret = clk_set_rate(pixel_clk, pixel_rate);
	if (ret) {
		drm_err(bridge->dev, "failed to set %lu Hz pixel clock: %pe\n",
			pixel_rate, ERR_PTR(ret));
		return;
	}
	actual_rate = clk_get_rate(pixel_clk);
	if (actual_rate != rounded_rate) {
		drm_err(bridge->dev,
			"pixel clock did not lock: requested %lu Hz, expected %ld Hz, actual %lu Hz\n",
			pixel_rate, rounded_rate, actual_rate);
		return;
	}

	/* CLKRATIO describes the relationship inside TX; CRG158 bit 4 must
	 * also divide the physical PCLKNX clock for YUV420 output.
	 */
	ret = clk_set_rate(hdmi->clks[HISTB_HDMI_PIXELNX_CLK_INDEX].clk,
		conn_state->hdmi.output_format == HDMI_COLORSPACE_YUV420 ?
		actual_rate / 2 : actual_rate);
	if (ret) {
		drm_err(bridge->dev, "failed to set HDMI PCLKNX divider: %pe\n",
			ERR_PTR(ret));
		return;
	}

	ret = clk_prepare_enable(pixel_clk);
	if (ret) {
		drm_err(bridge->dev, "failed to enable pixel clock: %pe\n",
			ERR_PTR(ret));
		return;
	}
	actual_rate = clk_get_rate(pixel_clk);
	if (actual_rate != rounded_rate) {
		drm_err(bridge->dev,
			"enabled pixel clock changed: expected %ld Hz, actual %lu Hz\n",
			rounded_rate, actual_rate);
		clk_disable_unprepare(pixel_clk);
		return;
	}

	hdmi->pixel_enabled = true;
	hdmi->active_connector = connector;
	/* Match the vendor CV200 clock-route settle time before PHY setup. */
	usleep_range(1000, 2000);
	ret = histb_hdmi_vendor_reset_sequence(hdmi);
	if (ret) {
		drm_err(bridge->dev, "failed to reset HDMI bus/core: %pe\n",
			ERR_PTR(ret));
		goto disable_pixel;
	}
	ret = histb_hdmi_hw_init(hdmi);
	if (ret) {
		drm_err(bridge->dev, "failed to reinitialize HDMI TX: %pe\n",
			ERR_PTR(ret));
		goto disable_pixel;
	}
	histb_hdmi_configure_mode(hdmi, &crtc_state->adjusted_mode,
				  conn_state->hdmi.tmds_char_rate ?: pixel_rate,
				  connector->display_info.is_hdmi,
				  conn_state->hdmi.output_format,
				  conn_state->hdmi.output_bpc,
				  !!conn_state->hdr_output_metadata);
	hdmi->mode_ready = true;
	return;

disable_pixel:
	histb_hdmi_abort_mode(hdmi);
	hdmi->mode_ready = false;
	hdmi->active_connector = NULL;
	hdmi->pixel_enabled = false;
	clk_disable_unprepare(pixel_clk);
}

static void
histb_hdmi_bridge_atomic_enable(struct drm_bridge *bridge,
				struct drm_bridge_state *old_bridge_state)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	struct drm_atomic_state *state = old_bridge_state->base.state;
	struct drm_connector *connector;
	int ret;
	guard(mutex)(&hdmi->audio_lock);

	if (!hdmi->mode_ready)
		return;

	connector = drm_atomic_get_new_connector_for_encoder(state,
							     bridge->encoder);
	if (!connector) {
		drm_err(bridge->dev, "failed to find HDMI connector state\n");
		return;
	}

	if (hdmi->high_tmds && !histb_hdmi_scdc_enable(hdmi, connector)) {
		drm_err(bridge->dev,
			"failed to configure HDMI 2.0 SCDC; keeping TMDS disabled\n");
		return;
	}

	histb_hdmi_tmds_enable(hdmi, true);
	histb_hdmi_validate_link(hdmi);

	if (connector->display_info.is_hdmi) {
		ret = drm_atomic_helper_connector_hdmi_update_infoframes(connector,
									 state);
		if (ret) {
			drm_err(bridge->dev,
				"failed to program HDMI infoframes: %pe\n",
				ERR_PTR(ret));
			histb_hdmi_tmds_enable(hdmi, false);
			return;
		}
		ret = histb_audio_enable(&hdmi->audio, connector);
		if (ret)
			dev_err(hdmi->dev, "HDMI PCM restore failed: %d\n", ret);
	}
	if (hdmi->attached && (hdmi->hdcp14 || hdmi->hdcp22)) {
		hdmi->hdcp_requested = connector->state->content_protection !=
			DRM_MODE_CONTENT_PROTECTION_UNDESIRED;
		histb_hdmi_hdcp_queue(hdmi, 0);
	}
}

static void
histb_hdmi_bridge_atomic_disable(struct drm_bridge *bridge,
				 struct drm_bridge_state *old_bridge_state)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);

	guard(mutex)(&hdmi->audio_lock);
	hdmi->hdcp_requested = false;
	histb_hdmi_hdcp_stop(hdmi);
	histb_hdmi_hdcp_queue(hdmi, 0);
	histb_audio_disable(&hdmi->audio);
	histb_hdmi_tmds_enable(hdmi, false);
	histb_hdmi_scdc_disable(hdmi);
	histb_hdmi_disable_infoframes(hdmi);
	hdmi->mode_ready = false;
}

static void
histb_hdmi_bridge_atomic_post_disable(struct drm_bridge *bridge,
				      struct drm_bridge_state *old_bridge_state)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	struct clk *pixel_clk = hdmi->clks[HISTB_HDMI_PIXEL_CLK_INDEX].clk;

	guard(mutex)(&hdmi->audio_lock);
	if (hdmi->pixel_enabled) {
		clk_disable_unprepare(pixel_clk);
		hdmi->pixel_enabled = false;
	}
	hdmi->active_connector = NULL;
	hdmi->high_tmds = false;
}

static const struct drm_bridge_funcs histb_hdmi_bridge_funcs = {
	.attach = histb_hdmi_bridge_attach,
	.detach = histb_hdmi_bridge_detach,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_check = histb_hdmi_bridge_atomic_check,
	.atomic_pre_enable = histb_hdmi_bridge_atomic_pre_enable,
	.atomic_enable = histb_hdmi_bridge_atomic_enable,
	.atomic_disable = histb_hdmi_bridge_atomic_disable,
	.atomic_post_disable = histb_hdmi_bridge_atomic_post_disable,
	.detect = histb_hdmi_bridge_detect,
	.hpd_enable = histb_hdmi_bridge_hpd_enable,
	.hpd_disable = histb_hdmi_bridge_hpd_disable,
	.edid_read = histb_hdmi_bridge_edid_read,
	.mode_valid = histb_hdmi_bridge_mode_valid,
	.hdmi_tmds_char_rate_valid = histb_hdmi_tmds_char_rate_valid,
	.hdmi_clear_infoframe = histb_hdmi_clear_infoframe,
	.hdmi_write_infoframe = histb_hdmi_write_infoframe,
};

static void histb_hdmi_disable_resources(struct histb_hdmi *hdmi)
{
	guard(mutex)(&hdmi->audio_lock);
	hdmi->hdcp_requested = false;
	histb_hdmi_hdcp_stop(hdmi);
	histb_audio_disable(&hdmi->audio);
	if (hdmi->aux_enabled) {
		histb_hdmi_tmds_enable(hdmi, false);
		histb_hdmi_scdc_disable(hdmi);
	}
	if (hdmi->pixel_enabled) {
		struct clk *pixel_clk =
			hdmi->clks[HISTB_HDMI_PIXEL_CLK_INDEX].clk;

		clk_disable_unprepare(pixel_clk);
		hdmi->pixel_enabled = false;
	}
	if (hdmi->aux_enabled) {
		WRITE_ONCE(hdmi->aux_enabled, false);
		if (hdmi->tx_irq >= 0 && !hdmi->hpd_polling)
			synchronize_irq(hdmi->tx_irq);
		reset_control_bulk_assert(ARRAY_SIZE(hdmi->resets),
					  hdmi->resets);
		clk_bulk_disable_unprepare(HISTB_HDMI_AUX_CLK_COUNT,
					   hdmi->clks);
	}
}

static int __maybe_unused histb_hdmi_suspend(struct device *dev)
{
	struct histb_hdmi *hdmi = dev_get_drvdata(dev);
	struct clk *pixel_clk;
	int ret = 0;

#if IS_ENABLED(CONFIG_CEC_CORE)
	histb_hdmi_cec_pause(hdmi);
#endif
	WRITE_ONCE(hdmi->hpd_enabled, false);
	if (hdmi->aux_enabled)
		histb_hdmi_writeb(hdmi, HISTB_HDMI_INTR1_MASK, 0);
	if (hdmi->tx_irq >= 0)
		synchronize_irq(hdmi->tx_irq);
	cancel_delayed_work_sync(&hdmi->hpd_work);
	mutex_lock(&hdmi->audio_lock);
	hdmi->hdcp_requested = false;
	histb_hdmi_hdcp_stop(hdmi);
	mutex_unlock(&hdmi->audio_lock);
	cancel_delayed_work_sync(&hdmi->hdcp_work);
	if (hdmi->hdcp_connector && hdmi->bridge.dev) {
		ret = drm_modeset_lock(&hdmi->bridge.dev->mode_config.connection_mutex,
				       NULL);
		if (ret)
			return ret;
		if (hdmi->hdcp_connector->state->content_protection ==
		    DRM_MODE_CONTENT_PROTECTION_ENABLED)
			drm_hdcp_update_content_protection(hdmi->hdcp_connector,
					DRM_MODE_CONTENT_PROTECTION_DESIRED);
		drm_modeset_unlock(&hdmi->bridge.dev->mode_config.connection_mutex);
	}

	guard(mutex)(&hdmi->audio_lock);
	histb_audio_disable(&hdmi->audio);
	if (!hdmi->aux_enabled)
		return 0;

	histb_hdmi_tmds_enable(hdmi, false);
	histb_hdmi_scdc_disable(hdmi);
	histb_hdmi_disable_infoframes(hdmi);
	hdmi->mode_ready = false;

	if (hdmi->pixel_enabled) {
		pixel_clk = hdmi->clks[HISTB_HDMI_PIXEL_CLK_INDEX].clk;
		clk_disable_unprepare(pixel_clk);
		hdmi->pixel_enabled = false;
	}

	/* Tear down the resource lifecycle with all three reset lines. */
	ret = reset_control_bulk_assert(ARRAY_SIZE(hdmi->resets),
					hdmi->resets);
	WRITE_ONCE(hdmi->aux_enabled, false);
	if (hdmi->tx_irq >= 0 && !hdmi->hpd_polling)
		synchronize_irq(hdmi->tx_irq);
	clk_bulk_disable_unprepare(HISTB_HDMI_AUX_CLK_COUNT, hdmi->clks);
	hdmi->active_connector = NULL;

	return ret;
}

static int __maybe_unused histb_hdmi_resume(struct device *dev)
{
	struct histb_hdmi *hdmi = dev_get_drvdata(dev);
	int ret;

	guard(mutex)(&hdmi->audio_lock);
	if (hdmi->aux_enabled)
		return 0;

	ret = clk_bulk_prepare_enable(HISTB_HDMI_AUX_CLK_COUNT, hdmi->clks);
	if (ret)
		return ret;
	hdmi->aux_enabled = true;

	ret = reset_control_bulk_deassert(ARRAY_SIZE(hdmi->resets),
					hdmi->resets);
	if (ret)
		goto disable_aux;
	usleep_range(1000, 2000);

	ret = histb_hdmi_vendor_reset_sequence(hdmi);
	if (ret)
		goto disable_aux;
	ret = histb_hdmi_hw_init(hdmi);
	if (ret)
		goto disable_aux;
#if IS_ENABLED(CONFIG_CEC_CORE)
	hdmi->cec_suspended = false;
	histb_hdmi_cec_hw_config(hdmi);
#endif

	dev_dbg(dev, "HDMI transmitter resumed with vendor reset/init sequence\n");
	return 0;

disable_aux:
	reset_control_bulk_assert(ARRAY_SIZE(hdmi->resets), hdmi->resets);
	clk_bulk_disable_unprepare(HISTB_HDMI_AUX_CLK_COUNT, hdmi->clks);
	hdmi->aux_enabled = false;
	return ret;
}

static const struct dev_pm_ops histb_hdmi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(histb_hdmi_suspend, histb_hdmi_resume)
};

static u8 histb_hdmi_hdcp_read(void *ctx, u32 reg)
{
	return histb_hdmi_readb(ctx, reg);
}

static void histb_hdmi_hdcp_write(void *ctx, u32 reg, u8 value)
{
	histb_hdmi_writeb(ctx, reg, value);
}

static int histb_hdmi_hdcp_init(struct platform_device *pdev,
				struct histb_hdmi *hdmi)
{
	struct histb_hdcp_io *io = &hdmi->hdcp_io;
	struct resource *core = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct resource *cipher = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							     "cipher");
	struct resource *ctrl = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							   "hdcp-ctrl");

	io->dev = hdmi->dev;
	io->ctx = hdmi;
	io->read = histb_hdmi_hdcp_read;
	io->write = histb_hdmi_hdcp_write;
	io->peri = hdmi->perictrl;
	if (cipher) {
		if (resource_size(cipher) < 0x4000)
			return -EINVAL;
		io->cipher = devm_ioremap_resource(hdmi->dev, cipher);
		if (IS_ERR(io->cipher))
			return PTR_ERR(io->cipher);
		hdmi->cipher_reset = devm_reset_control_get_exclusive(hdmi->dev, "cipher");
		if (IS_ERR(hdmi->cipher_reset))
			return PTR_ERR(hdmi->cipher_reset);
		hdmi->hdcp14 = histb_hdcp14_create(io);
		if (IS_ERR(hdmi->hdcp14))
			return PTR_ERR(hdmi->hdcp14);
	}
	if (ctrl) {
		/* PRAM is already covered by the controller's 128 KiB region.
		 * Do not reserve/map it a second time with an overlapping resource.
		 */
		if (!core || resource_size(core) < 0x20000 || resource_size(ctrl) < 4)
			return -EINVAL;
		io->pram = hdmi->base + 0x10000;
		io->avcttpt = devm_ioremap_resource(hdmi->dev, ctrl);
		if (IS_ERR(io->avcttpt))
			return PTR_ERR(io->avcttpt);
		hdmi->hdcp22 = histb_hdcp22_create(io);
		if (IS_ERR(hdmi->hdcp22))
			return PTR_ERR(hdmi->hdcp22);
	}
	return 0;
}

static int histb_hdmi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_hdmi *hdmi;
	unsigned int i;
	int ret;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;
	hdmi->dev = dev;
	mutex_init(&hdmi->audio_lock);
	mutex_init(&hdmi->packet_lock);
	mutex_init(&hdmi->ddc_lock);
	INIT_DELAYED_WORK(&hdmi->hdcp_work, histb_hdmi_hdcp_work);
	hdmi->audio.lock = &hdmi->audio_lock;
	hdmi->audio.data = hdmi;
	hdmi->audio.program = histb_hdmi_audio_program;
	hdmi->audio.stop = histb_hdmi_audio_stop;
	hdmi->audio.restore = histb_hdmi_audio_restore;
	hdmi->audio.extended = true;
	INIT_DELAYED_WORK(&hdmi->hpd_work, histb_hdmi_hpd_work);
#if IS_ENABLED(CONFIG_CEC_CORE)
	INIT_DELAYED_WORK(&hdmi->cec_work, histb_hdmi_cec_work);
#endif
	hdmi->last_reported_status = -1;
	hdmi->hpd_polling = device_property_read_bool(dev, "hisilicon,hpd-poll");
	hdmi->tx_irq = platform_get_irq_byname_optional(pdev, "tx");
	if (hdmi->tx_irq == -EPROBE_DEFER)
		return hdmi->tx_irq;
	if (hdmi->tx_irq < 0 && hdmi->tx_irq != -ENXIO)
		return dev_err_probe(dev, hdmi->tx_irq,
				     "failed to get optional HDMI TX IRQ\n");

	hdmi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hdmi->base))
		return PTR_ERR(hdmi->base);

	for (i = 0; i < ARRAY_SIZE(hdmi->clks); i++)
		hdmi->clks[i].id = histb_hdmi_clk_names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(hdmi->clks), hdmi->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get HDMI clocks\n");

	for (i = 0; i < ARRAY_SIZE(hdmi->resets); i++)
		hdmi->resets[i].id = histb_hdmi_reset_names[i];
	ret = devm_reset_control_bulk_get_exclusive(dev,
						    ARRAY_SIZE(hdmi->resets),
						    hdmi->resets);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get HDMI resets\n");

	hdmi->perictrl = syscon_regmap_lookup_by_phandle_args(dev->of_node,
							      "hisilicon,perictrl",
							      1,
							      &hdmi->perictrl_offset);
	if (IS_ERR(hdmi->perictrl))
		return dev_err_probe(dev, PTR_ERR(hdmi->perictrl),
				     "failed to get peripheral control register\n");
	ret = histb_hdmi_hdcp_init(pdev, hdmi);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize HDCP resources\n");

	ret = clk_bulk_prepare_enable(HISTB_HDMI_AUX_CLK_COUNT, hdmi->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable HDMI clocks\n");
	hdmi->aux_enabled = true;

	/* Warm boots can leave the transmitter in the firmware's last mode. */
	ret = reset_control_bulk_assert(ARRAY_SIZE(hdmi->resets),
					hdmi->resets);
	if (ret) {
		clk_bulk_disable_unprepare(HISTB_HDMI_AUX_CLK_COUNT,
					   hdmi->clks);
		hdmi->aux_enabled = false;
		return dev_err_probe(dev, ret, "failed to assert HDMI resets\n");
	}
	usleep_range(1000, 2000);

	ret = reset_control_bulk_deassert(ARRAY_SIZE(hdmi->resets),
					hdmi->resets);
	if (ret) {
		histb_hdmi_disable_resources(hdmi);
		return dev_err_probe(dev, ret, "failed to deassert HDMI resets\n");
	}
	usleep_range(1000, 2000);

	ret = histb_hdmi_vendor_reset_sequence(hdmi);
	if (ret) {
		histb_hdmi_disable_resources(hdmi);
		return dev_err_probe(dev, ret,
				     "failed to run HDMI vendor reset sequence\n");
	}

	ret = histb_hdmi_hw_init(hdmi);
	if (ret) {
		histb_hdmi_disable_resources(hdmi);
		return dev_err_probe(dev, ret,
				     "failed to initialize HDMI routing\n");
	}
	if (hdmi->tx_irq >= 0 && !hdmi->hpd_polling) {
		ret = devm_request_irq(dev, hdmi->tx_irq, histb_hdmi_irq,
				       IRQF_SHARED, dev_name(dev), hdmi);
		if (ret) {
			histb_hdmi_disable_resources(hdmi);
			return dev_err_probe(dev, ret,
					     "failed to request HDMI TX IRQ\n");
		}
		dev_dbg(dev, "HDMI TX HPD IRQ %d enabled from R002 INTR1\n",
			 hdmi->tx_irq);
	} else if (hdmi->hpd_polling) {
		dev_dbg(dev, "HDMI HPD uses R002 150ms/10ms register polling\n");
	}
	hdmi->ddc.owner = THIS_MODULE;
	hdmi->ddc.algo = &histb_hdmi_ddc_algorithm;
	hdmi->ddc.dev.parent = dev;
	strscpy(hdmi->ddc.name, "HiSTB HDMI SCDC", sizeof(hdmi->ddc.name));
	i2c_set_adapdata(&hdmi->ddc, hdmi);
	ret = devm_i2c_add_adapter(dev, &hdmi->ddc);
	if (ret) {
		histb_hdmi_disable_resources(hdmi);
		return dev_err_probe(dev, ret,
				     "failed to register HDMI DDC adapter\n");
	}

	hdmi->bridge.funcs = &histb_hdmi_bridge_funcs;
	hdmi->bridge.of_node = dev->of_node;
	hdmi->bridge.ddc = &hdmi->ddc;
	hdmi->bridge.type = DRM_MODE_CONNECTOR_HDMIA;
	hdmi->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID |
			   DRM_BRIDGE_OP_HDMI;
	if (hdmi->tx_irq >= 0 || hdmi->hpd_polling)
		hdmi->bridge.ops |= DRM_BRIDGE_OP_HPD;
	hdmi->bridge.interlace_allowed = false;
	hdmi->bridge.vendor = "HISI";
	hdmi->bridge.product = "Hi3798CV200 HDMI";
	hdmi->bridge.supported_formats = BIT(HDMI_COLORSPACE_RGB) |
		BIT(HDMI_COLORSPACE_YUV444) |
		BIT(HDMI_COLORSPACE_YUV422) |
		BIT(HDMI_COLORSPACE_YUV420);
	hdmi->bridge.color_format_property = true;
	hdmi->bridge.max_bpc = 12;


	ret = histb_audio_register(dev, &hdmi->audio);
	if (ret) {
		histb_hdmi_disable_resources(hdmi);
		return dev_err_probe(dev, ret, "failed to register HDMI codec\n");
	}
#if IS_ENABLED(CONFIG_CEC_CORE)
	/* The auxiliary domain keeps CEC usable while a sleeping TV drops HPD. */
	hdmi->cec_adap = cec_allocate_adapter(&histb_hdmi_cec_ops, hdmi,
					      dev_name(dev),
					      CEC_CAP_DEFAULTS | CEC_CAP_CONNECTOR_INFO,
					      1);
	if (IS_ERR(hdmi->cec_adap)) {
		ret = PTR_ERR(hdmi->cec_adap);
		hdmi->cec_adap = NULL;
		dev_warn(dev, "CEC adapter allocation failed: %d\n", ret);
	} else {
		ret = cec_register_adapter(hdmi->cec_adap, dev);
		if (ret) {
			cec_delete_adapter(hdmi->cec_adap);
			hdmi->cec_adap = NULL;
			dev_warn(dev, "CEC adapter registration failed: %d\n", ret);
		} else {
			dev_info(dev, "registered HDMI-CEC adapter\n");
			ret = devm_add_action_or_reset(dev, histb_hdmi_cec_release, hdmi);
			if (ret) {
				histb_hdmi_disable_resources(hdmi);
				return ret;
			}
		}
	}
#endif
	ret = devm_drm_bridge_add(dev, &hdmi->bridge);
	if (ret) {
		histb_hdmi_disable_resources(hdmi);
		return ret;
	}
	platform_set_drvdata(pdev, hdmi);
	dev_info(dev, "registered HDMI bridge with HPD, DDC and TMDS output\n");

	return 0;
}

static void histb_hdmi_remove(struct platform_device *pdev)
{
	struct histb_hdmi *hdmi = platform_get_drvdata(pdev);

	mutex_lock(&hdmi->audio_lock);
	hdmi->removing = true;
	mutex_unlock(&hdmi->audio_lock);
	histb_hdmi_bridge_detach(&hdmi->bridge);
	histb_audio_unregister(&hdmi->audio);
#if IS_ENABLED(CONFIG_CEC_CORE)
	if (hdmi->cec_adap)
		devm_release_action(&pdev->dev, histb_hdmi_cec_release, hdmi);
#endif
	histb_hdmi_disable_resources(hdmi);
}

static const struct of_device_id histb_hdmi_of_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-hdmi" },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_hdmi_of_match);

static struct platform_driver histb_hdmi_driver = {
	.probe = histb_hdmi_probe,
	.remove = histb_hdmi_remove,
	.driver = {
		.name = "histb-hdmi",
		.of_match_table = histb_hdmi_of_match,
		.pm = &histb_hdmi_pm_ops,
	},
};
module_platform_driver(histb_hdmi_driver);

MODULE_DESCRIPTION("HiSilicon Hi3798CV200 HDMI transmitter bridge");
MODULE_LICENSE("GPL");
