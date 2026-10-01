// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon HiSTB HDMI transmitter bridge
 *
 * The transmitter contains a private DDC controller.  Keep its register
 * protocol local to this driver and expose sink discovery through the DRM
 * bridge helpers.
 */

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
#include <linux/workqueue.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_modes.h>
#include <drm/drm_print.h>
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

#define HISTB_HDMI_ACR_CTRL		HISTB_HDMI_REG(0xa, 0x01)
#define HISTB_HDMI_AUD_MODE		HISTB_HDMI_REG(0xa, 0x14)
#define HISTB_HDMI_I2S_IN_CTRL		HISTB_HDMI_REG(0xa, 0x1d)
#define HISTB_HDMI_I2S_CHST3		HISTB_HDMI_REG(0xa, 0x21)
#define HISTB_HDMI_I2S_CHST4		HISTB_HDMI_REG(0xa, 0x22)
#define HISTB_HDMI_I2S_IN_SIZE		HISTB_HDMI_REG(0xa, 0x24)
#define HISTB_HDMI_AIP_HDMI2MHL		HISTB_HDMI_REG(0xa, 0x2d)
#define HISTB_HDMI_AUDP_TXCTRL		HISTB_HDMI_REG(0xa, 0x2f)
#define HISTB_HDMI_TPI_DOWN_SMPL_CTRL	HISTB_HDMI_REG(0xa, 0x61)
#define HISTB_HDMI_TPI_AUD_CONFIG	HISTB_HDMI_REG(0xa, 0x62)
#define HISTB_HDMI_TPI_AUD_FS		HISTB_HDMI_REG(0xa, 0x63)
#define HISTB_HDMI_CEC_CONFIG_CPI	HISTB_HDMI_REG(0xf, 0x8e)
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
#define HISTB_HDMI_VP_CSC_OUT_BT601	BIT(2)
#define HISTB_HDMI_VP_CSC_OUT_PC	BIT(4)
#define HISTB_HDMI_VP_CSC_OUT_RGB	BIT(5)
#define HISTB_HDMI_VP_CSC_IN_BT601	BIT(6)
#define HISTB_HDMI_VP_CSC_IN_PC		BIT(8)
#define HISTB_HDMI_VP_CSC_IN_RGB		BIT(9)
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
	struct i2c_adapter ddc;
	struct drm_bridge bridge;
	struct drm_connector *active_connector;
	struct delayed_work hpd_work;
	int tx_irq;
	int last_reported_status;
	bool hpd_enabled;
	bool hpd_polling;
	bool aux_enabled;
	bool pixel_enabled;
	bool mode_ready;
	bool high_tmds;
	bool scdc_enabled;
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
					    unsigned int bpc)
{
	u16 output_mapping = HISTB_HDMI_VP_OUTPUT_MAP_444;
	u16 csc_config = HISTB_HDMI_VP_CSC_ENABLE |
			 HISTB_HDMI_VP_CSC_IN_PC | HISTB_HDMI_VP_CSC_IN_RGB;
	u8 dither = bpc == 8 ? HISTB_HDMI_VP_DITHER_10_TO_8 :
				     HISTB_HDMI_VP_DITHER_NO_CHANGE;

	histb_hdmi_reset_video_path(hdmi);

	/* VDP supplies full-range RGB; match the YUV matrix to the AVI. */
	if (histb_hdmi_colorimetry(mode) == HDMI_COLORIMETRY_ITU_601)
		csc_config |= HISTB_HDMI_VP_CSC_IN_BT601 |
			      HISTB_HDMI_VP_CSC_OUT_BT601;
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
	histb_hdmi_updateb(hdmi, HISTB_HDMI_INFOFRAME_CTRL,
			   HISTB_HDMI_INFOFRAME_ENABLE |
			   HISTB_HDMI_INFOFRAME_REPEAT,
			   enable ? HISTB_HDMI_INFOFRAME_ENABLE |
			   HISTB_HDMI_INFOFRAME_REPEAT : 0);
}

static void histb_hdmi_infoframe_wait(struct histb_hdmi *hdmi, bool active)
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
		selector = 9;
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
	static const u8 selectors[] = { 0, 2, 3, 5, 9 };
	unsigned int i;

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
	else if (tmds_rate < 297000000)
		config = &histb_hdmi_phy_configs[2];
	else if (tmds_rate < 340000000)
		config = &histb_hdmi_phy_configs[3];
	else
		config = &histb_hdmi_phy_configs[4];

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

	histb_hdmi_writeb(hdmi, HISTB_HDMI_I2S_CHST3, 0x02);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_I2S_CHST4, 0x02);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_ACR_CTRL, 0x0c);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_I2S_IN_SIZE, 0x0b);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_I2S_IN_CTRL, 0x60);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_AUD_MODE, BIT(4), BIT(4));
	histb_hdmi_writeb(hdmi, HISTB_HDMI_AIP_HDMI2MHL, 0x00);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_AUDP_TXCTRL, BIT(1), 0x00);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_TPI_DOWN_SMPL_CTRL,
			   BIT(2), 0x00);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_TPI_AUD_CONFIG, 0x00);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_TPI_AUD_FS, 0x82);
	histb_hdmi_updateb(hdmi, HISTB_HDMI_AUDP_TXCTRL, BIT(7), BIT(7));

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
	histb_hdmi_writeb(hdmi, HISTB_HDMI_INTR1_MASK, 0);
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

	return 0;
}

static void histb_hdmi_configure_mode(struct histb_hdmi *hdmi,
				      const struct drm_display_mode *mode,
				      u64 tmds_rate, bool is_hdmi,
				      enum hdmi_colorspace format,
				      unsigned int bpc)
{
	u8 sync = 0;

	histb_hdmi_tmds_enable(hdmi, false);
	histb_hdmi_phy_configure(hdmi, tmds_rate);

	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		sync |= HISTB_HDMI_VP_SYNC_VPOS;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		sync |= HISTB_HDMI_VP_SYNC_HPOS;
	histb_hdmi_pwd_srst(hdmi);
	histb_hdmi_configure_output_depth(hdmi, bpc);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_OUTPUT_SYNC, sync);

	/* The full-range RGB input pin order does not depend on the sink format. */
	histb_hdmi_writeb(hdmi, HISTB_HDMI_VP_INPUT_SYNC_ADJ,
			  HISTB_HDMI_VP_AUTO_ADJ_DISABLE);
	histb_hdmi_writew(hdmi, HISTB_HDMI_VP_INPUT_MAPPING,
			  HISTB_HDMI_VP_INPUT_MAP_RGB);
	histb_hdmi_configure_video_path(hdmi, mode, format, bpc);
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
	dev_dbg(hdmi->dev,
		 "CV200 HDMI mode %ux%u@%u %s %u bpc tmds=%llu sync=%02x in=%04x in_map=%04x out=%04x out_map=%04x ratio=%02x\n",
		 mode->hdisplay, mode->vdisplay, drm_mode_vrefresh(mode),
		 drm_hdmi_connector_get_output_format_name(format), bpc,
		 tmds_rate, histb_hdmi_readb(hdmi, HISTB_HDMI_VP_OUTPUT_SYNC),
		 histb_hdmi_readw(hdmi, HISTB_HDMI_VP_INPUT_FORMAT),
		 histb_hdmi_readw(hdmi, HISTB_HDMI_VP_INPUT_MAPPING),
		 histb_hdmi_readw(hdmi, HISTB_HDMI_VP_OUTPUT_FORMAT),
		 histb_hdmi_readw(hdmi, HISTB_HDMI_VP_OUTPUT_MAPPING),
		 histb_hdmi_readb(hdmi, HISTB_HDMI_CLK_RATIO));
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

	if (num == 1 && msgs[0].addr == HISTB_HDMI_SCDC_I2C_ADDR &&
	    !msgs[0].flags && msgs[0].len >= 2) {
		ret = histb_hdmi_scdc_transfer(hdmi, msgs[0].buf[0],
					       msgs[0].buf + 1,
					       msgs[0].len - 1, false);
		return ret ? ret : num;
	}

	if (num == 2 && msgs[0].addr == HISTB_HDMI_SCDC_I2C_ADDR &&
	    msgs[1].addr == HISTB_HDMI_SCDC_I2C_ADDR &&
	    !msgs[0].flags && msgs[0].len == 1 &&
	    msgs[1].flags == I2C_M_RD && msgs[1].len) {
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
	u8 hpd_rsen = histb_hdmi_readb(hdmi, HISTB_HDMI_TPI_HPD_RSEN);

	return hpd_rsen & HISTB_HDMI_HPD_CONNECTED ?
		       connector_status_connected : connector_status_disconnected;
}

static void histb_hdmi_hpd_work(struct work_struct *work)
{
	struct histb_hdmi *hdmi = container_of(to_delayed_work(work),
					      struct histb_hdmi, hpd_work);
	enum drm_connector_status status;
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
	if (READ_ONCE(hdmi->last_reported_status) != status) {
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
	histb_hdmi_writeb(hdmi, HISTB_HDMI_INTR1_MASK, 0);
	cancel_delayed_work(&hdmi->hpd_work);
}

static const struct drm_edid *
histb_hdmi_bridge_edid_read(struct drm_bridge *bridge,
			    struct drm_connector *connector)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	const struct drm_edid *edid;

	guard(mutex)(&hdmi->ddc_lock);
	edid = drm_edid_read_custom(connector, histb_hdmi_read_edid_block, hdmi);

	return edid;
}

static enum drm_mode_status
histb_hdmi_bridge_mode_valid(struct drm_bridge *bridge,
			     const struct drm_display_info *info,
			     const struct drm_display_mode *mode)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	unsigned long rate = mode->clock * 1000UL;
	unsigned long rate_delta;
	long rounded_rate;

	if (mode->flags & (DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLSCAN))
		return MODE_NO_INTERLACE;
	if (mode->clock < 25000)
		return MODE_CLOCK_LOW;
	if (mode->clock > 594000)
		return MODE_CLOCK_HIGH;
	if (mode->clock > 340000 &&
	    (!info || !info->is_hdmi || !info->hdmi.scdc.supported ||
	     !info->hdmi.scdc.scrambling.supported))
		return MODE_CLOCK_HIGH;

	rounded_rate = clk_round_rate(hdmi->clks[HISTB_HDMI_PIXEL_CLK_INDEX].clk,
				      rate);
	if (rounded_rate < 0)
		return MODE_CLOCK_RANGE;

	/* Production VESA PLL plans differ from their nominal clocks by <= 0.52%. */
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

	ret = histb_hdmi_infoframe_select(hdmi, type);
	if (ret)
		return ret;

	histb_hdmi_infoframe_enable(hdmi, false);
	histb_hdmi_infoframe_wait(hdmi, false);

	return 0;
}

static int histb_hdmi_write_infoframe(struct drm_bridge *bridge,
				      enum hdmi_infoframe_type type,
				      const u8 *buffer, size_t len)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	unsigned int i;
	int ret;

	if (!buffer || !len || len > HISTB_HDMI_INFOFRAME_BYTES)
		return -EINVAL;

	ret = histb_hdmi_infoframe_select(hdmi, type);
	if (ret)
		return ret;

	histb_hdmi_infoframe_enable(hdmi, false);
	histb_hdmi_infoframe_wait(hdmi, false);
	for (i = 0; i < HISTB_HDMI_INFOFRAME_BYTES; i++)
		histb_hdmi_writeb(hdmi, HISTB_HDMI_INFOFRAME_DATA +
				  i * sizeof(u32), i < len ? buffer[i] : 0);
	histb_hdmi_infoframe_enable(hdmi, true);
	histb_hdmi_infoframe_wait(hdmi, true);

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
	else
		usleep_range(1000, 2000);
}

static int histb_hdmi_bridge_atomic_check(struct drm_bridge *bridge,
					  struct drm_bridge_state *bridge_state,
					  struct drm_crtc_state *crtc_state,
					  struct drm_connector_state *conn_state)
{
	struct hdmi_avi_infoframe *avi;
	int ret;

	ret = drm_atomic_helper_connector_hdmi_check(conn_state->connector,
						     bridge_state->base.state);
	if (ret)
		return ret;

	avi = &conn_state->hdmi.infoframes.avi.data.avi;
	if (conn_state->hdmi.output_format == HDMI_COLORSPACE_RGB) {
		if (conn_state->hdmi.broadcast_rgb ==
		    DRM_HDMI_BROADCAST_RGB_LIMITED)
			return -EINVAL;

		conn_state->hdmi.is_limited_range = false;
		avi->quantization_range = HDMI_QUANTIZATION_RANGE_FULL;
	} else {
		conn_state->hdmi.is_limited_range = true;
		avi->colorimetry = histb_hdmi_colorimetry(&crtc_state->adjusted_mode);
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
				  conn_state->hdmi.output_bpc);
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
		}
	}
}

static void
histb_hdmi_bridge_atomic_disable(struct drm_bridge *bridge,
				 struct drm_bridge_state *old_bridge_state)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);

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

	if (hdmi->pixel_enabled) {
		clk_disable_unprepare(pixel_clk);
		hdmi->pixel_enabled = false;
	}
	hdmi->active_connector = NULL;
	hdmi->high_tmds = false;
}

static const struct drm_bridge_funcs histb_hdmi_bridge_funcs = {
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
		reset_control_bulk_assert(ARRAY_SIZE(hdmi->resets),
					  hdmi->resets);
		clk_bulk_disable_unprepare(HISTB_HDMI_AUX_CLK_COUNT,
					   hdmi->clks);
		hdmi->aux_enabled = false;
	}
}

static int __maybe_unused histb_hdmi_suspend(struct device *dev)
{
	struct histb_hdmi *hdmi = dev_get_drvdata(dev);
	struct clk *pixel_clk;
	int ret = 0;

	WRITE_ONCE(hdmi->hpd_enabled, false);
	if (hdmi->aux_enabled)
		histb_hdmi_writeb(hdmi, HISTB_HDMI_INTR1_MASK, 0);
	if (hdmi->tx_irq >= 0)
		synchronize_irq(hdmi->tx_irq);
	cancel_delayed_work_sync(&hdmi->hpd_work);

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
	clk_bulk_disable_unprepare(HISTB_HDMI_AUX_CLK_COUNT, hdmi->clks);
	hdmi->aux_enabled = false;
	hdmi->active_connector = NULL;

	return ret;
}

static int __maybe_unused histb_hdmi_resume(struct device *dev)
{
	struct histb_hdmi *hdmi = dev_get_drvdata(dev);
	int ret;

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
	INIT_DELAYED_WORK(&hdmi->hpd_work, histb_hdmi_hpd_work);
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
	mutex_init(&hdmi->ddc_lock);
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

	WRITE_ONCE(hdmi->hpd_enabled, false);
	histb_hdmi_writeb(hdmi, HISTB_HDMI_INTR1_MASK, 0);
	if (hdmi->tx_irq >= 0)
		synchronize_irq(hdmi->tx_irq);
	cancel_delayed_work_sync(&hdmi->hpd_work);
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

MODULE_DESCRIPTION("HiSilicon HiSTB HDMI transmitter bridge");
MODULE_LICENSE("GPL");
