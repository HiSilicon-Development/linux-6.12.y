// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hi3798MV310 hisiv100 HDMI controller and external HDMI PHY.
 *
 * Unlike CV200 (imagev200), this controller uses native 32-bit registers
 * and an external PHY. Register fields and sequences follow SPC060
 * hdmi_2_0/hal/ctrl/hisiv100, hal/phy/hisiv100 and product/hisiv100.
 * Progressive SDR RGB, 8 bpc, including HDMI 2.0 SCDC/scrambling.
 * Deep colour, HDCP and CEC are not advertised. PCM uses the native AIP.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hdmi.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/workqueue.h>
#include "histb-hdmi-audio.h"

#include <drm/display/drm_hdmi_state_helper.h>
#include <drm/display/drm_scdc.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_modes.h>
#include <drm/drm_print.h>

#define MV310_PWD_RESET		0x0010
#define MV310_SCDC_CTRL		0x0014
#define MV310_DDC_RDATA		0x0038
#define MV310_DDC_WDATA		0x003c
#define MV310_DDC_COUNT		0x0040
#define MV310_DDC_SLAVE		0x0044
#define MV310_DDC_STATE		0x0048
#define MV310_DDC_CMD		0x004c
#define MV310_DDC_REQUEST	0x0068
#define MV310_DDC_ACK		0x006c
#define MV310_PWD_INTR_MASK	0x0108
#define MV310_TIMING_GEN		0x0800
#define MV310_VIDEO_BLANK	0x0830
#define MV310_DITHER		0x08ec
#define MV310_CSC		0x0934
#define MV310_DOWNSAMPLE		0x0980
#define MV310_DATA_ALIGN		0x098c
#define MV310_VIDEO_DMUX		0x09a8
#define MV310_AUDIO_CTRL		0x1000
#define MV310_AUDIO_I2S		0x1004
#define MV310_AUDIO_CHST0	0x100c
#define MV310_AUDIO_CHST1	0x1010
#define MV310_AUDIO_FIFO		0x1018
#define MV310_AUDIO_ACR		0x1040
#define MV310_AUDIO_N		0x1048
#define MV310_AUDIO_HEADER	0x183c
#define MV310_PACK_FIFO		0x1800
#define MV310_PACK_FIFO_ST	0x1804
#define MV310_AVI_HEADER		0x1818
#define MV310_SPD_HEADER		0x1860
#define MV310_VENDOR_HEADER	0x1980
#define MV310_AVI_CFG		0x19a4
#define MV310_SPD_CFG		0x19a8
#define MV310_AUDIO_CFG		0x19ac
#define MV310_CP_CFG		0x19b8
#define MV310_VENDOR_CFG		0x19d0
#define MV310_AVMIXER		0x1a08
#define MV310_AVMUTE		0x1a0c
#define MV310_ENCODER		0x1a60
#define MV310_HPD_CFG		0x4020
#define MV310_HPD_FILTER		0x4024
#define MV310_AON_STATE		0x4028
#define MV310_AON_MASK		0x4030
#define MV310_AON_INTR		0x4034
#define MV310_DDC_CTRL		0x4040
#define MV310_DDC_AON_STATE	0x4054
#define MV310_DDC_MANUAL		0x405c

#define MV310_DDC_ERRORS		GENMASK(1, 0)
#define MV310_DDC_BUSY		BIT(2)
#define MV310_HPD		BIT(0)
#define MV310_DDC_TIMEOUT_US	100000
#define MV310_MAX_TMDS		594000000ULL

#define MV310_PHY_INSSC		0x00
#define MV310_PHY_RESET		0x10
#define MV310_PHY_CLK_HIGH	0x50
#define MV310_PHY_CLK_LOW	0x54
#define MV310_PHY_STATUS		0x98
#define MV310_PHY_POWER		0x64
#define MV310_PHY_MAIN		0x68
#define MV310_PHY_PRE		0x6c
#define MV310_PHY_CAP		0x74
#define MV310_PHY_TERM		0x78
#define MV310_PHY_TEST		0x80
#define MV310_PHY_PLL		0x8c
#define MV310_PHY_PLL_TEST	0x90
#define MV310_PHY_FBSSC		0xb0
#define MV310_PHY_FBSTEP		0xb8
#define MV310_PHY_OUTPUT_PD	GENMASK(3, 0)
#define MV310_PHY_DE_PD		GENMASK(8, 5)
#define MV310_PHY_SUPPLY_PD	(GENMASK(10, 9) | BIT(19) | BIT(20))

#define MV310_SYS_MUX		0x44
#define MV310_SYS_MUX_MASK	GENMASK(24, 21)
#define MV310_SYS_CTRL		0x48
#define MV310_SYS_GATES		(BIT(19) | BIT(21) | BIT(22))
#define MV310_SYS_RESETS		(BIT(20) | BIT(23) | BIT(24))
#define MV310_PIXEL		6
#define MV310_PERI_AUDIO_MUX	0x8b0
#define MV310_PERI_AUDIO_MASK	GENMASK(4, 0)

static const char * const mv310_clock_names[] = {
	"ssc", "ssc-bypass", "osc", "os", "as", "tmds", "pixel",
};
static const char * const mv310_reset_names[] = {
	"bus", "core", "ssc", "phy", "tmds",
};

struct mv310_hdmi {
	struct device *dev;
	void __iomem *core;
	void __iomem *phy;
	struct regmap *sysctrl;
	struct regmap *perictrl;
	struct clk_bulk_data clocks[ARRAY_SIZE(mv310_clock_names)];
	struct reset_control_bulk_data resets[ARRAY_SIZE(mv310_reset_names)];
	struct drm_bridge bridge;
	/* Serializes DDC with suspend/remove and transmitter reinitialization. */
	struct mutex lock;
	struct histb_hdmi_audio audio;
	struct delayed_work hpd_work;
	u32 saved_mux;
	u32 saved_sysctrl;
	u32 saved_audio_mux;
	int irq;
	int last_status;
	bool powered;
	bool pixel_enabled;
	bool mode_ready;
	bool hpd_enabled;
};

static struct mv310_hdmi *to_mv310(struct drm_bridge *bridge)
{
	return container_of(bridge, struct mv310_hdmi, bridge);
}

static void mv310_update(void __iomem *base, u32 reg, u32 mask, u32 value)
{
	writel((readl(base + reg) & ~mask) | (value & mask), base + reg);
}

static bool mv310_connected(struct mv310_hdmi *hdmi)
{
	return readl(hdmi->core + MV310_AON_STATE) & MV310_HPD;
}

static void mv310_audio_stop(void *data)
{
	struct mv310_hdmi *hdmi = data;

	if (!hdmi->powered)
		return;
	writel(BIT(1), hdmi->core + MV310_AUDIO_CTRL);
	writel(0, hdmi->core + MV310_AUDIO_CFG);
}

static void mv310_phy_off(struct mv310_hdmi *hdmi)
{
	mv310_update(hdmi->phy, MV310_PHY_POWER,
		     MV310_PHY_OUTPUT_PD | MV310_PHY_DE_PD | MV310_PHY_SUPPLY_PD,
		     MV310_PHY_OUTPUT_PD | MV310_PHY_DE_PD | MV310_PHY_SUPPLY_PD);
	histb_audio_disable(&hdmi->audio);
	hdmi->mode_ready = false;
}

static void mv310_packets_off(struct mv310_hdmi *hdmi)
{
	u32 reg;

	/* Disable all firmware packet slots, including audio and generic HDR. */
	for (reg = MV310_AVI_CFG; reg <= MV310_VENDOR_CFG; reg += 4)
		writel(0, hdmi->core + reg);
	writel(0, hdmi->core + MV310_AVMIXER);
	writel(0, hdmi->core + MV310_AUDIO_CTRL);
}

static void mv310_hw_init(struct mv310_hdmi *hdmi)
{
	writel(0, hdmi->core + MV310_AON_MASK);
	writel(0, hdmi->core + MV310_PWD_INTR_MASK);
	writel(0x3ff3, hdmi->core + MV310_AON_INTR);
	/* Physical HPD only: disable override, inversion and software HPD. */
	writel(0x7cf1, hdmi->core + MV310_HPD_CFG);
	writel(0x00090063, hdmi->core + MV310_HPD_FILTER);
	/* PWD DDC master, automatic bus mode, reset-default 100 kHz divider. */
	writel(0x4a0, hdmi->core + MV310_DDC_CTRL);
	writel(0, hdmi->core + MV310_SCDC_CTRL);
	writel(BIT(0), hdmi->core + MV310_PWD_RESET);
	udelay(5);
	writel(0, hdmi->core + MV310_PWD_RESET);
	mv310_packets_off(hdmi);
	mv310_phy_off(hdmi);
}

static int mv310_power_on(struct mv310_hdmi *hdmi)
{
	int ret, cleanup_ret;
	/* hisiv100 hdmi_platform.h:55-60: I2S is bit1, not the CV200 AIP. */
	ret = regmap_update_bits(hdmi->perictrl, MV310_PERI_AUDIO_MUX,
				 MV310_PERI_AUDIO_MASK, BIT(1));
	if (ret)
		return ret;

	ret = regmap_update_bits(hdmi->sysctrl, MV310_SYS_MUX,
				 MV310_SYS_MUX_MASK, 0);
	if (ret)
		goto restore_sysctrl;
	ret = regmap_update_bits(hdmi->sysctrl, MV310_SYS_CTRL,
				 MV310_SYS_GATES | MV310_SYS_RESETS,
				 MV310_SYS_GATES | MV310_SYS_RESETS);
	if (ret)
		goto restore_sysctrl;
	ret = clk_bulk_prepare_enable(MV310_PIXEL, hdmi->clocks);
	if (ret)
		goto restore_sysctrl;
	ret = reset_control_bulk_assert(ARRAY_SIZE(hdmi->resets), hdmi->resets);
	if (ret)
		goto disable_clocks;
	usleep_range(1000, 2000);
	ret = regmap_update_bits(hdmi->sysctrl, MV310_SYS_CTRL,
				 MV310_SYS_RESETS, 0);
	if (ret)
		goto disable_clocks;
	ret = reset_control_bulk_deassert(ARRAY_SIZE(hdmi->resets), hdmi->resets);
	if (ret)
		goto assert_resets;
	usleep_range(1000, 2000);
	mv310_hw_init(hdmi);
	WRITE_ONCE(hdmi->powered, true);
	return 0;

assert_resets:
	cleanup_ret = reset_control_bulk_assert(ARRAY_SIZE(hdmi->resets), hdmi->resets);
	if (cleanup_ret)
		dev_warn(hdmi->dev, "HDMI reset unwind failed: %d\n", cleanup_ret);
disable_clocks:
	clk_bulk_disable_unprepare(MV310_PIXEL, hdmi->clocks);
restore_sysctrl:
	cleanup_ret = regmap_update_bits(hdmi->sysctrl, MV310_SYS_CTRL,
					 MV310_SYS_GATES | MV310_SYS_RESETS,
					 hdmi->saved_sysctrl);
	if (cleanup_ret)
		dev_warn(hdmi->dev, "HDMI sysctrl unwind failed: %d\n", cleanup_ret);
	cleanup_ret = regmap_update_bits(hdmi->sysctrl, MV310_SYS_MUX,
					 MV310_SYS_MUX_MASK, hdmi->saved_mux);
	if (cleanup_ret)
		dev_warn(hdmi->dev, "HDMI pinmux unwind failed: %d\n", cleanup_ret);
	cleanup_ret = regmap_update_bits(hdmi->perictrl, MV310_PERI_AUDIO_MUX,
					 MV310_PERI_AUDIO_MASK, hdmi->saved_audio_mux);
	if (cleanup_ret)
		dev_warn(hdmi->dev, "HDMI audio mux unwind failed: %d\n", cleanup_ret);
	return ret;
}

static void mv310_power_off(struct mv310_hdmi *hdmi)
{
	int ret;

	if (!hdmi->powered)
		return;
	writel(0, hdmi->core + MV310_AON_MASK);
	WRITE_ONCE(hdmi->powered, false);
	if (hdmi->irq >= 0)
		synchronize_irq(hdmi->irq);
	mv310_phy_off(hdmi);
	mv310_packets_off(hdmi);
	if (hdmi->pixel_enabled) {
		clk_disable_unprepare(hdmi->clocks[MV310_PIXEL].clk);
		hdmi->pixel_enabled = false;
	}
	ret = reset_control_bulk_assert(ARRAY_SIZE(hdmi->resets), hdmi->resets);
	if (ret)
		dev_warn(hdmi->dev, "HDMI reset assertion failed: %d\n", ret);
	ret = regmap_update_bits(hdmi->sysctrl, MV310_SYS_CTRL,
				 MV310_SYS_GATES | MV310_SYS_RESETS,
				 MV310_SYS_RESETS);
	if (ret)
		dev_warn(hdmi->dev, "HDMI sysctrl reset failed: %d\n", ret);
	clk_bulk_disable_unprepare(MV310_PIXEL, hdmi->clocks);
}

static enum drm_connector_status mv310_detect(struct drm_bridge *bridge)
{
	struct mv310_hdmi *hdmi = to_mv310(bridge);
	enum drm_connector_status status;

	mutex_lock(&hdmi->lock);
	status = !hdmi->powered ? connector_status_unknown :
		 mv310_connected(hdmi) ? connector_status_connected :
		 connector_status_disconnected;
	mutex_unlock(&hdmi->lock);
	return status;
}

static void mv310_hpd_work(struct work_struct *work)
{
	struct mv310_hdmi *hdmi = container_of(to_delayed_work(work),
					     struct mv310_hdmi, hpd_work);
	enum drm_connector_status status;

	if (!READ_ONCE(hdmi->hpd_enabled) || !READ_ONCE(hdmi->powered))
		return;
	status = mv310_detect(&hdmi->bridge);
	if (status != connector_status_connected) {
		mutex_lock(&hdmi->lock);
		histb_audio_disable(&hdmi->audio);
		mutex_unlock(&hdmi->lock);
	}
	if (status != hdmi->last_status) {
		hdmi->last_status = status;
		drm_bridge_hpd_notify(&hdmi->bridge, status);
	}
	if (hdmi->irq < 0 && READ_ONCE(hdmi->hpd_enabled) &&
	    READ_ONCE(hdmi->powered))
		schedule_delayed_work(&hdmi->hpd_work, msecs_to_jiffies(100));
}

static irqreturn_t mv310_irq(int irq, void *data)
{
	struct mv310_hdmi *hdmi = data;
	u32 pending;

	if (!READ_ONCE(hdmi->powered))
		return IRQ_NONE;
	pending = readl(hdmi->core + MV310_AON_INTR) &
		  readl(hdmi->core + MV310_AON_MASK) & MV310_HPD;
	if (!pending)
		return IRQ_NONE;
	writel(pending, hdmi->core + MV310_AON_INTR);
	if (READ_ONCE(hdmi->hpd_enabled))
		schedule_delayed_work(&hdmi->hpd_work, 0);
	return IRQ_HANDLED;
}

static void mv310_hpd_enable(struct drm_bridge *bridge)
{
	struct mv310_hdmi *hdmi = to_mv310(bridge);

	mutex_lock(&hdmi->lock);
	WRITE_ONCE(hdmi->hpd_enabled, true);
	hdmi->last_status = -1;
	if (hdmi->powered) {
		writel(MV310_HPD, hdmi->core + MV310_AON_INTR);
		writel(hdmi->irq >= 0 ? MV310_HPD : 0,
		       hdmi->core + MV310_AON_MASK);
		schedule_delayed_work(&hdmi->hpd_work, 0);
	}
	mutex_unlock(&hdmi->lock);
}

static void mv310_hpd_disable(struct drm_bridge *bridge)
{
	struct mv310_hdmi *hdmi = to_mv310(bridge);

	mutex_lock(&hdmi->lock);
	WRITE_ONCE(hdmi->hpd_enabled, false);
	if (hdmi->powered)
		writel(0, hdmi->core + MV310_AON_MASK);
	mutex_unlock(&hdmi->lock);
	if (hdmi->irq >= 0)
		synchronize_irq(hdmi->irq);
	/* DRM holds bridge->hpd_mutex here; a notifier may be waiting on it. */
	cancel_delayed_work(&hdmi->hpd_work);
}

static int mv310_ddc_error(struct mv310_hdmi *hdmi)
{
	u32 status = readl(hdmi->core + MV310_DDC_STATE) |
		     readl(hdmi->core + MV310_DDC_AON_STATE);

	if (!mv310_connected(hdmi))
		return -ENODEV;
	if (status & BIT(0))
		return -ENXIO;
	if (status & BIT(1))
		return -EIO;
	return 0;
}

/* Called under lock; EDID reads and SCDC byte transfers share arbitration. */
static int mv310_ddc_transfer(struct mv310_hdmi *hdmi, u8 slave, u8 offset,
			      u8 segment, u8 *buffer, size_t length, bool write)
{
	u32 status;
	size_t i;
	int ret, release_ret;

	if (!hdmi->powered || !mv310_connected(hdmi))
		return -ENODEV;
	if (!length || length > 128 || (write && length != 1))
		return -EINVAL;
	writel(1, hdmi->core + MV310_DDC_REQUEST);
	ret = readl_poll_timeout(hdmi->core + MV310_DDC_ACK, status,
				 status & BIT(0), 100, MV310_DDC_TIMEOUT_US);
	if (ret)
		goto release;
	ret = readl_poll_timeout(hdmi->core + MV310_DDC_MANUAL, status,
				 (status & 3) == 3, 100, MV310_DDC_TIMEOUT_US);
	if (ret)
		goto abort;
	ret = readl_poll_timeout(hdmi->core + MV310_DDC_STATE, status,
				 !(status & MV310_DDC_BUSY), 100,
				 MV310_DDC_TIMEOUT_US);
	if (ret)
		goto abort;
	writel(9, hdmi->core + MV310_DDC_CMD);
	writel(slave << 1 | offset << 8 | segment << 16,
	       hdmi->core + MV310_DDC_SLAVE);
	writel(length << 8, hdmi->core + MV310_DDC_COUNT);
	usleep_range(1000, 2000);
	writel(write ? 7 : segment ? 4 : 2, hdmi->core + MV310_DDC_CMD);
	if (write) {
		writel(*buffer, hdmi->core + MV310_DDC_WDATA);
		usleep_range(1000, 2000);
	}
	for (i = 0; !write && i < length; i++) {
		ret = readl_poll_timeout(hdmi->core + MV310_DDC_COUNT, status,
					(status & 0x1f) || mv310_ddc_error(hdmi),
					10, 20000);
		if (!ret)
			ret = mv310_ddc_error(hdmi);
		if (ret)
			goto abort;
		buffer[i] = readl(hdmi->core + MV310_DDC_RDATA);
		/* BSP requires >=5 us after reading each FIFO byte. */
		udelay(5);
	}
	ret = readl_poll_timeout(hdmi->core + MV310_DDC_STATE, status,
				 !(status & MV310_DDC_BUSY) || mv310_ddc_error(hdmi),
				 10, MV310_DDC_TIMEOUT_US);
	if (!ret)
		ret = mv310_ddc_error(hdmi);
abort:
	if (ret)
		writel(0xf, hdmi->core + MV310_DDC_CMD);
release:
	writel(0, hdmi->core + MV310_DDC_REQUEST);
	release_ret = readl_poll_timeout(hdmi->core + MV310_DDC_ACK, status,
					!(status & BIT(0)), 100,
					MV310_DDC_TIMEOUT_US);
	if (!ret)
		ret = release_ret;
	if (ret)
		dev_dbg(hdmi->dev, "DDC %02x:%02x failed: %d\n", slave, offset, ret);
	return ret;
}

static int mv310_read_edid_block(void *data, u8 *buffer,
			       unsigned int block, size_t length)
{
	if (block > 511)
		return -EINVAL;
	return mv310_ddc_transfer(data, 0x50, (block & 1) * 128,
				  block / 2, buffer, length, false);
}

static int mv310_scdc_configure(struct mv310_hdmi *hdmi,
			       const struct drm_display_info *info,
			       unsigned long rate)
{
	u8 version, config, readback;
	int ret;

	if (!info->hdmi.scdc.supported)
		return rate > 340000000 ? -EINVAL : 0;
	ret = mv310_ddc_transfer(hdmi, 0x54, SCDC_SINK_VERSION, 0,
				 &version, 1, false);
	if (ret)
		return ret;
	if (!version)
		return -EIO;
	version = 1;
	ret = mv310_ddc_transfer(hdmi, 0x54, SCDC_SOURCE_VERSION, 0,
				 &version, 1, true);
	if (ret)
		return ret;
	config = rate > 340000000 ?
		 SCDC_SCRAMBLING_ENABLE | SCDC_TMDS_BIT_CLOCK_RATIO_BY_40 : 0;
	ret = mv310_ddc_transfer(hdmi, 0x54, SCDC_TMDS_CONFIG, 0,
				 &config, 1, true);
	if (ret)
		return ret;
	/* The sink must have at least 1 ms to change its TMDS clock ratio. */
	usleep_range(1000, 2000);
	ret = mv310_ddc_transfer(hdmi, 0x54, SCDC_TMDS_CONFIG, 0,
				 &readback, 1, false);
	if (ret)
		return ret;
	return (readback & (SCDC_SCRAMBLING_ENABLE |
			    SCDC_TMDS_BIT_CLOCK_RATIO_BY_40)) == config ? 0 : -EIO;
}

static const struct drm_edid *mv310_edid_read(struct drm_bridge *bridge,
					   struct drm_connector *connector)
{
	struct mv310_hdmi *hdmi = to_mv310(bridge);
	const struct drm_edid *edid = NULL;

	mutex_lock(&hdmi->lock);
	if (hdmi->powered && mv310_connected(hdmi))
		edid = drm_edid_read_custom(connector, mv310_read_edid_block, hdmi);
	mutex_unlock(&hdmi->lock);
	return edid;
}

static enum drm_mode_status mv310_mode_valid(struct drm_bridge *bridge,
					   const struct drm_display_info *info,
					   const struct drm_display_mode *mode)
{
	struct mv310_hdmi *hdmi = to_mv310(bridge);
	unsigned long rate = mode->clock * 1000UL;
	long rounded;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;
	if (mode->flags & (DRM_MODE_FLAG_DBLSCAN | DRM_MODE_FLAG_DBLCLK) ||
	    mode->vscan > 1)
		return MODE_BAD;
	if (mode->clock < 25000)
		return MODE_CLOCK_LOW;
	if (rate > MV310_MAX_TMDS)
		return MODE_CLOCK_HIGH;
	if (rate > 340000000 && (!info->is_hdmi || !info->hdmi.scdc.supported ||
				 !info->hdmi.scdc.scrambling.supported))
		return MODE_CLOCK_HIGH;
	rounded = clk_round_rate(hdmi->clocks[MV310_PIXEL].clk, rate);
	if (rounded < 0 || abs_diff((unsigned long)rounded, rate) > rate / 1000)
		return MODE_CLOCK_RANGE;
	return MODE_OK;
}

static enum drm_mode_status mv310_tmds_valid(const struct drm_bridge *bridge,
					   const struct drm_display_mode *mode,
					   unsigned long long rate)
{
	return rate >= 25000000 && rate <= MV310_MAX_TMDS ?
		MODE_OK : MODE_CLOCK_RANGE;
}

static int mv310_atomic_check(struct drm_bridge *bridge,
			      struct drm_bridge_state *bridge_state,
			      struct drm_crtc_state *crtc_state,
			      struct drm_connector_state *conn_state)
{
	int ret;

	if (mv310_mode_valid(bridge, &conn_state->connector->display_info,
			     &crtc_state->adjusted_mode) != MODE_OK)
		return -EINVAL;
	if (conn_state->hdr_output_metadata)
		return -EINVAL;
	ret = drm_atomic_helper_connector_hdmi_check(conn_state->connector,
						     bridge_state->base.state);
	if (ret)
		return ret;
	if (conn_state->hdmi.output_format != HDMI_COLORSPACE_RGB ||
	    conn_state->hdmi.output_bpc != 8 ||
	    conn_state->hdmi.broadcast_rgb == DRM_HDMI_BROADCAST_RGB_LIMITED)
		return -EINVAL;
	conn_state->hdmi.is_limited_range = false;
	conn_state->hdmi.infoframes.avi.data.avi.quantization_range =
		HDMI_QUANTIZATION_RANGE_FULL;
	return 0;
}

static int mv310_packet_registers(enum hdmi_infoframe_type type,
				  u32 *header, u32 *config)
{
	switch (type) {
	case HDMI_INFOFRAME_TYPE_AUDIO:
		*header = MV310_AUDIO_HEADER;
		*config = MV310_AUDIO_CFG;
		return 0;
	case HDMI_INFOFRAME_TYPE_AVI:
		*header = MV310_AVI_HEADER;
		*config = MV310_AVI_CFG;
		return 0;
	case HDMI_INFOFRAME_TYPE_SPD:
		*header = MV310_SPD_HEADER;
		*config = MV310_SPD_CFG;
		return 0;
	case HDMI_INFOFRAME_TYPE_VENDOR:
		*header = MV310_VENDOR_HEADER;
		*config = MV310_VENDOR_CFG;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int mv310_clear_infoframe(struct drm_bridge *bridge,
				 enum hdmi_infoframe_type type)
{
	struct mv310_hdmi *hdmi = to_mv310(bridge);
	u32 header, config;
	int ret = mv310_packet_registers(type, &header, &config);

	if (ret)
		return ret;
	writel(0, hdmi->core + config);
	return 0;
}

static int mv310_write_infoframe(struct drm_bridge *bridge,
				 enum hdmi_infoframe_type type,
				 const u8 *buffer, size_t length)
{
	struct mv310_hdmi *hdmi = to_mv310(bridge);
	u32 header, config, word;
	unsigned int sub, byte, index;
	int ret = mv310_packet_registers(type, &header, &config);

	if (ret)
		return ret;
	if (length < 4 || length > 31)
		return -EINVAL;
	writel(0, hdmi->core + config);
	writel(buffer[0] | buffer[1] << 8 | buffer[2] << 16,
	       hdmi->core + header);
	/* Four seven-byte subpackets, stored as alternating 4/3 byte words. */
	for (sub = 0; sub < 8; sub++) {
		word = 0;
		for (byte = 0; byte < (sub & 1 ? 3 : 4); byte++) {
			index = 3 + (sub / 2) * 7 + (sub & 1 ? 4 : 0) + byte;
			if (index < length)
				word |= (u32)buffer[index] << (byte * 8);
		}
		writel(word, hdmi->core + header + 4 + sub * 4);
	}
	writel(3, hdmi->core + config);
	return 0;
}

static int mv310_audio_program(void *data, const struct hdmi_codec_params *params)
{
	struct mv310_hdmi *hdmi = data;
	const u8 *status = params->iec.status;
	u8 buffer[HDMI_INFOFRAME_SIZE(AUDIO)];
	int ret;

	ret = hdmi_audio_infoframe_pack_only(&params->cea, buffer, sizeof(buffer));
	if (ret < 0)
		return ret;
	ret = mv310_write_infoframe(&hdmi->bridge, HDMI_INFOFRAME_TYPE_AUDIO,
				    buffer, ret);
	if (ret)
		return ret;
	/* hisiv100 hdmi_hal_ctrl.c:2980-3031, hdmi_reg_audio_path.h.
	 * I2S, stereo layout 0, SD0; automatic CTS follows the AIAO clock.
	 */
	writel((status[4] & 0xf) << 8, hdmi->core + MV310_AUDIO_I2S);
	writel(status[0] | status[1] << 8 | status[2] << 16 | status[3] << 24,
	       hdmi->core + MV310_AUDIO_CHST0);
	writel(status[4], hdmi->core + MV310_AUDIO_CHST1);
	writel(0xf00, hdmi->core + MV310_AUDIO_FIFO);
	writel(histb_audio_n(params->sample_rate), hdmi->core + MV310_AUDIO_N);
	writel(BIT(0) | BIT(2), hdmi->core + MV310_AUDIO_ACR);
	mv310_update(hdmi->core, MV310_PWD_RESET, BIT(7) | BIT(8), BIT(7) | BIT(8));
	udelay(5);
	mv310_update(hdmi->core, MV310_PWD_RESET, BIT(7) | BIT(8), 0);
	writel(0xe4000 | BIT(4) | BIT(0), hdmi->core + MV310_AUDIO_CTRL);
	return 0;
}

static int mv310_phy_configure(struct mv310_hdmi *hdmi, unsigned long rate)
{
	u32 divider, main_data, de, cap, term, status, cap_enable;

	divider = rate <= 50000000 ? 3 : rate <= 100000000 ? 2 :
		  rate <= 200000000 ? 1 : 0;
	/* MV310 enhanced analogue table, hdmi_hal_phy.c:176-184. */
	main_data = rate <= 100000000 ? 0x2c : rate <= 340000000 ? 0x2d : 0x39;
	de = rate <= 100000000 ? 0 : rate <= 165000000 ? 3 : 2;
	cap = rate <= 100000000 || rate > 340000000 ? 1 :
	      rate <= 165000000 ? 0 : 2;
	cap_enable = rate > 165000000 && rate <= 340000000 ? 4 : 0;
	term = rate <= 165000000 ? 0x20 : rate <= 340000000 ? 0x10 : 0x80;
	mv310_update(hdmi->phy, MV310_PHY_POWER,
		     MV310_PHY_SUPPLY_PD | GENMASK(22, 21) | GENMASK(18, 11) | BIT(4),
		     divider << 21 | 1 << 11 | BIT(4));
	mv310_update(hdmi->phy, MV310_PHY_PLL,
		     GENMASK(17, 14) | GENMASK(11, 5) | BIT(19),
		     3 << 14 | 0x20 << 5);
	mv310_update(hdmi->phy, MV310_PHY_INSSC,
		     BIT(31) | BIT(30) | GENMASK(28, 27), BIT(31) | BIT(30));
	writel(1, hdmi->phy + MV310_PHY_RESET);
	udelay(5);
	writel(0, hdmi->phy + MV310_PHY_RESET);
	/* BSP requires >=500 us after clock configuration and PHY soft reset. */
	usleep_range(1000, 2000);
	mv310_update(hdmi->phy, MV310_PHY_CLK_HIGH, GENMASK(18, 9),
		     rate > 340000000 ? 0 : 0x1f << 9);
	writel(rate > 340000000 ? 0xfffff : 0x01f07c1f,
	       hdmi->phy + MV310_PHY_CLK_LOW);
	mv310_update(hdmi->phy, MV310_PHY_FBSSC, GENMASK(27, 0), 0x01640000);
	writel(0, hdmi->phy + MV310_PHY_FBSTEP);
	mv310_update(hdmi->phy, MV310_PHY_PLL_TEST, GENMASK(19, 0), 0);
	mv310_update(hdmi->phy, MV310_PHY_MAIN, GENMASK(29, 0),
		     0x1a | main_data << 6 | main_data << 12 | main_data << 18);
	mv310_update(hdmi->phy, MV310_PHY_PRE, GENMASK(17, 0),
		     de | de << 6 | de << 12);
	mv310_update(hdmi->phy, MV310_PHY_CAP, GENMASK(17, 6),
		     cap << 6 | (cap | cap_enable) << 9 |
		     (cap | cap_enable) << 12 | (cap | cap_enable) << 15);
	writel(0x60 | term << 8 | term << 16 | term << 24,
	       hdmi->phy + MV310_PHY_TERM);
	mv310_update(hdmi->phy, MV310_PHY_TEST, GENMASK(15, 0), 0x51);
	/* SSC bypass uses the HDMI clock comparator, not analogue TXPLL lock. */
	return readl_poll_timeout(hdmi->phy + MV310_PHY_STATUS, status,
				  status & BIT(20), 100, 100000);
}

static void mv310_pre_enable(struct drm_bridge *bridge,
			     struct drm_bridge_state *old_state)
{
	struct mv310_hdmi *hdmi = to_mv310(bridge);
	struct drm_atomic_state *state = old_state->base.state;
	struct drm_connector *connector;
	struct drm_connector_state *conn_state;
	struct drm_crtc_state *crtc_state;
	unsigned long rate;
	int ret;

	connector = drm_atomic_get_new_connector_for_encoder(state, bridge->encoder);
	if (!connector)
		return;
	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (!conn_state || !conn_state->crtc)
		return;
	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (!crtc_state)
		return;
	rate = crtc_state->adjusted_mode.clock * 1000UL;
	mutex_lock(&hdmi->lock);
	if (!hdmi->powered) {
		ret = -EHOSTDOWN;
		goto error;
	}
	mv310_phy_off(hdmi);
	ret = clk_set_rate(hdmi->clocks[MV310_PIXEL].clk, rate);
	if (ret)
		goto error;
	if (abs_diff(clk_get_rate(hdmi->clocks[MV310_PIXEL].clk), rate) > rate / 1000) {
		ret = -ERANGE;
		goto error;
	}
	ret = clk_prepare_enable(hdmi->clocks[MV310_PIXEL].clk);
	if (ret)
		goto error;
	hdmi->pixel_enabled = true;
	ret = mv310_phy_configure(hdmi, rate);
	if (ret)
		goto error;
	/* Native R/G/B lanes feed the HDMI Cr/Y/Cb inputs. */
	writel(0x11, hdmi->core + MV310_VIDEO_DMUX);
	writel(4, hdmi->core + MV310_DITHER);
	writel(0x880, hdmi->core + MV310_CSC);
	writel(8, hdmi->core + MV310_DOWNSAMPLE);
	writel(0, hdmi->core + MV310_DATA_ALIGN);
	mv310_update(hdmi->core, MV310_TIMING_GEN, BIT(0), 0);
	writel(0, hdmi->core + MV310_VIDEO_BLANK);
	writel(rate > 340000000 ? BIT(0) | BIT(1) : 0,
	       hdmi->core + MV310_ENCODER);
	writel(connector->display_info.is_hdmi ? BIT(0) | BIT(2) : 0,
	       hdmi->core + MV310_AVMIXER);
	writel(0, hdmi->core + MV310_AVMUTE);
	/* Data reset after configuring the video path, minimum 80 us settling. */
	writel(BIT(3) | GENMASK(8, 6), hdmi->core + MV310_PWD_RESET);
	udelay(5);
	writel(0, hdmi->core + MV310_PWD_RESET);
	/* Enable clock detection after reset; otherwise the packer stays blank. */
	writel(BIT(2) | BIT(4), hdmi->core + MV310_PACK_FIFO);
	usleep_range(1000, 2000);
	ret = mv310_scdc_configure(hdmi, &connector->display_info, rate);
	if (ret)
		goto error;
	hdmi->mode_ready = true;
	mutex_unlock(&hdmi->lock);
	return;
error:
	if (hdmi->powered)
		mv310_phy_off(hdmi);
	if (hdmi->pixel_enabled) {
		clk_disable_unprepare(hdmi->clocks[MV310_PIXEL].clk);
		hdmi->pixel_enabled = false;
	}
	dev_err(hdmi->dev, "HDMI modeset preparation failed: %d\n", ret);
	mutex_unlock(&hdmi->lock);
}

static void mv310_enable(struct drm_bridge *bridge,
			 struct drm_bridge_state *old_state)
{
	struct mv310_hdmi *hdmi = to_mv310(bridge);
	struct drm_atomic_state *state = old_state->base.state;
	struct drm_connector *connector;
	u32 status;
	int ret;

	connector = drm_atomic_get_new_connector_for_encoder(state, bridge->encoder);
	if (!connector)
		return;
	mutex_lock(&hdmi->lock);
	if (!hdmi->mode_ready || !mv310_connected(hdmi))
		goto out;
	if (connector->display_info.is_hdmi) {
		ret = drm_atomic_helper_connector_hdmi_update_infoframes(connector, state);
		if (ret) {
			dev_err(hdmi->dev, "HDMI infoframes failed: %d\n", ret);
			mv310_phy_off(hdmi);
			goto out;
		}
	}
	/* CRTC timing is running now; require the pixel/TMDS ratio to settle. */
	ret = readl_poll_timeout(hdmi->core + MV310_PACK_FIFO_ST, status,
				 status & BIT(0), 1000, 20000);
	if (ret) {
		dev_err(hdmi->dev, "TMDS clock did not stabilize: %d\n", ret);
		mv310_phy_off(hdmi);
		goto out;
	}
	mv310_update(hdmi->phy, MV310_PHY_POWER,
		     MV310_PHY_OUTPUT_PD | MV310_PHY_DE_PD,
		     readl(hdmi->phy + MV310_PHY_PRE) & GENMASK(17, 0) ? BIT(5) :
		     MV310_PHY_DE_PD);
	ret = histb_audio_enable(&hdmi->audio, connector);
	if (ret)
		dev_err(hdmi->dev, "HDMI PCM restore failed: %d\n", ret);
out:
	mutex_unlock(&hdmi->lock);
}

static void mv310_disable(struct drm_bridge *bridge,
			  struct drm_bridge_state *old_state)
{
	struct mv310_hdmi *hdmi = to_mv310(bridge);

	mutex_lock(&hdmi->lock);
	if (hdmi->powered) {
		mv310_phy_off(hdmi);
		mv310_packets_off(hdmi);
	}
	mutex_unlock(&hdmi->lock);
}

static void mv310_post_disable(struct drm_bridge *bridge,
			       struct drm_bridge_state *old_state)
{
	struct mv310_hdmi *hdmi = to_mv310(bridge);

	mutex_lock(&hdmi->lock);
	if (hdmi->pixel_enabled) {
		clk_disable_unprepare(hdmi->clocks[MV310_PIXEL].clk);
		hdmi->pixel_enabled = false;
	}
	mutex_unlock(&hdmi->lock);
}

static const struct drm_bridge_funcs mv310_bridge_funcs = {
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_check = mv310_atomic_check,
	.atomic_pre_enable = mv310_pre_enable,
	.atomic_enable = mv310_enable,
	.atomic_disable = mv310_disable,
	.atomic_post_disable = mv310_post_disable,
	.detect = mv310_detect,
	.hpd_enable = mv310_hpd_enable,
	.hpd_disable = mv310_hpd_disable,
	.edid_read = mv310_edid_read,
	.mode_valid = mv310_mode_valid,
	.hdmi_tmds_char_rate_valid = mv310_tmds_valid,
	.hdmi_clear_infoframe = mv310_clear_infoframe,
	.hdmi_write_infoframe = mv310_write_infoframe,
};

static void mv310_release(void *data)
{
	struct mv310_hdmi *hdmi = data;
	int ret;

	mv310_hpd_disable(&hdmi->bridge);
	cancel_delayed_work_sync(&hdmi->hpd_work);
	mutex_lock(&hdmi->lock);
	mv310_power_off(hdmi);
	ret = regmap_update_bits(hdmi->sysctrl, MV310_SYS_CTRL,
				 MV310_SYS_GATES | MV310_SYS_RESETS,
				 hdmi->saved_sysctrl);
	if (ret)
		dev_warn(hdmi->dev, "HDMI sysctrl restore failed: %d\n", ret);
	ret = regmap_update_bits(hdmi->sysctrl, MV310_SYS_MUX,
				 MV310_SYS_MUX_MASK, hdmi->saved_mux);
	if (ret)
		dev_warn(hdmi->dev, "HDMI pinmux restore failed: %d\n", ret);
	ret = regmap_update_bits(hdmi->perictrl, MV310_PERI_AUDIO_MUX,
				 MV310_PERI_AUDIO_MASK, hdmi->saved_audio_mux);
	if (ret)
		dev_warn(hdmi->dev, "HDMI audio mux restore failed: %d\n", ret);
	mutex_unlock(&hdmi->lock);
}

static int mv310_suspend(struct device *dev)
{
	struct mv310_hdmi *hdmi = dev_get_drvdata(dev);

	/* Preserve the bridge's HPD subscription across system sleep. */
	mutex_lock(&hdmi->lock);
	mv310_power_off(hdmi);
	mutex_unlock(&hdmi->lock);
	cancel_delayed_work_sync(&hdmi->hpd_work);
	return 0;
}

static int mv310_resume(struct device *dev)
{
	struct mv310_hdmi *hdmi = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&hdmi->lock);
	ret = mv310_power_on(hdmi);
	if (!ret && hdmi->hpd_enabled) {
		writel(hdmi->irq >= 0 ? MV310_HPD : 0,
		       hdmi->core + MV310_AON_MASK);
		schedule_delayed_work(&hdmi->hpd_work, 0);
	}
	mutex_unlock(&hdmi->lock);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mv310_pm_ops, mv310_suspend, mv310_resume);

static int mv310_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mv310_hdmi *hdmi;
	unsigned int i;
	int ret;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;
	hdmi->dev = dev;
	mutex_init(&hdmi->lock);
	hdmi->audio.lock = &hdmi->lock;
	hdmi->audio.data = hdmi;
	hdmi->audio.program = mv310_audio_program;
	hdmi->audio.stop = mv310_audio_stop;
	INIT_DELAYED_WORK(&hdmi->hpd_work, mv310_hpd_work);
	hdmi->last_status = -1;
	hdmi->irq = platform_get_irq_byname_optional(pdev, "aon");
	if (hdmi->irq < 0 && hdmi->irq != -ENXIO)
		return dev_err_probe(dev, hdmi->irq, "failed to get AON IRQ\n");
	hdmi->core = devm_platform_ioremap_resource_byname(pdev, "core");
	if (IS_ERR(hdmi->core))
		return PTR_ERR(hdmi->core);
	hdmi->phy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(hdmi->phy))
		return PTR_ERR(hdmi->phy);
	for (i = 0; i < ARRAY_SIZE(hdmi->clocks); i++)
		hdmi->clocks[i].id = mv310_clock_names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(hdmi->clocks), hdmi->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get HDMI clocks\n");
	for (i = 0; i < ARRAY_SIZE(hdmi->resets); i++)
		hdmi->resets[i].id = mv310_reset_names[i];
	ret = devm_reset_control_bulk_get_exclusive(dev, ARRAY_SIZE(hdmi->resets),
						   hdmi->resets);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get HDMI resets\n");
	hdmi->sysctrl = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "hisilicon,sysctrl");
	if (IS_ERR(hdmi->sysctrl))
		return dev_err_probe(dev, PTR_ERR(hdmi->sysctrl), "missing HDMI sysctrl\n");
	hdmi->perictrl = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "hisilicon,perictrl");
	if (IS_ERR(hdmi->perictrl))
		return dev_err_probe(dev, PTR_ERR(hdmi->perictrl), "missing peripheral syscon\n");
	ret = regmap_read(hdmi->perictrl, MV310_PERI_AUDIO_MUX, &hdmi->saved_audio_mux);
	if (ret)
		return ret;
	ret = regmap_read(hdmi->sysctrl, MV310_SYS_MUX, &hdmi->saved_mux);
	if (ret)
		return ret;
	ret = regmap_read(hdmi->sysctrl, MV310_SYS_CTRL, &hdmi->saved_sysctrl);
	if (ret)
		return ret;
	ret = mv310_power_on(hdmi);
	if (ret)
		return dev_err_probe(dev, ret, "HDMI power sequence failed\n");
	ret = devm_add_action_or_reset(dev, mv310_release, hdmi);
	if (ret)
		return ret;
	if (hdmi->irq >= 0) {
		ret = devm_request_irq(dev, hdmi->irq, mv310_irq, IRQF_SHARED,
				       dev_name(dev), hdmi);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request HDMI AON IRQ\n");
	}
	hdmi->bridge.funcs = &mv310_bridge_funcs;
	hdmi->bridge.of_node = dev->of_node;
	hdmi->bridge.type = DRM_MODE_CONNECTOR_HDMIA;
	hdmi->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID |
			   DRM_BRIDGE_OP_HPD | DRM_BRIDGE_OP_HDMI;
	hdmi->bridge.vendor = "HISI";
	hdmi->bridge.product = "Hi3798MV310 HDMI";
	hdmi->bridge.supported_formats = BIT(HDMI_COLORSPACE_RGB);
	hdmi->bridge.max_bpc = 8;
	platform_set_drvdata(pdev, hdmi);
	ret = devm_drm_bridge_add(dev, &hdmi->bridge);
	if (ret)
		return ret;
	ret = histb_audio_register(dev, &hdmi->audio);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register HDMI codec\n");
	dev_info(dev, "registered hisiv100 HDMI RGB8 bridge (%s HPD), sink %s\n",
		 hdmi->irq >= 0 ? "AON IRQ" : "polled",
		 mv310_connected(hdmi) ? "connected" : "disconnected");
	return 0;
}

static void mv310_remove(struct platform_device *pdev)
{
	struct mv310_hdmi *hdmi = platform_get_drvdata(pdev);

	histb_audio_unregister(&hdmi->audio);
	mv310_hpd_disable(&hdmi->bridge);
	cancel_delayed_work_sync(&hdmi->hpd_work);
}

static const struct of_device_id mv310_of_match[] = {
	{ .compatible = "hisilicon,hi3798mv310-hdmi" },
	{ }
};
MODULE_DEVICE_TABLE(of, mv310_of_match);

static struct platform_driver mv310_driver = {
	.probe = mv310_probe,
	.remove = mv310_remove,
	.driver = {
		.name = "histb-hdmi-mv310",
		.of_match_table = mv310_of_match,
		.pm = pm_sleep_ptr(&mv310_pm_ops),
	},
};
module_platform_driver(mv310_driver);

MODULE_DESCRIPTION("HiSilicon Hi3798MV310 hisiv100 HDMI bridge and PHY");
MODULE_LICENSE("GPL");
