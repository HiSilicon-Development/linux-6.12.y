// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3798CV200 AIAO playback driver
 *
 * Register layout and stop sequencing follow HiSTBLinuxV100R005C00SPC060
 * source/msp/drv/aiao/aiao_v1_1/common/src/hal_aiao_priv.c, which credits
 * Copyright 2004 - 2014, Hisilicon Tech. Co., Ltd.
 *
 * The controller owns a native circular DMA buffer.  ALSA's application
 * pointer is written directly to the hardware producer pointer while the
 * hardware consumer pointer provides the PCM position.
 *
 * Three transmit DAIs are exposed:
 *   DAI 0 - board-selected I2S bank (the internal Tianlai DAC on Q10 Pro);
 *   DAI 1 - internal HDMI I2S bank (TX3 on CV200, TX2 on MV310);
 *   DAI 2 - S/PDIF transmitter.  A SPDIF-TX DMA bank feeds the matching
 *           SPDIF encoder ("SPDIFER"), which drives the optical/coaxial
 *           pad selected by AIAO_SPDIF_TX_MUX.  The encoder's channel
 *           status comes from the ALSA IEC958 control, so IEC 61937
 *           (AC-3/DTS) bursts pass through bit-exact: the TX DSP is
 *           bypassed and the encoder is switched to compressed mode.
 *
 * All DAIs share one DMA engine and one channel-status state, so only one
 * PCM can be open at a time.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/pm.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rcupdate.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>

#include <sound/asoundef.h>
#include <sound/pcm.h>
#include <sound/pcm_iec958.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define AIAO_INT_ENA			0x0000
#define AIAO_INT_STATUS			0x0004
#define AIAO_INT_RAW			0x0008
#define AIAO_SPDIF_TX_MUX		0x0020
#define AIAO_SWITCH_TX_BCLK		0x002c
#define  AIAO_TX0_BCLK_SOURCE_MASK	GENMASK(3, 0)
#define  AIAO_TX0_BCLK_SOURCE_TX0	8
#define AIAO_STATUS			0x0030
#define AIAO_OUTSTANDING		0x0034
#define  AIAO_OUTSTANDING_MASK		GENMASK(2, 0)

/* Per-bank register windows. */
#define AIAO_TX_BASE			0x2000
#define AIAO_SPDIF_TX_BASE		0x3000
#define AIAO_TX_BANK_STRIDE		0x0100
#define AIAO_SPDIFER_BASE		0x4000
#define AIAO_SPDIFER_STRIDE		0x1000
#define AIAO_I2S_TX_CRG_BASE		0x0140
#define AIAO_SPDIF_CRG_BASE		0x0180

/* Offsets inside an I2S-TX or SPDIF-TX bank (they share one layout). */
#define AIAO_TX_IF_ATTR			0x0000
#define  AIAO_TX_PRECISION_MASK		GENMASK(3, 2)
#define  AIAO_TX_CHANNELS_MASK		GENMASK(5, 4)
#define  AIAO_TX_SOURCE_MASK		GENMASK(23, 20)
#define  AIAO_TX_SD0_MASK		GENMASK(25, 24)
#define  AIAO_TX_SD1_MASK		GENMASK(27, 26)
#define  AIAO_TX_SD2_MASK		GENMASK(29, 28)
#define  AIAO_TX_SD3_MASK		GENMASK(31, 30)
#define AIAO_TX_DSP_CTRL		0x0004
#define  AIAO_TX_MUTE_FADE		BIT(1)
#define  AIAO_TX_VOLUME_MASK		GENMASK(14, 8)
#define  AIAO_TX_FADE_IN_MASK		GENMASK(19, 16)
#define  AIAO_TX_FADE_OUT_MASK		GENMASK(23, 20)
#define  AIAO_TX_BYPASS			BIT(27)
#define  AIAO_TX_ENABLE			BIT(28)
#define  AIAO_TX_DISABLE_DONE		BIT(29)

#define AIAO_TX_BUF_ADDR		0x0080
#define AIAO_TX_BUF_SIZE		0x0084
#define AIAO_TX_BUF_WPTR		0x0088
#define AIAO_TX_BUF_RPTR		0x008c
#define AIAO_TX_BUF_ALEMPTY		0x0090
#define AIAO_TX_TRANS_SIZE		0x0094
#define AIAO_TX_INT_ENA			0x00a0
#define AIAO_TX_INT_RAW			0x00a4
#define AIAO_TX_INT_STATUS		0x00a8
#define AIAO_TX_INT_CLR			0x00ac
#define  AIAO_TX_INT_PERIOD		BIT(0)
#define  AIAO_TX_INT_EMPTY		BIT(1)
#define  AIAO_TX_INT_ALL		GENMASK(7, 0)

/* Clock configuration, 8 bytes per bank in the 0x140/0x180 windows. */
#define AIAO_CRG_CFG0			0x0000
#define AIAO_CRG_CFG1			0x0004
#define  AIAO_CRG_MCLK_DIV_MASK		GENMASK(26, 0)
#define  AIAO_CRG_BCLK_DIV_MASK		GENMASK(3, 0)
#define  AIAO_CRG_FSCLK_DIV_MASK	GENMASK(6, 4)
#define  AIAO_CRG_CLK_EN		BIT(8)
#define  AIAO_CRG_BCLK_DIV_128FS	2	/* index into {1,3,2,4,...} */
#define  AIAO_CRG_FS_DIV_128FS		4	/* index into {16,32,48,64,128,256} */
#define  AIAO_CRG_BCLK_DIV_64FS		3
#define  AIAO_CRG_FS_DIV_64FS		3

/* S/PDIF encoder ("SPDIFER") register bank. */
#define AIAO_SPDIF_CTRL			0x0000
#define  AIAO_SPDIF_EN			BIT(0)
#define AIAO_SPDIF_CONFIG		0x0004
#define  AIAO_SPDIF_MODE_PCM		0
#define  AIAO_SPDIF_MODE_COMPRESSED	1
#define AIAO_SPDIF_CS1			0x0020
#define AIAO_SPDIF_CS2			0x0024
#define AIAO_SPDIF_CS3			0x0028
#define AIAO_SPDIF_CS4			0x002c
#define AIAO_SPDIF_CS5			0x0030

#define AIAO_EPLL_RATE			933888000ULL
#define AIAO_MCLK_RATIO			256ULL
#define AIAO_BUFFER_ALIGN		128
#define AIAO_DMA_GAP			32
#define AIAO_BUFFER_PTR_MASK		GENMASK(23, 0)
#define AIAO_BUFFER_BYTES_MAX		(256 * 1024)
#define AIAO_IRQ_ACK_RETRIES		4

#define AIAO_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
		    SNDRV_PCM_RATE_12000 | SNDRV_PCM_RATE_16000 | \
		    SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_24000 | \
		    SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		    SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
		    SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
		    SNDRV_PCM_RATE_192000)

/* Consumer S/PDIF covers 32 kHz .. 192 kHz; the IEC 60958 status codes for
 * all seven rates are representable in the encoder.
 */
#define AIAO_SPDIF_RATES (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
			  SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
			  SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
			  SNDRV_PCM_RATE_192000)

/* Only hw_params-owned configuration is replayed after controller reset. */
struct aiao_tx_config {
	u32 crg0;
	u32 crg1;
	u32 if_attr;
	u32 dsp_ctrl;
	u32 dma_addr;
	u32 buffer_bytes;
	u32 period_bytes;
	unsigned int channel;
	unsigned int spdif_pad;
	unsigned int rate;
	unsigned int sample_bits;
	u8 cs[5];
	bool spdif;
	bool hdmi_bypass;
	bool valid;
	unsigned int frame_bytes;
};

struct hi3798cv200_aiao {
	struct device *dev;
	void __iomem *base;
	struct clk *bus_clk;
	struct clk *mclk;
	unsigned long mclk_rate;
	struct reset_control *reset;
	struct snd_pcm_substream __rcu *substream;
	/* Serializes top-level and channel register read-modify-write cycles. */
	spinlock_t reg_lock;
	struct aiao_tx_config config;
	u32 last_pos;
	int stop_error;
	bool prepared;
	/* IRQF_SHARED handlers must not touch a controller with gated clocks. */
	bool powered;
	/* Selected by the board's audio routing. */
	unsigned int tx_channel;
	unsigned int analog_tx_channel;
	bool hdmi_stream;
	unsigned int hdmi_tx_channel;
	unsigned int spdif_tx_channel;
	unsigned int spdif_pad;
	bool spdif_stream;
	/* ALSA "IEC958 Playback Default" AES0..AES3 bytes. */
	u8 iec958[4];
	bool iec958_valid;
	int irq;
	atomic_t opened;
	/* Only six-channel streams need a padded hardware ring. The public
	 * mmap ring remains six-channel; ack publishes each frame once.
	 */
	void *packed_area;
	dma_addr_t packed_addr;
	size_t packed_bytes;
	snd_pcm_uframes_t packed_appl;
};

static inline u32 aiao_bank_base(unsigned int channel, bool spdif)
{
	return (spdif ? AIAO_SPDIF_TX_BASE : AIAO_TX_BASE) +
	       AIAO_TX_BANK_STRIDE * channel;
}

static inline u32 aiao_crg_base(unsigned int channel, bool spdif)
{
	return (spdif ? AIAO_SPDIF_CRG_BASE : AIAO_I2S_TX_CRG_BASE) +
	       8 * channel;
}

static inline u32 aiao_encoder_base(unsigned int channel)
{
	return AIAO_SPDIFER_BASE + AIAO_SPDIFER_STRIDE * channel;
}

static inline u32 aiao_tx_reg(const struct hi3798cv200_aiao *aiao, u32 reg)
{
	return aiao_bank_base(aiao->tx_channel, aiao->spdif_stream) + reg;
}

static inline u32 aiao_tx_crg_reg(const struct hi3798cv200_aiao *aiao, u32 reg)
{
	return aiao_crg_base(aiao->tx_channel, aiao->spdif_stream) + reg;
}

static inline u32 aiao_cfg_reg(const struct aiao_tx_config *config, u32 reg)
{
	return aiao_bank_base(config->channel, config->spdif) + reg;
}

static inline u32 aiao_cfg_crg_reg(const struct aiao_tx_config *config, u32 reg)
{
	return aiao_crg_base(config->channel, config->spdif) + reg;
}

static inline u32 aiao_tx_top_irq(const struct hi3798cv200_aiao *aiao)
{
	return BIT((aiao->spdif_stream ? 24 : 16) + aiao->tx_channel);
}

static const struct snd_pcm_hardware hi3798cv200_aiao_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_NO_REWINDS,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.rates = AIAO_RATES,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = 2,
	.channels_max = 8,
	.period_bytes_min = AIAO_BUFFER_ALIGN,
	.period_bytes_max = 64 * 1024,
	.periods_min = 2,
	.periods_max = 128,
	.buffer_bytes_max = AIAO_BUFFER_BYTES_MAX,
};

/*
 * Deep standby loses the FIFO as well as the DMA registers.  Do not advertise
 * INFO_RESUME: ALSA leaves the stream SUSPENDED, snd_pcm_resume() returns
 * -ENOSYS, and userspace must prepare/refill it.  prepare resets both hardware
 * byte positions to zero before ALSA resets hw_ptr/appl_ptr to that origin.
 * PAUSE retains the live ring; system resume never replays its old pointers.
 */

static inline u32 aiao_read(struct hi3798cv200_aiao *aiao, u32 reg)
{
	return readl(aiao->base + reg);
}

static inline void aiao_write(struct hi3798cv200_aiao *aiao, u32 reg, u32 val)
{
	writel(val, aiao->base + reg);
}

static void aiao_update_bits(struct hi3798cv200_aiao *aiao, u32 reg,
			     u32 mask, u32 val)
{
	unsigned long flags;
	u32 tmp;

	spin_lock_irqsave(&aiao->reg_lock, flags);
	tmp = aiao_read(aiao, reg);
	tmp &= ~mask;
	tmp |= val & mask;
	aiao_write(aiao, reg, tmp);
	spin_unlock_irqrestore(&aiao->reg_lock, flags);
}

static u32 aiao_mclk_div(struct hi3798cv200_aiao *aiao, unsigned int rate)
{
	u64 numerator = (u64)rate * AIAO_MCLK_RATIO * BIT_ULL(27);

	return DIV_ROUND_CLOSEST_ULL(numerator, aiao->mclk_rate);
}

/* IEC 60958-3 / IEC 60958-4 channel-status bytes, as ALSA defines them. */
static u32 aiao_fs_code(unsigned int rate)
{
	switch (rate) {
	case 32000:
		return IEC958_AES3_CON_FS_32000;
	case 44100:
		return IEC958_AES3_CON_FS_44100;
	case 48000:
		return IEC958_AES3_CON_FS_48000;
	case 88200:
		return IEC958_AES3_CON_FS_88200;
	case 96000:
		return IEC958_AES3_CON_FS_96000;
	case 176400:
		return IEC958_AES3_CON_FS_176400;
	case 192000:
		return IEC958_AES3_CON_FS_192000;
	default:
		return IEC958_AES3_CON_FS_NOTID;
	}
}

static u32 aiao_orig_fs_code(unsigned int rate)
{
	switch (rate) {
	case 32000:
		return IEC958_AES4_CON_ORIGFS_32000;
	case 44100:
		return IEC958_AES4_CON_ORIGFS_44100;
	case 48000:
		return IEC958_AES4_CON_ORIGFS_48000;
	case 88200:
		return IEC958_AES4_CON_ORIGFS_88200;
	case 96000:
		return IEC958_AES4_CON_ORIGFS_96000;
	case 176400:
		return IEC958_AES4_CON_ORIGFS_176400;
	case 192000:
		return IEC958_AES4_CON_ORIGFS_192000;
	default:
		return IEC958_AES4_CON_ORIGFS_NOTID;
	}
}

static u32 aiao_wordlen_code(unsigned int sample_bits)
{
	/* AES4 bit 0 is "max word length 24"; bits 3:1 select the word
	 * length class. 16-bit -> 0x02 (20/16), 24-bit -> 0x0b (24/20 + max).
	 */
	if (sample_bits <= 16)
		return IEC958_AES4_CON_WORDLEN_20_16;

	return IEC958_AES4_CON_WORDLEN_24_20 | IEC958_AES4_CON_MAX_WORDLEN_24;
}

/* L/R channel-status bytes live in one register, four bits each.  The
 * encoder sends the same AES2 byte on both subframes; only the channel
 * number differs.  An unspecified channel number becomes the stereo pair
 * 1/2, matching the vendor's 0x2010 default.
 */
static u32 aiao_cs3_value(const u8 cs[5])
{
	unsigned int source = cs[2] & IEC958_AES2_CON_SOURCE;
	unsigned int channel = (cs[2] & IEC958_AES2_CON_CHANNEL) >> 4;
	unsigned int left = channel ? channel : 1;
	unsigned int right = channel ? (channel < 15 ? channel + 1 : 15) : 2;

	return source | (left << 4) | (source << 8) | (right << 12);
}

/*
 * Build AES0..AES4 from the ALSA control (AES0..AES3) and the stream
 * parameters.  The driver owns the sampling-frequency and word-length
 * fields because they must match the clock it programmed; everything else
 * is left to userspace.  A non-zero AES0 bit 1 marks an IEC 61937
 * compressed burst and is preserved.
 */
static void aiao_build_cs(struct hi3798cv200_aiao *aiao,
			  const struct aiao_tx_config *config, u8 cs[5])
{
	/* Callers serialize against the control's put callback: hw_params
	 * takes reg_lock, and aiao_update_spdif_output already holds it.
	 */
	snd_pcm_create_iec958_consumer_default(cs, 5);
	if (aiao->iec958_valid)
		memcpy(cs, aiao->iec958, sizeof(aiao->iec958));

	cs[3] = (cs[3] & ~IEC958_AES3_CON_FS) | aiao_fs_code(config->rate);
	cs[4] = (cs[4] & ~(IEC958_AES4_CON_WORDLEN |
			   IEC958_AES4_CON_MAX_WORDLEN_24 |
			   IEC958_AES4_CON_ORIGFS)) |
		aiao_wordlen_code(config->sample_bits) |
		aiao_orig_fs_code(config->rate);
}

static u32 aiao_dsp_ctrl_value(const struct aiao_tx_config *config)
{
	/* A compressed burst must reach the encoder untouched, and the HDMI
	 * transmitter must see IEC 61937 preambles bit-exact.  Both therefore
	 * bypass the volume/fade block.  LPCM keeps the vendor's fade-in/out
	 * and 0 dB setting.
	 */
	if ((config->spdif && (config->cs[0] & IEC958_AES0_NONAUDIO)) ||
	    config->hdmi_bypass)
		return AIAO_TX_BYPASS;

	return AIAO_TX_MUTE_FADE |
	       FIELD_PREP(AIAO_TX_VOLUME_MASK, 0x79) |
	       FIELD_PREP(AIAO_TX_FADE_IN_MASK, 4) |
	       FIELD_PREP(AIAO_TX_FADE_OUT_MASK, 3);
}

/* Write the five channel-status bytes and the PCM/compressed mode select.
 * Caller holds reg_lock or owns the controller exclusively.
 */
static void aiao_program_cs(struct hi3798cv200_aiao *aiao,
			    const struct aiao_tx_config *config, const u8 cs[5])
{
	u32 enc = aiao_encoder_base(config->channel);

	aiao_write(aiao, enc + AIAO_SPDIF_CS1, (cs[0] << 8) | cs[0]);
	aiao_write(aiao, enc + AIAO_SPDIF_CS2, (cs[1] << 8) | cs[1]);
	aiao_write(aiao, enc + AIAO_SPDIF_CS3, aiao_cs3_value(cs));
	aiao_write(aiao, enc + AIAO_SPDIF_CS4, (cs[3] << 8) | cs[3]);
	aiao_write(aiao, enc + AIAO_SPDIF_CS5, (cs[4] << 8) | cs[4]);
	aiao_write(aiao, enc + AIAO_SPDIF_CONFIG,
		   (cs[0] & IEC958_AES0_NONAUDIO) ?
		   AIAO_SPDIF_MODE_COMPRESSED : AIAO_SPDIF_MODE_PCM);
}

static void aiao_restore_tx_config(struct hi3798cv200_aiao *aiao)
{
	const struct aiao_tx_config *config = &aiao->config;

	if (!config->valid)
		return;

	/* Replay only the owner of hw_params, never another DAI's TX bank. */
	aiao_write(aiao, aiao_cfg_reg(config, AIAO_TX_INT_ENA), 0);
	aiao_write(aiao, aiao_cfg_reg(config, AIAO_TX_INT_CLR), AIAO_TX_INT_ALL);
	aiao_write(aiao, aiao_cfg_crg_reg(config, AIAO_CRG_CFG1), 0);
	aiao_write(aiao, aiao_cfg_crg_reg(config, AIAO_CRG_CFG0), config->crg0);
	aiao_write(aiao, aiao_cfg_crg_reg(config, AIAO_CRG_CFG1), config->crg1);
	aiao_write(aiao, aiao_cfg_crg_reg(config, AIAO_CRG_CFG1),
		   config->crg1 | AIAO_CRG_CLK_EN);

	if (config->spdif) {
		u32 pad = config->spdif_pad;

		/* Pad k: bits [8k+2:8k] select the encoder, [8k+3] enables it. */
		aiao_update_bits(aiao, AIAO_SPDIF_TX_MUX,
				 GENMASK(8 * pad + 3, 8 * pad),
				 (config->channel | BIT(3)) << (8 * pad));
	} else {
		u32 bclk_mask = GENMASK(config->channel * 4 + 3,
					config->channel * 4);
		u32 bclk_value = (8 + config->channel) << (config->channel * 4);

		aiao_update_bits(aiao, AIAO_SWITCH_TX_BCLK, bclk_mask,
				 bclk_value);
	}

	aiao_write(aiao, aiao_cfg_reg(config, AIAO_TX_IF_ATTR), config->if_attr);
	aiao_write(aiao, aiao_cfg_reg(config, AIAO_TX_DSP_CTRL),
		   config->dsp_ctrl);
	aiao_write(aiao, aiao_cfg_reg(config, AIAO_TX_BUF_ADDR),
		   config->dma_addr);
	aiao_write(aiao, aiao_cfg_reg(config, AIAO_TX_BUF_SIZE),
		   config->buffer_bytes);
	aiao_write(aiao, aiao_cfg_reg(config, AIAO_TX_BUF_ALEMPTY),
		   config->period_bytes);
	aiao_write(aiao, aiao_cfg_reg(config, AIAO_TX_TRANS_SIZE),
		   config->period_bytes);
	/* No queued samples survive the required ALSA prepare/refill operation. */
	aiao_write(aiao, aiao_cfg_reg(config, AIAO_TX_BUF_WPTR), 0);
	aiao_write(aiao, aiao_cfg_reg(config, AIAO_TX_BUF_RPTR), 0);

	if (config->spdif) {
		aiao_program_cs(aiao, config, config->cs);
		aiao_write(aiao, aiao_encoder_base(config->channel) +
			   AIAO_SPDIF_CTRL, 0);
	}
}

/* Recompute and push the S/PDIF channel status after userspace changed the
 * IEC958 control.
 */
static void aiao_update_spdif_output(struct hi3798cv200_aiao *aiao)
{
	struct aiao_tx_config *config = &aiao->config;
	unsigned long flags;
	u8 cs[5];

	spin_lock_irqsave(&aiao->reg_lock, flags);
	if (!config->valid || !config->spdif)
		goto unlock;

	aiao_build_cs(aiao, config, cs);
	memcpy(config->cs, cs, sizeof(config->cs));
	config->dsp_ctrl = aiao_dsp_ctrl_value(config);
	if (!READ_ONCE(aiao->powered))
		goto unlock;

	aiao_write(aiao, aiao_cfg_reg(config, AIAO_TX_DSP_CTRL),
		   config->dsp_ctrl);
	aiao_program_cs(aiao, config, cs);
unlock:
	spin_unlock_irqrestore(&aiao->reg_lock, flags);
}

static int aiao_stop_tx(struct hi3798cv200_aiao *aiao)
{
	u32 val, trans_size;
	int ret = 0;
	const u32 tx_dsp = aiao_tx_reg(aiao, AIAO_TX_DSP_CTRL);
	const u32 tx_rptr = aiao_tx_reg(aiao, AIAO_TX_BUF_RPTR);
	const u32 tx_trans = aiao_tx_reg(aiao, AIAO_TX_TRANS_SIZE);
	const u32 tx_int_ena = aiao_tx_reg(aiao, AIAO_TX_INT_ENA);
	const u32 tx_int_clr = aiao_tx_reg(aiao, AIAO_TX_INT_CLR);
	const u32 top_irq = aiao_tx_top_irq(aiao);

	if (!READ_ONCE(aiao->powered))
		return 0;

	/* Stop delivery before the disable handshake can empty the FIFO. */
	aiao_write(aiao, tx_int_ena, 0);
	aiao_update_bits(aiao, AIAO_INT_ENA, top_irq, 0);
	trans_size = aiao_read(aiao, tx_trans);
	val = aiao_read(aiao, tx_dsp);
	if ((val & AIAO_TX_ENABLE) || aiao->stop_error) {
		if (val & AIAO_TX_ENABLE) {
			aiao_update_bits(aiao, tx_dsp, AIAO_TX_ENABLE, 0);
			aiao->last_pos = aiao_read(aiao, tx_rptr) &
				AIAO_BUFFER_PTR_MASK;
		}
		aiao_write(aiao, tx_trans, 1);
		aiao_write(aiao, tx_rptr, 0);
		udelay(500);
		ret = readl_poll_timeout_atomic(aiao->base + tx_dsp, val,
						val & AIAO_TX_DISABLE_DONE, 10, 1000);
		if (ret)
			dev_warn_ratelimited(aiao->dev,
					     "AIAO TX%u stop timed out\n", aiao->tx_channel);
		if (!ret)
			aiao_write(aiao, tx_rptr, aiao->last_pos);
		if (trans_size)
			aiao_write(aiao, tx_trans, trans_size);
	}

	aiao_write(aiao, tx_int_clr, AIAO_TX_INT_ALL);
	if (aiao->spdif_stream) {
		u32 enc = aiao_encoder_base(aiao->tx_channel);

		/* Vendor stop sequence: restore channel numbers, then gate the
		 * biphase encoder so the pad goes idle instead of emitting the
		 * tail of the ring.
		 */
		aiao_write(aiao, enc + AIAO_SPDIF_CS3, 0x2010);
		aiao_write(aiao, enc + AIAO_SPDIF_CTRL, 0);
	}
	rcu_assign_pointer(aiao->substream, NULL);
	aiao->stop_error = ret;
	if (ret)
		aiao->prepared = false;
	return ret;
}

static int hi3798cv200_aiao_open(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	snd_soc_set_runtime_hwparams(substream,
				     &hi3798cv200_aiao_pcm_hardware);

	/* The 24-bit DMA path consumes the most significant bits of a word,
	 * not ALSA S24_LE's low three bytes. Let userspace convert to S32.
	 */
	ret = snd_pcm_hw_constraint_msbits(runtime, 0, 32, 24);
	if (ret < 0)
		return ret;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
					 AIAO_BUFFER_ALIGN);
	if (ret < 0)
		return ret;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
					 AIAO_BUFFER_ALIGN);
	if (ret < 0)
		return ret;

	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;
	/* Constraint refinement can return positive success; always claim DMA. */
	if (atomic_cmpxchg(&aiao->opened, 0, 1))
		return -EBUSY;
	return 0;
}

static int hi3798cv200_aiao_close(struct snd_soc_component *component,
				  struct snd_pcm_substream *substream)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);

	synchronize_rcu();
	atomic_set(&aiao->opened, 0);
	return 0;
}

static int hi3798cv200_aiao_sync_stop(struct snd_soc_component *component,
				     struct snd_pcm_substream *substream)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);

	/* Drain callbacks that observed the substream before STOP cleared it. */
	synchronize_irq(aiao->irq);
	return 0;
}


/* AIAO TX has only mono/stereo/eight-slot encodings (BSP hal_aiao_priv.c
 * AIAO_LOW_SetI2SChNum:2265-2279). CEA slots 0..5 are FL FR LFE FC RL RR;
 * the unused final pair must be silent, not a repeat of the next frame.
 * This is the only unavoidable data copy; stereo/eight-channel mmap stays
 * directly backed by the hardware DMA ring.
 */
static void aiao_expand_6ch(void *dst, const void *src, size_t frames,
			    unsigned int sample_bytes)
{
	u8 *out = dst;
	const u8 *in = src;

	while (frames--) {
		memcpy(out, in, 6 * sample_bytes);
		memset(out + 6 * sample_bytes, 0, 2 * sample_bytes);
		out += 8 * sample_bytes;
		in += 6 * sample_bytes;
	}
}

static int hi3798cv200_aiao_hw_params(struct snd_soc_component *component,
				      struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aiao_tx_config *config = &aiao->config;
	u32 precision, if_attr, crg1;
	size_t buffer_bytes = params_buffer_bytes(params);
	size_t period_bytes = params_period_bytes(params);
	unsigned int channels = params_channels(params);
	unsigned int sample_bytes = snd_pcm_format_physical_width(params_format(params)) / 8;
	unsigned int hw_channels = channels == 6 ? 8 : channels;
	dma_addr_t dma_addr = runtime->dma_addr;
	bool spdif = aiao->spdif_stream;
	unsigned long flags;
	int ret;

	if (!READ_ONCE(aiao->powered))
		return -ESTRPIPE;
	if (spdif && channels != 2)
		return -EINVAL;
	if (!spdif && channels != 2 &&
	    (!aiao->hdmi_stream || aiao->hdmi_tx_channel != 3 ||
	     (channels != 6 && channels != 8)))
		return -EINVAL;
	if (!IS_ALIGNED(runtime->dma_addr, AIAO_BUFFER_ALIGN) ||
	    upper_32_bits(runtime->dma_addr))
		return -EINVAL;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		precision = 1;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		precision = 2;
		break;
	default:
		return -EINVAL;
	}
	ret = aiao_stop_tx(aiao);
	if (ret)
		return ret;
	aiao->prepared = false;
	config->valid = false;
	/* hw_params may be repeated without hw_free. Never leave an old
	 * six-channel bounce buffer live after switching back to direct DMA.
	 */
	if (aiao->packed_area) {
		dma_free_coherent(aiao->dev, aiao->packed_bytes,
				  aiao->packed_area, aiao->packed_addr);
		aiao->packed_area = NULL;
	}
	if (channels == 6) {
		buffer_bytes = params_buffer_size(params) * 8 * sample_bytes;
		period_bytes = params_period_size(params) * 8 * sample_bytes;
		aiao->packed_area = dma_alloc_coherent(aiao->dev, buffer_bytes,
						      &aiao->packed_addr, GFP_KERNEL);
		if (!aiao->packed_area)
			return -ENOMEM;
		aiao->packed_bytes = buffer_bytes;
		dma_addr = aiao->packed_addr;
	}

	if (spdif) {
		/* The SPDIF-TX bank only has the channel-count and precision
		 * fields; there is no SD source mux.
		 */
		if_attr = FIELD_PREP(AIAO_TX_PRECISION_MASK, precision) |
			  FIELD_PREP(AIAO_TX_CHANNELS_MASK, 1);
		/* 256 fs MCLK, BCLK = 128 fs (biphase cell rate), frame = 128
		 * BCLK; index values follow hal_aiao_priv.c:479-492/464-477.
		 */
		crg1 = FIELD_PREP(AIAO_CRG_BCLK_DIV_MASK,
				  AIAO_CRG_BCLK_DIV_128FS) |
		       FIELD_PREP(AIAO_CRG_FSCLK_DIV_MASK, AIAO_CRG_FS_DIV_128FS);
	} else {
		if_attr = FIELD_PREP(AIAO_TX_PRECISION_MASK, precision) |
			  FIELD_PREP(AIAO_TX_CHANNELS_MASK, hw_channels == 8 ? 3 : 1) |
			  FIELD_PREP(AIAO_TX_SOURCE_MASK, aiao->tx_channel) |
			  FIELD_PREP(AIAO_TX_SD0_MASK, 0) |
			  FIELD_PREP(AIAO_TX_SD1_MASK, 1) |
			  FIELD_PREP(AIAO_TX_SD2_MASK, 2) |
			  FIELD_PREP(AIAO_TX_SD3_MASK, 3);
		crg1 = FIELD_PREP(AIAO_CRG_BCLK_DIV_MASK,
				  AIAO_CRG_BCLK_DIV_64FS) |
		       FIELD_PREP(AIAO_CRG_FSCLK_DIV_MASK, AIAO_CRG_FS_DIV_64FS);
	}

	memset(config, 0, sizeof(*config));
	config->crg0 = FIELD_PREP(AIAO_CRG_MCLK_DIV_MASK,
				  aiao_mclk_div(aiao, params_rate(params)));
	config->crg1 = crg1;
	config->if_attr = if_attr;
	config->dma_addr = lower_32_bits(dma_addr);
	config->buffer_bytes = buffer_bytes;
	config->period_bytes = period_bytes;
	config->channel = aiao->tx_channel;
	config->spdif_pad = aiao->spdif_pad;
	config->rate = params_rate(params);
	config->sample_bits = precision == 1 ? 16 : 24;
	config->spdif = spdif;
	config->frame_bytes = hw_channels * sample_bytes;
	config->hdmi_bypass = !spdif && aiao->hdmi_stream &&
			      aiao->hdmi_tx_channel == 3;
	spin_lock_irqsave(&aiao->reg_lock, flags);
	aiao_build_cs(aiao, config, config->cs);
	spin_unlock_irqrestore(&aiao->reg_lock, flags);
	config->dsp_ctrl = aiao_dsp_ctrl_value(config);
	config->valid = true;

	aiao->prepared = false;
	aiao->last_pos = 0;
	aiao_restore_tx_config(aiao);

	dev_dbg(aiao->dev, "AIAO TX%u%s: %u Hz, %u significant bits%s\n",
		aiao->tx_channel, spdif ? " (S/PDIF)" : "",
		params_rate(params), precision == 1 ? 16 : 24,
		config->cs[0] & IEC958_AES0_NONAUDIO ? ", non-audio" : "");
	return 0;
}

static int hi3798cv200_aiao_hw_free(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	int ret;

	ret = aiao_stop_tx(aiao);
	if (ret)
		return ret;
	aiao->prepared = false;
	aiao->config.valid = false;
	if (aiao->packed_area) {
		dma_free_coherent(aiao->dev, aiao->packed_bytes,
				  aiao->packed_area, aiao->packed_addr);
		aiao->packed_area = NULL;
	}
	if (READ_ONCE(aiao->powered)) {
		aiao_write(aiao, aiao_tx_crg_reg(aiao, AIAO_CRG_CFG1), 0);
		aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX_BUF_ADDR), 0);
		aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX_BUF_SIZE), 0);
		if (aiao->spdif_stream)
			aiao_write(aiao, aiao_encoder_base(aiao->tx_channel) +
				   AIAO_SPDIF_CTRL, 0);
	}
	return ret;
}

static int hi3798cv200_aiao_prepare(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	int ret;

	if (!READ_ONCE(aiao->powered))
		return -ESTRPIPE;
	if (!aiao->config.valid)
		return -EINVAL;

	ret = aiao_stop_tx(aiao);
	if (ret)
		return ret;
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX_DSP_CTRL), aiao->config.dsp_ctrl);
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX_BUF_WPTR), 0);
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX_BUF_RPTR), 0);
	aiao->last_pos = 0;
	aiao->packed_appl = 0;
	aiao->prepared = true;
	return 0;
}

static int hi3798cv200_aiao_trigger(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream,
				    int cmd)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	u32 tx_int_clr = aiao_tx_reg(aiao, AIAO_TX_INT_CLR);
	u32 tx_int_ena = aiao_tx_reg(aiao, AIAO_TX_INT_ENA);
	u32 tx_dsp = aiao_tx_reg(aiao, AIAO_TX_DSP_CTRL);
	u32 top_irq = aiao_tx_top_irq(aiao);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (!READ_ONCE(aiao->powered) || !aiao->prepared)
			return -ESTRPIPE;
		rcu_assign_pointer(aiao->substream, substream);
		aiao_write(aiao, tx_int_clr, AIAO_TX_INT_ALL);
		aiao_write(aiao, tx_int_ena,
			   AIAO_TX_INT_PERIOD | AIAO_TX_INT_EMPTY);
		aiao_update_bits(aiao, AIAO_INT_ENA, top_irq, top_irq);
		dma_wmb();
		aiao_update_bits(aiao, tx_dsp, AIAO_TX_ENABLE, AIAO_TX_ENABLE);
		if (aiao->spdif_stream) {
			u32 enc = aiao_encoder_base(aiao->tx_channel);

			/* Vendor AIAO_HW_SetStart SPDIF branch: channel
			 * numbers are rewritten on every start, then the
			 * biphase encoder is released.
			 */
			aiao_write(aiao, enc + AIAO_SPDIF_CS3,
				   aiao_cs3_value(aiao->config.cs));
			aiao_write(aiao, enc + AIAO_SPDIF_CTRL, AIAO_SPDIF_EN);
		}
		return 0;

	case SNDRV_PCM_TRIGGER_RESUME:
		/* FIFO contents were lost; only prepare/refill can restart safely. */
		return -ENOSYS;

	case SNDRV_PCM_TRIGGER_SUSPEND:
		aiao->prepared = false;
		return aiao_stop_tx(aiao);

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		return aiao_stop_tx(aiao);

	default:
		return -EINVAL;
	}
}

/* Both PCM callbacks run with the ALSA stream lock held. */
static int hi3798cv200_aiao_publish(struct snd_soc_component *component,
					struct snd_pcm_substream *substream,
					u32 rptr)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	struct snd_pcm_runtime *runtime = substream->runtime;
	size_t buffer_bytes = aiao->config.buffer_bytes;
	unsigned int stride = aiao->config.frame_bytes;
	snd_pcm_sframes_t queued_frames;
	size_t hw_pos, advanced, queued_bytes, pos;

	if (!buffer_bytes)
		return 0;
	if (rptr >= buffer_bytes)
		return -EIO;
	/* pointer and ack share ALSA's stream lock. Only ack copies new
	 * six-channel frames; a pointer-only update must not expose an
	 * application pointer that has not yet reached the physical ring.
	 */
	if (aiao->packed_area &&
	    READ_ONCE(runtime->control->appl_ptr) != aiao->packed_appl)
		return 0;
	hw_pos = (READ_ONCE(runtime->status->hw_ptr) % runtime->buffer_size) * stride;
	advanced = (rptr + buffer_bytes - hw_pos) % buffer_bytes;
	queued_frames = snd_pcm_playback_hw_avail(runtime) - advanced / stride;
	if (queued_frames < 0 ||
	    queued_frames > (snd_pcm_sframes_t)runtime->buffer_size)
		return -EPIPE;
	queued_bytes = queued_frames * stride;
	pos = (READ_ONCE(runtime->control->appl_ptr) % runtime->buffer_size) * stride;
	if (queued_bytes > buffer_bytes - AIAO_DMA_GAP)
		pos = (pos + buffer_bytes -
		       (queued_bytes - (buffer_bytes - AIAO_DMA_GAP))) %
		      buffer_bytes;
	dma_wmb();
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX_BUF_WPTR), pos);
	/* EMPTY is a level condition. Rearm it only when scanout has more data;
	 * ALSA still decides genuine underrun versus normal final-buffer drain.
	 */
	if (queued_bytes &&
	    (aiao_read(aiao, aiao_tx_reg(aiao, AIAO_TX_DSP_CTRL)) & AIAO_TX_ENABLE))
		aiao_update_bits(aiao, aiao_tx_reg(aiao, AIAO_TX_INT_ENA),
				 AIAO_TX_INT_EMPTY, AIAO_TX_INT_EMPTY);
	return 0;
}

static snd_pcm_uframes_t
hi3798cv200_aiao_pointer(struct snd_soc_component *component,
				struct snd_pcm_substream *substream)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	u32 pos = aiao->last_pos;

	if (!READ_ONCE(aiao->powered) || !aiao->prepared)
		return aiao->config.frame_bytes ? pos / aiao->config.frame_bytes : 0;
	pos = aiao_read(aiao, aiao_tx_reg(aiao, AIAO_TX_BUF_RPTR)) &
		AIAO_BUFFER_PTR_MASK;

	if (hi3798cv200_aiao_publish(component, substream, pos))
		return SNDRV_PCM_POS_XRUN;
	aiao->last_pos = pos;
	return pos / aiao->config.frame_bytes;
}

static int hi3798cv200_aiao_ack(struct snd_soc_component *component,
				struct snd_pcm_substream *substream)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	u32 rptr;
	snd_pcm_uframes_t appl = READ_ONCE(substream->runtime->control->appl_ptr);
	snd_pcm_uframes_t frames, offset, count;
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (!READ_ONCE(aiao->powered) || !aiao->prepared)
		return -ESTRPIPE;
	if (aiao->packed_area) {
		frames = appl >= aiao->packed_appl ? appl - aiao->packed_appl :
			appl + runtime->boundary - aiao->packed_appl;
		if (frames > runtime->buffer_size)
			return -EPIPE;
		offset = aiao->packed_appl % runtime->buffer_size;
		count = min(frames, runtime->buffer_size - offset);
		aiao_expand_6ch(aiao->packed_area + offset * aiao->config.frame_bytes,
				runtime->dma_area + frames_to_bytes(runtime, offset),
				count, runtime->sample_bits / 8);
		if (frames > count)
			aiao_expand_6ch(aiao->packed_area, runtime->dma_area,
					frames - count, runtime->sample_bits / 8);
		aiao->packed_appl = appl;
	}
	rptr = aiao_read(aiao, aiao_tx_reg(aiao, AIAO_TX_BUF_RPTR)) &
		AIAO_BUFFER_PTR_MASK;

	return hi3798cv200_aiao_publish(component, substream, rptr);
}

/* IEC958 control.  It belongs to the S/PDIF PCM device so that alsa-lib's
 * iec958 plugin and applications using AES0..AES3 find it on the same
 * device number they opened.
 */
static int hi3798cv200_aiao_iec958_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;
	return 0;
}

static int hi3798cv200_aiao_iec958_get(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct hi3798cv200_aiao *aiao = snd_kcontrol_chip(kcontrol);
	unsigned long flags;

	spin_lock_irqsave(&aiao->reg_lock, flags);
	memcpy(ucontrol->value.iec958.status, aiao->iec958,
	       sizeof(aiao->iec958));
	spin_unlock_irqrestore(&aiao->reg_lock, flags);
	return 0;
}

static int hi3798cv200_aiao_iec958_put(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct hi3798cv200_aiao *aiao = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	bool changed;

	spin_lock_irqsave(&aiao->reg_lock, flags);
	changed = memcmp(aiao->iec958, ucontrol->value.iec958.status,
			 sizeof(aiao->iec958));
	if (changed) {
		memcpy(aiao->iec958, ucontrol->value.iec958.status,
		       sizeof(aiao->iec958));
		aiao->iec958_valid = true;
	}
	spin_unlock_irqrestore(&aiao->reg_lock, flags);

	if (changed)
		aiao_update_spdif_output(aiao);
	return changed;
}

static int hi3798cv200_aiao_iec958_mask_get(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	memset(ucontrol->value.iec958.status, 0xff, 4);
	return 0;
}

static const struct snd_kcontrol_new hi3798cv200_aiao_iec958_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, DEFAULT),
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = hi3798cv200_aiao_iec958_info,
		.get = hi3798cv200_aiao_iec958_get,
		.put = hi3798cv200_aiao_iec958_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, MASK),
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = hi3798cv200_aiao_iec958_info,
		.get = hi3798cv200_aiao_iec958_mask_get,
	},
};

static int hi3798cv200_aiao_pcm_construct(struct snd_soc_component *component,
					  struct snd_soc_pcm_runtime *rtd)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_card *card = rtd->card->snd_card;
	struct snd_kcontrol *kcontrol;
	int ret, i;

	ret = snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV,
					     component->dev, 64 * 1024,
					     AIAO_BUFFER_BYTES_MAX);
	if (ret)
		return ret;

	if (cpu_dai->id != 2)
		return 0;

	for (i = 0; i < ARRAY_SIZE(hi3798cv200_aiao_iec958_controls); i++) {
		kcontrol = snd_ctl_new1(&hi3798cv200_aiao_iec958_controls[i],
					aiao);
		if (!kcontrol)
			return -ENOMEM;
		kcontrol->id.device = rtd->pcm->device;
		ret = snd_ctl_add(card, kcontrol);
		if (ret)
			return ret;
	}
	dev_dbg(component->dev, "IEC958 controls on PCM device %u\n",
		rtd->pcm->device);
	return 0;
}

static int hi3798cv200_aiao_component_suspend(struct snd_soc_component *component)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);

	/* ASoC has already suspended the PCM, including a paused/prepared one. */
	aiao->prepared = false;
	return aiao_stop_tx(aiao);
}

static int hi3798cv200_aiao_component_resume(struct snd_soc_component *component)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);

	/* Device resume_early restored configuration, never the playback queue. */
	return READ_ONCE(aiao->powered) ? 0 : -EIO;
}

static const struct snd_soc_component_driver hi3798cv200_aiao_component = {
	.name = "hi3798cv200-aiao",
	.suspend = hi3798cv200_aiao_component_suspend,
	.resume = hi3798cv200_aiao_component_resume,
	.open = hi3798cv200_aiao_open,
	.close = hi3798cv200_aiao_close,
	.sync_stop = hi3798cv200_aiao_sync_stop,
	.hw_params = hi3798cv200_aiao_hw_params,
	.hw_free = hi3798cv200_aiao_hw_free,
	.prepare = hi3798cv200_aiao_prepare,
	.trigger = hi3798cv200_aiao_trigger,
	.pointer = hi3798cv200_aiao_pointer,
	.ack = hi3798cv200_aiao_ack,
	.pcm_construct = hi3798cv200_aiao_pcm_construct,
};

static int hi3798cv200_aiao_set_fmt(struct snd_soc_dai *dai,
				   unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S)
		return -EINVAL;

	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) !=
	    SND_SOC_DAIFMT_BP_FP)
		return -EINVAL;

	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;

	return 0;
}

static int hi3798cv200_aiao_spdif_set_fmt(struct snd_soc_dai *dai,
					  unsigned int fmt)
{
	/* S/PDIF is a fixed biphase-mark transmitter: it has no I2S-style
	 * frame format, clock polarity or provider role.  The machine driver
	 * may copy an "i2s" property from another link; accept any value
	 * instead of failing card instantiation.
	 */
	return 0;
}

static const unsigned int aiao_hdmi_channels[] = { 2, 6, 8 };
static const struct snd_pcm_hw_constraint_list aiao_hdmi_channel_list = {
	.count = ARRAY_SIZE(aiao_hdmi_channels),
	.list = aiao_hdmi_channels,
};

static int hi3798cv200_aiao_dai_startup(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	struct hi3798cv200_aiao *aiao = snd_soc_dai_get_drvdata(dai);

	switch (dai->id) {
	case 0:
		aiao->tx_channel = aiao->analog_tx_channel;
		aiao->hdmi_stream = false;
		aiao->spdif_stream = false;
		break;
	case 1:
		aiao->tx_channel = aiao->hdmi_tx_channel;
		aiao->hdmi_stream = true;
		aiao->spdif_stream = false;
		if (aiao->hdmi_tx_channel == 3)
			return snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_CHANNELS, &aiao_hdmi_channel_list);
		break;
	case 2:
		aiao->tx_channel = aiao->spdif_tx_channel;
		aiao->hdmi_stream = false;
		aiao->spdif_stream = true;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct snd_soc_dai_ops hi3798cv200_aiao_dai_ops = {
	.startup = hi3798cv200_aiao_dai_startup,
	.set_fmt = hi3798cv200_aiao_set_fmt,
};

static const struct snd_soc_dai_ops hi3798cv200_aiao_spdif_dai_ops = {
	.startup = hi3798cv200_aiao_dai_startup,
	.set_fmt = hi3798cv200_aiao_spdif_set_fmt,
};

static struct snd_soc_dai_driver hi3798cv200_aiao_dais[] = {
	{
		.id = 0,
		.name = "hi3798-aiao-analog",
		.playback = {
			.stream_name = "Analog Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = AIAO_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &hi3798cv200_aiao_dai_ops,
	}, {
		.id = 1,
		.name = "hi3798-aiao-hdmi",
		.playback = {
			.stream_name = "HDMI Playback",
			.channels_min = 2,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &hi3798cv200_aiao_dai_ops,
	}, {
		.id = 2,
		.name = "hi3798-aiao-spdif",
		.playback = {
			.stream_name = "S/PDIF Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = AIAO_SPDIF_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &hi3798cv200_aiao_spdif_dai_ops,
	},
};

static bool hi3798cv200_aiao_ack_tx0(struct hi3798cv200_aiao *aiao,
					     u32 raw)
{
	u32 top_raw = 0, top_status = 0, status = 0;
	unsigned int attempt;
	u32 tx_int_clr = aiao_tx_reg(aiao, AIAO_TX_INT_CLR);
	u32 tx_int_raw = aiao_tx_reg(aiao, AIAO_TX_INT_RAW);
	u32 tx_int_status = aiao_tx_reg(aiao, AIAO_TX_INT_STATUS);
	u32 tx_int_ena = aiao_tx_reg(aiao, AIAO_TX_INT_ENA);
	u32 top_irq = aiao_tx_top_irq(aiao);

	/* An empty ring immediately reasserts EMPTY after W1C. Consume that
	 * notification once, then let period_elapsed handle drain/XRUN and
	 * publish() rearm it when another DMA range becomes available.
	 */
	if (raw & AIAO_TX_INT_EMPTY)
		aiao_update_bits(aiao, tx_int_ena, AIAO_TX_INT_EMPTY, 0);
	for (attempt = 0; attempt < AIAO_IRQ_ACK_RETRIES; attempt++) {
		if (raw)
			aiao_write(aiao, tx_int_clr, raw);
		raw = aiao_read(aiao, tx_int_raw);
		status = aiao_read(aiao, tx_int_status);
		top_raw = aiao_read(aiao, AIAO_INT_RAW);
		top_status = aiao_read(aiao, AIAO_INT_STATUS);
		if (!(status & AIAO_TX_INT_ALL) && !(top_status & top_irq))
			return true;
	}

	aiao_write(aiao, tx_int_ena, 0);
	aiao_update_bits(aiao, AIAO_INT_ENA, top_irq, 0);
	dev_err_ratelimited(aiao->dev,
				   "AIAO TX%u IRQ stayed active after %u acks; masked top=%08x/%08x tx=%08x/%08x\n",
				   aiao->tx_channel, AIAO_IRQ_ACK_RETRIES,
				   top_raw, top_status, raw, status);
	return false;
}

static irqreturn_t hi3798cv200_aiao_irq(int irq, void *data)
{
	struct hi3798cv200_aiao *aiao = data;
	struct snd_pcm_substream *substream;
	u32 top, raw, unknown;
	bool handled = false, acked;
	u32 top_irq = aiao_tx_top_irq(aiao);

	if (!READ_ONCE(aiao->powered))
		return IRQ_NONE;

	top = aiao_read(aiao, AIAO_INT_STATUS);
	if (!top)
		return IRQ_NONE;
	if (top & top_irq) {
		handled = true;
		raw = aiao_read(aiao, aiao_tx_reg(aiao, AIAO_TX_INT_RAW));
		acked = hi3798cv200_aiao_ack_tx0(aiao, raw);
		if (!acked || (raw & (AIAO_TX_INT_PERIOD | AIAO_TX_INT_EMPTY))) {
			rcu_read_lock();
			substream = rcu_dereference(aiao->substream);
			if (substream && snd_pcm_running(substream)) {
				if (acked)
					snd_pcm_period_elapsed(substream);
				else
					snd_pcm_stop_xrun(substream);
			}
			rcu_read_unlock();
		}
	}

	unknown = top & ~top_irq;
	if (unknown) {
		aiao_update_bits(aiao, AIAO_INT_ENA, unknown, 0);
		dev_warn_ratelimited(aiao->dev,
					     "masked unowned AIAO top interrupt 0x%08x\n",
					     unknown);
		handled = true;
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int aiao_power_on(struct hi3798cv200_aiao *aiao)
{
	u32 val;
	int ret, reset_ret;

	if (READ_ONCE(aiao->powered))
		return 0;

	/* CRG restores its parent selectors before device resume_early. */
	ret = clk_set_rate(aiao->bus_clk, 50000000);
	if (ret)
		return ret;
	ret = clk_set_rate(aiao->mclk, aiao->mclk_rate);
	if (ret)
		return ret;
	if (clk_get_rate(aiao->mclk) != aiao->mclk_rate)
		return -ERANGE;

	ret = reset_control_assert(aiao->reset);
	if (ret)
		return ret;
	ret = clk_prepare_enable(aiao->mclk);
	if (ret)
		return ret;
	ret = clk_prepare_enable(aiao->bus_clk);
	if (ret) {
		clk_disable_unprepare(aiao->mclk);
		return ret;
	}

	udelay(1);
	ret = reset_control_deassert(aiao->reset);
	if (ret)
		goto assert_reset;
	ret = readl_poll_timeout(aiao->base + AIAO_STATUS, val, val & BIT(0),
				 1, 1000);
	if (ret) {
		dev_err(aiao->dev, "controller did not leave reset\n");
		goto assert_reset;
	}

	aiao_update_bits(aiao, AIAO_OUTSTANDING, AIAO_OUTSTANDING_MASK, 2);
	aiao_write(aiao, AIAO_INT_ENA, 0);
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX_INT_ENA), 0);
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX_INT_CLR), AIAO_TX_INT_ALL);
	aiao_restore_tx_config(aiao);
	aiao->stop_error = 0;
	WRITE_ONCE(aiao->powered, true);
	return 0;

assert_reset:
	reset_ret = reset_control_assert(aiao->reset);
	if (reset_ret)
		dev_err(aiao->dev, "failed to reassert reset: %d\n", reset_ret);
	clk_disable_unprepare(aiao->bus_clk);
	clk_disable_unprepare(aiao->mclk);
	return ret;
}

static int hi3798cv200_aiao_suspend_late(struct device *dev)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(dev);
	int ret;

	if (!READ_ONCE(aiao->powered))
		return 0;
	/* An ignore_suspend link must not silently lose an active DMA stream. */
	if (rcu_access_pointer(aiao->substream))
		return -EBUSY;

	/* All cards have completed their normal ASoC/PCM suspend callbacks. */
	aiao->prepared = false;
	ret = aiao_stop_tx(aiao);
	if (ret)
		return ret;

	/* Never wait for period_elapsed while holding reg_lock/ALSA's lock. */
	WRITE_ONCE(aiao->powered, false);
	synchronize_irq(aiao->irq);
	synchronize_rcu();
	ret = reset_control_assert(aiao->reset);
	if (ret) {
		WRITE_ONCE(aiao->powered, true);
		return ret;
	}
	clk_disable_unprepare(aiao->bus_clk);
	clk_disable_unprepare(aiao->mclk);
	return 0;
}

static int hi3798cv200_aiao_resume_early(struct device *dev)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(dev);
	int ret;

	/* Restore before the card's deferred component resume, never start DMA. */
	ret = aiao_power_on(aiao);
	if (ret)
		dev_err(dev, "failed to restore AIAO: %d\n", ret);
	return ret;
}

static const struct dev_pm_ops hi3798cv200_aiao_pm_ops = {
	LATE_SYSTEM_SLEEP_PM_OPS(hi3798cv200_aiao_suspend_late,
				hi3798cv200_aiao_resume_early)
};

static void hi3798cv200_aiao_disable(void *data)
{
	struct hi3798cv200_aiao *aiao = data;
	int ret;

	if (!READ_ONCE(aiao->powered))
		return;
	aiao_stop_tx(aiao);
	WRITE_ONCE(aiao->powered, false);
	synchronize_irq(aiao->irq);
	synchronize_rcu();
	ret = reset_control_assert(aiao->reset);
	if (ret)
		dev_err(aiao->dev, "failed to assert reset on removal: %d\n", ret);
	clk_disable_unprepare(aiao->bus_clk);
	clk_disable_unprepare(aiao->mclk);
}

static int hi3798cv200_aiao_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hi3798cv200_aiao *aiao;
	int ret;

	aiao = devm_kzalloc(dev, sizeof(*aiao), GFP_KERNEL);
	if (!aiao)
		return -ENOMEM;
	aiao->dev = dev;
	spin_lock_init(&aiao->reg_lock);
	aiao->tx_channel = 0;
	ret = device_property_read_u32(dev, "hisilicon,tx-channel",
					      &aiao->tx_channel);
	if (ret && ret != -EINVAL)
		return dev_err_probe(dev, ret, "failed to read TX channel\n");
	if (aiao->tx_channel > 3)
		return dev_err_probe(dev, -EINVAL, "invalid TX channel %u\n",
				     aiao->tx_channel);

	aiao->analog_tx_channel = aiao->tx_channel;
	/* BSP drv_ao_op.c:64-78: S40/CV200 HDMI is TX3, S28/MV310 TX2. */
	aiao->hdmi_tx_channel = of_device_is_compatible(dev->of_node,
						      "hisilicon,hi3798mv310-aiao") ? 2 : 3;
	/* BSP drv_ao_op.c:83 maps the logical consumer S/PDIF output to
	 * SPDIF TX1; boards that route TX0/TX2 override this.
	 */
	aiao->spdif_tx_channel = 1;
	ret = device_property_read_u32(dev, "hisilicon,spdif-tx-channel",
				       &aiao->spdif_tx_channel);
	if (ret && ret != -EINVAL)
		return dev_err_probe(dev, ret, "failed to read S/PDIF TX channel\n");
	if (aiao->spdif_tx_channel > 2)
		return dev_err_probe(dev, -EINVAL,
				     "invalid S/PDIF TX channel %u\n",
				     aiao->spdif_tx_channel);
	aiao->spdif_pad = aiao->spdif_tx_channel;
	ret = device_property_read_u32(dev, "hisilicon,spdif-tx-pad",
				       &aiao->spdif_pad);
	if (ret && ret != -EINVAL)
		return dev_err_probe(dev, ret, "failed to read S/PDIF pad\n");
	if (aiao->spdif_pad > 3)
		return dev_err_probe(dev, -EINVAL, "invalid S/PDIF pad %u\n",
				     aiao->spdif_pad);

	aiao->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(aiao->base))
		return PTR_ERR(aiao->base);

	aiao->bus_clk = devm_clk_get(dev, "bus");
	if (IS_ERR(aiao->bus_clk))
		return dev_err_probe(dev, PTR_ERR(aiao->bus_clk),
				     "failed to get bus clock\n");

	aiao->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(aiao->mclk))
		return dev_err_probe(dev, PTR_ERR(aiao->mclk),
				     "failed to get master clock\n");

	aiao->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(aiao->reset))
		return dev_err_probe(dev, PTR_ERR(aiao->reset),
				     "failed to get reset\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	aiao->mclk_rate = of_device_is_compatible(dev->of_node,
						 "hisilicon,hi3798mv310-aiao") ?
		860160000 : AIAO_EPLL_RATE;
	aiao->irq = platform_get_irq(pdev, 0);
	if (aiao->irq < 0)
		return aiao->irq;

	/* Power on before registering the component so the first PCM open sees
	 * a live controller; register-level state needs no clock here.
	 */
	snd_pcm_create_iec958_consumer_default(aiao->iec958,
					       sizeof(aiao->iec958));

	ret = aiao_power_on(aiao);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power on AIAO\n");

	platform_set_drvdata(pdev, aiao);
	ret = devm_add_action_or_reset(dev, hi3798cv200_aiao_disable, aiao);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, aiao->irq, hi3798cv200_aiao_irq,
			       IRQF_SHARED,
			       dev_name(dev), aiao);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	return devm_snd_soc_register_component(dev,
					       &hi3798cv200_aiao_component,
					       hi3798cv200_aiao_dais,
					       ARRAY_SIZE(hi3798cv200_aiao_dais));
}

static const struct of_device_id hi3798cv200_aiao_of_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-aiao" },
	{ .compatible = "hisilicon,hi3798mv310-aiao" },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3798cv200_aiao_of_match);

static struct platform_driver hi3798cv200_aiao_driver = {
	.probe = hi3798cv200_aiao_probe,
	.driver = {
		.name = "hi3798cv200-aiao",
		.of_match_table = hi3798cv200_aiao_of_match,
		.pm = pm_sleep_ptr(&hi3798cv200_aiao_pm_ops),
	},
};
module_platform_driver(hi3798cv200_aiao_driver);

MODULE_DESCRIPTION("HiSilicon Hi3798CV200/Hi3798MV310 AIAO playback driver");
MODULE_LICENSE("GPL");
