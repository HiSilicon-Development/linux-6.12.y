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
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rcupdate.h>
#include <linux/reset.h>
#include <linux/spinlock.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define AIAO_INT_ENA			0x0000
#define AIAO_INT_STATUS			0x0004
#define AIAO_INT_RAW			0x0008
#define AIAO_SWITCH_TX_BCLK		0x002c
#define  AIAO_TX0_BCLK_SOURCE_MASK	GENMASK(3, 0)
#define  AIAO_TX0_BCLK_SOURCE_TX0	8
#define AIAO_STATUS			0x0030
#define AIAO_OUTSTANDING		0x0034
#define  AIAO_OUTSTANDING_MASK		GENMASK(2, 0)

#define AIAO_TX0_CRG_CFG0		0x0140
#define AIAO_TX0_CRG_CFG1		0x0144
#define  AIAO_CRG_MCLK_DIV_MASK		GENMASK(26, 0)
#define  AIAO_CRG_BCLK_DIV_MASK		GENMASK(3, 0)
#define  AIAO_CRG_FSCLK_DIV_MASK	GENMASK(6, 4)
#define  AIAO_CRG_CLK_EN		BIT(8)

#define AIAO_TX0_IF_ATTR		0x2000
#define  AIAO_TX_PRECISION_MASK		GENMASK(3, 2)
#define  AIAO_TX_CHANNELS_MASK		GENMASK(5, 4)
#define  AIAO_TX_SOURCE_MASK		GENMASK(23, 20)
#define  AIAO_TX_SD0_MASK		GENMASK(25, 24)
#define  AIAO_TX_SD1_MASK		GENMASK(27, 26)
#define  AIAO_TX_SD2_MASK		GENMASK(29, 28)
#define  AIAO_TX_SD3_MASK		GENMASK(31, 30)
#define AIAO_TX0_DSP_CTRL		0x2004
#define  AIAO_TX_MUTE_FADE		BIT(1)
#define  AIAO_TX_VOLUME_MASK		GENMASK(14, 8)
#define  AIAO_TX_FADE_IN_MASK		GENMASK(19, 16)
#define  AIAO_TX_FADE_OUT_MASK		GENMASK(23, 20)
#define  AIAO_TX_ENABLE		BIT(28)
#define  AIAO_TX_DISABLE_DONE		BIT(29)

#define AIAO_TX0_BUF_ADDR		0x2080
#define AIAO_TX0_BUF_SIZE		0x2084
#define AIAO_TX0_BUF_WPTR		0x2088
#define AIAO_TX0_BUF_RPTR		0x208c
#define AIAO_TX0_BUF_ALEMPTY		0x2090
#define AIAO_TX0_TRANS_SIZE		0x2094
#define AIAO_TX0_INT_ENA		0x20a0
#define AIAO_TX0_INT_RAW		0x20a4
#define AIAO_TX0_INT_STATUS		0x20a8
#define AIAO_TX0_INT_CLR		0x20ac
#define  AIAO_TX_INT_PERIOD		BIT(0)
#define  AIAO_TX_INT_EMPTY		BIT(1)
#define  AIAO_TX_INT_ALL		GENMASK(7, 0)

#define AIAO_TOP_TX0_INT		BIT(16)
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
	u32 dsp_ctrl;
	/* Selected by the board's audio routing. */
	unsigned int tx_channel;
	unsigned int analog_tx_channel;
	unsigned int hdmi_tx_channel;
	int irq;
	atomic_t opened;
};

static inline u32 aiao_tx_reg(const struct hi3798cv200_aiao *aiao, u32 tx0_reg)
{
	return tx0_reg + aiao->tx_channel * 0x100;
}

static inline u32 aiao_tx_crg_reg(const struct hi3798cv200_aiao *aiao,
					  u32 tx0_reg)
{
	return tx0_reg + aiao->tx_channel * 8;
}

static inline u32 aiao_tx_top_irq(const struct hi3798cv200_aiao *aiao)
{
	return BIT(16 + aiao->tx_channel);
}

static inline u32 aiao_tx_bclk_mask(const struct hi3798cv200_aiao *aiao)
{
	return GENMASK(aiao->tx_channel * 4 + 3, aiao->tx_channel * 4);
}

static inline u32 aiao_tx_bclk_source(const struct hi3798cv200_aiao *aiao)
{
	return 8 + aiao->tx_channel;
}
static inline u32 aiao_tx_bclk_value(const struct hi3798cv200_aiao *aiao)
{
	return aiao_tx_bclk_source(aiao) << (aiao->tx_channel * 4);
}

static const struct snd_pcm_hardware hi3798cv200_aiao_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME |
		SNDRV_PCM_INFO_NO_REWINDS,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.rates = AIAO_RATES,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = 2,
	.channels_max = 2,
	.period_bytes_min = AIAO_BUFFER_ALIGN,
	.period_bytes_max = 64 * 1024,
	.periods_min = 2,
	.periods_max = 128,
	.buffer_bytes_max = AIAO_BUFFER_BYTES_MAX,
};

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

static int aiao_stop_tx(struct hi3798cv200_aiao *aiao)
{
	u32 val, trans_size, rptr;
	int ret = 0;
	const u32 tx_dsp = aiao_tx_reg(aiao, AIAO_TX0_DSP_CTRL);
	const u32 tx_rptr = aiao_tx_reg(aiao, AIAO_TX0_BUF_RPTR);
	const u32 tx_trans = aiao_tx_reg(aiao, AIAO_TX0_TRANS_SIZE);
	const u32 tx_int_ena = aiao_tx_reg(aiao, AIAO_TX0_INT_ENA);
	const u32 tx_int_clr = aiao_tx_reg(aiao, AIAO_TX0_INT_CLR);
	const u32 top_irq = aiao_tx_top_irq(aiao);

	/* Stop delivery before the disable handshake can empty the FIFO. */
	aiao_write(aiao, tx_int_ena, 0);
	aiao_update_bits(aiao, AIAO_INT_ENA, top_irq, 0);
	trans_size = aiao_read(aiao, tx_trans);
	val = aiao_read(aiao, tx_dsp);
	if (val & AIAO_TX_ENABLE) {
		aiao_update_bits(aiao, tx_dsp, AIAO_TX_ENABLE, 0);
		rptr = aiao_read(aiao, tx_rptr) & AIAO_BUFFER_PTR_MASK;
		aiao_write(aiao, tx_trans, 1);
		aiao_write(aiao, tx_rptr, 0);
		udelay(500);
		ret = readl_poll_timeout_atomic(aiao->base + tx_dsp, val,
						val & AIAO_TX_DISABLE_DONE, 10, 1000);
		if (ret)
			dev_warn_ratelimited(aiao->dev,
					     "AIAO TX%u stop timed out\n", aiao->tx_channel);
		if (!ret)
			aiao_write(aiao, tx_rptr, rptr);
		if (trans_size)
			aiao_write(aiao, tx_trans, trans_size);
	}

	aiao_write(aiao, tx_int_clr, AIAO_TX_INT_ALL);
	rcu_assign_pointer(aiao->substream, NULL);
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


static int hi3798cv200_aiao_hw_params(struct snd_soc_component *component,
				      struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	struct snd_pcm_runtime *runtime = substream->runtime;
	u32 precision, if_attr, crg1;
	size_t buffer_bytes = params_buffer_bytes(params);
	size_t period_bytes = params_period_bytes(params);
	u32 tx_crg0, tx_crg1, tx_if, tx_dsp, tx_addr, tx_size;
	u32 tx_wptr, tx_rptr, tx_alempty, tx_trans;

	if (params_channels(params) != 2)
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

	if_attr = FIELD_PREP(AIAO_TX_PRECISION_MASK, precision) |
		  FIELD_PREP(AIAO_TX_CHANNELS_MASK, 1) |
		  FIELD_PREP(AIAO_TX_SOURCE_MASK, aiao->tx_channel) |
		  FIELD_PREP(AIAO_TX_SD0_MASK, 0) |
		  FIELD_PREP(AIAO_TX_SD1_MASK, 1) |
		  FIELD_PREP(AIAO_TX_SD2_MASK, 2) |
		  FIELD_PREP(AIAO_TX_SD3_MASK, 3);

	crg1 = FIELD_PREP(AIAO_CRG_BCLK_DIV_MASK, 3) |
	       FIELD_PREP(AIAO_CRG_FSCLK_DIV_MASK, 3);
	tx_crg0 = aiao_tx_crg_reg(aiao, AIAO_TX0_CRG_CFG0);
	tx_crg1 = aiao_tx_crg_reg(aiao, AIAO_TX0_CRG_CFG1);
	tx_if = aiao_tx_reg(aiao, AIAO_TX0_IF_ATTR);
	tx_dsp = aiao_tx_reg(aiao, AIAO_TX0_DSP_CTRL);
	tx_addr = aiao_tx_reg(aiao, AIAO_TX0_BUF_ADDR);
	tx_size = aiao_tx_reg(aiao, AIAO_TX0_BUF_SIZE);
	tx_wptr = aiao_tx_reg(aiao, AIAO_TX0_BUF_WPTR);
	tx_rptr = aiao_tx_reg(aiao, AIAO_TX0_BUF_RPTR);
	tx_alempty = aiao_tx_reg(aiao, AIAO_TX0_BUF_ALEMPTY);
	tx_trans = aiao_tx_reg(aiao, AIAO_TX0_TRANS_SIZE);

	aiao_update_bits(aiao, tx_crg1, AIAO_CRG_CLK_EN, 0);
	aiao_write(aiao, tx_crg0,
		   FIELD_PREP(AIAO_CRG_MCLK_DIV_MASK,
			      aiao_mclk_div(aiao, params_rate(params))));
	aiao_write(aiao, tx_crg1, crg1);
	aiao_write(aiao, tx_crg1, crg1 | AIAO_CRG_CLK_EN);
	aiao_update_bits(aiao, AIAO_SWITCH_TX_BCLK,
			 aiao_tx_bclk_mask(aiao), aiao_tx_bclk_value(aiao));
	aiao_write(aiao, tx_if, if_attr);

	aiao->dsp_ctrl = AIAO_TX_MUTE_FADE |
		FIELD_PREP(AIAO_TX_VOLUME_MASK, 0x79) |
		FIELD_PREP(AIAO_TX_FADE_IN_MASK, 4) |
		FIELD_PREP(AIAO_TX_FADE_OUT_MASK, 3);
	aiao_write(aiao, tx_dsp, aiao->dsp_ctrl);
	aiao_write(aiao, tx_addr, lower_32_bits(runtime->dma_addr));
	aiao_write(aiao, tx_size, buffer_bytes);
	aiao_write(aiao, tx_wptr, 0);
	aiao_write(aiao, tx_rptr, 0);
	aiao_write(aiao, tx_alempty, period_bytes);
	aiao_write(aiao, tx_trans, period_bytes);

	dev_dbg(aiao->dev, "AIAO TX%u: %u Hz, %u significant bits\n",
		aiao->tx_channel, params_rate(params), precision == 1 ? 16 : 24);
	return 0;
}

static int hi3798cv200_aiao_hw_free(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	int ret;

	ret = aiao_stop_tx(aiao);
	aiao_write(aiao, aiao_tx_crg_reg(aiao, AIAO_TX0_CRG_CFG1), 0);
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX0_BUF_ADDR), 0);
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX0_BUF_SIZE), 0);
	return ret;
}

static int hi3798cv200_aiao_prepare(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	int ret;

	ret = aiao_stop_tx(aiao);
	if (ret)
		return ret;
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX0_DSP_CTRL), aiao->dsp_ctrl);
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX0_BUF_WPTR), 0);
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX0_BUF_RPTR), 0);
	return 0;
}

static int hi3798cv200_aiao_trigger(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream,
				    int cmd)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	u32 tx_int_clr = aiao_tx_reg(aiao, AIAO_TX0_INT_CLR);
	u32 tx_int_ena = aiao_tx_reg(aiao, AIAO_TX0_INT_ENA);
	u32 tx_dsp = aiao_tx_reg(aiao, AIAO_TX0_DSP_CTRL);
	u32 top_irq = aiao_tx_top_irq(aiao);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		rcu_assign_pointer(aiao->substream, substream);
		aiao_write(aiao, tx_int_clr, AIAO_TX_INT_ALL);
		aiao_write(aiao, tx_int_ena,
			   AIAO_TX_INT_PERIOD | AIAO_TX_INT_EMPTY);
		aiao_update_bits(aiao, AIAO_INT_ENA, top_irq, top_irq);
		dma_wmb();
		aiao_update_bits(aiao, tx_dsp, AIAO_TX_ENABLE, AIAO_TX_ENABLE);
		return 0;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
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
	size_t buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
	snd_pcm_sframes_t queued_frames;
	size_t hw_pos, advanced, queued_bytes, pos;

	if (!buffer_bytes)
		return 0;
	if (rptr >= buffer_bytes)
		return -EIO;
	hw_pos = frames_to_bytes(runtime,
		READ_ONCE(runtime->status->hw_ptr) % runtime->buffer_size);
	advanced = (rptr + buffer_bytes - hw_pos) % buffer_bytes;
	queued_frames = snd_pcm_playback_hw_avail(runtime) -
		bytes_to_frames(runtime, advanced);
	if (queued_frames < 0 ||
	    queued_frames > (snd_pcm_sframes_t)runtime->buffer_size)
		return -EPIPE;
	queued_bytes = frames_to_bytes(runtime, queued_frames);
	pos = frames_to_bytes(runtime,
		READ_ONCE(runtime->control->appl_ptr) % runtime->buffer_size);
	if (queued_bytes > buffer_bytes - AIAO_DMA_GAP)
		pos = (pos + buffer_bytes -
		       (queued_bytes - (buffer_bytes - AIAO_DMA_GAP))) %
		      buffer_bytes;
	dma_wmb();
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX0_BUF_WPTR), pos);
	/* EMPTY is a level condition. Rearm it only when scanout has more data;
	 * ALSA still decides genuine underrun versus normal final-buffer drain.
	 */
	if (queued_bytes &&
	    (aiao_read(aiao, aiao_tx_reg(aiao, AIAO_TX0_DSP_CTRL)) & AIAO_TX_ENABLE))
		aiao_update_bits(aiao, aiao_tx_reg(aiao, AIAO_TX0_INT_ENA),
				 AIAO_TX_INT_EMPTY, AIAO_TX_INT_EMPTY);
	return 0;
}

static snd_pcm_uframes_t
hi3798cv200_aiao_pointer(struct snd_soc_component *component,
				struct snd_pcm_substream *substream)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	struct snd_pcm_runtime *runtime = substream->runtime;
	u32 pos = aiao_read(aiao, aiao_tx_reg(aiao, AIAO_TX0_BUF_RPTR)) &
		AIAO_BUFFER_PTR_MASK;

	if (hi3798cv200_aiao_publish(component, substream, pos))
		return SNDRV_PCM_POS_XRUN;
	return bytes_to_frames(runtime, pos);
}

static int hi3798cv200_aiao_ack(struct snd_soc_component *component,
				struct snd_pcm_substream *substream)
{
	struct hi3798cv200_aiao *aiao = dev_get_drvdata(component->dev);
	u32 rptr = aiao_read(aiao, aiao_tx_reg(aiao, AIAO_TX0_BUF_RPTR)) &
		AIAO_BUFFER_PTR_MASK;

	return hi3798cv200_aiao_publish(component, substream, rptr);
}

static int hi3798cv200_aiao_pcm_construct(struct snd_soc_component *component,
					  struct snd_soc_pcm_runtime *rtd)
{
	return snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV,
					      component->dev, 64 * 1024,
					      AIAO_BUFFER_BYTES_MAX);
}

static const struct snd_soc_component_driver hi3798cv200_aiao_component = {
	.name = "hi3798cv200-aiao",
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

static int hi3798cv200_aiao_dai_startup(struct snd_pcm_substream *substream,
				      struct snd_soc_dai *dai)
{
	struct hi3798cv200_aiao *aiao = snd_soc_dai_get_drvdata(dai);

	aiao->tx_channel = dai->id ? aiao->hdmi_tx_channel : aiao->analog_tx_channel;
	return 0;
}

static const struct snd_soc_dai_ops hi3798cv200_aiao_dai_ops = {
	.startup = hi3798cv200_aiao_dai_startup,
	.set_fmt = hi3798cv200_aiao_set_fmt,
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
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &hi3798cv200_aiao_dai_ops,
	},
};

static bool hi3798cv200_aiao_ack_tx0(struct hi3798cv200_aiao *aiao,
					     u32 raw)
{
	u32 top_raw = 0, top_status = 0, status = 0;
	unsigned int attempt;
	u32 tx_int_clr = aiao_tx_reg(aiao, AIAO_TX0_INT_CLR);
	u32 tx_int_raw = aiao_tx_reg(aiao, AIAO_TX0_INT_RAW);
	u32 tx_int_status = aiao_tx_reg(aiao, AIAO_TX0_INT_STATUS);
	u32 tx_int_ena = aiao_tx_reg(aiao, AIAO_TX0_INT_ENA);
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

	top = aiao_read(aiao, AIAO_INT_STATUS);
	if (!top)
		return IRQ_NONE;
	if (top & top_irq) {
		handled = true;
		raw = aiao_read(aiao, aiao_tx_reg(aiao, AIAO_TX0_INT_RAW));
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

static void hi3798cv200_aiao_disable(void *data)
{
	struct hi3798cv200_aiao *aiao = data;

	aiao_stop_tx(aiao);
	reset_control_assert(aiao->reset);
	clk_disable_unprepare(aiao->bus_clk);
	clk_disable_unprepare(aiao->mclk);
}

static int hi3798cv200_aiao_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hi3798cv200_aiao *aiao;
	u32 val;
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

	ret = clk_set_rate(aiao->bus_clk, 50000000);
	if (ret)
		return dev_err_probe(dev, ret, "failed to select 50 MHz bus clock\n");

	aiao->mclk_rate = of_device_is_compatible(dev->of_node,
						 "hisilicon,hi3798mv310-aiao") ?
		860160000 : AIAO_EPLL_RATE;
	ret = clk_set_rate(aiao->mclk, aiao->mclk_rate);
	if (ret)
		return dev_err_probe(dev, ret, "failed to select audio PLL clock\n");
	if (clk_get_rate(aiao->mclk) != aiao->mclk_rate)
		return dev_err_probe(dev, -ERANGE, "unexpected audio PLL rate\n");

	ret = reset_control_assert(aiao->reset);
	if (ret)
		return dev_err_probe(dev, ret, "failed to assert reset\n");

	ret = clk_prepare_enable(aiao->mclk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable master clock\n");

	ret = clk_prepare_enable(aiao->bus_clk);
	if (ret) {
		clk_disable_unprepare(aiao->mclk);
		return dev_err_probe(dev, ret, "failed to enable bus clock\n");
	}

	udelay(1);
	ret = reset_control_deassert(aiao->reset);
	if (ret)
		goto disable_clocks;

	ret = readl_poll_timeout(aiao->base + AIAO_STATUS, val, val & BIT(0),
				 1, 1000);
	if (ret) {
		dev_err(dev, "controller did not leave reset\n");
		goto assert_reset;
	}

	aiao_update_bits(aiao, AIAO_OUTSTANDING,
			 AIAO_OUTSTANDING_MASK, 2);
	aiao_write(aiao, AIAO_INT_ENA, 0);
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX0_INT_ENA), 0);
	aiao_write(aiao, aiao_tx_reg(aiao, AIAO_TX0_INT_CLR), AIAO_TX_INT_ALL);
	aiao->irq = platform_get_irq(pdev, 0);
	if (aiao->irq < 0) {
		ret = aiao->irq;
		goto assert_reset;
	}

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

assert_reset:
	reset_control_assert(aiao->reset);
disable_clocks:
	clk_disable_unprepare(aiao->bus_clk);
	clk_disable_unprepare(aiao->mclk);
	return ret;
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
	},
};
module_platform_driver(hi3798cv200_aiao_driver);

MODULE_DESCRIPTION("HiSilicon Hi3798CV200/Hi3798MV310 AIAO playback driver");
MODULE_LICENSE("GPL");
