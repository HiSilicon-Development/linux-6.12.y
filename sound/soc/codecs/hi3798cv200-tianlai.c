// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3798CV200 Tianlai internal stereo DAC
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>

#define TIANLAI_ADAC0			0x110
#define  TIANLAI_DACR_VOLUME_MASK	GENMASK(6, 0)
#define  TIANLAI_DACL_VOLUME_MASK	GENMASK(14, 8)
#define  TIANLAI_DACR_PATH		BIT(22)
#define  TIANLAI_DACL_PATH		BIT(23)
#define  TIANLAI_DIGITAL_MUTE_MASK	GENMASK(29, 28)
#define  TIANLAI_DIGITAL_PD_MASK	GENMASK(31, 30)

#define TIANLAI_ADAC1			0x114
#define  TIANLAI_POP_DIRECT_R		BIT(0)
#define  TIANLAI_POP_RES_MASK		GENMASK(2, 1)
#define  TIANLAI_POP_DIRECT_L		BIT(3)
#define  TIANLAI_POP_CLK_MASK		GENMASK(5, 4)
#define  TIANLAI_RATE_MASK		GENMASK(21, 19)
#define  TIANLAI_DATA_BITS_MASK	GENMASK(23, 22)

#define TIANLAI_ADAC2			0x160
#define  TIANLAI_PD_VREF		BIT(2)
#define  TIANLAI_PD_CTCM_BIAS		BIT(3)
#define  TIANLAI_PD_BIAS		BIT(4)
#define  TIANLAI_PD_DACR		BIT(5)
#define  TIANLAI_PD_DACL		BIT(6)
#define  TIANLAI_PD_CTCM		BIT(7)
#define  TIANLAI_ANALOG_RESET		BIT(13)
#define  TIANLAI_ANALOG_MUTE_MASK	GENMASK(15, 14)
#define  TIANLAI_TD_SEL_MASK		GENMASK(20, 16)
#define  TIANLAI_FAST_START		BIT(21)
#define  TIANLAI_POPFREE_R		BIT(22)
#define  TIANLAI_POPFREE_L		BIT(23)
#define  TIANLAI_FALLING_EDGE		BIT(24)
#define  TIANLAI_CLOCK_SELECT		BIT(25)

#define TIANLAI_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
		       SNDRV_PCM_RATE_12000 | SNDRV_PCM_RATE_16000 | \
		       SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_24000 | \
		       SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		       SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
		       SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
		       SNDRV_PCM_RATE_192000)

struct hi3798cv200_tianlai {
	struct regmap *regmap;
	struct clk *clk;
	struct reset_control *reset;
	struct gpio_desc *mute_gpio;
	/* Protects power sequencing and the requested mute state. */
	struct mutex lock;
	unsigned int rate;
	bool powered;
	bool muted;
};

static unsigned int tianlai_rate_code(unsigned int rate)
{
	if (rate >= 176400)
		return 4;
	if (rate >= 88200)
		return 3;
	if (rate >= 32000)
		return 2;
	if (rate >= 16000)
		return 1;
	return 0;
}

static int tianlai_set_rate(struct hi3798cv200_tianlai *tianlai,
			    unsigned int rate)
{
	tianlai->rate = rate;
	return regmap_update_bits(tianlai->regmap, TIANLAI_ADAC1,
				  TIANLAI_RATE_MASK,
				  FIELD_PREP(TIANLAI_RATE_MASK,
					     tianlai_rate_code(rate)));
}

static int tianlai_apply_mute(struct hi3798cv200_tianlai *tianlai,
			      bool mute)
{
	int ret;

	if (mute && tianlai->mute_gpio)
		gpiod_set_value_cansleep(tianlai->mute_gpio, 1);

	ret = regmap_update_bits(tianlai->regmap, TIANLAI_ADAC0,
				 TIANLAI_DIGITAL_MUTE_MASK,
				 mute ? TIANLAI_DIGITAL_MUTE_MASK : 0);
	if (ret)
		return ret;

	ret = regmap_update_bits(tianlai->regmap, TIANLAI_ADAC2,
				 TIANLAI_ANALOG_MUTE_MASK,
				 mute ? TIANLAI_ANALOG_MUTE_MASK : 0);
	if (ret)
		return ret;

	if (!mute && tianlai->mute_gpio)
		gpiod_set_value_cansleep(tianlai->mute_gpio, 0);

	return 0;
}

static int tianlai_power_up(struct hi3798cv200_tianlai *tianlai)
{
	u32 mask, val;
	int ret;

	if (tianlai->powered)
		return 0;

	ret = reset_control_assert(tianlai->reset);
	if (ret)
		return ret;

	ret = clk_prepare_enable(tianlai->clk);
	if (ret)
		return ret;
	usleep_range(10, 20);

	ret = regmap_update_bits(tianlai->regmap, TIANLAI_ADAC2,
				 TIANLAI_ANALOG_RESET,
				 TIANLAI_ANALOG_RESET);
	if (ret)
		goto disable_clock;
	ret = regmap_update_bits(tianlai->regmap, TIANLAI_ADAC2,
				 TIANLAI_ANALOG_RESET, 0);
	if (ret)
		goto disable_clock;

	ret = regmap_update_bits(tianlai->regmap, TIANLAI_ADAC2,
				 TIANLAI_CLOCK_SELECT | TIANLAI_FALLING_EDGE,
				 TIANLAI_FALLING_EDGE);
	if (ret)
		goto disable_clock;

	ret = reset_control_deassert(tianlai->reset);
	if (ret)
		goto disable_clock;

	mask = TIANLAI_DACR_VOLUME_MASK | TIANLAI_DACL_VOLUME_MASK |
	       TIANLAI_DACR_PATH | TIANLAI_DACL_PATH |
	       TIANLAI_DIGITAL_MUTE_MASK | TIANLAI_DIGITAL_PD_MASK;
	val = FIELD_PREP(TIANLAI_DACR_VOLUME_MASK, 0x06) |
	      FIELD_PREP(TIANLAI_DACL_VOLUME_MASK, 0x06) |
	      TIANLAI_DACR_PATH | TIANLAI_DIGITAL_MUTE_MASK;
	ret = regmap_update_bits(tianlai->regmap, TIANLAI_ADAC0, mask, val);
	if (ret)
		goto assert_reset;

	mask = TIANLAI_DATA_BITS_MASK | TIANLAI_POP_RES_MASK |
	       TIANLAI_POP_CLK_MASK | TIANLAI_POP_DIRECT_R |
	       TIANLAI_POP_DIRECT_L;
	val = FIELD_PREP(TIANLAI_DATA_BITS_MASK, 3) |
	      FIELD_PREP(TIANLAI_POP_RES_MASK, 1) |
	      FIELD_PREP(TIANLAI_POP_CLK_MASK, 1);
	ret = regmap_update_bits(tianlai->regmap, TIANLAI_ADAC1, mask, val);
	if (ret)
		goto assert_reset;

	ret = tianlai_set_rate(tianlai, tianlai->rate);
	if (ret)
		goto assert_reset;

	mask = TIANLAI_PD_VREF | TIANLAI_PD_CTCM_BIAS |
	       TIANLAI_PD_BIAS | TIANLAI_PD_DACR | TIANLAI_PD_DACL |
	       TIANLAI_PD_CTCM | TIANLAI_ANALOG_MUTE_MASK |
	       TIANLAI_TD_SEL_MASK | TIANLAI_FAST_START |
	       TIANLAI_POPFREE_R | TIANLAI_POPFREE_L;
	val = TIANLAI_PD_DACR | TIANLAI_PD_DACL |
	      TIANLAI_ANALOG_MUTE_MASK | TIANLAI_POPFREE_R |
	      TIANLAI_POPFREE_L;
	if (tianlai->mute_gpio)
		val |= TIANLAI_FAST_START;
	ret = regmap_update_bits(tianlai->regmap, TIANLAI_ADAC2, mask, val);
	if (ret)
		goto assert_reset;

	ret = regmap_update_bits(tianlai->regmap, TIANLAI_ADAC1,
				 TIANLAI_POP_DIRECT_R | TIANLAI_POP_DIRECT_L,
				 TIANLAI_POP_DIRECT_R | TIANLAI_POP_DIRECT_L);
	if (ret)
		goto assert_reset;

	if (tianlai->mute_gpio)
		usleep_range(10000, 11000);
	else
		msleep(500);

	ret = regmap_update_bits(tianlai->regmap, TIANLAI_ADAC2,
				 TIANLAI_PD_DACR | TIANLAI_PD_DACL |
				 TIANLAI_FAST_START,
				 0);
	if (ret)
		goto assert_reset;

	ret = tianlai_apply_mute(tianlai, tianlai->muted);
	if (ret)
		goto assert_reset;

	tianlai->powered = true;
	return 0;

assert_reset:
	reset_control_assert(tianlai->reset);
disable_clock:
	clk_disable_unprepare(tianlai->clk);
	return ret;
}

static void tianlai_power_down(struct hi3798cv200_tianlai *tianlai)
{
	unsigned int volume;

	if (!tianlai->powered)
		return;

	tianlai_apply_mute(tianlai, true);

	for (volume = 0x06; volume < 0x7f; volume += 10) {
		regmap_update_bits(tianlai->regmap, TIANLAI_ADAC0,
				   TIANLAI_DACR_VOLUME_MASK |
				   TIANLAI_DACL_VOLUME_MASK,
				   FIELD_PREP(TIANLAI_DACR_VOLUME_MASK,
					      min(volume, 0x7fU)) |
				   FIELD_PREP(TIANLAI_DACL_VOLUME_MASK,
					      min(volume, 0x7fU)));
		usleep_range(1000, 1500);
	}

	regmap_update_bits(tianlai->regmap, TIANLAI_ADAC1,
			   TIANLAI_POP_RES_MASK | TIANLAI_POP_CLK_MASK |
			   TIANLAI_POP_DIRECT_R | TIANLAI_POP_DIRECT_L,
			   FIELD_PREP(TIANLAI_POP_RES_MASK, 3) |
			   FIELD_PREP(TIANLAI_POP_CLK_MASK, 1));
	regmap_update_bits(tianlai->regmap, TIANLAI_ADAC2,
			   TIANLAI_POPFREE_R | TIANLAI_POPFREE_L |
			   TIANLAI_PD_DACR | TIANLAI_PD_DACL,
			   TIANLAI_POPFREE_R | TIANLAI_POPFREE_L |
			   TIANLAI_PD_DACR | TIANLAI_PD_DACL);
	msleep(50);
	regmap_update_bits(tianlai->regmap, TIANLAI_ADAC2,
			   TIANLAI_PD_VREF | TIANLAI_PD_CTCM_BIAS |
			   TIANLAI_PD_BIAS | TIANLAI_PD_CTCM,
			   TIANLAI_PD_VREF | TIANLAI_PD_CTCM_BIAS |
			   TIANLAI_PD_BIAS | TIANLAI_PD_CTCM);
	regmap_update_bits(tianlai->regmap, TIANLAI_ADAC0,
			   TIANLAI_DIGITAL_MUTE_MASK |
			   TIANLAI_DIGITAL_PD_MASK,
			   TIANLAI_DIGITAL_MUTE_MASK |
			   TIANLAI_DIGITAL_PD_MASK);
	regmap_update_bits(tianlai->regmap, TIANLAI_ADAC2,
			   TIANLAI_ANALOG_RESET, TIANLAI_ANALOG_RESET);

	reset_control_assert(tianlai->reset);
	clk_disable_unprepare(tianlai->clk);
	tianlai->powered = false;
}

static int tianlai_dapm_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct hi3798cv200_tianlai *tianlai =
		snd_soc_component_get_drvdata(component);
	int ret = 0;

	mutex_lock(&tianlai->lock);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = tianlai_power_up(tianlai);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		tianlai_power_down(tianlai);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&tianlai->lock);

	return ret;
}

static int tianlai_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct hi3798cv200_tianlai *tianlai =
		snd_soc_component_get_drvdata(dai->component);
	int ret;

	if (params_format(params) != SNDRV_PCM_FORMAT_S16_LE &&
	    params_format(params) != SNDRV_PCM_FORMAT_S24_LE)
		return -EINVAL;

	mutex_lock(&tianlai->lock);
	ret = tianlai_set_rate(tianlai, params_rate(params));
	mutex_unlock(&tianlai->lock);
	return ret;
}

static int tianlai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S)
		return -EINVAL;
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) !=
	    SND_SOC_DAIFMT_BC_FC)
		return -EINVAL;
	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;

	return 0;
}

static int tianlai_mute_stream(struct snd_soc_dai *dai, int mute,
			       int direction)
{
	struct hi3798cv200_tianlai *tianlai =
		snd_soc_component_get_drvdata(dai->component);
	int ret = 0;

	mutex_lock(&tianlai->lock);
	tianlai->muted = mute;
	if (tianlai->powered)
		ret = tianlai_apply_mute(tianlai, mute);
	mutex_unlock(&tianlai->lock);
	return ret;
}

static const struct snd_soc_dai_ops tianlai_dai_ops = {
	.hw_params = tianlai_hw_params,
	.set_fmt = tianlai_set_fmt,
	.mute_stream = tianlai_mute_stream,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver tianlai_dai = {
	.name = "hi3798cv200-tianlai-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = TIANLAI_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &tianlai_dai_ops,
};

static const struct snd_soc_dapm_widget tianlai_widgets[] = {
	SND_SOC_DAPM_DAC_E("DAC", "Playback", SND_SOC_NOPM, 0, 0,
			   tianlai_dapm_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
};

static const struct snd_soc_dapm_route tianlai_routes[] = {
	{ "LOUT", NULL, "DAC" },
	{ "ROUT", NULL, "DAC" },
};

static int tianlai_component_probe(struct snd_soc_component *component)
{
	struct hi3798cv200_tianlai *tianlai = dev_get_drvdata(component->dev);

	snd_soc_component_init_regmap(component, tianlai->regmap);
	return 0;
}

static void tianlai_component_remove(struct snd_soc_component *component)
{
	struct hi3798cv200_tianlai *tianlai =
		snd_soc_component_get_drvdata(component);

	mutex_lock(&tianlai->lock);
	tianlai_power_down(tianlai);
	mutex_unlock(&tianlai->lock);
	snd_soc_component_exit_regmap(component);
}

static const struct snd_soc_component_driver tianlai_component = {
	.probe = tianlai_component_probe,
	.remove = tianlai_component_remove,
	.dapm_widgets = tianlai_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tianlai_widgets),
	.dapm_routes = tianlai_routes,
	.num_dapm_routes = ARRAY_SIZE(tianlai_routes),
	.use_pmdown_time = 1,
	.endianness = 1,
};

static int hi3798cv200_tianlai_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hi3798cv200_tianlai *tianlai;
	struct device_node *parent;
	int ret;

	tianlai = devm_kzalloc(dev, sizeof(*tianlai), GFP_KERNEL);
	if (!tianlai)
		return -ENOMEM;

	parent = of_get_parent(dev->of_node);
	if (!parent)
		return -ENODEV;
	tianlai->regmap = syscon_node_to_regmap(parent);
	of_node_put(parent);
	if (IS_ERR(tianlai->regmap))
		return dev_err_probe(dev, PTR_ERR(tianlai->regmap),
				     "failed to get peripheral regmap\n");

	tianlai->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(tianlai->clk))
		return dev_err_probe(dev, PTR_ERR(tianlai->clk),
				     "failed to get DAC clock\n");

	tianlai->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(tianlai->reset))
		return dev_err_probe(dev, PTR_ERR(tianlai->reset),
				     "failed to get DAC reset\n");

	tianlai->mute_gpio = devm_gpiod_get_optional(dev, "mute",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(tianlai->mute_gpio))
		return dev_err_probe(dev, PTR_ERR(tianlai->mute_gpio),
				     "failed to get mute GPIO\n");

	ret = reset_control_assert(tianlai->reset);
	if (ret)
		return dev_err_probe(dev, ret, "failed to assert DAC reset\n");

	tianlai->rate = 48000;
	tianlai->muted = true;
	mutex_init(&tianlai->lock);
	platform_set_drvdata(pdev, tianlai);

	return devm_snd_soc_register_component(dev, &tianlai_component,
					       &tianlai_dai, 1);
}

static const struct of_device_id hi3798cv200_tianlai_of_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-tianlai-codec" },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3798cv200_tianlai_of_match);

static struct platform_driver hi3798cv200_tianlai_driver = {
	.probe = hi3798cv200_tianlai_probe,
	.driver = {
		.name = "hi3798cv200-tianlai",
		.of_match_table = hi3798cv200_tianlai_of_match,
	},
};
module_platform_driver(hi3798cv200_tianlai_driver);

MODULE_DESCRIPTION("HiSilicon Hi3798CV200 Tianlai internal DAC driver");
MODULE_LICENSE("GPL");
