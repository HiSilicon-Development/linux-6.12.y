/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __HISTB_HDMI_AUDIO_H__
#define __HISTB_HDMI_AUDIO_H__

#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_eld.h>
#include <sound/hdmi-codec.h>

/* Both transmitters consume stereo I2S from the board-selected AIAO TX.
 * The lock is shared with the bridge's modeset and power lifecycle.
 */
struct histb_hdmi_audio {
	struct mutex *lock;
	struct platform_device *codec;
	void *data;
	int (*program)(void *data, const struct hdmi_codec_params *params);
	void (*stop)(void *data);
	struct hdmi_codec_params params;
	u8 eld[MAX_ELD_BYTES];
	bool active;
	bool configured;
	bool muted;
	bool removing;
};

static u32 histb_audio_n(unsigned int rate)
{
	switch (rate) {
	case 32000: return 4096;
	case 44100: return 6272;
	case 48000: return 6144;
	case 88200: return 12544;
	case 96000: return 12288;
	case 176400: return 25088;
	case 192000: return 24576;
	default: return 0;
	}
}

static u8 histb_audio_fs(unsigned int rate)
{
	switch (rate) {
	case 32000: return 3;
	case 44100: return 0;
	case 48000: return 2;
	case 88200: return 8;
	case 96000: return 10;
	case 176400: return 12;
	case 192000: return 14;
	default: return 1;
	}
}

static bool histb_audio_sink_supports(struct histb_hdmi_audio *audio,
				     const struct hdmi_codec_params *params)
{
	static const unsigned int rates[] = {
		32000, 44100, 48000, 88200, 96000, 176400, 192000,
	};
	struct cea_sad sad;
	unsigned int i, rate_bit = 0;

	for (i = 0; i < ARRAY_SIZE(rates); i++)
		if (params->sample_rate == rates[i])
			rate_bit = BIT(i);
	for (i = 0; i < drm_eld_sad_count(audio->eld); i++) {
		if (drm_eld_sad_get(audio->eld, i, &sad))
			continue;
		if (sad.format == HDMI_AUDIO_CODING_TYPE_PCM &&
		    sad.channels + 1 >= params->channels && (sad.freq & rate_bit) &&
		    (sad.byte2 & (params->sample_width == 16 ? BIT(0) : BIT(2))))
			return true;
	}
	return false;
}

/* Caller holds the bridge/audio lock. A disabled display never emits audio. */
static int histb_audio_apply(struct histb_hdmi_audio *audio)
{
	audio->stop(audio->data);
	if (!audio->active || !audio->configured || audio->muted || audio->removing)
		return 0;
	if (!histb_audio_sink_supports(audio, &audio->params))
		return -EINVAL;
	return audio->program(audio->data, &audio->params);
}

static int histb_audio_hw_params(struct device *dev, void *data,
				struct hdmi_codec_daifmt *fmt,
				struct hdmi_codec_params *params)
{
	struct histb_hdmi_audio *audio = data;
	int ret;

	if (fmt->fmt != HDMI_I2S || fmt->bit_clk_inv || fmt->frame_clk_inv ||
	    fmt->bit_clk_provider || fmt->frame_clk_provider ||
	    (fmt->bit_fmt != SNDRV_PCM_FORMAT_S16_LE &&
	     fmt->bit_fmt != SNDRV_PCM_FORMAT_S32_LE) ||
	    params->channels != 2 ||
	    (params->sample_width != 16 && params->sample_width != 32) ||
	    !histb_audio_n(params->sample_rate) ||
	    (params->iec.status[0] & (IEC958_AES0_NONAUDIO | IEC958_AES0_PROFESSIONAL)))
		return -EINVAL;
	guard(mutex)(audio->lock);
	if (audio->removing || !audio->active)
		return -ENODEV;
	/* AIAO S32_LE carries 24 significant MSBs (the CPU DAI constrains
	 * msbits to 24). Do not advertise nonexistent 32-bit HDMI LPCM.
	 */
	params->sample_width = params->sample_width == 32 ? 24 : 16;
	params->iec.status[3] = (params->iec.status[3] & 0xf0) |
		histb_audio_fs(params->sample_rate);
	params->iec.status[4] = ((~histb_audio_fs(params->sample_rate) & 0xf) << 4) |
		(params->sample_width == 16 ? 2 : 11);
	if (!histb_audio_sink_supports(audio, params))
		return -EINVAL;
	audio->params = *params;
	audio->configured = true;
	ret = histb_audio_apply(audio);
	if (ret)
		audio->configured = false;
	return ret;
}

static void histb_audio_shutdown(struct device *dev, void *data)
{
	struct histb_hdmi_audio *audio = data;

	guard(mutex)(audio->lock);
	audio->configured = false;
	audio->muted = true;
	if (!audio->removing)
		audio->stop(audio->data);
}

static int histb_audio_mute(struct device *dev, void *data, bool mute, int direction)
{
	struct histb_hdmi_audio *audio = data;

	guard(mutex)(audio->lock);
	if (audio->removing)
		return -ENODEV;
	audio->muted = mute;
	return histb_audio_apply(audio);
}

static int histb_audio_get_eld(struct device *dev, void *data, u8 *buf, size_t len)
{
	struct histb_hdmi_audio *audio = data;

	guard(mutex)(audio->lock);
	if (audio->removing || !audio->active)
		return -ENODEV;
	memcpy(buf, audio->eld, min(len, sizeof(audio->eld)));
	return 0;
}

static const struct hdmi_codec_ops histb_audio_ops = {
	.hw_params = histb_audio_hw_params,
	.audio_shutdown = histb_audio_shutdown,
	.mute_stream = histb_audio_mute,
	.get_eld = histb_audio_get_eld,
	.no_capture_mute = 1,
};

static int histb_audio_register(struct device *dev, struct histb_hdmi_audio *audio)
{
	struct hdmi_codec_pdata pdata = {
		.ops = &histb_audio_ops,
		.i2s = 1,
		.no_i2s_capture = 1,
		.max_i2s_channels = 2,
		.data = audio,
	};

	audio->muted = true;
	audio->codec = platform_device_register_data(dev, HDMI_CODEC_DRV_NAME,
						    PLATFORM_DEVID_AUTO,
						    &pdata, sizeof(pdata));
	return PTR_ERR_OR_ZERO(audio->codec);
}

/* Called before powering off MMIO and before unregistering the child codec. */
static void histb_audio_unregister(struct histb_hdmi_audio *audio)
{
	mutex_lock(audio->lock);
	audio->removing = true;
	audio->stop(audio->data);
	audio->active = false;
	mutex_unlock(audio->lock);
	platform_device_unregister(audio->codec);
}

static int histb_audio_enable(struct histb_hdmi_audio *audio,
			      struct drm_connector *connector)
{
	audio->active = connector->display_info.is_hdmi &&
		connector->display_info.has_audio;
	mutex_lock(&connector->eld_mutex);
	memcpy(audio->eld, connector->eld, sizeof(audio->eld));
	mutex_unlock(&connector->eld_mutex);
	/* CTA Basic Audio implies stereo 16-bit 32/44.1/48 kHz even if the
	 * display supplies no Audio Data Block. Preserve that mandated SAD.
	 */
	if (audio->active && !drm_eld_sad_count(audio->eld)) {
		unsigned int offset = DRM_ELD_CEA_SAD(drm_eld_mnl(audio->eld), 0);

		audio->eld[offset] = (HDMI_AUDIO_CODING_TYPE_PCM << 3) | 1;
		audio->eld[offset + 1] = 7;
		audio->eld[offset + 2] = 1;
		audio->eld[DRM_ELD_SAD_COUNT_CONN_TYPE] |= 1 << DRM_ELD_SAD_COUNT_SHIFT;
		audio->eld[DRM_ELD_BASELINE_ELD_LEN] =
			DIV_ROUND_UP(drm_eld_calc_baseline_block_size(audio->eld), 4);
	}
	return histb_audio_apply(audio);
}

static void histb_audio_disable(struct histb_hdmi_audio *audio)
{
	audio->active = false;
	audio->stop(audio->data);
}
#endif
