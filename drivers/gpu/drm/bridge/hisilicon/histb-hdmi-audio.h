/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __HISTB_HDMI_AUDIO_H__
#define __HISTB_HDMI_AUDIO_H__

#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_eld.h>
#include <sound/hdmi-codec.h>

/* CV200 accepts four I2S lanes; MV310 deliberately remains stereo.
 * The lock is shared with the bridge's modeset and power lifecycle.
 */
struct histb_hdmi_audio {
	struct mutex *lock;
	struct platform_device *codec;
	void *data;
	int (*program)(void *data, const struct hdmi_codec_params *params);
	void (*stop)(void *data);
	int (*restore)(void *data);
	struct hdmi_codec_params params;
	u8 eld[MAX_ELD_BYTES];
	bool extended;
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

static bool histb_audio_hbr(const struct hdmi_codec_params *params)
{
	return (params->iec.status[0] & IEC958_AES0_NONAUDIO) &&
		(params->coding_type == HDMI_AUDIO_CODING_TYPE_DTS_HD ||
		 params->coding_type == HDMI_AUDIO_CODING_TYPE_MLP);
}

/* Pure carrier validation: ALSA supplies already packed IEC61937 bursts,
 * not compressed elementary frames. HBR is four 16-bit pairs at 176.4/192
 * kHz; its 705.6/768 kHz stereo equivalent must not be treated as LPCM.
 */
static bool histb_audio_carrier_valid(const struct hdmi_codec_params *p,
				    bool extended)
{
	bool raw = p->iec.status[0] & IEC958_AES0_NONAUDIO;

	if (!histb_audio_n(p->sample_rate) ||
	    (p->iec.status[0] & IEC958_AES0_PROFESSIONAL))
		return false;
	if (!raw)
		return (p->channels == 2 ||
			(extended && (p->channels == 6 || p->channels == 8))) &&
			(p->sample_width == 16 || p->sample_width == 24) &&
			(!extended || p->coding_type == HDMI_AUDIO_CODING_TYPE_PCM);
	if (!extended || p->sample_width != 16 ||
	    !histb_audio_n(p->content_rate) ||
	    p->content_channels < 1 || p->content_channels > 8)
		return false;
	if (histb_audio_hbr(p))
		return p->channels == 8 &&
			((p->sample_rate == 192000 &&
			  (p->content_rate == 48000 || p->content_rate == 96000 ||
			   p->content_rate == 192000)) ||
			 (p->sample_rate == 176400 &&
			  (p->content_rate == 44100 || p->content_rate == 88200 ||
			   p->content_rate == 176400)));
	if (p->channels != 2)
		return false;
	switch (p->coding_type) {
	case HDMI_AUDIO_CODING_TYPE_AC3:
		return p->content_rate <= 48000 && p->content_channels <= 6 &&
			p->sample_rate == p->content_rate;
	case HDMI_AUDIO_CODING_TYPE_DTS:
		return p->content_rate <= 96000 && p->content_channels <= 6 &&
			p->sample_rate == p->content_rate;
	case HDMI_AUDIO_CODING_TYPE_EAC3:
		return p->content_rate <= 48000 &&
			p->sample_rate == 4 * p->content_rate;
	default:
		return false;
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
	bool raw = params->iec.status[0] & IEC958_AES0_NONAUDIO;
	unsigned int rate = raw ? params->content_rate : params->sample_rate;
	unsigned int channels = raw ? params->content_channels : params->channels;
	unsigned int coding = raw ? params->coding_type : HDMI_AUDIO_CODING_TYPE_PCM;

	if (!histb_audio_carrier_valid(params, audio->extended))
		return false;
	for (i = 0; i < ARRAY_SIZE(rates); i++)
		if (rate == rates[i])
			rate_bit = BIT(i);
	for (i = 0; i < drm_eld_sad_count(audio->eld); i++) {
		if (drm_eld_sad_get(audio->eld, i, &sad))
			continue;
		if (sad.format == coding && sad.channels + 1 >= channels &&
		    (sad.freq & rate_bit) &&
		    (raw || (sad.byte2 &
			(params->sample_width == 16 ? BIT(0) : BIT(2)))))
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
	    (params->sample_width != 16 && params->sample_width != 32) ||
	    (!audio->extended && params->channels != 2))
		return -EINVAL;
	guard(mutex)(audio->lock);
	if (audio->removing || !audio->active)
		return -ENODEV;
	/* AIAO S32_LE carries 24 significant MSBs (the CPU DAI constrains
	 * msbits to 24). Do not advertise nonexistent 32-bit HDMI LPCM.
	 */
	params->sample_width = params->sample_width == 32 ? 24 : 16;
	if (!histb_audio_carrier_valid(params, audio->extended))
		return -EINVAL;
	/* DMA words follow hdmi-codec's CEA channel-map order. Only complete
	 * 5.1/7.1 maps are implemented, not sparse speaker allocations.
	 */
	if (!(params->iec.status[0] & IEC958_AES0_NONAUDIO) &&
	    ((params->channels == 6 && params->cea.channel_allocation != 0x0b) ||
	     (params->channels == 8 && params->cea.channel_allocation != 0x13)))
		return -EINVAL;
	/* si_drv_tx_regs.h I2S_CHST3: HBR's two-channel equivalent is
	 * 768 kHz (0x09) or 705.6 kHz (FS extension 2, code 0x0d).
	 */
	params->iec.status[3] = (params->iec.status[3] & 0x30) |
		(histb_audio_hbr(params) ?
		 (params->sample_rate == 192000 ? 0x09 : 0x8d) :
		 histb_audio_fs(params->sample_rate));
	params->iec.status[4] =
		((~histb_audio_fs(params->iec.status[0] & IEC958_AES0_NONAUDIO ?
				 params->content_rate : params->sample_rate) & 0xf) << 4) |
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
	int ret;

	guard(mutex)(audio->lock);
	if (audio->removing)
		return -ENODEV;
	/* HPD can drop without a modeset. Revalidate the existing live link
	 * before exposing its ELD to a newly opened PCM stream. */
	if (audio->restore) {
		ret = audio->restore(audio->data);
		if (ret)
			return ret;
	}
	if (!audio->active)
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
		.max_i2s_channels = audio->extended ? 8 : 2,
		.iec61937 = audio->extended,
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
