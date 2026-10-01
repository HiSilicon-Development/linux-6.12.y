// SPDX-License-Identifier: GPL-2.0-only
/*
 * MaxLinear MxL214 four-channel DVB-C frontend
 *
 * The MxLWare register and firmware algorithms are retained in-kernel. This
 * file owns their lifetime and exposes only the Linux DVB frontend API.
 */

#include <linux/bitops.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/units.h>
#include <media/mxl214.h>

#include "MxL_HRCLS_Common.h"
#include "MxL_HRCLS_CommonApi.h"
#include "MxL_HRCLS_DemodApi.h"
#include "MxL_HRCLS_PhyCtrl.h"
#include "mxl214-priv.h"

#define MXL214_DEVICE_ID_COUNT	MXL_HRCLS_MAX_NUM_DEVICES
#define MXL214_DEFAULT_TIMEOUT_MS	5000
#define MXL214_STATS_INTERVAL_MS		500
#define MXL214_LOCK_QUALIFY_MS		500
#define MXL214_CODEWORD_BITS		(204U * 8U)

static DEFINE_MUTEX(mxl214_device_id_lock);
static unsigned long mxl214_device_ids;

static const struct dvb_frontend_ops mxl214_ops;

static unsigned int mxl214_expected_xpt_output(unsigned int channel)
{
	return channel + MXL_HRCLS_XPT_OUT_2;
}

static int mxl214_alloc_device_id(void)
{
	unsigned int id;

	mutex_lock(&mxl214_device_id_lock);
	id = find_first_zero_bit(&mxl214_device_ids,
				 MXL214_DEVICE_ID_COUNT);
	if (id < MXL214_DEVICE_ID_COUNT)
		set_bit(id, &mxl214_device_ids);
	mutex_unlock(&mxl214_device_id_lock);

	return id < MXL214_DEVICE_ID_COUNT ? id : -ENOSPC;
}

static void mxl214_free_device_id(unsigned int id)
{
	mutex_lock(&mxl214_device_id_lock);
	clear_bit(id, &mxl214_device_ids);
	mutex_unlock(&mxl214_device_id_lock);
}

static void mxl214_clear_context(struct mxl214_state *state)
{
	MXL_HRCLS_DEV_CONTEXT_T *context;

	context = MxL_HRCLS_Ctrl_GetDeviceContext(state->device_id);
	if (context)
		memset(context, 0, sizeof(*context));
	MxL_HRCLS_OEM_DataPtr[state->device_id] = NULL;
	state->initialized = false;
}

static int mxl214_wait_tuner(struct mxl214_state *state)
{
	MXL_HRCLS_TUNER_STATUS_E lock = MXL_HRCLS_TUNER_DISABLED;
	unsigned long deadline;
	MXL_STATUS_E status;

	deadline = jiffies + msecs_to_jiffies(state->lock_timeout_ms);
	do {
		usleep_range(1000, 2000);
		status = MxLWare_HRCLS_API_ReqTunerLockStatus(
			state->device_id, MXL_HRCLS_FULLBAND_TUNER, &lock);
	} while (status == MXL_SUCCESS &&
		 lock != MXL_HRCLS_TUNER_LOCKED &&
		 time_before(jiffies, deadline));

	if (status == MXL_SUCCESS && lock == MXL_HRCLS_TUNER_LOCKED)
		return 0;

	dev_err_ratelimited(&state->client->dev,
			    "full-band tuner failed to lock: status=%u state=%u\n",
			    status, lock);
	return status == MXL_SUCCESS ? -ETIMEDOUT : -EREMOTEIO;
}

static int mxl214_wait_channel(struct mxl214_state *state, unsigned int id)
{
	MXL_HRCLS_CHAN_STATUS_E lock = MXL_HRCLS_CHAN_DISABLED;
	MXL_HRCLS_RX_PWR_ACCURACY_E accuracy = MXL_HRCLS_PWR_INVALID;
	unsigned long deadline;
	UINT16 power = 0;
	MXL_STATUS_E power_status;
	MXL_STATUS_E status;

	deadline = jiffies + msecs_to_jiffies(state->lock_timeout_ms);
	do {
		usleep_range(1000, 2000);
		status = MxLWare_HRCLS_API_ReqTunerChanStatus(
			state->device_id, id, &lock);
	} while (status == MXL_SUCCESS &&
		 lock != MXL_HRCLS_CHAN_LOCKED &&
		 time_before(jiffies, deadline));

	if (status == MXL_SUCCESS && lock == MXL_HRCLS_CHAN_LOCKED)
		return 0;

	power_status = MxLWare_HRCLS_API_ReqTunerRxPwr(
		state->device_id, id, &power, &accuracy);
	dev_err_ratelimited(&state->client->dev,
			    "channel %u (physical %u) tuner readiness timed out: status=%u state=%u power-status=%u power=%u.%u dBuV accuracy=%u\n",
			    id, id + 2, status, lock, power_status,
			    power / 10, power % 10, accuracy);
	return status == MXL_SUCCESS ? -ETIMEDOUT : -EREMOTEIO;
}

static int mxl214_api_result(struct mxl214_state *state, unsigned int id,
			     const char *stage, MXL_STATUS_E status)
{
	if (status == MXL_SUCCESS)
		return 0;

	dev_err_ratelimited(&state->client->dev,
			    "channel %u %s failed: status=%u\n",
			    id, stage, status);
	return -EREMOTEIO;
}

static int mxl214_hw_init_locked(struct mxl214_state *state)
{
	const struct firmware *firmware;
	MXL_HRCLS_DEV_VER_T version;
	const char *stage = "driver init";
	MXL_STATUS_E status;
	int ret;

	if (state->initialized)
		return 0;

	status = MxLWare_HRCLS_API_CfgDrvInit(state->device_id, state,
					      MXL_HRCLS_DEVICE_214);
	if (status != MXL_SUCCESS)
		goto fail;

	stage = "hardware reset";
	status = MxLWare_HRCLS_API_CfgDevReset(state->device_id);
	if (status != MXL_SUCCESS)
		goto fail;

	stage = "reference clock setup";
	status = MxLWare_HRCLS_API_CfgDevXtalSetting(state->device_id, 0);
	if (status != MXL_SUCCESS && status != MXL_NOT_SUPPORTED)
		goto fail;
	if (status == MXL_NOT_SUPPORTED)
		dev_info(&state->client->dev,
			 "reference clock already running after reset request\n");

	stage = "downstream calibration load";
	status = MxLWare_HRCLS_API_CfgTunerDsCalDataLoad(state->device_id);
	if (status != MXL_SUCCESS)
		dev_warn(&state->client->dev,
			 "calibration unavailable; RF power is approximate\n");

	stage = "pre-download version query";
	status = MxLWare_HRCLS_API_ReqDevVersionInfo(state->device_id,
						      &version);
	if (status != MXL_SUCCESS)
		goto fail;
	if (!version.firmwareDownloaded) {
		ret = request_firmware(&firmware, MXL214_FIRMWARE,
				       &state->client->dev);
		if (ret) {
			dev_err(&state->client->dev,
				"firmware %s unavailable: %d\n",
				MXL214_FIRMWARE, ret);
			goto fail_ret;
		}
		if (!firmware->size || firmware->size > U16_MAX) {
			dev_err(&state->client->dev,
				"invalid firmware size: %zu\n", firmware->size);
			ret = -EINVAL;
			goto release_firmware;
		}

		stage = "firmware download";
		status = MxLWare_HRCLS_API_CfgDevFirmwareDownload(
			state->device_id, firmware->size,
			(UINT8 *)firmware->data, NULL);
		if (status != MXL_SUCCESS) {
			ret = -EREMOTEIO;
			goto release_firmware;
		}
		release_firmware(firmware);

		stage = "post-download version query";
		status = MxLWare_HRCLS_API_ReqDevVersionInfo(state->device_id,
							      &version);
		if (status != MXL_SUCCESS)
			goto fail;
		if (!version.firmwareDownloaded) {
			dev_err(&state->client->dev,
				"firmware download completed but firmware is not running\n");
			ret = -EIO;
			goto fail_ret;
		}
	} else {
		dev_info(&state->client->dev,
			 "reusing firmware retained across the reset request\n");
	}

	dev_info(&state->client->dev,
		 "firmware %u.%u.%u.%u-RC%u, chip revision %u\n",
		 version.firmwareVer[0], version.firmwareVer[1],
		 version.firmwareVer[2], version.firmwareVer[3],
		 version.firmwareVer[4], version.chipVersion);

	stage = "full-band tuner enable";
	status = MxLWare_HRCLS_API_CfgTunerEnable(
		state->device_id, MXL_HRCLS_FULLBAND_TUNER);
	/* MxLWare reports NOT_SUPPORTED when firmware already powers the tuner. */
	if (status != MXL_SUCCESS && status != MXL_NOT_SUPPORTED)
		goto fail;
	ret = mxl214_wait_tuner(state);
	if (ret)
		goto fail_ret;
	stage = "XPT four-channel mode";
	status = MxLWare_HRCLS_API_CfgXpt(state->device_id,
					  MXL_HRCLS_XPT_MODE_NO_MUX_4);
	if (status != MXL_SUCCESS)
		goto fail;

	stage = "global MPEG output setup";
	/* This API switches all outputs to four-wire common-clock mode. */
	if (!state->three_wire) {
		status = MxLWare_HRCLS_API_CfgDemodMpegOutGlobalParams(
			state->device_id, MXL_HRCLS_MPEG_CLK_POSITIVE,
			MXL_HRCLS_MPEG_DRV_MODE_1X, MXL_HRCLS_MPEG_CLK_56_21MHz);
		if (status != MXL_SUCCESS)
			goto fail;
	}

	state->initialized = true;
	return 0;

release_firmware:
	release_firmware(firmware);
fail_ret:
	mxl214_clear_context(state);
	return ret;
fail:
	dev_err(&state->client->dev,
		"chip initialization failed at %s: status=%u\n",
		stage, status);
	mxl214_clear_context(state);
	return -EREMOTEIO;
}

static int mxl214_qam(enum fe_modulation modulation,
		      MXL_HRCLS_QAM_TYPE_E *qam)
{
	switch (modulation) {
	case QAM_16:
		*qam = MXL_HRCLS_QAM16;
		break;
	case QAM_32:
		*qam = MXL_HRCLS_QAM32;
		break;
	case QAM_64:
		*qam = MXL_HRCLS_QAM64;
		break;
	case QAM_128:
		*qam = MXL_HRCLS_QAM128;
		break;
	case QAM_256:
		*qam = MXL_HRCLS_QAM256;
		break;
	case QAM_AUTO:
		*qam = MXL_HRCLS_QAM_AUTO;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void mxl214_stats_reset(struct mxl214_channel *channel)
{
	struct dtv_frontend_properties *properties;

	properties = &channel->frontend.dtv_property_cache;
	properties->strength.len = 1;
	properties->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	properties->cnr.len = 1;
	properties->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	properties->pre_bit_error.len = 1;
	properties->pre_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	properties->pre_bit_count.len = 1;
	properties->pre_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	properties->post_bit_error.len = 1;
	properties->post_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	properties->post_bit_count.len = 1;
	properties->post_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	properties->block_error.len = 1;
	properties->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	properties->block_count.len = 1;
	properties->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	channel->pre_bit_errors = 0;
	channel->pre_bit_count = 0;
	channel->block_errors = 0;
	channel->block_count = 0;
	channel->stats_deadline = 0;
	channel->lock_deadline = 0;
	channel->stats_started = false;
	channel->lock_candidate = false;
	channel->retune_restart_done = false;
	channel->demod_enabled = false;
	channel->locked = false;
}

static int mxl214_update_strength_locked(struct mxl214_channel *channel)
{
	struct dtv_frontend_properties *properties;
	struct mxl214_state *state = channel->state;
	MXL_HRCLS_RX_PWR_ACCURACY_E accuracy = MXL_HRCLS_PWR_INVALID;
	UINT16 power = 0;
	MXL_STATUS_E status;
	int percent;

	properties = &channel->frontend.dtv_property_cache;
	status = MxLWare_HRCLS_API_ReqTunerRxPwr(state->device_id,
		channel->id, &power, &accuracy);
	if (status != MXL_SUCCESS || accuracy == MXL_HRCLS_PWR_INVALID) {
		properties->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		return -EREMOTEIO;
	}

	percent = clamp_t(int, (int)power / 10 - 5, 0, 100);
	properties->strength.stat[0].scale = FE_SCALE_RELATIVE;
	properties->strength.stat[0].uvalue = percent * U16_MAX / 100;
	return 0;
}

static int mxl214_update_cnr_locked(struct mxl214_channel *channel)
{
	struct dtv_frontend_properties *properties;
	struct mxl214_state *state = channel->state;
	UINT16 db_x10 = 0;
	MXL_STATUS_E status;

	properties = &channel->frontend.dtv_property_cache;
	status = MxLWare_HRCLS_API_ReqDemodSnr(state->device_id,
		channel->id, &db_x10);
	if (status != MXL_SUCCESS) {
		properties->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		return -EREMOTEIO;
	}

	properties->cnr.stat[0].scale = FE_SCALE_DECIBEL;
	properties->cnr.stat[0].svalue = (s64)db_x10 * 100;
	return 0;
}

static void mxl214_publish_counters(struct mxl214_channel *channel)
{
	struct dtv_frontend_properties *properties;

	properties = &channel->frontend.dtv_property_cache;
	properties->pre_bit_error.stat[0].scale = FE_SCALE_COUNTER;
	properties->pre_bit_error.stat[0].uvalue = channel->pre_bit_errors;
	properties->pre_bit_count.stat[0].scale = FE_SCALE_COUNTER;
	properties->pre_bit_count.stat[0].uvalue = channel->pre_bit_count;
	properties->post_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	properties->post_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	properties->block_error.stat[0].scale = FE_SCALE_COUNTER;
	properties->block_error.stat[0].uvalue = channel->block_errors;
	properties->block_count.stat[0].scale = FE_SCALE_COUNTER;
	properties->block_count.stat[0].uvalue = channel->block_count;
}

static int mxl214_update_error_stats_locked(struct mxl214_channel *channel)
{
	struct mxl214_state *state = channel->state;
	MXL_HRCLS_DMD_STAT_CNT_T counters = { };
	MXL_STATUS_E status;

	if (!channel->stats_started) {
		status = MxLWare_HRCLS_API_CfgDemodErrorStatClear(
			state->device_id, channel->id);
		if (status != MXL_SUCCESS)
			return -EREMOTEIO;
		channel->stats_started = true;
		mxl214_publish_counters(channel);
		return 0;
	}

	/* ReqDemodErrorStat snapshots and clears the hardware counters itself. */
	status = MxLWare_HRCLS_API_ReqDemodErrorStat(state->device_id,
		channel->id, &counters);
	if (status != MXL_SUCCESS)
		return -EREMOTEIO;

	channel->pre_bit_errors += counters.CorrBits;
	channel->pre_bit_count +=
		(u64)counters.CwReceived * MXL214_CODEWORD_BITS;
	channel->block_errors += counters.CwErrCount;
	channel->block_count += counters.CwReceived;
	mxl214_publish_counters(channel);
	return 0;
}

static void mxl214_update_stats_locked(struct mxl214_channel *channel,
				       bool mpeg_locked)
{
	struct mxl214_state *state = channel->state;
	int ret;

	if (channel->stats_deadline &&
	    time_before(jiffies, channel->stats_deadline))
		return;
	channel->stats_deadline =
		jiffies + msecs_to_jiffies(MXL214_STATS_INTERVAL_MS);

	ret = mxl214_update_strength_locked(channel);
	if (ret)
		dev_dbg(&state->client->dev,
			"channel %u RF power statistics unavailable\n",
			channel->id);
	ret = mxl214_update_cnr_locked(channel);
	if (ret)
		dev_dbg(&state->client->dev,
			"channel %u CNR statistics unavailable\n", channel->id);
	if (!mpeg_locked)
		return;

	ret = mxl214_update_error_stats_locked(channel);
	if (ret)
		dev_dbg(&state->client->dev,
			"channel %u error counters unavailable\n", channel->id);
}

static int mxl214_qualify_lock_locked(struct mxl214_channel *channel,
				      bool full_lock)
{
	struct mxl214_state *state = channel->state;
	MXL_HRCLS_DMD_STAT_CNT_T counters = { };
	MXL_STATUS_E status;

	if (!full_lock) {
		channel->lock_candidate = false;
		channel->lock_deadline = 0;
		channel->locked = false;
		channel->stats_started = false;
		channel->stats_deadline = 0;
		return 0;
	}

	if (channel->locked)
		return 0;

	if (!channel->lock_candidate) {
		status = MxLWare_HRCLS_API_CfgDemodErrorStatClear(
			state->device_id, channel->id);
		if (status != MXL_SUCCESS)
			return -EREMOTEIO;

		channel->lock_candidate = true;
		channel->lock_deadline =
			jiffies + msecs_to_jiffies(MXL214_LOCK_QUALIFY_MS);
		return 0;
	}

	if (time_before(jiffies, channel->lock_deadline))
		return 0;

	/*
	 * The demod can briefly report all lock bits before its first FEC
	 * window settles.  Qualify lock only after one complete, error-free
	 * codeword window; a dirty or empty window starts another bounded pass.
	 */
	status = MxLWare_HRCLS_API_ReqDemodErrorStat(state->device_id,
		channel->id, &counters);
	if (status != MXL_SUCCESS)
		return -EREMOTEIO;

	if (!counters.CwReceived || counters.CwErrCount) {
		channel->lock_deadline =
			jiffies + msecs_to_jiffies(MXL214_LOCK_QUALIFY_MS);
		return 0;
	}

	channel->lock_candidate = false;
	channel->lock_deadline = 0;
	channel->locked = true;
	channel->stats_started = true;
	channel->stats_deadline =
		jiffies + msecs_to_jiffies(MXL214_STATS_INTERVAL_MS);
	mxl214_publish_counters(channel);
	return 0;
}

static int mxl214_init(struct dvb_frontend *frontend)
{
	struct mxl214_channel *channel = frontend->demodulator_priv;
	struct mxl214_state *state = channel->state;
	int ret;

	mutex_lock(&state->lock);
	ret = mxl214_hw_init_locked(state);
	mutex_unlock(&state->lock);
	return ret;
}

static int mxl214_sleep(struct dvb_frontend *frontend)
{
	(void)frontend;
	/* Frontends share one chip, so keep its lifetime device-scoped. */
	return 0;
}

static int mxl214_set_frontend(struct dvb_frontend *frontend)
{
	struct mxl214_channel *channel = frontend->demodulator_priv;
	struct mxl214_state *state = channel->state;
	struct mxl214_xpt_readback readback;
	struct dtv_frontend_properties *properties;
	MXL_HRCLS_XPT_MPEGOUT_PARAM_T output = {
		.enable = MXL_ENABLE,
		.lsbOrMsbFirst = MXL_HRCLS_MPEG_SERIAL_MSB_1ST,
		.mpegSyncPulseWidth = MXL_HRCLS_MPEG_SYNC_WIDTH_BIT,
		.mpegValidPol = MXL_HRCLS_MPEG_ACTIVE_HIGH,
		.mpegSyncPol = MXL_HRCLS_MPEG_ACTIVE_HIGH,
		.mpegClkPol = MXL_HRCLS_MPEG_CLK_POSITIVE,
		.clkFreq = MXL_HRCLS_MPEG_CLK_56_21MHz,
		.mpegPadDrv = {
			.padDrvMpegSyn = MXL_HRCLS_MPEG_DRV_MODE_2X,
			.padDrvMpegDat = MXL_HRCLS_MPEG_DRV_MODE_2X,
			.padDrvMpegVal = MXL_HRCLS_MPEG_DRV_MODE_2X,
		},
	};
	MXL_HRCLS_QAM_TYPE_E qam;
	MXL_STATUS_E status;
	int ret;

	if (state->three_wire)
		output.mpegClkPol = MXL_HRCLS_MPEG_CLK_NEGATIVE;

	properties = &frontend->dtv_property_cache;
	if (!properties->frequency || !properties->symbol_rate)
		return -EINVAL;
	ret = mxl214_qam(properties->modulation, &qam);
	if (ret)
		return ret;

	mutex_lock(&state->lock);
	ret = mxl214_hw_init_locked(state);
	if (ret)
		goto unlock;
	mxl214_stats_reset(channel);

	status = MxLWare_HRCLS_API_CfgXptOutput(state->device_id,
		channel->id, &output);
	ret = mxl214_api_result(state, channel->id, "XPT output", status);
	if (ret)
		goto unlock;
	ret = mxl214_xpt_readback(state->device_id, channel->id, &readback);
	if (ret) {
		dev_err(&state->client->dev,
			"channel %u XPT readback failed: %d\n",
			channel->id, ret);
		goto unlock;
	}
	dev_dbg(&state->client->dev,
		 "channel %u XPT readback: phys=%u core=%u oe=%u mck=%u tsclk=%u nco=%u mode=%u pad=%u/%u/%u eco=%04x/%04x\n",
		 channel->id, readback.physical_output,
		 readback.output_enable, readback.mpeg_output_enable,
		 readback.mpeg_clock_enable, readback.ts_clock_enable,
		 readback.nco_count_min, readback.xpt_ts_mode,
		 readback.pad_sync_drive, readback.pad_data_drive,
		 readback.pad_valid_drive, readback.eco4, readback.eco8);
	if (readback.physical_output !=
	    mxl214_expected_xpt_output(channel->id) ||
	    readback.output_enable != 1 ||
	    readback.mpeg_output_enable != 1 ||
	    readback.mpeg_clock_enable != 1 ||
	    readback.ts_clock_enable != !state->three_wire ||
	    readback.nco_count_min != 6 || readback.xpt_ts_mode != 0 ||
	    readback.pad_sync_drive != 1 ||
	    readback.pad_data_drive != 1 ||
	    readback.pad_valid_drive != 1 ||
	    (!state->three_wire &&
	     ((readback.eco4 & 0xfc) != 0xf0 || (readback.eco8 & 0x80))) ||
	    (state->three_wire && (readback.eco4 & 0xf0))) {
		dev_err(&state->client->dev,
			"channel %u XPT readback does not match %u-wire output\n",
			channel->id, state->three_wire ? 3 : 4);
		ret = -EIO;
		goto unlock;
	}

	status = MxLWare_HRCLS_API_CfgTunerChanTune(state->device_id,
		channel->id, 8, properties->frequency);
	ret = mxl214_api_result(state, channel->id, "tuner channel tune",
				status);
	if (ret)
		goto unlock;
	ret = mxl214_wait_channel(state, channel->id);
	if (ret)
		goto unlock;

	status = MxLWare_HRCLS_API_CfgDemodEnable(state->device_id,
		channel->id, MXL_TRUE);
	ret = mxl214_api_result(state, channel->id, "demod enable", status);
	if (ret)
		goto unlock;
	status = MxLWare_HRCLS_API_CfgDemodAdcIqFlip(state->device_id,
		channel->id, MXL_HRCLS_IQ_AUTO);
	ret = mxl214_api_result(state, channel->id, "demod IQ setup", status);
	if (ret)
		goto unlock;
	status = MxLWare_HRCLS_API_CfgDemodAnnexQamType(state->device_id,
		channel->id, MXL_HRCLS_ANNEX_A, qam);
	ret = mxl214_api_result(state, channel->id, "demod Annex/QAM setup",
				status);
	if (ret)
		goto unlock;
	status = MxLWare_HRCLS_API_CfgDemodSymbolRate(state->device_id,
		channel->id, properties->symbol_rate, properties->symbol_rate);
	ret = mxl214_api_result(state, channel->id, "demod symbol-rate setup",
				status);
	if (ret)
		goto unlock;
	status = MxLWare_HRCLS_API_CfgDemodRestart(state->device_id,
		channel->id);
	ret = mxl214_api_result(state, channel->id, "demod restart", status);
	if (ret)
		goto unlock;
	status = MxLWare_HRCLS_API_CfgDemodErrorStatClear(state->device_id,
		channel->id);
	ret = mxl214_api_result(state, channel->id,
				"error counter reset", status);
	if (ret)
		goto unlock;

	channel->demod_enabled = true;
	ret = 0;
	goto unlock;
unlock:
	mutex_unlock(&state->lock);
	return ret;
}

static int mxl214_read_status(struct dvb_frontend *frontend,
			      enum fe_status *frontend_status)
{
	struct mxl214_channel *channel = frontend->demodulator_priv;
	struct mxl214_state *state = channel->state;
	MXL_BOOL_E qam = MXL_FALSE;
	MXL_BOOL_E fec = MXL_FALSE;
	MXL_BOOL_E mpeg = MXL_FALSE;
	MXL_BOOL_E retune = MXL_FALSE;
	MXL_STATUS_E status;
	bool full_lock;
	bool qualified;
	int ret;

	*frontend_status = FE_NONE;
	mutex_lock(&state->lock);
	if (!state->initialized || !channel->demod_enabled) {
		mutex_unlock(&state->lock);
		return 0;
	}
	status = MxLWare_HRCLS_API_ReqDemodAllLockStatus(
		state->device_id, channel->id, &qam, &fec, &mpeg, &retune);
	if (status != MXL_SUCCESS) {
		mutex_unlock(&state->lock);
		return mxl214_api_result(state, channel->id,
					 "demod lock query", status);
	}
	if (retune) {
		if (!channel->retune_restart_done) {
			status = MxLWare_HRCLS_API_CfgDemodRestart(
				state->device_id, channel->id);
			if (status != MXL_SUCCESS) {
				mutex_unlock(&state->lock);
				return mxl214_api_result(state, channel->id,
					"firmware-requested demod restart",
					status);
			}
			channel->retune_restart_done = true;
			dev_dbg(&state->client->dev,
				"channel %u firmware requested one demod restart\n",
				channel->id);
			status = MxLWare_HRCLS_API_ReqDemodAllLockStatus(
				state->device_id, channel->id, &qam, &fec,
				&mpeg, &retune);
			if (status != MXL_SUCCESS) {
				mutex_unlock(&state->lock);
				return mxl214_api_result(state, channel->id,
					"post-restart demod lock query", status);
			}
		}
	}
	full_lock = qam && fec && mpeg && !retune;
	ret = mxl214_qualify_lock_locked(channel, full_lock);
	if (ret) {
		mutex_unlock(&state->lock);
		return ret;
	}
	if (qam && !retune)
		mxl214_update_stats_locked(channel, channel->locked);
	qualified = channel->locked;
	mutex_unlock(&state->lock);
	if (retune)
		return 0;
	if (qam)
		*frontend_status |= FE_HAS_SIGNAL | FE_HAS_CARRIER;
	if (fec)
		*frontend_status |= FE_HAS_VITERBI | FE_HAS_SYNC;
	if (qualified)
		*frontend_status |= FE_HAS_LOCK;
	return 0;
}

static enum dvbfe_algo mxl214_get_frontend_algo(struct dvb_frontend *frontend)
{
	(void)frontend;
	return DVBFE_ALGO_HW;
}

static int mxl214_tune(struct dvb_frontend *frontend, bool re_tune,
		       unsigned int mode_flags, unsigned int *delay,
		       enum fe_status *status)
{
	int ret;

	(void)mode_flags;
	*status = FE_NONE;
	*delay = HZ / 2;
	if (re_tune) {
		ret = mxl214_set_frontend(frontend);
		if (ret)
			return ret;
	}

	return mxl214_read_status(frontend, status);
}

static int mxl214_read_signal_strength(struct dvb_frontend *frontend,
				       u16 *strength)
{
	struct mxl214_channel *channel = frontend->demodulator_priv;
	struct mxl214_state *state = channel->state;
	struct dtv_frontend_properties *properties;
	int ret;

	properties = &frontend->dtv_property_cache;
	mutex_lock(&state->lock);
	if (!channel->demod_enabled) {
		*strength = 0;
		ret = 0;
		goto unlock;
	}

	ret = mxl214_update_strength_locked(channel);
	if (!ret)
		*strength = properties->strength.stat[0].uvalue;
unlock:
	mutex_unlock(&state->lock);
	return ret;
}

static int mxl214_read_snr(struct dvb_frontend *frontend, u16 *snr)
{
	struct mxl214_channel *channel = frontend->demodulator_priv;
	struct mxl214_state *state = channel->state;
	struct dtv_frontend_properties *properties;
	int ret;

	properties = &frontend->dtv_property_cache;
	mutex_lock(&state->lock);
	if (!channel->demod_enabled) {
		*snr = 0;
		ret = 0;
		goto unlock;
	}

	ret = mxl214_update_cnr_locked(channel);
	if (!ret)
		*snr = min_t(u64, properties->cnr.stat[0].svalue, 50000) *
			U16_MAX / 50000;
unlock:
	mutex_unlock(&state->lock);
	return ret;
}

static int mxl214_read_ber(struct dvb_frontend *frontend, u32 *ber)
{
	struct mxl214_channel *channel = frontend->demodulator_priv;
	struct mxl214_state *state = channel->state;

	mutex_lock(&state->lock);
	if (channel->locked)
		mxl214_update_stats_locked(channel, true);
	*ber = min_t(u64, channel->pre_bit_errors, U32_MAX);
	mutex_unlock(&state->lock);
	return 0;
}

static int mxl214_read_ucblocks(struct dvb_frontend *frontend, u32 *blocks)
{
	struct mxl214_channel *channel = frontend->demodulator_priv;
	struct mxl214_state *state = channel->state;

	mutex_lock(&state->lock);
	if (channel->locked)
		mxl214_update_stats_locked(channel, true);
	*blocks = min_t(u64, channel->block_errors, U32_MAX);
	mutex_unlock(&state->lock);
	return 0;
}

static const struct dvb_frontend_ops mxl214_ops = {
	.delsys = { SYS_DVBC_ANNEX_A },
	.info = {
		.name = "MaxLinear MxL214 DVB-C",
		.frequency_min_hz = 44 * MHz,
		.frequency_max_hz = 1002 * MHz,
		.frequency_stepsize_hz = 62500,
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 7125000,
		.caps = FE_CAN_QAM_16 | FE_CAN_QAM_32 | FE_CAN_QAM_64 |
			FE_CAN_QAM_128 | FE_CAN_QAM_256 | FE_CAN_QAM_AUTO |
			FE_CAN_INVERSION_AUTO,
	},
	.init = mxl214_init,
	.sleep = mxl214_sleep,
	.get_frontend_algo = mxl214_get_frontend_algo,
	.tune = mxl214_tune,
	.set_frontend = mxl214_set_frontend,
	.read_status = mxl214_read_status,
	.read_ber = mxl214_read_ber,
	.read_signal_strength = mxl214_read_signal_strength,
	.read_snr = mxl214_read_snr,
	.read_ucblocks = mxl214_read_ucblocks,
};

struct dvb_frontend *mxl214_get_frontend(struct i2c_client *client,
					 unsigned int id)
{
	struct mxl214_state *state;

	if (!client || id >= MXL214_NUM_FRONTENDS)
		return ERR_PTR(-EINVAL);
	state = i2c_get_clientdata(client);
	if (!state)
		return ERR_PTR(-EPROBE_DEFER);
	return &state->channels[id].frontend;
}
EXPORT_SYMBOL_GPL(mxl214_get_frontend);

static int mxl214_probe(struct i2c_client *client)
{
	struct mxl214_state *state;
	const char *ts_mode = "serial-4wire";
	int ret;
	int device_id;
	unsigned int i;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	if (of_property_present(client->dev.of_node, "maxlinear,ts-mode")) {
		if (of_property_read_string(client->dev.of_node,
					    "maxlinear,ts-mode", &ts_mode) ||
		    (strcmp(ts_mode, "serial-3wire") &&
		     strcmp(ts_mode, "serial-4wire")))
			return dev_err_probe(&client->dev, -EINVAL,
					     "invalid TS output mode\n");
	}

	device_id = mxl214_alloc_device_id();
	if (device_id < 0)
		return device_id;

	state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (!state) {
		mxl214_free_device_id(device_id);
		return -ENOMEM;
	}
	state->client = client;
	state->device_id = device_id;
	state->lock_timeout_ms = MXL214_DEFAULT_TIMEOUT_MS;
	state->three_wire = !strcmp(ts_mode, "serial-3wire");
	mutex_init(&state->lock);

	state->reset_gpio = devm_gpiod_get(&client->dev, "reset",
					    GPIOD_OUT_LOW);
	if (IS_ERR(state->reset_gpio)) {
		device_id = PTR_ERR(state->reset_gpio);
		mxl214_free_device_id(state->device_id);
		return device_id;
	}

	ret = mxl214_hw_init_locked(state);
	if (ret) {
		mxl214_free_device_id(state->device_id);
		return dev_err_probe(&client->dev, ret,
				     "MxL214 hardware initialization failed\n");
	}

	for (i = 0; i < MXL214_NUM_FRONTENDS; i++) {
		state->channels[i].state = state;
		state->channels[i].id = i;
		state->channels[i].frontend.ops = mxl214_ops;
		state->channels[i].frontend.demodulator_priv =
			&state->channels[i];
		mxl214_stats_reset(&state->channels[i]);
	}
	i2c_set_clientdata(client, state);

	dev_info(&client->dev,
		 "four DVB-C frontends available (%s)\n",
		 ts_mode);
	return 0;
}

static void mxl214_remove(struct i2c_client *client)
{
	struct mxl214_state *state = i2c_get_clientdata(client);

	mutex_lock(&state->lock);
	mxl214_clear_context(state);
	mutex_unlock(&state->lock);
	mxl214_free_device_id(state->device_id);
}

static const struct of_device_id mxl214_of_match[] = {
	{ .compatible = "maxlinear,mxl214c" },
	{ .compatible = "maxlinear,mxl214" },
	{ }
};
MODULE_DEVICE_TABLE(of, mxl214_of_match);

static const struct i2c_device_id mxl214_id[] = {
	{ "mxl214c", 0 },
	{ "mxl214", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxl214_id);

static struct i2c_driver mxl214_driver = {
	.driver = {
		.name = "mxl214",
		.of_match_table = mxl214_of_match,
	},
	.probe = mxl214_probe,
	.remove = mxl214_remove,
	.id_table = mxl214_id,
};
module_i2c_driver(mxl214_driver);

MODULE_AUTHOR("MaxLinear, Inc.");
MODULE_DESCRIPTION("MaxLinear MxL214/MxL214C four-channel DVB-C frontend");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(MXL214_FIRMWARE);
MODULE_FIRMWARE(MXL214_NVRAM);
