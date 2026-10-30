// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Cypress FX2 / HDIC HD2312 DTMB receiver driver
 *
 * Based on hanwckf's HD2312 driver. Firmware-specific adapters share
 * its USB control and frontend protocol.
 *
 * Copyright (C) 2024 hanwckf <hanwckf@vip.qq.com>
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include "hd2312.h"

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static int hd2312_ctrl(struct dvb_usb_device *d, u8 request, bool read,
		       void *data, u16 length, unsigned int timeout_ms)
{
	struct hd2312_state *state = d_to_priv(d);
	unsigned int pipe = read ? usb_rcvctrlpipe(d->udev, 0) :
				   usb_sndctrlpipe(d->udev, 0);
	u8 request_type = USB_TYPE_VENDOR | (read ? USB_DIR_IN : USB_DIR_OUT);
	int ret;

	if (length > sizeof(state->ctrl_buf))
		return -EINVAL;

	if (mutex_lock_interruptible(&d->usb_mutex))
		return -EAGAIN;

	if (length && !read)
		memcpy(state->ctrl_buf, data, length);

	ret = usb_control_msg(d->udev, pipe, request, request_type, 0x00fe, 0,
			      state->ctrl_buf, length, timeout_ms);

	if (ret < 0)
		goto out_unlock;

	if (ret != length) {
		ret = -EREMOTEIO;
		goto out_unlock;
	}

	if (read && length)
		memcpy(data, state->ctrl_buf, length);

	ret = 0;

out_unlock:
	mutex_unlock(&d->usb_mutex);
	return ret;
}

static int hd2312_identify_vendor(struct dvb_usb_device *d)
{
	struct hd2312_state *state = d_to_priv(d);
	const char *vendor_name = "Generic HDIC";
	static const u8 cvb_key[] = "cidanaKEY";
	u8 auth_resp[25];
	int ret;

	state->vendor = HD2312_VENDOR_GENERIC;

	ret = hd2312_ctrl(d, HD2312_REQ_VERSION, true, state->fw_version,
			  sizeof(state->fw_version), HD2312_USB_TIMEOUT_MS);
	if (ret) {
		dev_warn(&d->udev->dev, "HD2312: failed to read firmware version: %d\n", ret);
		return ret;
	}

	/* Select known firmware profiles; these bytes are not a USB vendor ID. */
	if (state->fw_version[1] == 0x08 &&
	    state->fw_version[2] == 0x20 &&
	    state->fw_version[3] == 0x44) {
		switch (state->fw_version[0]) {
		case HD2312_VENDOR_LETV:
			state->vendor = HD2312_VENDOR_LETV;
			vendor_name = "LeShi / Letv";
			break;
		case HD2312_VENDOR_AIWA:
			state->vendor = HD2312_VENDOR_AIWA;
			vendor_name = "Aiwa itv720";
			break;
		case HD2312_VENDOR_CVB:
			state->vendor = HD2312_VENDOR_CVB;
			vendor_name = "CVB T302A";
			break;
		default:
			break;
		}
	}

	dev_info(&d->udev->dev, "HD2312: detected %s (firmware: %d.%d.%d%d)\n",
		 vendor_name,
		 state->fw_version[0], state->fw_version[1],
		 state->fw_version[2], state->fw_version[3]);

	/* Perform CVB authentication handshake if required by firmware */
	if (state->vendor == HD2312_VENDOR_CVB) {
		ret = hd2312_ctrl(d, HD2312_REQ_CVB_EXTRA_AUTH, false,
				  (void *)cvb_key, sizeof(cvb_key) - 1,
				  HD2312_USB_TIMEOUT_MS);
		if (ret)
			return ret;

		ret = hd2312_ctrl(d, HD2312_REQ_CVB_EXTRA_DATA, true,
				  auth_resp, sizeof(auth_resp),
				  HD2312_USB_TIMEOUT_MS);
		if (ret)
			return ret;
		dev_info(&d->udev->dev, "HD2312: CVB auth handshake completed\n");
	}

	return 0;
}

static int hd2312_power_ctrl(struct dvb_usb_device *d, int onoff)
{
	u8 req = onoff ? HD2312_REQ_POWER_ON : HD2312_REQ_POWER_OFF;

	return hd2312_ctrl(d, req, false, NULL, 0, HD2312_USB_TIMEOUT_MS);
}

static int hd2312_streaming_ctrl(struct dvb_frontend *fe, int onoff)
{
	struct dvb_usb_device *d = fe_to_d(fe);
	u8 req = onoff ? HD2312_REQ_STREAM_ON : HD2312_REQ_STREAM_OFF;

	return hd2312_ctrl(d, req, false, NULL, 0, HD2312_USB_TIMEOUT_MS);
}

static void hd2312_invalidate_stats(struct dvb_frontend *fe)
{
	struct hd2312_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;

	state->cached_strength = 0;
	state->cached_snr = 0;
	c->strength.len = 1;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->strength.stat[0].uvalue = 0;
	c->cnr.len = 1;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->cnr.stat[0].svalue = 0;
}

static int hd2312_init(struct dvb_frontend *fe)
{
	struct hd2312_state *state = fe->demodulator_priv;

	state->frequency = 0;
	hd2312_invalidate_stats(fe);
	return 0;
}

static int hd2312_sleep(struct dvb_frontend *fe)
{
	return hd2312_init(fe);
}

static int hd2312_set_frontend(struct dvb_frontend *fe)
{
	struct hd2312_state *state = fe->demodulator_priv;
	struct dvb_usb_device *d = fe_to_d(fe);
	u32 freq = fe->dtv_property_cache.frequency;
	u8 buf[4] = {
		(u8)(freq >> 24),
		(u8)(freq >> 16),
		(u8)(freq >> 8),
		(u8)freq,
	};
	int ret;

	state->frequency = 0;
	hd2312_invalidate_stats(fe);

	ret = hd2312_ctrl(d, HD2312_REQ_FREQUENCY, false, buf, sizeof(buf),
			  HD2312_USB_TIMEOUT_MS);
	if (ret)
		return ret;

	state->frequency = freq;
	return 0;
}

static int hd2312_get_frontend(struct dvb_frontend *fe,
			       struct dtv_frontend_properties *c)
{
	static const enum fe_transmit_mode transmission_modes[] = {
		TRANSMISSION_MODE_C1, TRANSMISSION_MODE_C3780,
	};
	static const enum fe_guard_interval guard_intervals[] = {
		GUARD_INTERVAL_PN945, GUARD_INTERVAL_PN595,
		GUARD_INTERVAL_PN420,
	};
	static const enum fe_code_rate code_rates[] = {
		FEC_2_5, FEC_3_5, FEC_4_5,
	};
	static const enum fe_interleaving interleavings[] = {
		INTERLEAVING_720, INTERLEAVING_240,
	};
	static const enum fe_modulation modulations[] = {
		QAM_4_NR, QPSK, QAM_16, QAM_32, QAM_64,
	};
	struct hd2312_state *state = fe->demodulator_priv;
	struct dvb_usb_device *d = fe_to_d(fe);
	u8 *buf = state->status;
	int ret;

	c->frequency = state->frequency;
	ret = hd2312_ctrl(d, HD2312_REQ_FRONTEND, true, buf, 6,
			  HD2312_USB_TIMEOUT_MS);
	if (ret)
		return ret;

	c->transmission_mode = (buf[0] < ARRAY_SIZE(transmission_modes)) ?
		transmission_modes[buf[0]] : TRANSMISSION_MODE_AUTO;
	c->guard_interval = (buf[1] < ARRAY_SIZE(guard_intervals)) ?
		guard_intervals[buf[1]] : GUARD_INTERVAL_AUTO;
	c->fec_inner = (buf[2] < ARRAY_SIZE(code_rates)) ?
		code_rates[buf[2]] : FEC_AUTO;
	c->interleaving = (buf[3] < ARRAY_SIZE(interleavings)) ?
		interleavings[buf[3]] : INTERLEAVING_AUTO;
	c->modulation = (buf[4] < ARRAY_SIZE(modulations)) ?
		modulations[buf[4]] : QAM_AUTO;
	c->inversion = (buf[5] == 0) ? INVERSION_ON :
		       ((buf[5] == 1) ? INVERSION_OFF : INVERSION_AUTO);
	return 0;
}

static int hd2312_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct hd2312_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct dvb_usb_device *d = fe_to_d(fe);
	u8 locked = 0;
	int ret;

	*status = 0;
	hd2312_invalidate_stats(fe);

	if (!state->frequency)
		return 0;

	/* CVB firmware can time out while waiting for a carrier. */
	ret = hd2312_ctrl(d, HD2312_REQ_LOCK, true, &locked, sizeof(locked),
			  HD2312_LOCK_TIMEOUT_MS);
	if (ret == -ETIMEDOUT)
		return 0;
	if (ret)
		return ret;
	if (locked != 1)
		return 0;

	*status = FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
		  FE_HAS_SYNC | FE_HAS_LOCK;

	ret = hd2312_ctrl(d, HD2312_REQ_STRENGTH, true,
			  state->statistics, 4, HD2312_USB_TIMEOUT_MS);
	if (ret)
		return ret;
	state->cached_strength =
		min_t(u8, state->statistics[3], 100) * 0xffffU / 100;
	c->strength.stat[0].scale = FE_SCALE_RELATIVE;
	c->strength.stat[0].uvalue = state->cached_strength;

	ret = hd2312_ctrl(d, HD2312_REQ_SNR, true,
			  state->statistics, 2, HD2312_USB_TIMEOUT_MS);
	if (ret)
		return ret;
	if (state->statistics[0] != 0xff && state->statistics[1] < 100) {
		u32 snr_mdb = state->statistics[0] * 1000 +
			      state->statistics[1] * 10;
		u32 scaled_mdb = min_t(u32, snr_mdb, HD2312_SNR_FULL_SCALE_MDB);

		c->cnr.stat[0].scale = FE_SCALE_RELATIVE;
		c->cnr.stat[0].uvalue =
			scaled_mdb * 0xffffU / HD2312_SNR_FULL_SCALE_MDB;
		state->cached_snr = c->cnr.stat[0].uvalue;
	}

	return 0;
}

static int hd2312_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct hd2312_state *state = fe->demodulator_priv;

	*strength = 0;
	if (fe->dtv_property_cache.strength.stat[0].scale != FE_SCALE_RELATIVE)
		return -EAGAIN;

	*strength = state->cached_strength;
	return 0;
}

static int hd2312_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct hd2312_state *state = fe->demodulator_priv;

	if (fe->dtv_property_cache.cnr.stat[0].scale != FE_SCALE_RELATIVE)
		return -EAGAIN;

	*snr = state->cached_snr;
	return 0;
}

static int hd2312_get_tune_settings(struct dvb_frontend *fe,
				    struct dvb_frontend_tune_settings *fe_tune_settings)
{
	fe_tune_settings->min_delay_ms = 800;
	fe_tune_settings->step_size = 0;
	fe_tune_settings->max_drift = 0;
	return 0;
}

static const struct dvb_frontend_ops hd2312_ops = {
	/* Retain the DVB-T userspace interface used by this DTMB adapter. */
	.delsys = { SYS_DVBT },
	.info = {
		.name = "HDIC HD2312 DTMB",
		.frequency_min_hz = 52 * MHz,
		.frequency_max_hz = 866 * MHz,
		.frequency_stepsize_hz = 10 * kHz,
		.caps = FE_CAN_FEC_AUTO | FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_BANDWIDTH_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO,
	},
	.init = hd2312_init,
	.sleep = hd2312_sleep,
	.set_frontend = hd2312_set_frontend,
	.get_tune_settings = hd2312_get_tune_settings,
	.get_frontend = hd2312_get_frontend,
	.read_status = hd2312_read_status,
	.read_signal_strength = hd2312_read_signal_strength,
	.read_snr = hd2312_read_snr,
};

static int hd2312_frontend_attach(struct dvb_usb_adapter *adap)
{
	struct hd2312_state *state = adap_to_priv(adap);

	memcpy(&state->fe.ops, &hd2312_ops, sizeof(state->fe.ops));
	state->fe.demodulator_priv = state;

	switch (state->vendor) {
	case HD2312_VENDOR_LETV:
		strscpy(state->fe.ops.info.name, "Letv HD2312 DTMB",
			sizeof(state->fe.ops.info.name));
		break;
	case HD2312_VENDOR_AIWA:
		strscpy(state->fe.ops.info.name, "Aiwa itv720 DTMB",
			sizeof(state->fe.ops.info.name));
		break;
	case HD2312_VENDOR_CVB:
		strscpy(state->fe.ops.info.name, "CVB T302A DTMB",
			sizeof(state->fe.ops.info.name));
		break;
	default:
		break;
	}

	adap->fe[0] = &state->fe;
	return 0;
}

static int hd2312_probe_device(struct dvb_usb_device *d)
{
	struct hd2312_state *state = d_to_priv(d);

	state->d = d;
	return hd2312_identify_vendor(d);
}

static const struct dvb_usb_device_properties hd2312_props = {
	.driver_name = KBUILD_MODNAME,
	.owner = THIS_MODULE,
	.adapter_nr = adapter_nr,
	.size_of_priv = sizeof(struct hd2312_state),
	.probe = hd2312_probe_device,
	.power_ctrl = hd2312_power_ctrl,
	.frontend_attach = hd2312_frontend_attach,
	.streaming_ctrl = hd2312_streaming_ctrl,
	.num_adapters = 1,
	.adapter = {
		{
			.stream = DVB_USB_STREAM_BULK(0x82, 8, 24064),
		},
	},
};

static const struct usb_device_id hd2312_id_table[] = {
	{ DVB_USB_DEVICE(HD2312_USB_VID, HD2312_USB_PID, &hd2312_props,
			 "HDIC HD2312 USB DTMB receiver", NULL) },
	{ }
};
MODULE_DEVICE_TABLE(usb, hd2312_id_table);

static struct usb_driver hd2312_usb_driver = {
	.name = KBUILD_MODNAME,
	.id_table = hd2312_id_table,
	.probe = dvb_usbv2_probe,
	.disconnect = dvb_usbv2_disconnect,
	.suspend = dvb_usbv2_suspend,
	.resume = dvb_usbv2_resume,
	.reset_resume = dvb_usbv2_reset_resume,
};

module_usb_driver(hd2312_usb_driver);

MODULE_AUTHOR("hanwckf <hanwckf@vip.qq.com>");
MODULE_DESCRIPTION("HDIC HD2312 USB2.0 DTMB receiver driver");
MODULE_LICENSE("GPL");
