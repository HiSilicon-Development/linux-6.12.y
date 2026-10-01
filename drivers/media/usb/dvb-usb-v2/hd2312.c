// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Cypress FX2 / HDIC HD2312 DTMB receiver driver
 *
 * Copyright (C) 2024 hanwckf <hanwckf@vip.qq.com>
 * Copyright (C) 2023 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#include <linux/delay.h>
#include "dvb_usb.h"

#define HD2312_USB_TIMEOUT_MS	1000
#define HD2312_USB_VID		0x04b4
#define HD2312_USB_PID		0x1004

#define HD2312_REQ_CVB_EXTRA_AUTH	0xbe
#define HD2312_REQ_CVB_EXTRA_DATA	0xbf
#define HD2312_REQ_STREAM_ON	0xab
#define HD2312_REQ_STREAM_OFF	0xac
#define HD2312_REQ_POWER_ON	0xad
#define HD2312_REQ_POWER_OFF	0xae
#define HD2312_REQ_FRONTEND	0xe7
#define HD2312_REQ_STRENGTH	0xeb
#define HD2312_REQ_SNR		0xe8
#define HD2312_REQ_LOCK		0xea
#define HD2312_REQ_VERSION	0xed
#define HD2312_REQ_FREQUENCY	0xfc

struct hd2312_state {
	struct dvb_frontend fe;
	u32 frequency;
	unsigned long last_stats_jiffies;
	u16 cached_strength;
	u16 cached_snr;
	u8 status[6];
	u8 statistics[4];
	u8 version[4];
	bool cvb_firmware;
};
DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static int hd2312_ctrl(struct dvb_usb_device *d, u8 request, bool read,
		       void *data, u16 length)
{
	unsigned int pipe = read ? usb_rcvctrlpipe(d->udev, 0) :
				   usb_sndctrlpipe(d->udev, 0);
	u8 request_type = USB_TYPE_VENDOR | (read ? USB_DIR_IN : USB_DIR_OUT);
	void *usb_buf = NULL;
	int ret;

	if (length) {
		usb_buf = read ? kmalloc(length, GFP_KERNEL) :
				 kmemdup(data, length, GFP_KERNEL);
		if (!usb_buf)
			return -ENOMEM;
	}

	if (mutex_lock_interruptible(&d->usb_mutex)) {
		ret = -EAGAIN;
		goto out_free;
	}

	ret = usb_control_msg(d->udev, pipe, request, request_type, 0x00fe, 0,
			      usb_buf, length, HD2312_USB_TIMEOUT_MS);
	mutex_unlock(&d->usb_mutex);

	if (ret < 0)
		goto out_free;
	if (ret != length) {
		ret = -EREMOTEIO;
		goto out_free;
	}
	if (read && length)
		memcpy(data, usb_buf, length);
	ret = 0;

out_free:
	kfree(usb_buf);
	return ret;
}

static int hd2312_command(struct dvb_usb_device *d, u8 request)
{
	return hd2312_ctrl(d, request, false, NULL, 0);
}

static int hd2312_probe_device(struct dvb_usb_device *d)
{
	static const u8 cvb_key[] = "cidanaKEY";
	struct hd2312_state *state = d_to_priv(d);
	u8 version[4];
	u8 hash[22];
	int ret;

	ret = hd2312_ctrl(d, HD2312_REQ_VERSION, true, version, sizeof(version));
	if (ret)
		return dev_err_probe(&d->udev->dev, ret,
				     "failed to read firmware version\n");

	memcpy(state->version, version, sizeof(version));
	state->frequency = 0;
	state->cvb_firmware = false;

	/*
	 * CVB (Cidana) DTMB dongles across different production runs
	 * respond to 0xBE ("cidanaKEY") and return a 22-byte serial/auth hash.
	 */
	ret = hd2312_ctrl(d, HD2312_REQ_CVB_EXTRA_AUTH, false,
			  (void *)cvb_key, sizeof(cvb_key) - 1);
	if (!ret) {
		ret = hd2312_ctrl(d, HD2312_REQ_CVB_EXTRA_DATA, true,
				  hash, sizeof(hash));
		if (!ret && (hash[0] >= 0x20 && hash[0] <= 0x7e)) {
			state->cvb_firmware = true;
			dev_info(&d->udev->dev,
				 "CVB HD2312 authenticated: SN=%.22s (fw %u.%u.%u%u)\n",
				 hash, version[0], version[1], version[2], version[3]);
		}
	}

	if (!state->cvb_firmware)
		dev_info(&d->udev->dev, "firmware %u.%u.%u%u\n",
			 version[0], version[1], version[2], version[3]);

	return 0;
}

static int hd2312_power_ctrl(struct dvb_usb_device *d, int onoff)
{
	return hd2312_command(d, onoff ? HD2312_REQ_POWER_ON :
					 HD2312_REQ_POWER_OFF);
}

static int hd2312_streaming_ctrl(struct dvb_frontend *fe, int onoff)
{
	struct dvb_usb_device *d = fe_to_d(fe);

	return hd2312_command(d, onoff ? HD2312_REQ_STREAM_ON :
					 HD2312_REQ_STREAM_OFF);
}

static int hd2312_init(struct dvb_frontend *fe)
{
	struct hd2312_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;

	state->frequency = 0;
	c->strength.len = 1;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->cnr.len = 1;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	return 0;
}

static int hd2312_set_frontend(struct dvb_frontend *fe)
{
	struct hd2312_state *state = fe->demodulator_priv;
	struct dvb_usb_device *d = fe_to_d(fe);
	u32 frequency = fe->dtv_property_cache.frequency;
	u8 buf[4] = {
		frequency >> 24,
		frequency >> 16,
		frequency >> 8,
		frequency,
	};
	int ret;

	ret = hd2312_ctrl(d, HD2312_REQ_FREQUENCY, false, buf, sizeof(buf));
	if (ret)
		return ret;
	state->frequency = frequency;
	state->last_stats_jiffies = 0;
	state->cached_snr = 0;
	state->cached_strength = 0;
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
	ret = hd2312_ctrl(d, HD2312_REQ_FRONTEND, true, buf, 6);
	if (ret)
		return 0;

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

static u16 hd2312_quality_to_u16(u8 snr)
{
	u16 quality = (u16)snr + 3;

	/* Match the legacy HD2312 quality mapping used by DtmbUSB. */
	if (quality >= 70)
		return 0;
	return quality * 0xffffU / 100;
}

static int hd2312_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct hd2312_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct dvb_usb_device *d = fe_to_d(fe);
	unsigned long interval = state->cached_snr ? (HZ * 2) : (HZ / 10);
	int ret;

	*status = 0;
	if (!state->frequency)
		return 0;

	if (!state->last_stats_jiffies ||
	    time_after(jiffies, state->last_stats_jiffies + interval)) {
		state->last_stats_jiffies = jiffies;
		ret = hd2312_ctrl(d, HD2312_REQ_STRENGTH, true,
				  state->statistics, 4);
		if (!ret) {
			c->strength.stat[0].scale = FE_SCALE_RELATIVE;
			c->strength.stat[0].uvalue =
				min_t(u8, state->statistics[3], 100) * 0xffffU / 100;
			state->cached_strength = c->strength.stat[0].uvalue;
		}

		ret = hd2312_ctrl(d, HD2312_REQ_SNR, true,
				  state->statistics, 2);
		if (!ret) {
			c->cnr.stat[0].scale = FE_SCALE_RELATIVE;
			c->cnr.stat[0].uvalue =
				hd2312_quality_to_u16(state->statistics[0]);
			state->cached_snr = c->cnr.stat[0].uvalue;
		}
	}

	if (state->cached_snr > 0)
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
			  FE_HAS_SYNC | FE_HAS_LOCK;
	else if (state->cached_strength > 0)
		*status = FE_HAS_SIGNAL;

	return 0;
}

static int hd2312_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct hd2312_state *state = fe->demodulator_priv;

	*strength = state->cached_strength;
	return 0;
}

static int hd2312_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct hd2312_state *state = fe->demodulator_priv;

	*snr = state->cached_snr;
	return 0;
}

static int hd2312_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	*ucblocks = 0;
	return 0;
}

static int hd2312_get_tune_settings(
	struct dvb_frontend *fe,
	struct dvb_frontend_tune_settings *fe_tune_settings)
{
	fe_tune_settings->min_delay_ms = 800;
	fe_tune_settings->step_size = 0;
	fe_tune_settings->max_drift = 0;
	return 0;
}

static const struct dvb_frontend_ops hd2312_ops = {
	.delsys = { SYS_DVBT },
	.info = {
		.name = "HDIC HD2312 DTMB",
		.frequency_min_hz = 52 * MHz,
		.frequency_max_hz = 866 * MHz,
		.frequency_stepsize_hz = 10 * kHz,
		.caps = FE_CAN_FEC_AUTO | FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_BANDWIDTH_AUTO | FE_CAN_GUARD_INTERVAL_AUTO |
			FE_CAN_HIERARCHY_AUTO,
	},
	.init = hd2312_init,
	.set_frontend = hd2312_set_frontend,
	.get_tune_settings = hd2312_get_tune_settings,
	.get_frontend = hd2312_get_frontend,
	.read_status = hd2312_read_status,
	.read_signal_strength = hd2312_read_signal_strength,
	.read_snr = hd2312_read_snr,
	.read_ucblocks = hd2312_read_ucblocks,
};

static int hd2312_frontend_attach(struct dvb_usb_adapter *adap)
{
	struct hd2312_state *state = adap_to_priv(adap);

	memcpy(&state->fe.ops, &hd2312_ops, sizeof(state->fe.ops));
	state->fe.demodulator_priv = state;

	if (state->cvb_firmware)
		strscpy(state->fe.ops.info.name, "CVB HD2312 DTMB",
			sizeof(state->fe.ops.info.name));

	adap->fe[0] = &state->fe;
	return 0;
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
			 "HDIC HD2312 DTMB receiver", NULL) },
	{ }
};
MODULE_DEVICE_TABLE(usb, hd2312_id_table);

static struct usb_driver hd2312_driver = {
	.name = KBUILD_MODNAME,
	.id_table = hd2312_id_table,
	.probe = dvb_usbv2_probe,
	.disconnect = dvb_usbv2_disconnect,
	.suspend = dvb_usbv2_suspend,
	.resume = dvb_usbv2_resume,
	.reset_resume = dvb_usbv2_reset_resume,
	.no_dynamic_id = 1,
	.soft_unbind = 1,
};
module_usb_driver(hd2312_driver);

MODULE_AUTHOR("hanwckf <hanwckf@vip.qq.com>");
MODULE_AUTHOR("Xiaodong Ni <nxiaodong520@gmail.com>");
MODULE_DESCRIPTION("HDIC HD2312 USB2.0 DTMB receiver driver");
MODULE_LICENSE("GPL");
