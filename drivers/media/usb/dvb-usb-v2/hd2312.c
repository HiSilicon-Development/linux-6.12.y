// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Cypress FX2 / HDIC HD2312 DTMB receiver driver
 *
 * Copyright (C) 2024 hanwckf <hanwckf@vip.qq.com>
 * Copyright (C) 2023 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#include "dvb_usb.h"

#define HD2312_USB_TIMEOUT_MS	2000
#define HD2312_USB_VID		0x04b4
#define HD2312_USB_PID		0x1004

#define HD2312_REQ_CVB_KEY	0xbe
#define HD2312_REQ_CVB_HASH	0xbf
#define HD2312_REQ_STREAM_ON	0xab
#define HD2312_REQ_STREAM_OFF	0xac
#define HD2312_REQ_POWER_ON	0xad
#define HD2312_REQ_POWER_OFF	0xae
#define HD2312_REQ_FRONTEND	0xe7
#define HD2312_REQ_SNR		0xe8
#define HD2312_REQ_LOCK		0xea
#define HD2312_REQ_STRENGTH	0xeb
#define HD2312_REQ_VERSION	0xed
#define HD2312_REQ_FREQUENCY	0xfc

enum hd2312_variant {
	HD2312_GENERIC,
	HD2312_LETV,
	HD2312_AIWA,
	HD2312_CVB,
};

struct hd2312_state {
	struct dvb_frontend fe;
	enum hd2312_variant variant;
	u32 frequency;
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
	u8 hash[22];
	u8 version[4];
	int ret;

	ret = hd2312_ctrl(d, HD2312_REQ_VERSION, true, version,
			   sizeof(version));
	if (ret)
		return dev_err_probe(&d->udev->dev, ret,
				     "failed to read firmware version\n");

	state->variant = HD2312_GENERIC;
	if (version[1] == 0x08 && version[2] == 0x20 && version[3] == 0x44) {
		switch (version[0]) {
		case 0x03:
			state->variant = HD2312_LETV;
			break;
		case 0x05:
			state->variant = HD2312_AIWA;
			break;
		case 0x06:
			state->variant = HD2312_CVB;
			break;
		default:
			break;
		}
	}

	if (state->variant == HD2312_CVB) {
		ret = hd2312_ctrl(d, HD2312_REQ_CVB_KEY, false,
				   (u8 *)cvb_key, sizeof(cvb_key) - 1);
		if (ret)
			return dev_err_probe(&d->udev->dev, ret,
					     "failed to send CVB authorization key\n");

		ret = hd2312_ctrl(d, HD2312_REQ_CVB_HASH, true, hash,
				   sizeof(hash));
		if (ret)
			return dev_err_probe(&d->udev->dev, ret,
					     "failed to read CVB authorization response\n");
	}

	dev_info(&d->udev->dev, "firmware %u.%u.%u%u, variant %u\n",
		 version[0], version[1], version[2], version[3], state->variant);

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
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;

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
	if (!ret)
		state->frequency = frequency;

	return ret;
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
	u8 buf[6];
	int ret;

	ret = hd2312_ctrl(d, HD2312_REQ_FRONTEND, true, buf, sizeof(buf));
	if (ret)
		return ret;

	c->frequency = state->frequency;
	c->transmission_mode = buf[0] < ARRAY_SIZE(transmission_modes) ?
		transmission_modes[buf[0]] : TRANSMISSION_MODE_AUTO;
	c->guard_interval = buf[1] < ARRAY_SIZE(guard_intervals) ?
		guard_intervals[buf[1]] : GUARD_INTERVAL_AUTO;
	c->fec_inner = buf[2] < ARRAY_SIZE(code_rates) ?
		code_rates[buf[2]] : FEC_AUTO;
	c->interleaving = buf[3] < ARRAY_SIZE(interleavings) ?
		interleavings[buf[3]] : INTERLEAVING_AUTO;
	c->modulation = buf[4] < ARRAY_SIZE(modulations) ?
		modulations[buf[4]] : QAM_AUTO;
	c->inversion = buf[5] == 0 ? INVERSION_ON :
			(buf[5] == 1 ? INVERSION_OFF : INVERSION_AUTO);

	return 0;
}

static int hd2312_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct hd2312_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct dvb_usb_device *d = fe_to_d(fe);
	u8 strength[4];
	u8 snr[2];
	u8 lock;
	int ret;

	*status = 0;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	ret = hd2312_ctrl(d, HD2312_REQ_LOCK, true, &lock, sizeof(lock));
	if (ret)
		return ret;
	if (lock != 1)
		return 0;

	*status = FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
		  FE_HAS_SYNC | FE_HAS_LOCK;

	ret = hd2312_ctrl(d, HD2312_REQ_STRENGTH, true, strength,
			   sizeof(strength));
	if (!ret) {
		if (state->variant == HD2312_CVB) {
			c->strength.stat[0].scale = FE_SCALE_DECIBEL;
			c->strength.stat[0].svalue = -(s64)strength[3] * 1000;
		} else {
			c->strength.stat[0].scale = FE_SCALE_RELATIVE;
			c->strength.stat[0].uvalue = min_t(u8, strength[3], 100) *
						     0xffffU / 100;
		}
	}

	ret = hd2312_ctrl(d, HD2312_REQ_SNR, true, snr, sizeof(snr));
	if (!ret) {
		c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
		c->cnr.stat[0].svalue = (s64)snr[0] * 1000 + snr[1] * 10;
	}

	return 0;
}

static const struct dvb_frontend_ops hd2312_ops = {
	.delsys = { SYS_DTMB },
	.info = {
		.name = "HDIC HD2312 DTMB",
		.frequency_min_hz = 52 * MHz,
		.frequency_max_hz = 866 * MHz,
		.frequency_stepsize_hz = 10 * kHz,
		.caps = FE_CAN_FEC_AUTO | FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO | FE_CAN_GUARD_INTERVAL_AUTO,
	},
	.init = hd2312_init,
	.set_frontend = hd2312_set_frontend,
	.get_frontend = hd2312_get_frontend,
	.read_status = hd2312_read_status,
};

static int hd2312_frontend_attach(struct dvb_usb_adapter *adap)
{
	struct hd2312_state *state = adap_to_priv(adap);

	memcpy(&state->fe.ops, &hd2312_ops, sizeof(state->fe.ops));
	state->fe.demodulator_priv = state;

	switch (state->variant) {
	case HD2312_LETV:
		strscpy(state->fe.ops.info.name, "Letv HD2312 DTMB",
			sizeof(state->fe.ops.info.name));
		break;
	case HD2312_AIWA:
		strscpy(state->fe.ops.info.name, "Aiwa HD2312 DTMB",
			sizeof(state->fe.ops.info.name));
		break;
	case HD2312_CVB:
		strscpy(state->fe.ops.info.name, "CVB HD2312 DTMB",
			sizeof(state->fe.ops.info.name));
		break;
	default:
		break;
	}

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
			.stream = DVB_USB_STREAM_BULK(0x82, 8, 4096),
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
