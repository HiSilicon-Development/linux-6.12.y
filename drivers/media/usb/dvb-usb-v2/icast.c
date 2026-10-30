// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SMIT iCast DVB-C/DTMB USB receiver.
 *
 * CI/SAS bootstrap and protocol adaptation based on nxdong520/iCast and
 * wwek/dvb-cn-usb. Manufacturer-library provenance and physical timing
 * have not been independently validated for this source revision.
 *
 * Copyright (C) 2026 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/usb.h>
#include <linux/unaligned.h>
#include "dvb_usb.h"

#define ICAST_TIMEOUT_MS	500
#define ICAST_READY_INIT	100
#define ICAST_SAS_TIMEOUT_MS	5000
#define ICAST_MSG_TUNE		0x0003
#define ICAST_MSG_TUNE_ACK	0x0004
#define ICAST_MSG_STATUS		0x0005
#define ICAST_MSG_STATUS_RSP	0x0006
#define ICAST_MSG_HW_INFO	0x1007
#define ICAST_MSG_HW_INFO_RSP	0x1008

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

struct icast_state {
	struct dvb_usb_device *d;
	struct dvb_frontend fe;
	u32 frequency;
	u32 symbol_rate;
	enum fe_modulation modulation;
	enum fe_delivery_system delivery_system;
	u16 cached_strength;
	u16 cached_snr;
	u8 token;
	bool dtmb;
	bool initialized;
	bool tuned;
	bool disconnected;
	u8 buf[4096] __aligned(ARCH_DMA_MINALIGN);
	u8 reply[4096];
};

/* offset initially identifies the BER length byte. */
static int icast_decode_length(const u8 *buf, size_t len, size_t *offset,
			       size_t *length)
{
	u8 count;

	if (*offset >= len)
		return -EBADMSG;
	count = buf[(*offset)++];
	if (!(count & 0x80)) {
		*length = count;
		return 0;
	}
	count &= 0x7f;
	if (!count || count > 2 || count > len - *offset)
		return -EBADMSG;
	*length = buf[(*offset)++];
	if (count == 2)
		*length = (*length << 8) | buf[(*offset)++];
	return 0;
}

static int icast_parse_sas(const u8 *buf, size_t len, u16 *id,
			   const u8 **payload, size_t *payload_len)
{
	static const u8 header[] = { 1, 0x90, 2, 0, 3, 0x9f, 0x9a, 7 };
	size_t offset = sizeof(header), apdu_len;
	u16 message_len;
	int ret;

	if (len < sizeof(header) || memcmp(buf, header, sizeof(header)))
		return -ENOMSG;
	ret = icast_decode_length(buf, len, &offset, &apdu_len);
	if (ret)
		return ret;
	if (apdu_len < 7 || apdu_len != len - offset)
		return -EBADMSG;
	message_len = get_unaligned_be16(buf + offset + 1);
	*id = get_unaligned_be16(buf + offset + 3);
	*payload_len = get_unaligned_be16(buf + offset + 5);
	if (message_len != *payload_len + 4 || apdu_len != *payload_len + 7)
		return -EBADMSG;
	*payload = buf + offset + 7;
	return 0;
}

/* All command sequences hold d->usb_mutex, including ready/queue reads. */
static int icast_xfer(struct icast_state *state, int len)
{
	struct usb_device *udev = state->d->udev;
	int ret, actual;

	if (state->disconnected)
		return -ENODEV;
	ret = usb_bulk_msg(udev, usb_sndbulkpipe(udev, 0x01), state->buf,
			   len, &actual, ICAST_TIMEOUT_MS);
	if (ret)
		return ret;
	if (actual != len)
		return -EREMOTEIO;
	ret = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, 0x82), state->buf,
			   sizeof(state->buf), &actual, ICAST_TIMEOUT_MS);
	return ret ? ret : actual;
}

static int icast_frame(struct icast_state *state, size_t len,
		       const u8 **payload, size_t *payload_len, bool *complete)
{
	u8 *buf = state->buf;
	size_t offset = 3, end;
	int ret;

	if (len < 4 || buf[0] != 1 || buf[1] != 0)
		return -EBADMSG;
	ret = icast_decode_length(buf, len, &offset, payload_len);
	if (ret)
		return ret;
	if (*payload_len > len - offset)
		return -EBADMSG;
	end = offset + *payload_len;
	*payload = buf + offset;
	*complete = (buf[2] == 0x80 && *payload_len >= 2 &&
		     (*payload)[1] == 0x80) ||
		    (len - end >= 4 && buf[end] == 0x80 &&
		     buf[end + 3] == 0x80);
	return 0;
}

static int icast_ready(struct icast_state *state, unsigned int attempts)
{
	const u8 *payload;
	size_t payload_len;
	unsigned int i;
	bool complete;
	int ret;

	for (i = 0; i < attempts; i++) {
		memcpy(state->buf, "\x01\x00\xa0\x01\x01", 5);
		ret = icast_xfer(state, 5);
		if (ret < 0)
			return ret;
		ret = icast_frame(state, ret, &payload, &payload_len, &complete);
		if (ret)
			return ret;
		if (payload_len < 2)
			return -EBADMSG;
		if (payload[1] == 0x80)
			return 0;
		if (payload[1])
			return -EPROTO;
		if (i + 1 < attempts)
			msleep((i + 1) / 2 + 3);
	}
	return -ETIMEDOUT;
}

static int icast_command(struct icast_state *state, u8 target,
			 const u8 *data, u8 len, unsigned int attempts)
{
	const u8 *payload;
	size_t payload_len;
	bool complete;
	int ret;

	state->buf[0] = 1;
	state->buf[1] = 0;
	state->buf[2] = target;
	state->buf[3] = len;
	memcpy(state->buf + 4, data, len);
	ret = icast_xfer(state, len + 4);
	if (ret < 0)
		return ret;
	ret = icast_frame(state, ret, &payload, &payload_len, &complete);
	if (ret || complete || !attempts)
		return ret;
	msleep(100);
	return icast_ready(state, attempts);
}

static int icast_receive(struct icast_state *state, unsigned int attempts,
			 const u8 **payload, size_t *payload_len)
{
	bool complete;
	int ret;

	memcpy(state->buf, "\x01\x00\x81\x01\x01", 5);
	ret = icast_xfer(state, 5);
	if (ret < 0)
		return ret;
	ret = icast_frame(state, ret, payload, payload_len, &complete);
	if (ret)
		return ret;
	if (!complete && attempts) {
		/* Ready polling reuses the DMA buffer; retain this queued message. */
		memcpy(state->reply, *payload, *payload_len);
		*payload = state->reply;
		ret = icast_ready(state, attempts);
		if (ret)
			return ret;
	}
	msleep(50);
	return 0;
}

static int icast_wait_message(struct icast_state *state, u16 expected,
			      const u8 **payload, size_t *payload_len)
{
	const u8 *message;
	size_t len;
	unsigned long deadline;
	u16 id;
	int ret;

	/* The vendor waits 5 s; queued notifications are not timeouts. */
	deadline = jiffies + msecs_to_jiffies(ICAST_SAS_TIMEOUT_MS);
	do {
		/* A queued SAS reply need not carry the next-command ready flag. */
		ret = icast_receive(state, 0, &message, &len);
		if (ret)
			return ret;
		ret = icast_parse_sas(message, len, &id, payload, payload_len);
		if (ret == -ENOMSG)
			continue;
		if (ret)
			return ret;
		if (id == expected)
			return 0;
	} while (time_before(jiffies, deadline));
	return -ETIMEDOUT;
}

static void icast_build_message(u8 *buf, u8 token, u16 id, u16 len)
{
	static const u8 header[] = { 1, 0x90, 2, 0, 3, 0x9f, 0x9a, 7 };

	memcpy(buf, header, sizeof(header));
	buf[8] = len + 7;
	buf[9] = token;
	put_unaligned_be16(len + 4, buf + 10);
	put_unaligned_be16(id, buf + 12);
	put_unaligned_be16(len, buf + 14);
}

static int icast_query(struct icast_state *state, u16 id, const char *signature)
{
	u8 cmd[20];

	icast_build_message(cmd, state->token++, id, 4);
	memcpy(cmd + 16, signature, 4);
	return icast_command(state, 0xa0, cmd, sizeof(cmd), ICAST_READY_INIT);
}

/* GDHW fields are bounded ##-separated strings, not a GTYP bitmask. */
static int icast_parse_dtmb(const u8 *payload, size_t len, bool *dtmb)
{
	bool found = false, supported = false;

	if (!len || memchr(payload, '\0', len))
		return -EBADMSG;
	while (len) {
		const char *separator = strnstr((const char *)payload, "##", len);
		size_t field_len = separator ?
			separator - (const char *)payload : len;

		if (field_len >= 5 && !memcmp(payload, "DTMB:", 5)) {
			if (found || field_len != 6 ||
			    (payload[5] != 'Y' && payload[5] != 'N'))
				return -EBADMSG;
			supported = payload[5] == 'Y';
			found = true;
		}
		if (!separator)
			break;
		payload += field_len + 2;
		len -= field_len + 2;
	}
	if (!found)
		return -ENODATA;
	*dtmb = supported;
	return 0;
}

static void icast_invalidate(struct icast_state *state)
{
	struct dtv_frontend_properties *c = &state->fe.dtv_property_cache;

	state->cached_strength = 0;
	state->cached_snr = 0;
	c->strength.len = 1;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->strength.stat[0].uvalue = 0;
	c->cnr.len = 1;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->cnr.stat[0].uvalue = 0;
}

struct icast_init_step {
	u8 target;
	u8 len;
	u16 attempts;
	u8 reads;
	u8 read_attempts;
	u8 data[37];
};

/* Preserve the CI/SAS resource/session setup from the reference drivers. */
static const struct icast_init_step icast_init_steps[] = {
	{ 0x82, 1, 100, 1, 0, { 1 } },
	{ 0xa0, 10, 0, 0, 0,
	  { 1, 0x92, 7, 0, 0, 1, 0, 0x41, 0, 1 } },
	{ 0xa0, 9, 100, 1, 0,
	  { 1, 0x90, 2, 0, 1, 0x9f, 0x80, 0x10, 0 } },
	{ 0xa0, 9, 100, 1, 0,
	  { 1, 0x90, 2, 0, 1, 0x9f, 0x80, 0x12, 0 } },
	{ 0xa0, 37, 100, 1, 0,
	  { 1, 0x90, 2, 0, 1, 0x9f, 0x80, 0x11, 0x1c,
	    0, 1, 0, 0x41, 0, 2, 0, 0x41, 0, 3, 0, 0x41,
	    0, 0x24, 0, 0x41, 0, 0x40, 0, 0x41, 0, 0x96,
	    0x10, 1, 0, 0x20, 0, 0x41 } },
	{ 0xa0, 10, 0, 0, 0,
	  { 1, 0x92, 7, 0, 0, 2, 0, 0x41, 0, 2 } },
	{ 0xa0, 9, 100, 2, 0,
	  { 1, 0x90, 2, 0, 2, 0x9f, 0x80, 0x20, 0 } },
	/* This session completes only after the following SAS handshake. */
	{ 0xa0, 10, 0, 0, 0,
	  { 1, 0x92, 7, 0, 0, 0x96, 0x10, 1, 0, 3 } },
	{ 0xa0, 17, 100, 2, 100,
	  { 1, 0x90, 2, 0, 3, 0x9f, 0x9a, 0, 8,
	    'S', 'M', 'i', 'T', 'Z', 'B', 'J', 'L' } },
	{ 0xa0, 10, 300, 1, 0,
	  { 1, 0x92, 7, 0, 0, 0x96, 0x10, 1, 0, 4 } },
};

static int icast_initialize(struct icast_state *state)
{
	static const u8 session[] = { 1, 0x92, 7, 0, 0, 3, 0, 0x41, 0, 5 };
	static const u8 open[] = { 1, 0x90, 2, 0, 5, 0x9f, 0x80, 0x30, 0 };
	const u8 *payload;
	size_t payload_len;
	unsigned int i, j;
	int ret;

	state->initialized = false;
	state->dtmb = false;
	state->tuned = false;
	state->frequency = 0;
	icast_invalidate(state);
	memcpy(state->buf, "\xfe\x00\x10\x00", 4);
	ret = icast_xfer(state, 4);
	if (ret < 0)
		return ret;
	if (ret != 4)
		return -EBADMSG;
	msleep(100);
	for (i = 0; i < ARRAY_SIZE(icast_init_steps); i++) {
		const struct icast_init_step *step = &icast_init_steps[i];

		ret = icast_command(state, step->target, step->data, step->len,
				    step->attempts);
		if (ret)
			return ret;
		for (j = 0; j < step->reads; j++) {
			ret = icast_receive(state, j ? 0 : step->read_attempts,
					    &payload, &payload_len);
			if (ret)
				return ret;
		}
	}
	state->token = 1;
	ret = icast_query(state, ICAST_MSG_HW_INFO, "GDHW");
	if (ret)
		return ret;
	ret = icast_wait_message(state, ICAST_MSG_HW_INFO_RSP,
				 &payload, &payload_len);
	if (ret)
		return ret;
	ret = icast_parse_dtmb(payload, payload_len, &state->dtmb);
	if (ret)
		return ret;
	ret = icast_command(state, 0xa0, session, sizeof(session), 0);
	if (ret)
		return ret;
	ret = icast_command(state, 0xa0, open, sizeof(open), 0);
	if (ret)
		return ret;
	state->initialized = true;
	dev_info(&state->d->udev->dev, "iCast: %s (GDHW DTMB:%c)\n",
		 state->dtmb ? "DTMB (DVB-T API) and DVB-C" : "DVB-C",
		 state->dtmb ? 'Y' : 'N');
	return 0;
}

static int icast_read_config(struct dvb_usb_device *d)
{
	struct icast_state *state = d_to_priv(d);
	int ret;

	mutex_lock(&d->usb_mutex);
	ret = icast_initialize(state);
	if (ret)
		state->initialized = false;
	mutex_unlock(&d->usb_mutex);
	return ret;
}

static int icast_fe_init(struct dvb_frontend *fe)
{
	struct icast_state *state = fe->demodulator_priv;
	int ret;

	mutex_lock(&state->d->usb_mutex);
	state->tuned = false;
	state->frequency = 0;
	icast_invalidate(state);
	ret = state->disconnected ? -ENODEV :
		state->initialized ? 0 : -EIO;
	mutex_unlock(&state->d->usb_mutex);
	return ret;
}

static int icast_set_frontend(struct dvb_frontend *fe)
{
	struct icast_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	const u8 *payload;
	size_t payload_len;
	u32 symbol_rate = 0, modulation = 0;
	u8 cmd[32] = { 0 };
	int ret;

	mutex_lock(&state->d->usb_mutex);
	state->tuned = false;
	state->frequency = 0;
	icast_invalidate(state);
	ret = -ENODEV;
	if (state->disconnected)
		goto out;
	ret = -EIO;
	if (!state->initialized)
		goto out;
	ret = -EINVAL;
	if (c->frequency < 10 * MHz || c->frequency > 862 * MHz)
		goto out;
	if (c->delivery_system == SYS_DVBC_ANNEX_A) {
		if (c->symbol_rate < 1000000 || c->symbol_rate > 45000000)
			goto out;
		symbol_rate = c->symbol_rate / 1000;
		switch (c->modulation) {
		case QAM_16:
			modulation = 16;
			break;
		case QAM_32:
			modulation = 32;
			break;
		case QAM_64:
			modulation = 64;
			break;
		case QAM_128:
			modulation = 128;
			break;
		case QAM_256:
			modulation = 256;
			break;
		default:
			goto out;
		}
	} else if (c->delivery_system == SYS_DVBT) {
		if (!state->dtmb)
			goto out;
		if (c->bandwidth_hz && c->bandwidth_hz != 8 * MHz)
			goto out;
	} else {
		goto out;
	}
	icast_build_message(cmd, state->token++, ICAST_MSG_TUNE, 16);
	put_unaligned_le32(c->frequency / 1000, cmd + 16);
	put_unaligned_le32(symbol_rate, cmd + 20);
	put_unaligned_le32(modulation, cmd + 24);
	cmd[28] = 8;
	ret = icast_command(state, 0xa0, cmd, sizeof(cmd), ICAST_READY_INIT);
	if (ret)
		goto out;
	ret = icast_wait_message(state, ICAST_MSG_TUNE_ACK,
				 &payload, &payload_len);
	if (ret)
		goto out;
	if (payload_len != 4) {
		ret = -EBADMSG;
		goto out;
	}
	if (get_unaligned_be32(payload)) {
		ret = -EREMOTEIO;
		goto out;
	}
	state->frequency = c->frequency;
	state->symbol_rate = symbol_rate * 1000;
	state->modulation = c->delivery_system == SYS_DVBT ? QAM_AUTO :
			    c->modulation;
	state->delivery_system = c->delivery_system;
	state->tuned = true;
out:
	mutex_unlock(&state->d->usb_mutex);
	return ret;
}

static int icast_get_frontend(struct dvb_frontend *fe,
			      struct dtv_frontend_properties *props)
{
	struct icast_state *state = fe->demodulator_priv;

	mutex_lock(&state->d->usb_mutex);
	if (state->tuned) {
		props->frequency = state->frequency;
		props->symbol_rate = state->symbol_rate;
		props->modulation = state->modulation;
		if (state->delivery_system == SYS_DVBT)
			props->bandwidth_hz = 8 * MHz;
	}
	mutex_unlock(&state->d->usb_mutex);
	return 0;
}

static int icast_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct icast_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	const u8 *payload;
	size_t payload_len;
	int ret = 0;

	*status = 0;
	mutex_lock(&state->d->usb_mutex);
	icast_invalidate(state);
	if (state->disconnected) {
		ret = -ENODEV;
		goto out;
	}
	if (!state->tuned)
		goto out;
	ret = icast_query(state, ICAST_MSG_STATUS, "GSTA");
	if (ret)
		goto out;
	ret = icast_wait_message(state, ICAST_MSG_STATUS_RSP,
				 &payload, &payload_len);
	if (ret)
		goto out;
	if (payload_len != 16) {
		ret = -EBADMSG;
		goto out;
	}
	if (get_unaligned_le32(payload) != state->frequency / 1000) {
		ret = -EAGAIN;
		goto out;
	}
	if (payload[13]) {
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
			  FE_HAS_SYNC | FE_HAS_LOCK;
		if (payload[14] <= 100) {
			state->cached_strength = payload[14] * 0xffffU / 100;
			c->strength.stat[0].scale = FE_SCALE_RELATIVE;
			c->strength.stat[0].uvalue = state->cached_strength;
		}
		if (payload[15] <= 100) {
			state->cached_snr = payload[15] * 0xffffU / 100;
			c->cnr.stat[0].scale = FE_SCALE_RELATIVE;
			c->cnr.stat[0].uvalue = state->cached_snr;
		}
	}
out:
	mutex_unlock(&state->d->usb_mutex);
	return ret;
}

static int icast_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct icast_state *state = fe->demodulator_priv;
	int ret;

	mutex_lock(&state->d->usb_mutex);
	ret = fe->dtv_property_cache.strength.stat[0].scale ==
		FE_SCALE_RELATIVE ? 0 : -EAGAIN;
	*strength = state->cached_strength;
	mutex_unlock(&state->d->usb_mutex);
	return ret;
}

static int icast_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct icast_state *state = fe->demodulator_priv;
	int ret;

	mutex_lock(&state->d->usb_mutex);
	ret = fe->dtv_property_cache.cnr.stat[0].scale ==
		FE_SCALE_RELATIVE ? 0 : -EAGAIN;
	*snr = state->cached_snr;
	mutex_unlock(&state->d->usb_mutex);
	return ret;
}

static int icast_get_tune_settings(struct dvb_frontend *fe,
				   struct dvb_frontend_tune_settings *settings)
{
	settings->min_delay_ms = 800;
	settings->step_size = 0;
	settings->max_drift = 0;
	return 0;
}

static enum dvbfe_algo icast_get_frontend_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}

static int icast_tune(struct dvb_frontend *fe, bool re_tune,
		      unsigned int mode_flags, unsigned int *delay,
		      enum fe_status *status)
{
	int ret;

	*delay = msecs_to_jiffies(800);
	*status = 0;
	if (re_tune) {
		ret = icast_set_frontend(fe);
		if (ret)
			return ret;
	}
	return icast_read_status(fe, status);
}

/* Cable-only by default; GDHW enables the DTMB-compatible DVB-T API. */
static const struct dvb_frontend_ops icast_ops = {
	.delsys = { SYS_DVBC_ANNEX_A },
	.info = {
		.name = "SMIT iCast DVB-C",
		.frequency_min_hz = 10 * MHz,
		.frequency_max_hz = 862 * MHz,
		.frequency_stepsize_hz = 10 * kHz,
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 45000000,
		.caps = FE_CAN_QAM_16 | FE_CAN_QAM_32 | FE_CAN_QAM_64 |
			FE_CAN_QAM_128 | FE_CAN_QAM_256 | FE_CAN_FEC_AUTO,
	},
	.init = icast_fe_init,
	.sleep = icast_fe_init,
	.set_frontend = icast_set_frontend,
	.get_frontend = icast_get_frontend,
	.get_frontend_algo = icast_get_frontend_algo,
	.get_tune_settings = icast_get_tune_settings,
	.tune = icast_tune,
	.read_status = icast_read_status,
	.read_signal_strength = icast_read_signal_strength,
	.read_snr = icast_read_snr,
};

static int icast_frontend_attach(struct dvb_usb_adapter *adap)
{
	struct icast_state *state = adap_to_priv(adap);

	state->fe.ops = icast_ops;
	if (state->dtmb) {
		state->fe.ops.delsys[0] = SYS_DVBT;
		state->fe.ops.delsys[1] = SYS_DVBC_ANNEX_A;
		strscpy(state->fe.ops.info.name, "SMIT iCast DTMB/DVB-C",
			sizeof(state->fe.ops.info.name));
		state->fe.ops.info.caps |= FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO;
	}
	state->fe.demodulator_priv = state;
	adap->fe[0] = &state->fe;
	return 0;
}

static int icast_probe(struct dvb_usb_device *d)
{
	struct icast_state *state = d_to_priv(d);

	state->d = d;
	return 0;
}

static void icast_exit(struct dvb_usb_device *d)
{
	struct icast_state *state = d_to_priv(d);

	mutex_lock(&d->usb_mutex);
	state->disconnected = true;
	state->initialized = false;
	state->tuned = false;
	icast_invalidate(state);
	mutex_unlock(&d->usb_mutex);
}

static int icast_reset_resume(struct usb_interface *intf)
{
	struct dvb_usb_device *d = usb_get_intfdata(intf);
	int ret;

	ret = icast_read_config(d);
	if (ret)
		return ret;
	return dvb_usbv2_resume(intf);
}

static const struct dvb_usb_device_properties icast_props = {
	.driver_name = KBUILD_MODNAME,
	.owner = THIS_MODULE,
	.adapter_nr = adapter_nr,
	.size_of_priv = sizeof(struct icast_state),
	.probe = icast_probe,
	.read_config = icast_read_config,
	.frontend_attach = icast_frontend_attach,
	.exit = icast_exit,
	.num_adapters = 1,
	.adapter = {
		{ .stream = DVB_USB_STREAM_BULK(0x84, 8, 8192) },
	},
};

static const struct usb_device_id icast_id_table[] = {
	{ DVB_USB_DEVICE(0x29df, 0x0001, &icast_props,
			 "SMIT iCast USB receiver", NULL) },
	{ }
};
MODULE_DEVICE_TABLE(usb, icast_id_table);

static struct usb_driver icast_usb_driver = {
	.name = KBUILD_MODNAME,
	.id_table = icast_id_table,
	.probe = dvb_usbv2_probe,
	.disconnect = dvb_usbv2_disconnect,
	.suspend = dvb_usbv2_suspend,
	.resume = dvb_usbv2_resume,
	.reset_resume = icast_reset_resume,
};
module_usb_driver(icast_usb_driver);

MODULE_AUTHOR("Xiaodong Ni <nxiaodong520@gmail.com>");
MODULE_DESCRIPTION("SMIT iCast DVB-C/DTMB USB receiver driver");
MODULE_LICENSE("GPL");
