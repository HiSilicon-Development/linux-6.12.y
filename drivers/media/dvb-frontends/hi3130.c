// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hi3130 external v200 DVB-C Annex A receiver.
 *
 * Independently written Linux integration, using the register descriptions,
 * programming sequence and numerical hardware data in HiSilicon's
 * HiSTBLinuxV100R005C00SPC050 frontend driver. See hi3130-priv.h for provenance.
 * The supported external revision is identified by MCTRL_1[7:5] == 5.
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/int_log.h>
#include <linux/kref.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <media/dvb_frontend.h>
#include <media/hi3130.h>
#include <media/r858.h>

#include "hi3130-priv.h"

#define HI3130_BANK 0xe0
#define HI3130_MCTRL1 0x40
#define HI3130_STATUS 0x4a
#define HI3130_XTAL_HZ 28800000

struct hi3130_state {
	struct i2c_client *client;
	struct i2c_adapter *tuner_adapter;
	struct gpio_desc *reset;
	struct mutex lock;
	struct kref ref;
	struct dvb_frontend fe;
	struct r858_config tuner;
	void (*tuner_release)(struct dvb_frontend *fe);
	bool claimed;
	bool removed;
	bool initialized;
	bool tuned;
	u8 modulation;
};

static struct i2c_driver hi3130_driver;

static int hi3130_write(struct hi3130_state *s, u8 reg, u8 val)
{
	u8 buf[] = { reg, val };
	int ret = i2c_master_send(s->client, buf, sizeof(buf));

	return ret == sizeof(buf) ? 0 : ret < 0 ? ret : -EREMOTEIO;
}

static int hi3130_read(struct hi3130_state *s, u8 reg, u8 *val)
{
	struct i2c_msg msg[] = {
		{ .addr = s->client->addr, .len = 1, .buf = &reg },
		{ .addr = s->client->addr,
		  .flags = I2C_M_RD,
		  .len = 1,
		  .buf = val },
	};
	int ret = i2c_transfer(s->client->adapter, msg, ARRAY_SIZE(msg));

	return ret == ARRAY_SIZE(msg) ? 0 : ret < 0 ? ret : -EREMOTEIO;
}

static int hi3130_update(struct hi3130_state *s, u8 reg, u8 mask, u8 val)
{
	u8 old;
	int ret = hi3130_read(s, reg, &old);

	if (ret)
		return ret;
	return hi3130_write(s, reg, (old & ~mask) | (val & mask));
}

static int hi3130_write_regs(struct hi3130_state *s,
			     const struct hi3130_reg *regs, size_t count)
{
	int ret;
	size_t i;

	for (i = 0; i < count; i++) {
		ret = hi3130_write(s, regs[i].addr, regs[i].val);
		if (ret)
			return ret;
	}
	return 0;
}

static void hi3130_stats_reset(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;

	c->strength.len = 1;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->cnr.len = 1;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->pre_bit_error.len = 1;
	c->pre_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->pre_bit_count.len = 1;
	c->pre_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->post_bit_error.len = 1;
	c->post_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->post_bit_count.len = 1;
	c->post_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_error.len = 1;
	c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_count.len = 1;
	c->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
}

static int hi3130_identify(struct hi3130_state *s)
{
	u8 id;
	int ret = hi3130_read(s, HI3130_MCTRL1, &id);

	if (ret)
		return ret;
	if ((id >> 5) != 5) {
		dev_err(&s->client->dev,
			"unsupported external Hi3130 revision: MCTRL1=%#02x\n",
			id);
		return -ENODEV;
	}
	return 0;
}

static int hi3130_hw_init(struct hi3130_state *s)
{
	/*
	 * Zero selects the standard TS pad assignment in IOSHARE_CTRL4..9
	 * for data0..7, valid, sync and clock.
	 */
	static const struct hi3130_reg ts_pins[] = {
		{ 0x24, 0x00 }, { 0x25, 0x00 }, { 0x26, 0x00 },
		{ 0x27, 0x00 }, { 0x28, 0x00 }, { 0x29, 0x00 },
	};
	int ret;

	s->initialized = false;
	s->tuned = false;
	hi3130_stats_reset(&s->fe);
	/* SDK tuner_chip_reset: active-low 100 ms, then settle for 50 ms. */
	gpiod_set_value_cansleep(s->reset, 1);
	msleep(100);
	gpiod_set_value_cansleep(s->reset, 0);
	msleep(50);
	ret = hi3130_identify(s);
	if (ret)
		return ret;
	ret = hi3130_update(s, HI3130_BANK, 3, 2);
	if (ret)
		return ret;
	ret = hi3130_write_regs(s, hi3130_hard, ARRAY_SIZE(hi3130_hard));
	if (ret)
		return ret;
	ret = hi3130_write_regs(s, ts_pins, ARRAY_SIZE(ts_pins));
	if (ret)
		return ret;
	/* Serial output uses the SDK's 74.25 MHz TS clock selection. */
	ret = hi3130_update(s, 0x00, BIT(1), 0);
	if (ret)
		return ret;
	ret = hi3130_update(s, HI3130_BANK, 3, 0);
	if (!ret)
		s->initialized = true;
	return ret;
}

static int hi3130_init(struct dvb_frontend *fe)
{
	struct hi3130_state *s = fe->demodulator_priv;
	int ret;

	mutex_lock(&s->lock);
	ret = s->removed ? -ENODEV : hi3130_hw_init(s);
	mutex_unlock(&s->lock);
	return ret;
}

static int hi3130_sleep(struct dvb_frontend *fe)
{
	struct hi3130_state *s = fe->demodulator_priv;
	int ret;

	mutex_lock(&s->lock);
	ret = s->removed ? -ENODEV : hi3130_write(s, HI3130_MCTRL1, 0xff);
	s->initialized = false;
	s->tuned = false;
	hi3130_stats_reset(fe);
	mutex_unlock(&s->lock);
	return ret;
}

static int hi3130_qam_index(enum fe_modulation modulation)
{
	switch (modulation) {
	case QAM_16:
		return 0;
	case QAM_32:
		return 1;
	case QAM_64:
		return 2;
	case QAM_128:
		return 3;
	case QAM_256:
		return 4;
	default:
		return -EINVAL;
	}
}

static int hi3130_program(struct hi3130_state *s,
			  const struct dtv_frontend_properties *c,
			  unsigned int qam)
{
	static const struct hi3130_reg low_rate[] = {
		{ 0xf5, 0x22 }, { 0xf6, 0x22 }, { 0xf7, 0x02 }, { 0xf8, 0x30 },
		{ 0xd7, 0x22 }, { 0xd8, 0x22 }, { 0xd1, 0x22 },
	};
	static const struct hi3130_reg reset[] = {
		{ 0x52, 0x20 },
		{ HI3130_MCTRL1, 0xff },
		{ HI3130_MCTRL1, 0x00 },
		{ 0x52, 0xef },
	};
	static const u8 phase[] = { 0x08, 0x08, 0x08, 0x05, 0x03 };
	static const u8 phase_low[] = { 0x08, 0x08, 0x02, 0x01, 0x00 };
	static const u8 custom[] = { 0x20, 0x20, 0x59, 0x65, 0x6a };
	u32 ratio = div_u64((u64)HI3130_XTAL_HZ * 8192, c->symbol_rate);
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(hi3130_qam); i++) {
		ret = hi3130_write(s, hi3130_qam[i].addr,
				   hi3130_qam[i].val[qam]);
		if (ret)
			return ret;
	}
	/*
	 * FS_CTRL1/2 is 0x1550 for a 5 MHz IF and 0x162b for a 5.5 MHz IF.
	 * Select by the explicitly configured IF, not Linux adapter ordering.
	 */
	if (s->tuner.if_hz == 5500000) {
		ret = hi3130_write(s, 0xb1, 0x16);
		if (ret)
			return ret;
		ret = hi3130_write(s, 0xb2, 0x2b);
		if (ret)
			return ret;
	}
	ret = hi3130_write(s, 0x43, custom[qam]);
	if (ret)
		return ret;
	/* Internal ADC, CMOS AGC and two's-complement ADC data, as SDK. */
	ret = hi3130_update(s, 0x47, 0x84, 0);
	if (ret)
		return ret;
	ret = hi3130_update(s, 0x44, 0x88, 0x08);
	if (ret)
		return ret;
	/* Serial TS: DATA/CLOCK, receiver discovers packet sync in-band. */
	ret = hi3130_update(s, 0x99, 0x50, 0x10);
	if (ret)
		return ret;
	ret = hi3130_update(s, 0x98, BIT(3), BIT(3));
	if (ret)
		return ret;
	ret = hi3130_update(s, 0xa6, BIT(3),
			    c->inversion == INVERSION_ON ? BIT(3) : 0);
	if (ret)
		return ret;
	ret = hi3130_update(s, 0x14, 7, ratio >> 16);
	if (ret)
		return ret;
	ret = hi3130_write(s, 0x15, ratio >> 8);
	if (ret)
		return ret;
	ret = hi3130_write(s, 0x16, ratio);
	if (ret)
		return ret;
	if (qam >= 2) {
		if (c->symbol_rate < 4800000) {
			ret = hi3130_update(s, 0xc7, BIT(4), BIT(4));
			if (ret)
				return ret;
			ret = hi3130_write_regs(s, low_rate,
						ARRAY_SIZE(low_rate));
			if (ret)
				return ret;
		}
		ret = hi3130_write(s, 0xe1,
				   c->symbol_rate < 4800000 ? phase_low[qam] :
							      phase[qam]);
		if (ret)
			return ret;
	}
	return hi3130_write_regs(s, reset, ARRAY_SIZE(reset));
}

static int hi3130_set_frontend(struct dvb_frontend *fe)
{
	struct hi3130_state *s = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int qam = hi3130_qam_index(c->modulation);
	int ret;

	if (qam < 0 || c->delivery_system != SYS_DVBC_ANNEX_A ||
	    c->symbol_rate < 870000 || c->symbol_rate > 7200000 ||
	    c->frequency < 45000000 || c->frequency > 1002000000 ||
	    (c->inversion != INVERSION_OFF && c->inversion != INVERSION_ON))
		return -EINVAL;
	mutex_lock(&s->lock);
	s->tuned = false;
	hi3130_stats_reset(fe);
	if (s->removed) {
		ret = -ENODEV;
		goto out;
	}
	if (!s->initialized) {
		ret = hi3130_hw_init(s);
		if (ret)
			goto out;
	}
	ret = fe->ops.tuner_ops.set_params(fe);
	if (ret)
		goto out;
	ret = hi3130_program(s, c, qam);
	if (ret) {
		s->initialized = false;
		goto out;
	}
	s->modulation = qam;
	s->tuned = true;
out:
	mutex_unlock(&s->lock);
	return ret;
}

static int hi3130_read_cnr(struct hi3130_state *s)
{
	/* SDK unf_tuner.c TUNER_GetSNR_C: 10log10(K * 2^20 / MSE). */
	static const u32 power[] = {
		(5U << 20) / 18,   (1U << 20) / 5,	(42U << 20) / 196,
		(82U << 20) / 484, (170U << 20) / 1024,
	};
	u32 mse = 0;
	u8 hi, lo;
	unsigned int i;
	s64 cnr;
	int ret;

	for (i = 0; i < 64; i++) {
		ret = hi3130_write(s, 0x25, 1);
		if (ret)
			return ret;
		ret = hi3130_read(s, 0x0a, &hi);
		if (ret)
			return ret;
		ret = hi3130_read(s, 0x0b, &lo);
		if (ret)
			return ret;
		mse += ((u32)hi << 8) | lo;
	}
	mse >>= 6;
	/* Zero MSE has no finite CNR; do not invent a 50 dB ceiling. */
	if (!mse)
		return 0;
	cnr = (s64)intlog10(power[s->modulation]) - intlog10(mse);
	cnr = div_s64(cnr * 10000, 1 << 24);
	s->fe.dtv_property_cache.cnr.stat[0].scale = FE_SCALE_DECIBEL;
	s->fe.dtv_property_cache.cnr.stat[0].svalue = cnr;
	return 0;
}

static int hi3130_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct hi3130_state *s = fe->demodulator_priv;
	u8 val;
	unsigned int i;
	int ret = 0;

	*status = 0;
	mutex_lock(&s->lock);
	hi3130_stats_reset(fe);
	if (s->removed) {
		ret = -ENODEV;
		goto out;
	}
	if (!s->initialized || !s->tuned)
		goto out;
	for (i = 0; i < 4; i++) {
		ret = hi3130_read(s, HI3130_STATUS, &val);
		if (ret)
			goto out;
		/* Only these flags have an evidenced mapping; no guessed stages. */
		if (val & BIT(7))
			*status = FE_HAS_SIGNAL;
		if ((val & 0xf8) == 0xf8) {
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				  FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
			ret = hi3130_read_cnr(s);
			break;
		}
	}
out:
	if (ret) {
		*status = 0;
		hi3130_stats_reset(fe);
	}
	mutex_unlock(&s->lock);
	return ret;
}

static void hi3130_free(struct kref *ref)
{
	struct hi3130_state *s = container_of(ref, struct hi3130_state, ref);

	i2c_put_adapter(s->tuner.peer_adapter);
	i2c_put_adapter(s->tuner_adapter);
	put_device(&s->client->dev);
	kfree(s);
}

static void hi3130_release(struct dvb_frontend *fe)
{
	struct hi3130_state *s = fe->demodulator_priv;

	mutex_lock(&s->lock);
	/* DVB detach can precede the final close of an open frontend file. */
	if (s->tuner_release) {
		void (*release)(struct dvb_frontend *) = s->tuner_release;

		s->tuner_release = NULL;
		release(fe);
		dvb_detach(release);
	}
	fe->frontend_priv = NULL;
	s->claimed = false;
	s->tuned = false;
	mutex_unlock(&s->lock);
	kref_put(&s->ref, hi3130_free);
	/* With MEDIA_ATTACH, the DVB core drops this reference after release. */
	if (!IS_ENABLED(CONFIG_MEDIA_ATTACH))
		module_put(THIS_MODULE);
}

static const struct dvb_frontend_ops hi3130_ops = {
	.delsys = { SYS_DVBC_ANNEX_A },
	.info = {
		.name = "HiSilicon Hi3130 v200 DVB-C",
		.frequency_min_hz = 45000000,
		.frequency_max_hz = 1002000000,
		.frequency_stepsize_hz = 62500,
		.symbol_rate_min = 870000,
		.symbol_rate_max = 7200000,
		.caps = FE_CAN_QAM_16 | FE_CAN_QAM_32 | FE_CAN_QAM_64 |
			FE_CAN_QAM_128 | FE_CAN_QAM_256 | FE_CAN_FEC_AUTO,
	},
	.release = hi3130_release,
	.init = hi3130_init,
	.sleep = hi3130_sleep,
	.set_frontend = hi3130_set_frontend,
	.read_status = hi3130_read_status,
};

struct dvb_frontend *hi3130_get_frontend(struct i2c_client *client,
					 unsigned int id)
{
	struct hi3130_state *s;
	struct dvb_frontend *fe;

	if (!client || id)
		return ERR_PTR(-EINVAL);
	/*
	 * A bridge can call this from probe with its own device lock held.
	 * Never wait for the supplier lock: supplier unbind may already be
	 * waiting for that bridge through a managed device link.
	 */
	if (!device_trylock(&client->dev))
		return ERR_PTR(-EPROBE_DEFER);
	fe = ERR_PTR(-EPROBE_DEFER);
	if (client->dev.driver != &hi3130_driver.driver ||
	    !device_is_bound(&client->dev) ||
	    READ_ONCE(client->dev.links.status) != DL_DEV_DRIVER_BOUND)
		goto unlock_device;
	/* The supplier lock protects clientdata and its initial state ref. */
	s = i2c_get_clientdata(client);
	if (!s)
		goto unlock_device;
	mutex_lock(&s->lock);
	if (s->removed) {
		fe = ERR_PTR(-ENODEV);
	} else if (s->claimed) {
		fe = ERR_PTR(-EBUSY);
	} else if (!try_module_get(THIS_MODULE)) {
		fe = ERR_PTR(-ENODEV);
	} else {
		/* Restore callbacks after the previous frontend's final release. */
		s->fe.ops = hi3130_ops;
		fe = dvb_attach(r858_attach, &s->fe, s->tuner_adapter,
				&s->tuner);
		if (!fe) {
			module_put(THIS_MODULE);
			fe = ERR_PTR(-ENODEV);
		} else {
			s->tuner_release = fe->ops.tuner_ops.release;
			fe->ops.tuner_ops.release = NULL;
			/* Unregister leaves DEVICE_REMOVED until the next claim. */
			WRITE_ONCE(fe->exit, DVB_FE_NO_EXIT);
			s->claimed = true;
			kref_get(&s->ref);
		}
	}
	mutex_unlock(&s->lock);
unlock_device:
	device_unlock(&client->dev);
	return fe;
}
EXPORT_SYMBOL_GPL(hi3130_get_frontend);

static int hi3130_parse_tuner(struct hi3130_state *s)
{
	struct device_node *np, *peer = NULL, *bus = NULL;
	struct r858_config *cfg = &s->tuner;
	u32 val;
	int ret = -EINVAL;

	np = of_get_child_by_name(s->client->dev.of_node, "tuner");
	if (!np)
		return -EINVAL;
	bus = of_parse_phandle(np, "i2c-parent", 0);
	if (!bus)
		goto out;
	s->tuner_adapter = of_get_i2c_adapter_by_node(bus);
	of_node_put(bus);
	bus = NULL;
	if (!s->tuner_adapter) {
		ret = -EPROBE_DEFER;
		goto out;
	}
	if (of_property_read_u32(np, "reg", &val) || val > 0x7f)
		goto out;
	cfg->i2c_address = val;
	if (of_property_read_u32(np, "rafael,tuner-index", &val) || val > 1)
		goto out;
	cfg->tuner_index = val;
	if (of_property_read_u32(np, "clock-frequency", &cfg->xtal_hz) ||
	    cfg->xtal_hz != 24000000)
		goto out;
	if (of_property_read_u32(np, "rafael,if-frequency-hz", &cfg->if_hz) ||
	    (cfg->if_hz != 5000000 && cfg->if_hz != 5500000))
		goto out;
	if (of_property_read_u32(np, "rafael,xtal-role", &val) ||
	    val > R858_XTAL_SLAVE_NO_OUTPUT)
		goto out;
	cfg->xtal_role = val;
	if (of_property_read_u32(np, "rafael,xtal-cap", &val) || val > 31)
		goto out;
	cfg->xtal_cap = val;
	cfg->loop_through = of_property_read_bool(np, "rafael,loop-through");
	cfg->clock_out = of_property_read_bool(np, "rafael,clock-out");
	peer = of_parse_phandle(np, "rafael,peer", 0);
	if (!peer || peer == np || of_property_read_u32(peer, "reg", &val) ||
	    val > 0x7f)
		goto out;
	cfg->peer_address = val;
	if (of_property_read_u32(peer, "rafael,if-frequency-hz",
				 &cfg->peer_if_hz) ||
	    (cfg->peer_if_hz != 5000000 && cfg->peer_if_hz != 5500000))
		goto out;
	if (of_property_read_u32(peer, "rafael,tuner-index", &val) ||
	    val != (cfg->tuner_index ^ 1))
		goto out;
	bus = of_parse_phandle(peer, "i2c-parent", 0);
	if (!bus)
		goto out;
	cfg->peer_adapter = of_get_i2c_adapter_by_node(bus);
	ret = cfg->peer_adapter ? 0 : -EPROBE_DEFER;
out:
	of_node_put(bus);
	of_node_put(peer);
	of_node_put(np);
	return ret;
}

static int hi3130_link_tuner_controller(struct hi3130_state *s,
					struct i2c_adapter *adapter)
{
	struct device *controller = adapter->dev.parent;
	int ret = 0;

	/* This adapter's own I2C children are removed by i2c_del_adapter(). */
	if (adapter == s->client->adapter)
		return 0;
	if (!controller)
		return -ENODEV;
	/*
	 * The adapter itself has no driver. Its owning controller must unbind
	 * this consumer before deleting an adapter whose reference is retained
	 * by this frontend. Never link the two demodulators to each other.
	 */
	if (!device_trylock(controller))
		return -EPROBE_DEFER;
	if (!device_is_bound(controller) ||
	    READ_ONCE(controller->links.status) != DL_DEV_DRIVER_BOUND)
		ret = -EPROBE_DEFER;
	else if (!device_link_add(&s->client->dev, controller,
				  DL_FLAG_AUTOREMOVE_CONSUMER))
		ret = -EINVAL;
	device_unlock(controller);
	/* The driver core owns this managed link, also on probe failure. */
	return ret;
}

static int hi3130_probe(struct i2c_client *client)
{
	struct hi3130_state *s;
	const char *ts;
	u32 xtal;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;
	if (of_property_read_u32(client->dev.of_node, "clock-frequency",
				 &xtal) ||
	    xtal != HI3130_XTAL_HZ ||
	    of_property_read_string(client->dev.of_node, "hisilicon,ts-mode",
				    &ts) ||
	    strcmp(ts, "serial-nosync"))
		return dev_err_probe(
			&client->dev, -EINVAL,
			"28.8 MHz ADC clock and serial-nosync required\n");
	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->client = client;
	get_device(&client->dev);
	kref_init(&s->ref);
	mutex_init(&s->lock);
	s->fe.ops = hi3130_ops;
	s->fe.demodulator_priv = s;
	ret = hi3130_parse_tuner(s);
	if (ret)
		goto fail;
	ret = hi3130_link_tuner_controller(s, s->tuner_adapter);
	if (ret)
		goto fail;
	if (s->tuner.peer_adapter != s->tuner_adapter) {
		ret = hi3130_link_tuner_controller(s, s->tuner.peer_adapter);
		if (ret)
			goto fail;
	}
	s->reset = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(s->reset)) {
		ret = PTR_ERR(s->reset);
		goto fail;
	}
	ret = hi3130_hw_init(s);
	if (ret)
		goto fail;
	i2c_set_clientdata(client, s);
	dev_info(&client->dev,
		 "Hi3130 external v200 DVB-C frontend available\n");
	return 0;
fail:
	if (!IS_ERR_OR_NULL(s->reset))
		gpiod_set_value_cansleep(s->reset, 1);
	kref_put(&s->ref, hi3130_free);
	return dev_err_probe(&client->dev, ret,
			     "Hi3130 initialization failed\n");
}

static void hi3130_remove(struct i2c_client *client)
{
	struct hi3130_state *s = i2c_get_clientdata(client);

	mutex_lock(&s->lock);
	s->removed = true;
	s->initialized = false;
	s->tuned = false;
	hi3130_stats_reset(&s->fe);
	gpiod_set_value_cansleep(s->reset, 1);
	mutex_unlock(&s->lock);
	i2c_set_clientdata(client, NULL);
	kref_put(&s->ref, hi3130_free);
}

static const struct of_device_id hi3130_of_match[] = {
	{ .compatible = "hisilicon,hi3130" },
	{}
};
MODULE_DEVICE_TABLE(of, hi3130_of_match);

static const struct i2c_device_id hi3130_id[] = { { "hi3130", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, hi3130_id);

static struct i2c_driver hi3130_driver = {
	.driver = {
		.name = "hi3130",
		.of_match_table = hi3130_of_match,
	},
	.probe = hi3130_probe,
	.remove = hi3130_remove,
	.id_table = hi3130_id,
};
module_i2c_driver(hi3130_driver);

MODULE_DESCRIPTION("HiSilicon Hi3130 external v200 DVB-C Annex A demodulator");
MODULE_AUTHOR("DVB-IP contributors");
MODULE_LICENSE("GPL");
