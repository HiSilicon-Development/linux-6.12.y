/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef AVL6381_PRIV_H_
#define AVL6381_PRIV_H_

#include <linux/types.h>
#include <linux/dvb/frontend.h>

struct avl6381_config {
	/* Demodulator I2C address (7-bit). */
	u8 i2c_address;
	struct dvb_frontend **fe;

	/*
	 * optional bridge hook: select the RF input path (terrestrial vs
	 * cable) via bridge GPIOs before tuning.
	 */
	int (*tuner_select_input)(struct dvb_frontend *fe,
				  enum fe_delivery_system delivery_system);
};

struct i2c_adapter;

struct dvb_frontend *avl6381_attach(const struct avl6381_config *config,
				    struct i2c_adapter *i2c);

#endif
