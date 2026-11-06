/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _MEDIA_HI3130_H_
#define _MEDIA_HI3130_H_

#include <linux/err.h>
#include <linux/i2c.h>

struct dvb_frontend;

/*
 * Acquire frontend 0 of a bound Hi3130 I2C device, or an ERR_PTR().
 * The bridge must call dvb_unregister_frontend() for a registered frontend,
 * then dvb_frontend_detach() once for each successful acquisition, including
 * registration failures. Open frontend files keep the receiver alive after
 * removal; it cannot be acquired again until its last user releases it.
 * The bridge must prevent driver unbind while the frontend is registered.
 */
#if IS_REACHABLE(CONFIG_DVB_HI3130)
struct dvb_frontend *hi3130_get_frontend(struct i2c_client *client,
					 unsigned int id);
#else
static inline struct dvb_frontend *
hi3130_get_frontend(struct i2c_client *client, unsigned int id)
{
	return ERR_PTR(-ENODEV);
}
#endif

#endif
