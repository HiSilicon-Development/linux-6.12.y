// SPDX-License-Identifier: GPL-2.0-only
#ifndef _MEDIA_MXL214_H_
#define _MEDIA_MXL214_H_

#include <linux/err.h>
#include <linux/i2c.h>

struct dvb_frontend;

/*
 * Acquire channel 0..3 of a bound MxL214 I2C device, or an ERR_PTR().
 * The bridge must call dvb_unregister_frontend() for a registered frontend,
 * then dvb_frontend_detach() once for each successful acquisition, including
 * registration failures. Open frontend files keep the channel alive after
 * removal; a channel cannot be acquired again until its last user releases it.
 * The bridge must prevent driver unbind while any frontend is registered.
 */
#if IS_REACHABLE(CONFIG_DVB_MXL214)
struct dvb_frontend *mxl214_get_frontend(struct i2c_client *client,
					 unsigned int id);
#else
static inline struct dvb_frontend *
mxl214_get_frontend(struct i2c_client *client, unsigned int id)
{
	return ERR_PTR(-ENODEV);
}
#endif

#endif
