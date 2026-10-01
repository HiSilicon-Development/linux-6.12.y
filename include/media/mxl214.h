// SPDX-License-Identifier: GPL-2.0-only
#ifndef _MEDIA_MXL214_H_
#define _MEDIA_MXL214_H_

#include <linux/err.h>
#include <linux/i2c.h>

struct dvb_frontend;

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
