// SPDX-License-Identifier: GPL-2.0-only
/*
 * EM28xx USB registration.
 *
 * VID:PID only selects the bridge and its isolated board profile; it never
 * permits one board's initialization sequence to fall through to another.
 * Derived from DTMB/dvb-cn-usb/src/em28xx/ (GPL, MODULE_AUTHOR wwek).
 */
#include <linux/module.h>
#include <linux/usb.h>

#include "em28xx-dtmb.h"

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static int em28xx_probe(struct dvb_usb_device *d)
{
	struct em28xx_state *st = d_to_priv(d);

	st->d = d;
	mutex_init(&st->lock);
	return 0;
}

static void em28xx_exit(struct dvb_usb_device *d)
{
	struct em28xx_state *st = d_to_priv(d);

	/* Drain in-flight frontend I/O before the core stops its worker and URBs. */
	mutex_lock(&st->lock);
	st->disconnected = true;
	st->initialized = false;
	st->tuned = false;
	st->streaming = false;
	mutex_unlock(&st->lock);
}

static const struct dvb_usb_device_properties dt5236_props = {
	.driver_name = EM28XX_DRIVER_NAME,
	.owner = THIS_MODULE,
	.adapter_nr = adapter_nr,
	.size_of_priv = sizeof(struct em28xx_state),
	.probe = em28xx_probe,
	.exit = em28xx_exit,
	.frontend_attach = dt5236_frontend_attach,
	.streaming_ctrl = dt5236_streaming_ctrl,
	.get_stream_config = dt5236_get_stream_config,
	.num_adapters = 1,
	.adapter = {
		{
			/* DT5236 实机验证的 EP84/alt1 传输参数；不要按总缓冲量改写。 */
			.stream = DVB_USB_STREAM_ISOC(0x84, 8, 48, 940, 1),
		},
	},
};

#define EM28XX_USB_DEVICE(pid, name) \
	{ DVB_USB_DEVICE(EM28XX_VID, pid, &dt5236_props, name, NULL) }

static const struct usb_device_id em28xx_id_table[] = {
	EM28XX_USB_DEVICE(EM28XX_PID, "EM28xx USB frontend profile"),
	/* 2889 is the audio companion, not a validated EP84 DTMB interface. */
	{ }
};
MODULE_DEVICE_TABLE(usb, em28xx_id_table);

static struct usb_driver em28xx_usb_driver = {
	.name = EM28XX_DRIVER_NAME,
	.id_table = em28xx_id_table,
	.probe = dvb_usbv2_probe,
	.disconnect = dvb_usbv2_disconnect,
	.suspend = dvb_usbv2_suspend,
	.resume = dvb_usbv2_resume,
	.reset_resume = dvb_usbv2_reset_resume,
};

module_usb_driver(em28xx_usb_driver);

MODULE_AUTHOR("wwek");
MODULE_DESCRIPTION("EM28xx USB bridge with Aigo DT5236 DTMB frontend");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("dvb-fe-xc5000-1.6.114.fw");
