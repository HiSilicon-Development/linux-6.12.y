// SPDX-License-Identifier: GPL-2.0
/*
 * Keymap for the DVB-IP 1001 / 1002 / 1004 front-panel IR receiver.
 *
 * Derived from the factory /etc/key.xml of the same model, as extracted in
 * Work/1001/IP1001-IR-REVERSE-20260807/STOCK-KEYMAP-EXTRACT.txt.  The factory
 * table stores scancodes with the stock 0x80 protocol prefix; the public
 * mapping drops that prefix and is carried as RC_PROTO_NECX.
 *
 * The board device trees name this map with linux,rc-map-name =
 * "rc-dvbip-1001", so without it rc-core falls back to "rc-empty" and no key
 * event can ever be produced, however well the receiver works.
 */

#include <media/rc-map.h>
#include <linux/module.h>

static struct rc_map_table dvbip_1001[] = {
	{ 0x102600, KEY_0 },
	{ 0x102601, KEY_1 },
	{ 0x102602, KEY_2 },
	{ 0x102603, KEY_3 },
	{ 0x102604, KEY_4 },
	{ 0x102605, KEY_5 },
	{ 0x102606, KEY_6 },
	{ 0x102607, KEY_7 },
	{ 0x102608, KEY_8 },
	{ 0x102609, KEY_9 },

	{ 0x10260a, KEY_BACK },
	{ 0x102683, KEY_BACK },
	{ 0x10260c, KEY_POWER },
	{ 0x10260d, KEY_MUTE },
	{ 0x10260f, KEY_INFO },		/* second info code */
	{ 0x102610, KEY_VOLUMEUP },
	{ 0x102611, KEY_VOLUMEDOWN },
	{ 0x102620, KEY_CHANNELUP },
	{ 0x102621, KEY_CHANNELDOWN },
	{ 0x102631, KEY_PLAYPAUSE },
	{ 0x102654, KEY_MENU },
	{ 0x102658, KEY_UP },
	{ 0x102659, KEY_DOWN },
	{ 0x10265a, KEY_LEFT },
	{ 0x10265b, KEY_RIGHT },
	{ 0x10265c, KEY_ENTER },
	{ 0x10266d, KEY_INFO },
	{ 0x10266e, KEY_GREEN },
	{ 0x10266f, KEY_YELLOW },
	{ 0x102670, KEY_BLUE },
	{ 0x102682, KEY_HOME },
	{ 0x10269f, KEY_NUMERIC_STAR },
	{ 0x1026a0, KEY_NUMERIC_POUND },
	{ 0x1026ad, KEY_F9 },
	{ 0x1026c3, KEY_F10 },
	{ 0x1026cc, KEY_EPG },		/* factory KEY_FN_1 */
	{ 0x1026f5, KEY_LIST },		/* factory KEY_F8 */
};

static struct rc_map_list dvbip_1001_map = {
	.map = {
		.scan		= dvbip_1001,
		.size		= ARRAY_SIZE(dvbip_1001),
		.rc_proto	= RC_PROTO_NECX,
		.name		= "rc-dvbip-1001",
	}
};

static int __init init_rc_map_dvbip_1001(void)
{
	return rc_map_register(&dvbip_1001_map);
}

static void __exit exit_rc_map_dvbip_1001(void)
{
	rc_map_unregister(&dvbip_1001_map);
}

module_init(init_rc_map_dvbip_1001);
module_exit(exit_rc_map_dvbip_1001);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DVB-IP 1001/1002/1004 IR keymap (factory derived)");
