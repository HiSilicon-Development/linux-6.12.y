// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * EC6110T reference system/etc/key.xml: dianxin20-key and hisi-key.
 * NEC complement bytes are removed to form rc-core address/command scancodes.
 * The vendor-only KEY_BTRCPAIRING has no equivalent Linux pairing action.
 */
#include <linux/module.h>
#include <media/rc-map.h>

static struct rc_map_table ec6110t_keymap[] = {
	/* dianxin20-key */
	{ 0x2280, KEY_VOLUMEUP },
	{ 0x2281, KEY_VOLUMEDOWN },
	{ 0x2282, KEY_MENU },
	{ 0x2283, KEY_YELLOW },
	{ 0x2284, KEY_FORWARD },
	{ 0x2285, KEY_CHANNELUP },
	{ 0x2286, KEY_CHANNELDOWN },
	{ 0x2287, KEY_0 },
	{ 0x2288, KEY_HOME },
	{ 0x2289, KEY_REWIND },
	{ 0x228a, KEY_7 },
	{ 0x228b, KEY_8 },
	{ 0x228c, KEY_PAGEDOWN },
	{ 0x228d, KEY_SETUP },
	{ 0x228e, KEY_4 },
	{ 0x228f, KEY_5 },
	{ 0x2291, KEY_GREEN },
	{ 0x2292, KEY_1 },
	{ 0x2293, KEY_2 },
	{ 0x2295, KEY_BACK },
	{ 0x2296, KEY_STOP },
	{ 0x2298, KEY_AUDIO },
	{ 0x2299, KEY_LEFT },
	{ 0x229c, KEY_MUTE },
	{ 0x22c1, KEY_RIGHT },
	{ 0x22c3, KEY_BLUE },
	{ 0x22c4, KEY_9 },
	{ 0x22c5, KEY_HELP },
	{ 0x22c8, KEY_6 },
	{ 0x22ca, KEY_UP },
	{ 0x22cc, KEY_3 },
	{ 0x22cd, KEY_RED },
	{ 0x22ce, KEY_ENTER },
	{ 0x22d2, KEY_DOWN },
	{ 0x22d6, KEY_SEARCH },
	{ 0x22d9, KEY_PLAYPAUSE },
	{ 0x22da, KEY_NUMERIC_POUND },
	{ 0x22dc, KEY_POWER },
	{ 0x22dd, KEY_PAGEUP },
	/* hisi-key */
	{ 0x0057, KEY_POWER },
	{ 0x0065, KEY_LAST },
	{ 0x0080, KEY_VOLUMEUP },
	{ 0x0081, KEY_VOLUMEDOWN },
	{ 0x0082, KEY_FN_B },
	{ 0x0084, KEY_F1 },
	{ 0x0085, KEY_CHANNELUP },
	{ 0x0086, KEY_CHANNELDOWN },
	{ 0x0087, KEY_0 },
	{ 0x0088, KEY_BACKSPACE },
	{ 0x0089, KEY_F2 },
	{ 0x008a, KEY_7 },
	{ 0x008b, KEY_8 },
	{ 0x008c, KEY_SUBTITLE },
	{ 0x008e, KEY_4 },
	{ 0x008f, KEY_5 },
	{ 0x0090, KEY_BACK },
	{ 0x0092, KEY_1 },
	{ 0x0093, KEY_2 },
	{ 0x0094, KEY_FAVORITES },
	{ 0x0095, KEY_SETUP },
	{ 0x0096, KEY_F4 },
	{ 0x0097, KEY_SEARCH },
	{ 0x0098, KEY_PAGEDOWN },
	{ 0x0099, KEY_LEFT },
	{ 0x009a, KEY_FN_E },
	{ 0x009b, KEY_FN_D },
	{ 0x009d, KEY_MENU },
	{ 0x009f, KEY_FN_1 },
	{ 0x00c0, KEY_FN_F },
	{ 0x00c1, KEY_RIGHT },
	{ 0x00c2, KEY_FN_S },
	{ 0x00c3, KEY_PLAYPAUSE },
	{ 0x00c4, KEY_9 },
	{ 0x00c6, KEY_FN_2 },
	{ 0x00c7, KEY_INFO },
	{ 0x00c8, KEY_6 },
	{ 0x00ca, KEY_UP },
	{ 0x00cb, KEY_HOME },
	{ 0x00cc, KEY_3 },
	{ 0x00ce, KEY_ENTER },
	{ 0x00cf, KEY_PAGEUP },
	{ 0x00d0, KEY_STOP },
	{ 0x00d1, KEY_AUDIO },
	{ 0x00d2, KEY_DOWN },
	{ 0x00d6, KEY_FORWARD },
	{ 0x00d9, KEY_F3 },
	{ 0x00da, KEY_REWIND },
	{ 0x00dd, KEY_MUTE },
};

static struct rc_map_list ec6110t_map = {
	.map = {
		.scan = ec6110t_keymap,
		.size = ARRAY_SIZE(ec6110t_keymap),
		.rc_proto = RC_PROTO_NEC,
		.name = RC_MAP_EC6110T,
	}
};

static int __init init_rc_map_ec6110t(void)
{
	return rc_map_register(&ec6110t_map);
}

static void __exit exit_rc_map_ec6110t(void)
{
	rc_map_unregister(&ec6110t_map);
}

module_init(init_rc_map_ec6110t);
module_exit(exit_rc_map_ec6110t);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("EC6110T reference NEC remote keytable");
