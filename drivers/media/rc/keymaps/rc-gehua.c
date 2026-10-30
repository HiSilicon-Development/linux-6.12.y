// SPDX-License-Identifier: GPL-2.0
/*
 * Keymap for the Gehua NECX remote control.
 *
 * Derived from the factory /etc/key.xml. The factory table stores
 * scancodes with a 0x80 protocol prefix; this map drops that prefix and
 * uses RC_PROTO_NECX.
 */

#include <media/rc-map.h>
#include <linux/module.h>

static struct rc_map_table gehua[] = {
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

static struct rc_map_list gehua_map = {
	.map = {
		.scan		= gehua,
		.size		= ARRAY_SIZE(gehua),
		.rc_proto	= RC_PROTO_NECX,
		.name		= RC_MAP_GEHUA,
	}
};

static int __init init_rc_map_gehua(void)
{
	return rc_map_register(&gehua_map);
}

static void __exit exit_rc_map_gehua(void)
{
	rc_map_unregister(&gehua_map);
}

module_init(init_rc_map_gehua);
module_exit(exit_rc_map_gehua);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Gehua NECX remote control keymap");
