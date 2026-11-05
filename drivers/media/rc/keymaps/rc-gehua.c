// SPDX-License-Identifier: GPL-2.0
/*
 * Keymap for the Gehua RC6 mode 6A, 32-bit remote control.
 *
 * Derived from the factory /etc/key.xml. The factory table stores
 * full 32-bit RC6 scancodes. The 0x80 high byte is part of the
 * scancode, not a protocol prefix to strip.
 */

#include <media/rc-map.h>
#include <linux/module.h>

static struct rc_map_table gehua[] = {
	{ 0x80102600, KEY_0 },
	{ 0x80102601, KEY_1 },
	{ 0x80102602, KEY_2 },
	{ 0x80102603, KEY_3 },
	{ 0x80102604, KEY_4 },
	{ 0x80102605, KEY_5 },
	{ 0x80102606, KEY_6 },
	{ 0x80102607, KEY_7 },
	{ 0x80102608, KEY_8 },
	{ 0x80102609, KEY_9 },

	{ 0x8010260a, KEY_BACK },
	{ 0x80102683, KEY_BACK },
	{ 0x8010260c, KEY_POWER },
	{ 0x8010260d, KEY_MUTE },
	{ 0x8010260f, KEY_INFO },		/* second info code */
	{ 0x80102610, KEY_VOLUMEUP },
	{ 0x80102611, KEY_VOLUMEDOWN },
	{ 0x80102620, KEY_CHANNELUP },
	{ 0x80102621, KEY_CHANNELDOWN },
	{ 0x80102631, KEY_PLAYPAUSE },
	{ 0x80102654, KEY_MENU },
	{ 0x80102658, KEY_UP },
	{ 0x80102659, KEY_DOWN },
	{ 0x8010265a, KEY_LEFT },
	{ 0x8010265b, KEY_RIGHT },
	{ 0x8010265c, KEY_ENTER },
	{ 0x8010266d, KEY_INFO },
	{ 0x8010266e, KEY_GREEN },
	{ 0x8010266f, KEY_YELLOW },
	{ 0x80102670, KEY_BLUE },
	{ 0x80102682, KEY_HOME },
	{ 0x8010269f, KEY_NUMERIC_STAR },
	{ 0x801026a0, KEY_NUMERIC_POUND },
	{ 0x801026ad, KEY_F9 },
	{ 0x801026c3, KEY_F10 },
	{ 0x801026cc, KEY_EPG },		/* factory KEY_FN_1 */
	{ 0x801026f5, KEY_LIST },		/* factory KEY_F8 */
};

static struct rc_map_list gehua_map = {
	.map = {
		.scan		= gehua,
		.size		= ARRAY_SIZE(gehua),
		.rc_proto	= RC_PROTO_RC6_6A_32,
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
MODULE_DESCRIPTION("Gehua RC6-6A-32 remote control keymap");
