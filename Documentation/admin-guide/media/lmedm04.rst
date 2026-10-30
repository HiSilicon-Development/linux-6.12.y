.. SPDX-License-Identifier: GPL-2.0

Firmware files for lmedm04 cards
================================

To extract firmware for the DM04/QQBOX you need to copy the
following file(s) to this directory.

For DM04+/QQBOX LME2510C (Sharp 7395 Tuner)
-------------------------------------------

The Sharp 7395 driver can be found in windows/system32/drivers

US2A0D.sys (dated 17 Mar 2009)


and run:

.. code-block:: none

	scripts/get_dvb_firmware lme2510c_s7395

will produce dvb-usb-lme2510c-s7395.fw

An alternative but older firmware can be found on the driver
disk DVB-S_EN_3.5A in BDADriver/driver

LMEBDA_DVBS7395C.sys (dated 18 Jan 2008)

and run:

.. code-block:: none

	./get_dvb_firmware lme2510c_s7395_old

will produce dvb-usb-lme2510c-s7395.fw

The LG firmware can be found on the driver
disk DM04+_5.1A[LG] in BDADriver/driver

For DM04 LME2510 (LG Tuner)
---------------------------

LMEBDA_DVBS.sys (dated 13 Nov 2007)

and run:


.. code-block:: none

	./get_dvb_firmware lme2510_lg

will produce dvb-usb-lme2510-lg.fw


Other LG firmware can be extracted manually from US280D.sys
only found in windows/system32/drivers

dd if=US280D.sys ibs=1 skip=42360 count=3924 of=dvb-usb-lme2510-lg.fw

For DM04 LME2510C (LG Tuner)
----------------------------

.. code-block:: none

	dd if=US280D.sys ibs=1 skip=35200 count=3850 of=dvb-usb-lme2510c-lg.fw


The Sharp 0194 tuner driver can be found in windows/system32/drivers

US290D.sys (dated 09 Apr 2009)

For LME2510
-----------

.. code-block:: none

	dd if=US290D.sys ibs=1 skip=36856 count=3976 of=dvb-usb-lme2510-s0194.fw


For LME2510C
------------


.. code-block:: none

	dd if=US290D.sys ibs=1 skip=33152 count=3697 of=dvb-usb-lme2510c-s0194.fw


The m88rs2000 tuner driver can be found in windows/system32/drivers

US2B0D.sys (dated 29 Jun 2010)


.. code-block:: none

	dd if=US2B0D.sys ibs=1 skip=34432 count=3871 of=dvb-usb-lme2510c-rs2000.fw

We need to modify id of rs2000 firmware or it will warm boot id 3344:1120.


.. code-block:: none


	echo -ne \\xF0\\x22 | dd conv=notrunc bs=1 count=2 seek=266 of=dvb-usb-lme2510c-rs2000.fw

Copy the firmware file(s) to /lib/firmware

CH1 DTMB receivers
------------------

The separate ``CONFIG_DVB_USB_LME2510_DTMB`` driver supports CH1 receivers
with an LGS8GL5 demodulator and MAX2165 tuner. These devices can share USB
ID 3344:1120 with the DVB-S models above; their firmware is not interchangeable.

The DTMB driver requests ``dvb-usb-lme2510c-dtmb-win-rom.bin`` and
``dvb-usb-lme2510c-dtmb-win-fw.bin``. The PID 22d0 variant instead requests
``dvb-usb-lme2510c-dtmb-win-fw-pid0xd0.bin`` for the second stage.

Each MAX2165 frequency change writes the divider block and register 0x0a,
then reads register 0x04 and writes it back with its upper nibble set. This
matches the vendor tuning sequence and preserves the calibrated lower nibble.
A failure in either readback or writeback aborts tuning.

The LGS8GL5 parallel TS interface uses an inverted, gated clock (register
0xc2 = 0x2a). Using the normal clock polarity can acquire RF lock while the
LME2510C bulk endpoint delivers no transport data.

The driver completes the acquisition search during each requested tune and
uses hardware-algorithm polling afterward. While TS is active, status comes
from endpoint 0x8a; command-register reads would leave streaming mode. An
initially unavailable asynchronous status must not trigger software zigzag
retuning. Stopping from streaming mode first receives the firmware's 0x77
mode-exit response, then repeats the clear command in command mode.

Only complete register replies are used, while bridge commands accept their
documented leading acknowledgement even when the bridge returns a longer
packet. Status packets are parsed in complete eight-byte units and expire
after two seconds. Variants without endpoint 0x8a can receive TS, but their
lock and quality are unknown during streaming because command reads would
interrupt capture. No status-settling packet discard is applied.

The runtime IDs are ``3344:1120``, ``3344:1122``, ``3344:22d0``,
``3344:22d8``, ``3344:e388`` and ``1b80:e388``; ``3344:1111`` is the cold
bootloader ID. The shared-ID DVB-S driver must not bind the DTMB board.
Firmware redistribution rights and current-source hardware coverage for
each runtime variant still require independent confirmation.
