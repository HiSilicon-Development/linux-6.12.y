.. SPDX-License-Identifier: GPL-2.0

USB drivers
===========

The USB boards are identified by an identification called USB ID.

The ``lsusb`` command allows identifying the USB IDs::

    $ lsusb
    ...
    Bus 001 Device 015: ID 046d:082d Logitech, Inc. HD Pro Webcam C920
    Bus 001 Device 074: ID 2040:b131 Hauppauge
    Bus 001 Device 075: ID 2013:024f PCTV Systems nanoStick T2 290e
    ...

Newer camera devices use a standard way to expose themselves as such,
via USB Video Class. Those cameras are automatically supported by the
``uvc-driver``.

Older cameras and TV USB devices uses USB Vendor Classes: each vendor
defines its own way to access the device. This section contains
card lists for such vendor-class devices.

While this is not as common as on PCI, sometimes the same USB ID is used
by different products. So, several media drivers allow passing a ``card=``
parameter, in order to setup a card number that would match the correct
settings for an specific product type.

The current supported USB cards (not including staging drivers) are
listed below\ [#]_.

.. [#]

   some of the drivers have sub-drivers, not shown at this table.
   In particular, gspca driver has lots of sub-drivers,
   for cameras not supported by the USB Video Class (UVC) driver,
   as shown at :doc:`gspca card list <gspca-cardlist>`.

======================  =========================================================
Driver                  Name
======================  =========================================================
airspy                  AirSpy
au0828                  Auvitek AU0828
b2c2-flexcop-usb        Technisat/B2C2 Air/Sky/Cable2PC USB
cx231xx                 Conexant cx231xx USB video capture
dvb-as102               Abilis AS102 DVB receiver
dvb-ttusb-budget        Technotrend/Hauppauge Nova - USB devices
dvb-usb-a800            AVerMedia AverTV DVB-T USB 2.0 (A800)
dvb-usb-af9005          Afatech AF9005 DVB-T USB1.1
dvb-usb-af9015          Afatech AF9015 DVB-T USB2.0
dvb-usb-af9035          Afatech AF9035 DVB-T USB2.0
dvb-usb-anysee          Anysee DVB-T/C USB2.0
dvb-usb-au6610          Alcor Micro AU6610 USB2.0
dvb-usb-avl6381         Runqida AVL6381 DTMB with IT9303 USB bridge
dvb-usb-az6007          AzureWave 6007 and clones DVB-T/C USB2.0
dvb-usb-az6027          Azurewave DVB-S/S2 USB2.0 AZ6027
dvb-usb-ce6230          Intel CE6230 DVB-T USB2.0
dvb-usb-cinergyT2       Terratec CinergyT2/qanu USB 2.0 DVB-T
dvb-usb-cxusb           Conexant USB2.0 hybrid
dvb-usb-dib0700         DiBcom DiB0700
dvb-usb-dibusb-common   DiBcom DiB3000M-B
dvb-usb-dibusb-mc       DiBcom DiB3000M-C/P
dvb-usb-digitv          Nebula Electronics uDigiTV DVB-T USB2.0
dvb-usb-dtt200u         WideView WT-200U and WT-220U (pen) DVB-T
dvb-usb-dtv5100         AME DTV-5100 USB2.0 DVB-T
dvb-usb-dvbsky          DVBSky USB
dvb-usb-dw2102          DvbWorld & TeVii DVB-S/S2 USB2.0
dvb-usb-ec168           E3C EC168 DVB-T USB2.0
dvb-usb-em28xx-dtmb     Aigo DT5236 DTMB (eb1a:2885)
dvb-usb-gl861           Genesys Logic GL861 USB2.0
dvb-usb-gp8psk          GENPIX 8PSK->USB module
dvb-usb-hd2312          HDIC HD2312 DTMB (04b4:1004)
dvb-usb-icast           SMIT iCast DVB-C/DTMB USB2.0 (29df:0001)
dvb-usb-lmedm04         LME DM04/QQBOX DVB-S USB2.0
dvb-usb-lme2510-dtmb    CH1 LME2510C DTMB with LGS8GL5/MAX2165
dvb-usb-m920x           Uli m920x DVB-T USB2.0
dvb-usb-nova-t-usb2     Hauppauge WinTV-NOVA-T usb2 DVB-T USB2.0
dvb-usb-opera           Opera1 DVB-S USB2.0 receiver
dvb-usb-pctv452e        Pinnacle PCTV HDTV Pro USB device/TT Connect S2-3600
dvb-usb-rtl28xxu        Realtek RTL28xxU DVB USB
dvb-usb-technisat-usb2  Technisat DVB-S/S2 USB2.0
dvb-usb-ttusb2          Pinnacle 400e DVB-S USB2.0
dvb-usb-umt-010         HanfTek UMT-010 DVB-T USB2.0
dvb_usb_v2              Support for various USB DVB devices v2
dvb-usb-vp702x          TwinhanDTV StarBox and clones DVB-S USB2.0
dvb-usb-vp7045          TwinhanDTV Alpha/MagicBoxII, DNTV tinyUSB2, Beetle USB2.0
em28xx                  Empia EM28xx USB devices
go7007                  WIS GO7007 MPEG encoder
gspca                   Drivers for several USB Cameras
hackrf                  HackRF
hdpvr                   Hauppauge HD PVR
msi2500                 Mirics MSi2500
mxl111sf-tuner          MxL111SF DTV USB2.0
pvrusb2                 Hauppauge WinTV-PVR USB2
pwc                     USB Philips Cameras
s2250                   Sensoray 2250/2251
s2255drv                USB Sensoray 2255 video capture device
smsusb                  Siano SMS1xxx based MDTV receiver
ttusb_dec               Technotrend/Hauppauge USB DEC devices
usbtv                   USBTV007 video capture
uvcvideo                USB Video Class (UVC)
zd1301                  ZyDAS ZD1301
======================  =========================================================

SMIT iCast capability and validation boundary
--------------------------------------------

The ``dvb-usb-icast`` driver handles USB ID ``29df:0001``. It reads the
receiver's ``GDHW`` capability reply before registering the frontend:
``DTMB:Y`` exposes DTMB through the legacy DVB-T API and DVB-C Annex A,
while ``DTMB:N`` exposes DVB-C Annex A only. Malformed or unknown capability replies fail attachment;
the driver does not assume a delivery system when detection fails.

DTMB uses an 8 MHz channel. DVB-C requires an explicit QAM16, QAM32,
QAM64, QAM128 or QAM256 modulation and symbol rate; QAM_AUTO is not
advertised because the reference protocol does not establish cable
modulation autodetection. GSTA level and quality percentages are exposed
on the 0..65535 relative scale in DVBv5 and legacy strength/SNR ioctls.
Retuning, lock loss and failed reads invalidate those statistics; BER and
calibrated dBm/dB values are unavailable.

The protocol adaptation is based on the public
`nxdong520/iCast <https://github.com/nxdong520/iCast>`_ and
`wwek/dvb-cn-usb <https://github.com/wwek/dvb-cn-usb>`_ sources. It is not
based on fresh decompilation of a manufacturer library: the vendor
``libicast_base.so`` and associated captures were unavailable.

No iCast receiver was available for hardware validation. USB capability
detection, DVB-C and DTMB tuning, RF lock, and transport-stream reception
remain unverified on hardware; source integration does not establish
successful reception.

Historical verification reports describe a module build against
6.12.112-dvbip-1006 production headers, 6342 simulated-reply assertions
under address/undefined-behavior sanitizers, and module loading without
an attached receiver. Those reports are not fresh verification of the
current source and establish neither physical USB timing, DMA nor RF
reception.


HDIC HD2312 firmware profiles
----------------------------

The ``dvb-usb-hd2312`` driver handles ``04b4:1004`` and identifies generic,
Letv, Aiwa itv720 and CVB T302A profiles from the firmware version reply.
The CVB profile also performs its firmware-specific activation exchange.
Control reads and writes must complete at their exact requested length;
transport errors are not valid register or status replies. A CVB lock-query
timeout is treated as an unlocked receiver, not successful reception.

DTMB retains the existing DVB-T userspace interface. Strength and quality
use relative statistics, not calibrated RF measurements. Runtime behavior
of every firmware profile, suspend/resume and unplug during I/O still need
receiver-specific validation on the current source.

Runqida AVL6381 with IT9303 bridge
--------------------------------

The ``dvb-usb-avl6381`` driver handles ``04b4:9306`` and ``048d:9306`` with
an AVL6381 demodulator and MXL603 tuner on bridge I2C bus 1. It requests
``dvb-usb-it9303-01.fw`` and accepts both classic and scatter firmware
records. Mailbox replies require a complete transfer, a valid checksum
and successful firmware status before their payload is used.

The ``048d:9306`` ID overlaps the generic ``dvb-usb-af9035`` IT9303
support. The generic driver's ``ITE 9303 Generic`` entry claims
``048d:9306``; the AVL6381 table claims both ``048d:9306`` and
``04b4:9306``. The exact competing binding at
``048d:9306`` is therefore ``dvb-usb-avl6381`` (``DVB_USB_AVL6381``) versus
``dvb-usb-af9035`` (``DVB_USB_AF9035``). The USB ID does not identify the
demodulator/tuner wiring or distinguish the Runqida board.

USB binding ownership for overlapping IDs
------------------------------------------

The following USB IDs are claimed by competing in-tree bindings:

* ``048d:9306``: ``dvb-usb-avl6381`` (Runqida AVL6381 DTMB) and
  ``dvb-usb-af9035`` (generic ``ITE 9303 Generic``).
* ``3344:1120``: ``dvb-usb-lme2510-dtmb`` (LME2510C DTMB receiver) and
  ``dvb-usb-lmedm04`` (generic ``DM04_LME2510C_DVB-S``).
* ``3344:1122``: ``dvb-usb-lme2510-dtmb`` (LME2510C DTMB receiver) and
  ``dvb-usb-lmedm04`` (generic ``DM04_LME2510_DVB-S``).
* ``eb1a:2885``: ``dvb-usb-em28xx-dtmb`` (Aigo DT5236 DTMB) and
  ``em28xx`` (generic ``MSI Digivox Trio``/EM2884 profile).

The Kconfig symbols for each pair are mutually exclusive so one owner is
selected in a kernel configuration. Kconfig is not a runtime USB ownership
mechanism: it cannot arbitrate out-of-tree modules, dynamic IDs, or a board
whose wiring differs from its USB identification. A system with multiple
physical sticks sharing one of these IDs may therefore require one selected
owner for all instances; the bindings cannot be selected per device. Board-
level identification and hardware validation remain unresolved and are
required before treating an owner choice as validated.

These IDs overlap other ITE bridge support; the USB ID alone does not
identify the demodulator/tuner wiring. Bind the driver only to the matching
Runqida board. Fresh hardware validation of firmware download, RF tuning,
TS reception and reset-resume remains required.

Aigo DT5236 signal statistics
----------------------------

The ``dvb-usb-em28xx-dtmb`` driver follows the dedicated ``lgs8gl5`` driver
by reading relative signal strength from register 0x4b bits 6:0; bit 7 is
the carrier flag. It retains the DT5236 seven-bit strength scaling. The
generic LGS8Gxx AGC bands for registers 0x3f/0x3e do not apply to this
profile: a locked DT5236 reports 0x0028 there, below those bands.

Signal quality comes from the inverse average-noise register 0x95 after
full demodulator lock. Both DVBv5 statistics and legacy strength/SNR ioctls
use the 0..65535 relative scale; these values are not calibrated dBm or dB.
Noise zero saturates at 65535 rather than wrapping to zero.

Retuning and sleeping invalidate both statistics. Loss of lock invalidates
quality; failed metric reads invalidate the affected statistic without
discarding a successful lock/status read. Applications can display relative
bars without changing their signal-statistics handling.

DT5236 acquisition searches the PN modes in the driver and uses the DVB
hardware tuning algorithm. A requested tune performs that search once;
later polls read status without resetting the demodulator or shifting the
requested frequency. This avoids software zigzag restarting acquisition
while parameter detection is still settling. Loss of lock remains visible
and does not imply successful reception.

Historical field reports on IP1006 with 6.12.112-dvbip-1006 describe recovery across
546 -> 698 (no signal) -> 546 MHz with one demux feed held open. The two
546 MHz stable intervals contained 9,031,332 and 9,026,256 bytes with valid
PAT/PMT CRCs and no sync, TEI or continuity errors. The invalid-frequency
stable interval contained no bytes; buffered tail data remained observable
immediately after retuning.

Tvheadend's DT5236 frontend also had to be enabled in its existing network.
Two 25-second HTTP subscriptions at 546 MHz produced first data in about
2.1 seconds. Excluding the first five seconds, 35,037,184 and 35,133,440
bytes had valid PAT/PMT CRCs and no sync, TEI or continuity errors.
Acquisition errors remained in the full captures and the application's
transport-error counter. The user confirmed that 522 and 546 MHz are
normally usable at this location, while 538 MHz has mosaic artifacts.
The observed 538 MHz corruption is retained as that local reception
baseline and is excluded from clean-channel acceptance of this driver
repair. This report does not establish its physical cause. These short
546 MHz runs do not establish all-channel or long-duration reception.

These field observations predate the current source audit. They are not
fresh execution evidence for this revision. The audit removes unconditional
per-poll bring-up logging and obsolete pre-4.15 frontend compatibility
without changing the DT5236 register sequence or supported board profile.

.. toctree::
	:maxdepth: 1

	au0828-cardlist
	cx231xx-cardlist
	em28xx-cardlist
	siano-cardlist

	gspca-cardlist

	dvb-usb-dib0700-cardlist
	dvb-usb-dibusb-mb-cardlist
	dvb-usb-dibusb-mc-cardlist

	dvb-usb-a800-cardlist
	dvb-usb-af9005-cardlist
	dvb-usb-az6027-cardlist
	dvb-usb-cinergyT2-cardlist
	dvb-usb-cxusb-cardlist
	dvb-usb-digitv-cardlist
	dvb-usb-dtt200u-cardlist
	dvb-usb-dtv5100-cardlist
	dvb-usb-dw2102-cardlist
	dvb-usb-gp8psk-cardlist
	dvb-usb-m920x-cardlist
	dvb-usb-nova-t-usb2-cardlist
	dvb-usb-opera1-cardlist
	dvb-usb-pctv452e-cardlist
	dvb-usb-technisat-usb2-cardlist
	dvb-usb-ttusb2-cardlist
	dvb-usb-umt-010-cardlist
	dvb-usb-vp702x-cardlist
	dvb-usb-vp7045-cardlist

	dvb-usb-af9015-cardlist
	dvb-usb-af9035-cardlist
	dvb-usb-anysee-cardlist
	dvb-usb-au6610-cardlist
	dvb-usb-az6007-cardlist
	dvb-usb-ce6230-cardlist
	dvb-usb-dvbsky-cardlist
	dvb-usb-ec168-cardlist
	dvb-usb-gl861-cardlist
	dvb-usb-lmedm04-cardlist
	dvb-usb-mxl111sf-cardlist
	dvb-usb-rtl28xxu-cardlist
	dvb-usb-zd1301-cardlist

	other-usb-cardlist
