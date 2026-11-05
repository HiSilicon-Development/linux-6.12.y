========================
GPU Driver Documentation
========================

.. toctree::
   :maxdepth: 3

   amdgpu/index
   i915
   imagination/index
   mcde
   meson
   pl111
   tegra
   tve200
   v3d
   vc4
   vkms
   bridge/dw-hdmi
   xen-front
   xe/index
   afbc
   komeda-kms
   panfrost

HiSilicon HiSTB display and audio
===============================

The Hi3798CV200 VDP exposes three linear RGB565/XRGB8888/ARGB8888 graphics
planes (G0, G1 and physical G3) and two NV12/NV21 video overlays (V0 and V1).
The third graphics plane is the BSP's logical HD2, not physical G2.
Graphics are unscaled; V0 supports progressive scaling and V1 supports
unscaled positioning with vertical chroma reconstruction. Both video inputs
share VP0; no independent VP1 is exposed. Video colour properties select
BT.601 or BT.709 and limited or full range.

G0/G1/G3 compose through MIXG0/GP0, and V0/V1 compose through MIXV0/VP0.
Atomic z-order must keep each group contiguous; interleaving the groups is
rejected. Graphics support premultiplied, coverage and opaque pixel
blending. A single visible graphics input uses GP0 for global alpha after
pixel composition. With multiple visible graphics inputs, premultiplied
formats with pixel alpha require fully opaque plane alpha: per-layer RGB
attenuation for that combination is not established by the hardware path.
Secondary graphics fetchers retain their own request/FIFO policy rather
than inheriting G0's request grouping.

CV200 V0 uses the normal full-tap 8/6/4/4 filter provider. Its DMA image
stores twelve signed 10-bit coefficients per 16-byte record, with Q9
normalization and a padding phase. VDM performs the SDR YCbCr-to-RGB
conversion in the BSP's 14-bit YCbCr input domain with gamma and
normalization bypassed. Legacy V0 CSC registers are not the active SDR
conversion path. No CRTC gamma LUT is exposed: the available CV200 DHD
sources do not establish its RGB sampling and packed-table contract.

V0 additionally accepts native packed NV15 ten-bit video. P010 input is
repacked without reducing its ten significant bits into double-buffered
NV15 storage owned by the framebuffer; native NV15 remains zero-copy when
its DMA mapping is contiguous. The HDR route accepts already-HDR,
limited-range BT.2020 video with HDR10/PQ or HLG metadata and performs
nonlinear YCbCr-to-RGB conversion. HDMI RGB output bypasses the TX matrix;
YCbCr output selects the imagev200 built-in BT.2020 non-constant-luminance
RGB-to-limited-YCbCr conversion. The sink must advertise the matching
BT.2020 RGB or YCC colorimetry, transfer function, static metadata and at
least ten-bit output. HDR DRM packets use the CV200 GEN3 packet buffer;
the generic register header's GEN4 comment does not describe this path.
SDR graphics, V1 and transfer-function/tone-map conversion are not supported
in this HDR composition. Removing HDR metadata restores the SDR path.
An eight-bit capture of ten-bit input proves fetch/conversion behaviour,
not preservation of ten-bit precision or HDR transfer at the HDMI sink.
The available CV200 sources do not establish the GDM sixteen-bit coefficient
decoder or all VDM/GDM-to-CVM numerical shifts. Register widths alone are not
sufficient to generate independent transfer/tone-map tables. No speculative
LUTs or source-transfer property are exposed for those unsupported routes.


The HDMI connector exposes Content Protection for HDCP Content Type 0.
Authentication requires licensed, administratively trusted provisioning:

* ``hisilicon/hi3798cv200-hdcp14.bin``: the board's 332-byte encrypted key
  package, not a raw key; its package CRC and provisioned root selector are
  checked before cipher-to-transmitter loading.
* ``hisilicon/hi3798cv200-hdcp22.bin``: the platform's 16384-byte raw MCU
  program, not the BSP C initializer text; the hardware upload/code-check
  handshake is required.
* ``display_hdcp_srm.bin``: an authentic DCP revocation list. Bounds and
  revocation identifiers are validated from one firmware snapshot, but the
  driver does not cryptographically verify the SRM signature. Provisioning
  must establish that trust outside this driver.

The non-test HDCP2 SRM published at
https://www.digital-cp.com/resources is distinct from its test SRM. Archive
origin, CRCs and MCU code equality do not replace signature/provisioning
trust. A normal-world PRAM readback failure must stop authentication even
with the correct MCU image; TEE-protected platforms require their matching
TrustedCore HDMI service instead of speculative security-register writes.

Missing or rejected provisioning leaves Content Protection Desired, never
Enabled. A successful property update requires observed authentication and
encryption, including repeater topology/revocation checks. Source inspection
and missing-firmware negative tests do not establish television
authentication. HDMI ARC is not exposed: the available CV200 sources do
not establish an HDMI ARC-receiver-to-AIAO hardware route.

The CV200 CEC adapter obtains its physical address from the connector EDID.
Transmit completion is reported only from hardware ACK/NACK status, and
received frames are drained from the CPI FIFO into the CEC framework. A
bounded status worker services the transmitter's native process-context
CPI path. Bus/core and PWD resets restore the claimed address and CPI
configuration; detach invalidates the connector association before it is
released. CEC control transport is separate from ARC audio reception.

MV310 has a separate graphics fetch engine. Its primary RGB plane does not
scale; the CV200 V0 register layout and filter tables are not used on MV310.

Scanout requires aligned, 32-bit-addressable DMA ranges. Physically
contiguous PRIME imports remain zero-copy; scattered imports use a
contiguous shadow. Both NV12/NV21 pitches and both complete DMA ranges
are checked before committing. Atomic video-only updates use the same
vblank/event boundary as graphics updates, and scaler coefficient images
are double-buffered until the preceding commit retires.

Kernel, DTB and the complete module tree must come from the same build,
including when rolling back. An identical kernel release string does not
guarantee matching ``CONFIG_MODVERSIONS`` symbol versions. Restoring only
the boot image can leave display, audio and other modules unable to load.
On usr-merged systems, preserve the ``/lib`` symlink: install the module
subdirectory into ``/usr/lib`` instead of extracting a top-level ``lib``
directory over the root filesystem.

With ``CONFIG_DEBUG_FS``, ``/sys/kernel/debug/dri/<device>/vdp_regs``
provides read-only register snapshots and accumulated underflow/master
error IRQ counts. Snapshots do not access a reset or clock-gated VDP.

The HDMI bridges expose per-device ``hdmi-codec`` DAIs, fed by AIAO TX3
on CV200 and TX2 on MV310. AIAO DAI 0 retains board-selected analog routing;
DAI 1 selects native HDMI. The links share one playback owner: opening the
other link during playback returns ``-EBUSY`` without changing the active
TX bank. CV200 HDMI accepts 2/6/8-channel LPCM; MV310 remains stereo.
Rates are 32/44.1/48/88.2/96/176.4/192 kHz, constrained by sink ELD,
using S16_LE or 24 significant MSBs in S32_LE. Matching updated DTBs are
required for the one-cell AIAO DAI specifier; CV200 DVB-IP cards enumerate
analog and HDMI as separate PCM devices.

CV200 multichannel samples follow the exposed CEA channel map:
``FL FR LFE FC RL RR [RLC RRC]``, with allocations 0x0b and 0x13 for
5.1 and 7.1 respectively. A conventional WAV channel order is not
automatically equivalent. AIAO has no six-slot encoding: six-channel
streams retain a six-channel ALSA mmap ring and are expanded, on submission,
into a coherent eight-slot hardware ring with a silent final pair. Stereo
and eight-channel streams remain direct DMA. Physical queue positions and
the 32-byte guard use hardware frame stride. Playback must be prepared and
refilled after system suspend; stale FIFO pointers are never resumed.

CV200 also accepts already-packed IEC61937 bursts. The PCM-interface
controls ``IEC61937 Codec``, ``IEC61937 Content Rate`` and
``IEC61937 Content Channels`` describe the compressed programme, separately
from the ALSA carrier. Codec choices are ``PCM``, ``AC-3``, ``DTS``,
``E-AC-3``, ``DTS-HD`` and ``MLP``. Set these controls and the
``IEC958 Playback Default`` NONAUDIO bit **before opening the PCM**;
changed writes while open return ``-EBUSY``. PCM requires NONAUDIO clear
and codec ``PCM``. Passthrough requires NONAUDIO set, consumer IEC958
status and the matching codec. The driver validates codec/content channels
and rate against the sink's SADs, not against LPCM carrier dimensions.

The supported S16_LE passthrough carriers are:

* AC-3: two channels at the content rate, up to 48 kHz/six content channels.
* DTS: two channels at the content rate, up to 96 kHz/six content channels.
* E-AC-3: two channels at four times the content rate.
* DTS-HD/MLP HBR: eight channels at 176.4 kHz for 44.1/88.2/176.4 kHz
  content, or 192 kHz for 48/96/192 kHz content. HBR IEC status describes
  the 705.6/768 kHz stereo equivalent, while ACR uses the physical carrier.

An unsupported 128 kHz physical carrier is rejected. The application must
provide framed bursts, not elementary compressed packets, and must not
resample, change software volume or remix their channels. The driver does
not decode bursts to verify the supplied content description. MV310 does
not expose these passthrough controls or new multichannel capabilities.

For a card whose HDMI playback is ``hw:0,1``, address controls explicitly,
for example::

    amixer -c 0 cget "iface=PCM,name='IEC61937 Codec',device=1"
    amixer -c 0 cset "iface=PCM,name='IEC61937 Codec',device=1" AC-3
    amixer -c 0 cset "iface=PCM,name='IEC61937 Content Rate',device=1" 48000
    amixer -c 0 cset "iface=PCM,name='IEC61937 Content Channels',device=1" 6

Use libasound's IEC958 control API to set NONAUDIO on that PCM/device, or
``iecset -D hw:0 professional off audio off`` when that card has a unique
IEC958 playback-default control. ``amixer`` does not implement IEC958
ASCII-value writes; ``iecset -n`` selects a control index, not a PCM device.
Then a packed AC-3 carrier can be submitted with::

    aplay -D hw:0,1 -t raw -f S16_LE -r 48000 -c 2 programme.iec61937

Restore consumer audio mode (``iecset ... audio on``) and codec ``PCM``
before opening LPCM again. Use an explicit ``hw`` PCM: ALSA ``hdmi`` or
``iec958`` hooks which change controls after opening can fail with
``-EBUSY`` or retain already-installed LPCM ELD constraints.

Neither ALSA write completion nor an active display proves that the sink
received HDMI audio. Multichannel and HBR positive acceptance requires a
capable receiver/analyser; rejection on a stereo-only sink is not that proof.

AIAO EMPTY is a level condition, including at normal end-of-stream drain.
It is consumed once and rearmed when further DMA data is published; ALSA
still distinguishes completed drain from underrun. Interrupt delivery is
stopped before the TX disable handshake, while persistent non-empty
interrupt failures remain reported and masked.


.. only::  subproject and html

   Indices
   =======

   * :ref:`genindex`
