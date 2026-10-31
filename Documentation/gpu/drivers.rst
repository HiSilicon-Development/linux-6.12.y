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

The Hi3798CV200 VDP exposes a linear RGB565/XRGB8888/ARGB8888 primary
graphics plane (G0) and an NV12/NV21 video overlay (V0). G0 is unscaled;
V0 supports progressive scaling, positioning, global alpha and ordering
relative to G0. Video colour properties select BT.601 or BT.709 and limited
or full range. Graphics support premultiplied, coverage and opaque pixel
blending; global graphics alpha is applied at GP0 after pixel composition.

CV200 video uses the normal full-tap 8/6/4/4 filter provider. Its DMA image
stores twelve signed 10-bit coefficients per 16-byte record, with Q9
normalization and a padding phase. VDM performs the SDR YCbCr-to-RGB
conversion in the BSP's 14-bit YCbCr input domain with gamma and
normalization bypassed. Legacy V0 CSC registers are not the active SDR
conversion path. No CRTC gamma LUT is exposed: the available CV200 DHD
sources do not establish its RGB sampling and packed-table contract.

MV310 has a separate graphics fetch engine. Its primary RGB plane does not
scale; the CV200 V0 register layout and filter tables are not used on MV310.

Scanout requires aligned, 32-bit-addressable DMA ranges. Physically
contiguous PRIME imports remain zero-copy; scattered imports use a
contiguous shadow. Both NV12/NV21 pitches and both complete DMA ranges
are checked before committing. Atomic video-only updates use the same
vblank/event boundary as graphics updates, and scaler coefficient images
are double-buffered until the preceding commit retires.

With ``CONFIG_DEBUG_FS``, ``/sys/kernel/debug/dri/<device>/vdp_regs``
provides read-only register snapshots and accumulated underflow/master
error IRQ counts. Snapshots do not access a reset or clock-gated VDP.

The HDMI bridges expose per-device ``hdmi-codec`` DAIs, fed by AIAO TX3
on CV200 and TX2 on MV310. AIAO DAI 0 retains board-selected analog routing;
DAI 1 selects native HDMI. The links share one playback owner: opening the
other link during playback returns ``-EBUSY`` without changing the active
TX bank. HDMI accepts
stereo PCM at 32/44.1/48/88.2/96/176.4/192 kHz, constrained by sink ELD,
using S16_LE or 24 significant MSBs in S32_LE. Unsupported formats are
rejected. Matching updated DTBs are required for the one-cell AIAO DAI
specifier; CV200 DVB-IP cards enumerate analog and HDMI as separate PCM
devices. Neither ALSA write completion nor an active display proves that
the sink received HDMI audio.


.. only::  subproject and html

   Indices
   =======

   * :ref:`genindex`
