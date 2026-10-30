# SPDX-License-Identifier: GPL-2.0-only

obj-$(CONFIG_DVB_HISTB_DMX) += histb-dmx.o
obj-$(CONFIG_VIDEO_HISTB_JPGE) += histb-jpge.o
obj-$(CONFIG_VIDEO_HISTB_JPGD) += histb-jpgd.o
obj-$(CONFIG_VIDEO_HISTB_PNG) += histb-png.o
obj-$(CONFIG_VIDEO_HISTB_VDEC) += histb-vdec.o
obj-$(CONFIG_VIDEO_HISTB_VENC) += histb-venc.o
obj-$(CONFIG_VIDEO_HISTB_VPSS) += histb-vpss.o
obj-$(CONFIG_VIDEO_HISTB_MEDIA_SMMU) += histb-media-smmu.o

histb-vdec-y := histb-vdec-core.o histb-vdec-v5r6.o \
		histb-vdec-avs2.o histb-vdec-mpeg1.o histb-vdec-mpeg4.o \
		histb-vdec-vc1.o histb-vdec-mvc.o histb-vdec-mvc-ingress.o
histb-venc-y := histb-venc-core.o histb-venc-rc.o histb-venc-v500.o \
		histb-venc-v500-syntax.o
