/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HISTB_VPSS_H
#define HISTB_VPSS_H

#include <linux/types.h>

struct device;
struct histb_vpss;

struct histb_vpss_frame {
	dma_addr_t input_dma;
	dma_addr_t output_dma;
	void *output_cpu;
	u32 input_width;
	u32 input_height;
	u32 width;
	u32 height;
	u32 input_stride;
	u32 input_height_align;
	u32 output_stride;
	bool input_ten_bit;
	bool output_ten_bit;
};

struct histb_vpss *histb_vpss_get(struct device *consumer);
void histb_vpss_put(struct histb_vpss *vpss);
int histb_vpss_detile(struct histb_vpss *vpss,
		      const struct histb_vpss_frame *frame);

#endif
