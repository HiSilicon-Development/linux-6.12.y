// SPDX-License-Identifier: GPL-2.0-only
#ifndef HISTB_VENC_V500_H
#define HISTB_VENC_V500_H

#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/types.h>

#define HISTB_V500_IMAGE_SIZE 4096
#define HISTB_V500_INT_MASK 0x0000
#define HISTB_V500_INT_CLEAR 0x0004
#define HISTB_V500_START 0x0008
#define HISTB_V500_STATUS 0x3004
#define HISTB_V500_DONE BIT(0)
#define HISTB_V500_ERRORS (BIT(2) | BIT(3) | BIT(6) | BIT(7))

/* Word offsets match venc_hal_ext.h's 32-bit wire layout. No host pointers. */
struct histb_v500_config {
	u32 reserved_base;
	struct {
		u32 protocol, profile, width, height;
		u32 split_enable, split_mode, split_size, csc, secure;
		u32 low_delay, timeout, pt_bits, parameter_set, ddr_mode;
	} control;
	struct {
		u32 store, package, source_y, source_c, source_v;
		u32 source_stride_y, source_stride_c;
		u32 recon_y[2], recon_c[2], recon_stride_y, recon_stride_c, recon_index;
		u32 tmv[2], pme[2], pme_info[3], pme_index, poc[6], tunnel;
		u32 stream[16], stream_size[16], stream_rp, stream_wp;
		u32 parameter_set, parameter_size, parameter_words[48];
	} buffer;
	struct {
		u32 intra, target_bits, qp, min_qp, max_qp;
		s32 i_delta, p_delta;
		u32 avbr;
	} rc;
	struct {
		u32 source_y_bypass, source_c_bypass, source_v_bypass;
		u32 ns_table, ns_error_write, ns_error_read;
		u32 secure_table, secure_error_write, secure_error_read;
	} smmu;
	struct {
		u32 header[4], reorder[2], marking[2], bits[2], cabac[8], part1;
	} slice;
	u32 pts[2], ext_flag, ext_fill, image_dma;
};

struct histb_v500_memory {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	u32 luma_size, stride;
	u32 tmv[2], pme[2], pme_info[3];
	u32 image_offset, pointers_offset;
	bool hevc;
};

/* Capacity-1080P layout: nonzero dimensions up to 1920x1088. The complete
 * coherent allocation must fit the 32-bit hardware DMA address domain.
 * Allocation failure leaves an empty memory object; invalid dimensions leave
 * any existing allocation untouched.
 */
/* Caller serializes MMIO access and supplies a zeroed 4096-byte image and
 * a separate 0x116c-byte tuning workspace. Configuration must use valid
 * hardware dimensions (height >= 16 for H.264, >= 32 for HEVC), recon_index
 * below two and pme_index below three. All buffer addresses are 32-bit DMA.
 */
int histb_v500_alloc(struct device *dev, struct histb_v500_memory *memory,
		     unsigned int width, unsigned int height, bool hevc);
void histb_v500_free(struct device *dev, struct histb_v500_memory *memory);
void histb_v500_configure(void __iomem *regs, struct histb_v500_config *config,
			 u32 *image, u32 *tune);
void histb_v500_bypass(void __iomem *regs);
#endif
