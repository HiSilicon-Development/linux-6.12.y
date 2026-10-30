/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __HISTB_MEDIA_SMMU_H__
#define __HISTB_MEDIA_SMMU_H__

#include <linux/dma-mapping.h>
#include <linux/sizes.h>
#include <linux/types.h>

#define HISTB_MEDIA_SMMU_IOVA_SIZE		0xfff00000ULL
#define HISTB_MEDIA_SMMU_DMA_SIZE		SZ_1G
#define HISTB_MEDIA_SMMU_PAGE_SIZE		SZ_4K
#define HISTB_MEDIA_SMMU_PGTABLE_ENTRIES	\
	(HISTB_MEDIA_SMMU_IOVA_SIZE / HISTB_MEDIA_SMMU_PAGE_SIZE)
#define HISTB_MEDIA_SMMU_PGTABLE_SIZE		SZ_4M
#define HISTB_MEDIA_SMMU_PGTABLE_ALIGN	SZ_1M
#define HISTB_MEDIA_SMMU_ERROR_SIZE		0x200U
#define HISTB_MEDIA_SMMU_ERROR_ALIGN		0x100U
#define HISTB_MEDIA_SMMU_ERROR_POOL		\
	(2 * HISTB_MEDIA_SMMU_ERROR_SIZE + HISTB_MEDIA_SMMU_ERROR_ALIGN)

struct histb_media_smmu_state;

struct histb_media_smmu {
	struct histb_media_smmu_state *state;
	bool identity_needs_reset;
};

/* Zero-initialize before mapping; unmap only after the engine stops DMA. */
struct histb_media_smmu_mapping {
	struct histb_media_smmu_state *state;
	dma_addr_t iova;
	unsigned long pool_base;
	size_t pool_size;
	size_t size;
};

int histb_media_smmu_prepare_identity(struct histb_media_smmu *smmu,
				      struct device *dev);
void histb_media_smmu_release_identity(void *data);
bool histb_media_smmu_ready(const struct histb_media_smmu *smmu);
dma_addr_t histb_media_smmu_pt_dma(const struct histb_media_smmu *smmu);
dma_addr_t histb_media_smmu_err_rd_dma(const struct histb_media_smmu *smmu);
dma_addr_t histb_media_smmu_err_wr_dma(const struct histb_media_smmu *smmu);
bool histb_media_smmu_dma_range_valid(dma_addr_t dma, size_t size);
/* Each successful mapping retains the shared table until unmap. */
int histb_media_smmu_map_phys(struct histb_media_smmu *smmu,
			      dma_addr_t phys, size_t size, unsigned int align,
			      struct histb_media_smmu_mapping *mapping);
void histb_media_smmu_unmap(struct histb_media_smmu_mapping *mapping);

#endif
