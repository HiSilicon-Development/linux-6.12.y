// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hi3798CV200 media SMMU page-table and IOVA lifecycle.
 *
 * The CV200 media engines consume a 32-bit, one-entry-per-4KiB page table.
 * Keep the table, fault sinks and IOVA allocator shared across HiVXE clients.
 */
#include <linux/build_bug.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/genalloc.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/sizes.h>

#include "histb-media-smmu.h"

struct histb_media_smmu_buffer {
	void *cpu;
	dma_addr_t dma;
	size_t size;
};

struct histb_media_smmu_state {
	struct device *dev;
	struct histb_media_smmu_buffer raw;
	void *pt_cpu;
	void *err_rd_cpu;
	void *err_wr_cpu;
	dma_addr_t pt_dma;
	dma_addr_t err_rd_dma;
	dma_addr_t err_wr_dma;
	struct gen_pool *iova_pool;
	unsigned int refs;
};

static DEFINE_MUTEX(histb_media_smmu_lock);
static struct histb_media_smmu_state *histb_media_smmu_global;

bool histb_media_smmu_dma_range_valid(dma_addr_t dma, size_t size)
{
	dma_addr_t end;

	if (!size || size > HISTB_MEDIA_SMMU_DMA_SIZE || dma < SZ_1M ||
	    check_add_overflow(dma, (dma_addr_t)(size - 1), &end))
		return false;

	return end < HISTB_MEDIA_SMMU_DMA_SIZE;
}
EXPORT_SYMBOL_GPL(histb_media_smmu_dma_range_valid);

static int smmu_aligned_view(const struct histb_media_smmu_buffer *raw,
			     size_t alignment, size_t size, void **cpu,
			     dma_addr_t *dma)
{
	dma_addr_t aligned, end;
	size_t offset;

	if (!raw->cpu || !raw->size || !size || !alignment)
		return -EINVAL;

	aligned = ALIGN(raw->dma, alignment);
	if (aligned < raw->dma)
		return -ERANGE;
	offset = (size_t)(aligned - raw->dma);
	if (offset > raw->size || size > raw->size - offset)
		return -ERANGE;
	end = aligned + size - 1;
	if (end < aligned || upper_32_bits(end))
		return -ERANGE;

	*cpu = (u8 *)raw->cpu + offset;
	*dma = aligned;
	return 0;
}

static void smmu_state_free(struct histb_media_smmu_state *state)
{
	if (!state)
		return;
	if (state->iova_pool)
		gen_pool_destroy(state->iova_pool);
	if (state->raw.cpu)
		dma_free_coherent(state->dev, state->raw.size, state->raw.cpu,
				  state->raw.dma);
	put_device(state->dev);
	kfree(state);
}

void histb_media_smmu_release_identity(void *data)
{
	struct histb_media_smmu *smmu = data;
	struct histb_media_smmu_state *state;
	bool free_state = false;

	mutex_lock(&histb_media_smmu_lock);
	state = smmu->state;
	smmu->state = NULL;
	smmu->identity_needs_reset = false;
	if (state && --state->refs == 0) {
		histb_media_smmu_global = NULL;
		free_state = true;
	}
	mutex_unlock(&histb_media_smmu_lock);

	if (free_state)
		smmu_state_free(state);
}
EXPORT_SYMBOL_GPL(histb_media_smmu_release_identity);

int histb_media_smmu_prepare_identity(struct histb_media_smmu *smmu,
				      struct device *dev)
{
	struct histb_media_smmu_state *state;
	size_t allocation;
	dma_addr_t end;
	unsigned int i;
	int ret;

	BUILD_BUG_ON(HISTB_MEDIA_SMMU_PGTABLE_ENTRIES * sizeof(__le32) >
		     HISTB_MEDIA_SMMU_PGTABLE_SIZE);

	mutex_lock(&histb_media_smmu_lock);
	if (smmu->state) {
		mutex_unlock(&histb_media_smmu_lock);
		return 0;
	}

	state = histb_media_smmu_global;
	if (state) {
		/* Keep the allocating driver's DMA pool bound while clients use it. */
		if (dev != state->dev &&
		    !device_link_add(dev, state->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
			mutex_unlock(&histb_media_smmu_lock);
			return -ENOMEM;
		}
		state->refs++;
		smmu->state = state;
		smmu->identity_needs_reset = true;
		mutex_unlock(&histb_media_smmu_lock);
		return 0;
	}

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state) {
		mutex_unlock(&histb_media_smmu_lock);
		return -ENOMEM;
	}

	if (check_add_overflow((size_t)HISTB_MEDIA_SMMU_PGTABLE_SIZE,
			       (size_t)HISTB_MEDIA_SMMU_PGTABLE_ALIGN - 1,
			       &allocation) ||
	    check_add_overflow(allocation,
			       (size_t)HISTB_MEDIA_SMMU_ERROR_POOL, &allocation)) {
		ret = -EOVERFLOW;
		goto fail;
	}

	state->dev = get_device(dev);
	state->raw.size = allocation;
	state->raw.cpu = dma_alloc_coherent(dev, allocation, &state->raw.dma,
					    GFP_KERNEL);
	if (!state->raw.cpu) {
		ret = -ENOMEM;
		goto fail;
	}

	ret = smmu_aligned_view(&state->raw,
				HISTB_MEDIA_SMMU_PGTABLE_ALIGN,
				HISTB_MEDIA_SMMU_PGTABLE_SIZE,
				&state->pt_cpu, &state->pt_dma);
	if (ret)
		goto fail;

	state->err_rd_cpu = (u8 *)state->pt_cpu +
			    HISTB_MEDIA_SMMU_PGTABLE_SIZE;
	state->err_rd_dma = state->pt_dma + HISTB_MEDIA_SMMU_PGTABLE_SIZE;
	state->err_wr_cpu = (u8 *)state->err_rd_cpu +
			    HISTB_MEDIA_SMMU_ERROR_SIZE;
	state->err_wr_dma = state->err_rd_dma + HISTB_MEDIA_SMMU_ERROR_SIZE;
	end = state->err_wr_dma + HISTB_MEDIA_SMMU_ERROR_SIZE - 1;
	if (end < state->err_wr_dma || end > state->raw.dma + allocation - 1 ||
	    !histb_media_smmu_dma_range_valid(state->pt_dma,
					       HISTB_MEDIA_SMMU_PGTABLE_SIZE) ||
	    !histb_media_smmu_dma_range_valid(state->err_rd_dma,
					       HISTB_MEDIA_SMMU_ERROR_SIZE) ||
	    !histb_media_smmu_dma_range_valid(state->err_wr_dma,
					       HISTB_MEDIA_SMMU_ERROR_SIZE)) {
		ret = -ERANGE;
		goto fail;
	}

	state->iova_pool = gen_pool_create(order_base_2(SZ_4K), -1);
	if (!state->iova_pool) {
		ret = -ENOMEM;
		goto fail;
	}
	ret = gen_pool_add(state->iova_pool, HISTB_MEDIA_SMMU_DMA_SIZE,
			   HISTB_MEDIA_SMMU_IOVA_SIZE - HISTB_MEDIA_SMMU_DMA_SIZE,
			   -1);
	if (ret)
		goto fail;

	memset(state->raw.cpu, 0, state->raw.size);
	/* Reserve the first MiB and last MiB; low physical DMA stays identity. */
	for (i = SZ_1M / HISTB_MEDIA_SMMU_PAGE_SIZE;
	     i < HISTB_MEDIA_SMMU_DMA_SIZE / HISTB_MEDIA_SMMU_PAGE_SIZE; i++)
		((__le32 *)state->pt_cpu)[i] =
			cpu_to_le32(i * HISTB_MEDIA_SMMU_PAGE_SIZE | 1);
	memset(state->err_rd_cpu, 0, HISTB_MEDIA_SMMU_ERROR_SIZE);
	memset(state->err_wr_cpu, 0, HISTB_MEDIA_SMMU_ERROR_SIZE);
	dma_wmb();

	state->refs = 1;
	histb_media_smmu_global = state;
	smmu->state = state;
	smmu->identity_needs_reset = true;
	mutex_unlock(&histb_media_smmu_lock);
	return 0;

fail:
	mutex_unlock(&histb_media_smmu_lock);
	smmu_state_free(state);
	return ret;
}
EXPORT_SYMBOL_GPL(histb_media_smmu_prepare_identity);

bool histb_media_smmu_ready(const struct histb_media_smmu *smmu)
{
	return smmu && smmu->state;
}
EXPORT_SYMBOL_GPL(histb_media_smmu_ready);

dma_addr_t histb_media_smmu_pt_dma(const struct histb_media_smmu *smmu)
{
	return smmu->state->pt_dma;
}
EXPORT_SYMBOL_GPL(histb_media_smmu_pt_dma);

dma_addr_t histb_media_smmu_err_rd_dma(const struct histb_media_smmu *smmu)
{
	return smmu->state->err_rd_dma;
}
EXPORT_SYMBOL_GPL(histb_media_smmu_err_rd_dma);

dma_addr_t histb_media_smmu_err_wr_dma(const struct histb_media_smmu *smmu)
{
	return smmu->state->err_wr_dma;
}
EXPORT_SYMBOL_GPL(histb_media_smmu_err_wr_dma);

int histb_media_smmu_map_phys(struct histb_media_smmu *smmu,
				      dma_addr_t phys, size_t size, unsigned int align,
				      struct histb_media_smmu_mapping *mapping)
{
	struct histb_media_smmu_state *state;
	size_t pool_size, mapped, mapped_size;
	unsigned long pool_base, iova;
	unsigned int page;
	int ret = 0;

	if (!smmu || !mapping || !size || !IS_ALIGNED(phys, SZ_4K))
		return -EINVAL;
	if (check_add_overflow(size, (size_t)SZ_4K - 1, &mapped_size))
		return -EOVERFLOW;
	mapped_size &= ~((size_t)SZ_4K - 1);
	if (!histb_media_smmu_dma_range_valid(phys, mapped_size))
		return -EINVAL;
	align = max_t(unsigned int, align, SZ_4K);
	if (!is_power_of_2(align))
		return -EINVAL;
	if (check_add_overflow(mapped_size, (size_t)align - 1, &pool_size))
		return -EOVERFLOW;

	mutex_lock(&histb_media_smmu_lock);
	state = smmu->state;
	if (!state) {
		ret = -EINVAL;
		goto unlock;
	}
	if (mapping->state) {
		ret = -EBUSY;
		goto unlock;
	}
	pool_base = gen_pool_alloc(state->iova_pool, pool_size);
	if (!pool_base) {
		ret = -ENOMEM;
		goto unlock;
	}
	iova = ALIGN(pool_base, align);
	if (iova < pool_base || iova + mapped_size > pool_base + pool_size) {
		gen_pool_free(state->iova_pool, pool_base, pool_size);
		ret = -ERANGE;
		goto unlock;
	}

	for (mapped = 0; mapped < mapped_size; mapped += SZ_4K) {
		page = (iova + mapped) / SZ_4K;
		((__le32 *)state->pt_cpu)[page] =
			cpu_to_le32((phys + mapped) | 1);
	}
	dma_wmb();
	state->refs++;
	mapping->state = state;
	mapping->iova = iova;
	mapping->pool_base = pool_base;
	mapping->pool_size = pool_size;
	mapping->size = mapped_size;
unlock:
	mutex_unlock(&histb_media_smmu_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(histb_media_smmu_map_phys);

void histb_media_smmu_unmap(struct histb_media_smmu_mapping *mapping)
{
	struct histb_media_smmu_state *state;
	size_t unmapped;
	unsigned int page;
	bool free_state;

	if (!mapping || !mapping->state || !mapping->size)
		return;
	state = mapping->state;
	mutex_lock(&histb_media_smmu_lock);
	for (unmapped = 0; unmapped < mapping->size; unmapped += SZ_4K) {
		page = (mapping->iova + unmapped) / SZ_4K;
		((__le32 *)state->pt_cpu)[page] = 0;
	}
	dma_wmb();
	gen_pool_free(state->iova_pool, mapping->pool_base,
		      mapping->pool_size);
	memset(mapping, 0, sizeof(*mapping));
	free_state = --state->refs == 0;
	if (free_state)
		histb_media_smmu_global = NULL;
	mutex_unlock(&histb_media_smmu_lock);
	if (free_state)
		smmu_state_free(state);
}
EXPORT_SYMBOL_GPL(histb_media_smmu_unmap);

MODULE_LICENSE("GPL");
