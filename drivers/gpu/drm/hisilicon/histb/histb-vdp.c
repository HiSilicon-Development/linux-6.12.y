// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM driver for the HiSilicon Hi3798CV200 video display processor.
 */

#include <linux/align.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-buf.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_prime.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#define HISTB_VDP_VOINTSTA		0x0004
#define HISTB_VDP_VOMSKINTSTA		0x0008
#define HISTB_VDP_VOINTMSK		0x000c
#define HISTB_VDP_VO_MUX		0x0100
#define HISTB_VDP_VOCTRL		0x0000
#define HISTB_VDP_VOMIDORDER		0x0028
#define HISTB_VDP_VOLOWPOWER_CTRL	0x002c
#define HISTB_VDP_VOAXICTRL		0x0030
#define HISTB_VDP_VOAXICTRL1		0x0034
#define HISTB_VDP_VOMASTERSEL		0x0038
#define HISTB_VDP_VERSION1		0x0020
#define HISTB_VDP_VERSION2		0x0024

#define HISTB_VDP_GDM_CTRL		0x3700
#define HISTB_VDP_GDM_RGB2LMS_CTRL	0x3800
#define HISTB_VDP_CVM_CTRL		0x3900
#define HISTB_VDP_CVM1_RGB2YUV_CTRL	0x3d00
#define HISTB_VDP_CVM2_RGB2YUV_CTRL	0x3e00
#define HISTB_VDP_CVM2_RGB2YUV_COEF0	0x3e04
#define HISTB_VDP_CVM2_RGB2YUV_SCALE2P	0x3e28
#define HISTB_VDP_CVM2_RGB2YUV_MIN	0x3e2c
#define HISTB_VDP_CVM2_RGB2YUV_MAX	0x3e30
#define HISTB_VDP_CVM2_RGB2YUV_OUT_DC0	0x3e34
#define HISTB_VDP_CVM2_RGB2YUV_OUT_DC1	0x3e38
#define HISTB_VDP_CVM2_RGB2YUV_OUT_DC2	0x3e3c
#define HISTB_VDP_HDR_CTRL		0x3f80
#define HISTB_VDP_HDR_UPD		0x3f84

#define HISTB_VDP_G0_CTRL		0x7000
#define HISTB_VDP_G0_UPD		0x7004
#define HISTB_VDP_G0_ADDR		0x7010
#define HISTB_VDP_G0_NADDR		0x7018
#define HISTB_VDP_G0_STRIDE		0x701c
#define HISTB_VDP_G0_IRESO		0x7020
#define HISTB_VDP_G0_SFPOS		0x7024
#define HISTB_VDP_G0_SMMU_BYPASS0	0x7028
#define HISTB_VDP_G0_CBMPARA		0x7030
#define HISTB_VDP_G0_DCMP_ADDR		0x7050
#define HISTB_VDP_G0_DCMP_NADDR		0x7054
#define HISTB_VDP_G0_DCMP_OFFSET	0x7058
#define HISTB_VDP_G0_DFPOS		0x7080
#define HISTB_VDP_G0_DLPOS		0x7084
#define HISTB_VDP_G0_VFPOS		0x7088
#define HISTB_VDP_G0_VLPOS		0x708c
#define HISTB_VDP_G0_ALPHA		0x7094
#define HISTB_VDP_G0_CSC_IDC		0x70c0

#define HISTB_VDP_GP0_CTRL		0x8000
#define HISTB_VDP_GP0_UPD		0x8004
#define HISTB_VDP_GP0_ORESO		0x8008
#define HISTB_VDP_GP0_IRESO		0x800c
#define HISTB_VDP_GP0_GALPHA		0x8020
#define HISTB_VDP_GP0_DFPOS		0x8100
#define HISTB_VDP_GP0_DLPOS		0x8104
#define HISTB_VDP_GP0_VFPOS		0x8108
#define HISTB_VDP_GP0_VLPOS		0x810c
#define HISTB_VDP_GP0_CSC_IDC		0x8120
#define HISTB_VDP_GP0_ZME_HSP		0x8140
#define HISTB_VDP_GP0_ZME_VSP		0x814c

#define HISTB_VDP_MIXG0_MIX		0xb208
#define HISTB_VDP_CBM_BKG1		0xb400
#define HISTB_VDP_CBM_MIX1		0xb408

#define HISTB_VDP_DHD0_CTRL		0xc000
#define HISTB_VDP_DHD0_VSYNC		0xc004
#define HISTB_VDP_DHD0_HSYNC1		0xc008
#define HISTB_VDP_DHD0_HSYNC2		0xc00c
#define HISTB_VDP_DHD0_VPLUS		0xc010
#define HISTB_VDP_DHD0_PWR		0xc014
#define HISTB_VDP_DHD0_VTTHD		0xc01c
#define HISTB_VDP_DHD0_SYNC_INV		0xc020
#define HISTB_VDP_DHD0_DATA_SEL		0xc02c
#define HISTB_VDP_DHD0_CSC_IDC		0xc040
#define HISTB_VDP_DHD0_HDMI_CLIP_L	0xc090
#define HISTB_VDP_DHD0_HDMI_CLIP_H	0xc094

#define HISTB_VDP_DHD0_VBLANK		BIT(0)

#define HISTB_VDP_G0_IFMT		GENMASK(7, 0)
#define HISTB_VDP_G0_BITEXT		GENMASK(9, 8)
#define HISTB_VDP_G0_REQ_CTRL		GENMASK(15, 14)
#define HISTB_VDP_G0_READ_PROGRESSIVE	BIT(26)
#define HISTB_VDP_G0_UPDATE_PROGRESSIVE	BIT(27)
#define HISTB_VDP_G0_NONSECURE		BIT(30)
#define HISTB_VDP_G0_ENABLE		BIT(31)
#define HISTB_VDP_G0_PIXEL_ALPHA	BIT(12)
#define HISTB_VDP_G0_STRIDE_MASK	GENMASK(15, 0)
#define HISTB_VDP_G0_STRIDE_UNIT	16

#define HISTB_VDP_GP0_READ_PROGRESSIVE	BIT(31)
#define HISTB_VDP_GP0_CSC_ENABLE	BIT(22)
#define HISTB_VDP_GP0_MUX1_SEL		GENMASK(1, 0)
#define HISTB_VDP_GP0_MUX2_SEL		GENMASK(5, 4)
#define HISTB_VDP_GP0_AOUT_SEL		GENMASK(9, 8)
#define HISTB_VDP_GP0_BOUT_SEL		GENMASK(13, 12)
/* CV200 Linux HIFB/PQ channel A: ZME -> CSC, mux1=3, mux2=0, A=2, B=1. */
#define HISTB_VDP_GP0_ROUTE_ZME_CSC	(FIELD_PREP(HISTB_VDP_GP0_MUX1_SEL, 3) | \
					 FIELD_PREP(HISTB_VDP_GP0_MUX2_SEL, 0) | \
					 FIELD_PREP(HISTB_VDP_GP0_AOUT_SEL, 2) | \
					 FIELD_PREP(HISTB_VDP_GP0_BOUT_SEL, 1))
#define HISTB_VDP_GP0_ZME_H_ENABLE	BIT(31)
#define HISTB_VDP_GP0_ZME_V_ENABLE	BIT(31)

#define HISTB_VDP_GDM_ENABLES		GENMASK(3, 0)
#define HISTB_VDP_GDM_RGB2LMS_ENABLE	BIT(0)
#define HISTB_VDP_CVM_ENABLE		BIT(0)
#define HISTB_VDP_CVM_SELECT		BIT(1)
#define HISTB_VDP_CVM_PIPE_ENABLES	GENMASK(5, 0)
#define HISTB_VDP_CVM2_RGB2YUV_ENABLE	BIT(0)
#define HISTB_VDP_CVM2_CSC_PRECISION	15
#define HISTB_VDP_CVM2_CSC_SCALE_ENABLE	BIT(5)
#define HISTB_VDP_CVM2_CSC_UNITY		BIT(14)
#define HISTB_VDP_HDR_ENABLES		GENMASK(31, 30)
#define HISTB_VDP_HDR_REGUP		BIT(0)

#define HISTB_VDP_DHD0_REGUP		BIT(0)
#define HISTB_VDP_DHD0_PROGRESSIVE	BIT(4)
#define HISTB_VDP_DHD0_HDMI_MODE	BIT(14)
#define HISTB_VDP_DHD0_TWOCHN_DEBUG	BIT(15)
#define HISTB_VDP_DHD0_CSC_ENABLE	BIT(22)
#define HISTB_VDP_DHD0_P2I_ENABLE	BIT(28)
#define HISTB_VDP_DHD0_CBAR_SELECT	BIT(29)
#define HISTB_VDP_DHD0_CBAR_ENABLE	BIT(30)
#define HISTB_VDP_DHD0_CBAR_MASK	(HISTB_VDP_DHD0_CBAR_SELECT | \
					 HISTB_VDP_DHD0_CBAR_ENABLE)
#define HISTB_VDP_DHD0_ENABLE		BIT(31)

#define HISTB_VDP_DATE_HSYNC_INV	BIT(13)

#define HISTB_VDP_DHD0_CLIP_FULL	GENMASK(29, 0)

#define HISTB_VDP_VO_MUX_HDMI_SEL	BIT(3)

/* CV200 global pixel-interleaver controls. */
#define HISTB_VDP_VOCTRL_TWOCHN_EN	BIT(24)
#define HISTB_VDP_VOCTRL_TWOCHN_MODE	BIT(25)
#define HISTB_VDP_VOCTRL_CLK_GATE_EN	BIT(31)
#define HISTB_VDP_VOLOWPOWER_DEFAULT	0x00005b2b

#define HISTB_VDP_G0_SMMU_BYPASS_2D	BIT(0)
#define HISTB_VDP_G0_SMMU_BYPASS_3D_READ BIT(1)
#define HISTB_VDP_G0_CSC_ENABLE		BIT(22)
#define HISTB_VDP_G0_OFL_MASTER		BIT(29)
#define HISTB_VDP_VOMASTERSEL_G0		BIT(14)
#define HISTB_VDP_VOMIDORDER_MID_ENABLE	BIT(3)

/* Values used by the CV200 BSP VDP_DRIVER_Initial() AXI setup. */
#define HISTB_VDP_VOAXICTRL_FIELDS	GENMASK(11, 0)
#define HISTB_VDP_VOAXICTRL_MULTI0	BIT(15)
#define HISTB_VDP_VOAXICTRL_FIELDS1	GENMASK(27, 16)
#define HISTB_VDP_VOAXICTRL_MULTI1	BIT(31)
#define HISTB_VDP_VOAXICTRL_MASK	(HISTB_VDP_VOAXICTRL_FIELDS | \
					 HISTB_VDP_VOAXICTRL_MULTI0 | \
					 HISTB_VDP_VOAXICTRL_FIELDS1 | \
					 HISTB_VDP_VOAXICTRL_MULTI1)
#define HISTB_VDP_VOAXICTRL_VALUE	0x07ff87ff
#define HISTB_VDP_VOAXICTRL1_FIELDS	(GENMASK(11, 0) | GENMASK(27, 16))
#define HISTB_VDP_VOAXICTRL1_VALUE	0x07ff07ff

#define HISTB_VDP_SIZE(w, h)		(((w) - 1) | (((h) - 1) << 12))
#define HISTB_VDP_POS(x, y)		((x) | ((y) << 12))

#define HISTB_VDP_MIN_WIDTH		320
#define HISTB_VDP_MIN_HEIGHT		240
#define HISTB_VDP_MAX_WIDTH		4096
#define HISTB_VDP_MAX_HEIGHT		2160

enum histb_vdp_gfx_format {
	HISTB_VDP_GFX_RGB565 = 0x42,
	HISTB_VDP_GFX_XRGB8888 = 0x60,
	HISTB_VDP_GFX_ARGB8888 = 0x68,
};

struct histb_vdp {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector *connector;
	void __iomem *base;
	struct clk_bulk_data clks[7];
	unsigned int num_clks;
	struct reset_control *reset;
	/* Protects interrupt state while the register clock is running. */
	spinlock_t irq_lock;
	int irq;
	bool resources_enabled;
	bool powered;
	bool vblank_enabled;
};

/*
 * The CV200 VDP fetches through a 32-bit, non-IOMMU AXI path.  A PRIME
 * buffer exported by a renderer may be physically scattered, so retain the
 * imported attachment for the source while scanning out a private,
 * physically contiguous DMA shadow.
 */
struct histb_vdp_shadow_object {
	struct drm_gem_dma_object dma;
	struct sg_table *import_sgt;
	size_t source_size;
};

static const struct drm_gem_object_funcs histb_vdp_shadow_object_funcs;

static const u32 histb_vdp_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static inline struct histb_vdp *to_histb_vdp(struct drm_device *drm)
{
	return container_of(drm, struct histb_vdp, drm);
}

static inline struct histb_vdp_shadow_object *
to_histb_vdp_shadow_object(struct drm_gem_object *obj)
{
	if (obj->funcs != &histb_vdp_shadow_object_funcs)
		return NULL;

	return container_of(to_drm_gem_dma_obj(obj),
				    struct histb_vdp_shadow_object, dma);
}

static inline bool histb_vdp_dma_address_valid(dma_addr_t address)
{
	return !upper_32_bits(address) &&
	       IS_ALIGNED(address, HISTB_VDP_G0_STRIDE_UNIT);
}

static void histb_vdp_shadow_object_free(struct drm_gem_object *obj)
{
	struct histb_vdp_shadow_object *shadow =
		to_histb_vdp_shadow_object(obj);

	if (shadow->dma.vaddr)
		dma_free_wc(obj->dev->dev, obj->size, shadow->dma.vaddr,
			     shadow->dma.dma_addr);

	if (obj->import_attach)
		drm_prime_gem_destroy(obj, shadow->import_sgt);

	drm_gem_object_release(obj);
	kfree(shadow);
}

/* drm_gem_vmap() holds obj->resv, which is the imported dma-buf reservation. */
static int histb_vdp_shadow_object_vmap(struct drm_gem_object *obj,
					struct iosys_map *map)
{
	struct dma_buf_attachment *attach = obj->import_attach;

	if (!attach)
		return -EINVAL;

	return dma_buf_vmap(attach->dmabuf, map);
}

static void histb_vdp_shadow_object_vunmap(struct drm_gem_object *obj,
					   struct iosys_map *map)
{
	struct dma_buf_attachment *attach = obj->import_attach;

	if (attach)
		dma_buf_vunmap(attach->dmabuf, map);
}

static const struct drm_gem_object_funcs histb_vdp_shadow_object_funcs = {
	.free = histb_vdp_shadow_object_free,
	.print_info = drm_gem_dma_object_print_info,
	.get_sg_table = drm_gem_dma_object_get_sg_table,
	.vmap = histb_vdp_shadow_object_vmap,
	.vunmap = histb_vdp_shadow_object_vunmap,
	.mmap = drm_gem_dma_object_mmap,
	.vm_ops = &drm_gem_dma_vm_ops,
};

static struct drm_gem_object *
histb_vdp_prime_import_sg_table(struct drm_device *dev,
				struct dma_buf_attachment *attach,
				struct sg_table *sgt)
{
	struct histb_vdp_shadow_object *shadow;
	struct drm_gem_object *obj;
	size_t size;
	int ret;

	/* Preserve the normal zero-copy path whenever the AXI address is linear. */
	if (drm_prime_get_contiguous_size(sgt) >= attach->dmabuf->size)
		return drm_gem_dma_prime_import_sg_table_vmap(dev, attach, sgt);

	size = PAGE_ALIGN(attach->dmabuf->size);
	if (!size)
		return ERR_PTR(-EINVAL);

	shadow = kzalloc(sizeof(*shadow), GFP_KERNEL);
	if (!shadow)
		return ERR_PTR(-ENOMEM);

	obj = &shadow->dma.base;
	obj->funcs = &histb_vdp_shadow_object_funcs;
	drm_gem_private_object_init(dev, obj, size);
	shadow->import_sgt = sgt;
	shadow->source_size = attach->dmabuf->size;
	shadow->dma.map_noncoherent = false;

	ret = drm_gem_create_mmap_offset(obj);
	if (ret)
		goto err_release;

	shadow->dma.vaddr = dma_alloc_wc(dev->dev, size,
					&shadow->dma.dma_addr,
					GFP_KERNEL | __GFP_NOWARN);
	if (!shadow->dma.vaddr) {
		ret = -ENOMEM;
		goto err_release;
	}

	if (!histb_vdp_dma_address_valid(shadow->dma.dma_addr)) {
		ret = -EINVAL;
		dma_free_wc(dev->dev, size, shadow->dma.vaddr,
				shadow->dma.dma_addr);
		shadow->dma.vaddr = NULL;
		goto err_release;
	}

	return obj;

err_release:
	drm_gem_object_release(obj);
	kfree(shadow);
	return ERR_PTR(ret);
}

/*
 * Copy a PRIME framebuffer into the contiguous CV200 scanout shadow.  The
 * shadow plane mapping is established by the atomic commit helpers and is
 * deliberately retained until the old plane state is cleaned up.
 */
static int histb_vdp_copy_shadow(struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_shadow_plane_state *shadow_state;
	struct drm_gem_object *obj;
	struct histb_vdp_shadow_object *shadow;
	size_t frame_size;
	int ret;

	if (!fb)
		return 0;

	obj = drm_gem_fb_get_obj(fb, 0);
	if (!obj)
		return -EINVAL;

	shadow = to_histb_vdp_shadow_object(obj);
	if (!shadow)
		return 0;

	shadow_state = to_drm_shadow_plane_state(state);
	if (iosys_map_is_null(&shadow_state->map[0]))
		return -EIO;

	frame_size = (size_t)fb->pitches[0] * fb->height;
	if (shadow->source_size > shadow->dma.base.size ||
	    frame_size > shadow->source_size ||
	    fb->offsets[0] > shadow->source_size - frame_size ||
	    fb->offsets[0] > shadow->dma.base.size - frame_size)
		return -EINVAL;

	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (ret)
		return ret;

	/* Keep the source and G0 destination at the framebuffer's byte offset. */
	iosys_map_memcpy_from((char *)shadow->dma.vaddr + fb->offsets[0],
			      &shadow_state->data[0], 0, frame_size);
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
	dma_wmb(); /* CPU copy complete before VDP DMA. */

	return 0;
}

static u32 histb_vdp_gfx_format(u32 format)
{
	switch (format) {
	case DRM_FORMAT_RGB565:
		return HISTB_VDP_GFX_RGB565;
	case DRM_FORMAT_XRGB8888:
		return HISTB_VDP_GFX_XRGB8888;
	case DRM_FORMAT_ARGB8888:
	default:
		return HISTB_VDP_GFX_ARGB8888;
	}
}

static bool
histb_vdp_plane_config_unchanged(const struct drm_plane_state *old_state,
				 const struct drm_plane_state *new_state)
{
	const struct drm_framebuffer *old_fb;
	const struct drm_framebuffer *new_fb;

	if (!old_state || !old_state->fb || !new_state->fb)
		return false;

	old_fb = old_state->fb;
	new_fb = new_state->fb;

	return old_fb->format->format == new_fb->format->format &&
	       old_fb->pitches[0] == new_fb->pitches[0] &&
	       old_fb->width == new_fb->width &&
	       old_fb->height == new_fb->height &&
	       old_state->src_x == new_state->src_x &&
	       old_state->src_y == new_state->src_y &&
	       old_state->src_w == new_state->src_w &&
	       old_state->src_h == new_state->src_h &&
	       old_state->crtc_x == new_state->crtc_x &&
	       old_state->crtc_y == new_state->crtc_y &&
	       old_state->crtc_w == new_state->crtc_w &&
	       old_state->crtc_h == new_state->crtc_h;
}

/*
 * G0 and GP0 have separate shadow latches, but the CV200 graphics path only
 * observes a complete pair.  Read back each request to flush the posted AXI
 * write before issuing the second request.
 */
static void histb_vdp_latch_gfx(struct histb_vdp *vdp)
{
	/* Keep the framebuffer writes ordered before both posted latch requests. */
	dma_wmb();
	writel(1, vdp->base + HISTB_VDP_G0_UPD);
	readl(vdp->base + HISTB_VDP_G0_UPD);
	writel(1, vdp->base + HISTB_VDP_GP0_UPD);
	readl(vdp->base + HISTB_VDP_GP0_UPD);
}

static int histb_vdp_program_plane(struct histb_vdp *vdp,
				   struct drm_plane_state *state,
				   const struct drm_plane_state *old_state)
{
	struct drm_framebuffer *fb = state->fb;
	dma_addr_t address;
	u32 format = histb_vdp_gfx_format(fb->format->format);
	u32 width = state->src_w >> 16;
	u32 height = state->src_h >> 16;
	u32 x = state->crtc_x;
	u32 y = state->crtc_y;
	u32 ctrl;
	int ret;

	ret = histb_vdp_copy_shadow(state);
	if (ret)
		return ret;

	address = drm_fb_dma_get_gem_addr(fb, state, 0);
	if (!histb_vdp_dma_address_valid(address)) {
		drm_err(&vdp->drm, "invalid G0 DMA address %pad\n", &address);
		return -EINVAL;
	}

	writel(lower_32_bits(address), vdp->base + HISTB_VDP_G0_ADDR);
	/*
	 * A same-configuration page flip only changes the GFX shadow address.
	 * Avoid restaging live format, geometry and fetch controls before the
	 * paired GFX/GP update requests.
	 */
	if (histb_vdp_plane_config_unchanged(old_state, state)) {
		histb_vdp_latch_gfx(vdp);
		return 0;
	}

	/* G0 is the scanout layer and uses AXI master 0 in the CV200 BSP. */
	writel(readl(vdp->base + HISTB_VDP_VOMASTERSEL) &
	       ~HISTB_VDP_VOMASTERSEL_G0,
	       vdp->base + HISTB_VDP_VOMASTERSEL);
	/* Keep all unused fetch paths deterministic after the firmware handoff. */
	writel(0, vdp->base + HISTB_VDP_G0_NADDR);
	writel(HISTB_VDP_G0_SMMU_BYPASS_2D |
	       HISTB_VDP_G0_SMMU_BYPASS_3D_READ,
	       vdp->base + HISTB_VDP_G0_SMMU_BYPASS0);
	writel(0, vdp->base + HISTB_VDP_G0_DCMP_ADDR);
	writel(0, vdp->base + HISTB_VDP_G0_DCMP_NADDR);
	writel(0, vdp->base + HISTB_VDP_G0_DCMP_OFFSET);
	writel(fb->pitches[0] / HISTB_VDP_G0_STRIDE_UNIT,
	       vdp->base + HISTB_VDP_G0_STRIDE);
	writel(HISTB_VDP_SIZE(width, height),
	       vdp->base + HISTB_VDP_G0_IRESO);
	writel(0, vdp->base + HISTB_VDP_G0_SFPOS);
	writel(HISTB_VDP_POS(x, y), vdp->base + HISTB_VDP_G0_DFPOS);
	writel(HISTB_VDP_POS(x + width - 1, y + height - 1),
	       vdp->base + HISTB_VDP_G0_DLPOS);
	writel(HISTB_VDP_POS(x, y), vdp->base + HISTB_VDP_G0_VFPOS);
	writel(HISTB_VDP_POS(x + width - 1, y + height - 1),
	       vdp->base + HISTB_VDP_G0_VLPOS);

	ctrl = FIELD_PREP(HISTB_VDP_G0_IFMT, format) |
	       FIELD_PREP(HISTB_VDP_G0_BITEXT, 3) |
	       FIELD_PREP(HISTB_VDP_G0_REQ_CTRL, 3) |
	       HISTB_VDP_G0_READ_PROGRESSIVE |
	       HISTB_VDP_G0_UPDATE_PROGRESSIVE |
	       HISTB_VDP_G0_NONSECURE |
	       HISTB_VDP_G0_ENABLE;
	ctrl &= ~HISTB_VDP_G0_OFL_MASTER;
	writel(ctrl, vdp->base + HISTB_VDP_G0_CTRL);
	writel(0xff | (fb->format->has_alpha ? HISTB_VDP_G0_PIXEL_ALPHA : 0),
	       vdp->base + HISTB_VDP_G0_CBMPARA);
	writel(0xff, vdp->base + HISTB_VDP_G0_ALPHA);
	histb_vdp_latch_gfx(vdp);

	return 0;
}

static void histb_vdp_program_axi(struct histb_vdp *vdp)
{
	u32 value;

	value = readl(vdp->base + HISTB_VDP_VOMIDORDER);
	value |= HISTB_VDP_VOMIDORDER_MID_ENABLE;
	writel(value, vdp->base + HISTB_VDP_VOMIDORDER);

	value = readl(vdp->base + HISTB_VDP_VOAXICTRL);
	value = (value & ~(HISTB_VDP_VOAXICTRL_MASK)) |
		HISTB_VDP_VOAXICTRL_VALUE;
	writel(value, vdp->base + HISTB_VDP_VOAXICTRL);

	value = readl(vdp->base + HISTB_VDP_VOAXICTRL1);
	value = (value & ~HISTB_VDP_VOAXICTRL1_FIELDS) |
		(HISTB_VDP_VOAXICTRL1_VALUE & HISTB_VDP_VOAXICTRL1_FIELDS);
	writel(value, vdp->base + HISTB_VDP_VOAXICTRL1);
}

static void histb_vdp_dhd0_stage(struct histb_vdp *vdp, u32 set, u32 clear)
{
	u32 ctrl;

	ctrl = readl(vdp->base + HISTB_VDP_DHD0_CTRL);
	ctrl &= ~clear;
	ctrl |= set | HISTB_VDP_DHD0_REGUP;
	writel(ctrl, vdp->base + HISTB_VDP_DHD0_CTRL);
}

static void histb_vdp_dhd0_update(struct histb_vdp *vdp)
{
	histb_vdp_dhd0_stage(vdp, 0, 0);
}

static void histb_vdp_program_rgb_identity_path(struct histb_vdp *vdp)
{
	unsigned int i;
	u32 value;

	/*
	 * The simple DRM pipe submits RGB. Clear optional color transforms,
	 * but keep the CV200 graphics precision conversion below active.
	 */
	writel(readl(vdp->base + HISTB_VDP_GDM_CTRL) &
	       ~HISTB_VDP_GDM_ENABLES,
	       vdp->base + HISTB_VDP_GDM_CTRL);
	writel(readl(vdp->base + HISTB_VDP_GDM_RGB2LMS_CTRL) &
	       ~HISTB_VDP_GDM_RGB2LMS_ENABLE,
	       vdp->base + HISTB_VDP_GDM_RGB2LMS_CTRL);
	writel(readl(vdp->base + HISTB_VDP_HDR_CTRL) &
	       ~HISTB_VDP_HDR_ENABLES,
	       vdp->base + HISTB_VDP_HDR_CTRL);
	writel(readl(vdp->base + HISTB_VDP_G0_CSC_IDC) &
	       ~HISTB_VDP_G0_CSC_ENABLE,
	       vdp->base + HISTB_VDP_G0_CSC_IDC);
	writel(readl(vdp->base + HISTB_VDP_GP0_CSC_IDC) &
	       ~HISTB_VDP_GP0_CSC_ENABLE,
	       vdp->base + HISTB_VDP_GP0_CSC_IDC);
	writel(readl(vdp->base + HISTB_VDP_DHD0_CSC_IDC) &
	       ~HISTB_VDP_DHD0_CSC_ENABLE,
	       vdp->base + HISTB_VDP_DHD0_CSC_IDC);

	/* Clear every CVM processing latch, but preserve output metadata bits. */
	value = readl(vdp->base + HISTB_VDP_CVM_CTRL);
	value &= ~HISTB_VDP_CVM_PIPE_ENABLES;
	writel(value, vdp->base + HISTB_VDP_CVM_CTRL);
	writel(readl(vdp->base + HISTB_VDP_CVM1_RGB2YUV_CTRL) & ~BIT(0),
	       vdp->base + HISTB_VDP_CVM1_RGB2YUV_CTRL);
	writel(readl(vdp->base + HISTB_VDP_CVM2_RGB2YUV_CTRL) &
	       ~HISTB_VDP_CVM2_RGB2YUV_ENABLE,
	       vdp->base + HISTB_VDP_CVM2_RGB2YUV_CTRL);
	/*
	 * CV200 HIFB uses the HDR/CVM2 block even for RGB-to-RGB CSC. Its
	 * Q10 identity coefficient is scaled by 2^(15-10) / 2, giving Q14
	 * unity. Bypassing CVM2 does not preserve the graphics sample width:
	 * adjacent pixel lanes overflow differently before the DHD mixer.
	 */
	for (i = 0; i < 9; i++)
		writel(i % 4 == 0 ? HISTB_VDP_CVM2_CSC_UNITY : 0,
		       vdp->base + HISTB_VDP_CVM2_RGB2YUV_COEF0 + i * 4);
	writel(HISTB_VDP_CVM2_CSC_PRECISION |
	       HISTB_VDP_CVM2_CSC_SCALE_ENABLE,
	       vdp->base + HISTB_VDP_CVM2_RGB2YUV_SCALE2P);
	writel(0, vdp->base + HISTB_VDP_CVM2_RGB2YUV_MIN);
	writel(0x7fff, vdp->base + HISTB_VDP_CVM2_RGB2YUV_MAX);
	writel(0, vdp->base + HISTB_VDP_CVM2_RGB2YUV_OUT_DC0);
	writel(0, vdp->base + HISTB_VDP_CVM2_RGB2YUV_OUT_DC1);
	writel(0, vdp->base + HISTB_VDP_CVM2_RGB2YUV_OUT_DC2);
	writel(value | HISTB_VDP_CVM_ENABLE,
	       vdp->base + HISTB_VDP_CVM_CTRL);
	writel(readl(vdp->base + HISTB_VDP_CVM2_RGB2YUV_CTRL) |
	       HISTB_VDP_CVM2_RGB2YUV_ENABLE,
	       vdp->base + HISTB_VDP_CVM2_RGB2YUV_CTRL);
	writel(HISTB_VDP_HDR_REGUP, vdp->base + HISTB_VDP_HDR_UPD);
}

static void histb_vdp_program_timing(struct histb_vdp *vdp,
				     const struct drm_display_mode *mode)
{
	u32 hfront = mode->hsync_start - mode->hdisplay;
	u32 hsync = mode->hsync_end - mode->hsync_start;
	u32 hback = mode->htotal - mode->hsync_end;
	u32 vfront = mode->vsync_start - mode->vdisplay;
	u32 vsync = mode->vsync_end - mode->vsync_start;
	u32 vback = mode->vtotal - mode->vsync_end;
	u32 sync_inv = HISTB_VDP_DATE_HSYNC_INV;
	u32 ctrl;

	/* CV200's blanking fields include the sync pulse width. */
	writel((mode->hdisplay - 1) | ((hback + hsync - 1) << 16),
	       vdp->base + HISTB_VDP_DHD0_HSYNC1);
	writel((hfront - 1), vdp->base + HISTB_VDP_DHD0_HSYNC2);
	writel((mode->vdisplay - 1) | ((vback + vsync - 1) << 12) |
	       ((vfront - 1) << 22), vdp->base + HISTB_VDP_DHD0_VSYNC);
	writel(0, vdp->base + HISTB_VDP_DHD0_VPLUS);
	writel((hsync - 1) | ((vsync - 1) << 16),
	       vdp->base + HISTB_VDP_DHD0_PWR);
	writel(0, vdp->base + HISTB_VDP_DHD0_VTTHD);

	/* Match the same-device DHD default while HDMI HS/VS stay non-inverted. */
	writel(sync_inv, vdp->base + HISTB_VDP_DHD0_SYNC_INV);

	/* DHD0 is source zero for the HDMI path. */
	writel(readl(vdp->base + HISTB_VDP_VO_MUX) & ~BIT(3),
	       vdp->base + HISTB_VDP_VO_MUX);
	writel(0, vdp->base + HISTB_VDP_DHD0_DATA_SEL);

	/* The DRM framebuffer is RGB; keep the VDP path an explicit identity. */
	histb_vdp_program_rgb_identity_path(vdp);
	writel(0, vdp->base + HISTB_VDP_DHD0_HDMI_CLIP_L);
	writel(HISTB_VDP_DHD0_CLIP_FULL,
	       vdp->base + HISTB_VDP_DHD0_HDMI_CLIP_H);

	/* Route G0 through GP0 and make GP0 the only DHD0 mixer input. */
	writel(1, vdp->base + HISTB_VDP_MIXG0_MIX);
	writel(0, vdp->base + HISTB_VDP_CBM_BKG1);
	writel(2, vdp->base + HISTB_VDP_CBM_MIX1);

	/*
	 * The simple pipe always scans a 1:1 framebuffer. Stock Linux PQ clears
	 * both scaler enables in this case while retaining the HIFB ZME -> CSC
	 * topology. Do this explicitly instead of inheriting Fastboot state.
	 */
	writel(readl(vdp->base + HISTB_VDP_GP0_ZME_HSP) &
	       ~HISTB_VDP_GP0_ZME_H_ENABLE,
	       vdp->base + HISTB_VDP_GP0_ZME_HSP);
	writel(readl(vdp->base + HISTB_VDP_GP0_ZME_VSP) &
	       ~HISTB_VDP_GP0_ZME_V_ENABLE,
	       vdp->base + HISTB_VDP_GP0_ZME_VSP);
	writel(HISTB_VDP_GP0_READ_PROGRESSIVE |
	       HISTB_VDP_GP0_ROUTE_ZME_CSC,
	       vdp->base + HISTB_VDP_GP0_CTRL);
	writel(HISTB_VDP_SIZE(mode->hdisplay, mode->vdisplay),
	       vdp->base + HISTB_VDP_GP0_IRESO);
	writel(HISTB_VDP_SIZE(mode->hdisplay, mode->vdisplay),
	       vdp->base + HISTB_VDP_GP0_ORESO);
	writel(0xff, vdp->base + HISTB_VDP_GP0_GALPHA);
	writel(0, vdp->base + HISTB_VDP_GP0_DFPOS);
	writel(HISTB_VDP_POS(mode->hdisplay - 1, mode->vdisplay - 1),
	       vdp->base + HISTB_VDP_GP0_DLPOS);
	writel(0, vdp->base + HISTB_VDP_GP0_VFPOS);
	writel(HISTB_VDP_POS(mode->hdisplay - 1, mode->vdisplay - 1),
	       vdp->base + HISTB_VDP_GP0_VLPOS);
	writel(1, vdp->base + HISTB_VDP_GP0_UPD);
	readl(vdp->base + HISTB_VDP_GP0_UPD);
	writel(HISTB_VDP_HDR_REGUP, vdp->base + HISTB_VDP_HDR_UPD);

	/* Stock stages HDMI mode before the separate interface-enable update. */
	ctrl = readl(vdp->base + HISTB_VDP_DHD0_CTRL);
	ctrl &= ~(HISTB_VDP_DHD0_ENABLE | HISTB_VDP_DHD0_CBAR_MASK);
	ctrl |= HISTB_VDP_DHD0_PROGRESSIVE | HISTB_VDP_DHD0_HDMI_MODE |
		HISTB_VDP_DHD0_TWOCHN_DEBUG;
	writel(ctrl | HISTB_VDP_DHD0_REGUP,
	       vdp->base + HISTB_VDP_DHD0_CTRL);
}

static int histb_vdp_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct histb_vdp *vdp = to_histb_vdp(pipe->crtc.dev);
	unsigned long flags;

	spin_lock_irqsave(&vdp->irq_lock, flags);
	vdp->vblank_enabled = true;
	if (vdp->powered)
		writel(readl(vdp->base + HISTB_VDP_VOINTMSK) |
		       HISTB_VDP_DHD0_VBLANK,
		       vdp->base + HISTB_VDP_VOINTMSK);
	spin_unlock_irqrestore(&vdp->irq_lock, flags);

	return 0;
}

static void histb_vdp_disable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct histb_vdp *vdp = to_histb_vdp(pipe->crtc.dev);
	unsigned long flags;

	spin_lock_irqsave(&vdp->irq_lock, flags);
	vdp->vblank_enabled = false;
	if (vdp->powered)
		writel(readl(vdp->base + HISTB_VDP_VOINTMSK) &
		       ~HISTB_VDP_DHD0_VBLANK,
		       vdp->base + HISTB_VDP_VOINTMSK);
	spin_unlock_irqrestore(&vdp->irq_lock, flags);
}

/* CV200's BSP exposes reset completion through the two VDPVERSION words. */
static int histb_vdp_wait_reset_finished(struct histb_vdp *vdp)
{
	u32 version1;

	return readl_poll_timeout(vdp->base + HISTB_VDP_VERSION1, version1,
				  !version1 &&
				  !readl(vdp->base + HISTB_VDP_VERSION2),
				  2000, 80000);
}

static int histb_vdp_resources_enable(struct histb_vdp *vdp)
{
	int ret;

	if (vdp->resources_enabled)
		return 0;

	ret = clk_bulk_prepare_enable(vdp->num_clks, vdp->clks);
	if (ret)
		return ret;

	/* Drop every display state inherited from the boot-logo pipeline. */
	ret = reset_control_assert(vdp->reset);
	if (ret) {
		clk_bulk_disable_unprepare(vdp->num_clks, vdp->clks);
		return ret;
	}
	/* Match the vendor reset-to-readback ordering around the reset edge. */
	mb();
	udelay(10);
	ret = histb_vdp_wait_reset_finished(vdp);
	if (ret) {
		drm_err(&vdp->drm,
			 "VDP reset did not complete before deassert: VDPVERSION1=%08x VDPVERSION2=%08x\n",
			 readl(vdp->base + HISTB_VDP_VERSION1),
			 readl(vdp->base + HISTB_VDP_VERSION2));
		clk_bulk_disable_unprepare(vdp->num_clks, vdp->clks);
		return ret;
	}

	ret = reset_control_deassert(vdp->reset);
	if (ret) {
		clk_bulk_disable_unprepare(vdp->num_clks, vdp->clks);
		return ret;
	}
	/* Publish the deassertion before touching the first VDP register. */
	mb();
	udelay(10);

	vdp->resources_enabled = true;
	/* CV200 VDP_DRIVER_Initial() programs the SRAM low-power controls. */
	writel(HISTB_VDP_VOLOWPOWER_DEFAULT,
	       vdp->base + HISTB_VDP_VOLOWPOWER_CTRL);
	return 0;
}

static void histb_vdp_resources_disable(struct histb_vdp *vdp)
{
	if (!vdp->resources_enabled)
		return;

	if (!reset_control_assert(vdp->reset)) {
		/* The BSP waits for both version words before removing the clocks. */
		mb();
		udelay(10);
		if (histb_vdp_wait_reset_finished(vdp))
			drm_warn(&vdp->drm,
				 "VDP reset completion timeout while disabling: VDPVERSION1=%08x VDPVERSION2=%08x\n",
				 readl(vdp->base + HISTB_VDP_VERSION1),
				 readl(vdp->base + HISTB_VDP_VERSION2));
	}
	clk_bulk_disable_unprepare(vdp->num_clks, vdp->clks);
	vdp->resources_enabled = false;
}

static void histb_vdp_pipe_enable(struct drm_simple_display_pipe *pipe,
				  struct drm_crtc_state *crtc_state,
				  struct drm_plane_state *plane_state)
{
	struct histb_vdp *vdp = to_histb_vdp(pipe->crtc.dev);
	unsigned long flags;
	int ret;

	ret = histb_vdp_resources_enable(vdp);
	if (ret) {
		drm_err(&vdp->drm, "failed to enable VDP resources: %pe\n",
			ERR_PTR(ret));
		return;
	}

	/*
	 * The DRM core may already have enabled the vblank interrupt before this
	 * CRTC enable runs (the first atomic commit arms its flip event first).
	 * Masking every source here would discard that enable and, because the
	 * core only calls enable_vblank on the 0->1 reference transition, the
	 * display controller would never raise a single vblank interrupt again:
	 * page flips then complete immediately with a zero timestamp.
	 * Keep the vblank source masked only while no user asked for it.
	 */
	writel(vdp->vblank_enabled ? HISTB_VDP_DHD0_VBLANK : 0,
	       vdp->base + HISTB_VDP_VOINTMSK);
	writel(~0, vdp->base + HISTB_VDP_VOMSKINTSTA);
	histb_vdp_program_axi(vdp);
	/*
	 * CV200's production VO HAL ignores the caller's value and hard-codes
	 * twochn_mode=1 together with DHD0 twochn_debug=1. Program that state
	 * before the DHD0 staging sequence, as done by the vendor driver.
	 */
	writel(readl(vdp->base + HISTB_VDP_VOCTRL) |
		HISTB_VDP_VOCTRL_CLK_GATE_EN |
		HISTB_VDP_VOCTRL_TWOCHN_EN |
		HISTB_VDP_VOCTRL_TWOCHN_MODE,
	       vdp->base + HISTB_VDP_VOCTRL);
	/* Match stock's disabled DHD0 default/update boundary. */
	histb_vdp_dhd0_stage(vdp,
			     HISTB_VDP_DHD0_PROGRESSIVE |
			     HISTB_VDP_DHD0_TWOCHN_DEBUG,
			     HISTB_VDP_DHD0_ENABLE |
			     HISTB_VDP_DHD0_HDMI_MODE |
			     HISTB_VDP_DHD0_P2I_ENABLE |
			     HISTB_VDP_DHD0_CBAR_MASK);
	histb_vdp_program_timing(vdp, &crtc_state->adjusted_mode);
	/* Interface enable is a separate update, followed by stock's second request. */
	histb_vdp_dhd0_stage(vdp,
			     HISTB_VDP_DHD0_ENABLE |
			     HISTB_VDP_DHD0_PROGRESSIVE |
			     HISTB_VDP_DHD0_HDMI_MODE |
			     HISTB_VDP_DHD0_TWOCHN_DEBUG,
			     HISTB_VDP_DHD0_CBAR_MASK);
	histb_vdp_dhd0_update(vdp);
	ret = histb_vdp_program_plane(vdp, plane_state, NULL);
	if (ret) {
		drm_err(&vdp->drm, "failed to program initial G0 plane: %pe\n",
			 ERR_PTR(ret));
		histb_vdp_resources_disable(vdp);
		return;
	}
	spin_lock_irqsave(&vdp->irq_lock, flags);
	vdp->powered = true;
	if (vdp->vblank_enabled)
		writel(HISTB_VDP_DHD0_VBLANK,
		       vdp->base + HISTB_VDP_VOINTMSK);
	spin_unlock_irqrestore(&vdp->irq_lock, flags);

	enable_irq(vdp->irq);

	/*
	 * The DRM core keeps vblank interrupts disabled (and refuses
	 * drm_crtc_vblank_get()) until the driver announces that the CRTC can
	 * serve them.  Announcing it late enough that our interrupt path is
	 * armed, but early enough that the first atomic commit can arm its flip
	 * event, is what makes page flips complete on a real vblank with a
	 * valid timestamp instead of being acknowledged immediately.
	 */
	drm_crtc_vblank_on(&pipe->crtc);
}

static void histb_vdp_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct histb_vdp *vdp = to_histb_vdp(pipe->crtc.dev);
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_pending_vblank_event *event;
	unsigned long flags;

	/* Stop the core from queueing vblank work before the pipe goes away. */
	drm_crtc_vblank_off(crtc);

	if (!vdp->powered)
		goto send_event;

	spin_lock_irqsave(&vdp->irq_lock, flags);
	writel(0, vdp->base + HISTB_VDP_VOINTMSK);
	writel(HISTB_VDP_DHD0_VBLANK,
	       vdp->base + HISTB_VDP_VOMSKINTSTA);
	writel(readl(vdp->base + HISTB_VDP_G0_CTRL) &
	       ~HISTB_VDP_G0_ENABLE, vdp->base + HISTB_VDP_G0_CTRL);
	/* G0 is latched through GP0 on the production graphics path. */
	histb_vdp_latch_gfx(vdp);
	writel((readl(vdp->base + HISTB_VDP_DHD0_CTRL) &
		~(HISTB_VDP_DHD0_ENABLE | HISTB_VDP_DHD0_CBAR_MASK)) |
	       HISTB_VDP_DHD0_REGUP,
	       vdp->base + HISTB_VDP_DHD0_CTRL);
	spin_unlock_irqrestore(&vdp->irq_lock, flags);

	disable_irq(vdp->irq);

	spin_lock_irqsave(&vdp->irq_lock, flags);
	vdp->powered = false;
	spin_unlock_irqrestore(&vdp->irq_lock, flags);

	/* Atomic disable is a channel transition, not a VDP block teardown. */

send_event:
	spin_lock_irqsave(&vdp->drm.event_lock, flags);
	event = crtc->state->event;
	if (event) {
		crtc->state->event = NULL;
		drm_crtc_send_vblank_event(crtc, event);
	}
	spin_unlock_irqrestore(&vdp->drm.event_lock, flags);
}

static enum drm_mode_status
histb_vdp_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
			  const struct drm_display_mode *mode)
{
	u32 hfront, hsync, hback;
	u32 vfront, vsync, vback;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;
	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;

	if (mode->hdisplay < HISTB_VDP_MIN_WIDTH ||
	    mode->hdisplay > HISTB_VDP_MAX_WIDTH)
		return MODE_BAD_HVALUE;
	if (mode->vdisplay < HISTB_VDP_MIN_HEIGHT ||
	    mode->vdisplay > HISTB_VDP_MAX_HEIGHT)
		return MODE_BAD_VVALUE;

	if (mode->hsync_start <= mode->hdisplay ||
	    mode->hsync_end <= mode->hsync_start ||
	    mode->htotal <= mode->hsync_end)
		return MODE_H_ILLEGAL;
	if (mode->vsync_start <= mode->vdisplay ||
	    mode->vsync_end <= mode->vsync_start ||
	    mode->vtotal <= mode->vsync_end)
		return MODE_V_ILLEGAL;

	hfront = mode->hsync_start - mode->hdisplay;
	hsync = mode->hsync_end - mode->hsync_start;
	hback = mode->htotal - mode->hsync_end;
	vfront = mode->vsync_start - mode->vdisplay;
	vsync = mode->vsync_end - mode->vsync_start;
	vback = mode->vtotal - mode->vsync_end;

	if (hfront > 0x10000 || hback > 0x10000)
		return MODE_BAD_HVALUE;
	if (hsync > 0x10000)
		return MODE_HSYNC_WIDE;
	if (vfront > 0x400 || vback > 0x400)
		return MODE_BAD_VVALUE;
	if (vsync > 0x100)
		return MODE_VSYNC_WIDE;

	return MODE_OK;
}

static int histb_vdp_pipe_check(struct drm_simple_display_pipe *pipe,
				struct drm_plane_state *plane_state,
				struct drm_crtc_state *crtc_state)
{
	const struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	struct drm_framebuffer *fb = plane_state->fb;
	dma_addr_t address;
	int ret;

	if (histb_vdp_pipe_mode_valid(pipe, mode) != MODE_OK)
		return -EINVAL;

	ret = drm_atomic_helper_check_plane_state(plane_state, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, true);
	if (ret)
		return ret;

	if (!fb)
		return 0;

	if (!IS_ALIGNED(fb->pitches[0], HISTB_VDP_G0_STRIDE_UNIT) ||
	    fb->pitches[0] / HISTB_VDP_G0_STRIDE_UNIT >
		FIELD_MAX(HISTB_VDP_G0_STRIDE_MASK))
		return -EINVAL;

	address = drm_fb_dma_get_gem_addr(fb, plane_state, 0);
	if (!histb_vdp_dma_address_valid(address))
		return -EINVAL;

	return 0;
}

static void histb_vdp_pipe_update(struct drm_simple_display_pipe *pipe,
				  struct drm_plane_state *old_state)
{
	struct histb_vdp *vdp = to_histb_vdp(pipe->crtc.dev);
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_pending_vblank_event *event = crtc->state->event;
	unsigned long flags;

	if (vdp->powered) {
		int ret = histb_vdp_program_plane(vdp, pipe->plane.state, old_state);

		if (ret)
			drm_err(&vdp->drm, "failed to update G0 plane: %pe\n",
				ERR_PTR(ret));
	}

	if (!event)
		return;

	crtc->state->event = NULL;
	spin_lock_irqsave(&vdp->drm.event_lock, flags);
	if (vdp->powered && crtc->state->active) {
		int vbret = drm_crtc_vblank_get(crtc);

		if (vbret == 0) {
			drm_crtc_arm_vblank_event(crtc, event);
		} else {
			drm_crtc_send_vblank_event(crtc, event);
		}
	} else {
		drm_crtc_send_vblank_event(crtc, event);
	}
	spin_unlock_irqrestore(&vdp->drm.event_lock, flags);
}

static const struct drm_simple_display_pipe_funcs histb_vdp_pipe_funcs = {
	.mode_valid = histb_vdp_pipe_mode_valid,
	.enable = histb_vdp_pipe_enable,
	.disable = histb_vdp_pipe_disable,
	.check = histb_vdp_pipe_check,
	.update = histb_vdp_pipe_update,
	.enable_vblank = histb_vdp_enable_vblank,
	.disable_vblank = histb_vdp_disable_vblank,
	DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS,
};

static struct drm_framebuffer *
histb_vdp_fb_create(struct drm_device *drm, struct drm_file *file,
		    const struct drm_mode_fb_cmd2 *mode_cmd)
{
	if (!IS_ALIGNED(mode_cmd->pitches[0], HISTB_VDP_G0_STRIDE_UNIT) ||
	    mode_cmd->pitches[0] / HISTB_VDP_G0_STRIDE_UNIT >
		FIELD_MAX(HISTB_VDP_G0_STRIDE_MASK) ||
	    !IS_ALIGNED(mode_cmd->offsets[0], HISTB_VDP_G0_STRIDE_UNIT)) {
		dev_err(drm->dev,
			"fb_create rejected: %p4cc %ux%u pitch=%u offset=%u modifier=0x%llx flags=0x%x\n",
			&mode_cmd->pixel_format, mode_cmd->width, mode_cmd->height,
			mode_cmd->pitches[0], mode_cmd->offsets[0],
			(unsigned long long)mode_cmd->modifier[0], mode_cmd->flags);
		return ERR_PTR(-EINVAL);
	}

	return drm_gem_fb_create(drm, file, mode_cmd);
}

static const struct drm_mode_config_funcs histb_vdp_mode_config_funcs = {
	.fb_create = histb_vdp_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static const struct drm_mode_config_helper_funcs histb_vdp_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail,
};

DEFINE_DRM_GEM_DMA_FOPS(histb_vdp_fops);

static int histb_vdp_dumb_create(struct drm_file *file,
				 struct drm_device *drm,
				 struct drm_mode_create_dumb *args)
{
	args->pitch = ALIGN(DIV_ROUND_UP(args->width * args->bpp, 8),
			    HISTB_VDP_G0_STRIDE_UNIT);

	return drm_gem_dma_dumb_create_internal(file, drm, args);
}

static const struct drm_driver histb_vdp_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &histb_vdp_fops,
	.dumb_create = histb_vdp_dumb_create,
	.gem_prime_import_sg_table = histb_vdp_prime_import_sg_table,
	.name = "histb-vdp",
	.desc = "HiSilicon HiSTB video display processor",
	.date = "20260801",
};

static irqreturn_t histb_vdp_irq(int irq, void *data)
{
	struct histb_vdp *vdp = data;
	u32 status;

	status = readl(vdp->base + HISTB_VDP_VOINTSTA);
	if (!(status & HISTB_VDP_DHD0_VBLANK))
		return IRQ_NONE;

	writel(HISTB_VDP_DHD0_VBLANK,
	       vdp->base + HISTB_VDP_VOMSKINTSTA);
	drm_crtc_handle_vblank(&vdp->pipe.crtc);

	return IRQ_HANDLED;
}

static int histb_vdp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_bridge *bridge;
	struct histb_vdp *vdp;
	struct drm_device *drm;
	int ret;

	vdp = devm_drm_dev_alloc(dev, &histb_vdp_driver,
				 struct histb_vdp, drm);
	if (IS_ERR(vdp))
		return PTR_ERR(vdp);

	drm = &vdp->drm;
	spin_lock_init(&vdp->irq_lock);

	vdp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vdp->base))
		return PTR_ERR(vdp->base);

	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, PTR_ERR(bridge),
				     "failed to find output bridge\n");

	vdp->clks[0].id = "bus";
	vdp->clks[1].id = "hd";
	vdp->clks[2].id = "config";
	vdp->clks[3].id = "core";
	vdp->clks[4].id = "video";
	vdp->clks[5].id = "graphics";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(vdp->clks) - 1, vdp->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get VDP clocks\n");
	vdp->num_clks = ARRAY_SIZE(vdp->clks) - 1;

	/* Older DTs omit the gate between the DHD timing engine and HDMI. */
	vdp->clks[6].id = "bp";
	vdp->clks[6].clk = devm_clk_get_optional(dev, "bp");
	if (IS_ERR(vdp->clks[6].clk))
		return dev_err_probe(dev, PTR_ERR(vdp->clks[6].clk),
				     "failed to get VDP output clock\n");
	if (vdp->clks[6].clk)
		vdp->num_clks++;
	else
		dev_dbg(dev, "DT omits the VO output gate; using legacy VDP clock set\n");

	vdp->reset = devm_reset_control_get_exclusive(dev, "vdp");
	if (IS_ERR(vdp->reset))
		return dev_err_probe(dev, PTR_ERR(vdp->reset),
				     "failed to get VDP reset\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = HISTB_VDP_MIN_WIDTH;
	drm->mode_config.min_height = HISTB_VDP_MIN_HEIGHT;
	drm->mode_config.max_width = HISTB_VDP_MAX_WIDTH;
	drm->mode_config.max_height = HISTB_VDP_MAX_HEIGHT;
	drm->mode_config.preferred_depth = 32;
	drm->mode_config.funcs = &histb_vdp_mode_config_funcs;
	drm->mode_config.helper_private = &histb_vdp_mode_config_helpers;

	ret = drm_simple_display_pipe_init(drm, &vdp->pipe,
					   &histb_vdp_pipe_funcs,
					   histb_vdp_formats,
					   ARRAY_SIZE(histb_vdp_formats),
					   NULL, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "failed to create display pipe\n");

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize vblank\n");

	ret = drm_bridge_attach(&vdp->pipe.encoder, bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return dev_err_probe(dev, ret, "failed to attach bridge\n");

	vdp->connector = drm_bridge_connector_init(drm, &vdp->pipe.encoder);
	if (IS_ERR(vdp->connector))
		return dev_err_probe(dev, PTR_ERR(vdp->connector),
				     "failed to create connector\n");

	ret = drm_connector_attach_encoder(vdp->connector, &vdp->pipe.encoder);
	if (ret)
		return ret;

	vdp->irq = platform_get_irq(pdev, 0);
	if (vdp->irq < 0)
		return vdp->irq;

	ret = devm_request_irq(dev, vdp->irq, histb_vdp_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), vdp);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request VDP IRQ\n");

	drm_mode_config_reset(drm);
	ret = drmm_kms_helper_poll_init(drm);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to initialize connector HPD handling\n");
	platform_set_drvdata(pdev, drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_fbdev_dma_setup(drm, 32);

	return 0;
}

static void histb_vdp_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	struct histb_vdp *vdp = to_histb_vdp(drm);

	drm_dev_unregister(drm);
	drm_atomic_helper_shutdown(drm);
	histb_vdp_resources_disable(vdp);
}

static void histb_vdp_shutdown(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	struct histb_vdp *vdp = to_histb_vdp(drm);

	drm_atomic_helper_shutdown(drm);
	histb_vdp_resources_disable(vdp);
}

static int histb_vdp_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct histb_vdp *vdp = to_histb_vdp(drm);
	int ret;

	ret = drm_mode_config_helper_suspend(drm);
	if (!ret)
		histb_vdp_resources_disable(vdp);

	return ret;
}

static int histb_vdp_resume(struct device *dev)
{
	return drm_mode_config_helper_resume(dev_get_drvdata(dev));
}

static DEFINE_SIMPLE_DEV_PM_OPS(histb_vdp_pm_ops,
				histb_vdp_suspend, histb_vdp_resume);

static const struct of_device_id histb_vdp_of_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-vdp" },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_vdp_of_match);

static struct platform_driver histb_vdp_platform_driver = {
	.probe = histb_vdp_probe,
	.remove_new = histb_vdp_remove,
	.shutdown = histb_vdp_shutdown,
	.driver = {
		.name = "histb-vdp",
		.pm = pm_sleep_ptr(&histb_vdp_pm_ops),
		.of_match_table = histb_vdp_of_match,
	},
};
module_platform_driver(histb_vdp_platform_driver);

MODULE_AUTHOR("HiSilicon Technologies Co., Ltd.");
MODULE_DESCRIPTION("HiSilicon Hi3798CV200 VDP DRM driver");
MODULE_IMPORT_NS(DMA_BUF);
MODULE_LICENSE("GPL");
