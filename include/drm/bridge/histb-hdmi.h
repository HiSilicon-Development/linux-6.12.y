/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __DRM_BRIDGE_HISTB_HDMI_H__
#define __DRM_BRIDGE_HISTB_HDMI_H__

struct drm_bridge;
struct drm_connector;

int histb_hdmi_connector_init(struct drm_bridge *bridge,
			      struct drm_connector *connector);

#endif
