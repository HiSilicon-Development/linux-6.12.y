// SPDX-License-Identifier: GPL-2.0
#ifndef __HISTB_HDMI_HDCP_H
#define __HISTB_HDMI_HDCP_H

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <drm/display/drm_hdcp.h>

struct device;
struct drm_connector;
struct regmap;

/* CV200 imagev200 only. Callers serialize engine and DDC register access. */
struct histb_hdcp_io {
	struct device *dev;
	void *ctx;
	u8 (*read)(void *ctx, u32 reg);
	void (*write)(void *ctx, u32 reg, u8 value);
	struct regmap *peri;
	void __iomem *cipher;
	void __iomem *pram;
	void __iomem *avcttpt;
};

/* Internal state is private to each protocol implementation. */
struct histb_hdcp14;
struct histb_hdcp22;

struct histb_hdcp14 *histb_hdcp14_create(struct histb_hdcp_io *io);
int histb_hdcp14_start(struct histb_hdcp14 *hdcp);
/* 1: verified authentication, 0: still authenticating, negative: failed. */
int histb_hdcp14_poll(struct histb_hdcp14 *hdcp, struct drm_connector *connector);
void histb_hdcp14_stop(struct histb_hdcp14 *hdcp);

struct histb_hdcp22 *histb_hdcp22_create(struct histb_hdcp_io *io);
int histb_hdcp22_start(struct histb_hdcp22 *hdcp);
int histb_hdcp22_poll(struct histb_hdcp22 *hdcp, struct drm_connector *connector);
void histb_hdcp22_stop(struct histb_hdcp22 *hdcp);

/* Validate every variable-length record before the generic DRM parser sees
 * it. SRM signatures remain within the kernel firmware trust boundary, as
 * in drm_hdcp_helper; this is structural validation, not a DCP signature
 * verifier. Supply an authentic, administratively trusted DCP SRM.
 * The generic DRM helper checks the first generation only: refuse trailing
 * generation records rather than authenticating with an incomplete list.
 */
static inline int histb_hdcp_validate_srm(const u8 *data, size_t size)
{
	size_t header = sizeof(struct hdcp_srm_header), length, payload, pos;
	unsigned int signature, count;
	bool hdcp2;

	if (size < header + 3 || data[1])
		return -EINVAL;
	if (data[0] == (DRM_HDCP_1_4_SRM_ID << 4))
		hdcp2 = false;
	else if (data[0] == ((DRM_HDCP_2_SRM_ID << 4) | DRM_HDCP_2_INDICATOR))
		hdcp2 = true;
	else
		return -EINVAL;
	signature = hdcp2 ? DRM_HDCP_2_DCP_SIG_SIZE : DRM_HDCP_1_4_DCP_SIG_SIZE;
	length = ((size_t)data[header] << 16) |
		 ((size_t)data[header + 1] << 8) | data[header + 2];
	if (length != size - header || length < 3 + signature)
		return -EINVAL;
	payload = length - 3 - signature;
	data += header + 3;
	if (hdcp2) {
		if (payload < DRM_HDCP_2_NO_OF_DEV_PLUS_RESERVED_SZ ||
		    (data[1] & 0x3f) || data[2] || data[3])
			return -EINVAL;
		count = (data[0] << 2) | DRM_HDCP_2_KSV_COUNT_2_LSBITS(data[1]);
		return payload == DRM_HDCP_2_NO_OF_DEV_PLUS_RESERVED_SZ +
			count * DRM_HDCP_KSV_LEN ? 0 : -EINVAL;
	}
	/* HDCP 1.4 Table A-23 has no VRL payload in an empty SRM. */
	for (pos = 0; pos < payload;) {
		count = data[pos++];
		if ((count & 0x80) || count * DRM_HDCP_KSV_LEN > payload - pos)
			return -EINVAL;
		pos += count * DRM_HDCP_KSV_LEN;
	}
	return 0;
}

/* Check the exact validated firmware snapshot; the generic helper re-requests
 * the file and treats a missing replacement as an empty revocation list.
 */
static inline int histb_hdcp_check_revocation(const u8 *data, size_t size,
					    const u8 *ids, unsigned int id_count)
{
	size_t header = sizeof(struct hdcp_srm_header), payload, pos = 0;
	unsigned int signature, count, i, j;
	bool hdcp2;
	int ret = histb_hdcp_validate_srm(data, size);

	if (ret)
		return ret;
	hdcp2 = data[0] == ((DRM_HDCP_2_SRM_ID << 4) | DRM_HDCP_2_INDICATOR);
	signature = hdcp2 ? DRM_HDCP_2_DCP_SIG_SIZE : DRM_HDCP_1_4_DCP_SIG_SIZE;
	payload = size - header - 3 - signature;
	if (!payload)
		return 0;
	data += header + 3;
	if (hdcp2) {
		count = (data[0] << 2) | DRM_HDCP_2_KSV_COUNT_2_LSBITS(data[1]);
		pos = DRM_HDCP_2_NO_OF_DEV_PLUS_RESERVED_SZ;
	} else {
		count = data[pos++];
	}
	for (;;) {
		for (i = 0; i < count; i++)
			for (j = 0; j < id_count; j++)
				if (!memcmp(data + pos + i * DRM_HDCP_KSV_LEN,
					    ids + j * DRM_HDCP_KSV_LEN, DRM_HDCP_KSV_LEN))
					return -EKEYREJECTED;
		pos += count * DRM_HDCP_KSV_LEN;
		if (hdcp2 || pos == payload)
			break;
		count = data[pos++];
	}
	return 0;
}

#endif
