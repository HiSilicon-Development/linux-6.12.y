/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_HISTB_VPSS_H
#define _UAPI_LINUX_HISTB_VPSS_H

#include <linux/types.h>
#include <linux/videodev2.h>

#define HISTB_VPSS_DEI_TOP_FIELD_FIRST 1U
#define HISTB_VPSS_DEI_MCDI_EN         2U
#define HISTB_VPSS_DEI_TNR_EN          4U

/*
 * Four dequeued, driver-owned MMAP NV12 capture surfaces, in field order:
 * LAST, CUR (the output field), NEXT1, NEXT2. bottom_mask identifies each
 * field's parity. The destination is a distinct, dequeued capture surface.
 * Userspace must retain every source/destination buffer until ioctl returns.
 */
struct histb_vpss_dei_request {
	__u32 source[4];
	__u32 bottom_mask;
	__u32 destination;
	__u32 target_width;
	__u32 target_height;
	__u32 flags;
};

#define VIDIOC_HISTB_VPSS_DEI \
	_IOW('V', BASE_VIDIOC_PRIVATE, struct histb_vpss_dei_request)

#endif
