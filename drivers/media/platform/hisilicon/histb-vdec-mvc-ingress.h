/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HISTB_VDEC_MVC_INGRESS_H
#define HISTB_VDEC_MVC_INGRESS_H

#include <linux/v4l2-controls.h>
#include "histb-vdec-mvc.h"

/* Internal request frontend, not a private userspace ABI. */
struct histb_mvc_slice_identity {
	struct histb_mvc_view view;
	u32 first_mb;
	u32 slice_type;
	u32 pps_id;
	u32 header_bytes;
	u8 nal_ref_idc;
};

struct histb_mvc_surface {
	void *buffer;
	u64 timestamp;
	u64 serial;
	s32 poc;
	u32 frame_num;
	s32 apc_slot;
	u16 view_id;
	u8 view_index;
	u8 temporal;
	u8 interview;
	u8 decoding;
	u8 output_pending;
	u8 consumer;
};

struct histb_mvc_ingress {
	struct histb_mvc_subsps subset;
	struct histb_mvc_surface surface[HISTB_MVC_MAX_FRAME_STORE];
	struct histb_mvc_slice_identity picture;
	u64 references;
	u64 retain;
	int active;
	int base;
	u64 serial;
	bool draining;
};

void histb_mvc_ingress_init(struct histb_mvc_ingress *in);
/* NAL includes its one-byte header, excludes Annex-B start code. Scratch
 * is caller-owned and at least bytes long. Parameter changes require an
 * empty decoder (including capture buffers still held by the consumer).
 */
int histb_mvc_ingress_subsps(struct histb_mvc_ingress *in,
		const u8 *nal, u32 bytes, u8 *scratch, u32 scratch_bytes);
int histb_mvc_ingress_identify(const struct histb_mvc_ingress *in,
		const u8 *nal, u32 bytes, struct histb_mvc_slice_identity *id);
/* Locate one VCL NAL; consume parameter prefixes. Returned coordinates
 * exclude the start code, matching H264 header_bit_size semantics. */
u32 histb_mvc_ingress_scratch_size(const u8 *data, u32 bytes, bool annex_b);
int histb_mvc_ingress_packet(struct histb_mvc_ingress *in,
		const u8 *data, u32 bytes, bool annex_b,
		u8 *scratch, u32 scratch_bytes, u32 *offset, u32 *length,
		struct histb_mvc_slice_identity *id);
/* Drop obsolete current-view temporal owners. A requeued decode target also
 * surrenders its old temporal owner when absent from this request's DPB.
 * Inter-view, output, consumer and active reference pins remain unchanged. */
int histb_mvc_ingress_prepare(struct histb_mvc_ingress *in,
		const struct histb_mvc_slice_identity *id,
		const struct v4l2_ctrl_h264_decode_params *decode, void *buffer);
/* One view picture per transaction. Existing standard H264 request controls
 * contain final Annex-G lists, including inter-view entries. Decode DPB
 * timestamps are globally unique across both views; no new control is used.
 */
int histb_mvc_ingress_begin(struct histb_mvc_ingress *in,
		const struct histb_mvc_slice_identity *id,
		const struct v4l2_ctrl_h264_sps *sps,
		const struct v4l2_ctrl_h264_pps *pps,
		const struct v4l2_ctrl_h264_decode_params *decode,
		const struct v4l2_ctrl_h264_slice_params *slice,
		void *buffer, u64 timestamp, s32 apc_slot);
/* Validate every continuation slice before the existing H264 packer. */
int histb_mvc_ingress_slice(const struct histb_mvc_ingress *in,
		const struct histb_mvc_slice_identity *id,
		const struct v4l2_ctrl_h264_decode_params *decode,
		const struct v4l2_ctrl_h264_slice_params *slice);
void histb_mvc_ingress_complete(struct histb_mvc_ingress *in, bool success);
/* Output is decode order per view; a stateless consumer performs display
 * reordering. Return slot or -EAGAIN. buffer/timestamp/view_id stay valid
 * until capture QBUF calls requeue. Completing output is NOT releasing DPB.
 */
int histb_mvc_ingress_output(struct histb_mvc_ingress *in, unsigned int view);
int histb_mvc_ingress_requeue(struct histb_mvc_ingress *in, void *buffer);
bool histb_mvc_ingress_busy(const struct histb_mvc_ingress *in, void *buffer);
/* Core removes its APC and drops its decoded-buffer reference before retire.
 * Retire returns -EBUSY until temporal/inter-view/hardware/output/user owners
 * are all gone. It does not free the core's opaque buffer itself.
 */
int histb_mvc_ingress_retire(struct histb_mvc_ingress *in, void *buffer);
int histb_mvc_ingress_drain(struct histb_mvc_ingress *in);
bool histb_mvc_ingress_drained(const struct histb_mvc_ingress *in);
#endif
