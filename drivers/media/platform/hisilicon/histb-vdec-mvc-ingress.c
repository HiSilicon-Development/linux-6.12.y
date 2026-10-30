// SPDX-License-Identifier: GPL-2.0-only
/* MV310 two-view H264 request ingress. Native authority and integration:
 * out/ec6110t/20261026-mv310-codecs/mvc-ingress-contract.txt.
 * Syntax identity is read from encoded NALs, never guessed from profile_idc.
 * Ordinary stateless controls remain the authoritative full H264 syntax and
 * final ordered reference lists; this layer does not implement a stateful
 * H264_MVC format or manufacture missing controls.
 */
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include "histb-vdec-mvc-ingress.h"

/* Read the small slice identity directly from EBSP without copying a slice. */
struct ingress_bits {
	const u8 *data;
	u32 bytes;
	u32 pos;
	u8 byte;
	u8 left;
	u8 zeros;
	bool error;
};

static u32 ingress_bit(struct ingress_bits *b)
{
	if (!b->left) {
		if (b->pos >= b->bytes) {
			b->error = true;
			return 0;
		}
		b->byte = b->data[b->pos++];
		if (b->zeros == 2 && b->byte == 3) {
			if (b->pos >= b->bytes || b->data[b->pos] > 3) {
				b->error = true;
				return 0;
			}
			b->byte = b->data[b->pos++];
			b->zeros = 0;
		}
		b->zeros = b->byte ? 0 : min_t(u8, b->zeros + 1, 2);
		b->left = 8;
	}
	return (b->byte >> --b->left) & 1;
}

static u32 ingress_ue(struct ingress_bits *b)
{
	u32 n = 0, v = 1;

	while (!b->error && !ingress_bit(b)) {
		if (++n == 32) {
			b->error = true;
			return 0;
		}
	}
	while (n-- && !b->error)
		v = (v << 1) | ingress_bit(b);
	return v - 1;
}

void histb_mvc_ingress_init(struct histb_mvc_ingress *in)
{
	memset(in, 0, sizeof(*in));
	in->active = -1;
	in->base = -1;
}

int histb_mvc_ingress_subsps(struct histb_mvc_ingress *in,
		const u8 *nal, u32 bytes, u8 *scratch, u32 scratch_bytes)
{
	struct histb_mvc_subsps subset;
	u32 i, n = 0, zeros = 0, list;
	int ret;

	if (!in || !nal || !scratch || bytes < 2 || scratch_bytes < bytes ||
	    (nal[0] & 0x9f) != HISTB_MVC_NAL_TYPE_SUBSPS)
		return -EINVAL;
	for (i = 1; i < bytes; i++) {
		if (zeros == 2 && nal[i] == 3) {
			if (i + 1 == bytes || nal[i + 1] > 3)
				return -EINVAL;
			zeros = 0;
			continue;
		}
		scratch[n++] = nal[i];
		zeros = nal[i] ? 0 : min(zeros + 1, 2U);
	}
	ret = histb_mvc_parse_subsps(&subset, scratch, n);
	if (ret != HISTB_MVC_OK)
		return ret == HISTB_MVC_UNSUPPORTED ? -EOPNOTSUPP : -EINVAL;
	if (subset.num_views_minus1 != 1)
		return -EOPNOTSUPP;
	for (list = 0; list < 2; list++) {
		if (subset.num_anchor_refs[list][1] > 1 ||
		    subset.num_non_anchor_refs[list][1] > 1 ||
		    (subset.num_anchor_refs[list][1] &&
		     subset.anchor_ref[list][1][0] != subset.view_id[0]) ||
		    (subset.num_non_anchor_refs[list][1] &&
		     subset.non_anchor_ref[list][1][0] != subset.view_id[0]))
			return -EINVAL;
	}
	if (in->subset.is_valid && !memcmp(&in->subset, &subset, sizeof(subset)))
		return 0;
	for (i = 0; i < ARRAY_SIZE(in->surface); i++)
		if (in->surface[i].buffer)
			return -EBUSY;
	in->subset = subset;
	return 0;
}

int histb_mvc_ingress_identify(const struct histb_mvc_ingress *in,
		const u8 *nal, u32 bytes, struct histb_mvc_slice_identity *id)
{
	struct histb_mvc_nal_ext ext;
	struct ingress_bits bits = {};
	u32 type;
	int ret;

	if (!in || !in->subset.is_valid || !nal || bytes < 2 || !id ||
	    (nal[0] & 0x80))
		return -EINVAL;
	memset(id, 0, sizeof(*id));
	type = nal[0] & 31;
	id->nal_ref_idc = (nal[0] >> 5) & 3;
	id->header_bytes = 1;
	if (type == HISTB_MVC_NAL_TYPE_SLICEEXT) {
		ret = histb_mvc_parse_mvc_extension(&ext, nal + 1, bytes - 1);
		if (ret != HISTB_MVC_OK || !ext.reserved_one_bit)
			return ret == HISTB_MVC_UNSUPPORTED ? -EOPNOTSUPP : -EINVAL;
		id->header_bytes = 4;
		id->view.view_id = ext.view_id;
		id->view.anchor_pic_flag = ext.anchor_pic_flag;
		id->view.inter_view_flag = ext.inter_view_flag;
		type = ext.non_idr_flag ? HISTB_MVC_NAL_TYPE_SLICE : HISTB_MVC_NAL_TYPE_IDR;
	} else if (type == HISTB_MVC_NAL_TYPE_SLICE || type == HISTB_MVC_NAL_TYPE_IDR) {
		id->view.view_id = in->subset.view_id[0];
		id->view.inter_view_flag = 1;
		id->view.anchor_pic_flag = type == HISTB_MVC_NAL_TYPE_IDR;
	} else {
		return -ENODATA;
	}
	if (id->view.view_id == in->subset.view_id[0])
		id->view.voidx = 0;
	else if (id->view.view_id == in->subset.view_id[1])
		id->view.voidx = 1;
	else
		return -EINVAL;
	id->view.nal_unit_type = type;
	bits.data = nal + id->header_bytes;
	bits.bytes = bytes - id->header_bytes;
	id->first_mb = ingress_ue(&bits);
	id->slice_type = ingress_ue(&bits);
	id->pps_id = ingress_ue(&bits);
	if (bits.error || id->slice_type > 9 || id->pps_id > 255)
		return -EINVAL;
	id->slice_type %= 5;
	return 0;
}

static u32 ingress_start_code(const u8 *data, u32 bytes, u32 from)
{
	u32 i;

	if (bytes < 3)
		return bytes;
	for (i = from; i < bytes - 2; i++)
		if (!data[i] && !data[i + 1] && data[i + 2] == 1)
			return i;
	return bytes;
}

u32 histb_mvc_ingress_scratch_size(const u8 *data, u32 bytes, bool annex_b)
{
	u32 pos, needed = 0;

	if (!data || !annex_b)
		return 0;
	pos = ingress_start_code(data, bytes, 0);
	while (pos < bytes) {
		u32 start = pos + 3;
		u32 next = ingress_start_code(data, bytes, start);

		if (start < bytes && (data[start] & 31) == HISTB_MVC_NAL_TYPE_SUBSPS)
			needed = max(needed, next - start);
		pos = next;
	}
	return needed;
}

int histb_mvc_ingress_packet(struct histb_mvc_ingress *in,
		const u8 *data, u32 bytes, bool annex_b,
		u8 *scratch, u32 scratch_bytes, u32 *offset, u32 *length,
		struct histb_mvc_slice_identity *id)
{
	u32 pos;
	bool found = false;
	int ret;

	if (!in || !data || !bytes || !offset || !length || !id)
		return -EINVAL;
	if (!annex_b) {
		*offset = 0;
		*length = bytes;
		if (!in->subset.is_valid && (data[0] & 31) != HISTB_MVC_NAL_TYPE_SLICEEXT) {
			memset(id, 0, sizeof(*id));
			return 0;
		}
		return histb_mvc_ingress_identify(in, data, bytes, id);
	}
	pos = ingress_start_code(data, bytes, 0);
	while (pos < bytes) {
		u32 start = pos + 3;
		u32 next = ingress_start_code(data, bytes, start);
		u32 end = next, type;

		while (end > start && !data[end - 1])
			end--;
		if (start >= end)
			return -EINVAL;
		type = data[start] & 31;
		if (type == HISTB_MVC_NAL_TYPE_SUBSPS) {
			ret = histb_mvc_ingress_subsps(in, data + start, end - start,
						     scratch, scratch_bytes);
			if (ret)
				return ret;
		} else if (type == HISTB_MVC_NAL_TYPE_SLICE ||
			   type == HISTB_MVC_NAL_TYPE_IDR ||
			   type == HISTB_MVC_NAL_TYPE_SLICEEXT) {
			if (found)
				return -EINVAL;
			memset(id, 0, sizeof(*id));
			if (in->subset.is_valid || type == HISTB_MVC_NAL_TYPE_SLICEEXT) {
				ret = histb_mvc_ingress_identify(in, data + start, end - start, id);
				if (ret)
					return ret;
			}
			*offset = start;
			*length = end - start;
			found = true;
		}
		pos = next;
	}
	return found ? 0 : -ENODATA;
}

static int ingress_find(const struct histb_mvc_ingress *in, u64 timestamp)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(in->surface); i++)
		if (in->surface[i].buffer && in->surface[i].timestamp == timestamp)
			return i;
	return -ENOENT;
}

static bool ingress_cross_allowed(const struct histb_mvc_ingress *in,
		const struct histb_mvc_slice_identity *id, unsigned int list)
{
	return id->view.voidx && (id->view.anchor_pic_flag ?
		in->subset.num_anchor_refs[list][1] :
		in->subset.num_non_anchor_refs[list][1]);
}

static int ingress_lists(const struct histb_mvc_ingress *in,
		const struct histb_mvc_slice_identity *id,
		const struct v4l2_ctrl_h264_decode_params *decode,
		const struct v4l2_ctrl_h264_slice_params *slice)
{
	unsigned int list, i, n;

	if (slice->first_mb_in_slice != id->first_mb || slice->slice_type != id->slice_type)
		return -EINVAL;
	for (list = 0; list < 2; list++) {
		const struct v4l2_h264_reference *refs = list ?
			slice->ref_pic_list1 : slice->ref_pic_list0;

		n = id->slice_type == V4L2_H264_SLICE_TYPE_B ?
			(list ? slice->num_ref_idx_l1_active_minus1 :
			 slice->num_ref_idx_l0_active_minus1) + 1 :
			id->slice_type == V4L2_H264_SLICE_TYPE_P && !list ?
			slice->num_ref_idx_l0_active_minus1 + 1 : 0;
		if (n > ARRAY_SIZE(slice->ref_pic_list0))
			return -E2BIG;
		for (i = 0; i < n; i++) {
			const struct v4l2_h264_dpb_entry *dpb;
			const struct histb_mvc_surface *s;
			int slot;

			if (refs[i].index >= ARRAY_SIZE(decode->dpb))
				return -EINVAL;
			dpb = &decode->dpb[refs[i].index];
			if (!(dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE))
				return -EINVAL;
			slot = ingress_find(in, dpb->reference_ts);
			if (slot < 0)
				return slot;
			s = &in->surface[slot];
			if (s->decoding)
				return -EBUSY;
			if (s->view_index == id->view.voidx) {
				if (!s->temporal)
					return -EINVAL;
			} else if (slot != in->base || !s->interview ||
				   !ingress_cross_allowed(in, id, list) ||
				   s->frame_num != decode->frame_num) {
				return -EINVAL;
			}
		}
	}
	return 0;
}

int histb_mvc_ingress_prepare(struct histb_mvc_ingress *in,
		const struct histb_mvc_slice_identity *id,
		const struct v4l2_ctrl_h264_decode_params *decode, void *buffer)
{
	unsigned int i, j;

	if (!in || !id || !decode || !buffer || !in->subset.is_valid ||
	    in->active >= 0 || in->draining || id->view.voidx > 1 ||
	    id->view.view_id != in->subset.view_id[id->view.voidx])
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(in->surface); i++) {
		struct histb_mvc_surface *s = &in->surface[i];

		if (!s->buffer || !s->temporal ||
		    (s->view_index != id->view.voidx && s->buffer != buffer))
			continue;
		for (j = 0; j < ARRAY_SIZE(decode->dpb); j++)
			if ((decode->dpb[j].flags & V4L2_H264_DPB_ENTRY_FLAG_VALID) &&
			    decode->dpb[j].reference_ts == s->timestamp)
				break;
		if (j == ARRAY_SIZE(decode->dpb))
			s->temporal = 0;
	}
	return 0;
}

int histb_mvc_ingress_begin(struct histb_mvc_ingress *in,
		const struct histb_mvc_slice_identity *id,
		const struct v4l2_ctrl_h264_sps *sps,
		const struct v4l2_ctrl_h264_pps *pps,
		const struct v4l2_ctrl_h264_decode_params *decode,
		const struct v4l2_ctrl_h264_slice_params *slice,
		void *buffer, u64 timestamp, s32 apc_slot)
{
	u64 retain = 0, references = 0;
	unsigned int i;
	int free = -1, ret;
	bool idr;

	if (!in || !id || !sps || !pps || !decode || !slice)
		return -EINVAL;
	idr = id->view.nal_unit_type == HISTB_MVC_NAL_TYPE_IDR;

	if (in->draining || in->active >= 0)
		return -EBUSY;
	if (!buffer || apc_slot < 0 || !in->subset.is_valid ||
	    id->view.voidx > 1 || id->view.view_id != in->subset.view_id[id->view.voidx] ||
	    id->pps_id != pps->pic_parameter_set_id ||
	    pps->seq_parameter_set_id != sps->seq_parameter_set_id ||
	    (id->view.voidx && sps->seq_parameter_set_id != in->subset.sps.seq_parameter_set_id) ||
	    idr != !!(decode->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) ||
	    id->nal_ref_idc != decode->nal_ref_idc)
		return -EINVAL;
	/* Field pairing remains the ordinary core's responsibility. This ingress
	 * currently accepts complete frame pictures only, never aliases views.
	 */
	if (decode->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC)
		return -EOPNOTSUPP;
	if (idr && id->view.voidx == 0 && in->base >= 0)
		return -EBUSY;
	if (!id->view.voidx && in->base >= 0)
		return -EPIPE; /* Missing dependent picture: never silently replace it. */
	for (i = 0; i < ARRAY_SIZE(in->surface); i++) {
		const struct histb_mvc_surface *s = &in->surface[i];

		if (s->buffer && (s->buffer == buffer || s->timestamp == timestamp ||
				  s->apc_slot == apc_slot))
			return -EBUSY;
		if (!s->buffer && free < 0)
			free = i;
	}
	if (free < 0)
		return -ENOSPC;
	for (i = 0; i < ARRAY_SIZE(decode->dpb); i++) {
		const struct v4l2_h264_dpb_entry *dpb = &decode->dpb[i];
		int slot;

		if (!(dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID))
			continue;
		slot = ingress_find(in, dpb->reference_ts);
		if (slot < 0)
			return slot;
		if (in->surface[slot].decoding)
			return -EBUSY;
		if (in->surface[slot].view_index == id->view.voidx) {
			if (!in->surface[slot].temporal || idr)
				return -EINVAL;
			retain |= BIT_ULL(slot);
		}
		references |= BIT_ULL(slot);
	}
	if (id->view.voidx && (in->base < 0 ||
	    in->surface[in->base].frame_num != decode->frame_num))
		return -EPIPE;
	ret = ingress_lists(in, id, decode, slice);
	if (ret)
		return ret;
	in->surface[free] = (struct histb_mvc_surface) {
		.buffer = buffer, .timestamp = timestamp, .apc_slot = apc_slot,
		.view_id = id->view.view_id, .view_index = id->view.voidx,
		.frame_num = decode->frame_num, .poc = min(decode->top_field_order_cnt,
							decode->bottom_field_order_cnt),
		.decoding = 1, .serial = in->serial++,
	};
	in->picture = *id;
	in->active = free;
	in->references = references;
	in->retain = retain;
	return free;
}

int histb_mvc_ingress_slice(const struct histb_mvc_ingress *in,
		const struct histb_mvc_slice_identity *id,
		const struct v4l2_ctrl_h264_decode_params *decode,
		const struct v4l2_ctrl_h264_slice_params *slice)
{
	unsigned int i;

	if (!in || !id || !decode || !slice)
		return -EINVAL;
	if (in->active < 0 || id->view.view_id != in->picture.view.view_id ||
	    id->view.anchor_pic_flag != in->picture.view.anchor_pic_flag ||
	    id->view.inter_view_flag != in->picture.view.inter_view_flag ||
	    id->view.nal_unit_type != in->picture.view.nal_unit_type ||
	    id->pps_id != in->picture.pps_id || id->nal_ref_idc != in->picture.nal_ref_idc ||
	    decode->frame_num != in->surface[in->active].frame_num)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(decode->dpb); i++) {
		int slot;

		if (!(decode->dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_VALID))
			continue;
		slot = ingress_find(in, decode->dpb[i].reference_ts);
		if (slot < 0 || !(in->references & BIT_ULL(slot)))
			return -EINVAL;
	}
	return ingress_lists(in, id, decode, slice);
}

void histb_mvc_ingress_complete(struct histb_mvc_ingress *in, bool success)
{
	struct histb_mvc_surface *s;
	unsigned int i;

	if (in->active < 0)
		return;
	s = &in->surface[in->active];
	if (success) {
		for (i = 0; i < ARRAY_SIZE(in->surface); i++)
			if (in->surface[i].view_index == s->view_index)
				in->surface[i].temporal = !!(in->retain & BIT_ULL(i));
		s->temporal = !!in->picture.nal_ref_idc;
		s->output_pending = 1;
		if (!s->view_index && in->picture.view.inter_view_flag) {
			s->interview = 1;
			in->base = in->active;
		} else if (s->view_index && in->base >= 0) {
			in->surface[in->base].interview = 0;
			in->base = -1;
		}
	} else {
		/* Do not commit reference marking on hardware failure. */
		memset(s, 0, sizeof(*s));
	}
	s->decoding = 0;
	in->active = -1;
	in->references = 0;
	in->retain = 0;
}

int histb_mvc_ingress_output(struct histb_mvc_ingress *in, unsigned int view)
{
	int slot = -EAGAIN;
	unsigned int i;

	if (view > 1)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(in->surface); i++) {
		struct histb_mvc_surface *s = &in->surface[i];

		if (s->buffer && s->view_index == view && s->output_pending &&
		    (slot < 0 || s->serial < in->surface[slot].serial))
			slot = i;
	}
	if (slot >= 0) {
		in->surface[slot].output_pending = 0;
		in->surface[slot].consumer = 1;
	}
	return slot;
}

int histb_mvc_ingress_requeue(struct histb_mvc_ingress *in, void *buffer)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(in->surface); i++)
		if (in->surface[i].buffer == buffer && in->surface[i].consumer) {
			in->surface[i].consumer = 0;
			return 0;
		}
	return -ENOENT;
}

bool histb_mvc_ingress_busy(const struct histb_mvc_ingress *in, void *buffer)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(in->surface); i++) {
		const struct histb_mvc_surface *s = &in->surface[i];

		if (s->buffer == buffer && (s->temporal || s->interview || s->decoding ||
					 s->output_pending || s->consumer ||
					 (in->references & BIT_ULL(i))))
			return true;
	}
	return false;
}

int histb_mvc_ingress_retire(struct histb_mvc_ingress *in, void *buffer)
{
	unsigned int i;

	if (!buffer || histb_mvc_ingress_busy(in, buffer))
		return -EBUSY;
	for (i = 0; i < ARRAY_SIZE(in->surface); i++)
		if (in->surface[i].buffer == buffer) {
			memset(&in->surface[i], 0, sizeof(in->surface[i]));
			return 0;
		}
	return -ENOENT;
}

int histb_mvc_ingress_drain(struct histb_mvc_ingress *in)
{
	unsigned int i;

	if (in->active >= 0)
		return -EBUSY;
	in->draining = true;
	in->base = -1;
	for (i = 0; i < ARRAY_SIZE(in->surface); i++) {
		in->surface[i].temporal = 0;
		in->surface[i].interview = 0;
	}
	return 0;
}

bool histb_mvc_ingress_drained(const struct histb_mvc_ingress *in)
{
	unsigned int i;

	if (!in->draining || in->active >= 0)
		return false;
	for (i = 0; i < ARRAY_SIZE(in->surface); i++)
		if (in->surface[i].output_pending)
			return false;
	return true;
}
