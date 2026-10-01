// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon HiSTB transport stream demultiplexer
 *
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dvb/ca.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/reset.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <media/dmxdev.h>
#include <media/dvb_demux.h>
#include <media/dvb_frontend.h>
#include <media/dvbdev.h>
#include <media/mxl214.h>

#define HISTB_DMX_MAX_INPUTS		6
#define HISTB_DMX_RAW_DEMUXES		7
#define HISTB_DMX_MAX_FILTERS		256
#define HISTB_DMX_HW_CHANNELS		96
#define HISTB_DMX_CA_SLOTS		32
#define HISTB_DMX_CA_ALIASES		4

#define HISTB_TS_PACKET_SIZE		188
/* Generic full-multiplex mode keeps large blocks to absorb IRQ latency. */
#define HISTB_TS_PACKETS_PER_BLOCK	256
#define HISTB_TS_BLOCK_SIZE		(HISTB_TS_PACKET_SIZE * \
					 HISTB_TS_PACKETS_PER_BLOCK)
#define HISTB_TS_BLOCK_COUNT		32
#define HISTB_TS_DEFAULT_BLOCK_COUNT	16
#define HISTB_TS_LEGACY_BLOCK_COUNT	32
#define HISTB_FQ_DEPTH			(HISTB_TS_BLOCK_COUNT + 1)
#define HISTB_OQ_DEPTH			HISTB_TS_BLOCK_COUNT
#define HISTB_IRQ_DRAIN_PASSES		HISTB_OQ_DEPTH
#define HISTB_QUEUE_RECOVERY_LIMIT	3
#define HISTB_DMX_FLUSH_POLL_ATTEMPTS	0x1000
#define HISTB_DMX_CLEAR_TIMEOUT_US	100000
#define HISTB_PARTIAL_POLL_MS		20

#define HISTB_FQ_REC_BASE		2
#define HISTB_FQ_ALMOST_FULL		4
#define HISTB_OQ_ALMOST_FULL		4
#define HISTB_OQ_IRQ_THRESHOLD		1

#define HISTB_TS_INTERFACE(port)	(0x1100 + (port) * 0x100)
#define HISTB_TS_DUMMY_FORCE(port)	(0x11d0 + (port) * 0x100)
#define HISTB_TS_COUNT(port)		(0x1104 + (port) * 0x100)
#define HISTB_TS_COUNT_CTRL(port)	(0x1108 + (port) * 0x100)
#define HISTB_TS_ERROR_COUNT(port)	(0x110c + (port) * 0x100)
#define HISTB_TS_ERROR_CTRL(port)	(0x1110 + (port) * 0x100)
#define HISTB_TS_AFIFO_STATUS(port)	(0x1118 + (port) * 0x100)
#define HISTB_TS_SYNCON_TH		GENMASK(2, 0)
#define HISTB_TS_SYNCOFF_TH		GENMASK(5, 4)
#define HISTB_TS_FIFO_RATE		GENMASK(10, 8)
#define HISTB_TS_FIFO_MODE		BIT(11)
#define HISTB_TS_SERIAL_2BIT		BIT(12)
#define HISTB_TS_SERIAL_NOSYNC		BIT(14)
#define HISTB_TS_SERIAL_NOVALID		BIT(15)
#define HISTB_TS_NOSYNC_204		BIT(16)
#define HISTB_TS_DUMMY_SYNC		BIT(19)
#define HISTB_TS_SYNC_MODE		GENMASK(21, 20)
#define HISTB_TS_SYNC_CLEAR		BIT(28)
#define HISTB_TS_SERIAL			BIT(29)
#define HISTB_TS_DATA_LINE_0		BIT(30)
#define HISTB_TS_PORT_ENABLE		BIT(31)
#define HISTB_TS_COUNTER_START		1


#define HISTB_SWITCH_CFG0		0x3a00
#define HISTB_SWITCH_CFG1		0x3a04
#define HISTB_SWITCH_FAKE_ENABLE		0x3a10
#define HISTB_SWITCH_FAKE_ENABLE_BIT	BIT(31)
#define HISTB_DMX_CTRL_FUNC		0x3a30
#define HISTB_DMX_GLB_CTRL2(id)		(0x3b04 + (id) * 0x10)
#define HISTB_DMX_GLB_CTRL3(id)		(0x3b08 + (id) * 0x10)
#define HISTB_DMX_GLB_FLUSH		0x3a80
#define HISTB_DMX_FLUSH_CHANNEL		GENMASK(6, 0)
#define HISTB_DMX_FLUSH_TYPE		GENMASK(9, 8)
#define HISTB_DMX_FLUSH_COMMAND		BIT(12)
#define HISTB_DMX_FLUSH_DONE		BIT(16)
#define HISTB_DMX_FLUSH_TYPE_RECORD	2
#define HISTB_DMX_SPS_REF_REC_CHANNEL	GENMASK(15, 8)
#define HISTB_DMX_SPS_REF_REC_INVALID	0xff
#define HISTB_DMX_SPS_PAUSE_TS_TAIL	BIT(20)
#define HISTB_DMX_MODE0			0x3e40
#define HISTB_DMX_MODE0_PID_COPY	BIT(0)
#define HISTB_DMX_REC_TYPE_DESCRAM_TS	1
#define HISTB_DMX_REC_TYPE_SCRAM_TS	3
#define HISTB_DMX_REC_TYPE_ALL_TS	4
#define HISTB_DMX_REC_BUF		GENMASK(14, 8)
#define HISTB_DMX_REC_BUF_ENABLE	BIT(16)
#define HISTB_DMX_REC_BUF_INVALID	FIELD_MAX(HISTB_DMX_REC_BUF)

#define HISTB_DMX_PID_CTRL(id)		(0x3000 + (id) * 4)
#define HISTB_DMX_PID_EN(id)		(0x3200 + (id) * 4)
#define HISTB_DMX_PID_VALUE(id)		(0x3400 + (id) * 4)
#define HISTB_DMX_PID_REC_BUF(id)	(0x3600 + (id) * 4)
#define HISTB_DMX_CHANNEL_TS_COUNT(id)	(0xda00 + (id) * 4)
#define HISTB_DMX_PID_DATA_TYPE		GENMASK(1, 0)
#define HISTB_DMX_PID_CW_INDEX		GENMASK(6, 2)
#define HISTB_DMX_PID_DESCRAMBLE	BIT(7)
#define HISTB_DMX_PID_AF_MODE		GENMASK(9, 8)
#define HISTB_DMX_PID_PUSI_DISABLE	BIT(14)
#define HISTB_DMX_PID_CC_REPEAT		BIT(15)
#define HISTB_DMX_PID_TS_POST		BIT(16)
#define HISTB_DMX_PID_REC_DMX		GENMASK(6, 4)
#define HISTB_DMX_PID_VALUE_MASK	GENMASK(12, 0)
#define HISTB_DMX_PID_REC_OQ		GENMASK(6, 0)

#define HISTB_DMX_CW_SET		0x3b80
#define HISTB_DMX_CW_DATA		0x3b84
#define HISTB_DMX_CA_INFO0		0x3b90
#define HISTB_DMX_CA_ENTROPY		0x3ba8
#define HISTB_DMX_CHAN_CW_TAB(id)	(0x3be0 + ((id) / 16) * 4)
#define HISTB_DMX_CW_WORD		GENMASK(1, 0)
#define HISTB_DMX_CW_PARITY		BIT(8)
#define HISTB_DMX_CW_GROUP		GENMASK(13, 9)
#define HISTB_DMX_CW_TYPE		GENMASK(23, 16)
#define HISTB_DMX_CW_IV		BIT(24)
#define HISTB_DMX_CA_HARDONLY_CSA2	BIT(0)
#define HISTB_DMX_CA_DISABLE_CSA2	BIT(8)

/* CA_SET_PID was removed from current UAPI, but OSCam still uses its ABI. */
struct histb_ca_pid {
	u32 pid;
	s32 index;
};

#define HISTB_CA_SET_PID	_IOW('o', 135, struct histb_ca_pid)

#define HISTB_REC_BUF_SET(id)		(0x8400 + (id) * 4)
#define HISTB_DMX_REC_SET(id)		(0x84d0 + (id) * 4)
#define HISTB_REC_BUF_ID			GENMASK(7, 0)
#define HISTB_REC_BUF_VALID		BIT(12)
#define HISTB_REC_BUF_INVALID		FIELD_MAX(HISTB_REC_BUF_ID)

#define HISTB_PVR_INT_SCAN		0x0000
#define HISTB_ENA_PVR_INT		0x0004
#define HISTB_PVR_INT_DATA_AVAILABLE	BIT(8)
#define HISTB_INT_FQ_CHANNEL0		0xc028
#define HISTB_RAW_FQ_CHANNEL0		0xc020
#define HISTB_ENA_FQ_CHANNEL0		0xc030
#define HISTB_TYPE_FQ_CHANNEL0		0xc038
#define HISTB_RAW_OQ_DESC0		0xc040
#define HISTB_INT_OQ_DESC0		0xc050
#define HISTB_ENA_OQ_DESC0		0xc060
#define HISTB_CLR_OQ_SMMU0		0xcf10
#define HISTB_ENA_OQ_SMMU0		0xcf30
#define HISTB_RAW_CLEAR_CHANNEL		0xc0e0
#define HISTB_CLEAR_CHANNEL_COMMAND	0xc184
#define HISTB_CLEAR_CHANNEL_ID		GENMASK(7, 0)
#define HISTB_CLEAR_CHANNEL_START	BIT(8)
#define HISTB_CLEAR_CHANNEL_TYPE		GENMASK(10, 9)
#define HISTB_CLEAR_CHANNEL_DONE	BIT(0)
#define HISTB_CLEAR_CHANNEL_RECORD	1
#define HISTB_INT_STA_TYPE		0xc100
#define HISTB_ENA_INT_TYPE		0xc104
#define HISTB_INT_STA_ALL		0xc110
#define HISTB_ENA_INT_ALL		0xc114
#define HISTB_OQ_ENABLE0			0xc130
#define HISTB_FQ_ENABLE0			0xc140
#define HISTB_REC_TSCNT_CFG0		0xc1e0
#define HISTB_SCD_TSCNT_ENABLE		0xc1f0
#define HISTB_FQ_INIT_DONE		0xc160
#define HISTB_OQ_INIT_DONE		0xc164
#define HISTB_DESC_O_IRQ_REGION0	BIT(4)
#define HISTB_FQ_IRQ_REGION0		BIT(2)
#define HISTB_INT_ALL			BIT(0)
#define HISTB_INT_TYPE_REGION0		(HISTB_FQ_IRQ_REGION0 | \
					 HISTB_DESC_O_IRQ_REGION0)
#define HISTB_DMX_CLR_WAIT_TIME		0xc180
#define HISTB_DMX_CLR_WAIT_DEFAULT	0x2400

#define HISTB_FQ_WORD0(id)		(0xd000 + (id) * 0x10)
#define HISTB_FQ_WORD1(id)		(0xd004 + (id) * 0x10)
#define HISTB_FQ_WORD2(id)		(0xd008 + (id) * 0x10)
#define HISTB_FQ_WORD3(id)		(0xd00c + (id) * 0x10)

#define HISTB_OQ_WORD0(id)		(0xe000 + (id) * 0x10)
#define HISTB_OQ_WORD1(id)		(0xe004 + (id) * 0x10)
#define HISTB_OQ_WORD2(id)		(0xe008 + (id) * 0x10)
#define HISTB_OQ_WORD3(id)		(0xe00c + (id) * 0x10)
#define HISTB_OQ_WORD4(id)		(0xe800 + (id) * 0x10)
#define HISTB_OQ_WORD5(id)		(0xe804 + (id) * 0x10)
#define HISTB_OQ_WORD6(id)		(0xe808 + (id) * 0x10)
#define HISTB_OQ_WORD7(id)		(0xe80c + (id) * 0x10)
#define HISTB_OQ_INTERRUPT_COUNT(id)	(0xd800 + (id) * 0x4)
#define HISTB_OQ_WRITE_PTR		GENMASK(9, 0)
#define HISTB_OQ_BUFFER_VALID		BIT(7)

enum histb_ts_mode {
	HISTB_TS_PARALLEL_VALID,
	HISTB_TS_PARALLEL_NOSYNC_188,
	HISTB_TS_PARALLEL_NOSYNC_204,
	HISTB_TS_PARALLEL_NOSYNC_AUTO,
	HISTB_TS_SERIAL_SYNC,
	HISTB_TS_SERIAL_NOSYNC_MODE,
	HISTB_TS_SERIAL_NOSYNC_NOVALID_MODE,
};

struct histb_fq_desc {
	__le32 start_addr;
	__le32 buf_len;
};

struct histb_oq_desc {
	__le32 start_addr;
	__le32 ca_ctrl_buf_len;
	__le32 pvr_ctrl_data_len;
	__le32 reserved;
};

struct histb_ts_buffer {
	void *cpu;
	dma_addr_t dma;
};

struct histb_dmx;

struct histb_dmx_channel {
	bool used;
	u8 input;
	u16 pid;
	unsigned int refs;
};

struct histb_dmx_input {
	struct histb_dmx *dmx;
	u32 port;
	enum histb_ts_mode mode;
	bool clock_inverted;
	bool legacy_ts_tuning;
	bool hardware_pid_filtering;
	unsigned int block_count;
	unsigned int fq_depth;
	unsigned int oq_depth;
	u32 serial_data_line;
	struct clk *clk;

	struct mutex lock; /* Serializes feed state and queue processing. */
	unsigned int feed_count;
	unsigned int all_pid_count;
	unsigned int software_feed_count;
	DECLARE_BITMAP(software_feeds, HISTB_DMX_MAX_FILTERS);
	bool running;
	bool record_enabled;
	bool selected_pid_mode;
	bool descrambled_record_mode;
	bool queue_faulted;
	bool flush_faulted;
	s8 ca_pid_slot[DMX_MAX_PID];
	s8 ca_slot_hw[HISTB_DMX_CA_SLOTS];
	u8 ca_cw[HISTB_DMX_CA_SLOTS][2][8];
	bool ca_cw_valid[HISTB_DMX_CA_SLOTS][2];

	struct histb_ts_buffer buffers[HISTB_TS_BLOCK_COUNT];
	struct histb_fq_desc *fq_desc;
	dma_addr_t fq_dma;
	struct histb_oq_desc *oq_desc;
	dma_addr_t oq_dma;
	u16 fq_write;
	u16 oq_read;
	u32 partial_addr;
	u32 partial_len;
	u8 fq_id;
	u8 oq_id;
	u8 rec_id;

	struct dvb_adapter adapter;
	struct dvb_device *ca_devs[HISTB_DMX_CA_ALIASES];
	struct dvb_demux demux;
	struct dmxdev dmxdev;
	struct dmx_frontend hw_frontend;
	struct dmx_frontend mem_frontend;
	struct i2c_client *frontend_client;
	struct dvb_frontend *frontend;
	bool adapter_registered;
	unsigned int ca_registered;
	unsigned int ca_open_count;
	bool demux_registered;
	bool dmxdev_registered;
	bool hw_frontend_registered;
	bool mem_frontend_registered;
	bool frontend_registered;

	/* Updated by the threaded drain path. */
	u64 irq_events;
	u64 blocks_done;
	u64 bytes_done;
	u64 partial_reads;
	u64 invalid_desc;
	u64 misaligned_desc;
	u64 queue_recoveries;
	u64 queue_recovery_failures;
	u64 flush_failures;
	u64 clear_commands;
	u64 clear_pre_busy;
	u64 clear_slow_completions;
	u64 clear_timeouts;
	u64 fq_almost_full;
	u64 fq_overflows;
	unsigned int queue_recovery_attempts;
};

static bool histb_dmx_select_pid(const struct histb_dmx_input *input)
{
	return input->hardware_pid_filtering && !input->all_pid_count &&
	       !input->software_feed_count;
}

static int histb_dmx_update_stream_mode_locked(struct histb_dmx_input *input);
static int histb_dmx_flush_channel(struct histb_dmx *dmx,
				   unsigned int channel_id);

struct histb_dmx {
	struct device *dev;
	struct delayed_work partial_work;
	void __iomem *regs;
	struct reset_control *reset;
	struct clk_bulk_data core_clks[3];
	struct histb_dmx_input inputs[HISTB_DMX_MAX_INPUTS];
	unsigned int input_count;
	int irq;
	spinlock_t reg_lock; /* Protects shared register updates. */
	struct mutex ca_lock; /* Serializes PID channels and clear-CW slots. */
	struct histb_dmx_channel channels[HISTB_DMX_HW_CHANNELS];
	DECLARE_BITMAP(ca_slots, HISTB_DMX_CA_SLOTS);
	u32 ca_info;
	bool normal_csa2;
	u32 irq_pending;
	u32 fq_pending;
	u32 fq_overflow_pending;
	u32 active_oqs;
	bool irq_live;
	bool clear_engine_faulted;
};

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static void histb_dmx_update_bits(struct histb_dmx *dmx, u32 offset,
				  u32 mask, u32 value)
{
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&dmx->reg_lock, flags);
	reg = readl(dmx->regs + offset);
	reg = (reg & ~mask) | (value & mask);
	writel(reg, dmx->regs + offset);
	spin_unlock_irqrestore(&dmx->reg_lock, flags);
}

static struct histb_dmx_channel *
histb_dmx_find_channel_locked(struct histb_dmx_input *input, u16 pid)
{
	struct histb_dmx *dmx = input->dmx;
	unsigned int id;

	for (id = 0; id < HISTB_DMX_HW_CHANNELS; id++) {
		struct histb_dmx_channel *channel = &dmx->channels[id];

		if (channel->used && channel->input == input->port &&
		    channel->pid == pid)
			return channel;
	}

	return NULL;
}

static unsigned int histb_dmx_channel_id(struct histb_dmx *dmx,
					 struct histb_dmx_channel *channel)
{
	return channel - dmx->channels;
}

static void histb_dmx_write_cw_locked(struct histb_dmx *dmx,
				      unsigned int slot, unsigned int parity,
				      const u8 cw[8])
{
	unsigned int word;
	u32 ctrl;

	for (word = 0; word < 2; word++) {
		ctrl = readl(dmx->regs + HISTB_DMX_CW_SET);
		ctrl &= ~(HISTB_DMX_CW_WORD | HISTB_DMX_CW_PARITY |
			  HISTB_DMX_CW_GROUP | HISTB_DMX_CW_TYPE |
			  HISTB_DMX_CW_IV);
		ctrl |= FIELD_PREP(HISTB_DMX_CW_WORD, word) |
			FIELD_PREP(HISTB_DMX_CW_GROUP, slot);
		if (parity)
			ctrl |= HISTB_DMX_CW_PARITY;

		writel(ctrl, dmx->regs + HISTB_DMX_CW_SET);
		writel(get_unaligned_le32(cw + word * sizeof(u32)),
		       dmx->regs + HISTB_DMX_CW_DATA);
	}

	readl(dmx->regs + HISTB_DMX_CW_DATA);
}

static void histb_dmx_clear_cw_locked(struct histb_dmx *dmx,
				      unsigned int slot)
{
	static const u8 zero_cw[8];

	histb_dmx_write_cw_locked(dmx, slot, 0, zero_cw);
	histb_dmx_write_cw_locked(dmx, slot, 1, zero_cw);
}

static int histb_dmx_alloc_ca_slot_locked(struct histb_dmx_input *input,
					  unsigned int index)
{
	struct histb_dmx *dmx = input->dmx;
	unsigned int slot;

	if (!dmx->normal_csa2)
		return -EOPNOTSUPP;
	if (input->ca_slot_hw[index] >= 0)
		return input->ca_slot_hw[index];

	slot = find_first_zero_bit(dmx->ca_slots, HISTB_DMX_CA_SLOTS);
	if (slot == HISTB_DMX_CA_SLOTS)
		return -ENOSPC;

	__set_bit(slot, dmx->ca_slots);
	input->ca_slot_hw[index] = slot;
	histb_dmx_clear_cw_locked(dmx, slot);
	/* The original normal CSA2 API defaults to OPEN (48-bit) entropy. */
	histb_dmx_update_bits(dmx, HISTB_DMX_CA_ENTROPY, BIT(slot), 0);
	if (input->ca_cw_valid[index][0])
		histb_dmx_write_cw_locked(dmx, slot, 0,
					  input->ca_cw[index][0]);
	if (input->ca_cw_valid[index][1])
		histb_dmx_write_cw_locked(dmx, slot, 1,
					  input->ca_cw[index][1]);

	return slot;
}

static void histb_dmx_set_channel_ca_locked(struct histb_dmx_input *input,
					    unsigned int channel_id,
					    int slot)
{
	struct histb_dmx *dmx = input->dmx;
	u32 mask = HISTB_DMX_PID_CW_INDEX | HISTB_DMX_PID_DESCRAMBLE;
	u32 shift = (channel_id % 16) * 2;

	if (slot < 0) {
		histb_dmx_update_bits(dmx, HISTB_DMX_PID_CTRL(channel_id),
				      mask, 0);
		return;
	}

	/* CSA2 uses CW table zero on HiPVRV200. */
	histb_dmx_update_bits(dmx, HISTB_DMX_CHAN_CW_TAB(channel_id),
			      GENMASK(shift + 1, shift), 0);
	histb_dmx_update_bits(dmx, HISTB_DMX_PID_CTRL(channel_id), mask,
			      FIELD_PREP(HISTB_DMX_PID_CW_INDEX, slot) |
			      HISTB_DMX_PID_DESCRAMBLE);
}

static void histb_dmx_set_channel_record_locked(struct histb_dmx_input *input,
						unsigned int channel_id,
						bool enable)
{
	histb_dmx_update_bits(input->dmx, HISTB_DMX_PID_EN(channel_id),
			      HISTB_DMX_PID_REC_DMX,
			      enable ? FIELD_PREP(HISTB_DMX_PID_REC_DMX,
						  input->port + 1) : 0);
}

static void histb_dmx_program_channel_locked(struct histb_dmx_input *input,
					     unsigned int channel_id,
					     u16 pid)
{
	struct histb_dmx *dmx = input->dmx;
	int index = input->ca_pid_slot[pid];
	int slot = index >= 0 ? input->ca_slot_hw[index] : -1;
	u32 ctrl;

	ctrl = FIELD_PREP(HISTB_DMX_PID_DATA_TYPE, 1) |
	       FIELD_PREP(HISTB_DMX_PID_AF_MODE, 1) |
	       HISTB_DMX_PID_PUSI_DISABLE | HISTB_DMX_PID_CC_REPEAT |
	       HISTB_DMX_PID_TS_POST;

	writel(0, dmx->regs + HISTB_DMX_PID_EN(channel_id));
	writel(ctrl, dmx->regs + HISTB_DMX_PID_CTRL(channel_id));
	writel(FIELD_PREP(HISTB_DMX_PID_VALUE_MASK, pid),
	       dmx->regs + HISTB_DMX_PID_VALUE(channel_id));
	writel(FIELD_PREP(HISTB_DMX_PID_REC_OQ, input->oq_id),
	       dmx->regs + HISTB_DMX_PID_REC_BUF(channel_id));
	histb_dmx_set_channel_ca_locked(input, channel_id, slot);
	histb_dmx_set_channel_record_locked(input, channel_id,
					    histb_dmx_select_pid(input));
}

static void histb_dmx_clear_channel_locked(struct histb_dmx *dmx,
					   unsigned int channel_id)
{
	u32 shift = (channel_id % 16) * 2;

	writel(0, dmx->regs + HISTB_DMX_PID_EN(channel_id));
	writel(0, dmx->regs + HISTB_DMX_PID_CTRL(channel_id));
	writel(FIELD_PREP(HISTB_DMX_PID_VALUE_MASK, 0x1fff),
	       dmx->regs + HISTB_DMX_PID_VALUE(channel_id));
	writel(0, dmx->regs + HISTB_DMX_PID_REC_BUF(channel_id));
	histb_dmx_update_bits(dmx, HISTB_DMX_CHAN_CW_TAB(channel_id),
			      GENMASK(shift + 1, shift), 0);
}

static int histb_dmx_get_channel_locked(struct histb_dmx_input *input, u16 pid)
{
	struct histb_dmx *dmx = input->dmx;
	struct histb_dmx_channel *channel;
	unsigned int id;

	channel = histb_dmx_find_channel_locked(input, pid);
	if (channel) {
		channel->refs++;
		return 0;
	}

	for (id = 0; id < HISTB_DMX_HW_CHANNELS; id++)
		if (!dmx->channels[id].used)
			break;
	if (id == HISTB_DMX_HW_CHANNELS)
		return -ENOSPC;

	channel = &dmx->channels[id];
	channel->used = true;
	channel->input = input->port;
	channel->pid = pid;
	channel->refs = 1;
	histb_dmx_program_channel_locked(input, id, pid);

	return 0;
}

static int histb_dmx_put_channel_locked(struct histb_dmx_input *input, u16 pid)
{
	struct histb_dmx *dmx = input->dmx;
	struct histb_dmx_channel *channel;
	unsigned int id;

	channel = histb_dmx_find_channel_locked(input, pid);
	if (!channel || !channel->refs)
		return -ENOENT;

	if (channel->refs > 1) {
		channel->refs--;
		return 0;
	}

	id = histb_dmx_channel_id(dmx, channel);
	if (input->running) {
		int ret = histb_dmx_flush_channel(dmx, id);

		if (ret) {
			input->flush_failures++;
			input->queue_faulted = true;
			input->flush_faulted = true;
			dev_err_ratelimited(dmx->dev,
					    "input %u channel %u release flush failed: %d\n",
					    input->port, id, ret);
			return ret;
		}
	}

	channel->refs = 0;
	histb_dmx_clear_channel_locked(dmx, id);
	memset(channel, 0, sizeof(*channel));

	return 0;
}

static bool
histb_dmx_uses_hardware_ca_locked(const struct histb_dmx_input *input)
{
	const struct histb_dmx *dmx = input->dmx;
	unsigned int id;

	for (id = 0; id < HISTB_DMX_HW_CHANNELS; id++) {
		const struct histb_dmx_channel *channel = &dmx->channels[id];
		int index;

		if (!channel->used || channel->input != input->port)
			continue;

		index = input->ca_pid_slot[channel->pid];
		if (index >= 0 && input->ca_slot_hw[index] >= 0)
			return true;
	}

	return false;
}

static void histb_dmx_release_ca_slot_locked(struct histb_dmx_input *input,
					     unsigned int index)
{
	struct histb_dmx *dmx = input->dmx;
	int slot = input->ca_slot_hw[index];
	unsigned int pid;

	if (slot < 0)
		return;

	for (pid = 0; pid < DMX_MAX_PID; pid++)
		if (input->ca_pid_slot[pid] == index)
			return;

	histb_dmx_clear_cw_locked(dmx, slot);
	histb_dmx_update_bits(dmx, HISTB_DMX_CA_ENTROPY, BIT(slot), 0);
	__clear_bit(slot, dmx->ca_slots);
	input->ca_slot_hw[index] = -1;
}

static void histb_dmx_reset_ca_locked(struct histb_dmx_input *input)
{
	struct histb_dmx *dmx = input->dmx;
	unsigned int index;
	unsigned int id;

	for (id = 0; id < HISTB_DMX_HW_CHANNELS; id++) {
		struct histb_dmx_channel *channel = &dmx->channels[id];

		if (channel->used && channel->input == input->port)
			histb_dmx_set_channel_ca_locked(input, id, -1);
	}

	memset(input->ca_pid_slot, -1, sizeof(input->ca_pid_slot));
	memset(input->ca_cw, 0, sizeof(input->ca_cw));
	memset(input->ca_cw_valid, 0, sizeof(input->ca_cw_valid));
	for (index = 0; index < HISTB_DMX_CA_SLOTS; index++)
		histb_dmx_release_ca_slot_locked(input, index);
}

static int histb_dmx_ca_set_pid_locked(struct histb_dmx_input *input,
				       const struct histb_ca_pid *ca_pid)
{
	struct histb_dmx *dmx = input->dmx;
	struct histb_dmx_channel *channel;
	int channel_id = -1;
	int old_index;
	int old_slot = -1;
	int slot = -1;
	int ret;

	if (ca_pid->pid >= DMX_MAX_PID || ca_pid->index < -1 ||
	    ca_pid->index >= HISTB_DMX_CA_SLOTS)
		return -EINVAL;
	if (ca_pid->index >= 0 &&
	    (input->all_pid_count || input->software_feed_count))
		return -ENOSPC;

	if (ca_pid->index >= 0) {
		slot = histb_dmx_alloc_ca_slot_locked(input, ca_pid->index);
		if (slot < 0)
			return slot;
	}

	old_index = input->ca_pid_slot[ca_pid->pid];
	if (old_index >= 0)
		old_slot = input->ca_slot_hw[old_index];
	input->ca_pid_slot[ca_pid->pid] = ca_pid->index;
	channel = histb_dmx_find_channel_locked(input, ca_pid->pid);
	if (channel) {
		channel_id = histb_dmx_channel_id(dmx, channel);
		histb_dmx_set_channel_ca_locked(input, channel_id, slot);
	}

	ret = input->running ? histb_dmx_update_stream_mode_locked(input) : 0;
	if (ret) {
		input->ca_pid_slot[ca_pid->pid] = old_index;
		if (channel_id >= 0)
			histb_dmx_set_channel_ca_locked(input, channel_id,
							old_slot);
		if (ca_pid->index >= 0 && ca_pid->index != old_index)
			histb_dmx_release_ca_slot_locked(input, ca_pid->index);
		return ret;
	}

	if (old_index >= 0 && old_index != ca_pid->index)
		histb_dmx_release_ca_slot_locked(input, old_index);

	return 0;
}

static int histb_dmx_ca_ioctl(struct file *file, unsigned int cmd, void *arg)
{
	struct dvb_device *dvbdev = file->private_data;
	struct histb_dmx_input *input = dvbdev->priv;
	struct histb_dmx *dmx = input->dmx;
	int ret = 0;

	switch (cmd) {
	case CA_RESET:
		mutex_lock(&input->lock);
		mutex_lock(&dmx->ca_lock);
		histb_dmx_reset_ca_locked(input);
		if (input->running)
			ret = histb_dmx_update_stream_mode_locked(input);
		mutex_unlock(&dmx->ca_lock);
		mutex_unlock(&input->lock);
		break;
	case CA_GET_CAP: {
		struct ca_caps *caps = arg;

		caps->slot_num = 0;
		caps->slot_type = CA_DESCR;
		caps->descr_num = dmx->normal_csa2 ? HISTB_DMX_CA_SLOTS : 0;
		caps->descr_type = dmx->normal_csa2 ? CA_ECD : 0;
		break;
	}
	case CA_GET_DESCR_INFO: {
		struct ca_descr_info *info = arg;

		info->num = dmx->normal_csa2 ? HISTB_DMX_CA_SLOTS : 0;
		info->type = dmx->normal_csa2 ? CA_ECD : 0;
		break;
	}
	case CA_SET_DESCR: {
		const struct ca_descr *descr = arg;
		int slot;

		if (descr->index >= HISTB_DMX_CA_SLOTS || descr->parity > 1)
			return -EINVAL;

		mutex_lock(&dmx->ca_lock);
		slot = histb_dmx_alloc_ca_slot_locked(input, descr->index);
		if (slot < 0)
			ret = slot;
		else
			histb_dmx_write_cw_locked(dmx, slot, descr->parity,
						  descr->cw);
		if (!ret) {
			memcpy(input->ca_cw[descr->index][descr->parity],
			       descr->cw, sizeof(descr->cw));
			input->ca_cw_valid[descr->index][descr->parity] = true;
		}
		mutex_unlock(&dmx->ca_lock);
		break;
	}
	case HISTB_CA_SET_PID:
		mutex_lock(&input->lock);
		mutex_lock(&dmx->ca_lock);
		ret = histb_dmx_ca_set_pid_locked(input, arg);
		mutex_unlock(&dmx->ca_lock);
		mutex_unlock(&input->lock);
		break;
	case CA_GET_SLOT_INFO:
	case CA_GET_MSG:
	case CA_SEND_MSG:
		ret = -EOPNOTSUPP;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static int histb_dmx_ca_release(struct inode *inode, struct file *file)
{
	struct dvb_device *dvbdev = file->private_data;
	struct histb_dmx_input *input;
	struct histb_dmx *dmx;

	if (!dvbdev)
		return -ENODEV;

	input = dvbdev->priv;
	dmx = input->dmx;

	mutex_lock(&input->lock);
	mutex_lock(&dmx->ca_lock);
	if (WARN_ON(!input->ca_open_count)) {
		mutex_unlock(&dmx->ca_lock);
		mutex_unlock(&input->lock);
		return dvb_generic_release(inode, file);
	}

	input->ca_open_count--;
	/* Drop stale PID/CW state after the final CA alias is closed. */
	if (!input->ca_open_count) {
		histb_dmx_reset_ca_locked(input);
		if (input->running)
			histb_dmx_update_stream_mode_locked(input);
	}
	mutex_unlock(&dmx->ca_lock);
	mutex_unlock(&input->lock);

	return dvb_generic_release(inode, file);
}

static int histb_dmx_ca_open(struct inode *inode, struct file *file)
{
	struct dvb_device *dvbdev;
	struct histb_dmx_input *input;
	struct histb_dmx *dmx;
	int ret;

	ret = dvb_generic_open(inode, file);
	if (ret)
		return ret;

	dvbdev = file->private_data;
	input = dvbdev->priv;
	dmx = input->dmx;

	mutex_lock(&dmx->ca_lock);
	input->ca_open_count++;
	mutex_unlock(&dmx->ca_lock);

	return 0;
}

static const struct file_operations histb_dmx_ca_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = dvb_generic_ioctl,
	.open = histb_dmx_ca_open,
	.release = histb_dmx_ca_release,
	.llseek = noop_llseek,
};

static const struct dvb_device histb_dmx_ca_device = {
	.users = 1,
	.readers = 1,
	.writers = 1,
	.fops = &histb_dmx_ca_fops,
	.kernel_ioctl = histb_dmx_ca_ioctl,
};

static void histb_dmx_set_pvr_irq_locked(struct histb_dmx *dmx, bool enable)
{
	writel(enable, dmx->regs + HISTB_ENA_PVR_INT);
	readl(dmx->regs + HISTB_ENA_PVR_INT);
}

static void histb_dmx_ack_oq_locked(struct histb_dmx *dmx, u32 bits)
{
	if (!bits)
		return;

	writel(bits, dmx->regs + HISTB_RAW_OQ_DESC0);
	readl(dmx->regs + HISTB_RAW_OQ_DESC0);
}

static void histb_dmx_set_oq_irq_locked(struct histb_dmx *dmx, u32 mask, u32 value)
{
	u32 enabled;

	if (!mask)
		return;

	enabled = readl(dmx->regs + HISTB_ENA_OQ_DESC0);
	enabled = (enabled & ~mask) | (value & mask);
	writel(enabled, dmx->regs + HISTB_ENA_OQ_DESC0);
	readl(dmx->regs + HISTB_ENA_OQ_DESC0);
}

static void histb_dmx_set_fq_irq_locked(struct histb_dmx *dmx, u32 mask, u32 value)
{
	u32 enabled;

	if (!mask)
		return;

	enabled = readl(dmx->regs + HISTB_ENA_FQ_CHANNEL0);
	enabled = (enabled & ~mask) | (value & mask);
	writel(enabled, dmx->regs + HISTB_ENA_FQ_CHANNEL0);
	readl(dmx->regs + HISTB_ENA_FQ_CHANNEL0);
}

static bool histb_dmx_irq_owned_locked(struct histb_dmx *dmx)
{
	return (readl(dmx->regs + HISTB_PVR_INT_SCAN) &
		HISTB_PVR_INT_DATA_AVAILABLE) ||
	       (readl(dmx->regs + HISTB_INT_STA_ALL) & HISTB_INT_ALL) ||
	       (readl(dmx->regs + HISTB_INT_STA_TYPE) &
		HISTB_INT_TYPE_REGION0) ||
	       readl(dmx->regs + HISTB_INT_OQ_DESC0);
}

static u32 histb_dmx_capture_fq_locked(struct histb_dmx *dmx)
{
	u32 enabled = readl(dmx->regs + HISTB_ENA_FQ_CHANNEL0);
	u32 status = readl(dmx->regs + HISTB_INT_FQ_CHANNEL0);
	u32 type = readl(dmx->regs + HISTB_TYPE_FQ_CHANNEL0);
	u32 fired = status & enabled;
	u32 pending = 0;
	unsigned int port;

	if (!fired)
		return 0;

	histb_dmx_set_fq_irq_locked(dmx, fired, 0);
	writel(fired, dmx->regs + HISTB_RAW_FQ_CHANNEL0);
	readl(dmx->regs + HISTB_RAW_FQ_CHANNEL0);
	for (port = 0; port < HISTB_DMX_MAX_INPUTS; port++) {
		struct histb_dmx_input *input = &dmx->inputs[port];
		u32 bit;

		if (!input->dmx)
			continue;
		bit = BIT(input->fq_id);
		if (!(fired & bit))
			continue;
		pending |= BIT(port);
		if (type & bit)
			dmx->fq_overflow_pending |= BIT(port);
	}
	dmx->fq_pending |= pending;

	return pending;
}

static u32 histb_dmx_capture_oq_locked(struct histb_dmx *dmx,
				       bool preserve_active, u32 *orphan)
{
	u32 active;
	u32 fired;
	u32 raw;
	u32 status;

	status = readl(dmx->regs + HISTB_INT_OQ_DESC0);
	raw = readl(dmx->regs + HISTB_RAW_OQ_DESC0);
	fired = status | raw;
	if (!fired)
		return 0;

	histb_dmx_set_oq_irq_locked(dmx, fired, 0);
	active = fired & dmx->active_oqs;
	*orphan |= fired & ~dmx->active_oqs;
	histb_dmx_ack_oq_locked(dmx, preserve_active ? fired & ~active : fired);
	dmx->irq_pending |= active;

	return active;
}

static void histb_dmx_set_irq_live(struct histb_dmx *dmx, bool live)
{
	unsigned long flags;

	spin_lock_irqsave(&dmx->reg_lock, flags);
	if (live) {
		writel(HISTB_INT_TYPE_REGION0,
		       dmx->regs + HISTB_ENA_INT_TYPE);
		readl(dmx->regs + HISTB_ENA_INT_TYPE);
		writel(HISTB_INT_ALL, dmx->regs + HISTB_ENA_INT_ALL);
		readl(dmx->regs + HISTB_ENA_INT_ALL);
		dmx->irq_live = true;
		histb_dmx_set_pvr_irq_locked(dmx, dmx->active_oqs != 0);
	} else {
		dmx->irq_live = false;
		histb_dmx_set_pvr_irq_locked(dmx, false);
		writel(0, dmx->regs + HISTB_ENA_INT_TYPE);
		readl(dmx->regs + HISTB_ENA_INT_TYPE);
		writel(0, dmx->regs + HISTB_ENA_INT_ALL);
		readl(dmx->regs + HISTB_ENA_INT_ALL);
	}
	spin_unlock_irqrestore(&dmx->reg_lock, flags);
}

static void histb_dmx_mask_all_interrupts(struct histb_dmx *dmx)
{
	unsigned long flags;

	spin_lock_irqsave(&dmx->reg_lock, flags);
	dmx->irq_live = false;
	dmx->irq_pending = 0;
	dmx->fq_pending = 0;
	dmx->fq_overflow_pending = 0;
	dmx->active_oqs = 0;
	histb_dmx_set_pvr_irq_locked(dmx, false);
	writel(0, dmx->regs + HISTB_ENA_FQ_CHANNEL0);
	readl(dmx->regs + HISTB_ENA_FQ_CHANNEL0);
	writel(0, dmx->regs + HISTB_ENA_OQ_DESC0);
	readl(dmx->regs + HISTB_ENA_OQ_DESC0);
	writel(0, dmx->regs + HISTB_ENA_INT_TYPE);
	readl(dmx->regs + HISTB_ENA_INT_TYPE);
	writel(0, dmx->regs + HISTB_ENA_INT_ALL);
	readl(dmx->regs + HISTB_ENA_INT_ALL);
	spin_unlock_irqrestore(&dmx->reg_lock, flags);
}

/*
 * CV200's original demux setup has one global mode bit which is required
 * when more than one logical channel uses the same PID.  Keep this as a
 * separate, checked initialization transaction: silently omitting it makes
 * the first PID owner win and leaves later inputs empty.
 */
static int histb_dmx_init_original(struct histb_dmx *dmx)
{
	unsigned int raw_dmx;
	u32 value;

	for (raw_dmx = 0; raw_dmx < HISTB_DMX_RAW_DEMUXES; raw_dmx++) {
		histb_dmx_update_bits(dmx, HISTB_DMX_GLB_CTRL3(raw_dmx),
				      HISTB_DMX_SPS_REF_REC_CHANNEL,
				      FIELD_PREP(HISTB_DMX_SPS_REF_REC_CHANNEL,
						 HISTB_DMX_SPS_REF_REC_INVALID));
		if (FIELD_GET(HISTB_DMX_SPS_REF_REC_CHANNEL,
			      readl(dmx->regs + HISTB_DMX_GLB_CTRL3(raw_dmx))) !=
		    HISTB_DMX_SPS_REF_REC_INVALID)
			return -EIO;
	}

	histb_dmx_update_bits(dmx, HISTB_DMX_MODE0,
			      HISTB_DMX_MODE0_PID_COPY,
			      HISTB_DMX_MODE0_PID_COPY);
	if (!(readl(dmx->regs + HISTB_DMX_MODE0) &
	      HISTB_DMX_MODE0_PID_COPY))
		return -EIO;

	value = readl(dmx->regs + HISTB_DMX_CLR_WAIT_TIME);
	value = (value & ~GENMASK(15, 0)) | HISTB_DMX_CLR_WAIT_DEFAULT;
	writel(value, dmx->regs + HISTB_DMX_CLR_WAIT_TIME);
	if ((readl(dmx->regs + HISTB_DMX_CLR_WAIT_TIME) & GENMASK(15, 0)) !=
	    HISTB_DMX_CLR_WAIT_DEFAULT)
		return -EIO;

	histb_dmx_update_bits(dmx, HISTB_SWITCH_FAKE_ENABLE,
			      HISTB_SWITCH_FAKE_ENABLE_BIT,
			      HISTB_SWITCH_FAKE_ENABLE_BIT);
	if (!(readl(dmx->regs + HISTB_SWITCH_FAKE_ENABLE) &
	      HISTB_SWITCH_FAKE_ENABLE_BIT))
		return -EIO;

	return 0;
}

static void histb_dmx_set_oq_irq(struct histb_dmx_input *input, bool enable)
{
	struct histb_dmx *dmx = input->dmx;
	unsigned long flags;
	u32 bit = BIT(input->oq_id);

	spin_lock_irqsave(&dmx->reg_lock, flags);
	histb_dmx_set_oq_irq_locked(dmx, bit, 0);
	if (enable) {
		histb_dmx_ack_oq_locked(dmx, bit);
		dmx->irq_pending &= ~bit;
		dmx->active_oqs |= bit;
		histb_dmx_set_oq_irq_locked(dmx, bit, bit);
	} else {
		dmx->irq_pending &= ~bit;
		dmx->active_oqs &= ~bit;
		histb_dmx_ack_oq_locked(dmx, bit);
	}
	histb_dmx_set_pvr_irq_locked(dmx,
				     dmx->irq_live && dmx->active_oqs);
	spin_unlock_irqrestore(&dmx->reg_lock, flags);
}

static int histb_dmx_parse_mode(struct device *dev, struct device_node *np,
				struct histb_dmx_input *input)
{
	const char *mode;
	int ret;

	ret = of_property_read_string(np, "hisilicon,ts-mode", &mode);
	if (ret)
		return dev_err_probe(dev, ret, "%pOF: missing TS mode\n", np);

	if (!strcmp(mode, "parallel-valid"))
		input->mode = HISTB_TS_PARALLEL_VALID;
	else if (!strcmp(mode, "parallel-nosync-188"))
		input->mode = HISTB_TS_PARALLEL_NOSYNC_188;
	else if (!strcmp(mode, "parallel-nosync-204"))
		input->mode = HISTB_TS_PARALLEL_NOSYNC_204;
	else if (!strcmp(mode, "parallel-nosync-auto"))
		input->mode = HISTB_TS_PARALLEL_NOSYNC_AUTO;
	else if (!strcmp(mode, "serial-sync"))
		input->mode = HISTB_TS_SERIAL_SYNC;
	else if (!strcmp(mode, "serial-nosync"))
		input->mode = HISTB_TS_SERIAL_NOSYNC_MODE;
	else if (!strcmp(mode, "serial-nosync-novalid"))
		input->mode = HISTB_TS_SERIAL_NOSYNC_NOVALID_MODE;
	else
		return dev_err_probe(dev, -EINVAL, "%pOF: invalid TS mode %s\n",
				     np, mode);


	input->clock_inverted = of_property_read_bool(np,
					      "hisilicon,clock-inverted");
	input->legacy_ts_tuning =
		of_property_read_bool(np, "hisilicon,cv200-legacy-ts-tuning");
	input->hardware_pid_filtering =
		of_property_read_bool(np, "hisilicon,hardware-pid-filtering");
	input->block_count = input->legacy_ts_tuning ?
		HISTB_TS_LEGACY_BLOCK_COUNT : HISTB_TS_DEFAULT_BLOCK_COUNT;
	input->fq_depth = input->block_count + 1;
	input->oq_depth = input->block_count;
	input->serial_data_line = 0;
	of_property_read_u32(np, "hisilicon,serial-data-line",
			     &input->serial_data_line);

	return 0;
}

static void histb_dmx_configure_port(struct histb_dmx_input *input)
{
	struct histb_dmx *dmx = input->dmx;
	u32 offset = HISTB_TS_INTERFACE(input->port);
	u32 clear_mask;
	u32 value;

	value = readl(dmx->regs + offset);
	value &= ~HISTB_TS_PORT_ENABLE;
	value |= HISTB_TS_SYNC_CLEAR;
	writel(value, dmx->regs + offset);
	usleep_range(2000, 2500);

	clear_mask = HISTB_TS_SYNCON_TH | HISTB_TS_SYNCOFF_TH |
		     HISTB_TS_SERIAL_2BIT | HISTB_TS_SERIAL_NOSYNC |
		     HISTB_TS_SERIAL_NOVALID |
		     HISTB_TS_NOSYNC_204 |
		     HISTB_TS_SYNC_MODE | HISTB_TS_SYNC_CLEAR |
		     HISTB_TS_SERIAL | HISTB_TS_DATA_LINE_0 |
		     HISTB_TS_PORT_ENABLE;
	if (input->legacy_ts_tuning)
		clear_mask |= HISTB_TS_FIFO_RATE | HISTB_TS_FIFO_MODE |
			      HISTB_TS_DUMMY_SYNC;
	value &= ~clear_mask;
	value |= FIELD_PREP(HISTB_TS_SYNCON_TH, 5) |
		 FIELD_PREP(HISTB_TS_SYNCOFF_TH, 1);
	if (input->legacy_ts_tuning)
		value |= FIELD_PREP(HISTB_TS_FIFO_RATE, 3) |
			 HISTB_TS_FIFO_MODE;

	switch (input->mode) {
	case HISTB_TS_PARALLEL_VALID:
		value |= FIELD_PREP(HISTB_TS_SYNC_MODE, 1);
		break;
	case HISTB_TS_PARALLEL_NOSYNC_188:
		value |= FIELD_PREP(HISTB_TS_SYNC_MODE, 2);
		break;
	case HISTB_TS_PARALLEL_NOSYNC_204:
		value |= FIELD_PREP(HISTB_TS_SYNC_MODE, 2) |
			 HISTB_TS_NOSYNC_204;
		break;
	case HISTB_TS_PARALLEL_NOSYNC_AUTO:
		value |= FIELD_PREP(HISTB_TS_SYNC_MODE, 3);
		break;
	case HISTB_TS_SERIAL_SYNC:
		value |= HISTB_TS_SERIAL;
		break;
	case HISTB_TS_SERIAL_NOSYNC_MODE:
		value |= HISTB_TS_SERIAL | HISTB_TS_SERIAL_NOSYNC;
		break;
	case HISTB_TS_SERIAL_NOSYNC_NOVALID_MODE:
		value |= HISTB_TS_SERIAL | HISTB_TS_SERIAL_NOSYNC |
			 HISTB_TS_SERIAL_NOVALID;
		break;
	}

	if (input->serial_data_line == 0)
		value |= HISTB_TS_DATA_LINE_0;

	writel(value, dmx->regs + offset);
	writel(value | HISTB_TS_PORT_ENABLE, dmx->regs + offset);
	dev_info(dmx->dev,
		 "TSI%u %s tuning: if=%08x\n", input->port,
		 input->legacy_ts_tuning ? "legacy" : "generic",
		 readl(dmx->regs + offset));
	writel(HISTB_TS_COUNTER_START,
	       dmx->regs + HISTB_TS_COUNT_CTRL(input->port));
	writel(HISTB_TS_COUNTER_START,
	       dmx->regs + HISTB_TS_ERROR_CTRL(input->port));
}

static void histb_dmx_route_input(struct histb_dmx_input *input)
{
	u32 offset;
	u32 shift;
	u32 source;

	if (input->port < 4) {
		offset = HISTB_SWITCH_CFG0;
		shift = input->port * 8;
	} else {
		offset = HISTB_SWITCH_CFG1;
		shift = (input->port - 4) * 8;
	}

	source = input->port + 1;

	histb_dmx_update_bits(input->dmx, offset,
			      GENMASK(shift + 7, shift), source << shift);
}

static struct histb_ts_buffer *
histb_dmx_find_buffer(struct histb_dmx_input *input, u32 dma)
{
	unsigned int i;

	for (i = 0; i < input->block_count; i++)
		if (lower_32_bits(input->buffers[i].dma) == dma)
			return &input->buffers[i];

	return NULL;
}

static void histb_dmx_set_record(struct histb_dmx_input *input, bool enable,
				 u32 record_type)
{
	struct histb_dmx *dmx = input->dmx;
	u32 shift = input->port * 4 + 1;
	u32 tscnt_shift = input->rec_id * 8;
	u32 value;

	if (!enable) {
		/* Match DmxUnsetRecBuf() before selecting record type NONE. */
		histb_dmx_update_bits(dmx, HISTB_DMX_GLB_CTRL2(input->port),
				      HISTB_DMX_REC_BUF |
				      HISTB_DMX_REC_BUF_ENABLE,
				      FIELD_PREP(HISTB_DMX_REC_BUF,
						 HISTB_DMX_REC_BUF_INVALID) |
				      HISTB_DMX_REC_BUF_ENABLE);
		histb_dmx_update_bits(dmx, HISTB_DMX_REC_SET(input->port),
				      BIT(input->rec_id), 0);
		histb_dmx_update_bits(dmx, HISTB_REC_BUF_SET(input->rec_id),
				      HISTB_REC_BUF_ID | HISTB_REC_BUF_VALID,
				      FIELD_PREP(HISTB_REC_BUF_ID,
						 HISTB_REC_BUF_INVALID));
		histb_dmx_update_bits(dmx, HISTB_DMX_CTRL_FUNC,
				      GENMASK(shift + 2, shift), 0);
		return;
	}

	histb_dmx_update_bits(dmx, HISTB_REC_TSCNT_CFG0,
			      GENMASK(tscnt_shift + 7, tscnt_shift),
			      (input->oq_id | BIT(7)) << tscnt_shift);
	histb_dmx_update_bits(dmx, HISTB_SCD_TSCNT_ENABLE,
			      BIT(input->rec_id), BIT(input->rec_id));
	histb_dmx_update_bits(dmx, HISTB_DMX_GLB_CTRL3(input->port),
			      HISTB_DMX_SPS_PAUSE_TS_TAIL,
			      HISTB_DMX_SPS_PAUSE_TS_TAIL);

	histb_dmx_update_bits(dmx, HISTB_DMX_CTRL_FUNC,
			      GENMASK(shift + 2, shift), record_type << shift);

	value = FIELD_PREP(HISTB_REC_BUF_ID, input->oq_id) |
		HISTB_REC_BUF_VALID;
	writel(value, dmx->regs + HISTB_REC_BUF_SET(input->rec_id));
	histb_dmx_update_bits(dmx, HISTB_DMX_REC_SET(input->port),
			      BIT(input->rec_id), BIT(input->rec_id));
	histb_dmx_update_bits(dmx, HISTB_DMX_GLB_CTRL2(input->port),
			      HISTB_DMX_REC_BUF | HISTB_DMX_REC_BUF_ENABLE,
			      FIELD_PREP(HISTB_DMX_REC_BUF,
					 input->oq_id) |
			      HISTB_DMX_REC_BUF_ENABLE);
}

static void histb_dmx_sync_record_mode_locked(struct histb_dmx_input *input)
{
	struct histb_dmx *dmx = input->dmx;
	bool descramble = histb_dmx_uses_hardware_ca_locked(input);
	bool select_pid = histb_dmx_select_pid(input);
	unsigned int id;

	descramble &= select_pid;
	if (input->record_enabled &&
	    input->selected_pid_mode == select_pid &&
	    input->descrambled_record_mode == descramble)
		return;

	if (select_pid)
		histb_dmx_set_record(input, true,
				     descramble ?
				     HISTB_DMX_REC_TYPE_DESCRAM_TS :
				     HISTB_DMX_REC_TYPE_SCRAM_TS);

	for (id = 0; id < HISTB_DMX_HW_CHANNELS; id++) {
		struct histb_dmx_channel *channel = &dmx->channels[id];

		if (channel->used && channel->input == input->port)
			histb_dmx_set_channel_record_locked(input, id,
							    select_pid);
	}

	if (!select_pid)
		histb_dmx_set_record(input, true, HISTB_DMX_REC_TYPE_ALL_TS);

	input->record_enabled = true;
	input->selected_pid_mode = select_pid;
	input->descrambled_record_mode = descramble;
}

static void histb_dmx_init_descriptors(struct histb_dmx_input *input)
{
	unsigned int i;

	memset(input->fq_desc, 0,
	       sizeof(*input->fq_desc) * input->fq_depth);
	memset(input->oq_desc, 0,
	       sizeof(*input->oq_desc) * input->oq_depth);

	for (i = 0; i < input->block_count; i++) {
		input->fq_desc[i].start_addr =
			cpu_to_le32(lower_32_bits(input->buffers[i].dma));
		input->fq_desc[i].buf_len = cpu_to_le32(HISTB_TS_BLOCK_SIZE);
	}

	input->fq_write = input->fq_depth - 1;
	input->oq_read = 0;
	input->partial_addr = 0;
	input->partial_len = 0;
	dma_wmb();
}

/*
 * The CV200 path primes serial no-sync inputs with a short dummy-sync
 * pulse. Keep it a board DT option: the register is not part of the generic
 * DVB binding and must not be applied to unrelated HiSTB variants.
 */
static void histb_dmx_set_dummy_sync(struct histb_dmx_input *input,
				     bool enable)
{
	struct histb_dmx *dmx = input->dmx;
	u32 offset = HISTB_TS_INTERFACE(input->port);

	if (!input->legacy_ts_tuning)
		return;

	if (enable) {
		writel(1, dmx->regs + HISTB_TS_DUMMY_FORCE(input->port));
		udelay(5);
		writel(0, dmx->regs + HISTB_TS_DUMMY_FORCE(input->port));
		udelay(5);
	}

	histb_dmx_update_bits(dmx, offset, HISTB_TS_DUMMY_SYNC,
			      enable ? HISTB_TS_DUMMY_SYNC : 0);
}

static void histb_dmx_log_stream_state(struct histb_dmx_input *input)
{
	struct histb_dmx *dmx = input->dmx;

	dev_dbg(dmx->dev,
		"input %u stream state: if=%08x ts=%u err=%u afifo=%08x route=%08x dmx=%08x rec=%08x fq=%08x oq=%08x irq=%08x/%08x\n",
		 input->port,
		 readl(dmx->regs + HISTB_TS_INTERFACE(input->port)),
		 readl(dmx->regs + HISTB_TS_COUNT(input->port)),
		 readl(dmx->regs + HISTB_TS_ERROR_COUNT(input->port)),
		 readl(dmx->regs + HISTB_TS_AFIFO_STATUS(input->port)),
		 readl(dmx->regs + (input->port < 4 ? HISTB_SWITCH_CFG0 :
							  HISTB_SWITCH_CFG1)),
		 readl(dmx->regs + HISTB_DMX_CTRL_FUNC),
		 readl(dmx->regs + HISTB_DMX_GLB_CTRL2(input->port)),
		 readl(dmx->regs + HISTB_FQ_WORD2(input->fq_id)),
		 readl(dmx->regs + HISTB_OQ_WORD5(input->oq_id)),
		 readl(dmx->regs + HISTB_INT_OQ_DESC0),
		 readl(dmx->regs + HISTB_ENA_OQ_DESC0));
}

static void
histb_dmx_log_channel_state_locked(struct histb_dmx_input *input, u16 pid)
{
	struct histb_dmx *dmx = input->dmx;
	struct histb_dmx_channel *channel;
	unsigned int id;

	channel = histb_dmx_find_channel_locked(input, pid);
	if (!channel)
		return;

	id = histb_dmx_channel_id(dmx, channel);
	dev_dbg(dmx->dev,
		"input %u channel %u pid %04x: ctrl=%08x en=%08x value=%08x recbuf=%08x hits=%u recset=%08x flag=%08x glb2=%08x glb3=%08x mode=%08x tscnt=%08x replace=%08x fqen=%08x oqen=%08x fq2=%08x oq5=%08x oq6=%08x\n",
		input->port, id, pid,
		readl(dmx->regs + HISTB_DMX_PID_CTRL(id)),
		readl(dmx->regs + HISTB_DMX_PID_EN(id)),
		readl(dmx->regs + HISTB_DMX_PID_VALUE(id)),
		readl(dmx->regs + HISTB_DMX_PID_REC_BUF(id)),
		readl(dmx->regs + HISTB_DMX_CHANNEL_TS_COUNT(id)),
		readl(dmx->regs + HISTB_REC_BUF_SET(input->rec_id)),
		readl(dmx->regs + HISTB_DMX_REC_SET(input->port)),
		readl(dmx->regs + HISTB_DMX_GLB_CTRL2(input->port)),
		readl(dmx->regs + HISTB_DMX_GLB_CTRL3(input->port)),
		readl(dmx->regs + HISTB_DMX_MODE0),
		readl(dmx->regs + HISTB_REC_TSCNT_CFG0),
		readl(dmx->regs + HISTB_SCD_TSCNT_ENABLE),
		readl(dmx->regs + HISTB_FQ_ENABLE0),
		readl(dmx->regs + HISTB_OQ_ENABLE0),
		readl(dmx->regs + HISTB_FQ_WORD2(input->fq_id)),
		readl(dmx->regs + HISTB_OQ_WORD5(input->oq_id)),
		readl(dmx->regs + HISTB_OQ_WORD6(input->oq_id)));
}

static void histb_dmx_start_stream(struct histb_dmx_input *input)
{
	struct histb_dmx *dmx = input->dmx;
	u32 bit = BIT(input->oq_id);
	u32 value;

	histb_dmx_update_bits(dmx, HISTB_FQ_ENABLE0, BIT(input->fq_id), 0);
	histb_dmx_update_bits(dmx, HISTB_OQ_ENABLE0, bit, 0);
	histb_dmx_set_oq_irq(input, false);

	histb_dmx_init_descriptors(input);
	histb_dmx_sync_record_mode_locked(input);

	writel(HISTB_FQ_ALMOST_FULL << 24,
	       dmx->regs + HISTB_FQ_WORD0(input->fq_id));
	writel(0, dmx->regs + HISTB_FQ_WORD1(input->fq_id));
	writel(input->fq_depth << 16 | input->fq_write,
	       dmx->regs + HISTB_FQ_WORD2(input->fq_id));
	writel(lower_32_bits(input->fq_dma),
	       dmx->regs + HISTB_FQ_WORD3(input->fq_id));

	writel(0, dmx->regs + HISTB_OQ_WORD0(input->oq_id));
	writel(0, dmx->regs + HISTB_OQ_WORD1(input->oq_id));
	writel(0, dmx->regs + HISTB_OQ_WORD2(input->oq_id));
	writel(0, dmx->regs + HISTB_OQ_WORD3(input->oq_id));
	writel(0, dmx->regs + HISTB_OQ_WORD4(input->oq_id));
	writel(0, dmx->regs + HISTB_OQ_WORD5(input->oq_id));
	value = HISTB_OQ_IRQ_THRESHOLD << 26 |
		input->oq_depth << 16 |
		HISTB_OQ_ALMOST_FULL << 8 | input->fq_id;
	writel(value, dmx->regs + HISTB_OQ_WORD6(input->oq_id));
	writel(lower_32_bits(input->oq_dma),
	       dmx->regs + HISTB_OQ_WORD7(input->oq_id));

	histb_dmx_update_bits(dmx, HISTB_ENA_OQ_SMMU0, bit, 0);
	writel(bit, dmx->regs + HISTB_CLR_OQ_SMMU0);
	writel(BIT(input->fq_id), dmx->regs + HISTB_RAW_FQ_CHANNEL0);
	histb_dmx_update_bits(dmx, HISTB_ENA_FQ_CHANNEL0,
			      BIT(input->fq_id), BIT(input->fq_id));
	/* Clear stale OQ state and arm it before hardware can produce data. */
	input->running = true;
	histb_dmx_set_oq_irq(input, true);
	histb_dmx_update_bits(dmx, HISTB_FQ_ENABLE0, BIT(input->fq_id),
			      BIT(input->fq_id));
	histb_dmx_update_bits(dmx, HISTB_OQ_ENABLE0, bit, bit);

	histb_dmx_set_dummy_sync(input, true);
}

static int histb_dmx_flush_channel(struct histb_dmx *dmx,
				   unsigned int channel_id)
{
	unsigned int attempt;
	u32 value;

	value = readl(dmx->regs + HISTB_DMX_GLB_FLUSH);
	value &= ~(HISTB_DMX_FLUSH_CHANNEL | HISTB_DMX_FLUSH_TYPE |
		   HISTB_DMX_FLUSH_COMMAND | HISTB_DMX_FLUSH_DONE);
	value |= FIELD_PREP(HISTB_DMX_FLUSH_CHANNEL, channel_id) |
		 FIELD_PREP(HISTB_DMX_FLUSH_TYPE,
			    HISTB_DMX_FLUSH_TYPE_RECORD) |
		 HISTB_DMX_FLUSH_COMMAND;
	writel(value, dmx->regs + HISTB_DMX_GLB_FLUSH);

	for (attempt = 0; attempt < HISTB_DMX_FLUSH_POLL_ATTEMPTS;
	     attempt++) {
		value = readl(dmx->regs + HISTB_DMX_GLB_FLUSH);
		if (value & HISTB_DMX_FLUSH_DONE)
			return 0;
	}

	return -ETIMEDOUT;
}

static int histb_dmx_clear_record_oq(struct histb_dmx_input *input)
{
	struct histb_dmx *dmx = input->dmx;
	unsigned int attempt;
	int ret;
	u32 value;

	if (dmx->clear_engine_faulted)
		return -EIO;

	input->clear_commands++;
	value = readl(dmx->regs + HISTB_CLEAR_CHANNEL_COMMAND);
	if (value & HISTB_CLEAR_CHANNEL_START) {
		input->clear_pre_busy++;
		ret = readl_poll_timeout(dmx->regs + HISTB_CLEAR_CHANNEL_COMMAND,
					 value,
					 !(value & HISTB_CLEAR_CHANNEL_START),
					 1, HISTB_DMX_CLEAR_TIMEOUT_US);
		if (ret)
			goto engine_timeout;
	}

	/* RAW_CLEAR_CHANNEL is W1C; reject a stale completion before arming. */
	writel(HISTB_CLEAR_CHANNEL_DONE,
	       dmx->regs + HISTB_RAW_CLEAR_CHANNEL);
	if (readl(dmx->regs + HISTB_RAW_CLEAR_CHANNEL) &
	    HISTB_CLEAR_CHANNEL_DONE) {
		ret = -EIO;
		goto engine_fault;
	}

	value = readl(dmx->regs + HISTB_CLEAR_CHANNEL_COMMAND);
	value &= ~(HISTB_CLEAR_CHANNEL_ID | HISTB_CLEAR_CHANNEL_START |
		   HISTB_CLEAR_CHANNEL_TYPE);
	value |= FIELD_PREP(HISTB_CLEAR_CHANNEL_ID, input->oq_id) |
		 FIELD_PREP(HISTB_CLEAR_CHANNEL_TYPE,
			    HISTB_CLEAR_CHANNEL_RECORD) |
			 HISTB_CLEAR_CHANNEL_START;
	writel(value, dmx->regs + HISTB_CLEAR_CHANNEL_COMMAND);

	for (attempt = 0; attempt < HISTB_DMX_FLUSH_POLL_ATTEMPTS;
	     attempt++) {
		value = readl(dmx->regs + HISTB_RAW_CLEAR_CHANNEL);
		if (value & HISTB_CLEAR_CHANNEL_DONE)
			break;
	}
	if (attempt == HISTB_DMX_FLUSH_POLL_ATTEMPTS) {
		input->clear_slow_completions++;
		ret = readl_poll_timeout(dmx->regs + HISTB_RAW_CLEAR_CHANNEL,
					 value,
					 value & HISTB_CLEAR_CHANNEL_DONE,
					 1, HISTB_DMX_CLEAR_TIMEOUT_US);
		if (ret)
			goto engine_timeout;
	}

	writel(HISTB_CLEAR_CHANNEL_DONE,
	       dmx->regs + HISTB_RAW_CLEAR_CHANNEL);
	if (readl(dmx->regs + HISTB_RAW_CLEAR_CHANNEL) &
	    HISTB_CLEAR_CHANNEL_DONE) {
		ret = -EIO;
		goto engine_fault;
	}

	ret = readl_poll_timeout(dmx->regs + HISTB_CLEAR_CHANNEL_COMMAND,
				 value, !(value & HISTB_CLEAR_CHANNEL_START),
				 1, HISTB_DMX_CLEAR_TIMEOUT_US);
	if (ret)
		goto engine_timeout;

	return 0;

engine_timeout:
	input->clear_timeouts++;
	ret = -ETIMEDOUT;
engine_fault:
	dmx->clear_engine_faulted = true;
	return ret;
}

static int histb_dmx_flush_record_path_locked(struct histb_dmx_input *input)
{
	struct histb_dmx *dmx = input->dmx;
	unsigned int id;
	int ret;

	for (id = 0; id < HISTB_DMX_HW_CHANNELS; id++) {
		struct histb_dmx_channel *channel = &dmx->channels[id];

		if (!channel->used || channel->input != input->port)
			continue;
		ret = histb_dmx_flush_channel(dmx, id);
		if (ret) {
			input->flush_failures++;
			dev_warn_ratelimited(dmx->dev,
					     "input %u channel %u record flush timed out\n",
					     input->port, id);
			return ret;
		}
	}

	ret = histb_dmx_clear_record_oq(input);
	if (!ret)
		return 0;
	input->flush_failures++;
	if (ret == -ETIMEDOUT)
		dev_warn_ratelimited(dmx->dev,
				     "input %u record OQ clear timed out\n",
				     input->port);
	else
		dev_err_ratelimited(dmx->dev,
				    "input %u record OQ completion state did not clear\n",
				    input->port);
	return ret;
}

static int histb_dmx_stop_stream(struct histb_dmx_input *input)
{
	struct histb_dmx *dmx = input->dmx;
	u32 bit = BIT(input->oq_id);

	histb_dmx_log_stream_state(input);
	input->running = false;
	histb_dmx_set_oq_irq(input, false);
	histb_dmx_set_dummy_sync(input, false);
	histb_dmx_update_bits(dmx, HISTB_ENA_FQ_CHANNEL0,
			      BIT(input->fq_id), 0);
	writel(BIT(input->fq_id), dmx->regs + HISTB_RAW_FQ_CHANNEL0);
	histb_dmx_update_bits(dmx, HISTB_FQ_ENABLE0, BIT(input->fq_id), 0);
	histb_dmx_update_bits(dmx, HISTB_OQ_ENABLE0, bit, 0);
	histb_dmx_update_bits(dmx, HISTB_ENA_OQ_SMMU0, bit, bit);

	/* Match DmxFqStop() and DmxOqStop() before clearing record ownership. */
	writel(0, dmx->regs + HISTB_FQ_WORD0(input->fq_id));
	writel(0, dmx->regs + HISTB_FQ_WORD1(input->fq_id));
	writel(0, dmx->regs + HISTB_FQ_WORD2(input->fq_id));
	writel(0, dmx->regs + HISTB_FQ_WORD3(input->fq_id));
	writel(0, dmx->regs + HISTB_OQ_INTERRUPT_COUNT(input->oq_id));
	writel(0, dmx->regs + HISTB_OQ_WORD0(input->oq_id));
	writel(0, dmx->regs + HISTB_OQ_WORD1(input->oq_id));
	writel(0, dmx->regs + HISTB_OQ_WORD2(input->oq_id));
	writel(0, dmx->regs + HISTB_OQ_WORD3(input->oq_id));
	writel(0, dmx->regs + HISTB_OQ_WORD4(input->oq_id));
	writel(0, dmx->regs + HISTB_OQ_WORD5(input->oq_id));
	writel(0, dmx->regs + HISTB_OQ_WORD6(input->oq_id));
	writel(0, dmx->regs + HISTB_OQ_WORD7(input->oq_id));
	histb_dmx_set_record(input, false, 0);
	input->record_enabled = false;
	if (histb_dmx_flush_record_path_locked(input)) {
		input->queue_faulted = true;
		input->flush_faulted = true;
		return -EIO;
	}

	return 0;
}

static int histb_dmx_update_stream_mode_locked(struct histb_dmx_input *input)
{
	bool hardware_ca = histb_dmx_uses_hardware_ca_locked(input);
	bool select_pid = histb_dmx_select_pid(input);
	bool descramble = hardware_ca && select_pid;

	if (input->hardware_pid_filtering && hardware_ca && !select_pid)
		return -EBUSY;

	if (!input->running) {
		if (input->queue_faulted)
			return -EIO;
		histb_dmx_start_stream(input);
		return 0;
	}

	if (input->record_enabled &&
	    input->selected_pid_mode == select_pid &&
	    input->descrambled_record_mode == descramble)
		return 0;

	if (histb_dmx_stop_stream(input))
		return -EIO;
	histb_dmx_start_stream(input);

	return 0;
}

static int histb_dmx_start_feed(struct dvb_demux_feed *feed)
{
	struct histb_dmx_input *input = feed->demux->priv;
	struct histb_dmx *dmx = input->dmx;
	bool wildcard = feed->pid == DMX_MAX_PID;
	int ret = 0;

	if (feed->type != DMX_TYPE_TS && feed->type != DMX_TYPE_SEC)
		return -EINVAL;

	mutex_lock(&input->lock);
	mutex_lock(&dmx->ca_lock);
	if (input->queue_faulted) {
		ret = -EIO;
		goto unlock;
	}
	if (wildcard) {
		if (histb_dmx_uses_hardware_ca_locked(input))
			ret = -EBUSY;
		else
			input->all_pid_count++;
	} else if (input->hardware_pid_filtering &&
		   input->software_feed_count) {
		if (input->ca_pid_slot[feed->pid] >= 0) {
			ret = -ENOSPC;
		} else {
			__set_bit(feed->index, input->software_feeds);
			input->software_feed_count++;
		}
	} else {
		if (input->hardware_pid_filtering && input->all_pid_count &&
		    input->ca_pid_slot[feed->pid] >= 0)
			ret = -EBUSY;
		else
			ret = histb_dmx_get_channel_locked(input, feed->pid);
		if (ret == -ENOSPC && input->hardware_pid_filtering &&
		    !histb_dmx_uses_hardware_ca_locked(input) &&
		    input->ca_pid_slot[feed->pid] < 0) {
			__set_bit(feed->index, input->software_feeds);
			input->software_feed_count++;
			ret = 0;
			dev_warn_ratelimited(dmx->dev,
					     "input %u exhausted shared PID channels; using ALL_TS fallback\n",
					     input->port);
		}
	}
	if (ret)
		goto unlock;

	input->feed_count++;
	ret = histb_dmx_update_stream_mode_locked(input);
	if (ret) {
		input->feed_count--;
		if (wildcard) {
			input->all_pid_count--;
		} else if (test_bit(feed->index, input->software_feeds)) {
			__clear_bit(feed->index, input->software_feeds);
			input->software_feed_count--;
		} else {
			histb_dmx_put_channel_locked(input, feed->pid);
		}
	}

unlock:
	mutex_unlock(&dmx->ca_lock);
	mutex_unlock(&input->lock);

	return ret;
}

static int histb_dmx_stop_feed(struct dvb_demux_feed *feed)
{
	struct histb_dmx_input *input = feed->demux->priv;
	struct histb_dmx *dmx = input->dmx;
	bool wildcard = feed->pid == DMX_MAX_PID;
	int ret = 0;

	mutex_lock(&input->lock);
	if (WARN_ON(!input->feed_count)) {
		mutex_unlock(&input->lock);
		return -EINVAL;
	}

	mutex_lock(&dmx->ca_lock);
	/* Keep the final PID channels live until the record path is flushed. */
	if (input->feed_count == 1 && input->running &&
	    histb_dmx_stop_stream(input)) {
		ret = -EIO;
		goto unlock;
	}

	if (wildcard) {
		if (WARN_ON(!input->all_pid_count))
			ret = -EINVAL;
		else
			input->all_pid_count--;
	} else if (test_bit(feed->index, input->software_feeds)) {
		__clear_bit(feed->index, input->software_feeds);
		if (WARN_ON(!input->software_feed_count))
			ret = -EINVAL;
		else
			input->software_feed_count--;
	} else {
		histb_dmx_log_channel_state_locked(input, feed->pid);
		ret = histb_dmx_put_channel_locked(input, feed->pid);
	}
	if (ret)
		goto unlock;

	input->feed_count--;
	if (!input->feed_count) {
		if (!input->flush_faulted) {
			input->queue_faulted = false;
			input->queue_recovery_attempts = 0;
		}
	} else if (input->running) {
		if (histb_dmx_update_stream_mode_locked(input))
			ret = -EIO;
	}

unlock:
	mutex_unlock(&dmx->ca_lock);
	mutex_unlock(&input->lock);

	return ret;
}

static void histb_dmx_deliver(struct histb_dmx_input *input,
			      struct histb_ts_buffer *buffer, u32 offset, u32 len)
{
	if (!len)
		return;
	input->bytes_done += len;
	dvb_dmx_swfilter_packets(&input->demux, buffer->cpu + offset,
				len / HISTB_TS_PACKET_SIZE);
}

/*
 * BSP DMXOsiOQGetBbsAddrSize exposes the committed EOP prefix while the
 * producer still owns the current block. Never return it to FQ early.
 * Recheck the producer index so a block switch cannot mix address/length.
 */
static void histb_dmx_process_partial(struct histb_dmx_input *input)
{
	struct histb_dmx *dmx = input->dmx;
	struct histb_ts_buffer *buffer;
	u32 ctrl, addr, block_len, len, write;

	if (!input->selected_pid_mode)
		return;
	write = FIELD_GET(HISTB_OQ_WRITE_PTR,
			 readl(dmx->regs + HISTB_OQ_WORD5(input->oq_id)));
	if (write != input->oq_read)
		return;
	ctrl = readl(dmx->regs + HISTB_OQ_WORD1(input->oq_id));
	addr = readl(dmx->regs + HISTB_OQ_WORD4(input->oq_id));
	block_len = readl(dmx->regs + HISTB_OQ_WORD3(input->oq_id)) >> 16;
	len = readl(dmx->regs + HISTB_OQ_WORD2(input->oq_id)) & 0xffff;
	if (write != FIELD_GET(HISTB_OQ_WRITE_PTR,
			      readl(dmx->regs + HISTB_OQ_WORD5(input->oq_id))) ||
	    !(ctrl & HISTB_OQ_BUFFER_VALID) || !block_len ||
	    block_len > HISTB_TS_BLOCK_SIZE || len > block_len ||
	    len <= input->partial_len || len % HISTB_TS_PACKET_SIZE ||
	    (input->partial_len && addr != input->partial_addr))
		return;
	buffer = histb_dmx_find_buffer(input, addr);
	if (!buffer)
		return;
	dma_rmb();
	histb_dmx_deliver(input, buffer, input->partial_len, len - input->partial_len);
	input->partial_addr = addr;
	input->partial_len = len;
	input->partial_reads++;
}

static void histb_dmx_process_input(struct histb_dmx_input *input, bool poll)
{
	struct histb_dmx *dmx = input->dmx;
	u32 value;
	u32 oq_write;
	unsigned int done = 0;

	mutex_lock(&input->lock);
	if (!input->running || (poll && !input->selected_pid_mode))
		goto unlock;

	if (!poll)
		input->irq_events++;
	value = readl(dmx->regs + HISTB_OQ_WORD5(input->oq_id));
	oq_write = FIELD_GET(HISTB_OQ_WRITE_PTR, value);
	if (oq_write >= input->oq_depth) {
		input->invalid_desc++;
		dev_warn_ratelimited(dmx->dev,
				     "input %u invalid OQ write pointer %u/%u\n",
				     input->port, oq_write, input->oq_depth);
		mutex_lock(&dmx->ca_lock);
		input->queue_recovery_attempts++;
		if (!histb_dmx_stop_stream(input) &&
		    input->queue_recovery_attempts <=
		    HISTB_QUEUE_RECOVERY_LIMIT) {
			input->queue_recoveries++;
			histb_dmx_start_stream(input);
		} else {
			input->queue_faulted = true;
			input->queue_recovery_failures++;
		}
		mutex_unlock(&dmx->ca_lock);
		goto unlock;
	}
	while (input->oq_read != oq_write && done++ < input->oq_depth) {
		struct histb_oq_desc *oq = &input->oq_desc[input->oq_read];
		struct histb_fq_desc *fq = &input->fq_desc[input->fq_write];
		struct histb_ts_buffer *buffer;
		u32 addr;
		u32 block_len;
		u32 data_len;

		dma_rmb();
		addr = le32_to_cpu(oq->start_addr);
		block_len = le32_to_cpu(oq->ca_ctrl_buf_len) & 0xffff;
		data_len = le32_to_cpu(oq->pvr_ctrl_data_len) & 0xffff;
		buffer = histb_dmx_find_buffer(input, addr);

		if (!buffer || block_len > HISTB_TS_BLOCK_SIZE ||
		    data_len > block_len ||
		    data_len % HISTB_TS_PACKET_SIZE ||
		    (input->partial_len &&
		     (addr != input->partial_addr || data_len < input->partial_len))) {
			input->invalid_desc++;
			if (data_len % HISTB_TS_PACKET_SIZE)
				input->misaligned_desc++;
			dev_warn_ratelimited(dmx->dev,
					     "input %u invalid descriptor %08x/%u/%u\n",
					     input->port, addr, block_len,
					     data_len);
			mutex_lock(&dmx->ca_lock);
			input->queue_recovery_attempts++;
			if (!histb_dmx_stop_stream(input) &&
			    input->queue_recovery_attempts <=
			    HISTB_QUEUE_RECOVERY_LIMIT) {
				input->queue_recoveries++;
				histb_dmx_start_stream(input);
			} else {
				input->queue_faulted = true;
				input->queue_recovery_failures++;
			}
			mutex_unlock(&dmx->ca_lock);
			break;
		}

		histb_dmx_deliver(input, buffer, input->partial_len,
				  data_len - input->partial_len);
		input->partial_addr = 0;
		input->partial_len = 0;
		input->blocks_done++;

		fq->start_addr = cpu_to_le32(addr);
		fq->buf_len = cpu_to_le32(buffer ? HISTB_TS_BLOCK_SIZE : 0);
		dma_wmb();

		input->oq_read++;
		if (input->oq_read == input->oq_depth)
			input->oq_read = 0;
		input->fq_write++;
		if (input->fq_write == input->fq_depth)
			input->fq_write = 0;

		writel(input->oq_read << 16,
		       dmx->regs + HISTB_OQ_WORD5(input->oq_id));
		writel(input->fq_write,
		       dmx->regs + HISTB_FQ_WORD2(input->fq_id));
		value = readl(dmx->regs + HISTB_OQ_WORD5(input->oq_id));
		oq_write = FIELD_GET(HISTB_OQ_WRITE_PTR, value);
	}
	if (input->running)
		histb_dmx_process_partial(input);

unlock:
	mutex_unlock(&input->lock);
}

static void histb_dmx_partial_work(struct work_struct *work)
{
	struct histb_dmx *dmx = container_of(to_delayed_work(work),
					   struct histb_dmx, partial_work);
	unsigned int port;

	for (port = 0; port < HISTB_DMX_MAX_INPUTS; port++)
		if (dmx->inputs[port].dmx)
			histb_dmx_process_input(&dmx->inputs[port], true);
	schedule_delayed_work(&dmx->partial_work,
			      msecs_to_jiffies(HISTB_PARTIAL_POLL_MS));
}

static irqreturn_t histb_dmx_irq(int irq, void *data)
{
	struct histb_dmx *dmx = data;
	unsigned long flags;
	u32 orphan = 0;
	bool wake_thread;

	spin_lock_irqsave(&dmx->reg_lock, flags);
	if (!histb_dmx_irq_owned_locked(dmx)) {
		spin_unlock_irqrestore(&dmx->reg_lock, flags);
		return IRQ_NONE;
	}

	histb_dmx_set_pvr_irq_locked(dmx, false);
	histb_dmx_capture_fq_locked(dmx);
	histb_dmx_capture_oq_locked(dmx, false, &orphan);
	wake_thread = dmx->irq_live &&
		      (dmx->active_oqs || dmx->fq_pending);
	spin_unlock_irqrestore(&dmx->reg_lock, flags);

	if (orphan)
		dev_warn_ratelimited(dmx->dev,
				     "disabled and acknowledged inactive OQ IRQs %08x\n",
				     orphan);

	return wake_thread ? IRQ_WAKE_THREAD : IRQ_HANDLED;
}

static irqreturn_t histb_dmx_irq_thread(int irq, void *data)
{
	struct histb_dmx *dmx = data;
	unsigned int pass;

	for (pass = 0; pass < HISTB_IRQ_DRAIN_PASSES; pass++) {
		unsigned long flags;
		unsigned int port;
		bool last = pass == HISTB_IRQ_DRAIN_PASSES - 1;
		bool more;
		bool stuck = false;
		u32 deferred = 0;
		u32 fq_pending;
		u32 fq_overflow;
		u32 orphan = 0;
		u32 pending;
		u32 rearm;

		spin_lock_irqsave(&dmx->reg_lock, flags);
		pending = dmx->irq_pending;
		dmx->irq_pending = 0;
		fq_pending = dmx->fq_pending;
		dmx->fq_pending = 0;
		fq_overflow = dmx->fq_overflow_pending;
		dmx->fq_overflow_pending = 0;
		spin_unlock_irqrestore(&dmx->reg_lock, flags);

		for (port = 0; port < HISTB_DMX_MAX_INPUTS; port++) {
			if ((pending | fq_pending) & BIT(port)) {
				if (fq_pending & BIT(port)) {
					if (fq_overflow & BIT(port))
						dmx->inputs[port].fq_overflows++;
					else
						dmx->inputs[port].fq_almost_full++;
				}
				histb_dmx_process_input(&dmx->inputs[port], false);
			}
		}

		spin_lock_irqsave(&dmx->reg_lock, flags);
		histb_dmx_capture_oq_locked(dmx, last, &orphan);
		/* Leave newly asserted FQ status pending for a clean retrigger. */
		if (!last)
			histb_dmx_capture_fq_locked(dmx);
		rearm = pending & dmx->active_oqs & ~dmx->irq_pending;
		histb_dmx_set_oq_irq_locked(dmx, rearm, rearm);

		if (!dmx->irq_pending && !dmx->fq_pending &&
		    !pending && !fq_pending &&
		    histb_dmx_irq_owned_locked(dmx))
			histb_dmx_capture_oq_locked(dmx, last, &orphan);

		for (port = 0; port < HISTB_DMX_MAX_INPUTS; port++) {
			struct histb_dmx_input *input = &dmx->inputs[port];

			if (!(fq_pending & BIT(port)) || !input->running)
				continue;
			histb_dmx_set_fq_irq_locked(dmx, BIT(input->fq_id),
						    BIT(input->fq_id));
		}

		more = dmx->irq_pending != 0 || dmx->fq_pending != 0;
		if (last && more) {
			deferred = dmx->irq_pending & dmx->active_oqs;
			dmx->irq_pending = 0;
			if (dmx->irq_live && dmx->active_oqs) {
				histb_dmx_set_oq_irq_locked(dmx, deferred, deferred);
				histb_dmx_set_pvr_irq_locked(dmx, true);
			}
		} else if (!more) {
			if (!pending && histb_dmx_irq_owned_locked(dmx))
				stuck = dmx->irq_live && dmx->active_oqs;
			else if (dmx->irq_live && dmx->active_oqs)
				histb_dmx_set_pvr_irq_locked(dmx, true);
		}
		spin_unlock_irqrestore(&dmx->reg_lock, flags);

		if (orphan)
			dev_warn_ratelimited(dmx->dev,
					     "disabled and acknowledged inactive OQ IRQs %08x\n",
					     orphan);
		if (stuck)
			dev_err_ratelimited(dmx->dev,
					    "PVR interrupt remained asserted without OQ status; "
					    "global gate left masked\n");
		if (last && more)
			dev_warn_ratelimited(dmx->dev,
					     "IRQ drain budget exhausted; rearmed OQs %08x\n",
					     deferred);

		if (!more || last)
			break;
	}

	return IRQ_HANDLED;
}

static int histb_dmx_alloc_dma(struct histb_dmx_input *input)
{
	struct device *dev = input->dmx->dev;
	unsigned int i;

	for (i = 0; i < input->block_count; i++) {
		struct histb_ts_buffer *buffer = &input->buffers[i];

		buffer->cpu = dmam_alloc_coherent(dev, HISTB_TS_BLOCK_SIZE,
						  &buffer->dma, GFP_KERNEL);
		if (!buffer->cpu)
			return -ENOMEM;
	}

	input->fq_desc = dmam_alloc_coherent(dev,
					     sizeof(*input->fq_desc) * input->fq_depth,
					     &input->fq_dma, GFP_KERNEL);
	if (!input->fq_desc)
		return -ENOMEM;

	input->oq_desc = dmam_alloc_coherent(dev,
					     sizeof(*input->oq_desc) * input->oq_depth,
					     &input->oq_dma, GFP_KERNEL);
	if (!input->oq_desc)
		return -ENOMEM;

	return 0;
}

static void histb_dmx_put_i2c_client(void *data)
{
	struct i2c_client *client = data;

	put_device(&client->dev);
}

static int histb_dmx_parse_frontend(struct histb_dmx_input *input,
				    struct device_node *np)
{
	struct device *dev = input->dmx->dev;
	struct device_node *endpoint;
	struct device_node *frontend_node;
	struct device_node *remote_endpoint;
	struct i2c_client *client;
	struct dvb_frontend *frontend;
	struct of_endpoint remote = { };
	int ret;

	endpoint = of_graph_get_next_endpoint(np, NULL);
	if (!endpoint)
		return 0;

	remote_endpoint = of_graph_get_remote_endpoint(endpoint);
	of_node_put(endpoint);
	if (!remote_endpoint)
		return dev_err_probe(dev, -EINVAL,
				     "%pOF: missing remote frontend endpoint\n", np);

	ret = of_graph_parse_endpoint(remote_endpoint, &remote);
	frontend_node = of_graph_get_port_parent(remote_endpoint);
	of_node_put(remote_endpoint);
	if (ret || !frontend_node) {
		of_node_put(frontend_node);
		return dev_err_probe(dev, ret ? ret : -EINVAL,
				     "%pOF: invalid remote frontend endpoint\n",
				     np);
	}

	if (!of_device_is_compatible(frontend_node, "maxlinear,mxl214")) {
		ret = dev_err_probe(dev, -ENODEV,
				    "%pOF: unsupported frontend %pOF\n",
				    np, frontend_node);
		of_node_put(frontend_node);
		return ret;
	}

	client = of_find_i2c_device_by_node(frontend_node);
	of_node_put(frontend_node);
	if (!client)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "%pOF: frontend device is not ready\n", np);

	frontend = mxl214_get_frontend(client, remote.port);
	if (IS_ERR(frontend)) {
		ret = dev_err_probe(dev, PTR_ERR(frontend),
				    "%pOF: frontend channel %u is not ready\n",
				    np, remote.port);
		put_device(&client->dev);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, histb_dmx_put_i2c_client, client);
	if (ret)
		return ret;

	input->frontend_client = client;
	input->frontend = frontend;

	return 0;
}

static void histb_dmx_unregister_input(struct histb_dmx_input *input)
{
	if (input->frontend_registered)
		dvb_unregister_frontend(input->frontend);
	if (input->mem_frontend_registered)
		input->demux.dmx.remove_frontend(&input->demux.dmx,
						  &input->mem_frontend);
	if (input->hw_frontend_registered)
		input->demux.dmx.remove_frontend(&input->demux.dmx,
						  &input->hw_frontend);
	while (input->ca_registered) {
		input->ca_registered--;
		dvb_unregister_device(input->ca_devs[input->ca_registered]);
	}
	if (input->dmxdev_registered)
		dvb_dmxdev_release(&input->dmxdev);
	if (input->demux_registered)
		dvb_dmx_release(&input->demux);
	if (input->adapter_registered)
		dvb_unregister_adapter(&input->adapter);
}

static int histb_dmx_register_input(struct histb_dmx_input *input)
{
	struct device *dev = input->dmx->dev;
	unsigned int id;
	char *name;
	int ret;

	name = devm_kasprintf(dev, GFP_KERNEL, "HiSilicon HiSTB TS input %u",
			      input->port);
	if (!name)
		return -ENOMEM;

	ret = dvb_register_adapter(&input->adapter, name, THIS_MODULE, dev,
				   adapter_nr);
	if (ret < 0)
		return ret;
	input->adapter_registered = true;
	input->adapter.priv = input;

	/*
	 * Tvheadend encodes its adapter number as OSCam's CA mask.  Provide
	 * ca0..ca3 on every adapter so unmodified local DVBAPI opens the
	 * matching /dev/dvb/adapterN/caN node.
	 */
	for (id = 0; id < HISTB_DMX_CA_ALIASES; id++) {
		ret = dvb_register_device(&input->adapter,
					  &input->ca_devs[id],
					  &histb_dmx_ca_device, input,
					  DVB_DEVICE_CA, 0);
		if (ret)
			goto error;
		input->ca_registered++;
	}

	input->demux.dmx.capabilities = DMX_TS_FILTERING |
					DMX_SECTION_FILTERING |
					DMX_MEMORY_BASED_FILTERING;
	input->demux.priv = input;
	input->demux.filternum = HISTB_DMX_MAX_FILTERS;
	input->demux.feednum = HISTB_DMX_MAX_FILTERS;
	input->demux.start_feed = histb_dmx_start_feed;
	input->demux.stop_feed = histb_dmx_stop_feed;

	ret = dvb_dmx_init(&input->demux);
	if (ret)
		goto error;
	input->demux_registered = true;

	input->dmxdev.filternum = HISTB_DMX_MAX_FILTERS;
	input->dmxdev.demux = &input->demux.dmx;
	ret = dvb_dmxdev_init(&input->dmxdev, &input->adapter);
	if (ret)
		goto error;
	input->dmxdev_registered = true;

	input->hw_frontend.source = DMX_FRONTEND_0;
	ret = input->demux.dmx.add_frontend(&input->demux.dmx,
					    &input->hw_frontend);
	if (ret)
		goto error;
	input->hw_frontend_registered = true;

	input->mem_frontend.source = DMX_MEMORY_FE;
	ret = input->demux.dmx.add_frontend(&input->demux.dmx,
					    &input->mem_frontend);
	if (ret)
		goto error;
	input->mem_frontend_registered = true;

	ret = input->demux.dmx.connect_frontend(&input->demux.dmx,
						&input->hw_frontend);
	if (ret)
		goto error;

	if (input->frontend) {
		ret = dvb_register_frontend(&input->adapter, input->frontend);
		if (ret)
			goto error;
		input->frontend_registered = true;
	}

	return 0;

error:
	histb_dmx_unregister_input(input);
	return ret;
}

static int histb_dmx_parse_inputs(struct histb_dmx *dmx)
{
	struct device *dev = dmx->dev;
	struct device_node *child;

	for_each_available_child_of_node(dev->of_node, child) {
		struct histb_dmx_input *input;
		char clk_name[8];
		u32 port;
		int ret;

		ret = of_property_read_u32(child, "reg", &port);
		if (ret || port >= HISTB_DMX_MAX_INPUTS) {
			ret = dev_err_probe(dev, ret ? ret : -EINVAL,
					    "%pOF: invalid input number\n", child);
			of_node_put(child);
			return ret;
		}

		input = &dmx->inputs[port];
		if (input->dmx) {
			ret = dev_err_probe(dev, -EINVAL,
					    "%pOF: duplicate input %u\n",
					    child, port);
			of_node_put(child);
			return ret;
		}

		input->dmx = dmx;
		input->port = port;
		input->fq_id = HISTB_FQ_REC_BASE + port;
		input->oq_id = port;
		input->rec_id = port;
		memset(input->ca_pid_slot, -1, sizeof(input->ca_pid_slot));
		memset(input->ca_slot_hw, -1, sizeof(input->ca_slot_hw));
		mutex_init(&input->lock);

		ret = histb_dmx_parse_mode(dev, child, input);
		if (ret) {
			of_node_put(child);
			return ret;
		}
		ret = histb_dmx_parse_frontend(input, child);
		if (ret) {
			of_node_put(child);
			return ret;
		}

		snprintf(clk_name, sizeof(clk_name), "tsi%u", port);
		input->clk = devm_clk_get(dev, clk_name);
		if (IS_ERR(input->clk)) {
			ret = dev_err_probe(dev, PTR_ERR(input->clk),
					    "failed to get %s clock\n", clk_name);
			of_node_put(child);
			return ret;
		}

		ret = clk_set_phase(input->clk,
				    input->clock_inverted ? 180 : 0);
		if (ret) {
			ret = dev_err_probe(dev, ret,
					    "failed to set %s phase\n", clk_name);
			of_node_put(child);
			return ret;
		}

		ret = histb_dmx_alloc_dma(input);
		if (ret) {
			ret = dev_err_probe(dev, ret,
					    "failed to allocate input %u DMA\n",
					    port);
			of_node_put(child);
			return ret;
		}

		dmx->input_count++;
	}

	if (!dmx->input_count)
		return dev_err_probe(dev, -ENODEV, "no enabled TS inputs\n");

	return 0;
}

static void histb_dmx_disable_clocks(struct histb_dmx *dmx)
{
	int port;

	for (port = HISTB_DMX_MAX_INPUTS - 1; port >= 0; port--)
		if (dmx->inputs[port].dmx)
			clk_disable_unprepare(dmx->inputs[port].clk);
	clk_bulk_disable_unprepare(ARRAY_SIZE(dmx->core_clks),
				   dmx->core_clks);
}

static int histb_dmx_enable_clocks(struct histb_dmx *dmx)
{
	unsigned int port;
	int ret;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(dmx->core_clks),
				      dmx->core_clks);
	if (ret)
		return ret;

	for (port = 0; port < HISTB_DMX_MAX_INPUTS; port++) {
		if (!dmx->inputs[port].dmx)
			continue;

		ret = clk_prepare_enable(dmx->inputs[port].clk);
		if (ret)
			goto disable_inputs;
	}

	return 0;

disable_inputs:
	while (port--)
		if (dmx->inputs[port].dmx)
			clk_disable_unprepare(dmx->inputs[port].clk);
	clk_bulk_disable_unprepare(ARRAY_SIZE(dmx->core_clks),
				   dmx->core_clks);

	return ret;
}

static int histb_dmx_hw_initialize(struct histb_dmx *dmx)
{
	u32 status;
	unsigned int port;
	int ret;

	ret = reset_control_assert(dmx->reset);
	if (ret)
		return ret;
	udelay(1);
	ret = reset_control_deassert(dmx->reset);
	if (ret)
		return ret;

	/* Do not inherit Fastboot interrupt ownership across the reset handoff. */
	histb_dmx_mask_all_interrupts(dmx);
	ret = readl_poll_timeout(dmx->regs + HISTB_FQ_INIT_DONE, status,
				 !(status & BIT(0)) &&
				 !(readl(dmx->regs + HISTB_OQ_INIT_DONE) & BIT(0)),
				 10, USEC_PER_SEC);
	if (ret)
		return ret;

	ret = histb_dmx_init_original(dmx);
	if (ret)
		return ret;
	dmx->clear_engine_faulted = false;

	dmx->ca_info = readl(dmx->regs + HISTB_DMX_CA_INFO0);
	dmx->normal_csa2 = !(dmx->ca_info &
				(HISTB_DMX_CA_HARDONLY_CSA2 |
				 HISTB_DMX_CA_DISABLE_CSA2));

	for (port = 0; port < HISTB_DMX_MAX_INPUTS; port++) {
		if (!dmx->inputs[port].dmx)
			continue;
		histb_dmx_configure_port(&dmx->inputs[port]);
		histb_dmx_route_input(&dmx->inputs[port]);
	}

	return 0;
}

static int histb_dmx_restore_input_locked(struct histb_dmx_input *input)
{
	struct histb_dmx *dmx = input->dmx;
	unsigned int index;
	unsigned int id;

	for (index = 0; index < HISTB_DMX_CA_SLOTS; index++) {
		int slot = input->ca_slot_hw[index];

		if (slot < 0)
			continue;
		if (!dmx->normal_csa2)
			return -EOPNOTSUPP;
		histb_dmx_clear_cw_locked(dmx, slot);
		histb_dmx_update_bits(dmx, HISTB_DMX_CA_ENTROPY, BIT(slot), 0);
		if (input->ca_cw_valid[index][0])
			histb_dmx_write_cw_locked(dmx, slot, 0,
						  input->ca_cw[index][0]);
		if (input->ca_cw_valid[index][1])
			histb_dmx_write_cw_locked(dmx, slot, 1,
						  input->ca_cw[index][1]);
	}

	for (id = 0; id < HISTB_DMX_HW_CHANNELS; id++) {
		struct histb_dmx_channel *channel = &dmx->channels[id];

		if (channel->used && channel->input == input->port)
			histb_dmx_program_channel_locked(input, id, channel->pid);
	}

	input->record_enabled = false;
	input->selected_pid_mode = false;
	input->descrambled_record_mode = false;
	if (input->feed_count)
		histb_dmx_start_stream(input);

	return 0;
}

static void histb_dmx_disable_interrupts(struct histb_dmx *dmx)
{
	histb_dmx_set_irq_live(dmx, false);
}

static int histb_dmx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_dmx *dmx;
	unsigned int port;
	int ret;

	dmx = devm_kzalloc(dev, sizeof(*dmx), GFP_KERNEL);
	if (!dmx)
		return -ENOMEM;

	dmx->dev = dev;
	INIT_DELAYED_WORK(&dmx->partial_work, histb_dmx_partial_work);
	spin_lock_init(&dmx->reg_lock);
	mutex_init(&dmx->ca_lock);
	dmx->irq_pending = 0;
	dmx->active_oqs = 0;
	platform_set_drvdata(pdev, dmx);

	dmx->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dmx->regs))
		return PTR_ERR(dmx->regs);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "32-bit DMA is unavailable\n");

	dmx->core_clks[0].id = "bus";
	dmx->core_clks[1].id = "core";
	dmx->core_clks[2].id = "ref";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(dmx->core_clks),
				dmx->core_clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get core clocks\n");

	dmx->reset = devm_reset_control_get_exclusive(dev, "demux");
	if (IS_ERR(dmx->reset))
		return dev_err_probe(dev, PTR_ERR(dmx->reset),
				     "failed to get reset\n");

	ret = histb_dmx_parse_inputs(dmx);
	if (ret)
		return ret;

	ret = histb_dmx_enable_clocks(dmx);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");

	ret = histb_dmx_hw_initialize(dmx);
	if (ret) {
		dev_err(dev, "demux hardware initialization failed: %d\n", ret);
		goto assert_reset;
	}

	dev_info(dev, "CV200 CA: normal CSA2 clear-CW %s (CA_INFO0=%08x)\n",
		 dmx->normal_csa2 ? "available" : "unavailable",
		 dmx->ca_info);

	dmx->irq = platform_get_irq(pdev, 0);
	if (dmx->irq < 0) {
		ret = dmx->irq;
		goto assert_reset;
	}

	ret = devm_request_threaded_irq(dev, dmx->irq, histb_dmx_irq,
					histb_dmx_irq_thread,
					IRQF_SHARED | IRQF_ONESHOT,
					dev_name(dev), dmx);
	if (ret) {
		ret = dev_err_probe(dev, ret, "failed to request IRQ\n");
		goto assert_reset;
	}

	histb_dmx_set_irq_live(dmx, true);

	for (port = 0; port < HISTB_DMX_MAX_INPUTS; port++) {
		if (!dmx->inputs[port].dmx)
			continue;

		ret = histb_dmx_register_input(&dmx->inputs[port]);
		if (ret) {
			dev_err(dev, "failed to register input %u: %d\n", port,
				ret);
			goto unregister_inputs;
		}
	}

	dev_info(dev,
		 "registered %u transport stream inputs (PID copy enabled, mode0=%08x)\n",
		 dmx->input_count, readl(dmx->regs + HISTB_DMX_MODE0));

	schedule_delayed_work(&dmx->partial_work,
			      msecs_to_jiffies(HISTB_PARTIAL_POLL_MS));
	return 0;

unregister_inputs:
	histb_dmx_disable_interrupts(dmx);
	synchronize_irq(dmx->irq);
	while (port--)
		if (dmx->inputs[port].dmx)
			histb_dmx_unregister_input(&dmx->inputs[port]);
assert_reset:
	reset_control_assert(dmx->reset);
	histb_dmx_disable_clocks(dmx);

	return ret;
}

static int __maybe_unused histb_dmx_suspend(struct device *dev)
{
	struct histb_dmx *dmx = dev_get_drvdata(dev);
	unsigned int port;
	int ret = 0;

	cancel_delayed_work_sync(&dmx->partial_work);
	histb_dmx_disable_interrupts(dmx);
	synchronize_irq(dmx->irq);

	for (port = 0; port < HISTB_DMX_MAX_INPUTS; port++) {
		struct histb_dmx_input *input = &dmx->inputs[port];

		if (!input->dmx)
			continue;
		mutex_lock(&input->lock);
		mutex_lock(&dmx->ca_lock);
		if (input->running)
			ret = histb_dmx_stop_stream(input);
		mutex_unlock(&dmx->ca_lock);
		mutex_unlock(&input->lock);
		if (ret)
			goto restore_streams;
	}

	ret = reset_control_assert(dmx->reset);
	if (ret)
		goto restore_streams;
	histb_dmx_disable_clocks(dmx);

	return 0;

restore_streams:
	for (port = 0; port < HISTB_DMX_MAX_INPUTS; port++) {
		struct histb_dmx_input *input = &dmx->inputs[port];

		if (!input->dmx || !input->feed_count || input->running ||
		    input->queue_faulted)
			continue;
		mutex_lock(&input->lock);
		mutex_lock(&dmx->ca_lock);
		histb_dmx_start_stream(input);
		mutex_unlock(&dmx->ca_lock);
		mutex_unlock(&input->lock);
	}
	histb_dmx_set_irq_live(dmx, true);

	schedule_delayed_work(&dmx->partial_work,
			      msecs_to_jiffies(HISTB_PARTIAL_POLL_MS));
	return ret;
}

static int __maybe_unused histb_dmx_resume(struct device *dev)
{
	struct histb_dmx *dmx = dev_get_drvdata(dev);
	unsigned int port;
	int ret;

	ret = histb_dmx_enable_clocks(dmx);
	if (ret)
		return ret;

	ret = histb_dmx_hw_initialize(dmx);
	if (ret)
		goto disable_hardware;

	for (port = 0; port < HISTB_DMX_MAX_INPUTS; port++) {
		struct histb_dmx_input *input = &dmx->inputs[port];

		if (!input->dmx)
			continue;
		mutex_lock(&input->lock);
		mutex_lock(&dmx->ca_lock);
		ret = input->queue_faulted ? -EIO :
			histb_dmx_restore_input_locked(input);
		mutex_unlock(&dmx->ca_lock);
		mutex_unlock(&input->lock);
		if (ret)
			goto disable_hardware;
	}

	histb_dmx_set_irq_live(dmx, true);
	schedule_delayed_work(&dmx->partial_work,
			      msecs_to_jiffies(HISTB_PARTIAL_POLL_MS));
	return 0;

disable_hardware:
	histb_dmx_mask_all_interrupts(dmx);
	reset_control_assert(dmx->reset);
	for (port = 0; port < HISTB_DMX_MAX_INPUTS; port++) {
		if (!dmx->inputs[port].dmx)
			continue;
		dmx->inputs[port].running = false;
		dmx->inputs[port].record_enabled = false;
	}
	histb_dmx_disable_clocks(dmx);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(histb_dmx_pm_ops, histb_dmx_suspend,
				histb_dmx_resume);

static void histb_dmx_remove(struct platform_device *pdev)
{
	struct histb_dmx *dmx = platform_get_drvdata(pdev);
	unsigned int port;

	cancel_delayed_work_sync(&dmx->partial_work);
	histb_dmx_disable_interrupts(dmx);
	synchronize_irq(dmx->irq);

	for (port = 0; port < HISTB_DMX_MAX_INPUTS; port++) {
		struct histb_dmx_input *input = &dmx->inputs[port];

		if (!input->dmx)
			continue;
		mutex_lock(&input->lock);
		mutex_lock(&dmx->ca_lock);
		if (input->running)
			histb_dmx_stop_stream(input);
		mutex_unlock(&dmx->ca_lock);
		mutex_unlock(&input->lock);
	}

	for (port = 0; port < HISTB_DMX_MAX_INPUTS; port++)
		if (dmx->inputs[port].dmx)
			histb_dmx_unregister_input(&dmx->inputs[port]);

	reset_control_assert(dmx->reset);
	histb_dmx_disable_clocks(dmx);
}

static const struct of_device_id histb_dmx_of_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-demux" },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_dmx_of_match);

static struct platform_driver histb_dmx_driver = {
	.probe = histb_dmx_probe,
	.remove_new = histb_dmx_remove,
	.driver = {
		.name = "histb-dmx",
		.of_match_table = histb_dmx_of_match,
		.pm = pm_sleep_ptr(&histb_dmx_pm_ops),
	},
};
module_platform_driver(histb_dmx_driver);

MODULE_AUTHOR("HiSilicon Technologies Co., Ltd.");
MODULE_DESCRIPTION("HiSilicon HiSTB transport stream demultiplexer");
MODULE_LICENSE("GPL");
