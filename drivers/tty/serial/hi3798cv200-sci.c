// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3798CV200 ISO/IEC 7816 smart card interface
 *
 * Copyright (C) 2026 HiSilicon Technologies Co., Ltd.
 */

#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/bitrev.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/sysfs.h>
#include <linux/tty_flip.h>
#include <linux/workqueue.h>

#define HISTB_SCI_DATA			0x00
#define HISTB_SCI_CR0			0x04
#define HISTB_SCI_CR1			0x08
#define HISTB_SCI_CR2			0x0c
#define HISTB_SCI_CLKICC		0x10
#define HISTB_SCI_ETU			0x14
#define HISTB_SCI_BAUD			0x18
#define HISTB_SCI_TIDE			0x1c
#define HISTB_SCI_DMACR			0x20
#define HISTB_SCI_STABLE		0x24
#define HISTB_SCI_ATIME			0x28
#define HISTB_SCI_DTIME			0x2c
#define HISTB_SCI_ATRSTIME		0x30
#define HISTB_SCI_ATRDTIME		0x34
#define HISTB_SCI_STOPTIME		0x38
#define HISTB_SCI_STARTTIME		0x3c
#define HISTB_SCI_RETRY			0x40
#define HISTB_SCI_CHTIMELS		0x44
#define HISTB_SCI_CHTIMEMS		0x48
#define HISTB_SCI_BLKTIMELS		0x4c
#define HISTB_SCI_BLKTIMEMS		0x50
#define HISTB_SCI_CHGUARD		0x54
#define HISTB_SCI_BLKGUARD		0x58
#define HISTB_SCI_RXTIME		0x5c
#define HISTB_SCI_TXCOUNT		0x64
#define HISTB_SCI_RXCOUNT		0x68
#define HISTB_SCI_IMSC			0x6c
#define HISTB_SCI_MIS			0x74
#define HISTB_SCI_ICR			0x78
#define HISTB_SCI_SYNCACT		0x7c

#define HISTB_SCI_DATA_VALUE		GENMASK(7, 0)
#define HISTB_SCI_DATA_PARITY		BIT(8)

#define HISTB_SCI_CR0_SENSE		BIT(0)
#define HISTB_SCI_CR0_ORDER		BIT(1)
#define HISTB_SCI_CR0_TX_NAK		BIT(3)
#define HISTB_SCI_CR0_RX_NAK		BIT(5)
#define HISTB_SCI_CR0_PARITY_ENABLE	BIT(8)
#define HISTB_SCI_CR0_VCC_ENABLE_INV	BIT(9)
#define HISTB_SCI_CR0_DETECT_INV	BIT(10)
#define HISTB_SCI_CR0_CONVENTION	(HISTB_SCI_CR0_SENSE | \
					 HISTB_SCI_CR0_ORDER)

#define HISTB_SCI_CR1_ATR_TIMEOUT	BIT(0)
#define HISTB_SCI_CR1_BLOCK_TIMEOUT	BIT(1)
#define HISTB_SCI_CR1_TX_MODE		BIT(2)
#define HISTB_SCI_CR1_CLK_OPEN_DRAIN	BIT(3)
#define HISTB_SCI_CR1_BLOCK_GUARD	BIT(4)
#define HISTB_SCI_CR1_RESET_OPEN_DRAIN	BIT(7)
#define HISTB_SCI_CR1_VCCEN_OPEN_DRAIN	BIT(8)

#define HISTB_SCI_CR2_STARTUP		BIT(0)
#define HISTB_SCI_CR2_FINISH		BIT(1)
#define HISTB_SCI_CR2_WRESET		BIT(2)

#define HISTB_SCI_TIDE_RX_SHIFT		6

#define HISTB_SCI_FIFO_COUNT		GENMASK(5, 0)
#define HISTB_SCI_FIFO_SIZE		32

#define HISTB_SCI_INT_CARD_IN		BIT(0)
#define HISTB_SCI_INT_CARD_OUT		BIT(1)
#define HISTB_SCI_INT_CARD_UP		BIT(2)
#define HISTB_SCI_INT_CARD_DOWN		BIT(3)
#define HISTB_SCI_INT_TX_ERROR		BIT(4)
#define HISTB_SCI_INT_ATR_START_TIMEOUT	BIT(5)
#define HISTB_SCI_INT_ATR_DURATION_TIMEOUT BIT(6)
#define HISTB_SCI_INT_RX_OVERRUN	BIT(10)
#define HISTB_SCI_INT_RX_TIDE		BIT(13)
#define HISTB_SCI_INT_TX_TIDE		BIT(14)
#define HISTB_SCI_INT_CARD		(HISTB_SCI_INT_CARD_IN | \
					 HISTB_SCI_INT_CARD_OUT | \
					 HISTB_SCI_INT_CARD_UP | \
					 HISTB_SCI_INT_CARD_DOWN)
#define HISTB_SCI_INT_ATR_TIMEOUT	(HISTB_SCI_INT_ATR_START_TIMEOUT | \
					 HISTB_SCI_INT_ATR_DURATION_TIMEOUT)
#define HISTB_SCI_INT_RX		(HISTB_SCI_INT_RX_TIDE | \
					 HISTB_SCI_INT_RX_OVERRUN)
#define HISTB_SCI_INT_ALL		GENMASK(14, 0)

#define HISTB_SCI_CARD_PRESENT		BIT(10)

#define HISTB_SCI_CARD_CLOCK_MIN	1000000U
#define HISTB_SCI_CARD_CLOCK_MAX	5000000U
#define HISTB_SCI_ACTIVATION_DELAY_MS	10

#define HISTB_SCI_DEFAULT_STABLE	136
#define HISTB_SCI_DEFAULT_ACTIVE	42500
#define HISTB_SCI_DEFAULT_DEACTIVE	200
#define HISTB_SCI_DEFAULT_ATR_START	40000
#define HISTB_SCI_DEFAULT_ATR_DURATION	19200
#define HISTB_SCI_DEFAULT_STOP		1860
#define HISTB_SCI_DEFAULT_START		700
#define HISTB_SCI_DEFAULT_RETRY		3
#define HISTB_SCI_DEFAULT_CHAR_TIMEOUT	(65535 - 12)
#define HISTB_SCI_DEFAULT_BLOCK_TIMEOUT	(9600 - 12)
#define HISTB_SCI_DEFAULT_BLOCK_GUARD	(22 - 12)
#define HISTB_SCI_DEFAULT_RX_TIMEOUT	25000
#define HISTB_SCI_ATR_MAX_LENGTH	256
#define HISTB_SCI_IRQ_DRAIN_LIMIT	64

enum histb_sci_state {
	HISTB_SCI_STATE_NO_CARD,
	HISTB_SCI_STATE_INACTIVE,
	HISTB_SCI_STATE_WAIT_ATR,
	HISTB_SCI_STATE_READ_ATR,
	HISTB_SCI_STATE_READY,
};

enum histb_sci_atr_stage {
	HISTB_SCI_ATR_TS,
	HISTB_SCI_ATR_T0,
	HISTB_SCI_ATR_INTERFACE,
	HISTB_SCI_ATR_HISTORICAL,
	HISTB_SCI_ATR_TCK,
};

struct histb_sci_port {
	struct uart_port port;
	struct tty_port *tty_port;
	struct clk *clk;
	struct reset_control *reset;
	struct workqueue_struct *rx_workqueue;
	struct delayed_work activate_work;
	u32 imsc;
	u32 cr1_output_mode;
	enum histb_sci_state state;
	enum histb_sci_atr_stage atr_stage;
	u16 atr_length;
	u8 atr[HISTB_SCI_ATR_MAX_LENGTH];
	u16 atr_char_guard;
	u8 atr_interface_mask;
	u8 atr_historical_bytes;
	u8 atr_group;
	u8 atr_protocol;
	u8 atr_checksum;
	bool vcc_active_low;
	bool detect_active_high;
	bool atr_inverse;
	bool atr_protocol_seen;
	bool atr_tck_required;
	bool atr_has_char_guard;
	bool opened;
	bool card_present;
	bool tx_active;
	bool tx_drain_pending;
	atomic64_t irq_count;
	atomic64_t irq_drain_limit_count;
	atomic64_t rx_byte_count;
	atomic64_t tx_byte_count;
	atomic64_t tx_transaction_count;
	atomic64_t tx_rekick_count;
	atomic64_t tx_drain_append_count;
	atomic64_t tx_rx_handoff_count;
	atomic64_t tx_short_handoff_count;
	atomic64_t tx_refill_handoff_count;
	atomic64_t atr_complete_count;
	atomic64_t atr_fail_count;
	atomic64_t atr_start_timeout_count;
	atomic64_t atr_duration_timeout_count;
	atomic64_t tx_error_count;
	atomic64_t rx_overrun_count;
	atomic64_t fifo_clear_count;
	atomic64_t recovery_count;
	u32 last_irq_status;
};

static struct uart_driver histb_sci_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "hi3798cv200-sci",
	.dev_name = "ttySCI",
	.nr = 1,
};

static inline struct histb_sci_port *to_histb_sci(struct uart_port *port)
{
	return container_of(port, struct histb_sci_port, port);
}

static inline u32 histb_sci_read(struct histb_sci_port *sci, u32 reg)
{
	return readl(sci->port.membase + reg);
}

static inline void histb_sci_write(struct histb_sci_port *sci, u32 value,
				   u32 reg)
{
	writel(value, sci->port.membase + reg);
}

static void histb_sci_write_imsc(struct histb_sci_port *sci)
{
	histb_sci_write(sci, sci->imsc, HISTB_SCI_IMSC);
}

static bool histb_sci_card_is_present(struct histb_sci_port *sci)
{
	return histb_sci_read(sci, HISTB_SCI_SYNCACT) &
		HISTB_SCI_CARD_PRESENT;
}

static void histb_sci_set_rx_mode(struct histb_sci_port *sci)
{
	u32 cr1 = histb_sci_read(sci, HISTB_SCI_CR1);

	histb_sci_write(sci, cr1 & ~HISTB_SCI_CR1_TX_MODE, HISTB_SCI_CR1);
}

static void histb_sci_set_tx_mode(struct histb_sci_port *sci)
{
	u32 cr1 = histb_sci_read(sci, HISTB_SCI_CR1);

	histb_sci_write(sci, cr1 | HISTB_SCI_CR1_TX_MODE, HISTB_SCI_CR1);
}

static void histb_sci_set_direct_convention(struct histb_sci_port *sci)
{
	u32 cr0 = histb_sci_read(sci, HISTB_SCI_CR0);

	histb_sci_write(sci, cr0 & ~HISTB_SCI_CR0_CONVENTION,
			HISTB_SCI_CR0);
}

static void histb_sci_clear_fifos(struct histb_sci_port *sci)
{
	histb_sci_write(sci, U32_MAX, HISTB_SCI_TXCOUNT);
	histb_sci_write(sci, U32_MAX, HISTB_SCI_RXCOUNT);
	sci->tx_drain_pending = false;
	atomic64_inc(&sci->fifo_clear_count);
}

static void histb_sci_reset_atr(struct histb_sci_port *sci)
{
	sci->atr_stage = HISTB_SCI_ATR_TS;
	sci->atr_length = 0;
	sci->atr_char_guard = 0;
	sci->atr_interface_mask = 0;
	sci->atr_historical_bytes = 0;
	sci->atr_group = 0;
	sci->atr_protocol = 0;
	sci->atr_checksum = 0;
	sci->atr_inverse = false;
	sci->atr_protocol_seen = false;
	sci->atr_tck_required = false;
	sci->atr_has_char_guard = false;
}

static void histb_sci_update_card_present(struct histb_sci_port *sci,
					  bool present)
{
	if (present == sci->card_present)
		return;

	sci->card_present = present;
	uart_handle_dcd_change(&sci->port, present);
}

static void histb_sci_finish(struct histb_sci_port *sci)
{
	histb_sci_write(sci, HISTB_SCI_CR2_FINISH, HISTB_SCI_CR2);
	histb_sci_reset_atr(sci);
	sci->state = sci->card_present ? HISTB_SCI_STATE_INACTIVE :
					 HISTB_SCI_STATE_NO_CARD;
	sci->tx_active = false;
	sci->tx_drain_pending = false;
}

static void histb_sci_prepare_atr(struct histb_sci_port *sci)
{
	u32 cr0 = histb_sci_read(sci, HISTB_SCI_CR0);
	u32 cr1 = histb_sci_read(sci, HISTB_SCI_CR1);

	histb_sci_reset_atr(sci);
	histb_sci_write(sci, cr0 & ~(HISTB_SCI_CR0_CONVENTION |
				       HISTB_SCI_CR0_TX_NAK |
				       HISTB_SCI_CR0_RX_NAK), HISTB_SCI_CR0);
	histb_sci_write(sci, cr1 | HISTB_SCI_CR1_ATR_TIMEOUT,
			HISTB_SCI_CR1);
	sci->imsc |= HISTB_SCI_INT_ATR_TIMEOUT | HISTB_SCI_INT_RX;
	histb_sci_write_imsc(sci);
	sci->state = HISTB_SCI_STATE_WAIT_ATR;
}

static void histb_sci_schedule_activation(struct histb_sci_port *sci)
{
	histb_sci_finish(sci);
	histb_sci_clear_fifos(sci);
	histb_sci_set_direct_convention(sci);
	histb_sci_write(sci, HISTB_SCI_INT_ALL, HISTB_SCI_ICR);
	mod_delayed_work(system_wq, &sci->activate_work,
			 msecs_to_jiffies(HISTB_SCI_ACTIVATION_DELAY_MS));
}

static void histb_sci_stop_tx(struct uart_port *port);

/*
 * Recover only after an actual SCI error or an explicit userspace request.
 * Normal TX -> RX hand-off uses the smaller original FIFO/direction sequence.
 */
static void histb_sci_recover(struct histb_sci_port *sci, bool rearm)
{
	bool present = histb_sci_card_is_present(sci);

	histb_sci_stop_tx(&sci->port);
	histb_sci_write(sci, 0, HISTB_SCI_IMSC);
	histb_sci_write(sci, HISTB_SCI_INT_ALL, HISTB_SCI_ICR);
	histb_sci_clear_fifos(sci);
	histb_sci_set_rx_mode(sci);
	histb_sci_reset_atr(sci);
	histb_sci_update_card_present(sci, present);
	sci->state = present ? HISTB_SCI_STATE_INACTIVE :
			     HISTB_SCI_STATE_NO_CARD;
	sci->tx_active = false;
	sci->tx_drain_pending = false;
	atomic64_inc(&sci->recovery_count);

	if (rearm && sci->opened &&
	    (sci->port.iso7816.flags & SER_ISO7816_ENABLED) && present) {
		sci->imsc = HISTB_SCI_INT_CARD;
		histb_sci_schedule_activation(sci);
		return;
	}

	sci->imsc = HISTB_SCI_INT_CARD;
	if (sci->opened &&
	    (sci->port.iso7816.flags & SER_ISO7816_ENABLED))
		sci->imsc |= HISTB_SCI_INT_RX;
	histb_sci_write_imsc(sci);
}

static void histb_sci_activate_work(struct work_struct *work)
{
	struct histb_sci_port *sci =
		container_of(to_delayed_work(work), struct histb_sci_port,
			     activate_work);
	struct uart_port *port = &sci->port;
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);
	if (sci->opened &&
	    (port->iso7816.flags & SER_ISO7816_ENABLED) &&
	    histb_sci_card_is_present(sci)) {
		histb_sci_update_card_present(sci, true);
		histb_sci_clear_fifos(sci);
		histb_sci_write(sci, HISTB_SCI_INT_ALL, HISTB_SCI_ICR);
		histb_sci_prepare_atr(sci);
		histb_sci_write(sci, HISTB_SCI_CR2_STARTUP, HISTB_SCI_CR2);
	} else if (!histb_sci_card_is_present(sci)) {
		histb_sci_update_card_present(sci, false);
		sci->state = HISTB_SCI_STATE_NO_CARD;
	}
	uart_port_unlock_irqrestore(port, flags);
}

static void histb_sci_write_timeout(struct histb_sci_port *sci, u32 value,
				    u32 high_reg, u32 low_reg)
{
	histb_sci_write(sci, value >> 16, high_reg);
	histb_sci_write(sci, value & 0xffff, low_reg);
}

static int histb_sci_calculate_timing(struct histb_sci_port *sci,
				      const struct serial_iso7816 *conf,
				      u32 *clkicc, u32 *etu, u32 *baud)
{
	u64 divisor, baud_total, baud_count, card_period;
	u32 etu_value, remainder;

	if (!conf->sc_fi || !conf->sc_di ||
	    conf->clk < HISTB_SCI_CARD_CLOCK_MIN ||
	    conf->clk > HISTB_SCI_CARD_CLOCK_MAX)
		return -EINVAL;

	card_period = 2ULL * conf->clk;
	divisor = DIV_ROUND_CLOSEST_ULL(sci->port.uartclk, card_period);
	if (!divisor || divisor > 256)
		return -EINVAL;

	/* The controller counts module clocks for each card clock period. */
	baud_total = div_u64(conf->sc_fi, conf->sc_di) * 2 * divisor;
	if (!baud_total)
		return -EINVAL;

	for (etu_value = 5; etu_value < 256; etu_value++) {
		baud_count = div_u64_rem(baud_total, etu_value, &remainder);
		if (!remainder && baud_count && baud_count - 1 <= U16_MAX)
			break;
	}
	if (etu_value == 256)
		return -EINVAL;

	*clkicc = divisor - 1;
	*etu = etu_value;
	*baud = baud_count - 1;

	return 0;
}

static int histb_sci_apply_iso7816(struct histb_sci_port *sci,
				   const struct serial_iso7816 *conf)
{
	u32 clkicc, etu, baud, cr0, cr1, protocol;
	u64 stable;
	int ret;

	protocol = conf->flags & SER_ISO7816_T_PARAM;
	if (protocol != SER_ISO7816_T(0) &&
	    protocol != SER_ISO7816_T(1))
		return -EINVAL;
	if (conf->tg > U8_MAX)
		return -EINVAL;

	ret = histb_sci_calculate_timing(sci, conf, &clkicc, &etu, &baud);
	if (ret)
		return ret;

	cr0 = HISTB_SCI_CR0_PARITY_ENABLE;
	if (sci->vcc_active_low)
		cr0 |= HISTB_SCI_CR0_VCC_ENABLE_INV;
	if (!sci->detect_active_high)
		cr0 |= HISTB_SCI_CR0_DETECT_INV;

	/* T=0 NAK and block timeout stay disabled until ATR is complete. */
	cr1 = HISTB_SCI_CR1_ATR_TIMEOUT | HISTB_SCI_CR1_BLOCK_GUARD |
	       sci->cr1_output_mode;

	histb_sci_write(sci, cr0, HISTB_SCI_CR0);
	histb_sci_write(sci, cr1, HISTB_SCI_CR1);
	histb_sci_write(sci, clkicc, HISTB_SCI_CLKICC);
	histb_sci_write(sci, etu, HISTB_SCI_ETU);
	histb_sci_write(sci, baud, HISTB_SCI_BAUD);
	histb_sci_write(sci, 1 << HISTB_SCI_TIDE_RX_SHIFT,
			HISTB_SCI_TIDE);
	histb_sci_write(sci, 0, HISTB_SCI_DMACR);

	stable = (u64)HISTB_SCI_DEFAULT_STABLE * sci->port.uartclk;
	stable = DIV_ROUND_CLOSEST_ULL(stable, 65535U * 1000U);
	histb_sci_write(sci, stable, HISTB_SCI_STABLE);
	histb_sci_write(sci, HISTB_SCI_DEFAULT_ACTIVE, HISTB_SCI_ATIME);
	histb_sci_write(sci, HISTB_SCI_DEFAULT_DEACTIVE, HISTB_SCI_DTIME);
	histb_sci_write(sci, HISTB_SCI_DEFAULT_ATR_START,
			HISTB_SCI_ATRSTIME);
	histb_sci_write(sci, HISTB_SCI_DEFAULT_ATR_DURATION,
			HISTB_SCI_ATRDTIME);
	histb_sci_write(sci, HISTB_SCI_DEFAULT_STOP, HISTB_SCI_STOPTIME);
	histb_sci_write(sci, HISTB_SCI_DEFAULT_START, HISTB_SCI_STARTTIME);
	histb_sci_write(sci, HISTB_SCI_DEFAULT_RETRY |
			(HISTB_SCI_DEFAULT_RETRY << 3), HISTB_SCI_RETRY);
	histb_sci_write_timeout(sci, HISTB_SCI_DEFAULT_CHAR_TIMEOUT,
				HISTB_SCI_CHTIMEMS, HISTB_SCI_CHTIMELS);
	histb_sci_write_timeout(sci, HISTB_SCI_DEFAULT_BLOCK_TIMEOUT,
				HISTB_SCI_BLKTIMEMS, HISTB_SCI_BLKTIMELS);
	histb_sci_write(sci, conf->tg, HISTB_SCI_CHGUARD);
	histb_sci_write(sci, HISTB_SCI_DEFAULT_BLOCK_GUARD,
			HISTB_SCI_BLKGUARD);
	histb_sci_write(sci, HISTB_SCI_DEFAULT_RX_TIMEOUT, HISTB_SCI_RXTIME);

	histb_sci_reset_atr(sci);
	sci->imsc = HISTB_SCI_INT_CARD | HISTB_SCI_INT_ATR_TIMEOUT |
		    HISTB_SCI_INT_RX;
	histb_sci_write_imsc(sci);

	return 0;
}

/*
 * A second enabled configuration is the userspace working protocol selected
 * after reading ATR/PPS. Changing Fi/Di must not power-cycle the active card
 * or return it to ATR mode.
 */
static int histb_sci_update_iso7816(struct histb_sci_port *sci,
				    const struct serial_iso7816 *conf)
{
	u32 clkicc, etu, baud, cr0, cr1;
	u32 protocol = conf->flags & SER_ISO7816_T_PARAM;
	int ret;

	if (protocol != SER_ISO7816_T(0) && protocol != SER_ISO7816_T(1))
		return -EINVAL;
	if (conf->tg > U8_MAX)
		return -EINVAL;
	ret = histb_sci_calculate_timing(sci, conf, &clkicc, &etu, &baud);
	if (ret)
		return ret;
	if (sci->tx_active ||
	    !kfifo_is_empty(&sci->port.state->port.xmit_fifo) ||
	    ((histb_sci_read(sci, HISTB_SCI_TXCOUNT) |
	      histb_sci_read(sci, HISTB_SCI_RXCOUNT)) & HISTB_SCI_FIFO_COUNT))
		return -EBUSY;

	cr0 = histb_sci_read(sci, HISTB_SCI_CR0);
	cr0 &= ~(HISTB_SCI_CR0_TX_NAK | HISTB_SCI_CR0_RX_NAK);
	if (protocol == SER_ISO7816_T(0))
		cr0 |= HISTB_SCI_CR0_TX_NAK | HISTB_SCI_CR0_RX_NAK;
	cr1 = histb_sci_read(sci, HISTB_SCI_CR1);
	cr1 &= ~HISTB_SCI_CR1_ATR_TIMEOUT;
	histb_sci_write(sci, cr0, HISTB_SCI_CR0);
	histb_sci_write(sci, cr1, HISTB_SCI_CR1);
	histb_sci_write(sci, clkicc, HISTB_SCI_CLKICC);
	histb_sci_write(sci, etu, HISTB_SCI_ETU);
	histb_sci_write(sci, baud, HISTB_SCI_BAUD);
	histb_sci_write(sci, conf->tg, HISTB_SCI_CHGUARD);
	histb_sci_write(sci, HISTB_SCI_INT_ATR_TIMEOUT, HISTB_SCI_ICR);
	sci->imsc &= ~HISTB_SCI_INT_ATR_TIMEOUT;
	histb_sci_write_imsc(sci);
	sci->atr_protocol = protocol >> 4;
	sci->state = HISTB_SCI_STATE_READY;
	return 0;
}

static int histb_sci_config_iso7816(struct uart_port *port,
				    struct serial_iso7816 *conf)
{
	struct histb_sci_port *sci = to_histb_sci(port);
	u32 allowed = SER_ISO7816_ENABLED | SER_ISO7816_T_PARAM;
	bool present;
	int ret;

	if (conf->flags & ~allowed)
		return -EINVAL;

	if (!(conf->flags & SER_ISO7816_ENABLED)) {
		cancel_delayed_work(&sci->activate_work);
		histb_sci_finish(sci);
		sci->imsc = HISTB_SCI_INT_CARD;
		histb_sci_write_imsc(sci);
		memset(conf, 0, sizeof(*conf));
		port->iso7816 = *conf;
		return 0;
	}

	present = histb_sci_card_is_present(sci);
	if ((port->iso7816.flags & SER_ISO7816_ENABLED) && sci->opened &&
	    present && !delayed_work_pending(&sci->activate_work) &&
	    (sci->state == HISTB_SCI_STATE_READY ||
	     sci->state == HISTB_SCI_STATE_INACTIVE)) {
		ret = histb_sci_update_iso7816(sci, conf);
		if (!ret)
			port->iso7816 = *conf;
		return ret;
	}

	ret = histb_sci_apply_iso7816(sci, conf);
	if (ret)
		return ret;

	port->iso7816 = *conf;
	present = histb_sci_card_is_present(sci);
	histb_sci_update_card_present(sci, present);
	if (present)
		histb_sci_schedule_activation(sci);
	else
		sci->state = HISTB_SCI_STATE_NO_CARD;

	return 0;
}

static unsigned int histb_sci_tx_empty(struct uart_port *port)
{
	struct histb_sci_port *sci = to_histb_sci(port);

	return (histb_sci_read(sci, HISTB_SCI_TXCOUNT) &
		HISTB_SCI_FIFO_COUNT) ? 0 : TIOCSER_TEMT;
}

static void histb_sci_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int histb_sci_get_mctrl(struct uart_port *port)
{
	struct histb_sci_port *sci = to_histb_sci(port);
	u32 mctrl = TIOCM_CTS | TIOCM_DSR;

	if (histb_sci_card_is_present(sci))
		mctrl |= TIOCM_CAR;

	return mctrl;
}

static void histb_sci_stop_tx(struct uart_port *port)
{
	struct histb_sci_port *sci = to_histb_sci(port);

	sci->imsc &= ~HISTB_SCI_INT_TX_TIDE;
	histb_sci_write_imsc(sci);
}

static void histb_sci_handoff_to_rx(struct histb_sci_port *sci,
				    bool final_refill)
{
	histb_sci_stop_tx(&sci->port);

	/*
	 * SCI_PushData() clears RX only when an interrupt refill places the last
	 * software byte in the hardware FIFO.  The short-command path already
	 * cleared RX at the transaction boundary and switches direction directly.
	 */
	if (final_refill) {
		histb_sci_write(sci, U32_MAX, HISTB_SCI_RXCOUNT);
		atomic64_inc(&sci->tx_refill_handoff_count);
	} else {
		atomic64_inc(&sci->tx_short_handoff_count);
	}

	histb_sci_set_rx_mode(sci);
	sci->tx_active = false;
	sci->tx_drain_pending = true;
	atomic64_inc(&sci->tx_rx_handoff_count);
}

static bool histb_sci_tx_chars(struct histb_sci_port *sci)
{
	struct uart_port *port = &sci->port;
	u32 count, pending, space;
	u8 ch;

	count = histb_sci_read(sci, HISTB_SCI_TXCOUNT) &
		HISTB_SCI_FIFO_COUNT;
	count = min_t(u32, count, HISTB_SCI_FIFO_SIZE);
	space = HISTB_SCI_FIFO_SIZE - count;

	pending = uart_port_tx_limited_flags(port, ch, UART_TX_NOSTOP, space,
					     true,
					     ({
						 histb_sci_write(sci, ch,
								 HISTB_SCI_DATA);
						 atomic64_inc(&sci->tx_byte_count);
					     }),
					     ({}));

	return !pending && !port->x_char;
}

static void histb_sci_start_tx(struct uart_port *port)
{
	struct histb_sci_port *sci = to_histb_sci(port);
	bool refill = sci->tx_active;
	bool append_to_draining;

	if (!port->x_char && kfifo_is_empty(&port->state->port.xmit_fifo))
		return;

	append_to_draining = !refill && sci->tx_drain_pending &&
		(histb_sci_read(sci, HISTB_SCI_TXCOUNT) &
		 HISTB_SCI_FIFO_COUNT);

	if (!refill && !append_to_draining) {
		/* SCI_SendData() starts each transaction with both FIFOs clean. */
		histb_sci_clear_fifos(sci);
		histb_sci_set_tx_mode(sci);
		histb_sci_write(sci, 1 << HISTB_SCI_TIDE_RX_SHIFT,
				HISTB_SCI_TIDE);
		sci->tx_active = true;
		atomic64_inc(&sci->tx_transaction_count);
	} else if (refill) {
		/* serial_core may kick an active stream more than once. */
		atomic64_inc(&sci->tx_rekick_count);
	} else {
		/* Preserve bytes still draining from a fragmented TTY write. */
		histb_sci_set_tx_mode(sci);
		sci->tx_active = true;
		sci->tx_drain_pending = false;
		atomic64_inc(&sci->tx_drain_append_count);
	}

	/* Clear a stale empty-FIFO indication before filling the FIFO. */
	histb_sci_write(sci, HISTB_SCI_INT_TX_TIDE, HISTB_SCI_ICR);
	if (histb_sci_tx_chars(sci)) {
		histb_sci_handoff_to_rx(sci, refill || append_to_draining);
		return;
	}

	sci->imsc |= HISTB_SCI_INT_TX_TIDE;
	histb_sci_write_imsc(sci);
}

static void histb_sci_stop_rx(struct uart_port *port)
{
	struct histb_sci_port *sci = to_histb_sci(port);

	sci->imsc &= ~HISTB_SCI_INT_RX;
	histb_sci_write_imsc(sci);
}

static void histb_sci_start_rx(struct uart_port *port)
{
	struct histb_sci_port *sci = to_histb_sci(port);

	if (port->iso7816.flags & SER_ISO7816_ENABLED) {
		sci->imsc |= HISTB_SCI_INT_RX;
		histb_sci_write_imsc(sci);
	}
}

static void histb_sci_atr_fail(struct histb_sci_port *sci,
			       const char *reason)
{
	atomic64_inc(&sci->atr_fail_count);
	dev_warn_ratelimited(sci->port.dev, "invalid ATR: %s\n", reason);
	histb_sci_finish(sci);
}

static void histb_sci_atr_complete(struct histb_sci_port *sci)
{
	u32 requested = sci->port.iso7816.flags & SER_ISO7816_T_PARAM;
	u32 cr0 = histb_sci_read(sci, HISTB_SCI_CR0);
	u32 cr1 = histb_sci_read(sci, HISTB_SCI_CR1);
	u32 guard = sci->port.iso7816.tg;

	if (sci->atr_protocol > 1) {
		histb_sci_atr_fail(sci, "unsupported protocol");
		return;
	}

	cr0 &= ~(HISTB_SCI_CR0_CONVENTION | HISTB_SCI_CR0_TX_NAK |
		 HISTB_SCI_CR0_RX_NAK);
	if (sci->atr_inverse)
		cr0 |= HISTB_SCI_CR0_CONVENTION;
	if (sci->atr_protocol == 0)
		cr0 |= HISTB_SCI_CR0_TX_NAK | HISTB_SCI_CR0_RX_NAK;
	histb_sci_write(sci, cr0, HISTB_SCI_CR0);

	if (sci->atr_has_char_guard) {
		guard = sci->atr_char_guard;
		if (guard == U8_MAX)
			guard = 0;
		else if (sci->atr_protocol == 1)
			guard++;
	}
	histb_sci_write(sci, guard, HISTB_SCI_CHGUARD);

	histb_sci_write(sci, cr1 & ~HISTB_SCI_CR1_ATR_TIMEOUT,
			HISTB_SCI_CR1);
	histb_sci_write(sci, HISTB_SCI_INT_ATR_TIMEOUT, HISTB_SCI_ICR);
	sci->imsc &= ~HISTB_SCI_INT_ATR_TIMEOUT;
	histb_sci_write_imsc(sci);
	sci->state = HISTB_SCI_STATE_READY;
	atomic64_inc(&sci->atr_complete_count);

	if (requested != SER_ISO7816_T(sci->atr_protocol))
		dev_warn_ratelimited(sci->port.dev,
				     "ATR offers T=%u while userspace selected T=%u\n",
				     sci->atr_protocol, requested >> 4);
}

static void histb_sci_atr_advance(struct histb_sci_port *sci)
{
	if (sci->atr_interface_mask) {
		sci->atr_stage = HISTB_SCI_ATR_INTERFACE;
		return;
	}

	if (sci->atr_historical_bytes) {
		sci->atr_stage = HISTB_SCI_ATR_HISTORICAL;
		return;
	}

	if (sci->atr_tck_required) {
		sci->atr_stage = HISTB_SCI_ATR_TCK;
		return;
	}

	histb_sci_atr_complete(sci);
}

static void histb_sci_atr_consume(struct histb_sci_port *sci, u8 *byte)
{
	u8 item;

	if (sci->atr_length >= HISTB_SCI_ATR_MAX_LENGTH) {
		histb_sci_atr_fail(sci, "longer than 256 bytes");
		return;
	}
	sci->atr_length++;

	if (sci->atr_stage != HISTB_SCI_ATR_TS && sci->atr_inverse)
		*byte = ~bitrev8(*byte);
	sci->atr[sci->atr_length - 1] = *byte;

	switch (sci->atr_stage) {
	case HISTB_SCI_ATR_TS:
		if (*byte != 0x3b && *byte != 0x3f && *byte != 0x03) {
			histb_sci_atr_fail(sci, "invalid TS");
			return;
		}
		sci->atr_inverse = *byte != 0x3b;
		if (*byte == 0x03)
			*byte = 0x3f;
		sci->atr_stage = HISTB_SCI_ATR_T0;
		sci->state = HISTB_SCI_STATE_READ_ATR;
		break;

	case HISTB_SCI_ATR_T0:
		sci->atr_checksum = *byte;
		sci->atr_interface_mask = *byte >> 4;
		sci->atr_historical_bytes = *byte & 0x0f;
		sci->atr_group = 1;
		histb_sci_atr_advance(sci);
		break;

	case HISTB_SCI_ATR_INTERFACE:
		if (!sci->atr_interface_mask) {
			histb_sci_atr_fail(sci, "unexpected interface byte");
			return;
		}

		sci->atr_checksum ^= *byte;
		item = __ffs(sci->atr_interface_mask);
		sci->atr_interface_mask &= ~BIT(item);

		if (item == 2 && sci->atr_group == 1) {
			sci->atr_char_guard = *byte;
			sci->atr_has_char_guard = true;
		} else if (item == 3) {
			if (!sci->atr_protocol_seen) {
				sci->atr_protocol = *byte & 0x0f;
				sci->atr_protocol_seen = true;
			}
			if ((*byte & 0x0f) != 0)
				sci->atr_tck_required = true;
			sci->atr_group++;
			sci->atr_interface_mask = *byte >> 4;
		}
		histb_sci_atr_advance(sci);
		break;

	case HISTB_SCI_ATR_HISTORICAL:
		if (!sci->atr_historical_bytes) {
			histb_sci_atr_fail(sci, "unexpected historical byte");
			return;
		}
		sci->atr_checksum ^= *byte;
		sci->atr_historical_bytes--;
		histb_sci_atr_advance(sci);
		break;

	case HISTB_SCI_ATR_TCK:
		sci->atr_checksum ^= *byte;
		if (sci->atr_checksum) {
			histb_sci_atr_fail(sci, "TCK checksum mismatch");
			return;
		}
		histb_sci_atr_complete(sci);
		break;
	}
}

static bool histb_sci_rx_chars(struct histb_sci_port *sci)
{
	struct uart_port *port = &sci->port;
	bool received = false;
	u32 count, data, status;
	u8 ch, flag;

	count = histb_sci_read(sci, HISTB_SCI_RXCOUNT) &
		HISTB_SCI_FIFO_COUNT;
	while (count--) {
		data = histb_sci_read(sci, HISTB_SCI_DATA);
		status = data & HISTB_SCI_DATA_PARITY;
		ch = data & HISTB_SCI_DATA_VALUE;
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (sci->state == HISTB_SCI_STATE_WAIT_ATR ||
		    sci->state == HISTB_SCI_STATE_READ_ATR)
			histb_sci_atr_consume(sci, &ch);

		if (status) {
			port->icount.parity++;
			flag = TTY_PARITY;
		}
		if (!uart_handle_sysrq_char(port, ch))
			uart_insert_char(port, status, 0, ch, flag);
		atomic64_inc(&sci->rx_byte_count);
		received = true;
	}

	return received;
}

static void histb_sci_handle_card_down(struct histb_sci_port *sci)
{
	bool present;
	int ret;

	if (sci->state <= HISTB_SCI_STATE_INACTIVE) {
		present = histb_sci_card_is_present(sci);
		histb_sci_update_card_present(sci, present);
		sci->state = present ? HISTB_SCI_STATE_INACTIVE :
				       HISTB_SCI_STATE_NO_CARD;
		return;
	}

	cancel_delayed_work(&sci->activate_work);
	histb_sci_write(sci, 0, HISTB_SCI_IMSC);
	histb_sci_write(sci, HISTB_SCI_INT_ALL, HISTB_SCI_ICR);

	if (sci->port.iso7816.flags & SER_ISO7816_ENABLED) {
		ret = histb_sci_apply_iso7816(sci, &sci->port.iso7816);
		if (ret) {
			dev_warn_ratelimited(sci->port.dev,
					     "failed to restore ISO7816 settings after CARD_DOWN: %d\n",
					     ret);
			sci->imsc = HISTB_SCI_INT_CARD;
			histb_sci_write_imsc(sci);
		}
	} else {
		sci->imsc = HISTB_SCI_INT_CARD;
		histb_sci_write_imsc(sci);
	}

	histb_sci_clear_fifos(sci);
	histb_sci_set_rx_mode(sci);
	histb_sci_reset_atr(sci);
	sci->tx_active = false;
	present = histb_sci_card_is_present(sci);
	histb_sci_update_card_present(sci, present);
	sci->state = present ? HISTB_SCI_STATE_INACTIVE :
			       HISTB_SCI_STATE_NO_CARD;
}

static void histb_sci_handle_card_event(struct histb_sci_port *sci, u32 event)
{
	switch (event) {
	case HISTB_SCI_INT_CARD_IN:
		histb_sci_update_card_present(sci, true);
		if (sci->state == HISTB_SCI_STATE_NO_CARD) {
			sci->state = HISTB_SCI_STATE_INACTIVE;
			if (sci->port.iso7816.flags & SER_ISO7816_ENABLED)
				histb_sci_schedule_activation(sci);
		}
		break;

	case HISTB_SCI_INT_CARD_OUT:
		cancel_delayed_work(&sci->activate_work);
		histb_sci_update_card_present(sci, false);
		histb_sci_reset_atr(sci);
		sci->state = HISTB_SCI_STATE_NO_CARD;
		histb_sci_stop_tx(&sci->port);
		sci->tx_active = false;
		histb_sci_clear_fifos(sci);
		break;

	case HISTB_SCI_INT_CARD_DOWN:
		histb_sci_handle_card_down(sci);
		break;

	case HISTB_SCI_INT_CARD_UP:
		histb_sci_update_card_present(sci, true);
		if (sci->port.iso7816.flags & SER_ISO7816_ENABLED)
			histb_sci_prepare_atr(sci);
		break;
	}
}

static void histb_sci_handle_atr_start_timeout(struct histb_sci_port *sci,
					       u32 pending)
{
	u32 rx_count, syncact;

	if (sci->state != HISTB_SCI_STATE_WAIT_ATR &&
	    sci->state != HISTB_SCI_STATE_READ_ATR)
		return;

	rx_count = histb_sci_read(sci, HISTB_SCI_RXCOUNT) &
		   HISTB_SCI_FIFO_COUNT;
	syncact = histb_sci_read(sci, HISTB_SCI_SYNCACT);
	dev_warn_ratelimited(sci->port.dev,
			     "ATR start timeout: state=%u cr0=%#x cr1=%#x clkicc=%u etu=%u baud=%u rx=%u imsc=%#x mis=%#x syncact=%#x\n",
			     sci->state,
			     histb_sci_read(sci, HISTB_SCI_CR0),
			     histb_sci_read(sci, HISTB_SCI_CR1),
			     histb_sci_read(sci, HISTB_SCI_CLKICC),
			     histb_sci_read(sci, HISTB_SCI_ETU),
			     histb_sci_read(sci, HISTB_SCI_BAUD), rx_count,
			     histb_sci_read(sci, HISTB_SCI_IMSC),
			     pending, syncact);
	atomic64_inc(&sci->atr_start_timeout_count);
	histb_sci_reset_atr(sci);
	sci->state = sci->card_present ? HISTB_SCI_STATE_INACTIVE :
					 HISTB_SCI_STATE_NO_CARD;
	sci->imsc &= ~HISTB_SCI_INT_ATR_START_TIMEOUT;
	histb_sci_write_imsc(sci);
}

static irqreturn_t histb_sci_interrupt(int irq, void *data)
{
	struct histb_sci_port *sci = data;
	struct uart_port *port = &sci->port;
	bool handled = false;
	bool received = false;
	unsigned int rounds;
	unsigned long flags;
	u32 event, pending, status;

	uart_port_lock_irqsave(port, &flags);
	atomic64_inc(&sci->irq_count);
	for (rounds = 0; rounds < HISTB_SCI_IRQ_DRAIN_LIMIT; rounds++) {
		status = histb_sci_read(sci, HISTB_SCI_MIS) &
			 HISTB_SCI_INT_ALL;
		if (!status)
			break;
		pending = status;
		WRITE_ONCE(sci->last_irq_status, pending);

		while (status) {
			event = BIT(__ffs(status));
			status &= ~event;
			histb_sci_write(sci, event, HISTB_SCI_ICR);
			handled = true;

			switch (event) {
			case HISTB_SCI_INT_CARD_IN:
			case HISTB_SCI_INT_CARD_OUT:
			case HISTB_SCI_INT_CARD_UP:
			case HISTB_SCI_INT_CARD_DOWN:
				histb_sci_handle_card_event(sci, event);
				if (event == HISTB_SCI_INT_CARD_DOWN)
					status = 0;
				break;
			case HISTB_SCI_INT_TX_ERROR:
				port->icount.frame++;
				atomic64_inc(&sci->tx_error_count);
				histb_sci_stop_tx(port);
				histb_sci_set_rx_mode(sci);
				sci->tx_active = false;
				sci->tx_drain_pending = false;
				status = 0;
				break;
			case HISTB_SCI_INT_ATR_START_TIMEOUT:
				histb_sci_handle_atr_start_timeout(sci, pending);
				break;
			case HISTB_SCI_INT_ATR_DURATION_TIMEOUT:
				atomic64_inc(&sci->atr_duration_timeout_count);
				if (sci->state == HISTB_SCI_STATE_READ_ATR)
					dev_warn_ratelimited(port->dev,
							     "ATR duration timeout; partial ATR remains available to userspace\n");
				break;
			case HISTB_SCI_INT_RX_OVERRUN:
				port->icount.overrun++;
				atomic64_inc(&sci->rx_overrun_count);
				received |= tty_insert_flip_char(&port->state->port, 0,
								 TTY_OVERRUN) != 0;
				break;
			case HISTB_SCI_INT_RX_TIDE:
				received |= histb_sci_rx_chars(sci);
				break;
			case HISTB_SCI_INT_TX_TIDE:
				histb_sci_stop_tx(port);
				if (!sci->tx_active)
					break;
				if (histb_sci_tx_chars(sci)) {
					histb_sci_handoff_to_rx(sci, true);
				} else {
					sci->imsc |= HISTB_SCI_INT_TX_TIDE;
					histb_sci_write_imsc(sci);
				}
				break;
			}
		}
	}

	if (rounds == HISTB_SCI_IRQ_DRAIN_LIMIT) {
		atomic64_inc(&sci->irq_drain_limit_count);
		dev_warn_ratelimited(port->dev,
				     "interrupt status did not quiesce\n");
	}
	uart_port_unlock_irqrestore(port, flags);

	if (received)
		tty_flip_buffer_push(&port->state->port);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static void histb_sci_hw_init(struct histb_sci_port *sci)
{
	u32 cr0 = 0;

	if (sci->vcc_active_low)
		cr0 |= HISTB_SCI_CR0_VCC_ENABLE_INV;
	if (!sci->detect_active_high)
		cr0 |= HISTB_SCI_CR0_DETECT_INV;

	histb_sci_write(sci, 0, HISTB_SCI_IMSC);
	histb_sci_write(sci, HISTB_SCI_INT_ALL, HISTB_SCI_ICR);
	histb_sci_write(sci, cr0, HISTB_SCI_CR0);
	histb_sci_write(sci, sci->cr1_output_mode, HISTB_SCI_CR1);
	histb_sci_write(sci, 0, HISTB_SCI_DMACR);
	histb_sci_clear_fifos(sci);
	histb_sci_finish(sci);
	sci->state = HISTB_SCI_STATE_NO_CARD;
}

static int histb_sci_startup(struct uart_port *port)
{
	struct histb_sci_port *sci = to_histb_sci(port);
	unsigned long flags;
	int ret;

	ret = clk_prepare_enable(sci->clk);
	if (ret)
		return ret;

	ret = reset_control_deassert(sci->reset);
	if (ret)
		goto err_disable_clock;

	histb_sci_hw_init(sci);
	ret = request_irq(port->irq, histb_sci_interrupt, 0, dev_name(port->dev),
			  sci);
	if (ret)
		goto err_assert_reset;

	uart_port_lock_irqsave(port, &flags);
	sci->opened = true;
	sci->card_present = histb_sci_card_is_present(sci);
	sci->state = sci->card_present ? HISTB_SCI_STATE_INACTIVE :
					  HISTB_SCI_STATE_NO_CARD;
	sci->imsc = HISTB_SCI_INT_CARD;
	if (port->iso7816.flags & SER_ISO7816_ENABLED) {
		ret = histb_sci_apply_iso7816(sci, &port->iso7816);
		if (!ret && sci->card_present)
			histb_sci_schedule_activation(sci);
	} else {
		histb_sci_write_imsc(sci);
	}
	uart_port_unlock_irqrestore(port, flags);
	if (ret)
		goto err_free_irq;

	return 0;

err_free_irq:
	uart_port_lock_irqsave(port, &flags);
	sci->opened = false;
	sci->imsc = 0;
	histb_sci_write_imsc(sci);
	histb_sci_finish(sci);
	uart_port_unlock_irqrestore(port, flags);
	free_irq(port->irq, sci);
	cancel_delayed_work_sync(&sci->activate_work);
err_assert_reset:
	reset_control_assert(sci->reset);
err_disable_clock:
	clk_disable_unprepare(sci->clk);
	return ret;
}

static void histb_sci_shutdown(struct uart_port *port)
{
	struct histb_sci_port *sci = to_histb_sci(port);
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);
	sci->opened = false;
	sci->imsc = 0;
	histb_sci_write_imsc(sci);
	histb_sci_finish(sci);
	uart_port_unlock_irqrestore(port, flags);

	free_irq(port->irq, sci);
	cancel_delayed_work_sync(&sci->activate_work);
	reset_control_assert(sci->reset);
	clk_disable_unprepare(sci->clk);
}

static void histb_sci_flush_buffer(struct uart_port *port)
{
	struct histb_sci_port *sci = to_histb_sci(port);

	histb_sci_stop_tx(port);
	sci->tx_active = false;
	histb_sci_clear_fifos(sci);
	histb_sci_set_rx_mode(sci);
}

static const char *histb_sci_state_name(enum histb_sci_state state)
{
	switch (state) {
	case HISTB_SCI_STATE_NO_CARD:
		return "no-card";
	case HISTB_SCI_STATE_INACTIVE:
		return "inactive";
	case HISTB_SCI_STATE_WAIT_ATR:
		return "wait-atr";
	case HISTB_SCI_STATE_READ_ATR:
		return "read-atr";
	case HISTB_SCI_STATE_READY:
		return "ready";
	default:
		return "unknown";
	}
}

static ssize_t state_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct histb_sci_port *sci = dev_get_drvdata(dev);
	struct uart_port *port = &sci->port;
	unsigned long flags;
	enum histb_sci_state state;

	uart_port_lock_irqsave(port, &flags);
	state = sci->state;
	uart_port_unlock_irqrestore(port, flags);

	return sysfs_emit(buf, "%s\n", histb_sci_state_name(state));
}
static DEVICE_ATTR_RO(state);

static ssize_t card_present_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct histb_sci_port *sci = dev_get_drvdata(dev);
	struct uart_port *port = &sci->port;
	unsigned long flags;
	bool present;

	uart_port_lock_irqsave(port, &flags);
	present = sci->card_present;
	uart_port_unlock_irqrestore(port, flags);

	return sysfs_emit(buf, "%u\n", present);
}
static DEVICE_ATTR_RO(card_present);

static ssize_t tx_active_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct histb_sci_port *sci = dev_get_drvdata(dev);
	struct uart_port *port = &sci->port;
	unsigned long flags;
	bool active;

	uart_port_lock_irqsave(port, &flags);
	active = sci->tx_active;
	uart_port_unlock_irqrestore(port, flags);

	return sysfs_emit(buf, "%u\n", active);
}
static DEVICE_ATTR_RO(tx_active);

static ssize_t atr_protocol_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct histb_sci_port *sci = dev_get_drvdata(dev);
	struct uart_port *port = &sci->port;
	unsigned long flags;
	u8 protocol;
	bool seen;

	uart_port_lock_irqsave(port, &flags);
	protocol = sci->atr_protocol;
	seen = sci->atr_protocol_seen;
	uart_port_unlock_irqrestore(port, flags);

	if (!seen)
		return sysfs_emit(buf, "unknown\n");
	return sysfs_emit(buf, "T=%u\n", protocol);
}
static DEVICE_ATTR_RO(atr_protocol);

static ssize_t atr_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct histb_sci_port *sci = dev_get_drvdata(dev);
	struct uart_port *port = &sci->port;
	unsigned long flags;
	u8 atr[HISTB_SCI_ATR_MAX_LENGTH];
	u16 len;
	int pos;
	u16 i;

	uart_port_lock_irqsave(port, &flags);
	len = min_t(u16, sci->atr_length, HISTB_SCI_ATR_MAX_LENGTH);
	memcpy(atr, sci->atr, len);
	uart_port_unlock_irqrestore(port, flags);

	pos = scnprintf(buf, PAGE_SIZE, "len=%u", len);
	for (i = 0; i < len && pos < PAGE_SIZE - 4; i++)
		pos += scnprintf(buf + pos, PAGE_SIZE - pos, " %02x", atr[i]);
	pos += scnprintf(buf + pos, PAGE_SIZE - pos, "\n");
	return pos;
}
static DEVICE_ATTR_RO(atr);

#define HISTB_SCI_STAT_ATTR(_name, _member) \
	static ssize_t _name##_show(struct device *dev, \
				    struct device_attribute *attr, char *buf) \
	{ \
		struct histb_sci_port *sci = dev_get_drvdata(dev); \
		return sysfs_emit(buf, "%lld\n", \
				  (long long)atomic64_read(&sci->_member)); \
	} \
	static DEVICE_ATTR_RO(_name)

HISTB_SCI_STAT_ATTR(irq_count, irq_count);
HISTB_SCI_STAT_ATTR(irq_drain_limit_count, irq_drain_limit_count);
HISTB_SCI_STAT_ATTR(rx_bytes, rx_byte_count);
HISTB_SCI_STAT_ATTR(tx_bytes, tx_byte_count);
HISTB_SCI_STAT_ATTR(tx_transactions, tx_transaction_count);
HISTB_SCI_STAT_ATTR(tx_rekicks, tx_rekick_count);
HISTB_SCI_STAT_ATTR(tx_drain_appends, tx_drain_append_count);
HISTB_SCI_STAT_ATTR(tx_rx_handoffs, tx_rx_handoff_count);
HISTB_SCI_STAT_ATTR(tx_short_handoffs, tx_short_handoff_count);
HISTB_SCI_STAT_ATTR(tx_refill_handoffs, tx_refill_handoff_count);
HISTB_SCI_STAT_ATTR(atr_complete, atr_complete_count);
HISTB_SCI_STAT_ATTR(atr_fail, atr_fail_count);
HISTB_SCI_STAT_ATTR(atr_start_timeout, atr_start_timeout_count);
HISTB_SCI_STAT_ATTR(atr_duration_timeout, atr_duration_timeout_count);
HISTB_SCI_STAT_ATTR(tx_errors, tx_error_count);
HISTB_SCI_STAT_ATTR(rx_overruns, rx_overrun_count);
HISTB_SCI_STAT_ATTR(fifo_resets, fifo_clear_count);
HISTB_SCI_STAT_ATTR(recoveries, recovery_count);

static ssize_t last_irq_status_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct histb_sci_port *sci = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%#x\n", READ_ONCE(sci->last_irq_status));
}
static DEVICE_ATTR_RO(last_irq_status);

static ssize_t recover_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct histb_sci_port *sci = dev_get_drvdata(dev);
	struct uart_port *port = &sci->port;
	unsigned long flags;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	uart_port_lock_irqsave(port, &flags);
	if (!sci->opened) {
		uart_port_unlock_irqrestore(port, flags);
		return -EIO;
	}
	histb_sci_recover(sci, true);
	uart_port_unlock_irqrestore(port, flags);

	return count;
}
static DEVICE_ATTR_WO(recover);

static struct attribute *histb_sci_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_card_present.attr,
	&dev_attr_tx_active.attr,
	&dev_attr_atr.attr,
	&dev_attr_atr_protocol.attr,
	&dev_attr_irq_count.attr,
	&dev_attr_irq_drain_limit_count.attr,
	&dev_attr_rx_bytes.attr,
	&dev_attr_tx_bytes.attr,
	&dev_attr_tx_transactions.attr,
	&dev_attr_tx_rekicks.attr,
	&dev_attr_tx_drain_appends.attr,
	&dev_attr_tx_rx_handoffs.attr,
	&dev_attr_tx_short_handoffs.attr,
	&dev_attr_tx_refill_handoffs.attr,
	&dev_attr_atr_complete.attr,
	&dev_attr_atr_fail.attr,
	&dev_attr_atr_start_timeout.attr,
	&dev_attr_atr_duration_timeout.attr,
	&dev_attr_tx_errors.attr,
	&dev_attr_rx_overruns.attr,
	&dev_attr_fifo_resets.attr,
	&dev_attr_recoveries.attr,
	&dev_attr_last_irq_status.attr,
	&dev_attr_recover.attr,
	NULL,
};

static const struct attribute_group histb_sci_group = {
	.name = "sci",
	.attrs = histb_sci_attrs,
};

static void histb_sci_set_termios(struct uart_port *port,
				  struct ktermios *termios,
				  const struct ktermios *old)
{
	unsigned long flags;

	if (old)
		tty_termios_copy_hw(termios, old);

	uart_port_lock_irqsave(port, &flags);
	port->read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= HISTB_SCI_DATA_PARITY;
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= HISTB_SCI_DATA_PARITY;
	uart_update_timeout(port, CS8 | PARENB, 9600);
	uart_port_unlock_irqrestore(port, flags);
}

static const char *histb_sci_type(struct uart_port *port)
{
	return port->type == PORT_GENERIC ? "Hi3798CV200 SCI" : NULL;
}

static void histb_sci_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_GENERIC;
}

static const struct uart_ops histb_sci_uart_ops = {
	.tx_empty = histb_sci_tx_empty,
	.set_mctrl = histb_sci_set_mctrl,
	.get_mctrl = histb_sci_get_mctrl,
	.stop_tx = histb_sci_stop_tx,
	.start_tx = histb_sci_start_tx,
	.stop_rx = histb_sci_stop_rx,
	.start_rx = histb_sci_start_rx,
	.startup = histb_sci_startup,
	.shutdown = histb_sci_shutdown,
	.flush_buffer = histb_sci_flush_buffer,
	.set_termios = histb_sci_set_termios,
	.type = histb_sci_type,
	.config_port = histb_sci_config_port,
};

static int histb_sci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_sci_port *sci;
	struct resource *res;
	const char *detect_property = "hisilicon,card-detect-active-high";
	int ret;

	sci = devm_kzalloc(dev, sizeof(*sci), GFP_KERNEL);
	if (!sci)
		return -ENOMEM;

	sci->port.membase =
		devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(sci->port.membase))
		return PTR_ERR(sci->port.membase);

	sci->port.irq = platform_get_irq(pdev, 0);
	if (sci->port.irq < 0)
		return sci->port.irq;

	sci->clk = devm_clk_get(dev, "sci");
	if (IS_ERR(sci->clk))
		return dev_err_probe(dev, PTR_ERR(sci->clk),
				     "failed to get clock\n");

	sci->reset = devm_reset_control_get_exclusive(dev, "sci");
	if (IS_ERR(sci->reset))
		return dev_err_probe(dev, PTR_ERR(sci->reset),
				     "failed to get reset\n");

	sci->vcc_active_low =
		device_property_read_bool(dev, "hisilicon,vcc-active-low");
	sci->detect_active_high = device_property_read_bool(dev, detect_property);
	if (device_property_read_bool(dev, "hisilicon,clock-open-drain"))
		sci->cr1_output_mode |= HISTB_SCI_CR1_CLK_OPEN_DRAIN;
	if (device_property_read_bool(dev, "hisilicon,reset-open-drain"))
		sci->cr1_output_mode |= HISTB_SCI_CR1_RESET_OPEN_DRAIN;
	if (device_property_read_bool(dev, "hisilicon,vcc-enable-open-drain"))
		sci->cr1_output_mode |= HISTB_SCI_CR1_VCCEN_OPEN_DRAIN;
	INIT_DELAYED_WORK(&sci->activate_work, histb_sci_activate_work);

	sci->port.dev = dev;
	sci->port.mapbase = res->start;
	sci->port.iotype = UPIO_MEM32;
	sci->port.flags = UPF_BOOT_AUTOCONF | UPF_FIXED_PORT | UPF_FIXED_TYPE;
	sci->port.ops = &histb_sci_uart_ops;
	sci->port.fifosize = HISTB_SCI_FIFO_SIZE;
	sci->port.type = PORT_GENERIC;
	sci->port.line = 0;
	sci->port.uartclk = clk_get_rate(sci->clk);
	if (!sci->port.uartclk)
		return dev_err_probe(dev, -EINVAL, "clock rate is zero\n");
	sci->port.iso7816_config = histb_sci_config_iso7816;
	spin_lock_init(&sci->port.lock);
	sci->rx_workqueue = alloc_workqueue("ttySCI0-flip",
					    WQ_UNBOUND | WQ_HIGHPRI |
					    WQ_MEM_RECLAIM, 1);
	if (!sci->rx_workqueue)
		return -ENOMEM;
	sci->tty_port = &histb_sci_uart_driver.state[sci->port.line].port;
	tty_buffer_set_workqueue(sci->tty_port, sci->rx_workqueue);

	platform_set_drvdata(pdev, sci);
	ret = uart_add_one_port(&histb_sci_uart_driver, &sci->port);
	if (ret) {
		tty_buffer_set_workqueue(sci->tty_port, NULL);
		destroy_workqueue(sci->rx_workqueue);
		cancel_delayed_work_sync(&sci->activate_work);
		return ret;
	}
	ret = devm_device_add_group(dev, &histb_sci_group);
	if (ret) {
		uart_remove_one_port(&histb_sci_uart_driver, &sci->port);
		tty_buffer_set_workqueue(sci->tty_port, NULL);
		destroy_workqueue(sci->rx_workqueue);
		cancel_delayed_work_sync(&sci->activate_work);
		return ret;
	}

	return 0;
}

static void histb_sci_remove(struct platform_device *pdev)
{
	struct histb_sci_port *sci = platform_get_drvdata(pdev);

	uart_remove_one_port(&histb_sci_uart_driver, &sci->port);
	tty_buffer_set_workqueue(sci->tty_port, NULL);
	destroy_workqueue(sci->rx_workqueue);
	cancel_delayed_work_sync(&sci->activate_work);
}

static const struct of_device_id histb_sci_of_match[] = {
	{ .compatible = "hisilicon,hi3798cv200-sci" },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_sci_of_match);

static struct platform_driver histb_sci_platform_driver = {
	.probe = histb_sci_probe,
	.remove_new = histb_sci_remove,
	.driver = {
		.name = "hi3798cv200-sci",
		.of_match_table = histb_sci_of_match,
	},
};

static int __init histb_sci_init(void)
{
	int ret;

	ret = uart_register_driver(&histb_sci_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&histb_sci_platform_driver);
	if (ret)
		uart_unregister_driver(&histb_sci_uart_driver);

	return ret;
}
module_init(histb_sci_init);

static void __exit histb_sci_exit(void)
{
	platform_driver_unregister(&histb_sci_platform_driver);
	uart_unregister_driver(&histb_sci_uart_driver);
}
module_exit(histb_sci_exit);

MODULE_AUTHOR("HiSilicon Technologies Co., Ltd.");
MODULE_DESCRIPTION("HiSilicon Hi3798CV200 smart card interface");
MODULE_LICENSE("GPL");
