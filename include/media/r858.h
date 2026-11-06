/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _MEDIA_R858_H
#define _MEDIA_R858_H

#include <linux/types.h>
#include <linux/kconfig.h>
#include <linux/stddef.h>

struct dvb_frontend;
struct i2c_adapter;

/* Package crystal wiring, not the internal tuner number or an I2C address. */
enum r858_xtal_role {
	R858_XTAL_INDEPENDENT,
	R858_XTAL_MASTER,
	R858_XTAL_SLAVE_INPUT,
	R858_XTAL_SLAVE_NO_OUTPUT,
};

/**
 * struct r858_config - board wiring for one internal R858 receiver
 * @i2c_address: selected receiver's 7-bit address
 * @tuner_index: internal receiver index, 0 or 1; not a package number
 * @peer_address: other internal receiver's 7-bit address (also required)
 * @peer_adapter: other receiver's bus, NULL for the supplied attach adapter
 * @xtal_hz: crystal/reference frequency in Hz; 24000000 only
 * @if_hz: selected receiver's DVB-C IF in Hz; 5000000 or 5500000
 * @peer_if_hz: other receiver's explicit IF in Hz; 5000000 or 5500000
 * @xtal_role: package's oscillator/input/output wiring
 * @xtal_cap: vendor crystal capacitance code, 0..31 (not pF)
 * @loop_through: retain active RF loop-through, including in standby
 * @clock_out: enable the package's optional outgoing reference; required
 * for MASTER/SLAVE_INPUT crystal chains
 *
 * Both internal endpoints must be reachable during calibration, even when
 * only one DVB frontend is attached. Configurations naming the same internal
 * tuner-0 bus/address share a package mutex, register images and calibration
 * data. They must agree on crystal wiring, capacitance and loop-through.
 * The two buses are referenced until the last receiver is released. Caller
 * must keep any I2C-gate/provider path usable until DVB detach completes.
 *
 * Supported profiles use DVB-C Annex A, 8 MHz bandwidth and 5/5.5 MHz IF.
 * The DVB core's occupied-bandwidth request may be less than 8 MHz; it does
 * not select another calibration profile. Shared LNA current is aggregated
 * across both receiving paths, including a pending tune or restored path.
 * Before either frontend can receive, both internal receivers complete their
 * selected profile's LPF calibration and four image-rejection measurements.
 * Both IF values must be supplied at first attach, even when the peer
 * frontend is unattached. Profiles remain immutable for the package lifetime;
 * later attaches must agree on each endpoint's IF, not infer it from index.
 * Wakeup restores local receiver state without replaying shared registers.
 * Detach attempts each endpoint's local power-down even when shared-primary
 * access fails; such failures remain errors, not successful standby claims.
 * The API and Linux implementation are independent GPL contributions, not
 * a relicensing of the Rafael reference source credited in the driver.
 *
 * The 8-bit write addresses 0x14/0x34 become Linux addresses 0x0a/0x1a.
 * Neither those addresses nor the bus number identifies the package wiring.
 */
struct r858_config {
	u8 i2c_address;
	u8 tuner_index;
	u8 peer_address;
	struct i2c_adapter *peer_adapter;
	u32 xtal_hz;
	u32 if_hz;
	u32 peer_if_hz;
	enum r858_xtal_role xtal_role;
	u8 xtal_cap;
	bool loop_through;
	bool clock_out;
};

#if IS_REACHABLE(CONFIG_MEDIA_TUNER_R858)
struct dvb_frontend *r858_attach(struct dvb_frontend *fe,
				 struct i2c_adapter *adapter,
				 const struct r858_config *config);
#else
static inline struct dvb_frontend *r858_attach(struct dvb_frontend *fe,
					       struct i2c_adapter *adapter,
					       const struct r858_config *config)
{
	return NULL;
}
#endif
#endif
