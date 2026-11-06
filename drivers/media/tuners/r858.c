// SPDX-License-Identifier: GPL-2.0-only
/*
 * Independently authored Linux support for the Rafael Micro R858.
 *
 * Hardware reference: R858_GUI_v4_1.6, 2017-01-25, in
 * HiSTBLinuxV100R005C00SPC050/source/msp/drv/frontend/tuner/R858/.
 * That reference credits Copyright 2015 Rafaelmicro., Inc., Vincent Chien.
 * Its register values, frequency limits and hardware calibration protocol
 * are the source of the numerical hardware facts documented below. This
 * GPL contribution does not grant a license to Rafael's reference sources.
 * No vendor functions or SDK types are incorporated in this implementation.
 *
 * Scope: 24 MHz reference, DVB-C Annex A, 8 MHz channel, 5/5.5 MHz IF.
 * Each physical package has two I2C endpoints, possibly on different buses.
 * The package lock protects both receivers and every shared analog resource.
 * Primary registers 48..55 contain the shared ring PLL/LNA controls. Tail
 * registers of both endpoints are live package state, never standby copies.
 */
#include <linux/bitfield.h>
#include <linux/bitrev.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <media/dvb_frontend.h>
#include <media/r858.h>

#define R858_LOCAL_FIRST 8
#define R858_LOCAL_COUNT 40
#define R858_TAIL_FIRST 48
#define R858_TAIL_COUNT 8
#define R858_REFERENCE_KHZ 24000
#define R858_MIN_LO_HZ 35469000
#define R858_MAX_RF_HZ 1002000000
#define R858_IMR_IF_KHZ 5300

struct r858_iq {
	u8 gain;
	u8 phase;
	u8 cap;
};

struct r858_calibration {
	struct r858_iq iq[4];
	u8 filter_coarse;
	u8 filter_fine;
	bool filter_half;
};

/*
 * Each receiver selects one IF-dependent filter and high-pass profile at
 * attach, including its low-pass filter calibration.
 */
struct r858_profile {
	u32 if_hz;
	u16 filter_cal_khz;
	u8 hpf_corner;
};

static const struct r858_profile r858_profiles[] = {
	{ 5000000, 9250, 10 },
	{ 5500000, 9750, 7 },
};

static const struct r858_profile *r858_find_profile(u32 if_hz)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(r858_profiles); i++)
		if (r858_profiles[i].if_hz == if_hz)
			return &r858_profiles[i];
	return NULL;
}

struct r858_receiver {
	u8 local[R858_LOCAL_COUNT];
	u8 standby[R858_LOCAL_COUNT];
	struct r858_calibration cal;
	const struct r858_profile *profile;
	u32 frequency;
	bool calibrated;
	bool awake;
	bool tuned;
};

struct r858_package {
	struct list_head list;
	struct mutex lock;
	struct i2c_adapter *bus[2];
	u8 address[2];
	struct r858_receiver receiver[2];
	u8 tail[2][R858_TAIL_COUNT];
	unsigned int attached;
	unsigned int touched;
	unsigned int active;
	enum r858_xtal_role xtal_role;
	u8 xtal_cap;
	u8 xtal_drive;
	bool loop_through;
	bool clock_out;
	bool ready;
};

struct r858_state {
	struct r858_package *package;
	unsigned int index;
};

/* Ownership registry only; no process-global register/calibration variables. */
static LIST_HEAD(r858_packages);
static DEFINE_MUTEX(r858_registry_lock);

/* Register reset values, in address order 8..47. The three rows describe
 * normal reception, automatic image measurement and LPF measurement. The
 * endpoint-specific differences are data, not an SDK execution framework.
 * Reference: R858.c initialization/IMR/LPF register arrays. */
struct r858_preset {
	u8 local[2][R858_LOCAL_COUNT];
	u8 tail[2][R858_TAIL_COUNT];
};

static const struct r858_preset r858_normal = {
	.local = {
		{ 0x10, 0x70, 0xa7, 0x01, 0x06, 0x12, 0x30, 0x89,
		  0x20, 0x20, 0x7b, 0x82, 0x30, 0x0a, 0x55, 0x15,
		  0x83, 0x46, 0xa1, 0x08, 0x00, 0xe1, 0x5a, 0x38,
		  0xdb, 0x87, 0x35, 0x49, 0x39, 0x49, 0x45, 0x00,
		  0xcd, 0x2a, 0x0b, 0xe8, 0xd8, 0x00, 0xd2, 0x80 },
		{ 0x10, 0x70, 0xa7, 0x01, 0x06, 0x12, 0x30, 0x89,
		  0x20, 0x20, 0x7b, 0x82, 0x30, 0x11, 0xaa, 0xea,
		  0x83, 0x46, 0xa1, 0x08, 0x00, 0xe1, 0x5a, 0x38,
		  0xd8, 0x87, 0x35, 0x49, 0x39, 0x49, 0x45, 0x00,
		  0xcd, 0x2a, 0x0b, 0xe8, 0xd8, 0x00, 0xd2, 0x80 },
	},
	.tail = {
		{ 0x22, 0x02, 0x00, 0x57, 0xad, 0xf6, 0x77, 0x1a },
		{ 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08 },
	},
};

static const struct r858_preset r858_image_measurement = {
	.local = {
		{ 0x12, 0x00, 0xb0, 0x01, 0x06, 0x11, 0x36, 0x88,
		  0xf0, 0x00, 0x0f, 0x00, 0x4a, 0x0d, 0x55, 0xd5,
		  0x00, 0x53, 0xa2, 0x08, 0x00, 0xb9, 0x38, 0xf8,
		  0x9c, 0x88, 0x4c, 0x68, 0x38, 0xf8, 0x01, 0x06,
		  0xf6, 0x50, 0x00, 0x40, 0xfe, 0x5d, 0xd0, 0x18 },
		{ 0x12, 0xf0, 0xb4, 0x01, 0x06, 0x15, 0x3a, 0x88,
		  0xf3, 0xd0, 0x8b, 0x81, 0x48, 0x0d, 0x55, 0xd5,
		  0x00, 0x53, 0xa2, 0x08, 0x00, 0x00, 0x38, 0xf8,
		  0x9c, 0x88, 0x4c, 0x68, 0x38, 0xf8, 0x01, 0x06,
		  0xf6, 0x50, 0x00, 0x40, 0xc0, 0x45, 0xd6, 0x18 },
	},
	.tail = {
		{ 0x20, 0x40, 0xd1, 0x00, 0x90, 0x25, 0x00, 0x08 },
		{ 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	},
};

static const struct r858_preset r858_filter_measurement = {
	.local = {
		{ 0x12, 0x70, 0xb0, 0x01, 0x06, 0x03, 0x02, 0x88,
		  0x20, 0x00, 0x78, 0x80, 0x00, 0x10, 0x55, 0x55,
		  0x80, 0x55, 0xc7, 0x08, 0x00, 0xb9, 0x5a, 0xf8,
		  0xda, 0x87, 0x3c, 0x19, 0x39, 0x39, 0x01, 0x06,
		  0xad, 0xf0, 0x0f, 0xe0, 0xd8, 0x07, 0xd2, 0x00 },
		{ 0x12, 0x70, 0xb4, 0x01, 0x06, 0x03, 0x02, 0x88,
		  0x20, 0x00, 0x78, 0x80, 0x00, 0x10, 0x55, 0x55,
		  0x80, 0x55, 0xc7, 0x08, 0x00, 0xb9, 0x5a, 0xf8,
		  0xda, 0x87, 0x3c, 0x19, 0x39, 0x39, 0x01, 0x06,
		  0xad, 0xf0, 0x0f, 0xe0, 0xd8, 0x07, 0xd2, 0x00 },
	},
	.tail = {
		{ 0xe0, 0xe1, 0x11, 0x17, 0xa1, 0xd6, 0x73, 0x18 },
		{ 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	},
};

static u8 *r858_register(struct r858_package *p, unsigned int index, u8 reg)
{
	if (reg < R858_TAIL_FIRST)
		return &p->receiver[index].local[reg - R858_LOCAL_FIRST];
	return &p->tail[index][reg - R858_TAIL_FIRST];
}

static void r858_cache_field(struct r858_package *p, unsigned int index, u8 reg,
			     u8 mask, u8 value)
{
	u8 *entry = r858_register(p, index, reg);

	*entry = (*entry & ~mask) | (value & mask);
}

static int r858_write(struct r858_package *p, unsigned int index, u8 reg,
		      const u8 *value, unsigned int count)
{
	u8 data[R858_LOCAL_COUNT + R858_TAIL_COUNT + 1];
	struct i2c_msg msg = {
		.addr = p->address[index],
		.len = count + 1,
		.buf = data,
	};
	int ret;

	if (!count || count > sizeof(data) - 1 || reg < R858_LOCAL_FIRST ||
	    reg + count > R858_TAIL_FIRST + R858_TAIL_COUNT)
		return -EINVAL;
	data[0] = reg;
	memcpy(data + 1, value, count);
	p->touched |= BIT(index);
	ret = i2c_transfer(p->bus[index], &msg, 1);
	return ret == 1 ? 0 : ret < 0 ? ret : -EREMOTEIO;
}

static int r858_update(struct r858_package *p, unsigned int index, u8 reg,
		       u8 mask, u8 value)
{
	u8 next = (*r858_register(p, index, reg) & ~mask) | (value & mask);
	int ret;

	ret = r858_write(p, index, reg, &next, 1);
	if (!ret)
		*r858_register(p, index, reg) = next;
	return ret;
}

static int r858_read(struct r858_package *p, unsigned int index, u8 *data,
		     unsigned int count)
{
	u8 zero = 0;
	struct i2c_msg msg[] = {
		{ .addr = p->address[index], .len = 1, .buf = &zero },
		{ .addr = p->address[index],
		  .flags = I2C_M_RD,
		  .len = count,
		  .buf = data },
	};
	unsigned int i;
	int ret;

	ret = i2c_transfer(p->bus[index], msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg))
		return ret < 0 ? ret : -EREMOTEIO;
	/* Register reads shift LSB first; the write protocol does not. */
	for (i = 0; i < count; i++)
		data[i] = bitrev8(data[i]);
	return 0;
}

struct r858_field {
	u8 reg;
	u8 mask;
	u8 value;
};

static int r858_fields(struct r858_package *p, unsigned int index,
		       const struct r858_field *fields, unsigned int count)
{
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = r858_update(p, index, fields[i].reg, fields[i].mask,
				  fields[i].value);
		if (ret)
			return ret;
	}
	return 0;
}

static int r858_write_local(struct r858_package *p, unsigned int index)
{
	return r858_write(p, index, R858_LOCAL_FIRST, p->receiver[index].local,
			  R858_LOCAL_COUNT);
}

static int r858_write_tail(struct r858_package *p, unsigned int index)
{
	return r858_write(p, index, R858_TAIL_FIRST, p->tail[index],
			  R858_TAIL_COUNT);
}

static void r858_set_xtal_fields(struct r858_package *p, unsigned int index)
{
	bool oscillator = p->xtal_role <= R858_XTAL_MASTER;
	u8 cap = oscillator ? p->xtal_cap : 0;
	u8 value = (cap / 2) << 3;

	if (cap & 1)
		value |= BIT(0);
	if (oscillator)
		value |= BIT(7);
	if (p->xtal_role == R858_XTAL_INDEPENDENT)
		value |= p->xtal_drive << 1;
	r858_cache_field(p, index, 29, 0xff, value);
}

static void r858_load_local(struct r858_package *p, unsigned int index,
			    const struct r858_preset *preset)
{
	memcpy(p->receiver[index].local, preset->local[index],
	       R858_LOCAL_COUNT);
	r858_set_xtal_fields(p, index);
}

static int r858_measure_adc(struct r858_package *p, unsigned int index, u8 *adc)
{
	u8 status[3];
	int ret;

	msleep(2);
	ret = r858_read(p, index, status, sizeof(status));
	if (!ret)
		*adc = FIELD_GET(GENMASK(7, 2), status[1]);
	return ret;
}

static int r858_wait_lock(struct r858_package *p, unsigned int index)
{
	u8 status[4];
	unsigned int poll;
	int ret;

	for (poll = 0; poll < 10; poll++) {
		ret = r858_read(p, index, status, sizeof(status));
		if (ret)
			return ret;
		if (status[3] & BIT(0))
			return 0;
		msleep(10);
	}
	return -ETIMEDOUT;
}

/* First-match frequency rules. Bounds are kHz, [first, end), so exact RF
 * exceptions have end = first + 1. The values are hardware field codes.
 * Reference: 24 MHz charge-pump and crystal-divider selection facts. */
struct r858_range {
	u32 first;
	u32 end;
	u8 value;
};

static u8 r858_range_value(const struct r858_range *table, unsigned int count,
			   u32 frequency, u8 otherwise)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		if (frequency >= table[i].first && frequency < table[i].end)
			return table[i].value;
	return otherwise;
}

static const struct r858_range r858_ref_primary[] = {
	{ 48000, 57000, 2 },	{ 528000, 528001, 2 }, { 552000, 552001, 2 },
	{ 576000, 576001, 2 },	{ 600000, 600001, 2 }, { 624000, 624001, 2 },
	{ 648000, 648001, 2 },	{ 672000, 672001, 2 }, { 696000, 696001, 2 },
	{ 720000, 720001, 2 },	{ 744000, 744001, 2 }, { 768000, 768001, 2 },
	{ 57000, 238000, 1 },	{ 488000, 497000, 1 }, { 848000, 857000, 1 },
	{ 900000, 1003000, 1 }, { 828000, 828001, 1 }, { 512000, 780001, 1 },
};

static const struct r858_range r858_ref_secondary[] = {
	{ 72000, 72001, 1 },	{ 48000, 114000, 2 },  { 114000, 198000, 1 },
	{ 357000, 412000, 1 },	{ 429000, 460000, 1 }, { 513000, 780001, 1 },
	{ 878000, 1003000, 1 }, { 216000, 216001, 1 }, { 360000, 360001, 1 },
};

static const struct r858_range r858_cp_primary[] = {
	{ 48000, 69000, 0 },   { 76000, 117000, 0 },  { 164000, 238000, 0 },
	{ 488000, 497000, 0 }, { 512000, 524000, 0 }, { 533000, 547000, 0 },
	{ 557000, 573000, 0 }, { 581000, 596000, 0 }, { 605000, 620000, 0 },
	{ 629000, 644000, 0 }, { 846000, 857000, 0 }, { 900000, 1000000, 0 },
	{ 69000, 76000, 5 },   { 648000, 648001, 5 }, { 672000, 672001, 5 },
	{ 696000, 696001, 5 }, { 720000, 720001, 5 }, { 744000, 744001, 5 },
	{ 768000, 768001, 5 }, { 600000, 600001, 3 }, { 117000, 198000, 4 },
	{ 864000, 900001, 5 }, { 528000, 528001, 1 },
};

static const struct r858_range r858_cp_secondary[] = {
	{ 48000, 198000, 4 },  { 376000, 405000, 0 },  { 436000, 452000, 0 },
	{ 513000, 516000, 0 }, { 878000, 1000000, 0 }, { 216000, 216001, 3 },
	{ 360000, 360001, 3 }, { 357000, 376000, 3 },  { 405000, 411000, 3 },
	{ 384000, 384001, 2 }, { 408000, 408001, 2 },  { 429000, 436000, 2 },
	{ 444000, 444001, 2 }, { 452000, 459000, 2 },  { 556000, 556001, 2 },
	{ 468000, 468001, 1 }, { 504000, 504001, 1 },  { 552000, 552001, 1 },
};

static const struct r858_range r858_offset_primary[] = {
	{ 69000, 92000, 1 },   { 164000, 237000, 1 }, { 528000, 528001, 1 },
	{ 552000, 552001, 1 }, { 576000, 576001, 1 },
};

static const struct r858_range r858_offset_secondary[] = {
	{ 48000, 114000, 1 },  { 165000, 171000, 1 }, { 357000, 376000, 1 },
	{ 405000, 412001, 1 }, { 429000, 436000, 1 }, { 452000, 460000, 1 },
	{ 444000, 444001, 1 }, { 468000, 468001, 1 }, { 504000, 504001, 1 },
	{ 552000, 552001, 1 },
};

static const struct r858_range r858_cpi_primary[] = {
	{ 69000, 92000, 1 },
	{ 164000, 238000, 1 },
};

static const struct r858_range r858_cpi_secondary[] = {
	{ 72000, 72001, 0 },   { 48000, 114000, 1 },  { 165000, 172000, 1 },
	{ 357000, 376000, 1 }, { 384000, 384001, 1 }, { 405000, 412000, 1 },
	{ 429000, 436000, 1 }, { 444000, 444001, 1 }, { 468000, 468001, 1 },
	{ 504000, 504001, 1 },
};

#define r858_lookup(table, frequency, fallback) \
	r858_range_value(table, ARRAY_SIZE(table), frequency, fallback)

struct r858_divider {
	u8 ratio;
	u8 code;
	u8 iq_current;
	u8 output_bias;
	bool primary_degeneracy;
};

/* VCO: 2270 <= f < 4540 MHz. Divider codes are not log2(ratio). */
static const struct r858_divider r858_dividers[] = {
	{ 2, 0, 0, 0xe0, false }, { 4, 1, 0, 0xe0, false },
	{ 8, 2, 0, 0x90, false }, { 16, 4, 1, 0xd0, true },
	{ 32, 6, 1, 0xd0, true }, { 64, 7, 1, 0xd0, false },
};

static int r858_program_pll(struct r858_package *p, unsigned int index,
			    u32 lo_khz, bool calibration)
{
	const struct r858_divider *divider = NULL;
	u32 rf_khz = lo_khz + p->receiver[index].profile->if_hz / 1000;
	u32 reference, vco, integer, fraction, modulus;
	u32 reference_divider = 1;
	u16 sigma_delta;
	u8 charge = 0, cpi = 0, offset = 0;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(r858_dividers); i++) {
		vco = lo_khz * r858_dividers[i].ratio;
		if (vco >= 2270000 && vco < 4540000) {
			divider = &r858_dividers[i];
			break;
		}
	}
	if (!divider)
		return -ERANGE;
	if (!calibration) {
		reference_divider =
			index ? r858_lookup(r858_ref_secondary, rf_khz, 2) :
				r858_lookup(r858_ref_primary, rf_khz, 2);
	}
	/* Shared-reference operation uses the reset loop-current tuple. The
	 * self-oscillating reference has additional RF-dependent loop settings. */
	if (!calibration && p->xtal_role == R858_XTAL_INDEPENDENT) {
		charge = index ? r858_lookup(r858_cp_secondary, rf_khz, 0) :
				 r858_lookup(r858_cp_primary, rf_khz, 0);
		cpi = index ? r858_lookup(r858_cpi_secondary, rf_khz, 0) :
			      r858_lookup(r858_cpi_primary, rf_khz, 0);
		offset = index ? r858_lookup(r858_offset_secondary, rf_khz, 0) :
				 r858_lookup(r858_offset_primary, rf_khz, 0);
	}
	reference = R858_REFERENCE_KHZ / reference_divider;
	modulus = 2 * reference;
	vco = lo_khz * divider->ratio;
	integer = vco / modulus;
	fraction = vco % modulus;
	/* Keep the fractional accumulator away from integer and half-cycle
	 * boundary spurs: 1/128 cycle at the ends, 1/256 around the centre. */
	if (fraction * 64 < reference) {
		fraction = 0;
	} else if (fraction * 64 > reference * 127) {
		fraction = 0;
		integer++;
	} else if (fraction < reference && fraction * 128 > reference * 127) {
		fraction = reference * 127 / 128;
	} else if (fraction > reference && fraction * 128 < reference * 129) {
		fraction = reference * 129 / 128;
	}
	if (integer < 13 || integer > 524)
		return -ERANGE;
	sigma_delta = div_u64((u64)fraction << 16, modulus);
	r858_cache_field(p, index, 21, 0x7f, (integer - 13) / 4);
	r858_cache_field(p, index, 25, 0xff, 0x04 | ((integer - 13) % 4));
	r858_cache_field(p, index, 22, 0xff, sigma_delta & 0xff);
	r858_cache_field(p, index, 23, 0xff, sigma_delta >> 8);
	r858_cache_field(p, index, 24, 0x80, 0x80);
	r858_cache_field(p, index, 26, 0xff, divider->code);
	r858_cache_field(p, index, 27, 0x3f, charge | (cpi << 5));
	r858_cache_field(p, index, 28, 0x67, reference_divider == 2 ? 0x20 : 0);
	if (!fraction)
		r858_cache_field(p, index, 28, 0x03, 0x03);
	r858_set_xtal_fields(p, index);
	r858_cache_field(p, index, 38, 0x0e, offset << 1);
	r858_cache_field(p, index, 47, 0x60, 0);
	r858_cache_field(p, index, 10, 0x40, divider->iq_current << 6);
	r858_cache_field(p, index, 41, 0xf0, divider->output_bias);
	r858_cache_field(p, index, 44, 0x08,
			 !index && divider->primary_degeneracy ? 0x08 : 0);
	ret = r858_update(p, index, 48, 0x03, !index || p->clock_out ? 2 : 3);
	if (ret)
		return ret;
	ret = r858_write_local(p, index);
	if (ret)
		return ret;
	msleep(10 * reference_divider);
	ret = r858_update(p, index, 38, 0x0c, 0x08);
	return ret ?: r858_wait_lock(p, index);
}

struct r858_rf_filter {
	u32 end;
	u8 capacitance;
	u8 notch;
};

/* MUX tracking filter cutovers in kHz; final row covers the upper band.
 * The calibration injection path uses its own LPF settings, not the cable
 * input path. These are register-field facts from the RF selection tables. */
static const struct r858_rf_filter r858_cable_filter[] = {
	{ 77000, 15, 10 }, { 85000, 15, 4 }, { 115000, 13, 3 },
	{ 125000, 11, 1 }, { 141000, 9, 0 }, { 157000, 8, 0 },
	{ 181000, 6, 0 },  { 205000, 3, 0 }, { ~0U, 0, 0 },
};

static const struct r858_rf_filter r858_injection_filter[] = {
	{ 73000, 8, 10 }, { 81000, 8, 4 },  { 89000, 8, 3 },
	{ 121000, 6, 1 }, { 145000, 4, 0 }, { 153000, 3, 0 },
	{ 177000, 2, 0 }, { 201000, 1, 0 }, { ~0U, 0, 0 },
};

static const struct r858_range r858_bpf[] = {
	{ 0, 580000, 0 },
	{ 580000, 660000, 6 },
	{ 660000, 900000, 3 },
};

static const struct r858_range r858_hpf[] = {
	{ 0, 480000, 0 },
	{ 480000, 550000, 1 },
	{ 550000, 700000, 2 },
};

static const struct r858_range r858_polyphase[] = {
	{ 0, 133000, 2 },
	{ 133000, 221000, 1 },
	{ 221000, 789000, 0 },
};

static unsigned int r858_iq_band(u32 lo_khz)
{
	static const u32 boundary[] = { 205000, 400000, 760000 };
	unsigned int band;

	for (band = 0; band < ARRAY_SIZE(boundary); band++)
		if (lo_khz < boundary[band])
			break;
	return band;
}

static int r858_apply_iq(struct r858_package *p, unsigned int index,
			 const struct r858_iq *iq)
{
	struct r858_field field[] = {
		{ 16, 0x1f, iq->gain },
		{ 17, 0x1f, iq->phase },
		{ 15, 0x06, iq->cap << 1 },
	};

	return r858_fields(p, index, field, ARRAY_SIZE(field));
}

static int r858_program_mux(struct r858_package *p, unsigned int index,
			    u32 lo_khz, bool calibration)
{
	const struct r858_rf_filter *rf_filter =
		calibration ? r858_injection_filter : r858_cable_filter;
	struct r858_iq zero = {};
	unsigned int band;
	int ret;

	for (band = 0; lo_khz >= rf_filter[band].end; band++)
		;
	r858_cache_field(p, index, 11, 0xff,
			 (lo_khz < 340000 ? 2 : 0) |
				 (rf_filter[band].capacitance << 2) |
				 (r858_lookup(r858_hpf, lo_khz, 3) << 6));
	r858_cache_field(p, index, 12, 0xff,
			 r858_lookup(r858_bpf, lo_khz, 7) |
				 (rf_filter[band].notch << 3));
	r858_cache_field(p, index, 14, 0x0c,
			 r858_lookup(r858_polyphase, lo_khz, 3) << 2);
	r858_cache_field(p, index, 9, 0x10,
			 calibration || lo_khz > 200000 ? 0x10 : 0);
	ret = r858_write_local(p, index);
	if (ret)
		return ret;
	return r858_apply_iq(
		p, index,
		calibration ? &zero :
			      &p->receiver[index].cal.iq[r858_iq_band(lo_khz)]);
}

/* The oscillator test uses N=79, /8, full 24 MHz reference, with observed
 * lock and VCO-bank window 43..55. Drive fields descend 3 (weak) to 0
 * (strong); retain one stronger step of headroom after a successful test. */
static int r858_check_crystal(struct r858_package *p)
{
	static const struct r858_field test[] = {
		{ 38, 0x0c, 0 }, { 21, 0x7f, 16 },   { 25, 0x03, 2 },
		{ 26, 0x07, 2 }, { 29, 0x78, 0x78 }, { 24, 0x7f, 0x40 },
		{ 24, 0x40, 0 },
	};
	u8 status[4], bank;
	int drive, ret;

	p->xtal_drive = 0;
	if (p->xtal_role > R858_XTAL_MASTER)
		return 0;
	ret = r858_fields(p, 0, test, ARRAY_SIZE(test));
	if (ret)
		return ret;
	for (drive = 3; drive >= 0; drive--) {
		ret = r858_update(p, 0, 29, 0x06, drive << 1);
		if (ret)
			return ret;
		msleep(20);
		ret = r858_read(p, 0, status, sizeof(status));
		if (ret)
			return ret;
		bank = FIELD_GET(GENMASK(6, 1), status[3]);
		if ((status[3] & BIT(0)) && bank >= 43 && bank <= 55) {
			p->xtal_drive = drive ? drive - 1 : 0;
			return 0;
		}
	}
	return -ETIMEDOUT;
}

static int r858_calibration_path(struct r858_package *p, unsigned int index,
				 const struct r858_preset *preset)
{
	int ret;

	memcpy(p->tail[0], preset->tail[0], R858_TAIL_COUNT);
	memcpy(p->tail[1], preset->tail[1], R858_TAIL_COUNT);
	ret = r858_write_tail(p, 0);
	if (ret)
		return ret;
	r858_load_local(p, index, preset);
	ret = r858_write_local(p, index);
	if (ret)
		return ret;
	/* Exactly one receiver may see the internal ring source. */
	ret = r858_update(p, 0, 14, 0x02, index ? 0 : 0x02);
	if (!ret)
		ret = r858_update(p, 1, 14, 0x02, index ? 0x02 : 0);
	if (!ret)
		ret = r858_update(p, 0, 28, 0x80, 0);
	if (!ret)
		ret = r858_update(p, 0, 49, 0x02, 0);
	return ret;
}

struct r858_ring {
	u8 multiplier;
	u8 first_divider;
	u8 second_divider;
	u8 divisor;
};

/* Ring oscillator: 8 * multiplier * reference, followed by two dividers.
 * The measured image points are 102, 326.4, 576 and 816 MHz at 24 MHz. */
static const struct r858_ring r858_rings[] = {
	{ 17, 0, 3, 32 },
	{ 17, 1, 1, 10 },
	{ 18, 2, 0, 6 },
	{ 17, 0, 0, 4 },
};

static int r858_calibrate_iq_point(struct r858_package *p, unsigned int index,
				   unsigned int point, struct r858_iq *result)
{
	const struct r858_ring *ring = &r858_rings[point];
	static const struct r858_field start[] = {
		{ 35, 0x0e, 0x06 }, { 37, 0x06, 0x06 }, { 44, 0x01, 0 },
		{ 15, 0x80, 0 },    { 16, 0x0f, 0 },	{ 17, 0x0f, 0 },
		{ 35, 0x01, 0x01 }, { 44, 0x01, 0x01 },
	};
	u32 ring_khz =
		8 * R858_REFERENCE_KHZ * ring->multiplier / ring->divisor;
	struct r858_iq trial = {};
	u8 status[3], adc, best = 0xff;
	unsigned int poll, cap;
	int gain, ret;

	ret = r858_update(p, 0, 48, 0xc8, ring->second_divider << 6);
	if (!ret)
		ret = r858_update(p, 0, 49, 0x30, ring->first_divider << 4);
	if (!ret)
		ret = r858_update(p, 0, 50, 0x1f, ring->multiplier);
	if (!ret)
		ret = r858_program_mux(p, index, ring_khz + R858_IMR_IF_KHZ,
				       true);
	if (!ret)
		ret = r858_program_pll(p, index, ring_khz + R858_IMR_IF_KHZ,
				       true);
	if (!ret)
		ret = r858_update(p, index, 16, 0x20, 0x20);
	if (!ret)
		ret = r858_update(p, 0, 53, 0x3e, 0);
	if (ret)
		return ret;
	for (gain = 15; gain > 0; gain--) {
		ret = r858_update(p, index, 37, 0xf0, gain << 4);
		if (ret)
			return ret;
		msleep(5);
		ret = r858_measure_adc(p, index, &adc);
		if (ret)
			return ret;
		if (adc < 50)
			break;
	}
	ret = r858_fields(p, index, start, ARRAY_SIZE(start));
	if (ret)
		return ret;
	msleep(55);
	for (poll = 0; poll < 31; poll++) {
		ret = r858_read(p, index, status, sizeof(status));
		if (ret)
			return ret;
		if (status[1] & BIT(index))
			break;
		msleep(1);
	}
	if (poll == 31)
		return -ETIMEDOUT;
	trial.gain = FIELD_GET(GENMASK(2, 0), status[2]);
	trial.phase = FIELD_GET(GENMASK(6, 4), status[2]);
	if (status[2] & BIT(3))
		trial.gain |= BIT(4);
	if (status[2] & BIT(7))
		trial.phase |= BIT(4);
	ret = r858_update(p, index, 15, 0x80, 0x80);
	if (!ret)
		ret = r858_update(p, index, 44, 0x01, 0);
	if (ret)
		return ret;
	for (cap = 0; cap < 3; cap++) {
		trial.cap = cap;
		ret = r858_apply_iq(p, index, &trial);
		if (ret)
			return ret;
		ret = r858_measure_adc(p, index, &adc);
		if (ret)
			return ret;
		if (adc < best) {
			best = adc;
			*result = trial;
		}
	}
	return 0;
}

static int r858_filter_sample(struct r858_package *p, unsigned int index,
			      u8 code, bool half, u8 *adc)
{
	int ret;

	ret = r858_update(p, index, 20, 0x7c, (code << 3) | (half ? 4 : 0));
	if (ret)
		return ret;
	msleep(5);
	return r858_measure_adc(p, index, adc);
}

static int r858_calibrate_filter(struct r858_package *p, unsigned int index,
				 struct r858_calibration *cal)
{
	static const u8 coarse[] = { 0, 2, 3 };
	u8 sample, baseline, end;
	unsigned int gain, band, code;
	int ret;

	ret = r858_calibration_path(p, index, &r858_filter_measurement);
	if (!ret)
		ret = r858_program_pll(
			p, index,
			68000 - p->receiver[index].profile->filter_cal_khz,
			true);
	if (!ret)
		ret = r858_update(p, index, 19, 0x02, 0);
	if (ret)
		return ret;
	for (gain = 0; gain < 16; gain++) {
		ret = r858_update(p, index, 37, 0xf0, gain << 4);
		if (ret)
			return ret;
		msleep(5);
		ret = r858_measure_adc(p, index, &sample);
		if (ret)
			return ret;
		if (sample > 40)
			break;
	}
	if (gain == 16)
		return -ERANGE;
	/* Coarse bandwidth must exhibit >16 ADC counts of useful adjustment
	 * between fine codes 0 and 13. No response is not a calibrated filter. */
	for (band = 0; band < ARRAY_SIZE(coarse); band++) {
		ret = r858_update(p, index, 16, 0xc0, coarse[band] << 6);
		if (!ret)
			ret = r858_filter_sample(p, index, 0, false, &baseline);
		if (!ret)
			ret = r858_filter_sample(p, index, 13, false, &end);
		if (ret)
			return ret;
		if (baseline > end + 16)
			break;
	}
	if (band == ARRAY_SIZE(coarse))
		return -ERANGE;
	cal->filter_coarse = coarse[band];
	ret = r858_filter_sample(p, index, 0, false, &baseline);
	if (ret)
		return ret;
	for (code = 1; code < 16; code++) {
		ret = r858_filter_sample(p, index, code, false, &sample);
		if (ret)
			return ret;
		if (baseline > sample + 16)
			break;
	}
	if (code == 16)
		return -ERANGE;
	cal->filter_fine = code;
	cal->filter_half = false;
	ret = r858_filter_sample(p, index, code - 1, true, &sample);
	if (ret)
		return ret;
	if (baseline > sample + 16) {
		cal->filter_fine--;
		cal->filter_half = true;
	}
	return 0;
}

/* Common production cable AGC settings. IF-dependent HPF and measured LPF
 * fields are supplied by the receiver's immutable profile/calibration.
 * RF threshold=0x29, filter threshold=0x39, discharge mode=normal (1). */
static const struct r858_field r858_cable_fields[] = {
	{ 8, 0x10, 0x10 },  { 10, 0x02, 0 },	{ 13, 0x60, 0 },
	{ 16, 0x20, 0x20 }, { 17, 0xe0, 0xa0 }, { 19, 0x41, 0 },
	{ 24, 0x02, 0x02 }, { 25, 0x20, 0 },	{ 30, 0xff, 0x38 },
	{ 31, 0xff, 0x29 }, { 32, 0x0f, 0x0c }, { 33, 0xff, 0x8a },
	{ 34, 0xf6, 0x54 }, { 35, 0xff, 0x5a }, { 36, 0xf0, 0x30 },
	{ 37, 0xff, 0x39 }, { 38, 0x30, 0 },	{ 39, 0xd2, 0 },
	{ 40, 0x01, 0x01 }, { 46, 0xc0, 0x80 }, { 47, 0x07, 0x04 },
};

static int r858_apply_cable(struct r858_package *p, unsigned int index,
			    const struct r858_calibration *cal)
{
	struct r858_field filter[] = {
		{ 18, 0xf8, p->receiver[index].profile->hpf_corner << 3 },
		{ 16, 0xc0, cal->filter_coarse << 6 },
		{ 20, 0x7c,
		  (cal->filter_fine << 3) | (cal->filter_half ? 4 : 0) },
	};
	int ret;

	ret = r858_fields(p, index, r858_cable_fields,
			  ARRAY_SIZE(r858_cable_fields));
	if (!ret)
		ret = r858_fields(p, index, filter, ARRAY_SIZE(filter));
	if (!ret)
		ret = r858_update(p, index, 48, 0x23,
				  0x20 | (!index || p->clock_out ? 2 : 3));
	/* Normal LNA/RF discharge (mode 1), common LNA top=1. */
	if (!ret)
		ret = r858_update(p, 0, 53, 0x3e, 0x36);
	if (!ret)
		ret = r858_update(p, 0, 55, 0x16, 0);
	if (!ret)
		ret = r858_update(p, 0, 54, 0xe0, 0xc0);
	if (!ret)
		ret = r858_update(p, 0, 49, 0x02, 0x02);
	if (!ret)
		ret = r858_update(p, 0, 51, 0x87,
				  p->loop_through ? 0x87 : 0x83);
	return ret;
}

static void r858_invalidate(struct r858_package *p)
{
	unsigned int i;

	p->ready = false;
	for (i = 0; i < 2; i++) {
		p->receiver[i].calibrated = false;
		p->receiver[i].tuned = false;
	}
}

/* No reception is published until BOTH receivers have their sole supported
 * LPF profile as well as all image points. Tuning the second frontend never
 * starts another shared-ring calibration while the first is receiving. */
static int r858_prepare(struct r858_package *p)
{
	static const u8 order[] = { 2, 1, 0, 3 };
	struct r858_calibration staged[2] = {};
	unsigned int index, point;
	int ret;

	r858_invalidate(p);
	for (index = 0; index < 2; index++) {
		/* Mark possibly powered before the transfer: even a failed setup
		 * must be eligible for best-effort power-down on detach. */
		p->receiver[index].awake = true;
		p->receiver[index].frequency = 0;
		p->active |= BIT(index);
		r858_load_local(p, index, &r858_normal);
		memcpy(p->tail[index], r858_normal.tail[index],
		       R858_TAIL_COUNT);
		ret = r858_write_tail(p, index);
		if (!ret)
			ret = r858_write_local(p, index);
		if (ret)
			return ret;
	}
	ret = r858_check_crystal(p);
	if (ret)
		return ret;
	for (index = 0; index < 2; index++) {
		ret = r858_calibration_path(p, index, &r858_image_measurement);
		if (ret)
			return ret;
		for (point = 0; point < ARRAY_SIZE(order); point++) {
			ret = r858_calibrate_iq_point(
				p, index, order[point],
				&staged[index].iq[order[point]]);
			if (ret)
				return ret;
		}
		ret = r858_calibrate_filter(p, index, &staged[index]);
		if (ret)
			return ret;
	}
	/* Discard the calibration register images, not the measured results. */
	for (index = 0; index < 2; index++) {
		r858_load_local(p, index, &r858_normal);
		memcpy(p->tail[index], r858_normal.tail[index],
		       R858_TAIL_COUNT);
		ret = r858_write_tail(p, index);
		if (!ret)
			ret = r858_write_local(p, index);
		if (ret)
			return ret;
	}
	for (index = 0; index < 2; index++) {
		ret = r858_apply_cable(p, index, &staged[index]);
		if (ret)
			return ret;
	}
	for (index = 0; index < 2; index++) {
		p->receiver[index].cal = staged[index];
		p->receiver[index].calibrated = true;
		p->receiver[index].awake = true;
		p->receiver[index].frequency = 0;
	}
	p->active = BIT(0) | BIT(1);
	p->ready = true;
	return 0;
}

/* Primary R51[6:5] controls the common LNA current (reference R858.c,
 * RF-current selection). Any receiving path in [330, 620) MHz needs the
 * high-current setting. A nonzero pending frequency replaces that endpoint's
 * committed frequency, so retuning either path cannot override its peer. */
static int r858_update_lna_current(struct r858_package *p, unsigned int active,
				   unsigned int pending_index, u32 pending_hz)
{
	unsigned int index;
	u32 frequency;
	bool high = false;

	for (index = 0; index < ARRAY_SIZE(p->receiver); index++) {
		const struct r858_receiver *receiver = &p->receiver[index];

		if (!(active & BIT(index)) || !receiver->awake)
			continue;
		if (pending_hz && index == pending_index)
			frequency = pending_hz;
		else if (receiver->tuned)
			frequency = receiver->frequency;
		else
			continue;
		if (frequency >= 330000000 && frequency < 620000000) {
			high = true;
			break;
		}
	}
	return r858_update(p, 0, 51, 0x60, high ? 0 : 0x40);
}

/* Production AGC thresholds: LNA=0x38, RF=0x29, mixer=0x5a, filter=0x39;
 * widening adds four VTH codes when the peer powers down.
 * Derive these settings from current active receivers, not historical flags. */
static int r858_update_thresholds(struct r858_package *p, unsigned int active)
{
	bool single = active == BIT(0) || active == BIT(1);
	struct r858_field threshold[] = {
		{ 31, 0xff, single ? 0x2d : 0x29 },
		{ 35, 0xff, single ? 0x5e : 0x5a },
		{ 37, 0xff, single ? 0x3d : 0x39 },
	};
	unsigned int index;
	int ret;

	ret = r858_update(p, 0, 30, 0xff, single ? 0x3c : 0x38);
	if (ret)
		return ret;
	for (index = 0; index < 2; index++) {
		if (!(active & BIT(index)))
			continue;
		ret = r858_fields(p, index, threshold, ARRAY_SIZE(threshold));
		if (ret)
			return ret;
	}
	return 0;
}

struct r858_power_step {
	u8 reg;
	u8 mask;
	u8 value;
	u16 delay_ms;
};

/* Individual power sequencing preserves crystal outputs and the common LNA.
 * Delays and register power fields are hardware facts from standby/wakeup. */
static const struct r858_power_step r858_power_down[] = {
	{ 9, 0xf5, 0xf5, 150 },	 { 8, 0x34, 0x34, 150 },
	{ 13, 0x10, 0, 150 },	 { 46, 0x01, 0x01, 150 },
	{ 26, 0x18, 0x18, 150 }, { 14, 0x30, 0x30, 150 },
	{ 25, 0x30, 0x30, 150 }, { 47, 0x60, 0x60, 150 },
	{ 8, 0x02, 0x02, 300 },	 { 8, 0x08, 0x08, 300 },
};

static int r858_sleep_receiver(struct r858_package *p, unsigned int index)
{
	struct r858_receiver *receiver = &p->receiver[index];
	unsigned int step, active = p->active & ~BIT(index);
	int ret, error, local_error;

	if (!(p->touched & BIT(index)) || !receiver->awake)
		return 0;
	memcpy(receiver->standby, receiver->local, R858_LOCAL_COUNT);
	error = r858_update_thresholds(p, active);
	ret = r858_update_lna_current(p, active, 0, 0);
	if (!error)
		error = ret;
	/* Shared-primary failure must not prevent local shutdown on a working
	 * peer bus. Attempt every local power step, preserving the first error. */
	local_error = r858_update(p, index, 10, 0xff, 0xff);
	if (!error)
		error = local_error;
	msleep(150);
	if (index) {
		ret = r858_update(p, index, 9, 0x02, 0x02);
		if (!local_error)
			local_error = ret;
		if (!error)
			error = ret;
	}
	msleep(150);
	for (step = 0; step < ARRAY_SIZE(r858_power_down); step++) {
		ret = r858_update(p, index, r858_power_down[step].reg,
				  r858_power_down[step].mask,
				  r858_power_down[step].value);
		if (!local_error)
			local_error = ret;
		if (!error)
			error = ret;
		msleep(r858_power_down[step].delay_ms);
	}
	receiver->tuned = false;
	if (!local_error) {
		p->active = active;
		receiver->awake = false;
	}
	return error;
}

static int r858_wake_receiver(struct r858_package *p, unsigned int index)
{
	struct r858_receiver *receiver = &p->receiver[index];
	static const struct r858_power_step restore[] = {
		{ 8, 0x02, 0, 300 },  { 47, 0x60, 0, 150 },
		{ 25, 0x30, 0, 150 }, { 14, 0x30, 0, 150 },
		{ 26, 0x18, 0, 150 },
	};
	static const struct r858_power_step enable[] = {
		{ 46, 0x01, 0, 150 },
		{ 13, 0x10, 0x10, 150 },
		{ 8, 0x34, 0x30, 150 },
		{ 9, 0xf5, 0x04, 150 },
	};
	unsigned int step;
	u8 value;
	int ret;

	if (receiver->awake)
		return 0;
	/* A partial power-up is not safely asleep on an I2C error. */
	receiver->awake = true;
	ret = r858_update_lna_current(p, p->active | BIT(index), index,
				      receiver->frequency);
	if (!ret)
		ret = r858_update(p, index, 8, 0x08, 0);
	if (ret)
		return ret;
	msleep(300);
	for (step = 0; step < ARRAY_SIZE(restore); step++) {
		value = receiver->standby[restore[step].reg - R858_LOCAL_FIRST];
		ret = r858_update(p, index, restore[step].reg,
				  restore[step].mask, value);
		if (ret)
			return ret;
		msleep(restore[step].delay_ms);
	}
	for (step = 0; step < ARRAY_SIZE(enable); step++) {
		ret = r858_update(p, index, enable[step].reg, enable[step].mask,
				  enable[step].value);
		if (ret)
			return ret;
		msleep(enable[step].delay_ms);
	}
	if (index) {
		ret = r858_update(p, index, 9, 0x02, 0);
		if (ret)
			return ret;
	}
	ret = r858_update(p, index, 10, 0xff, 0);
	if (ret)
		return ret;
	/* R30 of endpoint zero is also the common LNA threshold. Preserve its
	 * live value while restoring local receiver state. In particular there
	 * is NO saved copy of shared registers 48..55 to replay here. */
	if (!index)
		receiver->standby[30 - R858_LOCAL_FIRST] =
			*r858_register(p, 0, 30);
	memcpy(receiver->local, receiver->standby, R858_LOCAL_COUNT);
	ret = r858_write_local(p, index);
	if (!ret)
		ret = r858_update_thresholds(p, p->active | BIT(index));
	if (!ret && receiver->frequency)
		ret = r858_wait_lock(p, index);
	if (ret)
		return ret;
	p->active |= BIT(index);
	receiver->awake = true;
	receiver->tuned = receiver->frequency != 0;
	return 0;
}

static int r858_gate(struct dvb_frontend *fe, bool enable)
{
	return fe->ops.i2c_gate_ctrl ? fe->ops.i2c_gate_ctrl(fe, enable) : 0;
}

static int r858_gate_close(struct dvb_frontend *fe, int result)
{
	int ret = r858_gate(fe, false);

	return result ?: ret;
}

static int r858_init(struct dvb_frontend *fe)
{
	struct r858_state *state = fe->tuner_priv;
	struct r858_package *p = state->package;
	int ret;

	mutex_lock(&p->lock);
	ret = r858_gate(fe, true);
	if (!ret) {
		ret = p->ready ? 0 : r858_prepare(p);
		if (!ret)
			ret = r858_wake_receiver(p, state->index);
		ret = r858_gate_close(fe, ret);
	}
	if (ret)
		r858_invalidate(p);
	mutex_unlock(&p->lock);
	return ret;
}

static int r858_set_params(struct dvb_frontend *fe)
{
	struct r858_state *state = fe->tuner_priv;
	struct r858_package *p = state->package;
	struct r858_receiver *receiver = &p->receiver[state->index];
	const struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u32 if_khz = receiver->profile->if_hz / 1000;
	u32 rf_khz = c->frequency / 1000;
	int ret;

	/* DVB core supplies occupied bandwidth (symbol rate * 1.15), not the
	 * nominal channel width. Keep the calibrated 8 MHz hardware profile. */
	if (c->delivery_system != SYS_DVBC_ANNEX_A ||
	    c->bandwidth_hz > 8000000 ||
	    c->frequency < R858_MIN_LO_HZ + receiver->profile->if_hz ||
	    c->frequency > R858_MAX_RF_HZ)
		return -EINVAL;
	mutex_lock(&p->lock);
	receiver->tuned = false;
	ret = r858_gate(fe, true);
	if (!ret) {
		ret = p->ready ? 0 : r858_prepare(p);
		if (!ret)
			ret = r858_wake_receiver(p, state->index);
		if (!ret)
			ret = r858_apply_cable(p, state->index, &receiver->cal);
		if (!ret)
			ret = r858_update_lna_current(
				p, p->active, state->index, rf_khz * 1000);
		if (!ret)
			ret = r858_program_mux(p, state->index, rf_khz - if_khz,
					       false);
		if (!ret)
			ret = r858_program_pll(p, state->index, rf_khz - if_khz,
					       false);
		if (!ret)
			ret = r858_update_thresholds(p, p->active);
		ret = r858_gate_close(fe, ret);
	}
	if (ret) {
		r858_invalidate(p);
	} else {
		receiver->frequency = rf_khz * 1000;
		receiver->tuned = true;
	}
	mutex_unlock(&p->lock);
	return ret;
}

static int r858_get_if_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct r858_state *state = fe->tuner_priv;

	*frequency = state->package->receiver[state->index].profile->if_hz;
	return 0;
}

static int r858_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct r858_state *state = fe->tuner_priv;

	mutex_lock(&state->package->lock);
	*frequency = state->package->receiver[state->index].frequency;
	mutex_unlock(&state->package->lock);
	return 0;
}

static int r858_get_bandwidth(struct dvb_frontend *fe, u32 *bandwidth)
{
	*bandwidth = 8000000;
	return 0;
}

static int r858_get_status(struct dvb_frontend *fe, u32 *status)
{
	struct r858_state *state = fe->tuner_priv;
	struct r858_package *p = state->package;
	struct r858_receiver *receiver = &p->receiver[state->index];
	u8 data[4];
	int ret = 0;

	*status = 0;
	mutex_lock(&p->lock);
	if (!p->ready || !receiver->awake || !receiver->tuned)
		goto out;
	ret = r858_gate(fe, true);
	if (!ret) {
		ret = r858_read(p, state->index, data, sizeof(data));
		if (!ret && (data[3] & BIT(0)))
			*status = TUNER_STATUS_LOCKED;
		ret = r858_gate_close(fe, ret);
	}
	if (ret) {
		*status = 0;
		r858_invalidate(p);
	}
out:
	mutex_unlock(&p->lock);
	return ret;
}

static int r858_sleep(struct dvb_frontend *fe)
{
	struct r858_state *state = fe->tuner_priv;
	struct r858_package *p = state->package;
	int ret = 0;

	mutex_lock(&p->lock);
	if (!p->receiver[state->index].awake)
		goto out;
	ret = r858_gate(fe, true);
	if (!ret) {
		ret = r858_sleep_receiver(p, state->index);
		ret = r858_gate_close(fe, ret);
	}
	if (ret)
		r858_invalidate(p);
out:
	mutex_unlock(&p->lock);
	return ret;
}

static void r858_release(struct dvb_frontend *fe)
{
	struct r858_state *state = fe->tuner_priv;
	struct r858_package *p;
	unsigned int index;
	int ret, error;

	if (!state)
		return;
	p = state->package;
	error = r858_sleep(fe);
	mutex_lock(&r858_registry_lock);
	p->attached &= ~BIT(state->index);
	if (!p->attached) {
		mutex_lock(&p->lock);
		ret = r858_gate(fe, true);
		if (!error)
			error = ret;
		/* A failed open owns neither the transport nor the demod gate
		 * lock. Once open, shared-primary failures must not stop either
		 * endpoint's best-effort local power-down. */
		if (!ret) {
			for (index = 0; index < ARRAY_SIZE(p->receiver);
			     index++) {
				ret = r858_sleep_receiver(p, index);
				if (!error)
					error = ret;
			}
			error = r858_gate_close(fe, error);
		}
		if (error)
			dev_warn(&p->bus[state->index]->dev,
				 "R858 final package standby: %d\n", error);
		mutex_unlock(&p->lock);
		list_del(&p->list);
		i2c_put_adapter(p->bus[0]);
		i2c_put_adapter(p->bus[1]);
		kfree(p);
	} else if (error) {
		dev_warn(&p->bus[state->index]->dev,
			 "R858 detach standby: %d\n", error);
	}
	mutex_unlock(&r858_registry_lock);
	fe->tuner_priv = NULL;
	kfree(state);
}

static const struct dvb_tuner_ops r858_tuner_ops = {
	.info = {
		.name = "Rafael Micro R858 DVB-C",
		.frequency_min_hz = R858_MIN_LO_HZ + 5000000,
		.frequency_max_hz = R858_MAX_RF_HZ,
		.frequency_step_hz = 1000,
	},
	.init = r858_init,
	.set_params = r858_set_params,
	.get_if_frequency = r858_get_if_frequency,
	.get_frequency = r858_get_frequency,
	.get_bandwidth = r858_get_bandwidth,
	.get_status = r858_get_status,
	.sleep = r858_sleep,
	.release = r858_release,
};

static bool r858_same_wiring(const struct r858_package *p,
			     struct i2c_adapter *const bus[2],
			     const u8 address[2],
			     const struct r858_config *config)
{
	return p->bus[1] == bus[1] && p->address[1] == address[1] &&
	       p->xtal_role == config->xtal_role &&
	       p->xtal_cap == config->xtal_cap &&
	       p->loop_through == config->loop_through &&
	       p->clock_out == config->clock_out;
}

struct dvb_frontend *r858_attach(struct dvb_frontend *fe,
				 struct i2c_adapter *adapter,
				 const struct r858_config *config)
{
	struct r858_package *p = NULL, *entry;
	struct r858_state *state;
	struct i2c_adapter *bus[2];
	const struct r858_profile *profile[2];
	u8 address[2];
	unsigned int index, peer, i;

	if (!fe || !adapter || !config || fe->tuner_priv ||
	    config->tuner_index > 1 || config->xtal_hz != 24000000 ||
	    !r858_find_profile(config->if_hz) ||
	    !r858_find_profile(config->peer_if_hz) || config->xtal_cap > 31 ||
	    (unsigned int)config->xtal_role > R858_XTAL_SLAVE_NO_OUTPUT ||
	    config->i2c_address < 0x08 || config->i2c_address > 0x77 ||
	    config->peer_address < 0x08 || config->peer_address > 0x77 ||
	    ((config->xtal_role == R858_XTAL_MASTER ||
	      config->xtal_role == R858_XTAL_SLAVE_INPUT) &&
	     !config->clock_out))
		return NULL;
	index = config->tuner_index;
	peer = index ^ 1;
	profile[index] = r858_find_profile(config->if_hz);
	profile[peer] = r858_find_profile(config->peer_if_hz);
	bus[index] = adapter;
	bus[peer] = config->peer_adapter ?: adapter;
	address[index] = config->i2c_address;
	address[peer] = config->peer_address;
	if ((bus[0] == bus[1] && address[0] == address[1]) ||
	    !i2c_check_functionality(bus[0], I2C_FUNC_I2C) ||
	    !i2c_check_functionality(bus[1], I2C_FUNC_I2C))
		return NULL;
	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;
	mutex_lock(&r858_registry_lock);
	list_for_each_entry(entry, &r858_packages, list) {
		if (entry->bus[0] == bus[0] &&
		    entry->address[0] == address[0]) {
			p = entry;
			break;
		}
	}
	if (p) {
		if ((p->attached & BIT(index)) ||
		    !r858_same_wiring(p, bus, address, config) ||
		    p->receiver[index].profile != profile[index] ||
		    p->receiver[peer].profile != profile[peer])
			goto fail;
	} else {
		p = kzalloc(sizeof(*p), GFP_KERNEL);
		if (!p)
			goto fail;
		for (i = 0; i < 2; i++) {
			p->bus[i] = i2c_get_adapter(bus[i]->nr);
			if (p->bus[i] != bus[i])
				goto put_buses;
			p->address[i] = address[i];
		}
		mutex_init(&p->lock);
		p->xtal_role = config->xtal_role;
		p->xtal_cap = config->xtal_cap;
		p->loop_through = config->loop_through;
		p->clock_out = config->clock_out;
		for (i = 0; i < 2; i++) {
			p->receiver[i].profile = profile[i];
			r858_load_local(p, i, &r858_normal);
			memcpy(p->tail[i], r858_normal.tail[i],
			       R858_TAIL_COUNT);
		}
		list_add(&p->list, &r858_packages);
	}
	p->attached |= BIT(index);
	mutex_unlock(&r858_registry_lock);
	state->package = p;
	state->index = index;
	fe->tuner_priv = state;
	fe->ops.tuner_ops = r858_tuner_ops;
	fe->ops.tuner_ops.info.frequency_min_hz =
		R858_MIN_LO_HZ + config->if_hz;
	return fe;
put_buses:
	for (i = 0; i < 2; i++)
		if (p->bus[i])
			i2c_put_adapter(p->bus[i]);
	kfree(p);
fail:
	mutex_unlock(&r858_registry_lock);
	kfree(state);
	return NULL;
}
EXPORT_SYMBOL_GPL(r858_attach);

MODULE_DESCRIPTION("Independent Linux Rafael Micro R858 DVB-C tuner driver");
MODULE_LICENSE("GPL");
