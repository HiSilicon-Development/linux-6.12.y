// SPDX-License-Identifier: GPL-2.0-only
/* Copyright(c) 2007 - 2017 Realtek Corporation.
 *
 * RTL8822B QFN iFEM (RFE 12) data from Realtek's
 * rtl88x2BS v5.2.21.3_28325.20180614_COEX20180430-5e5e:
 * hal/phydm/rtl8822b/halhwimg8822b_bb.c (phy_reg_pg_type12) and
 * hal/phydm/rtl8822b/halhwimg8822b_rf.c (txpowertrack_type12).
 *
 * The vendor AGC, BB and radio A/B tables are already present unchanged
 * in rtw8822b_table.c, including their RFE 12 conditions. Its type12
 * power-limit table is identical to rtw8822b_txpwr_lmt_type0 (585 entries).
 */

#include "main.h"
#include "phy.h"
#include "rtw8822b_table.h"

static const struct rtw_phy_pg_cfg_pair rtw8822b_bb_pg_type12[] = {
	{ 0, 0, 0, 0x00000c20, 0xffffffff, 0x32343638, },
	{ 0, 0, 0, 0x00000c24, 0xffffffff, 0x36384042, },
	{ 0, 0, 0, 0x00000c28, 0xffffffff, 0x28303234, },
	{ 0, 0, 0, 0x00000c2c, 0xffffffff, 0x34363840, },
	{ 0, 0, 0, 0x00000c30, 0xffffffff, 0x26283032, },
	{ 0, 0, 1, 0x00000c34, 0xffffffff, 0x34363840, },
	{ 0, 0, 1, 0x00000c38, 0xffffffff, 0x26283032, },
	{ 0, 0, 0, 0x00000c3c, 0xffffffff, 0x34363840, },
	{ 0, 0, 0, 0x00000c40, 0xffffffff, 0x26283032, },
	{ 0, 0, 0, 0x00000c44, 0xffffffff, 0x38402224, },
	{ 0, 0, 1, 0x00000c48, 0xffffffff, 0x30323436, },
	{ 0, 0, 1, 0x00000c4c, 0xffffffff, 0x22242628, },
	{ 0, 1, 0, 0x00000e20, 0xffffffff, 0x32343638, },
	{ 0, 1, 0, 0x00000e24, 0xffffffff, 0x36384042, },
	{ 0, 1, 0, 0x00000e28, 0xffffffff, 0x28303234, },
	{ 0, 1, 0, 0x00000e2c, 0xffffffff, 0x34363840, },
	{ 0, 1, 0, 0x00000e30, 0xffffffff, 0x26283032, },
	{ 0, 1, 1, 0x00000e34, 0xffffffff, 0x34363840, },
	{ 0, 1, 1, 0x00000e38, 0xffffffff, 0x26283032, },
	{ 0, 1, 0, 0x00000e3c, 0xffffffff, 0x34363840, },
	{ 0, 1, 0, 0x00000e40, 0xffffffff, 0x26283032, },
	{ 0, 1, 0, 0x00000e44, 0xffffffff, 0x38402224, },
	{ 0, 1, 1, 0x00000e48, 0xffffffff, 0x30323436, },
	{ 0, 1, 1, 0x00000e4c, 0xffffffff, 0x22242628, },
	{ 1, 0, 0, 0x00000c24, 0xffffffff, 0x32343638, },
	{ 1, 0, 0, 0x00000c28, 0xffffffff, 0x24262830, },
	{ 1, 0, 0, 0x00000c2c, 0xffffffff, 0x30323436, },
	{ 1, 0, 0, 0x00000c30, 0xffffffff, 0x22242628, },
	{ 1, 0, 1, 0x00000c34, 0xffffffff, 0x30323436, },
	{ 1, 0, 1, 0x00000c38, 0xffffffff, 0x22242628, },
	{ 1, 0, 0, 0x00000c3c, 0xffffffff, 0x30323436, },
	{ 1, 0, 0, 0x00000c40, 0xffffffff, 0x22242628, },
	{ 1, 0, 0, 0x00000c44, 0xffffffff, 0x34361820, },
	{ 1, 0, 1, 0x00000c48, 0xffffffff, 0x26283032, },
	{ 1, 0, 1, 0x00000c4c, 0xffffffff, 0x18202224, },
	{ 1, 1, 0, 0x00000e24, 0xffffffff, 0x32343638, },
	{ 1, 1, 0, 0x00000e28, 0xffffffff, 0x24262830, },
	{ 1, 1, 0, 0x00000e2c, 0xffffffff, 0x30323436, },
	{ 1, 1, 0, 0x00000e30, 0xffffffff, 0x22242628, },
	{ 1, 1, 1, 0x00000e34, 0xffffffff, 0x30323436, },
	{ 1, 1, 1, 0x00000e38, 0xffffffff, 0x22242628, },
	{ 1, 1, 0, 0x00000e3c, 0xffffffff, 0x30323436, },
	{ 1, 1, 0, 0x00000e40, 0xffffffff, 0x22242628, },
	{ 1, 1, 0, 0x00000e44, 0xffffffff, 0x34361820, },
	{ 1, 1, 1, 0x00000e48, 0xffffffff, 0x26283032, },
	{ 1, 1, 1, 0x00000e4c, 0xffffffff, 0x18202224, },
};

RTW_DECL_TABLE_BB_PG(rtw8822b_bb_pg_type12);

/* Both paths and all three 5 GHz subbands use the same vendor curves.
 * The 2.4 GHz OFDM/CCK curves match the existing chip power-track table.
 */
const u8 rtw8822b_pwrtrk_type12_5g_n[RTW_PWR_TRK_TBL_SZ] = {
	0, 1, 1, 2, 2, 3, 3, 4, 4, 5,
	5, 6, 7, 7, 8, 8, 9, 9, 10, 10,
	11, 11, 12, 12, 13, 13, 14, 14, 15, 15
};

const u8 rtw8822b_pwrtrk_type12_5g_p[RTW_PWR_TRK_TBL_SZ] = {
	0, 1, 1, 2, 2, 3, 3, 4, 5, 5,
	6, 6, 7, 7, 8, 9, 9, 10, 10, 11,
	11, 12, 13, 13, 14, 14, 15, 15, 16, 17
};
