/******************************************************************************
 *
 * Copyright(c) 2016 - 2019 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/
#define _RTL8852C_BTC_C_

#include "../../hal_headers_le.h"
#include "../hal_btc.h"

#ifdef CONFIG_BTCOEX
#if defined (CONFIG_RTL8852C) || defined (CONFIG_RTL8842A)

#include "btc_8852c.h"

/* WL rssi threshold in % (dbm = % - 110)
 * array size limit by BTC_WL_RSSI_THMAX
 * BT rssi threshold in % (dbm = % - 100)
 * array size limit by BTC_BT_RSSI_THMAX
 */
static const u8 btc_8852c_wl_rssi_thres[BTC_WL_RSSI_THMAX] = {60, 50, 40, 30};
static const u8 btc_8852c_bt_rssi_thres[BTC_BT_RSSI_THMAX] = {50, 40, 30, 20};
/* for WL LNA2/TIA Level0/1 setup  */
static const struct btc_rf_cfg btc_8852c_rf_0[] = {
	{0xef, 0x1000}, {0x33, 0x0},  {0x3f, 0x15}, {0x33, 0x1},  {0x3f, 0x17},
	{0x33, 0x2}, 	{0x3f, 0x15}, {0x33, 0x3},  {0x3f, 0x17}, {0xef, 0x0},
	{0xef, 0x4000}, {0x33, 0x7},  {0x3e, 0x0},  {0x3f, 0x700},{0x33, 0x6},
	{0x3e, 0x0},    {0x3f, 0x700},{0x33, 0xf},  {0x3e, 0x0},  {0x3f, 0x700},
	{0x33, 0xe},    {0x3e, 0x0},  {0x3f, 0x700},{0x33, 0x17}, {0x3e, 0x0},
	{0x3f, 0x700}, 	{0x33, 0x16}, {0x3e, 0x0},  {0x3f, 0x700},{0xef, 0x0},
	{0xee, 0x10}, 	{0x33, 0x0},  {0x3f, 0x3},  {0x33, 0x1},  {0x3f, 0x3},
	{0x33, 0x2}, 	{0x3f, 0x3},  {0xee, 0x0}};
static const struct btc_rf_cfg btc_8852c_rf_1[] = {
	{0xef, 0x1000}, {0x33, 0x0},  {0x3f, 0x15}, {0x33, 0x1},    {0x3f, 0x5},
	{0x33, 0x2}, 	{0x3f, 0x15}, {0x33, 0x3},  {0x3f, 0x5},    {0xef, 0x0},
	{0xef, 0x4000}, {0x33, 0x7},  {0x3e, 0x0},  {0x3f, 0xa0700},{0x33, 0x6},
	{0x3e, 0x0}, {0x3f, 0xa0700}, {0x33, 0xf},  {0x3e, 0x0},{0x3f, 0xa0700},
	{0x33, 0xe},    {0x3e, 0x0},  {0x3f, 0xa0700},{0x33, 0x17}, {0x3e, 0x0},
	{0x3f, 0xa0700},{0x33, 0x16}, {0x3e, 0x0},  {0x3f, 0xa0700},{0xef, 0x0},
	{0xee, 0x10}, 	{0x33, 0x0},  {0x3f, 0x2},  {0x33, 0x1},    {0x3f, 0x2},
	{0x33, 0x2}, 	{0x3f, 0x2},  {0xee, 0x0}};

static struct btc_chip_ops btc_8852c_ops = {
	_8852c_rfe_type,
	_8852c_init_cfg,
	_8852c_wl_tx_power,
	_8852c_wl_rx_gain,
	_8852c_wl_btg_standby,
	_8852c_wl_req_mac,
	_8852c_get_reg_status,
	_8852c_bt_rssi
};

/* Set  WL/BT periodical moniter reg, Max size: CXMREG_MAX*/
/*
 * =========== CO-Rx AGC related monitor ===========
 * Co-Rx condition : WL RSSI < -35dBm (BB defined)
 * If WL RSSI < -35dBm --> 0x4aa4[18] = 0;
 * 0x4aa4[18] = 0: Not to adjust WL Rx AGC by GNT_BT_Rx.
 *		    To avoid WL Rx gain change.
 * 0x4aa4[18] = 1: When WL RSSI too strong, out of Co-Rx range
 *		    give up the Co-Rx WL enhancement.
 * 0x476c[31:24] = 0x20 (dBm); LNA6 op1dB
 * 0x4778[7:0]= 0x30 (dBm); TIA index0 LNA6 op1dB
 */
static struct fbtc_mreg btc_8852c_mon_reg[] = {
	{REG_MAC, 4, 0xda00},
	{REG_MAC, 4, 0xda04},
	{REG_MAC, 4, 0xda24},
	{REG_MAC, 4, 0xda30},
	{REG_MAC, 4, 0xda34},
	{REG_MAC, 4, 0xda38},
	{REG_MAC, 4, 0xda44},
	{REG_MAC, 4, 0xda48},
	{REG_MAC, 4, 0xda4c},
	{REG_MAC, 4, 0xd200},
	{REG_MAC, 4, 0xd220},
	{REG_BB, 4, 0x980},
	{REG_BB, 4, 0x4aa4},
	{REG_BB, 4, 0x4778},
	{REG_BB, 4, 0x476c},
	/*{REG_RF, 2, 0x2},*/ /* for RF, bytes->path, 1:A, 2:B... */
	/*{REG_RF, 2, 0x18},*/
	/*{REG_BT_RF, 2, 0x9},*/
	/*{REG_BT_MODEM, 2, 0xa} */
};

/* wl_tx_power: 255->original or BTC_WL_DEF_TX_PWR
 *              else-> bit7:signed bit, ex: 13:13dBm, 0x85:-5dBm
 * wl_rx_gain: 0->original, 1-> for Free-run, 2-> for BTG co-rx
 * bt_tx_power: decrease power, 0->original, 5 -> decreas 5dB.
 * bt_rx_gain: BT LNA constrain Level, 7->original
 */

struct btc_rf_trx_para btc_8852c_rf_ul[] = {
	{15, 0, 0, 7}, /* 0 -> original */
	{15, 2, 0, 7}, /* 1 -> for BT-connected ACI issue && BTG co-rx */
	{15, 0, 0, 7}, /* 2 ->reserved for shared-antenna */
	{15, 0, 0, 7}, /* 3- >reserved for shared-antenna */
	{15, 0, 0, 7}, /* 4 ->reserved for shared-antenna */
	{15, 1, 0, 7}, /* the below id is for non-shared-antenna free-run */
	{ 6, 1, 0, 7},
	{13, 1, 0, 7},
	{13, 1, 0, 7}
};

struct btc_rf_trx_para btc_8852c_rf_dl[] = {
	{15, 0, 0, 7}, /* 0 -> original */
	{15, 2, 0, 7}, /* 1 -> reserved for shared-antenna */
	{15, 0, 0, 7}, /* 2 ->reserved for shared-antenna */
	{15, 0, 0, 7}, /* 3- >reserved for shared-antenna */
	{15, 0, 0, 7}, /* 4 ->reserved for shared-antenna */
	{15, 1, 0, 7}, /* the below id is for non-shared-antenna free-run */
	{15, 1, 0, 7},
	{15, 1, 0, 7},
	{15, 1, 0, 7}
};

const struct btc_chip chip_8842a = {
	0x8842A, /* chip id */
	0x7, /* chip HW feature/parameter, refer to enum btc_chip_feature */
	0x7, /* desired bt_ver */
	0x070d0000, /* desired wl_fw btc ver */
	0x1, /* scoreboard version */
	0x1, /* mailbox version*/
	BTC_COEX_RTK_MODE, /* pta_mode */
	BTC_COEX_INNER, /* pta_direction */
	6, /* afh_guard_ch */
	btc_8852c_wl_rssi_thres, /* wl rssi threshold level */
	btc_8852c_bt_rssi_thres, /* bt rssi threshold level */
	(u8)2, /* rssi tolerance */
	&btc_8852c_ops, /* chip-dependent function */
	ARRAY_SIZE(btc_8852c_mon_reg),
	btc_8852c_mon_reg, /* wl moniter register */
	ARRAY_SIZE(btc_8852c_rf_ul),
	btc_8852c_rf_ul,
	ARRAY_SIZE(btc_8852c_rf_dl),
	btc_8852c_rf_dl
};

const struct btc_chip chip_8852c = {
	0x8852C, /* chip id */
	0x7, /* chip HW feature/parameter, refer to enum btc_chip_feature */
	0x7, /* desired bt_ver */
	0x070d0000, /* desired wl_fw btc ver */
	0x1, /* scoreboard version */
	0x1, /* mailbox version*/
	BTC_COEX_RTK_MODE, /* pta_mode */
	BTC_COEX_INNER, /* pta_direction */
	6, /* afh_guard_ch */
	btc_8852c_wl_rssi_thres, /* wl rssi threshold level */
	btc_8852c_bt_rssi_thres, /* bt rssi threshold level */
	(u8)2, /* rssi tolerance */
	&btc_8852c_ops, /* chip-dependent function */
	ARRAY_SIZE(btc_8852c_mon_reg),
	btc_8852c_mon_reg, /* wl moniter register */
	ARRAY_SIZE(btc_8852c_rf_ul),
	btc_8852c_rf_ul,
	ARRAY_SIZE(btc_8852c_rf_dl),
	btc_8852c_rf_dl
};

u8 _8852c_get_rf_path(struct btc_t *btc, u8 antnum)
{
	u8 ret;

	switch (antnum) {
		default:
		case 1:
			ret = btc->mdinfo.ant.single_pos;
			break;
		case 2:
			ret = RF_PATH_AB;
			break;
		case 3:
			ret = RF_PATH_ABC;
			break;
	}
	return ret;
}

void _8852c_rfe_type(struct btc_t *btc)
{
	struct rtw_phl_com_t *p = btc->phl;
	struct rtw_hal_com_t *h = btc->hal;
	struct btc_module *module = &btc->mdinfo;
	struct btc_dm *dm = &btc->dm;
	u8 tx_path_pos, rx_path_pos;

	PHL_TRACE(COMP_PHL_BTC, _PHL_DEBUG_, "[BTC], %s !! \n", __FUNCTION__);

	/* get from final capability of device  */
	module->rfe_type = p->dev_cap.rfe_type;
	module->kt_ver = h->cv;
	module->bt_solo = 0;
	module->switch_type = BTC_SWITCH_INTERNAL;
	module->wa_type |= BTC_WA_INIT_SCAN;
	dm->wl_trx_nss_en = 0;

	module->ant.type = BTC_ANT_SHARED;
	module->ant.num = 2;
	module->ant.isolation = 10;
	module->ant.diversity = 0;
	/* WL 1-stream+1-Ant is located at 0:s0(path-A) or 1:s1(path-B) */
	module->ant.single_pos = RF_PATH_B;
	module->ant.btg_pos = RF_PATH_B;
	module->ant.stream_cnt = (p->phy_cap[0].txss << 4) + p->phy_cap[0].rxss;

	if (module->rfe_type == 0) {
		dm->error |= BTC_DMERR_RFE_TYPE0;
		return;
	}

	/*rfe_type odd: 2-Ant(shared), even: 3-Ant(non-shared)*/
	module->ant.num = (module->rfe_type % 2)?  2 : 3;

	if (module->ant.num == 3) {
		module->ant.type = BTC_ANT_DEDICATED;
		module->bt_pos = BTC_BT_ALONE;
	} else {
		module->ant.type = BTC_ANT_SHARED;
		module->bt_pos = BTC_BT_BTG;
		dm->wl_trx_nss_en = 1;
	}
	dm->wl_trx_nss_en &= !!(GET_DEV_BTC_CAP(p).btc_deg_wifi_cap & BTC_DRG_WIFI_CAP_TRX1SS);
	tx_path_pos = _8852c_get_rf_path(btc, btc->phl->phy_cap[0].tx_path_num);
	rx_path_pos = _8852c_get_rf_path(btc, btc->phl->phy_cap[0].rx_path_num);
	module->ant.path_pos = (tx_path_pos << 4) + rx_path_pos;
}

void _8852c_get_reg_status(struct btc_t *btc, u8 type, void *status)
{
	struct btc_module *md = &btc->mdinfo;
	struct fbtc_mreg_val *pmreg = NULL;
	u8 *pos = (u8*)status;
	u32 *val = (u32*)status;
	u8 i = 0;
	u32 reg_val;
	u32 pre_agc_addr = R_BTC_BB_PRE_AGC_S1;

	switch (type) {
	case BTC_CSTATUS_TXDIV_POS:
		if (md->switch_type == BTC_SWITCH_INTERNAL) {
			*pos = BTC_ANT_DIV_MAIN;
			break;
		}

		break;
	case BTC_CSTATUS_RXDIV_POS:
		if (md->switch_type == BTC_SWITCH_INTERNAL) {
			*pos = BTC_ANT_DIV_MAIN;
			break;
		}

		break;
	case BTC_CSTATUS_BB_GNT_MUX:
		reg_val = hal_read32(btc->hal, R_BTC_BB_BTG_RX | 0x10000);
		*val = (reg_val & B_BTC_BB_GNT_MUX) == 0? 1 : 0;
		break;
	case BTC_CSTATUS_BB_GNT_MUX_MON:
		if (!btc->fwinfo.rpt_fbtc_mregval.cinfo.valid)
			return;
		pmreg = &btc->fwinfo.rpt_fbtc_mregval.finfo;
		/*  Check BB reg 0x10980 setup from period reg-mon*/
		for (i = 0; i < pmreg->reg_num; i++) {
			if (btc->chip->mon_reg[i].type == REG_BB &&
			    btc->chip->mon_reg[i].offset == R_BTC_BB_BTG_RX) {
				reg_val = pmreg->mreg_val[i];
			     	*val = (reg_val & B_BTC_BB_GNT_MUX) == 0? 1 : 0;
				break;
			} else if (i == pmreg->reg_num - 1) {
				*val = BTC_BB_GNT_NOTFOUND;
				return;
			}
		}
		break;
	case BTC_CSTATUS_BB_PRE_AGC:
		reg_val = hal_read32(btc->hal, pre_agc_addr | 0x10000);
		reg_val &= B_BTC_BB_PRE_AGC_MASK;
		*val = (reg_val == B_BTC_BB_PRE_AGC_VAL)? 1 : 0;
		break;
	case BTC_CSTATUS_BB_PRE_AGC_MON:
		if (!btc->fwinfo.rpt_fbtc_mregval.cinfo.valid)
			return;
		pmreg = &btc->fwinfo.rpt_fbtc_mregval.finfo;
		/*  Check BB reg Pre-AGC setup from period reg-mon*/
		for (i = 0; i < pmreg->reg_num; i++) {
			if (btc->chip->mon_reg[i].type == REG_BB &&
			    btc->chip->mon_reg[i].offset == pre_agc_addr) {
				break; /* found */
			} else if (i == pmreg->reg_num - 1) {
				*val = BTC_BB_PRE_AGC_NOTFOUND;
				return;
			}
		}

		reg_val = pmreg->mreg_val[i] & B_BTC_BB_PRE_AGC_MASK;
		*val = (reg_val == B_BTC_BB_PRE_AGC_VAL)? 1 : 0;
		break;
	}
}

void _8852c_wl_tx_power(struct btc_t *btc, u32 level)
{
	/*
	 * =========== All-Time WL Tx power control ==========
	 * (ex: all-time fix WL Tx 10dBm , don't care GNT _BT and GNT _LTE)
	 * Turn off per-packet power control
	 * 0xD220[1] = 0, 0xD220[2] = 0;
	 *
	 * disable using related power table
	 * 0xd208[20] = 0, 0xd208[21] = 0, 0xd21c[17] = 0, 0xd20c[29] = 0;
	 *
	 * enable force tx power mode and value
	 * 0xD200[9] = 1;
	 * 0xD200[8:0] = 0x28; S(9,2): 1step= 0.25dB, i.e. 40*0.25 = 10 dBm
	 * =========== per-packet Tx power control ===========
	 * (ex: GNT_BT = 1 -> 5dBm,  GNT _BT = 0 -> 10dBm)
	 * Turn on per-packet power control
	 * 0xD220[1] = 1, 0xD220[2] = 0;
	 * 0xD220[11:3] = 0x14; S(9,2): 1step = 0.25dB, i.e. 20*0.25 = 5 dBm
	 *
	 * disable using related power table
	 * 0xd208[20] = 0, 0xd208[21] = 0, 0xd21c[17] = 0, 0xd20c[29] = 0;
	 *
	 * enable force tx power mode and value
	 * 0xD200[9] = 1;
	 * 0xD200[8:0] = 0x28; S(9,2): 1 step = 0.25dB, i.e. 40*0.25 = 10 dBm
	 * ===================================================================
	 * level define:
	 *    if level = 255 -> back to original (btc don't control)
	 *    else in dBm --> bit7->signed bit, ex: 0xa-> +10dBm, 0x85-> -5dBm
	 * pwr_val define:
	 *     bit15~0  --> All-time (GNT_BT = 0) Tx power control
	 *     bit31~16 --> Per-Packet (GNT_BT = 1) Tx power control
	 */
	u32 pwr_val = bMASKDW, phi_idx = HW_PHY_0;
	bool en = false;

	if ((level & 0x7f) < BTC_WL_DEF_TX_PWR) { /* back to original */
		pwr_val = (level & 0x3f) << 2; /* to fit s(9,2) format */

		if (level & BIT(7)) /* negative value */
			pwr_val |= BIT(8); /* to fit s(9,2) format */
		pwr_val |= bMASKHW;
		en = true;
	}

	if (btc->hal->dbcc_en)
		phi_idx = btc->cx.wl.pta_req_mac;

	rtw_hal_rf_wlan_tx_power_control(btc->hal, phi_idx, ALL_TIME_CTRL,
					 pwr_val, en);
}

void _8852c_wl_rx_gain(struct btc_t *btc, u32 level)
{
	/* To improve BT ACI in co-rx
	 * level=0 Default: TIA 1/0= (LNA2,TIAN6) = (7,1)/(5,1) = 21dB/12dB
	 * level=1 Fix LNA2=5: TIA 1/0= (LNA2,TIAN6) = (5,0)/(5,1) = 18dB/12dB
	 */
	u32 mask = bMASKRF, n = 0, i = 0, val = 0;
	u32 type = (btc->mdinfo.ant.btg_pos << 8) | RTW_MAC_RF_CMD_OFLD;
	const struct btc_rf_cfg *rf = NULL;
	bool en;

	switch (level) {
	case 0: /* original */
	default:
		btc->dm.wl_lna2 = 0;
		break;
	case 1: /* for FDD free-run */
		btc->dm.wl_lna2 = 0;
		break;
	case 2: /* for BTG Co-Rx*/
		btc->dm.wl_lna2 = 1;
		break;
	}

	if (btc->dm.wl_lna2 == 0) {
		rf = btc_8852c_rf_0;
		n = ARRAY_SIZE(btc_8852c_rf_0);
	} else {
		rf = btc_8852c_rf_1;
		n = ARRAY_SIZE(btc_8852c_rf_1);
	}

	while (1) {
		en = (i == n-1) ? true : false;

		val = rf->val;
		/* bit[10] = 1 if non-shared-ant for 8851b */
		if (btc->hal->chip_id == CHIP_WIFI6_8851B &&
		    btc->mdinfo.ant.type == BTC_ANT_DEDICATED)
			val |= 0x4;

		_btc_io_w(btc, type, rf->addr, mask, val, en);
		i++;
		if (i > n-1)
			break;
		rf++;
	}
}

u8 _8852c_bt_rssi(struct btc_t *btc, u8 val)
{
	val = (val <= 127? 100 : (val >= 156? val - 156 : 0));
	val = val + 6; /* compensate offset */

	if (val > 100)
		val = 100;

	return (val);
}

void _8852c_wl_trx_mask(struct btc_t *btc, u32 type, u8 group, u32 val)
{
	_btc_io_w(btc, type, R_BTC_RF_LUT_WA, bMASKRF, group, false);
	_btc_io_w(btc, type, R_BTC_RF_LUT_WD0, bMASKRF, val, false);
}

void _8852c_wl_btg_standby(struct btc_t *btc, u32 state)
{
	u32 data = (state == 1) ? 0x179c : 0x208;
	u32 type = (btc->mdinfo.ant.btg_pos << 8) | RTW_MAC_RF_CMD_OFLD;

	/* set WL standby = Rx for GNT_BT_Tx = 1->0 settle issue */
	_btc_io_w(btc, type, R_BTC_RF_LUT_EN, bMASKRF, BIT(19), false);
	_btc_io_w(btc, type, R_BTC_RF_LUT_WA, bMASKRF, 0x1, false);
	_btc_io_w(btc, type, R_BTC_RF_LUT_WD1, bMASKRF, 0x620, false);
	_btc_io_w(btc, type, R_BTC_RF_LUT_WD0, bMASKRF, data, false);
	_btc_io_w(btc, type, R_BTC_RF_LUT_EN, bMASKRF, 0x0, true);
}

void _8852c_wl_req_mac(struct btc_t *btc, u8 mac_id)
{
	u32 val1;

	_read_cx_reg(btc, R_BTC_CFG, &val1);

	if (mac_id == HW_PHY_0)
		val1 = val1 & (~B_BTC_WL_SRC);
	else
		val1 = val1 | B_BTC_WL_SRC;

	_write_cx_reg(btc, R_BTC_CFG, val1);
}

void _8852c_wl_pri(struct btc_t *btc, u8 map, bool state)
{
	u32 bitmap = 0;
	u32 reg = R_BTC_COEX_WL_REQ;

	switch (map) {
	case BTC_PRI_MASK_TX_RESP:
	default:
		bitmap = B_BTC_RSP_ACK_HI;
		break;
	case BTC_PRI_MASK_BEACON:
		bitmap = B_BTC_TX_BCN_HI;
		break;
	case BTC_PRI_MASK_RX_CCK: /* Hi-Pri if rx_time > 8ms */
		bitmap = B_BTC_PRI_MASK_RX_TIME_V1;
		break;
	case BTC_PRI_MASK_TX_CCK: /* Hi-Pri if tx_time > 8ms */
		bitmap = B_BTC_PRI_MASK_TX_TIME;
		break;
	case BTC_PRI_MASK_TX_TRIG: /* Hi-Pri if Tx Trig Frame */
		bitmap = B_BTC_TX_TRI_HI;
		break;
	}

	_btc_io_w(btc, RTW_MAC_MAC_CMD_OFLD, reg, bitmap, state, false);
}

void _8852c_init_cfg(struct btc_t *btc)
{
	struct rtw_hal_com_t *h = btc->hal;
	struct btc_module *module = &btc->mdinfo;
	struct btc_ant_info *ant = &module->ant;
	struct btc_dm *dm = &btc->dm;
	struct btc_wl_trx_nss_para *trx_nss = &dm->wl_trx_nss;
	u32 path, type, path_min, path_max;

	PHL_INFO("[BTC], %s !! \n", __FUNCTION__);

	/* PTA init  */
	rtw_hal_mac_coex_init(h, btc->chip->pta_mode, btc->chip->pta_direction);

	/* set WL Tx response = Hi-Pri */
	_8852c_wl_pri(btc, BTC_PRI_MASK_TX_RESP, true);

	/* set WL Tx beacon = Hi-Pri */
	_8852c_wl_pri(btc, BTC_PRI_MASK_BEACON, true);

	/* set WL Tx trigger frame = Hi-Pri */
	_8852c_wl_pri(btc, BTC_PRI_MASK_TX_TRIG, true);

	/* for 1-Ant && 1-ss case: only 1-path */
	if (ant->stream_cnt == 0x11) {
		path_min = ant->single_pos;
		path_max = path_min;
	} else {
		path_min = RF_PATH_A;
		path_max = RF_PATH_B;
	}

	path = path_min;

	while (1) {
		type = (path << 8) | RTW_MAC_RF_CMD_OFLD;

		/* set rf gnt-debug off when init*/
		if (dm->btc_initing)
			_btc_io_w(btc, type, R_BTC_RF_BTG_CTRL,
				  bMASKRF, 0x0, false);

		/* set DEBUG_LUT_RFMODE_MASK = 1 to start trx-mask-setup */
		_btc_io_w(btc, type, R_BTC_RF_LUT_EN, bMASKRF, BIT(17), false);

		/* For 1T2R/1T1R at BTG, mask all WL Tx */
		if (module->bt_pos == BTC_BT_BTG && ant->btg_pos == path) {
			if (trx_nss->tx_limit) {
				_8852c_wl_trx_mask(btc, type, BTC_BT_SS_GROUP, 0x5dd);
				_8852c_wl_trx_mask(btc, type, BTC_BT_TX_GROUP, 0x55d);
				_8852c_wl_trx_mask(btc, type, BTC_BT_RX_GROUP, 0x5dd);
			} else {
				_8852c_wl_trx_mask(btc, type, BTC_BT_SS_GROUP, 0x5ff);
				_8852c_wl_trx_mask(btc, type, BTC_BT_TX_GROUP, 0x55f);
				_8852c_wl_trx_mask(btc, type, BTC_BT_RX_GROUP, 0x5df);
			}
		} else {
			_8852c_wl_trx_mask(btc, type, BTC_BT_SS_GROUP, 0x5ff);
			_8852c_wl_trx_mask(btc, type, BTC_BT_TX_GROUP, 0x5ff);
			_8852c_wl_trx_mask(btc, type, BTC_BT_RX_GROUP, 0x5df);
		}

		/* set DEBUG_LUT_RFMODE_MASK = 0 to stop trx-mask-setup */
		_btc_io_w(btc, type, R_BTC_RF_LUT_EN, bMASKRF, 0, false);

		path++;
		if (path > path_max)
			break;
	}

	/* set PTA break table */
	_btc_io_w(btc, RTW_MAC_MAC_CMD_OFLD, R_BTC_BREAK_TABLE, bMASKDW,
		  0xf0ffffff, false);

	/* enable BT counter 0xda10[1:0] = 2b'11 */
	_btc_io_w(btc, RTW_MAC_MAC_CMD_OFLD, R_BTC_BT_CNT_CFG,
		  B_BTC_BT_CNT_EN | B_BTC_BT_CNT_RST_V1,
		  B_BTC_BT_CNT_EN | B_BTC_BT_CNT_RST_V1, true);
}

#endif /* CONFIG_RTL8852C */
#endif

