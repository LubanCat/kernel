/******************************************************************************
 *
 * Copyright(c) 2007 - 2020  Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 * wlanfae <wlanfae@realtek.com>
 * Realtek Corporation, No. 2, Innovation Road II, Hsinchu Science Park,
 * Hsinchu 300, Taiwan.
 *
 * Larry Finger <Larry.Finger@lwfinger.net>
 *
 *****************************************************************************/
#include "../halbb_precomp.h"

#ifdef BB_8851B_SUPPORT

bool halbb_chk_pkg_valid_8851b(struct bb_info *bb, u8 bb_ver, u8 rf_ver)
{
	bool valid = true;

#if 0
	if (bb_ver >= X && rf_ver >= Y)
		valid = true;
	else if (bb_ver < X && rf_ver < Y)
		valid = true;
	else
		valid = false;
#endif

	if (!valid) {
		/*halbb_set_reg(bb, 0x1c3c, (BIT(0) | BIT(1)), 0x0);*/
		BB_WARNING("[%s] Pkg_ver{bb, rf}={%d, %d} disable all BB block\n",
			 __func__, bb_ver, rf_ver);
	}

	return valid;
}

bool halbb_chk_tx_idle_8851b(struct bb_info *bb, enum phl_phy_idx phy_idx)
{
	u8 tx_state = 0;
	bool idle = false;
	u32 dbg_port = 0x30002;

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	if (phy_idx == HW_PHY_1)
		return false;

	if (halbb_bb_dbg_port_racing(bb, DBGPORT_PRI_3)) {
		halbb_dbg_port_sel(bb, (u16)(dbg_port & 0xffff),
				   (u8)((dbg_port & 0xff0000) >> 16), 0x0, 0x1);
		BB_DBG(bb, DBG_PHY_CONFIG,
		       "*Set dbg_port=(0x%x)\n", dbg_port);
	} else {
		dbg_port = halbb_get_bb_dbg_port_idx(bb);
		BB_DBG(bb, DBG_PHY_CONFIG,
		       "[Set dbg_port fail!] Curr-DbgPort=0x%x\n", dbg_port);

		return false;
	}

	/* Release DBG port */
	halbb_release_bb_dbg_port(bb);

	tx_state = (u8)(halbb_get_bb_dbg_port_val(bb) & 0x3f);

	if (tx_state == 0)
		idle = true;
	else
		idle = false;

	return idle;
}

void halbb_stop_pmac_tx_8851b(struct bb_info *bb,
			      struct halbb_pmac_info *tx_info,
			      enum phl_phy_idx phy_idx)
{
	if (tx_info->is_cck) { // CCK
		if (tx_info->mode == CONT_TX) {
			halbb_set_reg(bb, 0x2300, BIT(26), 1);
			halbb_set_reg(bb, 0x2338, BIT(17), 0);
			halbb_set_reg(bb, 0x2300, BIT(28), 0);
			halbb_set_reg(bb, 0x2300, BIT(26), 0);
		} else if (tx_info->mode == PKTS_TX) {
			halbb_set_reg_cmn(bb, 0x9c4, BIT(4), 0, phy_idx);
		}
	} else { // OFDM
		if (tx_info->mode == CONT_TX)
			halbb_set_reg_cmn(bb, 0x9c4, BIT(0), 0, phy_idx);
		else if (tx_info->mode == PKTS_TX)
			halbb_set_reg_cmn(bb, 0x9c4, BIT(4), 0, phy_idx);
	}
}


void halbb_start_pmac_tx_8851b(struct bb_info *bb,
			       struct halbb_pmac_info *tx_info,
			       enum halbb_pmac_mode mode, u32 pkt_cnt,u16 period,
			       enum phl_phy_idx phy_idx)
{
	if (mode == CONT_TX) {
		if (tx_info->is_cck) {
			halbb_set_reg(bb, 0x2338, BIT(17), 1);
			halbb_set_reg(bb, 0x2300, BIT(28), 0);
		} else {
			halbb_set_reg_cmn(bb, 0x9c4, BIT(0), 1, phy_idx);
		}
	} else if (mode == PKTS_TX) {
		/*Tx_N_PACKET_EN */
		halbb_set_reg_cmn(bb, 0x9c4, BIT(4), 1, phy_idx);
		/*Tx_N_PERIOD */
		halbb_set_reg_cmn(bb, 0x9c4, 0xffffff00, period, phy_idx);
		/*Tx_N_PACKET */
		halbb_set_reg_cmn(bb, 0x9c8, 0xffffffff, pkt_cnt, phy_idx);
	} else if (mode == CCK_CARRIER_SIPPRESSION_TX) {
		if (tx_info->is_cck) {
			/*Carrier Suppress Tx*/
			halbb_set_reg(bb, 0x2338, BIT(18), 1);
			/*Disable scrambler at payload part*/
			halbb_set_reg(bb, 0x2304, BIT(26), 1);
		} else {
			return;
		}
		/*Tx_N_PACKET_EN */
		halbb_set_reg_cmn(bb, 0x9c4, BIT(4), 1, phy_idx);
		/*Tx_N_PERIOD */
		halbb_set_reg_cmn(bb, 0x9c4, 0xffffff00, period, phy_idx);
		/*Tx_N_PACKET */
		halbb_set_reg_cmn(bb, 0x9c8, 0xffffffff, pkt_cnt, phy_idx);
	}
	/*Tx_EN */
	halbb_set_reg_cmn(bb, 0x9c0, BIT(0), 1, phy_idx);
	halbb_set_reg_cmn(bb, 0x9c0, BIT(0), 0, phy_idx);
}


void halbb_set_pmac_tx_8851b(struct bb_info *bb, struct halbb_pmac_info *tx_info,
			     enum phl_phy_idx phy_idx)
{
	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	if (!tx_info->en_pmac_tx) {
		halbb_stop_pmac_tx_8851b(bb, tx_info, phy_idx);
		/* PD hit enable */
		halbb_set_reg_cmn(bb, 0xc3c, BIT(9), 0, phy_idx);
		halbb_set_reg(bb, 0x2344, BIT(31), 0);
		return;
	}
	/*Turn on PMAC */
	/* Tx */
	halbb_set_reg_cmn(bb, 0x0980, BIT(0), 1, phy_idx);
	/* Rx */
	halbb_set_reg_cmn(bb, 0x0980, BIT(16), 1, phy_idx);
	halbb_set_reg_cmn(bb, 0x0988, 0x3f, 0x3f, phy_idx);

	/* PD hit enable */
	halbb_set_reg(bb, 0x704, BIT(1), 0);
	halbb_set_reg_cmn(bb, 0xc3c, BIT(9), 1, phy_idx);
	halbb_set_reg(bb, 0x2344, BIT(31), 1);
	halbb_set_reg(bb, 0x704, BIT(1), 1);

	halbb_start_pmac_tx_8851b(bb, tx_info, tx_info->mode, tx_info->tx_cnt,
		       tx_info->period, phy_idx);
}

void halbb_set_tmac_tx_8851b(struct bb_info *bb, enum phl_phy_idx phy_idx)
{
	/* To do: 0x0d80[16] [25] / 0x0d88[5:0] Should be set to default value in parameter package*/
	/* Turn on TMAC */
	halbb_set_reg_cmn(bb, 0x0980, BIT(0), 0, phy_idx);
	halbb_set_reg_cmn(bb, 0x0980, BIT(16), 0, phy_idx);
	halbb_set_reg_cmn(bb, 0x0988, 0xfff, 0, phy_idx);
	halbb_set_reg_cmn(bb, 0x0994, 0xf0, 0, phy_idx);
	
	// PDP bypass from TMAC
	halbb_set_reg_cmn(bb, 0x09a4, BIT(10), 0, phy_idx);
	// TMAC Tx path
	halbb_set_reg_cmn(bb, 0x09a4, 0x1c, 0, phy_idx);
	// TMAC Tx power
	halbb_set_reg_cmn(bb, 0x09a4, BIT(16), 0, phy_idx);
}

void halbb_dpd_bypass_8851b(struct bb_info *bb, bool pdp_bypass,
			      enum phl_phy_idx phy_idx)
{
	halbb_set_reg_cmn(bb, 0x09a4, BIT(10), 1, phy_idx);
	halbb_set_reg_cmn(bb, 0x45b8, BIT(16), pdp_bypass, phy_idx);
}

void halbb_ic_hw_setting_init_8851b(struct bb_info *bb)
{
	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	if(phl_is_mp_mode(bb->phl_com)) {
		// r_en_sound_wo_ndp
		halbb_set_reg(bb, 0xd7c, BIT(1), 1);
	} else {
		// r_en_sound_wo_ndp
		halbb_set_reg(bb, 0xd7c, BIT(1), 0);
	}
	halbb_phy_efuse_get_info(bb, TD_HIDE_EFUSE_ADDR_8851B, 1, &(bb->bb_efuse_i.efuse_adc_td));

	/*Modify ch_trk coef.*/
	bb->pre_ch_trk_state = false;
}

void halbb_ic_hw_setting_8851b(struct bb_info *bb)
{
	u8 bw = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.bw;
	u16 id = bb->phl_com->id.id >> 16 & 0xFF;
	bool cur_id03_state = false;
	bool pre_id03_state = bb->pre_ch_trk_state;
	bool is_2g = 0;
	struct bb_env_mntr_info *env = &bb->bb_env_mntr_i;
	u32 cnt_diff = 0;
	struct bb_stat_info *stat = &bb->bb_stat_i;
	struct bb_cca_info *cca = &stat->bb_cca_i;
	struct bb_fa_info *fa = &stat->bb_fa_i;

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	if (bb->hal_com->band[bb->bb_phy_idx].cur_chandef.band == BAND_ON_24G)
		is_2g = true;

	/*r_bt_rx_mode_en*/
	if (is_2g && bb->bb_link_i.is_linked && (bb->bb_ch_i.rssi_min < (75 << 1))) {// if rssi < -35 dbm
		halbb_set_reg(bb, 0x4738, BIT(18), 0x0);
		BB_DBG(bb, DBG_PHY_CONFIG, "[BT] BTG enable, Is linked\n");
		BB_DBG(bb, DBG_PHY_CONFIG, "[BT][RSSI] rssi_min=0x%x\n", bb->bb_ch_i.rssi_min >> 1);
	} else {
		halbb_set_reg(bb, 0x4738, BIT(18), 0x1);
		BB_DBG(bb, DBG_PHY_CONFIG, "[BT] is_2g=%d, is_linked=%d\n", is_2g, bb->bb_link_i.is_linked);
		BB_DBG(bb, DBG_PHY_CONFIG, "[BT][RSSI] rssi_min=0x%x\n", bb->bb_ch_i.rssi_min >> 1);
	}

	/*Modify ch_trk coef.*/
	if (id == 0x3)
		cur_id03_state = (bw == CHANNEL_WIDTH_80) ? true : false;
	bb->pre_ch_trk_state = cur_id03_state;
	if (cur_id03_state != pre_id03_state)
		halbb_ch_trk_update(bb, cur_id03_state);

/*
	cnt_diff = cca->cnt_ofdm_cca - fa->cnt_ofdm_fail;
	// dynamic setting for anti-interference 
	if (env->nhm_ratio > 20) {
		halbb_set_reg(bb, 0x46F8, 0xffffffff, 0x13c6);
		halbb_set_reg(bb, 0x4440, BIT(31), 0x1);
	}
*/

}

bool halbb_set_pd_lower_bound_8851b(struct bb_info *bb, u8 bound,
				      enum channel_width bw,
				      enum phl_phy_idx phy_idx)
{
	/*
	Range of bound value:
	BW20: 95~33
	BW40: 92~30
	BW80: 89~27
	*/
	#ifdef HALBB_DIG_TDMA_SUPPORT
	struct bb_dig_info *bb_dig = &bb->bb_dig_i;
	struct bb_dig_op_unit *bb_dig_u = bb_dig->p_cur_dig_unit;
	#endif
	u8 bw_attenuation = 0;
	u8 subband_filter_atteniation = 7;
	u8 bound_idx = 0;
	bool rpt = true;

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	if (bound == 0) {
		halbb_set_reg_cmn(bb, 0x4860, BIT(30), 0, phy_idx);
		BB_DBG(bb, DBG_PHY_CONFIG,
		       "[PD Bound] Set Boundary to default!\n");
		return true;
	}

	bb->bb_cmn_backup_i.cur_pd_lower_bound = bound;

	if ((bw == CHANNEL_WIDTH_20) || (bw == CHANNEL_WIDTH_10) || (bw == CHANNEL_WIDTH_5)) {
		bw_attenuation = 0;
	} else if (bw == CHANNEL_WIDTH_40) {
		bw_attenuation = 3;
	} else if (bw == CHANNEL_WIDTH_80) {
		bw_attenuation = 6;
	} else {
		BB_DBG(bb, DBG_PHY_CONFIG,
		       "[PD Bound] Only support BW5/10/20/40/80 !\n");
		return false;
	}

	bound += (bw_attenuation + subband_filter_atteniation);
	// If Boundary dbm is odd, set it to even number
	bound = bound % 2 ? bound + 1 : bound;

	if (bound < 40) {
		BB_DBG(bb, DBG_PHY_CONFIG,
		       "[PD Bound] Threshold too high, set to highest level!\n");
		bound = 40;
		rpt = false;
	}

	if (bound > 102) {
		BB_DBG(bb, DBG_PHY_CONFIG,
		       "[PD Bound] Threshold too low, disable PD lower bound function!\n");
		halbb_set_reg_cmn(bb, 0x4860, BIT(30), 0, phy_idx);
		return true;
	}

	bound_idx =  (102 - bound) >> 1;

	halbb_set_reg_cmn(bb, 0x4860, 0x7c0, bound_idx, phy_idx);
	halbb_set_reg_cmn(bb, 0x4860, BIT(30), 1, phy_idx);

	BB_DBG(bb, DBG_PHY_CONFIG, "[PD Bound] Set Boundary Success!\n");

	#ifdef HALBB_DIG_TDMA_SUPPORT
	if ((bb_dig->dig_mode != DIG_TDMA) && (bb_dig->dig_mode != DIG_TDMA_ADV))
		return rpt;

	if (bb_dig_u->state_identifier == DIG_TDMA_LOW)
		bb_dig->dig_state_l_i.pd_low_th = bound;
	else
		bb_dig->dig_state_h_i.pd_low_th = bound;
	#endif

	return rpt;
}

bool halbb_set_pd_lower_bound_cck_8851b(struct bb_info *bb, u8 bound,
				      enum channel_width bw,
				      enum phl_phy_idx phy_idx)
{
	#ifdef HALBB_DIG_TDMA_SUPPORT
	struct bb_dig_info *bb_dig = &bb->bb_dig_i;
	struct bb_dig_op_unit *bb_dig_u = bb_dig->p_cur_dig_unit;
	#endif
	u8 bw_attenuation = 0;
	u8 subband_filter_atteniation = 5;
	s8 bound_tmp = 0;

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	if (bound == 0) {
		halbb_set_reg_cmn(bb, 0x23b0, BIT(23), 0, phy_idx);
		BB_DBG(bb, DBG_PHY_CONFIG,
			"[PD Bound] Set Boundary to default!\n");
		return true;
	}

	if ((bw == CHANNEL_WIDTH_20) || (bw == CHANNEL_WIDTH_10) || (bw == CHANNEL_WIDTH_5)) {
		bw_attenuation = 0;
	}
	else if (bw == CHANNEL_WIDTH_40) {
		bw_attenuation = 3;
	}
	else if (bw == CHANNEL_WIDTH_80) {
		bw_attenuation = 6;
	}
	else {
		BB_DBG(bb, DBG_PHY_CONFIG,
			"[PD Bound] Only support BW5/10/20/40/80 !\n");
		return true;
	}

	bound += (bw_attenuation + subband_filter_atteniation);
	bound_tmp = (-1) * MIN_2(bound, 128);

	halbb_set_reg_cmn(bb, 0x23b0, 0xff000000, bound_tmp, phy_idx);
	halbb_set_reg_cmn(bb, 0x23b4, 0xff000000, 0x7f, phy_idx);
	halbb_set_reg_cmn(bb, 0x23b0, BIT(23), 1, phy_idx);

	BB_DBG(bb, DBG_PHY_CONFIG, "[PD Bound] Set CCK Boundary Success!\n");

	#ifdef HALBB_DIG_TDMA_SUPPORT
	if ((bb_dig->dig_mode != DIG_TDMA) && (bb_dig->dig_mode != DIG_TDMA_ADV))
		return true;

	if (bb_dig_u->state_identifier == DIG_TDMA_LOW)
		bb_dig->dig_state_l_i.rssi_nocca_low_th= bound_tmp;
	else
		bb_dig->dig_state_h_i.rssi_nocca_low_th = bound_tmp;
	#endif

	return true;
}

u8 halbb_querry_pd_lower_bound_8851b(struct bb_info *bb, bool get_en_info, enum phl_phy_idx phy_idx)
{
	u8 tmp;

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	if (get_en_info)
		tmp = (u8)halbb_get_reg_cmn(bb, 0x4860, BIT(30), phy_idx);
	else
		tmp = bb->bb_cmn_backup_i.cur_pd_lower_bound;

	return tmp;
}

void halbb_pop_en_8851b(struct bb_info *bb, bool en, enum phl_phy_idx phy_idx)
{
	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	if (en) {
		halbb_set_reg_cmn(bb, 0x47D4, BIT(8), 1, phy_idx);

	} else {
		halbb_set_reg_cmn(bb, 0x47D4, BIT(8), 0, phy_idx);
	}
}

bool halbb_querry_pop_en_8851b(struct bb_info *bb, enum phl_phy_idx phy_idx)
{
	bool en;

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	en = (bool)halbb_get_reg_cmn(bb, 0x47D4, BIT(8), phy_idx);

	return en;
}

void halbb_ch_trk_update(struct bb_info *bb, bool wa_state)
{
	if (wa_state == true) {
		halbb_set_reg(bb, 0x43f8, 0x3F000, 0x20);
		halbb_set_reg(bb, 0x43f8, 0xFC0000, 0x20);
		halbb_set_reg(bb, 0x43fc, 0xFC0, 0x3c);
		halbb_set_reg(bb, 0x42a0, 0x6000, 0x2);
		halbb_set_reg(bb, 0x4300, 0x3F, 0xf);
		halbb_set_reg(bb, 0x4304, 0xFC0000, 0x3);
		halbb_set_reg(bb, 0x434c, 0x3F,0xf);
		halbb_set_reg(bb, 0x434c, 0xFC0, 0x6);
		halbb_set_reg(bb, 0x4350, 0x3F000000, 0x7);
		halbb_set_reg(bb, 0x4394, 0xE00, 0x3);
	} else if (wa_state == false) {
		halbb_set_reg(bb, 0x43f8, 0x3F000, 0x8);
		halbb_set_reg(bb, 0x43f8, 0xFC0000, 0x4);
		halbb_set_reg(bb, 0x43fc, 0xFC0, 0x4);
		halbb_set_reg(bb, 0x42a0, 0x6000, 0x1);
		halbb_set_reg(bb, 0x4300, 0x3F, 0x19);
		halbb_set_reg(bb, 0x4304, 0xFC0000, 0x19);
		halbb_set_reg(bb, 0x434c, 0x3F, 0x1b);
		halbb_set_reg(bb, 0x434c, 0xFC0, 0x17);
		halbb_set_reg(bb, 0x4350, 0x3F000000, 0x19);
		halbb_set_reg(bb, 0x4394, 0xE00, 0x1);
	}
}

#endif
