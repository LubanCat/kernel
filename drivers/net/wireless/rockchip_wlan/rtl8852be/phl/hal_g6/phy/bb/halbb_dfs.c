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
#include "halbb_precomp.h"

#ifdef HALBB_DFS_SUPPORT
void halbb_dfs(struct bb_info *bb)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	struct bb_link_info *link = &bb->bb_link_i;
	struct bb_env_mntr_info *env_mntr = &bb->bb_env_mntr_i;
	struct bb_stat_info *stat = &bb->bb_stat_i;
	struct bb_fa_info *fa = &stat->bb_fa_i;
	struct bb_ch_info *ch = &bb->bb_ch_i;
	u8 bw = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.bw;
	u8 chan = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.chan;
	u32 val = 0;

	halbb_show_cr_cnt(bb, BB_WD_DFS);

	if (!(bb->support_ability & BB_DFS)) {
		BB_DBG(bb, DBG_DFS, "Not support DFS function!\n");
		return;
	}

	if (bb->pause_ability & BB_ENVMNTR) {
		BB_DBG(bb, DBG_DFS, "[%s] Pause Env Mntr in LV=%d, Resume it with LV_2\n",
		       __func__, bb->pause_lv_table.lv_env_mntr);
		halbb_pause_func(bb, F_ENV_MNTR, HALBB_RESUME_NO_RECOVERY, HALBB_PAUSE_LV_2, 1, &val, HW_PHY_0);
	}

	if (bb->phl_com->dev_state & RTW_DEV_IN_DFS_CAC_PERIOD)
		bb_dfs->In_CAC_Flag = true;
	else
		bb_dfs->In_CAC_Flag = false;

	BB_DFS_DBG(bb, DYN_PRINT, "DFS Region Domain = %d, BW = %d, Channel = %d\n",
		bb_dfs->dfs_rgn_domain, bw, chan);
	BB_DFS_DBG(bb, DYN_PRINT, "[bb_sys_up_time/CAC_flag]: [%d,%d]\n",
		bb->bb_sys_up_time, bb_dfs->In_CAC_Flag);

	if (bb_dfs->dfs_dyn_setting_en) {
#ifdef HALBB_DFS_GEN2_SERIES
		if (bb->ic_sub_type == BB_IC_SUB_TYPE_8852C_8852D || bb->bb_80211spec == BB_BE_IC)
			halbb_dfs_dyn_setting_gen2(bb);
		else
#endif
			halbb_dfs_dyn_setting(bb);
	}

	BB_DFS_DBG(bb, DYN_PRINT, "[Is_link/rssi_min/rssi_max] = [%d, %d, %d]\n",
		link->is_linked, ch->rssi_min >> 1, ch->rssi_max >> 1);
	BB_DFS_DBG(bb, DYN_PRINT, "[TX_TP/RX_TP/T_TP/FA_CNT] = [%d, %d, %d, %d]\n",
		link->tx_tp, link->rx_tp, link->total_tp, fa->cnt_fail_all);
	BB_DFS_DBG(bb, DYN_PRINT, "[Tx_RTO/RX_RTO/I_RTO/N_RTO] = [%d, %d, %d, %d]\n",
		env_mntr->env_mntr_rpt_bg.nhm_tx_ratio, env_mntr->env_mntr_rpt_bg.nhm_cca_ratio, 
		env_mntr->env_mntr_rpt_bg.nhm_idle_ratio, env_mntr->env_mntr_rpt_bg.nhm_ratio);
	BB_DFS_DBG(bb, DYN_PRINT, "\n");
}

void halbb_dfs_init(struct bb_info *bb)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	struct bb_dfs_cr_info* cr = &bb->bb_cmn_hooker->bb_dfs_cr_i;

	BB_DBG(bb, DBG_DFS, "[%s]===>\n", __func__);

	/*DFS Parameter Initialization*/
	bb_dfs->dfs_rgn_domain = bb->phl_com->dfs_info.region_domain;
	halbb_dfs_rgn_dmn_dflt_cnfg(bb);
	bb_dfs->chrp_obsrv_flag = false;
	bb_dfs->dfs_sw_trgr_mode = false;
	bb_dfs->dfs_sw_trgr_mode_extra = false;
	bb_dfs->dfs_dbg_mode = false;
	bb_dfs->dfs_dyn_setting_en = true;
	bb_dfs->mask_fake_rpt_en = false;
	bb_dfs->dbg_print_component =
		/*HWDET_PRINT |		*/
		/*SWDET_PRINT |		*/
		/*BRK_PRINT |		*/
		/*DYN_PRINT |		*/
		/*RIVIL_PRINT |		*/
		0;
	bb_dfs->is_mic_w53 = false;
	bb_dfs->is_mic_w56 = false;
	bb_dfs->ppb_prcnt = DFS_PPB_IDLE_PRCNT;
	bb_dfs->pw_factor = PW_FTR_IDLE;
	bb_dfs->pri_factor = PRI_FTR_IDLE;
	bb_dfs->pw_ofst = 3;
	bb_dfs->pri_ofst = 2;
	bb_dfs->adv_pri_ofst = 2;

	bb_dfs->fk_dfs_num_th = 3;
	bb_dfs->dfs_tp_th = 2;
	bb_dfs->dfs_idle_prd_th = 50;
	bb_dfs->dfs_rx_rto_th = 15;
	bb_dfs->dfs_fa_th = 50;
	bb_dfs->dfs_nhm_th = 2;
	bb_dfs->pw_max_th = 660;  // lng pulse max pw = 110us
	bb_dfs->dfs_rssi_th = 38;

	bb_dfs->In_CAC_Flag = true;
	bb_dfs->first_dyn_set_flag = true;
	bb_dfs->dyn_reset_flag = true;

	bb_dfs->pw_diff_th = 12;
	bb_dfs->pri_diff_th = 5;
	bb_dfs->pw_lng_chrp_diff_th = 20;
	bb_dfs->lng_rdr_cnt = 0;
	bb_dfs->lng_rdr_cnt_sg1 = 0;
	bb_dfs->lng_rdr_cnt_pre = 0;
	bb_dfs->lng_rdr_cnt_sg1_pre = 0;

	bb_dfs->invalid_lng_pulse_h_th = 3;
	bb_dfs->invalid_lng_pulse_l_th = 1;

	bb_dfs->rpt_rdr_cnt = 0;
	bb_dfs->fake_rdr_cnt_th = 10;
	bb_dfs->pri_mask_th = 3;

	bb_dfs->detect_state = DFS_Adaptive_State;  //avoid FRD when init
	bb_dfs->adap_detect_cnt_init = 2;
	bb_dfs->adap_detect_cnt_add = 2;
	bb_dfs->adap_detect_cnt_th = 30;
	bb_dfs->adap_detect_cnt = 0;
	bb_dfs->adap_detect_cnt_all = 0;
	bb_dfs->adap_detect_brk_en = false;

	bb_dfs->rpt_sg_history = 0;
	bb_dfs->rpt_sg_history_all = 0;

	#ifdef HALBB_DFS_GEN2_SERIES
		bb_dfs->sub20_detect_en = false;
		bb_dfs->l2h_init_val = 0xbd;
		bb_dfs->l2h_val = 0xb6;  // set to -74dBm for gen2 dfs FCC detection BW test
	#endif
}

void halbb_mac_cfg_dfs_rpt(struct bb_info *bb, bool rpt_en)
{
	struct rtw_hal_com_t *hal_com = bb->hal_com;
	struct hal_mac_dfs_rpt_cfg conf = {
		.rpt_en = rpt_en,
		.rpt_num_th = 1, /* 0:29 , 1:61 , 2:93 , 3:125 */
		.rpt_en_to = rpt_en,
		.rpt_to = 3, /* 1:20ms , 2:40ms , 3:80ms */
	};
	rtw_hal_mac_cfg_dfs_rpt(hal_com, &conf);
}

void halbb_radar_detect_reset(struct bb_info *bb)
{
	struct bb_dfs_cr_info *cr = &bb->bb_cmn_hooker->bb_dfs_cr_i;

#ifdef HALBB_DBCC_SUPPORT
	if (bb->hal_com->dbcc_en) {
		if (bb->bb_phy_idx == HW_PHY_0) {
			halbb_mac_cfg_dfs_rpt(bb, false);
			halbb_set_reg_cmn(bb, cr->dfs_en, cr->dfs_en_m, 0, HW_PHY_0);
			halbb_mac_cfg_dfs_rpt(bb, true);
			halbb_set_reg_cmn(bb, cr->dfs_en, cr->dfs_en_m, 1, HW_PHY_0);
		}
		else
		{
			halbb_mac_cfg_dfs_rpt(bb, false);
			halbb_set_reg_cmn(bb, cr->dfs_en_p1, cr->dfs_en_p1_m, 0, HW_PHY_1);
			halbb_mac_cfg_dfs_rpt(bb, true);
			halbb_set_reg_cmn(bb, cr->dfs_en_p1, cr->dfs_en_p1_m, 1, HW_PHY_1);
		}
	}
	else
#endif
	{
		halbb_mac_cfg_dfs_rpt(bb, false);
		halbb_set_reg_cmn(bb, cr->dfs_en, cr->dfs_en_m, 0, HW_PHY_0);
		halbb_mac_cfg_dfs_rpt(bb, true);
		halbb_set_reg_cmn(bb, cr->dfs_en, cr->dfs_en_m, 1, HW_PHY_0);
	}
}

void halbb_radar_detect_disable(struct bb_info *bb)
{
	struct bb_dfs_cr_info *cr = &bb->bb_cmn_hooker->bb_dfs_cr_i;

#ifdef HALBB_DBCC_SUPPORT
	if (bb->hal_com->dbcc_en) {
		if (bb->bb_phy_idx == HW_PHY_0) {
			halbb_mac_cfg_dfs_rpt(bb, false);
			halbb_set_reg_cmn(bb, cr->dfs_en, cr->dfs_en_m, 0, HW_PHY_0);
		}
		else
		{
			halbb_mac_cfg_dfs_rpt(bb, false);
			halbb_set_reg_cmn(bb, cr->dfs_en_p1, cr->dfs_en_p1_m, 0, HW_PHY_1);
		}
	}
	else
#endif
	{
		halbb_mac_cfg_dfs_rpt(bb, false);
		halbb_set_reg_cmn(bb, cr->dfs_en, cr->dfs_en_m, 0, HW_PHY_0);
	}
}

void halbb_radar_detect_enable(struct bb_info *bb)
{
	struct bb_dfs_cr_info *cr = &bb->bb_cmn_hooker->bb_dfs_cr_i;

#ifdef HALBB_DBCC_SUPPORT
	if (bb->hal_com->dbcc_en) {
		if (bb->bb_phy_idx == HW_PHY_0) {
			halbb_mac_cfg_dfs_rpt(bb, true);
			halbb_set_reg_cmn(bb, cr->dfs_en, cr->dfs_en_m, 1, HW_PHY_0);
		}
		else
		{
			halbb_mac_cfg_dfs_rpt(bb, true);
			halbb_set_reg_cmn(bb, cr->dfs_en_p1, cr->dfs_en_p1_m, 1, HW_PHY_1);
		}
	}
	else
#endif
	{
		halbb_mac_cfg_dfs_rpt(bb, true);
		halbb_set_reg_cmn(bb, cr->dfs_en, cr->dfs_en_m, 1, HW_PHY_0);
	}
}

void halbb_dfs_enable_cac_flag(struct bb_info* bb)
{
	bb->bb_dfs_i.In_CAC_Flag = true;
}

void halbb_dfs_disable_cac_flag(struct bb_info* bb)
{
	bb->bb_dfs_i.In_CAC_Flag = false;
}

void halbb_dfs_change_dmn(struct bb_info *bb, u8 ch, u8 bw)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	bool is_w53_band = false, is_w56_band = false;

	bb_dfs->dfs_rgn_domain = bb->phl_com->dfs_info.region_domain;
	halbb_dfs_rgn_dmn_dflt_cnfg(bb);
	if ((bw == CHANNEL_WIDTH_160) && (ch >= 36) && (ch <= 48))
		is_w53_band = true;
	else if ((ch >= 52) && (ch <= 64))
		is_w53_band = true;
	else if ((ch >= 100) && (ch <= 144))
		is_w56_band = true;

	if (bb_dfs->dfs_rgn_domain == DFS_REGD_JAP)
		halbb_dfs_rgn_dmn_cnfg_by_ch(bb, is_w53_band, is_w56_band);
}

bool halbb_is_dfs_band(struct bb_info *bb, u8 ch, u8 bw)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	bool is_w53_band = false, is_w56_band = false;


	if ((bw == CHANNEL_WIDTH_160) && (ch >= 36) && (ch <= 48))
		is_w53_band = true;
	else if ((ch >= 52) && (ch <= 64))
		is_w53_band = true;
	else if ((ch >= 100) && (ch <= 144))
		is_w56_band = true;

	if (bb_dfs->dfs_rgn_domain == DFS_REGD_JAP)
		halbb_dfs_rgn_dmn_cnfg_by_ch(bb, is_w53_band, is_w56_band);

	if ((is_w53_band) || (is_w56_band))
		return true;
	else
		return false;
}

void halbb_dfs_rgn_dmn_dflt_cnfg(struct bb_info *bb)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	u8 i;
#ifdef HALBB_DFS_GEN2_SERIES
	/* PW unit: 400ns ; PRI unit: 12.8us */
	/*FCC *//*Type {1,2,3,4,6,0,L,R}*/
	/*Ref. type  judge indepently*/
	u16 pw_min_fcc_tab_gen2[DFS_RDR_TYP_NUM] = {2,2,15,27,2,2,125,2};
	u16 pw_max_fcc_tab_gen2[DFS_RDR_TYP_NUM] = {2,13,25,50,2,2,250,2};
	u16 pri_min_fcc_tab_gen2[DFS_RDR_TYP_NUM] = {40,12,15,15,26,111,78,1000};
	u16 pri_max_fcc_tab_gen2[DFS_RDR_TYP_NUM] = {240,18,40,40,27,112,157,1000};
	//u8 ppb_fcc_tab_gen2[DFS_RDR_TYP_NUM] = {18,23,16,12,14,18,15,13};
	u8 ppb_fcc_tab_gen2[DFS_RDR_TYP_NUM] = {11,14,10,7,8,11,9,8};
	u8 ppb_fcc_tp_tab_gen2[DFS_RDR_TYP_NUM] = {9,11,8,6,7,9,7,6};

	/*ETSI*//*Type {1,2,3,4,5,6,CN_R,R}*/
	/*Ref. type judge indepently*/
	u16 pw_min_etsi_tab_gen2[DFS_RDR_TYP_NUM] = {1,1,1,50,1,1,2,2};
	u16 pw_max_etsi_tab_gen2[DFS_RDR_TYP_NUM] = {13,38,75,75,5,5,2,2};
	u16 pri_min_etsi_tab_gen2[DFS_RDR_TYP_NUM] = {78,48,19,19,195,65,1000,1000};
	u16 pri_max_etsi_tab_gen2[DFS_RDR_TYP_NUM] = {391,391,34,40,261,196,1000,1000};
	//u8 ppb_etsi_tab_gen2[DFS_RDR_TYP_NUM] = {10,15,24,20,20,30,15,13};
	u8 ppb_etsi_tab_gen2[DFS_RDR_TYP_NUM] = {6,9,15,12,12,18,9,8};
	u8 ppb_etsi_tp_tab_gen2[DFS_RDR_TYP_NUM] = {4,7,9,8,10,12,7,6};

	/*KCC*/
	u16 pw_min_kcc_tab_gen2[DFS_RDR_TYP_NUM] = { 2,2,5,2,0,0,0,0 };
	u16 pw_max_kcc_tab_gen2[DFS_RDR_TYP_NUM] = { 2,2,5,2,1000,1000,1000,1000 };
	u16 pri_min_kcc_tab_gen2[DFS_RDR_TYP_NUM] = { 111,43,236,26,0,0,0,0 };
	u16 pri_max_kcc_tab_gen2[DFS_RDR_TYP_NUM] = { 112,44,237,27,0,0,0,0 };
	//u8 ppb_kcc_tab_gen2[DFS_RDR_TYP_NUM] = { 18,10,20,20,255,255,255,255 };
	u8 ppb_kcc_tab_gen2[DFS_RDR_TYP_NUM] = { 11,6,12,12,255,255,255,255 };
	u8 ppb_kcc_tp_tab_gen2[DFS_RDR_TYP_NUM] = { 9,5,10,10,255,255,255,255 };
#endif
	/* PW unit: 200ns ; PRI unit: 25us */
	/*FCC *//*Type {1,2,3,4,6,0,L}*/
	u16 pw_min_fcc_tab[DFS_RDR_TYP_NUM] = {5,5,30,55,5,5,250,0};
	u16 pw_max_fcc_tab[DFS_RDR_TYP_NUM] = {5,25,50,100,5,5,500,1000};
	u16 pri_min_fcc_tab[DFS_RDR_TYP_NUM] = {20,6,8,8,13,57,40,0};
	u16 pri_max_fcc_tab[DFS_RDR_TYP_NUM] = {123,10,20,20,14,58,80,0};
	u8 ppb_fcc_tab[DFS_RDR_TYP_NUM] = {11,14,10,7,9,11,9,255};
	u8 ppb_fcc_tp_tab[DFS_RDR_TYP_NUM] = {9,11,8,6,7,9,7,6};

	/*ETSI*//*Type {1,2,3,4,5,6,R,CN_R}*/
	/*reduce ppb of Type1 from 10 to 9 in order to increase detection rate*/
	/*increase type3 pw max to 30us due to SRRC spec*/
	u16 pw_min_etsi_tab[DFS_RDR_TYP_NUM] = {2,2,2,100,2,2,5,5};
	u16 pw_max_etsi_tab[DFS_RDR_TYP_NUM] = {25,75,150,150,15,15,5,5};
	u16 pri_min_etsi_tab[DFS_RDR_TYP_NUM] = {40,25,12,10,100,33,57,40};
	u16 pri_max_etsi_tab[DFS_RDR_TYP_NUM] = {200,200,18,20,134,100,58,40};
	u8 ppb_etsi_tab[DFS_RDR_TYP_NUM] = {6,9,15,12,12,18,11,11};
	u8 ppb_etsi_tp_tab[DFS_RDR_TYP_NUM] = {4,7,12,10,10,15,7,6};

	/*KCC*//* reduce ppb of Type 3 from 70 to 20 due to buffer size */
	u16 pw_min_kcc_tab[DFS_RDR_TYP_NUM] = { 5,5,10,5,0,0,0,0 };
	u16 pw_max_kcc_tab[DFS_RDR_TYP_NUM] = { 6,6,11,6,1000,1000,1000,1000 };
	u16 pri_min_kcc_tab[DFS_RDR_TYP_NUM] = { 57,22,121,13,0,0,0,0 };
	u16 pri_max_kcc_tab[DFS_RDR_TYP_NUM] = { 58,23,122,14,0,0,0,0 };
	u8 ppb_kcc_tab[DFS_RDR_TYP_NUM] = { 11,6,12,12,255,255,255,255 };
	u8 ppb_kcc_tp_tab[DFS_RDR_TYP_NUM] = { 9,5,10,10,255,255,255,255 };

#ifdef HALBB_DFS_GEN2_SERIES
	if (bb->ic_sub_type == BB_IC_SUB_TYPE_8852C_8852D || bb->bb_80211spec == BB_BE_IC) {
		if (bb_dfs->dfs_rgn_domain == DFS_REGD_FCC) {
			bb_dfs->l_rdr_exst_flag = true;
			halbb_mem_cpy(bb, &bb_dfs->pw_min_tab, &pw_min_fcc_tab_gen2, sizeof(bb_dfs->pw_min_tab));
			halbb_mem_cpy(bb, &bb_dfs->pw_max_tab, &pw_max_fcc_tab_gen2, sizeof(bb_dfs->pw_max_tab));
			halbb_mem_cpy(bb, &bb_dfs->pri_min_tab, &pri_min_fcc_tab_gen2, sizeof(bb_dfs->pri_min_tab));
			halbb_mem_cpy(bb, &bb_dfs->pri_max_tab, &pri_max_fcc_tab_gen2, sizeof(bb_dfs->pri_max_tab));
			halbb_mem_cpy(bb, &bb_dfs->ppb_tab, &ppb_fcc_tab_gen2, sizeof(bb_dfs->ppb_tab));
			halbb_mem_cpy(bb, &bb_dfs->ppb_tp_tab, &ppb_fcc_tp_tab_gen2, sizeof(bb_dfs->ppb_tp_tab));
		}
		if (bb_dfs->dfs_rgn_domain == DFS_REGD_ETSI) {
			bb_dfs->l_rdr_exst_flag = false;
			halbb_mem_cpy(bb, &bb_dfs->pw_min_tab, &pw_min_etsi_tab_gen2, sizeof(bb_dfs->pw_min_tab));
			halbb_mem_cpy(bb, &bb_dfs->pw_max_tab, &pw_max_etsi_tab_gen2, sizeof(bb_dfs->pw_max_tab));
			halbb_mem_cpy(bb, &bb_dfs->pri_min_tab, &pri_min_etsi_tab_gen2, sizeof(bb_dfs->pri_min_tab));
			halbb_mem_cpy(bb, &bb_dfs->pri_max_tab, &pri_max_etsi_tab_gen2, sizeof(bb_dfs->pri_max_tab));
			halbb_mem_cpy(bb, &bb_dfs->ppb_tab, &ppb_etsi_tab_gen2, sizeof(bb_dfs->ppb_tab));
			halbb_mem_cpy(bb, &bb_dfs->ppb_tp_tab, &ppb_etsi_tp_tab_gen2, sizeof(bb_dfs->ppb_tp_tab));
		}
		if (bb_dfs->dfs_rgn_domain == DFS_REGD_KCC) {
			bb_dfs->l_rdr_exst_flag = false;
			halbb_mem_cpy(bb, &bb_dfs->pw_min_tab, &pw_min_kcc_tab_gen2, sizeof(bb_dfs->pw_min_tab));
			halbb_mem_cpy(bb, &bb_dfs->pw_max_tab, &pw_max_kcc_tab_gen2, sizeof(bb_dfs->pw_max_tab));
			halbb_mem_cpy(bb, &bb_dfs->pri_min_tab, &pri_min_kcc_tab_gen2, sizeof(bb_dfs->pri_min_tab));
			halbb_mem_cpy(bb, &bb_dfs->pri_max_tab, &pri_max_kcc_tab_gen2, sizeof(bb_dfs->pri_max_tab));
			halbb_mem_cpy(bb, &bb_dfs->ppb_tab, &ppb_kcc_tab_gen2, sizeof(bb_dfs->ppb_tab));
			halbb_mem_cpy(bb, &bb_dfs->ppb_tp_tab, &ppb_kcc_tp_tab_gen2, sizeof(bb_dfs->ppb_tp_tab));
		}
	}
	else
#endif
	{
		if (bb_dfs->dfs_rgn_domain == DFS_REGD_FCC) {
			bb_dfs->l_rdr_exst_flag = true;
			halbb_mem_cpy(bb, &bb_dfs->pw_min_tab, &pw_min_fcc_tab, sizeof(bb_dfs->pw_min_tab));
			halbb_mem_cpy(bb, &bb_dfs->pw_max_tab, &pw_max_fcc_tab, sizeof(bb_dfs->pw_max_tab));
			halbb_mem_cpy(bb, &bb_dfs->pri_min_tab, &pri_min_fcc_tab, sizeof(bb_dfs->pri_min_tab));
			halbb_mem_cpy(bb, &bb_dfs->pri_max_tab, &pri_max_fcc_tab, sizeof(bb_dfs->pri_max_tab));
			halbb_mem_cpy(bb, &bb_dfs->ppb_tab, &ppb_fcc_tab, sizeof(bb_dfs->ppb_tab));
			halbb_mem_cpy(bb, &bb_dfs->ppb_tp_tab, &ppb_fcc_tp_tab, sizeof(bb_dfs->ppb_tp_tab));
		}
		if (bb_dfs->dfs_rgn_domain == DFS_REGD_ETSI) {
			bb_dfs->l_rdr_exst_flag = false;
			halbb_mem_cpy(bb, &bb_dfs->pw_min_tab, &pw_min_etsi_tab, sizeof(bb_dfs->pw_min_tab));
			halbb_mem_cpy(bb, &bb_dfs->pw_max_tab, &pw_max_etsi_tab, sizeof(bb_dfs->pw_max_tab));
			halbb_mem_cpy(bb, &bb_dfs->pri_min_tab, &pri_min_etsi_tab, sizeof(bb_dfs->pri_min_tab));
			halbb_mem_cpy(bb, &bb_dfs->pri_max_tab, &pri_max_etsi_tab, sizeof(bb_dfs->pri_max_tab));
			halbb_mem_cpy(bb, &bb_dfs->ppb_tab, &ppb_etsi_tab, sizeof(bb_dfs->ppb_tab));
			halbb_mem_cpy(bb, &bb_dfs->ppb_tp_tab, &ppb_etsi_tp_tab, sizeof(bb_dfs->ppb_tp_tab));
		}
		if (bb_dfs->dfs_rgn_domain == DFS_REGD_KCC) {
			bb_dfs->l_rdr_exst_flag = false;
			halbb_mem_cpy(bb, &bb_dfs->pw_min_tab, &pw_min_kcc_tab, sizeof(bb_dfs->pw_min_tab));
			halbb_mem_cpy(bb, &bb_dfs->pw_max_tab, &pw_max_kcc_tab, sizeof(bb_dfs->pw_max_tab));
			halbb_mem_cpy(bb, &bb_dfs->pri_min_tab, &pri_min_kcc_tab, sizeof(bb_dfs->pri_min_tab));
			halbb_mem_cpy(bb, &bb_dfs->pri_max_tab, &pri_max_kcc_tab, sizeof(bb_dfs->pri_max_tab));
			halbb_mem_cpy(bb, &bb_dfs->ppb_tab, &ppb_kcc_tab, sizeof(bb_dfs->ppb_tab));
			halbb_mem_cpy(bb, &bb_dfs->ppb_tp_tab, &ppb_kcc_tp_tab, sizeof(bb_dfs->ppb_tp_tab));
		}
	}
	halbb_mem_cpy(bb, &bb_dfs->ppb_typ_th, &bb_dfs->ppb_tab, sizeof(bb_dfs->ppb_typ_th));
}

void halbb_dfs_rgn_dmn_cnfg_by_ch(struct bb_info *bb, bool w53_band,
				  bool w56_band)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	u8 i;
#ifdef HALBB_DFS_GEN2_SERIES
	/* PW unit: 400ns ; PRI unit: 12.8us */
	/*Type {1,2,3,4,5,6,7,8}*/ /*Type3~8 determine by long pulse*/
	/*W2 = 20~110us for Type3.4, W2 = 30~32us for Type5~8*/
	u16 pw_min_mic_w53_tab_gen2[DFS_RDR_TYP_NUM] = {1,1,50,50,75,75,75,75};
	u16 pw_max_mic_w53_tab_gen2[DFS_RDR_TYP_NUM] = {13,38,275,275,80,80,80,80};
	u16 pri_min_mic_w53_tab_gen2[DFS_RDR_TYP_NUM] = {78,48,7,7,6,6,6,6};
	u16 pri_max_mic_w53_tab_gen2[DFS_RDR_TYP_NUM] = {391,391,391,391,71,85,87,106};
	//u8 ppb_mic_w53_tab_gen2[DFS_RDR_TYP_NUM] = {10,15,22,22,30,25,24,20};
	u8 ppb_mic_w53_tab_gen2[DFS_RDR_TYP_NUM] = {6,9,15,15,18,15,15,12};
	u8 ppb_mic_w53_tp_tab_gen2[DFS_RDR_TYP_NUM] = {4,7,12,12,12,12,12,10};

	/*Type {1,2,3,4,5,6,L,Hop}*/
	/*Type1.2 judge indepently*/
	u16 pw_min_mic_w56_tab_gen2[DFS_RDR_TYP_NUM] = {1,2,5,2,15,27,125,2};
	u16 pw_max_mic_w56_tab_gen2[DFS_RDR_TYP_NUM] = {2,2,5,13,25,50,250,3};
	u16 pri_min_mic_w56_tab_gen2[DFS_RDR_TYP_NUM] = {1000,1000,312,11,15,15,78,26};
	u16 pri_max_mic_w56_tab_gen2[DFS_RDR_TYP_NUM] = {1000,1000,313,18,40,40,157,27};
	//u8 ppb_mic_w56_tab_gen2[DFS_RDR_TYP_NUM] = {11,11,11,23,16,12,15,14};
	u8 ppb_mic_w56_tab_gen2[DFS_RDR_TYP_NUM] = {11,11,11,14,10,7,9,8};
	u8 ppb_mic_w56_tp_tab_gen2[DFS_RDR_TYP_NUM] = {5,5,5,11,8,6,7,7};
#endif
	/* PW unit: 200ns ; PRI unit: 25us */
	/*Type {1,2,3,4,5,6,7,8}*/ /*Type3~8 determine by long pulse*/
	/*W2 = 20~110us for Type3.4, W2 = 30~32us for Type5~8*/
	u16 pw_min_mic_w53_tab[DFS_RDR_TYP_NUM] = {2,2,100,100,150,150,150,150};
	u16 pw_max_mic_w53_tab[DFS_RDR_TYP_NUM] = {25,75,550,550,160,160,160,160};
	u16 pri_min_mic_w53_tab[DFS_RDR_TYP_NUM] = {40,25,3,3,2,2,2,2};
	u16 pri_max_mic_w53_tab[DFS_RDR_TYP_NUM] = {200,200,200,200,36,43,44,54};
	u8 ppb_mic_w53_tab[DFS_RDR_TYP_NUM] = {6,9,12,12,16,14,14,12};
	u8 ppb_mic_w53_tp_tab[DFS_RDR_TYP_NUM] = {4,7,11,11,15,12,12,10};

	/*Type {1,2,3,4,5,6,L,8}*/
	/* reduce ppb of Type 3 from 18 to 10 and reduce pri of Type 3 from 160 to 80 due to buffer size */
	u16 pw_min_mic_w56_tab[DFS_RDR_TYP_NUM] = {2,5,10,5,30,55,250,5};
	u16 pw_max_mic_w56_tab[DFS_RDR_TYP_NUM] = {3,5,11,25,50,100,500,5};
	u16 pri_min_mic_w56_tab[DFS_RDR_TYP_NUM] = {55,57,80,6,8,8,40,13};
	u16 pri_max_mic_w56_tab[DFS_RDR_TYP_NUM] = {56,58,160,10,20,20,80,14};
	u8 ppb_mic_w56_tab[DFS_RDR_TYP_NUM] = {11,11,11,14,10,7,9,8};
	u8 ppb_mic_w56_tp_tab[DFS_RDR_TYP_NUM] = {5,5,5,11,8,6,7,7};

#ifdef HALBB_DFS_GEN2_SERIES
	if (bb->ic_sub_type == BB_IC_SUB_TYPE_8852C_8852D || bb->bb_80211spec == BB_BE_IC) {
		if (bb_dfs->dfs_rgn_domain == DFS_REGD_JAP) {
			if (w53_band) {
				bb_dfs->is_mic_w53 = true;
				bb_dfs->is_mic_w56 = false;
				bb_dfs->l_rdr_exst_flag = false;
				halbb_mem_cpy(bb, &bb_dfs->pw_min_tab, &pw_min_mic_w53_tab_gen2, sizeof(bb_dfs->pw_min_tab));
				halbb_mem_cpy(bb, &bb_dfs->pw_max_tab, &pw_max_mic_w53_tab_gen2, sizeof(bb_dfs->pw_max_tab));
				halbb_mem_cpy(bb, &bb_dfs->pri_min_tab, &pri_min_mic_w53_tab_gen2, sizeof(bb_dfs->pri_min_tab));
				halbb_mem_cpy(bb, &bb_dfs->pri_max_tab, &pri_max_mic_w53_tab_gen2, sizeof(bb_dfs->pri_max_tab));
				halbb_mem_cpy(bb, &bb_dfs->ppb_tab, &ppb_mic_w53_tab_gen2, sizeof(bb_dfs->ppb_tab));
				halbb_mem_cpy(bb, &bb_dfs->ppb_tp_tab, &ppb_mic_w53_tp_tab_gen2, sizeof(bb_dfs->ppb_tp_tab));

			} else if (w56_band) {
				bb_dfs->is_mic_w53 = false;
				bb_dfs->is_mic_w56 = true;
				bb_dfs->l_rdr_exst_flag = true;
				halbb_mem_cpy(bb, &bb_dfs->pw_min_tab, &pw_min_mic_w56_tab_gen2, sizeof(bb_dfs->pw_min_tab));
				halbb_mem_cpy(bb, &bb_dfs->pw_max_tab, &pw_max_mic_w56_tab_gen2, sizeof(bb_dfs->pw_max_tab));
				halbb_mem_cpy(bb, &bb_dfs->pri_min_tab, &pri_min_mic_w56_tab_gen2, sizeof(bb_dfs->pri_min_tab));
				halbb_mem_cpy(bb, &bb_dfs->pri_max_tab, &pri_max_mic_w56_tab_gen2, sizeof(bb_dfs->pri_max_tab));
				halbb_mem_cpy(bb, &bb_dfs->ppb_tab, &ppb_mic_w56_tab_gen2, sizeof(bb_dfs->ppb_tab));
				halbb_mem_cpy(bb, &bb_dfs->ppb_tp_tab, &ppb_mic_w56_tp_tab_gen2, sizeof(bb_dfs->ppb_tp_tab));
			}
		}
	}
	else
#endif
	{
		if (bb_dfs->dfs_rgn_domain == DFS_REGD_JAP) {
			if (w53_band) {
				bb_dfs->is_mic_w53 = true;
				bb_dfs->is_mic_w56 = false;
				bb_dfs->l_rdr_exst_flag = false;
				halbb_mem_cpy(bb, &bb_dfs->pw_min_tab, &pw_min_mic_w53_tab, sizeof(bb_dfs->pw_min_tab));
				halbb_mem_cpy(bb, &bb_dfs->pw_max_tab, &pw_max_mic_w53_tab, sizeof(bb_dfs->pw_max_tab));
				halbb_mem_cpy(bb, &bb_dfs->pri_min_tab, &pri_min_mic_w53_tab, sizeof(bb_dfs->pri_min_tab));
				halbb_mem_cpy(bb, &bb_dfs->pri_max_tab, &pri_max_mic_w53_tab, sizeof(bb_dfs->pri_max_tab));
				halbb_mem_cpy(bb, &bb_dfs->ppb_tab, &ppb_mic_w53_tab, sizeof(bb_dfs->ppb_tab));
				halbb_mem_cpy(bb, &bb_dfs->ppb_tp_tab, &ppb_mic_w53_tp_tab, sizeof(bb_dfs->ppb_tp_tab));

			} else if (w56_band) {
				bb_dfs->is_mic_w53 = false;
				bb_dfs->is_mic_w56 = true;
				bb_dfs->l_rdr_exst_flag = true;
				halbb_mem_cpy(bb, &bb_dfs->pw_min_tab, &pw_min_mic_w56_tab, sizeof(bb_dfs->pw_min_tab));
				halbb_mem_cpy(bb, &bb_dfs->pw_max_tab, &pw_max_mic_w56_tab, sizeof(bb_dfs->pw_max_tab));
				halbb_mem_cpy(bb, &bb_dfs->pri_min_tab, &pri_min_mic_w56_tab, sizeof(bb_dfs->pri_min_tab));
				halbb_mem_cpy(bb, &bb_dfs->pri_max_tab, &pri_max_mic_w56_tab, sizeof(bb_dfs->pri_max_tab));
				halbb_mem_cpy(bb, &bb_dfs->ppb_tab, &ppb_mic_w56_tab, sizeof(bb_dfs->ppb_tab));
				halbb_mem_cpy(bb, &bb_dfs->ppb_tp_tab, &ppb_mic_w56_tp_tab, sizeof(bb_dfs->ppb_tp_tab));
			}
		}
	}
	halbb_mem_cpy(bb, &bb_dfs->ppb_typ_th, &bb_dfs->ppb_tab, sizeof(bb_dfs->ppb_typ_th));
}

void halbb_radar_chrp_mntr(struct bb_info *bb, bool chrp_flag, bool is_sg1)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	u8 i = 0;

#ifdef HALBB_DFS_GEN2_SERIES
	if (bb->ic_sub_type == BB_IC_SUB_TYPE_8852C_8852D || bb->bb_80211spec == BB_BE_IC) {
		if (bb->bb_sys_up_time - bb_dfs->chrp_srt_t >= DFS_FCC_LP_LNGTH) {
			bb_dfs->chrp_obsrv_flag = false;
			bb_dfs->chrp_srt_t = 0;
			bb_dfs->lng_rdr_cnt = 0;
			bb_dfs->sub20_0_lng_rdr_cnt = 0;
			bb_dfs->sub20_1_lng_rdr_cnt = 0;
			bb_dfs->sub20_2_lng_rdr_cnt = 0;
			bb_dfs->sub20_3_lng_rdr_cnt = 0;
			bb_dfs->sub20_4_lng_rdr_cnt = 0;
			bb_dfs->sub20_5_lng_rdr_cnt = 0;
			bb_dfs->sub20_6_lng_rdr_cnt = 0;
			bb_dfs->sub20_7_lng_rdr_cnt = 0;
		}
		if (!(bb_dfs->chrp_obsrv_flag) && (chrp_flag)) {
			bb_dfs->chrp_srt_t = bb->bb_sys_up_time;
			bb_dfs->chrp_obsrv_flag = true;
		}
	}
	else 
#endif
	{
		if (is_sg1) {// Seg1 of TW DFS
			if (bb->bb_sys_up_time - bb_dfs->chrp_srt_t_sg1 >= DFS_FCC_LP_LNGTH) {
				bb_dfs->chrp_obsrv_flag_sg1 = false;
				bb_dfs->chrp_srt_t_sg1 = 0;
				//bb_dfs->chrp_cnt = 0;
				bb_dfs->lng_rdr_cnt_sg1 = 0;
			}
			if ((chrp_flag) && !(bb_dfs->chrp_obsrv_flag_sg1)) {
				bb_dfs->chrp_srt_t_sg1 = bb->bb_sys_up_time;
				bb_dfs->chrp_obsrv_flag_sg1 = true;
			}
		} else {
			if (bb->bb_sys_up_time - bb_dfs->chrp_srt_t >= DFS_FCC_LP_LNGTH) {
				bb_dfs->chrp_obsrv_flag = false;
				bb_dfs->chrp_srt_t = 0;
				//bb_dfs->chrp_cnt = 0;
				bb_dfs->lng_rdr_cnt = 0;
			}
			if ((chrp_flag) && !(bb_dfs->chrp_obsrv_flag)) {
				bb_dfs->chrp_srt_t = bb->bb_sys_up_time;
				bb_dfs->chrp_obsrv_flag = true;
			}
		}
	}
}

u16 halbb_radar_detect(struct bb_info *bb, struct hal_dfs_rpt *dfs_rpt)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	struct bb_link_info *link = &bb->bb_link_i;
	u16 rdr_detected = false, rdr_detected_sg1 = false;
	u8 bw = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.bw;
	u8 chan = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.chan;
	u16 i = 0, pw_diff_chrp = 0, pw_diff_lng = 0;
	u16 pw_diff_chrp_sg1 = 0, pw_diff_lng_sg1 = 0, pw_max = 0, pw_max_sg1 = 0;

	bb_dfs->min_pw_lng = 65535;
	bb_dfs->min_pw_chrp = 65535;
	bb_dfs->max_pw_lng = 0;
	bb_dfs->max_pw_chrp = 0;
	bb_dfs->chrp_rdr_cnt = 0;

	bb_dfs->min_pw_lng_sg1 = 65535;
	bb_dfs->min_pw_chrp_sg1 = 65535;
	bb_dfs->max_pw_lng_sg1 = 0;
	bb_dfs->max_pw_chrp_sg1 = 0;
	bb_dfs->chrp_rdr_cnt_sg1 = 0;

	for (i = 0; i < DFS_RDR_TYP_NUM ; i++) {
		bb_dfs->min_pw_shrt[i] = 65535;
		bb_dfs->min_pri_shrt[i] = 65535;

		bb_dfs->max_pw_shrt[i] = 0;
		bb_dfs->max_pri_shrt[i] = 0;

		bb_dfs->pw_diff_shrt[i] = 0;
		bb_dfs->pri_diff_shrt[i] = 0;

		bb_dfs->min_pw_shrt_sg1[i] = 65535;
		bb_dfs->min_pri_shrt_sg1[i] = 65535;

		bb_dfs->max_pw_shrt_sg1[i] = 0;
		bb_dfs->max_pri_shrt_sg1[i] = 0;

		bb_dfs->pw_diff_shrt_sg1[i] = 0;
		bb_dfs->pri_diff_shrt_sg1[i] = 0;
	}

	/* SW Trigger Mode */
	if (bb_dfs->dfs_sw_trgr_mode && bb_dfs->dfs_sw_trgr_mode_extra) {
		rdr_detected = (RDR_FULL | RDR_EXTRA_FULL);
		BB_DBG(bb, DBG_DFS, "[HALBB] Radar SW-Trigger Mode All!\n");
		return rdr_detected;
	} else if (bb_dfs->dfs_sw_trgr_mode) {
		rdr_detected = RDR_FULL;
		BB_DBG(bb, DBG_DFS, "[HALBB] Radar SW-Trigger Mode Normal!\n");
		return rdr_detected;
	} else if (bb_dfs->dfs_sw_trgr_mode_extra) {
		rdr_detected = RDR_EXTRA_FULL;
		BB_DBG(bb, DBG_DFS, "[HALBB] Radar SW-Trigger Mode Extra Path!\n");
		return rdr_detected;
	}

	if (!(bb->support_ability & BB_DFS)) {
		BB_DBG(bb, DBG_DFS, "Not support DFS function!\n");
		return false;
	}

#ifdef HALBB_DFS_GEN2_SERIES
	if (bb->ic_sub_type == BB_IC_SUB_TYPE_8852C_8852D || bb->bb_80211spec == BB_BE_IC)
		return halbb_radar_detect_gen2(bb, dfs_rpt);
	else
#endif
	{
	/* DFS Info Parsing/Processing*/
	for (i = 0; i < (dfs_rpt->dfs_num) ; i++)
		halbb_radar_info_processing(bb, dfs_rpt, i);

	if (!(bb_dfs->l_rdr_exst_flag) || bb_dfs->rpt_rdr_cnt == 0 || bb_dfs->mask_fake_rpt_en) {
		/* Check Fake DFS rpt */
		if (bb_dfs->rpt_rdr_cnt < bb_dfs->fk_dfs_num_th) {
			BB_DFS_DBG(bb, TRIVIL_PRINT, "Non-existent form of DFS!\n");
			goto DETECTING_END;
		}
	}

	BB_DFS_DBG(bb, HWDET_PRINT, "\n");
	BB_DFS_DBG(bb, HWDET_PRINT, "[%s]===>\n", __func__);
	BB_DFS_DBG(bb, HWDET_PRINT, "DFS Region Domain = %d, BW = %d, Channel = %d\n",
	       bb_dfs->dfs_rgn_domain, bw, chan);
	BB_DFS_DBG(bb, HWDET_PRINT, "phy_idx = %d, dfs_num = %d, rpt_num = %d\n",
	       dfs_rpt->phy_idx, dfs_rpt->dfs_num, bb_dfs->rpt_rdr_cnt);
	BB_DFS_DBG(bb, HWDET_PRINT, "is_link = %d, is_CAC = %d, is_idle = %d\n",
	       link->is_linked, bb_dfs->In_CAC_Flag, bb_dfs->idle_flag);
	BB_DFS_DBG(bb, HWDET_PRINT, "pw_factor =  %d, pri_factor = %d, ppb_prcnt = %d\n",
		bb_dfs->pw_factor,bb_dfs->pri_factor,bb_dfs->ppb_prcnt);

	 /* DFS radar comparsion*/
	for (i = 0; i < (bb_dfs->rpt_rdr_cnt) ; i++) {
		halbb_radar_ptrn_cmprn(bb, i, bb_dfs->pri_rpt[i], bb_dfs->pw_rpt[i], bb_dfs->chrp_rpt[i], bb_dfs->seg_rpt[i]);
		BB_DFS_DBG(bb, HWDET_PRINT, "DFS_RPT: [pw, pri, c_flag, is_sg1] = [%d, %d, %d, %d]\n",
			bb_dfs->pw_rpt[i], bb_dfs->pri_rpt[i], bb_dfs->chrp_rpt[i], bb_dfs->seg_rpt[i]);
	}

	/* Monitor Time and valid radar counter show*/
	if (bb_dfs->dbg_print_component & HWDET_PRINT) {
		if (bb_dfs->l_rdr_exst_flag) {
			if (bb_dfs->is_tw_en) {
				BB_DBG(bb, DBG_DFS, "[mntr_prd, sys_t, chrp_srt_t]: [%d, %d, %d]\n",
	     	 			(bb->bb_sys_up_time - bb_dfs->chrp_srt_t),
	       			bb->bb_sys_up_time, bb_dfs->chrp_srt_t);
				BB_DBG(bb, DBG_DFS, "[mntr_prd, sys_t, chrp_srt_t_sg1]: [%d, %d, %d]\n",
	     	 			(bb->bb_sys_up_time - bb_dfs->chrp_srt_t_sg1),
	       			bb->bb_sys_up_time, bb_dfs->chrp_srt_t_sg1);
			}
			else
				BB_DBG(bb, DBG_DFS, "[mntr_prd, sys_t, chrp_srt_t]: [%d, %d, %d]\n",
	     	 			(bb->bb_sys_up_time - bb_dfs->chrp_srt_t),
	       			bb->bb_sys_up_time, bb_dfs->chrp_srt_t);
		}
		if (bb_dfs->is_tw_en) {
			BB_DBG(bb, DBG_DFS, "==================seg 0==================\n");
			BB_DBG(bb, DBG_DFS, "lng_rdr_cnt = %d\n", bb_dfs->lng_rdr_cnt);
			BB_DBG(bb, DBG_DFS, "srt_rdr_cnt = [%d, %d, %d, %d, %d, %d, %d, %d]\n",
				bb_dfs->srt_rdr_cnt[0], bb_dfs->srt_rdr_cnt[1],
				bb_dfs->srt_rdr_cnt[2], bb_dfs->srt_rdr_cnt[3],
				bb_dfs->srt_rdr_cnt[4], bb_dfs->srt_rdr_cnt[5],
				bb_dfs->srt_rdr_cnt[6], bb_dfs->srt_rdr_cnt[7]);
			BB_DBG(bb, DBG_DFS, "==================seg 1==================\n");
			BB_DBG(bb, DBG_DFS, "lng_rdr_cnt_sg1 = %d\n", bb_dfs->lng_rdr_cnt_sg1);
			BB_DBG(bb, DBG_DFS, "srt_rdr_cnt_sg1 = [%d, %d, %d, %d, %d, %d, %d, %d]\n",
				bb_dfs->srt_rdr_cnt_sg1[0], bb_dfs->srt_rdr_cnt_sg1[1],
				bb_dfs->srt_rdr_cnt_sg1[2], bb_dfs->srt_rdr_cnt_sg1[3],
				bb_dfs->srt_rdr_cnt_sg1[4], bb_dfs->srt_rdr_cnt_sg1[5],
				bb_dfs->srt_rdr_cnt_sg1[6], bb_dfs->srt_rdr_cnt_sg1[7]);
			BB_DBG(bb, DBG_DFS, "\n");
		}
		else {
			BB_DBG(bb, DBG_DFS, "\n");
			BB_DBG(bb, DBG_DFS, "lng_rdr_cnt = %d\n", bb_dfs->lng_rdr_cnt);
			BB_DBG(bb, DBG_DFS, "srt_rdr_cnt = [%d, %d, %d, %d, %d, %d, %d, %d]\n",
				bb_dfs->srt_rdr_cnt[0], bb_dfs->srt_rdr_cnt[1],
				bb_dfs->srt_rdr_cnt[2], bb_dfs->srt_rdr_cnt[3],
				bb_dfs->srt_rdr_cnt[4], bb_dfs->srt_rdr_cnt[5],
				bb_dfs->srt_rdr_cnt[6], bb_dfs->srt_rdr_cnt[7]);
			BB_DBG(bb, DBG_DFS, "\n");
		}
	}

	/* PW Diff calculation */
	for (i = 0; i < DFS_RDR_TYP_NUM ; i++) {
		bb_dfs->pw_diff_shrt[i] = bb_dfs->max_pw_shrt[i] - bb_dfs->min_pw_shrt[i];
		bb_dfs->pri_diff_shrt[i] = bb_dfs->max_pri_shrt[i] - bb_dfs->min_pri_shrt[i];
		BB_DFS_DBG(bb, HWDET_PRINT, "short type %d : [pw_diff,pri_diff] = [%d,%d] \n", (i + 1), bb_dfs->pw_diff_shrt[i], bb_dfs->pri_diff_shrt[i]);
		if (bb_dfs->is_tw_en) {
			bb_dfs->pw_diff_shrt_sg1[i] = bb_dfs->max_pw_shrt_sg1[i] - bb_dfs->min_pw_shrt_sg1[i];
			bb_dfs->pri_diff_shrt_sg1[i] = bb_dfs->max_pri_shrt_sg1[i] - bb_dfs->min_pri_shrt_sg1[i];
			BB_DFS_DBG(bb, HWDET_PRINT, "short type %d : [pw_diff_sg1,pri_diff_sg1] = [%d,%d] \n", (i + 1), bb_dfs->pw_diff_shrt_sg1[i], bb_dfs->pri_diff_shrt_sg1[i]);
		}
	}
	pw_diff_chrp = bb_dfs->max_pw_chrp - bb_dfs->min_pw_chrp;
	BB_DFS_DBG(bb, HWDET_PRINT, "chrp pulse : [pw_diff] = [%d] \n",pw_diff_chrp);
	if (bb_dfs->is_tw_en) {
		pw_diff_chrp_sg1 = bb_dfs->max_pw_chrp_sg1 - bb_dfs->min_pw_chrp_sg1;
		BB_DFS_DBG(bb, HWDET_PRINT, "chrp pulse : [pw_diff_sg1] = [%d] \n",pw_diff_chrp_sg1);
	}

	/*MAX PW*/
	for (i = 0; i < dfs_rpt->dfs_num ; i++) {
		if (bb_dfs->is_tw_en) {
			if (bb_dfs->seg_rpt[i])
				pw_max_sg1 = MAX_2(pw_max_sg1, bb_dfs->pw_rpt[i]);
			else
				pw_max = MAX_2(pw_max, bb_dfs->pw_rpt[i]);
		}
		else {
			pw_max = MAX_2(pw_max, bb_dfs->pw_rpt[i]);
		}
	}
	BB_DFS_DBG(bb, HWDET_PRINT, "[pw_max,pw_max_sg1,pw_max_th] = [%d,%d,%d] \n", pw_max, pw_max_sg1, bb_dfs->pw_max_th);

	/*lng pulse cnt reset*/
	if (bb_dfs->l_rdr_exst_flag) {
		pw_diff_lng = bb_dfs->max_pw_lng - bb_dfs->min_pw_lng;
		BB_DFS_DBG(bb, HWDET_PRINT, "long pulse : [pw_diff] = [%d] \n",pw_diff_lng);
		if (pw_diff_lng > bb_dfs->pw_lng_chrp_diff_th) {
			bb_dfs->lng_rdr_cnt = 0;          // reset lng_rdr cnt
			BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "long type PW Diff BRK, reset lng_rdr cnt!\n");
		}
		if (pw_diff_chrp > bb_dfs->pw_lng_chrp_diff_th) {
			bb_dfs->lng_rdr_cnt = 0;          // reset lng_rdr cnt
			BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "chrp type PW Diff BRK, reset lng_rdr cnt!\n");
		}
		if (pw_max > bb_dfs->pw_max_th) {
			bb_dfs->lng_rdr_cnt = 0;          // reset lng_rdr cnt
			BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "pw_max BRK, reset lng_rdr cnt!\n");
		}
		if (bb_dfs->chrp_rdr_cnt > bb_dfs->invalid_lng_pulse_h_th) {   // max PPB=3 in lng rdr (plus one as margin)
			bb_dfs->lng_rdr_cnt = 0;          // reset lng_rdr cnt
			BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "Invalid long pulse cnt BRK, reset lng_rdr cnt!\n");
		}
		if (bb_dfs->lng_rdr_cnt - bb_dfs->lng_rdr_cnt_pre <= bb_dfs->invalid_lng_pulse_l_th) // it means this time increment is not reliable
			bb_dfs->lng_rdr_cnt = bb_dfs->lng_rdr_cnt_pre;
		if (((u8)dfs_rpt->dfs_num - bb_dfs->rpt_rdr_cnt) > bb_dfs->fake_rdr_cnt_th) {
			bb_dfs->lng_rdr_cnt = 0;          // reset lng_rdr cnt
			BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "Reset lng_rdr cnt due to many fake rdr !\n");
		}
		if (bb_dfs->is_tw_en) {
			pw_diff_lng_sg1 = bb_dfs->max_pw_lng_sg1 - bb_dfs->min_pw_lng_sg1;
			BB_DFS_DBG(bb, HWDET_PRINT, "long pulse : [pw_diff_sg1] = [%d] \n",pw_diff_lng_sg1);
			if (pw_diff_lng_sg1 > bb_dfs->pw_lng_chrp_diff_th) {
				bb_dfs->lng_rdr_cnt_sg1 = 0;          // reset lng_rdr cnt
				BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "long type PW Diff BRK, reset lng_rdr cnt_sg1!\n");
			}
			if (pw_diff_chrp_sg1 > bb_dfs->pw_lng_chrp_diff_th) {
				bb_dfs->lng_rdr_cnt_sg1 = 0;          // reset lng_rdr cnt
				BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "chrp type PW Diff BRK, reset lng_rdr cnt_sg1!\n");
			}
			if (pw_max_sg1 > bb_dfs->pw_max_th) {
				bb_dfs->lng_rdr_cnt_sg1 = 0;          // reset lng_rdr cnt
				BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "pw_max_sg1 BRK, reset lng_rdr cnt_sg1!\n");
			}
			if (bb_dfs->chrp_rdr_cnt_sg1 > bb_dfs->invalid_lng_pulse_h_th) {   // max PPB=3 in lng rdr (plus one as margin)
				bb_dfs->lng_rdr_cnt_sg1 = 0;          // reset lng_rdr cnt
				BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "Invalid long pulse cnt BRK, reset lng_rdr cnt_sg1!\n");
			}
			if (bb_dfs->lng_rdr_cnt_sg1 - bb_dfs->lng_rdr_cnt_sg1_pre <= bb_dfs->invalid_lng_pulse_l_th) // it means this time increment is not reliable
				bb_dfs->lng_rdr_cnt_sg1 = bb_dfs->lng_rdr_cnt_sg1_pre;
			if (((u8)dfs_rpt->dfs_num - bb_dfs->rpt_rdr_cnt) > bb_dfs->fake_rdr_cnt_th) {
				bb_dfs->lng_rdr_cnt_sg1 = 0;          // reset lng_rdr cnt
				BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "Reset lng_rdr cnt_sg1 due to many fake rdr !\n");
			}
		}
	}

	BB_DFS_DBG(bb, HWDET_PRINT, "[pw_diff_th,pw_lng_chrp_diff_th,pri_diff_th] = [%d,%d,%d] \n",
		bb_dfs->pw_diff_th,bb_dfs->pw_lng_chrp_diff_th,bb_dfs->pri_diff_th);
	BB_DFS_DBG(bb, HWDET_PRINT, "[adap_cnt,adap_cnt_all,adap_cnt_th] =[%d,%d,%d]\n",
		bb_dfs->adap_detect_cnt, bb_dfs->adap_detect_cnt_all, bb_dfs->adap_detect_cnt_th);

	/* Check if DFS matching cnts exceed ppb th*/
	for (i = 0; i < DFS_RDR_TYP_NUM ; i++) {
		if ((i == DFS_L_RDR_IDX) && (bb_dfs->l_rdr_exst_flag)) {
			if ((bb_dfs->lng_rdr_cnt >= bb_dfs->ppb_typ_th[i])) {
				if ((bw == CHANNEL_WIDTH_160) && (chan >= 36 && chan <= 64)) {
					rdr_detected = false;
					BB_DFS_DBG(bb, HWDET_PRINT, "Long Rdr not in Band1 !\n");
				}
				else {
					rdr_detected = true;
					BB_DFS_DBG(bb, HWDET_PRINT, "Long Rdr Appeared!\n");
					BB_DFS_DBG(bb, HWDET_PRINT, "Long Rdr reaches threshold (ppb_th:%d)!\n",bb_dfs->ppb_typ_th[i]);
				}
			}
		} else {
			if (bb_dfs->srt_rdr_cnt[i] >= bb_dfs->ppb_typ_th[i]) {
				 rdr_detected = true;
				BB_DFS_DBG(bb, HWDET_PRINT, "Rdr Type %d reaches threshold (ppb_th:%d)!\n", (i+1), bb_dfs->ppb_typ_th[i]);
				 if (bb_dfs->pw_diff_shrt[i] > bb_dfs->pw_diff_th) {
				 	rdr_detected = false;
					BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "Short type %d PW Diff BRK, pw_diff_shrt = %d\n",(i+1),bb_dfs->pw_diff_shrt[i]);
				}
				if (bb_dfs->pri_diff_shrt[i] > bb_dfs->pri_diff_th) {
					rdr_detected = false;
					BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "Short type %d PRI Diff BRK, pri_diff_shrt = %d\n",(i+1),bb_dfs->pri_diff_shrt[i]);
				}
			}
		}
	}

	/*Drop the detected RDR to prevent FRD*/
	if (rdr_detected) {
		if (((u8)dfs_rpt->dfs_num - bb_dfs->rpt_rdr_cnt) > bb_dfs->fake_rdr_cnt_th) {
			rdr_detected = false;
			BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "RDR drop due to many fake rdr !\n");
		}
		if (pw_max > bb_dfs->pw_max_th) {
			rdr_detected = false;
			BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "RDR drop by PW_MAX BRK !\n");
		}
		if (pw_diff_chrp > bb_dfs->pw_lng_chrp_diff_th) {
			rdr_detected = false;
			BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "RDR drop by Chrp PW Diff BRK !\n");
		}
		if ((bb_dfs->adap_detect_cnt_all > bb_dfs->adap_detect_cnt_th) && (bb_dfs->adap_detect_brk_en)) {
			rdr_detected = false;
			BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "RDR drop by ENV_2 !\n");
		} else if (bb_dfs->detect_state) {
			rdr_detected = false;
			BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "RDR drop by ENV !\n");
		}
		if ((bb_dfs->n_seq_flag) && (bw < CHANNEL_WIDTH_160)) {
			rdr_detected = false;
			BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "Non-sequential DFS Dropped!\n");
			for (i = 0; i < (dfs_rpt->dfs_num) ; i++) {
				BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "[seq_num, is_sg1] = [%d, %d]\n",
					 bb_dfs->seq_num_rpt_all[i], bb_dfs->seg_rpt_all[i]);
			}
		}
	}

	/* Check if DFS matching cnts exceed ppb th for TW DFS*/
	if (bb_dfs->is_tw_en) {
		for (i = 0; i < DFS_RDR_TYP_NUM ; i++) {
			if ((i == DFS_L_RDR_IDX) && (bb_dfs->l_rdr_exst_flag))  {
				if ((bb_dfs->lng_rdr_cnt_sg1 >= bb_dfs->ppb_typ_th[i])) {
					rdr_detected_sg1 = true;
					BB_DFS_DBG(bb, HWDET_PRINT, "seg1: Long Rdr Appeared!\n");
					BB_DFS_DBG(bb, HWDET_PRINT, "seg1: Long Rdr reaches threshold (ppb_th:%d)!\n",bb_dfs->ppb_typ_th[i]);
				}
			} else {
				if (bb_dfs->srt_rdr_cnt_sg1[i] >= bb_dfs->ppb_typ_th[i]) {
					 rdr_detected_sg1 = true;
					BB_DFS_DBG(bb, HWDET_PRINT, "seg1: Rdr Type %d reaches threshold (ppb_th:%d)!\n",
						(i+1), bb_dfs->ppb_typ_th[i]);
					 if (bb_dfs->pw_diff_shrt_sg1[i] > bb_dfs->pw_diff_th) {
					 	rdr_detected_sg1 = false;
						BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "seg1: Short type %d PW Diff BRK, pw_diff_shrt_sg1 = %d\n",(i+1),bb_dfs->pw_diff_shrt_sg1[i]);
					}
					if (bb_dfs->pri_diff_shrt_sg1[i] > bb_dfs->pri_diff_th) {
						rdr_detected_sg1 = false;
						BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "seg1: Short type %d PRI Diff BRK, pri_diff_shrt_sg1 = %d\n",(i+1),bb_dfs->pri_diff_shrt_sg1[i]);
					}
				}
			}
		}
	}

	/*Drop the detected RDR to prevent FRD for TW DFS*/
	if (bb_dfs->is_tw_en) {
		if (rdr_detected_sg1) {
			if (((u8)dfs_rpt->dfs_num - bb_dfs->rpt_rdr_cnt) > bb_dfs->fake_rdr_cnt_th) {
				rdr_detected_sg1 = false;
				BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "seg1: RDR drop due to many fake rdr !\n");
			}
			if (pw_max_sg1 > bb_dfs->pw_max_th) {
				rdr_detected_sg1 = false;
				BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "seg1: RDR drop by PW_MAX BRK !\n");
			}
			if (pw_diff_chrp_sg1 > bb_dfs->pw_lng_chrp_diff_th) {
				rdr_detected_sg1 = false;
				BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "seg1: RDR drop by Chrp PW Diff BRK !\n");
			}
			if ((bb_dfs->adap_detect_cnt_all > bb_dfs->adap_detect_cnt_th) && (bb_dfs->adap_detect_brk_en)) {
				rdr_detected_sg1 = false;
				BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "seg1: RDR drop by ENV_2 !\n");
			} else if (bb_dfs->detect_state) {
				rdr_detected_sg1 = false;
				BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "seg1: RDR drop by ENV !\n");
			}
			if ((bb_dfs->n_seq_flag_sg1) && (bw < CHANNEL_WIDTH_160)) {
				rdr_detected_sg1 = false;
				BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "seg1: Non-sequential DFS Dropped!\n");
				for (i = 0; i < (dfs_rpt->dfs_num) ; i++) {
					BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "[seq_num, is_sg1] = [%d, %d]\n",
						 bb_dfs->seq_num_rpt_all[i], bb_dfs->seg_rpt_all[i]);
				}
			}
		}
	}

	if (bb_dfs->dbg_print_component & HWDET_PRINT)
		bb_dfs->dbg_print_component &= ~SWDET_PRINT;

	/* Debug Mode */
	if (rdr_detected) {
		if (bb_dfs->dbg_print_component & SWDET_PRINT) {
			BB_DBG(bb, DBG_DFS, "\n");
			BB_DBG(bb, DBG_DFS, "[%s]===>\n", __func__);
			BB_DBG(bb, DBG_DFS, "DFS Region Domain = %d, BW = %d, Channel = %d\n",
		      		bb_dfs->dfs_rgn_domain, bw, chan);
			BB_DBG(bb, DBG_DFS, "phy_idx = %d, dfs_num = %d, rpt_num = %d\n",
		       		dfs_rpt->phy_idx, dfs_rpt->dfs_num, bb_dfs->rpt_rdr_cnt);
			BB_DBG(bb, DBG_DFS, "is_link = %d, is_CAC = %d, is_idle = %d\n",
		      	 	link->is_linked, bb_dfs->In_CAC_Flag, bb_dfs->idle_flag);
			BB_DBG(bb, DBG_DFS, "pw_factor =  %d, pri_factor = %d, ppb_prcnt = %d\n",
				bb_dfs->pw_factor,bb_dfs->pri_factor,bb_dfs->ppb_prcnt);
			for (i = 0; i < DFS_RDR_TYP_NUM ; i++) {
				BB_DBG(bb, DBG_DFS, "Type %d: [pw_lbd-pw_ubd], [pri_lbd-pri_ubd], [ppb_thd] = [%d-%d], [%d-%d], [%d]\n",
				      (i+1), bb_dfs->pw_lbd[i],
				      bb_dfs->pw_ubd[i], bb_dfs->pri_lbd[i],
				      bb_dfs->pri_ubd[i], bb_dfs->ppb_typ_th[i]);
			}
			for (i = 0; i < bb_dfs->rpt_rdr_cnt ; i++) {
				BB_DBG(bb, DBG_DFS, "DFS_RPT %d: [pw, pri, c_flag, is_sg1] = [%d, %d, %d, %d]\n",
			              (i + 1), bb_dfs->pw_rpt[i], bb_dfs->pri_rpt[i],
				       bb_dfs->chrp_rpt[i], bb_dfs->seg_rpt[i]);
			}
			BB_DBG(bb, DBG_DFS, "lng_rdr_cnt = %d\n", bb_dfs->lng_rdr_cnt);
			BB_DBG(bb, DBG_DFS, "srt_rdr_cnt = [%d, %d, %d, %d, %d, %d, %d, %d]\n",
			       bb_dfs->srt_rdr_cnt[0], bb_dfs->srt_rdr_cnt[1],
			       bb_dfs->srt_rdr_cnt[2], bb_dfs->srt_rdr_cnt[3],
			       bb_dfs->srt_rdr_cnt[4], bb_dfs->srt_rdr_cnt[5],
			       bb_dfs->srt_rdr_cnt[6], bb_dfs->srt_rdr_cnt[7]);
			BB_DBG(bb, DBG_DFS, "pw_diff_th = %d, pw_lng_chrp_diff_th=%d, pri_diff_th =%d, pw_max_th =%d\n",
				bb_dfs->pw_diff_th, bb_dfs->pw_lng_chrp_diff_th, bb_dfs->pri_diff_th, bb_dfs->pw_max_th);
			BB_DBG(bb, DBG_DFS, "adap_cnt = %d, adap_cnt_all = %d\n",
				bb_dfs->adap_detect_cnt, bb_dfs->adap_detect_cnt_all);
		}
		/* Reset Long radar Counter */
		bb_dfs->lng_rdr_cnt = 0;
		if (bb_dfs->dfs_dbg_mode) {
			rdr_detected = false;
			BB_DBG(bb, DBG_DFS, "Radar is detected in DFS debug mode!\n");
		}
		else
			BB_DBG(bb, DBG_DFS, "Radar is detected in DFS general mode!\n");
	}

	/* Debug Mode for TW DFS*/
	if (bb_dfs->is_tw_en) {
		if (rdr_detected_sg1) {
			if (bb_dfs->dbg_print_component & SWDET_PRINT) {
				BB_DBG(bb, DBG_DFS, "\n");
				BB_DBG(bb, DBG_DFS, "[%s]===>\n", __func__);
				BB_DBG(bb, DBG_DFS, "DFS Region Domain = %d, BW = %d, Channel = %d\n",
			      		bb_dfs->dfs_rgn_domain, bw, chan);
				BB_DBG(bb, DBG_DFS, "phy_idx = %d, dfs_num = %d, rpt_num = %d\n",
			       		dfs_rpt->phy_idx, dfs_rpt->dfs_num, bb_dfs->rpt_rdr_cnt);
				BB_DBG(bb, DBG_DFS, "is_link = %d, is_CAC = %d, is_idle = %d\n",
			      	 	link->is_linked, bb_dfs->In_CAC_Flag, bb_dfs->idle_flag);
				BB_DBG(bb, DBG_DFS, "pw_factor =  %d, pri_factor = %d, ppb_prcnt = %d\n",
					bb_dfs->pw_factor,bb_dfs->pri_factor,bb_dfs->ppb_prcnt);
				for (i = 0; i < DFS_RDR_TYP_NUM; i++) {
					BB_DBG(bb, DBG_DFS, "Type %d: [pw_lbd-pw_ubd], [pri_lbd-pri_ubd], [ppb_thd] = [%d-%d], [%d-%d], [%d]\n",
				      (i+1), bb_dfs->pw_lbd[i],
				      bb_dfs->pw_ubd[i], bb_dfs->pri_lbd[i],
				      bb_dfs->pri_ubd[i], bb_dfs->ppb_typ_th[i]);
				}
				for (i = 0; i < bb_dfs->rpt_rdr_cnt; i++) {
					BB_DBG(bb, DBG_DFS, "DFS_RPT %d: [pw, pri, c_flag, is_sg1] = [%d, %d, %d, %d]\n",
						(i + 1), bb_dfs->pw_rpt[i], bb_dfs->pri_rpt[i],
						bb_dfs->chrp_rpt[i], bb_dfs->seg_rpt[i]);
				}
				BB_DBG(bb, DBG_DFS, "lng_rdr_cnt_sg1 = %d\n", bb_dfs->lng_rdr_cnt_sg1);
				BB_DBG(bb, DBG_DFS, "srt_rdr_cnt_sg1 = [%d, %d, %d, %d, %d, %d, %d, %d]\n",
					bb_dfs->srt_rdr_cnt_sg1[0], bb_dfs->srt_rdr_cnt_sg1[1],
					bb_dfs->srt_rdr_cnt_sg1[2], bb_dfs->srt_rdr_cnt_sg1[3],
					bb_dfs->srt_rdr_cnt_sg1[4], bb_dfs->srt_rdr_cnt_sg1[5],
					bb_dfs->srt_rdr_cnt_sg1[6], bb_dfs->srt_rdr_cnt_sg1[7]);
				BB_DBG(bb, DBG_DFS, "pw_diff_th = %d, pw_lng_chrp_diff_th=%d, pri_diff_th =%d, pw_max_th =%d\n",
					bb_dfs->pw_diff_th, bb_dfs->pw_lng_chrp_diff_th, bb_dfs->pri_diff_th, bb_dfs->pw_max_th);
				BB_DBG(bb, DBG_DFS, "adap_cnt = %d, adap_cnt_all = %d\n",
					bb_dfs->adap_detect_cnt, bb_dfs->adap_detect_cnt_all);
			}
			/* Reset Long radar Counter */
			bb_dfs->lng_rdr_cnt_sg1 = 0;
			if (bb_dfs->dfs_dbg_mode) {
				rdr_detected_sg1 = false;
				BB_DBG(bb, DBG_DFS, "seg1:Radar is detected in DFS debug mode!\n");
			}
			else
				BB_DBG(bb, DBG_DFS, "seg1:Radar is detected in DFS general mode!\n");
		}
	}

DETECTING_END:
	/* Reset SW Counter/Flag */
	bb_dfs->n_seq_flag = false;
	bb_dfs->n_seq_flag_sg1 = false;
	bb_dfs->rpt_rdr_cnt = 0;
	for (i = 0; i < DFS_RDR_TYP_NUM; i++) {
		bb_dfs->srt_rdr_cnt[i] = 0;
		bb_dfs->srt_rdr_cnt_sg1[i] = 0;
	}
	for (i = 0; i < dfs_rpt->dfs_num ; i++) {
		bb_dfs->pw_rpt[i] = 0;
		bb_dfs->pri_rpt[i] = 0;
		bb_dfs->chrp_rpt[i] = 0;
		bb_dfs->seg_rpt[i] = 0;
		bb_dfs->seg_rpt_all[i] = 0;
		bb_dfs->seq_num_rpt[i] = 0;
		bb_dfs->seq_num_rpt_all[i] = 0;
	}
	//reset rpt sg memory
	bb_dfs->rpt_sg_history = 0;
	bb_dfs->rpt_sg_history_all = 0;

	//store lng_rdr_cnt for invalid check
	bb_dfs->lng_rdr_cnt_pre = bb_dfs->lng_rdr_cnt;
	bb_dfs->lng_rdr_cnt_sg1_pre = bb_dfs->lng_rdr_cnt_sg1;

	if (rdr_detected || rdr_detected_sg1)
		return RDR_FULL;
	else
		return false;
	}
}

#ifdef HALBB_DFS_GEN2_SERIES

void halbb_radar_info_processing_gen2(struct bb_info *bb,
				 struct hal_dfs_rpt *rpt, u16 dfs_rpt_idx)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	struct bb_rdr_info_gen2 *dfs_rdr_info = (struct bb_rdr_info_gen2 *)rpt->dfs_ptr;
	struct bb_dfs_cr_info *cr = &bb->bb_cmn_hooker->bb_dfs_cr_i;

	bool dfs_more = 0, dfs_phy_def = 0, sub20_1_chirp_flag = 0, sub20_1_pres = 0;
	bool sub20_0_chirp_flag = 0, sub20_0_pres = 0;
	u8 format = 0, bbid = 0;
	u16 seq_num = 0, sub20_1_pw = 0, sub20_1_pri = 0, sub20_0_pw = 0, sub20_0_pri = 0;
	u16 RSVD = 0;

	dfs_more = dfs_rdr_info->dfs_more;
	dfs_phy_def = dfs_rdr_info->dfs_phy_def;
	format = dfs_rdr_info->dfs_format;
	bbid = dfs_rdr_info->dfs_bbid;
	seq_num = (dfs_rdr_info->dfs_seq_num_m << 4) | (dfs_rdr_info->dfs_seq_num_l);
	RSVD = (dfs_rdr_info->RSVD_m << 8) | (dfs_rdr_info->RSVD_l);
	sub20_1_pres = dfs_rdr_info->dfs_sub20_1_pres;
	if (sub20_1_pres) { // parsing the report if report is present
		sub20_1_pw = (dfs_rdr_info->dfs_sub20_1_pw_m << 1) | (dfs_rdr_info->dfs_sub20_1_pw_l);
		sub20_1_pri = (dfs_rdr_info->dfs_sub20_1_pri_m << 2) | (dfs_rdr_info->dfs_sub20_1_pri_l);
		sub20_1_chirp_flag = dfs_rdr_info->dfs_sub20_1_chirp_flag;
	}
	sub20_0_pres = dfs_rdr_info->dfs_sub20_0_pres;
	if (sub20_0_pres) {
		sub20_0_pw = (dfs_rdr_info->dfs_sub20_0_pw_m << 5) | (dfs_rdr_info->dfs_sub20_0_pw_l);
		sub20_0_pri = (dfs_rdr_info->dfs_sub20_0_pri_m << 6) | (dfs_rdr_info->dfs_sub20_0_pri_l);
		sub20_0_chirp_flag = dfs_rdr_info->dfs_sub20_0_chirp_flag;
	}

	rpt->dfs_ptr += DFS_RPT_LENGTH;

	/* mask pri < th dfs report because it may generate lots of rpts*/
	if ((sub20_0_pri >= bb_dfs->pri_mask_th) || (sub20_1_pri >= bb_dfs->pri_mask_th)) {
		bb_dfs->dfs_more[bb_dfs->rpt_rdr_cnt] = dfs_more;
		bb_dfs->dfs_phy_def[bb_dfs->rpt_rdr_cnt] = dfs_phy_def;
		bb_dfs->format[bb_dfs->rpt_rdr_cnt] = format;
		bb_dfs->bbid[bb_dfs->rpt_rdr_cnt] = bbid;
		bb_dfs->seq_num[bb_dfs->rpt_rdr_cnt] = seq_num;
		bb_dfs->RSVD[bb_dfs->rpt_rdr_cnt] = RSVD;
		bb_dfs->sub20_1_pw[bb_dfs->rpt_rdr_cnt] = sub20_1_pw;
		bb_dfs->sub20_1_pri[bb_dfs->rpt_rdr_cnt] = sub20_1_pri;
		bb_dfs->sub20_1_chirp_flag[bb_dfs->rpt_rdr_cnt] = sub20_1_chirp_flag;
		bb_dfs->sub20_1_pres[bb_dfs->rpt_rdr_cnt] = sub20_1_pres;
		bb_dfs->sub20_0_pw[bb_dfs->rpt_rdr_cnt] = sub20_0_pw;
		bb_dfs->sub20_0_pri[bb_dfs->rpt_rdr_cnt] = sub20_0_pri;
		bb_dfs->sub20_0_chirp_flag[bb_dfs->rpt_rdr_cnt] = sub20_0_chirp_flag;
		bb_dfs->sub20_0_pres[bb_dfs->rpt_rdr_cnt] = sub20_0_pres;
		bb_dfs->rpt_rdr_cnt ++;
	}

	if ((sub20_0_pri == 511) ||(sub20_1_pri == 511))
		bb_dfs->pri_full_cnt ++;
}

void halbb_radar_cnt_accumulate(struct bb_info *bb, u8 rpt_idx, bool sub20_1, u8 rdr_type_idx, bool is_long)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;

	if (is_long) {
		if (!sub20_1) {
			switch (bb_dfs->format[rpt_idx]) {
				case 0:
					bb_dfs->sub20_0_lng_rdr_cnt ++;
					break;
				case 1:
					bb_dfs->sub20_2_lng_rdr_cnt ++;
					break;
				case 2:
					bb_dfs->sub20_4_lng_rdr_cnt ++;
					break;
				case 3:
					bb_dfs->sub20_6_lng_rdr_cnt ++;
					break;
				default:
					break;
			}
		}
		else {
			switch (bb_dfs->format[rpt_idx]) {
				case 0:
					bb_dfs->sub20_1_lng_rdr_cnt ++;
					break;
				case 1:
					bb_dfs->sub20_3_lng_rdr_cnt ++;
					break;
				case 2:
					bb_dfs->sub20_5_lng_rdr_cnt ++;
					break;
				case 3:
					bb_dfs->sub20_7_lng_rdr_cnt ++;
					break;
				default:
					break;
			}
		}
	}
	else {
		if (!sub20_1) {
			switch (bb_dfs->format[rpt_idx]) {
				case 0:
					bb_dfs->sub20_0_srt_rdr_cnt[rdr_type_idx] ++;
					break;
				case 1:
					bb_dfs->sub20_2_srt_rdr_cnt[rdr_type_idx] ++;
					break;
				case 2:
					bb_dfs->sub20_4_srt_rdr_cnt[rdr_type_idx] ++;
					break;
				case 3:
					bb_dfs->sub20_6_srt_rdr_cnt[rdr_type_idx] ++;
					break;
				default:
					break;
			}
		}
		else {
			switch (bb_dfs->format[rpt_idx]) {
				case 0:
					bb_dfs->sub20_1_srt_rdr_cnt[rdr_type_idx] ++;
					break;
				case 1:
					bb_dfs->sub20_3_srt_rdr_cnt[rdr_type_idx] ++;
					break;
				case 2:
					bb_dfs->sub20_5_srt_rdr_cnt[rdr_type_idx] ++;
					break;
				case 3:
					bb_dfs->sub20_7_srt_rdr_cnt[rdr_type_idx] ++;
					break;
				default:
					break;
			}
		}
	}
}

void halbb_radar_ptrn_cmprn_gen2(struct bb_info *bb)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	u16 pw_lb[DFS_RDR_TYP_NUM] = {0}, pw_ub[DFS_RDR_TYP_NUM] = {0};
	u16 pri_lb[DFS_RDR_TYP_NUM] = {0}, pri_ub[DFS_RDR_TYP_NUM] = {0};
	u8 ppb_th[DFS_RDR_TYP_NUM] = {0}, i = 0, k = 0, m = 0;
	u8 adv_pri_cnt_temp = 0, adv_pri_cnt_temp0 = 0, adv_pri_cnt_temp1 = 0;

	if ((bb_dfs->dfs_rgn_domain == DFS_REGD_FCC || bb_dfs->is_mic_w56))
		halbb_radar_chrp_mntr(bb, false, false); // reset lng cnt when monitor time > lng pulse length

	for (k = 0; k < DFS_RDR_TYP_NUM ; k++) {
		pw_lb[k] = SUBTRACT_TO_0(bb_dfs->pw_min_tab[k], bb_dfs->pw_ofst);
			if (pw_lb[k] == 0)
				pw_lb[k]++;
			pw_ub[k] = bb_dfs->pw_max_tab[k] + bb_dfs->pw_ofst;
			pri_lb[k] = bb_dfs->pri_min_tab[k] - bb_dfs->pri_ofst;
			pri_ub[k] = bb_dfs->pri_max_tab[k] + bb_dfs->pri_ofst;

		// long radar determine
		if ((k == DFS_L_RDR_IDX) && (bb_dfs->dfs_rgn_domain == DFS_REGD_FCC || bb_dfs->is_mic_w56)) {
			for (i = 0; i < (bb_dfs->rpt_rdr_cnt) ; i++) {
				if (((bb_dfs->sub20_0_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_0_pw[i] <= pw_ub[k])) ||
				     ((bb_dfs->sub20_1_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_1_pw[i] <= pw_ub[k]))) {
					if (bb_dfs->rpt_rdr_cnt >= 8) {  // prevent lng pulse FRD
						BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "Not count long pulse due to much report to prevent FRD !\n");
						break;
					}
					halbb_radar_chrp_mntr(bb, true, false);   // monitor 12s when detect first lng pulse
				     }
				if (i == 0) {
					if (((bb_dfs->sub20_0_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_0_pw[i] <= pw_ub[k])))
						halbb_radar_cnt_accumulate(bb, i, false, k, true);
					if (((bb_dfs->sub20_1_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_1_pw[i] <= pw_ub[k])))
						halbb_radar_cnt_accumulate(bb, i, true, k, true);
				}
				else {
					if ((bb_dfs->sub20_0_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_0_pw[i] <= pw_ub[k]) &&
			  		     (bb_dfs->sub20_0_pri[i] >= pri_lb[k]) && (bb_dfs->sub20_0_pri[i] <= pri_ub[k]))
						halbb_radar_cnt_accumulate(bb, i, false, k, true);
					if ((bb_dfs->sub20_1_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_1_pw[i] <= pw_ub[k]) &&
			    		     (bb_dfs->sub20_1_pri[i] >= pri_lb[k]) && (bb_dfs->sub20_1_pri[i] <= pri_ub[k]))
						halbb_radar_cnt_accumulate(bb, i, true, k, true);
				}
			}
			bb_dfs->lng_rdr_cnt = bb_dfs->sub20_0_lng_rdr_cnt + bb_dfs->sub20_1_lng_rdr_cnt +
			bb_dfs->sub20_2_lng_rdr_cnt + bb_dfs->sub20_3_lng_rdr_cnt +
			bb_dfs->sub20_4_lng_rdr_cnt + bb_dfs->sub20_5_lng_rdr_cnt +
			bb_dfs->sub20_6_lng_rdr_cnt + bb_dfs->sub20_7_lng_rdr_cnt;
		}

		// short pulse determine
		for (i = 0; i < (bb_dfs->rpt_rdr_cnt) ; i++) {
			adv_pri_cnt_temp = 0, adv_pri_cnt_temp0 = 0, adv_pri_cnt_temp1 = 0;
			if ((bb_dfs->sub20_0_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_0_pw[i] <= pw_ub[k]) &&
			    (bb_dfs->sub20_0_pri[i] >= pri_lb[k]) && (bb_dfs->sub20_0_pri[i] <= pri_ub[k])) {
				halbb_radar_cnt_accumulate(bb, i, false, k, false);
				for (m = 0; m < (bb_dfs->rpt_rdr_cnt); m++) {  // count report which pri is similar
					if (((bb_dfs->sub20_0_pw[m] >= pw_lb[k]) && (bb_dfs->sub20_0_pw[m] <= pw_ub[k])) &&
					     (DIFF_2(bb_dfs->sub20_0_pri[i],bb_dfs->sub20_0_pri[m]) <= bb_dfs->adv_pri_ofst)) {
					     	adv_pri_cnt_temp0++;
					}
					if (((bb_dfs->sub20_1_pw[m] >= pw_lb[k]) && (bb_dfs->sub20_1_pw[m] <= pw_ub[k])) &&
					     (DIFF_2(bb_dfs->sub20_0_pri[i],bb_dfs->sub20_1_pri[m]) <= bb_dfs->adv_pri_ofst)) {
					     	adv_pri_cnt_temp0++;
					}
				}
			}
			if ((bb_dfs->sub20_1_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_1_pw[i] <= pw_ub[k]) &&
			    (bb_dfs->sub20_1_pri[i] >= pri_lb[k]) && (bb_dfs->sub20_1_pri[i] <= pri_ub[k])) {
				halbb_radar_cnt_accumulate(bb, i, true, k, false);
				for (m = 0; m < (bb_dfs->rpt_rdr_cnt); m++) {  // count report which pri is similar
					if (((bb_dfs->sub20_0_pw[m] >= pw_lb[k]) && (bb_dfs->sub20_0_pw[m] <= pw_ub[k])) &&
					     (DIFF_2(bb_dfs->sub20_1_pri[i],bb_dfs->sub20_0_pri[m]) <= bb_dfs->adv_pri_ofst)) {
					     	adv_pri_cnt_temp1++;
					}
					if (((bb_dfs->sub20_1_pw[m] >= pw_lb[k]) && (bb_dfs->sub20_1_pw[m] <= pw_ub[k])) &&
					     (DIFF_2(bb_dfs->sub20_1_pri[i],bb_dfs->sub20_1_pri[m]) <= bb_dfs->adv_pri_ofst)) {
					     	adv_pri_cnt_temp1++;
					}
				}
			}
			adv_pri_cnt_temp = MAX_2(adv_pri_cnt_temp0, adv_pri_cnt_temp1);
			bb_dfs->adv_pri_cnt[k] = MAX_2(bb_dfs->adv_pri_cnt[k], adv_pri_cnt_temp);

			/* Judge indepently because pw is small and pri is fixed to prevent radar at middle of two sub20 splitting report*/
			// Ref. type and W56 Type2 judge (PRI is same)
			if (((bb_dfs->dfs_rgn_domain == DFS_REGD_FCC || bb_dfs->dfs_rgn_domain == DFS_REGD_ETSI) && (k == DFS_RDR_TYP_NUM-1)) ||
			     ((bb_dfs->is_mic_w56) && (k == 1))) {
				if ((bb_dfs->sub20_0_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_0_pw[i] <= pw_ub[k]) &&
				    ((DIFF_2(bb_dfs->sub20_0_pri[i],REF_TYPE_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_0_pri[i],2*REF_TYPE_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_0_pri[i],3*REF_TYPE_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_0_pri[i],4*REF_TYPE_PRI) <= bb_dfs->adv_pri_ofst)))
				    	halbb_radar_cnt_accumulate(bb, i, false, k, false);
				if ((bb_dfs->sub20_1_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_1_pw[i] <= pw_ub[k]) &&
				    ((DIFF_2(bb_dfs->sub20_1_pri[i],REF_TYPE_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_1_pri[i],2*REF_TYPE_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_1_pri[i],3*REF_TYPE_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_1_pri[i],4*REF_TYPE_PRI) <= bb_dfs->adv_pri_ofst)))
				    	halbb_radar_cnt_accumulate(bb, i, true, k, false);
			}
			// SRRC Ref. type judge
			else if ((bb_dfs->dfs_rgn_domain == DFS_REGD_ETSI) && (k == DFS_RDR_TYP_NUM-2)) {
				if ((bb_dfs->sub20_0_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_0_pw[i] <= pw_ub[k]) &&
				    ((DIFF_2(bb_dfs->sub20_0_pri[i],SRRC_RFE_TYPE_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_0_pri[i],2*SRRC_RFE_TYPE_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_0_pri[i],3*SRRC_RFE_TYPE_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_0_pri[i],4*SRRC_RFE_TYPE_PRI) <= bb_dfs->adv_pri_ofst)))
				    	halbb_radar_cnt_accumulate(bb, i, false, k, false);
				if ((bb_dfs->sub20_1_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_1_pw[i] <= pw_ub[k]) &&
				    ((DIFF_2(bb_dfs->sub20_1_pri[i],SRRC_RFE_TYPE_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_1_pri[i],2*SRRC_RFE_TYPE_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_1_pri[i],3*SRRC_RFE_TYPE_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_1_pri[i],4*SRRC_RFE_TYPE_PRI) <= bb_dfs->adv_pri_ofst)))
				    	halbb_radar_cnt_accumulate(bb, i, true, k, false);
			}
			// W56 type1 judge
			else if ((bb_dfs->is_mic_w56) && (k == 0)) {
				if ((bb_dfs->sub20_0_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_0_pw[i] <= pw_ub[k]) &&
				    ((DIFF_2(bb_dfs->sub20_0_pri[i],W56_Type1_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_0_pri[i],2*W56_Type1_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_0_pri[i],3*W56_Type1_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_0_pri[i],4*W56_Type1_PRI) <= bb_dfs->adv_pri_ofst)))
				    	halbb_radar_cnt_accumulate(bb, i, false, k, false);
				if ((bb_dfs->sub20_1_pw[i] >= pw_lb[k]) && (bb_dfs->sub20_1_pw[i] <= pw_ub[k]) &&
				    ((DIFF_2(bb_dfs->sub20_1_pri[i],W56_Type1_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_1_pri[i],2*W56_Type1_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_1_pri[i],3*W56_Type1_PRI) <= bb_dfs->adv_pri_ofst) ||
				    (DIFF_2(bb_dfs->sub20_1_pri[i],4*W56_Type1_PRI) <= bb_dfs->adv_pri_ofst)))
				    	halbb_radar_cnt_accumulate(bb, i, true, k, false);
			}
			/*=======================================================================*/
		}
		// accumulate all sub20 report
		bb_dfs->srt_rdr_cnt[k] = bb_dfs->sub20_0_srt_rdr_cnt[k] + bb_dfs->sub20_1_srt_rdr_cnt[k] +
					bb_dfs->sub20_2_srt_rdr_cnt[k] + bb_dfs->sub20_3_srt_rdr_cnt[k] +
					bb_dfs->sub20_4_srt_rdr_cnt[k] + bb_dfs->sub20_5_srt_rdr_cnt[k] +
					bb_dfs->sub20_6_srt_rdr_cnt[k] + bb_dfs->sub20_7_srt_rdr_cnt[k];

		bb_dfs->pw_lbd[k] = pw_lb[k];
		bb_dfs->pw_ubd[k] = pw_ub[k];
		bb_dfs->pri_lbd[k] = pri_lb[k];
		bb_dfs->pri_ubd[k] = pri_ub[k];
	}
}

u16 halbb_radar_judge_gen2(struct bb_info *bb)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	struct bb_link_info *link = &bb->bb_link_i;
	struct bb_env_mntr_info *env_mntr = &bb->bb_env_mntr_i;
	struct bb_stat_info *stat = &bb->bb_stat_i;
	struct bb_fa_info *fa = &stat->bb_fa_i;
	u8 bw = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.bw;
	u8 chan = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.chan;
	u8 valid_rpt_cnt = 0;
	u16 i = 0, rdr_detected = 0;

	// lots of pri_full_cnt in TP mode will decrease detection rate
	valid_rpt_cnt = ((bb_dfs->idle_flag == true) ? (bb_dfs->rpt_rdr_cnt) : (bb_dfs->rpt_rdr_cnt - bb_dfs->pri_full_cnt));

	/* Check if DFS matching cnts exceed ppb th*/
	for (i = 0; i < DFS_RDR_TYP_NUM ; i++) {
		if ((i == DFS_L_RDR_IDX) && (bb_dfs->l_rdr_exst_flag)) { // long pulse
			if (bb_dfs->lng_rdr_cnt >= bb_dfs->ppb_typ_th[DFS_L_RDR_IDX]) {
				BB_DFS_DBG(bb, HWDET_PRINT, "Long Rdr reaches threshold (ppb_th:%d)!\n",bb_dfs->ppb_typ_th[DFS_L_RDR_IDX]);
				if (!bb_dfs->sub20_detect_en) {   // full band detect
					rdr_detected = RDR_FULL;
					BB_DFS_DBG(bb, HWDET_PRINT, "Long Rdr detected in FB!\n");
				}
				else {   // sub20 detect
					if (bb_dfs->sub20_0_lng_rdr_cnt) {
						rdr_detected |=  RDR_SUB20_0;
						BB_DFS_DBG(bb, HWDET_PRINT, "Long Rdr detected in sub20_0 !\n");
					}
					if (bb_dfs->sub20_1_lng_rdr_cnt) {
						rdr_detected |=  RDR_SUB20_1;
						BB_DFS_DBG(bb, HWDET_PRINT, "Long Rdr detected in sub20_1 !\n");
					}
					if (bb_dfs->sub20_2_lng_rdr_cnt) {
						rdr_detected |=  RDR_SUB20_2;
						BB_DFS_DBG(bb, HWDET_PRINT, "Long Rdr detected in sub20_2 !\n");
					}
					if (bb_dfs->sub20_3_lng_rdr_cnt) {
						rdr_detected |=  RDR_SUB20_3;
						BB_DFS_DBG(bb, HWDET_PRINT, "Long Rdr detected in sub20_3 !\n");
					}
					if (bb_dfs->sub20_4_lng_rdr_cnt) {
						rdr_detected |=  RDR_SUB20_4;
						BB_DFS_DBG(bb, HWDET_PRINT, "Long Rdr detected in sub20_4 !\n");
					}
					if (bb_dfs->sub20_5_lng_rdr_cnt) {
						rdr_detected |=  RDR_SUB20_5;
						BB_DFS_DBG(bb, HWDET_PRINT, "Long Rdr detected in sub20_5 !\n");
					}
					if (bb_dfs->sub20_6_lng_rdr_cnt) {
						rdr_detected |=  RDR_SUB20_6;
						BB_DFS_DBG(bb, HWDET_PRINT, "Long Rdr detected in sub20_6 !\n");
					}
					if (bb_dfs->sub20_7_lng_rdr_cnt) {
						rdr_detected |=  RDR_SUB20_7;
						BB_DFS_DBG(bb, HWDET_PRINT, "Long Rdr detected in sub20_7 !\n");
					}
				}
				bb_dfs->lng_rdr_cnt = 0;
				bb_dfs->sub20_0_lng_rdr_cnt = 0;
				bb_dfs->sub20_1_lng_rdr_cnt = 0;
				bb_dfs->sub20_2_lng_rdr_cnt = 0;
				bb_dfs->sub20_3_lng_rdr_cnt = 0;
				bb_dfs->sub20_4_lng_rdr_cnt = 0;
				bb_dfs->sub20_5_lng_rdr_cnt = 0;
				bb_dfs->sub20_6_lng_rdr_cnt = 0;
				bb_dfs->sub20_7_lng_rdr_cnt = 0;
			}
		}
		else { // short pulse
			if (bb_dfs->srt_rdr_cnt[i] >= bb_dfs->ppb_typ_th[i]) {
				BB_DFS_DBG(bb, HWDET_PRINT, "Rdr Type %d reaches threshold (ppb_th:%d)!\n", (i+1), bb_dfs->ppb_typ_th[i]);

				// Avoid FRD ================================================================
				// srt_cnt must larger than rpt_cnt/2 and adv_pri_cnt must larger than srt_cnt/2
				if ((bb_dfs->srt_rdr_cnt[i] >= (valid_rpt_cnt >> 1)) && (bb_dfs->adv_pri_cnt[i] >= (bb_dfs->srt_rdr_cnt[i] >> 1))) 
					goto DETECTING_END;
				// ETSI type4/5 is mutiple pri so can not consider adv_pri
				else if ((bb_dfs->dfs_rgn_domain == DFS_REGD_ETSI) && (i == 4 || i == 5) && (bb_dfs->srt_rdr_cnt[i] >= (bb_dfs->rpt_rdr_cnt >> 1)))
					goto DETECTING_END;
				// W53 type3~8 is mix pulse, (rpt/2) reports are short pulse so they can not consider
				else if (bb_dfs->is_mic_w53 && (i != 0 && i !=1) && (bb_dfs->srt_rdr_cnt[i] >= (bb_dfs->rpt_rdr_cnt >> 2)))
					goto DETECTING_END;
				// W56 type1.2 judge indepently so cat not consider adv_pri
				else if (bb_dfs->is_mic_w56 && (i == 0 || i == 1) && (bb_dfs->srt_rdr_cnt[i] >= (bb_dfs->rpt_rdr_cnt >> 1)))
					goto DETECTING_END;
				// Ref type. detect
				else if ((((bb_dfs->dfs_rgn_domain == DFS_REGD_FCC) || (bb_dfs->dfs_rgn_domain == DFS_REGD_ETSI)) && (i == DFS_RDR_TYP_NUM-1)) ||
				             ((bb_dfs->dfs_rgn_domain == DFS_REGD_ETSI) && (i == DFS_RDR_TYP_NUM-2))) {
					// max ref. type rdr count is 20 in SRRC
					if ((rdr_detected == false) && ((bb_dfs->rpt_rdr_cnt >= 35) || (bb_dfs->srt_rdr_cnt[i] < (bb_dfs->rpt_rdr_cnt >> 1)))) {
						BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "BRK Ref. tye RDR due to much rpt or less srt_cnt !\n");
						continue;
					}
					BB_DFS_DBG(bb, HWDET_PRINT, "Appear Ref. type RDR !\n");
					goto DETECTING_END;
				}
				else {
					BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "BRK due to less srt_cnt or adv_pri_cnt!\n");
					rdr_detected = false;
					continue;
				}

				DETECTING_END:
				if (!bb_dfs->sub20_detect_en) {  // full band detect
					rdr_detected = RDR_FULL;
					BB_DFS_DBG(bb, HWDET_PRINT, "Rdr Type %d detected in FB !\n", (i+1));
				}
				else {   // sub20 detect
					if (bb_dfs->sub20_0_srt_rdr_cnt[i]) {
						rdr_detected |= RDR_SUB20_0;
						BB_DFS_DBG(bb, HWDET_PRINT, "Rdr Type %d detected in sub20_0 !\n", (i+1));
					}
					if (bb_dfs->sub20_1_srt_rdr_cnt[i]) {
						rdr_detected |= RDR_SUB20_1;
						BB_DFS_DBG(bb, HWDET_PRINT, "Rdr Type %d detected in sub20_1 !\n", (i+1));
					}
					if (bb_dfs->sub20_2_srt_rdr_cnt[i]) {
						rdr_detected |= RDR_SUB20_2;
						BB_DFS_DBG(bb, HWDET_PRINT, "Rdr Type %d detected in sub20_2 !\n", (i+1));
					}
					if (bb_dfs->sub20_3_srt_rdr_cnt[i]) {
						rdr_detected |= RDR_SUB20_3;
						BB_DFS_DBG(bb, HWDET_PRINT, "Rdr Type %d detected in sub20_3 !\n", (i+1));
					}
					if (bb_dfs->sub20_4_srt_rdr_cnt[i]) {
						rdr_detected |= RDR_SUB20_4;
						BB_DFS_DBG(bb, HWDET_PRINT, "Rdr Type %d detected in sub20_4 !\n", (i+1));
					}
					if (bb_dfs->sub20_5_srt_rdr_cnt[i]) {
						rdr_detected |= RDR_SUB20_5;
						BB_DFS_DBG(bb, HWDET_PRINT, "Rdr Type %d detected in sub20_5 !\n", (i+1));
					}
					if (bb_dfs->sub20_6_srt_rdr_cnt[i]) {
						rdr_detected |= RDR_SUB20_6;
						BB_DFS_DBG(bb, HWDET_PRINT, "Rdr Type %d detected in sub20_6 !\n", (i+1));
					}
					if (bb_dfs->sub20_7_srt_rdr_cnt[i]) {
						rdr_detected |= RDR_SUB20_7;
						BB_DFS_DBG(bb, HWDET_PRINT, "Rdr Type %d detected in sub20_7 !\n", (i+1));
					}
				}
			}
		}
	}

	/*Drop the detected RDR to prevent FRD*/
	if (rdr_detected) {
		if (bb_dfs->detect_state) {
			rdr_detected = false;
			BB_DFS_DBG(bb, (HWDET_PRINT | BRK_PRINT), "RDR drop by ENV !\n");
		}
	}
	return rdr_detected;
}

u16 halbb_radar_detect_gen2(struct bb_info *bb, struct hal_dfs_rpt *dfs_rpt)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	struct bb_link_info *link = &bb->bb_link_i;
	u8 bw = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.bw;
	u8 chan = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.chan;
	u16 i = 0, rdr_detected = 0;

	/* DFS Info Parsing/Processing*/
	for (i = 0; i < (dfs_rpt->dfs_num) ; i++)
		halbb_radar_info_processing_gen2(bb, dfs_rpt, i);

	/*compare and accumulate radar cnt*/
	halbb_radar_ptrn_cmprn_gen2(bb);

	// Prevent lots of report
	if (!((bb_dfs->rpt_rdr_cnt > bb_dfs->fk_dfs_num_th) || (bb_dfs->chrp_obsrv_flag == true))) {
		BB_DFS_DBG(bb, TRIVIL_PRINT, "Non-existent form of DFS!\n");
		goto DETECTING_END;
	}

	halbb_radar_show_log(bb, dfs_rpt, false);  // Print HW detect

	rdr_detected =  halbb_radar_judge_gen2(bb);  // Judge final radar by cnt and FRD

	if (rdr_detected) {
		halbb_radar_show_log(bb, dfs_rpt, true);  // Print SW detect 
		// reset lng rdr cnt
		bb_dfs->lng_rdr_cnt = 0;
		bb_dfs->sub20_0_lng_rdr_cnt = 0;
		bb_dfs->sub20_1_lng_rdr_cnt = 0;
		bb_dfs->sub20_2_lng_rdr_cnt = 0;
		bb_dfs->sub20_3_lng_rdr_cnt = 0;
		bb_dfs->sub20_4_lng_rdr_cnt = 0;
		bb_dfs->sub20_5_lng_rdr_cnt = 0;
		bb_dfs->sub20_6_lng_rdr_cnt = 0;
		bb_dfs->sub20_7_lng_rdr_cnt = 0;

		BB_DFS_DBG(bb, (SWDET_PRINT | HWDET_PRINT), "rdr_detected = 0x%x\n",rdr_detected);
		if (bb_dfs->dfs_dbg_mode) {
			BB_DBG(bb, DBG_DFS, "Radar is detected in DFS debug mode!\n");
			rdr_detected = false;
		}
		else
			BB_DBG(bb, DBG_DFS, "Radar is detected in DFS general mode!\n");
	}

	DETECTING_END:
	/* Reset SW Counter/Flag */
	bb_dfs->rpt_rdr_cnt = 0;
	bb_dfs->pri_full_cnt = 0;
	for (i = 0; i < DFS_RDR_TYP_NUM; i++) {
		bb_dfs->srt_rdr_cnt[i] = 0;
		bb_dfs->sub20_0_srt_rdr_cnt[i] = 0;
		bb_dfs->sub20_1_srt_rdr_cnt[i] = 0;
		bb_dfs->sub20_2_srt_rdr_cnt[i] = 0;
		bb_dfs->sub20_3_srt_rdr_cnt[i] = 0;
		bb_dfs->sub20_4_srt_rdr_cnt[i] = 0;
		bb_dfs->sub20_5_srt_rdr_cnt[i] = 0;
		bb_dfs->sub20_6_srt_rdr_cnt[i] = 0;
		bb_dfs->sub20_7_srt_rdr_cnt[i] = 0;
		bb_dfs->adv_pri_cnt[i] = 0;
	}
	for (i = 0; i < dfs_rpt->dfs_num ; i++) {
		bb_dfs->dfs_more[i] = 0;
		bb_dfs->dfs_phy_def[i] = 0;
		bb_dfs->format[i] = 0;
		bb_dfs->bbid[i] = 0;
		bb_dfs->seq_num[i] = 0;
		bb_dfs->RSVD[i] = 0;
		bb_dfs->sub20_1_pw[i] = 0;
		bb_dfs->sub20_1_pri[i] = 0;
		bb_dfs->sub20_1_chirp_flag[i] = 0;
		bb_dfs->sub20_1_pres[i] = 0;
		bb_dfs->sub20_0_pw[i] = 0;
		bb_dfs->sub20_0_pri[i] = 0;
		bb_dfs->sub20_0_chirp_flag[i] = 0;
		bb_dfs->sub20_0_pres[i] = 0;
	}
	//store lng_rdr_cnt for invalid check
	bb_dfs->lng_rdr_cnt_pre = bb_dfs->lng_rdr_cnt;
	return rdr_detected;
}

void halbb_dfs_dyn_setting_gen2(struct bb_info *bb)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	struct bb_link_info *link = &bb->bb_link_i;
	struct bb_env_mntr_info *env_mntr = &bb->bb_env_mntr_i;
	struct bb_dfs_cr_info *cr = &bb->bb_cmn_hooker->bb_dfs_cr_i;
	struct bb_stat_info *stat = &bb->bb_stat_i;
	struct bb_fa_info *fa = &stat->bb_fa_i;
	struct bb_ch_info *ch = &bb->bb_ch_i;
	u8 bw = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.bw;
	u8 chan = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.chan;
	u8 i = 0;

	if (bb_dfs->first_dyn_set_flag) {
		bb_dfs->first_dyn_set_flag = false;
		bb_dfs->detect_state = DFS_Adaptive_State;
		bb_dfs->adap_detect_cnt = 1;  // mask first 2s when init
		BB_DFS_DBG(bb, DYN_PRINT, "First dyn setting !\n");
		BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Adaptive State : [%d]\n", bb_dfs->adap_detect_cnt);
		return;
	}

	if (link->total_tp < bb_dfs->dfs_tp_th)
		bb_dfs->idle_flag = true;
	else 
		bb_dfs->idle_flag = false;

	if (bb_dfs->idle_flag) {
		bb_dfs->pw_ofst = 2;
		bb_dfs->pri_ofst = 1;
		bb_dfs->adv_pri_ofst = 1;
		for (i = 0; i < DFS_RDR_TYP_NUM; i++) {
			bb_dfs->ppb_typ_th[i] = bb_dfs->ppb_tab[i];
		}
	}
	else {
		bb_dfs->pw_ofst = 3;
		bb_dfs->pri_ofst = 2;
		bb_dfs->adv_pri_ofst = 2;
		for (i = 0; i < DFS_RDR_TYP_NUM; i++) {
			bb_dfs->ppb_typ_th[i] = bb_dfs->ppb_tp_tab[i]; 
		}
	}

	// for FCC detection BW test
	if (bb_dfs->dfs_rgn_domain == DFS_REGD_FCC) {
		if (bb_dfs->idle_flag && bb_dfs->dfs_dbg_mode) {
			bb_dfs->ppb_typ_th[DFS_RDR_TYP_NUM-1] = 3;
			halbb_set_reg(bb, cr->dfs_l2h_th, cr->dfs_l2h_th_m, bb_dfs->l2h_val);
			BB_DFS_DBG(bb, DYN_PRINT, "Dynamic set L2H to 0x%x\n", bb_dfs->l2h_val);
		}
		else {
			halbb_set_reg(bb, cr->dfs_l2h_th, cr->dfs_l2h_th_m, bb_dfs->l2h_init_val);
			BB_DFS_DBG(bb, DYN_PRINT, "Reset L2H to 0x%x\n", bb_dfs->l2h_init_val);
		}
	}
	else {
		halbb_set_reg(bb, cr->dfs_l2h_th, cr->dfs_l2h_th_m, bb_dfs->l2h_init_val);
		BB_DFS_DBG(bb, DYN_PRINT, "Reset L2H to 0x%x\n", bb_dfs->l2h_init_val);
	}
	/*============================================================*/

	/* Supressing false detection */
	switch (bb_dfs->detect_state) {
	case DFS_Normal_State:
		if ((env_mntr->env_mntr_rpt_bg.nhm_idle_ratio < bb_dfs->dfs_idle_prd_th && env_mntr->env_mntr_rpt_bg.nhm_idle_ratio != ENV_MNTR_FAIL_BYTE) ||
			 (env_mntr->env_mntr_rpt_bg.nhm_ratio > bb_dfs->dfs_nhm_th && env_mntr->env_mntr_rpt_bg.nhm_ratio != ENV_MNTR_FAIL_BYTE) ||
			 (env_mntr->env_mntr_rpt_bg.nhm_cca_ratio > bb_dfs->dfs_rx_rto_th && env_mntr->env_mntr_rpt_bg.nhm_cca_ratio != ENV_MNTR_FAIL_BYTE) ||
			 (fa->cnt_fail_all > bb_dfs->dfs_fa_th) ||
			 ((bb->ic_sub_type == BB_IC_SUB_TYPE_8852C_8852D) && ((ch->rssi_max >> 1) > bb_dfs->dfs_rssi_th))) {
			bb_dfs->adap_detect_cnt = bb_dfs->adap_detect_cnt_init;
			bb_dfs->adap_detect_cnt_all += 1;
			bb_dfs->detect_state = DFS_Adaptive_State;
			BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] NHM/FA triggers Adaptive State\n");
			BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Adaptive State : [%d]\n", bb_dfs->adap_detect_cnt);
		}
		else {
			BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Normal State\n");
		}
		BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Adap_detect_cnt_all = [%d]\n", bb_dfs->adap_detect_cnt_all);

		break;

	case DFS_Adaptive_State:
		bb_dfs->adap_detect_cnt = SUBTRACT_TO_0(bb_dfs->adap_detect_cnt,1);
		if ((env_mntr->env_mntr_rpt_bg.nhm_idle_ratio < bb_dfs->dfs_idle_prd_th && env_mntr->env_mntr_rpt_bg.nhm_idle_ratio != ENV_MNTR_FAIL_BYTE) ||
			 (env_mntr->env_mntr_rpt_bg.nhm_ratio > bb_dfs->dfs_nhm_th && env_mntr->env_mntr_rpt_bg.nhm_ratio != ENV_MNTR_FAIL_BYTE) ||
			 (env_mntr->env_mntr_rpt_bg.nhm_cca_ratio > bb_dfs->dfs_rx_rto_th && env_mntr->env_mntr_rpt_bg.nhm_cca_ratio != ENV_MNTR_FAIL_BYTE) ||
			 (fa->cnt_fail_all > bb_dfs->dfs_fa_th) || (link->total_tp == 0 && env_mntr->env_mntr_rpt_bg.nhm_idle_ratio <= 96) ||
			 ((bb->ic_sub_type == BB_IC_SUB_TYPE_8852C_8852D) && ((ch->rssi_max >> 1) > bb_dfs->dfs_rssi_th))) {
			//bb_dfs->adap_detect_cnt = MIN_2(255, bb_dfs->adap_detect_cnt + bb_dfs->adap_detect_cnt_add);
			bb_dfs->adap_detect_cnt = bb_dfs->adap_detect_cnt_add;
			bb_dfs->adap_detect_cnt_all = MIN_2(255, bb_dfs->adap_detect_cnt_all+1);
			BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] NHM/FA triggers Adaptive State again\n");
		}
		if (bb_dfs->adap_detect_cnt == 0) {
			bb_dfs->detect_state = DFS_Normal_State;
			BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Normal State\n");
		} 
		else {
			BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Adaptive State : [%d]\n", bb_dfs->adap_detect_cnt);
		}
		BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Adap_detect_cnt_all = [%d]\n", bb_dfs->adap_detect_cnt_all);

		break;

	default:
		bb_dfs->detect_state = DFS_Normal_State;
		bb_dfs->adap_detect_cnt = 0;

		break;
	}
}


void halbb_radar_show_log(struct bb_info *bb, struct hal_dfs_rpt *dfs_rpt, u16 rdr_detected)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	struct bb_link_info *link = &bb->bb_link_i;
	u8 bw = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.bw;
	u8 chan = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.chan;
	u16 i = 0;

	if (((!rdr_detected) && (bb_dfs->dbg_print_component & HWDET_PRINT)) ||(rdr_detected && (bb_dfs->dbg_print_component & SWDET_PRINT))) {
		BB_DBG(bb, DBG_DFS, "\n");
		BB_DBG(bb, DBG_DFS, "[halbb_radar_detect_gen2]===>\n");
		BB_DBG(bb, DBG_DFS, "DFS Region Domain = %d, BW = %d, Channel = %d\n",
		       bb_dfs->dfs_rgn_domain, bw, chan);
		BB_DBG(bb, DBG_DFS, "phy_idx = %d, dfs_num = %d, rpt_num = %d, pri_full_num =%d\n",
		       dfs_rpt->phy_idx, dfs_rpt->dfs_num, bb_dfs->rpt_rdr_cnt, bb_dfs->pri_full_cnt);
		BB_DBG(bb, DBG_DFS, "is_link = %d, is_CAC = %d, is_idle = %d\n",
		       link->is_linked, bb_dfs->In_CAC_Flag, bb_dfs->idle_flag);

		for (i = 0; i < DFS_RDR_TYP_NUM ; i++) {
			BB_DBG(bb, DBG_DFS, "Type %d: [pw_lbd-pw_ubd], [pri_lbd-pri_ubd], [ppb_thd] = [%d-%d], [%d-%d], [%d]\n",
			(i + 1), bb_dfs->pw_lbd[i], bb_dfs->pw_ubd[i], bb_dfs->pri_lbd[i], bb_dfs->pri_ubd[i], bb_dfs->ppb_typ_th[i]);
		}

		/*show long radar monitor time*/
		if (bb_dfs->l_rdr_exst_flag) {
			BB_DBG(bb, DBG_DFS, "[mntr_prd, sys_t, chrp_srt_t]: [%d, %d, %d]\n",
				(bb->bb_sys_up_time - bb_dfs->chrp_srt_t), bb->bb_sys_up_time, bb_dfs->chrp_srt_t);
		}
		BB_DBG(bb, DBG_DFS, "\n");
		BB_DBG(bb, DBG_DFS, "DFS_RPT: [more, phy_def, format, bbid, pw0, pri0, pw1, pri1]\n");
		for (i = 0; i < (bb_dfs->rpt_rdr_cnt) ; i++) {
			BB_DBG(bb, DBG_DFS, "DFS_RPT: [%d, %d, (%d), %d && %3d, %3d && %3d, %3d]\n",
			bb_dfs->dfs_more[i], bb_dfs->dfs_phy_def[i], bb_dfs->format[i], bb_dfs->bbid[i],
			bb_dfs->sub20_0_pw[i], bb_dfs->sub20_0_pri[i],
			bb_dfs->sub20_1_pw[i], bb_dfs->sub20_1_pri[i]);
		}

		BB_DBG(bb, DBG_DFS, "\n");
		BB_DBG(bb, DBG_DFS, "sub20_0_srt_rdr_cnt = [%2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d]\n",
		bb_dfs->sub20_0_srt_rdr_cnt[0], bb_dfs->sub20_0_srt_rdr_cnt[1],
		bb_dfs->sub20_0_srt_rdr_cnt[2], bb_dfs->sub20_0_srt_rdr_cnt[3],
		bb_dfs->sub20_0_srt_rdr_cnt[4], bb_dfs->sub20_0_srt_rdr_cnt[5],
		bb_dfs->sub20_0_srt_rdr_cnt[6], bb_dfs->sub20_0_srt_rdr_cnt[7]);
		if (bw >= CHANNEL_WIDTH_40) {
			BB_DBG(bb, DBG_DFS, "sub20_1_srt_rdr_cnt = [%2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d]\n",
			bb_dfs->sub20_1_srt_rdr_cnt[0], bb_dfs->sub20_1_srt_rdr_cnt[1],
			bb_dfs->sub20_1_srt_rdr_cnt[2], bb_dfs->sub20_1_srt_rdr_cnt[3],
			bb_dfs->sub20_1_srt_rdr_cnt[4], bb_dfs->sub20_1_srt_rdr_cnt[5],
			bb_dfs->sub20_1_srt_rdr_cnt[6], bb_dfs->sub20_1_srt_rdr_cnt[7]);
		}
		if (bw >= CHANNEL_WIDTH_80) {
			BB_DBG(bb, DBG_DFS, "sub20_2_srt_rdr_cnt = [%2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d]\n",
			bb_dfs->sub20_2_srt_rdr_cnt[0], bb_dfs->sub20_2_srt_rdr_cnt[1],
			bb_dfs->sub20_2_srt_rdr_cnt[2], bb_dfs->sub20_2_srt_rdr_cnt[3],
			bb_dfs->sub20_2_srt_rdr_cnt[4], bb_dfs->sub20_2_srt_rdr_cnt[5],
			bb_dfs->sub20_2_srt_rdr_cnt[6], bb_dfs->sub20_2_srt_rdr_cnt[7]);
			BB_DBG(bb, DBG_DFS, "sub20_3_srt_rdr_cnt = [%2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d]\n",
			bb_dfs->sub20_3_srt_rdr_cnt[0], bb_dfs->sub20_3_srt_rdr_cnt[1],
			bb_dfs->sub20_3_srt_rdr_cnt[2], bb_dfs->sub20_3_srt_rdr_cnt[3],
			bb_dfs->sub20_3_srt_rdr_cnt[4], bb_dfs->sub20_3_srt_rdr_cnt[5],
			bb_dfs->sub20_3_srt_rdr_cnt[6], bb_dfs->sub20_3_srt_rdr_cnt[7]);
		}
		if (bw >= CHANNEL_WIDTH_160) {
			BB_DBG(bb, DBG_DFS, "sub20_4_srt_rdr_cnt = [%2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d]\n",
			bb_dfs->sub20_4_srt_rdr_cnt[0], bb_dfs->sub20_4_srt_rdr_cnt[1],
			bb_dfs->sub20_4_srt_rdr_cnt[2], bb_dfs->sub20_4_srt_rdr_cnt[3],
			bb_dfs->sub20_4_srt_rdr_cnt[4], bb_dfs->sub20_4_srt_rdr_cnt[5],
			bb_dfs->sub20_4_srt_rdr_cnt[6], bb_dfs->sub20_4_srt_rdr_cnt[7]);
			BB_DBG(bb, DBG_DFS, "sub20_5_srt_rdr_cnt = [%2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d]\n",
			bb_dfs->sub20_5_srt_rdr_cnt[0], bb_dfs->sub20_5_srt_rdr_cnt[1],
			bb_dfs->sub20_5_srt_rdr_cnt[2], bb_dfs->sub20_5_srt_rdr_cnt[3],
			bb_dfs->sub20_5_srt_rdr_cnt[4], bb_dfs->sub20_5_srt_rdr_cnt[5],
			bb_dfs->sub20_5_srt_rdr_cnt[6], bb_dfs->sub20_5_srt_rdr_cnt[7]);
			BB_DBG(bb, DBG_DFS, "sub20_6_srt_rdr_cnt = [%2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d]\n",
			bb_dfs->sub20_6_srt_rdr_cnt[0], bb_dfs->sub20_6_srt_rdr_cnt[1],
			bb_dfs->sub20_6_srt_rdr_cnt[2], bb_dfs->sub20_6_srt_rdr_cnt[3],
			bb_dfs->sub20_6_srt_rdr_cnt[4], bb_dfs->sub20_6_srt_rdr_cnt[5],
			bb_dfs->sub20_6_srt_rdr_cnt[6], bb_dfs->sub20_6_srt_rdr_cnt[7]);
			BB_DBG(bb, DBG_DFS, "sub20_7_srt_rdr_cnt = [%2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d]\n",
			bb_dfs->sub20_7_srt_rdr_cnt[0], bb_dfs->sub20_7_srt_rdr_cnt[1],
			bb_dfs->sub20_7_srt_rdr_cnt[2], bb_dfs->sub20_7_srt_rdr_cnt[3],
			bb_dfs->sub20_7_srt_rdr_cnt[4], bb_dfs->sub20_7_srt_rdr_cnt[5],
			bb_dfs->sub20_7_srt_rdr_cnt[6], bb_dfs->sub20_7_srt_rdr_cnt[7]);
		}
		BB_DBG(bb, DBG_DFS, "srt_rdr_cnt_all = [%2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d]\n",
		bb_dfs->srt_rdr_cnt[0], bb_dfs->srt_rdr_cnt[1],
		bb_dfs->srt_rdr_cnt[2], bb_dfs->srt_rdr_cnt[3],
		bb_dfs->srt_rdr_cnt[4], bb_dfs->srt_rdr_cnt[5],
		bb_dfs->srt_rdr_cnt[6], bb_dfs->srt_rdr_cnt[7]);
		if (bb_dfs->l_rdr_exst_flag) {
			BB_DBG(bb, DBG_DFS, "lng_rdr_cnt_sub20 = [%2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d]\n",
			bb_dfs->sub20_0_lng_rdr_cnt, bb_dfs->sub20_1_lng_rdr_cnt,
			bb_dfs->sub20_2_lng_rdr_cnt, bb_dfs->sub20_3_lng_rdr_cnt,
			bb_dfs->sub20_4_lng_rdr_cnt, bb_dfs->sub20_5_lng_rdr_cnt,
			bb_dfs->sub20_6_lng_rdr_cnt, bb_dfs->sub20_7_lng_rdr_cnt);
			BB_DBG(bb, DBG_DFS, "lng_rdr_cnt_all = [%d]\n", bb_dfs->lng_rdr_cnt);
		}
		BB_DBG(bb, DBG_DFS, "adv_pri_cnt = [%2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d]\n",
		bb_dfs->adv_pri_cnt[0], bb_dfs->adv_pri_cnt[1],
		bb_dfs->adv_pri_cnt[2], bb_dfs->adv_pri_cnt[3],
		bb_dfs->adv_pri_cnt[4], bb_dfs->adv_pri_cnt[5],
		bb_dfs->adv_pri_cnt[6], bb_dfs->adv_pri_cnt[7]);
		BB_DBG(bb, DBG_DFS, "\n");
	}
}

#endif

void halbb_radar_seq_inspctn(struct bb_info *bb, u16 dfs_rpt_idx,
				u8 c_num, u8 p_num, bool is_sg1, u8 c_seg, u8 p_seg)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;

#ifdef HALBB_TW_DFS_SERIES
	if (bb->ic_sub_type != BB_IC_SUB_TYPE_8852C_8852D) {
		if (dfs_rpt_idx != 0 && ( bb_dfs->rpt_sg_history_all & BIT(is_sg1) )) {
			if (p_num == DFS_MAX_SEQ_NUM) {
				if (c_num != 0 && c_seg == p_seg) {
					if (is_sg1)
						bb_dfs->n_seq_flag_sg1 = true;
					else
						bb_dfs->n_seq_flag = true;
				}
			} else {
				if (ABS_8(c_num - p_num) > 1 && c_seg == p_seg) {
					if (is_sg1)
						bb_dfs->n_seq_flag_sg1 = true;
					else
						bb_dfs->n_seq_flag = true;
				}
			}
		} else {
			bb_dfs->rpt_sg_history_all = bb_dfs->rpt_sg_history_all | BIT(is_sg1);
		}
		if (is_sg1)
			bb_dfs->lst_seq_num_sg1 = c_num;
		else
			bb_dfs->lst_seq_num = c_num;
		bb_dfs->lst_seg_idx = c_seg;
	}
	else
#endif
	{
		if (dfs_rpt_idx != 0) {
			if (p_num == DFS_MAX_SEQ_NUM) {
				if (c_num != 0)
					bb_dfs->n_seq_flag = true;
			} else {
				if (ABS_8(c_num - p_num) > 1)
					bb_dfs->n_seq_flag = true;
			}
		}
		bb_dfs->lst_seq_num = c_num;
		bb_dfs->lst_seg_idx = c_seg;
	}
}

void halbb_radar_ptrn_cmprn(struct bb_info *bb, u16 dfs_rpt_idx,
				u8 pri, u16 pw, bool chrp_flag, bool is_sg1)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	u16 j = 0, pw_lbd = 0, pri_lbd = 0, pri_ubd = 0;
	u16 pw_ubd = 0;

	bb_dfs->rpt_sg_history = bb_dfs->rpt_sg_history | BIT(is_sg1);

	if (bb_dfs->l_rdr_exst_flag)
		halbb_radar_chrp_mntr(bb, chrp_flag, is_sg1);

	/*record the min.max pw of chirp pulse and chirp pulse cnt*/
	if (chrp_flag) {
		if (is_sg1) {
			bb_dfs->chrp_rdr_cnt_sg1++;
			if (pw > bb_dfs->max_pw_chrp_sg1)
				bb_dfs->max_pw_chrp_sg1 = pw;
			if (pw < bb_dfs->min_pw_chrp_sg1)
				bb_dfs->min_pw_chrp_sg1 = pw;
		}
		else {
			bb_dfs->chrp_rdr_cnt++;
			if (pw > bb_dfs->max_pw_chrp)
				bb_dfs->max_pw_chrp = pw;
			if (pw < bb_dfs->min_pw_chrp)
				bb_dfs->min_pw_chrp = pw;
		}
	}

	for (j = 0; j < DFS_RDR_TYP_NUM ; j++) {
		pw_lbd = MIN_2(bb_dfs->pw_min_tab[j] - 1, (bb_dfs->pw_min_tab[j] * (8 - bb_dfs->pw_factor) >> 3));
	//	pw_lbd = (pw_lbd <= 1) ? 2 : pw_lbd;
		pw_ubd = MAX_2(bb_dfs->pw_max_tab[j] + 1, (bb_dfs->pw_max_tab[j] * (8 + bb_dfs->pw_factor) >> 3));
		pri_lbd = (bb_dfs->pri_min_tab[j] * (8 - bb_dfs->pri_factor) >> 3);
		if ((bb_dfs->pri_max_tab[j] * (8 + bb_dfs->pri_factor) >> 3) <= 0xDC)
			pri_ubd = (bb_dfs->pri_max_tab[j] * (8 + bb_dfs->pri_factor) >> 3);
		else
			pri_ubd = 0xDC;

		if ((j == DFS_L_RDR_IDX) && (bb_dfs->l_rdr_exst_flag)) {
			if (dfs_rpt_idx == 0 || !(bb_dfs->rpt_sg_history & BIT(is_sg1))) {
				if ((pw_lbd <= pw) && (pw_ubd >= pw) && (pri != 0) && chrp_flag) {  // first rpt doesn't consider pri because it's not real
					if (is_sg1) {
						bb_dfs->lng_rdr_cnt_sg1++;
						if (pw > bb_dfs->max_pw_lng_sg1)
							bb_dfs->max_pw_lng_sg1 = pw;
						if (pw < bb_dfs->min_pw_lng_sg1)
							bb_dfs->min_pw_lng_sg1 = pw;
					}
					else {
						bb_dfs->lng_rdr_cnt++;
						if (pw > bb_dfs->max_pw_lng)
							bb_dfs->max_pw_lng = pw;
						if (pw < bb_dfs->min_pw_lng)
							bb_dfs->min_pw_lng = pw;
					}
				}
			}
			else {
				if ((pw_lbd <= pw) && (pw_ubd >= pw) && (pri_lbd <= pri) && (pri_ubd >= pri) && chrp_flag) {
					if (is_sg1) {
						bb_dfs->lng_rdr_cnt_sg1++;
						if (pw > bb_dfs->max_pw_lng_sg1)
							bb_dfs->max_pw_lng_sg1 = pw;
						if (pw < bb_dfs->min_pw_lng_sg1)
							bb_dfs->min_pw_lng_sg1 = pw;
					}
					else {
						bb_dfs->lng_rdr_cnt++;
						if (pw > bb_dfs->max_pw_lng)
							bb_dfs->max_pw_lng = pw;
						if (pw < bb_dfs->min_pw_lng)
							bb_dfs->min_pw_lng = pw;
					}
				}
			}
		}
		else {
			if (((bb_dfs->dfs_rgn_domain == DFS_REGD_ETSI) && (j == DFS_SPCL_RDR_IDX_ETSI)) ||     // ETSI Type4 is chirp
				((bb_dfs->is_mic_w53 == true) && (j==2 || j==3 || j==4 || j ==5 || j ==6 || j==7))) { // W53 type3~8 determine by long pulse
				if ((pw_lbd <= pw) && (pw_ubd >= pw) && (pri_lbd <= pri) && (pri_ubd >= pri) && chrp_flag) {
					if (is_sg1) {
						bb_dfs->srt_rdr_cnt_sg1[j]++;
						if (pw > bb_dfs->max_pw_shrt_sg1[j])
							bb_dfs->max_pw_shrt_sg1[j] = pw;
						if (pw < bb_dfs->min_pw_shrt_sg1[j])
							bb_dfs->min_pw_shrt_sg1[j] = pw;
						if (!(dfs_rpt_idx == 0) && (bb_dfs->rpt_sg_history & BIT(is_sg1))) {
							if (pri > bb_dfs->max_pri_shrt_sg1[j])
								bb_dfs->max_pri_shrt_sg1[j] = pri;
							if (pri < bb_dfs->min_pri_shrt_sg1[j])
								bb_dfs->min_pri_shrt_sg1[j] = pri;
						}
					}
					else {
						bb_dfs->srt_rdr_cnt[j]++;
						if (pw > bb_dfs->max_pw_shrt[j])
							bb_dfs->max_pw_shrt[j] = pw;
						if (pw < bb_dfs->min_pw_shrt[j])
							bb_dfs->min_pw_shrt[j] = pw;
						if (!(dfs_rpt_idx == 0) && (bb_dfs->rpt_sg_history & BIT(is_sg1))) {
							if (pri > bb_dfs->max_pri_shrt[j])
								bb_dfs->max_pri_shrt[j] = pri;
							if (pri < bb_dfs->min_pri_shrt[j])
								bb_dfs->min_pri_shrt[j] = pri;
						}
					}
				}
			}
			else if ((bb_dfs->dfs_rgn_domain == DFS_REGD_ETSI) && (j == 4 || j==5)) { // ETSI TYPE5.6 multi pri
				if ((pw_lbd <= pw) && (pw_ubd >= pw) && (pri_lbd <= pri) && (pri_ubd >= pri) && !(chrp_flag)) {
					if (is_sg1) {
						bb_dfs->srt_rdr_cnt_sg1[j]++;
						if (pw > bb_dfs->max_pw_shrt_sg1[j])
							bb_dfs->max_pw_shrt_sg1[j] = pw;
						if (pw < bb_dfs->min_pw_shrt_sg1[j])
							bb_dfs->min_pw_shrt_sg1[j] = pw;
					}
					else {
						bb_dfs->srt_rdr_cnt[j]++;
						if (pw > bb_dfs->max_pw_shrt[j])
							bb_dfs->max_pw_shrt[j] = pw;
						if (pw < bb_dfs->min_pw_shrt[j])
							bb_dfs->min_pw_shrt[j] = pw;
					}
				}
			}
			else if ((pw_lbd <= pw) && (pw_ubd >= pw) && (pri_lbd <= pri) && (pri_ubd >= pri) && !(chrp_flag)) {
				if (is_sg1) {
					bb_dfs->srt_rdr_cnt_sg1[j]++;
					if (pw > bb_dfs->max_pw_shrt_sg1[j])
						bb_dfs->max_pw_shrt_sg1[j] = pw;
					if (pw < bb_dfs->min_pw_shrt_sg1[j])
						bb_dfs->min_pw_shrt_sg1[j] = pw;
					if (!(dfs_rpt_idx == 0) && (bb_dfs->rpt_sg_history & BIT(is_sg1))) {
						if (pri > bb_dfs->max_pri_shrt_sg1[j])
							bb_dfs->max_pri_shrt_sg1[j] = pri;
						if (pri < bb_dfs->min_pri_shrt_sg1[j])
							bb_dfs->min_pri_shrt_sg1[j] = pri;
					}
				}
				else {
					bb_dfs->srt_rdr_cnt[j]++;
					if (pw > bb_dfs->max_pw_shrt[j])
						bb_dfs->max_pw_shrt[j] = pw;
					if (pw < bb_dfs->min_pw_shrt[j])
						bb_dfs->min_pw_shrt[j] = pw;
					if (!(dfs_rpt_idx == 0) && (bb_dfs->rpt_sg_history & BIT(is_sg1))) {
						if (pri > bb_dfs->max_pri_shrt[j])
							bb_dfs->max_pri_shrt[j] = pri;
						if (pri < bb_dfs->min_pri_shrt[j])
							bb_dfs->min_pri_shrt[j] = pri;
					}
				}
			}
		}
		if (dfs_rpt_idx == 0) {
			BB_DFS_DBG(bb, HWDET_PRINT, "Type %d: [pw_lbd-pw_ubd], [pri_lbd-pri_ubd], [ppb_thd] = [%d-%d], [%d-%d], [%d]\n",
			(j + 1), pw_lbd, pw_ubd, pri_lbd, pri_ubd, bb_dfs->ppb_typ_th[j]);
		}
		bb_dfs->pw_lbd[j] = pw_lbd;
		bb_dfs->pw_ubd[j] = pw_ubd;
		bb_dfs->pri_lbd[j] = pri_lbd;
		bb_dfs->pri_ubd[j] = pri_ubd;
	}
}

void halbb_radar_info_processing(struct bb_info *bb,
				 struct hal_dfs_rpt *rpt, u16 dfs_rpt_idx)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	struct bb_rdr_info_gen0 *dfs_rdr_info_gen0 = (struct bb_rdr_info_gen0 *)rpt->dfs_ptr;
	struct bb_rdr_info_gen1 *dfs_rdr_info_gen1 = (struct bb_rdr_info_gen1 *)rpt->dfs_ptr;
	struct bb_dfs_cr_info *cr = &bb->bb_cmn_hooker->bb_dfs_cr_i;

	u8 i, pri = 0, cur_seq_num = 0, pre_seq_num = 0;
	u8 cur_seg_idx = 0, pre_seg_idx = 0;// For TW DFS
	u16 pw = 0;
	bool chrp_flag = false;
	bool is_sg1 = false;

#ifdef HALBB_TW_DFS_SERIES
	if (bb->ic_sub_type != BB_IC_SUB_TYPE_8852C_8852D) {
		if (bb_dfs->is_tw_en) {
			pw = (dfs_rdr_info_gen1->rdr_info_sg0_pw_m << 4) | (dfs_rdr_info_gen1->rdr_info_sg0_pw_l);
			//Seg0 of TW DFS
			if (pw != 0) {
				cur_seq_num = dfs_rdr_info_gen1->rdr_info_sg0_seq;
				pre_seq_num = bb_dfs->lst_seq_num;
				cur_seg_idx = 0;
				pre_seg_idx = bb_dfs->lst_seg_idx;
				pri = (dfs_rdr_info_gen1->rdr_info_sg0_pri_m << 4) | (dfs_rdr_info_gen1->rdr_info_sg0_pri_l);
				chrp_flag = dfs_rdr_info_gen1->rdr_info_sg0_chirp_flag;
				is_sg1 = false;
			}
			//Seg1 of TW DFS
			else {
				cur_seq_num = dfs_rdr_info_gen1->rdr_info_sg1_seq;
				pre_seq_num = bb_dfs->lst_seq_num_sg1;
				cur_seg_idx = 1;
				pre_seg_idx = bb_dfs->lst_seg_idx;
				pw = (dfs_rdr_info_gen1->rdr_info_sg1_pw_m << 7) | (dfs_rdr_info_gen1->rdr_info_sg1_pw_l);
				pri = (dfs_rdr_info_gen1->rdr_info_sg1_pri_m << 7) | (dfs_rdr_info_gen1->rdr_info_sg1_pri_l);
				chrp_flag = dfs_rdr_info_gen1->rdr_info_sg1_chirp_flag;
				is_sg1 = true;
			}
		}
		else {
			cur_seq_num = dfs_rdr_info_gen1->rdr_info_sg0_seq;
			pre_seq_num = bb_dfs->lst_seq_num;
			pw = (dfs_rdr_info_gen1->rdr_info_sg0_pw_m << 4) | (dfs_rdr_info_gen1->rdr_info_sg0_pw_l);
			pri = (dfs_rdr_info_gen1->rdr_info_sg0_pri_m << 4) | (dfs_rdr_info_gen1->rdr_info_sg0_pri_l);
			chrp_flag = dfs_rdr_info_gen1->rdr_info_sg0_chirp_flag;
			is_sg1 = false;
		}
	}
	else
#endif
	{
		cur_seq_num = dfs_rdr_info_gen0->rdr_info_seq;
		pre_seq_num = bb_dfs->lst_seq_num;
		cur_seg_idx = 0;
		pre_seg_idx = 0;
		if (rpt->phy_idx == HW_PHY_0) {
			pw = (dfs_rdr_info_gen0->rdr_info_sg0_pw_m << 7) |
			     (dfs_rdr_info_gen0->rdr_info_sg0_pw_l);
			pri = (dfs_rdr_info_gen0->rdr_info_sg0_pri_m << 7) |
			      (dfs_rdr_info_gen0->rdr_info_sg0_pri_l);
			chrp_flag = dfs_rdr_info_gen0->rdr_info_sg0_chirp_flag;
		} else if (rpt->phy_idx == HW_PHY_1) {
			pw = (dfs_rdr_info_gen0->rdr_info_sg1_pw_m << 4) |
			     (dfs_rdr_info_gen0->rdr_info_sg1_pw_l);
			pri = (dfs_rdr_info_gen0->rdr_info_sg1_pri_m << 4) |
			      (dfs_rdr_info_gen0->rdr_info_sg1_pri_l);
			chrp_flag = dfs_rdr_info_gen0->rdr_info_sg1_chirp_flag;
		}
	}

	rpt->dfs_ptr += DFS_RPT_LENGTH;
	bb_dfs->seg_rpt_all[bb_dfs->rpt_rdr_cnt] = is_sg1;
	bb_dfs->seq_num_rpt_all[bb_dfs->rpt_rdr_cnt] = cur_seq_num;

	halbb_radar_seq_inspctn(bb, dfs_rpt_idx, cur_seq_num, pre_seq_num, is_sg1, cur_seg_idx, pre_seg_idx);

#ifdef HALBB_TW_DFS_SERIES   // 160M ch36~64 seg0 report is non real radar
	if (bb->ic_sub_type != BB_IC_SUB_TYPE_8852C_8852D) {
		if (bb_dfs->is_tw_en && bb_dfs->bypass_seg0) {
			if (!is_sg1)
				return;
		}
	}
#endif

	/* mask pri < th dfs report because it may generate lots of rpts*/
	if (pri >= bb_dfs->pri_mask_th) {
		bb_dfs->pw_rpt[bb_dfs->rpt_rdr_cnt] = pw;
		bb_dfs->pri_rpt[bb_dfs->rpt_rdr_cnt] = pri;
		bb_dfs->chrp_rpt[bb_dfs->rpt_rdr_cnt] = chrp_flag;
		bb_dfs->seg_rpt[bb_dfs->rpt_rdr_cnt] = is_sg1;
		bb_dfs->seq_num_rpt[bb_dfs->rpt_rdr_cnt] = cur_seq_num;
		bb_dfs->rpt_rdr_cnt ++;
	}
}

void halbb_dfs_dyn_setting(struct bb_info *bb)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	struct bb_link_info *link = &bb->bb_link_i;
	struct bb_env_mntr_info *env_mntr = &bb->bb_env_mntr_i;
	struct bb_dfs_cr_info *cr = &bb->bb_cmn_hooker->bb_dfs_cr_i;
	struct bb_stat_info *stat = &bb->bb_stat_i;
	struct bb_fa_info *fa = &stat->bb_fa_i;
	u8 bw = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.bw;
	u8 chan = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.chan;
	u8 i;

	if (bb_dfs->first_dyn_set_flag) {
		bb_dfs->first_dyn_set_flag = false;
		bb_dfs->detect_state = DFS_Adaptive_State;
		bb_dfs->adap_detect_cnt = 1;  // mask first 2s when init
		BB_DFS_DBG(bb, DYN_PRINT, "First dyn setting !\n");
		BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Adaptive State : [%d]\n", bb_dfs->adap_detect_cnt);
		return;
	}
	// first set dbgmode reset cnt
	if (bb_dfs->dfs_dbg_mode && bb_dfs->dyn_reset_flag) {
		bb_dfs->adap_detect_cnt = 0;
		bb_dfs->adap_detect_cnt_all = 0;
		bb_dfs->detect_state = DFS_Normal_State;
		bb_dfs->dyn_reset_flag = false;
	}

	/*if (bb_dfs->dfs_dbg_mode)
		bb_dfs->adap_detect_brk_en = false;
	else
		bb_dfs->adap_detect_brk_en = true; */

#ifdef HALBB_TW_DFS_SERIES
	if (bb->ic_sub_type != BB_IC_SUB_TYPE_8852C_8852D) {
		switch (bb->hal_com->band[bb->bb_phy_idx].cur_chandef.bw) {
		case CHANNEL_WIDTH_20:
		case CHANNEL_WIDTH_40:
			bb_dfs->is_tw_en = false;
			bb_dfs->bypass_seg0 = false;
			break;
		case CHANNEL_WIDTH_80:
			bb_dfs->is_tw_en = (bool)halbb_get_reg_cmn(bb, cr->tw_dfs_en, cr->tw_dfs_en_m, bb->bb_phy_idx);
			bb_dfs->bypass_seg0 = false;
			break;
		case CHANNEL_WIDTH_160:
		case CHANNEL_WIDTH_80_80:
			bb_dfs->is_tw_en = true;
			if (!(bb_dfs->dfs_rgn_domain == DFS_REGD_FCC) && (chan >= 36) && (chan <=64))
				bb_dfs->bypass_seg0 = true;
			else
				bb_dfs->bypass_seg0 = false;
			break;
		default:
			bb_dfs->is_tw_en = false;
			bb_dfs->bypass_seg0 = false;
			break;
		}
	}
#endif

	if (link->total_tp < bb_dfs->dfs_tp_th) {
		bb_dfs->idle_flag = true;
		bb_dfs->pw_diff_th = 12;
		bb_dfs->pw_lng_chrp_diff_th = 18;
		bb_dfs->pri_diff_th = 5;
		bb_dfs->pw_factor = PW_FTR_IDLE;
		bb_dfs->pri_factor = PRI_FTR_IDLE;
		bb_dfs->ppb_prcnt = DFS_PPB_IDLE_PRCNT;
		for (i = 0; i < DFS_RDR_TYP_NUM; i++) {
			bb_dfs->ppb_typ_th[i] = bb_dfs->ppb_tab[i];
		}
	}
	else {
		bb_dfs->idle_flag = false;
		bb_dfs->pw_diff_th = 20;
		bb_dfs->pw_lng_chrp_diff_th = 20;
		bb_dfs->pri_diff_th = 170;
		bb_dfs->pw_factor = PW_FTR;
		bb_dfs->pri_factor = PRI_FTR;
		bb_dfs->ppb_prcnt = DFS_PPB_PRCNT;
		for (i = 0; i < DFS_RDR_TYP_NUM; i++) {
			bb_dfs->ppb_typ_th[i] = bb_dfs->ppb_tp_tab[i];
		}
	}

	/* Supressing false detection */
	switch (bb_dfs->detect_state) {
	case DFS_Normal_State:
		if ((env_mntr->env_mntr_rpt_bg.nhm_idle_ratio < bb_dfs->dfs_idle_prd_th && env_mntr->env_mntr_rpt_bg.nhm_idle_ratio != ENV_MNTR_FAIL_BYTE) ||
			 (env_mntr->env_mntr_rpt_bg.nhm_ratio > bb_dfs->dfs_nhm_th && env_mntr->env_mntr_rpt_bg.nhm_ratio != ENV_MNTR_FAIL_BYTE) ||
			 (env_mntr->env_mntr_rpt_bg.nhm_cca_ratio > bb_dfs->dfs_rx_rto_th && env_mntr->env_mntr_rpt_bg.nhm_cca_ratio != ENV_MNTR_FAIL_BYTE) ||
			 (fa->cnt_fail_all > bb_dfs->dfs_fa_th) || (link->total_tp == 0 && env_mntr->env_mntr_rpt_bg.nhm_idle_ratio <= 96)) {
			bb_dfs->adap_detect_cnt = bb_dfs->adap_detect_cnt_init;
			bb_dfs->adap_detect_cnt_all += 1;
			bb_dfs->detect_state = DFS_Adaptive_State;
			BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] NHM/FA triggers Adaptive State\n");
			BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Adaptive State : [%d]\n", bb_dfs->adap_detect_cnt);
		}
		else {
			BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Normal State\n");
		}
		BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Adap_detect_cnt_all = [%d]\n", bb_dfs->adap_detect_cnt_all);

		break;

	case DFS_Adaptive_State:
		bb_dfs->adap_detect_cnt = SUBTRACT_TO_0(bb_dfs->adap_detect_cnt,1);
		if ((env_mntr->env_mntr_rpt_bg.nhm_idle_ratio < bb_dfs->dfs_idle_prd_th && env_mntr->env_mntr_rpt_bg.nhm_idle_ratio != ENV_MNTR_FAIL_BYTE) ||
			 (env_mntr->env_mntr_rpt_bg.nhm_ratio > bb_dfs->dfs_nhm_th && env_mntr->env_mntr_rpt_bg.nhm_ratio != ENV_MNTR_FAIL_BYTE) ||
			 (env_mntr->env_mntr_rpt_bg.nhm_cca_ratio > bb_dfs->dfs_rx_rto_th && env_mntr->env_mntr_rpt_bg.nhm_cca_ratio != ENV_MNTR_FAIL_BYTE) ||
			 (fa->cnt_fail_all > bb_dfs->dfs_fa_th) || (link->total_tp == 0 && env_mntr->env_mntr_rpt_bg.nhm_idle_ratio <= 96)) {
			//bb_dfs->adap_detect_cnt = MIN_2(255, bb_dfs->adap_detect_cnt + bb_dfs->adap_detect_cnt_add);
			bb_dfs->adap_detect_cnt = bb_dfs->adap_detect_cnt_add;
			bb_dfs->adap_detect_cnt_all = MIN_2(255, bb_dfs->adap_detect_cnt_all+1);
			BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] NHM/FA triggers Adaptive State again\n");
		}
		if (bb_dfs->adap_detect_cnt == 0) {
			bb_dfs->detect_state = DFS_Normal_State;
			BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Normal State\n");
		}
		else {
			BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Adaptive State : [%d]\n", bb_dfs->adap_detect_cnt);
		}
		BB_DFS_DBG(bb, DYN_PRINT, "[DFS Status] Adap_detect_cnt_all = [%d]\n", bb_dfs->adap_detect_cnt_all);

		break;

	default:
		bb_dfs->detect_state = DFS_Normal_State;
		bb_dfs->adap_detect_cnt = 0;

		break;
	}
}

void halbb_dfs_debug(struct bb_info *bb, char input[][16], u32 *_used,
			 char *output, u32 *_out_len)
{
	struct bb_dfs_info *bb_dfs = &bb->bb_dfs_i;
	u8 bw = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.bw;
	u8 chan = bb->hal_com->band[bb->bb_phy_idx].cur_chandef.chan;
	char help[] = "-h";
	u32 var[10] = {0};
	u32 used = *_used;
	u32 out_len = *_out_len;
	u8 i = 0;

	HALBB_SCAN(input[1], DCMD_DECIMAL, &var[0]);

	if ((_os_strcmp(input[1], help) == 0)) {
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "{1} Set DFS_SW_TRGR_MODE => \n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{1} Set Normal Path => {0}: Disable, {1}: Enable\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{2} Set Extra Path => {0}: Disable, {1}: Enable\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "{2} Set DFS_DBG_MODE => {0}: Disable, {1}: Enable\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "{3} Set PRNT LEVEL => \n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{1} Set DBG_HWDET_PRINT => {0}: Disable, {1}: Enable\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{2} Set DBG_SWDET_PRINT => {0}: Disable, {1}: Enable\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{3} Set DBG_BRK_PRINT => {0}: Disable, {1}: Enable\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{4} Set DBG_DYN_PRINT => {0}: Disable, {1}: Enable\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{5} Set DBG_TRIVIL_PRINT => {0}: Disable, {1}: Enable\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "{4} Set DYN_SETTING_EN => {0}: Disable, {1}: Enable\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "{5} Set Adap_detect_brk_EN => {0}: Disable, {1}: Enable\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "{6} Set Detection Parameter => \n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{1} Set the threshold of fake DFS number => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{2} Set the threshold of DFS_TP Threshold => {Mbps}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{3} Set the threshold of DFS_Idle_Period Threshold => {Percent: 0-100}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{4} Set the threshold of DFS_FA Threshold => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{5} Set the threshold of DFS_NHM Threshold => {Percent: 0-100}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{6} Reset aci_disable_detect_cnt\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{7} Set the threshold of pw_diff_th => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{8} Set the threshold of pri_diff_th => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{9} Set the threshold of adap_detect_cnt_init => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{10} Set the threshold of adap_detect_cnt_add => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{11} Set the threshold of adap_detect_cnt_th => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{12} Set the threshold of pw_max_th => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{13} Set the threshold of pw_lng_chrp_th => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{14} Set the threshold of invalid_lng_pulse_h_th => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{15} Set the threshold of invalid_lng_pulse_l_th => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{16} Set the threshold of pri_mask_th => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{17} Set bypass_seg0_en => {0}: Disable, {1}: Enable\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{18} Set mask_fake_rpt_en => {0}: Disable, {1}: Enable\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{19} Set adv_pri_ofst => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{20} Set the threshold of DFS_RX_RTO Threshold => {Percent: 0-100}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{21} Set the threshold of dfs_rssi_th => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{22} Set the threshold of fake_rdr_cnt_th => {Num}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "{7} Set Idle mode pw/pri/ppb thd => \n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{1} Set pw[i+1] thd=> {i} {LB} {UB}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{2} Set pri[i+1] thd => {i} {LB} {UB}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{3} Set ppb[i+1] thd => {i} {PPB}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "{8} Set TP mode ppb thd => \n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "---{3} Set ppb[i+1] thd => {i} {PPB}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "{set_l2h_init} => {hex}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "{set_l2h_val} => {hex}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "{100} Show all parameter\n");
	} else if (var[0] == 100) {
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DFS Region Domain: %s\n",
			   (bb_dfs->dfs_rgn_domain > 1) ?
			   (bb_dfs->dfs_rgn_domain > 2) ?
			   "ETSI": "MIC" : "FCC");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "BW = %d, CH = %d\n",
			    bw,chan);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "Is_Idle =  %d\n",
			    bb_dfs->idle_flag);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DFS_SW_TRGR_MODE = %d\n",
			    bb_dfs->dfs_sw_trgr_mode);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DFS_SW_TRGR_MODE_Extra = %d\n",
			    bb_dfs->dfs_sw_trgr_mode_extra);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DFS_DBG_MODE = %d\n",
			    bb_dfs->dfs_dbg_mode);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DBG_HWDET_PRINT = %d\n",
			    (bool)(bb_dfs->dbg_print_component & HWDET_PRINT));
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DBG_SWDET_PRINT = %d\n",
			    (bool)(bb_dfs->dbg_print_component & SWDET_PRINT));
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DBG_BRK_PRINT = %d\n",
			    (bool)(bb_dfs->dbg_print_component & BRK_PRINT));
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DBG_DYN_PRINT = %d\n",
			    (bool)(bb_dfs->dbg_print_component & DYN_PRINT));
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DBG_TRIVIL_PRINT = %d\n",
			    (bool)(bb_dfs->dbg_print_component & TRIVIL_PRINT));
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DYN_SETTING_EN = %d\n",
			    bb_dfs->dfs_dyn_setting_en);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "Adap_detect_brk_EN = %d\n",
			    bb_dfs->adap_detect_brk_en);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "Fake DFS Num Threshold = %d\n",
			    bb_dfs->fk_dfs_num_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DFS_TP Threshold = %d\n",
			    bb_dfs->dfs_tp_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DFS_Idle_Period Threshold = %d\n",
			    bb_dfs->dfs_idle_prd_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DFS_FA Threshold = %d\n",
			    bb_dfs->dfs_fa_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DFS_NHM Threshold = %d\n",
			    bb_dfs->dfs_nhm_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "DFS_RX_RTO Threshold = %d\n",
			    bb_dfs->dfs_rx_rto_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "pw_diff_th = %d\n",
			    bb_dfs->pw_diff_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "pri_diff_th = %d\n",
				bb_dfs->pri_diff_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "adap_detect_cnt_init = %d\n",
				bb_dfs->adap_detect_cnt_init);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "adap_detect_cnt_add = %d\n",
				bb_dfs->adap_detect_cnt_add);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "adap_detect_cnt_th = %d\n",
				bb_dfs->adap_detect_cnt_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "pw_max_th = %d\n",
				bb_dfs->pw_max_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "pw_lng_chrp_diff_th = %d\n",
				bb_dfs->pw_lng_chrp_diff_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "invalid_lng_pulse_h_th = %d\n",
				bb_dfs->invalid_lng_pulse_h_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "invalid_lng_pulse_l_th = %d\n",
				bb_dfs->invalid_lng_pulse_l_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "pri_mask_th = %d\n",
				bb_dfs->pri_mask_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "bypass_seg0 = %d\n",
				bb_dfs->bypass_seg0);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "mask_fake_rpt_en = %d\n",
				bb_dfs->mask_fake_rpt_en);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "adv_pri_ofst = %d\n",
				bb_dfs->adv_pri_ofst);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "dfs_rssi_th = %d\n",
				bb_dfs->dfs_rssi_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "fake_rdr_cnt_th = %d\n",
				bb_dfs->fake_rdr_cnt_th);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "l2h_init_value = 0x%x\n",
				bb_dfs->l2h_init_val);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "l2h_value = 0x%x\n",
				bb_dfs->l2h_val);
		BB_DBG_CNSL(out_len, used, output + used, out_len - used, "Radar TH : \n");
		for (i = 0; i < DFS_RDR_TYP_NUM ; i++) {
			BB_DBG_CNSL(out_len, used, output + used, out_len - used, "Type%d : pw = [%d-%d], pri = [%d-%d], ppb_th = %d\n",
				i+1,bb_dfs->pw_lbd[i],bb_dfs->pw_ubd[i],
				bb_dfs->pri_lbd[i],bb_dfs->pri_ubd[i],bb_dfs->ppb_typ_th[i]);
		}
	} else {
		if (var[0] == 1) {
			HALBB_SCAN(input[2], DCMD_DECIMAL, &var[1]);
			HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
			if (var[1] == 1) {
				bb_dfs->dfs_sw_trgr_mode = (bool)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "DFS_SW_TRGR_MODE = %d\n",
					 bb_dfs->dfs_sw_trgr_mode);
			} else if (var[1] == 2) {
				bb_dfs->dfs_sw_trgr_mode_extra = (bool)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "DFS_SW_TRGR_MODE_Extra = %d\n",
					bb_dfs->dfs_sw_trgr_mode_extra);
			}
		} else if (var[0] == 2) {
			HALBB_SCAN(input[2], DCMD_DECIMAL, &var[1]);
			bb_dfs->dfs_dbg_mode = (bool)var[1];
			BB_DBG_CNSL(out_len, used, output + used,
				    out_len - used, "DFS_DBG_MODE = %d\n",
				    bb_dfs->dfs_dbg_mode);
		} else if (var[0] == 3) {
			HALBB_SCAN(input[2], DCMD_DECIMAL, &var[1]);
			if (var[1] == 1) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->dbg_print_component = 
					(var[2] == true ? bb_dfs->dbg_print_component | HWDET_PRINT : bb_dfs->dbg_print_component & ~HWDET_PRINT);
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "DBG_HWDET_PRINT = %d\n",
					    (bool)(bb_dfs->dbg_print_component & HWDET_PRINT));
			} else if (var[1] == 2) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->dbg_print_component = 
					(var[2] == true ? bb_dfs->dbg_print_component | SWDET_PRINT : bb_dfs->dbg_print_component & ~SWDET_PRINT);
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "DBG_SWDET_PRINT = %d\n",
					   (bool)(bb_dfs->dbg_print_component & SWDET_PRINT));
			} else if (var[1] == 3) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->dbg_print_component = 
					(var[2] == true ? bb_dfs->dbg_print_component | BRK_PRINT : bb_dfs->dbg_print_component & ~BRK_PRINT);
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "DBG_BRK_PRINT = %d\n",
					    (bool)(bb_dfs->dbg_print_component & BRK_PRINT));
			} else if (var[1] == 4) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->dbg_print_component = 
					(var[2] == true ? bb_dfs->dbg_print_component | DYN_PRINT : bb_dfs->dbg_print_component & ~DYN_PRINT);
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "DBG_DYN_PRINT = %d\n",
					    (bool)(bb_dfs->dbg_print_component & DYN_PRINT));
			} else if (var[1] == 5) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->dbg_print_component = 
					(var[2] == true ? bb_dfs->dbg_print_component | TRIVIL_PRINT : bb_dfs->dbg_print_component & ~TRIVIL_PRINT);
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "DBG_TRIVIL_PRINT = %d\n",
					    (bool)(bb_dfs->dbg_print_component & TRIVIL_PRINT));
			}
		} else if (var[0] == 4) {
			HALBB_SCAN(input[2], DCMD_DECIMAL, &var[1]);
			bb_dfs->dfs_dyn_setting_en = (bool)var[1];
			BB_DBG_CNSL(out_len, used, output + used,
				    out_len - used, "DYN_SETTING_EN = %d\n",
				    bb_dfs->dfs_dyn_setting_en);
			if (bb_dfs->dfs_dyn_setting_en == false) {
				bb_dfs->adap_detect_cnt = 0;
				bb_dfs->adap_detect_cnt_all =0;
				bb_dfs->detect_state = DFS_Normal_State;
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "reset aci_disable_detect_cnt\n");
			}
		} else if (var[0] == 5) {
			HALBB_SCAN(input[2], DCMD_DECIMAL, &var[1]);
			bb_dfs->adap_detect_brk_en = (bool)var[1];
			BB_DBG_CNSL(out_len, used, output + used,
				    out_len - used, "Adap_detect_brk_EN = %d\n",
				    bb_dfs->adap_detect_brk_en);
		} else if (var[0] == 6) {
			HALBB_SCAN(input[2], DCMD_DECIMAL, &var[1]);
			if (var[1] == 1) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->fk_dfs_num_th = (u8)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "fk_dfs_num_th = %d\n",
					    bb_dfs->fk_dfs_num_th);
			} else if (var[1] == 2) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->dfs_tp_th = (u8)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "dfs_tp_th = %d\n",
					    bb_dfs->dfs_tp_th);
			} else if (var[1] == 3) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->dfs_idle_prd_th = (u8)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "dfs_idle_prd_th = %d\n",
					    bb_dfs->dfs_idle_prd_th);
			} else if (var[1] == 4) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->dfs_fa_th= (u16)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "dfs_fa_th = %d\n",
					    bb_dfs->dfs_fa_th);
			} else if (var[1] == 5) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->dfs_nhm_th= (u8)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "dfs_nhm_th = %d\n",
					    bb_dfs->dfs_nhm_th);
			} else if (var[1] == 6) {
				bb_dfs->adap_detect_cnt = 0;
				bb_dfs->adap_detect_cnt_all =0;
				bb_dfs->detect_state = DFS_Normal_State;
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "reset aci_disable_detect_cnt\n");
			} else if (var[1] == 7) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->pw_diff_th= (u16)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "pw_diff_th = %d\n",
					    bb_dfs->pw_diff_th);
			} else if (var[1] == 8) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->pri_diff_th = (u16)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "pri_diff_th = %d\n",
					bb_dfs->pri_diff_th);
			} else if (var[1] == 9) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->adap_detect_cnt_init= (u8)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "adap_detct_cnt_init = %d\n",
					bb_dfs->adap_detect_cnt_init);
			} else if (var[1] == 10) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->adap_detect_cnt_add = (u8)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "adap_detct_cnt_add = %d\n",
					bb_dfs->adap_detect_cnt_add);
			} else if (var[1] == 11) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->adap_detect_cnt_th= (u8)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "adap_detct_cnt_th = %d\n",
					bb_dfs->adap_detect_cnt_th);
			} else if (var[1] == 12) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->pw_max_th = (u16)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "pw_max_th = %d\n",
					bb_dfs->pw_max_th);
			} else if (var[1] == 13) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->pw_lng_chrp_diff_th = (u16)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "pw_lng_chrp_diff_th = %d\n",
					bb_dfs->pw_lng_chrp_diff_th);
			} else if (var[1] == 14) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->invalid_lng_pulse_h_th = (u16)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "invalid_lng_pulse_h_th = %d\n",
					bb_dfs->invalid_lng_pulse_h_th);
			} else if (var[1] == 15) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->invalid_lng_pulse_l_th = (u16)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "invalid_lng_pulse_l_th = %d\n",
					bb_dfs->invalid_lng_pulse_l_th);
			} else if (var[1] == 16) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->pri_mask_th= (u8)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "pri_mask_th = %d\n",
					bb_dfs->pri_mask_th);
			} else if (var[1] == 17) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->bypass_seg0= (bool)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "bypass_seg0 = %d\n",
					bb_dfs->bypass_seg0);
			} else if (var[1] == 18) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->mask_fake_rpt_en = (bool)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "mask_fake_rpt_en = %d\n",
					bb_dfs->mask_fake_rpt_en);
			} else if (var[1] == 19) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->adv_pri_ofst = (u16)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					out_len - used, "adv_pri_ofst = %d\n",
					bb_dfs->adv_pri_ofst);
			} else if (var[1] == 20) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->dfs_rx_rto_th= (u8)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "dfs_rx_rto_th = %d\n",
					    bb_dfs->dfs_rx_rto_th);
			} else if (var[1] == 21) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				bb_dfs->dfs_rssi_th= (u8)var[2];
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "dfs_rssi_th = %d\n",
					    bb_dfs->dfs_rssi_th);
			} else if (var[1] == 22) {
			HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
			bb_dfs->fake_rdr_cnt_th = (u8)var[2];
			BB_DBG_CNSL(out_len, used, output + used,
				out_len - used, "fake_rdr_cnt_th = %d\n",
				bb_dfs->fake_rdr_cnt_th);
			}
		} else if (var[0] == 7) {
			HALBB_SCAN(input[2], DCMD_DECIMAL, &var[1]);
			if (var[1] == 1) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				HALBB_SCAN(input[4], DCMD_DECIMAL, &var[3]);
				HALBB_SCAN(input[5], DCMD_DECIMAL, &var[4]);
				i = (u8)var[2];
				bb_dfs->pw_min_tab[i]= (u16)var[3];
				bb_dfs->pw_max_tab[i]= (u16)var[4];
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "Type%d: pw = [%d-%d]\n",
					    i+1,bb_dfs->pw_min_tab[i],bb_dfs->pw_max_tab[i]);
			} else if (var[1] == 2) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				HALBB_SCAN(input[4], DCMD_DECIMAL, &var[3]);
				HALBB_SCAN(input[5], DCMD_DECIMAL, &var[4]);
				i = (u8)var[2];
				bb_dfs->pri_min_tab[i]= (u16)var[3];
				bb_dfs->pri_max_tab[i]= (u16)var[4];
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "Type%d: pri = [%d-%d]\n",
					    i+1,bb_dfs->pri_min_tab[i],bb_dfs->pri_max_tab[i]);
			} else if (var[1] == 3) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				HALBB_SCAN(input[4], DCMD_DECIMAL, &var[3]);
				i = (u8)var[2];
				bb_dfs->ppb_tab[i]= (u8)var[3];
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "Type%d: Idle ppb = %d\n",
					    i+1,bb_dfs->ppb_tab[i]);
			}
		} else if (var[0] == 8) {
			HALBB_SCAN(input[2], DCMD_DECIMAL, &var[1]);
			if (var[1] == 3) {
				HALBB_SCAN(input[3], DCMD_DECIMAL, &var[2]);
				HALBB_SCAN(input[4], DCMD_DECIMAL, &var[3]);
				i = (u8)var[2];
				bb_dfs->ppb_tp_tab[i]= (u8)var[3];
				BB_DBG_CNSL(out_len, used, output + used,
					    out_len - used, "Type%d: TP ppb = %d\n",
					    i+1,bb_dfs->ppb_tp_tab[i]);
			}
		}  else if (_os_strcmp(input[1], "set_l2h_init") == 0) {
			HALBB_SCAN(input[2], DCMD_HEX, &var[0]);
			bb_dfs->l2h_init_val = (u32)var[0];
			BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				    "l2h_init_val = 0x%x\n", bb_dfs->l2h_init_val);
		} else if (_os_strcmp(input[1], "set_l2h_val") == 0) {
			HALBB_SCAN(input[2], DCMD_HEX, &var[0]);
			bb_dfs->l2h_val = (u32)var[0];
			BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				    "l2h_val = 0x%x\n", bb_dfs->l2h_val);
		} 
	}

	*_used = used;
	*_out_len = out_len;
}

void halbb_cr_cfg_dfs_init(struct bb_info *bb)
{
	struct bb_dfs_cr_info *cr = &bb->bb_cmn_hooker->bb_dfs_cr_i;

	switch (bb->cr_type) {

#ifdef HALBB_COMPILE_AP_SERIES
	case BB_AP:
		cr->dfs_en = DFS_EN_A;
		cr->dfs_en_m = DFS_EN_A_M;
		break;
#endif

#ifdef HALBB_COMPILE_AP2_SERIES
	case BB_AP2:
		cr->dfs_en = DFS_EN_A2;
		cr->dfs_en_m = DFS_EN_A2_M;
		cr->dfs_l2h_th = SEG0R_L2H_TH_A2;
		cr->dfs_l2h_th_m = SEG0R_L2H_TH_A2_M;
		cr->tw_dfs_en = SEG0R_TW_DFS_EN_A2;
		cr->tw_dfs_en_m = SEG0R_TW_DFS_EN_A2_M;
		cr->dfs_en_p1 = DFS_PATH1_EN_A2;
		cr->dfs_en_p1_m = DFS_PATH1_EN_A2_M;
		break;

#endif

#ifdef HALBB_COMPILE_CLIENT_SERIES
	case BB_CLIENT:
		cr->dfs_en = DFS_EN_C;
		cr->dfs_en_m = DFS_EN_C_M;
		break;
#endif
#ifdef HALBB_COMPILE_BE0_SERIES
	case BB_BE0:
		cr->dfs_en = DFS_EN_BE0;
		cr->dfs_en_m = DFS_EN_BE0_M;
		cr->dfs_l2h_th = SEG0R_L2H_TH_BE0;
		cr->dfs_l2h_th_m = SEG0R_L2H_TH_BE0_M;
		break;
#endif
#ifdef HALBB_COMPILE_BE1_SERIES
	case BB_BE1:
		cr->dfs_l2h_th = SEG0R_L2H_TH_BE1;
		cr->dfs_l2h_th_m = SEG0R_L2H_TH_BE1_M;
		break;
#endif
	default:
		BB_WARNING("[%s] BBCR Hook FAIL!\n", __func__);
		if (bb->bb_dbg_i.cr_fake_init_hook_en) {
			BB_TRACE("[%s] BBCR fake init\n", __func__);
			halbb_cr_hook_fake_init(bb, (u32 *)cr, (sizeof(struct bb_dfs_cr_info) >> 2));
		}
		break;
	}

	if (bb->bb_dbg_i.cr_init_hook_recorder_en) {
		BB_TRACE("[%s] BBCR Hook dump\n", __func__);
		halbb_cr_hook_init_dump(bb, (u32 *)cr, (sizeof(struct bb_dfs_cr_info) >> 2));
	}
}
#endif
