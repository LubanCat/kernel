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

void halbb_mp_bt_cfg(struct bb_info *bb, bool bt_connect)
{
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;
	u8 lna_val = 0, lna_ori = 0, lna_gain_ofst = 0;

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	if (bb->ic_type == BB_RTL8852A) {
		BB_DBG(bb, DBG_PHY_CONFIG, "IC not support !!\n");
		return;
	}

	if (bb->ic_type == BB_RTL8852C)
		lna_gain_ofst = 12;
	else if (bb->ic_sub_type == BB_IC_SUB_TYPE_8852B_8852BP)
		lna_gain_ofst = 8;

	lna_ori = (u8)halbb_get_reg(bb, cr->path1_g_lna6, cr->path1_g_lna6_m);

	if (bt_connect) {
		lna_val = (lna_ori <= lna_gain_ofst) ? 0 : lna_ori - lna_gain_ofst;
		halbb_set_reg(bb, cr->path1_g_lna6, cr->path1_g_lna6_m, lna_val);
	} else {
		lna_val = ((lna_ori + lna_gain_ofst) >= 0xff) ? 0xff : lna_ori + lna_gain_ofst;
		halbb_set_reg(bb, cr->path1_g_lna6, cr->path1_g_lna6_m, lna_val);
	}

	BB_DBG(bb, DBG_PHY_CONFIG, "BT connect:%d, lna_val:0x%x\n", bt_connect, lna_val);
}

u16 halbb_mp_get_tx_ok(struct bb_info *bb, u32 rate_index,
			enum phl_phy_idx phy_idx)
{
	u32 tx_ok;

	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;

	if (halbb_is_cck_rate(bb, (u16)rate_index))
		tx_ok = halbb_get_reg(bb, cr->cnt_ccktxon, cr->cnt_ccktxon_m);
	else
		tx_ok = halbb_get_reg_cmn(bb, cr->cnt_ofdmtxon, cr->cnt_ofdmtxon_m, phy_idx);
	return (u16)tx_ok;
}

u32 halbb_rx_crc_ok_6(struct bb_info *bb, enum phl_phy_idx phy_idx)
{
	u32 cck_ok = 0, ofdm_ok = 0, ht_ok = 0, vht_ok = 0, he_ok = 0;
	u32 crc_ok;

	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;

	BB_DBG(bb, DBG_PHY_CONFIG, "[%s] phy_idx=%d\n", __func__, phy_idx);

	if (phy_idx == HW_PHY_0)
		cck_ok = halbb_get_reg(bb, cr->cnt_cck_crc32ok_p0, cr->cnt_cck_crc32ok_p0_m);
	else
		cck_ok = halbb_get_reg(bb, cr->cnt_cck_crc32ok_p1, cr->cnt_cck_crc32ok_p1_m);

	ofdm_ok = halbb_get_reg_cmn(bb, cr->cnt_l_crc_ok, cr->cnt_l_crc_ok_m, phy_idx);
	ht_ok = halbb_get_reg_cmn(bb, cr->cnt_ht_crc_ok, cr->cnt_ht_crc_ok_m, phy_idx);
	vht_ok = halbb_get_reg_cmn(bb, cr->cnt_vht_crc_ok, cr->cnt_vht_crc_ok_m, phy_idx);
	he_ok = halbb_get_reg_cmn(bb, cr->cnt_he_crc_ok, cr->cnt_he_crc_ok_m, phy_idx);

	crc_ok = cck_ok + ofdm_ok + ht_ok + vht_ok + he_ok;

	// === [Reset cnt] === //
	if ((cck_ok == 0xFFFF) || (ofdm_ok == 0xFFFF) || (ht_ok == 0xFFFF)||
	    (vht_ok == 0xFFFF) || (he_ok == 0xFFFF)) {
		halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 1, phy_idx);
		halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 0, phy_idx);
		crc_ok = 0;
	}

	return crc_ok;
}

u32 halbb_rx_crc_ok_7(struct bb_info *bb, enum phl_phy_idx phy_idx)
{
	u32 cck_ok = 0, ofdm_ok = 0, ht_ok = 0, vht_ok = 0, he_ok = 0, eht_ok = 0;
	u32 crc_ok;

	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;

	BB_DBG(bb, DBG_PHY_CONFIG, "[%s] phy_idx=%d\n", __func__, phy_idx);

	if (bb->ic_type & BB_IC_DBCC_BB0_BB1) {
		cck_ok = halbb_get_reg_cmn(bb, cr->cnt_cck_crc32ok_p0, cr->cnt_cck_crc32ok_p0_m, phy_idx);
	} else {
		if (phy_idx == HW_PHY_0)
			cck_ok = halbb_get_reg(bb, cr->cnt_cck_crc32ok_p0, cr->cnt_cck_crc32ok_p0_m);
		else
			cck_ok = halbb_get_reg(bb, cr->cnt_cck_crc32ok_p1, cr->cnt_cck_crc32ok_p1_m);
	}
	ofdm_ok = halbb_get_reg_cmn(bb, cr->cnt_l_crc_ok, cr->cnt_l_crc_ok_m, phy_idx);
	ht_ok = halbb_get_reg_cmn(bb, cr->cnt_ht_crc_ok, cr->cnt_ht_crc_ok_m, phy_idx);
	vht_ok = halbb_get_reg_cmn(bb, cr->cnt_vht_crc_ok, cr->cnt_vht_crc_ok_m, phy_idx);
	he_ok = halbb_get_reg_cmn(bb, cr->cnt_he_crc_ok, cr->cnt_he_crc_ok_m, phy_idx);
	eht_ok = halbb_get_reg_cmn(bb, cr->cnt_eht_crc_ok, cr->cnt_eht_crc_ok_m, phy_idx);

	crc_ok = cck_ok + ofdm_ok + ht_ok + vht_ok + he_ok + eht_ok;

	// === [Reset cnt] === //
	if ((cck_ok == 0xFFFF) || (ofdm_ok == 0xFFFF) || (ht_ok == 0xFFFF)||
	    (vht_ok == 0xFFFF) || (he_ok == 0xFFFF) || (eht_ok == 0xFFFF)) {
		halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 1, phy_idx);
		halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 0, phy_idx);
		crc_ok = 0;
	}

	return crc_ok;
}

u32 halbb_mp_get_rx_crc_ok(struct bb_info *bb, enum phl_phy_idx phy_idx)
{
	u32 crc_ok = 0;

	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;


	switch (bb->bb_80211spec) {
	case BB_AX_IC:
		crc_ok = halbb_rx_crc_ok_6(bb, phy_idx);
		break;
	case BB_BE_IC:
		crc_ok = halbb_rx_crc_ok_7(bb, phy_idx);
		break;
	default:
		break;
	}

	return crc_ok;
}

u32 halbb_rx_crc_err_6(struct bb_info *bb, enum phl_phy_idx phy_idx)
{
	u32 cck_err = 0, ofdm_err = 0, ht_err = 0, vht_err = 0, he_err = 0;
	u32 crc_err;

	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;

	BB_DBG(bb, DBG_PHY_CONFIG, "[%s] phy_idx=%d\n", __func__, phy_idx);

	if (phy_idx == HW_PHY_0)
		cck_err = halbb_get_reg(bb, cr->cnt_cck_crc32fail_p0, cr->cnt_cck_crc32fail_p0_m);
	else
		cck_err = halbb_get_reg(bb, cr->cnt_cck_crc32fail_p1, cr->cnt_cck_crc32fail_p1_m);

	ofdm_err = halbb_get_reg_cmn(bb, cr->cnt_l_crc_err, cr->cnt_l_crc_err_m, phy_idx);
	ht_err = halbb_get_reg_cmn(bb, cr->cnt_ht_crc_err, cr->cnt_ht_crc_err_m, phy_idx);
	vht_err = halbb_get_reg_cmn(bb, cr->cnt_vht_crc_err, cr->cnt_vht_crc_err_m, phy_idx);
	he_err = halbb_get_reg_cmn(bb, cr->cnt_he_crc_err, cr->cnt_he_crc_err_m, phy_idx);

	crc_err = cck_err + ofdm_err + ht_err + vht_err + he_err;

	// === [Reset cnt] === //
	if ((cck_err == 0xFFFF) || (ofdm_err == 0xFFFF) || (ht_err == 0xFFFF)||
	    (vht_err == 0xFFFF) || (he_err == 0xFFFF)) {
		halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 1, phy_idx);
		halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 0, phy_idx);
		crc_err = 0;
	}

	return crc_err;
}

u32 halbb_rx_crc_err_7(struct bb_info *bb, enum phl_phy_idx phy_idx)
{
	u32 cck_err = 0, ofdm_err = 0, ht_err = 0, vht_err = 0, he_err = 0, eht_err = 0;
	u32 crc_err;

	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;

	BB_DBG(bb, DBG_PHY_CONFIG, "[%s] phy_idx=%d\n", __func__, phy_idx);

	if (bb->ic_type & BB_IC_DBCC_BB0_BB1) {
		cck_err = halbb_get_reg_cmn(bb, cr->cnt_cck_crc32fail_p0, cr->cnt_cck_crc32fail_p0_m, phy_idx);
	} else {
		if (phy_idx == HW_PHY_0)
			cck_err = halbb_get_reg(bb, cr->cnt_cck_crc32fail_p0, cr->cnt_cck_crc32fail_p0_m);
		else
			cck_err = halbb_get_reg(bb, cr->cnt_cck_crc32fail_p1, cr->cnt_cck_crc32fail_p1_m);
	}
	ofdm_err = halbb_get_reg_cmn(bb, cr->cnt_l_crc_err, cr->cnt_l_crc_err_m, phy_idx);
	ht_err = halbb_get_reg_cmn(bb, cr->cnt_ht_crc_err, cr->cnt_ht_crc_err_m, phy_idx);
	vht_err = halbb_get_reg_cmn(bb, cr->cnt_vht_crc_err, cr->cnt_vht_crc_err_m, phy_idx);
	he_err = halbb_get_reg_cmn(bb, cr->cnt_he_crc_err, cr->cnt_he_crc_err_m, phy_idx);
	eht_err = halbb_get_reg_cmn(bb, cr->cnt_eht_crc_err, cr->cnt_eht_crc_err_m, phy_idx);

	crc_err = cck_err + ofdm_err + ht_err + vht_err + he_err + eht_err;

	// === [Reset cnt] === //
	if ((cck_err == 0xFFFF) || (ofdm_err == 0xFFFF) || (ht_err == 0xFFFF)||
	    (vht_err == 0xFFFF) || (he_err == 0xFFFF) || (eht_err == 0xFFFF)) {
		halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 1, phy_idx);
		halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 0, phy_idx);
		crc_err = 0;
	}

	return crc_err;
}

u32 halbb_mp_get_rx_crc_err(struct bb_info *bb, enum phl_phy_idx phy_idx)
{
	u32 crc_err = 0;

	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;


	switch (bb->bb_80211spec) {
	case BB_AX_IC:
		crc_err = halbb_rx_crc_err_6(bb, phy_idx);
		break;
	case BB_BE_IC:
		crc_err = halbb_rx_crc_err_7(bb, phy_idx);
		break;
	default:
		break;
	}

	return crc_err;
}

void halbb_mp_cnt_reset(struct bb_info *bb)
{
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;

	halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 1, bb->bb_phy_idx);
	halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 0, bb->bb_phy_idx);
}

void halbb_mp_reset_cnt(struct bb_info *bb)
{
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;

	// PHY0 cnt reset
	halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 1, HW_PHY_0);
	halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 0, HW_PHY_0);
	// PHY1 cnt reset
	halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 1, HW_PHY_1);
	halbb_set_reg_cmn(bb, cr->rst_all_cnt, cr->rst_all_cnt_m, 0, HW_PHY_1);
}

void halbb_mp_psts_setting(struct bb_info *bb, u32 ie_bitmap_setting)
{
	struct bb_physts_info	*physts = &bb->bb_physts_i;
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	if (ie_bitmap_setting & BIT(IE00_CMN_CCK)) {
		halbb_set_reg(bb, cr->phy_sts_bitmap_he_mu, cr->phy_sts_bitmap_he_mu_m, BIT(IE00_CMN_CCK));
		halbb_set_reg(bb, cr->phy_sts_bitmap_vht_mu, cr->phy_sts_bitmap_vht_mu_m, BIT(IE00_CMN_CCK));
		halbb_set_reg(bb, cr->phy_sts_bitmap_cck, cr->phy_sts_bitmap_cck_m, BIT(IE00_CMN_CCK));
		halbb_set_reg(bb, cr->phy_sts_bitmap_legacy, cr->phy_sts_bitmap_legacy_m, BIT(IE00_CMN_CCK));
		halbb_set_reg(bb, cr->phy_sts_bitmap_ht, cr->phy_sts_bitmap_ht_m, BIT(IE00_CMN_CCK));
		halbb_set_reg(bb, cr->phy_sts_bitmap_vht, cr->phy_sts_bitmap_vht_m, BIT(IE00_CMN_CCK));
		halbb_set_reg(bb, cr->phy_sts_bitmap_he, cr->phy_sts_bitmap_he_m, BIT(IE00_CMN_CCK));
	}
	if (ie_bitmap_setting & BIT(IE01_CMN_OFDM)) {
		halbb_set_reg(bb, cr->phy_sts_bitmap_he_mu, cr->phy_sts_bitmap_he_mu_m, BIT(IE01_CMN_OFDM));
		halbb_set_reg(bb, cr->phy_sts_bitmap_vht_mu, cr->phy_sts_bitmap_vht_mu_m, BIT(IE01_CMN_OFDM));
		halbb_set_reg(bb, cr->phy_sts_bitmap_cck, cr->phy_sts_bitmap_cck_m, BIT(IE01_CMN_OFDM));
		halbb_set_reg(bb, cr->phy_sts_bitmap_legacy, cr->phy_sts_bitmap_legacy_m, BIT(IE01_CMN_OFDM));
		halbb_set_reg(bb, cr->phy_sts_bitmap_ht, cr->phy_sts_bitmap_ht_m, BIT(IE01_CMN_OFDM));
		halbb_set_reg(bb, cr->phy_sts_bitmap_vht, cr->phy_sts_bitmap_vht_m, BIT(IE01_CMN_OFDM));
		halbb_set_reg(bb, cr->phy_sts_bitmap_he, cr->phy_sts_bitmap_he_m, BIT(IE01_CMN_OFDM));
	}

	BB_DBG(bb, DBG_PHY_CONFIG, "[MP] physts ie bitmap setting : 0x%08x\n", ie_bitmap_setting);


}

void
halbb_mp_get_psts_ie_bitmap(struct bb_info *bb, struct bb_mp_psts *bb_mp_physts)
{
	struct bb_physts_info	*physts = &bb->bb_physts_i;
	u32 *bitmap_ptr = &physts->physts_bitmap_recv;
	u32 *mp_bitmap_ptr = &bb_mp_physts->ie_bitmap;

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	mp_bitmap_ptr = bitmap_ptr;

}


void
halbb_mp_get_psts_ie_00(struct bb_info *bb, struct bb_mp_psts *bb_mp_physts)
{
	struct bb_physts_info	*physts = &bb->bb_physts_i;
	struct bb_physts_rslt_0_info *psts_0 = &physts->bb_physts_rslt_0_i;
	struct mp_physts_rslt_0 *mp_psts_0 = &bb_mp_physts->mp_physts_rslt_0_i;

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	mp_psts_0 = (struct mp_physts_rslt_0 *)psts_0;

}

void
halbb_mp_get_psts_ie_01(struct bb_info *bb, struct bb_mp_psts *bb_mp_physts)
{
	struct bb_physts_info	*physts = &bb->bb_physts_i;
	struct bb_physts_rslt_1_info *psts_1 = &physts->bb_physts_rslt_1_i;
	//struct bb_mp_psts *mp_physts = &bb->bb_mp_psts_i;
	struct mp_physts_rslt_1 *mp_psts_1 = &bb_mp_physts->mp_physts_rslt_1_i;

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	mp_psts_1 = (struct mp_physts_rslt_1 *)psts_1;

}

void
halbb_mp_get_psts(struct bb_info *bb , struct bb_mp_psts *bb_mp_physts)
{
	struct bb_physts_info	*physts = &bb->bb_physts_i;
	struct bb_physts_rslt_hdr_info	*psts_h = &physts->bb_physts_rslt_hdr_i;


	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	// IE bitmap info //
	halbb_mp_get_psts_ie_bitmap(bb , bb_mp_physts);

	// detail info of per IE //
	if (bb_mp_physts->ie_bitmap & BIT(IE00_CMN_CCK)) {
		halbb_mp_get_psts_ie_00(bb , bb_mp_physts);
	}
	if (bb_mp_physts->ie_bitmap & BIT(IE01_CMN_OFDM)) {
		halbb_mp_get_psts_ie_01(bb , bb_mp_physts);
	}
}

void halbb_keeper_cond(struct bb_info *bb, bool keeper_en, u8 keeper_trig_cond,
		       u8 dbg_sel, enum phl_phy_idx phy_idx)
{
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;

	halbb_set_reg_cmn(bb, cr->sts_keeper_en, cr->sts_keeper_en_m, keeper_en,
			  phy_idx); //0x0738[4]
	halbb_set_reg_cmn(bb, cr->sts_keeper_trig_cond,
			  cr->sts_keeper_trig_cond_m, keeper_trig_cond, phy_idx); //0x0738[7:6]
	halbb_set_reg_cmn(bb, cr->sts_dbg_sel, cr->sts_dbg_sel_m, dbg_sel,
			  phy_idx); //0x0738[30:28]
}

void halbb_dbg_port_sel(struct bb_info *bb, u16 dbg_port_sel, u8 dbg_port_ip_sel,
			bool dbg_port_ref_clk_en, bool dbg_port_en)
{
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;

	halbb_set_reg(bb, cr->dbg_port_sel, 0xfff, dbg_port_sel); // 0x20f0[11:0]
	halbb_set_reg(bb, cr->dbg_port_ip_sel, cr->dbg_port_ip_sel_m,
		      dbg_port_ip_sel); // 0x20f0[23:16]
	halbb_set_reg(bb, cr->dbg_port_ref_clk_en, cr->dbg_port_ref_clk_en_m,
		      dbg_port_ref_clk_en); // 0x20f4[24]
	halbb_set_reg(bb, cr->dbg_port_en, cr->dbg_port_en_m, dbg_port_en); // 0x20f8[31]
	
}

u8 halbb_mp_get_rxevm(struct bb_info *bb, u8 user, u8 strm, bool is_seg_0)
{
#if 0
	// Note: Only supports 2SS ! //
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;
	enum phl_phy_idx phy_idx = bb->bb_phy_idx;
	u8 rxevm;

	bool is_cck;
	u8 i = 0;
	u32 rxevm_tmp = 0;
	u32 mask_0[4] = {0xff, 0xff00, 0xff0000, 0xff000000};
	u8 addr_ofst[2] = {0, 20};
	u32 dbg_port = 0;
	u16 tmp = 0;
	u32 bitmap[7] = {cr->phy_sts_bitmap_he_mu, cr->phy_sts_bitmap_vht_mu,
			 cr->phy_sts_bitmap_trigbase, cr->phy_sts_bitmap_legacy,
			 cr->phy_sts_bitmap_ht, cr->phy_sts_bitmap_vht,
			 cr->phy_sts_bitmap_he};
	u32 bitmap_m[7] = {cr->phy_sts_bitmap_he_mu_m,
			   cr->phy_sts_bitmap_vht_mu_m,
			   cr->phy_sts_bitmap_trigbase_m,
			   cr->phy_sts_bitmap_legacy_m, cr->phy_sts_bitmap_ht_m,
			   cr->phy_sts_bitmap_vht_m, cr->phy_sts_bitmap_he_m};
	u32 bitmap_restore[7] = {0};

	//=== Backup Bitmap value ===//
	for (i = 0; i < 7; i++)
		bitmap_restore[i] = halbb_get_reg_cmn(bb, bitmap[i], bitmap_m[i], phy_idx);
	//=== Set Bitmap ===//
	for (i = 0; i < 7; i++)
		halbb_set_reg_cmn(bb, bitmap[i], bitmap_m[i], 0x300002, phy_idx);

	// Set keeper condition //
	halbb_keeper_cond(bb, true, 0x1, 0x2, phy_idx);

	// DBG port polling //
	if (halbb_bb_dbg_port_racing(bb, DBGPORT_PRI_3)) {
		tmp = (phy_idx == HW_PHY_0 ? 0x700 : 0x701);
		halbb_dbg_port_sel(bb, tmp, 0x1, 0x0, 0x1);
	} else {
		dbg_port = halbb_get_bb_dbg_port_idx(bb);
		BB_TRACE("[Set dbg_port fail!] Curr-DbgPort=0x%x\n", dbg_port);
		return bb->rxevm;
	}

	if (halbb_get_reg(bb, 0x40b0, BIT(4)) == 0) { // Only use [4], Total [31:0]
		BB_WARNING("No CRC_OK in Keeper!\n");
		//=== Restore Bitmap value ===//
		for (i = 0; i < 7; i++)
			halbb_set_reg_cmn(bb, bitmap[i], bitmap_m[i], bitmap_restore[i], phy_idx);
		return bb->rxevm;
	}

	halbb_set_reg_cmn(bb, cr->sts_keeper_read, cr->sts_keeper_read_m, 1, phy_idx);

	halbb_delay_us(bb, 2);

	if (halbb_get_reg(bb, cr->dbg32_d, BIT(5)) == 1) {
		bb->rxevm.rxevm_seg[phy_idx].rxevm_valid = true;
		// Determine CCK pkt
		halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 1, phy_idx);
		is_cck = (halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xffff00, phy_idx) == 11) ? true : false; // Only use [23:8], Total [31:0]

		// CCK
		if (is_cck) {
			halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 3, phy_idx);
			rxevm_tmp = halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff00, phy_idx); // Only use [15:8], Total [31:0]
			halbb_set_reg_cmn(bb, cr->sts_keeper_read, cr->sts_keeper_read_m, 0, phy_idx);
			bb->rxevm.rxevm_seg[0].rxevm_user[0].rxevm_ss_0 = (u8)rxevm_tmp;
			return bb->rxevm;
		}

		switch (user) {
		case 0:
			halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 15 + addr_ofst[is_seg_0], phy_idx);
			rxevm = halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xffff00, phy_idx);
			break;
		case 1:
			
			break;
		case 2:
			
			break;
		case 3:
			
			break;
		default:
			break;
		}

		for (i = 0; i < 2; i++) {
			
			// Usr0
			halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 15 + addr_ofst[i], phy_idx);
			rxevm_tmp = halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xffff00, phy_idx); // Only use [23:8], Total [31:0]
			bb->rxevm.rxevm_seg[i].rxevm_user[0].rxevm_ss_0 = (rxevm_tmp & 0xff);
			bb->rxevm.rxevm_seg[i].rxevm_user[0].rxevm_ss_1 = ((rxevm_tmp & 0xff00) >> 8);
			// Usr1
			halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 19 + addr_ofst[i], phy_idx);
			bb->rxevm.rxevm_seg[i].rxevm_user[1].rxevm_ss_0 = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff000000, phy_idx); // Only use [31:24], Total [31:0]
			halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 18 + addr_ofst[i], phy_idx);
			bb->rxevm.rxevm_seg[i].rxevm_user[1].rxevm_ss_1 = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff, phy_idx); // Only use [7:0], Total [31:0]
			// Usr2
			halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 22 + addr_ofst[i], phy_idx);
			rxevm_tmp = halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xffff00, phy_idx); // Only use [23:8], Total [31:0]
			bb->rxevm.rxevm_seg[i].rxevm_user[2].rxevm_ss_0 = (rxevm_tmp & 0xff);
			bb->rxevm.rxevm_seg[i].rxevm_user[2].rxevm_ss_1 = ((rxevm_tmp & 0xff00) >> 8);
			// Usr3
			halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 26 + addr_ofst[i], phy_idx);
			bb->rxevm.rxevm_seg[i].rxevm_user[3].rxevm_ss_0 = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff000000, phy_idx); // Only use [31:24], Total [31:0]
			halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 29 + addr_ofst[i], phy_idx);
			bb->rxevm.rxevm_seg[i].rxevm_user[3].rxevm_ss_1 = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff, phy_idx); // Only use [7:0], Total [31:0]
		}
	} else {
		bb->rxevm.rxevm_seg[phy_idx].rxevm_valid = false;
	}
	halbb_set_reg_cmn(bb, cr->sts_keeper_read, cr->sts_keeper_read_m, 0, phy_idx);

	halbb_release_bb_dbg_port(bb);

	//=== Restore Bitmap value ===//
	for (i = 0; i < 7; i++)
		halbb_set_reg_cmn(bb, bitmap[i], bitmap_m[i], bitmap_restore[i], phy_idx);

	return bb->rxevm;
#elif 0
	u8 rx_evm;
	u8 mode;
	u32 user_mask[4] = {0xff000000, 0xff0000, 0xff00, 0xff};

	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);
	/*==== Error handling ====*/
	if ((user >= 4) || (strm >= 4)) {
		BB_WARNING("Invalid User or Stream\n");
		return false;
	}
	/*=== [All tone] ===*/
	halbb_set_reg(bb, cr->rpt_tone_evm_idx, 0x3ff, 0); // Only use [9:0], Total [10:0]
	halbb_set_reg_cmn(bb, cr->rpt_tone_evm_idx, 0x3ff, 0, HW_PHY_1);
	/*=== [Switch MUX] ===*/
	halbb_set_reg(bb, cr->dbg_port_ref_clk_en, cr->dbg_port_ref_clk_en_m, 0);
	halbb_set_reg(bb, cr->dbg_port_en, cr->dbg_port_en_m, 1);
	halbb_set_reg(bb, cr->dbg_port_ip_sel, 0x1f, 2); // Only use [4:0], Total [7:0]
	halbb_set_reg(bb, cr->dbg_port_sel, 0xf0, 1); // Only use [7:4], Total [15:0]

	mode = user + 1 + rxevm_table * 8;

	halbb_set_reg(bb, cr->dbg_port_sel, 0xf, mode); // Only use [3:0], Total [15:0]

	rx_evm = (u8)halbb_get_reg(bb, cr->dbg32_d, user_mask[strm]);
	return rx_evm;
#else
	return 0;
#endif
}

struct rxevm_physts halbb_mp_get_rxevm_physts(struct bb_info *bb,
					      enum phl_phy_idx phy_idx)
{
	// Note: Only supports 2SS ! //
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;

	bool is_cck;
	u8 i = 0;
	u32 rxevm_tmp = 0;
	u8 addr_ofst[2] = {0, 20};
	u32 dbg_port = 0;
	u16 tmp = 0;
	u32 bitmap[7] = {cr->phy_sts_bitmap_he_mu, cr->phy_sts_bitmap_vht_mu,
			 cr->phy_sts_bitmap_trigbase, cr->phy_sts_bitmap_legacy,
			 cr->phy_sts_bitmap_ht, cr->phy_sts_bitmap_vht,
			 cr->phy_sts_bitmap_he};
	u32 bitmap_m[7] = {cr->phy_sts_bitmap_he_mu_m,
			   cr->phy_sts_bitmap_vht_mu_m,
			   cr->phy_sts_bitmap_trigbase_m,
			   cr->phy_sts_bitmap_legacy_m, cr->phy_sts_bitmap_ht_m,
			   cr->phy_sts_bitmap_vht_m, cr->phy_sts_bitmap_he_m};
	u32 bitmap_restore[7] = {0};

	BB_DBG(bb, DBG_PHY_CONFIG, "[Rxevm] halbb_mp_get_rxevm_physts==========================>\n");

	//=== Backup Bitmap value ===//
	for (i = 0; i < 7; i++)
		bitmap_restore[i] = halbb_get_reg_cmn(bb, bitmap[i], bitmap_m[i], phy_idx);
	//=== Set Bitmap ===//
	for (i = 0; i < 7; i++)
		halbb_set_reg_cmn(bb, bitmap[i], bitmap_m[i], 0x300002, phy_idx);

	// Set keeper condition //
	halbb_keeper_cond(bb, true, 0x1, 0x2, phy_idx);

	// DBG port polling //
	if (halbb_bb_dbg_port_racing(bb, DBGPORT_PRI_3)) {
		tmp = (phy_idx == HW_PHY_0 ? 0x700 : 0x701);
		halbb_dbg_port_sel(bb, tmp, 0x1, 0x0, 0x1);
	} else {
		dbg_port = halbb_get_bb_dbg_port_idx(bb);
		BB_TRACE("[Set dbg_port fail!] Curr-DbgPort=0x%x\n", dbg_port);
		return bb->rxevm;
	}

	// Config user0
	halbb_set_reg_cmn(bb, cr->sts_user_sel, cr->sts_user_sel_m, 0, phy_idx);

	// Set to Read State //
	halbb_set_reg_cmn(bb, cr->sts_keeper_read, cr->sts_keeper_read_m, 0, HW_PHY_0);
	halbb_delay_ms(bb, 1);
	// Set to Write State //
	halbb_set_reg_cmn(bb, cr->sts_keeper_read, cr->sts_keeper_read_m, 1, HW_PHY_0);

	halbb_delay_us(bb, 2);

	if (halbb_get_reg(bb, cr->dbg32_d, BIT(5)) == 1) {
		// Determine CCK pkt
		halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 1, phy_idx);
		is_cck = (halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xf, phy_idx) == 11) ? true : false;

		// CCK
		if (is_cck) {
			halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 5, phy_idx);
			rxevm_tmp = halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff00, phy_idx); // Only use [15:8], Total [31:0]
			bb->rxevm.rxevm_seg[0].rxevm_user[0].rxevm_ss_0 = (u8)rxevm_tmp;
			bb->rxevm.rxevm_seg[0].rxevm_user[0].rxevm_ss_1 = 0;
			BB_DBG(bb, DBG_PHY_CONFIG, "[Rxevm][CCK][Phy-%d] rxevm_ss_0=0x%x\n", phy_idx, bb->rxevm.rxevm_seg[0].rxevm_user[0].rxevm_ss_0);
			BB_DBG(bb, DBG_PHY_CONFIG, "[Rxevm][CCK][Phy-%d] rxevm_ss_1=0x%x\n", phy_idx, bb->rxevm.rxevm_seg[0].rxevm_user[0].rxevm_ss_1);
		} else {
			if ((bb->ic_type == BB_RTL8852A) || (bb->ic_type == BB_RTL8852B) || (bb->ic_type == BB_RTL8851B)) {
				for (i = 0; i < 2; i++) {
					// Usr0
					halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 15 + addr_ofst[i], phy_idx);
					rxevm_tmp = halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xffff00, phy_idx); // Only use [23:8], Total [31:0]
					bb->rxevm.rxevm_seg[i].rxevm_user[0].rxevm_ss_0 = (rxevm_tmp & 0xff);
					bb->rxevm.rxevm_seg[i].rxevm_user[0].rxevm_ss_1 = ((rxevm_tmp & 0xff00) >> 8);

					// Usr1
					halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 19 + addr_ofst[i], phy_idx);
					bb->rxevm.rxevm_seg[i].rxevm_user[1].rxevm_ss_0 = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff000000, phy_idx); // Only use [31:24], Total [31:0]
					halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 18 + addr_ofst[i], phy_idx);
					bb->rxevm.rxevm_seg[i].rxevm_user[1].rxevm_ss_1 = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff, phy_idx); // Only use [7:0], Total [31:0]
					// Usr2
					halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 22 + addr_ofst[i], phy_idx);
					rxevm_tmp = halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xffff00, phy_idx); // Only use [23:8], Total [31:0]
					bb->rxevm.rxevm_seg[i].rxevm_user[2].rxevm_ss_0 = (rxevm_tmp & 0xff);
					bb->rxevm.rxevm_seg[i].rxevm_user[2].rxevm_ss_1 = ((rxevm_tmp & 0xff00) >> 8);
					// Usr3
					halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 26 + addr_ofst[i], phy_idx);
					bb->rxevm.rxevm_seg[i].rxevm_user[3].rxevm_ss_0 = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff000000, phy_idx); // Only use [31:24], Total [31:0]
					halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 29 + addr_ofst[i], phy_idx);
					bb->rxevm.rxevm_seg[i].rxevm_user[3].rxevm_ss_1 = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff, phy_idx); // Only use [7:0], Total [31:0]
				}
			} else {
				// Usr0
				halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 15, phy_idx);
				bb->rxevm.rxevm_seg[0].rxevm_user[0].rxevm_ss_0 = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff000000, phy_idx);
				halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 19, phy_idx);
				bb->rxevm.rxevm_seg[0].rxevm_user[0].rxevm_ss_1 = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff000000, phy_idx);
				BB_DBG(bb, DBG_PHY_CONFIG, "[Rxevm] [Phy-%d] rxevm_ss_0=0x%x\n", phy_idx, bb->rxevm.rxevm_seg[0].rxevm_user[0].rxevm_ss_0);
				BB_DBG(bb, DBG_PHY_CONFIG, "[Rxevm] [Phy-%d] rxevm_ss_1=0x%x\n", phy_idx, bb->rxevm.rxevm_seg[0].rxevm_user[0].rxevm_ss_1);
			}
		}
		 bb->bb_cmn_backup_i.last_rxevm_rpt = bb->rxevm;
	} else {
		BB_DBG(bb, DBG_PHY_CONFIG, "[Rxevm] No crc_ok\n");
		bb->rxevm = bb->bb_cmn_backup_i.last_rxevm_rpt;
	}

	halbb_release_bb_dbg_port(bb);

	//=== Restore Bitmap value ===//
	//for (i = 0; i < 7; i++)
	//	halbb_set_reg_cmn(bb, bitmap[i], bitmap_m[i], bitmap_restore[i], phy_idx);

	return bb->rxevm;
}
/*
u16 halbb_mp_get_pwdb_diff(struct bb_info *bb, enum rf_path path)
{
	u16 pwdb_diff;
	u32 rpt_addr[4] = {0x4048, 0x4060, 0x4078, 0x4090};

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	if (path > RF_PATH_D) {
		BB_WARNING("Invalid Path\n");
		return false;
	}

	pwdb_diff = (u16)halbb_get_reg(bb, rpt_addr[path], 0xff800);
	return pwdb_diff;

}
*/

u8 halbb_mp_get_rssi_td(struct bb_info *bb, enum rf_path path)
{
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;
	u32 rpt_mask_ofst[4] = {0xff, 0xff00, 0xff0000, 0xff000000};
	u32 dbg_port = 0;
	u8 rssi;

	// Set keeper condition //
	halbb_keeper_cond(bb, true, 0x1, 0x2, HW_PHY_0);

	// DBG port polling //
	if (halbb_bb_dbg_port_racing(bb, DBGPORT_PRI_3)) {
		halbb_set_reg(bb, cr->dbg_port_ref_clk_en, cr->dbg_port_ref_clk_en_m, 0);
		halbb_set_reg(bb, cr->dbg_port_en, cr->dbg_port_en_m, 1);
		halbb_set_reg(bb, cr->dbg_port_ip_sel, cr->dbg_port_ip_sel_m, 1);
		halbb_set_reg(bb, cr->dbg_port_sel, 0xf00, 7); // Only use [11:8], Total [15:0]

		halbb_set_reg(bb, cr->dbg_port_sel, 0xff, 0);	// Only use [7:0], Total [15:0]
	} else {
		dbg_port = halbb_get_bb_dbg_port_idx(bb);
		BB_TRACE("[Set dbg_port fail!] Curr-DbgPort=0x%x\n", dbg_port);
		return 0xff;
	}

	// Set to Read State //
	halbb_set_reg_cmn(bb, cr->sts_keeper_read, cr->sts_keeper_read_m, 0, HW_PHY_0);
	halbb_delay_ms(bb, 1);
	// Set to Write State //
	halbb_set_reg_cmn(bb, cr->sts_keeper_read, cr->sts_keeper_read_m, 1, HW_PHY_0);

	halbb_delay_us(bb, 2);

	// Polling machanism that determines if read state is successfully set //
	if (halbb_get_reg(bb, cr->dbg32_d, BIT(5)) == 1) {
		halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 0, HW_PHY_0);
		rssi = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, rpt_mask_ofst[path], HW_PHY_0); // Only use [23:15], Total [31:0]
		// bb->bb_cmn_backup_i.last_rpl = rssi;
	} else {
		BB_TRACE("[RSSI] Read state fail !!\n");
		rssi = 0xff;
	}

	halbb_release_bb_dbg_port(bb);

	return rssi;
}

u8 halbb_mp_get_rssi(struct bb_info *bb, enum rf_path path)
{
	u8 rssi;
	u32 dbg_port = 0;

	// RSSI_FD
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;

	// Phy0 / 1
	halbb_set_reg_cmn(bb, cr->sts_keeper_en, cr->sts_keeper_en_m, 1, HW_PHY_0);
	halbb_set_reg_cmn(bb, cr->sts_keeper_trig_cond, cr->sts_keeper_trig_cond_m, 1, HW_PHY_0);
	halbb_set_reg_cmn(bb, cr->sts_dbg_sel, cr->sts_dbg_sel_m, 2, HW_PHY_0);

	if (halbb_bb_dbg_port_racing(bb, DBGPORT_PRI_3)) {
		halbb_set_reg(bb, cr->dbg_port_ref_clk_en, cr->dbg_port_ref_clk_en_m, 0);
		halbb_set_reg(bb, cr->dbg_port_en, cr->dbg_port_en_m, 1);
		halbb_set_reg(bb, cr->dbg_port_ip_sel, cr->dbg_port_ip_sel_m, 1);
		halbb_set_reg(bb, cr->dbg_port_sel, 0xf00, 7); // Only use [11:8], Total [15:0]

		halbb_set_reg(bb, cr->dbg_port_sel, 0xff, 0);	// Only use [7:0], Total [15:0]
	} else {
		dbg_port = halbb_get_bb_dbg_port_idx(bb);
		BB_TRACE("[Set dbg_port fail!] Curr-DbgPort=0x%x\n", dbg_port);
		return bb->bb_cmn_backup_i.last_rpl;
	}

	// Set to Read State //
	halbb_set_reg_cmn(bb, cr->sts_keeper_read, cr->sts_keeper_read_m, 0, HW_PHY_0);
	halbb_delay_ms(bb, 1);
	// Set to Write State //
	halbb_set_reg_cmn(bb, cr->sts_keeper_read, cr->sts_keeper_read_m, 1, HW_PHY_0);

	halbb_delay_us(bb, 2);

	if (halbb_get_reg(bb, cr->dbg32_d, BIT(5)) == 1) {
		halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 3, HW_PHY_0);
		rssi = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff00, HW_PHY_0); // Only use [23:8], Total [31:0]
		bb->bb_cmn_backup_i.last_rpl = rssi;
	} else {
		rssi = bb->bb_cmn_backup_i.last_rpl;
	}


	halbb_release_bb_dbg_port(bb);

	return rssi;
}

s32 halbb_rssi_cal(struct bb_info *bb, u8 rssi_0, u8 rssi_1, bool is_higher_rssi_path, enum phl_phy_idx phy_idx)
{
	u8 rssi_diff = rssi_0 - rssi_1;
	s32 rssi_cal;
	u64 alpha_tmp;

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	if (rssi_diff == 0)
		return 0;

	rssi_diff = rssi_diff % 2 ? (rssi_diff + 1) >> 1 : rssi_diff >> 1;

	alpha_tmp = halbb_db_2_linear((u32) rssi_diff);

	alpha_tmp = (alpha_tmp >> 2) % 2 ? (alpha_tmp >> FRAC_BITS) + 1 : alpha_tmp >> FRAC_BITS;

	if (is_higher_rssi_path)
		rssi_cal = halbb_convert_to_db(alpha_tmp) -
			   halbb_convert_to_db(alpha_tmp + 1) + 3;
	else
		rssi_cal = 3 - halbb_convert_to_db(alpha_tmp + 1);

	return rssi_cal;
}

struct rssi_physts halbb_get_mp_rssi_physts(struct bb_info *bb, enum rf_path path, enum phl_phy_idx phy_idx)
{
	// RSSI_FD: This function is used for MP UI report, which is RPL value
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;
	struct bb_physts_info *physts = &bb->bb_physts_i;
	struct bb_physts_rslt_hdr_info *psts_h = &physts->bb_physts_rslt_hdr_i;
	struct bb_efuse_info efuse = bb->bb_efuse_i;
	struct rssi_physts rssi_rpt;
	bool rssi_0_high = psts_h->rssi[0] > psts_h->rssi[1] ? true : false;
	s32 efuse_tmp = 0;
	u8 band = 0;
	u8 central_ch = bb->hal_com->band[phy_idx].cur_chandef.center_ch;
	u8 RPL = 0;
	u32 rssi = 0;
	u32 dbg_port = 0;
	s32 cal_tmp[2] = {0};
	u16 tmp = 0;

	halbb_mem_set(bb, &rssi_rpt,0,sizeof(rssi_rpt));

	BB_DBG(bb, DBG_PHY_CONFIG, "<====== %s ======>\n", __func__);

	// 2G Band: (0)
	// 5G Band: (1):Low, (2): Mid, (3):High
	if (central_ch <= 14)
		band = 0;
	else if (central_ch >= 36 && central_ch <= 64)
		band = 1;
	else if (central_ch >= 100 && central_ch <= 144)
		band = 2;
	else if (central_ch >= 149 && central_ch <= 177)
		band = 3;
	else
		band = 0;

	efuse_tmp = efuse.gain_offset[RF_PATH_A][band] - efuse.gain_offset[RF_PATH_B][band];

	// Phy0 / 1
	halbb_set_reg_cmn(bb, cr->sts_keeper_en, cr->sts_keeper_en_m, 1, phy_idx);
	halbb_set_reg_cmn(bb, cr->sts_keeper_trig_cond, cr->sts_keeper_trig_cond_m, 1, phy_idx);
	halbb_set_reg_cmn(bb, cr->sts_dbg_sel, cr->sts_dbg_sel_m, 2, phy_idx);

	if (halbb_bb_dbg_port_racing(bb, DBGPORT_PRI_3)) {
		tmp = (phy_idx == HW_PHY_0 ? 0x700 : 0x701);
		halbb_dbg_port_sel(bb, tmp, 0x1, 0x0, 0x1);
	} else {
		dbg_port = halbb_get_bb_dbg_port_idx(bb);
		BB_TRACE("[Set dbg_port fail!] Curr-DbgPort=0x%x\n", dbg_port);
		return bb->bb_cmn_backup_i.last_rssi_rpt;
	}

	// Set to Read State //
	halbb_set_reg_cmn(bb, cr->sts_keeper_read, cr->sts_keeper_read_m, 0, HW_PHY_0);
	halbb_delay_ms(bb, 1);
	// Set to Write State //
	halbb_set_reg_cmn(bb, cr->sts_keeper_read, cr->sts_keeper_read_m, 1, HW_PHY_0);

	halbb_delay_us(bb, 2);

	#if 1
		if (halbb_get_reg(bb, cr->dbg32_d, BIT(5)) == 1) {
			halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 3, phy_idx);
			RPL = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xff00, phy_idx); // Only use [23:8], Total [31:0]
			bb->bb_cmn_backup_i.last_rpl = RPL;
		} else {
			RPL = bb->bb_cmn_backup_i.last_rpl;
		}
	#else
		if (halbb_get_reg(bb, cr->dbg32_d, BIT(5)) != 1) {
			halbb_release_bb_dbg_port(bb);
			return bb->bb_cmn_backup_i.last_rssi_rpt;
		}
	#endif


	halbb_release_bb_dbg_port(bb);

	#if 1
		if (path == RF_PATH_A) {
			rssi_rpt.rssi_seg[0].rssi[0] = (s32)(RPL - 220);
			rssi_rpt.rssi_seg[0].rssi[1] = 0;
		} else if (path == RF_PATH_B) {
			rssi_rpt.rssi_seg[0].rssi[0] = 0;
			rssi_rpt.rssi_seg[0].rssi[1] = (s32)(RPL - 220);
		} else {
			if (rssi_0_high) {
				if (psts_h->rssi[0] - psts_h->rssi[1] > 12) {
					rssi_rpt.rssi_seg[0].rssi[0] = (s32)(RPL - 220 + 6);
					rssi_rpt.rssi_seg[0].rssi[1] = (s32)(psts_h->rssi[1] - 220);
				} else {
					cal_tmp[0] = halbb_rssi_cal(bb, psts_h->rssi[0], psts_h->rssi[1], true, phy_idx);
					cal_tmp[1] = halbb_rssi_cal(bb, psts_h->rssi[0], psts_h->rssi[1], false, phy_idx);
					rssi_rpt.rssi_seg[0].rssi[0] = (s32)((RPL - 220) + (cal_tmp[0] << 1));
					rssi_rpt.rssi_seg[0].rssi[1] = (s32)((RPL - 220) + (efuse_tmp << 1) + (cal_tmp[1] << 1));
				}
			} else {
				if (psts_h->rssi[1] - psts_h->rssi[0] > 12) {
					rssi_rpt.rssi_seg[0].rssi[0] = (s32)(psts_h->rssi[0] - 220);
					rssi_rpt.rssi_seg[0].rssi[1] = (s32)(RPL - 220 + 6);
				} else {
					cal_tmp[0] = halbb_rssi_cal(bb, psts_h->rssi[1], psts_h->rssi[0], false, phy_idx);
					cal_tmp[1] = halbb_rssi_cal(bb, psts_h->rssi[1], psts_h->rssi[0], true, phy_idx);
					rssi_rpt.rssi_seg[0].rssi[0] = (s32)((RPL - 220) + (cal_tmp[0] << 1));
					rssi_rpt.rssi_seg[0].rssi[1] = (s32)((RPL - 220) + (efuse_tmp << 1) + (cal_tmp[1] << 1));
				}
			}
		}
	#else
		#if 0
		rssi_rpt.rssi_seg[0].rssi[0] = (s32)((rssi & 0xff) - 220);
		rssi_rpt.rssi_seg[0].rssi[1] = (s32)(((rssi & 0xff00) >> 8) - 220);
		rssi_rpt.rssi_seg[0].rssi[2] = (s32)(((rssi & 0xff0000) >> 16) - 220);
		rssi_rpt.rssi_seg[0].rssi[3] = (s32)(((rssi & 0xff000000) >> 24) - 220);
		#endif
		rssi = (u32)halbb_mp_get_rpl(bb, RF_PATH_A, phy_idx);
		rssi_rpt.rssi_seg[0].rssi[0] = (s32)((rssi & 0xff) - 220);
		rssi = (u32)halbb_mp_get_rpl(bb, RF_PATH_B, phy_idx);
		rssi_rpt.rssi_seg[0].rssi[1] = (s32)((rssi & 0xff) - 220);
		BB_DBG(bb, DBG_PHY_CONFIG, "[RSSI] [Phy-%d] RSSI[0]=0x%x\n", phy_idx, rssi_rpt.rssi_seg[0].rssi[0]);
		BB_DBG(bb, DBG_PHY_CONFIG, "[RSSI] [Phy-%d] RSSI[1]=0x%x\n", phy_idx, rssi_rpt.rssi_seg[0].rssi[1]);

		bb->bb_cmn_backup_i.last_rssi_rpt = rssi_rpt;
	#endif

	return rssi_rpt;
}

u16 halbb_mp_get_rpl(struct bb_info *bb, enum rf_path path, enum phl_phy_idx phy_idx)
{
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;
	u16 rpl = 0;
	u32 rpl_m[2] = {0x1fff, 0x3ffe000};

	// Error hadling
	if (path > RF_PATH_B || path < RF_PATH_A) {
		BB_WARNING("[%s] path=%d\n", __func__, path);
		return 0xffff;
	}

	if (bb->ic_type == BB_RTL8852C)
		rpl = (u16)halbb_get_reg_cmn(bb, cr->pw_dbm_rx0, *(rpl_m + path), phy_idx);
	else
		rpl = 0xffff;

	return rpl;
}

u32 halbb_mp_get_dc_lvl(struct bb_info *bb, enum rf_path path, bool i_ch, enum phl_phy_idx phy_idx)
{
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;
	u16 path_tmp[2] = {0xb12, 0xb22};
	u32 tmp = 0;
	u32 dbg_port = 0;

	// Error hadling
	if (path > RF_PATH_B || path < RF_PATH_A) {
		BB_WARNING("[%s] path=%d\n", __func__, path);
		return 0xffff;
	}

	// DBG port polling //
	if (halbb_bb_dbg_port_racing(bb, DBGPORT_PRI_3)) {
		halbb_dbg_port_sel(bb, *(path_tmp + path), 0x1, 0x0, 0x1);
	} else {
		dbg_port = halbb_get_bb_dbg_port_idx(bb);
		BB_TRACE("[Set dbg_port fail!] Curr-DbgPort=0x%x\n", dbg_port);
		return 0xffffffff;
	}

	// Release DBG port //
	halbb_release_bb_dbg_port(bb);

	if (i_ch)
		tmp = halbb_get_reg(bb, cr->dbg32_d, 0xfff); // I channel
	else
		tmp = halbb_get_reg(bb, cr->dbg32_d, 0xfff000); // Q channel

	return tmp;
}

u16 halbb_mp_get_pwdbm(struct bb_info *bb, enum rf_path path, enum phl_phy_idx phy_idx)
{
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;
	u16 pwdbm;

	// Error hadling
	if (path > RF_PATH_D) {
		BB_WARNING("Invalid Path!\n");
		return 0xffff;
	}

	if (path == RF_PATH_A)
		pwdbm = (u16)halbb_get_reg(bb, cr->path0_rssi_at_agc_rdy,
					   cr->path0_rssi_at_agc_rdy_m);
	else if (path == RF_PATH_B)
		pwdbm = (u16)halbb_get_reg(bb, cr->path1_rssi_at_agc_rdy,
					   cr->path1_rssi_at_agc_rdy_m);
	else
		pwdbm = 0xffff;

	return pwdbm;
}

u16 halbb_mp_get_cfo(struct bb_info *bb, enum phl_phy_idx phy_idx)
{
	struct bb_rpt_cr_info *cr = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;
	u32 dbg_port = 0;
	u16 cfo = 0;
	u16 tmp = 0;

	// Set keeper condition //
	halbb_keeper_cond(bb, true, 0x1, 0x2, HW_PHY_0);

	// DBG port polling //
	if (halbb_bb_dbg_port_racing(bb, DBGPORT_PRI_3)) {
		tmp = (phy_idx == HW_PHY_0 ? 0x700 : 0x701);
		halbb_dbg_port_sel(bb, tmp, 0x1, 0x0, 0x1);
	} else {
		dbg_port = halbb_get_bb_dbg_port_idx(bb);
		BB_TRACE("[Set dbg_port fail!] Curr-DbgPort=0x%x\n", dbg_port);
		return 0xffff;
	}

	// Set to Read State //
	halbb_set_reg_cmn(bb, cr->sts_keeper_read, cr->sts_keeper_read_m, 0, HW_PHY_0);
	halbb_delay_ms(bb, 1);
	// Set to Write State //
	halbb_set_reg_cmn(bb, cr->sts_keeper_read, cr->sts_keeper_read_m, 1, HW_PHY_0);

	halbb_delay_us(bb, 2);

	// Polling machanism that determines if read state is successfully set  //
	if (halbb_get_reg(bb, cr->dbg32_d, BIT(5)) == 1) {
		halbb_set_reg_cmn(bb, cr->sts_keeper_addr, cr->sts_keeper_addr_m, 2, HW_PHY_0); // 0x738[15:18]
		cfo = (u8)halbb_get_reg_cmn(bb, cr->sts_keeper_data, 0xfff00000, HW_PHY_0); // Only use [31:20], Total [31:0]
		bb->bb_cmn_backup_i.last_cfo = cfo;
	} else {
		cfo = bb->bb_cmn_backup_i.last_cfo;
	}


	// Release DBG port //
	halbb_release_bb_dbg_port(bb);

	return cfo;
}

void halbb_cvrt_2_mp(struct bb_info *bb)
{
	BB_DBG(bb, DBG_COMMON_FLOW, "[%s]", __func__);
	
	bb->bb_link_i.is_linked = true;
	halbb_physts_cvrt_2_mp(bb);
}

void halbb_plcp_show_log(struct bb_info *bb, u32 *_used, char *output,
			 u32 *_out_len)
{
	char plcp_rpt[][16] = {{"PLCP Success"}, {"Length Exceed"}, {"CCK Invalid"},
				{"OFDM Invalid"}, {"HT Invalid"}, {"VHT Invalid"},
				{"HE Invalid"}, {"EHT Invalid"}, {"Spec Invalid"}};
	char gi[][5] = {{"0.4"}, {"0.8"}, {"1.6"}, {"3.2"}};
	char *text_tmp = NULL;

	if (bb->plcp_in.ppdu_type >= 9)
		text_tmp = "EHT ";
	else if (bb->plcp_in.ppdu_type >= 5)
		text_tmp = "HE ";
	else if (bb->plcp_in.ppdu_type >= 4)
		text_tmp = "VHT ";
	else if (bb->plcp_in.ppdu_type >= 2)
		text_tmp = "HT";
	else
		text_tmp = "";

	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "=== [Summary} ============\n");
	if (bb->plcp_in.ppdu_type >= 4) { // >= VHT mode
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			"FIX @ %s %dSS-MCS%d, ",
			text_tmp, bb->plcp_in.usr[0].nss, bb->plcp_in.usr[0].mcs);
	} else if (bb->plcp_in.ppdu_type >= 2) { // >= HT mode
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			"FIX @ HT MCS%d, ",
			bb->plcp_in.usr[0].mcs);
	} else { // Legacy
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			"FIX @ %d M, ",
			bb_phy_rate_table[bb->plcp_in.usr[0].mcs + 4]);
	}
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			"BW:%dM, GI_LTF:%dx%s, Length:%d\n",
			20 << bb->plcp_in.dbw, 1 << bb->plcp_in.he_ltf_type, gi[bb->plcp_in.gi], bb->plcp_in.usr[0].apep);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			"Pkt cnt:%d, Period:%d(us)\n", bb->pmac_in.tx_cnt, bb->pmac_in.period);
}

bool halbb_lbk_info_err_handling(struct bb_info *bb, struct halbb_lbk_info *lbk_i,
				 u32 *_used, char *output, u32 *_out_len)
{
	bool rpt = true;

	if (lbk_i->tx_path == lbk_i->rx_path) {
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"[Error] LBK Tx path == Rx path !!\n");
		rpt = false;
	} else if (bb->plcp_in.dbw != lbk_i->bw) {
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"[Error] dbw (%dM) != cbw(%dM) !!\n",
				20 << bb->plcp_in.dbw, 20 << lbk_i->bw);
		rpt = false;
	}

	return rpt;
}

void halbb_mp_plcp_hdr_init(struct bb_info *bb)
{
	u16 user_cnt = 0;

#ifdef HALBB_COMPILE_BE_SERIES
	if(bb->bb_cmn_hooker->wlan_mode_max == WLAN_MD_11BE) {
		bb->plcp_in.ppdu_type = 9;
		bb->plcp_in.gi = 1;
		bb->plcp_in.he_ltf_type = 1;
	} else
#endif
	if (bb->bb_cmn_hooker->wlan_mode_max == WLAN_MD_11AX) {
		bb->plcp_in.ppdu_type = 5;
		bb->plcp_in.gi = 1;
		bb->plcp_in.he_ltf_type = 1;
	} else if (bb->bb_cmn_hooker->wlan_mode_max == WLAN_MD_11AC) {
		bb->plcp_in.ppdu_type = 4;
		bb->plcp_in.gi = 0;
		bb->plcp_in.he_ltf_type = 0;
	} else {
		bb->plcp_in.ppdu_type = 2;
		bb->plcp_in.gi = 0;
		bb->plcp_in.he_ltf_type = 0;
	}

	// Common info
	bb->plcp_in.source_gen_mode = 2;
	bb->plcp_in.locked_clk = 1;
	bb->plcp_in.dyn_bw = 0;
	bb->plcp_in.ndp_en = 0;
	bb->plcp_in.long_preamble_en = 1;
	bb->plcp_in.stbc = 0;
	// bb->plcp_in.gi = 0; //[Todo] 0:0.4,1:0.8,2:1.6,3:3.2
	bb->plcp_in. tb_l_len = 0;
	bb->plcp_in.tb_ru_tot_sts_max = 0;
	bb->plcp_in.vht_txop_not_allowed = 0;
	bb->plcp_in.tb_disam = 0;
	bb->plcp_in.doppler = 0;
	// bb->plcp_in.he_ltf_type = 0; //[Todo] 2-bit: 0:1x,1:2x,2:4x

	bb->plcp_in.ht_l_len = 0;
	bb->plcp_in.preamble_puncture = 0;
	bb->plcp_in.he_mcs_sigb = 0;
	bb->plcp_in.he_dcm_sigb = 0;
	bb->plcp_in.he_sigb_compress_en = 1;
	bb->plcp_in.max_tx_time_0p4us = 0;

	bb->plcp_in.ul_flag = 0;
	bb->plcp_in.tb_ldpc_extra = 0;
	bb->plcp_in.bss_color = 10;
	bb->plcp_in.sr = 0;
	bb->plcp_in.beamchange_en = 1;
	bb->plcp_in.he_er_u106ru_en = 0;
	bb->plcp_in.ul_srp1 = 0;
	bb->plcp_in.ul_srp2 = 0;
	bb->plcp_in.ul_srp3 = 0;
	bb->plcp_in.ul_srp4 = 0;
	bb->plcp_in.mode = 0; // APEP len

	bb->plcp_in.group_id = 63;
	// bb->plcp_in.ppdu_type = 1; // [Todo] legacy
	bb->plcp_in.txop = 127;
	bb->plcp_in.tb_strt_sts = 0;
	bb->plcp_in.tb_pre_fec_padding_factor = 0;
	bb->plcp_in.txsc = 0;
	bb->plcp_in.tb_mumimo_mode_en = 0;
	bb->plcp_in.dbw = 0; // [Todo] bw20

	bb->plcp_in.nominal_t_pe = 0;
	bb->plcp_in.ness = 0;
	bb->plcp_in.cbw = 0; // [Todo] bw20
	bb->plcp_in.n_user = 1;
	bb->plcp_in.tb_rsvd = 0;

	bb->plcp_in.punc_pattern = 15;
	bb->plcp_in.eht_mcs_sig = 0;
	bb->plcp_in.txsb = 0;

	// per-user
	for (user_cnt = 0; user_cnt < 4; user_cnt++) {
		bb->plcp_in.usr[user_cnt].mcs = 7; //6-bit
		bb->plcp_in.usr[user_cnt].mpdu_len = 0; //14-bit
		bb->plcp_in.usr[user_cnt].n_mpdu = 0; //9-bit
		bb->plcp_in.usr[user_cnt].fec = 0; //1-bit
		bb->plcp_in.usr[user_cnt].dcm = 0; //1-bit
		bb->plcp_in.usr[user_cnt].aid = 0; //12-bit
		bb->plcp_in.usr[user_cnt].scrambler_seed = 63; //8-bit: rand (1~255)
		bb->plcp_in.usr[user_cnt].random_init_seed = 63; //8-bit: rand (1~255)
		bb->plcp_in.usr[user_cnt].apep = 1000; //22-bit
		bb->plcp_in.usr[user_cnt].ru_alloc = 0; //8-bit
		bb->plcp_in.usr[user_cnt].nss = 1; //4-bit
		bb->plcp_in.usr[user_cnt].txbf = 0; //1-bit
		bb->plcp_in.usr[user_cnt].pwr_boost_db = 0; //5-bit
		bb->plcp_in.usr[user_cnt].ru_size = 0; //8-bit
		bb->plcp_in.usr[user_cnt].ru_idx = 1; //8-bit
	}
}

void halbb_pmac_cmd_set(struct bb_info *bb, struct halbb_pmac_info *txinfo,
			struct halbb_lbk_info *lbk_i, char input[][16], u32 *_used, char *output, u32 *_out_len)
{
	u32 val[10] = {0};

	if (_os_strcmp(input[3], "list") == 0 || _os_strcmp(input[3], "-h") == 0) {
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    	    "[PMAC Tx Info]\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			    "is_cck     ,cck_lbk_en\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			    "period(us) ,tx_cnt\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			    "mode {1: Pkt Tx / 3: Cont Tx}\n\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			    "[LBK verify Info]\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			    "tx_path    ,rx_path ,    bw\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			    "bb_lbk {0: AFE lbk / 1: BB lbk}\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			    "tx_delay(ms) (for LBK trigger)\n");
		return;
	}

	if (_os_strcmp(input[3], "en_pmac_tx") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		txinfo->en_pmac_tx = (u8)val[0];
	} else if (_os_strcmp(input[3], "is_cck") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		txinfo->is_cck = (u8)val[0];
	} else if (_os_strcmp(input[3], "tx_cnt") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		txinfo->tx_cnt = (u16)val[0];
	} else if (_os_strcmp(input[3], "period") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		txinfo->period = (u16)val[0];
	} else if (_os_strcmp(input[3], "tx_time") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		txinfo->tx_time = (u8)val[0];
	} else if (_os_strcmp(input[3], "cck_lbk_en") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		txinfo->cck_lbk_en = (bool)val[0];
	} else if (_os_strcmp(input[3], "mode") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		txinfo->mode = (u8)val[0];
	} else if (_os_strcmp(input[3], "bb_lbk") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		lbk_i->is_dgt_lbk = (bool)val[0];
	} else if (_os_strcmp(input[3], "tx_path") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		lbk_i->tx_path = (enum rf_path)val[0];
	} else if (_os_strcmp(input[3], "rx_path") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		lbk_i->rx_path = (enum rf_path)val[0];
	} else if (_os_strcmp(input[3], "bw") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		lbk_i->bw = (enum channel_width)val[0];
	} else if (_os_strcmp(input[3], "tx_delay") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		lbk_i->tx_delay = (u16)val[0];
	}
}

void halbb_pmac_cmd_show(struct bb_info *bb, struct halbb_pmac_info *txinfo,
			 struct halbb_lbk_info *lbk_i, char input[][16],
			 u32 *_used, char *output, u32 *_out_len)
{
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "[PMAC Tx Info]\n");
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "      is_cck : %5d, cck_lbk_en  : %5d\n", txinfo->is_cck, txinfo->cck_lbk_en);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "  period(us) : %5d,     tx_cnt  : %5d\n", txinfo->period, txinfo->tx_cnt);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "        mode : %5d {1: Pkt Tx / 3: Cont Tx}\n\n", txinfo->mode);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "[LBK verify Info]\n");
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "     tx_path : path%d,  rx_path : path%d, bw : %dM\n", lbk_i->tx_path, lbk_i->rx_path, 20 << lbk_i->bw);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "      bb_lbk : %5d {0: AFE lbk / 1: BB lbk}\n", lbk_i->is_dgt_lbk);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "tx_delay(ms) : %5d (for LBK trigger)\n", lbk_i->tx_delay);
}

void halbb_plcp_cmd_set(struct bb_info *bb, struct halbb_plcp_info *in,
		    struct usr_plcp_gen_in *user, char input[][16], u32 *_used,
		    char *output, u32 *_out_len)
{
	u32 val[10] = {0};

	if (_os_strcmp(input[3], "list") == 0 || _os_strcmp(input[3], "-h") == 0) {
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			    "set general parameter : echo bb plcp_tx set {patameter} {value}\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			    " set user's parameter : echo bb plcp_tx set user_idx {idx} {parameter} {value}\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			    " show all parameters  : echo bb plcp_tx show\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			    "         set OFDM 6M  : echo bb plcp_tx set ofdm\n");
		return;
	}
	if (_os_strcmp(input[3], "ofdm") == 0) {
		halbb_plcp_ofdm_6m(bb, in, in->usr, input, _used, output, _out_len);
	} else if (_os_strcmp(input[3], "ppdu_type") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->ppdu_type = (u8)val[0];
	} else if (_os_strcmp(input[3], "stbc") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->stbc = (u8)val[0];
	} else if (_os_strcmp(input[3], "he_dcm_sigb") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->he_dcm_sigb = (u8)val[0];
	} else if (_os_strcmp(input[3], "he_mcs_sigb") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->he_mcs_sigb = (u8)val[0];
	} else if (_os_strcmp(input[3], "dbw") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->dbw = (u8)val[0];
	} else if (_os_strcmp(input[3], "gi") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->gi = (u8)val[0];
	} else if (_os_strcmp(input[3], "ltf_type") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->he_ltf_type = (u8)val[0];
	} else if (_os_strcmp(input[3], "mode") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->mode = (u8)val[0];
	} else if (_os_strcmp(input[3], "max_tx_time") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->max_tx_time_0p4us = (u16)val[0];
	} else if (_os_strcmp(input[3], "n_user") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->n_user = (u8)val[0];
	} else if (_os_strcmp(input[3], "he_er_u106ru_en") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->he_er_u106ru_en = (u8)val[0];
	} else if (_os_strcmp(input[3], "tb_l_len") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->tb_l_len = (u16)val[0];
	} else if (_os_strcmp(input[3], "ru_tot_sts_max") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->tb_ru_tot_sts_max = (u8)val[0];
	} else if (_os_strcmp(input[3], "tb_disam") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->tb_disam = (u8)val[0];
	} else if (_os_strcmp(input[3], "tb_ldpc_extra") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->tb_ldpc_extra = (u8)val[0];
	} else if (_os_strcmp(input[3], "tb_pre_fec_pad") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->tb_pre_fec_padding_factor = (u8)val[0];
	} else if (_os_strcmp(input[3], "l_preamble_en") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->long_preamble_en = (bool)val[0];
	} else if (_os_strcmp(input[3], "cbw") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->cbw = (u8)val[0];
	} else if (_os_strcmp(input[3], "txsc") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->txsc = (u8)val[0];
	}else if (_os_strcmp(input[3], "punc_pattern") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->punc_pattern = (u8)val[0];
	}else if (_os_strcmp(input[3], "tb_mumimo_en") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->tb_mumimo_mode_en = (u8)val[0];
	} else if (_os_strcmp(input[3], "txsb") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->txsb = (u8)val[0];
	} else if (_os_strcmp(input[3], "eht_mcs_sig") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->eht_mcs_sig = (u8)val[0];
	} else if (_os_strcmp(input[3], "user_idx") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		HALBB_SCAN(input[6], DCMD_DECIMAL, &val[1]);
		if (_os_strcmp(input[5], "nss") == 0) {
			in->usr[val[0]].nss = (u8)val[1];
		} else if (_os_strcmp(input[5], "fec") == 0) {
			in->usr[val[0]].fec = (u8)val[1];
		} else if (_os_strcmp(input[5], "payload") == 0 || _os_strcmp(input[5], "apep") == 0) {
			in->usr[val[0]].apep = val[1];
		} else if (_os_strcmp(input[5], "dcm") == 0) {
			in->usr[val[0]].dcm = (bool)val[1];
		} else if (_os_strcmp(input[5], "mcs") == 0) {
			in->usr[val[0]].mcs = (u8)val[1];
		} else if (_os_strcmp(input[5], "mpdu_len") == 0) {
			in->usr[val[0]].mpdu_len = (u16)val[1];
		} else if (_os_strcmp(input[5], "n_mpdu") == 0) {
			in->usr[val[0]].n_mpdu = (u16)val[1];
		} else if (_os_strcmp(input[5], "scrambler_seed") == 0) {
			in->usr[val[0]].scrambler_seed = (u8)val[1];
		} else if (_os_strcmp(input[5], "random_init_seed") == 0) {
			in->usr[val[0]].random_init_seed = (u8)val[1];
		} else if (_os_strcmp(input[5], "aid") == 0) {
			in->usr[val[0]].aid = (u16)val[1];
		} else if (_os_strcmp(input[5], "ru_alloc") == 0) {
			in->usr[val[0]].ru_alloc = (u8)val[1];
		} else if (_os_strcmp(input[5], "pwr_boost_db") == 0) {
			in->usr[val[0]].pwr_boost_db = (u8)val[1];
		}
	}
/*The following parameters are fixed in halbb_plcp_gen_init_7*/
#if 0
	else if (_os_strcmp(input[3], "bss_color") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->bss_color = (u8)val[0];
	} else if (_os_strcmp(input[3], "ht_l_len") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->ht_l_len = (u16)val[0];
	} else if (_os_strcmp(input[3], "ndp_en") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->ndp_en = (u8)val[0];
	} else if (_os_strcmp(input[3], "doppler") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->doppler = (u8)val[0];
	} else if (_os_strcmp(input[3], "nominal_t_pe") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->nominal_t_pe = (u8)val[0];
	} else if (_os_strcmp(input[3], "ness") == 0) {
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[0]);
		in->ness = (u8)val[0];
	}
#endif
}

void halbb_plcp_cmd_show(struct bb_info *bb, struct halbb_plcp_info *in,
		    struct usr_plcp_gen_in *user, char input[][16], u32 *_used,
		    char *output, u32 *_out_len)
{
	int i = 0;
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "        ppdu_type : %5d,                    n_user : %5d\n", in->ppdu_type, in->n_user);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "              cbw : %5d,                       dbw : %5d \n", in->cbw, in->dbw);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "         ltf_type : %5d (0:1x, 1:2x, 2:4x),     gi : %5d (0:0.4,1:0.8,2:1.6,3:3.2)\n", in->he_ltf_type, in->gi);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "             mode : %5d,                      stbc : %5d \n", in->mode, in->stbc);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "             txsb : %5d,              punc_pattern : %5d\n", in->txsb, in->punc_pattern);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "      he_dcm_sigb : %5d,               he_mcs_sigb : %5d\n", in->he_dcm_sigb, in->he_mcs_sigb);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "  he_er_u106ru_en : %5d,                  tb_l_len : %5d\n", in->he_er_u106ru_en, in->tb_l_len);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "   ru_tot_sts_max : %5d,                  tb_disam : %5d\n", in->tb_ru_tot_sts_max, in->tb_disam);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "    tb_ldpc_extra : %5d,            tb_pre_fec_pad : %5d\n", in->tb_ldpc_extra, in->tb_pre_fec_padding_factor);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "      max_tx_time : %5d,             l_preamble_en : %5d\n", in->max_tx_time_0p4us, in->long_preamble_en);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "      eht_mcs_sig : %5d,              tb_mumimo_en : %5d\n", in->eht_mcs_sig, in->tb_mumimo_mode_en);
/*The following parameters are fixed in halbb_plcp_gen_init_7*/
#if 0
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "     nominal_t_pe : %5d,                   doppler : %5d\n", in->nominal_t_pe, in->doppler);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "         ht_l_len : %5d,                 bss_color : %5d\n", in->ht_l_len, in->bss_color);
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "             ness : %5d\n", in->ness);
#endif
	for (i = 0; i < in->n_user; i++){
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		            "user_idx {%d} : \n", i);
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		            "              nss : %5d,                       mcs : %5d\n", in->usr[i].nss, in->usr[i].mcs);
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		            "         ru_alloc : %5d,                   payload : %5d(bytes)\n", in->usr[i].ru_alloc, in->usr[i].apep);
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		            "              fec : %5d,                       aid : %5d\n", in->usr[i].fec, in->usr[i].aid);
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		            "   scrambler_seed : %5d,          random_init_seed : %5d\n", in->usr[i].scrambler_seed, in->usr[0].random_init_seed);
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		            "           n_mpdu : %5d,                  mpdu_len : %5d\n", in->usr[i].n_mpdu, in->usr[i].mpdu_len);
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		            "              dcm : %5d,              pwr_boost_db : %5d\n", in->usr[i].dcm, in->usr[i].pwr_boost_db);
	}
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "[Reference]\n");
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "ppdu_type ==> 0:bmode, 1:Legacy, 2:HT_MF, 3:HT_GF, 4:VHT, 5:HE_SU, 6:HE_ER_SU\n");
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "              7:HE_MU, 8:HE_TB, 9:EHT_SU, 10:EHT_MU, 11:EHT_RU, 12:EHT_ERSU, 13:EHT_TB\n");
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "       bw ==> 0:BW20, 1:BW40, 2:BW80, 3:BW160/BW80+80, 4:BW320\n");
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		    "     mode ==> 0:payload, 1:max_tx_time, 2:n_mpdu,mpdu_len, 3:tb_trigger_mode\n");
}

#ifdef HALBB_COMPILE_BE_SERIES
void halbb_mp_script_gen(struct bb_info *bb, char input[][16], u32 *_used,
			 char *output, u32 *_out_len)
{
	struct bb_stat_cr_info *cr = &bb->bb_cmn_hooker->bb_stat_cr_i;
	struct bb_rpt_cr_info *cr_mp = &bb->bb_cmn_hooker->bb_rpt_i.bb_rpt_cr_i;
	struct halbb_lbk_info *lbk_i = &bb->bb_lbk_i;
	u32 ret_value = 0;
	u32 val[10] = {0};
	u16 rpt = 0;
	u8 i = 0;

	if (_os_strcmp(input[2], "-h") == 0) {
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"bb_pwr_seq\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"bb_lbk\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"bb_tx_path\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"bb_rx_path\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"bb_tx_pwr\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"bb_report\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"bb_reset\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"bb_cca_cfg\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"bb_ch_bw\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"set_ch_bw {pri_ch} {central_ch_seg0} {central_ch_seg1} {band} {bw}\n");
		return;
	}

	if (_os_strcmp(input[2], "bb_pwr_seq") == 0) {
		BB_TRACE("[MP] // <====== bb_pwr_seq ======>\n");
		// Pre-Init
		halbb_bb_pre_init(bb, HW_PHY_0);
		if ((bb->ic_type & BB_IC_DBCC_LEGACY) || (bb->ic_type & BB_IC_DBCC_MLO))
			halbb_bb_pre_init(bb, HW_PHY_1);
		// Init BB CR
		halbb_init_cr_default(bb, false, 0, NULL, HW_PHY_0);
		if ((bb->ic_type & BB_IC_DBCC_LEGACY) || (bb->ic_type & BB_IC_DBCC_MLO))
			halbb_init_cr_default(bb, false, 0, NULL, HW_PHY_1);
		// Post-Init
		halbb_bb_post_init(bb, HW_PHY_0);
		if ((bb->ic_type & BB_IC_DBCC_LEGACY) || (bb->ic_type & BB_IC_DBCC_MLO))
			halbb_bb_post_init(bb, HW_PHY_1);
		BB_TRACE("[MP] // <====== bb_pwr_seq [End] ======>\n");
	} else if (_os_strcmp(input[2], "bb_lbk") == 0) {
		BB_TRACE("[MP] // <====== bb_lbk ======>\n");
		BB_TRACE("[MP] // <====== BB_Digital_LBK_PathA_to_PathB_enable ======>\n");
		// Enable BB LBK (A to B)
		halbb_cfg_lbk_info_init(bb);
		halbb_cfg_lbk(bb, true, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
		BB_TRACE("[MP] // <====== BB_Digital_LBK_PathB_to_PathA_enable ======>\n");
		// Enable BB LBK (B to A)
		bb->bb_lbk_i.tx_path = RF_PATH_B;
		bb->bb_lbk_i.rx_path = RF_PATH_A;
		halbb_cfg_lbk(bb, true, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
		if (bb->ic_type == BB_RTL8934A) {
			BB_TRACE("[MP] // <====== BB_Digital_LBK_PathC_to_PathD_enable ======>\n");
			// Enable BB LBK (B to A)
			bb->bb_lbk_i.tx_path = RF_PATH_C;
			bb->bb_lbk_i.rx_path = RF_PATH_D;
			halbb_cfg_lbk(bb, true, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
			BB_TRACE("[MP] // <====== BB_Digital_LBK_PathD_to_PathC_enable ======>\n");
			// Enable BB LBK (D to C)
			bb->bb_lbk_i.tx_path = RF_PATH_D;
			bb->bb_lbk_i.rx_path = RF_PATH_C;
			halbb_cfg_lbk(bb, true, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
		}
		BB_TRACE("[MP] // <====== BB_Digital_LBK_disable ======>\n");
		// Disable BB LBK
		halbb_cfg_lbk(bb, false, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
		BB_TRACE("[MP] // <====== AFE_LBK_PathA_to_PathB_enable ======>\n");
		// Enable AFE LBK (A to B)
		halbb_cfg_lbk_info_init(bb);
		bb->bb_lbk_i.is_dgt_lbk = false;
		halbb_cfg_lbk(bb, true, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
		BB_TRACE("[MP] // <====== AFE_LBK_PathB_to_PathA_enable ======>\n");
		// Enable AFE LBK (B to A)
		bb->bb_lbk_i.tx_path = RF_PATH_B;
		bb->bb_lbk_i.rx_path = RF_PATH_A;
		halbb_cfg_lbk(bb, true, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
		if (bb->ic_type == BB_RTL8934A) {
			BB_TRACE("[MP] // <====== AFE_LBK_PathC_to_PathD_enable ======>\n");
			// Enable BB LBK (B to A)
			bb->bb_lbk_i.tx_path = RF_PATH_C;
			bb->bb_lbk_i.rx_path = RF_PATH_D;
			halbb_cfg_lbk(bb, true, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
			BB_TRACE("[MP] // <====== AFE_LBK_PathD_to_PathC_enable ======>\n");
			// Enable BB LBK (D to C)
			bb->bb_lbk_i.tx_path = RF_PATH_D;
			bb->bb_lbk_i.rx_path = RF_PATH_C;
			halbb_cfg_lbk(bb, true, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
		}
		BB_TRACE("[MP] // <====== AFE_LBK_disable ======>\n");
		// Disable AFE LBK
		halbb_cfg_lbk(bb, false, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
		BB_TRACE("[MP] // <====== bb_lbk [End] ======>\n");
	} else if (_os_strcmp(input[2], "bb_tx_path") == 0) {
		halbb_cfg_tx_path_pmac(bb, BB_PATH_A, bb->bb_phy_idx);
		halbb_cfg_tx_path_pmac(bb, BB_PATH_B, bb->bb_phy_idx);
		if (bb->ic_type == BB_RTL8934A) {
			halbb_cfg_tx_path_pmac(bb, BB_PATH_C, bb->bb_phy_idx);
			halbb_cfg_tx_path_pmac(bb, BB_PATH_D, bb->bb_phy_idx);
			halbb_cfg_tx_path_pmac(bb, BB_PATH_ABCD, bb->bb_phy_idx);
		} else {
			halbb_cfg_tx_path_pmac(bb, BB_PATH_AB, bb->bb_phy_idx);
		}
	} else if (_os_strcmp(input[2], "bb_rx_path") == 0) {
		halbb_cfg_rx_path(bb, BB_PATH_A, bb->bb_phy_idx);
		halbb_cfg_rx_path(bb, BB_PATH_B, bb->bb_phy_idx);
		if (bb->ic_type == BB_RTL8934A) {
			halbb_cfg_rx_path(bb, BB_PATH_C, bb->bb_phy_idx);
			halbb_cfg_rx_path(bb, BB_PATH_D, bb->bb_phy_idx);
			halbb_cfg_rx_path(bb, BB_PATH_ABCDE, bb->bb_phy_idx);
		} else {
			halbb_cfg_rx_path(bb, BB_PATH_AB, bb->bb_phy_idx);
		}
	} else if (_os_strcmp(input[2], "bb_tx_pwr") == 0) {
		BB_TRACE("[MP] // <====== bb_tx_pwr ======>\n");
		halbb_set_txpwr_dbm(bb, 0x40, bb->bb_phy_idx);
		BB_TRACE("[MP] // <====== bb_tx_pwr [End] ======>\n");
	} else if (_os_strcmp(input[2], "bb_report") == 0) {
		BB_TRACE("[MP] // <====== bb_report ======>\n");
		BB_TRACE("[MP] // --- cck cca ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->cck_cca, cr->cck_cca_m, bb->bb_phy_idx);
		BB_TRACE("[MP] // --- ofdm cca ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->ofdm_cca, cr->ofdm_cca_m, bb->bb_phy_idx);
		BB_TRACE("[MP] // --- cck_crc_ok ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->cck_crc32ok, cr->cck_crc32ok_m, bb->bb_phy_idx);
		BB_TRACE("[MP] // --- cck_crc_err ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->cck_crc32fail, cr->cck_crc32fail_m, bb->bb_phy_idx);
		BB_TRACE("[MP] // --- legacy_crc_ok ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->l_crc_ok, cr->l_crc_ok_m, bb->bb_phy_idx);
		BB_TRACE("[MP] // --- legacy_crc_err ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->l_crc_err, cr->l_crc_err_m, bb->bb_phy_idx);
		BB_TRACE("[MP] // --- ht_crc_ok ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->ht_crc_ok, cr->ht_crc_ok_m, bb->bb_phy_idx);
		BB_TRACE("[MP] // --- ht_crc_err ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->ht_crc_err, cr->ht_crc_err_m, bb->bb_phy_idx);
		BB_TRACE("[MP] // --- vht_crc_ok ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->vht_crc_ok, cr->vht_crc_ok_m, bb->bb_phy_idx);
		BB_TRACE("[MP] // --- vht_crc_err ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->vht_crc_err, cr->vht_crc_err_m, bb->bb_phy_idx);
		BB_TRACE("[MP] // --- he_crc_ok ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->he_crc_ok, cr->he_crc_ok_m, bb->bb_phy_idx);
		BB_TRACE("[MP] // --- he_crc_err ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->he_crc_err, cr->he_crc_err_m, bb->bb_phy_idx);
		BB_TRACE("[MP] // --- eht_crc_ok ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->eht_crc_ok, cr->eht_crc_ok_m, bb->bb_phy_idx);
		BB_TRACE("[MP] // --- eht_crc_err ---\n");
		ret_value = halbb_get_reg_cmn(bb, cr->eht_crc_err, cr->eht_crc_err_m, bb->bb_phy_idx);

		for (i = 0; i < bb->num_ss; i++) {
			bb->bb_dv_pxp_dbg_i.evm_case = IS_OFDM;
			BB_TRACE("[MP] // --- OFDM SS%d Rx EVM ---\n", i);
			rpt = halbb_physts_keeper(bb, KEEPER_EVM, i, bb->bb_phy_idx);
			bb->bb_dv_pxp_dbg_i.evm_case = DISABLE;
		}
		bb->bb_dv_pxp_dbg_i.evm_case = IS_CCK;
		BB_TRACE("[MP] // --- CCK Rx EVM ---\n");
		rpt = halbb_physts_keeper(bb, KEEPER_EVM, 0, bb->bb_phy_idx);
		bb->bb_dv_pxp_dbg_i.evm_case = DISABLE;
		BB_TRACE("[MP] // <====== bb_report [End] ======>\n");
	} else if (_os_strcmp(input[2], "bb_reset") == 0) {
		BB_TRACE("[MP] // <====== bb_reset ======>\n");
		halbb_set_reg_cmn(bb, cr_mp->bb_rst, cr_mp->bb_rst_m, 0, bb->bb_phy_idx);
		halbb_set_reg_cmn(bb, cr_mp->bb_rst, cr_mp->bb_rst_m, 1, bb->bb_phy_idx);
		BB_TRACE("[MP] // <====== bb_reset [End] ======>\n");
	} else if (_os_strcmp(input[2], "bb_cca_cfg") == 0) {
		BB_TRACE("[MP] // <====== bb_cca_cfg ======>\n");
		BB_TRACE("[MP] // cck cca disable\n");
		halbb_set_reg_cmn(bb, cr_mp->dis_11b_cca, cr_mp->dis_11b_cca_m, 1, bb->bb_phy_idx);
		BB_TRACE("[MP] // cck cca enable\n");
		halbb_set_reg_cmn(bb, cr_mp->dis_11b_cca, cr_mp->dis_11b_cca_m, 0, bb->bb_phy_idx);
		BB_TRACE("[MP] // cck ofdm disable\n");
		halbb_set_reg_cmn(bb, cr_mp->dis_pd_flag, cr_mp->dis_pd_flag_m, 1, bb->bb_phy_idx);
		BB_TRACE("[MP] // cck ofdm enable\n");
		halbb_set_reg_cmn(bb, cr_mp->dis_pd_flag, cr_mp->dis_pd_flag_m, 0, bb->bb_phy_idx);
		BB_TRACE("[MP] // <====== bb_cca_cfg [End] ======>\n");
	} else if (_os_strcmp(input[2], "bb_ch_bw") == 0) {
		BB_TRACE("[MP] // BB_5G_CBW20_PriSB0_CH36\n");
		halbb_ctrl_bw_ch(bb, 36, 36, 0, BAND_ON_5G, CHANNEL_WIDTH_20, bb->bb_phy_idx);
		BB_TRACE("[MP] // BB_5G_CBW40_PriSB0_CH38\n");
		halbb_ctrl_bw_ch(bb, 36, 38, 0, BAND_ON_5G, CHANNEL_WIDTH_40, bb->bb_phy_idx);
	} else if (_os_strcmp(input[2], "set_ch_bw") == 0) {
		HALBB_SCAN(input[3], DCMD_DECIMAL, &val[0]);
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[1]);
		HALBB_SCAN(input[5], DCMD_DECIMAL, &val[2]);
		HALBB_SCAN(input[6], DCMD_DECIMAL, &val[3]);
		HALBB_SCAN(input[7], DCMD_DECIMAL, &val[4]);
		halbb_ctrl_bw_ch(bb, (u8)val[0], (u8)val[1], (u8)val[2],
					  (enum band_type)val[3],
					  (enum channel_width)val[4],
					  bb->bb_phy_idx);
	}
}
#endif

void halbb_mp_dbg(struct bb_info *bb, char input[][16], u32 *_used,
		  char *output, u32 *_out_len)
{
	u32 val[10] = {0};
	u32 used = *_used;
	u32 out_len = *_out_len;
	u8 pri_sb = 0;
	char *text_tmp = NULL;
	// struct rxevm_physts rxevm;
	struct halbb_lbk_info *lbk_i = &bb->bb_lbk_i;
	struct halbb_pmac_info *txinfo = &bb->pmac_in;
	struct halbb_plcp_info *in = &bb->plcp_in;
	enum plcp_sts rpt = SPEC_INVALID;
	bool rpt_match = false;
	u16 rxevm = 0;
	char plcp_rpt[][16] = {{"PLCP Success"}, {"Length Exceed"}, {"CCK Invalid"},
				{"OFDM Invalid"}, {"HT Invalid"}, {"VHT Invalid"},
				{"HE Invalid"}, {"EHT Invalid"}, {"Spec Invalid"}};
	char gi[][5] = {{"0.4"}, {"0.8"}, {"1.6"}, {"3.2"}};

	// halbb_mem_set(bb, &rxevm,0,sizeof(rxevm));

	if (_os_strcmp(input[1], "-h") == 0) {
#ifdef HALBB_COMPILE_BE_SERIES
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "evm {\"cck\"/\"ofdm\"} {ss}\n");
#endif
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "lbk {-h}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "pmac_tx {-h}\n");
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "plcp {-h}\n");
		#ifdef HALBB_COMPILE_BE_SERIES
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "mp_script {-h}\n");
		#endif
		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "reset_cnt\n");
#ifdef HALBB_COMPILE_BE_SERIES
	} else if (_os_strcmp(input[1], "evm") == 0) {
		HALBB_SCAN(input[3], DCMD_DECIMAL, &val[0]);

		if (val[0] >= bb->num_ss) {
			BB_DBG_CNSL(out_len, used, output + used, out_len - used,
					"[MP] Invalid ss = %d\n", val[0]);
			return;
		}

		if (_os_strcmp(input[2], "cck") == 0)
			bb->bb_dv_pxp_dbg_i.evm_case = IS_CCK;
		else
			bb->bb_dv_pxp_dbg_i.evm_case = IS_OFDM;

		rxevm = halbb_physts_keeper(bb, KEEPER_EVM, (u8)val[0], bb->bb_phy_idx);

		BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			"[MP] [SS%d] RXEVM = -%d\n", val[0], rxevm >> 2);

		bb->bb_dv_pxp_dbg_i.evm_case = DISABLE;
#endif
	} else if (_os_strcmp(input[1], "lbk") == 0) {
		if (_os_strcmp(input[2], "-h") == 0) {
			BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "lbk trig\n");
			BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "lbk auto {en}\n");
			BB_DBG_CNSL(out_len, used, output + used, out_len - used,
			 "lbk cfg {en}\n");
			return;
		}

		if (_os_strcmp(input[2], "auto") == 0) {
			HALBB_SCAN(input[3], DCMD_DECIMAL, &val[0]);

			if (val[0] == 1) {
				BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
					"Watchdog LBK verify enable\n");

				halbb_ctrl_bw(bb, pri_sb, BAND_ON_5G, lbk_i->bw, bb->bb_phy_idx);
				halbb_cfg_lbk(bb, true, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
				rpt = halbb_plcp_gen(bb, &bb->plcp_in, bb->plcp_in.usr, bb->bb_phy_idx);

				BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
						"%s\n",	plcp_rpt[rpt]);

				// Error handling
				if (rpt != PLCP_SUCCESS)
					return;

				rpt_match = halbb_lbk_info_err_handling(bb, lbk_i, _used, output, _out_len);
				if (!rpt_match)
					return;

				halbb_plcp_show_log(bb, _used, output, _out_len);
				bb->bb_cmn_hooker->lbk_verify_en = true;
			} else {
				BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
					"Watchdog LBK verify disable\n");
				bb->bb_cmn_hooker->lbk_verify_en = false;
				bb->pmac_in.en_pmac_tx = false;
				halbb_set_pmac_tx(bb, &bb->pmac_in, bb->bb_phy_idx);
				halbb_cfg_lbk(bb, false, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
				halbb_set_tmac_tx(bb, bb->bb_phy_idx);
			}
		} else if (_os_strcmp(input[2], "trig") == 0) {
			BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
					"One shot trigger LBK\n");
			halbb_ctrl_bw(bb, pri_sb, BAND_ON_5G, lbk_i->bw, bb->bb_phy_idx);
			halbb_cfg_lbk(bb, true, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
			rpt = halbb_plcp_gen(bb, &bb->plcp_in, bb->plcp_in.usr, bb->bb_phy_idx);

			BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
						"%s\n",	plcp_rpt[rpt]);

			// Error handling
			if (rpt != PLCP_SUCCESS)
				return;

			rpt_match = halbb_lbk_info_err_handling(bb, lbk_i, _used, output, _out_len);
			if (!rpt_match)
				return;

			halbb_plcp_show_log(bb, _used, output, _out_len);

			BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
					"Tx delay:%d(ms)\n", lbk_i->tx_delay);

			bb->pmac_in.en_pmac_tx = true;
			halbb_set_pmac_tx(bb, &bb->pmac_in, bb->bb_phy_idx);

			halbb_delay_ms(bb, bb->bb_lbk_i.tx_delay);

			bb->pmac_in.en_pmac_tx = false;
			halbb_set_pmac_tx(bb, &bb->pmac_in, bb->bb_phy_idx);
			halbb_cfg_lbk(bb, false, lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
		} else if (_os_strcmp(input[2], "cfg") == 0) {
			HALBB_SCAN(input[3], DCMD_DECIMAL, &val[0]);
			halbb_cfg_lbk(bb, (bool)val[0], lbk_i->is_dgt_lbk, lbk_i->tx_path, lbk_i->rx_path, lbk_i->bw, bb->bb_phy_idx);
			BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				    "LBK cfg en=%d\n", val[0]);
		}

	} else if (_os_strcmp(input[1], "pmac_tx") == 0) {
		if (_os_strcmp(input[2], "-h") == 0) {
			BB_DBG_CNSL(out_len, used, output + used, out_len - used,
				    "set : set input argument\n");
			BB_DBG_CNSL(out_len, used, output + used, out_len - used,
				    "show : show parameters setting\n");
			BB_DBG_CNSL(out_len, used, output + used, out_len - used,
				    "default : set parameters as default value\n");
			BB_DBG_CNSL(out_len, used, output + used, out_len - used,
				    "trig_tx {1/2}: start/stop tx, 1 : start, 2: stop\n");
			return;
		}

		if (_os_strcmp(input[2], "set") == 0) {
			halbb_pmac_cmd_set(bb, txinfo, lbk_i, input, _used, output, _out_len);
		} else if (_os_strcmp(input[2], "show") == 0) {
			halbb_pmac_cmd_show(bb, txinfo, lbk_i, input, _used, output, _out_len);
		} else if (_os_strcmp(input[2], "default") == 0) {
			halbb_cfg_pmac_tx_info_init(bb);
			halbb_cfg_lbk_info_init(bb);
		} else if (_os_strcmp(input[2], "trig_tx") == 0) {
			if (_os_strcmp(input[3], "1") == 0)
				txinfo->en_pmac_tx = 1;
			else if (_os_strcmp(input[3], "2") == 0)
				txinfo->en_pmac_tx = 0;
			halbb_set_pmac_tx(bb, txinfo, bb->bb_phy_idx);
		}
	} else if (_os_strcmp(input[1], "plcp") == 0) {
		if (_os_strcmp(input[2], "-h") == 0) {
			BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			    "set : set input argument\n");
			BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"show : show parameters setting\n");
			BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"default : set parameters as default value\n");
			BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"gen : plcp gen\n");
			return;
		}

		if (_os_strcmp(input[2], "set") == 0) {
			halbb_plcp_cmd_set(bb, in, in->usr, input, _used, output, _out_len);
		} else if (_os_strcmp(input[2], "show") == 0){
			halbb_plcp_cmd_show(bb, in, in->usr, input, _used, output, _out_len);
		} else if (_os_strcmp(input[2], "default") == 0){
			halbb_mp_plcp_hdr_init(bb);
		} else if (_os_strcmp(input[2], "gen") == 0){
			halbb_plcp_gen(bb, in, in->usr, bb->bb_phy_idx);
		}
	}
#ifdef HALBB_COMPILE_BE_SERIES
	else if (_os_strcmp(input[1], "mp_script") == 0) {
		halbb_mp_script_gen(bb, input, _used, output, _out_len);
	}
#endif
	else if (_os_strcmp(input[1], "reset_cnt") == 0) {
		halbb_mp_cnt_reset(bb);
	}

	*_used = used;
	*_out_len = out_len;
}

void halbb_cr_cfg_mp_init(struct bb_info *bb)
{
	struct bb_rpt_info *rpt_info = &bb->bb_cmn_hooker->bb_rpt_i;
	struct bb_rpt_cr_info *cr = &rpt_info->bb_rpt_cr_i;

	switch (bb->cr_type) {

	#ifdef HALBB_COMPILE_AP_SERIES
	case BB_AP:
		cr->cnt_ccktxon = CNT_CCKTXON_A;
		cr->cnt_ccktxon_m = CNT_CCKTXON_A_M;
		cr->cnt_ofdmtxon = CNT_OFDMTXON_A;
		cr->cnt_ofdmtxon_m = CNT_OFDMTXON_A_M;
		cr->cnt_cck_crc32ok_p0 = CNT_CCK_CRC32OK_P0_A;
		cr->cnt_cck_crc32ok_p0_m = CNT_CCK_CRC32OK_P0_A_M;
		cr->cnt_cck_crc32ok_p1 = CNT_CCK_CRC32OK_P1_A;
		cr->cnt_cck_crc32ok_p1_m = CNT_CCK_CRC32OK_P1_A_M;
		cr->cnt_l_crc_ok = CNT_L_CRC_OK_A;
		cr->cnt_l_crc_ok_m = CNT_L_CRC_OK_A_M;
		cr->cnt_ht_crc_ok = CNT_HT_CRC_OK_A;
		cr->cnt_ht_crc_ok_m = CNT_HT_CRC_OK_A_M;
		cr->cnt_vht_crc_ok = CNT_VHT_CRC_OK_A;
		cr->cnt_vht_crc_ok_m = CNT_VHT_CRC_OK_A_M;
		cr->cnt_he_crc_ok = CNT_HE_CRC_OK_A;
		cr->cnt_he_crc_ok_m = CNT_HE_CRC_OK_A_M;
		cr->cnt_cck_crc32fail_p0 = CNT_CCK_CRC32FAIL_P0_A;
		cr->cnt_cck_crc32fail_p0_m = CNT_CCK_CRC32FAIL_P0_A_M;
		cr->cnt_cck_crc32fail_p1 = CNT_CCK_CRC32FAIL_P1_A;
		cr->cnt_cck_crc32fail_p1_m = CNT_CCK_CRC32FAIL_P1_A_M;
		cr->cnt_l_crc_err = CNT_L_CRC_ERR_A;
		cr->cnt_l_crc_err_m = CNT_L_CRC_ERR_A_M;
		cr->cnt_ht_crc_err = CNT_HT_CRC_ERR_A;
		cr->cnt_ht_crc_err_m = CNT_HT_CRC_ERR_A_M;
		cr->cnt_vht_crc_err = CNT_VHT_CRC_ERR_A;
		cr->cnt_vht_crc_err_m = CNT_VHT_CRC_ERR_A_M;
		cr->cnt_he_crc_err = CNT_HE_CRC_ERR_A;
		cr->cnt_he_crc_err_m = CNT_HE_CRC_ERR_A_M;
		cr->rst_all_cnt = RST_ALL_CNT_A;
		cr->rst_all_cnt_m = RST_ALL_CNT_A_M;
		cr->phy_sts_bitmap_he_mu = PHY_STS_BITMAP_HE_MU_A;
		cr->phy_sts_bitmap_he_mu_m = PHY_STS_BITMAP_HE_MU_A_M;
		cr->phy_sts_bitmap_vht_mu = PHY_STS_BITMAP_VHT_MU_A;
		cr->phy_sts_bitmap_vht_mu_m = PHY_STS_BITMAP_VHT_MU_A_M;
		cr->phy_sts_bitmap_cck = PHY_STS_BITMAP_CCK_A;
		cr->phy_sts_bitmap_cck_m = PHY_STS_BITMAP_CCK_A_M;
		cr->phy_sts_bitmap_legacy = PHY_STS_BITMAP_LEGACY_A;
		cr->phy_sts_bitmap_legacy_m = PHY_STS_BITMAP_LEGACY_A_M;
		cr->phy_sts_bitmap_ht = PHY_STS_BITMAP_HT_A;
		cr->phy_sts_bitmap_ht_m = PHY_STS_BITMAP_HT_A_M;
		cr->phy_sts_bitmap_vht = PHY_STS_BITMAP_VHT_A;
		cr->phy_sts_bitmap_vht_m = PHY_STS_BITMAP_VHT_A_M;
		cr->phy_sts_bitmap_he = PHY_STS_BITMAP_HE_A;
		cr->phy_sts_bitmap_he_m = PHY_STS_BITMAP_HE_A_M;
		cr->rpt_tone_evm_idx = RPT_TONE_EVM_IDX_A;
		cr->rpt_tone_evm_idx_m = RPT_TONE_EVM_IDX_A_M;
		cr->dbg_port_ref_clk_en = DBG_PORT_REF_CLK_EN_A;
		cr->dbg_port_ref_clk_en_m = DBG_PORT_REF_CLK_EN_A_M;
		cr->dbg_port_en = DBG_PORT_EN_A;
		cr->dbg_port_en_m = DBG_PORT_EN_A_M;
		cr->dbg_port_ip_sel = DBG_PORT_IP_SEL_A;
		cr->dbg_port_ip_sel_m = DBG_PORT_IP_SEL_A_M;
		cr->dbg_port_sel = DBG_PORT_SEL_A;
		cr->dbg_port_sel_m = DBG_PORT_SEL_A_M;
		cr->dbg32_d = DBG32_D_A;
		cr->dbg32_d_m = DBG32_D_A_M;
		cr->phy_sts_bitmap_trigbase = PHY_STS_BITMAP_TRIGBASE_A;
		cr->phy_sts_bitmap_trigbase_m = PHY_STS_BITMAP_TRIGBASE_A_M;
		cr->sts_keeper_en = STS_KEEPER_EN_A;
		cr->sts_keeper_en_m = STS_KEEPER_EN_A_M;
		cr->sts_keeper_trig_cond = STS_KEEPER_TRIG_COND_A;
		cr->sts_keeper_trig_cond_m = STS_KEEPER_TRIG_COND_A_M;
		cr->sts_dbg_sel = STS_DBG_SEL_A;
		cr->sts_dbg_sel_m = STS_DBG_SEL_A_M;
		cr->sts_keeper_read = STS_KEEPER_READ_A;
		cr->sts_keeper_read_m = STS_KEEPER_READ_A_M;
		cr->sts_keeper_addr = STS_KEEPER_ADDR_A;
		cr->sts_keeper_addr_m = STS_KEEPER_ADDR_A_M;
		cr->sts_keeper_data = STS_KEEPER_DATA_A;
		cr->sts_keeper_data_m = STS_KEEPER_DATA_A_M;
		cr->sts_user_sel = STS_USER_SEL_A;
		cr->sts_user_sel_m = STS_USER_SEL_A_M;
		break;

	#endif
	#ifdef HALBB_COMPILE_AP2_SERIES
	case BB_AP2:
		cr->cnt_ccktxon = CNT_CCKTXON_A2;
		cr->cnt_ccktxon_m = CNT_CCKTXON_A2_M;
		cr->cnt_ofdmtxon = CNT_OFDMTXON_A2;
		cr->cnt_ofdmtxon_m = CNT_OFDMTXON_A2_M;
		cr->cnt_cck_crc32ok_p0 = CNT_CCK_CRC32OK_P0_A2;
		cr->cnt_cck_crc32ok_p0_m = CNT_CCK_CRC32OK_P0_A2_M;
		cr->cnt_cck_crc32ok_p1 = CNT_CCK_CRC32OK_P1_A2;
		cr->cnt_cck_crc32ok_p1_m = CNT_CCK_CRC32OK_P1_A2_M;
		cr->cnt_l_crc_ok = CNT_L_CRC_OK_A2;
		cr->cnt_l_crc_ok_m = CNT_L_CRC_OK_A2_M;
		cr->cnt_ht_crc_ok = CNT_HT_CRC_OK_A2;
		cr->cnt_ht_crc_ok_m = CNT_HT_CRC_OK_A2_M;
		cr->cnt_vht_crc_ok = CNT_VHT_CRC_OK_A2;
		cr->cnt_vht_crc_ok_m = CNT_VHT_CRC_OK_A2_M;
		cr->cnt_he_crc_ok = CNT_HE_CRC_OK_A2;
		cr->cnt_he_crc_ok_m = CNT_HE_CRC_OK_A2_M;
		cr->cnt_cck_crc32fail_p0 = CNT_CCK_CRC32FAIL_P0_A2;
		cr->cnt_cck_crc32fail_p0_m = CNT_CCK_CRC32FAIL_P0_A2_M;
		cr->cnt_cck_crc32fail_p1 = CNT_CCK_CRC32FAIL_P1_A2;
		cr->cnt_cck_crc32fail_p1_m = CNT_CCK_CRC32FAIL_P1_A2_M;
		cr->cnt_l_crc_err = CNT_L_CRC_ERR_A2;
		cr->cnt_l_crc_err_m = CNT_L_CRC_ERR_A2_M;
		cr->cnt_ht_crc_err = CNT_HT_CRC_ERR_A2;
		cr->cnt_ht_crc_err_m = CNT_HT_CRC_ERR_A2_M;
		cr->cnt_vht_crc_err = CNT_VHT_CRC_ERR_A2;
		cr->cnt_vht_crc_err_m = CNT_VHT_CRC_ERR_A2_M;
		cr->cnt_he_crc_err = CNT_HE_CRC_ERR_A2;
		cr->cnt_he_crc_err_m = CNT_HE_CRC_ERR_A2_M;
		cr->rst_all_cnt = RST_ALL_CNT_A2;
		cr->rst_all_cnt_m = RST_ALL_CNT_A2_M;
		cr->phy_sts_bitmap_he_mu = PHY_STS_BITMAP_HE_MU_A2;
		cr->phy_sts_bitmap_he_mu_m = PHY_STS_BITMAP_HE_MU_A2_M;
		cr->phy_sts_bitmap_vht_mu = PHY_STS_BITMAP_VHT_MU_A2;
		cr->phy_sts_bitmap_vht_mu_m = PHY_STS_BITMAP_VHT_MU_A2_M;
		cr->phy_sts_bitmap_cck = PHY_STS_BITMAP_CCK_A2;
		cr->phy_sts_bitmap_cck_m = PHY_STS_BITMAP_CCK_A2_M;
		cr->phy_sts_bitmap_legacy = PHY_STS_BITMAP_LEGACY_A2;
		cr->phy_sts_bitmap_legacy_m = PHY_STS_BITMAP_LEGACY_A2_M;
		cr->phy_sts_bitmap_ht = PHY_STS_BITMAP_HT_A2;
		cr->phy_sts_bitmap_ht_m = PHY_STS_BITMAP_HT_A2_M;
		cr->phy_sts_bitmap_vht = PHY_STS_BITMAP_VHT_A2;
		cr->phy_sts_bitmap_vht_m = PHY_STS_BITMAP_VHT_A2_M;
		cr->phy_sts_bitmap_he = PHY_STS_BITMAP_HE_A2;
		cr->phy_sts_bitmap_he_m = PHY_STS_BITMAP_HE_A2_M;
		cr->rpt_tone_evm_idx = RPT_TONE_EVM_IDX_A2;
		cr->rpt_tone_evm_idx_m = RPT_TONE_EVM_IDX_A2_M;
		cr->dbg_port_ref_clk_en = DBG_PORT_REF_CLK_EN_A2;
		cr->dbg_port_ref_clk_en_m = DBG_PORT_REF_CLK_EN_A2_M;
		cr->dbg_port_en = DBG_PORT_EN_A2;
		cr->dbg_port_en_m = DBG_PORT_EN_A2_M;
		cr->dbg_port_ip_sel = DBG_PORT_IP_SEL_A2;
		cr->dbg_port_ip_sel_m = DBG_PORT_IP_SEL_A2_M;
		cr->dbg_port_sel = DBG_PORT_SEL_A2;
		cr->dbg_port_sel_m = DBG_PORT_SEL_A2_M;
		cr->dbg32_d = DBG32_D_A2;
		cr->dbg32_d_m = DBG32_D_A2_M;
		cr->phy_sts_bitmap_trigbase = PHY_STS_BITMAP_TRIGBASE_A2;
		cr->phy_sts_bitmap_trigbase_m = PHY_STS_BITMAP_TRIGBASE_A2_M;
		cr->sts_keeper_en = STS_KEEPER_EN_A2;
		cr->sts_keeper_en_m = STS_KEEPER_EN_A2_M;
		cr->sts_keeper_trig_cond = STS_KEEPER_TRIG_COND_A2;
		cr->sts_keeper_trig_cond_m = STS_KEEPER_TRIG_COND_A2_M;
		cr->sts_dbg_sel = STS_DBG_SEL_A2;
		cr->sts_dbg_sel_m = STS_DBG_SEL_A2_M;
		cr->sts_keeper_read = STS_KEEPER_READ_A2;
		cr->sts_keeper_read_m = STS_KEEPER_READ_A2_M;
		cr->sts_keeper_addr = STS_KEEPER_ADDR_A2;
		cr->sts_keeper_addr_m = STS_KEEPER_ADDR_A2_M;
		cr->sts_keeper_data = STS_KEEPER_DATA_A2;
		cr->sts_keeper_data_m = STS_KEEPER_DATA_A2_M;
		cr->path0_rssi_at_agc_rdy = PATH0_RSSI_AT_AGC_RDY_A2;
		cr->path0_rssi_at_agc_rdy_m = PATH0_RSSI_AT_AGC_RDY_A2_M;
		cr->path1_rssi_at_agc_rdy = PATH1_RSSI_AT_AGC_RDY_A2;
		cr->path1_rssi_at_agc_rdy_m = PATH1_RSSI_AT_AGC_RDY_A2_M;
		cr->sts_user_sel = STS_USER_SEL_A2;
		cr->sts_user_sel_m = STS_USER_SEL_A2_M;
		cr->path1_g_lna6 = PATH1_R_G_G_LNA6_A2;
		cr->path1_g_lna6_m = PATH1_R_G_G_LNA6_A2_M;
		break;

	#endif
	#ifdef HALBB_COMPILE_CLIENT_SERIES
	case BB_CLIENT:
		cr->cnt_ccktxon = CNT_CCKTXON_C;
		cr->cnt_ccktxon_m = CNT_CCKTXON_C_M;
		cr->cnt_ofdmtxon = CNT_OFDMTXON_C;
		cr->cnt_ofdmtxon_m = CNT_OFDMTXON_C_M;
		cr->cnt_cck_crc32ok_p0 = CNT_CCK_CRC32OK_P0_C;
		cr->cnt_cck_crc32ok_p0_m = CNT_CCK_CRC32OK_P0_C_M;
		cr->cnt_cck_crc32ok_p1 = CNT_CCK_CRC32OK_P1_C;
		cr->cnt_cck_crc32ok_p1_m = CNT_CCK_CRC32OK_P1_C_M;
		cr->cnt_l_crc_ok = CNT_L_CRC_OK_C;
		cr->cnt_l_crc_ok_m = CNT_L_CRC_OK_C_M;
		cr->cnt_ht_crc_ok = CNT_HT_CRC_OK_C;
		cr->cnt_ht_crc_ok_m = CNT_HT_CRC_OK_C_M;
		cr->cnt_vht_crc_ok = CNT_VHT_CRC_OK_C;
		cr->cnt_vht_crc_ok_m = CNT_VHT_CRC_OK_C_M;
		cr->cnt_he_crc_ok = CNT_HE_CRC_OK_C;
		cr->cnt_he_crc_ok_m = CNT_HE_CRC_OK_C_M;
		cr->cnt_cck_crc32fail_p0 = CNT_CCK_CRC32FAIL_P0_C;
		cr->cnt_cck_crc32fail_p0_m = CNT_CCK_CRC32FAIL_P0_C_M;
		cr->cnt_cck_crc32fail_p1 = CNT_CCK_CRC32FAIL_P1_C;
		cr->cnt_cck_crc32fail_p1_m = CNT_CCK_CRC32FAIL_P1_C_M;
		cr->cnt_l_crc_err = CNT_L_CRC_ERR_C;
		cr->cnt_l_crc_err_m = CNT_L_CRC_ERR_C_M;
		cr->cnt_ht_crc_err = CNT_HT_CRC_ERR_C;
		cr->cnt_ht_crc_err_m = CNT_HT_CRC_ERR_C_M;
		cr->cnt_vht_crc_err = CNT_VHT_CRC_ERR_C;
		cr->cnt_vht_crc_err_m = CNT_VHT_CRC_ERR_C_M;
		cr->cnt_he_crc_err = CNT_HE_CRC_ERR_C;
		cr->cnt_he_crc_err_m = CNT_HE_CRC_ERR_C_M;
		cr->rst_all_cnt = RST_ALL_CNT_C;
		cr->rst_all_cnt_m = RST_ALL_CNT_C_M;
		cr->phy_sts_bitmap_he_mu = PHY_STS_BITMAP_HE_MU_C;
		cr->phy_sts_bitmap_he_mu_m = PHY_STS_BITMAP_HE_MU_C_M;
		cr->phy_sts_bitmap_vht_mu = PHY_STS_BITMAP_VHT_MU_C;
		cr->phy_sts_bitmap_vht_mu_m = PHY_STS_BITMAP_VHT_MU_C_M;
		cr->phy_sts_bitmap_cck = PHY_STS_BITMAP_CCK_C;
		cr->phy_sts_bitmap_cck_m = PHY_STS_BITMAP_CCK_C_M;
		cr->phy_sts_bitmap_legacy = PHY_STS_BITMAP_LEGACY_C;
		cr->phy_sts_bitmap_legacy_m = PHY_STS_BITMAP_LEGACY_C_M;
		cr->phy_sts_bitmap_ht = PHY_STS_BITMAP_HT_C;
		cr->phy_sts_bitmap_ht_m = PHY_STS_BITMAP_HT_C_M;
		cr->phy_sts_bitmap_vht = PHY_STS_BITMAP_VHT_C;
		cr->phy_sts_bitmap_vht_m = PHY_STS_BITMAP_VHT_C_M;
		cr->phy_sts_bitmap_he = PHY_STS_BITMAP_HE_C;
		cr->phy_sts_bitmap_he_m = PHY_STS_BITMAP_HE_C_M;
		cr->rpt_tone_evm_idx = RPT_TONE_EVM_IDX_C;
		cr->rpt_tone_evm_idx_m = RPT_TONE_EVM_IDX_C_M;
		cr->dbg_port_ref_clk_en = DBG_PORT_REF_CLK_EN_C;
		cr->dbg_port_ref_clk_en_m = DBG_PORT_REF_CLK_EN_C_M;
		cr->dbg_port_en = DBG_PORT_EN_C;
		cr->dbg_port_en_m = DBG_PORT_EN_C_M;
		cr->dbg_port_ip_sel = DBG_PORT_IP_SEL_C;
		cr->dbg_port_ip_sel_m = DBG_PORT_IP_SEL_C_M;
		cr->dbg_port_sel = DBG_PORT_SEL_C;
		cr->dbg_port_sel_m = DBG_PORT_SEL_C_M;
		cr->dbg32_d = DBG32_D_C;
		cr->dbg32_d_m = DBG32_D_C_M;
		cr->phy_sts_bitmap_trigbase = PHY_STS_BITMAP_TRIGBASE_C;
		cr->phy_sts_bitmap_trigbase_m = PHY_STS_BITMAP_TRIGBASE_C_M;
		cr->sts_keeper_en = STS_KEEPER_EN_C;
		cr->sts_keeper_en_m = STS_KEEPER_EN_C_M;
		cr->sts_keeper_trig_cond = STS_KEEPER_TRIG_COND_C;
		cr->sts_keeper_trig_cond_m = STS_KEEPER_TRIG_COND_C_M;
		cr->sts_dbg_sel = STS_DBG_SEL_C;
		cr->sts_dbg_sel_m = STS_DBG_SEL_C_M;
		cr->sts_keeper_read = STS_KEEPER_READ_C;
		cr->sts_keeper_read_m = STS_KEEPER_READ_C_M;
		cr->sts_keeper_addr = STS_KEEPER_ADDR_C;
		cr->sts_keeper_addr_m = STS_KEEPER_ADDR_C_M;
		cr->sts_keeper_data = STS_KEEPER_DATA_C;
		cr->sts_keeper_data_m = STS_KEEPER_DATA_C_M;
		cr->pw_dbm_rx0 = P0_L_TOT_PW_DBM_RX0_C;
		cr->pw_dbm_rx0_m = P0_L_TOT_PW_DBM_RX0_C_M;
		cr->sts_user_sel = STS_USER_SEL_C;
		cr->sts_user_sel_m = STS_USER_SEL_C_M;
		cr->path1_g_lna6 = PATH1_R_G_G_LNA6_C;
		cr->path1_g_lna6_m = PATH1_R_G_G_LNA6_C_M;
		break;
	#endif

	#ifdef HALBB_COMPILE_BE0_SERIES
	case BB_BE0:
		cr->cnt_ccktxon = CNT_CCKTXON_BE0;
		cr->cnt_ccktxon_m = CNT_CCKTXON_BE0_M;
		cr->cnt_ofdmtxon = CNT_OFDMTXON_BE0;
		cr->cnt_ofdmtxon_m = CNT_OFDMTXON_BE0_M;
		cr->cnt_cck_crc32ok_p0 = CNT_CCK_CRC32OK_P0_BE0;
		cr->cnt_cck_crc32ok_p0_m = CNT_CCK_CRC32OK_P0_BE0_M;
		cr->cnt_cck_crc32ok_p1 = CNT_CCK_CRC32OK_P1_BE0;
		cr->cnt_cck_crc32ok_p1_m = CNT_CCK_CRC32OK_P1_BE0_M;
		cr->cnt_l_crc_ok = CNT_L_CRC_OK_BE0;
		cr->cnt_l_crc_ok_m = CNT_L_CRC_OK_BE0_M;
		cr->cnt_ht_crc_ok = CNT_HT_CRC_OK_BE0;
		cr->cnt_ht_crc_ok_m = CNT_HT_CRC_OK_BE0_M;
		cr->cnt_vht_crc_ok = CNT_VHT_CRC_OK_BE0;
		cr->cnt_vht_crc_ok_m = CNT_VHT_CRC_OK_BE0_M;
		cr->cnt_he_crc_ok = CNT_HE_CRC_OK_BE0;
		cr->cnt_he_crc_ok_m = CNT_HE_CRC_OK_BE0_M;
		cr->cnt_eht_crc_ok = CNT_EHT_CRC_OK_BE0;
		cr->cnt_eht_crc_ok_m = CNT_EHT_CRC_OK_BE0_M;
		cr->cnt_cck_crc32fail_p0 = CNT_CCK_CRC32FAIL_P0_BE0;
		cr->cnt_cck_crc32fail_p0_m = CNT_CCK_CRC32FAIL_P0_BE0_M;
		cr->cnt_cck_crc32fail_p1 = CNT_CCK_CRC32FAIL_P1_BE0;
		cr->cnt_cck_crc32fail_p1_m = CNT_CCK_CRC32FAIL_P1_BE0_M;
		cr->cnt_l_crc_err = CNT_L_CRC_ERR_BE0;
		cr->cnt_l_crc_err_m = CNT_L_CRC_ERR_BE0_M;
		cr->cnt_ht_crc_err = CNT_HT_CRC_ERR_BE0;
		cr->cnt_ht_crc_err_m = CNT_HT_CRC_ERR_BE0_M;
		cr->cnt_vht_crc_err = CNT_VHT_CRC_ERR_BE0;
		cr->cnt_vht_crc_err_m = CNT_VHT_CRC_ERR_BE0_M;
		cr->cnt_he_crc_err = CNT_HE_CRC_ERR_BE0;
		cr->cnt_he_crc_err_m = CNT_HE_CRC_ERR_BE0_M;
		cr->cnt_eht_crc_err = CNT_EHT_CRC_ERR_BE0;
		cr->cnt_eht_crc_err_m = CNT_EHT_CRC_ERR_BE0_M;
		cr->rst_all_cnt = RST_ALL_CNT_BE0;
		cr->rst_all_cnt_m = RST_ALL_CNT_BE0_M;
		cr->phy_sts_bitmap_he_mu = PHY_STS_BITMAP_HE_MU_BE0;
		cr->phy_sts_bitmap_he_mu_m = PHY_STS_BITMAP_HE_MU_BE0_M;
		cr->phy_sts_bitmap_vht_mu = PHY_STS_BITMAP_VHT_MU_BE0;
		cr->phy_sts_bitmap_vht_mu_m = PHY_STS_BITMAP_VHT_MU_BE0_M;
		cr->phy_sts_bitmap_cck = PHY_STS_BITMAP_CCK_BE0;
		cr->phy_sts_bitmap_cck_m = PHY_STS_BITMAP_CCK_BE0_M;
		cr->phy_sts_bitmap_legacy = PHY_STS_BITMAP_LEGACY_BE0;
		cr->phy_sts_bitmap_legacy_m = PHY_STS_BITMAP_LEGACY_BE0_M;
		cr->phy_sts_bitmap_ht = PHY_STS_BITMAP_HT_BE0;
		cr->phy_sts_bitmap_ht_m = PHY_STS_BITMAP_HT_BE0_M;
		cr->phy_sts_bitmap_vht = PHY_STS_BITMAP_VHT_BE0;
		cr->phy_sts_bitmap_vht_m = PHY_STS_BITMAP_VHT_BE0_M;
		cr->phy_sts_bitmap_he = PHY_STS_BITMAP_HE_BE0;
		cr->phy_sts_bitmap_he_m = PHY_STS_BITMAP_HE_BE0_M;
		cr->rpt_tone_evm_idx = RXINT_R_RPT_TONE_EVM_IDX_BE0;
		cr->rpt_tone_evm_idx_m = RXINT_R_RPT_TONE_EVM_IDX_BE0_M;
		cr->dbg_port_ref_clk_en = DBG_PORT_REF_CLK_EN_BE0;
		cr->dbg_port_ref_clk_en_m = DBG_PORT_REF_CLK_EN_BE0_M;
		cr->dbg_port_en = DBG_PORT_EN_BE0;
		cr->dbg_port_en_m = DBG_PORT_EN_BE0_M;
		cr->dbg_port_ip_sel = DBG_PORT_IP_SEL_BE0;
		cr->dbg_port_ip_sel_m = DBG_PORT_IP_SEL_BE0_M;
		cr->dbg_port_sel = DBG_PORT_SEL_BE0;
		cr->dbg_port_sel_m = DBG_PORT_SEL_BE0_M;
		cr->dbg32_d = DBG32_D_BE0;
		cr->dbg32_d_m = DBG32_D_BE0_M;
		cr->phy_sts_bitmap_trigbase = PHY_STS_BITMAP_TRIGBASE_BE0;
		cr->phy_sts_bitmap_trigbase_m = PHY_STS_BITMAP_TRIGBASE_BE0_M;
		cr->sts_keeper_en = STS_KEEPER_EN_BE0;
		cr->sts_keeper_en_m = STS_KEEPER_EN_BE0_M;
		cr->sts_keeper_trig_cond = STS_KEEPER_TRIG_COND_BE0;
		cr->sts_keeper_trig_cond_m = STS_KEEPER_TRIG_COND_BE0_M;
		cr->sts_dbg_sel = STS_DBG_SEL_BE0;
		cr->sts_dbg_sel_m = STS_DBG_SEL_BE0_M;
		cr->sts_keeper_read = STS_KEEPER_READ_BE0;
		cr->sts_keeper_read_m = STS_KEEPER_READ_BE0_M;
		cr->sts_keeper_addr = STS_KEEPER_ADDR_BE0;
		cr->sts_keeper_addr_m = STS_KEEPER_ADDR_BE0_M;
		cr->sts_keeper_data = STS_KEEPER_DATA_BE0;
		cr->sts_keeper_data_m = STS_KEEPER_DATA_BE0_M;
		cr->path0_rssi_at_agc_rdy = PATH0_RSSI_AT_AGC_RDY_BE0;
		cr->path0_rssi_at_agc_rdy_m = PATH0_RSSI_AT_AGC_RDY_BE0_M;
		cr->path1_rssi_at_agc_rdy = PATH1_RSSI_AT_AGC_RDY_BE0;
		cr->path1_rssi_at_agc_rdy_m = PATH1_RSSI_AT_AGC_RDY_BE0_M;
		cr->sts_user_sel = STS_USER_SEL_BE0;
		cr->sts_user_sel_m = STS_USER_SEL_BE0_M;
		cr->path1_g_lna6 = PATH1_R_G_G_LNA6_BE0;
		cr->path1_g_lna6_m = PATH1_R_G_G_LNA6_BE0_M;
		break;
	#endif

	#ifdef HALBB_COMPILE_BE1_SERIES
	case BB_BE1:
		cr->cnt_ccktxon = CNT_CCKTXON_BE1;
		cr->cnt_ccktxon_m = CNT_CCKTXON_BE1_M;
		cr->cnt_ofdmtxon = CNT_OFDMTXON_BE1;
		cr->cnt_ofdmtxon_m = CNT_OFDMTXON_BE1_M;
		cr->cnt_cck_crc32ok_p0 = CNT_CCK_CRC32OK_P0_BE1;
		cr->cnt_cck_crc32ok_p0_m = CNT_CCK_CRC32OK_P0_BE1_M;
		cr->cnt_cck_crc32ok_p1 = CNT_CCK_CRC32OK_P1_BE1;
		cr->cnt_cck_crc32ok_p1_m = CNT_CCK_CRC32OK_P1_BE1_M;
		cr->cnt_l_crc_ok = CNT_L_CRC_OK_BE1; //
		cr->cnt_l_crc_ok_m = CNT_L_CRC_OK_BE1_M;
		cr->cnt_ht_crc_ok = CNT_HT_CRC_OK_BE1;
		cr->cnt_ht_crc_ok_m = CNT_HT_CRC_OK_BE1_M;
		cr->cnt_vht_crc_ok = CNT_VHT_CRC_OK_BE1;
		cr->cnt_vht_crc_ok_m = CNT_VHT_CRC_OK_BE1_M;
		cr->cnt_he_crc_ok = CNT_HE_CRC_OK_BE1;
		cr->cnt_he_crc_ok_m = CNT_HE_CRC_OK_BE1_M;
		cr->cnt_eht_crc_ok = CNT_EHT_CRC_OK_BE1;
		cr->cnt_eht_crc_ok_m = CNT_EHT_CRC_OK_BE1_M;
		cr->cnt_cck_crc32fail_p0 = CNT_CCK_CRC32FAIL_P0_BE1;
		cr->cnt_cck_crc32fail_p0_m = CNT_CCK_CRC32FAIL_P0_BE1_M;
		cr->cnt_cck_crc32fail_p1 = CNT_CCK_CRC32FAIL_P1_BE1;
		cr->cnt_cck_crc32fail_p1_m = CNT_CCK_CRC32FAIL_P1_BE1_M;
		cr->cnt_l_crc_err = CNT_L_CRC_ERR_BE1;
		cr->cnt_l_crc_err_m = CNT_L_CRC_ERR_BE1_M;
		cr->cnt_ht_crc_err = CNT_HT_CRC_ERR_BE1;
		cr->cnt_ht_crc_err_m = CNT_HT_CRC_ERR_BE1_M;
		cr->cnt_vht_crc_err = CNT_VHT_CRC_ERR_BE1;
		cr->cnt_vht_crc_err_m = CNT_VHT_CRC_ERR_BE1_M;
		cr->cnt_he_crc_err = CNT_HE_CRC_ERR_BE1;
		cr->cnt_he_crc_err_m = CNT_HE_CRC_ERR_BE1_M;
		cr->cnt_eht_crc_err = CNT_EHT_CRC_ERR_BE1;
		cr->cnt_eht_crc_err_m = CNT_EHT_CRC_ERR_BE1_M;
		cr->rst_all_cnt = TOP_CTRL_P0_R_RST_ALL_CNT_BE1;
		cr->rst_all_cnt_m = TOP_CTRL_P0_R_RST_ALL_CNT_BE1_M;
		cr->phy_sts_bitmap_he_mu = TOP_CTRL_P0_R_PHY_STS_BITMAP_HE_MU_BE1;
		cr->phy_sts_bitmap_he_mu_m = TOP_CTRL_P0_R_PHY_STS_BITMAP_HE_MU_BE1_M;
		cr->phy_sts_bitmap_vht_mu = TOP_CTRL_P0_R_PHY_STS_BITMAP_VHT_MU_BE1;
		cr->phy_sts_bitmap_vht_mu_m = TOP_CTRL_P0_R_PHY_STS_BITMAP_VHT_MU_BE1_M;
		cr->phy_sts_bitmap_cck = TOP_CTRL_P0_R_PHY_STS_BITMAP_CCK_BE1;
		cr->phy_sts_bitmap_cck_m = TOP_CTRL_P0_R_PHY_STS_BITMAP_CCK_BE1_M;
		cr->phy_sts_bitmap_legacy = TOP_CTRL_P0_R_PHY_STS_BITMAP_LEGACY_BE1;
		cr->phy_sts_bitmap_legacy_m = TOP_CTRL_P0_R_PHY_STS_BITMAP_LEGACY_BE1_M;
		cr->phy_sts_bitmap_ht = TOP_CTRL_P0_R_PHY_STS_BITMAP_HT_BE1;
		cr->phy_sts_bitmap_ht_m = TOP_CTRL_P0_R_PHY_STS_BITMAP_HT_BE1_M;
		cr->phy_sts_bitmap_vht = TOP_CTRL_P0_R_PHY_STS_BITMAP_VHT_BE1;
		cr->phy_sts_bitmap_vht_m = TOP_CTRL_P0_R_PHY_STS_BITMAP_VHT_BE1_M;
		cr->phy_sts_bitmap_he = TOP_CTRL_P0_R_PHY_STS_BITMAP_HE_BE1;
		cr->phy_sts_bitmap_he_m = TOP_CTRL_P0_R_PHY_STS_BITMAP_HE_BE1_M;
		cr->rpt_tone_evm_idx = RXINT_R_RPT_TONE_EVM_IDX_BE1;
		cr->rpt_tone_evm_idx_m = RXINT_R_RPT_TONE_EVM_IDX_BE1_M;
		cr->dbg_port_ref_clk_en = DBG_PORT_REF_CLK_EN_BE1;
		cr->dbg_port_ref_clk_en_m = DBG_PORT_REF_CLK_EN_BE1_M;
		cr->dbg_port_en = DBG_PORT_EN_BE1;
		cr->dbg_port_en_m = DBG_PORT_EN_BE1_M;
		cr->dbg_port_ip_sel = DBG_PORT_IP_SEL_BE1;
		cr->dbg_port_ip_sel_m = DBG_PORT_IP_SEL_BE1_M;
		cr->dbg_port_sel = DBG_PORT_SEL_BE1;
		cr->dbg_port_sel_m = DBG_PORT_SEL_BE1_M;
		cr->dbg32_d = DBG32_D_BE1;
		cr->dbg32_d_m = DBG32_D_BE1_M;
		cr->phy_sts_bitmap_trigbase = TOP_CTRL_P0_R_PHY_STS_BITMAP_TRIGBASE_BE1;
		cr->phy_sts_bitmap_trigbase_m = TOP_CTRL_P0_R_PHY_STS_BITMAP_TRIGBASE_BE1_M;
		cr->sts_keeper_en = TOP_CTRL_P0_R_STS_KEEPER_EN_BE1;
		cr->sts_keeper_en_m = TOP_CTRL_P0_R_STS_KEEPER_EN_BE1_M;
		cr->sts_keeper_trig_cond = TOP_CTRL_P0_R_STS_KEEPER_TRIG_COND_BE1;
		cr->sts_keeper_trig_cond_m = TOP_CTRL_P0_R_STS_KEEPER_TRIG_COND_BE1_M;
		cr->sts_dbg_sel = TOP_CTRL_P0_R_STS_DBG_SEL_BE1;
		cr->sts_dbg_sel_m = TOP_CTRL_P0_R_STS_DBG_SEL_BE1_M;
		cr->sts_keeper_read = TOP_CTRL_P0_R_STS_KEEPER_READ_BE1;
		cr->sts_keeper_read_m = TOP_CTRL_P0_R_STS_KEEPER_READ_BE1_M;
		cr->sts_keeper_addr = TOP_CTRL_P0_R_STS_KEEPER_ADDR_BE1;
		cr->sts_keeper_addr_m = TOP_CTRL_P0_R_STS_KEEPER_ADDR_BE1_M;
		cr->sts_keeper_data = STS_KEEPER_DATA_BE1;
		cr->sts_keeper_data_m = STS_KEEPER_DATA_BE1_M;
		cr->path0_rssi_at_agc_rdy = RSSI_AT_AGC_RDY_PATH0_BE1;
		cr->path0_rssi_at_agc_rdy_m = RSSI_AT_AGC_RDY_PATH0_BE1_M;
		cr->path1_rssi_at_agc_rdy = RSSI_AT_AGC_RDY_PATH1_BE1;
		cr->path1_rssi_at_agc_rdy_m = RSSI_AT_AGC_RDY_PATH1_BE1_M;
		cr->sts_user_sel = TOP_CTRL_P0_R_STS_USER_SEL_BE1;
		cr->sts_user_sel_m = TOP_CTRL_P0_R_STS_USER_SEL_BE1_M;
		cr->path1_g_lna6 = PATH1_R_G_G_LNA6_BE1;
		cr->path1_g_lna6_m = PATH1_R_G_G_LNA6_BE1_M;
		break;
	#endif

	default:
		BB_WARNING("[%s] BBCR Hook FAIL!\n", __func__);
		if (bb->bb_dbg_i.cr_fake_init_hook_en) {
			BB_TRACE("[%s] BBCR fake init\n", __func__);
			halbb_cr_hook_fake_init(bb, (u32 *)cr, (sizeof(struct bb_rpt_cr_info) >> 2));
		}
		break;
	}

	if (bb->bb_dbg_i.cr_init_hook_recorder_en) {
		BB_TRACE("[%s] BBCR Hook dump\n", __func__);
		halbb_cr_hook_init_dump(bb, (u32 *)cr, (sizeof(struct bb_rpt_cr_info) >> 2));
	}
}


