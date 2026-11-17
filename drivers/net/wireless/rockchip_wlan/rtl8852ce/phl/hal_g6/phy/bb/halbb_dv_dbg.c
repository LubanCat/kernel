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

#ifdef HALBB_DV_PXP_DBG_SUPPORT
void halbb_print_ic_type(struct bb_info *bb, u32 *_used, char *output, u32 *_out_len)
{
	char *name = NULL;

	switch (bb->ic_type) {
	#ifdef BB_8852A_2_SUPPORT
	case BB_RTL8852A:
		name = "8852A(>Bcut)";
		break;
	#endif
	#ifdef BB_8852B_SUPPORT
	case BB_RTL8852B:
		name = "8852B";
		break;
	#endif
	#ifdef BB_8852C_SUPPORT
	case BB_RTL8852C:
		name = "8852C";
		break;
	#endif
	#ifdef BB_8192XB_SUPPORT
	case BB_RTL8192XB:
		name = "8192XB";
		break;
	#endif
	#ifdef BB_8851B_SUPPORT
	case BB_RTL8851B:
		name = "8851B";
		break;
	#endif
	#ifdef BB_1115_SUPPORT
	case BB_RLE1115:
		name = "_1115";
		break;
	#endif
	#ifdef BB_8922A_SUPPORT
	case BB_RTL8922A:
		name = "8922A";
		break;
	#endif
	#ifdef BB_8934A_SUPPORT
	case BB_RTL8934A:
		name = "8934A";
		break;
	#endif

	default:
		BB_WARNING("[%s]\n", __func__);
		break;
	}
	BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used, "  %-25s: RTL%s\n",
		 "IC", name);
}

void halbb_dv_pxp_print_en(struct bb_info *bb, bool en)
{
	bb->bb_dv_pxp_dbg_i.is_print = en;
}

void halbb_dv_pxp_set_chiptop(struct bb_info *bb, enum phl_phy_idx phy_idx, bool en)
{
	u32 chiptop_rstb_bb_func = 0x18600084;
	u32 bbrst_bitmask[2] = {BIT(0), BIT(8)};
	u32 bbrst_val[2] = {BIT(0), BIT(8)};
	u32 val = 0;

	if (!bb->bb_dv_pxp_dbg_i.is_dv_pxp_dbg)
		return;

	if (en)
		val = bbrst_val[phy_idx];
	else
		val = 0;

	BB_TRACE("[DV/PXP]0x3, 0x%08x, 0x%08x, 0x%08x\n", chiptop_rstb_bb_func,
		    bbrst_bitmask[phy_idx], val&bbrst_bitmask[phy_idx]);
}

void halbb_dv_pxp_dbg(struct bb_info *bb, char input[][16], u32 *_used,
			char *output, u32 *_out_len)
{
	u8 prim_sb = 0;
	u32 val[10] = {0};
	u32 cr = 0;
	u32 used = *_used;
	u32 out_len = *_out_len;
	bool rpt = true;
	u16 i = 0, j = 0, k = 0;

	if (_os_strcmp(input[1], "-h") == 0) {
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			 "adv_bb_dm {en}\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			 "fake_ic_type {AX : 1~15, BE : 16~31}\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			 "is_print {en}\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			 "lbk {en} {0:AFE/1:BB} {tx_path} {rx_path} {bw} {phy_idx}\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			 "phy_init {phy_idx}\n");
		BB_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			 "bw {pri_ch} {band} {bw} {phy_idx}\n");
	} else if (_os_strcmp(input[1], "adv_bb_dm") == 0) {
		HALBB_SCAN(input[2], DCMD_DECIMAL, &val[0]);
		bb->adv_bb_dm_en = (bool)val[0];
		bb->bb_dv_pxp_dbg_i.is_dv_pxp_dbg = (bool)!val[0];
	} else if (_os_strcmp(input[1], "fake_ic_type") == 0) {
		HALBB_SCAN(input[2], DCMD_DECIMAL, &val[0]);
		bb->ic_type = BIT(val[0]);
		halbb_print_ic_type(bb, &used, output, &out_len);
		if (bb->ic_type >= BB_RLE1115)
			bb->bb_80211spec = BB_BE_IC;
		else
			bb->bb_80211spec = BB_AX_IC;
	} else if (_os_strcmp(input[1], "is_print") == 0) {
		HALBB_SCAN(input[2], DCMD_DECIMAL, &val[0]);
		halbb_dv_pxp_print_en(bb, (bool)val[0]);
	} else if (_os_strcmp(input[1], "lbk") == 0) {
		HALBB_SCAN(input[2], DCMD_DECIMAL, &val[0]);
		HALBB_SCAN(input[3], DCMD_DECIMAL, &val[1]);
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[2]);
		HALBB_SCAN(input[5], DCMD_DECIMAL, &val[3]);
		HALBB_SCAN(input[6], DCMD_DECIMAL, &val[4]);
		HALBB_SCAN(input[7], DCMD_DECIMAL, &val[5]);
		halbb_dv_pxp_print_en(bb, true);
		halbb_dv_pxp_set_chiptop(bb, HW_PHY_0, false);
		halbb_dv_pxp_set_chiptop(bb, HW_PHY_1, false);
		rpt = halbb_cfg_lbk(bb, (bool)val[0], (bool)val[1], (enum rf_path)val[2],
				    (enum rf_path)val[3], (enum channel_width)val[4],
				    (enum phl_phy_idx)val[5]);
		halbb_dv_pxp_set_chiptop(bb, HW_PHY_0, true);
		halbb_dv_pxp_set_chiptop(bb, HW_PHY_1, true);
		halbb_dv_pxp_print_en(bb, false);
	} else if (_os_strcmp(input[1], "phy_init") == 0) {
		HALBB_SCAN(input[2], DCMD_DECIMAL, &val[0]);
		halbb_dv_pxp_print_en(bb, true);
		halbb_dv_pxp_set_chiptop(bb, HW_PHY_0, false);
		rpt = halbb_init_cr_default(bb, false, 0, &val[0], (enum phl_phy_idx)val[0]);
		halbb_dv_pxp_set_chiptop(bb, HW_PHY_0, true);
		halbb_dv_pxp_print_en(bb, false);
	} else if (_os_strcmp(input[1], "bw") == 0) {
		HALBB_SCAN(input[2], DCMD_DECIMAL, &val[0]);
		HALBB_SCAN(input[3], DCMD_DECIMAL, &val[1]);
		HALBB_SCAN(input[4], DCMD_DECIMAL, &val[2]);
		HALBB_SCAN(input[5], DCMD_DECIMAL, &val[3]);
		halbb_dv_pxp_print_en(bb, true);
		halbb_dv_pxp_set_chiptop(bb, (enum phl_phy_idx)val[3], false);
		if (bb->bb_80211spec == BB_BE_IC)
			prim_sb = halbb_get_prim_sb (bb, bb->bb_ch_i.fc_ch_idx,(u8)val[0],
						      (enum channel_width)val[2]);
		else
			prim_sb = (u8)val[0];
		rpt = halbb_ctrl_bw(bb, prim_sb, (enum band_type)val[1],
				    (enum channel_width)val[2], (enum phl_phy_idx)val[3]);
		halbb_dv_pxp_set_chiptop(bb, (enum phl_phy_idx)val[3], true);
		halbb_dv_pxp_print_en(bb, false);
	}
}

void halbb_dv_pxp_dbg_init(struct bb_info *bb)
{

	bb->bb_dv_pxp_dbg_i.is_dv_pxp_dbg = false;
	bb->bb_dv_pxp_dbg_i.is_print = false;
}
#if 0
void halbb_cr_cfg_dv_dbg_init(struct bb_info *bb)
{
	struct bb_dbg_cr_info *cr = &bb->bb_dbg_i.bb_dbg_cr_i;

	switch (bb->cr_type) {

	#ifdef HALBB_COMPILE_BE0_SERIES
	case BB_BE0:

		break;
	#endif

	#ifdef HALBB_COMPILE_BE1_SERIES
	case BB_BE1:

		break;
	#endif

	#ifdef HALBB_COMPILE_BE2_SERIES
	case BB_BE2:

		break;
	#endif

	default:
		BB_WARNING("[%s] BBCR Hook FAIL!\n", __func__);

		if (bb->bb_dbg_i.cr_fake_init_hook_en) {
			BB_TRACE("[%s] BBCR fake init\n", __func__);
			halbb_cr_hook_fake_init(bb, (u32 *)cr, (sizeof(struct bb_dbg_cr_info) >> 2));
		}

		break;
	}

	if (bb->bb_dbg_i.cr_init_hook_recorder_en) {
		BB_TRACE("[%s] BBCR Hook dump\n", __func__);
		halbb_cr_hook_init_dump(bb, (u32 *)cr, (sizeof(struct bb_dbg_cr_info) >> 2));
	}
}
#endif
#endif

