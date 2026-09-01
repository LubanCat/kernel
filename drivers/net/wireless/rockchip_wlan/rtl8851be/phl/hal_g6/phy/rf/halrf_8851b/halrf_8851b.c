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
#include "../halrf_precomp.h"
#ifdef RF_8851B_SUPPORT
bool halrf_bw_setting_8851b(struct rf_info *rf, enum rf_path path, enum channel_width bw, bool is_dav)
{
	u32 rf_reg18 = 0;
	u32 reg_reg18_addr = 0x0;

	RF_DBG(rf, DBG_RF_RFK, "[RFK]===> %s\n", __func__);	
	if(is_dav)
		reg_reg18_addr =0x18;
	else
		reg_reg18_addr =0x10018;

	rf_reg18 = halrf_rrf(rf, path, reg_reg18_addr, MASKRF);
	/*==== [Error handling] ====*/
	if (rf_reg18 == INVALID_RF_DATA) {		
		RF_DBG(rf, DBG_RF_RFK, "[RFK]Invalid RF_0x18 for Path-%d\n", path);
		return false;
	}
	rf_reg18 &= ~(BIT(11) | BIT(10));
	/*==== [Switch bandwidth] ====*/
	switch (bw) {
	case CHANNEL_WIDTH_5:
	case CHANNEL_WIDTH_10:
	case CHANNEL_WIDTH_20:
		/*RF bandwidth */
		rf_reg18 |= (BIT(11) | BIT(10));
		break;
	case CHANNEL_WIDTH_40:
		/*RF bandwidth */
		rf_reg18 |= BIT(11);
		break;
	case CHANNEL_WIDTH_80:
		/*RF bandwidth */
		rf_reg18 |= BIT(10);
		break;
	default:
		RF_DBG(rf, DBG_RF_RFK, "[RFK]Fail to set CH\n");
	}

	/*==== [Write RF register] ====*/
	
	rf_reg18 = (u32)(rf_reg18 & 0xf0fff) | BIT(12);

	halrf_wrf(rf, path, reg_reg18_addr, MASKRF, rf_reg18);
	RF_DBG(rf, DBG_RF_RFK, "[RFK] set %x at path%d, %x =0x%x\n",bw, path, reg_reg18_addr, halrf_rrf(rf, path, reg_reg18_addr, MASKRF));
	return true;
}

bool halrf_ctrl_bw_8851b(struct rf_info *rf, enum channel_width bw)
{
	bool is_dav;
	//RF_DBG(rf, DBG_RF_RFK, "[RFK]===> %s\n", __func__);

	/*==== Error handling ====*/
	if (bw >= CHANNEL_WIDTH_MAX ) {
		RF_DBG(rf, DBG_RF_RFK,"[RFK]Fail to switch bw(bw:%d)\n", bw);
		return false;
	}

	//DAV
	is_dav = true;
	halrf_bw_setting_8851b(rf, RF_PATH_A, bw, is_dav);
//	halrf_bw_setting_8851b(rf, RF_PATH_B, bw, is_dav);
	//DDV	
	is_dav = false;
	halrf_bw_setting_8851b(rf, RF_PATH_A, bw, is_dav);	
//	halrf_bw_setting_8851b(rf, RF_PATH_B, bw, is_dav);

	halrf_dack_reset_8851b(rf, RF_PATH_A);

	//RF_DBG(rf, DBG_RF_RFK, "[RFK] BW: %d\n", bw);
	//RF_DBG(rf, DBG_RF_RFK, "[RFK] 0x18 = 0x%x\n",halrf_rrf(rf, RF_PATH_A, 0x18, MASKRF));

	return true;
}

	
bool halrf_set_s0_arfc18_8851b(struct rf_info *rf, u32 val)
{
	u32 c = 1000;
	bool timeout = false;

	halrf_write_fwofld_start(rf);		/*FW Offload Start*/

	halrf_wrf(rf, RF_PATH_A, 0x18, MASKRF, val);
#ifdef HALRF_CONFIG_FW_IO_OFLD_SUPPORT
	if (rf->phl_com->dev_cap.io_ofld) {
		if (!halrf_polling_rf(rf, RF_PATH_A, 0xb7, BIT(8), 0x0, c)) {
			timeout = true;
			RF_WARNING("[LCK]LCK timeout\n");
		}
	}
#else 
	c = 0;
	while (c < 1000) {
		if (halrf_rrf(rf, RF_PATH_A, 0xb7, BIT(8)) == 0)
			break;
		c++;
		halrf_delay_us(rf, 1);
	}

	if (c == 1000) {
		timeout = true;
		RF_WARNING("[LCK]LCK timeout\n");
	}
#endif

	halrf_write_fwofld_end(rf);		/*FW Offload End*/

	return timeout;
}

void halrf_lck_check_8851b(struct rf_info *rf)
{

}

void halrf_set_ch_8851b(struct rf_info *rf, u32 val) {

	halrf_set_s0_arfc18_8851b(rf, val);
}

bool halrf_ch_setting_8851b(struct rf_info *rf,   enum rf_path path, u8 central_ch,
			    bool *is_2g_ch, bool is_dav)
{
	u32 rf_reg18 = 0;	
	u32 reg_reg18_addr = 0x0;

	RF_DBG(rf, DBG_RF_RFK, "[RFK]===> %s\n", __func__);
	
	if(is_dav)
		reg_reg18_addr = 0x18;
	else
		reg_reg18_addr =0x10018;

	rf_reg18 = halrf_rrf(rf, path, reg_reg18_addr, MASKRF);
	/*==== [Error handling] ====*/
	if (rf_reg18 == INVALID_RF_DATA) {
		RF_DBG(rf, DBG_RF_RFK, "[RFK]Invalid RF_0x18 for Path-%d\n", path);
		return false;
	}
	*is_2g_ch = (central_ch <= 14) ? true : false;
	/*==== [Set RF Reg 0x18] ====*/
	rf_reg18 &= ~0x3e3ff; /*[17:16],[9:8],[7:0]*/
	rf_reg18 |= central_ch; /* Channel*/
	/*==== [5G Setting] ====*/
	if (!*is_2g_ch)
		rf_reg18 |= (BIT(16) | BIT(8));

	rf_reg18 = (u32)(rf_reg18 & 0xf0fff) | BIT(12);	
	if ((path == RF_PATH_B) || !(is_dav))
		halrf_wrf(rf, path, reg_reg18_addr, MASKRF, rf_reg18);
	else
		halrf_set_ch_8851b(rf, rf_reg18);
//	halrf_delay_us(rf, 100);
	halrf_wrf(rf, path, 0xcf, BIT(0), 0);
	halrf_wrf(rf, path, 0xcf, BIT(0), 1);
	RF_DBG(rf, DBG_RF_RFK, "[RFK]CH: %d for Path-%d, reg0x%x = 0x%x\n", central_ch, path, reg_reg18_addr, halrf_rrf(rf, path, reg_reg18_addr, MASKRF));
	return true;
}

bool halrf_ctrl_ch_8851b(struct rf_info *rf,  u8 central_ch)
{
	bool is_2g_ch;
	bool is_dav;

	//RF_DBG(rf, DBG_RF_RFK, "[RFK]===> %s\n", __func__);

	/*==== Error handling ====*/
	if ((central_ch > 14 && central_ch < 36) ||
		    (central_ch > 64 && central_ch < 100) ||
		    (central_ch > 144 && central_ch < 149) ||
		central_ch > 177 ) {
		RF_DBG(rf, DBG_RF_RFK, "[RFK]Invalid CH:%d \n", central_ch);
		return false;
	}
	//DAV
	is_dav = true;
	halrf_ch_setting_8851b(rf, RF_PATH_A, central_ch, &is_2g_ch, is_dav);
	//DDV
	is_dav = false;
	halrf_ch_setting_8851b(rf, RF_PATH_A, central_ch, &is_2g_ch, is_dav);	
	//RF_DBG(rf, DBG_RF_RFK, "[RFK] CH: %d\n", central_ch);
	return true;
}

void halrf_set_lo_8851b(struct rf_info *rf, bool is_on, enum rf_path path)
{
	if (is_on) {
		halrf_rf_direct_cntrl_8851b(rf, path, false);
		halrf_wrf(rf, path, 0x0, MASKRFMODE, 0x2);
		halrf_wrf(rf, path, 0x58, BIT(1), 0x1);
		halrf_wrf(rf, path, 0xde, 0x1800, 0x3);
		halrf_wrf(rf, path, 0x56, 0x1c00, 0x1);
		halrf_wrf(rf, path, 0x56, 0x1e0, 0x1);
	} else {
		halrf_wrf(rf, path, 0x58, BIT(1), 0x0);
		halrf_rf_direct_cntrl_8851b(rf, path, true);
		halrf_wrf(rf, path, 0xde, 0x1800, 0x0);
	}
}

void halrf_rf_direct_cntrl_8851b(struct rf_info *rf, enum rf_path path, bool is_bybb)
{
	if (is_bybb)
		halrf_wrf(rf, path, 0x5, BIT(0), 0x1);
	else
		halrf_wrf(rf, path, 0x5, BIT(0), 0x0);
}

void halrf_drf_direct_cntrl_8851b(struct rf_info *rf, enum rf_path path, bool is_bybb)
{
	if (is_bybb)
		halrf_wrf(rf, path, 0x10005, BIT(0), 0x1);
	else
		halrf_wrf(rf, path, 0x10005, BIT(0), 0x0);
}


void halrf_lo_test_8851b(struct rf_info *rf, bool is_on, enum rf_path path)
{
	switch (path) {
		case RF_PATH_A:
			halrf_set_lo_8851b(rf, is_on, RF_PATH_A);
			halrf_set_lo_8851b(rf, false, RF_PATH_B);
			break;
		case RF_PATH_B:
			halrf_set_lo_8851b(rf, false, RF_PATH_A);
			halrf_set_lo_8851b(rf, is_on, RF_PATH_B);
			break;
		case RF_PATH_AB:
			halrf_set_lo_8851b(rf, is_on, RF_PATH_A);
			halrf_set_lo_8851b(rf, is_on, RF_PATH_B);
			break;
		default:
			break;
	}
}

u8 halrf_kpath_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx) {

	return RF_A;
}

void _rx_dck_info_8851b(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path, bool is_afe)
{
	struct halrf_rx_dck_info *rx_dck = &rf->rx_dck;

	rx_dck->is_afe = is_afe;
	rx_dck->loc[path].cur_band = rf->hal_com->band[phy].cur_chandef.band;
	rx_dck->loc[path].cur_bw = rf->hal_com->band[phy].cur_chandef.bw;
	rx_dck->loc[path].cur_ch = rf->hal_com->band[phy].cur_chandef.center_ch;

	RF_DBG(rf, DBG_RF_RXDCK, "[RX_DCK] ==== S%d RX DCK (%s / CH%d / %s / by %s)====\n", path,
		rx_dck->loc[path].cur_band == 0 ? "2G" :
		(rx_dck->loc[path].cur_band == 1 ? "5G" : "6G"),
		rx_dck->loc[path].cur_ch,
	       rx_dck->loc[path].cur_bw == 0 ? "20M" :
	       (rx_dck->loc[path].cur_bw == 1 ? "40M" : "80M"),
	       	rx_dck->is_afe ? "AFE" : "RFC");	
}

void halrf_rxbb_ofst_swap_8851b(struct rf_info *rf, enum rf_path path, u8 rf_mode)
{
	u32 val, val_i, val_q;

	val_i = halrf_rrf(rf, path, 0x92, 0xF0000); /*[19:16]*/
	val_q = halrf_rrf(rf, path, 0x93, 0xF0000); /*[19:16]*/

	val = val_q << 4 | val_i; /*0x93[19:16] = 0x3f[7:4]*/

	halrf_wrf(rf, path, 0xee, BIT(6), 0x1);
	halrf_wrf(rf, path, 0x33, MASKRF, rf_mode);
	halrf_wrf(rf, path, 0x3f, MASKRF, val);
	halrf_wrf(rf, path, 0xee, BIT(6), 0x0);

	RF_DBG(rf, DBG_RF_RXDCK, "[RX_DCK] val_i = 0x%x, val_q = 0x%x, 0x3F = 0x%x\n",
		val_i, val_q, val);
}

void halrf_set_rx_dck_8851b(struct rf_info *rf, enum rf_path path, u8 rf_mode)
{
	u32 cnt = 0;
#if 0
		halrf_btc_rfk_ntfy(rf, phy_map, RF_BTC_RXDCK, RFK_ONESHOT_START);
#endif
	halrf_wrf(rf, path, 0x92, BIT(0), 0x0);
	halrf_wrf(rf, path, 0x92, BIT(0), 0x1);

	while ((halrf_rrf(rf, path, 0x92, BIT(8)) == 0x0) && (cnt < 1000)) {
			halrf_delay_us(rf, 2);
		cnt++;
		}
	
		halrf_wrf(rf, path, 0x92, BIT(0), 0x0);

	RF_DBG(rf, DBG_RF_RXDCK, "[RX_DCK] S%d RXDCK finish (cnt = %d)\n",
		path, cnt);

	halrf_rxbb_ofst_swap_8851b(rf, path, rf_mode);
#if 0
	halrf_btc_rfk_ntfy(rf, phy_map, RF_BTC_RXDCK, RFK_ONESHOT_STOP);
#endif

}

void halrf_rx_dck_8851b(struct rf_info *rf, enum phl_phy_idx phy, bool is_afe) 
{
	u8 path;
	u32 rf_reg5;

	RF_DBG(rf, DBG_RF_RXDCK, "[RX_DCK] ****** RXDCK Start (Ver: 0x%x, Cv: %d) ******\n",
		RXDCK_VER_8851B, rf->hal_com->cv);

	for (path = 0; path < 1; path++) {
		_rx_dck_info_8851b(rf, phy, path, is_afe);

		rf_reg5 = halrf_rrf(rf, path, 0x5, MASKRF);

		if (rf->is_tssi_mode[path]) 
			halrf_wreg(rf, 0x5818 + (path << 13), BIT(30), 0x1); /*TSSI pause*/

		halrf_wrf(rf, path, 0x5, BIT(0), 0x0);

		halrf_wrf(rf, path, 0x00, MASKRFMODE, RF_RX);
		halrf_set_rx_dck_8851b(rf, path, RF_RX);

		halrf_wrf(rf, path, 0x5, MASKRF, rf_reg5);

		if (rf->is_tssi_mode[path])
			halrf_wreg(rf, 0x5818 + (path << 13), BIT(30), 0x0); /*TSSI resume*/

	}
}

void halrf_rx_dck_onoff_8851b(struct rf_info *rf, bool is_enable)
{
	u8 path;

	for (path = 0; path < 1; path++) {
		halrf_wrf(rf, path, 0x93, BIT(0), !is_enable);
		if (!is_enable) {
			halrf_wrf(rf, path, 0x92, 0xFFE00, 0x440); /*[19:9]*/
			halrf_wrf(rf, path, 0x93, 0xFFE00, 0x440); /*[19:9]*/
		}
	}
}

void halrf_rck_8851b(struct rf_info *rf, enum rf_path path)
{
	u8 cnt = 0;
	u32 rf_reg5;
	u32 rck_val = 0;

	RF_DBG(rf, DBG_RF_RFK, "[RCK] ====== S%d RCK ======\n", path);

	rf_reg5 = halrf_rrf(rf, path, 0x5, MASKRF);

	halrf_wrf(rf, path, 0x5, BIT(0), 0x0);
	halrf_wrf(rf, path, 0x0, MASKRFMODE, RF_RX);
	
	RF_DBG(rf, DBG_RF_RFK, "[RCK] RF0x00 = 0x%05x\n", halrf_rrf(rf, path, 0x00, MASKRF));

	/*RCK trigger*/
	halrf_wrf(rf, path, 0x1b, MASKRF, 0x00240);

	while ((halrf_rrf(rf, path, 0x1c, BIT(3)) == 0x00) && (cnt < 10)) {
		halrf_delay_us(rf, 2);
	cnt++;
	}

	rck_val = halrf_rrf(rf, path, 0x1b, 0x07C00); /*[14:10]*/

	RF_DBG(rf, DBG_RF_RFK, "[RCK] rck_val = 0x%x, count = %d\n", rck_val, cnt);

	halrf_wrf(rf, path, 0x1b, MASKRF, rck_val);

	halrf_wrf(rf, path, 0x5, MASKRF, rf_reg5);

	RF_DBG(rf, DBG_RF_RFK, "[RCK] RF 0x1b = 0x%x\n",
	       halrf_rrf(rf, path, 0x1b, MASKRF));
}

void iqk_backup_8851b(struct rf_info *rf, enum rf_path path) 
{
	return;
}

void halrf_bf_config_rf_8851b(struct rf_info *rf)
{
	halrf_wrf(rf, RF_PATH_A, 0xef, BIT(19), 0x1);
	halrf_wrf(rf, RF_PATH_A, 0x33, 0xf, 0x1);
	halrf_wrf(rf, RF_PATH_A, 0x3e, MASKRF, 0x000c6);
	halrf_wrf(rf, RF_PATH_A, 0x3f, MASKRF, 0x00082);
	halrf_wrf(rf, RF_PATH_A, 0x33, 0xf, 0x3);
	halrf_wrf(rf, RF_PATH_A, 0x3e, MASKRF, 0x000c6);
	halrf_wrf(rf, RF_PATH_A, 0x3f, MASKRF, 0x035e7);
	halrf_wrf(rf, RF_PATH_A, 0x33, 0xf, 0xa);
	halrf_wrf(rf, RF_PATH_A, 0x3e, MASKRF, 0x000c6);
	halrf_wrf(rf, RF_PATH_A, 0x3f, MASKRF, 0x035f7);
	halrf_wrf(rf, RF_PATH_A, 0x33, 0xf, 0xb);
	halrf_wrf(rf, RF_PATH_A, 0x3e, MASKRF, 0x000c6);
	halrf_wrf(rf, RF_PATH_A, 0x3f, MASKRF, 0x035ef);
	halrf_wrf(rf, RF_PATH_A, 0xef, BIT(19), 0x0);

	halrf_wrf(rf, RF_PATH_B, 0xef, BIT(19), 0x1);
	halrf_wrf(rf, RF_PATH_B, 0x33, 0xf, 0x1);
	halrf_wrf(rf, RF_PATH_B, 0x3e, MASKRF, 0x00031);
	halrf_wrf(rf, RF_PATH_B, 0x3f, MASKRF, 0x00020);
	halrf_wrf(rf, RF_PATH_B, 0x33, 0xf, 0x3);
	halrf_wrf(rf, RF_PATH_B, 0x3e, MASKRF, 0x00031);
	halrf_wrf(rf, RF_PATH_B, 0x3f, MASKRF, 0x80d79);
	halrf_wrf(rf, RF_PATH_B, 0x33, 0xf, 0xa);
	halrf_wrf(rf, RF_PATH_B, 0x3e, MASKRF, 0x00031);
	halrf_wrf(rf, RF_PATH_B, 0x3f, MASKRF, 0x00d7d);
	halrf_wrf(rf, RF_PATH_B, 0x33, 0xf, 0xb);
	halrf_wrf(rf, RF_PATH_B, 0x3e, MASKRF, 0x00031);
	halrf_wrf(rf, RF_PATH_B, 0x3f, MASKRF, 0x00d7b);
	halrf_wrf(rf, RF_PATH_B, 0xef, BIT(19), 0x0);
}

void halrf_by_rate_dpd_8851b(struct rf_info *rf, enum phl_phy_idx phy)
{
	halrf_mac_set_pwr_reg_8851b(rf, phy, 0xd230 + (phy << 13), 0x0003FFFF, 0x21861); /*[17:0]*/
}

void halrf_set_dpd_backoff_8851b(struct rf_info *rf, enum phl_phy_idx phy)
{
	struct halrf_dpk_info *dpk = &rf->dpk;
	u8 tx_scale, ofdm_bkof, path, kpath;

	kpath = halrf_kpath_8851b(rf, phy);

	ofdm_bkof = (u8)halrf_rreg(rf, 0x44a0 + (phy << 13), 0x0001F000); /*[16:12]*/
	tx_scale = (u8)halrf_rreg(rf, 0x44a0 + (phy << 13), 0x0000007F); /*[6:0]*/

	if ((ofdm_bkof + tx_scale) >= 44) { /*move dpd backoff to bb, and set dpd backoff to 0*/
		dpk->dpk_gs[phy] = 0x7f;
		for (path = 0; path < DPK_RF_PATH_MAX_8851B; path++) {
			if (kpath & BIT(path)) {
				halrf_wreg(rf, 0x81bc + (path << 8), 0x007FFFFF, 0x7f7f7f); /*[22:0]*/
				RF_DBG(rf, DBG_RF_RFK, "[RFK] Set S%d DPD backoff to 0dB\n", path);
			}
		}
	} else
		dpk->dpk_gs[phy] = 0x5b;
}

void halrf_dpk_init_8851b(struct rf_info *rf)
{
	//halrf_by_rate_dpd_8851b(rf, HW_PHY_0);
	//halrf_set_dpd_backoff_8851b(rf, HW_PHY_0);
}

#ifdef HALRF_KIP_CR_CHECK
bool halrf_rfk_reg_check_8851b(struct rf_info *rf)
{
	u32 i, reg, temp;
	bool fail = false;

	for (i = 0; i < KIP_REG; i++) {
		reg = 0x8000 + i*4;
		if (((reg >= 0x8000 && reg < 0x8200) || 
			(reg >= 0x8500 && reg < 0x9100) || 
			(reg >= 0x9500 && reg < 0x9830) ||
			(reg >= 0xa500 && reg < 0xa640) ||
			(reg >= 0xb100 && reg < 0xb1fc) ||
			(reg >= 0xb200 && reg < 0xb240)) &&
			(reg != 0x8014) &&
			(reg != 0x80fc) &&
			(reg != 0x81fc)) {
			temp = halrf_rreg(rf, reg, MASKDWORD);
			if (rf->rfk_reg[i] != temp) {
				RF_DBG(rf, DBG_RF_RFK,
					"[RFK]reg 0x%x b 0x%x/a 0x%x\n",
					reg,
					rf->rfk_reg[i],
					temp);
#if 0
				for (t =0; t < 10; t++) {
					temp = halrf_rreg(rf, reg, MASKDWORD);
					RF_DBG(rf, DBG_RF_RFK, "[RFK]reg 0x%x mismatch: before 0x%x/after 0x%x\n", reg, rf->rfk_reg[i], temp);
					halrf_delay_ms(rf, 1);
				}
#endif
				fail = true;
				rf->rfk_check_fail_count++;
			}
		}
	}
	return fail;
}

void halrf_rfk_reg_reload_8851b(struct rf_info *rf)
{
	u32 i, reg;

#ifdef CF_PHL_BB_CTRL_RX_CCA
	halrf_bb_ctrl_rx_cca(rf, false, HW_PHY_0);
#else
	RF_WARNING("[RF]%s !! no CF_PHL_BB_CTRL_RX_CCA\n", __FUNCTION__);
#endif
	halrf_wreg(rf, 0x0a70, 0x3e0, 0x1f);
	halrf_wreg(rf, 0x8080, MASKDWORD, 0x00000005);
	halrf_wreg(rf, 0x8088, MASKDWORD, 0x80000110);
	halrf_wreg(rf, 0x81d8, MASKDWORD, 0x00010001);
	halrf_wreg(rf, 0x82d8, MASKDWORD, 0x00010001);
	RF_DBG(rf, DBG_RF_RFK, "[RFK]KIP_REG = %d\n", KIP_REG);
	for (i = 0; i < KIP_REG; i++) {
		reg = 0x8000 + i*4;
		halrf_wreg(rf, reg, MASKDWORD, rf->rfk_reg[i]);
	}

	halrf_wreg(rf, 0x81d8, MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x81dc, MASKDWORD, 0x00010003);
	halrf_wreg(rf, 0x81dc, MASKDWORD, 0x00000002);
	halrf_wreg(rf, 0x82d8, MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x82dc, MASKDWORD, 0x00010003);
	halrf_wreg(rf, 0x82dc, MASKDWORD, 0x00000002);
	halrf_wreg(rf, 0x8080, MASKDWORD, 0x00000004);
	halrf_wreg(rf, 0x8080, MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x8088, MASKDWORD, 0x80000000);
	halrf_wreg(rf, 0x0a70, 0x3e0, 0x0);
#ifdef CF_PHL_BB_CTRL_RX_CCA
	halrf_bb_ctrl_rx_cca(rf, true, HW_PHY_0);
#else
	RF_WARNING("[RF]%s !! no CF_PHL_BB_CTRL_RX_CCA\n", __FUNCTION__);
#endif
}

void halrf_rfk_reg_backup_8851b(struct rf_info *rf)
{
	u32 i, reg;

#ifdef CF_PHL_BB_CTRL_RX_CCA
	halrf_bb_ctrl_rx_cca(rf, false, HW_PHY_0);
#else
	RF_WARNING("[RF]%s !! no CF_PHL_BB_CTRL_RX_CCA\n", __FUNCTION__);
#endif
	halrf_wreg(rf, 0x0a70, 0x3e0, 0x1f);
	halrf_wreg(rf, 0x8080, MASKDWORD, 0x00000005);
	halrf_wreg(rf, 0x8088, MASKDWORD, 0x80000110);
	halrf_wreg(rf, 0x81d8, MASKDWORD, 0x00010001);
	halrf_wreg(rf, 0x82d8, MASKDWORD, 0x00010001);
	RF_DBG(rf, DBG_RF_RFK, "[RFK]KIP_REG = %d\n", KIP_REG);
	for (i = 0; i < KIP_REG; i++) {
		reg = 0x8000 + i*4;
		rf->rfk_reg[i] = halrf_rreg(rf, reg, MASKDWORD);
	}

	for (i = 0; i < KIP_REG; i++) {
		reg = 0x8000 + i*4;
		RF_DBG(rf, DBG_RF_RFK,
			"[RFK]bk reg 0x%x = 0x%x\n",
			reg,
			rf->rfk_reg[i]);
	}

	halrf_wreg(rf, 0x81d8, MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x81dc, MASKDWORD, 0x00010003);
	halrf_wreg(rf, 0x81dc, MASKDWORD, 0x00000002);
	halrf_wreg(rf, 0x82d8, MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x82dc, MASKDWORD, 0x00010003);
	halrf_wreg(rf, 0x82dc, MASKDWORD, 0x00000002);
	halrf_wreg(rf, 0x8080, MASKDWORD, 0x00000004);
	halrf_wreg(rf, 0x8080, MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x8088, MASKDWORD, 0x80000000);
	halrf_wreg(rf, 0x0a70, 0x3e0, 0x0);
#ifdef CF_PHL_BB_CTRL_RX_CCA
	halrf_bb_ctrl_rx_cca(rf, true, HW_PHY_0);
#else
	RF_WARNING("[RF]%s !! no CF_PHL_BB_CTRL_RX_CCA\n", __FUNCTION__);
#endif
}

bool halrf_rfk_reg_check_fail_8851b(struct rf_info *rf)
{
	bool fail = false;

#ifdef CF_PHL_BB_CTRL_RX_CCA
	halrf_bb_ctrl_rx_cca(rf, false, HW_PHY_0);
#else
	RF_WARNING("[RF]%s !! no CF_PHL_BB_CTRL_RX_CCA\n", __FUNCTION__);
#endif
	rf->rfk_check_fail_count = 0;
	halrf_wreg(rf, 0x0a70, 0x3e0, 0x1f);
	halrf_wreg(rf, 0x8080, MASKDWORD, 0x00000005);
	halrf_wreg(rf, 0x8088, MASKDWORD, 0x80000110);
	halrf_wreg(rf, 0x81d8, MASKDWORD, 0x00010001);
	halrf_wreg(rf, 0x82d8, MASKDWORD, 0x00010001);
	fail = halrf_rfk_reg_check_8851b(rf);
	halrf_wreg(rf, 0x81d8, MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x81dc, MASKDWORD, 0x00010003);	
	halrf_wreg(rf, 0x81dc, MASKDWORD, 0x00000002);
	halrf_wreg(rf, 0x82d8, MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x82dc, MASKDWORD, 0x00010003);
	halrf_wreg(rf, 0x82dc, MASKDWORD, 0x00000002);
	halrf_wreg(rf, 0x8080, MASKDWORD, 0x00000004);
	halrf_wreg(rf, 0x8080, MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x8088, MASKDWORD, 0x80000000);
	halrf_wreg(rf, 0x0a70, 0x3e0, 0x0);
	RF_DBG(rf, DBG_RF_RFK,
		"[RFK]fail count = %d\n",
		rf->rfk_check_fail_count);
#ifdef CF_PHL_BB_CTRL_RX_CCA
	halrf_bb_ctrl_rx_cca(rf, true, HW_PHY_0);
#else
	RF_WARNING("[RF]%s !! no CF_PHL_BB_CTRL_RX_CCA\n", __FUNCTION__);
#endif
	return fail;
}

static u32 r_d[] = {0x18, 0xb2, 0xc5};
static u32 r_m[] = {0xfffff, 0xfffff, 0x8000};

void halrf_rfc_reg_backup_8851b(struct rf_info *rf)
{
	u8 i;

	for (i = 0; i < 3; i++) {
		rf->rfc_reg[0][i] = halrf_rrf(rf, RF_PATH_A, r_d[i], r_m[i]);
		rf->rfc_reg[1][i] = halrf_rrf(rf, RF_PATH_B, r_d[i], r_m[i]);  
	}
}

bool halrf_rfc_reg_check_fail_8851b(struct rf_info *rf)
{
	u8 i;
	u32 temp;
	bool fail = false;

	for (i = 0; i < 3; i++) {
		temp = halrf_rrf(rf, RF_PATH_A, r_d[i], r_m[i]);
		if (rf->rfc_reg[0][i] !=temp) {
			RF_DBG(rf, DBG_RF_RFK,
				"[RFK]S0 0x%x mask=0x%x, old=0x%x, new=0x%x\n",
				r_d[i],
				r_m[i],
				rf->rfc_reg[0][i],
				temp);
			fail = true;
		}
	}
	for (i = 0; i < 3; i++) {
		temp = halrf_rrf(rf, RF_PATH_B, r_d[i], r_m[i]);
		if (rf->rfc_reg[1][i] !=temp) {
			RF_DBG(rf, DBG_RF_RFK,
				"[RFK]S1 0x%x mask=0x%x, old=0x%x, new=0x%x\n",
				r_d[i],
				r_m[i],
				rf->rfc_reg[1][i],
				temp);
			fail = true;
		}
	}
	return fail;
}
#endif

void halrf_set_rxbb_bw_8851b(struct rf_info *rf, enum channel_width bw, enum rf_path path)
{
	halrf_write_fwofld_start(rf);

	halrf_wrf(rf, path, 0xee, BIT(2), 0x1);
	halrf_wrf(rf, path, 0x33, 0x0001F, 0x12); /*[4:0]*/

	if (bw == CHANNEL_WIDTH_20)
		halrf_wrf(rf, path, 0x3f, 0x0003F, 0x1b); /*[5:0]*/
	else if (bw == CHANNEL_WIDTH_40)
		halrf_wrf(rf, path, 0x3f, 0x0003F, 0x13); /*[5:0]*/
	else if (bw == CHANNEL_WIDTH_80)
		halrf_wrf(rf, path, 0x3f, 0x0003F, 0xb); /*[5:0]*/
	else
		halrf_wrf(rf, path, 0x3f, 0x0003F, 0x3); /*[5:0]*/

	RF_DBG(rf, DBG_RF_RFK, "[RFK] set S%d RXBB BW 0x3F = 0x%x\n", path,
		halrf_rrf(rf, path, 0x3f, 0x0003F));

	halrf_wrf(rf, path, 0xee, BIT(2), 0x0);

	halrf_write_fwofld_end(rf);
}

void halrf_rxbb_bw_8851b(struct rf_info *rf, enum phl_phy_idx phy, enum channel_width bw)
{
	u8 kpath, path;

	kpath = halrf_kpath_8851b(rf, phy);
	
	for (path = 0; path < KPATH; path++) {
		if ((kpath & BIT(path)) && (rf->pre_rxbb_bw[path] != bw)) {
			halrf_set_rxbb_bw_8851b(rf, bw, path);
			rf->pre_rxbb_bw[path] = bw;
		} else
			RF_DBG(rf, DBG_RF_RFK,
			       "[RFK] S%d RXBB BW unchanged (pre_bw = 0x%x)\n",
			       path, rf->pre_rxbb_bw[path]);
	}
}

void halrf_disconnect_notify_8851b(struct rf_info *rf, struct rtw_chan_def *chandef  ) {
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	struct halrf_dbcc_info *dbcc_info = &rf->dbcc_info;
	struct halrf_mcc_info *mcc_info = &rf->mcc_info;
	u8 path, ch;

	RF_DBG(rf, DBG_RF_RFK, "[IQK]===>%s\n", __func__);
	/*[IQK disconnect]*/
	for (ch = 0; ch < 2; ch++) {
		for (path = 0; path < KPATH; path++) {
			if (iqk_info->iqk_mcc_ch[ch][path] == chandef->center_ch)
				iqk_info->iqk_mcc_ch[ch][path] = 0x0;
		}
}

	/*TXGAPK*/
	for (ch = 0; ch < 2; ch++) {
		if (txgapk_info->txgapk_mcc_ch[ch] == chandef->center_ch)
			txgapk_info->txgapk_mcc_ch[ch] = 0x0;
	}

	/*DBCC*/
	for (ch = 0; ch < 2; ch++) {
		for (path = 0; path < 2; path++)
			if (dbcc_info->ch[ch][path] == chandef->center_ch)
				dbcc_info->ch[ch][path]  = 0x0;
	}

	/*MCC*/
	for (ch = 0; ch < 2; ch++) {
		if (mcc_info->ch[ch] == chandef->center_ch)
			mcc_info->ch[ch] = 0x0;
	}
}

#define AACK_REK_8851B 10
void halrf_aack_8851b(struct rf_info *rf)
{
	u32 temp05, tempd3, c, ib[4], i, rek;
	bool timeout;

	RF_DBG(rf, DBG_RF_LCK, "[LCK]DO AACK\n");		
	temp05 = halrf_rrf(rf, 0, 0x5, MASKRF);
	tempd3 = halrf_rrf(rf, 0, 0xd3, MASKRF);
	halrf_wrf(rf, 0, 0x0, MASKRFMODE, 0x3);
	halrf_wrf(rf, 0, 0x5, MASKRF, 0x0);
	halrf_wrf(rf, 0, 0xd3, BIT(4), 0x0);
	for (rek = 0; rek < AACK_REK_8851B; rek ++) {
		halrf_wrf(rf, 0, 0xca, MASKRF, 0x82008);
		halrf_wrf(rf, 0, 0xca, MASKRF, 0x82009);
		halrf_delay_us(rf, 100);
		c = 0;
		while (c < 1000) {
			if (halrf_rrf(rf, RF_PATH_A, 0xc9, BIT(5)) == 0)
				break;
			c++;
			halrf_delay_us(rf, 1);
		}

		if (c == 1000) {
			timeout = true;
			RF_WARNING("[LCK]AACK timeout\n");
		}
		halrf_wrf(rf, 0, 0xb3, BIT(7), 0x1);
		for (i = 0; i < 4; i ++) {
			halrf_wrf(rf, 0, 0xb2, 0x300, i);
			ib[i] = halrf_rrf(rf, 0, 0xc9, 0x1f);
		}
		halrf_wrf(rf, 0, 0xb3, BIT(7), 0x0);

		if (ib[0] == 0 || ib[1] == 0 || ib[2] == 0 || ib[3] == 0) {
			RF_WARNING("[LCK]AACK fail, [%d %d %d %d]", ib[0], ib[1], ib[2], ib[3]);
			for (i = 0; i < 0x100; i ++) {
				RF_WARNING("[LCK]0x%x = 0x%x", i, halrf_rrf(rf, 0x0, i, MASKRF));
			}
		}
		if (ib[0] != 0 && ib[1] != 0 && ib[2] != 0 && ib[3] != 0)
			break;
	}
	if (rek != 0)
		RF_WARNING("[LCK]AACK rek = %d\n", rek);
	if (rek == AACK_REK_8851B){
		RF_WARNING("[LCK]fix AACK val= 5\n");
		halrf_wrf(rf, 0, 0xca, MASKRF, 0x00000);
		halrf_wrf(rf, 0, 0xb2, MASKRF, 0x21400);
	}
	halrf_wrf(rf, 0, 0x5, MASKRF, temp05);
	halrf_wrf(rf, 0, 0xd3, MASKRF, tempd3);
}


void halrf_lc_cal_8851b(struct rf_info *rf)
{
	u32 temp05, temp18, tempd3;

	RF_DBG(rf, DBG_RF_LCK, "[LCK]DO LCK\n");		
	temp05 = halrf_rrf(rf, RF_PATH_A, 0x5, MASKRF);
	temp18 = halrf_rrf(rf, RF_PATH_A, 0x18, MASKRF);
	tempd3 = halrf_rrf(rf, RF_PATH_A, 0xd3, MASKRF);
	halrf_wrf(rf, RF_PATH_A, 0x0, MASKRFMODE, 0x3);
	halrf_wrf(rf, RF_PATH_A, 0x5, MASKRF, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0xd3, BIT(8), 0x1);
	halrf_set_ch_8851b(rf, temp18);
	halrf_wrf(rf, RF_PATH_A, 0xd3, MASKRF, tempd3);
	halrf_wrf(rf, RF_PATH_A, 0x5, MASKRF, temp05);
	rf->lck_ther_s0 = rf->cur_ther_s0;
}

void halrf_lck_8851b(struct rf_info *rf)
{
	halrf_aack_8851b(rf);
	halrf_lc_cal_8851b(rf);
}

void halrf_lck_tracking_8851b(struct rf_info *rf)
{
	u32 temp;

	RF_DBG(rf, DBG_RF_LCK, "[LCK]curT_s0(%d), lckT_s0(%d)\n",
		rf->cur_ther_s0,	rf->lck_ther_s0);

	if (rf->lck_ther_s0 == 0) {
		rf->lck_ther_s0 = rf->cur_ther_s0;
		return;
	}

	temp = HALRF_ABS(rf->lck_ther_s0, rf->cur_ther_s0);
	RF_DBG(rf, DBG_RF_LCK, "[LCK]delta = %d, LCK_TH=%x\n",
		temp, LCK_TH_8851B);

	if(temp >= LCK_TH_8851B)		
		halrf_lck_trigger(rf);
}

bool halrf_check_mcc_ch_8851b(struct rf_info *rf, struct rtw_chan_def *chandef) {

	struct halrf_mcc_info *mcc_info = &rf->mcc_info;
	u8 ch;

	bool check = false;
	RF_DBG(rf, DBG_RF_RFK, "[IQK]===>%s, center_ch(%d)\n", __func__, chandef->center_ch);
	/*[IQK check_mcc_ch]*/
	for (ch = 0; ch < 2; ch++) {
			if (mcc_info->ch[ch] == chandef->center_ch)  {
				check = true;
				return check;
			}
	}
	return check;
}

void halrf_fw_ntfy_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx) {
	struct halrf_mcc_info *mcc_info = &rf->mcc_info;
	u8 i = 0x0;
	u32 data_to_fw[5] = {0};
	u16 len = (u16) (sizeof(data_to_fw) / sizeof(u32))*4;
	
	data_to_fw[0] = (u32) mcc_info->ch[0];
	data_to_fw[1] = (u32) mcc_info->ch[0];
	data_to_fw[2] = (u32) mcc_info->ch[1];
	data_to_fw[3] = (u32) mcc_info->ch[1];
	data_to_fw[4] = rf->hal_com->band[phy_idx].cur_chandef.center_ch;

	RF_DBG(rf, DBG_RF_RFK, "[IQK] len = 0x%x\n", len);
	for (i =0; i < 5; i++)
		RF_DBG(rf, DBG_RF_RFK, "[IQK] data_to_fw[%x] = 0x%x\n", i, data_to_fw[i]);

	halrf_fill_h2c_cmd(rf, len, FWCMD_H2C_GET_MCCCH, 0xa, H2CB_TYPE_DATA, (u32 *) data_to_fw);	

	return;
}

void halrf_before_one_shot_enable_8851b(struct rf_info *rf) {
	
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);

	/* set 0x80d4[21:16]=0x03 (before oneshot NCTL) to get report later */
	halrf_wreg(rf, 0x80d4, 0x003F0000, 0x03);
		
	 RF_DBG(rf, DBG_RF_RFK, "======> before set one-shot bit, 0x%x= 0x%x\n", 0x8010, halrf_rreg(rf, 0x8010, MASKDWORD)); 
}


bool halrf_one_shot_nctl_done_check_8851b(struct rf_info *rf, enum rf_path path) {
	
	/* for check status */
	u32 r_bff8 = 0;
	u32 r_80fc = 0;
	bool is_ready = false;
	u16 count = 1;

	rf->nctl_ck_times[0] = 0;
	rf->nctl_ck_times[1] = 0;
	
	/* for 0xbff8 check NCTL DONE */
	while (count < 2000) {	
		r_bff8 = halrf_rreg(rf, 0xbff8, MASKBYTE0);
				
		if (r_bff8 == 0x55) {
			is_ready = true;
			break;
		}	
		halrf_delay_us(rf, 10);
		count++;
	}
	
	halrf_delay_us(rf, 1);
	/* txgapk_info->txgapk_chk_cnt[path][id][0] = count; */
	rf->nctl_ck_times[0] = count;
	
	 RF_DBG(rf, DBG_RF_RFK, "======> check 0xBFF8[7:0] = 0x%x, IsReady = %d, ReadTimes = %d,delay 1 us\n", r_bff8, is_ready, count); 

	
	/* for 0x80fc check NCTL DONE */	
	count = 1;
	is_ready = false;
	while (count < 2000) {			
		r_80fc = halrf_rreg(rf, 0x80fc, MASKLWORD);
	
		if (r_80fc == 0x8000) {
			is_ready = true;
			break;
		}	
		halrf_delay_us(rf, 1);
		count++;
	}

	halrf_delay_us(rf, 1);
	/* txgapk_info->txgapk_chk_cnt[path][id][1] = count; */
	rf->nctl_ck_times[1] = count;
		
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);

	RF_DBG(rf, DBG_RF_RFK, "======> check 0x80fc[15:0] = 0x%x, IsReady = %d, ReadTimes = %d, 0x%x= 0x%x \n", r_80fc, is_ready, count, 0x8010, halrf_rreg(rf, 0x8010, MASKDWORD) ); 

	return is_ready;
}

void halrf_adc_fifo_rst_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx, u8 path)
{
	halrf_wreg(rf, 0x20fc, 0xffff0000, 0x0101);
	halrf_delay_us(rf, 10);
	halrf_wreg(rf, 0x20fc, 0xffff0000, 0x1111);
}
#ifdef HALRF_EVENT_DUMP_LOG
static u32 check_rfc_reg_8851b[] = {0x9f, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x10000, 0x1, 0x10001,
	0x5, 0x10005, 0x8, 0x18, 0x10018,
	0x2, 0x10002, 0x11, 0x10011, 0x53,
	0x10055,	0x58, 0x63, 0x6e, 0x6f,
	0x7e, 0x7f, 0x80, 0x81, 0x8d,
	0x8f, 0x90, 0x92, 0x93, 0xa0,
	0xb2, 0xc5};
static u32 check_dack_reg_8851b[] = {0x12a0, 0x12b8, 0x030c,
	0x032c, 0xc000, 0xc004, 0xc020, 0xc024,
	0xc0d4, 0xc0f0, 0xc0f4, 0xc05c, 0xc080,
	0xc048, 0xc06c, 0xc060, 0xc084, 0xc040,
	0xc064
	};
static u32 check_iqk_reg_8851b[] = {0x8000, 0x8004, 0x8008, 0x8080, 0x808c,
	0x8120, 0x8124, 0x8138, 0x813c, 0x81dc,
	0x9fe0, 0x9fe4, 0x9fe8, 0x9fec, 0x9f30,
	0x9f40, 0x9f50, 0x9f60, 0x9f70, 0x9f80,
	0x9f90, 0x9fa0};
static u32 check_dpk_reg_8851b[] = {0x80b0, 0x81bc, 0x81b4, 0x81c4,
	0x81c8, 0x58d4
	};
static u32 check_tssi_reg_8851b[] = {0x0304, 0x5818, 0x581c, 0x5820, 0x1c60,
	0x1c44, 0x5838, 0x5630, 0x5634, 0x58f8,
	0x12c0, 0x120c, 0x1c04, 0x1c0c, 0x1c18,
	};
#endif
void halrf_quick_checkrf_8851b(struct rf_info *rf)
{
#ifdef HALRF_EVENT_DUMP_LOG
	u32 path, temp, i;
	u32	len = sizeof(check_rfc_reg_8851b) / sizeof(u32);
	u32	*add = (u32 *)check_rfc_reg_8851b;

	/*check RFC*/
	RF_TRACE("======RFC======\n");
	for (path = 0; path < 1; path++) {
		for (i = 0; i < len; i++ ) {
			temp = halrf_rrf(rf, path, add[i], MASKRF);
			RF_TRACE("RF%d 0x%x = 0x%x\n", path, add[i], temp);
		}
	}
	/*check DACK*/
	RF_TRACE("======DACK======\n");
	len = sizeof(check_dack_reg_8851b) / sizeof(u32);
	add = check_dack_reg_8851b;
	for (i = 0; i < len; i++ ) {
		temp = halrf_rreg(rf, add[i], MASKDWORD);
		RF_TRACE("0x%x = 0x%x\n", add[i], temp);
	}
	/*check IQK*/
	RF_TRACE("======IQK======\n");
	len = sizeof(check_iqk_reg_8851b) / sizeof(u32);
	add = check_iqk_reg_8851b;

	for (i = 0; i < len; i++ ) {
		temp = halrf_rreg(rf, add[i], MASKDWORD);
		RF_TRACE("0x%x = 0x%x\n", add[i], temp);
	}
	/*check DPK*/
	RF_TRACE("======DPK======\n");
	len = sizeof(check_dpk_reg_8851b) / sizeof(u32);
	add = check_dpk_reg_8851b;
	for (i = 0; i < len; i++ ) {
		temp = halrf_rreg(rf, add[i], MASKDWORD);
		RF_TRACE("0x%x = 0x%x\n", add[i], temp);
	}
	/*check TSSI*/
	RF_TRACE("======TSSI======\n");
	len = sizeof(check_tssi_reg_8851b) / sizeof(u32);
	add = check_tssi_reg_8851b;
	for (i = 0; i < len; i++ ) {
		temp = halrf_rreg(rf, add[i], MASKDWORD);
		RF_TRACE("0x%x = 0x%x\n", add[i], temp);
	}
#endif
}

static u32 backup_mac_reg_8851b[] = {0x0};
static u32 backup_bb_reg_8851b[] = {0xc0d4, 0xc0d8, 0xc0c4, 0xc0ec, 0xc0e8};
static u32 backup_rf_reg_8851b[] = {0xef, 0xde, 0x0, 0x1e, 0x2, 0x85, 0x90, 0x5};

#if 1
static struct halrf_iqk_ops iqk_ops= {
    .iqk_kpath = halrf_kpath_8851b,
    .iqk_mcc_page_sel = iqk_mcc_page_sel_8851b,
    .iqk_get_ch_info = iqk_get_ch_info_8851b,
    .iqk_macbb_setting = iqk_macbb_setting_8851b,
    .iqk_preset = iqk_preset_8851b,
    .iqk_start_iqk = iqk_start_iqk_8851b,
    .iqk_restore = iqk_restore_8851b,
    .iqk_afebb_restore = iqk_afebb_restore_8851b,  
};

struct rfk_iqk_info rf_iqk_hwspec_8851b = {
  	.rf_iqk_ops = &iqk_ops,
	.rf_max_path_num = 1,
	.rf_iqk_version = iqk_version_8851b,
	.rf_iqk_ch_num = 1,
	.rf_iqk_path_num = 1,

#if 0
	.backup_mac_reg = backup_mac_reg_8851b,
	.backup_mac_reg_num = ARRAY_SIZE(backup_mac_reg_8851b),
#else
	.backup_mac_reg = backup_mac_reg_8851b,
	.backup_mac_reg_num = 0,
#endif
    .backup_bb_reg = backup_bb_reg_8851b,
    .backup_bb_reg_num = ARRAY_SIZE(backup_bb_reg_8851b),
    .backup_rf_reg = backup_rf_reg_8851b,
    .backup_rf_reg_num = ARRAY_SIZE(backup_rf_reg_8851b),
};

#endif
#endif
