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

#ifndef __iram_func__
#define __iram_func__
#endif



__iram_func__
static void _iqk_read_fft_dbcc0_8851b(struct rf_info *rf, u8 path)
{
	u8 i = 0x0;
	u32 fft[6] = {0x0};

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	halrf_wreg(rf, 0x80d4, MASKDWORD, 0x00160000);
	fft[0] = halrf_rreg(rf, 0x80fc, MASKDWORD);
	halrf_wreg(rf, 0x80d4, MASKDWORD, 0x00170000);
	fft[1] = halrf_rreg(rf, 0x80fc, MASKDWORD);
	halrf_wreg(rf, 0x80d4, MASKDWORD, 0x00180000);
	fft[2] = halrf_rreg(rf, 0x80fc, MASKDWORD);
	halrf_wreg(rf, 0x80d4, MASKDWORD, 0x00190000);
	fft[3] = halrf_rreg(rf, 0x80fc, MASKDWORD);
	halrf_wreg(rf, 0x80d4, MASKDWORD, 0x001a0000);
	fft[4] = halrf_rreg(rf, 0x80fc, MASKDWORD);
	halrf_wreg(rf, 0x80d4, MASKDWORD, 0x001b0000);
	fft[5] = halrf_rreg(rf, 0x80fc, MASKDWORD);
	for(i =0; i< 6; i++)
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x,fft[%x]= %x\n", path, i, fft[i]);

	return;
}

__iram_func__
static void _iqk_read_xym_dbcc0_8851b(struct rf_info *rf, u8 path)
{

	u8 i = 0x0;
	u32 tmp = 0x0;
	u32 bk_reg = 0x0;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	bk_reg = halrf_rreg(rf, 0x8138, MASKDWORD);

	halrf_wreg(rf, 0x8000, 0x00000006, path);
	halrf_wreg(rf, 0x801c, 0x00000003, 0x1);

	for (i = 0x0; i < 0x18; i++) {
		halrf_wreg(rf, 0x8014, MASKDWORD, 0x000000c0 + i);
		halrf_wreg(rf, 0x8014, MASKDWORD, 0x00000000);
		tmp = halrf_rreg(rf, 0x8138 + (path << 8), MASKDWORD);
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, 0x8%x38 = %x\n", path, 1 << path, tmp);
		halrf_delay_us(rf, 1);
	}
	halrf_wreg(rf, 0x801c, 0x00000003, 0x0);
	halrf_wreg(rf, 0x8138+ (path << 8), MASKDWORD, 0x40000000);
	halrf_wreg(rf, 0x8014, MASKDWORD, 0x80010100);
	halrf_delay_us(rf, 1);
	halrf_wreg(rf, 0x8138, MASKDWORD, bk_reg);

	return;
}

__iram_func__
static void _iqk_read_txcfir_dbcc0_8851b(struct rf_info *rf, u8 path, u8 group)
{

	u8 idx = 0x0;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	halrf_wreg(rf, 0x81d8+ (path << 8), MASKDWORD, 0x00000001);
	if (path == 0x0) {
		switch (group) {
		case 0:
			for (idx = 0; idx < 0x0d; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK] %x = %x\n", 0x8f20 + (idx << 2), 
					halrf_rreg(rf, 0x8f20 + (idx << 2), MASKDWORD));
			}
			break;
		case 1:
			for (idx = 0; idx < 0x0d; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK] %x = %x\n", 0x8f54 + (idx << 2),
					halrf_rreg(rf, 0x8f54 + (idx << 2), MASKDWORD));
			}
			break;
		case 2:
			for (idx = 0; idx < 0x0d; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK] %x = %x\n", 0x8f88 + (idx << 2),
					halrf_rreg(rf, 0x8f88 + (idx << 2), MASKDWORD));
			}
			break;
		case 3:
			for (idx = 0; idx < 0x0d; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK] %x = %x\n", 0x8fbc + (idx << 2),
					halrf_rreg(rf, 0x8fbc + (idx << 2), MASKDWORD));
			}
			break;
		default:
			break;
		}
		RF_DBG(rf, DBG_RF_IQK, "[IQK]\n");
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x8f50 = %x\n", halrf_rreg(rf, 0x8f50, MASKDWORD));
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x8f84 = %x\n", halrf_rreg(rf, 0x8f84, MASKDWORD));
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x8fb8 = %x\n", halrf_rreg(rf, 0x8fb8, MASKDWORD));
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x8fec = %x\n", halrf_rreg(rf, 0x8fec, MASKDWORD));
	} else {
		switch (group) {
		case 0:
			for (idx = 0; idx < 0x0d; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK] %x = %x\n", 0x9320 + (idx << 2),
					halrf_rreg(rf, 0x9320 + (idx << 2), MASKDWORD));
			}
			break;
		case 1:
			for (idx = 0; idx < 0x0d; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK] %x = %x\n", 0x9354 + (idx << 2),
					halrf_rreg(rf, 0x9354 + (idx << 2), MASKDWORD));
			}
			break;
		case 2:
			for (idx = 0; idx < 0x0d; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK] %x = %x\n", 0x9388 + (idx << 2),
					halrf_rreg(rf, 0x9388 + (idx << 2), MASKDWORD));
			}
			break;
		case 3:
			for (idx = 0; idx < 0x0d; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK] %x = %x\n", 0x93bc + (idx << 2),
					halrf_rreg(rf, 0x93bc + (idx << 2), MASKDWORD));
			}
			break;
		default:
			break;
		}
		RF_DBG(rf, DBG_RF_IQK, "[IQK]\n");
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x9350 = %x\n", halrf_rreg(rf, 0x9350, MASKDWORD));
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x9384 = %x\n", halrf_rreg(rf, 0x9384, MASKDWORD));
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x93b8 = %x\n", halrf_rreg(rf, 0x93b8, MASKDWORD));
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x93ec = %x\n", halrf_rreg(rf, 0x93ec, MASKDWORD));
	}
	halrf_wreg(rf, 0x81d8+ (path << 8), MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x81d4 + (path << 8), 0x003f0000, 0xc);
	halrf_delay_us(rf, 1);


	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, 0x8%xfc = %x\n",
		path, 1 << path, halrf_rreg(rf, 0x81fc + (path << 8), MASKDWORD));

	return;
}

__iram_func__
static void _iqk_read_rxcfir_dbcc0_8851b(struct rf_info *rf, u8 path, u8 group)
{

	u8 idx = 0x0;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	halrf_wreg(rf, 0x81d8 + (path << 8), MASKDWORD, 0x00000001);

	if (path == 0x0) {
		switch (group) {
		case 0:
			for (idx = 0; idx < 0x10; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK]%x = %x\n", 0x8d00 + (idx << 2),
					halrf_rreg(rf, 0x8d00 + (idx << 2), MASKDWORD));
			}
			break;
		case 1:
			for (idx = 0; idx < 0x10; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK]%x = %x\n", 0x8d44 + (idx << 2),
					halrf_rreg(rf, 0x8d44 + (idx << 2), MASKDWORD));
			}
			break;
		case 2:
			for (idx = 0; idx < 0x10; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK]%x = %x\n", 0x8d88 + (idx << 2),
					halrf_rreg(rf, 0x8d88 + (idx << 2), MASKDWORD));
			}
			break;
		case 3:
			for (idx = 0; idx < 0x10; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK]%x = %x\n", 0x8dcc + (idx << 2),
					halrf_rreg(rf, 0x8dcc + (idx << 2), MASKDWORD));
			}
			break;
		default:
			break;
		}
		RF_DBG(rf, DBG_RF_IQK, "[IQK]\n");
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x8d40 = %x\n", halrf_rreg(rf, 0x8d40, MASKDWORD));
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x8d84 = %x\n", halrf_rreg(rf, 0x8d84, MASKDWORD));
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x8dc8 = %x\n", halrf_rreg(rf, 0x8dc8, MASKDWORD));
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x8e0c = %x\n", halrf_rreg(rf, 0x8e0c, MASKDWORD));
	} else {
		switch (group) {
		case 0:
			for (idx = 0; idx < 0x10; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK]%x = %x\n", 0x9100 + (idx << 2),
					halrf_rreg(rf, 0x9100 + (idx << 2), MASKDWORD));
			}
			break;
		case 1:
			for (idx = 0; idx < 0x10; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK]%x = %x\n", 0x9144 + (idx << 2),
					halrf_rreg(rf, 0x9144 + (idx << 2), MASKDWORD));
			}
			break;
		case 2:
			for(idx = 0; idx < 0x10; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK]%x = %x\n", 0x9188 + (idx << 2),
					halrf_rreg(rf, 0x9188 + (idx << 2), MASKDWORD));
			}
			break;
		case 3:
			for (idx = 0; idx < 0x10; idx++) {
				RF_DBG(rf, DBG_RF_IQK, "[IQK]%x = %x\n", 0x91cc + (idx << 2),
					halrf_rreg(rf, 0x91cc + (idx << 2), MASKDWORD));
			}
			break;
		default:
			break;
		}
		RF_DBG(rf, DBG_RF_IQK, "[IQK]\n");
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x9140 = %x\n", halrf_rreg(rf, 0x9140, MASKDWORD));
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x9184 = %x\n", halrf_rreg(rf, 0x9184, MASKDWORD));
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x91c8 = %x\n", halrf_rreg(rf, 0x91c8, MASKDWORD));
		RF_DBG(rf, DBG_RF_IQK, "[IQK] 0x920c = %x\n", halrf_rreg(rf, 0x920c, MASKDWORD));
	}
	halrf_wreg(rf, 0x81d8 + (path << 8), MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x81d4 + (path << 8), 0x003f0000, 0xd);

	
	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, 0x8%xfc = %x\n",
		path, 1 << path, halrf_rreg(rf, 0x81fc + (path << 8), MASKDWORD));

	return;
}

__iram_func__
static void _iqk_sram_8851b(struct rf_info *rf, u8 path)
{
	u32 i = 0x0;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	halrf_wreg(rf, 0x80d4, MASKDWORD, 0x00020000);
	halrf_wreg(rf, 0x8074, MASKDWORD, 0x80000000);
	halrf_wreg(rf, 0x80e8, MASKDWORD, 0x00000080);
	halrf_wreg(rf, 0x80d8, MASKDWORD, 0x00010000);
	halrf_wreg(rf, 0x802c, 0x00000fff, 0x009);

	for (i = 0; i <= 0x9f; i++) {
		halrf_wreg(rf, 0x80d8, MASKDWORD, 0x00010000 + i);
		RF_DBG(rf, DBG_RF_IQK, "[IQK]0x%x\n",
			halrf_rreg(rf, 0x80fc, 0x0fff0000));
	}

	for (i = 0; i <= 0x9f; i++) {
		halrf_wreg(rf, 0x80d8, MASKDWORD, 0x00010000 + i);
		RF_DBG(rf, DBG_RF_IQK, "[IQK]0x%x\n",
			halrf_rreg(rf, 0x80fc, 0x00000fff));
	}
	halrf_wreg(rf, 0x80e8, MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x80d8, MASKDWORD, 0x00000000);

	return;
}

__iram_func__
static void _iqk_rxk_setting_8851b(struct rf_info *rf, u8 path)
{
	//struct halrf_iqk_info *iqk_info = &rf->iqk;

	//RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	//halrf_wrf(rf, path, 0x8f, 0x01000, 0x1);
	halrf_wrf(rf, path, 0x00, 0xf0000, 0xc);
	//1E[19]=POW_IQKPLL
	halrf_wrf(rf, path, 0x1e, 0x80000, 0x0);
	halrf_wrf(rf, path, 0x1e, 0x80000, 0x1);
	
	return;
}

__iram_func__
static bool _iqk_check_cal_8851b(struct rf_info *rf, u8 path, u8 ktype)
{

//struct halrf_iqk_info *iqk_info = &rf->iqk;
	bool notready = true, fail = true;
	u32 cnt1 = 0x0;
	u32 cnt2 = 0x0;

	//RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);

	while (notready) {
		if (halrf_rreg(rf, 0xbff8, MASKBYTE0) == 0x55) {
			halrf_delay_us(rf, 10);
			notready = false;
		} else {
			halrf_delay_us(rf, 10);
			cnt1++;
		}
		if (cnt1 > 820) {
			fail = true;
			RF_DBG(rf, DBG_RF_IQK, "[IQK]NCTL1 IQK timeout!!!\n");
			break;
		}
	}

	notready = true;

	while (notready) {
		if (halrf_rreg(rf, 0x80fc, 0x0000ffff) == 0x8000) {
			halrf_delay_us(rf, 10);
			notready = false;
		} else {
			halrf_delay_us(rf, 10);
			cnt2++;
		}
		if (cnt2 > 20) {
			fail = true;
			RF_DBG(rf, DBG_RF_IQK, "[IQK]NCTL2 IQK timeout!!!\n");
			break;
		}
	}

	halrf_wreg(rf, 0x8010, MASKBYTE0, 0x0);

	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, type= %x, cnt1= %d, cnt2= %d, notready = %x\n", path, ktype, cnt1, cnt2, notready);
	return notready;
}

__iram_func__
static bool _iqk_one_shot_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
				 u8 path, u8 ktype)
{

	struct halrf_iqk_info *iqk_info = &rf->iqk;
	bool notready = false;
	u32 iqk_cmd = 0x0;
	u32	reg_rfc_ctl = 0x0;
	u32 mask_rfc_ctl = 0x0;

	reg_rfc_ctl = 0x5670;
	mask_rfc_ctl = 0x00000002; 

	//RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	//halrf_btc_rfk_ntfy(rf, phy_map, RF_BTC_IQK, RFK_ONESHOT_START);
	switch (ktype) {
	case ID_TXAGC:
		//RF_DBG(rf, DBG_RF_IQK, "[IQK]============ S%d TXAGC ============\n", path);
		//iqk_cmd = 0x008 | (1 << (4 + path)) | (path << 1);
		break;
	case ID_A_FLoK_coarse:
		RF_DBG(rf, DBG_RF_IQK, "[IQK]============ S%d ID_A_FLoK_coarse ============\n", path);
		halrf_wreg(rf, reg_rfc_ctl, mask_rfc_ctl, 0x1);
		iqk_cmd = 0x108 | (1 << (4 + path));
		break;	
	case ID_G_FLoK_coarse:
		RF_DBG(rf, DBG_RF_IQK, "[IQK]============ S%d ID_G_FLoK_coarse ============\n", path);
		halrf_wreg(rf, reg_rfc_ctl, mask_rfc_ctl, 0x1);
		iqk_cmd = 0x108 | (1 << (4 + path));
		break;
	case ID_A_FLoK_fine:
		RF_DBG(rf, DBG_RF_IQK, "[IQK]============ S%d ID_A_FLoK_fine ============\n", path);
		halrf_wreg(rf, reg_rfc_ctl, mask_rfc_ctl, 0x1);
		iqk_cmd = 0x308 | (1 << (4 + path));
		break;	
	case ID_G_FLoK_fine:
		RF_DBG(rf, DBG_RF_IQK, "[IQK]============ S%d ID_G_FLoK_fine ============\n", path);
		halrf_wreg(rf, reg_rfc_ctl, mask_rfc_ctl, 0x1);
		iqk_cmd = 0x308 | (1 << (4 + path));
		break;
	case ID_TXK:
		RF_DBG(rf, DBG_RF_IQK, "[IQK]============ S%d ID_TXK ============\n", path);
		halrf_wreg(rf, reg_rfc_ctl, mask_rfc_ctl, 0x0);
		iqk_cmd = 0x008 | (1 << (path + 4)) | (((0x8 + iqk_info->iqk_bw[path] ) & 0xf) << 8);
		break;
	case ID_RXAGC:
		RF_DBG(rf, DBG_RF_IQK, "[IQK]============ S%d ID_RXAGC ============\n", path);
		halrf_wreg(rf, reg_rfc_ctl, mask_rfc_ctl, 0x1);
		iqk_cmd = 0x708 | (1 << (4 + path)) | (path << 1);
		break;
	case ID_RXK:
		RF_DBG(rf, DBG_RF_IQK, "[IQK]============ S%d ID_RXK ============\n", path);
		halrf_wreg(rf, reg_rfc_ctl, mask_rfc_ctl, 0x1);
		iqk_cmd = 0x008 | (1 << (path + 4)) | (((0xc + iqk_info->iqk_bw[path]) & 0xf) << 8);
		break;
	case ID_NBTXK:
		RF_DBG(rf, DBG_RF_IQK, "[IQK]============ S%d ID_NBTXK ============\n", path);
		halrf_wreg(rf, reg_rfc_ctl, mask_rfc_ctl, 0x0);
		halrf_wreg(rf, 0x802c, 0x00000fff, 0x011);
		iqk_cmd = 0x408 | (1 << (4 + path));
		break;
	case ID_NBRXK:
		RF_DBG(rf, DBG_RF_IQK, "[IQK]============ S%d ID_NBRXK ============\n", path);
		halrf_wreg(rf, reg_rfc_ctl, mask_rfc_ctl, 0x1);
		halrf_wreg(rf, 0x802c, 0x0fff0000, 0x011);
		iqk_cmd = 0x608 | (1 << (4 + path));
		break;
	default:
		return false;
		break;
	}

	halrf_wreg(rf, 0x8000, MASKDWORD, iqk_cmd + 1);
	notready = _iqk_check_cal_8851b(rf, path, ktype);
	if (iqk_info->iqk_xym_en == true)
		_iqk_read_xym_dbcc0_8851b(rf, path);
	if (iqk_info->iqk_fft_en == true)
		_iqk_read_fft_dbcc0_8851b(rf, path);
	if (iqk_info->iqk_sram_en == true)
		if (ktype == ID_NBRXK || ktype == ID_RXK)
			_iqk_sram_8851b(rf, path);

	if (iqk_info->iqk_cfir_en == true) {
		if (ktype == ID_TXK) {
			_iqk_read_txcfir_dbcc0_8851b(rf, path, 0x0);
			_iqk_read_txcfir_dbcc0_8851b(rf, path, 0x1);
			_iqk_read_txcfir_dbcc0_8851b(rf, path, 0x2);
			_iqk_read_txcfir_dbcc0_8851b(rf, path, 0x3);
		} else {
			_iqk_read_rxcfir_dbcc0_8851b(rf, path, 0x0);
			_iqk_read_rxcfir_dbcc0_8851b(rf, path, 0x1);
			_iqk_read_rxcfir_dbcc0_8851b(rf, path, 0x2);
			_iqk_read_rxcfir_dbcc0_8851b(rf, path, 0x3);
		}
	}

	//8. IQK cotrol RFC
	halrf_wreg(rf, reg_rfc_ctl, mask_rfc_ctl, 0x0);
	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, ktype= %x, id = %x, notready = %x\n", path, ktype, iqk_cmd + 1, notready);

	//halrf_btc_rfk_ntfy(rf, phy_map, RF_BTC_IQK, RFK_ONESHOT_STOP);

	return notready;
}
__iram_func__
static bool _rxk_2g_group_sel_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
				  u8 path)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	u8 gp = 0x0;
	u32 rf_0;
	u32 g_idxrxgain[4] = {0x10e, 0x116, 0x28e, 0x296};
	u32 g_idxattc2[4] = {0x0, 0xf, 0x0, 0xf};
	u32 g_idxrxagc[4] = {0x0, 0x1, 0x2, 0x3};
	bool notready = false;
	bool kfail = false;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);


	for (gp = 0; gp < 0x4; gp++) {
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, gp = %x\n", path,gp);
		halrf_wrf(rf, path, 0x00, 0x03ff0, g_idxrxgain[gp]);
		halrf_wrf(rf, path, 0x83, 0x00f00, g_idxattc2[gp]);
		halrf_wreg(rf, 0x8154, 0x00000100, 0x1);
		halrf_wreg(rf, 0x8154, 0x00000008, 0x0);
		halrf_wreg(rf, 0x8154, 0x00000007, gp);
		
		//RX_Auto_Gian
		halrf_wrf(rf, path, 0x1e, MASKRF, 0x80013);
		halrf_delay_us(rf, 10);
		rf_0 = halrf_rrf(rf, path, 0x00, MASKRF);
		halrf_wreg(rf, 0x8024, 0x000fffff, rf_0);
		halrf_wreg(rf, 0x8044, 0x0000e000, g_idxrxagc[gp]); 
		halrf_wreg(rf, 0x802c, 0x0fff0000, 0x11);
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_RXAGC);
		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, RXAGC 0x8008 = 0x%x, rxbb = %x\n", path, halrf_rreg(rf, 0x8008, MASKDWORD), halrf_rrf(rf, path, 0x00, 0x003e0));
		//NBRXK
		halrf_wrf(rf, path, 0x1e, 0x0003f, 0x13);
		halrf_wreg(rf, 0x802c, 0x0fff0000, 0x011);
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_NBRXK);
		iqk_info->nb_rxcfir[path] = halrf_rreg(rf, 0x813c, MASKDWORD) | 0x2;
		//WBIQK
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_RXK);
		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, WBRXK 0x8008 = 0x%x\n", path, halrf_rreg(rf, 0x8008, MASKDWORD));

	}

	if (!notready)
		kfail = (bool)halrf_rreg(rf, 0x8008, BIT(26));
	
	if(kfail) {
		_iqk_read_xym_dbcc0_8851b(rf, path);
		_iqk_sram_8851b(rf, path);
	}

	
	if (kfail) {
		//halrf_wreg(rf, 0x8124, 0x0000000f, 0x0);
		halrf_wreg(rf, 0x813c + (path << 8), MASKDWORD, iqk_info->nb_rxcfir[path] | 0x2);
		iqk_info->is_wb_txiqk[path] = false;
	} else {
		halrf_wreg(rf, 0x813c + (path << 8), MASKDWORD, 0x40000000);
		iqk_info->is_wb_txiqk[path] = true;
	}

	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, kfail = 0x%x, 0x8%x3c = 0x%x\n", path, kfail, 1 << path, iqk_info->nb_rxcfir[path]);
	return kfail;
}


__iram_func__
static bool _rxk_5g_group_sel_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx, u8 path)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	u8 gp = 0;
	u32 rf_0 = 0;
	u32 a_idxrxgain[4] = {0x10C, 0x112, 0x28c, 0x292};
	u32 a_idxattc2[4] = {0xf, 0xf, 0xf, 0xf};
	u32 a_idxrxagc[4] = {0x4, 0x5, 0x6 ,0x7};
	bool notready = false;
	bool kfail = false;
	u32 rf8c = 0;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	halrf_wrf(rf, RF_PATH_A, 0xef, MASKRF, 0x01000);
	halrf_wrf(rf, RF_PATH_A, 0x33, MASKRF, 0x00004);
	halrf_wrf(rf, RF_PATH_A, 0x3f, MASKRF, 0x00017);
	halrf_wrf(rf, RF_PATH_A, 0x33, MASKRF, 0x00005);
	halrf_wrf(rf, RF_PATH_A, 0x3f, MASKRF, 0x00027);
	halrf_wrf(rf, RF_PATH_A, 0xef, MASKRF, 0x00000);

	rf8c = halrf_rrf(rf, RF_PATH_A, 0x8c, 0x00020);
	halrf_wrf(rf, RF_PATH_A, 0x0, 0xf0000, 0xc);
	
	for (gp = 0; gp < 0x4; gp++) {
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, gp = %x\n", path, gp);

		halrf_wrf(rf, RF_PATH_A, 0x00, 0x03ff0, a_idxrxgain[gp]);
		halrf_wrf(rf, RF_PATH_A, 0x8c, 0x0000f, a_idxattc2[gp]);
		halrf_wrf(rf, RF_PATH_A, 0x8c, 0x00020, 0x1);

		halrf_wreg(rf, 0x8154, 0x00000100, 0x1);
		halrf_wreg(rf, 0x8154, 0x00000008, 0x0);
		halrf_wreg(rf, 0x8154, 0x00000007, gp);
		
		//RX_Auto_Gian
		halrf_wrf(rf, path, 0x1e, MASKRF, 0x80013);
		halrf_delay_us(rf, 100);
		rf_0 = halrf_rrf(rf, path, 0x00, MASKRF);
		halrf_wreg(rf, 0x8024, 0x000fffff, rf_0);
		halrf_wreg(rf, 0x8044, 0x0000e000, a_idxrxagc[gp]); 
		halrf_wreg(rf, 0x802c, 0x0fff0000, 0x11);
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_RXAGC);
		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, RXAGC 0x8008 = 0x%x, rxbb = %x\n", path, halrf_rreg(rf, 0x8008, MASKDWORD), halrf_rrf(rf, path, 0x00, 0x003e0));
		//NBRXK
		halrf_wrf(rf, path, 0x1e, 0x0003f, 0x13);
		halrf_wreg(rf, 0x802c, 0x0fff0000, 0x011);
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_NBRXK);
		iqk_info->nb_rxcfir[path] = halrf_rreg(rf, 0x813c, MASKDWORD) | 0x2;
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, NBRXK 0x8008 = 0x%x\n", path, halrf_rreg(rf, 0x8008, MASKDWORD));
		//WBIQK
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_RXK);
		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, WBRXK 0x8008 = 0x%x, 0x813c =%x\n", path, halrf_rreg(rf, 0x8008, MASKDWORD), iqk_info->nb_rxcfir[path]);

	}

	if (!notready)
		kfail = (bool)halrf_rreg(rf, 0x8008, BIT(26));
	
	if(kfail) {
		_iqk_read_xym_dbcc0_8851b(rf, path);
		_iqk_sram_8851b(rf, path);
	}

	if (kfail) {
		halrf_wreg(rf, 0x8124, 0x0000000f, 0x0);
		halrf_wreg(rf, 0x813c + (path << 8), MASKDWORD, iqk_info->nb_rxcfir[path] | 0x2);
		iqk_info->is_wb_txiqk[path] = false;
	} else {
		halrf_wreg(rf, 0x813c + (path << 8), MASKDWORD, 0x40000000);
		iqk_info->is_wb_txiqk[path] = true;
	}

	halrf_wrf(rf, RF_PATH_A, 0x8c, 0x00020, rf8c);
	halrf_wrf(rf, RF_PATH_A, 0xef, MASKRF, 0x01000);
	halrf_wrf(rf, RF_PATH_A, 0x33, MASKRF, 0x00004);
	halrf_wrf(rf, RF_PATH_A, 0x3f, MASKRF, 0x00037);
	halrf_wrf(rf, RF_PATH_A, 0x33, MASKRF, 0x00005);
	halrf_wrf(rf, RF_PATH_A, 0x3f, MASKRF, 0x00027);
	halrf_wrf(rf, RF_PATH_A, 0xef, MASKRF, 0x00000);

	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, kfail = 0x%x, 0x8%x3c = 0x%x\n", path, kfail, 1 << path, iqk_info->nb_rxcfir[path]);
	return kfail;
}

__iram_func__
static bool _iqk_5g_nbrxk_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
			      u8 path)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	u8 gp = 0;
	u32 rf_0 = 0;
	u32 a_idxrxgain[4] = { 0x10C, 0x112, 0x28c, 0x292};
	u32 a_idxattc2[4] = {0xf, 0xf, 0xf, 0xf};
	u32 a_idxrxagc[4] = {0x4, 0x5, 0x6 ,0x7};
	bool notready = false;
	bool kfail = false;
	u32 rf8c = 0;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	halrf_wrf(rf, RF_PATH_A, 0xef, MASKRF, 0x01000);
	halrf_wrf(rf, RF_PATH_A, 0x33, MASKRF, 0x00004);
	halrf_wrf(rf, RF_PATH_A, 0x3f, MASKRF, 0x00017);
	halrf_wrf(rf, RF_PATH_A, 0x33, MASKRF, 0x00005);
	halrf_wrf(rf, RF_PATH_A, 0x3f, MASKRF, 0x00027);
	halrf_wrf(rf, RF_PATH_A, 0xef, MASKRF, 0x00000);

	rf8c = halrf_rrf(rf, RF_PATH_A, 0x8c, 0x00020);
	halrf_wrf(rf, RF_PATH_A, 0x0, 0xf0000, 0xc);

	//for (gp = 0; gp < 0x4; gp++) {
	gp = 2;
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, gp = %x\n", path,gp);

		halrf_wrf(rf, RF_PATH_A, 0x00, 0x03ff0, a_idxrxgain[gp]);
		halrf_wrf(rf, RF_PATH_A, 0x8c, 0x0000f, a_idxattc2[gp]);
		halrf_wrf(rf, RF_PATH_A, 0x8c, 0x00020, 0x1);

		halrf_wreg(rf, 0x8154, 0x00000100, 0x1);
		halrf_wreg(rf, 0x8154, 0x00000008, 0x0);
		halrf_wreg(rf, 0x8154, 0x00000007, gp);
		
		//RX_Auto_Gian
		halrf_wrf(rf, path, 0x1e, MASKRF, 0x80013);
		halrf_delay_us(rf, 100);
		rf_0 = halrf_rrf(rf, path, 0x00, MASKRF);
		halrf_wreg(rf, 0x8024, 0x000fffff, rf_0);
		halrf_wreg(rf, 0x8044, 0x0000e000, a_idxrxagc[gp]); 
		halrf_wreg(rf, 0x802c, 0x0fff0000, 0x11);
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_RXAGC);
		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, RXAGC 0x8008 = 0x%x, rxbb = %x\n", path, halrf_rreg(rf, 0x8008, MASKDWORD), halrf_rrf(rf, path, 0x00, 0x003e0));
		//NBRXK
		halrf_wrf(rf, path, 0x1e, 0x0003f, 0x13);
		halrf_wreg(rf, 0x802c, 0x0fff0000, 0x011);
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_NBRXK);
		iqk_info->nb_rxcfir[path] = halrf_rreg(rf, 0x813c, MASKDWORD) | 0x2;
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, NBRXK 0x8008 = 0x%x\n", path, halrf_rreg(rf, 0x8008, MASKDWORD));
		//WBIQK
		//notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_RXK);
		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, WBRXK 0x8008 = 0x%x, 0x813c =%x\n", path, halrf_rreg(rf, 0x8008, MASKDWORD), iqk_info->nb_rxcfir[path]);

	//}
	
	if (!notready)
		kfail = (bool)halrf_rreg(rf, 0x8008, BIT(26));

	if (kfail) {
		halrf_wreg(rf, 0x8124 + (path << 8), 0x0000000f, 0x0);
		halrf_wreg(rf, 0x813c + (path << 8), MASKDWORD, 0x40000002);
		iqk_info->is_wb_rxiqk[path] = false;
	} else {
		//halrf_wreg(rf, 0x8124 + (path << 8), 0x0000000f, 0x0);
		iqk_info->is_wb_rxiqk[path] = false;
	}

	halrf_wrf(rf, RF_PATH_A, 0x8c, 0x00020, rf8c);
	halrf_wrf(rf, RF_PATH_A, 0xef, MASKRF, 0x01000);
	halrf_wrf(rf, RF_PATH_A, 0x33, MASKRF, 0x00004);
	halrf_wrf(rf, RF_PATH_A, 0x3f, MASKRF, 0x00037);
	halrf_wrf(rf, RF_PATH_A, 0x33, MASKRF, 0x00005);
	halrf_wrf(rf, RF_PATH_A, 0x3f, MASKRF, 0x00027);
	halrf_wrf(rf, RF_PATH_A, 0xef, MASKRF, 0x00000);

	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, kfail = 0x%x, 0x8%x3c = 0x%x\n", path, kfail, 1 << path, iqk_info->nb_rxcfir[path]);
	return kfail;
}


__iram_func__
static bool _iqk_2g_nbrxk_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
			      u8 path)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	u8 gp = 0x0;
	u32 rf_0;
	u32 g_idxrxgain[4] = {0x10e, 0x116, 0x28e, 0x296};
	u32 g_idxattc2[4] = {0x0, 0xf, 0x0, 0xf};
	u32 g_idxrxagc[4] = {0x0, 0x1, 0x2, 0x3};
	bool notready = false;
	bool kfail = false;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);


	//for (gp = 0; gp < 0x4; gp++) {
		gp = 0x3;
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, gp = %x\n", path,gp);
		halrf_wrf(rf, path, 0x00, 0x03ff0, g_idxrxgain[gp]);
		halrf_wrf(rf, path, 0x83, 0x00f00, g_idxattc2[gp]);
		halrf_wreg(rf, 0x8154, 0x00000100, 0x1);
		halrf_wreg(rf, 0x8154, 0x00000008, 0x0);
		halrf_wreg(rf, 0x8154, 0x00000007, gp);
		
		//RX_Auto_Gian
		halrf_wrf(rf, path, 0x1e, MASKRF, 0x80013);
		halrf_delay_us(rf, 10);
		rf_0 = halrf_rrf(rf, path, 0x00, MASKRF);
		halrf_wreg(rf, 0x8024, 0x000fffff, rf_0);
		halrf_wreg(rf, 0x8044, 0x0000e000, g_idxrxagc[gp]); 
		halrf_wreg(rf, 0x802c, 0x0fff0000, 0x11);
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_RXAGC);
		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, RXAGC 0x8008 = 0x%x, rxbb = %x\n", path, halrf_rreg(rf, 0x8008, MASKDWORD), halrf_rrf(rf, path, 0x00, 0x003e0));
		//NBRXK
		//if(gp == 0x3) {
		halrf_wrf(rf, path, 0x1e, 0x0003f, 0x13);
		halrf_wreg(rf, 0x802c, 0x0fff0000, 0x011);
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_NBRXK);
		iqk_info->nb_rxcfir[path] = halrf_rreg(rf, 0x813c, MASKDWORD) | 0x2;
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, NBRXK 0x8008 = 0x%x\n", path, halrf_rreg(rf, 0x8008, MASKDWORD));
		//}
		//WBIQK
		//notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_RXK);
		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, WBRXK 0x8008 = 0x%x\n", path, halrf_rreg(rf, 0x8008, MASKDWORD));

	//}

	if (!notready)
		kfail = (bool)halrf_rreg(rf, 0x8008, BIT(26));

	if (kfail) {
		//halrf_wreg(rf, 0x8124 + (path << 8), 0x0000000f, 0x0);
		halrf_wreg(rf, 0x813c + (path << 8), MASKDWORD, 0x40000002);
		iqk_info->is_wb_rxiqk[path] = false;
	} else {
		//halrf_wreg(rf, 0x8124 + (path << 8), 0x0000000f, 0x0);
		iqk_info->is_wb_rxiqk[path] = false;
	}

	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, kfail = 0x%x, 0x8%x3c = 0x%x\n", path, kfail, 1 << path, iqk_info->nb_rxcfir[path]);
	return kfail;

}

__iram_func__
static void _iqk_rxclk_setting_8851b(struct rf_info *rf, u8 path)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;

	//RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	halrf_wrf(rf, path, 0x8f, 0x01000, 0x1);
	if (iqk_info->iqk_bw[path] == CHANNEL_WIDTH_80) { //BW80
		//05_BW80_DAC_off_ADC160_path0_KS_51B_reg
		halrf_wreg(rf, 0x20fc, 0xffff0000, 0x0101);
		halrf_wreg(rf, 0x5670, 0x00002000, 0x1);
		halrf_wreg(rf, 0x12a0, 0x00080000, 0x1);
		halrf_wreg(rf, 0x12a0, 0x00070000, 0x2);
		halrf_wreg(rf, 0x5670, 0x60000000, 0x1);
		halrf_wreg(rf, 0xc0d4, 0x00000780, 0x8);
		halrf_wreg(rf, 0xc0d4, 0x00007800, 0x2);
		halrf_wreg(rf, 0xc0d4, 0x0c000000, 0x2);
		halrf_wreg(rf, 0xc0d8, 0x000001e0, 0x5);
		halrf_wreg(rf, 0xc0c4, 0x003e0000, 0xf);
		halrf_wreg(rf, 0xc0ec, 0x00006000, 0x0);
		//halrf_wrf(rf, path, 0x8f, 0x01000, 0x1);
		halrf_wreg(rf, 0x12b8, 0x40000000, 0x1);
		halrf_wreg(rf, 0x030c, 0xff000000, 0x0f);
		halrf_wreg(rf, 0x030c, 0xff000000, 0x03);
		halrf_wreg(rf, 0x032c, 0xffff0000, 0x0001);
		halrf_wreg(rf, 0x032c, 0xffff0000, 0x0041);
		halrf_wreg(rf, 0x20fc, 0xffff0000, 0x1101);
	} else {
		//05_BW2040_DAC_off_ADC080_path0_KS_51B_reg
		halrf_wreg(rf, 0x20fc, 0xffff0000, 0x0101);
		halrf_wreg(rf, 0x5670, 0x00002000, 0x1);
		halrf_wreg(rf, 0x12a0, 0x00080000, 0x1);
		halrf_wreg(rf, 0x12a0, 0x00070000, 0x2);
		halrf_wreg(rf, 0x5670, 0x60000000, 0x0);
		halrf_wreg(rf, 0xc0d4, 0x00000780, 0x8);
		halrf_wreg(rf, 0xc0d4, 0x00007800, 0x2);
		halrf_wreg(rf, 0xc0d4, 0x0c000000, 0x2);
		halrf_wreg(rf, 0xc0d8, 0x000001e0, 0x5);
		halrf_wreg(rf, 0xc0c4, 0x003e0000, 0xf);
		halrf_wreg(rf, 0xc0ec, 0x00006000, 0x2);
		//halrf_wrf(rf, path, 0x8f, 0x01000, 0x1);
		halrf_wreg(rf, 0x12b8, 0x40000000, 0x1);
		halrf_wreg(rf, 0x030c, 0xff000000, 0x0f);
		halrf_wreg(rf, 0x030c, 0xff000000, 0x03);
		halrf_wreg(rf, 0x032c, 0xffff0000, 0x0001);
		halrf_wreg(rf, 0x032c, 0xffff0000, 0x0041);
		halrf_wreg(rf, 0x20fc, 0xffff0000, 0x1101);
	}
	
	return;
}

__iram_func__
static bool _txk_5g_group_sel_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
				  u8 path)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	bool notready = false;
	bool kfail = false;
	u8 gp = 0x0;
	u8 a_idx[2] = {2, 3};
	u32 a_power_range[2] = {0x0, 0x0};
	u32 a_track_range[2] = { 0x7, 0x7};
	u32 a_gain_bb[2] = {0x08, 0x0d};
	u32 a_itqt[2] = {0x12, 0x12};
	u32 a_att_smxr[2] = {0x0, 0x2};

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);	

	halrf_wreg(rf, 0x810c, MASKDWORD, 0x33332222);
	for (gp = 0x0; gp < 0x2; gp++) {
		halrf_wrf(rf, path, 0x11, 0x00003, a_power_range[gp]);
		halrf_wrf(rf, path, 0x11, 0x00070, a_track_range[gp]);
		halrf_wrf(rf, path, 0x11, 0x1f000, a_gain_bb[gp]);
		halrf_wrf(rf, path, 0x60, 0x00007, a_att_smxr[gp]);

		halrf_wreg(rf, 0x8154, 0x00000100, 0x1);
		halrf_wreg(rf, 0x8154, 0x00000008, 0x1);
		halrf_wreg(rf, 0x8154, 0x00000004, 0x0);
		halrf_wreg(rf, 0x8154, 0x00000003, a_idx[gp]);
		halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
		halrf_wreg(rf, 0x802c, 0x00000fff, 0x011);
		halrf_wreg(rf, 0x81cc, MASKDWORD, a_itqt[gp]);

		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_NBTXK);
		iqk_info->nb_txcfir[path] = halrf_rreg(rf, 0x8138, MASKDWORD)  | 0x2;

		halrf_wreg(rf, 0x81cc + (path << 8), MASKDWORD, a_itqt[gp]);
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_TXK);

	}

	if (!notready)
		kfail = (bool)halrf_rreg(rf, 0x8008, BIT(26));

	if (kfail) {
		//halrf_wreg(rf, 0x8124, 0x00000f00, 0x0);
		halrf_wreg(rf, 0x8138 + (path << 8), MASKDWORD, iqk_info->nb_txcfir[path] | 0x2);
		iqk_info->is_wb_txiqk[path] = false;
	} else {
		halrf_wreg(rf, 0x8138 + (path << 8), MASKDWORD, 0x40000000);
		iqk_info->is_wb_txiqk[path] = true;
	}
	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, kfail = 0x%x, 0x8%x38 = 0x%x\n", path, kfail, 1 << path, iqk_info->nb_txcfir[path]);
	return kfail;
}

__iram_func__
static bool _txk_2g_group_sel_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
				  u8 path)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	bool notready = false;
	bool kfail = false;
	u8 gp = 0x0;
	
	u32 g_power_range[1] = {0x0};
	u32 g_track_range[1] = {0x6};
	u32 g_gain_bb[1] = {0x10};
	u32 g_itqt[1] = {0x12};

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);	

	//WA for 5G by gainK
	halrf_wreg(rf, 0x810c, MASKDWORD, 0x00000000);	
	for (gp = 0x0; gp < 0x1; gp++) {

		halrf_wrf(rf, path, 0x11, 0x00003, g_power_range[gp]);
		halrf_wrf(rf, path, 0x11, 0x00070, g_track_range[gp]);
		halrf_wrf(rf, path, 0x11, 0x1f000, g_gain_bb[gp]);
	
		halrf_wreg(rf, 0x81cc, MASKDWORD, g_itqt[gp]);
		halrf_wreg(rf, 0x8154, 0x00000100, 0x1);
		halrf_wreg(rf, 0x8154, 0x00000008, 0x1);
		halrf_wreg(rf, 0x8154, 0x00000004, 0x0);
		halrf_wreg(rf, 0x8154, 0x00000003, gp);
		halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
		
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_NBTXK);
		iqk_info->nb_txcfir[path] = halrf_rreg(rf, 0x8138, MASKDWORD)  | 0x2;

		halrf_wreg(rf, 0x81cc + (path << 8), MASKDWORD, g_itqt[gp]);
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_TXK);

	}

	if (!notready)
		kfail = (bool)halrf_rreg(rf, 0x8008, BIT(26));

	if (kfail) {
		//halrf_wreg(rf, 0x8124, 0x00000f00, 0x0);
		halrf_wreg(rf, 0x8138 + (path << 8), MASKDWORD, iqk_info->nb_txcfir[path] | 0x2);
		iqk_info->is_wb_txiqk[path] = false;
	} else {
		halrf_wreg(rf, 0x8138 + (path << 8), MASKDWORD, 0x40000000);
		iqk_info->is_wb_txiqk[path] = true;
	}

	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, kfail = 0x%x, 0x8%x38 = 0x%x\n", path, kfail, 1 << path, iqk_info->nb_txcfir[path]);
	return kfail;
}

__iram_func__
static bool _iqk_5g_nbtxk_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
			      u8 path)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	bool notready = false;
	bool kfail = false;
	u8 gp = 0x0;
	u8 a_idx[2] = {2, 3};
	u32 a_power_range[2] = {0x0, 0x0};
	u32 a_track_range[2] = {0x7, 0x7};
	u32 a_gain_bb[2] = {0x08, 0x0d};
	u32 a_itqt[2] = {0x12, 0x12};
	u32 a_att_smxr[2] = {0x0, 0x2};

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);	
	//for (gp = 0x0; gp < 0x1; gp++) {
		gp = 0;
		halrf_wrf(rf, path, 0x11, 0x00003, a_power_range[gp]);
		halrf_wrf(rf, path, 0x11, 0x00070, a_track_range[gp]);
		halrf_wrf(rf, path, 0x11, 0x1f000, a_gain_bb[gp]);
		halrf_wrf(rf, path, 0x60, 0x00007, a_att_smxr[gp]);

		halrf_wreg(rf, 0x8154, 0x00000100, 0x1);
		halrf_wreg(rf, 0x8154, 0x00000008, 0x1);
		halrf_wreg(rf, 0x8154, 0x00000004, 0x0);
		halrf_wreg(rf, 0x8154, 0x00000003, a_idx[gp]);
		halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
		halrf_wreg(rf, 0x81cc, MASKDWORD, a_itqt[gp]);

		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_NBTXK);
		iqk_info->nb_txcfir[path] = halrf_rreg(rf, 0x8138, MASKDWORD)  | 0x2;

		//halrf_wreg(rf, 0x81cc + (path << 8), MASKDWORD, a_itqt[gp]);
		//notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_TXK);

//	}

	if (!notready)
		kfail = (bool)halrf_rreg(rf, 0x8008, BIT(26));
	
	if (kfail) {
		//halrf_wreg(rf, 0x8124 + (path << 8), 0x0000000f, 0x0);
		halrf_wreg(rf, 0x8138 + (path << 8), MASKDWORD, 0x40000002);
		iqk_info->is_wb_rxiqk[path] = false;
	} else {
		//halrf_wreg(rf, 0x8124 + (path << 8), 0x0000000f, 0x0);
		iqk_info->is_wb_rxiqk[path] = false;
	}


	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, kfail = 0x%x, 0x8%x38 = 0x%x\n", path, kfail, 1 << path, iqk_info->nb_txcfir[path]);
	return kfail;
}

__iram_func__
static bool _iqk_2g_nbtxk_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
			      u8 path)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	bool notready = false;
	bool kfail = false;
	u8 gp = 0x0;
	
	u32 g_power_range[1] = {0x0};
	u32 g_track_range[1] = {0x6};
	u32 g_gain_bb[1] = {0x10};
	u32 g_itqt[1] = {0x12};

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);	
	for (gp = 0x0; gp < 0x1; gp++) {

		halrf_wrf(rf, path, 0x11, 0x00003, g_power_range[gp]);
		halrf_wrf(rf, path, 0x11, 0x00070, g_track_range[gp]);
		halrf_wrf(rf, path, 0x11, 0x1f000, g_gain_bb[gp]);
	
		halrf_wreg(rf, 0x81cc, MASKDWORD, g_itqt[gp]);
		halrf_wreg(rf, 0x8154, 0x00000100, 0x1);
		halrf_wreg(rf, 0x8154, 0x00000008, 0x1);
		halrf_wreg(rf, 0x8154, 0x00000004, 0x0);
		halrf_wreg(rf, 0x8154, 0x00000003, gp);
		halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
		
		notready = _iqk_one_shot_8851b(rf, phy_idx, path, ID_NBTXK);
		iqk_info->nb_txcfir[path] = halrf_rreg(rf, 0x8138 + (path << 8), MASKDWORD)  | 0x2;
	}

	if (!notready)
		kfail = (bool)halrf_rreg(rf, 0x8008, BIT(26));
	
	if (kfail) {
		//halrf_wreg(rf, 0x8124 + (path << 8), 0x0000000f, 0x0);
		halrf_wreg(rf, 0x8138 + (path << 8), MASKDWORD, 0x40000002);
		iqk_info->is_wb_rxiqk[path] = false;
	} else {
		//halrf_wreg(rf, 0x8124 + (path << 8), 0x0000000f, 0x0);
		iqk_info->is_wb_rxiqk[path] = false;
	}

	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, kfail = 0x%x, 0x8%x38 = 0x%x\n", path, kfail, 1 << path, iqk_info->nb_txcfir[path]);
	return kfail;

}


__iram_func__
static void _lok_res_table_8851b(struct rf_info *rf, u8 path, u8 ibias)
{

	struct halrf_iqk_info *iqk_info = &rf->iqk;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	//RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, ibias = %x\n", path, ibias);
	halrf_wrf(rf, path, 0xef, MASKRF, 0x2);
	if (iqk_info->iqk_band[path] == BAND_ON_24G)
		halrf_wrf(rf, path, 0x33, MASKRF, 0x0);
	else
		halrf_wrf(rf, path, 0x33, MASKRF, 0x1);
	halrf_wrf(rf, path, 0x3f, MASKRF, ibias);
	halrf_wrf(rf, path, 0xef, MASKRF, 0x0);
	halrf_wrf(rf, path, 0x7c, BIT(5), 0x1);
#ifndef HALRF_CONFIG_FW_IO_OFLD_SUPPORT
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, 0x7c = %x\n", path, halrf_rrf(rf, path, 0x7c,MASKRF));
#endif
	
	return;
}

__iram_func__
static bool _lok_finetune_check_8851b(struct rf_info *rf, u8 path)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	bool is_fail = false, is_fail1 = false,  is_fail2 = false;
	u32 temp = 0x0;
	u32 core_i = 0x0;
	u32 core_q = 0x0;
	u32 fine_i = 0x0;
	u32 fine_q = 0x0;
	u8 ch = 0x0;
	u32 vbuff_i = 0x0;
	u32 vbuff_q = 0x0;

	//RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	temp = halrf_rrf(rf, path, 0x58, MASKRF);
	core_i = (temp & 0xf8000) >> 15;
	core_q = (temp & 0x07c00) >> 10;
	fine_i = (temp & 0x003c0) >> 6;
	fine_q = (temp & 0x0003c) >> 2;
	ch = ((iqk_info->iqk_times /2) % 2) & 0x1;
	
	if (core_i  < 0x2 || core_i  > 0x1d || core_q < 0x2 || core_q > 0x1d) {
		is_fail1 = true;
	} else {	
		is_fail1 = false;
	}
	iqk_info->lok_idac[ch][path] = temp;
	
	temp = halrf_rrf(rf, path, 0x0a, MASKRF);
	vbuff_i = (temp & 0xfc000) >> 14;
	vbuff_q = (temp & 0x003f0) >> 4;

	if (vbuff_i  < 0x2 || vbuff_i  > 0x3d || vbuff_q < 0x2 || vbuff_q > 0x3d) {
		is_fail2 = true;
	} else {	
		is_fail2 = false;
	}
	iqk_info->lok_vbuf[ch][path] = temp;

	is_fail = is_fail1 |  is_fail2;
	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, lok_idac[%x][%x] = 0x%x\n", path, ch, path, iqk_info->lok_idac[ch][path]);
	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, lok_vbuf[%x][%x] = 0x%x\n", path, ch, path, iqk_info->lok_vbuf[ch][path]);

	return is_fail;
}

__iram_func__
static bool _iqk_2g_lok_one_gain_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
			    u8 path)
{
	bool fail = false;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	
	halrf_wrf(rf, RF_PATH_A, 0x51, 0x80000, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0x51, 0x00800, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0x52, 0x00800, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0x55, 0x0001f, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0xef, 0x00004, 0x1);
	halrf_wrf(rf, RF_PATH_A, 0x33, 0x000ff, 0x00);
	halrf_wrf(rf, RF_PATH_A, 0x0, 0xffff0, 0x403e);
	halrf_wrf(rf, RF_PATH_A, 0x11, 0x00003, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0x11, 0x00070, 0x6);
	halrf_wrf(rf, RF_PATH_A, 0x11, 0x1f000, 0x0c);
	halrf_wreg(rf, 0x5670, 0x00000002, 0x1);
	halrf_wreg(rf, 0x81cc, 0x0000003f, 0x12);
	halrf_wreg(rf, 0x802c, 0x00000fff, 0x021);
	halrf_wreg(rf, 0x8000, MASKDWORD, 0x00000119);
	fail= _iqk_check_cal_8851b(rf, path, lok);
	halrf_wreg(rf, 0x81cc, 0x0000003f, 0x12);
	halrf_wreg(rf, 0x802c, 0x00000fff, 0x021);
	halrf_wreg(rf, 0x8000, MASKDWORD, 0x00000319);
	fail= _iqk_check_cal_8851b(rf, path, lok);
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
	halrf_wreg(rf, 0x5670, 0x00000002, 0x0);
	return fail;
}

__iram_func__
static bool _iqk_5g_lok_one_gain_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
			    u8 path)
{

	bool fail = false;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);

	halrf_wrf(rf, RF_PATH_A, 0x60, 0x00007, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0x55, 0x0001f, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0xef, 0x00004, 0x1);
	halrf_wrf(rf, RF_PATH_A, 0x33, 0x000ff, 0x80);
	halrf_wrf(rf, RF_PATH_A, 0x0, 0xffff0, 0x403e);
	halrf_wrf(rf, RF_PATH_A, 0x11, 0x00003, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0x11, 0x00070, 0x7);
	halrf_wrf(rf, RF_PATH_A, 0x11, 0x1f000, 0x0c);
	halrf_wreg(rf, 0x5670, 0x00000002, 0x1);
	halrf_wreg(rf, 0x81cc, 0x0000003f, 0x12);
	halrf_wreg(rf, 0x802c, 0x00000fff, 0x021);
	halrf_wreg(rf, 0x8000, MASKDWORD, 0x00000119);
	fail= _iqk_check_cal_8851b(rf, path, lok);
	halrf_wreg(rf, 0x81cc, 0x0000003f, 0x12);
	halrf_wreg(rf, 0x802c, 0x00000fff, 0x021);
	halrf_wreg(rf, 0x8000, MASKDWORD, 0x00000319);
	fail= _iqk_check_cal_8851b(rf, path, lok);
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
	halrf_wreg(rf, 0x5670, 0x00000002, 0x0);
	return fail;

}
__iram_func__
static bool _iqk_2g_lok_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
			    u8 path)
{
	bool fail = false;
	u8 i = 0;
	u32 g_txbb[10] = {0x02, 0x06, 0x0a, 0x0c, 0x0e, 0x10, 0x12, 0x14, 0x16, 0x17};
	u32 g_itqt[10] = {0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x12, 0x12, 0x12, 0x1b};
	u32 g_wa[10] = {0x00, 0x04, 0x08, 0x0c, 0x0e, 0x10, 0x12, 0x14, 0x16, 0x17};
	
	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	halrf_wrf(rf, RF_PATH_A, 0xdf, 0x00004, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0x11, 0x00003, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0x11, 0x00070, 0x6);
	for(i = 0; i< 10; i++) {
		halrf_wrf(rf, RF_PATH_A, 0x11, 0x1f000, g_txbb[i]);
		halrf_wrf(rf, RF_PATH_A, 0x33, 0x000ff, g_wa[i]);
		halrf_wreg(rf, 0x5670, 0x00000002, 0x1);
		halrf_wreg(rf, 0x81cc, 0x0000003f, g_itqt[i]);
		halrf_wreg(rf, 0x802c, 0x00000fff, 0x021);
		halrf_wreg(rf, 0x8000, MASKDWORD, 0x00000109 | (1 << (4 + path)));
		fail= _iqk_check_cal_8851b(rf, path, lok);
		halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
		halrf_wreg(rf, 0x81cc, 0x0000003f, g_itqt[i]);
		halrf_wreg(rf, 0x8000, MASKDWORD, 0x00000309 | (1 << (4 + path)));
		fail= _iqk_check_cal_8851b(rf, path, lok);
		halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
		halrf_wreg(rf, 0x5670, 0x00000002, 0x0);
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S0, i = %x, 0x8[19:15] = 0x%x,0x8[09:05] = 0x%x\n", i, halrf_rrf(rf, RF_PATH_A, 0x08, 0xf8000), halrf_rrf(rf, RF_PATH_A, 0x08, 0x003e0));
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S0, i = %x, 0x9[19:16] = 0x%x,0x9[09:06] = 0x%x\n", i, halrf_rrf(rf, RF_PATH_A, 0x09, 0xf0000), halrf_rrf(rf, RF_PATH_A, 0x09, 0x003c0));
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S0, i = %x, 0x58 = %x\n", i, halrf_rrf(rf, RF_PATH_A, 0x58, MASKRF));
	}
	return fail;
}

__iram_func__
static bool _iqk_5g_lok_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
			    u8 path)
{
	bool fail = false;
	u8 i = 0;
	u32 a_txbb[10] = {0x02, 0x06, 0x0a, 0x0c, 0x0e, 0x10, 0x12, 0x14, 0x16, 0x17};
	u32 a_itqt[10] = {0x09, 0x09, 0x09, 0x12, 0x12, 0x12, 0x1b, 0x1b, 0x1b, 0x1b};
	u32 a_wa[10] = {0x80, 0x84, 0x88, 0x8c, 0x8e, 0x90, 0x92, 0x94, 0x96, 0x97};

	
	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	halrf_wrf(rf, RF_PATH_A, 0xdf, 0x00004, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0x11, 0x00003, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0x11, 0x00070, 0x7);
	for(i = 0; i< 10; i++) {
		halrf_wrf(rf, RF_PATH_A, 0x11, 0x1f000, a_txbb[i]);
		halrf_wrf(rf, RF_PATH_A, 0x33, 0x000ff, a_wa[i]);
		halrf_wreg(rf, 0x5670, 0x00000002, 0x1);
		halrf_wreg(rf, 0x81cc, 0x0000003f, a_itqt[i]);
		halrf_wreg(rf, 0x802c, 0x00000fff, 0x021);
		halrf_wreg(rf, 0x8000, MASKDWORD, 0x00000109 | (1 << (4 + path)));
		fail= _iqk_check_cal_8851b(rf, path, lok);
		halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
		halrf_wreg(rf, 0x81cc, 0x0000003f, a_itqt[i]);
		halrf_wreg(rf, 0x802c, 0x00000fff, 0x021);
		halrf_wreg(rf, 0x8000, MASKDWORD, 0x00000309 | (1 << (4 + path)));
		fail = _iqk_check_cal_8851b(rf, path, lok);
		halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
		halrf_wreg(rf, 0x5670, 0x00000002, 0x0);		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S0, i = %x, 0x8[19:15] = 0x%x,0x8[09:05] = 0x%x\n", i, halrf_rrf(rf, RF_PATH_A, 0x08, 0xf8000), halrf_rrf(rf, RF_PATH_A, 0x08, 0x003e0));
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S0, i = %x, 0x9[19:16] = 0x%x,0x9[09:06] = 0x%x\n", i, halrf_rrf(rf, RF_PATH_A, 0x09, 0xf0000), halrf_rrf(rf, RF_PATH_A, 0x09, 0x003c0));
		RF_DBG(rf, DBG_RF_IQK, "[IQK]S0, i = %x, 0x58 = %x\n", i, halrf_rrf(rf, RF_PATH_A, 0x58, MASKRF));
	}
	return fail;
}

__iram_func__
static void _iqk_txk_setting_8851b(struct rf_info *rf, u8 path)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	//u32 tmp = 0x0;

	//TX init gain setting
	/*0/1:G/A*/

	//RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	switch (iqk_info->iqk_band[path]) {
	case BAND_ON_24G:
		//03_G_S0_TxLOK_51B_reg		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]BAND_ON_24G\n");
		halrf_wrf(rf, RF_PATH_A, 0x51, 0x80000, 0x0);
		halrf_wrf(rf, RF_PATH_A, 0x51, 0x00800, 0x0);
		halrf_wrf(rf, RF_PATH_A, 0x52, 0x00800, 0x0);
		halrf_wrf(rf, RF_PATH_A, 0x55, 0x0001f, 0x4);
		halrf_wrf(rf, RF_PATH_A, 0xef, 0x00004, 0x1);
		halrf_wrf(rf, RF_PATH_A, 0x00, 0xffff0, 0x403e);
		halrf_wrf(rf, RF_PATH_A, 0x11, 0x00003, 0x0);
		halrf_wrf(rf, RF_PATH_A, 0x11, 0x00070, 0x6);
		halrf_wrf(rf, RF_PATH_A, 0x11, 0x1f000, 0x10);
		halrf_delay_us(rf, 1);
		break;
	case BAND_ON_5G:
		RF_DBG(rf, DBG_RF_IQK, "[IQK]BAND_ON_5G\n");
		halrf_wrf(rf, RF_PATH_A, 0x60, 0x00007, 0x0);
		halrf_wrf(rf, RF_PATH_A, 0x55, 0x0001f, 0x4);
		halrf_wrf(rf, RF_PATH_A, 0xef, 0x00004, 0x1);
		halrf_wrf(rf, RF_PATH_A, 0x00, 0xffff0, 0x403e);
		halrf_wrf(rf, RF_PATH_A, 0x11, 0x00003, 0x0);
		halrf_wrf(rf, RF_PATH_A, 0x11, 0x00070, 0x7);
		halrf_wrf(rf, RF_PATH_A, 0x11, 0x1f000, 0x7);
		halrf_delay_us(rf, 1);
		break;
	default:
		break;
	}
/*
	tmp = halrf_rrf(rf, path, 0x00, MASKRF);
	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, rf%x = 0x%x\n", path, path, tmp);
*/
	return;
}

__iram_func__
static void _iqk_txclk_setting_8851b(struct rf_info *rf, u8 path)
{

	//RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	//04_CLK_Setting_TxIQK_DAC960MHz_nonDBCC_PHY0_path01_reg
	halrf_wreg(rf, 0x12b8, 0x40000000, 0x1);
	halrf_wreg(rf, 0x32b8, 0x40000000, 0x1);
	halrf_delay_us(rf, 1);
	halrf_wreg(rf, 0x030c, 0xff000000, 0x1f);
	halrf_delay_us(rf, 1);
	halrf_wreg(rf, 0x030c, 0xff000000, 0x13);
	halrf_wreg(rf, 0x032c, 0xffff0000, 0x0001);
	halrf_delay_us(rf, 1);
	halrf_wreg(rf, 0x032c, 0xffff0000, 0x0041);
	return;
}

__iram_func__
static void _iqk_info_iqk_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
				 u8 path)
{
#if 0
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	u32 tmp = 0x0;
	bool flag = 0x0;

	//RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	iqk_info->thermal[path] = halrf_get_thermal_8851b(rf, path);	
	iqk_info->thermal_rek_en = false;
	flag = iqk_info->lok_cor_fail[0][path];
	halrf_wreg(rf, 0x9fe0, BIT(0) << (path * 4), flag);
	flag = iqk_info->lok_fin_fail[0][path];
	halrf_wreg(rf, 0x9fe0, BIT(1) << (path * 4), flag);
	flag = iqk_info->iqk_tx_fail[0][path];
	halrf_wreg(rf, 0x9fe0, BIT(2) << (path * 4), flag);
	flag = iqk_info->iqk_rx_fail[0][path];
	halrf_wreg(rf, 0x9fe0, BIT(3) << (path * 4), flag);

	tmp = halrf_rreg(rf, 0x8124 + (path << 8), MASKDWORD);
	iqk_info->bp_iqkenable[path] = tmp;
	tmp = halrf_rreg(rf, 0x8138 + (path << 8), MASKDWORD);
	iqk_info->bp_txkresult[path] = tmp;
	tmp = halrf_rreg(rf, 0x813c + (path << 8), MASKDWORD);
	iqk_info->bp_rxkresult[path] = tmp;

	halrf_wreg(rf, 0x9fe8, 0x0000ff00, (u8)iqk_info->iqk_times);

	tmp = halrf_rreg(rf, 0x9fe0, 0x0000000f << (path * 4));
	if (tmp != 0x0)
		iqk_info->iqk_fail_cnt++;
	halrf_wreg(rf, 0x9fe8, 0x00ff0000 << (path * 4), iqk_info->iqk_fail_cnt);
	return;
#endif
}

__iram_func__
static void _iqk_by_path_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
				u8 path)
{

	struct halrf_iqk_info *iqk_info = &rf->iqk;
	bool lok_is_fail = false;
	u8 i = 0;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);

	//TXGAPK
	
	if (rf->chlk_map & HAL_RF_TXGAPK)
		if (rf->support_ability & HAL_RF_TXGAPK)
			halrf_do_txgapk_8851b(rf, phy_idx);

	//LOK
	for (i = 0; i < 1; i++) {
		_iqk_txk_setting_8851b(rf, path);
		if (iqk_info->iqk_band[path] == BAND_ON_24G)
			lok_is_fail = _iqk_2g_lok_8851b(rf, phy_idx, path);
			//lok_is_fail = _iqk_2g_lok_one_gain_8851b(rf, phy_idx, path);
		else
			lok_is_fail = _iqk_5g_lok_8851b(rf, phy_idx, path);
			//lok_is_fail = _iqk_5g_lok_one_gain_8851b(rf, phy_idx, path);
		if (!lok_is_fail)
			break;
	}

	//TXK
	if(iqk_info->is_nbiqk) {		
		if (iqk_info->iqk_band[path] == BAND_ON_24G)
			iqk_info->iqk_tx_fail[0][path] = _iqk_2g_nbtxk_8851b(rf, phy_idx, path);
		else
			iqk_info->iqk_tx_fail[0][path] = _iqk_5g_nbtxk_8851b(rf, phy_idx, path);
	} else {
		if (iqk_info->iqk_band[path] == BAND_ON_24G)
			iqk_info->iqk_tx_fail[0][path] = _txk_2g_group_sel_8851b(rf, phy_idx, path);
		else
			iqk_info->iqk_tx_fail[0][path] = _txk_5g_group_sel_8851b(rf, phy_idx, path);
	}
	//RX
	_iqk_rxclk_setting_8851b(rf, path);
	_iqk_rxk_setting_8851b(rf, path);
	halrf_adc_fifo_rst_8851b(rf, phy_idx, path);

	if(iqk_info->is_nbiqk) {		
		if (iqk_info->iqk_band[path] == BAND_ON_24G)
			iqk_info->iqk_rx_fail[0][path] = _iqk_2g_nbrxk_8851b(rf, phy_idx, path);
		else
			iqk_info->iqk_rx_fail[0][path] = _iqk_5g_nbrxk_8851b(rf, phy_idx, path);
	} else {
		if (iqk_info->iqk_band[path] == BAND_ON_24G)
			iqk_info->iqk_rx_fail[0][path] = _rxk_2g_group_sel_8851b(rf, phy_idx, path);
		else
			iqk_info->iqk_rx_fail[0][path] = _rxk_5g_group_sel_8851b(rf, phy_idx, path);
	}

	return;
}

__iram_func__
void iqk_backup_rf0_8851b(
	struct rf_info *rf, u8 path,
	u32 backup_rf0[rf_reg_num_8851b],
	u32 backup_rf_reg0[rf_reg_num_8851b])
{
	u8 i;

	if(path != RF_PATH_A)
		return;
	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	for (i = 0; i < rf_reg_num_8851b; i++) {
		backup_rf0[i] = halrf_rrf(rf, RF_PATH_A, backup_rf_reg0[i], MASKRF);		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]bk rf0, %x = %x\n", backup_rf_reg0[i], backup_rf0[i]);
	}	
	return;
}

__iram_func__
void iqk_backup_rf1_8851b(
	struct rf_info *rf, u8 path,
	u32 backup_rf1[rf_reg_num_8851b],
	u32 backup_rf_reg1[rf_reg_num_8851b])
{
	u8 i;

	if(path != RF_PATH_B)
		return;

	//DBG_LOG_SERIOUS(DBGMSG_RF, DBG_WARNING, "[IQK] 06 \n");
	for (i = 0; i < rf_reg_num_8851b; i++) {
		backup_rf1[i] = halrf_rrf(rf, RF_PATH_B, backup_rf_reg1[i], MASKRF);		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]bk rf1, %x = %x\n", backup_rf_reg1[i], backup_rf1[i]);
	}	
	return;
}

__iram_func__
void iqk_restore_rf0_8851b(
	struct rf_info *rf, u8 path,
	u32 backup_rf0[rf_reg_num_8851b],
	u32 backup_rf_reg0[rf_reg_num_8851b])
{
	u32 i = 0;
	
	if(path != RF_PATH_A)
		return;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);	
	//DBG_LOG_SERIOUS(DBGMSG_RF, DBG_WARNING, "[IQK] 13 \n");
	for (i = 0; i < rf_reg_num_8851b; i++) {
		halrf_wrf(rf, RF_PATH_A, backup_rf_reg0[i], MASKRF, backup_rf0[i]);		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]restore rf0, S%d, 0x%x = %x\n", path, backup_rf_reg0[i], halrf_rrf(rf, RF_PATH_A, backup_rf_reg0[i], MASKRF));
	}
	return;
}

__iram_func__
void iqk_restore_rf1_8851b(
	struct rf_info *rf, u8 path,
	u32 backup_rf1[rf_reg_num_8851b],
	u32 backup_rf_reg1[rf_reg_num_8851b])
{
	u32 i;
	
	if(path != RF_PATH_B)
		return;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	for (i = 0; i < rf_reg_num_8851b; i++) {
		halrf_wrf(rf, RF_PATH_B, backup_rf_reg1[i], MASKRF, backup_rf1[i]);		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]restore rf1, S%d, 0x%x = %x\n", path, backup_rf_reg1[i], halrf_rrf(rf, RF_PATH_B, backup_rf_reg1[i], MASKRF));		
	}
	return;
}


__iram_func__
void iqk_set_info_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx,
				 u8 path)
{
#if 0
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	u32 tmp = 0x0;
	bool flag = 0x0;

	//RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	iqk_info->thermal[path] = halrf_get_thermal_8851b(rf, path);	
	iqk_info->thermal_rek_en = false;

	flag = (bool) halrf_rreg(rf, 0x9fe0, BIT(0) << (path * 4));
	iqk_info->lok_cor_fail[0][path] = flag;

	flag = (bool) halrf_rreg(rf, 0x9fe0, BIT(1) << (path * 4));
	iqk_info->lok_fin_fail[0][path] = flag;

	flag = (bool) halrf_rreg(rf, 0x9fe0, BIT(2) << (path * 4));
	iqk_info->iqk_tx_fail[0][path] = flag;

	flag = (bool) halrf_rreg(rf, 0x9fe0, BIT(3) << (path * 4));
	iqk_info->iqk_rx_fail[0][path] = flag;
	
	tmp = halrf_rreg(rf, 0x8124 + (path << 8), MASKDWORD);
	iqk_info->bp_iqkenable[path] = tmp;

	tmp = halrf_rreg(rf, 0x8138 + (path << 8), MASKDWORD);
	iqk_info->bp_txkresult[path] = tmp;

	tmp = halrf_rreg(rf, 0x813c + (path << 8), MASKDWORD);
	iqk_info->bp_rxkresult[path] = tmp;

	iqk_info->iqk_times = (u8) halrf_rreg(rf, 0x9fe8, 0x0000ff00);
	iqk_info->iqk_fail_cnt = halrf_rreg(rf, 0x9fe8, 0x00ff0000 << (path * 4));
	return;
#endif
}


__iram_func__
void iqk_get_ch_info_8851b(struct rf_info *rf, enum phl_phy_idx phy, u8 path)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
//	u8 ver;
	u8 idx = 0;

	iqk_info->iqk_band[path] = rf->hal_com->band[phy].cur_chandef.band;
	iqk_info->iqk_bw[path] = rf->hal_com->band[phy].cur_chandef.bw;
	iqk_info->iqk_ch[path] = rf->hal_com->band[phy].cur_chandef.center_ch;	
	iqk_info->iqk_mcc_ch[idx][path] = rf->hal_com->band[phy].cur_chandef.center_ch;
	iqk_info->iqk_table_idx[path] =  idx;

	RF_DBG(rf, DBG_RF_IQK, "[IQK] test version 0x11\n");

	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%d (PHY%d): / DBCC %s/ %s/ CH%d/ %s\n",
		   path, phy,  rf->hal_com->dbcc_en ? "on" : "off",
		   iqk_info->iqk_band[path]  == 0 ? "2G" : (iqk_info->iqk_band[path]  == 1 ? "5G" : "6G"),
		   iqk_info->iqk_ch[path] ,
		   iqk_info->iqk_bw[path] == 0 ? "20M" : (iqk_info->iqk_bw[path] == 1 ? "40M" : "80M"));	
	RF_DBG(rf, DBG_RF_IQK, "[IQK]times = 0x%x, ch =%x\n", iqk_info->iqk_times , idx);	
	RF_DBG(rf, DBG_RF_IQK, "[IQK]iqk_mcc_ch[%x][%x] = 0x%x\n",  (u8)idx, (u8)path, iqk_info->iqk_mcc_ch[idx][path]);
	RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, iqk_info->syn1to2= 0x%x\n", path, iqk_info->syn1to2);

	//halrf_wreg(rf, 0x9fe0, 0xff000000, iqk_version_8851b);
	//2G5G6G = 0/1/2
	//halrf_wreg(rf, 0x9fe4, 0x000f << (path * 16), (u8)iqk_info->iqk_band[path]);
	//20/40/80 = 0/1/2
	//halrf_wreg(rf, 0x9fe4, 0x00f0 << (path * 16), (u8)iqk_info->iqk_bw[path]);
	//halrf_wreg(rf, 0x9fe4, 0xff00 << (path * 16), (u8)iqk_info->iqk_ch[path]); 

	//ver = (u8) halrf_get_8851b_nctl_reg_ver();
	//halrf_wreg(rf, 0x9fe8, 0x000000ff, ver);

	return;
}

__iram_func__
void halrf_iqk_reload_8851b(struct rf_info *rf, u8 path)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	u32 tmp;
	//RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	iqk_info->reload_cnt++;
	tmp = iqk_info->bp_iqkenable[path];
	halrf_wreg(rf, 0x8124 + (path << 8), MASKDWORD, tmp);
	tmp = iqk_info->bp_txkresult[path];
	halrf_wreg(rf, 0x8138 + (path << 8), MASKDWORD, tmp);
	tmp = iqk_info->bp_rxkresult[path];
	halrf_wreg(rf, 0x813c + (path << 8), MASKDWORD, tmp);
	return;
}

__iram_func__
void iqk_start_iqk_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx, u8 path)
{

	//RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	
	_iqk_by_path_8851b(rf, phy_idx, path);
	return;
}

__iram_func__
void iqk_restore_8851b(struct rf_info *rf, u8 path)
{
//	struct halrf_iqk_info *iqk_info = &rf->iqk;
	bool fail;

	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);

	//98_S0_IQK_Reg_Restore_51B_20220823_reg
	halrf_wreg(rf, 0x8000, MASKDWORD, 0x00001219);
	halrf_delay_us(rf, 10);
	fail = _iqk_check_cal_8851b(rf, path, 0x0);
	
	
	halrf_wrf(rf, RF_PATH_A, 0xef, 0x00004, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0xdf, 0x01000, 0x0);
	
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
	halrf_wreg(rf, 0x8008, MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x8088, MASKDWORD, 0x80000000);

	return;	
}

__iram_func__
void iqk_afebb_restore_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx, u8 path)
{
	//99_BB_AFE_DPK_S0_restore_20211024_51B
	halrf_wreg(rf, 0x12b8, 0x40000000, 0x0);
	halrf_wreg(rf, 0x20fc, 0x00010000, 0x1);
	halrf_wreg(rf, 0x20fc, 0x00100000, 0x0);
	halrf_wreg(rf, 0x20fc, 0x01000000, 0x1);
	halrf_wreg(rf, 0x20fc, 0x10000000, 0x0);
	halrf_wreg(rf, 0x5670, MASKDWORD, 0x00000000);
	halrf_wreg(rf, 0x12a0, 0x000ff000, 0x00);
	halrf_wreg(rf, 0x20fc, 0x00010000, 0x0);
	halrf_wreg(rf, 0x20fc, 0x01000000, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0x10005, 0x00001, 0x1);

	return;
}

__iram_func__
void iqk_preset_8851b(struct rf_info *rf, u8 path)
{
	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	//02_IQK_Preset_path0_20210804_reg
	halrf_wrf(rf, path, 0x05, 0x00001, 0x0);
	halrf_wreg(rf, 0x8008, MASKDWORD, 0x00000080);
	halrf_wreg(rf, 0x8088, MASKDWORD, 0x81ff010a);
	return;
}

__iram_func__
void iqk_macbb_setting_8851b(struct rf_info *rf, enum phl_phy_idx phy_idx, u8 path)
{
	RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
	//01_BB_AFE_forDPK_S0_20211024_51B
	halrf_wrf(rf, RF_PATH_A, 0x10005, 0x00001, 0x0);
	halrf_wreg(rf, 0x20fc, 0x00010000, 0x1);
	halrf_wreg(rf, 0x20fc, 0x00100000, 0x0);
	halrf_wreg(rf, 0x20fc, 0x01000000, 0x1);
	halrf_wreg(rf, 0x20fc, 0x10000000, 0x0);
	halrf_wreg(rf, 0x5670, MASKDWORD, 0xf801fffd);
	halrf_wreg(rf, 0x5670, 0x00004000, 0x1);
	halrf_wreg(rf, 0x12a0, 0x00008000, 0x1);
	halrf_wreg(rf, 0x5670, 0x80000000, 0x1);
	halrf_wreg(rf, 0x12a0, 0x00007000, 0x7);
	halrf_wreg(rf, 0x5670, 0x00002000, 0x1);
	halrf_wreg(rf, 0x12a0, 0x00080000, 0x1);
	halrf_wreg(rf, 0x12a0, 0x00070000, 0x3);
	halrf_wreg(rf, 0x5670, 0x60000000, 0x2);
	halrf_wreg(rf, 0xc0d4, 0x00000780, 0x9);
	halrf_wreg(rf, 0xc0d4, 0x00007800, 0x1);
	halrf_wreg(rf, 0xc0d4, 0x0c000000, 0x0);
	halrf_wreg(rf, 0xc0d8, 0x000001e0, 0x3);
	halrf_wreg(rf, 0xc0c4, 0x003e0000, 0xa);
	halrf_wreg(rf, 0xc0ec, 0x00006000, 0x0);
	halrf_wreg(rf, 0xc0e8, 0x00000040, 0x1);
#if 1
	halrf_wreg(rf, 0x12b8, 0x40000000, 0x1);
	halrf_delay_us(rf, 2);
	halrf_wreg(rf, 0x030c, 0xff000000, 0x1f);	
	halrf_delay_us(rf, 10);
	halrf_wreg(rf, 0x030c, 0xff000000, 0x13);
	halrf_delay_us(rf, 2);
	halrf_wreg(rf, 0x032c, 0xffff0000, 0x0001);	
	halrf_delay_us(rf, 2);
	halrf_wreg(rf, 0x032c, 0xffff0000, 0x0041);	
	halrf_delay_us(rf, 10);
#endif
#if 1
	halrf_wreg(rf, 0x12b8, 0x40000000, 0x1);
	halrf_delay_us(rf, 2);
	halrf_wreg(rf, 0x030c, 0xff000000, 0x1f);	
	halrf_delay_us(rf, 10);
	halrf_wreg(rf, 0x030c, 0xff000000, 0x13);
	halrf_delay_us(rf, 2);
	halrf_wreg(rf, 0x032c, 0xffff0000, 0x0001);	
	halrf_delay_us(rf, 2);
	halrf_wreg(rf, 0x032c, 0xffff0000, 0x0041);	
	halrf_delay_us(rf, 10);
#endif	
	halrf_wreg(rf, 0x20fc, 0x00100000, 0x1);
	halrf_wreg(rf, 0x20fc, 0x10000000, 0x1);

	return;
}

__iram_func__
void halrf_iqk_toneleakage_8851b(struct rf_info *rf, u8 path)
{
}

__iram_func__
void halrf_iqk_onoff_8851b(struct rf_info *rf, bool is_enable)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;

	if (is_enable) {
		halrf_wreg(rf, 0x8124, MASKDWORD, 0x00000505); /*ch0*/
		halrf_wreg(rf, 0x8224, MASKDWORD, 0x00000505); /*ch1*/
		iqk_info->is_iqk_enable = true;
	} else {
		halrf_wreg(rf, 0x8124, MASKDWORD, 0x00000000); /*ch0*/
		halrf_wreg(rf, 0x8224, MASKDWORD, 0x00000000); /*ch1*/
		iqk_info->is_iqk_enable = false;
	}
	RF_DBG(rf, DBG_RF_IQK, "[IQK] IQK enable :  %s !!!\n",
	       iqk_info->is_iqk_enable ? "enable" : "disable");
	return;
}

__iram_func__
void halrf_iqk_tx_bypass_8851b(struct rf_info *rf, u8 path)
{
	if (path == RF_PATH_A) { //path A
		/*ch0*/
		halrf_wreg(rf, 0x8124, 0x00000f00, 0x0);
		halrf_wreg(rf, 0x8138, MASKDWORD, 0x40000002);
	} else {
		/*ch1*/
		halrf_wreg(rf, 0x8224, 0x00000f00, 0x0);
		halrf_wreg(rf, 0x8238, MASKDWORD, 0x40000002);
	}
	return;
}

__iram_func__
void halrf_iqk_rx_bypass_8851b(struct rf_info *rf, u8 path)
{
	if (path == RF_PATH_A) { //path A
		/*ch0*/
		halrf_wreg(rf, 0x8124, 0x0000000f, 0x0);
		halrf_wreg(rf, 0x813c, MASKDWORD, 0x40000002);
	} else {
		/*ch1*/
		halrf_wreg(rf, 0x8224, 0x0000000f, 0x0);
		halrf_wreg(rf, 0x823c, MASKDWORD, 0x40000002);
	}
	return;
}

__iram_func__
void halrf_iqk_lok_bypass_8851b(struct rf_info *rf, u8 path)
{
	halrf_wrf(rf, path, 0xdf, 0x00004, 0x1);
	halrf_wrf(rf, path, 0x58, MASKRF, 0x84220);
	return;
}

__iram_func__
void halrf_nbiqk_enable_8851b(struct rf_info *rf, bool nbiqk_en)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;

	if (nbiqk_en) {
		iqk_info->is_nbiqk = true;
	} else {
		iqk_info->is_nbiqk = false;
	}

	return;
}

__iram_func__
void halrf_iqk_xym_enable_8851b(struct rf_info *rf, bool iqk_xym_en)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;

	if (iqk_xym_en) {
		iqk_info->iqk_xym_en = true;
	} else {
		iqk_info->iqk_xym_en = false;
	}

	return;
}

__iram_func__
void halrf_iqk_fft_enable_8851b(struct rf_info *rf, bool iqk_fft_en)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;

	if (iqk_fft_en) {
		iqk_info->iqk_fft_en = true;
	} else {
		iqk_info->iqk_fft_en = false;
	}

	return;
}

__iram_func__
void halrf_iqk_sram_enable_8851b(struct rf_info *rf, bool iqk_sram_en)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;

	if (iqk_sram_en) {
		iqk_info->iqk_sram_en = true;
	} else {
		iqk_info->iqk_sram_en = false;
	}

	return;
}

__iram_func__
void halrf_iqk_cfir_enable_8851b(struct rf_info *rf, bool iqk_cfir_en)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;

	if (iqk_cfir_en) {
		iqk_info->iqk_cfir_en = true;
	} else {
		iqk_info->iqk_cfir_en = false;
	}
	return;
}

__iram_func__
void halrf_iqk_track_8851b(
	struct rf_info *rf)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	u8 path =0x0, i = 0x0;
	u8 cur_ther = 0x0, ther_avg_cnt = 0;
	u32 ther_avg[2] = {0};

	/*only check path 0 */
	for (path = 0; path < 1; path++) {
		cur_ther = halrf_get_thermal_8851b(rf, path);
		iqk_info->ther_avg[path][iqk_info->ther_avg_idx] = cur_ther;		
		/*Average times */
		ther_avg_cnt = 0;
		for (i = 0; i < 0x4; i++) {
			if (iqk_info->ther_avg[path][i]) {
				ther_avg[path] += iqk_info->ther_avg[path][i];
				ther_avg_cnt++;
			}
		}
		/*Calculate Average ThermalValue after average enough times*/
		if (ther_avg_cnt) {
			cur_ther = (u8)(ther_avg[path] / ther_avg_cnt);
		}
		if (HALRF_ABS(cur_ther, iqk_info->thermal[path] ) > IQK_THR_ReK) {
			iqk_info->thermal_rek_en = true;
		}
		else {
			iqk_info->thermal_rek_en = false;
		}
		//RF_DBG(rf, DBG_RF_IQK, "[IQK]S%x, iqk_ther =%d, ther_now = %d\n", path, iqk->thermal[path], cur_ther);
	}
	iqk_info->ther_avg_idx++;
	if (iqk_info->ther_avg_idx == 0x4)
		iqk_info->ther_avg_idx = 0;
	return;
}

__iram_func__
bool halrf_iqk_get_ther_rek_8851b(struct rf_info *rf ) {
	struct halrf_iqk_info *iqk_info = &rf->iqk;

	return iqk_info->thermal_rek_en;
}

__iram_func__
u8 halrf_iqk_get_mcc_ch0_8851b(struct rf_info *rf ) {
	
	struct halrf_iqk_info *iqk_info = &rf->iqk;

	return iqk_info->iqk_mcc_ch[0][0];
}

__iram_func__
u8 halrf_iqk_get_mcc_ch1_8851b(struct rf_info *rf ) {
	
	struct halrf_iqk_info *iqk_info = &rf->iqk;

	return iqk_info->iqk_mcc_ch[1][0];
}

__iram_func__
bool halrf_check_fwiqk_done_8851b(struct rf_info *rf)
{
	u32 counter = 0x0;
	bool flag = false;
	bool isfail = false;
#if 1
	while (1) {
		if (halrf_rreg(rf, 0xbff8, MASKBYTE0) == 0xaa  || counter > 3000) {
			if(halrf_rreg(rf, 0x8010, MASKBYTE0) == 0xaa) {
				flag = true;
				break;
			}
		}
		counter++;
		halrf_delay_us(rf, 10);
	};
#else
	for(counter = 0; counter < 6000; counter++)
		halrf_delay_us(rf, 10);
#endif
	if (counter < 10)
		isfail = true; 
	else
		isfail = false; 
		
	if(flag) {
		RF_DBG(rf, DBG_RF_IQK, "[IQK] Load FW Done, counter = %d!!\n", counter);
	} else {
		RF_DBG(rf, DBG_RF_IQK, "[IQK] Load FW Fail, counter = %d!!\n", counter);	
		halrf_iqk_tx_bypass_8851b(rf, RF_PATH_A);
		halrf_iqk_tx_bypass_8851b(rf, RF_PATH_B);
		halrf_iqk_rx_bypass_8851b(rf, RF_PATH_A);
		halrf_iqk_rx_bypass_8851b(rf, RF_PATH_B);
		halrf_iqk_lok_bypass_8851b(rf, RF_PATH_A);		
		halrf_iqk_lok_bypass_8851b(rf, RF_PATH_B);
	}
	halrf_wreg(rf, 0x8010, 0x000000ff,0x0);
	return isfail;
}

__iram_func__
void halrf_enable_fw_iqk_8851b(struct rf_info *rf, bool is_fw_iqk)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;

	if (is_fw_iqk) {
		iqk_info->is_fw_iqk = true;
	} else {
		iqk_info->is_fw_iqk = false;
	}
	return;
}

__iram_func__ 
u8 halrf_get_iqk_times_8851b(struct rf_info *rf) {
	u8 times  = 0x0;

	//times = (u8) halrf_rreg(rf, 0x9fe8, 0x0000ff00);
	return times;
}

__iram_func__ 
u32 halrf_get_iqk_ver_8851b(void)
{
	return iqk_version_8851b;
}

bool iqk_mcc_page_sel_8851b(struct rf_info *rf, enum phl_phy_idx phy, u8 path)
{
	return true;
}
__iram_func__
void iqk_init_8851b(struct rf_info *rf)
{
	struct halrf_iqk_info *iqk_info = &rf->iqk;
	u8 idx, path;

	//halrf_wreg(rf, 0x9fe0, MASKDWORD, 0x0);	
	if (!iqk_info->is_iqk_init) {		
		RF_DBG(rf, DBG_RF_IQK, "[IQK]===>%s\n", __func__);
		iqk_info->is_iqk_init = true;
		iqk_info->is_nbiqk = false;
		iqk_info->iqk_fft_en = false;
		iqk_info->iqk_sram_en = false;
		iqk_info->iqk_cfir_en = false;
		iqk_info->iqk_xym_en = false;
		iqk_info->thermal_rek_en = false;
		iqk_info->ther_avg_idx = 0x0;
		iqk_info->iqk_times = 0x0;
		/*channel/path/TRX(TX:0, RX:1) */
		iqk_info->iqk_times = 0x0;
		iqk_info->is_fw_iqk = false;
		for (idx = 0; idx < 2; idx++) { //channel		
			iqk_info->iqk_channel[idx] = 0x0;
			for (path = 0; path < ss_8851b; path++) {//path
				iqk_info->lok_cor_fail[idx][path] = false;
				iqk_info->lok_fin_fail[idx][path] = false;
				iqk_info->iqk_tx_fail[idx][path] = false;
				iqk_info->iqk_rx_fail[idx][path] = false;
				iqk_info->iqk_mcc_ch[idx][path] = 0x0;
				iqk_info->iqk_table_idx[path] = 0x0;
			}
		}
	}
	return;
}
#endif


