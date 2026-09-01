/******************************************************************************
 *
 * Copyright(c) 2007 - 2017  Realtek Corporation.
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

void _txgapk_backup_bb_registers_8851b(
	struct rf_info *rf,
	enum phl_phy_idx phy,
	u32 *reg,
	u32 *reg_backup,
	u32 reg_num)
{
	u32 i;

	for (i = 0; i < reg_num; i++) {
		reg_backup[i] = halrf_rreg(rf, reg[i], MASKDWORD);

		RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK] Backup BB 0x%x = 0x%x\n",
		       reg[i], reg_backup[i]);
	}
}

void _txgapk_reload_bb_registers_8851b(
	struct rf_info *rf,
	enum phl_phy_idx phy,
	u32 *reg,
	u32 *reg_backup,
	u32 reg_num)

{
	u32 i;

	for (i = 0; i < reg_num; i++) {
		halrf_wreg(rf, reg[i], MASKDWORD, reg_backup[i]);

		RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK] Reload BB 0x%x = 0x%x\n",
		       reg[i], reg_backup[i]);
	}
}

void _halrf_txgapk_bkup_rf_8851b(
	struct rf_info *rf,
	u32 *rf_reg,
	u32 rf_bkup[][TXGAPK_RF_REG_NUM_8851B],
	u8 path)
{
	u8 i;

	for (i = 0; i < TXGAPK_RF_REG_NUM_8851B; i++) {
		rf_bkup[path][i] = halrf_rrf(rf, path, rf_reg[i], MASKRF);
		
			RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK] Backup RF S%d 0x%x = %x\n",
				path, rf_reg[i], rf_bkup[path][i]);
	}
}

void _halrf_txgapk_reload_rf_8851b(
	struct rf_info *rf,
	u32 *rf_reg,
	u32 rf_bkup[][TXGAPK_RF_REG_NUM_8851B],
	u8 path)
{
	u8 i;

	for (i = 0; i < TXGAPK_RF_REG_NUM_8851B; i++) {
		halrf_wrf(rf, path, rf_reg[i], MASKRF, rf_bkup[path][i]);
		
			RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK] Reload RF S%d 0x%x = %x\n",
				path, rf_reg[i], rf_bkup[path][i]);
	}
}


void _halrf_txgapk_before_one_shot_enable_8851b
	(struct rf_info *rf)
{
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s\n", __func__);
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);

	/* set 0x80d4[21:16]=0x03 (before oneshot NCTL) to get report later */
	halrf_wreg(rf, 0x80d4, 0x003F0000, 0x03);
		
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> before set one-shot bit, 0x%x= 0x%x\n", 0x8010, halrf_rreg(rf, 0x8010, MASKDWORD));
}

void _halrf_txgapk_one_shot_nctl_done_check_8851b
	(struct rf_info *rf, enum txgapk_id id, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	
	/* for check status */
	u32 r_bff8 = 0;
	u32 r_80fc = 0;
	bool is_ready = false;
	u16 count = 1;

	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s\n", __func__);
	
	/* for 0xbff8 check NCTL DONE */
	while (count < 2000) {	
		r_bff8 = halrf_rreg(rf, 0xbff8, MASKBYTE0);
				
		if (r_bff8 == 0x55) {
			is_ready = true;
			break;
		}	
		/* r_bff8 = 0; */
		halrf_delay_us(rf, 10);
		count++;
	}
	
	halrf_delay_us(rf, 1);
	txgapk_info->txgapk_chk_cnt[path][id][0] = count; 
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> check 0xBFF8[7:0] = 0x%x, IsReady = %d, ReadTimes = %d,delay 1 us\n", r_bff8, is_ready, count);

	

	/* for 0x80fc check NCTL DONE */	
	count = 1;
	is_ready = false;
	while (count < 2000) {			
		r_80fc = halrf_rreg(rf, 0x80fc, MASKLWORD);
	
		if (r_80fc == 0x8000) {
			is_ready = true;
			break;
		}	
		/* r_80fc = 0; */
		halrf_delay_us(rf, 1);
		count++;
	}

	halrf_delay_us(rf, 1);
	txgapk_info->txgapk_chk_cnt[path][id][1] = count; 
		
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);

	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> check 0x80fc[15:0] = 0x%x, IsReady = %d, ReadTimes = %d, 0x%x= 0x%x \n", r_80fc, is_ready, count, 0x8010, halrf_rreg(rf, 0x8010, MASKDWORD) ); 
		
}


void _halrf_txgapk_track_table_nctl_2g_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	
	u32 i;
	u32 d[17] = {0}, ta[17] = {0};
	u32 rf_tmp = 0;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, path = %d\n", __func__, path);

	halrf_wrf(rf, RF_PATH_A, 0x00, MASKRF, 0x50121);
	halrf_wrf(rf, RF_PATH_A, 0xdf, 0x01000, 0x1);
	halrf_wrf(rf, RF_PATH_A, 0x9e, 0x00100, 0x1);
	halrf_wrf(rf, RF_PATH_A, 0x83, 0x00007, 0x0);
	halrf_wrf(rf, RF_PATH_A, 0x83, 0x000f0, 0x7);
	halrf_delay_us(rf, 1);
	
	//04_G_S0_GapK_Track_51B-0217
	halrf_wreg(rf, 0x80e4, 0x0000003f, 0x01);
	halrf_wreg(rf, 0x801c, 0x000e0000, 0x2);
	halrf_wreg(rf, 0x80e0, 0x000001f0, 0x7);
	halrf_wreg(rf, 0x80e0, 0x0000f000, 0x0);
	halrf_wreg(rf, 0x80cc, 0x003f0000, 0x2d);
	//halrf_wreg(rf, 0x8158, 0x001fffff, 0x2aaa);
	halrf_wreg(rf, 0x8158, 0x001fffff, 0x25aa);	
	RF_DBG(rf, DBG_RF_IQK, "[IQK]0x1005e =	0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x1005e, MASKRF));
	
	halrf_wreg(rf, 0x5670, 0x00000002, 0x1);
	halrf_wreg(rf, 0x81cc, 0x0000003f, 0x12);
	halrf_wreg(rf, 0x802c, 0x0fff0000, 0x009);
	halrf_wreg(rf, 0x8000, MASKDWORD, 0x00001019);
	_halrf_txgapk_one_shot_nctl_done_check_8851b(rf, TXGAPK_TRACK, path);
	
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
	halrf_wreg(rf, 0x81cc, 0x0000003f, 0x3f);
	halrf_wreg(rf, 0x8000, MASKDWORD, 0x00001519);
	_halrf_txgapk_one_shot_nctl_done_check_8851b(rf, TXGAPK_TRACK, path);
	
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
	halrf_wreg(rf, 0x5670, 0x00000002, 0x0);
	halrf_wreg(rf, 0x80e0, 0x00000001, 0x0);
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]0x1005c[17:11] =  0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x1005c, 0x3f800));
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]0x1005c[05:00] =  0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x1005c, 0x0003f));
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]0x1005e[17:12] =  0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x1005e, 0x3f000));
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]0x1005e[05:00] =  0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x1005e, 0x0003f));		
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]rf_0x11[1:0]iPA =	0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x11, 0x00003));
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]rf_0x11[6:4]PAD&MOD=  0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x11, 0x00070));
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]rf_0x11[16:12]TxBB =  0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x11, 0x1f000));
	
	/* ===== Read GapK Results, Bcut resolution = 0.0625 dB ===== */
	halrf_wreg(rf, 0x80d4, MASKDWORD, 0x00130000);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x3);
#if 0
	d[0] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[1] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[2] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[3] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else 
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[0] = rf_tmp & 0x0000007f;
	d[1] = rf_tmp & 0x00003f80;
	d[2] = rf_tmp & 0x001fc000;
	d[3] = rf_tmp & 0x0fe00000;

	d[1] = d[1] >> 7;
	d[2] = d[2] >> 14;
	d[3] = d[3] >> 21;
#endif

	
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x4);
#if 0	
	d[4] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[5] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[6] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[7] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[4] = rf_tmp & 0x0000007f;
	d[5] = rf_tmp & 0x00003f80;
	d[6] = rf_tmp & 0x001fc000;
	d[7] = rf_tmp & 0x0fe00000;

	d[5] = d[5] >> 7;
	d[6] = d[6] >> 14;
	d[7] = d[7] >> 21;
#endif	

	
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x5);
#if 0
	d[8] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[9] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[10] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[11] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[8] = rf_tmp & 0x0000007f;
	d[9] = rf_tmp & 0x00003f80;
	d[10] = rf_tmp & 0x001fc000;
	d[11] = rf_tmp & 0x0fe00000;

	d[9] = d[9] >> 7;
	d[10] = d[10] >> 14;
	d[11] = d[11] >> 21;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x6);
#if 0
	d[12] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[13] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[14] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[15] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[12] = rf_tmp & 0x0000007f;
	d[13] = rf_tmp & 0x00003f80;
	d[14] = rf_tmp & 0x001fc000;
	d[15] = rf_tmp & 0x0fe00000;

	d[13] = d[13] >> 7;
	d[14] = d[14] >> 14;
	d[15] = d[15] >> 21;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x7);
	d[16] = halrf_rreg(rf, 0x80fc, 0x0000007f);

	 
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x9);
#if 0
	ta[0] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[1] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[2] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[3] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);

	ta[0] = rf_tmp & 0x000000ff;
	ta[1] = rf_tmp & 0x0000ff00;
	ta[2] = rf_tmp & 0x00ff0000;
	ta[3] = rf_tmp & 0xff000000;

	ta[1] = ta[1] >> 8;
	ta[2] = ta[2] >> 16;
	ta[3] = ta[3] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xa);
#if 0
	ta[4] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[5] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[6] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[7] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[4] = rf_tmp & (0x000000ff);
	ta[5] = rf_tmp & (0x0000ff00);
	ta[6] = rf_tmp & (0x00ff0000);
	ta[7] = rf_tmp & (0xff000000);	

	ta[5] = ta[5] >> 8;
	ta[6] = ta[6] >> 16;
	ta[7] = ta[7] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xb);
#if 0
	ta[8] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[9] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[10] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[11] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[8] =rf_tmp & (0x000000ff);
	ta[9] = rf_tmp & (0x0000ff00);
	ta[10]= rf_tmp & (0x00ff0000);
	ta[11] = rf_tmp & (0xff000000);	

	ta[9] = ta[9] >> 8;
	ta[10] = ta[10] >> 16;
	ta[11] = ta[11] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xc);
#if 0
	ta[12] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[13] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[14] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[15] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[12] = rf_tmp & (0x000000ff);
	ta[13] = rf_tmp & (0x0000ff00);
	ta[14] = rf_tmp & (0x00ff0000);
	ta[15] = rf_tmp & (0xff000000);	

	ta[13] = ta[13] >> 8;
	ta[14] = ta[14] >> 16;
	ta[15] = ta[15] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xd);
	ta[16] = halrf_rreg(rf, 0x80fc, 0x000000ff);


#if 0
	/* for debug */
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x2);
	// = halrf_rreg(rf, 0x80fc, 0x0000007f);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x0);
	// = halrf_rreg(rf, 0x80fc, MASKDWORD);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x2);
	// = halrf_rreg(rf, 0x80fc, 0x007f0000);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x1);
	// = halrf_rreg(rf, 0x80fc, MASKDWORD);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xf);
	// = halrf_rreg(rf, 0x80fc, MASKDWORD);
#endif

	halrf_wreg(rf, 0x81cc, MASKDWORD, 0x24);
	halrf_wreg(rf, 0x80d0, 0x00100000, 0x1);
	for (i = 0; i < 17; i++) {
		if (d[i] & BIT(6))
			txgapk_info->track_d[path][i] = (s32)(d[i] | 0xffffff80);
		else
			txgapk_info->track_d[path][i] = (s32)(d[i]);

		RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]	track	d[%d][%d]=0x%x\n",
			path, i, txgapk_info->track_d[path][i]);
	}

	for (i = 0; i < 17; i++) {
		if (ta[i] & BIT(7))
			txgapk_info->track_ta[path][i] = (s32)(ta[i] | 0xffffff00);
		else
			txgapk_info->track_ta[path][i] = (s32)(ta[i]);
		
		RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]	track	ta[%d][%d]=0x%x\n",
			path, i, txgapk_info->track_ta[path][i]);
	}
	
}


void _halrf_txgapk_track_table_nctl_5g_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;

	u32 i;
	u32 d[17] = {0}, ta[17] = {0};
	u32 rf_tmp = 0;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, path = %d\n", __func__, path);
	//03_RF_for_DPK_Amode_v0
	halrf_wrf(rf, RF_PATH_A, 0x00, MASKRF, 0x50121);
	halrf_wrf(rf, RF_PATH_A, 0xdf, 0x01000, 0x1);
	halrf_wrf(rf, RF_PATH_A, 0x9e, 0x00100, 0x1);
	halrf_wrf(rf, RF_PATH_A, 0x8c, 0x0e000, 0x3);
	halrf_delay_us(rf, 1);
	
	//04_A_S0_GapK_Track_51B-0217
	halrf_wreg(rf, 0x80e4, 0x0000003f, 0x01);
	halrf_wreg(rf, 0x801c, 0x000e0000, 0x2);
	halrf_wreg(rf, 0x80e0, 0x000001f0, 0x07);
	halrf_wreg(rf, 0x80e0, 0x0000f000, 0x0);
	halrf_wreg(rf, 0x80cc, 0x003f0000, 0x1b);
	//halrf_wreg(rf, 0x8158, 0x001fffff, 0xaa8);
	halrf_wreg(rf, 0x8158, 0x001fffff, 0x4aa8);
	RF_DBG(rf, DBG_RF_TXGAPK, "[IQK]0x1005e =	0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x1005e, MASKRF));
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]0x8158 =	0x%x\n", halrf_rreg(rf, 0x8158, 0x001fffff));
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]0x1005e =	0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x1005e, MASKRF));
	halrf_wreg(rf, 0x5670, 0x00000002, 0x1);
	halrf_wreg(rf, 0x81cc, 0x0000003f, 0x12);
	halrf_wreg(rf, 0x802c, 0x0fff0000, 0x009);
	halrf_wreg(rf, 0x8000, MASKDWORD, 0x00001019);
	_halrf_txgapk_one_shot_nctl_done_check_8851b(rf, TXGAPK_TRACK, path);
	
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
	halrf_wreg(rf, 0x81cc, 0x0000003f, 0x3f);
	halrf_wreg(rf, 0x8000, MASKDWORD, 0x00001519);
	_halrf_txgapk_one_shot_nctl_done_check_8851b(rf, TXGAPK_TRACK, path);
	
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
	halrf_wreg(rf, 0x5670, 0x00000002, 0x0);
	halrf_wreg(rf, 0x80e0, 0x00000001, 0x0);

	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]0x1005c[17:11] =  0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x1005c, 0x3f800));
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]0x1005c[05:00] =  0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x1005c, 0x0003f));
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]0x1005e[17:12] =  0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x1005e, 0x3f000));
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]0x1005e[05:00] =  0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x1005e, 0x0003f));		
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]rf_0x11[1:0]iPA =	0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x11, 0x00003));
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]rf_0x11[6:4]PAD&MOD=  0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x11, 0x00070));
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]rf_0x11[16:12]TxBB =  0x%x\n", halrf_rrf(rf, RF_PATH_A, 0x11, 0x1f000));

	/* ===== Read GapK Results, Bcut resolution = 0.0625 dB ===== */
	halrf_wreg(rf, 0x80d4, MASKDWORD, 0x00130000);	
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x3);
#if 0
	d[0] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[1] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[2] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[3] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[0] = rf_tmp & (0x0000007f);
	d[1] = rf_tmp & (0x00003f80);
	d[2] = rf_tmp & (0x001fc000);
	d[3] = rf_tmp & (0x0fe00000);

	d[1] = d[1] >> 7;
	d[2] = d[2] >> 14;
	d[3] = d[3] >> 21;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x4);
#if 0
	d[4] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[5] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[6] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[7] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[4] = rf_tmp & (0x0000007f);
	d[5] = rf_tmp & (0x00003f80);
	d[6] = rf_tmp & (0x001fc000);
	d[7] = rf_tmp & (0x0fe00000);

	d[5] = d[5] >> 7;
	d[6] = d[6] >> 14;
	d[7] = d[7] >> 21;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x5);
#if 0
	d[8] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[9] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[10] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[11] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[8] = rf_tmp & (0x0000007f);
	d[9] = rf_tmp & (0x00003f80);
	d[10] = rf_tmp & (0x001fc000);
	d[11] = rf_tmp & (0x0fe00000);

	d[9] = d[9] >> 7;
	d[10] = d[10] >> 14;
	d[11] = d[11] >> 21;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x6);
#if 0
	d[12] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[13] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[14] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[15] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[12] = rf_tmp & (0x0000007f);
	d[13] = rf_tmp & (0x00003f80);
	d[14] = rf_tmp & (0x001fc000);
	d[15] = rf_tmp & (0x0fe00000);

	d[13] = d[13] >> 7;
	d[14] = d[14] >> 14;
	d[15] = d[15] >> 21;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x7);
	d[16] = halrf_rreg(rf, 0x80fc, 0x0000007f);

	 
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x9);
#if 0
	ta[0] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[1] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[2] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[3] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[0] =  rf_tmp & (0x000000ff);
	ta[1] = rf_tmp & (0x0000ff00);
	ta[2] = rf_tmp & (0x00ff0000);
	ta[3] = rf_tmp & (0xff000000);

	ta[1] = ta[1] >> 8;
	ta[2] = ta[2] >> 16;
	ta[3] = ta[3] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xa);
#if 0
	ta[4] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[5] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[6] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[7] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[4] = rf_tmp & (0x000000ff);
	ta[5] = rf_tmp & (0x0000ff00);
	ta[6] = rf_tmp & (0x00ff0000);
	ta[7] = rf_tmp & (0xff000000);	

	ta[5] = ta[5] >> 8;
	ta[6] = ta[6] >> 16;
	ta[7] = ta[7] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xb);
#if 0
	ta[8] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[9] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[10] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[11] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[8] = rf_tmp & (0x000000ff);
	ta[9] = rf_tmp & (0x0000ff00);
	ta[10]= rf_tmp & (0x00ff0000);
	ta[11] = rf_tmp & (0xff000000);	

	ta[9] = ta[9] >> 8;
	ta[10] = ta[10] >> 16;
	ta[11] = ta[11] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xc);
#if 0
	ta[12] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[13] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[14] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[15] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[12] = rf_tmp & (0x000000ff);
	ta[13] = rf_tmp & (0x0000ff00);
	ta[14] = rf_tmp & (0x00ff0000);
	ta[15] = rf_tmp & (0xff000000);		

	ta[13] = ta[13] >> 8;
	ta[14] = ta[14] >> 16;
	ta[15] = ta[15] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xd);
	ta[16] = halrf_rreg(rf, 0x80fc, 0x000000ff);

#if 0
	/* debug */
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x2);
	// = halrf_rreg(rf, 0x80fc, 0x0000007f);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x0);
	// = halrf_rreg(rf, 0x80fc, MASKDWORD);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x2);
	// = halrf_rreg(rf, 0x80fc, 0x007f0000);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x1);
	// = halrf_rreg(rf, 0x80fc, MASKDWORD);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xf);
	// = halrf_rreg(rf, 0x80fc, MASKDWORD);
#endif

	halrf_wreg(rf, 0x81cc, MASKDWORD, 0x24);
	halrf_wreg(rf, 0x80d0, 0x00100000, 0x1);
	for (i = 0; i < 17; i++) {
		if (d[i] & BIT(6))
			txgapk_info->track_d[path][i] = (s32)(d[i] | 0xffffff80);
		else
			txgapk_info->track_d[path][i] = (s32)(d[i]);

		RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]	track	d[%d][%d]=0x%x\n",
			path, i, txgapk_info->track_d[path][i]);
	}

	for (i = 0; i < 17; i++) {
		if (ta[i] & BIT(7))
			txgapk_info->track_ta[path][i] = (s32)(ta[i] | 0xffffff00);
		else
			txgapk_info->track_ta[path][i] = (s32)(ta[i]);
		
		RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]	track	ta[%d][%d]=0x%x\n",
			path, i, txgapk_info->track_ta[path][i]);
	}

}


void _halrf_txgapk_track_table_nctl_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	u8 channel = rf->hal_com->band[phy].cur_chandef.center_ch;
	
	if (channel >= 1 && channel <= 14)
		_halrf_txgapk_track_table_nctl_2g_8851b(rf, phy, path);
	else
		_halrf_txgapk_track_table_nctl_5g_8851b(rf, phy, path);
}

void _halrf_txgapk_write_track_table_default_2g_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u32 bias = 0;
	
	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 8;
		
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);
		

	/* NIC */	
	/* AP iFEM */
	/* AP eFEM */
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x1); /* enter debug mode before write */
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x0|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x1|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x2|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x3|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x4|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x5|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_track_table_default_5gl_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 8;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);

	/* NIC */	
	/* AP iFEM */
	/* AP eFEM */
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x1); /* enter debug mode before write */
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x0|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x1|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x2|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x3|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x4|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x5|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_track_table_default_5gm_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u32 bias = 0;
	
	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 8;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);
	
	/* NIC */	
	/* AP iFEM */
	/* AP eFEM */
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x1); /* enter debug mode before write */
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x0|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x1|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x2|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x3|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x4|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x5|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_track_table_default_5gh_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 8;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);

	/* NIC */	
	/* AP iFEM */
	/* AP eFEM */
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x1); /* enter debug mode before write */
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x0|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x1|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x2|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x3|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x4|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, (0x5|bias));
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x3f, 0x0);
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_track_table_default_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	u8 channel = rf->hal_com->band[phy].cur_chandef.center_ch;

	if (channel >= 1 && channel <= 14)
		_halrf_txgapk_write_track_table_default_2g_8851b(rf, phy, path);
	else if (channel >= 36 && channel <= 64)
		_halrf_txgapk_write_track_table_default_5gl_8851b(rf, phy, path);
	else if (channel >= 100 && channel <= 144)
		_halrf_txgapk_write_track_table_default_5gm_8851b(rf, phy, path);
	else if (channel >= 149 && channel <= 177)
		_halrf_txgapk_write_track_table_default_5gh_8851b(rf, phy, path);
}

void _halrf_txgapk_write_track_table_2g_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u8 rfe_type = rf->phl_com->dev_cap.rfe_type;
	u8 pa_change[6] = {0};

	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 8;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);
	
	if (rfe_type <= 50) { 
		/* NIC */	
		/* AP iFEM */
		pa_change[0] = 2;
		pa_change[1] = 4;
		pa_change[2] = 6;
		pa_change[3] = 8;
		pa_change[4] = 10;
		pa_change[5] = 12;		
	}
	else { 
		/* AP eFEM */
		pa_change[0] = 1;
		pa_change[1] = 3;
		pa_change[2] = 5;
		pa_change[3] = 7;
		pa_change[4] = 9;
		pa_change[5] = 11;	
	}

	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x1); /* enter debug mode before write */
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x0|bias); 
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[0]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x1|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[1]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x2|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[2]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x3|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[3]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x4|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[4]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x5|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[5]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_track_table_5gl_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u8 pa_change[6] = {0};
	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 8;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);

	/* NIC */	
	/* AP iFEM */
	/* AP eFEM */
	pa_change[0] = 4;
	pa_change[1] = 6;
	pa_change[2] = 8;
	pa_change[3] = 10;
	pa_change[4] = 12;
	pa_change[5] = 13;
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x1); /* enter debug mode before write */
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x0|bias); 
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[0]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x1|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[1]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x2|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[2]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x3|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[3]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x4|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[4]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x5|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[5]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x0); /* exit debug mode after write */

}

void _halrf_txgapk_write_track_table_5gm_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u8 pa_change[6] = {0};
	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 8;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);

	/* NIC */	
	/* AP iFEM */
	/* AP eFEM */
	pa_change[0] = 4;
	pa_change[1] = 6;
	pa_change[2] = 8;
	pa_change[3] = 10;
	pa_change[4] = 12;
	pa_change[5] = 13;
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x1); /* enter debug mode before write */
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x0|bias); 
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[0]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x1|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[1]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x2|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[2]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x3|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[3]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x4|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[4]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x5|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[5]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x0); /* exit debug mode after write */

}

void _halrf_txgapk_write_track_table_5gh_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u8 pa_change[6] = {0};
	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 8;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);

	/* NIC */	
	/* AP iFEM */
	/* AP eFEM */
	pa_change[0] = 4;
	pa_change[1] = 6;
	pa_change[2] = 8;
	pa_change[3] = 10;
	pa_change[4] = 12;
	pa_change[5] = 13;
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x1); /* enter debug mode before write */
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x0|bias); 
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[0]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x1|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[1]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x2|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[2]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x3|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[3]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x4|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[4]] / 2) & 0x3f);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x0f, 0x5|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->track_ta[path][pa_change[5]] / 2) & 0x3f);

	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x08000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_track_table_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	u8 channel = rf->hal_com->band[phy].cur_chandef.center_ch;

	if (channel >= 1 && channel <= 14)
		_halrf_txgapk_write_track_table_2g_8851b(rf, phy, path);
	else if (channel >= 36 && channel <= 64)
		_halrf_txgapk_write_track_table_5gl_8851b(rf, phy, path);
	else if (channel >= 100 && channel <= 144)
		_halrf_txgapk_write_track_table_5gm_8851b(rf, phy, path);
	else if (channel >= 149 && channel <= 177)
		_halrf_txgapk_write_track_table_5gh_8851b(rf, phy, path);
}

void _halrf_txgapk_power_table_nctl_2g_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;

	u32 i;
	u32 d[17] = {0}, ta[17] = {0};
	u32 rf_tmp = 0;

	//05_G_S0_GapK_Power_51B-0217
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s\n", __func__);
	halrf_wreg(rf, 0x80e0, 0x000001f0, 0x07);
	halrf_wreg(rf, 0x80cc, 0x003f0000, 0x24);
	halrf_wreg(rf, 0x8170, 0x001fffff, 0x000088);
	halrf_wreg(rf, 0x5670, 0x00000002, 0x1);
	halrf_wreg(rf, 0x81cc, MASKDWORD, 0x12);
	halrf_wreg(rf, 0x802c, 0x0fff0000, 0x009);
	halrf_wreg(rf, 0x8000, MASKDWORD, 0x00001119);
	_halrf_txgapk_one_shot_nctl_done_check_8851b(rf, TXGAPK_PWR, path);
	
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
	halrf_wreg(rf, 0x81cc, 0x0000003f, 0x3f);
	halrf_wreg(rf, 0x8000, MASKDWORD, 0x00001619);
	_halrf_txgapk_one_shot_nctl_done_check_8851b(rf, TXGAPK_PWR, path);
	
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
	halrf_wreg(rf, 0x5670, 0x00000002, 0x0);
	halrf_wreg(rf, 0x801c, 0x000e0000, 0x0);


	/* ===== Read GapK Results, Bcut resolution = 0.0625 dB ===== */
	halrf_wreg(rf, 0x80d4, MASKDWORD, 0x00130000);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x3);
#if 0
	d[0] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[1] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[2] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[3] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[0] = rf_tmp & (0x0000007f);
	d[1] = rf_tmp & (0x00003f80);
	d[2] = rf_tmp & (0x001fc000);
	d[3] = rf_tmp & (0x0fe00000);

	d[1] = d[1] >> 7;
	d[2] = d[2] >> 14;
	d[3] = d[3] >> 21;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x4);
#if 0
	d[4] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[5] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[6] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[7] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[4] = rf_tmp & (0x0000007f);
	d[5] = rf_tmp & (0x00003f80);
	d[6] = rf_tmp & (0x001fc000);
	d[7] = rf_tmp & (0x0fe00000);

	d[5] = d[5] >> 7;
	d[6] = d[6] >> 14;
	d[7] = d[7] >> 21;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x5);
#if 0
	d[8] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[9] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[10] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[11] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[8] = rf_tmp & (0x0000007f);
	d[9] = rf_tmp & (0x00003f80);
	d[10] = rf_tmp & (0x001fc000);
	d[11] = rf_tmp & (0x0fe00000);

	d[9] = d[9] >> 7;
	d[10] = d[10] >> 14;
	d[11] = d[11] >> 21;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x6);
#if 0
	d[12] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[13] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[14] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[15] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[12] = rf_tmp & (0x0000007f);
	d[13] = rf_tmp & (0x00003f80);
	d[14] = rf_tmp & (0x001fc000);
	d[15] = rf_tmp & (0x0fe00000);

	d[13] = d[13] >> 7;
	d[14] = d[14] >> 14;
	d[15] = d[15] >> 21;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x7);
	d[16] = halrf_rreg(rf, 0x80fc, 0x0000007f);


	
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x9);
#if 0
	ta[0] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[1] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[2] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[3] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[0] = rf_tmp & (0x000000ff);
	ta[1] = rf_tmp & (0x0000ff00);
	ta[2] = rf_tmp & (0x00ff0000);
	ta[3] = rf_tmp & (0xff000000);

	ta[1] = ta[1] >> 8;
	ta[2] = ta[2] >> 16;
	ta[3] = ta[3] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xa);
#if 0
	ta[4] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[5] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[6] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[7] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[4] = rf_tmp & (0x000000ff);
	ta[5] = rf_tmp & (0x0000ff00);
	ta[6] = rf_tmp & (0x00ff0000);
	ta[7] = rf_tmp & (0xff000000);

	ta[5] = ta[5] >> 8;
	ta[6] = ta[6] >> 16;
	ta[7] = ta[7] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xb);
#if 0
	ta[8] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[9] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[10] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[11] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[8] = rf_tmp & (0x000000ff);
	ta[9] = rf_tmp & (0x0000ff00);
	ta[10] = rf_tmp & (0x00ff0000);
	ta[11] = rf_tmp & (0xff000000);

	ta[9] = ta[9] >> 8;
	ta[10] = ta[10] >> 16;
	ta[11] = ta[11] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xc);
#if 0
	ta[12] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[13] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[14] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[15] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[12] = rf_tmp & (0x000000ff);
	ta[13] = rf_tmp & (0x0000ff00);
	ta[14] = rf_tmp & (0x00ff0000);
	ta[15] = rf_tmp & (0xff000000);

	ta[13] = ta[13] >> 8;
	ta[14] = ta[14] >> 16;
	ta[15] = ta[15] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xd);
	ta[16] = halrf_rreg(rf, 0x80fc, 0x000000ff);

#if 0
	/* debug */ 
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x2);
	// = halrf_rreg(rf, 0x80fc, 0x0000007f);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x0);
	// = halrf_rreg(rf, 0x80fc, MASKDWORD);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x2);
	// = halrf_rreg(rf, 0x80fc, 0x007f0000);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x1);
	// = halrf_rreg(rf, 0x80fc, MASKDWORD);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xf);
	// = halrf_rreg(rf, 0x80fc, MASKDWORD);
#endif

	/* for debug */
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> afr restore 0x801c, 0x%x= 0x%x\n", 0x8010, halrf_rreg(rf, 0x8010, MASKDWORD));	
	halrf_wreg(rf, 0x81cc, MASKDWORD, 0x1b);
	halrf_wreg(rf, 0x80d0, 0x00100000, 0x1);
	for (i = 0; i < 17; i++) {
		if (d[i] & BIT(6))
			txgapk_info->power_d[path][i] = (s32)(d[i] | 0xffffff80);
		else
			txgapk_info->power_d[path][i] = (s32)(d[i]);

		RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]	power	d[%d][%d]=0x%x\n",
			path, i, txgapk_info->power_d[path][i]);
	}

	for (i = 0; i < 17; i++) {
		if (ta[i] & BIT(7))
			txgapk_info->power_ta[path][i] = (s32)(ta[i] | 0xffffff00);
		else
			txgapk_info->power_ta[path][i] = (s32)(ta[i]);
		
		RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]	power	ta[%d][%d]=0x%x\n",
			path, i, txgapk_info->power_ta[path][i]);
	}

}


void _halrf_txgapk_power_table_nctl_5g_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	/* u8 rfe_type = rf->phl_com->dev_cap.rfe_type; */

	u32 i;
	u32 d[17] = {0}, ta[17] = {0};
	u32 rf_tmp = 0;

	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s\n", __func__);

	//05_A_S0_GapK_Power_51B-0217
	halrf_wreg(rf, 0x80e0, 0x000001f0, 0x07);
	halrf_wreg(rf, 0x80cc, 0x003f0000, 0x1b);
	halrf_wreg(rf, 0x8170, 0x001fffff, 0x000480);
	halrf_wreg(rf, 0x5670, 0x00000002, 0x1);
	halrf_wreg(rf, 0x81cc, MASKDWORD, 0x12);
	halrf_wreg(rf, 0x802c, 0x0fff0000, 0x009);
	halrf_wreg(rf, 0x8000, MASKDWORD, 0x00001119);
	_halrf_txgapk_one_shot_nctl_done_check_8851b(rf, TXGAPK_PWR, path);
	
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
	halrf_wreg(rf, 0x81cc, 0x0000003f, 0x3f);
	halrf_wreg(rf, 0x8000, MASKDWORD, 0x00001619);
	_halrf_txgapk_one_shot_nctl_done_check_8851b(rf, TXGAPK_PWR, path);
	
	halrf_wreg(rf, 0x8010, 0x000000ff, 0x00);
	halrf_wreg(rf, 0x5670, 0x00000002, 0x0);
	halrf_wreg(rf, 0x801c, 0x000e0000, 0x0);

	/* ===== Read GapK Results, Bcut resolution = 0.0625 dB ===== */	
	halrf_wreg(rf, 0x80d4, MASKDWORD, 0x00130000);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x3);
#if 0
	d[0] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[1] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[2] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[3] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[0] = rf_tmp & (0x0000007f);
	d[1] = rf_tmp & (0x00003f80);
	d[2] = rf_tmp & (0x001fc000);
	d[3] = rf_tmp & (0x0fe00000);

	d[1] = d[1] >> 7;
	d[2] = d[2] >> 14;
	d[3] = d[3] >> 21;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x4);
#if 0
	d[4] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[5] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[6] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[7] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[4] = rf_tmp & (0x0000007f);
	d[5] = rf_tmp & (0x00003f80);
	d[6] = rf_tmp & (0x001fc000);
	d[7] = rf_tmp & (0x0fe00000);

	d[5] = d[5] >> 7;
	d[6] = d[6] >> 14;
	d[7] = d[7] >> 21;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x5);
#if 0
	d[8] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[9] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[10] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[11] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[8] = rf_tmp & (0x0000007f);
	d[9] = rf_tmp & (0x00003f80);
	d[10] = rf_tmp & (0x001fc000);
	d[11] = rf_tmp & (0x0fe00000);

	d[9] = d[9] >> 7;
	d[10] = d[10] >> 14;
	d[11] = d[11] >> 21;
#endif
	 
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x6);
#if 0
	d[12] = halrf_rreg(rf, 0x80fc, 0x0000007f);
	d[13] = halrf_rreg(rf, 0x80fc, 0x00003f80);
	d[14] = halrf_rreg(rf, 0x80fc, 0x001fc000);
	d[15] = halrf_rreg(rf, 0x80fc, 0x0fe00000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0x0fffffff);
	d[12] = rf_tmp & (0x0000007f);
	d[13] = rf_tmp & (0x00003f80);
	d[14] = rf_tmp & (0x001fc000);
	d[15] = rf_tmp & (0x0fe00000);

	d[13] = d[13] >> 7;
	d[14] = d[14] >> 14;
	d[15] = d[15] >> 21;
#endif	
	 
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x7);
	d[16] = halrf_rreg(rf, 0x80fc, 0x0000007f);

	 
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x9);
#if 0
	ta[0] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[1] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[2] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[3] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[0] = rf_tmp & (0x000000ff);
	ta[1] = rf_tmp & (0x0000ff00);
	ta[2] = rf_tmp & (0x00ff0000);
	ta[3] = rf_tmp & (0xff000000);
	
	ta[1] = ta[1] >> 8;
	ta[2] = ta[2] >> 16;
	ta[3] = ta[3] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xa);
#if 0
	ta[4] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[5] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[6] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[7] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[4] = rf_tmp & (0x000000ff);
	ta[5] = rf_tmp & (0x0000ff00);
	ta[6] = rf_tmp & (0x00ff0000);
	ta[7] = rf_tmp & (0xff000000);

	ta[5] = ta[5] >> 8;
	ta[6] = ta[6] >> 16;
	ta[7] = ta[7] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xb);
#if 0
	ta[8] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[9] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[10] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[11] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[8] = rf_tmp & (0x000000ff);
	ta[9] = rf_tmp & (0x0000ff00);
	ta[10] = rf_tmp & (0x00ff0000);
	ta[11] = rf_tmp & (0xff000000);

	ta[9] = ta[9] >> 8;
	ta[10] = ta[10] >> 16;
	ta[11] = ta[11] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xc);
#if 0
	ta[12] = halrf_rreg(rf, 0x80fc, 0x000000ff);
	ta[13] = halrf_rreg(rf, 0x80fc, 0x0000ff00);
	ta[14] = halrf_rreg(rf, 0x80fc, 0x00ff0000);
	ta[15] = halrf_rreg(rf, 0x80fc, 0xff000000);
#else
	rf_tmp = halrf_rreg(rf, 0x80fc, 0xffffffff);
	ta[12] = rf_tmp & (0x000000ff);
	ta[13] = rf_tmp & (0x0000ff00);
	ta[14] = rf_tmp & (0x00ff0000);
	ta[15] = rf_tmp & (0xff000000);

	ta[13] = ta[13] >> 8;
	ta[14] = ta[14] >> 16;
	ta[15] = ta[15] >> 24;
#endif

	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xd);
	ta[16] = halrf_rreg(rf, 0x80fc, 0x000000ff);

#if 0
	 /* debug */
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x2);
	// = halrf_rreg(rf, 0x80fc, 0x0000007f);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x0);
	// = halrf_rreg(rf, 0x80fc, MASKDWORD);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x2);
	// = halrf_rreg(rf, 0x80fc, 0x007f0000);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0x1);
	// = halrf_rreg(rf, 0x80fc, MASKDWORD);
	halrf_wreg(rf, 0x80e4, 0x00000f00, 0xf);
	// = halrf_rreg(rf, 0x80fc, MASKDWORD);
#endif
	
	/* for debug */
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> afr restore 0x801c, 0x%x= 0x%x\n", 0x8010, halrf_rreg(rf, 0x8010, MASKDWORD));
	halrf_wreg(rf, 0x81cc, MASKDWORD, 0x1b);
	halrf_wreg(rf, 0x80d0, 0x00100000, 0x1);
	for (i = 0; i < 17; i++) {
		if (d[i] & BIT(6))
			txgapk_info->power_d[path][i] = (s32)(d[i] | 0xffffff80);
		else
			txgapk_info->power_d[path][i] = (s32)(d[i]);

		RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]	power	d[%d][%d]=0x%x\n",
			path, i, txgapk_info->power_d[path][i]);
	}

	for (i = 0; i < 17; i++) {
		if (ta[i] & BIT(7))
			txgapk_info->power_ta[path][i] = (s32)(ta[i] | 0xffffff00);
		else
			txgapk_info->power_ta[path][i] = (s32)(ta[i]);
		
		RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]	power	ta[%d][%d]=0x%x\n",
			path, i, txgapk_info->power_ta[path][i]);
	}

}


void _halrf_txgapk_power_table_nctl_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	u8 channel = rf->hal_com->band[phy].cur_chandef.center_ch;

	if (channel >= 1 && channel <= 14)
		_halrf_txgapk_power_table_nctl_2g_8851b(rf, phy, path);
	else
		_halrf_txgapk_power_table_nctl_5g_8851b(rf, phy, path);

}

void _halrf_txgapk_write_power_table_default_2g_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 4;

	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);
	
	/* NIC */
	/* AP iFEM */
	/* AP eFEM */
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x1); /* enter debug mode before write */
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x0|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x1|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x2|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, 0x0); 
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_power_table_default_5gl_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 4;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);

	/* NIC */
	/* AP iFEM */
	/* AP eFEM */
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x1); /* enter debug mode before write */
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x0|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x1|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x2|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, 0x0); 
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_power_table_default_5gm_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 4;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);

	/* NIC */
	/* AP iFEM */
	/* AP eFEM */
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x1); /* enter debug mode before write */
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x0|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x1|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x2|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, 0x0); 

	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_power_table_default_5gh_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 4;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);

	/* NIC */
	/* AP iFEM */
	/* AP eFEM */
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x1); /* enter debug mode before write */
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x0|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x1|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, 0x0);
	
	halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x2|bias);
	halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, 0x0); 
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_power_table_default_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	u8 channel = rf->hal_com->band[phy].cur_chandef.center_ch;

	if (channel >= 1 && channel <= 14)
		_halrf_txgapk_write_power_table_default_2g_8851b(rf, phy, path);
	else if (channel >= 36 && channel <= 64)
		_halrf_txgapk_write_power_table_default_5gl_8851b(rf, phy, path);
	else if (channel >= 100 && channel <= 144)
		_halrf_txgapk_write_power_table_default_5gm_8851b(rf, phy, path);
	else if (channel >= 149 && channel <= 177)
		_halrf_txgapk_write_power_table_default_5gh_8851b(rf, phy, path);
}

void _halrf_txgapk_write_power_table_2g_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u8 rfe_type = rf->phl_com->dev_cap.rfe_type;
	u8 pa_change[3] = {0};

	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 4;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);

	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x1); /* enter debug mode before write */

	if (rfe_type <= 50) {
		/* NIC */
		/* AP iFEM */
		
		pa_change[0] = 6;
		pa_change[1] = 8;
		pa_change[2] = 10;

		halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x0|bias);
		halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->power_ta[path][pa_change[0]] / 2) & 0x3f);
		
		halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x1|bias);
		halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->power_ta[path][pa_change[1]] / 2) & 0x3f);
		
		halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x2|bias);
		halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->power_ta[path][pa_change[2]] / 2) & 0x3f);
	}
	else {
		/* AP eFEM */
		pa_change[0] = 14;
		
		halrf_wrf(rf, path, 0x33, 0x01fff, 0x1);
		halrf_wrf(rf, path, 0x3f, 0x0003f, (txgapk_info->power_ta[path][pa_change[0]] / 2) & 0x3f);
	}

	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_power_table_5gl_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u8 rfe_type = rf->phl_com->dev_cap.rfe_type;
	u8 pa_change[3] = {0};

	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 4;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);

	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x1); /* enter debug mode before write */

	if (rfe_type < 50) {
		/* NIC */
		pa_change[0] = 6;
		pa_change[1] = 8;
		pa_change[2] = 10;

		halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x0|bias);
		halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->power_ta[path][pa_change[0]] / 2) & 0x3f);
		
		halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x1|bias);
		halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->power_ta[path][pa_change[1]] / 2) & 0x3f);
		
		halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x2|bias);
		halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->power_ta[path][pa_change[2]] / 2) & 0x3f);
	} else if (rfe_type == 50) {
		/* AP iFEM */
		pa_change[0] = 6;
		pa_change[1] = 9;
		pa_change[2] = 10;

		halrf_wrf(rf, path, 0x33, 0x01fff, 0x21);
		halrf_wrf(rf, path, 0x3f, 0x0003f, (txgapk_info->power_ta[path][pa_change[0]] / 2) & 0x3f);
		halrf_wrf(rf, path, 0x33, 0x01fff, 0x23);
		halrf_wrf(rf, path, 0x3f, 0x0003f, (txgapk_info->power_ta[path][pa_change[1]] / 2) & 0x3f);
		halrf_wrf(rf, path, 0x33, 0x01fff, 0x26);
		halrf_wrf(rf, path, 0x3f, 0x0003f, (txgapk_info->power_ta[path][pa_change[2]] / 2) & 0x3f);
	} else {
		/* AP eFEM */
		pa_change[0] = 14;

		halrf_wrf(rf, path, 0x33, 0x01fff, 0x21);
		halrf_wrf(rf, path, 0x3f, 0x0003f, (txgapk_info->power_ta[path][pa_change[0]] / 2) & 0x3f);
	}

	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_power_table_5gm_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u8 rfe_type = rf->phl_com->dev_cap.rfe_type;
	u8 pa_change[3] = {0};

	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 4;
	
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);

	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x1); /* enter debug mode before write */

	if (rfe_type < 50) {
		/* NIC */
		pa_change[0] = 6;
		pa_change[1] = 8;
		pa_change[2] = 10;

		halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x0|bias);
		halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->power_ta[path][pa_change[0]] / 2) & 0x3f);
		
		halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x1|bias);
		halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->power_ta[path][pa_change[1]] / 2) & 0x3f);
		
		halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x2|bias);
		halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->power_ta[path][pa_change[2]] / 2) & 0x3f);
	} else if (rfe_type == 50) {
		/* AP iFEM */
		pa_change[0] = 6;
		pa_change[1] = 9;
		pa_change[2] = 10;

		halrf_wrf(rf, path, 0x33, 0x01fff, 0x29);
		halrf_wrf(rf, path, 0x3f, 0x0003f, (txgapk_info->power_ta[path][pa_change[0]] / 2) & 0x3f);
		halrf_wrf(rf, path, 0x33, 0x01fff, 0x2b);
		halrf_wrf(rf, path, 0x3f, 0x0003f, (txgapk_info->power_ta[path][pa_change[1]] / 2) & 0x3f);
		halrf_wrf(rf, path, 0x33, 0x01fff, 0x2e);
		halrf_wrf(rf, path, 0x3f, 0x0003f, (txgapk_info->power_ta[path][pa_change[2]] / 2) & 0x3f);
	} else {
		/* AP eFEM */
		pa_change[0] = 14;
		
		halrf_wrf(rf, path, 0x33, 0x01fff, 0x29);
		halrf_wrf(rf, path, 0x3f, 0x0003f, (txgapk_info->power_ta[path][pa_change[0]] / 2) & 0x3f);
	}
	
	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_power_table_5gh_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u8 rfe_type = rf->phl_com->dev_cap.rfe_type;
	u8 pa_change[3] = {0};

	u32 bias = 0;

	/* table1 */
	if (txgapk_info->txgapk_table_idx == 1)
		bias = 4;
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s, table index = %d, addr_bias = %d\n", __func__, txgapk_info->txgapk_table_idx, bias);
	

	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x1); /* enter debug mode before write */

	if (rfe_type < 50) {
		/* NIC */
		pa_change[0] = 6;
		pa_change[1] = 8;
		pa_change[2] = 10;

		halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x0|bias);
		halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->power_ta[path][pa_change[0]] / 2) & 0x3f);
		
		halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x1|bias);
		halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->power_ta[path][pa_change[1]] / 2) & 0x3f);
		
		halrf_wrf(rf, path, TXGAP_TB_ADDR_8851B, 0x07, 0x2|bias);
		halrf_wrf(rf, path, TXGAP_TB_VAL_8851B, 0x0003f, (txgapk_info->power_ta[path][pa_change[2]] / 2) & 0x3f);
	} else if (rfe_type == 50) {
		/* AP iFEM */
		pa_change[0] = 6;
		pa_change[1] = 9;
		pa_change[2] = 10;

		halrf_wrf(rf, path, 0x33, 0x01fff, 0x31);
		halrf_wrf(rf, path, 0x3f, 0x0003f, (txgapk_info->power_ta[path][pa_change[0]] / 2) & 0x3f);
		halrf_wrf(rf, path, 0x33, 0x01fff, 0x33);
		halrf_wrf(rf, path, 0x3f, 0x0003f, (txgapk_info->power_ta[path][pa_change[1]] / 2) & 0x3f);
		halrf_wrf(rf, path, 0x33, 0x01fff, 0x36);
		halrf_wrf(rf, path, 0x3f, 0x0003f, (txgapk_info->power_ta[path][pa_change[2]] / 2) & 0x3f);
	} else {
		/* AP eFEM */
		pa_change[0] = 14;

		halrf_wrf(rf, path, 0x33, 0x01fff, 0x31);
		halrf_wrf(rf, path, 0x3f, 0x0003f, (txgapk_info->power_ta[path][pa_change[0]] / 2) & 0x3f);	
	}

	halrf_wrf(rf, path, TXGAPK_DEBUGMASK_8851B, 0x40000, 0x0); /* exit debug mode after write */
}

void _halrf_txgapk_write_power_table_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path)
{
	u8 channel = rf->hal_com->band[phy].cur_chandef.center_ch;

	if (channel >= 1 && channel <= 14)
		_halrf_txgapk_write_power_table_2g_8851b(rf, phy, path);
	else if (channel >= 36 && channel <= 64)
		_halrf_txgapk_write_power_table_5gl_8851b(rf, phy, path);
	else if (channel >= 100 && channel <= 144)
		_halrf_txgapk_write_power_table_5gm_8851b(rf, phy, path);
	else if (channel >= 149 && channel <= 177)
		_halrf_txgapk_write_power_table_5gh_8851b(rf, phy, path);
}

void _halrf_txgapk_iqk_bk_reg_by_mode_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path, bool is_dbcc)
{
	u32 path_setting[2] = {0x0e19, 0x0e29};
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s\n", __func__);
	
	if (!is_dbcc) {
		/* no dbcc */

		_halrf_txgapk_before_one_shot_enable_8851b(rf);
		halrf_wreg(rf, 0x8000, MASKDWORD, path_setting[path]);

		_halrf_txgapk_one_shot_nctl_done_check_8851b(rf, TXGAPK_IQKBK, path);
		
		
		halrf_wreg(rf, 0x8008, MASKDWORD, 0x00000000);
		halrf_wreg(rf, 0x8088, MASKDWORD, 0x80000000);
		
	} else {
		/* dbcc */
		if (phy == HW_PHY_0) {
			halrf_wreg(rf, 0x8120, MASKDWORD, 0x10010000);
			halrf_wreg(rf, 0x8140, 0x00000100, 0x0);
			halrf_wreg(rf, 0x8150, MASKDWORD, 0xe4e4e4e4);
			halrf_wreg(rf, 0x8154, 0x00000100, 0x0);
			halrf_wreg(rf, 0x81cc, 0x0000003f, 0x0);
			halrf_wreg(rf, 0x81dc, MASKDWORD, 0x00000002);
		} else if (phy == HW_PHY_1) {
			halrf_wreg(rf, 0x8220, MASKDWORD, 0x10010000);
			halrf_wreg(rf, 0x8240, 0x00000100, 0x0);
			halrf_wreg(rf, 0x8250, MASKDWORD, 0xe4e4e4e4);
			halrf_wreg(rf, 0x8254, 0x00000100, 0x0);
			halrf_wreg(rf, 0x82cc, 0x0000003f, 0x0);
			halrf_wreg(rf, 0x82dc, MASKDWORD, 0x00000002);
		}
	}
		
	halrf_wrf(rf, path, 0xef, 0x00004, 0x0);

	halrf_wrf(rf, path, 0x0, 0xf0000, 0x3);
	
	halrf_wrf(rf, RF_PATH_A, 0x5, 0x00001, 0x1);
	halrf_wrf(rf, RF_PATH_A, 0x10005, 0x00001, 0x1);
	
}



void _halrf_txgapk_afe_bk_reg_by_mode_8851b
	(struct rf_info *rf, enum phl_phy_idx phy, enum rf_path path, bool is_dbcc)
{
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s\n", __func__);

	if (!is_dbcc) {
		/* no dbcc */
		halrf_wreg(rf, 0x20fc, 0xffff0000, 0x0303);
		halrf_wreg(rf, 0x12b8, 0x40000000, 0x0);
		halrf_wreg(rf, 0x32b8, 0x40000000, 0x0);
		halrf_wreg(rf, 0x5864, 0xc0000000, 0x0);
		halrf_wreg(rf, 0x7864, 0xc0000000, 0x0);
		halrf_wreg(rf, 0x2008, 0x01ffffff, 0x0000000);
		halrf_wreg(rf, 0x0c1c, 0x00000004, 0x0);
		halrf_wreg(rf, 0x0700, 0x08000000, 0x0);
		halrf_wreg(rf, 0x0c70, 0x0000001f, 0x03);
		halrf_wreg(rf, 0x0c70, 0x000003e0, 0x03);
		halrf_wreg(rf, 0x12a0, 0x000ff000, 0x00);
		halrf_wreg(rf, 0x32a0, 0x000ff000, 0x00);
		halrf_wreg(rf, 0x0700, 0x07000000, 0x0);

		#if 1 /* jerry recommand to fix */
		/* halrf_wreg(rf, 0x0c3c, 0x00000200, 0x0); */
		halrf_wreg(rf, 0x2344, 0x80000000, 0x0);
		halrf_wreg(rf, 0x20fc, 0xffff0000, 0x0000);
		halrf_wreg(rf, 0x58c8, 0x01000000, 0x0);
		halrf_wreg(rf, 0x78c8, 0x01000000, 0x0);
		halrf_wreg(rf, 0x0c3c, 0x00000200, 0x0); /* block OFDM CCK */
		#else /* default */
		halrf_wreg(rf, 0x0c3c, 0x00000200, 0x0); 
		halrf_wreg(rf, 0x2344, 0x80000000, 0x0);
		halrf_wreg(rf, 0x20fc, 0xffff0000, 0x0000);
		halrf_wreg(rf, 0x58c8, 0x01000000, 0x0);
		halrf_wreg(rf, 0x78c8, 0x01000000, 0x0);
		#endif		
	} else {
		/* dbcc */
		if (phy == HW_PHY_0) {
			halrf_wreg(rf, 0x20fc, 0xffff0000, 0x0101);
			halrf_wreg(rf, 0x12b8, 0x40000000, 0x0);
			halrf_wreg(rf, 0x5864, 0xc0000000, 0x0);
			halrf_wreg(rf, 0x2008, 0x01ffffff, 0x0000000);
			halrf_wreg(rf, 0x0c1c, 0x00000004, 0x0);
			halrf_wreg(rf, 0x0700, 0x08000000, 0x0);
			halrf_wreg(rf, 0x0c70, 0x0000001f, 0x03);
			halrf_wreg(rf, 0x0c70, 0x000003e0, 0x03);
			halrf_wreg(rf, 0x12a0, 0x000ff000, 0x00);
			halrf_wreg(rf, 0x0700, 0x07000000, 0x0);
			halrf_wreg(rf, 0x0c3c, 0x00000200, 0x0);
			halrf_wreg(rf, 0x2344, 0x80000000, 0x0);
			halrf_wreg(rf, 0x20fc, 0xffff0000, 0x0000);
			halrf_wreg(rf, 0x58c8, 0x01000000, 0x0);
		} else if (phy == HW_PHY_1) {
			halrf_wreg(rf, 0x20fc, 0xffff0000, 0x0202);
			halrf_wreg(rf, 0x32b8, 0x40000000, 0x0);
			halrf_wreg(rf, 0x7864, 0xc0000000, 0x0);
			halrf_wreg(rf, 0x2008, 0x01ffffff, 0x0000000);
			halrf_wreg(rf, 0x2c1c, 0x00000004, 0x0);
			halrf_wreg(rf, 0x2700, 0x08000000, 0x0);
			halrf_wreg(rf, 0x0cf0, 0x000001ff, 0x000);
			halrf_wreg(rf, 0x32a0, 0x000ff000, 0x00);
			halrf_wreg(rf, 0x2700, 0x07000000, 0x0);
			halrf_wreg(rf, 0x2c3c, 0x00000200, 0x0);
			halrf_wreg(rf, 0x20fc, 0xffff0000, 0x0000);
			halrf_wreg(rf, 0x78c8, 0x01000000, 0x0);
			halrf_wreg(rf, 0x0c70, 0x0000001f, 0x03);
		}
	}
	
}




void _halrf_do_non_dbcc_txgapk_8851b(struct rf_info *rf,
					enum phl_phy_idx phy)
{
	u8 i;	
	u32 rf_3wire_a_die[2] = {0};
	u32 rf_3wire_d_die[2] = {0};
	

	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s   phy=%d\n", __func__, phy);


	rf_3wire_a_die[RF_PATH_A] = halrf_rrf(rf, RF_PATH_A, 0x5, 0xfffff);
	rf_3wire_d_die[RF_PATH_A] = halrf_rrf(rf, RF_PATH_A, 0x10005, 0xfffff);

	//halrf_txgapk_write_table_default_8851b(rf, phy);
	for (i = 0; i < 1; i++) {
		_halrf_txgapk_track_table_nctl_8851b(rf, phy, i);
		//_halrf_txgapk_write_track_table_8851b(rf, phy, i);
		_halrf_txgapk_power_table_nctl_8851b(rf, phy, i);
		//_halrf_txgapk_write_power_table_8851b(rf, phy, i);

	}


}
void _halrf_txgapk_get_ch_info_8851b(struct rf_info *rf, enum phl_phy_idx phy)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	
	u8 idx = 0;
	u8 get_empty_table = false;

	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s \n", __func__);

	for  (idx = 0;  idx < 2; idx++) {
		if (txgapk_info->txgapk_mcc_ch[idx] == 0) {
			get_empty_table = true;
			break;
		}
	}
	//RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK] (1)  idx = %x\n", idx);

	if (false == get_empty_table) {
		idx = txgapk_info->txgapk_table_idx + 1;
		if (idx > 1) {
			idx = 0;
		}		
		//RF_DBG(rf, DBG_RF_IQK, "[TXGAPK]we will replace iqk table index(%d), !!!!! \n", idx);
	}	
	//RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK] (2)  idx = %x\n", idx);


	txgapk_info->txgapk_table_idx =  idx;
	txgapk_info->txgapk_mcc_ch[idx] = rf->hal_com->band[phy].cur_chandef.center_ch;
	txgapk_info->ch[0] = rf->hal_com->band[phy].cur_chandef.center_ch;	
	



}
void halrf_do_txgapk_8851b(struct rf_info *rf,
					enum phl_phy_idx phy)
{
	struct halrf_gapk_info *txgapk_info = &rf->gapk;

	txgapk_info->is_txgapk_ok = true;
	txgapk_info->r0x8010[0] = halrf_rreg(rf, 0x8010, MASKDWORD);

	_halrf_txgapk_get_ch_info_8851b(rf, phy);
	
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s   phy=%d, dbcc_en = %d, table = %d \n", __func__, phy, rf->hal_com->dbcc_en, txgapk_info->txgapk_table_idx);
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> version = 0x%x\n", TXGAPK_VER_8851B); 
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> before GapK process, 0x%x= 0x%x\n", 0x8010, txgapk_info->r0x8010[0]);


	/* 0:table_0, 1:table_1 */
	halrf_wrf(rf, RF_PATH_A, 0x18, 0x80000, txgapk_info->txgapk_table_idx);
	halrf_wrf(rf, RF_PATH_A, 0x10018, 0x80000, txgapk_info->txgapk_table_idx);
	
	_halrf_do_non_dbcc_txgapk_8851b(rf, phy);

	txgapk_info->r0x8010[1] = halrf_rreg(rf, 0x8010, MASKDWORD);
	RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> After GapK process, 0x%x= 0x%x\n", 0x8010, txgapk_info->r0x8010[1]);
}

void halrf_txgapk_init_8851b(struct rf_info *rf) 
{	
	struct halrf_gapk_info *txgapk_info = &rf->gapk;
	u8 idx;

	if(!txgapk_info->is_gapk_init) {
		RF_DBG(rf, DBG_RF_TXGAPK, "[TXGAPK]======> %s \n", __func__);
		txgapk_info->is_gapk_init = true;
	
		for (idx = 0; idx < 2; idx++) { //channel
			txgapk_info->txgapk_mcc_ch[idx] = 0;
		}
		txgapk_info->txgapk_table_idx = 0;		
	}

}


void halrf_txgapk_enable_8851b
	(struct rf_info *rf, enum phl_phy_idx phy)
{
	return;
}

void halrf_txgapk_write_table_default_8851b
	(struct rf_info *rf, enum phl_phy_idx phy)
{
	return;
}

#endif
