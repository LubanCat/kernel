/******************************************************************************
 *
 * Copyright(c) 2019 Realtek Corporation.
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
#include "halrf_precomp.h"
#ifdef PHL_PLATFORM_WINDOWS
#ifdef HALRF_TAS_SUPPORT
#include <math.h>


#define IS_DPR_ON(state) (state == HALRF_TAS_STATE_DPR_ON)

/* Convert Power index to mW */
static double _pi_2_mw(s16 pwr_dbm)
{
	double pwr = pwr_dbm;

	pwr = pwr / (4 * 10);
	return pow(10, pwr);
}

/* Convert mW to Power index */
static s16 _mw_2_pi(double pwr_mw)
{
	s16 pwr = 0;
	double tmp, tmp_float;

	if (pwr_mw <= 0)
		return 0;

	tmp = log10(pwr_mw) * 10;

	/*Rounding to the second decimal place*/
	tmp = tmp * 10;

	tmp_float = tmp - (s16)tmp;
	tmp = tmp - tmp_float;

	if (tmp_float > 0 && tmp_float < 0.5)
		tmp = tmp / 10;
	else if (tmp_float >= 0.5 && tmp_float < 1)
		tmp = (tmp + 1) / 10;
	else
		tmp = tmp / 10;

	pwr = (s16)(tmp * 4);
	return pwr;
}

static void _halrf_tas_set_mv_avg(
	struct rf_info *rf)
{
	struct halrf_tas_info *tas_info = &rf->tas_info;
	u8 band = rf->hal_com->band[0].cur_chandef.band;
	u8 reg = halrf_get_regulation_info(rf, band);

	if (band == BAND_ON_24G){
		if (reg == PW_LMT_REGU_IC || reg == PW_LMT_REGU_KCC)
			tas_info->tas_mv_avg = 180;
		else if (reg == PW_LMT_REGU_FCC)
			tas_info->tas_mv_avg = 50;
		else
			tas_info->tas_mv_avg = 50;

	} else if (band == BAND_ON_5G){
		if (reg == PW_LMT_REGU_IC || reg == PW_LMT_REGU_KCC)
			tas_info->tas_mv_avg = 180;
		else if (reg == PW_LMT_REGU_FCC)
			tas_info->tas_mv_avg = 30;
		else
			tas_info->tas_mv_avg = 50;

	} else {	/*BAND_ON_6G*/
		if (reg == PW_LMT_REGU_6G_IC_LPI || reg == PW_LMT_REGU_6G_IC_STD || reg == PW_LMT_REGU_6G_IC_VLP ||
			reg == PW_LMT_REGU_6G_KCC_LPI || reg == PW_LMT_REGU_6G_KCC_STD || reg == PW_LMT_REGU_6G_KCC_VLP)
			tas_info->tas_mv_avg = 180;
		else if (reg == PW_LMT_REGU_6G_FCC_LPI || reg == PW_LMT_REGU_6G_FCC_STD || reg == PW_LMT_REGU_6G_FCC_VLP)
			tas_info->tas_mv_avg = 15;
		else
			tas_info->tas_mv_avg = 50;
	}

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s   band=%d   reg=%d   tas_info->tas_mv_avg=%d\n",
		__func__, band, reg, tas_info->tas_mv_avg);
}

static bool _halrf_tas_get_txpwr(
	struct rf_info *rf,
	s16 *txpwr
	)
{
	struct halrf_tas_info *tas_info = &rf->tas_info;
	s16 pwr_dbm = 0;
	double pwr_mw = 0; 
	u32 i;

	/* reset txpwr value */
	*txpwr = 0;

#if 0
	if (RTW_HAL_STATUS_SUCCESS !=
		rtw_hal_bb_get_tx_info_power(rf->hal_com,
			&pwr_dbm)) /* in hw watchdog, use directly */
		return false;

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ===>%s diver pwr_dbm=%d\n",
		__func__, pwr_dbm);
#endif
	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] tas_info->tas_fw_info.tas_cur_idx=%2d\n", tas_info->tas_fw_info.tas_cur_idx);

	if (tas_info->tas_fw_info.tas_cur_idx > 0 && tas_info->tas_fw_info.tas_cur_idx <= 20) {
		for (i = 0; i < tas_info->tas_fw_info.tas_cur_idx; i++) {
			RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] tas_info->tas_fw_info.tas_txpwr_his[%d]=%2d\n",
				i, tas_info->tas_fw_info.tas_txpwr_his[i]);
			pwr_mw = pwr_mw + _pi_2_mw((s16)tas_info->tas_fw_info.tas_txpwr_his[i]);
		}

		pwr_dbm = _mw_2_pi(pwr_mw / tas_info->tas_fw_info.tas_cur_idx);
	}

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ===>%s fw pwr_dbm=%d\n",
		__func__, pwr_dbm);

	/* set txpwr value */
	*txpwr = pwr_dbm;
	return true;
}

static void _halrf_tas_his_dump(
	struct rf_info *rf)
{
	struct halrf_tas_info *tas_info = &rf->tas_info;
	u8 i = 0;

	for (i = 0; i < tas_info->tas_mv_avg; i++) {
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] txpwr history (ori, r), idx_%02d = %d, %d\n",
			i, tas_info->txpwr_his[i], (u32)(tas_info->txpwr_r_his[i]));
	}

	for (i = 0; i < HALRF_TAS_MV_AVG_TX_RATIO_WINDOW; i++) {
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] tx ratio history idx_%02d = %d\n",
			i, tas_info->tx_ratio_his[i]);
	}
}

static void _halrf_tas_update_sar_limit(
	struct rf_info *rf,
	s8 update_sar_limit)
{
	struct rtw_phl_ext_pwr_lmt_info *ext_pwr_info = &rf->hal_com->band[HW_PHY_0].rtw_tpu_i.ext_pwr_lmt_i;

	ext_pwr_info->ext_pwr_lmt_2_4g[0] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_5g_band1[0] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_5g_band2[0] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_5g_band3[0] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_5g_band4[0] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_6g_unii_5_1[0] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_6g_unii_5_2[0] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_6g_unii_6[0] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_6g_unii_7_1[0] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_6g_unii_7_2[0] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_6g_unii_8[0] = update_sar_limit;

	ext_pwr_info->ext_pwr_lmt_2_4g[1] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_5g_band1[1] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_5g_band2[1] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_5g_band3[1] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_5g_band4[1] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_6g_unii_5_1[1] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_6g_unii_5_2[1] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_6g_unii_6[1] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_6g_unii_7_1[1] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_6g_unii_7_2[1] = update_sar_limit;
	ext_pwr_info->ext_pwr_lmt_6g_unii_8[1] = update_sar_limit;
}


static void _halrf_tas_state_decision(
	struct rf_info *rf)
{
	struct halrf_tas_info *tas_info = &rf->tas_info;
	s16 dpr_on_th = (s16)tas_info->dpr_on_th;
	s16 dpr_off_th = (s16)tas_info->dpr_off_th;
	s16 sar_org_lmt = 0;

	halrf_tas_get_th(rf, &dpr_on_th, &dpr_off_th, &sar_org_lmt);

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] =======> %s\n", __func__);

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] tas_info->tx_ratio_limit_en=%d\n",
		tas_info->tx_ratio_limit_en);

	tas_info->tas_delta = HALRF_TAS_SAR_LMT_GAP + HALRF_TAS_DPR_GAP * 2;  // 2 dBm

	if (tas_info->cur_txpwr_avg >= dpr_on_th) {
		/* Tx power avg over dpr_on threshold, turn ON SAR */
		tas_info->cur_state = HALRF_TAS_STATE_DPR_ON;
		tas_info->tas_off_delta_pwr = 0;
		halrf_power_limit_set_ext_pwr_limit_table(rf, HW_PHY_0);
		halrf_power_limit_set_ext_pwr_limit_ru_table(rf, HW_PHY_0);
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] txpwr_avg(%d) >= DPR_ON_TH(%d), state(DPR_ON)\n",
			tas_info->cur_txpwr_avg, dpr_on_th);
	} else if (tas_info->cur_txpwr_avg < dpr_off_th && ((tas_info->cur_tx_ratio_avg < 70) || !(tas_info->tx_ratio_limit_en))) {
#if 0
		/* Tx power avg below dpr_off threshold, turn OFF SAR */
		tas_info->cur_state = HALRF_TAS_STATE_DPR_OFF;
		hal->ext_pwr_lmt_en = false;
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] txpwr_avg(%d) < DPR_OFF_TH(%d), state(DPR_OFF)\n",
			tas_info->cur_txpwr_avg, dpr_off_th);
#else
		/* Tx power avg below dpr_off threshold, turn OFF SAR */
		tas_info->cur_state = HALRF_TAS_STATE_DPR_OFF;
		tas_info->tas_off_delta_pwr = tas_info->tas_delta + (HALRF_TAS_DPR_GAP * 4); /*4 dBm*/
		halrf_power_limit_set_ext_pwr_limit_table(rf, HW_PHY_0);
		halrf_power_limit_set_ext_pwr_limit_ru_table(rf, HW_PHY_0);
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] txpwr_avg(%d) < DPR_OFF_TH(%d), state(DPR_OFF)\n",
			tas_info->cur_txpwr_avg, dpr_off_th);
#endif
	} else if (tas_info->cur_txpwr_avg < dpr_on_th && tas_info->cur_tx_ratio_avg >= 70 && tas_info->tx_ratio_limit_en) {
		/* Tx power avg in between dpr_on/dpr_off thresholds, keep original state */
		/* Tx ratio avg >= 70, keep static SAR */

		tas_info->cur_state = HALRF_TAS_STATE_STATIC_SAR;
		tas_info->tas_off_delta_pwr = tas_info->tas_delta; /*Static SAR*/
		halrf_power_limit_set_ext_pwr_limit_table(rf, HW_PHY_0);
		halrf_power_limit_set_ext_pwr_limit_ru_table(rf, HW_PHY_0);
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] between ON(%d)/OFF(%d), Tx ratio avg(%d) >= 70 keep state(%s)\n",
			dpr_on_th, dpr_off_th, tas_info->cur_tx_ratio_avg,
			(IS_DPR_ON(tas_info->cur_state) ? "DPR_ON" :
			((tas_info->cur_state == HALRF_TAS_STATE_STATIC_SAR) ? "STATIC SAR" : "DPR_OFF")));
	} else {
		/* Tx power avg in between dpr_on/dpr_off thresholds, keep original state */
		tas_info->cur_state = tas_info->last_state;
		tas_info->tas_pwr_over_sar_lmt_check = true;
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] between ON(%d)/OFF(%d), keep state(%s)\n",
			dpr_on_th, dpr_off_th, (IS_DPR_ON(tas_info->cur_state) ? "DPR_ON" : "DPR_OFF"));
	}

	/* call cmd to update ext pwr lmt */
	halrf_set_power(rf, HW_PHY_0, (PWR_LIMIT | PWR_LIMIT_RU));

	/* update last state */
	tas_info->last_state = tas_info->cur_state;
}

/* Calculate average power in the unit of mW */
static void _halrf_calc_avg_pwr(
	struct rf_info *rf,
	u8 tx_ratio /* 0~100, percentage */
	)
{
	struct halrf_tas_info *tas_info = &rf->tas_info;
	s16 pwr_dbm = 0;
	double pwr_mw = 0; 
	double total_txpwr = 0;
	double total_tx_ratio = 0;
	double txpwr_avg = 0;
	u8 i = 0;
	u8 ratio = tx_ratio;

	/*
	 * Special case: if idle tx pwr use -40 dBm (and tx ratio set to 100)
	 * suggested by RF Alvin
	 */

	if (tx_ratio == 0) {
		pwr_dbm = -40 * 4;
		ratio = 100;
	} else {
		/* get current txpwr */
		if (false == _halrf_tas_get_txpwr(rf, &pwr_dbm)) {
			RF_DBG(rf, DBG_RF_TAS, "[RF_TAS], update sts fail\n");
			return;
		}
	}

	tas_info->txpwr_his[tas_info->cur_idx] = pwr_dbm;
	tas_info->last_txpwr = pwr_dbm;

	tas_info->tx_ratio_his[tas_info->cur_radio_idx] = tx_ratio;

	/* Calculate average power on mW unit */
	pwr_mw = _pi_2_mw(pwr_dbm) * ratio; /* txpwr x ratio */

	tas_info->txpwr_r_his[tas_info->cur_idx] = pwr_mw;
	for (i = 0; i < tas_info->tas_mv_avg; i++) {
		total_txpwr += tas_info->txpwr_r_his[i];
	}

	for (i = 0; i < HALRF_TAS_MV_AVG_TX_RATIO_WINDOW; i++) {
		total_tx_ratio += tas_info->tx_ratio_his[i];
	}

	/* divided by 100 because tx_ratio is represented in percentage */
	txpwr_avg = ((total_txpwr / tas_info->tas_mv_avg) / 100);
	tas_info->cur_tx_ratio_avg = (u8)(total_tx_ratio / HALRF_TAS_MV_AVG_TX_RATIO_WINDOW);

	tas_info->cur_txpwr_avg = _mw_2_pi(txpwr_avg);

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] idx(%d), txpwr(%d), ratio_idx(%d), tx_ratio(%d), txpwr_avg = %d, tx_ratio_avg = %d\n",
		tas_info->cur_idx, pwr_dbm, tas_info->cur_radio_idx, tx_ratio, tas_info->cur_txpwr_avg, tas_info->cur_tx_ratio_avg);
}

static void _halrf_tas_update_sts(
	struct rf_info *rf,
	u8 tx_ratio /* 0~100, percentage */
	)
{
	struct halrf_tas_info *tas_info = &rf->tas_info;

	/* calc avg txpwr */
	_halrf_calc_avg_pwr(rf, tx_ratio);

	/* debug dump TAS history */
	_halrf_tas_his_dump(rf);

	/* determine TAS state */
	_halrf_tas_state_decision(rf);

	/* update tx power idx */
	if ((tas_info->cur_idx + 1) >= (u8)tas_info->tas_mv_avg)
		tas_info->cur_idx = 0;
	else
		tas_info->cur_idx++;

	/* update tx ratio idx */
	if ((tas_info->cur_radio_idx + 1) >= HALRF_TAS_MV_AVG_TX_RATIO_WINDOW)
		tas_info->cur_radio_idx = 0;
	else
		tas_info->cur_radio_idx++;	
}

static void _halrf_tas_default_txpwr_txratio(
	struct rf_info *rf)
{
	struct halrf_tas_info *tas_info = &rf->tas_info;
	s16 dpr_on_th, dpr_off_th , sar_org_lmt;
	s16 pwr_dbm = 0;
	u8 i = 0;

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s\n", __func__);

	_halrf_tas_set_mv_avg(rf);

	halrf_tas_get_th(rf, &dpr_on_th, &dpr_off_th, &sar_org_lmt);

	for (i = 0; i < tas_info->tas_mv_avg; i++) {
		//pwr_dbm = HALRF_TAS_DEF_PWR;
		pwr_dbm = sar_org_lmt;
		rf->tas_info.txpwr_his[i] = pwr_dbm;
		rf->tas_info.txpwr_r_his[i] =
			_pi_2_mw(pwr_dbm) * HALRF_TAS_DEF_RATIO;
	}

	for (i = 0; i < HALRF_TAS_MV_AVG_TX_RATIO_WINDOW; i++)
		rf->tas_info.tx_ratio_his[i] = HALRF_TAS_DEF_RATIO;

}

static void _halrf_update_tas_cfg(
	struct rf_info *rf)
{
	struct halrf_tas_info *tas_info = &rf->tas_info;
	u8 tas_en = tas_info->tas_en & BIT0;

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] =======> %s   tas_info->tas_mv_avg=%d\n",
		__func__, tas_info->tas_mv_avg);

	_halrf_tas_default_txpwr_txratio(rf);

	// === Temp turn off TAS ===
	tas_info->tas_en &= 0xFE; // clear BIT0 to disable tas
	tas_info->cur_idx = 0;
	tas_info->cur_radio_idx = 0;
	tas_info->last_txpwr = 0;
	tas_info->cur_txpwr_avg = 0;
	tas_info->cur_ratio = 0;
	tas_info->tas_off_delta_pwr = 0;
	tas_info->cur_state = HALRF_TAS_STATE_DPR_OFF;
	tas_info->last_state = HALRF_TAS_STATE_DPR_OFF;
	tas_info->tx_ratio_limit_en = true;

	switch (tas_info->tas_p_idx) {
	default:
	case HALRF_PATTERN_1:
	case HALRF_PATTERN_2:
	case HALRF_PATTERN_3:
	case HALRF_PATTERN_4:
	case HALRF_PATTERN_5:
	case HALRF_PATTERN_6:
	case HALRF_PATTERN_7:
	case HALRF_PATTERN_8:
		tas_info->dpr_gap = HALRF_TAS_DPR_GAP; // 1 dBm
		tas_info->tas_delta = HALRF_TAS_SAR_LMT_GAP + HALRF_TAS_DPR_GAP * 2;  // 2 dBm
		tas_info->tas_delta_show_cli = tas_info->tas_delta;
		break;
#if 0
	case HALRF_PATTERN_5:
		tas_info->dpr_gap = HALRF_TAS_DPR_GAP; // 1 dBm
		tas_info->tas_delta = HALRF_TAS_SAR_LMT_GAP + HALRF_TAS_DPR_GAP * 3;  // 3 dBm
		break;
	case HALRF_PATTERN_6:
		tas_info->dpr_gap = HALRF_TAS_DPR_GAP * 2; // 2 dBm
		tas_info->tas_delta = HALRF_TAS_SAR_LMT_GAP + HALRF_TAS_DPR_GAP * 4;  // 4 dBm
		break;
#endif
	}

	// === turn TAS back on ===
	if (tas_en)
		tas_info->tas_en |= 0x01; // set BIT0 to enable tas
}

void halrf_tas_get_th(
	struct rf_info *rf,
	s16 *dpr_on_th,
	s16 *dpr_off_th,
	s16 *sar_org_lmt)
{
	struct halrf_tas_info *tas_info = &rf->tas_info;
	struct rtw_phl_ext_pwr_lmt_info *ext_pwr_info = &rf->hal_com->band[HW_PHY_0].rtw_tpu_i.ext_pwr_lmt_i;
	s8 ext_pwr_lmt = 0x7f;
	u8 channel = rf->hal_com->band[HW_PHY_0].cur_chandef.center_ch;
	u8 band = rf->hal_com->band[HW_PHY_0].cur_chandef.band;

	*dpr_on_th = (u16)tas_info->dpr_on_th;
	*dpr_off_th = (u16)tas_info->dpr_off_th;

	if (band == BAND_ON_24G)	/*2G*/
		ext_pwr_lmt = ext_pwr_info->ext_pwr_lmt_2_4g[0];
	else if (band == BAND_ON_5G && (channel >= 36 && channel <= 48))	/*5G*/
		ext_pwr_lmt = ext_pwr_info->ext_pwr_lmt_5g_band1[0];
	else if (band == BAND_ON_5G && (channel >= 50 && channel <= 64))
		ext_pwr_lmt = ext_pwr_info->ext_pwr_lmt_5g_band2[0];
	else if (band == BAND_ON_5G && (channel >= 100 && channel <= 144))
		ext_pwr_lmt = ext_pwr_info->ext_pwr_lmt_5g_band3[0];
	else if (band == BAND_ON_5G && (channel >= 149 && channel <= 177))
		ext_pwr_lmt = ext_pwr_info->ext_pwr_lmt_5g_band4[0];
	else if (band == BAND_ON_6G && (channel >= 1 && channel <= 45))	/*6G*/
		ext_pwr_lmt = ext_pwr_info->ext_pwr_lmt_6g_unii_5_1[0];
	else if (band == BAND_ON_6G && (channel >= 47 && channel <= 93))
		ext_pwr_lmt = ext_pwr_info->ext_pwr_lmt_6g_unii_5_2[0];
	else if (band == BAND_ON_6G && (channel >= 97 && channel <= 117))
		ext_pwr_lmt = ext_pwr_info->ext_pwr_lmt_6g_unii_6[0];
	else if (band == BAND_ON_6G && (channel >= 119 && channel <= 149))
		ext_pwr_lmt = ext_pwr_info->ext_pwr_lmt_6g_unii_7_1[0];
	else if (band == BAND_ON_6G && (channel >= 151 && channel <= 189))
		ext_pwr_lmt = ext_pwr_info->ext_pwr_lmt_6g_unii_7_2[0];
	else if (band == BAND_ON_6G && (channel >= 193 && channel <= 253))
		ext_pwr_lmt = ext_pwr_info->ext_pwr_lmt_6g_unii_8[0];
	else
		return;

	*sar_org_lmt = (s16)ext_pwr_lmt;

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s   ext_pwr_lmt=%d\n",
		__func__, ext_pwr_lmt);
}

void halrf_tas_watchdog(
	struct rf_info *rf)
{
	struct rtw_env_report env_rpt = {0};
	struct halrf_tas_info *tas_info = &(rf->tas_info);
	u8 channel = rf->hal_com->band[HW_PHY_0].cur_chandef.center_ch;
	u8 band = rf->hal_com->band[0].cur_chandef.band;
	bool tas_en = tas_info->tas_en & BIT0;
	s16 dpr_on_th = (s16)tas_info->dpr_on_th;
	s16 dpr_off_th = (s16)tas_info->dpr_off_th;
	s16 sar_org_lmt = 0;
	s8 dpr_on = 0;
	s32 int_tmp[2], float_tmp[2];
	u32 tmp[2] = {0};

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s\n", __func__);

//	if (rf->tas_info.case_id_update) {
//		RF_DBG(rf, DBG_RF_TAS, "%s, update case id to [%d]\n", __func__, rf->tas_info.tas_case_id);
//		_halrf_update_tas_cfg(rf);
//		return;
//	}

	if (!(rf->support_ability & HAL_RF_TAS)) {
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s, rf->support_ability=0x%x Return!!!\n",
			__func__, rf->support_ability);
		return;
	}

	if (tas_info->tas_pause) {
		if (tas_info->tas_pause_time <= HALRF_TAS_PAUSE_IC_TIME)
			tas_info->tas_pause_time++;

		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %s, skip, tas_pause(0x%02X), tas_info->tas_pause_time=%d\n",
			__func__, rf->tas_info.tas_pause, tas_info->tas_pause_time);
		return;
	}

	if (!halrf_is_tas_en(rf)) {
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %s, skip, tas_en(0x%02X)\n", __func__,
			rf->tas_info.tas_en);
		return;
	}

	if (tas_info->last_channel == 0 && tas_info->last_band == 0) {
		tas_info->last_channel = channel;
		tas_info->last_band = band;
	} else {
		if (tas_info->last_channel != channel || tas_info->last_band != band) {
			RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %s, last_channel=%d last_band=%d, channel=%d band=%d\n",
				__func__,
				tas_info->last_channel, tas_info->last_band,
				channel, band);

			_halrf_tas_default_txpwr_txratio(rf);

			tas_info->last_channel = channel;
			tas_info->last_band = band;
		}
	}

	_halrf_tas_set_mv_avg(rf);

	/* get nhm_tx_ratio from rtw_env_report */
	rtw_hal_bb_env_rpt(rf->hal_com, &env_rpt, HW_PHY_0);

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s   env_rpt.rpt_status=%d\n",
		__func__, env_rpt.rpt_status);

	if (env_rpt.rpt_status) { /* rpt_status: 1 = ok, 0 = fail */
		/* query nhm_tx_ratio ok */
		rf->tas_info.cur_ratio = env_rpt.nhm_tx_ratio;
		_halrf_tas_update_sts(rf, env_rpt.nhm_tx_ratio);

		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s   env_rpt.nhm_tx_ratio=%d\n",
			__func__, env_rpt.nhm_tx_ratio);
	} else {
		/* query nhm_tx_ratio fail */
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %s, query tx ratio fail, skip this round\n", __func__);
	}

	halrf_tas_get_th(rf, &dpr_on_th, &dpr_off_th, &sar_org_lmt);

	if ((tas_info->cur_txpwr_avg > sar_org_lmt + HALRF_TAS_DPR_GAP) && (tas_info->tas_pwr_over_sar_lmt_check))	/*+1 dBm*/
		tas_info->tas_pwr_over_sar_lmt++;

	/*Debug view debug message*/
	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = 0x%x\n",
		"TAS Version", TAS_VER);

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = %s / (%d)\n",
		"TAS Status / Case_id", ((tas_en) ? "Enable" : "Disable"), tas_info->tas_case_id);

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = %d\n",
		"TAS Over cnt", tas_info->tas_pwr_over_sar_lmt);

	if (dpr_on_th > halrf_get_tas_delta(rf))
		dpr_on = (s8)(sar_org_lmt - halrf_get_tas_delta(rf));
	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = %d.%02d / %d.%02d / %d.%02d\n",
		"TAS ON_TH / OFF_TH / DPR_ON",
		dpr_on_th / 4, dpr_on_th * 100 / 4 % 100,
		dpr_off_th / 4, dpr_off_th * 100 / 4 % 100,
		dpr_on / 4, dpr_on * 100 / 4 % 100);

	int_tmp[0] = tas_info->cur_txpwr_avg / 4;
	float_tmp[0] = tas_info->cur_txpwr_avg * 100 / 4 % 100;
	float_tmp[0] < 0 ? float_tmp[0] = float_tmp[0] * -1 : 0;
	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = %3d.%02d / %s\n",
		"TAS TXPWR_AVG / STATE",
		int_tmp[0], float_tmp[0],
		((tas_info->cur_state == HALRF_TAS_STATE_DPR_ON) ? "DPR_ON" :
		((tas_info->cur_state == HALRF_TAS_STATE_STATIC_SAR) ? "STATIC_SAR" : "DPR_OFF")));

	int_tmp[0] = tas_info->last_txpwr / 4;
	float_tmp[0] = tas_info->last_txpwr * 100 / 4 % 100;
	float_tmp[0] < 0 ? float_tmp[0] = float_tmp[0] * -1 : 0;
	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = %3d / %d.%02d\n",
		"TAS TX_RATIO / TXPWR",
		tas_info->cur_ratio,
		int_tmp[0], float_tmp[0]);

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s =  %d\n",
		"TAS TX_RATIO_AVG",
		tas_info->cur_tx_ratio_avg);

	tmp[0] = halrf_rreg(rf, 0x56cc, 0x1ff);
	tmp[1] = halrf_rreg(rf, 0x76cc, 0x1ff);
	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = 0x%x(%d.%d) / 0x%x(%d.%d)\n",
		"TX ext Pwr diff A / B reg",
		tmp[0],
		tmp[0] / 4, ((tmp[0] * 100) / 4) % 100,
		tmp[1],
		tmp[1] / 4, ((tmp[1] * 100) / 4) % 100);

	tmp[0] = halrf_rreg(rf, 0x1c78, 0x1ff);
	tmp[1] = halrf_rreg(rf, 0x3c78, 0x1ff);
	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = %d.%d / %d.%d\n",
		"T-MAC xdbm A / B",
		tmp[0] / 4, ((tmp[0] * 100) / 4) % 100,
		tmp[1] / 4, ((tmp[1] * 100) / 4) % 100);

	if (tas_info->tas_pwr_by_rate_ctrl_value < 0)
		float_tmp[0] = -1 *(tas_info->tas_pwr_by_rate_ctrl_value / 2 % 2) * 10 / 2;
	else
		float_tmp[0] = (tas_info->tas_pwr_by_rate_ctrl_value / 2 % 2) * 10 / 2;
	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = %s / %d.%d dBm\n",
		"TAS test Item / Power",
		(tas_info->tas_test_item == HALRF_TAS_SET_PWR_BY_RATE_N_DBM) ? "N dBm" :
			((tas_info->tas_test_item == HALRF_TAS_SET_PWR_BY_RATE_HALF_SAR_LIMIT) ? "0.5 SAR" :
			((tas_info->tas_test_item == HALRF_TAS_SET_PWR_BY_RATE_SAR_LIMIT_ADD4) ? "SAR + 4" : "Default")),
		(tas_info->tas_pwr_by_rate_ctrl_value / 4), float_tmp[0]);

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = %d / %d\n",
		"TAS delta / TAS off delta pwr",
		tas_info->tas_delta, tas_info->tas_off_delta_pwr);

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = %d\n",
		"TAS TX ration en",
		tas_info->tx_ratio_limit_en);

	RF_DBG(rf, DBG_RF_TAS, " %-30s = %d / %d\n",
		"TAS TX pause / Times",
		tas_info->tas_pause, tas_info->tas_pause_time);
}

void halrf_tas_powersaving_watchdog(
	struct rf_info *rf)
{
	struct halrf_tas_info *tas_info = &rf->tas_info;
	bool tas_en = tas_info->tas_en & BIT0;
	s16 dpr_on_th = (s16)tas_info->dpr_on_th;
	s16 dpr_off_th = (s16)tas_info->dpr_off_th;
	s16 sar_org_lmt = 0;
	s8 dpr_on = 0;
	s32 int_tmp[2], float_tmp[2];

	/*Don't read, write register in the API*/

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s\n", __func__);

	if (!(rf->support_ability & HAL_RF_TAS)) {
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s, rf->support_ability=0x%x Return!!!\n",
			__func__, rf->support_ability);
		return;
	}

	if (tas_info->tas_pause) {
		if (tas_info->tas_pause_time <= HALRF_TAS_PAUSE_IC_TIME)
			tas_info->tas_pause_time++;

		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %s, skip, tas_pause(0x%02X), tas_info->tas_pause_time=%d\n",
			__func__, rf->tas_info.tas_pause, tas_info->tas_pause_time);
		return;
	}

	if (!halrf_is_tas_en(rf)) {
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %s, skip, tas_en(0x%02X)\n", __func__,
			rf->tas_info.tas_en);
		return;
	}

	_halrf_tas_set_mv_avg(rf);

	/* calc avg txpwr */
	_halrf_calc_avg_pwr(rf, 0);

	/* debug dump TAS history */
	_halrf_tas_his_dump(rf);

	/* update tx power idx */
	if ((tas_info->cur_idx + 1) >= (u8)tas_info->tas_mv_avg)
		tas_info->cur_idx = 0;
	else
		tas_info->cur_idx++;

	/* update tx ratio idx */
	if ((tas_info->cur_radio_idx + 1) >= HALRF_TAS_MV_AVG_TX_RATIO_WINDOW)
		tas_info->cur_radio_idx = 0;
	else
		tas_info->cur_radio_idx++;

	halrf_tas_get_th(rf, &dpr_on_th, &dpr_off_th, &sar_org_lmt);

	/*Debug view debug message*/
	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = %s / (%d)\n",
		"TAS Status / Case_id", ((tas_en) ? "Enable" : "Disable"), tas_info->tas_case_id);

	RF_DBG(rf, DBG_RF_TAS, " %-30s = %d\n",
		"TAS Over cnt", tas_info->tas_pwr_over_sar_lmt);

	if (dpr_on_th > halrf_get_tas_delta(rf))
		dpr_on = (s8)(sar_org_lmt - halrf_get_tas_delta(rf));
	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = %d.%02d / %d.%02d / %d.%02d\n",
		"TAS ON_TH / OFF_TH / DPR_ON",
		dpr_on_th / 4, dpr_on_th * 100 / 4 % 100,
		dpr_off_th / 4, dpr_off_th * 100 / 4 % 100,
		dpr_on / 4, dpr_on * 100 / 4 % 100);

	int_tmp[0] = tas_info->cur_txpwr_avg / 4;
	float_tmp[0] = tas_info->cur_txpwr_avg * 100 / 4 % 100;
	float_tmp[0] < 0 ? float_tmp[0] = float_tmp[0] * -1 : 0;
	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = %3d.%02d / %s\n",
		"TAS TXPWR_AVG / STATE",
		int_tmp[0], float_tmp[0],
		((tas_info->cur_state == HALRF_TAS_STATE_DPR_ON) ? "DPR_ON" :
		((tas_info->cur_state == HALRF_TAS_STATE_STATIC_SAR) ? "STATIC_SAR" : "DPR_OFF")));

	int_tmp[0] = tas_info->last_txpwr / 4;
	float_tmp[0] = tas_info->last_txpwr * 100 / 4 % 100;
	float_tmp[0] < 0 ? float_tmp[0] = float_tmp[0] * -1 : 0;
	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s = %3d / %d.%02d\n",
		"TAS TX_RATIO / TXPWR",
		tas_info->cur_ratio,
		int_tmp[0], float_tmp[0]);

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %-30s =  %d\n",
		"TAS TX_RATIO_AVG",
		tas_info->cur_tx_ratio_avg);
}

void halrf_tas_start(
	struct rf_info *rf)
{
	struct halrf_tas_info *tas_info = &rf->tas_info;

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s\n", __func__);

	if (!(rf->support_ability & HAL_RF_TAS)) {
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s, rf->support_ability=0x%x Return!!!\n",
			__func__, rf->support_ability);
		return;
	}

	if (tas_info->tas_ext_pwr_lmt_en_check == false) {
		tas_info->tas_ext_pwr_lmt_en_tmp = rf->hal_com->band[HW_PHY_0].rtw_tpu_i.ext_pwr_lmt_en;
		tas_info->tas_ext_pwr_lmt_en_check = true;
	}

	rf->hal_com->band[HW_PHY_0].rtw_tpu_i.ext_pwr_lmt_en = true;
	rf->tas_info.tas_en |= BIT(0);

	_halrf_update_tas_cfg(rf);
	halrf_tas_update_dpr_th(rf);
}

void halrf_tas_stop(
	struct rf_info *rf)
{
	struct rtw_hal_com_t *hal = rf->hal_com;
	struct halrf_tas_info *tas_info = &rf->tas_info;

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s\n", __func__);

	if (!(rf->support_ability & HAL_RF_TAS)) {
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s, rf->support_ability=0x%x Return!!!\n",
			__func__, rf->support_ability);
		return;
	}

	rf->tas_info.tas_en &= ~BIT0;

	/* call cmd to update ext pwr lmt */
	tas_info->tas_delta = 0;
	tas_info->tas_case_id = 0;
	tas_info->tas_off_delta_pwr = 0;

	if (tas_info->tas_ext_pwr_lmt_en_check == true) {
		rf->hal_com->band[HW_PHY_0].rtw_tpu_i.ext_pwr_lmt_en = tas_info->tas_ext_pwr_lmt_en_tmp;
		tas_info->tas_ext_pwr_lmt_en_check = false;
	}

	halrf_power_limit_set_ext_pwr_limit_table(rf, HW_PHY_0);
	halrf_power_limit_set_ext_pwr_limit_ru_table(rf, HW_PHY_0);
	halrf_set_power(rf, HW_PHY_0, (PWR_LIMIT | PWR_LIMIT_RU));
}


void halrf_update_tas_def_setting(
	struct rf_info *rf, u32 tas_config)
{
	struct halrf_tas_info *tas_info = &rf->tas_info;
	u8 dpr_on_th = 0;
	u8 dpr_off_th = 0;
	u8 dpr_gap = 0;
	u8 tas_delta = 0;
	u8 i = 0;
	s16 pwr_dbm = 0;

	/*
	 * tas_config
	 * byte0: tas_en, meaning is described as follows
	 * byte0[7:1]: rsvd
	 * byte0[0]: en, 1 = tas enable, 0 = tas disable
	 * byte1: dpr_on_th, unit=0.25 dBm
	 * byte2: dpr_off_th, unit=0.25 dBm
	 * byte3: tas_diff, meaning is described as follows
	 * byte3[2:0]: dpr_gap, 0~7, unit=0.5 dBm, start from 0.5, i.e. 0.5~4 dBm
	 * byte3[6:3]: tas_delta, 0~15, unit=0.5 dBm, i.e. 0~7.5 dBm
	 * byte3[7]: tas_diff_valid, 1 = tas_diff will take effect, 0 = use driver default
	 */

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %s, tas_config(0x%08X)\n", __func__, tas_config);
	/* Byte0 */
	rf->tas_info.tas_en = (u8)(tas_config & 0xFF);
	/* Byte1, Byte2 */
	dpr_on_th = (u8)((tas_config & 0xFF00) >> 8);
	dpr_off_th = (u8)((tas_config & 0xFF0000) >> 16);
	if ((dpr_on_th > dpr_off_th) &&
		(dpr_on_th < HALRF_TAS_MAX_TH) && (dpr_off_th < HALRF_TAS_MAX_TH)) {
		rf->tas_info.dpr_on_th = dpr_on_th;
		rf->tas_info.dpr_off_th = dpr_off_th;
	} else {
		rf->tas_info.dpr_on_th = HALRF_TAS_DPR_ON_TH;
		rf->tas_info.dpr_off_th = HALRF_TAS_DPR_OFF_TH;
	}
	/* Byte3 */
	if ((tas_config & BIT31)) { /* Byte3[7] tas_diff_valid */
		dpr_gap = (u8)((((tas_config & 0x07000000) >> 24) + 1) * 2);
		tas_delta = (u8)(((tas_config & 0x78000000) >> 27) * 2);
	} else {
		/* tas_diff_valid not set, use driver default */
		dpr_gap = HALRF_TAS_DPR_GAP;
		tas_delta = HALRF_TAS_SAR_LMT_GAP + HALRF_TAS_DELTA;
	}
	rf->tas_info.dpr_gap = dpr_gap;
	rf->tas_info.tas_delta = tas_delta;

	/* update array default value */
	for (i = 0; i < tas_info->tas_mv_avg; i++) {
		pwr_dbm = HALRF_TAS_DEF_PWR;
		rf->tas_info.txpwr_his[i] = pwr_dbm;
		rf->tas_info.txpwr_r_his[i] =
			_pi_2_mw(pwr_dbm) * HALRF_TAS_DEF_RATIO;
	}

	for (i = 0; i < HALRF_TAS_MV_AVG_TX_RATIO_WINDOW; i++)
		rf->tas_info.tx_ratio_his[i] = HALRF_TAS_DEF_RATIO;


	if (rf->tas_info.tas_ext_pwr_lmt_en_check == false) {
		rf->tas_info.tas_ext_pwr_lmt_en_tmp = rf->hal_com->band[HW_PHY_0].rtw_tpu_i.ext_pwr_lmt_en;
		rf->tas_info.tas_ext_pwr_lmt_en_check = true;
	}

	rf->tas_info.tas_pwr_over_sar_lmt = 0;
	rf->tas_info.tas_pwr_over_sar_lmt_check = false;

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] %s, tas_en(0x%02X), dpr_on_th(%d), dpr_off_th(%d), dpr_gap(%d), tas_delta(%d), pattern(%d)\n", __func__,
			rf->tas_info.tas_en,
			rf->tas_info.dpr_on_th,
			rf->tas_info.dpr_off_th,
			rf->tas_info.dpr_gap,
			rf->tas_info.tas_delta,
			rf->tas_info.tas_p_idx);
}

static const char *_halrf_frac_to_str(u32 pwr)
{
	u32 remainder = pwr % 4;

	switch (remainder) {
	case 0:
		return "";
	break;
	case 1:
		return ".25";
	break;
	case 2:
		return ".5";
	break;
	case 3:
		return ".75";
	break;
	default:
		return "";
	break;
	};
}

void halrf_tas_enable(struct rf_info *rf, char input[][16], u32 *_used,
				char *output, u32 *_out_len)

{
	struct halrf_tas_info *tas_info = &rf->tas_info;
	struct rtw_hal_com_t *hal = rf->hal_com;
	u32 val = 0;
	u32 pattern = 0;

	if (!(rf->support_ability & HAL_RF_TAS)) {
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s, rf->support_ability=0x%x Return!!!\n",
			__func__, rf->support_ability);
		return;
	}

	_os_sscanf(input[2], "%d", &val);
	if (val > 1) {
		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			"\n invalid input(%d), TAS enable should be 0 or 1\n", val);
		return;
	}

	if ((val == 0 && !halrf_is_tas_en(rf))
		|| (val == 1 && halrf_is_tas_en(rf))) {
		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			"\n TAS Status no change.\n");
		return;
	}
	
	if (val == 0) {
		halrf_tas_stop(rf);
		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			"\n Disable TAS manually ");
	} else {
		halrf_tas_start(rf);
		_halrf_update_tas_cfg(rf);
		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			"\n Enable TAS manually");
	}

	RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used, "\n");
}

void halrf_tas_case_id(struct rf_info *rf, char input[][16], u32 *_used,
				char *output, u32 *_out_len)

{
	struct halrf_tas_info *tas_info = &(rf->tas_info);
	struct rtw_hal_com_t *hal = rf->hal_com;
	u32 val = 0;
	u32 pattern = 0;
	s16 dpr_on_th, dpr_off_th , sar_org_lmt;
	//s8 sar_limit[8] = {48, 48, 48, 54, 76, 40, 60, 32};
	s8 sar_limit[8] = {56, 56, 50, 54, 18, 18, 18, 32};

	if (!(rf->support_ability & HAL_RF_TAS)) {
		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] ======> %s, rf->support_ability=0x%x Return!!!\n",
			__func__, rf->support_ability);
		return;
	}

	_os_sscanf(input[2], "%d", &val);

	if (val > 8 || val < 1) {
		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			"\n invalid input(%d), case id should be 1~8\n", val);
		return;
	}

	RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		"\n TAS case id set to [%d]", val);

#if 0
	if (val <= 7) { // test case 1~6
		pattern = val - 1;
		_halrf_tas_update_sar_limit(rf, sar_limit[val - 1]);
	} else if (val == 7) { // test case 7
		pattern = HALRF_PATTERN_3;
		_halrf_tas_update_sar_limit(rf, sar_limit[HALRF_PATTERN_3]);
	} else if (val == 8) { // test case 8
		pattern = HALRF_PATTERN_7;
		_halrf_tas_update_sar_limit(rf, sar_limit[HALRF_PATTERN_7]);
	} else if (val == 9) { // test case 9
		pattern = HALRF_PATTERN_8;
		_halrf_tas_update_sar_limit(rf, sar_limit[HALRF_PATTERN_8]);
	}
#else
	pattern = val - 1;
	_halrf_tas_update_sar_limit(rf, sar_limit[val - 1]);

#endif
	tas_info->tas_case_id = (u8)val;
	tas_info->tas_p_idx = (u8)pattern;
	tas_info->tas_en |= BIT0;
	tas_info->tas_pwr_over_sar_lmt = 0;
	tas_info->tas_pwr_over_sar_lmt_check = false;

	if (tas_info->tas_ext_pwr_lmt_en_check == false) {
		tas_info->tas_ext_pwr_lmt_en_tmp = rf->hal_com->band[HW_PHY_0].rtw_tpu_i.ext_pwr_lmt_en;
		tas_info->tas_ext_pwr_lmt_en_check = true;
	}

	rf->hal_com->band[HW_PHY_0].rtw_tpu_i.ext_pwr_lmt_en = true;

	halrf_power_limit_set_ext_pwr_limit_table(rf, HW_PHY_0);
	halrf_power_limit_set_ext_pwr_limit_ru_table(rf, HW_PHY_0);

	_halrf_update_tas_cfg(rf);
	halrf_tas_get_th(rf, &dpr_on_th, &dpr_off_th, &sar_org_lmt);
	tas_info->dpr_on_th = sar_org_lmt - HALRF_TAS_SAR_LMT_GAP;
	tas_info->dpr_off_th = sar_org_lmt - HALRF_TAS_SAR_LMT_GAP - tas_info->dpr_gap;
	RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		"\n sar_lmt=%d  dpr_on_th=%d  dpr_off_th=%d  dpr_gap=%d\n",
		sar_org_lmt, tas_info->dpr_on_th, tas_info->dpr_off_th, tas_info->dpr_gap);
}

void halrf_tas_info_dump(struct rf_info *rf, char input[][16], u32 *_used,
				char *output, u32 *_out_len)
{
	struct halrf_tas_info *tas_info = &(rf->tas_info);
	struct rtw_hal_com_t *hal = rf->hal_com;
	bool support = true;
	bool tas_en = false;
	s16 dpr_on_th = (u16)tas_info->dpr_on_th;
	s16 dpr_off_th = (u16)tas_info->dpr_off_th;
	s16 sar_org_lmt = 0;
	s8 dpr_on = 0;
	u32 db_factor = 4;
#define _halrf_pwr_to_str(pwr, factor) (pwr / factor), _halrf_frac_to_str(pwr)

	RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		"\n [========== HALRF TAS INFO ==========] case_id(%d) ",
		tas_info->tas_case_id);
	if (support) {
		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"\n %-30s = 0x%x", "TAS Version", TAS_VER);

		tas_en = (tas_info->tas_en & BIT0);
		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"\n Status: %s", ((tas_en) ? "Enable" : "Disable"));
		if (!tas_en)
			goto exit; /* skip detail if tas not enabled */

		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"\n TAS Over cnt = %d", tas_info->tas_pwr_over_sar_lmt);

		halrf_tas_get_th(rf, &dpr_on_th, &dpr_off_th, &sar_org_lmt);
		if (dpr_on_th > tas_info->tas_delta_show_cli)
			dpr_on = (s8)(sar_org_lmt - tas_info->tas_delta_show_cli);
		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"\n DPR_ON_TH: %d%s, DPR_OFF_TH: %d%s, DPR_ON: %d%s",
				_halrf_pwr_to_str(dpr_on_th, db_factor),
				_halrf_pwr_to_str(dpr_off_th, db_factor),
				_halrf_pwr_to_str(dpr_on, db_factor));
		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"\n CUR_TXPWR_AVG: %s%d%s, CUR_STATE: %s",
				(tas_info->cur_txpwr_avg < 0) ? "-" : "",
				_halrf_pwr_to_str(
					((tas_info->cur_txpwr_avg < 0) ? (-1 * tas_info->cur_txpwr_avg) : (tas_info->cur_txpwr_avg)), db_factor),
				((tas_info->cur_state == HALRF_TAS_STATE_DPR_ON) ? "DPR_ON" :
				((tas_info->cur_state == HALRF_TAS_STATE_STATIC_SAR) ? "STATIC_SAR" : "DPR_OFF")));
		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"\n CUR_RATIO: %d, TXPWR: %s%d%s",
				tas_info->cur_ratio,
				(tas_info->last_txpwr < 0) ? "-" : "",
				_halrf_pwr_to_str(
					((tas_info->last_txpwr < 0) ? (-1 * tas_info->last_txpwr) : (tas_info->last_txpwr)), db_factor));
		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"\n TX_RATIO_AVG: %d",
				tas_info->cur_tx_ratio_avg);
	} else {
		/* Not supported if BIOS SAR unavailable */
		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
				"\n Status: N/A");
	}
exit:
	RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used, "\n");
}

void halrf_tas_tx_ratio_lmt_en(struct rf_info *rf, char input[][16], u32 *_used,
				char *output, u32 *_out_len)

{
	struct halrf_tas_info *tas_info = &(rf->tas_info);
	u32 val = 0;

	_os_sscanf(input[2], "%d", &val);

	tas_info->tx_ratio_limit_en = (bool)val;

	RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		"tx_ratio_limit_en = %d\n", tas_info->tx_ratio_limit_en);
}

void halrf_tas_config(struct rf_info *rf, char input[][16], u32 *_used,
				char *output, u32 *_out_len)

{
	struct halrf_tas_info *tas_info = &(rf->tas_info);
	u32 tas_config = 0x0;

	_os_sscanf(input[2], "%x", &tas_config);

	halrf_update_tas_def_setting(rf, tas_config);

	RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		"tas_config = 0x%x\n", tas_config);
}

void halrf_tas_set_power_by_rate_n_dbm(struct rf_info *rf, char input[][16], u32 *_used,
						char *output, u32 *_out_len)
{
	struct halrf_tas_info *tas_info = &(rf->tas_info);
	s32 tmp;

	_os_sscanf(input[2], "%d", &tmp);

	if (tmp > -80 && tmp < 120) { /*tmp range is -20dBm ~ 30dBm*/
		tas_info->tas_pwr_by_rate_ctrl = true;
		tas_info->tas_test_item = HALRF_TAS_SET_PWR_BY_RATE_N_DBM;
		tas_info->tas_pwr_by_rate_ctrl_value = tmp;	/*unit: 0.25 dBm (multiply by 4) */

		if (tas_info->tas_pwr_by_rate_ctrl_value < 0)
			tmp = -1 *(tas_info->tas_pwr_by_rate_ctrl_value % 4) * 100 / 4;
		else
			tmp = (tas_info->tas_pwr_by_rate_ctrl_value % 4) * 100 / 4;

		if (rf->hal_com->dbcc_en)
			halrf_set_power(rf, HW_PHY_1, PWR_BY_RATE);

		halrf_set_power(rf, HW_PHY_0, PWR_BY_RATE);

		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			"Set tas power by rate = %d.%d dBm\n",
			(tas_info->tas_pwr_by_rate_ctrl_value / 4), tmp);
	} else {
		tas_info->tas_pwr_by_rate_ctrl = false;
		tas_info->tas_test_item = HALRF_TAS_SET_PWR_BY_RATE_TO_DEFAULT;
		tas_info->tas_pwr_by_rate_ctrl_value = tmp;	/*unit: 0.25 dBm (multiply by 4) */

		if (tas_info->tas_pwr_by_rate_ctrl_value < 0)
			tmp = -1 *(tas_info->tas_pwr_by_rate_ctrl_value % 4) * 100 / 4;
		else
			tmp = (tas_info->tas_pwr_by_rate_ctrl_value % 4) * 100 / 4;

		RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
			"Set tas power by rate = %d.%d dBm, out off range -20dBm ~ 30dBm\n",
			(tas_info->tas_pwr_by_rate_ctrl_value / 4), tmp);

		tas_info->tas_pwr_by_rate_ctrl_value = 0;	/*unit: 0.25 dBm (multiply by 4) */
	}
}

void halrf_tas_set_power_by_rate_half_sar_limit(struct rf_info *rf, char input[][16], u32 *_used,
						char *output, u32 *_out_len)
{
	struct halrf_tas_info *tas_info = &(rf->tas_info);

	tas_info->tas_pwr_by_rate_ctrl = true;
	tas_info->tas_test_item = HALRF_TAS_SET_PWR_BY_RATE_HALF_SAR_LIMIT;

	if (rf->hal_com->dbcc_en)
		halrf_set_power(rf, HW_PHY_1, PWR_BY_RATE);

	halrf_set_power(rf, HW_PHY_0, PWR_BY_RATE);

	RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		"Set tas power by rate = 0.5 times sar limit\n");
}

void halrf_tas_set_power_by_rate_sar_limit_add_4db(struct rf_info *rf, char input[][16], u32 *_used,
						char *output, u32 *_out_len)
{
	struct halrf_tas_info *tas_info = &(rf->tas_info);

	tas_info->tas_pwr_by_rate_ctrl = true;
	tas_info->tas_test_item = HALRF_TAS_SET_PWR_BY_RATE_SAR_LIMIT_ADD4;

	if (rf->hal_com->dbcc_en)
		halrf_set_power(rf, HW_PHY_1, PWR_BY_RATE);

	halrf_set_power(rf, HW_PHY_0, PWR_BY_RATE);

	RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		"Set tas power by rate = sar limit + 4 dB\n");
}

void halrf_tas_set_power_by_rate_to_default(struct rf_info *rf, char input[][16], u32 *_used,
						char *output, u32 *_out_len)
{
	struct halrf_tas_info *tas_info = &(rf->tas_info);

	tas_info->tas_pwr_by_rate_ctrl = false;
	tas_info->tas_test_item = HALRF_TAS_SET_PWR_BY_RATE_TO_DEFAULT;
	tas_info->tas_pwr_by_rate_ctrl_value = 0;

	if (rf->hal_com->dbcc_en)
		halrf_set_power(rf, HW_PHY_1, PWR_BY_RATE);

	halrf_set_power(rf, HW_PHY_0, PWR_BY_RATE);

	RF_DBG_CNSL(*_out_len, *_used, output + *_used, *_out_len - *_used,
		"Set tas power by rate to default\n");
}

#endif	/*HALRF_TAS_SUPPORT*/
#endif	/*PHL_PLATFORM_WINDOWS*/

void halrf_tas_set_oft_to_zero_scan_start(struct rf_info *rf)
{
#ifdef PHL_PLATFORM_WINDOWS
#ifdef HALRF_TAS_SUPPORT

	struct halrf_tas_info *tas_info = &rf->tas_info;

	if (halrf_is_tas_en(rf)) {
		tas_info->tas_delta_tmp = tas_info->tas_delta;
		tas_info->tas_off_delta_pwr_tmp = tas_info->tas_off_delta_pwr;
	} else {
		tas_info->tas_delta_tmp = 0;
		tas_info->tas_off_delta_pwr_tmp = 0;
	}

	tas_info->tas_delta = 0;
	tas_info->tas_off_delta_pwr = 0;

	halrf_power_limit_set_ext_pwr_limit_table(rf, HW_PHY_0);
	halrf_power_limit_set_ext_pwr_limit_ru_table(rf, HW_PHY_0);
	halrf_set_power(rf, HW_PHY_0, (PWR_LIMIT | PWR_LIMIT_RU));

#endif	/*HALRF_TAS_SUPPORT*/
#endif	/*PHL_PLATFORM_WINDOWS*/
}

void halrf_tas_set_oft_scan_end(struct rf_info *rf)
{
#ifdef PHL_PLATFORM_WINDOWS
#ifdef HALRF_TAS_SUPPORT

	struct halrf_tas_info *tas_info = &rf->tas_info;

	if (halrf_is_tas_en(rf)) {
		tas_info->tas_delta = tas_info->tas_delta_tmp;
		tas_info->tas_off_delta_pwr = tas_info->tas_off_delta_pwr_tmp;
	} else {
		tas_info->tas_delta_tmp = 0;
		tas_info->tas_off_delta_pwr_tmp = 0;

		tas_info->tas_delta = 0;
		tas_info->tas_off_delta_pwr = 0;
	}

	halrf_power_limit_set_ext_pwr_limit_table(rf, HW_PHY_0);
	halrf_power_limit_set_ext_pwr_limit_ru_table(rf, HW_PHY_0);
	halrf_set_power(rf, HW_PHY_0, (PWR_LIMIT | PWR_LIMIT_RU));

#endif	/*HALRF_TAS_SUPPORT*/
#endif	/*PHL_PLATFORM_WINDOWS*/
}

void halrf_tas_update_dpr_th(struct rf_info *rf)
{
#ifdef PHL_PLATFORM_WINDOWS
#ifdef HALRF_TAS_SUPPORT
	struct halrf_tas_info *tas_info = &(rf->tas_info);
	s16 dpr_on_th, dpr_off_th , sar_org_lmt;

	if (rf->tas_info.tas_en & BIT0) {
		halrf_tas_get_th(rf, &dpr_on_th, &dpr_off_th, &sar_org_lmt);
		tas_info->dpr_on_th = sar_org_lmt - HALRF_TAS_SAR_LMT_GAP;
		tas_info->dpr_off_th = sar_org_lmt - HALRF_TAS_SAR_LMT_GAP - tas_info->dpr_gap;
	}
#endif	/*HALRF_TAS_SUPPORT*/
#endif	/*PHL_PLATFORM_WINDOWS*/
}

s8 halrf_tas_get_test_item_power(struct rf_info *rf)
{
#ifdef PHL_PLATFORM_WINDOWS
#ifdef HALRF_TAS_SUPPORT
	struct halrf_tas_info *tas_info = &(rf->tas_info);
	s16 dpr_on_th, dpr_off_th, sar_org_lmt;
	s8 tmp = 0;

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] =======> %s   tas_info->tas_test_item = %d\n",
		__func__, tas_info->tas_test_item);

	switch (tas_info->tas_test_item) {
	case HALRF_TAS_SET_PWR_BY_RATE_N_DBM:

		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] =======> %s   tas_info->tas_pwr_by_rate_ctrl_value = %d\n",
			__func__, tas_info->tas_pwr_by_rate_ctrl_value);
		
		return (s8)(tas_info->tas_pwr_by_rate_ctrl_value);
		break;
	case HALRF_TAS_SET_PWR_BY_RATE_HALF_SAR_LIMIT:
		halrf_tas_get_th(rf, &dpr_on_th, &dpr_off_th, &sar_org_lmt);

		tmp = (s8)(sar_org_lmt / 2);
		tas_info->tas_pwr_by_rate_ctrl_value = (s32)tmp;

		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] =======> %s   sar_org_lmt / 2 = %d\n",
			__func__, tmp);

		return tmp;
		break;
	case HALRF_TAS_SET_PWR_BY_RATE_SAR_LIMIT_ADD4:
		halrf_tas_get_th(rf, &dpr_on_th, &dpr_off_th, &sar_org_lmt);

		tmp = (s8)(sar_org_lmt + HALRF_TAS_SAR_LMT_GAP * 4);
		tas_info->tas_pwr_by_rate_ctrl_value = (s32)tmp;

		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] =======> %s   sar_org_lmt + 4 dB = %d\n",
			__func__, tmp);

		return tmp;	/*sar limit +4 dB*/
		break;
	default:
		return tmp;
		break;
	}
#endif	/*HALRF_TAS_SUPPORT*/
#endif	/*PHL_PLATFORM_WINDOWS*/

	RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] <======= %s\n", __func__);

	return 0;
}

void halrf_tas_pause(struct rf_info *rf, bool is_stop)
{
#ifdef PHL_PLATFORM_WINDOWS
#ifdef HALRF_TAS_SUPPORT
	struct halrf_tas_info *tas_info = &rf->tas_info;
	u8 band = rf->hal_com->band[0].cur_chandef.band;
	u8 reg;

	tas_info->tas_pause = is_stop;

	if (tas_info->tas_pause) {
		tas_info->cur_state = HALRF_TAS_STATE_STATIC_SAR;
		tas_info->tas_off_delta_pwr = tas_info->tas_delta; /*Static SAR*/

		halrf_power_limit_set_ext_pwr_limit_table(rf, HW_PHY_0);
		halrf_power_limit_set_ext_pwr_limit_ru_table(rf, HW_PHY_0);
		
		halrf_set_power(rf, HW_PHY_0, (PWR_LIMIT | PWR_LIMIT_RU));
		if (rf->hal_com->dbcc_en)
			halrf_set_power(rf, HW_PHY_0, (PWR_LIMIT | PWR_LIMIT_RU));

		RF_DBG(rf, DBG_RF_TAS, "[RF_TAS] =======> %s   is_stop=%d\n", __func__, is_stop);
	
		tas_info->tas_pause_time = 0;
	} else {
		reg = halrf_get_regulation_info(rf, band);
		if (band == BAND_ON_24G || band == BAND_ON_5G){
			if (reg == PW_LMT_REGU_IC) {
				if (tas_info->tas_pause_time >= HALRF_TAS_PAUSE_IC_TIME)
					_halrf_tas_default_txpwr_txratio(rf);
			} else {
				if (tas_info->tas_pause_time >= HALRF_TAS_PAUSE_FCC_TIME)
					_halrf_tas_default_txpwr_txratio(rf);
			}	
		} else {	/*BAND_ON_6G*/
			if (reg == PW_LMT_REGU_6G_IC_LPI || reg == PW_LMT_REGU_6G_IC_STD || reg == PW_LMT_REGU_6G_IC_VLP) {
				if (tas_info->tas_pause_time >= HALRF_TAS_PAUSE_IC_TIME)
					_halrf_tas_default_txpwr_txratio(rf);
			} else {
				if (tas_info->tas_pause_time >= HALRF_TAS_PAUSE_FCC_TIME)
					_halrf_tas_default_txpwr_txratio(rf);
			}
		}

		tas_info->tas_pause_time = 0;
	}

#endif	/*HALRF_TAS_SUPPORT*/
#endif	/*PHL_PLATFORM_WINDOWS*/
}


