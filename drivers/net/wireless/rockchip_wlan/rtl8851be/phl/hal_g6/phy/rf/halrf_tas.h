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
#ifndef _HALRF_TAS_H_
#define _HALRF_TAS_H_

#ifdef PHL_PLATFORM_WINDOWS
#ifdef HALRF_TAS_SUPPORT

#define TAS_VER 0x1

#define HALRF_TAS_MV_AVG_WINDOW 180 /* watchdog period is 2s, target window is 360 sec, 360/2 = 180 */
#define HALRF_TAS_MV_AVG_TX_RATIO_WINDOW 6 /* watchdog period is 2s, target window is 12 sec, 12/2 = 6 */
#define HALRF_TAS_PAUSE_FCC_TIME 50 /* watchdog period is 2s, target window is 100 sec, 100/2 = 50, TAS pause 50 timees, reset TX AVG power */
#define HALRF_TAS_PAUSE_IC_TIME HALRF_TAS_MV_AVG_WINDOW /* watchdog period is 2s, target window is 360 sec, 360/2 = 180, TAS pause 180 timees, reset TX AVG power */
#define HALRF_TAS_WATCHDOG_TIME 2
#define HALRF_TAS_DEF_PWR 80 /* 20 dBm, unit: 0.25 dBm (multiply by 4) */
#define HALRF_TAS_DEF_RATIO 80 /* default tx ratio 80% */
#define HALRF_TAS_DPR_ON_TH 24 /* 6 dBm, unit: 0.25 dBm (multiply by 4) */
#define HALRF_TAS_DPR_OFF_TH 16 /* 4 dBm, unit: 0.25 dBm (multiply by 4) */
#define HALRF_TAS_SAR_LMT_GAP 4 /* 1 dBm, unit: 0.25 dBm (multiply by 4) */
#define HALRF_TAS_DPR_GAP 4 /* 1 dBm, unit: 0.25 dBm (multiply by 4) */
#define HALRF_TAS_DELTA 8 /* 2 dBm, unit: 0.25 dBm (multiply by 4) */
#define HALRF_TAS_UNDEF 0xFF
#define HALRF_TAS_MAX_TH 0x7F
 
enum halrf_tas_state {
	HALRF_TAS_STATE_DPR_ON = 0,
	HALRF_TAS_STATE_DPR_OFF = 1,
	HALRF_TAS_STATE_STATIC_SAR = 2,
};
 
enum halrf_tas_p_idx {
	HALRF_PATTERN_1 = 0,
	HALRF_PATTERN_2 = 1,
	HALRF_PATTERN_3 = 2,
	HALRF_PATTERN_4 = 3,
	HALRF_PATTERN_5 = 4,
	HALRF_PATTERN_6 = 5,
	HALRF_PATTERN_7 = 6,
	HALRF_PATTERN_8 = 7,
};

enum halrf_tas_test_item {
	HALRF_TAS_SET_PWR_BY_RATE_TO_DEFAULT = 0,
	HALRF_TAS_SET_PWR_BY_RATE_N_DBM,
	HALRF_TAS_SET_PWR_BY_RATE_HALF_SAR_LIMIT,
	HALRF_TAS_SET_PWR_BY_RATE_SAR_LIMIT_ADD4,
};

struct halrf_tas_fw_info {
    u32 tas_cur_idx;
    s16 tas_txpwr_his[20];
};
 
struct halrf_tas_info {
	u32 tas_mv_avg;
	s16 txpwr_his[HALRF_TAS_MV_AVG_WINDOW];
	s16 tx_ratio_his[HALRF_TAS_MV_AVG_TX_RATIO_WINDOW];
	s16 last_txpwr;
	double txpwr_r_his[HALRF_TAS_MV_AVG_WINDOW]; /* txpwr x ratio */
	s16 cur_txpwr_avg;
	u8 cur_tx_ratio_avg;
	u8 cur_ratio;
	u8 cur_state;
	u8 last_state;
	u8 cur_idx;
	u8 cur_radio_idx;
	s8 dpr_on_th;
	s8 dpr_off_th;
	u8 dpr_gap;
	u8 tas_delta;
	u8 tas_delta_show_cli;
	s8 tas_off_delta_pwr;
	u8 tas_delta_tmp;
	s8 tas_off_delta_pwr_tmp;
	u8 tas_en;
	u8 tas_pause;
	u32 tas_pause_time;
	u8 tas_p_idx;
	u8 tas_case_id;
	struct rtw_phl_ext_pwr_lmt_info sar_lmt;
	struct halrf_tas_fw_info tas_fw_info;
	enum halrf_tas_test_item tas_test_item;
	bool tx_ratio_limit_en;
	bool tas_ext_pwr_lmt_en_tmp;
	bool tas_ext_pwr_lmt_en_check;
	u32 tas_pwr_over_sar_lmt;
	bool tas_pwr_over_sar_lmt_check;
	bool tas_pwr_by_rate_ctrl;
	s32 tas_pwr_by_rate_ctrl_value;		/* unit: 0.25 dBm (multiply by 4) */
	u8 last_channel;
	u8 last_band;
};

void halrf_tas_get_th(struct rf_info *rf, s16 *dpr_on_th, s16 *dpr_off_th, s16 *sar_org_lmt);

void halrf_tas_watchdog(struct rf_info *rf);

void halrf_tas_powersaving_watchdog(struct rf_info *rf);

void halrf_tas_enable(struct rf_info *rf, char input[][16], u32 *_used,
				char *output, u32 *_out_len);
void halrf_tas_case_id(struct rf_info *rf, char input[][16], u32 *_used,
				char *output, u32 *_out_len);
void halrf_tas_info_dump(struct rf_info *rf, char input[][16], u32 *_used,
				char *output, u32 *_out_len);
void halrf_tas_tx_ratio_lmt_en(struct rf_info *rf, char input[][16], u32 *_used,
				char *output, u32 *_out_len);

void halrf_tas_config(struct rf_info *rf, char input[][16], u32 *_used,
				char *output, u32 *_out_len);

void halrf_tas_set_power_by_rate_n_dbm(struct rf_info *rf, char input[][16], u32 *_used,
					char *output, u32 *_out_len);

void halrf_tas_set_power_by_rate_half_sar_limit(struct rf_info *rf, char input[][16], u32 *_used,
						char *output, u32 *_out_len);

void halrf_tas_set_power_by_rate_sar_limit_add_4db(struct rf_info *rf, char input[][16], u32 *_used,
						char *output, u32 *_out_len);

void halrf_tas_set_power_by_rate_to_default(struct rf_info *rf, char input[][16], u32 *_used,
						char *output, u32 *_out_len);

#define halrf_is_tas_en(rf) (rf->tas_info.tas_en & BIT0)
#define halrf_get_tas_delta(rf) (rf->tas_info.tas_delta)


#endif /*HALRF_TAS_SUPPORT*/
#endif	/*PHL_PLATFORM_WINDOWS*/

void halrf_tas_set_oft_to_zero_scan_start(struct rf_info *rf);

void halrf_tas_set_oft_scan_end(struct rf_info *rf);

void halrf_tas_update_dpr_th(struct rf_info *rf);

s8 halrf_tas_get_test_item_power(struct rf_info *rf);

void halrf_tas_pause(struct rf_info *rf, bool is_stop);

#endif	/*_HALRF_TAS_H_*/
