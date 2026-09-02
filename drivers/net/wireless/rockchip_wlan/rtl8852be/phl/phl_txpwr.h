/******************************************************************************
 *
 * Copyright(c) 2021 Realtek Corporation.
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
#ifndef _PHL_TXPWR_H_
#define _PHL_TXPWR_H_

/* Related definition for Realtek defined Dynamic Antenna Gain (DAG) */
#define RTK_DAG_ANT_CNT 2
#define RTK_DAG_5G_SUB_BAND_CNT 4
#define RTK_DAG_6G_SUB_BAND_CNT 6

#pragma pack(1)
/*
 * These struct are for dynamic antenna gain feature.
 * Using antenna gain compensation to modify RF table of related regulatory.
 */
struct rtw_phl_ant_gain_comp {
	s8 ag_2g_comp; /* 2.4G antenna gain compensation */
	s8 ag_5g_comp[RTK_DAG_5G_SUB_BAND_CNT]; /* 5G antenna gain compensation */
	s8 ag_6g_comp[RTK_DAG_6G_SUB_BAND_CNT]; /* 6G antenna gain compensation */
};

struct rtw_phl_regu_dyn_ant_gain {
	u8 regu_country_2g; /* Related halrf_pw_lmt_regulation_type */
	u8 regu_country_5g; /* Related halrf_pw_lmt_regulation_type */
	u8 regu_country_6g; /* Related halrf_pw_lmt_regulation_type */
	struct rtw_phl_ant_gain_comp ag_comp[RTK_DAG_ANT_CNT];
};
#pragma pack()

u8 rtw_phl_get_pw_lmt_regu_type(void *phl, enum band_type band);
int rtw_phl_get_pw_lmt_regu_type_from_str(void *phl, const char *str);
const char *rtw_phl_get_pw_lmt_regu_str_from_type(void *phl, u8 regu);
int rtw_phl_get_pw_lmt_regu_type_of_band_from_str(void *phl
	, enum band_type band, const char *str);
const char *rtw_phl_get_pw_lmt_regu_str_from_type_of_band(void *phl
	, enum band_type band, u8 regu);

const char *rtw_phl_get_pw_lmt_regu_type_str(void *phl, enum band_type band);

bool rtw_phl_pw_lmt_regu_tbl_exist(void *phl, enum band_type band, u8 regu);
u8 rtw_phl_ext_reg_codemap_search(void *phl, u16 domain_code, const char *country, const char **reg_name);
u8 rtw_phl_ext_reg_codemap_of_band_search(void *phl, enum band_type band
	, u16 domain_code, const char *country, const char **reg_name);

bool rtw_phl_get_pwr_lmt_en(void *phl, u8 band_idx);

struct txpwr_regu_info_t *rtw_phl_get_pw_lmt_regu_info(void *phl);
void rtw_phl_free_pw_lmt_regu_info(void *phl, struct txpwr_regu_info_t *info);
bool rtw_phl_is_current_pwr_lmt_regu(void *phl, enum band_type band, u8 regu);

enum rtw_phl_status
phl_cmd_txpwr_ctl_hdl(void *phl, u8 *param_buf);

enum rtw_phl_status rtw_phl_get_txinfo_pwr(void *phl, s16 *pwr_dbm);

#ifdef CONFIG_CMD_DISP
enum rtw_phl_status phl_cmd_tpe_update_hdl(void *phl, u8 *param);
#endif

s8 rtw_phl_get_power_by_rate_band(void *phl, u8 hw_band, u16 rate, u8 dcm, u8 offset, u32 band);
s8 rtw_phl_get_power_limit_option(void *phl, u8 hw_band, u8 rf_path, u16 rate,
	u8 bandwidth, u8 beamforming, u8 tx_num, u8 channel, u32 band, u8 reg);
s8 rtw_phl_get_power_limit_ru_option(void *phl,
	u8 hw_band, u8 rf_path, u16 rate, u8 bandwidth,
	u8 tx_num, u8 channel, u32 band, u8 reg);
u8 rtw_phl_get_tx_tbl_to_tx_pwr_times(void *phl);
s8 rtw_phl_get_power_limit_value_ww(void *phl);
s8 rtw_phl_get_power_limit_value_na(void *phl);
u32 rtw_phl_get_regulation_max_num(void *phl, enum band_type band);
#endif /*_PHL_TXPWR_H_*/
