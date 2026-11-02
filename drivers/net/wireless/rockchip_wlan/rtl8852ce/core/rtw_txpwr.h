/******************************************************************************
 *
 * Copyright(c) 2007 - 2022 Realtek Corporation.
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
#ifndef __RTW_TXPWR_H__
#define __RTW_TXPWR_H__

#define MBM_PDBM 100
#define UNSPECIFIED_MBM 32767 /* maximum of s16 */

#define TPC_MODE_DISABLE	0
#define TPC_MODE_MANUAL		1
#define TPC_MODE_INVALID	2	/* keep last */

#define TPC_MANUAL_CONSTRAINT_MAX 600 /* mB */

struct txpwr_param_status {
	bool enable;
	bool loaded;
	bool external_src;
};

#define SET_TXPWR_PARAM_STATUS(_status, _enable, _loaded, _ext_src) \
	do { \
		(_status)->enable = _enable; \
		(_status)->loaded = _loaded; \
		(_status)->external_src = _ext_src; \
	} while (0)

struct tx_power_ext_info {
	struct txpwr_param_status by_rate;
	struct txpwr_param_status lmt;
#ifdef CONFIG_80211AX_HE
	struct txpwr_param_status lmt_ru;
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	struct txpwr_param_status lmt_6g;
	struct txpwr_param_status lmt_ru_6g;
#endif
};

s16 mb_of_ntx(u8 ntx);
s8 txpwr_mbm_to_txgi_s8_with_max(s16 mbm, u8 txgi_max, u8 txgi_pdbm);

void txpwr_idx_get_dbm_str(s8 idx, u8 txgi_max, s8 txgi_ww, u8 txgi_pdbm, SIZE_T cwidth, char dbm_str[], u8 dbm_str_len);
void txpwr_mbm_get_dbm_str(s16 mbm, SIZE_T cwidth, char dbm_str[], u8 dbm_str_len);

void rtw_update_txpwr_level(struct dvobj_priv *dvobj, enum phl_band_idx band_idx);
void rtw_update_txpwr_level_all_hwband(struct dvobj_priv *dvobj);

void dump_tx_power_ext_info(void *sel, struct dvobj_priv *dvobj);
void dump_txpwr_tpc_settings(void *sel, struct dvobj_priv *dvobj);

#endif /* __RTW_TXPWR_H__ */