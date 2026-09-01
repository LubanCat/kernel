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
#define _RTW_TXPWR_C_

#include <drv_types.h>

static const s16 _mb_of_ntx[] = {
	0,		/* 1TX */
	301,	/* 2TX */
	477,	/* 3TX */
	602,	/* 4TX */
	699,	/* 5TX */
	778,	/* 6TX */
	845,	/* 7TX */
	903,	/* 8TX */
};

/* get mB(100 *dB) for specifc TX count relative to 1TX */
s16 mb_of_ntx(u8 ntx)
{
	if (ntx == 0 || ntx > 8) {
		RTW_ERR("ntx=%u, out of range\n", ntx);
		rtw_warn_on(1);
		return 0;
	}

	return _mb_of_ntx[ntx - 1];
}

/*
* input with txpwr value in unit of mBm
* return txpwr in unit of TX Gain Index
*/
s8 txpwr_mbm_to_txgi_s8_with_max(s16 mbm, u8 txgi_max, u8 txgi_pdbm)
{
	s16 max_mbm = (txgi_max > S8_MAX ? S8_MAX : txgi_max) * MBM_PDBM / txgi_pdbm;
	s16 min_mbm = S8_MIN * MBM_PDBM  / txgi_pdbm;

	mbm = rtw_min(max_mbm, mbm);
	mbm = rtw_max(min_mbm, mbm);

	return mbm * txgi_pdbm / MBM_PDBM;
}

/*
* input with txpwr value in unit of txpwr index
* return string in length 6 at least (for -xx.xx)
*/
void txpwr_idx_get_dbm_str(s8 idx, u8 txgi_max, s8 txgi_ww, u8 txgi_pdbm, SIZE_T cwidth, char dbm_str[], u8 dbm_str_len)
{
	char fmt[16];

	if (idx == txgi_max) {
		snprintf(fmt, 16, "%%%zus", cwidth >= 6 ? cwidth + 1 : 6);
		snprintf(dbm_str, dbm_str_len, fmt, "NA");
	} else if (idx == txgi_ww) {
		snprintf(fmt, 16, "%%%zus", cwidth >= 6 ? cwidth + 1 : 6);
		snprintf(dbm_str, dbm_str_len, fmt, "WW");
	} else if (idx > -txgi_pdbm && idx < 0) { /* -0.xx */
		snprintf(fmt, 16, "%%%zus-0.%%02d", cwidth >= 6 ? cwidth - 4 : 1);
		snprintf(dbm_str, dbm_str_len, fmt, "", (rtw_abs(idx) % txgi_pdbm) * 100 / txgi_pdbm);
	} else if (idx % txgi_pdbm) { /* d.xx */
		snprintf(fmt, 16, "%%%zud.%%02d", cwidth >= 6 ? cwidth - 2 : 3);
		snprintf(dbm_str, dbm_str_len, fmt, idx / txgi_pdbm, (rtw_abs(idx) % txgi_pdbm) * 100 / txgi_pdbm);
	} else { /* d */
		snprintf(fmt, 16, "%%%zud", cwidth >= 6 ? cwidth + 1 : 6);
		snprintf(dbm_str, dbm_str_len, fmt, idx / txgi_pdbm);
	}
}

/*
* input with txpwr value in unit of mbm
* return string in length 6 at least (for -xx.xx)
*/
void txpwr_mbm_get_dbm_str(s16 mbm, SIZE_T cwidth, char dbm_str[], u8 dbm_str_len)
{
	char fmt[16];

	if (mbm == UNSPECIFIED_MBM) {
		snprintf(fmt, 16, "%%%zus", cwidth >= 6 ? cwidth + 1 : 6);
		snprintf(dbm_str, dbm_str_len, fmt, "NA");
	} else if (mbm > -MBM_PDBM && mbm < 0) { /* -0.xx */
		snprintf(fmt, 16, "%%%zus-0.%%02d", cwidth >= 6 ? cwidth - 4 : 1);
		snprintf(dbm_str, dbm_str_len, fmt, "", (rtw_abs(mbm) % MBM_PDBM) * 100 / MBM_PDBM);
	} else if (mbm % MBM_PDBM) { /* d.xx */
		snprintf(fmt, 16, "%%%zud.%%02d", cwidth >= 6 ? cwidth - 2 : 3);
		snprintf(dbm_str, dbm_str_len, fmt, mbm / MBM_PDBM, (rtw_abs(mbm) % MBM_PDBM) * 100 / MBM_PDBM);
	} else { /* d */
		snprintf(fmt, 16, "%%%zud", cwidth >= 6 ? cwidth + 1 : 6);
		snprintf(dbm_str, dbm_str_len, fmt, mbm / MBM_PDBM);
	}
}

void rtw_update_txpwr_level(struct dvobj_priv *dvobj, enum phl_band_idx band_idx)
{
	struct rf_ctl_t *rfctl = dvobj_to_rfctl(dvobj);

	rtw_txpwr_hal_update_pwr(dvobj, band_idx);

	rtw_rfctl_update_op_mode(rfctl, 0, 0, 0);
}

void rtw_update_txpwr_level_all_hwband(struct dvobj_priv *dvobj)
{
	rtw_update_txpwr_level(dvobj, HW_BAND_MAX);
}

void dump_tx_power_ext_info(void *sel, struct dvobj_priv *dvobj)
{
	struct tx_power_ext_info info;
	struct {
		const char *str;
		struct txpwr_param_status *status;
	} params[] = {
		{"tx_power_by_rate", &info.by_rate},
		{"tx_power_limit", &info.lmt},
		#ifdef CONFIG_80211AX_HE
		{"tx_power_limit_ru", &info.lmt_ru},
		#endif
		#if CONFIG_IEEE80211_BAND_6GHZ
		{"tx_power_limit_6g", &info.lmt_6g},
		{"tx_power_limit_ru_6g", &info.lmt_ru_6g},
		#endif
	};
	u8 num_of_param = sizeof(params) / sizeof(params[0]);
	u8 i;

	if (rtw_txpwr_hal_get_ext_info(dvobj, &info)) {
		for (i = 0; i < num_of_param; i++) {
			RTW_PRINT_SEL(sel, "%s: %s, %s, %s\n", params[i].str
				, params[i].status->enable ? "enabled" : "disabled"
				, params[i].status->loaded ? "loaded" : "unloaded"
				, params[i].status->external_src ? "file" : "default"
			);
		}
	} else
		RTW_PRINT_SEL(sel, "not ready\n");
}

void dump_txpwr_tpc_settings(void *sel, struct dvobj_priv *dvobj)
{
	struct rf_ctl_t *rfctl = dvobj_to_rfctl(dvobj);

	if (rfctl->tpc_mode == TPC_MODE_DISABLE)
		RTW_PRINT_SEL(sel, "mode:DISABLE(%d)\n", rfctl->tpc_mode);
	else if (rfctl->tpc_mode == TPC_MODE_MANUAL) {
		RTW_PRINT_SEL(sel, "mode:MANUAL(%d)\n", rfctl->tpc_mode);
		RTW_PRINT_SEL(sel, "constraint:%d (mB)\n", rfctl->tpc_manual_constraint);
	}
}

