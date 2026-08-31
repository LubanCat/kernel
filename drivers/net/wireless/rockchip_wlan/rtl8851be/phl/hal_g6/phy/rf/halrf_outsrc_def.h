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
#ifndef __HALRF_OUTSRC_DEF_H__
#define __HALRF_OUTSRC_DEF_H__

struct rf_info;

struct halrf_fem_info {
	u8 elna_2g;		/*@with 2G eLNA  NO/Yes = 0/1*/
	u8 elna_5g;		/*@with 5G eLNA  NO/Yes = 0/1*/
	u8 elna_6g;		/*@with 6G eLNA  NO/Yes = 0/1*/
	u8 epa_2g;		/*@with 2G ePA    NO/Yes = 0/1*/
	u8 epa_5g;		/*@with 5G ePA    NO/Yes = 0/1*/
	u8 epa_6g;		/*@with 6G ePA    NO/Yes = 0/1*/
};

enum halrf_rfk_err_type {
	HALRF_RFK_ERR_IQK = BIT(0),
	HALRF_RFK_ERR_DPK = BIT(1),
	HALRF_RFK_ERR_DACK = BIT(2),
	HALRF_RFK_ERR_RXDCK = BIT(3),
	HALRF_RFK_ERR_TXGAPK = BIT(4),
	HALRF_RFK_ERR_TSSI = BIT(5),
};

enum halrf_notify_type {
	HALRF_NOTIFY_UNDEF = 0,
	HALRF_NOTIFY_RFK_ERR = 1,
	HALRF_NOTIFY_MAX = 0xFF
};

/* err code version follow framework */
struct halrf_err_code_v1 {
	u32 iqk;
	u32 dpk;
	u32 dack;
	u32 rxdck;
	u32 txgapk;
	u32 tssi;
};

/* IQK Report */
struct halrf_ex_iqk_rpt_v1 {
	u32 iqk_dz_lok[2][4]; /* path/value */
	u32 iqk_dz_tx_xym[2][4]; /* path/value */
	u32 iqk_dz_rx_xym[2][4]; /* path/value */
	u32 iqk_dz_rx_rxbb[2][4]; /* path/value */
	u32 iqk_dz_s0_rxsram[4][1280]; /* group/value */
	u32 iqk_dz_s1_rxsram[4][1280]; /* group/value */
};

/* DPK Report */
struct halrf_ex_dpk_rpt_v1 {
	u32 dpk_rxsram[4][512];
	u32 dpk_pas[4][32];
};

/* RT Report */
struct halrf_ex_rt_rpt_v2 {
	u32 ch_info[10][2][3]; //last10 /cv
	u32 tssi_code[10][2]; //tssi/path
	u32 drv_lck_fail_count;
	u32 fw_lck_fail_count;
};

struct halrf_ex_rt_rpt_v1 {
	u32 ch_info[10][2][2]; //last10 /cv
	u8 tssi_code[10][2]; //tssi/path
	u32 drv_lck_fail_count;
	u32 fw_lck_fail_count;
};

/* TXGAPK Report */
struct halrf_ex_txgapk_rpt_v1 {
	u32 txgapk_dz_max_nctl_cnt; 
	s32 txgapk_dz_track_d[2][17];
	s32 txgapk_dz_power_d[2][17];
};


/*
 * Note: when change happen on err code or sub report(iqk/dpk/rt/...),
 *       add struct halrf_outsrc_rpt_vxxx
 *       and set HALRF_OUTSRC_FRAMEWORK_VER/HALRF_XXX_RPT to corresponding version
 */
#define HALRF_OUTSRC_FRAMEWORK_VER 2
typedef struct halrf_outsrc_rpt_v2 HALRF_OUTSRC_RPT;
#define HALRF_OUTSRC_IQK_RPT_VER 1
typedef struct halrf_ex_iqk_rpt_v1 HALRF_IQK_RPT;
#define HALRF_OUTSRC_DPK_RPT_VER 1
typedef struct halrf_ex_dpk_rpt_v1 HALRF_DPK_RPT;
#define HALRF_OUTSRC_RT_RPT_VER 2
typedef struct halrf_ex_rt_rpt_v2 HALRF_RT_RPT;
#define HALRF_OUTSRC_TXGAPK_RPT_VER 1
typedef struct halrf_ex_txgapk_rpt_v1 HALRF_TXGAPK_RPT;

/* OUTSRC Report */
struct halrf_outsrc_rpt_v2 { /*** changelog: update rt_rpt to v2 ***/
	struct halrf_err_code_v1 err; /* err code version follow framework */
	u8 iqk_rpt_ver;
	struct halrf_ex_iqk_rpt_v1 iqk_rpt;
	u8 dpk_rpt_ver;
	struct halrf_ex_dpk_rpt_v1 dpk_rpt;
	u8 rt_rpt_ver;
	struct halrf_ex_rt_rpt_v2 rt_rpt;
	u8 txgapk_rpt_ver;
	struct halrf_ex_txgapk_rpt_v1 txgapk_rpt;
};

struct halrf_outsrc_rpt_v1 {
	struct halrf_err_code_v1 err; /* err code version follow framework */
	u8 iqk_rpt_ver;
	struct halrf_ex_iqk_rpt_v1 iqk_rpt;
	u8 dpk_rpt_ver;
	struct halrf_ex_dpk_rpt_v1 dpk_rpt;
	u8 rt_rpt_ver;
	struct halrf_ex_rt_rpt_v1 rt_rpt;
	u8 txgapk_rpt_ver;
	struct halrf_ex_txgapk_rpt_v1 txgapk_rpt;
};

struct halrf_rt_rpt {
	u32 ch_info[10][2][3]; //last10 /cv
	u32 tssi_code[10][2]; //tssi/path
	u32 drv_lck_fail_count;
	u32 fw_lck_fail_count;
};

struct halrf_rfk_dz_rpt {
	u32 iqk_dz_code;
	u32 dpk_dz_code;
	u32 dpk_rxsram[4][512];
	u32 dpk_pas[4][32];
	u32 dack_dz_code;
	u32 rxdck_dz_code;
	u32 txgapk_dz_code;
	u32 tssi_dz_code;
//IQK
	u32 iqk_dz_lok[2][4]; //path/value
	u32 iqk_dz_tx_xym[2][4];//path/value
	u32 iqk_dz_rx_xym[2][4];//path/value
	u32 iqk_dz_rx_rxbb[2][4];//path/value
	u32 iqk_dz_s0_rxsram[4][1280]; //group/value
	u32 iqk_dz_s1_rxsram[4][1280]; //group/value

//TXGAPK
	u32 txgapk_dz_max_nctl_cnt; 
	s32 txgapk_dz_track_d[2][17];
	s32 txgapk_dz_power_d[2][17];
};


struct halrf_ex_dz_info {
	struct halrf_rt_rpt rf_rt_rpt;
	struct halrf_rfk_dz_rpt rfk_dz_rpt;
};


struct halrf_fem_info halrf_ex_efem_info(struct rf_info *rf);

struct halrf_ex_dz_info halrf_ex_rfdz_info(struct rf_info *rf);

enum rtw_hal_status halrf_query_rfdz_err_code(void *rf_void, u32 *err_code);

enum rtw_hal_status halrf_query_rf_diag_buf_len(u32 *len);
enum rtw_hal_status halrf_query_rf_diag_buf(void *rf_void, u8 *ver, u8 *buf, u32 buf_len);

enum rtw_hal_status halrf_query_rfdz_info(void *rf_void, u8 *rt_rpt_buf, u32 rt_buf_len, u8 *rfk_rpt_buf, u32 rfk_buf_len);
#endif

