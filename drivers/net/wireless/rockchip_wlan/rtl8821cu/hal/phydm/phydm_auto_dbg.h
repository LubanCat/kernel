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

#ifndef __PHYDM_AUTO_DBG_H__
#define __PHYDM_AUTO_DBG_H__

#define AUTO_DBG_VERSION "1.2" /* @2024.05.016  Sam, Add phydm query api for driver team*/

/* @1 ============================================================
 * 1  Definition
 * 1 ============================================================
 */

#define AUTO_CHK_HANG_STEP_MAX 3
#define DBGPORT_CHK_NUM 6

#ifdef PHYDM_AUTO_DEGBUG

/* @1 ============================================================
 * 1  enumeration
 * 1 ============================================================
 */

enum auto_dbg_type_e {
	AUTO_DBG_STOP		= 0,
	AUTO_DBG_CHECK_HANG	= 1,
	AUTO_DBG_CHECK_RA	= 2,
	AUTO_DBG_CHECK_DIG	= 3
};

/* @1 ============================================================
 * 1  structure
 * 1 ============================================================
 */

struct n_dbgport_803 {
	/*@BYTE 3*/
	u8 bb_rst_b : 1;
	u8 glb_rst_b : 1;
	u8 zero_1bit_1 : 1;
	u8 ofdm_rst_b : 1;
	u8 cck_txpe : 1;
	u8 ofdm_txpe : 1;
	u8 phy_tx_on : 1;
	u8 tdrdy : 1;
	/*@BYTE 2*/
	u8 txd : 8;
	/*@BYTE 1*/
	u8 cck_cca_pp : 1;
	u8 ofdm_cca_pp : 1;
	u8 rx_rst : 1;
	u8 rdrdy : 1;
	u8 rxd_7_4 : 4;
	/*@BYTE 0*/
	u8 rxd_3_0 : 4;
	u8 ofdm_tx_en : 1;
	u8 cck_tx_en : 1;
	u8 zero_1bit_2 : 1;
	u8 clk_80m : 1;
};

struct phydm_auto_dbg_struct {
	enum auto_dbg_type_e auto_dbg_type;
	u8 dbg_step;
	u16 dbg_port_table[DBGPORT_CHK_NUM];
	u32 dbg_port_val[DBGPORT_CHK_NUM];
	u16 ofdm_t_cnt;
	u16 ofdm_r_cnt;
	u16 cck_t_cnt;
	u16 cck_r_cnt;
	u16 ofdm_crc_error_cnt;
	u16 cck_crc_error_cnt;
};

/* @1 ============================================================
 * 1  function prototype
 * 1 ============================================================
 */

enum phydm_auto_dbg_type {
	AUTO_DBG_CHK_HANG	= 0,
	AUTO_DBG_CHECK_TX	= 1,
	AUTO_DBG_STORE_PMAC	= 2,
	AUTO_DBG_PHY_UTILITY	= 3
};


struct phydm_phystatus_avg_info { /*U(8,0)*/
	/*@[CCK]*/
	u8			rssi_cck_avg;
	u8			rssi_beacon_avg[RF_PATH_MEM_SIZE];
	#ifdef PHYSTS_3RD_TYPE_SUPPORT
	#if (defined(PHYDM_COMPILE_ABOVE_2SS))
	u8			rssi_cck_avg_abv_2ss[RF_PATH_MEM_SIZE - 1];
	#endif
	#endif
	/*@[OFDM]*/
	u8			rssi_ofdm_avg[RF_PATH_MEM_SIZE];
	u8			evm_ofdm_avg;
	u8			snr_ofdm_avg[RF_PATH_MEM_SIZE];
	/*@[1SS]*/
	u8			rssi_1ss_avg[RF_PATH_MEM_SIZE];
	u8			evm_1ss_avg;
	u8			snr_1ss_avg[RF_PATH_MEM_SIZE];
	/*@[2SS]*/
	#if (defined(PHYDM_COMPILE_ABOVE_2SS))
	u8			rssi_2ss_avg[RF_PATH_MEM_SIZE];
	u8			evm_2ss_avg[2];
	u8			snr_2ss_avg[RF_PATH_MEM_SIZE];
	#endif
	/*@[3SS]*/
	#if (defined(PHYDM_COMPILE_ABOVE_3SS))
	u8			rssi_3ss_avg[RF_PATH_MEM_SIZE];
	u8			evm_3ss_avg[3];
	u8			snr_3ss_avg[RF_PATH_MEM_SIZE];
	#endif
	/*@[4SS]*/
	#if (defined(PHYDM_COMPILE_ABOVE_4SS))
	u8			rssi_4ss_avg[RF_PATH_MEM_SIZE];
	u8			evm_4ss_avg[4];
	u8			snr_4ss_avg[RF_PATH_MEM_SIZE];
	#endif
	#ifdef PHYDM_AUTO_DEGBUG
	u8			evm_1ss_avg_all;
	u8			evm_min_avg;
	u8			evm_max_avg;
	u8			evm_ss_avg[PHYDM_MAX_SS];
	u8			snr_per_path_avg[RF_PATH_MEM_SIZE];
	#endif
};

struct phydm_bkp_phy_utility_info {
	u16	rx_utility;
	u16	avg_phy_rate;
	u8	rx_rate_plurality;
	u8	tx_rate;
	s32	cfo_avg; /*u(32,0)*/
	#ifdef PHYDM_PHYSTAUS_AUTO_SWITCH
	u32	cn_avg;	/*u(10,3)*/
	#endif
	struct phydm_phystatus_avg_info physts_avg_i; /*U(8,0)*/
};

struct phydm_cca_info {
	u32		cnt_ofdm_cca;
	u32		cnt_cck_cca;
	u32		cnt_cca_all;
};

struct phydm_crc_info {
	u32		cnt_cck_crc32_error;
	u32		cnt_cck_crc32_ok;
	u32		cnt_ofdm_crc32_error;
	u32		cnt_ofdm_crc32_ok;
	u32		cnt_ht_crc32_error;
	u32		cnt_ht_crc32_ok;
	u32		cnt_vht_crc32_error;
	u32		cnt_vht_crc32_ok;
	u32		cnt_crc32_error_all;
	u32		cnt_crc32_ok_all;
	u32		cnt_mpdu_crc32_ok;
	u32		cnt_mpdu_crc32_error;
	u32		cnt_mpdu_miss;
};

struct phydm_crc2_info {
	u8		ofdm2_rate_idx;
	u32		cnt_ofdm2_crc32_error;
	u32		cnt_ofdm2_crc32_ok;
	u8		ofdm2_pcr;
	u8		ht2_rate_idx;
	u32		cnt_ht2_crc32_error;
	u32		cnt_ht2_crc32_ok;
	u8		ht2_pcr;
	u8		vht2_rate_idx;
	u32		cnt_vht2_crc32_error;
	u32		cnt_vht2_crc32_ok;
	u8		vht2_pcr;
};


struct phydm_legacy_fa_info {
	u32		cnt_parity_fail;
	u32		cnt_rate_illegal;
	u32		cnt_fast_fsync;
	u32		cnt_sb_search_fail;
};

struct phydm_ht_fa_info {
	u32		cnt_crc8_fail;
	u32		cnt_mcs_fail;
};

struct phydm_vht_fa_info {
	u32		cnt_crc8_fail_vht;
	u32		cnt_mcs_fail_vht;
};

struct phydm_fa_info {
	u32		cnt_cck_fail;
	u32		cnt_ofdm_fail;
	u32		cnt_fail_all;
	struct phydm_legacy_fa_info	legacy_fa_i;
	struct phydm_ht_fa_info		ht_fa_i;
	struct phydm_vht_fa_info	vht_fa_i;
};

struct phydm_tx_cnt_info {
	u32		cck_mac_txen;
	u32		cck_phy_txon;
	u32		ofdm_mac_txen;
	u32		ofdm_phy_txon;
};


struct phydm_bkp_pmac_info {
	u32				cnt_bt_polluted;
	struct phydm_tx_cnt_info	tx_cnt_i;
	struct phydm_cca_info		cca_i;
	struct phydm_crc_info		crc_i;
	struct phydm_crc2_info		crc2_i;
	struct phydm_fa_info		fa_i;
};

struct phydm_chk_hang_info {
	u16		consecutive_no_tx_cnt;
	u16		consecutive_no_rx_cnt;
	boolean		hang_occur;
};

struct phydm_auto_dbg_info {
	u32 auto_dbg_type_i;
	struct phydm_bkp_pmac_info bkp_pmac_i;
	struct phydm_bkp_phy_utility_info bkp_phy_utility_i;
	struct phydm_chk_hang_info chk_hang_i;
};


void phydm_dbg_port_dump(void *dm_void, u32 *used, char *output, u32 *out_len);

void phydm_auto_dbg_console(
	void *dm_void,
	char input[][16],
	u32 *_used,
	char *output,
	u32 *_out_len);

void phydm_auto_dbg_engine(void *dm_void);

void phydm_auto_dbg_engine_init(void *dm_void);

void phydm_auto_debug_watchdog(void *dm_void);

void phydm_auto_debug_init(void *dm_void);

void phydm_auto_debug_dbg(
	void *dm_void,
	char input[][16],
	u32 *_used,
	char *output,
	u32 *_out_len);

void phydm_query_phy_utility_info(void *dm_void, struct phydm_bkp_phy_utility_info *rpt);

void phydm_query_pmac_info(void *dm_void, struct phydm_bkp_pmac_info *rpt);

void phydm_query_hang_info(void *dm_void, struct phydm_chk_hang_info *rpt);

void phydm_auto_debug_en(void *dm_void, enum phydm_auto_dbg_type type, boolean en);

#endif
#endif
