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

struct halrf_rt_rpt {
	u32 ch_info[10][2][2]; //last10 /cv
	u8 tssi_code[10][2]; //tssi/path
	u32 drv_lck_fail_count;
	u32 fw_lck_fail_count;
};

struct halrf_rfk_dz_rpt {
	u32 iqk_dz_code;
	u32 dpk_dz_code;
	u32 dpk_rxsram[4][512];
	u32 dpk_pas[4][32];
	u32 dack_s0_dz_code;
	u32 dack_s1_dz_code;	
	u32 rxdck_dz_code;
	u32 txgapk_dz_code;
	u32 tssi_dz_code;
//IQK
	u32 iqk_dz_lok[2][4]; //path/value
	u32 iqk_dz_tx_xym[2][4];//path/value
	u32 iqk_dz_rx_xym[2][4];//path/value
	u32 iqk_dz_rx_rxbb[2][4];//path/value
	u32 iqk_dz_rx_sram[2][4];//path/value
};


struct halrf_ex_dz_info {
	struct halrf_rt_rpt rf_rt_rpt;
	struct halrf_rfk_dz_rpt rfk_dz_rpt;
};


struct halrf_fem_info halrf_ex_efem_info(struct rf_info *rf);

struct halrf_ex_dz_info halrf_ex_rfdz_info(struct rf_info *rf);

enum rtw_hal_status halrf_query_rfdz_info(void *rf_void, u8 *rt_rpt_buf, u32 rt_buf_len, u8 *rfk_rpt_buf, u32 rfk_buf_len);
#endif

