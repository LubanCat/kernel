/** @file */
/******************************************************************************
 *
 * Copyright(c) 2019 Realtek Corporation. All rights reserved.
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
 ******************************************************************************/

#ifndef _MAC_AX_WOW_DIAG_H_
#define _MAC_AX_WOW_DIAG_H_

#include "../type.h"
#include "fwcmd.h"

#define STA_DIAG_POLL_MS 1
#define STA_DIAG_POLL_CNT 20
#define WOW_EVT_POLL_CNT 200
#define WOW_DIAG_LATEST_BCN 10
#define	GTK_TX_DIAG_MAX 16
#define WOW_DIAG_SER_L1_MAX 8

enum STA_DIAG_SUPPRT {
	STA_DIAG_SUPPRT_NORMAL = 0,
	STA_DIAG_SUPPRT_WOW = 1
};

enum _STA_DIAG_BIT_ {
	B_STA_DIAG_RPT = BIT(FWCMD_C2H_FUNC_STA_DIAG_RPT),
	B_STA_DIAG_SCAN = BIT(FWCMD_C2H_FUNC_STA_DIAG_SCAN),
	B_STA_DIAG_LAST = 0xFFFFFFFF
};

enum _WOW_DIAG_BIT_ {
	B_WOW_DIAG_GTK = BIT(FWCMD_C2H_FUNC_WOW_DIAG_GTK),
	B_WOW_DIAG_AP_LOST = BIT(FWCMD_C2H_FUNC_WOW_DIAG_AP_LOST),
	B_WOW_DIAG_SER = BIT(FWCMD_C2H_FUNC_WOW_DIAG_SER),
	B_WOW_DIAG_RX_EVT = BIT(FWCMD_C2H_FUNC_WOW_DIAG_RX_EVT),
	B_WOW_DIAG_KEEPALIVE = BIT(FWCMD_C2H_FUNC_WOW_DIAG_KEEPALIVE),
	B_WOW_DIAG_ARP = BIT(FWCMD_C2H_FUNC_WOW_DIAG_ARP),
	B_WOW_DIAG_NS = BIT(FWCMD_C2H_FUNC_WOW_DIAG_NS),
	B_WOW_DIAG_PMF = BIT(FWCMD_C2H_FUNC_WOW_DIAG_PMF),
	B_WOW_DIAG_PER_WAKE = BIT(FWCMD_C2H_FUNC_WOW_DIAG_PER_WAKE),
	B_WOW_DIAG_NLO = BIT(FWCMD_C2H_FUNC_WOW_DIAG_NLO),
	B_WOW_DIAG_STA_CSA = BIT(FWCMD_C2H_FUNC_WOW_DIAG_STA_CSA),
	B_WOW_DIAG_NOT_SUPPORT = BIT(31),
	B_WOW_DIAG_LAST = 0xFFFFFFFF
};

enum _WOW_EVT_BIT_ {
	B_WOW_DUMP_RX = BIT(FWCMD_C2H_FUNC_WOW_DUMP_RX),
	B_WOW_DUMP_MEM = BIT(FWCMD_C2H_FUNC_WOW_DUMP_MEM),
	B_WOW_DUMP_ROLE = BIT(FWCMD_C2H_FUNC_WOW_DUMP_ROLE),
	B_WOW_DUMP_BB_RF_REG = BIT(FWCMD_C2H_FUNC_WOW_DUMP_BB_RF_REG),
	B_WOW_DIAG_EVT_LAST = 0xFFFFFFFF
};

enum _STA_DIAG_CATEGORY_ {
	STA_DIAG_CAT_COMMON = 0,
	STA_DIAG_CAT_WOW = 1,
	STA_DIAG_CAT_WOW_EVT = 2,
	STA_DIAG_CAT_WOW_EVT_DACK = 3
};

/**
 * @struct mac_ax_wow_diag_gtk
 * @brief mac_ax_wow_diag_gtk
 *
 * @var mac_ax_wow_diag_gtk::aoac_report
 * Please Place Description here.
 */
struct mac_ax_wow_diag_gtk {
	/* dword0 */
	u32 m1_rcv:16;
	u32 m2_enq:16;
	/* dword1 */
	u32 m2_suc:16;
	u32 mac_drop:8;
	u32 life_drop:8;
	/* dword2 */
	u32 retry_drop:8;
	u32 other_err:8;
	u32 m1_rcv_last:8;
	u32 m2_enq_last:8;
	/* dword3 */
	u32 mac_drop_last:8;
	u32 life_drop_last:8;
	u32 retry_drop_last:8;
	u32 other_err_last:8;
};

struct mac_ax_wow_diag_gtk_tx {
	u32 enq_tsf_h;
	u32 enq_tsf_l;
	u32 cb_tsf_h;
	u32 cb_tsf_l;
	u32 ret_type:8;
	u32 result:8;
	u32 ser_l0:1;
	u32 ser_l1:1;
	u32 rsvd:14;
};

struct mac_ax_wow_diag_gtk_info {
	struct mac_ax_wow_diag_gtk diag_gtk;
	struct mac_ax_wow_diag_gtk_tx gtk_tx[GTK_TX_DIAG_MAX];
	u32 tx_idx;
	u32 bcn_early_cnt;
	u32 bcn_ok_cnt;
	u32 bcn_ok_latest_mask;
};

struct mac_ax_wow_diag_aplost {
	 /* dword0 */
	 u32 bcn_cnt:8;
	 u32 tx_success_cnt:8;
	 u32 tx_fail_cnt:8;
	 u32 tx_fail_rsn:8;
	 /* dword1 */
	 u32 disconnect_cnt:8;
	 u32 disconnect_limit:8;
	 u32 retry_cnt:8;
	 u32 retry_limit:8;
};

struct mac_ax_wow_diag_aplost_info {
	struct mac_ax_wow_diag_aplost diag_aplost;
};

struct mac_ax_wow_diag_ser {
	u32 l0_cnt:16;
	u32 l1_cnt:16;
};

struct max_ax_wow_diag_tsf_info {
	u32 tsf_h;
	u32 tsf_l;
};

struct mac_ax_wow_diag_ser_info {
	struct mac_ax_wow_diag_ser dig_ser;
	struct max_ax_wow_diag_tsf_info l1_tsf[WOW_DIAG_SER_L1_MAX];
};

u32 c2h_wow_evt_done_ack_hdl(struct mac_ax_adapter *adapter,
			     struct rtw_c2h_info *info);

u32 c2h_sta_diag(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		 struct rtw_c2h_info *info);

u32 c2h_wow_diag(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		 struct rtw_c2h_info *info);

u32 c2h_wow_tri_evt(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		    struct rtw_c2h_info *info);

/**
 * @brief mac_req_wow_diag_rpt
 *
 * @param *adapter
 * @param err_rpt
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_req_sta_diag_rpt(struct mac_ax_adapter *adapter,
			 struct mac_ax_sta_diag_err_rpt *rpt_out);
/**
 * @}
 */

/**
 * @brief mac_req_wow_diag_rpt
 *
 * @param *adapter
 * @param err_rpt
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_req_wow_diag_rpt(struct mac_ax_adapter *adapter,
			 struct mac_ax_sta_diag_err_rpt *err_rpt);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @brief mac_req_wow_tri_evt
 *
 * @param *adapter
 * @return Please Place Description here.
 * @retval u32
 */

u32 mac_req_wow_tri_evt(struct mac_ax_adapter *adapter);
/**
 * @}
 */

/**
 * @brief set_sta_diag_stat
 *
 * @param *adapter
 * @param feature
 * @param cat
 * @return Please Place Description here.
 * @retval u32
 */

void set_sta_diag_stat(struct mac_ax_adapter *adapter, u32 feature, u8 cat);
/**
 * @}
 */
#endif // #define _MAC_AX_WOW_DIAG_H_
