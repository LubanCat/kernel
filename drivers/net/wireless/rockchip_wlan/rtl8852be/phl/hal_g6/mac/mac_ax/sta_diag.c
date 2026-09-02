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

#include "sta_diag.h"
#include "mac_priv.h"

#if MAC_SELF_DIAG_INFO
static struct c2h_proc_func c2h_proc_sta_diag[] = {
	{c2h_sta_diag_rpt_hdl, FWCMD_C2H_FUNC_STA_DIAG_RPT},
	{c2h_sta_diag_scan_hdl, FWCMD_C2H_FUNC_STA_DIAG_SCAN},
	{c2h_sta_diag_ps_hdl, FWCMD_C2H_FUNC_STA_DIAG_PS},
	{c2h_sta_diag_csa_hdl, FWCMD_C2H_FUNC_STA_DIAG_CSA},
#if MAC_FEAT_MCC
	{c2h_sta_diag_mcc_hdl, FWCMD_C2H_FUNC_STA_DIAG_MCC},
#endif /* MAC_FEAT_MCC */
	{c2h_sta_diag_efuse_hdl, FWCMD_C2H_FUNC_STA_DIAG_EFUSE},
	{NULL, FWCMD_C2H_FUNC_NULL},
};

static struct c2h_proc_func c2h_proc_wow_diag[] = {
	{c2h_wow_diag_gtk_hdl, FWCMD_C2H_FUNC_WOW_DIAG_GTK},
	{c2h_wow_diag_ap_lost_hdl, FWCMD_C2H_FUNC_WOW_DIAG_AP_LOST},
	{c2h_wow_diag_ser_hdl, FWCMD_C2H_FUNC_WOW_DIAG_SER},
	{NULL, FWCMD_C2H_FUNC_WOW_DIAG_RX_EVT},
	{NULL, FWCMD_C2H_FUNC_WOW_DIAG_KEEPALIVE},
	{NULL, FWCMD_C2H_FUNC_WOW_DIAG_ARP},
	{NULL, FWCMD_C2H_FUNC_WOW_DIAG_NS},
	{NULL, FWCMD_C2H_FUNC_WOW_DIAG_PMF},
	{NULL, FWCMD_C2H_FUNC_WOW_DIAG_PER_WAKE},
	{NULL, FWCMD_C2H_FUNC_WOW_DIAG_NLO},
	{NULL, FWCMD_C2H_FUNC_WOW_DIAG_STA_CSA},
	{NULL, FWCMD_C2H_FUNC_NULL},
};

static struct c2h_proc_func c2h_proc_wow_tri_evt[] = {
	{c2h_wow_evt_dump_rx_hdl, FWCMD_C2H_FUNC_WOW_DUMP_RX},
	{c2h_wow_evt_dump_role_hdl, FWCMD_C2H_FUNC_WOW_DUMP_ROLE},
	{c2h_wow_evt_dump_bb_rf_reg_hdl, FWCMD_C2H_FUNC_WOW_DUMP_BB_RF_REG},
	{NULL, FWCMD_C2H_FUNC_NULL},
};

static u32(*send_evt[])(struct mac_ax_adapter *adapter) = {
	send_h2c_evt_req_rx_pkt,
	send_h2c_evt_req_mem,
	send_h2c_evt_req_role_info,
	send_h2c_evt_req_bb_rf_reg,
	NULL
};

u32 c2h_sta_diag_rpt_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			 u32 len, struct rtw_c2h_info *info)
{
	struct mac_ax_sta_diag_rpt *diag_rpt =
		&adapter->sta_diag_info.diag_rpt;
	u8 *c2h_content = buf + FWCMD_HDR_LEN;

	PLTFM_MSG_ALWAYS("[STADIAG]------ Start ------\n");

	PLTFM_MEMCPY(diag_rpt, c2h_content,
		     sizeof(struct mac_ax_sta_diag_rpt));
	PLTFM_MSG_ALWAYS("diagnositic num:%d, err_code:%x\n",
			 diag_rpt->chk_num_normal, diag_rpt->err_code);
	PLTFM_MSG_ALWAYS("bcn_early:%d, bcn_ok:%d, bcn_rcv_mask:%x\n",
			 diag_rpt->bcn_early_cnt, diag_rpt->bcn_ok_cnt,
			 diag_rpt->bcn_rcv_mask_last);

	if (!diag_rpt->err_code_wow &&
	    diag_rpt->err_code_wow != B_WOW_DIAG_NOT_SUPPORT) {
		PLTFM_MSG_ALWAYS("wow diagnositic num:%d, err_code:%x\n",
				 diag_rpt->chk_num_wow, diag_rpt->err_code_wow);

		PLTFM_MSG_ALWAYS("wow_start_tsf_h=%x, wow_start_tsf_l=%x\n",
				 diag_rpt->wow_enter_tsf_h, diag_rpt->wow_enter_tsf_l);
		PLTFM_MSG_ALWAYS("wow fw toggle wake cnt:%d\n",
				 diag_rpt->wow_toggle_wake_cnt);
	}
	if (diag_rpt->err_code_wow == B_WOW_DIAG_NOT_SUPPORT)
		PLTFM_MSG_ALWAYS("WOW_DIAG_NOT_SUPPORT\n");

	set_sta_diag_stat(adapter, B_STA_DIAG_RPT, STA_DIAG_CAT_COMMON);

	PLTFM_MSG_ALWAYS("[STADIAG]------ End ------\n");

	return MACSUCCESS;
}

u32 c2h_sta_diag_scan_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			  u32 len, struct rtw_c2h_info *info)
{
	struct fwcmd_sta_diag_scan *diag_scan;

	diag_scan = (struct fwcmd_sta_diag_scan *)(buf + FWCMD_HDR_LEN);

	set_sta_diag_stat(adapter, B_STA_DIAG_SCAN, STA_DIAG_CAT_COMMON);

	PLTFM_MSG_ALWAYS("[STADIAG]------ Scan Start ------\n");

	PLTFM_MSG_ALWAYS("Null related\n");
	PLTFM_MSG_ALWAYS("    api fail cnt: %d\n",
			 GET_FIELD(diag_scan->dword0, FWCMD_C2H_STA_DIAG_SCAN_NULL_API_FAIL_CNT));
	PLTFM_MSG_ALWAYS("    rls rpt drop cnt (rty, life, macid): (%d, %d, %d)\n",
			 GET_FIELD(diag_scan->dword0,
				   FWCMD_C2H_STA_DIAG_SCAN_NULL_CB_FAIL_RTY_DROP_CNT),
			 GET_FIELD(diag_scan->dword0,
				   FWCMD_C2H_STA_DIAG_SCAN_NULL_CB_FAIL_LIFE_DROP_CNT),
			 GET_FIELD(diag_scan->dword0,
				   FWCMD_C2H_STA_DIAG_SCAN_NULL_CB_FAIL_MACID_DROP_CNT));

	PLTFM_MSG_ALWAYS("Tx related\n");
	PLTFM_MSG_ALWAYS("    read pktofld fail cnt: %d, malformed probe cnt: %d\n",
			 GET_FIELD(diag_scan->dword1,
				   FWCMD_C2H_STA_DIAG_SCAN_READ_PKTOFLD_FAIL_CNT),
			 GET_FIELD(diag_scan->dword2,
				   FWCMD_C2H_STA_DIAG_SCAN_MALFORMED_PROBE_CNT));
	PLTFM_MSG_ALWAYS("    api fail cnt: %d, last ret (type, result): (%d, %d)\n",
			 GET_FIELD(diag_scan->dword2, FWCMD_C2H_STA_DIAG_SCAN_TXAPI_FAIL_CNT),
			 GET_FIELD(diag_scan->dword2,
				   FWCMD_C2H_STA_DIAG_SCAN_TXAPI_LAST_FAIL_RETTYPE),
			 GET_FIELD(diag_scan->dword2,
				   FWCMD_C2H_STA_DIAG_SCAN_TXAPI_LAST_FAIL_RESULT));
	PLTFM_MSG_ALWAYS("    rls rpt drop cnt (rty, life, macid): (%d, %d, %d)\n",
			 GET_FIELD(diag_scan->dword1,
				   FWCMD_C2H_STA_DIAG_SCAN_TX_CB_FAIL_RTY_DROP_CNT),
			 GET_FIELD(diag_scan->dword1,
				   FWCMD_C2H_STA_DIAG_SCAN_TX_CB_FAIL_LIFE_DROP_CNT),
			 GET_FIELD(diag_scan->dword1,
				   FWCMD_C2H_STA_DIAG_SCAN_TX_CB_FAIL_MACID_DROP_CNT));

	PLTFM_MSG_ALWAYS("Ctrl related\n");
	PLTFM_MSG_ALWAYS("    chsw failed ret: %x, mlo_ctrl fail cnt (bb, rf): (%d, %d)\n",
			 diag_scan->dword3,
			 GET_FIELD(diag_scan->dword4,
				   FWCMD_C2H_STA_DIAG_SCAN_BB_CTRL_MLO_FAIL_CNT),
			 GET_FIELD(diag_scan->dword4,
				   FWCMD_C2H_STA_DIAG_SCAN_RF_CTRL_MLO_FAIL_CNT));

	PLTFM_MSG_ALWAYS("[STADIAG]------ Scan End ------\n");

	return MACSUCCESS;
}

u32 c2h_sta_diag_ps_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			u32 len, struct rtw_c2h_info *info)
{
	struct fwcmd_sta_diag_ps *c2h_content;
	struct rtw_mac_ps_diag_info *p_per_role_diag_addr;
	struct rtw_mac_ps_diag_info_c2h *pcontent;
	u8 macid_num;
	u8 idx;

	c2h_content = (struct fwcmd_sta_diag_ps *)(buf + FWCMD_HDR_LEN);

	set_sta_diag_stat(adapter, B_STA_DIAG_PS, STA_DIAG_CAT_COMMON);

	PLTFM_MSG_ALWAYS("[STADIAG][PS]------ PS Start ------\n");

	pcontent = (struct rtw_mac_ps_diag_info_c2h *)c2h_content;

	PLTFM_MSG_ALWAYS("[STADIAG][PS] RPWM_NOFY:%d, RPWM_REQ:%d\n",
			 pcontent->rpwm_ntfy_cnt,
			 pcontent->rpwm_req_cnt);

	PLTFM_MSG_ALWAYS("[STADIAG][PS] RPWM_ACT_CNT:%d, RPWM_CG_CNT:%d, RPWM_PG_CNT:%d\n",
			 pcontent->rpwm_act_cnt,
			 pcontent->rpwm_cg_cnt,
			 pcontent->rpwm_pg_cnt);

	PLTFM_MSG_ALWAYS("[STADIAG][PS] MStatus:0x%x\n",
			 pcontent->mstatus_val);

	PLTFM_MSG_ALWAYS("[STADIAG][PS] CG_CNT:%d, PG_CNT:%d\n",
			 pcontent->enter_cg_cnt,
			 pcontent->enter_pg_cnt);

	PLTFM_MSG_ALWAYS("[STADIAG][PS] B0_NOR:%d, B0_CLSRF:%d, B0_OPENRF:%d\n",
			 pcontent->b0_nor_mode_cnt,
			 pcontent->b0_cls_rf_cnt,
			 pcontent->b0_open_rf_cnt);

	PLTFM_MSG_ALWAYS("[STADIAG][PS] B1_NOR:%d, B1_CLSRF:%d, B1_OPENRF:%d\n",
			 pcontent->b1_nor_mode_cnt,
			 pcontent->b1_cls_rf_cnt,
			 pcontent->b1_open_rf_cnt);

	macid_num = (u8)pcontent->total_macid_num;

	p_per_role_diag_addr = (struct rtw_mac_ps_diag_info *)&c2h_content->dword5;

	PLTFM_MSG_ALWAYS("[STADIAG][PS]------ Role Info Start(%d) ------\n", macid_num);

	for (idx = 0; idx < macid_num; idx++) {
		PLTFM_MSG_ALWAYS("[STADIAG][PS] ----Macid %d----\n",
				 p_per_role_diag_addr->macid);

		PLTFM_MSG_ALWAYS("[STADIAG][PS] Bcn_UC:%d, Bcn_BC:%d\n",
				 p_per_role_diag_addr->bcn_uc_hit_cnt,
				 p_per_role_diag_addr->bcn_bc_hit_cnt);

		PLTFM_MSG_ALWAYS("[STADIAG][PS] Bcn_Rate:%d, Bcn_M:%d, Nor_M:%d, KpOn:%d\n",
				 p_per_role_diag_addr->rx_bcn_rate,
				 p_per_role_diag_addr->rx_bcn_mode_rate,
				 p_per_role_diag_addr->rx_nor_mode_rate,
				 p_per_role_diag_addr->rx_keep_on_rate);

		PLTFM_MSG_ALWAYS("[STADIAG][PS] Bcn_erly:%d, Bcn_To:%d, Ttbb_Sh:%d\n",
				 p_per_role_diag_addr->bcn_erly_time,
				 p_per_role_diag_addr->bcn_to_time,
				 p_per_role_diag_addr->tbtt_shift_time);

		PLTFM_MSG_ALWAYS("[STADIAG][PS] N1_Fail:%d, N0_Fail:%d, TNull_Fail:%d\n",
				 p_per_role_diag_addr->tx_null1_fail_cnt,
				 p_per_role_diag_addr->tx_null0_fail_cnt,
				 p_per_role_diag_addr->null_tx_fail_cnt);

		PLTFM_MSG_ALWAYS("[STADIAG][PS] TxPktIn:%d, SxTxOk:%d\n",
				 p_per_role_diag_addr->txpktin_cnt,
				 p_per_role_diag_addr->sw_tx_ok_cnt);

		p_per_role_diag_addr = p_per_role_diag_addr + 1;
	}

	PLTFM_MSG_ALWAYS("[STADIAG][PS]------ PS End ------\n");

	return MACSUCCESS;
}

u32 c2h_sta_diag_csa_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			 u32 len, struct rtw_c2h_info *info)
{
	struct fwcmd_sta_diag_csa *diag_csa = (struct fwcmd_sta_diag_csa *)(buf + FWCMD_HDR_LEN);

	set_sta_diag_stat(adapter, B_STA_DIAG_SCAN, STA_DIAG_CAT_COMMON);

	PLTFM_MSG_ALWAYS("[STADIAG]------ CSA Start ------\n");

	PLTFM_MSG_ALWAYS("Invalid pri from bcn cnt: %d\n",
			 GET_FIELD(diag_csa->dword0,
				   FWCMD_C2H_STA_DIAG_CSA_PRI_NOT_VALID_CNT));
	if (GET_FIELD(diag_csa->dword0, FWCMD_C2H_STA_DIAG_CSA_PRI_NOT_VALID_CNT)) {
		PLTFM_MSG_ALWAYS("    (last invalid pri: %d)\n",
				 GET_FIELD(diag_csa->dword0,
					   FWCMD_C2H_STA_DIAG_CSA_NOT_VALID_PRI));
	}

	PLTFM_MSG_ALWAYS("Invalid bw from bcn cnt: %d\n",
			 GET_FIELD(diag_csa->dword0,
				   FWCMD_C2H_STA_DIAG_CSA_BW_NOT_VALID_CNT));
	if (GET_FIELD(diag_csa->dword0, FWCMD_C2H_STA_DIAG_CSA_BW_NOT_VALID_CNT)) {
		PLTFM_MSG_ALWAYS("    (last invalid bw: %d)\n",
				 GET_FIELD(diag_csa->dword0,
					   FWCMD_C2H_STA_DIAG_CSA_NOT_VALID_BW));
	}

	PLTFM_MSG_ALWAYS("BCN timeout in new ch cnt: %d\n",
			 GET_FIELD(diag_csa->dword1,
				   FWCMD_C2H_STA_DIAG_CSA_NEWBCN_TIMEOUT_CNT));

	PLTFM_MSG_ALWAYS("Chsw failed cnt: %d\n",
			 GET_FIELD(diag_csa->dword1,
				   FWCMD_C2H_STA_DIAG_CSA_CHSW_FAILED_CNT));
	if (GET_FIELD(diag_csa->dword1, FWCMD_C2H_STA_DIAG_CSA_CHSW_FAILED_CNT))
		PLTFM_MSG_ALWAYS("    (last ret: %x)\n", diag_csa->dword2);

	PLTFM_MSG_ALWAYS("[STADIAG]------ CSA End ------\n");

	return MACSUCCESS;
}

#if MAC_FEAT_MCC

u32 c2h_sta_diag_mcc_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			 u32 len, struct rtw_c2h_info *info)
{
	struct fwcmd_sta_diag_mcc *diag_mcc = (struct fwcmd_sta_diag_mcc *)(buf + FWCMD_HDR_LEN);

	set_sta_diag_stat(adapter, B_STA_DIAG_MCC, STA_DIAG_CAT_COMMON);

	PLTFM_MSG_ALWAYS("[STADIAG]------ MCC Start ------\n");

	PLTFM_MSG_ALWAYS("h2c_add_mcc_fail_cnt: %u\n",
			 GET_FIELD(diag_mcc->dword0,
				   FWCMD_C2H_STA_DIAG_MCC_H2C_ADD_MCC_FAIL_CNT));
	PLTFM_MSG_ALWAYS("h2c_start_mcc_fail_cnt: %u\n",
			 GET_FIELD(diag_mcc->dword0,
				   FWCMD_C2H_STA_DIAG_MCC_H2C_START_MCC_FAIL_CNT));
	PLTFM_MSG_ALWAYS("h2c_stop_mcc_fail_cnt: %u\n",
			 GET_FIELD(diag_mcc->dword0,
				   FWCMD_C2H_STA_DIAG_MCC_H2C_STOP_MCC_FAIL_CNT));
	PLTFM_MSG_ALWAYS("h2c_del_mcc_fail_cnt: %u\n",
			 GET_FIELD(diag_mcc->dword0,
				   FWCMD_C2H_STA_DIAG_MCC_H2C_DEL_MCC_FAIL_CNT));
	PLTFM_MSG_ALWAYS("h2c_mcc_set_dur_fail_cnt: %u\n",
			 GET_FIELD(diag_mcc->dword1,
				   FWCMD_C2H_STA_DIAG_MCC_H2C_MCC_SET_DUR_FAIL_CNT));
	PLTFM_MSG_ALWAYS("change_role_fail_cnt: %u\n",
			 GET_FIELD(diag_mcc->dword1,
				   FWCMD_C2H_STA_DIAG_MCC_CHANGE_ROLE_FAIL_CNT));
	PLTFM_MSG_ALWAYS("sw_ch_fail_cnt: %u\n",
			 GET_FIELD(diag_mcc->dword1,
				   FWCMD_C2H_STA_DIAG_MCC_SW_CH_FAIL_CNT));

	PLTFM_MSG_ALWAYS("[STADIAG]------ MCC End ------\n");

	return MACSUCCESS;
}

#endif /* MAC_FEAT_MCC */

u32 c2h_sta_diag_efuse_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			   u32 len, struct rtw_c2h_info *info)
{
	struct fwcmd_sta_diag_efuse *diag_efuse;

	diag_efuse = (struct fwcmd_sta_diag_efuse *)(buf + FWCMD_HDR_LEN);

	set_sta_diag_stat(adapter, B_STA_DIAG_EFUSE, STA_DIAG_CAT_COMMON);

	PLTFM_MSG_ALWAYS("[STADIAG]------ EFUSE Start ------\n");

	PLTFM_MSG_ALWAYS("efuse error code: %u\n",
			 GET_FIELD(diag_efuse->dword0,
				   FWCMD_C2H_STA_DIAG_EFUSE_H2C_EFUSE_ERR_CODE));

	PLTFM_MSG_ALWAYS("[STADIAG]------ EFUSE End ------\n");

	return MACSUCCESS;
}

u32 c2h_sta_diag(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		 struct rtw_c2h_info *info)
{
	struct c2h_proc_func *proc = c2h_proc_sta_diag;
	u32 (*handler)(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		       struct rtw_c2h_info *info) = NULL;
	u32 hdr0;
	u32 func;

	PLTFM_MSG_TRACE("%s,cat(%d)class(%d)func(%d)len(%d)\n", __func__,
			info->c2h_cat, info->c2h_class, info->c2h_func, len);

	hdr0 = ((struct fwcmd_hdr *)buf)->hdr0;
	hdr0 = le32_to_cpu(hdr0);

	func = GET_FIELD(hdr0, C2H_HDR_FUNC);

	while (proc->id != FWCMD_C2H_FUNC_NULL) {
		if (func == proc->id && proc->handler) {
			handler = proc->handler;
			break;
		}
		proc++;
	}

	if (!handler) {
		PLTFM_MSG_ERR("%s null func handler id: %X", __func__, func);
		return MACNOITEM;
	}

	return handler(adapter, buf, len, info);
}

u32 send_h2c_req_sta_diag_rpt(struct mac_ax_adapter *adapter,
			      u8 wowlan)
{
	u32 ret = MACSUCCESS;
	struct h2c_info h2c_info = { 0 };
	struct fwcmd_req_sta_diag_rpt *content;

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_wow_diag_ctrl);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_STA_DIAG;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_REQ_STA_DIAG_RPT;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	content = (struct fwcmd_req_sta_diag_rpt *)
		   PLTFM_MALLOC(h2c_info.content_len);

	if (!content) {
		PLTFM_MSG_ERR("%s: malloc fail\n", __func__);
		return MACNPTR;
	}

	content->dword0 =
		cpu_to_le32(wowlan ? FWCMD_H2C_REQ_STA_DIAG_RPT_REQ_WOW_RPT : 0);
	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)content);
	if (ret)
		PLTFM_MSG_ERR("[STADIAG]Tx H2C fail (%d)\n", ret);

	PLTFM_FREE(content, h2c_info.content_len);

	return ret;
}

u32 sta_diag_rpt_proc(struct mac_ax_adapter *adapter, u8 supp_type)
{
	struct mac_sta_diag_info *diag_info = &adapter->sta_diag_info;
	struct mac_ax_sta_diag_rpt *diag_rpt = &diag_info->diag_rpt;
	struct rtw_phl_com_t *phl_com = (struct rtw_phl_com_t *)adapter->phl_adapter;
	struct rtw_wcpu_cap_t *wcpu_cap = &phl_com->dev_cap.wcpu_cap;
	u32 *c2h_rcv =  &diag_info->c2h_rcv;
	u32 *err_code = &diag_rpt->err_code;
	u32 ret = MACSUCCESS;
	u8 cnt;

	if (adapter->sm.fwdl != MAC_AX_FWDL_INIT_RDY)
		return MACNOFW;

	if (wcpu_cap->valid) {
		if (!wcpu_cap->mac_ofld_cap.sta_fw_diag)
			return MACFWNOSUPPORT;
	}

	ret = send_h2c_req_sta_diag_rpt(adapter, supp_type);
	if (ret) {
		PLTFM_MSG_ERR("[STADIAG]Request H2C Fail\n");
		return ret;
	}
	*c2h_rcv = 0;

	if (supp_type == STA_DIAG_SUPPRT_WOW)
		diag_info->wow_diag_info.diag_c2h_rcv = 0;

	// Wait for diag report
	cnt = STA_DIAG_POLL_CNT;
	while (!(*c2h_rcv & B_STA_DIAG_RPT)) {
		PLTFM_DELAY_MS(STA_DIAG_POLL_MS);
		if (cnt == 0) {
			PLTFM_MSG_ERR("[STADIG]Poll report c2h fail(0x%x)\n", *c2h_rcv);
			return MACPOLLTO;
		}
		cnt--;
	}
	*c2h_rcv &= ~B_STA_DIAG_RPT;

	// Wait for feature report
	cnt = STA_DIAG_POLL_CNT;
	while (*err_code != *c2h_rcv) {
		PLTFM_DELAY_MS(STA_DIAG_POLL_MS);
		if (cnt == 0) {
			PLTFM_MSG_ERR("[STADIG]Poll C2H fail. err_map(%x), c2h_rcv(%x)\n",
				      *err_code, *c2h_rcv);
			ret = MACPOLLTO;
			break;
		}
		cnt--;
	}

	return ret;
}

u32 mac_req_sta_diag_rpt(struct mac_ax_adapter *adapter,
			 struct mac_ax_sta_diag_err_rpt *rpt_out)
{
	struct mac_ax_sta_diag_rpt *diag_rpt =
		&adapter->sta_diag_info.diag_rpt;
	u32 ret = MACSUCCESS;

	PLTFM_MEMSET(diag_rpt, 0, sizeof(struct mac_ax_sta_diag_rpt));
	ret = sta_diag_rpt_proc(adapter, STA_DIAG_SUPPRT_NORMAL);
	if (ret)
		return ret;

	if (rpt_out) {
		rpt_out->err = diag_rpt->err_code;
		rpt_out->wow_err =  0;
	} else {
		PLTFM_MSG_ERR("[STADIAG]Null report pointer\n");
		ret = MACNPTR;
	}

	return ret;
}

u32 c2h_wow_evt_done_ack_hdl(struct mac_ax_adapter *adapter,
			     struct rtw_c2h_info *info)
{
	switch (info->c2h_func) {
	case FWCMD_H2C_FUNC_WOW_REQ_RX_PKT:
		PLTFM_MSG_ALWAYS("[WOWEVT]Received WOW_REQ_RX_PKT done ack\n");
		if (info->h2c_return == MACSUCCESS) {
			set_sta_diag_stat(adapter, BIT(FWCMD_H2C_FUNC_WOW_REQ_RX_PKT),
					  STA_DIAG_CAT_WOW_EVT_DACK);
		} else {
			PLTFM_MSG_ERR("[WOWEVT]WOW_REQ_RX_PKT failed\n");
		}
		break;
	case FWCMD_H2C_FUNC_WOW_REQ_MEM:
		PLTFM_MSG_ALWAYS("[WOWEVT]Received WOW_REQ_MEM done ack\n");
		if (info->h2c_return == MACSUCCESS) {
			set_sta_diag_stat(adapter, BIT(FWCMD_H2C_FUNC_WOW_REQ_MEM),
					  STA_DIAG_CAT_WOW_EVT_DACK);
		} else {
			PLTFM_MSG_ERR("[WOWEVT]WOW_REQ_MEM failed\n");
		}
		break;
	case FWCMD_H2C_FUNC_WOW_REQ_ROLE_INFO:
		PLTFM_MSG_ALWAYS("[WOWEVT]Received REQ_ROLE_INFO done ack\n");
		if (info->h2c_return == MACSUCCESS) {
			set_sta_diag_stat(adapter, BIT(FWCMD_H2C_FUNC_WOW_REQ_ROLE_INFO),
					  STA_DIAG_CAT_WOW_EVT_DACK);
		} else {
			PLTFM_MSG_ERR("[WOWEVT]WOW_REQ_ROLE_INFO failed\n");
		}
		break;
	case FWCMD_H2C_FUNC_WOW_REQ_BB_RF_REG:
		PLTFM_MSG_ALWAYS("[WOWEVT]Received REQ_BB_RF_REG done ack\n");
		if (info->h2c_return == MACSUCCESS) {
			set_sta_diag_stat(adapter, BIT(FWCMD_H2C_FUNC_WOW_REQ_BB_RF_REG),
					  STA_DIAG_CAT_WOW_EVT_DACK);
		} else {
			PLTFM_MSG_ERR("[WOWEVT]WOW_REQ_BB_RF_REG failed\n");
		}
		break;
	default:
		PLTFM_MSG_ERR("[WOWEVT]Unknown wow done ack\n");
		break;
	}

	return MACSUCCESS;
}

u32 c2h_wow_diag_gtk_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			 u32 len, struct rtw_c2h_info *info)
{
	struct mac_ax_wow_diag_gtk_info diag_gtk_info;
	struct mac_ax_wow_diag_gtk *diag_gtk = &diag_gtk_info.diag_gtk;
	struct mac_ax_wow_diag_gtk_tx *gtk_tx = &diag_gtk_info.gtk_tx[0];
	u8 *c2h_content = buf + FWCMD_HDR_LEN;
	u8 idx;

	set_sta_diag_stat(adapter, B_WOW_DIAG_GTK, STA_DIAG_CAT_WOW);

	PLTFM_MEMCPY(&diag_gtk_info, c2h_content,
		     sizeof(struct mac_ax_wow_diag_gtk_info));

	PLTFM_MSG_ALWAYS("[WOWDAIG]------ GTK Start ------\n");

	PLTFM_MSG_ALWAYS("m1_rcv(%d) m2_enq(%d) m2_success(%d)\n",
			 diag_gtk->m1_rcv, diag_gtk->m2_enq, diag_gtk->m2_suc);
	PLTFM_MSG_ALWAYS("m2_mac_drop(%d) m2_life_drop(%d)\n",
			 diag_gtk->mac_drop, diag_gtk->life_drop);
	PLTFM_MSG_ALWAYS("m2_retry_drop(%d) other_err(%d)\n",
			 diag_gtk->retry_drop, diag_gtk->other_err);
	PLTFM_MSG_ALWAYS("m1_rcv_last(%d) m2_enq_last(%d)\n",
			 diag_gtk->m1_rcv_last, diag_gtk->m2_enq_last);
	PLTFM_MSG_ALWAYS("m2_mac_drop_last(%d) m2_life_drop_last(%d)\n",
			 diag_gtk->mac_drop_last, diag_gtk->life_drop_last);
	PLTFM_MSG_ALWAYS("m2_retry_drop_last(%d) other_err_last(%d)\n",
			 diag_gtk->retry_drop_last, diag_gtk->other_err_last);

	for (idx = 0; idx < GTK_TX_DIAG_MAX; idx++) {
		PLTFM_MSG_ALWAYS("Tx[%d] enq_tsf_h=%x enq_tsf_l=%x\n",
				 idx, gtk_tx[idx].enq_tsf_h, gtk_tx[idx].enq_tsf_l);
		PLTFM_MSG_ALWAYS("Tx[%d] cb_tsf_h=%x cb_tsf_l=%x\n",
				 idx, gtk_tx[idx].cb_tsf_h, gtk_tx[idx].cb_tsf_l);
		PLTFM_MSG_ALWAYS("Tx[%d] ret_type(%x) result(%x)\n",
				 idx, gtk_tx[idx].ret_type, gtk_tx[idx].result);
		PLTFM_MSG_ALWAYS("Tx[%d] ser_l0(%x) ser_l1(%x)\n",
				 idx, gtk_tx[idx].ser_l0, gtk_tx[idx].ser_l1);
	}

	PLTFM_MSG_ALWAYS("m2_tx_idx(%d)\n", diag_gtk_info.tx_idx);

	PLTFM_MSG_ALWAYS("[WOWDIAG]------ GTK End ------\n");

	return MACSUCCESS;
}

u32 c2h_wow_diag_ap_lost_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			     u32 len, struct rtw_c2h_info *info)
{
	struct mac_ax_wow_diag_aplost_info diag_aplost_info;
	struct mac_ax_wow_diag_aplost *diag_aplost =
		&diag_aplost_info.diag_aplost;
	u8 *c2h_content = buf + FWCMD_HDR_LEN;

	set_sta_diag_stat(adapter, B_WOW_DIAG_AP_LOST, STA_DIAG_CAT_WOW);

	PLTFM_MEMCPY(&diag_aplost_info, c2h_content,
		     sizeof(struct mac_ax_wow_diag_aplost_info));

	PLTFM_MSG_ALWAYS("[WOWDIAG]------ AP Lost Start ------\n");

	PLTFM_MSG_ALWAYS("bcn_cnt(%d) tx_success_cnt(%d)\n",
			 diag_aplost->bcn_cnt, diag_aplost->tx_success_cnt);
	PLTFM_MSG_ALWAYS("tx_fail_cnt(%d) tx_fail_rsn(%d)\n",
			 diag_aplost->tx_fail_cnt, diag_aplost->tx_fail_rsn);
	PLTFM_MSG_ALWAYS("disconnect_cnt(%d) disconnect_limit(%d)\n",
			 diag_aplost->disconnect_cnt, diag_aplost->disconnect_limit);
	PLTFM_MSG_ALWAYS("retry_cnt(%d) retry_limit(%d)\n",
			 diag_aplost->retry_cnt, diag_aplost->retry_limit);

	PLTFM_MSG_ALWAYS("[WOWDIAG]------ AP Lost End ------\n");

	return MACSUCCESS;
}

u32 c2h_wow_diag_ser_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			 u32 len, struct rtw_c2h_info *info)
{
	struct mac_ax_wow_diag_ser_info diag_ser_info;
	struct mac_ax_wow_diag_ser *diag_ser =
		&diag_ser_info.dig_ser;
	struct max_ax_wow_diag_tsf_info *l1_tsf =
		&diag_ser_info.l1_tsf[0];
	u8 *c2h_content = buf + FWCMD_HDR_LEN;
	u8 idx;

	set_sta_diag_stat(adapter, B_WOW_DIAG_SER, STA_DIAG_CAT_WOW);

	PLTFM_MEMCPY(&diag_ser_info, c2h_content,
		     sizeof(struct mac_ax_wow_diag_ser_info));

	PLTFM_MSG_ALWAYS("[WOWDIAG]------ SER Start ------\n");

	PLTFM_MSG_ALWAYS("l1_cnt(%d)\n", diag_ser->l1_cnt);

	for (idx = 0; idx < WOW_DIAG_SER_L1_MAX; idx++) {
		PLTFM_MSG_ALWAYS("l1_tsf[%d]=0x%x,0x%x\n",
				 idx, l1_tsf[idx].tsf_h, l1_tsf[idx].tsf_l);
	}

	PLTFM_MSG_ALWAYS("[WOWDIAG]------ SER End ------\n");

	return MACSUCCESS;
}

u32 c2h_wow_diag(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		 struct rtw_c2h_info *info)
{
	struct c2h_proc_func *proc = c2h_proc_wow_diag;
	u32 (*handler)(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		       struct rtw_c2h_info *info) = NULL;
	u32 hdr0;
	u32 func;

	PLTFM_MSG_TRACE("%s,cat(%d)class(%d)func(%d)len(%d)\n", __func__,
			info->c2h_cat, info->c2h_class, info->c2h_func, len);

	hdr0 = ((struct fwcmd_hdr *)buf)->hdr0;
	hdr0 = le32_to_cpu(hdr0);

	func = GET_FIELD(hdr0, C2H_HDR_FUNC);

	while (proc->id != FWCMD_C2H_FUNC_NULL) {
		if (func == proc->id && proc->handler) {
			handler = proc->handler;
			break;
		}
		proc++;
	}

	if (!handler) {
		PLTFM_MSG_ERR("%s null func handler id: %X", __func__, func);
		return MACNOITEM;
	}

	return handler(adapter, buf, len, info);
}

u32 c2h_wow_evt_dump_rx_hdl(struct mac_ax_adapter *adapter,
			    u8 *buf, u32 len, struct rtw_c2h_info *info)
{
	set_sta_diag_stat(adapter, B_WOW_DUMP_RX, STA_DIAG_CAT_WOW_EVT);

	PLTFM_MSG_ALWAYS("[WOWEVT]------ Dump Rx Start ------\n");

	PLTFM_MSG_ALWAYS("[WOWEVT]------ Dump Rx End ------\n");

	return MACSUCCESS;
}

u32 c2h_wow_evt_dump_role_hdl(struct mac_ax_adapter *adapter,
			      u8 *buf, u32 len, struct rtw_c2h_info *info)
{
	set_sta_diag_stat(adapter, B_WOW_DUMP_ROLE, STA_DIAG_CAT_WOW_EVT);

	PLTFM_MSG_ALWAYS("[WOWEVT]------ Dump Role Start ------\n");

	PLTFM_MSG_ALWAYS("[WOWEVT]------ Dump Role End ------\n");

	return MACSUCCESS;
}

u32 c2h_wow_evt_dump_bb_rf_reg_hdl(struct mac_ax_adapter *adapter,
				   u8 *buf, u32 len, struct rtw_c2h_info *info)
{
	set_sta_diag_stat(adapter, B_WOW_DUMP_BB_RF_REG, STA_DIAG_CAT_WOW_EVT);

	PLTFM_MSG_ALWAYS("[WOWEVT]------ Dump BB RF REG Start ------\n");

	PLTFM_MSG_ALWAYS("[WOWEVT]------ Dump BB RF REG End ------\n");

	return MACSUCCESS;
}

u32 c2h_wow_tri_evt(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		    struct rtw_c2h_info *info)
{
	struct c2h_proc_func *proc = c2h_proc_wow_tri_evt;
	u32 (*handler)(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		       struct rtw_c2h_info *info) = NULL;
	u32 hdr0;
	u32 func;

	hdr0 = ((struct fwcmd_hdr *)buf)->hdr0;
	hdr0 = le32_to_cpu(hdr0);

	func = GET_FIELD(hdr0, C2H_HDR_FUNC);

	while (proc->id != FWCMD_C2H_FUNC_NULL) {
		if (func == proc->id && proc->handler) {
			handler = proc->handler;
			break;
		}
		proc++;
	}

	if (!handler) {
		PLTFM_MSG_ERR("%s, null func handler id: %X", __func__, func);
		return MACNOITEM;
	}

	return handler(adapter, buf, len, info);
}

u32 mac_req_wow_diag_rpt(struct mac_ax_adapter *adapter,
			 struct mac_ax_sta_diag_err_rpt *rpt_out)
{
	struct mac_sta_diag_info *diag_info = &adapter->sta_diag_info;
	struct mac_ax_sta_diag_rpt *diag_rpt = &diag_info->diag_rpt;
	struct mac_wow_diag_info *wow_diag_info = &diag_info->wow_diag_info;
	u32 *diag_rcv = &wow_diag_info->diag_c2h_rcv;
	u32 *evt_c2h_rcv = &wow_diag_info->evt_c2h_rcv;
	u32 ret = MACSUCCESS;
	u8 cnt;
	u32 evt_en;

	if (adapter->sm.fwdl != MAC_AX_FWDL_INIT_RDY)
		return MACNOFW;

	evt_en = wow_diag_info->evt_en;
	PLTFM_MEMSET(diag_info, 0, sizeof(struct mac_sta_diag_info));
	ret = sta_diag_rpt_proc(adapter, STA_DIAG_SUPPRT_WOW);
	if (ret)
		return ret;

	if (diag_rpt->err_code_wow == B_WOW_DIAG_NOT_SUPPORT) {
		PLTFM_MSG_ALWAYS("[WOWDIAG]FW not support\n");
		return MACFWNOSUPPORT;
	}

	// Wait for wowlan report
	cnt = STA_DIAG_POLL_CNT;
	PLTFM_MSG_TRACE("[WOWDIAG]Start to poll ERR(0x%x)\n",
			diag_rpt->err_code_wow);
	while (diag_rpt->err_code_wow != *diag_rcv) {
		PLTFM_DELAY_MS(STA_DIAG_POLL_MS);
		if (cnt == 0) {
			PLTFM_MSG_ERR("[WOWDIG]Poll C2H fail. err_map(%x), c2h_rcv(%x)\n",
				      diag_rpt->err_code_wow, *diag_rcv);
			ret = MACPOLLTO;
			break;
		}
		cnt--;
	}

	// Wait for wowlan triggered event report
	if (evt_en) {
		PLTFM_MSG_TRACE("[WOWEVT]Start to poll EVT(0x%x)\n", evt_en);
		cnt = WOW_EVT_POLL_CNT;
		while (evt_en != *evt_c2h_rcv) {
			PLTFM_DELAY_MS(STA_DIAG_POLL_MS);
			if (cnt == 0) {
				PLTFM_MSG_ERR("[WOWEVT]Poll C2H fail. event_map(%x), c2h_rcv(%x)\n",
					      evt_en, *evt_c2h_rcv);
				ret = MACPOLLTO;
				break;
			}
			cnt--;
		}
	}

	if (rpt_out) {
		rpt_out->err = diag_rpt->err_code;
		rpt_out->wow_err = diag_rpt->err_code_wow;
	} else {
		PLTFM_MSG_ERR("[WOWDIAG]Null report pointer\n");
		ret = MACNPTR;
	}

	return ret;
}

u32 send_h2c_evt_req_rx_pkt(struct mac_ax_adapter *adapter)
{
	u32 ret = MACSUCCESS;
	struct h2c_info h2c_info = { 0 };
	struct fwcmd_wow_req_rx_pkt *content;
	struct wow_tri_evt_parm *evt_parm =
		&adapter->sta_diag_info.wow_diag_info.evt_parm;
	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_wow_req_rx_pkt);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_WOW_TRI_EVT;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_WOW_REQ_RX_PKT;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	content =
		(struct fwcmd_wow_req_rx_pkt *)PLTFM_MALLOC(h2c_info.content_len);

	if (!content) {
		PLTFM_MSG_ERR("%s: malloc fail\n", __func__);
		return MACNPTR;
	}
	content->dword0 =
		cpu_to_le32(SET_WORD(evt_parm->pkt_num,
				     FWCMD_H2C_WOW_REQ_RX_PKT_PKT_NUM) |
			    SET_WORD(evt_parm->pld_size,
				     FWCMD_H2C_WOW_REQ_RX_PKT_PLD_SIZE));

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)content);
	if (ret)
		PLTFM_MSG_ERR("[WOWEVT]REQ_RX_PKT H2C fail (%d)\n", ret);

	PLTFM_FREE(content, h2c_info.content_len);

	return ret;
}

u32 send_h2c_evt_req_mem(struct mac_ax_adapter *adapter)
{
	u32 ret = MACSUCCESS;
	struct h2c_info h2c_info = { 0 };
	struct fwcmd_wow_req_mem *content;
	struct wow_tri_evt_parm *evt_parm =
		&adapter->sta_diag_info.wow_diag_info.evt_parm;
	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_wow_req_mem);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_WOW_TRI_EVT;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_WOW_REQ_MEM;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	content = (struct fwcmd_wow_req_mem *)
		PLTFM_MALLOC(h2c_info.content_len);

	if (!content) {
		PLTFM_MSG_ERR("%s: malloc fail\n", __func__);
		return MACNPTR;
	}
	content->dword0 =
	cpu_to_le32((evt_parm->heap_info ? FWCMD_H2C_WOW_REQ_MEM_HEAP_INFO : 0) |
		(evt_parm->mem_info ? FWCMD_H2C_WOW_REQ_MEM_MEM_INFO : 0) |
		(evt_parm->wow_start ? FWCMD_H2C_WOW_REQ_MEM_WOW_START : 0) |
		(evt_parm->wow_end ? FWCMD_H2C_WOW_REQ_MEM_WOW_END : 0));

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)content);
	if (ret)
		PLTFM_MSG_ERR("[WOWEVT]REQ_MEM H2C fail (%d)\n", ret);

	PLTFM_FREE(content, h2c_info.content_len);

	return ret;
}

u32 send_h2c_evt_req_role_info(struct mac_ax_adapter *adapter)
{
	u32 ret = MACSUCCESS;
	struct h2c_info h2c_info = { 0 };

	h2c_info.agg_en = 0;
	h2c_info.content_len = 0;
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_WOW_TRI_EVT;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_WOW_REQ_ROLE_INFO;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	ret = mac_h2c_common(adapter, &h2c_info, NULL);
	if (ret)
		PLTFM_MSG_ERR("[WOWEVT]REQ_ROLE_INFO H2C fail (%d)\n", ret);

	return ret;
}

u32 send_h2c_evt_req_bb_rf_reg(struct mac_ax_adapter *adapter)
{
	u32 ret = MACSUCCESS;
	struct h2c_info h2c_info = { 0 };
	struct fwcmd_wow_req_bb_rf_reg *content;
	struct wow_tri_evt_parm *evt_parm =
		&adapter->sta_diag_info.wow_diag_info.evt_parm;
	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_wow_req_bb_rf_reg);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_WOW_TRI_EVT;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_WOW_REQ_BB_RF_REG;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	content = (struct fwcmd_wow_req_bb_rf_reg *)
		PLTFM_MALLOC(h2c_info.content_len);

	if (!content) {
		PLTFM_MSG_ERR("%s: malloc fail\n", __func__);
		return MACNPTR;
	}
	content->dword0 =
	cpu_to_le32((evt_parm->bb ? FWCMD_H2C_WOW_REQ_BB_RF_REG_BB : 0) |
		(evt_parm->rf ? FWCMD_H2C_WOW_REQ_BB_RF_REG_RF : 0));

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)content);
	if (ret)
		PLTFM_MSG_ERR("[WOWEVT]REQ_BB_RF_REG H2C fail (%d)\n", ret);

	PLTFM_FREE(content, h2c_info.content_len);

	return ret;
}

u32 mac_req_wow_tri_evt(struct mac_ax_adapter *adapter)
{
	struct mac_wow_diag_info *diag_info =
		&adapter->sta_diag_info.wow_diag_info;
	u32 ret = MACSUCCESS;
	u32 evt_en;
	u8 idx, cnt, evt_cnt;

	if (diag_info->evt_en == 0) {
		PLTFM_MSG_ALWAYS("[WOWEVT]No evt enabled\n");
		return ret;
	} else {
		PLTFM_MSG_ALWAYS("[WOWEVT]evt_en=0x%x\n", diag_info->evt_en);
	}

	evt_en = diag_info->evt_en;
	evt_cnt = 0;
	while (evt_en) {
		evt_cnt += (u32)(evt_en & 0x1);
		evt_en >>= 0x1;
	}
	PLTFM_MSG_ALWAYS("[WOWEVT]evt_cnt=%d\n", evt_cnt);

	diag_info->evt_dack = 0;
	diag_info->evt_c2h_rcv = 0;
	if (diag_info->evt_en) {
		for (idx = 0; idx < evt_cnt; idx++) {
			if (BIT(idx) & diag_info->evt_en) {
				if (send_evt[idx]) {
					ret = send_evt[idx](adapter);
					if (ret)
						PLTFM_MSG_ERR("evt fail\n");
				} else {
					break;
				}
			}
		}
	}

	cnt = STA_DIAG_POLL_CNT;
	while (diag_info->evt_dack != diag_info->evt_en) {
		PLTFM_DELAY_MS(STA_DIAG_POLL_MS);
		if (--cnt == 0) {
			PLTFM_MSG_ERR("[WOWEVT]Poll done ack fail. event_map(%x), event_dack(%x)\n",
				      diag_info->evt_en, diag_info->evt_dack);
			return MACPOLLTO;
		}
	}

	return ret;
}

u32 mac_req_sta_diagnosis(struct mac_ax_adapter *adapter,
			  u32 *total_check_num, u32 *err_num, char *output, u32 out_len, u32 *used)
{
	struct mac_ax_sta_diag_rpt *diag_rpt =
		&adapter->sta_diag_info.diag_rpt;
	u32 ret = MACSUCCESS;

	PLTFM_MEMSET(diag_rpt, 0, sizeof(struct mac_ax_sta_diag_rpt));
	ret = sta_diag_rpt_proc(adapter, STA_DIAG_SUPPRT_NORMAL);
	if (ret)
		return ret;

	*total_check_num += diag_rpt->chk_num_normal;

	if (diag_rpt->err_code & B_STA_DIAG_SCAN) {
		MAC_DBG_MSG(out_len, *used, output + *used, out_len - *used,
			    "STA FW diagnosis - Scan fail\n");
		(*err_num)++;
	}
	if (diag_rpt->err_code & B_STA_DIAG_CSA) {
		MAC_DBG_MSG(out_len, *used, output + *used, out_len - *used,
			    "STA FW diagnosis - CSA fail\n");
		(*err_num)++;
	}
	if (diag_rpt->err_code & B_STA_DIAG_PS) {
		MAC_DBG_MSG(out_len, *used, output + *used, out_len - *used,
			    "STA FW diagnosis - PS fail\n");
		(*err_num)++;
	}
#if MAC_FEAT_MCC
	if (diag_rpt->err_code & B_STA_DIAG_MCC) {
		MAC_DBG_MSG(out_len, *used, output + *used, out_len - *used,
			    "STA FW diagnosis - MCC fail\n");
		(*err_num)++;
	}
#endif //MAC_FEAT_MCC
	if (diag_rpt->err_code & B_STA_DIAG_EFUSE) {
		MAC_DBG_MSG(out_len, *used, output + *used, out_len - *used,
			    "STA FW diagnosis - Efuse fail\n");
		(*err_num)++;
	}

	return ret;
}

void set_sta_diag_stat(struct mac_ax_adapter *adapter, u32 feature, u8 cat)
{
	struct mac_sta_diag_info *diag_info = &adapter->sta_diag_info;
	u32 *c2h_rcv = NULL;

	switch (cat) {
	case STA_DIAG_CAT_COMMON:
		c2h_rcv = &diag_info->c2h_rcv;
		break;
	case STA_DIAG_CAT_WOW:
		c2h_rcv = &diag_info->wow_diag_info.diag_c2h_rcv;
		break;
	case STA_DIAG_CAT_WOW_EVT:
		c2h_rcv = &diag_info->wow_diag_info.evt_c2h_rcv;
		break;
	case STA_DIAG_CAT_WOW_EVT_DACK:
		c2h_rcv = &diag_info->wow_diag_info.evt_dack;
		break;
	default:
		PLTFM_MSG_ERR("[STADIAG]Wrong cat=%d\n", cat);
		break;
	}

	if (c2h_rcv)
		*c2h_rcv |= feature;
	else
		PLTFM_MSG_ERR("[STADIAG]Null global variable for c2h status\n");
}
#endif //#if MAC_SELF_DIAG_INFO
