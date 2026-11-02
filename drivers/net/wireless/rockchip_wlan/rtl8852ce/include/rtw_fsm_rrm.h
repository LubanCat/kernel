/******************************************************************************
 *
 * Copyright(c) 2019 - 2023 Realtek Corporation.
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
#ifndef __RTW_FSM_RRM_H__
#define __RTW_FSM_RRM_H__

/* for external reference */
/*IEEE Std 80211k Figure 7-95b Neighbor Report element format*/
struct nb_rpt_hdr {
	u8 id; /*0x34: Neighbor Report Element ID*/
	u8 len;
	u8 bssid[ETH_ALEN];
	u32 bss_info;
	u8 reg_class;
	u8 ch_num;
	u8 phy_type;
};

struct rrm_nb_list {
	struct nb_rpt_hdr ent;

	/* -1 : exclude, 0:wo/preference,
	 * 1-255 : preference, 255 indicates the most preferred BSS
	 */
	int preference;
};

struct rrm_nb_rpt {
	struct rrm_nb_list nb_list[RTW_MAX_NB_RPT_NUM];
	struct rtw_ieee80211_channel ch_list[RTW_CHANNEL_SCAN_AMOUNT];
	u8 nb_list_num;
	u8 ch_list_num;

	systime last_update;
};

#ifdef CONFIG_RTW_FSM_RRM
u8 rm_post_event_hdl(_adapter *padapter, u8 *pbuf);

struct fsm_priv;
struct rrm_obj;

/* Header file for application to invoke rrm service */

struct rrm_priv {
	u8 enable;
	_queue rm_queue; /* active rrm obj */

	u8 rm_en_cap_def[5];
	u8 rm_en_cap_assoc[5];
	u8 meas_token;

	/* NB report */
	struct rrm_nb_rpt nb_rpt;

	struct fsm_main *fsm;
	/* rm debug */
	void *prm_sel;
};

int rtw_rrm_reg_fsm(struct fsm_priv *fsmpriv);

u8 rm_add_nb_req(_adapter *a, struct sta_info *psta);
void rtw_ap_parse_sta_rm_en_cap(_adapter *a,
	struct sta_info *psta, struct rtw_ieee802_11_elems *elem);
unsigned int rrm_on_action(_adapter *a, union recv_frame *precv_frame);

void rrm_update_cap(u8 *frame_head, _adapter *pa, u32 pktlen, int offset);
void RM_IE_handler(_adapter *padapter, PNDIS_802_11_VARIABLE_IEs pIE);
int rtw_rrm_get_nb_rpt(_adapter *a, struct rrm_nb_rpt * nb_rpt);
u32 rrm_parse_nb_list(struct rrm_nb_rpt *pnb_rpt, u8 *ie, u32 ie_len);
void rrm_sort_nb_list(struct rrm_nb_rpt *pnb_rpt);

#endif /* CONFIG_RTW_FSM_RRM */
#endif /* __RTW_FSM_RRM_H__ */

