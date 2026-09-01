/******************************************************************************
 *
 * Copyright(c) 2023 - 2024 Realtek Corporation.
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
#ifndef __RTW_FSM_BTM_H__
#define __RTW_FSM_BTM_H__

#ifdef CONFIG_RTW_FSM_BTM
struct fsm_priv;
struct btm_obj;
struct roam_nb_info;

/* Header file for application to invoke btm service */

struct btm_priv {
	struct fsm_main *fsm;
	struct btm_obj *btm;
};

int rtw_btm_reg_fsm(struct fsm_priv *fsmpriv);
int rtw_btm_new_obj(_adapter *a, struct sta_info *psta,
	struct roam_nb_info *pnb, u8 roam_reason);
void rtw_btm_notify_action_resp(struct btm_obj *pbtm);
void rtw_btm_notify_scan_done(void *obj);
void rtw_btm_notify_scan_found_candidate(struct btm_obj *pbtm, struct wlan_network *pnetwork);
#endif /* CONFIG_RTW_FSM_BTM */
#endif /* __RTW_BTM_FSM_H__ */
