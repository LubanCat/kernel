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
#include <drv_types.h>

#ifdef CONFIG_RTW_FSM_BTM

#define BTM_CONNECT_WAIT 2000
#define BTM_DISASSOC_WAIT 1000
#ifdef CONFIG_IEEE80211_BAND_6GHZ
#define BTM_SCAN_TIMEOUT 15000
#else
#define BTM_SCAN_TIMEOUT 10000
#endif
#define BTM_SCAN_DENY_WAIT 500
#define BTM_FT_ACTION_TIMEOUT 1000
#define BTM_SCAN_DWELL_TIME 30
/*
* State machine
*/
enum BTM_STATE_ST {
	BTM_ST_START,
	BTM_ST_SCAN,
	BTM_ST_OTD,
	BTM_ST_ROAM,
	BTM_ST_END,
	BTM_ST_MAX
};

enum BTM_EV_ID {
	BTM_EV_scan_deny_expire,
	BTM_EV_scan_timeout,
	BTM_EV_scan_found_candidate,
	BTM_EV_scan_opch_reported,
	BTM_EV_connect_timeout,
	BTM_EV_disassoc_timeout,
	BTM_EV_ft_action_timeout,
	BTM_EV_ft_action_resp,
	BTM_EV_max
};

static int btm_start_st_hdl(void *obj, u16 event, void *param);
static int btm_scan_st_hdl(void *obj, u16 event, void *param);
static int btm_otd_st_hdl(void *obj, u16 event, void *param);
static int btm_roam_st_hdl(void *obj, u16 event, void *param);
static int btm_end_st_hdl(void *obj, u16 event, void *param);

/* STATE table */
static struct fsm_state_ent btm_state_tbl[] = {
	ST_ENT(BTM_ST_START, btm_start_st_hdl),
	ST_ENT(BTM_ST_SCAN, btm_scan_st_hdl),
	ST_ENT(BTM_ST_OTD, btm_otd_st_hdl),
	ST_ENT(BTM_ST_ROAM, btm_roam_st_hdl),
	ST_ENT(BTM_ST_END, btm_end_st_hdl),
};

/* EVENT table */
static struct fsm_event_ent btm_event_tbl[] = {
	EV_DBG(BTM_EV_scan_deny_expire),
	EV_DBG(BTM_EV_scan_timeout),
	EV_DBG(BTM_EV_scan_found_candidate),
	EV_DBG(BTM_EV_scan_opch_reported),
	EV_DBG(BTM_EV_connect_timeout),
	EV_DBG(BTM_EV_disassoc_timeout),
	EV_DBG(BTM_EV_ft_action_timeout),
	EV_DBG(BTM_EV_ft_action_resp),
	EV_DBG(BTM_EV_max) /* EV_MAX for fsm safety checking */
};

struct btm_obj {

	u32 cid;
	u8 resp_reason;
	u8 roam_reason;
	u8 ft_req_retry_cnt;
	u8 op_ch_reported;
	struct roam_nb_info pnb;
	struct wlan_network *candidate;
	int finish;

	/* obj default */
	struct fsm_obj *obj;
	struct fsm_main *fsm;
	_list list;
};

/*
 * btm sub function
 */
#if CONFIG_IEEE80211_BAND_6GHZ
/* save PSC channel and remove other channel */
static int btm_scan_ch_psc(struct btm_obj *pbtm, struct rtw_ieee80211_channel *out, u32 out_num)
{
	int i, ch_num = 0;
	struct rtw_ieee80211_channel *chs;

	chs = rtw_zmalloc(sizeof(struct rtw_ieee80211_channel) * out_num);
	if (!chs)
		return out_num;

	_rtw_memcpy(chs, out, sizeof(struct rtw_ieee80211_channel) * out_num);
	_rtw_memset(out, 0, sizeof(struct rtw_ieee80211_channel) * out_num);

	for (i = 0; i < out_num; i ++) {
		if (chs[i].band != BAND_ON_6G)
			continue;

		 /* PSC = 16*n+5 */
		if ((chs[i].hw_value-5)%16 != 0)
			continue;

		/* Save PSC channel */
		_rtw_memcpy(&out[ch_num++], &chs[i], sizeof(struct rtw_ieee80211_channel));
	}
	rtw_mfree(chs, sizeof(struct rtw_ieee80211_channel) * out_num);
	return ch_num;
}
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

static int btm_scan_ch_sort(struct btm_obj *pbtm, struct rtw_ieee80211_channel *out, u32 out_num)
{
	int i, j, k, l;
	struct rtw_ieee80211_channel *chs;
	_adapter *a = obj2adp(pbtm);
	u8 *band_order = a->mlmepriv.roam_scan_order;

	chs = rtw_zmalloc(sizeof(struct rtw_ieee80211_channel) * out_num);
	if (!chs)
		return out_num;

	_rtw_memcpy(chs, out, sizeof(struct rtw_ieee80211_channel) * out_num);
	_rtw_memset(out, 0, sizeof(struct rtw_ieee80211_channel) * out_num);

	k = 0;
	for (i = 0; i < 3; i++) {
		if (band_order[i] >= BAND_MAX)
			break;
		for (j = 0; j < out_num; j++) {
			if (chs[j].hw_value == 0)
				break;
			if (chs[j].band != band_order[i])
				continue;
			if (chs[j].flags & RTW_IEEE80211_CHAN_PASSIVE_SCAN)
				continue;

			/* ACTIVE CH */
			_rtw_memcpy(&out[k++], &chs[j], sizeof(struct rtw_ieee80211_channel));
		}

		for (j = 0; j < out_num; j++) {
			if (chs[j].hw_value == 0)
				break;
			if (chs[j].band != band_order[i])
				continue;
			if (!(chs[j].flags & RTW_IEEE80211_CHAN_PASSIVE_SCAN))
				continue;

			/* PASSIVE CH */
			_rtw_memcpy(&out[k++], &chs[j], sizeof(struct rtw_ieee80211_channel));

		}
	}
	rtw_mfree(chs, sizeof(struct rtw_ieee80211_channel) * out_num);
	return k;
}

/* extend roam channel into full scan channel */
static int btm_roam_ch_decision(struct btm_obj *pbtm, struct rtw_ieee80211_channel *out,
		u32 out_num, struct rtw_ieee80211_channel *in_ch, u32 in_num)
{
	_adapter *a = obj2adp(pbtm);
	struct rtw_ieee80211_channel *roam_ch = NULL, *full_ch = NULL;
	int i, j, k, roam_ch_num = 0, full_ch_num = 0;

	/* sort original ROAM channel */
	if (in_num) {
		roam_ch = rtw_zmalloc(sizeof(struct rtw_ieee80211_channel) * in_num);
		if (!roam_ch)
			goto roam_exit;
		roam_ch_num = rtw_scan_ch_decision(a, roam_ch,
					in_num, in_ch, in_num, _TRUE);
		roam_ch_num = btm_scan_ch_sort(pbtm, roam_ch, roam_ch_num);
	}

	full_ch = rtw_zmalloc(sizeof(struct rtw_ieee80211_channel) * RTW_CHANNEL_SCAN_AMOUNT);
	if (!full_ch)
		goto full_exit;

	/* get full channel */
	full_ch_num = rtw_scan_ch_decision(a, full_ch,
				RTW_CHANNEL_SCAN_AMOUNT, NULL, 0, _TRUE);

#if CONFIG_IEEE80211_BAND_6GHZ
	/* only accept 6G PSC channel */
	full_ch_num = btm_scan_ch_psc(pbtm, full_ch, full_ch_num);
#endif
	/* sort full channel */
	full_ch_num = btm_scan_ch_sort(pbtm, full_ch, full_ch_num);

	_rtw_memset(out, 0, sizeof(struct rtw_ieee80211_channel) * out_num);

	if (roam_ch_num) {
		/* combine ROAM ch and full ch */
		_rtw_memcpy(out, roam_ch, sizeof(struct rtw_ieee80211_channel) * roam_ch_num);

		k = roam_ch_num;
		for (i = 0; i < full_ch_num; i++) {
			for (j = 0; j < k; j++) {
				if (roam_ch[j].hw_value == full_ch[i].hw_value)
					goto next;
			}
			_rtw_memcpy(&out[roam_ch_num++] ,&full_ch[i], sizeof(struct rtw_ieee80211_channel));
next:
			; /* make compile happy:label at end of compound statement */
		}
	} else {
		roam_ch_num = full_ch_num;
		_rtw_memcpy(out ,full_ch, sizeof(struct rtw_ieee80211_channel) * full_ch_num);
	}

	if (full_ch)
		rtw_mfree(full_ch, sizeof(struct rtw_ieee80211_channel) * RTW_CHANNEL_SCAN_AMOUNT);
full_exit:
	if (roam_ch)
		rtw_mfree(roam_ch, sizeof(struct rtw_ieee80211_channel) * in_num);
roam_exit:
#if 0
	for (i = 0; i < roam_ch_num; i++) {
		RTW_INFO("roam ch: %d\n", out[i].hw_value);
	}
#endif
	return roam_ch_num;
}

static void btm_nb_scan_list_set(struct btm_obj *pbtm, struct sitesurvey_parm *pparm)
{
	u32 i;
	struct roam_nb_info *pnb = &pbtm->pnb;
	_adapter *a = obj2adp(pbtm);
	struct mlme_priv *pmlmepriv = &a->mlmepriv;

	if (pmlmepriv->cur_network_scanned) {
		_rtw_memcpy(&pparm->ssid[0], &pmlmepriv->cur_network_scanned->network.Ssid,
			sizeof(struct _NDIS_802_11_SSID));
		pparm->ssid_num = 1;
	}

	for (i = 0; i < pnb->nb_rpt.nb_list_num; i++) {
		_rtw_memcpy(pparm->nb[i].bssid, pnb->nb_rpt.nb_list[i].ent.bssid, ETH_ALEN);
		pparm->nb[i].ch = pnb->nb_rpt.nb_list[i].ent.ch_num;
		pparm->nb_num++;
	}

	pparm->scan_mode = RTW_PHL_SCAN_ACTIVE;
	pparm->duration = BTM_SCAN_DWELL_TIME;
	pparm->ch_num = btm_roam_ch_decision(pbtm, pparm->ch, RTW_CHANNEL_SCAN_AMOUNT,
		pnb->nb_rpt.ch_list, pnb->nb_rpt.ch_list_num);
	pnb->nb_rpt_valid = _FALSE;
}

static int btm_roam_scan(struct btm_obj *pbtm)
{
	_adapter *a = obj2adp(pbtm);
	struct sitesurvey_parm *parm;
	u8 ret = _FAIL;

	parm = (struct sitesurvey_parm*)rtw_zmalloc(sizeof(*parm));

	if (!parm)
		return ret;

	rtw_init_sitesurvey_parm(parm);
	btm_nb_scan_list_set(pbtm, parm);
	parm->scan_type = RTW_SCAN_ROAM;
	parm->psta = obj2sta(pbtm);

	ret = rtw_sitesurvey_cmd(a, parm);
	rtw_mfree(parm, sizeof(*parm));

	return ret;
}

static int btm_del_obj(struct btm_obj *pbtm)
{
	struct btm_priv *btmpriv = obj2priv(pbtm);
	struct sta_info *psta = obj2sta(pbtm);

	psta->btm = NULL;
	btmpriv->btm = NULL;

	rtw_fsm_deactivate_obj(pbtm);

	return _SUCCESS;
}

/*
 * BTM state handler
 */
static int btm_start_st_hdl(void *obj, u16 event, void *param)
{
	struct btm_obj *pbtm = (struct btm_obj *)obj;
	_adapter *a = obj2adp(pbtm);
	struct btm_priv *btmpriv = (struct btm_priv *)obj2priv(pbtm);
	struct roam_nb_info *pnb = &pbtm->pnb;
	u8 ssc_chk;
	int i;

	switch (event) {
	case FSM_EV_STATE_IN:
		pbtm->resp_reason = 1; /* unspecified reject reason */
		for (i = 0; i < pnb->nb_rpt.nb_list_num; i++)
			FSM_INFO(pbtm, "roam list "MAC_FMT"\n",
				MAC_ARG(pnb->nb_rpt.nb_list[i].ent.bssid));
		fallthrough;
	case BTM_EV_scan_deny_expire:
		ssc_chk = rtw_sitesurvey_condition_check(a, _FALSE);
		if (ssc_chk == SS_DENY_BUSY_TRAFFIC) {
			FSM_INFO(pbtm, "need to roam, don't care BusyTraffic\n");
		} else if (ssc_chk != SS_ALLOW) {
			rtw_fsm_set_alarm(pbtm, BTM_SCAN_DENY_WAIT, BTM_EV_scan_deny_expire);
			return _SUCCESS;
		}
		/* Able to scan */
		rtw_fsm_st_goto(pbtm, BTM_ST_SCAN);
		break;
	case FSM_EV_ABORT:
	case FSM_EV_DISCONNECTED:
		if (pbtm->roam_reason & RTW_ROAM_BTM)
			rtw_wnm_issue_action(a, &pbtm->pnb, RTW_WLAN_ACTION_WNM_BTM_RSP,
				pbtm->resp_reason, pbtm->pnb.btm_cache.dialog_token);
		rtw_fsm_st_goto(pbtm, BTM_ST_END);
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(pbtm);
		break;
	default:
		break;
	}

	return _SUCCESS;
}

static int btm_scan_st_hdl(void *obj, u16 event, void *param)
{
	struct btm_obj *pbtm = (struct btm_obj *)obj;
	_adapter *a = obj2adp(pbtm);
	struct mlme_priv *pmlmepriv = &a->mlmepriv;
	struct roam_nb_info *pnb = &pbtm->pnb;
	struct wlan_network *pnetwork = NULL;
	struct wlan_network *cnetwork = pmlmepriv->cur_network_scanned;
	struct roam_nb_info *mlmepriv_pnb = &(pmlmepriv->nb_info);

	switch (event) {
	case FSM_EV_STATE_IN:
		if (!btm_roam_scan(pbtm)) {
			rtw_fsm_st_goto(pbtm, BTM_ST_END);
			return _SUCCESS;
		}
		pbtm->op_ch_reported = 0;
		/* Set scan timeout */
		rtw_fsm_set_alarm(pbtm, BTM_SCAN_TIMEOUT, BTM_EV_scan_timeout);
		break;
	case BTM_EV_scan_opch_reported:
		pbtm->op_ch_reported = 1;
		break;
	case BTM_EV_scan_found_candidate:
		pnetwork = (struct wlan_network *)param;

		FSM_DBG(pbtm, "Recv %s ("MAC_FMT")\n",pnetwork->network.Ssid.Ssid,
			MAC_ARG(pnetwork->network.MacAddress));

		if ((pnetwork->network.Configuration.DSConfig ==\
			cnetwork->network.Configuration.DSConfig) &&
			(pnetwork->network.Configuration.Band ==\
			 cnetwork->network.Configuration.Band)) {

			if (pbtm->op_ch_reported) /* already reported */
				return _SUCCESS;
			rtw_fsm_set_alarm_ext(pbtm, 100, BTM_EV_scan_opch_reported, 1, NULL);
		}

		if (pbtm->roam_reason & RTW_ROAM_ACTIVE)
			rtw_check_roaming_candidate(pmlmepriv, &pbtm->candidate, pnetwork);
		else if (pbtm->roam_reason & RTW_ROAM_BTM)
			pbtm->candidate = rtw_wnm_btm_candidate_check(a, &pbtm->pnb, pnetwork);
		else if (pbtm->roam_reason & RTW_ROAM_ON_RESUME)
			rtw_check_join_candidate(pmlmepriv, &pbtm->candidate, pnetwork);
		else
			return _SUCCESS;

		if (!pbtm->candidate)
			return _SUCCESS;

		a->mlmepriv.roam_network = pbtm->candidate;
		rtw_scan_abort_no_wait(a);
		/* wait scan done */
		break;
	case FSM_EV_SCAN_DONE:
		rtw_fsm_cancel_alarm(pbtm);

		if (pbtm->roam_reason & RTW_ROAM_ACTIVE) {
			if (!pbtm->candidate) {
				FSM_INFO(pbtm, "Pick a candidate from scan queue\n");
				pbtm->candidate = rtw_select_roaming_candidate(pmlmepriv);
			}

		} else if (pbtm->roam_reason & RTW_ROAM_BTM) {

			pbtm->resp_reason = rtw_wmn_btm_rsp_reason_decision(a, pnb);
			if (pbtm->resp_reason > 0) {
				pbtm->candidate = NULL;
				rtw_fsm_st_goto(pbtm, BTM_ST_END);
				return _SUCCESS;
			}
			if (!pbtm->candidate) {
				FSM_INFO(pbtm, "Pick a candidate from scan queue\n");
#ifdef DBG_WNM_UNIT_TEST
				pbtm->candidate = rtw_wnm_btm_candidate_select(a, pnb);
#else
				if (!rtw_mbo_wifi_logo_test(a))
					pbtm->candidate = rtw_select_roaming_candidate(pmlmepriv);
				else
					pbtm->candidate = rtw_wnm_btm_candidate_select(a, pnb);
#endif
			}
			if (pbtm->candidate) {
				_rtw_memcpy(pnb->roam_target_addr,
					(u8 *)&pbtm->candidate->network.MacAddress,
					ETH_ALEN);

				/* for ReAssocreq check in rtw_wnm_btm_reassoc_req() */
				/* TODO Do NOT use global variable */
				_rtw_memcpy(mlmepriv_pnb->roam_target_addr,
					(u8 *)&pbtm->candidate->network.MacAddress,
					ETH_ALEN);
			}

		} else if (pbtm->roam_reason & RTW_ROAM_ON_RESUME) {
			if (rtw_select_and_join_from_scanned_queue(pmlmepriv))
				pbtm->finish = _SUCCESS;

			rtw_fsm_st_goto(pbtm, BTM_ST_END);
			return _SUCCESS;
		}

		/* RTW_ROAM_ACTIVE */
		if (!pbtm->candidate) {
			if (pbtm->resp_reason == 0)
				pbtm->resp_reason = 7;

			rtw_fsm_st_goto(pbtm, BTM_ST_END);
			return _SUCCESS;
		}
		a->mlmepriv.roam_network = pbtm->candidate;

		if (rtw_ft_otd_roam(a))
			rtw_fsm_st_goto(pbtm, BTM_ST_OTD); /* Over The DS */
		else
			rtw_fsm_st_goto(pbtm, BTM_ST_ROAM);
		break;
	case BTM_EV_scan_timeout:
	case FSM_EV_ABORT:
	case FSM_EV_DISCONNECTED:
		rtw_scan_abort_no_wait(a);
		rtw_fsm_st_goto(pbtm, BTM_ST_END);
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(pbtm);
		rtw_fsm_cancel_alarm_ext(pbtm, 1);
		if (pbtm->roam_reason & RTW_ROAM_BTM)
			rtw_wnm_issue_action(a, pnb, RTW_WLAN_ACTION_WNM_BTM_RSP,
				pbtm->resp_reason, pnb->btm_cache.dialog_token);
		break;
	default:
		break;
	}

	return _SUCCESS;
}

static int btm_otd_st_hdl(void *obj, u16 event, void *param)
{
	struct btm_obj *pbtm = (struct btm_obj *)obj;
	_adapter *a = obj2adp(pbtm);
	struct mlme_priv *pmlmepriv = &a->mlmepriv;
	struct ft_roam_info *pft_roam = &(pmlmepriv->ft_roam);

	switch (event) {
	case FSM_EV_STATE_IN:
		FSM_INFO(pbtm, "FT OTD roam\n");
		fallthrough;
	case BTM_EV_ft_action_timeout:
		if (pbtm->ft_req_retry_cnt++ >= RTW_FT_ACTION_REQ_LMT) {
			FSM_WARN(pbtm, "RX FT action timeout, try OTA roam\n");
			rtw_ft_clr_flags(a, RTW_FT_PEER_OTD_EN);
			rtw_fsm_st_goto(pbtm, BTM_ST_ROAM);
			return _SUCCESS;
		}
		/* send ft action frame */
		rtw_ft_issue_action_req(a, pbtm->candidate->network.MacAddress);
		rtw_fsm_set_alarm(pbtm, BTM_FT_ACTION_TIMEOUT, BTM_EV_ft_action_timeout);
		break;
	case BTM_EV_ft_action_resp:
		rtw_fsm_cancel_alarm(pbtm);
		rtw_fsm_st_goto(pbtm, BTM_ST_ROAM);
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(pbtm);
		break;
	case FSM_EV_DISCONNECTED:
	case FSM_EV_ABORT:
		rtw_fsm_st_goto(pbtm, BTM_ST_END);
		break;
	default:
		break;
	}
	return _SUCCESS;
}

static int btm_roam_st_hdl(void *obj, u16 event, void *param)
{
	struct btm_obj *pbtm = (struct btm_obj *)obj;
	_adapter *a = obj2adp(pbtm);
	struct mlme_priv *pmlmepriv = &a->mlmepriv;
	struct mlme_ext_info *pmlmeinfo = &a->mlmeextpriv.mlmext_info;

	switch (event) {
	case FSM_EV_STATE_IN:
		RTW_INFO("roaming from %s("MAC_FMT"), length:%d\n",
			pmlmepriv->dev_cur_network.network.Ssid.Ssid,
			MAC_ARG(pmlmepriv->dev_cur_network.network.MacAddress),
			pmlmepriv->dev_cur_network.network.Ssid.SsidLength);

		if (pbtm->pnb.btm_cache.req_mode & DISASSOC_IMMINENT) {
			/* indicates that the STA is to be disassociated
			 * from the current AP */
			rtw_fsm_set_alarm(pbtm, BTM_DISASSOC_WAIT, BTM_EV_disassoc_timeout);
			return _SUCCESS;
		}
		fallthrough;
	case BTM_EV_disassoc_timeout:
		if (rtw_ft_otd_roam(a)) {
			pmlmeinfo->disconnect_code = DISCONNECTION_BY_DRIVER_DUE_TO_FT;
		} else {
			rtw_set_to_roam(a, a->registrypriv.max_roaming_times);
			pmlmeinfo->disconnect_code = DISCONNECTION_BY_DRIVER_DUE_TO_ROAMING;
		}

		receive_disconnect(a, pmlmepriv->dev_cur_network.network.MacAddress
			, WLAN_REASON_ACTIVE_ROAM, _FALSE);
		break;
	case FSM_EV_DISCONNECTED:
		FSM_INFO(pbtm, "roaming to %s(" MAC_FMT ")\n",
			pbtm->candidate->network.Ssid.Ssid,
			MAC_ARG(pbtm->candidate->network.MacAddress));

		_rtw_memcpy(&pmlmepriv->assoc_ssid,
			&pmlmepriv->dev_cur_network.network.Ssid, sizeof(NDIS_802_11_SSID));
		pmlmepriv->assoc_ch = 0;
		pmlmepriv->assoc_band = BAND_MAX;
		pmlmepriv->assoc_by_bssid = _FALSE;

		/* Start to connect */
		if (!rtw_do_join(a)) {
			rtw_fsm_st_goto(pbtm, BTM_ST_END);
		}
		rtw_fsm_set_alarm(pbtm, BTM_CONNECT_WAIT, BTM_EV_connect_timeout);
		break;
	case FSM_EV_CONNECTED:
		pbtm->finish = _SUCCESS;
		rtw_set_to_roam(a, 0);
		rtw_fsm_st_goto(pbtm, BTM_ST_END);
		break;
	case FSM_EV_CONNECT_FAIL:
		/* don't trigger disconnect. Connect module will do it! */
		pbtm->finish = _SUCCESS;
		fallthrough;
	case FSM_EV_ABORT:
	case BTM_EV_connect_timeout:
		rtw_fsm_st_goto(pbtm, BTM_ST_END);
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(pbtm);
		break;
	default:
		break;
	}

	return _SUCCESS;
}

static int btm_end_st_hdl(void *obj, u16 event, void *param)
{
	struct btm_obj *pbtm = (struct btm_obj *)obj;
	_adapter *a = obj2adp(pbtm);

	switch (event) {
	case FSM_EV_STATE_IN:

		if (pbtm->finish == _FAIL) {
			rtw_set_to_roam(a, 0);
			rtw_ft_reset_status(a);
			if (pbtm->candidate) /* driver disconnected */
				rtw_indicate_disconnect(a, 0, _FALSE);
		}
		btm_del_obj(pbtm);
		break;
	case FSM_EV_ABORT:
		break;
	case FSM_EV_STATE_OUT:
		break;
	default:
		break;
	}

	return _SUCCESS;
}

static void btm_dump_obj_cb(void *obj, char *p, int *sz)
{
	/* nothing to do for now */
}

static void btm_dump_fsm_cb(void *fsm, char *p, int *sz)
{
	/* nothing to do for now */
}

static int btm_init_priv_cb(void *priv)
{
	struct btm_priv *pbtmpriv = (struct btm_priv *)priv;

	return _SUCCESS;
}

static int btm_deinit_priv_cb(void *priv)
{
	struct btm_priv *pbtmpriv = (struct btm_priv *)priv;

	return _SUCCESS;
}

static void btm_debug(void *obj, char input[][MAX_ARGV], u32 input_num,
	char *output, u32 *out_len)
{
	return;
}

/* For EXTERNAL application to create a btm object
 * return
 * btm_obj: ptr to new btm object
 */
int rtw_btm_new_obj(_adapter *a, struct sta_info *psta,
	struct roam_nb_info *pnb, u8 roam_reason)
{
	struct btm_priv *btmpriv = &a->fsmpriv.btmpriv;
	struct fsm_main *fsm = btmpriv->fsm;
	struct fsm_obj *obj;
	struct btm_obj *pbtm;
	struct mlme_priv *pmlmepriv = &a->mlmepriv;
	u16 cid = 0;

	if (btmpriv->btm) {
		FSM_WARN_(fsm, "BTM request is on going ...\n");
		return _FAIL;
	}

	if (!psta)
		psta = rtw_get_stainfo(&a->stapriv, get_bssid(pmlmepriv));

	if (!psta)
		psta = rtw_get_stainfo(&a->stapriv, a->mac_addr);

	if (!psta) {
		FSM_WARN_(fsm, "psta NOT found\n");
		return _FAIL;
	}

	if (roam_reason == RTW_ROAM_BTM)
		cid = pnb->btm_cache.dialog_token;

	obj = rtw_fsm_new_obj(fsm, psta, cid, (void **)&pbtm, sizeof(*pbtm));

	if (pbtm == NULL) {
		FSM_ERR_(fsm, "malloc obj fail\n");
		return _FAIL;
	}
	pbtm->fsm = fsm;
	pbtm->obj = obj;
	pbtm->cid = cid;
	pbtm->roam_reason = roam_reason;
	pbtm->finish = _FAIL;

	_rtw_memcpy(&pbtm->pnb, pnb, sizeof(*pnb));

	/* activate it immediately */
	rtw_fsm_activate_obj(pbtm);
	pmlmepriv->need_to_roam = _TRUE;
	btmpriv->btm = pbtm;
	psta->btm = pbtm;

	return _SUCCESS;
}

void rtw_btm_notify_action_resp(struct btm_obj *pbtm)
{
	rtw_fsm_gen_msg(pbtm, NULL, 0, BTM_EV_ft_action_resp);
}

void rtw_btm_notify_scan_found_candidate(struct btm_obj *pbtm, struct wlan_network *pnetwork)
{
	_adapter *a = obj2adp(pbtm);

	if (!rtw_chk_roam_flags(a, RTW_ROAM_QUICK_SCAN))
		return;

	if (!pnetwork)
		return;
	rtw_fsm_gen_msg(pbtm, (void *)pnetwork, 0, BTM_EV_scan_found_candidate);
}

/* For EXTERNAL application to create RRM FSM */
int rtw_btm_reg_fsm(struct fsm_priv *fsmpriv)
{
	struct fsm_root *root = fsmpriv->root;
	struct fsm_main *fsm = NULL;
	struct rtw_fsm_tb tb;
	struct btm_priv *btmpriv = &fsmpriv->btmpriv;

	memset(&tb, 0, sizeof(tb));
	tb.max_state = sizeof(btm_state_tbl)/sizeof(btm_state_tbl[0]);
	tb.max_event = sizeof(btm_event_tbl)/sizeof(btm_event_tbl[0]);
	tb.state_tbl = btm_state_tbl;
	tb.evt_tbl = btm_event_tbl;
	tb.priv = btmpriv;
	tb.init_priv = btm_init_priv_cb;
	tb.deinit_priv = btm_deinit_priv_cb;
	tb.dump_obj = btm_dump_obj_cb;
	tb.dump_fsm = btm_dump_fsm_cb;
	tb.dbg_level = FSM_LV_INFO;
	tb.evt_level = FSM_LV_INFO;
	tb.debug = btm_debug;

	fsm = rtw_fsm_register_fsm(root, "wnm", &tb);
	btmpriv->fsm = fsm;

	if (!fsm)
		return _FAIL;

	return _SUCCESS;
}
#endif /* CONFIG_RTW_FSM_BTM */
