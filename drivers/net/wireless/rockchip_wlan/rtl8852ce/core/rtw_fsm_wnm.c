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
#if CONFIG_IEEE80211_BAND_6GHZ
#define BTM_SCAN_TIMEOUT 15000
#else
#define BTM_SCAN_TIMEOUT 10000
#endif
#define BTM_SCAN_DENY_WAIT 500
#define BTM_FT_ACTION_TIMEOUT 1000

#define BTM_NB_POLL_INTERVAL 10
#define BTM_NB_POLL_TIMES 20
/*
* State machine
*/
enum BTM_STATE_ST {
	BTM_ST_START,
	BTM_ST_NB,
	BTM_ST_SCAN,
	BTM_ST_OTD,
	BTM_ST_ROAM,
	BTM_ST_END,
	BTM_ST_MAX
};

enum BTM_EV_ID {
	BTM_EV_nb_poll,
	BTM_EV_scan_deny_expire,
	BTM_EV_scan_timeout,
	BTM_EV_scan_probe_req,
	BTM_EV_scan_probe_req_timeout,
	BTM_EV_scan_found_candidate,
	BTM_EV_scan_opch_reported,
	BTM_EV_connect_timeout,
	BTM_EV_disassoc_timeout,
	BTM_EV_ft_action_timeout,
	BTM_EV_ft_action_resp,
	BTM_EV_max
};

static int btm_start_st_hdl(void *obj, u16 event, void *param);
static int btm_nb_req_st_hdl(void *obj, u16 event, void *param);
static int btm_scan_st_hdl(void *obj, u16 event, void *param);
static int btm_otd_st_hdl(void *obj, u16 event, void *param);
static int btm_roam_st_hdl(void *obj, u16 event, void *param);
static int btm_end_st_hdl(void *obj, u16 event, void *param);

/* STATE table */
static struct fsm_state_ent btm_state_tbl[] = {
	ST_ENT(BTM_ST_START, btm_start_st_hdl),
	ST_ENT(BTM_ST_NB, btm_nb_req_st_hdl),
	ST_ENT(BTM_ST_SCAN, btm_scan_st_hdl),
	ST_ENT(BTM_ST_OTD, btm_otd_st_hdl),
	ST_ENT(BTM_ST_ROAM, btm_roam_st_hdl),
	ST_ENT(BTM_ST_END, btm_end_st_hdl),
};

/* EVENT table */
static struct fsm_event_ent btm_event_tbl[] = {
	EV_DBG(BTM_EV_nb_poll),
	EV_DBG(BTM_EV_scan_deny_expire),
	EV_DBG(BTM_EV_scan_timeout),
	EV_DBG(BTM_EV_scan_probe_req),
	EV_DBG(BTM_EV_scan_probe_req_timeout),
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
	int nb_poll_times;
	int finish;
	int unexpect_terminated;

	struct sitesurvey_parm parm;
	struct rtw_ieee80211_channel scan_ch;

	/* obj default */
	struct fsm_obj *obj;
	struct fsm_main *fsm;
	_list list;
};

/*
 * btm sub function
 */
static void btm_dump_scan_ch(struct rtw_ieee80211_channel *ch, u32 num)
{
	int i;
	char str[128];

	if (num == 0)
		return;

	memset(str, 0, sizeof(str));
	for (i = 0; i < num; i++) {
		sprintf(str + strlen(str) ,"[%3d-%s] ",
			ch[i].hw_value, rtw_band_str(ch[i].band));

		if ((i+1)%8 == 0) {
			printk("%s\n",str);
			memset(str, 0, sizeof(str));
		}
	}
	if (strlen(str))
		printk("%s\n",str);
}

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
		if (chs[i].band != BAND_ON_6G) {
			_rtw_memcpy(&out[ch_num++], &chs[i], sizeof(struct rtw_ieee80211_channel));
			continue;
		}

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

static int btm_scan_ch_sort(struct btm_obj *pbtm, struct rtw_ieee80211_channel *out, u32 out_num, u8 remove_op)
{
	int i, j, k, l;
	struct rtw_ieee80211_channel *chs;
	_adapter *a = obj2adp(pbtm);
	struct mlme_priv *pmlmepriv = &a->mlmepriv;
	u8 *band_order = a->mlmepriv.roam_scan_order;
	u16 c_ch = pmlmepriv->cur_network_scanned->network.Configuration.DSConfig;
	u8 c_band = pmlmepriv->cur_network_scanned->network.Configuration.Band;

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
			if (remove_op && chs[j].hw_value == c_ch && chs[j].band == c_band)
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
			if (remove_op && chs[j].hw_value == c_ch && chs[j].band == c_band)
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
	struct mlme_priv *pmlmepriv = &a->mlmepriv;
	struct rtw_ieee80211_channel *roam_ch = NULL, *full_ch = NULL;
	int i, j, k, roam_ch_num = 0, full_ch_num = 0, ins_op_ch = 0;
	u16 c_ch = pmlmepriv->cur_network_scanned->network.Configuration.DSConfig;
	u8 c_band = pmlmepriv->cur_network_scanned->network.Configuration.Band;

	/* sort original ROAM channel */
	if (in_num) {
		roam_ch = rtw_zmalloc(sizeof(struct rtw_ieee80211_channel) * (in_num + ins_op_ch));
		if (!roam_ch)
			goto roam_exit;
		roam_ch_num = rtw_scan_ch_decision(a, roam_ch,
					in_num, in_ch, in_num, 0);

		if (roam_ch_num > 1)
			roam_ch_num = btm_scan_ch_sort(pbtm, roam_ch, roam_ch_num, ins_op_ch);
	}

	full_ch = rtw_zmalloc(sizeof(struct rtw_ieee80211_channel) * RTW_CHANNEL_SCAN_AMOUNT);
	if (!full_ch)
		goto full_exit;

	/* get full channel */
	full_ch_num = rtw_scan_ch_decision(a, full_ch,
				RTW_CHANNEL_SCAN_AMOUNT, NULL, 0, 0);

#if CONFIG_IEEE80211_BAND_6GHZ
	/* only accept 6G PSC channel */
	full_ch_num = btm_scan_ch_psc(pbtm, full_ch, full_ch_num);
#endif
	/* sort full channel */
	full_ch_num = btm_scan_ch_sort(pbtm, full_ch, full_ch_num, ins_op_ch);

	_rtw_memset(out, 0, sizeof(struct rtw_ieee80211_channel) * out_num);

	/* insert current op channel to 1'st entry */
	k = 0;
	if (ins_op_ch) {
		out[k].hw_value = c_ch;
		out[k].band = c_band;
		k++;
	}

	if (roam_ch_num) {
		/* combine ROAM ch and full ch */
		for (i = 0; i < roam_ch_num; i++) {
			if (ins_op_ch && (roam_ch[i].hw_value == c_ch) && (roam_ch[i].band == c_band))
				goto next_roam_ch;

			_rtw_memcpy(&out[k++] ,&roam_ch[i], sizeof(struct rtw_ieee80211_channel));
next_roam_ch:
			; /* make compile happy:label at end of compound statement */
		}

		for (i = 0; i < full_ch_num; i++) {
			for (j = 0; j < (roam_ch_num + ins_op_ch); j++) {
				if ((roam_ch[j].hw_value == full_ch[i].hw_value) &&
					(roam_ch[j].band == full_ch[i].band))
					goto next_full_ch;
			}
			_rtw_memcpy(&out[k++] ,&full_ch[i], sizeof(struct rtw_ieee80211_channel));
next_full_ch:
			; /* make compile happy:label at end of compound statement */
		}
	} else {
		_rtw_memcpy(&out[ins_op_ch] ,full_ch, sizeof(struct rtw_ieee80211_channel) * full_ch_num);
		k += full_ch_num;
	}

	if (full_ch)
		rtw_mfree(full_ch, sizeof(struct rtw_ieee80211_channel) * RTW_CHANNEL_SCAN_AMOUNT);
full_exit:
	if (roam_ch)
		rtw_mfree(roam_ch, sizeof(struct rtw_ieee80211_channel) * (in_num + ins_op_ch));
roam_exit:
	btm_dump_scan_ch(out, k);
	return k;
}

static void btm_nb_scan_list_set(struct btm_obj *pbtm, struct sitesurvey_parm *pparm)
{
	int i, j, sz;
	struct roam_nb_info *pnb = &pbtm->pnb;
	_adapter *a = obj2adp(pbtm);
	struct mlme_priv *pmlmepriv = &a->mlmepriv;
	struct rtw_ieee80211_channel *chs;

	/* SSID */
	if (pmlmepriv->cur_network_scanned) {
		_rtw_memcpy(&pparm->ssid[0], &pmlmepriv->cur_network_scanned->network.Ssid,
			sizeof(struct _NDIS_802_11_SSID));
		pparm->ssid_num = 1;
	}

	/* NB report specified bssid */
	for (i = 0; i < pnb->nb_rpt.nb_list_num; i++) {
		_rtw_memcpy(pparm->nb[i].bssid, pnb->nb_rpt.nb_list[i].ent.bssid, ETH_ALEN);

		pparm->nb[i].ch = pnb->nb_rpt.nb_list[i].ent.ch_num;
		pparm->nb[i].band = pnb->nb_rpt.ch_list[i].band;
		pparm->nb_num++;
	}
	pparm->scan_mode = RTW_PHL_SCAN_ACTIVE;
	pparm->duration = ROAM_P_CH_SURVEY_TO;
	pparm->ch_num = btm_roam_ch_decision(pbtm, pparm->ch, RTW_CHANNEL_SCAN_AMOUNT,
		pnb->nb_rpt.ch_list, pnb->nb_rpt.ch_list_num);
	pnb->nb_rpt_valid = _FALSE;
}

static int btm_roam_scan(struct btm_obj *pbtm)
{
	_adapter *a = obj2adp(pbtm);
	struct sitesurvey_parm *parm = &pbtm->parm;

	rtw_init_sitesurvey_parm(parm);
	btm_nb_scan_list_set(pbtm, parm);
	parm->scan_type = RTW_SCAN_ROAM;
	parm->psta = obj2sta(pbtm);

	return rtw_sitesurvey_cmd(a, parm);
}

static int btm_issue_probe_req(struct btm_obj *pbtm)
{
	_adapter *a = obj2adp(pbtm);
	struct sta_info *psta = obj2sta(pbtm);
	struct _ADAPTER_LINK *al = psta->padapter_link;
	NDIS_802_11_SSID *ssid;
	int i, j;

	if (!pbtm->parm.ssid_num) {
		issue_probereq(a, al, NULL, NULL);
		return _SUCCESS;
	}

	for (i = 0; i < pbtm->parm.ssid_num; i++) {
		if (pbtm->parm.ssid[i].SsidLength == 0)
			continue;

		ssid = &pbtm->parm.ssid[i];

		for (j = 0; j < pbtm->parm.nb_num; j++) {
			if (pbtm->scan_ch.hw_value == pbtm->parm.nb[j].ch &&
			    pbtm->scan_ch.band == pbtm->parm.nb[j].band)
				issue_probereq(a, al, ssid, pbtm->parm.nb[j].bssid);
		}
		issue_probereq(a, al, ssid, NULL);
	}

	return _SUCCESS;
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

	switch (event) {
	case FSM_EV_STATE_IN:
		if (btmpriv->btm) {
			FSM_ERR(pbtm, "pbtm exists\n");
			rtw_fsm_st_goto(pbtm, BTM_ST_END);
			return _SUCCESS;
		} else {
			btmpriv->btm = pbtm;
		}
		FSM_INFO(pbtm, "roam start, reason=0x%02x\n", pbtm->roam_reason);

		pbtm->resp_reason = 1; /* unspecified reject reason */
		if (pbtm->roam_reason & RTW_ROAM_ACTIVE &&
			!rtw_rrm_get_nb_rpt(a, &pbtm->pnb.nb_rpt)) /* invalid */
			rtw_fsm_st_goto(pbtm, BTM_ST_NB);
		else
			rtw_fsm_st_goto(pbtm, BTM_ST_SCAN);
		break;
	case FSM_EV_ABORT:
		if (pbtm->roam_reason & RTW_ROAM_BTM)
			rtw_wnm_issue_action(a, &pbtm->pnb, RTW_WLAN_ACTION_WNM_BTM_RSP,
				pbtm->resp_reason, pbtm->pnb.btm_cache.dialog_token);
		fallthrough;
	case FSM_EV_DISCONNECT_START: /* no need to reply btm_req */
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

static int btm_nb_req_st_hdl(void *obj, u16 event, void *param)
{
	struct btm_obj *pbtm = (struct btm_obj *)obj;
	_adapter *a = obj2adp(pbtm);
	int i, valid;
	u8 band;

	switch (event) {
	case FSM_EV_STATE_IN:
		/* issue NB req */
		rm_add_nb_req(a, obj2sta(pbtm));

		rtw_fsm_set_alarm(pbtm, BTM_NB_POLL_INTERVAL, BTM_EV_nb_poll);
		pbtm->nb_poll_times = BTM_NB_POLL_TIMES;
		break;
	case BTM_EV_nb_poll:
		if (rtw_rrm_get_nb_rpt(a, &pbtm->pnb.nb_rpt) || (--pbtm->nb_poll_times) == 0)
			rtw_fsm_st_goto(pbtm, BTM_ST_SCAN); /* go ahead even there is no NB report */
		else
			rtw_fsm_set_alarm(pbtm, BTM_NB_POLL_INTERVAL, BTM_EV_nb_poll);
		break;
	case FSM_EV_ABORT:
		if (pbtm->roam_reason & RTW_ROAM_BTM)
			rtw_wnm_issue_action(a, &pbtm->pnb, RTW_WLAN_ACTION_WNM_BTM_RSP,
				pbtm->resp_reason, pbtm->pnb.btm_cache.dialog_token);
		fallthrough;
	case FSM_EV_DISCONNECT_START: /* no need to reply btm_req */
		rtw_fsm_st_goto(pbtm, BTM_ST_END);
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(pbtm);
		for (i = 0; i < pbtm->pnb.nb_rpt.nb_list_num; i++) {
			band = rtw_get_band_by_op_class(NULL, pbtm->pnb.nb_rpt.nb_list[i].ent.reg_class);
			FSM_INFO(pbtm, "roam list "MAC_FMT" opc:%3d, ch:%3d-%s\n",
				MAC_ARG(pbtm->pnb.nb_rpt.nb_list[i].ent.bssid),
				pbtm->pnb.nb_rpt.nb_list[i].ent.reg_class,
				pbtm->pnb.nb_rpt.nb_list[i].ent.ch_num,
				rtw_band_str(band));
		}
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
	WLAN_BSSID_EX *bss;
	u8 ssc_chk;

	switch (event) {
	case FSM_EV_STATE_IN:
		pbtm->unexpect_terminated = _FALSE;
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
	case BTM_EV_scan_probe_req:
		memcpy((void *)&pbtm->scan_ch, param, sizeof(struct rtw_ieee80211_channel));
		rtw_fsm_set_alarm_ext(pbtm, 10, BTM_EV_scan_probe_req_timeout, 2, NULL);
		fallthrough;
	case BTM_EV_scan_probe_req_timeout:
		btm_issue_probe_req(pbtm);
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
		if (pbtm->unexpect_terminated) {
			rtw_fsm_st_goto(pbtm, BTM_ST_END);
			return _SUCCESS;
		}
		rtw_fsm_cancel_alarm(pbtm);

		if (pbtm->roam_reason & RTW_ROAM_ACTIVE) {
			if (pbtm->candidate) {
				pmlmepriv->roam_scan_count = 0;
				pmlmepriv->roam_freeze_rssi = 0;
			} else {
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

			if (pbtm->candidate) {
				_rtw_memcpy(pnb->roam_target_addr,
					(u8 *)&pbtm->candidate->network.MacAddress,
					ETH_ALEN);

				/* for ReAssocreq check in rtw_wnm_btm_reassoc_req() */
				/* TODO Do NOT use global variable */
				_rtw_memcpy(mlmepriv_pnb->roam_target_addr,
					(u8 *)&pbtm->candidate->network.MacAddress,
					ETH_ALEN);
			} else {
				FSM_INFO(pbtm, "Pick a candidate from scan queue\n");

				if (rtw_mbo_wifi_logo_test(a) || rtw_chk_roam_flags(a, RTW_ROAM_BTM_IGNORE_DELTA))
					pbtm->candidate = rtw_wnm_btm_candidate_select(a, pnb);
				else
					pbtm->candidate = rtw_select_roaming_candidate(pmlmepriv);
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
	case FSM_EV_ABORT:
	case FSM_EV_DISCONNECT_START:
		pbtm->unexpect_terminated = _TRUE;
		rtw_scan_abort_no_wait(a);
		/* need to wait scan done */
		break;
	case BTM_EV_scan_timeout:
		rtw_scan_abort_no_wait(a);
		/* may not expect scan done event */
		rtw_fsm_st_goto(pbtm, BTM_ST_END);
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(pbtm);
		rtw_fsm_cancel_alarm_ext(pbtm, 1);
		rtw_fsm_cancel_alarm_ext(pbtm, 2);

		if (pbtm->candidate) {
			bss = &pbtm->candidate->network;
			FSM_INFO(pbtm, "rtw_select_roam_candidate: %s("MAC_FMT", ch:%u-%s) rssi:%d\n",
				pbtm->candidate->network.Ssid.Ssid,
				MAC_ARG(bss->MacAddress),
				bss->Configuration.DSConfig,
				rtw_band_str(bss->Configuration.Band),
				(int)bss->PhyInfo.rssi);
		} else {
			FSM_INFO(pbtm, "rtw_select_roam_candidate: NULL\n");
		}

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
	case FSM_EV_DISCONNECT_START:
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
		FSM_INFO(pbtm, "roam from %s("MAC_FMT", ch:%d-%s) rssi:%d\n",
			pmlmepriv->dev_cur_network.network.Ssid.Ssid,
			MAC_ARG(pmlmepriv->dev_cur_network.network.MacAddress),
			pmlmepriv->dev_cur_network.network.Configuration.DSConfig,
			rtw_band_str(pmlmepriv->dev_cur_network.network.Configuration.Band),
			a->recvinfo.bcn_rssi);

		_rtw_memcpy(&pmlmepriv->roam_from_addr,
			pmlmepriv->dev_cur_network.network.MacAddress, ETH_ALEN);

#if 0 /* No need to wait AP disconnects us */
		if (pbtm->pnb.btm_cache.req_mode & DISASSOC_IMMINENT) {
			/* indicates that the STA is to be disassociated
			 * from the current AP */
			rtw_fsm_set_alarm(pbtm, BTM_DISASSOC_WAIT, BTM_EV_disassoc_timeout);
			return _SUCCESS;
		}
		fallthrough;
	case BTM_EV_disassoc_timeout:
#endif
		if (rtw_ft_otd_roam(a)) {
			pmlmeinfo->disconnect_code = DISCONNECTION_BY_DRIVER_DUE_TO_FT;
		} else {
			rtw_set_to_roam(a, a->registrypriv.max_roaming_times);
			pmlmeinfo->disconnect_code = DISCONNECTION_BY_DRIVER_DUE_TO_ROAMING;
		}

		rtw_fsm_set_alarm(pbtm, BTM_DISASSOC_WAIT, BTM_EV_disassoc_timeout);

		receive_disconnect(a, pmlmepriv->dev_cur_network.network.MacAddress
			, WLAN_REASON_ACTIVE_ROAM, _FALSE);
		break;

	case FSM_EV_DISCONNECT_END:
		rtw_fsm_cancel_alarm(pbtm);
		fallthrough;
	case BTM_EV_disassoc_timeout:
		FSM_INFO(pbtm, "roam to %s("MAC_FMT", ch:%d-%s) rssi:%d\n",
			pbtm->candidate->network.Ssid.Ssid,
			MAC_ARG(pbtm->candidate->network.MacAddress),
			pbtm->candidate->network.Configuration.DSConfig,
			rtw_band_str(pbtm->candidate->network.Configuration.Band),
			pbtm->candidate->network.PhyInfo.rssi);

		_rtw_memcpy(&pmlmepriv->assoc_ssid,
			&pmlmepriv->dev_cur_network.network.Ssid, sizeof(NDIS_802_11_SSID));
		pmlmepriv->assoc_ch = 0;
		pmlmepriv->assoc_band = BAND_MAX;
		pmlmepriv->assoc_by_bssid = _FALSE;

		/* Start to connect */
		if (!rtw_do_join(a)) {
			rtw_fsm_st_goto(pbtm, BTM_ST_END);
			return _SUCCESS;
		}
#if 1
		/* finish when disconnected */
		pbtm->finish = _SUCCESS;
		rtw_fsm_st_goto(pbtm, BTM_ST_END);
#else
		rtw_fsm_set_alarm(pbtm, BTM_CONNECT_WAIT, BTM_EV_connect_timeout);
#endif
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

static u16 btm_new_cid(struct btm_priv *btmpriv)
{
	do {
		btmpriv->id_seq++;
	} while (btmpriv->id_seq == 0);

	return (btmpriv->id_seq << 8);
}

/* For EXTERNAL application to create a btm object
 * return
 * btm_obj: ptr to new btm object
 */
int rtw_btm_new_obj(_adapter *a, struct sta_info *psta,
	struct roam_nb_info *pnb, u16 roam_reason)
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
	else
		cid = btm_new_cid(btmpriv);

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

	if (roam_reason == RTW_ROAM_BTM)
		_rtw_memcpy(&pbtm->pnb, pnb, sizeof(*pnb));
	else /* copy from rrm */
		rtw_rrm_get_nb_rpt(a, &pbtm->pnb.nb_rpt);

	/* activate it immediately */
	rtw_fsm_activate_obj(pbtm);
	pmlmepriv->need_to_roam = _TRUE;
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

void rtw_btm_notify_scan_probe_req(struct btm_obj *pbtm, struct phl_scan_channel *scan_ch)
{
	struct rtw_ieee80211_channel ch;

	ch.hw_value = scan_ch->channel;
	ch.band = scan_ch->band;
	rtw_fsm_gen_msg(pbtm, &ch, sizeof(ch), BTM_EV_scan_probe_req);
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
#if RTW_WNM_DBG
	tb.dbg_level = FSM_LV_DBG;
	tb.evt_level = FSM_LV_DBG;
#else
	tb.dbg_level = FSM_LV_INFO;
	tb.evt_level = FSM_LV_INFO;
#endif
	tb.debug = btm_debug;

	fsm = rtw_fsm_register_fsm(root, "wnm", &tb);
	btmpriv->fsm = fsm;

	if (!fsm)
		return _FAIL;

	return _SUCCESS;
}
#endif /* CONFIG_RTW_FSM_BTM */
