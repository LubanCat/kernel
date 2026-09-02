/******************************************************************************
 *
 * Copyright(c) 2013 - 2017 Realtek Corporation.
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
#ifdef CONFIG_BTC

#include <drv_types.h>

enum rtw_btc_esoc_type rtw_btc_update_ext_soc_type(u8 esoc_type)
{
	enum rtw_btc_esoc_type phl_esoc_type = BTC_EXT_SOC_NONE;

	switch (esoc_type) {
	case 0:
		phl_esoc_type = BTC_EXT_SOC_NONE;
		break;
	case 1:
		phl_esoc_type = BTC_EXT_SOC_8761;
		break;
	case 2:
		phl_esoc_type = BTC_EXT_SOC_8771;
		break;
	case 3:
		phl_esoc_type = BTC_EXT_SOC_SILAB_MG21;
		break;
	case 4:
		phl_esoc_type = BTC_EXT_SOC_NORDI_NRF52840;
		break;
	default:
		RTW_WARN("%s: Unsupported esoc_type %d\n", __func__, esoc_type);
		break;
	}

	return phl_esoc_type;
}

#ifdef CONFIG_BTC_TRXSS_CHG
static u8 _rtw_btc_trxss_chg_hdl_sta(_adapter *padapter, u8 ss_chg_to, u8 hdl_tx, u8 hdl_rx)
{
	struct _ADAPTER_LINK *alink = NULL;
	struct sta_info *psta = NULL;
	enum wlan_mode wmode = WLAN_MD_INVALID;
	u8 final_ss = 0;

	psta = rtw_get_stainfo(&padapter->stapriv, get_bssid(&padapter->mlmepriv));
	if (psta == NULL) {
		RTW_ERR("%s: psta = NULL!\n", __func__);
		return _FAIL;
	}

	alink = GET_PRIMARY_LINK(padapter);
	wmode = psta->phl_sta->wmode;

	if ((wmode & WLAN_MD_11AX) || (wmode & WLAN_MD_11AC)) {
		final_ss = rtw_backup_and_get_final_ss(padapter, psta, ss_chg_to);
		if (wmode & WLAN_MD_11AX)
			rtw_he_om_ctrl_trx_ss(padapter, alink, psta, final_ss, hdl_tx);
		else if (wmode & WLAN_MD_11AC)
			rtw_vht_op_mode_ctrl_rx_nss(padapter, alink, psta, final_ss, hdl_tx);
	} else if (wmode & WLAN_MD_11N) {
		if (ss_chg_to)
			rtw_ssmps_enter(padapter, psta);
		else
			rtw_ssmps_leave(padapter, psta);
	}

	return _SUCCESS;
}

static u8 _rtw_btc_trxss_chg_hdl_go(_adapter *padapter, u8 ss_chg_to, u8 hdl_tx, u8 hdl_rx)
{
	struct sta_priv *stapriv = NULL;
	struct sta_info *sta = NULL;
	u32 i, stainfo_offset;
	_list *plist, *phead;
	u8 chk_num = 0;
	u8 chk_list[NUM_STA];
	struct _ADAPTER_LINK *a_link = GET_PRIMARY_LINK(padapter);
	struct link_mlme_priv *mlmepriv = &(a_link->mlmepriv);
	struct link_mlme_ext_priv *mlmeext = &(a_link->mlmeextpriv);
	struct link_mlme_ext_info *mlmeinfo = &(mlmeext->mlmext_info);
	WLAN_BSSID_EX *network = &(mlmeinfo->network);
	struct HT_caps_element *ht_caps;
	u8 *p, *ie = network->IEs;
	u32 len = 0;

	if (!mlmepriv->htpriv.ht_option)
		return _FAIL;

	if (!mlmeinfo->HT_caps_enable)
		return _FAIL;

	stapriv = &padapter->stapriv;
	_rtw_spinlock_bh(&stapriv->asoc_list_lock);

	phead = &stapriv->asoc_list;
	plist = get_next(phead);

	while ((rtw_end_of_queue_search(phead, plist)) == _FALSE) {
		sta = LIST_CONTAINOR(plist, struct sta_info, asoc_list);
		plist = get_next(plist);

		stainfo_offset = rtw_stainfo_offset(stapriv, sta);
		if (stainfo_offset_valid(stainfo_offset))
			chk_list[chk_num++] = stainfo_offset;
		continue;
	}

	_rtw_spinunlock_bh(&stapriv->asoc_list_lock);

	for (i = 0; i < chk_num; i++) {
		sta = rtw_get_stainfo_by_offset(stapriv, chk_list[i]);
		if (ss_chg_to)
			rtw_ssmps_enter(padapter, sta);
		else
			rtw_ssmps_leave(padapter, sta);
	}

	p = rtw_get_ie(ie + _BEACON_IE_OFFSET_, _HT_CAPABILITY_IE_, &len, (network->IELength - _BEACON_IE_OFFSET_));
	if (p && len > 0) {
		ht_caps = (struct HT_caps_element *)(p + 2);
		RTW_INFO("%s: orig ht caps info = 0x%02x\n", __func__, ht_caps->u.HT_cap_element.HT_caps_info);

		if (ss_chg_to)
			SET_HT_CAP_ELE_SM_PS(&(ht_caps->u.HT_cap_element.HT_caps_info), SM_PS_STATIC);
		else
			SET_HT_CAP_ELE_SM_PS(&(ht_caps->u.HT_cap_element.HT_caps_info), SM_PS_DISABLE);

		RTW_INFO("%s: modify ht caps info = 0x%02x\n", __func__, ht_caps->u.HT_cap_element.HT_caps_info);
		rtw_update_beacon(padapter, a_link, _HT_CAPABILITY_IE_, NULL, _TRUE, RTW_CMDF_DIRECTLY);
	}

	return _SUCCESS;
}

static u8 _rtw_btc_trxss_chg_hdl_ap(_adapter *padapter, u8 ss_chg_to, u8 hdl_tx, u8 hdl_rx)
{
	/* to be implemented */

	return _SUCCESS;
}

u8 rtw_btc_trxss_chg_hdl(struct dvobj_priv *dvobj, struct phl_msg *msg, u16 evt_id)
{
	_adapter *padapter = dvobj_get_primary_adapter(dvobj);
	_adapter *iface = NULL;
	enum rtw_core_btc_cmd_id btc_cmd_id = RTW_CORE_BTC_CMD_MAX;
	u8 i = 0;
	u8 ret = _FAIL;

	if (!rtw_hw_is_mimo_support(padapter)) {
		RTW_WARN("%s: skip since mimo is not supported!\n", __func__);
		return _FAIL;
	}

#ifdef CONFIG_DBCC_SUPPORT
	if (rtw_phl_mr_is_db(dvobj->phl)) {
		RTW_WARN("%s: skip since dbcc is running!\n", __func__);
		return _FAIL;
	}
#endif

	if (!(GET_DEV_BTC_CAP(GET_PHL_COM(dvobj)).btc_deg_wifi_cap & BTC_DRG_WIFI_CAP_TRX1SS)) {
		RTW_WARN("%s: btc trxss change isn't enabled!\n", __func__);
		return _FAIL;
	}

	switch (evt_id) {
	case MSG_EVT_ANN_RX1SS:
		btc_cmd_id = RTW_CORE_BTC_CMD_TRXSS_LMT;
		break;
	case MSG_EVT_ANN_RX_MAXSS:
		btc_cmd_id = RTW_CORE_BTC_CMD_TRXSS_NO_LMT;
		break;
	default:
		RTW_ERR("%s: unsupported evt_id %d!\n", __func__, evt_id);
		return _FAIL;
	}

	RTW_INFO("%s: evt_id = %d and macid_map[0] = 0x%08X, macid_map[1] = 0x%08X!\n",
		 __func__, evt_id, (u32)msg->rsvd[0].value, (u32)msg->rsvd[1].value);

	ret = rtw_core_btc_cmd(padapter, btc_cmd_id, 0);

	return ret;
}
#endif /* CONFIG_BTC_TRXSS_CHG */

u8 rtw_core_btc_hdl(_adapter *padapter, enum rtw_core_btc_cmd_id btc_cmd_id)
{
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);
	_adapter *iface = NULL;
	u8 i = 0;
	u8 ss_chg_to = 0;
	u8 ret = _SUCCESS;

	switch (btc_cmd_id) {
#ifdef CONFIG_BTC_TRXSS_CHG
	case RTW_CORE_BTC_CMD_TRXSS_LMT:
	case RTW_CORE_BTC_CMD_TRXSS_NO_LMT:
		ss_chg_to = (btc_cmd_id == RTW_CORE_BTC_CMD_TRXSS_LMT) ? 1 : 0;

		for (i = 0; i < dvobj->iface_nums; i++) {
			iface = dvobj->padapters[i];
			if (iface == NULL)
				continue;
			if (!rtw_is_adapter_up(iface))
				continue;

			if (is_client_associated_to_ap(iface)) {
				RTW_INFO("%s: handle "ADPT_FMT" in sta mode!\n", __func__, ADPT_ARG(iface));
				ret = _rtw_btc_trxss_chg_hdl_sta(iface, ss_chg_to, _TRUE, _TRUE);
			} else if (MLME_IS_GO(iface)) {
				RTW_INFO("%s: handle "ADPT_FMT" in P2P-GO mode!\n", __func__, ADPT_ARG(iface));
				ret = _rtw_btc_trxss_chg_hdl_go(iface, ss_chg_to, _TRUE, _TRUE);
			} else if (MLME_IS_AP(iface)) {
				RTW_INFO("%s: handle "ADPT_FMT" in AP mode!\n", __func__, ADPT_ARG(iface));
				ret = _rtw_btc_trxss_chg_hdl_ap(iface, ss_chg_to, _TRUE, _TRUE);
			}
		}

		break;
#endif
	default:
		break;
	}

	return ret;
}

u8 rtw_core_btc_cmd(_adapter *padapter, enum rtw_core_btc_cmd_id btc_cmd_id, u8 flags)
{
	u8 ret = _SUCCESS;
	struct	cmd_obj	*pcmdobj;
	struct	drvextra_cmd_parm *pcmd_parm;
	struct	cmd_priv *pcmdpriv = &adapter_to_dvobj(padapter)->cmdpriv;
	struct submit_ctx sctx;

	pcmdobj = (struct cmd_obj *)rtw_zmalloc(sizeof(struct cmd_obj));
	if (pcmdobj == NULL) {
		ret = _FAIL;
		goto exit;
	}
	pcmdobj->padapter = padapter;

	pcmd_parm = (struct drvextra_cmd_parm *)rtw_zmalloc(sizeof(struct drvextra_cmd_parm));
	if (pcmd_parm == NULL) {
		rtw_mfree((u8 *)pcmdobj, sizeof(struct cmd_obj));
		ret = _FAIL;
		goto exit;
	}

	pcmd_parm->ec_id = CORE_BTC_CID;
	pcmd_parm->type = (int)btc_cmd_id;

	init_h2fwcmd_w_parm_no_rsp(pcmdobj, pcmd_parm, CMD_SET_DRV_EXTRA);

	if (flags & RTW_CMDF_WAIT_ACK) {
		pcmdobj->sctx = &sctx;
		rtw_sctx_init(&sctx, 3000);
	}

	ret = rtw_enqueue_cmd(pcmdpriv, pcmdobj);

	if (ret == _SUCCESS && (flags & RTW_CMDF_WAIT_ACK)) {
		rtw_sctx_wait(&sctx, __func__);
		_rtw_mutex_lock_interruptible(&pcmdpriv->sctx_mutex);
		if (sctx.status == RTW_SCTX_SUBMITTED)
			pcmdobj->sctx = NULL;
		_rtw_mutex_unlock(&pcmdpriv->sctx_mutex);
		if (sctx.status != RTW_SCTX_DONE_SUCCESS)
			ret = _FAIL;
	}

exit:
	return ret;
}

#endif /* CONFIG_BTC */

