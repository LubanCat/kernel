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
#include <drv_types.h>

#ifdef CONFIG_CFG80211_SME_OFFLOAD


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
int cfg80211_rtw_probe_client(struct wiphy *wiphy,
			      struct net_device *ndev,
			      const u8 *peer,
			      u64 *cookie)
{
	RTW_INFO(FUNC_NDEV_FMT"\n", FUNC_NDEV_ARG(ndev));
	return 0;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
static size_t
rtw_sme_alloc_auth_data(const struct cfg80211_auth_request *req,
			struct rtw_sme_auth_data **pp)
{
	u16 auth_alg;
	size_t data_len;
	struct rtw_sme_auth_data *data;
	const u8 *auth_data;
	size_t auth_data_len;
	struct ieee80211_channel *chan;
	u8 tx_chan;
	u8 tx_band;

	switch (req->auth_type) {
	case NL80211_AUTHTYPE_OPEN_SYSTEM:
		auth_alg = WLAN_AUTH_OPEN;
		break;
	case NL80211_AUTHTYPE_SHARED_KEY:
		auth_alg = WLAN_AUTH_SHARED_KEY;
		break;
	case NL80211_AUTHTYPE_SAE:
		auth_alg = WLAN_AUTH_SAE;
		break;
	case NL80211_AUTHTYPE_FT:
		auth_alg = WLAN_AUTH_FT;
		break;
	default:
		/*
		 * TODO: handle the following:
		 * NL80211_AUTHTYPE_NETWORK_EAP
		 */
		return -1;
	}

	*pp = NULL;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	auth_data = req->auth_data;
	auth_data_len = req->auth_data_len;
#else
	auth_data = req->sae_data;
	auth_data_len = req->sae_data_len;
#endif

	chan = req->bss->channel;
	tx_band = nl80211_band_to_rtw_band(chan->band);
	tx_chan = (u8)ieee80211_frequency_to_channel(chan->center_freq);

	data_len = sizeof(*data) + auth_data_len + req->ie_len;
	data = (struct rtw_sme_auth_data *)rtw_zmalloc(data_len);
	if (data == NULL)
		return 0;

	_rtw_memcpy(data->bssid, req->bss->bssid, ETH_ALEN);
	data->tx_band = tx_band;
	data->tx_chan = tx_chan;

	RTW_INFO("%s, tx channel=%d\n", __func__, data->tx_chan);

	if (auth_data_len >= 4) {
		if (req->auth_type == NL80211_AUTHTYPE_SAE) {
			__le16 *pos = (__le16 *)auth_data;

			data->sae_trans = le16_to_cpu(pos[0]);
			data->sae_status = le16_to_cpu(pos[1]);
		}

		_rtw_memcpy(data->data, auth_data + 4, auth_data_len - 4);
		data->data_len += auth_data_len - 4;
	}

	data->expect_trans = 2;
	if (req->auth_type == NL80211_AUTHTYPE_SAE) {
		data->expect_trans = data->sae_trans;
	}

	if (req->ie && req->ie_len) {
		_rtw_memcpy(&data->data[data->data_len], req->ie, req->ie_len);
		data->data_len += req->ie_len;
	}

	if (req->key && req->key_len) {
		data->key_len = req->key_len;
		data->key_index = req->key_idx;
		_rtw_memcpy(data->key, req->key, req->key_len);
	}

	data->auth_alg = auth_alg;

	*pp = data;

	return data_len;
}

static size_t
rtw_cfg80211_build_auth_frame(struct _ADAPTER *padapter, u16 transaction,
			      u16 auth_alg, u16 status, const u8 *extra,
			      size_t extra_len, const u8 *da, const u8 *bssid,
			      const u8 *key, u8 key_len, u8 key_idx, u8 *buf,
			      size_t buflen)
{
	int wep_enc = 0;
	u8 *pos, *start;
	size_t frame_len;
	struct rtw_ieee80211_hdr_3addr *hdr;
	struct mlme_ext_info *mlme_ext = &padapter->mlmeextpriv.mlmext_info;
	u8 *iv, *icv;

	RTW_INFO("%s(): building auth, trans=%u, alg=%u, status=%u, extra_len=%zd\n",
		 __func__, transaction, auth_alg, status, extra_len);

	if (auth_alg == WLAN_AUTH_SHARED_KEY && transaction == 3)
		wep_enc = 1;

	_rtw_memset(buf, 0, buflen);

	pos = start = buf;
	hdr = (struct rtw_ieee80211_hdr_3addr *)pos;
	set_frame_sub_type(hdr, WIFI_AUTH); /* type and subtype */
	_rtw_memcpy(hdr->addr1, da, ETH_ALEN);
	_rtw_memcpy(hdr->addr2, adapter_mac_addr(padapter), ETH_ALEN);
	_rtw_memcpy(hdr->addr3, bssid, ETH_ALEN);
	/* seq is delt with later in update_mgntframe_attrib() */
	pos += sizeof(struct rtw_ieee80211_hdr_3addr);

	if (wep_enc) {
		u8 *tmp;

		RTW_INFO("%s(): iv=%u, key_idx=%u\n",
			 __FUNCTION__, mlme_ext->iv, key_idx);

		SetPrivacy(&hdr->frame_ctl);

		/* prepend iv */
		tmp = pos;

		*tmp++ = (mlme_ext->iv >> 16) & 0xff;
		*tmp++ = (mlme_ext->iv >> 8) & 0xff;
		*tmp++ = mlme_ext->iv & 0xff;
		*tmp++ = key_idx << 6;

		mlme_ext->iv++;

		iv = pos;
		pos += 4;
	}

	*(u16 *)pos = cpu_to_le16(auth_alg);
	pos += 2;

	*(u16 *)pos = cpu_to_le16(transaction);
	pos += 2;

	*(u16 *)pos = cpu_to_le16(status);
	pos += 2;

	if (extra) {
		_rtw_memcpy(pos, extra, extra_len);
		pos += extra_len;
	}

	/* wep encrypt, ref issue_auth() */
	if (wep_enc) {
		u8 arc4key[3 + WEP_KEY_LEN];
		struct arc4context arc4ctx;
		u8 *data;
		size_t data_len;
		u32 crc;

		/* construct arc4 key */
		_rtw_memcpy(arc4key, iv, 3);
		_rtw_memcpy(arc4key + 3, key, key_len);

		/* append crc */
		data = iv + 4;
		data_len = (size_t)(pos - data);
		crc = getcrc32(data, (sint)data_len);
		*(u32 *)pos = cpu_to_le32(crc);

		icv = pos;
		pos += 4;

		/* encrypt */
		arcfour_init(&arc4ctx, arc4key, 3 + key_len);
		arcfour_encrypt(&arc4ctx, data, data, data_len);
		arcfour_encrypt(&arc4ctx, icv, icv, 4);
	}

	frame_len = (size_t)(pos - start);

	RTW_DBG_DUMP("cfg80211_rtw_build_auth(): frame:\n", buf, frame_len);

	return frame_len;
}

static int rtw_cfg80211_send_auth(struct _ADAPTER *padapter)
{
	size_t tx_buf_len;
	size_t tx_param_len;
	struct mgnt_tx_parm *tx_param;
	u8 *tx_frame_buf;
	struct mlme_priv *mlme_priv = &padapter->mlmepriv;
	u16 trans = 1;
	u16 status = 0;
	int tx_res;

	RTW_INFO("%s\n", __FUNCTION__);

	/*
	 * evaluate frame len, can be a rough estimation
	 * hdr + alg + auth_data_len + ie_len
	 * TODO: consider cases other than SAE
	 */
	tx_buf_len = sizeof(struct rtw_ieee80211_hdr_3addr) + 1024;
	tx_param_len = sizeof(*tx_param) + tx_buf_len;
	tx_param = (struct mgnt_tx_parm *)rtw_zmalloc(tx_param_len);
	if (tx_param == NULL)
		goto error;

	tx_frame_buf = (u8 *)tx_param + sizeof(*tx_param);
	_rtw_memset(tx_frame_buf, 0, tx_buf_len);

	tx_param->no_cck = 0;
	tx_param->buf = tx_frame_buf;
	tx_param->len = 0;	/* to be filled when frame is constructed */
	tx_param->wait_ack = 1;

	_rtw_spinlock_bh(&mlme_priv->lock);
	if (mlme_priv->sme_auth_data) {
		struct rtw_sme_auth_data *auth_data;

		auth_data = mlme_priv->sme_auth_data;
		tx_param->tx_band = auth_data->tx_band;
		tx_param->tx_ch = auth_data->tx_chan;

		if (auth_data->auth_alg == WLAN_AUTH_SAE) {
			trans = auth_data->sae_trans;
			status = auth_data->sae_status;
		}

		tx_param->len =
			rtw_cfg80211_build_auth_frame(padapter, trans,
						      auth_data->auth_alg,
						      status,
						      auth_data->data,
						      auth_data->data_len,
						      auth_data->bssid,
						      auth_data->bssid,
						      NULL,
						      0,
						      0,
						      tx_frame_buf,
						      tx_buf_len);
	} else {
		RTW_INFO("%s(): auth data has been freed, not able to build the frame\n",
				 __FUNCTION__);
		_rtw_spinunlock_bh(&mlme_priv->lock);
		goto error;
	}
	_rtw_spinunlock_bh(&mlme_priv->lock);

	if (tx_param->len <= 0)
		goto error;

	RTW_INFO_DUMP("tx auth data", tx_param->buf, tx_param->len);
	tx_res = rtw_mgnt_tx_handler(padapter, (u8 *)tx_param);
	if (tx_res < 0) {
		RTW_ERR("%s(): failed to issue tx param\n", __FUNCTION__);
		goto error;
	}
	if (tx_param)
		rtw_mfree((u8 *)tx_param, tx_param_len);

	return 0;

error:
	if (tx_param)
		rtw_mfree((u8 *)tx_param, tx_param_len);
	return 0;
}

int rtw_sme_send_auth_challenge(_adapter *padapter, u8 *frame,
				size_t frame_len,
				struct rtw_sme_auth_data *data)
{
	const u8 *challenge;
	sint challenge_len;
	struct xmit_frame *xmit;
	struct pkt_attrib *attrib;
	struct mlme_ext_priv *mlme_ext = &padapter->mlmeextpriv;
	u8 *tx_frame_buf;
	size_t tx_buf_len;
	struct rtw_ieee80211_hdr_3addr *hdr;
	/* ToDo CONFIG_RTW_MLD: [currently primary link only] */
	struct _ADAPTER_LINK *padapter_link = GET_PRIMARY_LINK(padapter);

	/* parse the challenge text */
	challenge = rtw_get_ie(frame + WLAN_HDR_A3_LEN + _AUTH_IE_OFFSET_,
			       _CHLGETXT_IE_, &challenge_len,
			       frame_len - WLAN_HDR_A3_LEN - _AUTH_IE_OFFSET_);
	if (!challenge)
		return -1;

	/* prepare for tx */
	if (alink_is_tx_blocked_by_ch_waiting(padapter_link))
		return -1;

	xmit = alloc_mgtxmitframe(&padapter->xmitpriv);
	if (xmit == NULL)
		return -1;

	attrib = &xmit->attrib;
	update_mgntframe_attrib(padapter, padapter_link, attrib);

	_rtw_memset(xmit->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);
	tx_frame_buf = (u8 *)xmit->buf_addr + TXDESC_OFFSET;
	tx_buf_len = 1024;			/* TODO */

	/* construct the frame */
	data->expect_trans = 4;

	attrib->pktlen = rtw_cfg80211_build_auth_frame(padapter,
						       3,
						       data->auth_alg,
						       0,
						       challenge,
						       challenge_len + 2,
						       data->bssid,
						       data->bssid,
						       data->key,
						       data->key_len,
						       data->key_index,
						       tx_frame_buf,
						       tx_buf_len);
	attrib->last_txcmdsz = attrib->pktlen;

	/* assign seq number */
	hdr = (struct rtw_ieee80211_hdr_3addr *)tx_frame_buf;
	SetSeqNum(hdr, mlme_ext->mgnt_seq);
	mlme_ext->mgnt_seq++;

	/* send the frame */
	dump_mgntframe(padapter, xmit);

	return 0;
}

int cfg80211_rtw_auth(struct wiphy *wiphy, struct net_device *ndev,
		      struct cfg80211_auth_request *req)
{
	struct rtw_sme_auth_data *auth_data = NULL;
	size_t auth_data_len;
	_adapter *padapter;
	struct mlme_priv *mlme_priv;
	struct mlme_ext_priv *pmlmeext;
	struct mlme_ext_info *pmlmeinfo;
	int attempt = 0;
	systime start_time = rtw_get_current_time();
	int tx_res;
	struct beacon_keys *cur_beacon;
	struct _ADAPTER_LINK *padapter_link;
	/* ToDo CONFIG_RTW_MLD: [currently primary link only] */
	struct link_mlme_priv *lmlmepriv;
#ifdef CONFIG_RTW_MBO
	unsigned short result;
#endif

	RTW_INFO("%s(): auth_type=%d\n", __FUNCTION__, req->auth_type);

	/* find adapter */
	padapter = (_adapter *)rtw_netdev_priv(ndev);
	padapter_link = GET_PRIMARY_LINK(padapter);
	lmlmepriv = &padapter_link->mlmepriv;

	mlme_priv = &padapter->mlmepriv;
	cur_beacon = &lmlmepriv->cur_beacon_keys;


	pmlmeext = &padapter->mlmeextpriv;
	pmlmeinfo = &(pmlmeext->mlmext_info);

#ifdef CONFIG_RTW_MBO
	if (((req->auth_type == NL80211_AUTHTYPE_FT) ||
	     (mlme_priv->sme_do_btm_roam == 1)) &&
	    (pmlmeinfo->state & WIFI_FW_ASSOC_SUCCESS)) {
		result = (u16)(0);
		rtw_disconnect(wiphy, ndev, result);
	}
	mlme_priv->sme_do_btm_roam = (u8)0;
#endif

	pmlmeinfo->state &= (~WIFI_FW_AUTH_NULL);
	pmlmeinfo->state |= WIFI_FW_AUTH_STATE;
	_rtw_memset(cur_beacon, 0, sizeof(struct beacon_keys));

	/* alloc and build auth data, will be freed when done or timeout  */
	auth_data_len = rtw_sme_alloc_auth_data(req, &auth_data);
	if (auth_data_len <= 0) {
		return -EFAULT;
	}

#ifdef RTW_WKARD_LINUX_CMD_WKARD
	RTW_INFO("debug: smeofld wl cmd +\n");
	rtw_smeofld_wk_cmd(padapter);
	RTW_INFO("debug: smeofld wl cmd -\n");
#endif

	/* pre-authenticate */
	_rtw_spinlock_bh(&mlme_priv->lock);

	if (mlme_priv->sme_auth_data) {
		/* there's an ongoing auth req, finish it */
		RTW_INFO("%s(): free existing auth\n", __FUNCTION__);
		_cancel_timer_ex(&mlme_priv->sme_auth_timer);
		cfg80211_auth_timeout(ndev, mlme_priv->sme_auth_data->bssid);
		rtw_mfree((u8 *)mlme_priv->sme_auth_data,
			  mlme_priv->sme_auth_data_len);
		mlme_priv->sme_auth_data = NULL;
	}

	mlme_priv->sme_auth_data_len = auth_data_len;
	mlme_priv->sme_auth_data = auth_data;

	mlme_priv->smeofld_sta = 1;

	_rtw_spinunlock_bh(&mlme_priv->lock);

	/* Auth timeout will be handled in rtw_cfg80211_auth_timeout_handler */
	_set_timer(&mlme_priv->sme_auth_timer, REAUTH_TO);

	/* issue_sae_auth */
	while (1) {
		attempt++;
		rtw_mi_set_scan_deny(padapter, 1000);
		rtw_mi_scan_abort(padapter, _TRUE);

		tx_res = rtw_cfg80211_send_auth(padapter);
		if (tx_res == 0) {
			break;
		} else if (tx_res < 0) {
			if (3 <= attempt)
				break;			/* no more retry */
			continue;			/* try again */
		}
	}

	RTW_INFO(FUNC_ADPT_FMT" %s (%d/%d) in %d ms\n", FUNC_ADPT_ARG(padapter),
		 tx_res == 0 ? "OK" : "FAIL", attempt, 3,
		 rtw_get_passing_time_ms(start_time));

	return 0;
}

static void sme_chk_set_union_ch(_adapter *padapter,
				 struct _ADAPTER_LINK *padapter_link)
{
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);
	struct rtw_chan_def u_chdef = {0};

#ifndef CONFIG_MCC_MODE
	if (rtw_mi_check_status(padapter, MI_LINKED)) {
		if ((RTW_PHL_STATUS_SUCCESS == rtw_phl_mr_get_chandef(
						dvobj->phl, padapter->phl_role,
						padapter_link->wrlink, &u_chdef)) &&
		    (u_chdef.chan != 0)) {
			rtw_set_chbw_cmd(padapter,
					 padapter_link,
					 &u_chdef,
					 0,
					 RFK_TYPE_FORCE_NOT_DO);
		} else {
			RTW_ERR("%s get union chandef failed\n",  __func__);
		}
	}
#endif

}

bool rtw_sme_onauth_client(_adapter *padapter,
			   struct _ADAPTER_LINK *padapter_link,
			   u8 *pframe,
			   uint pkt_len,
			   unsigned int algthm,
			   unsigned int seq,
			   unsigned int status)
{
	struct mlme_priv *pmlmepriv = &(padapter->mlmepriv);
	int done = 0;

	RTW_INFO("%s(): rx auth, alg=%u, seq=%u, status=%u\n",
		 __FUNCTION__, algthm, seq, status);

	_rtw_spinlock_bh(&pmlmepriv->lock);

	if (pmlmepriv->sme_auth_data == NULL ||
	    seq != pmlmepriv->sme_auth_data->expect_trans ||
	    algthm != pmlmepriv->sme_auth_data->auth_alg ||
	    !_rtw_memcmp(pmlmepriv->sme_auth_data->bssid, get_ta(pframe),
			 ETH_ALEN)) {
		_rtw_spinunlock_bh(&pmlmepriv->lock);
		return _FAIL;
	}

	if (algthm == WLAN_AUTH_SHARED_KEY && seq == 2) {
		rtw_sme_send_auth_challenge(padapter, pframe, (size_t)pkt_len,
					    pmlmepriv->sme_auth_data);
	}

	if ((algthm == WLAN_AUTH_OPEN && seq == 2) ||
	    (algthm == WLAN_AUTH_SHARED_KEY && seq == 4) ||
	    (algthm == WLAN_AUTH_SAE && (seq == 1 || seq == 2)) ||
	    (algthm == WLAN_AUTH_FT && seq == 2)) {
		RTW_INFO("%s(): auth done, free auth data, alg=%u, seq=%u\n",
			 __FUNCTION__, algthm, seq);
		_cancel_timer_ex(&pmlmepriv->sme_auth_timer);
		rtw_mfree((u8 *)pmlmepriv->sme_auth_data,
			  pmlmepriv->sme_auth_data_len);
		pmlmepriv->sme_auth_data = NULL;
		done = 1;
	}
	_rtw_spinunlock_bh(&pmlmepriv->lock);

	if(done) {
		RTW_INFO("Inform rx auth on ch=%u, pkt_len=%u\n",
			 rtw_get_oper_ch(padapter, padapter_link), pkt_len);
		cfg80211_rx_mlme_mgmt(padapter->pnetdev, pframe, pkt_len);
		if(!(algthm == WLAN_AUTH_SAE && seq == 1))
			sme_chk_set_union_ch(padapter, padapter_link);
	}
	return _SUCCESS;
}

static int rtw_sme_pre_assoc(struct wiphy *wiphy, struct net_device *ndev,
			     struct cfg80211_assoc_request *req)
{
	_adapter *padapter = (_adapter *)rtw_netdev_priv(ndev);
	struct mlme_priv *mlme_priv = &padapter->mlmepriv;
	struct mlme_ext_priv *mlme_ext = &padapter->mlmeextpriv;
	struct mlme_ext_info *mlme_info = &mlme_ext->mlmext_info;
	struct cfg80211_connect_params *conn_param;
	size_t ssid_len;
	const u8 *ssid, *pos, *end;

	conn_param =
		(struct cfg80211_connect_params *)rtw_zmalloc(sizeof(*conn_param));
	if (conn_param == NULL)
		return -1;

	/* prepare for translate */
	ssid = NULL;
	ssid_len = 0;
	if (req->bss->proberesp_ies && req->bss->proberesp_ies->len) {
		pos = req->bss->proberesp_ies->data;
		end = pos + req->bss->proberesp_ies->len;
		while (pos < end) {
			if (pos[0] == WLAN_EID_SSID) {
				ssid = pos + 2;
				ssid_len = pos[1];
				break;
			}
			pos += 2 + pos[1];
		}
	}
	if (ssid == NULL && req->bss->beacon_ies && req->bss->beacon_ies->len) {
		pos = req->bss->beacon_ies->data;
		end = pos + req->bss->beacon_ies->len;
		while (pos < end) {
			if (pos[0] == WLAN_EID_SSID) {
				ssid = pos + 2;
				ssid_len = pos[1];
				break;
			}
			pos += 2 + pos[1];
		}
	}
	if (ssid == NULL) {
		RTW_INFO("%s(): no ssid found\n", __func__);
		if(conn_param)
			rtw_mfree((u8 *)conn_param, sizeof(*conn_param));
		return -1;
	}

	RTW_INFO_DUMP("rtw_sme_pre_assoc(): parsed ssid:" , ssid, ssid_len);
	RTW_INFO("%s(): wpa versions=%u\n", __func__, req->crypto.wpa_versions);

	/* translate */
	conn_param->channel = req->bss->channel;
	conn_param->bssid = req->bss->bssid;
	conn_param->ssid = ssid;
	conn_param->ssid_len = ssid_len;
	conn_param->auth_type = 0;	/* don't care */
	conn_param->ie = req->ie;
	conn_param->ie_len = req->ie_len;
	conn_param->privacy = 0;	/* don't care */
	conn_param->mfp = NL80211_MFP_NO; /* don't care */
	conn_param->crypto = req->crypto;
	conn_param->key = NULL;
	conn_param->key_len = 0;
	conn_param->key_idx = 0;
	conn_param->flags = 0;		/* don't care */
	conn_param->bg_scan_period = 0; /* don't care */
	conn_param->ht_capa = req->ht_capa;
	conn_param->ht_capa_mask = req->ht_capa_mask;
	conn_param->vht_capa = req->vht_capa;
	conn_param->vht_capa_mask = req->vht_capa_mask;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
	conn_param->pbss = 0;		/* don't care */
#endif

	if (conn_param->crypto.n_ciphers_pairwise == 0 &&
	    conn_param->crypto.wpa_versions != 0) {
		RTW_INFO("%s(): target is open, but trying to connect with wpa, set wpa_versions to 0\n", __FUNCTION__);
		conn_param->crypto.wpa_versions = 0;
	}

	/*
	 * Here we are going all the way down to
	 * rtw_select_and_join_from_scanned_queue(). Note that rtw_joinbss_cmd()
	 * can't be called directly here because there are I/O operations there
	 * and doing I/O in the context of this function can cause BUG:
	 * scheduling while atomic.
	 */
	if (rtw_connect(wiphy, ndev, conn_param) < 0) {
		RTW_INFO("%s(): rtw_connect() failed\n", __FUNCTION__);
		rtw_mfree((u8 *)conn_param, sizeof(*conn_param));
		return -1;
	}

	/* invoke the join bss cmd */
	_rtw_spinlock_bh(&mlme_priv->scanned_queue.lock);
	if (rtw_joinbss_cmd(padapter, mlme_priv->sme_assoc_cand) != _SUCCESS) {
		RTW_INFO("%s(): rtw_joinbss_cmd() failed\n", __FUNCTION__);
		_rtw_spinunlock_bh(&mlme_priv->scanned_queue.lock);
		rtw_mfree((u8 *)conn_param, sizeof(*conn_param));
		return -1;
	}
	_rtw_spinunlock_bh(&mlme_priv->scanned_queue.lock);

	/*
	 * Cancel join timer as we handle timeout in
	 * rtw_cfg80211_assoc_timeout_handler()
	 */
	_cancel_timer_ex(&mlme_ext->link_timer);

	if (mlme_priv->smeofld_sta)
		rtw_sctx_init(&mlme_priv->sme_join_cmd_sctx, 10 * 1000);

	/* wait for completion of join_cmd_hdl() */
	rtw_sctx_wait(&mlme_priv->sme_join_cmd_sctx, __func__);
	if (mlme_priv->sme_join_cmd_sctx.status != RTW_SCTX_DONE_SUCCESS) {
		RTW_INFO("%s(): join_cmd_hdl() failed\n", __FUNCTION__);
		rtw_mfree((u8 *)conn_param, sizeof(*conn_param));
		return -1;
	}

	/*
	 * Set state
	 * Note that this has to be done after start_clnt_join() as it sets the
	 * state with an assignment statement and that would clear all other
	 * flags.
	 */
	mlme_info->state &= ~WIFI_FW_AUTH_NULL; /* TODO: this may be useless */
	mlme_info->state |= WIFI_FW_WAIT_BEACON;


	RTW_INFO("%s(): join_cmd_hdl() done\n", __FUNCTION__);

	/* set state, copied from start_clnt_assoc() */
	mlme_info->state &= (~(WIFI_FW_AUTH_NULL | WIFI_FW_AUTH_STATE));
	mlme_info->state |= (WIFI_FW_AUTH_SUCCESS | WIFI_FW_ASSOC_STATE);

	if(conn_param)
		rtw_mfree((u8 *)conn_param, sizeof(*conn_param));

	return 0;
}

static int rtw_sme_send_assoc_req(struct _ADAPTER *padapter,
				  struct cfg80211_assoc_request *req)
{
	size_t tx_buf_len;
	size_t tx_param_len;
	struct mgnt_tx_parm *tx_param;
	u8 *tx_frame_buf;
	struct ieee80211_channel *chan;
	u8 tx_chan;
	u8 tx_band;
	struct _ADAPTER_LINK *padapter_link = GET_PRIMARY_LINK(padapter);
	/* ToDo CONFIG_RTW_MLD: [currently primary link only] */
	int tx_res;
	struct mlme_priv *mlme_priv = &padapter->mlmepriv;
	struct pkt_attrib *pattrib = NULL;
	struct rtw_ieee80211_hdr *pwlanhdr;
	unsigned short *fctrl;
	unsigned char *pframe;

	RTW_INFO("%s\n", __FUNCTION__);

	/*
	 * Allocate buffer for tx_param including tx frame buffer.
	 */
	tx_buf_len =  sizeof(struct rtw_ieee80211_hdr_3addr) + 1024;
	tx_param_len = sizeof(*tx_param) + tx_buf_len;
	tx_param = (struct mgnt_tx_parm *)rtw_zmalloc(tx_param_len);
	if (tx_param == NULL)
		goto error;

	chan = req->bss->channel;
	tx_band = nl80211_band_to_rtw_band(chan->band);
	tx_chan = (u8)ieee80211_frequency_to_channel(chan->center_freq);

	tx_frame_buf = (u8 *)tx_param + sizeof(*tx_param);
	tx_param->tx_band = tx_band;
	tx_param->tx_ch = tx_chan;
	tx_param->no_cck = 0;
	tx_param->buf = tx_frame_buf;
	tx_param->len = 0;	/* to be filled when frame is constructed */
	tx_param->wait_ack = 0;

	_rtw_memset(tx_frame_buf, 0, tx_buf_len);

	pattrib = rtw_zmalloc(sizeof(struct pkt_attrib));
	if (pattrib == NULL)
		goto error;

	pattrib->pktlen = 0;
	update_mgntframe_attrib(padapter, padapter_link, pattrib);

	pframe = tx_frame_buf;

	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;

	_rtw_memcpy(pwlanhdr->addr1, req->bss->bssid, ETH_ALEN);
	_rtw_memcpy(pwlanhdr->addr2, padapter_link->mac_addr, ETH_ALEN);
	_rtw_memcpy(pwlanhdr->addr3, req->bss->bssid, ETH_ALEN);

	SetSeqNum(pwlanhdr, padapter->mlmeextpriv.mgnt_seq);
	padapter->mlmeextpriv.mgnt_seq++;
	if (req->prev_bssid != NULL)
		set_frame_sub_type(pframe, WIFI_REASSOCREQ);
	else
		set_frame_sub_type(pframe, WIFI_ASSOCREQ);

	pframe += sizeof(struct rtw_ieee80211_hdr_3addr);
	pattrib->pktlen = sizeof(struct rtw_ieee80211_hdr_3addr);

	/* build assoc req, seq is delt with later in _cfg80211_rtw_mgmt_tx() */
	tx_param->len =
		rtw_build_assocreq_ies(padapter, padapter_link, pframe, pattrib,
				       req->prev_bssid != NULL, req->ie,
				       req->ie_len);
	if (tx_param->len <= 0) {
		RTW_ERR("%s(): rtw_build_assocreq_ies() failed\n",
		        __FUNCTION__);
		goto error;
	}

	RTW_DBG_DUMP("rtw_build_assocreq_ies(): frame:\n", tx_frame_buf,
		     (int)tx_param->len);

	/*
	 * update assoc_req if build assoc req succeeded, ref
	 * _issue_assocreq()
	 */
	rtw_buf_update(&mlme_priv->assoc_req, &mlme_priv->assoc_req_len,
		       tx_param->buf, tx_param->len);

	tx_res = rtw_mgnt_tx_handler(padapter, (u8 *)tx_param);
	if (tx_res < 0) {
		RTW_ERR("%s(): failed to issue tx param\n", __FUNCTION__);
		goto error;
	}

	if (pattrib)
		rtw_mfree((u8 *)pattrib, sizeof(struct pkt_attrib));

	if (tx_param)
		rtw_mfree((u8 *)tx_param, tx_param_len);

	return 0;

error:
	/*
	 * free the assoc_req recorded if build assoc req failed, ref
	 * _issue_assocreq()
	 */
	rtw_buf_free(&mlme_priv->assoc_req, &mlme_priv->assoc_req_len);

	if (pattrib)
		rtw_mfree((u8 *)pattrib, sizeof(struct pkt_attrib));

	if (tx_param)
		rtw_mfree((u8 *)tx_param, tx_param_len);
	return -1;
}

int cfg80211_rtw_assoc(struct wiphy *wiphy, struct net_device *ndev,
		       struct cfg80211_assoc_request *req)
{
	_adapter *padapter;
	struct mlme_priv *mlme_priv;
	int attempt = 0;
	systime start_time = rtw_get_current_time();
	int tx_res;

	/* find adapter */
	padapter = (_adapter *)rtw_netdev_priv(ndev);
	mlme_priv = &padapter->mlmepriv;

	RTW_INFO("%s(): smeofld_sta=%d\n", __FUNCTION__,
		 mlme_priv->smeofld_sta);

	RTW_DBG_DUMP("BSSID", req->bss->bssid, ETH_ALEN);

	RTW_DBG_DUMP("wpa send ie contents", req->ie, req->ie_len );

	/* pre-association */
	if (rtw_sme_pre_assoc(wiphy, ndev, req) < 0) {
		RTW_ERR("rtw_sme_pre_assoc\n");
		return -EFAULT;
	}

	/* prepare for association */
	_rtw_spinlock_bh(&mlme_priv->lock);
	if (mlme_priv->cfg80211_sme_assoc_bss) {
		/* there's an ongoing assoc req, finish it */
		RTW_INFO("%s(): free existing assoc\n", __FUNCTION__);
		_cancel_timer_ex(&mlme_priv->sme_assoc_timer);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0))
		struct cfg80211_assoc_failure data = {
				.timeout = true,
				.bss[0] = mlme_priv->cfg80211_sme_assoc_bss,
		};
		cfg80211_assoc_failure(ndev, &data);
#else /*LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)*/
		cfg80211_assoc_timeout(ndev, mlme_priv->cfg80211_sme_assoc_bss);
#endif/*LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)*/
		mlme_priv->cfg80211_sme_assoc_bss = NULL;
	}
	mlme_priv->cfg80211_sme_assoc_bss = req->bss;
	_rtw_spinunlock_bh(&mlme_priv->lock);

	/* Assoc timeout is handled in rtw_cfg80211_assoc_timeout_handler() */
	_set_timer(&mlme_priv->sme_assoc_timer, REASSOC_TO);

	/* send assoc req */
	while (1) {
		attempt++;
		rtw_mi_set_scan_deny(padapter, 1000);
		rtw_mi_scan_abort(padapter, _TRUE);

		tx_res = rtw_sme_send_assoc_req(padapter, req);
		if (tx_res == 0) {
			break;
		} else if (tx_res < 0) {
			if (3 <= attempt)
				break;			/* no more retry */
			continue;			/* try again */
		}
	}

	RTW_INFO(FUNC_ADPT_FMT" %s (%d/%d) in %d ms\n", FUNC_ADPT_ARG(padapter),
		 tx_res == 0 ? "OK" : "FAIL", attempt, 3,
		 rtw_get_passing_time_ms(start_time));

	return 0;
}

static void sme_cfg80211_rx_assoc_resp(_adapter *padapter,
				       struct cfg80211_bss *bss,
				       u8 *pframe,
				       uint pkt_len,
				       int uapsd_queues)
{
	struct mlme_priv *priv = &padapter->mlmepriv;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 1, 0))
	unsigned short frame_type, ie_offset = 0;
	u8* areq_ie = NULL;
	u32 areq_ielen = 0;

	frame_type = get_frame_sub_type(priv->assoc_req);
	if (frame_type == WIFI_ASSOCREQ)
		ie_offset = _ASOCREQ_IE_OFFSET_;
	else if (frame_type == WIFI_REASSOCREQ)
		ie_offset = _REASOCREQ_IE_OFFSET_;
	else
		RTW_ERR("invalid in pmlmepriv->assoc_req\n");
	areq_ie = priv->assoc_req + IEEE80211_MGMT_HDR_LEN + ie_offset;
	areq_ielen = priv->assoc_req_len - IEEE80211_MGMT_HDR_LEN - ie_offset;
	if (ie_offset > 0) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0))
		struct cfg80211_rx_assoc_resp resp = {
			.links[0].bss = bss,
			.buf = pframe,
			.len = pkt_len,
			.uapsd_queues = uapsd_queues,
			.req_ies = areq_ie,
			.req_ies_len = areq_ielen
		};
		cfg80211_rx_assoc_resp(padapter->pnetdev, &resp);
#else/*KERVER 5.1.0-6.0.0*/
		cfg80211_rx_assoc_resp(padapter->pnetdev,
				       bss,
				       pframe,
				       pkt_len,
				       uapsd_queues,
				       areq_ie,
				       areq_ielen);
#endif/*KERNEL_VERSION(6, 0, 0))*/
	}
#else/*KERVER < 5.1.0*/
	cfg80211_rx_assoc_resp(padapter->pnetdev, bss, pframe,
			       pkt_len, uapsd_queues);
#endif/*KERNEL_VERSION(5, 1, 0))*/
}

void rtw_sme_onassocrsp(_adapter *padapter, struct _ADAPTER_LINK *padapter_link,
			u8 *pframe, uint pkt_len)
{
	struct link_mlme_ext_priv *lmlmeextpriv = &padapter_link->mlmeextpriv;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct cfg80211_bss *bss;
	struct registry_priv *reg = &padapter->registrypriv;
	int uapsd_queues = 0;


	_cancel_timer_ex(&pmlmepriv->sme_assoc_timer);

#ifdef CONFIG_WMMPS_STA
	if (TEST_FLAG(reg->uapsd_ac_enable, DRV_CFG_UAPSD_VO))
		uapsd_queues |= WMM_IE_UAPSD_VO;
	if (TEST_FLAG(reg->uapsd_ac_enable, DRV_CFG_UAPSD_VI))
		uapsd_queues |= WMM_IE_UAPSD_VI;
	if (TEST_FLAG(reg->uapsd_ac_enable, DRV_CFG_UAPSD_BK))
		uapsd_queues |= WMM_IE_UAPSD_BK;
	if (TEST_FLAG(reg->uapsd_ac_enable, DRV_CFG_UAPSD_BE))
		uapsd_queues |= WMM_IE_UAPSD_BE;
#endif

	_rtw_spinlock_bh(&pmlmepriv->lock);
	bss = pmlmepriv->cfg80211_sme_assoc_bss;
	pmlmepriv->cfg80211_sme_assoc_bss = NULL;
	_rtw_spinunlock_bh(&pmlmepriv->lock);

	RTW_INFO("driver_interface=%d\n", padapter->iface_id);
	RTW_INFO("Inform rx assoc resp on ch=%u, pkt_len=%u\n",
		 lmlmeextpriv->chandef.chan, pkt_len);
	if (bss) {
		sme_cfg80211_rx_assoc_resp(padapter, bss, pframe, pkt_len,
					   uapsd_queues);
	}
	sme_chk_set_union_ch(padapter, padapter_link);
}

static size_t rtw_sme_build_deauth(struct _ADAPTER *padapter,
				   struct cfg80211_deauth_request *req,
				   struct mgnt_tx_parm **pp)
{
	size_t tx_frame_len;
	u8 *tx_frame_buf;
	struct mgnt_tx_parm *tx_param = NULL;
	u8 tx_chan;
	u8 tx_band;
	u8 *pos, *start;
	struct rtw_ieee80211_hdr_3addr *hdr;
	struct _ADAPTER_LINK *iface_link;
	struct link_mlme_ext_priv *mlmeextpriv;

	RTW_INFO("%s\n", __FUNCTION__);

	/*
	 * evaluate frame len, note that this has to be exact
	 * hdr + reason_code + ies
	 */
	tx_frame_len = sizeof(struct rtw_ieee80211_hdr_3addr) + 2 + req->ie_len;

	tx_param = (struct mgnt_tx_parm *)rtw_zmalloc(sizeof(*tx_param) +
						      tx_frame_len);
	if (tx_param == NULL)
		goto error;

	iface_link = GET_PRIMARY_LINK(padapter);
	mlmeextpriv = &iface_link->mlmeextpriv;
	tx_band = mlmeextpriv->chandef.band;
	tx_chan = mlmeextpriv->chandef.chan;
	tx_frame_buf = (u8 *)tx_param + sizeof(*tx_param);

	tx_param->tx_band = tx_band;
	tx_param->tx_ch = tx_chan;
	tx_param->no_cck = 0;
	tx_param->buf = tx_frame_buf;
	tx_param->len = tx_frame_len;
	tx_param->wait_ack = 0;

	_rtw_memset(tx_frame_buf, 0, tx_frame_len);
	pos = start = tx_frame_buf;

	/*
	 * construct mac header,
	 * seq is delt with later in _cfg80211_rtw_mgmt_tx()
	 */
	hdr = (struct rtw_ieee80211_hdr_3addr *)pos;
	set_frame_sub_type(hdr, WIFI_DEAUTH); /* type and subtype */
	_rtw_memcpy(hdr->addr1, req->bssid, ETH_ALEN);
	_rtw_memcpy(hdr->addr2, adapter_mac_addr(padapter), ETH_ALEN);
	_rtw_memcpy(hdr->addr3, req->bssid, ETH_ALEN);
	pos += sizeof(struct rtw_ieee80211_hdr_3addr);

	/* reason code */
	*(u16 *)pos = cpu_to_le16(req->reason_code);
	pos += 2;

	/* extra IEs */
	if (req->ie) {
		_rtw_memcpy(pos, req->ie, req->ie_len);
		pos += req->ie_len;
	}

	RTW_DBG_DUMP("rtw_sme_build_deauth(): frame:\n",
		     start, (int)(pos - start));

	*pp = tx_param;
	return sizeof(*tx_param) + tx_frame_len;

error:
	*pp = NULL;
	if (tx_param)
		rtw_mfree((u8 *)tx_param, sizeof(*tx_param) + tx_frame_len);
	return 0;
}

int cfg80211_rtw_deauth(struct wiphy *wiphy, struct net_device *ndev,
			struct cfg80211_deauth_request *req)
{
	_adapter *padapter;
	struct mlme_priv *mlme_priv;
	int tx_res;
	int attempt = 0;
	systime start_time = rtw_get_current_time();

	RTW_INFO("%s\n", __FUNCTION__);

	/* find adapter */
	padapter = (_adapter *)rtw_netdev_priv(ndev);
	mlme_priv = &padapter->mlmepriv;

	mlme_priv->sme_deauth_from_wpas = (u8)1;
	rtw_disconnect(wiphy, ndev, req->reason_code);
	mlme_priv->sme_deauth_from_wpas = (u8)0;

	/* check req */
	if (req->local_state_change) {
		RTW_INFO("%s(): local state change only\n", __FUNCTION__);
		return 0;
	}

	/* find adapter */
	padapter = (_adapter *)rtw_netdev_priv(ndev);
	mlme_priv = &padapter->mlmepriv;

	while (1) {
		struct mgnt_tx_parm *tx_param = NULL;
		size_t tx_param_len;

		tx_param_len = rtw_sme_build_deauth(padapter, req, &tx_param);
		if (tx_param_len <= 0)
			continue;			/* try again */

		attempt++;
		rtw_mi_set_scan_deny(padapter, 1000);
		rtw_mi_scan_abort(padapter, _TRUE);

		tx_res = rtw_mgnt_tx_handler(padapter, (u8 *)tx_param);
		if (tx_res == 0) {
			if (tx_param)
				rtw_mfree((u8 *)tx_param, tx_param_len);
			break;
		} else if (tx_res < 0) {
			RTW_ERR("%s(): failed to issue tx param\n",
				__FUNCTION__);
			if (tx_param)
				rtw_mfree((u8 *)tx_param, tx_param_len);
			if (3 <= attempt)
				break;			/* no more retry */
		}
	}

	RTW_INFO(FUNC_ADPT_FMT" %s (%d/%d) in %d ms\n", FUNC_ADPT_ARG(padapter),
		 tx_res == 0 ? "OK" : "FAIL", attempt, 3,
		 rtw_get_passing_time_ms(start_time));

	return 0;
}

static size_t rtw_sme_build_disassoc(struct _ADAPTER *padapter,
				     struct cfg80211_disassoc_request *req,
				     struct mgnt_tx_parm **pp)
{
	struct _ADAPTER_LINK *iface_link;
	struct link_mlme_ext_priv *mlmeextpriv;
	struct mgnt_tx_parm *tx_param = NULL;
	struct rtw_ieee80211_hdr_3addr *hdr;
	size_t tx_frame_len;
	u8 *tx_frame_buf;
	u8 tx_chan;
	u8 tx_band;
	u8 *pos, *start;

	RTW_INFO("%s\n", __FUNCTION__);

	/*
	 * evaluate frame len, note that this has to be exact
	 * hdr + reason_code + ies
	 */
	tx_frame_len = sizeof(struct rtw_ieee80211_hdr_3addr) + 2 + req->ie_len;

	tx_param = (struct mgnt_tx_parm *)rtw_zmalloc(sizeof(*tx_param) +
						      tx_frame_len);
	if (tx_param == NULL)
		goto error;

	iface_link = GET_PRIMARY_LINK(padapter);
	mlmeextpriv = &iface_link->mlmeextpriv;
	tx_band = mlmeextpriv->chandef.band;
	tx_chan = mlmeextpriv->chandef.chan;

	tx_frame_buf = (u8 *)tx_param + sizeof(*tx_param);

	tx_param->tx_band = tx_band;
	tx_param->tx_ch = tx_chan;
	tx_param->no_cck = 0;
	tx_param->buf = tx_frame_buf;
	tx_param->len = tx_frame_len;
	tx_param->wait_ack = 0;

	_rtw_memset(tx_frame_buf, 0, tx_frame_len);
	pos = start = tx_frame_buf;

	/*
	 * construct mac header, seq is delt with later in _cfg80211_rtw_mgmt_tx
	 */
	hdr = (struct rtw_ieee80211_hdr_3addr *)pos;
	set_frame_sub_type(hdr, WIFI_DEAUTH); /* type and subtype */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0))
	_rtw_memcpy(hdr->addr1, req->ap_addr, ETH_ALEN);
#else
	_rtw_memcpy(hdr->addr1, req->bss->bssid, ETH_ALEN);
#endif
	_rtw_memcpy(hdr->addr2, adapter_mac_addr(padapter), ETH_ALEN);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0))
	_rtw_memcpy(hdr->addr3, req->ap_addr, ETH_ALEN);
#else
	_rtw_memcpy(hdr->addr3, req->bss->bssid, ETH_ALEN);
#endif
	pos += sizeof(struct rtw_ieee80211_hdr_3addr);

	/* reason code */
	*(u16 *)pos = cpu_to_le16(req->reason_code);
	pos += 2;

	/* extra IEs */
	if (req->ie) {
		_rtw_memcpy(pos, req->ie, req->ie_len);
		pos += req->ie_len;
	}

	RTW_DBG_DUMP("rtw_sme_build_disassoc(): frame:\n",
		      start, (int)(pos - start));

	*pp = tx_param;
	return sizeof(*tx_param) + tx_frame_len;

error:
	*pp = NULL;
	if (tx_param)
		rtw_mfree((u8 *)tx_param, sizeof(*tx_param) + tx_frame_len);
	return 0;
}

int cfg80211_rtw_disassoc(struct wiphy *wiphy,
			  struct net_device *ndev,
			  struct cfg80211_disassoc_request *req)
{
	_adapter *padapter;
	struct mlme_priv *mlme_priv;
	int tx_res;
	int attempt = 0;
	systime start_time = rtw_get_current_time();

	RTW_INFO("%s\n", __FUNCTION__);

	rtw_disconnect(wiphy, ndev, req->reason_code);

	/* check req */
	if (req->local_state_change) {
		RTW_INFO("%s(): local state change only\n", __FUNCTION__);
		return 0;
	}

	/* find adapter */
	padapter = (_adapter *)rtw_netdev_priv(ndev);
	mlme_priv = &padapter->mlmepriv;

	while (1) {
		struct mgnt_tx_parm *tx_param = NULL;
		size_t tx_param_len;

		tx_param_len = rtw_sme_build_disassoc(padapter, req, &tx_param);
		if (tx_param_len <= 0)
			continue;			/* try again */

		attempt++;
		rtw_mi_set_scan_deny(padapter, 1000);
		rtw_mi_scan_abort(padapter, _TRUE);

		tx_res = rtw_mgnt_tx_handler(padapter, (u8 *)tx_param);
		if (tx_res == 0) {
			break;
		} else if (tx_res < 0) {
			RTW_ERR("%s(): failed to issue tx param\n",
				__FUNCTION__);
			if (tx_param)
				rtw_mfree((u8 *)tx_param, tx_param_len);
			if (3 <= attempt)
				break;			/* no more retry */
		}
	}

	RTW_INFO(FUNC_ADPT_FMT" %s (%d/%d) in %d ms\n", FUNC_ADPT_ARG(padapter),
		 tx_res == 0 ? "OK" : "FAIL", attempt, 3,
		 rtw_get_passing_time_ms(start_time));

	return 0;
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 30) */

void rtw_cfg80211_sme_indicate_disconnect(_adapter *padapter, u16 reason,
					  u8 locally_generated)
{
	struct wireless_dev *pwdev = padapter->rtw_wdev;
	struct rtw_wdev_priv *pwdev_priv = adapter_wdev_data(padapter);
	struct mlme_ext_priv *mlme_priv = &padapter->mlmeextpriv;
	struct mlme_ext_info *mlme_info = &mlme_priv->mlmext_info;
	struct rtw_ieee80211_hdr_3addr *hdr;
	u8 deauth_buf[sizeof(struct rtw_ieee80211_hdr_3addr) + 2] = {0};
	u8 *pos;
	u8 bcst_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

	if (padapter->mlmepriv.sme_deauth_from_wpas == 0) {
		RTW_INFO("%s(): reporting cfg80211_tx_mlme_mgmt\n"
			 , __FUNCTION__);

		pos = deauth_buf;
		hdr = (struct rtw_ieee80211_hdr_3addr *)pos;
		set_frame_sub_type(hdr, WIFI_DEAUTH); /* type and subtype */
		_rtw_memcpy(hdr->addr1, bcst_addr, ETH_ALEN);
		_rtw_memcpy(hdr->addr2, get_my_bssid(&mlme_info->dev_network),
			    ETH_ALEN);
		_rtw_memcpy(hdr->addr3, get_my_bssid(&mlme_info->dev_network),
			    ETH_ALEN);
		pos += sizeof(struct rtw_ieee80211_hdr_3addr);
		*(u16 *)pos = cpu_to_le16(reason);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
		cfg80211_tx_mlme_mgmt(padapter->pnetdev, deauth_buf,
				      sizeof(deauth_buf), false);
#else
		cfg80211_tx_mlme_mgmt(padapter->pnetdev, deauth_buf,
				      sizeof(deauth_buf));
#endif
		_rtw_spinlock_bh(&pwdev_priv->connect_req_lock);
		rtw_wdev_free_connect_req(pwdev_priv);
		_rtw_spinunlock_bh(&pwdev_priv->connect_req_lock);
	} else if (padapter->mlmepriv.sme_deauth_from_wpas == 1) {
		RTW_INFO(FUNC_ADPT_FMT" call cfg80211_disconnected, rsn:%d\n",
			 FUNC_ADPT_ARG(padapter), reason);
		rtw_cfg80211_disconnected(pwdev, reason, NULL, 0,
					  locally_generated, GFP_ATOMIC);

		_rtw_spinlock_bh(&pwdev_priv->connect_req_lock);
		rtw_wdev_free_connect_req(pwdev_priv);
		_rtw_spinunlock_bh(&pwdev_priv->connect_req_lock);
	}
}

void rtw_append_ies_from_supplicant(u8 **pframe, int *pktlen, u8 *wpas_ie,
				    int wpas_ie_len){
	u8 *oui = NULL;
	u8 *target_ie = NULL;
	u32 target_ie_len = 0;
	u8 eid;
	uint cnt;

	/* In this case, use the target IE from cfg80211 instead of using our
	 * own */

	cnt = 0;
	while (cnt < wpas_ie_len) {
		target_ie = &wpas_ie[cnt];
		target_ie_len = wpas_ie[cnt + 1] + 2;
		eid = wpas_ie[cnt];
		if(target_ie != NULL && eid != 0) {
			if(eid == _VENDOR_SPECIFIC_IE_){
				oui = &wpas_ie[cnt + 2];
				rtw_ies_remove_ie(*pframe, pktlen, 0,
						  eid, oui, 4);
				/* driver only support wpa oui currently */
			} else {
				rtw_ies_remove_ie(*pframe, pktlen, 0,
						  eid, NULL, 0);
			}

			rtw_ies_add_ie(*pframe, pktlen, 0, eid, target_ie + 2,
				       target_ie_len - 2);
			RTW_DBG("replace eid=%d\n", eid);
			RTW_DBG_DUMP("after replace ie", *pframe , *pktlen);
		}
		cnt += target_ie_len;
	}
}

void rtw_cfg80211_auth_timeout_handler(void *ctx)
{
	_adapter *padapter = (_adapter *)ctx;
	struct mlme_priv *mlme_priv = &padapter->mlmepriv;
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);
	struct rtw_chan_def u_chdef = {0};
	/* ToDo CONFIG_RTW_MLD: [currently primary link only] */
	struct _ADAPTER_LINK *padapter_link = GET_PRIMARY_LINK(padapter);
	u8 val8;

	RTW_INFO("%s(): fw_state=%x\n", __FUNCTION__, get_fwstate(mlme_priv));

	if (RTW_CANNOT_RUN(adapter_to_dvobj(padapter)))
		return;

	/* disable beacon function */
	/* val8 = 0; */
	/* rtw_hal_set_hwreg(padapter, HW_VAR_BCN_FUNC, (u8 *)(&val8)); */

	_rtw_spinlock_bh(&mlme_priv->lock);
	if (mlme_priv->sme_auth_data) {
		RTW_INFO("%s(): authentication timeout\n", __FUNCTION__);
		cfg80211_auth_timeout(padapter->pnetdev,
				      mlme_priv->sme_auth_data->bssid);
		rtw_mfree((u8 *)mlme_priv->sme_auth_data,
			  mlme_priv->sme_auth_data_len);
		mlme_priv->sme_auth_data = NULL;
	} else {
		RTW_INFO("%s(): authentication timeout but no correspong bssid \recorded\n"
			 , __FUNCTION__);
	}
	_rtw_spinunlock_bh(&mlme_priv->lock);
	if (rtw_mi_check_status(padapter, MI_LINKED)) {
		if ((rtw_phl_mr_get_chandef(dvobj->phl, padapter->phl_role,
					    padapter_link->wrlink,
					    &u_chdef) == RTW_PHL_STATUS_SUCCESS)
		    && (u_chdef.chan != 0)) {
			rtw_set_chbw_cmd(padapter, padapter_link, &u_chdef, 0,
					 RFK_TYPE_FORCE_NOT_DO);
		} else {
			RTW_ERR("%s get union chandef failed\n", __func__);
		}
	}
	rtw_clear_scan_deny(padapter);

	return;
}

void rtw_cfg80211_assoc_timeout_handler(void *ctx)
{
	_adapter *padapter = (_adapter *)ctx;
	struct mlme_priv *mlme_priv = &padapter->mlmepriv;
	struct mlme_ext_priv *mlme_ext = &padapter->mlmeextpriv;
	struct rtw_wdev_priv *pwdev_priv = adapter_wdev_data(padapter);
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);
	struct rtw_chan_def u_chdef = {0};
	/* ToDo CONFIG_RTW_MLD: [currently primary link only] */
	struct _ADAPTER_LINK *padapter_link = GET_PRIMARY_LINK(padapter);

	RTW_INFO("%s(): fw_state=%x\n", __FUNCTION__, get_fwstate(mlme_priv));

	if (RTW_CANNOT_RUN(adapter_to_dvobj(padapter)))
		return;

	_rtw_spinlock_bh(&mlme_priv->lock);
	if (mlme_priv->cfg80211_sme_assoc_bss) {
		RTW_INFO("%s(): association timeout\n", __FUNCTION__);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0))
		struct cfg80211_assoc_failure data = {
			.timeout = true,
			.bss[0] = mlme_priv->cfg80211_sme_assoc_bss,
		};
		cfg80211_assoc_failure(padapter->pnetdev, &data);
#else/*LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)*/
		cfg80211_assoc_timeout(padapter->pnetdev,
				       mlme_priv->cfg80211_sme_assoc_bss);
#endif/*LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)*/
		mlme_priv->cfg80211_sme_assoc_bss = NULL;
	} else {
		RTW_INFO("%s(): association timeout but no correspong bss recorded\n",
			 __FUNCTION__);
	}
	_rtw_spinunlock_bh(&mlme_priv->lock);
	if (rtw_mi_check_status(padapter, MI_LINKED)) {
		if ((rtw_phl_mr_get_chandef(dvobj->phl, padapter->phl_role,
					    padapter_link->wrlink,
					    &u_chdef) == RTW_PHL_STATUS_SUCCESS)
					    && (u_chdef.chan != 0)) {
			rtw_set_chbw_cmd(padapter, padapter_link, &u_chdef, 0,
					 RFK_TYPE_FORCE_NOT_DO);
		} else {
			RTW_ERR("%s get union chandef failed\n", __func__);
		}
	}
	_rtw_spinlock_bh(&pwdev_priv->connect_req_lock);
	rtw_wdev_free_connect_req(pwdev_priv);
	_rtw_spinunlock_bh(&pwdev_priv->connect_req_lock);
	if ((check_fwstate(mlme_priv, WIFI_UNDER_LINKING)) == _TRUE) {
		_clr_fwstate_(mlme_priv, WIFI_UNDER_LINKING);
	}
	report_join_res(padapter, -2, WLAN_STATUS_UNSPECIFIED_FAILURE);
	rtw_clear_scan_deny(padapter);
	_cancel_timer_ex(&mlme_ext->link_timer);

	return;
}

unsigned int OnAuth_ApSmeOffloaded(_adapter *padapter,
				   struct _ADAPTER_LINK *padapter_link,
				   struct sta_info *sta,
				   const u8 *frame,
				   size_t frame_len,
				   u16 alg,
				   u16 seq,
				   enum dot11AuthAlgrthmNum auth_mode)
{
	struct link_mlme_ext_priv *lmlmeextpriv = &padapter_link->mlmeextpriv;
	int auth_start = _FALSE;
	int auth_comp = _FALSE;
	u8 *buf;
	size_t len;

	if (!padapter->mlmepriv.smeofld_ap)
		return _FAIL;

	/* make a copy of the frame */
	buf = rtw_zmalloc(frame_len);
	if (!buf)
		return _FAIL;
	_rtw_memcpy(buf, frame, frame_len);
	len = frame_len;

	/* Update sta state based on auth mode, alg and seq */
	RTW_INFO("%s(): auth_mode=%d, alg=%u, seq=%u\n", __FUNCTION__,
		 auth_mode, alg, seq);
	switch (auth_mode) {
	case dot11AuthAlgrthm_Open:
		if (alg == WLAN_AUTH_OPEN)
			auth_comp = _TRUE;
		break;
	case dot11AuthAlgrthm_SAE:
	case dot11AuthAlgrthm_8021X:
		if (alg == WLAN_AUTH_SAE) {
			if (seq == 1)		/* commit */
				auth_start = _TRUE;
			else if (seq == 2)	/* confirm */
				auth_comp = _TRUE;
		} else if (alg == WLAN_AUTH_OPEN) {
			if (seq == 1)
				auth_comp = _TRUE;
				/* PMK cache or SAE transition mode */
		}
		break;
	case dot11AuthAlgrthm_FT_PSK:
		if (alg == WLAN_AUTH_OPEN || alg == WLAN_AUTH_FT)
			auth_comp = _TRUE;
		break;
	case dot11AuthAlgrthm_Shared: /* fall through */
	case dot11AuthAlgrthm_Auto:
		if (alg == WLAN_AUTH_SHARED_KEY) {
			if (seq == 1) {
				auth_start = _TRUE;
			} else if (seq == 3) {
				if (frame_len < WLAN_HDR_A3_LEN - 4)
					break;

				/* remove iv (len=4),
				 * keep privacy set for hostapd */
				_rtw_memcpy(buf + WLAN_HDR_A3_LEN,
					    frame + WLAN_HDR_A3_LEN + 4,
					    frame_len - WLAN_HDR_A3_LEN - 4);
				len = frame_len - 4;

				auth_comp = _TRUE;
				sta->state &= ~WIFI_FW_AUTH_STATE;
				/* Ref OnAuth() */
			}
		} else if (alg == WLAN_AUTH_OPEN) {
			auth_comp = _TRUE;
		}
		break;
	default:
		if (buf)
			rtw_mfree(buf, frame_len);
		return _FAIL;
	}

	RTW_INFO("%s(): inform rx auth on ch=%u, frame_len=%zd, drv_intf=%d\n",
		 __FUNCTION__,  lmlmeextpriv->chandef.chan, frame_len,
		 padapter->iface_id);
	rtw_cfg80211_rx_mgmt(padapter->rtw_wdev,
			     rtw_ch2freq( lmlmeextpriv->chandef.chan), 0,
			     buf, len, GFP_ATOMIC);

	if (auth_start) {
		RTW_INFO("%s(): set auth start\n", __FUNCTION__);
		sta->state &= ~WIFI_FW_AUTH_SUCCESS;
		sta->state &= ~WIFI_FW_AUTH_NULL;
		sta->state |= WIFI_FW_AUTH_STATE;
		sta->authalg = alg;
	}

	if (auth_comp) {
		RTW_INFO("%s(): set auth completed\n", __FUNCTION__);
		sta->state &= ~WIFI_FW_AUTH_NULL;
		sta->state |= WIFI_FW_AUTH_SUCCESS;
		sta->expire_to = padapter->stapriv.assoc_to;
	}

	if (buf)
		rtw_mfree(buf, frame_len);

	return _SUCCESS;
}
#endif /* CONFIG_CFG80211_SME_OFFLOAD */