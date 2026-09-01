/******************************************************************************
 *
 * Copyright(c) 2007 - 2023 Realtek Corporation.
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
#ifndef __RTW_SME_OFLD_H_
#define __RTW_SME_OFLD_H_
#ifdef CONFIG_CFG80211_SME_OFFLOAD
/**
 * struct rtw_sme_auth_data - Internal driver authentication data
 * @auth_alg: The authentication algorithm, could be WLAN_AUTH_OPEN,
 *  WLAN_AUTH_SHARED_KEY, WLAN_AUTH_SAE, WLAN_AUTH_FT, ... etc.
 * @bssid: BSSID of the BSS being authenticated
 * @tx_chan: Channel that the Authentication frame should be transmitted on
 * @key_len: Length of WEP key for shared key authentication
 * @key_index: Index of WEP key for shared key authentication
 * @sae_trans: SAE transaction number
 * @sae_status: SAE status code
 * @expect_trans: The expecting transaction number of the receiving
 *  authentication frame
 * @data_len: Length of the authentication data
 * @data: Fields and elements in Authentication frames. This contains the
 *  authentication frame body (non-IE and IE data), excluding the Authentication
 *  algorithm number, i.e., starting at the Authentication transaction sequence
 *  number field
 */
struct rtw_sme_auth_data {
	u16 auth_alg;

	u8 bssid[ETH_ALEN];
	u8 tx_band;
	u8 tx_chan;

	u8 key_len, key_index;
	u8 key[13];

	u16 sae_trans, sae_status;

	u16 expect_trans;

	size_t data_len;
	u8 data[];
};

int cfg80211_rtw_probe_client(struct wiphy *wiphy,
			      struct net_device *ndev,
			      const u8 *peer,
			      u64 *cookie);

int rtw_sme_send_auth_challenge(_adapter *padapter, u8 *frame,
				size_t frame_len,
				struct rtw_sme_auth_data *data);

int cfg80211_rtw_auth(struct wiphy *wiphy, struct net_device *ndev,
		      struct cfg80211_auth_request *req);

bool rtw_sme_onauth_client(_adapter *padapter,
			   struct _ADAPTER_LINK *padapter_link,
			   u8 *pframe,
			   uint pkt_len,
			   unsigned int algthm,
			   unsigned int seq,
			   unsigned int status);

int cfg80211_rtw_assoc(struct wiphy *wiphy, struct net_device *ndev,
		       struct cfg80211_assoc_request *req);

void rtw_sme_onassocrsp(_adapter *padapter, struct _ADAPTER_LINK *padapter_link,
			u8 *pframe, uint pkt_len);

int cfg80211_rtw_deauth(struct wiphy *wiphy, struct net_device *ndev,
			struct cfg80211_deauth_request *req);

int cfg80211_rtw_disassoc(struct wiphy *wiphy,
			  struct net_device *ndev,
			  struct cfg80211_disassoc_request *req);

void rtw_cfg80211_sme_indicate_disconnect(_adapter *padapter, u16 reason,
					  u8 locally_generated);

void rtw_append_ies_from_supplicant(u8 **pframe, int *pktlen, u8 *wpas_ie,
				    int wpas_ie_len);

void rtw_cfg80211_auth_timeout_handler(void *ctx);
void rtw_cfg80211_assoc_timeout_handler(void *ctx);

unsigned int OnAuth_ApSmeOffloaded(_adapter *padapter,
				   struct _ADAPTER_LINK *padapter_link,
				   struct sta_info *sta,
				   const u8 *frame,
				   size_t frame_len,
				   u16 alg,
				   u16 seq,
				   enum dot11AuthAlgrthmNum auth_mode);
#endif /* CONFIG_CFG80211_SME_OFFLOAD */
#endif /* __RTW_SME_OFLD_H_ */