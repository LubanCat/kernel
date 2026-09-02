/******************************************************************************
 *
 * Copyright(c) 2019 - 2021 Realtek Corporation.
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
#ifndef _RTW_PHL_H_
#define _RTW_PHL_H_

typedef struct rtw_phl_com_t HAL_DATA_TYPE; /*, *PHAL_DATA_TYPE;*/

#define GET_PHL_INFO(_dvobj)	(_dvobj->phl)
#define GET_PHL_COM(_dvobj)	(_dvobj->phl_com)
#define GET_HAL_SPEC(_dvobj)	(&(GET_PHL_COM(_dvobj)->hal_spec))


#define GET_PHY_CAP(_dvobj, _band)     (&(GET_PHL_COM(_dvobj))->phy_cap[_band])
/* Using the macro when band info not ready. */
#define GET_PHY_TX_NSS_BY_BAND(_dvobj, _band) ((GET_PHY_CAP(_dvobj, _band))->txss)
#define GET_PHY_RX_NSS_BY_BAND(_dvobj, _band) ((GET_PHY_CAP(_dvobj, _band))->rxss)

#define GET_TX_PATH_NUM(_dvobj, _band) ((GET_PHY_CAP(_dvobj, _band))->tx_path_num)
#define GET_RX_PATH_NUM(_dvobj, _band) ((GET_PHY_CAP(_dvobj, _band))->rx_path_num)

#define GET_HAL_RFPATH_NUM(_dvobj) ((GET_PHL_COM(_dvobj))->rf_path_num)
/* refer to (hal_data->version_id.RFType / registrypriv->rf_path / 8814a from efuse or registrypriv)*/
#define GET_HAL_RFPATH(_dvobj) ((GET_PHL_COM(_dvobj))->rf_type)

#define GET_WIFI_ROLE_LINK_CURRENT_CH(_adapter_link) (_adapter_link->wrlink->chandef.chan)
#define GET_WIFI_ROLE_LINK_CURRENT_BAND(_adapter_link) (_adapter_link->wrlink->chandef.band)
#define WIFI_ROLE_LINK_IS_ON_24G(_adapter_link) (_adapter_link->wrlink->chandef.band == BAND_ON_24G)
#define WIFI_ROLE_LINK_IS_ON_5G(_adapter_link) (_adapter_link->wrlink->chandef.band == BAND_ON_5G)
#define WIFI_ROLE_LINK_IS_ON_6G(_adapter_link) (_adapter_link->wrlink->chandef.band == BAND_ON_6G)

#ifdef CONFIG_USB_HCI
#define RTW_LITEXMITBUF_NR 256
#define RTW_XMITURB_NR 256
/*
 * NR_RECV_URB is the number of maximum bulk-in URBs submitted by the driver.
 * Since PHL will resubmit the bulk-in URB before the last completed URB and
 * literecvbuf are available, the driver needs to allocate NR_RECV_URB+1 URB
 * and literecvbuf to prevent URB resubmission failure.
 */
#define RTW_LITERECVBUF_NR (NR_RECV_URB + 1)
#define RTW_RECVURB_NR (NR_RECV_URB + 1)
#define RTW_INTINBUF_NR 1
#define RTW_INTINURB_NR 1
#endif

#ifdef CONFIG_PCI_HCI
#define IOT_BASE	0x11
#elif defined(CONFIG_USB_HCI)
#define IOT_BASE	0x12
#elif defined(CONFIG_SDIO_HCI)
#define IOT_BASE	0x13
#else
#define IOT_BASE	0x10
#endif

#ifndef IOT_VENDOR
#define IOT_VENDOR	0
#endif

#define IOT_ID(T)	(((T) << 16) | (IOT_VENDOR << 8) | IOT_BASE)

s8 rtw_phl_rssi_to_dbm(u8 rssi);

void rtw_hw_dump_hal_spec(void *sel, struct dvobj_priv *dvobj);
void rtw_dump_phl_sta_info(void *sel, struct sta_info *sta);

bool rtw_hw_chk_band_cap(struct dvobj_priv *dvobj, u8 cap);
bool rtw_hw_chk_bw_cap(struct dvobj_priv *dvobj, u8 cap);
bool rtw_hw_chk_proto_cap(struct dvobj_priv *dvobj, u8 cap);
bool rtw_hw_chk_wl_func(struct dvobj_priv *dvobj, u8 func);
bool rtw_hw_is_band_support(struct dvobj_priv *dvobj, u8 band);
bool rtw_hw_is_bw_support(struct dvobj_priv *dvobj, u8 bw);
bool rtw_hw_is_wireless_mode_support(struct dvobj_priv *dvobj, u8 mode);
u8 rtw_hw_get_wireless_mode(struct dvobj_priv *dvobj);
u8 rtw_hw_get_band_cap(struct dvobj_priv *dvobj);
u8 rtw_hw_get_mac_addr(struct dvobj_priv *dvobj, u8 *hw_mac_addr);

bool rtw_hw_is_mimo_support(_adapter *adapter);
u8 rtw_hw_largest_bw(struct dvobj_priv *dvobj, u8 in_bw);
u8 rtw_hw_init(struct dvobj_priv *dvobj);
void rtw_hw_deinit(struct dvobj_priv *dvobj);


u8 rtw_hw_start(struct dvobj_priv *dvobj);
void rtw_hw_stop(struct dvobj_priv *dvobj);
bool rtw_hw_get_init_completed(struct dvobj_priv *dvobj);
bool rtw_hw_is_init_completed(struct dvobj_priv *dvobj);
void rtw_hw_cap_init(struct dvobj_priv *dvobj);
void rtw_dump_rfe_type(struct dvobj_priv *d);

void rtw_collect_adapter_link_mac_addr(_adapter *adapter, u8 **mac);
u8 rtw_hw_iface_init(_adapter *adapter);
u8 rtw_hw_iface_type_change(_adapter *adapter, u8 iface_type);
void rtw_hw_iface_deinit(_adapter *adapter);

/* security */
u8 rtw_sec_algo_drv2phl(enum security_type drv_algo);
u8 rtw_sec_algo_phl2drv(enum rtw_enc_algo phl_algo);
int rtw_hw_add_key(struct _ADAPTER *a, struct sta_info *sta,
		u8 keyid, enum security_type keyalgo, u8 keytype, u8 *key,
		u8 spp, enum phl_cmd_type cmd_type, u32 cmd_timeout);
int rtw_hw_del_key(struct _ADAPTER *a, struct sta_info *sta,
		u8 keyid, u8 keytype, enum phl_cmd_type cmd_type, u32 cmd_timeout);
int rtw_hw_del_all_key(struct _ADAPTER *a, struct sta_info *sta,
		enum phl_cmd_type cmd_type, u32 cmd_timeout);

/* settting */
int rtw_hw_set_ch_bw(struct _ADAPTER *a, struct _ADAPTER_LINK *alink, enum band_type band,
		u8 ch, enum channel_width bw, u8 offset, enum rfk_tri_type rt_type);
int rtw_hw_set_edca(struct _ADAPTER *a, struct _ADAPTER_LINK *alink, u8 ac, u32 param);

/* connect */
#ifdef RTW_WKARD_UPDATE_PHL_ROLE_CAP
void rtw_update_phl_cap_by_rgstry(struct _ADAPTER *a, struct _ADAPTER_LINK *alink);
#endif
void rtw_update_phl_sta_cap(struct _ADAPTER *a, struct sta_info *sta,
			    struct protocol_cap_t *cap);
void rtw_update_phl_sta_edca(struct _ADAPTER *a, struct _ADAPTER_LINK *alink, enum rtw_ac ac, u32 param);
#if 0
int rtw_hw_prepare_connect(struct _ADAPTER *a, struct sta_info *sta, u8 *target_addr);
#endif

int rtw_hw_connect_remove_sta(struct _ADAPTER *a, struct sta_info *sta);
int rtw_hw_connect_abort(struct _ADAPTER *a);
int rtw_hw_connected(struct _ADAPTER *a);
int rtw_hw_connected_apmode(struct _ADAPTER *a, struct sta_info *sta);
int rtw_hw_disconnect(struct _ADAPTER *a, struct sta_info *sta);

void rtw_update_roch_chan_def(struct _ADAPTER_LINK *adapter_link,
				u8 remain_ch, enum channel_width remain_bw,
				enum chan_offset offset,
				enum band_type band);

void rtw_hw_update_chan_def(_adapter *adapter, struct _ADAPTER_LINK *adapter_link);

u8 rtw_sta_hal_media_status_rpt_cmd(_adapter *a, struct sta_info *sta, bool connected, u8 flag);
u8 rtw_sta_hal_ra_mask_update_cmd(_adapter *a, struct sta_info *sta, u8 flag);

u8 rtw_link_hal_core_stop_beacon(struct _ADAPTER_LINK *alink, bool stop, u8 flag);

#ifdef RTW_DETECT_HANG
void rtw_is_hang_check(struct _ADAPTER *a);
#endif

/*
 * define _HT_IOT_PEER here is a workaround, because fail to include
 * rtw_mlme_ext.h before declaring rtw_update_phl_iot().
 */
enum _HT_IOT_PEER;
void rtw_update_phl_iot(struct _ADAPTER *a, enum _HT_IOT_PEER peer);
int rtw_get_sta_tx_stat(_adapter *adapter, struct sta_info *sta);

#ifdef CONFIG_RTW_ACS
u8 rtw_acs_get_clm_ratio(struct _ADAPTER *a, enum band_type band, u8 idx);
u8 rtw_acs_get_nhm_ratio(struct _ADAPTER *a, enum band_type band, u8 ch);
s8 rtw_acs_get_noise_dbm(struct _ADAPTER *a, enum band_type band, u8 idx);
int rtw_acs_get_report(struct _ADAPTER *a, enum band_type band, u8 ch, struct rtw_acs_info_parm *rpt);
#endif /* CONFIG_RTW_ACS */

void rtw_dump_env_rpt(struct _ADAPTER *a, void *sel);

#ifdef DBG_RX_DFRAME_RAW_DATA
void rtw_dump_rx_dframe_info(struct _ADAPTER *padapter, void *sel);
#endif

#ifdef CONFIG_WOWLAN
u8 rtw_hw_wow(struct _ADAPTER *a, u8 wow_en);
#endif

#ifdef CONFIG_MCC_MODE
u8 rtw_hw_mcc_chk_inprogress(struct _ADAPTER *a, struct _ADAPTER_LINK *adapter_link);
#endif

void rtw_edcca_hal_update(struct dvobj_priv *dvobj);

#if CONFIG_TXPWR_LIMIT
enum txpwr_lmt_reg_exc_match {
	TXPWR_LMT_REG_EXC_MATCH_NONE = 0,
	TXPWR_LMT_REG_EXC_MATCH_COUNTRY,
	TXPWR_LMT_REG_EXC_MATCH_DOMAIN,
};

enum txpwr_lmt_reg_exc_match rtw_txpwr_hal_lmt_reg_exc_search(struct dvobj_priv* dvobj
	, const char *country, u8 domain, const char **reg_name);
#if CONFIG_IEEE80211_BAND_6GHZ
enum txpwr_lmt_reg_exc_match rtw_txpwr_hal_lmt_reg_exc_6g_search(struct dvobj_priv* dvobj
	, const char *country, u8 domain, const char **reg_name);
#endif
bool rtw_txpwr_hal_lmt_reg_search(struct dvobj_priv* dvobj, enum band_type band, const char *name);
void rtw_txpwr_hal_set_current_lmt_regs_by_name(struct dvobj_priv* dvobj, char *names_of_band[], int names_len_of_band[]);
void rtw_txpwr_hal_get_current_lmt_regs_name(struct dvobj_priv* dvobj, char *names_of_band[], int names_len_of_band[]);
void dump_txpwr_lmt(void *sel, _adapter *adapter);
#ifdef CONFIG_80211AX_HE
void dump_txpwr_lmt_ru(void *sel, _adapter *adapter);
#endif
#endif /* CONFIG_TXPWR_LIMIT */

void dump_txpwr_by_rate(void *sel, _adapter *adapter);

#ifdef CONFIG_DFS_MASTER
void rtw_dfs_hal_radar_detect_disable(struct dvobj_priv *dvobj, u8 band_idx);
void rtw_dfs_hal_radar_detect_enable(struct dvobj_priv *dvobj, u8 band_idx, bool cac, u32 rd_freq_hi, u32 rd_freq_lo);
void rtw_dfs_hal_set_cac_status(struct dvobj_priv *dvobj, u8 band_idx, bool cac);
void rtw_dfs_hal_csa_mg_tx_pause(struct dvobj_priv *dvobj, u8 band_idx, bool pause);
bool rtw_dfs_hal_region_supported(struct dvobj_priv* dvobj, enum rtw_dfs_regd domain);
void rtw_dfs_hal_update_region(struct dvobj_priv *dvobj, u8 band_idx, enum rtw_dfs_regd domain);
u8 rtw_dfs_hal_radar_detect_polling_int_ms(struct dvobj_priv *dvobj);
#endif /* CONFIG_DFS_MASTER */

#define SPF_PHL_RF_019_SAR /* for HALRF PW_LMT_REGU_EXT_PWR mechanism */

bool rtw_txpwr_hal_is_txpwr_limit_needed(struct dvobj_priv *dvobj);
bool rtw_txpwr_hal_get_pwr_lmt_en(struct dvobj_priv *dvobj);
struct tx_power_ext_info;
bool rtw_txpwr_hal_get_ext_info(struct dvobj_priv *dvobj, struct tx_power_ext_info *info);
void rtw_txpwr_hal_update_pwr(struct dvobj_priv *dvobj, enum phl_band_idx band_idx);

u8 get_phy_tx_nss(_adapter *adapter, struct _ADAPTER_LINK *adapter_link);
u8 get_phy_rx_nss(_adapter *adapter, struct _ADAPTER_LINK *adapter_link);
bool phy_is_txpwr_user_mbm_valid(_adapter *adapter, s16 mbm);
u8 rtw_backup_and_get_final_ss(_adapter *adapter, struct sta_info *sta, u8 chg_ss);
void rtw_ctrl_and_backup_assoc_cap_rx_nss(_adapter *adapter, struct sta_info *sta, u8 rx_nss);
#ifdef CONFIG_DBCC_P2P_BG_LISTEN
bool rtw_dbcc_b0_sta_chan_chk(struct _ADAPTER *a);

enum dbcc_chk_pcd {
	DBCC_CHK_STA_CON,
	DBCC_CHK_STA_DIS,
	DBCC_CHK_AP_START,
	DBCC_CHK_AP_STOP
};
bool rtw_dbcc_chk_enable_hdl(_adapter *adapter,
				enum phl_cmd_type cmd_type,
				enum dbcc_chk_pcd chk_pcd);
bool rtw_dbcc_chk_disable_hdl(_adapter *adapter,
				struct rtw_chan_def *new_chdef,
				enum phl_cmd_type cmd_type,
				enum dbcc_chk_pcd chk_pcd);
enum rtw_phl_status
rtw_discon_end_dbcc_en_notify(struct _ADAPTER *a, enum phl_module_id mdl_id);

#endif

#endif /* _RTW_HW_H_ */
