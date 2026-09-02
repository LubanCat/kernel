/******************************************************************************
 *
 * Copyright(c)2019 Realtek Corporation.
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
#ifndef _PHL_WOW_DEF_H_
#define _PHL_WOW_DEF_H_

enum rtw_wow_op_mode {
	RTW_WOW_OP_NONE = 0,
	RTW_WOW_OP_PWR_DOWN,
	RTW_WOW_OP_DISCONNECT_STBY,
	RTW_WOW_OP_CONNECT_STBY,
	RTW_WOW_OP_MAX = 0xF
};

enum rtw_wow_cfg_step {
	/* suspend */
	WOW_INIT_PRECONFIG = BIT0,
	WOW_INIT = BIT1,
	WOW_FUNC_EN = BIT2,
	WOW_FUNC_START = BIT3,
	WOW_INIT_POSTCONFIG = BIT4,
	/* resume */
	WOW_PHL_START = BIT5,
	WOW_GET_PWR_STATE = BIT6,
	WOW_HANDLE_WAKE_RSN = BIT7,
	WOW_MODULE_START = BIT8,
	WOW_HANDLE_AOAC_RPT_PHASE0 = BIT9,
	WOW_HANDLE_AOAC_RPT_PHASE1 = BIT10,
	WOW_FUNC_DIS = BIT11,
	WOW_FUNC_STOP = BIT12,
	WOW_DEINIT = BIT13,
	/* both */
	WOW_PS_PWR_CFG = BIT14,
	WOW_SER_CTRL = BIT15,
	WOW_SET_WOWLAN = BIT16,
	WOW_CFG_WOW_SLEEP = BIT17,
	WOW_PS_PROTO_CFG = BIT18,
	WOW_PPDU_STS_CFG = BIT19,
	WOW_DBG_DUMP = BIT20
};

#define SET_WOW_INIT_ERR(_wow_info, _flags) \
	(_wow_info)->err.init |= (_flags)

#define SET_WOW_DEINIT_ERR(_wow_info, _flags) \
	(_wow_info)->err.deinit |= (_flags)

struct rtw_keep_alive_info {
	/* core */
	u8 keep_alive_en;
	u8 keep_alive_period;
	u8 keep_alive_pkt_type;
	/* phl/hal */
	u8 keep_alive_pkt_id;
};

struct rtw_disc_det_info {
	/* core */
	u8 disc_det_en;
	u8 disc_wake_en;
	u8 try_pkt_count;
	u8 check_period;
	u8 cnt_bcn_lost_en;
	u8 cnt_bcn_lost_limit;
};

#define MAX_NLO_NUM 10
#define MAX_SSID_LEN 32
#define MAX_NLO_CHANNEL 40

struct rtw_nlo_info {
	u8 nlo_en;
	u8 num_of_networks;
	u8 num_of_hidden_ap;
	u8 ssid[MAX_NLO_NUM][MAX_SSID_LEN];
	u8 ssidlen[MAX_NLO_NUM];
	u8 chipertype[MAX_NLO_NUM];
	u8 compare_cipher_type;
	u8 probe_req_id;
	struct scan_ofld_ch_info channel_list[MAX_NLO_CHANNEL];
	u8 channel_num;
	u32 delay; /* ms */
	u32 period; /* ms */
	u8 cycle;
	u32 slow_period; /* ms */
	void (* construct_pbreq)(void *priv, u8 *pkt_buf, u16 *len);
};

struct rtw_arp_ofld_content {
	u8 arp_en;
	u8 a3[MAC_ADDRESS_LENGTH];
	u8 remote_ipv4_addr[IPV4_ADDRESS_LENGTH];
	u8 host_ipv4_addr[IPV4_ADDRESS_LENGTH];
	u8 remote_mac_addr[MAC_ADDRESS_LENGTH];
};

struct rtw_arp_ofld_info {
	u8 arp_en;
	u8 arp_action; /* 0 = send arp response, 1 = wake up host */
	u8 arp_rsp_id;
	struct rtw_arp_ofld_content arp_ofld_content;
};

struct rtw_ndp_ofld_content {
	u8 ndp_en;
	u8 chk_remote_ip;
	u8 num_target_ip;
	u8 mac_addr[MAC_ADDRESS_LENGTH];
	u8 remote_ipv6_addr[IPV6_ADDRESS_LENGTH];
	u8 target_ipv6_addr[2][IPV6_ADDRESS_LENGTH];
};

struct rtw_ndp_ofld_info {
	u8 ndp_en;
	u8 ndp_id;
	struct rtw_ndp_ofld_content ndp_ofld_content[2];
};

struct rtw_gtk_ofld_content {
	u8 kck[32];
	u32 kck_len;
	u8 kek[32];
	u32 kek_len;
	u8 tk1[16];
	u8 txmickey[8];
	u8 rxmickey[8];
	u8 replay_cnt[8];

	u8 igtk_keyid[4];
	u8 ipn[8];
	u8 igtk[2][32];
	u8 igtk_len;
	u8 psk[32];
	u8 psk_len;
};

struct rtw_gtk_ofld_info {
	/* core */
	u8 gtk_en;
	u8 tkip_en;
	u8 ieee80211w_en;
	u8 pairwise_wakeup;
	u8 bip_sec_algo;
	u8 akmtype_byte3;
	struct rtw_gtk_ofld_content gtk_ofld_content;

	/* phl */
	u8 hw_11w_en; /* keep 1 for BIP-CMAC-128 so far */
	u8 gtk_rsp_id; /* eapol pkt id */
	u8 sa_query_id;
};

#define MAX_WOW_PATTERN_SIZE_BIT 128
#define MAX_WOW_PATTERN_SIZE_BYTE 16
#define MAX_WOW_PATTERN_SIZE_DWORD 4

struct rtw_wowcam_upd_info {
	u8 rw;
	u8 wow_cam_idx;
	u32 wake_mask[4];
	u16 match_crc;

	u8 is_negative_pattern_match;
	u8 skip_mac_hdr;
	u8 uc;
	u8 mc;
	u8 bc;

	u8 valid;
	u8 ptrn[MAX_WOW_PATTERN_SIZE_BIT];
	u32 ptrn_len;
	u8 mask[MAX_WOW_PATTERN_SIZE_BYTE];
};

#define MAX_WOW_CAM_NUM 18
struct rtw_pattern_match_info{
	struct rtw_wowcam_upd_info wowcam_info[MAX_WOW_CAM_NUM];
};

#define MAX_REALWOW_KCP_SIZE 124 /* (100 + 24) */
#define MAX_REALWOW_PAYLOAD 64

struct rtw_realwow_ofld_content {
	u16 interval; /* unit : 1 ms */
	u16 keep_alive_pkt_size;
	u16 ack_lost_limit;
	u16 ack_ptrn_size;
	u16 wakeup_ptrn_size;
	u16 keep_alive_pkt_ptrn[MAX_REALWOW_KCP_SIZE];
	u8 ack_ptrn[MAX_REALWOW_PAYLOAD];
	u8 wakeup_ptrn[MAX_REALWOW_PAYLOAD];
	u32 wakeup_sec_num;
};

struct rtw_realwow_info {
	u8 realwow_en;
	u8 auto_wakeup;
	u8 keepalive_id;
	u8 wakeup_pattern_id;
	u8 ack_pattern_id;
	struct rtw_realwow_ofld_content realwow_ofld_content;
};

struct rtw_wow_gpio_info {
	struct rtw_dev2hst_gpio_info d2h_gpio_info;
	enum rtw_gpio_mode dev2hst_gpio_mode;
	u8 dev2hst_gpio;
	u8 dev2hst_high;
};

struct rtw_periodic_wake_info {
	u8 periodic_wake_en;
	u32 wake_period;
	u32 wake_duration;
};

struct rtw_remote_wake_ctrl_info {
	/* core */
	u8 ptk_tx_iv[IV_LENGTH];
	u8 valid_check;
	u8 symbol_check_en;
	u8 gtk_key_idx;
	u8 ptk_rx_iv[IV_LENGTH];
	u8 gtk_rx_iv_idx0[IV_LENGTH];
	u8 gtk_rx_iv_idx1[IV_LENGTH];
	u8 gtk_rx_iv_idx2[IV_LENGTH];
	u8 gtk_rx_iv_idx3[IV_LENGTH];
};


#ifdef CONFIG_PHL_MDNS_OFFLOAD
#define MAX_MDNS_RESP_NUM 8
#define MAX_MDNS_RESP_LEN 512
#define MAX_MDNS_MATCH_CRITERIA_NUM 8
#define MAX_MDNS_PASSTHRU_NAME_NUM 8
#define MAX_MDNS_DOMAIN_NAME_LEN 255
#define PASSTHRU_FORWARD_ALL 0
#define PASSTHRU_DROP_ALL 1
#define PASSTHRU_LIST 2

struct rtw_mdns_ipv4_header {
	u8 src_ipv4_addr[IPV4_ADDRESS_LENGTH];
	u8 dst_ipv4_addr[IPV4_ADDRESS_LENGTH];
	u8 remote_mac_addr[MAC_ADDRESS_LENGTH];
	u8 protect_bit;
	u8 sec_hdr_len;
	u8 ipv4_pktid;
};

struct rtw_mdns_ipv6_header {
	u8 src_ipv6_addr[IPV6_ADDRESS_LENGTH];
	u8 dst_ipv6_addr[IPV6_ADDRESS_LENGTH];
	u8 remote_mac_addr[MAC_ADDRESS_LENGTH];
	u8 protect_bit;
	u8 sec_hdr_len;
	u8 ipv6_pktid;
};

struct rtw_mdns_match_criteria {
	u16 name_offset;
	u16 type;
	u8 name_len;
};

struct rtw_mdns_resp_entry {
	u16 content_len;
	u8 data_pktid;
	u8 match_ct_num;
	u8 content[MAX_MDNS_RESP_LEN];
	struct rtw_mdns_match_criteria match_ct[MAX_MDNS_MATCH_CRITERIA_NUM];
};

struct rtw_mdns_passthru_name {
	u8 name[MAX_MDNS_DOMAIN_NAME_LEN];
	u8 name_len;
	u8 pass_pktid;
};

struct rtw_mdns_passthru_list {
	u8 passthru_behavior;
	u8 passthru_name_num;
	struct rtw_mdns_passthru_name passthru_name[MAX_MDNS_PASSTHRU_NAME_NUM];
};

struct rtw_mdns_ofld_info {
	u8 mdns_en;
	u8 offload_state;
	struct rtw_mdns_resp_entry resp_entry[MAX_MDNS_RESP_NUM];
	struct rtw_mdns_passthru_list passthru_list;
	struct rtw_mdns_ipv4_header mdns_ipv4_header;
	struct rtw_mdns_ipv6_header mdns_ipv6_header;
};
#endif /* CONFIG_PHL_MDNS_OFFLOAD */
#ifdef CONFIG_PHL_APF
#ifdef CONFIG_APF_RAM_SIZE
#define MAX_APF_RAM_SIZE CONFIG_APF_RAM_SIZE
#define APF_RAM_FRAG_SIZE CONFIG_APF_RAM_FRAG_SIZE
#else
#warning "CONFIG_APF_RAM_SIZE is not defined, using default ram size 1024"
#define MAX_APF_RAM_SIZE 1024
#define APF_RAM_FRAG_SIZE 1024
#endif /* CONFIG_APF_RAM_SIZE */

#ifdef CONFIG_APF_TP_TH
#define APF_ACTIVE_TP_TH CONFIG_APF_ACTIVE_TP_TH
#else
#define APF_ACTIVE_TP_TH 10
#endif /* CONFIG_APF_TP_TH */


#define APF_RAM_FRAG_NUM(__len, __size) (u32)(((__len)/(__size)) + ((__len)&((__size) - 1) ? 1 : 0))
#define STR_FW_MODE(__apf_fw_mode) (__apf_fw_mode == RTW_FW_NIC ? "FW_NIC": \
				    (__apf_fw_mode == RTW_FW_WOWLAN ? "FW_WOWLAN" : "Unknown"))

enum phl_apf_force_en_state {
	PHL_APF_FORCE_IGNORE = 0,
	PHL_APF_FORCE_DIS,
};

enum phl_apf_cmd {
	PHL_APF_CMD_DISABLE = 0,
	PHL_APF_CMD_ENABLE  = 1,
	PHL_APF_CMD_REPORT  = 2,
	PHL_APF_CMD_STANDBY = 3,
	PHL_APF_CMD_CLEAR   = 4,
	PHL_APF_CMD_CHANGE  = 5,
	/* used for drv */
	PHL_APF_CMD_DRV,
	/* used in watchdog for tp check */
	PHL_APF_CMD_CHECK,
	PHL_APF_CMD_MEDIA_STATUS_CHG,
	/* used in suspend/resume process */
	PHL_APF_CMD_CHG_FW,
	PHL_APF_CMD_NONE    = 0xFF,
};

enum phl_apf_state {
	PHL_APF_DISABLE    = 0,
	PHL_APF_ENABLE     = 1,
	PHL_APF_UNFINISHED = 2,
	PHL_APF_STANDBY    = 3,
};

struct _apf_enum_map {
	u32 val;
	const char *str;
};

static const struct _apf_enum_map PHL_APF_CMD_MAP[] = {
	{PHL_APF_CMD_DISABLE, "PHL_APF_CMD_DISABLE"},
	{PHL_APF_CMD_ENABLE, "PHL_APF_CMD_ENABLE"},
	{PHL_APF_CMD_REPORT, "PHL_APF_CMD_REPORT"},
	{PHL_APF_CMD_STANDBY, "PHL_APF_CMD_STANDBY"},
	{PHL_APF_CMD_CLEAR, "PHL_APF_CMD_CLEAR"},
	{PHL_APF_CMD_CHANGE, "PHL_APF_CMD_CHANGE"},
	{PHL_APF_CMD_DRV, "PHL_APF_CMD_DRV"},
	{PHL_APF_CMD_CHECK, "PHL_APF_CMD_CHECK"},
	{PHL_APF_CMD_MEDIA_STATUS_CHG, "PHL_APF_CMD_MEDIA_STATUS_CHG"},
	{PHL_APF_CMD_CHG_FW, "PHL_APF_CMD_CHG_FW"},
	{PHL_APF_CMD_NONE, "PHL_APF_CMD_NONE"},
};
#define PHL_APF_CMD_MAP_SIZE (sizeof(PHL_APF_CMD_MAP) / sizeof(PHL_APF_CMD_MAP[0]))

static const struct _apf_enum_map PHL_APF_STATE_MAP[] = {
	{PHL_APF_DISABLE, "PHL_APF_DISABLE"},
	{PHL_APF_ENABLE, "PHL_APF_ENABLE"},
	{PHL_APF_UNFINISHED, "PHL_APF_UNFINISHED"},
	{PHL_APF_STANDBY, "PHL_APF_STANDBY"},
};
#define PHL_APF_STATE_MAP_SIZE (sizeof(PHL_APF_STATE_MAP) / sizeof(PHL_APF_STATE_MAP[0]))

static inline const char* get_enum_str_from_map(int val, const struct _apf_enum_map *map, size_t map_size, const char *unknown_str) {
	size_t i;
	for (i = 0; i < map_size; ++i) {
		if (map[i].val == val) {
			return map[i].str;
		}
	}
	return unknown_str;
}

static inline const char* _apf_cmd_to_str(enum phl_apf_cmd cmd) {
	return get_enum_str_from_map(cmd, PHL_APF_CMD_MAP, PHL_APF_CMD_MAP_SIZE, "UNKNOWN_PHL_APF_CMD");
}

static inline const char* _apf_state_to_str(enum phl_apf_state state) {
	return get_enum_str_from_map(state, PHL_APF_STATE_MAP, PHL_APF_STATE_MAP_SIZE, "UNKNOWN_PHL_APF_STATE");
}

struct _apf_ram {
	u16 prog_len;
	u8 buf[MAX_APF_RAM_SIZE];
};
struct phl_apf_cmd_blob {
	enum phl_apf_cmd cmd;
	enum rtw_fw_type fw_mode;
	u8 mac_chg;
};
struct phl_apf_info {
	u8 apf_cmd;
	enum phl_apf_state pre_apf_state;
	enum phl_apf_state apf_state;
	enum rtw_fw_type fw_mode;
	struct _apf_ram ram;

	u16 macid;
	/* ==== pkt ofld related ====*/
	/* dummy, this will be replace by apf prog */
	u8 a1[MAC_ADDRESS_LENGTH];
	/* sta mac addr */
	u8 a2[MAC_ADDRESS_LENGTH];
	/* bssid */
	u8 a3[MAC_ADDRESS_LENGTH];
	/* used for ram frag pkt ofld */
	u8 ofld_idx;
	/* used for fw chg, keep apf counter consistent */
	u8 is_ram_ofld;
	/* used to record how much pkt_ofld used */
	u8 ram_frag_num;
	/* indicate pkt ofld is for mac hdr or not */
	u8 mac_chg;
	/* apf_ram_token[0] : mac hdr, apf_ram_token[1:7] : apf prog */
	u32 apf_ram_token[8];
	/* apf_ram_frag_pktid[0] : mac hdr, apf_ram_frag_pktid[1:7] : apf prog */
	u8 apf_ram_frag_pktid[8];
	/* security info for mac hdr */
	u8 pairwise_sec_algo;
	u8 group_sec_algo;
	u8 hw_sec_iv;
	u8 key_idx;
	u8 protect_bit;
	u8 sec_hdr_len;

	/* DBG purpose */
	enum phl_apf_force_en_state apf_force_dis;
#ifdef	CONFIG_PHL_APF_DBG
	u64 apf_cmd_seq;
#endif
	u32 wait_start_time;
	u32 wait_period;

};
#endif /* CONFIG_PHL_APF */


struct rtw_wow_wake_info {
	/* core */
	u8 wow_en;
	u8 drop_all_pkt;
	u8 rx_parse_after_wake;
	u8 pairwise_sec_algo;
	u8 group_sec_algo;
	u8 bip_sec_algo;
	u8 pattern_match_en;
	u8 magic_pkt_en;
	u8 hw_unicast_en;
	u8 fw_unicast_en;
	u8 deauth_wakeup;
	u8 rekey_wakeup;
	u8 eap_wakeup;
	u8 all_data_wakeup;
	struct rtw_remote_wake_ctrl_info remote_wake_ctrl_info;
};

struct rtw_aoac_report {
	/* status check */
	u8 rpt_fail;

	/* report from fw */
	u8 rpt_ver;
	u8 sec_type;
	u8 key_idx;
	u8 pattern_idx;
	u8 rekey_ok;
	u8 ptk_tx_iv[IV_LENGTH];
	u8 eapol_key_replay_count[8];
	u8 gtk[32];
	u8 ptk_rx_iv[IV_LENGTH];
	u8 gtk_rx_iv[4][IV_LENGTH];
	u8 igtk_key_id[8];
	u8 igtk_ipn[8];
	u8 igtk[32];
	u8 csa_pri_ch;
	u8 csa_bw:4;
	u8 csa_ch_offset:4;
	u8 csa_chsw_failed:1;
	u8 csa_ch_band:2;
	u8 csa_rsvd0:5;
	u8 csa_rsvd1;
};

#ifdef CONFIG_WOWLAN

/* Exported APIs to core */
enum rtw_phl_status rtw_phl_cfg_keep_alive_info(void *phl, struct rtw_keep_alive_info *info);
enum rtw_phl_status rtw_phl_cfg_disc_det_info(void *phl, struct rtw_disc_det_info *info);
void rtw_phl_cfg_nlo_info(void *phl, struct rtw_nlo_info *info);
void rtw_phl_cfg_periodic_wake_info(void *phl, struct rtw_periodic_wake_info *info);
void rtw_phl_cfg_wow_csa_info(void *phl, struct rtw_hal_mac_sta_csa *info);
void rtw_phl_cfg_arp_ofld_info(void *phl, struct rtw_arp_ofld_info *info);
void rtw_phl_cfg_ndp_ofld_info(void *phl, struct rtw_ndp_ofld_info *info);
#ifdef CONFIG_PHL_MDNS_OFFLOAD
void rtw_phl_cfg_mdns_ofld_info(void *phl, struct rtw_mdns_ofld_info *info);
#endif
enum rtw_phl_status rtw_phl_remove_wow_ptrn_info(void *phl, u8 phl_ptrn_id);
enum rtw_phl_status rtw_phl_get_wow_ptrn_info(void *phl, struct rtw_wowcam_upd_info *info, u8 wowcam_id);
enum rtw_phl_status rtw_phl_add_wow_ptrn_info(void *phl, struct rtw_wowcam_upd_info *info, u8 *phl_ptrn_id);
enum rtw_phl_status rtw_phl_cfg_gtk_ofld_info(void *phl, struct rtw_gtk_ofld_info *info);
enum rtw_phl_status rtw_phl_cfg_realwow_info(void *phl, struct rtw_realwow_info *info);
enum rtw_phl_status rtw_phl_cfg_wow_wake(void *phl, struct rtw_wow_wake_info *info);
enum rtw_phl_status rtw_phl_cfg_gpio_wake_pulse(void *phl, struct rtw_wow_gpio_info *info);
const char *rtw_phl_get_wow_rsn_str(void *phl, u8 wake_rsn);
enum rtw_phl_status rtw_phl_cfg_wow_set_sw_gpio_mode(void *phl, struct rtw_wow_gpio_info *info);
enum rtw_phl_status rtw_phl_cfg_wow_sw_gpio_ctrl(void *phl, struct rtw_wow_gpio_info *info);
void rtw_phl_wow_set_no_link_mode(void *phl, u8 no_link_mode);
u32 rtw_phl_get_wow_fail_stat(void *phl);

#ifdef CONFIG_PHL_APF
static inline bool _is_state_impl(enum phl_apf_state state, const enum phl_apf_state *states, size_t count)
{
	size_t i;
	for (i = 0; i < count; ++i) {
		if (state == states[i])
			return true;
	}
	return false;
}
#define IS_STATE(state, ...) _is_state_impl(state, (enum phl_apf_state[]){__VA_ARGS__}, sizeof((enum phl_apf_state[]){__VA_ARGS__}) / sizeof(enum phl_apf_state))
#define IS_STATE_ERR(state, ...) _is_state_impl(state, (enum phl_apf_state[]){__VA_ARGS__}, sizeof((enum phl_apf_state[]){__VA_ARGS__}) / sizeof(enum phl_apf_state))

enum rtw_phl_status rtw_phl_init_apf(void *phl, struct phl_apf_info *apf_info);
enum rtw_phl_status rtw_phl_manipulate_apf(void *phl, u8 *param);
enum rtw_phl_status rtw_phl_manipulate_apf_cmd(void *phl, struct phl_apf_cmd_blob *cmd_blob, u8 cmd_type);
#endif /* CONFIG_PHL_APF */

#endif /* CONFIG_WOWLAN */

#endif /* _PHL_WOW_DEF_H_ */

