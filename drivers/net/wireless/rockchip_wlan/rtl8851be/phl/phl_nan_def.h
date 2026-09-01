/******************************************************************************
 *
 * Copyright(c) 2024 Realtek Corporation.
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
#ifndef _PHL_NAN_DEF_H_
#define _PHL_NAN_DEF_H_

#ifdef CONFIG_PHL_NAN

#define SYNC_BCN_INTVL (512)
#define DISC_BCN_INTVL (100)

static const u8 nan_network_id[MAC_ALEN] = {0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00};
static const u8 nan_cluster_id[MAC_ALEN] = {0x50, 0x6f, 0x9a, 0x01, 0x00, 0x00};

static unsigned char NAN_OUI_BCN[] = {0x50, 0x6F, 0x9A, 0x13};
static unsigned char NAN_OUI_SDF[] = {0x50, 0x6F, 0x9A, 0x13};
static unsigned char WFA_OUI[] = {0x50, 0x6F, 0x9A};

#define BCN_ATTR_START_OFFSET	(6)
#define SDF_ATTR_START_OFFSET	(6)

#define DEFAULT_NAN_24G_CH	(6)
#define DEFAULT_NAN_5G_CH	(149)
#define MAX_IE_LEN_FOR_NAN	(350)

/* NAN definition for public usage */

/* halmac api */
enum nan_act_req_module_id {
	NAN_ACT_REQ_MODULE_NAN_DW = 0x0,
	NAN_ACT_REQ_MODULE_NAN_DISC_BCN,
	NAN_ACT_REQ_MODULE_NAN_FAW,
	NAN_ACT_REQ_MODULE_STA,
	NAN_ACT_REQ_MODULE_AP,
	NAN_ACT_REQ_MODULE_PAUSE,
	NAN_ACT_REQ_MODULE_SWITCH,
	NAN_ACT_REQ_MODULE_MAX
};

enum nan_act_schdl_req_option {
	NAN_ACT_REQ_OPT_REMOVE = BIT0,
	NAN_ACT_REQ_OPT_UPDATE = BIT1
};

enum nan_func_ctrl_option {
	NAN_CTRL_OPT_ENABLE = BIT0,
	NAN_CTRL_OPT_TEST_MODE = BIT1,
	NAN_CTRL_OPT_DUAL_BAND = BIT2,
	NAN_CTRL_OPT_DISABLE_DISC_BCN_24G = BIT3,
	NAN_CTRL_OPT_DISABLE_DISC_BCN_5G = BIT4,
	NAN_CTRL_OPT_BYPASS_FWFIFO = BIT5,
	NAN_CTRL_OPT_IGNORE_CBSSID_BCN = BIT6,
	NAN_CTRL_OPT_UPDATE_PARAMETER = BIT7
};

enum nan_func_ctrl_update_para_option {
	NAN_CTRL_UPDATE_PORT_IDX = BIT0,
	NAN_CTRL_UPDATE_MAC_ID = BIT1,
	NAN_CTRL_UPDATE_MASTER_PREF = BIT2,
	NAN_CTRL_UPDATE_RANDOM_FACTOR = BIT3,
	NAN_CTRL_UPDATE_OP_CH_24G = BIT4,
	NAN_CTRL_UPDATE_OP_CH_5G = BIT5,
	NAN_CTRL_UPDATE_TIME_INDICATE_PERIOD = BIT6,
	NAN_CTRL_UPDATE_NAN_CLUSTER_ID = BIT7
};

enum rtw_nan_bcn_type {
	NAN_BCN_TYPE_SYNC = 0,
	NAN_BCN_TYPE_DISC = 1
};


struct rtw_phl_nan_send_bcn {
	u8 port;
	u8 mbssid;
	u8 band;
	u8 grp_ie_ofst;
	u8 macid;
	u8 ssn_sel;
	u8 ssn_mode;
	u16 rate_sel;
	u8 txpwr;
	u8 txinfo_ctrl_en;
	u8 ntx_path_en;
	u8 path_map_a;
	u8 path_map_b;
	u8 path_map_c;
	u8 path_map_d;
	u8 antsel_a;
	u8 antsel_b;
	u8 antsel_c;
	u8 antsel_d;
	u8 sw_tsf;
	u8 *pld_buf;
	u16 pld_len;
	u16 csa_ofst;
};

struct rtw_phl_act_skd_req_para {
	u8 module_id;	/* refer to enum halmac_act_req_module_id */
	u8 priority;	/* priority of dedicated schedule */
	u8 options;
	u8 rsvd;
	/* todo: un-use param */
	/* u32 start_time; */ /* absolute time in accordance with TSF of the dedicated HW port */
	/* u32 duration; */ /* persented time each schedule */
	/* u32 period; */ /* time interval between two consecutive schedule */
	u8 tsf_idx;	/* the dedicated HW port idx */
	u8 channel; /* central channel*/
	u8 bw;
	u8 prim_chnl; /* primary channel */
	u8 ch_band_type;
};

struct rtw_phl_nan_func_ctrl {
	u8 port_idx;
	u8 master_pref;
	u8 random_factor;
	u8 op_ch_24g;
	u8 op_ch_5g;
	u8 time_indicate_period;
	u8 cluster_id[MAC_ALEN];
	u16 options; /* refer to halmac_nan_func_ctrl_option */
	u16 para_options;
	u16 mac_id_bcn;
	u16 mac_id_mgn;
	u8 fw_test_para_1;
	u8 fw_test_para_2;
};

/* nan module : sync engine */

/* (ac) enum nan_cmd_type */
enum rtw_phl_nan_cmd_type {
	/* NAN cmd type */
	NAN_CMD_TYPE_NONE	= 0,
	NAN_CMD_TYPE_NATIVE_IW	= 1,
	NAN_CMD_TYPE_CFGVENDOR	= 2
};

struct rtw_phl_nan_cluster_rpt {
	/* NAN_C2H_TYPE_CLUSTER_INFO */
	u8 cluster_id[MAC_ALEN];
	u8 master_pref;
	u8 random_factor;
	u64 amr;
	u32 ambtt;
	u8 hop_count;
};

struct rtw_phl_nan_tsf_rpt {
	/* NAN_C2H_TYPE_TSF_INFO */
	u32 port_dwst_low;	/* DW start time by dedicated HW port TSF */
	u32 fr_dwst_low;	/* DW start time by free run counter TSF */
};

struct rtw_phl_nan_info_rpt {
	enum mac_ax_nan_c2h_type c2h_type;
	union {
		struct rtw_phl_nan_cluster_rpt cluster_rpt;
		struct rtw_phl_nan_tsf_rpt tsf_rpt;
	} u;
};

enum rtw_phl_nan_pkt_type {
	NAN_PKT_TYPE_NONE	= 0x0,
	NAN_PKT_TYPE_BCN	= 0x1,
	NAN_PKT_TYPE_SDF	= 0x2,
};

/* nan module : discovery engine */

#define NAN_SRVC_ID_LEN		(6)
#define MAX_SERVICE_NAME_LEN	(255)
#define MAX_SDA_SRVC_INFO_LEN	(255)
#define MAX_MATCHING_FILTERS	(16)
#define MAX_MATCHING_FILTER_LEN	(32)

/* Service control field (SCF) of service descriptor attribute */
#define SCF_TYPE_MASK		0x3
#define SCF_TYPE_PUBLISH	0x0
#define SCF_TYPE_SUBSCRIBE	0x1
#define SCF_TYPE_FOLLOWUP	0x2
#define SCF_TYPE_RSVD		0x3

enum _nan_srv_search_type{
	GET_SRVC_BY_COOKIE		= BIT(0),
	GET_SRVC_BY_SRVC_ID		= BIT(1),
	GET_SRVC_BY_SRVC_TYPE		= BIT(2),
	GET_SRVC_BY_INST_ID		= BIT(3),
	GET_SRVC_BY_PUBLISH_INST_ID	= BIT(4),
	GET_SRVC_BY_FIRST_SRVC_ID	= BIT(5),
};

enum rtw_phl_nan_publish_solicited_tx_type {
	NAN_SOLICITED_TX_MULTICAST = 1,
	NAN_SOLICITED_TX_UNICAST   = 2,
	NAN_SOLICITED_TX_BROADCAST = 3
};

/* (define/enum) nan module : data engine */

#define MAX_IPV6_INFO_LEN 		(8)
#define MAX_PMK_LEN			(32)
#define MAX_PMKID_LEN			(16)
#define SDA_SERVICE_ID_LEN		(6)

/* (define/enum) common */

enum rtw_phl_nan_report_type {
	/* discovery engine */
	NAN_REPORT_RX_MATCH_SERVICE	= 0x1,	/* cfg80211 */
	NAN_REPORT_RX_DISC_RESULT	= 0x2,	/* cfgvendor */
	NAN_REPORT_RX_MATCH_FOLLOWUP	= 0x3,	/* cfgvendor */
	NAN_REPORT_REPLY_PUBLISH	= 0x4,	/* cfgvendor */
	NAN_REPORT_REPLY_FOLLOWUP	= 0x5,	/* cfgvendor */
	NAN_REPORT_RM_PUBLISH		= 0x6,	/* cfgvendor */
	NAN_REPORT_RM_SUBSCRIBE		= 0x7,	/* cfgvendor */
};

enum rtw_phl_nan_free_type {
	/* discovery engine */
	NAN_FREE_SERVICE	= 0x1, /* cfg80211 */
};

/* (struct) common */
struct rtw_phl_nan_ie_container {
	/* allocate memory in core layer using phl api,
	 * and free them in phl layer */
	u8 *ie_s_rate; /* support rate */
	u8 *ie_es_rate; /* ext support rate */
	u8 *ie_ht;
	u8 *ie_vht;

	u16 ie_s_rate_len;
	u16 ie_es_rate_len;
	u16 ie_ht_len;
	u16 ie_vht_len;
};

struct rtw_phl_nan_ops {
	enum rtw_phl_status (*prepare_ies)(void *drv_priv,
					   u8 band_support,
					   struct rtw_phl_nan_ie_container *ies);
	enum rtw_phl_status (*tx_mgnt)(void *drv_priv,
				       u16 macid,
				       u8 *a1, u8 *a2, u8 *a3, u8 *addr,
				       u8 *nan_attr_buf, u16 nan_attr_buf_len,
				       enum rtw_phl_nan_pkt_type nan_pkt_type);
	enum rtw_phl_status (*get_attr_start)(u8 *precv_frame,
					      u16 precv_frame_len,
					      u8 **ppattr_start,
					      u16 *total_len);
	enum rtw_phl_status (*get_frame_body)(u8 *precv_frame,
					      u16 precv_frame_len,
					      u8 **ppframe_body,
					      u16 *frame_body_len);
	enum rtw_phl_status (*report_to_osdep)(void *drv_priv, u8 *addr,
					       enum rtw_phl_nan_report_type type,
					       void *para,
					       u16 para_len);
	enum rtw_phl_status (*free_osdep_data)(enum rtw_phl_nan_free_type type,
					       void *para,
					       u16 para_len);
	enum rtw_phl_status (*alloc_peer_sta_info)(void *drv_priv,
						   u8* nan_addr,
						   u8* peer_addr,
						   u8 *ht_cap,
						   u8 *vht_cap,
						   u8 *macid,
						   u8 is_ndi);
	enum rtw_phl_status (*free_peer_sta_info)(void *drv_priv,
						  u8 *nan_addr,
						  struct rtw_phl_stainfo_t *peer_phl_sta_info);
};

/* (struct) nan module : data engine */
struct rtw_phl_nan_qos_req {
#if 0
	u8 tid;
	u32 srvc_data_size;
	u32 mean_data_rate;
	u8 max_srvc_int;
#endif
	u8 min_duration;
	u8 max_latency;
	u8 type;
};

/* (struct) nan module : discovery engine */

struct rtw_phl_nan_func_filter {
	u8 filter[MAX_MATCHING_FILTER_LEN];
	u8 len;
};

struct rtw_phl_nan_service_info {
	/*
	 * service info can be conatined in SDA/SDEA/NDP/NDPE
	 * If the content start with 0x50-6F-9A (Wi-Fi Alliance specific OUI),
	 * the format of service info is defineded in spec
	 */
	u8 *info; /* [MAX_SDA_SRVC_INFO_LEN]; */
	u16 len;
};

struct rtw_phl_nan_srf_mac_addrs {
	u8 mac_addr[MAC_ALEN];
};

struct rtw_phl_nan_sec_req {
	/*
	*	pmk and pmkid only use for publish
	*	and these two parameters follow specific service.
	*	pmkid is calculated form pmk and broadcast
	*	mac addr,and used in solicted/unsolicated publish SDF
	*/
	u8 pmk[MAX_PMK_LEN];
	u8 pmkid[MAX_PMKID_LEN];
};

struct rtw_phl_nan_publish_info {
	u8 publish_id; /* publish_instance_id */
	u8 type; /* bitmap : unsolicited(bit1) | solicited(bit0) */
	u8 solicited_tx_type; /* nan_publish_solicited_tx_type */

	/* common service id and config for pub/sub */
	u8 service_id[NAN_SRVC_ID_LEN];
	u8 service_name[MAX_SERVICE_NAME_LEN];
	struct rtw_phl_nan_service_info srv_spec_info;
	u8 period; /* announcement_period for publish or query_period for subscribe */
	u8 awake_dw_int; /* unit: discovery windows */
	u8 close_range; /* the pub/sub message is limited in range to close proximity */
	u16 sde_control; /* nan_service_descriptor_extension_controls */
	u32 ttl; /* unit: discovery windows */

	/* publishing service info and config */
	u8 event_condition; /* determine when publish related events are generated; nan_publish_event_condition_flags */
	u8 cipher_suite_id; /* nan_cipher_suite_id */
	u8 ipv6_info[MAX_IPV6_INFO_LEN];
	u8 responder_data_address[MAC_ALEN]; /* for unicast solicited publish */
	struct rtw_phl_nan_service_info data_spec_info; /* i.e. ndp specific info */

	/* out-of-spec variables */
	u32 config_flags;

	/* matching filter */
	u8 num_tx_filters;
	u8 num_rx_filters;
	struct rtw_phl_nan_func_filter rx_filters[MAX_MATCHING_FILTERS];
	struct rtw_phl_nan_func_filter tx_filters[MAX_MATCHING_FILTERS];

	struct rtw_phl_nan_sec_req sec_req;

	/*
	*	nan_qos_req =>
	*	Unicast: data_path_type = 0 qos = 1
	*	Multicast: data_path_type = 1 qos = 1
	*/
	struct rtw_phl_nan_qos_req qos_req;

	/* range_config_param not support yet */

	u64 cookie; /* unique NAN function identifier; implement aspect */
};

struct rtw_phl_nan_subscribe_info {
	bool subscribe_active;
	u8 subscribe_id; /* subscribe_instance_id */

	/* common service id and config for pub/sub */
	u8 service_id[NAN_SRVC_ID_LEN];
	u8 service_name[MAX_SERVICE_NAME_LEN];
	struct rtw_phl_nan_service_info srv_spec_info;
	u8 period; /* announcement_period for publish or query_period for subscribe */
	u8 awake_dw_int;
	bool close_range;
	u32 ttl;
	u16 sde_control; /* nan_service_descriptor_extension_controls */

	/* matching filter */
	u8 mf_flag; /* non-zero if matching_filter_tx!=matching_filter_rx, zero if matching_filter_tx == matching_filter_rx */
	struct rtw_phl_nan_func_filter rx_filters[MAX_MATCHING_FILTERS];
	struct rtw_phl_nan_func_filter tx_filters[MAX_MATCHING_FILTERS];
	u8 num_tx_filters;
	u8 num_rx_filters;

	/* service response filter */
	bool srf_include;
	u8 *srf_bf;
	u8 srf_bf_len;
	u8 srf_bf_idx;
	struct rtw_phl_nan_srf_mac_addrs *srf_macs; /* ptr of mac addr * srf_num_macs */
	int srf_num_macs;

	/* range_config_param not support yet */

	u64 cookie; /* unique NAN function identifier; implement aspect */
};

struct rtw_phl_nan_followup_info {
	u8 followup_id; /* followup_instance_id */

	/* req service info : instance id, dest */
	u8 followup_reqid;
	u8 followup_dest[MAC_ALEN];

	/* matched service info : publish or subscribe instance id, service id(hash) */
	u8 matched_instance_id;
	u8 matched_service_id[NAN_SRVC_ID_LEN];

	struct rtw_phl_nan_service_info srv_spec_info;

	u8 service_auto_followup_type; /* enum nan_auto_reply_followup_type */

	u64 cookie; /* unique NAN function identifier; implement aspect */
};

union rtw_phl_nan_srv_data {
	/* data */
	struct rtw_phl_nan_publish_info *p_pdata;
	struct rtw_phl_nan_subscribe_info *s_pdata;
	struct rtw_phl_nan_followup_info *f_pdata;
};

struct rtw_phl_nan_ndp_specific_info {
	/*
	 * NDP specific info can be conatined in NDP/NDPE
	 * The content can be filled in ndp specific info format for NDP or
	 * TLV list format for NDPE
	 */
	u8 *pbuf;
	u16 len;
};

struct rtw_phl_nan_dp_auto_rsp_info {
	u8 ndp_rsp_mode;
	u8 publish_id;
	u8 ndp_state;
	u8 ndp_type;
	u8 *ipv6_info;
	struct rtw_phl_nan_ndp_specific_info info;
};

struct rtw_phl_nan_srvc_instance_entry {
	_os_list list;
	u8 service_control;

	union rtw_phl_nan_srv_data u_srv_data;
	struct rtw_phl_nan_dp_auto_rsp_info ndp_auto_rsp_info;
};

struct rtw_phl_nan_srvc_ext_info {
	u8 updating;
	/* indicate when cmd srv_ext_info is called and
	 * these info need to update to the adding service in rtw_add_nan_func_xxx
	 */
	u8 period; /* (pub/sub) announcement_period for publish or query_period for subscribe */
	u8 event_condition; /* (pub) determine when publish related events are generated; nan_publish_event_condition_flags */
	u8 proximity_flag; /* (pub/sub)  discovery range; not implement */
	u8 awake_dw_int; /* (pub/sub)  Interval between two DWs during which the device supporting the service is awake to transmit or receive */
	u8 further_srvc_disc; /* (pub) more frames will come with this service */
	u8 further_srvc_disc_func; /* (pub) 0: followup, 1 : GAS */
	u8 data_path_flag; /* (pub) NDP setup is required */
	u8 data_path_type; /* (pub) 0: unicast */
	u8 qos; /* (pub) qos required */
	u8 sec; /* (pub) sec required */

	/*
	 * pmk_publish and pmkid_publish only use for publish
	 * and these two parameters follow specific service.
	 * pmkid_publish is calculated form pmk_publish and broadcast
	 * mac addr,and used in solicted/unsolicated publish SDF
	 */
	u8 pmk_publish[MAX_PMK_LEN];
	u8 pmkid_publish[MAX_PMKID_LEN];
	u8 gtk_protection;

	/*
	 * nan_qos_req =>
	 * Unicast: data_path_type = 0 qos = 1
	 * Multicast: data_path_type = 1 qos = 1
	 */
	struct rtw_phl_nan_qos_req qos_req;
	/* range_config_param not support yet */

	u8 service_id[NAN_SRVC_ID_LEN]; /* (pub/sub) overwrite service info feild */
	u16 csid; /* represent in bitwise */

	/* Compatible with NAN Utility v1.10 */
	u32 reserved_byte;
};

/* struct : report */

struct rtw_phl_nan_rpt_match_srv {
	u8 type;
	u8 my_inst_id;
	u8 peer_inst_id;
	u8 *addr;
	u16 info_len;
	u8 *info;
	u64 cookie;
};

/**
 * rtw_phl_nan_if_session_on -
 *
 * Starting nan feature takes places when user issues a command nan start,
 * which means that the correponsive adapter and network-related structions are ready.
 *
 * @phl_info: pointer of phl_info which contains nan_info
 */
u8 rtw_phl_nan_if_session_on(void *phl_info);

/**
 * rtw_phl_nan_if_session_init
 * @phl_info: pointer of phl_info which contains nan_info
 */
u8 rtw_phl_nan_if_session_init(void *phl_info);

/**
 * rtw_phl_nan_get_wrole_addr - get the addr of the binding wifi role
 * @phl_info: pointer of phl_info which contains nan_info
 */
u8 *rtw_phl_nan_get_wrole_addr(void *phl_info);

/**
 * rtw_phl_nan_set_addr - set addr from the binding wifi role
 * @phl_info: pointer of phl_info which contains nan_info
 * @mac_addr:
 */
void rtw_phl_nan_set_addr(void *phl_info, u8 *mac_addr);

/* nan module (func) : discovery engine */

void
rtw_phl_nan_free_service_data(void *drv, u8 service_type,
			      union rtw_phl_nan_srv_data p_srv_data);

void
rtw_phl_nan_free_service_entry(void *drv,
			       struct rtw_phl_nan_srvc_instance_entry *entry);
/* (ac) get_service_cookie_from_service_instance */
u64
phl_nan_get_srv_ck_from_entry(struct rtw_phl_nan_srvc_instance_entry *srvc_inst);

/**
 * rtw_phl_nan_init_ops
 * @phl: to access nan_info
 * @ops: nan ops from core
 */
void rtw_phl_nan_init_ops(void *phl, struct rtw_phl_nan_ops *ops);

/**
 * rtw_phl_nan_on_action_public_nan
 *  - Handle servcice discovery and NAN action frame
 * @phl: priv info from core layer
 * @precv_frame: rx frame head
 * @frame_body: rx frame body
 * @precv_frame_len: len of rx frame from the header
 * @is_privacy:
 * (ac) on_action_public_nan
 *
 * RETURN:
 * RTW_PHL_STATUS_SUCCESS: NAN function is working and is from the same cluster
 * RTW_PHL_STATUS_FAILURE: NAN function is not working or is not from the same cluster
 */
enum rtw_phl_status
rtw_phl_nan_on_action_public_nan(void* phl, u8 *precv_frame,
				 u8 *frame_body, u16 precv_frame_len,
				 u8 is_privacy);

/**
 * rtw_phl_nan_is_nan_frame - Check the received frame is NAN frame or not
 * @phl: priv info from core layer
 * @pframe: rx frame buf
 * (ac) is_nan_frame
 *
 * RETURN:
 * true: is NAN frame
 * false: not NAN frame
 */
u8 rtw_phl_nan_is_nan_frame(void *phl, u8 *pframe);

/**
 * rtw_phl_nan_replace_with_nan_mac
 *  - the dst of nan mgnt frames is nan nmi
 *  - replace with nan mac addr if this frame is heading to nan de
 * @phl: priv info from core layer
 * @frame_ptr: rx frame ptr
 * @dst: dst from rtw rx attribute
 * (ac) in-place in (core) wlanhdr_to_ethhdr
 *
 * RETURN:
 * true: relaced
 * false: skip
 */
u8 rtw_phl_nan_replace_with_nan_mac(void *phl, u8 *frame_ptr, u8 *dst);

/* nan api */

/**
 * rtw_phl_nan_set_srvc_ext_info
 * set external service info
 * @phl: priv info from core layer
 * @srvc_ext_info: external service info
 * (ac) in-place in (core) rtw_cfgvendor_set_nan_srvc_extinfo
 *
 * RETURN:
 * true: set success
 * false: set fail
 */
u8
rtw_phl_nan_set_srvc_ext_info(void *phl,
			      struct rtw_phl_nan_srvc_ext_info *srvc_ext_info);

enum rtw_phl_status
rtw_phl_cmd_add_nan_func(void *phl, u8 *instance_id, u8 srv_type,
			 union rtw_phl_nan_srv_data srv_data);

enum rtw_phl_status
rtw_phl_cmd_del_nan_func(void *phl, u64 cookie);

struct rtw_phl_nan_ss_param {
	struct rtw_wifi_role_t *phl_role;
	struct rtw_wifi_role_link_t *rlink;
	u32 token;

	/* NAN parameters from core layer */
	u8 start;
	u8 is_rmmod;
	u8 nan_cmd_type;
	u8 master_pref;
	u8 bands;

	/* NAN core layer handler */
	struct rtw_phl_nan_ss_ops *ops;
	void *priv;
};

enum rtw_phl_status
rtw_phl_cmd_nan_ss_req(void *phl, struct rtw_phl_nan_ss_param *param);

/**
 * rtw_phl_nan_stop_directly
 * call nan stop functions directly. use this function when driver can
 * not add a cmd (e.g., device is suprise removed)
 * @phl: priv info from core layer
 */
void
rtw_phl_nan_stop_directly(void *phl);

struct rtw_phl_nan_ss_ops {
	int (*nan_start_complete_notify)(struct rtw_phl_nan_ss_param *param);
	int (*nan_stop_prepare_notify)(struct rtw_phl_nan_ss_param *param);
	int (*nan_stop_complete_notify)(struct rtw_phl_nan_ss_param *param);
};

#endif /* CONFIG_PHL_NAN */
#endif /* _PHL_NAN_DEF_H_ */
