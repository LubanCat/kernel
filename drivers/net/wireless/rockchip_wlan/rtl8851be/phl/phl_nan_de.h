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
#ifndef _PHL_NAN_DE_H_
#define _PHL_NAN_DE_H_

#ifdef CONFIG_PHL_NAN
static u32 crc32_tab[] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
	0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
	0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
	0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
	0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
	0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
	0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
	0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
	0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
	0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
	0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
	0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
	0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
	0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
	0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
	0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
	0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
	0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
	0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
	0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
	0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
	0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

#define GET_PEER_SRVC_INFO_BY_SRVC_ID BIT(0)
#define GET_PEER_SRVC_INFO_BY_INST_ID BIT(1)

#ifndef HT_CAP_IE_LEN
#define HT_CAP_IE_LEN 26
#endif

#ifndef VHT_CAP_IE_LEN
#define VHT_CAP_IE_LEN 22
#endif

/* Field of Service Info */
#define SERVICE_INFO_FIELD_SPECIFIC_INFO_OFFSET	(4)
#define GET_SRVC_INFO_FIELD_PROTO_TYPE(srvc_info_start) \
	(*((srvc_info_start) + 3))

#define phl_nan_is_scf_type_pub(srvc_ctrl) \
	(((srvc_ctrl) & SCF_TYPE_MASK) == SCF_TYPE_PUBLISH)
#define phl_nan_is_scf_type_sub(srvc_ctrl) \
	(((srvc_ctrl) & SCF_TYPE_MASK) == SCF_TYPE_SUBSCRIBE)
#define phl_nan_is_scf_type_followup(srvc_ctrl) \
	(((srvc_ctrl) & SCF_TYPE_MASK) == SCF_TYPE_FOLLOWUP)

#define phl_nan_is_pub_srvc_inst(p_srvcentry) \
	phl_nan_is_scf_type_pub((p_srvcentry)->service_control)

#define phl_nan_is_sub_srvc_inst(p_srvcentry) \
	phl_nan_is_scf_type_sub((p_srvcentry)->service_control)

#define phl_nan_is_followup_srvc_inst(p_srvcentry) \
	phl_nan_is_scf_type_followup((p_srvcentry)->service_control)

#define phl_nan_is_peer_publish_srvc(p_peer_srvc) \
	phl_nan_is_scf_type_pub((p_peer_srvc)->srvc_ctrl)

#define phl_nan_is_peer_subscribe_srvc(p_peer_srvc) \
	phl_nan_is_scf_type_sub((p_peer_srvc)->srvc_ctrl)

#define phl_nan_is_peer_followup_srvc(p_peer_srvc) \
	phl_nan_is_scf_type_followup((p_peer_srvc)->srvc_ctrl)


#define phl_nan_get_followup_autoreply_type_from_entry(p_srvcentry) \
	(phl_nan_is_followup_srvc_inst(p_srvcentry) ? \
	((p_srvcentry)->u_srv_data.f_pdata->service_auto_followup_type):0)

#define phl_nan_is_start_with_wfa_oui(d, srvc_info) \
	(_os_mem_cmp(d, srvc_info, WFA_OUI, 3) == true)

/* SCF_TYPE_PUBLISH */

#define phl_nan_is_solicited_publish_info(p_data) (p_data->type & BIT0)

#define phl_nan_is_unsolicited_publish_info(p_data) (p_data->type & BIT1)

#define phl_nan_is_solicited_publish_srvc_instance(p_srvcentry) \
	(phl_nan_is_pub_srvc_inst(p_srvcentry) && \
	(phl_nan_is_solicited_publish_info((p_srvcentry)->u_srv_data.p_pdata)))

#define phl_nan_is_unsolicited_publish_srvc_instance(p_srvcentry) \
	(phl_nan_is_pub_srvc_inst(p_srvcentry) && \
	(phl_nan_is_unsolicited_publish_info((p_srvcentry)->u_srv_data.p_pdata)))

#define phl_nan_get_sde_control_from_srv_instance(p_srvcentry) \
	(phl_nan_is_pub_srvc_inst(p_srvcentry) ? \
	((p_srvcentry)->u_srv_data.p_pdata->sde_control) : 0 )

#define phl_nan_is_sec_publish_srv_instance(p_srvcentry) \
	(phl_nan_is_pub_srvc_inst(p_srvcentry) ? \
	(TEST_STATUS_FLAG((p_srvcentry)->u_srv_data.p_pdata->sde_control,\
			   NAN_SDE_CTRL_SEC_REQUIRED)) : 0)


/* SCF_TYPE_SUBSCRIBE */
#define phl_nan_is_active_subscribe_info(p_data) (p_data->subscribe_active)

#define phl_nan_is_active_subscribe_srvc_instance(p_srvcentry) \
	(phl_nan_is_sub_srvc_inst(p_srvcentry) && \
	phl_nan_is_active_subscribe_info(p_srvcentry->u_srv_data.s_pdata))

#define phl_nan_de_info_to_mac_info(de_info) \
	(&(phl_container_of(de_info, struct phl_nan_info,\
	 discovery_engine_priv)->sync_engine_priv.mac_info))

enum nan_publish_type {
	NAN_SOLICITED_PUBLISH = 1 << 0,
	NAN_UNSOLICITED_PUBLISH = 1 << 1,
};

enum nan_publish_event_condition_flags {
	NAN_PUBLISH_EVENT_CONDITION_SOLICITED_TX = 1,
};

enum nan_publish_config_flags {
	NAN_PUBLISH_CONFIG_FLAG_DISCOVERY_RANGE_LIMITED		   = (1 << 0),
	NAN_PUBLISH_CONFIG_FLAG_RANGING_REQUIRED		   = (1 << 1),
	NAN_PUBLISH_CONFIG_FLAG_ADD_DEVICE_CAPABILITY_ATTRIBUTE	   = (1 << 2),
	NAN_PUBLISH_CONFIG_FLAG_ADD_AVAILABILITY_ATTRIBUTE	   = (1 << 3),
	NAN_PUBLISH_CONFIG_FLAG_ENABLE_SERVICE_BEACON_CARRY	   = (1 << 4),
	NAN_PUBLISH_CONFIG_FLAG_ENABLE_SERVICE_AUTOMATIC_RESPONSE  = (1 << 5),
} ;

/* (ac) nan_service_descriptor_extension_controls */
enum nan_service_descriptor_extension_controls {
	NAN_SDE_CTRL_FSD_REQUIRED		= (1 << 0),
	NAN_SDE_CTRL_FSD_WITH_GAS		= (1 << 1),
	NAN_SDE_CTRL_DATAPATH_REQ_REQUIRED	= (1 << 2),
	NAN_SDE_CTRL_DATAPATH_TYPE		= (1 << 3),
	NAN_SDE_CTRL_MULTICAST_TYPE		= (1 << 4),
	NAN_SDE_CTRL_QOS_REQUIRED		= (1 << 5),
	NAN_SDE_CTRL_SEC_REQUIRED		= (1 << 6),
	NAN_SDE_CTRL_RANGING_REQUIRED		= (1 << 7),
	NAN_SDE_CTRL_RANGE_LIMIT_PRESENT	= (1 << 8),
	NAN_SDE_CTRL_SRVC_INFO_PRESENT		= (1 << 9),
};

enum nan_subscribe_flags {
	NAN_SUBSCRIBE_FLAG_DISCOVERY_RANGE_LIMITED	     = (1 << 0),
	NAN_SUBSCRIBE_FLAG_RANGING_REQUIRED		     = (1 << 1),
	NAN_SUBSCRIBE_EXCLUDE_MAC_ADDRESS_FILTER	     = (1 << 2),
	NAN_SUBSCRIBE_INCLUDE_BLOOM_FILTER		     = (1 << 3),
	NAN_SUBSCRIBE_FLAG_ADD_DEVICE_CAPABILITY_ATTRIBUTE   = (1 << 4),
	NAN_SUBSCRIBE_FLAG_ADD_AVAILABILITY_ATTRIBUTE	     = (1 << 5),
	NAN_SUBSCRIBE_FLAG_ENABLE_SERVICE_BEACON_CARRY	     = (1 << 6),
	NAN_SUBSCRIBE_FLAG_ENABLE_SERVICE_AUTOMATIC_RESPONSE = (1 << 7),
};

enum nan_auto_reply_followup_type {
	TYPE_NAN_AUTO_FOLLOWUP_MAC_ADDR	= 1,
	TYPE_NAN_AUTO_FOLLOWUP_IPV6	= 2,
	TYPE_NAN_AUTO_FOLLOWUP_TALK	= 3
};

enum nan_service_termination_reason
{
	NAN_SERVICE_TERMINATION_REASON_TIME_OUT       = 1,
	NAN_SERVICE_TERMINATION_REASON_USER_INITIATED = 2,
	NAN_SERVICE_TERMINATION_REASON_FAILURE        = 3,
	NAN_SERVICE_TERMINATION_REASON_CHIP_RESET     = 4,
	NAN_SERVICE_TERMINATION_REASON_NAN_DISABLE    = 5,
	NAN_SERVICE_TERMINATION_REASON_CHIP_REJECT    = 6,
};

#define DEFAULT_AWKAKE_DW_INT	1
#define DEFAULT_BACST_PERIOD	1

struct phl_nan_sdf_ctx {
	u8 peer_addr[MAC_ALEN];
	u8 instance_id;
	u8 requestor_id;
	u8 service_control;
	u16 binding_bitmap;
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry;
	void *p_peer_info_entry;
	void *p_peer_srvc_info_entry;
};

struct phl_nan_de_priv {
	struct phl_queue publish_srvc_queue;
	struct phl_queue subscribe_srvc_queue;
	struct phl_queue followup_srvc_queue;

	/* DO NOT change orders of declarations above,
	 * put additional members below */
	struct phl_queue auto_folloup_srvc_queue;
	u64 auto_followup_cookie;

	u32 srvc_num;
	u8 instant_id;
	u8 min_awake_dw_intvl; /* min intvl within all existing NAN services */
	u8 flag; /* refer to enum nan_de_priv_flags*/
	struct rtw_phl_nan_srvc_ext_info srvc_ext_info;
	/* a block mem contains more info before adding a service */
};

struct phl_nan_srvc_phyinfo {
	s8 publish_rssi;
	u8 publish_ch;
	s8 subscribe_rssi;
	u8 subscribe_ch;
	s8 followup_rssi;
	u8 followup_ch;
};

struct phl_nan_srvc_specific_info {
	u8 *pbuf;
	u16 len;
};

struct phl_nan_generic_service_info {
	u16 trans_port;
	u8 trans_protocol;
	u8 *service_name;
	u16 service_name_len;
	u8 name_of_srvc_inst;
};

struct phl_nan_peer_srvc_info_entry {
	_os_list list;
	u32 ref_cnt;
	u32 flag; /*( NDP & QoS)*/
	/* Service Descriptor Attribute */
	u8 service_id[NAN_SRVC_ID_LEN];
	u8 peer_inst_id;
	u8 srvc_ctrl;
	/* Service Descriptor Extension Attribute */
	u8 srvc_update_indicator;

	/* Phy Info from SDF */
	struct phl_nan_srvc_phyinfo srvc_phyinfo;

	struct phl_nan_srvc_specific_info followup_srvc_info;
	struct phl_nan_srvc_specific_info srvc_info;
	u64 assoc_srvc_inst_ck; /* associated srvc_instance_entry's cookie*/
	/*Service Descriptor Extension Attribute */
	u16 sdea_ctrl;

	/* Generic Service Protocol in SDEA/NDPE */
	struct phl_nan_generic_service_info gen_srvc_info;
};

struct phl_nan_de_send_sdf_cmd {
	u8 peer_info_mac[MAC_ALEN];
	void *p_peer_srvc_info_entry;
	u8 auto_followup_serv_inst;
	u64 serv_inst_ck;
};

/* void _phl_nan_dump_func(void *sel, _adapter *padapter);

u8
_phl_nan_insert_send_mgnt_frame_entry(struct rtw_nan_priv *pnanpriv,
				      struct xmit_frame *pxmitframe,
				      struct nan_peer_info_entry* ppeer_entry,
				      u8 is_xmit_now); */

void phl_nan_init_de_priv(struct phl_info_t *phl_info,
			  struct phl_nan_de_priv *pdiscovery_engine_priv);

/**
 * phl_nan_deinit_de_priv - Free NAN discovery engine private structure
 * @phl_info: phl info
 * @de_info: nan discovery engine private structure
 * (ac) _rtw_free_nan_de_priv
 */
void
phl_nan_deinit_de_priv(struct phl_info_t *phl_info, struct phl_nan_de_priv *de_info);

/**
 * phl_nan_free_srvc_inst_q - Free NAN discovery engine service function instance queue
 * @phl_info: phl info
 * @de_info: nan discovery engine private structure
 * @srvc_q: nan service function instance queue
 * (ac) _rtw_free_nan_srvc_instance_queue
 */
void
phl_nan_free_srvc_inst_q(struct phl_info_t *phl_info,
			 struct phl_nan_de_priv *de_info,
			 struct phl_queue *srvc_q);

/**
 * phl_nan_alloc_peer_srvc_entry
 *  - Allocate a peer service info entry and insert to peer service info queue.
 * @psrvc_info_queue: peer service info queue
 * @p_peer_srvc_info: pointer of the allocated peer service info entry
 * (ac) _rtw_alloc_nan_peer_srvc_info_entry
 *
 * RETURN:
 * true: Allocate peer service entry info success
 * false: Allocate peer service entry info fail
 */
u8
phl_nan_alloc_peer_srvc_entry(void *d,
			      struct phl_queue *srvc_info_q,
			      struct phl_nan_peer_srvc_info_entry **p_peer_srvc_info);

/**
 * phl_nan_free_peer_srvc_queue
 *  - Free all peer service info entry from the queue.
 * @d:
 * @p_peer_srvc_info_queue: peer service info queue
 * (ac) _rtw_free_nan_peer_srvc_info_queue
 *
 */
void phl_nan_free_peer_srvc_queue(void *d, struct phl_queue *srvc_info_q);

/*
 * phl_nan_find_srvc_inst_entry - Find local service function with search type
 * if return true, driver must call phl_nan_done_srvc_inst_entry() to unlock queue
 * @de_info: discovery engine priv
 * @pdata: search key
 * @stype: search type
 * @srv_inst: retuen service entry if exist
 * (ac) rtw_nan_find_srvc_instance_entry
 *
 * RETURN:
 * true: Service is exist.
 * false: Service is not exist.
 */
u8
phl_nan_find_srvc_inst_entry(void *d,
			     struct phl_nan_de_priv *de_info,
			     void *pdata,
			     u8 stype,
			     struct rtw_phl_nan_srvc_instance_entry **srv_inst);

void
phl_nan_done_srvc_inst_entry(void *d,
			     struct phl_nan_de_priv *de_info,
			     struct rtw_phl_nan_srvc_instance_entry *srv_inst);

/**
 * phl_nan_phl_nan_issue_nan_sdf - issue an NAN Service Discovery frame
 * @de_info
 * @psdf_ctx: sdf context structure
 * @wait_ack: used xmit ack
 * (ac) issue_nan_sdf
 *
 * Returns:
 * _SUCCESS: No xmit ack is used or acked
 * _FAIL: not acked when using xmit ack or alloc_mgtxmitframe fail
 */
u8 phl_nan_issue_nan_sdf(struct phl_nan_de_priv *de_info,
			 struct phl_nan_sdf_ctx *psdf_ctx, int wait_ack);

/**
 * phl_nan_send_bcast_sdf
 *  - ff
 * @de_info: priv data of de
 * (ac) _rtw_nan_send_bcast_sdf
 */
void phl_nan_send_bcast_sdf(void *d, struct phl_nan_de_priv *de_info);

/**
 * phl_nan_de_on_sdf - Handle the NAN service discovery frame
 * @nan_info: nan priv
 * @precv_frame: received sdf
 * @len: len of frame body
 * (ac) _rtw_nan_de_on_sdf
 */
void phl_nan_de_on_sdf(struct phl_nan_de_priv *de_info, u8 *recv_frame, u16 len);

/**
 * phl_nan_de_dw_timeout_hdl
 *  - decide if de issue bcast sdf according to min_awake_dw_intvl
 * @de_info: priv data of de
 * @tsf: current tsf of nan wrole's hw port
 * @nan_dw_idx: the index of current DW from previous DW0
 * (ac) rtw_nan_de_dw_timeout_handler
 *
 * RETURN:
 * true: issue bcast sdf
 * false: skip
 */
u8 phl_nan_de_dw_timeout_hdl(void *d, struct phl_nan_de_priv *de_info,
				 u32 tsf, u32 dw_idx);

u16 phl_nan_append_cipher_suite_attr(void *d, u8* pframe,
				     struct phl_nan_de_priv *de_info,
				     struct phl_nan_sdf_ctx *pdata);

u16
phl_nan_de_append_sec_ctx_info_attr(struct phl_nan_de_priv *de_info,
				    u8 *pframe,
				    struct phl_nan_sdf_ctx *pdata);

void phl_nan_disc_engine_evt_hdl(struct phl_nan_de_priv *de_info,
					    u32 type,void* pbuf);

/**
 * phl_nan_peer_srvc_info_exist - Check the service id	of sdf is exist in peer service list or not
 * @d: for mem ops
 * @p_peer_info_entry: peer info of the received sdf
 * @pdata: data for search
 * @search_type:
 * @p_peer_srvc_info_entry: return the peer service info entry if peer service exist
 * (ac) is_nan_peer_srvc_info_exist
 *
 * RETURN:
 * true: Peer service exist.
 * false: Peer service not exist.
 */
u8 phl_nan_peer_srvc_info_exist(
	void *d, void *p_peer_info_entry, u8 *pdata, u8 search_type,
	struct phl_nan_peer_srvc_info_entry **p_peer_srvc_info_entry);

/**
 * phl_nan_auto_followup_free_mem - Free mem allocated for auto follow-up
 *	Unlike the mem of nan func from iw cmd(add_func/del_func),
 *	which is took care via followup_srvc_queue,
 *	these mem for auto followup needs to be freed in different cases.
 * @de_info: priv data of de
 * @srvc_inst: service instance entry that contain auto followup data
 * @need_dequeue: indicates if the service instance entry needs to be dequeue
 * (ac) _rtw_nan_auto_followup_free_mem
 *
 */
void
phl_nan_auto_followup_free_mem(struct phl_nan_de_priv *de_info,
			       struct rtw_phl_nan_srvc_instance_entry *srvc_inst,
			       u8 need_dequeue);

void phl_nan_update_service_ext_info_from_de(
	void *d,
	struct phl_nan_de_priv *de_info,
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry);

/* api for struct rtw_phl_nan_srvc_instance_entry_ */
struct rtw_phl_nan_publish_info*
phl_nan_get_pub_from_entry(
	struct rtw_phl_nan_srvc_instance_entry* p_srvc_entry);

struct rtw_phl_nan_subscribe_info*
phl_nan_get_sub_from_entry(
	struct rtw_phl_nan_srvc_instance_entry* p_srvc_entry);

struct rtw_phl_nan_followup_info*
phl_nan_get_followup_from_entry(
	struct rtw_phl_nan_srvc_instance_entry* p_srvc_entry);

/* (ac) get_service_id_from_service_instance */
u8 *phl_nan_get_srv_id_from_entry(struct rtw_phl_nan_srvc_instance_entry *srvc_inst);

u32 phl_nan_get_ttl_from_entry(
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry);

/* (ac) get_instance_id_from_service_instance */
u8
phl_nan_get_inid_from_entry(struct rtw_phl_nan_srvc_instance_entry *srvc_inst);

struct rtw_phl_nan_func_filter * phl_nan_get_tx_mf_from_entry(
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry,
	u8 *num_tx_mf);

struct rtw_phl_nan_service_info * phl_nan_get_si_from_entry(
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry);

u8 phl_nan_get_adw_int_from_entry(
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry);

u8 phl_nan_chk_pub_solicited_tx_for_srvc_inst(
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry,
	u8 tx_type);

void _rtw_remove_srvc_instance_data(
	void *d,
	struct phl_nan_de_priv *de_info,
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry);

u8 phl_nan_check_srv_type(
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry,
	u8 type);

enum rtw_phl_status
phl_add_nan_func_evt_hdl(struct phl_info_t *phl_info, void *pbuf);

enum rtw_phl_status
phl_del_nan_func_evt_hdl(struct phl_info_t *phl_info, void *pbuf);

/**
 * phl_nan_de_set_default_gen_srv_info
 * @d:
 * @srv_info:
 * (ac) rtw_nan_de_set_default_gen_srv_info
 *
 * RETURN:
 * true:
 * false:
 */
u16 phl_nan_de_set_default_gen_srv_info(void *d, u8 *srv_info);

void
phl_nan_free_all_auto_flup_srv_inst(struct phl_info_t *phl_info);

/**
 * phl_nan_add_peer_srvc_check - Check the received sdf should be added or not
 * @de_info:
 * @pmatch_sda: sda of sdf with match service id
 * @p_srvc_entry: own service function entry with same service id
 * @ppeer_addr: mac address of sdf
 * @precv_frame: rx frame
 * (ac) _rtw_nan_add_peer_srvc_check
 *
 * RETURN:
 * true: Service is added to peer service info.
 * false: Service is not added to peer service info.
 */
u8 phl_nan_add_peer_srvc_check(
	struct phl_nan_de_priv *de_info, u8 *pmatch_sda,
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry,
	u8 *ppeer_addr, u8 *precv_frame, u16 frame_len);
#endif /* CONFIG_PHL_NAN */
#endif
