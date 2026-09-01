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
#ifndef _PHL_NAN_H_
#define _PHL_NAN_H_

#ifdef CONFIG_PHL_NAN

#include "phl_nan_de.h"

#define TYPE_BEACON_FRAME 0x80
#define cap_ShortPremble BIT(5)
#define cap_ShortSlot BIT(10)

#define DEFAULT_MAP_ID (0)
#define MAX_NDI_NUM (2)

#define MAX_SRVC_INSTANCE_TYPE (3)
#define FIRST_SRVC_INSTANCE_ID (1)
#define DEFAULT_NAN_24G_CH (6)
#define DEFAULT_NAN_5G_CH (149)
#define FW_TIME_INFO_INDICATION_PERIOD (4)

#define NAN_ATTR_CONTENT_OFFSET (3)

/* nan-spec-related : constants & attributes */

#define MAX_NAN_SYNC_BCN_LEN (128)
#define MAX_NAN_DISC_BCN_LEN (350)

#define MAX_DEV_CAP_ATTR_LEN (12)

#define MAX_CHANNEL_SWITCH_TIME (2*TU)

/* Field Control of Service Descriptor Extension attribute */
#define SERVICE_DESC_EXT_FIELD_START_OFFSET		(3)
#define SERVICE_DESC_EXT_FIELD_CONTROL_OFFSET		(4)
#define SERVICE_DESC_EXT_FIELD_INSTANCE_ID_LEN		(1)
#define SERVICE_DESC_EXT_FIELD_CONTROL_LEN		(2)
#define SERVICE_DESC_EXT_FIELD_RANGE_LIMIT_LEN		(4)
#define SERVICE_DESC_EXT_FIELD_SERVICE_UPDATE_LEN	(1)
#define SERVICE_DESC_EXT_FIELD_SERVICE_INFO_LEN		(2)

#define BIT_SDEA_CONTROL_RANGE_LIMIT_PRESENT		(8)
#define BIT_SDEA_CONTROL_SRVC_UPDATE_INDICATOR_PRESENT	(9)

/* Macro to configure Service Descriptor Extension attribute */
#define SET_NAN_SERVICE_DESC_EXT_ELE_INSTANCE_ID(_p_ele_start, _val) \
	SET_BITS_TO_LE_1BYTE(_p_ele_start, 0, 8, _val)
#define SET_NAN_SERVICE_DESC_EXT_ELE_CONTROL(_p_ele_start, _val) \
	SET_BITS_TO_LE_2BYTE(_p_ele_start + 1, 0, 16, _val)

/* Field Control of Connection Capability attribute */
#define CONNECTION_CAP_FIELD_START_OFFSET	(3)
#define CONNECTION_CAP_FIELD_BITMAP_LEN		(2)

/* Macro to configure Connection Capability attribute */
#define SET_NAN_CONNECTION_CAP_ELE_WIFI_DIRECT(_p_ele_start, _val) \
	SET_BITS_TO_LE_2BYTE(_p_ele_start, 0, 1, _val)
#define SET_NAN_CONNECTION_CAP_ELE_P2PS(_p_ele_start, _val) \
	SET_BITS_TO_LE_2BYTE(_p_ele_start, 1, 1, _val)
#define SET_NAN_CONNECTION_CAP_ELE_TDLS(_p_ele_start, _val) \
	SET_BITS_TO_LE_2BYTE(_p_ele_start, 2, 1, _val)
#define SET_NAN_CONNECTION_CAP_ELE_INFRA(_p_ele_start, _val) \
	SET_BITS_TO_LE_2BYTE(_p_ele_start, 3, 1, _val)
#define SET_NAN_CONNECTION_CAP_ELE_IBSS(_p_ele_start, _val) \
	SET_BITS_TO_LE_2BYTE(_p_ele_start, 4, 1, _val)
#define SET_NAN_CONNECTION_CAP_ELE_MESH(_p_ele_start, _val) \
	SET_BITS_TO_LE_2BYTE(_p_ele_start, 5, 1, _val)

/* Field Control of Device Capability attribute */
#define DEVICE_CAP_FIELD_START_OFFSET		(3)
#define DEVICE_CAP_MAPID_LEN			(1)
#define DEVICE_CAP_COMMITTED_DW_INFO_LEN	(2)
#define DEVICE_CAP_SUPPORT_BANDS_LEN		(1)
#define DEVICE_CAP_OPERATION_MODE_LEN		(1)
#define DEVICE_CAP_NUMBER_OF_ANTENNAS_LEN	(1)
#define DEVICE_CAP_MAX_CHANNEL_SWITCH_TIME_LEN	(2)
#define DEVICE_CAP_CAPABILITIES_LEN		(1)

/* Macro to configure Device Capability attribute */
#define SET_NAN_DEVICE_CAP_ELE_MAPID(_p_ele_start, _val) \
	SET_BITS_TO_LE_1BYTE(_p_ele_start, 0, 5, _val)
#define SET_NAN_DEVICE_CAP_ELE_24G_DW(_p_ele_start, _val) \
	SET_BITS_TO_LE_2BYTE(_p_ele_start + 1, 0, 3, _val)
#define SET_NAN_DEVICE_CAP_ELE_5G_DW(_p_ele_start, _val) \
	SET_BITS_TO_LE_2BYTE(_p_ele_start + 1, 3, 3, _val)
#define SET_NAN_DEVICE_CAP_ELE_24G_DW_OVERWRITE(_p_ele_start, _val) \
	SET_BITS_TO_LE_2BYTE(_p_ele_start + 1, 6, 4, _val)
#define SET_NAN_DEVICE_CAP_ELE_5G_DW_OVERWRITE(_p_ele_start, _val) \
	SET_BITS_TO_LE_2BYTE(_p_ele_start + 1, 10, 4, _val)
#define SET_NAN_DEVICE_CAP_ELE_SUPPORT_BAND_24G(_p_ele_start, _val) \
	SET_BITS_TO_LE_1BYTE(_p_ele_start + 3, 2, 1, _val)
#define SET_NAN_DEVICE_CAP_ELE_SUPPORT_BAND_5G(_p_ele_start, _val) \
	SET_BITS_TO_LE_1BYTE(_p_ele_start + 3, 4, 1, _val)
#define SET_NAN_DEVICE_CAP_ELE_OPERATION_PHY_MODE(_p_ele_start, _val) \
	SET_BITS_TO_LE_1BYTE(_p_ele_start + 4, 0, 1, _val)
#define SET_NAN_DEVICE_CAP_ELE_OPERATION_VHT_160NC(_p_ele_start, _val) \
	SET_BITS_TO_LE_1BYTE(_p_ele_start + 4, 1, 1, _val)
#define SET_NAN_DEVICE_CAP_ELE_OPERATION_VHT_160C(_p_ele_start, _val) \
	SET_BITS_TO_LE_1BYTE(_p_ele_start + 4, 2, 1, _val)
#define SET_NAN_DEVICE_CAP_ELE_ANTENNA_NUM_TX(_p_ele_start, _val) \
	SET_BITS_TO_LE_1BYTE(_p_ele_start + 5, 0, 4, _val)
#define SET_NAN_DEVICE_CAP_ELE_ANTENNA_NUM_RX(_p_ele_start, _val) \
	SET_BITS_TO_LE_1BYTE(_p_ele_start + 5, 4, 4, _val)
#define SET_NAN_DEVICE_CAP_ELE_MAX_CHANNEL_SWITCH_TIME(_p_ele_start, _val) \
	SET_BITS_TO_LE_2BYTE(_p_ele_start + 6, 0, 16, _val)
#define SET_NAN_DEVICE_CAP_ELE_CAPABILITIES(_p_ele_start, _val) \
	SET_BITS_TO_LE_1BYTE(_p_ele_start + 8, 0, 8, _val)

#define SET_NDPE_SUPPORT_IN_CAPABILITIES(_cap, _val) \
	((_cap) = ((_cap)&0xf7) | ((_val) << 3))

#define GET_NAN_DEVICE_CAP_ELE_24G_DW(_p_ele_start) \
	LE_BITS_TO_2BYTE(_p_ele_start + 1, 0, 3)
#define GET_NAN_DEVICE_CAP_ELE_5G_DW(_p_ele_start) \
	LE_BITS_TO_2BYTE(_p_ele_start + 1, 3, 3)
#define GET_NAN_DEVICE_CAP_ELE_24G_DW_OVERWRITE(_p_ele_start) \
	LE_BITS_TO_2BYTE(_p_ele_start + 1, 6, 4)
#define GET_NAN_DEVICE_CAP_ELE_5G_DW_OVERWRITE(_p_ele_start) \
	LE_BITS_TO_2BYTE(_p_ele_start + 1, 10, 4)
#define GET_NAN_DEVICE_CAP_ELE_SUPPORT_BAND_5G(_p_ele_start) \
	LE_BITS_TO_2BYTE(_p_ele_start + 3, 4, 1)
#define GET_NAN_DEVICE_CAP_ELE_MAX_CHANNEL_SWITCH_TIME(_p_ele_start) \
	LE_BITS_TO_2BYTE(_p_ele_start + 6, 0, 16)

/* Field offset of service descriptor attribute */
#define SDA_FIELD_START_OFFSET			(3)  /* Would be SDA_SERVICE_ID_OFFSET */
#define SDA_SERVICE_ID_OFFSET			(3)
#define SDA_INSTANCE_ID_OFFSET			(9)
#define SDA_REQUESTOR_INSTANCE_ID_OFFSET	(10)
#define SDA_SERVICE_CONTROL_OFFSET		(11)
#define SDA_OPTIONAL_FIELD_OFFSET		(12)
#define SDA_INSTANCE_ID_LEN			(1)
#define SDA_REQUESTOR_INSTANCE_ID_LEN		(1)
#define SDA_SERVICE_CONTROL_LEN			(1)
#define SDA_BINDING_BITMAP_LEN			(2)

/* Service control field (SCF) of service descriptor attribute */
#define SCF_TYPE_MASK		0x3
#define SCF_TYPE_PUBLISH	0x0
#define SCF_TYPE_SUBSCRIBE	0x1
#define SCF_TYPE_FOLLOWUP	0x2
#define SCF_TYPE_RSVD		0x3
#define BIT_SCF_MF_PRESENT	BIT(2)	/* Matching filter present */
#define BIT_SCF_SRF_PRESENT	BIT(3)	/* Service response filter present */
#define BIT_SCF_SI_PRESENT	BIT(4)	/* Service info present */
#define BIT_SCF_DRL		BIT(5)	/* Discovery Range Limited */
#define BIT_SCF_BB_PRESENT	BIT(6)	/* Binding bitmap present */
#define BIT_SCF_RSVD		BIT(7)	/* Reserved */

/* Service response filter (SRF) control field */
#define BIT_SRF_CTRL_TYPE	BIT(0)
#define BIT_SRF_CTRL_INCLUDE	BIT(1)
#define SRF_CTRL_BFI_MASK	0xC
#define SRF_CTRL_BFI_SHIFT	(2)

/* NAN IE Element ID field 1 byte +
 * Length filed 1 byte +
 * Maximum Length 256
 */
#define MAX_NAN_IE_LEN		(256+2)
#define NAN_IE_FIXED_LEN	(6)
#define NAN_IE_ID_OFFSET	(0)
#define NAN_IE_LEN_OFFSET	(1)
#define NAN_IE_OUI_OFFSET	(2)
#define NAN_IE_OUI_TYPE_OFFSET	(5)
#define NAN_IE_ATTR_OFFSET	(6)

#define MAX_SDF_LEN		300

/* Peer srvc info flags */
#define PEER_SRVC_INFO_FLAG_VALID BIT(0)

#define phl_nan_get_nmi(nan_info) (nan_info->sync_engine_priv.mac_info.nmi.mac)

enum _nan_srvc_type {
	NAN_SRVC_TYPE_NONE			= 0x0,
	NAN_SRVC_TYPE_UNSOLICITED_PUBLISH	= 0x1,
	NAN_SRVC_TYPE_SOLICITED_PUBLISH		= 0x2,
	NAN_SRVC_TYPE_ACTIVE_SUBSCRIBE		= 0x3,
	NAN_SRVC_TYPE_PASSIVE_SUBSCRIBE		= 0x4,
	NAN_SRVC_TYPE_FOLLOW_UP			= 0x5,
};

enum _nan_attr_id {
	NAN_ATTR_ID_MASTER_INDICATION			=	0x0,
	NAN_ATTR_ID_CLUSTER				=	0x1,
	NAN_ATTR_ID_SERVICE_ID_LIST			=	0x2,
	NAN_ATTR_ID_SERVICE_DESCRIPTOR			=	0x3,
	NAN_ATTR_ID_NAN_CONNECTION_CAPABILITY		=	0x4,
	NAN_ATTR_ID_WLAN_INFRASTRUCTURE			=	0x5,
	NAN_ATTR_ID_P2P_OPERATION			=	0x6,
	NAN_ATTR_ID_IBSS				=	0x7,
	NAN_ATTR_ID_MESH				=	0x8,
	NAN_ATTR_ID_FURTHER_NAN_SERVICE_DISCOVERY	=	0x9,
	NAN_ATTR_ID_FURTHER_AVAILABILITY_MAP		=	0xA,
	NAN_ATTR_ID_COUNTRY_CODE			=	0xB,
	NAN_ATTR_ID_RANGING				=	0xC,
	NAN_ATTR_ID_CLUSTER_DISCOVERY			=	0xD,
	NAN_ATTR_ID_SRV_DES_EXT				=	0xE,
	NAN_ATTR_ID_DEVICE_CAPABILITY			=	0xF,
	NAN_ATTR_ID_NDP					=	0x10,
	NAN_ATTR_ID_NAN_AVAIL				=	0x12,
	NAN_ATTR_ID_NDC					=	0x13,
	NAN_ATTR_ID_NDL					=	0x14,
	NAN_ATTR_ID_NDL_QOS				=	0x15,
	NAN_ATTR_ID_UNALIGNED_SCHEDULE			=	0x17,
	NAN_ATTR_ID_RANGING_INFORMATION			=	0x1A,
	NAN_ATTR_ID_RANGING_SETUP			=	0x1B,
	NAN_ATTR_ID_FTM_RANGING_EPORT			=	0x1C,
	NAN_ATTR_ID_ELEMENT_CONTAINER			=	0x1D,
	NAN_ATTR_ID_EXTENDED_WLAN_INFRASTRUCTURE	=	0x1E,
	NAN_ATTR_ID_EXTENDED_P2P_OPERATION		=	0x1F,
	NAN_ATTR_ID_EXTENDED_IBSS			=	0x20,
	NAN_ATTR_ID_EXTENDED_MESH			=	0x21,
	NAN_ATTR_ID_CIPHER_SUITE_INFO			=	0x22,
	NAN_ATTR_ID_SECURITY_CONTEXT_INFO		=	0x23,
	NAN_ATTR_ID_SHARED_KEY_DESCRIPTOR		=	0x24,
	NAN_ATTR_ID_PUBLIC_AVAILABILITY			=	0x27,
	NAN_ATTR_ID_SUBSCRIBE_SERVICE_ID_LIST		=	0x28,
	NAN_ATTR_ID_NDP_EXTENSION			=	0x29,
	NAN_ATTR_ID_VECDOR_SPECIFIC			=	0xDD,
	NAN_ATTR_MAX,
};

enum _nan_srvc_protocol_type {
	NAN_SRVC_PROTOCOL_BONJOUR	= 1,
	NAN_SRVC_PROTOCOL_GENIC		= 2,
};

enum _nan_generic_srvc_sub_attr_id {
	NAN_GEN_SRVC_TRANSPORT_PORT		= 0x0,
	NAN_GEN_SRVC_TRANSPORT_PROTOCOL		= 0x1,
	NAN_GEN_SRVC_SERVICE_NAME		= 0x2,
	NAN_GEN_SRVC_NAME_OF_SRVC_INST		= 0x3,
	NAN_GEN_SRVC_TEXT_INFO			= 0x4,
	NAN_GEN_SRVC_UUID			= 0x5,
	NAN_GEN_SRVC_BLOB			= 0x6,
	NAN_GEN_SRVC_RESERVD			= 0x7,
	NAN_GEN_SRVC_VENDOR_SPEC_INFO		= 0xDD,
};

/* (ac) enum nan_priv_flags */
enum _nan_priv_flags{
	/* NAN priv flags */
	NAN_PRIV_FLAG_NEW_CLUS_JOINED	= BIT(0),
	NAN_PRIV_FLAG_RMMOD		= BIT(1),
	NAN_PRIV_FLAG_NAN_SESSION_INIT	= BIT(2),
	NAN_PRIV_FLAG_NAN_SESSION_START	= BIT(3),
};

enum _nan_engine_type{
	NAN_EN_SYNC = 0,
	NAN_EN_DISC = 1,
};

enum _nan_mac_info_flags{
	NAN_MAC_FLAG_ALLOW_JOIN_NEW_CLUSTER	= BIT(0),
	NAN_MAC_FLAG_VHT_ENABLE			= BIT(1),
};

enum _nan_event_type {
	NAN_EVENT_NONE,
	NAN_EVENT_NEXT_DW,
	NAN_EVENT_JOIN_NEW_CLUSTER,
	NAN_EVENT_CLUSTER_JOIN_DONE,
	NAN_EVENT_SEND_SDF,
	NAN_EVENT_ALLOC_PEER_NMI_STA_INFO,
	NAN_EVENT_RMV_PEER_NMI_STA_INFO,
	NAN_EVENT_RPRT_MATCH_EVT,
	NAN_EVENT_ADD_NAN_FUNC,
	NAN_EVENT_DEL_NAN_FUNC,
};

#ifndef TU
#define TU 1024 /* 1 TU equals 1024 microseconds */
#endif
#define NAN_SLOT (16)
#define DW_INTERVAL (512)
#define DW0_INTERVAL (8192)
#define DW_5G_OFFSET (128)
#define SW_DW_TIMER_OFFSET (3 * NAN_SLOT)
#define SW_DW_TIMER_DEVIATION (10)

struct phl_nan_mac_addr_info {
	u8 mac[MAC_ALEN];
	_os_lock bssid_lock;
	u8 bssid[MAC_ALEN];
};

#define NAN_MAX_AVAIL_TIME_BITMAP_LEN 64
struct phl_nan_avail_t_bitmap {
	u16 start_offset_bit; /* unit: bits */
	u16 period_bit; /* unit: bits */
	u8 bit_duration_tu; /* unit: tu */
	u8 time_bitmap_len;
	u8 time_bitmap[NAN_MAX_AVAIL_TIME_BITMAP_LEN];
};

struct phl_nan_bcn {
	u16 sync_bcn_intvl_ms;
	u16 disc_bcn_intvl_ms;
	struct phl_nan_avail_t_bitmap avail_t_bitmap;
	struct rtw_bcn_info_cmn nan_sync_bcn;
	struct rtw_bcn_info_cmn nan_disc_bcn;
};

struct phl_nan_tsf_info {
	u32 port_dwst_low; /* DW start time by dedicated HW port TSF */
	u32 fr_dwst_low; /* DW start time by free run counter TSF */
};

struct phl_nan_master_indication {
	u8 master_pref;
	u8 random_factor;
};

struct phl_nan_anchor_master_info {
	u64 amr;
	u8 hop_count;
	u32 ambtt;
};

struct phl_nan_mac_info {
	struct phl_nan_mac_addr_info nmi; /* settings for NAN Management Interface */
	u16 sync_bcn_mac_id; /* macid for sync beacon transmission */
	u16 disc_bcn_mac_id; /* macid for disc beacon transmission */
	u16 mgn_mac_id; /* macid for mgnt transmission */
	struct phl_nan_tsf_info tsf_info;
};

struct phl_nan_committed_dw_info {
	u16 dw_24g : 3;
	u16 dw_5g : 3;
	u16 dw_overwrite_24g : 4;
	u16 dw_overwrite_5g : 4;
	u16 reserved : 2;
	u8 apply_to_all_map;
};

struct phl_nan_peer_info_entry {
	_os_list list;
	u8 mac[MAC_ALEN]; /* Peer's NMI for data link setup & protocol handshakes */
	u8 devcap_attr[MAX_DEV_CAP_ATTR_LEN]; /* preserve device capability attribute with MAP ID = 8b'0*/
	u8 ht_cap[HT_CAP_IE_LEN]; /* HT Cap */
	u8 vht_cap[VHT_CAP_IE_LEN]; /* VHT Cap */
	struct phl_nan_committed_dw_info cmt_dw_info; /* Committed DW Info extract from Device Cap Attribute */
	u32 flag; /* enum nan_peer_info_flags */
	u8 reason_code; /* used when rejecting schedule, enum _nan_reason_code */
	struct phl_queue srvc_info_queue; /* list is composed of service information received from peer */
};

struct phl_nan_sync_priv {
	struct phl_nan_mac_info mac_info;
	struct phl_nan_bcn bcn_info; /* NAN Beacon */
	struct phl_nan_master_indication master_indication;
	struct phl_nan_anchor_master_info am_info;
	u8 dual_band_support; /* enum band_type */
	u8 op_ch_24g;
	u8 op_ch_5g;
	u8 b_dis_24g_disc_bcn;
	u8 b_dis_5g_disc_bcn;
	u8 cmt_dw_peri_24G; /* unit: NAN_SLOT (16tu) */
	u8 cmt_dw_peri_5G; /* unit: NAN_SLOT (16tu) */
	_os_timer dw_timer;
	u32 flag; /*refer to enum nan_mac_info_flags*/
	u32 start_stop_cmd_token;
};

struct phl_nan_mgnt_frame_entry {
	_os_list list;
	enum rtw_phl_nan_pkt_type pkt_type;
	u8 *nan_attr_buf;
	u16 attr_len;
	u8 peer_addr[MAC_ALEN];
};

/**
 * struct phl_nan_info - NAN feature description
 *
 * @nan_wrole: pointer of binding wifi role
 * @wrole_addr: indicate the addr of the binding wifi role
 *
 * @mac_id_bitmap:
 * @flag: enum nan_priv_flags
 * @cmd_type: enum rtw_phl_nan_cmd_type
 *
 * @peer_info_list_queue: peers collection
 * Notice that driver only see peers within the same cluster as it belongs to
 * @mgnt_frame_queue: nan mgnt frame collection
 * nan mgnt frames are constucted and sent from driver now; such as SDF, NAF.
 * this queue contains nan mgnt frames which waiting for DW to issue
 *
 * NAN engines
 *
 * @sync_engine_priv: private data for syncengine;
 * the NAN sync engine locates in FW,
 * so most of these are parameters to nan h2c commands
 * @discovery_engine_priv:
 */
struct phl_nan_info {
	/* nan device priv */
	struct rtw_wifi_role_t *nan_wrole;
	struct rtw_wifi_role_link_t *nan_rlink;
	/*
	 * wifi role is free at very early stage in rtw_dev_remove flow
	 * (means no nan stop, nan interface del in front), so nan_info save the
	 * addr to access the right adapter and finish nan deinit process.
	 */
	u8 wrole_addr[MAC_ALEN];
	/*
	 * To control the tx timing of nan sync bcn, nan disc bcn and mgnt
	 * frames, fw need uses 3 macids to identify mac roles.
	 * Allocate 2 stainfo for nan disc bcn and nan mgnt frame.
	 */
	struct rtw_phl_stainfo_t *nan_disc_bcn_sta;
	struct rtw_phl_stainfo_t *nan_mgnt_sta;

	/* nan session */
	u8 cmd_type; /* enum nan_cmd_type */
	u16 mac_id_bitmap;
	u32 flag;
	struct phl_queue peer_info_queue;
	struct phl_queue mgnt_frame_queue;
	struct rtw_phl_nan_ie_container nan_ies;

	/* nan engines */
	struct phl_nan_sync_priv sync_engine_priv;
	struct phl_nan_de_priv discovery_engine_priv;

	/* op pointers */
	struct rtw_phl_nan_ops nan_ops;

	/* ref pointer */
	struct phl_info_t *phl_info;
};

struct phl_nan_event_parm {
	u32 event_type;
	u32 length;
	struct phl_nan_info *nan_info;
	/* TODO:(jw.hou@realtek.com): This coding approach avoids the
	 * Windows C++ compiler warning C4200.
	 */
	u8 data[1];
};

struct phl_nan_add_srv_parm {
	u8 srv_type;
	union rtw_phl_nan_srv_data srv_data;

	u8 *p_instance_id; /* return to the core layer */
};

struct phl_nan_alloc_nmi_sta_info_cmd {
	u8 peer_mac[MAC_ALEN];
};
struct phl_nan_rmv_nmi_sta_info_cmd {
	struct rtw_phl_stainfo_t *phl_sta;
};

#define phl_nan_info_to_drvpriv(naninfo) \
	phl_to_drvpriv((struct phl_info_t *)phl_container_of(naninfo,\
							     struct phl_info_t,\
							     naninfo))
#define phl_nan_cancel_dw_timer(drv, naninfo) \
	do { \
		/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			     "%s: cancel_dw_timer(%p)\n",
			     __func__, (nanpriv)); */ \
		_os_cancel_timer(drv, &(naninfo->sync_engine_priv.dw_timer)); \
	} while (0)


#define phl_nan_set_dw_timer(drv, naninfo, ms) \
	do { \
		/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: et_sw_timer(%p, %d)\n",
			     __func__, (nanpriv), (ms)); */ \
		_os_set_timer(drv, &(naninfo->sync_engine_priv.dw_timer), (ms)); \
	} while (0)

#define phl_nan_cast_cmd_ptr(_type, _length, _nan_info, _ptlv_hdr)\
	do { \
		(_ptlv_hdr)->event_type = (_type);\
		(_ptlv_hdr)->length = (_length);\
		(_ptlv_hdr)->nan_info = (_nan_info);\
	} while (0)

#define phl_nan_get_committed_dw_period(de_info, cmt_dw) \
	do { \
		struct phl_nan_info *nan_info =\
			phl_container_of(de_info, struct phl_nan_info,\
					 discovery_engine_priv);\
		cmt_dw = (((nan_info)->sync_engine_priv.cmt_dw_peri_24G)\
			  / (DW_INTERVAL / NAN_SLOT));\
	} while (0)

#define phl_nan_get_de_info(phl_info) \
	(&(phl_info->nan_info.discovery_engine_priv))

#define phl_nan_done_peer_info_entry(d, nan_info) \
	do { \
		_os_spinunlock(d, &nan_info->peer_info_queue.lock, _bh, NULL); \
	} while (0)

/**
 * phl_nan_append_attr - Append the NAN attribute according to attribute ID
 * @mac_info:
 * @pbuf: frame buffer
 * @attr_id: NAN attribute ID
 * @pdata: other information for the attribute, may be NULL
 * (ac) append_nan_attr
 *
 * RETURN:
 * Length of the attribute
 */
u16 phl_nan_append_attr(void *d, struct phl_nan_mac_info *mac_info,
			u8 *pbuf, u8 attr_id, void *pdata);

/**
 * phl_nan_alloc_peer_info_entry - Allocate NAN peer info
 * if return true, driver must call phl_nan_done_peer_info_entry() to unlock queue
 * @nan_info: rtw_nan_priv
 * @p_peer_info_entry: return nan_peer_info_entry if allocate successfully
 * @mac_address: peer's nmi
 * (ac) _rtw_alloc_nan_peer_info_entry
 *
 * RETURN:
 * true: Success
 * false: FAIL
 */
u8
phl_nan_alloc_peer_info_entry(struct phl_nan_info *nan_info,
			      struct phl_nan_peer_info_entry **p_peer_info_entry,
			      u8 *mac_addr);

u8 phl_nan_is_mgnt_frame_need_enqueue(struct phl_nan_info *nan_info,
				      struct phl_nan_peer_info_entry *peer_info);

/**
 * phl_nan_insert_mgnt_frame_entry -
 *  - prepare a nan (unicast) mgnt frame and,
 *  - decide if issue now according to is_xmit_now and other parameters
 * @nan_info: phl_nan_info
 * @nan_attr_buf: nan attribute lists
 * @attr_len: len of nan_attr_buf
 * @peer_info_mac: target peer's mac
 * @pkt_type: mgmt packet type
 * @is_xmit_now: now or enqueue
 * (ac) _rtw_nan_insert_send_mgnt_frame_entry
 */
enum rtw_phl_status
phl_nan_insert_mgnt_frame_entry(struct phl_nan_info *nan_info,
				u8 *nan_attr_buf,
				u16 attr_len,
				u8 *peer_info_mac,
				enum rtw_phl_nan_pkt_type pkt_type,
				u8 mgnt_need_enq);

/**
 * phl_nan_append_device_cap - Append the Device Capability attribute
 * @d: drvpriv
 * @pbuf: frame buffer
 * @sync_info: to get dw_cnt, nan support bands, and flags
 * (ac) _append_nan_device_cap
 *
 * RETURN:
 * Length of dca
 */
u16
phl_nan_append_device_cap(void *d, u8 *pbuf, struct phl_nan_sync_priv *sync_info);

/**
* _append_nan_element_container - Append the Element Container attribute
* @d: drvpriv
* @nan_info: to access nan_ies
* @pbuf: frame buffer
* (ac) _append_nan_element_container
*
* RETURN:
* Length of Element Container attribute
*/
u16
phl_nan_append_ele_container(void *d, struct phl_nan_info *nan_info, u8 *pbuf);

/**
 * phl_nan_c2h_info_parsing
 * @nan_info:
 * @pbuf: nan rpt from c2h
 * (ac) rtw_nan_c2h_info_parsing
 */
void phl_nan_c2h_info_parsing(struct phl_nan_info *nan_info, u8 *buf);

/**
 * phl_cmd_nan_event_hdl - Handle all NAN releated command
 * @phl_info:
 * @pbuf: pointer of phl_nan_event_parm; need to free mem in the handler.
 *
 * Return : enum rtw_phl_status status
 * RTW_PHL_STATUS_SUCCESS: NAN event handler found
 * RTW_PHL_STATUS_FAILURE: NAN evnet type not defined
 */
enum rtw_phl_status
phl_cmd_nan_event_hdl(struct phl_info_t *phl_info, u8 *pbuf);

/**
 * phl_nan_enqueue_cmd - enqueue nan cmd for multithreading processing
 * @phl_info: pointer of phl_info which contains nan_info
 * @type: _nan_event_type
 * @pbuf: buffer for additional information
 * @len: buffer length
 * @cmd_type: directly or enqueue
 * @cmd_timeout:
 * (ac) enqueue_nan_cmd
 */
enum rtw_phl_status
phl_nan_enqueue_cmd(struct phl_info_t *phl_info,
		    enum _nan_event_type type, void* pbuf, u32 len,
		    enum phl_cmd_type cmd_type, u32 cmd_timeout);

/**
 * phl_nan_is_peer_info_exist
 * if return true, driver must call phl_nan_done_peer_info_entry() to unlock queue
 * @nan_info: pointer of phl_info which contains nan_info
 * @peer_addr: peer_info's mac address
 * @peer_info: double pointer for query the peer info
 * (ac) is_nan_peer_info_exist
 */
u8
phl_nan_is_peer_info_exist(struct phl_nan_info *nan_info,
			   u8 *peer_addr,
			   struct phl_nan_peer_info_entry **peer_info);

void phl_nan_set_cluster_id(struct phl_nan_info *nan_info, u8 *cluster_id);

void phl_nan_get_cluster_id(struct phl_nan_info *nan_info, u8 *cluster_id);

/**
 * phl_nan_dw_timer_hdl - dw handler
 * @ctx: context for discovery handler
 */
void
phl_nan_dw_timer_hdl(void *ctx);

#define _phl_nan_get_info(_phl) \
	(&(((struct phl_info_t *)_phl)->nan_info))

/**
 * phl_nan_mdl_init - during phl module init process (driver-loading)
 * @phl_info: pointer of phl_info which contains nan_info
 */
enum rtw_phl_status
phl_nan_mdl_init(struct phl_info_t* phl_info);

/**
 * phl_nan_mdl_deinit - during phl module de-init process(driver-unloading)
 * @phl_info: pointer of phl_info which contains nan_info
 */
void
phl_nan_mdl_deinit(struct phl_info_t* phl_info);

/**
 * phl_nan_dump_bcn_pkt - dump nan bcn for debugging
 * @nan_bcn_info: the description of sync and disc nan bcn
 */
void phl_nan_dump_bcn_pkt(struct phl_nan_bcn nan_bcn_info);

/* phl_nan_debug_dump_data - dump buf data for debugging */
void phl_nan_debug_dump_data(u8 *buf, u32 buf_len, const char *prefix);

struct phl_nan_info *
phl_nan_get_info(enum _nan_engine_type type, void *p_engine);

#else /* !CONFIG_PHL_NAN */

#define phl_nan_mdl_init(phl_info) RTW_PHL_STATUS_SUCCESS
#define phl_nan_mdl_deinit(phl_info)

#endif /* CONFIG_PHL_NAN */
#endif /* _PHL_NAN_H_ */
