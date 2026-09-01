/** @file */
/******************************************************************************
 *
 * Copyright(c) 2019 Realtek Corporation. All rights reserved.
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
 ******************************************************************************/

#ifndef _MAC_AX_FW_CMD_H_
#define _MAC_AX_FW_CMD_H_

#include "../type.h"
#include "../fw_ax/inc_hdr/fwcmd_intf.h"
#include "fwcmd_intf_f2p.h"
#include "fwcmd_intf_f2p_v1.h"
#include "trx_desc.h"
#include "fwofld.h"
#include "p2p.h"
#include "flash.h"
#include "dbg_cmd.h"

#define FWCMD_HDR_LEN		8
#define C2HREG_HDR_LEN 2
#define H2CREG_HDR_LEN 2
#define C2HREG_CONTENT_LEN 14
#define H2CREG_CONTENT_LEN 14
#define C2HREG_LEN (C2HREG_HDR_LEN + C2HREG_CONTENT_LEN)
#define H2CREG_LEN (H2CREG_HDR_LEN + H2CREG_CONTENT_LEN)

#define H2C_CMD_LEN		64
#define H2C_DATA_LEN		256
#define H2C_LONG_DATA_LEN	2048

#define H2C_MAX_TOTAL_LEN 2048

#define SET_FWCMD_ID(_t, _ca, _cl, _f)                                         \
		(SET_WORD(_t, H2C_HDR_DEL_TYPE) | SET_WORD(_ca, H2C_HDR_CAT) | \
		 SET_WORD(_cl, H2C_HDR_CLASS) | SET_WORD(_f, H2C_HDR_FUNC))
#define GET_FWCMD_TYPE(id)	(GET_FIELD(id, C2H_HDR_DEL_TYPE))
#define GET_FWCMD_CAT(id)	(GET_FIELD(id, C2H_HDR_CAT))
#define GET_FWCMD_CLASS(id)	(GET_FIELD(id, C2H_HDR_CLASS))
#define GET_FWCMD_FUNC(id)	(GET_FIELD(id, C2H_HDR_FUNC))

#define FWCMD_TYPE_H2C	0
#define FWCMD_TYPE_C2H	1

#define FWCMD_C2H_CL_NULL		0xFF
#define FWCMD_C2H_FUNC_NULL		0xFF
#define FWCMD_C2H_CAT_NULL		0xFF
#define AP_SELF_DIAG_RST_ELEMENT_SIZE		12
#define AP_SELF_DIAG_RST_WARN_BITMAP_OFFSET	4
#define AP_SELF_DIAG_RST_ERR_BITMAP_OFFSET	8

#define OM_ADV_DL_MU_DATA_DIS	0

#define H2CB_CMD_HDR_SIZE	(FWCMD_HDR_LEN + WD_BODY_LEN_V1)
#define H2CB_CMD_SIZE		(H2C_CMD_LEN - FWCMD_HDR_LEN)
#define H2CB_CMD_QLEN		8

#define H2CB_DATA_HDR_SIZE	(FWCMD_HDR_LEN + WD_BODY_LEN_V1)
#define H2CB_DATA_SIZE		(H2C_DATA_LEN - FWCMD_HDR_LEN)
#define H2CB_DATA_QLEN		4

#define H2CB_LONG_DATA_HDR_SIZE	(FWCMD_HDR_LEN + WD_BODY_LEN)
#define H2CB_LONG_DATA_SIZE	(H2C_LONG_DATA_LEN - FWCMD_HDR_LEN)
#define H2CB_LONG_DATA_QLEN	1

#define FWCMD_WQ_MAX_JOB_NUM	5

#define FWCMD_LMT		12

#define MAC_AX_H2C_LMT_EN	0

#define FWCMD_H2CREG_BYTE0_SH 0
#define FWCMD_H2CREG_BYTE0_MSK 0xFF
#define FWCMD_H2CREG_BYTE1_SH 8
#define FWCMD_H2CREG_BYTE1_MSK 0xFF
#define FWCMD_H2CREG_BYTE2_SH 16
#define FWCMD_H2CREG_BYTE2_MSK 0xFF
#define FWCMD_H2CREG_BYTE3_SH 24
#define FWCMD_H2CREG_BYTE3_MSK 0xFF

#define SCANOFLD_RSP_EVT_ID 1
#define SCANOFLD_RSP_EVT_PARSE 1
#define SCANOFLD_ACK_BAND_SHIFT 6
#define SCANOFLD_ACK_RETURN_MASK 0x3F

#define H2CB_FLAGS_FREED	BIT(0)

#define MAC_AX_H2CREG_CNT 100
#define MAC_AX_H2CREG_US 200

/* cat: NONMAC */
#define H2C_CLASS_PHYDM_MAX 0x7
#define H2C_CLASS_RF_MAX 0xf
#define H2C_CLASS_BTC_MAX 0x17

#if MAC_FEAT_TWT_OFDMA_EN
// C2H TWT_NOTIFY_EVT
#define FWCMD_C2H_FUNC_TWT_NOTIFY_EVT 0x02
#define FWCMD_C2H_TWT_NOTIFY_EVT_TYPE_SH 0
#define FWCMD_C2H_TWT_NOTIFY_EVT_TYPE_MSK 0xff
#define FWCMD_C2H_TWT_NOTIFY_EVT_TWT_ID_SH 8
#define FWCMD_C2H_TWT_NOTIFY_EVT_TWT_ID_MSK 0x7
#define FWCMD_C2H_TWT_NOTIFY_EVT_TSF_LOW_SH 0
#define FWCMD_C2H_TWT_NOTIFY_EVT_TSF_LOW_MSK 0xffffffff
#define FWCMD_C2H_TWT_NOTIFY_EVT_TSF_HIGH_SH 0
#define FWCMD_C2H_TWT_NOTIFY_EVT_TSF_HIGH_MSK 0xffffffff
// H2C TWT_OFDMA_INFO_UPD
#define FWCMD_H2C_FUNC_TWT_OFDMA_INFO_UPD 0x03
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_OPTION_SH 0
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_OPTION_MSK 0x3
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_TWT_ID_SH 2
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_TWT_ID_MSK 0x7
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_MAX_TF_RETRY_NUM_SH 5
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_MAX_TF_RETRY_NUM_MSK 0xff
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_MAX_DL_RETRY_NUM_SH 13
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_MAX_DL_RETRY_NUM_MSK 0xff
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_ROUND_NUM_SH 21
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_ROUND_NUM_MSK 0xff
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_PREFERRED_AC_SH 29
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_PREFERRED_AC_MSK 0x3
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_HTC_BSR_CTRL_EN BIT(31)
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_ROUND_INTERVAL_SH 0
#define FWCMD_H2C_TWT_OFDMA_INFO_UPD_ROUND_INTERVAL_MSK 0xffff
#endif

/**
 * @struct h2c_buf_head
 * @brief h2c_buf_head
 *
 * @var h2c_buf_head::next
 * Please Place Description here.
 * @var h2c_buf_head::prev
 * Please Place Description here.
 * @var h2c_buf_head::pool
 * Please Place Description here.
 * @var h2c_buf_head::size
 * Please Place Description here.
 * @var h2c_buf_head::qlen
 * Please Place Description here.
 * @var h2c_buf_head::suspend
 * Please Place Description here.
 * @var h2c_buf_head::lock
 * Please Place Description here.
 */
struct h2c_buf_head {
	/* keep first */
	struct h2c_buf *next;
	struct h2c_buf *prev;
	u8 *pool;
	u32 size;
	u32 qlen;
	u8 suspend;
	mac_ax_mutex lock;
};

/**
 * @struct fwcmd_wkb_head
 * @brief fwcmd_wkb_head
 *
 * @var fwcmd_wkb_head::next
 * Please Place Description here.
 * @var fwcmd_wkb_head::prev
 * Please Place Description here.
 * @var fwcmd_wkb_head::qlen
 * Please Place Description here.
 * @var fwcmd_wkb_head::lock
 * Please Place Description here.
 */
struct fwcmd_wkb_head {
	/* keep first */
	struct h2c_buf *next;
	struct h2c_buf *prev;
	u32 qlen;
	mac_ax_mutex lock;
};

/**
 * @struct h2c_buf
 * @brief h2c_buf
 *
 * @var h2c_buf::next
 * Please Place Description here.
 * @var h2c_buf::prev
 * Please Place Description here.
 * @var h2c_buf::_class_
 * Please Place Description here.
 * @var h2c_buf::id
 * Please Place Description here.
 * @var h2c_buf::master
 * Please Place Description here.
 * @var h2c_buf::len
 * Please Place Description here.
 * @var h2c_buf::head
 * Please Place Description here.
 * @var h2c_buf::end
 * Please Place Description here.
 * @var h2c_buf::data
 * Please Place Description here.
 * @var h2c_buf::tail
 * Please Place Description here.
 * @var h2c_buf::hdr_len
 * Please Place Description here.
 * @var h2c_buf::flags
 * Please Place Description here.
 * @var h2c_buf::h2c_seq
 * Please Place Description here.
 */
struct h2c_buf {
	/* keep first */
	struct h2c_buf *next;
	struct h2c_buf *prev;
	enum h2c_buf_class _class_;
	u32 id;
	u32 len;
	u8 *head;
	u8 *end;
	u8 *data;
	u8 *tail;
	u32 hdr_len;
	u32 flags;
	u8 master;
	u8 h2c_seq;
};

struct h2c_info {
	u16 content_len;
	u8 h2c_cat;
	u8 h2c_class;
	u8 h2c_func;
	u8 rec_ack;
	u8 done_ack;
	u8 agg_en;
};

/**
 * @struct c2h_proc_class
 * @brief c2h_proc_class
 *
 * @var c2h_proc_class::id
 * Please Place Description here.
 * @var c2h_proc_class::handler
 * Please Place Description here.
 */
struct h2c_allloc_status {
	u16 ldata;
	u16 data;
	u16 cmd;
	u16 mac;
	u16 bb;
	u16 rf;
	u16 btc;
};

/**
 * @struct c2h_proc_class
 * @brief c2h_proc_class
 *
 * @var c2h_proc_class::id
 * Please Place Description here.
 * @var c2h_proc_class::handler
 * Please Place Description here.
 */
struct c2h_proc_class {
	u32 (*handler)(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		       struct rtw_c2h_info *info);
	u16 id;
};

struct fw_status_proc_class {
	u32 (*handler)(struct mac_ax_adapter *adapter, u8 *buf, u32 len);
	u16 id;
};

/**
 * @struct c2h_proc_func
 * @brief c2h_proc_func
 *
 * @var c2h_proc_func::id
 * Please Place Description here.
 * @var c2h_proc_func::handler
 * Please Place Description here.
 */
struct c2h_proc_func {
	u32 (*handler)(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		       struct rtw_c2h_info *info);
	u16 id;
};

/**
 * @struct mac_ax_c2hreg_info
 * @brief mac_ax_c2hreg_info
 *
 * @var mac_ax_c2hreg_info::id
 * Please Place Description here.
 * @var mac_ax_c2hreg_info::total_len
 * Please Place Description here.
 * @var mac_ax_c2hreg_info::content
 * Please Place Description here.
 * @var mac_ax_c2hreg_info::c2hreg
 * Please Place Description here.
 */

struct mac_ax_c2hreg_info {
	u8 *content;
	u8 id;
	u8 content_len;
	u8 c2hreg[C2HREG_LEN];
};

struct mac_ax_c2hreg_cont {
	struct fwcmd_c2hreg c2h_content;
	u8 id;
	u8 content_len;
};

struct mac_ax_c2hreg_poll {
	struct mac_ax_c2hreg_cont c2hreg_cont;
	u32 retry_cnt;
	u32 retry_wait_us;
	u8 polling_id;
};

struct mac_ax_h2creg_info {
	struct fwcmd_h2creg h2c_content;
	u8 id;
	u8 content_len;
};

struct c2h_event_id_proc {
	u32 (*hdl)(struct mac_ax_adapter *adapter, struct rtw_c2h_info *c2h,
		   enum phl_msg_evt_id *id, u8 *c2h_info);
	u8 cat;
	u8 cls;
	u8 func;
};

#if MAC_FEAT_TWT_OFDMA_EN
// C2H TWT_NOTIFY_EVT
struct fwcmd_twt_notify_evt {
	u32 dword0;
	u32 dword1;
	u32 dword2;
};

// H2C TWT_OFDMA_INFO_UPD
struct fwcmd_twt_ofdma_info_upd {
	u32 dword0;
	u32 dword1;
};
#endif

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */
/**
 * @brief h2cb_init
 *
 * @param *adapter
 * @return Please Place Description here.
 * @retval u32
 */
u32 h2cb_init(struct mac_ax_adapter *adapter);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief h2cb_exit
 *
 * @param *adapter
 * @return Please Place Description here.
 * @retval u32
 */
u32 h2cb_exit(struct mac_ax_adapter *adapter);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief h2cb_alloc
 *
 * @param *adapter
 * @param buf_class
 * @return Please Place Description here.
 * @retval  rtw_h2c_pkt
 */
#if MAC_AX_PHL_H2C
struct rtw_h2c_pkt *h2cb_alloc(struct mac_ax_adapter *adapter,
			       enum rtw_h2c_pkt_type buf_class);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief h2cb_free
 *
 * @param *adapter
 * @param *h2cb
 * @return Please Place Description here.
 * @retval void
 */
void h2cb_free(struct mac_ax_adapter *adapter, struct rtw_h2c_pkt *h2cb);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief h2cb_push
 *
 * @param *h2cb
 * @param len
 * @return Please Place Description here.
 * @retval u8
 */
u8 *h2cb_push(struct rtw_h2c_pkt *h2cb, u32 len);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief h2cb_put
 *
 * @param *h2cb
 * @param len
 * @return Please Place Description here.
 * @retval u8
 */
u8 *h2cb_put(struct rtw_h2c_pkt *h2cb, u32 len);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief h2c_pkt_set_hdr
 *
 * @param *adapter
 * @param *h2cb
 * @param type
 * @param cat
 * @param _class_
 * @param func
 * @param rack
 * @param dack
 * @return Please Place Description here.
 * @retval u32
 */
u32 h2c_pkt_set_hdr(struct mac_ax_adapter *adapter, struct rtw_h2c_pkt *h2cb,
		    u8 type, u8 cat, u8 _class_, u8 func, u16 rack, u16 dack);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief h2c_pkt_set_hdr_fwdl
 *
 * @param *adapter
 * @param *h2cb
 * @param type
 * @param cat
 * @param _class_
 * @param func
 * @param rack
 * @param dack
 * @return Please Place Description here.
 * @retval u32
 */
u32 h2c_pkt_set_hdr_fwdl(struct mac_ax_adapter *adapter,
			 struct rtw_h2c_pkt *h2cb, u8 type, u8 cat, u8 _class_,
			 u8 func, u16 rack, u16 dack);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief h2c_pkt_build_txd
 *
 * @param *adapter
 * @param *h2cb
 * @return Please Place Description here.
 * @retval u32
 */
u32 h2c_pkt_build_txd(struct mac_ax_adapter *adapter, struct rtw_h2c_pkt *h2cb);
/**
 * @}
 * @}
 */

/**
 * @brief h2c_agg_enqueue
 *
 * @param *adapter
 * @param *h2cb
 * @return Please Place Description here.
 * @retval u32
 */
u32 h2c_agg_enqueue(struct mac_ax_adapter *adapter, struct rtw_h2c_pkt *h2cb);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief h2cb_alloc
 *
 * @param *adapter
 * @param buf_class
 * @return Please Place Description here.
 * @retval  h2c_buf
 */
#else
struct h2c_buf *h2cb_alloc(struct mac_ax_adapter *adapter,
			   enum h2c_buf_class buf_class);
/**
 * @}
 * @}
 */
void h2cb_free(struct mac_ax_adapter *adapter, struct h2c_buf *h2cb);
u8 *h2cb_push(struct h2c_buf *h2cb, u32 len);
u8 *h2cb_put(struct h2c_buf *h2cb, u32 len);
u32 h2c_pkt_set_hdr(struct mac_ax_adapter *adapter, struct h2c_buf *h2cb,
		    u8 type, u8 cat, u8 _class_, u8 func, u16 rack, u16 dack);
u32 h2c_pkt_set_hdr_fwdl(struct mac_ax_adapter *adapter, struct h2c_buf *h2cb,
			 u8 type, u8 cat, u8 _class_, u8 func, u16 rack,
			 u16 dack);
u32 h2c_pkt_build_txd(struct mac_ax_adapter *adapter, struct h2c_buf *h2cb);
u32 h2c_agg_enqueue(struct mac_ax_adapter *adapter, h2c_buf *h2cb);

u32 get_h2cb_status(struct h2c_allloc_status *h2c_status);

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief fwcmd_wq_enqueue
 *
 * @param *adapter
 * @param *h2cb
 * @return Please Place Description here.
 * @retval u32
 */
#endif
u32 fwcmd_wq_enqueue(struct mac_ax_adapter *adapter, struct h2c_buf *h2cb);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup C2H
 * @{
 */

/**
 * @brief mac_process_c2h
 *
 * @param *adapter
 * @param *buf
 * @param len
 * @param *ret
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_process_c2h(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		    u8 *ret);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup C2H
 * @{
 */

/**
 * @brief c2h_field_parsing
 *
 * @param *hdr
 * @param *info
 * @return Please Place Description here.
 * @retval u8
 */
u8 c2h_field_parsing(struct mac_ax_adapter *adapter,
		     struct fwcmd_hdr *hdr, struct rtw_c2h_info *info);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief mac_host_getpkt_h2c
 *
 * @param *adapter
 * @param macid
 * @param pkttype
 * @return Please Place Description here.
 * @retval u32
 */
 #if MAC_FEAT_PSAP
u32 mac_host_getpkt_h2c(struct mac_ax_adapter *adapter, u8 macid, u8 pkttype);
 #endif
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief mac_outsrc_h2c_common
 *
 * @param *adapter
 * @param *hdr
 * @param *pvalue
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_outsrc_h2c_common(struct mac_ax_adapter *adapter,
			  struct rtw_g6_h2c_hdr *hdr, u32 *pvalue);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief h2c_end_flow
 *
 * @param *adapter
 * @return Please Place Description here.
 * @retval u32
 */
u32 h2c_end_flow(struct mac_ax_adapter *adapter);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup H2C
 * @{
 */

/**
 * @brief mac_send_h2creg
 *
 * @param *adapter
 * @param *content
 * @param len
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_send_h2creg(struct mac_ax_adapter *adapter, u32 *content, u8 len);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup C2H
 * @{
 */

/**
 * @brief mac_process_c2hreg
 *
 * @param *adapter
 * @param *info
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_process_c2hreg(struct mac_ax_adapter *adapter,
		       struct mac_ax_c2hreg_info *info);
/**
 * @}
 * @}
 */

u32 proc_msg_reg(struct mac_ax_adapter *adapter,
		 struct mac_ax_h2creg_info *h2c,
		 struct mac_ax_c2hreg_poll *c2h);

/**
 * @addtogroup Firmware
 * @{
 * @addtogroup C2H
 * @{
 */

/**
 * @brief mac_get_c2h_event
 *
 * Get the phl_msg_evt_id from C2H packet
 *
 * @param *adapter
 * @param *c2h
 * @param *id
 * @return Return 0 when getting event ID successfully.
 * @retval u32
 */

u32 mac_get_c2h_event(struct mac_ax_adapter *adapter,
		      struct rtw_c2h_info *c2h,
		      enum phl_msg_evt_id *id,
		      u8 *c2h_info);

/**
 * @}
 * @}
 */

u32 mac_set_h2c_c2h_mon(struct mac_ax_adapter *adapter, u8 en);

u32 mac_h2c_common(struct mac_ax_adapter *adapter, struct h2c_info *info, u32 *content);

u32 mac_get_h2c_max_content_len(struct mac_ax_adapter *adapter);

u32 c2h_wow_aoac_report_hdl(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			    struct rtw_c2h_info *info);
u32 c2h_wow_apf_report_hdl(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			   struct rtw_c2h_info *info);

u32 c2h_wow(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
	    struct rtw_c2h_info *info);

static u32 c2h_usr_tx_rpt_info(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			       struct rtw_c2h_info *info);

static u32 c2h_tx_duty_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			   u32 len, struct rtw_c2h_info *info);

u32 c2h_twt(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
	    struct rtw_c2h_info *info);

static u32 c2h_tsf32_togl_rpt_hdl(struct mac_ax_adapter *adapter, u8 *buf,
				  u32 len, struct rtw_c2h_info *info);

static u32 c2h_scanofld_rsp_hdl(struct mac_ax_adapter *adapter, u8 *buf,
				u32 len, struct rtw_c2h_info *info);

static u32 c2h_read_rsp_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			    u32 len, struct rtw_c2h_info *info);

static u32 c2h_pkt_ofld_rsp_hdl(struct mac_ax_adapter *adapter, u8 *buf,
				u32 len, struct rtw_c2h_info *info);

u32 c2h_nan(struct mac_ax_adapter *adapter, u8 *buf, u32 len, struct rtw_c2h_info *info);

#if MAC_FEAT_MCC
u32 c2h_mcc_tsf_rpt_hdl(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			struct rtw_c2h_info *info);

u32 c2h_mcc_status_rpt_hdl(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			   struct rtw_c2h_info *info);

u32 c2h_mcc_req_ack_hdl(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			struct rtw_c2h_info *info);

u32 c2h_mcc_rcv_ack_hdl(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			struct rtw_c2h_info *info);

u32 c2h_mcc(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
	    struct rtw_c2h_info *info);

#endif /* MAC_FEAT_MCC */

static u32 c2h_macid_pause_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			       u32 len, struct rtw_c2h_info *info);

u32 c2h_fw_ofld(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		struct rtw_c2h_info *info);

u32 c2h_fw_info(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		struct rtw_c2h_info *info);

u32 c2h_fw_dbg(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
	       struct rtw_c2h_info *info);

static u32 c2h_fwi_rev_ack(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			   struct rtw_c2h_info *info);

static u32 c2h_fwi_done_ack_parse(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
				  struct rtw_c2h_info *info);

static u32 c2h_fwi_cmd_log(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			   struct rtw_c2h_info *info);

static u32 c2h_fwi_bcn_upd_done(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
				struct rtw_c2h_info *info);

static u32 c2h_fwi_bcn_stats(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			     struct rtw_c2h_info *info);

static u32 c2h_fwi_bcn_csazero(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			       struct rtw_c2h_info *info);

static u32 c2h_fwi_bcn_bc_chg_zero(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
				   struct rtw_c2h_info *info);

static u32 c2h_fwdx_info_handler(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
				 struct rtw_c2h_info *info);

static u32 c2h_frame_to_act_rpt(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
				struct rtw_c2h_info *info);

u32 c2h_fast_ch_sw(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		   struct rtw_c2h_info *info);

u32 c2h_wps_rpt(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		struct rtw_c2h_info *info);

static u32 c2h_twt_wait_announ_hdl(struct mac_ax_adapter *adapter, u8 *buf,
				   u32 len, struct rtw_c2h_info *info);

u32 c2h_rx_dbg_hdl(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		   struct rtw_c2h_info *info);

u32 c2h_port_init_stat(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		       struct rtw_c2h_info *info);

u32 c2h_port_cfg_stat(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		      struct rtw_c2h_info *info);

u32 c2h_nan_tsf_info_hdl(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			 struct rtw_c2h_info *info);

u32 c2h_nan_cluster_info_hdl(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			     struct rtw_c2h_info *info);

u32 c2h_nan_cluster_join_hdl(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			     struct rtw_c2h_info *info);

static u32 c2h_misc_ccxrpt(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			   struct rtw_c2h_info *info);

u32 c2h_fast_ch_sw_rpt_hdl(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			   struct rtw_c2h_info *info);

static u32 c2h_dump_efuse_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			      u32 len, struct rtw_c2h_info *info);

static u32 c2h_csi_tx_result_hdl(struct mac_ax_adapter *adapter, u8 *buf,
				 u32 len, struct rtw_c2h_info *info);

static u32 c2h_cmd_ofld_rsp_hdl(struct mac_ax_adapter *adapter, u8 *buf,
				u32 len, struct rtw_c2h_info *info);

static u32 c2h_cl_mport(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			struct rtw_c2h_info *info);

static u32 c2h_cl_misc(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
		       struct rtw_c2h_info *info);

static u32 c2h_ch_switch_rpt_hdl(struct mac_ax_adapter *adapter, u8 *buf,
				 u32 len, struct rtw_c2h_info *info);

static u32 c2h_beacon_resend_hdl(struct mac_ax_adapter *adapter, u8 *buf,
				 u32 len, struct rtw_c2h_info *info);

static u32 c2h_bcn_sync_rpt_info(struct mac_ax_adapter *adapter, u8 *buf,
				 u32 len, struct rtw_c2h_info *info);

static u32 c2h_bcn_filter_rpt_hdl(struct mac_ax_adapter *adapter, u8 *buf,
				  u32 len, struct rtw_c2h_info *info);

static u32 c2h_bcn_erly_notify(struct mac_ax_adapter *adapter, u8 *buf, u32 len,
			       struct rtw_c2h_info *info);

static u32 get_wps_rpt_event_id(struct mac_ax_adapter *adapter,
				struct rtw_c2h_info *c2h,
				enum phl_msg_evt_id *id,
				u8 *c2h_info);

static u32 get_bcn_resend_event(struct mac_ax_adapter *adapter,
				struct rtw_c2h_info *c2h,
				enum phl_msg_evt_id *id,
				u8 *c2h_info);

static u32 get_tsf32_togl_rpt_event(struct mac_ax_adapter *adapter,
				    struct rtw_c2h_info *c2h,
				    enum phl_msg_evt_id *id,
				    u8 *c2h_info);

static u32 get_fw_rx_dbg_event(struct mac_ax_adapter *adapter,
			       struct rtw_c2h_info *c2h,
			       enum phl_msg_evt_id *id,
			       u8 *c2h_info);

static u32 get_bcn_csa_event(struct mac_ax_adapter *adapter,
			     struct rtw_c2h_info *c2h,
			     enum phl_msg_evt_id *id,
			     u8 *c2h_info);

static u32 get_bcn_bc_chg_event(struct mac_ax_adapter *adapter,
				struct rtw_c2h_info *c2h,
				enum phl_msg_evt_id *id,
				u8 *c2h_info);

static u32 get_scanofld_event(struct mac_ax_adapter *adapter, struct rtw_c2h_info *c2h,
			      enum phl_msg_evt_id *id, u8 *c2h_info);

static u32 get_usr_txrpt_info_event(struct mac_ax_adapter *adapter,
				    struct rtw_c2h_info *c2h,
				    enum phl_msg_evt_id *id,
				    u8 *c2h_info);

static u32 get_frame_to_act_rpt_event(struct mac_ax_adapter *adapter,
				      struct rtw_c2h_info *c2h,
				      enum phl_msg_evt_id *id,
				      u8 *c2h_info);

static u32 get_nan_cluster_info_event(struct mac_ax_adapter *adapter,
				      struct rtw_c2h_info *c2h,
				      enum phl_msg_evt_id *id,
				      u8 *c2h_info);

static u32 get_nan_tsf_info_event(struct mac_ax_adapter *adapter,
				  struct rtw_c2h_info *c2h,
				  enum phl_msg_evt_id *id,
				  u8 *c2h_info);

static u32 get_nan_cluster_join_event(struct mac_ax_adapter *adapter,
				      struct rtw_c2h_info *c2h,
				      enum phl_msg_evt_id *id,
				      u8 *c2h_info);

static u32 c2h_wait_announ_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			       u32 len, struct rtw_c2h_info *info);

static u32 c2h_stat_rpt_hdl(struct mac_ax_adapter *adapter, u8 *buf,
			    u32 len, struct rtw_c2h_info *info);

#if MAC_FEAT_TWT_OFDMA_EN
static u32 get_twt_notify_event(struct mac_ax_adapter *adapter,
				struct rtw_c2h_info *c2h,
				enum phl_msg_evt_id *id,
				u8 *c2h_info);

static u32 c2h_twt_notify_evt_hdl(struct mac_ax_adapter *adapter, u8 *buf,
				  u32 len, struct rtw_c2h_info *info);
#endif

static u32 get_twt_wait_announ_event(struct mac_ax_adapter *adapter,
				     struct rtw_c2h_info *c2h,
				     enum phl_msg_evt_id *id,
				     u8 *c2h_info);

static u32 c2h_fwi_done_ack(struct mac_ax_adapter *adapter, struct rtw_c2h_info *info);

#endif

