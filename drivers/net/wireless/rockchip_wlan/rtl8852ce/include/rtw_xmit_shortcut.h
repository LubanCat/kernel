/******************************************************************************
 *
 * Copyright(c) 2007 - 2019 Realtek Corporation.
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
#ifndef _RTW_XMIT_SHORTCUT_H_
#define _RTW_XMIT_SHORTCUT_H_

#ifdef CONFIG_CORE_TXSC

//#define USE_ONE_WLHDR /* do not use one hdr when use sw amsdu in txsc */ /* TBD, open this when hw_hde_conv ready */
/*
#ifndef USE_ONE_WLHDR
#define USE_PREV_WLHDR_BUF
#endif
*/
#define CORE_TXSC_ENTRY_NUM 8
#define CORE_TXSC_WLHDR_SIZE (WLHDR_SIZE + SNAP_SIZE + 2 + _AES_IV_LEN_)
#define CORE_TXSC_DEBUG_BUF_SIZE (sizeof(struct rtw_xmit_req) + sizeof(struct rtw_pkt_buf_list)*2)

enum txsc_action_type {
	TXSC_NONE,
	TXSC_SKIP,
	TXSC_ADD,
	TXSC_APPLY,
	TXSC_AMSDU_APPLY,
};

enum full_cnt_type {
	PHL_WD_EMPTY,
	PHL_BD_FULL,
	PHL_WD_RECYCLE_NOTHING,
	PHL_WD_RECYCLE_OK,
};

#ifdef CONFIG_TXSC_AMSDU

#define MAX_AMSDU_ENQ_NUM 256

enum txsc_amsdu_timer_type {
	TXSC_AMSDU_TIMER_UNSET,
	TXSC_AMSDU_TIMER_SETTING,
	TXSC_AMSDU_TIMER_TIMEOUT,
};

enum txsc_amsdu_enq_type {
	TXSC_AMSDU_NEED_DEQ,
	TXSC_AMSDU_ENQ_ABORT,
	TXSC_AMSDU_ENQ_SUCCESS,
};

struct txsc_amsdu_swq {
	_lock	txsc_amsdu_lock;
	struct sk_buff *skb_q[MAX_AMSDU_ENQ_NUM];
	u32 wptr;
	u32 rptr;
	u32 cnt;
};
#endif

struct txsc_pkt_entry {
	enum txsc_action_type step;
	struct sta_info *psta;
	struct rtw_xmit_req *ptxreq;

	u8 txsc_id;
	u8 priority;

	struct sk_buff *xmit_skb[MAX_TXSC_SKB_NUM];
	u8 skb_cnt;
	bool is_sta_sleep;
#ifdef CONFIG_TXSC_AMSDU
	u8 ac;
	bool amsdu;
	bool is_amsdu_timeout;
#endif
#ifdef TXSC_DBG_COPY_ORI_WLHDR_MDATA
	bool is_spec_pkt;
	struct rtw_t_meta_data mdata_ori;
	u8      wlhdr_ori[CORE_TXSC_WLHDR_SIZE];
	u8      wlhdr_ori_len;
#endif
};

struct txsc_entry {
	u8	txsc_is_used;
	u8	txsc_ethdr[ETH_HLEN];

	/* wlhdr --- */
	#ifdef USE_ONE_WLHDR
	u8	*txsc_wlhdr;
	#else
	u8	txsc_wlhdr[CORE_TXSC_WLHDR_SIZE];
	#endif
	u8	txsc_wlhdr_len;
	struct rtw_pkt_buf_list	txsc_pkt_list0;
	/* wlhdr --- */

	struct rtw_t_meta_data	txsc_mdata;
	u32	txsc_frag_len;/* for pkt frag check */
	u8 txsc_phl_id; /* CONFIG_PHL_TXSC */
#ifdef CONFIG_TXSC_AMSDU
	bool txsc_amsdu;
#endif
	u32	txsc_cache_hit;
};

void _print_txreq_pklist(struct xmit_frame *pxframe, struct rtw_xmit_req *ptxsc_txreq, struct sk_buff *pskb, const char *func);
void txsc_init(_adapter *padapter);
void txsc_clear(_adapter *padapter);
void txsc_dump(_adapter *padapter, void *m);
void txsc_dump_data(u8 *buf, u16 buf_len, const char *prefix);
u8 txsc_get_sc_cached_entry(_adapter *padapter, struct sk_buff *pskb, struct txsc_pkt_entry *txsc_pkt);
void txsc_add_sc_cache_entry(_adapter *padapter, struct xmit_frame *pxframe, struct txsc_pkt_entry *txsc_pkt);
u8 txsc_apply_sc_cached_entry(_adapter *padapter, struct txsc_pkt_entry *txsc_pkt);
#ifdef CONFIG_PCI_HCI
void txsc_fill_txreq_phyaddr(_adapter *padapter, struct rtw_pkt_buf_list *pkt_list);
void txsc_recycle_txreq_phyaddr(_adapter *padapter, struct rtw_xmit_req *txreq);
#endif
void txsc_free_txreq(_adapter *padapter, struct rtw_xmit_req *txreq);
void txsc_debug_sc_entry(_adapter *padapter, struct xmit_frame *pxframe, struct txsc_pkt_entry *txsc_pkt);
void txsc_issue_addbareq_cmd(_adapter *padapter, u8 priority, struct sta_info *psta, u8 issue_when_busy);
#ifdef CONFIG_TXSC_AMSDU
void txsc_amsdu_queue_free(_adapter *padapter, struct sta_info *psta);
void txsc_amsdu_queue_init(_adapter *padapter, struct sta_info *psta);
u8 txsc_amsdu_enqueue(_adapter *padapter, struct txsc_pkt_entry *txsc_pkt, u8 *status);
s32 txsc_amsdu_timeout_tx(struct sta_info *psta, u8 ac);
void txsc_amsdu_sta_init(_adapter *padapter, struct sta_info* psta);
void txsc_amsdu_clear(_adapter *padapter);
void txsc_amsdu_dump(_adapter *padapter, void *m);
#endif /* CONFIG_TXSC_AMSDU */
#endif /* CONFIG_CORE_TXSC */
#endif /* _RTW_XMIT_SHORTCUT_H_ */

