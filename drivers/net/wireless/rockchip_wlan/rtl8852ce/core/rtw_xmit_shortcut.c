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

#include <drv_types.h>

#ifdef CONFIG_CORE_TXSC
u8 DBG_PRINT_TXREQ_ONCE;
void _print_txreq_pklist(struct xmit_frame *pxframe, struct rtw_xmit_req *ptxreq, struct sk_buff *pskb, const char *func)
{
	struct rtw_pkt_buf_list *pkt_list = NULL;
	struct rtw_xmit_req *txreq = NULL;
	u8 pkt_cnt = 0, i;
#ifndef TXSC_DBG_DUMP_SPEC_PKT
	if (DBG_PRINT_TXREQ_ONCE == 1) {
#endif
		RTW_PRINT("%s\n", func);
		RTW_PRINT("[%s] pxframe=%p txreq=%p\n", func, pxframe, ptxreq);

		if (pskb)
			txsc_dump_data(pskb->data, ETH_HLEN, "ETHHDR");

		if (ptxreq != NULL)
			txreq = ptxreq;
		else
			txreq = pxframe->phl_txreq;

		pkt_list = (struct rtw_pkt_buf_list *)txreq->pkt_list;
		pkt_cnt = txreq->pkt_cnt;

		RTW_PRINT("os_priv:%p, treq_type:%d, pkt_cnt:%d, total_len:%d, shortcut_id:%d\n\n",
				txreq->os_priv, txreq->treq_type, txreq->pkt_cnt, txreq->total_len, txreq->shortcut_id);

		for (i = 0; i < pkt_cnt; i++) {
			RTW_PRINT("pkt_list[%d]\n", i);
			txsc_dump_data(pkt_list->vir_addr, pkt_list->length, "pkt_list");
			pkt_list++;
		}
		DBG_PRINT_TXREQ_ONCE = 0;
#ifndef TXSC_DBG_DUMP_SPEC_PKT
	}
#endif
}

#ifdef CONFIG_TXSC_AMSDU
static void txsc_amsdu_vo_timeout_handler(void *func_ontext)
{
	struct sta_info *psta = (struct sta_info *)func_ontext;
	u8 ac = 0;

	psta->txsc_amsdu_vo_timeout_sts = TXSC_AMSDU_TIMER_TIMEOUT;

	/* do amsdu tx */
	txsc_amsdu_timeout_tx(psta, ac);
}

static void txsc_amsdu_vi_timeout_handler(void *func_ontext)
{
	struct sta_info *psta = (struct sta_info *)func_ontext;
	u8 ac = 1;

	psta->txsc_amsdu_vi_timeout_sts = TXSC_AMSDU_TIMER_TIMEOUT;

	/* do amsdu tx */
	txsc_amsdu_timeout_tx(psta, ac);
}

static void txsc_amsdu_be_timeout_handler(void *func_ontext)
{
	struct sta_info *psta = (struct sta_info *)func_ontext;
	u8 ac = 2;

	psta->txsc_amsdu_be_timeout_sts = TXSC_AMSDU_TIMER_TIMEOUT;

	/* do amsdu tx */
	txsc_amsdu_timeout_tx(psta, ac);
}

static void txsc_amsdu_bk_timeout_handler(void *func_ontext)
{
	struct sta_info *psta = (struct sta_info *)func_ontext;
	u8 ac = 3;

	psta->txsc_amsdu_bk_timeout_sts = TXSC_AMSDU_TIMER_TIMEOUT;

	/* do amsdu tx */
	txsc_amsdu_timeout_tx(psta, ac);
}

void txsc_amsdu_queue_init(_adapter *padapter, struct sta_info *psta)
{
	u32 i, j;

	for (i = 0; i < 4; i++) {
		psta->amsdu_txq[i].cnt = 0;
		psta->amsdu_txq[i].wptr = 0;
		psta->amsdu_txq[i].rptr = 0;
		for (j = 0; j < MAX_AMSDU_ENQ_NUM; j++)
			psta->amsdu_txq[i].skb_q[j] = NULL;

		/* init lock */
		_rtw_spinlock_init(&psta->amsdu_txq[i].txsc_amsdu_lock);
	}

	/* init timer */
	rtw_init_timer(&(psta->txsc_amsdu_vo_timer), txsc_amsdu_vo_timeout_handler, psta);
	psta->txsc_amsdu_vo_timeout_sts = TXSC_AMSDU_TIMER_UNSET;

	rtw_init_timer(&(psta->txsc_amsdu_vi_timer), txsc_amsdu_vi_timeout_handler, psta);
	psta->txsc_amsdu_vi_timeout_sts = TXSC_AMSDU_TIMER_UNSET;

	rtw_init_timer(&(psta->txsc_amsdu_be_timer), txsc_amsdu_be_timeout_handler, psta);
	psta->txsc_amsdu_be_timeout_sts = TXSC_AMSDU_TIMER_UNSET;

	rtw_init_timer(&(psta->txsc_amsdu_bk_timer), txsc_amsdu_bk_timeout_handler, psta);
	psta->txsc_amsdu_bk_timeout_sts = TXSC_AMSDU_TIMER_UNSET;

	psta->txsc_amsdu_num = padapter->tx_amsdu;
}

void txsc_amsdu_queue_free(_adapter *padapter, struct sta_info *psta)
{
	struct sk_buff *pskb = NULL;
	struct txsc_amsdu_swq *txq = NULL;
	u32 i, j;

	for (i = 0; i < 4; i++) {
		txq = &psta->amsdu_txq[i];

		_rtw_spinlock_bh(&txq->txsc_amsdu_lock);
		for (j = 0; j < MAX_AMSDU_ENQ_NUM; j++) {
			pskb = txq->skb_q[j];
			if (pskb != NULL)
				rtw_os_pkt_complete(padapter, pskb);
		}
		_rtw_spinunlock_bh(&txq->txsc_amsdu_lock);

		_rtw_spinlock_free(&txq->txsc_amsdu_lock);
	}

	/* cancel timer */
	_cancel_timer_ex(&psta->txsc_amsdu_vo_timer);
	_cancel_timer_ex(&psta->txsc_amsdu_vi_timer);
	_cancel_timer_ex(&psta->txsc_amsdu_be_timer);
	_cancel_timer_ex(&psta->txsc_amsdu_bk_timer);
}

static void txsc_amsdu_set_timer(struct sta_info *psta, u8 ac)
{
	_timer *txsc_amsdu_timer = NULL;

	switch (ac) {
	case 0:
		txsc_amsdu_timer = &psta->txsc_amsdu_vo_timer;
		break;

	case 1:
		txsc_amsdu_timer = &psta->txsc_amsdu_vi_timer;
		break;

	case 2:
		txsc_amsdu_timer = &psta->txsc_amsdu_be_timer;
		break;

	case 3:
		txsc_amsdu_timer = &psta->txsc_amsdu_bk_timer;
		break;

	default:
		txsc_amsdu_timer = &psta->txsc_amsdu_be_timer;
		break;
	}

	_set_timer(txsc_amsdu_timer, 1);
}

static u8 txsc_amsdu_get_timer_status(struct sta_info *psta, u8 ac)
{
	u8 status = TXSC_AMSDU_TIMER_UNSET;

	switch (ac) {
	case 0:
		status = psta->txsc_amsdu_vo_timeout_sts;
		break;

	case 1:
		status = psta->txsc_amsdu_vi_timeout_sts;
		break;

	case 2:
		status = psta->txsc_amsdu_be_timeout_sts;
		break;

	case 3:
		status = psta->txsc_amsdu_bk_timeout_sts;
		break;

	default:
		status = psta->txsc_amsdu_be_timeout_sts;
		break;
	}

	return status;
}

static void txsc_amsdu_set_timer_status(struct sta_info *psta, u8 ac, u8 status)
{
	switch (ac) {
	case 0:
		psta->txsc_amsdu_vo_timeout_sts = status;
		break;

	case 1:
		psta->txsc_amsdu_vi_timeout_sts = status;
		break;

	case 2:
		psta->txsc_amsdu_be_timeout_sts = status;
		break;

	case 3:
		psta->txsc_amsdu_bk_timeout_sts = status;
		break;

	default:
		psta->txsc_amsdu_be_timeout_sts = status;
		break;
	}
}

static u8 _up_to_qid(u8 up)
{
	u8 ac = 0;

	switch (up) {

	case 1:
	case 2:
		ac = 3;/* bk */
		break;

	case 4:
	case 5:
		ac = 1;/* vi */
		break;

	case 6:
	case 7:
		ac = 0;/* vo */
		break;

	case 0:
	case 3:
	default:
		ac = 2;/* be */
		break;
	}

	return ac;
}

static u8 txsc_amsdu_dequeue(_adapter *padapter, struct txsc_pkt_entry *txsc_pkt)
{
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct sta_info *psta = txsc_pkt->psta;
	struct txsc_amsdu_swq *txq = NULL;
	u8 deq_cnt = 0, i, timer_sts, priority = 0;
	u8 ac = txsc_pkt->ac;
	u8 amsdu_num = 0;

	txq = &psta->amsdu_txq[ac];

	_rtw_spinlock_bh(&txq->txsc_amsdu_lock);

	if (psta->state & WIFI_SLEEP_STATE) {
		pxmitpriv->cnt_txsc_amsdu_deq_ps++;
		goto abort_deq;
	}

	amsdu_num = psta->txsc_amsdu_num;
	if (amsdu_num == 0)
		amsdu_num = padapter->tx_amsdu;

	/* for amsdu timout */
	if (txsc_pkt->is_amsdu_timeout && txq->cnt < amsdu_num)
		amsdu_num = txq->cnt;

	for (i = 0; i < amsdu_num; i++) {
		if (txq->skb_q[txq->rptr] == NULL) {
			/* RTW_ERR("txsc_amsdu_dequeue: deq but skb = NULL\n"); */
			break;
		}

			txsc_pkt->xmit_skb[i] = txq->skb_q[txq->rptr];
			txq->skb_q[txq->rptr] = NULL;
			txq->rptr = (txq->rptr + 1) % MAX_AMSDU_ENQ_NUM;
			txq->cnt--;

			deq_cnt++;
		}

	/* update priority when timeout */
	if (txsc_pkt->is_amsdu_timeout && deq_cnt > 0) {
			priority = *(txsc_pkt->xmit_skb[0]->data + ETH_HLEN + 1);
			txsc_pkt->priority = tos_to_up(priority);
		}

abort_deq:

	timer_sts = txsc_amsdu_get_timer_status(psta, ac);
	if (txq->cnt > 0 && timer_sts != TXSC_AMSDU_TIMER_SETTING) {
		txsc_amsdu_set_timer(psta, ac);
		txsc_amsdu_set_timer_status(psta, ac, TXSC_AMSDU_TIMER_SETTING);
	}

	pxmitpriv->cnt_txsc_amsdu_deq[ac] += deq_cnt;

	txsc_pkt->skb_cnt = deq_cnt;

	_rtw_spinunlock_bh(&txq->txsc_amsdu_lock);

	return deq_cnt;
}

u8 txsc_amsdu_enqueue(_adapter *padapter, struct txsc_pkt_entry *txsc_pkt, u8 *status)
{
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct sta_info *psta = txsc_pkt->psta;
	struct sk_buff *pskb = txsc_pkt->xmit_skb[0];
	struct txsc_amsdu_swq *txq = NULL;
	u8 i, ac = 0, res = _SUCCESS, timer_sts;
	u8 amsdu_num = 0;

	if (!pxmitpriv->txsc_amsdu_enable || !pskb || !psta)
		return _FAIL;

	if (txsc_pkt->step != TXSC_AMSDU_APPLY)
		return _FAIL;

	/* get tx amsdu swq */
	ac = _up_to_qid(txsc_pkt->priority);
	txq = &psta->amsdu_txq[ac];
	txsc_pkt->ac = ac;

	amsdu_num = psta->txsc_amsdu_num;
	if (amsdu_num == 0)
		amsdu_num = padapter->tx_amsdu;

	_rtw_spinlock_bh(&txq->txsc_amsdu_lock);

	if (txq->cnt < MAX_AMSDU_ENQ_NUM) {
		txq->skb_q[txq->wptr] = pskb;
		txq->wptr = (txq->wptr + 1) % MAX_AMSDU_ENQ_NUM;
		txq->cnt++;

		pxmitpriv->cnt_txsc_amsdu_enq[ac]++;

		txsc_pkt->xmit_skb[0] = NULL;

		/* if sta in ps mode, keep queuing and nit deq */
		if (txq->cnt < amsdu_num || (psta->state & WIFI_SLEEP_STATE)){
			*status = TXSC_AMSDU_ENQ_SUCCESS;
			if (psta->state & WIFI_SLEEP_STATE)
				pxmitpriv->cnt_txsc_amsdu_enq_ps++;
		}
	} else {
		pxmitpriv->cnt_txsc_amsdu_enq_abort[ac]++;
		*status = TXSC_AMSDU_ENQ_ABORT;
		txsc_pkt->amsdu = false;
	}

	/* set amsdu timer */
	timer_sts = txsc_amsdu_get_timer_status(psta, ac);
	if (txq->cnt > 0 && timer_sts != TXSC_AMSDU_TIMER_SETTING) {
		txsc_amsdu_set_timer(psta, ac);
		txsc_amsdu_set_timer_status(psta, ac, TXSC_AMSDU_TIMER_SETTING);
	}

	_rtw_spinunlock_bh(&txq->txsc_amsdu_lock);

abort:
	return _SUCCESS;
}

static void txsc_amsdu_timeout_init_pkt_entry(_adapter *padapter, struct txsc_pkt_entry *txsc_pkt, struct sta_info *psta, u8 ac)
{
	u8 i = 0;

	txsc_pkt->step = TXSC_NONE;
	txsc_pkt->txsc_id = 0xff;
	txsc_pkt->ptxreq = NULL;

	for (i = 0; i < MAX_TXSC_SKB_NUM; i++)
		txsc_pkt->xmit_skb[i] = NULL;

	txsc_pkt->psta = psta;
	txsc_pkt->skb_cnt = 0;/* this is for amsdu timeout */

	txsc_pkt->amsdu = false;
	txsc_pkt->ac = ac;
	txsc_pkt->is_amsdu_timeout = true;
}

static void ieee8023_header_to_rfc1042(struct sk_buff *skb, bool add_pad)
{
	void *data;
	int pad;
	__be16 len;
	const int headroom = SNAP_SIZE + 2;

	if (!skb)
		return;

	if (skb_headroom(skb) < headroom) {
		RTW_WARN("%s: headroom=%d isn't enough\n", __func__, skb_headroom(skb));
		if (pskb_expand_head(skb, headroom, 0, GFP_ATOMIC)) {
			RTW_ERR("%s: no headroom=%d for skb\n",
				__func__, headroom);
			return;
		}
	}

	data = skb_push(skb, SNAP_SIZE + 2);
	memmove(data, data + SNAP_SIZE + 2, 2 * ETH_ALEN);

	data += 2 * ETH_ALEN;
	len = cpu_to_be16(skb->len - 2 * ETH_ALEN - 2);
	memcpy(data, &len, 2);
	memcpy(data + 2, rtw_rfc1042_header, SNAP_SIZE);

	if (add_pad && (skb->len & (4 - 1))) {
		pad = 4 - (skb->len & (4 - 1));
		if (skb_tailroom(skb) < pad) {
			RTW_ERR("%s: no tailroom=%d for skb\n",
				__func__, pad);
			return;
		}
		rtw_skb_put_zero(skb, pad);
	}
}

void txsc_amsdu_sta_init(_adapter *padapter, struct sta_info* psta)
{
	if (psta->hepriv.he_option == _TRUE || psta->vhtpriv.vht_option == _TRUE)
		psta->txsc_amsdu_max = psta->txsc_amsdu_num = padapter->tx_amsdu;
	else if (psta->htpriv.ht_option == _TRUE)
		psta->txsc_amsdu_max = psta->txsc_amsdu_num = 2; /* Max amsdu num in N mode is 2 */
	else
		psta->txsc_amsdu_max = 0; /* disable tx amsdu */
}
#endif /* CONFIG_TXSC_AMSDU */

void txsc_init(_adapter *padapter)
{
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;

	pxmitpriv->txsc_enable = 1; /* default TXSC on */

#ifdef CONFIG_TXSC_AMSDU
	pxmitpriv->txsc_amsdu_enable = 1; /* default SW amsdu on */
	//pxmitpriv->txsc_amsdu_force_num = padapter->tx_amsdu; /* default force to max amsdu num */
	padapter->tx_amsdu = 2;
#endif
}

void txsc_clear(_adapter *padapter)
{
	struct sta_priv	*pstapriv = &padapter->stapriv;
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct sta_info *psta = NULL;
	int i, tmp = 0;
	_list	*plist, *phead;

	RTW_PRINT("[TXSC] clear txsc entry\n");

	_rtw_spinlock_bh(&pxmitpriv->txsc_lock);

	if (pxmitpriv->txsc_enable) {
		tmp = pxmitpriv->txsc_enable;
		pxmitpriv->txsc_enable = 0;
	}

	pxmitpriv->ptxsc_sta_cached = NULL;

	for (i = 0; i < NUM_STA; i++) {
		phead = &(pstapriv->sta_hash[i]);
		plist = get_next(phead);

		while ((rtw_end_of_queue_search(phead, plist)) == _FALSE) {
			psta = LIST_CONTAINOR(plist, struct sta_info, hash_list);

			plist = get_next(plist);

			psta->txsc_cache_num = 0;
			psta->txsc_cur_idx = 0;
			psta->txsc_cache_idx = 0;
			psta->txsc_cache_hit = 0;
			#ifdef CONFIG_TXSC_AMSDU
			psta->txsc_amsdu_hit = 0;
			#endif
			psta->txsc_cache_miss = 0;
			psta->txsc_path_slow = 0;
			_rtw_memset(psta->txsc_entry_cache, 0x0, sizeof(struct txsc_entry) * CORE_TXSC_ENTRY_NUM);

		}
	}

	pxmitpriv->txsc_enable = tmp;

	_rtw_spinunlock_bh(&pxmitpriv->txsc_lock);
}

void txsc_dump(_adapter *padapter, void *m)
{
	struct sta_priv	*pstapriv = &padapter->stapriv;
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct sta_info *psta = NULL;
	int i, j;
	_list	*plist, *phead;

	RTW_PRINT_SEL(m, "[txsc][core] (txsc,enable) txsc_enable:%d\n", pxmitpriv->txsc_enable);

	RTW_PRINT_SEL(m, "\n");
	for (i = 0; i < NUM_STA; i++) {
		phead = &(pstapriv->sta_hash[i]);
		plist = get_next(phead);

		while ((rtw_end_of_queue_search(phead, plist)) == _FALSE) {
			psta = LIST_CONTAINOR(plist, struct sta_info, hash_list);

			plist = get_next(plist);

			RTW_PRINT_SEL(m, "[%d] STA[%02x:%02x:%02x:%02x:%02x:%02x]\n", i,
				psta->phl_sta->mac_addr[0], psta->phl_sta->mac_addr[1], psta->phl_sta->mac_addr[2],
				psta->phl_sta->mac_addr[3], psta->phl_sta->mac_addr[4], psta->phl_sta->mac_addr[5]);
			RTW_PRINT_SEL(m, "[txsc] cur_idx:%d\n", psta->txsc_cur_idx);
			RTW_PRINT_SEL(m, "[txsc][core] txsc_path_slow:%d\n", psta->txsc_path_slow);
			RTW_PRINT_SEL(m, "[txsc][core] txsc_path_ps:%d\n", psta->txsc_path_ps);
			RTW_PRINT_SEL(m, "[txsc][core] txsc_cache_hit:%d\n", psta->txsc_cache_hit);
			#ifdef CONFIG_TXSC_AMSDU
			RTW_PRINT_SEL(m, "[txsc][core] txsc_amsdu_hit:%d\n", psta->txsc_amsdu_hit);
			#endif
			RTW_PRINT_SEL(m, "[txsc][core] txsc_cache_miss:%d\n", psta->txsc_cache_miss);
			RTW_PRINT_SEL(m, "\n");
			for (j = 0 ; j < CORE_TXSC_ENTRY_NUM; j++) {
				if (!psta->txsc_entry_cache[j].txsc_is_used)
					continue;
				RTW_PRINT_SEL(m, " [%d][txsc][core] txsc_core_hit:%d\n", j, psta->txsc_entry_cache[j].txsc_cache_hit);
				#ifdef CONFIG_PHL_TXSC
				RTW_PRINT_SEL(m, " [%d][txsc][phl] txsc_phl_hit:%d\n", j, psta->phl_sta->phl_txsc[j].txsc_cache_hit);
				#endif
				#ifdef CONFIG_TXSC_AMSDU
				RTW_PRINT_SEL(m, " [%d][txsc] txsc_amsdu:%d\n", j, psta->txsc_entry_cache[j].txsc_amsdu);
				#endif
				RTW_PRINT_SEL(m, "\n");
			}
		}
	}
}

void txsc_dump_data(u8 *buf, u16 buf_len, const char *prefix)
{
	int i = 0, j;

	RTW_PRINT("[txsc_dump] [%s (%uB)]@%p:\n", prefix, buf_len, buf);

	if (buf == NULL) {
		RTW_PRINT("[txsc_dump] NULL!\n");
		return;
	}

	while (i < buf_len) {
		RTW_PRINT("[txsc_dump] %04X -", i);
		for (j = 0; (j < 4) && (i < buf_len); j++, i += 4)
			RTW_PRINT("  %02X %02X %02X %02X", buf[i], buf[i+1], buf[i+2], buf[i+3]);
		RTW_PRINT("\n");
	}
}

#ifdef CONFIG_PCI_HCI
void txsc_recycle_txreq_phyaddr(_adapter *padapter, struct rtw_xmit_req *txreq)
{
	PPCI_DATA pci_data = dvobj_to_pci(padapter->dvobj);
	struct pci_dev *pdev = pci_data->ppcidev;
	struct rtw_pkt_buf_list *pkt_list = (struct rtw_pkt_buf_list *)txreq->pkt_list;
	dma_addr_t phy_addr = 0;

	/* only recycle pkt_list[1] = skb->data for SW TXSC */
	pkt_list++;

	phy_addr = (pkt_list->phy_addr_l);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	phy_addr |= ((u64)pkt_list->phy_addr_h << 32);
#endif
	pci_unmap_bus_addr(pdev, &phy_addr, pkt_list->length, DMA_TO_DEVICE);
}

void txsc_fill_txreq_phyaddr(_adapter *padapter, struct rtw_pkt_buf_list *pkt_list)
{
	PPCI_DATA pci_data = dvobj_to_pci(padapter->dvobj);
	struct pci_dev *pdev = pci_data->ppcidev;
	dma_addr_t phy_addr = 0;

	pci_get_bus_addr(pdev, pkt_list->vir_addr, &phy_addr, pkt_list->length, DMA_TO_DEVICE);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	pkt_list->phy_addr_h =  phy_addr >> 32;
#else
	pkt_list->phy_addr_h = 0x0;
#endif
	pkt_list->phy_addr_l = phy_addr & 0xFFFFFFFF;
}
#endif

static void txsc_init_pkt_entry(_adapter *padapter, struct sk_buff *pskb, struct txsc_pkt_entry *txsc_pkt)
{
	u8 priority = 0, i;

	txsc_pkt->step = TXSC_NONE;
	txsc_pkt->txsc_id = 0xff;
	txsc_pkt->ptxreq = NULL;

	for (i = 0; i < MAX_TXSC_SKB_NUM; i++)
		txsc_pkt->xmit_skb[i] = NULL;

	txsc_pkt->psta = NULL;
	txsc_pkt->xmit_skb[0] = pskb;
	txsc_pkt->skb_cnt = 1;

#ifdef CONFIG_TXSC_AMSDU
	txsc_pkt->amsdu = false;
	txsc_pkt->ac = 2; /* default be */
	txsc_pkt->is_amsdu_timeout = false;
#endif

	priority = *(pskb->data + ETH_HLEN + 1);
	txsc_pkt->priority = tos_to_up(priority);
}

static void txsc_add_sc_check(_adapter *padapter, struct xmit_frame *pxframe, struct txsc_pkt_entry *txsc_pkt)
{
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct pkt_attrib *pattrib = &pxframe->attrib;
	struct sta_info *psta = pxframe->attrib.psta;

	if (!pxmitpriv->txsc_enable) {
		if (psta->txsc_cache_num > 0)
			txsc_pkt->step = TXSC_SKIP;
		else
			txsc_pkt->step = TXSC_NONE;
		goto exit;
	}

	if (pxframe->attrib.nr_frags > 1 || pxframe->attrib.bswenc == 1)
		goto exit;

	if (txsc_pkt->step != TXSC_NONE)
		goto exit;

	if (pattrib->qos_en &&
		pattrib->ampdu_en == 1 &&
		pattrib->ether_type == ETH_P_IP &&
		!IS_MCAST(pattrib->ra) &&
		!pattrib->icmp_pkt &&
		!pattrib->dhcp_pkt) {

		RTW_PRINT("[%s] sta[%02x] add eth_type=0x%x pkt to txsc\n",
			__func__, pattrib->psta->phl_sta->mac_addr[5], pattrib->ether_type);

		txsc_pkt->step = TXSC_ADD;
	}

exit:
	return;
}

static u8 txsc_get_sc_entry(_adapter *padapter, struct sk_buff *pskb, struct txsc_pkt_entry *txsc_pkt)
{
	struct sta_priv	*pstapriv = &padapter->stapriv;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct sta_info *psta = NULL;
	u8 *ptxsc_ethdr = NULL;
	u8 i, res, da[6], offset, res2 = 0, sta_hit = 0;

	res = _FAIL;
	offset = 6;
	txsc_pkt->is_sta_sleep = 0;

	if (!pxmitpriv->txsc_enable)
		return res;

#ifdef CONFIG_TXSC_AMSDU
	if (txsc_pkt->is_amsdu_timeout) {
		psta = txsc_pkt->psta;
		goto get_txsc_entry;
	}
#endif

	if (pxmitpriv->ptxsc_sta_cached) {
		if (pxmitpriv->ptxsc_sta_cached->phl_sta)
			sta_hit = _rtw_memcmp(pxmitpriv->ptxsc_sta_cached->phl_sta->mac_addr, pskb->data, 6);

		if (sta_hit)
			psta = pxmitpriv->ptxsc_sta_cached;
	}

	if (!sta_hit) {
		_rtw_memcpy(da, pskb->data, 6);
		if (IS_MCAST(da)) {
			res = _FAIL;
		} else {
			if (check_fwstate(pmlmepriv, WIFI_AP_STATE))
				psta = rtw_get_stainfo(pstapriv, da);
			else if (check_fwstate(pmlmepriv, WIFI_STATION_STATE))
				psta = rtw_get_stainfo(pstapriv, get_bssid(pmlmepriv));

			if (!psta)
				res = _FAIL;
		}
	}

	if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) && IS_MCAST(pskb->data)) {
		txsc_pkt->step = TXSC_SKIP;
		goto exit;
	}
	if (!psta) {
		res = _FAIL;
		//RTW_INFO("%s get sta fail\n", __func__);
		goto exit;
	}

#ifdef CONFIG_TXSC_AMSDU
get_txsc_entry:
#endif

	/* skip power saving mode */
	if (psta->state & WIFI_SLEEP_STATE) {
		res = _FAIL;
		txsc_pkt->is_sta_sleep = 1;
		txsc_pkt->step = TXSC_SKIP;
		psta->txsc_path_ps++;
		RTW_INFO("sta sleep state, goto exit\n");
		goto exit;
	}

	if (psta->txsc_cache_num == 0) {
		res = _FAIL;
		goto exit;
	}

	pxmitpriv->ptxsc_sta_cached = psta;
	txsc_pkt->step = TXSC_NONE;
	ptxsc_ethdr = (u8 *)&psta->txsc_entry_cache[psta->txsc_cache_idx].txsc_ethdr;
	res2 = _rtw_memcmp((pskb->data + offset), (ptxsc_ethdr + offset), (ETH_HLEN - offset));

	if (res2 &&
		(pskb->len <= psta->txsc_entry_cache[psta->txsc_cache_idx].txsc_frag_len)) {

#ifdef CONFIG_TXSC_AMSDU
		if (pxmitpriv->txsc_amsdu_enable &&
			psta->txsc_entry_cache[psta->txsc_cache_idx].txsc_amsdu &&
			(psta->txsc_amsdu_num || txsc_pkt->is_amsdu_timeout))
			txsc_pkt->step = TXSC_AMSDU_APPLY;
		else
#endif
			txsc_pkt->step = TXSC_APPLY;

		txsc_pkt->psta = psta;
		txsc_pkt->txsc_id = psta->txsc_cache_idx;
		res = _SUCCESS;
	} else {

		for (i = 0; i < CORE_TXSC_ENTRY_NUM; i++) {
			if (i != psta->txsc_cache_idx && psta->txsc_entry_cache[i].txsc_is_used) {

				ptxsc_ethdr = (u8 *)&(psta->txsc_entry_cache[i].txsc_ethdr);
				if (_rtw_memcmp((pskb->data + offset), (ptxsc_ethdr + offset), (ETH_HLEN - offset)) &&
					(pskb->len <= psta->txsc_entry_cache[i].txsc_frag_len)) {
#ifdef CONFIG_TXSC_AMSDU
					if (pxmitpriv->txsc_amsdu_enable &&
						psta->txsc_entry_cache[i].txsc_amsdu &&
						(psta->txsc_amsdu_num || txsc_pkt->is_amsdu_timeout))
						txsc_pkt->step = TXSC_AMSDU_APPLY;
					else
#endif
						txsc_pkt->step = TXSC_APPLY;

					txsc_pkt->txsc_id = i;
					txsc_pkt->psta = psta;
					psta->txsc_cache_idx = i;
					res = _SUCCESS;

					break;
				}
			}
		}
	}

#ifdef CONFIG_TXSC_AMSDU
	if (txsc_pkt->step == TXSC_AMSDU_APPLY)
		txsc_pkt->amsdu = true;
	else
		txsc_pkt->amsdu = false;
#endif

	/* skip power saving mode */
	if (res == _SUCCESS && (psta->state & WIFI_SLEEP_STATE)) {
	#ifdef CONFIG_TXSC_AMSDU
		if (!txsc_pkt->amsdu) {
			res = _FAIL;
			txsc_pkt->step = TXSC_SKIP;
		}
	#endif
		psta->txsc_path_ps++;
	}

exit:
	return res;
}

static void txsc_prepare_sc_entry(_adapter *padapter, struct xmit_frame *pxframe, struct txsc_pkt_entry *txsc_pkt)
{
	struct sta_priv	*pstapriv = &padapter->stapriv;
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct sta_info *psta = pxframe->attrib.psta;
	struct pkt_attrib *pattrib = &pxframe->attrib;
	struct rtw_xmit_req *txreq = pxframe->phl_txreq;
	struct rtw_pkt_buf_list *pkt_list = (struct rtw_pkt_buf_list *)txreq->pkt_list;
	struct rtw_pkt_buf_list *ptxsc_pkt_list0 = NULL;
	struct rtw_t_meta_data *ptxsc_mdata = NULL;
	struct sk_buff *pskb = txsc_pkt->xmit_skb[0];
	u8 i, idx;
	u8 *ptxsc_ethdr = NULL;
	u8 *ptxsc_wlhdr = NULL;
	u8 *ptxsc_wlhdr_len = NULL;

	if (!psta) {
		RTW_ERR("%s: fetal err, XF_STA = NULL, please check.\n", __func__);
		return;
	}

	if (txsc_pkt->step == TXSC_SKIP) {
		if (pxmitpriv->txsc_enable && psta->txsc_cache_num > 0)
			psta->txsc_cache_miss++;
		else
			psta->txsc_path_slow++;
	} else if (txsc_pkt->step == TXSC_NONE) {
		psta->txsc_path_slow++;
	}

	if (txsc_pkt->step != TXSC_ADD)
		return;

	idx = psta->txsc_cur_idx;

	ptxsc_ethdr = (u8 *)&psta->txsc_entry_cache[idx].txsc_ethdr;
	#ifdef USE_ONE_WLHDR
	ptxsc_wlhdr = psta->txsc_entry_cache[idx].txsc_wlhdr;
	#else
	ptxsc_wlhdr = (u8 *)&psta->txsc_entry_cache[idx].txsc_wlhdr;
	#endif
	ptxsc_pkt_list0 = &psta->txsc_entry_cache[idx].txsc_pkt_list0;
	ptxsc_mdata = &psta->txsc_entry_cache[idx].txsc_mdata;
	ptxsc_wlhdr_len = &psta->txsc_entry_cache[idx].txsc_wlhdr_len;

	_rtw_spinlock_bh(&pxmitpriv->txsc_lock);

	if (psta->txsc_entry_cache[idx].txsc_is_used == 1)
		RTW_PRINT("[CORE_TXSC] txsc entry is full, replace rentry[%d]\n", idx);

	/* ALLOC WLHDR in DMA addr */
	#ifdef USE_ONE_WLHDR
	if (!ptxsc_wlhdr) {
		ptxsc_wlhdr = rtw_zmalloc(CORE_TXSC_WLHDR_SIZE);
		psta->txsc_entry_cache[idx].txsc_wlhdr = ptxsc_wlhdr;
	}
	#endif

	/* ETH HDR */
	_rtw_memcpy(ptxsc_ethdr, pskb->data, ETH_HLEN);

	/* WLAN HDR + LLC */
	_rtw_memcpy(ptxsc_wlhdr, pkt_list->vir_addr, pkt_list->length);
	*ptxsc_wlhdr_len = pkt_list->length;

	/* pkt_list[0] */
	ptxsc_pkt_list0->vir_addr = ptxsc_wlhdr;
	ptxsc_pkt_list0->length = pkt_list->length;
	#ifdef USE_ONE_WLHDR
	#ifdef CONFIG_PCI_HCI
	txsc_fill_txreq_phyaddr(padapter, ptxsc_pkt_list0);
	#endif
	#endif

	/* META DATA */
	_rtw_memcpy(ptxsc_mdata, &txreq->mdata, sizeof(*ptxsc_mdata));

	/* FRAGE_LEN */
	psta->txsc_entry_cache[idx].txsc_frag_len = pxframe->attrib.frag_len_txsc;

	psta->txsc_entry_cache[idx].txsc_is_used = 1;
	psta->txsc_cache_idx = idx;
	psta->txsc_cur_idx = (psta->txsc_cur_idx + 1) % CORE_TXSC_ENTRY_NUM;
	if (psta->txsc_cache_num < CORE_TXSC_ENTRY_NUM)
		psta->txsc_cache_num++;

	psta->txsc_path_slow++;

	pxmitpriv->ptxsc_sta_cached = psta;

	txreq->treq_type = RTW_PHL_TREQ_TYPE_PHL_ADD_TXSC | RTW_PHL_TREQ_TYPE_PHL_UPDATE_TXSC;

	/* set shortcut id  */
	txreq->shortcut_id = idx;
	psta->txsc_entry_cache[idx].txsc_phl_id = idx;

	RTW_PRINT("[CORE_TXSC][ADD] core_txsc_idx:%d(cur_idx:%d), txreq_sc_id:%d, txsc_frag_len:%d\n",
		idx, psta->txsc_cur_idx, txreq->shortcut_id, psta->txsc_entry_cache[idx].txsc_frag_len);

#ifdef CONFIG_TXSC_AMSDU
	if (pxmitpriv->txsc_amsdu_enable && pxframe->attrib.amsdu_ampdu_en)
		psta->txsc_entry_cache[idx].txsc_amsdu = true;
	else
		psta->txsc_entry_cache[idx].txsc_amsdu = false;
#endif

	/* for debug */
	dbg_dump_txreq_mdata(ptxsc_mdata, __func__);

	_rtw_spinunlock_bh(&pxmitpriv->txsc_lock);
}

u8 txsc_get_sc_cached_entry(_adapter *padapter, struct sk_buff *pskb, struct txsc_pkt_entry *txsc_pkt)
{
	txsc_init_pkt_entry(padapter, pskb, txsc_pkt);
	return txsc_get_sc_entry(padapter, pskb, txsc_pkt);
}

void txsc_add_sc_cache_entry(_adapter *padapter, struct xmit_frame *pxframe, struct txsc_pkt_entry *txsc_pkt)
{
	txsc_add_sc_check(padapter, pxframe, txsc_pkt);
	txsc_prepare_sc_entry(padapter, pxframe, txsc_pkt);
}

static void _txsc_update_sec_iv(_adapter *padapter, struct sta_info *psta, struct rtw_xmit_req *txreq)
{
	if (!psta)
		return;

	//RTW_INFO("%s sec_type = %d\n\n", __func__, txreq->mdata.sec_type);
	switch (txreq->mdata.sec_type) {
	case RTW_ENC_WEP40:
	case RTW_ENC_WEP104:
		WEP_IV(psta->iv, psta->dot11txpn, psta->key_idx);
		break;

	case RTW_ENC_TKIP:
		TKIP_IV(psta->iv, psta->dot11txpn, 0);
		break;

	case RTW_ENC_CCMP:
		AES_IV(psta->iv, psta->dot11txpn, 0);
		//RTW_INFO("AES IV \n");
		break;

	case RTW_ENC_GCMP:
	case RTW_ENC_GCMP256:
		GCMP_IV(psta->iv, psta->dot11txpn, 0);

		break;

	case RTW_ENC_CCMP256:
		GCMP_IV(psta->iv, psta->dot11txpn, 0);

		break;

/*
#ifdef CONFIG_WAPI_SUPPORT
	case _SMS4_:
		rtw_wapi_get_iv(padapter, pattrib->ra, pattrib->iv);
		break;
#endif
*/
	default:
		break;
	}

	return;

}

u8 txsc_apply_sc_cached_entry(_adapter *padapter, struct txsc_pkt_entry *txsc_pkt)
{
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct rtw_pkt_buf_list	*pkt_list = NULL;
	struct rtw_pkt_buf_list *pkt_list0 = NULL;
	struct rtw_t_meta_data	*mdata = NULL;
	struct rtw_xmit_req	*txreq = NULL;
	struct xmit_txreq_buf *txreq_buf = NULL;
	struct txsc_entry *txsc;
	struct sta_info *psta = txsc_pkt->psta;
	struct sk_buff *xmit_skb[MAX_TXSC_SKB_NUM];
	u8 *hdr = NULL;
	u16 *qc;
	int wlhdr_copy_len, payload_offset = ETH_HLEN;
	u8 idx, priority = 0, res = _SUCCESS;
	u8 i, skb_cnt = 0, is_hdr_need_update = 0, txsc_wlhdr_len;
	u32 tail_sz, wlan_tail;
	u8 *tail, iv, offset;
	#ifndef USE_ONE_WLHDR
	u8 *head, *ptxsc_wlhdr;
	#endif

	if (!pxmitpriv->txsc_enable || !psta)
		return _SUCCESS;

	if (txsc_pkt->step != TXSC_APPLY &&
		txsc_pkt->step != TXSC_AMSDU_APPLY)
		return _SUCCESS;

	_rtw_memset(xmit_skb, 0x0, sizeof(xmit_skb));

	/* get cached entry */
	idx = txsc_pkt->txsc_id;
	txsc = &psta->txsc_entry_cache[idx];

	pkt_list0 = &txsc->txsc_pkt_list0;
	mdata = &txsc->txsc_mdata;
	#ifdef USE_ONE_WLHDR
	txsc_wlhdr_len = pkt_list0->length;
	#else
	ptxsc_wlhdr = (u8 *)txsc->txsc_wlhdr;
	txsc_wlhdr_len = txsc->txsc_wlhdr_len;
	#endif

	#ifdef USE_ONE_WLHDR
	txreq_buf = (struct xmit_txreq_buf *)get_txreq_buffer(padapter, (u8 **)&txreq, (u8 **)&pkt_list, NULL, NULL);
	#else
	txreq_buf = (struct xmit_txreq_buf *)get_txreq_buffer(padapter, (u8 **)&txreq, (u8 **)&pkt_list, (u8 **)&head, NULL);
	#endif

	if (txreq_buf == NULL) {
		res = _FAIL;
		goto exit;
	}

	/* init txreq buf */
	txreq_buf->adapter = padapter;

	/* fill txreq other */
	txreq->os_priv = (void *)txreq_buf;
	txreq->pkt_list = (u8 *)pkt_list;
	txreq->pkt_cnt = 0;
	txreq->total_len = 0;

	/* fill txsc pkt entry */
	txsc_pkt->ptxreq = txreq;

#ifdef CONFIG_TXSC_AMSDU
	if (txsc_pkt->amsdu) {
		payload_offset = 0;

		/* if amsdu timeout, it is deq already */
		if (txsc_pkt->is_amsdu_timeout)
			skb_cnt = txsc_pkt->skb_cnt;
		else
			skb_cnt = txsc_amsdu_dequeue(padapter, txsc_pkt);

		if (skb_cnt > 0) {
			for (i = 0; i < skb_cnt; i++) {
				xmit_skb[i] = txsc_pkt->xmit_skb[i];
				ieee8023_header_to_rfc1042(xmit_skb[i], ((i + 1) == skb_cnt) ? false : true);
			}
			pxmitpriv->cnt_txsc_amsdu_dump[skb_cnt]++;
		} else {
			pxmitpriv->cnt_txsc_amsdu_dump[skb_cnt]++;
			pxmitpriv->cnt_txsc_amsdu_deq_empty++;
			res = _FAIL;
			goto exit;
		}
	} else
#endif
	{
		/* for no tx_amsdu case */
		xmit_skb[0] = txsc_pkt->xmit_skb[0];
		skb_cnt = txsc_pkt->skb_cnt;
	}

	if (skb_cnt == 0)
		RTW_PRINT("[ERR][%s:%d] skb_cnt = 0 is a fatel error, plz check\n", __func__, __LINE__);

#ifdef CONFIG_TXSC_AMSDU
	/*for avoid amsdu exchange */
	if (!pxmitpriv->txsc_amsdu_enable)
		txsc->txsc_amsdu = false;
#endif

	/* fill_txreq_mdata */
#ifdef TXSC_DBG_COPY_ORI_WLHDR_MDATA 
	if (txsc_pkt->is_spec_pkt) {
        _rtw_memcpy(&txreq->mdata, &txsc_pkt->mdata_ori, sizeof(txreq->mdata));
    } else
#endif
		_rtw_memcpy(&txreq->mdata, mdata, sizeof(txreq->mdata));

	/* Update TID from IP header */
	/* priority = *(xmit_skb[0]->data + ETH_HLEN + 1); */
	/*txreq->mdata.tid = tos_to_up(priority); */
	priority = txreq->mdata.tid = txsc_pkt->priority;
	txreq->mdata.cat = rtw_phl_cvt_tid_to_cat(priority);

	/* SW shortcut --- */
	/* rtw_core_wlan_fill_head */
	/* use swseq in amsdu */
	if (txreq->mdata.hw_seq_mode == 0) {
		/* generate sw seq */
		//priority = txreq->mdata.tid;
		psta->sta_xmitpriv.txseq_tid[priority]++;
		psta->sta_xmitpriv.txseq_tid[priority] &= 0xFFF;
		txreq->mdata.sw_seq = psta->sta_xmitpriv.txseq_tid[priority];

		hdr = txsc->txsc_wlhdr;
		SetSeqNum(hdr, txreq->mdata.sw_seq);

		#ifdef CONFIG_TXSC_AMSDU
			qc = (u16 *)(hdr + WLAN_HDR_A3_LEN);/* set amsdu bit */
			if (txsc_pkt->amsdu && txsc->txsc_amsdu)
			SetAMsdu(qc, 1);
			else
				SetAMsdu(qc, 0);
		#endif

		is_hdr_need_update = 1;
	}

	#ifdef CONFIG_TXSC_AMSDU
	if (txsc_pkt->amsdu && txsc->txsc_amsdu)
		wlhdr_copy_len = (txsc_wlhdr_len - (SNAP_SIZE + sizeof(u16)));
	else
	#endif
		wlhdr_copy_len = txsc_wlhdr_len;

#ifdef TXSC_DBG_COPY_ORI_WLHDR_MDATA
	if (!txsc_pkt->is_spec_pkt) {
#endif
		if (txreq->mdata.sec_hw_enc) {
			if (padapter->dvobj->phl_com->dev_cap.sec_cap.hw_form_hdr &&
				mdata->sec_type != RTW_ENC_WAPI && mdata->hw_sec_iv == 0) {
				txreq->mdata.sec_keyid = psta->key_idx;
				/* Fill PN of IV */
				for (i = 0; i < 6; i++)
					txreq->mdata.iv[i] = (psta->dot11txpn.val >> 8 * i) & 0xff;
				_txsc_update_sec_iv(padapter, psta, txreq);
				RTW_DBG("%s: keyid=%d, iv=" PN_FMT "\n", __func__,
					txreq->mdata.sec_keyid, PN_ARG(txreq->mdata.iv));
			}
			else {
				//RTW_INFO("%s udpate iv, psta->iv_len : %d, psta->dot11txpn : %d\n", __func__, psta->iv_len, psta->dot11txpn);
				//RTW_PRINT_DUMP("[before upadte iv]", ptxsc_wlhdr, wlhdr_copy_len);
				_txsc_update_sec_iv(padapter, psta, txreq);
				offset = wlhdr_copy_len - psta->iv_len  - RTW_SZ_LLC;
				_rtw_memcpy((hdr + offset), psta->iv, psta->iv_len);
				//RTW_INFO("%s offset : %d\n", __func__, offset);
				//RTW_PRINT_DUMP("[IV]", psta->iv, psta->iv_len);
				//RTW_PRINT_DUMP("[after upadte iv]", pkt_list0->vir_addr, wlhdr_copy_len);
			}
		}
#ifdef TXSC_DBG_COPY_ORI_WLHDR_MDATA
	}
#endif

	/* WLAN header from cache */
	#ifdef USE_ONE_WLHDR
	_rtw_memcpy(pkt_list, pkt_list0, sizeof(struct rtw_pkt_buf_list));
	#else
	#ifdef USE_PREV_WLHDR_BUF
	if (txreq_buf->macid == txreq->mdata.macid && txreq_buf->txsc_id == idx) {
		if (is_hdr_need_update) {
			SetSeqNum(head, txreq->mdata.sw_seq);/* set sw seq */
			if (padapter->dvobj->phl_com->dev_cap.sec_cap.hw_form_hdr &&
			    mdata->sec_type != RTW_ENC_WAPI && mdata->hw_sec_iv == 0)
				; /* do nothing if hw_form_hdr */
			else
				_rtw_memcpy((head + offset), psta->iv, psta->iv_len);

			#ifdef CONFIG_TXSC_AMSDU
				qc = (u16 *)(head + WLAN_HDR_A3_LEN);/* set amsdu bit */
			if (txsc_pkt->amsdu && txsc->txsc_amsdu)
					SetAMsdu(qc, 1);/* set amsdu bit */
				else
					SetAMsdu(qc, 0);
			#endif
		}
	} else
	#endif /* USE_PREV_WLHDR_BUF */
	{
	#ifdef TXSC_DBG_COPY_ORI_WLHDR_MDATA
		if (txsc_pkt->is_spec_pkt) {
                        wlhdr_copy_len = txsc_pkt->wlhdr_ori_len;
                        SetSeqNum(txsc_pkt->wlhdr_ori, txsc_pkt->mdata_ori.sw_seq);
                        _rtw_memcpy(head, txsc_pkt->wlhdr_ori, wlhdr_copy_len);
        } else
	#endif
		_rtw_memcpy(head, ptxsc_wlhdr, wlhdr_copy_len);
	}

	qc = (u16 *)(head + WLAN_HDR_A3_LEN);
	SetPriority(qc, priority);

	/* fill wlhdr in pkt_list[0] */
	pkt_list->vir_addr = head;
	pkt_list->length = wlhdr_copy_len;
	#ifdef CONFIG_PCI_HCI
	txsc_fill_txreq_phyaddr(padapter, pkt_list);
	#endif
	#endif/* USE_ONE_WLHDR */

	txreq->total_len += pkt_list->length;
	txreq->pkt_cnt++;

	#ifdef USE_PREV_WLHDR_BUF
	txreq_buf->macid = txreq->mdata.macid;
	txreq_buf->txsc_id = idx;
	#endif/* USE_PREV_WLHDR_BUF */

	/* Payload w.o. ether header */ /* CONFIG_TXSC_AMSDU for multiple skb */
	for (i = 0; i < skb_cnt; i++) {
		pkt_list++;

		pkt_list->vir_addr = xmit_skb[i]->data + payload_offset;
		pkt_list->length = xmit_skb[i]->len - payload_offset;

		txreq->total_len += pkt_list->length;
		txreq->pkt_cnt++;
		
		#ifdef CONFIG_PCI_HCI
		txsc_fill_txreq_phyaddr(padapter, pkt_list);
		#endif

		txreq_buf->pkt[i] = (u8 *)xmit_skb[i];
	}

	txreq->treq_type = RTW_PHL_TREQ_TYPE_CORE_TXSC;

	if (txsc_pkt->step == TXSC_APPLY) {
		psta->txsc_cache_hit++;
		txsc->txsc_cache_hit++;
	}
#ifdef CONFIG_TXSC_AMSDU
	else if (txsc_pkt->step == TXSC_AMSDU_APPLY) {
		psta->txsc_amsdu_hit++;
		txsc->txsc_cache_hit++;
	}
#endif
	else
		psta->txsc_path_slow++;
	/* SW shortcut --- */

	txreq_buf->pkt_cnt = skb_cnt;/* for recycle multiple skb */
	txreq->mdata.pktlen = txreq->total_len;
	txreq->shortcut_id = psta->txsc_entry_cache[idx].txsc_phl_id;

	/* send addbareq */
	txsc_issue_addbareq_cmd(padapter, priority, psta, _TRUE);

	/* fix rate */
	if (padapter->fix_rate != NO_FIX_RATE) {
		mdata->userate_sel = 1;
		mdata->f_rate = GET_FIX_RATE(padapter->fix_rate);
		mdata->f_gi_ltf = GET_FIX_RATE_SGI(padapter->fix_rate);
		if (!padapter->data_fb)
			mdata->dis_data_rate_fb = 1;
	}

#ifdef RTW_PHL_DBG_CMD
	/* Update force rate settings so force rate takes effects
	 * after shortcut cached
	 */
	if (padapter->txForce_enable) {
		if (padapter->txForce_rate != INV_TXFORCE_VAL) {
			txreq->mdata.f_rate = padapter->txForce_rate;
			txreq->mdata.userate_sel = 1;
			txreq->mdata.dis_data_rate_fb = 1;
			txreq->mdata.dis_rts_rate_fb = 1;
		}
		if (padapter->txForce_agg != INV_TXFORCE_VAL)
			txreq->mdata.ampdu_en = padapter->txForce_agg;
		if (padapter->txForce_aggnum != INV_TXFORCE_VAL)
			txreq->mdata.max_agg_num = padapter->txForce_aggnum;
		if (padapter->txForce_gi != INV_TXFORCE_VAL)
			txreq->mdata.f_gi_ltf = padapter->txForce_gi;
		if (padapter->txForce_ampdu_density != INV_TXFORCE_VAL)
			txreq->mdata.ampdu_density = padapter->txForce_ampdu_density;
		if (padapter->txForce_bw != INV_TXFORCE_VAL)
			txreq->mdata.f_bw = padapter->txForce_bw;
	}
#endif /* RTW_PHL_DBG_CMD */

	/* for tx debug */
	dbg_dump_txreq_mdata(&txreq->mdata, __func__);
	_print_txreq_pklist(NULL, txsc_pkt->ptxreq, xmit_skb[0], __func__);

exit:
	return res;
}

void txsc_free_txreq(_adapter *padapter, struct rtw_xmit_req *txreq)
{
	struct xmit_txreq_buf *ptxreq_buf = NULL;
	struct sk_buff *tx_skb = NULL;
	_queue *queue = NULL;
	u8 i;

	if (txreq != NULL)
		ptxreq_buf = txreq->os_priv;
	else
		return;

	#ifdef CONFIG_PCI_HCI
	#ifdef USE_ONE_WLHDR
	txsc_recycle_txreq_phyaddr(padapter, txreq);
	#else
	core_recycle_txreq_phyaddr(padapter, txreq);
	#endif
	#endif

	if (!ptxreq_buf) {
		RTW_ERR("%s: NULL ptxreq_buf !!\n", __func__);
		rtw_warn_on(1);
		return;
	}

	queue = &padapter->free_txreq_queue;
	_rtw_spinlock_bh(&queue->lock);

	txreq->os_priv = NULL;
	txreq->pkt_list = NULL;
	txreq->treq_type = RTW_PHL_TREQ_TYPE_NORMAL;

	for (i = 0; i < ptxreq_buf->pkt_cnt; i++) {
		tx_skb = (struct sk_buff *)ptxreq_buf->pkt[i];
		if (tx_skb)
			rtw_os_pkt_complete(padapter, tx_skb);
		else
			RTW_DBG("%s:tx recyele: tx_skb=NULL\n", __func__);
		/* ptxreq_buf->pkt[i] = NULL; */
	}

	rtw_list_delete(&ptxreq_buf->list);
	rtw_list_insert_tail(&ptxreq_buf->list, get_list_head(queue));
	padapter->free_txreq_cnt++;

	_rtw_spinunlock_bh(&queue->lock);
}

#if 0/* tmp mark this debug function */
void txsc_debug_sc_entry(_adapter *padapter, struct xmit_frame *pxframe, struct txsc_pkt_entry *txsc_pkt)
{
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct rtw_pkt_buf_list *pkt_list = NULL;
	struct rtw_pkt_buf_list *txsc_pkt_list = NULL;
	struct rtw_xmit_req *txreq = pxframe->phl_txreq;
	struct rtw_xmit_req *ptxsc_txreq = txsc_pkt->ptxreq;
	u8 i;

	if (txsc_pkt->step != TXSC_DEBUG)
		return;

	if (pxmitpriv->txsc_debug_mask&BIT2) {

		RTW_PRINT("\n\nNormal Path TXREQ: %p\n\n", txreq);
		txsc_dump_data((u8 *)txreq, sizeof(struct rtw_xmit_req), "txreq:");
		RTW_PRINT("os_priv=%p\n", txreq->os_priv);
		RTW_PRINT("treq_type=%d shortcut_id=%d\n", txreq->treq_type, txreq->shortcut_id);
		RTW_PRINT("total_len=%d pkt_cnt=%d\n", txreq->total_len, txreq->pkt_cnt);
		pkt_list = (struct rtw_pkt_buf_list *)txreq->pkt_list;
		for (i = 0; i < txreq->pkt_cnt; i++) {
			RTW_PRINT("[%d]", i);
			txsc_dump_data((u8 *)pkt_list, sizeof(struct rtw_pkt_buf_list), "pklist");
			txsc_dump_data((u8 *)pkt_list->vir_addr, pkt_list->length, "pkt_list->vir_addr");
			pkt_list++;
		}
		RTW_PRINT("mdata: pktlen=%d sw_seq=%d\n", txreq->mdata.pktlen, txreq->mdata.sw_seq);

		RTW_PRINT("\n\nShortcut Path TXREQ: %p\n\n", ptxsc_txreq);
		txsc_dump_data((u8 *)ptxsc_txreq, sizeof(struct rtw_xmit_req), "ptxsc_txreq:");
		RTW_PRINT("os_priv=%p\n", ptxsc_txreq->os_priv);
		RTW_PRINT("treq_type=%d shortcut_id=%d\n", ptxsc_txreq->treq_type, ptxsc_txreq->shortcut_id);
		RTW_PRINT("total_len=%d pkt_cnt=%d\n", ptxsc_txreq->total_len, ptxsc_txreq->pkt_cnt);
		txsc_pkt_list = (struct rtw_pkt_buf_list *)ptxsc_txreq->pkt_list;
		for (i = 0; i < txreq->pkt_cnt; i++) {
			RTW_PRINT("[%d]", i);
			txsc_dump_data((u8 *)txsc_pkt_list, sizeof(struct rtw_pkt_buf_list), "pklist");
			txsc_dump_data((u8 *)txsc_pkt_list->vir_addr, txsc_pkt_list->length, "pkt_list->vir_addr");
			txsc_pkt_list++;
		}
		RTW_PRINT("mdata: pktlen=%d sw_seq=%d\n", ptxsc_txreq->mdata.pktlen, ptxsc_txreq->mdata.sw_seq);

	} else {
		if (!_rtw_memcmp(&txreq->mdata, &ptxsc_txreq->mdata, sizeof(struct rtw_t_meta_data))) {
			txsc_dump_data((u8 *)&txreq->mdata, sizeof(struct rtw_t_meta_data), "txreq->mdata");
			txsc_dump_data((u8 *)&ptxsc_txreq->mdata, sizeof(struct rtw_t_meta_data), "ptxsc_txreq->mdata");
		}

		pkt_list = (struct rtw_pkt_buf_list *)txreq->pkt_list;
		txsc_pkt_list = (struct rtw_pkt_buf_list *)ptxsc_txreq->pkt_list;
		if (pkt_list->length != txsc_pkt_list->length) {
			txsc_dump_data((u8 *)pkt_list, sizeof(struct rtw_pkt_buf_list), "pkt_list[0]");
			txsc_dump_data((u8 *)txsc_pkt_list, sizeof(struct rtw_pkt_buf_list), "txsc_pkt_list[0]");
			txsc_dump_data((u8 *)pkt_list->vir_addr, pkt_list->length, "pkt_list[0]->vir_addr");
			txsc_dump_data(txsc_pkt_list->vir_addr, pkt_list->length, "txsc_pkt_list[0]->vir_addr");
		} else if (pkt_list->length == txsc_pkt_list->length) {
			if (!_rtw_memcmp(pkt_list->vir_addr, txsc_pkt_list->vir_addr, pkt_list->length)) {
				txsc_dump_data((u8 *)pkt_list->vir_addr, pkt_list->length, "pkt_list[0]->vir_addr");
				txsc_dump_data((u8 *)txsc_pkt_list->vir_addr, pkt_list->length, "txsc_pkt_list[0]->vir_addr");
			}
		}
	}

	/* DO NOT WD CACHE */
	txreq->shortcut_id = 0;
	txreq->treq_type = RTW_PHL_TREQ_TYPE_NORMAL;
}
#endif

#ifdef CONFIG_TXSC_AMSDU
void txsc_amsdu_clear(_adapter *padapter)
{
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	int i;

	RTW_PRINT("[amsdu] clear amsdu counter\n");

	for (i = 0; i < 4; i++) {
		pxmitpriv->cnt_txsc_amsdu_enq[i] = 0;
		pxmitpriv->cnt_txsc_amsdu_enq_abort[i] = 0;
		pxmitpriv->cnt_txsc_amsdu_deq[i] = 0;
	}
	pxmitpriv->cnt_txsc_amsdu_deq_empty = 0;
	//pxmitpriv->cnt_txsc_amsdu_deq_ampdu = 0;
	pxmitpriv->cnt_txsc_amsdu_enq_ps = 0;
	pxmitpriv->cnt_txsc_amsdu_deq_ps = 0;
	pxmitpriv->cnt_txsc_amsdu_timeout_deq_empty = 0;

	for (i = 0; i < (MAX_TXSC_SKB_NUM + 1); i++) {
		pxmitpriv->cnt_txsc_amsdu_dump[i] = 0;
		pxmitpriv->cnt_txsc_amsdu_timeout_dump[i] = 0;
	}
	for(i = 0; i < ARRAY_SIZE(pxmitpriv->cnt_txsc_amsdu_timeout_ok); i++) {
		pxmitpriv->cnt_txsc_amsdu_timeout_ok[i] = 0;
		pxmitpriv->cnt_txsc_amsdu_timeout_fail[i] = 0;
	}
}
void txsc_amsdu_dump(_adapter *padapter, void *m)
{
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);
	//_queue *queue = &dvobj->txsc_amsdu_timeout_queue;
	struct sta_priv	*pstapriv = &padapter->stapriv;
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct sta_info *psta = NULL;
	u8 i, j;
	_list *plist, *phead;

	RTW_PRINT_SEL(m, "[amsdu] txsc amsdu enable: %d\n", pxmitpriv->txsc_amsdu_enable);
	//RTW_PRINT_SEL(m, "[amsdu] txsc amsdu mode: %d\n", pxmitpriv->txsc_amsdu_mode);
	RTW_PRINT_SEL(m, "[amsdu] amsdu num: %d\n", padapter->tx_amsdu);
	RTW_PRINT_SEL(m, "[amsdu] amsdu rate: %d\n", padapter->tx_amsdu_rate);
	RTW_PRINT_SEL(m, "[amsdu] txsc_amsdu_force_num: %d\n", pxmitpriv->txsc_amsdu_force_num);
	RTW_PRINT_SEL(m, "[amsdu] amsdu max num: %d\n", MAX_TXSC_SKB_NUM);
	RTW_PRINT_SEL(m, "[amsdu] amsdu max queue num: %d\n", MAX_AMSDU_ENQ_NUM);

	RTW_PRINT_SEL(m, "\n");
	for (i = 0; i < 4; i++) {
		if (pxmitpriv->cnt_txsc_amsdu_enq[i] != 0 ||
			pxmitpriv->cnt_txsc_amsdu_deq[i] != 0 ||
			pxmitpriv->cnt_txsc_amsdu_enq_abort[i] != 0) {
			RTW_PRINT_SEL(m, "[amsdu] ac[%d] enq/deq/abort: %llu / %llu / %llu\n", i,
				pxmitpriv->cnt_txsc_amsdu_enq[i],
				pxmitpriv->cnt_txsc_amsdu_deq[i],
				pxmitpriv->cnt_txsc_amsdu_enq_abort[i]);
		}
	}

	RTW_PRINT_SEL(m, "\n");
	for (i = 0; i < (MAX_TXSC_SKB_NUM + 1); i++) {
		if (pxmitpriv->cnt_txsc_amsdu_dump[i] != 0)
			RTW_PRINT_SEL(m, "[amsdu] pkt_num:%d tx: %llu\n", i, pxmitpriv->cnt_txsc_amsdu_dump[i]);
	}
	/*RTW_PRINT_SEL(m, "[amsdu] deq empty/ampdu: %d / %d\n",
		pxmitpriv->cnt_txsc_amsdu_deq_empty, pxmitpriv->cnt_txsc_amsdu_deq_ampdu);*/
	RTW_PRINT_SEL(m, "[amsdu] ps enq/deq: %llu / %llu\n",
		pxmitpriv->cnt_txsc_amsdu_enq_ps, pxmitpriv->cnt_txsc_amsdu_deq_ps);

	RTW_PRINT_SEL(m, "\n");
	for (i = 0; i < (MAX_TXSC_SKB_NUM + 1); i++) {
		if (pxmitpriv->cnt_txsc_amsdu_timeout_dump[i] != 0)
			RTW_PRINT_SEL(m, "[amsdu] pkt_num:%d timout_tx: %llu\n", i, pxmitpriv->cnt_txsc_amsdu_timeout_dump[i]);
	}
	RTW_PRINT_SEL(m, "[amsdu] timeout_deq_empty: %llu\n", pxmitpriv->cnt_txsc_amsdu_timeout_deq_empty);

	RTW_PRINT_SEL(m, "\n");
	for (i = 0; i < ARRAY_SIZE(pxmitpriv->cnt_txsc_amsdu_timeout_ok); i++) {
		if (pxmitpriv->cnt_txsc_amsdu_timeout_ok[i] != 0 ||
			pxmitpriv->cnt_txsc_amsdu_timeout_fail[i] != 0)
			RTW_PRINT_SEL(m, "[amsdu] ac[%d] timeout ok/fail: %llu / %llu\n", i,
				pxmitpriv->cnt_txsc_amsdu_timeout_ok[i],
				pxmitpriv->cnt_txsc_amsdu_timeout_fail[i]);
	}

	RTW_PRINT_SEL(m, "\n");
	/*RTW_PRINT_SEL(m, "[amsdu] dvobj->txsc_amsdu_timeout_use_hw_timer: %d\n",
	              dvobj->txsc_amsdu_timeout_use_hw_timer);
	RTW_PRINT_SEL(m, "[amsdu] dvobj->txsc_amsdu_timeout_queue: empty/cnt: %d / %d\n",
	              _rtw_queue_empty(queue), queue->qlen);
	for (j = 0; j < N_TIMEOUT; j++)
		RTW_PRINT_SEL(m, "[amsdu] dvobj->txsc_amsdu_timeout[%u]: %u\n",
		              j, dvobj->txsc_amsdu_timeout[j]);*/
	RTW_PRINT_SEL(m, "\n");
	for (i = 0; i < NUM_STA; i++) {
		phead = &(pstapriv->sta_hash[i]);
		plist = get_next(phead);

		while ((rtw_end_of_queue_search(phead, plist)) == _FALSE) {
			psta = LIST_CONTAINOR(plist, struct sta_info, hash_list);
			plist = get_next(plist);

			//if (!psta || (psta == padapter->self_sta))
			if (!psta)
				continue;

			RTW_PRINT_SEL(m, "[%d] STA[%pM]->txq[ac]\n", i, psta->phl_sta->mac_addr);
			/*RTW_PRINT_SEL(m, "	[amsdu] txsc_amsdu_timeout:%u\n",
			              psta->txsc_amsdu_timeout);*/
			/* AMSDU_BY_SIZE */
			RTW_PRINT_SEL(m, "	[amsdu] cap:%d, num:%d, max:%d, size:%d, max_size:%d\n",
			              psta->phl_sta->asoc_cap.max_amsdu_len,
			              psta->txsc_amsdu_num,
			              psta->txsc_amsdu_max,
			              //psta->txsc_amsdu_size,
						  0,
			              //psta->txsc_amsdu_max_size
						  0);
			for (j = 0; j < 4; j++) {
				if (psta->amsdu_txq[j].cnt != 0 ||
					psta->amsdu_txq[j].rptr !=0 ||
					psta->amsdu_txq[j].wptr !=0)// ||
					//psta->amsdu_txq[j].skb_qlen != 0)
					RTW_PRINT_SEL(m, "	amsdu_txq[%d]: cnt:%d, rptr:%d, wptr:%d, qlen:%d\n",
					              j,
					              psta->amsdu_txq[j].cnt,
					              psta->amsdu_txq[j].rptr,
					              psta->amsdu_txq[j].wptr,
					              //psta->amsdu_txq[j].skb_qlen
								  0);
			}
			RTW_PRINT_SEL(m, "\n");
		}
	}
}

s32 txsc_amsdu_timeout_tx(struct sta_info *psta, u8 ac)
{
	_adapter *padapter = psta->padapter;
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct txsc_pkt_entry txsc_pkt;
	u8 i;

	txsc_amsdu_timeout_init_pkt_entry(padapter, &txsc_pkt, psta, ac);

	txsc_pkt.skb_cnt = txsc_amsdu_dequeue(padapter, &txsc_pkt);
	if (txsc_pkt.skb_cnt == 0) {
		pxmitpriv->cnt_txsc_amsdu_timeout_deq_empty++;
		goto abort_core_tx;
	}

	if (txsc_get_sc_entry(padapter, txsc_pkt.xmit_skb[0], &txsc_pkt) != _SUCCESS)
		goto abort_core_tx;

	if (txsc_apply_sc_cached_entry(padapter, &txsc_pkt) == _FAIL)
		goto abort_core_tx;

	if (core_tx_call_phl(padapter, NULL, &txsc_pkt) == FAIL)
		goto abort_core_tx;

	pxmitpriv->cnt_txsc_amsdu_timeout_ok[ac]++;

	return SUCCESS;

abort_core_tx:

	if (txsc_pkt.ptxreq)
		txsc_free_txreq(padapter, txsc_pkt.ptxreq);
	else {
		for (i = 0; i < txsc_pkt.skb_cnt; i++) {
			if (txsc_pkt.xmit_skb[i])
				rtw_os_pkt_complete(padapter, txsc_pkt.xmit_skb[i]);
		}
	}

	pxmitpriv->cnt_txsc_amsdu_timeout_fail[ac]++;

	return FAIL;
}
#endif

#ifdef CONFIG_80211N_HT
static u8 txsc_issue_addbareq_check(_adapter *padapter, u8 issue_when_busy)
{
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct registry_priv *pregistry = &padapter->registrypriv;

	if (pregistry->tx_quick_addba_req == 0) {
		if ((issue_when_busy == _TRUE) && (pmlmepriv->LinkDetectInfo.bBusyTraffic == _FALSE))
			return _FALSE;

		if (pmlmepriv->LinkDetectInfo.NumTxOkInPeriod < 100)
			return _FALSE;
	}

	return _TRUE;
}

void txsc_issue_addbareq_cmd(_adapter *padapter, u8 priority, struct sta_info *psta, u8 issue_when_busy)
{
	u8 issued;
	struct ht_priv	*phtpriv;
	struct ampdu_priv *ampdu_priv;

	if (txsc_issue_addbareq_check(padapter, issue_when_busy) == _FALSE)
		return;

	phtpriv = &psta->htpriv;
	ampdu_priv = &psta->ampdu_priv;

	if ((phtpriv->ht_option == _TRUE) && (ampdu_priv->ampdu_enable == _TRUE)) {
		issued = (ampdu_priv->agg_enable_bitmap >> priority) & 0x1;
		issued |= (ampdu_priv->candidate_tid_bitmap >> priority) & 0x1;

		if (issued == 0) {
			RTW_INFO("rtw_issue_addbareq_cmd, p=%d\n", priority);
			psta->ampdu_priv.candidate_tid_bitmap |= BIT((u8)priority);
			rtw_addbareq_cmd(padapter, (u8) priority, psta->phl_sta->mac_addr);
		}
	}

}
#endif /* CONFIG_80211N_HT */
#endif /* CONFIG_CORE_TXSC */

