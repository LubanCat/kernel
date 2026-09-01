#include "mac_priv.h"
#include "tx_statistic.h"

u32 mac_enable_tx_statistic(struct mac_ax_adapter *adapter, u8 en)
{
#ifdef MAC_TXRPT_STATISTIC
	u32 ret;
	struct fwcmd_txrpt_forward *forward_info;
	struct h2c_info h2c_info = {0};

	if (adapter->sm.fwdl != MAC_AX_FWDL_INIT_RDY) {
		PLTFM_MSG_WARN("%s fw not ready\n", __func__);
		return MACFWNONRDY;
	}

	if (!adapter->txrpt_dbg_stat)
		return MACNOITEM;

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_txrpt_forward);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_MEDIA_RPT;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_TXRPT_FORWARD;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	forward_info = (struct fwcmd_txrpt_forward *)PLTFM_MALLOC(h2c_info.content_len);
	if (!forward_info) {
		PLTFM_MSG_ERR("%s: h2c MALLOC fail\n", __func__);
		return MACNPTR;
	}

	forward_info->dword0 = cpu_to_le32(en ? FWCMD_H2C_TXRPT_FORWARD_EN : 0);

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)forward_info);

	PLTFM_FREE(forward_info, h2c_info.content_len);

	PLTFM_MEMSET(adapter->txrpt_dbg_stat, 0,
		     sizeof(struct mac_txrpt_dbg_stat));

	return ret;
#else
	return MACSUCCESS;
#endif
}

u32 mac_process_txrpt(struct mac_ax_adapter *adapter, u8 *frame, u32 len)
{
#ifdef MAC_TXRPT_STATISTIC
	struct mac_txrpt_by_endian *rpt = (struct mac_txrpt_by_endian *)frame;
	struct mac_txrpt_dbg_info *txrpt_dbg_info = NULL;

	u8 index = 0;

	u32 macid = 0xffff;
	u32 ppdu_type = 0;

	u32 total_packet = 0;
	u32 fail_packet = 0;
	u32 success_packet = 0;

	u32 rate_gen = 0;
	u32 rate_mcs = 0;
	u32 rate_nss = 0;
	u32 bw = 0;

	u16 queue_time;

	u32 acc_tx_time;
	u32 pre_zld;
	u32 mid_zld;
	u32 post_zld;

	u32 diff_cnt;

	u32 data_retry_cnt = 0;
	u32 rts_tx_retry_cnt = 0;

	for (index = 0 ; index < sizeof(struct mac_ccxrpt); index += 4)
		*(u32 *)((u8 *)frame + index) = cpu_to_le32(*(u32 *)((u8 *)frame + index));

	if (rpt->rpt_sel != 0)
		return MACSUCCESS;

	if (!adapter->txrpt_dbg_stat)
		return MACNOITEM;

	macid = rpt->macid;

	if (macid >= MAC_STA_NUM)
		return MACNOITEM;

	txrpt_dbg_info = &adapter->txrpt_dbg_stat->txrpt_dbg_info[macid];
	PLTFM_MUTEX_LOCK(&adapter->txrpt_dbg_stat->dbg_info_lock[macid]);

	ppdu_type = rpt->ppdu_type;

	total_packet   = rpt->total_pkt_num;
	success_packet = rpt->pkt_ok_num;
	fail_packet    = total_packet - success_packet;

	txrpt_dbg_info->ppdu_cnt[ppdu_type]++;

	rate_gen = rpt->final_rate >> 7;
	bw = rpt->data_bw;
	rate_mcs = rpt->final_rate & 0xF;
	rate_nss = rpt->final_rate >> 4 & 0x7;

	if (total_packet == 0)
		txrpt_dbg_info->total_0[ppdu_type]++;

	if (success_packet == 0)
		txrpt_dbg_info->ok_0[ppdu_type]++;

	txrpt_dbg_info->tx_ok    += success_packet;
	txrpt_dbg_info->tx_fail += fail_packet;

	txrpt_dbg_info->ppdu_type_ok_fail[ppdu_type][MPDU_SUCCESS]
	+= success_packet;
	txrpt_dbg_info->ppdu_type_ok_fail[ppdu_type][MPDU_FAIL]
	+= fail_packet;

	if (rate_gen == 0) {
		txrpt_dbg_info->ofdm_CCK_ok_fail[rate_mcs][MPDU_SUCCESS]
		+= success_packet;
		txrpt_dbg_info->ofdm_CCK_ok_fail[rate_mcs][MPDU_FAIL]
		+= fail_packet;
	} else if (rate_gen == 1) {
		txrpt_dbg_info->ht_ok_fail[bw][rate_nss][rate_mcs][MPDU_SUCCESS]
		+= success_packet;
		txrpt_dbg_info->ht_ok_fail[bw][rate_nss][rate_mcs][MPDU_FAIL]
		+= fail_packet;
	} else if (rate_gen == 2) {
		txrpt_dbg_info->vht_ok_fail[ppdu_type][bw]
		[rate_nss][rate_mcs][MPDU_SUCCESS]
		+= success_packet;
		txrpt_dbg_info->vht_ok_fail[ppdu_type][bw]
		[rate_nss][rate_mcs][MPDU_FAIL]
		+= fail_packet;
	} else if (rate_gen == 3) {
		txrpt_dbg_info->he_ok_fail[ppdu_type][bw + 3]
		[rate_nss][rate_mcs][MPDU_SUCCESS]
		+= success_packet;
		txrpt_dbg_info->he_ok_fail[ppdu_type][bw + 3]
		[rate_nss][rate_mcs][MPDU_FAIL]
		+= fail_packet;
	}

	data_retry_cnt = rpt->data_tx_cnt - 1;
	data_retry_cnt = data_retry_cnt > 7 ? 7 : data_retry_cnt;
	txrpt_dbg_info->retry_lvl_cnt[ppdu_type][data_retry_cnt]++;
	txrpt_dbg_info->ppdu_retry_cnt[ppdu_type] += !(!data_retry_cnt);
	txrpt_dbg_info->tx_retry += !(!data_retry_cnt);

	queue_time = (u16)(rpt->queue_time & 0xFFFF);
	txrpt_dbg_info->queue_time_max =
		queue_time > txrpt_dbg_info->queue_time_max ?\
		queue_time : txrpt_dbg_info->queue_time_max;

	txrpt_dbg_info->queue_time_total += queue_time;
	txrpt_dbg_info->queue_time_last
	[txrpt_dbg_info->ppdu_cnt[ppdu_type] % 100] = queue_time;

	acc_tx_time = rpt->acctxtime;
	txrpt_dbg_info->total_tx_time += acc_tx_time;

	txrpt_dbg_info->max_tx_time =
		acc_tx_time > txrpt_dbg_info->max_tx_time ?
		acc_tx_time : txrpt_dbg_info->max_tx_time;

	txrpt_dbg_info->max_tx_time =
		acc_tx_time < txrpt_dbg_info->min_tx_time ?
		acc_tx_time : txrpt_dbg_info->min_tx_time;

	txrpt_dbg_info->pri_user_cnt[ppdu_type] += rpt->bpri;
	txrpt_dbg_info->muru2su_cnt[ppdu_type - 1] += rpt->mu2su;
	txrpt_dbg_info->rts_state_cnt[rpt->rts_tx_state]++;
	txrpt_dbg_info->tx_state_cnt[rpt->tx_state]++;

	txrpt_dbg_info->collision_head_cnt += rpt->collision_head;
	txrpt_dbg_info->collision_head_cnt += rpt->collision_tail;

	rts_tx_retry_cnt = rpt->rts_tx_cnt > 0 ? rpt->rts_tx_cnt - 1 : 0;
	rts_tx_retry_cnt = rts_tx_retry_cnt > 7 ? 7 : rts_tx_retry_cnt;
	txrpt_dbg_info->rts_tx_retry_cnt[rts_tx_retry_cnt]++;

	pre_zld  = rpt->pre_zld_len;
	mid_zld  = rpt->mid_zld_len;
	post_zld = rpt->post_zld_len;

	txrpt_dbg_info->pre_zld_total[ppdu_type] += pre_zld;
	txrpt_dbg_info->pre_zld_max[ppdu_type] =
		pre_zld > txrpt_dbg_info->pre_zld_max[ppdu_type] ?
		pre_zld : txrpt_dbg_info->pre_zld_max[ppdu_type];

	txrpt_dbg_info->mid_zld_total[ppdu_type] += mid_zld;
	txrpt_dbg_info->mid_zld_max[ppdu_type] =
		mid_zld > txrpt_dbg_info->mid_zld_max[ppdu_type] ?
		mid_zld : txrpt_dbg_info->mid_zld_max[ppdu_type];

	txrpt_dbg_info->post_zld_total[ppdu_type] += post_zld;
	txrpt_dbg_info->post_zld_max[ppdu_type] =
		post_zld > txrpt_dbg_info->post_zld_max[ppdu_type] ?
		post_zld : txrpt_dbg_info->post_zld_max[ppdu_type];

	diff_cnt = rpt->diff_pkt_num;
	diff_cnt = diff_cnt > 4 ? 4 : diff_cnt;
	txrpt_dbg_info->diff_lvl_cnt[ppdu_type][diff_cnt]++;

	PLTFM_MUTEX_UNLOCK(&adapter->txrpt_dbg_stat->dbg_info_lock[macid]);
#endif
	return MACSUCCESS;
}

u32 mac_get_sta_tx_dbg_info(struct mac_ax_adapter *adapter, u16 macid,
			    struct mac_tx_debug_info *tx_dbg_info)
{
#ifdef MAC_TXRPT_STATISTIC
	struct mac_txrpt_dbg_info *txrpt_dbg_info = NULL;

	if (macid >= MAC_STA_NUM)
		return MACNOITEM;

	if (!adapter->txrpt_dbg_stat)
		return MACNOITEM;

	txrpt_dbg_info = &adapter->txrpt_dbg_stat->txrpt_dbg_info[macid];

	tx_dbg_info->he_ok_fail	       = &txrpt_dbg_info->he_ok_fail;
	tx_dbg_info->vht_ok_fail       = &txrpt_dbg_info->vht_ok_fail;
	tx_dbg_info->ht_ok_fail        = &txrpt_dbg_info->ht_ok_fail;
	tx_dbg_info->ofdm_CCK_ok_fail  = &txrpt_dbg_info->ofdm_CCK_ok_fail;
	tx_dbg_info->ppdu_type_ok_fail = &txrpt_dbg_info->ppdu_type_ok_fail;

	tx_dbg_info->ppdu_retry_cnt   = &txrpt_dbg_info->ppdu_retry_cnt;

	tx_dbg_info->tx_ok   = txrpt_dbg_info->tx_ok;
	tx_dbg_info->tx_fail = txrpt_dbg_info->tx_fail;
	tx_dbg_info->tx_retry = txrpt_dbg_info->tx_retry;
#endif
	return MACSUCCESS;
}

u32 mac_clr_sta_tx_dbg_info(struct mac_ax_adapter *adapter, u16 macid)
{
#ifdef MAC_TXRPT_STATISTIC
	struct mac_txrpt_dbg_info *txrpt_dbg_info = NULL;

	if (macid >= MAC_STA_NUM)
		return MACNOITEM;

	if (!adapter->txrpt_dbg_stat)
		return MACNOITEM;

	txrpt_dbg_info = &adapter->txrpt_dbg_stat->txrpt_dbg_info[macid];
	PLTFM_MUTEX_LOCK(&adapter->txrpt_dbg_stat->dbg_info_lock[macid]);
	PLTFM_MEMSET(txrpt_dbg_info, 0, sizeof(struct mac_txrpt_dbg_info));
	PLTFM_MUTEX_UNLOCK(&adapter->txrpt_dbg_stat->dbg_info_lock[macid]);

#endif
	return MACSUCCESS;
}