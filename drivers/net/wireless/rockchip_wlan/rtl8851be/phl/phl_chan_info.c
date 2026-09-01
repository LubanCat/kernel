/******************************************************************************
 *
 * Copyright(c) 2019 Realtek Corporation.
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
#define _PHL_CHAN_INFO_C_
#include "phl_headers.h"

#ifdef CONFIG_PHL_CHANNEL_INFO

#ifdef CONFIG_PHL_CSI_FW_TX_OFLD
static u8 _chinfo_get_pkt_ofld_ids(struct rtw_chinfo_cur_parm *cur_parm, u8 *pkt_id)
{
	u8 i;
	u8 cnt = 0;

	for (i = 0; i < MAX_CHAN_INFO_CLIENT; i++) {
		if (cur_parm->pkt_id[i] != NOT_USED) {
			if (pkt_id)
				pkt_id[cnt] = cur_parm->pkt_id[i];
			cnt++;
		}
	}

	return cnt;
}

static enum rtw_phl_status
_chinfo_chk_pkt_offload(struct phl_info_t *phl_info,
				struct rtw_phl_stainfo_t *peer_sta, bool enable)
{
	struct rtw_chinfo_cur_parm *cur_parm = phl_info->phl_com->cur_parm;
	enum rtw_phl_status sts = RTW_PHL_STATUS_SUCCESS;
	struct rtw_pkt_ofld_null_info null_info = {0};
	struct rtw_phl_stainfo_t *self_sta = rtw_phl_get_stainfo_self(phl_info, peer_sta->rlink);
	u16 macid = peer_sta->macid;
	u8 ori_pkt_num = _chinfo_get_pkt_ofld_ids(cur_parm, NULL);
	bool pkt_ids_changed = false;
	u8 pkt_id;
	u32 token;

	if (cur_parm->action_parm.mode != CHINFO_MODE_ACK || !enable) {
		if (cur_parm->pkt_id[macid] != NOT_USED) {
			rtw_phl_pkt_ofld_cancel(phl_info, macid,
				PKT_TYPE_NULL_DATA,
				&cur_parm->pkt_token[macid]);
			cur_parm->pkt_id[macid] = NOT_USED;
			pkt_ids_changed = true;
		}
		goto chk_ids;
	}

	pkt_id = phl_pkt_ofld_get_id(phl_info, macid, PKT_TYPE_NULL_DATA);
	if (pkt_id == NOT_USED) {
		void *d = phl_to_drvpriv(phl_info);
		u8 *a3;

		if (rtw_phl_is_ap_category(peer_sta->wrole->type))
			a3 = peer_sta->rlink->mac_addr;
		else
			a3 = self_sta->mac_addr; /* this is AP's addr */

		_os_mem_cpy(d, null_info.a1, peer_sta->mac_addr, MAC_ADDRESS_LENGTH);
		_os_mem_cpy(d, null_info.a2, peer_sta->rlink->mac_addr, MAC_ADDRESS_LENGTH);
		_os_mem_cpy(d, null_info.a3, a3, MAC_ADDRESS_LENGTH);
	}

	sts = rtw_phl_pkt_ofld_request(phl_info, macid,
		PKT_TYPE_NULL_DATA, &token, &null_info, __func__);
	if (sts != RTW_PHL_STATUS_SUCCESS)
		goto exit;

	if (pkt_id == NOT_USED)
		pkt_id = phl_pkt_ofld_get_id(phl_info, macid, PKT_TYPE_NULL_DATA);

	if (cur_parm->pkt_id[peer_sta->macid] != pkt_id) {
		cur_parm->pkt_id[peer_sta->macid] = pkt_id;
		pkt_ids_changed = true;
	}
	cur_parm->pkt_token[peer_sta->macid] = token;

chk_ids:
	if (pkt_ids_changed) {
		enum rtw_hal_status hsts;
		u8 retry_cnt = 5;
		u16 period = cur_parm->action_parm.trig_period;
		u8 pkt_num;
		u8 pkt_ids[MAX_CHAN_INFO_CLIENT];
		enum rtw_data_rate rate = RTW_DATA_RATE_OFDM6;

		pkt_num = _chinfo_get_pkt_ofld_ids(cur_parm, pkt_ids);

		if (ori_pkt_num)
			hsts = rtw_hal_ch_info_pkt_ofld(phl_info->hal, self_sta->macid,
						false, period, retry_cnt, rate,
						pkt_num, pkt_ids);

		hsts = rtw_hal_ch_info_pkt_ofld(phl_info->hal, self_sta->macid,
					pkt_num ? true : false, period, retry_cnt, rate,
					pkt_num, pkt_ids);
		if (hsts != RTW_HAL_STATUS_SUCCESS)
			sts = RTW_PHL_STATUS_FAILURE;
	}

exit:
	return sts;
}
#endif /* CONFIG_PHL_CSI_FW_TX_OFLD */

static enum rtw_phl_status
_chk_cfg_chinfo(struct phl_info_t *phl_info,
			struct rtw_chinfo_action_parm *act_param)
{
	enum rtw_phl_status sts = RTW_PHL_STATUS_SUCCESS;
	struct rtw_chinfo_cur_parm *cur_parm = NULL;
	enum phl_chinfo_mode mode;
	struct rtw_phl_com_t *phl_com = phl_info->phl_com;
	struct rtw_phl_stainfo_t *peer_sta;
	struct rtw_phl_stainfo_t *self_sta;
	u16 macid;
	u8 arr_idx = 0;

	cur_parm = phl_com->cur_parm;
	if (cur_parm == NULL) {
		PHL_ERR("[CHAN INFO]%s: cur_parm is null\n", __func__);
		sts = RTW_PHL_STATUS_FAILURE;
		goto exit;
	}

	peer_sta = act_param->sta;
	if (peer_sta == NULL) {
		PHL_ERR("[CHAN INFO]%s: peer_sta is null\n", __func__);
		sts = RTW_PHL_STATUS_FAILURE;
		goto exit;
	}

	self_sta = rtw_phl_get_stainfo_self(phl_info, peer_sta->rlink);
	if (self_sta == NULL) {
		PHL_ERR("[CHAN INFO]%s: self_sta is null\n", __func__);
		sts = RTW_PHL_STATUS_FAILURE;
		goto exit;
	}

	mode = cur_parm->num == 0 ? act_param->mode : cur_parm->action_parm.mode;
	macid = mode == CHINFO_MODE_ACK ? self_sta->macid : peer_sta->macid;

	/* check macid for cur_parm  */
	arr_idx = (u8)_os_division64(macid, 8);


	if (act_param->enable) {
		if (cur_parm->macid_bitmap[arr_idx] & BIT(_os_modular64(macid, 8))) {
			PHL_WARN("[CH INFO] macid:%d already enable(0x%02x)\n",
				macid, cur_parm->macid_bitmap[arr_idx]);

			sts = RTW_PHL_STATUS_FAILURE;
			goto exit;
		}
	} else {
		if (!(cur_parm->macid_bitmap[arr_idx] & BIT(_os_modular64(macid, 8)))) {
			PHL_WARN("[CH INFO] macid:%d already disable(0x%02x)\n",
				macid, cur_parm->macid_bitmap[arr_idx]);

			sts = RTW_PHL_STATUS_FAILURE;
			goto exit;
		}
	}
exit:
	return sts;
}

enum rtw_phl_status
_phl_cfg_chinfo(struct phl_info_t *phl_info,
			    struct rtw_chinfo_action_parm *act_param)
{
	enum rtw_phl_status sts = RTW_PHL_STATUS_SUCCESS;
	enum rtw_hal_status hsts = RTW_HAL_STATUS_SUCCESS;
	struct rtw_phl_com_t *phl_com = phl_info->phl_com;
	struct rtw_chinfo_cur_parm *cur_param = phl_com->cur_parm;
	struct rtw_wifi_role_link_t *rlink = act_param->sta->rlink;

	sts = _chk_cfg_chinfo(phl_info, act_param);
	if (sts != RTW_PHL_STATUS_SUCCESS) {
		PHL_WARN("[CH INFO] %s _chk_cfg_chinfo fail(%d)\n", __func__, sts);
		goto exit;
	}

	switch (act_param->act) {
	case CHINFO_ACT_EN:
		hsts = rtw_hal_ch_info_en(phl_info->hal, act_param);
		if (hsts != RTW_HAL_STATUS_SUCCESS) {
			sts = RTW_PHL_STATUS_FAILURE;
			goto exit;
		}

		/* for light mode(from ppdu sts) */
		if (cur_param->action_parm.enable_mode == CHINFO_EN_LIGHT_MODE
			&& cur_param->action_parm.mode == CHINFO_MODE_ACK
		) {
			if (act_param->enable == true && cur_param->num == 1)
				rtw_phl_mr_set_rxfltr_type_by_mode(phl_info, rlink,
					RX_FLTR_TYPE_MODE_CHAN_INFO_EN);
			else if (act_param->enable == false && cur_param->num == 0)
				rtw_phl_mr_set_rxfltr_type_by_mode(phl_info, rlink,
					RX_FLTR_TYPE_MODE_CHAN_INFO_DIS);
		}
		#ifdef CONFIG_PHL_PKTOFLD
		sts = _chinfo_chk_pkt_offload(phl_info, act_param->sta, act_param->enable);
		#endif
		break;
	case CHINFO_ACT_CFG:
		sts = rtw_hal_cfg_chinfo(phl_info->hal, act_param);
		if (sts != RTW_PHL_STATUS_SUCCESS)
			PHL_ERR("%s: cfg_chinfo fail\n", __func__);
		break;
	default:
		PHL_ERR("%s: unknow act(%d)\n", __func__, act_param->act);
		sts = RTW_PHL_STATUS_INVALID_PARAM;
		break;
	}
exit:
	return sts;
}

#ifdef CONFIG_CMD_DISP
enum rtw_phl_status
phl_cmd_cfg_chinfo_hdl(struct phl_info_t *phl_info, u8 *param)
{
	struct rtw_chinfo_action_parm *act_parm = (struct rtw_chinfo_action_parm *)param;

	return _phl_cfg_chinfo(phl_info, act_parm);
}

static void _phl_cfg_chinfo_done(void *drv_priv, u8 *cmd, u32 cmd_len, enum rtw_phl_status status)
{
	if (cmd) {
		_os_kmem_free(drv_priv, cmd, cmd_len);
		cmd = NULL;
		PHL_INFO("%s.....\n", __func__);
	}
}

enum rtw_phl_status
_phl_cmd_cfg_chinfo(struct phl_info_t *phl_info,
			    struct rtw_chinfo_action_parm *act_parm,
			    enum phl_cmd_type cmd_type,
			    u32 cmd_timeout)
{
	enum rtw_phl_status sts = RTW_PHL_STATUS_FAILURE;
	void *d = phl_to_drvpriv(phl_info);
	struct rtw_chinfo_action_parm *param = NULL;
	u32 param_len;

	if (cmd_type == PHL_CMD_DIRECTLY)
		return _phl_cfg_chinfo(phl_info, act_parm);

	param_len = sizeof(struct rtw_chinfo_action_parm);
	param = _os_kmem_alloc(phl_to_drvpriv(phl_info), param_len);
	if (param == NULL) {
		PHL_ERR("%s: alloc param failed!\n", __func__);
		goto _exit;
	}

	_os_mem_cpy(d, param, act_parm, sizeof(struct rtw_chinfo_action_parm));

	sts = phl_cmd_enqueue(phl_info,
	                      act_parm->sta->rlink->hw_band,
	                      MSG_EVT_CFG_CHINFO,
	                      (u8 *)param,
	                      param_len,
	                      _phl_cfg_chinfo_done,
	                      cmd_type,
	                      cmd_timeout);
	if (is_cmd_failure(sts)) {
		/* Send cmd success, but wait cmd fail*/
		sts = RTW_PHL_STATUS_FAILURE;
	} else if (sts != RTW_PHL_STATUS_SUCCESS) {
		/* Send cmd fail */
		_os_kmem_free(phl_to_drvpriv(phl_info), param, param_len);
		sts = RTW_PHL_STATUS_FAILURE;
	}

_exit:
	return sts;
}

#endif

enum rtw_phl_status rtw_phl_cmd_cfg_chinfo(void *phl,
					   struct rtw_chinfo_action_parm *act_parm,
					   enum phl_cmd_type cmd_type,
					   u32 cmd_timeout)
{
	struct phl_info_t *phl_info = (struct phl_info_t *)phl;

#ifdef CONFIG_CMD_DISP
	return _phl_cmd_cfg_chinfo(phl_info, act_parm, cmd_type, cmd_timeout);
#else
	return _phl_cfg_chinfo(phl_info, act_parm);
#endif
}

#ifndef CONFIG_PHL_CHANNEL_INFO_DIRECT_INDICATE
enum rtw_phl_status rtw_phl_query_chan_info(void *phl, u32 buf_len,
	u8* chan_info_buffer, u32 *length, struct csi_header_t *csi_header)
{
	struct phl_info_t *phl_info = (struct phl_info_t *)phl;
	void *drv_priv = phl_to_drvpriv(phl_info);
	struct rtw_phl_com_t *phl_com = phl_info->phl_com;
	struct chan_info_t *chan_info_pkt_latest = NULL;
	enum rtw_phl_status status = RTW_PHL_STATUS_FAILURE;

	if(chan_info_buffer == NULL){
		PHL_ERR("buffer is not ready.\n");
		return status;
	}

	/* Get the latest channel info from busy queue. */
	chan_info_pkt_latest = rtw_phl_query_busy_chaninfo(drv_priv, phl_com);
	if (chan_info_pkt_latest != NULL) {
		if (buf_len < chan_info_pkt_latest->length) {
			PHL_ERR("%s: Buffer length not sufficient! \n", __func__);
			rtw_phl_enqueue_idle_chaninfo(drv_priv, phl_com, chan_info_pkt_latest);
			return status;
		}
		/* copy raw data resources. */
		_os_mem_cpy(drv_priv, chan_info_buffer,
			chan_info_pkt_latest->chan_info_buffer, chan_info_pkt_latest->length);
		_os_mem_cpy(drv_priv, csi_header,
			&chan_info_pkt_latest->csi_header, sizeof(struct csi_header_t));
		*length = chan_info_pkt_latest->length;

		rtw_phl_enqueue_idle_chaninfo(drv_priv, phl_com, chan_info_pkt_latest);
		status = RTW_PHL_STATUS_SUCCESS;
	} else {
		PHL_TRACE(COMP_PHL_CHINFO, _PHL_DEBUG_, "%s: There is no channel info packet.\n", __func__);
	}
	return status;
}

static enum rtw_phl_status _phl_chaninfo_init_rx_pool(struct phl_info_t *phl_info)
{
	struct rx_chan_info_pool *chan_info_pool;
	struct chan_info_t *chan_info_pkt;
	u32 buf_len, i;

	buf_len = sizeof(*chan_info_pool);
	chan_info_pool = _os_mem_alloc(phl_to_drvpriv(phl_info), buf_len);

	if (NULL != chan_info_pool) {
		_os_mem_set(phl_to_drvpriv(phl_info), chan_info_pool, 0, buf_len);
		INIT_LIST_HEAD(&chan_info_pool->idle);
		INIT_LIST_HEAD(&chan_info_pool->busy);
		_os_spinlock_init(phl_to_drvpriv(phl_info),
					&chan_info_pool->idle_lock);
		_os_spinlock_init(phl_to_drvpriv(phl_info),
					&chan_info_pool->busy_lock);
		chan_info_pool->idle_cnt = 0;

		for (i = 0; i < CHAN_INFO_PKT_TOTAL; i++) {
			chan_info_pkt = &chan_info_pool->channl_info_pkt[i];
			chan_info_pkt->chan_info_buffer = _os_mem_alloc(phl_to_drvpriv(phl_info),
				CHAN_INFO_MAX_SIZE);
			if (NULL != chan_info_pkt->chan_info_buffer) {
				chan_info_pkt->length = 0;
				INIT_LIST_HEAD(&chan_info_pkt->list);
				list_add_tail(&chan_info_pkt->list, &chan_info_pool->idle);
				chan_info_pool->idle_cnt++;
			} else {
				return RTW_PHL_STATUS_RESOURCE;
			}
		}
		phl_info->phl_com->chan_info_pool = chan_info_pool;
	} else {
		return RTW_PHL_STATUS_RESOURCE;
	}

	return RTW_PHL_STATUS_SUCCESS;
}

static void _phl_chaninfo_deinit_rx_pool(struct phl_info_t *phl_info)
{
	struct rx_chan_info_pool *chan_info_pool;
	u8* chan_info_buffer;
	u32 buf_len, i;

	chan_info_pool = phl_info->phl_com->chan_info_pool;
	if (NULL != chan_info_pool) {
		_os_spinlock_free(phl_to_drvpriv(phl_info),
					&chan_info_pool->idle_lock);
		_os_spinlock_free(phl_to_drvpriv(phl_info),
					&chan_info_pool->busy_lock);
		for (i = 0; i < CHAN_INFO_PKT_TOTAL; i++) {
			chan_info_buffer = chan_info_pool->channl_info_pkt[i].chan_info_buffer;
			if (chan_info_buffer != NULL) {
				_os_mem_free(phl_to_drvpriv(phl_info), chan_info_buffer,
					CHAN_INFO_MAX_SIZE);
				chan_info_pool->channl_info_pkt[i].chan_info_buffer = NULL;
			}
		}
		buf_len = sizeof(*chan_info_pool);
		_os_mem_free(phl_to_drvpriv(phl_info), chan_info_pool, buf_len);
		phl_info->phl_com->chan_info_pool = NULL;
	}
}
#else
static enum rtw_phl_status _phl_chaninfo_init_rx_pkt(struct phl_info_t *phl_info)
{
	struct chan_info_t *chan_info_pkt;
	u32 buf_len;

	/* single chan_info_pkt mainly for rich mode to wait last seg */
	buf_len = sizeof(*chan_info_pkt) + CHAN_INFO_MAX_SIZE;
	chan_info_pkt = _os_mem_alloc(phl_to_drvpriv(phl_info), buf_len);
	if (chan_info_pkt == NULL)
		return RTW_PHL_STATUS_RESOURCE;

	_os_mem_set(phl_to_drvpriv(phl_info), chan_info_pkt, 0, buf_len);
	chan_info_pkt->chan_info_buffer = ((u8 *)chan_info_pkt) + sizeof(*chan_info_pkt);
	phl_info->phl_com->chan_info = chan_info_pkt;

	return RTW_PHL_STATUS_SUCCESS;
}

static void _phl_chaninfo_deinit_rx_pkt(struct phl_info_t *phl_info)
{
	if (phl_info->phl_com->chan_info) {
		_os_mem_free(phl_to_drvpriv(phl_info), phl_info->phl_com->chan_info
			, sizeof(*phl_info->phl_com->chan_info) + CHAN_INFO_MAX_SIZE);
		phl_info->phl_com->chan_info = NULL;
	}
}
#endif

/*channel info packet pool init/deinit*/
static void _phl_chaninfo_deinit (struct phl_info_t *phl_info)
{
	FUNCIN();

	#ifndef CONFIG_PHL_CHANNEL_INFO_DIRECT_INDICATE
	_phl_chaninfo_deinit_rx_pool(phl_info);
	#else
	_phl_chaninfo_deinit_rx_pkt(phl_info);
	#endif

	if (NULL != phl_info->phl_com->cur_parm) {
		_os_mem_free(phl_to_drvpriv(phl_info), phl_info->phl_com->cur_parm,
					sizeof(struct rtw_chinfo_cur_parm));
		phl_info->phl_com->cur_parm = NULL;
	}

	FUNCOUT();
}

static enum rtw_phl_status _phl_chaninfo_init(struct phl_info_t *phl_info)
{
	enum rtw_phl_status pstatus = RTW_PHL_STATUS_SUCCESS;
	u32 i = 0;

	FUNCIN_WSTS(pstatus);

	#ifndef CONFIG_PHL_CHANNEL_INFO_DIRECT_INDICATE
	pstatus = _phl_chaninfo_init_rx_pool(phl_info);
	#else
	pstatus = _phl_chaninfo_init_rx_pkt(phl_info);
	#endif
	if (pstatus != RTW_PHL_STATUS_SUCCESS)
		goto exit;

	phl_info->phl_com->cur_parm = _os_mem_alloc(phl_to_drvpriv(phl_info),
				sizeof(struct rtw_chinfo_cur_parm));
	if (NULL == phl_info->phl_com->cur_parm) {
		pstatus = RTW_PHL_STATUS_RESOURCE;
		goto exit;
	}

	phl_info->phl_com->cur_parm->rate = RTW_DATA_RATE_OFDM6;

#ifdef CONFIG_PHL_CSI_FW_TX_OFLD
	for (i = 0; i < MAX_CHAN_INFO_CLIENT; i++)
		phl_info->phl_com->cur_parm->pkt_id[i] = NOT_USED;
#endif

exit:
	if (RTW_PHL_STATUS_SUCCESS != pstatus)
		_phl_chaninfo_deinit(phl_info);
	FUNCOUT_WSTS(pstatus);

	return pstatus;
}

enum rtw_phl_status phl_chaninfo_init(struct phl_info_t *phl_info)
{
	enum rtw_phl_status phl_status = RTW_PHL_STATUS_FAILURE;

	phl_status = _phl_chaninfo_init(phl_info);
	if (phl_status != RTW_PHL_STATUS_SUCCESS)
		PHL_ERR("channel info pool allocate fail\n");

	return phl_status;
}

void phl_chaninfo_deinit(struct phl_info_t *phl_info)
{
	_phl_chaninfo_deinit(phl_info);
}

void phl_chaninfo_pkt_init(void* drv_priv, struct chan_info_t *chan_info_pkt)
{
	_os_mem_set(drv_priv, &chan_info_pkt->csi_header, 0,
		sizeof( chan_info_pkt->csi_header));
	_os_mem_set(drv_priv, chan_info_pkt->chan_info_buffer, 0,
		CHAN_INFO_MAX_SIZE);
	chan_info_pkt->length = 0;
}

#ifndef CONFIG_PHL_CHANNEL_INFO_DIRECT_INDICATE
u32 rtw_phl_get_chaninfo_idle_number(void* drv_priv, struct rtw_phl_com_t *phl_com)
{
	u32 number;
	struct rx_chan_info_pool *chan_info_pool = NULL;

	chan_info_pool = phl_com->chan_info_pool;
	number = chan_info_pool->idle_cnt;
	return number;
}

u32 rtw_phl_get_chaninfo_busy_number(void* drv_priv, struct rtw_phl_com_t *phl_com)
{
	u32 number;
	struct rx_chan_info_pool *chan_info_pool = NULL;

	chan_info_pool = phl_com->chan_info_pool;
	number = chan_info_pool->busy_cnt;
	return number;
}

struct chan_info_t *rtw_phl_query_idle_chaninfo(void* drv_priv, struct rtw_phl_com_t *phl_com)
{
	struct rx_chan_info_pool *chan_info_pool = NULL;
	struct chan_info_t *chan_info_pkt = NULL;

	chan_info_pool = phl_com->chan_info_pool;

	_os_spinlock(drv_priv, &chan_info_pool->idle_lock, _bh, NULL);
	if (false == list_empty(&chan_info_pool->idle)) {
		chan_info_pkt = list_first_entry(&chan_info_pool->idle,
			struct chan_info_t, list);
		list_del(&chan_info_pkt->list);
		chan_info_pool->idle_cnt--;
	}
	_os_spinunlock(drv_priv, &chan_info_pool->idle_lock, _bh, NULL);

	return chan_info_pkt;
}

struct chan_info_t *rtw_phl_query_busy_chaninfo(void* drv_priv, struct rtw_phl_com_t *phl_com)
{
	struct rx_chan_info_pool *chan_info_pool = NULL;
	struct chan_info_t *chan_info_pkt = NULL;

	chan_info_pool = phl_com->chan_info_pool;

	_os_spinlock(drv_priv, &chan_info_pool->busy_lock, _bh, NULL);
	if (false == list_empty(&chan_info_pool->busy)) {
		chan_info_pkt = list_first_entry(&chan_info_pool->busy,
			struct chan_info_t, list);
		list_del(&chan_info_pkt->list);
		chan_info_pool->busy_cnt--;
	}
	_os_spinunlock(drv_priv, &chan_info_pool->busy_lock, _bh, NULL);

	return chan_info_pkt;
}

struct chan_info_t *rtw_phl_query_busy_chaninfo_latest(void* drv_priv, struct rtw_phl_com_t *phl_com)
{
	struct rx_chan_info_pool *chan_info_pool = NULL;
	struct chan_info_t *chan_info_pkt = NULL;

	chan_info_pool = phl_com->chan_info_pool;

	_os_spinlock(drv_priv, &chan_info_pool->busy_lock, _bh, NULL);
	if (false == list_empty(&chan_info_pool->busy)) {
		chan_info_pkt = list_last_entry(&chan_info_pool->busy,
			struct chan_info_t, list);
		list_del(&chan_info_pkt->list);
		chan_info_pool->busy_cnt--;
	}
	_os_spinunlock(drv_priv, &chan_info_pool->busy_lock, _bh, NULL);

	return chan_info_pkt;
}


void rtw_phl_enqueue_idle_chaninfo(void* drv_priv, struct rtw_phl_com_t *phl_com,
				struct chan_info_t *chan_info_pkt)
{
	struct rx_chan_info_pool *chan_info_pool = NULL;

	chan_info_pool = phl_com->chan_info_pool;

	_os_spinlock(drv_priv, &chan_info_pool->idle_lock, _bh, NULL);
	phl_chaninfo_pkt_init(drv_priv, chan_info_pkt);
	INIT_LIST_HEAD(&chan_info_pkt->list);
	list_add_tail(&chan_info_pkt->list, &chan_info_pool->idle);
	chan_info_pool->idle_cnt++;
	_os_spinunlock(drv_priv, &chan_info_pool->idle_lock, _bh, NULL);
}

struct chan_info_t * rtw_phl_recycle_busy_chaninfo(void* drv_priv,
	struct rtw_phl_com_t *phl_com, struct chan_info_t *chan_info_pkt)
{
	struct rx_chan_info_pool *chan_info_pool = NULL;
	struct chan_info_t *chan_info_pkt_recycle = NULL;

	chan_info_pool = phl_com->chan_info_pool;

	_os_spinlock(drv_priv, &chan_info_pool->busy_lock, _bh, NULL);
	/* enqueue the latest first. */
	INIT_LIST_HEAD(&chan_info_pkt->list);
	list_add_tail(&chan_info_pkt->list, &chan_info_pool->busy);
	chan_info_pool->busy_cnt++;

	/* if the number is greater than max, dequeue the oldest one.*/
	if (chan_info_pool->busy_cnt > MAX_CHAN_INFO_PKT_KEEP) {
		chan_info_pkt_recycle = list_first_entry(&chan_info_pool->busy,
			struct chan_info_t, list);
		list_del(&chan_info_pkt_recycle->list);
		chan_info_pool->busy_cnt--;
	}
	_os_spinunlock(drv_priv, &chan_info_pool->busy_lock, _bh, NULL);

	return chan_info_pkt_recycle;
}
#endif /*# ifndef CONFIG_PHL_CHANNEL_INFO_DIRECT_INDICATE */

#endif /* CONFIG_PHL_CHANNEL_INFO */
