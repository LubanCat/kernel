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
#define _PHL_NAN_DE_C
#include "phl_headers.h"

#ifdef CONFIG_PHL_NAN

/* (ac) indicate_issue_solicited_publish_sdf */
void _phl_nan_indicate_issue_solicited_publish_sdf(
	struct phl_nan_de_priv *de_info,
	struct phl_nan_peer_info_entry *peer_entry,
	struct phl_nan_peer_srvc_info_entry *peer_srvc_entry)
{
	struct phl_nan_info *nan_info = phl_nan_get_info(NAN_EN_DISC, de_info);
	struct phl_info_t *phl_info = nan_info->phl_info;
	void *d = phl_to_drvpriv(phl_info);
	struct phl_nan_de_send_sdf_cmd cmd;

	_os_mem_cpy(d, cmd.peer_info_mac, peer_entry->mac, MAC_ALEN);
	peer_srvc_entry->ref_cnt++;
	cmd.p_peer_srvc_info_entry = (void *)peer_srvc_entry;
	cmd.serv_inst_ck = peer_srvc_entry->assoc_srvc_inst_ck;

	if (RTW_PHL_STATUS_SUCCESS !=  phl_nan_enqueue_cmd(nan_info->phl_info,
							   NAN_EVENT_SEND_SDF,
							   (void *)(&cmd),
							   sizeof(struct phl_nan_de_send_sdf_cmd),
							   PHL_CMD_NO_WAIT, 0))
		peer_srvc_entry->ref_cnt--;
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s: peer[%02x:%02x:%02x:%02x:%02x:%02x]\n", __func__,
		  peer_entry->mac[0], peer_entry->mac[1], peer_entry->mac[2],
		  peer_entry->mac[3], peer_entry->mac[4], peer_entry->mac[5]);
}

/* (ac) indicate_issue_follow_up_sdf */
void _phl_nan_indicate_issue_follow_up_sdf(
	void *d, struct phl_nan_de_priv *de_info,
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry)
{
	struct phl_nan_info *nan_info = phl_nan_get_info(NAN_EN_DISC, de_info);
	struct phl_nan_de_send_sdf_cmd cmd = {0};
	struct phl_nan_peer_info_entry *peer_entry = NULL;
	struct phl_nan_peer_srvc_info_entry *peer_srvc_entry = NULL;
	struct rtw_phl_nan_followup_info *followup =
		phl_nan_get_followup_from_entry(p_srvc_entry);

	if (phl_nan_is_peer_info_exist(nan_info, followup->followup_dest,
				       &peer_entry) != true) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: peer[%02x:%02x:%02x:%02x:%02x:%02x] not found\n",
			  __func__,
			  followup->followup_dest[0],
			  followup->followup_dest[1],
			  followup->followup_dest[2],
			  followup->followup_dest[3],
			  followup->followup_dest[4],
			  followup->followup_dest[5]);
		return;
	}

	if (phl_nan_peer_srvc_info_exist(d,
					 (void *)peer_entry,
					 (void *)&followup->followup_reqid,
					 GET_PEER_SRVC_INFO_BY_INST_ID,
					 &peer_srvc_entry) != true) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: peer[%02x:%02x:%02x:%02x:%02x:%02x] srvc_entry not found\n",
			  __func__,
			  followup->followup_dest[0],
			  followup->followup_dest[1],
			  followup->followup_dest[2],
			  followup->followup_dest[3],
			  followup->followup_dest[4],
			  followup->followup_dest[5]);
		goto unlock_exit;
	}

	_os_mem_cpy(d, cmd.peer_info_mac, followup->followup_dest, MAC_ALEN);
	peer_srvc_entry->ref_cnt++;
	cmd.p_peer_srvc_info_entry = (void *)peer_srvc_entry;
	if (phl_nan_get_followup_autoreply_type_from_entry(p_srvc_entry))
		cmd.auto_followup_serv_inst = true;
	cmd.serv_inst_ck = phl_nan_get_srv_ck_from_entry(p_srvc_entry);

	if (phl_nan_enqueue_cmd(nan_info->phl_info,
				NAN_EVENT_SEND_SDF,
				(void *)(&cmd),
				sizeof(struct phl_nan_de_send_sdf_cmd),
				PHL_CMD_NO_WAIT, 0)
				== RTW_PHL_STATUS_FAILURE) {
		peer_srvc_entry->ref_cnt--;
		phl_nan_auto_followup_free_mem(de_info, p_srvc_entry, true);
	} else {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: peer[%02x:%02x:%02x:%02x:%02x:%02x]\n",
			  __func__,
			  followup->followup_dest[0],
			  followup->followup_dest[1],
			  followup->followup_dest[2],
			  followup->followup_dest[3],
			  followup->followup_dest[4],
			  followup->followup_dest[5]);
	}

unlock_exit:
	phl_nan_done_peer_info_entry(d, nan_info);
}

/* (ac) indicate_report_match_event */
void
_phl_nan_indicate_report_match_event(struct phl_nan_de_priv *de_info,
				     struct phl_nan_peer_info_entry *peer_info,
				     struct phl_nan_peer_srvc_info_entry *peer_srvc)
{
	struct phl_nan_info *nan_info = phl_nan_get_info(NAN_EN_DISC, de_info);
	struct phl_info_t *phl_info = nan_info->phl_info;
	void *d = phl_to_drvpriv(phl_info);
	struct phl_nan_de_send_sdf_cmd cmd = {0};

	_os_mem_cpy(d, cmd.peer_info_mac, peer_info->mac, MAC_ALEN);
	peer_srvc->ref_cnt++;
	cmd.p_peer_srvc_info_entry = (void *)peer_srvc;
	cmd.serv_inst_ck = peer_srvc->assoc_srvc_inst_ck;
	if (RTW_PHL_STATUS_SUCCESS != phl_nan_enqueue_cmd(nan_info->phl_info,
							  NAN_EVENT_RPRT_MATCH_EVT,
							  (void *)(&cmd),
							  sizeof(struct phl_nan_de_send_sdf_cmd),
							  PHL_CMD_NO_WAIT, 0))
		peer_srvc->ref_cnt--;
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s: peer[%02x:%02x:%02x:%02x:%02x:%02x]\n",
		  __func__,
		  peer_info->mac[0], peer_info->mac[1],
		  peer_info->mac[2], peer_info->mac[3],
		  peer_info->mac[4], peer_info->mac[5]);
}

/**
 * phl_nan_init_de_priv - Initialize NAN discovery engine private structure
 * @de_info: NAN discovery engine private structure
 *
 */
void phl_nan_init_de_priv(struct phl_info_t *phl_info,
			  struct phl_nan_de_priv *de_info)
{
	void *d = phl_to_drvpriv(phl_info);

	pq_init(d, &(de_info->publish_srvc_queue));
	pq_init(d, &(de_info->subscribe_srvc_queue));
	pq_init(d, &(de_info->followup_srvc_queue));
	pq_init(d, &(de_info->auto_folloup_srvc_queue));
	de_info->instant_id = FIRST_SRVC_INSTANCE_ID;
	de_info->flag = 0;
	de_info->min_awake_dw_intvl = 0;
	_os_mem_set(d, &de_info->srvc_ext_info, 0,
		    sizeof(struct rtw_phl_nan_srvc_ext_info));
}

void
phl_nan_deinit_de_priv(struct phl_info_t *phl_info, struct phl_nan_de_priv *de_info)
{
	void *d = phl_to_drvpriv(phl_info);

	/*clear allocated service instance*/
	phl_nan_free_srvc_inst_q(phl_info, de_info, &de_info->publish_srvc_queue);
	phl_nan_free_srvc_inst_q(phl_info, de_info, &de_info->subscribe_srvc_queue);
	phl_nan_free_srvc_inst_q(phl_info, de_info, &de_info->followup_srvc_queue);
	phl_nan_free_srvc_inst_q(phl_info, de_info, &de_info->auto_folloup_srvc_queue);
	pq_deinit(d, &de_info->publish_srvc_queue);
	pq_deinit(d, &de_info->subscribe_srvc_queue);
	pq_deinit(d, &de_info->followup_srvc_queue);
	pq_deinit(d, &de_info->auto_folloup_srvc_queue);

	if (de_info->srvc_num)
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: srvc num err(%d)\n",
			  __func__, de_info->srvc_num);
	de_info->srvc_num = 0;
	de_info->instant_id = 0;
	_os_mem_set(d, &de_info->srvc_ext_info, 0, sizeof(struct rtw_phl_nan_srvc_ext_info));
}

struct rtw_phl_nan_publish_info *
phl_nan_get_pub_from_entry(struct rtw_phl_nan_srvc_instance_entry *srvc_inst)
{
	if (phl_nan_is_pub_srvc_inst(srvc_inst))
		return (srvc_inst->u_srv_data.p_pdata);
	return NULL;
}

struct rtw_phl_nan_subscribe_info *
phl_nan_get_sub_from_entry(struct rtw_phl_nan_srvc_instance_entry *srvc_inst)
{
	if (phl_nan_is_sub_srvc_inst(srvc_inst))
		return (srvc_inst->u_srv_data.s_pdata);
	return NULL;
}

struct rtw_phl_nan_followup_info *
phl_nan_get_followup_from_entry(struct rtw_phl_nan_srvc_instance_entry *srvc_inst)
{
	if (phl_nan_is_followup_srvc_inst(srvc_inst))
		return (srvc_inst->u_srv_data.f_pdata);
	return NULL;
}

u8 *phl_nan_get_srv_id_from_entry(struct rtw_phl_nan_srvc_instance_entry *srvc_inst)
{
	if (phl_nan_is_pub_srvc_inst(srvc_inst))
		return srvc_inst->u_srv_data.p_pdata->service_id;
	else if (phl_nan_is_sub_srvc_inst(srvc_inst))
		return srvc_inst->u_srv_data.s_pdata->service_id;
	else if (phl_nan_is_followup_srvc_inst(srvc_inst))
		return srvc_inst->u_srv_data.f_pdata->matched_service_id;

	return NULL;
}

u64
phl_nan_get_srv_ck_from_entry(struct rtw_phl_nan_srvc_instance_entry *srvc_inst)
{
	if (phl_nan_is_pub_srvc_inst(srvc_inst))
		return srvc_inst->u_srv_data.p_pdata->cookie;
	else if (phl_nan_is_sub_srvc_inst(srvc_inst))
		return srvc_inst->u_srv_data.s_pdata->cookie;
	else if (phl_nan_is_followup_srvc_inst(srvc_inst))
		return srvc_inst->u_srv_data.f_pdata->cookie;

	return 0;
}

u32 phl_nan_get_ttl_from_entry(struct rtw_phl_nan_srvc_instance_entry *srvc_inst)
{
	if (phl_nan_is_pub_srvc_inst(srvc_inst))
		return srvc_inst->u_srv_data.p_pdata->ttl;
	else if (phl_nan_is_sub_srvc_inst(srvc_inst))
		return srvc_inst->u_srv_data.s_pdata->ttl;

	return 0;
}

/* get instance id from a srvc_inst, service instance entry */
u8
phl_nan_get_inid_from_entry(struct rtw_phl_nan_srvc_instance_entry *srvc_inst)
{
	u8 instance_id = 0;

	if (phl_nan_is_pub_srvc_inst(srvc_inst))
		instance_id = srvc_inst->u_srv_data.p_pdata->publish_id;
	else if (phl_nan_is_sub_srvc_inst(srvc_inst))
		instance_id = srvc_inst->u_srv_data.s_pdata->subscribe_id;
	else if (phl_nan_is_followup_srvc_inst(srvc_inst))
		instance_id = srvc_inst->u_srv_data.f_pdata->followup_id;

	return instance_id;
}

struct rtw_phl_nan_func_filter *
phl_nan_get_tx_mf_from_entry(struct rtw_phl_nan_srvc_instance_entry *srvc_inst,
			     u8 *num_tx_mf)
{
	*num_tx_mf = 0;
	if (phl_nan_is_pub_srvc_inst(srvc_inst)) {
		*num_tx_mf = srvc_inst->u_srv_data.p_pdata->num_tx_filters;
		return srvc_inst->u_srv_data.p_pdata->tx_filters;
	} else if (phl_nan_is_sub_srvc_inst(srvc_inst)) {
		*num_tx_mf = srvc_inst->u_srv_data.s_pdata->num_tx_filters;
		return srvc_inst->u_srv_data.s_pdata->tx_filters;
	}

	return NULL;
}

/* serivice info */
struct rtw_phl_nan_service_info *
phl_nan_get_si_from_entry(struct rtw_phl_nan_srvc_instance_entry *srvc_inst)
{
	if (phl_nan_is_pub_srvc_inst(srvc_inst) &&
	    srvc_inst->u_srv_data.p_pdata->srv_spec_info.len != 0)
		return &(srvc_inst->u_srv_data.p_pdata->srv_spec_info);
	else if (phl_nan_is_sub_srvc_inst(srvc_inst) &&
		 srvc_inst->u_srv_data.s_pdata->srv_spec_info.len != 0)
		return &(srvc_inst->u_srv_data.s_pdata->srv_spec_info);
	else if (phl_nan_is_followup_srvc_inst(srvc_inst) &&
		 srvc_inst->u_srv_data.f_pdata->srv_spec_info.len != 0)
		return &(srvc_inst->u_srv_data.f_pdata->srv_spec_info);

	return NULL;
}

u8
phl_nan_get_adw_int_from_entry(struct rtw_phl_nan_srvc_instance_entry *srvc_inst)
{
	if (phl_nan_is_pub_srvc_inst(srvc_inst))
		return srvc_inst->u_srv_data.p_pdata->awake_dw_int;
	if (phl_nan_is_sub_srvc_inst(srvc_inst))
		return srvc_inst->u_srv_data.s_pdata->awake_dw_int;

	return 0;
}

u8
phl_nan_chk_pub_solicited_tx_for_srvc_inst(struct rtw_phl_nan_srvc_instance_entry *srvc_inst,
					   u8 tx_type)
{
	if (!phl_nan_is_pub_srvc_inst(srvc_inst))
		return false;

	return TEST_STATUS_FLAG(srvc_inst->u_srv_data.p_pdata->solicited_tx_type, tx_type);
}

void
_phl_nan_free_srv_info(void *drv, struct rtw_phl_nan_service_info srv_spec_info)
{
	if (NULL == srv_spec_info.info)
		return;

	/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s free len(%d)\n", __func__,
		  srv_spec_info.len); */

	_os_kmem_free(drv, (void *)srv_spec_info.info, sizeof(u8) * (srv_spec_info.len));
}

void
_phl_nan_free_srvc_srf(void *drv, struct rtw_phl_nan_subscribe_info *sub_data)
{
	if (NULL == sub_data)
		return;

	if (sub_data->srf_bf_len)
		_os_kmem_free(drv, (void *)sub_data->srf_bf,
			      sizeof(u8) * sub_data->srf_bf_len);

	if (sub_data->srf_num_macs)
		_os_kmem_free(drv, (void *)sub_data->srf_macs,
			     sizeof(struct rtw_phl_nan_srf_mac_addrs) * sub_data->srf_num_macs);
}

void
rtw_phl_nan_free_service_data(void *drv,
			      u8 service_type,
			      union rtw_phl_nan_srv_data p_srv_data)
{
	/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: srvc type(%x)\n",
		  __func__, service_type); */
	switch (service_type) {
	case SCF_TYPE_PUBLISH:
		_phl_nan_free_srv_info(drv, p_srv_data.p_pdata->srv_spec_info);
		_phl_nan_free_srv_info(drv, p_srv_data.p_pdata->data_spec_info);
		_os_kmem_free(drv, (void *)p_srv_data.p_pdata,
			      sizeof(struct rtw_phl_nan_publish_info));
		break;
	case SCF_TYPE_SUBSCRIBE:
		_phl_nan_free_srv_info(drv, p_srv_data.s_pdata->srv_spec_info);
		_phl_nan_free_srvc_srf(drv, p_srv_data.s_pdata);
		_os_kmem_free(drv, (void *)p_srv_data.s_pdata,
			      sizeof(struct rtw_phl_nan_subscribe_info));
		break;
	case SCF_TYPE_FOLLOWUP:
		_phl_nan_free_srv_info(drv, p_srv_data.f_pdata->srv_spec_info);
		_os_kmem_free(drv, (void *)p_srv_data.f_pdata,
			      sizeof(struct rtw_phl_nan_followup_info));
		break;
	default:
		PHL_WARN("%s: srvc type unkown(%x)\n", __func__, service_type);
		break;
	}
}

void
rtw_phl_nan_free_service_entry(void *d,
			       struct rtw_phl_nan_srvc_instance_entry *entry)
{
	struct rtw_phl_nan_dp_auto_rsp_info *auto_rsp = &entry->ndp_auto_rsp_info;

	if (auto_rsp->info.len) {
		_os_kmem_free(d, (void *)auto_rsp->info.pbuf, auto_rsp->info.len);
		auto_rsp->info.len = 0;
		auto_rsp->info.pbuf = NULL;
	}

	_os_kmem_free(d, (void *)entry, sizeof(struct rtw_phl_nan_srvc_instance_entry));
}

/**
 * phl_nan_remove_srvc_instance_data
 *   - Free a NAN discovery engine service function instance
 * @padapter
 * @p_srvc: NAN service function instance
 * @b_queue_free:
 * (ac) _rtw_remove_srvc_instance_data
 *
 */
static void _phl_nan_remove_srvc_instance_data(
	void *d, struct phl_nan_de_priv *de_info,
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry)
{
	u8 type;
	struct phl_queue *p_dedicated_queue = NULL;

	type = p_srvc_entry->service_control & SCF_TYPE_MASK;
	switch (type) {
	case SCF_TYPE_PUBLISH:
		p_dedicated_queue = &(de_info->publish_srvc_queue);
		break;
	case SCF_TYPE_SUBSCRIBE:
		p_dedicated_queue = &(de_info->subscribe_srvc_queue);
		break;
	case SCF_TYPE_FOLLOWUP:
		p_dedicated_queue = &(de_info->followup_srvc_queue);
		break;
	default:
		PHL_WARN("%s: srvc type unkown(%x)\n", __func__, type);
		break;
	}

	if (!p_dedicated_queue)
		return;

	/* already locked by caller */
	list_del(&p_srvc_entry->list);
	p_dedicated_queue->cnt++;

	de_info->srvc_num--;
}

void
phl_nan_free_srvc_inst_q(struct phl_info_t *phl_info,
			 struct phl_nan_de_priv *de_info,
			 struct phl_queue *srvc_q)
{
	void *d = phl_to_drvpriv(phl_info);
	struct rtw_phl_nan_srvc_instance_entry *srvc_inst = NULL;
	_os_list *obj = NULL;
	u8 srv_type = 0xff;
	u8 srvc_inst_id = 0;
	u64 srvc_cookie = 0;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: cnt %d\n", __func__, srvc_q->cnt);

	if (srvc_q->cnt == 0)
		return;

	while (pq_pop(d, srvc_q, &obj, _first, _bh)) {
		srvc_inst = (struct rtw_phl_nan_srvc_instance_entry *)obj;
		srv_type = srvc_inst->service_control & SCF_TYPE_MASK;
		srvc_cookie = phl_nan_get_srv_ck_from_entry(srvc_inst);
		srvc_inst_id = phl_nan_get_inid_from_entry(srvc_inst);

		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: srvc_num(%u) srvc_type(%d) => Run(instance_id(%u), cookie(%llu)) !!\n",
			  __func__,
			  de_info->srvc_num,
			  srv_type,
			  srvc_inst_id,
			  srvc_cookie);

		de_info->srvc_num--;

		if (srv_type == 0xff)
			continue;

		rtw_phl_nan_free_service_data(d, srv_type, srvc_inst->u_srv_data);
		rtw_phl_nan_free_service_entry(d, srvc_inst);
	}
}

/**
 * _phl_nan_dump_service_specific_info - Dump all NAN function
 * @srvc_info: service info
 * (ac) rtw_dump_nan_service_specific_info
 */
void _phl_nan_dump_service_specific_info(
	struct phl_nan_srvc_specific_info *srvc_info)
{
	u32 i = 0;

	if (!srvc_info->len)
		return;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "service_specific_info: %s\n", srvc_info->pbuf);
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "service_specific_info_len: %d\n", srvc_info->len);
	for (i = 0; i < srvc_info->len; i++) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "\t [0x%X] %02X %02X %02X %02X %02X %02X %02X %02X \n",
			  i,
			  srvc_info->pbuf[i],
			  srvc_info->pbuf[i + 1],
			  srvc_info->pbuf[i + 2],
			  srvc_info->pbuf[i + 3],
			  srvc_info->pbuf[i + 4],
			  srvc_info->pbuf[i + 5],
			  srvc_info->pbuf[i + 6],
			  srvc_info->pbuf[i + 7]);
		i = i + 7;
	}
}

u8
phl_nan_alloc_peer_srvc_entry(void *d,
			      struct phl_queue *srvc_info_q,
			      struct phl_nan_peer_srvc_info_entry **p_peer_srvc_info)
{
	struct phl_nan_peer_srvc_info_entry *entry_temp =
		_os_kmem_alloc(d, sizeof(struct phl_nan_peer_srvc_info_entry));

	if (!entry_temp) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: create entry_temp fail!\n", __func__);
		return false;
	}

	pq_push(d, srvc_info_q, (_os_list *)entry_temp, _tail, _bh);
	*p_peer_srvc_info = entry_temp;
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: create peer srvc info!\n", __func__);

	return true;
}

/**
 * _phl_nan_free_peer_srvc_entry_data - Free data of a service info entry.
 * @d:
 * @p_peer_srvc_info: peer service info entry
 * (ac) _rtw_free_nan_peer_srvc_info_entry_data
 *
 */
void
_phl_nan_free_peer_srvc_entry_data(void *d,
				   struct phl_nan_peer_srvc_info_entry *peer_srvc)
{
	if (!peer_srvc ||
	    !TEST_STATUS_FLAG(peer_srvc->flag, PEER_SRVC_INFO_FLAG_VALID))
		return;

	if (peer_srvc->srvc_info.len) {
		_os_kmem_free(d, peer_srvc->srvc_info.pbuf,
			      peer_srvc->srvc_info.len);
	}
	if (peer_srvc->followup_srvc_info.len) {
		_os_kmem_free(d, peer_srvc->followup_srvc_info.pbuf,
			      peer_srvc->followup_srvc_info.len);
	}
	if (peer_srvc->gen_srvc_info.service_name_len) {
		_os_kmem_free(d, peer_srvc->gen_srvc_info.service_name,
			      peer_srvc->gen_srvc_info.service_name_len);
	}

	peer_srvc->assoc_srvc_inst_ck = 0;
	CLEAR_STATUS_FLAG(peer_srvc->flag, PEER_SRVC_INFO_FLAG_VALID);
}

void phl_nan_free_peer_srvc_queue(void *d, struct phl_queue *srvc_info_q)
{
	struct phl_nan_peer_srvc_info_entry *srvc_entry = NULL, *n = NULL;

	_os_spinlock(d, &srvc_info_q->lock, _bh, NULL);
	phl_list_for_loop_safe(srvc_entry, n,
			       struct phl_nan_peer_srvc_info_entry,
			       &srvc_info_q->queue, list) {
		list_del(&srvc_entry->list);
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "free peer srvc info[%02x:%02x:%02x:%02x:%02x:%02x]\n",
			  srvc_entry->service_id[0],
			  srvc_entry->service_id[1],
			  srvc_entry->service_id[2],
			  srvc_entry->service_id[3],
			  srvc_entry->service_id[4],
			  srvc_entry->service_id[5]);
		_phl_nan_free_peer_srvc_entry_data(d, srvc_entry);
		_os_kmem_free(d, srvc_entry, sizeof(struct phl_nan_peer_srvc_info_entry));
	}
	_os_spinunlock(d, &srvc_info_q->lock, _bh, NULL);
}

void _phl_nan_update_awake_dw_interval(struct phl_nan_de_priv *de_info,
				       u8 new_awakw_dw_intvl)
{
	de_info->min_awake_dw_intvl =
		(!de_info->min_awake_dw_intvl) ? (new_awakw_dw_intvl) :
		((new_awakw_dw_intvl < de_info->min_awake_dw_intvl) ?
		(new_awakw_dw_intvl) : (de_info->min_awake_dw_intvl));
	/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "min_awake_dw_intvl = %d\n",
		  de_info->min_awake_dw_intvl); */
}

/*
 *  iw cmd "add_func", a native iw nan cmd to add a service,
 *  does not support some basic nan parameters for adding service,
 *  so these parameters need to pass to driver via a vendor cmd(srv_ext_info),
 *  and are saved in (struct rtw_nan_de_priv) until here.
 *
 *  Update to the service instance which is about to add and
 *  zero the memory in (struct rtw_nan_de_priv)
 */
void phl_nan_update_service_ext_info_from_de(
	void *d, struct phl_nan_de_priv *de_info,
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry)
{
	struct rtw_phl_nan_publish_info *p_pub_info = NULL;
	struct rtw_phl_nan_subscribe_info *p_sub_info = NULL;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s ==> \n", __func__);

	p_pub_info = phl_nan_get_pub_from_entry(p_srvc_entry);
	p_sub_info = phl_nan_get_sub_from_entry(p_srvc_entry);

	if (p_pub_info != NULL) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: p_pub_info\n", __func__);

		p_pub_info->period = de_info->srvc_ext_info.period;
		p_pub_info->awake_dw_int = de_info->srvc_ext_info.awake_dw_int;
		p_pub_info->event_condition =
			de_info->srvc_ext_info.event_condition;

		if (de_info->srvc_ext_info.further_srvc_disc) {
			SET_STATUS_FLAG(p_pub_info->sde_control,
					NAN_SDE_CTRL_FSD_REQUIRED);

			if (de_info->srvc_ext_info.further_srvc_disc_func == 0)
				CLEAR_STATUS_FLAG(p_pub_info->sde_control,
						  NAN_SDE_CTRL_FSD_WITH_GAS);
		}

		if (de_info->srvc_ext_info.data_path_flag) {
			SET_STATUS_FLAG(p_pub_info->sde_control,
					NAN_SDE_CTRL_DATAPATH_REQ_REQUIRED);

			if (de_info->srvc_ext_info.data_path_type == 0)
				CLEAR_STATUS_FLAG(p_pub_info->sde_control,
						  NAN_SDE_CTRL_DATAPATH_TYPE);
		}

		if (de_info->srvc_ext_info.qos) {
			SET_STATUS_FLAG(p_pub_info->sde_control,
					NAN_SDE_CTRL_QOS_REQUIRED);
			p_pub_info->qos_req.max_latency =
				de_info->srvc_ext_info.qos_req.max_latency;
			p_pub_info->qos_req.min_duration =
				de_info->srvc_ext_info.qos_req.min_duration;
		}

		if (de_info->srvc_ext_info.sec) {
			SET_STATUS_FLAG(p_pub_info->sde_control,
					NAN_SDE_CTRL_SEC_REQUIRED);

			_os_mem_cpy(d, p_pub_info->sec_req.pmk,
				    de_info->srvc_ext_info.pmk_publish,
				    MAX_PMK_LEN);
		}

		if (de_info->srvc_ext_info.service_id[0] != 0 &&
		    de_info->srvc_ext_info.service_id[1] != 0) {
			_os_mem_cpy(d, p_pub_info->service_id,
				    de_info->srvc_ext_info.service_id,
				    NAN_SRVC_ID_LEN);
		}
	} else if (p_sub_info != NULL) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: sub_info\n", __func__);
		p_sub_info->period = de_info->srvc_ext_info.period;
		p_sub_info->awake_dw_int = de_info->srvc_ext_info.awake_dw_int;

		if (de_info->srvc_ext_info.service_id[0] != 0 &&
		    de_info->srvc_ext_info.service_id[1] != 0) {
			_os_mem_cpy(d,
				    p_sub_info->service_id,
				    de_info->srvc_ext_info.service_id,
				    NAN_SRVC_ID_LEN);
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "%s update service_id: %02x:%02x:%02x:%02x:%02x:%02x\n",
				  __func__,
				  p_sub_info->service_id[0],
				  p_sub_info->service_id[1],
				  p_sub_info->service_id[2],
				  p_sub_info->service_id[3],
				  p_sub_info->service_id[4],
				  p_sub_info->service_id[5]);
		}
	} else {
		/* no parameter in srvc_ext_info need to update to followup service */
		return;
	}

	/* clear memory for the next service-adding */
	_os_mem_set(d, &de_info->srvc_ext_info, 0, sizeof(struct rtw_phl_nan_srvc_ext_info));
}

/*
 * 1. Use crc32.c from open source
 * http://www.opensource.apple.com/source/xnu/xnu-1456.1.26/bsd/libkern/crc32.c
 *
 * 2. Calculate BloomFilter H(j,X,M)
 *    where j is index, X is macaddress, and M is Bloom Filter size in bits
 *
 */
u32 _phl_nan_crc32(u32 crc, const void *buf, size_t size)
{
	const char *p;

	p = (const char *)buf;
	while (size--)
		crc = crc32_tab[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
	return crc;
}

u16 _phl_nan_hash(void *d, u8 j, const u8 *buf, u16 len)
{
	char a[7] = {0};
	u32 b = 0;
	u32 crc_init = 0xffffffff;

	if (j > 0x0f)
		return 0xffff;
	a[0] = j;
	_os_mem_cpy(d, a + 1, buf, 6);
	b = _phl_nan_crc32(crc_init, (const char *)a, sizeof(a));
	b &= 0x0000ffff;

#ifdef CONFIG_PHL_NAN_DEBUG
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: Hash %d return bit %d\n",
		  __func__, j, (b % len));
#endif

	return b % len;
}

u8 _phl_nan_bloom_filter(void *d, u8 idx, const u8 *addr, u16 len, u8 *out)
{
	u16 absbit = 0;
	u16 filterbyte = 0;
	u16 filterbit = 0;
	u8 i = 0, j = 0;

	if (idx > 4)
		return 0xff;

	j = (idx * 4);

	while (i < 4) {
		absbit = _phl_nan_hash(d, j, addr, len);
		filterbyte = absbit / 8;
		filterbit = absbit % 8;
		*(out + filterbyte) |= (1 << filterbit);
		i++;
		j++;
	}

#ifdef CONFIG_PHL_NAN_DEBUG
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: [%d] content = \n", __func__,
		  idx);
	debug_dump_data(out, (len / 8), " ");
#endif

	return 0;
}

/*
 * _phl_nan_chk_srf_bf - Check service response filter with bloom filter
 * @nmi: Own NAN nmi mac address
 * @bf_idx: bloom filter index
 * @pbf: bloom filter
 * @bf_len: length of bloom filter
 * @srf_include: include bit in the srf ctrl field
 *    - 0 indicates that STAs present in the Address Set shall not send responses
 *	to the received query discovery frame.
 *    - 1 indicates that only STAs present in the Address Set shall send a
 *	response to a received query discovery frame.
 * RETURN:
 * true: Response.
 * false: Should not response.
 */
u8
_phl_nan_chk_srf_bf(void *d, u8 *nmi, u8 bf_idx, u8 *pbf, u8 bf_len, u8 srf_incl)
{
	u8 *own_set = NULL;
	u8 i = 0, cmp = 0;
	bool is_match = true;
	u16 bf_bits = bf_len * 8;

	own_set = _os_kmem_alloc(d, bf_len);
	if (own_set == NULL) {
		PHL_ERR("%s(): allocate own_set fail.\n", __func__);
		return false;
	}
	_phl_nan_bloom_filter(d, bf_idx, nmi, bf_bits, own_set);

	for (i = 0; i < bf_len; i++) {
		cmp = own_set[i] & *(pbf + i);
		if (0 != _os_mem_cmp(d, &own_set[i], &cmp, 1)) {
			is_match = false;
			break;
		}
	}

	_os_kmem_free(d, own_set, bf_len);

	return (is_match) ? ((srf_incl) ? true : false) :
			    ((srf_incl) ? false : true);
}

/*
 * _phl_nan_chk_srf_mac - Check service response filter with MAC address set
 * @nmi: Own NAN nmi mac address
 * @addr_set: the starting of address set in srf field of sda attr
 * @addr_set_len: length of the adress set
 * @srf_incl: include bit in the srf ctrl field
 *    - 0 indicates that STAs present in the Address Set shall not send responses
 *	to the received query discovery frame.
 *    - 1 indicates that only STAs present in the Address Set shall send a
 *	response to a received query discovery frame.
 * RETURN:
 * true: Response.
 * false: Should not response.
 */
u8
_phl_nan_chk_srf_mac(void *d, u8 *nmi, u8 *addr_set, u8 addr_set_len, u8 srf_incl)
{
	u8 i = 0;
	bool is_match = false;
	u8 stop = addr_set_len / MAC_ALEN;

	for (i = 0; i < stop; i++) {
		if (0 == _os_mem_cmp(d, nmi, (addr_set + i * MAC_ALEN), MAC_ALEN)) {
			is_match = true;
			break;
		}
	}

	return (is_match) ? ((srf_incl) ? true : false) :
			    ((srf_incl) ? false : true);
}

/*
 * _phl_nan_is_match_pub_rx_match_fltr - Check match filter for received subscribe msg
 * @p_pub_data: Local publish service
 * @pmf_list: match filter list in sda of sdf
 * @mf_list_len: length of match filter list
 * (ac) is_match_publish_rx_match_filter
 *
 * RETURN:
 * true: Match.
 * false: Not match.
 */
u8
_phl_nan_is_match_pub_rx_match_fltr(void *d,
				    struct rtw_phl_nan_publish_info *pub_data,
				    u8 *pmf_list,
				    u8 mf_list_len)
{
	u8 self_mf_idx = 0, rx_mf_len = 0, rx_mf_offset = 0;
	struct rtw_phl_nan_func_filter *rx_filters = pub_data->rx_filters;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s\n", __func__);

	while (rx_mf_offset < mf_list_len) {
		rx_mf_len = *(pmf_list + rx_mf_offset);
		rx_mf_offset++;

#ifdef CONFIG_PHL_NAN_DEBUG
		debug_dump_data((pmf_list + rx_mf_offset), rx_mf_len,
				"MF in received subscribe msg:");
#endif

		if (self_mf_idx < pub_data->num_rx_filters) {
#ifdef CONFIG_PHL_NAN_DEBUG
			debug_dump_data(rx_filters[self_mf_idx].filter,
					rx_filters[self_mf_idx].len,
					"MF in rx filter:");
#endif

			if ((rx_mf_len != 0) && (rx_filters[self_mf_idx].len != 0)) {
				if (rx_mf_len != rx_filters[self_mf_idx].len)
					return false;
				if (0 != _os_mem_cmp(d,
						     pmf_list + rx_mf_offset,
						     rx_filters[self_mf_idx].filter,
						     rx_mf_len))
					return false;
			}
		} else if (rx_mf_len != 0) {
			return false;
		}

		self_mf_idx++;
		rx_mf_offset += rx_mf_len;
	}

	return true;
}

/*
 * _phl_nan_is_match_sub_rx_match_fltr - Check match filter for received publish msg
 * @p_sub_data: Local subscribe service
 * @pmf_list: match filter list in sda of sdf
 * @mf_list_len: length of match filter list
 * (ac) is_match_subscribe_rx_match_filter
 *
 * RETURN:
 * true: Match.
 * false: Not match.
 */
u8
_phl_nan_is_match_sub_rx_match_fltr(void *d,
				    struct rtw_phl_nan_subscribe_info *sub_data,
				    u8 *pmf_list,
				    u8 mf_list_len)
{
	u8 self_mf_idx = 0, rx_mf_len = 0, rx_mf_offset = 0;
	struct rtw_phl_nan_func_filter *rx_filters = sub_data->rx_filters;

	/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s\n", __func__);*/

	for (; self_mf_idx < sub_data->num_rx_filters; self_mf_idx++) {
		if (rx_mf_offset >= mf_list_len) {
			if (rx_filters[self_mf_idx].len != 0)
				return false;
			continue;
		}

		rx_mf_len = *(pmf_list + rx_mf_offset);
		rx_mf_offset++;

#ifdef CONFIG_PHL_NAN_DEBUG
		debug_dump_data((pmf_list + rx_mf_offset), rx_mf_len,
				"MF in received publish msg:");
		debug_dump_data(rx_filters[self_mf_idx].filter,
				rx_filters[self_mf_idx].len,
				"MF in rx filter:");
#endif

		if ((rx_mf_len != 0) && (rx_filters[self_mf_idx].len != 0)) {
			if (rx_mf_len != rx_filters[self_mf_idx].len)
				return false;
			if (0 != _os_mem_cmp(d,
					     pmf_list + rx_mf_offset,
					     rx_filters[self_mf_idx].filter,
					     rx_mf_len))
				return false;
		}

		rx_mf_offset += rx_mf_len;
	}

	return true;
}

/*
 * _phl_nan_de_chk_trigger_cond - Check receive sdf with local service function
 * @d:
 * @nmi: self nmi
 * @srvc_inst: local service inst entry
 * @psda: sda of received sdf
 * @filter_len:
 * (ac) _rtw_nan_de_check_trigger_condition
 *
 * RETURN:
 * true: Service match.
 * false: Service not match.
 */
u8
_phl_nan_de_chk_trigger_cond(void *d,
			     u8 *nmi,
			     struct rtw_phl_nan_srvc_instance_entry *srvc_inst,
			     u8 *psda,
			     u32 *filter_len)
{
	u8 srvc_ctrl = 0;
	u8 mf_offset = 0, mf_list_len = 0;
	u8 srf_offset = 0, srf_len = 0, srf_addr_set_len = 0, srf_ctrl = 0;
	u8 *pmf_list = NULL, *psrf = NULL, *psrf_addr_set = NULL;

	/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s\n", __func__);*/
	*filter_len = 0;
	srvc_ctrl = *(psda + SDA_SERVICE_CONTROL_OFFSET);

	PHL_TRACE(COMP_PHL_NAN, _PHL_DEBUG_, "%s: srvc_ctrl = %x\n", __func__, srvc_ctrl);

	/* check if matching filter exist */
	if (srvc_ctrl & BIT_SCF_MF_PRESENT) {
		mf_offset = ((srvc_ctrl & BIT_SCF_BB_PRESENT))?
			    (SDA_OPTIONAL_FIELD_OFFSET + SDA_BINDING_BITMAP_LEN):
			    (SDA_OPTIONAL_FIELD_OFFSET);
		mf_list_len = *(psda + mf_offset);
		*filter_len += (1 + mf_list_len);
		pmf_list = psda + mf_offset + 1;
	}

	if (phl_nan_is_pub_srvc_inst(srvc_inst) &&
	    phl_nan_is_scf_type_sub(srvc_ctrl) &&
	    false == _phl_nan_is_match_pub_rx_match_fltr(
				d,
				phl_nan_get_pub_from_entry(srvc_inst),
				pmf_list,
				mf_list_len))
		return false;

	if (phl_nan_is_sub_srvc_inst(srvc_inst) &&
	    phl_nan_is_scf_type_pub(srvc_ctrl) &&
	    false == _phl_nan_is_match_sub_rx_match_fltr(
				d,
				phl_nan_get_sub_from_entry(srvc_inst),
				pmf_list,
				mf_list_len))
		return false;

	/* check if service response filter exist */
	if (srvc_ctrl & BIT_SCF_SRF_PRESENT) {
		srf_offset = (srvc_ctrl & BIT_SCF_BB_PRESENT)? \
			     (SDA_OPTIONAL_FIELD_OFFSET + SDA_BINDING_BITMAP_LEN): \
			     (SDA_OPTIONAL_FIELD_OFFSET);
		srf_offset += (mf_list_len)? (mf_list_len + 1) : 0;
		srf_len = *(psda + srf_offset);
		*filter_len += (1 + srf_len);
		srf_addr_set_len = srf_len - 1; /* srf_addr_set_len = srf_len - SRF_CONTROL_FIELD_LEN(1) */
		psrf = psda + srf_offset + 1;
		srf_ctrl = *psrf;
		psrf_addr_set = psrf + 1;

		/* SRF type: 0 (mac address), 1 (bloom filter) */
		if ((srf_ctrl & BIT_SRF_CTRL_TYPE) == 0) {
			if (false == _phl_nan_chk_srf_mac(
					d,
					nmi,
					psrf_addr_set,
					srf_addr_set_len,
					srf_ctrl & BIT_SRF_CTRL_INCLUDE))
				return false;
		} else if (false == _phl_nan_chk_srf_bf(
					d,
					nmi,
					(srf_ctrl & SRF_CTRL_BFI_MASK) >> SRF_CTRL_BFI_SHIFT,
					psrf_addr_set,
					srf_addr_set_len,
					srf_ctrl & BIT_SRF_CTRL_INCLUDE)) {
			return false;
		}
	}

	return true;
}

u8
phl_nan_peer_srvc_info_exist(void *d,
			     void *p_peer_info_entry,
			     u8 *pdata,
			     u8 search_type,
			     struct phl_nan_peer_srvc_info_entry **peer_srvc_info)
{
	struct phl_nan_peer_info_entry *peer = p_peer_info_entry;
	struct phl_nan_peer_srvc_info_entry *srvc_entry = NULL, *n = NULL;

	*peer_srvc_info = NULL;

	_os_spinlock(d, &peer->srvc_info_queue.lock, _bh, NULL);
	phl_list_for_loop_safe(srvc_entry, n,
			       struct phl_nan_peer_srvc_info_entry,
			       &peer->srvc_info_queue.queue, list) {
		if (search_type == GET_PEER_SRVC_INFO_BY_SRVC_ID &&
		    0 == _os_mem_cmp(d, srvc_entry->service_id, pdata, NAN_SRVC_ID_LEN)) {
			*peer_srvc_info = srvc_entry;
			break;
		}

		if (search_type == GET_PEER_SRVC_INFO_BY_INST_ID &&
		    srvc_entry->peer_inst_id == *pdata) {
			*peer_srvc_info = srvc_entry;
			break;
		}
	}
	_os_spinunlock(d, &peer->srvc_info_queue.lock, _bh, NULL);

	return (*peer_srvc_info == NULL) ? (false) : (true);
}

/**
 * _phl_nan_sdf_device_cap_parsing
 *   - Parse the Device Capability attribute from a peer
 * @nan_info:
 * @pdev_cap: dev_cap field address
 * @ppeer_addr: mac address of sdf
 * (ac) _rtw_nan_sdf_device_cap_parsing
 *
 * RETURN:
 * true: dev_cap update success
 * false: dev_cap update fail
 */
u8 _phl_nan_sdf_device_cap_parsing(struct phl_nan_info *nan_info,
				   u8 *pdev_cap, u8 *ppeer_addr)
{
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_peer_info_entry *p_peer_info_entry = NULL;
	enum rtw_phl_status r_status = RTW_PHL_STATUS_FAILURE;

	/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s\n", __func__); */

	if (true == phl_nan_is_peer_info_exist(nan_info, ppeer_addr,
					       &p_peer_info_entry)) {
#ifdef CONFIG_NAN_R2
		r_status = phl_nan_schdlr_parse_dev_cap_attr(&nan_info->scheduler_priv,
							     p_peer_info_entry,
							     pdev_cap,
							     false);
#else
		r_status = RTW_PHL_STATUS_SUCCESS;
#endif
		phl_nan_done_peer_info_entry(d, nan_info);
	} else {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s Peer Info Not Found\n", __func__);
	}

	return (r_status) ? false : true;
}

/* (ac) copy_peer_srvc_info_content */
u8
_phl_nan_copy_peer_srvc_info_content(void *d,
				     struct phl_nan_peer_srvc_info_entry *peer_srvc_entry,
				     u8 *pbuf,
				     u16 len)
{
	/* follow up service */
	if (phl_nan_is_peer_followup_srvc(peer_srvc_entry)) {
		if (peer_srvc_entry->followup_srvc_info.pbuf &&
		    peer_srvc_entry->followup_srvc_info.len &&
		    (len > peer_srvc_entry->followup_srvc_info.len)) {
			_os_kmem_free(d,
				      peer_srvc_entry->followup_srvc_info.pbuf,
				      peer_srvc_entry->followup_srvc_info.len);
			peer_srvc_entry->followup_srvc_info.pbuf = NULL;
			peer_srvc_entry->followup_srvc_info.len = 0;
		}

		if (NULL == peer_srvc_entry->followup_srvc_info.pbuf)
			peer_srvc_entry->followup_srvc_info.pbuf = _os_kmem_alloc(d, len);
		if (NULL == peer_srvc_entry->followup_srvc_info.pbuf)
			return false;

		peer_srvc_entry->followup_srvc_info.len = len;
		_os_mem_cpy(d, peer_srvc_entry->followup_srvc_info.pbuf, pbuf, len);

		return true;
	}

	/* pub or sub service */
	if (peer_srvc_entry->srvc_info.pbuf &&
	    peer_srvc_entry->srvc_info.len &&
	    (len > peer_srvc_entry->srvc_info.len)) {
		_os_kmem_free(d,
			      peer_srvc_entry->srvc_info.pbuf,
			      peer_srvc_entry->srvc_info.len);
		peer_srvc_entry->srvc_info.pbuf = NULL;
		peer_srvc_entry->srvc_info.len = 0;
	}

	if (NULL == peer_srvc_entry->srvc_info.pbuf)
		peer_srvc_entry->srvc_info.pbuf = _os_kmem_alloc(d, len);
	if (NULL == peer_srvc_entry->srvc_info.pbuf)
		return false;

	peer_srvc_entry->srvc_info.len = len;
	_os_mem_cpy(d, peer_srvc_entry->srvc_info.pbuf, pbuf, len);

	return true;
}

/* (ac) _rtw_copy_peer_srvc_name */
u8
_phl_nan_copy_peer_srvc_name(void *d,
			     struct phl_nan_peer_srvc_info_entry *srvc_info,
			     u8 *pbuf,
			     u16 len)
{
	/* realloc memory if len is not enough */
	if (srvc_info->gen_srvc_info.service_name &&
	    srvc_info->gen_srvc_info.service_name_len &&
	    (len > srvc_info->gen_srvc_info.service_name_len)) {
		_os_kmem_free(d,
			      srvc_info->gen_srvc_info.service_name,
			      srvc_info->gen_srvc_info.service_name_len);
		srvc_info->gen_srvc_info.service_name = NULL;
		srvc_info->gen_srvc_info.service_name_len = 0;
	}
	if (NULL == srvc_info->gen_srvc_info.service_name)
		srvc_info->gen_srvc_info.service_name = _os_kmem_alloc(d, len);
	if (NULL == srvc_info->gen_srvc_info.service_name)
		return false;

	srvc_info->gen_srvc_info.service_name_len = len;
	_os_mem_cpy(d, srvc_info->gen_srvc_info.service_name, pbuf, len);

	return true;
}

/**
 * _phl_nan_parse_generic_srvc - Parse the Generic Service Protocol
 * @d:
 * @peer_srvc_entry: peer's service
 * @psrvc_info: service info
 * @srvc_len: service length
 * (ac) _rtw_nan_generic_service_parsing
 *
 * RETURN:
 * true: is a generic service
 * false: not a generic service
 */
u8
_phl_nan_parse_generic_srvc(void *d,
			    struct phl_nan_peer_srvc_info_entry *peer_srvc_entry,
			    u8 *psrvc_info,
			    u16 srvc_len)
{
	u16 gen_srvc_attr_len = 0;
	u16 sub_attr_len = 0;
	u8 *pgen_srvc_attr_start = NULL;
	u8 *pgen_srvc_attr_end = NULL;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s n", __func__);

	if (phl_nan_is_start_with_wfa_oui(d, psrvc_info) == false)
		return false;

	if (GET_SRVC_INFO_FIELD_PROTO_TYPE(psrvc_info) != NAN_SRVC_PROTOCOL_GENIC)
		return false;

	pgen_srvc_attr_start = psrvc_info + SERVICE_INFO_FIELD_SPECIFIC_INFO_OFFSET;
	gen_srvc_attr_len = srvc_len - 4;
	pgen_srvc_attr_end = pgen_srvc_attr_start + gen_srvc_attr_len;

	while ((pgen_srvc_attr_start + NAN_ATTR_CONTENT_OFFSET) < pgen_srvc_attr_end) {
		_os_mem_cpy(d, (u8 *)&sub_attr_len, pgen_srvc_attr_start + 1, 2);
		sub_attr_len = le16_to_cpu(sub_attr_len);
		if (!sub_attr_len)
			return false;

		switch (pgen_srvc_attr_start[0]) {
		case NAN_GEN_SRVC_TRANSPORT_PORT:
			_os_mem_cpy(d,
				    &peer_srvc_entry->gen_srvc_info.trans_port,
				    pgen_srvc_attr_start +
				    NAN_ATTR_CONTENT_OFFSET, 2);
			peer_srvc_entry->gen_srvc_info.trans_port =
				le16_to_cpu(peer_srvc_entry->gen_srvc_info.trans_port);
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "%s Sub_attribute ID = 0x%x, Transport port[%d]n",
				  __func__,
				  pgen_srvc_attr_start[0],
				  peer_srvc_entry->gen_srvc_info.trans_port);
			break;
		case NAN_GEN_SRVC_TRANSPORT_PROTOCOL:
			peer_srvc_entry->gen_srvc_info.trans_protocol =
				pgen_srvc_attr_start[NAN_ATTR_CONTENT_OFFSET];
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "%s Sub_attribute ID = 0x%x, Transport Protocol[%d]n",
				  __func__,
				  pgen_srvc_attr_start[0],
				  peer_srvc_entry->gen_srvc_info.trans_protocol);
			break;
		case NAN_GEN_SRVC_SERVICE_NAME:
			_phl_nan_copy_peer_srvc_name(
				d, peer_srvc_entry,
				pgen_srvc_attr_start + NAN_ATTR_CONTENT_OFFSET,
				sub_attr_len);
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "%s Sub_attribute ID = 0x%x\n",
				  __func__,
				  pgen_srvc_attr_start[0]);
			/* RTW_INFO_DUMP("Service Name : ",
				      peer_srvc_entry->gen_srvc_info.service_name,
				      peer_srvc_entry->gen_srvc_info.service_name_len); */
			break;
		case NAN_GEN_SRVC_NAME_OF_SRVC_INST:
			break;
		case NAN_GEN_SRVC_TEXT_INFO:
			break;
		case NAN_GEN_SRVC_UUID:
			break;
		case NAN_GEN_SRVC_BLOB:
			break;
		default:
			break;
		}

		pgen_srvc_attr_start += (sub_attr_len + NAN_ATTR_CONTENT_OFFSET);
	}

	return true;
}

/**
 * _phl_nan_sdf_sdea_parsing
 *   - Parse the Service Descriptor Extension attribute from a peer
 * @padapter
 * @psdea: sdea field address
 * @ppeer_addr: mac address of sdf
 * (ac) _rtw_nan_sdf_sdea_parsing
 *
 * RETURN:
 * true: SDEA update success
 * false: SDEA update fail
 */
u8
_phl_nan_sdf_sdea_parsing(struct phl_nan_info *nan_info,
			  u8 *psdea,
			  u8 *peer_addr,
			  u16 attr_len)
{
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_peer_info_entry *peer_entry = NULL;
	struct phl_nan_peer_srvc_info_entry *peer_srvc_entry = NULL;
	u8 offset = SERVICE_DESC_EXT_FIELD_CONTROL_OFFSET +
		    SERVICE_DESC_EXT_FIELD_CONTROL_LEN;
	u16 srvc_info_len = 0;
	u8 ret = false;

	/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s\n", __func__); */

	if (true != phl_nan_is_peer_info_exist(nan_info, peer_addr, &peer_entry))
		return ret;

	if (true != phl_nan_peer_srvc_info_exist(d, (void *)peer_entry,
						 (psdea + NAN_ATTR_CONTENT_OFFSET),
						 GET_PEER_SRVC_INFO_BY_INST_ID,
						 &peer_srvc_entry)) {
		goto unlock_exit;
	}

	_os_mem_cpy(d, &(peer_srvc_entry->sdea_ctrl),
		    (psdea + SERVICE_DESC_EXT_FIELD_CONTROL_OFFSET),
		    SERVICE_DESC_EXT_FIELD_CONTROL_LEN);

	peer_srvc_entry->sdea_ctrl = le16_to_cpu(peer_srvc_entry->sdea_ctrl);

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "sdea_control[0x%x]\n", peer_srvc_entry->sdea_ctrl);

	if ((peer_srvc_entry->sdea_ctrl >>
		BIT_SDEA_CONTROL_RANGE_LIMIT_PRESENT) & 0x1)
		offset += 4;

	if ((peer_srvc_entry->sdea_ctrl >>
		BIT_SDEA_CONTROL_SRVC_UPDATE_INDICATOR_PRESENT) & 0x1)
	{
		peer_srvc_entry->srvc_update_indicator = *(psdea + offset);
		offset += 1;
	}
	if (attr_len > (offset + 2 - SERVICE_DESC_EXT_FIELD_START_OFFSET)) {
		_os_mem_cpy(d, &srvc_info_len, (psdea + offset), 2);
		srvc_info_len = le16_to_cpu(srvc_info_len);

		if (srvc_info_len) {
			offset += 2;
			if (phl_nan_is_start_with_wfa_oui(d, psdea + offset) &&
			    NAN_SRVC_PROTOCOL_GENIC ==
			    GET_SRVC_INFO_FIELD_PROTO_TYPE(psdea + offset)) {
				/* Service with Service Specific Info */
				_phl_nan_parse_generic_srvc(d,
							    peer_srvc_entry,
							    psdea + offset,
							    srvc_info_len);
			}
			_phl_nan_copy_peer_srvc_info_content(d,
							     peer_srvc_entry,
							     psdea + offset,
							     srvc_info_len);
		}
	}

	ret = true;

unlock_exit:
	phl_nan_done_peer_info_entry(d, nan_info);

	return ret;
}

void
phl_nan_auto_followup_free_mem(struct phl_nan_de_priv *de_info,
			       struct rtw_phl_nan_srvc_instance_entry *srvc_inst,
			       u8 need_dequeue)
{
	struct phl_nan_info *nan_info = phl_nan_get_info(NAN_EN_DISC, de_info);
	void *d = phl_nan_info_to_drvpriv(nan_info);

	if (!phl_nan_get_followup_autoreply_type_from_entry(srvc_inst))
		return;

	if (need_dequeue) {
		/* already locked by _phl_nan_alloc_auto_flup_srv_inst() */
		list_del(&srvc_inst->list);
		de_info->auto_folloup_srvc_queue.cnt--;

		de_info->srvc_num--;
	}

	rtw_phl_nan_free_service_data(d, SCF_TYPE_FOLLOWUP, srvc_inst->u_srv_data);
	rtw_phl_nan_free_service_entry(d, srvc_inst);
}

u16 phl_nan_de_set_default_gen_srv_info(void *d, u8 *srv_info)
{
	u16 gen_srvc_len = 0;
	u8 sub_attri_id = 0;
	u16 sub_attri_len = 0;
	u16 sub_attri_content = 0;
	u16 offset = 0;

	sub_attri_id = NAN_GEN_SRVC_TRANSPORT_PORT;
	sub_attri_len = 2;
	sub_attri_content = 7000; /* Port number */
	gen_srvc_len += sub_attri_len + 3;

	/* Sub-attribute ID */
	_os_mem_cpy(d, srv_info + offset, &sub_attri_id, 1);
	offset += 1;

	/* Sub-attribute len */
	_os_mem_cpy(d, srv_info + offset, &sub_attri_len, 2);
	offset += 2;

	/* Sub-attribute content */
	_os_mem_cpy(d, srv_info + offset, &sub_attri_content, sub_attri_len);
	offset += 2;

	sub_attri_id = NAN_GEN_SRVC_TRANSPORT_PROTOCOL;
	sub_attri_len = 1;
	sub_attri_content = 0x11; /* UDP */
	gen_srvc_len += sub_attri_len + 3;

	/* Sub-attribute ID */
	_os_mem_cpy(d, srv_info + offset, &sub_attri_id, 1);
	offset += 1;

	/* Sub-attribute len */
	_os_mem_cpy(d, srv_info + offset, &sub_attri_len, 2);
	offset += 2;

	/* Sub-attribute content */
	_os_mem_cpy(d, srv_info + offset, &sub_attri_content, sub_attri_len);
	offset += 1;

	phl_nan_debug_dump_data(srv_info, gen_srvc_len, "Generic info ");
	return gen_srvc_len;
}

static u8
_phl_nan_deq_auto_flup_srv_inst(void *d,
				struct phl_nan_de_priv *de_info,
				u64 cookie,
				struct rtw_phl_nan_srvc_instance_entry **srv_inst)
{
	struct phl_queue *auto_flup_q = NULL;
	u64 srvc_cookie = 0;
	u8 ret = false;

	auto_flup_q = &de_info->auto_folloup_srvc_queue;

	_os_spinlock(d, &auto_flup_q->lock, _bh, NULL);
	phl_list_for_loop((*srv_inst),
			  struct rtw_phl_nan_srvc_instance_entry,
			  &auto_flup_q->queue, list) {
		srvc_cookie = phl_nan_get_srv_ck_from_entry((*srv_inst));
		if (srvc_cookie == cookie) {
			list_del(&((*srv_inst)->list));
			auto_flup_q->cnt--;

			de_info->srvc_num--;

			ret = true;
			break;
		}
	}
	_os_spinunlock(d, &auto_flup_q->lock, _bh, NULL);

	return ret;
}

static struct rtw_phl_nan_srvc_instance_entry *
_phl_nan_alloc_auto_flup_srv_inst(struct phl_info_t *phl_info,
				  struct rtw_phl_nan_followup_info *pdata)
{
	struct phl_nan_de_priv *de_info = phl_nan_get_de_info(phl_info);
	struct rtw_phl_nan_srvc_instance_entry *srv_inst = NULL;
	struct phl_queue *auto_flup_q = NULL;
	void *d = phl_info->phl_com->drv_priv;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s ==> \n", __func__);

	auto_flup_q = &(de_info->auto_folloup_srvc_queue);

	/* Allocate new service entry */
	srv_inst = _os_kmem_alloc(d, sizeof(*srv_inst));
	if (!srv_inst) {
		PHL_ERR("%s: alloc p_srvc_entry failed!\n", __func__);
		return srv_inst;
	}
	_os_mem_set(d, (void *)srv_inst, 0, sizeof(*srv_inst));

	srv_inst->service_control = SCF_TYPE_FOLLOWUP | BIT_SCF_SI_PRESENT;
	srv_inst->u_srv_data.f_pdata = pdata;

	/* Insert the service entry to the related service queue */
	_os_spinlock(d, &auto_flup_q->lock, _ps, NULL);
	list_add(&srv_inst->list, &auto_flup_q->queue);
	auto_flup_q->cnt++;

	de_info->srvc_num++;

	_phl_nan_indicate_issue_follow_up_sdf(d, de_info, srv_inst);
	_os_spinunlock(d, &auto_flup_q->lock, _ps, NULL);

	return srv_inst;
}

void
phl_nan_free_all_auto_flup_srv_inst(struct phl_info_t *phl_info)
{
	struct phl_nan_de_priv *de_info = phl_nan_get_de_info(phl_info);
	struct phl_queue *auto_flup_q = NULL;

	auto_flup_q = &de_info->auto_folloup_srvc_queue;

	phl_nan_free_srvc_inst_q(phl_info, de_info, auto_flup_q);
}

/**
 * _phl_nan_auto_followup - Reply specific followup if as a publisher
 * @pmatch_sda: sda of sdf with match service id
 * @srvc_inst: own service inst entry with same service id
 * @peer_addr: peer nmi
 * (ac) _rtw_nan_auto_followup
 *
 * RETURN:
 * true: done.
 * false: fail to issue.
 */
static u8
_phl_nan_auto_followup(struct phl_nan_de_priv *de_info,
		       u8 *pmatch_sda,
		       struct rtw_phl_nan_srvc_instance_entry *srvc_inst,
		       u8 *peer_addr)
{
	struct phl_nan_info *nan_info = phl_nan_get_info(NAN_EN_DISC, de_info);
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_peer_info_entry *peer_info = NULL;
	struct phl_nan_peer_srvc_info_entry *peer_srvc_info = NULL;
	struct rtw_phl_nan_followup_info *f_data = NULL;
	struct rtw_phl_nan_service_info r_srv_spec_info = {0};
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry = NULL;
	u8 b_peer_exist = false;
	u8 b_peer_srvc_exist = false;
	u8 srvc_ctrl = *(pmatch_sda + SDA_SERVICE_CONTROL_OFFSET);
	u8 reply_type = 0; /* 1: mac, 2: ipv6, 3: talk */

	if (!phl_nan_is_scf_type_followup(srvc_ctrl) || !(srvc_ctrl & BIT_SCF_SI_PRESENT))
		return false;

	b_peer_exist = phl_nan_is_peer_info_exist(nan_info, peer_addr,
						  &peer_info);
	b_peer_srvc_exist = (b_peer_exist)?
			    (phl_nan_peer_srvc_info_exist(
					d,
					(void *)peer_info,
					(pmatch_sda + NAN_ATTR_CONTENT_OFFSET),
					GET_PEER_SRVC_INFO_BY_SRVC_ID,
					&peer_srvc_info)):
			    (false);
	if (b_peer_exist == true)
		phl_nan_done_peer_info_entry(d, nan_info);

	if (!phl_nan_is_pub_srvc_inst(srvc_inst) && (false == b_peer_srvc_exist)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s, followup recv without corresponding peer_srvc_info_entry\n",
			  __func__);
		return false;
	}

	f_data = _os_kmem_alloc(d, sizeof(struct rtw_phl_nan_followup_info));
	if (!f_data) {
		PHL_WARN("%s: allocate f_data failed\n", __func__);
		goto free_followup_data;
	}

	/* todo: not consider other filter length */
	/* Check: servive info exist */
	r_srv_spec_info.len = *(pmatch_sda + SDA_OPTIONAL_FIELD_OFFSET);
	r_srv_spec_info.info = _os_kmem_alloc(d, r_srv_spec_info.len);
	if (!r_srv_spec_info.info) {
		PHL_WARN("%s: allocate r_srv_spec_info.info failed\n", __func__);
		goto free_followup_data;
	}
	_os_mem_cpy(d, (void *)r_srv_spec_info.info,
		    (pmatch_sda + SDA_OPTIONAL_FIELD_OFFSET + 1),
		    r_srv_spec_info.len);
	if (!r_srv_spec_info.len || NULL == r_srv_spec_info.info)
		goto free_followup_data;

	/* Check: auto reply or not, support format: (1) mac addr (2) ipv6 */
#ifdef CONFIG_PHL_NAN_DEBUG
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s, dump incoming followup frame\n", __func__);
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "recv followup serv_spec_info_len = %d\n", r_srv_spec_info.len);
	debug_dump_data(r_srv_spec_info.info,
			r_srv_spec_info.len,
			"recv followup serv_spec_info:\n");
#endif /* CONFIG_PHL_NAN_DEBUG */

	if (0 == _os_mem_cmp(d, peer_addr, r_srv_spec_info.info, MAC_ALEN)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s, TYPE_NAN_AUTO_FOLLOWUP_MAC_ADDR\n", __func__);
		f_data->srv_spec_info.len = r_srv_spec_info.len;
		f_data->srv_spec_info.info =_os_kmem_alloc(d, f_data->srv_spec_info.len);
		if (!f_data->srv_spec_info.info) {
			PHL_WARN("%s: allocate f_data->srv_spec_info.info failed\n", __func__);
			goto free_followup_data;
		}
		_os_mem_cpy(d,
			    (void *)f_data->srv_spec_info.info,
			    nan_info->sync_engine_priv.mac_info.nmi.mac,
			    MAC_ALEN);
		reply_type = TYPE_NAN_AUTO_FOLLOWUP_MAC_ADDR;
	} else if (r_srv_spec_info.info[0] == 0xfe && r_srv_spec_info.info[1] == 0x80) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s, TYPE_NAN_AUTO_FOLLOWUP_IPV6 \n", __func__);
		/* TODO: need to revise
		 *	for publisher, the ipv6 should be from the publish info.
		 *	for subscriber, the ipv6 does not determine at this point. */
		f_data->srv_spec_info.len = r_srv_spec_info.len;
		f_data->srv_spec_info.info = _os_kmem_alloc(d, f_data->srv_spec_info.len);
		if (!f_data->srv_spec_info.info) {
			PHL_WARN("%s: allocate f_data->srv_spec_info.info failed\n", __func__);
			goto free_followup_data;
		}
#ifdef CONFIG_NAN_R3_MERGE_PROCESS
		/* (TODO) */
		rtw_phl_nan_de_set_default_ipv6(
			d, need IPV6 from core,
			(void *)f_data->srv_spec_info.info);
#endif
		reply_type = TYPE_NAN_AUTO_FOLLOWUP_IPV6;
	} else if (0 == _os_mem_cmp(d, r_srv_spec_info.info, "Let's talk", r_srv_spec_info.len)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s, TYPE_NAN_AUTO_FOLLOWUP_CVer \n", __func__);
		f_data->srv_spec_info.len = 5;
		f_data->srv_spec_info.info = _os_kmem_alloc(d, f_data->srv_spec_info.len);
		if (!f_data->srv_spec_info.info) {
			PHL_WARN("%s: allocate f_data->srv_spec_info.info failed\n", __func__);
			goto free_followup_data;
		}
		_os_mem_cpy(d,
			    (void *)f_data->srv_spec_info.info,
			    (unsigned char *)"Ready",
			    5);
		reply_type = TYPE_NAN_AUTO_FOLLOWUP_TALK;
	} else {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s, not support case. (%lu)\n",
			  __func__, sizeof("Let's talk"));
		phl_nan_debug_dump_data(r_srv_spec_info.info,
					r_srv_spec_info.len,
					"serv_spec_info:");
		goto free_followup_data;
	}

	_phl_nan_free_srv_info(d, r_srv_spec_info);

	/* fill other info for issue reply followup */
	f_data->followup_id = *(pmatch_sda + SDA_REQUESTOR_INSTANCE_ID_OFFSET);
	f_data->followup_reqid = *(pmatch_sda + SDA_INSTANCE_ID_OFFSET);
	f_data->service_auto_followup_type = reply_type;
	if (de_info->auto_followup_cookie == 0)
		de_info->auto_followup_cookie++;
	f_data->cookie = de_info->auto_followup_cookie++;
	_os_mem_cpy(d, f_data->followup_dest, peer_addr, MAC_ALEN);
	_os_mem_cpy(d,
		    f_data->matched_service_id,
		    (pmatch_sda + SDA_SERVICE_ID_OFFSET),
		    NAN_SRVC_ID_LEN);

#ifdef CONFIG_PHL_NAN_DEBUG
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s, dump reply followup frame\n", __func__);
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "reply followup_id = %d\n", f_data->followup_id);
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "reply followup_reqid = %d\n",
		  f_data->followup_reqid);
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "reply auto_followup_cookie = %llu\n",
		  f_data->cookie);
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "reply matched instance id = %d\n",
		  f_data->matched_instance_id);
	debug_dump_data(f_data->followup_dest, MAC_ALEN,
			"reply matched dest:");
	debug_dump_data(f_data->matched_service_id,
			NAN_SRVC_ID_LEN,
			"reply matched service_id:");
	debug_dump_data(f_data->srv_spec_info.info,
			f_data->srv_spec_info.len,
			"reply followup serv_spec_info:");
#endif /* CONFIG_PHL_NAN_DEBUG */

	p_srvc_entry = _phl_nan_alloc_auto_flup_srv_inst(nan_info->phl_info,
							 f_data);
	if (!p_srvc_entry)
		goto free_followup_data;

	return true;

free_followup_data:
	_phl_nan_free_srv_info(d, r_srv_spec_info);
	_phl_nan_free_srv_info(d, f_data->srv_spec_info);
	if (f_data)
		_os_kmem_free(d, f_data, sizeof(struct rtw_phl_nan_followup_info));

	return false;
}

u8
phl_nan_add_peer_srvc_check(struct phl_nan_de_priv *de_info,
			    u8 *pmatch_sda,
			    struct rtw_phl_nan_srvc_instance_entry *srvc_inst,
			    u8 *ppeer_addr,
			    u8 *frame,
			    u16 frame_len)
{
	struct phl_nan_info *nan_info = phl_nan_get_info(NAN_EN_DISC, de_info);
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_peer_info_entry *peer_info = NULL;
	struct phl_nan_peer_srvc_info_entry *peer_srvc_info = NULL;
	u8 b_peer_exist = false;
	u8 b_peer_srvc_info_exist = false;
	u8 srvc_ctrl = *(pmatch_sda + SDA_SERVICE_CONTROL_OFFSET);
	u32 offset = 0;
	u32 filter_len = 0;
	u16 total_len = 0;
	u8 *attr_start = NULL;
	u8 ret = false;

	if (RTW_PHL_STATUS_SUCCESS !=
	    nan_info->nan_ops.get_attr_start(frame, frame_len, &attr_start, &total_len)) {
		PHL_WARN("%s: parsing frame fail\n", __func__);
		goto exit;
	}

	b_peer_exist = phl_nan_is_peer_info_exist(nan_info, ppeer_addr, &peer_info);
	b_peer_srvc_info_exist =
		(b_peer_exist) ? (phl_nan_peer_srvc_info_exist(
					 d, (void *)peer_info,
					 (pmatch_sda + NAN_ATTR_CONTENT_OFFSET),
					 GET_PEER_SRVC_INFO_BY_SRVC_ID,
					 &peer_srvc_info)) :
				 (false);

	if (!phl_nan_is_pub_srvc_inst(srvc_inst) &&
	    phl_nan_is_scf_type_followup(srvc_ctrl) &&
	    false == b_peer_srvc_info_exist) {
		PHL_WARN("%s: followup recv without corresponding peer_srvc_info_entry\n",
			  __func__);
		goto exit;
	}

	/* Macthing match filter or SRF in Service Descriptor Attribute if any */
	if (true == _phl_nan_de_chk_trigger_cond(d,
						 phl_nan_get_nmi(nan_info),
						 srvc_inst,
						 pmatch_sda,
						 &filter_len)) {
		if (b_peer_exist == false &&
		    false == phl_nan_alloc_peer_info_entry(nan_info, &peer_info,
							   ppeer_addr)) {
			goto exit;
		} else {
			b_peer_exist = true;
		}

		if (b_peer_srvc_info_exist == false &&
		    false == phl_nan_alloc_peer_srvc_entry(d,
							   &peer_info->srvc_info_queue,
							   &peer_srvc_info)) {
			goto exit;
		}

		peer_srvc_info->assoc_srvc_inst_ck = phl_nan_get_srv_ck_from_entry(srvc_inst);
		_os_mem_cpy(d, peer_srvc_info->service_id,
			    pmatch_sda + SDA_SERVICE_ID_OFFSET,
			    SDA_SERVICE_ID_LEN);
		_os_mem_cpy(d, &peer_srvc_info->peer_inst_id,
			    pmatch_sda + SDA_INSTANCE_ID_OFFSET,
			    SDA_INSTANCE_ID_LEN);
		peer_srvc_info->srvc_ctrl = srvc_ctrl;
		SET_STATUS_FLAG(peer_srvc_info->flag, PEER_SRVC_INFO_FLAG_VALID);
		if (srvc_ctrl & BIT_SCF_SI_PRESENT) {
			offset = (SDA_OPTIONAL_FIELD_OFFSET + filter_len);
			if (srvc_ctrl & BIT_SCF_BB_PRESENT)
				offset += SDA_BINDING_BITMAP_LEN;

			_phl_nan_copy_peer_srvc_info_content(d,
							     peer_srvc_info,
							     pmatch_sda + offset + 1,
							     (u16)*(pmatch_sda + offset));
		}

		/* cfgvendor_merge : get rssi from pkt and store it in peer_srvc_info */

		if ((srvc_ctrl & SCF_TYPE_MASK) == SCF_TYPE_FOLLOWUP) {
			/* rtw_cfgvendor_nan_event_hdr_follow_up_received(
				peer_info,
				peer_srvc_info); */
		} else if ((srvc_ctrl & SCF_TYPE_MASK) == SCF_TYPE_PUBLISH) {
			/* rtw_cfgvendor_nan_event_hdr_discovery_result_received(
				srvc_inst,
				peer_info,
				peer_srvc_info); */
		} else if (((srvc_ctrl & SCF_TYPE_MASK) == SCF_TYPE_SUBSCRIBE) &&
			   phl_nan_is_solicited_publish_srvc_instance(srvc_inst)) {
			/* send solicited publish SDF back */
			_phl_nan_indicate_issue_solicited_publish_sdf(
				de_info, peer_info, peer_srvc_info);
		}

		_phl_nan_indicate_report_match_event(de_info,
						     peer_info,
						     peer_srvc_info);

		ret = true;
	} else if (b_peer_srvc_info_exist) {
		if (peer_srvc_info->ref_cnt < 1) {
			_os_spinlock(d, &peer_info->srvc_info_queue.lock, _bh, NULL);
			list_del(&peer_srvc_info->list);
			_phl_nan_free_peer_srvc_entry_data(d, peer_srvc_info);
			_os_kmem_free(d, peer_srvc_info, sizeof(struct phl_nan_peer_srvc_info_entry));
			_os_spinunlock(d, &peer_info->srvc_info_queue.lock, _bh,NULL);
		}
	}

exit:
	if (b_peer_exist)
		phl_nan_done_peer_info_entry(d, nan_info);

	return ret;
}

u8
phl_nan_find_srvc_inst_entry(void *d,
			     struct phl_nan_de_priv *de_info,
			     void *pdata,
			     u8 stype,
			     struct rtw_phl_nan_srvc_instance_entry **srv_inst)
{
	struct phl_queue *dedicated_q = NULL;
	u8 *srvc_id = NULL;
	u8 srvc_inst_id = 0;
	u64 srvc_cookie = 0;
	u8 b_found = false;
	u8 i = 0;

	/* Find exist service instance from publish service queue first */
	dedicated_q = &de_info->publish_srvc_queue;
	for (i = 0; i < MAX_SRVC_INSTANCE_TYPE && (!b_found); i++, dedicated_q++) {
		_os_spinlock(d, &dedicated_q->lock, _bh, NULL);
		phl_list_for_loop((*srv_inst),
				  struct rtw_phl_nan_srvc_instance_entry,
				  &dedicated_q->queue, list) {
			srvc_id = phl_nan_get_srv_id_from_entry((*srv_inst));
			srvc_cookie = phl_nan_get_srv_ck_from_entry((*srv_inst));
			srvc_inst_id = phl_nan_get_inid_from_entry((*srv_inst));

			/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				     "%s: GET_SRVC_BY Type(0x%x) => Run(inst_id(%u), cookie(%llu)).\n",
				     __func__, stype, srvc_inst_id,
				     srvc_cookie); */

			if ((TEST_STATUS_FLAG(stype, GET_SRVC_BY_COOKIE) &&
			     srvc_cookie == *((u64 *)pdata)) ||
			    /* by cookie */
			    (TEST_STATUS_FLAG(stype, GET_SRVC_BY_SRVC_ID) &&
			     0 == _os_mem_cmp(d, srvc_id, (u8 *)pdata, NAN_SRVC_ID_LEN))||
			    /* by service id */
			    (TEST_STATUS_FLAG(stype, GET_SRVC_BY_INST_ID) &&
			     srvc_inst_id == *((u8 *)pdata)) ||
			    /* by instance id */
			    (TEST_STATUS_FLAG(stype, GET_SRVC_BY_PUBLISH_INST_ID) &&
			     phl_nan_is_pub_srvc_inst(*srv_inst) &&
			     srvc_inst_id == *((u8 *)pdata)) ||
			    /* by publish service id */
			    (TEST_STATUS_FLAG(stype,GET_SRVC_BY_FIRST_SRVC_ID)))
			    /* first service; design for wfa logo test */
			{
				b_found = true;
				PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
					  "%s: GET_SRVC_BY Type(0x%x) => Run(inst_id(%u), cookie(%llu)). found !! \n",
					  __func__, stype, srvc_inst_id, srvc_cookie);
				break;
			}
		}

		if (b_found == false)
			_os_spinunlock(d, &dedicated_q->lock, _bh, NULL);
	}

	return b_found;
}

void
phl_nan_done_srvc_inst_entry(void *d,
			     struct phl_nan_de_priv *de_info,
			     struct rtw_phl_nan_srvc_instance_entry *srv_inst)
{
	struct phl_queue *dedicated_q = NULL;
	u8 type;

	type = srv_inst->service_control & SCF_TYPE_MASK;
	switch (type) {
	case SCF_TYPE_PUBLISH:
		dedicated_q = &(de_info->publish_srvc_queue);
		break;
	case SCF_TYPE_SUBSCRIBE:
		dedicated_q = &(de_info->subscribe_srvc_queue);
		break;
	case SCF_TYPE_FOLLOWUP:
		dedicated_q = &(de_info->followup_srvc_queue);
		break;
	default:
		PHL_WARN("%s: srvc type unkown(%x)\n", __func__, type);
		break;
	}

	if (!dedicated_q)
		return;

	_os_spinunlock(d, &dedicated_q->lock, _bh, NULL);
}

/*
 * _phl_nan_parsing_sdf_attr - parse all IEs in the received sdf
 * @de_info: discovery engine priv
 * @r_frame: received sdf frame
 * @r_frame_len: rx pk len
 * (ac) rtw_nan_parsing_nan_sdf_attr
 */
u8
_phl_nan_parsing_sdf_attr(struct phl_nan_de_priv *de_info, u8 *r_frame, u16 r_frame_len)
{
	struct phl_nan_info *nan_info = phl_nan_get_info(NAN_EN_DISC, de_info);
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct rtw_phl_nan_srvc_instance_entry *p_srvc = NULL;
	u8 *pnan_attr_start = NULL;
	u8 peer_nmi[MAC_ALEN] = {0};
	u16 attr_len = 0;
	u8 ret = true;
	u8 num_match_srvc = 0, num_match_followup = 0;

	if (RTW_PHL_STATUS_FAILURE ==
	    nan_info->nan_ops.get_attr_start(r_frame, r_frame_len, &pnan_attr_start, NULL))
		return false;

	_os_mem_cpy(d, peer_nmi, r_frame + 10, MAC_ALEN);
	while ((pnan_attr_start + NAN_ATTR_CONTENT_OFFSET) < (r_frame + r_frame_len)) {
		_os_mem_cpy(d, (u8 *)&attr_len, pnan_attr_start + 1, 2);
		attr_len = le16_to_cpu(attr_len);
		if (!attr_len) {
			ret = false;
			break;
		}

		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s Attribute ID = 0x%x\n",
			  __func__, pnan_attr_start[0]);

		switch (pnan_attr_start[0]) {
		case NAN_ATTR_ID_SERVICE_DESCRIPTOR:
			/* Matching service ID in Service Descriptor Attribute */

			if (false == phl_nan_find_srvc_inst_entry(
					d, de_info,
					pnan_attr_start + SDA_SERVICE_ID_OFFSET,
					GET_SRVC_BY_SRVC_ID,
					&p_srvc)) {
				break;
			}

			/* Rx publish or subscribe sdf,
			   check with filter function to decide
			   if add peer or not*/
			if (true == phl_nan_add_peer_srvc_check(de_info,
								pnan_attr_start,
								p_srvc,
								peer_nmi,
								r_frame,
								r_frame_len))
				num_match_srvc++;
			/* Rx followup sdf, check auto reply or not */
			if (true == _phl_nan_auto_followup(de_info,
							   pnan_attr_start,
							   p_srvc,
							   peer_nmi))
				num_match_followup++;

			phl_nan_done_srvc_inst_entry(d, de_info, p_srvc);

			break;
		case NAN_ATTR_ID_SRV_DES_EXT:
			if (_phl_nan_sdf_sdea_parsing(nan_info, pnan_attr_start,
						      peer_nmi, attr_len)
						      == false)
				PHL_WARN("Wrong SDEA Info!\n");
			break;
		case NAN_ATTR_ID_DEVICE_CAPABILITY:
			if (_phl_nan_sdf_device_cap_parsing(nan_info,
							    pnan_attr_start,
							    peer_nmi) == false)
				PHL_WARN("Wrong Device Cap Info!\n");
			break;
		default:
			break;
		}

		pnan_attr_start += (attr_len + NAN_ATTR_CONTENT_OFFSET);
	}

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: Match %d service, %d followup\n",
		  __func__, num_match_srvc, num_match_followup);

	return ret;
}

/*
 * phl_nan_check_srv_type - Check service entry is specific type or not
 * @p_srvc_entry: local service function entry
 * @type: enum _nan_srvc_type
 * (ac) rtw_nan_check_service_type
 *
 * RETURN:
 * true: Match
 * false: Not match
 */
u8
phl_nan_check_srv_type(struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry,
		       u8 type)
{
	u8 ret = false;
	switch (type) {
	case NAN_SRVC_TYPE_UNSOLICITED_PUBLISH:
		if (phl_nan_is_unsolicited_publish_srvc_instance(p_srvc_entry))
			ret = true;
		if (ret)
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "%s : UNSOLICITED_PUBLISH\n", __func__);
		break;
	case NAN_SRVC_TYPE_SOLICITED_PUBLISH:
		if (phl_nan_is_solicited_publish_srvc_instance(p_srvc_entry))
			ret = true;
		if (ret)
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "%s : SOLICITED_PUBLISH\n", __func__);
		break;
	case NAN_SRVC_TYPE_ACTIVE_SUBSCRIBE:
		if (phl_nan_is_active_subscribe_srvc_instance(p_srvc_entry))
			ret = true;
		if (ret)
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "%s : ACTIVE_SUBSCRIBE\n", __func__);
		break;
	case NAN_SRVC_TYPE_PASSIVE_SUBSCRIBE:
		if (!phl_nan_is_active_subscribe_srvc_instance(p_srvc_entry))
			ret = true;
		if (ret)
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "%s : PASSIVE_SUBSCRIBE\n", __func__);
		break;
	case NAN_SRVC_TYPE_FOLLOW_UP:
		if (phl_nan_is_followup_srvc_inst(p_srvc_entry))
			ret = true;
		if (ret)
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "%s : FOLLOW_UP\n", __func__);
		break;
	default:
		break;
	}
	return ret;
}

/* alloc struct rtw_phl_nan_srvc_instance_entry */
/**
 * _phl_nan_alloc_publish_srv_inst -
 *  Allocate NAN discovery engine service function instance for publish service
 * @phl_info: phl_info
 * @pdata: a pointer of publish service data
 * (ac) _rtw_alloc_srvc_instance_data_publish
 *
 * RETURN:
 * a pointer of struct rtw_phl_nan_srvc_instance_entry: allocate successfully
 * NULL: fail to allocate
 */
static struct rtw_phl_nan_srvc_instance_entry *
_phl_nan_alloc_publish_srv_inst(struct phl_info_t *phl_info,
				struct rtw_phl_nan_publish_info *pdata)
{
	struct phl_nan_de_priv *de_info = phl_nan_get_de_info(phl_info);
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry = NULL;
	struct phl_queue *p_dedicated_queue = NULL;
	void *d = phl_info->phl_com->drv_priv;
	u8 committed_dw_period = 0;
	u8 srvc_ctrl = 0;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s ==> \n", __func__);

	/* temp use 2.4g committed dw period */
	phl_nan_get_committed_dw_period(de_info, committed_dw_period);

	srvc_ctrl |= SCF_TYPE_PUBLISH;
	p_dedicated_queue = &(de_info->publish_srvc_queue);

	if (!p_dedicated_queue)
		return p_srvc_entry;

	if (pdata->num_tx_filters != 0)
		srvc_ctrl |= BIT_SCF_MF_PRESENT;
	if (pdata->srv_spec_info.len != 0)
		srvc_ctrl |= BIT_SCF_SI_PRESENT;

	/* Allocate new service entry */
	p_srvc_entry = _os_kmem_alloc(d, sizeof(*p_srvc_entry));
	if (!p_srvc_entry) {
		PHL_ERR("%s: alloc p_srvc_entry failed!\n", __func__);
		return p_srvc_entry;
	}
	_os_mem_set(d, (void *)p_srvc_entry, 0, sizeof(*p_srvc_entry));
	p_srvc_entry->service_control = srvc_ctrl;
	p_srvc_entry->u_srv_data.p_pdata = pdata;

	if (!pdata->publish_id) {
		pdata->publish_id = de_info->instant_id;
		de_info->instant_id++;
	} else {
		de_info->instant_id = pdata->publish_id;
	}

	if (de_info->srvc_ext_info.updating)
		phl_nan_update_service_ext_info_from_de(d, de_info,
							p_srvc_entry);

	/* ToDo: awake_dw_int and period is not referenced.
	 * should check before trx sdf and bcast sdf. */
	pdata->awake_dw_int = (pdata->awake_dw_int) ?
			      (pdata->awake_dw_int) :
			      (committed_dw_period);
	pdata->period = (pdata->period < pdata->awake_dw_int) ?
			(pdata->awake_dw_int) :
			(pdata->period);

	_phl_nan_update_awake_dw_interval(de_info, pdata->awake_dw_int);

	/* Insert the service entry to the related service queue */
	_os_spinlock(d, &p_dedicated_queue->lock, _bh, NULL);
	list_add(&p_srvc_entry->list, &p_dedicated_queue->queue);
	p_dedicated_queue->cnt++;

	de_info->srvc_num++;
	_os_spinunlock(d, &p_dedicated_queue->lock, _bh, NULL);

	return p_srvc_entry;
}

/**
 * _phl_nan_alloc_subscribe_srv_inst -
 *  Allocate NAN discovery engine service function instance for subscribe service
 * @phl_info: phl_info
 * @pdata: a pointer of subscribe service data
 * (ac) _rtw_alloc_srvc_instance_data_subscribe
 *
 * RETURN:
 * a pointer of struct rtw_phl_nan_srvc_instance_entry: allocate successfully
 * NULL: fail to allocate
 */
static struct rtw_phl_nan_srvc_instance_entry *
_phl_nan_alloc_subscribe_srv_inst(struct phl_info_t *phl_info,
				  struct rtw_phl_nan_subscribe_info *pdata)
{
	struct phl_nan_de_priv *de_info = phl_nan_get_de_info(phl_info);
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry = NULL;
	struct phl_queue *p_dedicated_queue = NULL;
	void *d = phl_info->phl_com->drv_priv;
	u8 committed_dw_period = 0;
	u8 srvc_ctrl = 0;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s ==> \n", __func__);

	/* temp use 2.4g committed dw period */
	phl_nan_get_committed_dw_period(de_info, committed_dw_period);
	srvc_ctrl |= SCF_TYPE_SUBSCRIBE;
	p_dedicated_queue = &(de_info->subscribe_srvc_queue);

	if (!p_dedicated_queue)
		return p_srvc_entry;

	if (pdata->num_tx_filters != 0)
		srvc_ctrl |= BIT_SCF_MF_PRESENT;
	if (pdata->srf_bf_len != 0 || pdata->srf_num_macs != 0)
		srvc_ctrl |= BIT_SCF_SRF_PRESENT;
	if (pdata->srv_spec_info.len != 0)
		srvc_ctrl |= BIT_SCF_SI_PRESENT;

	if (!pdata->subscribe_id) {
		pdata->subscribe_id = de_info->instant_id;
		de_info->instant_id++;
	} else {
		de_info->instant_id = pdata->subscribe_id;
	}

	/* Allocate new service entry */
	p_srvc_entry = _os_kmem_alloc(d, sizeof(*p_srvc_entry));
	if (!p_srvc_entry) {
		PHL_ERR("%s: alloc p_srvc_entry failed!\n", __func__);
		return p_srvc_entry;
	}
	_os_mem_set(d, (void *)p_srvc_entry, 0, sizeof(*p_srvc_entry));

	p_srvc_entry->service_control = srvc_ctrl;
	p_srvc_entry->u_srv_data.s_pdata = pdata;

	if (de_info->srvc_ext_info.updating)
		phl_nan_update_service_ext_info_from_de(d, de_info, p_srvc_entry);

	/* ToDo: awake_dw_int and period is not referenced. should check before trx sdf and bcast sdf. */
	pdata->awake_dw_int = (pdata->awake_dw_int) ? (pdata->awake_dw_int) :
						      (committed_dw_period);
	pdata->period = (pdata->period < pdata->awake_dw_int) ?
			(pdata->awake_dw_int) : (pdata->period);
	_phl_nan_update_awake_dw_interval(de_info, pdata->awake_dw_int);

	/* Insert the service entry to the related service queue */
	_os_spinlock(d, &p_dedicated_queue->lock, _bh, NULL);
	list_add(&p_srvc_entry->list, &p_dedicated_queue->queue);
	p_dedicated_queue->cnt++;

	de_info->srvc_num++;
	_os_spinunlock(d, &p_dedicated_queue->lock, _bh, NULL);

	return p_srvc_entry;
}

/**
 * _phl_nan_alloc_followup_srv_inst -
 *   Allocate NAN discovery engine service function instance for followup service
 * @phl_info: phl_info
 * @pdata: a pointer of followup service data
 * (ac) _rtw_alloc_srvc_instance_data_followup
 *
 * RETURN:
 * a pointer of struct rtw_phl_nan_srvc_instance_entry: allocate successfully
 * NULL: fail to allocate
 */
static struct rtw_phl_nan_srvc_instance_entry *
_phl_nan_alloc_followup_srv_inst(struct phl_info_t *phl_info,
				 struct rtw_phl_nan_followup_info *pdata)
{
	struct phl_nan_de_priv *de_info = phl_nan_get_de_info(phl_info);
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry = NULL;
	struct phl_queue *p_dedicated_queue = NULL;
	void *d = phl_info->phl_com->drv_priv;
	u8 srvc_ctrl = 0;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s ==> \n", __func__);

	srvc_ctrl |= SCF_TYPE_FOLLOWUP;
	if (pdata->srv_spec_info.len)
		srvc_ctrl |= BIT_SCF_SI_PRESENT;

	p_dedicated_queue = &(de_info->followup_srvc_queue);

	if (!p_dedicated_queue)
		return p_srvc_entry;

	if (!pdata->followup_id) {
		pdata->followup_id = de_info->instant_id;
		de_info->instant_id++;
	} else {
		de_info->instant_id = pdata->followup_id;
	}

	/* Allocate new service entry */
	p_srvc_entry = _os_kmem_alloc(d, sizeof(*p_srvc_entry));
	if (!p_srvc_entry) {
		PHL_ERR("%s: alloc p_srvc_entry failed!\n", __func__);
		return p_srvc_entry;
	}
	_os_mem_set(d, (void *)p_srvc_entry, 0, sizeof(*p_srvc_entry));

	p_srvc_entry->service_control = srvc_ctrl;
	p_srvc_entry->u_srv_data.f_pdata = pdata;

	/* Insert the service entry to the related service queue */
	_os_spinlock(d, &p_dedicated_queue->lock, _bh, NULL);
	list_add(&p_srvc_entry->list, &p_dedicated_queue->queue);
	p_dedicated_queue->cnt++;

	de_info->srvc_num++;

	_phl_nan_indicate_issue_follow_up_sdf(d, de_info, p_srvc_entry);
	_os_spinunlock(d, &p_dedicated_queue->lock, _bh, NULL);

	return p_srvc_entry;
}

/**
 * _phl_nan_dump_service
 * @p_srvc_entry: the service which is going to dump info
 * (ac) dump_rtw_nan_service
 */
static void
_phl_nan_dump_service(struct rtw_phl_nan_srvc_instance_entry *srvc_entry)
{
	struct rtw_phl_nan_publish_info *pub_data = NULL;
	struct rtw_phl_nan_subscribe_info *sub_data = NULL;
	struct rtw_phl_nan_followup_info *flwp_data = NULL;
	struct rtw_phl_nan_service_info *p_si = NULL;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "=======================\n");
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "dump_rtw_nan_service:\n");

	switch (srvc_entry->service_control & SCF_TYPE_MASK) {
	case SCF_TYPE_PUBLISH:
		pub_data = srvc_entry->u_srv_data.p_pdata;
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "type: publish\n");
		if (phl_nan_is_solicited_publish_info(pub_data)) {
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "publish_type: solicited\n");
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "publish_bcast: %s\n",
				  ((TEST_STATUS_FLAG(pub_data->solicited_tx_type, \
						     NAN_SOLICITED_TX_BROADCAST)) ?
				   "true" : "false"));
		}

		if (phl_nan_is_unsolicited_publish_info(pub_data)) {
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "publish_type: unsolicited\n");
		}

		if (TEST_STATUS_FLAG(pub_data->sde_control, NAN_SDE_CTRL_SEC_REQUIRED)){
			phl_nan_debug_dump_data(pub_data->sec_req.pmk, MAX_PMK_LEN, "pmk: ");
		}
		phl_nan_debug_dump_data(pub_data->ipv6_info, MAX_IPV6_INFO_LEN, "ipv6: ");
		break;
	case SCF_TYPE_SUBSCRIBE:
		sub_data = srvc_entry->u_srv_data.s_pdata;
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "type: subscribe\n");
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "subscribe_active: %s\n",
			  (phl_nan_is_active_subscribe_info(sub_data) ?
			  "true" : "false"));
		break;
	case SCF_TYPE_FOLLOWUP:
		flwp_data = srvc_entry->u_srv_data.f_pdata;
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "type: follow up\n");
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "followup_id: %d\n",
			  flwp_data->followup_id);
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "followup_reqid: %d\n",
			  flwp_data->followup_reqid);
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "followup_dest: %x:%x:%x:%x:%x:%x\n",
			  flwp_data->followup_dest[0],
			  flwp_data->followup_dest[1],
			  flwp_data->followup_dest[2],
			  flwp_data->followup_dest[3],
			  flwp_data->followup_dest[4],
			  flwp_data->followup_dest[5]);
		break;
	default:
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "type: unknown\n");
	}

	phl_nan_debug_dump_data(phl_nan_get_srv_id_from_entry(srvc_entry),
				NAN_SRVC_ID_LEN, "service_id: ");

	if (pub_data || sub_data) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "ttl: %d\n",
			  phl_nan_get_ttl_from_entry(srvc_entry));
	}
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "instance_id: %d\n",
		  phl_nan_get_inid_from_entry(srvc_entry));
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "cookie: %llu\n",
		  phl_nan_get_srv_ck_from_entry(srvc_entry));

	p_si = phl_nan_get_si_from_entry(srvc_entry);
	if (p_si)
		phl_nan_debug_dump_data(p_si->info, p_si->len, "serv_spec_info: ");

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "=======================\n");
}

static u8
_nan_is_srv_inst_ck_valid(struct phl_nan_add_srv_parm *nan_add_srv_parm)
{
	u64 cookie = 0;

	switch (nan_add_srv_parm->srv_type & SCF_TYPE_MASK) {
	case SCF_TYPE_PUBLISH:
		cookie = nan_add_srv_parm->srv_data.p_pdata->cookie;
		break;
	case SCF_TYPE_SUBSCRIBE:
		cookie = nan_add_srv_parm->srv_data.s_pdata->cookie;
		break;
	case SCF_TYPE_FOLLOWUP:
		cookie = nan_add_srv_parm->srv_data.f_pdata->cookie;
		break;
	}

	if (!cookie)
		PHL_WARN("srv_inst's cookie cannot be 0\n");

	return cookie ? true : false;
}

enum rtw_phl_status
phl_add_nan_func_evt_hdl(struct phl_info_t *phl_info, void *pbuf)
{
	struct phl_nan_event_parm *p_parm = (struct phl_nan_event_parm *)pbuf;
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry = NULL;
	struct phl_nan_add_srv_parm *nan_add_srv_parm = NULL;
	union rtw_phl_nan_srv_data *p_srv_data = NULL;
	u8 *p_instance_id = NULL;
	enum rtw_phl_status psts = RTW_PHL_STATUS_SUCCESS;

	nan_add_srv_parm = (struct phl_nan_add_srv_parm *)p_parm->data;
	p_instance_id = nan_add_srv_parm->p_instance_id;
	p_srv_data = &nan_add_srv_parm->srv_data;

	if (false == _nan_is_srv_inst_ck_valid(nan_add_srv_parm)) {
		psts = RTW_PHL_STATUS_FAILURE;
		goto exit;
	}

	switch (nan_add_srv_parm->srv_type & SCF_TYPE_MASK) {
	case SCF_TYPE_PUBLISH:
		p_srvc_entry = _phl_nan_alloc_publish_srv_inst(phl_info,
							       p_srv_data->p_pdata);
		break;
	case SCF_TYPE_SUBSCRIBE:
		p_srvc_entry = _phl_nan_alloc_subscribe_srv_inst(phl_info,
								 p_srv_data->s_pdata);
		break;
	case SCF_TYPE_FOLLOWUP:
		p_srvc_entry = _phl_nan_alloc_followup_srv_inst(phl_info,
								p_srv_data->f_pdata);
		break;
	}

	if (!p_srvc_entry) {
		psts = RTW_PHL_STATUS_FAILURE;
		goto exit;
	}

	*p_instance_id = phl_nan_get_inid_from_entry(p_srvc_entry);
#ifdef CONFIG_PHL_NAN_DEBUG
	_phl_nan_dump_service(p_srvc_entry);
#endif

exit:
	return psts;
}

/* (ac) rtw_remove_nan_service */
static struct rtw_phl_nan_srvc_instance_entry *
_phl_nan_remove_service(struct phl_info_t *phl_info, u64 cookie)
{
	struct phl_nan_de_priv *de_info = phl_nan_get_de_info(phl_info);
	struct rtw_phl_nan_srvc_instance_entry *srvc_inst = NULL;
	void *d = phl_info->phl_com->drv_priv;
#ifdef CONFIG_NAN_MERGE_PROCESS
	u8 instance_id = 0;
	u8 is_publish = 0;
#endif /* CONFIG_NAN_MERGE_PROCESS */

	if (true == phl_nan_find_srvc_inst_entry(d,
						 de_info,
						 (void *)&cookie,
						 GET_SRVC_BY_COOKIE,
						 &srvc_inst)) {
#ifdef CONFIG_NAN_MERGE_PROCESS
		is_publish = phl_nan_is_pub_srvc_inst(srvc_inst);
		instance_id = phl_nan_get_inid_from_entry(srvc_inst);
#endif /* CONFIG_NAN_MERGE_PROCESS */

		_phl_nan_remove_srvc_instance_data(d, de_info, srvc_inst);

		phl_nan_done_srvc_inst_entry(d, de_info, srvc_inst);
#ifdef CONFIG_NAN_MERGE_PROCESS
		/* Send H2C to stop bcast NAN sdf */
		/* phl_nan_issue_de_info(phl_info, de_info); */

		/* Update the content of bcast sdf */
		/* phl_nan_dl_bcast_sdf(phl_info); */

		/* if (is_publish) {
			rtw_cfgvendor_nan_event_hdr_publish_terminated(
				instance_id,
				NAN_SERVICE_TERMINATION_REASON_USER_INITIATED);
		} else {
			rtw_cfgvendor_nan_event_hdr_subscribe_terminated(
				instance_id,
				NAN_SERVICE_TERMINATION_REASON_USER_INITIATED);
		} */
#endif /* CONFIG_NAN_MERGE_PROCESS */
	} else {
		srvc_inst = NULL;

		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: service not found, cookie =%llu!\n",
			  __func__, cookie);
	}

	return srvc_inst;
}

enum rtw_phl_status
phl_del_nan_func_evt_hdl(struct phl_info_t *phl_info, void *pbuf)
{
	struct phl_nan_event_parm *p_parm = (struct phl_nan_event_parm *)pbuf;
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry = NULL;
	enum rtw_phl_status psts = RTW_PHL_STATUS_FAILURE;
	u64 cookie = *(u64 *)p_parm->data;
	u8 type = 0;

	p_srvc_entry = _phl_nan_remove_service(phl_info, cookie);
	if (p_srvc_entry != NULL) {
		type = p_srvc_entry->service_control & SCF_TYPE_MASK;
		rtw_phl_nan_free_service_data(phl_to_drvpriv(phl_info),
					      type, p_srvc_entry->u_srv_data);
		rtw_phl_nan_free_service_entry(phl_to_drvpriv(phl_info),
					       p_srvc_entry);
		psts = RTW_PHL_STATUS_SUCCESS;
	}

	return psts;
}

enum rtw_phl_status
rtw_phl_cmd_add_nan_func(void *phl, u8 *p_instance_id, u8 srv_type,
			 union rtw_phl_nan_srv_data srv_data)
{
	struct phl_info_t *phl_info = (struct phl_info_t *)phl;
	struct phl_nan_add_srv_parm nan_add_srv_parm = {0};
	void *d = phl_to_drvpriv(phl_info);
	enum rtw_phl_status psts;

	nan_add_srv_parm.srv_type = srv_type;
	_os_mem_cpy(d, (void *)&nan_add_srv_parm.srv_data, (void *)&srv_data,
		    sizeof(union rtw_phl_nan_srv_data));
	nan_add_srv_parm.p_instance_id = p_instance_id;
	psts = phl_nan_enqueue_cmd(phl_info, NAN_EVENT_ADD_NAN_FUNC,
				   &nan_add_srv_parm,
				   sizeof(struct phl_nan_add_srv_parm),
				   PHL_CMD_WAIT, 2000);
	return psts;
}

enum rtw_phl_status
rtw_phl_cmd_del_nan_func(void *phl, u64 cookie)
{
	struct phl_info_t *phl_info = (struct phl_info_t *)phl;
	enum rtw_phl_status psts;

	psts = phl_nan_enqueue_cmd(phl_info, NAN_EVENT_DEL_NAN_FUNC, &cookie,
				   sizeof(u64), PHL_CMD_NO_WAIT, 0);
	return psts;
}

static void _phl_nan_prepare_nan_sdf_attr(struct phl_nan_de_priv *de_info,
					  struct phl_nan_sdf_ctx *psdf_ctx,
					  u8 *nan_attr_buf,
					  u16 *attr_len)
{
	struct phl_nan_info *nan_info = phl_nan_get_info(NAN_EN_DISC, de_info);
	struct phl_nan_mac_info *mac = phl_nan_de_info_to_mac_info(de_info);
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct rtw_phl_nan_publish_info *pub_data = NULL;
	struct rtw_phl_nan_service_info *p_si = NULL;
	u8 is_sdea_append = false;

	/* NAN IE */
	*attr_len += phl_nan_append_attr(d, mac, nan_attr_buf + *attr_len,
					 NAN_ATTR_ID_SERVICE_DESCRIPTOR,
					 psdf_ctx);

	p_si = phl_nan_get_si_from_entry(psdf_ctx->p_srvc_entry);
	if ((p_si) &&
	   ((p_si->len > MAX_SDA_SRVC_INFO_LEN) ||
	    (phl_nan_is_start_with_wfa_oui(d, p_si->info)))) {
		*attr_len += phl_nan_append_attr(d, mac,
						 nan_attr_buf + *attr_len,
						 NAN_ATTR_ID_SRV_DES_EXT,
						 psdf_ctx);
		is_sdea_append = true;
	}

	pub_data = phl_nan_get_pub_from_entry(psdf_ctx->p_srvc_entry);
	if (pub_data &&
	    TEST_STATUS_FLAG(pub_data->sde_control, NAN_SDE_CTRL_DATAPATH_REQ_REQUIRED)) {
		/* Append these IEs only when Data Required in Publish SDFs */
		if (is_sdea_append == false)
			*attr_len += phl_nan_append_attr(d,
							 mac,
							 nan_attr_buf + *attr_len,
							 NAN_ATTR_ID_SRV_DES_EXT,
							 psdf_ctx);
		*attr_len += phl_nan_append_attr(d,
						 mac,
						 nan_attr_buf + *attr_len,
						 NAN_ATTR_ID_DEVICE_CAPABILITY,
						 NULL);
		*attr_len += phl_nan_append_attr(d,
						 mac,
						 nan_attr_buf + *attr_len,
						 NAN_ATTR_ID_NAN_AVAIL,
						 NULL);
	} else if (pub_data && pub_data->awake_dw_int) {
		/* RTW_INFO("phl_nan_issue_nan_sdf! awakeW(%d)\n",
			    psdf_ctx->srvc_ext_info.awake_dw_int); */
		*attr_len += phl_nan_append_attr(d,
						 mac,
						 nan_attr_buf + *attr_len,
						 NAN_ATTR_ID_DEVICE_CAPABILITY,
						 psdf_ctx);
	}

	if (pub_data &&
	    TEST_STATUS_FLAG(pub_data->sde_control, NAN_SDE_CTRL_SEC_REQUIRED)) {
		*attr_len += phl_nan_append_attr(d,
						 mac,
						 nan_attr_buf + *attr_len,
						 NAN_ATTR_ID_CIPHER_SUITE_INFO,
						 (void *)psdf_ctx);
		*attr_len += phl_nan_append_attr(d,
						 mac,
						 nan_attr_buf + *attr_len,
						 NAN_ATTR_ID_SECURITY_CONTEXT_INFO,
						 (void *)psdf_ctx);
	}

	*attr_len += phl_nan_append_attr(d, mac, nan_attr_buf + *attr_len,
					 NAN_ATTR_ID_NAN_CONNECTION_CAPABILITY,
					 NULL);
}

/**
 * _phl_nan_issue_bcast_sdf - issue NAN Service Discovery broadcast frames
 * @de_info: priv data of de
 * @type: broadcast service discovery frame type
 * @direct:
 * (ac) issue_nan_bcast_sdf
 *
 * Returns:
 * _SUCCESS: alloc_mgtxmitframe success
 * _FAIL: alloc_mgtxmitframe fail
 */
u8
_phl_nan_issue_bcast_sdf(void *d,
			 struct phl_nan_de_priv *de_info,
			 u8 type,
			 u8 direct)
{
	struct phl_nan_mac_info *mac = phl_nan_de_info_to_mac_info(de_info);
	struct phl_nan_info *nan_info = phl_nan_get_info(NAN_EN_DISC, de_info);
	struct phl_queue *dedicated_q = NULL;
	struct phl_nan_sdf_ctx sdf_ctx = {0};
	struct rtw_phl_nan_srvc_instance_entry *srvc_entry = NULL, *n = NULL;
	struct rtw_phl_nan_publish_info *pub_data = NULL;
	struct rtw_phl_nan_service_info *p_si = NULL;
	u8 nan_attr_buf[MAX_SDF_LEN] = {0}; /* w/o tx desc, wlanhdr, and sdf_hdr */
	u8 first_service = true;
	u16 attr_len = 0;
	u8 ndp_flag = false;
	u8 security_flag = false;
	u8 awake_dw_flag = false;
	u8 is_sdea_append = false;
	u8 cluster_id[MAC_ALEN] = {0};

	if (!nan_info->nan_ops.tx_mgnt) {
		PHL_ERR("%s: nan_ops.tx_mgnt is a null pointer !!\n", __func__);
		return false;
	}

	_os_mem_cpy(d, sdf_ctx.peer_addr, nan_network_id, ETH_ALEN);
	sdf_ctx.requestor_id = 0x0;
	sdf_ctx.binding_bitmap = 0;

	if (type == NAN_SRVC_TYPE_UNSOLICITED_PUBLISH)
		dedicated_q = &de_info->publish_srvc_queue;
	else if (type == NAN_SRVC_TYPE_ACTIVE_SUBSCRIBE)
		dedicated_q = &de_info->subscribe_srvc_queue;
	else
		return false;

	/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s for type %x =>\n", __func__, type); */

	_os_spinlock(d, &dedicated_q->lock, _bh, NULL);
	phl_list_for_loop_safe(srvc_entry, n,
			       struct rtw_phl_nan_srvc_instance_entry,
			       &dedicated_q->queue, list) {

		if (!phl_nan_check_srv_type(srvc_entry, type))
			continue;

		sdf_ctx.p_srvc_entry = srvc_entry;
		sdf_ctx.service_control = srvc_entry->service_control;
		sdf_ctx.instance_id = phl_nan_get_inid_from_entry(srvc_entry);

		pub_data = phl_nan_get_pub_from_entry(srvc_entry);
		if (pub_data) {
			if (TEST_STATUS_FLAG(pub_data->sde_control,
					     NAN_SDE_CTRL_DATAPATH_REQ_REQUIRED))
				ndp_flag = true;
			if (TEST_STATUS_FLAG(pub_data->sde_control,
					     NAN_SDE_CTRL_SEC_REQUIRED))
				security_flag = true;
		}

		/* contruct nan attribute here */
		/* leave mgntframe amd sdf/naf header to nan_ops.tx_mgnt */
		if (first_service)
			first_service = false;

		/* NAN IE */
		attr_len += phl_nan_append_attr(d,
						mac,
						nan_attr_buf + attr_len,
						NAN_ATTR_ID_SERVICE_DESCRIPTOR,
						&sdf_ctx);

		p_si = phl_nan_get_si_from_entry(sdf_ctx.p_srvc_entry);
		if (p_si &&
		    ((p_si->len > MAX_SDA_SRVC_INFO_LEN) ||
		     phl_nan_is_start_with_wfa_oui(d, p_si->info))) {
			attr_len += phl_nan_append_attr(d,
							mac,
							nan_attr_buf + attr_len,
							NAN_ATTR_ID_SRV_DES_EXT,
							&sdf_ctx);
			is_sdea_append = true;
		}

		if ((type == NAN_SRVC_TYPE_UNSOLICITED_PUBLISH) &&
		    ndp_flag &&
		    (is_sdea_append == false)) {
			attr_len += phl_nan_append_attr(d, mac,
				nan_attr_buf + attr_len,
				NAN_ATTR_ID_SRV_DES_EXT,
				&sdf_ctx);
		}

		/* RTW_INFO("%s: type(%x), awake_dw_int(%d)\n", __func__,
			    type, sdf_ctx.srvc_ext_info.awake_dw_int); */
		if (((type == NAN_SRVC_TYPE_UNSOLICITED_PUBLISH) ||
		     (type == NAN_SRVC_TYPE_ACTIVE_SUBSCRIBE)) &&
		    phl_nan_get_adw_int_from_entry(sdf_ctx.p_srvc_entry)) {
			awake_dw_flag = true;
		}

		if (attr_len > MAX_SDF_LEN) {
			/* TODO
			*
			* NAN SDF or the NAN NAF should not exceed 200 microseconds
			* but shall not exceed 400 microseconds
			* MAX_SDF_LEN also need to consider rsvd page size
			*/
#if (0)
			pattrib->last_txcmdsz = pattrib->pktlen;
			dump_mgntframe(padapter, pmgntframe);
			ret = _SUCCESS;
			first_service = true;
			if (direct == false) {
				RTW_INFO("%s: Offload only one bcast sdf\n",
					 __func__);
				_exit_critical_bh(&dedicated_q->lock, &irqL);
				goto exit;
			}
#endif
		}
	}
	_os_spinunlock(d, &dedicated_q->lock, _bh, NULL);

	if (attr_len) {
		/* Other NAN IEs */
		nan_info = phl_nan_get_info(NAN_EN_DISC, de_info);
		attr_len += phl_nan_append_attr(d,
						mac,
						nan_attr_buf + attr_len,
						NAN_ATTR_ID_ELEMENT_CONTAINER,
						nan_info);

		if (((type == NAN_SRVC_TYPE_UNSOLICITED_PUBLISH) && ndp_flag) ||
		    awake_dw_flag) {
			/* Append these IEs only when Data Required in Publish SDFs */
			attr_len += phl_nan_append_attr(d,
							mac,
							nan_attr_buf + attr_len,
							NAN_ATTR_ID_DEVICE_CAPABILITY,
							NULL);
			if (ndp_flag) {
				attr_len += phl_nan_append_attr(
						d,
						mac,
						nan_attr_buf + attr_len,
						NAN_ATTR_ID_NAN_AVAIL,
						NULL);
			}
		}

		if ((type <= NAN_SRVC_TYPE_SOLICITED_PUBLISH)
			&& (security_flag == true)) {
			attr_len += phl_nan_append_attr(d, mac,
					nan_attr_buf + attr_len,
					NAN_ATTR_ID_CIPHER_SUITE_INFO,
					NULL);
			attr_len += phl_nan_append_attr(d, mac,
					nan_attr_buf + attr_len,
					NAN_ATTR_ID_SECURITY_CONTEXT_INFO,
					NULL);
		}

		attr_len += phl_nan_append_attr(d, mac, nan_attr_buf + attr_len,
					NAN_ATTR_ID_NAN_CONNECTION_CAPABILITY,
					NULL);

		/* issue via tx_mgnt */
		phl_nan_get_cluster_id(nan_info, cluster_id);
		nan_info->nan_ops.tx_mgnt(d,
					  mac->mgn_mac_id,
					  NULL,
					  mac->nmi.mac,
					  cluster_id,
					  nan_info->wrole_addr,
					  nan_attr_buf,
					  attr_len,
					  NAN_PKT_TYPE_SDF);
	}

	return true;
}

void phl_nan_send_bcast_sdf(void *d, struct phl_nan_de_priv *de_info)
{
	_phl_nan_issue_bcast_sdf(d, de_info,
				 NAN_SRVC_TYPE_UNSOLICITED_PUBLISH, true);
	_phl_nan_issue_bcast_sdf(d, de_info,
				 NAN_SRVC_TYPE_ACTIVE_SUBSCRIBE, true);
}


void phl_nan_de_on_sdf(struct phl_nan_de_priv *de_info, u8 *recv_frame, u16 len)
{
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s\n", __func__);
	/* Parsing all IEs in SDF */
	if (false == _phl_nan_parsing_sdf_attr(de_info, recv_frame, len))
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: Parsing NAN SDF Attributes Fail!\n", __func__);
}

void phl_nan_disc_engine_evt_hdl(struct phl_nan_de_priv *de_info, u32 type, void *pbuf)
{
	struct phl_nan_info *nan_info = phl_nan_get_info(NAN_EN_DISC, de_info);
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_event_parm *p_parm = (struct phl_nan_event_parm *)pbuf;
	struct phl_nan_de_send_sdf_cmd send_sdf_cmd = {0};
	struct rtw_phl_nan_followup_info *fdata = NULL;
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry = NULL;
	struct phl_nan_peer_info_entry *peer_info = NULL;
	struct phl_nan_peer_srvc_info_entry *peer_srv_info = NULL;
	struct rtw_phl_nan_rpt_match_srv para = {0};
	struct phl_nan_sdf_ctx srf_ctx = {0};
	u8 no_srvc_inst = false;
	u8 no_peer_info = false;
	u8 sdf_attr_buf[MAX_SDF_LEN] = {0};
	u16 sdf_attr_len = 0;
	u8 mgnt_need_enq = false;
	u8 *srvc_info = NULL;

	_os_mem_cpy(d, &send_sdf_cmd, p_parm->data, p_parm->length);

	if (send_sdf_cmd.auto_followup_serv_inst == true) {
		if (false == _phl_nan_deq_auto_flup_srv_inst(d,
							     de_info,
							     send_sdf_cmd.serv_inst_ck,
							     &p_srvc_entry)) {
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "%s: auto_flup_inst_entry not found (auto flup cookie=%llu)\n",
				  __func__, send_sdf_cmd.serv_inst_ck);
			no_srvc_inst = true;
		}
	} else {
		if (false == phl_nan_find_srvc_inst_entry(d,
							  de_info,
							  &send_sdf_cmd.serv_inst_ck,
							  GET_SRVC_BY_COOKIE,
							  &p_srvc_entry)) {
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "%s: serv_inst_entry not found (cookie=%llu)\n",
				  __func__, send_sdf_cmd.serv_inst_ck);
			no_srvc_inst = true;
		}
	}

	if (false == phl_nan_is_peer_info_exist(nan_info,
						send_sdf_cmd.peer_info_mac,
						&peer_info)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: peer_info not found (%02x:%02x:%02x:%02x:%02x:%02x)\n", __func__,
			  send_sdf_cmd.peer_info_mac[0],
			  send_sdf_cmd.peer_info_mac[1],
			  send_sdf_cmd.peer_info_mac[2],
			  send_sdf_cmd.peer_info_mac[3],
			  send_sdf_cmd.peer_info_mac[4],
			  send_sdf_cmd.peer_info_mac[5]);
		no_peer_info = true;
	} else {
		/* Ensure that ref_cnt can be counted down */
		peer_srv_info = (struct phl_nan_peer_srvc_info_entry *)
				(send_sdf_cmd.p_peer_srvc_info_entry);
		peer_srv_info->ref_cnt--;
	}

	if (no_peer_info || no_srvc_inst) {
		if (!no_peer_info)
			phl_nan_done_peer_info_entry(d, nan_info);
		if (!no_srvc_inst) {
			if (send_sdf_cmd.auto_followup_serv_inst == true) {
				phl_nan_auto_followup_free_mem(de_info,
							       p_srvc_entry,
							       false);
			} else {
				phl_nan_done_srvc_inst_entry(d, de_info,
							     p_srvc_entry);
			}
		}

		goto exit;
	}

	switch (type) {
	case NAN_EVENT_SEND_SDF:
		srf_ctx.p_srvc_entry = p_srvc_entry;
		srf_ctx.service_control = p_srvc_entry->service_control;
		if (phl_nan_chk_pub_solicited_tx_for_srvc_inst(
			p_srvc_entry, NAN_SOLICITED_TX_BROADCAST) ||
		    phl_nan_is_active_subscribe_srvc_instance(p_srvc_entry)) {
			_os_mem_cpy(d, srf_ctx.peer_addr,
				    nan_network_id, MAC_ALEN);
			srf_ctx.requestor_id = 0;
		} else {
			_os_mem_cpy(d, srf_ctx.peer_addr, peer_info->mac,
				    MAC_ALEN);
			srf_ctx.requestor_id = peer_srv_info->peer_inst_id;
		}
		srf_ctx.binding_bitmap = 0;
		srf_ctx.instance_id = phl_nan_get_inid_from_entry(p_srvc_entry);

		if (phl_nan_is_followup_srvc_inst(p_srvc_entry)) {
			fdata = phl_nan_get_followup_from_entry(p_srvc_entry);
			srf_ctx.instance_id = fdata->followup_id;
			srf_ctx.requestor_id = fdata->followup_reqid;
		}
		srf_ctx.p_peer_info_entry = peer_info;
		srf_ctx.p_peer_srvc_info_entry = peer_srv_info;

		_phl_nan_prepare_nan_sdf_attr(de_info, &srf_ctx, sdf_attr_buf,
					      &sdf_attr_len);

		mgnt_need_enq = phl_nan_is_mgnt_frame_need_enqueue(nan_info,
								   peer_info);

		/* cfgvendor merge */
		/* if (SCF_TYPE_PUBLISH ==
		   (p_srvc_entry->service_control & SCF_TYPE_MASK))
			rtw_cfgvendor_nan_event_hdr_publish_replied_private(padapter,
				p_srvc_entry,
				send_sdf_cmd.p_peer_info_entry,
				send_sdf_cmd.p_peer_srvc_info_entry);
		else if (SCF_TYPE_FOLLOWUP ==
			(p_srvc_entry->service_control & SCF_TYPE_MASK))
			rtw_cfgvendor_nan_event_hdr_follow_up_tx_complete(padapter,
				p_srvc_entry,
				send_sdf_cmd.p_peer_info_entry,
				send_sdf_cmd.p_peer_srvc_info_entry); */

		phl_nan_done_peer_info_entry(d, nan_info);

		if (send_sdf_cmd.auto_followup_serv_inst == true)
			phl_nan_auto_followup_free_mem(de_info, p_srvc_entry, false);
		else
			phl_nan_done_srvc_inst_entry(d, de_info, p_srvc_entry);

		phl_nan_insert_mgnt_frame_entry(nan_info,
						sdf_attr_buf,
						sdf_attr_len,
						send_sdf_cmd.peer_info_mac,
						NAN_PKT_TYPE_SDF,
						mgnt_need_enq);
		break;
	case NAN_EVENT_RPRT_MATCH_EVT:
		srvc_info = _os_kmem_alloc(d, peer_srv_info->srvc_info.len);
		if (!srvc_info) {
			PHL_TRACE(COMP_PHL_NAN, _PHL_WARNING_,
				  "%s: alloc mem failed when reporting a match event!\n",
				  __func__);

			phl_nan_done_peer_info_entry(d, nan_info);

			if (send_sdf_cmd.auto_followup_serv_inst == true) {
				phl_nan_auto_followup_free_mem(de_info,
							       p_srvc_entry,
							       false);
			} else {
				phl_nan_done_srvc_inst_entry(d, de_info,
							     p_srvc_entry);
			}
		} else {
			_os_mem_cpy(d, srvc_info, peer_srv_info->srvc_info.pbuf,
				    peer_srv_info->srvc_info.len);

			para.type = peer_srv_info->srvc_ctrl & SCF_TYPE_MASK;
			para.my_inst_id = phl_nan_get_inid_from_entry(p_srvc_entry);
			para.peer_inst_id = peer_srv_info->peer_inst_id;
			para.addr = send_sdf_cmd.peer_info_mac;
			para.info_len = peer_srv_info->srvc_info.len;
			para.info = srvc_info;
			para.cookie = phl_nan_get_srv_ck_from_entry(p_srvc_entry);

			phl_nan_done_peer_info_entry(d, nan_info);

			if (send_sdf_cmd.auto_followup_serv_inst == true) {
				phl_nan_auto_followup_free_mem(de_info,
							       p_srvc_entry,
							       false);
			} else {
				phl_nan_done_srvc_inst_entry(d, de_info,
							     p_srvc_entry);
			}

			nan_info->nan_ops.report_to_osdep(d, nan_info->wrole_addr,
							  NAN_REPORT_RX_MATCH_SERVICE,
							  (void *)&para, sizeof(para));

			_os_kmem_free(d, srvc_info, peer_srv_info->srvc_info.len);
		}
		break;
	default:
		phl_nan_done_peer_info_entry(d, nan_info);

		if (send_sdf_cmd.auto_followup_serv_inst == true)
			phl_nan_auto_followup_free_mem(de_info, p_srvc_entry, false);
		else
			phl_nan_done_srvc_inst_entry(d, de_info, p_srvc_entry);
		break;
	}

exit:
	return;
}

u8
phl_nan_de_dw_timeout_hdl(void *d, struct phl_nan_de_priv *de_info, u32 tsf, u32 dw_idx)
{
	/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s: nan_dw_idx[%d], awake dw intvl[%d]\n", __func__,
		  nan_dw_idx, de_info->min_awake_dw_intvl); */

	if (de_info->min_awake_dw_intvl == 0)
		return false;

	if (de_info->min_awake_dw_intvl == 1 || !(dw_idx % de_info->min_awake_dw_intvl)) {
		/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s ==>\n", __func__); */
		phl_nan_send_bcast_sdf(d, de_info);
	}

	return true;
}
#endif /* CONFIG_PHL_NAN */
