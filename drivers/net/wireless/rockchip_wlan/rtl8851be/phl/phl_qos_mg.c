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
#ifdef CONFIG_QOS_MG
#define _PHL_QOS_MG_C_
#include "phl_headers.h"

#define get_qm_ctrl(_phl) (_phl->qm_ctrl)

static void
_dump_clf_tp10(struct rtw_phl_tclas_type10 *t10)
{
	u8 idx = 0, dump_byte = 12;

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "###### dump classifier type 10 #######\n");
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]: prot_inst(%d), prot_num_next_hdr(%d)\n",
		t10->protocol_instance, t10->protocol_num_nexthdr);
	if (MAX_TCLAS_CLASSIFIER_FILTER < dump_byte) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]: filter_byte(%d)\n",
			t10->filter_byte);
		for (idx = 0; idx < MAX_TCLAS_CLASSIFIER_FILTER; idx++)
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]: filter_value[idx](%02x)\n",
				t10->filter_value[idx]);
		for (idx = 0; idx < MAX_TCLAS_CLASSIFIER_FILTER; idx++)
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]: filter_mask[idx](%02x)\n",
				t10->filter_mask[idx]);
		return;
	}
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]: filter_byte(%d), Filter val(0x%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x)\n",
		t10->filter_byte,
		t10->filter_value[0], t10->filter_value[1],
		t10->filter_value[2], t10->filter_value[3],
		t10->filter_value[4], t10->filter_value[5],
		t10->filter_value[6], t10->filter_value[7],
		t10->filter_value[8], t10->filter_value[9],
		t10->filter_value[10], t10->filter_value[11]);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]: filter_mask(0x%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x)\n",
		t10->filter_mask[0], t10->filter_mask[1],
		t10->filter_mask[2], t10->filter_mask[3],
		t10->filter_mask[4], t10->filter_mask[5],
		t10->filter_mask[6], t10->filter_mask[7],
		t10->filter_mask[8], t10->filter_mask[9],
		t10->filter_mask[10], t10->filter_mask[11]);

}

void
_dump_clf_tp4(struct rtw_phl_tclas_type4 *t4)
{
	u8 ip_ver;
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "###### dump classifier type 4 #######\n");
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: 0x%02X\n", "Classifier Mask", t4->mask);
	ip_ver = t4->version;
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "IP Version", ip_ver);

	if (ip_ver == 4) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d.%d.%d.%d\n", "IPv4 Source IP",
			t4->ip.v4.src_ip[0],
			t4->ip.v4.src_ip[1],
			t4->ip.v4.src_ip[2],
			t4->ip.v4.src_ip[3]);
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d.%d.%d.%d\n", "IPv4 Destination IP",
			t4->ip.v4.dst_ip[0],
			t4->ip.v4.dst_ip[1],
			t4->ip.v4.dst_ip[2],
			t4->ip.v4.dst_ip[3]);
	} else if (ip_ver == 6) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n", "IPv6 Source IP",
			t4->ip.v6.src_ip[0], t4->ip.v6.src_ip[1], t4->ip.v6.src_ip[2], t4->ip.v6.src_ip[3],
			t4->ip.v6.src_ip[4], t4->ip.v6.src_ip[5], t4->ip.v6.src_ip[6], t4->ip.v6.src_ip[7],
			t4->ip.v6.src_ip[8], t4->ip.v6.src_ip[9], t4->ip.v6.src_ip[10], t4->ip.v6.src_ip[11],
			t4->ip.v6.src_ip[12], t4->ip.v6.src_ip[13], t4->ip.v6.src_ip[14], t4->ip.v6.src_ip[15]);
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n", "IPv6 Destination IP",
			t4->ip.v6.dst_ip[0], t4->ip.v6.dst_ip[1], t4->ip.v6.dst_ip[2], t4->ip.v6.dst_ip[3],
			t4->ip.v6.dst_ip[4], t4->ip.v6.dst_ip[5], t4->ip.v6.dst_ip[6], t4->ip.v6.dst_ip[7],
			t4->ip.v6.dst_ip[8], t4->ip.v6.dst_ip[9], t4->ip.v6.dst_ip[10], t4->ip.v6.dst_ip[11],
			t4->ip.v6.dst_ip[12], t4->ip.v6.dst_ip[13], t4->ip.v6.dst_ip[14], t4->ip.v6.dst_ip[15]);
	}

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Source Port", t4->src_port);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Destination Port", t4->dst_port);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "DSCP", t4->dscp);

	if (ip_ver == 4)
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Protocol", t4->prot_or_nhdr);
	else
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Next Header", t4->prot_or_nhdr);

	if (ip_ver == 6)
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %s\n", "Flow Label", (char *)t4->flow_lbl);
}

static void
_dump_clf(struct rtw_phl_tclas_clf *clf)
{
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "###### dump classifier #######\n");
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[MSCS]: clf type(%d)\n", clf->tp);

	switch (clf->tp) {
	case CLSFR_TYPE_0:
		break;
	case CLSFR_TYPE_1:
		break;
	case CLSFR_TYPE_2:
		break;
	case CLSFR_TYPE_3:
		break;
	case CLSFR_TYPE_4:
		_dump_clf_tp4(&clf->u.t4);
		break;
	case CLSFR_TYPE_5:
		break;
	case CLSFR_TYPE_6:
		break;
	case CLSFR_TYPE_7:
		break;
	case CLSFR_TYPE_8:
		break;
	case CLSFR_TYPE_9:
		break;
	case CLSFR_TYPE_10:
		_dump_clf_tp10(&clf->u.t10);
		break;
	default:
		break;
	}
}

static void
_dump_qos_char(struct rtw_phl_qos_ele *qos_char, const char *caller)
{
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "######### %s: _dump_qos_char ##########\n", caller);

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Direction", qos_char->direction);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "TID", qos_char->tid);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "User Priority", qos_char->up);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: 0x%02X\n", "Presence Bitmap", qos_char->presence_bitmap);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Link ID", qos_char->link_id);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Minimum Services Interval", qos_char->min_service_interval);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Maximum Services Interval", qos_char->max_service_interval);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Minimum Data Rate", qos_char->min_datarate);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Delay Bound", qos_char->delay_bound);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Maximum MSDU Size", qos_char->max_msdu_size);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: 0x%08x\n", "Services Start Time", qos_char->service_start_time);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Services Start Time LinkID", qos_char->serv_st_lnkid);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Mean Data Rate", qos_char->mean_datarate);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Burst Size", qos_char->burst_size);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "MSDU Lifetime", qos_char->msdu_lifetime);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "MSDU Delivery Ratio", qos_char->msdu_delivery_ratio);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "MSDU Count Exponet", qos_char->msdu_count_exponet);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Medium Time", qos_char->medium_time);
}

void
_dump_scs_descriptor(struct rtw_phl_stainfo_t *sta, const char *caller)
{
	u8 idx, tclas_idx , classifier_type, req_type;
	struct rtw_phl_tclas_element *tclas_ele = NULL;
	struct rtw_phl_qos_ele *qos_char = NULL;

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "######### %s: _dump_scs_descriptor ##########\n", caller);
	for (idx = 0; idx < MAX_SCS_DESCRIPTOR; idx++) {
		if (sta->scs_info[idx].scsid != 0) {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "------------------- %d ------------------\n", idx+1);
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,"%-25s: %d\n", "SCS ID", sta->scs_info[idx].scsid);
			req_type = sta->scs_info[idx].req_type;

			if (req_type == 0)
				PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s:\033[1;36m Add \033[m\n", "Request Type");
			else if (req_type == 1)
				PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s:\033[1;36m Remove \033[m\n", "Request Type");
			else if (req_type == 2)
				PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s:\033[1;36m Change \033[m\n", "Request Type");
			else
				PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: [already process]\n", "Request Type");

			PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "iac_valid", sta->scs_info[idx].iac_valid);
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "AC UP", sta->scs_info[idx].iacp.ac_up);
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Alternate Queue", sta->scs_info[idx].iacp.alternate_queue);
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Drop Eligibility", sta->scs_info[idx].iacp.drop_eligibility);
			for (tclas_idx = 0; tclas_idx < MAX_SCS_DESCRIPTOR; tclas_idx++) {
				tclas_ele = &(sta->scs_info[idx].tclas[tclas_idx]);
				if (tclas_ele->valid) {
					PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "###### dump classifier #######\n");
					PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Classifier UP", tclas_ele->up);
					PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "Classifier Type", tclas_ele->type);
					classifier_type = tclas_ele->type;
					switch (classifier_type) {
					case CLSFR_TYPE_0:
						break;
					case CLSFR_TYPE_1:
						break;
					case CLSFR_TYPE_2:
						break;
					case CLSFR_TYPE_3:
						break;
					case CLSFR_TYPE_4:
						_dump_clf_tp4(&tclas_ele->u.t4);
						break;
					case CLSFR_TYPE_5:
						break;
					case CLSFR_TYPE_6:
						break;
					case CLSFR_TYPE_7:
						break;
					case CLSFR_TYPE_8:
						break;
					case CLSFR_TYPE_9:
						break;
					case CLSFR_TYPE_10:
						_dump_clf_tp10(&tclas_ele->u.t10);
						break;
					default:
						break;
					}
				}
			}

			PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%-25s: %d\n", "TCLAS Processing", sta->scs_info[idx].processing);
			qos_char = &sta->scs_info[idx].qos_ele;
			if (qos_char->valid) {
				_dump_qos_char(qos_char, __FUNCTION__);
			}
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "----------------------------------------\n");
		}
	}
}

static void
_dump_wfa_cap(struct rtw_phl_wfa_cap_ele *cap_i, const char *caller)
{
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "######### %s: _dump_wfa_cap: ##########\n", caller);
	if (cap_i->cap[0] & QOSMG_DSCP_POL)
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[WFA_CAP]: QOSMG_DSCP_POL\n");
	if (cap_i->cap[0] & QOSMG_USOL_DSCP_POL_ASSOC)
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[WFA_CAP]: QOSMG_USOL_DSCP_POL_ASSOC\n");
	if (cap_i->cap[0] & QOSMG_NONEHT_SCS_TFC_DESC)
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[WFA_CAP]: QOSMG_NONEHT_SCS_TFC_DESC\n");
}

static void
_dump_scs_rsp(struct rtw_phl_scs_rsp_f *scs_rsp_f, const char *caller)
{
	u8 idx = 0;
	struct rtw_phl_scs_sts_duple *sts = NULL;

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "######### %s: _dump_scs_rsp ##########\n", caller);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[SCS_RSP]: dt(%d), cnt(%d)\n",
		scs_rsp_f->dt, scs_rsp_f->cnt);
	for (idx = 0; idx < scs_rsp_f->cnt; idx++) {
		sts = &scs_rsp_f->sts_list[idx];
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[SCS_RSP]: Status[%d]: SCSID(%d), sts(%d)\n",
			idx, sts->scsid, sts->sts);
	}
}

static void
_dump_qos_map(struct rtw_phl_qos_map_ele *map, const char *caller)
{
	u8 idx = 0;

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "######### %s: _dump_qos_map: ##########\n", caller);
	for (idx = 0; idx < map->exc_num; idx++) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMap]: DSCP Exce[%d]: dscp(%d), up(%d)\n",
			idx, map->dscp_exc[idx].dscp,
			map->dscp_exc[idx].up);
	}
	for (idx = 0; idx < DSCP_RNG_NUM; idx++) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMap]: UP(%d) DSCP Rng: %d ~ %d\n",
			idx, map->dscp_rng[idx].dscp_l,
			map->dscp_rng[idx].dscp_h);
	}
}

static void
_dump_mscs_desc(struct rtw_phl_mscs_desc *desc, const char *caller)
{
	struct rtw_phl_tclas_mask *tclas_mask = NULL;
	struct rtw_phl_tclas_clf *clf = NULL;
	u8 idx = 0;

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "######### %s: _dump_mscs_desc: ##########\n",
		caller);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[MSCS]: req_tp(%d), Ctrl:(Bitmap(0x%02x), lim(%d), stm_timeout(%d), tclas_mask_num(%d)\n",
		desc->req_tp, desc->up_ctrl.up_bitmap, desc->up_ctrl.up_lim,
		desc->stm_timeout, desc->tclas_mask_num);
	for (idx = 0; idx < desc->tclas_mask_num; idx++) {
		tclas_mask = &desc->tclas_mask[idx];
		clf = &tclas_mask->clf;
		_dump_clf(clf);
	}
	/* Optional subelements */
	if (desc->mscs_sts_valide) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[MSCS]: Optional subeles: mscs_sts(%d)\n",
			desc->mscs_sts);
	}
}

static void
_dump_mscs_rsp(struct rtw_phl_mscs_rsp *rsp, const char *caller)
{
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "######### %s: _dump_mscs_rsp: ##########\n",
		caller);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[MSCS Rsp]: dt(%d), sts (%d)\n",
		rsp->dt, rsp->sts);
	if (rsp->desc_valide)
		_dump_mscs_desc(&rsp->desc, __FUNCTION__);
}

static void
_dump_qm_attr_port(struct rtw_phl_qos_mg_ele *qm, const char *caller)
{
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"######### %s: _dump_qm_attr_port: ##########\n",
		caller);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"[QM Attr Port]: start_port(%d), end_port(%d)\n",
		qm->start_port, qm->end_port);
}

static void
_dump_qm_attr_dscp(struct rtw_phl_qos_mg_ele *qm, const char *caller)
{
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"######### %s: _dump_qm_attr_dscp: ##########\n",
		caller);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"[QM Attr Dscp]: dscp_pol_id(%d), req_tp(%d), dscp(%d)\n",
		qm->dscp_pol_id, qm->req_tp, qm->dscp);
}

static void
_dump_qm_attr_tclas(struct rtw_phl_qos_mg_ele *qm, const char *caller)
{
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"######### %s: _dump_qm_attr_tclas: ##########\n",
		caller);

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"[QM Attr Tclas]: tp(%d)\n", qm->clf.tp);
	if (4 == qm->clf.tp)
		_dump_clf_tp4(&qm->clf.u.t4);
	else if (10 == qm->clf.tp)
		_dump_clf_tp10(&qm->clf.u.t10);
	else
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_,
			"[QM Attr Tclas]: Unimplemented Type\n");
}

static void
_dump_qm_attr_domain(struct rtw_phl_qos_mg_ele *qm, const char *caller)
{
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"######### %s: _dump_qm_attr_domain: ##########\n",
		caller);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"[QM Attr Domain]: dom_len(%d)\n", qm->dom_len);
	debug_dump_buf(qm->dom_name, qm->dom_len, "[QM Attr Domain] Domain: ");
}

static void
_dump_qm_ele(struct rtw_phl_qos_mg_ele *qm, const char *caller)
{
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"######### %s: _dump_qm_ele: ##########\n",
		caller);
	if (qm->port_attr_valid)
		_dump_qm_attr_port(qm, __FUNCTION__);
	if (qm->dscp_attr_valid)
		_dump_qm_attr_dscp(qm, __FUNCTION__);
	if (qm->tclas_attr_valid)
		_dump_qm_attr_tclas(qm, __FUNCTION__);
	if (qm->dom_attr_valid)
		_dump_qm_attr_domain(qm, __FUNCTION__);
}

static void
_dump_dscp_pol_qur(struct rtw_phl_dscp_pol_qur *qur, const char *caller)
{
	u8 idx = 0;

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"######### %s: _dump_dscp_pol_qur: ##########\n",
		caller);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"[DSCP Pol Qur]: dt(%d), qm_ele_num(%d)\n",
		qur->dt, qur->qm_ele_num);
	for (idx = 0; idx < qur->qm_ele_num; idx++) {
		_dump_qm_ele(&qur->qm_ele[idx], __FUNCTION__);
	}
}

static void
_dump_dscp_pol_req(struct rtw_phl_dscp_pol_req *req, const char *caller)
{
	u8 idx = 0;

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"######### %s: _dump_dscp_pol_req: ##########\n",
		caller);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"[DSCP Pol Req]: dt(%d), more(%d), reset(%d), qm_ele_num(%d)\n",
		req->dt, req->ctrl.more, req->ctrl.reset, req->qm_ele_num);
	for (idx = 0; idx < req->qm_ele_num; idx++) {
		_dump_qm_ele(&req->qm_ele[idx], __FUNCTION__);
	}
}


static void
_dump_dscp_pol_rsp(struct rtw_phl_dscp_pol_rsp *rsp, const char *caller)
{
	u8 idx = 0;

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"######### %s: _dump_dscp_pol_rsp: ##########\n",
		caller);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"[DSCP Pol Rsp]: dt(%d), more(%d), reset(%d), cnt(%d)\n",
		rsp->dt, rsp->ctrl.more, rsp->ctrl.reset, rsp->cnt);
	for (idx = 0; idx < rsp->cnt; idx++) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
			"[DSCP Pol Rsp]: sts duples[%d]: id(%d), sts(%d)\n",
			idx, rsp->sts[idx].id, rsp->sts[idx].sts);
	}
}

void
phl_parse_tclas_type_0(struct rtw_phl_com_t *phl_com,
		       struct rtw_phl_stainfo_t *sta,
		       u8 scs_index,
		       u8 tclas_index,
		       u8 *ele_pos,
		       u16 ele_len)
{
	void *d = phlcom_to_drvpriv(phl_com);
	u8 *classifier_mask = ele_pos + 2;
	struct rtw_phl_tclas_element *tclas_ele = &(sta->scs_info[scs_index].tclas[tclas_index]);
	u8 tclas_offset = 0;
	u8 *tclas_pos = NULL;
	tclas_pos = ele_pos;

	tclas_ele->valid = true;

	tclas_ele->up = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
	tclas_offset += 1;

	tclas_ele->type = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
	tclas_offset += 1;

	tclas_ele->u.t0.mask = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
	tclas_offset += 1;

	if (GET_TYPE_0_SOURCE_ADDRESS_PRESENT(classifier_mask)) {
		_os_mem_cpy(d, tclas_ele->u.t0.source_addr , (tclas_pos+tclas_offset), MAC_ALEN);
		tclas_offset += 6;
	}
	if (GET_TYPE_0_DESTINATION_ADDRESS_PRESENT(classifier_mask)) {
		_os_mem_cpy(d, tclas_ele->u.t0.destination_addr , (tclas_pos+tclas_offset), MAC_ALEN);
		tclas_offset += 6;
	}
	if (GET_TYPE_0_TYPE_PRESENT(classifier_mask)) {
		tclas_ele->u.t0.type = LE_BITS_TO_2BYTE(tclas_pos+tclas_offset, 0, 16);
		tclas_offset += 2;
	}
}

void
phl_parse_tclas_type_1(struct rtw_phl_com_t *phl_com,
		       struct rtw_phl_stainfo_t *sta,
		       u8 scs_index,
		       u8 tclas_index,
		       u8 *ele_pos,
		       u16 ele_len)
{
	void *d = phlcom_to_drvpriv(phl_com);
	u8 *classifier_mask = ele_pos + 2;
	struct rtw_phl_tclas_element *tclas_ele = &(sta->scs_info[scs_index].tclas[tclas_index]);
	u8 tclas_offset = 0;
	u8 *tclas_pos = NULL;
	tclas_pos = ele_pos;

	tclas_ele->valid = true;

	tclas_ele->up = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
	tclas_offset += 1;

	tclas_ele->type = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
	tclas_offset += 1;

	tclas_ele->u.t1.mask = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
	tclas_offset += 1;

	if (GET_TYPE_1_VERSION_PRESENT(classifier_mask)) {
		tclas_ele->u.t1.version = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
		tclas_offset += 1;
	}
	if (GET_TYPE_1_SOURCE_IP_ADDRESS_PRESENT(classifier_mask)) {
		_os_mem_cpy(d, tclas_ele->u.t1.src_ip , (tclas_pos+tclas_offset), 4);
		tclas_offset += 4;
	}
	if (GET_TYPE_1_DESTINATION_IP_ADDRESS_PRESENT(classifier_mask)) {
		_os_mem_cpy(d, tclas_ele->u.t1.dst_ip , (tclas_pos+tclas_offset), 4);
		tclas_offset += 4;
	}
	if (GET_TYPE_1_SOURCE_PORT_PRESENT(classifier_mask)) {
		tclas_ele->u.t1.src_port = LE_BITS_TO_2BYTE(tclas_pos+tclas_offset, 0, 16);
		tclas_offset += 2;
	}
	if (GET_TYPE_1_DESTINATION_PORT_PRESENT(classifier_mask)) {
		tclas_ele->u.t1.dst_port = LE_BITS_TO_2BYTE(tclas_pos+tclas_offset, 0, 16);
		tclas_offset += 2;
	}
	if (GET_TYPE_1_DSCP_PRESENT(classifier_mask)) {
		tclas_ele->u.t1.dscp = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
		tclas_offset += 1;
	}
	if (GET_TYPE_1_PROTOCOL_PRESENT(classifier_mask)) {
		tclas_ele->u.t1.protocol = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
		tclas_offset += 1;
	}

}

void
phl_parse_tclas_type_4(struct rtw_phl_com_t *phl_com,
		       struct rtw_phl_stainfo_t *sta,
		       u8 scs_index,
		       u8 tclas_index,
		       u8 *ele_pos,
		       u16 ele_len)
{
	void *d = phlcom_to_drvpriv(phl_com);
	u8 *classifier_mask = ele_pos + 2;
	struct rtw_phl_tclas_element *tclas_ele = &(sta->scs_info[scs_index].tclas[tclas_index]);
	u8 tclas_offset = 0 , ip_ver = 0;
	u8 *tclas_pos = NULL;
	tclas_pos = ele_pos;

	tclas_ele->valid = true;
	tclas_ele->up = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
	tclas_offset += 1;

	tclas_ele->type = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
	tclas_offset += 1;

	tclas_ele->u.t4.mask = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
	tclas_offset += 1;

	if (GET_TYPE_4_VERSION_PRESENT(classifier_mask)) {
		ip_ver = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
		tclas_ele->u.t4.version = ip_ver;
	}
	tclas_offset += 1;

	if (ip_ver == 4) {
		if (GET_TYPE_4_SOURCE_IP_ADDRESS_PRESENT(classifier_mask))
			_os_mem_cpy(d, tclas_ele->u.t4.ip.v4.src_ip , (tclas_pos+tclas_offset), 4);
		tclas_offset += 4;
	} else {
		if (GET_TYPE_4_SOURCE_IP_ADDRESS_PRESENT(classifier_mask))
			_os_mem_cpy(d, tclas_ele->u.t4.ip.v6.src_ip , (tclas_pos+tclas_offset), 16);
		tclas_offset += 16;
	}

	if (ip_ver == 4) {
		if (GET_TYPE_4_DESTINATION_IP_ADDRESS_PRESENT(classifier_mask))
			_os_mem_cpy(d, tclas_ele->u.t4.ip.v4.dst_ip , (tclas_pos+tclas_offset), 4);
		tclas_offset += 4;
	} else {
		if (GET_TYPE_4_DESTINATION_IP_ADDRESS_PRESENT(classifier_mask))
			_os_mem_cpy(d, tclas_ele->u.t4.ip.v6.dst_ip , (tclas_pos+tclas_offset), 16);
		tclas_offset += 16;
	}

	if (GET_TYPE_4_SOURCE_PORT_PRESENT(classifier_mask))
		tclas_ele->u.t4.src_port = LE_BITS_TO_2BYTE(tclas_pos+tclas_offset, 0, 16);
	tclas_offset += 2;
	if (GET_TYPE_4_DESTINATION_PORT_PRESENT(classifier_mask))
		tclas_ele->u.t4.dst_port = LE_BITS_TO_2BYTE(tclas_pos+tclas_offset, 0, 16);
	tclas_offset += 2;
	if (GET_TYPE_4_DSCP_PRESENT(classifier_mask))
		tclas_ele->u.t4.dscp = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
	tclas_offset += 1;
	if (GET_TYPE_4_PROTOCOL_OR_NEXTHEADR_PRESENT(classifier_mask))
		tclas_ele->u.t4.prot_or_nhdr = LE_BITS_TO_1BYTE(tclas_pos+tclas_offset, 0, 8);
	tclas_offset += 1;
	if(ip_ver == 6){
		if (GET_TYPE_4_FLOW_LABEL_PRESENT(classifier_mask)) {
			_os_mem_cpy(d, tclas_ele->u.t4.flow_lbl , (tclas_pos+tclas_offset), 3);
		}
		tclas_offset += 3;
	}
}
void
phl_parse_tclas_type_10(struct rtw_phl_com_t *phl_com,
			struct rtw_phl_stainfo_t *sta,
			u8 scs_index,
			u8 tclas_index,
			u8 *ele_pos,
			u16 ele_len)
{
	void *d = phlcom_to_drvpriv(phl_com);
	struct rtw_phl_tclas_element *tclas_ele = &(sta->scs_info[scs_index].tclas[tclas_index]);
	u8 rlen = 0, filter_len = 0;
	u8 *tclas_pos = NULL;
	tclas_pos = ele_pos;

	if (ele_len > (4 + MAX_TCLAS_CLASSIFIER_FILTER + MAX_TCLAS_CLASSIFIER_FILTER)) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]%s: Size of filter_value(%d) > MAX_TCLAS_CLASSIFIER_FILTER(%d)\n",
			__FUNCTION__, (ele_len - 4) / 2, MAX_TCLAS_CLASSIFIER_FILTER);
		return;
	}
	tclas_ele->valid = true;

	tclas_ele->up = LE_BITS_TO_1BYTE(tclas_pos, 0, 8);
	rlen = 1;

	tclas_ele->type = LE_BITS_TO_1BYTE(tclas_pos + rlen, 0, 8);
	rlen += 1;

	tclas_ele->u.t10.protocol_instance = LE_BITS_TO_1BYTE(tclas_pos + rlen, 0, 8);
	rlen += 1;
	tclas_ele->u.t10.protocol_num_nexthdr = LE_BITS_TO_1BYTE(tclas_pos + rlen, 0, 8);
	rlen += 1;
	filter_len = (u8)((ele_len - rlen) / 2);
	tclas_ele->u.t10.filter_byte = filter_len;
	_os_mem_cpy(d, tclas_ele->u.t10.filter_value, (tclas_pos + rlen), filter_len);
	rlen += filter_len;
	_os_mem_cpy(d, tclas_ele->u.t10.filter_mask, (tclas_pos + rlen), filter_len);
	rlen += filter_len;
}

u16
_parse_clf_tp10(struct phl_info_t *phl, u8 *pkt,
		u16 len, u8 clf_len, struct rtw_phl_tclas_type10 *t10)
{
	void *d = phl_to_drvpriv(phl);
	u8 rlen = 0, filter_len = 0;
	u8 tp = 0;

	if (clf_len > (3 + MAX_TCLAS_CLASSIFIER_FILTER + MAX_TCLAS_CLASSIFIER_FILTER)) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]%s: Size of filter_value(%d) > MAX_TCLAS_CLASSIFIER_FILTER(%d)\n",
			__FUNCTION__, (clf_len - 3) / 2, MAX_TCLAS_CLASSIFIER_FILTER);
		goto _exit;
	}

	tp = LE_BITS_TO_1BYTE(pkt + rlen, 0, 8);
	rlen = 1;
	t10->protocol_instance = LE_BITS_TO_1BYTE(pkt + rlen, 0, 8);
	rlen += 1;
	t10->protocol_num_nexthdr = LE_BITS_TO_1BYTE(pkt + rlen, 0, 8);
	rlen += 1;
	filter_len = (clf_len - rlen) / 2;
	t10->filter_byte = filter_len;
	_os_mem_cpy(d, t10->filter_value, (pkt + rlen), filter_len);
	rlen += filter_len;
	_os_mem_cpy(d, t10->filter_mask, (pkt + rlen), filter_len);
	rlen += filter_len;
_exit:
	return rlen;
}

u16
_parse_clf_tp4(struct phl_info_t *phl, u8 *pkt,
		u16 len, struct rtw_phl_tclas_type4 *t4)
{
	void *d = phl_to_drvpriv(phl);
	u16 rlen  = 0;
	u8 *clf_mask = NULL;
	u8 tp = 0, ip_ver = 0;

	tp = LE_BITS_TO_1BYTE(pkt + rlen, 0, 8);
	rlen = 1;
	clf_mask = pkt + rlen;
	t4->mask = LE_BITS_TO_1BYTE(pkt + rlen, 0, 8);
	rlen += 1;

	if (GET_TYPE_4_VERSION_PRESENT(clf_mask)) {
		ip_ver = LE_BITS_TO_1BYTE(pkt + rlen, 0, 8);
		t4->version = ip_ver;
	}
	rlen += 1;

	if (ip_ver == 4) {
		if (GET_TYPE_4_SOURCE_IP_ADDRESS_PRESENT(clf_mask))
			_os_mem_cpy(d, t4->ip.v4.src_ip , (pkt + rlen), 4);
		rlen += 4;
	} else {
		if (GET_TYPE_4_SOURCE_IP_ADDRESS_PRESENT(clf_mask))
			_os_mem_cpy(d, t4->ip.v6.src_ip , (pkt + rlen), 16);
		rlen += 16;
	}

	if (ip_ver == 4) {
		if (GET_TYPE_4_DESTINATION_IP_ADDRESS_PRESENT(clf_mask))
			_os_mem_cpy(d, t4->ip.v4.dst_ip , (pkt + rlen), 4);
		rlen += 4;
	} else {
		if (GET_TYPE_4_DESTINATION_IP_ADDRESS_PRESENT(clf_mask))
			_os_mem_cpy(d, t4->ip.v6.dst_ip , (pkt + rlen), 16);
		rlen += 16;
	}

	if (GET_TYPE_4_SOURCE_PORT_PRESENT(clf_mask))
		t4->src_port = LE_BITS_TO_2BYTE(pkt + rlen, 0, 16);
	rlen += 2;
	if (GET_TYPE_4_DESTINATION_PORT_PRESENT(clf_mask))
		t4->dst_port = LE_BITS_TO_2BYTE(pkt + rlen, 0, 16);
	rlen += 2;
	if (GET_TYPE_4_DSCP_PRESENT(clf_mask))
		t4->dscp = LE_BITS_TO_1BYTE(pkt + rlen, 0, 8);
	rlen += 1;
	if (GET_TYPE_4_PROTOCOL_OR_NEXTHEADR_PRESENT(clf_mask))
		t4->prot_or_nhdr = LE_BITS_TO_1BYTE(pkt + rlen, 0, 8);
	rlen += 1;
	if(ip_ver == 6){
		if (GET_TYPE_4_FLOW_LABEL_PRESENT(clf_mask)) {
			_os_mem_cpy(d, t4->flow_lbl , (pkt + rlen), 3);
		}
		rlen += 3;
	}
	return rlen;
}

u8
phl_parse_qos_char(u8 *buf, struct rtw_phl_qos_ele *qos_char)
{
	u8 rlen = 0;
	u8 ele_id = 0, ext_ele_id = 0, ele_len = 0;

	ele_id = GET_ELE_ID(buf);
	if (ele_id != EID_EXTENSION)
		goto _exit;
	ele_len = GET_ELE_LEN(buf + ELE_SZ);
	ext_ele_id = GET_EXTEND_ELE_ID(buf + ELE_SZ + LEN_SZ);
	if (ext_ele_id != EID_EXT_QOS_CHARACTERISTICS)
		goto _exit;
	rlen = ELE_SZ + LEN_SZ + EX_ELE_SZ;
	/* Ctrl */
	qos_char->direction = GET_QOS_CONTROL_INFO_DIRECTION(buf + rlen);
	qos_char->tid = GET_QOS_CONTROL_INFO_TID(buf + rlen);
	qos_char->up = GET_QOS_CONTROL_INFO_USER_PRIORITY(buf + rlen);
	qos_char->presence_bitmap = GET_QOS_CONTROL_INFO_BITMAP(buf + rlen);
	qos_char->link_id = GET_QOS_CONTROL_INFO_LINKID(buf + rlen);
	rlen += QOS_CHAR_CTRL_SZ;

	qos_char->min_service_interval =
			GET_QOS_ELE_MINIMUM_SERVICE_INTERVAL(buf + rlen);
	rlen += QOS_CHAR_MIN_SER_INTVL_SZ;
	qos_char->max_service_interval =
			GET_QOS_ELE_MAXIMUM_SERVICE_INTERVAL(buf + rlen);
	rlen += QOS_CHAR_MAX_SER_INTVL_SZ;
	qos_char->min_datarate =
			GET_QOS_ELE_MINIMUM_DATA_RATE(buf + rlen);
	rlen += QOS_CHAR_MIN_DATA_RATE_SZ;
	qos_char->delay_bound =
		GET_QOS_ELE_DELAY_BOUND(buf + rlen);
	rlen += QOS_CHAR_DELAY_BOUND_SZ;
	/* Option */
	if (qos_char->presence_bitmap & QOS_CHAR_MAX_MSDU_SZ_PRESN) {
		qos_char->max_msdu_size =
			GET_QOS_ELE_MAXIMUM_MSDU_SIZE(buf + rlen);
		rlen += QOS_CHAR_MAX_MSDU_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_SER_ST_PRESN) {
		qos_char->service_start_time =
			GET_QOS_ELE_SERVICE_START_TIME(buf + rlen);
		rlen += QOS_CHAR_SER_ST_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_SER_ST_LNKID_PRESN) {
		qos_char->serv_st_lnkid =
			GET_QOS_ELE_SERVICE_START_TIME_LNKID(buf + rlen);
		rlen += QOS_CHAR_SER_ST_LNKID_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_MEAN_DATA_RATE_PRESN) {
		qos_char->mean_datarate = GET_QOS_ELE_MEAN_DATA_RATE(buf + rlen);
		rlen += QOS_CHAR_MEAN_DATA_RATE_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_BURST_SZ_PRESN) {
		qos_char->burst_size = GET_QOS_ELE_BURST_SIZE(buf + rlen);
		rlen += QOS_CHAR_BURST_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_MSDU_LIFE_T_PRESN) {
		qos_char->msdu_lifetime = GET_QOS_ELE_MSDU_LIFETIME(buf + rlen);
		rlen += QOS_CHAR_MSDU_LIFETIME_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_DELIVERY_RATIO_PRESN) {
		qos_char->msdu_delivery_ratio =
			GET_QOS_ELE_MSDU_DELIVERY_RATIO(buf + rlen);
		rlen += QOS_CHAR_MSDU_DELIVERY_RATIO_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_CNT_EXP_PRESN) {
		qos_char->msdu_count_exponet =
			GET_QOS_ELE_MSDU_COUNT_EXPONENT(buf + rlen);
		rlen += QOS_CHAR_MSDU_CNT_EXP_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_MED_T_PRESN) {
		qos_char->medium_time = GET_QOS_ELE_MEDIUM_TIME(buf + rlen);
		rlen += QOS_CHAR_MED_T_SZ;
	}
_exit:
	return rlen;
}


u8
rtw_phl_search_scs_index(struct rtw_phl_stainfo_t *sta, u8 req_type, u8 scs_id, u8 *scs_index)
{
	u8 idx;
	u8 is_already_scsid = 0, is_new_scsid = 0;
	if (req_type == SCS_ADD) {
		for (idx = 0; idx < MAX_SCS_DESCRIPTOR; idx++) {
			if (scs_id == sta->scs_info[idx].scsid) {
				is_already_scsid = idx;
				break;
			}
			if (sta->scs_info[idx].scsid == 0) {
				if (is_new_scsid == 0) {
					*scs_index = idx;
					is_new_scsid = 1;
				}
			}
		}
		if (is_already_scsid) {
			*scs_index = is_already_scsid;
			return SCS_REQ_SUCCESS;
		}
		if (is_new_scsid == 0)
			return INSUFFICIENT_TCLAS_PROCESSING_RESOURCES;

		return SCS_REQ_SUCCESS;

	} else if (req_type == SCS_CHANGE || req_type == SCS_REMOVE) {
		for (idx = 0; idx < MAX_SCS_DESCRIPTOR; idx++) {
			if (scs_id == sta->scs_info[idx].scsid)
			{
				*scs_index = idx;
				return SCS_REQ_SUCCESS;
			}
		}
		return REQUESTED_TCLAS_NOT_SUPPORTED;
	} else {
		return TCLAS_PROCESSING_TERMINATED;
	}
}

/*
*
* +--------+--------+-------+---------+-----------------+-------+----------+-----------------+-------------+
* | Element| Length | SCSID | Request |  Intra-Access   | TCLAS |  TCLAS   |	    QoS      |  Optional   |
* | ID     |	    |       |  Type   |Category Priority|	|Processing| Characterisitcs | Subelements |
* +--------+--------+-------+---------+-----------------+-------+----------+-----------------+-------------+
*/
u8
rtw_phl_parse_scs_descriptor(struct rtw_phl_com_t *phl_com,
				struct rtw_phl_stainfo_t *sta,
				u8 *ele_start,
				u16 ele_len)
{
	u8 *ele_pos = NULL;
	u8 scs_index = 0 , req_type = 0, scs_id = 0, element_id = 0, tclas_length = 0;
	u8 processing = 0, classifier_type = 0 ,tclas_index = 0, scs_count = 0;
	u16 offset = 0 , scs_position = 0 ,scs_len = 0;
	void *d = phlcom_to_drvpriv(phl_com);
	u8 scs_ret = SCS_REQ_SUCCESS;

	ele_pos = ele_start;
	element_id = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);   /* offset start from SCS Descriptor ELEMENT ID : 185 */
	_os_mem_set(d, &(sta->scs_temp), 0, sizeof(struct rtw_phl_temp_scs_status) * MAX_TEMP_SCS_STATUS);
	while (element_id == EID_SCS_DESCRIPTOR) {
		offset += 1;
		scs_len = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);
		offset += 1;
		scs_position = offset;
		scs_id = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);
		offset += 1;
		req_type = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);
		offset += 1;

		if (scs_count >= MAX_TEMP_SCS_STATUS)
			break;

		scs_ret = rtw_phl_search_scs_index(sta, req_type, scs_id, &scs_index);
		if (scs_ret != SCS_REQ_SUCCESS) {
			sta->scs_temp[scs_count].scsid = scs_id;
			sta->scs_temp[scs_count].status = scs_ret;
			sta->scs_temp[scs_count].req_type = req_type;
			if (ele_len > (scs_position + scs_len)) {
				offset = scs_position + scs_len;
				element_id = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);
				scs_count++;
				continue;
			} else {
				break;
			}
		}

		sta->scs_info[scs_index].req_type = req_type;
		if (req_type == SCS_REMOVE) {
			element_id = 0;
			/* TODO: call halmac api */
		}
		if (req_type == SCS_CHANGE || req_type == SCS_ADD) {
			sta->scs_info[scs_index].scsid = scs_id;
			/* need to modify */
			/* Intra-Access Category Priority */
			element_id = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);
			if (element_id == EID_INTER_AC_PRIO) {
				offset += 2;	/* skip Length field */
				sta->scs_info[scs_index].iacp.ac_up = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 3);
				sta->scs_info[scs_index].iacp.alternate_queue = LE_BITS_TO_1BYTE(ele_pos+offset, 3, 1);
				sta->scs_info[scs_index].iacp.drop_eligibility = LE_BITS_TO_1BYTE(ele_pos+offset, 4, 1);
				offset += 1;
			}
			/* TCLAS Element */
			element_id = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);
			_os_mem_set(d, &(sta->scs_info[scs_index].tclas), 0, sizeof(struct rtw_phl_tclas_element) * MAX_TCLAS_ELE);
			tclas_index = 0 ;
			while (element_id == EID_TCLAS) {
				if (tclas_index >= MAX_TCLAS_ELE) {
					sta->scs_info[scs_index].scsid = 0;
					sta->scs_temp[scs_count].scsid = scs_id;
					scs_ret = INSUFFICIENT_TCLAS_PROCESSING_RESOURCES;
					sta->scs_temp[scs_count].status = scs_ret;
					sta->scs_temp[scs_count].req_type = req_type;
					if (ele_len > (scs_position + scs_len)) {
						offset = scs_position + scs_len;
						element_id = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);
						continue;
					} else {
						break;
					}
				}
				offset += 1;
				tclas_length = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);

				offset += 2;   /* to get classifier type */
				classifier_type = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);

				switch (classifier_type) { /* TODO : parsing TCLAS (type 0~10) => (from User Priority) */
				case CLSFR_TYPE_0:
					phl_parse_tclas_type_0(phl_com, sta, scs_index, tclas_index, ele_pos+(offset-1), tclas_length);
					break;
				case CLSFR_TYPE_1:
					phl_parse_tclas_type_1(phl_com, sta, scs_index, tclas_index, ele_pos+(offset-1), tclas_length);
					break;
				case CLSFR_TYPE_2:
					break;
				case CLSFR_TYPE_3:
					break;
				case CLSFR_TYPE_4:
					phl_parse_tclas_type_4(phl_com, sta, scs_index, tclas_index, ele_pos+(offset-1), tclas_length);
					break;
				case CLSFR_TYPE_5:
					break;
				case CLSFR_TYPE_6:
					break;
				case CLSFR_TYPE_7:
					break;
				case CLSFR_TYPE_8:
					break;
				case CLSFR_TYPE_9:
					break;
				case CLSFR_TYPE_10:
					if (((tclas_length - 4) / 2) > MAX_TCLAS_CLASSIFIER_FILTER ) {
						sta->scs_info[scs_index].scsid = 0;
						sta->scs_temp[scs_count].scsid = scs_id;
						scs_ret = REQUESTED_TCLAS_NOT_SUPPORTED;
						sta->scs_temp[scs_count].status = scs_ret;
						sta->scs_temp[scs_count].req_type = req_type;
						if (ele_len > (scs_position + scs_len)) {
							offset = scs_position + scs_len;
							element_id = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);
							continue;
						}
						else {
							break;
						}
					}
					phl_parse_tclas_type_10(phl_com, sta, scs_index, tclas_index, ele_pos+(offset-1), tclas_length);
					break;
				default:
					break;
				}
				offset += tclas_length - 1;

				if (ele_len > offset) {
					element_id = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);
					tclas_index ++;
				} else {
					element_id = 0;
					break;
				}
			}
			if (element_id == EID_TCLAS_PROCESSING) {
				offset += 2;	/* skip Length field */
				processing = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);
				if (processing == 2) {
					sta->scs_info[scs_index].scsid = 0;
					sta->scs_temp[scs_count].scsid = scs_id;
					scs_ret = REQUESTED_TCLAS_NOT_SUPPORTED;
					sta->scs_temp[scs_count].status = scs_ret;
					sta->scs_temp[scs_count].req_type = req_type;
					/*If TCLAS Processing element is present , Processing subfield shall have a value of 0 or 1 */
					if (ele_len > (scs_position + scs_len)) {
						offset = scs_position + scs_len;
						element_id = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);
						continue;
					} else {
						break;
					}
				} else {
					sta->scs_info[scs_index].processing = processing;
				}
				offset += 1;
			} else {
				sta->scs_info[scs_index].processing = 1;
			}

			if (ele_len > offset) {
			/* QoS Characteristics Element */
				offset += phl_parse_qos_char(ele_pos + offset,
					&sta->scs_info[scs_index].qos_ele);
			}
		}
		if (scs_ret == SCS_REQ_SUCCESS) {
			sta->scs_temp[scs_count].scsid = scs_id;
			if (req_type == SCS_REMOVE) {
				scs_ret = TCLAS_PROCESSING_TERMINATED;
				sta->scs_temp[scs_count].status = scs_ret;
			} else {
				sta->scs_temp[scs_count].status = SCS_REQ_SUCCESS;
			}
			sta->scs_temp[scs_count].req_type = req_type;
		}
		if (ele_len > offset) {
			element_id = LE_BITS_TO_1BYTE(ele_pos+offset, 0, 8);
		}
		scs_count ++;
		/* TODO: ADD or CHANGE SUCCESS , call halmac api */
	}

	_dump_scs_descriptor(sta, __FUNCTION__);
	return scs_ret;
}

static u8
_fill_clf_tp10(struct rtw_phl_tclas_type10 *t10, u8 *buf)
{
	u8 len = 0, idx = 0;

	SET_TCLAS_TP10_PROT_INST(buf, t10->protocol_instance);
	len = TCLAS_CLF_TP10_PROT_INST_SZ;
	SET_TCLAS_TP10_PROT_NUM_NEXT_HDR(buf + len, t10->protocol_num_nexthdr);
	len += TCLAS_CLF_TP10_PROT_NUM_NEXT_HDR_SZ;
	for (idx = 0; idx < t10->filter_byte; idx++) {
		SET_TCLAS_TP10_FILTER(buf + len, t10->filter_value[idx]);
		len += 1;
	}
	for (idx = 0; idx < t10->filter_byte; idx++) {
		SET_TCLAS_TP10_FILTER(buf + len, t10->filter_mask[idx]);
		len += 1;
	}
	_dump_clf_tp10(t10);
	return len;
}

static u8
_fill_clf_tp4(struct rtw_phl_com_t *phl_com,
			struct rtw_phl_tclas_type4 *t4,
			u8 *pbuf)
{
	void *drv = phl_com->drv_priv;
	u8 ip_ver;
	u8 *pstart = pbuf;
	u8 *p = pbuf;			/* start from user priority field */

	*p = t4->mask;
	p += 1;

	ip_ver = t4->version;

	SET_TYPE_4_VERSION(p, ip_ver);
	p += 1;
	if (ip_ver == 4) {
		_os_mem_cpy(drv, p, t4->ip.v4.src_ip , 4);
		p += 4;
		_os_mem_cpy(drv, p, t4->ip.v4.dst_ip , 4);
		p += 4;
	} else if (ip_ver == 6) {
		_os_mem_cpy(drv, p, t4->ip.v6.src_ip , 16);
		p += 16;
		_os_mem_cpy(drv, p, t4->ip.v6.dst_ip , 16);
		p += 16;
	}
	SET_TYPE_4_SOURCE_PORT(p, t4->src_port);
	p += 2;
	SET_TYPE_4_DESTINATION_PORT(p, t4->dst_port);
	p += 2;
	SET_TYPE_4_DSCP(p, t4->dscp);
	p += 1;
	SET_TYPE_4_PROTOCOL_OR_NHDR(p, t4->prot_or_nhdr);
	p += 1;
	if (ip_ver == 6) {
		_os_mem_cpy(drv , p, t4->flow_lbl, 3);
		p += 3;
	} else {
		p += 1;
	}
	return (u8)(p - pstart);
}

static u8
_fill_tclas_ele(struct rtw_phl_com_t *phl_com,
		struct rtw_phl_tclas_element *tclas_ele,
		u8 *buf)
{
	u8 len = 0;

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]%s:\n",
		__FUNCTION__);
	if (tclas_ele->type != 4 && tclas_ele->type != 10) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]%s: tclas_ele->type(%d) is not implement\n",
			__FUNCTION__, tclas_ele->type);
		return len;
	}
	SET_TCLAS_ELE_ID(buf);
	len = ELE_SZ + LEN_SZ;
	SET_TCLAS_UP(buf + len, tclas_ele->up);
	len += TCLAS_UP_SZ;
	SET_TCLAS_CLF_TP(buf + len, tclas_ele->type);
	len += TCLAS_CLF_TP_SZ;
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]%s: up(%d), clf_type(%d)\n",
		__FUNCTION__, tclas_ele->up, tclas_ele->type);
	if (tclas_ele->type == 4) {
		len += _fill_clf_tp4(phl_com, &tclas_ele->u.t4,
						buf + len);
	} else if (tclas_ele->type == 10) {
		len += _fill_clf_tp10(&tclas_ele->u.t10, buf + len);
	}
	SET_TCLAS_LEN(buf + ELE_SZ, len - ELE_SZ - LEN_SZ);
	debug_dump_buf(buf, len, "[QoSMg]TCLAS Ele.: ");
	return len;
}

static u8
_fill_qos_char_ele(u8 *buf, struct rtw_phl_qos_ele *qos_char)
{
	u8 len = 0;

	SET_ELE_ID(buf, EID_EXTENSION);
	len = ELE_SZ + LEN_SZ;
	SET_EXTEND_ELE_ID(buf + len, EID_EXT_QOS_CHARACTERISTICS);
	len += EX_ELE_SZ;
	/* Ctrl */
	SET_QOS_CONTROL_INFO_DIRECTION(buf + len, qos_char->direction);
	SET_QOS_CONTROL_INFO_TID(buf + len, qos_char->tid);
	SET_QOS_CONTROL_INFO_USER_PRIORITY(buf + len, qos_char->up);
	SET_QOS_CONTROL_INFO_BITMAP(buf + len, qos_char->presence_bitmap);
	SET_QOS_CONTROL_INFO_LINKID(buf + len, qos_char->link_id);
	len += QOS_CHAR_CTRL_SZ;

	SET_QOS_ELE_MINIMUM_SERVICE_INTERVAL(buf + len, qos_char->min_service_interval);
	len += QOS_CHAR_MIN_SER_INTVL_SZ;
	SET_QOS_ELE_MAXIMUM_SERVICE_INTERVAL(buf + len, qos_char->max_service_interval);
	len += QOS_CHAR_MAX_SER_INTVL_SZ;
	SET_QOS_ELE_MINIMUM_DATA_RATE(buf + len, qos_char->min_datarate);
	len += QOS_CHAR_MIN_DATA_RATE_SZ;
	SET_QOS_ELE_DELAY_BOUND(buf + len, qos_char->delay_bound);
	len += QOS_CHAR_DELAY_BOUND_SZ;
	/* Option */
	if (qos_char->presence_bitmap & QOS_CHAR_MAX_MSDU_SZ_PRESN) {
		SET_QOS_ELE_MAXIMUM_MSDU_SIZE(buf + len, qos_char->max_msdu_size);
		len += QOS_CHAR_MAX_MSDU_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_SER_ST_PRESN) {
		SET_QOS_ELE_SERVICE_START_TIME(buf + len, qos_char->service_start_time);
		len += QOS_CHAR_SER_ST_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_SER_ST_LNKID_PRESN) {
		SET_QOS_ELE_SERVICE_START_TIME_LNKID(buf + len, qos_char->serv_st_lnkid);
		len += QOS_CHAR_SER_ST_LNKID_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_MEAN_DATA_RATE_PRESN) {
		SET_QOS_ELE_MEAN_DATA_RATE(buf + len, qos_char->mean_datarate);
		len += QOS_CHAR_MEAN_DATA_RATE_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_BURST_SZ_PRESN) {
		SET_QOS_ELE_BURST_SIZE(buf + len, qos_char->burst_size);
		len += QOS_CHAR_BURST_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_MSDU_LIFE_T_PRESN) {
		SET_QOS_ELE_MSDU_LIFETIME(buf + len, qos_char->msdu_lifetime);
		len += QOS_CHAR_MSDU_LIFETIME_SZ;
	}

	if ((qos_char->presence_bitmap & QOS_CHAR_DELIVERY_RATIO_PRESN) &&
	    (qos_char->presence_bitmap & QOS_CHAR_CNT_EXP_PRESN)) {
		u8 info = 0;

		info = qos_char->msdu_count_exponet << 4;
		info += qos_char->msdu_delivery_ratio;
		SET_QOS_ELE_MSDU_DELIVERY_INFO(buf + len,
					       info);
		len += QOS_CHAR_MSDU_DELIVERY_RATIO_SZ;
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
			"[QoSMg]%s: !!!!!! msdu_delivery_info(0x%x)\n",
			__FUNCTION__, info);
	}
/*
	if (qos_char->presence_bitmap & QOS_CHAR_DELIVERY_RATIO_PRESN) {
		SET_QOS_ELE_MSDU_DELIVERY_RATIO(buf + len, qos_char->msdu_delivery_ratio);
		len += QOS_CHAR_MSDU_DELIVERY_RATIO_SZ;
	}
	if (qos_char->presence_bitmap & QOS_CHAR_CNT_EXP_PRESN) {
		SET_QOS_ELE_MSDU_COUNT_EXPONENT(buf + len, qos_char->msdu_count_exponet);
		len += QOS_CHAR_MSDU_CNT_EXP_SZ;
	}
*/
	if (qos_char->presence_bitmap & QOS_CHAR_MED_T_PRESN) {
		SET_QOS_ELE_MEDIUM_TIME(buf + len, qos_char->medium_time);
		len += QOS_CHAR_MED_T_SZ;
	}
	SET_ELE_LEN(buf + ELE_SZ, len - (ELE_SZ + LEN_SZ)); /* QoS Characteristics Length subfield*/
	return len;
}

u8
rtw_phl_build_scs_descriptor(struct rtw_phl_com_t *phl_com,
				struct rtw_phl_stainfo_t *sta,
				u8 scs_id,
				u8 req_type,
				u8 *pbuf)
{
	u8 len = 0 , scs_idx, tclas_idx , tclas_count = 0;
	u8 *pstart = pbuf;
	u8 *p = pbuf + 4;			/* start from Intra-Access Category Priority Element */
	struct rtw_phl_tclas_element *tclas_ele = NULL;
	struct rtw_phl_scs_info *scs_i = NULL;

	for (scs_idx = 0; scs_idx < MAX_SCS_DESCRIPTOR; scs_idx++) {
		if (scs_id == sta->scs_info[scs_idx].scsid) {
			/* Element ID */
			pbuf[0] = EID_SCS_DESCRIPTOR;
			/* SCSID field */
			pbuf[2] = scs_id;
			pbuf[3] = req_type;
			len += 2;		/* lenght count start form SCSID field*/
			scs_i = &sta->scs_info[scs_idx];
			break;
		}
	}
	/* can't not found scsid */
	if (scs_i == NULL)
		return len;
	_dump_scs_descriptor(sta, __FUNCTION__);
	if ((req_type == SCS_CHANGE || req_type == SCS_ADD) &&
	     scs_i->iac_valid) {
		pbuf[4] = EID_INTER_AC_PRIO;	/*Intra-Access Category Priority Element ID : 184*/
		pbuf[5] = 1;
		p += 2;
		SET_IACP_USER_PRIORITY(p, scs_i->iacp.ac_up);
		SET_IACP_ALTERNATE_QUEUE(p, scs_i->iacp.alternate_queue);
		SET_IACP_DROP_ELIGIBILITY(p, scs_i->iacp.drop_eligibility);
		p += 1;
	}

	for (tclas_idx = 0; tclas_idx < MAX_TCLAS_ELE; tclas_idx++) {
		tclas_ele = &scs_i->tclas[tclas_idx];
		if (tclas_ele->valid) {
			p += _fill_tclas_ele(phl_com, tclas_ele, p);
			tclas_count++;
		}
	}
	if (tclas_count > 1) {			/*If the number of TCLAS is than 1, need include TCLAS Processing element */
		SET_TCLAS_PROCS_ELE_ID(p, EID_TCLAS_PROCESSING);
		p += 1;
		SET_TCLAS_PROCS_LENGTH(p, 1);
		p += 1;
		SET_TCLAS_PROCS_PROCESSING(p, scs_i->processing);
		p += 1;
	}
	if (scs_i->qos_ele.valid) {
		p += _fill_qos_char_ele(p ,&scs_i->qos_ele);
	}
	pbuf[1]= (u8)(p - pstart - 2);
	len = (u8)(p - pstart);
	debug_dump_buf(pbuf, len, "[QoSMg]SCS Desc.: ");
	return len;	/* From Element ID/ Len */
}

u16
rtw_phl_build_all_scs_desc(void *phl,
	struct rtw_phl_stainfo_t *sta, u8 *pbuf)
{
	struct phl_info_t *phl_i = (struct phl_info_t *)phl;
	struct rtw_phl_scs_info *scs_desc = NULL;
	u8 idx = 0;
	u16 len = 0;

	for (idx = 0; idx < MAX_SCS_DESCRIPTOR; idx++) {
		scs_desc = &sta->scs_info[idx];
		if (scs_desc->scsid == 0)
			continue;
		len += rtw_phl_build_scs_descriptor(phl_i->phl_com, sta,
			scs_desc->scsid, scs_desc->req_type, pbuf + len);
	}
	return len;
}

/*
 @pkt: start from Category field
 */
u8
rtw_phl_parse_scs_rsp(struct rtw_phl_scs_rsp_f *scs_rsp_f, u8 *pkt,
				u16 len)
{
	u8 idx = 0;
	u16 rlen = 0;
	struct rtw_phl_scs_sts_duple *sts = NULL;

	rlen = ACTION_CAT_SZ + ACTION_CODE_SZ;
	scs_rsp_f->dt = GET_ACTION_DT(pkt + rlen);
	rlen += ACTION_TOKEN_SZ;
	scs_rsp_f->cnt = GET_SCS_RSP_CNT(pkt + rlen);
	rlen += SCS_RSP_CNT_SZ;
	for (idx = 0; idx < scs_rsp_f->cnt; idx++) {
		if (idx >= MAX_SCS_LIST_NUM) {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_, "%s: idx(%d) >= MAX_SCS_LIST_NUM(%d)\n",
				__FUNCTION__, idx, MAX_SCS_LIST_NUM);
			break;
		}
		sts = &scs_rsp_f->sts_list[idx];
		sts->scsid = GET_SCS_RSP_SCSID(pkt + rlen);
		rlen += SCSID_SZ;
		sts->sts = GET_SCS_RSP_STS(pkt + rlen);
		rlen += SCS_RSP_STS_SZ;
	}
	_dump_scs_rsp(scs_rsp_f, __FUNCTION__);
	debug_dump_buf(pkt, len, "SCS Rsp: ");
	return true;
}

u8
rtw_phl_fill_scs_rsp(struct rtw_phl_scs_rsp_f *scs_rsp_f, u8 *buf)
{
	u8 idx = 0, len = 0;
	struct rtw_phl_scs_sts_duple *sts = NULL;

	SET_SCS_RSP_CNT(buf, scs_rsp_f->cnt);
	len = SCS_RSP_CNT_SZ;
	for (idx = 0; idx < scs_rsp_f->cnt; idx++) {
		if (idx >= MAX_SCS_LIST_NUM) {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_, "%s: idx(%d) >= MAX_SCS_LIST_NUM(%d)\n",
				__FUNCTION__, idx, MAX_SCS_LIST_NUM);
			break;
		}
		sts = &scs_rsp_f->sts_list[idx];
		SET_SCS_RSP_SCSID(buf + len, sts->scsid);
		len += SCSID_SZ;
		SET_SCS_RSP_STS(buf + len, sts->sts);
		len += SCS_RSP_STS_SZ;
	}
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "%s: len(%d)\n",
		__FUNCTION__, len);
	_dump_scs_rsp(scs_rsp_f, __FUNCTION__);
	debug_dump_buf(buf, len, "SCS Rsp: ");
	return len;
}

u16
rtw_phl_fill_wfa_cap_ele(u8 *buf, struct rtw_phl_wfa_cap_ele *cap_i)
{
	u16 len = 0;
	u8 idx = 0;

	SET_VENDOR_SPEC_ID(buf);
	len = ELE_SZ + LEN_SZ;
	SET_OUI_AND_TYPE(buf + len, WFA_OUI_CAP);
	len += OUI_TP_SZ;
	SET_WFA_CAP_LEN(buf + len, MAX_WFA_CAP);
	len += WFA_CAP_LEN_SZ;
	for (idx = 0; idx < MAX_WFA_CAP; idx++) {
		SET_WFA_CAP(buf + len, cap_i->cap[idx]);
		len += 1;
	}
	/* Final set len fields */
	SET_WFA_CAP_TOTAL_LEN(buf + ELE_SZ, len - (ELE_SZ + LEN_SZ));
	_dump_wfa_cap(cap_i, __FUNCTION__);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]%s: len(%d)\n",
		__FUNCTION__, len);
	debug_dump_buf(buf, len, "[QoSMg]WFA Cap Ele.: ");
	return len;
}

u16
rtw_phl_parse_qos_map(struct rtw_phl_qos_map_ele *map, u8 *ie, u16 ie_len)
{
	u8 ret = false;
	u8 idx = 0;
	u16 rlen = 0;
	u16 dscp_rng_len = DSCP_RNG_NUM * 2;
	u16 dscp_exc_len = ie_len - dscp_rng_len;
	u8 dscp_exc_cnt = (u8)(dscp_exc_len / 2);

	if (dscp_exc_cnt > MAX_DSCP_EXC_NUM) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_, "[QoSMg]%s: dscp_exc_cnt(%d) > MAX_DSCP_EXC_NUM(%d)\n",
			__FUNCTION__, dscp_exc_cnt, MAX_DSCP_EXC_NUM);
		goto _exit;
	}
	if (dscp_exc_cnt == 0)
		goto _dscp_rng;
	map->exc_num = dscp_exc_cnt;
	for (idx = 0; idx < map->exc_num; idx++) {
		map->dscp_exc[idx].dscp = GET_DSCP_EXC_DSCP(ie + rlen);
		rlen += 1;
		map->dscp_exc[idx].up = GET_DSCP_EXC_DSCP(ie + rlen);
		rlen += 1;
	}
_dscp_rng:
	for (idx = 0; idx < DSCP_RNG_NUM; idx++) {
		map->dscp_rng[idx].dscp_l = GET_DSCP_RNG_DSCP_L(ie + rlen);
		rlen += 1;
		map->dscp_rng[idx].dscp_h = GET_DSCP_RNG_DSCP_H(ie + rlen);
		rlen += 1;
	}
	_dump_qos_map(map, __FUNCTION__);
	ret = true;
_exit:
	return ret;
}

u16
rtw_phl_fill_qos_map(u8 *buf, struct rtw_phl_qos_map_ele *map)
{
	u16 len = 0;
	u8 idx = 0;

	if (map->exc_num > MAX_DSCP_EXC_NUM) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_, "[QoSMg]%s: info->exc_num(%d) > MAX_DSCP_EXC_NUM(%d)\n",
			__FUNCTION__, map->exc_num, MAX_DSCP_EXC_NUM);
		return len;
	}
	SET_ELE_ID(buf, EID_QOS_MAP_SET);
	len = ELE_SZ + LEN_SZ;
	for (idx = 0; idx < map->exc_num; idx++) {
		SET_DSCP_EXC_DSCP(buf + len, map->dscp_exc[idx].dscp);
		len += 1;
		SET_DSCP_EXC_UP(buf + len, map->dscp_exc[idx].up);
		len += 1;
	}
	for (idx = 0; idx < DSCP_RNG_NUM; idx++) {
		SET_DSCP_RNG_DSCP_L(buf + len, map->dscp_rng[idx].dscp_l);
		len += 1;
		SET_DSCP_RNG_DSCP_H(buf + len, map->dscp_rng[idx].dscp_h);
		len += 1;
	}
	/* Final set len fields */
	SET_ELE_LEN(buf + ELE_SZ, len - (ELE_SZ + LEN_SZ));
	_dump_qos_map(map, __FUNCTION__);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]%s: len(%d)\n",
		__FUNCTION__, len);
	debug_dump_buf(buf, len, "[QoSMg]QoS Map Ele.: ");
	return len;
}

u16
rtw_phl_parse_mscs_desc(struct rtw_phl_mscs_desc *desc, u8 *pkt, u16 len)
{
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]%s: todo\n",
		__FUNCTION__);
	return 0;
}

u16
rtw_phl_fill_mscs_desc(void *phl, u8 *buf, struct rtw_phl_mscs_desc *desc)
{
	struct phl_info_t *phl_i = (struct phl_info_t *)phl;
	struct rtw_phl_com_t *phl_c = phl_i->phl_com;
	struct rtw_phl_tclas_mask *tclas_mask = NULL;
	struct rtw_phl_tclas_clf *clf = NULL;
	u16 len = 0, tclas_start = 0, tclas_len = 0;
	u8 idx = 0;

	if (desc->tclas_mask_num > MAX_TCLAS_MASK_NUM) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_, "[QoSMg]%s: desc->tclas_mask_num(%d) > MAX_TCLAS_MASK_NUM(%d)\n",
			__FUNCTION__, desc->tclas_mask_num, MAX_TCLAS_MASK_NUM);
		goto _exit;
	}

	SET_ELE_ID(buf, EID_EXTENSION);
	len = ELE_SZ + LEN_SZ;
	SET_EXTEND_ELE_ID(buf + len, EID_EXT_MSCS_DESC);
	len += EX_ELE_SZ;

	SET_MSCS_REQ_TP(buf + len, desc->req_tp);
	len += REQ_TP_SZ;

	if (SCS_REQ_REMOVE == desc->req_tp) {
		/* Up control and stream timeout is reserved */
		len += (UP_CTRL_SZ + STM_TIMEOUT_SZ);
		goto _subele;
	}
	/* UP Control */
	SET_MSCS_UP_CTRL_BITMAP(buf + len, desc->up_ctrl.up_bitmap);
	SET_MSCS_UP_CTRL_LIM(buf + len + UP_BITMAP_SZ, desc->up_ctrl.up_lim);
	len += UP_CTRL_SZ;

	SET_MSCS_STREAM_TIMEOUT(buf + len, desc->stm_timeout);
	len += STM_TIMEOUT_SZ;

	/* TCLAS MASK- option */
	for (idx = 0; idx < desc->tclas_mask_num; idx++) {
		tclas_start = len;
		tclas_mask = &desc->tclas_mask[idx];
		clf = &tclas_mask->clf;

		SET_ELE_ID(buf + len, EID_EXTENSION);
		len += ELE_SZ + LEN_SZ;
		SET_EXTEND_ELE_ID(buf + len, EID_EXT_TCLAS_MASK);
		len += EX_ELE_SZ;
		SET_TCLAS_CLF_TP(buf + len, clf->tp);
		len += TCLAS_CLF_TP_SZ;
		if (tclas_mask->clf.tp == 4) {
			len += _fill_clf_tp4(phl_c, &clf->u.t4,
							buf + len);
		} else if (tclas_mask->clf.tp == 10) {
			len += _fill_clf_tp10(&clf->u.t10, buf + len);
		}
		tclas_len = len - tclas_start - ELE_SZ - LEN_SZ;
		SET_ELE_LEN(buf + tclas_start + ELE_SZ, tclas_len);
	}
_subele:
	/* Optional subelements */
	if (desc->mscs_sts_valide) {
		SET_MSCS_SUB_ELE_ID(buf + len, MSCS_SUBEID_MSCS_STS);
		len += MSCS_SUB_ELE_SZ;
		SET_MSCS_SUB_ELE_LEN(buf + len, STS_CODE_SZ);
		len += MSCS_SUB_LEN_SZ;
		SET_MSCS_SUB_ELE_MCSC_STS(buf + len, desc->mscs_sts);
		len += STS_CODE_SZ;
	}
	/* Final set len fields */
	SET_ELE_LEN(buf + ELE_SZ, len - (ELE_SZ + LEN_SZ));
	_dump_mscs_desc(desc, __FUNCTION__);
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]%s: len(%d)\n",
		__FUNCTION__, len);
	debug_dump_buf(buf, len, "[QoSMg]MSCS Desc: ");
_exit:
	return len;
}

/*
 @pkt: start from Category field
 * |Category|Robust Action|Dialog Token|Status|MSCS Desc(option)|
 * | <<<<<<<<<<<<<<<<<<         Len           >>>>>>>>>>>>>>>>>>|
 */
u16
rtw_phl_parse_mscs_rsp(struct rtw_phl_mscs_rsp *rsp, u8 *pkt, u16 len)
{
	u16 rlen = 0;

	rlen = ACTION_CAT_SZ + ACTION_CODE_SZ;
	rsp->dt = GET_ACTION_DT(pkt + rlen);
	rlen += ACTION_TOKEN_SZ;
	rsp->sts = GET_MSCS_RSP_STS(pkt + rlen);
	rlen += STS_CODE_SZ;
	if ((len - rlen) < MIN_MSCS_DESC_LEN)
		goto _exit;
	rlen += rtw_phl_parse_mscs_desc(&rsp->desc, pkt + rlen, len - rlen);
	_dump_mscs_rsp(rsp, __FUNCTION__);
_exit:
	return rlen;
}

u16
rtw_phl_fill_mscs_rsp(void *phl, u8 *buf, struct rtw_phl_mscs_rsp *rsp)
{
	u16 len = 0;

	SET_MSCS_RSP_STS(buf, rsp->sts);
	len += STS_CODE_SZ;
	if (STS_SUCCESS == rsp->sts)
		goto _exit;
	if (rsp->desc_valide)
		len += rtw_phl_fill_mscs_desc(phl, buf + len, &rsp->desc);
_exit:
	_dump_mscs_rsp(rsp, __FUNCTION__);
	debug_dump_buf(buf, len, "[QoSMg]MSCS Rsp: ");
	return len;
}


u16
_fill_qm_attr_port(struct phl_info_t *phl, u8 *buf, struct rtw_phl_qos_mg_ele *qm)
{
	u16 len = 0;

	SET_QM_ATTR_ID(buf, QM_ATTR_PORT);
	len = ATTR_ID_SZ + LEN_SZ;
	SET_PORT_RNG_ATTR_PORT(buf + len, qm->start_port);
	len += PORT_SZ;
	SET_PORT_RNG_ATTR_PORT(buf + len, qm->end_port);
	len += PORT_SZ;
	SET_ELE_LEN(buf + ATTR_ID_SZ, len - (ATTR_ID_SZ + LEN_SZ));
	return len;
}

u16
_fill_qm_attr_dscp(struct phl_info_t *phl, u8 *buf, struct rtw_phl_qos_mg_ele *qm)
{
	u16 len = 0;

	SET_QM_ATTR_ID(buf, QM_ATTR_DSCP);
	len = ATTR_ID_SZ + LEN_SZ;
	SET_DSCP_POL_ID(buf + len, qm->dscp_pol_id);
	len += DSCP_POL_ID_SZ;
	SET_DSCP_POL_REQ_TP(buf + len, qm->req_tp);
	len += DSCP_REQ_TP_SZ;
	SET_DSCP_POL_DSCP(buf + len, qm->dscp);
	len += DSCP_SZ;
	SET_ELE_LEN(buf + ATTR_ID_SZ, len - (ATTR_ID_SZ + LEN_SZ));
	return len;
}

u16
_fill_qm_attr_tclas(struct phl_info_t *phl, u8 *buf, struct rtw_phl_qos_mg_ele *qm)
{
	u16 len = 0;

	SET_QM_ATTR_ID(buf, QM_ATTR_TCLAS);
	len = ATTR_ID_SZ + LEN_SZ;
	SET_TCLAS_CLF_TP(buf + len, qm->clf.tp);
	len += TCLAS_CLF_TP_SZ;
	if (qm->clf.tp == 4) {
		len += _fill_clf_tp4(phl->phl_com, &qm->clf.u.t4,
						buf + len);
	} else if (qm->clf.tp == 10) {
		len += _fill_clf_tp10(&qm->clf.u.t10, buf + len);
	} else {
		len = 0;
	}
	SET_ELE_LEN(buf + ATTR_ID_SZ, len - (ATTR_ID_SZ + LEN_SZ));
	return len;
}

u16
_fill_qm_attr_domain(struct phl_info_t *phl, u8 *buf, struct rtw_phl_qos_mg_ele *qm)
{
	u16 len = 0;
	void *d = phl_to_drvpriv(phl);

	if (qm->dom_len > MAX_DOM_NAME_LEN) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_, "[QoSMg]%s: dom_len(%d) > MAX_DOM_NAME_LEN(%d)\n",
			__FUNCTION__, qm->dom_len, MAX_DOM_NAME_LEN);
		goto _exit;
	}
	SET_QM_ATTR_ID(buf, QM_ATTR_DOMAIN);
	len = ATTR_ID_SZ + LEN_SZ;
	_os_mem_cpy(d, buf + len, qm->dom_name, qm->dom_len);
	len += qm->dom_len;
	SET_ELE_LEN(buf + ATTR_ID_SZ, len - (ATTR_ID_SZ + LEN_SZ));
_exit:
	return len;
}

u16
rtw_phl_fill_qm_ele(void *phl, u8 *buf, struct rtw_phl_qos_mg_ele *qm)
{
	u16 len = 0;

	SET_ELE_ID(buf, EID_VENDOR_SPECIFIC);
	len = ELE_SZ + LEN_SZ;
	SET_OUI_AND_TYPE(buf + len, WFA_OUI_QM_ELE);
	len += OUI_TP_SZ;
	if (qm->port_attr_valid)
		len += _fill_qm_attr_port(phl, buf + len, qm);
	if (qm->dscp_attr_valid)
		len += _fill_qm_attr_dscp(phl, buf + len, qm);
	if (qm->tclas_attr_valid)
		len += _fill_qm_attr_tclas(phl, buf + len, qm);
	if (qm->dom_attr_valid)
		len += _fill_qm_attr_domain(phl, buf + len, qm);
	SET_ELE_LEN(buf + ELE_SZ, len - (ELE_SZ + LEN_SZ));
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "[QoSMg]%s: len(%d)\n",
		__FUNCTION__, len);
	debug_dump_buf(buf, len, "[QoSMg]QM Ele.: ");
	return len;
}

/*
 * DSCP Policy Query Action frame
 * @dt: dialog token
 */
u16
rtw_phl_fill_dscp_pol_qur(void *phl, u8 *buf, struct rtw_phl_dscp_pol_qur *qur)
{
	u16 len = 0;
	u8 idx = 0;

	_dump_dscp_pol_qur(qur, __FUNCTION__);
	buf[0] = ACT_CAT_VENDOR_PROTECT;
	len = ACTION_CAT_SZ;
	SET_OUI_AND_TYPE(buf + len, WFA_OUI_QM_ACT);
	len += OUI_TP_SZ;
	SET_QM_OUI_SUBTP(buf + len, DSCP_POL_Q);
	len += QM_OUI_SUBTP_SZ;
	SET_ACT_DT(buf + len, qur->dt);
	len += ACTION_TOKEN_SZ;
	for (idx = 0; idx < qur->qm_ele_num; idx++) {
		len += rtw_phl_fill_qm_ele(phl, buf + len, &qur->qm_ele[idx]);
	}
	debug_dump_buf(buf, len, "[DSCP Pol Qur] pkt: ");
	return len;
}

/*
 * DSCP Policy Request Action frame
 * @dt: dialog token
 */
u16
rtw_phl_fill_dscp_pol_req(void *phl, u8 *buf, struct rtw_phl_dscp_pol_req *req)
{
	u16 len = 0;
	u8 idx = 0;

	_dump_dscp_pol_req(req, __FUNCTION__);
	buf[0] = ACT_CAT_VENDOR_PROTECT;
	len = ACTION_CAT_SZ;
	SET_OUI_AND_TYPE(buf + len, WFA_OUI_QM_ACT);
	len += OUI_TP_SZ;
	SET_QM_OUI_SUBTP(buf + len, DSCP_POL_REQ);
	len += QM_OUI_SUBTP_SZ;
	SET_ACT_DT(buf + len, req->dt);
	len += ACTION_TOKEN_SZ;
	SET_DSCP_REQ_CTRL_MORE(buf + len, req->ctrl.more);
	SET_DSCP_REQ_CTRL_RESET(buf + len, req->ctrl.reset);
	len += DSCP_REQ_CTRL_SZ;
	for (idx = 0; idx < req->qm_ele_num; idx++) {
		len += rtw_phl_fill_qm_ele(phl, buf + len, &req->qm_ele[idx]);
	}
	debug_dump_buf(buf, len, "[DSCP Pol Req] pkt: ");
	return len;
}

/*
 * DSCP Policy Response Action frame
 * @dt: dialog token
 */
u16
rtw_phl_fill_dscp_pol_rsp(void *phl, u8 *buf, struct rtw_phl_dscp_pol_rsp *rsp)
{
	u16 len = 0;
	u8 idx = 0;

	_dump_dscp_pol_rsp(rsp, __FUNCTION__);
	buf[0] = ACT_CAT_VENDOR_PROTECT;
	len = ACTION_CAT_SZ;
	SET_OUI_AND_TYPE(buf + len, WFA_OUI_QM_ACT);
	len += OUI_TP_SZ;
	SET_QM_OUI_SUBTP(buf + len, DSCP_POL_RSP);
	len += QM_OUI_SUBTP_SZ;
	SET_ACT_DT(buf + len, rsp->dt);
	len += ACTION_TOKEN_SZ;
	SET_DSCP_RSP_CTRL_MORE(buf + len, rsp->ctrl.more);
	SET_DSCP_RSP_CTRL_RESET(buf + len, rsp->ctrl.reset);
	len += DSCP_RSP_CTRL_SZ;
	SET_DSCP_POL_STS_CNT(buf + len, rsp->cnt);
	len += DSCP_POL_STS_CNT_SZ;
	for (idx = 0; idx < rsp->cnt; idx++) {
		SET_DSCP_POL_ID(buf + len, rsp->sts[idx].id);
		len += DSCP_POL_ID_SZ;
		SET_DSCP_POL_STS(buf + len, rsp->sts[idx].sts);
		len += DSCP_POL_STS_SZ;
	}
	debug_dump_buf(buf, len, "[DSCP Pol Rsp] pkt: ");
	return len;
}

u16
_parse_qm_attr_port(struct phl_info_t *phl, u8 *pkt, u16 len, struct rtw_phl_qos_mg_ele *qm)
{
	u16 rlen = 0;
	u8 info_len = 0;

	qm->port_attr_valid = true;
	rlen = ATTR_ID_SZ;
	info_len = GET_ELE_LEN(pkt + rlen);
	rlen += LEN_SZ;

	qm->start_port = GET_PORT_RNG_ATTR_PORT(pkt + rlen);
	rlen += PORT_SZ;
	qm->end_port = GET_PORT_RNG_ATTR_PORT(pkt + rlen);
	rlen += PORT_SZ;
	return rlen;
}

u16
_parse_qm_attr_dscp(struct phl_info_t *phl, u8 *pkt, u16 len, struct rtw_phl_qos_mg_ele *qm)
{
	u16 rlen = 0;
	u8 info_len = 0;

	qm->dscp_attr_valid = true;
	rlen = ATTR_ID_SZ;
	info_len = GET_ELE_LEN(pkt + rlen);
	rlen += LEN_SZ;

	qm->dscp_pol_id = GET_DSCP_POL_ID(pkt + rlen);
	rlen += DSCP_POL_ID_SZ;
	qm->req_tp = GET_DSCP_POL_REQ_TP(pkt + rlen);
	rlen += DSCP_REQ_TP_SZ;
	qm->dscp = GET_DSCP_POL_DSCP(pkt + rlen);
	rlen += DSCP_SZ;
	return rlen;
}

u16
_parse_qm_attr_tclas(struct phl_info_t *phl, u8 *pkt, u16 len, struct rtw_phl_qos_mg_ele *qm)
{
	u16 rlen = 0;
	u8 clf_len = 0;

	qm->tclas_attr_valid = true;
	rlen = ATTR_ID_SZ;
	clf_len = GET_ELE_LEN(pkt + rlen);
	rlen += LEN_SZ;

	qm->clf.tp = GET_CLF_TP(pkt + rlen);
	if (qm->clf.tp == 4) {
		rlen += _parse_clf_tp4(phl, pkt + rlen, len - rlen, &qm->clf.u.t4);
	} else if (qm->clf.tp == 10) {
		rlen += _parse_clf_tp10(phl, pkt + rlen, len - rlen, clf_len,
					&qm->clf.u.t10);
	} else {
		rlen = 0;
	}
	return rlen;
}

u16
_parse_qm_attr_domain(struct phl_info_t *phl, u8 *pkt, u16 len, struct rtw_phl_qos_mg_ele *qm)
{
	void *d = phl_to_drvpriv(phl);
	u16 rlen = 0;

	qm->dom_attr_valid = true;
	rlen = ATTR_ID_SZ;
	qm->dom_len = GET_ELE_LEN(pkt + rlen);
	rlen += LEN_SZ;

	if (qm->dom_len > MAX_DOM_NAME_LEN) {
		_os_mem_cpy(d, qm->dom_name, pkt + rlen, MAX_DOM_NAME_LEN);
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_, "[QoSMg]%s: dom_len(%d) > MAX_DOM_NAME_LEN(%d), only store partial domain\n",
			__FUNCTION__, qm->dom_len, MAX_DOM_NAME_LEN);
	} else {
		_os_mem_cpy(d, qm->dom_name, pkt + rlen, qm->dom_len);
	}
	rlen += qm->dom_len;
	return rlen;
}

u16
rtw_phl_parse_qm_ele(void *phl, u8 *pkt, u16 len,
			struct rtw_phl_qos_mg_ele *qm)
{
	struct phl_info_t *phl_i = (struct phl_info_t *)phl;
	void *d = phl_to_drvpriv(phl_i);
	u16 rlen = 0;
	u8 qm_len = 0, all_attr_len = 0, r_attr_len = 0, attr_len = 0, attr = 0;
	u8 cnt = 0, max_attr_cnt = 85; /* (255/3) */
	u8 oui_tp[4] = {0};

	if (EID_VENDOR_SPECIFIC != GET_ELE_ID(pkt)) {
		goto _exit;
	}
	rlen = ELE_SZ;
	qm_len = GET_ELE_LEN(pkt + rlen);
	all_attr_len = qm_len - OUI_TP_SZ;
	rlen += LEN_SZ;
	GET_OUI_AND_TYPE(pkt + rlen, oui_tp);
	if (0 != _os_mem_cmp(d, oui_tp, WFA_OUI_QM_ELE, OUI_TP_SZ)) {
		goto _exit;
	}
	rlen += OUI_TP_SZ;
	if ((len - rlen) < all_attr_len) {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_, "[QoSMg]%s: (len(%d) - rlen(%d) < all_attr_len(%d) \n",
			__FUNCTION__, len, rlen, all_attr_len);
		goto _exit;
	}
	while (r_attr_len < all_attr_len) {
		attr = GET_QM_ATTR_ID(pkt + rlen);
		if (QM_ATTR_PORT== attr) {
			attr_len = (u8)_parse_qm_attr_port(phl, pkt + rlen,
							len - rlen, qm);
		} else if (QM_ATTR_PORT == attr) {
			attr_len = (u8)_parse_qm_attr_dscp(phl, pkt + rlen,
							len - rlen, qm);
		} else if (QM_ATTR_PORT == attr) {
			attr_len = (u8)_parse_qm_attr_tclas(phl, pkt + rlen,
							len - rlen, qm);
		} else if (QM_ATTR_PORT == attr) {
			attr_len = (u8)_parse_qm_attr_domain(phl, pkt + rlen,
								len - rlen, qm);
		} else {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_, "[QoSMg]%s: Unknown attr id(%d) \n",
			__FUNCTION__, attr);
			goto _exit;
		}
		r_attr_len += attr_len;
		rlen += attr_len;
		cnt++;
		if (cnt == max_attr_cnt) {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_,
				"[QoSMg]%s: r_attr_len(%d), all_attr_len(%d), pleacs check ie len, some error.\n",
				__FUNCTION__, r_attr_len, all_attr_len);
			goto _exit;

		}
	}
_exit:
	return rlen;
}

/*
 * DSCP Policy Query Action frame
 * @dt: dialog token
 */
u16
rtw_phl_parse_dscp_pol_qur(void *phl, u8 *pkt, u16 len,
			struct rtw_phl_dscp_pol_qur *qur)
{
	u16 rlen = 0;

	rlen = ACTION_CAT_SZ + OUI_TP_SZ + QM_OUI_SUBTP_SZ;
	qur->dt = GET_ACT_DT(pkt + rlen);
	rlen += ACTION_TOKEN_SZ;

	qur->qm_ele_num = 0;
	while ((len - rlen) > (ELE_SZ + LEN_SZ + OUI_TP_SZ)) {
		if (qur->qm_ele_num >= MAX_QOS_MG_ELE_NUM) {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_,
				"[QoSMg]%s: qm_ele_num(%d) >= MAX_QOS_MG_ELE_NUM(%d),len(%d), rlen(%d)\n",
				__FUNCTION__, qur->qm_ele_num,
				MAX_QOS_MG_ELE_NUM, len, rlen);
			goto _exit;
		}
		rlen += rtw_phl_parse_qm_ele(phl, pkt + rlen, len - rlen,
					&qur->qm_ele[qur->qm_ele_num]);
		qur->qm_ele_num++;
	}
_exit:
	return rlen;
}

/*
 * DSCP Policy Request Action frame
 * @dt: dialog token
 */
u16
rtw_phl_parse_dscp_pol_req(void *phl, u8 *pkt, u16 len,
				struct rtw_phl_dscp_pol_req *req)
{
	u16 rlen = 0;

	rlen = ACTION_CAT_SZ + OUI_TP_SZ + QM_OUI_SUBTP_SZ;
	req->dt = GET_ACT_DT(pkt + rlen);
	rlen += ACTION_TOKEN_SZ;

	req->ctrl.more = GET_DSCP_REQ_CTRL_MORE(pkt + rlen);
	req->ctrl.reset = GET_DSCP_REQ_CTRL_RESET(pkt + rlen);
	rlen += DSCP_REQ_CTRL_SZ;
	req->qm_ele_num = 0;

	while ((len - rlen) > (ELE_SZ + LEN_SZ + OUI_TP_SZ)) {
		if (req->qm_ele_num >= MAX_QOS_MG_ELE_NUM) {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_,
				"[QoSMg]%s: qm_ele_num(%d) >= MAX_QOS_MG_ELE_NUM(%d),len(%d), rlen(%d)\n",
				__FUNCTION__, req->qm_ele_num,
				MAX_QOS_MG_ELE_NUM, len, rlen);
			goto _exit;
		}
		rlen += rtw_phl_parse_qm_ele(phl, pkt + rlen, len - rlen,
					&req->qm_ele[req->qm_ele_num]);
		req->qm_ele_num++;
	}
_exit:
	return rlen;
}

/*
 * DSCP Policy Response Action frame
 * @dt: dialog token
 */
u16
rtw_phl_parse_dscp_pol_rsp(void *phl, u8 *pkt, u16 len,
			struct rtw_phl_dscp_pol_rsp *rsp)
{
	u8 idx = 0;
	u16 rlen = 0;

	rlen = ACTION_CAT_SZ + OUI_TP_SZ + QM_OUI_SUBTP_SZ;
	rsp->dt = GET_ACT_DT(pkt + rlen);
	rlen += ACTION_TOKEN_SZ;

	rsp->ctrl.more = GET_DSCP_RSP_CTRL_MORE(pkt + rlen);
	rsp->ctrl.reset = GET_DSCP_RSP_CTRL_RESET(pkt + rlen);
	rlen += DSCP_RSP_CTRL_SZ;
	rsp->cnt = GET_DSCP_POL_STS_CNT(pkt + rlen);
	rlen += DSCP_POL_STS_CNT_SZ;
	for (idx = 0; idx < rsp->cnt; idx++) {
		rsp->sts[idx].id = GET_DSCP_POL_ID(pkt + rlen);
		rlen += DSCP_POL_ID_SZ;
		rsp->sts[idx].sts = GET_DSCP_POL_STS(pkt + rlen);
		rlen += DSCP_POL_STS_SZ;
	}
	return rlen;
}

#ifdef CONFIG_DRV_HDL_QM_POL

static void
_dump_all_rule(struct qm_ruleset *ruleset,
		const char *caller)
{
	struct qm_rule_i *rule = NULL;
	u8 idx = 0;

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"######### %s: %s: ##########\n",
		caller, __FUNCTION__);
	for (idx = 0; idx < MAX_QM_RULE; idx++) {
		rule = &ruleset->rule[idx];
		if (!rule->tp)
			continue;
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
			"%s: Idx[%d], TP(%d), Rank(%d)\n",
			__FUNCTION__, idx, rule->tp, rule->rank);
		if (QM_RULE_DSCP_POL == rule->tp) {
			_dump_qm_ele(&rule->info.qm_ele, __FUNCTION__);
		}
	}
}

struct qm_rule_i *
_get_spec_dscp_rule(struct phl_info_t *phl, struct qm_ruleset *ruleset,
		u8 id)
{
	struct qm_rule_i *rule = NULL, *tgt_rule = NULL;
	u8 idx = 0;

	for (idx = 0; idx < MAX_QM_RULE; idx++) {
		rule = &ruleset->rule[idx];
		if (QM_RULE_DSCP_POL != rule->tp)
			continue;
		if (rule->info.qm_ele.dscp_attr_valid &&
		    rule->info.qm_ele.dscp_pol_id == id) {
			tgt_rule = rule;
		}
	}
	return tgt_rule;
}

static void
_free_rule(struct phl_info_t *phl, struct qm_rule_i *rule)
{
	void *priv = phl_to_drvpriv(phl);

	_os_mem_set(priv, rule, 0, sizeof(struct qm_rule_i));
}

struct qm_rule_i *
_get_new_rule(struct phl_info_t *phl, struct qm_ruleset *ruleset,
		enum rule_type tp)
{
	struct qm_rule_i *rule = NULL, *tgt_rule = NULL;
	u8 idx = 0;

	for (idx = 0; idx < MAX_QM_RULE; idx++) {
		rule = &ruleset->rule[idx];
		if (!rule->tp) {
			rule->tp = tp;
			tgt_rule = rule;
			break;
		}
	}
	return tgt_rule;
}

static void
_dump_all_ruleset(struct phl_info_t *phl, const char *caller)
{
	struct qm_ctrl_info *ctrl = get_qm_ctrl(phl);
	struct qm_ruleset *ruleset = NULL;
	u8 idx = 0;

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"######### %s: %s: ##########\n", caller, __FUNCTION__);

	for (idx = 0; idx < MAX_QM_RULE_ROLE; idx++) {
		ruleset = &ctrl->ruleset[idx];
		if (NULL == ruleset->wr)
			continue;
		_dump_all_rule(ruleset, caller);
	}
}

static void
_free_ruleset(struct phl_info_t *phl, struct qm_ruleset *ruleset)
{
	void *priv = phl_to_drvpriv(phl);

	_os_mem_set(priv, ruleset, 0, sizeof(struct qm_ruleset));
}

struct qm_ruleset *
_get_new_ruleset(struct phl_info_t *phl, struct rtw_wifi_role_t *wr)
{
	struct qm_ctrl_info *ctrl = get_qm_ctrl(phl);
	struct qm_ruleset *ruleset = NULL, *tgt_ruleset = NULL;
	u8 idx = 0;

	for (idx = 0; idx < MAX_QM_RULE_ROLE; idx++) {
		ruleset = &ctrl->ruleset[idx];
		if (NULL == ruleset->wr) {
			ruleset->wr = wr;
			tgt_ruleset = ruleset;
			break;
		}
	}
	return tgt_ruleset;
}

struct qm_ruleset *
_get_wr_ruleset(struct phl_info_t *phl, struct rtw_wifi_role_t *wr)
{
	struct qm_ctrl_info *ctrl = get_qm_ctrl(phl);
	struct qm_ruleset *ruleset = NULL, *tgt_ruleset = NULL;
	u8 idx = 0;

	for (idx = 0; idx < MAX_QM_RULE_ROLE; idx++) {
		ruleset = &ctrl->ruleset[idx];
		if (ruleset->wr == wr) {
			tgt_ruleset = ruleset;
			break;
		}
	}
	return tgt_ruleset;
}

static u8
_dscp2tid_def( u8 dscp)
{
	u8 tid = 0;

	if (dscp == 0x08)
		tid = 1;
	else if (dscp <= 0x10)
		tid = 0;
	else if (dscp <= 0x16)
		tid = 3;
	else if (dscp <= 0x26)
		tid = 4;
	else if (dscp <= 0x28)
		tid = 5;
	else if (dscp <= 0x2e)
		tid = 6;
	else
		tid = 7;
	return tid;
}

static bool
_match_tclas(struct phl_info_t *phl, u8 *pkt, u16 len,
		struct rtw_phl_tclas_clf *clf)
{
	bool ret = false;

	return ret;
}

static bool
_match_port_range(struct phl_info_t *phl, u8 *pkt, u16 len,
		u16 start_port, u16 end_port)
{
	bool ret = false;

	return ret;
}



static bool
_match_dom(struct phl_info_t *phl, u8 *pkt, u16 len,
		struct rtw_phl_qos_mg_ele *qm_ele)
{
	bool ret = false;

	return ret;
}

static bool
_match_dscp_pol(struct phl_info_t *phl, u8 *pkt, u16 len,
		struct rtw_phl_qos_mg_ele *qm_ele, u8 *dscp, u8 *tid)
{
	bool ret = false;

	if (!qm_ele->dscp_attr_valid)
		goto _exit;
	if (qm_ele->port_attr_valid) {
		ret = _match_port_range(phl, pkt, len, qm_ele->start_port,
					qm_ele->end_port);
		if (!ret) {
			goto _exit;
		}
	}
	if (qm_ele->dom_attr_valid) {
		ret = _match_dom(phl, pkt, len, qm_ele);
		if (!ret) {
			goto _exit;
		}
	}
	if (qm_ele->tclas_attr_valid) {
		ret = _match_tclas(phl, pkt, len, &qm_ele->clf);
		if (!ret) {
			goto _exit;
		}
	}
	if (!ret)
		goto _exit;
	*dscp =  qm_ele->dscp;
	*tid = _dscp2tid_def(*dscp);
	ret = true;
	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
		"%s:Match policy id(%d), dscp(%d), tid(%d)\n",
		__FUNCTION__, qm_ele->dscp_pol_id, *dscp, *tid);
_exit:
	return ret;
}

static u8
_count_t4_req(u8 mask)
{
	u8 idx = 0, cnt = 0;

	/* Bit0 is (IP) Version field and no need to counted */
	for (idx = 1; idx < 8; idx++) {
		if (mask & BIT(idx))
			cnt++;
	}
	return cnt;
}

static u8
_count_clf_req(struct rtw_phl_tclas_clf *clf)
{
	u8 cnt = 0;

	if (4 == clf->tp) {
		struct rtw_phl_tclas_type4 *t4 = &clf->u.t4;

		cnt += _count_t4_req(t4->mask);
	}
	return cnt;
}

static u8
_count_dscp_pol_req(struct rtw_phl_qos_mg_ele *qm_ele)
{
	u8 cnt = 0;

	if (qm_ele->port_attr_valid)
		cnt++;
	if (qm_ele->dom_attr_valid)
		cnt++;
	if (qm_ele->tclas_attr_valid)
		cnt += _count_clf_req(&qm_ele->clf);
	return cnt;
}


static void
_rank_rule(struct phl_info_t *phl, struct qm_ruleset *ruleset)
{
	void *d = phl_to_drvpriv(phl);
	struct qm_rule_i *rule = NULL;
	struct qm_rule_i tmp = {0};
	u8 idx = 0, jdx = 0;

	for (idx = 0; idx < MAX_QM_RULE; idx++) {
		rule = &ruleset->rule[idx];
		if (!rule->tp)
			continue;
		if (QM_RULE_DSCP_POL == rule->tp)
			rule->rank = _count_dscp_pol_req(&rule->info.qm_ele);
	}
	for (idx = 0; idx < MAX_QM_RULE; idx++) {
		for (jdx = 1; jdx < MAX_QM_RULE; jdx++) {
			if (ruleset->rule[idx].rank > ruleset->rule[jdx].rank)
				continue;
			_os_mem_cpy(d, &tmp, &ruleset->rule[idx],
					sizeof(struct qm_rule_i));
			_os_mem_cpy(d, &ruleset->rule[idx], &ruleset->rule[jdx],
					sizeof(struct qm_rule_i));
			_os_mem_cpy(d, &ruleset->rule[jdx], &tmp,
					sizeof(struct qm_rule_i));
		}
	}
}

bool
_up_dsco_pol_rule(struct phl_info_t *phl, struct qm_ruleset *ruleset,
		struct rtw_phl_qos_mg_ele *qm_ele)
{
	bool ret = false;
	void *priv = phl_to_drvpriv(phl);
	struct qm_rule_i *rule = NULL;

	rule = _get_spec_dscp_rule(phl, ruleset, qm_ele->dscp_pol_id);
	if (DSCP_POL_REQ_ADD == qm_ele->req_tp) {
		if (rule) {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_WARNING_,
				"[QoSMg]%s: dscp_pol_id(%d) has exsited! Free it first.\n",
				__FUNCTION__, qm_ele->dscp_pol_id);
			_free_rule(phl, rule);
		}
		rule = _get_new_rule(phl, ruleset, QM_RULE_DSCP_POL);
		if (NULL == rule) {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_,
				"[QoSMg]%s: Get new rule fail, wr_id(%d), dscp_pol_id(%d)\n",
				__FUNCTION__, ruleset->wr->id,
				qm_ele->dscp_pol_id);
			goto _exit;
		}
		_os_mem_cpy(priv, &rule->info.qm_ele, qm_ele,
			    sizeof(struct rtw_phl_qos_mg_ele));
		ret = true;
	} else if (DSCP_POL_REQ_UPDATE == qm_ele->req_tp) {
		if (rule) {
			_os_mem_cpy(priv, &rule->info.qm_ele, qm_ele,
				    sizeof(struct rtw_phl_qos_mg_ele));
		} else {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_WARNING_,
				"[QoSMg]%s: dscp_pol_id(%d) don't exsited! expected?\n",
				__FUNCTION__, qm_ele->dscp_pol_id);
		}
		ret = true;
	} else if (DSCP_POL_REQ_REMOVE == qm_ele->req_tp) {
		if (rule) {
			_free_rule(phl, rule);
		} else {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
				"[QoSMg]%s: dscp_pol_id(%d) don't exsited!\n",
				__FUNCTION__, qm_ele->dscp_pol_id);
		}
		ret = true;
	} else {
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_WARNING_,
			"[QoSMg]%s: Unknown req_tp(%d)\n",
			__FUNCTION__, qm_ele->req_tp);
	}
_exit:
	return ret;
}

bool
_up_dscp_pol(struct phl_info_t *phl, struct rtw_wifi_role_t *wr,
		struct rtw_phl_qos_mg_ele *qm_ele)
{
	bool ret = false;
	void *d = phl_to_drvpriv(phl);
	struct qm_ctrl_info *ctrl = get_qm_ctrl(phl);
	struct qm_ruleset *ruleset = NULL;

	_os_spinlock(d, &ctrl->lock, _bh, NULL);

	ruleset = _get_wr_ruleset(phl, wr);
	if (NULL == ruleset) {
		ruleset = _get_new_ruleset(phl, wr);
		if (NULL == ruleset) {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_,
				"[QoSMg]%s: Get Ruleset failed, wr_id(%d)\n",
				__FUNCTION__, wr->id);
			goto _exit;
		}
	}
	ret = _up_dsco_pol_rule(phl, ruleset, qm_ele);
	_rank_rule(phl, ruleset);
_exit:
	_os_spinunlock(d, &ctrl->lock, _bh, NULL);

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_,
		"[QoSMg]%s: ret(%d), wr_id(%d), dscp_attr_valid(%d), dscp_pol_id(%d)\n",
		__FUNCTION__, ret, wr->id, qm_ele->dscp_attr_valid,
		qm_ele->dscp_pol_id);
	return ret;
}

bool
rtw_phl_match_qm_pol(void *phl, u8 *pkt, u16 len, struct rtw_wifi_role_t *wr,
		u8 *dscp, u8 *tid)
{
	bool ret = false;
	struct phl_info_t *phl_i = (struct phl_info_t *)phl;
	struct qm_ctrl_info *ctrl = get_qm_ctrl(phl_i);
	struct qm_ruleset *ruleset = NULL;
	struct qm_rule_i *rule = NULL;
	void *d = phl_to_drvpriv(phl_i);
	u8 idx = 0;

	_os_spinlock(d, &ctrl->lock, _bh, NULL);

	ruleset = _get_wr_ruleset(phl, wr);
	if (NULL == ruleset) {
		goto _exit;
	}

	for (idx = 0; idx < MAX_QM_RULE; idx++) {
		rule = &ruleset->rule[idx];
		if (!rule->tp)
			continue;
		if (QM_RULE_DSCP_POL == rule->tp) {
			ret = _match_dscp_pol(phl, pkt, len, &rule->info.qm_ele,
						dscp, tid);
		} else {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_,
				"%s:Unknown type(%d)\n",
				__FUNCTION__, rule->tp);
		}
		if (ret)
			break;
	}

_exit:
	_os_spinunlock(d, &ctrl->lock, _bh, NULL);
	return ret;
}

bool
rtw_phl_remove_dscp_pol(void *phl, struct rtw_phl_stainfo_t *sta, u8 id)
{
	bool ret = false;
	struct rtw_phl_qos_mg_ele qm_ele = {0};

	qm_ele.dscp_attr_valid = true;
	qm_ele.dscp_pol_id = id;
	qm_ele.req_tp = DSCP_POL_REQ_REMOVE;
	ret = _up_dscp_pol(phl, sta->wrole, &qm_ele);
	_dump_all_ruleset(phl, __FUNCTION__);
	return ret;
}

bool
rtw_phl_hdl_dscp_pol_req(void *phl, struct rtw_phl_stainfo_t *sta,
	struct rtw_phl_dscp_pol_req *req, struct rtw_phl_dscp_pol_rsp *rsp)
{
	bool ret = true;
	u8 idx = 0;

	rsp->dt = req->dt;
	for (idx = 0; idx < req->qm_ele_num; idx++) {
		if (false == req->qm_ele[idx].dscp_attr_valid)
			continue;
		if (_up_dscp_pol(phl, sta->wrole, &req->qm_ele[idx])) {
			rsp->sts[rsp->cnt].sts = DSCP_POL_SUCCESS;
		} else {
			rsp->sts[rsp->cnt].sts = DSCP_POL_REQUEST_DECLINED;
		}
		rsp->sts[rsp->cnt].id = req->qm_ele[idx].dscp_pol_id;
		rsp->cnt++;
	}
	_dump_all_ruleset(phl, __FUNCTION__);
	return ret;
}
#endif /* CONFIG_DRV_HDL_QM_POL */

void phl_qm_deinit(struct phl_info_t *phl)
{
#ifdef CONFIG_DRV_HDL_QM_POL
	void *d = phl_to_drvpriv(phl);

	PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "phl_qm_deinit\n");
	if (phl->qm_ctrl != NULL) {
		struct qm_ctrl_info *ctrl = get_qm_ctrl(phl);

		_os_spinlock_free(d, &ctrl->lock);
		_os_mem_free(d, phl->qm_ctrl, sizeof(struct qm_ctrl_info));
		PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "phl_qm_deinit(): Free qm_ctrl\n");
	}
#endif /* CONFIG_DRV_HDL_QM_POL */
}

enum rtw_phl_status phl_qm_init(struct phl_info_t *phl)
{
	enum rtw_phl_status sts = RTW_PHL_STATUS_SUCCESS;
#ifdef CONFIG_DRV_HDL_QM_POL
	void *d = phl_to_drvpriv(phl);

	if (phl->qm_ctrl == NULL) {
		phl->qm_ctrl = _os_mem_alloc(d, sizeof(struct qm_ctrl_info));
		if (phl->qm_ctrl) {
			struct qm_ctrl_info *ctrl = get_qm_ctrl(phl);

			_os_spinlock_init(d, &ctrl->lock);
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_INFO_, "Allocate qm_ctrl_info\n");
		} else {
			PHL_TRACE(COMP_PHL_QOSMG, _PHL_ERR_, "Allocate qm_ctrl_info Fail\n");
			sts = RTW_PHL_STATUS_FAILURE;
		}
	}
#endif /* CONFIG_DRV_HDL_QM_POL */
	return sts;
}

#endif /* CONFIG_QOS_MG */
