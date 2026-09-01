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
#define _PHL_NAN_C_
#include "phl_headers.h"

#ifdef CONFIG_PHL_NAN

/**
 * _phl_nan_free_mgnt_frame_entry
 *   - free the mem of entry (struct phl_nan_mgnt_frame_entry)
 * @nan_info: context for nan
 * @entry: a send entry contained a nan mgnt frame
 * (ac) _rtw_nan_free_send_mgnt_frame_entry
 */
void _phl_nan_free_mgnt_frame_entry(struct phl_nan_info *nan_info,
				    struct phl_nan_mgnt_frame_entry *entry)
{
	void *d = phl_nan_info_to_drvpriv(nan_info);

	if (entry == NULL)
		return;
	_os_mem_free(d, (void *)entry->nan_attr_buf, entry->attr_len);
	_os_mem_free(d, (void *)entry, sizeof(struct phl_nan_mgnt_frame_entry));
}

/**
 * _phl_nan_free_mgnt_frame_queue - free the mem of the entrie mgnt frame queue
 * @nan_info: context for nan
 * (ac) _rtw_free_nan_mgnt_frame_queue
 */
void _phl_nan_free_mgnt_frame_queue(struct phl_nan_info *nan_info)
{
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_mgnt_frame_entry *p_send_entry = NULL;
	_os_list *obj = NULL;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: cnt %d\n",
		  __func__, nan_info->mgnt_frame_queue.cnt);

	while (pq_pop(d, &nan_info->mgnt_frame_queue, &obj, _first, _bh)) {
		p_send_entry = (struct phl_nan_mgnt_frame_entry *)obj;
		_phl_nan_free_mgnt_frame_entry(nan_info, p_send_entry);
	}
}

void phl_nan_dump_bcn_pkt(struct phl_nan_bcn nan_bcn_info)
{
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "======= NAN PKT DUMP =======\n");

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "nan sync bcn: interval(%d), len(%d)\n",
		  nan_bcn_info.sync_bcn_intvl_ms,
		  nan_bcn_info.nan_sync_bcn.bcn_length);

	phl_nan_debug_dump_data(nan_bcn_info.nan_sync_bcn.bcn_buf,
				nan_bcn_info.nan_sync_bcn.bcn_length,
				"sync bcn buf");

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "nan disc bcn: interval(%d), len(%d)\n",
		  nan_bcn_info.disc_bcn_intvl_ms,
		  nan_bcn_info.nan_disc_bcn.bcn_length);

	phl_nan_debug_dump_data(nan_bcn_info.nan_disc_bcn.bcn_buf,
				nan_bcn_info.nan_disc_bcn.bcn_length,
				"disc bcn buf");

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "==========================\n");
}

void phl_nan_debug_dump_data(u8 *buf, u32 buf_len, const char *prefix)
{
	u32 i;

	if (buf == NULL) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "[debug dump] buf is NULL\n");
		return;
	}

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "[debug dump] %s\n", prefix);
	for (i = 0; i < buf_len; i++) {
		if (!(i % 8))
			PHL_DATA(COMP_PHL_NAN, _PHL_INFO_, "\n");
		PHL_DATA(COMP_PHL_NAN, _PHL_INFO_, "%02X ", buf[i]);
	}
	PHL_DATA(COMP_PHL_NAN, _PHL_INFO_, "\n");
}

struct phl_nan_info *
phl_nan_get_info(enum _nan_engine_type type, void *p_engine)
{
	struct phl_nan_info *nan_info = NULL;

	switch (type) {
	case NAN_EN_SYNC:
		nan_info = (struct phl_nan_info *)
			    phl_container_of(p_engine,
					     struct phl_nan_info,
					     sync_engine_priv);
		break;
	case NAN_EN_DISC:
		nan_info = (struct phl_nan_info *)
			    phl_container_of(p_engine,
					     struct phl_nan_info,
					     discovery_engine_priv);
		break;
#ifdef CONFIG_NAN_R2
	case NAN_EN_DATA:
		nan_info = (struct phl_nan_info *)
				phl_container_of(p_engine,
						struct phl_nan_info,
						data_engine_priv);
		break;
	case NAN_EN_SCHD:
		nan_info = (struct phl_nan_info *)
				phl_container_of(p_engine,
						 struct phl_nan_info,
						 scheduler_priv);
		break;
#endif
	default:
		break;
	}

	return nan_info;
}

/**
 * _phl_nan_append_connection_cap - Append the Connection Capability attribute
 * @d:
 * @pbuf: frame buffer
 * (ac) _append_nan_connection_cap
 *
 * RETURN:
 * Length of dca
 */
u16 _phl_nan_append_connection_cap(void *d, u8 *pbuf)
{
	u8 *ie_start = pbuf + CONNECTION_CAP_FIELD_START_OFFSET;
	u16 attr_len = 0;

	/* Wi-Fi Direct */
	SET_NAN_CONNECTION_CAP_ELE_WIFI_DIRECT(ie_start, 0x0);
	/* P2Ps */
	SET_NAN_CONNECTION_CAP_ELE_P2PS(ie_start, 0x0);
	/* TDLS */
	SET_NAN_CONNECTION_CAP_ELE_TDLS(ie_start, 0x0);
	/* WLAN Infrastructure */
	SET_NAN_CONNECTION_CAP_ELE_INFRA(ie_start, 0x0);
	/* IBSS */
	SET_NAN_CONNECTION_CAP_ELE_IBSS(ie_start, 0x0);
	/* Mesh */
	SET_NAN_CONNECTION_CAP_ELE_MESH(ie_start, 0x0);
	attr_len += CONNECTION_CAP_FIELD_BITMAP_LEN;

	return attr_len;
}

u16 phl_nan_append_ele_container(void *d, struct phl_nan_info *nan_info, u8 *pbuf)
{
	u16 attr_len = 0;
	struct rtw_phl_nan_ie_container *ies = &nan_info->nan_ies;

	/* Map ID */
	SET_NAN_DEVICE_CAP_ELE_MAPID(pbuf, 0);
	attr_len += DEVICE_CAP_MAPID_LEN;

	/* Append Support Rate */
	_os_mem_cpy(d, pbuf + attr_len, ies->ie_s_rate, ies->ie_s_rate_len);
	attr_len += ies->ie_s_rate_len;

	/* Append Ext Support Rate */
	if (ies->ie_es_rate_len) {
		_os_mem_cpy(d, pbuf + attr_len, ies->ie_es_rate,
			    ies->ie_es_rate_len);
		attr_len += ies->ie_es_rate_len;
	}

	/* Append HT Capability */
	_os_mem_cpy(d, pbuf + attr_len, ies->ie_ht, ies->ie_ht_len);
	attr_len += ies->ie_ht_len;

	/* Append VHT Capability */
	_os_mem_cpy(d, pbuf + attr_len, ies->ie_vht, ies->ie_vht_len);
	attr_len += ies->ie_vht_len;

	return attr_len;
}

/**
 * phl_nan_append_device_cap - Append the Device Capability attribute
 * @d:
 * @pbuf: frame buffer
 * @sync_info:
 * (ac) _append_nan_device_cap
 *
 * RETURN:
 * Length of dca
 */
u16 phl_nan_append_device_cap(void *d, u8 *pbuf,
			      struct phl_nan_sync_priv *sync_info)
{
	u8 dw_cnt_2g = 0, dw_cnt_5g = 0;
	u8 *ie_start = pbuf + DEVICE_CAP_FIELD_START_OFFSET;
	u16 attr_len = 0;
	u8 cap = 0;

	dw_cnt_2g = (u8)(sync_info->cmt_dw_peri_24G * NAN_SLOT / DW_INTERVAL);
	dw_cnt_5g = (u8)(sync_info->cmt_dw_peri_5G * NAN_SLOT / DW_INTERVAL);

	/* Map ID */
	SET_NAN_DEVICE_CAP_ELE_MAPID(ie_start, 0);
	/* SET_NAN_DEVICE_CAP_ELE_MAPID(pIeStart, ((DEFAULT_MAP_ID << 1) | BIT0)); */

	attr_len += DEVICE_CAP_MAPID_LEN;

	/* Committed DW Info */
	SET_NAN_DEVICE_CAP_ELE_24G_DW(ie_start, dw_cnt_2g);
	SET_NAN_DEVICE_CAP_ELE_24G_DW_OVERWRITE(ie_start, DEFAULT_MAP_ID);
	SET_NAN_DEVICE_CAP_ELE_5G_DW_OVERWRITE(ie_start, DEFAULT_MAP_ID);
	attr_len += DEVICE_CAP_COMMITTED_DW_INFO_LEN;

	/* Supported Bands */
	SET_NAN_DEVICE_CAP_ELE_SUPPORT_BAND_24G(ie_start, 0x1);
	if (sync_info->dual_band_support & BAND_CAP_5G) {
		SET_NAN_DEVICE_CAP_ELE_SUPPORT_BAND_5G(ie_start, 0x1);
		SET_NAN_DEVICE_CAP_ELE_5G_DW(ie_start, dw_cnt_5g);
	} else {
		SET_NAN_DEVICE_CAP_ELE_SUPPORT_BAND_5G(ie_start, 0x0);
		SET_NAN_DEVICE_CAP_ELE_5G_DW(ie_start, 0x0);
	}
	attr_len += DEVICE_CAP_SUPPORT_BANDS_LEN;

	/* Operation Mode */
	if (TEST_STATUS_FLAG(sync_info->flag, NAN_MAC_FLAG_VHT_ENABLE)) {
		SET_NAN_DEVICE_CAP_ELE_OPERATION_PHY_MODE(ie_start, 0x1);
	} else {
		SET_NAN_DEVICE_CAP_ELE_OPERATION_PHY_MODE(ie_start, 0x0);
	}
	attr_len += DEVICE_CAP_OPERATION_MODE_LEN;

	/* Number of Antennas */
	/* Bit 0 -3: Number of TX antennas */
	/* Bit 4-7: Number of RX antennas */
	/* Value 0 indicates the information is not available */
	SET_NAN_DEVICE_CAP_ELE_ANTENNA_NUM_TX(ie_start, 0x0);
	SET_NAN_DEVICE_CAP_ELE_ANTENNA_NUM_RX(ie_start, 0x0);
	attr_len += DEVICE_CAP_NUMBER_OF_ANTENNAS_LEN;

	/* Max Channel Switch Time */
	/* Value 0 indicates the information is not available */
	SET_NAN_DEVICE_CAP_ELE_MAX_CHANNEL_SWITCH_TIME(ie_start,
						       MAX_CHANNEL_SWITCH_TIME);
	attr_len += DEVICE_CAP_MAX_CHANNEL_SWITCH_TIME_LEN;

	/* Capabilities */
	SET_NDPE_SUPPORT_IN_CAPABILITIES(cap, 1);
	SET_NAN_DEVICE_CAP_ELE_CAPABILITIES(ie_start, cap);
	attr_len += DEVICE_CAP_CAPABILITIES_LEN;

	return attr_len;
}

/**
 * _phl_nan_append_sda_ext - Append the Service Descriptor Extension attribute
 * @d:
 * @pbuf: frame buffer
 * @psdf_ctx:  content for issuing a sdf
 * (ac) _append_nan_sda_ext
 *
 * RETURN:
 * Length of dca
 */
u16 _phl_nan_append_sda_ext(void *d, u8 *pbuf, struct phl_nan_sdf_ctx *psdf_ctx)
{
	u16 attr_len = 0;
	u8 *ie_start = pbuf + SERVICE_DESC_EXT_FIELD_START_OFFSET;
	u8 inst_id = 0;
	struct rtw_phl_nan_srvc_instance_entry *p_srvc_entry;
	struct rtw_phl_nan_service_info *p_si = NULL;

	p_srvc_entry = psdf_ctx->p_srvc_entry;
	inst_id = (phl_nan_is_followup_srvc_inst(p_srvc_entry)) ?
			  phl_nan_get_inid_from_entry(p_srvc_entry) :
			  psdf_ctx->instance_id;

	/* Instance ID */
	SET_NAN_SERVICE_DESC_EXT_ELE_INSTANCE_ID(ie_start, inst_id);
	attr_len += SERVICE_DESC_EXT_FIELD_INSTANCE_ID_LEN;

	/* Control */
	SET_NAN_SERVICE_DESC_EXT_ELE_CONTROL(
		ie_start,
		phl_nan_get_sde_control_from_srv_instance(p_srvc_entry));
	attr_len += SERVICE_DESC_EXT_FIELD_CONTROL_LEN;

	/* Range Limit */
	/* Service Update Indicator */
	/* Service Info Length */
	/* Service Info */
	p_si = phl_nan_get_si_from_entry(psdf_ctx->p_srvc_entry);
	if ((psdf_ctx->service_control & BIT_SCF_SI_PRESENT) && p_si) {
		*(ie_start + attr_len) = (u8)(p_si->len);
		*(ie_start + attr_len + 1) = (u8)((p_si->len) >> 8);
		_os_mem_cpy(d,
			    (ie_start + attr_len +
			     SERVICE_DESC_EXT_FIELD_SERVICE_INFO_LEN),
			    p_si->info, p_si->len);
		attr_len +=
			(p_si->len + SERVICE_DESC_EXT_FIELD_SERVICE_INFO_LEN);
	}

	return attr_len;
}

/**
 * _phl_nan_append_sda
 *  - Append the service descriptor attribute according to nan_sdf_ctx
 * @d:
 * @pbuf: frame buffer
 * @psdf_ctx: content for issuing a sdf
 * (ac) _append_nan_sda
 *
 * RETURN:
 * Length of sda
 */
u16 _phl_nan_append_sda(void *d, u8 *pbuf, struct phl_nan_sdf_ctx *psdf_ctx)
{
	u16 attr_len = 0;
	u8 i = 0;
	u8 mf_len = 0, mf_num = 0;
	u16 mf_len_offset = 0, mf_offset = 0;
	struct rtw_phl_nan_func_filter *p_mf = NULL;
	u8 srf_len = 0, srf_control = 0;
	u16 srf_len_offset = 0, srf_offset = 0;
	struct rtw_phl_nan_subscribe_info *p_sub_info = NULL;
	u16 si_len_offset = 0, si_offset = 0;
	struct rtw_phl_nan_service_info *p_si = NULL;
	u8 inst_id = 0, req_id = 0;
	struct rtw_phl_nan_followup_info *f_data =
		phl_nan_get_followup_from_entry(psdf_ctx->p_srvc_entry);

	_os_mem_cpy(d, (pbuf + SDA_SERVICE_ID_OFFSET),
		    phl_nan_get_srv_id_from_entry(psdf_ctx->p_srvc_entry),
		    SDA_SERVICE_ID_LEN);
	attr_len += SDA_SERVICE_ID_LEN;

	if (f_data) {
		inst_id = f_data->followup_id;
		req_id = f_data->followup_reqid;
	} else {
		inst_id = psdf_ctx->instance_id;
		req_id = psdf_ctx->requestor_id;
	}

	/* Instance ID */
	_os_mem_cpy(d, (pbuf + SDA_INSTANCE_ID_OFFSET), &inst_id,
		    SDA_INSTANCE_ID_LEN);
	attr_len += SDA_INSTANCE_ID_LEN;

	/* Requestor Instance ID */
	_os_mem_cpy(d, (pbuf + SDA_REQUESTOR_INSTANCE_ID_OFFSET), &req_id,
		    SDA_REQUESTOR_INSTANCE_ID_LEN);
	attr_len += SDA_REQUESTOR_INSTANCE_ID_LEN;

	/* Service control */
	_os_mem_cpy(d, (pbuf + SDA_SERVICE_CONTROL_OFFSET),
		    &psdf_ctx->service_control, SDA_SERVICE_CONTROL_LEN);
	attr_len += SDA_SERVICE_CONTROL_LEN;

	/* binding bitmap */
	if (psdf_ctx->service_control & BIT_SCF_BB_PRESENT) {
		_os_mem_cpy(d, (pbuf + SDA_OPTIONAL_FIELD_OFFSET),
			    &psdf_ctx->binding_bitmap, SDA_BINDING_BITMAP_LEN);
		attr_len += SDA_BINDING_BITMAP_LEN;
	}

	/* tx matching filter */
	p_mf = phl_nan_get_tx_mf_from_entry(psdf_ctx->p_srvc_entry, &mf_num);
	if ((psdf_ctx->service_control & BIT_SCF_MF_PRESENT) && p_mf) {
		mf_len_offset = SDA_FIELD_START_OFFSET + attr_len;
		mf_offset = mf_len_offset + 1;

		for (i = 0; i < mf_num; i++) {
			_os_mem_cpy(d, (pbuf + mf_offset), &(p_mf[i].len), 1);
			mf_offset += 1;
			mf_len += 1;
			_os_mem_cpy(d, (pbuf + mf_offset), p_mf[i].filter,
				    p_mf[i].len);
			mf_offset += p_mf[i].len;
			mf_len += p_mf[i].len;
		}
		_os_mem_cpy(d, (pbuf + mf_len_offset), &mf_len, 1);
		attr_len = attr_len + 1 + mf_len;
	}

	/* service response filter : mac addr or bloom filter  */
	if ((psdf_ctx->service_control & BIT_SCF_SRF_PRESENT) &&
	    phl_nan_is_sub_srvc_inst(psdf_ctx->p_srvc_entry)) {
		p_sub_info = phl_nan_get_sub_from_entry(psdf_ctx->p_srvc_entry);
		srf_len_offset = SDA_FIELD_START_OFFSET + attr_len;
		srf_offset = srf_len_offset + 1;
		if (p_sub_info->srf_bf_len != 0) {
			srf_control |= BIT_SRF_CTRL_TYPE;
			srf_control |= ((p_sub_info->srf_bf_idx << SRF_CTRL_BFI_SHIFT) &
					SRF_CTRL_BFI_MASK);
		}
		if (p_sub_info->srf_include)
			srf_control |= BIT_SRF_CTRL_INCLUDE;
		_os_mem_cpy(d, (pbuf + srf_offset), &srf_control, 1);
		srf_offset += 1;
		srf_len += 1;

		if (p_sub_info->srf_num_macs != 0) {
			for (i = 0; i < p_sub_info->srf_num_macs; i++) {
				_os_mem_cpy(d, (pbuf + srf_offset),
					    p_sub_info->srf_macs[i].mac_addr,
					    ETH_ALEN);
				srf_offset += ETH_ALEN;
				srf_len += ETH_ALEN;
			}
		} else {
			_os_mem_cpy(d, (pbuf + srf_offset),
				    p_sub_info->srf_bf,
				    p_sub_info->srf_bf_len);
			srf_offset += p_sub_info->srf_bf_len;
			srf_len += p_sub_info->srf_bf_len;
		}

		_os_mem_cpy(d, (pbuf + srf_len_offset), &srf_len, 1);
		attr_len = attr_len + 1 + srf_len;
	}

	/*
	 * service info
	 * If the OUI subfield is set to 0x50-6F-9A (Wi-Fi Alliance specific OUI),
	 * the Service Protocol Type subfield is included subsequently, append
	 * service info in SDEA
	 */
	p_si = phl_nan_get_si_from_entry(psdf_ctx->p_srvc_entry);
	if ((psdf_ctx->service_control & BIT_SCF_SI_PRESENT) && p_si &&
	    (p_si->len <= MAX_SDA_SRVC_INFO_LEN) &&
	    (!phl_nan_is_start_with_wfa_oui(d, p_si->info))) {
		si_len_offset = SDA_FIELD_START_OFFSET + attr_len;
		si_offset = si_len_offset + 1;

		_os_mem_cpy(d, (pbuf + si_len_offset), &p_si->len, 1);
		_os_mem_cpy(d, (pbuf + si_offset), p_si->info, p_si->len);
		attr_len = attr_len + 1 + p_si->len;
	} else {
		*(pbuf + SDA_SERVICE_CONTROL_OFFSET) &= ~(BIT_SCF_SI_PRESENT);
	}
	return attr_len;
}

u16
phl_nan_append_attr(void *d,
		    struct phl_nan_mac_info *mac_info,
		    u8 *pbuf,
		    u8 attr_id,
		    void *pdata)
{
	struct phl_nan_sync_priv *sync_info = (struct phl_nan_sync_priv *)
	    phl_container_of(mac_info, struct phl_nan_sync_priv, mac_info);
#ifdef CONFIG_NAN_R2
	struct phl_nan_info *nan_info = phl_nan_get_info(NAN_EN_SYNC, sync_info);
#endif
	u16 attr_len = 0;

	switch (attr_id) {
	case NAN_ATTR_ID_MASTER_INDICATION:
		*pbuf = NAN_ATTR_ID_MASTER_INDICATION;
		attr_len = 2;
		_os_mem_cpy(d, (pbuf + 1), &attr_len, 2);
		*(pbuf + 3) = sync_info->master_indication.master_pref;
		*(pbuf + 4) = sync_info->master_indication.random_factor;
		attr_len += NAN_ATTR_CONTENT_OFFSET;
		break;
	case NAN_ATTR_ID_CLUSTER:
		*pbuf = NAN_ATTR_ID_CLUSTER;
		attr_len = 13;
		_os_mem_cpy(d, (pbuf + 1), &attr_len, 2);
		/* Anchor Master Information */
		_os_mem_set(d, (pbuf + 3), 0, 13);
		attr_len += NAN_ATTR_CONTENT_OFFSET;
		break;
	case NAN_ATTR_ID_SERVICE_DESCRIPTOR:
		*pbuf = NAN_ATTR_ID_SERVICE_DESCRIPTOR;
		attr_len = _phl_nan_append_sda(d, pbuf,
					       (struct phl_nan_sdf_ctx *)pdata);
		_os_mem_cpy(d, (pbuf + 1), &attr_len, 2);
		attr_len += NAN_ATTR_CONTENT_OFFSET;
		break;
	case NAN_ATTR_ID_DEVICE_CAPABILITY:
		*pbuf = NAN_ATTR_ID_DEVICE_CAPABILITY;
		attr_len = phl_nan_append_device_cap(d, pbuf, sync_info);
		_os_mem_cpy(d, (pbuf + 1), &attr_len, 2);
		attr_len += NAN_ATTR_CONTENT_OFFSET;
		break;
	case NAN_ATTR_ID_NAN_CONNECTION_CAPABILITY:
		*pbuf = NAN_ATTR_ID_NAN_CONNECTION_CAPABILITY;
		attr_len = _phl_nan_append_connection_cap(d, pbuf);
		_os_mem_cpy(d, (pbuf + 1), &attr_len, 2);
		attr_len += NAN_ATTR_CONTENT_OFFSET;
		break;
	case NAN_ATTR_ID_SRV_DES_EXT:
		*pbuf = NAN_ATTR_ID_SRV_DES_EXT;
		attr_len = _phl_nan_append_sda_ext(d, pbuf,
						   (struct phl_nan_sdf_ctx *)pdata);
		_os_mem_cpy(d, (pbuf + 1), &attr_len, 2);
		attr_len += NAN_ATTR_CONTENT_OFFSET;
		break;
#ifdef CONFIG_NAN_R2
	case NAN_ATTR_ID_NAN_AVAIL:
		phl_nan_schdlr_derive_avail_attr_w_tmap(&nan_info->scheduler_priv,
							AVAIL_ENTRY_TYPE_FLAG_ISSUE_SDF);
		phl_nan_schdlr_cnstr_avail_attr(&nan_info->scheduler_priv,
						pbuf,
						NDL_STATUS_NONE,
						&attr_len,
						false);
		break;
#endif
	case NAN_ATTR_ID_ELEMENT_CONTAINER:
		*pbuf = NAN_ATTR_ID_ELEMENT_CONTAINER;
		attr_len = /* Map ID & Element*/
		     phl_nan_append_ele_container(d,
						  (struct phl_nan_info *)pdata,
						  (pbuf + NAN_ATTR_CONTENT_OFFSET));
		_os_mem_cpy(d, (pbuf + 1), &attr_len, 2);
		attr_len += NAN_ATTR_CONTENT_OFFSET;
		break;
#ifdef CONFIG_NAN_R2
	case NAN_ATTR_ID_CIPHER_SUITE_INFO:
		*pbuf = NAN_ATTR_ID_CIPHER_SUITE_INFO;
		attr_len = phl_nan_append_cipher_suite_attr(d, pbuf,
							    &nan_info->discovery_engine_priv,
							    (struct phl_nan_sdf_ctx *)pdata);
		_os_mem_cpy(d, (pbuf + 1), &attr_len, 2);
		attr_len += NAN_ATTR_CONTENT_OFFSET;
		break;
	case NAN_ATTR_ID_SECURITY_CONTEXT_INFO:
		*pbuf = NAN_ATTR_ID_SECURITY_CONTEXT_INFO;
		attr_len = phl_nan_de_append_sec_ctx_info_attr(
				&nan_info->discovery_engine_priv,
				pbuf,
				(struct phl_nan_sdf_ctx *)pdata);
		_os_mem_cpy(d, (pbuf + 1), &attr_len, 2);
		attr_len += NAN_ATTR_CONTENT_OFFSET;
		break;
#endif
	default:
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: Unknown attr id (0x%x)\n",
			  __func__, attr_id);
		break;
	}

	return attr_len;
}

/**
 * _phl_nan_construct_beacon - Construct the NAN sync or discovery beacon
 * @drv_priv:
 * @sync_info: priv infog of sync engine
 * @nan_bcn_type: enum rtw_nan_bcn_type, sync or disc
 */
void _phl_nan_construct_beacon(void *drv_priv,
			       struct phl_nan_sync_priv *sync_info,
			       u8 nan_bcn_type)
{
	u32 pktlen;
	u16 cap_info = 0;
	u16 attr_len = 0, total_attr_len = 0;
	u8 bc_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	struct rtw_bcn_info_cmn *bcn_cmn = (nan_bcn_type == NAN_BCN_TYPE_SYNC) ?
					    &(sync_info->bcn_info.nan_sync_bcn) :
					    &(sync_info->bcn_info.nan_disc_bcn);
	u8 *pnan_ie, *pkt_buf = bcn_cmn->bcn_buf;
	struct phl_nan_mac_info *mac_info = &(sync_info->mac_info);
	struct phl_nan_info *nan_info = phl_nan_get_info(NAN_EN_SYNC, sync_info);
	u8 cluster_id[MAC_ALEN] = {0};

	phl_nan_get_cluster_id(nan_info, cluster_id);
	_os_mem_cpy(drv_priv, bcn_cmn->bssid, cluster_id, MAC_ALEN);

	SET_80211_PKT_HDR_FRAME_CONTROL(pkt_buf, 0);
	SET_80211_PKT_HDR_TYPE_AND_SUBTYPE(pkt_buf, TYPE_BEACON_FRAME);
	SET_80211_PKT_HDR_PWR_MGNT(pkt_buf, 0);

	SET_80211_PKT_HDR_ADDRESS1(drv_priv, pkt_buf, bc_addr);
	SET_80211_PKT_HDR_ADDRESS2(drv_priv, pkt_buf, mac_info->nmi.mac);
	SET_80211_PKT_HDR_ADDRESS3(drv_priv, pkt_buf, cluster_id);
	SET_80211_PKT_HDR_DURATION(pkt_buf, 0);
	SET_80211_PKT_HDR_FRAGMENT_SEQUENCE(pkt_buf, 0);

	pkt_buf += 24;
	pktlen = 24;

	/* HW timestamp */
	pkt_buf += 8;
	pktlen += 8;

	/* Beacon interval: 2 bytes */
	_os_mem_cpy(drv_priv, pkt_buf, &bcn_cmn->bcn_interval, 2);

	pkt_buf += 2;
	pktlen += 2;

	/* capability info: 2 bytes */
	cap_info |= cap_ShortSlot;
	cap_info |= cap_ShortPremble;
	_os_mem_cpy(drv_priv, pkt_buf, (unsigned char *)&cap_info, 2);

	pkt_buf += 2;
	pktlen += 2;

	/* NAN IE */
	pnan_ie = pkt_buf + NAN_IE_ATTR_OFFSET;
	*pnan_ie = NAN_ATTR_ID_MASTER_INDICATION; /* Attribute ID */
	attr_len = 2;
	_os_mem_cpy(drv_priv, (pnan_ie + 1), &attr_len, 2); /* Length */
	total_attr_len = attr_len + 3;

	pnan_ie = pkt_buf + NAN_IE_ATTR_OFFSET + total_attr_len;
	*pnan_ie = NAN_ATTR_ID_CLUSTER; /* Attribute ID */
	attr_len = 13;
	_os_mem_cpy(drv_priv, (pnan_ie + 1), &attr_len, 2); /* Length */
	_os_mem_set(drv_priv, (pnan_ie + 3), 0, 13);	    /* Anchor Master Information */
	total_attr_len += (attr_len + 3);

	/* Vendor Element ID */
	*(pkt_buf + NAN_IE_ID_OFFSET) = 0xDD;
	/* Length */
	*(pkt_buf + NAN_IE_LEN_OFFSET) = (u8)(4 + total_attr_len);
	/* NAN OUI */
	_os_mem_cpy(drv_priv, (pkt_buf + NAN_IE_OUI_OFFSET), NAN_OUI_SDF, 4);

	pkt_buf += (NAN_IE_FIXED_LEN + total_attr_len);
	pktlen += (NAN_IE_FIXED_LEN + total_attr_len);

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "[PHLNAN] nan_construct_beacon %d\n", pktlen);
	bcn_cmn->bcn_length = pktlen;
}

/**
 * _phl_nan_set_bcn - Set sync or disc nan beacon
 * @drv_priv:
 * @sync_info: priv infog of sync engine
 * @nan_bcn_type: enum rtw_nan_bcn_type, sync or disc
 * @wrole_id: to set vaule for struct rtw_bcn_info_cmn
 */
void _phl_nan_set_bcn(void *drv_priv,
		      struct phl_nan_sync_priv *sync_info,
		      u8 nan_bcn_type, u8 wrole_id)
{
	struct rtw_bcn_info_cmn *bcn_cmn = (nan_bcn_type == NAN_BCN_TYPE_SYNC) ?
					    &sync_info->bcn_info.nan_sync_bcn :
					    &sync_info->bcn_info.nan_disc_bcn;

	_os_mem_set(drv_priv, bcn_cmn, 0, sizeof(struct rtw_bcn_info_cmn));

	/* nan_set_beacon_cmn */
	bcn_cmn->bcn_id = 0; /* TODO: assigned by the core layer, but currently useless*/
	bcn_cmn->role_idx = wrole_id;
	bcn_cmn->bcn_interval = (nan_bcn_type == NAN_BCN_TYPE_SYNC) ?
				 SYNC_BCN_INTVL : DISC_BCN_INTVL;
	bcn_cmn->bcn_offload = BIT(BCN_HW_SEQ);
	bcn_cmn->bcn_rate = RTW_DATA_RATE_OFDM6;
	bcn_cmn->bcn_dtim = 0;

	_phl_nan_construct_beacon(drv_priv, sync_info, nan_bcn_type);
}

/**
 * _phl_nan_cfg_fw_schder_for_bcns
 *   - cfg bcn parameters in sync_info for all nan beacons
 *   - issue scheduler h2c for both sync/disc bcn
 * @phl_info:
 * @rlink: for 2 hw ports
 * @sync_info: priv info of sync engine
 * @wrole_id: as arg of _phl_nan_set_bcn
 */
enum rtw_phl_status
_phl_nan_cfg_fw_schder_for_bcns(struct phl_info_t *phl_info,
				struct rtw_wifi_role_link_t *rlink,
				struct phl_nan_sync_priv *sync_info)
{
	u8 chnl = sync_info->op_ch_24g;

	/* set action schedule req (2g) */
	if (RTW_HAL_STATUS_SUCCESS !=
	    rtw_hal_nan_set_skd_dw_and_disc_bcn(phl_info->hal, rlink, false,
						chnl)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: fail(2G)!!.\n",
			  __func__);
		return RTW_PHL_STATUS_FAILURE;
	}

	if ((sync_info->dual_band_support & BAND_CAP_5G) == 0)
		return RTW_PHL_STATUS_SUCCESS;

	/* set action schedule req (5g) */
	chnl = sync_info->op_ch_5g;
	if (RTW_HAL_STATUS_SUCCESS !=
	    rtw_hal_nan_set_skd_dw_and_disc_bcn(phl_info->hal, rlink, false, chnl)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: fail(5G)!!.\n", __func__);
		return RTW_PHL_STATUS_FAILURE;
	}

	return RTW_PHL_STATUS_SUCCESS;
}

/**
 * _phl_nan_send_nan_bcns - cfg and update fw bcn module
 * @drv_priv: for mem ops
 * @hal: for issuing h2c
 * @rlink: for hw_port
 * @sync_info: priv info for nan sync engine
 */
enum rtw_phl_status
_phl_nan_send_nan_bcns(void *drv_priv, void *hal,
		       struct rtw_wifi_role_link_t *rlink,
		       struct phl_nan_sync_priv *sync_info)
{
	struct phl_nan_bcn *nan_bcn_info = &sync_info->bcn_info;
	u8 dual = (sync_info->dual_band_support & BAND_CAP_5G) ? 1 : 0;
	u8 wrole_id = rlink->wrole->id;

	/* fill nan sync bcn content */
	_phl_nan_set_bcn(drv_priv, sync_info, NAN_BCN_TYPE_SYNC, wrole_id);

	phl_nan_debug_dump_data(sync_info->bcn_info.nan_sync_bcn.bcn_buf,
				sync_info->bcn_info.nan_sync_bcn.bcn_length,
				"sync bcn buf");

	if (RTW_HAL_STATUS_SUCCESS !=
	    rtw_hal_nan_send_bcn(hal, rlink, dual, NAN_BCN_TYPE_SYNC,
				 sync_info->mac_info.sync_bcn_mac_id,
				 &nan_bcn_info->nan_sync_bcn)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "[PHLNAN] rtw_hal_nan_send_bcn fail (NAN_BCN_TYPE_SYNC)!!.\n");
		return RTW_PHL_STATUS_FAILURE;
	}

	nan_bcn_info->nan_sync_bcn.bcn_added = true;

	/* fill nan disc bcn content */
	_phl_nan_set_bcn(drv_priv, sync_info, NAN_BCN_TYPE_DISC, wrole_id);

	phl_nan_debug_dump_data(sync_info->bcn_info.nan_disc_bcn.bcn_buf,
				sync_info->bcn_info.nan_disc_bcn.bcn_length,
				"disc bcn buf");

	/* TODO: should use disc_bcn_mac_id after halmac updates the H2C format */
	if (RTW_HAL_STATUS_SUCCESS !=
	    rtw_hal_nan_send_bcn(hal, rlink, dual, NAN_BCN_TYPE_DISC,
				 sync_info->mac_info.sync_bcn_mac_id,
				 &nan_bcn_info->nan_disc_bcn)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "[PHLNAN] rtw_hal_nan_send_bcn fail (NAN_BCN_TYPE_DISC)!!.\n");
		return RTW_PHL_STATUS_FAILURE;
	}

	nan_bcn_info->nan_disc_bcn.bcn_added = true;

	return RTW_PHL_STATUS_SUCCESS;
}

enum rtw_phl_status
_phl_nan_rmv_nan_bcns(void *drv_priv, void *hal,
		       struct rtw_wifi_role_link_t *rlink,
		       struct phl_nan_sync_priv *sync_info)
{
	struct phl_nan_bcn *nan_bcn_info = &sync_info->bcn_info;

	if (RTW_HAL_STATUS_SUCCESS !=
	    rtw_hal_nan_rmv_bcn(hal, rlink, NAN_BCN_TYPE_DISC)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "[PHLNAN] rtw_hal_nan_rmv_bcn fail (NAN_BCN_TYPE_DISC)!!.\n");
	}

	nan_bcn_info->nan_disc_bcn.bcn_added = false;

	if (RTW_HAL_STATUS_SUCCESS !=
	    rtw_hal_nan_rmv_bcn(hal, rlink, NAN_BCN_TYPE_SYNC)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "[PHLNAN] rtw_hal_nan_rmv_bcn fail (NAN_BCN_TYPE_SYNC)!!.\n");
	}

	nan_bcn_info->nan_sync_bcn.bcn_added = false;

	return RTW_PHL_STATUS_SUCCESS;
}

/**
 * _phl_nan_function_ctrl
 * @phl_info:
 * @enable: enable nan or not
 * @rlink: for hw_port and ext_hw_port
 * @sync_info: priv info for nan sync engine
 */
enum rtw_phl_status
 _phl_nan_function_ctrl(struct phl_info_t *phl_info,
			u8 enable,
			struct rtw_wifi_role_link_t *rlink,
			struct phl_nan_sync_priv *sync_info)
{
	void *d = phl_to_drvpriv(phl_info);
	struct phl_nan_mac_info *mac_info = &sync_info->mac_info;
	struct rtw_phl_nan_func_ctrl func_ctrl = {0};
	u8 dual = (sync_info->dual_band_support & BAND_CAP_5G) ? 1 : 0;
	struct phl_nan_info *nan_info = phl_nan_get_info(NAN_EN_SYNC, sync_info);
	u8 cluster_id[MAC_ALEN] = {0};

	func_ctrl.master_pref = sync_info->master_indication.master_pref;
	func_ctrl.random_factor = sync_info->master_indication.random_factor;
	func_ctrl.op_ch_24g = DEFAULT_NAN_24G_CH;
	func_ctrl.op_ch_5g = DEFAULT_NAN_5G_CH;
	/* TODO: need new H2C format to configure 3 macid */
	func_ctrl.mac_id_bcn = mac_info->sync_bcn_mac_id;
	func_ctrl.mac_id_mgn = mac_info->mgn_mac_id;
	func_ctrl.time_indicate_period = FW_TIME_INFO_INDICATION_PERIOD;
	func_ctrl.port_idx = rlink->hw_port;
	phl_nan_get_cluster_id(nan_info, cluster_id);
	_os_mem_cpy(d, func_ctrl.cluster_id, cluster_id, MAC_ALEN);

	if (enable == true) {
		SET_STATUS_FLAG(func_ctrl.options, NAN_CTRL_OPT_ENABLE);
		SET_STATUS_FLAG(func_ctrl.options, NAN_CTRL_OPT_UPDATE_PARAMETER);

		SET_STATUS_FLAG(func_ctrl.para_options, NAN_CTRL_UPDATE_MASTER_PREF);
		SET_STATUS_FLAG(func_ctrl.para_options, NAN_CTRL_UPDATE_RANDOM_FACTOR);
		SET_STATUS_FLAG(func_ctrl.para_options, NAN_CTRL_UPDATE_NAN_CLUSTER_ID);

		if (dual)
			SET_STATUS_FLAG(func_ctrl.options, NAN_CTRL_OPT_DUAL_BAND);
		if (sync_info->b_dis_24g_disc_bcn)
			SET_STATUS_FLAG(func_ctrl.options, NAN_CTRL_OPT_DISABLE_DISC_BCN_24G);
		if ((sync_info->b_dis_5g_disc_bcn) || !dual)
			SET_STATUS_FLAG(func_ctrl.options, NAN_CTRL_OPT_DISABLE_DISC_BCN_5G);

		SET_STATUS_FLAG(func_ctrl.options, NAN_CTRL_OPT_IGNORE_CBSSID_BCN);

		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: op(%x), para_options(%x)!!.\n", __func__,
			  func_ctrl.options,
			  func_ctrl.para_options);
	}

	if (RTW_HAL_STATUS_SUCCESS != rtw_hal_nan_func_ctrl(phl_info->hal, &func_ctrl)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "[PHLNAN] rtw_hal_nan_func_ctrl fail !!.\n");
		return RTW_PHL_STATUS_FAILURE;
	}

	return RTW_PHL_STATUS_SUCCESS;
}

enum rtw_phl_status phl_nan_mdl_init(struct phl_info_t *phl_info)
{
	void *d = phl_to_drvpriv(phl_info);
	struct phl_nan_info *nan_info = &(phl_info->nan_info);

	_os_mem_set(d, nan_info, 0, sizeof(*nan_info));
	nan_info->phl_info = phl_info;

	return RTW_PHL_STATUS_SUCCESS;
}

void phl_nan_mdl_deinit(struct phl_info_t *phl_info)
{
	struct phl_nan_info *nan_info = &(phl_info->nan_info);

	nan_info->phl_info = NULL;
}

/**
 * _phl_nan_rmv_peer_nmi_sta_info
 * @nan_info: phl_nan_info
 * @nmi: peer's nmi
 * @direct: do remove nmu sta info directly
 * (ac) indicate_rmv_nmi_sta_info
 */
void
_phl_nan_rmv_peer_nmi_sta_info(struct phl_nan_info *nan_info, u8 *nmi, u8 direct)
{
	struct phl_nan_mac_info *mac_info = &nan_info->sync_engine_priv.mac_info;
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_rmv_nmi_sta_info_cmd cmd = {0};
	struct rtw_phl_stainfo_t *nmi_sta =
		rtw_phl_get_stainfo_by_addr_ex(nan_info->phl_info, nmi);

	if (!nmi_sta) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: nmi[%02x:%02x:%02x:%02x:%02x:%02x] sta_info does not exist\n",
			  __func__, nmi[0], nmi[1], nmi[2], nmi[3], nmi[4], nmi[5]);
		return;
	}

	if (direct) {
		nan_info->nan_ops.free_peer_sta_info(d, mac_info->nmi.mac, nmi_sta);
		return;
	}

	cmd.phl_sta = nmi_sta;
	phl_nan_enqueue_cmd(nan_info->phl_info,
			    NAN_EVENT_RMV_PEER_NMI_STA_INFO,
			    (void *)(&cmd),
			    sizeof(struct phl_nan_rmv_nmi_sta_info_cmd),
			    PHL_CMD_DIRECTLY, 0);

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s: nmi[%02x:%02x:%02x:%02x:%02x:%02x]\n", __func__,
		  nmi[0], nmi[1], nmi[2],
		  nmi[3], nmi[4], nmi[5]);
}

/**
 * _phl_nan_free_peer_info_queue - Free NAN peer info queue
 * @nan_info: phl_nan_info
 * (ac) _rtw_free_nan_peer_info_queue
 */
void _phl_nan_free_peer_info_queue(struct phl_nan_info *nan_info)
{
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_peer_info_entry *peer_entry = NULL;
	_os_list *obj = NULL;
	u8 rmv_direct = false;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: cnt %d\n", __func__,
		  nan_info->peer_info_queue.cnt);

	while (pq_pop(d, &nan_info->peer_info_queue, &obj, _first, _bh)) {
		peer_entry = (struct phl_nan_peer_info_entry *)obj;
		/**
		 * Note: As rtw_nan_free_stainfo will send host msg box h2c which will
		 * poll reg val using sleep and wait method, the spinlock of queue
		 * should be released before entering phl_nan_free_peer_ndl_info_data.
		 */
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "free peer info[%02x-%02x-%02x-%02x-%02x-%02x]\n",
			  peer_entry->mac[0], peer_entry->mac[1],
			  peer_entry->mac[2], peer_entry->mac[3],
			  peer_entry->mac[4], peer_entry->mac[5]);

		/* note: we can free stainfo only if associated cam entry is cleared */
		rmv_direct = (TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_RMMOD) ||
#ifdef CONFIG_NAN_R2
			      !TEST_STATUS_FLAG(peer_entry->flag, PEER_INFO_FLAG_PMF_REQUIRED) ||
#endif
			      0) ? (true) : (false);
		_phl_nan_rmv_peer_nmi_sta_info(nan_info, peer_entry->mac, rmv_direct);
#ifdef CONFIG_NAN_R2
		phl_nan_free_peer_ndl_info_data(nan_info, peer_entry, false);
		pq_deinit(d, &peer_entry->ndl_info.schedule_entry_queue);
		pq_deinit(d, &peer_entry->ndp_queue);
#endif
		phl_nan_free_peer_srvc_queue(d, &peer_entry->srvc_info_queue);
		pq_deinit(d, &peer_entry->srvc_info_queue);

		_os_kmem_free(d, (void *)peer_entry, sizeof(*peer_entry));
	}
#ifdef CONFIG_NAN_R2
	if (nan_info->scheduler_priv.obsolete_fw_schdl_id_map) {
		phl_nan_schdlr_withdraw_fw_schdl(
			&nan_info->scheduler_priv,
			nan_info->scheduler_priv.obsolete_fw_schdl_id_map);
		nan_info->scheduler_priv.obsolete_fw_schdl_id_map = 0;
	}
#endif
}

static void
_phl_nan_session_init(struct phl_info_t *phl_info)
{
	void *d = phl_to_drvpriv(phl_info);
	struct phl_nan_info *nan_info = &(phl_info->nan_info);

	if (TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_INIT))
		return;

	/* reset variables */
	nan_info->cmd_type = NAN_CMD_TYPE_NONE;
	nan_info->mac_id_bitmap = 0;

	phl_nan_init_de_priv(phl_info, &nan_info->discovery_engine_priv);
#ifdef CONFIG_NAN_R2
	phl_nan_init_schdlr_priv(&nan_info->scheduler_priv);
	phl_nan_init_data_eng_priv(&nan_info->data_engine_priv);
#endif
	pq_init(d, &nan_info->peer_info_queue);
	pq_init(d, &nan_info->mgnt_frame_queue);
	_os_init_timer(phl_to_drvpriv(phl_info),
		       &nan_info->sync_engine_priv.dw_timer,
		       phl_nan_dw_timer_hdl, phl_info, "phl_nan_dw_timer");

	SET_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_INIT);
}

static void
_phl_nan_session_deinit(struct phl_info_t *phl_info)
{
	void *d = phl_to_drvpriv(phl_info);
	struct phl_nan_info *nan_info = &(phl_info->nan_info);

	if (!TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_INIT))
		return;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s =>\n", __func__);

	/* nan engines */
	phl_nan_deinit_de_priv(nan_info->phl_info, &nan_info->discovery_engine_priv);
#ifdef CONFIG_NAN_R2
	phl_nan_deinit_schdlr_priv(&nan_info->scheduler_priv);
	phl_nan_deinit_data_eng_priv(&nan_info->data_engine_priv);
#endif
	/* nan queues and timers */
	_phl_nan_free_peer_info_queue(nan_info);
	pq_deinit(d, &nan_info->peer_info_queue);
	_phl_nan_free_mgnt_frame_queue(nan_info);
	pq_deinit(d, &nan_info->mgnt_frame_queue);
	_os_cancel_timer(d, &(nan_info->sync_engine_priv.dw_timer));
	_os_release_timer(d, &(nan_info->sync_engine_priv.dw_timer));

	CLEAR_STATUS_FLAG(nan_info->flag,
			  NAN_PRIV_FLAG_NAN_SESSION_INIT | NAN_PRIV_FLAG_RMMOD);
}

static enum rtw_phl_status
_phl_nan_hw_init(struct phl_info_t *phl_info,
		 struct rtw_wifi_role_t *nan_wrole,
		 struct rtw_wifi_role_link_t *rlink)
{
	struct phl_nan_info *nan_info = &phl_info->nan_info;
	enum rtw_phl_status rstatus = RTW_PHL_STATUS_SUCCESS;
	enum role_type rtype = PHL_RTYPE_NAN;

	if (HW_PORT_MAX == rlink->ext_hw_port) {
		rstatus = phl_get_available_hw_port(phl_info,
						    rlink->hw_band,
						    &rlink->ext_hw_port);
		if (rstatus != RTW_PHL_STATUS_SUCCESS) {
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "%s: rstatus (phl_get_available_hw_port) %x\n",
				  __func__, rstatus);
			goto exit;
		}
	}

	if (RTW_PHL_STATUS_SUCCESS != phl_wifi_role_change(phl_info,
							   nan_wrole,
							   rlink,
							   WR_CHG_TYPE,
							   (u8 *)&rtype)) {
		rstatus = RTW_PHL_STATUS_FAILURE;
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: rstatus (WR_CHG_TYPE) %x\n",
			  __func__, rstatus);
		goto exit;
	}

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: rtype[%d] -> [%d]\n",
		  __func__, nan_wrole->type, rtype);

	if (NULL == nan_info->nan_disc_bcn_sta) {
		rstatus = rtw_phl_cmd_stainfo_ext_ctrl(phl_info,
						       &nan_info->nan_disc_bcn_sta,
						       nan_wrole->mac_addr,
						       nan_wrole,
						       DEV_TYPE_LEGACY,
						       0,
						       rlink,
						       true,
						       false,
						       PHL_CMD_DIRECTLY,
						       0);
		if (RTW_PHL_STATUS_SUCCESS != rstatus) {
			PHL_ERR("%s allocate nan_disc_bcn_sta fail!\n", __func__);
			goto exit;
		}
	}

	if (NULL == nan_info->nan_mgnt_sta) {
		rstatus = rtw_phl_cmd_stainfo_ext_ctrl(phl_info,
						       &nan_info->nan_mgnt_sta,
						       nan_wrole->mac_addr,
						       nan_wrole,
						       DEV_TYPE_LEGACY,
						       0,
						       rlink,
						       true,
						       false,
						       PHL_CMD_DIRECTLY,
						       0);
		if (RTW_PHL_STATUS_SUCCESS != rstatus) {
			PHL_ERR("%s allocate nan_mgnt_sta fail!\n", __func__);
			goto exit;
		}
	}

exit:
	return rstatus;
}

static void
_phl_nan_hw_deinit(struct phl_info_t *phl_info,
		   struct rtw_wifi_role_t *nan_wrole,
		   struct rtw_wifi_role_link_t *rlink)
{
	struct phl_nan_info *nan_info = &phl_info->nan_info;
	enum rtw_phl_status rstatus = RTW_PHL_STATUS_SUCCESS;
	enum role_type rtype = PHL_RTYPE_STATION;

	if (nan_info->nan_mgnt_sta) {
		rstatus = rtw_phl_cmd_stainfo_ext_ctrl(phl_info,
						       &nan_info->nan_mgnt_sta,
						       nan_wrole->mac_addr,
						       nan_wrole,
						       DEV_TYPE_LEGACY,
						       0,
						       rlink,
						       false,
						       false,
						       PHL_CMD_DIRECTLY,
						       0);
		if (RTW_PHL_STATUS_SUCCESS != rstatus)
			PHL_ERR("%s free nan_mgnt_sta fail!\n", __func__);
		nan_info->nan_mgnt_sta = NULL;
	}

	if (nan_info->nan_disc_bcn_sta) {
		rstatus = rtw_phl_cmd_stainfo_ext_ctrl(phl_info,
						       &nan_info->nan_disc_bcn_sta,
						       nan_wrole->mac_addr,
						       nan_wrole,
						       DEV_TYPE_LEGACY,
						       0,
						       rlink,
						       false,
						       false,
						       PHL_CMD_DIRECTLY,
						       0);
		if (RTW_PHL_STATUS_SUCCESS != rstatus)
			PHL_ERR("%s free nan_sync_bcn_sta fail!\n", __func__);
		nan_info->nan_disc_bcn_sta = NULL;
	}

	rstatus = phl_wifi_role_change(phl_info,
				       nan_wrole,
				       rlink,
				       WR_CHG_TYPE,
				       (u8 *)&rtype);
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s: rstatus (WR_CHG_TYPE) %x\n", __func__, rstatus);

	rstatus = rtw_hal_port_deinit(phl_info->hal, rlink, true);
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s: rstatus (rtw_hal_port_deinit) %x\n", __func__, rstatus);

	phl_release_ext_hw_port(phl_info, rlink);
}

/**
 * _phl_nan_init_mac_info - Initialize NAN mac resources
 * @nan_info: phl_nan_info
 * (ac) _rtw_init_nan_mac_info
 *
 * RETURN: enum rtw_phl_status
 */
enum rtw_phl_status _phl_nan_init_mac_info(struct phl_nan_info *nan_info)
{
	void *d = phl_to_drvpriv(nan_info->phl_info);
	u8 null_addr[MAC_ALEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	struct phl_nan_sync_priv *sync_info = &nan_info->sync_engine_priv;
	struct rtw_phl_stainfo_t *sta = NULL;

#ifdef CONFIG_NAN_R2
	sync_info->mac_info.ndi_ctx.ndi_available_cnt = MAX_NDI_NUM;
#endif
	if (0 == _os_mem_cmp(d, sync_info->mac_info.nmi.mac, null_addr, MAC_ALEN))
		_os_mem_cpy(d, sync_info->mac_info.nmi.mac, nan_info->wrole_addr, MAC_ALEN);

	if (!sync_info->op_ch_24g)
		sync_info->op_ch_24g = DEFAULT_NAN_24G_CH;
	if (!sync_info->op_ch_5g)
		sync_info->op_ch_5g = DEFAULT_NAN_5G_CH;

	sync_info->master_indication.random_factor = sync_info->mac_info.nmi.mac[5] << 1;
	sync_info->cmt_dw_peri_24G = (sync_info->cmt_dw_peri_24G) ?
					(sync_info->cmt_dw_peri_24G) :
					(DW_INTERVAL/ NAN_SLOT);
	sync_info->cmt_dw_peri_5G = (sync_info->cmt_dw_peri_5G ) ?
					(sync_info->cmt_dw_peri_5G) :
					(DW_INTERVAL/ NAN_SLOT);

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s: cmt_dw_period[%d][%d]\n", __func__,
		  sync_info->cmt_dw_peri_24G,
		  sync_info->cmt_dw_peri_5G);

	/*1. enable HW port for NMI => done by wrole flow */

	/*2. set MACID for NMI Transmittion*/
	sta = rtw_phl_get_stainfo_self(nan_info->phl_info, nan_info->nan_rlink);
	if (sta == NULL) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_ERR_,
			  "_phl_nan_init_mac_info(): no stainfo_self\n");
		return RTW_PHL_STATUS_FAILURE;
	}

	sync_info->mac_info.sync_bcn_mac_id = sta->macid;
	sync_info->mac_info.disc_bcn_mac_id = nan_info->nan_disc_bcn_sta->macid;
	sync_info->mac_info.mgn_mac_id = nan_info->nan_mgnt_sta->macid;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s: disc_bcn_mac_id[%d], sync_bcn_mac_id[%d], mgn_mac_id[%d]\n",
		  __func__,
		  sync_info->mac_info.disc_bcn_mac_id,
		  sync_info->mac_info.sync_bcn_mac_id,
		  sync_info->mac_info.mgn_mac_id);

	if (0 == _os_mem_cmp(d, sync_info->mac_info.nmi.bssid, null_addr, MAC_ALEN)) {
		/*set default cluster id same as FW did*/
		_os_mem_cpy(d, sync_info->mac_info.nmi.bssid, nan_cluster_id, 4);
		sync_info->mac_info.nmi.bssid[4] = (u8)_os_random32(d);
		sync_info->mac_info.nmi.bssid[5] = (u8)_os_random32(d);
	} else if (0 != _os_mem_cmp(d, sync_info->mac_info.nmi.bssid, nan_cluster_id, 4)) {
		/**
		 * The NAN Cluster ID is a MAC address that takes a value
		 * from 50-6F-9A-01-00-00 to 50-6F-9A-01-FF-FF
		 * and is carried in the A3 field of some of the NAN frames
		 */
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "bssid:%02x-%02x-%02x-%02x-%02x-%02x, bssid must start with 50:6f:9a:01\n",
			  sync_info->mac_info.nmi.bssid[0],
			  sync_info->mac_info.nmi.bssid[1],
			  sync_info->mac_info.nmi.bssid[2],
			  sync_info->mac_info.nmi.bssid[3],
			  sync_info->mac_info.nmi.bssid[4],
			  sync_info->mac_info.nmi.bssid[5]);
		return RTW_PHL_STATUS_FAILURE;
	}

	_os_spinlock_init(d, &sync_info->mac_info.nmi.bssid_lock);

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  " NMI mac_addr:%02x-%02x-%02x-%02x-%02x-%02x macid:%u,bitmap =%x\n",
		  sync_info->mac_info.nmi.mac[0],
		  sync_info->mac_info.nmi.mac[1],
		  sync_info->mac_info.nmi.mac[2],
		  sync_info->mac_info.nmi.mac[3],
		  sync_info->mac_info.nmi.mac[4],
		  sync_info->mac_info.nmi.mac[5],
		  sync_info->mac_info.mgn_mac_id,
		  nan_info->mac_id_bitmap);
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  " NMI bssid:%02x-%02x-%02x-%02x-%02x-%02x\n",
		  sync_info->mac_info.nmi.bssid[0],
		  sync_info->mac_info.nmi.bssid[1],
		  sync_info->mac_info.nmi.bssid[2],
		  sync_info->mac_info.nmi.bssid[3],
		  sync_info->mac_info.nmi.bssid[4],
		  sync_info->mac_info.nmi.bssid[5]);

	return RTW_PHL_STATUS_SUCCESS;
}

/**
 * _phl_nan_free_mac_info - Free NAN mac resources
 * @nan_info: phl_nan_info
 * (ac) _rtw_free_nan_mac_info
 */
void _phl_nan_free_mac_info(struct phl_nan_info *nan_info)
{
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_sync_priv *sync_info = &nan_info->sync_engine_priv;

#ifdef CONFIG_PHL_NAN_DEBUG
	/* move to mr control; remove these whenever verification is done */
	/* rtw_halmac_set_network_type(adapter_to_dvobj(pnanpriv->padapter),
		pnanpriv->mac_info.nmi.port_idx, _HW_STATE_NOLINK_); */

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  " NMI mac_addr:%02x-%02x-%02x-%02x-%02x-%02x, macid:%u, bitmap =%x \n",
		  sync_info->mac_info.nmi.mac[0],
		  sync_info->mac_info.nmi.mac[1],
		  sync_info->mac_info.nmi.mac[2],
		  sync_info->mac_info.nmi.mac[3],
		  sync_info->mac_info.nmi.mac[4],
		  sync_info->mac_info.nmi.mac[5],
		  sync_info->mac_info.mgn_mac_id,
		  nan_info->mac_id_bitmap);
#endif /* CONFIG_PHL_NAN_DEBUG */

	_os_mem_set(d, &sync_info->mac_info.nmi.bssid, 0, MAC_ALEN);
	_os_spinlock_free(d, &sync_info->mac_info.nmi.bssid_lock);
}

/**
 * _phl_nan_session_stop -
 *
 * 1. Free mac resources
 * 2. Stop SW DW timer
 * 3. Send H2C to stop FW NAN function
 *
 * @phl_info: pointer of phl_info which contains nan_info
 * @is_rmmod: if this takes place within rmmod process
 */
static void
_phl_nan_session_stop(struct phl_info_t *phl_info, u8 is_rmmod)
{
	void *d = phl_to_drvpriv(phl_info);
	struct phl_nan_info *nan_info = _phl_nan_get_info(phl_info);
	struct phl_nan_sync_priv *sync_priv = &nan_info->sync_engine_priv;
	struct rtw_wifi_role_link_t *rlink = nan_info->nan_rlink;
	struct rtw_phl_nan_ie_container *ies = &nan_info->nan_ies;

	if (!TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_START))
		return;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s => \n", __func__);

	if (is_rmmod == true)
		SET_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_RMMOD);

	CLEAR_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_START);

	_phl_nan_function_ctrl(phl_info, false, rlink, sync_priv);

	_phl_nan_rmv_nan_bcns(d, phl_info->hal, rlink, sync_priv);

	_phl_nan_free_mac_info(nan_info);
	if (ies->ie_s_rate_len)
		_os_mem_free(d, ies->ie_s_rate, ies->ie_s_rate_len);
	if (ies->ie_es_rate_len)
		_os_mem_free(d, ies->ie_es_rate, ies->ie_es_rate_len);
	if (ies->ie_ht_len)
		_os_mem_free(d, ies->ie_ht, ies->ie_ht_len);
	if (ies->ie_vht_len)
		_os_mem_free(d, ies->ie_vht, ies->ie_vht_len);

	rtw_hal_en_fw_log_comp(phl_info->hal, FL_COMP_NAN, false);
}

/**
 * _phl_nan_session_start -
 *
 * 1. Initialize mac resources
 * 2. Send H2C to start FW NAN function and download NAN beacon
 * 3. Start SW DW timer
 *
 * @phl_info: pointer of phl_info which contains nan_info
 * @nan_cmd_type: enum rtw_phl_nan_cmd_type
 * @mp: master preference
 * @bands: bitmap of support bands; using phl layer macros
 * @ies: support rate, ext suuport rate, ht, vht
 */
static enum rtw_phl_status
_phl_nan_session_start(struct phl_info_t *phl_info,
		       u8 nan_cmd_type,
		       u8 mp,
		       u8 bands,
		       struct rtw_phl_nan_ie_container *ies)
{
	void *d = phl_to_drvpriv(phl_info);
	struct phl_nan_info *nan_info = _phl_nan_get_info(phl_info);
	struct phl_nan_sync_priv *sync_priv = &nan_info->sync_engine_priv;
	struct rtw_wifi_role_link_t *rlink = nan_info->nan_rlink;

	if (TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_START))
		goto error;

	/* cmd type could be only changed here for consistency */
	/* Do not use cmd_type in core layer after preparation functions */
	nan_info->cmd_type = nan_cmd_type;

	/* set up default vaules if no input from user space */
	sync_priv->master_indication.master_pref = (mp) ? mp : 5;
	sync_priv->dual_band_support = bands;

	_os_mem_cpy(d, &nan_info->nan_ies, ies, sizeof(*ies));

	rtw_hal_en_fw_log_comp(phl_info->hal, FL_COMP_NAN, true);

	if (nan_info->nan_rlink->cap.bw != CHANNEL_WIDTH_20)
		SET_STATUS_FLAG(sync_priv->flag, NAN_MAC_FLAG_VHT_ENABLE);

	phl_nan_set_dw_timer(d, nan_info, 1);

	if (RTW_PHL_STATUS_SUCCESS != _phl_nan_init_mac_info(nan_info))
		goto error_free_ies;
#ifdef CONFIG_NAN_R2
	phl_nan_post_init_schdlr_priv(&nan_info->scheduler_priv);
#endif
	if (RTW_PHL_STATUS_SUCCESS !=
	    _phl_nan_send_nan_bcns(d, phl_info->hal, rlink, sync_priv))
		goto error_free_mac;

	if (RTW_PHL_STATUS_SUCCESS !=
	    _phl_nan_cfg_fw_schder_for_bcns(phl_info, rlink, sync_priv))
		goto error_rmv_bcns;

	if (RTW_PHL_STATUS_SUCCESS !=
	    _phl_nan_function_ctrl(phl_info, true, rlink, sync_priv))
		goto error_rmv_bcns;

	SET_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_START);
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: succeed\n", __func__);

	return RTW_PHL_STATUS_SUCCESS;

error_rmv_bcns:
	_phl_nan_rmv_nan_bcns(d, phl_info->hal, rlink, sync_priv);
error_free_mac:
	_phl_nan_free_mac_info(nan_info);
error_free_ies:
	if (ies->ie_s_rate_len)
		_os_mem_free(d, ies->ie_s_rate, ies->ie_s_rate_len);
	if (ies->ie_es_rate_len)
		_os_mem_free(d, ies->ie_es_rate, ies->ie_es_rate_len);
	if (ies->ie_ht_len)
		_os_mem_free(d, ies->ie_ht, ies->ie_ht_len);
	if (ies->ie_vht_len)
		_os_mem_free(d, ies->ie_vht, ies->ie_vht_len);

	rtw_hal_en_fw_log_comp(phl_info->hal, FL_COMP_NAN, false);
error:
	return RTW_PHL_STATUS_FAILURE;
}

u8 phl_nan_is_mgnt_frame_need_enqueue(struct phl_nan_info *nan_info,
				      struct phl_nan_peer_info_entry *peer_info)
{
	struct phl_nan_sync_priv *sync_priv = &nan_info->sync_engine_priv;
	u8 *peer_band_offset = peer_info->devcap_attr + DEVICE_CAP_FIELD_START_OFFSET;
	u8 is_peer_5g_en = GET_NAN_DEVICE_CAP_ELE_SUPPORT_BAND_5G(peer_band_offset);
	u8 is_self_5g_en = (sync_priv->dual_band_support & BAND_CAP_5G);
	/* temp use 2.4g committed dw period */
	u32 self_cmt_dw = (sync_priv->cmt_dw_peri_24G * NAN_SLOT) / DW_INTERVAL;
	u32 peer_cmt_dw = 0;
	u8 ret = true;

	peer_cmt_dw = MIN(peer_info->cmt_dw_info.dw_5g,
			  peer_info->cmt_dw_info.dw_24g);
	peer_cmt_dw = (peer_cmt_dw) ? (BIT0 << (peer_cmt_dw - 1)) : (1);

	/* If peer's committed DW interval is less than or equal to ours'
	 * We simply download this pkt into Tx Buffer at any time and
	 * let FW to actually transmit it within current or next committed DW
	 * Otherwise, we have to make sure pkt appears in the Tx buffer
	 * a little bit earlier before designated DW.
	 * Thus, Mgnt frame Q is adopted to approach this requirement.
	 */
	if ((is_peer_5g_en || (!is_peer_5g_en && !is_self_5g_en)) &&
	    (peer_cmt_dw <= self_cmt_dw))
		ret = false;

	return ret;
}

enum rtw_phl_status
phl_nan_insert_mgnt_frame_entry(struct phl_nan_info *nan_info,
				u8 *nan_attr_buf,
				u16 attr_len,
				u8 *peer_info_mac,
				enum rtw_phl_nan_pkt_type pkt_type,
				u8 mgnt_need_enq)
{
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_mgnt_frame_entry *send_entry = NULL;
	struct phl_nan_sync_priv *sync_priv = &nan_info->sync_engine_priv;
	struct phl_nan_mac_info *mac_info = &sync_priv->mac_info;
	u8 cluster_id[MAC_ALEN] = {0};
	enum rtw_phl_status phl_status = RTW_PHL_STATUS_FAILURE;

	if (mgnt_need_enq == true) {
		send_entry = (struct phl_nan_mgnt_frame_entry *)
			     _os_mem_alloc(d, sizeof(struct phl_nan_mgnt_frame_entry));
		if (!send_entry) {
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
				  "%s: create send_entry fail!\n", __func__);
			goto exit;
		}

		/* init send_entry */
		send_entry->nan_attr_buf = _os_mem_alloc(d, attr_len);
		_os_mem_cpy(d, send_entry->nan_attr_buf, nan_attr_buf, attr_len);
		send_entry->pkt_type = pkt_type;
		send_entry->attr_len = attr_len;
		_os_mem_cpy(d, send_entry->peer_addr, peer_info_mac, MAC_ALEN);

		pq_push(d, &nan_info->mgnt_frame_queue, (_os_list *)send_entry, _tail, _bh);

		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: create send_entry success!\n", __func__);
	} else {
		phl_nan_get_cluster_id(nan_info, cluster_id);
		phl_status = nan_info->nan_ops.tx_mgnt(d,
						       mac_info->mgn_mac_id,
						       peer_info_mac,
						       mac_info->nmi.mac,
						       cluster_id,
						       nan_info->wrole_addr,
						       nan_attr_buf,
						       attr_len,
						       pkt_type);
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: send directly %s\n", __func__,
			  ((phl_status == RTW_PHL_STATUS_SUCCESS) ? "success" : "fail"));
	}

exit:
	return phl_status;
}

/**
 * _phl_nan_send_mgnt_frame_queue
 *  - walk through whole mgnt_frame_queue and send pkts if timing is right
 * @nan_info: phl_nan_info
 * @dw_idx: the Nnd DW bwteen 2 DW0, N=0~15
 * @is_5g: a 5g DW or not
 * (ac) _rtw_nan_send_mgnt_frame_queue
 */
void
_phl_nan_send_mgnt_frame_queue(struct phl_nan_info *nan_info,
			       u32 dw_idx,
			       u8 is_5g)
{
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_mgnt_frame_entry *snd_ent = NULL;
	struct phl_nan_mac_info *mac_info = &nan_info->sync_engine_priv.mac_info;
	struct phl_queue tmp_q;
	struct phl_nan_peer_info_entry *peer_info = NULL;
	_os_list *obj = NULL;
	u32 peer_cmt_dw = 0;
	u8 *peer_band_ofst = NULL;
	u8 is_peer_5g_en = 0;
	u8 cluster_id[MAC_ALEN] = {0};

	if (!nan_info->nan_ops.tx_mgnt) {
		PHL_ERR("%s: nan_ops.tx_mgnt is a null pointer !!\n", __func__);
		return;
	}

	pq_init(d, &tmp_q);

	/* search token in scan request queue */
	while (pq_pop(d, &nan_info->mgnt_frame_queue, &obj, _first, _bh)) {
		snd_ent = (struct phl_nan_mgnt_frame_entry *)obj;

		if (false == phl_nan_is_peer_info_exist(nan_info,
							snd_ent->peer_addr,
							&peer_info)) {
			_phl_nan_free_mgnt_frame_entry(nan_info, snd_ent);
			continue;
		}

		/* get peer's 5g band support */
		peer_band_ofst = peer_info->devcap_attr +
				 DEVICE_CAP_FIELD_START_OFFSET;
		is_peer_5g_en = GET_NAN_DEVICE_CAP_ELE_SUPPORT_BAND_5G(peer_band_ofst);
		if (is_5g == true && !is_peer_5g_en) {
			phl_nan_done_peer_info_entry(d, nan_info);
			pq_push(d, &tmp_q, (_os_list *)snd_ent, _tail, _bh);
			continue;
		}

		/* get peer's committed dw */
		peer_cmt_dw = (is_5g == true) ? (peer_info->cmt_dw_info.dw_5g) :
						(peer_info->cmt_dw_info.dw_24g);

		phl_nan_done_peer_info_entry(d, nan_info);

		peer_cmt_dw = (peer_cmt_dw) ? (BIT0 << (peer_cmt_dw - 1)) : (1);
		if (dw_idx % peer_cmt_dw) {
			pq_push(d, &tmp_q, (_os_list *)snd_ent, _tail, _bh);
			continue;
		}

		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s!!\n", __func__);
		phl_nan_get_cluster_id(nan_info, cluster_id);
		nan_info->nan_ops.tx_mgnt(d,
					  mac_info->mgn_mac_id,
					  snd_ent->peer_addr,
					  mac_info->nmi.mac,
					  cluster_id,
					  nan_info->wrole_addr,
					  snd_ent->nan_attr_buf,
					  snd_ent->attr_len,
					  snd_ent->pkt_type);
		/* post process */

		_phl_nan_free_mgnt_frame_entry(nan_info, snd_ent);
	}

	while (pq_pop(d, &tmp_q, &obj, _first, _bh))
		pq_push(d, &nan_info->mgnt_frame_queue, obj, _tail, _bh);

	pq_deinit(d, &tmp_q);
}

u8
phl_nan_alloc_peer_info_entry(struct phl_nan_info *nan_info,
			      struct phl_nan_peer_info_entry **p_peer_info_entry,
			      u8 *mac_addr)
{

	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_queue *peer_queue = &nan_info->peer_info_queue;
	struct phl_nan_peer_info_entry *entry = NULL;
	struct phl_nan_alloc_nmi_sta_info_cmd cmd = {0};

	/* note: alloc peer info might be called during rx action frame */
	entry = _os_kmem_alloc(d, sizeof(struct phl_nan_peer_info_entry));
	if (!entry) {
		PHL_WARN("%s: create peer_info fail!\n", __func__);
		return false;
	}

	/* init entry */
	pq_init(d, &entry->srvc_info_queue);
#ifdef CONFIG_NAN_R2
	pq_init(d, &entry->ndp_queue);
	_phl_nan_init_peer_ndl_info_data(d, entry);
#endif
	_os_mem_cpy(d, entry->mac, mac_addr, MAC_ALEN);

	_os_spinlock(d, &peer_queue->lock, _bh, NULL);
	list_add(&entry->list, &peer_queue->queue);
	peer_queue->cnt++;

	/* enqueue alloc peer nmi sta info */
	_os_mem_cpy(d, cmd.peer_mac, entry->mac, MAC_ALEN);
	if (RTW_PHL_STATUS_SUCCESS !=
	    phl_nan_enqueue_cmd(nan_info->phl_info,
				NAN_EVENT_ALLOC_PEER_NMI_STA_INFO,
				(void *)(&cmd),
				sizeof(struct phl_nan_alloc_nmi_sta_info_cmd),
				PHL_CMD_NO_WAIT, 0)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: send alloc nmi sta info fail!\n", __func__);
		list_del(&entry->list);
		peer_queue->cnt--;
		_os_spinunlock(d, &peer_queue->lock, _bh, NULL);
#ifdef CONFIG_NAN_R2
		pq_deinit(d, &entry->ndl_info.schedule_entry_queue);
		pq_deinit(d, &entry->ndp_queue);
#endif
		pq_deinit(d, &entry->srvc_info_queue);
		_os_mem_free(d, entry, sizeof(struct phl_nan_peer_info_entry));

		return false;
	}

	*p_peer_info_entry = entry;
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: create peer!\n", __func__);

	return true;
}

void phl_nan_dw_timer_hdl(void *ctx)
{
	struct phl_info_t *phl_info = (struct phl_info_t *)ctx;

	if (RTW_PHL_STATUS_SUCCESS != phl_nan_enqueue_cmd(phl_info,
							  NAN_EVENT_NEXT_DW,
							  NULL,
							  0,
							  PHL_CMD_NO_WAIT,
							  0)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: Get avail cmd entry fail!\n", __func__);
	}
}

/**
 * _phl_nan_dw_timeout_handler
 * @nan_info: phl_nan_info
 * @tsf: current tsf of nan wrole's hw port
 * (ac) dw_timeout_handler
 */
void _phl_nan_dw_timeout_handler(struct phl_nan_info *nan_info, u32 tsf)
{
	void *d = phl_nan_info_to_drvpriv(nan_info);
	u8 is_skip = false;
	u8 is_5g = false;
	u32 dw_idx = 0;
	u32 tsf_tmp = (tsf / TU) % DW_INTERVAL;

	is_skip = (((tsf_tmp < DW_5G_OFFSET) &&
		    (tsf_tmp > (DW_5G_OFFSET - SW_DW_TIMER_DEVIATION))) ||
		   (tsf_tmp > (DW_INTERVAL - SW_DW_TIMER_DEVIATION))) ?
			  (true) : (false);

	if (true == is_skip ||
	    TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NEW_CLUS_JOINED)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: action skipped due to timing deviation\n",
			  __func__);
		return;
	}

	tsf_tmp = ((tsf / TU) % DW0_INTERVAL);
	if (nan_info->sync_engine_priv.dual_band_support & BAND_CAP_5G) {
		is_5g = ((tsf_tmp % DW_INTERVAL) < DW_5G_OFFSET) ? (true) :
								   (false);
		dw_idx = (is_5g) ? (tsf_tmp / DW_INTERVAL) :
				   ((tsf_tmp / DW_INTERVAL) + 1);
	} else {
		dw_idx = ((tsf_tmp / DW_INTERVAL) + 1);
	}
	dw_idx = (dw_idx == 16) ? (0) : (dw_idx);

	/* to send_bcast_sdf */
	phl_nan_de_dw_timeout_hdl(d, &nan_info->discovery_engine_priv, tsf, dw_idx);
	/* to send unicast sdf */
	_phl_nan_send_mgnt_frame_queue(nan_info, dw_idx, is_5g);
}

/**
 * phl_nan_event_next_dw_hdl - Calculate and set the next DW timer
 * (ac) rtw_nan_event_next_dw_hdl
 */
void phl_nan_event_next_dw_hdl(struct phl_info_t *phl_info)
{
	u32 tsf_temp = 0;
	u32 tsf_h = 0, tsf_l = 0;
	u32 next_dw = 0;
	u32 next_2g_dw = 0;
	u32 next_5g_dw = 0;
	void *drv = phl_info->phl_com->drv_priv;
	struct phl_nan_info *nan_info = _phl_nan_get_info(phl_info);
	struct rtw_wifi_role_link_t *rlink = NULL;
	u32 cmt_dw_tu_2g = nan_info->sync_engine_priv.cmt_dw_peri_24G * NAN_SLOT;
	u32 cmt_dw_tu_5g = nan_info->sync_engine_priv.cmt_dw_peri_5G * NAN_SLOT;

	rlink = nan_info->nan_rlink;
	if (RTW_HAL_STATUS_SUCCESS != rtw_hal_get_tsf(phl_info->hal,
						      rlink->hw_band,
						      rlink->hw_port,
						      &tsf_h,
						      &tsf_l)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_WARNING_,
			  "%s: Get tsf fail\n", __func__);
		return;
	}

	/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s: tsf_l[0x%x], dual_band_support[%x]\n", __func__, tsf_l,
		  nan_info->sync_engine_priv.dual_band_support); */

	tsf_temp = tsf_l;
	tsf_l = tsf_l / TU;

	if (nan_info->sync_engine_priv.dual_band_support & BAND_CAP_5G) {
		/**
		 * As the committed dw period is separated into 2g and 5g, we
		 * need to calculate both next_2g_dw and next_5g_dw, and choose
		 * the early one for timer setup.
		 */
		if ((tsf_l % DW_INTERVAL) < DW_5G_OFFSET) {
			/* current dw is 5g dw */
			next_2g_dw = cmt_dw_tu_2g - (tsf_l % cmt_dw_tu_2g);
			next_5g_dw = cmt_dw_tu_5g - (tsf_l % cmt_dw_tu_5g) + DW_5G_OFFSET;
		} else {
			/* current dw is 2g dw */
			next_2g_dw = (2 * cmt_dw_tu_2g) - (tsf_l % cmt_dw_tu_2g);
			next_5g_dw = cmt_dw_tu_5g - (tsf_l % cmt_dw_tu_5g) + DW_5G_OFFSET;
		}
		/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s:--- [%d][%d]\n",
			     __func__, next_2g_dw, next_5g_dw); */
		next_dw = (next_2g_dw < next_5g_dw) ? next_2g_dw : next_5g_dw;
	} else {
		next_dw = (2 * cmt_dw_tu_2g) -
				  (tsf_l % cmt_dw_tu_2g);
	}
	/* PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s:--- [%d][%d][%d][%d][%d]\n",
		__func__, next_dw, SW_DW_TIMER_OFFSET, tsf_l,
		cmt_dw_tu_2g, cmt_dw_tu_5g); */

	next_dw -= SW_DW_TIMER_OFFSET;
	next_dw = (next_dw * TU / 1000);
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: [%d]\n", __func__, next_dw);

	if (TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NEW_CLUS_JOINED))
		CLEAR_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NEW_CLUS_JOINED);

	phl_nan_set_dw_timer(drv, nan_info, next_dw);
	_phl_nan_dw_timeout_handler(nan_info, tsf_temp);
}

/**
 * _phl_nan_join_cluster_hdl
 * @nan_info: phl_nan_info
 * (ac) rtw_nan_clear_existing_peer_info_hdl
 */
void _phl_nan_join_cluster_hdl(struct phl_nan_info *nan_info)
{
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: is allow[%d]\n", __func__,
		  (TEST_STATUS_FLAG(nan_info->sync_engine_priv.flag,
				    NAN_MAC_FLAG_ALLOW_JOIN_NEW_CLUSTER)) ?
				    (1) : (0));
	/* issue h2c whenever it's available. */
	/* rtw_halmac_nan_cluster_join(adapter_to_dvobj(padapter), is_allow); */
	CLEAR_STATUS_FLAG(nan_info->sync_engine_priv.flag,
			  NAN_MAC_FLAG_ALLOW_JOIN_NEW_CLUSTER);
}

/**
 * _phl_nan_clear_existing_peer_info_hdl - clear queue
 * @nan_info: phl_nan_info
 * (ac) rtw_nan_clear_existing_peer_info_hdl
 */
void _phl_nan_clear_existing_peer_info_hdl(struct phl_nan_info *nan_info)
{
	/* rtw_nan_clear_existing_peer_info_hdl */
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s\n", __func__);
	_phl_nan_free_peer_info_queue(nan_info);
}

enum rtw_phl_status
phl_cmd_nan_event_hdl(struct phl_info_t *phl_info, u8 *pbuf)
{
	void *d = phl_to_drvpriv(phl_info);
	struct phl_nan_info *nan_info = _phl_nan_get_info(phl_info);
	struct phl_nan_mac_info *mac_info = &nan_info->sync_engine_priv.mac_info;
	struct phl_nan_event_parm *p_parm = (struct phl_nan_event_parm *)pbuf;
	struct rtw_phl_stainfo_t *phl_sta = NULL;
	struct phl_nan_peer_info_entry *peer_info = NULL;
	u8 *peer_mac = NULL;
	u8 ht_cap[HT_CAP_IE_LEN];
	u8 vht_cap[VHT_CAP_IE_LEN];
	u8 nmi_mac_id = 0;
	enum rtw_phl_status psts = RTW_PHL_STATUS_SUCCESS;

	if (!TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_START) &&
	    p_parm->event_type != NAN_EVENT_RMV_PEER_NMI_STA_INFO &&
#ifdef CONFIG_NAN_R2
	    p_parm->event_type != NAN_EVENT_RMV_PEER_NDI_STA_INFO &&
#endif
	    1) {
		psts = RTW_PHL_STATUS_FAILURE;
		goto exit;
	}

	switch (p_parm->event_type) {
	case NAN_EVENT_NEXT_DW:
		phl_nan_event_next_dw_hdl(phl_info);
		break;
	case NAN_EVENT_JOIN_NEW_CLUSTER:
		_phl_nan_join_cluster_hdl(nan_info);
		break;
	case NAN_EVENT_CLUSTER_JOIN_DONE:
		_phl_nan_clear_existing_peer_info_hdl(nan_info);
		phl_nan_free_all_auto_flup_srv_inst(phl_info);
		break;
	case NAN_EVENT_ALLOC_PEER_NMI_STA_INFO:
		peer_mac = ((struct phl_nan_alloc_nmi_sta_info_cmd *)(p_parm->data))->peer_mac;
		if (false == phl_nan_is_peer_info_exist(nan_info, peer_mac,
							&peer_info)) {
			PHL_WARN("%s: peer_info(%02x:%02x:%02x:%02x:%02x:%02x) not found!\n", __func__,
				 peer_mac[0], peer_mac[1], peer_mac[2],
				 peer_mac[3], peer_mac[4], peer_mac[5]);
			psts = RTW_PHL_STATUS_FAILURE;
			break;
		}
		phl_nan_done_peer_info_entry(d, nan_info);

		psts = nan_info->nan_ops.alloc_peer_sta_info(d,
							     mac_info->nmi.mac,
							     peer_mac,
							     ht_cap,
							     vht_cap,
							     &nmi_mac_id,
							     false);
		if (psts != RTW_PHL_STATUS_SUCCESS) {
			PHL_WARN("%s: allocate sta_info(%02x:%02x:%02x:%02x:%02x:%02x) failed!\n", __func__,
				 peer_mac[0], peer_mac[1], peer_mac[2],
				 peer_mac[3], peer_mac[4], peer_mac[5]);
			/* TODO: the peer_info_entry should be removed for the next sta_info allocation */
			break;
		}

		if (false == phl_nan_is_peer_info_exist(nan_info, peer_mac,
							&peer_info)) {
			PHL_WARN("%s: peer_info(%02x:%02x:%02x:%02x:%02x:%02x) not found!\n", __func__,
				 peer_mac[0], peer_mac[1], peer_mac[2],
				 peer_mac[3], peer_mac[4], peer_mac[5]);
			psts = RTW_PHL_STATUS_FAILURE;
			break;
		}

		_os_mem_cpy(d, peer_info->ht_cap, ht_cap, HT_CAP_IE_LEN);
		_os_mem_cpy(d, peer_info->vht_cap, ht_cap, VHT_CAP_IE_LEN);
#ifdef CONFIG_NAN_R2
		peer_info->ndl_info.ndl_mac_id = nmi_mac_id;
#endif
		phl_nan_done_peer_info_entry(d, nan_info);

		break;
	case NAN_EVENT_RMV_PEER_NMI_STA_INFO:
		phl_sta = ((struct phl_nan_rmv_nmi_sta_info_cmd *)(p_parm->data))->phl_sta;
		nan_info->nan_ops.free_peer_sta_info(d, mac_info->nmi.mac, phl_sta);
		break;
#ifdef CONFIG_NAN_R2
	case NAN_EVENT_PAUSE_FAW_TX:
	case NAN_EVENT_REJECT_NEW_SCHDL:
	case NAN_EVENT_CHECK_NEW_SCHDL_DOABLE:
	case NAN_EVENT_APPLY_NEW_TIME_MAP_PATTERN:
	case NAN_EVENT_NDL_SEND_DATA_PATH_FRAME:
	case NAN_EVENT_NDL_SEND_SCHDL_FRAME:
	case NAN_EVENT_WITHDRAW_FW_SCHDL:
		phl_nan_schdlr_evt_handler(&nan_info->scheduler_priv,
					   p_parm->event_type, (void *)p_parm);
		break;
	case NAN_EVENT_NDP_SEND_DATA_PATH_FRAME:
	case NAN_EVENT_PEER_NDP_EMPTY:
	case NAN_EVENT_RMV_PEER_NDI_STA_INFO:
		phl_nan_data_engine_event_hdl(&nan_info->data_engine_priv,
					      p_parm->event_type,
					      (void *)p_parm);
		break;
#endif
	case NAN_EVENT_SEND_SDF:
	case NAN_EVENT_RPRT_MATCH_EVT:
		phl_nan_disc_engine_evt_hdl(&nan_info->discovery_engine_priv,
					    p_parm->event_type, (void *)p_parm);
		break;
	case NAN_EVENT_ADD_NAN_FUNC:
		psts = phl_add_nan_func_evt_hdl(phl_info, (void *)p_parm);
		break;
	case NAN_EVENT_DEL_NAN_FUNC:
		psts = phl_del_nan_func_evt_hdl(phl_info, (void *)p_parm);
		break;
	default:
		PHL_TRACE(COMP_PHL_NAN, _PHL_WARNING_,
			  "%s: NAN event type not found! (%d)\n",
			  __func__, p_parm->event_type);
		psts = RTW_PHL_STATUS_FAILURE;
		goto exit;
	}

exit:
	return psts;
}

/**
 * _phl_nan_cmd_event_cb - Free memory
 * @drv_priv:
 * @cmd: pointer of phl_nan_event_parm; need to free mem in the handler.
 * @cmd_len: len of cmd
 * @status
 */
static void
_phl_nan_cmd_event_cb(void *drv_priv, u8 *cmd, u32 cmd_len,
		     enum rtw_phl_status status)
{
	if (cmd) {
		_os_kmem_free(drv_priv, cmd, cmd_len);
		cmd = NULL;
		/* PHL_INFO("%s.\n", __func__); */
	}
}

/**
 * _phl_nan_cluster_rpt_hdl
 * @nan_info: phl_nan_info
 * @cluster_rpt: cluster report from c2h
 * (ac) _nan_parse_c2h_cluster_info
 */
void
_phl_nan_cluster_rpt_hdl(struct phl_nan_info *nan_info,
			 struct rtw_phl_nan_cluster_rpt *clus_rpt)
{
	/* _nan_parse_c2h_cluster_info */
	struct phl_nan_sync_priv *sync_priv = &nan_info->sync_engine_priv;
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_master_indication *master_indi = &sync_priv->master_indication;
	struct phl_nan_anchor_master_info *am_info = &sync_priv->am_info;
	struct phl_nan_mac_info *mac_info = &sync_priv->mac_info;

	master_indi->master_pref = clus_rpt->master_pref;
	master_indi->random_factor = clus_rpt->random_factor;
	am_info->amr = clus_rpt->amr;
	am_info->ambtt = clus_rpt->ambtt;
	am_info->hop_count = clus_rpt->hop_count;
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s: master_pref[%d],random_factor[%d] AMR[0x%llx] hop_count[%d] AMBTT[0x%x]\n",
		  __func__,
		  master_indi->master_pref,
		  master_indi->random_factor,
		  am_info->amr,
		  am_info->hop_count,
		  am_info->ambtt);

	/* cluster id comparison */
	if (0 == _os_mem_cmp(d, mac_info->nmi.bssid, clus_rpt->cluster_id, MAC_ALEN)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: Cluster ID not changed\n", __func__);
		return;
	}
	if (mac_info->nmi.bssid[0] == 0)
		return;

	phl_nan_set_cluster_id(nan_info, clus_rpt->cluster_id);
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s:New Cluster ID 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n",
		  __func__,
		  mac_info->nmi.bssid[0],
		  mac_info->nmi.bssid[1],
		  mac_info->nmi.bssid[2],
		  mac_info->nmi.bssid[3],
		  mac_info->nmi.bssid[4],
		  mac_info->nmi.bssid[5]);

	phl_nan_cancel_dw_timer(d, nan_info);
	/* rtw_cfgvendor_nan_event_hdr_cluster_changed(pnanpriv->padapter); */
	phl_nan_enqueue_cmd(nan_info->phl_info,
			    NAN_EVENT_CLUSTER_JOIN_DONE,
			    NULL, 0, PHL_CMD_NO_WAIT, 0);
#ifdef CONFIG_NAN_R2
	phl_nan_schdlr_set_clus_info(&nan_info->scheduler_priv);
#endif
	/* to trigger event NAN_EVENT_NEXT_DW for the new cluster */
	SET_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NEW_CLUS_JOINED);
	phl_nan_set_dw_timer(d, nan_info, 1);
}

/**
 * _phl_nan_parse_c2h_tsf_info
 * @nan_info: phl_nan_info
 * @buf: from c2h
 * @size: of the buf
 * (ac) _nan_parse_c2h_tsf_info
 */
enum rtw_phl_status
_phl_nan_parse_c2h_tsf_info(struct phl_nan_info *nan_info, u8 *buf, u32 size)
{
	/* _nan_parse_c2h_tsf_info */
	void *d = phl_nan_info_to_drvpriv(nan_info);
	u32 start_offset = 0;
	if (size < sizeof(struct phl_nan_tsf_info))
		return RTW_PHL_STATUS_RESOURCE;
	_os_mem_cpy(d, &(nan_info->sync_engine_priv.mac_info.tsf_info),
		    ((struct phl_nan_tsf_info *)buf),
		    sizeof(struct phl_nan_tsf_info));
	start_offset =
		nan_info->sync_engine_priv.mac_info.tsf_info.port_dwst_low %
		(DW0_INTERVAL * TU);
	start_offset = start_offset / (NAN_SLOT * TU);
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s: tsf info port dwst[0x%x], fr dwst[0x%x]\n", __func__,
		  nan_info->sync_engine_priv.mac_info.tsf_info.port_dwst_low,
		  nan_info->sync_engine_priv.mac_info.tsf_info.fr_dwst_low);
#ifdef CONFIG_NAN_R2
	phl_nan_schdlr_set_time_info(&nan_info->scheduler_priv,
				     (u16)start_offset,
				     nan_info->sync_engine_priv.mac_info.tsf_info.fr_dwst_low);
#endif
	return RTW_PHL_STATUS_SUCCESS;
}

/**
 * _phl_nan_parse_c2h_cluster_join_notify
 * @nan_info: phl_nan_info
 * @buf: from c2h
 * @size: of the buf
 * (ac) _nan_parse_c2h_cluster_join_notify
 */
enum rtw_phl_status
_phl_nan_parse_c2h_cluster_join_notify(struct phl_nan_info *nan_info,
				       u8 *buf,
				       u32 size)
{
	/* _nan_parse_c2h_cluster_join_notify */
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s\n", __func__);
	SET_STATUS_FLAG(nan_info->sync_engine_priv.flag,
			NAN_MAC_FLAG_ALLOW_JOIN_NEW_CLUSTER);
#ifdef CONFIG_NAN_R2
	phl_nan_schdlr_clus_join_req(&nan_info->scheduler_priv);
#endif
	phl_nan_enqueue_cmd(nan_info->phl_info,
			    NAN_EVENT_JOIN_NEW_CLUSTER,
			    NULL, 0, PHL_CMD_NO_WAIT, 0);
	return RTW_PHL_STATUS_SUCCESS;
}

void phl_nan_c2h_info_parsing(struct phl_nan_info *nan_info, u8 *buf)
{
	struct rtw_phl_nan_info_rpt *nan_rpt = NULL;

	if (!buf) {
		PHL_WARN("%s: got a NULL buf from c2h\n", __func__);
		return;
	}

	nan_rpt = (struct rtw_phl_nan_info_rpt *)buf;
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: c2h type:%d\n",
		  __func__, nan_rpt->c2h_type);
	switch (nan_rpt->c2h_type) {
	case NAN_C2H_TYPE_CLUSTER_INFO:
		_phl_nan_cluster_rpt_hdl(nan_info, &nan_rpt->u.cluster_rpt);
		break;
	case NAN_C2H_TYPE_TSF_INFO:
		/* ret = _phl_nan_parse_c2h_tsf_info(nan_info, buf, size); */
		break;
	case NAN_C2H_TYPE_CLUSTER_JOIN:
		/* ret = _phl_nan_parse_c2h_cluster_join_notify(nan_info,
							     buf, size); */
		break;
	default:
		break;
	}
}

u8
phl_nan_is_peer_info_exist(struct phl_nan_info *nan_info,
			   u8 *peer_addr,
			   struct phl_nan_peer_info_entry **peer_info)
{
	void *d = phl_nan_info_to_drvpriv(nan_info);
	u8 ret = false;

	_os_spinlock(d, &nan_info->peer_info_queue.lock, _bh, NULL);
	phl_list_for_loop((*peer_info),
			  struct phl_nan_peer_info_entry,
			  &nan_info->peer_info_queue.queue, list) {
		if (0 == _os_mem_cmp(d, (*peer_info)->mac, peer_addr, MAC_ALEN)) {
			ret = true;
			break;
		}
	}

	if (ret != true)
		_os_spinunlock(d, &nan_info->peer_info_queue.lock, _bh, NULL);

	return ret;
}

u8 rtw_phl_nan_if_session_on(void *phl_info)
{
	struct phl_nan_info *nan_info = _phl_nan_get_info(phl_info);

	return (TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_START));
}

u8 rtw_phl_nan_if_session_init(void *phl_info)
{
	struct phl_nan_info *nan_info = _phl_nan_get_info(phl_info);

	return (TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_INIT));
}

u8 *rtw_phl_nan_get_wrole_addr(void *phl_info)
{
	struct phl_nan_info *nan_info = _phl_nan_get_info(phl_info);

	return nan_info->wrole_addr;
}

void rtw_phl_nan_set_addr(void *phl_info, u8 *mac_addr)
{
	struct phl_nan_info *nan_info = _phl_nan_get_info(phl_info);
	void *d = phl_nan_info_to_drvpriv(nan_info);

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s:value->mac_addr 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n", __func__,
		  mac_addr[0],
		  mac_addr[1],
		  mac_addr[2],
		  mac_addr[3],
		  mac_addr[4],
		  mac_addr[5]);
	_os_mem_cpy(d, nan_info->wrole_addr, mac_addr, MAC_ALEN);
}

void phl_nan_set_cluster_id(struct phl_nan_info *nan_info, u8 *cluster_id)
{
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_mac_info *macinfo = &nan_info->sync_engine_priv.mac_info;

	_os_spinlock(d, &macinfo->nmi.bssid_lock, _bh, NULL);
	_os_mem_cpy(d, (void *)macinfo->nmi.bssid, (void *)cluster_id, MAC_ALEN);
	_os_spinunlock(d, &macinfo->nmi.bssid_lock, _bh, NULL);
}

void phl_nan_get_cluster_id(struct phl_nan_info *nan_info, u8 *cluster_id)
{
	void *d = phl_nan_info_to_drvpriv(nan_info);
	struct phl_nan_mac_info *macinfo = &nan_info->sync_engine_priv.mac_info;

	_os_spinlock(d, &macinfo->nmi.bssid_lock, _bh, NULL);
	_os_mem_cpy(d, (void *)cluster_id, (void *)macinfo->nmi.bssid, MAC_ALEN);
	_os_spinunlock(d, &macinfo->nmi.bssid_lock, _bh, NULL);
}

enum rtw_phl_status
phl_nan_enqueue_cmd(struct phl_info_t *phl_info,
		    enum _nan_event_type type, void *pbuf, u32 len,
		    enum phl_cmd_type cmd_type, u32 cmd_timeout)
{
	void *drv = phl_info->phl_com->drv_priv;
	struct phl_nan_info *nan_info = _phl_nan_get_info(phl_info);
	enum rtw_phl_status psts = RTW_PHL_STATUS_FAILURE;
	struct phl_nan_event_parm *param = NULL;
	u32 param_len;

	param_len = sizeof(struct phl_nan_event_parm) + len;
	param = _os_kmem_alloc(drv, param_len);
	if (param == NULL) {
		PHL_ERR("%s: alloc param failed!\n", __func__);
		goto _exit;
	}

	_os_mem_set(drv, param, 0, param_len);

	phl_nan_cast_cmd_ptr(type, len, nan_info, param);

	if (len)
		_os_mem_cpy(drv, param->data, pbuf, len);

	if (cmd_type == PHL_CMD_DIRECTLY) {
		psts = phl_cmd_nan_event_hdl(phl_info, (u8 *)param);
		_os_kmem_free(drv, param, param_len);
		goto _exit;
	}

	psts = phl_cmd_enqueue(phl_info,
			       nan_info->nan_rlink->hw_band,
			       MSG_EVT_NAN_ENTRY,
			       (u8 *)param, param_len,
			       _phl_nan_cmd_event_cb,
			       cmd_type, cmd_timeout);
	if (psts == RTW_PHL_STATUS_CMD_TIMEOUT) {
		PHL_ERR("%s: Send cmd succeeded, but wait cmd failed (%d)\n",
			__func__, psts);
	} else if (psts != RTW_PHL_STATUS_SUCCESS) {
		/* Send cmd failed or send cmd succeeded but did not execute */
		PHL_ERR("%s: Send cmd failed (%d)\n", __func__, psts);
		if (!is_cmd_failure(psts))
			_os_kmem_free(drv, param, param_len);
		psts = RTW_PHL_STATUS_FAILURE;
	}

_exit:
	return psts;
}

void rtw_phl_nan_init_ops(void *phl, struct rtw_phl_nan_ops *ops)
{
	struct phl_nan_info *nan_info = &(((struct phl_info_t *)phl)->nan_info);
	nan_info->nan_ops.prepare_ies = ops->prepare_ies;
	nan_info->nan_ops.tx_mgnt = ops->tx_mgnt;
	nan_info->nan_ops.get_attr_start = ops->get_attr_start;
	nan_info->nan_ops.get_frame_body = ops->get_frame_body;
	nan_info->nan_ops.report_to_osdep = ops->report_to_osdep;
	nan_info->nan_ops.free_osdep_data = ops->free_osdep_data;
	nan_info->nan_ops.alloc_peer_sta_info = ops->alloc_peer_sta_info;
	nan_info->nan_ops.free_peer_sta_info = ops->free_peer_sta_info;
#ifdef CONFIG_NAN_R2
	nan_info->nan_ops.pmk_to_ptk = ops->pmk_to_ptk;
	nan_info->nan_ops.get_key_mic = ops->get_key_mic;
	nan_info->nan_ops.get_auth_token = ops->get_auth_token;
	nan_info->nan_ops.get_pmkid = ops->get_pmkid;
	nan_info->nan_ops.set_pairwise_key = ops->set_pairwise_key;
#endif
}

enum rtw_phl_status
rtw_phl_nan_on_action_public_nan(void* phl, u8 *precv_frame,
				 u8 *frame_body, u16 precv_frame_len,
				 u8 is_privacy)
{
	struct phl_nan_info *nan_info = _phl_nan_get_info(phl);
	void *d = phl_nan_info_to_drvpriv(nan_info);

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s: type[%d]\n", __func__, *(frame_body + 2));

	if (!TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_START))
		return RTW_PHL_STATUS_FAILURE;

	if (!rtw_phl_nan_is_nan_frame(phl, precv_frame))
		return RTW_PHL_STATUS_FAILURE;

	if (_os_mem_cmp(d, frame_body + 2, NAN_OUI_SDF, 4) == 0) {
		phl_nan_de_on_sdf(&nan_info->discovery_engine_priv,
				  precv_frame, precv_frame_len);
	}
#ifdef CONFIG_NAN_R2
	else if (_os_mem_cmp(d, frame_body + 2, NAN_OUI_NAF, 4) == 0) {
		phl_nan_de_on_naf(&nan_info->discovery_engine_priv, precv_frame,
				  frame_body, precv_frame_len, is_privacy);
	}
#endif

	return RTW_PHL_STATUS_SUCCESS;
}

u8 rtw_phl_nan_is_nan_frame(void *phl, u8 *pframe)
{
	u8 ret = false;
	struct phl_nan_info *nan_info = _phl_nan_get_info(phl);
	struct phl_nan_mac_info *macinfo = &nan_info->sync_engine_priv.mac_info;
	void *d = phl_nan_info_to_drvpriv(nan_info);
#ifdef CONFIG_NAN_R2
	struct phl_nan_schdl_ndc_info_entry *p_ndc_info_entry = NULL;
#endif
	u8 cluster_id[MAC_ALEN] = {0};

	if (!TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_START))
		goto exit;

	phl_nan_get_cluster_id(nan_info, cluster_id);

	if (((_os_mem_cmp(d, pframe + 4, macinfo->nmi.mac, ETH_ALEN) == 0) ||
	     (_os_mem_cmp(d, pframe + 4, nan_network_id, ETH_ALEN) == 0)) &&
	    ((_os_mem_cmp(d, pframe + 16, cluster_id, ETH_ALEN) == 0) ||
#ifdef CONFIG_NAN_R2
	     true == phl_nan_schdlr_is_ndc_info_exist(&nan_info->scheduler_priv,
						      pframe + 16,
						      &p_ndc_info_entry) ||
#endif
	     0))
		ret = true;
exit:
	return ret;
}

u8 rtw_phl_nan_replace_with_nan_mac(void *phl, u8 *frame_ptr, u8 *dst)
{
	u8 ret = false;
	struct phl_nan_info *nan_info = _phl_nan_get_info(phl);
	struct phl_nan_mac_info *macinfo = &nan_info->sync_engine_priv.mac_info;
	void *d = phl_nan_info_to_drvpriv(nan_info);

	if (!TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_START))
		goto exit;

	if (_os_mem_cmp(d, dst, macinfo->nmi.mac, ETH_ALEN) == 0) {
		_os_mem_cpy(d, frame_ptr, nan_info->wrole_addr, ETH_ALEN);
		ret = true;
		goto exit;
	}

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s: dst =/= nmi ? dst 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n",
		  __func__, dst[0], dst[1], dst[2], dst[3], dst[4], dst[5]);
exit:
	return ret;
}

u8
rtw_phl_nan_set_srvc_ext_info(void *phl,
			      struct rtw_phl_nan_srvc_ext_info *ext_info)
{
	struct phl_info_t *phl_info = phl;
	void *d = phl_to_drvpriv(phl_info);
	struct rtw_phl_nan_srvc_ext_info *srvc_ext =
		&phl_info->nan_info.discovery_engine_priv.srvc_ext_info;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s\n", __func__);

	if (!rtw_phl_nan_if_session_on(phl_info)) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
			  "%s: NAN has not started!\n", __func__);
		return false;
	}

	_os_mem_cpy(d, srvc_ext, ext_info, sizeof(struct rtw_phl_nan_srvc_ext_info));
	srvc_ext->updating = true;

	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  "%s service_id: %02x:%02x:%02x:%02x:%02x:%02x\n",
		  __func__,
		  srvc_ext->service_id[0],
		  srvc_ext->service_id[1],
		  srvc_ext->service_id[2],
		  srvc_ext->service_id[3],
		  srvc_ext->service_id[4],
		  srvc_ext->service_id[5]);

	return true;
}

void
rtw_phl_nan_stop_directly(void *phl)
{
	struct phl_info_t *phl_info = (struct phl_info_t *)phl;
	struct phl_nan_info *nan_info = NULL;

	nan_info = &phl_info->nan_info;

	_phl_nan_session_stop(phl_info, true);
	_phl_nan_session_deinit(phl_info);
	_phl_nan_hw_deinit(phl_info, nan_info->nan_wrole, nan_info->nan_rlink);

	nan_info->nan_wrole = NULL;
	nan_info->nan_rlink = NULL;
}

static enum phl_mdl_ret_code
_nan_ss_req_acquired(void *dispr, void *priv)
{
	struct rtw_phl_nan_ss_param *param = (struct rtw_phl_nan_ss_param *)priv;
	struct rtw_phl_com_t *phl_com = param->phl_role->phl_com;
	struct phl_info_t *phl_info = phl_com->phl_priv;
	struct phl_msg msg = {0};
	struct phl_msg_attribute attr = {0};
	enum rtw_phl_status status = RTW_PHL_STATUS_SUCCESS;
	u8 band_idx = 0;

	FUNCIN();

	phl_dispr_get_idx(dispr, &band_idx);
	msg.band_idx = band_idx;

	SET_MSG_MDL_ID_FIELD(msg.msg_id, PHL_FG_MDL_NAN_START_STOP);
	if (param->start)
		SET_MSG_EVT_ID_FIELD(msg.msg_id, MSG_EVT_NAN_START);
	else
		SET_MSG_EVT_ID_FIELD(msg.msg_id, MSG_EVT_NAN_STOP);

	status = phl_disp_eng_send_msg(phl_info, &msg, &attr, NULL);

	if (status != RTW_PHL_STATUS_SUCCESS) {
		if (param->start) {
			param->ops->nan_start_complete_notify(param);
		} else {
			param->ops->nan_stop_prepare_notify(param);
			rtw_phl_nan_stop_directly(phl_info);
			param->ops->nan_stop_complete_notify(param);
		}
		return MDL_RET_FAIL;
	} else {
		return MDL_RET_SUCCESS;
	}
}

static enum phl_mdl_ret_code
_nan_ss_msg_hdlr(void *dispr, void *priv, struct phl_msg* msg)
{
	enum rtw_phl_status status = RTW_PHL_STATUS_FAILURE;
	struct rtw_phl_nan_ss_param *param = NULL;
	struct rtw_phl_nan_ie_container ies = {0};
	struct phl_info_t *phl_info = NULL;
	struct phl_nan_info *nan_info = NULL;
	struct phl_msg nextmsg = {0};
	struct phl_msg_attribute attr = {0};
	u8 band_idx = 0;

	param = (struct rtw_phl_nan_ss_param *)priv;
	phl_info = param->phl_role->phl_com->phl_priv;
	nan_info = &phl_info->nan_info;

	if (MSG_MDL_ID_FIELD(msg->msg_id) != PHL_FG_MDL_NAN_START_STOP)
		return MDL_RET_IGNORE;

	if (IS_MSG_FAIL(msg->msg_id)) {
		PHL_INFO("%s :: MSG(%d)_FAIL - EVT_ID=%d \n", __func__,
			 MSG_MDL_ID_FIELD(msg->msg_id), MSG_EVT_ID_FIELD(msg->msg_id));
	}

	phl_dispr_get_idx(dispr, &band_idx);
	nextmsg.band_idx = band_idx;
	SET_MSG_MDL_ID_FIELD(nextmsg.msg_id, PHL_FG_MDL_NAN_START_STOP);
	nextmsg.rsvd[0].ptr = (u8 *)param->rlink;

	switch (MSG_EVT_ID_FIELD(msg->msg_id)) {
	case MSG_EVT_NAN_START:
		PHL_INFO("[%d]MSG_EVT_NAN_START \n", band_idx);

		param->rlink->bcn_cmn.role_idx = param->phl_role->id;

		status = _phl_nan_hw_init(phl_info, param->phl_role, param->rlink);
		if (status != RTW_PHL_STATUS_SUCCESS) {
			PHL_ERR("%s - (_phl_nan_hw_init) fail with error = %d\n",
				__func__, status);
			goto exit_stop;
		}

		nan_info->nan_wrole = param->phl_role;
		nan_info->nan_rlink = param->rlink;

		_phl_nan_session_init(phl_info);

		nan_info->nan_ops.prepare_ies(param->priv, param->bands, &ies);

		status = _phl_nan_session_start(phl_info,
						param->nan_cmd_type,
						param->master_pref,
						param->bands,
						&ies);
		if (status != RTW_PHL_STATUS_SUCCESS) {
			PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_, "%s: fail\n", __func__);
			goto exit_stop;
		}

		SET_MSG_EVT_ID_FIELD(nextmsg.msg_id, MSG_EVT_NAN_START_END);

		status = phl_disp_eng_send_msg(phl_info, &nextmsg, &attr, NULL);
		if (status != RTW_PHL_STATUS_SUCCESS) {
			PHL_ERR("%s :: [MSG_EVT_NAN_START] send_msg failed\n", __func__);
			rtw_phl_nan_stop_directly(phl_info);
			status = phl_disp_eng_free_token(phl_info, band_idx, &param->token);
			param->ops->nan_stop_complete_notify(param);
		}

		break;
	case MSG_EVT_NAN_START_END:
		PHL_INFO("[%d]MSG_EVT_NAN_START_END \n", band_idx);

		status = phl_disp_eng_free_token(phl_info, band_idx, &param->token);
		if (status == RTW_PHL_STATUS_SUCCESS)
			param->ops->nan_start_complete_notify(param);
		else
			PHL_WARN("%s :: [MSG_EVT_NAN_START_END] Abort occurred, skip!\n", __func__);
		break;
	case MSG_EVT_NAN_STOP:
		PHL_INFO("[%d]MSG_EVT_NAN_STOP \n", band_idx);

		param->ops->nan_stop_prepare_notify(param);

		_phl_nan_session_stop(phl_info, param->is_rmmod);
		_phl_nan_session_deinit(phl_info);
		_phl_nan_hw_deinit(phl_info, param->phl_role, param->rlink);

		nan_info->nan_wrole = NULL;
		nan_info->nan_rlink = NULL;

		SET_MSG_EVT_ID_FIELD(nextmsg.msg_id, MSG_EVT_NAN_STOP_END);

		status = phl_disp_eng_send_msg(phl_info, &nextmsg, &attr, NULL);
		if (status != RTW_PHL_STATUS_SUCCESS) {
			PHL_ERR("%s :: [MSG_EVT_NAN_STOP] send_msg failed\n", __func__);
			status = phl_disp_eng_free_token(phl_info, band_idx, &param->token);
			param->ops->nan_stop_complete_notify(param);
		}

		break;
	case MSG_EVT_NAN_STOP_END:
		PHL_INFO("[%d]MSG_EVT_NAN_STOP_END \n", band_idx);

		status = phl_disp_eng_free_token(phl_info, band_idx, &param->token);
		if (status == RTW_PHL_STATUS_SUCCESS) {
			param->ops->nan_stop_complete_notify(param);
		} else {
			PHL_WARN("%s :: [MSG_EVT_NAN_STOP_END] Abort occurred, skip!\n", __func__);
		}

		break;
	default:
		/* unknown state */
		break;
	}

	return MDL_RET_SUCCESS;

exit_stop:
	SET_MSG_EVT_ID_FIELD(nextmsg.msg_id, MSG_EVT_NAN_STOP);
	status = phl_disp_eng_send_msg(phl_info, &nextmsg, &attr, NULL);
	if (status != RTW_PHL_STATUS_SUCCESS) {
		rtw_phl_nan_stop_directly(phl_info);
		status = phl_disp_eng_free_token(phl_info, band_idx, &param->token);
		param->ops->nan_stop_complete_notify(param);
	}
	return MDL_RET_SUCCESS;
}

static enum phl_mdl_ret_code
_nan_ss_req_abort(void *dispr, void *priv)
{
	struct rtw_phl_nan_ss_param *param = (struct rtw_phl_nan_ss_param *)priv;
	struct rtw_phl_com_t *phl_com = param->phl_role->phl_com;
	struct phl_info_t *phl_info = phl_com->phl_priv;
	struct phl_nan_info *nan_info = _phl_nan_get_info(phl_info);
	struct phl_msg msg = {0};
	struct phl_msg_attribute attr = {0};
	enum rtw_phl_status status = RTW_PHL_STATUS_SUCCESS;
	u8 band_idx = 0;

	FUNCIN();

	phl_dispr_get_idx(dispr, &band_idx);

	msg.band_idx = band_idx;

	if (TEST_STATUS_FLAG(nan_info->flag, NAN_PRIV_FLAG_NAN_SESSION_START)) {
		param->ops->nan_stop_prepare_notify(param);
		rtw_phl_nan_stop_directly(phl_info);
	}

	SET_MSG_MDL_ID_FIELD(msg.msg_id, PHL_FG_MDL_NAN_START_STOP);
	SET_MSG_EVT_ID_FIELD(msg.msg_id, MSG_EVT_NAN_STOP_END);

	status = phl_disp_eng_send_msg(phl_info, &msg, &attr, NULL);
	if (status != RTW_PHL_STATUS_SUCCESS)
		param->ops->nan_stop_complete_notify(param);

	return MDL_RET_SUCCESS;
}

static enum phl_mdl_ret_code
_nan_ss_query_info(void *dispr, void *priv, struct phl_module_op_info* info)
{
	struct rtw_phl_nan_ss_param *param = (struct rtw_phl_nan_ss_param*)priv;
	enum phl_mdl_ret_code ret = MDL_RET_SUCCESS;

	switch (info->op_code) {
	case FG_REQ_OP_GET_ROLE:
		info->outbuf = (u8 *)param->phl_role;
		break;
	case FG_REQ_OP_GET_ROLE_LINK:
		info->outbuf = (u8 *)param->rlink;
		break;
	case FG_REQ_OP_GET_MDL_ID:
		*(info->outbuf) = PHL_FG_MDL_NAN_START_STOP;
		break;
	default:
		ret = MDL_RET_IGNORE;
		break;
	}

	return ret;
}

static enum phl_mdl_ret_code
_nan_ss_set_info(void *dispr, void *priv, struct phl_module_op_info* info)
{
	return MDL_RET_SUCCESS;
}

static void
_nan_ss_req_init(struct phl_cmd_token_req *fgreq, struct rtw_phl_nan_ss_param *param)
{
	fgreq->module_id = PHL_FG_MDL_NAN_START_STOP;
	fgreq->opt = FG_CMD_OPT_EXCLUSIVE;
	fgreq->priv = param;
	fgreq->role = param->phl_role;
	fgreq->acquired = _nan_ss_req_acquired;
	fgreq->abort = _nan_ss_req_abort;
	fgreq->msg_hdlr = _nan_ss_msg_hdlr;
	fgreq->set_info = _nan_ss_set_info;
	fgreq->query_info = _nan_ss_query_info;
}

enum rtw_phl_status
rtw_phl_cmd_nan_ss_req(void *phl, struct rtw_phl_nan_ss_param *param)
{
	enum rtw_phl_status pstatus = RTW_PHL_STATUS_FAILURE;
	struct phl_info_t *phl_info = (struct phl_info_t *)phl;
	struct phl_nan_info *nan_info = &phl_info->nan_info;
	struct phl_cmd_token_req fqreq = {0};

	_nan_ss_req_init(&fqreq, param);

	pstatus = phl_disp_eng_add_token_req(phl_info, param->rlink->hw_band,
					     &fqreq, &param->token);
	if ((pstatus == RTW_PHL_STATUS_SUCCESS) ||
	    (pstatus == RTW_PHL_STATUS_PENDING))
		pstatus = RTW_PHL_STATUS_SUCCESS;

	nan_info->sync_engine_priv.start_stop_cmd_token = param->token;

	return pstatus;
}
#endif /* CONFIG_PHL_NAN */
