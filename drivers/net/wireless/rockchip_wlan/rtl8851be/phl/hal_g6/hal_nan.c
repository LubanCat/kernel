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
#define _HAL_NAN_C_
#include "hal_headers.h"

#ifdef CONFIG_PHL_NAN
enum rtw_hal_status
rtw_hal_nan_send_bcn(void *hal,
		     struct rtw_wifi_role_link_t *rlink,
		     u8 dual,
		     enum rtw_nan_bcn_type nan_bcn_type,
		     u16 bcn_macid,
		     struct rtw_bcn_info_cmn *bcn)
{
	enum rtw_hal_status hstatus = RTW_HAL_STATUS_FAILURE;
	struct rtw_phl_nan_send_bcn m_nan_send_bcn = {0};
	struct hal_info_t *hal_info = hal;

	/* common setting */
	m_nan_send_bcn.band = rlink->hw_band;
	m_nan_send_bcn.macid = (u8)bcn_macid;
	m_nan_send_bcn.rate_sel = (u16)bcn->bcn_rate;

	if (bcn->bcn_offload & BIT(BCN_HW_SEQ)) {
		m_nan_send_bcn.ssn_sel = 1;
		m_nan_send_bcn.ssn_mode = 1;
	} else {
		m_nan_send_bcn.ssn_sel = 0;
		m_nan_send_bcn.ssn_mode = 0;
	}

	m_nan_send_bcn.pld_buf = bcn->bcn_buf;
	m_nan_send_bcn.pld_len = (u16)bcn->bcn_length;
	if (nan_bcn_type == NAN_BCN_TYPE_SYNC)
		m_nan_send_bcn.port = rlink->hw_port;
	else
		m_nan_send_bcn.port = rlink->ext_hw_port;

	hstatus = rtw_hal_mac_nan_mport_config(hal_info,
					       nan_bcn_type,
					       m_nan_send_bcn.port,
					       dual,
					       true);
	if (RTW_HAL_STATUS_SUCCESS != hstatus)
		return hstatus;

	hstatus = rtw_hal_mac_nan_send_bcn_content(hal_info, &m_nan_send_bcn);
	if (RTW_HAL_STATUS_SUCCESS != hstatus)
		return hstatus;

	return RTW_HAL_STATUS_SUCCESS;
}

enum rtw_hal_status
rtw_hal_nan_rmv_bcn(void *hal,
		    struct rtw_wifi_role_link_t *rlink,
		    enum rtw_nan_bcn_type nan_bcn_type)
{
	enum rtw_hal_status hstatus = RTW_HAL_STATUS_FAILURE;
	struct hal_info_t *hal_info = hal;
	u8 bcn_hw_port = HW_PORT_MAX;

	if (nan_bcn_type == NAN_BCN_TYPE_SYNC)
		bcn_hw_port = rlink->hw_port;
	else
		bcn_hw_port = rlink->ext_hw_port;

	hstatus = rtw_hal_mac_nan_mport_config(hal_info,
					       nan_bcn_type,
					       bcn_hw_port,
					       false,
					       false);
	if (RTW_HAL_STATUS_SUCCESS != hstatus) {
		PHL_TRACE(COMP_PHL_NAN, _PHL_ERR_,
			  "%s: disable mport config fail (%d), port(%d)\n",
			  __func__, hstatus, rlink->hw_port);
	}

	return hstatus;
}

enum rtw_hal_status
rtw_hal_nan_set_skd_dw_and_disc_bcn(void *hal,
				    struct rtw_wifi_role_link_t *rlink,
				    u8 is_rmv,
				    u8 ch)
{
	enum rtw_hal_status hstatus = RTW_HAL_STATUS_FAILURE;
	struct rtw_phl_act_skd_req_para m_nan_skd = {0};
	struct hal_info_t *hal_info = hal;
	void *drv = hal_to_drvpriv(hal_info);
	/* TODO: remove if not used */
	u32 skd_id = 0;

	/* send action skd req for dw */
	m_nan_skd.module_id = NAN_ACT_REQ_MODULE_NAN_DW;
	m_nan_skd.tsf_idx = rlink->hw_port;
	m_nan_skd.priority = 0; /* NAN_DW_SCHEDULE_PRIORITY */
	m_nan_skd.channel = ch;
	m_nan_skd.prim_chnl = ch;
	m_nan_skd.ch_band_type = (m_nan_skd.channel >= 1 && m_nan_skd.channel <= 14) ?
				 BAND_ON_24G : BAND_ON_5G;
	if (is_rmv == true)
		SET_STATUS_FLAG(m_nan_skd.options, NAN_ACT_REQ_OPT_REMOVE);

	hstatus = rtw_hal_mac_nan_act_skd_req(hal_info, &skd_id, &m_nan_skd);
	if (RTW_HAL_STATUS_SUCCESS != hstatus)
		return hstatus;

#ifdef CONFIG_PHL_NAN_DEBUG
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  " m_nan_skd => (sync bcn) module_id(%x), tsf_idx(%d), priority(%d), options(%x) on ch(%d)\n",
		  m_nan_skd.module_id, m_nan_skd.tsf_idx,
		  m_nan_skd.priority,  m_nan_skd.options, m_nan_skd.channel);
#endif

	/* send action skd req for disc bcn */
	_os_mem_set(drv, &m_nan_skd, 0, sizeof(struct rtw_phl_act_skd_req_para));

	m_nan_skd.module_id = NAN_ACT_REQ_MODULE_NAN_DISC_BCN;
	m_nan_skd.tsf_idx = rlink->ext_hw_port;
	m_nan_skd.priority = 0; /* NAN_DW_SCHEDULE_PRIORITY */
	m_nan_skd.channel = ch;
	m_nan_skd.prim_chnl = ch;
	m_nan_skd.ch_band_type = (m_nan_skd.channel >= 1 && m_nan_skd.channel <= 14) ?
				 BAND_ON_24G : BAND_ON_5G;
	if (is_rmv == true)
		SET_STATUS_FLAG(m_nan_skd.options, NAN_ACT_REQ_OPT_REMOVE);

	hstatus = rtw_hal_mac_nan_act_skd_req(hal_info, &skd_id, &m_nan_skd);
	if (RTW_HAL_STATUS_SUCCESS != hstatus)
		return hstatus;

#ifdef CONFIG_PHL_NAN_DEBUG
	PHL_TRACE(COMP_PHL_NAN, _PHL_INFO_,
		  " m_nan_skd => (disc bcn) module_id(%x), tsf_idx(%d), priority(%d), options(%x) on ch(%d)\n\n",
		  m_nan_skd.module_id, m_nan_skd.tsf_idx,
		  m_nan_skd.priority,  m_nan_skd.options, m_nan_skd.channel);
#endif

	return RTW_HAL_STATUS_SUCCESS;
}

enum rtw_hal_status
rtw_hal_nan_func_ctrl(void *hal, struct rtw_phl_nan_func_ctrl *nan_ctrl)
{
	struct hal_info_t *hal_info = hal;
	return rtw_hal_mac_nan_func_ctrl(hal_info, nan_ctrl);
}

#endif /* CONFIG_PHL_NAN */
