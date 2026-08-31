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
#define _PHL_CUSTOMIZE_FEATURE_C_
#include "../phl_headers.h"
#include "../phl_api.h"

#ifdef CONFIG_PHL_CUSTOM_FEATURE_VR
#ifdef CONFIG_PHL_CHANNEL_INFO_VR
#include "phl_custom_vr_csi.h"

enum csi_status {
	CSI_STATE_INIT = BIT0,
	CSI_STATE_RUNNING = BIT1,
};

void _phl_custom_csi_init(struct custom_csi_ctrl *csi_ctrl)
{
	/* param */
	csi_ctrl->param.light_mode = 1;
	csi_ctrl->param.bitmap_1x1 = 1;
	csi_ctrl->param.group_num = 0;
	csi_ctrl->param.ap_role = true;
	csi_ctrl->param.fb_en = false;
	csi_ctrl->param.trig_mode = CHINFO_MODE_ACK;
	csi_ctrl->param.fb_mode = 0;
	csi_ctrl->param.trig_intv = 0;
	csi_ctrl->param.rslt_qry_en = true;
	csi_ctrl->param.rslt_qry_mode = 0;
	/* info */
	csi_ctrl->info.sta = NULL;
}

struct rtw_phl_stainfo_t *
_ap_get_first_client_sta(void *phl,
                         struct rtw_wifi_role_link_t *rlink,
                         struct rtw_phl_stainfo_t *self)
{
	struct phl_info_t *phl_info = (struct phl_info_t *)phl;
	void *drv = phl_to_drvpriv(phl_info);
	struct rtw_phl_stainfo_t *n, *psta, *ret = NULL;

	if (rlink->assoc_sta_queue.cnt > 1) {
		_os_spinlock(drv, &rlink->assoc_sta_queue.lock, _bh, NULL);
		phl_list_for_loop_safe(psta, n, struct rtw_phl_stainfo_t,
			&rlink->assoc_sta_queue.queue, list) {
		if (_os_mem_cmp(drv, self->mac_addr,
					psta->mac_addr, MAC_ALEN) == 0) {
				continue;
			} else {
				ret = psta;
				break;
			}
		}
		_os_spinunlock(drv, &rlink->assoc_sta_queue.lock, _bh, NULL);
	}
	return ret;
}

static u8 _is_sta_ready(void* custom_ctx,
                        struct rtw_wifi_role_link_t *rlink,
                        struct rtw_chinfo_action_parm *act_parm)
{
	struct phl_info_t *phl_info = phl_custom_get_phl_info(custom_ctx);
	struct rtw_phl_stainfo_t *phl_sta = NULL;

	if (rlink->mstate != MLME_LINKED) {
		PHL_ERR("%s, Not connected!\n", __func__);
		return 0;
	}

	phl_sta = rtw_phl_get_stainfo_self(phl_info, rlink);
	if (phl_sta == NULL || phl_sta->active != true) {
		PHL_ERR("%s, phl_sta not available, return!\n", __func__);
		return 0;
	} else {
		PHL_INFO("%s, macid(%d), peer mac addr %02X-%02X-%02X-%02X-%02X-%02X\n",
			__func__, phl_sta->macid,
			phl_sta->mac_addr[0], phl_sta->mac_addr[1],
			phl_sta->mac_addr[2], phl_sta->mac_addr[3],
			phl_sta->mac_addr[4], phl_sta->mac_addr[5]);
		act_parm->sta = phl_sta;
	}
	return 1;
}

static u8 _is_ap_ready(void* custom_ctx,
                       struct custom_csi_ctrl *csi_ctrl,
                       struct rtw_wifi_role_link_t *rlink,
                       struct rtw_chinfo_action_parm *act_parm)
{
	struct phl_info_t *phl_info = phl_custom_get_phl_info(custom_ctx);
	struct rtw_phl_stainfo_t *phl_sta = NULL;

	if (rlink->mstate != MLME_LINKED) {
		PHL_ERR("%s, Not connected!\n", __func__);
		return 0;
	} else {
		phl_sta = rtw_phl_get_stainfo_self(phl_info, rlink);
		if (phl_sta == NULL || phl_sta->active != true) {
			PHL_ERR("%s, phl_sta not available\n", __func__);
			return 0;
		} else {
			struct rtw_phl_stainfo_t *psta = NULL;

			PHL_INFO("%s, self macid(%d), mac addr %02X-%02X-%02X-%02X-%02X-%02X\n",
				__func__, phl_sta->macid,
				phl_sta->mac_addr[0], phl_sta->mac_addr[1],
				phl_sta->mac_addr[2], phl_sta->mac_addr[3],
				phl_sta->mac_addr[4], phl_sta->mac_addr[5]);
			psta = _ap_get_first_client_sta(phl_info, rlink, phl_sta);
			if (psta != NULL) {
				PHL_INFO("%s, found assoc sta, macid(%d), mac addr %02X-%02X-%02X-%02X-%02X-%02X\n",
					__func__, psta->macid,
					psta->mac_addr[0], psta->mac_addr[1],
					psta->mac_addr[2], psta->mac_addr[3],
					psta->mac_addr[4], psta->mac_addr[5]);
				act_parm->sta = psta;
			} else {
				PHL_ERR("%s, assoc client not found.\n", __func__);
				return 0;
			}
		}
	}

	return 1;
}

enum phl_mdl_ret_code
rtw_phl_custom_csi_start(void* custom_ctx,
                         struct custom_csi_ctrl *csi_ctrl,
                         struct rtw_wifi_role_link_t *rlink)
{
	enum phl_mdl_ret_code ret = MDL_RET_SUCCESS;
	struct phl_info_t *phl_info = phl_custom_get_phl_info(custom_ctx);
	struct rtw_chinfo_action_parm act_parm = {0};
	u8 ready = 0;

	PHL_INFO("%s, ==>\n", __func__);

	SET_STATUS_FLAG(csi_ctrl->status, CSI_STATE_INIT);

	_phl_custom_csi_init(csi_ctrl);

	if (csi_ctrl->param.ap_role)
		ready = _is_ap_ready(custom_ctx, csi_ctrl, rlink, &act_parm);
	else
		ready = _is_sta_ready(custom_ctx, rlink, &act_parm);

	if (ready != 1) {
		ret = MDL_RET_FAIL;
		return ret;
	}

	csi_ctrl->info.sta = act_parm.sta;

	act_parm.act = CHINFO_ACT_CFG;
	act_parm.mode = csi_ctrl->param.trig_mode;
	if (csi_ctrl->param.light_mode == 1)
		act_parm.enable_mode = CHINFO_EN_LIGHT_MODE;
	else
		act_parm.enable_mode = CHINFO_EN_RICH_MODE;
	act_parm.accuracy = CHINFO_ACCU_1BYTE;
	act_parm.group_num = csi_ctrl->param.group_num;
	act_parm.enable = true;
	if (csi_ctrl->param.trig_intv > 0)
		act_parm.trig_period = csi_ctrl->param.trig_intv;
	else
		act_parm.trig_period = 30;
	if (csi_ctrl->param.bitmap_1x1 == 1)
		act_parm.ele_bitmap = 0x0101;
	else
		act_parm.ele_bitmap = 0x0303;
	rtw_phl_cmd_cfg_chinfo(phl_info, &act_parm, PHL_CMD_NO_WAIT, 0);
	act_parm.act = CHINFO_ACT_EN;
	rtw_phl_cmd_cfg_chinfo(phl_info, &act_parm, PHL_CMD_NO_WAIT, 0);

	SET_STATUS_FLAG(csi_ctrl->status, CSI_STATE_RUNNING);

	return ret;
}

enum phl_mdl_ret_code
rtw_phl_custom_csi_stop(void* custom_ctx, struct custom_csi_ctrl *csi_ctrl)
{
	enum phl_mdl_ret_code ret = MDL_RET_SUCCESS;
	struct phl_info_t *phl_info = phl_custom_get_phl_info(custom_ctx);
	struct rtw_chinfo_action_parm act_parm = {0};

	PHL_INFO("%s, ==>\n", __func__);

	if (!TEST_STATUS_FLAG(csi_ctrl->status, CSI_STATE_RUNNING)) {
		ret = MDL_RET_FAIL;
		return ret;
	}

	CLEAR_STATUS_FLAG(csi_ctrl->status, CSI_STATE_RUNNING);

	PHL_INFO("\n* Stop CSI for %s side.\n",
		(csi_ctrl->param.ap_role ? "AP" : "CLIENT"));

	act_parm.sta = csi_ctrl->info.sta;

	act_parm.act = CHINFO_ACT_CFG;
	act_parm.mode = csi_ctrl->param.trig_mode;
	if (csi_ctrl->param.light_mode == 1)
		act_parm.enable_mode = CHINFO_EN_LIGHT_MODE;
	else
		act_parm.enable_mode = CHINFO_EN_RICH_MODE;
	act_parm.accuracy = CHINFO_ACCU_1BYTE;
	act_parm.group_num = csi_ctrl->param.group_num;
	act_parm.enable = false; /* set FALSE to disable CSI */
	if (csi_ctrl->param.trig_intv > 0)
		act_parm.trig_period = csi_ctrl->param.trig_intv;
	else
		act_parm.trig_period = 30;
	if (csi_ctrl->param.bitmap_1x1 == 1)
		act_parm.ele_bitmap = 0x0101;
	else
		act_parm.ele_bitmap = 0x0303;
	rtw_phl_cmd_cfg_chinfo(phl_info, &act_parm, PHL_CMD_NO_WAIT, 0);
	act_parm.act = CHINFO_ACT_EN;
	rtw_phl_cmd_cfg_chinfo(phl_info, &act_parm, PHL_CMD_NO_WAIT, 0);

	CLEAR_STATUS_FLAG(csi_ctrl->status, CSI_STATE_INIT);

	return ret;
}

enum phl_mdl_ret_code
rtw_phl_custom_csi_rslt_query(void* custom_ctx, u8 *buf, u32 *len)
{
	struct phl_info_t *phl_info = phl_custom_get_phl_info(custom_ctx);
	struct csi_header_t csi_hdr = {0};
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;

	if (RTW_PHL_STATUS_SUCCESS == rtw_phl_query_chan_info(phl_info,
		MAX_DATA_SIZE, buf, len, &csi_hdr))
		ret = MDL_RET_SUCCESS;
	else
		PHL_ERR("%s, query chan info fail\n", __func__);

	return ret;
}

#endif /* CONFIG_PHL_CHANNEL_INFO_VR */
#endif /* CONFIG_PHL_CUSTOM_FEATURE_VR */
