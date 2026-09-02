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

#ifdef CONFIG_PHL_CUSTOM_FEATURE_VR
#include "phl_custom_vr.h"
#include "phl_custom_vr_csi.h"

enum phl_mdl_ret_code
_is_vr_mode_valid(void* custom_ctx,
                  struct _custom_vr_ctx* vr_ctx,
                  struct phl_msg* msg, u32 size)
{
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	enum phl_mdl_ret_code ret = MDL_RET_SUCCESS;
	if (!vr_ctx->init.enable || cmd->len < size || vr_ctx->init.wifi_role == NULL) {
		PHL_INFO(" %s, evt_id(%d) not accepted\n",
		         __FUNCTION__,
		         MSG_EVT_ID_FIELD(msg->msg_id));
		ret = MDL_RET_FAIL;
		return ret;
	}

	return ret;
}

bool
_feature_vr_enable_init_setting(void* custom_ctx,
                             struct _custom_vr_ctx* vr_ctx)
{
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_hal_com_t *hal_com = rtw_hal_get_halcom(phl->hal);

	vr_ctx->init.phl = phl;
#ifdef CONFIG_USB_HCI
	/* To Do: to offload gpio number in halmac for different ICs */
	rtw_hal_set_sw_gpio_mode(phl->phl_com, phl->hal
		, RTW_AX_SW_IO_MODE_OUTPUT_OD, 12);

	rtw_hal_set_sw_gpio_mode(phl->phl_com, phl->hal
		, RTW_AX_SW_IO_MODE_OUTPUT_OD, 14);
#endif
	rtw_hal_auto_debug_en_phy_util(hal_com, true);

	return true;
}

void
_feature_vr_enable_deinit_setting(void* custom_ctx,
                               struct _custom_vr_ctx* vr_ctx)
{
	/* TBD: merge codes from custom branch for the deinit setting*/
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_hal_com_t *hal_com = rtw_hal_get_halcom(phl->hal);

	rtw_hal_auto_debug_en_phy_util(hal_com, false);

	return;
}

enum phl_mdl_ret_code
_phl_custom_vr_feature_enable(void* custom_ctx,
                          struct _custom_vr_ctx* vr_ctx,
                          struct phl_msg* msg)
{
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);

	if (cmd->len < sizeof(u32))
		return MDL_RET_FAIL;

	vr_ctx->init.enable = *(u32*)(cmd->data);
	vr_ctx->init.test_mode = 0;
	if (vr_ctx->init.enable){
		_feature_vr_enable_init_setting(custom_ctx, vr_ctx);
	} else {
		_feature_vr_enable_deinit_setting(custom_ctx, vr_ctx);
	}

	PHL_INFO(" %s, vr feature enable(%d)\n", __FUNCTION__, vr_ctx->init.enable);
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&(vr_ctx->init.enable),
	                           sizeof(u32));

	return MDL_RET_SUCCESS;
}

enum phl_mdl_ret_code
_phl_custom_vr_feature_query(void* custom_ctx,
                             struct _custom_vr_ctx* vr_ctx,
                             struct phl_msg* msg)
{
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	if (cmd->len < sizeof(u32))
		return ret;
	PHL_INFO("%s, vr query feature enable(%d)\n",
	         __FUNCTION__,
	         vr_ctx->init.enable);
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&(vr_ctx->init.enable),
	                           sizeof(u32));
	ret = MDL_RET_SUCCESS;
	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_testmode_param(void* custom_ctx,
                              struct _custom_vr_ctx* vr_ctx,
                              struct phl_msg* msg)
{
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;

	if (cmd->len < sizeof(u32))
		return ret;
	vr_ctx->init.test_mode = *(u32*)(cmd->data);
	PHL_INFO("%s, test mode(0x%x)\n", __FUNCTION__,
	         vr_ctx->init.test_mode);
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&ret,
	                           sizeof(u8));

	ret = MDL_RET_SUCCESS;
	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_ampdu_cfg(void* custom_ctx,
                         struct _custom_vr_ctx* vr_ctx,
                         struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	void *d = phl_to_drvpriv(phl);
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	struct rtw_phl_custom_ampdu_cfg custom_vr_ampdu_cfg = {0};
	u32 size = sizeof(struct rtw_phl_custom_ampdu_cfg);
	u8* val = cmd->data;
	struct rtw_wifi_role_t *wrole = NULL;
	struct rtw_wifi_role_link_t *rlink = NULL;
	u8 idx = 0;

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if (ret != MDL_RET_SUCCESS) {
		goto exit;
	}

	wrole = vr_ctx->init.wifi_role;
	_os_mem_cpy(d, &custom_vr_ampdu_cfg, val, size);

	PHL_INFO("%s, halsta(%d) ampdu dur(%d) num(%d)\n",
	         __FUNCTION__,
	         hal_status,
	         custom_vr_ampdu_cfg.max_agg_time_32us,
	         custom_vr_ampdu_cfg.max_agg_num);

	for (idx = 0; idx < wrole->rlink_num; idx++) {
		rlink = get_rlink(wrole, idx);

		hal_status = rtw_hal_custom_cfg_tx_ampdu(phl->hal,
		                                         rlink,
		                                         &custom_vr_ampdu_cfg);
		if (hal_status != RTW_HAL_STATUS_SUCCESS)
			ret = MDL_RET_FAIL;
	}

exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&ret,
	                           sizeof(u8));
	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_ampdu_query(void* custom_ctx,
                           struct _custom_vr_ctx* vr_ctx,
                           struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	struct rtw_phl_custom_ampdu_cfg custom_vr_ampdu_cfg = {0};
	u32 size = sizeof(struct rtw_phl_custom_ampdu_cfg);
	struct rtw_wifi_role_t *wrole = NULL;
	struct rtw_wifi_role_link_t *rlink = NULL;
	u8 idx = 0;

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if (ret != MDL_RET_SUCCESS) {
		goto exit;
	}

	wrole = vr_ctx->init.wifi_role;

	for (idx = 0; idx < wrole->rlink_num; idx++) {
		rlink = get_rlink(wrole, idx);

		hal_status = rtw_hal_get_ampdu_cfg(phl->hal,
		                                   rlink,
		                                   &custom_vr_ampdu_cfg);
		if (hal_status != RTW_HAL_STATUS_SUCCESS)
			ret = MDL_RET_FAIL;
	}

	PHL_INFO(" %s, ampdu dur(%d) time(%d)\n",
	         __FUNCTION__,
	         custom_vr_ampdu_cfg.max_agg_time_32us,
	         custom_vr_ampdu_cfg.max_agg_num);
exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&custom_vr_ampdu_cfg,
	                           sizeof(struct rtw_phl_custom_ampdu_cfg));
	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_pdthr_cfg(void* custom_ctx,
                         struct _custom_vr_ctx* vr_ctx,
                         struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	struct rtw_wifi_role_t *wrole = NULL;
	struct rtw_wifi_role_link_t *rlink = NULL;
	int pd_thr = 0xff;
	u32 size = sizeof(int);

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if (ret != MDL_RET_SUCCESS) {
		goto exit;
	}

	wrole = vr_ctx->init.wifi_role;
	rlink = get_rlink(wrole, RTW_RLINK_PRIMARY);

	pd_thr = *(int*)(cmd->data);
	PHL_INFO("%s, pd_thr(%d)\n", __FUNCTION__, pd_thr);

	hal_status = rtw_hal_set_pkt_detect_thold(phl->hal,
						rlink->hw_band,
						(u32)pd_thr);
	PHL_INFO("%s, hal_status(%d)\n", __FUNCTION__, hal_status);
	if (hal_status != RTW_HAL_STATUS_SUCCESS)
		ret = MDL_RET_FAIL;
exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&ret,
	                           sizeof(u8));
	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_pdthr_query(void* custom_ctx,
                           struct _custom_vr_ctx* vr_ctx,
                           struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	struct rtw_wifi_role_t *wrole = NULL;
	struct rtw_wifi_role_link_t *rlink = NULL;
	int pd_thr = 0xff;
	u32 size = sizeof(int);

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if (ret != MDL_RET_SUCCESS) {
		goto exit;
	}

	wrole = vr_ctx->init.wifi_role;
	rlink = get_rlink(wrole, RTW_RLINK_PRIMARY);

	/*confirm whether pd thr is enabling or not*/
	pd_thr = rtw_hal_query_pkt_detect_thold(phl->hal,
	                                        true,
	                                        rlink->hw_band);
	if (pd_thr == 0) {
		PHL_INFO("%s, disable! pd_thr(%d)\n", __FUNCTION__, pd_thr);
	} else {
		pd_thr = rtw_hal_query_pkt_detect_thold(phl->hal,
		                                        false,
		                                        rlink->hw_band);
		PHL_INFO("%s, pd_thr(%d)\n", __FUNCTION__, pd_thr);
	}
exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&pd_thr,
	                           sizeof(int));

	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_pop_cfg(void* custom_ctx,
                       struct _custom_vr_ctx* vr_ctx,
                       struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_wifi_role_t *wrole = NULL;
	struct rtw_wifi_role_link_t *rlink = NULL;
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	u32 size = sizeof(u32);
	u32 pop_enable = 0xff;

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if (ret != MDL_RET_SUCCESS) {
		goto exit;
	}

	wrole = vr_ctx->init.wifi_role;
	rlink = get_rlink(wrole, RTW_RLINK_PRIMARY);

	pop_enable = *(u32*)(cmd->data);
	PHL_INFO("%s, pop_enable(%d)\n", __FUNCTION__, pop_enable);
	if (pop_enable != 0xff) {
		hal_status = rtw_hal_set_pop_en(phl->hal,
		                                (bool)pop_enable,
		                                rlink->hw_band);
		if (hal_status != RTW_HAL_STATUS_SUCCESS)
			ret = MDL_RET_FAIL;
	}
exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&ret,
	                           sizeof(u8));

	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_pop_query(void* custom_ctx,
                         struct _custom_vr_ctx* vr_ctx,
                         struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_wifi_role_t *wrole = NULL;
	struct rtw_wifi_role_link_t *rlink = NULL;
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	u32 size = sizeof(u32);
	u32 pop_enable = 0xff;

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if (ret != MDL_RET_SUCCESS){
		goto exit;
	}

	wrole = vr_ctx->init.wifi_role;
	rlink = get_rlink(wrole, RTW_RLINK_PRIMARY);

	pop_enable = rtw_hal_query_pop_en(phl->hal, rlink->hw_band);
	PHL_INFO("%s, pop_en(%d)\n", __FUNCTION__, pop_enable);
exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&pop_enable,
	                            sizeof(u32));

	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_set_tx_rate_masking(void* custom_ctx,
                                   struct _custom_vr_ctx* vr_ctx,
                                   struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	void *d = phl_to_drvpriv(phl);
	struct rtw_phl_stainfo_t *sta = NULL;
	struct rtw_wifi_role_t *wrole = NULL;
	struct rtw_wifi_role_link_t *rlink = NULL;
	_os_spinlockfg sp_flags;
	u64 tx_mask = (u64)(*(u32*)(cmd->data));
	u64 tmp_mask = 0;
	u8 idx = 0;


	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, sizeof(u32));
	if( ret != MDL_RET_SUCCESS){
		goto exit;
	}
	wrole = vr_ctx->init.wifi_role;

	/* Just only change the 1ss/2ss MCS rate*/
	tx_mask = (tx_mask << 12);

	/* search the stations in the queue*/

	for (idx = 0; idx < wrole->rlink_num; idx++) {
		rlink = get_rlink(wrole, idx);

		_os_spinlock(d, &rlink->assoc_sta_queue.lock, _irq, &sp_flags);

		phl_list_for_loop(sta, struct rtw_phl_stainfo_t,
					&rlink->assoc_sta_queue.queue, list) {
			if (sta == list_first_entry(&rlink->assoc_sta_queue.queue,
						struct rtw_phl_stainfo_t, list))
				continue;
			if (sta) {
				tmp_mask = sta->hal_sta->ra_info.cur_ra_mask & 0xfff;
				tx_mask |= tmp_mask;
				sta->hal_sta->ra_info.cur_ra_mask = tx_mask;
				PHL_INFO("%s: tx mask(0x%016llx) macid(%d)\n",
				         __FUNCTION__, tx_mask, sta->macid);
			}
		}
		_os_spinunlock(d, &rlink->assoc_sta_queue.lock, _irq, &sp_flags);
	}
exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&ret,
	                           sizeof(u8));
	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_get_tx_rate_masking(void* custom_ctx,
                                   struct _custom_vr_ctx* vr_ctx,
                                   struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	void *d = phl_to_drvpriv(phl);
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	struct rtw_phl_stainfo_t *sta = NULL;
	u32 tx_mask = 0;
	struct rtw_wifi_role_t *wrole = NULL;
	struct rtw_wifi_role_link_t *rlink = NULL;
	_os_spinlockfg sp_flags;
	u8 idx = 0;

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, sizeof(u32));
	if( ret != MDL_RET_SUCCESS){
		goto exit;
	}

	wrole = vr_ctx->init.wifi_role;

	/** Search the stations in the queue
	 *  We just need to query the first sta because all the masking
	 *  setting are the same for all sta.
	 */

	for (idx = 0; idx < wrole->rlink_num; idx++) {
		rlink = get_rlink(wrole, idx);

		_os_spinlock(d, &rlink->assoc_sta_queue.lock, _irq, &sp_flags);
		phl_list_for_loop(sta, struct rtw_phl_stainfo_t,
					&rlink->assoc_sta_queue.queue, list) {
			if (sta == list_first_entry(&rlink->assoc_sta_queue.queue,
						struct rtw_phl_stainfo_t, list))
				continue;
			if (sta) {
				tx_mask = (u32)(sta->hal_sta->ra_info.cur_ra_mask >> 12);
				break;
			}
		}

		_os_spinunlock(d, &rlink->assoc_sta_queue.lock, _irq, &sp_flags);
	}

	PHL_INFO("%s: tx mask(%x)\n", __FUNCTION__, tx_mask);

exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&tx_mask,
	                           sizeof(u32));
	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_set_tx_rate_rty_tbl(void* custom_ctx,
                                   struct _custom_vr_ctx* vr_ctx,
                                   struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	void *d = phl_to_drvpriv(phl);
	u8* val = (u8*)(cmd->data);
	u8 i = 0;
	u32 size = sizeof(struct _vr_tx_rty_param);

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if( ret != MDL_RET_SUCCESS){
		goto exit;
	}

	_os_mem_cpy(d, &vr_ctx->tx_rty_param, val, size);

	PHL_INFO("%s retry setting %s\n",
	          __func__,
	          vr_ctx->tx_rty_param.enable ? "enable" : "disable");

	for(i = 0; i < 24 ; i++){
	PHL_INFO("%s retry setting index(%d) rty_count(%d)\n",
	          __func__, i, vr_ctx->tx_rty_param.rty_rate_tbl[i]);
	}

	rtw_hal_set_tx_rate_rty_tbl(phl->hal,
	                            (bool)vr_ctx->tx_rty_param.enable,
	                            vr_ctx->tx_rty_param.rty_rate_tbl);

exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&ret,
	                           sizeof(u8));
	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_get_tx_rate_rty_tbl(void* custom_ctx,
                                   struct _custom_vr_ctx* vr_ctx,
                                   struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	u8 i = 0;
	u32 size = sizeof(struct _vr_tx_rty_param);

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if( ret != MDL_RET_SUCCESS){
		goto exit;
	}

	PHL_INFO("%s retry setting %s\n",
	          __func__,
	          vr_ctx->tx_rty_param.enable ? "enable" : "disable");

	for(i = 0; i < 24 ; i++){
	PHL_INFO("%s retry setting index(%d) rty_count(%d)\n",
	          __func__, i, vr_ctx->tx_rty_param.rty_rate_tbl[i]);
	}
exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&vr_ctx->tx_rty_param,
	                           size);
	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_edca_query(void* custom_ctx,
                          struct _custom_vr_ctx* vr_ctx,
                          struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	struct rtw_edca_param edca_param = {0};
	struct rtw_wifi_role_t *wifi_role = NULL;
	struct rtw_wifi_role_link_t *rlink = NULL;
	u8 idx = 0;

	edca_param.ac = 0xFF;
	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, sizeof(u8));
	if (ret != MDL_RET_SUCCESS)
		goto exit;

	wifi_role = vr_ctx->init.wifi_role;
	edca_param.ac = *((u8*)(cmd->data));

	for (idx = 0; idx < wifi_role->rlink_num; idx++) {
		rlink = get_rlink(wifi_role, idx);

		hal_status = rtw_hal_get_edca(phl->hal,
		                              rlink,
		                              &edca_param);
		if (hal_status != RTW_HAL_STATUS_SUCCESS)
			ret = MDL_RET_FAIL;
	}

	if (hal_status != RTW_HAL_STATUS_SUCCESS)
		edca_param.ac = 0xFF;

	PHL_INFO("%s, custom_vr_edca_param rpt: ac(%d), param(0x%x)\n",
	         __FUNCTION__,
	         edca_param.ac,
	         edca_param.param);
exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&edca_param,
	                           sizeof(struct rtw_edca_param));

	return MDL_RET_SUCCESS;
}

#ifdef CONFIG_PHL_CUSTOM_FRAME_STAT
bool
_is_stats_rpt_cfg_needed(struct _vr_stats_rpt_param *cur,
                         struct _vr_stats_rpt_cfg *target)
{
	if (cur->cfg.mode != target->mode)
		return true;

	if (cur->cfg.macid != target->macid)
		return true;

	if (cur->cfg.mode == USR_TX_STATS_RPT_PERIOD &&
	    (cur->cfg.rpt_period_us != target->rpt_period_us))
		return true;

	return false;
}

bool
_update_usr_tx_rpt_cfg(struct phl_info_t *phl,
                       struct _custom_vr_ctx *vr_ctx,
		       struct rtw_phl_stainfo_t *sta,
                       enum _usr_tx_stats_rpt_mode mode,
                       u16 macid,
                       u32 rpt_period_us)
{
	struct rtw_phl_usr_tx_rpt_cfg usr_tx_rpt_cfg = {0};

	usr_tx_rpt_cfg.mode = mode;
	usr_tx_rpt_cfg.rpt_start = 0;
	usr_tx_rpt_cfg.macid = macid;
	usr_tx_rpt_cfg.band = sta->rlink->hw_band;
	usr_tx_rpt_cfg.port = sta->rlink->hw_port;
	usr_tx_rpt_cfg.rpt_period_us = rpt_period_us;

	if (rtw_hal_set_usr_tx_rpt_cfg(phl->hal, &usr_tx_rpt_cfg))
		return false;

	/* update cur stats if succeed */
	vr_ctx->stats_rpt.cfg.mode = mode;
	vr_ctx->stats_rpt.cfg.macid = macid;
	vr_ctx->stats_rpt.cfg.rpt_period_us = rpt_period_us;
	return true;
}

void _reset_tx_stats_ring(void *drvpriv, struct _tx_stats_ring *ring)
{
	_os_spinlock(drvpriv, &ring->lock, _bh, NULL);
	ring->r_idx = ring->w_idx = 0;
	_os_mem_set(drvpriv,
	            ring->entry,
	            0,
	            MAX_TX_STATS_RING_ENTRY_NUM * sizeof(struct _tx_stats_entry));
	_os_spinunlock(drvpriv, &ring->lock, _bh, NULL);
}

bool
_vr_stats_cfg(struct phl_info_t *phl,
              struct _custom_vr_ctx *vr_ctx,
              struct rtw_phl_stainfo_t *sta,
              struct _vr_stats_rpt_cfg *cfg)
{
	bool ret = false;
	void *drvpriv = phl_to_drvpriv(phl);
	struct _tx_stats_ring *ring = &vr_ctx->stats_rpt.t_stats_ring;

	if (cfg->mode >= USR_TX_STATS_RPT_MAX) {
		PHL_INFO(" %s, invalid mode(%d)\n", __FUNCTION__, cfg->mode);
		goto exit;
	}

	/* 1. Set to disable first or target is USR_TX_STATS_RPT_DIS*/
	if (!_update_usr_tx_rpt_cfg(phl, vr_ctx, sta, USR_TX_STATS_RPT_DIS,
	                            (u16)cfg->macid, 0)) {
		PHL_INFO(" %s disable first fail\n", __FUNCTION__);
		goto exit;
	}
	/*if target is USR_TX_STATS_RPT_DIS, just reset tx_stats */
	if(cfg->mode == USR_TX_STATS_RPT_DIS) {
		_reset_tx_stats_ring(drvpriv, ring);
		ret = true;
		goto exit;
	}
	/* 2. set_ch_busy_sta_cfg for channel busy information */
	if (rtw_hal_set_ch_busy_stat_cfg(phl->hal, sta->rlink->hw_band)) {
		PHL_INFO(" %s setup busy info fail\n", __FUNCTION__);
		goto exit;
	}
	/* 3. Config stats mode */
	if (_update_usr_tx_rpt_cfg(phl, vr_ctx, sta, cfg->mode, (u16)cfg->macid,
	                           cfg->rpt_period_us)) {
		_reset_tx_stats_ring(drvpriv, ring);
		ret = true;
	}
exit:
	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_stats_rpt_cfg(void *custom_ctx,
                             struct _custom_vr_ctx *vr_ctx,
                             struct phl_msg *msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	struct rtw_phl_stainfo_t *sta = NULL;
	struct _vr_stats_rpt_param *stats_rpt = &(vr_ctx->stats_rpt);
	struct _vr_stats_rpt_cfg *cfg = NULL;
	u32 size = sizeof(struct _vr_stats_rpt_cfg);

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if (ret != MDL_RET_SUCCESS)
		goto exit;

	ret = MDL_RET_FAIL;
	cfg = (struct _vr_stats_rpt_cfg *)(cmd->data);

	/* Check whether the macid is existed */
	sta = rtw_phl_get_stainfo_by_macid(phl, (u16)cfg->macid);
	if (sta == NULL) {
		PHL_INFO(" %s, invalid macid(%d).\n", __FUNCTION__, cfg->macid);
		goto exit;
	}

	/* Check whether the configuration is changed. */
	if (!_is_stats_rpt_cfg_needed(stats_rpt, cfg)) {
		PHL_INFO(" %s, no need to config, mode(%d), macid(%d), period(%d).\n",
		         __FUNCTION__,
		         cfg->mode,
		         cfg->macid,
		         cfg->rpt_period_us);
		ret = MDL_RET_SUCCESS;
		goto exit;
	}
	if(_vr_stats_cfg(phl, vr_ctx, sta, cfg))
		ret = MDL_RET_SUCCESS;
exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8 *)&ret,
	                           sizeof(u8));

	return ret;
}
#endif /*CONFIG_PHL_CUSTOM_FRAME_STAT*/

enum phl_mdl_ret_code
_phl_custom_vr_sr_cfg(void* custom_ctx,
                       struct _custom_vr_ctx* vr_ctx,
                       struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	u32 size = sizeof(u8);
	u8 sr_enable = false;

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if (ret != MDL_RET_SUCCESS) {
		goto exit;
	}

	sr_enable = *(u8*)(cmd->data);
	PHL_INFO("%s, sr_enable(%d)\n", __FUNCTION__, sr_enable);

	hal_status = rtw_hal_set_spatial_reuse_en(phl->hal, (bool)sr_enable);
	if (hal_status != RTW_HAL_STATUS_SUCCESS)
		ret = MDL_RET_FAIL;

exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&ret,
	                           sizeof(u8));

	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_sr_query(void* custom_ctx,
                         struct _custom_vr_ctx* vr_ctx,
                         struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	u32 size = sizeof(u8);
	u8 sr_enable = 0xff;

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if (ret != MDL_RET_SUCCESS){
		goto exit;
	}

	sr_enable = rtw_hal_is_spatial_reuse_en(phl->hal);
	PHL_INFO("%s, sr_en(%d)\n", __FUNCTION__, sr_enable);
exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           &sr_enable,
	                            sizeof(u8));

	return ret;
}

enum phl_mdl_ret_code _phl_custom_vr_rf_scramble(void *custom_ctx,
						 struct _custom_vr_ctx *vr_ctx,
						 struct phl_msg *msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	void *d = phl_to_drvpriv(phl);
	struct rtw_custom_decrpt *cmd =
	    (struct rtw_custom_decrpt *)(msg->inbuf);
	struct _vr_rf_scrmb *val = (struct _vr_rf_scrmb *)(cmd->data);
	u32 size = sizeof(struct _vr_rf_scrmb);

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if (ret != MDL_RET_SUCCESS) {
		val->status = RTW_HAL_STATUS_FAILURE;
		goto exit;
	}

	PHL_INFO("%s: scramble cmd(%s)\n", __func__,
		 (val->is_set) ? "Set_scrmb" : "Get_scrmb");
	if (val->is_set) {
		hal_status = rtw_hal_set_usr_frame_to_act(
		    phl->hal, val->mode, val->to_thr, val->trigger_cnt,
		    val->sw_def_bmp);
		val->status = (u8)hal_status;
		if (hal_status == RTW_HAL_STATUS_SUCCESS)
			_os_mem_cpy(d, &vr_ctx->cur_scrmb_param, val, size);
	} else {
		_os_mem_cpy(d, val, &vr_ctx->cur_scrmb_param, size);
	}

exit:
	phl_custom_prepare_evt_rpt(custom_ctx, cmd->evt_id, cmd->customer_id,
				   (u8 *)val, sizeof(struct _vr_rf_scrmb));

	return ret;
}

struct custom_vr_perf_metrics_rlink {
	u8 mac_addr[MAC_ALEN];
	u8 rssi;
	char tx_rate_str[32];
	char rx_rate_str[32];
	u8 clm_ratio;
	u8 nhm_ratio;
	u8 snr_avg;
};

struct custom_vr_perf_metrics {
	u8 rlink_num;
	struct custom_vr_perf_metrics_rlink perf_metrics_rlink[RTW_RLINK_MAX];
};

enum phl_mdl_ret_code
_phl_custom_vr_perf_mertics_query(void* custom_ctx,
                                 struct _custom_vr_ctx* vr_ctx,
                                 struct phl_msg* msg) {
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	void *drv = phl_to_drvpriv(phl);
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	struct custom_vr_perf_metrics perf_metrics = {0};
	struct custom_vr_perf_metrics_rlink *curr = NULL;
	struct rtw_wifi_role_t *wrole = NULL;
	struct rtw_wifi_role_link_t *rlink = NULL;
	struct rtw_phl_stainfo_t *n, *psta;
	struct rtw_hal_com_t *hal_com = rtw_hal_get_halcom(phl->hal);
	struct rtw_env_report env_rpt = {0};
	u8 idx = 0;

	PHL_INFO("Enter %s\n", __func__);

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, sizeof(u8));
	if (ret != MDL_RET_SUCCESS)
		goto exit;

	wrole = vr_ctx->init.wifi_role;
	perf_metrics.rlink_num = wrole->rlink_num;
	PHL_INFO("%s(): rlink_num = %d\n", __func__, perf_metrics.rlink_num);
	for (idx = 0; idx < wrole->rlink_num; idx++) {
		rlink = get_rlink(wrole, idx);
		curr = &perf_metrics.perf_metrics_rlink[idx];
		_os_spinlock(drv, &rlink->assoc_sta_queue.lock, _bh, NULL);
		phl_list_for_loop_safe(psta, n, struct rtw_phl_stainfo_t,
		       &rlink->assoc_sta_queue.queue, list) {
			_os_mem_cpy(drv, curr->mac_addr, psta->mac_addr, MAC_ALEN);
			PHL_INFO("mac addr %02x-%02x-%02x-%02x-%02x-%02x\n",
			         curr->mac_addr[0], curr->mac_addr[1],
			         curr->mac_addr[2], curr->mac_addr[3],
			         curr->mac_addr[4], curr->mac_addr[5]);

			PHL_INFO("WROLE-IDX:%d RLINK-IDX:%d wlan_mode:0x%02x, chan:%d, bw:%d, rlink_state:%s\n",
			         psta->wrole->id,
			         psta->rlink->id,
			         psta->wmode,
			         psta->chandef.chan,
			         psta->chandef.bw,
			         rlink->mstate?((rlink->mstate & MLME_LINKING)?"Linking":"Linked Up"):"No Link");

			curr->rssi = psta->hal_sta->rssi_stat.ma_rssi;
			PHL_INFO("[Stats] MA RSSI:%d(dBm)\n",
			         curr->rssi - PHL_MAX_RSSI);

			convert_tx_rate(psta->hal_sta->ra_info.rpt_rt_i.mode,
			                 psta->hal_sta->ra_info.rpt_rt_i.mcs_ss_idx,
			                 curr->tx_rate_str, 32);
			PHL_INFO("[Stats] Tx Rate:%s\n", curr->tx_rate_str);

			convert_rx_rate(psta->stats.rx_rate, curr->rx_rate_str,
			                 32);
			PHL_INFO("[Stats] Rx Rate:%s\n", curr->rx_rate_str);
		}
		_os_spinunlock(drv, &rlink->assoc_sta_queue.lock, _bh, NULL);

		rtw_hal_env_rpt(hal_com, &env_rpt, rlink->hw_band);
		curr->clm_ratio = env_rpt.clm_ratio;
		curr->nhm_ratio = env_rpt.nhm_ratio;
		PHL_INFO("[Stats] clm_ratio = %d\n", curr->clm_ratio);
		PHL_INFO("[Stats] nhm_ratio = %d\n", curr->nhm_ratio);

		rtw_hal_query_snr_avg(hal_com, &curr->snr_avg, rlink->hw_band);
		PHL_INFO("[Stats] snr_avg = %d\n", curr->snr_avg);
	}

exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&perf_metrics,
	                           sizeof(struct custom_vr_perf_metrics));

	PHL_INFO("Leave %s\n", __func__);
	return MDL_RET_SUCCESS;
}

enum phl_mdl_ret_code _phl_custom_vr_set_ant_switch(
    void *custom_ctx, struct _custom_vr_ctx *vr_ctx, struct phl_msg *msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	struct rtw_custom_decrpt *cmd =
	    (struct rtw_custom_decrpt *)(msg->inbuf);
	u32 path = *(u32 *)(cmd->data);
	u32 size = sizeof(u32);

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if (ret != MDL_RET_SUCCESS) {
		goto exit;
	}

	PHL_INFO("%s: set rf path(%d)\n", __func__, path);

	switch (path) {
	case RF_PATH_AC:
		if ((rtw_hal_sw_gpio_ctrl(phl->phl_com, phl->hal, 0, 12) !=
		     RTW_HAL_STATUS_SUCCESS) ||
		    (rtw_hal_sw_gpio_ctrl(phl->phl_com, phl->hal, 0, 14) !=
		     RTW_HAL_STATUS_SUCCESS))
			ret = MDL_RET_FAIL;
		PHL_INFO("%s: set path to AC\n", __func__);
		break;
	case RF_PATH_AD:
		if ((rtw_hal_sw_gpio_ctrl(phl->phl_com, phl->hal, 0, 12) !=
		     RTW_HAL_STATUS_SUCCESS) ||
		    (rtw_hal_sw_gpio_ctrl(phl->phl_com, phl->hal, 1, 14) !=
		     RTW_HAL_STATUS_SUCCESS))
			ret = MDL_RET_FAIL;
		PHL_INFO("%s: set path to AD\n", __func__);
		break;
	case RF_PATH_BC:
		if ((rtw_hal_sw_gpio_ctrl(phl->phl_com, phl->hal, 1, 12) !=
		     RTW_HAL_STATUS_SUCCESS) ||
		    (rtw_hal_sw_gpio_ctrl(phl->phl_com, phl->hal, 0, 14) !=
		     RTW_HAL_STATUS_SUCCESS))
			ret = MDL_RET_FAIL;
		PHL_INFO("%s: set path to BC\n", __func__);
		break;
	case RF_PATH_BD:
		if ((rtw_hal_sw_gpio_ctrl(phl->phl_com, phl->hal, 1, 12) !=
		     RTW_HAL_STATUS_SUCCESS) ||
		    (rtw_hal_sw_gpio_ctrl(phl->phl_com, phl->hal, 1, 14) !=
		     RTW_HAL_STATUS_SUCCESS))
			ret = MDL_RET_FAIL;
		PHL_INFO("%s: set path to BD\n", __func__);
		break;
	default:
		PHL_INFO("%s: unexpected rf path!\n", __func__);
		break;
	}

exit:
	phl_custom_prepare_evt_rpt(custom_ctx, cmd->evt_id, cmd->customer_id,
				   (u8 *)&ret, sizeof(u32));

	return ret;
}

#ifdef CONFIG_PHL_CHANNEL_INFO_VR
enum phl_mdl_ret_code
_phl_custom_vr_csi_cfg(void* custom_ctx,
                       struct _custom_vr_ctx* vr_ctx,
                       struct phl_msg* msg) {
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	u32 size = sizeof(u8);
	u8 csi_enable = false;

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if (ret != MDL_RET_SUCCESS) {
		goto exit;
	}

	csi_enable = *(u8*)(cmd->data);
	PHL_INFO("%s, csi_enable(%d)\n", __FUNCTION__, csi_enable);

	if (csi_enable) {
		struct rtw_wifi_role_t *wrole = NULL;
		struct rtw_wifi_role_link_t *rlink = NULL;

		wrole = vr_ctx->init.wifi_role;
		rlink = get_rlink(wrole, RTW_RLINK_PRIMARY);
		ret = rtw_phl_custom_csi_start(custom_ctx,
		                               &vr_ctx->csi_ctrl,
		                               rlink);
	} else {
		ret = rtw_phl_custom_csi_stop(custom_ctx, &vr_ctx->csi_ctrl);
	}

exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&ret,
	                           sizeof(u8));

	return ret;
}

/*
 * custom_vr_csi - channel state information,
 * @len: length of channel info buffer
 * @buf: channel info buffer
 *       buf size is limited by minumim of MAX_DATA_SIZE (VR command data size)
 *       and CHAN_INFO_MAX_SIZE.
 */
struct custom_vr_csi {
	u32 len;
	u8 buf[MAX_DATA_SIZE - sizeof(u32)];
	/* u8 buf[CHAN_INFO_MAX_SIZE]; */
};

enum phl_mdl_ret_code
_phl_custom_vr_csi_query(void* custom_ctx,
                         struct _custom_vr_ctx* vr_ctx,
                         struct phl_msg* msg) {
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	struct rtw_custom_decrpt *cmd = (struct rtw_custom_decrpt *)(msg->inbuf);
	struct custom_vr_csi vr_csi = {0};
	u32 size = sizeof(struct custom_vr_csi);
	u32 i = 0, print_len = 0;
	u64 *buff_tmp = NULL;

	PHL_INFO("%s\n", __func__);

	ret = _is_vr_mode_valid(custom_ctx, vr_ctx, msg, size);
	if (ret != MDL_RET_SUCCESS)
		goto exit;

	ret = rtw_phl_custom_csi_rslt_query(custom_ctx,
	                                    (u8 *)vr_csi.buf,
	                                    &vr_csi.len);

	print_len = vr_csi.len >> 3;
	if (vr_csi.len % 8)
		print_len++;
	buff_tmp = (u64 *)vr_csi.buf;
	PHL_TRACE(COMP_PHL_CHINFO, _PHL_INFO_, "%s, CSI raw data: len = %d\n",
	          __func__, vr_csi.len);
	for (i = 0; i < print_len; i++)
		PHL_TRACE(COMP_PHL_CHINFO, _PHL_INFO_, "[%02d]0x%016llx\n",
		          i, buff_tmp[i]);

exit:
	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&vr_csi,
	                           sizeof(struct custom_vr_csi));

	return MDL_RET_SUCCESS;
}
#endif /* CONFIG_PHL_CHANNEL_INFO_VR */

enum phl_mdl_ret_code _phl_custom_vr_set_bss_color(
    void *custom_ctx, struct _custom_vr_ctx *vr_ctx, struct phl_msg *msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_SUCCESS;
	struct rtw_custom_decrpt *cmd =
	    (struct rtw_custom_decrpt *)(msg->inbuf);
	u8 bss_color = 0;

	bss_color = *(u8 *)(cmd->data);

	/* check if bss color is valid or not */
	if (bss_color >= 1 && bss_color <= 63) {
		vr_ctx->bsscolor_param.bsscolor = bss_color;
		vr_ctx->bsscolor_param.is_valid = true;
	}

	PHL_INFO("%s: bss color(0x%x) is_valid(%d)\n", __func__,
		 vr_ctx->bsscolor_param.bsscolor, vr_ctx->bsscolor_param.is_valid);

	phl_custom_prepare_evt_rpt(custom_ctx, cmd->evt_id, cmd->customer_id,
				   (u8 *)&ret, sizeof(u8));

	return ret;
}

enum phl_mdl_ret_code
phl_custom_hdl_vr_evt(void* dispr,
                      void* custom_ctx,
                      struct _custom_vr_ctx* vr_ctx,
                      struct phl_msg* msg)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;
	u8 prephase = (IS_MSG_IN_PRE_PHASE(msg->msg_id)) ? (true) : (false);

	if (prephase == true)
		return MDL_RET_SUCCESS;

	switch (MSG_EVT_ID_FIELD(msg->msg_id)) {
		case MSG_EVT_CUSTOME_FEATURE_ENABLE:
			ret = _phl_custom_vr_feature_enable(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_CUSTOME_FEATURE_QUERY:
			ret = _phl_custom_vr_feature_query(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_CUSTOME_TESTMODE_PARAM:
			ret = _phl_custom_vr_testmode_param(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_EDCA_QUERY:
			ret = _phl_custom_vr_edca_query(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_AMPDU_CFG:
			ret = _phl_custom_vr_ampdu_cfg(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_AMPDU_QUERY:
			ret = _phl_custom_vr_ampdu_query(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_PDTHR_CFG:
			ret = _phl_custom_vr_pdthr_cfg(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_PDTHR_QUERY:
			ret = _phl_custom_vr_pdthr_query(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_POP_CFG:
			ret = _phl_custom_vr_pop_cfg(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_POP_QUERY:
			ret = _phl_custom_vr_pop_query(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_SET_TX_RATE_MASKING:
			ret = _phl_custom_vr_set_tx_rate_masking(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_GET_TX_RATE_MASKING:
			ret = _phl_custom_vr_get_tx_rate_masking(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_SET_TX_RATE_RTY_TBL:
			ret = _phl_custom_vr_set_tx_rate_rty_tbl(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_GET_TX_RATE_RTY_TBL:
			ret = _phl_custom_vr_get_tx_rate_rty_tbl(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_GET_PERF_METRICS:
			ret = _phl_custom_vr_perf_mertics_query(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_SET_SPATIAL_REUSE:
			ret = _phl_custom_vr_sr_cfg(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_GET_SPATIAL_REUSE:
			ret = _phl_custom_vr_sr_query(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_RF_SCRAMBLE:
			ret = _phl_custom_vr_rf_scramble(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_SET_ANT_SWITCH:
			ret = _phl_custom_vr_set_ant_switch(custom_ctx, vr_ctx,
							    msg);
			break;
#ifdef CONFIG_PHL_CHANNEL_INFO_VR
		case MSG_EVT_SET_CSI:
			ret = _phl_custom_vr_csi_cfg(custom_ctx, vr_ctx, msg);
			break;
		case MSG_EVT_GET_CSI:
			ret = _phl_custom_vr_csi_query(custom_ctx, vr_ctx, msg);
			break;
#endif /* CONFIG_PHL_CHANNEL_INFO_VR */
		case MSG_EVT_SET_BSS_COLOR:
			ret = _phl_custom_vr_set_bss_color(custom_ctx, vr_ctx,
							   msg);
			break;
#ifdef CONFIG_PHL_CUSTOM_FRAME_STAT
		case MSG_EVT_STATS_RPT_CFG:
			ret = _phl_custom_vr_stats_rpt_cfg(custom_ctx, vr_ctx, msg);
			break;
#endif /*CONFIG_PHL_CUSTOM_FRAME_STAT*/
		default:
			ret = MDL_RET_SUCCESS;
			break;
	}
	PHL_INFO("%s, evt(%d), ret(%d)\n", __FUNCTION__,
	         MSG_EVT_ID_FIELD(msg->msg_id),
	         ret);
	return ret;
}
enum phl_mdl_ret_code
phl_custom_hdl_vr_fail_evt(void* dispr,
                           void* custom_ctx,
                           struct _custom_vr_ctx* vr_ctx,
                           struct phl_msg* msg)
{
	return MDL_RET_IGNORE;
}

enum phl_mdl_ret_code
_phl_custom_vr_set_wifi_role(void* dispr,
                             void* custom_ctx,
                             struct _custom_vr_ctx* vr_ctx,
                             struct rtw_custom_decrpt *cmd)
{
	enum phl_mdl_ret_code status = MDL_RET_SUCCESS;
	u32 size = sizeof(struct rtw_wifi_role_t *);

	PHL_INFO("%s, start\n", __FUNCTION__);

	if (cmd->len < size) {
		PHL_INFO("%s: illegal info len\n", __FUNCTION__);
		status = MDL_RET_FAIL;
		return status;
	}

	vr_ctx->init.wifi_role = *(struct rtw_wifi_role_t **)(cmd->data);

	phl_custom_prepare_evt_rpt(custom_ctx,
	                           cmd->evt_id,
	                           cmd->customer_id,
	                           (u8*)&status,
	                           sizeof(u8));

	PHL_INFO("%s, status(%d).\n", __FUNCTION__, status);

	return status;
}

#ifdef CONFIG_PHL_CUSTOM_FRAME_STAT
void _dump_frame_stats_rpt(struct rtw_phl_vr_frame_stats_rpt *cus_rpt)
{
	u8 q = 0;

	PHL_INFO("%s, rpt_mode(%d), tsf(%d).\n", __FUNCTION__,
		 cus_rpt->rpt_mode, cus_rpt->tsf);

	for (q = PHL_BE_QUEUE_SEL; q < PHL_AC_QUEUE_TOTAL; q++) {
		PHL_INFO(
		    " %s, q(%d): (t_ok, t_rty_fail, t_fail) = (%d, %d, %d)\n",
		    __FUNCTION__, q, cus_rpt->tx_ok_cnt[q],
		    cus_rpt->tx_rty_fail_cnt[0], cus_rpt->tx_fail_cnt[q]);
	}
}

bool
_prepare_frame_stats_rpt(struct phl_info_t *phl,
                         struct _vr_stats_rpt_param *stats_rpt,
                         struct rtw_mac_usr_tx_rpt_info *c2h_rpt)
{
	bool is_rpt_ready = false;
	u8 mode = (u8)c2h_rpt->rpt_mode;
	u16 macid = (u16)c2h_rpt->macid;
	void *drvpriv = phl_to_drvpriv(phl);
	struct rtw_phl_stainfo_t *sta = rtw_phl_get_stainfo_by_macid(phl, macid);
	struct _tx_stats_ring *ring = &stats_rpt->t_stats_ring;
	struct rtw_phl_vr_frame_stats_rpt *cus_rpt = NULL;
	u8 i;
	u32 latency_first_pkt, latency_last_pkt, latency_first_2_last;

	if (mode != stats_rpt->cfg.mode || macid != stats_rpt->cfg.macid || sta == NULL) {
		PHL_INFO("%s, skip, c2h_mode(%d)!=cfg_mode(%d) or c2h_macid(%d)!=cfg_macid(%d).\n",
	                 __FUNCTION__,
			 mode,
			 stats_rpt->cfg.mode,
	                 macid,
		         stats_rpt->cfg.macid);
		return false;
	}

	latency_first_pkt =  phl_get_diff_time_us(c2h_rpt->freerun_cnt_first_in,
	                                   c2h_rpt->freerun_cnt_first_out);
	latency_last_pkt = phl_get_diff_time_us(c2h_rpt->freerun_cnt_last_in,
	                                 c2h_rpt->freerun_cnt_last_out);
	latency_first_2_last = phl_get_diff_time_us(c2h_rpt->freerun_cnt_first_in,
	                                     c2h_rpt->freerun_cnt_last_out);

	_os_spinlock(drvpriv, &ring->lock, _bh, NULL);

	if (stats_rpt->cfg.mode == USR_TX_STATS_RPT_PERIOD)
		ring->w_idx = (ring->w_idx + 1) % MAX_TX_STATS_RING_ENTRY_NUM;

	/* select the first empty cus rpt */
	i = ring->r_idx;
	while (i != ring->w_idx) {
		if (ring->entry[i].cus_rpt.rpt_mode == USR_TX_STATS_RPT_DIS)
			break;
		else if (ring->entry[i].total_pkt_cnt == ring->entry[i].recv_rpt_cnt &&
		         !ring->entry[i].rpt_sent)
			is_rpt_ready = true;
		i = (i + 1) % MAX_TX_STATS_RING_ENTRY_NUM;
	}

	/* fill the cus rpt if index is valid */
	if (i == ring->w_idx) {
		PHL_WARN(" %s, skip this c2h rpt, ridx(%d), widx(%d)\n",
		         __FUNCTION__,
			 ring->r_idx,
		         ring->w_idx);
	} else {
		PHL_INFO(" %s, recv c2h for ring entry idx(%d)\n", __FUNCTION__, i);

		cus_rpt = &ring->entry[i].cus_rpt;
		cus_rpt->rpt_mode = stats_rpt->cfg.mode;
		cus_rpt->latency_first_pkt = latency_first_pkt;
		cus_rpt->latency_last_pkt = latency_last_pkt;
		cus_rpt->latency_first_2_last = latency_first_2_last;
		cus_rpt->tsf = c2h_rpt->freerun_cnt_last_out ;
		cus_rpt->rssi_dbm = PHL_MAX_RSSI - sta->hal_sta->rssi_stat.ma_rssi;
		cus_rpt->snr_dbm = sta->hal_sta->rssi_stat.snr_ma >> 4;

		cus_rpt->tx_rate_mode = sta->hal_sta->ra_info.rpt_rt_i.mode;
		cus_rpt->tx_mcs = sta->hal_sta->ra_info.rpt_rt_i.mcs_idx;
		cus_rpt->tx_nss =
		    sta->hal_sta->ra_info.rpt_rt_i.ss + 1;
		cus_rpt->tx_per = c2h_rpt->ra_ratio;
		cus_rpt->tx_ampdu_cnt_wo_last_pkt = c2h_rpt->tx_ppdu_cnt_wo_last_pkt;
		/* here we only cal avg mpdu cnt (per ampdu), the len of ampdu will be cal in core custom */
		cus_rpt->tx_ampdu_len_wo_last_pkt =
		        (c2h_rpt->tx_ppdu_cnt_wo_last_pkt)?
		        (c2h_rpt->tx_mpdu_cnt_wo_last_pkt / c2h_rpt->tx_ppdu_cnt_wo_last_pkt):
		        0;
		cus_rpt->tx_rts_cnt = c2h_rpt->tx_rts_cnt;
		cus_rpt->tx_rts_retry_cnt = c2h_rpt->tx_rts_retry_cnt;

		cus_rpt->rx_err_cnt = c2h_rpt->rx_err_cnt;
		cus_rpt->rx_clear_cycle = c2h_rpt->rx_clear_us;
		cus_rpt->total_cycle = c2h_rpt->rx_clear_us + c2h_rpt->busy_us;

		/* debug msg */
		_dump_frame_stats_rpt(cus_rpt);

		if (ring->entry[i].total_pkt_cnt <= ring->entry[i].recv_rpt_cnt) {
			is_rpt_ready = true;
		} else {
			PHL_WARN(" %s, total_pkt_cnt(%d) != recv_rpt_cnt(%d)\n",
			         __FUNCTION__,
			         ring->entry[i].total_pkt_cnt,
			         ring->entry[i].recv_rpt_cnt);
		}
	}
	_os_spinunlock(drvpriv, &ring->lock, _bh, NULL);

	return is_rpt_ready;
}

void
_usr_tx_rpt_notify_complete(void *priv, struct phl_msg *msg)
{
	struct _custom_vr_ctx *vr_ctx = ( struct _custom_vr_ctx *)priv;
	struct _tx_stats_ring *ring = &vr_ctx->stats_rpt.t_stats_ring;
	void *drvpriv = phl_to_drvpriv(vr_ctx->init.phl);
	u8 i;

	_os_spinlock(drvpriv, &ring->lock, _bh, NULL);
	i = ring->r_idx;
	PHL_INFO(" %s, idx(%d)\n", __FUNCTION__, i);
	_os_mem_set(drvpriv, ring->entry + i, 0, sizeof(struct _tx_stats_entry));
	ring->r_idx = (i + 1) % MAX_TX_STATS_RING_ENTRY_NUM;
	_os_spinunlock(drvpriv, &ring->lock, _bh, NULL);
}

enum phl_mdl_ret_code _phl_custom_vr_proc_usr_tx_rpt(
    void *dispr, void *custom_ctx, struct _custom_vr_ctx *vr_ctx,
    struct rtw_custom_decrpt *cmd)
{
	enum phl_mdl_ret_code status = MDL_RET_SUCCESS;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	void *drvpriv = phl_to_drvpriv(phl);
	struct _tx_stats_ring *ring = &vr_ctx->stats_rpt.t_stats_ring;
	struct rtw_mac_usr_tx_rpt_info *c;
	struct phl_msg msg = {0};
	struct phl_msg_attribute attr = {0};
	u8 i;

	if (!vr_ctx->init.enable ||
	    vr_ctx->stats_rpt.cfg.mode == USR_TX_STATS_RPT_DIS ||
	    cmd->len != sizeof(struct rtw_mac_usr_tx_rpt_info)) {
		PHL_INFO("%s, skip, vr_mode(%d) or cur_cfg_mode(%d) or "
			 "inlen(%d)!=%d.\n",
			 __FUNCTION__, vr_ctx->init.enable,
			 vr_ctx->stats_rpt.cfg.mode, cmd->len,
			 sizeof(struct rtw_mac_usr_tx_rpt_info));
		status = MDL_RET_IGNORE;
		goto exit;
	}

	/* prepare report */
	c = (struct rtw_mac_usr_tx_rpt_info *)cmd->data;
	if (!_prepare_frame_stats_rpt(phl, &vr_ctx->stats_rpt, c)) {
		status = MDL_RET_IGNORE;
		goto exit;
	}

	/* send report to core layer */
	_os_spinlock(drvpriv, &ring->lock, _bh, NULL);
	i = ring->r_idx;
	while (i != ring->w_idx) {
		if (!ring->entry[i].rpt_sent &&
		    ring->entry[i].cus_rpt.rpt_mode &&
		    ring->entry[i].total_pkt_cnt <= ring->entry[i].recv_rpt_cnt) {
			PHL_INFO(" %s, send report, idx(%d)\n", __FUNCTION__, i);

			_os_mem_set(drvpriv, &msg, 0, sizeof(struct phl_msg));
			_os_mem_set(drvpriv, &attr, 0, sizeof(struct phl_msg_attribute));

			SET_MSG_MDL_ID_FIELD(msg.msg_id, PHL_MDL_CUSTOM);
			SET_MSG_EVT_ID_FIELD(msg.msg_id, MSG_EVT_STATS_RPT_NOTIFY);
			msg.inbuf = (u8*)(&ring->entry[i].cus_rpt);
			msg.inlen = sizeof(struct rtw_phl_vr_frame_stats_rpt);
			msg.rsvd[0].value = vr_ctx->stats_rpt.cfg.macid;
			attr.completion.completion = _usr_tx_rpt_notify_complete;
			attr.completion.priv = (void*)vr_ctx;

			if (phl_msg_hub_send(phl, &attr, &msg) != RTW_PHL_STATUS_SUCCESS) {
				PHL_ERR("%s send msg fail\n", __func__);
				break;
			}
			ring->entry[i].rpt_sent = true;
		}
		i = (i + 1) % MAX_TX_STATS_RING_ENTRY_NUM;
	}
	_os_spinunlock(drvpriv, &ring->lock, _bh, NULL);
exit:
	return status;
}
#endif /*CONFIG_PHL_CUSTOM_FRAME_STAT*/

enum phl_mdl_ret_code
phl_custom_vr_feature_set_hdlr(void* dispr,
                               void* custom_ctx,
                               struct _custom_vr_ctx* vr_ctx,
                               struct rtw_custom_decrpt *cmd)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;

	switch (cmd->evt_id) {
		case MSG_EVT_CUSTOME_SET_WIFI_ROLE:
			ret = _phl_custom_vr_set_wifi_role(dispr, custom_ctx, vr_ctx, cmd);
			break;
#ifdef CONFIG_PHL_CUSTOM_FRAME_STAT
		case MSG_EVT_PROC_USR_TX_RPT:
			ret = _phl_custom_vr_proc_usr_tx_rpt(dispr, custom_ctx, vr_ctx, cmd);
			break;
#endif /*CONFIG_PHL_CUSTOM_FRAME_STAT*/
		default:
			ret = MDL_RET_SUCCESS;
			break;
	}
	PHL_INFO("%s, evt(%d), ret(%d)\n", __FUNCTION__, cmd->evt_id, ret);
	return ret;
}

enum phl_mdl_ret_code
_phl_custom_vr_get_bss_color(void* dispr,
                         void* custom_ctx,
                         struct _custom_vr_ctx* vr_ctx,
                         struct rtw_custom_decrpt *cmd)
{
	enum phl_mdl_ret_code status = MDL_RET_SUCCESS;
	struct phl_info_t *phl = phl_custom_get_phl_info(custom_ctx);
	void *d = phl_to_drvpriv(phl);

	if (cmd->len == 0)
		return MDL_RET_FAIL;
	_os_mem_cpy(d, cmd->data, &vr_ctx->bsscolor_param, cmd->len);

	return status;
}

enum phl_mdl_ret_code
phl_custom_vr_feature_query_hdlr(void* dispr,
                                 void* custom_ctx,
                                 struct _custom_vr_ctx* vr_ctx,
                                 struct rtw_custom_decrpt *cmd)
{
	enum phl_mdl_ret_code ret = MDL_RET_FAIL;

	switch (cmd->evt_id) {
	case MSG_EVT_GET_BSS_COLOR:
		ret = _phl_custom_vr_get_bss_color(dispr, custom_ctx, vr_ctx,
						      cmd);
		break;
	default:
		ret = MDL_RET_SUCCESS;
		break;
	}

	PHL_INFO("%s, evt(%d), ret(%d)\n", __FUNCTION__, cmd->evt_id, ret);
	return ret;
}

enum rtw_phl_status phl_custom_vr_init_protocol_cap(
    struct phl_info_t *phl_info, u8 hw_band, enum role_type rtype,
    struct protocol_cap_t *protocol_cap)
{
	enum rtw_phl_status status = RTW_PHL_STATUS_SUCCESS;
	void *d = phl_to_drvpriv(phl_info);
	struct phl_module_op_info op_info = {0};
	struct _vr_bss_color color_info = {0};
	struct rtw_custom_decrpt cmd = {0};

	/* Bss color customization */
	rtw_phl_custom_prepare_query_info_evt(MSG_EVT_GET_BSS_COLOR,
					      CUS_ID_VR, &cmd, &op_info,
					      sizeof(struct _vr_bss_color));

	if (phl_disp_eng_query_bk_module_info(phl_info, hw_band, PHL_MDL_CUSTOM,
					      &op_info) == MDL_RET_SUCCESS) {

		_os_mem_cpy(d, &color_info, cmd.data, cmd.len);

		/* get custom info by request */

		/* assign bss color with valid range 1 to 63 */
		if (color_info.is_valid == true)
			protocol_cap->bsscolor = color_info.bsscolor;

		PHL_INFO(
		    "%s, cap_info bss color(%d), protocol_cap bss_color(%d)\n",
		    __func__, color_info.bsscolor, protocol_cap->bsscolor);
	} else {
		status = RTW_PHL_STATUS_FAILURE;
	}
	return status;
}
#endif
