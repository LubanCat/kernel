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
#define _PHL_TEST_MP_CONFIG_C_
#include "../../phl_headers.h"
#include "phl_test_mp_def.h"
#include "../../hal_g6/test/mp/hal_test_mp_api.h"

#define MP_VER_BRANCH 19
#define MP_VER_MAJOR 1
#define MP_VER_MINOR 0
#define MP_VER_RLS 0

#ifdef CONFIG_PHL_TEST_MP
static enum rtw_phl_status phl_mp_config_start_dut(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_start_dut(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_get_device_info(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_get_dev_info(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_phy_idx(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_set_phy_idx(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_get_bw_mode(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_get_bw_mode(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_trx_path(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_set_trx_path(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status
phl_mp_config_reset_phy_cnt(struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_reset_phy_cnt(mp);
	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status
phl_mp_config_reset_mac_cnt(struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_reset_mac_cnt(mp);
	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status
phl_mp_config_reset_drv_cnt(struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_reset_drv_cnt(mp);
	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_modulation(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_set_modulation(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_trx_mode(struct mp_context *mp,
                                                      struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;


	if (arg->is_tmac_mode) {
		mp->phl_com->drv_mode = RTW_DRV_MODE_MP_TMAC;
	} else {
		mp->phl_com->drv_mode = RTW_DRV_MODE_MP;
	}

	PHL_INFO("%s: %s mode\n", __FUNCTION__, (arg->is_tmac_mode) ? "TMAC" : "PMAC");

	hal_status = rtw_hal_mp_config_set_trx_mode(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_get_modulation(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_get_modulation(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_rate(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_set_rate(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_get_mac_addr(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_get_mac_addr(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_mac_addr(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_set_mac_addr(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_ch_bw(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_set_ch_bw(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_get_tx_nss(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	struct rtw_phl_com_t *phl_com = mp->phl_com;

	arg->ant_tx = phl_com->phy_cap[mp->cur_phy].txss;

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = RTW_HAL_STATUS_SUCCESS;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_get_rx_nss(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	struct rtw_phl_com_t *phl_com = mp->phl_com;
	arg->ant_rx = phl_com->phy_cap[mp->cur_phy].rxss;

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = RTW_HAL_STATUS_SUCCESS;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_switch_btc_path(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_swith_btc_path(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = RTW_HAL_STATUS_SUCCESS;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_get_rfe_type(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	struct rtw_phl_com_t *phl_com = mp->phl_com;
	arg->rfe_type = phl_com->dev_cap.rfe_type;

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = RTW_HAL_STATUS_SUCCESS;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_trigger_fw_conflict(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_trigger_fw_conflict(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = RTW_HAL_STATUS_SUCCESS;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_get_uuid(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	arg->uuid = rtw_hal_get_uuid(mp);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = RTW_HAL_STATUS_SUCCESS;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_regulation(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	rtw_hal_set_regulation(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = RTW_HAL_STATUS_SUCCESS;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_gpio(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_set_gpio(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = hal_status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_get_drv_ver(
	struct mp_context *mp, struct mp_config_arg *arg)
{

	arg->drv_ver += (MP_VER_BRANCH << 24);
	arg->drv_ver += (MP_VER_MAJOR << 16);
	arg->drv_ver += (MP_VER_MINOR << 8);
	arg->drv_ver += MP_VER_RLS;

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = RTW_HAL_STATUS_SUCCESS;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_switch_antenna(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	enum rtw_hal_status hal_status = RTW_HAL_STATUS_FAILURE;

	hal_status = rtw_hal_mp_config_switch_antenna(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = RTW_HAL_STATUS_SUCCESS;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_bt_uart_en(struct mp_context *mp,
                                                        struct mp_config_arg *arg)
{
	rtw_hal_set_bt_uart_en(mp, arg);

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = RTW_HAL_STATUS_SUCCESS;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_mac_loopbk_enter(
	struct mp_context *mp, struct mp_config_arg *arg)
{

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = rtw_hal_set_mac_loopbk_enter(mp, arg);

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_hci_speed(
	struct mp_context *mp, struct mp_config_arg *arg)
{

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = rtw_hal_mp_set_hci_speed(mp, arg);
	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_get_hci_speed(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	/* Record the result */
	arg->cmd_ok = true;
	arg->status = rtw_hal_mp_get_hci_speed(mp, arg);

	PHL_INFO("%s: speed (%d)\n", __FUNCTION__, arg->hci_speed);

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_mac_general_io_test(
	struct mp_context *mp, struct mp_config_arg *arg)
{

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = rtw_hal_set_mac_fw_general_io_test(mp, arg);
	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_mac_l1ss_enable(
	struct mp_context *mp, struct mp_config_arg *arg)
{

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = rtw_hal_mp_set_mac_l1ss_enable(mp, arg);

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_mac_aspm_test(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	/* Record the result */
	arg->cmd_ok = true;
	arg->status = rtw_hal_set_mac_aspm_test(mp);

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_set_power_state(
    struct mp_context *mp, struct mp_config_arg *arg)
{
#ifdef CONFIG_POWER_SAVE
	enum rtw_phl_status status = RTW_PHL_STATUS_SUCCESS;
	struct ps_cfg cfg = {0};
	struct rtw_wifi_role_t *wrole = NULL;
	struct rtw_wifi_role_link_t *rlink = NULL;
	struct rtw_phl_stainfo_t *sta = NULL;
	u8 tar_pwr_lvl = arg->pwr_cfg.pwr_lvl;

	/* common configuration and no connection in mp case */
	cfg.pwr_cfg = true;
	cfg.ps_mode = PS_MODE_IPS;
	cfg.cur_pwr_lvl = mp->cur_pwr_lvl;

	/* configuration by case */
	switch (arg->pwr_cfg.pwr_state) {
	case MP_PS_RX_IDLE:
		PHL_INFO("%s: leave ips\n", __func__);
		if (mp->cur_pwr_lvl == PS_PWR_LVL_PWRON) {
			PHL_ERR("%s: cur state %d is invalid\n", __func__,
				mp->cur_pwr_lvl);
			status = RTW_PHL_STATUS_FAILURE;
			break;
		}
		cfg.macid = mp->ps_macid;
		cfg.pwr_lvl = PS_PWR_LVL_PWRON;
		cfg.proto_cfg =
		    (mp->cur_pwr_lvl == PS_PWR_LVL_PWROFF) ? false : true;
		status = phl_ps_leave_ps(mp->phl, &cfg);
		if (status != RTW_PHL_STATUS_SUCCESS) {
			PHL_ERR("%s: leave ps fail\n", __func__);
			status = RTW_PHL_STATUS_FAILURE;
			break;
		}
		mp->cur_pwr_lvl = PS_PWR_LVL_PWRON;
		break;
	case MP_PS_DRIVER_IPS:
		PHL_INFO("%s: driver ips\n", __func__);
		if (mp->cur_pwr_lvl != PS_PWR_LVL_PWRON) {
			PHL_ERR("%s: cur state %d is invalid\n", __func__,
				mp->cur_pwr_lvl);
			status = RTW_PHL_STATUS_FAILURE;
			break;
		}

		cfg.macid = PS_MACID_NONE;
		cfg.pwr_lvl = PS_PWR_LVL_PWROFF;
		cfg.proto_cfg = false;
		status = phl_ps_enter_ps(mp->phl, &cfg);
		if (status != RTW_PHL_STATUS_SUCCESS) {
			PHL_ERR("%s: enter ps fail\n", __func__);
			status = RTW_PHL_STATUS_FAILURE;
			break;
		}
		mp->cur_pwr_lvl = PS_PWR_LVL_PWROFF;
		mp->ps_macid = PS_MACID_NONE;
		break;
	case MP_PS_FW_IPS:
		PHL_INFO("%s: fw ips pg\n", __func__);
		if (mp->cur_pwr_lvl != PS_PWR_LVL_PWRON) {
			PHL_ERR("%s: cur state %d is invalid\n", __func__,
				mp->cur_pwr_lvl);
			status = RTW_PHL_STATUS_FAILURE;
			break;
		}
		if (tar_pwr_lvl != PS_PWR_LVL_PWR_GATED &&
		    tar_pwr_lvl != PS_PWR_LVL_CLK_GATED) {
			PHL_ERR("%s: target level %d is invalid\n", __func__,
				tar_pwr_lvl);
			status = RTW_PHL_STATUS_FAILURE;
			break;
		}
		wrole = phl_mr_get_role_by_bandidx(
		    mp->phl, (u8)rtw_hal_phy_idx_to_hw_band(arg->phy_idx));
		if (wrole == NULL) {
			PHL_ERR("%s: wrole is null with phy_idx(%d)!!",
				__func__, arg->phy_idx);
			status = RTW_PHL_STATUS_FAILURE;
			return status;
		}
		rlink = get_rlink(wrole, RTW_RLINK_PRIMARY);
		sta = rtw_phl_get_stainfo_self(mp->phl, rlink);

		cfg.macid = sta->macid;
		cfg.pwr_lvl = tar_pwr_lvl;
		cfg.proto_cfg = true;
		status = phl_ps_enter_ps(mp->phl, &cfg);
		if (status != RTW_PHL_STATUS_SUCCESS) {
			PHL_ERR("%s: enter ps fail\n", __func__);
			status = RTW_PHL_STATUS_FAILURE;
			break;
		}
		mp->cur_pwr_lvl = tar_pwr_lvl;
		mp->ps_macid = cfg.macid;
		break;
	case MP_PS_MAX:
	default:
		PHL_INFO("%s: unknown mp ps command\n", __func__);
		status = RTW_PHL_STATUS_FAILURE;
		break;
	}

	/* Record the result */
	arg->cmd_ok = true;
	arg->status = status;

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;
#endif
	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_get_max_hci_speed(
    struct mp_context *mp, struct mp_config_arg *arg)
{
	/* Record the result */
	arg->cmd_ok = true;
	arg->status = rtw_hal_mp_get_max_hci_speed(mp, arg);

	PHL_INFO("%s: speed (%d)\n", __FUNCTION__, arg->hci_speed);

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

static enum rtw_phl_status phl_mp_config_enable_bb_rf(
	struct mp_context *mp, struct mp_config_arg *arg)
{
	/* Record the result */
	arg->cmd_ok = true;
	arg->status = rtw_hal_mp_enable_bb_rf(mp, arg);

	PHL_INFO("%s: en_phy (%d)\n", __FUNCTION__, arg->en_phy);

	/* Transfer to report */
	mp->rpt = arg;
	mp->rpt_len = sizeof(struct mp_config_arg);
	mp->buf = NULL;
	mp->buf_len = 0;

	return RTW_PHL_STATUS_SUCCESS;
}

enum rtw_phl_status mp_config(struct mp_context *mp,struct mp_config_arg *arg)
{
	enum rtw_phl_status phl_status = RTW_PHL_STATUS_FAILURE;

	switch(arg->cmd) {
	case MP_CONFIG_CMD_START_DUT:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_START_DUT\n", __FUNCTION__);
		phl_status = phl_mp_config_start_dut(mp, arg);
		break;
	case MP_CONFIG_CMD_GET_DEVICE_INFO:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_GET_DEVICE_INFO\n",
			 __FUNCTION__);
		phl_status = phl_mp_config_get_device_info(mp, arg);
		break;
	case MP_CONFIG_CDM_SET_PHY_INDEX:
		PHL_INFO("%s: CMD = MP_CONFIG_CDM_SET_PHY_INDEX\n",
			 __FUNCTION__);
		phl_status = phl_mp_config_set_phy_idx(mp, arg);
		break;
	case MP_CONFIG_CMD_GET_BW:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_GET_BW\n", __FUNCTION__);
		phl_status = phl_mp_config_get_bw_mode(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_RF_TXRX_PATH:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_RF_PATH\n",
			 __FUNCTION__);
		phl_status = phl_mp_config_set_trx_path(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_RESET_PHY_COUNT:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_RESET_PHY_COUNT\n",
			 __FUNCTION__);
		phl_status = phl_mp_config_reset_phy_cnt(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_RESET_MAC_COUNT:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_RESET_MAC_COUNT\n",
			 __FUNCTION__);
		phl_status = phl_mp_config_reset_mac_cnt(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_RESET_DRV_COUNT:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_RESET_DRV_COUNT\n",
			 __FUNCTION__);
		phl_status = phl_mp_config_reset_drv_cnt(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_MODULATION:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_MODULATION\n",
			 __FUNCTION__);
		phl_status = phl_mp_config_set_modulation(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_TXRX_MODE:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_TXRX_MODE\n",
			 __FUNCTION__);
		phl_status = phl_mp_config_set_trx_mode(mp, arg);
		break;
	case MP_CONFIG_CMD_GET_MODULATION:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_GET_MODULATION\n",
			 __FUNCTION__);
		phl_status = phl_mp_config_get_modulation(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_RATE_IDX:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_RATE_IDX\n",
			 __FUNCTION__);
		phl_status = phl_mp_config_set_rate(mp, arg);
		break;
	case MP_CONFIG_CMD_GET_MAC_ADDR:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_GET_MAC_ADDR\n",
			 __FUNCTION__);
		phl_status = phl_mp_config_get_mac_addr(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_MAC_ADDR:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_MAC_ADDR\n",
			 __FUNCTION__);
		phl_status = phl_mp_config_set_mac_addr(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_CH_BW:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_CH_BW\n", __FUNCTION__);
		phl_status = phl_mp_config_set_ch_bw(mp, arg);
		break;
	case MP_CONFIG_CMD_GET_TX_NSS:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_GET_TX_NSS\n", __FUNCTION__);
		phl_status = phl_mp_config_get_tx_nss(mp, arg);
		break;
	case MP_CONFIG_CMD_GET_RX_NSS:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_GET_RX_NSS\n", __FUNCTION__);
		phl_status = phl_mp_config_get_rx_nss(mp, arg);
		break;
	case MP_CONFIG_CMD_SWITCH_BT_PATH:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SWITCH_BT_PATH\n", __FUNCTION__);
		phl_status = phl_mp_config_switch_btc_path(mp, arg);
		break;
	case MP_CONFIG_CMD_GET_RFE_TYPE:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_GET_RFE_TYPE\n", __FUNCTION__);
		phl_status = phl_mp_config_get_rfe_type(mp, arg);
		break;
	case MP_CONFIG_CMD_TRIGGER_FW_CONFLICT:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_GET_FW_RPT\n", __FUNCTION__);
		phl_status = phl_mp_config_trigger_fw_conflict(mp, arg);
		break;
	case MP_CONFIG_CMD_GET_UUID:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_GET_UUID\n", __FUNCTION__);
		phl_status = phl_mp_config_get_uuid(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_REGULATION:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_REGULATION\n", __FUNCTION__);
		phl_status = phl_mp_config_set_regulation(mp, arg);
		break;
	case MP_CONFIG_CMD_GET_DRV_VER:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_GET_DRV_VER\n", __FUNCTION__);
		phl_status = phl_mp_config_get_drv_ver(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_BT_UART:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_BT_UART\n", __FUNCTION__);
		phl_status = phl_mp_config_set_bt_uart_en(mp, arg);
		break;
	case MP_CONFIG_CMD_SWITCH_ANTENNA:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SWITCH_ANTENNA\n", __FUNCTION__);
		phl_status = phl_mp_config_switch_antenna(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_MAC_LOOPBK_ENTER:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_MAC_LOOPBK_ENTER\n", __FUNCTION__);
		phl_status = phl_mp_config_set_mac_loopbk_enter(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_HCI_SPEED:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_HCI_SPEED\n", __FUNCTION__);
		phl_status = phl_mp_config_set_hci_speed(mp, arg);
		break;
	case MP_CONFIG_CMD_GET_HCI_SPEED:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_GET_HCI_SPEED\n", __FUNCTION__);
		phl_status = phl_mp_config_get_hci_speed(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_MAC_GENERNAL_IO_TEST:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_MAC_GENERNAL_IO_TEST\n", __FUNCTION__);
		phl_status = phl_mp_config_set_mac_general_io_test(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_MAC_L1SS_ENABLE:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_MAC_L1SS_ENABLE\n", __FUNCTION__);
		phl_status = phl_mp_config_set_mac_l1ss_enable(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_MAC_ASPM_STATE:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_MAC_ASPM_STATE\n", __FUNCTION__);
		phl_status = phl_mp_config_set_mac_aspm_test(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_GPIO:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_GPIO\n", __FUNCTION__);
		phl_status = phl_mp_config_set_gpio(mp, arg);
		break;
	case MP_CONFIG_CMD_SET_MAC_PWR_STATE:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_SET_MAC_PWR_STATE\n", __FUNCTION__);
		phl_status = phl_mp_config_set_power_state(mp, arg);
		break;
	case MP_CONFIG_CMD_GET_MAX_HCI_SPEED:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_GET_MAX_HCI_SPEED\n", __FUNCTION__);
		phl_status = phl_mp_config_get_max_hci_speed(mp, arg);
		break;
	case MP_CONFIG_CMD_ENABLE_PHY:
		PHL_INFO("%s: CMD = MP_CONFIG_CMD_ENABLE_PHY\n", __FUNCTION__);
		phl_status = phl_mp_config_enable_bb_rf(mp, arg);
		break;
	default:
		PHL_WARN("%s: CMD NOT RECOGNIZED\n", __FUNCTION__);
		break;
	}

	return phl_status;
}
#endif /* CONFIG_PHL_TEST_MP */
