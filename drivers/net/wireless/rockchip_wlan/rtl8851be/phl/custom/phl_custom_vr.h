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
#ifndef _PHL_CUSTOM_VR_H_
#define _PHL_CUSTOM_VR_H_

#ifdef CONFIG_PHL_CUSTOM_FEATURE_VR
#include "phl_custom_vr_csi.h"

#define MAX_TX_STATS_RING_ENTRY_NUM 10

struct _vr_init_param {
	u32 enable;
	u32 test_mode;
	struct rtw_wifi_role_t *wifi_role;
        struct phl_info_t *phl;
};

struct _vr_tx_rty_param {
	u32 enable;
	u8 rty_rate_tbl[24]; /* 1SS MCS0 to 2SS MCS11 */
};

struct _vr_rf_scrmb {
	u8 is_set;
	u8 status;
	enum rtw_mac_usr_frame_to_act_mode mode;
	u32 to_thr;
	u8 trigger_cnt;
	u16 sw_def_bmp; /* enum mac_ax_wd_sw_def_bmp */
};

struct _vr_bss_color {
	u8 bsscolor;
	bool is_valid;
};

#ifdef CONFIG_PHL_CUSTOM_FRAME_STAT
/**
 * this struct is the stats rpt config param shared with sdk.
 * @mode: enum _usr_tx_stats_rpt_mode
 * @macid: client's macid
 * @period: only use if mode is USR_TX_STATS_RPT_PERIOD
 */
struct _vr_stats_rpt_cfg {
    u32 mode;
    u32 macid;
    u32 rpt_period_us;
};

struct _tx_stats_entry {
    u32 total_pkt_cnt;
    u32 recv_rpt_cnt;
    struct rtw_phl_vr_frame_stats_rpt cus_rpt;
    bool rpt_sent;
};

struct _tx_stats_ring {
    struct _tx_stats_entry entry[MAX_TX_STATS_RING_ENTRY_NUM];
    _os_lock lock;
    u8 r_idx;
    u8 w_idx;
};

struct _vr_stats_rpt_param {
    struct _vr_stats_rpt_cfg cfg;
    /* tx stats */
    struct _tx_stats_ring t_stats_ring;
    u32 tx_pending_byte[4];
    u32 tx_pending_cnt[4];
    /* for general rpt */
    _os_timer general_rpt_timer;
    u32 last_busy_cnt_rst_time;
    u32 last_tx_busy_cnt;
    u32 last_rx_busy_cnt;
};

#endif /*CONFIG_PHL_CUSTOM_FRAME_STAT*/

struct _custom_vr_ctx {
	struct _vr_init_param init;
	struct _vr_tx_rty_param tx_rty_param;
	struct _vr_rf_scrmb cur_scrmb_param;
	struct _vr_bss_color bsscolor_param;
#ifdef CONFIG_PHL_CHANNEL_INFO_VR
	struct custom_csi_ctrl csi_ctrl;
#endif /* CONFIG_PHL_CHANNEL_INFO_VR */
#ifdef CONFIG_PHL_CUSTOM_FRAME_STAT
	struct _vr_stats_rpt_param stats_rpt;
#endif /*CONFIG_PHL_CUSTOM_FRAME_STAT*/
};

enum phl_mdl_ret_code
phl_custom_hdl_vr_evt(void* dispr,
                      void* custom_ctx,
                      struct _custom_vr_ctx* vr_ctx,
                      struct phl_msg* msg);
enum phl_mdl_ret_code
phl_custom_hdl_vr_fail_evt(void* dispr,
                           void* custom_ctx,
                           struct _custom_vr_ctx* vr_ctx,
                           struct phl_msg* msg);

enum phl_mdl_ret_code
phl_custom_vr_feature_set_hdlr(void* dispr,
                               void* custom_ctx,
                               struct _custom_vr_ctx* vr_ctx,
                               struct rtw_custom_decrpt *cmd);

enum phl_mdl_ret_code
phl_custom_vr_feature_query_hdlr(void* dispr,
                                 void* custom_ctx,
                                 struct _custom_vr_ctx* vr_ctx,
                                 struct rtw_custom_decrpt *cmd);

enum phl_mdl_ret_code
phl_custom_vr_set_role_cap(void* dispr,
                           void* custom_ctx,
                           struct _custom_vr_ctx* vr_ctx,
                           struct phl_module_op_info* info);

enum rtw_phl_status phl_custom_vr_init_protocol_cap(
    struct phl_info_t *phl_info, u8 hw_band, enum role_type rtype,
    struct protocol_cap_t *protocol_cap);

#ifdef CONFIG_PHL_CUSTOM_FRAME_STAT
enum rtw_phl_status phl_custom_vr_txsts_notify(struct phl_info_t *phl_info,
					       u8 macid, u16 ac, u8 txsts);
#endif

#else
#define phl_custom_hdl_vr_evt(_dispr, _custom_ctx, _vr_ctx, _msg) (MDL_RET_IGNORE)
#define phl_custom_hdl_vr_fail_evt(_dispr, _custom_ctx, _vr_ctx, _msg) (MDL_RET_IGNORE)
#define phl_custom_vr_feature_set_hdlr(_dispr, _custom_ctx, _vr_ctx, _cmd) (MDL_RET_IGNORE)
#define phl_custom_vr_feature_query_hdlr(_dispr, _custom_ctx, _vr_ctx, _cmd) (MDL_RET_IGNORE)
#define phl_custom_vr_init_protocol_cap(_phl_info, _hw_band, _rtype,           \
					_protocol_cap)                         \
	(RTW_PHL_STATUS_SUCCESS)
#endif

#endif  /*_PHL_CUSTOMIZE_FEATURE_H_*/

