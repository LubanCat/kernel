/*
 * This module is designed to collect internal diagnostic information.
 *
 *	Author: Cosa
 *	History: Created at 2023/02/03
 */

#ifndef _PHL_DIAGNOSE_H_
#define _PHL_DIAGNOSE_H_

#define MAX_PHL_DIAG_MSG_LEN 1024
#define INVALID_SUBMODULE_DIAG_EVT 0xff

enum rtw_phl_diag_evt_level {
	PHL_DIAG_LVL_FATAL = 0x0, /* unrecoverable error */
	PHL_DIAG_LVL_CRITICAL = 0x1, /* severe error events */
	PHL_DIAG_LVL_ERROR = 0x2, /* non-severe error events */
	PHL_DIAG_LVL_MAX
};

enum rtw_phl_diag_evt_type {
	PHL_DIAG_EVT_MAC = 0,
	PHL_DIAG_EVT_BB,
	PHL_DIAG_EVT_RF,
	PHL_DIAG_EVT_BTC,
	PHL_DIAG_EVT_PS_FAIL,
	PHL_DIAG_EVT_SER,
	PHL_DIAG_EVT_MAX
};

/* backward compatibility */
#define PHL_DIAG_EVT_LEAVE_PS_FAIL PHL_DIAG_EVT_PS_FAIL

enum rtw_phl_diag_ser_type {
	PHL_DIAG_SER_L1,
	PHL_DIAG_SER_L2,
	PHL_DIAG_SER_UNKNOWN,
};

enum rtw_phl_diag_ps_mode {
	PHL_DIAG_PS_LPS,
	PHL_DIAG_PS_IPS,
	PHL_DIAG_PS_UNKNOWN,
};

enum rtw_phl_diag_ps_op {
	PHL_DIAG_PS_OP_ENTER,
	PHL_DIAG_PS_OP_LEAVE,
	PHL_DIAG_PS_OP_UNKNOWN,
};

enum rtw_phl_diag_pwr_lvl {
	PHL_DIAG_PWR_LVL_PWROFF,
	PHL_DIAG_PWR_LVL_PWR_GATED,
	PHL_DIAG_PWR_LVL_CLK_GATED,
	PHL_DIAG_PWR_LVL_RF_OFF,
	PHL_DIAG_PWR_LVL_PWR_ON,
	PHL_DIAG_PWR_LVL_UNKNOWN
};

enum rtw_phl_bb_dbg_type {
	PHL_BB_PMAC = BIT(0),
	PHL_BB_UTILITY = BIT(1),
};


struct diag_ser_content_v1 {
	u8 diag_ser_type;
};

/* deprecated */
struct diag_leave_ps_fail_content_v1 {
	u8 diag_ps_mode;
};

/* rename and update from "struct diag_leave_ps_fail_content_v1" */
struct diag_ps_fail_content_v2 {
	u8 diag_ps_mode;
	u8 diag_pwr_lvl;
	u8 diag_ps_op;
	u32 diag_u_proto_cfg;
	u8 diag_u_pwr_cfg;
};

enum rtw_phl_rfk_err_type {
	PHL_RFK_ERR_IQK = BIT(0),
	PHL_RFK_ERR_DPK = BIT(1),
	PHL_RFK_ERR_DACK = BIT(2),
	PHL_RFK_ERR_RXDCK = BIT(3),
	PHL_RFK_ERR_TXGAPK = BIT(4),
	PHL_RFK_ERR_TSSI = BIT(5),
};

struct rtw_phl_rfk_err_code_v1 {
	u32 iqk;
	u32 dpk;
	u32 dack;
	u32 rxdck;
	u32 txgapk;
	u32 tssi;
};

struct rtw_phl_diag_hal_buf {
	u8 ver;
	u32 len;
	u8 *buf;
};

struct rtw_phl_diag_rf_info {
	u8 ver;
	u32 len; /* length of raw data */
	u8 *buf; /* raw data, alloc by upper layer */
	struct rtw_phl_rfk_err_code_v1 err_code;
	struct rtw_phl_diag_hal_buf iqk_rpt; /* point to raw data's iqk section */
	struct rtw_phl_diag_hal_buf dpk_rpt; /* point to raw data's dpk section */
	struct rtw_phl_diag_hal_buf rt_rpt; /* point to raw data's rt section */
};

struct rtw_phl_diag_msg {
	enum rtw_phl_diag_evt_type type;
	enum rtw_phl_diag_evt_level level;
	u8 sub_evt; /* submodule event id */
	u8 ver; /* version of debug content */
	u32 len;
	u8 buf[MAX_PHL_DIAG_MSG_LEN];
};

bool phl_send_diag_hub_msg(void *phl, u16 phl_evt, u8 sub_evt,
	u8 level, u8 ver, u8 *buf, u32 len);

#endif /* _PHL_DIAGNOSE_H_ */

