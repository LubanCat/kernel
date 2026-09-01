/** @file */
/******************************************************************************
 *
 * Copyright(c) 2019 Realtek Corporation. All rights reserved.
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
 ******************************************************************************/

#ifndef _MAC_AX_STATE_MACH_H_
#define _MAC_AX_STATE_MACH_H_

#define MAC_AX_DFLT_SM \
	{{MAC_AX_MCC_EMPTY}, {MAC_AX_MCC_ADD_ROLE_OK}, {MAC_AX_MCC_REQ_IDLE}, {MAC_AX_MCC_REQ_OK}, \
	MAC_AX_PWR_OFF, MAC_AX_FWDL_IDLE, MAC_AX_EFUSE_IDLE, \
	MAC_AX_CMD_OFLD_IDLE, MAC_AX_OFLD_H2C_IDLE, MAC_AX_OFLD_H2C_IDLE, \
	MAC_AX_OFLD_H2C_IDLE, MAC_AX_OFLD_H2C_IDLE, \
	MAC_AX_FW_RESET_IDLE, MAC_AX_AOAC_RPT_IDLE, \
	MAC_AX_P2P_ACT_IDLE, MAC_AX_FUNC_OFF, MAC_AX_FUNC_OFF, \
	MAC_AX_FUNC_OFF, MAC_AX_FUNC_OFF, MAC_AX_FUNC_OFF, \
	MAC_AX_WOW_STOPTRX_IDLE, MAC_AX_MAC_NOT_RDY, MAC_AX_ROLE_ALOC_SUCC, \
	MAC_AX_PLAT_OFF, MAC_AX_IO_ST_NORM, MAC_AX_L2_EN, MAC_AX_SER_CTRL_SRT,\
	MAC_AX_OFLD_H2C_IDLE, MAC_AX_PROXY_IDLE, MAC_AX_PROXY_IDLE, MAC_AX_SENSING_CSI_IDLE,\
	MAC_AX_NAN_IDLE, MAC_AX_STA_CSA_IDLE, MAC_AX_STA_CSA_IDLE, MAC_AX_H2C_C2H_MON_OFF, MAC_AX_H2C_LB_IDLE}

enum mac_ax_mcc_status {
	MAC_AX_MCC_ADD_ROLE_OK = 0,
	MAC_AX_MCC_START_GROUP_OK = 1,
	MAC_AX_MCC_STOP_GROUP_OK = 2,
	MAC_AX_MCC_DEL_GROUP_OK = 3,
	MAC_AX_MCC_RESET_GROUP_OK = 4,

	//fail status
	MAC_AX_MCC_EMPTY_GRP_FAIL = 16,
	MAC_AX_MCC_ROLE_NOT_EXIST_FAIL = 17,
	MAC_AX_MCC_DATA_NOT_FOUND_FAIL = 18,
	MAC_AX_MCC_ACT_INVALID_FAIL = 19,
	MAC_AX_MCC_BANDTYPE_INVALID_FAIL = 20,
	MAC_AX_MCC_ADD_PSTIMER_FAIL = 21,
	MAC_AX_MCC_MALLOC_FAIL = 22,
	MAC_AX_MCC_SWITCH_CH_FAIL = 23,
	MAC_AX_MCC_TXNULL0_FAIL = 24,
	MAC_AX_MCC_PORT_FUNC_EN_FAIL = 25,
};

enum mac_ax_mcc_req_status {
	MAC_AX_MCC_REQ_OK = 0,

	//fail status
	MAC_AX_MCC_REQ_ROLE_NOT_EXIST_FAIL = 17,
	MAC_AX_MCC_REQ_DATA_NOT_FOUND_FAIL = 18,
	MAC_AX_MCC_REQ_ADD_PSTIMER_FAIL = 21,
	MAC_AX_MCC_REQ_MALLOC_FAIL = 22,
};

enum mac_ax_mcc_stm_state {
	MAC_AX_MCC_EMPTY = 0,
	MAC_AX_MCC_STATE_H2C_SENT = 1,
	MAC_AX_MCC_STATE_H2C_RCVD = 2,
	MAC_AX_MCC_ADD_DONE = 3,
	MAC_AX_MCC_START_DONE = 4,
	MAC_AX_MCC_STOP_DONE = 5,
	MAC_AX_MCC_STATE_ERROR = 6,
};

enum mac_ax_mcc_req_state {
	MAC_AX_MCC_REQ_IDLE = 0,
	MAC_AX_MCC_REQ_H2C_SENT = 1,
	MAC_AX_MCC_REQ_H2C_RCVD = 2,
	MAC_AX_MCC_REQ_DONE = 3,
	MAC_AX_MCC_REQ_FAIL = 4,
};

enum mac_ax_efuse_state {
	MAC_AX_EFUSE_IDLE = 0,
	MAC_AX_EFUSE_PHY = 1,
	MAC_AX_EFUSE_LOG_MAP = 2,
	MAC_AX_EFUSE_LOG_MASK = 3,
	MAC_AX_EFUSE_MAX = 4,
};

enum mac_ax_pwr_state {
	MAC_AX_PWR_OFF = 0,
	MAC_AX_PWR_ON = 1,
	MAC_AX_PWR_PRE_OFF = 2,
	MAC_AX_PWR_ERR = 3,
};

enum mac_ax_fwdl_state {
	MAC_AX_FWDL_IDLE = 0,
	MAC_AX_FWDL_CPU_ON = 1,
	MAC_AX_FWDL_H2C_PATH_RDY = 2,
	MAC_AX_FWDL_PATH_RDY = 3,
	MAC_AX_FWDL_INIT_RDY = 4,
};

enum mac_ax_cmd_ofld_state {
	MAC_AX_CMD_OFLD_IDLE = 0,
	MAC_AX_CMD_OFLD_PROC = 1,
	MAC_AX_CMD_OFLD_SENDING = 2,
	MAC_AX_CMD_OFLD_RCVD = 3,
};

enum mac_ax_ofld_h2c_state {
	MAC_AX_OFLD_H2C_IDLE = 0,
	MAC_AX_OFLD_H2C_SENDING = 1,
	MAC_AX_OFLD_H2C_RCVD = 2,
	MAC_AX_OFLD_H2C_DONE = 3,
	MAC_AX_OFLD_H2C_ERROR = 4,
};

enum mac_ax_fw_reset_state {
	MAC_AX_FW_RESET_IDLE = 0,
	MAC_AX_FW_RESET_RECV = 1,
	MAC_AX_FW_RESET_RECV_DONE = 2,
	MAC_AX_FW_RESET_PROCESS = 3,
};

enum mac_ax_aoac_rpt_state {
	MAC_AX_AOAC_RPT_IDLE = 0,
	MAC_AX_AOAC_RPT_H2C_SENDING = 1,
	MAC_AX_AOAC_RPT_H2C_RCVD = 2,
	MAC_AX_AOAC_RPT_H2C_DONE = 3,
	MAC_AX_AOAC_RPT_ERROR = 4,
};

enum mac_ax_p2p_act_state {
	MAC_AX_P2P_ACT_IDLE = 0,
	MAC_AX_P2P_ACT_BUSY = 1,
	MAC_AX_P2P_ACT_FAIL = 2,
};

enum mac_ax_func_state {
	MAC_AX_FUNC_OFF = 0,
	MAC_AX_FUNC_ON = 1,
};

enum mac_ax_wow_stoptrx_state {
	MAC_AX_WOW_STOPTRX_IDLE = 0,
	MAC_AX_WOW_STOPTRX_BUSY = 1,
	MAC_AX_WOW_STOPTRX_FAIL = 2,
};

enum mac_ax_mac_state {
	MAC_AX_MAC_NOT_RDY = 0,
	MAC_AX_MAC_RDY = 1,
	MAC_AX_MAC_INIT_ERR = 2,
	MAC_AX_MAC_DEINIT_ERR = 3,
	MAC_AX_MAC_FINIT_ERR = 4,
	MAC_AX_MAC_FDEINIT_ERR = 5,
};

enum mac_ax_role_state {
	MAC_AX_ROLE_ALOC_SUCC = 0,
	MAC_AX_ROLE_INIT_SUCC = 1,
	MAC_AX_ROLE_HW_UPD_SUCC = 2,
	MAC_AX_ROLE_ALOC_FAIL = 3,
	MAC_AX_ROLE_INIT_FAIL = 4,
	MAC_AX_ROLE_HW_UPD_FAIL = 5,
};

enum mac_ax_plat_state {
	MAC_AX_PLAT_OFF = 0,
	MAC_AX_PLAT_ON = 1,
};

enum mac_ax_io_st_state {
	MAC_AX_IO_ST_NORM = 0,
	MAC_AX_IO_ST_HANG = 1,
};

enum mac_ax_l2_state {
	MAC_AX_L2_DIS = 0,
	MAC_AX_L2_EN = 1,
	MAC_AX_L2_TRIG = 2,
};

enum mac_ax_ser_ctrl_state {
	MAC_AX_SER_CTRL_SRT = 0,
	MAC_AX_SER_CTRL_STOP = 1,
	MAC_AX_SER_CTRL_ERR = 2,
};

enum mac_ax_ch_switch_state {
	MAC_AX_CH_SWITCH_GET_RPT = 4,
};

enum mac_ax_proxy_state {
	MAC_AX_PROXY_IDLE = 0,
	MAC_AX_PROXY_SENDING = 1,
	MAC_AX_PROXY_BUSY = 2,
};

enum mac_ax_sensing_csi_state {
	MAC_AX_SENSING_CSI_IDLE = 0,
	MAC_AX_SENSING_CSI_SENDING = 1,
};

enum mac_ax_nan_state {
	MAC_AX_NAN_IDLE = 0,
	MAX_AX_NAN_ACT_H2C_FAIL = 1,
	MAX_AX_NAN_ACT_H2C_DONE = 2,
};

enum mac_ax_sta_csa_state {
	MAC_AX_STA_CSA_IDLE = 0,
	MAC_AX_STA_CSA_SENDING = 1,
	MAC_AX_STA_CSA_BUSY = 2,
};

enum mac_ax_h2c_c2h_mon_state {
	MAC_AX_H2C_C2H_MON_OFF = 0,
	MAC_AX_H2C_C2H_MON_ON = 1,
};

enum mac_ax_h2c_lb_state {
	MAC_AX_H2C_LB_IDLE = 0,
	MAC_AX_H2C_LB_PROC = 1,
};

/**
 * @struct mac_ax_state_mach
 * @brief mac_ax_state_mach
 *
 * @var mac_ax_state_mach::pwr
 * Please Place Description here.
 * @var mac_ax_state_mach::fwdl
 * Please Place Description here.
 * @var mac_ax_state_mach::efuse
 * Please Place Description here.
 * @var mac_ax_state_mach::read_request
 * Please Place Description here.
 * @var mac_ax_state_mach::write_request
 * Please Place Description here.
 * @var mac_ax_state_mach::conf_request
 * Please Place Description here.
 * @var mac_ax_state_mach::write_h2c
 * Please Place Description here.
 * @var mac_ax_state_mach::conf_h2c
 * Please Place Description here.
 * @var mac_ax_state_mach::read_h2c
 * Please Place Description here.
 * @var mac_ax_state_mach::pkt_ofld
 * Please Place Description here.
 * @var mac_ax_state_mach::efuse_ofld
 * Please Place Description here.
 * @var mac_ax_state_mach::macid_pause
 * Please Place Description here.
 * @var mac_ax_state_mach::mcc_group
 * Please Place Description here.
 * @var mac_ax_state_mach::mcc_request
 * Please Place Description here.
 * @var mac_ax_state_mach::fw_rst
 * Please Place Description here.
 * @var mac_ax_state_mach::aoac_rpt
 * Please Place Description here.
 * @var mac_ax_state_mach::p2p_stat
 * Please Place Description here.
 * @var mac_ax_state_mach::nan_stat
 * Please Place Description here.
 */
struct mac_ax_state_mach {
	enum mac_ax_mcc_stm_state mcc_group[4];
	enum mac_ax_mcc_status mcc_group_state[4];
	enum mac_ax_mcc_req_state mcc_request[4];
	enum mac_ax_mcc_req_status mcc_request_state[4];
	u8 pwr;
	u8 fwdl;
	u8 efuse;
	u8 cmd_state;
	u8 pkt_ofld;
	u8 efuse_ofld;
	u8 disable_rf;
	u8 sch_tx_en_ofld;
	u8 fw_rst;
	u8 aoac_rpt;
	u8 p2p_stat;
	u8 dmac_func;
	u8 cmac0_func;
	u8 cmac1_func;
	u8 bb0_func;
	u8 bb1_func;
	u8 wow_stoptrx_stat;
	u8 mac_rdy;
	u8 role_stat;
	u8 plat;
	u8 io_st;
	u8 l2_st;
	u8 ser_ctrl_st;
	u8 ch_switch;
	u8 proxy_st;
	u8 proxy_ret;
	u8 sensing_csi_st;
	u8 nan_stat;
	u8 sta_csa_st;
	u8 sta_csa_ret;
	u8 h2c_c2h_mon;
	u8 h2c_lb_st;
};

#endif
