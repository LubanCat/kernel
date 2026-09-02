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

#ifndef _MAC_AX_SER_H_
#define _MAC_AX_SER_H_

#include "../type.h"

#if MAC_AX_SDIO_SUPPORT
#include "_sdio.h"
#endif

#if MAC_AX_PCIE_SUPPORT
#include "_pcie.h"
#endif

#if MAC_AX_USB_SUPPORT
#include "_usb.h"
#endif

#define SEC_DBG_SEL 0x8B
#define SEC_DBG_PORT_NUM 0x10
#define SEC_DBG_PORT_FIELD_MSK 0xf
#define SEC_DBG_PORT_FIELD_SH 16

/*--------------------Define -------------------------------------------*/
#define MAC_SET_ERR_DLY_CNT 200
#define MAC_SET_ERR_DLY_US 50

#define DMAC_ERR_IMR_MASK 0xFFFFFFFF
#define DMAC_ERR_IMR_EN 0xFFFFFFFF
#define CMAC0_ERR_IMR_MASK 0xFFFFFFFF
#define CMAC0_ERR_IMR_EN 0xFFFFFFFF
#define CMAC1_ERR_IMR_MASK 0xFFFFFFFF
#define CMAC1_ERR_IMR_EN 0xFFFFFFFF
#define DMAC_ERR_IMR_DIS 0
#define CMAC0_ERR_IMR_DIS 0
#define CMAC1_ERR_IMR_DIS 0

#define FW_ST_MSK 0xFFFF
#define FW_ST_SH 8
#define FW_ST_ERR_IN 0x11

#define MAC_SER_STOP_DLY_CNT 200
#define MAC_SER_STOP_DLY_US 50

#define CMAC_AX_COMMON_BASE_ADDR 0xC000
#define CMAC_AX_COMMON_BASE_ADDR_C1 0xE000
#define CMAC_AX_COMMON_MAX_ADDR 0x1FC
#define CMAC_AX_SCH_BASE_ADDR 0xC200
#define CMAC_AX_SCH_BASE_ADDR_C1 0xE200
#define CMAC_AX_SCH_MAX_ADDR 0x3FC
#define CMAC_AX_PTCL_BASE_ADDR 0xC600
#define CMAC_AX_PTCL_BASE_ADDR_C1 0xE600
#define CMAC_AX_PTCL_MAX_ADDR 0x1FC
#define CMAC_AX_CDMA_BASE_ADDR 0xC800
#define CMAC_AX_CDMA_BASE_ADDR_C1 0xE800
#define CMAC_AX_CDMA_MAX_ADDR 0x2FC
#define CMAC_AX_TMAC_BASE_ADDR 0xCA00
#define CMAC_AX_TMAC_BASE_ADDR_C1 0xEA00
#define CMAC_AX_TMAC_MAX_ADDR 0xFC
#define CMAC_AX_TRXPTCL_BASE_ADDR 0xCC00
#define CMAC_AX_TRXPTCL_BASE_ADDR_C1 0xEC00
#define CMAC_AX_TRXPTCL_MAX_ADDR 0x1FC
#define CMAC_AX_RMAC_BASE_ADDR 0xCE00
#define CMAC_AX_RMAC_BASE_ADDR_C1 0xEE00
#define CMAC_AX_RMAC_MAX_ADDR 0x1FC
#define CMAC_AX_PWR_BASE_ADDR 0xD200
#define CMAC_AX_PWR_BASE_ADDR_C1 0xF200
#define CMAC_AX_PWR_MAX_ADDR 0x7FC
#define CMAC_AX_BTCOEX_BASE_ADDR 0xDA00
#define CMAC_AX_BTCOEX_BASE_ADDR_C1 0xFA00
#define CMAC_AX_BTCOEX_MAX_ADDR 160

#define DMAC_AX_TOP_OFF_BASE_ADDR 0x7000
#define DMAC_AX_TOP_OFF_MAX_ADDR 0x2FC
#define DMAC_AX_WL_PON_BASE_ADDR 0x7800
#define DMAC_AX_WL_PON_MAX_ADDR 0x154
#define DMAC_AX_COMMON_BASE_ADDR 0x8400
#define DMAC_AX_COMMON_MAX_ADDR 0x3FC
#define DMAC_AX_DISPATCHER_BASE_ADDR 0x8800
#define DMAC_AX_DISPATCHER_MAX_ADDR 0x15C
#define DMAC_AX_WDE_BASE_ADDR 0x8C00
#define DMAC_AX_WDE_MAX_ADDR 0x18C
#define DMAC_AX_PLE_BASE_ADDR 0x9000
#define DMAC_AX_PLE_MAX_ADDR 0x13C
#define DMAC_AX_WDRLS_BASE_ADDR 0x9400
#define DMAC_AX_WDRLS_MAX_ADDR 0x7C
#define DMAC_AX_BBRPT_BASE_ADDR 0x9600
#define DMAC_AX_BBRPT_MAX_ADDR 0x8C
#define DMAC_AX_CPUIO_BASE_ADDR 0x9800
#define DMAC_AX_CPUIO_MAX_ADDR 0x8C
#define DMAC_AX_PKTIN_BASE_ADDR 0x9A00
#define DMAC_AX_PKTIN_MAX_ADDR 0x48
#define DMAC_AX_MPDU_BASE_ADDR 0x9B00
#define DMAC_AX_MPDU_MAX_ADDR 0x1FC
#define DMAC_AX_SEC_BASE_ADDR 0x9D00
#define DMAC_AX_SEC_MAX_ADDR 0xFC
#define DMAC_AX_SS_BASE_ADDR 0x9E00
#define DMAC_AX_SS_MAX_ADDR 0xFC
#define DMAC_AX_TXPKTCTL_BASE_ADDR 0x9F00
#define DMAC_AX_TXPKTCTL_MAX_ADDR 0xFC
#define DMAC_AX_HCI_BASE_ADDR 0x6000
#define DMAC_AX_HCI_MAX_ADDR 0xFC
#define DMAC_BE_HAXIDMA_BASE_ADDR 0x1000
#define DMAC_BE_HAXIDMA_MAX_ADDR 0x7FC
/*--------------------Define Enum---------------------------------------*/
enum WCPU_ERR_SCENARIO {
	RXI300_ERROR = 1,
	CPU_EXCEPTION  = 2,
	ASSERTION  = 3,
	WDT_ALARM = 4,
};

/*--------------------Define MACRO--------------------------------------*/
/*--------------------Define Struct-------------------------------------*/
/*--------------------Function declaration------------------------------*/
u32 mac_trigger_cmac_err(struct mac_ax_adapter *adapter);
u32 mac_trigger_cmac1_err(struct mac_ax_adapter *adapter);
u32 mac_trigger_dmac_err(struct mac_ax_adapter *adapter);

u32 mac_dump_err_status(struct mac_ax_adapter *adapter,
			enum mac_ax_err_info err);
u32 mac_set_err_status(struct mac_ax_adapter *adapter,
		       enum mac_ax_err_info err);
u32 mac_get_err_status(struct mac_ax_adapter *adapter,
		       enum mac_ax_err_info *err);

u32 mac_lv1_rcvy(struct mac_ax_adapter *adapter,
		 enum mac_ax_lv1_rcvy_step step);

u32 mac_err_imr_ctrl(struct mac_ax_adapter *adapter, enum mac_ax_func_sw sw);
u32 mac_ser_ctrl(struct mac_ax_adapter *adapter, enum mac_ax_func_sw sw);
u32 mac_chk_err_status(struct mac_ax_adapter *adapter, u8 *ser_status);
u32 mac_dbg_log_dump(struct mac_ax_adapter *adapter);
u32 mac_dbg_log_lvl_adjust(struct mac_ax_adapter *adapter, struct mac_debug_log_lvl *lvl);
u32 mac_dump_ser_cnt(struct mac_ax_adapter *adapter, struct mac_ser_status *status);
u32 set_l2_status(struct mac_ax_adapter *adapter);
u32 get_ser_freq(struct mac_ax_adapter *adapter, u32 *total_check_num);
u32 ser_imr_config_patch(struct mac_ax_adapter *adapter, u8 band,
			 enum mac_ax_hwmod_sel sel);
u32 mac_set_ser_case(struct mac_ax_adapter *adapter,
		     struct mac_ser_dbg_info *ser_dbg_info);
#endif
