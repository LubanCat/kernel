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

#ifndef _MAC_AX_PHY_RPT_H_
#define _MAC_AX_PHY_RPT_H_

#include "../type.h"
#include "role.h"
#include "trxcfg.h"

#define MAC_MAX_4_USR	4
#define MAC_MAX_8_USR	8
#define MAC_AX_RX_CNT_SIZE 96
#define MAC_AX_DFS_HDR_SIZE 8
#define MAC_AX_DFS_RPT_SIZE 8
#define MAC_AX_DFS_RPT_SIZE_SH 3
#define MAC_AX_CH_INFO_BUF 0
#define B_AX_CH_INFO_BUF_128 0
#define B_AX_GET_CH_INFO_TO_DIS 0
#define B_AX_GET_CH_INFO_TO_8 2
#define B_AX_GET_CH_INFO_TO_28 7
#define B_AX_CH_INFO_INTVL_DIS 0
#define B_AX_CH_INFO_INTVL_1 1
#define B_AX_CH_INFO_INTVL_2 2
#define B_AX_CH_INFO_INTVL_4 4
#define B_AX_CH_INFO_INTVL_7 7
#define B_AX_CH_INFO_REQ_2 1
#define B_AX_DFS_BUF_64 1

#define MAC_AX_MAC_INFO_USE_SIZE 4
#define MAC_AX_PHY_RPT_CNT 3000
#define MAC_CHINFO_STOP_REQ_TIMEOUT	7000

struct mac_ax_mac_info_t {
	u32 dword0;
	u32 dword1;
};

struct mac_ax_dfs_hdr_t {
	u32 dword0;
	u32 dword1;
};

/**
 * @addtogroup Common
 * @{
 * @addtogroup BB_Related
 * @{
 */

/**
 * @brief mac_cfg_phy_rpt
 *
 * @param *adapter
 * @param *cfg
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_cfg_phy_rpt(struct mac_ax_adapter *adapter,
		    struct mac_ax_phy_rpt_cfg *cfg);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Common
 * @{
 * @addtogroup BB_Related
 * @{
 */

/**
 * @brief mac_parse_ppdu
 *
 * @param *adapter
 * @param *buf
 * @param ppdu_len
 * @param mac_info
 * @param *rpt
 * @return Please Place Description here.
 * @retval u32
 */

u32 mac_parse_ppdu(struct mac_ax_adapter *adapter,
		   u8 *buf, u32 ppdu_len, u8 mac_info,
		   struct mac_ax_ppdu_rpt *rpt);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Common
 * @{
 * @addtogroup BB_Related
 * @{
 */

/**
 * @brief mac_parse_dfs
 *
 * @param *adapter
 * @param *buf
 * @param dfs_len
 * @param *rpt
 * @return Please Place Description here.
 * @retval u32
 */

u32 mac_parse_dfs(struct mac_ax_adapter *adapter,
		  u8 *buf, u32 dfs_len, struct mac_ax_dfs_rpt *rpt);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Common
 * @{
 * @addtogroup BB_Related
 * @{
 */

/**
 * @brief mac_get_phy_rpt_cfg
 *
 * @param *adapter
 * @param *rpt
 * @return Please Place Description here.
 * @retval u32
 */

u32 mac_get_phy_rpt_cfg(struct mac_ax_adapter *adapter,
			struct mac_ax_phy_rpt_cfg *rpt);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Common
 * @{
 * @addtogroup BB_Related
 * @{
 */

/**
 * @brief mac_rst_drv_info
 *
 * @param *adapter
 * @return Please Place Description here.
 * @retval u32
 */

u32 mac_rst_drv_info(struct mac_ax_adapter *adapter);
/**
 * @}
 * @}
 */
#endif
