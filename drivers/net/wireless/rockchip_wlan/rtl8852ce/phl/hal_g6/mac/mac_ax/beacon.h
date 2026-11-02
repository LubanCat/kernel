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

#ifndef _MAC_AX_BEACON_H_
#define _MAC_AX_BEACON_H_

#include "../type.h"

/*--------------------Define -------------------------------------------*/
#define IECAM_INFO_SIZE 8 /* dword1 & dword2 is not used */

#define BCN_GRPIE_OFST_EN BIT(7)
#define MAX_BCN_PLD_SIZE 700
/*--------------------Define MACRO--------------------------------------*/
/*--------------------Define Enum---------------------------------------*/
/*--------------------Define Struct-------------------------------------*/
/*--------------------Function Prototype--------------------------------*/

/**
 * @brief mac_calc_crc
 *
 * @param *adapter
 * @param *info
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_calc_crc(struct mac_ax_adapter *adapter, struct mac_calc_crc_info *info);

/**
 * @brief mac_bcn_ofld_ctrl
 *
 * @param *adapter
 * @param *info
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_bcn_ofld_ctrl(struct mac_ax_adapter *adapter, struct mac_bcn_ofld_info *info);

/**
 * @brief mac_ie_cam_upd
 *
 * @param *adapter
 * @param *info
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_ie_cam_upd(struct mac_ax_adapter *adapter, struct mac_ax_ie_cam_cmd_info *info);

/**
 * @brief mac_set_bcn_ignore_edcca
 *
 * @param *adapter
 * @param *bcn_ignore_edcca
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_set_bcn_ignore_edcca(struct mac_ax_adapter *adapter,
			     struct mac_ax_bcn_ignore_edcca *bcn_ignore_edcca);

u32 mac_send_bcn_h2c(struct mac_ax_adapter *adapter,
		     struct mac_ax_bcn_info *info);

u32 mac_set_bcn_dynamic_mech(struct mac_ax_adapter *adapter,
			     struct mac_ax_bcn_dynamic_mech *bcn_dynamic_mech);

#endif
