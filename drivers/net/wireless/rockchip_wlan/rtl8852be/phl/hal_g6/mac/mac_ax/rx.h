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

#ifndef _MAC_AX_RX_H_
#define _MAC_AX_RX_H_

#include "../type.h"

/**
 * @brief set_resp_stat_rts_chk
 *
 * @param *adapter
 * @param mac_ax_set_resp_stat_rts_chk_cfg cfg
 * @return HW setting status
 * @retval u32
 */
u32 set_resp_stat_rts_chk(struct mac_ax_adapter *adapter,
			  struct mac_ax_set_resp_stat_rts_chk_cfg *cfg);

/**
 * @brief set_bacam_mode
 *
 * @param *adapter
 * @param mode_sel
 * @return Set the R_AX_RESPBA_CAM_CTRL bit 4 to be 0 or 1 which decide the
 * option mode in BA CAM.
 * @retval u32
 */
u32 set_bacam_mode(struct mac_ax_adapter *adapter, u8 mode_sel);
/**
 * @}
 * @}
 */

/**
 * @brief scheduler_set_prebkf
 *
 * @param *adapter
 * @param *mode_sel
 * @return Get the option mode from R_AX_RESPBA_CAM_CTRL bit 4 in the BA CAM.
 * @retval u32
 */
u32 get_bacam_mode(struct mac_ax_adapter *adapter, u8 *mode_sel);
/**
 * @}
 * @}
 */

u32 set_rxd_zld_en(struct mac_ax_adapter *adapter, u8 enable);

#endif