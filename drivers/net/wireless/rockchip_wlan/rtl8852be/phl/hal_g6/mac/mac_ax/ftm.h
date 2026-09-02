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

#ifndef _MAC_AX_FTM_H_
#define _MAC_AX_FTM_H_

#include "../mac_def.h"
#include "role.h"
#include "fwcmd.h"
#include "addr_cam.h"
#if MAC_FEAT_FTM

/*--------------------Define ----------------------------------------*/
#define FWCMD_H2C_FUNC_SECCAM_FTM 0x2
#define FWCMD_H2C_FUNC_SECCAM_FTM_CANCEL 0x3
#define FWCMD_H2C_FUNC_SECCAM_FTM_UPD 0x4

/*--------------------DSecurity cam type declaration-----------------*/
struct fwcmd_ftm_upd_info {
	u32 dword0;
	u32 dword1;
	u32 dword2;
	u32 dword3;
};

struct fwcmd_ftm_cancel_info {
	u32 dword0;
};

struct fwcmd_ftm_en_info {
	u32 dword0;
	u32 dword1;
	u32 dword2;
};

struct fwcmd_ftm_info {
	u32 dword0;
	u32 dword1;
	u32 dword2;
	u32 dword3;
	u32 dword4;
	u32 dword5;
	u32 dword6;
	u32 dword7;
	u32 dword8;
	u32 dword9;
};
/*--------------------Funciton declaration----------------------------*/

/**
 * @brief mac_sta_add_key
 *
 * @param *adapter
 * @param *sec_cam_content
 * @param mac_id
 * @param key_id
 * @param key_type
 * @return Please Place Description here.
 * @retval u32
 */

u32 mac_ista_ftm_proc(struct mac_ax_adapter *adapter,
		      struct mac_ax_ftm_para *ftmr);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Basic_TRX
 * @{
 * @addtogroup Security
 * @{
 */

u32 mac_ista_ftm_enable(struct mac_ax_adapter *adapter,
			u8 macid, bool enable);

u32 mac_ista_ftm_cancel(struct mac_ax_adapter *adapter,
			struct mac_ax_ftm_cancel_para *ftm_cancel);

u32 mac_ista_ftm_upd_para(struct mac_ax_adapter *adapter,
			  struct mac_ax_ftm_upd_para *ftm_upd_para);

#endif
#endif
