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
#ifndef _MAC_AX_SER_8852B_H_
#define _MAC_AX_SER_8852B_H_

#include "../fwofld.h"
#if MAC_AX_8852B_SUPPORT

#define IMR_IO_OFLD_NON_FLUSH	0

/**
 * @brief ser_imr_config_patch_8852b
 *
 * @param *adapter
 * @param band
 * @param sel
 * @return Please Place Description here.
 * @retval u32
 */
u32 ser_imr_config_patch_8852b(struct mac_ax_adapter *adapter, u8 band,
			       enum mac_ax_hwmod_sel sel);

#endif /* MAC_AX_8852B_SUPPORT  */
#endif
