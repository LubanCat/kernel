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

#ifndef _MAC_AX_PWR_SEQ_FUNC_H_
#define _MAC_AX_PWR_SEQ_FUNC_H_

#include "../mac_def.h"

u32 pwr_poll_u32(struct mac_ax_adapter *adapter, u32 offset,
		 u32 mask, u32 pwr_val);
#endif
