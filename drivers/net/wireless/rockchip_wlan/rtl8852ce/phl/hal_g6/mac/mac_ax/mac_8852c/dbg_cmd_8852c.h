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

#ifndef _MAC_AX_DBG_CMD_8852C_H_
#define _MAC_AX_DBG_CMD_8852C_H_

#include "../../type.h"

#if MAC_AX_8852C_SUPPORT

u32 get_check_reg_8852c(u32 *reg_num, struct check_reg_info **check_reg);

#endif

#endif // #define _MAC_AX_DBG_CMD_8852C_H_
