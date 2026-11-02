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
#ifndef _MAC_AX_ERR_FLAG_AUTO_GEN_8852C_H_
#define _MAC_AX_ERR_FLAG_AUTO_GEN_8852C_H_

#include "../../../../mac_def.h"
#include "../../../../mac_ax/dbgpkg.h"

#if MAC_AX_8852C_SUPPORT

// svn reversion
#define ERR_FLAG_REV_8852C 0

u32 err_flag_cmac_8852c(struct mac_ax_adapter *adapter,
			u32 cat, u8 band, struct mac_ax_err_flag_sts *status);

u32 err_flag_dmac_8852c(struct mac_ax_adapter *adapter,
			u32 cat, struct mac_ax_err_flag_sts *status);

u32 err_flag_rst_cmac_8852c(struct mac_ax_adapter *adapter,
			    u32 cat, u8 band);

u32 err_flag_rst_dmac_8852c(struct mac_ax_adapter *adapter,
			    u32 cat);

#endif /* MAC_AX_8852C_SUPPORT */
#endif /* _MAC_AX_ERR_FLAG_AUTO_GEN_8852C_H_ */