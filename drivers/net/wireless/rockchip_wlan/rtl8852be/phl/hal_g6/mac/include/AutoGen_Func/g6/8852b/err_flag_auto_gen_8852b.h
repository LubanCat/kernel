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
#ifndef _MAC_AX_ERR_FLAG_AUTO_GEN_8852B_H_
#define _MAC_AX_ERR_FLAG_AUTO_GEN_8852B_H_

#include "../../../../mac_def.h"
#include "../../../../mac_ax/dbgpkg.h"

#if MAC_AX_FEATURE_ERR_FLAG
#if MAC_AX_8852B_SUPPORT

// svn reversion
#define ERR_FLAG_REV_8852B 0

u32 err_flag_cmac_8852b(struct mac_ax_adapter *adapter,
			char *output, u32 out_len, u32 *used,
			u32 cat, u8 band, struct mac_ax_err_flag_sts *status);

u32 err_flag_dmac_8852b(struct mac_ax_adapter *adapter,
			char *output, u32 out_len, u32 *used,
			u32 cat, struct mac_ax_err_flag_sts *status);

u32 err_flag_rst_cmac_8852b(struct mac_ax_adapter *adapter,
			    u32 cat, u8 band);

u32 err_flag_rst_dmac_8852b(struct mac_ax_adapter *adapter,
			    u32 cat);

#endif /* MAC_AX_8852B_SUPPORT */
#endif /* MAC_AX_FEATURE_ERR_FLAG */
#endif /* _MAC_AX_ERR_FLAG_AUTO_GEN_8852B_H_ */