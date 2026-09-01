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
#ifndef _MAC_AX_ERR_FLAG_H_
#define _MAC_AX_ERR_FLAG_H_
#include "mac_priv.h"

#if MAC_AX_FEATURE_ERR_FLAG

u32 err_flag_cmac(struct mac_ax_adapter *adapter,
		  char *output, u32 out_len, u32 *used,
		  u32 cat, u8 band, struct mac_ax_err_flag_sts *status);

u32 err_flag_dmac(struct mac_ax_adapter *adapter,
		  char *output, u32 out_len, u32 *used,
		  u32 cat, struct mac_ax_err_flag_sts *status);

u32 err_flag_rst_cmac(struct mac_ax_adapter *adapter,
		      u32 cat, u8 band);

u32 err_flag_rst_dmac(struct mac_ax_adapter *adapter,
		      u32 cat);

u32 err_flag_chk(struct mac_ax_adapter *adapter,
		 char *output, u32 out_len, u32 *used,
		 struct mac_ax_err_flag_sts *status);
#endif /* MAC_AX_FEATURE_ERR_FLAG */
#endif /* _MAC_AX_ERR_FLAG_H_ */