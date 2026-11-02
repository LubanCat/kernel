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
#include "err_flag_auto_gen_8852c.h"
#include "../../../../mac_ax/mac_priv.h"

#if MAC_AX_8852C_SUPPORT
u32 err_flag_cmac_8852c(struct mac_ax_adapter *adapter,
			u32 cat, u8 band, struct mac_ax_err_flag_sts *status)
{
	return MACSUCCESS;
}

u32 err_flag_dmac_8852c(struct mac_ax_adapter *adapter,
			u32 cat, struct mac_ax_err_flag_sts *status)
{
	return MACSUCCESS;
}

u32 err_flag_rst_cmac_8852c(struct mac_ax_adapter *adapter,
			    u32 cat, u8 band)
{
	return MACSUCCESS;
}

u32 err_flag_rst_dmac_8852c(struct mac_ax_adapter *adapter,
			    u32 cat)
{
	return MACSUCCESS;
}
#endif /* MAC_AX_8852C_SUPPORT */
