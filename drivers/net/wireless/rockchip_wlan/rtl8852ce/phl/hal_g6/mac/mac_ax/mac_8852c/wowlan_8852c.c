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
#include "wowlan_8852c.h"

#if MAC_AX_8852C_SUPPORT

u32 get_wake_reason_8852c(struct mac_ax_adapter *adapter, u8 *wowlan_wake_reason)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	*wowlan_wake_reason = MAC_REG_R8(R_AX_DBG_WOW);

	return MACSUCCESS;
}

#endif