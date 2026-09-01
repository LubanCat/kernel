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

#include "mac_priv.h"
#if MAC_AX_8192XB_SUPPORT
#include "mac_8192xb/mac_priv_8192xb.h"
#endif
#if MAC_AX_8852A_SUPPORT
#include "mac_8852a/mac_priv_8852a.h"
#endif
#if MAC_AX_8852B_SUPPORT
#include "mac_8852b/mac_priv_8852b.h"
#endif
#if MAC_AX_8852C_SUPPORT
#include "mac_8852c/mac_priv_8852c.h"
#endif
#if MAC_AX_8851B_SUPPORT
#include "mac_8851b/mac_priv_8851b.h"
#endif
#if MAC_AX_8852D_SUPPORT
#include "mac_8852d/mac_priv_8852d.h"
#endif
#if MAC_AX_8852BT_SUPPORT
#include "mac_8852bt/mac_priv_8852bt.h"
#endif

u32 get_mac_ax_priv_size(void)
{
	return sizeof(struct mac_ax_priv_ops *);
}
