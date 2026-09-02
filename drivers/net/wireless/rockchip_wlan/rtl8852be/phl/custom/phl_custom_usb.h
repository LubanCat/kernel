/******************************************************************************
 *
 * Copyright(c) 2024 Realtek Corporation.
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
 *****************************************************************************/
#ifndef _PHL_CUSTOM_USB_H_
#define _PHL_CUSTOM_USB_H_

enum rtw_phl_status
rtw_phl_custom_usb_set_support_ability(void *phl, u32 ability,
				    enum phl_band_idx band_idx,
				    enum phl_cmd_type cmd_type, u32 cmd_timeout);
#endif
