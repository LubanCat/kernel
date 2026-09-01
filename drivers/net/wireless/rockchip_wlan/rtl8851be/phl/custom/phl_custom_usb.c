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
#define _PHL_CUSTOMIZE_FEATURE_USB_C_

#ifdef CONFIG_PHL_CUSTOM_FEATURE_USB
#include "../phl_headers.h"
#include "phl_custom_usb.h"

enum rtw_phl_status phl_cmd_set_usb_support_ability(struct phl_info_t *phl_info,
						u32 ability)
{
	if (rtw_hal_set_usb_support_ability(phl_info->hal, ability) == RTW_HAL_STATUS_SUCCESS) {
		PHL_INFO("%s success \n", __func__);
		return RTW_PHL_STATUS_SUCCESS;
	} else {
		return RTW_PHL_STATUS_FAILURE;
	}
}

enum rtw_phl_status
rtw_phl_custom_usb_set_support_ability(void *phl, u32 ability,
				    enum phl_band_idx band_idx,
				    enum phl_cmd_type cmd_type, u32 cmd_timeout)
{
	struct phl_info_t *phl_info = (struct phl_info_t *)phl;
	enum rtw_phl_status psts = RTW_PHL_STATUS_FAILURE;

#ifdef CONFIG_CMD_DISP
	if (cmd_type == PHL_CMD_DIRECTLY) {
		psts = phl_cmd_set_usb_support_ability(phl_info, ability);
	} else {
		psts = phl_cmd_enqueue(phl_info,
					band_idx,
					MSG_EVT_SET_USB_SW_ABILITY,
					(u8*)&ability,
					sizeof(u32),
					NULL,
					cmd_type,
					cmd_timeout);
	}

	if (is_cmd_failure(psts))
		/* Send cmd success, but wait cmd fail*/
		psts = RTW_PHL_STATUS_FAILURE;
	else if (psts != RTW_PHL_STATUS_SUCCESS)
		/* Send cmd fail */
		psts = RTW_PHL_STATUS_FAILURE;
#else
	psts = phl_cmd_set_usb_support_ability(phl_info, ability);
#endif
	return psts;
}
#endif
