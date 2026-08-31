/******************************************************************************
 *
 * Copyright(c) 2013 - 2017 Realtek Corporation.
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
#ifdef CONFIG_BTC

#include <drv_types.h>

enum rtw_btc_esoc_type rtw_btc_update_ext_soc_type(u8 esoc_type)
{
	enum rtw_btc_esoc_type phl_esoc_type = BTC_EXT_SOC_NONE;

	switch (esoc_type) {
	case 0:
		phl_esoc_type = BTC_EXT_SOC_NONE;
		break;
	case 1:
		phl_esoc_type = BTC_EXT_SOC_8761;
		break;
	case 2:
		phl_esoc_type = BTC_EXT_SOC_8771;
		break;
	case 3:
		phl_esoc_type = BTC_EXT_SOC_SILAB_MG21;
		break;
	case 4:
		phl_esoc_type = BTC_EXT_SOC_NORDI_NRF52840;
		break;
	default:
		RTW_WARN("%s: Unsupported esoc_type %d\n", __func__, esoc_type);
		break;
	}

	return phl_esoc_type;
}

#endif /* CONFIG_BTC */

