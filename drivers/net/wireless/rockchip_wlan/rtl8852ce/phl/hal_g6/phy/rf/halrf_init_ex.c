/******************************************************************************
 *
 * Copyright(c) 2019 Realtek Corporation.
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
#include "halrf_precomp.h"

//check ic by hal_i->chip_id

void halrf_rfe_ant_num_chk(void *rf_void)
{
	struct rf_info *rf = (struct rf_info *)rf_void;
	struct rtw_hal_com_t *hal_i = rf->hal_com;

	switch (hal_i->chip_id) {
//	switch (rf->ic_type) {
#ifdef RF_8852B_SUPPORT
	case CHIP_WIFI6_8852B:
		halrf_rfe_ant_num_chk_8852b(rf);
	break;
#endif

#ifdef RF_8852BT_SUPPORT
	case CHIP_WIFI6_8852BT:
		//halrf_rfe_ant_num_chk_8852bt(rf);
	break;
#endif

#ifdef RF_8852BPT_SUPPORT
	case CHIP_WIFI6_8852BPT:
		// halrf_rfe_ant_num_chk_8852bpt(rf);
	break;
#endif

	default:
	break;
	}
}



bool halrf_get_efuse_info(void *rf_void, u8 *efuse_map,
		       enum rtw_efuse_info id, void *value, u32 length,
		       u8 autoload_status)
{
	struct rf_info *rf = (struct rf_info *)rf_void;
	struct rtw_hal_com_t *hal_i = rf->hal_com;
	
#ifdef RF_8852A_SUPPORT
	if (hal_i->chip_id == CHIP_WIFI6_8852A)
		return halrf_get_efuse_info_8852a(rf, efuse_map, id, value, length,
					autoload_status);
#endif

#ifdef RF_8852B_SUPPORT
	if (hal_i->chip_id == CHIP_WIFI6_8852B)
		return halrf_get_efuse_info_8852b(rf, efuse_map, id, value, length,
					autoload_status);
#endif

#ifdef RF_8852BT_SUPPORT
	if (hal_i->chip_id == CHIP_WIFI6_8852BT)
		return halrf_get_efuse_info_8852bt(rf, efuse_map, id, value, length,
					autoload_status);
#endif

#ifdef RF_8852BPT_SUPPORT
	if (hal_i->chip_id == CHIP_WIFI6_8852BPT)
		return halrf_get_efuse_info_8852bpt(rf, efuse_map, id, value, length,
					autoload_status);
#endif

#ifdef RF_8852C_SUPPORT
	if (hal_i->chip_id == CHIP_WIFI6_8852C) //8852C
		return halrf_get_efuse_info_8852c(rf, efuse_map, id, value, length,
					autoload_status);
#endif

#ifdef RF_8842A_SUPPORT
	if (hal_i->chip_id == CHIP_WIFI6_8842A) //8842A 
		return halrf_get_efuse_info_8842a(rf, efuse_map, id, value, length,
					autoload_status);
#endif

#ifdef RF_8852D_SUPPORT
	if (hal_i->chip_id == CHIP_WIFI6_8852D) //52D, 32D define the same
		return halrf_get_efuse_info_8852d(rf, efuse_map, id, value, length,
					autoload_status);
#endif

/*
#ifdef RF_8832D_SUPPORT
	if (hal_i->chip_id == CHIP_WIFI6_8832D)
		return halrf_get_efuse_info_8832d(rf, efuse_map, id, value, length,
					autoload_status);
#endif
*/

#ifdef RF_8832BR_SUPPORT
	if (hal_i->chip_id == CHIP_WIFI6_8832BR)
		return halrf_get_efuse_info_8832br(rf, efuse_map, id, value, length,
					autoload_status);
#endif

#ifdef RF_8192XB_SUPPORT
	if (hal_i->chip_id == CHIP_WIFI6_8192XB)
		return halrf_get_efuse_info_8192xb(rf, efuse_map, id, value, length,
					autoload_status);
#endif

#ifdef RF_8852BP_SUPPORT
	if (hal_i->chip_id == CHIP_WIFI6_8852BP)
		return halrf_get_efuse_info_8852bp(rf, efuse_map, id, value, length,
					autoload_status);
#endif

#ifdef RF_8851B_SUPPORT
	if (hal_i->chip_id == CHIP_WIFI6_8851B)
		return halrf_get_efuse_info_8851b(rf, efuse_map, id, value, length,
					autoload_status);
#endif

	return 0;
}


