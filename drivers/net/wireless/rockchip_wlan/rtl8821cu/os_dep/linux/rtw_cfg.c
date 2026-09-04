/******************************************************************************
 *
 * Copyright(c) 2007 - 2023 Realtek Corporation.
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
#define _RTW_CFG_C_

#include <drv_types.h>

uint rtw_edcca_mode_sel = CONFIG_RTW_EDCCA_MODE_SEL;
module_param(rtw_edcca_mode_sel, uint, 0644);
MODULE_PARM_DESC(rtw_edcca_mode_sel, "0:NORMAL, 1:CS, 2:ADPT, 3:CBP, 0xFF:auto");

uint rtw_adaptivity_en = CONFIG_RTW_ADAPTIVITY_EN;
module_param(rtw_adaptivity_en, uint, 0644);
MODULE_PARM_DESC(rtw_adaptivity_en, "0:disable, 1:enable, 2:auto");

uint rtw_adaptivity_mode = CONFIG_RTW_ADAPTIVITY_MODE;
module_param(rtw_adaptivity_mode, uint, 0644);
MODULE_PARM_DESC(rtw_adaptivity_mode, "0:normal, 1:carrier sense");

int rtw_adaptivity_th_l2h_ini = CONFIG_RTW_ADAPTIVITY_TH_L2H_INI;
module_param(rtw_adaptivity_th_l2h_ini, int, 0644);
MODULE_PARM_DESC(rtw_adaptivity_th_l2h_ini, "th_l2h_ini for Adaptivity");

int rtw_adaptivity_th_edcca_hl_diff = CONFIG_RTW_ADAPTIVITY_TH_EDCCA_HL_DIFF;
module_param(rtw_adaptivity_th_edcca_hl_diff, int, 0644);
MODULE_PARM_DESC(rtw_adaptivity_th_edcca_hl_diff, "th_edcca_hl_diff for Adaptivity");

static void rtw_regsty_load_edcca_mode_settings(struct registry_priv *regsty)
{
	regsty->edcca_mode_sel = (u8)rtw_edcca_mode_sel;
	if (regsty->edcca_mode_sel < RTW_EDCCA_MODE_NUM || regsty->edcca_mode_sel == RTW_EDCCA_AUTO) {
		if (regsty->edcca_mode_sel == RTW_EDCCA_NORM) {
			/* consider old interfaces */
			if (rtw_adaptivity_en == RTW_ADAPTIVITY_EN_ENABLE) {
				if (rtw_adaptivity_mode == RTW_ADAPTIVITY_MODE_NORMAL)
					regsty->edcca_mode_sel = RTW_EDCCA_ADAPT;
				else if (rtw_adaptivity_mode == RTW_ADAPTIVITY_MODE_CARRIER_SENSE)
					regsty->edcca_mode_sel = RTW_EDCCA_CS;
			} else if (rtw_adaptivity_en == RTW_ADAPTIVITY_EN_AUTO)
				regsty->edcca_mode_sel = RTW_EDCCA_AUTO;
		}
	} else {
		RTW_WARN("%s invalid rtw_edcca_mode_sel(%u), set to %s\n", __func__
			, regsty->edcca_mode_sel, rtw_edcca_mode_str(RTW_EDCCA_NORM));
		regsty->edcca_mode_sel = RTW_EDCCA_NORM;
	}

	regsty->adaptivity_th_l2h_ini = (s8)rtw_adaptivity_th_l2h_ini;
	regsty->adaptivity_th_edcca_hl_diff = (s8)rtw_adaptivity_th_edcca_hl_diff;
}

uint rtw_load_registry(_adapter *adapter)
{
	uint status = _SUCCESS;
	struct registry_priv  *registry_par = &adapter->registrypriv;

	rtw_regsty_load_edcca_mode_settings(registry_par);

	return status;
}

static void rtw_cfg_edcca_mode_msg(void *sel, _adapter *adapter)
{
	struct registry_priv *regsty = &adapter->registrypriv;

	RTW_PRINT_SEL(sel, "RTW_EDCCA_");
	if (regsty->edcca_mode_sel == RTW_EDCCA_AUTO)
		_RTW_PRINT_SEL(sel, "AUTO\n");
	else
		_RTW_PRINT_SEL(sel, "%s\n", rtw_edcca_mode_str(regsty->edcca_mode_sel));
}

void rtw_cfg_adaptivity_config_msg(void *sel, _adapter *adapter)
{
	rtw_odm_adaptivity_ver_msg(sel, adapter);
	rtw_cfg_edcca_mode_msg(sel, adapter);
}

bool rtw_cfg_adaptivity_needed(_adapter *adapter)
{
	struct registry_priv *regsty = &adapter->registrypriv;

	return regsty->edcca_mode_sel != RTW_EDCCA_NORM;
}

