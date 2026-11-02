/******************************************************************************
 *
 * Copyright(c) 2023 - 2024 Realtek Corporation.
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
#ifndef __RTW_FSM_XXX_H__
#define __RTW_FSM_XXX_H__

#ifdef CONFIG_RTW_FSM_XXX
struct fsm_priv;
struct xxx_obj;

/* Header file for application to invoke xxx service */

struct xxx_priv {
	struct fsm_main *fsm;
	struct xxx_obj *xxx;
};

int rtw_xxx_reg_fsm(struct fsm_priv *fsmpriv);
struct xxx_obj *rtw_xxx_new_obj(_adapter *a, struct sta_info *psta, u16 cid);
void rtw_xxx_abort(struct xxx_priv *pxxxpriv);
#endif /* CONFIG_RTW_FSM_XXX */
#endif /* __RTW_FSM_XXX_H__ */
