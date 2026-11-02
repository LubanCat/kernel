/******************************************************************************
 *
 * Copyright(c) 2007 - 2020  Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 * wlanfae <wlanfae@realtek.com>
 * Realtek Corporation, No. 2, Innovation Road II, Hsinchu Science Park,
 * Hsinchu 300, Taiwan.
 *
 * Larry Finger <Larry.Finger@lwfinger.net>
 *
 *****************************************************************************/
#ifndef __HALBB_DV_PXP_DBG_H__
#define __HALBB_DV_PXP_DBG_H__

#include "../../hal_headers_le.h"

/*@--------------------------[Define] ---------------------------------------*/

/*@--------------------------[Enum]------------------------------------------*/

/*@--------------------------[Structure]-------------------------------------*/
struct bb_dv_pxp_dbg_info {
	bool is_dv_pxp_dbg;
	bool is_print;
};

struct bb_dv_pxp_dbg_cr_info {
	u32 dummy;
};

/*@--------------------------[Prptotype]-------------------------------------*/
struct bb_info;
void halbb_dv_pxp_dbg(struct bb_info *bb, char input[][16], u32 *_used, char *output, u32 *_out_len);
void halbb_dv_pxp_print_en(struct bb_info *bb, bool en);
void halbb_dv_pxp_dbg_init(struct bb_info *bb);
#if 0
void halbb_cr_cfg_dv_dbg_init(struct bb_info *bb);
#endif
#endif
