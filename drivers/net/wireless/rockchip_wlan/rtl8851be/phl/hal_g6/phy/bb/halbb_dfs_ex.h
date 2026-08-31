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
#ifndef __HALBB_DFS_EX_H__
#define __HALBB_DFS_EX_H__
/*@--------------------------[Define] ---------------------------------------*/
/*@--------------------------[Enum]------------------------------------------*/
enum halbb_radar_detect_t {
	RDR_EXTRA_SUB20_7		 = BIT(15),
	RDR_EXTRA_SUB20_6		 = BIT(14),
	RDR_EXTRA_SUB20_5		 = BIT(13),
	RDR_EXTRA_SUB20_4		 = BIT(12),
	RDR_EXTRA_SUB20_3		 = BIT(11),
	RDR_EXTRA_SUB20_2		 = BIT(10),
	RDR_EXTRA_SUB20_1		 = BIT(9),
	RDR_EXTRA_SUB20_0		 = BIT(8),
	RDR_SUB20_7		 	 = BIT(7),
	RDR_SUB20_6			 = BIT(6),
	RDR_SUB20_5			 = BIT(5),
	RDR_SUB20_4		 	 = BIT(4),
	RDR_SUB20_3			 = BIT(3),
	RDR_SUB20_2			 = BIT(2),
	RDR_SUB20_1		 	 = BIT(1),
	RDR_SUB20_0			 = BIT(0),
	RDR_EXTRA_FULL			 = 0xff00,
	RDR_FULL			 = 0x00ff
};
/*@--------------------------[Structure]-------------------------------------*/
struct bb_info;
struct hal_dfs_rpt;
/*@--------------------------[Prptotype]-------------------------------------*/
void halbb_dfs_init(struct bb_info *bb);
void halbb_radar_detect_reset(struct bb_info *bb);
void halbb_radar_detect_disable(struct bb_info *bb);
void halbb_radar_detect_enable(struct bb_info *bb);
void halbb_dfs_enable_cac_flag(struct bb_info* bb);
void halbb_dfs_disable_cac_flag(struct bb_info* bb);
void halbb_dfs_change_dmn(struct bb_info *bb, u8 ch, u8 bw);
bool halbb_is_dfs_band(struct bb_info *bb, u8 ch, u8 bw);
u16 halbb_radar_detect(struct bb_info *bb, struct hal_dfs_rpt *rpt);
#endif