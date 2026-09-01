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
#ifndef __HALBB_ULMU_MIMO_B_ENDIAN_H__
#define __HALBB_ULMU_MIMO_B_ENDIAN_H__

#ifdef HALBB_ULMU_SUPPORT
#ifdef PHL_FEATURE_AP

/*@--------------------------[Define] ---------------------------------------*/
/*[IO Reg]*/

/*@--------------------------[Enum]------------------------------------------*/



/*@--------------------------[Structure]-------------------------------------*/
struct bb_ulmu_info {
	u8 ss;
	u8 mcs;
	u8 ru_position;
	u8 target_rssi;
};

struct bb_ulmu_init {
	u8 macid_l;
	u8 macid_h;
	u8 entry;
	u8 band;
};

struct bb_ulmu_fix_info {
	u8 fix_mode;
	u8 grp_idx;
	u8 bw;
	u8 giltf;
	u8 band;
	u8 rsvd[3];
	u8 ss[MAX_ULMU_ENTRY];
	u8 mcs[MAX_ULMU_ENTRY];
	u8 mode[MAX_ULMU_ENTRY];
	u8 ru_position[MAX_ULMU_ENTRY];
	u8 target_rssi[MAX_ULMU_ENTRY];
};


/*@--------------------------[Prptotype]-------------------------------------*/

#endif /* PHL_FEATURE_AP */
#endif /* HALBB_ULMU_SUPPORT */

#endif