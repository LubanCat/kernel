/******************************************************************************
 *
 * Copyright(c) 2007 - 2022 Realtek Corporation.
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
#define _RTW_REGDB_RTK_C_

#include <drv_types.h>
#include "rtw_regdb_rtk_common.h"

#define RTW_MODULE_NAME		""
#define RTW_DOMAIN_MAP_VER	"63"
#define RTW_DOMAIN_MAP_M_VER	""
#define RTW_COUNTRY_MAP_VER	"39"
#define RTW_COUNTRY_MAP_M_VER	""

enum rtw_chd_2g {
	RTW_CHD_2G_INVALID = 0,

	RTW_CHD_2G_00,
	RTW_CHD_2G_01,
	RTW_CHD_2G_02,
	RTW_CHD_2G_03,
	RTW_CHD_2G_04,
	RTW_CHD_2G_05,
	RTW_CHD_2G_06,

	RTW_CHD_2G_MAX,
	RTW_CHD_2G_NULL = RTW_CHD_2G_00,
};

enum rtw_chd_5g {
	RTW_CHD_5G_INVALID = 0,

	RTW_CHD_5G_00,
	RTW_CHD_5G_01,
	RTW_CHD_5G_02,
	RTW_CHD_5G_03,
	RTW_CHD_5G_04,
	RTW_CHD_5G_05,
	RTW_CHD_5G_06,
	RTW_CHD_5G_07,
	RTW_CHD_5G_08,
	RTW_CHD_5G_09,
	RTW_CHD_5G_10,
	RTW_CHD_5G_11,
	RTW_CHD_5G_12,
	RTW_CHD_5G_13,
	RTW_CHD_5G_14,
	RTW_CHD_5G_15,
	RTW_CHD_5G_16,
	RTW_CHD_5G_17,
	RTW_CHD_5G_18,
	RTW_CHD_5G_19,
	RTW_CHD_5G_20,
	RTW_CHD_5G_21,
	RTW_CHD_5G_22,
	RTW_CHD_5G_23,
	RTW_CHD_5G_24,
	RTW_CHD_5G_25,
	RTW_CHD_5G_26,
	RTW_CHD_5G_27,
	RTW_CHD_5G_28,
	RTW_CHD_5G_29,
	RTW_CHD_5G_30,
	RTW_CHD_5G_31,
	RTW_CHD_5G_32,
	RTW_CHD_5G_33,
	RTW_CHD_5G_34,
	RTW_CHD_5G_35,
	RTW_CHD_5G_36,
	RTW_CHD_5G_37,
	RTW_CHD_5G_38,
	RTW_CHD_5G_39,
	RTW_CHD_5G_40,
	RTW_CHD_5G_41,
	RTW_CHD_5G_42,
	RTW_CHD_5G_43,
	RTW_CHD_5G_44,
	RTW_CHD_5G_45,
	RTW_CHD_5G_46,
	RTW_CHD_5G_47,
	RTW_CHD_5G_48,
	RTW_CHD_5G_49,
	RTW_CHD_5G_50,
	RTW_CHD_5G_51,
	RTW_CHD_5G_52,
	RTW_CHD_5G_53,
	RTW_CHD_5G_54,
	RTW_CHD_5G_55,

	RTW_CHD_5G_MAX,
	RTW_CHD_5G_NULL = RTW_CHD_5G_00,
};

static const CH_LIST_2G_T rtw_channel_def_2g[] = {
	/* RTW_CHD_2G_INVALID */	CH_LIST_ENT_2G(0),
	/* RTW_CHD_2G_00 */	CH_LIST_ENT_2G(0),
	/* RTW_CHD_2G_01 */	CH_LIST_ENT_2G(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, CLA_2G_12_14_PASSIVE),
	/* RTW_CHD_2G_02 */	CH_LIST_ENT_2G(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0),
	/* RTW_CHD_2G_03 */	CH_LIST_ENT_2G(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0),
	/* RTW_CHD_2G_04 */	CH_LIST_ENT_2G(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0),
	/* RTW_CHD_2G_05 */	CH_LIST_ENT_2G(10, 11, 12, 13, 0),
	/* RTW_CHD_2G_06 */	CH_LIST_ENT_2G(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, CLA_2G_12_14_PASSIVE),
};

#if CONFIG_IEEE80211_BAND_5GHZ
static const CH_LIST_5G_T rtw_channel_def_5g[] = {
	/* RTW_CHD_5G_INVALID */	CH_LIST_ENT_5G(0),
	/* RTW_CHD_5G_00 */	CH_LIST_ENT_5G(0),
	/* RTW_CHD_5G_01 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_02 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_03 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_04 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 149, 153, 157, 161, 165, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_05 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 149, 153, 157, 161, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_06 */	CH_LIST_ENT_5G(36, 40, 44, 48, 149, 153, 157, 161, 165, 0),
	/* RTW_CHD_5G_07 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 149, 153, 157, 161, 165, CLA_5G_B2_DFS),
	/* RTW_CHD_5G_08 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 149, 153, 157, 161, CLA_5G_B2_DFS),
	/* RTW_CHD_5G_09 */	CH_LIST_ENT_5G(149, 153, 157, 161, 165, 0),
	/* RTW_CHD_5G_10 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, CLA_5G_B2_DFS),
	/* RTW_CHD_5G_11 */	CH_LIST_ENT_5G(100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, CLA_5G_B3_DFS),
	/* RTW_CHD_5G_12 */	CH_LIST_ENT_5G(56, 60, 64, 100, 104, 108, 112, 116, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_13 */	CH_LIST_ENT_5G(56, 60, 64, 149, 153, 157, 161, 165, CLA_5G_B2_DFS),
	/* RTW_CHD_5G_14 */	CH_LIST_ENT_5G(36, 40, 44, 48, 0),
	/* RTW_CHD_5G_15 */	CH_LIST_ENT_5G(149, 153, 157, 161, 0),
	/* RTW_CHD_5G_16 */	CH_LIST_ENT_5G(100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 0),
	/* RTW_CHD_5G_17 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_18 */	CH_LIST_ENT_5G(52, 56, 60, 64, 100, 104, 108, 112, 116, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_19 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 132, 136, 140, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_20 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 132, 136, 140, 149, 153, 157, 161, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_21 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 132, 136, 140, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_22 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_23 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 149, 153, 157, 161, 165, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_24 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B2_PASSIVE | CLA_5G_B3_PASSIVE | CLA_5G_B4_PASSIVE),
	/* RTW_CHD_5G_25 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B2_PASSIVE | CLA_5G_B3_PASSIVE),
	/* RTW_CHD_5G_26 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B1_PASSIVE | CLA_5G_B2_PASSIVE | CLA_5G_B3_PASSIVE),
	/* RTW_CHD_5G_27 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B2_PASSIVE | CLA_5G_B3_PASSIVE),
	/* RTW_CHD_5G_28 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 149, 153, 157, 161, 165, CLA_5G_B2_PASSIVE),
	/* RTW_CHD_5G_29 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 149, 153, 157, 161, 165, CLA_5G_B1_PASSIVE | CLA_5G_B2_PASSIVE),
	/* RTW_CHD_5G_30 */	CH_LIST_ENT_5G(36, 40, 44, 48, 149, 153, 157, 161, 165, CLA_5G_B1_PASSIVE | CLA_5G_B4_PASSIVE),
	/* RTW_CHD_5G_31 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B1_PASSIVE | CLA_5G_B2_PASSIVE | CLA_5G_B3_PASSIVE | CLA_5G_B4_PASSIVE),
	/* RTW_CHD_5G_32 */	CH_LIST_ENT_5G(52, 56, 60, 64, 149, 153, 157, 161, 165, CLA_5G_B2_DFS),
	/* RTW_CHD_5G_33 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 132, 136, 140, 144, 149, 153, 157, 161, 165, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_34 */	CH_LIST_ENT_5G(100, 104, 108, 112, 116, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B3_DFS),
	/* RTW_CHD_5G_35 */	CH_LIST_ENT_5G(100, 104, 108, 112, 116, 132, 136, 140, CLA_5G_B3_DFS),
	/* RTW_CHD_5G_36 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165, CLA_5G_B1_PASSIVE | CLA_5G_B2_PASSIVE | CLA_5G_B2_DFS | CLA_5G_B3_PASSIVE | CLA_5G_B3_DFS | CLA_5G_B4_PASSIVE),
	/* RTW_CHD_5G_37 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, CLA_5G_B1_PASSIVE | CLA_5G_B2_PASSIVE),
	/* RTW_CHD_5G_38 */	CH_LIST_ENT_5G(52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_39 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 149, 153, 157, 161, 165, CLA_5G_B1_PASSIVE | CLA_5G_B2_DFS | CLA_5G_B3_DFS | CLA_5G_B4_DFS),
	/* RTW_CHD_5G_40 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B1_PASSIVE | CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_41 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B2_DFS | CLA_5G_B3_DFS | CLA_5G_B4_PASSIVE),
	/* RTW_CHD_5G_42 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B1_PASSIVE | CLA_5G_B2_DFS | CLA_5G_B3_DFS | CLA_5G_B4_PASSIVE),
	/* RTW_CHD_5G_43 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B1_PASSIVE | CLA_5G_B2_PASSIVE | CLA_5G_B3_PASSIVE | CLA_5G_B4_PASSIVE),
	/* RTW_CHD_5G_44 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 132, 136, 140, 149, 153, 157, 161, 165, CLA_5G_B1_PASSIVE | CLA_5G_B2_PASSIVE | CLA_5G_B3_PASSIVE | CLA_5G_B4_PASSIVE),
	/* RTW_CHD_5G_45 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 149, 153, 157, 161, 165, CLA_5G_B1_PASSIVE | CLA_5G_B2_PASSIVE | CLA_5G_B4_PASSIVE),
	/* RTW_CHD_5G_46 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 149, 153, 157, 161, CLA_5G_B2_PASSIVE),
	/* RTW_CHD_5G_47 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, CLA_5G_B2_PASSIVE | CLA_5G_B3_PASSIVE),
	/* RTW_CHD_5G_48 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_49 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_50 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 132, 136, 140, 144, 149, 153, 157, 161, 165, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_51 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_52 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165, 169, 173, 177, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_53 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 149, 153, 157, 161, 165, CLA_5G_B2_DFS | CLA_5G_B3_DFS),
	/* RTW_CHD_5G_54 */	CH_LIST_ENT_5G(36, 40, 44, 48, 149, 153, 157, 161, 0),
	/* RTW_CHD_5G_55 */	CH_LIST_ENT_5G(36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165, 169, 173, 177, CLA_5G_B1_PASSIVE | CLA_5G_B2_PASSIVE | CLA_5G_B2_DFS | CLA_5G_B3_PASSIVE | CLA_5G_B3_DFS | CLA_5G_B4_PASSIVE),
};
#endif /* CONFIG_IEEE80211_BAND_5GHZ */

static const struct chplan_ent_t RTW_ChannelPlanMap[] = {
	[0x00] = CHPLAN_ENT(ETSI , 02, ETSI , 49),
	[0x01] = CHPLAN_ENT(ETSI , 02, ETSI , 50),
	[0x02] = CHPLAN_ENT(ETSI , 03, ETSI , 07),
	[0x03] = CHPLAN_ENT(ACMA , 02, ACMA , 33),
	[0x04] = CHPLAN_ENT(ETSI , 02, ETSI , 51),
	[0x05] = CHPLAN_ENT(ETSI , 02, ETSI , 06),
	[0x06] = CHPLAN_ENT(ETSI , 02, ETSI , 07),
	[0x07] = CHPLAN_ENT(ETSI , 02, ETSI , 23),
	[0x08] = CHPLAN_ENT(ETSI , 02, ETSI , 21),
	[0x09] = CHPLAN_ENT(ETSI , 02, ETSI , 17),
	[0x0A] = CHPLAN_ENT(NA   , 00, NA   , 00),
	[0x0B] = CHPLAN_ENT(ETSI , 02, ETSI , 22),
	[0x0C] = CHPLAN_ENT(FCC  , 03, FCC  , 54),
	[0x0D] = CHPLAN_ENT(MKK  , 04, MKK  , 14),
	[0x1B] = CHPLAN_ENT(FCC  , 02, FCC  , 52),
	[0x1C] = CHPLAN_ENT(KCC  , 02, KCC  , 53),
	[0x20] = CHPLAN_ENT(WW   , 01, NA   , 00),
	[0x21] = CHPLAN_ENT(ETSI , 02, NA   , 00),
	[0x22] = CHPLAN_ENT(FCC  , 03, NA   , 00),
	[0x23] = CHPLAN_ENT(MKK  , 04, NA   , 00),
	[0x24] = CHPLAN_ENT(ETSI , 05, NA   , 00),
	[0x25] = CHPLAN_ENT(FCC  , 03, FCC  , 03),
	[0x26] = CHPLAN_ENT(ETSI , 01, ETSI , 02),
	[0x27] = CHPLAN_ENT(MKK  , 04, MKK  , 02),
	[0x28] = CHPLAN_ENT(KCC  , 01, KCC  , 05),
	[0x29] = CHPLAN_ENT(FCC  , 01, FCC  , 06),
	[0x2A] = CHPLAN_ENT(FCC  , 02, NA   , 00),
	[0x2B] = CHPLAN_ENT(IC   , 02, IC   , 33),
	[0x2C] = CHPLAN_ENT(MKK  , 02, NA   , 00),
	[0x2D] = CHPLAN_ENT(CHILE, 01, CHILE, 22),
	[0x2E] = CHPLAN_ENT(WW   , 03, WW   , 37),
	[0x2F] = CHPLAN_ENT(CHILE, 01, CHILE, 38),
	[0x30] = CHPLAN_ENT(FCC  , 01, FCC  , 07),
	[0x31] = CHPLAN_ENT(FCC  , 01, FCC  , 08),
	[0x32] = CHPLAN_ENT(FCC  , 01, FCC  , 09),
	[0x33] = CHPLAN_ENT(FCC  , 01, FCC  , 10),
	[0x34] = CHPLAN_ENT(FCC  , 03, FCC  , 01),
	[0x35] = CHPLAN_ENT(ETSI , 01, ETSI , 03),
	[0x36] = CHPLAN_ENT(ETSI , 01, ETSI , 04),
	[0x37] = CHPLAN_ENT(MKK  , 04, MKK  , 10),
	[0x38] = CHPLAN_ENT(MKK  , 04, MKK  , 11),
	[0x39] = CHPLAN_ENT(NCC  , 03, NCC  , 12),
	[0x3A] = CHPLAN_ENT(ETSI , 02, ETSI , 02),
	[0x3B] = CHPLAN_ENT(ACMA , 02, ACMA , 01),
	[0x3C] = CHPLAN_ENT(ETSI , 02, ETSI , 10),
	[0x3D] = CHPLAN_ENT(ETSI , 02, ETSI , 15),
	[0x3E] = CHPLAN_ENT(KCC  , 02, KCC  , 03),
	[0x3F] = CHPLAN_ENT(FCC  , 03, FCC  , 22),
	[0x40] = CHPLAN_ENT(NCC  , 03, NCC  , 13),
	[0x41] = CHPLAN_ENT(WW   , 06, NA   , 00),
	[0x42] = CHPLAN_ENT(ETSI , 02, ETSI , 14),
	[0x43] = CHPLAN_ENT(FCC  , 03, FCC  , 06),
	[0x44] = CHPLAN_ENT(NCC  , 03, NCC  , 09),
	[0x45] = CHPLAN_ENT(ACMA , 01, ACMA , 01),
	[0x46] = CHPLAN_ENT(FCC  , 03, FCC  , 15),
	[0x47] = CHPLAN_ENT(ETSI , 01, ETSI , 10),
	[0x48] = CHPLAN_ENT(ETSI , 01, ETSI , 07),
	[0x49] = CHPLAN_ENT(ETSI , 01, ETSI , 06),
	[0x4A] = CHPLAN_ENT(IC   , 03, IC   , 33),
	[0x4B] = CHPLAN_ENT(KCC  , 02, KCC  , 22),
	[0x4C] = CHPLAN_ENT(FCC  , 03, FCC  , 28),
	[0x4D] = CHPLAN_ENT(MEX  , 02, MEX  , 01),
	[0x4E] = CHPLAN_ENT(ETSI , 02, ETSI , 42),
	[0x4F] = CHPLAN_ENT(NA   , 00, MKK  , 43),
	[0x50] = CHPLAN_ENT(ETSI , 01, ETSI , 16),
	[0x51] = CHPLAN_ENT(ETSI , 01, ETSI , 09),
	[0x52] = CHPLAN_ENT(ETSI , 01, ETSI , 17),
	[0x53] = CHPLAN_ENT(NCC  , 03, NCC  , 18),
	[0x54] = CHPLAN_ENT(ETSI , 01, ETSI , 15),
	[0x55] = CHPLAN_ENT(FCC  , 03, FCC  , 01),
	[0x56] = CHPLAN_ENT(ETSI , 01, ETSI , 19),
	[0x57] = CHPLAN_ENT(FCC  , 03, FCC  , 20),
	[0x58] = CHPLAN_ENT(MKK  , 02, MKK  , 14),
	[0x59] = CHPLAN_ENT(ETSI , 01, ETSI , 21),
	[0x5A] = CHPLAN_ENT(NA   , 00, FCC  , 44),
	[0x5B] = CHPLAN_ENT(NA   , 00, FCC  , 45),
	[0x5C] = CHPLAN_ENT(NA   , 00, FCC  , 43),
	[0x5D] = CHPLAN_ENT(ETSI , 02, ETSI , 08),
	[0x5E] = CHPLAN_ENT(ETSI , 02, ETSI , 03),
	[0x5F] = CHPLAN_ENT(MKK  , 02, MKK  , 47),
	[0x60] = CHPLAN_ENT(FCC  , 03, FCC  , 09),
	[0x61] = CHPLAN_ENT(FCC  , 02, FCC  , 01),
	[0x62] = CHPLAN_ENT(FCC  , 02, FCC  , 03),
	[0x63] = CHPLAN_ENT(ETSI , 01, ETSI , 23),
	[0x64] = CHPLAN_ENT(MKK  , 02, MKK  , 24),
	[0x65] = CHPLAN_ENT(ETSI , 02, ETSI , 24),
	[0x66] = CHPLAN_ENT(FCC  , 03, FCC  , 27),
	[0x67] = CHPLAN_ENT(FCC  , 03, FCC  , 25),
	[0x68] = CHPLAN_ENT(FCC  , 02, FCC  , 27),
	[0x69] = CHPLAN_ENT(FCC  , 02, FCC  , 25),
	[0x6A] = CHPLAN_ENT(ETSI , 02, ETSI , 25),
	[0x6B] = CHPLAN_ENT(FCC  , 01, FCC  , 29),
	[0x6C] = CHPLAN_ENT(FCC  , 01, FCC  , 26),
	[0x6D] = CHPLAN_ENT(FCC  , 02, FCC  , 28),
	[0x6E] = CHPLAN_ENT(FCC  , 01, FCC  , 25),
	[0x6F] = CHPLAN_ENT(NA   , 00, ETSI , 06),
	[0x70] = CHPLAN_ENT(NA   , 00, ETSI , 30),
	[0x71] = CHPLAN_ENT(NA   , 00, ETSI , 25),
	[0x72] = CHPLAN_ENT(NA   , 00, ETSI , 31),
	[0x73] = CHPLAN_ENT(FCC  , 01, FCC  , 01),
	[0x74] = CHPLAN_ENT(FCC  , 02, FCC  , 19),
	[0x75] = CHPLAN_ENT(ETSI , 01, ETSI , 32),
	[0x76] = CHPLAN_ENT(FCC  , 02, FCC  , 22),
	[0x77] = CHPLAN_ENT(ETSI , 01, ETSI , 34),
	[0x78] = CHPLAN_ENT(FCC  , 03, FCC  , 35),
	[0x79] = CHPLAN_ENT(MKK  , 02, MKK  , 02),
	[0x7A] = CHPLAN_ENT(ETSI , 02, ETSI , 28),
	[0x7B] = CHPLAN_ENT(ETSI , 02, ETSI , 46),
	[0x7C] = CHPLAN_ENT(ETSI , 02, ETSI , 47),
	[0x7D] = CHPLAN_ENT(MKK  , 04, MKK  , 48),
	[0x7E] = CHPLAN_ENT(MKK  , 02, MKK  , 48),
	[0x7F] = CHPLAN_ENT(WW   , 01, WW   , 55),
};

#if CONFIG_IEEE80211_BAND_6GHZ
enum rtw_chd_6g {
	RTW_CHD_6G_INVALID = 0,

	RTW_CHD_6G_00,
	RTW_CHD_6G_01,
	RTW_CHD_6G_02,
	RTW_CHD_6G_03,
	RTW_CHD_6G_04,
	RTW_CHD_6G_05,	/* 6G Worldwide */
	RTW_CHD_6G_06,

	RTW_CHD_6G_MAX,
	RTW_CHD_6G_NULL = RTW_CHD_6G_00,
};

static const CH_LIST_6G_T rtw_channel_def_6g[] = {
	/* RTW_CHD_6G_INVALID */	CH_LIST_ENT_6G(0),
	/* RTW_CHD_6G_00 */	CH_LIST_ENT_6G(0),
	/* RTW_CHD_6G_01 */	CH_LIST_ENT_6G(1, 5, 9, 13, 17, 21, 25, 29, 33, 37, 41, 45, 49, 53, 57, 61, 65, 69, 73, 77, 81, 85, 89, 93, 0),
	/* RTW_CHD_6G_02 */	CH_LIST_ENT_6G(97, 101, 105, 109, 113, 117, 0),
	/* RTW_CHD_6G_03 */	CH_LIST_ENT_6G(121, 125, 129, 133, 137, 141, 145, 149, 153, 157, 161, 165, 169, 173, 177, 181, 185, 189, 0),
	/* RTW_CHD_6G_04 */	CH_LIST_ENT_6G(193, 197, 201, 205, 209, 213, 217, 221, 225, 229, 233, 0),
	/* RTW_CHD_6G_05 */	CH_LIST_ENT_6G(1, 5, 9, 13, 17, 21, 25, 29, 33, 37, 41, 45, 49, 53, 57, 61, 65, 69, 73, 77, 81, 85, 89, 93, 97, 101, 105, 109, 113, 117, 121, 125, 129, 133, 137, 141, 145, 149, 153, 157, 161, 165, 169, 173, 177, 181, 185, 189, 193, 197, 201, 205, 209, 213, 217, 221, 225, 229, 233, CLA_6G_B1_PASSIVE | CLA_6G_B2_PASSIVE | CLA_6G_B3_PASSIVE | CLA_6G_B4_PASSIVE),
	/* RTW_CHD_6G_06 */	CH_LIST_ENT_6G(1, 5, 9, 13, 17, 21, 25, 29, 33, 37, 41, 45, 49, 53, 57, 61, 65, 69, 73, 77, 81, 85, 89, 93, 97, 101, 105, 109, 113, 117, 121, 125, 129, 133, 137, 141, 145, 149, 153, 157, 161, 165, 169, 173, 177, 181, 185, 189, 193, 197, 201, 205, 209, 213, 217, 221, 225, 229, 233, 0),
};

static const struct chplan_6g_ent_t rtw_chplan_6g_map[] = {
	[0x00] = CHPLAN_6G_ENT(NA   , 00),
	[0x01] = CHPLAN_6G_ENT(FCC  , 01),
	[0x02] = CHPLAN_6G_ENT(FCC  , 02),
	[0x03] = CHPLAN_6G_ENT(FCC  , 03),
	[0x04] = CHPLAN_6G_ENT(FCC  , 04),
	[0x05] = CHPLAN_6G_ENT(FCC  , 06),
	[0x06] = CHPLAN_6G_ENT(ETSI , 01),
	[0x07] = CHPLAN_6G_ENT(IC   , 06),
	[0x08] = CHPLAN_6G_ENT(KCC  , 06),
	[0x09] = CHPLAN_6G_ENT(KCC  , 01),
	[0x1B] = CHPLAN_6G_ENT(ACMA , 01),
	[0x1C] = CHPLAN_6G_ENT(MKK  , 01),
	[0x7F] = CHPLAN_6G_ENT(WW   , 05),
};
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

static const struct country_chplan country_chplan_map[] = {
	COUNTRY_CHPLAN_ENT("AD", 0x3A, 0x00, DEF    , 1, 1, 1), /* Andorra */
	COUNTRY_CHPLAN_ENT("AE", 0x5E, 0x06, DEF    , 1, 1, 1), /* United Arab Emirates */
	COUNTRY_CHPLAN_ENT("AF", 0x42, 0x00, DEF    , 1, 1, 1), /* Afghanistan */
	COUNTRY_CHPLAN_ENT("AG", 0x76, 0x05, DEF    , 1, 1, 1), /* Antigua & Barbuda */
	COUNTRY_CHPLAN_ENT("AI", 0x5E, 0x06, DEF    , 1, 1, 1), /* Anguilla(UK) */
	COUNTRY_CHPLAN_ENT("AL", 0x5E, 0x00, DEF    , 1, 1, 1), /* Albania */
	COUNTRY_CHPLAN_ENT("AM", 0x5E, 0x06, DEF    , 1, 1, 1), /* Armenia */
	COUNTRY_CHPLAN_ENT("AN", 0x76, 0x00, DEF    , 1, 1, 1), /* Netherlands Antilles */
	COUNTRY_CHPLAN_ENT("AO", 0x5E, 0x00, DEF    , 1, 1, 1), /* Angola */
	COUNTRY_CHPLAN_ENT("AQ", 0x26, 0x00, DEF    , 1, 1, 1), /* Antarctica */
	COUNTRY_CHPLAN_ENT("AR", 0x4D, 0x00, DEF    , 1, 1, 1), /* Argentina */
	COUNTRY_CHPLAN_ENT("AS", 0x76, 0x00, DEF    , 1, 1, 1), /* American Samoa */
	COUNTRY_CHPLAN_ENT("AT", 0x5E, 0x06, DEF    , 1, 1, 1), /* Austria */
	COUNTRY_CHPLAN_ENT("AU", 0x03, 0x1B, DEF    , 1, 1, 1), /* Australia */
	COUNTRY_CHPLAN_ENT("AW", 0x76, 0x05, DEF    , 1, 1, 1), /* Aruba */
	COUNTRY_CHPLAN_ENT("AZ", 0x5E, 0x06, DEF    , 1, 1, 1), /* Azerbaijan */
	COUNTRY_CHPLAN_ENT("BA", 0x5E, 0x00, DEF    , 1, 1, 1), /* Bosnia & Herzegovina */
	COUNTRY_CHPLAN_ENT("BB", 0x76, 0x05, DEF    , 1, 1, 1), /* Barbados */
	COUNTRY_CHPLAN_ENT("BD", 0x5E, 0x00, DEF    , 1, 1, 1), /* Bangladesh */
	COUNTRY_CHPLAN_ENT("BE", 0x5E, 0x06, DEF    , 1, 1, 1), /* Belgium */
	COUNTRY_CHPLAN_ENT("BF", 0x5E, 0x00, DEF    , 1, 1, 1), /* Burkina Faso */
	COUNTRY_CHPLAN_ENT("BG", 0x5E, 0x06, DEF    , 1, 1, 1), /* Bulgaria */
	COUNTRY_CHPLAN_ENT("BH", 0x06, 0x06, DEF    , 1, 1, 1), /* Bahrain */
	COUNTRY_CHPLAN_ENT("BI", 0x3A, 0x06, DEF    , 1, 1, 1), /* Burundi */
	COUNTRY_CHPLAN_ENT("BJ", 0x3A, 0x00, DEF    , 1, 1, 1), /* Benin */
	COUNTRY_CHPLAN_ENT("BM", 0x76, 0x05, DEF    , 1, 1, 1), /* Bermuda (UK) */
	COUNTRY_CHPLAN_ENT("BN", 0x06, 0x00, DEF    , 1, 1, 1), /* Brunei */
	COUNTRY_CHPLAN_ENT("BO", 0x61, 0x00, DEF    , 1, 1, 1), /* Bolivia */
	COUNTRY_CHPLAN_ENT("BR", 0x62, 0x05, DEF    , 1, 1, 1), /* Brazil */
	COUNTRY_CHPLAN_ENT("BS", 0x76, 0x05, DEF    , 1, 1, 1), /* Bahamas */
	COUNTRY_CHPLAN_ENT("BT", 0x5E, 0x00, DEF    , 1, 1, 1), /* Bhutan */
	COUNTRY_CHPLAN_ENT("BV", 0x5E, 0x00, DEF    , 1, 1, 1), /* Bouvet Island (Norway) */
	COUNTRY_CHPLAN_ENT("BW", 0x5E, 0x06, DEF    , 1, 1, 1), /* Botswana */
	COUNTRY_CHPLAN_ENT("BY", 0x08, 0x00, DEF    , 0, 1, 1), /* Belarus */
	COUNTRY_CHPLAN_ENT("BZ", 0x76, 0x00, DEF    , 1, 1, 1), /* Belize */
	COUNTRY_CHPLAN_ENT("CA", 0x2B, 0x07, DEF    , 1, 1, 1), /* Canada */
	COUNTRY_CHPLAN_ENT("CC", 0x03, 0x00, DEF    , 1, 1, 1), /* Cocos (Keeling) Islands (Australia) */
	COUNTRY_CHPLAN_ENT("CD", 0x5E, 0x00, DEF    , 1, 1, 1), /* Congo, Republic of the */
	COUNTRY_CHPLAN_ENT("CF", 0x3A, 0x00, DEF    , 1, 1, 1), /* Central African Republic */
	COUNTRY_CHPLAN_ENT("CG", 0x5E, 0x00, DEF    , 1, 1, 1), /* Congo, Democratic Republic of the. Zaire */
	COUNTRY_CHPLAN_ENT("CH", 0x5E, 0x06, DEF    , 1, 1, 1), /* Switzerland */
	COUNTRY_CHPLAN_ENT("CI", 0x42, 0x00, DEF    , 1, 1, 1), /* Cote d'Ivoire */
	COUNTRY_CHPLAN_ENT("CK", 0x5E, 0x00, DEF    , 1, 1, 1), /* Cook Islands */
	COUNTRY_CHPLAN_ENT("CL", 0x76, 0x01, CHILE  , 1, 1, 1), /* Chile */
	COUNTRY_CHPLAN_ENT("CM", 0x5E, 0x00, DEF    , 1, 1, 1), /* Cameroon */
	COUNTRY_CHPLAN_ENT("CN", 0x06, 0x00, CN     , 1, 1, 1), /* China */
	COUNTRY_CHPLAN_ENT("CO", 0x76, 0x00, DEF    , 1, 1, 1), /* Colombia */
	COUNTRY_CHPLAN_ENT("CR", 0x76, 0x05, DEF    , 1, 1, 1), /* Costa Rica */
	COUNTRY_CHPLAN_ENT("CV", 0x5E, 0x00, DEF    , 1, 1, 1), /* Cape Verde */
	COUNTRY_CHPLAN_ENT("CX", 0x03, 0x00, DEF    , 1, 1, 1), /* Christmas Island (Australia) */
	COUNTRY_CHPLAN_ENT("CY", 0x5E, 0x06, DEF    , 1, 1, 1), /* Cyprus */
	COUNTRY_CHPLAN_ENT("CZ", 0x5E, 0x06, DEF    , 1, 1, 1), /* Czech Republic */
	COUNTRY_CHPLAN_ENT("DE", 0x5E, 0x06, DEF    , 1, 1, 1), /* Germany */
	COUNTRY_CHPLAN_ENT("DJ", 0x5E, 0x06, DEF    , 1, 1, 1), /* Djibouti */
	COUNTRY_CHPLAN_ENT("DK", 0x5E, 0x06, DEF    , 1, 1, 1), /* Denmark */
	COUNTRY_CHPLAN_ENT("DM", 0x76, 0x05, DEF    , 1, 1, 1), /* Dominica */
	COUNTRY_CHPLAN_ENT("DO", 0x76, 0x00, DEF    , 1, 1, 1), /* Dominican Republic */
	COUNTRY_CHPLAN_ENT("DZ", 0x00, 0x06, DEF    , 1, 1, 1), /* Algeria */
	COUNTRY_CHPLAN_ENT("EC", 0x76, 0x00, DEF    , 1, 1, 1), /* Ecuador */
	COUNTRY_CHPLAN_ENT("EE", 0x5E, 0x06, DEF    , 1, 1, 1), /* Estonia */
	COUNTRY_CHPLAN_ENT("EG", 0x3C, 0x00, DEF    , 1, 1, 1), /* Egypt */
	COUNTRY_CHPLAN_ENT("EH", 0x3C, 0x00, DEF    , 1, 1, 1), /* Western Sahara */
	COUNTRY_CHPLAN_ENT("ER", 0x3A, 0x00, DEF    , 1, 1, 1), /* Eritrea */
	COUNTRY_CHPLAN_ENT("ES", 0x5E, 0x06, DEF    , 1, 1, 1), /* Spain, Canary Islands, Ceuta, Melilla */
	COUNTRY_CHPLAN_ENT("ET", 0x3A, 0x00, DEF    , 1, 1, 1), /* Ethiopia */
	COUNTRY_CHPLAN_ENT("FI", 0x5E, 0x06, DEF    , 1, 1, 1), /* Finland */
	COUNTRY_CHPLAN_ENT("FJ", 0x76, 0x00, DEF    , 1, 1, 1), /* Fiji */
	COUNTRY_CHPLAN_ENT("FK", 0x5E, 0x00, DEF    , 1, 1, 1), /* Falkland Islands (Islas Malvinas) (UK) */
	COUNTRY_CHPLAN_ENT("FM", 0x76, 0x00, DEF    , 1, 1, 1), /* Micronesia, Federated States of (USA) */
	COUNTRY_CHPLAN_ENT("FO", 0x5E, 0x00, DEF    , 1, 1, 1), /* Faroe Islands (Denmark) */
	COUNTRY_CHPLAN_ENT("FR", 0x5E, 0x06, DEF    , 1, 1, 1), /* France */
	COUNTRY_CHPLAN_ENT("GA", 0x5E, 0x00, DEF    , 1, 1, 1), /* Gabon */
	COUNTRY_CHPLAN_ENT("GB", 0x0B, 0x06, UK     , 1, 1, 1), /* Great Britain (United Kingdom; England) */
	COUNTRY_CHPLAN_ENT("GD", 0x76, 0x05, DEF    , 1, 1, 1), /* Grenada */
	COUNTRY_CHPLAN_ENT("GE", 0x5E, 0x00, DEF    , 1, 1, 1), /* Georgia */
	COUNTRY_CHPLAN_ENT("GF", 0x3A, 0x00, DEF    , 1, 1, 1), /* French Guiana */
	COUNTRY_CHPLAN_ENT("GG", 0x5E, 0x00, DEF    , 1, 1, 1), /* Guernsey (UK) */
	COUNTRY_CHPLAN_ENT("GH", 0x5E, 0x06, DEF    , 1, 1, 1), /* Ghana */
	COUNTRY_CHPLAN_ENT("GI", 0x5E, 0x00, DEF    , 1, 1, 1), /* Gibraltar (UK) */
	COUNTRY_CHPLAN_ENT("GL", 0x5E, 0x00, DEF    , 1, 1, 1), /* Greenland (Denmark) */
	COUNTRY_CHPLAN_ENT("GM", 0x5E, 0x06, DEF    , 1, 1, 1), /* Gambia */
	COUNTRY_CHPLAN_ENT("GN", 0x5E, 0x00, DEF    , 1, 1, 1), /* Guinea */
	COUNTRY_CHPLAN_ENT("GP", 0x5E, 0x00, DEF    , 1, 1, 1), /* Guadeloupe (France) */
	COUNTRY_CHPLAN_ENT("GQ", 0x5E, 0x06, DEF    , 1, 1, 1), /* Equatorial Guinea */
	COUNTRY_CHPLAN_ENT("GR", 0x5E, 0x06, DEF    , 1, 1, 1), /* Greece */
	COUNTRY_CHPLAN_ENT("GS", 0x3A, 0x00, DEF    , 1, 1, 1), /* South Georgia and the Sandwich Islands (UK) */
	COUNTRY_CHPLAN_ENT("GT", 0x76, 0x05, DEF    , 1, 1, 1), /* Guatemala */
	COUNTRY_CHPLAN_ENT("GU", 0x76, 0x00, DEF    , 1, 1, 1), /* Guam (USA) */
	COUNTRY_CHPLAN_ENT("GW", 0x5E, 0x00, DEF    , 1, 1, 1), /* Guinea-Bissau */
	COUNTRY_CHPLAN_ENT("GY", 0x76, 0x05, DEF    , 1, 1, 1), /* Guyana */
	COUNTRY_CHPLAN_ENT("HK", 0x5E, 0x06, DEF    , 1, 1, 1), /* Hong Kong */
	COUNTRY_CHPLAN_ENT("HM", 0x03, 0x00, DEF    , 1, 1, 1), /* Heard and McDonald Islands (Australia) */
	COUNTRY_CHPLAN_ENT("HN", 0x76, 0x05, DEF    , 1, 1, 1), /* Honduras */
	COUNTRY_CHPLAN_ENT("HR", 0x5E, 0x06, DEF    , 1, 1, 1), /* Croatia */
	COUNTRY_CHPLAN_ENT("HT", 0x76, 0x00, DEF    , 0, 1, 1), /* Haiti */
	COUNTRY_CHPLAN_ENT("HU", 0x5E, 0x06, DEF    , 1, 1, 1), /* Hungary */
	COUNTRY_CHPLAN_ENT("ID", 0x5D, 0x00, DEF    , 1, 1, 1), /* Indonesia */
	COUNTRY_CHPLAN_ENT("IE", 0x5E, 0x06, DEF    , 1, 1, 1), /* Ireland */
	COUNTRY_CHPLAN_ENT("IL", 0x5E, 0x06, DEF    , 1, 1, 1), /* Israel */
	COUNTRY_CHPLAN_ENT("IM", 0x5E, 0x00, DEF    , 1, 1, 1), /* Isle of Man (UK) */
	COUNTRY_CHPLAN_ENT("IN", 0x5E, 0x00, DEF    , 1, 1, 1), /* India */
	COUNTRY_CHPLAN_ENT("IO", 0x3A, 0x00, DEF    , 1, 1, 1), /* British Indian Ocean Territory (UK) */
	COUNTRY_CHPLAN_ENT("IQ", 0x05, 0x00, DEF    , 1, 1, 1), /* Iraq */
	COUNTRY_CHPLAN_ENT("IR", 0x3A, 0x00, DEF    , 0, 0, 0), /* Iran */
	COUNTRY_CHPLAN_ENT("IS", 0x5E, 0x06, DEF    , 1, 1, 1), /* Iceland */
	COUNTRY_CHPLAN_ENT("IT", 0x5E, 0x06, DEF    , 1, 1, 1), /* Italy */
	COUNTRY_CHPLAN_ENT("JE", 0x5E, 0x00, DEF    , 1, 1, 1), /* Jersey (UK) */
	COUNTRY_CHPLAN_ENT("JM", 0x76, 0x05, DEF    , 1, 1, 1), /* Jamaica */
	COUNTRY_CHPLAN_ENT("JO", 0x05, 0x06, DEF    , 1, 1, 1), /* Jordan */
	COUNTRY_CHPLAN_ENT("JP", 0x7D, 0x1C, DEF    , 1, 1, 1), /* Japan- Telec */
	COUNTRY_CHPLAN_ENT("KE", 0x5E, 0x06, DEF    , 1, 1, 1), /* Kenya */
	COUNTRY_CHPLAN_ENT("KG", 0x5E, 0x06, DEF    , 1, 1, 1), /* Kyrgyzstan */
	COUNTRY_CHPLAN_ENT("KH", 0x5E, 0x00, DEF    , 1, 1, 1), /* Cambodia */
	COUNTRY_CHPLAN_ENT("KI", 0x5E, 0x00, DEF    , 1, 1, 1), /* Kiribati */
	COUNTRY_CHPLAN_ENT("KM", 0x5E, 0x06, DEF    , 1, 1, 1), /* Comoros */
	COUNTRY_CHPLAN_ENT("KN", 0x76, 0x05, DEF    , 1, 1, 1), /* Saint Kitts and Nevis */
	COUNTRY_CHPLAN_ENT("KR", 0x4B, 0x08, DEF    , 1, 1, 1), /* South Korea */
	COUNTRY_CHPLAN_ENT("KW", 0x5E, 0x00, DEF    , 1, 1, 1), /* Kuwait */
	COUNTRY_CHPLAN_ENT("KY", 0x76, 0x05, DEF    , 1, 1, 1), /* Cayman Islands (UK) */
	COUNTRY_CHPLAN_ENT("KZ", 0x5E, 0x00, DEF    , 1, 1, 1), /* Kazakhstan */
	COUNTRY_CHPLAN_ENT("LA", 0x5E, 0x00, DEF    , 1, 1, 1), /* Laos */
	COUNTRY_CHPLAN_ENT("LB", 0x5E, 0x00, DEF    , 1, 1, 1), /* Lebanon */
	COUNTRY_CHPLAN_ENT("LC", 0x76, 0x05, DEF    , 1, 1, 1), /* Saint Lucia */
	COUNTRY_CHPLAN_ENT("LI", 0x5E, 0x06, DEF    , 1, 1, 1), /* Liechtenstein */
	COUNTRY_CHPLAN_ENT("LK", 0x5E, 0x00, DEF    , 1, 1, 1), /* Sri Lanka */
	COUNTRY_CHPLAN_ENT("LR", 0x5E, 0x00, DEF    , 1, 1, 1), /* Liberia */
	COUNTRY_CHPLAN_ENT("LS", 0x5E, 0x00, DEF    , 1, 1, 1), /* Lesotho */
	COUNTRY_CHPLAN_ENT("LT", 0x5E, 0x06, DEF    , 1, 1, 1), /* Lithuania */
	COUNTRY_CHPLAN_ENT("LU", 0x5E, 0x06, DEF    , 1, 1, 1), /* Luxembourg */
	COUNTRY_CHPLAN_ENT("LV", 0x5E, 0x06, DEF    , 1, 1, 1), /* Latvia */
	COUNTRY_CHPLAN_ENT("LY", 0x5E, 0x00, DEF    , 1, 1, 1), /* Libya */
	COUNTRY_CHPLAN_ENT("MA", 0x3C, 0x06, DEF    , 1, 1, 1), /* Morocco */
	COUNTRY_CHPLAN_ENT("MC", 0x5E, 0x06, DEF    , 1, 1, 1), /* Monaco */
	COUNTRY_CHPLAN_ENT("MD", 0x5E, 0x06, DEF    , 1, 1, 1), /* Moldova */
	COUNTRY_CHPLAN_ENT("ME", 0x3A, 0x00, DEF    , 1, 1, 1), /* Montenegro */
	COUNTRY_CHPLAN_ENT("MF", 0x76, 0x00, DEF    , 1, 1, 1), /* Saint Martin */
	COUNTRY_CHPLAN_ENT("MG", 0x5E, 0x06, DEF    , 1, 1, 1), /* Madagascar */
	COUNTRY_CHPLAN_ENT("MH", 0x76, 0x00, DEF    , 1, 1, 1), /* Marshall Islands (USA) */
	COUNTRY_CHPLAN_ENT("MK", 0x3A, 0x00, DEF    , 1, 1, 1), /* Republic of Macedonia (FYROM) */
	COUNTRY_CHPLAN_ENT("ML", 0x5E, 0x00, DEF    , 1, 1, 1), /* Mali */
	COUNTRY_CHPLAN_ENT("MM", 0x3A, 0x00, DEF    , 1, 1, 1), /* Burma (Myanmar) */
	COUNTRY_CHPLAN_ENT("MN", 0x5E, 0x00, DEF    , 1, 1, 1), /* Mongolia */
	COUNTRY_CHPLAN_ENT("MO", 0x5E, 0x00, DEF    , 1, 1, 1), /* Macau */
	COUNTRY_CHPLAN_ENT("MP", 0x76, 0x00, DEF    , 1, 1, 1), /* Northern Mariana Islands (USA) */
	COUNTRY_CHPLAN_ENT("MQ", 0x3A, 0x00, DEF    , 1, 1, 1), /* Martinique (France) */
	COUNTRY_CHPLAN_ENT("MR", 0x5E, 0x00, DEF    , 1, 1, 1), /* Mauritania */
	COUNTRY_CHPLAN_ENT("MS", 0x3A, 0x00, DEF    , 1, 1, 1), /* Montserrat (UK) */
	COUNTRY_CHPLAN_ENT("MT", 0x5E, 0x06, DEF    , 1, 1, 1), /* Malta */
	COUNTRY_CHPLAN_ENT("MU", 0x5E, 0x06, DEF    , 1, 1, 1), /* Mauritius */
	COUNTRY_CHPLAN_ENT("MV", 0x3C, 0x00, DEF    , 1, 1, 1), /* Maldives */
	COUNTRY_CHPLAN_ENT("MW", 0x5E, 0x00, DEF    , 1, 1, 1), /* Malawi */
	COUNTRY_CHPLAN_ENT("MX", 0x4D, 0x00, DEF    , 1, 1, 1), /* Mexico */
	COUNTRY_CHPLAN_ENT("MY", 0x07, 0x06, DEF    , 1, 1, 1), /* Malaysia */
	COUNTRY_CHPLAN_ENT("MZ", 0x5E, 0x00, DEF    , 1, 1, 1), /* Mozambique */
	COUNTRY_CHPLAN_ENT("NA", 0x5E, 0x00, DEF    , 1, 1, 1), /* Namibia */
	COUNTRY_CHPLAN_ENT("NC", 0x3A, 0x00, DEF    , 1, 1, 1), /* New Caledonia */
	COUNTRY_CHPLAN_ENT("NE", 0x5E, 0x00, DEF    , 1, 1, 1), /* Niger */
	COUNTRY_CHPLAN_ENT("NF", 0x03, 0x00, DEF    , 1, 1, 1), /* Norfolk Island (Australia) */
	COUNTRY_CHPLAN_ENT("NG", 0x75, 0x00, DEF    , 1, 1, 1), /* Nigeria */
	COUNTRY_CHPLAN_ENT("NI", 0x76, 0x00, DEF    , 1, 1, 1), /* Nicaragua */
	COUNTRY_CHPLAN_ENT("NL", 0x5E, 0x06, DEF    , 1, 1, 1), /* Netherlands */
	COUNTRY_CHPLAN_ENT("NO", 0x5E, 0x06, DEF    , 1, 1, 1), /* Norway */
	COUNTRY_CHPLAN_ENT("NP", 0x06, 0x00, DEF    , 1, 1, 1), /* Nepal */
	COUNTRY_CHPLAN_ENT("NR", 0x5E, 0x00, DEF    , 1, 1, 1), /* Nauru */
	COUNTRY_CHPLAN_ENT("NU", 0x03, 0x00, DEF    , 1, 1, 1), /* Niue */
	COUNTRY_CHPLAN_ENT("NZ", 0x03, 0x1B, DEF    , 1, 1, 1), /* New Zealand */
	COUNTRY_CHPLAN_ENT("OM", 0x5E, 0x00, DEF    , 1, 1, 1), /* Oman */
	COUNTRY_CHPLAN_ENT("PA", 0x76, 0x00, DEF    , 1, 1, 1), /* Panama */
	COUNTRY_CHPLAN_ENT("PE", 0x76, 0x05, DEF    , 1, 1, 1), /* Peru */
	COUNTRY_CHPLAN_ENT("PF", 0x3A, 0x00, DEF    , 1, 1, 1), /* French Polynesia (France) */
	COUNTRY_CHPLAN_ENT("PG", 0x5E, 0x00, DEF    , 1, 1, 1), /* Papua New Guinea */
	COUNTRY_CHPLAN_ENT("PH", 0x5E, 0x00, DEF    , 1, 1, 1), /* Philippines */
	COUNTRY_CHPLAN_ENT("PK", 0x5E, 0x00, DEF    , 1, 1, 1), /* Pakistan */
	COUNTRY_CHPLAN_ENT("PL", 0x5E, 0x06, DEF    , 1, 1, 1), /* Poland */
	COUNTRY_CHPLAN_ENT("PM", 0x5E, 0x00, DEF    , 1, 1, 1), /* Saint Pierre and Miquelon (France) */
	COUNTRY_CHPLAN_ENT("PR", 0x76, 0x00, DEF    , 1, 1, 1), /* Puerto Rico */
	COUNTRY_CHPLAN_ENT("PS", 0x5E, 0x00, DEF    , 1, 1, 1), /* Palestine */
	COUNTRY_CHPLAN_ENT("PT", 0x5E, 0x06, DEF    , 1, 1, 1), /* Portugal */
	COUNTRY_CHPLAN_ENT("PW", 0x76, 0x00, DEF    , 1, 1, 1), /* Palau */
	COUNTRY_CHPLAN_ENT("PY", 0x76, 0x00, DEF    , 1, 1, 1), /* Paraguay */
	COUNTRY_CHPLAN_ENT("QA", 0x5E, 0x06, QATAR  , 1, 1, 1), /* Qatar */
	COUNTRY_CHPLAN_ENT("RE", 0x5E, 0x00, DEF    , 1, 1, 1), /* Reunion (France) */
	COUNTRY_CHPLAN_ENT("RO", 0x5E, 0x06, DEF    , 1, 1, 1), /* Romania */
	COUNTRY_CHPLAN_ENT("RS", 0x5E, 0x06, DEF    , 1, 1, 1), /* Serbia */
	COUNTRY_CHPLAN_ENT("RU", 0x09, 0x00, DEF    , 1, 1, 1), /* Russia(fac/gost), Kaliningrad */
	COUNTRY_CHPLAN_ENT("RW", 0x5E, 0x00, DEF    , 1, 1, 1), /* Rwanda */
	COUNTRY_CHPLAN_ENT("SA", 0x5E, 0x06, DEF    , 1, 1, 1), /* Saudi Arabia */
	COUNTRY_CHPLAN_ENT("SB", 0x3A, 0x00, DEF    , 1, 1, 1), /* Solomon Islands */
	COUNTRY_CHPLAN_ENT("SC", 0x76, 0x00, DEF    , 1, 1, 1), /* Seychelles */
	COUNTRY_CHPLAN_ENT("SE", 0x5E, 0x06, DEF    , 1, 1, 1), /* Sweden */
	COUNTRY_CHPLAN_ENT("SG", 0x5E, 0x00, DEF    , 1, 1, 1), /* Singapore */
	COUNTRY_CHPLAN_ENT("SH", 0x5E, 0x00, DEF    , 1, 1, 1), /* Saint Helena (UK) */
	COUNTRY_CHPLAN_ENT("SI", 0x5E, 0x06, DEF    , 1, 1, 1), /* Slovenia */
	COUNTRY_CHPLAN_ENT("SJ", 0x5E, 0x00, DEF    , 1, 1, 1), /* Svalbard (Norway) */
	COUNTRY_CHPLAN_ENT("SK", 0x5E, 0x06, DEF    , 1, 1, 1), /* Slovakia */
	COUNTRY_CHPLAN_ENT("SL", 0x5E, 0x06, DEF    , 1, 1, 1), /* Sierra Leone */
	COUNTRY_CHPLAN_ENT("SM", 0x3A, 0x00, DEF    , 1, 1, 1), /* San Marino */
	COUNTRY_CHPLAN_ENT("SN", 0x3A, 0x00, DEF    , 1, 1, 1), /* Senegal */
	COUNTRY_CHPLAN_ENT("SO", 0x3A, 0x00, DEF    , 1, 1, 1), /* Somalia */
	COUNTRY_CHPLAN_ENT("SR", 0x74, 0x05, DEF    , 1, 1, 1), /* Suriname */
	COUNTRY_CHPLAN_ENT("ST", 0x5E, 0x00, DEF    , 1, 1, 1), /* Sao Tome and Principe */
	COUNTRY_CHPLAN_ENT("SV", 0x76, 0x05, DEF    , 1, 1, 1), /* El Salvador */
	COUNTRY_CHPLAN_ENT("SX", 0x76, 0x00, DEF    , 1, 1, 1), /* Sint Marteen */
	COUNTRY_CHPLAN_ENT("SZ", 0x5E, 0x00, DEF    , 1, 1, 1), /* Swaziland */
	COUNTRY_CHPLAN_ENT("TC", 0x3A, 0x00, DEF    , 1, 1, 1), /* Turks and Caicos Islands (UK) */
	COUNTRY_CHPLAN_ENT("TD", 0x3A, 0x06, DEF    , 1, 1, 1), /* Chad */
	COUNTRY_CHPLAN_ENT("TF", 0x3A, 0x00, DEF    , 1, 1, 1), /* French Southern and Antarctic Lands (FR Southern Territories) */
	COUNTRY_CHPLAN_ENT("TG", 0x5E, 0x06, DEF    , 1, 1, 1), /* Togo */
	COUNTRY_CHPLAN_ENT("TH", 0x5E, 0x00, DEF    , 1, 1, 1), /* Thailand */
	COUNTRY_CHPLAN_ENT("TJ", 0x5E, 0x06, DEF    , 1, 1, 1), /* Tajikistan */
	COUNTRY_CHPLAN_ENT("TK", 0x03, 0x00, DEF    , 1, 1, 1), /* Tokelau */
	COUNTRY_CHPLAN_ENT("TM", 0x3A, 0x00, DEF    , 1, 1, 1), /* Turkmenistan */
	COUNTRY_CHPLAN_ENT("TN", 0x04, 0x00, DEF    , 1, 1, 1), /* Tunisia */
	COUNTRY_CHPLAN_ENT("TO", 0x3A, 0x00, DEF    , 1, 1, 1), /* Tonga */
	COUNTRY_CHPLAN_ENT("TR", 0x5E, 0x06, DEF    , 1, 1, 1), /* Turkey, Northern Cyprus */
	COUNTRY_CHPLAN_ENT("TT", 0x76, 0x00, DEF    , 1, 1, 1), /* Trinidad & Tobago */
	COUNTRY_CHPLAN_ENT("TV", 0x21, 0x00, DEF    , 0, 0, 0), /* Tuvalu */
	COUNTRY_CHPLAN_ENT("TW", 0x76, 0x00, DEF    , 1, 1, 1), /* Taiwan */
	COUNTRY_CHPLAN_ENT("TZ", 0x5E, 0x00, DEF    , 1, 1, 1), /* Tanzania */
	COUNTRY_CHPLAN_ENT("UA", 0x5E, 0x00, UKRAINE, 1, 1, 1), /* Ukraine */
	COUNTRY_CHPLAN_ENT("UG", 0x3A, 0x00, DEF    , 1, 1, 1), /* Uganda */
	COUNTRY_CHPLAN_ENT("US", 0x1B, 0x05, DEF    , 1, 1, 1), /* United States of America (USA) */
	COUNTRY_CHPLAN_ENT("UY", 0x30, 0x00, DEF    , 1, 1, 1), /* Uruguay */
	COUNTRY_CHPLAN_ENT("UZ", 0x3A, 0x06, DEF    , 1, 1, 1), /* Uzbekistan */
	COUNTRY_CHPLAN_ENT("VA", 0x5E, 0x00, DEF    , 1, 1, 1), /* Holy See (Vatican City) */
	COUNTRY_CHPLAN_ENT("VC", 0x76, 0x00, DEF    , 1, 1, 1), /* Saint Vincent and the Grenadines */
	COUNTRY_CHPLAN_ENT("VE", 0x30, 0x00, DEF    , 1, 1, 1), /* Venezuela */
	COUNTRY_CHPLAN_ENT("VG", 0x76, 0x05, DEF    , 1, 1, 1), /* British Virgin Islands (UK) */
	COUNTRY_CHPLAN_ENT("VI", 0x76, 0x00, DEF    , 1, 1, 1), /* United States Virgin Islands (USA) */
	COUNTRY_CHPLAN_ENT("VN", 0x5E, 0x00, DEF    , 1, 1, 1), /* Vietnam */
	COUNTRY_CHPLAN_ENT("VU", 0x26, 0x00, DEF    , 1, 1, 1), /* Vanuatu */
	COUNTRY_CHPLAN_ENT("WF", 0x3A, 0x00, DEF    , 1, 1, 1), /* Wallis and Futuna (France) */
	COUNTRY_CHPLAN_ENT("WS", 0x76, 0x00, DEF    , 1, 1, 1), /* Samoa */
	COUNTRY_CHPLAN_ENT("XK", 0x5E, 0x00, DEF    , 1, 1, 1), /* Kosovo */
	COUNTRY_CHPLAN_ENT("YE", 0x3A, 0x00, DEF    , 1, 1, 1), /* Yemen */
	COUNTRY_CHPLAN_ENT("YT", 0x5E, 0x00, DEF    , 1, 1, 1), /* Mayotte (France) */
	COUNTRY_CHPLAN_ENT("ZA", 0x5E, 0x00, DEF    , 1, 1, 1), /* South Africa */
	COUNTRY_CHPLAN_ENT("ZM", 0x5E, 0x00, DEF    , 1, 1, 1), /* Zambia */
	COUNTRY_CHPLAN_ENT("ZW", 0x5E, 0x00, DEF    , 1, 1, 1), /* Zimbabwe */
};

#include "rtw_regdb_rtk_common.c"

