/******************************************************************************
 *
 * Copyright(c) 2007 - 2024 Realtek Corporation.
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
#ifndef __RTW_REGDB_RTK_COMMON_H__
#define __RTW_REGDB_RTK_COMMON_H__

#define rtw_is_5g_band1(ch) ((ch) >= 36 && (ch) <= 48)
#define rtw_is_5g_band2(ch) ((ch) >= 52 && (ch) <= 64)
#define rtw_is_5g_band3(ch) ((ch) >= 100 && (ch) <= 144)
#define rtw_is_5g_band4(ch) ((ch) >= 149 && (ch) <= 177)

#define rtw_is_6g_band1(ch) ((ch) >= 1 && (ch) <= 93)
#define rtw_is_6g_band2(ch) ((ch) >= 97 && (ch) <= 117)
#define rtw_is_6g_band3(ch) ((ch) >= 121 && (ch) <= 189)
#define rtw_is_6g_band4(ch) ((ch) >= 193 && (ch) <= 237)

#define CH_MAP_2G 1
#define CH_MAP_5G 1
#define CH_MAP_6G 1
#define CC_2D_OFFSET 1

#define CHM_2G_0	0
#define CHM_2G_1	BIT0
#define CHM_2G_2	BIT1
#define CHM_2G_3	BIT2
#define CHM_2G_4	BIT3
#define CHM_2G_5	BIT4
#define CHM_2G_6	BIT5
#define CHM_2G_7	BIT6
#define CHM_2G_8	BIT7
#define CHM_2G_9	BIT8
#define CHM_2G_10	BIT9
#define CHM_2G_11	BIT10
#define CHM_2G_12	BIT11
#define CHM_2G_13	BIT12
#define CHM_2G_14	BIT13

#define CH_TO_CHM_2G(ch) ((ch) <= 14 ? BIT((ch) - 1) : 0)

#define CLA_2G_12_14_PASSIVE	BIT0

#define CHM_5G_0	0
#define CHM_5G_36	BIT0
#define CHM_5G_40	BIT1
#define CHM_5G_44	BIT2
#define CHM_5G_48	BIT3
#define CHM_5G_52	BIT4
#define CHM_5G_56	BIT5
#define CHM_5G_60	BIT6
#define CHM_5G_64	BIT7
#define CHM_5G_100	BIT8
#define CHM_5G_104	BIT9
#define CHM_5G_108	BIT10
#define CHM_5G_112	BIT11
#define CHM_5G_116	BIT12
#define CHM_5G_120	BIT13
#define CHM_5G_124	BIT14
#define CHM_5G_128	BIT15
#define CHM_5G_132	BIT16
#define CHM_5G_136	BIT17
#define CHM_5G_140	BIT18
#define CHM_5G_144	BIT19
#define CHM_5G_149	BIT20
#define CHM_5G_153	BIT21
#define CHM_5G_157	BIT22
#define CHM_5G_161	BIT23
#define CHM_5G_165	BIT24
#define CHM_5G_169	BIT25
#define CHM_5G_173	BIT26
#define CHM_5G_177	BIT27

#define CH_TO_CHM_5G(ch) \
	( \
		((ch) >= 36 && (ch) <= 64) ? (((ch) & 0x03) ? 0 : BIT(((ch) - 36) >> 2)) : \
		((ch) >= 100 && (ch) <= 144) ? (((ch) & 0x03) ? 0 : BIT(8 + (((ch) - 100) >> 2))) : \
		((ch) >= 149 && (ch) <= 177) ? ((((ch) - 1) & 0x03) ? 0 : BIT(20 + (((ch) - 149) >> 2))) : 0 \
	)

#define CLA_5G_B1_PASSIVE		BIT0
#define CLA_5G_B2_PASSIVE		BIT1
#define CLA_5G_B3_PASSIVE		BIT2
#define CLA_5G_B4_PASSIVE		BIT3
#define CLA_5G_B2_DFS			BIT4
#define CLA_5G_B3_DFS			BIT5
#define CLA_5G_B4_DFS			BIT6

#define CHM_6G_0	0
#define CHM_6G_1	BIT_ULL(0)
#define CHM_6G_5	BIT_ULL(1)
#define CHM_6G_9	BIT_ULL(2)
#define CHM_6G_13	BIT_ULL(3)
#define CHM_6G_17	BIT_ULL(4)
#define CHM_6G_21	BIT_ULL(5)
#define CHM_6G_25	BIT_ULL(6)
#define CHM_6G_29	BIT_ULL(7)
#define CHM_6G_33	BIT_ULL(8)
#define CHM_6G_37	BIT_ULL(9)
#define CHM_6G_41	BIT_ULL(10)
#define CHM_6G_45	BIT_ULL(11)
#define CHM_6G_49	BIT_ULL(12)
#define CHM_6G_53	BIT_ULL(13)
#define CHM_6G_57	BIT_ULL(14)
#define CHM_6G_61	BIT_ULL(15)
#define CHM_6G_65	BIT_ULL(16)
#define CHM_6G_69	BIT_ULL(17)
#define CHM_6G_73	BIT_ULL(18)
#define CHM_6G_77	BIT_ULL(19)
#define CHM_6G_81	BIT_ULL(20)
#define CHM_6G_85	BIT_ULL(21)
#define CHM_6G_89	BIT_ULL(22)
#define CHM_6G_93	BIT_ULL(23)
#define CHM_6G_97	BIT_ULL(24)
#define CHM_6G_101	BIT_ULL(25)
#define CHM_6G_105	BIT_ULL(26)
#define CHM_6G_109	BIT_ULL(27)
#define CHM_6G_113	BIT_ULL(28)
#define CHM_6G_117	BIT_ULL(29)
#define CHM_6G_121	BIT_ULL(30)
#define CHM_6G_125	BIT_ULL(31)
#define CHM_6G_129	BIT_ULL(32)
#define CHM_6G_133	BIT_ULL(33)
#define CHM_6G_137	BIT_ULL(34)
#define CHM_6G_141	BIT_ULL(35)
#define CHM_6G_145	BIT_ULL(36)
#define CHM_6G_149	BIT_ULL(37)
#define CHM_6G_153	BIT_ULL(38)
#define CHM_6G_157	BIT_ULL(39)
#define CHM_6G_161	BIT_ULL(40)
#define CHM_6G_165	BIT_ULL(41)
#define CHM_6G_169	BIT_ULL(42)
#define CHM_6G_173	BIT_ULL(43)
#define CHM_6G_177	BIT_ULL(44)
#define CHM_6G_181	BIT_ULL(45)
#define CHM_6G_185	BIT_ULL(46)
#define CHM_6G_189	BIT_ULL(47)
#define CHM_6G_193	BIT_ULL(48)
#define CHM_6G_197	BIT_ULL(49)
#define CHM_6G_201	BIT_ULL(50)
#define CHM_6G_205	BIT_ULL(51)
#define CHM_6G_209	BIT_ULL(52)
#define CHM_6G_213	BIT_ULL(53)
#define CHM_6G_217	BIT_ULL(54)
#define CHM_6G_221	BIT_ULL(55)
#define CHM_6G_225	BIT_ULL(56)
#define CHM_6G_229	BIT_ULL(57)
#define CHM_6G_233	BIT_ULL(58)
#define CHM_6G_237	BIT_ULL(59)
#define CHM_6G_241	BIT_ULL(60)
#define CHM_6G_245	BIT_ULL(61)
#define CHM_6G_249	BIT_ULL(62)
#define CHM_6G_253	BIT_ULL(63)

#define CH_TO_CHM_6G(ch) ((((ch) - 1) & 0x03) ? 0 : BIT(((ch) - 1) >> 2))

#define CLA_6G_B1_PASSIVE		BIT0
#define CLA_6G_B2_PASSIVE		BIT1
#define CLA_6G_B3_PASSIVE		BIT2
#define CLA_6G_B4_PASSIVE		BIT3

struct ch_list_2g_t {
	u16 ch_map:14;
	u16 attr:1;
};

struct ch_list_5g_t {
	u32 ch_map:28;
	u32 attr:7;
};

struct ch_list_6g_t {
	u64 ch_map;
	u8 attr;
};

#define CH_LIST_ENT_2G_ARGC_15( \
	c01, c02, c03, c04, c05, c06, c07, c08, c09, c10, \
	c11, c12, c13, c14, a) \
	{ \
		.attr = a, .ch_map = \
			CHM_2G_##c01 | CHM_2G_##c02 | CHM_2G_##c03 | CHM_2G_##c04 | CHM_2G_##c05 | \
			CHM_2G_##c06 | CHM_2G_##c07 | CHM_2G_##c08 | CHM_2G_##c09 | CHM_2G_##c10 | \
			CHM_2G_##c11 | CHM_2G_##c12 | CHM_2G_##c13 | CHM_2G_##c14 \
	}
#define CH_LIST_ENT_2G_ARGC_14(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, a)	CH_LIST_ENT_2G_ARGC_15(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, 0, a)
#define CH_LIST_ENT_2G_ARGC_13(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, a)		CH_LIST_ENT_2G_ARGC_15(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12,   0, 0, a)
#define CH_LIST_ENT_2G_ARGC_12(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, a)			CH_LIST_ENT_2G_ARGC_15(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11,   0,   0, 0, a)
#define CH_LIST_ENT_2G_ARGC_11(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, a)			CH_LIST_ENT_2G_ARGC_15(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10,   0,   0,   0, 0, a)
#define CH_LIST_ENT_2G_ARGC_10(c1, c2, c3, c4, c5, c6, c7, c8, c9, a)				CH_LIST_ENT_2G_ARGC_15(c1, c2, c3, c4, c5, c6, c7, c8, c9,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_2G_ARGC_09(c1, c2, c3, c4, c5, c6, c7, c8, a)				CH_LIST_ENT_2G_ARGC_15(c1, c2, c3, c4, c5, c6, c7, c8,  0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_2G_ARGC_08(c1, c2, c3, c4, c5, c6, c7, a)					CH_LIST_ENT_2G_ARGC_15(c1, c2, c3, c4, c5, c6, c7,  0,  0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_2G_ARGC_07(c1, c2, c3, c4, c5, c6, a)					CH_LIST_ENT_2G_ARGC_15(c1, c2, c3, c4, c5, c6,  0,  0,  0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_2G_ARGC_06(c1, c2, c3, c4, c5, a)						CH_LIST_ENT_2G_ARGC_15(c1, c2, c3, c4, c5,  0,  0,  0,  0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_2G_ARGC_05(c1, c2, c3, c4, a)						CH_LIST_ENT_2G_ARGC_15(c1, c2, c3, c4,  0,  0,  0,  0,  0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_2G_ARGC_04(c1, c2, c3, a)							CH_LIST_ENT_2G_ARGC_15(c1, c2, c3,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_2G_ARGC_03(c1, c2, a)							CH_LIST_ENT_2G_ARGC_15(c1, c2,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_2G_ARGC_02(c1, a)								CH_LIST_ENT_2G_ARGC_15(c1,  0,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_2G_ARGC_01(a)								CH_LIST_ENT_2G_ARGC_15( 0,  0,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0, 0, a)

#define __CH_LIST_ENT_2G_HDL( \
	                         a15, a14, a13, a12, a11, \
	a10, a09, a08, a07, a06, a05, a04, a03, a02, a01, \
	hdl, ...) hdl

#define __CH_LIST_ENT_2G(...) \
	__CH_LIST_ENT_2G_HDL(__VA_ARGS__, \
		CH_LIST_ENT_2G_ARGC_15, CH_LIST_ENT_2G_ARGC_14, CH_LIST_ENT_2G_ARGC_13, CH_LIST_ENT_2G_ARGC_12, CH_LIST_ENT_2G_ARGC_11, \
		CH_LIST_ENT_2G_ARGC_10, CH_LIST_ENT_2G_ARGC_09, CH_LIST_ENT_2G_ARGC_08, CH_LIST_ENT_2G_ARGC_07, CH_LIST_ENT_2G_ARGC_06, \
		CH_LIST_ENT_2G_ARGC_05, CH_LIST_ENT_2G_ARGC_04, CH_LIST_ENT_2G_ARGC_03, CH_LIST_ENT_2G_ARGC_02, CH_LIST_ENT_2G_ARGC_01) \
		(__VA_ARGS__)

#define CH_LIST_ENT_5G_ARGC_29( \
	c01, c02, c03, c04, c05, c06, c07, c08, c09, c10, \
	c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, \
	c21, c22, c23, c24, c25, c26, c27, c28, a) \
	{ \
		.attr = a, .ch_map = \
			CHM_5G_##c01 | CHM_5G_##c02 | CHM_5G_##c03 | CHM_5G_##c04 | CHM_5G_##c05 | \
			CHM_5G_##c06 | CHM_5G_##c07 | CHM_5G_##c08 | CHM_5G_##c09 | CHM_5G_##c10 | \
			CHM_5G_##c11 | CHM_5G_##c12 | CHM_5G_##c13 | CHM_5G_##c14 | CHM_5G_##c15 | \
			CHM_5G_##c16 | CHM_5G_##c17 | CHM_5G_##c18 | CHM_5G_##c19 | CHM_5G_##c20 | \
			CHM_5G_##c21 | CHM_5G_##c22 | CHM_5G_##c23 | CHM_5G_##c24 | CHM_5G_##c25 | \
			CHM_5G_##c26 | CHM_5G_##c27 | CHM_5G_##c28 \
	}
#define CH_LIST_ENT_5G_ARGC_28(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, a)	CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, 0, a)
#define CH_LIST_ENT_5G_ARGC_27(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, a)	CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_26(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, a)		CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_25(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, a)		CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_24(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, a)			CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_23(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, a)				CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_22(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, a)				CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_21(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, a)					CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_20(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, a)						CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_19(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, a)						CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_18(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, a)							CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_17(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, a)							CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_16(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, a)								CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_15(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, a)									CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_14(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, a)									CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_13(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, a)										CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_12(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, a)											CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_11(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, a)											CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_10(c1, c2, c3, c4, c5, c6, c7, c8, c9, a)												CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_09(c1, c2, c3, c4, c5, c6, c7, c8, a)												CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_08(c1, c2, c3, c4, c5, c6, c7, a)													CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6, c7,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_07(c1, c2, c3, c4, c5, c6, a)													CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5, c6,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_06(c1, c2, c3, c4, c5, a)														CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4, c5,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_05(c1, c2, c3, c4, a)														CH_LIST_ENT_5G_ARGC_29(c1, c2, c3, c4,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_04(c1, c2, c3, a)															CH_LIST_ENT_5G_ARGC_29(c1, c2, c3,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_03(c1, c2, a)															CH_LIST_ENT_5G_ARGC_29(c1, c2,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_02(c1, a)																CH_LIST_ENT_5G_ARGC_29(c1,  0,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_5G_ARGC_01(a)																CH_LIST_ENT_5G_ARGC_29( 0,  0,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)

#define __CH_LIST_ENT_5G_HDL( \
	     a29, a28, a27, a26, a25, a24, a23, a22, a21, \
	a20, a19, a18, a17, a16, a15, a14, a13, a12, a11, \
	a10, a09, a08, a07, a06, a05, a04, a03, a02, a01, \
	hdl, ...) hdl

#define __CH_LIST_ENT_5G(...) \
	__CH_LIST_ENT_5G_HDL(__VA_ARGS__, \
		                        CH_LIST_ENT_5G_ARGC_29, CH_LIST_ENT_5G_ARGC_28, CH_LIST_ENT_5G_ARGC_27, CH_LIST_ENT_5G_ARGC_26, \
		CH_LIST_ENT_5G_ARGC_25, CH_LIST_ENT_5G_ARGC_24, CH_LIST_ENT_5G_ARGC_23, CH_LIST_ENT_5G_ARGC_22, CH_LIST_ENT_5G_ARGC_21, \
		CH_LIST_ENT_5G_ARGC_20, CH_LIST_ENT_5G_ARGC_19, CH_LIST_ENT_5G_ARGC_18, CH_LIST_ENT_5G_ARGC_17, CH_LIST_ENT_5G_ARGC_16, \
		CH_LIST_ENT_5G_ARGC_15, CH_LIST_ENT_5G_ARGC_14, CH_LIST_ENT_5G_ARGC_13, CH_LIST_ENT_5G_ARGC_12, CH_LIST_ENT_5G_ARGC_11, \
		CH_LIST_ENT_5G_ARGC_10, CH_LIST_ENT_5G_ARGC_09, CH_LIST_ENT_5G_ARGC_08, CH_LIST_ENT_5G_ARGC_07, CH_LIST_ENT_5G_ARGC_06, \
		CH_LIST_ENT_5G_ARGC_05, CH_LIST_ENT_5G_ARGC_04, CH_LIST_ENT_5G_ARGC_03, CH_LIST_ENT_5G_ARGC_02, CH_LIST_ENT_5G_ARGC_01) \
		(__VA_ARGS__)

#define CH_LIST_ENT_6G_ARGC_65( \
	c01, c02, c03, c04, c05, c06, c07, c08, c09, c10, \
	c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, \
	c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, \
	c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, \
	c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, \
	c51, c52, c53, c54, c55, c56, c57, c58, c59, c60, \
	c61, c62, c63, c64, a) \
	{ \
		.attr = a, .ch_map = \
			CHM_6G_##c01 | CHM_6G_##c02 | CHM_6G_##c03 | CHM_6G_##c04 | CHM_6G_##c05 | \
			CHM_6G_##c06 | CHM_6G_##c07 | CHM_6G_##c08 | CHM_6G_##c09 | CHM_6G_##c10 | \
			CHM_6G_##c11 | CHM_6G_##c12 | CHM_6G_##c13 | CHM_6G_##c14 | CHM_6G_##c15 | \
			CHM_6G_##c16 | CHM_6G_##c17 | CHM_6G_##c18 | CHM_6G_##c19 | CHM_6G_##c20 | \
			CHM_6G_##c21 | CHM_6G_##c22 | CHM_6G_##c23 | CHM_6G_##c24 | CHM_6G_##c25 | \
			CHM_6G_##c26 | CHM_6G_##c27 | CHM_6G_##c28 | CHM_6G_##c29 | CHM_6G_##c30 | \
			CHM_6G_##c31 | CHM_6G_##c32 | CHM_6G_##c33 | CHM_6G_##c34 | CHM_6G_##c35 | \
			CHM_6G_##c36 | CHM_6G_##c37 | CHM_6G_##c38 | CHM_6G_##c39 | CHM_6G_##c40 | \
			CHM_6G_##c41 | CHM_6G_##c42 | CHM_6G_##c43 | CHM_6G_##c44 | CHM_6G_##c45 | \
			CHM_6G_##c46 | CHM_6G_##c47 | CHM_6G_##c48 | CHM_6G_##c49 | CHM_6G_##c50 | \
			CHM_6G_##c51 | CHM_6G_##c52 | CHM_6G_##c53 | CHM_6G_##c54 | CHM_6G_##c55 | \
			CHM_6G_##c56 | CHM_6G_##c57 | CHM_6G_##c58 | CHM_6G_##c59 | CHM_6G_##c60 | \
			CHM_6G_##c61 | CHM_6G_##c62 | CHM_6G_##c63 | CHM_6G_##c64 \
	}
#define CH_LIST_ENT_6G_ARGC_64(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, c60, c61, c62, c63, a)	CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, c60, c61, c62, c63, 0, a)
#define CH_LIST_ENT_6G_ARGC_63(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, c60, c61, c62, a)		CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, c60, c61, c62,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_62(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, c60, c61, a)		CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, c60, c61,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_61(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, c60, a)			CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, c60,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_60(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, a)				CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_59(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, a)				CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_58(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, a)					CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_57(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, a)					CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_56(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, a)						CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_55(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, a)							CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_54(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, a)							CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_53(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, a)								CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_52(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, a)									CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_51(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, a)									CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, c50,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_50(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49, a)										CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, c49,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_49(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48, a)										CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c48,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_48(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, a)											CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_47(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, a)												CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_46(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, a)												CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_45(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, a)													CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_44(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, a)														CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_43(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, a)														CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_42(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, a)															CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, c41,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_41(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40, a)															CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_40(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, a)																CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_39(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, a)																	CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_38(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, a)																	CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_37(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, a)																		CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_36(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, a)																			CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_35(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, a)																			CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_34(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, a)																				CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_33(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, a)																				CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_32(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, a)																					CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_31(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, a)																						CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_30(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, a)																						CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_29(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, a)																							CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_28(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, a)																								CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_27(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, a)																								CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_26(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, a)																									CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_25(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, a)																									CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_24(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, a)																										CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_23(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, a)																											CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_22(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, a)																											CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_21(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, a)																												CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_20(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, a)																													CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_19(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, a)																													CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_18(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, a)																														CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_17(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, a)																														CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_16(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, a)																															CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_15(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, a)																																CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_14(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, a)																																CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_13(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, a)																																	CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_12(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, a)																																		CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_11(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, a)																																		CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_10(c1, c2, c3, c4, c5, c6, c7, c8, c9, a)																																			CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8, c9,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_09(c1, c2, c3, c4, c5, c6, c7, c8, a)																																			CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7, c8,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_08(c1, c2, c3, c4, c5, c6, c7, a)																																				CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6, c7,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_07(c1, c2, c3, c4, c5, c6, a)																																				CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5, c6,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_06(c1, c2, c3, c4, c5, a)																																					CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4, c5,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_05(c1, c2, c3, c4, a)																																					CH_LIST_ENT_6G_ARGC_65(c1, c2, c3, c4,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_04(c1, c2, c3, a)																																						CH_LIST_ENT_6G_ARGC_65(c1, c2, c3,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_03(c1, c2, a)																																						CH_LIST_ENT_6G_ARGC_65(c1, c2,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_02(c1, a)																																							CH_LIST_ENT_6G_ARGC_65(c1,  0,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)
#define CH_LIST_ENT_6G_ARGC_01(a)																																							CH_LIST_ENT_6G_ARGC_65( 0,  0,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, a)

#define __CH_LIST_ENT_6G_HDL( \
	                         a65, a64, a63, a62, a61, \
	a60, a59, a58, a57, a56, a55, a54, a53, a52, a51, \
	a50, a49, a48, a47, a46, a45, a44, a43, a42, a41, \
	a40, a39, a38, a37, a36, a35, a34, a33, a32, a31, \
	a30, a29, a28, a27, a26, a25, a24, a23, a22, a21, \
	a20, a19, a18, a17, a16, a15, a14, a13, a12, a11, \
	a10, a09, a08, a07, a06, a05, a04, a03, a02, a01, \
	hdl, ...) hdl

#define __CH_LIST_ENT_6G(...) \
	__CH_LIST_ENT_6G_HDL(__VA_ARGS__, \
		CH_LIST_ENT_6G_ARGC_65, CH_LIST_ENT_6G_ARGC_64, CH_LIST_ENT_6G_ARGC_63, CH_LIST_ENT_6G_ARGC_62, CH_LIST_ENT_6G_ARGC_61, \
		CH_LIST_ENT_6G_ARGC_60, CH_LIST_ENT_6G_ARGC_59, CH_LIST_ENT_6G_ARGC_58, CH_LIST_ENT_6G_ARGC_57, CH_LIST_ENT_6G_ARGC_56, \
		CH_LIST_ENT_6G_ARGC_55, CH_LIST_ENT_6G_ARGC_54, CH_LIST_ENT_6G_ARGC_53, CH_LIST_ENT_6G_ARGC_52, CH_LIST_ENT_6G_ARGC_51, \
		CH_LIST_ENT_6G_ARGC_50, CH_LIST_ENT_6G_ARGC_49, CH_LIST_ENT_6G_ARGC_48, CH_LIST_ENT_6G_ARGC_47, CH_LIST_ENT_6G_ARGC_46, \
		CH_LIST_ENT_6G_ARGC_45, CH_LIST_ENT_6G_ARGC_44, CH_LIST_ENT_6G_ARGC_43, CH_LIST_ENT_6G_ARGC_42, CH_LIST_ENT_6G_ARGC_41, \
		CH_LIST_ENT_6G_ARGC_40, CH_LIST_ENT_6G_ARGC_39, CH_LIST_ENT_6G_ARGC_38, CH_LIST_ENT_6G_ARGC_37, CH_LIST_ENT_6G_ARGC_36, \
		CH_LIST_ENT_6G_ARGC_35, CH_LIST_ENT_6G_ARGC_34, CH_LIST_ENT_6G_ARGC_33, CH_LIST_ENT_6G_ARGC_32, CH_LIST_ENT_6G_ARGC_31, \
		CH_LIST_ENT_6G_ARGC_30, CH_LIST_ENT_6G_ARGC_29, CH_LIST_ENT_6G_ARGC_28, CH_LIST_ENT_6G_ARGC_27, CH_LIST_ENT_6G_ARGC_26, \
		CH_LIST_ENT_6G_ARGC_25, CH_LIST_ENT_6G_ARGC_24, CH_LIST_ENT_6G_ARGC_23, CH_LIST_ENT_6G_ARGC_22, CH_LIST_ENT_6G_ARGC_21, \
		CH_LIST_ENT_6G_ARGC_20, CH_LIST_ENT_6G_ARGC_19, CH_LIST_ENT_6G_ARGC_18, CH_LIST_ENT_6G_ARGC_17, CH_LIST_ENT_6G_ARGC_16, \
		CH_LIST_ENT_6G_ARGC_15, CH_LIST_ENT_6G_ARGC_14, CH_LIST_ENT_6G_ARGC_13, CH_LIST_ENT_6G_ARGC_12, CH_LIST_ENT_6G_ARGC_11, \
		CH_LIST_ENT_6G_ARGC_10, CH_LIST_ENT_6G_ARGC_09, CH_LIST_ENT_6G_ARGC_08, CH_LIST_ENT_6G_ARGC_07, CH_LIST_ENT_6G_ARGC_06, \
		CH_LIST_ENT_6G_ARGC_05, CH_LIST_ENT_6G_ARGC_04, CH_LIST_ENT_6G_ARGC_03, CH_LIST_ENT_6G_ARGC_02, CH_LIST_ENT_6G_ARGC_01) \
		(__VA_ARGS__)

struct ch_list_t {
	u8 *len_ch_attr;
};

#define CH_LIST_LEN(_ch_list) (_ch_list.len_ch_attr[0])
#define CH_LIST_CH(_ch_list, _i) (_ch_list.len_ch_attr[_i + 1])
#define CH_LIST_ATTRIB(_ch_list) (_ch_list.len_ch_attr[CH_LIST_LEN(_ch_list) + 1])

#define CH_LIST_ENT_ARGC_65(arg...) {.len_ch_attr = (u8[64 + 2]) {64, ##arg}, }
#define CH_LIST_ENT_ARGC_64(arg...) {.len_ch_attr = (u8[63 + 2]) {63, ##arg}, }
#define CH_LIST_ENT_ARGC_63(arg...) {.len_ch_attr = (u8[62 + 2]) {62, ##arg}, }
#define CH_LIST_ENT_ARGC_62(arg...) {.len_ch_attr = (u8[61 + 2]) {61, ##arg}, }
#define CH_LIST_ENT_ARGC_61(arg...) {.len_ch_attr = (u8[60 + 2]) {60, ##arg}, }
#define CH_LIST_ENT_ARGC_60(arg...) {.len_ch_attr = (u8[59 + 2]) {59, ##arg}, }
#define CH_LIST_ENT_ARGC_59(arg...) {.len_ch_attr = (u8[58 + 2]) {58, ##arg}, }
#define CH_LIST_ENT_ARGC_58(arg...) {.len_ch_attr = (u8[57 + 2]) {57, ##arg}, }
#define CH_LIST_ENT_ARGC_57(arg...) {.len_ch_attr = (u8[56 + 2]) {56, ##arg}, }
#define CH_LIST_ENT_ARGC_56(arg...) {.len_ch_attr = (u8[55 + 2]) {55, ##arg}, }
#define CH_LIST_ENT_ARGC_55(arg...) {.len_ch_attr = (u8[54 + 2]) {54, ##arg}, }
#define CH_LIST_ENT_ARGC_54(arg...) {.len_ch_attr = (u8[53 + 2]) {53, ##arg}, }
#define CH_LIST_ENT_ARGC_53(arg...) {.len_ch_attr = (u8[52 + 2]) {52, ##arg}, }
#define CH_LIST_ENT_ARGC_52(arg...) {.len_ch_attr = (u8[51 + 2]) {51, ##arg}, }
#define CH_LIST_ENT_ARGC_51(arg...) {.len_ch_attr = (u8[50 + 2]) {50, ##arg}, }
#define CH_LIST_ENT_ARGC_50(arg...) {.len_ch_attr = (u8[49 + 2]) {49, ##arg}, }
#define CH_LIST_ENT_ARGC_49(arg...) {.len_ch_attr = (u8[48 + 2]) {48, ##arg}, }
#define CH_LIST_ENT_ARGC_48(arg...) {.len_ch_attr = (u8[47 + 2]) {47, ##arg}, }
#define CH_LIST_ENT_ARGC_47(arg...) {.len_ch_attr = (u8[46 + 2]) {46, ##arg}, }
#define CH_LIST_ENT_ARGC_46(arg...) {.len_ch_attr = (u8[45 + 2]) {45, ##arg}, }
#define CH_LIST_ENT_ARGC_45(arg...) {.len_ch_attr = (u8[44 + 2]) {44, ##arg}, }
#define CH_LIST_ENT_ARGC_44(arg...) {.len_ch_attr = (u8[43 + 2]) {43, ##arg}, }
#define CH_LIST_ENT_ARGC_43(arg...) {.len_ch_attr = (u8[42 + 2]) {42, ##arg}, }
#define CH_LIST_ENT_ARGC_42(arg...) {.len_ch_attr = (u8[41 + 2]) {41, ##arg}, }
#define CH_LIST_ENT_ARGC_41(arg...) {.len_ch_attr = (u8[40 + 2]) {40, ##arg}, }
#define CH_LIST_ENT_ARGC_40(arg...) {.len_ch_attr = (u8[39 + 2]) {39, ##arg}, }
#define CH_LIST_ENT_ARGC_39(arg...) {.len_ch_attr = (u8[38 + 2]) {38, ##arg}, }
#define CH_LIST_ENT_ARGC_38(arg...) {.len_ch_attr = (u8[37 + 2]) {37, ##arg}, }
#define CH_LIST_ENT_ARGC_37(arg...) {.len_ch_attr = (u8[36 + 2]) {36, ##arg}, }
#define CH_LIST_ENT_ARGC_36(arg...) {.len_ch_attr = (u8[35 + 2]) {35, ##arg}, }
#define CH_LIST_ENT_ARGC_35(arg...) {.len_ch_attr = (u8[34 + 2]) {34, ##arg}, }
#define CH_LIST_ENT_ARGC_34(arg...) {.len_ch_attr = (u8[33 + 2]) {33, ##arg}, }
#define CH_LIST_ENT_ARGC_33(arg...) {.len_ch_attr = (u8[32 + 2]) {32, ##arg}, }
#define CH_LIST_ENT_ARGC_32(arg...) {.len_ch_attr = (u8[31 + 2]) {31, ##arg}, }
#define CH_LIST_ENT_ARGC_31(arg...) {.len_ch_attr = (u8[30 + 2]) {30, ##arg}, }
#define CH_LIST_ENT_ARGC_30(arg...) {.len_ch_attr = (u8[29 + 2]) {29, ##arg}, }
#define CH_LIST_ENT_ARGC_29(arg...) {.len_ch_attr = (u8[28 + 2]) {28, ##arg}, }
#define CH_LIST_ENT_ARGC_28(arg...) {.len_ch_attr = (u8[27 + 2]) {27, ##arg}, }
#define CH_LIST_ENT_ARGC_27(arg...) {.len_ch_attr = (u8[26 + 2]) {26, ##arg}, }
#define CH_LIST_ENT_ARGC_26(arg...) {.len_ch_attr = (u8[25 + 2]) {25, ##arg}, }
#define CH_LIST_ENT_ARGC_25(arg...) {.len_ch_attr = (u8[24 + 2]) {24, ##arg}, }
#define CH_LIST_ENT_ARGC_24(arg...) {.len_ch_attr = (u8[23 + 2]) {23, ##arg}, }
#define CH_LIST_ENT_ARGC_23(arg...) {.len_ch_attr = (u8[22 + 2]) {22, ##arg}, }
#define CH_LIST_ENT_ARGC_22(arg...) {.len_ch_attr = (u8[21 + 2]) {21, ##arg}, }
#define CH_LIST_ENT_ARGC_21(arg...) {.len_ch_attr = (u8[20 + 2]) {20, ##arg}, }
#define CH_LIST_ENT_ARGC_20(arg...) {.len_ch_attr = (u8[19 + 2]) {19, ##arg}, }
#define CH_LIST_ENT_ARGC_19(arg...) {.len_ch_attr = (u8[18 + 2]) {18, ##arg}, }
#define CH_LIST_ENT_ARGC_18(arg...) {.len_ch_attr = (u8[17 + 2]) {17, ##arg}, }
#define CH_LIST_ENT_ARGC_17(arg...) {.len_ch_attr = (u8[16 + 2]) {16, ##arg}, }
#define CH_LIST_ENT_ARGC_16(arg...) {.len_ch_attr = (u8[15 + 2]) {15, ##arg}, }
#define CH_LIST_ENT_ARGC_15(arg...) {.len_ch_attr = (u8[14 + 2]) {14, ##arg}, }
#define CH_LIST_ENT_ARGC_14(arg...) {.len_ch_attr = (u8[13 + 2]) {13, ##arg}, }
#define CH_LIST_ENT_ARGC_13(arg...) {.len_ch_attr = (u8[12 + 2]) {12, ##arg}, }
#define CH_LIST_ENT_ARGC_12(arg...) {.len_ch_attr = (u8[11 + 2]) {11, ##arg}, }
#define CH_LIST_ENT_ARGC_11(arg...) {.len_ch_attr = (u8[10 + 2]) {10, ##arg}, }
#define CH_LIST_ENT_ARGC_10(arg...) {.len_ch_attr = (u8[ 9 + 2]) { 9, ##arg}, }
#define CH_LIST_ENT_ARGC_09(arg...) {.len_ch_attr = (u8[ 8 + 2]) { 8, ##arg}, }
#define CH_LIST_ENT_ARGC_08(arg...) {.len_ch_attr = (u8[ 7 + 2]) { 7, ##arg}, }
#define CH_LIST_ENT_ARGC_07(arg...) {.len_ch_attr = (u8[ 6 + 2]) { 6, ##arg}, }
#define CH_LIST_ENT_ARGC_06(arg...) {.len_ch_attr = (u8[ 5 + 2]) { 5, ##arg}, }
#define CH_LIST_ENT_ARGC_05(arg...) {.len_ch_attr = (u8[ 4 + 2]) { 4, ##arg}, }
#define CH_LIST_ENT_ARGC_04(arg...) {.len_ch_attr = (u8[ 3 + 2]) { 3, ##arg}, }
#define CH_LIST_ENT_ARGC_03(arg...) {.len_ch_attr = (u8[ 2 + 2]) { 2, ##arg}, }
#define CH_LIST_ENT_ARGC_02(arg...) {.len_ch_attr = (u8[ 1 + 2]) { 1, ##arg}, }
#define CH_LIST_ENT_ARGC_01(arg...) {.len_ch_attr = (u8[ 0 + 2]) { 0, ##arg}, }

#define __CH_LIST_ENT_HDL( \
	                         a65, a64, a63, a62, a61, \
	a60, a59, a58, a57, a56, a55, a54, a53, a52, a51, \
	a50, a49, a48, a47, a46, a45, a44, a43, a42, a41, \
	a40, a39, a38, a37, a36, a35, a34, a33, a32, a31, \
	a30, a29, a28, a27, a26, a25, a24, a23, a22, a21, \
	a20, a19, a18, a17, a16, a15, a14, a13, a12, a11, \
	a10, a09, a08, a07, a06, a05, a04, a03, a02, a01, \
	hdl, ...) hdl

#define CH_LIST_ENT(...) \
	__CH_LIST_ENT_HDL(__VA_ARGS__, \
		CH_LIST_ENT_ARGC_65, CH_LIST_ENT_ARGC_64, CH_LIST_ENT_ARGC_63, CH_LIST_ENT_ARGC_62, CH_LIST_ENT_ARGC_61, \
		CH_LIST_ENT_ARGC_60, CH_LIST_ENT_ARGC_59, CH_LIST_ENT_ARGC_58, CH_LIST_ENT_ARGC_57, CH_LIST_ENT_ARGC_56, \
		CH_LIST_ENT_ARGC_55, CH_LIST_ENT_ARGC_54, CH_LIST_ENT_ARGC_53, CH_LIST_ENT_ARGC_52, CH_LIST_ENT_ARGC_51, \
		CH_LIST_ENT_ARGC_50, CH_LIST_ENT_ARGC_49, CH_LIST_ENT_ARGC_48, CH_LIST_ENT_ARGC_47, CH_LIST_ENT_ARGC_46, \
		CH_LIST_ENT_ARGC_45, CH_LIST_ENT_ARGC_44, CH_LIST_ENT_ARGC_43, CH_LIST_ENT_ARGC_42, CH_LIST_ENT_ARGC_41, \
		CH_LIST_ENT_ARGC_40, CH_LIST_ENT_ARGC_39, CH_LIST_ENT_ARGC_38, CH_LIST_ENT_ARGC_37, CH_LIST_ENT_ARGC_36, \
		CH_LIST_ENT_ARGC_35, CH_LIST_ENT_ARGC_34, CH_LIST_ENT_ARGC_33, CH_LIST_ENT_ARGC_32, CH_LIST_ENT_ARGC_31, \
		CH_LIST_ENT_ARGC_30, CH_LIST_ENT_ARGC_29, CH_LIST_ENT_ARGC_28, CH_LIST_ENT_ARGC_27, CH_LIST_ENT_ARGC_26, \
		CH_LIST_ENT_ARGC_25, CH_LIST_ENT_ARGC_24, CH_LIST_ENT_ARGC_23, CH_LIST_ENT_ARGC_22, CH_LIST_ENT_ARGC_21, \
		CH_LIST_ENT_ARGC_20, CH_LIST_ENT_ARGC_19, CH_LIST_ENT_ARGC_18, CH_LIST_ENT_ARGC_17, CH_LIST_ENT_ARGC_16, \
		CH_LIST_ENT_ARGC_15, CH_LIST_ENT_ARGC_14, CH_LIST_ENT_ARGC_13, CH_LIST_ENT_ARGC_12, CH_LIST_ENT_ARGC_11, \
		CH_LIST_ENT_ARGC_10, CH_LIST_ENT_ARGC_09, CH_LIST_ENT_ARGC_08, CH_LIST_ENT_ARGC_07, CH_LIST_ENT_ARGC_06, \
		CH_LIST_ENT_ARGC_05, CH_LIST_ENT_ARGC_04, CH_LIST_ENT_ARGC_03, CH_LIST_ENT_ARGC_02, CH_LIST_ENT_ARGC_01) \
		(__VA_ARGS__)

#if CH_MAP_2G
typedef struct ch_list_2g_t CH_LIST_2G_T;
#define CH_LIST_ENT_2G __CH_LIST_ENT_2G
#define CH_LIST_ATTRIB_2G(_ch_list) (_ch_list.attr)
#else
typedef struct ch_list_t CH_LIST_2G_T;
#define CH_LIST_ENT_2G CH_LIST_ENT
#define CH_LIST_ATTRIB_2G CH_LIST_ATTRIB
#endif

#if CH_MAP_5G
typedef struct ch_list_5g_t CH_LIST_5G_T;
#define CH_LIST_ENT_5G __CH_LIST_ENT_5G
#define CH_LIST_ATTRIB_5G(_ch_list) (_ch_list.attr)
#else
typedef struct ch_list_t CH_LIST_5G_T;
#define CH_LIST_ENT_5G CH_LIST_ENT
#define CH_LIST_ATTRIB_5G CH_LIST_ATTRIB
#endif

#if CH_MAP_6G
typedef struct ch_list_6g_t CH_LIST_6G_T;
#define CH_LIST_ENT_6G __CH_LIST_ENT_6G
#define CH_LIST_ATTRIB_6G(_ch_list) (_ch_list.attr)
#else
typedef struct ch_list_t CH_LIST_6G_T;
#define CH_LIST_ENT_6G CH_LIST_ENT
#define CH_LIST_ATTRIB_6G CH_LIST_ATTRIB
#endif

struct chplan_ent_t {
	u8 regd_2g; /* value of enum rtw_regd */
	u8 chd_2g;
#if CONFIG_IEEE80211_BAND_5GHZ
	u8 regd_5g; /* value of enum rtw_regd */
	u8 chd_5g;
#endif
};

#if CONFIG_IEEE80211_BAND_5GHZ
#define CHPLAN_ENT(_regd_2g, _chd_2g, _regd_5g, _chd_5g) {.regd_2g = RTW_REGD_##_regd_2g, .chd_2g = RTW_CHD_2G_##_chd_2g, .regd_5g = RTW_REGD_##_regd_5g, .chd_5g = RTW_CHD_5G_##_chd_5g}
#else
#define CHPLAN_ENT(_regd_2g, _chd_2g, _regd_5g, _chd_5g) {.regd_2g = RTW_REGD_##_regd_2g, .chd_2g = RTW_CHD_2G_##_chd_2g}
#endif

#define CHPLAN_ENT_NOT_DEFINED CHPLAN_ENT(NA, INVALID, NA, INVALID)

struct chplan_6g_ent_t {
	u8 regd; /* value of enum rtw_regd */
	u8 chd;
};

#define CHPLAN_6G_ENT(_regd, _chd) {.regd = RTW_REGD_##_regd, .chd = RTW_CHD_6G_##_chd}

#define CHPLAN_6G_ENT_NOT_DEFINED CHPLAN_6G_ENT(NA, INVALID)

#endif /* __RTW_REGDB_RTK_COMMON_H__ */

