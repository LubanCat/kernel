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
#define _RTW_OPC_UTILS_C_

#include <drv_types.h>

const char *const _opc_bw_str[] = {
	[OPC_BW20]	= "20M ",
	[OPC_BW40PLUS]	= "40M+",
	[OPC_BW40MINUS]	= "40M-",
	[OPC_BW40]	= "40M ",
	[OPC_BW80]	= "80M ",
	[OPC_BW160]	= "160M ",
	[OPC_BW80P80]	= "80+80M ",
	[OPC_BW_NUM]	= "UNKNOWN",
};

const u8 _opc_bw_to_ch_width[] = {
	[OPC_BW20]	= CHANNEL_WIDTH_20,
	[OPC_BW40PLUS]	= CHANNEL_WIDTH_40,
	[OPC_BW40MINUS]	= CHANNEL_WIDTH_40,
	[OPC_BW40]	= CHANNEL_WIDTH_40,
	[OPC_BW80]	= CHANNEL_WIDTH_80,
	[OPC_BW160]	= CHANNEL_WIDTH_160,
	[OPC_BW80P80]	= CHANNEL_WIDTH_80_80,
	[OPC_BW_NUM]	= CHANNEL_WIDTH_MAX,
};

#define RTW_GOPCID_NODEF 0
#define RTW_GOPCID_DECLARE_2G(_class) RTW_GOPCID_##_class
#define RTW_GOPCID_2G_END _RTW_GOPCID_2G_END
#if CONFIG_IEEE80211_BAND_5GHZ
#define RTW_GOPCID_DECLARE_5G(_class) RTW_GOPCID_##_class
#define RTW_GOPCID_5G_END _RTW_GOPCID_5G_END
#else
#define RTW_GOPCID_DECLARE_5G(_class) RTW_GOPCID_##_class = RTW_GOPCID_NODEF
#define RTW_GOPCID_5G_END RTW_GOPCID_2G_END
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
#define RTW_GOPCID_DECLARE_6G(_class) RTW_GOPCID_##_class
#define RTW_GOPCID_6G_END _RTW_GOPCID_6G_END
#else
#define RTW_GOPCID_DECLARE_6G(_class) RTW_GOPCID_##_class = RTW_GOPCID_NODEF
#define RTW_GOPCID_6G_END RTW_GOPCID_5G_END
#endif

#define RTW_GOPCID_INVALID(gid) ((gid) == RTW_GOPCID_NODEF || (gid) >= RTW_GOPCID_NUM)

enum rtw_gopc_id_2g {
	_RTW_GOPC_2G_PREV = RTW_GOPCID_NODEF,
	RTW_GOPCID_DECLARE_2G(81),
	RTW_GOPCID_DECLARE_2G(82),
	RTW_GOPCID_DECLARE_2G(83),
	RTW_GOPCID_DECLARE_2G(84),
	_RTW_GOPCID_2G_NUM,
	_RTW_GOPCID_2G_END = _RTW_GOPCID_2G_NUM - 1,
};

enum rtw_gopc_id_5g {
	_RTW_GOPCID_5G_PREV = RTW_GOPCID_2G_END,
	RTW_GOPCID_DECLARE_5G(115),
	RTW_GOPCID_DECLARE_5G(116),
	RTW_GOPCID_DECLARE_5G(117),
	RTW_GOPCID_DECLARE_5G(118),
	RTW_GOPCID_DECLARE_5G(119),
	RTW_GOPCID_DECLARE_5G(120),
	RTW_GOPCID_DECLARE_5G(121),
	RTW_GOPCID_DECLARE_5G(122),
	RTW_GOPCID_DECLARE_5G(123),
	RTW_GOPCID_DECLARE_5G(124),
	RTW_GOPCID_DECLARE_5G(125),
	RTW_GOPCID_DECLARE_5G(126),
	RTW_GOPCID_DECLARE_5G(127),
	RTW_GOPCID_DECLARE_5G(128),
	RTW_GOPCID_DECLARE_5G(129),
	_RTW_GOPCID_5G_NUM,
	_RTW_GOPCID_5G_END = _RTW_GOPCID_5G_NUM - 1,
};

enum rtw_gopc_id_6g {
	_RTW_GOPCID_6G_PREV = RTW_GOPCID_5G_END,
	RTW_GOPCID_DECLARE_6G(131),
	RTW_GOPCID_DECLARE_6G(132),
	RTW_GOPCID_DECLARE_6G(133),
	RTW_GOPCID_DECLARE_6G(134),
	_RTW_GOPCID_6G_NUM,
	_RTW_GOPCID_6G_END = _RTW_GOPCID_6G_NUM - 1,
};

enum rtw_gopc_id {
	RTW_GOPCID_0 = RTW_GOPCID_NODEF,
	RTW_GOPCID_NUM = RTW_GOPCID_6G_END + 1,

	/* below are unsupported */
	RTW_GOPCID_94,
	RTW_GOPCID_95,
	RTW_GOPCID_96,
	RTW_GOPCID_101,
	RTW_GOPCID_102,
	RTW_GOPCID_103,
	RTW_GOPCID_104,
	RTW_GOPCID_105,
	RTW_GOPCID_109,
	RTW_GOPCID_110,
	RTW_GOPCID_111,
	RTW_GOPCID_180,
	RTW_GOPCID_181,
	RTW_GOPCID_182,
	RTW_GOPCID_183,
	RTW_GOPCID_184,
	RTW_GOPCID_185,
	RTW_GOPCID_186,
};

/* global operating class database */
#define OP_CLASS_ENT(_class, _band, _bw, _len, arg...) \
	[RTW_GOPCID_##_class] = {.class_id = _class, .band = _band, .bw = _bw, .len_ch_attr = (uint8_t[_len + 1]) {_len, ##arg},}

/* 802.11-2020, 802.11ax-2021 Table E-4, partial */
const struct op_class_t global_op_class[] = {
	OP_CLASS_ENT(0,		BAND_MAX,	OPC_BW_NUM,	0),

	/* 2G ch1~13, 20M */
	OP_CLASS_ENT(81,	BAND_ON_24G,	OPC_BW20,	13,	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13),
	/* 2G ch14, 20M */
	OP_CLASS_ENT(82,	BAND_ON_24G,	OPC_BW20,	1,	14),
	/* 2G, 40M */
	OP_CLASS_ENT(83,	BAND_ON_24G,	OPC_BW40PLUS,	9,	1, 2, 3, 4, 5, 6, 7, 8, 9),
	OP_CLASS_ENT(84,	BAND_ON_24G,	OPC_BW40MINUS,	9,	5, 6, 7, 8, 9, 10, 11, 12, 13),

#if CONFIG_IEEE80211_BAND_5GHZ
	/* 5G band 1, 20M & 40M */
	OP_CLASS_ENT(115,	BAND_ON_5G,	OPC_BW20,	4,	36, 40, 44, 48),
	OP_CLASS_ENT(116,	BAND_ON_5G,	OPC_BW40PLUS,	2,	36, 44),
	OP_CLASS_ENT(117,	BAND_ON_5G,	OPC_BW40MINUS,	2,	40, 48),
	/* 5G band 2, 20M & 40M */
	OP_CLASS_ENT(118,	BAND_ON_5G,	OPC_BW20,	4,	52, 56, 60, 64),
	OP_CLASS_ENT(119,	BAND_ON_5G,	OPC_BW40PLUS,	2,	52, 60),
	OP_CLASS_ENT(120,	BAND_ON_5G,	OPC_BW40MINUS,	2,	56, 64),
	/* 5G band 3, 20M & 40M */
	OP_CLASS_ENT(121,	BAND_ON_5G,	OPC_BW20,	12,	100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144),
	OP_CLASS_ENT(122,	BAND_ON_5G,	OPC_BW40PLUS,	6,	100, 108, 116, 124, 132, 140),
	OP_CLASS_ENT(123,	BAND_ON_5G,	OPC_BW40MINUS,	6,	104, 112, 120, 128, 136, 144),
	/* 5G band 4, 20M & 40M */
	OP_CLASS_ENT(124,	BAND_ON_5G,	OPC_BW20,	4,	149, 153, 157, 161),
	OP_CLASS_ENT(125,	BAND_ON_5G,	OPC_BW20,	8,	149, 153, 157, 161, 165, 169, 173, 177),
	OP_CLASS_ENT(126,	BAND_ON_5G,	OPC_BW40PLUS,	4,	149, 157, 165, 173),
	OP_CLASS_ENT(127,	BAND_ON_5G,	OPC_BW40MINUS,	4,	153, 161, 169, 177),
	/* 5G, 80M & 160M */
	OP_CLASS_ENT(128,	BAND_ON_5G,	OPC_BW80,	28,	36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165, 169, 173, 177),
	OP_CLASS_ENT(129,	BAND_ON_5G,	OPC_BW160,	24,	36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 149, 153, 157, 161, 165, 169, 173, 177),
	#if 0 /* TODO */
	/* 5G, 80+80M */
	OP_CLASS_ENT(130,	BAND_ON_5G,	OPC_BW80P80,	28,	36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165, 169, 173, 177),
	#endif
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
	/* 6G, 20M */
	OP_CLASS_ENT(131,	BAND_ON_6G,	OPC_BW20,	59,	1, 5, 9, 13, 17, 21, 25, 29, 33, 37, 41, 45, 49, 53, 57, 61, 65, 69, 73, 77, 81, 85, 89, 93
									, 97, 101, 105, 109, 113, 117
									, 121, 125, 129, 133, 137, 141, 145, 149, 153, 157, 161, 165, 169, 173, 177, 181, 185, 189
									, 193, 197, 201, 205, 209, 213, 217, 221, 225, 229, 233),
	/* 6G, 40M */
	OP_CLASS_ENT(132,	BAND_ON_6G,	OPC_BW40,	58,	1, 5, 9, 13, 17, 21, 25, 29, 33, 37, 41, 45, 49, 53, 57, 61, 65, 69, 73, 77, 81, 85, 89, 93
									, 97, 101, 105, 109, 113, 117
									, 121, 125, 129, 133, 137, 141, 145, 149, 153, 157, 161, 165, 169, 173, 177, 181, 185, 189
									, 193, 197, 201, 205, 209, 213, 217, 221, 225, 229),
	/* 6G, 80M */
	OP_CLASS_ENT(133,	BAND_ON_6G,	OPC_BW80,	56,	1, 5, 9, 13, 17, 21, 25, 29, 33, 37, 41, 45, 49, 53, 57, 61, 65, 69, 73, 77, 81, 85, 89, 93
									, 97, 101, 105, 109, 113, 117
									, 121, 125, 129, 133, 137, 141, 145, 149, 153, 157, 161, 165, 169, 173, 177, 181, 185, 189
									, 193, 197, 201, 205, 209, 213, 217, 221),
	/* 6G, 160M */
	OP_CLASS_ENT(134,	BAND_ON_6G,	OPC_BW160,	56,	1, 5, 9, 13, 17, 21, 25, 29, 33, 37, 41, 45, 49, 53, 57, 61, 65, 69, 73, 77, 81, 85, 89, 93
									, 97, 101, 105, 109, 113, 117
									, 121, 125, 129, 133, 137, 141, 145, 149, 153, 157, 161, 165, 169, 173, 177, 181, 185, 189
									, 193, 197, 201, 205, 209, 213, 217, 221),
	#if 0 /* TODO */
	/* 6G, 80+80M */
	OP_CLASS_ENT(135,	BAND_ON_6G,	OPC_BW80P80,	56,	1, 5, 9, 13, 17, 21, 25, 29, 33, 37, 41, 45, 49, 53, 57, 61, 65, 69, 73, 77, 81, 85, 89, 93
									, 97, 101, 105, 109, 113, 117
									, 121, 125, 129, 133, 137, 141, 145, 149, 153, 157, 161, 165, 169, 173, 177, 181, 185, 189
									, 193, 197, 201, 205, 209, 213, 217, 221),
	/* 6G, 20M, ch2 */
	OP_CLASS_ENT(136,	BAND_ON_6G,	OPC_BW20,	1,	2),
	#endif
#endif
};

const int global_op_class_num = sizeof(global_op_class) / sizeof(struct op_class_t);

#define ALPHA2_IS_US(_alpha2)	(_alpha2[0] == 'U' && _alpha2[1] == 'S')
#define ALPHA2_IS_JP(_alpha2)	(_alpha2[0] == 'J' && _alpha2[1] == 'P')
#define ALPHA2_IS_CN(_alpha2)	(_alpha2[0] == 'C' && _alpha2[1] == 'N')

#define COPC_TO_GOPCID_ENT(_copc, _gopc) [_copc] = RTW_GOPCID_##_gopc

/* country specific opc to gopc_id, maintains only different part */
static const u8 us_opc_to_gopc_id[] = {
	COPC_TO_GOPCID_ENT(1, 115),
	COPC_TO_GOPCID_ENT(2, 118),
	COPC_TO_GOPCID_ENT(3, 124),
	COPC_TO_GOPCID_ENT(4, 121),
	COPC_TO_GOPCID_ENT(5, 125),
	COPC_TO_GOPCID_ENT(6, 103),
	COPC_TO_GOPCID_ENT(7, 103),
	COPC_TO_GOPCID_ENT(8, 102),
	COPC_TO_GOPCID_ENT(9, 102),
	COPC_TO_GOPCID_ENT(10, 101),
	COPC_TO_GOPCID_ENT(11, 101),
	COPC_TO_GOPCID_ENT(12, 81),
	COPC_TO_GOPCID_ENT(13, 94),
	COPC_TO_GOPCID_ENT(14, 95),
	COPC_TO_GOPCID_ENT(15, 96),
	COPC_TO_GOPCID_ENT(22, 116),
	COPC_TO_GOPCID_ENT(23, 119),
	COPC_TO_GOPCID_ENT(24, 122),
	COPC_TO_GOPCID_ENT(25, 126),
	COPC_TO_GOPCID_ENT(26, 126),
	COPC_TO_GOPCID_ENT(27, 117),
	COPC_TO_GOPCID_ENT(28, 120),
	COPC_TO_GOPCID_ENT(29, 123),
	COPC_TO_GOPCID_ENT(30, 127),
	COPC_TO_GOPCID_ENT(31, 127),
	COPC_TO_GOPCID_ENT(32, 83),
	COPC_TO_GOPCID_ENT(33, 84),
	COPC_TO_GOPCID_ENT(34, 180),
};

static const u8 eu_opc_to_gopc_id[] = {
	COPC_TO_GOPCID_ENT(1, 115),
	COPC_TO_GOPCID_ENT(2, 118),
	COPC_TO_GOPCID_ENT(3, 121),
	COPC_TO_GOPCID_ENT(4, 81),
	COPC_TO_GOPCID_ENT(5, 116),
	COPC_TO_GOPCID_ENT(6, 119),
	COPC_TO_GOPCID_ENT(7, 122),
	COPC_TO_GOPCID_ENT(8, 117),
	COPC_TO_GOPCID_ENT(9, 120),
	COPC_TO_GOPCID_ENT(10, 123),
	COPC_TO_GOPCID_ENT(11, 83),
	COPC_TO_GOPCID_ENT(12, 84),
	COPC_TO_GOPCID_ENT(17, 125),
	COPC_TO_GOPCID_ENT(18, 180),
};

static const u8 jp_opc_to_gopc_id[] = {
	COPC_TO_GOPCID_ENT(1, 115),
	COPC_TO_GOPCID_ENT(8, 109),
	COPC_TO_GOPCID_ENT(11, 109),
	COPC_TO_GOPCID_ENT(17, 110),
	COPC_TO_GOPCID_ENT(20, 110),
	COPC_TO_GOPCID_ENT(25, 111),
	COPC_TO_GOPCID_ENT(26, 111),
	COPC_TO_GOPCID_ENT(29, 111),
	COPC_TO_GOPCID_ENT(30, 81),
	COPC_TO_GOPCID_ENT(31, 82),
	COPC_TO_GOPCID_ENT(32, 118),
	COPC_TO_GOPCID_ENT(33, 118),
	COPC_TO_GOPCID_ENT(34, 121),
	COPC_TO_GOPCID_ENT(36, 116),
	COPC_TO_GOPCID_ENT(37, 119),
	COPC_TO_GOPCID_ENT(39, 122),
	COPC_TO_GOPCID_ENT(41, 117),
	COPC_TO_GOPCID_ENT(42, 120),
	COPC_TO_GOPCID_ENT(44, 123),
	COPC_TO_GOPCID_ENT(46, 104),
	COPC_TO_GOPCID_ENT(51, 105),
	COPC_TO_GOPCID_ENT(56, 83),
	COPC_TO_GOPCID_ENT(57, 84),
	COPC_TO_GOPCID_ENT(58, 121),
	COPC_TO_GOPCID_ENT(59, 180),
};

static const u8 cn_opc_to_gopc_id[] = {
	COPC_TO_GOPCID_ENT(1, 115),
	COPC_TO_GOPCID_ENT(2, 118),
	COPC_TO_GOPCID_ENT(3, 125),
	COPC_TO_GOPCID_ENT(4, 116),
	COPC_TO_GOPCID_ENT(5, 119),
	COPC_TO_GOPCID_ENT(6, 126),
	COPC_TO_GOPCID_ENT(7, 81),
	COPC_TO_GOPCID_ENT(8, 83),
	COPC_TO_GOPCID_ENT(9, 84),
	COPC_TO_GOPCID_ENT(10, 181),
	COPC_TO_GOPCID_ENT(11, 182),
	COPC_TO_GOPCID_ENT(12, 183),
	COPC_TO_GOPCID_ENT(13, 184),
	COPC_TO_GOPCID_ENT(14, 185),
	COPC_TO_GOPCID_ENT(15, 186),
};

static u8 get_gid_by_op_class(const char *alpha2, u8 op_class)
{
	if (alpha2 && strlen(alpha2) >= 2) {
		if (ALPHA2_IS_US(alpha2)) {
			if (op_class < ARRAY_SIZE(us_opc_to_gopc_id))
				return us_opc_to_gopc_id[op_class];
		} else if (ALPHA2_IS_JP(alpha2)) {
			if (op_class < ARRAY_SIZE(jp_opc_to_gopc_id))
				return jp_opc_to_gopc_id[op_class];
		} else if (ALPHA2_IS_CN(alpha2)) {
			if (op_class < ARRAY_SIZE(cn_opc_to_gopc_id))
				return cn_opc_to_gopc_id[op_class];
		} else {
			/* other country code, try EU */
			if (op_class < ARRAY_SIZE(eu_opc_to_gopc_id))
				return eu_opc_to_gopc_id[op_class];
		}
	}
	return RTW_GOPCID_NUM; /* for not translate */
}

static const struct op_class_t *get_global_opc_by_op_class(u8 op_class)
{
	int i;

	for (i = RTW_GOPCID_0 + 1; i < global_op_class_num; i++)
		if (global_op_class[i].class_id == op_class)
			break;

	return i < global_op_class_num ? &global_op_class[i] : NULL;
}

const struct op_class_t *get_opc_by_op_class(const char *alpha2, u8 op_class)
{
	u8 gid = get_gid_by_op_class(alpha2, op_class);

	if (gid == RTW_GOPCID_NUM) {
		/* not translated, search global op class for match */
		return get_global_opc_by_op_class(op_class);
	}

	if (RTW_GOPCID_INVALID(gid))
		return NULL;

	return &global_op_class[gid];
}

enum band_type rtw_get_band_by_op_class(const char *alpha2, u8 op_class)
{
	const struct op_class_t *opc = get_opc_by_op_class(alpha2, op_class);

	if (!opc) {
		char _alpha2[2] = {0};

		if (alpha2) {
			_alpha2[0] = alpha2[0];
			if (strlen(alpha2) >= 2)
				_alpha2[1] = alpha2[1];
		}

		RTW_INFO("%s can't get opc with alpha2:"ALPHA2_FMT" op_class:%u\n"
			, __func__, ALPHA2_ARG(_alpha2), op_class);
		return BAND_MAX;
	}

	return opc->band;
}

bool rtw_get_bw_offset_by_op_class_ch(const char *alpha2, u8 op_class, u8 ch, u8 *bw, u8 *offset)
{
	const struct op_class_t *opc;
	bool valid = false;

	opc = get_opc_by_op_class(alpha2, op_class);
	if (!opc) {
		char _alpha2[2] = {0};

		if (alpha2) {
			_alpha2[0] = alpha2[0];
			if (strlen(alpha2) >= 2)
				_alpha2[1] = alpha2[1];
		}

		RTW_INFO("%s can't get opc with alpha2:"ALPHA2_FMT" op_class:%u\n"
			, __func__, ALPHA2_ARG(_alpha2), op_class);
		goto exit;
	}

	*bw = opc_bw_to_ch_width(opc->bw);

	if (opc->bw == OPC_BW40PLUS)
		*offset = CHAN_OFFSET_UPPER;
	else if (opc->bw == OPC_BW40MINUS)
		*offset = CHAN_OFFSET_LOWER;

	if (rtw_get_offset_by_bchbw(opc->band, ch, *bw, offset))
		valid = true;

exit:
	return valid;
}

bool opc_contains_ch(const struct op_class_t *opc, u8 ch)
{
	int i;

	for (i = 0; i < OPC_CH_LIST_LEN(opc); i++)
		if (OPC_CH_LIST_CH(opc, i) == ch)
			break;

	return i < OPC_CH_LIST_LEN(opc);
}

void dump_op_class_ch_title(void *sel)
{
	RTW_PRINT_SEL(sel, "%-5s %-4s %-7s ch_list\n"
		, "class", "band", "bw");
}

static void dump_global_op_class_ch_single(void *sel, u8 gid)
{
	const struct op_class_t *opc = &global_op_class[gid];
	u8 i;

	RTW_PRINT_SEL(sel, "%5u %4s %7s"
		, opc->class_id
		, band_str(opc->band)
		, opc_bw_str(opc->bw));

	for (i = 0; i < OPC_CH_LIST_LEN(opc); i++)
		_RTW_PRINT_SEL(sel, " %u", OPC_CH_LIST_CH(opc, i));

	_RTW_PRINT_SEL(sel, "\n");
}

void dump_global_op_class(void *sel)
{
	u8 i;

	dump_op_class_ch_title(sel);

	for (i = RTW_GOPCID_0 + 1; i < global_op_class_num; i++)
		dump_global_op_class_ch_single(sel, i);
}

u8 _rtw_get_op_class_by_bchbw(enum band_type band, u8 ch, u8 bw, u8 offset)
{
	int i;
	u8 op_class = 0; /* invalid */

	switch (bw) {
	case CHANNEL_WIDTH_20:
	case CHANNEL_WIDTH_40:
	case CHANNEL_WIDTH_80:
	case CHANNEL_WIDTH_160:
	#if 0 /* TODO */
	case CHANNEL_WIDTH_80_80:
	#endif
		break;
	default:
		goto exit;
	}

	for (i = RTW_GOPCID_0 + 1; i < global_op_class_num; i++) {
		if (band != global_op_class[i].band)
			continue;

		if (opc_bw_to_ch_width(global_op_class[i].bw) != bw)
			continue;

		if ((global_op_class[i].bw == OPC_BW40PLUS
				&& offset != CHAN_OFFSET_UPPER)
			|| (global_op_class[i].bw == OPC_BW40MINUS
				&& offset != CHAN_OFFSET_LOWER)
		)
			continue;

		if (opc_contains_ch(&global_op_class[i], ch))
			goto get;
	}

get:
	if (i < global_op_class_num) {
		#if 0 /* TODO */
		if (bw == CHANNEL_WIDTH_80_80) {
			/* search another ch */
			if (!opc_contains_ch(&global_op_class[i], ch2))
				goto exit;
		}
		#endif

		op_class = global_op_class[i].class_id;
	}

exit:
	return op_class;
}

#if CONFIG_ALLOW_FUNC_2G_5G_ONLY
RTW_FUNC_2G_5G_ONLY u8 rtw_get_op_class_by_chbw(u8 ch, u8 bw, u8 offset)
{
	enum band_type band = BAND_MAX;

	if (rtw_is_2g_ch(ch))
		band = BAND_ON_24G;
	else if (rtw_is_5g_ch(ch))
		band = BAND_ON_5G;
	else
		return 0; /* invalid */

	return _rtw_get_op_class_by_bchbw(band, ch, bw, offset);
}
#endif

u8 rtw_get_op_class_by_bchbw(enum band_type band, u8 ch, u8 bw, u8 offset)
{
	return _rtw_get_op_class_by_bchbw(band, ch, bw, offset);
}

#ifdef CONFIG_RTW_OPCLASS_DEV
static bool dump_global_op_class_validate(void *sel, u8 gid)
{
	const struct op_class_t *opc = &global_op_class[gid];
	u8 i;
	u8 ch, bw, offset, cch;
	bool ret = true;

	switch (opc->bw) {
	case OPC_BW20:
		bw = CHANNEL_WIDTH_20;
		offset = CHAN_OFFSET_NO_EXT;
		break;
	case OPC_BW40PLUS:
		bw = CHANNEL_WIDTH_40;
		offset = CHAN_OFFSET_UPPER;
		break;
	case OPC_BW40MINUS:
		bw = CHANNEL_WIDTH_40;
		offset = CHAN_OFFSET_LOWER;
		break;
	case OPC_BW40:
		bw = CHANNEL_WIDTH_40;
		offset = CHAN_OFFSET_NO_EXT;
		break;
	case OPC_BW80:
		bw = CHANNEL_WIDTH_80;
		offset = CHAN_OFFSET_NO_EXT;
		break;
	case OPC_BW160:
		bw = CHANNEL_WIDTH_160;
		offset = CHAN_OFFSET_NO_EXT;
		break;
	case OPC_BW80P80: /* TODO */
	default:
		RTW_PRINT_SEL(sel, "class:%u unsupported opc_bw:%u\n"
			, opc->class_id, opc->bw);
		ret = false;
		goto exit;
	}

	for (i = 0; i < OPC_CH_LIST_LEN(opc); i++) {
		u8 *op_chs;
		u8 op_ch_num;
		u8 k;

		ch = OPC_CH_LIST_CH(opc, i);
		cch = rtw_get_center_ch_by_band(opc->band, ch ,bw, offset);
		if (!cch) {
			RTW_PRINT_SEL(sel, "can't get cch from class:%u ch:%u\n"
				, opc->class_id, ch);
			ret = false;
			continue;
		}

		if (!rtw_get_op_chs_by_bcch_bw(opc->band, cch, bw, &op_chs, &op_ch_num)) {
			RTW_PRINT_SEL(sel, "can't get op chs from class:%u cch:%u\n"
				, opc->class_id, cch);
			ret = false;
			continue;
		}

		for (k = 0; k < op_ch_num; k++) {
			if (*(op_chs + k) == ch)
				break;
		}
		if (k >= op_ch_num) {
			RTW_PRINT_SEL(sel, "can't get ch:%u from op_chs class:%u cch:%u\n"
				, ch, opc->class_id, cch);
			ret = false;
		}
	}

exit:
	return ret;
}

static void dump_get_opc_by_op_class(void *sel)
{
	char *alpha2[] = {"US", "DE", "JP", "CN", "TW"};
	int i, j;
	const struct op_class_t *opc;

	for (i = 0; i < ARRAY_SIZE(alpha2); i++) {
		for (j = 0; j < 256; j++) {
			opc = get_opc_by_op_class(alpha2[i], j);
			if (!opc)
				continue;
			RTW_PRINT_SEL(sel, "%c%c %3u -> %3u%s\n"
				, alpha2[i][0], alpha2[i][1], j, opc->class_id
				, j == opc->class_id ? " same" : ""
			);
		}
	}
}

void dump_opc_test(void *sel)
{
	int i;

	RTW_PRINT_SEL(sel, "RTW_GOPCID_81: %d\n", RTW_GOPCID_81);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_82: %d\n", RTW_GOPCID_82);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_83: %d\n", RTW_GOPCID_83);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_84: %d\n", RTW_GOPCID_84);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_115:%d\n", RTW_GOPCID_115);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_116:%d\n", RTW_GOPCID_116);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_117:%d\n", RTW_GOPCID_117);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_118:%d\n", RTW_GOPCID_118);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_119:%d\n", RTW_GOPCID_119);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_120:%d\n", RTW_GOPCID_120);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_121:%d\n", RTW_GOPCID_121);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_122:%d\n", RTW_GOPCID_122);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_123:%d\n", RTW_GOPCID_123);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_124:%d\n", RTW_GOPCID_124);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_125:%d\n", RTW_GOPCID_125);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_126:%d\n", RTW_GOPCID_126);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_127:%d\n", RTW_GOPCID_127);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_128:%d\n", RTW_GOPCID_128);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_129:%d\n", RTW_GOPCID_129);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_131:%d\n", RTW_GOPCID_131);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_132:%d\n", RTW_GOPCID_132);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_133:%d\n", RTW_GOPCID_133);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_134:%d\n", RTW_GOPCID_134);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_2G_END:%d\n", RTW_GOPCID_2G_END);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_5G_END:%d\n", RTW_GOPCID_5G_END);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_6G_END:%d\n", RTW_GOPCID_6G_END);
	RTW_PRINT_SEL(sel, "RTW_GOPCID_NUM:%d\n", RTW_GOPCID_NUM);

	for (i = RTW_GOPCID_0 + 1; i < RTW_GOPCID_NUM; i++)
		dump_global_op_class_validate(sel, i);

	dump_get_opc_by_op_class(sel);
}
#endif /* CONFIG_RTW_OPCLASS_DEV */
