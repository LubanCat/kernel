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

const char *const _opc_bw_str[OPC_BW_NUM] = {
	[OPC_BW20]	= "20M ",
	[OPC_BW40PLUS]	= "40M+",
	[OPC_BW40MINUS]	= "40M-",
	[OPC_BW40]	= "40M ",
	[OPC_BW80]	= "80M ",
	[OPC_BW160]	= "160M ",
	[OPC_BW80P80]	= "80+80M ",
};

const u8 _opc_bw_to_ch_width[OPC_BW_NUM] = {
	[OPC_BW20]	= CHANNEL_WIDTH_20,
	[OPC_BW40PLUS]	= CHANNEL_WIDTH_40,
	[OPC_BW40MINUS]	= CHANNEL_WIDTH_40,
	[OPC_BW40]	= CHANNEL_WIDTH_40,
	[OPC_BW80]	= CHANNEL_WIDTH_80,
	[OPC_BW160]	= CHANNEL_WIDTH_160,
	[OPC_BW80P80]	= CHANNEL_WIDTH_80_80,
};

/* global operating class database */
#define OP_CLASS_ENT(_class, _band, _bw, _len, arg...) \
	{.class_id = _class, .band = _band, .bw = _bw, .len_ch_attr = (uint8_t[_len + 1]) {_len, ##arg},}

/* 802.11-2020, 802.11ax-2021 Table E-4, partial */
const struct op_class_t global_op_class[] = {
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

const struct op_class_t *get_global_opc_by_id(u8 gid)
{
	int i;

	for (i = 0; i < global_op_class_num; i++)
		if (global_op_class[i].class_id == gid)
			break;

	return i < global_op_class_num ? &global_op_class[i] : NULL;
}

bool opc_contains_ch(const struct op_class_t *opc, u8 ch)
{
	int i;

	for (i = 0; i < OPC_CH_LIST_LEN(opc); i++)
		if (OPC_CH_LIST_CH(opc, i) == ch)
			break;

	return i < OPC_CH_LIST_LEN(opc);
}

/* -2: logic error, -1: error, 0: is already BW20 */
s16 get_sub_op_class(u8 gid, u8 ch)
{
	const struct op_class_t *opc = get_global_opc_by_id(gid);
	int i;
	enum channel_width bw;

	if (!opc)
		return -1;

	if (!opc_contains_ch(opc, ch)) {
		return -1;
	}

	if (opc->bw == OPC_BW20)
		return 0;

	bw = opc_bw_to_ch_width(opc->bw);

	for (i = 0; i < global_op_class_num; i++) {
		if (bw != opc_bw_to_ch_width(global_op_class[i].bw) + 1)
			continue;
		if (opc_contains_ch(&global_op_class[i], ch))
			break;
	}

	return i < global_op_class_num ? global_op_class[i].class_id : -2;
}

void dump_op_class_ch_title(void *sel)
{
	RTW_PRINT_SEL(sel, "%-5s %-4s %-7s ch_list\n"
		, "class", "band", "bw");
}

static void dump_global_op_class_ch_single(void *sel, u8 aid)
{
	const struct op_class_t *opc = &global_op_class[aid];
	u8 i;

	RTW_PRINT_SEL(sel, "%5u %4s %7s"
		, opc->class_id
		, band_str(opc->band)
		, opc_bw_str(opc->bw));

	for (i = 0; i < OPC_CH_LIST_LEN(opc); i++)
		_RTW_PRINT_SEL(sel, " %u", OPC_CH_LIST_CH(opc, i));

	_RTW_PRINT_SEL(sel, "\n");
}

#ifdef CONFIG_RTW_OPCLASS_DEV
static bool dbg_global_op_class_validate(u8 aid)
{
	const struct op_class_t *opc = &global_op_class[aid];
	u8 i;
	u8 ch, bw, offset, cch;
	bool ret = 1;

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
		RTW_ERR("%s class:%u unsupported opc_bw:%u\n"
			, __func__, opc->class_id, opc->bw);
		ret = 0;
		goto exit;
	}

	for (i = 0; i < OPC_CH_LIST_LEN(opc); i++) {
		u8 *op_chs;
		u8 op_ch_num;
		u8 k;

		ch = OPC_CH_LIST_CH(opc, i);
		cch = rtw_get_center_ch_by_band(opc->band, ch ,bw, offset);
		if (!cch) {
			RTW_ERR("%s can't get cch from class:%u ch:%u\n"
				, __func__, opc->class_id, ch);
			ret = 0;
			continue;
		}

		if (!rtw_get_op_chs_by_bcch_bw(opc->band, cch, bw, &op_chs, &op_ch_num)) {
			RTW_ERR("%s can't get op chs from class:%u cch:%u\n"
				, __func__, opc->class_id, cch);
			ret = 0;
			continue;
		}

		for (k = 0; k < op_ch_num; k++) {
			if (*(op_chs + k) == ch)
				break;
		}
		if (k >= op_ch_num) {
			RTW_ERR("%s can't get ch:%u from op_chs class:%u cch:%u\n"
				, __func__, ch, opc->class_id, cch);
			ret = 0;
		}
	}

exit:
	return ret;
}
#endif /* CONFIG_RTW_OPCLASS_DEV */

void dump_global_op_class(void *sel)
{
	u8 i;

	dump_op_class_ch_title(sel);

	for (i = 0; i < global_op_class_num; i++) {
		#ifdef CONFIG_RTW_OPCLASS_DEV
		dbg_global_op_class_validate(i);
		#endif
		dump_global_op_class_ch_single(sel, i);
	}
}

u8 _rtw_get_op_class_by_bchbw(enum band_type band, u8 ch, u8 bw, u8 offset)
{
	int i;
	u8 gid = 0; /* invalid */

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

	for (i = 0; i < global_op_class_num; i++) {
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

		gid = global_op_class[i].class_id;
	}

exit:
	return gid;
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

enum band_type rtw_get_band_by_op_class(u8 op_class)
{
	const struct op_class_t *opc = get_global_opc_by_id(op_class);

	if (!opc) {
		RTW_INFO("%s can't get opc with id:%u\n", __func__, op_class);
		return BAND_MAX;
	}

	return opc->band;
}

u8 rtw_get_bw_offset_by_op_class_ch(u8 gid, u8 ch, u8 *bw, u8 *offset)
{
	const struct op_class_t *opc;
	u8 valid = 0;

	opc = get_global_opc_by_id(gid);
	if (!opc)
		goto exit;

	*bw = opc_bw_to_ch_width(opc->bw);

	if (opc->bw == OPC_BW40PLUS)
		*offset = CHAN_OFFSET_UPPER;
	else if (opc->bw == OPC_BW40MINUS)
		*offset = CHAN_OFFSET_LOWER;

	if (rtw_get_offset_by_bchbw(opc->band, ch, *bw, offset))
		valid = 1;

exit:
	return valid;
}
