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
#define _RTW_CH_UTILS_C_

#include <drv_types.h>

u8 center_ch_2g[CENTER_CH_2G_NUM] = {
/* G00 */1, 2,
/* G01 */3, 4, 5,
/* G02 */6, 7, 8,
/* G03 */9, 10, 11,
/* G04 */12, 13,
/* G05 */14
};

u8 center_ch_2g_40m[CENTER_CH_2G_40M_NUM] = {
	3,
	4,
	5,
	6,
	7,
	8,
	9,
	10,
	11,
};

u8 op_chs_of_cch_2g_40m[CENTER_CH_2G_40M_NUM][2] = {
	{1, 5}, /* 3 */
	{2, 6}, /* 4 */
	{3, 7}, /* 5 */
	{4, 8}, /* 6 */
	{5, 9}, /* 7 */
	{6, 10}, /* 8 */
	{7, 11}, /* 9 */
	{8, 12}, /* 10 */
	{9, 13}, /* 11 */
};

#if CONFIG_IEEE80211_BAND_5GHZ
u8 center_ch_5g_all[CENTER_CH_5G_ALL_NUM] = {
/* G00 */36, 38, 40,
	42,
/* G01 */44, 46, 48,
	/* 50, */
/* G02 */52, 54, 56,
	58,
/* G03 */60, 62, 64,
/* G04 */100, 102, 104,
	106,
/* G05 */108, 110, 112,
	/* 114, */
/* G06 */116, 118, 120,
	122,
/* G07 */124, 126, 128,
/* G08 */132, 134, 136,
	138,
/* G09 */140, 142, 144,
/* G10 */149, 151, 153,
	155,
/* G11 */157, 159, 161,
	/* 163, */
/* G12 */165, 167, 169,
	171,
/* G13 */173, 175, 177
};

u8 center_ch_5g_20m[CENTER_CH_5G_20M_NUM] = {
/* G00 */36, 40,
/* G01 */44, 48,
/* G02 */52, 56,
/* G03 */60, 64,
/* G04 */100, 104,
/* G05 */108, 112,
/* G06 */116, 120,
/* G07 */124, 128,
/* G08 */132, 136,
/* G09 */140, 144,
/* G10 */149, 153,
/* G11 */157, 161,
/* G12 */165, 169,
/* G13 */173, 177
};

u8 center_ch_5g_40m[CENTER_CH_5G_40M_NUM] = {
/* G00 */38,
/* G01 */46,
/* G02 */54,
/* G03 */62,
/* G04 */102,
/* G05 */110,
/* G06 */118,
/* G07 */126,
/* G08 */134,
/* G09 */142,
/* G10 */151,
/* G11 */159,
/* G12 */167,
/* G13 */175
};

u8 op_chs_of_cch_5g_40m[CENTER_CH_5G_40M_NUM][2] = {
	{36, 40}, /* 38 */
	{44, 48}, /* 46 */
	{52, 56}, /* 54 */
	{60, 64}, /* 62 */
	{100, 104}, /* 102 */
	{108, 112}, /* 110 */
	{116, 120}, /* 118 */
	{124, 128}, /* 126 */
	{132, 136}, /* 134 */
	{140, 144}, /* 142 */
	{149, 153}, /* 151 */
	{157, 161}, /* 159 */
	{165, 169}, /* 167 */
	{173, 177}, /* 175 */
};

u8 center_ch_5g_80m[CENTER_CH_5G_80M_NUM] = {
/* G00 ~ G01*/42,
/* G02 ~ G03*/58,
/* G04 ~ G05*/106,
/* G06 ~ G07*/122,
/* G08 ~ G09*/138,
/* G10 ~ G11*/155,
/* G12 ~ G13*/171
};

u8 op_chs_of_cch_5g_80m[CENTER_CH_5G_80M_NUM][4] = {
	{36, 40, 44, 48}, /* 42 */
	{52, 56, 60, 64}, /* 58 */
	{100, 104, 108, 112}, /* 106 */
	{116, 120, 124, 128}, /* 122 */
	{132, 136, 140, 144}, /* 138 */
	{149, 153, 157, 161}, /* 155 */
	{165, 169, 173, 177}, /* 171 */
};

u8 center_ch_5g_160m[CENTER_CH_5G_160M_NUM] = {
/* G00 ~ G03*/50,
/* G04 ~ G07*/114,
/* G10 ~ G13*/163
};

u8 op_chs_of_cch_5g_160m[CENTER_CH_5G_160M_NUM][8] = {
	{36, 40, 44, 48, 52, 56, 60, 64}, /* 50 */
	{100, 104, 108, 112, 116, 120, 124, 128}, /* 114 */
	{149, 153, 157, 161, 165, 169, 173, 177}, /* 163 */
};
#endif /* CONFIG_IEEE80211_BAND_5GHZ */

#if CONFIG_IEEE80211_BAND_6GHZ
u8 center_ch_6g_20m[CENTER_CH_6G_20M_NUM] = {
	1, 5, 9, 13, 17, 21, 25, 29,
	33, 37, 41, 45, 49, 53, 57, 61,
	65, 69, 73, 77, 81, 85, 89, 93,
	97, 101, 105, 109, 113, 117, 121, 125,
	129, 133, 137, 141, 145, 149, 153, 157,
	161, 165, 169, 173, 177, 181, 185, 189,
	193, 197, 201, 205, 209, 213, 217, 221,
	225, 229, 233, 237, 241, 245, 249, 253,
};

u8 center_ch_6g_40m[CENTER_CH_6G_40M_NUM] = {
	3, 11, 19, 27,
	35, 43, 51, 59,
	67, 75, 83, 91,
	99, 107, 115, 123,
	131, 139, 147, 155,
	163, 171, 179, 187,
	195, 203, 211, 219,
	227, 235, 243, 251,
};

u8 op_chs_of_cch_6g_40m[CENTER_CH_6G_40M_NUM][2] = {
	{1, 5}, /* 3 */
	{9, 13}, /* 11 */
	{17, 21}, /* 19 */
	{25, 29}, /* 27 */
	{33, 37}, /* 35 */
	{41, 45}, /* 43 */
	{49, 53}, /* 51 */
	{57, 61}, /* 59 */
	{65, 69}, /* 67 */
	{73, 77}, /* 75 */
	{81, 85}, /* 83 */
	{89, 93}, /* 91 */
	{97, 101}, /* 99 */
	{105, 109}, /* 107 */
	{113, 117}, /* 115 */
	{121, 125}, /* 123 */
	{129, 133}, /* 131 */
	{137, 141}, /* 139 */
	{145, 149}, /* 147 */
	{153, 157}, /* 155 */
	{161, 165}, /* 163 */
	{169, 173}, /* 171 */
	{177, 181}, /* 179 */
	{185, 189}, /* 187 */
	{193, 197}, /* 195 */
	{201, 205}, /* 203 */
	{209, 213}, /* 211 */
	{217, 221}, /* 219 */
	{225, 229}, /* 227 */
	{233, 237}, /* 235 */
	{241, 245}, /* 243 */
	{249, 253}, /* 251 */
};

u8 center_ch_6g_80m[CENTER_CH_6G_80M_NUM] = {
	7, 23,
	39, 55,
	71, 87,
	103, 119,
	135, 151,
	167, 183,
	199, 215,
	231, 247,
};

u8 op_chs_of_cch_6g_80m[CENTER_CH_6G_80M_NUM][4] = {
	{1, 5, 9, 13}, /* 7 */
	{17, 21, 25, 29}, /* 23 */
	{33, 37, 41, 45}, /* 39 */
	{49, 53, 57, 61}, /* 55 */
	{65, 69, 73, 77}, /* 71 */
	{81, 85, 89, 93}, /* 87 */
	{97, 101, 105, 109}, /* 103 */
	{113, 117, 121, 125}, /* 119 */
	{129, 133, 137, 141}, /* 135 */
	{145, 149, 153, 157}, /* 151 */
	{161, 165, 169, 173}, /* 167 */
	{177, 181, 185, 189}, /* 183 */
	{193, 197, 201, 205}, /* 199 */
	{209, 213, 217, 221}, /* 215 */
	{225, 229, 233, 237}, /* 231 */
	{241, 245, 249, 253}, /* 247 */
};

u8 center_ch_6g_160m[CENTER_CH_6G_160M_NUM] = {
	15,
	47,
	79,
	111,
	143,
	175,
	207,
	239,
};

u8 op_chs_of_cch_6g_160m[CENTER_CH_6G_160M_NUM][8] = {
	{1, 5, 9, 13, 17, 21, 25, 29}, /* 15 */
	{33, 37, 41, 45, 49, 53, 57, 61}, /* 47 */
	{65, 69, 73, 77, 81, 85, 89, 93}, /* 79 */
	{97, 101, 105, 109, 113, 117, 121, 125}, /* 111 */
	{129, 133, 137, 141, 145, 149, 153, 157}, /* 143 */
	{161, 165, 169, 173, 177, 181, 185, 189}, /* 175 */
	{193, 197, 201, 205, 209, 213, 217, 221}, /* 207 */
	{225, 229, 233, 237, 241, 245, 249, 253}, /* 239 */
};
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

struct center_chs_ent_t {
	u8 ch_num;
	u8 *chs;
};

struct center_chs_ent_t center_chs_2g_by_bw[] = {
	{CENTER_CH_2G_NUM, center_ch_2g},
	{CENTER_CH_2G_40M_NUM, center_ch_2g_40m},
};

#if CONFIG_IEEE80211_BAND_5GHZ
struct center_chs_ent_t center_chs_5g_by_bw[] = {
	{CENTER_CH_5G_20M_NUM, center_ch_5g_20m},
	{CENTER_CH_5G_40M_NUM, center_ch_5g_40m},
	{CENTER_CH_5G_80M_NUM, center_ch_5g_80m},
	{CENTER_CH_5G_160M_NUM, center_ch_5g_160m},
};
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
struct center_chs_ent_t center_chs_6g_by_bw[] = {
	{CENTER_CH_6G_20M_NUM, center_ch_6g_20m},
	{CENTER_CH_6G_40M_NUM, center_ch_6g_40m},
	{CENTER_CH_6G_80M_NUM, center_ch_6g_80m},
	{CENTER_CH_6G_160M_NUM, center_ch_6g_160m},
};
#endif

/*
 * Get center channel of smaller bandwidth by @param band, @param cch, @param bw, @param offset
 * @band: the given band
 * @cch: the given center channel
 * @bw: the given bandwidth
 * @offset: the given primary SC offset of the given bandwidth
 *
 * return center channel of smaller bandiwdth if valid, or 0
 */
static u8 _rtw_get_scch_by_bcch_offset(enum band_type band, u8 cch, u8 bw, u8 offset)
{
	u8 t_cch = 0;

	if (bw == CHANNEL_WIDTH_20) {
		t_cch = cch;
		goto exit;
	}

	if (offset == CHAN_OFFSET_NO_EXT) {
		rtw_warn_on(1);
		goto exit;
	}

	if (band == BAND_ON_24G) {
		/* 2.4G, 40MHz */
		if (cch >= 3 && cch <= 11 && bw == CHANNEL_WIDTH_40) {
			t_cch = (offset == CHAN_OFFSET_LOWER) ? cch + 2 : cch - 2;
			goto exit;
		}
	}

#if CONFIG_IEEE80211_BAND_5GHZ
	else if (band == BAND_ON_5G) {
		/* 5G, 160MHz */
		if (cch >= 50 && cch <= 163 && bw == CHANNEL_WIDTH_160) {
			t_cch = (offset == CHAN_OFFSET_LOWER) ? cch + 8 : cch - 8;
			goto exit;

		/* 5G, 80MHz */
		} else if (cch >= 42 && cch <= 171 && bw == CHANNEL_WIDTH_80) {
			t_cch = (offset == CHAN_OFFSET_LOWER) ? cch + 4 : cch - 4;
			goto exit;

		/* 5G, 40MHz */
		} else if (cch >= 38 && cch <= 175 && bw == CHANNEL_WIDTH_40) {
			t_cch = (offset == CHAN_OFFSET_LOWER) ? cch + 2 : cch - 2;
			goto exit;

		}
	}
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
	else if (band == BAND_ON_6G) {
		/* 6G, 160MHz */
		if (cch >= 15 && cch <= 239 && bw == CHANNEL_WIDTH_160) {
			t_cch = (offset == CHAN_OFFSET_LOWER) ? cch + 8 : cch - 8;
			goto exit;

		/* 6G, 80MHz */
		} else if (cch >= 7 && cch <= 247 && bw == CHANNEL_WIDTH_80) {
			t_cch = (offset == CHAN_OFFSET_LOWER) ? cch + 4 : cch - 4;
			goto exit;

		/* 6G, 40MHz */
		} else if (cch >= 3 && cch <= 251 && bw == CHANNEL_WIDTH_40) {
			t_cch = (offset == CHAN_OFFSET_LOWER) ? cch + 2 : cch - 2;
			goto exit;

		}
	}
#endif

	rtw_warn_on(1);

exit:
	return t_cch;
}

#if CONFIG_ALLOW_FUNC_2G_5G_ONLY
RTW_FUNC_2G_5G_ONLY u8 rtw_get_scch_by_cch_offset(u8 cch, u8 bw, u8 offset)
{
	return _rtw_get_scch_by_bcch_offset(rtw_is_2g_ch(cch) ? BAND_ON_24G : BAND_ON_5G, cch, bw, offset);
}

RTW_FUNC_2G_5G_ONLY u8 rtw_get_scch_by_cch_opch(u8 cch, u8 bw, u8 opch)
{
	u8 offset = CHAN_OFFSET_NO_EXT;

	if (opch > cch)
		offset = CHAN_OFFSET_LOWER;
	else if (opch < cch)
		offset = CHAN_OFFSET_UPPER;

	return rtw_get_scch_by_cch_offset(cch, bw, offset);
}
#endif

u8 rtw_get_scch_by_bcch_offset(enum band_type band, u8 cch, u8 bw, u8 offset)
{
	return _rtw_get_scch_by_bcch_offset(band, cch, bw, offset);
}

/*
 * Get center channel of smaller bandwidth by @param band, @param cch, @param bw, @param opch
 * @band: the given band
 * @cch: the given center channel
 * @bw: the given bandwidth
 * @opch: the given operating channel
 *
 * return center channel of smaller bandiwdth if valid, or 0
 */
u8 rtw_get_scch_by_bcch_opch(enum band_type band, u8 cch, u8 bw, u8 opch)
{
	u8 offset = CHAN_OFFSET_NO_EXT;

	if (opch > cch)
		offset = CHAN_OFFSET_LOWER;
	else if (opch < cch)
		offset = CHAN_OFFSET_UPPER;

	return rtw_get_scch_by_bcch_offset(band, cch, bw, offset);
}

struct op_chs_ent_t {
	u8 ch_num;
	u8 *chs;
};

struct op_chs_ent_t op_chs_of_cch_2g_by_bw[] = {
	{1, center_ch_2g},
	{2, (u8 *)op_chs_of_cch_2g_40m},
};

#if CONFIG_IEEE80211_BAND_5GHZ
struct op_chs_ent_t op_chs_of_cch_5g_by_bw[] = {
	{1, center_ch_5g_20m},
	{2, (u8 *)op_chs_of_cch_5g_40m},
	{4, (u8 *)op_chs_of_cch_5g_80m},
	{8, (u8 *)op_chs_of_cch_5g_160m},
};
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
struct op_chs_ent_t op_chs_of_cch_6g_by_bw[] = {
	{1, center_ch_6g_20m},
	{2, (u8 *)op_chs_of_cch_6g_40m},
	{4, (u8 *)op_chs_of_cch_6g_80m},
	{8, (u8 *)op_chs_of_cch_6g_160m},
};
#endif

u8 center_chs_2g_num(u8 bw)
{
	if (bw > CHANNEL_WIDTH_40)
		return 0;

	return center_chs_2g_by_bw[bw].ch_num;
}

u8 center_chs_2g(u8 bw, u8 id)
{
	if (bw > CHANNEL_WIDTH_40)
		return 0;

	if (id >= center_chs_2g_num(bw))
		return 0;

	return center_chs_2g_by_bw[bw].chs[id];
}

#if CONFIG_IEEE80211_BAND_5GHZ
u8 center_chs_5g_num(u8 bw)
{
	if (bw > CHANNEL_WIDTH_160)
		return 0;

	return center_chs_5g_by_bw[bw].ch_num;
}

u8 center_chs_5g(u8 bw, u8 id)
{
	if (bw > CHANNEL_WIDTH_160)
		return 0;

	if (id >= center_chs_5g_num(bw))
		return 0;

	return center_chs_5g_by_bw[bw].chs[id];
}
#endif /* CONFIG_IEEE80211_BAND_5GHZ */

#if CONFIG_IEEE80211_BAND_6GHZ
u8 center_chs_6g_num(u8 bw)
{
	if (bw > CHANNEL_WIDTH_160)
		return 0;

	return center_chs_6g_by_bw[bw].ch_num;
}

u8 center_chs_6g(u8 bw, u8 id)
{
	if (bw > CHANNEL_WIDTH_160)
		return 0;

	if (id >= center_chs_6g_num(bw))
		return 0;

	return center_chs_6g_by_bw[bw].chs[id];
}
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

u8 (*center_chs_num_of_band[BAND_MAX])(u8 bw) = {
	[BAND_ON_24G] = center_chs_2g_num,
#if CONFIG_IEEE80211_BAND_5GHZ
	[BAND_ON_5G] = center_chs_5g_num,
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	[BAND_ON_6G] = center_chs_6g_num,
#endif
};

u8 (*center_chs_of_band[BAND_MAX])(u8 bw, u8 id) = {
	[BAND_ON_24G] = center_chs_2g,
#if CONFIG_IEEE80211_BAND_5GHZ
	[BAND_ON_5G] = center_chs_5g,
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	[BAND_ON_6G] = center_chs_6g,
#endif
};

/*
 * Get available op channels by @param cch, @param bw
 * @cch: the given center channel
 * @bw: the given bandwidth
 * @op_chs: the pointer to return pointer of op channel array
 * @op_ch_num: the pointer to return pointer of op channel number
 *
 * return valid (1) or not (0)
 */
static u8 _rtw_get_op_chs_by_bcch_bw(enum band_type band, u8 cch, u8 bw, u8 **op_chs, u8 *op_ch_num)
{
	int i;
	struct center_chs_ent_t *c_chs_ent = NULL;
	struct op_chs_ent_t *op_chs_ent = NULL;
	u8 valid = 1;

	if (band == BAND_ON_24G
		&& bw <= CHANNEL_WIDTH_40
	) {
		c_chs_ent = &center_chs_2g_by_bw[bw];
		op_chs_ent = &op_chs_of_cch_2g_by_bw[bw];

#if CONFIG_IEEE80211_BAND_5GHZ
	} else if (band == BAND_ON_5G
		&& bw <= CHANNEL_WIDTH_160
	) {
		c_chs_ent = &center_chs_5g_by_bw[bw];
		op_chs_ent = &op_chs_of_cch_5g_by_bw[bw];
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
	} else if (band == BAND_ON_6G
		&& bw <= CHANNEL_WIDTH_160
	) {
		c_chs_ent = &center_chs_6g_by_bw[bw];
		op_chs_ent = &op_chs_of_cch_6g_by_bw[bw];
#endif

	} else {
		valid = 0;
		goto exit;
	}

	for (i = 0; i < c_chs_ent->ch_num; i++)
		if (cch == *(c_chs_ent->chs + i))
			break;

	if (i == c_chs_ent->ch_num) {
		valid = 0;
		goto exit;
	}

	*op_chs = op_chs_ent->chs + op_chs_ent->ch_num * i;
	*op_ch_num = op_chs_ent->ch_num;

exit:
	return valid;
}

#if CONFIG_ALLOW_FUNC_2G_5G_ONLY
RTW_FUNC_2G_5G_ONLY u8 rtw_get_op_chs_by_cch_bw(u8 cch, u8 bw, u8 **op_chs, u8 *op_ch_num)
{
	return _rtw_get_op_chs_by_bcch_bw(rtw_is_2g_ch(cch) ? BAND_ON_24G : BAND_ON_5G, cch, bw, op_chs, op_ch_num);
}
#endif

u8 rtw_get_op_chs_by_bcch_bw(enum band_type band, u8 cch, u8 bw, u8 **op_chs, u8 *op_ch_num)
{
	return _rtw_get_op_chs_by_bcch_bw(band, cch, bw, op_chs, op_ch_num);
}

static u8 rtw_get_offsets_by_chbw_2g(u8 ch, u8 bw, u8 *r_offset, u8 *r_offset_num)
{
	u8 valid = 1;
	u8 offset[2] = {CHAN_OFFSET_NO_EXT, CHAN_OFFSET_NO_EXT};
	u8 offset_num = 1;

	if (bw >= CHANNEL_WIDTH_80
		|| ch < 1 || ch > 14
	) {
		valid = 0;
		goto exit;
	}

	if (bw == CHANNEL_WIDTH_20)
		goto exit;

	if (ch >= 1 && ch <= 4)
		offset[0] = CHAN_OFFSET_UPPER;
	else if (ch >= 5 && ch <= 9) {
		if (!r_offset_num) {
			/* return single offset */
			if (r_offset && (*r_offset == CHAN_OFFSET_UPPER || *r_offset == CHAN_OFFSET_LOWER))
				offset[0] = *r_offset; /* both lower and upper is valid, obey input value */
			else
				offset[0] = CHAN_OFFSET_LOWER; /* default use primary upper */
		} else {
			offset_num = 2;
			offset[0] = CHAN_OFFSET_LOWER;
			offset[1] = CHAN_OFFSET_UPPER;
		}
	} else if (ch >= 10 && ch <= 13)
		offset[0] = CHAN_OFFSET_LOWER;
	else {
		valid = 0; /* ch14 doesn't support 40MHz bandwidth */
		goto exit;
	}

exit:
	if (valid) {
		if (r_offset) {
			*r_offset = offset[0];
			if (offset_num == 2)
				*(r_offset + 1) = offset[1];
		}
		if (r_offset_num)
			*r_offset_num = offset_num;
	}
	return valid;
}

#if CONFIG_IEEE80211_BAND_5GHZ
static u8 rtw_get_offset_by_chbw_5g(u8 ch, u8 bw, u8 *r_offset)
{
	u8 valid = 1;
	u8 offset = CHAN_OFFSET_NO_EXT;

	if (ch < 36 || ch > 177) {
		valid = 0;
		goto exit;
	}

	switch (ch) {
	case 36:
	case 44:
	case 52:
	case 60:
	case 100:
	case 108:
	case 116:
	case 124:
	case 132:
	case 140:
	case 149:
	case 157:
	case 165:
	case 173:
		if (bw >= CHANNEL_WIDTH_40 && bw <= CHANNEL_WIDTH_160)
			offset = CHAN_OFFSET_UPPER;
		break;
	case 40:
	case 48:
	case 56:
	case 64:
	case 104:
	case 112:
	case 120:
	case 128:
	case 136:
	case 144:
	case 153:
	case 161:
	case 169:
	case 177:
		if (bw >= CHANNEL_WIDTH_40 && bw <= CHANNEL_WIDTH_160)
			offset = CHAN_OFFSET_LOWER;
		break;
	default:
		valid = 0;
		break;
	}

exit:
	if (valid && r_offset)
		*r_offset = offset;
	return valid;
}
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
static u8 rtw_get_offset_by_chbw_6g(u8 ch, u8 bw, u8 *r_offset)
{
	if (ch >= 1 && ch <= 253) {
		u8 mod8 = ch % 8;

		if (mod8 == 1) {
			if (bw == CHANNEL_WIDTH_20)
				*r_offset = CHAN_OFFSET_NO_EXT;
			else
				*r_offset = CHAN_OFFSET_UPPER;
			return 1;
		}
		if (mod8 == 5) {
			if (bw == CHANNEL_WIDTH_20)
				*r_offset = CHAN_OFFSET_NO_EXT;
			else
				*r_offset = CHAN_OFFSET_LOWER;
			return 1;
		}
	}

	return 0;
}
#endif

u8 rtw_get_offset_by_bchbw(enum band_type band, u8 ch, u8 bw, u8 *r_offset)
{
	if (band == BAND_ON_24G) {
		u8 offset[1] = {r_offset ? *r_offset : CHAN_OFFSET_NO_EXT};
		u8 ret = rtw_get_offsets_by_chbw_2g(ch, bw, offset, NULL);

		if (ret && r_offset)
			*r_offset = offset[0];
		return ret;
	}
#if CONFIG_IEEE80211_BAND_5GHZ
	else if (band == BAND_ON_5G)
		return rtw_get_offset_by_chbw_5g(ch, bw, r_offset);
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	else if (band == BAND_ON_6G)
		return rtw_get_offset_by_chbw_6g(ch, bw, r_offset);
#endif
	return 0;
}

#if CONFIG_ALLOW_FUNC_2G_5G_ONLY
RTW_FUNC_2G_5G_ONLY u8 rtw_get_offset_by_chbw(u8 ch, u8 bw, u8 *r_offset)
{
	return rtw_get_offset_by_bchbw(rtw_is_2g_ch(ch) ? BAND_ON_24G : BAND_ON_5G, ch, bw, r_offset);
}
#endif

u8 rtw_get_offsets_by_bchbw(enum band_type band, u8 ch, u8 bw, u8 *r_offset, u8 *r_offset_num)
{
	u8 ret = 0;

	if (band == BAND_ON_24G)
		ret = rtw_get_offsets_by_chbw_2g(ch, bw, r_offset, r_offset_num);
#if CONFIG_IEEE80211_BAND_5GHZ
	else if (band == BAND_ON_5G) {
		ret = rtw_get_offset_by_chbw_5g(ch, bw, r_offset);
		if (ret && r_offset_num)
			*r_offset_num = 1;
	}
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	else if (band == BAND_ON_6G) {
		ret = rtw_get_offset_by_chbw_6g(ch, bw, r_offset);
		if (ret && r_offset_num)
			*r_offset_num = 1;
	}
#endif
	return ret;
}

static u8 rtw_get_center_ch_2g(u8 ch, u8 bw, u8 offset)
{
	u8 cch = ch;

	if (bw == CHANNEL_WIDTH_80) {
		/* special case for 2.4G */
		cch = 7;
	} else if (bw == CHANNEL_WIDTH_40) {
		if (offset == CHAN_OFFSET_UPPER)
			cch = ch + 2;
		else if (offset == CHAN_OFFSET_LOWER)
			cch = ch - 2;

	} else if (bw == CHANNEL_WIDTH_20
		|| bw == CHANNEL_WIDTH_10
		|| bw == CHANNEL_WIDTH_5
	)
		; /* same as ch */
	else
		rtw_warn_on(1);

	return cch;
}

#if CONFIG_IEEE80211_BAND_5GHZ
static u8 rtw_get_center_ch_5g(u8 ch, u8 bw, u8 offset)
{
	u8 cch = ch;

	if (bw == CHANNEL_WIDTH_160) {
		if (ch % 4 == 0) {
			if (ch >= 36 && ch <= 64)
				cch = 50;
			else if (ch >= 100 && ch <= 128)
				cch = 114;
		} else if (ch % 4 == 1) {
			if (ch >= 149 && ch <= 177)
				cch = 163;
		}

	} else if (bw == CHANNEL_WIDTH_80) {
		if (ch % 4 == 0) {
			if (ch >= 36 && ch <= 48)
				cch = 42;
			else if (ch >= 52 && ch <= 64)
				cch = 58;
			else if (ch >= 100 && ch <= 112)
				cch = 106;
			else if (ch >= 116 && ch <= 128)
				cch = 122;
			else if (ch >= 132 && ch <= 144)
				cch = 138;
		} else if (ch % 4 == 1) {
			if (ch >= 149 && ch <= 161)
				cch = 155;
			else if (ch >= 165 && ch <= 177)
				cch = 171;
		}

	} else if (bw == CHANNEL_WIDTH_40) {
		if (offset == CHAN_OFFSET_UPPER)
			cch = ch + 2;
		else if (offset == CHAN_OFFSET_LOWER)
			cch = ch - 2;

	} else if (bw == CHANNEL_WIDTH_20
		|| bw == CHANNEL_WIDTH_10
		|| bw == CHANNEL_WIDTH_5
	)
		; /* same as ch */
	else
		rtw_warn_on(1);

	return cch;
}
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
static u8 rtw_get_center_ch_6g(u8 ch, u8 bw, u8 offset)
{
	static const u8 start[CHANNEL_WIDTH_MAX] = {
		[CHANNEL_WIDTH_40] = 3,
		[CHANNEL_WIDTH_80] = 7,
		[CHANNEL_WIDTH_160] = 15,
	};
	static const u8 shift[CHANNEL_WIDTH_MAX] = {
		[CHANNEL_WIDTH_40] = 3,
		[CHANNEL_WIDTH_80] = 4,
		[CHANNEL_WIDTH_160] = 5,
	};
	u8 cch = ch;

	if (bw == CHANNEL_WIDTH_20 || bw == CHANNEL_WIDTH_10 || bw == CHANNEL_WIDTH_5)
		goto exit;

	if (bw > CHANNEL_WIDTH_160) {
		rtw_warn_on(1);
		goto exit;
	}

	cch = (((ch - 1) >> shift[bw]) << shift[bw]) + start[bw];

exit:
	return cch;
}
#endif

u8 rtw_get_center_ch_by_band(enum band_type band, u8 ch, u8 bw, u8 offset)
{
	if (band == BAND_ON_24G)
		return rtw_get_center_ch_2g(ch, bw, offset);
#if CONFIG_IEEE80211_BAND_5GHZ
	else if (band == BAND_ON_5G)
		return rtw_get_center_ch_5g(ch, bw, offset);
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	else if (band == BAND_ON_6G)
		return rtw_get_center_ch_6g(ch, bw, offset);
#endif
	return 0;
}

#if CONFIG_ALLOW_FUNC_2G_5G_ONLY
RTW_FUNC_2G_5G_ONLY u8 rtw_get_center_ch(u8 ch, u8 bw, u8 offset)
{
	return rtw_get_center_ch_by_band(rtw_is_2g_ch(ch) ? BAND_ON_24G : BAND_ON_5G, ch, bw, offset);
}
#endif

/**
 * rtw_is_bchbw_grouped - test if the two ch settings can be grouped together
 * @band_a: band of set a
 * @ch_a: ch of set a
 * @bw_a: bw of set a
 * @offset_a: offset of set a
 * @band_b: band of set b
 * @ch_b: ch of set b
 * @bw_b: bw of set b
 * @offset_b: offset of set b
 */
bool _rtw_is_bchbw_grouped(enum band_type band_a, u8 ch_a, u8 bw_a, u8 offset_a
	, enum band_type band_b, u8 ch_b, u8 bw_b, u8 offset_b)
{
	bool is_grouped = _FALSE;

	if (band_a != band_b || ch_a != ch_b) {
		/* band/ch is different */
		goto exit;
	}

	if ((bw_a == CHANNEL_WIDTH_40 || bw_a == CHANNEL_WIDTH_80 || bw_a == CHANNEL_WIDTH_160)
		   && (bw_b == CHANNEL_WIDTH_40 || bw_b == CHANNEL_WIDTH_80 || bw_b == CHANNEL_WIDTH_160)
	) {
		if (offset_a != offset_b)
			goto exit;
	}

	is_grouped = _TRUE;

exit:
	return is_grouped;
}

#if CONFIG_ALLOW_FUNC_2G_5G_ONLY
RTW_FUNC_2G_5G_ONLY bool rtw_is_chbw_grouped(u8 ch_a, u8 bw_a, u8 offset_a, u8 ch_b, u8 bw_b, u8 offset_b)
{
	enum band_type band = BAND_ON_24G; /* unknown band, use same band for a & b */

	return _rtw_is_bchbw_grouped(band, ch_a, bw_a, offset_a, band, ch_b, bw_b, offset_b);
}
#endif

bool rtw_is_bchbw_grouped(enum band_type band_a, u8 ch_a, u8 bw_a, u8 offset_a
	, enum band_type band_b, u8 ch_b, u8 bw_b, u8 offset_b)
{
	return _rtw_is_bchbw_grouped(band_a, ch_a, bw_a, offset_a, band_b, ch_b, bw_b, offset_b);
}

/**
 * rtw_sync_bchbw - obey g_band, g_ch, adjust g_bw, g_offset, bw, offset
 * @req_band: pointer of the request band, may be modified further
 * @req_ch: pointer of the request ch, may be modified further
 * @req_bw: pointer of the request bw, may be modified further
 * @req_offset: pointer of the request offset, may be modified further
 * @g_band: pointer of the ongoing group band
 * @g_ch: pointer of the ongoing group ch
 * @g_bw: pointer of the ongoing group bw, may be modified further
 * @g_offset: pointer of the ongoing group offset, may be modified further
 */
void _rtw_sync_bchbw(enum band_type *req_band, u8 *req_ch, u8 *req_bw, u8 *req_offset
	, enum band_type *g_band, u8 *g_ch, u8 *g_bw, u8 *g_offset)
{
	*req_band = *g_band;
	*req_ch = *g_ch;

	if (*req_bw == CHANNEL_WIDTH_80 && *g_band == BAND_ON_24G) {
		/*2.4G ch, downgrade to 40Mhz */
		*req_bw = CHANNEL_WIDTH_40;
	}

	switch (*req_bw) {
	case CHANNEL_WIDTH_160:
	case CHANNEL_WIDTH_80:
	case CHANNEL_WIDTH_40:
		if (*g_bw == CHANNEL_WIDTH_40 || *g_bw == CHANNEL_WIDTH_80 || *g_bw == CHANNEL_WIDTH_160)
			*req_offset = *g_offset;
		else if (*g_bw == CHANNEL_WIDTH_20)
			rtw_get_offset_by_bchbw(*req_band, *req_ch, *req_bw, req_offset);

		if (*req_offset == CHAN_OFFSET_NO_EXT) {
			RTW_ERR("%s req %s BW without offset, down to 20MHz\n", __func__, ch_width_str(*req_bw));
			rtw_warn_on(1);
			*req_bw = CHANNEL_WIDTH_20;
		}
		break;
	case CHANNEL_WIDTH_20:
		*req_offset = CHAN_OFFSET_NO_EXT;
		break;
	default:
		RTW_ERR("%s req unsupported BW:%u\n", __func__, *req_bw);
		rtw_warn_on(1);
	}

	if (*req_bw > *g_bw) {
		*g_bw = *req_bw;
		*g_offset = *req_offset;
	}
}

#if CONFIG_ALLOW_FUNC_2G_5G_ONLY
RTW_FUNC_2G_5G_ONLY void rtw_sync_chbw(u8 *req_ch, u8 *req_bw, u8 *req_offset, u8 *g_ch, u8 *g_bw, u8 *g_offset)
{
	enum band_type band = rtw_is_2g_ch(*g_ch) ? BAND_ON_24G : BAND_ON_5G; /* follow g_ch's band */

	_rtw_sync_bchbw(&band, req_ch, req_bw, req_offset, &band, g_ch, g_bw, g_offset);
}
#endif

void rtw_sync_bchbw(enum band_type *req_band, u8 *req_ch, u8 *req_bw, u8 *req_offset
	, enum band_type *g_band, u8 *g_ch, u8 *g_bw, u8 *g_offset)
{
	_rtw_sync_bchbw(req_band, req_ch, req_bw, req_offset, g_band, g_ch, g_bw, g_offset);
}

static int rtw_ch2freq_2g(int ch)
{
	if (ch >= 1 && ch <= 14) {
		if (ch == 14)
			return 2484;
		else if (ch < 14)
			return 2407 + ch * 5;
	}

	return 0; /* not supported */
}

#if CONFIG_IEEE80211_BAND_5GHZ
static int rtw_ch2freq_5g(int ch)
{
	if (ch >= 36 && ch <= 177)
		return 5000 + ch * 5;

	return 0; /* not supported */
}
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
static int rtw_ch2freq_6g(int ch)
{
	if (ch >= 1 && ch <= 253)
		return 5950 + ch * 5;

	return 0; /* not supported */
}
#endif

int rtw_bch2freq(enum band_type band, int ch)
{
	if (band == BAND_ON_24G)
		return rtw_ch2freq_2g(ch);
#if CONFIG_IEEE80211_BAND_5GHZ
	else if (band == BAND_ON_5G)
		return rtw_ch2freq_5g(ch);
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	else if (band == BAND_ON_6G)
		return rtw_ch2freq_6g(ch);
#endif
	return 0;
}

#if CONFIG_ALLOW_FUNC_2G_5G_ONLY
RTW_FUNC_2G_5G_ONLY int rtw_ch2freq(int ch)
{
	return rtw_bch2freq(rtw_is_2g_ch(ch) ? BAND_ON_24G : BAND_ON_5G, ch);
}
#endif

int rtw_freq2ch(int freq)
{
	/* see 802.11 17.3.8.3.2 and Annex J */
	if (freq == 2484)
		return 14;
	else if (freq < 2484)
		return (freq - 2407) / 5;
	else if (freq >= 4910 && freq <= 4980)
		return (freq - 4000) / 5;
	else if (freq >= 5000 && freq < 5950)
		return (freq - 5000) / 5;
	else if (freq >= 5950 && freq <= 7215)
		return (freq - 5950) / 5;
	else if (freq >= 58320 && freq <= 64800)
		return (freq - 56160) / 2160;
	else
		return 0;
}

enum band_type rtw_freq2band(int freq)
{
	if (freq <= 2484)
		return BAND_ON_24G;
#if CONFIG_IEEE80211_BAND_5GHZ
	else if (freq >= 5000 && freq < 5950)
		return BAND_ON_5G;
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	else if (freq >= 5950 && freq <= 7215)
		return BAND_ON_6G;
#endif
	else
		return BAND_MAX;
}

enum channel_width rtw_frange_to_bw(u32 hi, u32 lo)
{
	u32 width = hi - lo;

	switch (width) {
	case 160:
		return CHANNEL_WIDTH_160;
	case 80:
		return CHANNEL_WIDTH_80;
	case 40:
		return CHANNEL_WIDTH_40;
	case 20:
		return CHANNEL_WIDTH_20;
	case 10:
		return CHANNEL_WIDTH_10;
	case 5:
		return CHANNEL_WIDTH_5;
	default:
		return CHANNEL_WIDTH_MAX;
	}
}

bool rtw_freq_consecutive(int a, int b)
{
	enum band_type band_a, band_b;

	band_a = rtw_freq2band(a);
	if (band_a == BAND_MAX)
		return 0;
	band_b = rtw_freq2band(b);
	if (band_b == BAND_MAX || band_a != band_b)
		return 0;

	switch (band_a) {
	case BAND_ON_24G:
		return rtw_abs(a - b) == 5;
	case BAND_ON_5G:
#if CONFIG_IEEE80211_BAND_6GHZ
	case BAND_ON_6G:
#endif
		return rtw_abs(a - b) == 20;
	default:
		return 0;
	}
}

bool rtw_bcchbw_to_freq_range(enum band_type band, u8 c_ch, u8 bw, u32 *hi, u32 *lo)
{
	u32 freq;
	u32 hi_ret = 0, lo_ret = 0;
	bool valid = _FALSE;

	if (hi)
		*hi = 0;
	if (lo)
		*lo = 0;

	freq = rtw_bch2freq(band, c_ch);

	if (!freq) {
		rtw_warn_on(1);
		goto exit;
	}

	if (bw == CHANNEL_WIDTH_160) {
		hi_ret = freq + 80;
		lo_ret = freq - 80;
	} else if (bw == CHANNEL_WIDTH_80) {
		hi_ret = freq + 40;
		lo_ret = freq - 40;
	} else if (bw == CHANNEL_WIDTH_40) {
		hi_ret = freq + 20;
		lo_ret = freq - 20;
	} else if (bw == CHANNEL_WIDTH_20) {
		hi_ret = freq + 10;
		lo_ret = freq - 10;
	} else
		rtw_warn_on(1);

	if (hi)
		*hi = hi_ret;
	if (lo)
		*lo = lo_ret;

	valid = _TRUE;

exit:
	return valid;
}

bool rtw_bchbw_to_freq_range(enum band_type band, u8 ch, u8 bw, u8 offset, u32 *hi, u32 *lo)
{
	return rtw_bcchbw_to_freq_range(band
		, rtw_get_center_ch_by_band(band, ch, bw, offset)
		, bw, hi, lo);
}

#if CONFIG_ALLOW_FUNC_2G_5G_ONLY
RTW_FUNC_2G_5G_ONLY bool rtw_chbw_to_freq_range(u8 ch, u8 bw, u8 offset, u32 *hi, u32 *lo)
{
	return rtw_bchbw_to_freq_range(rtw_is_2g_ch(ch) ? BAND_ON_24G : BAND_ON_5G, ch, bw, offset, hi, lo);
}
#endif
