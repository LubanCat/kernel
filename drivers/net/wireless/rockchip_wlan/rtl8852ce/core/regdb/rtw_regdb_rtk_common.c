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
#define _RTW_REGDB_RTK_COMMON_C_

#include "rtw_regdb.h"

static u8 rtk_regdb_get_default_regd_2g(u8 id)
{
	if (id < ARRAY_SIZE(RTW_ChannelPlanMap))
		return RTW_ChannelPlanMap[id].regd_2g;
	return RTW_REGD_NA;
}

#if CONFIG_IEEE80211_BAND_5GHZ
static u8 rtk_regdb_get_default_regd_5g(u8 id)
{
	if (id < ARRAY_SIZE(RTW_ChannelPlanMap))
		return RTW_ChannelPlanMap[id].regd_5g;
	return RTW_REGD_NA;
}
#endif

static bool rtk_regdb_is_domain_code_valid(u8 id)
{
	if (id < ARRAY_SIZE(RTW_ChannelPlanMap)) {
		const struct chplan_ent_t *chplan_map = &RTW_ChannelPlanMap[id];

		if (chplan_map->chd_2g != RTW_CHD_2G_INVALID
			#if CONFIG_IEEE80211_BAND_5GHZ
			&& chplan_map->chd_5g != RTW_CHD_5G_INVALID
			#endif
		)
			return true;
	}

	return false;
}

static bool rtk_regdb_domain_get_ch(u8 id, u32 ch, u8 *flags)
{
#if CONFIG_IEEE80211_BAND_5GHZ
	if (ch > 14) {
		u8 chd_5g = RTW_ChannelPlanMap[id].chd_5g;
		#if !CH_MAP_5G
		u8 index;

		for (index = 0; index < CH_LIST_LEN(rtw_channel_def_5g[chd_5g]); index++)
			if (CH_LIST_CH(rtw_channel_def_5g[chd_5g], index) == ch)
				goto chk_flags_5g;
		#else
		if (CH_TO_CHM_5G(ch) & rtw_channel_def_5g[chd_5g].ch_map)
			goto chk_flags_5g;
		#endif
		return false;

chk_flags_5g:
		if (flags) {
			u8 attrib = CH_LIST_ATTRIB_5G(rtw_channel_def_5g[chd_5g]);

			*flags = 0;

			if ((rtw_is_5g_band1(ch) && (attrib & CLA_5G_B1_PASSIVE)) /* band1 passive */
				|| (rtw_is_5g_band2(ch) && (attrib & CLA_5G_B2_PASSIVE)) /* band2 passive */
				|| (rtw_is_5g_band3(ch) && (attrib & CLA_5G_B3_PASSIVE)) /* band3 passive */
				|| (rtw_is_5g_band4(ch) && (attrib & CLA_5G_B4_PASSIVE)) /* band4 passive */
			)
				*flags |= RTW_CHF_NO_IR;

			if ((rtw_is_5g_band2(ch) && (attrib & CLA_5G_B2_DFS))
				|| (rtw_is_5g_band3(ch) && (attrib & CLA_5G_B3_DFS))
				|| (rtw_is_5g_band4(ch) && (attrib & CLA_5G_B4_DFS)))
				*flags |= RTW_CHF_DFS;
		}
	} else
#endif /* CONFIG_IEEE80211_BAND_5GHZ */
	 {
		u8 chd_2g = RTW_ChannelPlanMap[id].chd_2g;
		#if !CH_MAP_2G
		u8 index;

		for (index = 0; index < CH_LIST_LEN(rtw_channel_def_2g[chd_2g]); index++)
			if (CH_LIST_CH(rtw_channel_def_2g[chd_2g], index) == ch)
				goto chk_flags_2g;
		#else
		if (CH_TO_CHM_2G(ch) & rtw_channel_def_2g[chd_2g].ch_map)
			goto chk_flags_2g;
		#endif
		return false;

chk_flags_2g:
		if (flags) {
			u8 attrib = CH_LIST_ATTRIB_2G(rtw_channel_def_2g[chd_2g]);

			*flags = 0;

			if (ch >= 12 && ch <= 14 && (attrib & CLA_2G_12_14_PASSIVE))
				*flags |= RTW_CHF_NO_IR;
		}
	}

	return true;
}

#if CONFIG_IEEE80211_BAND_6GHZ
static u8 rtk_regdb_get_default_regd_6g(u8 id)
{
	if (id < ARRAY_SIZE(rtw_chplan_6g_map))
		return rtw_chplan_6g_map[id].regd;
	return RTW_REGD_NA;
}

static bool rtk_regdb_is_domain_code_6g_valid(u8 id)
{
	if (id < ARRAY_SIZE(rtw_chplan_6g_map)) {
		const struct chplan_6g_ent_t *chplan_map = &rtw_chplan_6g_map[id];

		if (chplan_map->chd != RTW_CHD_6G_INVALID)
			return true;
	}

	return false;
}

static bool rtk_regdb_domain_6g_get_ch(u8 id, u32 ch, u8 *flags)
{
	u8 chd_6g = rtw_chplan_6g_map[id].chd;
#if !CH_MAP_6G
	u8 index;

	for (index = 0; index < CH_LIST_LEN(rtw_channel_def_6g[chd_6g]); index++)
		if (CH_LIST_CH(rtw_channel_def_6g[chd_6g], index) == ch)
			goto chk_flags;
#else
	if (CH_TO_CHM_6G(ch) & rtw_channel_def_6g[chd_6g].ch_map)
		goto chk_flags;
#endif
	return false;

chk_flags:
	if (flags) {
		u8 attrib = CH_LIST_ATTRIB_6G(rtw_channel_def_6g[chd_6g]);

		*flags = 0;

		if ((rtw_is_6g_band1(ch) && (attrib & CLA_6G_B1_PASSIVE)) /* band1 passive */
			|| (rtw_is_6g_band2(ch) && (attrib & CLA_6G_B2_PASSIVE)) /* band2 passive */
			|| (rtw_is_6g_band3(ch) && (attrib & CLA_6G_B3_PASSIVE)) /* band3 passive */
			|| (rtw_is_6g_band4(ch) && (attrib & CLA_6G_B4_PASSIVE)) /* band4 passive */
		)
			*flags |= RTW_CHF_NO_IR;
	}

	return true;
}
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

#if CC_2D_OFFSET
static_assert(ARRAY_SIZE(country_chplan_map) <= U8_MAX);
#define CC_2D_NO_FOUND U8_MAX
static u8 cc_2d_offset[26][26];

static void build_cc_2d_offset(void)
{
	const struct country_chplan *map = country_chplan_map;
	u8 code[2];
	u8 i;

	_rtw_memset(cc_2d_offset, CC_2D_NO_FOUND, sizeof(cc_2d_offset));

	for (i = 0; i < ARRAY_SIZE(country_chplan_map); i++) {
		code[0] = map[i].alpha2[0] - 'A';
		code[1] = map[i].alpha2[1] - 'A';
		if (code[0] >= 26 || code[1] >= 26) {
			RTW_WARN("%s \"%c%c\" is not valid alpha2\n"
				, __func__, map[i].alpha2[0], map[i].alpha2[1]);
			continue;
		}
		if (cc_2d_offset[code[0]][code[1]] != CC_2D_NO_FOUND)
			RTW_WARN("%s \"%c%c\" has redundent entry\n"
				, __func__, map[i].alpha2[0], map[i].alpha2[1]);
		cc_2d_offset[code[0]][code[1]] = i;
	}
}
#endif

static bool rtk_regdb_get_chplan_from_alpha2(const char *alpha2, struct country_chplan *ent)
{
#if CC_2D_OFFSET
	u8 code[2] = {alpha2[0] - 'A', alpha2[1] - 'A'};
	u8 offset;

	if (code[0] >= 26 || code[1] >= 26)
		return false;

	offset = cc_2d_offset[code[0]][code[1]];

	if (offset == CC_2D_NO_FOUND)
		return false;

	if (ent)
		_rtw_memcpy(ent, &country_chplan_map[cc_2d_offset[code[0]][code[1]]], sizeof(*ent));
	return true;
#else
	const struct country_chplan *map = country_chplan_map;
	int i;

	for (i = 0; i < ARRAY_SIZE(country_chplan_map); i++) {
		if (alpha2[0] == map[i].alpha2[0] && alpha2[1] == map[i].alpha2[1]) {
			if (ent)
				_rtw_memcpy(ent, &map[i], sizeof(*ent));
			return true;
		}
	}
	return false;
#endif
}

#ifdef CONFIG_RTW_CHPLAN_DEV
static void rtk_regdb_dump_chplan_test(void *sel)
{
	int i, j;

	/* check 2G CHD redundent */
	for (i = RTW_CHD_2G_00; i < RTW_CHD_2G_MAX; i++) {
		for (j = RTW_CHD_2G_00; j < i; j++) {
			#if CH_MAP_2G
			if (_rtw_memcmp(&rtw_channel_def_2g[i], &rtw_channel_def_2g[j], sizeof(rtw_channel_def_2g[i])))
			#else
			if (CH_LIST_LEN(rtw_channel_def_2g[i]) == CH_LIST_LEN(rtw_channel_def_2g[j])
				&& _rtw_memcmp(&CH_LIST_CH(rtw_channel_def_2g[i], 0), &CH_LIST_CH(rtw_channel_def_2g[j], 0), CH_LIST_LEN(rtw_channel_def_2g[i]) + 1) == _TRUE)
			#endif
				RTW_PRINT_SEL(sel, "2G chd:%u and %u is the same\n", i, j);
		}
	}

	#if !CH_MAP_2G
	/* check 2G CHD invalid channel */
	for (i = RTW_CHD_2G_00; i < RTW_CHD_2G_MAX; i++) {
		for (j = 0; j < CH_LIST_LEN(rtw_channel_def_2g[i]); j++) {
			if (rtw_bch2freq(BAND_ON_24G, CH_LIST_CH(rtw_channel_def_2g[i], j)) == 0)
				RTW_PRINT_SEL(sel, "2G invalid ch:%u at (%d,%d)\n", CH_LIST_CH(rtw_channel_def_2g[i], j), i, j);
		}
	}
	#endif

#if CONFIG_IEEE80211_BAND_5GHZ
	/* check 5G CHD redundent */
	for (i = RTW_CHD_5G_00; i < RTW_CHD_5G_MAX; i++) {
		for (j = RTW_CHD_5G_00; j < i; j++) {
			#if CH_MAP_5G
			if (_rtw_memcmp(&rtw_channel_def_5g[i], &rtw_channel_def_5g[j], sizeof(rtw_channel_def_5g[i])))
			#else
			if (CH_LIST_LEN(rtw_channel_def_5g[i]) == CH_LIST_LEN(rtw_channel_def_5g[j])
				&& _rtw_memcmp(&CH_LIST_CH(rtw_channel_def_5g[i], 0), &CH_LIST_CH(rtw_channel_def_5g[j], 0), CH_LIST_LEN(rtw_channel_def_5g[i]) + 1) == _TRUE)
			#endif
				RTW_PRINT_SEL(sel, "5G chd:%u and %u is the same\n", i, j);
		}
	}

	#if !CH_MAP_5G
	/* check 5G CHD invalid channel */
	for (i = RTW_CHD_5G_00; i < RTW_CHD_5G_MAX; i++) {
		for (j = 0; j < CH_LIST_LEN(rtw_channel_def_5g[i]); j++) {
			if (rtw_bch2freq(BAND_ON_5G, CH_LIST_CH(rtw_channel_def_5g[i], j)) == 0)
				RTW_PRINT_SEL(sel, "5G invalid ch:%u at (%d,%d)\n", CH_LIST_CH(rtw_channel_def_5g[i], j), i, j);
		}
	}
	#endif
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
	/* check 6G CHD redundent */
	for (i = RTW_CHD_6G_00; i < RTW_CHD_6G_MAX; i++) {
		for (j = RTW_CHD_6G_00; j < i; j++) {
			#if CH_MAP_6G
			if (_rtw_memcmp(&rtw_channel_def_6g[i], &rtw_channel_def_6g[j], sizeof(rtw_channel_def_6g[i])))
			#else
			if (CH_LIST_LEN(rtw_channel_def_6g[i]) == CH_LIST_LEN(rtw_channel_def_6g[j])
				&& _rtw_memcmp(&CH_LIST_CH(rtw_channel_def_6g[i], 0), &CH_LIST_CH(rtw_channel_def_6g[j], 0), CH_LIST_LEN(rtw_channel_def_6g[i]) + 1) == _TRUE)
			#endif
				RTW_PRINT_SEL(sel, "6G chd:%u and %u is the same\n", i, j);
		}
	}

	#if !CH_MAP_6G
	/* check 6G CHD invalid channel */
	for (i = RTW_CHD_6G_00; i < RTW_CHD_6G_MAX; i++) {
		for (j = 0; j < CH_LIST_LEN(rtw_channel_def_6g[i]); j++) {
			if (rtw_bch2freq(BAND_ON_6G, CH_LIST_CH(rtw_channel_def_6g[i], j)) == 0)
				RTW_PRINT_SEL(sel, "6G invalid ch:%u at (%d,%d)\n", CH_LIST_CH(rtw_channel_def_6g[i], j), i, j);
		}
	}
	#endif
#endif

	/* check chplan 2G_5G redundent */
	for (i = 0; i < ARRAY_SIZE(RTW_ChannelPlanMap); i++) {
		if (!rtw_is_channel_plan_valid(i))
			continue;
		for (j = 0; j < i; j++) {
			if (!rtw_is_channel_plan_valid(j))
				continue;
			if (_rtw_memcmp(&RTW_ChannelPlanMap[i], &RTW_ChannelPlanMap[j], sizeof(RTW_ChannelPlanMap[i])) == _TRUE)
				RTW_PRINT_SEL(sel, "channel plan 0x%02x and 0x%02x is the same\n", i, j);
		}
	}

#if CONFIG_IEEE80211_BAND_6GHZ
	/* check chplan 6G redundent */
	for (i = 0; i < ARRAY_SIZE(rtw_chplan_6g_map); i++) {
		if (!rtw_is_channel_plan_6g_valid(i))
			continue;
		for (j = 0; j < i; j++) {
			if (!rtw_is_channel_plan_6g_valid(j))
				continue;
			if (_rtw_memcmp(&rtw_chplan_6g_map[i], &rtw_chplan_6g_map[j], sizeof(rtw_chplan_6g_map[i])) == _TRUE)
				RTW_PRINT_SEL(sel, "channel plan 6g 0x%02x and 0x%02x is the same\n", i, j);
		}
	}
#endif


	/* check country invalid chplan/chplan_6g */
{
	struct country_chplan ent;
	u8 code[2];

	for (code[0] = 'A'; code[0] <= 'Z'; code[0]++) {
		for (code[1] = 'A'; code[1] <= 'Z'; code[1]++) {
			if (!rtw_get_chplan_from_country(code, &ent))
				continue;
			if (!rtw_is_channel_plan_valid(ent.domain_code))
				RTW_PRINT_SEL(sel, "country \"%c%c\" has invalid domain_code:0x%02X\n", code[0], code[1], ent.domain_code);
			#if CONFIG_IEEE80211_BAND_6GHZ
			if (!rtw_is_channel_plan_6g_valid(ent.domain_code_6g))
				RTW_PRINT_SEL(sel, "country \"%c%c\" has invalid domain_code_6g:0x%02X\n", code[0], code[1], ent.domain_code_6g);
			#endif
		}
	}
}
}
#endif /* CONFIG_RTW_CHPLAN_DEV */

static void rtk_regdb_get_ver_str(char *buf, size_t buf_len)
{
	snprintf(buf, buf_len, "%s%s"
		"%s%s-%s%s"
		, RTW_MODULE_NAME, strlen(RTW_MODULE_NAME) ? "-" : ""
		, RTW_DOMAIN_MAP_VER, RTW_DOMAIN_MAP_M_VER, RTW_COUNTRY_MAP_VER, RTW_COUNTRY_MAP_M_VER);
}

bool rtk_regdb_init(void)
{
#if CC_2D_OFFSET
	build_cc_2d_offset();
#endif
	return true;
}

DECL_REGDB_OPS(
	rtk_regdb_init,				/* _init */
	NULL,					/* _deinit */
	rtk_regdb_get_default_regd_2g,		/* _get_default_regd_2g */
	rtk_regdb_get_default_regd_5g,		/* _get_default_regd_5g */
	rtk_regdb_is_domain_code_valid,		/* _is_domain_code_valid */
	rtk_regdb_domain_get_ch,		/* _domain_get_ch */
	rtk_regdb_get_default_regd_6g,		/* _get_default_regd_6g */
	rtk_regdb_is_domain_code_6g_valid,	/* _is_domain_code_6g_valid */
	rtk_regdb_domain_6g_get_ch,		/* _domain_6g_get_ch */
	rtk_regdb_get_chplan_from_alpha2,	/* _get_chplan_from_alpha2 */
	rtk_regdb_dump_chplan_test,		/* _dump_chplan_test */
	rtk_regdb_get_ver_str			/* _get_ver_str */
)

