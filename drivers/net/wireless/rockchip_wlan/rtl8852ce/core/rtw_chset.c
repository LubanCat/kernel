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
#define _RTW_CHSET_C_

#include <drv_types.h>

const char *const _rtw_ch_type_str[] = {
	[RTW_CHT_DIS]		= "DIS",
	[RTW_CHT_NO_IR]		= "NO_IR",
	[RTW_CHT_DFS]		= "DFS",
	[RTW_CHT_NO_HT40U]	= "NO_40M+",
	[RTW_CHT_NO_HT40L]	= "NO_40M-",
	[RTW_CHT_NO_80MHZ]	= "NO_80M",
	[RTW_CHT_NO_160MHZ]	= "NO_160M",
	[RTW_CHT_NUM]		= "UNKNOWN",
};

enum rtw_ch_type get_ch_type_from_str(const char *str, size_t str_len)
{
	u8 i;

	for (i = 0; i < RTW_CHT_NUM; i++)
		if (str_len == strlen(rtw_ch_type_str(i))
			&& strncmp(str, rtw_ch_type_str(i), str_len) == 0)
			return i;
	return RTW_CHT_NUM;
}

char *rtw_get_ch_flags_str(char *buf, u8 flags, char delim)
{
	char *pos = buf;
	char d_str[2] = {delim, '\0'};
	int i;

	for (i = 0; i < RTW_CHT_NUM; i++) {
		if (!(flags & BIT(i)))
			continue;
		pos += snprintf(pos, RTW_CH_FLAGS_STR_LEN - (pos - buf), "%s%s"
			, pos == buf ? "" : d_str, rtw_ch_type_str(i));
		if (pos >= buf + RTW_CH_FLAGS_STR_LEN - 1)
			break;
	}
	if (pos == buf)
		*buf = '\0';

	return buf;
}

int rtw_chset_init(struct rtw_chset *chset, u8 band_bmp)
{
	u8 ch_num = 0;
	int band, i;
	u8 (*center_chs_num)(u8);
	u8 (*center_chs)(u8, u8);
	u8 cch_num;

	_rtw_memset(chset->chs, 0, sizeof(RT_CHANNEL_INFO) * MAX_CHANNEL_NUM);

	for (band = 0; band < BAND_MAX; band++) {
		if (!(band_bmp & band_to_band_cap(band)))
			continue;

		center_chs_num = center_chs_num_of_band[band];
		center_chs = center_chs_of_band[band];
		if (!center_chs_num || !center_chs) {
			rtw_warn_on(1);
			continue;
		}

		chset->chs_of_band[band] = &chset->chs[ch_num];
		chset->chs_offset_of_band[band] = ch_num;
		chset->chs_len_of_band[band] = 0;

		cch_num = center_chs_num(CHANNEL_WIDTH_20);
		for (i = 0; i < cch_num; i++) {
			chset->chs[ch_num].band = band;
			chset->chs[ch_num].ChannelNum = center_chs(CHANNEL_WIDTH_20, i);
			chset->chs_len_of_band[band]++;;
			ch_num++;
		}
	}

	chset->chs_len = ch_num;

	return _SUCCESS;
}

/*
 * Search enabled channel with the @param ch of @param band in given @param ch_set
 * @ch_set: the given channel set
 * @band: the given band
 * @ch: the given channel number
 *
 * return the index of channel_num in channel_set, -1 if not found
 */
static int _rtw_chset_search_bch(const struct rtw_chset *chset, enum band_type band, u32 ch, bool include_dis)
{
	int idx_of_band = 255;

	if (band >= BAND_MAX || !chset->chs_of_band[band] || ch == 0)
		return -1;

	if (band == BAND_ON_24G)
		idx_of_band = ch_to_cch_2g_idx(ch);
#if CONFIG_IEEE80211_BAND_5GHZ
	else if (band == BAND_ON_5G)
		idx_of_band = ch_to_cch_5g_20m_idx(ch);
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	else if (band == BAND_ON_6G)
		idx_of_band = ch_to_cch_6g_20m_idx(ch);
#endif
	if (idx_of_band == 255)
		return -1;

	if (include_dis || !(chset->chs_of_band[band][idx_of_band].flags & RTW_CHF_DIS))
		return chset->chs_offset_of_band[band] + idx_of_band;

	return -1;
}

int rtw_chset_search_bch(const struct rtw_chset *chset, enum band_type band, u32 ch)
{
	return _rtw_chset_search_bch(chset, band, ch, false);
}

int rtw_chset_search_bch_include_dis(const struct rtw_chset *chset, enum band_type band, u32 ch)
{
	return _rtw_chset_search_bch(chset, band, ch, true);
}

RT_CHANNEL_INFO *rtw_chset_get_chinfo_by_bch(const struct rtw_chset *chset, enum band_type band, u32 ch, bool include_dis)
{
	int i = _rtw_chset_search_bch(chset, band, ch, include_dis);

	return i >= 0 ? (RT_CHANNEL_INFO *)&chset->chs[i] : NULL;
}

#if CONFIG_ALLOW_FUNC_2G_5G_ONLY
RTW_FUNC_2G_5G_ONLY int rtw_chset_search_ch(const struct rtw_chset *chset, u32 ch)
{
	return _rtw_chset_search_bch(chset, rtw_is_2g_ch(ch) ? BAND_ON_24G : BAND_ON_5G, ch, false);
}

RTW_FUNC_2G_5G_ONLY int rtw_chset_search_ch_include_dis(const struct rtw_chset *chset, u32 ch)
{
	return _rtw_chset_search_bch(chset, rtw_is_2g_ch(ch) ? BAND_ON_24G : BAND_ON_5G, ch, true);
}
#endif

/*
 * Check if the @param ch, bw, offset is valid for the given @param ch_set
 * @ch_set: the given channel set
 * @ch: the given channel number
 * @bw: the given bandwidth
 * @offset: the given channel offset
 *
 * return valid (1) or not (0)
 */
static u8 _rtw_chset_is_bchbw_valid(const struct rtw_chset *chset, enum band_type band, u8 ch, u8 bw, u8 offset
	, bool allow_primary_passive, bool allow_passive)
{
	u8 cch;
	u8 *op_chs;
	u8 op_ch_num;
	u8 valid = 0;
	int i;
	int ch_idx;

	cch = rtw_get_center_ch_by_band(band, ch, bw, offset);

	if (!rtw_get_op_chs_by_bcch_bw(band, cch, bw, &op_chs, &op_ch_num))
		goto exit;

	for (i = 0; i < op_ch_num; i++) {
		if (0)
			RTW_INFO("%u,%u,%u,%u - cch:%u, bw:%u, op_ch:%u\n", band, ch, bw, offset, cch, bw, *(op_chs + i));
		ch_idx = rtw_chset_search_bch(chset, band, *(op_chs + i));
		if (ch_idx == -1)
			break;
		if (!allow_passive && chset->chs[ch_idx].flags & RTW_CHF_NO_IR) {
			/* all sub chs are passive is not allowed and one of sub ch is NO_IR */
			if (!allow_primary_passive) /* even primary ch is not allow to be NO_IR */
				break;
			if (chset->chs[ch_idx].ChannelNum != ch) /* allow primary ch NO_IR, but this is not primary ch */
				break;
		}
		if (bw >= CHANNEL_WIDTH_40) {
			if ((chset->chs[ch_idx].flags & RTW_CHF_NO_HT40U) && i % 2 == 0)
				break;
			if ((chset->chs[ch_idx].flags & RTW_CHF_NO_HT40L) && i % 2 == 1)
				break;
		}
		if (bw >= CHANNEL_WIDTH_80 && (chset->chs[ch_idx].flags & RTW_CHF_NO_80MHZ))
			break;
		if (bw >= CHANNEL_WIDTH_160 && (chset->chs[ch_idx].flags & RTW_CHF_NO_160MHZ))
			break;
	}

	if (op_ch_num != 0 && i == op_ch_num)
		valid = 1;

exit:
	return valid;
}

#if CONFIG_ALLOW_FUNC_2G_5G_ONLY
RTW_FUNC_2G_5G_ONLY u8 rtw_chset_is_chbw_valid(const struct rtw_chset *chset, u8 ch, u8 bw, u8 offset, bool allow_primary_passive, bool allow_passive)
{
	return _rtw_chset_is_bchbw_valid(chset, rtw_is_2g_ch(ch) ? BAND_ON_24G : BAND_ON_5G, ch, bw, offset, allow_primary_passive, allow_passive);
}
#endif

u8 rtw_chset_is_bchbw_valid(const struct rtw_chset *chset, enum band_type band, u8 ch, u8 bw, u8 offset
	, bool allow_primary_passive, bool allow_passive)
{
	return _rtw_chset_is_bchbw_valid(chset, band, ch, bw, offset, allow_primary_passive, allow_passive);
}

/**
 * rtw_chset_sync_bchbw - obey g_ch, adjust g_bw, g_offset, bw, offset to fit in channel plan
 * @ch_set: channel plan to check
 * @req_ch: pointer of the request ch, may be modified further
 * @req_bw: pointer of the request bw, may be modified further
 * @req_offset: pointer of the request offset, may be modified further
 * @g_ch: pointer of the ongoing group ch
 * @g_bw: pointer of the ongoing group bw, may be modified further
 * @g_offset: pointer of the ongoing group offset, may be modified further
 * @allow_primary_passive: if allow passive primary ch when deciding chbw
 * @allow_passive: if allow passive ch (not primary) when deciding chbw
 */
void rtw_chset_sync_bchbw(const struct rtw_chset *chset, enum band_type *req_band, u8 *req_ch, u8 *req_bw, u8 *req_offset
	, enum band_type *g_band, u8 *g_ch, u8 *g_bw, u8 *g_offset, bool allow_primary_passive, bool allow_passive)
{
	enum band_type r_band;
	u8 r_ch, r_bw, r_offset;
	enum band_type u_band;
	u8 u_ch, u_bw, u_offset;
	u8 cur_bw = *req_bw;

	while (1) {
		r_band = *req_band;
		r_ch = *req_ch;
		r_bw = cur_bw;
		r_offset = *req_offset;
		u_band = *g_band;
		u_ch = *g_ch;
		u_bw = *g_bw;
		u_offset = *g_offset;

		rtw_sync_bchbw(&r_band, &r_ch, &r_bw, &r_offset, &u_band, &u_ch, &u_bw, &u_offset);

		if (rtw_chset_is_bchbw_valid(chset, r_band, r_ch, r_bw, r_offset, allow_primary_passive, allow_passive))
			break;
		if (cur_bw == CHANNEL_WIDTH_20) {
			rtw_warn_on(1);
			break;
		}
		cur_bw--;
	};

	*req_band = r_band;
	*req_ch = r_ch;
	*req_bw = r_bw;
	*req_offset = r_offset;
	*g_band = u_band;
	*g_ch = u_ch;
	*g_bw = u_bw;
	*g_offset = u_offset;
}

#if CONFIG_ALLOW_FUNC_2G_5G_ONLY
RTW_FUNC_2G_5G_ONLY void rtw_chset_sync_chbw(const struct rtw_chset *chset, u8 *req_ch, u8 *req_bw, u8 *req_offset
	, u8 *g_ch, u8 *g_bw, u8 *g_offset, bool allow_primary_passive, bool allow_passive)
{
	enum band_type band = rtw_is_2g_ch(*g_ch) ? BAND_ON_24G : BAND_ON_5G; /* follow g_ch's band */

	rtw_chset_sync_bchbw(chset, &band, req_ch, req_bw, req_offset, &band, g_ch, g_bw, g_offset, allow_primary_passive, allow_passive);
}
#endif

u8 *rtw_chset_set_spt_chs_ie(const struct rtw_chset *chset, u8 *buf_pos, uint *buf_len)
{
	u8 i = 0;
	u8 fch = 0, lch = 0, ch;
	u8 *cont = buf_pos + 2;
	const RT_CHANNEL_INFO *chinfo;

	while (i < chset->chs_len) {
		chinfo = &chset->chs[i++];

		if (chinfo->flags & RTW_CHF_DIS)
			continue;

		#if CONFIG_IEEE80211_BAND_6GHZ
		/* don't appnd 6G chs now, how to distinguish 2G/5G chs with 6G? */
		if (chinfo->band == BAND_ON_6G)
			continue;
		#endif

		ch = chinfo->ChannelNum;
		if (fch == 0) {
			fch = ch;
			lch = ch;
			continue;
		}

		if (lch + 1 != ch) {
			*cont = fch;
			*(cont + 1) = lch - fch + 1;
			cont += 2;
			fch = ch;
		}
		lch = ch;
	}
	if (fch) {
		/* last subband */
		*cont = fch;
		*(cont + 1) = lch - fch + 1;
		cont += 2;
	}

	if (cont > buf_pos + 2) {
		*buf_pos = WLAN_EID_SUPPORTED_CHANNELS;
		*(buf_pos + 1) = cont - buf_pos - 2;
		*buf_len += cont - buf_pos;
		return cont;
	}
	return buf_pos;
}

#ifdef CONFIG_PROC_DEBUG
void dump_chinfos(void *sel, const RT_CHANNEL_INFO *chinfos, u8 chinfo_num)
{
	u32 bhint_sec;
	char bhint_buf[8];
	u16 non_ocp_sec;
	char non_ocp_buf[8];
	char flags_buf[RTW_CH_FLAGS_STR_LEN];
	u8 enable_ch_num = 0;
	u8 i;

	RTW_PRINT_SEL(sel, "%-3s %-4s %-5s %-4s flags\n", "ch", "freq", "bhint", "nocp");

	for (i = 0; i < chinfo_num; i++) {
		if (chinfos[i].flags & RTW_CHF_DIS)
			continue;
		enable_ch_num++;

		bhint_sec = 0;
		if (CH_IS_BCN_HINT(&chinfos[i])) {
			bhint_sec = rtw_systime_to_ms(chinfos[i].bcn_hint_end_time - rtw_get_current_time()) / 1000;
			if (bhint_sec > 99999)
				bhint_sec = 99999;
		}
		snprintf(bhint_buf, 8, "%d", bhint_sec);

		non_ocp_sec = 0;
		#ifdef CONFIG_DFS_MASTER
		if (CH_IS_NON_OCP(&chinfos[i]))
			non_ocp_sec = rtw_systime_to_ms(chinfos[i].non_ocp_end_time - rtw_get_current_time()) / 1000;
		#endif
		snprintf(non_ocp_buf, 8, "%d", non_ocp_sec);

		RTW_PRINT_SEL(sel, "%3u %4u %5s %4s %s\n"
			, chinfos[i].ChannelNum, rtw_bch2freq(chinfos[i].band, chinfos[i].ChannelNum)
			, bhint_buf, non_ocp_buf, rtw_get_ch_flags_str(flags_buf, chinfos[i].flags, ' ')
		);
	}

	RTW_PRINT_SEL(sel, "total ch number:%d\n", enable_ch_num);
}
#endif /* CONFIG_PROC_DEBUG */

#ifdef CONFIG_RTW_CHSET_DEV
static bool dump_chset_init_test(void *sel, u8 band_bmp)
{
	struct rtw_chset *chset;
	RT_CHANNEL_INFO *chinfo;
	int band, i, j;
	u8 (*center_chs_num)(u8);
	u8 (*center_chs)(u8, u8);
	u8 cch_num;
	bool ret = true, valid, match;

	chset = rtw_malloc(sizeof(*chset));
	if (!chset) {
		RTW_PRINT_SEL(sel, "alloc chset fail\n");
		ret = false;
		goto exit;
	}

	rtw_chset_init(chset, band_bmp);

	for (band = 0; band < BAND_MAX; band++) {
		if (!(band_bmp & band_to_band_cap(band)))
			continue;

		center_chs_num = center_chs_num_of_band[band];
		center_chs = center_chs_of_band[band];
		if (!center_chs_num || !center_chs) {
			rtw_warn_on(1);
			continue;
		}

		cch_num = center_chs_num(CHANNEL_WIDTH_20);
		for (i = 0; i <= 255 ; i++) {
			for (j = 0; j < cch_num; j++)
				if (i == center_chs(CHANNEL_WIDTH_20, j))
					break;
			match = j < cch_num;
			chinfo = rtw_chset_get_chinfo_by_bch(chset, band, i, true);
			valid = (match && chinfo && band == chinfo->band && i == chinfo->ChannelNum) || (!match && !chinfo);
			if (!valid)
				RTW_PRINT_SEL(sel, "band:%u(%d) ch:%u(%d) fail\n"
					, band, chinfo ? chinfo->band : -1
					, i, chinfo ? chinfo->ChannelNum : -1);
			ret &= valid;
		}
	}

exit:
	return ret;
}

static void dump_chset_search_time_test(void *sel, struct rtw_chset *chset, enum band_type band, u8 *ch_array, size_t ch_array_sz, u32 times)
{
	u32 round = times / ch_array_sz;
	u32 i, r;
	sysptime start, end;

	start = rtw_sptime_get_raw();

	for (r = 0; r < round; r++)
		for (i = 0; i < ch_array_sz; i++)
			rtw_chset_get_chinfo_by_bch(chset, band, ch_array[i], true);

	end = rtw_sptime_get_raw();

	RTW_PRINT_SEL(sel, "%s\t%10u\t%10u\t%10lld\n", band_str(band), times, round, rtw_sptime_diff_ns(start, end));
}

void dump_chset_test(void *sel)
{
	struct rtw_chset *chset;
	u8 bmp;

	for (bmp = 0; bmp <= BIT(BAND_MAX) - 1; bmp++) {
		if (!dump_chset_init_test(sel, bmp))
			return;
	}

	chset = rtw_malloc(sizeof(*chset));
	if (!chset) {
		RTW_PRINT_SEL(sel, "alloc chset fail\n");
		return;
	}

	bmp = BIT(BAND_MAX) - 1;
	rtw_chset_init(chset, bmp);

	RTW_PRINT_SEL(sel, "band\ttimes\tround\tns\n");

	dump_chset_search_time_test(sel, chset, BAND_ON_24G, center_ch_2g, ARRAY_SIZE(center_ch_2g), 100000);
	dump_chset_search_time_test(sel, chset, BAND_ON_24G, center_ch_2g, 1, 100000);

	dump_chset_search_time_test(sel, chset, BAND_ON_5G, center_ch_5g_20m, ARRAY_SIZE(center_ch_5g_20m), 100000);
	dump_chset_search_time_test(sel, chset, BAND_ON_5G, center_ch_5g_20m, 1, 100000);

	dump_chset_search_time_test(sel, chset, BAND_ON_6G, center_ch_6g_20m, ARRAY_SIZE(center_ch_6g_20m), 100000);
	dump_chset_search_time_test(sel, chset, BAND_ON_6G, center_ch_6g_20m, 1, 100000);
}
#endif /* CONFIG_RTW_CHSET_DEV */

