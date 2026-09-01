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
#ifndef	__RTW_CH_UTILS_H_
#define __RTW_CH_UTILS_H_

#define CENTER_CH_2G_NUM	14
#define CENTER_CH_2G_40M_NUM	9

#define CENTER_CH_5G_20M_NUM	28	/* 20M center channels */
#define CENTER_CH_5G_40M_NUM	14	/* 40M center channels */
#define CENTER_CH_5G_80M_NUM	7	/* 80M center channels */
#define CENTER_CH_5G_160M_NUM	3	/* 160M center channels */
#define CENTER_CH_5G_ALL_NUM	(CENTER_CH_5G_20M_NUM + CENTER_CH_5G_40M_NUM + CENTER_CH_5G_80M_NUM)

#define CENTER_CH_6G_20M_NUM	64	/* 20M center channels */
#define CENTER_CH_6G_40M_NUM	32	/* 40M center channels */
#define CENTER_CH_6G_80M_NUM	16	/* 80M center channels */
#define CENTER_CH_6G_160M_NUM	8	/* 160M center channels */

#define	MAX_CHANNEL_NUM_2G	CENTER_CH_2G_NUM
#define	MAX_CHANNEL_NUM_5G	CENTER_CH_5G_20M_NUM
#define MAX_CHANNEL_NUM_6G	CENTER_CH_6G_20M_NUM
#define MAX_CHANNEL_NUM_2G_5G	(MAX_CHANNEL_NUM_2G + MAX_CHANNEL_NUM_5G)

#define	MAX_CHANNEL_NUM		( \
	MAX_CHANNEL_NUM_2G \
	+ (CONFIG_IEEE80211_BAND_5GHZ ? MAX_CHANNEL_NUM_5G : 0) \
	+ (CONFIG_IEEE80211_BAND_6GHZ ? MAX_CHANNEL_NUM_6G : 0) \
	)

#define ch_to_cch_2g_idx(ch) (((ch) <= 14) ? ((ch) - 1) : 255)

extern u8 center_ch_2g[CENTER_CH_2G_NUM];
extern u8 center_ch_2g_40m[CENTER_CH_2G_40M_NUM];

u8 center_chs_2g_num(u8 bw);
u8 center_chs_2g(u8 bw, u8 id);

#if CONFIG_IEEE80211_BAND_5GHZ
#define ch_to_cch_5g_20m_idx(ch) \
	( \
		((ch) >= 36 && (ch) <= 64) ? (((ch) & 0x03) ? 255 : (((ch) - 36) >> 2)) : \
		((ch) >= 100 && (ch) <= 144) ? (((ch) & 0x03) ? 255 : (8 + (((ch) - 100) >> 2))) : \
		((ch) >= 149 && (ch) <= 177) ? ((((ch) - 1) & 0x03) ? 255 : (20 + (((ch) - 149) >> 2))) : 255 \
	)

extern u8 center_ch_5g_20m[CENTER_CH_5G_20M_NUM];
extern u8 center_ch_5g_40m[CENTER_CH_5G_40M_NUM];
extern u8 center_ch_5g_80m[CENTER_CH_5G_80M_NUM];
extern u8 center_ch_5g_160m[CENTER_CH_5G_160M_NUM];
extern u8 center_ch_5g_all[CENTER_CH_5G_ALL_NUM];

u8 center_chs_5g_num(u8 bw);
u8 center_chs_5g(u8 bw, u8 id);
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
#define ch_to_cch_6g_20m_idx(ch) ((((ch) - 1) & 0x03) ? 255 : (((ch) - 1) >> 2))

extern u8 center_ch_6g_20m[CENTER_CH_6G_20M_NUM];
extern u8 center_ch_6g_40m[CENTER_CH_6G_40M_NUM];
extern u8 center_ch_6g_80m[CENTER_CH_6G_80M_NUM];
extern u8 center_ch_6g_160m[CENTER_CH_6G_160M_NUM];

u8 center_chs_6g_num(u8 bw);
u8 center_chs_6g(u8 bw, u8 id);
#endif

extern u8 (*center_chs_num_of_band[BAND_MAX])(u8 bw);
extern u8 (*center_chs_of_band[BAND_MAX])(u8 bw, u8 id);

RTW_FUNC_2G_5G_ONLY u8 rtw_get_scch_by_cch_offset(u8 cch, u8 bw, u8 offset);
RTW_FUNC_2G_5G_ONLY u8 rtw_get_scch_by_cch_opch(u8 cch, u8 bw, u8 opch);

RTW_FUNC_2G_5G_ONLY u8 rtw_get_op_chs_by_cch_bw(u8 cch, u8 bw, u8 **op_chs, u8 *op_ch_num);

RTW_FUNC_2G_5G_ONLY u8 rtw_get_offset_by_chbw(u8 ch, u8 bw, u8 *r_offset);
RTW_FUNC_2G_5G_ONLY u8 rtw_get_center_ch(u8 ch, u8 bw, u8 offset);

RTW_FUNC_2G_5G_ONLY bool rtw_is_chbw_grouped(u8 ch_a, u8 bw_a, u8 offset_a, u8 ch_b, u8 bw_b, u8 offset_b);
RTW_FUNC_2G_5G_ONLY void rtw_sync_chbw(u8 *req_ch, u8 *req_bw, u8 *req_offset, u8 *g_ch, u8 *g_bw, u8 *g_offset);

u8 rtw_get_scch_by_bcch_offset(enum band_type band, u8 cch, u8 bw, u8 offset);
u8 rtw_get_scch_by_bcch_opch(enum band_type band, u8 cch, u8 bw, u8 opch);

u8 rtw_get_op_chs_by_bcch_bw(enum band_type band, u8 cch, u8 bw, u8 **op_chs, u8 *op_ch_num);

u8 rtw_get_offset_by_bchbw(enum band_type band, u8 ch, u8 bw, u8 *r_offset);
u8 rtw_get_offsets_by_bchbw(enum band_type band, u8 ch, u8 bw, u8 *r_offset, u8 *r_offset_num);
u8 rtw_get_center_ch_by_band(enum band_type band, u8 ch, u8 bw, u8 offset);

bool rtw_is_bchbw_grouped(enum band_type band_a, u8 ch_a, u8 bw_a, u8 offset_a
	, enum band_type band_b, u8 ch_b, u8 bw_b, u8 offset_b);
void rtw_sync_bchbw(enum band_type *req_band, u8 *req_ch, u8 *req_bw, u8 *req_offset
	, enum band_type *g_band, u8 *g_ch, u8 *g_bw, u8 *g_offset);

RTW_FUNC_2G_5G_ONLY int rtw_ch2freq(int chan);
int rtw_bch2freq(enum band_type band, int ch);
int rtw_freq2ch(int freq);
enum band_type rtw_freq2band(int freq);
enum channel_width rtw_frange_to_bw(u32 hi, u32 lo);
bool rtw_freq_consecutive(int a, int b);
bool rtw_bcchbw_to_freq_range(enum band_type band, u8 c_ch, u8 bw, u32 *hi, u32 *lo);
bool rtw_bchbw_to_freq_range(enum band_type band, u8 ch, u8 bw, u8 offset, u32 *hi, u32 *lo);
RTW_FUNC_2G_5G_ONLY bool rtw_chbw_to_freq_range(u8 ch, u8 bw, u8 offset, u32 *hi, u32 *lo);

RTW_FUNC_2G_5G_ONLY static inline bool rtw_is_2g_ch(u8 ch) { return ch >= 1 && ch <= 14; }
RTW_FUNC_2G_5G_ONLY static inline bool rtw_is_5g_ch(u8 ch) { return ch >= 36 && ch <= 177; }
RTW_FUNC_2G_5G_ONLY static inline enum band_type rtw_get_band_type(u8 chan)
{
	return (chan > 14) ? BAND_ON_5G : BAND_ON_24G;
}

#endif /* __RTW_CH_UTILS_H_ */
