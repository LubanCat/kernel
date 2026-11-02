/******************************************************************************
 *
 * Copyright(c) 2019 - 2024 Realtek Corporation.
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
#ifndef _RTW_PHL_RATE_H_
#define _RTW_PHL_RATE_H_

#define RTW_PHL_RATE_SUPPORT_EHT (PHL_VER_CODE >= PHL_VERSION(2, 24, 0, 0))

#if (PHL_VER_CODE >= PHL_VERSION(2, 24, 0, 0))
typedef u8 hrate;
#else
typedef u16 hrate;
#endif

enum hal_rate_sec {
	HAL_RS_CCK		= 0,
	HAL_RS_OFDM		= 1,
	HAL_RS_HT_1SS		= 2,
	HAL_RS_HT_2SS		= 3,
	HAL_RS_HT_3SS		= 4,
	HAL_RS_HT_4SS		= 5,
	HAL_RS_VHT_1SS		= 6,
	HAL_RS_VHT_2SS		= 7,
	HAL_RS_VHT_3SS		= 8,
	HAL_RS_VHT_4SS		= 9,
	HAL_RS_HE_1SS		= 10,
	HAL_RS_HE_2SS		= 11,
	HAL_RS_HE_3SS		= 12,
	HAL_RS_HE_4SS		= 13,
	HAL_RS_HEDCM_1SS	= 14,
	HAL_RS_HEDCM_2SS	= 15,
#if RTW_PHL_RATE_SUPPORT_EHT
	HAL_RS_EHT_1SS		= 16,
	HAL_RS_EHT_2SS		= 17,
	HAL_RS_EHT_3SS		= 18,
	HAL_RS_EHT_4SS		= 19,
#endif
	HAL_RS_NUM,
};

#define IS_CCK_HAL_RS(rs) ((rs) == HAL_RS_CCK)
#define IS_OFDM_HAL_RS(rs) ((rs) == HAL_RS_OFDM)
#define IS_HT_HAL_RS(rs) ((rs) >= HAL_RS_HT_1SS && (rs) <= HAL_RS_HT_4SS)
#define IS_VHT_HAL_RS(rs) ((rs) >= HAL_RS_VHT_1SS && (rs) <= HAL_RS_VHT_4SS)
#define IS_HE_HAL_RS(rs) ((rs) >= HAL_RS_HE_1SS && (rs) <= HAL_RS_HE_4SS)
#define IS_HEDCM_HAL_RS(rs) ((rs) >= HAL_RS_HEDCM_1SS && (rs) <= HAL_RS_HEDCM_2SS)
#define IS_EHT_HAL_RS(rs) ((rs) >= HAL_RS_EHT_1SS && (rs) <= HAL_RS_EHT_4SS)

#define IS_1T_HAL_RS(rs) ((rs) == HAL_RS_CCK || (rs) == HAL_RS_OFDM \
				|| (rs) == HAL_RS_HT_1SS || (rs) == HAL_RS_VHT_1SS \
				|| (rs) == HAL_RS_HE_1SS|| (rs) == HAL_RS_HEDCM_1SS \
				|| (rs) == HAL_RS_EHT_1SS)
#define IS_2T_HAL_RS(rs) ((rs) == HAL_RS_HT_2SS || (rs) == HAL_RS_VHT_2SS \
				|| (rs) == HAL_RS_HE_2SS || (rs) == HAL_RS_HEDCM_2SS \
				|| (rs) == HAL_RS_EHT_2SS)
#define IS_3T_HAL_RS(rs) ((rs) == HAL_RS_HT_3SS || (rs) == HAL_RS_VHT_3SS \
				|| (rs) == HAL_RS_HE_3SS || (rs) == HAL_RS_EHT_3SS)
#define IS_4T_HAL_RS(rs) ((rs) == HAL_RS_HT_4SS || (rs) == HAL_RS_VHT_4SS \
				|| (rs) == HAL_RS_HE_4SS || (rs) == HAL_RS_EHT_4SS)


extern const char *const _hal_rate_sec_str[];
#define hal_rate_sec_str(rs) ((rs) > HAL_RS_NUM ? _hal_rate_sec_str[HAL_RS_NUM] : _hal_rate_sec_str[rs])

struct hal_rate_sec_ent {
	u8 tx_num; /* value of RF_TX_NUM */
	enum wlan_mode wl_mode;
	u8 rate_num;
	const hrate *rates;
};

extern const struct hal_rate_sec_ent hal_rates_by_sec[HAL_RS_NUM];

#define hal_rate_sec_tx_num(rs) (hal_rates_by_sec[(rs)].tx_num)
#define hal_rate_sec_wl_mode(rs) (hal_rates_by_sec[(rs)].wl_mode)
#define hal_rate_sec_rate_num(rs) (hal_rates_by_sec[(rs)].rate_num)

#endif /* _RTW_PHL_RATE_H_ */
