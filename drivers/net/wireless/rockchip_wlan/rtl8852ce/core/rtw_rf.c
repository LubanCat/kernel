/******************************************************************************
 *
 * Copyright(c) 2007 - 2019 Realtek Corporation.
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
#define _RTW_RF_C_

#include <drv_types.h>

const char *const _ch_width_str[CHANNEL_WIDTH_MAX] = {
	[CHANNEL_WIDTH_20]		= "20MHz",
	[CHANNEL_WIDTH_40]		= "40MHz",
	[CHANNEL_WIDTH_80]		= "80MHz",
	[CHANNEL_WIDTH_160]		= "160MHz",
	[CHANNEL_WIDTH_80_80]	= "80_80MHz",
	[CHANNEL_WIDTH_5]		= "5MHz",
	[CHANNEL_WIDTH_10]		= "10MHz",
};

const u8 _ch_width_to_bw_cap[CHANNEL_WIDTH_MAX] = {
	[CHANNEL_WIDTH_20]		= BW_CAP_20M,
	[CHANNEL_WIDTH_40]		= BW_CAP_40M,
	[CHANNEL_WIDTH_80]		= BW_CAP_80M,
	[CHANNEL_WIDTH_160]		= BW_CAP_160M,
	[CHANNEL_WIDTH_80_80]	= BW_CAP_80_80M,
	[CHANNEL_WIDTH_5]		= BW_CAP_5M,
	[CHANNEL_WIDTH_10]		= BW_CAP_10M,
};

const char *const _rtw_band_str[] = {
	[BAND_ON_24G]	= "2.4G",
	[BAND_ON_5G]	= "5G",
	[BAND_ON_6G]	= "6G",
	[BAND_MAX]		= "BAND_MAX",
};

const u8 _band_to_band_cap[] = {
	[BAND_ON_24G]	= BAND_CAP_2G,
	[BAND_ON_5G]	= BAND_CAP_5G,
	[BAND_ON_6G]	= BAND_CAP_6G,
	[BAND_MAX]		= 0,
};

#ifdef CONFIG_ECSA_PHL
int alink_get_supported_op_class(struct _ADAPTER_LINK *padapter_link, u8 *op_set, int len)
{
	_adapter *padapter = padapter_link->adapter;
	struct link_mlme_ext_priv *pmlmeext = &(padapter_link->mlmeextpriv);
	struct rtw_chset *chset = adapter_to_chset(padapter);
	int match = 0, i = 0, j, k = 0;
	const struct op_class_t *cl;
	u8 cur_op_class;

	cur_op_class = rtw_get_op_class_by_bchbw(pmlmeext->chandef.band,
						pmlmeext->chandef.chan,
						pmlmeext->chandef.bw,
						pmlmeext->chandef.offset);

	if (cur_op_class && k < len) {
		/* current op class SHALL be the 1st supported op class */
		*op_set = cur_op_class;
		k++;
	}

	for (i = 0; i < global_op_class_num; i++) {
		cl = &global_op_class[i];

		for (j = 0; j < OPC_CH_LIST_LEN(cl); j++) {
			if ((match = rtw_chset_search_bch(chset, cl->band, OPC_CH_LIST_CH(cl, j))) == -1) //
				break;
		}

		if (match != -1 && k < len)
			op_set[k++] = cl->class_id;
	}
	return (k > len ? len : k);
}

int get_supported_op_class(_adapter *padapter, u8 *op_set, int len)
{
	return alink_get_supported_op_class(GET_PRIMARY_LINK(padapter), op_set, len);
}
#endif /* CONFIG_ECSA_PHL */

const u8 _rf_type_to_rf_path[] = {
	1, /*RF_1T1R*/
	2, /*RF_1T2R*/
	2, /*RF_2T2R*/
	3, /*RF_2T3R*/
	4, /*RF_2T4R*/
	3, /*RF_3T3R*/
	4, /*RF_3T4R*/
	4, /*RF_4T4R*/
	4, /*RF_TYPE_MAX*/
};

const u8 _rf_type_to_rf_tx_cnt[] = {
	1, /*RF_1T1R*/
	1, /*RF_1T2R*/
	2, /*RF_2T2R*/
	2, /*RF_2T3R*/
	2, /*RF_2T4R*/
	3, /*RF_3T3R*/
	3, /*RF_3T4R*/
	4, /*RF_4T4R*/
	1, /*RF_TYPE_MAX*/
};

const u8 _rf_type_to_rf_rx_cnt[] = {
	1, /*RF_1T1R*/
	2, /*RF_1T2R*/
	2, /*RF_2T2R*/
	3, /*RF_2T3R*/
	4, /*RF_2T4R*/
	3, /*RF_3T3R*/
	4, /*RF_3T4R*/
	4, /*RF_4T4R*/
	1, /*RF_TYPE_MAX*/
};

const char *const _rf_type_to_rfpath_str[] = {
	"RF_1T1R",
	"RF_1T2R",
	"RF_2T2R",
	"RF_2T3R",
	"RF_2T4R",
	"RF_3T3R",
	"RF_3T4R",
	"RF_4T4R",
	"RF_TYPE_MAX"
};

static const u8 _trx_num_to_rf_type[RF_PATH_MAX][RF_PATH_MAX] = {
	{RF_1T1R,		RF_1T2R,		RF_TYPE_MAX,	RF_TYPE_MAX},
	{RF_TYPE_MAX,	RF_2T2R,		RF_2T3R,		RF_2T4R},
	{RF_TYPE_MAX,	RF_TYPE_MAX,	RF_3T3R,		RF_3T4R},
	{RF_TYPE_MAX,	RF_TYPE_MAX,	RF_TYPE_MAX,	RF_4T4R},
};

enum rf_type trx_num_to_rf_type(u8 tx_num, u8 rx_num)
{
	if (tx_num > 0 && tx_num <= RF_PATH_MAX && rx_num > 0 && rx_num <= RF_PATH_MAX)
		return _trx_num_to_rf_type[tx_num - 1][rx_num - 1];
	return RF_TYPE_MAX;
}

enum rf_type trx_bmp_to_rf_type(u8 tx_bmp, u8 rx_bmp)
{
	u8 tx_num = 0;
	u8 rx_num = 0;
	int i;

	for (i = 0; i < RF_PATH_MAX; i++) {
		if (tx_bmp >> i & BIT0)
			tx_num++;
		if (rx_bmp >> i & BIT0)
			rx_num++;
	}

	return trx_num_to_rf_type(tx_num, rx_num);
}

bool rf_type_is_a_in_b(enum rf_type a, enum rf_type b)
{
	return rf_type_to_rf_tx_cnt(a) <= rf_type_to_rf_tx_cnt(b)
		&& rf_type_to_rf_rx_cnt(a) <= rf_type_to_rf_rx_cnt(b);
}

static void rtw_path_bmp_limit_from_higher(u8 *bmp, u8 *bmp_bit_cnt, u8 bit_cnt_lmt)
{
	int i;

	for (i = RF_PATH_MAX - 1; *bmp_bit_cnt > bit_cnt_lmt && i >= 0; i--) {
		if (*bmp & BIT(i)) {
			*bmp &= ~BIT(i);
			(*bmp_bit_cnt)--;
		}
	}
}

u8 rtw_restrict_trx_path_bmp_by_rftype(u8 trx_path_bmp, enum rf_type type, u8 *tx_num, u8 *rx_num)
{
	u8 bmp_tx = (trx_path_bmp & 0xF0) >> 4;
	u8 bmp_rx = trx_path_bmp & 0x0F;
	u8 bmp_tx_num = 0, bmp_rx_num = 0;
	u8 tx_num_lmt, rx_num_lmt;
	enum rf_type ret_type = RF_TYPE_MAX;
	int i, j;

	for (i = 0; i < RF_PATH_MAX; i++) {
		if (bmp_tx & BIT(i))
			bmp_tx_num++;
		if (bmp_rx & BIT(i))
			bmp_rx_num++;
	}

	/* limit higher bit first according to input type */
	tx_num_lmt = rf_type_to_rf_tx_cnt(type);
	rx_num_lmt = rf_type_to_rf_rx_cnt(type);
	rtw_path_bmp_limit_from_higher(&bmp_tx, &bmp_tx_num, tx_num_lmt);
	rtw_path_bmp_limit_from_higher(&bmp_rx, &bmp_rx_num, rx_num_lmt);

	/* search for valid rf_type (larger RX prefer) */
	for (j = bmp_rx_num; j > 0; j--) {
		for (i = bmp_tx_num; i > 0; i--) {
			ret_type = trx_num_to_rf_type(i, j);
			if (RF_TYPE_VALID(ret_type)) {
				rtw_path_bmp_limit_from_higher(&bmp_tx, &bmp_tx_num, i);
				rtw_path_bmp_limit_from_higher(&bmp_rx, &bmp_rx_num, j);
				if (tx_num)
					*tx_num = bmp_tx_num;
				if (rx_num)
					*rx_num = bmp_rx_num;
				goto exit;
			}
		}
	}

exit:
	return RF_TYPE_VALID(ret_type) ? ((bmp_tx << 4) | bmp_rx) : 0x00;
}

#if CONFIG_TXPWR_LIMIT
void dump_regd_exc_list(void *sel, struct rf_ctl_t *rfctl)
{
	/* TODO: get from phl */
}
#endif /* CONFIG_TXPWR_LIMIT */

