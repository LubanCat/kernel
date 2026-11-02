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
#ifndef	__RTW_OPC_UTILS_H_
#define __RTW_OPC_UTILS_H_

enum opc_bw {
	OPC_BW20	= 0,
	OPC_BW40PLUS	= 1,
	OPC_BW40MINUS	= 2,
	OPC_BW40	= 3,
	OPC_BW80	= 4,
	OPC_BW160	= 5,
	OPC_BW80P80	= 6,
	OPC_BW_NUM,
};

extern const char *const _opc_bw_str[];
#define opc_bw_str(bw) (((bw) >= OPC_BW_NUM) ? _opc_bw_str[OPC_BW_NUM] : _opc_bw_str[bw])

extern const u8 _opc_bw_to_ch_width[];
#define opc_bw_to_ch_width(bw) (((bw) >= OPC_BW_NUM) ? _opc_bw_to_ch_width[OPC_BW_NUM] : _opc_bw_to_ch_width[bw])

struct op_class_t {
	u8 class_id;
	enum band_type band;
	enum opc_bw bw;
	u8 *len_ch_attr;
};

#define OPC_CH_LIST_LEN(_opc) (_opc->len_ch_attr[0])
#define OPC_CH_LIST_CH(_opc, _i) (_opc->len_ch_attr[_i + 1])

extern const struct op_class_t global_op_class[];
extern const int global_op_class_num;

const struct op_class_t *get_opc_by_op_class(const char *alpha2, u8 op_class);
enum band_type rtw_get_band_by_op_class(const char *alpha2, u8 op_class);
bool rtw_get_bw_offset_by_op_class_ch(const char *alpha2, u8 op_class, u8 ch, u8 *bw, u8 *offset);

bool opc_contains_ch(const struct op_class_t *opc, u8 ch);

void dump_op_class_ch_title(void *sel);
void dump_global_op_class(void *sel);

RTW_FUNC_2G_5G_ONLY u8 rtw_get_op_class_by_chbw(u8 ch, u8 bw, u8 offset);
u8 rtw_get_op_class_by_bchbw(enum band_type band, u8 ch, u8 bw, u8 offset);

#ifdef CONFIG_RTW_OPCLASS_DEV
void dump_opc_test(void *sel);
#endif

#endif /* __RTW_OPC_UTILS_H_ */
