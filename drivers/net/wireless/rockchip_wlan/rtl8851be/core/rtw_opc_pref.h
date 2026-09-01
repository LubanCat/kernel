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
#ifndef	__RTW_OPC_PREF_H_
#define __RTW_OPC_PREF_H_

struct op_ch_t {
	u8 ch;
	u8 static_non_op:1; /* not in channel list */
	u8 no_ir:1;
	s16 max_txpwr; /* mBm */
};

struct op_class_pref_t {
	u8 class_id;
	enum band_type band;
	enum opc_bw bw;
	u8 ch_num; /* number of chs */
	u8 op_ch_num; /* channel number which is not static non operable */
	u8 ir_ch_num; /* channel number which can init radiation */
	struct op_ch_t chs[];
};

struct rf_ctl_t;
int rtw_rfctl_op_class_pref_init(struct rf_ctl_t *rfctl, u8 band_bmp, u8 bw_bmp[]);
void rtw_rfctl_op_class_pref_deinit(struct rf_ctl_t *rfctl);

#define REG_BEACON_HINT		0
#define REG_TXPWR_CHANGE	1
#define REG_CHANGE		2

void op_class_pref_apply_regulatory(struct rf_ctl_t *rfctl, u8 reason);

struct rf_ctl_t;
void dump_cap_spt_op_class_ch(void *sel, struct rf_ctl_t *rfctl, bool detail);
void dump_reg_spt_op_class_ch(void *sel, struct rf_ctl_t *rfctl, bool detail);
void dump_cur_spt_op_class_ch(void *sel, struct rf_ctl_t *rfctl, bool detail);

#endif /* __RTW_OPC_PREF_H_ */
