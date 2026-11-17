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
#define _RTW_OPC_PREF_C_

#include <drv_types.h>

static struct op_class_pref_t *opc_pref_alloc(u8 op_class)
{
	int i, j;
	const struct op_class_t *opc;
	struct op_class_pref_t *opc_pref = NULL;
	u8 ch_num;

	for (i = 0; i < global_op_class_num; i++)
		if (global_op_class[i].class_id == op_class)
			break;

	if (i >= global_op_class_num)
		goto exit;

	opc = &global_op_class[i];
	ch_num = OPC_CH_LIST_LEN(opc);
	opc_pref = rtw_zmalloc(sizeof(*opc_pref) + (sizeof(struct op_ch_t) * ch_num));
	if (!opc_pref)
		goto exit;

	opc_pref->class_id = opc->class_id;
	opc_pref->band = opc->band;
	opc_pref->bw = opc->bw;

	for (j = 0; j < OPC_CH_LIST_LEN(opc); j++) {
		opc_pref->chs[j].ch = OPC_CH_LIST_CH(opc, j);
		opc_pref->chs[j].static_non_op = 1;
		opc_pref->chs[j].no_ir = 1;
		opc_pref->chs[j].max_txpwr = UNSPECIFIED_MBM;
	}
	opc_pref->ch_num = ch_num;

exit:
	return opc_pref;
}

static void opc_pref_free(struct op_class_pref_t *opc_pref)
{
	rtw_mfree(opc_pref, sizeof(*opc_pref) + (sizeof(struct op_ch_t) * opc_pref->ch_num));
}

int rtw_rfctl_op_class_pref_init(struct rf_ctl_t *rfctl, u8 band_bmp, u8 bw_bmp[])
{
	u8 bw;
	struct op_class_pref_t *opc_pref;
	int i;
	u8 op_class_num = 0;
	int ret = _FAIL;

	rfctl->spt_op_class_ch = rtw_zmalloc(sizeof(struct op_class_pref_t *) * global_op_class_num);
	if (!rfctl->spt_op_class_ch) {
		RTW_ERR("%s alloc rfctl->spt_op_class_ch fail\n", __func__);
		goto exit;
	}

	for (i = 0; i < global_op_class_num; i++) {
		if (!(band_bmp & band_to_band_cap(global_op_class[i].band)))
			continue;

		bw = opc_bw_to_ch_width(global_op_class[i].bw);
		if (bw == CHANNEL_WIDTH_MAX
			|| bw == CHANNEL_WIDTH_80_80 /* TODO */
		)
			continue;

		if (!(bw_bmp[global_op_class[i].band] & ch_width_to_bw_cap(bw)))
			continue;

		opc_pref = opc_pref_alloc(global_op_class[i].class_id);
		if (!opc_pref) {
			RTW_ERR("%s opc_pref_alloc(%u) fail\n", __func__, global_op_class[i].class_id);
			goto exit;
		}

		if (opc_pref->ch_num) {
			rfctl->spt_op_class_ch[i] = opc_pref;
			op_class_num++;
		} else
			opc_pref_free(opc_pref);
	}

	rfctl->cap_spt_op_class_num = op_class_num;
	ret = _SUCCESS;

exit:
	if (ret != _SUCCESS)
		rtw_rfctl_op_class_pref_deinit(rfctl);

	return ret;
}

void rtw_rfctl_op_class_pref_deinit(struct rf_ctl_t *rfctl)
{
	int i;

	if (!rfctl->spt_op_class_ch)
		return;

	for (i = 0; i < global_op_class_num; i++) {
		if (rfctl->spt_op_class_ch[i]) {
			opc_pref_free(rfctl->spt_op_class_ch[i]);
			rfctl->spt_op_class_ch[i] = NULL;
		}
	}

	rtw_mfree(rfctl->spt_op_class_ch, sizeof(struct op_class_pref_t *) * global_op_class_num);
	rfctl->spt_op_class_ch = NULL;
}

void op_class_pref_apply_regulatory(struct rf_ctl_t *rfctl, u8 reason)
{
	struct rtw_chset *chset = &rfctl->chset;
	u8 ch, bw, offset, cch;
	struct op_class_pref_t *opc_pref;
	int i, j;
	u8 reg_op_class_num = 0;
	u8 op_class_num = 0;

	for (i = 0; i < global_op_class_num; i++) {
		if (!rfctl->spt_op_class_ch[i])
			continue;
		opc_pref = rfctl->spt_op_class_ch[i];

		/* reset all channel */
		for (j = 0; j < opc_pref->ch_num; j++) {
			if (reason >= REG_CHANGE)
				opc_pref->chs[j].static_non_op = 1;
			if (reason != REG_TXPWR_CHANGE)
				opc_pref->chs[j].no_ir = 1;
			if (reason >= REG_TXPWR_CHANGE)
				opc_pref->chs[j].max_txpwr = UNSPECIFIED_MBM;
		}
		if (reason >= REG_CHANGE)
			opc_pref->op_ch_num = 0;
		if (reason != REG_TXPWR_CHANGE)
			opc_pref->ir_ch_num = 0;

		switch (opc_pref->bw) {
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
			continue;
		}

		if (!RFCTL_REG_EN_11AC(rfctl)
			&& (bw == CHANNEL_WIDTH_80 || bw == CHANNEL_WIDTH_160))
			continue;

		for (j = 0; j < opc_pref->ch_num; j++) {
			u8 *op_chs;
			u8 op_ch_num;
			u8 k, l;
			int chset_idx;

			ch = opc_pref->chs[j].ch;

			if (reason >= REG_TXPWR_CHANGE)
				opc_pref->chs[j].max_txpwr = rtw_rfctl_get_reg_max_txpwr_mbm(rfctl, opc_pref->band, ch, bw, offset, 1);

			if (reason == REG_TXPWR_CHANGE)
				continue;

			cch = rtw_get_center_ch_by_band(opc_pref->band, ch ,bw, offset);
			if (!cch)
				continue;

			if (!rtw_get_op_chs_by_bcch_bw(opc_pref->band, cch, bw, &op_chs, &op_ch_num))
				continue;

			for (k = 0, l = 0; k < op_ch_num; k++) {
				chset_idx = rtw_chset_search_bch(chset, opc_pref->band, *(op_chs + k));
				if (chset_idx == -1)
					break;
				if (bw >= CHANNEL_WIDTH_40) {
					if ((chset->chs[chset_idx].flags & RTW_CHF_NO_HT40U) && k % 2 == 0)
						break;
					if ((chset->chs[chset_idx].flags & RTW_CHF_NO_HT40L) && k % 2 == 1)
						break;
				}
				if (bw >= CHANNEL_WIDTH_80 && (chset->chs[chset_idx].flags & RTW_CHF_NO_80MHZ))
					break;
				if (bw >= CHANNEL_WIDTH_160 && (chset->chs[chset_idx].flags & RTW_CHF_NO_160MHZ))
					break;
				if ((chset->chs[chset_idx].flags & RTW_CHF_DFS) && !rtw_rfctl_radar_detect_supported(rfctl))
					continue;
				if (chset->chs[chset_idx].flags & RTW_CHF_NO_IR)
					continue;
				l++;
			}
			if (k < op_ch_num)
				continue;

			if (reason >= REG_CHANGE) {
				opc_pref->chs[j].static_non_op = 0;
				opc_pref->op_ch_num++;
			}

			if (l >= op_ch_num) {
				opc_pref->chs[j].no_ir = 0;
				opc_pref->ir_ch_num++;
			}
		}

		if (opc_pref->op_ch_num)
			reg_op_class_num++;
		if (opc_pref->ir_ch_num)
			op_class_num++;
	}

	rfctl->reg_spt_op_class_num = reg_op_class_num;
	rfctl->cur_spt_op_class_num = op_class_num;
}

static void dump_opc_pref_single(void *sel, struct op_class_pref_t *opc_pref, bool show_snon_ocp, bool show_no_ir, bool detail)
{
	u8 i;

	if (!show_snon_ocp && !opc_pref->op_ch_num)
		return;
	if (!show_no_ir && !opc_pref->ir_ch_num)
		return;

	RTW_PRINT_SEL(sel, "%5u %4s %7s"
		, opc_pref->class_id
		, band_str(opc_pref->band)
		, opc_bw_str(opc_pref->bw));
	for (i = 0; i < opc_pref->ch_num; i++) {
		if ((show_snon_ocp || !opc_pref->chs[i].static_non_op)
			&& (show_no_ir || !opc_pref->chs[i].no_ir)
		) {
			if (detail)
				_RTW_PRINT_SEL(sel, " %4u", opc_pref->chs[i].ch);
			else
				_RTW_PRINT_SEL(sel, " %u", opc_pref->chs[i].ch);
		}
	}
	_RTW_PRINT_SEL(sel, "\n");

	if (!detail)
		return;

	RTW_PRINT_SEL(sel, "                  ");
	for (i = 0; i < opc_pref->ch_num; i++) {
		if ((show_snon_ocp || !opc_pref->chs[i].static_non_op)
			&& (show_no_ir || !opc_pref->chs[i].no_ir)
		) {
			_RTW_PRINT_SEL(sel, "   %c%c"
				, opc_pref->chs[i].no_ir ? ' ' : 'I'
				, opc_pref->chs[i].static_non_op ? ' ' : 'E'
			);
		}
	}
	_RTW_PRINT_SEL(sel, "\n");

	RTW_PRINT_SEL(sel, "                  ");
	for (i = 0; i < opc_pref->ch_num; i++) {
		if ((show_snon_ocp || !opc_pref->chs[i].static_non_op)
			&& (show_no_ir || !opc_pref->chs[i].no_ir)
		) {
			if (opc_pref->chs[i].max_txpwr == UNSPECIFIED_MBM)
				_RTW_PRINT_SEL(sel, "     ");
			else
				_RTW_PRINT_SEL(sel, " %4d", opc_pref->chs[i].max_txpwr);
		}
	}
	_RTW_PRINT_SEL(sel, "\n");
}

void dump_cap_spt_op_class_ch(void *sel, struct rf_ctl_t *rfctl, bool detail)
{
	u8 i;

	dump_op_class_ch_title(sel);

	for (i = 0; i < global_op_class_num; i++) {
		if (!rfctl->spt_op_class_ch[i])
			continue;
		dump_opc_pref_single(sel, rfctl->spt_op_class_ch[i], 1, 1, detail);
	}

	RTW_PRINT_SEL(sel, "op_class number:%d\n", rfctl->cap_spt_op_class_num);
}

void dump_reg_spt_op_class_ch(void *sel, struct rf_ctl_t *rfctl, bool detail)
{
	u8 i;

	dump_op_class_ch_title(sel);

	for (i = 0; i < global_op_class_num; i++) {
		if (!rfctl->spt_op_class_ch[i])
			continue;
		dump_opc_pref_single(sel, rfctl->spt_op_class_ch[i], 0, 1, detail);
	}

	RTW_PRINT_SEL(sel, "op_class number:%d\n", rfctl->reg_spt_op_class_num);
}

void dump_cur_spt_op_class_ch(void *sel, struct rf_ctl_t *rfctl, bool detail)
{
	u8 i;

	dump_op_class_ch_title(sel);

	for (i = 0; i < global_op_class_num; i++) {
		if (!rfctl->spt_op_class_ch[i])
			continue;
		dump_opc_pref_single(sel, rfctl->spt_op_class_ch[i], 0, 0, detail);
	}

	RTW_PRINT_SEL(sel, "op_class number:%d\n", rfctl->cur_spt_op_class_num);
}
