/******************************************************************************
 *
 * Copyright(c) 2019 Realtek Corporation. All rights reserved.
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
 ******************************************************************************/
#include "cmac_tx.h"
#include "mac_priv.h"

#define MAC_AX_MULT32_SH 5
#define MAC_AX_MULT16_SH 4

static u32 stop_macid_ctn(struct mac_ax_adapter *adapter,
			  struct mac_role_tbl *role,
			  struct mac_ax_sch_tx_en_cfg *bak);
static u32 tx_idle_sel_ck_b(struct mac_ax_adapter *adapter,
			    enum ptcl_tx_sel sel, u8 band);
static u32 band_idle_ck(struct mac_ax_adapter *adapter, u8 band);
static void sch_2_u16(struct mac_ax_adapter *adapter,
		      struct mac_ax_sch_tx_en *tx_en, u16 *val16);
static void sch_2_u32(struct mac_ax_adapter *adapter,
		      struct mac_ax_sch_tx_en *tx_en, u32 *val32);
static u32 h2c_usr_edca(struct mac_ax_adapter *adapter,
			struct mac_ax_usr_edca_param *param);
static u32 h2c_usr_tx_rpt(struct mac_ax_adapter *adapter,
			  struct mac_ax_usr_tx_rpt_cfg *cfg);
static u32 tx_duty_h2c(struct mac_ax_adapter *adapter,
		       u16 pause_intvl, u16 tx_intvl);
static u32 h2c_usr_frame_to_act(struct mac_ax_adapter *adapter,
				struct mac_ax_usr_frame_to_act_cfg *param);
u32 mac_get_tx_cnt(struct mac_ax_adapter *adapter,
		   struct mac_ax_tx_cnt *cnt)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 txcnt_addr;
	u32 val32;
	u16 val16;
	u8 sel;

	if (cnt->band != 0 && cnt->band != 1)
		return MACNOITEM;
	if (check_mac_en(adapter, cnt->band, MAC_AX_CMAC_SEL))
		return MACHWNOTEN;
	txcnt_addr = (cnt->band == MAC_AX_BAND_0) ?
		      R_AX_TX_PPDU_CNT : R_AX_TX_PPDU_CNT_C1;
	for (sel = 0; sel < MAC_AX_TX_ALLTYPE; sel++) {
		val16 = MAC_REG_R16(txcnt_addr);
		val16 = SET_CLR_WORD(val16, sel, B_AX_PPDU_CNT_IDX);
		MAC_REG_W16(txcnt_addr, val16);
		PLTFM_DELAY_US(1000);
		val32 = MAC_REG_R32(txcnt_addr);
		cnt->txcnt[sel] = GET_FIELD(val32, B_AX_TX_PPDU_CNT);
	}
	return MACSUCCESS;
}

u32 mac_clr_tx_cnt(struct mac_ax_adapter *adapter,
		   struct mac_ax_tx_cnt *cnt)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u16 txcnt_addr;
	u16 val16;
	u16 to;
	u8 i;

	if (cnt->band != 0 && cnt->band != 1)
		return MACNOITEM;
	if (check_mac_en(adapter, cnt->band, MAC_AX_CMAC_SEL))
		return MACHWNOTEN;
	if (cnt->sel > MAC_AX_TX_ALLTYPE)
		return MACNOITEM;

	txcnt_addr = (cnt->band == MAC_AX_BAND_0) ?
		      R_AX_TX_PPDU_CNT : R_AX_TX_PPDU_CNT_C1;

#if MAC_AX_FW_REG_OFLD
	u32 ret;

	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		for (i = 0; i < MAC_AX_TX_ALLTYPE; i++) {
			if (cnt->sel == MAC_AX_TX_ALLTYPE || i == cnt->sel) {
				ret = MAC_REG_W_OFLD(txcnt_addr,
						     B_AX_PPDU_CNT_RIDX_MSK <<
						     B_AX_PPDU_CNT_RIDX_SH,
						     i, 0);
				if (ret != MACSUCCESS) {
					PLTFM_MSG_ERR("%s: write offload fail;"
						      "offset: %u, ret: %u\n",
						      __func__, txcnt_addr, ret);
					return ret;
				}
				ret = MAC_REG_W_OFLD(txcnt_addr, B_AX_RST_PPDU_CNT,
						     1, 0);
				if (ret != MACSUCCESS) {
					PLTFM_MSG_ERR("%s: write offload fail;"
						      "offset: %u, ret: %u\n",
						      __func__, txcnt_addr, ret);
					return ret;
				}
				ret = MAC_REG_P_OFLD(txcnt_addr, B_AX_RST_PPDU_CNT, 0,
						     (cnt->sel != MAC_AX_TX_ALLTYPE ||
						     i == MAC_AX_TX_ALLTYPE - 1) ?
						     1 : 0);
				if (ret != MACSUCCESS) {
					PLTFM_MSG_ERR("%s: poll offload fail;"
						      "offset: %u, ret: %u\n",
						      __func__, txcnt_addr, ret);
					return ret;
				}
			}
		}
		return MACSUCCESS;
	}
#endif
	to = 1000;
	for (i = 0; i < MAC_AX_TX_ALLTYPE; i++) {
		if (cnt->sel == MAC_AX_TX_ALLTYPE || i == cnt->sel) {
			val16 = MAC_REG_R16(txcnt_addr);
			val16 = SET_CLR_WORD(val16, i, B_AX_PPDU_CNT_RIDX) |
					     B_AX_RST_PPDU_CNT;
			MAC_REG_W16(txcnt_addr, val16);
			while (to--) {
				val16 = MAC_REG_R16(txcnt_addr);
				if (!(val16 & B_AX_RST_PPDU_CNT))
					break;
				PLTFM_DELAY_US(5);
			}
			if (to == 0)
				return MACPOLLTO;
		}
	}
	return MACSUCCESS;
}

u32 set_hw_ampdu_cfg(struct mac_ax_adapter *adapter,
		     struct mac_ax_ampdu_cfg *cfg)
{
	u16 max_agg_num;
	u8 band;
	u32 ret;
	u32 bk_addr, agg_addr;
	u32 val32;
	u8 val8, txtime_limit, max_agg_time;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	band = cfg->band;
	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	PLTFM_MSG_ALWAYS("[INFO] max_agg_time is not controled by %s", __func__);

	max_agg_num = cfg->max_agg_num;
	max_agg_time = cfg->max_agg_time_32us;
	txtime_limit = adapter->hw_info->max_agg_txtime_reg;

	bk_addr = band ? R_AX_AGG_BK_0_C1 : R_AX_AGG_BK_0;
	agg_addr = band ? R_AX_AMPDU_AGG_LIMIT_C1 : R_AX_AMPDU_AGG_LIMIT;

	val8 = MAC_REG_R8(bk_addr);
	switch (cfg->wdbk_mode) {
	case MAC_AX_WDBK_MODE_SINGLE_BK:
		val8 &= ~B_AX_WDBK_CFG;
		break;
	case MAC_AX_WDBK_MODE_GRP_BK:
		val8 |= B_AX_WDBK_CFG;
		break;
	default:
		return MACNOITEM;
	}

	switch (cfg->rty_bk_mode) {
	case MAC_AX_RTY_BK_MODE_AGG:
		val8 &= ~(B_AX_EN_RTY_BK | B_AX_EN_RTY_BK_COD);
		break;
	case MAC_AX_RTY_BK_MODE_RATE_FB:
		val8 &= ~(B_AX_EN_RTY_BK);
		val8 |= B_AX_EN_RTY_BK_COD;
		break;
	case MAC_AX_RTY_BK_MODE_BK:
		val8 |= B_AX_EN_RTY_BK | B_AX_EN_RTY_BK_COD;
		break;
	default:
		return MACNOITEM;
	}
	MAC_REG_W8(bk_addr, val8);

	val32 = MAC_REG_R32(agg_addr);
	if (max_agg_num > 0 && max_agg_num <= 0x100)
		val32 = SET_CLR_WORD(val32, max_agg_num - 1, B_AX_MAX_AGG_NUM);
	else
		return MACSETVALERR;
	if (max_agg_time > 0 && max_agg_time <= txtime_limit)
		val32 = SET_CLR_WORD(val32, max_agg_time, B_AX_AMPDU_MAX_TIME);
	else
		val32 = SET_CLR_WORD(val32, txtime_limit, B_AX_AMPDU_MAX_TIME);
	MAC_REG_W32(agg_addr, val32);

	return MACSUCCESS;
}

u32 set_hw_usr_tx_rpt_cfg(struct mac_ax_adapter *adapter,
			  struct mac_ax_usr_tx_rpt_cfg *cfg)
{
	u32 ret;

	ret = h2c_usr_tx_rpt(adapter, cfg);
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

u32 set_hw_usr_frame_te_act_cfg(struct mac_ax_adapter *adapter,
				struct mac_ax_usr_frame_to_act_cfg *cfg)
{
	u32 ret;

	ret = h2c_usr_frame_to_act(adapter, cfg);
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

u32 set_hw_usr_edca_param(struct mac_ax_adapter *adapter,
			  struct mac_ax_usr_edca_param *param)
{
	u32 ret;

	ret = h2c_usr_edca(adapter, param);
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

u32 set_hw_edca_param(struct mac_ax_adapter *adapter,
		      struct mac_ax_edca_param *param)
{
	u32 val32;
	u32 reg_edca;
	u32 ret;
	u16 val16;
	enum mac_ax_cmac_path_sel path;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	ret = check_mac_en(adapter, param->band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	ret = get_edca_addr(adapter, param, &reg_edca);
	if (ret != MACSUCCESS)
		return ret;

	path = param->path;

#if MAC_AX_FW_REG_OFLD
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		if (path == MAC_AX_CMAC_PATH_SEL_MG0_1 ||
		    path == MAC_AX_CMAC_PATH_SEL_MG2 ||
		    path == MAC_AX_CMAC_PATH_SEL_BCN) {
			val16 = SET_WORD((param->ecw_max << 4) | param->ecw_min,
					 B_AX_BE_0_CW) |
				SET_WORD(param->aifs_us, B_AX_BE_0_AIFS);
			ret = MAC_REG_W16_OFLD((u16)reg_edca, val16, 1);
			if (ret != MACSUCCESS)
				return ret;
		} else {
			val32 = SET_WORD(param->txop_32us, B_AX_BE_0_TXOPLMT) |
				SET_WORD((param->ecw_max << 4) | param->ecw_min,
					 B_AX_BE_0_CW) |
				SET_WORD(param->aifs_us, B_AX_BE_0_AIFS);
			ret = MAC_REG_W32_OFLD((u16)reg_edca, val32, 1);
			if (ret != MACSUCCESS)
				return ret;
		}
		return MACSUCCESS;
	}
#endif

	if (path == MAC_AX_CMAC_PATH_SEL_MG0_1 ||
	    path == MAC_AX_CMAC_PATH_SEL_MG2 ||
	    path == MAC_AX_CMAC_PATH_SEL_BCN) {
		val16 = SET_WORD((param->ecw_max << 4) | param->ecw_min,
				 B_AX_BE_0_CW) |
			SET_WORD(param->aifs_us, B_AX_BE_0_AIFS);
		MAC_REG_W16(reg_edca, val16);
	} else {
		val32 = SET_WORD(param->txop_32us, B_AX_BE_0_TXOPLMT) |
			SET_WORD((param->ecw_max << 4) | param->ecw_min,
				 B_AX_BE_0_CW) |
			SET_WORD(param->aifs_us, B_AX_BE_0_AIFS);
		MAC_REG_W32(reg_edca, val32);
	}

	return MACSUCCESS;
}

u32 get_hw_edca_param(struct mac_ax_adapter *adapter,
		      struct mac_ax_edca_param *param)
{
	u32 val32;
	u32 reg_edca;
	u32 ret;
	u16 val16;
	enum mac_ax_cmac_path_sel path;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	ret = check_mac_en(adapter, param->band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	ret = get_edca_addr(adapter, param, &reg_edca);
	if (ret != MACSUCCESS)
		return ret;

	path = param->path;

	if (path == MAC_AX_CMAC_PATH_SEL_MG0_1 ||
	    path == MAC_AX_CMAC_PATH_SEL_MG2 ||
	    path == MAC_AX_CMAC_PATH_SEL_BCN) {
		val16 = MAC_REG_R16(reg_edca);
		param->txop_32us = 0;
		param->aifs_us = GET_FIELD(val16, B_AX_BE_0_AIFS);
		param->ecw_max = (GET_FIELD(val16, B_AX_BE_0_CW) & 0xF0) >> 4;
		param->ecw_min = GET_FIELD(val16, B_AX_BE_0_CW) & 0x0F;
	} else {
		val32 = MAC_REG_R32(reg_edca);
		param->txop_32us = GET_FIELD(val32, B_AX_BE_0_TXOPLMT);
		param->aifs_us = GET_FIELD(val32, B_AX_BE_0_AIFS);
		param->ecw_max = (GET_FIELD(val32, B_AX_BE_0_CW) & 0xF0) >> 4;
		param->ecw_min = GET_FIELD(val32, B_AX_BE_0_CW) & 0x0F;
	}

	return MACSUCCESS;
}

u32 set_hw_edcca_param(struct mac_ax_adapter *adapter,
		       struct mac_ax_edcca_param *param)
{
	u32 reg_cca_ctl = 0;
	u32 ret;
	enum mac_ax_edcca_sel sel;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	ret = check_mac_en(adapter, param->band, MAC_AX_CMAC_SEL);
	if (ret)
		return ret;

#if MAC_AX_FW_REG_OFLD
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		sel = param->sel;
		if (sel == MAC_AX_EDCCA_IN_TB_CHK) {
			if (param->tb_check_en)
				reg_cca_ctl |= B_AX_TB_CHK_EDCCA;
			else
				reg_cca_ctl &= ~B_AX_TB_CHK_EDCCA;
		}
		if (sel == MAC_AX_EDCCA_IN_SIFS_CHK) {
			if (param->sifs_check_en)
				reg_cca_ctl |= B_AX_SIFS_CHK_EDCCA;
			else
				reg_cca_ctl &= ~B_AX_SIFS_CHK_EDCCA;
		}
		if (sel == MAC_AX_EDCCA_IN_CTN_CHK) {
			if (param->ctn_check_en)
				reg_cca_ctl |= B_AX_CTN_CHK_EDCCA;
			else
				reg_cca_ctl &= ~B_AX_CTN_CHK_EDCCA;
		} else {
			return MACNOITEM;
		}
		if (param->band)
			ret = MAC_REG_W32_OFLD((u16)R_AX_CCA_CONTROL_C1,
					       reg_cca_ctl, 1);
		else
			ret = MAC_REG_W32_OFLD((u16)R_AX_CCA_CONTROL,
					       reg_cca_ctl, 1);
		return ret;
	}
#endif

	if (param->band)
		reg_cca_ctl = MAC_REG_R32(R_AX_CCA_CONTROL_C1);
	else
		reg_cca_ctl = MAC_REG_R32(R_AX_CCA_CONTROL);

	sel = param->sel;
	if (sel == MAC_AX_EDCCA_IN_TB_CHK) {
		if (param->tb_check_en)
			reg_cca_ctl |= B_AX_TB_CHK_EDCCA;
		else
			reg_cca_ctl &= ~B_AX_TB_CHK_EDCCA;
	}
	if (sel == MAC_AX_EDCCA_IN_SIFS_CHK) {
		if (param->sifs_check_en)
			reg_cca_ctl |= B_AX_SIFS_CHK_EDCCA;
		else
			reg_cca_ctl &= ~B_AX_SIFS_CHK_EDCCA;
	}
	if (sel == MAC_AX_EDCCA_IN_CTN_CHK) {
		if (param->ctn_check_en)
			reg_cca_ctl |= B_AX_CTN_CHK_EDCCA;
		else
			reg_cca_ctl &= ~B_AX_CTN_CHK_EDCCA;
	} else {
		return MACNOITEM;
	}

	if (param->band)
		MAC_REG_W32(R_AX_CCA_CONTROL_C1, reg_cca_ctl);
	else
		MAC_REG_W32(R_AX_CCA_CONTROL, reg_cca_ctl);

	return MACSUCCESS;
}

u32 set_hw_muedca_param(struct mac_ax_adapter *adapter,
			struct mac_ax_muedca_param *param)
{
	u32 val32;
	u32 reg_edca;
	u32 ret;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	ret = check_mac_en(adapter, param->band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	ret = get_muedca_param_addr(adapter, param, &reg_edca);
	if (ret != MACSUCCESS)
		return ret;

#if MAC_AX_FW_REG_OFLD
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		val32 = SET_WORD(param->muedca_timer_32us,
				 B_AX_MUEDCA_BE_PARAM_0_TIMER) |
			SET_WORD((param->ecw_max << 4) | param->ecw_min,
				 B_AX_MUEDCA_BE_PARAM_0_CW) |
			SET_WORD(param->aifs_us, B_AX_MUEDCA_BE_PARAM_0_AIFS);
		ret = MAC_REG_W32_OFLD((u16)reg_edca, val32, 1);
		if (ret != MACSUCCESS)
			return ret;

		return MACSUCCESS;
	}
#endif

	val32 = SET_WORD(param->muedca_timer_32us,
			 B_AX_MUEDCA_BE_PARAM_0_TIMER) |
		SET_WORD((param->ecw_max << 4) | param->ecw_min,
			 B_AX_MUEDCA_BE_PARAM_0_CW) |
		SET_WORD(param->aifs_us, B_AX_MUEDCA_BE_PARAM_0_AIFS);
	MAC_REG_W32(reg_edca, val32);

	return MACSUCCESS;
}

u32 set_hw_muedca_ctrl(struct mac_ax_adapter *adapter,
		       struct mac_ax_muedca_cfg *cfg)
{
	u32 ret;
	u8 band;
	u16 val16;
	u32 reg_en;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	band = cfg->band;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	reg_en = band ? R_AX_MUEDCA_EN_C1 : R_AX_MUEDCA_EN;

#if MAC_AX_FW_REG_OFLD
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		val16 = 0;
		if (cfg->wmm_sel == MAC_AX_CMAC_WMM1_SEL) {
			ret = MAC_REG_W_OFLD((u16)reg_en, B_AX_MUEDCA_WMM_SEL,
					     1, 0);
			if (ret != MACSUCCESS)
				return ret;
		} else {
			ret = MAC_REG_W_OFLD((u16)reg_en, B_AX_MUEDCA_WMM_SEL,
					     0, 0);
			if (ret != MACSUCCESS)
				return ret;
		}

		ret = MAC_REG_W_OFLD((u16)reg_en, B_AX_MUEDCA_EN_0,
				     cfg->countdown_en, 0);
		if (ret != MACSUCCESS)
			return ret;

		ret = MAC_REG_W_OFLD((u16)reg_en, B_AX_SET_MUEDCATIMER_TF_0,
				     cfg->tb_update_en, 1);
		if (ret != MACSUCCESS)
			return ret;

		return MACSUCCESS;
	}
#endif

	val16 = MAC_REG_R16(reg_en);

	if (cfg->wmm_sel == MAC_AX_CMAC_WMM1_SEL)
		val16 |= B_AX_MUEDCA_WMM_SEL;
	else
		val16 &= ~B_AX_MUEDCA_WMM_SEL;

	if (cfg->countdown_en)
		val16 |= B_AX_MUEDCA_EN_0;
	else
		val16 &= ~B_AX_MUEDCA_EN_0;

	if (cfg->tb_update_en)
		val16 |= B_AX_SET_MUEDCATIMER_TF_0;
	else
		val16 &= ~B_AX_SET_MUEDCATIMER_TF_0;

	MAC_REG_W16(reg_en, val16);

	return MACSUCCESS;
}

u32 set_hw_tb_ppdu_ctrl(struct mac_ax_adapter *adapter,
			struct mac_ax_tb_ppdu_ctrl *ctrl)
{
	u16 val16;
	u8 pri_ac;
	u32 ret;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	ret = check_mac_en(adapter, ctrl->band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	switch (ctrl->pri_ac) {
	case MAC_AX_CMAC_AC_SEL_BE:
		pri_ac = 0;
		break;
	case MAC_AX_CMAC_AC_SEL_BK:
		pri_ac = 1;
		break;
	case MAC_AX_CMAC_AC_SEL_VI:
		pri_ac = 2;
		break;
	case MAC_AX_CMAC_AC_SEL_VO:
		pri_ac = 3;
		break;
	default:
		return MACNOITEM;
	}

	val16 = MAC_REG_R16(ctrl->band ? R_AX_TB_PPDU_CTRL_C1 :
			    R_AX_TB_PPDU_CTRL);
	val16 &= ~(B_AX_TB_PPDU_BE_DIS | B_AX_TB_PPDU_BK_DIS |
		   B_AX_TB_PPDU_VI_DIS | B_AX_TB_PPDU_VO_DIS);
	val16 |= (ctrl->be_dis ? B_AX_TB_PPDU_BE_DIS : 0) |
		 (ctrl->bk_dis ? B_AX_TB_PPDU_BK_DIS : 0) |
		 (ctrl->vi_dis ? B_AX_TB_PPDU_VI_DIS : 0) |
		 (ctrl->vo_dis ? B_AX_TB_PPDU_VO_DIS : 0);
	val16 = SET_CLR_WORD(val16, pri_ac, B_AX_SW_PREFER_AC);
	MAC_REG_W16(ctrl->band ? R_AX_TB_PPDU_CTRL_C1 : R_AX_TB_PPDU_CTRL,
		    val16);

	return MACSUCCESS;
}

u32 get_hw_tb_ppdu_ctrl(struct mac_ax_adapter *adapter,
			struct mac_ax_tb_ppdu_ctrl *ctrl)
{
	u16 val16;
	u8 pri_ac;
	u32 ret;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	ret = check_mac_en(adapter, ctrl->band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	val16 = MAC_REG_R16(ctrl->band ? R_AX_TB_PPDU_CTRL_C1 :
			    R_AX_TB_PPDU_CTRL);
	ctrl->be_dis = (val16 & B_AX_TB_PPDU_BE_DIS) ? 1 : 0;
	ctrl->bk_dis = (val16 & B_AX_TB_PPDU_BK_DIS) ? 1 : 0;
	ctrl->vi_dis = (val16 & B_AX_TB_PPDU_VI_DIS) ? 1 : 0;
	ctrl->vo_dis = (val16 & B_AX_TB_PPDU_VO_DIS) ? 1 : 0;
	pri_ac = GET_FIELD(val16, B_AX_SW_PREFER_AC);

	switch (pri_ac) {
	case 0:
		ctrl->pri_ac = MAC_AX_CMAC_AC_SEL_BE;
		break;
	case 1:
		ctrl->pri_ac = MAC_AX_CMAC_AC_SEL_BK;
		break;
	case 2:
		ctrl->pri_ac = MAC_AX_CMAC_AC_SEL_VI;
		break;
	case 3:
		ctrl->pri_ac = MAC_AX_CMAC_AC_SEL_VO;
		break;
	}

	return MACSUCCESS;
}

u32 set_hw_sch_tx_en(struct mac_ax_adapter *adapter,
		     struct mac_ax_sch_tx_en_cfg *cfg)
{
	u16 val16;
	u8 band;
	u32 ret;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u16 tx_en_u16;
	u16 mask_u16;
	u32 tx_en_u32, mask_u32, val32;
	struct mac_ax_sch_tx_en tx_en;
	struct mac_ax_sch_tx_en tx_en_mask;
	u8 chip_id = adapter->drv_info->sw_chip_id;

	band = cfg->band;
	tx_en = cfg->tx_en;
	tx_en_mask = cfg->tx_en_mask;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	sch_2_u16(adapter, &tx_en, &tx_en_u16);
	sch_2_u16(adapter, &tx_en_mask, &mask_u16);

	if (chip_id == MAC_AX_CHIP_ID_8852A ||
	    chip_id == MAC_AX_CHIP_ID_8852B ||
	    chip_id == MAC_AX_CHIP_ID_8851B ||
	    chip_id == MAC_AX_CHIP_ID_8852BT) {
		if (adapter->sm.fwdl != MAC_AX_FWDL_INIT_RDY) {
			val16 = MAC_REG_R16(band ? R_AX_CTN_TXEN_C1 :
					    R_AX_CTN_TXEN);

			val16 = (tx_en_u16 & mask_u16) |
				(~(~tx_en_u16 & mask_u16) & val16);

			MAC_REG_W16(band ? R_AX_CTN_TXEN_C1 : R_AX_CTN_TXEN,
				    val16);
		} else {
			hw_sch_tx_en(adapter, band, tx_en_u16, mask_u16);
		}
	} else if (chip_id == MAC_AX_CHIP_ID_8852C ||
		   chip_id == MAC_AX_CHIP_ID_8192XB ||
		   chip_id == MAC_AX_CHIP_ID_8852D) {
		sch_2_u32(adapter, &tx_en, &tx_en_u32);
		sch_2_u32(adapter, &tx_en_mask, &mask_u32);

		val32 = MAC_REG_R32(band ? R_AX_CTN_DRV_TXEN_C1 :
				    R_AX_CTN_DRV_TXEN);

		val32 = (tx_en_u32 & mask_u32) |
			(~(~tx_en_u32 & mask_u32) & val32);

		MAC_REG_W32(band ? R_AX_CTN_DRV_TXEN_C1 :
			    R_AX_CTN_DRV_TXEN, val32);
	} else {
		return MACNOITEM;
	}

	return MACSUCCESS;
}

u32 hw_sch_tx_en_h2c_pkt(struct mac_ax_adapter *adapter, u8 band,
			 u16 tx_en_u16, u16 mask_u16)
{
	u32 ret = MACSUCCESS;
	struct fwcmd_sch_tx_en_pkt *write_ptr;
	struct h2c_info h2c_info = {0};

	if (adapter->sm.sch_tx_en_ofld != MAC_AX_OFLD_H2C_IDLE) {
		PLTFM_MSG_ERR("[ERR]SchTxEn PKT state machine not MAC_AX_OFLD_H2C_IDLE\n");
		return MACPROCERR;
	}

	adapter->sm.sch_tx_en_ofld = MAC_AX_OFLD_H2C_SENDING;

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_sch_tx_en_pkt);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_FW_OFLD;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_SCH_TX_EN_PKT;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 1;

	write_ptr = (struct fwcmd_sch_tx_en_pkt *)PLTFM_MALLOC(h2c_info.content_len);
	if (!write_ptr)
		return MACBUFALLOC;

	write_ptr->dword0 =
		cpu_to_le32(SET_WORD(tx_en_u16, FWCMD_H2C_H2CPKT_SCH_TX_PAUSE_TX_EN));
	write_ptr->dword1 =
		cpu_to_le32(SET_WORD(mask_u16, FWCMD_H2C_H2CPKT_SCH_TX_PAUSE_MASK) |
			    (band ? FWCMD_H2C_H2CPKT_SCH_TX_PAUSE_BAND : 0));

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)write_ptr);
	PLTFM_FREE(write_ptr, h2c_info.content_len);
	return ret;
}

static u32 get_io_latency(struct mac_ax_adapter *adapter, u8 band, u32 *io_latency_us)
{
	struct mac_ax_freerun free0, free1;
	u32 ret;

	free0.band = band;
	free1.band = band;
	ret = mac_get_freerun(adapter, &free0);
	if (ret != MACSUCCESS)
		return ret;
	ret = mac_get_freerun(adapter, &free1);
	if (ret != MACSUCCESS)
		return ret;
	*io_latency_us = (free1.freerun_l - free0.freerun_l) / 2;

	return MACSUCCESS;
}

u32 hw_sch_tx_en(struct mac_ax_adapter *adapter, u8 band,
		 u16 tx_en_u16, u16 mask_u16)
{
	u32 ret, cnt;
	struct mac_ax_h2creg_info h2c = {0};
	struct mac_ax_c2hreg_poll c2h = {0};
	u32 io_latency_us = 0;
	static u8 c2h_path = TXEN_PATH_C2H_TBD;

	if (c2h_path == TXEN_PATH_C2H_TBD) {
		if (adapter->env_info.intf == MAC_AX_INTF_PCIE)
			c2h_path = TXEN_PATH_C2H_REG;
		else if (adapter->env_info.intf != MAC_AX_INTF_USB)
			c2h_path = TXEN_PATH_C2H_PKT;
	}
	if (adapter->drv_stats.rx_ok && c2h_path != TXEN_PATH_C2H_REG) {
		ret = hw_sch_tx_en_h2c_pkt(adapter, band, tx_en_u16, mask_u16);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]SchTxEn PKT %d\n", ret);
			return ret;
		}
		/* Wait for C2H */
		cnt = TX_PAUSE_WAIT_PKT_CNT;

		while (--cnt) {
			if (adapter->sm.sch_tx_en_ofld == MAC_AX_OFLD_H2C_DONE)
				break;
			PLTFM_SLEEP_US(TXEN_RETRY_WAIT_PKT_US);
		}
		adapter->sm.sch_tx_en_ofld = MAC_AX_OFLD_H2C_IDLE;
		if (!cnt) {
			PLTFM_MSG_ERR("[ERR]SchTxEn DONE ACK timeout\n");
			if (c2h_path == TXEN_PATH_C2H_TBD) {
				ret = get_io_latency(adapter, band, &io_latency_us);
				if (ret != MACSUCCESS) {
					PLTFM_MSG_ERR("[ERR] %s: get_io_latency fail\n",
						      __func__);
					return MACPROCERR;
				}
				/* if IO too slow, c2h reg not feasible */
				c2h_path = io_latency_us > TXEN_IO_LATENCY_THRESHOLD_US ?
					   TXEN_PATH_C2H_PKT : TXEN_PATH_C2H_REG;
			}

			return MACPROCERR;
		}
		return MACSUCCESS;
	}

	h2c.id = FWCMD_H2CREG_FUNC_SCH_TX_EN;
	h2c.content_len = sizeof(struct sch_tx_en_h2creg);

	h2c.h2c_content.dword0 =
		SET_WORD(tx_en_u16, FWCMD_H2C_H2CREG_SCH_TX_PAUSE_TX_EN);
	h2c.h2c_content.dword1 =
		SET_WORD(mask_u16, FWCMD_H2C_H2CREG_SCH_TX_PAUSE_MASK) |
		    (band ? FWCMD_H2C_H2CREG_SCH_TX_PAUSE_BAND : 0);

	c2h.polling_id = FWCMD_C2HREG_FUNC_TX_PAUSE_RPT;
	c2h.retry_cnt = TX_PAUSE_WAIT_CNT;
	c2h.retry_wait_us = TXEN_RETRY_WAIT_US;

	ret = proc_msg_reg(adapter, &h2c, &c2h);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]hw sch tx_en proc msg reg %d\n", ret);
		return ret;
	}
	return MACSUCCESS;
}

u32 get_hw_sch_tx_en(struct mac_ax_adapter *adapter,
		     struct mac_ax_sch_tx_en_cfg *cfg)
{
	u8 band;
	u32 ret, val32;
	u16 val16;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	struct mac_ax_sch_tx_en tx_en;
	u8 chip_id = adapter->drv_info->sw_chip_id;

	band = cfg->band;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	if (chip_id == MAC_AX_CHIP_ID_8852A ||
	    chip_id == MAC_AX_CHIP_ID_8852B ||
	    chip_id == MAC_AX_CHIP_ID_8851B ||
	    chip_id == MAC_AX_CHIP_ID_8852BT) {
		val16 = MAC_REG_R16(band ? R_AX_CTN_TXEN_C1 : R_AX_CTN_TXEN);
		u16_2_sch(adapter, &tx_en, val16);
		cfg->tx_en = tx_en;
	} else if (chip_id == MAC_AX_CHIP_ID_8852C ||
		   chip_id == MAC_AX_CHIP_ID_8192XB ||
		   chip_id == MAC_AX_CHIP_ID_8852D) {
		val32 = MAC_REG_R32(band ? R_AX_CTN_DRV_TXEN_C1 :
				    R_AX_CTN_DRV_TXEN);
		u32_2_sch(adapter, &tx_en, val32);
		cfg->tx_en = tx_en;
	} else {
		return MACNOITEM;
	}

	return MACSUCCESS;
}

u32 set_hw_lifetime_cfg(struct mac_ax_adapter *adapter,
			struct mac_ax_lifetime_cfg *cfg)
{
	u32 ret;
	u8 band;
	u8 val8;
	u32 val32;
	u32 reg_time_0, reg_time_1, reg_time_2, reg_en;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	band = cfg->band;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	reg_time_0 = band ? R_AX_LIFETIME_0_C1 : R_AX_LIFETIME_0;
	reg_time_1 = band ? R_AX_LIFETIME_1_C1 : R_AX_LIFETIME_1;
	reg_time_2 = band ? R_AX_LIFETIME_2_C1 : R_AX_LIFETIME_2;
	reg_en = band ? R_AX_PTCL_COMMON_SETTING_0_C1 :
		 R_AX_PTCL_COMMON_SETTING_0;

#if MAC_AX_FW_REG_OFLD
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		if (cfg->en.acq_en || cfg->en.mgq_en) {
			ret = MAC_REG_W_OFLD(R_AX_TX_PASTE_TIMESTAMP_SETTING,
					     B_AX_HDT_TIMESTAMP_EN, 1, 0);
			if (ret != MACSUCCESS)
				return ret;
		}
		val32 = SET_WORD(cfg->val.acq_val_1, B_AX_PKT_LIFETIME_1) |
			SET_WORD(cfg->val.acq_val_2, B_AX_PKT_LIFETIME_2);
		ret = MAC_REG_W32_OFLD((u16)reg_time_0, val32, 0);
		if (ret != MACSUCCESS)
			return ret;

		val32 = SET_WORD(cfg->val.acq_val_3, B_AX_PKT_LIFETIME_3) |
			SET_WORD(cfg->val.acq_val_4, B_AX_PKT_LIFETIME_4);
		ret = MAC_REG_W32_OFLD((u16)reg_time_1, val32, 0);
		if (ret != MACSUCCESS)
			return ret;

		ret = MAC_REG_W16_OFLD((u16)reg_time_2, cfg->val.mgq_val, 0);
		if (ret != MACSUCCESS)
			return ret;

		ret = MAC_REG_W_OFLD((u16)reg_en, B_AX_LIFETIME_EN,
				     cfg->en.acq_en, 0);
		if (ret != MACSUCCESS)
			return ret;
		ret = MAC_REG_W_OFLD((u16)reg_en, B_AX_MGQ_LIFETIME_EN,
				     cfg->en.mgq_en, 1);
		if (ret != MACSUCCESS)
			return ret;

		return MACSUCCESS;
	}
#endif

	if (cfg->en.acq_en || cfg->en.mgq_en)
		MAC_REG_W8(R_AX_TX_PASTE_TIMESTAMP_SETTING,
			   MAC_REG_R8(R_AX_TX_PASTE_TIMESTAMP_SETTING) |
			   B_AX_HDT_TIMESTAMP_EN);

	val32 = SET_WORD(cfg->val.acq_val_1, B_AX_PKT_LIFETIME_1) |
		SET_WORD(cfg->val.acq_val_2, B_AX_PKT_LIFETIME_2);
	MAC_REG_W32(reg_time_0, val32);

	val32 = SET_WORD(cfg->val.acq_val_3, B_AX_PKT_LIFETIME_3) |
		SET_WORD(cfg->val.acq_val_4, B_AX_PKT_LIFETIME_4);
	MAC_REG_W32(reg_time_1, val32);

	MAC_REG_W16(reg_time_2, cfg->val.mgq_val);

	val8 = MAC_REG_R8(reg_en);
	val8 &=	~(B_AX_LIFETIME_EN | B_AX_MGQ_LIFETIME_EN);
	val8 |= (cfg->en.acq_en ? B_AX_LIFETIME_EN : 0) |
		(cfg->en.mgq_en ? B_AX_MGQ_LIFETIME_EN : 0);
	MAC_REG_W8(reg_en, val8);

	return MACSUCCESS;
}

u32 get_hw_lifetime_cfg(struct mac_ax_adapter *adapter,
			struct mac_ax_lifetime_cfg *cfg)
{
	u32 ret;
	u8 band;
	u8 val8;
	u32 val32;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	band = cfg->band;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	val8 = MAC_REG_R8(band ? R_AX_PTCL_COMMON_SETTING_0_C1 :
			  R_AX_PTCL_COMMON_SETTING_0);
	cfg->en.acq_en = (val8 & B_AX_LIFETIME_EN) ? 1 : 0;
	cfg->en.mgq_en = (val8 & B_AX_MGQ_LIFETIME_EN) ? 1 : 0;

	val32 = MAC_REG_R32(band ? R_AX_LIFETIME_0_C1 : R_AX_LIFETIME_0);
	cfg->val.acq_val_1 = GET_FIELD(val32, B_AX_PKT_LIFETIME_1);
	cfg->val.acq_val_2 = GET_FIELD(val32, B_AX_PKT_LIFETIME_2);

	val32 = MAC_REG_R32(band ? R_AX_LIFETIME_1_C1 : R_AX_LIFETIME_1);
	cfg->val.acq_val_3 = GET_FIELD(val32, B_AX_PKT_LIFETIME_3);
	cfg->val.acq_val_4 = GET_FIELD(val32, B_AX_PKT_LIFETIME_4);

	cfg->val.mgq_val = MAC_REG_R16(band ? R_AX_LIFETIME_2_C1 :
				       R_AX_LIFETIME_2);

	return MACSUCCESS;
}

/* this function is only for WFA Tx TB PPDU SIFS timing workaround*/
u32 set_hw_sifs_r2t_t2t(struct mac_ax_adapter *adapter,
			struct mac_ax_sifs_r2t_t2t_ctrl *ctrl)
{
	u32 reg, val32, ret, mactxen;
	u8 band;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	band = ctrl->band;
	mactxen = ctrl->mactxen;

	PLTFM_MSG_ERR("[WARN] %s is only for ", __func__);
	PLTFM_MSG_ERR("WFA Tx TB PPDU SIFS timing workaround\n");

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	if (mactxen > MACTXEN_MAX || mactxen < MACTXEN_MIN) {
		PLTFM_MSG_ALWAYS("[ERR] sifs_r2t_t2t mactxen violation %d ",
				 mactxen);
		PLTFM_MSG_ALWAYS("must be %d ~ %d\n", MACTXEN_MIN, MACTXEN_MAX);
		return MACFUNCINPUT;
	}

	reg = band == MAC_AX_BAND_1 ? R_AX_PREBKF_CFG_1_C1 : R_AX_PREBKF_CFG_1;
	val32 = MAC_REG_R32(reg);
	val32 = SET_CLR_WORD(val32, ctrl->mactxen, B_AX_SIFS_MACTXEN_T1);
	MAC_REG_W32(reg, val32);

	return MACSUCCESS;
}

u32 resume_sch_tx(struct mac_ax_adapter *adapter,
		  struct mac_ax_sch_tx_en_cfg *bak)
{
	u32 ret;

	u16_2_sch(adapter, &bak->tx_en_mask, 0xFFFF);
	ret = set_hw_sch_tx_en(adapter, bak);
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

u32 stop_macid_tx(struct mac_ax_adapter *adapter, struct mac_role_tbl *role,
		  enum tb_stop_sel stop_sel, struct macid_tx_bak *bak)
{
	u8 band;
	u32 ret;
	struct mac_ax_macid_pause_cfg pause;

	band = role->info.band;

	if (role->info.a_info.tf_trs) {
		bak->ac_dis_bak.band = band;
		ret = stop_ac_tb_tx(adapter, stop_sel, &bak->ac_dis_bak);
		if (ret != MACSUCCESS)
			return ret;
	}

	pause.macid = role->macid;
	pause.pause = 1;
	ret = set_macid_pause(adapter, &pause);
	if (ret != MACSUCCESS)
		return ret;

	bak->sch_bak.band = band;
	ret = stop_macid_ctn(adapter, role, &bak->sch_bak);
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

u32 resume_macid_tx(struct mac_ax_adapter *adapter, struct mac_role_tbl *role,
		    struct macid_tx_bak *bak)
{
	u32 ret;
	struct mac_ax_macid_pause_cfg pause_cfg;

	if (role->info.band == MAC_AX_BAND_0) {
		u16_2_sch(adapter, &bak->sch_bak.tx_en_mask, 0xFFFF);
		ret = set_hw_sch_tx_en(adapter, &bak->sch_bak);
		if (ret != MACSUCCESS)
			return ret;
	}

	if (role->info.a_info.tf_trs) {
		ret = set_hw_tb_ppdu_ctrl(adapter, &bak->ac_dis_bak);
		if (ret != MACSUCCESS)
			return ret;
	}

	pause_cfg.macid = role->macid;
	pause_cfg.pause = 0;
	ret = set_macid_pause(adapter, &pause_cfg);
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

u32 tx_idle_poll_macid(struct mac_ax_adapter *adapter,
		       struct mac_role_tbl *role)
{
	struct mac_ax_priv_ops *p_ops = adapter_to_priv_ops(adapter);

	return p_ops->macid_idle_ck(adapter, role);
}

u32 tx_idle_poll_band(struct mac_ax_adapter *adapter, u8 band)
{
	return band_idle_ck(adapter, band);
}

u32 tx_idle_poll_sel(struct mac_ax_adapter *adapter, enum ptcl_tx_sel sel,
		     u8 band)
{
	return tx_idle_sel_ck_b(adapter, sel, band);
}

u32 stop_ac_tb_tx(struct mac_ax_adapter *adapter, enum tb_stop_sel stop_sel,
		  struct mac_ax_tb_ppdu_ctrl *ac_dis_bak)
{
	u32 ret;
	struct mac_ax_tb_ppdu_ctrl ctrl = {0};

	ret = get_hw_tb_ppdu_ctrl(adapter, ac_dis_bak);
	if (ret != MACSUCCESS)
		return ret;

	ctrl.band = ac_dis_bak->band;
	ctrl.pri_ac = ac_dis_bak->pri_ac;
	ctrl.be_dis = 0;
	ctrl.bk_dis = 0;
	ctrl.vi_dis = 0;
	ctrl.vo_dis = 0;

	switch (stop_sel) {
	case TB_STOP_SEL_ALL:
		ctrl.be_dis = 1;
		ctrl.bk_dis = 1;
		ctrl.vi_dis = 1;
		ctrl.vo_dis = 1;
		ret = set_hw_tb_ppdu_ctrl(adapter, &ctrl);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case TB_STOP_SEL_BE:
		ctrl.be_dis = 1;
		ret = set_hw_tb_ppdu_ctrl(adapter, &ctrl);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case TB_STOP_SEL_BK:
		ctrl.bk_dis = 1;
		ret = set_hw_tb_ppdu_ctrl(adapter, &ctrl);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case TB_STOP_SEL_VI:
		ctrl.vi_dis = 1;
		ret = set_hw_tb_ppdu_ctrl(adapter, &ctrl);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case TB_STOP_SEL_VO:
		ctrl.vo_dis = 1;
		ret = set_hw_tb_ppdu_ctrl(adapter, &ctrl);
		if (ret != MACSUCCESS)
			return ret;
		break;
	}

	return MACSUCCESS;
}

u32 get_edca_addr(struct mac_ax_adapter *adapter,
		  struct mac_ax_edca_param *param, u32 *reg_edca)
{
	u8 band;
	u32 ret;
	enum mac_ax_cmac_path_sel path;

	band = param->band;
	path = param->path;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	switch (path) {
	case MAC_AX_CMAC_PATH_SEL_BE0:
		*reg_edca =
			band ? R_AX_EDCA_BE_PARAM_0_C1 : R_AX_EDCA_BE_PARAM_0;
		break;
	case MAC_AX_CMAC_PATH_SEL_BK0:
		*reg_edca =
			band ? R_AX_EDCA_BK_PARAM_0_C1 : R_AX_EDCA_BK_PARAM_0;
		break;
	case MAC_AX_CMAC_PATH_SEL_VI0:
		*reg_edca =
			band ? R_AX_EDCA_VI_PARAM_0_C1 : R_AX_EDCA_VI_PARAM_0;
		break;
	case MAC_AX_CMAC_PATH_SEL_VO0:
		*reg_edca =
			band ? R_AX_EDCA_VO_PARAM_0_C1 : R_AX_EDCA_VO_PARAM_0;
		break;
	case MAC_AX_CMAC_PATH_SEL_BE1:
		*reg_edca =
			band ? R_AX_EDCA_BE_PARAM_1_C1 : R_AX_EDCA_BE_PARAM_1;
		break;
	case MAC_AX_CMAC_PATH_SEL_BK1:
		*reg_edca =
			band ? R_AX_EDCA_BK_PARAM_1_C1 : R_AX_EDCA_BK_PARAM_1;
		break;
	case MAC_AX_CMAC_PATH_SEL_VI1:
		*reg_edca =
			band ? R_AX_EDCA_VI_PARAM_1_C1 : R_AX_EDCA_VI_PARAM_1;
		break;
	case MAC_AX_CMAC_PATH_SEL_VO1:
		*reg_edca =
			band ? R_AX_EDCA_VO_PARAM_1_C1 : R_AX_EDCA_VO_PARAM_1;
		break;
	case MAC_AX_CMAC_PATH_SEL_MG0_1:
		*reg_edca =
			band ? R_AX_EDCA_MGQ_PARAM_C1 : R_AX_EDCA_MGQ_PARAM;
		break;
	case MAC_AX_CMAC_PATH_SEL_MG2:
		*reg_edca =
			band ? (R_AX_EDCA_MGQ_PARAM_C1 + 2) :
			(R_AX_EDCA_MGQ_PARAM + 2);
		break;
	case MAC_AX_CMAC_PATH_SEL_BCN:
		*reg_edca =
			band ? (R_AX_EDCA_BCNQ_PARAM_C1 + 2) :
			(R_AX_EDCA_BCNQ_PARAM + 2);
		break;
	case MAC_AX_CMAC_PATH_SEL_TF:
		*reg_edca = R_AX_EDCA_ULQ_PARAM;
		break;
	case MAC_AX_CMAC_PATH_SEL_TWT0:
	case MAC_AX_CMAC_PATH_SEL_TWT2:
		*reg_edca = R_AX_EDCA_TWT_PARAM_0;
		break;
	case MAC_AX_CMAC_PATH_SEL_TWT1:
	case MAC_AX_CMAC_PATH_SEL_TWT3:
		*reg_edca = R_AX_EDCA_TWT_PARAM_1;
		break;
	default:
		return MACNOITEM;
	}

	return MACSUCCESS;
}

u32 get_muedca_param_addr(struct mac_ax_adapter *adapter,
			  struct mac_ax_muedca_param *param,
			  u32 *reg_edca)
{
	u8 band;
	u32 ret;
	enum mac_ax_cmac_ac_sel ac;

	band = param->band;
	ac = param->ac;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	switch (ac) {
	case MAC_AX_CMAC_AC_SEL_BE:
		*reg_edca =
			band ? R_AX_MUEDCA_BE_PARAM_0_C1 :
			R_AX_MUEDCA_BE_PARAM_0;
		break;
	case MAC_AX_CMAC_AC_SEL_BK:
		*reg_edca =
			band ? R_AX_MUEDCA_BK_PARAM_0_C1 :
			R_AX_MUEDCA_BK_PARAM_0;
		break;
	case MAC_AX_CMAC_AC_SEL_VI:
		*reg_edca =
			band ? R_AX_MUEDCA_VI_PARAM_0_C1 :
			R_AX_MUEDCA_VI_PARAM_0;
		break;
	case MAC_AX_CMAC_AC_SEL_VO:
		*reg_edca =
			band ? R_AX_MUEDCA_VO_PARAM_0_C1 :
			R_AX_MUEDCA_VO_PARAM_0;
		break;
	default:
		return MACNOITEM;
	}

	return MACSUCCESS;
}

void tx_on_dly(struct mac_ax_adapter *adapter, u8 band)
{
	u32 val32;
	u32 drop_dly_max;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	val32 = MAC_REG_R32(band ? R_AX_TX_CTRL_C1 : R_AX_TX_CTRL);
	drop_dly_max = GET_FIELD(val32, B_AX_DROP_CHK_MAX_NUM) >> 2;
	PLTFM_DELAY_US((drop_dly_max > TX_DLY_MAX) ? drop_dly_max : TX_DLY_MAX);
}

/* for sw mode Tx, need to stop sch */
/* (for "F2PCMD.disable_sleep_chk"), soar 20200225*/
static u32 stop_macid_ctn(struct mac_ax_adapter *adapter,
			  struct mac_role_tbl *role,
			  struct mac_ax_sch_tx_en_cfg *bak)
{
	struct mac_ax_sch_tx_en_cfg cfg;
	u32 ret;

	ret = check_mac_en(adapter, role->info.band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	ret = get_hw_sch_tx_en(adapter, bak);
	if (ret != MACSUCCESS)
		return ret;

	cfg.band = role->info.band;
	u16_2_sch(adapter, &cfg.tx_en_mask, 0);

	u16_2_sch(adapter, &cfg.tx_en, 0);
	u16_2_sch(adapter, &cfg.tx_en_mask, 0xFFFF);
	cfg.tx_en_mask.mg0 = 0;
	cfg.tx_en_mask.mg1 = 0;
	cfg.tx_en_mask.mg2 = 0;
	cfg.tx_en_mask.hi = 0;
	cfg.tx_en_mask.bcn = 0;

	ret = set_hw_sch_tx_en(adapter, &cfg);
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

static u32 tx_idle_sel_ck_b(struct mac_ax_adapter *adapter,
			    enum ptcl_tx_sel sel, u8 band)
{
	u32 cnt;
	u8 val8;
	u32 ret;
	u8 ptcl_tx_qid;
	u32 poll_addr;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	poll_addr = band ? R_AX_PTCL_TX_CTN_SEL_C1 : R_AX_PTCL_TX_CTN_SEL;

	val8 = MAC_REG_R8(poll_addr);
	if (val8 & B_AX_PTCL_TX_ON_STAT)
		tx_on_dly(adapter, band);
	else
		return MACSUCCESS;

	switch (sel) {
	case PTCL_TX_SEL_HIQ:
		ptcl_tx_qid = PTCL_TXQ_HIQ;
		break;
	case PTCL_TX_SEL_MG0:
		ptcl_tx_qid = PTCL_TXQ_MG0;
		break;
	default:
		return MACNOITEM;
	}

	cnt = PTCL_IDLE_POLL_CNT;
	while (--cnt) {
		val8 = MAC_REG_R8(poll_addr);
		if ((val8 & B_AX_PTCL_TX_ON_STAT) && (val8 & B_AX_PTCL_DROP))
			PLTFM_DELAY_US(SW_CVR_DUR_US);
		else if ((val8 & B_AX_PTCL_TX_ON_STAT) &&
			 (GET_FIELD(val8, B_AX_PTCL_TX_QUEUE_IDX) ==
			 ptcl_tx_qid))
			PLTFM_DELAY_US(SW_CVR_DUR_US);
		else
			break;
	}
	PLTFM_MSG_ALWAYS("%s: cnt %d, band %d, 0x%x\n", __func__, cnt, band, val8);
	if (!cnt)
		return MACPOLLTXIDLE;

	return MACSUCCESS;
}

static u32 band_idle_ck(struct mac_ax_adapter *adapter, u8 band)
{
	u32 cnt;
	u8 val8;
	u32 ret;
	u32 poll_addr;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	poll_addr = band ? R_AX_PTCL_TX_CTN_SEL_C1 : R_AX_PTCL_TX_CTN_SEL;

	cnt = PTCL_IDLE_POLL_CNT;
	while (--cnt) {
		val8 = MAC_REG_R8(poll_addr);
		if (val8 & B_AX_PTCL_TX_ON_STAT)
			PLTFM_DELAY_US(SW_CVR_DUR_US);
		else
			break;
	}
	PLTFM_MSG_ALWAYS("%s: cnt %d, band %d, 0x%x\n", __func__, cnt, band, val8);
	if (!cnt)
		return MACPOLLTXIDLE;

	return MACSUCCESS;
}

static void sch_2_u16(struct mac_ax_adapter *adapter,
		      struct mac_ax_sch_tx_en *tx_en, u16 *val16)
{
	*val16 = (tx_en->be0 ? B_AX_CTN_TXEN_BE_0 : 0) |
		(tx_en->bk0 ? B_AX_CTN_TXEN_BK_0 : 0) |
		(tx_en->vi0 ? B_AX_CTN_TXEN_VI_0 : 0) |
		(tx_en->vo0 ? B_AX_CTN_TXEN_VO_0 : 0) |
		(tx_en->be1 ? B_AX_CTN_TXEN_BE_1 : 0) |
		(tx_en->bk1 ? B_AX_CTN_TXEN_BK_1 : 0) |
		(tx_en->vi1 ? B_AX_CTN_TXEN_VI_1 : 0) |
		(tx_en->vo1 ? B_AX_CTN_TXEN_VO_1 : 0) |
		(tx_en->mg0 ? B_AX_CTN_TXEN_MGQ : 0) |
		(tx_en->mg1 ? B_AX_CTN_TXEN_MGQ1 : 0) |
		(tx_en->mg2 ? B_AX_CTN_TXEN_CPUMGQ : 0) |
		(tx_en->hi ? B_AX_CTN_TXEN_HGQ : 0) |
		(tx_en->bcn ? B_AX_CTN_TXEN_BCNQ : 0) |
		(tx_en->ul ? B_AX_CTN_TXEN_ULQ : 0) |
		(tx_en->twt0 ? B_AX_CTN_TXEN_TWT_0 : 0) |
		(tx_en->twt1 ? B_AX_CTN_TXEN_TWT_1 : 0);
}

static void sch_2_u32(struct mac_ax_adapter *adapter,
		      struct mac_ax_sch_tx_en *tx_en, u32 *val32)
{
	*val32 = (tx_en->be0 ? B_AX_CTN_TXEN_BE_0 : 0) |
		(tx_en->bk0 ? B_AX_CTN_TXEN_BK_0 : 0) |
		(tx_en->vi0 ? B_AX_CTN_TXEN_VI_0 : 0) |
		(tx_en->vo0 ? B_AX_CTN_TXEN_VO_0 : 0) |
		(tx_en->be1 ? B_AX_CTN_TXEN_BE_1 : 0) |
		(tx_en->bk1 ? B_AX_CTN_TXEN_BK_1 : 0) |
		(tx_en->vi1 ? B_AX_CTN_TXEN_VI_1 : 0) |
		(tx_en->vo1 ? B_AX_CTN_TXEN_VO_1 : 0) |
		(tx_en->mg0 ? B_AX_CTN_TXEN_MGQ : 0) |
		(tx_en->mg1 ? B_AX_CTN_TXEN_MGQ1 : 0) |
		(tx_en->mg2 ? B_AX_CTN_TXEN_CPUMGQ : 0) |
		(tx_en->hi ? B_AX_CTN_TXEN_HGQ : 0) |
		(tx_en->bcn ? B_AX_CTN_TXEN_BCNQ : 0) |
		(tx_en->ul ? B_AX_CTN_TXEN_ULQ : 0) |
		(tx_en->twt0 ? B_AX_CTN_TXEN_TWT_0 : 0) |
		(tx_en->twt1 ? B_AX_CTN_TXEN_TWT_1 : 0) |
		(tx_en->twt2 ? B_AX_CTN_TXEN_TWT_2 : 0) |
		(tx_en->twt3 ? B_AX_CTN_TXEN_TWT_3 : 0);
}

void u16_2_sch(struct mac_ax_adapter *adapter,
	       struct mac_ax_sch_tx_en *tx_en, u16 val16)
{
	tx_en->be0 = val16 & B_AX_CTN_TXEN_BE_0 ? 1 : 0;
	tx_en->bk0 = val16 & B_AX_CTN_TXEN_BK_0 ? 1 : 0;
	tx_en->vi0 = val16 & B_AX_CTN_TXEN_VI_0 ? 1 : 0;
	tx_en->vo0 = val16 & B_AX_CTN_TXEN_VO_0 ? 1 : 0;
	tx_en->be1 = val16 & B_AX_CTN_TXEN_BE_1 ? 1 : 0;
	tx_en->bk1 = val16 & B_AX_CTN_TXEN_BK_1 ? 1 : 0;
	tx_en->vi1 = val16 & B_AX_CTN_TXEN_VI_1 ? 1 : 0;
	tx_en->vo1 = val16 & B_AX_CTN_TXEN_VO_1 ? 1 : 0;
	tx_en->mg0 = val16 & B_AX_CTN_TXEN_MGQ ? 1 : 0;
	tx_en->mg1 = val16 & B_AX_CTN_TXEN_MGQ1 ? 1 : 0;
	tx_en->mg2 = val16 & B_AX_CTN_TXEN_CPUMGQ ? 1 : 0;
	tx_en->hi = val16 & B_AX_CTN_TXEN_HGQ ? 1 : 0;
	tx_en->bcn = val16 & B_AX_CTN_TXEN_BCNQ ? 1 : 0;
	tx_en->ul = val16 & B_AX_CTN_TXEN_ULQ ? 1 : 0;
	tx_en->twt0 = val16 & B_AX_CTN_TXEN_TWT_0 ? 1 : 0;
	tx_en->twt1 = val16 & B_AX_CTN_TXEN_TWT_1 ? 1 : 0;
}

void u32_2_sch(struct mac_ax_adapter *adapter,
	       struct mac_ax_sch_tx_en *tx_en, u32 val32)
{
	tx_en->be0 = val32 & B_AX_CTN_TXEN_BE_0 ? 1 : 0;
	tx_en->bk0 = val32 & B_AX_CTN_TXEN_BK_0 ? 1 : 0;
	tx_en->vi0 = val32 & B_AX_CTN_TXEN_VI_0 ? 1 : 0;
	tx_en->vo0 = val32 & B_AX_CTN_TXEN_VO_0 ? 1 : 0;
	tx_en->be1 = val32 & B_AX_CTN_TXEN_BE_1 ? 1 : 0;
	tx_en->bk1 = val32 & B_AX_CTN_TXEN_BK_1 ? 1 : 0;
	tx_en->vi1 = val32 & B_AX_CTN_TXEN_VI_1 ? 1 : 0;
	tx_en->vo1 = val32 & B_AX_CTN_TXEN_VO_1 ? 1 : 0;
	tx_en->mg0 = val32 & B_AX_CTN_TXEN_MGQ ? 1 : 0;
	tx_en->mg1 = val32 & B_AX_CTN_TXEN_MGQ1 ? 1 : 0;
	tx_en->mg2 = val32 & B_AX_CTN_TXEN_CPUMGQ ? 1 : 0;
	tx_en->hi = val32 & B_AX_CTN_TXEN_HGQ ? 1 : 0;
	tx_en->bcn = val32 & B_AX_CTN_TXEN_BCNQ ? 1 : 0;
	tx_en->ul = val32 & B_AX_CTN_TXEN_ULQ ? 1 : 0;
	tx_en->twt0 = val32 & B_AX_CTN_TXEN_TWT_0 ? 1 : 0;
	tx_en->twt1 = val32 & B_AX_CTN_TXEN_TWT_1 ? 1 : 0;
	tx_en->twt2 = val32 & B_AX_CTN_TXEN_TWT_2 ? 1 : 0;
	tx_en->twt3 = val32 & B_AX_CTN_TXEN_TWT_3 ? 1 : 0;
}

static u32 h2c_usr_edca(struct mac_ax_adapter *adapter,
			struct mac_ax_usr_edca_param *param)
{
	struct fwcmd_usr_edca *fwcmd_tbl;
	struct h2c_info h2c_info = {0};
	u32 ret;

	if (adapter->sm.fwdl != MAC_AX_FWDL_INIT_RDY) {
		PLTFM_MSG_WARN("%s fw not ready\n", __func__);
		return MACFWNONRDY;
	}

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_usr_edca);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_FW_OFLD;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_USR_EDCA;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	fwcmd_tbl = (struct fwcmd_usr_edca *)PLTFM_MALLOC(h2c_info.content_len);
	if (!fwcmd_tbl)
		return MACBUFALLOC;

	fwcmd_tbl->dword0 =
	cpu_to_le32(SET_WORD(param->idx, FWCMD_H2C_USR_EDCA_PARAM_SEL) |
		    (param->enable ? FWCMD_H2C_USR_EDCA_ENABLE : 0) |
		    (param->band ? FWCMD_H2C_USR_EDCA_BAND : 0) |
		    (param->wmm ? FWCMD_H2C_USR_EDCA_WMM : 0) |
		    SET_WORD(param->ac, FWCMD_H2C_USR_EDCA_AC));
	fwcmd_tbl->dword1 =
	cpu_to_le32(SET_WORD(param->aggressive.txop_32us, TXOPLMT) |
		    SET_WORD((param->aggressive.ecw_max << 4) |
			     param->aggressive.ecw_min, CW) |
		    SET_WORD(param->aggressive.aifs_us, AIFS));
	fwcmd_tbl->dword2 =
	cpu_to_le32(SET_WORD(param->moderate.txop_32us, TXOPLMT) |
		    SET_WORD((param->moderate.ecw_max << 4) |
			     param->moderate.ecw_min, CW) |
		    SET_WORD(param->moderate.aifs_us, AIFS));

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)fwcmd_tbl);

	PLTFM_FREE(fwcmd_tbl, h2c_info.content_len);

	return ret;
}

static u32 h2c_usr_tx_rpt(struct mac_ax_adapter *adapter,
			  struct mac_ax_usr_tx_rpt_cfg *param)
{
	struct fwcmd_usr_tx_rpt *fwcmd_tbl;
	struct h2c_info h2c_info = {0};
	u32 ret;

	if (adapter->sm.fwdl != MAC_AX_FWDL_INIT_RDY) {
		PLTFM_MSG_WARN("%s fw not ready\n", __func__);
		return MACFWNONRDY;
	}

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_usr_tx_rpt);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_FW_OFLD;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_USR_TX_RPT;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	fwcmd_tbl = (struct fwcmd_usr_tx_rpt *)PLTFM_MALLOC(h2c_info.content_len);
	if (!fwcmd_tbl)
		return MACBUFALLOC;

	fwcmd_tbl->dword0 =
	cpu_to_le32(SET_WORD(param->mode, FWCMD_H2C_USR_TX_RPT_MODE) |
		    (param->rpt_start ? FWCMD_H2C_USR_TX_RPT_RTP_START : 0));
	fwcmd_tbl->dword1 =
	cpu_to_le32(SET_WORD(param->macid, FWCMD_H2C_USR_TX_RPT_MACID) |
		    (param->band ? FWCMD_H2C_USR_TX_RPT_BAND : 0) |
		    SET_WORD(param->port, FWCMD_H2C_USR_TX_RPT_PORT));
	fwcmd_tbl->dword2 = cpu_to_le32(param->rpt_period_us);

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)fwcmd_tbl);

	PLTFM_FREE(fwcmd_tbl, h2c_info.content_len);

	return ret;
}

u32 mac_set_cctl_max_tx_time(struct mac_ax_adapter *adapter,
			     struct mac_ax_max_tx_time *tx_time)
{
	struct mac_ax_ops *mops = adapter_to_mac_ops(adapter);
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	struct rtw_hal_mac_ax_cctl_info info, msk = {0};
	u32 ret = MACSUCCESS;
	struct mac_role_tbl *role;
	u8 band, txtime_limit, tx_time_reg;
	u32 offset, max_tx_time, val32;

	role = mac_role_srch(adapter, tx_time->macid);
	if (!role) {
		PLTFM_MSG_ERR("%s: The MACID%d does not exist\n",
			      __func__, tx_time->macid);
		return MACNOITEM;
	}

	txtime_limit = adapter->hw_info->max_agg_txtime_reg;

	max_tx_time = tx_time->max_tx_time == 0 ?
		(txtime_limit << 5) : tx_time->max_tx_time;

	if (tx_time->is_cctrl) {
		msk.ampdu_time_sel = 1;
		info.ampdu_time_sel = 1;
		msk.ampdu_max_time = FWCMD_H2C_CCTRL_AMPDU_MAX_TIME_MSK;
		info.ampdu_max_time = (max_tx_time - 512) >> 9;
		if ((info.ampdu_max_time << 4) >= txtime_limit) {
			PLTFM_MSG_ERR("%s: txtime_limit is 0x%x, is_cctrl %d\n",
				      __func__, txtime_limit, tx_time->is_cctrl);
			info.ampdu_max_time = txtime_limit >> 4;
		}
		ret = mops->upd_cctl_info(adapter, &info, &msk, tx_time->macid, 1);
	} else {
		band = role->info.band;
		offset = band == 0 ? R_AX_AMPDU_AGG_LIMIT : R_AX_AMPDU_AGG_LIMIT_C1;
		ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
		if (ret != MACSUCCESS)
			return ret;

		val32 = MAC_REG_R32(offset);
		if ((max_tx_time >> 5) >= txtime_limit) {
			PLTFM_MSG_ERR("%s: txtime_limit is 0x%x, is_cctrl %d\n",
				      __func__, txtime_limit, tx_time->is_cctrl);
			tx_time_reg = txtime_limit;
		} else {
			tx_time_reg = max_tx_time >> 5;
		}
		val32 = SET_CLR_WORD(val32, tx_time_reg, B_AX_AMPDU_MAX_TIME);
		MAC_REG_W32(offset, val32);
	}

	return ret;
}

u32 mac_get_max_tx_time(struct mac_ax_adapter *adapter,
			struct mac_ax_max_tx_time *tx_time)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ret = MACSUCCESS;
	struct mac_role_tbl *role;
	u32 offset;
	u8 band;

	role = mac_role_srch(adapter, tx_time->macid);
	if (!role) {
		PLTFM_MSG_ERR("%s: The MACID%d does not exist\n",
			      __func__, tx_time->macid);
		return MACNOITEM;
	}

	if (role->info.c_info.ampdu_time_sel) {
		tx_time->max_tx_time = (role->info.c_info.ampdu_max_time + 1) << 9;
		tx_time->is_cctrl = 1;
	} else {
		band = role->info.band;
		offset = band == 0 ? R_AX_AMPDU_AGG_LIMIT + 3 :
			R_AX_AMPDU_AGG_LIMIT_C1 + 3;
		ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
		if (ret == MACSUCCESS)
			tx_time->max_tx_time = MAC_REG_R8(offset) << 5;
		tx_time->is_cctrl = 0;
	}

	return ret;
}

u32 mac_set_hw_rts_th(struct mac_ax_adapter *adapter,
		      struct mac_ax_hw_rts_th *th)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ret, offset;
	u16 val;

	ret = check_mac_en(adapter, th->band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;
	offset = th->band ? R_AX_AGG_LEN_HT_0_C1 : R_AX_AGG_LEN_HT_0;

#if MAC_AX_FW_REG_OFLD
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = MAC_REG_W_OFLD((u16)offset,
				     B_AX_RTS_LEN_TH_MSK << B_AX_RTS_LEN_TH_SH,
				     th->len_th >> MAC_AX_MULT16_SH, 0);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("%s: config fail\n", __func__);
			return ret;
		}
		ret = MAC_REG_W_OFLD((u16)offset,
				     B_AX_RTS_TXTIME_TH_MSK <<
				     B_AX_RTS_TXTIME_TH_SH,
				     th->time_th >> MAC_AX_MULT32_SH, 1);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("%s: config fail\n", __func__);
			return ret;
		}
		return MACSUCCESS;
	}
#endif
	val = SET_WORD(th->len_th >> MAC_AX_MULT16_SH, B_AX_RTS_LEN_TH) |
		SET_WORD(th->time_th >> MAC_AX_MULT32_SH, B_AX_RTS_TXTIME_TH);
	MAC_REG_W16(offset, val);

	return MACSUCCESS;
}

u32 mac_get_hw_rts_th(struct mac_ax_adapter *adapter,
		      struct mac_ax_hw_rts_th *th)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ret, offset;
	u16 val;

	ret = check_mac_en(adapter, th->band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	offset = th->band ? R_AX_AGG_LEN_HT_0_C1 : R_AX_AGG_LEN_HT_0;
	val = MAC_REG_R16(offset);

	th->len_th = GET_FIELD(val, B_AX_RTS_LEN_TH);
	th->len_th = th->len_th << MAC_AX_MULT16_SH;
	th->time_th = GET_FIELD(val, B_AX_RTS_TXTIME_TH);
	th->time_th = th->time_th << MAC_AX_MULT32_SH;

	return MACSUCCESS;
}

static u32 h2c_usr_frame_to_act(struct mac_ax_adapter *adapter,
				struct mac_ax_usr_frame_to_act_cfg *param)
{
	struct fwcmd_frame_to_act *fwcmd_tbl;
	struct h2c_info h2c_info = {0};
	u32 ret;

	if (adapter->sm.fwdl != MAC_AX_FWDL_INIT_RDY) {
		PLTFM_MSG_WARN("%s fw not ready\n", __func__);
		return MACFWNONRDY;
	}

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_frame_to_act);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_FW_OFLD;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_FRAME_TO_ACT;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	fwcmd_tbl = (struct fwcmd_frame_to_act *)PLTFM_MALLOC(h2c_info.content_len);
	if (!fwcmd_tbl)
		return MACBUFALLOC;

	fwcmd_tbl->dword0 =
	cpu_to_le32(SET_WORD(param->mode, FWCMD_H2C_FRAME_TO_ACT_MODE) |
		    SET_WORD(param->trigger_cnt, FWCMD_H2C_FRAME_TO_ACT_TRIGGER_CNT) |
		    SET_WORD(param->sw_def_bmp, FWCMD_H2C_FRAME_TO_ACT_SW_DEF));
	fwcmd_tbl->dword1 =
	cpu_to_le32(SET_WORD(param->to_thr, FWCMD_H2C_FRAME_TO_ACT_TO_THR));
	fwcmd_tbl->dword2 = 0;

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)fwcmd_tbl);

	PLTFM_FREE(fwcmd_tbl, h2c_info.content_len);

	return ret;
}

u32 mac_tx_idle_poll(struct mac_ax_adapter *adapter,
		     struct mac_ax_tx_idle_poll_cfg *poll_cfg)
{
	switch (poll_cfg->sel) {
	case MAC_AX_TX_IDLE_POLL_SEL_BAND:
		return tx_idle_poll_band(adapter, poll_cfg->band);
	default:
		return MACNOITEM;
	}
}

u32 mac_set_tx_ru26_tb(struct mac_ax_adapter *adapter,
		       u8 disable)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val32;

#if MAC_AX_FW_REG_OFLD
	u32 ret;

	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		if (disable)
			ret = MAC_REG_W_OFLD(R_AX_RXTRIG_TEST_USER_2, B_AX_RXTRIG_RU26_DIS, 1, 1);
		else
			ret = MAC_REG_W_OFLD(R_AX_RXTRIG_TEST_USER_2, B_AX_RXTRIG_RU26_DIS, 0, 1);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n", __func__, R_AX_RXTRIG_TEST_USER_2);
			return ret;
		}
		return MACSUCCESS;
	}
#endif
	val32 = MAC_REG_R32(R_AX_RXTRIG_TEST_USER_2) & (~B_AX_RXTRIG_RU26_DIS);

	if (disable)
		MAC_REG_W32(R_AX_RXTRIG_TEST_USER_2, val32 | B_AX_RXTRIG_RU26_DIS);
	else
		MAC_REG_W32(R_AX_RXTRIG_TEST_USER_2, val32);

	return MACSUCCESS;
}

u32 set_cts2self(struct mac_ax_adapter *adapter,
		 struct mac_ax_cts2self_cfg *cfg)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 reg;
	u32 val32;

	if (!cfg) {
		PLTFM_MSG_ERR("[ERR]: the parameter is NULL in %s\n", __func__);
		return MACNPTR;
	}

	reg = cfg->band_sel == MAC_AX_BAND_1 ?
	      R_AX_SIFS_SETTING_C1 : R_AX_SIFS_SETTING;
	val32 = MAC_REG_R32(reg);

	if (cfg->threshold_sel == MAC_AX_CTS2SELF_DISABLE) {
		val32 &= ~B_AX_HW_CTS2SELF_EN;
		MAC_REG_W32(reg, val32);
		return MACSUCCESS;
	} else {
		val32 |= B_AX_HW_CTS2SELF_EN;
	}
	if (cfg->threshold_sel == MAC_AX_CTS2SELF_NON_SEC_THRESHOLD ||
	    cfg->threshold_sel == MAC_AX_CTS2SELF_BOTH_THRESHOLD) {
		if (cfg->non_sec_threshold & ~B_AX_HW_CTS2SELF_PKT_LEN_TH_MSK) {
			PLTFM_MSG_ERR("[ERR]: value out of range in %s\n", __func__);
			return MACHWNOSUP;
		}
		val32 = SET_CLR_WORD(val32, cfg->non_sec_threshold,
				     B_AX_HW_CTS2SELF_PKT_LEN_TH);
	}
	if (cfg->threshold_sel == MAC_AX_CTS2SELF_SEC_THRESHOLD ||
	    cfg->threshold_sel == MAC_AX_CTS2SELF_BOTH_THRESHOLD) {
		if (cfg->sec_threshold & ~B_AX_HW_CTS2SELF_PKT_LEN_TH_TWW_MSK) {
			PLTFM_MSG_ERR("[ERR]: value out of range in %s\n", __func__);
			return MACHWNOSUP;
		}
		val32 = SET_CLR_WORD(val32, cfg->sec_threshold,
				     B_AX_HW_CTS2SELF_PKT_LEN_TH_TWW);
	}

	MAC_REG_W32(reg, val32);
	return MACSUCCESS;
}

u32 mac_tx_duty(struct mac_ax_adapter *adapter,
		u16 pause_intvl, u16 tx_intvl)
{
	u32 ret;

	if (!(pause_intvl) || !(tx_intvl))
		return MACFUNCINPUT;

	ret = tx_duty_h2c(adapter, pause_intvl, tx_intvl);
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

u32 mac_tx_duty_stop(struct mac_ax_adapter *adapter)
{
	u32 ret;

	ret = tx_duty_h2c(adapter, 0, 0);
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

u32 tx_duty_h2c(struct mac_ax_adapter *adapter,
		u16 pause_intvl, u16 tx_intvl)
{
	u32 ret;
	struct fwcmd_tx_duty cfg;
	struct h2c_info h2c_info = {0};

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_tx_duty);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_FW_OFLD;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_TX_DUTY;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	cfg.dword0 =
	cpu_to_le32(SET_WORD(pause_intvl, FWCMD_H2C_TX_DUTY_PAUSE_INTVL) |
		    SET_WORD(tx_intvl, FWCMD_H2C_TX_DUTY_TX_INTVL)
	);
	cfg.dword1 = cpu_to_le32(pause_intvl ? 0 : FWCMD_H2C_TX_DUTY_STOP);

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)&cfg);

	return ret;
}

u32 set_cctl_rty_limit(struct mac_ax_adapter *adapter,
		       struct mac_ax_cctl_rty_lmt_cfg *cfg)
{
	struct mac_ax_ops *mops = adapter_to_mac_ops(adapter);
	struct rtw_hal_mac_ax_cctl_info info;
	struct rtw_hal_mac_ax_cctl_info mask;

	u32 data_rty, rts_rty;

	PLTFM_MEMSET(&mask, 0, sizeof(struct rtw_hal_mac_ax_cctl_info));
	PLTFM_MEMSET(&info, 0, sizeof(struct rtw_hal_mac_ax_cctl_info));

	data_rty = cfg->data_lmt_val == 0 ?
		DFLT_DATA_RTY_LIMIT : cfg->data_lmt_val;
	rts_rty = cfg->rts_lmt_val == 0 ?
		DFLT_RTS_RTY_LIMIT : cfg->rts_lmt_val;
	info.data_txcnt_lmt_sel = cfg->data_lmt_sel;
	info.data_tx_cnt_lmt = data_rty;
	info.rts_txcnt_lmt_sel = cfg->rts_lmt_sel;
	info.rts_txcnt_lmt = rts_rty;

	mask.data_txcnt_lmt_sel = TXCNT_LMT_MSK;
	mask.data_tx_cnt_lmt = FWCMD_H2C_CCTRL_DATA_TX_CNT_LMT_MSK;
	mask.rts_txcnt_lmt_sel = TXCNT_LMT_MSK;
	mask.rts_txcnt_lmt = FWCMD_H2C_CCTRL_RTS_TXCNT_LMT_MSK;

	mops->upd_cctl_info(adapter, &info, &mask, cfg->macid, TBL_WRITE_OP);

	return MACSUCCESS;
}

u32 set_data_rty_limit(struct mac_ax_adapter *adapter, struct mac_ax_rty_lmt *rty)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ret = MACSUCCESS;
	struct mac_role_tbl *role;
	u32 offset_l, offset_s;
	u8 band = HW_BAND_0;

	if (rty->macid < adapter->hw_info->macid_num) {
		role = mac_role_srch(adapter, rty->macid);
		if (!role) {
			PLTFM_MSG_ERR("%s: The MACID%d does not exist\n",
				      __func__, rty->macid);
			return MACNOITEM;
		}

		if (role->info.c_info.data_txcnt_lmt_sel) {
			PLTFM_MSG_ERR("%s: MACID%d follow CMAC_TBL setting\n",
				      __func__, rty->macid);
		}

		band = role->info.band;
		offset_l = band == 0 ? R_AX_TXCNT + 2 : R_AX_TXCNT_C1 + 2;
		offset_s = band == 0 ? R_AX_TXCNT + 3 : R_AX_TXCNT_C1 + 3;
		ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
		if (ret == MACSUCCESS) {
			MAC_REG_W8(offset_l, rty->tx_cnt);
			MAC_REG_W8(offset_s, rty->short_tx_cnt);
		}
	} else {
		ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
		if (ret == MACSUCCESS) {
			MAC_REG_W8(R_AX_TXCNT + 2, rty->tx_cnt);
			MAC_REG_W8(R_AX_TXCNT + 3, rty->short_tx_cnt);
		}
	}

	return ret;
}

u32 get_data_rty_limit(struct mac_ax_adapter *adapter, struct mac_ax_rty_lmt *rty)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ret = MACSUCCESS;
	struct mac_role_tbl *role;
	u32 offset_l, offset_s;
	u8 band;

	role = mac_role_srch(adapter, rty->macid);
	if (!role) {
		PLTFM_MSG_ERR("%s: The MACID%d does not exist\n",
			      __func__, rty->macid);
		return MACNOITEM;
	}

	if (role->info.c_info.data_txcnt_lmt_sel) {
		rty->tx_cnt = (u8)role->info.c_info.data_tx_cnt_lmt;
	} else {
		band = role->info.band;
		offset_l = band == 0 ? R_AX_TXCNT + 2 : R_AX_TXCNT_C1 + 2;
		offset_s = band == 0 ? R_AX_TXCNT + 3 : R_AX_TXCNT_C1 + 3;
		ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
		if (ret == MACSUCCESS) {
			rty->tx_cnt = MAC_REG_R8(offset_l);
			rty->short_tx_cnt = MAC_REG_R8(offset_s);
		}
	}

	return ret;
}

u32 scheduler_set_prebkf(struct mac_ax_adapter *adapter,
			 struct mac_ax_prebkf_setting *para)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 reg, val32;
#if MAC_AX_FW_REG_OFLD
	u32 ret;
#endif

	reg = para->band == MAC_AX_BAND_1 ? R_AX_PREBKF_CFG_0_C1 :
			    R_AX_PREBKF_CFG_0;
#if MAC_AX_FW_REG_OFLD
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = MAC_REG_W_OFLD((u16)reg, B_AX_PREBKF_TIME_MSK, para->val,
				     1);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("%s: config fail\n", __func__);
			return ret;
		}

		return MACSUCCESS;
	}
#endif
	val32 = MAC_REG_R32(reg);
	val32 = SET_CLR_WORD(val32, para->val, B_AX_PREBKF_TIME);
	MAC_REG_W32(reg, val32);

	return MACSUCCESS;
}

u32 get_block_tx_sel_msk(enum mac_ax_block_tx_sel src, u32 *msk)
{
	switch (src) {
	case MAC_AX_CCA:
		*msk = B_AX_CCA_EN;
		break;
	case MAC_AX_SEC20_CCA:
		*msk = B_AX_SEC20_EN;
		break;
	case MAC_AX_SEC40_CCA:
		*msk = B_AX_SEC40_EN;
		break;
	case MAC_AX_SEC80_CCA:
		*msk = B_AX_SEC80_EN;
		break;
	case MAC_AX_EDCCA:
		*msk = B_AX_EDCCA_EN;
		break;
	case MAC_AX_BTCCA:
		*msk = B_AX_BTCCA_EN;
		break;
	case MAC_AX_TX_NAV:
		*msk = B_AX_TX_NAV_EN;
		break;
	default:
		return MACNOITEM;
	}

	return MACSUCCESS;
}

u32 cfg_block_tx(struct mac_ax_adapter *adapter,
		 enum mac_ax_block_tx_sel src, u8 band, u8 en)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val, msk, ret;
	u32 reg = band == 0 ? R_AX_CCA_CFG_0 : R_AX_CCA_CFG_0_C1;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret) {
		PLTFM_MSG_ERR("%s: CMAC%d is NOT enabled\n", __func__, band);
		return ret;
	}

	ret = get_block_tx_sel_msk(src, &msk);
	if (ret) {
		PLTFM_MSG_ERR("%s: %d is NOT supported\n", __func__, src);
		return ret;
	}

#if MAC_AX_FW_REG_OFLD
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = MAC_REG_W_OFLD((u16)reg, msk,
				     en ? 1 : 0, 1);
		if (ret != MACSUCCESS)
			PLTFM_MSG_ERR("%s: write offload fail %d",
				      __func__, ret);

		return ret;
	}
#endif
	val = MAC_REG_R32(reg);

	if (en)
		val = val | msk;
	else
		val = val & ~msk;
	MAC_REG_W32(reg, val);

	return MACSUCCESS;
}

u32 get_block_tx(struct mac_ax_adapter *adapter,
		 enum mac_ax_block_tx_sel src, u8 band, u8 *en)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val, msk, ret;
	u32 reg = band == 0 ? R_AX_CCA_CFG_0 : R_AX_CCA_CFG_0_C1;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret) {
		PLTFM_MSG_ERR("%s: CMAC%d is NOT enabled", __func__, band);
		return ret;
	}

	val = MAC_REG_R32(reg);

	ret = get_block_tx_sel_msk(src, &msk);
	if (ret) {
		PLTFM_MSG_ERR("%s: %d is NOT supported\n", __func__, src);
		return ret;
	}

	*en = !!(val & msk);

	return MACSUCCESS;
}

u32 set_macid_pause(struct mac_ax_adapter *adapter,
		    struct mac_ax_macid_pause_cfg *cfg)
{
	struct mac_ax_macid_pause_sleep_cfg pause_sleep_cfg = {0};

	pause_sleep_cfg.macid = cfg->macid;
	pause_sleep_cfg.pause = cfg->pause;
	pause_sleep_cfg.sleep = cfg->pause;
	return set_macid_pause_sleep(adapter, &pause_sleep_cfg);
}

u32 macid_pause(struct mac_ax_adapter *adapter,
		struct mac_ax_macid_pause_grp *grp)
{
	u8 index;
	struct mac_ax_macid_pause_sleep_grp pause_sleep_grp = {{0}};

	for (index = 0; index < 4; index++) {
		pause_sleep_grp.pause_grp[index] = grp->pause_grp[index];
		pause_sleep_grp.pause_grp_mask[index] = grp->mask_grp[index];
		pause_sleep_grp.sleep_grp[index] = grp->pause_grp[index];
		pause_sleep_grp.sleep_grp_mask[index] = grp->mask_grp[index];
	}
	return macid_pause_sleep(adapter, &pause_sleep_grp);
}

u32 macid_pause_sleep(struct mac_ax_adapter *adapter,
		      struct mac_ax_macid_pause_sleep_grp *grp)
{
	u32 ret;
	struct fwcmd_macid_pause_sleep *para;
	struct h2c_info h2c_info = {0};

	ret = check_mac_en(adapter, MAC_AX_BAND_0, MAC_AX_CMAC_SEL);
	if (ret)
		return ret;

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_macid_pause_sleep);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_FW_OFLD;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_MACID_PAUSE_SLEEP;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	para = (struct fwcmd_macid_pause_sleep *)PLTFM_MALLOC(h2c_info.content_len);
	if (!para)
		return MACBUFALLOC;

	PLTFM_MEMSET(para, 0, sizeof(struct fwcmd_macid_pause_sleep));

	para->dword0 =
	cpu_to_le32(SET_WORD(grp->pause_grp[0],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_PAUSE_GRP_EN_1));
	para->dword1 =
	cpu_to_le32(SET_WORD(grp->pause_grp[1],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_PAUSE_GRP_EN_2));
	para->dword2 =
	cpu_to_le32(SET_WORD(grp->pause_grp[2],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_PAUSE_GRP_EN_3));
	para->dword3 =
	cpu_to_le32(SET_WORD(grp->pause_grp[3],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_PAUSE_GRP_EN_4));
	para->dword4 =
	cpu_to_le32(SET_WORD(grp->pause_grp_mask[0],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_PAUSE_GRP_MASK_1));
	para->dword5 =
	cpu_to_le32(SET_WORD(grp->pause_grp_mask[1],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_PAUSE_GRP_MASK_2));
	para->dword6 =
	cpu_to_le32(SET_WORD(grp->pause_grp_mask[2],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_PAUSE_GRP_MASK_3));
	para->dword7 =
	cpu_to_le32(SET_WORD(grp->pause_grp_mask[3],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_PAUSE_GRP_MASK_4));

	para->dword8 =
	cpu_to_le32(SET_WORD(grp->sleep_grp[0],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_SLEEP_GRP_EN_1));
	para->dword9 =
	cpu_to_le32(SET_WORD(grp->sleep_grp[1],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_SLEEP_GRP_EN_2));
	para->dword10 =
	cpu_to_le32(SET_WORD(grp->sleep_grp[2],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_SLEEP_GRP_EN_3));
	para->dword11 =
	cpu_to_le32(SET_WORD(grp->sleep_grp[3],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_SLEEP_GRP_EN_4));
	para->dword12 =
	cpu_to_le32(SET_WORD(grp->sleep_grp_mask[0],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_SLEEP_GRP_MASK_1));
	para->dword13 =
	cpu_to_le32(SET_WORD(grp->sleep_grp_mask[1],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_SLEEP_GRP_MASK_2));
	para->dword14 =
	cpu_to_le32(SET_WORD(grp->sleep_grp_mask[2],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_SLEEP_GRP_MASK_3));
	para->dword15 =
	cpu_to_le32(SET_WORD(grp->sleep_grp_mask[3],
			     FWCMD_H2C_MACID_PAUSE_SLEEP_SLEEP_GRP_MASK_4));

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)para);
	PLTFM_FREE(para, h2c_info.content_len);
	return ret;
}

u32 enable_macid_pause_sleep(struct mac_ax_adapter *adapter,
			     u8 macid, u8 enable, u32 *grp_reg)
{
	u32 val32;
	u8 macid_sh, macid_grp;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	macid_sh = macid & MACID_PAUSE_MSK;
	macid_grp = macid >> MACID_PAUSE_SH;
	if (macid_grp >= 4) {
		PLTFM_MSG_ERR("%s: unexpected macid %x\n",
			      __func__, macid);
		return MACNOTSUP;
	}
	val32 = MAC_REG_R32(grp_reg[macid_grp]);
	if (enable)
		MAC_REG_W32(grp_reg[macid_grp], val32 | BIT(macid_sh));
	else
		MAC_REG_W32(grp_reg[macid_grp], val32 & ~(BIT(macid_sh)));

	return MACSUCCESS;
}

u32 set_macid_pause_sleep(struct mac_ax_adapter *adapter,
			  struct mac_ax_macid_pause_sleep_cfg *cfg)
{
	u8 macid_sh;
	u8 macid_grp;
	u32 ret;
	struct mac_ax_macid_pause_sleep_grp grp = {{0}};
	u32 pause_reg[4] = {
		R_AX_SS_MACID_PAUSE_0, R_AX_SS_MACID_PAUSE_1,
		R_AX_SS_MACID_PAUSE_2, R_AX_SS_MACID_PAUSE_3
	};
	u32 sleep_reg[4] = {
		R_AX_MACID_SLEEP_0, R_AX_MACID_SLEEP_1,
		R_AX_MACID_SLEEP_2, R_AX_MACID_SLEEP_3
	};

	ret = check_mac_en(adapter, MAC_AX_BAND_0, MAC_AX_CMAC_SEL);
	if (ret)
		return ret;

	if (adapter->sm.fwdl != MAC_AX_FWDL_INIT_RDY) {
		ret = enable_macid_pause_sleep(adapter, cfg->macid, cfg->pause, pause_reg);
		if (ret) {
			PLTFM_MSG_ERR("%s: setting pause failed: %d\n",
				      __func__, ret);
			return ret;
		}
		ret = enable_macid_pause_sleep(adapter, cfg->macid, cfg->sleep, sleep_reg);
		if (ret) {
			PLTFM_MSG_ERR("%s: setting sleep failed: %d\n",
				      __func__, ret);
			return ret;
		}
	} else {
		macid_sh = cfg->macid & MACID_PAUSE_MSK;
		macid_grp = cfg->macid >> MACID_PAUSE_SH;

		grp.pause_grp_mask[macid_grp] = BIT(macid_sh);
		grp.pause_grp[macid_grp] = (cfg->pause ? 1 : 0) << macid_sh;

		grp.sleep_grp_mask[macid_grp] = BIT(macid_sh);
		grp.sleep_grp[macid_grp] = (cfg->sleep ? 1 : 0) << macid_sh;

		ret = macid_pause_sleep(adapter, &grp);

		if (ret)
			return ret;
	}
	return MACSUCCESS;
}

u32 get_macid_pause(struct mac_ax_adapter *adapter,
		    struct mac_ax_macid_pause_cfg *cfg)
{
	u32 val32 = 0;
	u8 macid_grp = cfg->macid >> 5;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ret;

	ret = check_mac_en(adapter, MAC_AX_BAND_0, MAC_AX_CMAC_SEL);
	if (ret)
		return ret;

	switch (macid_grp) {
	case 0:
		val32 = MAC_REG_R32(R_AX_SS_MACID_PAUSE_0) |
			MAC_REG_R32(R_AX_MACID_SLEEP_0);
		break;
	case 1:
		val32 = MAC_REG_R32(R_AX_SS_MACID_PAUSE_1) |
			MAC_REG_R32(R_AX_MACID_SLEEP_1);
		break;
	case 2:
		val32 = MAC_REG_R32(R_AX_SS_MACID_PAUSE_2) |
			MAC_REG_R32(R_AX_MACID_SLEEP_2);
		break;
	case 3:
		val32 = MAC_REG_R32(R_AX_SS_MACID_PAUSE_3) |
			MAC_REG_R32(R_AX_MACID_SLEEP_3);
		break;
	default:
		break;
	}
	cfg->pause = (u8)((val32 & BIT(cfg->macid & (32 - 1))) ? 1 : 0);

	return MACSUCCESS;
}

