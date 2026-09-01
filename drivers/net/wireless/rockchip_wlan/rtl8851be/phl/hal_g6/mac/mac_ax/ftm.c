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
#include "ftm.h"
#if MAC_FEAT_FTM
#define FWCMD_H2C_FTM_INFO_ASAP_SH 0
#define FWCMD_H2C_FTM_INFO_ASAP_MSK 0xff
#define FWCMD_H2C_FTM_INFO_PKTID_IFTMR_SH 8
#define FWCMD_H2C_FTM_INFO_PKTID_IFTMR_MSK 0xff
#define FWCMD_H2C_FTM_INFO_PKTID_IFTM_SH 16
#define FWCMD_H2C_FTM_INFO_PKTID_IFTM_MSK 0xff
#define FWCMD_H2C_FTM_INFO_TSF_TIMER_OFFSET_SH 24
#define FWCMD_H2C_FTM_INFO_TSF_TIMER_OFFSET_MSK 0xff
#define FWCMD_H2C_FTM_INFO_PARTIAL_TSF_TIMER_SH 0
#define FWCMD_H2C_FTM_INFO_PARTIAL_TSF_TIMER_MSK 0xffff
#define FWCMD_H2C_FTM_INFO_FIRST_PSTIMER_HIGH_SH 0
#define FWCMD_H2C_FTM_INFO_FIRST_PSTIMER_HIGH_MSK 0xffffffff
#define FWCMD_H2C_FTM_INFO_FIRST_PSTIMER_LOW_SH 0
#define FWCMD_H2C_FTM_INFO_FIRST_PSTIMER_LOW_MSK 0xffffffff
#define FWCMD_H2C_FTM_INFO_ITVL_SH 0
#define FWCMD_H2C_FTM_INFO_ITVL_MSK 0xffffffff
#define FWCMD_H2C_FTM_INFO_BURST_CNT_SH 0
#define FWCMD_H2C_FTM_INFO_BURST_CNT_MSK 0xff
#define FWCMD_H2C_FTM_INFO_RSP_CH_SH 16
#define FWCMD_H2C_FTM_INFO_RSP_CH_MSK 0xff
#define FWCMD_H2C_FTM_INFO_MODE_SH 24
#define FWCMD_H2C_FTM_INFO_MODE_MSK 0xff

u32 mac_update_ftm_upd_info(struct mac_ax_adapter *adapter,
			    struct fwcmd_ftm_upd_info *info)
{
	u32 ret = MACSUCCESS;
	struct h2c_info h2c_info = {0};
	struct fwcmd_ftm_upd_info *tbl;

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_ftm_upd_info);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_SEC_CAM;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_SECCAM_FTM_UPD;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	tbl = (struct fwcmd_ftm_upd_info *)PLTFM_MALLOC(h2c_info.content_len);
	if (!tbl) {
		PLTFM_MSG_ERR("FTM UPD h2c malloc content fail.\n");
		return MACBUFALLOC;
	}

	tbl->dword0 = info->dword0;
	tbl->dword1 = info->dword1;
	tbl->dword2 = info->dword2;
	tbl->dword3 = info->dword3;

	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_h2c_common(adapter, &h2c_info, (u32 *)tbl);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("FTM UPD h2c TX fail.\n");
			PLTFM_FREE(tbl, h2c_info.content_len);
			return ret;
		}
		PLTFM_FREE(tbl, h2c_info.content_len);
	} else {
		return MACNOFW;
	}

	return ret;
}

u32 mac_update_ftm_cancel_info(struct mac_ax_adapter *adapter,
			       struct fwcmd_ftm_cancel_info *info)
{
	u32 ret = MACSUCCESS;
	struct h2c_info h2c_info = {0};
	struct fwcmd_ftm_cancel_info *tbl;

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_ftm_cancel_info);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_SEC_CAM;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_SECCAM_FTM_CANCEL;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	tbl = (struct fwcmd_ftm_cancel_info *)PLTFM_MALLOC(h2c_info.content_len);
	if (!tbl) {
		PLTFM_MSG_ERR("FTM CANCEL h2c malloc content fail.\n");
		return MACBUFALLOC;
	}

	tbl->dword0 = info->dword0;

	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_h2c_common(adapter, &h2c_info, (u32 *)tbl);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("FTM CANCEL h2c TX fail.\n");
			PLTFM_FREE(tbl, h2c_info.content_len);
			return ret;
		}
		PLTFM_FREE(tbl, h2c_info.content_len);
	} else {
		return MACNOFW;
	}

	return ret;
}

u32 mac_update_ftm_info(struct mac_ax_adapter *adapter,
			struct fwcmd_ftm_info *info)
{
	u32 ret = MACSUCCESS;
	struct h2c_info h2c_info = {0};
	struct fwcmd_ftm_info *tbl;

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_ftm_info);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_SEC_CAM;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_SECCAM_FTM;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	tbl = (struct fwcmd_ftm_info *)PLTFM_MALLOC(h2c_info.content_len);
	if (!tbl) {
		PLTFM_MSG_ERR("FTM PROC h2c malloc content fail.\n");
		return MACBUFALLOC;
	}

	tbl->dword0 = info->dword0;
	tbl->dword1 = info->dword1;
	tbl->dword2 = info->dword2;
	tbl->dword3 = info->dword3;
	tbl->dword4 = info->dword4;
	tbl->dword5 = info->dword5;
	tbl->dword6 = info->dword6;
	tbl->dword7 = info->dword7;
	tbl->dword8 = info->dword8;
	tbl->dword9 = info->dword9;

	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_h2c_common(adapter, &h2c_info, (u32 *)tbl);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("FTM PROC h2c TX fail.\n");
			PLTFM_FREE(tbl, h2c_info.content_len);
			return ret;
		}
		PLTFM_FREE(tbl, h2c_info.content_len);
	} else {
		return MACNOFW;
	}

	return ret;
}

u32 fill_ftm_upd_para(struct mac_ax_adapter *adapter,
		      struct mac_ax_ftm_upd_para *upd_info,
		      struct fwcmd_ftm_upd_info *ftm_upd_fw_info)
{
	ftm_upd_fw_info->dword0 =
	cpu_to_le32(SET_WORD(upd_info->upd_pstimer_high,
			     FWCMD_H2C_FTM_INFO_FIRST_PSTIMER_HIGH));

	ftm_upd_fw_info->dword1 =
	cpu_to_le32(SET_WORD(upd_info->upd_pstimer_low,
			     FWCMD_H2C_FTM_INFO_FIRST_PSTIMER_LOW));

	ftm_upd_fw_info->dword2 =
	cpu_to_le32(SET_WORD(upd_info->itvl, FWCMD_H2C_FTM_INFO_ITVL));

	return MACSUCCESS;
}

u32 fill_ftm_cancel_para(struct mac_ax_adapter *adapter,
			 struct mac_ax_ftm_cancel_para *cancel_info,
			 struct fwcmd_ftm_cancel_info *ftm_cancel_fw_info)
{
	ftm_cancel_fw_info->dword0 =
	cpu_to_le32(SET_WORD(cancel_info->ch_parm_back.pri_ch, FWCMD_H2C_CH_SWITCH_PRI_CH) |
		    SET_WORD(cancel_info->ch_parm_back.central_ch, FWCMD_H2C_CH_SWITCH_CENTRAL_CH) |
		    SET_WORD(cancel_info->ch_parm_back.bw, FWCMD_H2C_CH_SWITCH_BW) |
		    SET_WORD(cancel_info->ch_parm_back.ch_band, FWCMD_H2C_CH_SWITCH_CH_BAND
) |
		    (cancel_info->ch_parm_back.band ? FWCMD_H2C_CH_SWITCH_BAND : 0) |
		    (cancel_info->ch_parm_back.reload_rf ? FWCMD_H2C_CH_SWITCH_RELOAD_RF : 0
));

	return MACSUCCESS;
}

u32 fill_ftm_para(struct mac_ax_adapter *adapter,
		  struct mac_ax_ftm_para *ftm_info,
		  struct fwcmd_ftm_info *ftm_fw_info)
{
	ftm_fw_info->dword0 =
	cpu_to_le32(SET_WORD(ftm_info->asap, FWCMD_H2C_FTM_INFO_ASAP) |
		    SET_WORD(ftm_info->pktid_iftmr, FWCMD_H2C_FTM_INFO_PKTID_IFTMR) |
		    SET_WORD(ftm_info->pktid_ftmr, FWCMD_H2C_FTM_INFO_PKTID_IFTM) |
		    SET_WORD(ftm_info->tsf_timer_offset, FWCMD_H2C_FTM_INFO_TSF_TIMER_OFFSET));

	ftm_fw_info->dword1 =
	cpu_to_le32(SET_WORD(ftm_info->partial_tsf_timer, FWCMD_H2C_FTM_INFO_PARTIAL_TSF_TIMER) |
		    SET_WORD(ftm_info->rsp_ch, FWCMD_H2C_FTM_INFO_RSP_CH) |
		    SET_WORD(ftm_info->mode, FWCMD_H2C_FTM_INFO_MODE));

	ftm_fw_info->dword2 =
	cpu_to_le32(SET_WORD(ftm_info->first_pstimer_high, FWCMD_H2C_FTM_INFO_FIRST_PSTIMER_HIGH));

	ftm_fw_info->dword3 =
	cpu_to_le32(SET_WORD(ftm_info->first_pstimer_low, FWCMD_H2C_FTM_INFO_FIRST_PSTIMER_LOW));

	ftm_fw_info->dword4 =
	cpu_to_le32(SET_WORD(ftm_info->itvl, FWCMD_H2C_FTM_INFO_ITVL));

	ftm_fw_info->dword5 =
	cpu_to_le32(SET_WORD(ftm_info->burst_cnt, FWCMD_H2C_FTM_INFO_BURST_CNT));

	ftm_fw_info->dword6 =
	cpu_to_le32(SET_WORD(ftm_info->ch_parm_trg.pri_ch, FWCMD_H2C_CH_SWITCH_PRI_CH) |
		    SET_WORD(ftm_info->ch_parm_trg.central_ch, FWCMD_H2C_CH_SWITCH_CENTRAL_CH) |
		    SET_WORD(ftm_info->ch_parm_trg.bw, FWCMD_H2C_CH_SWITCH_BW) |
		    SET_WORD(ftm_info->ch_parm_trg.ch_band, FWCMD_H2C_CH_SWITCH_CH_BAND) |
		    (ftm_info->ch_parm_trg.band ? FWCMD_H2C_CH_SWITCH_BAND : 0) |
		    (ftm_info->ch_parm_trg.reload_rf ? FWCMD_H2C_CH_SWITCH_RELOAD_RF : 0));

	ftm_fw_info->dword7 =
	cpu_to_le32(SET_WORD(ftm_info->ch_parm_ori.pri_ch, FWCMD_H2C_CH_SWITCH_PRI_CH) |
		    SET_WORD(ftm_info->ch_parm_ori.central_ch, FWCMD_H2C_CH_SWITCH_CENTRAL_CH) |
		    SET_WORD(ftm_info->ch_parm_ori.bw, FWCMD_H2C_CH_SWITCH_BW) |
		    SET_WORD(ftm_info->ch_parm_ori.ch_band, FWCMD_H2C_CH_SWITCH_CH_BAND) |
		    (ftm_info->ch_parm_ori.band ? FWCMD_H2C_CH_SWITCH_BAND : 0) |
		    (ftm_info->ch_parm_ori.reload_rf ? FWCMD_H2C_CH_SWITCH_RELOAD_RF : 0));

	return MACSUCCESS;
}

u32 mac_ista_ftm_upd_para(struct mac_ax_adapter *adapter,
			  struct mac_ax_ftm_upd_para *ftm_upd_info)
{
	u32 ftm_info[3] = {0}, ret = 0;

	ret = fill_ftm_upd_para(adapter, ftm_upd_info,
				(struct fwcmd_ftm_upd_info *)(&ftm_info));
	if (ret != MACSUCCESS)
		return ret;

	ret = (u8)mac_update_ftm_upd_info(adapter, (struct fwcmd_ftm_upd_info *)(&
ftm_info));
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

u32 mac_ista_ftm_cancel(struct mac_ax_adapter *adapter,
			struct mac_ax_ftm_cancel_para *ftm_cancel)
{
	u32 ftm_info = 0, ret = 0;

	ret = fill_ftm_cancel_para(adapter, ftm_cancel,
				   (struct fwcmd_ftm_cancel_info *)(&ftm_info));
	if (ret != MACSUCCESS)
		return ret;

	ret = (u8)mac_update_ftm_cancel_info(adapter,
					     (struct fwcmd_ftm_cancel_info *)(&ftm_info));
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

u32 mac_ista_ftm_proc(struct mac_ax_adapter *adapter,
		      struct mac_ax_ftm_para *ftmr)
{
	u32 ftm_fw_info[8] = {0}, ret = 0;

	ret = fill_ftm_para(adapter, ftmr,
			    (struct fwcmd_ftm_info *)(&ftm_fw_info));
	if (ret != MACSUCCESS)
		return ret;

	ret = (u8)mac_update_ftm_info(adapter,
				      (struct fwcmd_ftm_info *)(&ftm_fw_info));
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

u32 mac_ista_ftm_enable(struct mac_ax_adapter *adapter,
			u8 macid, bool enable)
{
	u32 ret = 0, val32 = 0;
	struct rtw_hal_mac_ax_cctl_info info, msk = {0};
	struct mac_role_tbl *role;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	role = mac_role_srch(adapter, macid);
	if (!role) {
		PLTFM_MSG_ERR("%s: The MACID%d does not exist\n",
			      __func__, macid);
		return MACNOITEM;
	}

	if (enable) {
		ret = check_mac_en(adapter, 0, MAC_AX_CMAC_SEL);
		if (ret != MACSUCCESS)
			return ret;

		// init BB FTM CLK
		val32 = MAC_REG_R32(0x10014);
		val32 |= (BIT5 | BIT6);
		MAC_REG_W32(0x10014, val32);

		val32 = MAC_REG_R32(0x109C4);
		val32 |= (BIT31 | BIT28 | BIT26);
		MAC_REG_W32(0x109C4, val32);

		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || \
	MAC_AX_8852BT_SUPPORT
			// init TRX FTM
			val32 = MAC_REG_R32(R_AX_WMAC_FTM_CTL);
			val32 |= (B_AX_FTM_EN | B_AX_RXFTM_EN);
			MAC_REG_W32(R_AX_WMAC_FTM_CTL, val32);

			// init TSF
			val32 = MAC_REG_R32(R_AX_PORT_CFG_P0);
			val32 |= (B_AX_PORT_FUNC_EN_P0);
			MAC_REG_W32(R_AX_PORT_CFG_P0, val32);
#endif
		}

		// Trun on FTM report
		msk.acq_rpt_en = 1;
		info.acq_rpt_en = 1;
		msk.mgq_rpt_en = 1;
		info.mgq_rpt_en = 1;
		msk.ulq_rpt_en = 1;
		info.ulq_rpt_en = 1;
		ret = mac_upd_cctl_info(adapter, &info, &msk, macid, 1);
	} else {
		// Turn off BB FTM CLK
		val32 = MAC_REG_R32(0x10014);
		val32 &= ~(BIT5 | BIT6);
		MAC_REG_W32(0x10014, val32);

		val32 = MAC_REG_R32(0x109C4);
		val32 &= ~(BIT31 | BIT28 | BIT26);
		MAC_REG_W32(0x109C4, val32);

		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || \
	MAC_AX_8852BT_SUPPORT
			// Turn off TRX FTM
			val32 = MAC_REG_R32(R_AX_WMAC_FTM_CTL);
			val32 &= ~(B_AX_FTM_EN | B_AX_RXFTM_EN);
			MAC_REG_W32(R_AX_WMAC_FTM_CTL, val32);
#endif
		}

		// Trun off FTM report
		msk.acq_rpt_en = 1;
		info.acq_rpt_en = 0;
		msk.mgq_rpt_en = 1;
		info.mgq_rpt_en = 0;
		msk.ulq_rpt_en = 1;
		info.ulq_rpt_en = 0;
		ret = mac_upd_cctl_info(adapter, &info, &msk, macid, 1);
	}

	return MACSUCCESS;
}

#endif