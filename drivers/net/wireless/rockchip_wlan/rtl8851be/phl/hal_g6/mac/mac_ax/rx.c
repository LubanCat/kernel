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
#include "rx.h"
#include "mac_priv.h"

u32 set_resp_stat_rts_chk(struct mac_ax_adapter *adapter,
			  struct mac_ax_set_resp_stat_rts_chk_cfg *cfg)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ret = MACSUCCESS, reg, val32;

	ret = check_mac_en(adapter, cfg->band, MAC_AX_CMAC_SEL);
	if (ret)
		return ret;

	reg = (cfg->band == MAC_AX_BAND_0) ? R_AX_RSP_CHK_SIG : R_AX_RSP_CHK_SIG_C1;
	val32 = MAC_REG_R32(reg);

	if (cfg->enable)
		val32 |= B_AX_RSP_STATIC_RTS_CHK_SERV_BW_EN;
	else
		val32 &= ~B_AX_RSP_STATIC_RTS_CHK_SERV_BW_EN;

	MAC_REG_W32(reg, val32);
	return MACSUCCESS;
}

u32 set_bacam_mode(struct mac_ax_adapter *adapter, u8 mode_sel)
{
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val32;

	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
		val32 = MAC_REG_R32(R_AX_RESPBA_CAM_CTRL) & (~B_AX_BACAM_ENT_CFG);

		if (mode_sel)
			MAC_REG_W32(R_AX_RESPBA_CAM_CTRL, val32 | B_AX_BACAM_ENT_CFG);
		else
			MAC_REG_W32(R_AX_RESPBA_CAM_CTRL, val32);
		return MACSUCCESS;
	}
#endif
	return MACHWNOSUP;
}

u32 get_bacam_mode(struct mac_ax_adapter *adapter, u8 *mode_sel)
{
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
		*mode_sel = MAC_REG_R8(R_AX_RESPBA_CAM_CTRL) & B_AX_BACAM_ENT_CFG ? 1 : 0;
		return MACSUCCESS;
	}
#endif
	return MACHWNOSUP;
}

u32 set_rxd_zld_en(struct mac_ax_adapter *adapter, u8 enable)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val32;
	u32 ret;

	ret = check_mac_en(adapter, MAC_AX_BAND_0, MAC_AX_CMAC_SEL);
	if (ret)
		return ret;

	if (enable) {
		val32 = MAC_REG_R32(R_AX_ZLENDEL_COUNT);
		val32 |= B_AX_RXD_DELI_EN;
		MAC_REG_W32(R_AX_ZLENDEL_COUNT, val32);
	} else {
		val32 = MAC_REG_R32(R_AX_ZLENDEL_COUNT);
		val32 &= ~B_AX_RXD_DELI_EN;
		MAC_REG_W32(R_AX_ZLENDEL_COUNT, val32);
	}

	return MACSUCCESS;
}