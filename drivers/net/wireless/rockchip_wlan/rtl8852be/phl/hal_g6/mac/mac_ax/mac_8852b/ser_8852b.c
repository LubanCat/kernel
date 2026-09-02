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

#include "ser_8852b.h"
#include "../../type.h"

#if MAC_AX_8852B_SUPPORT

static u32 cdma_imr_enable_patch_8852b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 ret = MACSUCCESS, reg;
	u32 msk = 0;
/*
 * AP  Disable B_AX_STS_FSM_HANG_ERROR_IMR
 * STA Enable  B_AX_STS_FSM_HANG_ERROR_IMR (Wait for "Scan+SER L0")
 */
	if (chk_patch_cmac_dma_err_fa(adapter) == PATCH_ENABLE) {
		msk |= B_AX_RXDATA_FSM_HANG_ERROR_IMR;
		reg = band == MAC_AX_BAND_0 ? R_AX_DLE_CTRL : R_AX_DLE_CTRL_C1;
		ret = write_mac_reg_auto_ofld(adapter, reg,
					      msk,
					      0x0,
					      IMR_IO_OFLD_NON_FLUSH);
		if (ret)
			PLTFM_MSG_ERR("[ERR]%sIO ofld fail", __func__);
	}

	return ret;
}

static u32 phy_intf_imr_enable_patch_8852b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 ret, reg;
	u32 msk = 0;

	msk = SET_CLR_WORD(msk, B_AX_PHYINTF_TIMEOUT_THR_MSK, B_AX_PHYINTF_TIMEOUT_THR);
	reg = band == MAC_AX_BAND_0 ?
		R_AX_PHYINFO_ERR_IMR : R_AX_PHYINFO_ERR_IMR_C1;
	ret = write_mac_reg_auto_ofld(adapter, reg,
				      msk,
				      0x7,
				      IMR_IO_OFLD_NON_FLUSH);
	if (ret)
		PLTFM_MSG_ERR("[ERR]%sIO ofld fail", __func__);

	return ret;
}

u32 ser_imr_config_patch_8852b(struct mac_ax_adapter *adapter, u8 band,
			       enum mac_ax_hwmod_sel sel)
{
	u32 ret = MACSUCCESS;
#if MAC_USB_IO_ACC_ON
	u32 ofldcap;
	struct mac_ax_ops *mops = adapter_to_mac_ops(adapter);
#endif

	ret = check_mac_en(adapter, band, sel);
	if (ret) {
		PLTFM_MSG_ERR("MAC%d band%d is not ready\n", sel, band);
		return ret;
	}

	if (sel == MAC_AX_DMAC_SEL) {
		ret = MACSUCCESS;
	} else if (sel == MAC_AX_CMAC_SEL) {
		ret = phy_intf_imr_enable_patch_8852b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]phy_intf_imr_enable %d\n", ret);
			return ret;
		}

		ret = cdma_imr_enable_patch_8852b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]cdma_imr_enable %d\n", ret);
			return ret;
		}
	} else {
		PLTFM_MSG_ERR("illegal sel %d\n", sel);
		return MACNOITEM;
	}
#if MAC_USB_IO_ACC_ON
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mops->get_hw_value(adapter, MAC_AX_HW_GET_FW_CAP, &ofldcap);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("Get MAC_AX_HW_GET_FW_CAP fail %d\n", ret);
			return ret;
		}
		if (ofldcap == IO_OFLD_DIS)
			return MACSUCCESS;
		ret = mac_cmd_ofld(adapter);
		if (ret != MACSUCCESS)
			PLTFM_MSG_ERR("[ERR]mac_cmd_ofld fail : %d\n", ret);
	}
#endif
	return ret;
}
#endif
