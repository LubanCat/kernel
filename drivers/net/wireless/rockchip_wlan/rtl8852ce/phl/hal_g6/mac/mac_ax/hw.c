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

#include "hw.h"
#include "coex.h"
#include "twt.h"
#include "fwofld.h"
#include "mac_priv.h"
#include "trxcfg.h"
#include "common.h"
#include "sounding.h"
#include "rx.h"
#include "trx_desc.h"

static struct mac_ax_host_rpr_cfg rpr_cfg_poh = {
	121, /* agg */
	255, /* tmr */
	0, /* agg_def */
	0, /* tmr_def */
	0, /* rsvd */
	MAC_AX_FUNC_EN, /* txok_en */
	MAC_AX_FUNC_EN, /* rty_lmt_en */
	MAC_AX_FUNC_EN, /* lft_drop_en */
	MAC_AX_FUNC_EN /* macid_drop_en */
};

static struct mac_ax_host_rpr_cfg rpr_cfg_stf = {
	121, /* agg */
	255, /* tmr */
	0, /* agg_def */
	0, /* tmr_def */
	0, /* rsvd */
	MAC_AX_FUNC_DIS, /* txok_en */
	MAC_AX_FUNC_DIS, /* rty_lmt_en */
	MAC_AX_FUNC_DIS, /* lft_drop_en */
	MAC_AX_FUNC_DIS /* macid_drop_en */
};

static struct mac_ax_drv_wdt_ctrl wdt_ctrl_def = {
	MAC_AX_PCIE_IGNORE,
	MAC_AX_PCIE_ENABLE
};

struct mac_ax_hw_info *mac_get_hw_info(struct mac_ax_adapter *adapter)
{
	return adapter->hw_info->done ? adapter->hw_info : NULL;
}

u32 set_enable_bb_rf(struct mac_ax_adapter *adapter, u8 enable)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 value32, ret;
	u8 value8;
	u8 wl_rfc_s0 = 0;
	u8 wl_rfc_s1 = 0;

	if (enable == 1) {
		value8 = MAC_REG_R8(R_AX_SYS_FUNC_EN);
		value8 |= B_AX_FEN_BBRSTB | B_AX_FEN_BB_GLB_RSTN;
		MAC_REG_W8(R_AX_SYS_FUNC_EN, value8);
#if MAC_AX_8852A_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A)) {
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 |= B_AX_WLRF1_CTRL_7 | B_AX_WLRF1_CTRL_1 |
				   B_AX_WLRF_CTRL_7 | B_AX_WLRF_CTRL_1;
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);

			value8 = PHYREG_SET_ALL_CYCLE;
			MAC_REG_W8(R_AX_PHYREG_SET, value8);
			ret = MACSUCCESS;
		}
#endif
#if MAC_AX_8852B_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852B)) {
			/* 0x200[18:17] = 2'b01 */
			value32 = MAC_REG_R32(R_AX_SPS_DIG_ON_CTRL0);
			value32 = SET_CLR_WORD(value32, 0x1, B_AX_REG_ZCDC_H);
			MAC_REG_W32(R_AX_SPS_DIG_ON_CTRL0, value32);

			/* RDC KS/BB suggest : write 1 then write 0 then write 1 */
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 & ~B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);

			wl_rfc_s0 = 0xC7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S0, wl_rfc_s0,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			wl_rfc_s1 = 0xC7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S1, wl_rfc_s1,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			#if defined(CONFIG_RTL8852BP)
				value8 = 0xd;
				ret = mac_write_xtal_si(adapter, XTAL3, value8,
							FULL_BIT_MASK);
				if (ret) {
					PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
					return ret;
				}
			#endif	//#if defined(CONFIG_RTL8852BP)

			value8 = PHYREG_SET_XYN_CYCLE;
			MAC_REG_W8(R_AX_PHYREG_SET, value8);
		}
#endif
#if MAC_AX_8852C_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C)) {
			/* 0x200[18:17] = 2'b01 */
			value32 = MAC_REG_R32(R_AX_SPS_DIG_ON_CTRL0);
			value32 = SET_CLR_WORD(value32, 0x1, B_AX_REG_ZCDC_H);
			MAC_REG_W32(R_AX_SPS_DIG_ON_CTRL0, value32);

			/* RDC KS/BB suggest : write 1 then write 0 then write 1 */
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 & ~B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);

			// for ADC PWR setting
			value32 = MAC_REG_R32(R_AX_AFE_OFF_CTRL1);
			value32 = (value32 & ~LDO2PW_LDO_VSEL);
			value32 |= SET_WORD(0x1, B_AX_S0_LDO_VSEL_F) |
				   SET_WORD(0x1, B_AX_S1_LDO_VSEL_F);
			MAC_REG_W32(R_AX_AFE_OFF_CTRL1, value32);

			value8 = 0x7;
			ret = mac_write_xtal_si(adapter, XTAL0, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0x6c;
			ret = mac_write_xtal_si(adapter, XTAL_SI_ANAPAR_WL, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0xc7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S0, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0xc7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S1, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0xd;
			ret = mac_write_xtal_si(adapter, XTAL3, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}
		}
#endif
#if MAC_AX_8192XB_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB)) {
			/* RDC KS/BB suggest : write 1 then write 0 then write 1 */
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 & ~B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);

			value8 = 0x7;
			ret = mac_write_xtal_si(adapter, XTAL0, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0x6c;
			ret = mac_write_xtal_si(adapter, XTAL_SI_ANAPAR_WL, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0xc7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S0, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0xc7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S1, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0xf;
			ret = mac_write_xtal_si(adapter, XTAL3, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}
		}
#endif
#if MAC_AX_8851B_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8851B)) {
			/* RDC KS/BB suggest : write 1 then write 0 then write 1 */
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 & ~B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);

			wl_rfc_s0 = 0xC7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S0, wl_rfc_s0,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			wl_rfc_s1 = 0xC7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S1, wl_rfc_s1,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = PHYREG_SET_XYN_CYCLE;
			MAC_REG_W8(R_AX_PHYREG_SET, value8);
		}
#endif
#if MAC_AX_8851E_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8851E)) {
			/* 0x200[18:17] = 2'b01 */
			value32 = MAC_REG_R32(R_AX_SPS_DIG_ON_CTRL0);
			value32 = SET_CLR_WORD(value32, 0x1, B_AX_REG_ZCDC_H);
			MAC_REG_W32(R_AX_SPS_DIG_ON_CTRL0, value32);

			/* RDC KS/BB suggest : write 1 then write 0 then write 1 */
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 & ~B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);

			// for ADC PWR setting
			value32 = MAC_REG_R32(R_AX_AFE_OFF_CTRL1);
			value32 = (value32 & ~LDO2PW_LDO_VSEL);
			value32 |= SET_WORD(0x1, B_AX_S0_LDO_VSEL_F) |
				   SET_WORD(0x1, B_AX_S1_LDO_VSEL_F);
			MAC_REG_W32(R_AX_AFE_OFF_CTRL1, value32);

			value8 = 0x7;
			ret = mac_write_xtal_si(adapter, XTAL0, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0x6c;
			ret = mac_write_xtal_si(adapter, XTAL_SI_ANAPAR_WL, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0xc7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S0, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0xc7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S1, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0xd;
			ret = mac_write_xtal_si(adapter, XTAL3, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}
		}
#endif
#if MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			/* 0x200[18:17] = 2'b01 */
			value32 = MAC_REG_R32(R_AX_SPS_DIG_ON_CTRL0);
			value32 = SET_CLR_WORD(value32, 0x1, B_AX_REG_ZCDC_H);
			MAC_REG_W32(R_AX_SPS_DIG_ON_CTRL0, value32);

			/* RDC KS/BB suggest : write 1 then write 0 then write 1 */
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 & ~B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);

			// for ADC PWR setting
			value32 = MAC_REG_R32(R_AX_AFE_OFF_CTRL1);
			value32 = (value32 & ~LDO2PW_LDO_VSEL);
			value32 |= SET_WORD(0x1, B_AX_S0_LDO_VSEL_F) |
				   SET_WORD(0x1, B_AX_S1_LDO_VSEL_F);
			MAC_REG_W32(R_AX_AFE_OFF_CTRL1, value32);

			value8 = 0x7;
			ret = mac_write_xtal_si(adapter, XTAL0, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0x6c;
			ret = mac_write_xtal_si(adapter, XTAL_SI_ANAPAR_WL, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0xc7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S0, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0xc7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S1, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = 0xd;
			ret = mac_write_xtal_si(adapter, XTAL3, value8,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}
		}
#endif
#if MAC_AX_8852BT_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			/* 0x200[18:17] = 2'b01 */
			value32 = MAC_REG_R32(R_AX_SPS_DIG_ON_CTRL0);
			value32 = SET_CLR_WORD(value32, 0x1, B_AX_REG_ZCDC_H);
			MAC_REG_W32(R_AX_SPS_DIG_ON_CTRL0, value32);

			/* RDC KS/BB suggest : write 1 then write 0 then write 1 */
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 & ~B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 | B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);

			// for ADC PWR setting
			value32 = MAC_REG_R32(R_AX_AFE_OFF_CTRL1);
			value32 = (value32 & ~LDO2PW_LDO_VSEL);
			value32 |= SET_WORD(0x1, B_AX_S0_LDO_VSEL_F) |
				   SET_WORD(0x1, B_AX_S1_LDO_VSEL_F);
			MAC_REG_W32(R_AX_AFE_OFF_CTRL1, value32);

			wl_rfc_s0 = 0xC7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S0, wl_rfc_s0,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			wl_rfc_s1 = 0xC7;
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S1, wl_rfc_s1,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			value8 = PHYREG_SET_XYN_CYCLE;
			MAC_REG_W8(R_AX_PHYREG_SET, value8);
		}
#endif
		adapter->sm.bb0_func = MAC_AX_FUNC_ON;
	} else {
		adapter->sm.bb0_func = MAC_AX_FUNC_OFF;

#if (MAC_AX_8852B_SUPPORT || MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || \
		MAC_AX_8851B_SUPPORT || MAC_AX_8852D_SUPPORT || MAC_AX_8852BT_SUPPORT)
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 & ~B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
		}
#endif

		value8 = MAC_REG_R8(R_AX_SYS_FUNC_EN);
		value8 &= (~(B_AX_FEN_BBRSTB | B_AX_FEN_BB_GLB_RSTN));
		MAC_REG_W8(R_AX_SYS_FUNC_EN, value8);
#if MAC_AX_8852A_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A)) {
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 &= (~(B_AX_WLRF1_CTRL_7 | B_AX_WLRF1_CTRL_1 |
				      B_AX_WLRF_CTRL_7 | B_AX_WLRF_CTRL_1));
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
		}
#endif
#if MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 & ~B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			ret = mac_read_xtal_si(adapter, XTAL_SI_WL_RFC_S0, &wl_rfc_s0);
			if (ret) {
				PLTFM_MSG_ERR("Read XTAL_SI fail!\n");
				return ret;
			}
			wl_rfc_s0 = (wl_rfc_s0 & 0xF8);
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S0, wl_rfc_s0,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}

			ret = mac_read_xtal_si(adapter, XTAL_SI_WL_RFC_S1, &wl_rfc_s1);
			if (ret) {
				PLTFM_MSG_ERR("Read XTAL_SI fail!\n");
				return ret;
			}
			wl_rfc_s1 = (wl_rfc_s1 & 0xF8);
			ret = mac_write_xtal_si(adapter, XTAL_SI_WL_RFC_S1, wl_rfc_s1,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}
		}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8851E_SUPPORT || MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851E) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			value32 = MAC_REG_R32(R_AX_WLRF_CTRL);
			value32 = (value32 & ~B_AX_AFC_AFEDIG);
			MAC_REG_W32(R_AX_WLRF_CTRL, value32);
			return MACSUCCESS;
		}
#endif
	}

	return MACSUCCESS;
}

static u32 set_append_fcs(struct mac_ax_adapter *adapter, u8 enable)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u8 value8;

	value8 = MAC_REG_R8(R_AX_MPDU_PROC);
	value8 = enable == 1 ? value8 | B_AX_APPEND_FCS :
			value8 & ~B_AX_APPEND_FCS;
	MAC_REG_W8(R_AX_MPDU_PROC, value8);

	return MACSUCCESS;
}

static u32 set_accept_icverr(struct mac_ax_adapter *adapter, u8 enable)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u8 value8;

	value8 = MAC_REG_R8(R_AX_MPDU_PROC);
	value8 = enable == 1 ? (value8 | B_AX_A_ICV_ERR) :
			(value8 & ~B_AX_A_ICV_ERR);
	MAC_REG_W8(R_AX_MPDU_PROC, value8);

	return MACSUCCESS;
}

u32 set_gt3_timer(struct mac_ax_adapter *adapter,
		  struct mac_ax_gt3_cfg *cfg)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val32;

	val32 = (cfg->count_en ? B_AX_GT_COUNT_EN : 0) |
		  (cfg->mode ? B_AX_GT_MODE : 0) |
		  (cfg->gt3_en ? B_AX_GT_EN : 0) |
		  (cfg->sort_en ? B_AX_GT_SORT_EN : 0) |
		  SET_WORD(cfg->timeout, B_AX_GT_DATA);

	switch (adapter->hw_info->chip_id) {
	case MAC_AX_CHIP_ID_8852A:
	case MAC_AX_CHIP_ID_8852B:
	case MAC_AX_CHIP_ID_8851B:
	case MAC_AX_CHIP_ID_8852BT:
		MAC_REG_W32(R_AX_GT3_CTRL, val32);
		break;
	default:
		MAC_REG_W32(R_AX_GT3_CTRL_V1, val32);
	}

	return MACSUCCESS;
}

u32 set_xtal_aac(struct mac_ax_adapter *adapter, u8 aac_mode)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

#if MAC_AX_8852A_SUPPORT
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A)) {
		u8 val8;

		val8 = MAC_REG_R8(R_AX_XTAL_ON_CTRL2);
		val8 &= ~(0x30);
		val8 |= ((aac_mode & B_AX_AAC_MODE_MSK) << B_AX_AAC_MODE_SH);
		MAC_REG_W8(R_AX_XTAL_ON_CTRL2, val8);
	}
#endif

	return MACSUCCESS;
}

u32 set_partial_pld_mode(struct mac_ax_adapter *adapter, u8 enable)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val32;

	if (enable) {
		val32 = MAC_REG_R32(R_AX_SEC_ENG_CTRL);
		val32 |= B_AX_TX_PARTIAL_MODE;
		MAC_REG_W32(R_AX_SEC_ENG_CTRL, val32);
	} else {
		val32 = MAC_REG_R32(R_AX_SEC_ENG_CTRL);
		val32 &= ~B_AX_TX_PARTIAL_MODE;
		MAC_REG_W32(R_AX_SEC_ENG_CTRL, val32);
	}

	return MACSUCCESS;
}

u32 set_nav_padding(struct mac_ax_adapter *adapter,
		    struct mac_ax_nav_padding *nav)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 reg_txop = nav->band ? R_AX_PTCL_NAV_PROT_LEN_C1 : R_AX_PTCL_NAV_PROT_LEN;
	u32 reg_cnt = nav->band ? R_AX_PROT_C1 : R_AX_PROT;
	u32 ret;
	u8 val8;
#if MAC_AX_FW_REG_OFLD
	u16 tmp;
#endif

	ret = check_mac_en(adapter, nav->band, MAC_AX_CMAC_SEL);
	if (ret)
		return ret;

#if MAC_AX_FW_REG_OFLD
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		tmp = nav->nav_pad_en ? nav->nav_padding : 0;
		ret = MAC_REG_W16_OFLD((u16)reg_txop,
				       tmp, 0);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("%s: write offload fail %d",
				      __func__, ret);
			return ret;
		}
		ret = MAC_REG_W16_OFLD((u16)reg_cnt,
				       tmp, 0);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("%s: write offload fail %d",
				      __func__, ret);
			return ret;
		}
		ret = MAC_REG_W_OFLD((u16)reg_cnt, B_AX_NAV_OVER_TXOP_EN,
				     nav->over_txop_en ? 1 : 0, 1);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("%s: write offload fail %d",
				      __func__, ret);
			return ret;
		}

		return MACSUCCESS;
	}
#endif

	if (nav->nav_pad_en) {
		MAC_REG_W16(reg_txop, nav->nav_padding);
		MAC_REG_W16(reg_cnt, nav->nav_padding);
		val8 = MAC_REG_R8(reg_cnt + 2);
		if (nav->over_txop_en)
			val8 |= BIT(0);
		else
			val8 &= ~BIT(0);
		MAC_REG_W8(reg_cnt + 2, val8);
	} else {
		MAC_REG_W16(reg_txop, 0);
		MAC_REG_W16(reg_cnt, 0);
	}

	return MACSUCCESS;
}

u32 set_core_swr_volt(struct mac_ax_adapter *adapter,
		      enum mac_ax_core_swr_volt volt_sel)
{
	u8 i, j, adjust = 0;
	s8 sign = 0, v, val8;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	val8 = adapter->hw_info->core_swr_volt_sel - volt_sel;
	if (val8 == 0) {
		return MACSUCCESS;
	} else if (val8 > 0) {
		adjust = adapter->hw_info->core_swr_volt_sel - volt_sel;
		sign = -1;
	} else {
		adjust = volt_sel - adapter->hw_info->core_swr_volt_sel;
		sign = 1;
	}

	for (i = 0; i < adjust; i++) {
		val8 = MAC_REG_R8(R_AX_SPS_DIG_ON_CTRL0);
		v = GET_FIELD(val8, B_AX_VOL_L1);
		v += sign;
		if (v < CORE_SWR_VOLT_MIN)
			v = CORE_SWR_VOLT_MIN;
		else if (v > CORE_SWR_VOLT_MAX)
			v = CORE_SWR_VOLT_MAX;

		val8 = SET_CLR_WORD(val8, v, B_AX_VOL_L1);
		MAC_REG_W8(R_AX_SPS_DIG_ON_CTRL0, val8);
		for (j = 0; j < POLL_SWR_VOLT_CNT; j++)
			PLTFM_DELAY_US(POLL_SWR_VOLT_US);
	}

	if (volt_sel == MAC_AX_SWR_NORM) {
		val8 = MAC_REG_R8(R_AX_SPS_DIG_ON_CTRL0);
		val8 = SET_CLR_WORD(val8, adapter->hw_info->core_swr_volt,
				    B_AX_VOL_L1);
		MAC_REG_W8(R_AX_SPS_DIG_ON_CTRL0, val8);
	}

	adapter->hw_info->core_swr_volt_sel = volt_sel;

	return MACSUCCESS;
}

u32 set_scope_cfg(struct mac_ax_adapter *adapter, struct mac_ax_scope_cfg *cfg)
{
#if MAC_AX_8851E_SUPPORT || MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val32 = 0, addr;

	switch (adapter->hw_info->chip_id) {
	case MAC_AX_CHIP_ID_8192XB:
	case MAC_AX_CHIP_ID_8851E:
		if (cfg->band == MAC_AX_BAND_1) {
			PLTFM_MSG_ERR("[ERR] Band 1 not support in this IC!\n");
			return MACHWNOSUP;
		}
		val32 = SET_CLR_WORD(val32, cfg->fwd, B_AX_SCOPE_FILTER);
		val32 = SET_CLR_WORD(val32, cfg->mode, B_AX_SCOPE_MODE);
		val32 = SET_CLR_WORD(val32, cfg->seg_size, B_AX_SCOPE_LEN);
		if (cfg->append_zero_en)
			val32 = val32 | B_AX_SCOPE_APPZERO;
		MAC_REG_W32(R_AX_SCOPE_CTRL, val32);
		return MACSUCCESS;
	case MAC_AX_CHIP_ID_8852C:
	case MAC_AX_CHIP_ID_8852D:
		addr = (cfg->band == MAC_AX_BAND_1) ? R_AX_SCOPE_CTRL : R_AX_SCOPE_CTRL_C1;
		val32 = SET_CLR_WORD(val32, cfg->fwd, B_AX_SCOPE_FILTER);
		val32 = SET_CLR_WORD(val32, cfg->mode, B_AX_SCOPE_MODE);
		val32 = SET_CLR_WORD(val32, cfg->seg_size, B_AX_SCOPE_LEN);
		if (cfg->append_zero_en)
			val32 = val32 | B_AX_SCOPE_APPZERO;
		MAC_REG_W32(addr, val32);
		return MACSUCCESS;

	case MAC_AX_CHIP_ID_8852A:
	case MAC_AX_CHIP_ID_8852B:
	case MAC_AX_CHIP_ID_8852BT:
	case MAC_AX_CHIP_ID_8851B:
	default:
		PLTFM_MSG_ERR("[ERR] The function not support in this IC\n!");
		break;
	}
#endif
	return MACHWNOSUP;
}

u32 set_cctl_preld(struct mac_ax_adapter *adapter,
		   struct mac_ax_cctl_preld_cfg *cfg)
{
#if MAC_AX_PCIE_SUPPORT
	struct mac_ax_ops *mops = adapter_to_mac_ops(adapter);
	struct rtw_hal_mac_ax_cctl_info info;
	struct rtw_hal_mac_ax_cctl_info mask;

	PLTFM_MEMSET(&mask, 0, sizeof(struct rtw_hal_mac_ax_cctl_info));
	PLTFM_MEMSET(&info, 0, sizeof(struct rtw_hal_mac_ax_cctl_info));

	info.preld_en = cfg->en;
	mask.preld_en = 1;

	mops->upd_cctl_info(adapter, &info, &mask, cfg->macid, TBL_WRITE_OP);

	return MACSUCCESS;
#else
	return MACPROCERR;
#endif
}

u32 cfg_wdt_isr_rst(struct mac_ax_adapter *adapter)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u8 val = MAC_REG_R8(R_AX_PLATFORM_ENABLE);

	val = val & ~B_AX_APB_WRAP_EN;
	MAC_REG_W8(R_AX_PLATFORM_ENABLE, val);

	val = val | B_AX_APB_WRAP_EN;
	MAC_REG_W8(R_AX_PLATFORM_ENABLE, val);

	return MACSUCCESS;
}

u32 mac_set_adapter_info(struct mac_ax_adapter *adapter,
			 struct mac_ax_adapter_info *set)
{
#ifdef RTW_WKARD_GET_PROCESSOR_ID
	adapter->drv_info->adpt_info.cust_proc_id.proc_id.proc_id_h =
		set->cust_proc_id.proc_id.proc_id_h;
	adapter->drv_info->adpt_info.cust_proc_id.proc_id.proc_id_l =
		set->cust_proc_id.proc_id.proc_id_l;
	adapter->drv_info->adpt_info.cust_proc_id.customer_id =
		set->cust_proc_id.customer_id;
	memcpy(set->cust_proc_id.base_board_id,
	       adapter->drv_info->adpt_info.cust_proc_id.base_board_id, BASE_BOARD_ID_LEN);
#endif
	return MACSUCCESS;
}

u32 mac_set_hw_value(struct mac_ax_adapter *adapter,
		     enum mac_ax_hw_id hw_id, void *val)
{
	struct mac_ax_priv_ops *p_ops = adapter_to_priv_ops(adapter);
	u32 ret = MACSUCCESS;

	if (!val) {
		PLTFM_MSG_ERR("[ERR]: the parameter is NULL in %s\n", __func__);
		return MACNPTR;
	}

	switch (hw_id) {
	case MAC_AX_HW_SETTING:
		break;
	case MAC_AX_HW_SET_ID_PAUSE:
		ret = set_macid_pause(adapter,
				      (struct mac_ax_macid_pause_cfg *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_SET_MULTI_ID_PAUSE:
		ret = macid_pause(adapter,
				  (struct mac_ax_macid_pause_grp *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_SET_ID_PAUSE_SLEEP:
		ret = set_macid_pause_sleep(adapter,
					    (struct mac_ax_macid_pause_sleep_cfg *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_SET_MULTI_ID_PAUSE_SLEEP:
		ret = macid_pause_sleep(adapter,
					(struct mac_ax_macid_pause_sleep_grp *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_SET_SCH_TXEN_CFG:
		ret = set_hw_sch_tx_en(adapter,
				       (struct mac_ax_sch_tx_en_cfg *)val);
		break;
	case MAC_AX_HW_SET_AMPDU_CFG:
		ret = set_hw_ampdu_cfg(adapter, (struct mac_ax_ampdu_cfg *)val);
		break;
	case MAC_AX_HW_SET_USR_EDCA_PARAM:
		ret =
		set_hw_usr_edca_param(adapter,
				      (struct mac_ax_usr_edca_param *)val);
		break;
	case MAC_AX_HW_SET_USR_TX_RPT_CFG:
		ret =
		set_hw_usr_tx_rpt_cfg(adapter,
				      (struct mac_ax_usr_tx_rpt_cfg *)val);
		break;
	case MAC_AX_HW_SET_EDCA_PARAM:
		ret = set_hw_edca_param(adapter,
					(struct mac_ax_edca_param *)val);
		break;
	case MAC_AX_HW_SET_EDCCA_PARAM:
		ret = set_hw_edcca_param(adapter,
					 (struct mac_ax_edcca_param *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_SET_MUEDCA_PARAM:
		ret = set_hw_muedca_param(adapter,
					  (struct mac_ax_muedca_param *)val);
		break;
	case MAC_AX_HW_SET_MUEDCA_CTRL:
		ret = set_hw_muedca_ctrl(adapter,
					 (struct mac_ax_muedca_cfg *)val);
		break;
	case MAC_AX_HW_SET_TBPPDU_CTRL:
		ret = set_hw_tb_ppdu_ctrl(adapter,
					  (struct mac_ax_tb_ppdu_ctrl *)val);
		break;
	case MAC_AX_HW_SET_HOST_RPR:
		set_host_rpr(adapter, (struct mac_ax_host_rpr_cfg *)val);
		break;
	case MAC_AX_HW_SET_DELAYTX_CFG:
		set_delay_tx_cfg(adapter, (struct mac_ax_delay_tx_cfg *)val);
		break;
	case MAC_AX_HW_SET_BW_CFG:
		ret = cfg_mac_bw(adapter, (struct mac_ax_cfg_bw *)val);
		break;
	case MAC_AX_HW_SET_CH_BUSY_STAT_CFG:
		ret = set_hw_ch_busy_cnt(adapter,
					 (struct mac_ax_ch_busy_cnt_cfg *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_SET_LIFETIME_CFG:
		ret = set_hw_lifetime_cfg(adapter,
					  (struct mac_ax_lifetime_cfg *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_EN_BB_RF:
		ret = set_enable_bb_rf(adapter, *(u8 *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_SET_APP_FCS:
		set_append_fcs(adapter, *(u8 *)val);
		break;
	case MAC_AX_HW_SET_RX_ICVERR:
		set_accept_icverr(adapter, *(u8 *)val);
		break;
	case MAC_AX_HW_SET_CCTL_RTY_LMT:
		set_cctl_rty_limit(adapter,
				   (struct mac_ax_cctl_rty_lmt_cfg *)val);
		break;
	case MAC_AX_HW_SET_COEX_GNT:
		ret = p_ops->cfg_gnt(adapter, (struct mac_ax_coex_gnt *)val);
		break;
	case MAC_AX_HW_SET_SCOREBOARD:
		mac_cfg_sb(adapter, *(u32 *)val);
		break;
	case MAC_AX_HW_SET_POLLUTED:
		mac_cfg_plt(adapter, (struct mac_ax_plt *)val);
		break;
	case MAC_AX_HW_SET_COEX_CTRL:
		p_ops->cfg_ctrl_path(adapter, *(u32 *)val);
		break;
	case MAC_AX_HW_SET_CLR_TX_CNT:
		ret = mac_clr_tx_cnt(adapter, (struct mac_ax_tx_cnt *)val);
		break;
	case MAC_AX_HW_SET_SLOT_TIME:
		mac_set_slot_time(adapter, *(enum mac_ax_slot_time *)val);
		break;
	case MAC_AX_HW_SET_XTAL_AAC_MODE:
		set_xtal_aac(adapter, *(u8 *)val);
		break;
	case MAC_AX_HW_SET_NAV_PADDING:
		ret = set_nav_padding(adapter, (struct mac_ax_nav_padding *)val);
		break;
	case MAC_AX_HW_SET_MAX_TX_TIME:
		ret = mac_set_cctl_max_tx_time(adapter,
					       (struct mac_ax_max_tx_time *)
					       val);
		break;
	case MAC_AX_HW_SET_SS_QUOTA_MODE:
		ret = set_ss_quota_mode(adapter,
					(struct mac_ax_ss_quota_mode_ctrl *)val);
		break;
	case MAC_AX_HW_SET_SS_QUOTA_SETTING:
		ret = ss_set_quotasetting(adapter,
					  (struct mac_ax_ss_quota_setting *)val);
		break;
	case MAC_AX_HW_SET_SCHE_PREBKF:
		ret = scheduler_set_prebkf(adapter,
					   (struct mac_ax_prebkf_setting *)val);
		break;
	case MAC_AX_HW_SET_WDT_ISR_RST:
		ret = cfg_wdt_isr_rst(adapter);
		break;
	case MAC_AX_HW_SET_RESP_ACK:
		ret = set_mac_resp_ack(adapter, (u32 *)val);
		break;
	case MAC_AX_HW_SET_HW_RTS_TH:
		ret = mac_set_hw_rts_th(adapter,
					(struct mac_ax_hw_rts_th *)val);
		break;
	case MAC_AX_HW_SET_TX_RU26_TB:
		ret = mac_set_tx_ru26_tb(adapter, *(u8 *)val);
		break;
	case MAC_AX_HW_SET_BACAM_MODE_SEL:
		ret = set_bacam_mode(adapter, *(u8 *)val);
		break;
	case MAC_AX_HW_SET_CORE_SWR_VOLT:
		ret = set_core_swr_volt(adapter,
					*(enum mac_ax_core_swr_volt *)val);
		break;
	case MAC_AX_HW_SET_PARTIAL_PLD_MODE:
		ret = set_partial_pld_mode(adapter, *(u8 *)val);
		break;
	case MAC_AX_HW_SET_GT3_TIMER:
		ret = set_gt3_timer(adapter,
				    (struct mac_ax_gt3_cfg *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_SET_RRSR_CFG:
		ret = p_ops->set_rrsr_cfg(adapter,
					  (struct mac_ax_rrsr_cfg *)val);
		break;
	case MAC_AX_HW_SET_CTS_RRSR_CFG:
		ret = p_ops->set_cts_rrsr_cfg(adapter,
					      (struct mac_ax_cts_rrsr_cfg *)val);
		break;
	case MAC_AX_HW_SET_ADAPTER:
		ret = mac_set_adapter_info(adapter,
					   (struct mac_ax_adapter_info *)val);
		break;
	case MAC_AX_HW_SET_RESP_ACK_CHK_CCA:
		ret = _patch_rsp_ack(adapter, (struct mac_ax_resp_chk_cca *)val);
		break;
	case MAC_AX_HW_SET_SIFS_R2T_T2T:
		ret =
		set_hw_sifs_r2t_t2t(adapter,
				    (struct mac_ax_sifs_r2t_t2t_ctrl *)val);
		break;
	case MAC_AX_HW_SET_RXD_ZLD_EN:
		ret = set_rxd_zld_en(adapter, *(u8 *)val);
		break;
	case MAC_AX_HW_SET_SER_DBG_LVL:
		ret = mac_dbg_log_lvl_adjust(adapter, (struct mac_debug_log_lvl *)val);
		break;
	case MAC_AX_HW_SET_DATA_RTY_LMT:
		ret = set_data_rty_limit(adapter, (struct mac_ax_rty_lmt *)val);
		break;
	case MAC_AX_HW_SET_CTS2SELF:
		ret = set_cts2self(adapter, (struct mac_ax_cts2self_cfg *)val);
		break;
	case MAC_AX_HW_SET_CSI_RELEASE_CFG:
		ret = set_csi_release_cfg(adapter, (struct mac_ax_csi_release_cfg *)val);
		break;
	case MAC_AX_HW_SET_FREERUN_RST:
		ret = mac_reset_freerun(adapter, (u8 *)val);
		break;
	case MAC_AX_HW_SET_SCOPE_CFG:
		ret = set_scope_cfg(adapter, (struct mac_ax_scope_cfg *)val);
		break;
	case MAC_AX_HW_SET_WD_CHECKSUM_CFG:
		ret = set_wd_checksum_cfg(adapter, (struct mac_ax_wd_checksum_cfg *)val);
		break;
	case MAC_AX_HW_SET_USR_FRAME_TO_ACT_CFG:
		ret = set_hw_usr_frame_te_act_cfg(adapter,
						  (struct mac_ax_usr_frame_to_act_cfg *)val);
		break;
	case MAC_AX_HW_SET_RESP_STAT_RTS_CHK_EN:
		ret = set_resp_stat_rts_chk(adapter,
					    (struct mac_ax_set_resp_stat_rts_chk_cfg *)val);
		break;
#if MAC_AX_SDIO_SUPPORT
	case MAC_AX_HW_SDIO_INFO:
		set_info_sdio(adapter, (struct mac_ax_sdio_info *)val);
		break;
	case MAC_AX_HW_SDIO_TX_MODE:
		ret = p_ops->tx_mode_cfg_sdio(adapter,
					      *(enum mac_ax_sdio_tx_mode *)val);
		break;
	case MAC_AX_HW_SDIO_RX_AGG:
		p_ops->rx_agg_cfg_sdio(adapter, (struct mac_ax_rx_agg_cfg *)val);
		break;
	case MAC_AX_HW_SDIO_TX_AGG:
		ret = tx_agg_cfg_sdio(adapter,
				      (struct mac_ax_sdio_txagg_cfg *)val);
		break;
	case MAC_AX_HW_SDIO_AVAL_PAGE:
		p_ops->aval_page_cfg_sdio(adapter, (struct mac_ax_aval_page_cfg *)val);
		break;
	case MAC_AX_HW_SDIO_MON_WT:
		p_ops->set_wt_cfg_sdio(adapter, *(u8 *)val);
		break;
#endif
#if MAC_AX_PCIE_SUPPORT
	case MAC_AX_HW_PCIE_CFGSPC_SET:
		ret = cfgspc_set_pcie(adapter,
				      (struct mac_ax_pcie_cfgspc_param *)val);
		break;
	case MAC_AX_HW_PCIE_RST_BDRAM:
		ret = p_ops->rst_bdram_pcie(adapter, *(u8 *)val);
		break;
	case MAX_AX_HW_PCIE_LTR_SW_TRIGGER:
		ret = p_ops->ltr_sw_trigger(adapter,
					    *(enum mac_ax_pcie_ltr_sw_ctrl *)val);
		break;
	case MAX_AX_HW_PCIE_MIT:
		ret = p_ops->trx_mit_pcie(adapter,
					  (struct mac_ax_pcie_trx_mitigation *)val);
		break;
	case MAX_AX_HW_PCIE_L2_LEAVE:
		ret = set_pcie_l2_leave(adapter, *(u8 *)val);
		break;
	case MAC_AX_HW_SET_CCTL_PRELD:
		set_cctl_preld(adapter, (struct mac_ax_cctl_preld_cfg *)val);
		break;
	case MAC_AX_HW_PCIE_DRIVING_MPONLY:
		set_pcie_driving_mponly(adapter, *(enum mac_ax_pcie_driving_ctrl *)val);
		break;
	case MAC_AX_HW_SET_PCIE_WPADDR_SEL:
		ret = pcie_set_wp_addr_sel(adapter, (struct mac_ax_pcie_wpaddr_sel *)val);
		break;
	case MAC_AX_HW_SET_PCIE_ADDR_H2:
		ret = pcie_set_addr_h2(adapter, (struct mac_ax_pcie_addr_h2 *)val);
		break;
	case MAC_AX_HW_PCIE_ASPM_FRONTDOOR_SET:
		ret = p_ops->pcie_aspm_frontdoor_set(adapter);
		break;
#endif
#if MAC_AX_USB_SUPPORT
	case MAC_AX_HW_SET_USB_UPHY_PLL_CFG:
		usb_uphy_pll_cfg(adapter, *(u8 *)val);
		break;
#endif
	default:
		return MACNOITEM;
	}

	return ret;
}

static u32 get_append_fcs(struct mac_ax_adapter *adapter, u8 *enable)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	*enable = MAC_REG_R8(R_AX_MPDU_PROC) & B_AX_APPEND_FCS ? 1 : 0;

	return MACSUCCESS;
}

static u32 get_accept_icverr(struct mac_ax_adapter *adapter, u8 *enable)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	*enable = MAC_REG_R8(R_AX_MPDU_PROC) & B_AX_A_ICV_ERR ? 1 : 0;

	return MACSUCCESS;
}

u32 get_pwr_state(struct mac_ax_adapter *adapter, enum mac_ax_mac_pwr_st *st)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val;

	val = GET_FIELD(MAC_REG_R32(R_AX_IC_PWR_STATE), B_AX_WLMAC_PWR_STE);

	if (val == MAC_AX_MAC_OFF) {
		*st = MAC_AX_MAC_OFF;
		adapter->mac_pwr_info.pwr_in_lps = 0;
		adapter->sm.fw_rst = MAC_AX_FW_RESET_IDLE;
		adapter->sm.pwr = MAC_AX_PWR_OFF;
		adapter->sm.mac_rdy = MAC_AX_MAC_NOT_RDY;
		PLTFM_MSG_WARN("WL MAC is in off state.\n");
	} else if (val == MAC_AX_MAC_ON) {
		*st = MAC_AX_MAC_ON;
	} else if (val == MAC_AX_MAC_LPS) {
		*st = MAC_AX_MAC_LPS;
	} else {
		PLTFM_MSG_ERR("Unexpected MAC state = 0x%X\n", val);
		return MACPWRSTAT;
	}

	return MACSUCCESS;
}

void get_dflt_nav(struct mac_ax_adapter *adapter, u16 *nav)
{
	/* data NAV is consist of SIFS and ACK/BA time */
	/* currently, we use SIFS + 64-bitmap BA as default NAV */
	/* we use OFDM-6M to estimate BA time */
	/* BA time = PLCP header(20us) + 32 bytes/data_rate */
	*nav = 63;
}

u32 mac_get_hw_value(struct mac_ax_adapter *adapter,
		     enum mac_ax_hw_id hw_id, void *val)
{
	u32 ret = MACSUCCESS;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	struct mac_ax_priv_ops *p_ops = adapter_to_priv_ops(adapter);

	if (!val) {
		PLTFM_MSG_ERR("[ERR]: the parameter is NULL in %s\n", __func__);
		return MACNPTR;
	}

	switch (hw_id) {
	case MAC_AX_HW_MAPPING:
		break;
	case MAC_AX_HW_GET_EFUSE_SIZE:
		*(u32 *)val = adapter->hw_info->efuse_size +
			      adapter->hw_info->dav_efuse_size;
		break;
	case MAC_AX_HW_GET_LOGICAL_EFUSE_SIZE:
		*(u32 *)val = adapter->hw_info->log_efuse_size +
			      adapter->hw_info->dav_log_efuse_size;
		break;
	case MAC_AX_HW_GET_LIMIT_LOG_EFUSE_SIZE:
		switch (adapter->env_info.intf) {
		case MAC_AX_INTF_PCIE:
			*(u32 *)val = adapter->hw_info->limit_efuse_size_pcie;
			break;
		case MAC_AX_INTF_USB:
			*(u32 *)val = adapter->hw_info->limit_efuse_size_usb;
			break;
		case MAC_AX_INTF_SDIO:
			*(u32 *)val = adapter->hw_info->limit_efuse_size_sdio;
			break;
		default:
			*(u32 *)val = adapter->hw_info->log_efuse_size;
			break;
		}
		*(u32 *)val += adapter->hw_info->dav_log_efuse_size;
		break;
	case MAC_AX_HW_GET_BT_EFUSE_SIZE:
		*(u32 *)val = adapter->hw_info->bt_efuse_size;
		break;
	case MAC_AX_HW_GET_BT_LOGICAL_EFUSE_SIZE:
		*(u32 *)val = adapter->hw_info->bt_log_efuse_size;
		break;
	case MAC_AX_HW_GET_EFUSE_MASK_SIZE:
		*(u32 *)val = (adapter->hw_info->log_efuse_size +
			       adapter->hw_info->dav_log_efuse_size) >> 4;
		break;
	case MAC_AX_HW_GET_LIMIT_EFUSE_MASK_SIZE:
		switch (adapter->env_info.intf) {
		case MAC_AX_INTF_PCIE:
			*(u32 *)val = adapter->hw_info->limit_efuse_size_pcie;
			break;
		case MAC_AX_INTF_USB:
			*(u32 *)val = adapter->hw_info->limit_efuse_size_usb;
			break;
		case MAC_AX_INTF_SDIO:
			*(u32 *)val = adapter->hw_info->limit_efuse_size_sdio;
			break;
		default:
			*(u32 *)val = adapter->hw_info->log_efuse_size;
			break;
		}
		*(u32 *)val += adapter->hw_info->dav_log_efuse_size;
		*(u32 *)val = *(u32 *)val >> 4;
		break;
	case MAC_AX_HW_GET_BT_EFUSE_MASK_SIZE:
		*(u32 *)val = adapter->hw_info->bt_log_efuse_size >> 4;
		break;
	case MAC_AX_HW_GET_DAV_LOG_EFUSE_SIZE:
		*(u32 *)val = adapter->hw_info->dav_log_efuse_size;
		break;
	case MAC_AX_HW_GET_EFUSE_VERSION_SIZE:
		*(u32 *)val = adapter->hw_info->efuse_version_size;
		break;
	case MAC_AX_HW_GET_ID_PAUSE:
		ret = get_macid_pause(adapter,
				      (struct mac_ax_macid_pause_cfg *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_GET_SCH_TXEN_STATUS:
		ret = get_hw_sch_tx_en(adapter,
				       (struct mac_ax_sch_tx_en_cfg *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_GET_EDCA_PARAM:
		ret = get_hw_edca_param(adapter,
					(struct mac_ax_edca_param *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_GET_TBPPDU_CTRL:
		ret = get_hw_tb_ppdu_ctrl(adapter,
					  (struct mac_ax_tb_ppdu_ctrl *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_GET_DELAYTX_CFG:
		get_delay_tx_cfg(adapter, (struct mac_ax_delay_tx_cfg *)val);
		break;
	case MAC_AX_HW_GET_SS_WMM_TBL:
		ret = get_ss_wmm_tbl(adapter,
				     (struct mac_ax_ss_wmm_tbl_ctrl *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_GET_CH_STAT_CNT:
		ret = get_hw_ch_stat_cnt(adapter,
					 (struct mac_ax_ch_stat_cnt *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_GET_LIFETIME_CFG:
		ret = get_hw_lifetime_cfg(adapter,
					  (struct mac_ax_lifetime_cfg *)val);
		if (ret != MACSUCCESS)
			return ret;
		break;
	case MAC_AX_HW_GET_APP_FCS:
		get_append_fcs(adapter, (u8 *)val);
		break;
	case MAC_AX_HW_GET_RX_ICVERR:
		get_accept_icverr(adapter, (u8 *)val);
		break;
	case MAC_AX_HW_GET_PWR_STATE:
		get_pwr_state(adapter, (enum mac_ax_mac_pwr_st *)val);
		break;
	case MAC_AX_HW_GET_SCOREBOARD:
		*(u32 *)val = MAC_REG_R32(R_AX_SCOREBOARD);
		break;
	case MAC_AX_HW_GET_BACAM_MODE_SEL:
		get_bacam_mode(adapter, (u8 *)val);
		break;
	case MAC_AX_HW_GET_WD_CHECKSUM_CFG:
		ret = get_wd_checksum_cfg(adapter, (struct mac_ax_wd_checksum_cfg *)val);
		break;
#if MAC_AX_SDIO_SUPPORT
	case MAC_AX_HW_SDIO_TX_AGG_SIZE:
		*(u16 *)val = adapter->sdio_info.tx_align_size;
		break;
	case MAC_AX_HW_GET_SDIO_RX_REQ_LEN:
		ret = p_ops->get_sdio_rx_req_len(adapter, (u32 *)val);
		break;
	case MAC_AX_HW_GET_SDIO_LPS_FLG:
		*(u8 *)val = adapter->mac_pwr_info.pwr_in_lps;
		break;
#endif
	case MAC_AX_HW_GET_WAKE_REASON:
		ret = p_ops->get_wake_reason(adapter, (u8 *)val);
		if (ret != 0)
			return ret;
		break;
	case MAC_AX_HW_GET_COEX_GNT:
		ret = p_ops->get_gnt(adapter, (struct mac_ax_coex_gnt *)val);
		break;
	case MAC_AX_HW_GET_COEX_CTRL:
		p_ops->get_ctrl_path(adapter, (u32 *)val);
		break;
	case MAC_AX_HW_GET_TX_CNT:
		ret = mac_get_tx_cnt(adapter, (struct mac_ax_tx_cnt *)val);
		if (ret != 0)
			return ret;
		break;
	case MAC_AX_HW_GET_TSF:
		ret = mac_get_tsf(adapter, (struct mac_ax_port_tsf *)val);
		break;
	case MAC_AX_HW_GET_FREERUN_CNT:
		ret = mac_get_freerun(adapter, (struct mac_ax_freerun *)val);
		break;
	case MAC_AX_HW_GET_MAX_TX_TIME:
		ret = mac_get_max_tx_time(adapter,
					  (struct mac_ax_max_tx_time *)val);
		break;
	case MAC_AX_HW_GET_POLLUTED_CNT:
		mac_get_bt_polt_cnt(adapter, (struct mac_ax_bt_polt_cnt *)val);
		break;
	case MAC_AX_HW_GET_DATA_RTY_LMT:
		get_data_rty_limit(adapter, (struct mac_ax_rty_lmt *)val);
		break;
	case MAC_AX_HW_GET_DFLT_NAV:
		get_dflt_nav(adapter, (u16 *)val);
		break;
	case MAC_AX_HW_GET_FW_CAP:
		ret = mac_get_fw_cap(adapter, (u32 *)val);
		break;
	case MAC_AX_HW_GET_RRSR_CFG:
		ret = p_ops->get_rrsr_cfg(adapter,
					  (struct mac_ax_rrsr_cfg *)val);
		break;
	case MAC_AX_HW_GET_CTS_RRSR_CFG:
		ret = p_ops->get_cts_rrsr_cfg(adapter,
					  (struct mac_ax_cts_rrsr_cfg *)val);
		break;
	case MAC_AX_HW_GET_USB_STS:
		ret = ops->get_rx_state(adapter, (u32 *)val);
		break;
	case MAC_AX_HW_GET_WD_PAGE_NUM:
		*(u32 *)val = (u32)adapter->dle_info.hif_min;
		break;
#if MAC_AX_PCIE_SUPPORT
	case MAC_AX_HW_GET_WPADDR_SEL_NUM:
		*(u32 *)val = adapter->pcie_info.wp_addrh_num;
		break;
#endif
	default:
		return MACNOITEM;
	}

	return ret;
}

u32 cfg_mac_bw(struct mac_ax_adapter *adapter, struct mac_ax_cfg_bw *cfg)
{
	u32 value32 = 0;
	u32 reg = 0;
	u8 value8 = 0;
	u8 chk_val8 = 0;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	struct rtw_hal_com_t *hal_com =
		(struct rtw_hal_com_t *)adapter->drv_adapter;

	u8 txsc20 = 0, txsc40 = 0, txsc80 = 0;
#if MAC_USB_IO_ACC_ON
	u32 ret, ofldcap;
	struct mac_ax_ops *mops = adapter_to_mac_ops(adapter);
#endif

#if (MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT)
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
		reg = cfg->band == MAC_AX_BAND_0 ?
		      R_AX_PHYINFO_ERR_IMR : R_AX_PHYINFO_ERR_IMR_C1;
		value32 = MAC_REG_R32(reg);
		value32 = SET_CLR_WORD(value32, 0x7, B_AX_PHYINTF_TIMEOUT_THR);
		MAC_REG_W32(reg, value32);
	}
#endif
#if (MAC_AX_8852C_SUPPORT || MAC_AX_8852D_SUPPORT)
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
		reg = cfg->band == MAC_AX_BAND_0 ?
		      R_AX_PHYINFO_ERR_IMR_V1 : R_AX_PHYINFO_ERR_IMR_V1_C1;
		value32 = MAC_REG_R32(reg);
		value32 = SET_CLR_WORD(value32, 0x7, B_AX_PHYINTF_TIMEOUT_THR_V1);
		MAC_REG_W32(reg, value32);
	}
#endif

	switch (cfg->cbw) {
	case CHANNEL_WIDTH_160:
		txsc80 = rtw_hal_bb_get_txsc(hal_com, cfg->pri_ch,
					     cfg->central_ch, cfg->cbw,
					     CHANNEL_WIDTH_80);
		fallthrough;
	case CHANNEL_WIDTH_80:
		txsc40 = rtw_hal_bb_get_txsc(hal_com, cfg->pri_ch,
					     cfg->central_ch, cfg->cbw,
					     CHANNEL_WIDTH_40);
		fallthrough;
	case CHANNEL_WIDTH_40:
		txsc20 = rtw_hal_bb_get_txsc(hal_com, cfg->pri_ch,
					     cfg->central_ch, cfg->cbw,
					     CHANNEL_WIDTH_20);
		break;
	case CHANNEL_WIDTH_10:
		value32 = MAC_REG_R32(R_AX_AFE_CTRL1);
		value32 = value32 | B_AX_CMAC_CLK_SEL;
		MAC_REG_W32(R_AX_AFE_CTRL1, value32);
		MAC_REG_W8(R_AX_SLOTTIME_CFG, SLOTTIME_10M);
		MAC_REG_W8(R_AX_RSP_CHK_SIG, ACK_TO_10M);
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			MAC_REG_W8(R_AX_TSF_32K_SEL, US_TIME_10M);
		}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8851E_SUPPORT || MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851E) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			MAC_REG_W8(R_AX_TSF_32K_SEL_V1, US_TIME_10M);
		}
#endif
#if (MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT)
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
		value32 = MAC_REG_R32(reg);
		value32 = SET_CLR_WORD(value32, 0x20, B_AX_PHYINTF_TIMEOUT_THR);
		MAC_REG_W32(reg, value32);
	}
#endif
#if (MAC_AX_8852C_SUPPORT || MAC_AX_8852D_SUPPORT)
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
		value32 = MAC_REG_R32(reg);
		value32 = SET_CLR_WORD(value32, 0x20, B_AX_PHYINTF_TIMEOUT_THR_V1);
		MAC_REG_W32(reg, value32);
	}
#endif
		break;
	case CHANNEL_WIDTH_5:
		value32 = MAC_REG_R32(R_AX_AFE_CTRL1);
		value32 = value32 | B_AX_CMAC_CLK_SEL | B_AX_PLL_DIV_SEL;
		MAC_REG_W32(R_AX_AFE_CTRL1, value32);
		MAC_REG_W8(R_AX_SLOTTIME_CFG, SLOTTIME_5M);
		MAC_REG_W8(R_AX_RSP_CHK_SIG, ACK_TO_5M);
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			MAC_REG_W8(R_AX_TSF_32K_SEL, US_TIME_5M);
		}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8851E_SUPPORT || MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851E) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			MAC_REG_W8(R_AX_TSF_32K_SEL_V1, US_TIME_5M);
		}
#endif
#if (MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT)
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
		value32 = MAC_REG_R32(reg);
		value32 = SET_CLR_WORD(value32, 0x20, B_AX_PHYINTF_TIMEOUT_THR);
		MAC_REG_W32(reg, value32);
	}
#endif
#if (MAC_AX_8852C_SUPPORT || MAC_AX_8852D_SUPPORT)
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
		value32 = MAC_REG_R32(reg);
		value32 = SET_CLR_WORD(value32, 0x20, B_AX_PHYINTF_TIMEOUT_THR_V1);
		MAC_REG_W32(reg, value32);
	}
#endif
		break;
	default:
		break;
	}

#if MAC_USB_IO_ACC_ON
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mops->get_hw_value(adapter, MAC_AX_HW_GET_FW_CAP, &ofldcap);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("Get MAC_AX_HW_GET_FW_CAP fail %d\n", ret);
			return ret;
		}
		if (ofldcap == IO_OFLD_DIS)
			goto normal;
		if (cfg->band) {//BAND1
			value8 = MAC_REG_R8(R_AX_WMAC_RFMOD_C1);
			chk_val8 = MAC_REG_R8(R_AX_TXRATE_CHK_C1);
		} else {//BAND0
			value8 = MAC_REG_R8(R_AX_WMAC_RFMOD);
			chk_val8 = MAC_REG_R8(R_AX_TXRATE_CHK);
		}
		value8 = value8 & (~(BIT(0) | BIT(1)));
		chk_val8 = chk_val8 & (~(BIT(0) | BIT(1)));

		switch (cfg->cbw) {
		case CHANNEL_WIDTH_160:
			value8 = value8 | BIT(1) | BIT(0);
			value32 = txsc20 | (txsc40 << 4) | (txsc80 << 8); //TXSC_160M;
			break;
		case CHANNEL_WIDTH_80:
			value8 = value8 | BIT(1);
			value32 = txsc20 | (txsc40 << 4); //TXSC_80M;
			break;
		case CHANNEL_WIDTH_40:
			value8 = value8 | BIT(0);
			value32 = txsc20; //TXSC_40M;
			break;
		case CHANNEL_WIDTH_20:
			value32 = 0; //TXSC_20M;
			break;
		case CHANNEL_WIDTH_10:
			value32 = 0; //TXSC_20M;
			break;
		case CHANNEL_WIDTH_5:
			value32 = 0; //TXSC_20M;
			break;
		default:
			break;
		}

		if (cfg->pri_ch >= CHANNEL_5G)
			chk_val8 |= B_AX_CHECK_CCK_EN | B_AX_RTS_LIMIT_IN_OFDM6;
		if (cfg->band_type == BAND_ON_24G)
			chk_val8 |= B_AX_BAND_MODE;
		else if (cfg->band_type == BAND_ON_5G)
			chk_val8 |= B_AX_CHECK_CCK_EN | B_AX_RTS_LIMIT_IN_OFDM6;
		else if (cfg->band_type == BAND_ON_6G)
			chk_val8 |= B_AX_CHECK_CCK_EN | B_AX_RTS_LIMIT_IN_OFDM6;
		else
			PLTFM_MSG_ERR("[ERR]band_type = %d\n", cfg->band_type);

		if (cfg->band) {//BAND1
			ret = MAC_REG_W_OFLD(R_AX_WMAC_RFMOD_C1, B_AX_WMAC_RFMOD_MSK,
					     value8, 0);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("%s: config fail\n", __func__);
				return ret;
			}
			ret = MAC_REG_W_OFLD(R_AX_TXRATE_CHK_C1, 0x13, chk_val8,
					     0);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("%s: config fail\n", __func__);
				return ret;
			}
			ret = MAC_REG_W32_OFLD(R_AX_TX_SUB_CARRIER_VALUE_C1,
					       value32, 1);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("%s: config fail\n", __func__);
				return ret;
			}
		} else {
			ret = MAC_REG_W_OFLD(R_AX_WMAC_RFMOD, B_AX_WMAC_RFMOD_MSK,
					     value8, 0);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("%s: config fail\n", __func__);
				return ret;
			}
			ret = MAC_REG_W_OFLD(R_AX_TXRATE_CHK, 0x13, chk_val8, 0);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("%s: config fail\n", __func__);
				return ret;
			}
			ret = MAC_REG_W32_OFLD(R_AX_TX_SUB_CARRIER_VALUE,
					       value32, 1);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("%s: config fail\n", __func__);
				return ret;
			}
		}

	return MACSUCCESS;
	}
normal:
#endif

	if (cfg->band) {//BAND1
		value8 = MAC_REG_R8(R_AX_WMAC_RFMOD_C1);
		chk_val8 = MAC_REG_R8(R_AX_TXRATE_CHK_C1);
	} else {//BAND0
		value8 = MAC_REG_R8(R_AX_WMAC_RFMOD);
		chk_val8 = MAC_REG_R8(R_AX_TXRATE_CHK);
	}
	value8 = value8 & (~(BIT(0) | BIT(1)));
	chk_val8 = chk_val8 & (~(BIT(0) | BIT(1) | BIT(4)));

	switch (cfg->cbw) {
	case CHANNEL_WIDTH_160:
		value8 = value8 | BIT(1) | BIT(0);
		value32 = txsc20 | (txsc40 << 4) | (txsc80 << 8); //TXSC_160M;
		break;
	case CHANNEL_WIDTH_80:
		value8 = value8 | BIT(1);
		value32 = txsc20 | (txsc40 << 4); //TXSC_80M;
		break;
	case CHANNEL_WIDTH_40:
		value8 = value8 | BIT(0);
		value32 = txsc20; //TXSC_40M;
		break;
	case CHANNEL_WIDTH_20:
		value32 = 0; //TXSC_20M;
		break;
	case CHANNEL_WIDTH_10:
		value32 = 0; //TXSC_20M;
		break;
	case CHANNEL_WIDTH_5:
		value32 = 0; //TXSC_20M;
		break;
	default:
		break;
	}

	/*Setting for CCK rate in 5G/6G Channel protection*/
	if (cfg->pri_ch >= CHANNEL_5G) // remove after phl setting band_type
		chk_val8 |= B_AX_CHECK_CCK_EN | B_AX_RTS_LIMIT_IN_OFDM6;

	if (cfg->band_type == BAND_ON_24G)
		chk_val8 |= B_AX_BAND_MODE;
	else if (cfg->band_type == BAND_ON_5G)
		chk_val8 |= B_AX_CHECK_CCK_EN | B_AX_RTS_LIMIT_IN_OFDM6;
	else if (cfg->band_type == BAND_ON_6G)
		chk_val8 |= B_AX_CHECK_CCK_EN | B_AX_RTS_LIMIT_IN_OFDM6;
	else
		PLTFM_MSG_ERR("[ERR]band_type = %d\n", cfg->band_type);

	if (cfg->band) {//BAND1
		MAC_REG_W8(R_AX_WMAC_RFMOD_C1, value8);
		MAC_REG_W8(R_AX_TXRATE_CHK_C1, chk_val8);
		MAC_REG_W32(R_AX_TX_SUB_CARRIER_VALUE_C1, value32);
	} else {
		MAC_REG_W8(R_AX_WMAC_RFMOD, value8);
		MAC_REG_W8(R_AX_TXRATE_CHK, chk_val8);
		MAC_REG_W32(R_AX_TX_SUB_CARRIER_VALUE, value32);
	}

	return MACSUCCESS;
}

u32 mac_write_xtal_si(struct mac_ax_adapter *adapter,
		      u8 offset, u8 val, u8 bitmask)
{
	u32 cnt = 0;
	u32 write_val = 0;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	cnt = XTAL_SI_POLLING_CNT;
	write_val = SET_CLR_WORD(write_val, offset, B_AX_WL_XTAL_SI_ADDR);
	write_val = SET_CLR_WORD(write_val, val, B_AX_WL_XTAL_SI_DATA);
	write_val = SET_CLR_WORD(write_val, bitmask, B_AX_WL_XTAL_SI_BITMASK);
	write_val = SET_CLR_WORD(write_val, XTAL_SI_NORMAL_WRITE,
				 B_AX_WL_XTAL_SI_MODE);
	write_val = (write_val | B_AX_WL_XTAL_SI_CMD_POLL);
	MAC_REG_W32(R_AX_WLAN_XTAL_SI_CTRL, write_val);

	while ((MAC_REG_R32(R_AX_WLAN_XTAL_SI_CTRL) & B_AX_WL_XTAL_SI_CMD_POLL)
						== B_AX_WL_XTAL_SI_CMD_POLL) {
		if (!cnt) {
			PLTFM_MSG_ERR("[ERR]xtal si not ready(W)\n");
			return MACPOLLTO;
		}
		cnt--;
		PLTFM_DELAY_US(XTAL_SI_POLLING_DLY_US);
	}

	return MACSUCCESS;
}

u32 mac_read_xtal_si(struct mac_ax_adapter *adapter,
		     u8 offset, u8 *val)
{
	u32 cnt = 0;
	u32 write_val = 0;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	cnt = XTAL_SI_POLLING_CNT;
	write_val = SET_CLR_WORD(write_val, offset, B_AX_WL_XTAL_SI_ADDR);
	write_val = SET_CLR_WORD(write_val, 0x00, B_AX_WL_XTAL_SI_DATA);
	write_val = SET_CLR_WORD(write_val, 0x00, B_AX_WL_XTAL_SI_BITMASK);
	write_val = SET_CLR_WORD(write_val, XTAL_SI_NORMAL_READ,
				 B_AX_WL_XTAL_SI_MODE);
	write_val = (write_val | B_AX_WL_XTAL_SI_CMD_POLL);
	MAC_REG_W32(R_AX_WLAN_XTAL_SI_CTRL, write_val);

	while ((MAC_REG_R32(R_AX_WLAN_XTAL_SI_CTRL) & B_AX_WL_XTAL_SI_CMD_POLL)
						== B_AX_WL_XTAL_SI_CMD_POLL) {
		if (!cnt) {
			PLTFM_MSG_ERR("[ERR]xtal_si not ready(R)\n");
			return MACPOLLTO;
		}
		cnt--;
		PLTFM_DELAY_US(XTAL_SI_POLLING_DLY_US);
	}

	*val = MAC_REG_R8(R_AX_WLAN_XTAL_SI_CTRL + 1);

	return MACSUCCESS;
}

u32 set_host_rpr(struct mac_ax_adapter *adapter,
		 struct mac_ax_host_rpr_cfg *cfg)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	struct mac_ax_host_rpr_cfg *def_cfg;
	enum mac_ax_host_rpr_mode mode;
	u8 is_poh;
	u32 val32, nval32;
	u32 ret = MACSUCCESS;

	if (adapter->env_info.intf == MAC_AX_INTF_PCIE) {
		ret = is_qta_poh(adapter, adapter->dle_info.qta_mode, &is_poh);
		if (ret) {
			PLTFM_MSG_ERR("is qta poh check fail %d\n", ret);
			return ret;
		}
		def_cfg = is_poh ? &rpr_cfg_poh : &rpr_cfg_stf;
		mode = is_poh ? MAC_AX_RPR_MODE_POH : MAC_AX_RPR_MODE_STF;
	} else {
		def_cfg = &rpr_cfg_stf;
		mode = MAC_AX_RPR_MODE_STF;
	}

	val32 = MAC_REG_R32(R_AX_WDRLS_CFG);
	nval32 = SET_CLR_WORD(val32, mode, B_AX_WDRLS_MODE);
	if (nval32 != val32)
		MAC_REG_W32(R_AX_WDRLS_CFG, nval32);

	val32 = MAC_REG_R32(R_AX_RLSRPT0_CFG0);
	nval32 = val32;
	if ((cfg->txok_en == MAC_AX_FUNC_DEF &&
	     def_cfg->txok_en == MAC_AX_FUNC_EN) ||
	    cfg->txok_en == MAC_AX_FUNC_EN)
		nval32 |= B_WDRLS_FLTR_TXOK;
	else
		nval32 &= ~B_WDRLS_FLTR_TXOK;
	if ((cfg->rty_lmt_en == MAC_AX_FUNC_DEF &&
	     def_cfg->rty_lmt_en == MAC_AX_FUNC_EN) ||
	    cfg->rty_lmt_en == MAC_AX_FUNC_EN)
		nval32 |= B_WDRLS_FLTR_RTYLMT;
	else
		nval32 &= ~B_WDRLS_FLTR_RTYLMT;
	if ((cfg->lft_drop_en == MAC_AX_FUNC_DEF &&
	     def_cfg->lft_drop_en == MAC_AX_FUNC_EN) ||
	    cfg->lft_drop_en == MAC_AX_FUNC_EN)
		nval32 |= B_WDRLS_FLTR_LIFTIM;
	else
		nval32 &= ~B_WDRLS_FLTR_LIFTIM;
	if ((cfg->macid_drop_en == MAC_AX_FUNC_DEF &&
	     def_cfg->macid_drop_en == MAC_AX_FUNC_EN) ||
	    cfg->macid_drop_en == MAC_AX_FUNC_EN)
		nval32 |= B_WDRLS_FLTR_MACID;
	else
		nval32 &= ~B_WDRLS_FLTR_MACID;
	if (nval32 != val32)
		MAC_REG_W32(R_AX_RLSRPT0_CFG0, nval32);

	val32 = MAC_REG_R32(R_AX_RLSRPT0_CFG1);
	nval32 = SET_CLR_WORD(val32, (cfg->agg_def ? def_cfg->agg : cfg->agg),
			      B_AX_RLSRPT0_AGGNUM);
	nval32 = SET_CLR_WORD(nval32, (cfg->tmr_def ? def_cfg->tmr : cfg->tmr),
			      B_AX_RLSRPT0_TO);
	if (nval32 != val32)
		MAC_REG_W32(R_AX_RLSRPT0_CFG1, nval32);

	return ret;
}

u32 mac_read_xcap_reg(struct mac_ax_adapter *adapter, u8 sc_xo, u32 *val)
{
#if MAC_AX_8852A_SUPPORT
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A)) {
		struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

		if (sc_xo) {
			*val = (MAC_REG_R32(R_AX_XTAL_ON_CTRL0) >> B_AX_XTAL_SC_XO_SH) &
			      B_AX_XTAL_SC_XO_MSK;
		} else {
			*val = (MAC_REG_R32(R_AX_XTAL_ON_CTRL0) >> B_AX_XTAL_SC_XI_SH) &
			      B_AX_XTAL_SC_XI_MSK;
		}
	}
#endif

	return MACSUCCESS;
}

u32 mac_write_xcap_reg(struct mac_ax_adapter *adapter, u8 sc_xo, u32 val)
{
#if MAC_AX_8852A_SUPPORT
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A)) {
		struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
		u32 val32;

		if (sc_xo) {
			val32 = MAC_REG_R32(R_AX_XTAL_ON_CTRL0);
			val32 &= ~(0xFE0000);
			val32 |= ((val & B_AX_XTAL_SC_XO_MSK) << B_AX_XTAL_SC_XO_SH);
			MAC_REG_W32(R_AX_XTAL_ON_CTRL0, val32);
		} else {
			val32 = MAC_REG_R32(R_AX_XTAL_ON_CTRL0);
			val32 &= ~(0x1FC00);
			val32 = val32 | ((val & B_AX_XTAL_SC_XI_MSK) <<
					 B_AX_XTAL_SC_XI_SH);
			MAC_REG_W32(R_AX_XTAL_ON_CTRL0, val32);
		}
	}
#endif

	return MACSUCCESS;
}

u32 mac_read_xcap_reg_dav(struct mac_ax_adapter *adapter, u8 sc_xo, u32 *val)
{
	u8 xtal_si_value;
	u32 ret;

	if (sc_xo) {
		ret = mac_read_xtal_si(adapter, XTAL_SI_XTAL_SC_XO, &xtal_si_value);

		if (ret) {
			PLTFM_MSG_ERR("Read XTAL_SI fail!\n");
			return ret;
		}
		*val = xtal_si_value;
	} else {
		ret = mac_read_xtal_si(adapter, XTAL_SI_XTAL_SC_XI, &xtal_si_value);

		if (ret) {
			PLTFM_MSG_ERR("Read XTAL_SI fail!\n");
			return ret;
		}
		*val = xtal_si_value;
	}

	return MACSUCCESS;
}

u32 mac_write_xcap_reg_dav(struct mac_ax_adapter *adapter, u8 sc_xo, u32 val)
{
	u8 xtal_si_value;
	u32 ret;

	xtal_si_value = (u8)val;
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8851B)) {
#if MAC_AX_8851B_SUPPORT
		struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
		u32 reg_value;

		if (sc_xo) {
			reg_value = MAC_REG_R32(R_AX_XTAL_ON_CTRL3);
			reg_value = SET_CLR_WORD(reg_value, val, B_AX_XTAL_SC_XO_A_BLOCK);
			MAC_REG_W32(R_AX_XTAL_ON_CTRL3, reg_value);
		} else {
			reg_value = MAC_REG_R32(R_AX_XTAL_ON_CTRL3);
			reg_value = SET_CLR_WORD(reg_value, val, B_AX_XTAL_SC_XI_A_BLOCK);
			MAC_REG_W32(R_AX_XTAL_ON_CTRL3, reg_value);
		}
#endif
	} else {
		if (sc_xo) {
			ret = mac_write_xtal_si(adapter, XTAL_SI_XTAL_SC_XO, xtal_si_value,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}
		} else {
			ret = mac_write_xtal_si(adapter, XTAL_SI_XTAL_SC_XI, xtal_si_value,
						FULL_BIT_MASK);
			if (ret) {
				PLTFM_MSG_ERR("Write XTAL_SI fail!\n");
				return ret;
			}
		}
	}
	return MACSUCCESS;
}

static inline u32 _get_addr_range(struct mac_ax_adapter *adapter, u32 addr)
{
	u32 addr_idx;

#define IOCHKRANG(chip) do { \
		if (ADDR_IS_AON_##chip(addr)) \
			addr_idx = ADDR_AON; \
		else if (ADDR_IS_HCI_##chip(addr)) \
			addr_idx = ADDR_HCI; \
		else if (ADDR_IS_DMAC_##chip(addr)) \
			addr_idx = ADDR_DMAC; \
		else if (ADDR_IS_CMAC0_##chip(addr)) \
			addr_idx = ADDR_CMAC0; \
		else if (ADDR_IS_CMAC1_##chip(addr)) \
			addr_idx = ADDR_CMAC1; \
		else if (ADDR_IS_BB0_##chip(addr)) \
			addr_idx = ADDR_BB0; \
		else if (ADDR_IS_BB1_##chip(addr)) \
			addr_idx = ADDR_BB1; \
		else if (ADDR_IS_RF_##chip(addr)) \
			addr_idx = ADDR_RF; \
		else if (ADDR_IS_IND_ACES_##chip(addr)) \
			addr_idx = ADDR_IND_ACES; \
		else if (ADDR_IS_RSVD_##chip(addr)) \
			addr_idx = ADDR_RSVD; \
		else if (ADDR_IS_PON_##chip(addr)) \
			addr_idx = ADDR_PON; \
		else \
			addr_idx = ADDR_INVALID; \
	} while (0)

	switch (adapter->hw_info->chip_id) {
	case MAC_AX_CHIP_ID_8852A:
		IOCHKRANG(8852A);
		break;
	case MAC_AX_CHIP_ID_8852B:
		IOCHKRANG(8852B);
		break;
	case MAC_AX_CHIP_ID_8852C:
		IOCHKRANG(8852C);
		break;
	case MAC_AX_CHIP_ID_8192XB:
		IOCHKRANG(8192XB);
		break;
	case MAC_AX_CHIP_ID_8851B:
		IOCHKRANG(8851B);
		break;
	case MAC_AX_CHIP_ID_8851E:
		IOCHKRANG(8851E);
		break;
	case MAC_AX_CHIP_ID_8852D:
		IOCHKRANG(8852D);
		break;
	case MAC_AX_CHIP_ID_8852BT:
		IOCHKRANG(8852BT);
		break;
	default:
		addr_idx = ADDR_INVALID;
		break;
	}

#undef IOCHKRANG
	return addr_idx;
}

u32 mac_io_chk_access(struct mac_ax_adapter *adapter, u32 offset)
{
	switch (_get_addr_range(adapter, offset)) {
	case ADDR_AON:
	case ADDR_HCI:
		return MACSUCCESS;
	case ADDR_PON:
		break;
	case ADDR_DMAC:
		if (adapter->sm.dmac_func != MAC_AX_FUNC_ON)
			return MACIOERRDMAC;
		break;
	case ADDR_CMAC0:
		if (adapter->sm.cmac0_func != MAC_AX_FUNC_ON)
			return MACIOERRCMAC0;
		break;
	case ADDR_CMAC1:
		if (adapter->sm.cmac1_func != MAC_AX_FUNC_ON)
			return MACIOERRCMAC1;
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT))
			return MACHWNOSUP;
		break;
	case ADDR_BB0:
#if CHK_BBRF_IO
		if (adapter->sm.bb0_func != MAC_AX_FUNC_ON)
			return MACIOERRBB0;
#endif
		break;
	case ADDR_BB1:
#if CHK_BBRF_IO
		if (adapter->sm.bb1_func != MAC_AX_FUNC_ON)
			return MACIOERRBB1;
#endif
		break;
	case ADDR_RF:
#if CHK_BBRF_IO
		if (adapter->sm.bb0_func != MAC_AX_FUNC_ON &&
		    adapter->sm.bb1_func != MAC_AX_FUNC_ON)
			return MACIOERRRF;
#endif
		break;
	case ADDR_IND_ACES:
		if (adapter->hw_info->is_sec_ic) {
			PLTFM_MSG_ERR("[ERR]security mode ind aces\n");
			return MACIOERRIND;
		}

		if (adapter->dbg_info.ind_aces_cnt > 1)
			PLTFM_MSG_ERR("[ERR]ind aces cnt %d ovf\n",
				      adapter->dbg_info.ind_aces_cnt);
		if (adapter->dbg_info.ind_aces_cnt != 1)
			return MACIOERRIND;
		break;
	case ADDR_RSVD:
		return MACIOERRRSVD;
	case ADDR_INVALID:
		return MACHWNOSUP;
	}

	if (adapter->sm.pwr != MAC_AX_PWR_ON)
		return MACIOERRPWR;

	if (adapter->sm.plat != MAC_AX_PLAT_ON)
		return MACIOERRPLAT;

	if (adapter->sm.io_st == MAC_AX_IO_ST_HANG)
		return MACIOERRISH;

	if ((adapter->sm.fw_rst == MAC_AX_FW_RESET_RECV_DONE ||
	     adapter->sm.fw_rst == MAC_AX_FW_RESET_PROCESS) &&
	    ADDR_NOT_ALLOW_SERL1(offset))
		return MACIOERRSERL1;

	if (adapter->sm.fw_rst == MAC_AX_FW_RESET_IDLE &&
	    adapter->mac_pwr_info.pwr_in_lps && ADDR_NOT_ALLOW_LPS(offset))
		return MACIOERRLPS;

	return MACSUCCESS;
}

u32 mac_get_bt_dis(struct mac_ax_adapter *adapter)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	return !!(MAC_REG_R32(R_AX_WL_BT_PWR_CTRL) & B_AX_BT_DISN_EN);
}

u32 mac_set_bt_dis(struct mac_ax_adapter *adapter, u8 en)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val;

	val = MAC_REG_R32(R_AX_WL_BT_PWR_CTRL);
	val = en ? val | B_AX_BT_DISN_EN : val & ~B_AX_BT_DISN_EN;
	MAC_REG_W32(R_AX_WL_BT_PWR_CTRL, val);

	return MACSUCCESS;
}

u32 mac_watchdog(struct mac_ax_adapter *adapter,
		 struct mac_ax_wdt_param *wdt_param)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	struct mac_ax_drv_wdt_ctrl *ctrl_def = &wdt_ctrl_def;
#if MAC_AX_PCIE_SUPPORT
	struct mac_ax_ops *mac_ops = adapter_to_mac_ops(adapter);
	struct mac_ax_csi_release_cfg cfg;
#endif
	u32 ret = MACSUCCESS, final_ret = MACSUCCESS;

/**
 * NOTICE!!!!!
 * In order to run every feature in the watchdog
 * DO NOT return in the middle of the watchdog function
 * Print error log for the failure feature and record error at final_ret
 */

#if MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
	if ((is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	     is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
	     is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) &&
	    (wdt_param->drv_ctrl.autok_wdt_ctrl == MAC_AX_PCIE_ENABLE ||
	     (wdt_param->drv_ctrl.autok_wdt_ctrl == MAC_AX_PCIE_DEFAULT &&
	      ctrl_def->autok_wdt_ctrl == MAC_AX_PCIE_ENABLE))) {
		ret = ops->pcie_autok_counter_avg(adapter);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("pcie_autok_counter_avg fail %d\n", ret);
			final_ret = ret;
		}
	}

	if ((is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	     is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
	     is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) &&
	    (wdt_param->drv_ctrl.tp_wdt_ctrl == MAC_AX_PCIE_ENABLE ||
	     (wdt_param->drv_ctrl.tp_wdt_ctrl == MAC_AX_PCIE_DEFAULT &&
	      ctrl_def->tp_wdt_ctrl == MAC_AX_PCIE_ENABLE))) {
		ret = ops->tp_adjust(adapter, wdt_param->tp);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("tp_adjust fail %d\n", ret);
			final_ret = ret;
		}
	}
#endif

#if MAC_AX_PCIE_SUPPORT
	if (adapter->env_info.intf == MAC_AX_INTF_PCIE) {
		if (wdt_param->tp.tx_tp > TP_10M || wdt_param->tp.rx_tp > TP_10M) {
			cfg.ctrl = MAC_AX_CSI_KEEP;
			cfg.band_sel = MAC_AX_BAND_0;
			if (check_mac_en(adapter, cfg.band_sel, MAC_AX_CMAC_SEL) == MACSUCCESS) {
				ret = mac_ops->set_hw_value(adapter, MAC_AX_HW_SET_CSI_RELEASE_CFG,
							    &cfg);
				if (ret != MACSUCCESS) {
					PLTFM_MSG_ERR("B%d set csi release cfg %d fail %d\n",
						      cfg.band_sel, cfg.ctrl, ret);
					final_ret = ret;
				}
			}

			cfg.band_sel = MAC_AX_BAND_1;
			if (check_mac_en(adapter, cfg.band_sel, MAC_AX_CMAC_SEL) == MACSUCCESS) {
				ret = mac_ops->set_hw_value(adapter, MAC_AX_HW_SET_CSI_RELEASE_CFG,
							    &cfg);
				if (ret != MACSUCCESS) {
					PLTFM_MSG_ERR("B%d set csi release cfg %d fail %d\n",
						      cfg.band_sel, cfg.ctrl, ret);
					final_ret = ret;
				}
			}
		} else {
			cfg.ctrl = MAC_AX_CSI_RELEASE;
			cfg.band_sel = MAC_AX_BAND_0;
			if (check_mac_en(adapter, cfg.band_sel, MAC_AX_CMAC_SEL) == MACSUCCESS) {
				ret = mac_ops->set_hw_value(adapter, MAC_AX_HW_SET_CSI_RELEASE_CFG,
							    &cfg);
				if (ret != MACSUCCESS) {
					PLTFM_MSG_ERR("B%d set csi release cfg %d fail %d\n",
						      cfg.band_sel, cfg.ctrl, ret);
					final_ret = ret;
				}
			}

			cfg.band_sel = MAC_AX_BAND_1;
			if (check_mac_en(adapter, cfg.band_sel, MAC_AX_CMAC_SEL) == MACSUCCESS) {
				ret = mac_ops->set_hw_value(adapter, MAC_AX_HW_SET_CSI_RELEASE_CFG,
							    &cfg);
				if (ret != MACSUCCESS) {
					PLTFM_MSG_ERR("B%d set csi release cfg %d fail %d\n",
						      cfg.band_sel, cfg.ctrl, ret);
					final_ret = ret;
				}
			}
		}
	}
#endif
	ret = mac_wdt_log(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("mac_wdt_log fail %d\n", ret);
		final_ret = ret;
	}

	return final_ret;
}

u32 mac_get_freerun(struct mac_ax_adapter *adapter,
		    struct mac_ax_freerun *freerun)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 reg_l, reg_h, ret;

	ret = check_mac_en(adapter, freerun->band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	reg_l = freerun->band == MAC_AX_BAND_0 ? R_AX_FREERUN_CNT_LOW : R_AX_FREERUN_CNT_LOW_C1;
	reg_h = freerun->band == MAC_AX_BAND_0 ? R_AX_FREERUN_CNT_HIGH : R_AX_FREERUN_CNT_HIGH_C1;
	freerun->freerun_l = MAC_REG_R32(reg_l);
	freerun->freerun_h = MAC_REG_R32(reg_h);

	return MACSUCCESS;
}

u32 mac_reset_freerun(struct mac_ax_adapter *adapter, u8 *band)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ret, reg, val;

	ret = check_mac_en(adapter, *band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	reg = (*band == MAC_AX_BAND_0) ? R_AX_MISC_0 : R_AX_MISC_0_C1;
	val = MAC_REG_R32(reg);
	MAC_REG_W32(reg, val | B_AX_RST_FREERUN_P);

	return MACSUCCESS;
}
