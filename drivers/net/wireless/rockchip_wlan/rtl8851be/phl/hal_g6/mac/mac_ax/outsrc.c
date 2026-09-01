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
#include "outsrc.h"

u32 mac_write_pwr_ofst_mode(struct mac_ax_adapter  *adapter,
			    u8 band, struct rtw_tpu_info *tpu)
{
#if MAC_USB_IO_ACC_ON
	struct mac_ax_ops *mac_ops = adapter_to_mac_ops(adapter);
	u32 ofldcap = 0;
#endif
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 cr = (band == HW_BAND_0) ? R_AX_PWR_RATE_OFST_CTRL :
		 R_AX_PWR_RATE_OFST_CTRL_C1;
	u32 val32 = 0, ret = 0;
	s8 *tmp = &tpu->pwr_ofst_mode[0];

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret  != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s CMAC%d not enable\n", __func__, band);
		return ret;
	}
#if MAC_USB_IO_ACC_ON
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_ops->get_hw_value(adapter, MAC_AX_HW_GET_FW_CAP, &ofldcap);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("Get MAC_HW_GET_FW_CAP fail %d\n", ret);
			return ret;
		}
		if (ofldcap) {
			val32 |= NIB_2_DW(0, 0, 0, tmp[4], tmp[3], tmp[2], tmp[1], tmp[0]);
			ret = MAC_REG_W_OFLD((u16)cr, 0xFFFFF, val32, 1);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("%s: config fail\n", __func__);
				return ret;
			}
		} else {
			val32 = MAC_REG_R32(cr) & ~0xFFFFF;
			val32 |= NIB_2_DW(0, 0, 0, tmp[4], tmp[3], tmp[2], tmp[1], tmp[0]);
			MAC_REG_W32(cr, val32);
		}
	}
#else
	val32 = MAC_REG_R32(cr) & ~0xFFFFF;
	val32 |= NIB_2_DW(0, 0, 0, tmp[4], tmp[3], tmp[2], tmp[1], tmp[0]);
	MAC_REG_W32(cr, val32);
#endif
	return MACSUCCESS;
}

u32 mac_write_pwr_ofst_bw(struct mac_ax_adapter  *adapter,
			  u8 band, struct rtw_tpu_info *tpu)
{
#if MAC_USB_IO_ACC_ON
	struct mac_ax_ops *mac_ops = adapter_to_mac_ops(adapter);
	u32 ofldcap = 0;
#endif
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 cr = (band == HW_BAND_0) ? R_AX_PWR_LMT_CTRL :
		 R_AX_PWR_LMT_CTRL_C1;
	u32 val32 = 0, ret = 0;
	s8 *tmp = &tpu->pwr_ofst_bw[0];

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s CMAC%d not enable\n", __func__, band);
		return ret;
	}
#if MAC_USB_IO_ACC_ON
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_ops->get_hw_value(adapter, MAC_AX_HW_GET_FW_CAP, &ofldcap);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("Get MAC_HW_GET_FW_CAP fail %d\n", ret);
			return ret;
		}
		if (ofldcap) {
			val32 |= NIB_2_DW(0, 0, 0, tmp[4], tmp[3], tmp[2], tmp[1], tmp[0]);
			ret = MAC_REG_W_OFLD((u16)cr, 0xFFFFF, val32, 1);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("%s: config fail\n", __func__);
				return ret;
			}
		} else {
			val32 = MAC_REG_R32(cr) & ~0xFFFFF;
			val32 |= NIB_2_DW(0, 0, 0, tmp[4], tmp[3], tmp[2], tmp[1], tmp[0]);
			MAC_REG_W32(cr, val32);
		}
	}
#else
	val32 = MAC_REG_R32(cr) & ~0xFFFFF;
	val32 |= NIB_2_DW(0, 0, 0, tmp[4], tmp[3], tmp[2], tmp[1], tmp[0]);
	MAC_REG_W32(cr, val32);
#endif
	return MACSUCCESS;
}

u32 mac_write_pwr_ref_reg(struct mac_ax_adapter  *adapter,
			  u8 band, struct rtw_tpu_info *tpu)
{
#if MAC_USB_IO_ACC_ON
	struct mac_ax_ops *mac_ops = adapter_to_mac_ops(adapter);
	u32 ofldcap = 0;
#endif
	u32 cr = (band == HW_BAND_0) ? R_AX_PWR_RATE_CTRL :
		 R_AX_PWR_RATE_CTRL_C1;
	u32 val32 = 0, ret = 0;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s CMAC%d not enable\n", __func__, band);
		return ret;
	}
#if MAC_USB_IO_ACC_ON
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_ops->get_hw_value(adapter, MAC_AX_HW_GET_FW_CAP, &ofldcap);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("Get MAC_HW_GET_FW_CAP fail %d\n", ret);
			return ret;
		}
		if (ofldcap) {
			ops = NULL;
			val32 |= (((tpu->ref_pow_ofdm & 0x1ff) << 9) |
				  ((tpu->ref_pow_cck & 0x1ff)));
			ret = MAC_REG_W_OFLD((u16)cr, 0xFFFFC00, val32, 1);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("%s: config fail\n", __func__);
				return ret;
			}
		} else {
			val32 = MAC_REG_R32(cr) & ~0xFFFFC00;
			val32 |= (((tpu->ref_pow_ofdm & 0x1ff) << 19) |
					  ((tpu->ref_pow_cck & 0x1ff) << 10));
			MAC_REG_W32(cr, val32);
		}

		return MACSUCCESS;
	}
#endif

	val32 = MAC_REG_R32(cr) & ~0xFFFFC00;
	val32 |= (((tpu->ref_pow_ofdm & 0x1ff) << 19) |
		  ((tpu->ref_pow_cck & 0x1ff) << 10));
	MAC_REG_W32(cr, val32);

	return MACSUCCESS;
}

u32 mac_write_pwr_limit_en(struct mac_ax_adapter  *adapter,
			   u8 band, struct rtw_tpu_info *tpu)
{
#if MAC_USB_IO_ACC_ON
	struct mac_ax_ops *mac_ops = adapter_to_mac_ops(adapter);
	u32 ofldcap = 0;
#endif
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 cr = (band == HW_BAND_0) ? R_AX_PWR_LMT_CTRL :
		 R_AX_PWR_LMT_CTRL_C1;
	u32 val32 = 0, ret = 0;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s CMAC%d not enable\n", __func__, band);
		return ret;
	}
#if MAC_USB_IO_ACC_ON
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_ops->get_hw_value(adapter, MAC_AX_HW_GET_FW_CAP, &ofldcap);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("Get MAC_HW_GET_FW_CAP fail %d\n", ret);
			return ret;
		}
		if (ofldcap) {
			if (tpu->pwr_lmt_en)
				val32 =  3;
			ret = MAC_REG_W_OFLD((u16)cr, 0x300000, val32, 0);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("%s: config fail\n", __func__);
				return ret;
			}
			val32 = 0;
			cr = (band == HW_BAND_0) ? R_AX_PWR_RU_LMT_CTRL : R_AX_PWR_RU_LMT_CTRL_C1;
			if (tpu->pwr_lmt_en)
				val32 =  1;
			ret = MAC_REG_W_OFLD((u16)cr, PWR_RU_LMT_CTRL_VAL, val32, 1);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("%s: config fail\n", __func__);
				return ret;
			}
		} else {
			val32 = MAC_REG_R32(cr) & ~0x300000;
			if (tpu->pwr_lmt_en)
				val32 |=  0x300000;
			MAC_REG_W32(cr, val32);
			cr = (band == HW_BAND_0) ? R_AX_PWR_RU_LMT_CTRL : R_AX_PWR_RU_LMT_CTRL_C1;
			val32 = MAC_REG_R32(cr) & ~BIT18;
			if (tpu->pwr_lmt_en)
				val32 |=  BIT18;
			MAC_REG_W32(cr, val32);
		}
	}
#else
	val32 = MAC_REG_R32(cr) & ~0x300000;
	if (tpu->pwr_lmt_en)
		val32 |=  0x300000;
	MAC_REG_W32(cr, val32);
	cr = (band == HW_BAND_0) ? R_AX_PWR_RU_LMT_CTRL : R_AX_PWR_RU_LMT_CTRL_C1;
		val32 = MAC_REG_R32(cr) & ~BIT18;
	if (tpu->pwr_lmt_en)
		val32 |=  BIT18;
	MAC_REG_W32(cr, val32);
#endif

	return MACSUCCESS;
}

u32 mac_read_pwr_reg(struct mac_ax_adapter  *adapter, u8 band,
		     const u32 offset, u32 *val)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ret;
	u32 access_offset = offset;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret  != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s CMAC%d not enable\n", __func__, band);
		return ret;
	}
	if (offset < R_AX_PWR_RATE_CTRL || offset > 0xFFFF) {
		PLTFM_MSG_ERR("[ERR]offset exceed pwr ctrl reg %x\n", offset);
		return MACBADDR;
	}
	if (band == MAC_AX_BAND_1)
		access_offset = offset | BIT13;
	ret = mac_check_access(adapter, access_offset);
	if (ret)
		return ret;
	*val = MAC_REG_R32(access_offset);

	return MACSUCCESS;
}

u32 mac_write_msk_pwr_reg(struct mac_ax_adapter  *adapter, u8 band,
			  const u32 offset, u32 mask, u32 val)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ret = MACSUCCESS;
	u32 access_offset = offset;
	u32 ori_val = 0;
	u8 shift;
#if MAC_USB_IO_ACC_ON
	struct mac_ax_ops *mac_ops = adapter_to_mac_ops(adapter);
	u32 ofldcap = 0;
#endif

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret  != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s CMAC%d not enable\n", __func__, band);
		return ret;
	}
	if (offset < R_AX_PWR_RATE_CTRL || offset > 0xFFFF) {
		PLTFM_MSG_ERR("[ERR]offset exceed pwr ctrl reg %x\n", offset);
		return MACBADDR;
	}
	if (band == MAC_AX_BAND_1)
		access_offset = offset | BIT13;
	ret = mac_check_access(adapter, access_offset);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]check access in %x\n", access_offset);
		return ret;
	}
#if MAC_USB_IO_ACC_ON
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_ops->get_hw_value(adapter, MAC_AX_HW_GET_FW_CAP, &ofldcap);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("Get MAC_HW_GET_FW_CAP fail %d\n", ret);
			return ret;
		}
		if (ofldcap) {
			ret = MAC_REG_W_OFLD((u16)access_offset, mask, val, 0);
			if (ret) {
				PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n", __func__,
					      access_offset);
				return ret;
			}
	
			return MACSUCCESS;
		}
	}
#endif
	if (mask != 0xffffffff) {
		shift = shift_mask(mask);
		ori_val = MAC_REG_R32(access_offset);
		val = ((ori_val) & (~mask)) | (((val << shift)) & mask);
	}
	MAC_REG_W32(access_offset, val);

	return MACSUCCESS;
}

u32 mac_write_pwr_reg(struct mac_ax_adapter  *adapter, u8 band,
		      const u32 offset, u32 val)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ret = MACSUCCESS;
	u32 access_offset = offset;
#if MAC_USB_IO_ACC_ON
	struct mac_ax_ops *mac_ops = adapter_to_mac_ops(adapter);
	u32 ofldcap = 0;
#endif

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret  != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s CMAC%d not enable\n", __func__, band);
		return ret;
	}
	if (offset < R_AX_PWR_RATE_CTRL || offset > 0xFFFF) {
		PLTFM_MSG_ERR("[ERR]offset exceed pwr ctrl reg %x\n", offset);
		return MACBADDR;
	}
	if (band == MAC_AX_BAND_1)
		access_offset = offset | BIT13;
	ret = mac_check_access(adapter, access_offset);
	if (ret)
		return ret;
#if MAC_USB_IO_ACC_ON
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_ops->get_hw_value(adapter, MAC_AX_HW_GET_FW_CAP, &ofldcap);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("Get MAC_HW_GET_FW_CAP fail %d\n", ret);
			return ret;
		}
		if (ofldcap) {
			ret = MAC_REG_W32_OFLD((u16)access_offset, val, 0);
			if (ret) {
				PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n", __func__,
					      access_offset);
				return ret;
			}
			return MACSUCCESS;
		}
	}
#endif
	MAC_REG_W32(access_offset, val);

	return MACSUCCESS;
}

u32 mac_write_msk_txpwr_reg(struct mac_ax_adapter  *adapter, u8 band,
			    const u32 offset, u32 mask, u32 val, u8 last_cmd)
{
	u32 ret = MACSUCCESS;
	u32 access_offset = offset;
#if MAC_USB_IO_ACC_ON
	struct mac_ax_ops *mac_ops = adapter_to_mac_ops(adapter);
	u32 ofldcap = 0;
#endif

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret  != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s CMAC%d not enable\n", __func__, band);
		return ret;
	}
	if (offset < R_AX_PWR_RATE_CTRL || offset > 0xFFFF) {
		PLTFM_MSG_ERR("[ERR]offset exceed pwr ctrl reg %x\n", offset);
		return MACBADDR;
	}
	if (band == MAC_AX_BAND_1)
		access_offset = offset | BIT13;
	ret = mac_check_access(adapter, access_offset);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]check access in %x\n", access_offset);
		return ret;
	}

#if MAC_USB_IO_ACC_ON
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_ops->get_hw_value(adapter, MAC_AX_HW_GET_FW_CAP, &ofldcap);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("Get MAC_HW_GET_FW_CAP fail %d\n", ret);
			return ret;
		}
		if (ofldcap) {
			ret = MAC_REG_W_OFLD((u16)access_offset, mask, val, last_cmd);
			if (ret) {
				PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n", __func__,
					      access_offset);
				return ret;
			}
		}

		return MACSUCCESS;
	}
#endif

	return MACNOFW;
}

u32 mac_write_txpwr_reg(struct mac_ax_adapter  *adapter, u8 band,
			const u32 offset, u32 val, u8 last_cmd)
{
	u32 ret = MACSUCCESS;
	u32 access_offset = offset;
#if MAC_USB_IO_ACC_ON
	struct mac_ax_ops *mac_ops = adapter_to_mac_ops(adapter);
	u32 ofldcap = 0;
#endif

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret  != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s CMAC%d not enable\n", __func__, band);
		return ret;
	}
	if (offset < R_AX_PWR_RATE_CTRL  || offset > 0xFFFF) {
		PLTFM_MSG_ERR("[ERR]offset exceed pwr ctrl reg %x\n", offset);
		return MACBADDR;
	}
	if (band == MAC_AX_BAND_1)
		access_offset = offset | BIT13;
	ret = mac_check_access(adapter, access_offset);
	if (ret)
		return ret;

#if MAC_USB_IO_ACC_ON
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_ops->get_hw_value(adapter, MAC_AX_HW_GET_FW_CAP, &ofldcap);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("Get MAC_HW_GET_FW_CAP fail %d\n", ret);
			return ret;
		}
		if (ofldcap) {
			ret = MAC_REG_W32_OFLD((u16)access_offset, val, last_cmd);
			if (ret) {
				PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n", __func__,
					      access_offset);
				return ret;
			}
		}
		return MACSUCCESS;
	}
#endif

	return MACNOFW;
}

u32 mac_write_pwr_limit_rua_reg(struct mac_ax_adapter  *adapter,
				u8 band, struct rtw_tpu_info *tpu)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ret;
	u16 cr = (band == HW_BAND_0) ? R_AX_PWR_RU_LMT_TABLE0 :
		 R_AX_PWR_RU_LMT_TABLE0_C1;
	s8 *tmp;
	u8 i, j;
#if MAC_USB_IO_ACC_ON
	struct mac_ax_ops *mac_ops = adapter_to_mac_ops(adapter);
	u32 ofldcap = 0;
#endif
	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret  != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s CMAC%d not enable\n", __func__, band);
		return ret;
	}
#if MAC_USB_IO_ACC_ON
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_ops->get_hw_value(adapter, MAC_AX_HW_GET_FW_CAP, &ofldcap);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("Get MAC_HW_GET_FW_CAP fail %d\n", ret);
			return ret;
		}
		if (ofldcap) {
			for (i = 0; i < HAL_MAX_PATH; i++) {
				/*RU 26*/
				tmp = &tpu->pwr_lmt_ru[i][0][0];
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp[3], tmp[2],
									tmp[1], tmp[0]),
									0);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
				cr += 4;
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp[7], tmp[6],
									tmp[5], tmp[4]),
									0);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
				cr += 4;
				/*RU 52*/
				tmp = &tpu->pwr_lmt_ru[i][1][0];
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp[3], tmp[2],
									tmp[1], tmp[0]),
									0);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
				cr += 4;
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp[7], tmp[6],
									tmp[5], tmp[4]),
									0);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
				cr += 4;
				/*RU 106*/
				tmp = &tpu->pwr_lmt_ru[i][2][0];
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp[3], tmp[2],
									tmp[1], tmp[0]),
									0);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
				cr += 4;
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp[7], tmp[6],
									tmp[5], tmp[4]),
									1);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
				cr += 4;
			}
			return MACSUCCESS;
		} else {
			for (i = 0; i < HAL_MAX_PATH; i++) {
				for (j = 0; j < TPU_SIZE_RUA; j++) {
					tmp = &tpu->pwr_lmt_ru[i][j][0];
					MAC_REG_W32(cr, BT_2_DW(tmp[3], tmp[2], tmp[1], tmp[0]));
					cr += 4;
					MAC_REG_W32(cr, BT_2_DW(tmp[7], tmp[6], tmp[5], tmp[4]));
					cr += 4;
				}
			}
			return MACSUCCESS;
		}
	}
#endif
	for (i = 0; i < HAL_MAX_PATH; i++) {
		for (j = 0; j < TPU_SIZE_RUA; j++) {
			tmp = &tpu->pwr_lmt_ru[i][j][0];
			MAC_REG_W32(cr, BT_2_DW(tmp[3], tmp[2], tmp[1], tmp[0]));
			cr += 4;
			MAC_REG_W32(cr, BT_2_DW(tmp[7], tmp[6], tmp[5], tmp[4]));
			cr += 4;
		}
	}
	return MACSUCCESS;
}

u32 mac_write_pwr_limit_reg(struct mac_ax_adapter  *adapter,
			    u8 band, struct rtw_tpu_pwr_imt_info *tpu)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 base = (band == HW_BAND_0) ? R_AX_PWR_RATE_CTRL :
		   R_AX_PWR_RATE_CTRL_C1;
	u32 ss_ofst = 0;
	u32 ret;
	u16 cr = 0;
	s8 *tmp, *tmp_1;
	u8 i, j;
#if MAC_USB_IO_ACC_ON
	struct mac_ax_ops *mac_ops = adapter_to_mac_ops(adapter);
	u32 ofldcap = 0;
#endif
	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret  != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s CMAC%d not enable\n", __func__, band);
		return ret;
	}
#if MAC_USB_IO_ACC_ON
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_ops->get_hw_value(adapter, MAC_AX_HW_GET_FW_CAP, &ofldcap);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("Get MAC_HW_GET_FW_CAP fail %d\n", ret);
			return ret;
		}
		if (ofldcap) {
			for (i = 0; i < HAL_MAX_PATH; i++) {
				tmp = &tpu->pwr_lmt_cck_20m[i][0];
				tmp_1 = &tpu->pwr_lmt_cck_40m[i][0];
				cr = (base | PWR_LMT_CCK_OFFSET) + ss_ofst;
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp_1[1], tmp_1[0],
									tmp[1], tmp[0]), 0);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
				tmp = &tpu->pwr_lmt_lgcy_20m[i][0];
				tmp_1 = &tpu->pwr_lmt_20m[i][0][0];
				cr = (base | PWR_LMT_LGCY_OFFSET) + ss_ofst;
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp_1[1], tmp_1[0],
									tmp[1], tmp[0]), 0);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
				cr = (base | PWR_LMT_TBL2_OFFSET) + ss_ofst;
				for (j = 1; j <= 5; j += 2) {
					tmp = &tpu->pwr_lmt_20m[i][j][0];
					tmp_1 = &tpu->pwr_lmt_20m[i][j + 1][0];
					ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp_1[1], tmp_1[0],
										tmp[1], tmp[0]), 0);
					if (ret) {
						PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
							      __func__, cr);
						return ret;
					}
					cr += 4;
				}
				tmp = &tpu->pwr_lmt_20m[i][7][0];
				tmp_1 = &tpu->pwr_lmt_40m[i][0][0];
				cr = (base | PWR_LMT_TBL5_OFFSET) + ss_ofst;
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp_1[1], tmp_1[0],
									tmp[1], tmp[0]), 0);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
				tmp = &tpu->pwr_lmt_40m[i][1][0];
				tmp_1 = &tpu->pwr_lmt_40m[i][2][0];
				cr = (base | PWR_LMT_TBL6_OFFSET) + ss_ofst;
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp_1[1], tmp_1[0],
									tmp[1], tmp[0]), 0);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
				tmp = &tpu->pwr_lmt_40m[i][3][0];
				tmp_1 = &tpu->pwr_lmt_80m[i][0][0];
				cr = (base | PWR_LMT_TBL7_OFFSET) + ss_ofst;
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp_1[1], tmp_1[0],
									tmp[1], tmp[0]), 0);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
				tmp = &tpu->pwr_lmt_80m[i][1][0];
				tmp_1 = &tpu->pwr_lmt_160m[i][0];
				cr = (base | PWR_LMT_TBL8_OFFSET) + ss_ofst;
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp_1[1], tmp_1[0],
									tmp[1], tmp[0]), 0);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
				tmp = &tpu->pwr_lmt_40m_0p5[i][0];
				tmp_1 = &tpu->pwr_lmt_40m_2p5[i][0];
				cr = (base | PWR_LMT_TBL9_OFFSET) + ss_ofst;
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp_1[1], tmp_1[0],
									tmp[1], tmp[0]), 1);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
				ss_ofst += PWR_LMT_TBL_UNIT;
			}
			return MACSUCCESS;
		} else {
			for (i = 0; i < HAL_MAX_PATH; i++) {
				tmp = &tpu->pwr_lmt_cck_20m[i][0];
				tmp_1 = &tpu->pwr_lmt_cck_40m[i][0];
				cr = (base | PWR_LMT_CCK_OFFSET) + ss_ofst;
				MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
				tmp = &tpu->pwr_lmt_lgcy_20m[i][0];
				tmp_1 = &tpu->pwr_lmt_20m[i][0][0];
				cr = (base | PWR_LMT_LGCY_OFFSET) + ss_ofst;
				MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
				cr = (base | PWR_LMT_TBL2_OFFSET) + ss_ofst;
				for (j = 1; j <= 5; j += 2) {
					tmp = &tpu->pwr_lmt_20m[i][j][0];
					tmp_1 = &tpu->pwr_lmt_20m[i][j + 1][0];
					MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1],
								tmp[0]));
					cr += 4;
				}
				tmp = &tpu->pwr_lmt_20m[i][7][0];
				tmp_1 = &tpu->pwr_lmt_40m[i][0][0];
				cr = (base | PWR_LMT_TBL5_OFFSET) + ss_ofst;
				MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
				tmp = &tpu->pwr_lmt_40m[i][1][0];
				tmp_1 = &tpu->pwr_lmt_40m[i][2][0];
				cr = (base | PWR_LMT_TBL6_OFFSET) + ss_ofst;
				MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
				tmp = &tpu->pwr_lmt_40m[i][3][0];
				tmp_1 = &tpu->pwr_lmt_80m[i][0][0];
				cr = (base | PWR_LMT_TBL7_OFFSET) + ss_ofst;
				MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
				tmp = &tpu->pwr_lmt_80m[i][1][0];
				tmp_1 = &tpu->pwr_lmt_160m[i][0];
				cr = (base | PWR_LMT_TBL8_OFFSET) + ss_ofst;
				MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
				tmp = &tpu->pwr_lmt_40m_0p5[i][0];
				tmp_1 = &tpu->pwr_lmt_40m_2p5[i][0];
				cr = (base | PWR_LMT_TBL9_OFFSET) + ss_ofst;
				MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
				ss_ofst += PWR_LMT_TBL_UNIT;
			}
			return MACSUCCESS;
		}
	}
#endif
	for (i = 0; i < HAL_MAX_PATH; i++) {
		tmp = &tpu->pwr_lmt_cck_20m[i][0];
		tmp_1 = &tpu->pwr_lmt_cck_40m[i][0];
		cr = (base | PWR_LMT_CCK_OFFSET) + ss_ofst;
		MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
		tmp = &tpu->pwr_lmt_lgcy_20m[i][0];
		tmp_1 = &tpu->pwr_lmt_20m[i][0][0];
		cr = (base | PWR_LMT_LGCY_OFFSET) + ss_ofst;
		MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
		cr = (base | PWR_LMT_TBL2_OFFSET) + ss_ofst;
		for (j = 1; j <= 5; j += 2) {
			tmp = &tpu->pwr_lmt_20m[i][j][0];
			tmp_1 = &tpu->pwr_lmt_20m[i][j + 1][0];
			MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1],
						tmp[0]));
			cr += 4;
		}
		tmp = &tpu->pwr_lmt_20m[i][7][0];
		tmp_1 = &tpu->pwr_lmt_40m[i][0][0];
		cr = (base | PWR_LMT_TBL5_OFFSET) + ss_ofst;
		MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
		tmp = &tpu->pwr_lmt_40m[i][1][0];
		tmp_1 = &tpu->pwr_lmt_40m[i][2][0];
		cr = (base | PWR_LMT_TBL6_OFFSET) + ss_ofst;
		MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
		tmp = &tpu->pwr_lmt_40m[i][3][0];
		tmp_1 = &tpu->pwr_lmt_80m[i][0][0];
		cr = (base | PWR_LMT_TBL7_OFFSET) + ss_ofst;
		MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
		tmp = &tpu->pwr_lmt_80m[i][1][0];
		tmp_1 = &tpu->pwr_lmt_160m[i][0];
		cr = (base | PWR_LMT_TBL8_OFFSET) + ss_ofst;
		MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
		tmp = &tpu->pwr_lmt_40m_0p5[i][0];
		tmp_1 = &tpu->pwr_lmt_40m_2p5[i][0];
		cr = (base | PWR_LMT_TBL9_OFFSET) + ss_ofst;
		MAC_REG_W32(cr, BT_2_DW(tmp_1[1], tmp_1[0], tmp[1], tmp[0]));
		ss_ofst += PWR_LMT_TBL_UNIT;
	}
	return MACSUCCESS;
}

u32 mac_write_pwr_by_rate_reg(struct mac_ax_adapter  *adapter,
			      u8 band, struct rtw_tpu_pwr_by_rate_info *tpu)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 base = (band == HW_BAND_0) ? R_AX_PWR_RATE_CTRL :
		   R_AX_PWR_RATE_CTRL_C1;
	u32 ret;
	u32 ss_ofst = 0;
	u16 cr = 0;
	s8 *tmp;
	u8 i, j;
#if MAC_USB_IO_ACC_ON
	struct mac_ax_ops *mac_ops = adapter_to_mac_ops(adapter);
	u32 ofldcap = 0;
#endif
	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret  != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s CMAC%d not enable\n", __func__, band);
		return ret;
	}
#if MAC_USB_IO_ACC_ON
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_ops->get_hw_value(adapter, MAC_AX_HW_GET_FW_CAP, &ofldcap);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("Get MAC_HW_GET_FW_CAP fail %d\n", ret);
			return ret;
		}
		if (ofldcap) {
			for (i = 0; i <= 8; i += 4) {
				tmp = &tpu->pwr_by_rate_lgcy[i];
				cr = (base | PWR_BY_RATE_LGCY_OFFSET) + i;
				ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp[3], tmp[2],
									tmp[1], tmp[0]), 0);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
						      __func__, cr);
					return ret;
				}
			}
			for (i = 0; i < HAL_MAX_PATH; i++) {
				for (j = 0; j <= 12; j += 4) {
					tmp = &tpu->pwr_by_rate[i][j];
					cr = (base | PWR_BY_RATE_OFFSET) + j + ss_ofst;
					ret = MAC_REG_W32_OFLD((u16)cr, BT_2_DW(tmp[3], tmp[2],
										tmp[1],	tmp[0]), 0);
					if (ret) {
						PLTFM_MSG_ERR("[ERR]%s FW_OFLD in %x\n",
							      __func__, cr);
						return ret;
					}
				}
				ss_ofst += 0x10; /*16*/
			}
			return MACSUCCESS;
		} else {
			for (i = 0; i <= 8; i += 4) {
				tmp = &tpu->pwr_by_rate_lgcy[i];
				cr = (base | PWR_BY_RATE_LGCY_OFFSET) + i;
				MAC_REG_W32(cr, BT_2_DW(tmp[3], tmp[2], tmp[1], tmp[0]));
			}
			for (i = 0; i < HAL_MAX_PATH; i++) {
				for (j = 0; j <= 12; j += 4) {
					tmp = &tpu->pwr_by_rate[i][j];
					cr = (base | PWR_BY_RATE_OFFSET) + j + ss_ofst;
					MAC_REG_W32(cr, BT_2_DW(tmp[3], tmp[2], tmp[1],
								tmp[0]));
				}
				ss_ofst += 0x10; /*16*/
			}
			return MACSUCCESS;
		}
	}
#endif
	for (i = 0; i <= 8; i += 4) {
		tmp = &tpu->pwr_by_rate_lgcy[i];
		cr = (base | PWR_BY_RATE_LGCY_OFFSET) + i;
		MAC_REG_W32(cr, BT_2_DW(tmp[3], tmp[2], tmp[1], tmp[0]));
	}
	for (i = 0; i < HAL_MAX_PATH; i++) {
		for (j = 0; j <= 12; j += 4) {
			tmp = &tpu->pwr_by_rate[i][j];
			cr = (base | PWR_BY_RATE_OFFSET) + j + ss_ofst;
			MAC_REG_W32(cr, BT_2_DW(tmp[3], tmp[2], tmp[1],
						tmp[0]));
		}
		ss_ofst += 0x10; /*16*/
	}
	return MACSUCCESS;
}

u32 mac_write_bbrst_reg(struct mac_ax_adapter  *adapter, u8 val)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u8 val8;

	val8 = MAC_REG_R8(R_AX_SYS_FUNC_EN);
	if (val)
		MAC_REG_W8(R_AX_SYS_FUNC_EN, val8 | B_AX_FEN_BBRSTB);
	else
		MAC_REG_W8(R_AX_SYS_FUNC_EN, val8 & (~B_AX_FEN_BBRSTB));
	return MACSUCCESS;
}
