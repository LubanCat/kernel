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

#include "trx_desc.h"

u32 set_wd_checksum_cfg(struct mac_ax_adapter *adapter,
			struct mac_ax_wd_checksum_cfg *config)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val;

	adapter->hw_info->wd_checksum_en = config->sw_fill_wd_checksum_en ? 1 : 0;
	val = MAC_REG_R32(R_AX_WD_CHECKSUM_FUNCTION_ENABLE);
	if (config->host_check_en)
		val |= B_AX_HDT_WD_CHKSUM_EN;
	else
		val &= ~(u32)B_AX_HDT_WD_CHKSUM_EN;
	if (config->cpu_check_en) {
		PLTFM_MSG_ERR("%s: CPU WD CHKSUM is not supported\n", __func__);
		return MACNOTSUP;
	}
	MAC_REG_W32(R_AX_WD_CHECKSUM_FUNCTION_ENABLE, val);

	return MACSUCCESS;
}

u32 get_wd_checksum_cfg(struct mac_ax_adapter *adapter,
			struct mac_ax_wd_checksum_cfg *config)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val;

	config->sw_fill_wd_checksum_en = adapter->hw_info->wd_checksum_en ? 1 : 0;
	val = MAC_REG_R32(R_AX_WD_CHECKSUM_FUNCTION_ENABLE);
	config->host_check_en = (val & B_AX_HDT_WD_CHKSUM_EN) ? 1 : 0;
	config->cpu_check_en = (val & B_AX_CDT_WD_CHKSUM_EN) ? 1 : 0;

	return MACSUCCESS;
}

u32 get_hdr_with_llc(struct mac_ax_adapter *adapter,
		     struct rtw_t_meta_data *info, u8 *headerwllc)
{
	u8 with_llc;
	u32 ret;

	*headerwllc = 0;
	if (info->msdu_type == RTW_PHL_MSDU_TYPE_80211) {
		*headerwllc = info->mac_hdr_len;
		ret = mac_read_with_llc(adapter, info->macid, &with_llc);
		if (ret != MACSUCCESS)
			return ret;
		if (with_llc)
			*headerwllc += 8;
		if (info->with_vlantag)
			*headerwllc += 4;
		*headerwllc += info->sec_hdr_len;
	} else if (info->msdu_type == RTW_PHL_MSDU_TYPE_ETHII) {
		*headerwllc += 14;
		if (info->with_vlantag)
			*headerwllc += 4;
	} else if (info->msdu_type == RTW_PHL_MSDU_TYPE_8023SNAP) {
		*headerwllc += 22;
		if (info->with_vlantag)
			*headerwllc += 4;
	}

	*headerwllc /= 2;

	return MACSUCCESS;
}

u32 get_hw_hdr_conv(struct mac_ax_adapter *adapter,
		    struct rtw_t_meta_data *info, u8 *smh_en, u8 *upd_wlan_hdr)
{
	*smh_en = 0;
	*upd_wlan_hdr = 0;
	if (info->hw_hdr_conv) {
		if (info->msdu_type == RTW_PHL_MSDU_TYPE_80211)
			*upd_wlan_hdr = 1;
		else if (info->msdu_type == RTW_PHL_MSDU_TYPE_ETHII ||
			 info->msdu_type == RTW_PHL_MSDU_TYPE_8023SNAP)
			*smh_en = 1;
	}

	return MACSUCCESS;
}
