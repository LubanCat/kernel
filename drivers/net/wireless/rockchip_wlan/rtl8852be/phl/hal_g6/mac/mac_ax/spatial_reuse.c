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
#include "spatial_reuse.h"

u32 mac_sr_update(struct mac_ax_adapter *adapter,
		  struct rtw_mac_ax_sr_info *sr_info,
		  enum mac_ax_band band)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u8 val8;
	u32 val32, ret, reg;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	reg = band == MAC_AX_BAND_1 ? R_AX_RX_SR_CTRL_C1 : R_AX_RX_SR_CTRL;
	val8 = MAC_REG_R8(reg);
	val8 = (sr_info->sr_en) ? (val8 | B_AX_SR_EN) : (val8 & ~B_AX_SR_EN);
	MAC_REG_W8(reg, val8);

	reg = band == MAC_AX_BAND_1 ? R_AX_TCR0_C1 : R_AX_TCR0;
	val8 = MAC_REG_R8(reg);
	val8 = (sr_info->sr_field_v15_allowed) ?
		(val8 | B_AX_TCR_SR_VAL15_ALLOW) :
		(val8 & ~B_AX_TCR_SR_VAL15_ALLOW);
	MAC_REG_W8(reg, val8);

	reg = band == MAC_AX_BAND_1 ? R_AX_SR_OBSS_PD_C1 : R_AX_SR_OBSS_PD;
	val32 = MAC_REG_R32(reg);
	val32 = SET_CLR_WORD(val32, sr_info->non_srg_obss_pd_min,
			     B_AX_NONSRG_OBSS_PD_MIN);
	val32 = SET_CLR_WORD(val32, sr_info->non_srg_obss_pd_max,
			     B_AX_NONSRG_OBSS_PD_MAX);
	val32 = SET_CLR_WORD(val32, sr_info->srg_obss_pd_min,
			     B_AX_SRG_OBSS_PD_MIN);
	val32 = SET_CLR_WORD(val32, sr_info->srg_obss_pd_max,
			     B_AX_SRG_OBSS_PD_MAX);
	MAC_REG_W32(reg, val32);

	reg = band == MAC_AX_BAND_1 ?
	      R_AX_SR_BSSCOLOR_BITMAP_C1 : R_AX_SR_BSSCOLOR_BITMAP;
	MAC_REG_W32(reg, sr_info->srg_bsscolor_bitmap_0);
	MAC_REG_W32(reg + 4, sr_info->srg_bsscolor_bitmap_1);

	reg = band == MAC_AX_BAND_1 ?
	      R_AX_SR_PARTIAL_BSSCOLOR_BITMAP_C1 :
	      R_AX_SR_PARTIAL_BSSCOLOR_BITMAP;
	MAC_REG_W32(reg, sr_info->srg_partbsid_bitmap_0);
	MAC_REG_W32(reg + 4, sr_info->srg_partbsid_bitmap_1);

	return MACSUCCESS;
}

u32 mac_sr_update_with_msk(struct mac_ax_adapter *adapter,
			   struct rtw_mac_ax_sr_info *sr_info_msk,
			   struct rtw_mac_ax_sr_info *sr_info,
			   enum mac_ax_band band)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u8 val8;
	u32 val32, ret, reg = 0;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	if (sr_info_msk->sr_en) {
		reg = band == MAC_AX_BAND_1 ? R_AX_RX_SR_CTRL_C1 : R_AX_RX_SR_CTRL;
		val8 = MAC_REG_R8(reg);
		val8 = (sr_info->sr_en) ? (val8 | B_AX_SR_EN) : (val8 & ~B_AX_SR_EN);
		MAC_REG_W8(reg, val8);
	}

	if (sr_info_msk->sr_field_v15_allowed) {
		reg = band == MAC_AX_BAND_1 ? R_AX_TCR0_C1 : R_AX_TCR0;
		val8 = MAC_REG_R8(reg);
		val8 = (u8)(sr_info->sr_field_v15_allowed) ?
			(val8 | B_AX_TCR_SR_VAL15_ALLOW) :
			(val8 & ~B_AX_TCR_SR_VAL15_ALLOW);
		MAC_REG_W8(reg, val8);
	}

	if (sr_info_msk->non_srg_obss_pd_min || sr_info_msk->non_srg_obss_pd_max ||
	    sr_info_msk->srg_obss_pd_min || sr_info_msk->srg_obss_pd_max) {
		reg = band == MAC_AX_BAND_1 ? R_AX_SR_OBSS_PD_C1 : R_AX_SR_OBSS_PD;
		val32 = MAC_REG_R32(reg);
	}

	if (sr_info_msk->non_srg_obss_pd_min)
		val32 = SET_CLR_WORD(val32, sr_info->non_srg_obss_pd_min, B_AX_NONSRG_OBSS_PD_MIN);
	if (sr_info_msk->non_srg_obss_pd_max)
		val32 = SET_CLR_WORD(val32, sr_info->non_srg_obss_pd_max, B_AX_NONSRG_OBSS_PD_MAX);
	if (sr_info_msk->srg_obss_pd_min)
		val32 = SET_CLR_WORD(val32, sr_info->srg_obss_pd_min, B_AX_SRG_OBSS_PD_MIN);
	if (sr_info_msk->srg_obss_pd_max)
		val32 = SET_CLR_WORD(val32, sr_info->srg_obss_pd_max, B_AX_SRG_OBSS_PD_MAX);

	if (sr_info_msk->non_srg_obss_pd_min || sr_info_msk->non_srg_obss_pd_max ||
	    sr_info_msk->srg_obss_pd_min || sr_info_msk->srg_obss_pd_max)
		MAC_REG_W32(reg, val32);

	reg = band == MAC_AX_BAND_1 ?
	      R_AX_SR_BSSCOLOR_BITMAP_C1 : R_AX_SR_BSSCOLOR_BITMAP;
	if (sr_info_msk->srg_bsscolor_bitmap_0)
		MAC_REG_W32(reg, sr_info->srg_bsscolor_bitmap_0);
	if (sr_info_msk->srg_bsscolor_bitmap_1)
		MAC_REG_W32(reg + 4, sr_info->srg_bsscolor_bitmap_1);

	reg = band == MAC_AX_BAND_1 ?
	      R_AX_SR_PARTIAL_BSSCOLOR_BITMAP_C1 :
	      R_AX_SR_PARTIAL_BSSCOLOR_BITMAP;
	if (sr_info_msk->srg_partbsid_bitmap_0)
		MAC_REG_W32(reg, sr_info->srg_partbsid_bitmap_0);
	if (sr_info_msk->srg_partbsid_bitmap_1)
		MAC_REG_W32(reg + 4, sr_info->srg_partbsid_bitmap_1);

	return MACSUCCESS;
}

u32 spatial_reuse_init(struct mac_ax_adapter *adapter,
		       enum mac_ax_band band)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u8 val8;
	u32 ret, reg;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	reg = band == MAC_AX_BAND_1 ? R_AX_RX_SR_CTRL_C1 : R_AX_RX_SR_CTRL;
	val8 = MAC_REG_R8(reg) & ~(B_AX_SR_EN | B_AX_SR_CTRL_PLCP_EN);
	MAC_REG_W8(reg, val8);

	reg = band == MAC_AX_BAND_1 ? R_AX_BSSID_SRC_CTRL_C1 : R_AX_BSSID_SRC_CTRL;
	val8 = MAC_REG_R8(reg) | B_AX_PLCP_SRC_EN;
	MAC_REG_W8(reg, val8);

	return MACSUCCESS;
}

