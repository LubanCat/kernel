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
#include "err_flag.h"
#include "mac_priv.h"

#if MAC_AX_FEATURE_ERR_FLAG

u32 err_flag_cmac(struct mac_ax_adapter *adapter,
		  char *output, u32 out_len, u32 *used,
		  u32 cat, u8 band, struct mac_ax_err_flag_sts *status)
{
	return MACSUCCESS;
}

u32 err_flag_dmac(struct mac_ax_adapter *adapter,
		  char *output, u32 out_len, u32 *used,
		  u32 cat, struct mac_ax_err_flag_sts *status)
{
	return MACSUCCESS;
}

u32 err_flag_rst_cmac(struct mac_ax_adapter *adapter,
		      u32 cat, u8 band)
{
	return MACSUCCESS;
}

u32 err_flag_rst_dmac(struct mac_ax_adapter *adapter,
		      u32 cat)
{
	return MACSUCCESS;
}

u32 err_flag_chk(struct mac_ax_adapter *adapter,
		 char *output, u32 out_len, u32 *used,
		 struct mac_ax_err_flag_sts *status)
{
	struct mac_ax_priv_ops *p_ops = adapter_to_priv_ops(adapter);
	u32 i, ret = MACSUCCESS;
	u32 rtn = MACSUCCESS;

	ret = check_mac_en(adapter, MAC_AX_BAND_0, MAC_AX_CMAC_SEL);
	if (ret == MACSUCCESS)
		for (i = MAC_AX_ERR_FLAG_CAT_COMMON;
		     i < MAC_AX_ERR_FLAG_CAT_LAST; i++) {
			if ((i == MAC_AX_ERR_FLAG_CAT_PCIE &&
			     adapter->env_info.intf != MAC_AX_INTF_PCIE) ||
			    (i == MAC_AX_ERR_FLAG_CAT_USB &&
			     adapter->env_info.intf != MAC_AX_INTF_USB) ||
			    (i == MAC_AX_ERR_FLAG_CAT_SDIO &&
			     adapter->env_info.intf != MAC_AX_INTF_SDIO))
				continue;
			ret = p_ops->err_flag_cmac(adapter,
						   output, out_len, used,
						   i, MAC_AX_BAND_0, status);
			if (ret)
				rtn = ret;
		}

	ret = check_mac_en(adapter, MAC_AX_BAND_1, MAC_AX_CMAC_SEL);
	if (ret == MACSUCCESS)
		for (i = MAC_AX_ERR_FLAG_CAT_COMMON;
		     i < MAC_AX_ERR_FLAG_CAT_LAST; i++) {
			if ((i == MAC_AX_ERR_FLAG_CAT_PCIE &&
			     adapter->env_info.intf != MAC_AX_INTF_PCIE) ||
			    (i == MAC_AX_ERR_FLAG_CAT_USB &&
			     adapter->env_info.intf != MAC_AX_INTF_USB) ||
			    (i == MAC_AX_ERR_FLAG_CAT_SDIO &&
			     adapter->env_info.intf != MAC_AX_INTF_SDIO))
				continue;
			ret = p_ops->err_flag_cmac(adapter,
						   output, out_len, used,
						   i, MAC_AX_BAND_1, status);
			if (ret)
				rtn = ret;
		}

	for (i = MAC_AX_ERR_FLAG_CAT_COMMON;
	     i < MAC_AX_ERR_FLAG_CAT_LAST; i++) {
		if ((i == MAC_AX_ERR_FLAG_CAT_PCIE &&
		     adapter->env_info.intf != MAC_AX_INTF_PCIE) ||
		    (i == MAC_AX_ERR_FLAG_CAT_USB &&
		     adapter->env_info.intf != MAC_AX_INTF_USB) ||
		    (i == MAC_AX_ERR_FLAG_CAT_SDIO &&
		     adapter->env_info.intf != MAC_AX_INTF_SDIO))
			continue;
		ret = p_ops->err_flag_dmac(adapter,
					   output, out_len, used,
					   i, status);
		if (ret)
			rtn = ret;
	}

	// reset flag
	ret = check_mac_en(adapter, MAC_AX_BAND_0, MAC_AX_CMAC_SEL);
	if (ret == MACSUCCESS)
		for (i = MAC_AX_ERR_FLAG_CAT_COMMON;
		     i < MAC_AX_ERR_FLAG_CAT_LAST; i++) {
			if ((i == MAC_AX_ERR_FLAG_CAT_PCIE &&
			     adapter->env_info.intf != MAC_AX_INTF_PCIE) ||
			    (i == MAC_AX_ERR_FLAG_CAT_USB &&
			     adapter->env_info.intf != MAC_AX_INTF_USB) ||
			    (i == MAC_AX_ERR_FLAG_CAT_SDIO &&
			     adapter->env_info.intf != MAC_AX_INTF_SDIO))
				continue;
			p_ops->err_flag_rst_cmac(adapter, i, MAC_AX_BAND_0);
		}

	ret = check_mac_en(adapter, MAC_AX_BAND_1, MAC_AX_CMAC_SEL);
	if (ret == MACSUCCESS)
		for (i = MAC_AX_ERR_FLAG_CAT_COMMON;
		     i < MAC_AX_ERR_FLAG_CAT_LAST; i++) {
			if ((i == MAC_AX_ERR_FLAG_CAT_PCIE &&
			     adapter->env_info.intf != MAC_AX_INTF_PCIE) ||
			    (i == MAC_AX_ERR_FLAG_CAT_USB &&
			     adapter->env_info.intf != MAC_AX_INTF_USB) ||
			    (i == MAC_AX_ERR_FLAG_CAT_SDIO &&
			     adapter->env_info.intf != MAC_AX_INTF_SDIO))
				continue;
			p_ops->err_flag_rst_cmac(adapter, i, MAC_AX_BAND_1);
		}

	for (i = MAC_AX_ERR_FLAG_CAT_COMMON;
	     i < MAC_AX_ERR_FLAG_CAT_LAST; i++) {
		if ((i == MAC_AX_ERR_FLAG_CAT_PCIE &&
		     adapter->env_info.intf != MAC_AX_INTF_PCIE) ||
		    (i == MAC_AX_ERR_FLAG_CAT_USB &&
		     adapter->env_info.intf != MAC_AX_INTF_USB) ||
		    (i == MAC_AX_ERR_FLAG_CAT_SDIO &&
		     adapter->env_info.intf != MAC_AX_INTF_SDIO))
			continue;
		p_ops->err_flag_rst_dmac(adapter, i);
	}
	return rtn;
}
#endif /* MAC_AX_FEATURE_ERR_FLAG */