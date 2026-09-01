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
#ifndef _MAC_OUTSRC_H_
#define _MAC_OUTSRC_H_
#include "../type.h"
#include "../mac_reg.h"

/*mac_write_pwr_limit_en_ax*/
#define PWR_RU_LMT_CTRL_VAL 262144 //0x400 00

u32 mac_write_bbrst_reg(struct mac_ax_adapter  *adapter, u8 val);

u32 mac_read_pwr_reg(struct mac_ax_adapter  *adapter, u8 band,
		     const u32 offset, u32 *val);

u32 mac_write_pwr_reg(struct mac_ax_adapter  *adapter, u8 band,
		      const u32 offset, u32 val);

u32 mac_write_msk_pwr_reg(struct mac_ax_adapter  *adapter, u8 band,
			  const u32 offset, u32 mask, u32 val);

u32 mac_write_pwr_ofst_mode(struct mac_ax_adapter  *adapter,
			    u8 band, struct rtw_tpu_info *tpu);

u32 mac_write_pwr_ofst_bw(struct mac_ax_adapter  *adapter,
			  u8 band, struct rtw_tpu_info *tpu);

u32 mac_write_pwr_ref_reg(struct mac_ax_adapter  *adapter,
			  u8 band, struct rtw_tpu_info *tpu);

u32 mac_write_pwr_limit_en(struct mac_ax_adapter  *adapter,
			   u8 band, struct rtw_tpu_info *tpu);

u32 mac_write_pwr_limit_rua_reg(struct mac_ax_adapter  *adapter,
				u8 band, struct rtw_tpu_info *tpu);

u32 mac_write_pwr_limit_reg(struct mac_ax_adapter  *adapter,
			    u8 band, struct rtw_tpu_pwr_imt_info *tpu);

u32 mac_write_pwr_by_rate_reg(struct mac_ax_adapter  *adapter,
			      u8 band, struct rtw_tpu_pwr_by_rate_info *tpu);

u32 mac_write_txpwr_reg(struct mac_ax_adapter  *adapter, u8 band,
			const u32 offset, u32 val, u8 last_cmd);

u32 mac_write_msk_txpwr_reg(struct mac_ax_adapter  *adapter, u8 band,
			    const u32 offset, u32 mask, u32 val, u8 last_cmd);
#endif //_MAC_OUTSRC_H_
