/******************************************************************************
 *
 * Copyright(c) 2019 Realtek Corporation.
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
 *****************************************************************************/
#ifndef _PHL_CUSTOM_CSI_H_
#define _PHL_CUSTOM_CSI_H_

#ifdef CONFIG_PHL_CUSTOM_FEATURE_VR
#ifdef CONFIG_PHL_CHANNEL_INFO_VR

struct custom_csi_param {
	u8 ap_role;
	u8 trig_mode;
	u16 trig_intv;
	u8 fb_en;
	u8 fb_mode;
	u8 rslt_qry_en;
	u8 rslt_qry_mode;
	u8 light_mode;
	u8 bitmap_1x1;
	u8 group_num;
};

struct custom_csi_info {
	struct rtw_phl_stainfo_t *sta;
};

struct custom_csi_ctrl {
	enum csi_status status;
	struct custom_csi_param param;
	struct custom_csi_info info;
};

enum phl_mdl_ret_code
rtw_phl_custom_csi_start(void* custom_ctx,
                         struct custom_csi_ctrl *csi_ctrl,
                         struct rtw_wifi_role_link_t *rlink);
enum phl_mdl_ret_code
rtw_phl_custom_csi_stop(void* custom_ctx, struct custom_csi_ctrl *csi_ctrl);

enum phl_mdl_ret_code
rtw_phl_custom_csi_rslt_query(void* custom_ctx, u8 *buf, u32 *len);

#endif /* CONFIG_PHL_CHANNEL_INFO_VR */
#endif /* CONFIG_PHL_CUSTOM_FEATURE_VR */

#endif /* #ifndef _PHL_CUSTOM_CSI_H_ */
