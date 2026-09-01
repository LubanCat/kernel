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
#ifndef _PHL_CUSTOM_H_
#define _PHL_CUSTOM_H_

#ifdef CONFIG_PHL_CUSTOM_FEATURE
enum rtw_phl_status
phl_register_custom_module(struct phl_info_t *phl_info);

enum rtw_phl_status
phl_custom_prepare_evt_rpt(void *custom_ctx,
                           u32 evt_id,
                           u32 customer_id,
                           u8 *rpt,
                           u32 rpt_len);

enum rtw_phl_status
rtw_phl_custom_prepare_set_info_evt(u32 evt_id,
                                    u32 customer_id,
                                    struct rtw_custom_decrpt *inbuf,
                                    struct phl_module_op_info *op_info,
                                    u32 data_len);

enum rtw_phl_status
rtw_phl_custom_prepare_query_info_evt(u32 evt_id,
                                      u32 customer_id,
                                      struct rtw_custom_decrpt *inbuf,
                                      struct phl_module_op_info *op_info,
                                      u32 data_len);
struct phl_info_t *
phl_custom_get_phl_info(void *custom_ctx);

enum rtw_phl_status
phl_custom_init_role_link_cap(struct phl_info_t *phl_info,
                         u8 hw_band,
                         struct role_link_cap_t *role_link_cap);

enum rtw_phl_status phl_custom_init_protocol_cap(
    struct phl_info_t *phl_info, u8 hw_band, enum role_type rtype,
    struct protocol_cap_t *protocol_cap);

enum rtw_phl_status phl_custom_txsts_notify(struct phl_info_t *phl_info,
					    u8 macid, u16 ac, u8 txsts);
#else
#define phl_register_custom_module(_phl_info) (RTW_PHL_STATUS_SUCCESS)
#define phl_custom_init_role_link_cap(_phl_info, _hw_band, _role_link_cap) (RTW_PHL_STATUS_SUCCESS)
#define phl_custom_init_protocol_cap(_phl_info, _hw_band, _rtype,              \
				     _protocol_cap)                            \
	(RTW_PHL_STATUS_SUCCESS)
#define phl_custom_txsts_notify(_phl_info, _macid, _ac, _txsts)                  \
	(RTW_PHL_STATUS_SUCCESS)
#endif

#endif  /*_PHL_CUSTOMIZE_FEATURE_H_*/

