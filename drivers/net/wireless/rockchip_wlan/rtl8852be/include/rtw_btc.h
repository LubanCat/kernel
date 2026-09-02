/******************************************************************************
 *
 * Copyright(c) 2013 - 2017 Realtek Corporation.
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
#ifdef CONFIG_BTC

#ifndef __RTW_BTC_H__
#define __RTW_BTC_H__

enum rtw_core_btc_cmd_id {
	RTW_CORE_BTC_CMD_TRXSS_LMT,
	RTW_CORE_BTC_CMD_TRXSS_NO_LMT,
	RTW_CORE_BTC_CMD_MAX
};

enum rtw_btc_esoc_type rtw_btc_update_ext_soc_type(u8 esoc_type);

u8 rtw_core_btc_hdl(_adapter *padapter, enum rtw_core_btc_cmd_id btc_cmd_id);
u8 rtw_core_btc_cmd(_adapter *padapter, enum rtw_core_btc_cmd_id btc_cmd_id, u8 flags);

#ifdef CONFIG_BTC_TRXSS_CHG
u8 rtw_btc_trxss_chg_hdl(struct dvobj_priv *dvobj, struct phl_msg *msg, u16 evt_id);
#endif

#endif /* __RTW_BTC_H__ */
#endif /* CONFIG_BTC */

