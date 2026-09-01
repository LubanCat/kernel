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
#define _RTL8851BE_OPS_C_
#include "../rtl8851b_hal.h"

void hal_set_ops_8851be(struct rtw_phl_com_t *phl_com,
				struct hal_info_t *hal)
{
	struct hal_ops_t *ops = hal_get_ops(hal);

	hal_set_ops_8851b(phl_com, hal);

	ops->init_hal_spec = init_hal_spec_8851be;
	ops->hal_get_efuse = hal_get_efuse_8851be;
	ops->hal_fast_start = hal_fast_start_8851be;
	ops->hal_fast_stop = hal_fast_stop_8851be;
	ops->hal_init = hal_init_8851be;
	ops->hal_deinit = hal_deinit_8851be;
	ops->hal_start = hal_start_8851be;
	ops->hal_stop = hal_stop_8851be;
	ops->hal_set_pcicfg = hal_set_pcicfg_8851be;
#ifdef CONFIG_WOWLAN
	ops->hal_wow_init = hal_wow_init_8851be;
	ops->hal_wow_deinit = hal_wow_deinit_8851be;
#endif /* CONFIG_WOWLAN */
	ops->hal_mp_init = hal_mp_init_8851be;
	ops->hal_mp_deinit = hal_mp_deinit_8851be;
	ops->hal_mp_path_chk = hal_mp_path_chk_8851be;

	ops->hal_hci_configure = hal_hci_cfg_8851be;
	ops->init_default_value = hal_init_default_value_8851be;
	ops->init_int_default_value = hal_init_int_default_value_8851be;
	ops->disable_interrupt_isr = hal_disable_int_isr_8851be;
	ops->enable_interrupt = hal_enable_int_8851be;
	ops->disable_interrupt = hal_disable_int_8851be;
	ops->recognize_interrupt = hal_recognize_int_8851be;
	ops->clear_interrupt = hal_clear_int_8851be;
	ops->interrupt_handler = hal_int_hdler_8851be;
	ops->restore_interrupt = hal_restore_int_8851be;
	ops->restore_rx_interrupt = hal_rx_int_restore_8851be;
	ops->get_pcicfg = hal_get_pcicfg_8851be;
#ifdef PHL_RXSC_ISR
	ops->check_rpq_isr = hal_rx_rpq_int_check_8851be;
#endif
}

