/******************************************************************************
 *
 * Copyright(c) 2024 Realtek Corporation.
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
#ifndef _PHL_INT_H_
#define _PHL_INT_H_

void rtw_phl_enable_interrupt_sync(struct rtw_phl_com_t* phl_com);
void rtw_phl_disable_interrupt_sync(struct rtw_phl_com_t* phl_com);

static inline void phl_restore_interrupt_sync(struct rtw_phl_com_t* phl_com, bool wlock, bool rx)
{
#ifdef CONFIG_SYNC_INTERRUPT
	struct rtw_phl_evt_ops *evt_ops = &phl_com->evt_ops;

	evt_ops->interrupt_restore(phl_com->drv_priv, rx);
#else
	#if defined(CONFIG_PCI_HCI)
		struct phl_info_t *phl_info = (struct phl_info_t *)phl_com->phl_priv;
	
		if (wlock) {
			void *drv = phl_to_drvpriv(phl_info);
			struct hci_info_t *hci_info = (struct hci_info_t *)phl_info->hci;
			_os_spinlockfg sp_flags;
	
			_os_spinlock(drv, &hci_info->int_hdl_lock, _irq, &sp_flags);
			if (hci_info->int_disabled == false)
				rtw_hal_restore_interrupt(phl_com, phl_info->hal);
			_os_spinunlock(drv, &hci_info->int_hdl_lock, _irq, &sp_flags);
		} else {
			rtw_hal_restore_interrupt(phl_com, phl_info->hal);
		}
	#endif
#endif
}

#endif /*_PHL_INT_H_*/
