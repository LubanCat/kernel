/** @file */
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

#ifndef _MAC_AX_HW_SEQ_H_
#define _MAC_AX_HW_SEQ_H_

#include "../type.h"
#include "fwcmd.h"
#if MAC_FEAT_HWSSN

#define DCTL_HW_SEQ_MASK_0 0xfff
#define DCTL_HW_SEQ_MASK_1 0xfff
#define DCTL_HW_SEQ_MASK_2 0xfff
#define DCTL_HW_SEQ_MASK_3 0xfff
#define DCTL_HW_SEQ_SHIFT_0 0
#define DCTL_HW_SEQ_SHIFT_1 12
#define DCTL_HW_SEQ_SHIFT_2 0
#define DCTL_HW_SEQ_SHIFT_3 12
#define DCTL_HW_SEQ_OFFSET_0_1 8
#define DCTL_HW_SEQ_OFFSET_2_3 12
enum {
	R_AX_HW_SEQ_0 = 0,
	R_AX_HW_SEQ_1 = 1,
	R_AX_HW_SEQ_2 = 2,
	R_AX_HW_SEQ_3 = 3,
};

enum hw_ssn_idx {
	DCTL_HW_SEQ_0 = 0,
	DCTL_HW_SEQ_1 = 1,
	DCTL_HW_SEQ_2 = 2,
	DCTL_HW_SEQ_3 = 3
};

/**
 * @struct mac_ax_set_hwseq_reg
 * @brief mac_ax_set_hwseq_reg
 *
 * @var mac_ax_set_hwseq_reg::reg_idx
 * Please Place Description here.
 * @var mac_ax_set_hwseq_reg::seq_val
 * Please Place Description here.
 * @var mac_ax_set_hwseq_reg::rsvd0
 * Please Place Description here.
 */
struct mac_ax_set_hwseq_reg {
	u32 reg_idx: 2;
	u32 seq_val: 12;
	u32 rsvd0: 18;
};

/**
 * @addtogroup Basic_TRX
 * @{
 * @addtogroup TX_ShortCut
 * @{
 */
/**
 * @brief mac_set_hwseq_reg
 *
 * @param *adapter
 * @param idx
 * @param val
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_set_hwseq_reg(struct mac_ax_adapter *adapter,
		      u8 idx,
		      u16 val);

/**
 * @}
 * @}
 */
/**
 * @addtogroup Basic_TRX
 * @{
 * @addtogroup TX_ShortCut
 * @{
 */
/**
 * @brief mac_set_hwseq_extend_macid
 *
 * @param *adapter
 * @param mac_set_hwseq_extend_macid_1115e
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_set_hwseq_extend_macid(struct mac_ax_adapter *adapter,
				     struct mac_ax_dctl_extend_macid *seq_info);

/**
 * @}
 * @}
 */
/**
 * @addtogroup Basic_TRX
 * @{
 * @addtogroup TX_ShortCut
 * @{
 */
/**
 * @brief mac_set_hwseq_dctl_seq_val_1115e
 *
 * @param *adapter
 * @param mac_ax_dctl_seq_val
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_set_hwseq_dctl_seq_val(struct mac_ax_adapter *adapter,
				     struct mac_ax_dctl_seq_val *seq_info);
/**
 * @}
 * @}
 */
/**
 * @addtogroup Basic_TRX
 * @{
 * @addtogroup TX_ShortCut
 * @{
 */
/**
 * @brief mac_set_hwseq_dctrl
 *
 * @param *adapter
 * @param macid
 * @param mac_ax_dctl_seq_cfg
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_set_hwseq_dctrl(struct mac_ax_adapter *adapter,
			u8 macid,
			struct mac_ax_dctl_seq_cfg *seq_info);
/**
 * @}
 * @}
 */

/**
 * @addtogroup Basic_TRX
 * @{
 * @addtogroup TX_ShortCut
 * @{
 */
/**
 * @brief mac_get_hwseq_cfg
 *
 * @param *adapter
 * @param macid
 * @param ref_sel 0:dmac_tbl; 1:CR
 * @param mac_ax_dctl_seq_cfg
 * @return Please Place Description here.
 * @retval u32
 */
u32 mac_get_hwseq_cfg(struct mac_ax_adapter *adapter,
		      u8 macid, u8 ref_sel,
		      struct mac_ax_dctl_seq_cfg *seq_info);
/**
 * @}
 * @}
 */

#endif
#endif
