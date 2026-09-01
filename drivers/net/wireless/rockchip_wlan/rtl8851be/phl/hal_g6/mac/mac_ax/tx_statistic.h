#ifndef _MAC_AX_TX_STATISTIC_H_
#define _MAC_AX_TX_STATISTIC_H_

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

#include "../type.h"

struct mac_txrpt_by_endian {
#if PLATFOM_IS_LITTLE_ENDIAN
	/* dword 0 */
	u32 rpt_sel:5;
	u32 polluted:1;
	u32 tx_state:2;
	u32 sw_define:4;
	u32 tx_polluted:1;
	u32 rx_polluted:1;
	u32 try_rate:1;
	u32 fixrate:1;
	u32 macid:7;
	u32 rsvd1:1;
	u32 qsel:6;
	u32 rsvd2:1;
	u32 txop_start:1;
	/* dword 1 */
	u32 queue_time:16;
	u32 acctxtime:8;
	u32 uid:3;
	u32 rsvd3:1;
	u32 bsr:1;
	u32 bmc:1;
	u32 bpri:1;
	u32 bbar:1;

	/* dword 2 */
	u32 final_rate:9;
	u32 final_gi_ltf:3;
	u32 data_bw:2;
	u32 mu2su:1;
	u32 mu_lmt:1;
	u32 final_rts_rate:9;
	u32 final_rts_gi_ltf:3;
	u32 rts_tx_state:2;
	u32 collision_head:1;
	u32 collision_tail:1;
	/* dword 3 */
	u32 total_pkt_num:9;
	u32 rsvd4:1;
	u32 data_tx_cnt:6;
	u32 pkt_ok_num:9;
	u32 rsvd5:1;
	u32 rts_tx_cnt:6;

	/* dword 4 */
	u32 init_rate:9;
	u32 init_gi_ltf:3;
	u32 ppdu_type:2;
	u32 he_tb_ppdu_flag:1;
	u32 ppdu_fst_rpt:1;
	u32 su_txpwr:6;
	u32 rsvd6:2;
	u32 ru_allocation:8;

	/* dword 5 */
	u32 fw_define:16;
	u32 s_idx:8;
	u32 sr_rx_count:4;
	u32 diff_pkt_num:4;
	/* dword 6 */
	u32 pre_zld_len:8;
	u32 mid_zld_len:8;
	u32 nsym:11;
	u32 txpwr_pd:5;
	/* dword 7 */
	u32 info_len:18;
	u32 rsvd7:5;
	u32 txd_over_fetch:1;
	u32 post_zld_len:8;
	/* dword 8 */
	u32 user_define:32;
	/* dword 9 */
	u32 rsvd8:16;
	u32 cmac_dbg_info:16;
#else
	/* dword 0 */
	u32 txop_start:1;
	u32 rsvd2:1;
	u32 qsel:6;
	u32 rsvd1:1;
	u32 macid:7;
	u32 fixrate:1;
	u32 try_rate:1;
	u32 rx_polluted:1;
	u32 tx_polluted:1;
	u32 sw_define:4;
	u32 tx_state:2;
	u32 polluted:1;
	u32 rpt_sel:5;

	/* dword 1 */
	u32 bbar:1;
	u32 bpri:1;
	u32 bmc:1;
	u32 bsr:1;
	u32 rsvd3:1;
	u32 uid:3;
	u32 acctxtime:8;
	u32 queue_time:16;

	/* dword 2 */
	u32 collision_tail:1;
	u32 collision_head:1;
	u32 rts_tx_state:2;
	u32 final_rts_gi_ltf:3;
	u32 final_rts_rate:9;
	u32 mu_lmt:1;
	u32 mu2su:1;
	u32 data_bw:2;
	u32 final_gi_ltf:3;
	u32 final_rate:9;

	/* dword 3 */
	u32 rts_tx_cnt:6;
	u32 rsvd5:1;
	u32 pkt_ok_num:9;
	u32 data_tx_cnt:6;
	u32 rsvd4:1;
	u32 total_pkt_num:9;

	/* dword 4 */
	u32 ru_allocation:8;
	u32 rsvd6:2;
	u32 su_txpwr:6;
	u32 ppdu_fst_rpt:1;
	u32 he_tb_ppdu_flag:1;
	u32 ppdu_type:2;
	u32 init_gi_ltf:3;
	u32 init_rate:9;

	/* dword 5 */
	u32 diff_pkt_num:4;
	u32 sr_rx_count:4;
	u32 s_idx:8;
	u32 fw_define:16;

	/* dword 6 */
	u32 txpwr_pd:5;
	u32 nsym:11;
	u32 mid_zld_len:8;
	u32 pre_zld_len:8;

	/* dword 7 */
	u32 post_zld_len:8;
	u32 txd_over_fetch:1;
	u32 rsvd7:5;
	u32 info_len:18;

	/* dword 8 */
	u32 user_define:32;

	/* dword 9 */
	u32 cmac_dbg_info:16;
	u32 rsvd8:16;
#endif
};

u32 mac_process_txrpt(struct mac_ax_adapter *adapter, u8 *buf, u32 len);

u32 mac_get_sta_tx_dbg_info(struct mac_ax_adapter *adapter, u16 macid,
			    struct mac_tx_debug_info *tx_dbg_info);

u32 mac_clr_sta_tx_dbg_info(struct mac_ax_adapter *adapter, u16 macid);

u32 mac_enable_tx_statistic(struct mac_ax_adapter *adapter, u8 en);

#endif /* _MAC_AX_TX_STATISTIC_H_ */