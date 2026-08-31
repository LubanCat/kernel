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

#include "ser.h"
#include "mac_priv.h"

#define DBG_SENARIO_SH 28

#define L0_TO_L1_EVENT_SH 28
#define L0_TO_L1_EVENT_MSK 0xF
#define US_TO_SEC 1000000
#define SEC_TO_MIN 60
#define SER_L1_FSM_SH 24
#define SER_L1_FSM_MSK 0xFF
#define SER_L1_FUN_POS_SH 16
#define SER_L1_FUN_POS_MSK 0xFF
#define CMAC1_COUNTER_SH 8
#define CMAC1_COUNTER_MSK 0xff

enum mac_ax_l0_to_l1_event {
	MAC_AX_L0_TO_L1_CHIF_IDLE = 0,
	MAC_AX_L0_TO_L1_CMAC_DMA_IDLE = 1,
	MAC_AX_L0_TO_L1_RLS_PKID = 2,
	MAC_AX_L0_TO_L1_PTCL_IDLE = 3,
	MAC_AX_L0_TO_L1_RX_QTA_LOST = 4,
	MAC_AX_L0_TO_L1_DLE_STAT_HANG = 5,
	MAC_AX_L0_TO_L1_PCIE_STUCK = 6,
	MAC_AX_L0_TO_L1_EVENT_MAX = 15,
};

#if MAC_AX_FEATURE_DBGPKG
static void dump_err_status_dispatcher(struct mac_ax_adapter *adapter);
static void dump_err_status_dmac(struct mac_ax_adapter *adapter);
static void dump_err_status_cmac(struct mac_ax_adapter *adapter, u8 band);
static void dump_cmac_cr(struct mac_ax_adapter *adapter, u8 band);
static void dump_dmac_cr(struct mac_ax_adapter *adapter);
static void dump_sram(struct mac_ax_adapter *adapter, u32 sel);
static void dump_err_status_pciedma_ax(struct mac_ax_adapter *adapter);

u32 mac_dbg_log_dump(struct mac_ax_adapter *adapter)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	struct mac_ax_dbgpkg dbg_val;
	struct mac_ax_dbgpkg_en dbg_en;
	u32 error = 0, dbg_port_dump_en = 0, val32, mem_size;

	error = (enum mac_ax_err_info)MAC_REG_R32(R_AX_HALT_C2H);

	switch (error) {
	case MAC_AX_ERR_L1_ERR_DMAC:
	case MAC_AX_ERR_L0_PROMOTE_TO_L1:
		PLTFM_MEMSET(&dbg_en, 0, sizeof(struct mac_ax_dbgpkg_en));
		if (adapter->ser_info.dbg_lvl.dmac_dbg_dump) {
			dbg_en.dmac_dbg = 1;
			dbg_port_dump_en = 1;
		}
		if (adapter->ser_info.dbg_lvl.cmac_dbg_dump) {
			dbg_en.cmac_dbg = 1;
			dbg_port_dump_en = 1;
		}
		if (adapter->ser_info.dbg_lvl.txflow_dbg_dump) {
			dbg_en.tx_flow_dbg = 1;
			dbg_port_dump_en = 1;
		}
		if (adapter->ser_info.dbg_lvl.dbgprt_dump) {
			dbg_en.mac_dbg_port = 1;
			dbg_en.dle_dbg = 1;
			dbg_port_dump_en = 1;
		}

		if (dbg_port_dump_en) {
			// for debug port log would cause fw wdt
			val32 = MAC_REG_R32(R_AX_HIMR0);
			val32 &= ~B_AX_WDT_TIMEOUT_INT_EN;
			MAC_REG_W32(R_AX_HIMR0, val32);
			mac_dbg_status_dump(adapter, &dbg_val, &dbg_en);
			val32 = MAC_REG_R32(R_AX_HIMR0);
			val32 |= B_AX_WDT_TIMEOUT_INT_EN;
			MAC_REG_W32(R_AX_HIMR0, val32);
		}
		if (adapter->ser_info.dbg_lvl.share_mem_dump) {
			mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_SHARED_BUF);
			if (mem_size == 0)
				return MACSUCCESS;
			val32 = MAC_REG_R32(R_AX_HIMR0);
			val32 &= ~B_AX_WDT_TIMEOUT_INT_EN;
			MAC_REG_W32(R_AX_HIMR0, val32);
			mac_mem_dump(adapter, MAC_AX_MEM_SHARED_BUF, 0x0, NULL, mem_size, 0);
			val32 = MAC_REG_R32(R_AX_HIMR0);
			val32 |= B_AX_WDT_TIMEOUT_INT_EN;
			MAC_REG_W32(R_AX_HIMR0, val32);
		}

		if (adapter->ser_info.dbg_lvl.mac_reg_dump)
			mac_reg_dump(adapter, MAC_AX_REG_MAC);
		break;
	case MAC_AX_ERR_L1_RESET_DISABLE_DMAC_DONE:
		break;
	case MAC_AX_ERR_L1_RESET_RECOVERY_DONE:
		break;
	default:
		break;
	}

	return MACSUCCESS;
}

u32 mac_dbg_log_lvl_adjust(struct mac_ax_adapter *adapter, struct mac_debug_log_lvl *lvl)
{
	adapter->ser_info.dbg_lvl.dbgprt_dump = lvl->dbgprt_dump;
	adapter->ser_info.dbg_lvl.share_mem_dump = lvl->share_mem_dump;
	adapter->ser_info.dbg_lvl.cmac_dbg_dump = lvl->cmac_dbg_dump;
	adapter->ser_info.dbg_lvl.dmac_dbg_dump = lvl->dmac_dbg_dump;
	adapter->ser_info.dbg_lvl.txflow_dbg_dump = lvl->txflow_dbg_dump;
	adapter->ser_info.dbg_lvl.mac_reg_dump = lvl->mac_reg_dump;

	return MACSUCCESS;
}

u32 mac_dump_qta_lost(struct mac_ax_adapter *adapter)
{
	struct dle_dfi_qempty_t qempty;
	struct dle_dfi_quota_t quota;
	struct dle_dfi_ctrl_t ctrl;
	struct mac_ax_priv_ops *p_ops = adapter_to_priv_ops(adapter);
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ret, val, not_empty, i;

	qempty.dle_type = DLE_CTRL_TYPE_PLE;
	qempty.grpsel = 0;
	ret = dle_dfi_qempty(adapter, &qempty);
	if (ret == MACSUCCESS)
		PLTFM_MSG_ALWAYS("DLE group0 empty: 0x%x\n", qempty.qempty);
	else
		PLTFM_MSG_ERR("%s: query DLE fail\n", __func__);

	for (not_empty = ~qempty.qempty, i = 0;
	     not_empty != 0; not_empty = not_empty >> 1, i++) {
		if (!(not_empty & BIT(0)))
			continue;
		ctrl.type = DLE_CTRL_TYPE_PLE;
		ctrl.target = DLE_DFI_TYPE_QLNKTBL;
		ctrl.addr = (QLNKTBL_ADDR_INFO_SEL_0 ?
			     QLNKTBL_ADDR_INFO_SEL : 0) |
			SET_WORD(i, QLNKTBL_ADDR_TBL_IDX);
		ret = p_ops->dle_dfi_ctrl(adapter, &ctrl);
		if (ret == MACSUCCESS)
			PLTFM_MSG_ALWAYS("qidx%d pktcnt = %d\n", i,
					 GET_FIELD(ctrl.out_data,
						   QLNKTBL_DATA_SEL1_PKT_CNT));
		else
			PLTFM_MSG_ERR("%s: query DLE fail\n", __func__);
	}

	/* cmac0 */
	quota.dle_type = DLE_CTRL_TYPE_PLE;
	quota.qtaid = 6;
	ret = dle_dfi_quota(adapter, &quota);
	if (ret == MACSUCCESS)
		PLTFM_MSG_ALWAYS("quota6 rsv/use: 0x%x/0x%x\n",
				 quota.rsv_pgnum, quota.use_pgnum);
	else
		PLTFM_MSG_ERR("%s: query DLE fail\n", __func__);

	val = MAC_REG_R32(R_AX_PLE_QTA6_CFG);
	PLTFM_MSG_ALWAYS("[PLE][CMAC0_RX]min_pgnum=0x%x\n",
			 GET_FIELD(val, B_AX_PLE_Q6_MIN_SIZE));
	PLTFM_MSG_ALWAYS("[PLE][CMAC0_RX]max_pgnum=0x%x\n",
			 GET_FIELD(val, B_AX_PLE_Q6_MAX_SIZE));

	val = MAC_REG_R32(R_AX_RX_FLTR_OPT);
	PLTFM_MSG_ALWAYS("[CMAC0_RX]B_AX_RX_MPDU_MAX_LEN=0x%x\n",
			 GET_FIELD(val, B_AX_RX_MPDU_MAX_LEN));

	val = MAC_REG_R32(R_AX_RSP_CHK_SIG);
	PLTFM_MSG_ALWAYS("R_AX_RSP_CHK_SIG=0x%x\n",
			 val);

	val = MAC_REG_R32(R_AX_TRXPTCL_RESP_0);
	PLTFM_MSG_ALWAYS("R_AX_TRXPTCL_RESP_0=0x%x\n",
			 val);

	val = MAC_REG_R32(R_AX_CCA_CONTROL);
	PLTFM_MSG_ALWAYS("R_AX_CCA_CONTROL=0x%x\n",
			 val);

	/* cmac1 */
	ret = check_mac_en(adapter, 1, MAC_AX_CMAC_SEL);

	if (ret == MACSUCCESS) {
		quota.dle_type = DLE_CTRL_TYPE_PLE;
		quota.qtaid = 7;
		ret = dle_dfi_quota(adapter, &quota);
		if (ret == MACSUCCESS)
			PLTFM_MSG_ALWAYS("quota7 rsv/use: 0x%x/0x%x\n",
					 quota.rsv_pgnum, quota.use_pgnum);
		else
			PLTFM_MSG_ERR("%s: query DLE fail\n", __func__);

		val = MAC_REG_R32(R_AX_PLE_QTA7_CFG);
		PLTFM_MSG_ALWAYS("[PLE][CMAC1_RX]min_pgnum=0x%x\n",
				 GET_FIELD(val, B_AX_PLE_Q7_MIN_SIZE));
		PLTFM_MSG_ALWAYS("[PLE][CMAC1_RX]max_pgnum=0x%x\n",
				 GET_FIELD(val, B_AX_PLE_Q7_MAX_SIZE));

		val = MAC_REG_R32(R_AX_RX_FLTR_OPT_C1);
		PLTFM_MSG_ALWAYS("[CMAC1_RX]B_AX_RX_MPDU_MAX_LEN=0x%x\n",
				 GET_FIELD(val, B_AX_RX_MPDU_MAX_LEN));

		val = MAC_REG_R32(R_AX_RSP_CHK_SIG_C1);
		PLTFM_MSG_ALWAYS("R_AX_RSP_CHK_SIG_C1=0x%x\n",
				 val);

		val = MAC_REG_R32(R_AX_TRXPTCL_RESP_0_C1);
		PLTFM_MSG_ALWAYS("R_AX_TRXPTCL_RESP_0_C1=0x%x\n",
				 val);

		val = MAC_REG_R32(R_AX_CCA_CONTROL_C1);
		PLTFM_MSG_ALWAYS("R_AX_CCA_CONTROL_C1=0x%x\n",
				 val);
	}

	val = MAC_REG_R32(R_AX_DLE_EMPTY0);
	PLTFM_MSG_ALWAYS("R_AX_DLE_EMPTY0=0x%x\n",
			 val);

	val = MAC_REG_R32(R_AX_DLE_EMPTY1);
	PLTFM_MSG_ALWAYS("R_AX_DLE_EMPTY1=0x%x\n",
			 val);

	dump_err_status_dispatcher(adapter);

	return MACSUCCESS;
}

u32 mac_dump_l0_to_l1(struct mac_ax_adapter *adapter,
		      enum mac_ax_err_info err)
{
	u32 dbg, event;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	PLTFM_MSG_ALWAYS("%s\n", __func__);

	dbg = MAC_REG_R32(R_AX_SER_DBG_INFO);
	event = GET_FIELD(dbg, L0_TO_L1_EVENT);

	switch (event) {
	case MAC_AX_L0_TO_L1_RX_QTA_LOST:
		PLTFM_MSG_ALWAYS("quota lost!\n");
		mac_dump_qta_lost(adapter);
		break;
	default:
		break;
	}

	return MACSUCCESS;
}

u32 mac_dump_err_status(struct mac_ax_adapter *adapter,
			enum mac_ax_err_info err)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	struct mac_ax_dbgpkg dbg_val;
	struct mac_ax_dbgpkg_en dbg_en;
	u32 cmac_err, dmac_err;
	u8 val8;
	u32 ret, i;
#if MAC_AX_FEATURE_DBGPKG
	struct mac_ax_priv_ops *p_ops = adapter_to_priv_ops(adapter);
#endif
	PLTFM_MSG_ERR("--->\n err=0x%x\n", err);
	if (err == MAC_AX_ERR_L1_ERR_DMAC ||
	    err == MAC_AX_ERR_L0_PROMOTE_TO_L1 ||
	    err == MAC_AX_ERR_L0_ERR_CMAC0 ||
	    err == MAC_AX_ERR_L0_ERR_CMAC1 ||
	    err == HALT_C2H_L1_DBG_MODE ||
	    err == MAC_AX_ERR_L0_PROMOTE_TO_L1_DBG) {
		PLTFM_MSG_ERR("R_AX_SER_DBG_INFO =0x%08x\n",
			      MAC_REG_R32(R_AX_SER_DBG_INFO));
		PLTFM_MSG_ERR("DBG Counter 1 (0x1C0)=0x%08x\n",
			      MAC_REG_R32(R_AX_DRV_FW_HSK_4));
		PLTFM_MSG_ERR("DBG Counter 2 (0x1C4)=0x%08x\n",
			      MAC_REG_R32(R_AX_DRV_FW_HSK_5));

		dump_err_status_dmac(adapter);
		dump_err_status_cmac(adapter, MAC_AX_BAND_0);
		dump_err_status_cmac(adapter, MAC_AX_BAND_1);

		if (adapter->env_info.intf == MAC_AX_INTF_PCIE) {
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
			if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
			    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
			    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
			    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
				PLTFM_MSG_ERR("R_AX_DBG_ERR_FLAG=0x%08x\n",
					      MAC_REG_R32(R_AX_DBG_ERR_FLAG));
				PLTFM_MSG_ERR("R_AX_LBC_WATCHDOG=0x%08x\n",
					      MAC_REG_R32(R_AX_LBC_WATCHDOG));
			}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
			if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
			    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
			    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
				PLTFM_MSG_ERR("R_AX_DBG_ERR_FLAG=0x%08x\n",
					      MAC_REG_R32(R_AX_DBG_ERR_FLAG_V1));
				PLTFM_MSG_ERR("R_AX_LBC_WATCHDOG=0x%08x\n",
					      MAC_REG_R32(R_AX_LBC_WATCHDOG_V1));
			}
#endif
		}
		if (err == MAC_AX_ERR_L0_PROMOTE_TO_L1 ||
		    err == MAC_AX_ERR_L0_PROMOTE_TO_L1_DBG)
			mac_dump_l0_to_l1(adapter, err);
	}

	if (err == MAC_AX_ERR_L1_RESET_RECOVERY_DONE)
		PLTFM_MSG_ERR("SER duration (unit: us)=0x%08x\n",
			      MAC_REG_R32(R_AX_PLE_WORD0F8));

	if (err == MAC_AX_ERR_L0_ERR_CMAC0 ||
	    err == MAC_AX_ERR_L0_ERR_CMAC1 ||
	    err == HALT_C2H_L1_DBG_MODE ||
	    err == MAC_AX_ERR_L0_PROMOTE_TO_L1_DBG) {
		cmac_err = MAC_REG_R32(R_AX_CMAC_ERR_ISR);
		dmac_err = MAC_REG_R32(R_AX_DMAC_ERR_ISR);
		dump_dmac_cr(adapter);
		dump_cmac_cr(adapter, MAC_AX_BAND_0);
		dump_cmac_cr(adapter, MAC_AX_BAND_1);
		// dump DLE TBL & STA SCH
		PLTFM_MEMSET(&dbg_en, 0, sizeof(struct mac_ax_dbgpkg_en));
		dbg_en.ss_dbg = 1;
		dbg_en.dle_dbg = 1;
		mac_dbg_status_dump(adapter, &dbg_val, &dbg_en);
#if MAC_AX_FEATURE_DBGPKG
		// dump debug port
		for (i = MAC_AX_DBG_PORT_SEL_PTCL_C0; i < MAC_AX_DBG_PORT_SEL_LAST; i++) {
			if (p_ops->is_dbg_port_not_valid(adapter, i))
				continue;
			ret = dbg_port_dump(adapter, i);
			if (ret == MACDBGPORTSEL) {
				PLTFM_MSG_WARN("no dbg port sel %d\n", i);
				continue;
			} else if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("dbg port %d dump %d\n", i, ret);
				break;
			}
		}
#endif
		// dump SRAM
		PLTFM_MSG_ERR("Dump SRAM start with free run enable\n");
		MAC_REG_W16(R_AX_RXGCK_CTRL, MAC_REG_R16(R_AX_RXGCK_CTRL) | B_AX_DISGCLK);
		PLTFM_MSG_ERR("R_AX_RXGCK_CTRL 0xCE06 = %d\n", MAC_REG_R16(R_AX_RXGCK_CTRL));
		for (i = MAC_AX_MEM_CPU_LOCAL; i < MAC_AX_MEM_LAST; i++)
			dump_sram(adapter, i);
		val8 = MAC_REG_R8(R_AX_UDM3) | BIT0;
		MAC_REG_W8(R_AX_UDM3, val8);
	}
	PLTFM_MSG_ERR("<---\n");

	return MACSUCCESS;
}

static void dump_err_status_dispatcher(struct mac_ax_adapter *adapter)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	PLTFM_MSG_ERR("R_AX_HOST_DISPATCHER_ERR_IMR=0x%08x\n",
		      MAC_REG_R32(R_AX_HOST_DISPATCHER_ERR_IMR));
	PLTFM_MSG_ERR("R_AX_HOST_DISPATCHER_ERR_ISR=0x%08x\n",
		      MAC_REG_R32(R_AX_HOST_DISPATCHER_ERR_ISR));

	PLTFM_MSG_ERR("R_AX_CPU_DISPATCHER_ERR_IMR=0x%08x\n",
		      MAC_REG_R32(R_AX_CPU_DISPATCHER_ERR_IMR));
	PLTFM_MSG_ERR("R_AX_CPU_DISPATCHER_ERR_ISR=0x%08x\n",
		      MAC_REG_R32(R_AX_CPU_DISPATCHER_ERR_ISR));
	PLTFM_MSG_ERR("R_AX_OTHER_DISPATCHER_ERR_IMR=0x%08x ",
		      MAC_REG_R32(R_AX_OTHER_DISPATCHER_ERR_IMR));
	PLTFM_MSG_ERR("R_AX_OTHER_DISPATCHER_ERR_ISR=0x%08x\n",
		      MAC_REG_R32(R_AX_OTHER_DISPATCHER_ERR_ISR));
}

static void dump_err_status_pciedma_ax(struct mac_ax_adapter *adapter)
{
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
#if MAC_AX_PCIE_SUPPORT
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	struct mac_ax_priv_ops *p_ops = adapter_to_priv_ops(adapter);
	struct mac_ax_hfc_ch_info *ch_info = adapter->hfc_param->ch_info;
	u32 val32;
	u8 ch = 0;

	PLTFM_MSG_ALWAYS("R_AX_PCIE_INIT_CFG1(0x1000)=0x%08x\n",
			 MAC_REG_R32(R_AX_PCIE_INIT_CFG1));
	PLTFM_MSG_ALWAYS("R_AX_PCIE_DMA_STOP1(0x1010)=0x%08x\n",
			 MAC_REG_R32(R_AX_PCIE_DMA_STOP1));
	PLTFM_MSG_ALWAYS("R_AX_PCIE_DMA_BUSY1(0x101C)=0x%08x\n",
			 MAC_REG_R32(R_AX_PCIE_DMA_BUSY1));
	PLTFM_MSG_ALWAYS("R_AX_DBG_ERR_FLAG(0x11C4)=0x%08x\n",
			 MAC_REG_R32(R_AX_DBG_ERR_FLAG));
	PLTFM_MSG_ALWAYS("R_AX_PCIE_HIMR00(0x10B0)=0x%08x\n",
			 MAC_REG_R32(R_AX_PCIE_HIMR00));
	PLTFM_MSG_ALWAYS("R_AX_PCIE_HISR00(0x10B4)=0x%08x\n",
			 MAC_REG_R32(R_AX_PCIE_HISR00));
	PLTFM_MSG_ALWAYS("R_AX_DLE_EMPTY0(0x8430)=0x%08x\n",
			 MAC_REG_R32(R_AX_DLE_EMPTY0));
	PLTFM_MSG_ALWAYS("R_AX_DLE_EMPTY1(0x8434)=0x%08x\n",
			 MAC_REG_R32(R_AX_DLE_EMPTY1));
	dump_err_status_dispatcher(adapter);
	PLTFM_MSG_ALWAYS("R_AX_HDP_DBG_INFO_0(0x8880)=0x%08x\n",
			 MAC_REG_R32(R_AX_HDP_DBG_INFO_0));
	PLTFM_MSG_ALWAYS("R_AX_HDP_DBG_INFO_2(0x8888)=0x%08x\n",
			 MAC_REG_R32(R_AX_HDP_DBG_INFO_2));
	PLTFM_MSG_ALWAYS("R_AX_HDP_DBG_INFO_3(0x888C)=0x%08x\n",
			 MAC_REG_R32(R_AX_HDP_DBG_INFO_3));
	val32 = MAC_REG_R32(R_AX_HDP_DBG_INFO_4);
	PLTFM_MSG_ALWAYS("R_AX_HDP_DBG_INFO_4(0x8890)=0x%08x\n",
			 val32);
	if (GET_FIELD(val32, B_AX_HFC_CH_FULL))
		for (ch = MAC_AX_DMA_ACH0; ch < MAC_AX_DMA_CH_NUM; ch++)
			if (IS_TXDMA_CH_VALID(ch)) {
				p_ops->get_fc_page_info(adapter, ch);
				PLTFM_MSG_ALWAYS("CH%d FC aval/used: %d/%d\n", ch,
						 (u32)ch_info[ch].aval,
						 (u32)ch_info[ch].used);
			}
	mac_bdinfo_dump(adapter);
#endif
#endif
}

static void dump_err_status_dmac(struct mac_ax_adapter *adapter)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	struct mac_ax_priv_ops *p_ops = adapter_to_priv_ops(adapter);
	struct mac_ax_hfc_ch_info *ch_info = adapter->hfc_param->ch_info;
	u32 dmac_err;

	dmac_err = MAC_REG_R32(R_AX_DMAC_ERR_ISR);
	PLTFM_MSG_ERR("R_AX_DMAC_ERR_ISR =0x%08x\n", dmac_err);
	PLTFM_MSG_ALWAYS("R_AX_DMAC_ERR_IMR=0x%x\n",
			 MAC_REG_R32(R_AX_DMAC_ERR_IMR));

	PLTFM_MSG_ERR("R_AX_DMAC_FUNC_EN =0x%08x\n",
		      MAC_REG_R32(R_AX_DMAC_FUNC_EN));
	PLTFM_MSG_ERR("R_AX_DMAC_CLK_EN =0x%08x\n",
		      MAC_REG_R32(R_AX_DMAC_CLK_EN));

	if (dmac_err) {
		PLTFM_MSG_ERR("R_AX_WDE_ERR_FLAG_CFG =0x%08x\n",
			      MAC_REG_R32(R_AX_WDE_ERR_FLAG_CFG_NUM1));
		PLTFM_MSG_ERR("R_AX_PLE_ERR_FLAG_CFG =0x%08x\n",
			      MAC_REG_R32(R_AX_PLE_ERR_FLAG_CFG_NUM1));
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			PLTFM_MSG_ERR("R_AX_PLE_ERRFLAG_MSG =0x%08x\n",
				      MAC_REG_R32(R_AX_PLE_ERRFLAG_MSG));
			PLTFM_MSG_ERR("R_AX_PLE_ERRFLAG_MSG 0x9030 Reg dump =0x%08x\n",
				      MAC_REG_R32(R_AX_PLE_ERRFLAG_MSG));
			PLTFM_MSG_ERR("R_AX_WDE_ERRFLAG_MSG 0x8C30 Reg dump =0x%08x\n",
				      MAC_REG_R32(R_AX_WDE_ERRFLAG_MSG));
			PLTFM_MSG_ERR("R_AX_PLE_DBGERR_LOCKEN 0x9020 Reg dump =0x%08x\n",
				      MAC_REG_R32(R_AX_PLE_DBGERR_LOCKEN));
			PLTFM_MSG_ERR("R_AX_PLE_DBGERR_STS 0x9024 Reg dump =0x%08x\n",
				      MAC_REG_R32(R_AX_PLE_DBGERR_STS));
		}
#endif
	}

	if (dmac_err & B_AX_WDRLS_ERR_FLAG) {
		PLTFM_MSG_ERR("R_AX_WDRLS_ERR_IMR =0x%08x\n",
			      MAC_REG_R32(R_AX_WDRLS_ERR_IMR));
		PLTFM_MSG_ERR("R_AX_WDRLS_ERR_ISR =0x%08x\n",
			      MAC_REG_R32(R_AX_WDRLS_ERR_ISR));
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			PLTFM_MSG_ERR("R_AX_RPQ_RXBD_IDX =0x%08x\n",
				      MAC_REG_R32(R_AX_RPQ_RXBD_IDX));
		}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			PLTFM_MSG_ERR("R_AX_RPQ_RXBD_IDX =0x%08x\n",
				      MAC_REG_R32(R_AX_RPQ_RXBD_IDX_V1));
		}
#endif
	}

	if (dmac_err & B_AX_WSEC_ERR_FLAG) {
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			PLTFM_MSG_ERR("R_AX_SEC_ERR_IMR_ISR =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_DEBUG));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_ENG_CTRL 0x9D00 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_ENG_CTRL));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_MPDU_PROC 0x9D04 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_MPDU_PROC));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_CAM_ACCESS 0x9D10 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_CAM_ACCESS));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_CAM_RDATA 0x9D14 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_CAM_RDATA));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_CAM_WDATA 0x9D18 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_CAM_WDATA));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_TX_DEBUG 0x9D20 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_TX_DEBUG));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_RX_DEBUG 0x9D24 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_RX_DEBUG));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_TRX_PKT_CNT 0x9D28 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_TRX_PKT_CNT));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_TRX_BLK_CNT 0x9D2C =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_TRX_BLK_CNT));
		}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
		u32 val32 = 0, i = 0;

		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			PLTFM_MSG_ERR("R_AX_SEC_ERR_IMR =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_ERROR_FLAG_IMR));
			PLTFM_MSG_ERR("R_AX_SEC_ERR_ISR =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_ERROR_FLAG));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_ENG_CTRL 0x9D00 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_ENG_CTRL));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_MPDU_PROC 0x9D04 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_MPDU_PROC));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_CAM_ACCESS 0x9D10 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_CAM_ACCESS));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_CAM_RDATA 0x9D14 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_CAM_RDATA));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_DEBUG1 0x9D1C =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_DEBUG1));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_TX_DEBUG 0x9D20 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_TX_DEBUG));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_RX_DEBUG 0x9D24 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_RX_DEBUG));
			PLTFM_MSG_ERR("SEC_Register R_AX_SEC_DEBUG2 0x9D28 =0x%08x\n",
				      MAC_REG_R32(R_AX_SEC_DEBUG2));

			val32 = MAC_REG_R32(R_AX_DBG_CTRL);
			val32 = SET_CLR_WORD(val32, SEC_DBG_SEL, B_AX_DBG_SEL0);
			val32 = SET_CLR_WORD(val32, SEC_DBG_SEL, B_AX_DBG_SEL1);
			MAC_REG_W32(R_AX_DBG_CTRL, val32);

			val32 = MAC_REG_R32(R_AX_SYS_STATUS1);
			val32 = SET_CLR_WORD(val32, MAC_DBG_SEL, B_AX_SEL_0XC0);
			MAC_REG_W32(R_AX_SYS_STATUS1, val32);
			PLTFM_MSG_ALWAYS("SECURITY ENGINE DEBUG PORT DUMP\n");
			for (i = 0; i < SEC_DBG_PORT_NUM; i++) {
				val32 = MAC_REG_R32(R_AX_SEC_ENG_CTRL);
				val32 = SET_CLR_WORD(val32, i, SEC_DBG_PORT_FIELD);
				MAC_REG_W32(R_AX_SEC_ENG_CTRL, val32);
				PLTFM_MSG_ERR("debug sel = %x, 0x9D28 =0x%08x\n",
					      i, MAC_REG_R32(R_AX_SEC_DEBUG2));
			}
		}
#endif
	}

	if (dmac_err & B_AX_MPDU_ERR_FLAG) {
		PLTFM_MSG_ERR("R_AX_MPDU_TX_ERR_IMR =0x%08x\n",
			      MAC_REG_R32(R_AX_MPDU_TX_ERR_IMR));
		PLTFM_MSG_ERR("R_AX_MPDU_TX_ERR_ISR =0x%08x\n",
			      MAC_REG_R32(R_AX_MPDU_TX_ERR_ISR));
		PLTFM_MSG_ERR("R_AX_MPDU_RX_ERR_IMR =0x%08x\n",
			      MAC_REG_R32(R_AX_MPDU_RX_ERR_IMR));
		PLTFM_MSG_ERR("R_AX_MPDU_RX_ERR_ISR =0x%08x\n",
			      MAC_REG_R32(R_AX_MPDU_RX_ERR_ISR));
	}

	if (dmac_err & B_AX_STA_SCHEDULER_ERR_FLAG) {
		PLTFM_MSG_ERR("R_AX_STA_SCHEDULER_ERR_IMR =0x%08x\n",
			      MAC_REG_R32(R_AX_STA_SCHEDULER_ERR_IMR));
		PLTFM_MSG_ERR("R_AX_STA_SCHEDULER_ERR_ISR= 0x%08x\n",
			      MAC_REG_R32(R_AX_STA_SCHEDULER_ERR_ISR));
	}

	if (dmac_err & B_AX_WDE_DLE_ERR_FLAG) {
		PLTFM_MSG_ERR("R_AX_WDE_ERR_IMR=0x%08x\n",
			      MAC_REG_R32(R_AX_WDE_ERR_IMR));
		PLTFM_MSG_ERR("R_AX_WDE_ERR_ISR=0x%08x\n",
			      MAC_REG_R32(R_AX_WDE_ERR_ISR));
		PLTFM_MSG_ERR("R_AX_PLE_ERR_IMR=0x%08x\n",
			      MAC_REG_R32(R_AX_PLE_ERR_IMR));
		PLTFM_MSG_ERR("R_AX_PLE_ERR_FLAG_ISR=0x%08x\n",
			      MAC_REG_R32(R_AX_PLE_ERR_FLAG_ISR));
	}

	if (dmac_err & B_AX_TXPKTCTRL_ERR_FLAG) {
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			PLTFM_MSG_ERR("R_AX_TXPKTCTL_ERR_IMR_ISR=0x%08x\n",
				      MAC_REG_R32(R_AX_TXPKTCTL_ERR_IMR_ISR));
			PLTFM_MSG_ERR("R_AX_TXPKTCTL_ERR_IMR_ISR_B1=0x%08x\n",
				      MAC_REG_R32(R_AX_TXPKTCTL_ERR_IMR_ISR_B1));
		}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			PLTFM_MSG_ERR("R_AX_TXPKTCTL_B0_ERRFLAG_IMR=0x%08x\n",
				      MAC_REG_R32(R_AX_TXPKTCTL_B0_ERRFLAG_IMR));
			PLTFM_MSG_ERR("R_AX_TXPKTCTL_B0_ERRFLAG_ISR=0x%08x\n",
				      MAC_REG_R32(R_AX_TXPKTCTL_B0_ERRFLAG_ISR));
			PLTFM_MSG_ERR("R_AX_TXPKTCTL_B1_ERRFLAG_IMR=0x%08x\n",
				      MAC_REG_R32(R_AX_TXPKTCTL_B1_ERRFLAG_IMR));
			PLTFM_MSG_ERR("R_AX_TXPKTCTL_B1_ERRFLAG_ISR=0x%08x\n",
				      MAC_REG_R32(R_AX_TXPKTCTL_B1_ERRFLAG_ISR));
		}
#endif
	}

	if (dmac_err & B_AX_PLE_DLE_ERR_FLAG) {
		PLTFM_MSG_ERR("R_AX_WDE_ERR_IMR=0x%08x\n",
			      MAC_REG_R32(R_AX_WDE_ERR_IMR));
		PLTFM_MSG_ERR("R_AX_WDE_ERR_ISR=0x%08x\n",
			      MAC_REG_R32(R_AX_WDE_ERR_ISR));
		PLTFM_MSG_ERR("R_AX_PLE_ERR_IMR=0x%08x\n",
			      MAC_REG_R32(R_AX_PLE_ERR_IMR));
		PLTFM_MSG_ERR("R_AX_PLE_ERR_FLAG_ISR=0x%08x\n",
			      MAC_REG_R32(R_AX_PLE_ERR_FLAG_ISR));
		PLTFM_MSG_ERR("R_AX_WD_CPUQ_OP_0=0x%08x\n",
			      MAC_REG_R32(R_AX_WD_CPUQ_OP_0));
		PLTFM_MSG_ERR("R_AX_WD_CPUQ_OP_1=0x%08x\n",
			      MAC_REG_R32(R_AX_WD_CPUQ_OP_1));
		PLTFM_MSG_ERR("R_AX_WD_CPUQ_OP_2=0x%08x\n",
			      MAC_REG_R32(R_AX_WD_CPUQ_OP_2));
		PLTFM_MSG_ERR("R_AX_WD_CPUQ_OP_STATUS=0x%08x\n",
			      MAC_REG_R32(R_AX_WD_CPUQ_OP_STATUS));
		PLTFM_MSG_ERR("R_AX_PL_CPUQ_OP_0=0x%08x\n",
			      MAC_REG_R32(R_AX_PL_CPUQ_OP_0));
		PLTFM_MSG_ERR("R_AX_PL_CPUQ_OP_1=0x%08x\n",
			      MAC_REG_R32(R_AX_PL_CPUQ_OP_1));
		PLTFM_MSG_ERR("R_AX_PL_CPUQ_OP_2=0x%08x\n",
			      MAC_REG_R32(R_AX_PL_CPUQ_OP_2));
		PLTFM_MSG_ERR("R_AX_PL_CPUQ_OP_STATUS=0x%08x\n",
			      MAC_REG_R32(R_AX_PL_CPUQ_OP_STATUS));
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			PLTFM_MSG_ERR("R_AX_RXDMA_PKT_INFO_0=0x%08x\n",
				      MAC_REG_R32(R_AX_RXDMA_PKT_INFO_0));
			PLTFM_MSG_ERR("R_AX_RXDMA_PKT_INFO_1=0x%08x\n",
				      MAC_REG_R32(R_AX_RXDMA_PKT_INFO_1));
			PLTFM_MSG_ERR("R_AX_RXDMA_PKT_INFO_2=0x%08x\n",
				      MAC_REG_R32(R_AX_RXDMA_PKT_INFO_2));
		}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			PLTFM_MSG_ERR("R_AX_RX_CTRL0=0x%08x\n",
				      MAC_REG_R32(R_AX_RX_CTRL0));
			PLTFM_MSG_ERR("R_AX_RX_CTRL1=0x%08x\n",
				      MAC_REG_R32(R_AX_RX_CTRL1));
			PLTFM_MSG_ERR("R_AX_RX_CTRL2=0x%08x\n",
				      MAC_REG_R32(R_AX_RX_CTRL2));
		}
#endif
		dump_err_status_dispatcher(adapter);
	}

	if (dmac_err & B_AX_PKTIN_ERR_FLAG) {
		PLTFM_MSG_ERR("R_AX_PKTIN_ERR_IMR =0x%08x\n",
			      MAC_REG_R32(R_AX_PKTIN_ERR_IMR));
		PLTFM_MSG_ERR("R_AX_PKTIN_ERR_ISR =0x%08x\n",
			      MAC_REG_R32(R_AX_PKTIN_ERR_ISR));
		PLTFM_MSG_ERR("R_AX_PKTIN_ERR_IMR =0x%08x ",
			      MAC_REG_R32(R_AX_PKTIN_ERR_IMR));
		PLTFM_MSG_ERR("R_AX_PKTIN_ERR_ISR =0x%08x\n",
			      MAC_REG_R32(R_AX_PKTIN_ERR_ISR));
	}

	if (dmac_err & B_AX_DISPATCH_ERR_FLAG)
		dump_err_status_dispatcher(adapter);

	if (dmac_err & B_AX_DLE_CPUIO_ERR_FLAG) {
		PLTFM_MSG_ERR("R_AX_CPUIO_ERR_IMR=0x%08x\n",
			      MAC_REG_R32(R_AX_CPUIO_ERR_IMR));
		PLTFM_MSG_ERR("R_AX_CPUIO_ERR_ISR=0x%08x\n",
			      MAC_REG_R32(R_AX_CPUIO_ERR_ISR));
	}

	if (dmac_err & B_AX_BBRPT_ERR_FLAG) {
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			PLTFM_MSG_ERR("R_AX_BBRPT_COM_ERR_IMR_ISR=0x%08x\n",
				      MAC_REG_R32(R_AX_BBRPT_COM_ERR_IMR_ISR));
			PLTFM_MSG_ERR("R_AX_BBRPT_CHINFO_ERR_ISR=0x%08x\n",
				      MAC_REG_R32(R_AX_BBRPT_CHINFO_ERR_ISR));
			PLTFM_MSG_ERR("R_AX_BBRPT_CHINFO_ERR_IMR=0x%08x\n",
				      MAC_REG_R32(R_AX_BBRPT_CHINFO_ERR_IMR));
			PLTFM_MSG_ERR("R_AX_BBRPT_DFS_ERR_IMR=0x%08x\n",
				      MAC_REG_R32(R_AX_BBRPT_DFS_ERR_IMR));
			PLTFM_MSG_ERR("R_AX_BBRPT_DFS_ERR_ISR=0x%08x\n",
				      MAC_REG_R32(R_AX_BBRPT_DFS_ERR_ISR));
		}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			PLTFM_MSG_ERR("R_AX_BBRPT_COM_ERR_IMR=0x%08x\n",
				      MAC_REG_R32(R_AX_BBRPT_COM_ERR_IMR));
			PLTFM_MSG_ERR("R_AX_BBRPT_COM_ERR_ISR=0x%08x\n",
				      MAC_REG_R32(R_AX_BBRPT_COM_ERR_ISR));
			PLTFM_MSG_ERR("R_AX_BBRPT_CHINFO_ERR_ISR=0x%08x\n",
				      MAC_REG_R32(R_AX_BBRPT_CHINFO_ERR_ISR));
			PLTFM_MSG_ERR("R_AX_BBRPT_CHINFO_ERR_IMR=0x%08x\n",
				      MAC_REG_R32(R_AX_BBRPT_CHINFO_ERR_IMR));
			PLTFM_MSG_ERR("R_AX_BBRPT_DFS_ERR_IMR=0x%08x\n",
				      MAC_REG_R32(R_AX_BBRPT_DFS_ERR_IMR));
			PLTFM_MSG_ERR("R_AX_BBRPT_DFS_ERR_ISR=0x%08x\n",
				      MAC_REG_R32(R_AX_BBRPT_DFS_ERR_ISR));
		}
#endif
	}

	if (dmac_err & B_AX_HAXIDMA_ERR_FLAG) {
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			u8 ch = 0;
			u32 val32;

			PLTFM_MSG_ALWAYS("R_AX_HAXI_INIT_CFG1(0x1000)=0x%08x\n",
					 MAC_REG_R32(R_AX_HAXI_INIT_CFG1));
			PLTFM_MSG_ALWAYS("R_AX_HAXI_DMA_STOP1(0x1010)=0x%08x\n",
					 MAC_REG_R32(R_AX_HAXI_DMA_STOP1));
			PLTFM_MSG_ALWAYS("R_AX_HAXI_DMA_BUSY1(0x101C)=0x%08x\n",
					 MAC_REG_R32(R_AX_HAXI_DMA_BUSY1));
			PLTFM_MSG_ALWAYS("R_AX_HAXI_DMA_BUSY3(0x1208)=0x%08x\n",
					 MAC_REG_R32(R_AX_HAXI_DMA_BUSY3));
			PLTFM_MSG_ALWAYS("R_AX_DBG_ERR_FLAG_V1(0x1104)=0x%08x\n",
					 MAC_REG_R32(R_AX_DBG_ERR_FLAG_V1));
			PLTFM_MSG_ALWAYS("R_AX_HAXI_IDCT_MSK(0x10B8)=0x%08x\n",
					 MAC_REG_R32(R_AX_HAXI_IDCT_MSK));
			PLTFM_MSG_ALWAYS("R_AX_HAXI_IDCT(0x10BC)=0x%08x\n",
					 MAC_REG_R32(R_AX_HAXI_IDCT));
			PLTFM_MSG_ALWAYS("R_AX_DLE_EMPTY0(0x8430)=0x%08x\n",
					 MAC_REG_R32(R_AX_DLE_EMPTY0));
			PLTFM_MSG_ALWAYS("R_AX_DLE_EMPTY1(0x8434)=0x%08x\n",
					 MAC_REG_R32(R_AX_DLE_EMPTY1));
			val32 = MAC_REG_R32(R_AX_HDP_DBG_INFO_2_V1);
			PLTFM_MSG_ALWAYS("R_AX_HDP_DBG_INFO_2_V1(0x8888)=0x%08x\n",
					 val32);
			if (GET_FIELD(val32, B_AX_TX_FULL_STS))
				for (ch = MAC_AX_DMA_ACH0; ch < MAC_AX_DMA_CH_NUM; ch++)
					p_ops->get_fc_page_info(adapter, ch);
					PLTFM_MSG_ALWAYS("CH%d FC aval/used: %d/%d\n", ch,
							 (u32)ch_info[ch].aval,
							 (u32)ch_info[ch].used);
			mac_bdinfo_dump(adapter);
		}
#endif
	}
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
#if MAC_AX_PCIE_SUPPORT
		if (adapter->env_info.intf == MAC_AX_INTF_PCIE) {
			u32 check_bits = B_AX_TX_STUCK | B_AX_RX_STUCK | B_AX_PCIE_TXBD_LEN0;

			if (MAC_REG_R32(R_AX_DBG_ERR_FLAG) & check_bits)
				dump_err_status_pciedma_ax(adapter);
		}
#endif //MAC_AX_PCIE_SUPPORT
	}
#endif

}

static void dump_err_status_cmac(struct mac_ax_adapter *adapter, u8 band)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 cmac_err;
	u32 ret;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return;

	PLTFM_MSG_ERR("CMAC Band =0x%02x\n", band);

	cmac_err = MAC_REG_R32(band == MAC_AX_BAND_0 ? R_AX_CMAC_ERR_ISR :
			       R_AX_CMAC_ERR_ISR_C1);
	PLTFM_MSG_ERR("R_AX_CMAC_ERR_ISR =0x%08x\n", cmac_err);

	PLTFM_MSG_ERR("R_AX_CMAC_FUNC_EN =0x%08x\n",
		      MAC_REG_R32(band == MAC_AX_BAND_0 ? R_AX_CMAC_FUNC_EN :
		      R_AX_CMAC_FUNC_EN_C1));
	PLTFM_MSG_ERR("R_AX_CK_EN =0x%08x\n",
		      MAC_REG_R32(band == MAC_AX_BAND_0 ? R_AX_CK_EN :
		      R_AX_CK_EN_C1));

	if (cmac_err & B_AX_SCHEDULE_TOP_ERR_IND) {
		PLTFM_MSG_ERR("R_AX_SCHEDULE_ERR_IMR=0x%08x\n",
			      MAC_REG_R32(band == MAC_AX_BAND_0 ?
			      R_AX_SCHEDULE_ERR_IMR : R_AX_SCHEDULE_ERR_IMR_C1));
		PLTFM_MSG_ERR("R_AX_SCHEDULE_ERR_ISR=0x%04x\n",
			      MAC_REG_R16(band == MAC_AX_BAND_0 ?
			      R_AX_SCHEDULE_ERR_ISR : R_AX_SCHEDULE_ERR_ISR_C1));
	}

	if (cmac_err & B_AX_PTCL_TOP_ERR_IND) {
		PLTFM_MSG_ERR("R_AX_PTCL_IMR0=0x%08x\n",
			      MAC_REG_R32(band == MAC_AX_BAND_0 ?
			      R_AX_PTCL_IMR0 : R_AX_PTCL_IMR0_C1));
		PLTFM_MSG_ERR("R_AX_PTCL_ISR0=0x%08x\n",
			      MAC_REG_R32(band == MAC_AX_BAND_0 ?
			      R_AX_PTCL_ISR0 : R_AX_PTCL_ISR0_C1));
	}

	if (cmac_err & B_AX_DMA_TOP_ERR_IND) {
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			PLTFM_MSG_ERR("R_AX_DLE_CTRL (0xC800)=0x%08x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_DLE_CTRL : R_AX_DLE_CTRL_C1));
			PLTFM_MSG_ERR("0xC828=0x%08x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_RX_INFO_RXSTS : R_AX_RX_INFO_RXSTS_C1));
		}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			PLTFM_MSG_ERR("R_AX_RX_ERR_FLAG (0xC800)=0x%08x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_RX_ERR_FLAG : R_AX_RX_ERR_FLAG_C1));
			PLTFM_MSG_ERR("R_AX_RX_ERR_FLAG_IMR (0xC804)=0x%08x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_RX_ERR_FLAG_IMR : R_AX_RX_ERR_FLAG_IMR_C1));
			PLTFM_MSG_ERR("R_AX_RX_ERR_FLAG (0xC86C)=0x%08x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_TX_ERR_FLAG : R_AX_TX_ERR_FLAG_C1));
			PLTFM_MSG_ERR("R_AX_RX_ERR_FLAG_IMR (0xC870)=0x%08x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_TX_ERR_FLAG_IMR : R_AX_TX_ERR_FLAG_IMR_C1));
		}
#endif
	}

	if (cmac_err & B_AX_PHYINTF_ERR_IND) {
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			PLTFM_MSG_ERR("R_AX_PHYINFO_ERR_IMR=0x%04x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_PHYINFO_ERR_IMR :
				      R_AX_PHYINFO_ERR_IMR_C1));
		}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			PLTFM_MSG_ERR("R_AX_PHYINFO_ERR_ISR=0x%04x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_PHYINFO_ERR_ISR :
				      R_AX_PHYINFO_ERR_ISR_C1));
			PLTFM_MSG_ERR("R_AX_PHYINFO_ERR_IMR=0x%04x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_PHYINFO_ERR_IMR_V1 :
				      R_AX_PHYINFO_ERR_IMR_V1_C1));
		}
#endif
	}

	if (cmac_err & B_AX_TXPWR_CTRL_ERR_IND) {
		PLTFM_MSG_ERR("R_AX_TXPWR_IMR=0x%08x\n",
			      MAC_REG_R32(band == MAC_AX_BAND_0 ?
			      R_AX_TXPWR_IMR : R_AX_TXPWR_IMR_C1));
		PLTFM_MSG_ERR("R_AX_TXPWR_ISR=0x%08x\n",
			      MAC_REG_R32(band == MAC_AX_BAND_0 ?
			      R_AX_TXPWR_ISR : R_AX_TXPWR_ISR_C1));
	}

	if (cmac_err & B_AX_WMAC_RX_ERR_IND) {
		PLTFM_MSG_ERR("R_AX_DBGSEL_TRXPTCL=0x%08x\n",
			      MAC_REG_R32(band == MAC_AX_BAND_0 ?
			      R_AX_DBGSEL_TRXPTCL : R_AX_DBGSEL_TRXPTCL_C1));
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			PLTFM_MSG_ERR("R_AX_PHYINFO_ERR_IMR=0x%04x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_PHYINFO_ERR_IMR :
				      R_AX_PHYINFO_ERR_IMR_C1));
			PLTFM_MSG_ERR("R_AX_RMAC_ERR_ISR=0x%04x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_RMAC_ERR_ISR :
				      R_AX_RMAC_ERR_ISR_C1));
		}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			PLTFM_MSG_ERR("R_AX_PHYINFO_ERR_ISR=0x%04x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_PHYINFO_ERR_ISR :
				      R_AX_PHYINFO_ERR_ISR_C1));
			PLTFM_MSG_ERR("R_AX_PHYINFO_ERR_IMR=0x%04x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_PHYINFO_ERR_IMR_V1 :
				      R_AX_PHYINFO_ERR_IMR_V1_C1));
			PLTFM_MSG_ERR("R_AX_RX_ERR_ISR=0x%04x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_RX_ERR_ISR :
				      R_AX_RX_ERR_ISR_C1));
			PLTFM_MSG_ERR("R_AX_RX_ERR_IMR=0x%04x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_RX_ERR_IMR :
				      R_AX_RX_ERR_IMR_C1));
		}
#endif
	}

	if (cmac_err & B_AX_WMAC_TX_ERR_IND) {
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			PLTFM_MSG_ERR("R_AX_TMAC_ERR_IMR_ISR=0x%08x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_TMAC_ERR_IMR_ISR :
				      R_AX_TMAC_ERR_IMR_ISR_C1));
		}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D)) {
			PLTFM_MSG_ERR("R_AX_TRXPTCL_ERROR_INDICA=0x%08x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_TRXPTCL_ERROR_INDICA :
				      R_AX_TRXPTCL_ERROR_INDICA_C1));
			PLTFM_MSG_ERR("R_AX_TRXPTCL_ERROR_INDICA_MASK=0x%08x\n",
				      MAC_REG_R32(band == MAC_AX_BAND_0 ?
				      R_AX_TRXPTCL_ERROR_INDICA_MASK :
				      R_AX_TRXPTCL_ERROR_INDICA_MASK_C1));
		}
#endif
		PLTFM_MSG_ERR("R_AX_DBGSEL_TRXPTCL=0x%08x\n",
			      MAC_REG_R32(band == MAC_AX_BAND_0 ?
			      R_AX_DBGSEL_TRXPTCL : R_AX_DBGSEL_TRXPTCL_C1));
	}

	cmac_err = MAC_REG_R32(band == MAC_AX_BAND_1 ? R_AX_CMAC_ERR_IMR_C1 :
			       R_AX_CMAC_ERR_IMR);
	PLTFM_MSG_ALWAYS("B%d R_AX_CMAC_ERR_IMR=0x%x\n", band, cmac_err);
}

u32 mac_dump_ser_cnt(struct mac_ax_adapter *adapter, struct mac_ser_status *status)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val32;

	val32 = MAC_REG_R32(R_AX_SER_DBG_INFO);
	status->l0_cnt = GET_FIELD(val32, B_AX_SER_L0_COUNTER);
	status->l1_cnt = GET_FIELD(val32, B_AX_SER_L1_COUNTER);
	status->l0_pro_event = GET_FIELD(val32, B_AX_SER_L0_PROMOTE_L1_EVENT);
	status->rmac_ppdu_hang_cnt = GET_FIELD(val32, B_AX_RMAC_PPDU_HANG_CNT);

	return MACSUCCESS;
}

static void dump_cmac_cr(struct mac_ax_adapter *adapter, u8 band)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 base_addr, max_addr, i;
	u32 ret;

	ret = check_mac_en(adapter, band, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return;

	PLTFM_MSG_ERR("Dump CMAC band %d\n", band);
	PLTFM_MSG_ERR("CMAC common\n");
	base_addr = (band == 0) ? CMAC_AX_COMMON_BASE_ADDR : CMAC_AX_COMMON_BASE_ADDR_C1;
	max_addr = CMAC_AX_COMMON_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x186%05x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("Scheduler\n");
	base_addr = (band == 0) ? CMAC_AX_SCH_BASE_ADDR : CMAC_AX_SCH_BASE_ADDR_C1;
	max_addr = CMAC_AX_SCH_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x186%05x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("PTCL\n");
	base_addr = (band == 0) ? CMAC_AX_PTCL_BASE_ADDR : CMAC_AX_PTCL_BASE_ADDR_C1;
	max_addr = CMAC_AX_PTCL_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x186%05x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("CDMA\n");
	base_addr = (band == 0) ? CMAC_AX_CDMA_BASE_ADDR : CMAC_AX_CDMA_BASE_ADDR_C1;
	max_addr = CMAC_AX_CDMA_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x186%05x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("TMAC\n");
	base_addr = (band == 0) ? CMAC_AX_TMAC_BASE_ADDR : CMAC_AX_TMAC_BASE_ADDR_C1;
	max_addr = CMAC_AX_TMAC_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x186%05x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("TRXPTCL\n");
	base_addr = (band == 0) ? CMAC_AX_TRXPTCL_BASE_ADDR : CMAC_AX_TRXPTCL_BASE_ADDR_C1;
	max_addr = CMAC_AX_TRXPTCL_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x186%05x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("RMAC\n");
	base_addr = (band == 0) ? CMAC_AX_RMAC_BASE_ADDR : CMAC_AX_RMAC_BASE_ADDR_C1;
	max_addr = CMAC_AX_RMAC_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x186%05x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("PWR\n");
	base_addr = (band == 0) ? CMAC_AX_PWR_BASE_ADDR : CMAC_AX_PWR_BASE_ADDR_C1;
	max_addr = CMAC_AX_PWR_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x186%05x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("BTCOEX\n");
	base_addr = (band == 0) ? CMAC_AX_BTCOEX_BASE_ADDR : CMAC_AX_BTCOEX_BASE_ADDR_C1;
	max_addr = CMAC_AX_BTCOEX_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x186%05x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));
}

static void dump_dmac_cr(struct mac_ax_adapter *adapter)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 base_addr, max_addr, i;

	PLTFM_MSG_ERR("Dump DMAC\n");
	PLTFM_MSG_ERR("TOP_OFF\n");
	base_addr = DMAC_AX_TOP_OFF_BASE_ADDR;
	max_addr = DMAC_AX_TOP_OFF_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("WL_PON\n");
	base_addr = DMAC_AX_WL_PON_BASE_ADDR;
	max_addr = DMAC_AX_WL_PON_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("Common\n");
	base_addr = DMAC_AX_COMMON_BASE_ADDR;
	max_addr = DMAC_AX_COMMON_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("Dispatcher\n");
	base_addr = DMAC_AX_DISPATCHER_BASE_ADDR;
	max_addr = DMAC_AX_DISPATCHER_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("WDE\n");
	base_addr = DMAC_AX_WDE_BASE_ADDR;
	max_addr = DMAC_AX_WDE_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("PLE\n");
	base_addr = DMAC_AX_PLE_BASE_ADDR;
	max_addr = DMAC_AX_PLE_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("WDRLS\n");
	base_addr = DMAC_AX_WDRLS_BASE_ADDR;
	max_addr = DMAC_AX_WDRLS_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("BBRPT\n");
	base_addr = DMAC_AX_BBRPT_BASE_ADDR;
	max_addr = DMAC_AX_BBRPT_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("CPUIO\n");
	base_addr = DMAC_AX_CPUIO_BASE_ADDR;
	max_addr = DMAC_AX_CPUIO_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("PLTIN\n");
	base_addr = DMAC_AX_PKTIN_BASE_ADDR;
	max_addr = DMAC_AX_PKTIN_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("MPDU Processor\n");
	base_addr = DMAC_AX_MPDU_BASE_ADDR;
	max_addr = DMAC_AX_MPDU_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("SEC ENG\n");
	base_addr = DMAC_AX_SEC_BASE_ADDR;
	max_addr = DMAC_AX_SEC_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("STA scheduler\n");
	base_addr = DMAC_AX_SS_BASE_ADDR;
	max_addr = DMAC_AX_SS_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("Tx packet control\n");
	base_addr = DMAC_AX_TXPKTCTL_BASE_ADDR;
	max_addr = DMAC_AX_TXPKTCTL_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("HCI Common\n");
	base_addr = DMAC_AX_HCI_BASE_ADDR;
	max_addr = DMAC_AX_HCI_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));

	PLTFM_MSG_ERR("HAXIDMA\n");
	base_addr = DMAC_BE_HAXIDMA_BASE_ADDR;
	max_addr = DMAC_BE_HAXIDMA_MAX_ADDR;
	for (i = 0x0; i <= max_addr; i += 4)
		PLTFM_MSG_ERR("0x1860%04x = 0x%08x\n", base_addr + i, MAC_REG_R32(base_addr + i));
}

static void dump_sram(struct mac_ax_adapter *adapter, u32 sram_sel)
{
	u32 mem_size, ret;

	switch (sram_sel) {
	case MAC_AX_MEM_AXIDMA:
		PLTFM_MSG_ERR("AXIDMA dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_AXIDMA);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_AXIDMA,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_SHARED_BUF:
		PLTFM_MSG_ERR("Shared memory dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_SHARED_BUF);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_SHARED_BUF,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_DMAC_TBL:
		PLTFM_MSG_ERR("DMAC table dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_DMAC_TBL);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_DMAC_TBL,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_STA_SCHED:
		PLTFM_MSG_ERR("STA Scheduler dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_STA_SCHED);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_STA_SCHED,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_RXPLD_FLTR_CAM:
		PLTFM_MSG_ERR("Rx payload filter dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_RXPLD_FLTR_CAM);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_RXPLD_FLTR_CAM,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_SECURITY_CAM:
		PLTFM_MSG_ERR("Secuirty CAM dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_SECURITY_CAM);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_SECURITY_CAM,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_CMAC_TBL:
		PLTFM_MSG_ERR("CMAC table dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_CMAC_TBL);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_CMAC_TBL,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_ADDR_CAM:
		PLTFM_MSG_ERR("Address CAM dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_ADDR_CAM);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_ADDR_CAM,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_BA_CAM:
		PLTFM_MSG_ERR("BA CAM dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_BA_CAM);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_BA_CAM,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_BCN_IE_CAM0:
		PLTFM_MSG_ERR("BCN IE CAM0 dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_BCN_IE_CAM0);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_BCN_IE_CAM0,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_BCN_IE_CAM1:
		PLTFM_MSG_ERR("BCN IE CAM1 start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_BCN_IE_CAM1);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_BCN_IE_CAM1,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_TXD_FIFO_0:
		PLTFM_MSG_ERR("TXD FIFO 0 dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_TXD_FIFO_0);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_TXD_FIFO_0,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_TXD_FIFO_1:
		PLTFM_MSG_ERR("TXD FIFO 1 dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_TXD_FIFO_1);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_TXD_FIFO_1,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_RXD_FIFO_0:
		PLTFM_MSG_ERR("RXD FIFO 0 dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_RXD_FIFO_0);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_RXD_FIFO_0,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_RXD_FIFO_1:
		PLTFM_MSG_ERR("RXD FIFO 1 dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_RXD_FIFO_1);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_RXD_FIFO_1,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_DMA_TXFIFO_0:
		PLTFM_MSG_ERR("DMA TXFIFO 0 dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_DMA_TXFIFO_0);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_DMA_TXFIFO_0,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_DMA_TXFIFO_1:
		PLTFM_MSG_ERR("DMA TXFIFO 1 dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_DMA_TXFIFO_1);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_DMA_TXFIFO_1,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_DMA_RXFIFO_0:
		PLTFM_MSG_ERR("DMA RXFIFO 0 dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_DMA_RXFIFO_0);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_DMA_RXFIFO_0,
					   0x0, NULL, mem_size, 0);
		break;
	case MAC_AX_MEM_DMA_RXFIFO_1:
		PLTFM_MSG_ERR("DMA RXFIFO 1 dump start\n");
		mem_size = mac_get_mem_size(adapter, MAC_AX_MEM_DMA_RXFIFO_1);
		if (mem_size != 0)
			ret = mac_mem_dump(adapter, MAC_AX_MEM_DMA_RXFIFO_1,
					   0x0, NULL, mem_size, 0);
		break;
	default:
		PLTFM_MSG_ERR("This SRAM sel is not supported\n");
		break;
	}
}

u32 mac_set_ser_case(struct mac_ax_adapter *adapter,
		     struct mac_ser_dbg_info *ser_dbg_info)
{
	u32 ret = 0;
	u8 *buf;
	struct h2c_info h2c_info = { 0 };
	struct fwcmd_ser_dbg_case_set *ser_case;

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_ser_dbg_case_set);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_SER;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_SER_DBG_CASE_SET;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 1;

	buf = (u8 *)PLTFM_MALLOC(h2c_info.content_len);
	if (!buf) {
		PLTFM_MSG_ERR("%s malloc h2c error\n", __func__);
		return MACNPTR;
	}
	ser_case = (struct fwcmd_ser_dbg_case_set *)buf;
	ser_case->dword0 = ser_dbg_info->ser_cmac_flag;
	ser_case->dword1 = ser_dbg_info->ser_dmac_flag;
	ser_case->dword2 = ser_dbg_info->address[0];
	ser_case->dword3 = ser_dbg_info->submodule_flag[0];
	ser_case->dword4 = ser_dbg_info->address[1];
	ser_case->dword5 = ser_dbg_info->submodule_flag[1];
	ser_case->dword6 = ser_dbg_info->address[2];
	ser_case->dword4 = ser_dbg_info->submodule_flag[2];

	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = mac_h2c_common(adapter, &h2c_info, (u32 *)ser_case);
		if (ret)
			goto fail;
	} else {
		ret = MACFWNONRDY;
		goto fail;
	}

	ret = MACSUCCESS;
fail:
	PLTFM_FREE(buf, h2c_info.content_len);

	return ret;
}
#endif /* #if MAC_AX_FEATURE_DBGPKG */

#if MAC_SELF_DIAG_INFO
u32 chk_serl1_freq(struct mac_ax_adapter *adapter)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ser_dbg_cnt = 0, dmac_err_cnt = 0, trigger_l1_maunal_cnt = 0;
	u32 hour, dmac_err_freq;

	hour = MAC_REG_R32(R_AX_FREERUN_CNT_HIGH);
	if (hour == 0)
		hour = 1;

	ser_dbg_cnt = MAC_REG_R32(R_AX_SER_DBG_INFO);
	dmac_err_cnt = GET_FIELD(ser_dbg_cnt, B_AX_SER_L1_COUNTER);
	trigger_l1_maunal_cnt = adapter->ser_info.ser_dbg_info.trigger_l1_maunal_cnt;

	/* Clear trigger count for self diagnosis */
	if (dmac_err_cnt >= trigger_l1_maunal_cnt)
		dmac_err_cnt -= trigger_l1_maunal_cnt;
	else
		dmac_err_cnt = 0;

	dmac_err_freq = dmac_err_cnt / hour;
	if (dmac_err_freq > 0)
		return MACNOITEM;

	return MACSUCCESS;
}

u32 chk_serl0_freq(struct mac_ax_adapter *adapter)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ser_dbg_cnt = 0, cmac0_err_cnt = 0, cmac1_err_cnt = 0, bb_only_err_cnt = 0;
	u32 cmac0_err_freq, cmac1_err_freq, hour, min;
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT ||\
	MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
	u32 cmac_err_diff, cmac_err_freq;
#endif
	hour = MAC_REG_R32(R_AX_FREERUN_CNT_HIGH);
	min = (MAC_REG_R32(R_AX_FREERUN_CNT_LOW) / (US_TO_SEC * SEC_TO_MIN)) + (hour * 60);

	ser_dbg_cnt = MAC_REG_R32(R_AX_SER_DBG_INFO);
	cmac0_err_cnt = GET_FIELD(ser_dbg_cnt, B_AX_SER_L0_COUNTER);
	cmac1_err_cnt = GET_FIELD(ser_dbg_cnt, CMAC1_COUNTER);
	bb_only_err_cnt = cmac1_err_cnt;

#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT ||\
	MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
		if (cmac0_err_cnt >= bb_only_err_cnt)
			cmac_err_diff = cmac0_err_cnt - bb_only_err_cnt;
		else
			cmac_err_diff = cmac0_err_cnt + 0xFF - bb_only_err_cnt;

		if (min != 0)
			cmac_err_freq = cmac_err_diff / min;
		else
			cmac_err_freq = cmac_err_diff;

		if (cmac_err_freq > 2) {
			PLTFM_MSG_ALWAYS("== 8852B IC More two L0 per min ==");
			PLTFM_MSG_ALWAYS("More than two SER L0 per minute\n");
			PLTFM_MSG_ALWAYS("cmac_err_freq = %d\n", cmac_err_freq);
			PLTFM_MSG_ALWAYS("R_AX_SER_DBG_INFO =0x%08x\n",
					 MAC_REG_R32(R_AX_SER_DBG_INFO));
			return MACNOITEM;
		}
		//bypass ser_l1_fsm and ser_l1_pos check for 52A / 52B / 51B / 52BT
		return MACSUCCESS;
	}
#endif
	if (min != 0) {
		cmac0_err_freq = cmac0_err_cnt / min;
		cmac1_err_freq = cmac1_err_cnt / min;
	} else {
		cmac0_err_freq = cmac0_err_cnt;
		cmac1_err_freq = cmac1_err_cnt;
	}

	if (cmac0_err_freq > 1 || cmac1_err_freq > 1) {
		PLTFM_MSG_ALWAYS("== Non 8852B IC More two L0 per min ==");
		PLTFM_MSG_ALWAYS("cmac0_err_freq = %d\n", cmac0_err_freq);
		PLTFM_MSG_ALWAYS("cmac1_err_freq = %d\n", cmac1_err_freq);
		PLTFM_MSG_ALWAYS("R_AX_SER_DBG_INFO =0x%08x\n",
				 MAC_REG_R32(R_AX_SER_DBG_INFO));
		return MACNOITEM;
	}
	return MACSUCCESS;
}

u32 chk_serl1_fsm(struct mac_ax_adapter *adapter, u32 *ser_err_bitmap)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 ser_dbg_cnt = 0, dmac_err_cnt = 0, dmac_fsm_info = 0;
	u8 ser_l1_fsm, ser_l1_pos;

	ser_dbg_cnt = MAC_REG_R32(R_AX_SER_DBG_INFO);
	dmac_err_cnt = GET_FIELD(ser_dbg_cnt, B_AX_SER_L1_COUNTER);

#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT ||\
	MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
		return MACSUCCESS;
	}
#endif
	dmac_fsm_info = MAC_REG_R32(R_AX_UDM3);
	ser_l1_fsm = GET_FIELD(dmac_fsm_info, SER_L1_FSM);
	ser_l1_pos = GET_FIELD(dmac_fsm_info, SER_L1_FUN_POS);

	if (dmac_err_cnt > 0) {
		if (ser_l1_fsm != 0) {
			PLTFM_MSG_ERR("SER L1 does not return to initial state\n");
			*ser_err_bitmap |= SER_SELF_DIAG_SER_L1_FSM_ERR;
		}
		if (ser_l1_pos != 0xff) {
			PLTFM_MSG_ERR("SER L1 position does not leave recovery flow\n");
			*ser_err_bitmap |= SER_SELF_DIAG_SER_L1_POS_ERR;
		}
		return MACNOITEM;
	}
	return MACSUCCESS;
}

u32 get_ser_freq(struct mac_ax_adapter *adapter, u32 *total_check_num, u32 *ser_err_bitmap)
{
	u32 cnt = 0, ret = 0;

	*total_check_num = SER_SELF_DIAGNOSIS_ITEM_NUM;
	*ser_err_bitmap = 0;

	ret = chk_serl1_freq(adapter);
	if (ret != MACSUCCESS) {
		*ser_err_bitmap |= SER_SELF_DIAG_SER_L1_FREQ_ERR;
		cnt += 1;
	}

	ret = chk_serl0_freq(adapter);
	if (ret != MACSUCCESS) {
		*ser_err_bitmap |= SER_SELF_DIAG_SER_L0_FREQ_ERR;
		cnt += 1;
	}

	ret = chk_serl1_fsm(adapter, ser_err_bitmap);
	if (ret != MACSUCCESS) {
		if (*ser_err_bitmap & SER_SELF_DIAG_SER_L1_FSM_ERR)
			cnt += 1;
		if (*ser_err_bitmap & SER_SELF_DIAG_SER_L1_POS_ERR)
			cnt += 1;
	}
	return cnt;
}

#endif /* #if MAC_SELF_DIAG_INFO */

u32 mac_trigger_cmac_err(struct mac_ax_adapter *adapter)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	struct mac_ax_ser_info *ser_info = &adapter->ser_info;
	struct mac_ser_dbg_info *ser_dbg_info = &ser_info->ser_dbg_info;
	u8 val8;
	u16 val16;
	u32 ret;

	PLTFM_MSG_ALWAYS("===== Trigger SER L0 Manual =====");
	ret = check_mac_en(adapter, MAC_AX_BAND_0, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	// Increase trigger count for self diagnosis
	ser_dbg_info->trigger_l0_maunal_cnt += 1;

	val8 = MAC_REG_R8(R_AX_CMAC_FUNC_EN);
	MAC_REG_W8(R_AX_CMAC_FUNC_EN, val8 & (~B_AX_TMAC_EN));
	PLTFM_DELAY_MS(1);
	MAC_REG_W8(R_AX_CMAC_FUNC_EN, val8);

	val16 = MAC_REG_R16(R_AX_PTCL_IMR0) | B_AX_F2PCMD_EMPTY_ERR_INT_EN;
	MAC_REG_W16(R_AX_PTCL_IMR0, val16);
	MAC_REG_W16(R_AX_PTCL_IMR0, val16 & ~B_AX_F2PCMD_EMPTY_ERR_INT_EN);

	return MACSUCCESS;
}

u32 mac_trigger_cmac1_err(struct mac_ax_adapter *adapter)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u8 val8;
	u16 val16;
	u32 ret;

	ret = check_mac_en(adapter, MAC_AX_BAND_1, MAC_AX_CMAC_SEL);
	if (ret != MACSUCCESS)
		return ret;

	val8 = MAC_REG_R8(R_AX_CMAC_FUNC_EN_C1);
	MAC_REG_W8(R_AX_CMAC_FUNC_EN_C1, val8 & (~B_AX_TMAC_EN));
	PLTFM_DELAY_MS(1);
	MAC_REG_W8(R_AX_CMAC_FUNC_EN_C1, val8);

	val16 = MAC_REG_R16(R_AX_PTCL_IMR0_C1) | B_AX_F2PCMD_EMPTY_ERR_INT_EN;
	MAC_REG_W16(R_AX_PTCL_IMR0_C1, val16);
	MAC_REG_W16(R_AX_PTCL_IMR0_C1, val16 & ~B_AX_F2PCMD_EMPTY_ERR_INT_EN);

	return MACSUCCESS;
}

u32 mac_trigger_dmac_err(struct mac_ax_adapter *adapter)
{
	struct cpuio_buf_req_t buf_req;
	struct cpuio_ctrl_t ctrl_para;
	struct mac_ax_ser_info *ser_info = &adapter->ser_info;
	struct mac_ser_dbg_info *ser_dbg_info = &ser_info->ser_dbg_info;
	u32 ret;

	// Increase trigger count for self diagnosis
	ser_dbg_info->trigger_l1_maunal_cnt += 1;
	// Use CPUIO to enqueue packet.WD
	buf_req.len = 0x20;
	ret = mac_dle_buf_req_wd(adapter, &buf_req);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]WDE DLE buf req\n");
		return ret;
	}

	// Enqueue two pkt_id, but only has one pkt_id.
	PLTFM_MEMSET((void *)&ctrl_para, 0, sizeof(ctrl_para));
	ctrl_para.cmd_type = CPUIO_OP_CMD_ENQ_TO_HEAD;
	ctrl_para.start_pktid = buf_req.pktid;
	ctrl_para.end_pktid = buf_req.pktid;
	ctrl_para.pkt_num = 1;
	ctrl_para.dst_pid = WDE_DLE_PORT_ID_WDRLS;
	ctrl_para.dst_qid = 4;
	ret = mac_set_cpuio_wd(adapter, &ctrl_para);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]WDE DLE enqueue to head\n");
		return ret;
	}
	return MACSUCCESS;
}

u32 mac_set_err_status(struct mac_ax_adapter *adapter,
		       enum mac_ax_err_info err)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 cnt = MAC_SET_ERR_DLY_CNT;
	u32 ret = MACSUCCESS;

	if (err > MAC_AX_SET_ERR_MAX) {
		PLTFM_MSG_ERR("Bad set-err-status value\n");
		return MACFUNCINPUT;
	}

	PLTFM_MUTEX_LOCK(&adapter->lock_info.err_set_lock);

	while (--cnt) {
		if (!MAC_REG_R32(R_AX_HALT_H2C_CTRL))
			break;
		PLTFM_DELAY_US(MAC_SET_ERR_DLY_US);
	}
	if (!cnt) {
		PLTFM_MSG_ERR("FW does not receive previous msg\n");
		ret = MACPOLLTO;
		goto end;
	}

	if (err == MAC_AX_ERR_L1_DISABLE_EN)
		adapter->sm.fw_rst = MAC_AX_FW_RESET_RECV_DONE;

	MAC_REG_W32(R_AX_HALT_H2C, err);
#if MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || MAC_AX_8852BT_SUPPORT || MAC_AX_8851B_SUPPORT
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B)) {
		MAC_REG_W32(R_AX_HALT_H2C_CTRL, B_AX_HALT_H2C_TRIGGER);
	}
#endif
#if MAC_AX_8852C_SUPPORT || MAC_AX_8852D_SUPPORT || MAC_AX_8192XB_SUPPORT
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852D) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB)) {
		if (err != MAC_AX_ERR_L1_DISABLE_EN &&
		    err != MAC_AX_ERR_L1_RCVY_EN) {
			MAC_REG_W32(R_AX_HALT_H2C_CTRL, B_AX_HALT_H2C_TRIGGER);
		}
	}
#endif

end:
	PLTFM_MUTEX_UNLOCK(&adapter->lock_info.err_set_lock);
	return ret;
}

u32 mac_get_err_status(struct mac_ax_adapter *adapter,
		       enum mac_ax_err_info *err)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 cnt = MAC_SET_ERR_DLY_CNT;
	u32 ret = MACSUCCESS;
	u32 error_scenario = 0;
#if MAC_AX_PLDR_DIAGNOSE_EN && MAC_AX_FEATURE_DBGPKG
	u8 is_dbg_dump = 0;
	u32 dbg_ret = MACSUCCESS, dump_ret = MACSUCCESS;
#endif

	PLTFM_MUTEX_LOCK(&adapter->lock_info.err_get_lock);
	adapter->sm.l2_st = MAC_AX_L2_DIS;

	while (--cnt) {
		if (MAC_REG_R32(R_AX_HALT_C2H_CTRL))
			break;
		PLTFM_DELAY_US(MAC_SET_ERR_DLY_US);
	}
	if (!cnt) {
		PLTFM_MSG_ERR("Polling FW err status fail\n");
		ret = MACPOLLTO;
		goto end;
	}

	*err = (enum mac_ax_err_info)MAC_REG_R32(R_AX_HALT_C2H);
	MAC_REG_W32(R_AX_HALT_C2H_CTRL, 0);
	switch (*err) {
	case MAC_AX_ERR_L1_ERR_DMAC:
	case MAC_AX_ERR_L0_PROMOTE_TO_L1:
	case HALT_C2H_L1_DBG_MODE:
		adapter->sm.fw_rst = MAC_AX_FW_RESET_RECV;
		break;
	case MAC_AX_ERR_L1_RESET_DISABLE_DMAC_DONE:
		adapter->sm.fw_rst = MAC_AX_FW_RESET_PROCESS;
		break;
	case MAC_AX_ERR_L1_RESET_RECOVERY_DONE:
		adapter->sm.fw_rst = MAC_AX_FW_RESET_IDLE;
		break;
	default:
		break;
	}

	/* Decode the error status from halc2h */
	error_scenario = (*err) >> DBG_SENARIO_SH;
	if (!(MAC_REG_R32(R_AX_UDM0) & B_AX_UDM0_DBG_MODE_CTRL)) {
		if (error_scenario == CPU_EXCEPTION)
			*err = MAC_AX_ERR_CPU_EXCEPTION;
		else if (error_scenario == ASSERTION)
			*err = MAC_AX_ERR_ASSERTION;
		else if (error_scenario == RXI300_ERROR)
			*err = MAC_AX_ERR_RXI300;
	}
#if MAC_AX_FEATURE_DBGPKG
#if MAC_AX_PLDR_DIAGNOSE_EN
	if (error_scenario == RXI300_ERROR ||
	    error_scenario == CPU_EXCEPTION ||
	    error_scenario == ASSERTION)
		is_dbg_dump = 1;
	if (is_dbg_dump) {
		dbg_ret = mac_dbgcmd_init(adapter);
		if (dbg_ret == MACSUCCESS) {
			dump_ret = mac_pldr_diagnose_dump(adapter);
			if (dump_ret != MACSUCCESS)
				PLTFM_MSG_ERR("[ERR] %s: mac_pldr_diagnose_dump fail, %d\n",
					      __func__, dump_ret);
			mac_dbgcmd_flush(adapter);
			mac_dbgcmd_deinit(adapter);
		} else if (dbg_ret != MACSUCCESS) {
			PLTFM_MSG_ERR("[ERR] %s: dbgcmd_init fail, %d, skip dump\n",
				      __func__, dump_ret);
		}
	}
#endif /* #if MAC_AX_PLDR_DIAGNOSE_EN */
	//3  3. Execute Recode Normal Debug Register
	fw_st_dbg_dump(adapter);
	mac_dump_err_status(adapter, *err);
	if (*err == MAC_AX_ERR_L0_ERR_CMAC0 ||
	    *err == MAC_AX_ERR_L0_ERR_CMAC1) {
		pltfm_dbg_dump(adapter);
	}
#endif /* #if MAC_AX_FEATURE_DBGPKG */
	//3 4. Execute Recode Share memory debug information
	if (MAC_REG_R32(R_AX_UDM0) & B_AX_UDM0_DBG_MODE_CTRL) {
		/* if debug mode =1 , dump share buffer */
		if (error_scenario) {
			*err = (enum mac_ax_err_info)MAC_AX_DUMP_SHAREBUFF_INDICATOR;
			//notify phl to print share buffer
		}
	}
#if MAC_AX_FEATURE_DBGPKG
	//3 5.  dump detailed log for further debug usage.
	mac_dbg_log_dump(adapter);
#endif /* #if MAC_AX_FEATURE_DBGPKG */
	MAC_REG_W32(R_AX_HALT_C2H, 0);
end:
	adapter->sm.l2_st = MAC_AX_L2_EN;
	PLTFM_MUTEX_UNLOCK(&adapter->lock_info.err_get_lock);
	return ret;
}

u32 mac_lv1_rcvy(struct mac_ax_adapter *adapter, enum mac_ax_lv1_rcvy_step step)
{
	u32 ret = MACSUCCESS;
#if MAC_AX_PCIE_SUPPORT
	u8 val8;
#endif

	switch (step) {
	case MAC_AX_LV1_RCVY_STEP_1:
		if (adapter->sm.fw_rst != MAC_AX_FW_RESET_RECV) {
			PLTFM_MSG_ERR("The rst-flow state is wrong\n");
			return MACPROCERR;
		}
#if MAC_AX_PCIE_SUPPORT
	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
		if (adapter->env_info.intf == MAC_AX_INTF_PCIE) {
			val8 = 0;
			ret = lv1rst_stop_dma_pcie(adapter, val8);
			if (ret) {
				PLTFM_MSG_ERR("lv1 rcvy pcie stop dma fail\n");
				return ret;
			}
		}
	} else {
		// 52C PCIE Stop flow made in FW
	}
#endif

#if MAC_AX_USB_SUPPORT
		if (adapter->env_info.intf == MAC_AX_INTF_USB) {
			ret = usb_flush_mode(adapter, MAC_AX_FUNC_EN);
			if (ret) {
				PLTFM_MSG_ERR("lv1 rcvy USB flush mode fail\n");
				return ret;
			}
			PLTFM_DELAY_MS(30);
		}
#endif
		break;

	case MAC_AX_LV1_RCVY_STEP_2:
		if (adapter->sm.fw_rst != MAC_AX_FW_RESET_PROCESS) {
			PLTFM_MSG_ERR("The rst-flow state is wrong\n");
			return MACPROCERR;
		}
#if MAC_AX_PCIE_SUPPORT
		if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
		    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
			if (adapter->env_info.intf == MAC_AX_INTF_PCIE) {
				val8 = 0;
				ret = lv1rst_start_dma_pcie(adapter, val8);
				if (ret) {
					PLTFM_MSG_ERR("lv1 rcvy pcie start dma fail\n");
					return ret;
				}
			}
		} else {
			// 52C PCIE Stop flow made in FW
		}
#endif

#if MAC_AX_USB_SUPPORT
		if (adapter->env_info.intf == MAC_AX_INTF_USB) {
			ret = 0;
			ret = usb_flush_mode(adapter, MAC_AX_FUNC_DIS);
			if (ret) {
				PLTFM_MSG_ERR("lv1 rcvy USB norm mode fail\n");
				return ret;
			}
		}
#endif
		break;

	default:
		return MACLV1STEPERR;
	}

	return ret;
}

u32 mac_err_imr_ctrl(struct mac_ax_adapter *adapter, enum mac_ax_func_sw sw)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 v32_dmac, v32_cmac0, v32_cmac1;
	u32 ret = MACSUCCESS;
	u8 is_dbcc;

	v32_dmac = sw != MAC_AX_FUNC_DIS ? DMAC_ERR_IMR_EN : DMAC_ERR_IMR_DIS;
	v32_cmac0 = sw != MAC_AX_FUNC_DIS ? CMAC0_ERR_IMR_EN : CMAC0_ERR_IMR_DIS;
	v32_cmac1 = sw != MAC_AX_FUNC_DIS ? CMAC1_ERR_IMR_EN : CMAC1_ERR_IMR_DIS;
	is_dbcc = is_curr_dbcc(adapter);

#if MAC_AX_FW_REG_OFLD
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		ret = MAC_REG_W_OFLD((u16)R_AX_DMAC_ERR_IMR, DMAC_ERR_IMR_MASK,
				     v32_dmac, 0);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("dmac err imr w ofld fail\n");
			return ret;
		}

		ret = MAC_REG_W_OFLD((u16)R_AX_CMAC_ERR_IMR, CMAC0_ERR_IMR_MASK,
				     v32_cmac0, (is_dbcc ? 0 : 1));
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("cmac0 err imr w ofld fail\n");
			return ret;
		}

		if (is_dbcc) {
			ret = MAC_REG_W_OFLD((u16)R_AX_CMAC_ERR_IMR_C1,
					     CMAC1_ERR_IMR_MASK,
					     v32_cmac1, 1);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("cmac1 err imr w ofld fail\n");
				return ret;
			}
		}

		return ret;
	}
#endif

	MAC_REG_W32(R_AX_DMAC_ERR_IMR, v32_dmac);
	MAC_REG_W32(R_AX_CMAC_ERR_IMR, v32_cmac0);

	if (is_dbcc)
		MAC_REG_W32(R_AX_CMAC_ERR_IMR_C1, v32_cmac1);

	return ret;
}

u32 mac_ser_ctrl(struct mac_ax_adapter *adapter, enum mac_ax_func_sw sw)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	enum mac_ax_err_info err_info;
	u32 val32, ret, cnt;

	if (sw == MAC_AX_FUNC_EN) {
		err_info = MAC_AX_ERR_L1_RCVY_START_REQ;
		adapter->sm.ser_ctrl_st = MAC_AX_SER_CTRL_SRT;
	} else if (sw == MAC_AX_FUNC_DIS) {
		err_info = MAC_AX_ERR_L1_RCVY_STOP_REQ;
		adapter->sm.ser_ctrl_st = MAC_AX_SER_CTRL_STOP;
	} else if (sw == MAC_AX_FUNC_WO_RECVY_DIS) {
		err_info = MAC_AX_ERR_L1_RCVY_STOP_WO_RECVY_REQ;
		adapter->sm.ser_ctrl_st = MAC_AX_SER_CTRL_SRT;
	} else if (sw == MAC_AX_FUNC_WO_RECVY_EN) {
		err_info = MAC_AX_ERR_L1_RCVY_START_WO_RECVY_REQ;
		adapter->sm.ser_ctrl_st = MAC_AX_SER_CTRL_STOP;
	} else {
		adapter->sm.ser_ctrl_st = MAC_AX_SER_CTRL_ERR;
		PLTFM_MSG_ERR("[ERR]SER ctrl input err %d\n", sw);
		return MACFUNCINPUT;
	}

	ret = mac_set_err_status(adapter, err_info);
	if (ret != MACSUCCESS) {
		adapter->sm.ser_ctrl_st = MAC_AX_SER_CTRL_ERR;
		PLTFM_MSG_ERR("[ERR]set err for stop ser %d\n", ret);
		return ret;
	}

	cnt = MAC_SET_ERR_DLY_CNT;
	while (cnt) {
		val32 = MAC_REG_R32(R_AX_HALT_H2C_CTRL);
		if (!(val32 & B_AX_HALT_H2C_TRIGGER))
			break;
		PLTFM_DELAY_US(MAC_SET_ERR_DLY_US);
		cnt--;
	}

	if (!cnt) {
		adapter->sm.ser_ctrl_st = MAC_AX_SER_CTRL_ERR;
		PLTFM_MSG_ERR("[ERR]FW not handle haltH2C req\n");
		ret = MACPOLLTO;
		return ret;
	}

	if (sw == MAC_AX_FUNC_EN || sw == MAC_AX_FUNC_WO_RECVY_DIS ||
	    sw == MAC_AX_FUNC_WO_RECVY_EN)
		return MACSUCCESS;

	cnt = MAC_SER_STOP_DLY_CNT;
	while (cnt) {
		PLTFM_DELAY_US(MAC_SER_STOP_DLY_US);
		val32 = MAC_REG_R32(R_AX_UDM0);
		val32 = GET_FIELD(val32, FW_ST);
		if (val32 != FW_ST_ERR_IN)
			break;
		cnt--;
	}

	if (!cnt) {
		adapter->sm.ser_ctrl_st = MAC_AX_SER_CTRL_ERR;
		PLTFM_MSG_ERR("[ERR]stop ser polling FW ST timeout\n");
		return MACPOLLTO;
	}

	return ret;
}

u32 mac_chk_err_status(struct mac_ax_adapter *adapter, u8 *ser_status)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 cnt = MAC_SET_ERR_DLY_CNT;
	u32 ret = MACSUCCESS;
	enum mac_ax_err_info err;

	PLTFM_MUTEX_LOCK(&adapter->lock_info.err_get_lock);
	adapter->sm.l2_st = MAC_AX_L2_DIS;

	while (--cnt) {
		if (MAC_REG_R32(R_AX_HALT_C2H_CTRL))
			break;
		PLTFM_DELAY_US(MAC_SET_ERR_DLY_US);
	}
	if (!cnt) {
		PLTFM_MSG_ERR("Polling FW err status fail\n");
		ret = MACPOLLTO;
		goto end;
	}

	err = (enum mac_ax_err_info)MAC_REG_R32(R_AX_HALT_C2H);

	switch (err) {
	case MAC_AX_ERR_L1_PREERR_DMAC:
	case MAC_AX_ERR_L0_PROMOTE_TO_L1:
		*ser_status = MAC_AX_L1_TRUE;
		break;
	default:
		*ser_status = MAC_AX_L1_FALSE;
		break;
	}

end:
	adapter->sm.l2_st = MAC_AX_L2_EN;
	PLTFM_MUTEX_UNLOCK(&adapter->lock_info.err_get_lock);
	return ret;
}

u32 set_l2_status(struct mac_ax_adapter *adapter)
{
	adapter->sm.l2_st = MAC_AX_L2_EN;

	return MACSUCCESS;
}

u32 ser_imr_config_patch(struct mac_ax_adapter *adapter, u8 band,
			 enum mac_ax_hwmod_sel sel)
{
	return MACSUCCESS;
}