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

#include "ser_auto_gen_8851b.h"
#if MAC_AX_8851B_SUPPORT

static u32 cmac_dma_top_imr_enable_8851b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 reg, ret;

	/* cmac_dma_top 0x1860c800 R_AX_DLE_CTRL  */
	/* cmac_dma_top 0x1860e800 R_AX_DLE_CTRL_C1  */
	reg = (band == MAC_AX_BAND_0) ?
		R_AX_DLE_CTRL : R_AX_DLE_CTRL_C1;
	//clear mask: 0x80c000, set value: 0xc000
	ret = write_mac_reg_auto_ofld(adapter, reg,
				      CMAC_DMA_TOP_IMR0_MSK_8851B,
				      CMAC_DMA_TOP_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 ptcltop_imr_enable_8851b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 reg, ret;

	/* ptcltop 0x1860c6c0 R_AX_PTCL_IMR0  */
	/* ptcltop 0x1860e6c0 R_AX_PTCL_IMR0_C1  */
	reg = (band == MAC_AX_BAND_0) ?
		R_AX_PTCL_IMR0 : R_AX_PTCL_IMR0_C1;
	//clear mask: 0xff80df01, set value: 0x10800001
	ret = write_mac_reg_auto_ofld(adapter, reg,
				      PTCLTOP_IMR0_MSK_8851B,
				      PTCLTOP_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 schedulertop_imr_enable_8851b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 reg, ret;

	/* schedulertop 0x1860c3e8 R_AX_SCHEDULE_ERR_IMR  */
	/* schedulertop 0x1860e3e8 R_AX_SCHEDULE_ERR_IMR_C1  */
	reg = (band == MAC_AX_BAND_0) ?
		R_AX_SCHEDULE_ERR_IMR : R_AX_SCHEDULE_ERR_IMR_C1;
	//clear mask: 0x3, set value: 0x1
	ret = write_mac_reg_auto_ofld(adapter, reg,
				      SCHEDULERTOP_IMR0_MSK_8851B,
				      SCHEDULERTOP_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 phyintf_imr_enable_8851b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 reg, ret;

	/* phyintf 0x1860ccfc R_AX_PHYINFO_ERR_IMR  */
	/* phyintf 0x1860ecfc R_AX_PHYINFO_ERR_IMR_C1  */
	reg = (band == MAC_AX_BAND_0) ?
		R_AX_PHYINFO_ERR_IMR : R_AX_PHYINFO_ERR_IMR_C1;
	//clear mask: 0x3f0000, set value: 0x10000
	ret = write_mac_reg_auto_ofld(adapter, reg,
				      PHYINTF_IMR0_MSK_8851B,
				      PHYINTF_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 rmac_imr_enable_8851b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 reg, ret;

	/* rmac 0x1860cef4 R_AX_RMAC_ERR_ISR  */
	/* rmac 0x1860eef4 R_AX_RMAC_ERR_ISR_C1  */
	reg = (band == MAC_AX_BAND_0) ?
		R_AX_RMAC_ERR_ISR : R_AX_RMAC_ERR_ISR_C1;
	//clear mask: 0xff000, set value: 0xe4000
	ret = write_mac_reg_auto_ofld(adapter, reg,
				      RMAC_IMR0_MSK_8851B,
				      RMAC_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 tmac_imr_enable_8851b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 reg, ret;

	/* tmac 0x1860ccec R_AX_TMAC_ERR_IMR_ISR  */
	/* tmac 0x1860ecec R_AX_TMAC_ERR_IMR_ISR_C1  */
	reg = (band == MAC_AX_BAND_0) ?
		R_AX_TMAC_ERR_IMR_ISR : R_AX_TMAC_ERR_IMR_ISR_C1;
	//clear mask: 0x780, set value: 0x780
	ret = write_mac_reg_auto_ofld(adapter, reg,
				      TMAC_IMR0_MSK_8851B,
				      TMAC_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 wdrls_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* wdrls 0x18609430 R_AX_WDRLS_ERR_IMR  */
	//clear mask: 0x3337, set value: 0x3327
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_WDRLS_ERR_IMR,
				      WDRLS_IMR0_MSK_8851B,
				      WDRLS_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 wsec_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* WSEC 0x18609d1c R_AX_SEC_DEBUG  */
	//clear mask: 0x8, set value: 0x8
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_SEC_DEBUG,
				      WSEC_IMR0_MSK_8851B,
				      WSEC_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 mpdu_tx_processor_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* MPDU TX Processor 0x18609bf4 R_AX_MPDU_TX_ERR_IMR  */
	//clear mask: 0x3e, set value: 0x0
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_MPDU_TX_ERR_IMR,
				      MPDU_TX_PROCESSOR_IMR0_MSK_8851B,
				      MPDU_TX_PROCESSOR_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 mpdu_rx_processor_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* MPDU RX Processor 0x18609cf4 R_AX_MPDU_RX_ERR_IMR  */
	//clear mask: 0xb, set value: 0x0
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_MPDU_RX_ERR_IMR,
				      MPDU_RX_PROCESSOR_IMR0_MSK_8851B,
				      MPDU_RX_PROCESSOR_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 sta_scheduler_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* STA scheduler 0x18609ef0 R_AX_STA_SCHEDULER_ERR_IMR  */
	//clear mask: 0x7, set value: 0x7
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_STA_SCHEDULER_ERR_IMR,
				      STA_SCHEDULER_IMR0_MSK_8851B,
				      STA_SCHEDULER_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 txpktctl_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* txpktctl 0x18609f1c R_AX_TXPKTCTL_ERR_IMR_ISR  */
	//clear mask: 0x30f, set value: 0x101
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_TXPKTCTL_ERR_IMR_ISR,
				      TXPKTCTL_IMR0_MSK_8851B,
				      TXPKTCTL_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	/* txpktctl 0x18609f2c R_AX_TXPKTCTL_ERR_IMR_ISR_B1  */
	//clear mask: 0x30f, set value: 0x303
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_TXPKTCTL_ERR_IMR_ISR_B1,
				      TXPKTCTL_IMR1_MSK_8851B,
				      TXPKTCTL_IMR1_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 wde_dle_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* wde_dle 0x18608c38 R_AX_WDE_ERR_IMR  */
	//clear mask: 0x7f0ff0ff, set value: 0x3f0ff0ff
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_WDE_ERR_IMR,
				      WDE_DLE_IMR0_MSK_8851B,
				      WDE_DLE_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 ple_dle_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* ple_dle 0x18609038 R_AX_PLE_ERR_IMR  */
	//clear mask: 0xf0ff0ff, set value: 0xf0ff0df
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_PLE_ERR_IMR,
				      PLE_DLE_IMR0_MSK_8851B,
				      PLE_DLE_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 pktin_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* PKTIN 0x18609a20 R_AX_PKTIN_ERR_IMR  */
	//clear mask: 0x1, set value: 0x1
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_PKTIN_ERR_IMR,
				      PKTIN_IMR0_MSK_8851B,
				      PKTIN_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 host_dispatcher_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* Host Dispatcher 0x18608850 R_AX_HOST_DISPATCHER_ERR_IMR  */
	//clear mask: 0xff0fffff, set value: 0xcc000161
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_HOST_DISPATCHER_ERR_IMR,
				      HOST_DISPATCHER_IMR0_MSK_8851B,
				      HOST_DISPATCHER_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 cpu_dispatcher_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* CPU Dispatcher 0x18608854 R_AX_CPU_DISPATCHER_ERR_IMR  */
	//clear mask: 0x3f07ffff, set value: 0x4000062
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_CPU_DISPATCHER_ERR_IMR,
				      CPU_DISPATCHER_IMR0_MSK_8851B,
				      CPU_DISPATCHER_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 dispatcher_dle_interface_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* Dispatcher DLE interface 0x18608858 R_AX_OTHER_DISPATCHER_ERR_IMR  */
	//clear mask: 0x3f031f1f, set value: 0x0
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_OTHER_DISPATCHER_ERR_IMR,
				      DISPATCHER_DLE_INTERFACE_IMR0_MSK_8851B,
				      DISPATCHER_DLE_INTERFACE_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 dle_cpuio_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* dle_cpuio 0x18609840 R_AX_CPUIO_ERR_IMR  */
	//clear mask: 0x1111, set value: 0x1111
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_CPUIO_ERR_IMR,
				      DLE_CPUIO_IMR0_MSK_8851B,
				      DLE_CPUIO_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 bbrpt_imr_enable_8851b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* bbrpt 0x1860960c R_AX_BBRPT_COM_ERR_IMR_ISR  */
	//clear mask: 0x1, set value: 0x1
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_BBRPT_COM_ERR_IMR_ISR,
				      BBRPT_IMR0_MSK_8851B,
				      BBRPT_IMR0_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	/* bbrpt 0x1860962c R_AX_BBRPT_CHINFO_ERR_IMR_ISR  */
	//clear mask: 0xff, set value: 0x0
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_BBRPT_CHINFO_ERR_IMR_ISR,
				      BBRPT_IMR1_MSK_8851B,
				      BBRPT_IMR1_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	/* bbrpt 0x1860963c R_AX_BBRPT_DFS_ERR_IMR_ISR  */
	//clear mask: 0x1, set value: 0x1
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_BBRPT_DFS_ERR_IMR_ISR,
				      BBRPT_IMR2_MSK_8851B,
				      BBRPT_IMR2_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	/* bbrpt 0x1860966c R_AX_LA_ERRFLAG  */
	//clear mask: 0x1, set value: 0x1
	ret = write_mac_reg_auto_ofld(adapter,
				      R_AX_LA_ERRFLAG,
				      BBRPT_IMR3_MSK_8851B,
				      BBRPT_IMR3_SET_8851B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

u32 ser_imr_config_8851b(struct mac_ax_adapter *adapter, u8 band,
			 enum mac_ax_hwmod_sel sel)
{
	u32 ret;

	ret = check_mac_en(adapter, band, sel);
	if (ret) {
		PLTFM_MSG_ERR("MAC%d band%d is not ready\n", sel, band);
		return ret;
	}

	if (sel == MAC_AX_CMAC_SEL) {
		ret = cmac_dma_top_imr_enable_8851b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]cmac_dma_top_imr_enable %d\n", ret);
			return ret;
		}

		ret = ptcltop_imr_enable_8851b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]ptcltop_imr_enable %d\n", ret);
			return ret;
		}

		ret = schedulertop_imr_enable_8851b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]schedulertop_imr_enable %d\n", ret);
			return ret;
		}

		ret = phyintf_imr_enable_8851b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]phyintf_imr_enable %d\n", ret);
			return ret;
		}

		ret = rmac_imr_enable_8851b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]rmac_imr_enable %d\n", ret);
			return ret;
		}

		ret = tmac_imr_enable_8851b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]tmac_imr_enable %d\n", ret);
			return ret;
		}
	} else if (sel == MAC_AX_DMAC_SEL) {
		ret = wdrls_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]wdrls_imr_enable %d\n", ret);
			return ret;
		}

		ret = wsec_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]wsec_imr_enable %d\n", ret);
			return ret;
		}

		ret = mpdu_tx_processor_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]mpdu_tx_processor_imr_enable %d\n", ret);
			return ret;
		}

		ret = mpdu_rx_processor_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]mpdu_rx_processor_imr_enable %d\n", ret);
			return ret;
		}

		ret = sta_scheduler_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]sta_scheduler_imr_enable %d\n", ret);
			return ret;
		}

		ret = txpktctl_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]txpktctl_imr_enable %d\n", ret);
			return ret;
		}

		ret = wde_dle_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]wde_dle_imr_enable %d\n", ret);
			return ret;
		}

		ret = ple_dle_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]ple_dle_imr_enable %d\n", ret);
			return ret;
		}

		ret = pktin_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]pktin_imr_enable %d\n", ret);
			return ret;
		}

		ret = host_dispatcher_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]host_dispatcher_imr_enable %d\n", ret);
			return ret;
		}

		ret = cpu_dispatcher_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]cpu_dispatcher_imr_enable %d\n", ret);
			return ret;
		}

		ret = dispatcher_dle_interface_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]dispatcher_dle_interface_imr_enable %d\n", ret);
			return ret;
		}

		ret = dle_cpuio_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]dle_cpuio_imr_enable %d\n", ret);
			return ret;
		}

		ret = bbrpt_imr_enable_8851b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]bbrpt_imr_enable %d\n", ret);
			return ret;
		}
	} else {
		PLTFM_MSG_ERR("illegal sel %d\n", sel);
		return MACNOITEM;
	}
	return ret;
}
#endif /* #if MAC_AX_8851B_SUPPORT */
