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

#include "ser_auto_gen_8852b.h"
#if MAC_AX_8852B_SUPPORT

static u32 cmac_dma_top_imr_enable_8852b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 reg, ret;

	/* cmac_dma_top 0x1860c800 */
	/* cmac_dma_top 0x1860e800 */
	reg = (band == MAC_AX_BAND_0) ?
		CMAC_DMA_TOP_IMR0_C0_8852B : CMAC_DMA_TOP_IMR0_C1_8852B;
	//clear mask: 0x80c000, set value: 0xc000
	ret = write_mac_reg_auto_ofld(adapter, reg,
				      CMAC_DMA_TOP_IMR0_MSK_8852B,
				      CMAC_DMA_TOP_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 ptcltop_imr_enable_8852b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 reg, ret;

	/* ptcltop 0x1860c6c0 */
	/* ptcltop 0x1860e6c0 */
	reg = (band == MAC_AX_BAND_0) ?
		PTCLTOP_IMR0_C0_8852B : PTCLTOP_IMR0_C1_8852B;
	//clear mask: 0xff80df01, set value: 0x10800001
	ret = write_mac_reg_auto_ofld(adapter, reg,
				      PTCLTOP_IMR0_MSK_8852B,
				      PTCLTOP_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 schedulertop_imr_enable_8852b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 reg, ret;

	/* schedulertop 0x1860c3e8 */
	/* schedulertop 0x1860e3e8 */
	reg = (band == MAC_AX_BAND_0) ?
		SCHEDULERTOP_IMR0_C0_8852B : SCHEDULERTOP_IMR0_C1_8852B;
	//clear mask: 0x3, set value: 0x1
	ret = write_mac_reg_auto_ofld(adapter, reg,
				      SCHEDULERTOP_IMR0_MSK_8852B,
				      SCHEDULERTOP_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 phyintf_imr_enable_8852b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 reg, ret;

	/* phyintf 0x1860ccfc */
	/* phyintf 0x1860ecfc */
	reg = (band == MAC_AX_BAND_0) ?
		PHYINTF_IMR0_C0_8852B : PHYINTF_IMR0_C1_8852B;
	//clear mask: 0x3f0000, set value: 0x10000
	ret = write_mac_reg_auto_ofld(adapter, reg,
				      PHYINTF_IMR0_MSK_8852B,
				      PHYINTF_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 rmac_imr_enable_8852b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 reg, ret;

	/* rmac 0x1860cef4 */
	/* rmac 0x1860eef4 */
	reg = (band == MAC_AX_BAND_0) ?
		RMAC_IMR0_C0_8852B : RMAC_IMR0_C1_8852B;
	//clear mask: 0xff000, set value: 0xe4000
	ret = write_mac_reg_auto_ofld(adapter, reg,
				      RMAC_IMR0_MSK_8852B,
				      RMAC_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 tmac_imr_enable_8852b(struct mac_ax_adapter *adapter, u8 band)
{
	u32 reg, ret;

	/* tmac 0x1860ccec */
	/* tmac 0x1860ecec */
	reg = (band == MAC_AX_BAND_0) ?
		TMAC_IMR0_C0_8852B : TMAC_IMR0_C1_8852B;
	//clear mask: 0x780, set value: 0x780
	ret = write_mac_reg_auto_ofld(adapter, reg,
				      TMAC_IMR0_MSK_8852B,
				      TMAC_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 wdrls_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* wdrls 0x18609430 */
	//clear mask: 0x3337, set value: 0x3327
	ret = write_mac_reg_auto_ofld(adapter,
				      WDRLS_IMR0_8852B,
				      WDRLS_IMR0_MSK_8852B,
				      WDRLS_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 wsec_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* WSEC 0x18609d1c */
	//clear mask: 0x8, set value: 0x8
	ret = write_mac_reg_auto_ofld(adapter,
				      WSEC_IMR0_8852B,
				      WSEC_IMR0_MSK_8852B,
				      WSEC_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 mpdu_tx_processor_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* MPDU TX Processor 0x18609bf4 */
	//clear mask: 0x3e, set value: 0x0
	ret = write_mac_reg_auto_ofld(adapter,
				      MPDU_TX_PROCESSOR_IMR0_8852B,
				      MPDU_TX_PROCESSOR_IMR0_MSK_8852B,
				      MPDU_TX_PROCESSOR_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 mpdu_rx_processor_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* MPDU RX Processor 0x18609cf4 */
	//clear mask: 0xb, set value: 0x0
	ret = write_mac_reg_auto_ofld(adapter,
				      MPDU_RX_PROCESSOR_IMR0_8852B,
				      MPDU_RX_PROCESSOR_IMR0_MSK_8852B,
				      MPDU_RX_PROCESSOR_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 sta_scheduler_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* STA scheduler 0x18609ef0 */
	//clear mask: 0x7, set value: 0x7
	ret = write_mac_reg_auto_ofld(adapter,
				      STA_SCHEDULER_IMR0_8852B,
				      STA_SCHEDULER_IMR0_MSK_8852B,
				      STA_SCHEDULER_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 txpktctl_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* txpktctl 0x18609f1c */
	//clear mask: 0x30f, set value: 0x101
	ret = write_mac_reg_auto_ofld(adapter,
				      TXPKTCTL_IMR0_8852B,
				      TXPKTCTL_IMR0_MSK_8852B,
				      TXPKTCTL_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	/* txpktctl 0x18609f2c */
	//clear mask: 0x30f, set value: 0x303
	ret = write_mac_reg_auto_ofld(adapter,
				      TXPKTCTL_IMR1_8852B,
				      TXPKTCTL_IMR1_MSK_8852B,
				      TXPKTCTL_IMR1_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 wde_dle_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* wde_dle 0x18608c38 */
	//clear mask: 0x7f0ff0ff, set value: 0x3f0ff0ff
	ret = write_mac_reg_auto_ofld(adapter,
				      WDE_DLE_IMR0_8852B,
				      WDE_DLE_IMR0_MSK_8852B,
				      WDE_DLE_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 ple_dle_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* ple_dle 0x18609038 */
	//clear mask: 0xf0ff0ff, set value: 0xf0ff0df
	ret = write_mac_reg_auto_ofld(adapter,
				      PLE_DLE_IMR0_8852B,
				      PLE_DLE_IMR0_MSK_8852B,
				      PLE_DLE_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 pktin_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* PKTIN 0x18609a20 */
	//clear mask: 0x1, set value: 0x1
	ret = write_mac_reg_auto_ofld(adapter,
				      PKTIN_IMR0_8852B,
				      PKTIN_IMR0_MSK_8852B,
				      PKTIN_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 host_dispatcher_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* Host Dispatcher 0x18608850 */
	//clear mask: 0xff0fffff, set value: 0xcc000161
	ret = write_mac_reg_auto_ofld(adapter,
				      HOST_DISPATCHER_IMR0_8852B,
				      HOST_DISPATCHER_IMR0_MSK_8852B,
				      HOST_DISPATCHER_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 cpu_dispatcher_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* CPU Dispatcher 0x18608854 */
	//clear mask: 0x3f07ffff, set value: 0x4000062
	ret = write_mac_reg_auto_ofld(adapter,
				      CPU_DISPATCHER_IMR0_8852B,
				      CPU_DISPATCHER_IMR0_MSK_8852B,
				      CPU_DISPATCHER_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 dispatcher_dle_interface_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* Dispatcher DLE interface 0x18608858 */
	//clear mask: 0x3f031f1f, set value: 0x0
	ret = write_mac_reg_auto_ofld(adapter,
				      DISPATCHER_DLE_INTERFACE_IMR0_8852B,
				      DISPATCHER_DLE_INTERFACE_IMR0_MSK_8852B,
				      DISPATCHER_DLE_INTERFACE_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 dle_cpuio_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* dle_cpuio 0x18609840 */
	//clear mask: 0x1111, set value: 0x1111
	ret = write_mac_reg_auto_ofld(adapter,
				      DLE_CPUIO_IMR0_8852B,
				      DLE_CPUIO_IMR0_MSK_8852B,
				      DLE_CPUIO_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

static u32 bbrpt_imr_enable_8852b(struct mac_ax_adapter *adapter)
{
	u32 ret;

	/* bbrpt 0x1860960c */
	//clear mask: 0x1, set value: 0x1
	ret = write_mac_reg_auto_ofld(adapter,
				      BBRPT_IMR0_8852B,
				      BBRPT_IMR0_MSK_8852B,
				      BBRPT_IMR0_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	/* bbrpt 0x1860962c */
	//clear mask: 0xff, set value: 0x0
	ret = write_mac_reg_auto_ofld(adapter,
				      BBRPT_IMR1_8852B,
				      BBRPT_IMR1_MSK_8852B,
				      BBRPT_IMR1_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	/* bbrpt 0x1860963c */
	//clear mask: 0x1, set value: 0x1
	ret = write_mac_reg_auto_ofld(adapter,
				      BBRPT_IMR2_8852B,
				      BBRPT_IMR2_MSK_8852B,
				      BBRPT_IMR2_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	/* bbrpt 0x1860966c */
	//clear mask: 0x1, set value: 0x1
	ret = write_mac_reg_auto_ofld(adapter,
				      BBRPT_IMR3_8852B,
				      BBRPT_IMR3_MSK_8852B,
				      BBRPT_IMR3_SET_8852B,
				      IMR_IO_OFLD_NON_FLUSH);

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s IO ofld fail", __func__);
		return ret;
	}

	return MACSUCCESS;
}

u32 ser_imr_config_8852b(struct mac_ax_adapter *adapter, u8 band,
			 enum mac_ax_hwmod_sel sel)
{
	u32 ret;

	ret = check_mac_en(adapter, band, sel);
	if (ret) {
		PLTFM_MSG_ERR("MAC%d band%d is not ready\n", sel, band);
		return ret;
	}

	if (sel == MAC_AX_CMAC_SEL) {
		ret = cmac_dma_top_imr_enable_8852b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]cmac_dma_top_imr_enable %d\n", ret);
			return ret;
		}

		ret = ptcltop_imr_enable_8852b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]ptcltop_imr_enable %d\n", ret);
			return ret;
		}

		ret = schedulertop_imr_enable_8852b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]schedulertop_imr_enable %d\n", ret);
			return ret;
		}

		ret = phyintf_imr_enable_8852b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]phyintf_imr_enable %d\n", ret);
			return ret;
		}

		ret = rmac_imr_enable_8852b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]rmac_imr_enable %d\n", ret);
			return ret;
		}

		ret = tmac_imr_enable_8852b(adapter, band);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]tmac_imr_enable %d\n", ret);
			return ret;
		}
	} else if (sel == MAC_AX_DMAC_SEL) {
		ret = wdrls_imr_enable_8852b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]wdrls_imr_enable %d\n", ret);
			return ret;
		}

		ret = wsec_imr_enable_8852b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]wsec_imr_enable %d\n", ret);
			return ret;
		}

		ret = mpdu_tx_processor_imr_enable_8852b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]mpdu_tx_processor_imr_enable %d\n", ret);
			return ret;
		}

		ret = mpdu_rx_processor_imr_enable_8852b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]mpdu_rx_processor_imr_enable %d\n", ret);
			return ret;
		}

		ret = sta_scheduler_imr_enable_8852b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]sta_scheduler_imr_enable %d\n", ret);
			return ret;
		}

		ret = txpktctl_imr_enable_8852b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]txpktctl_imr_enable %d\n", ret);
			return ret;
		}

		ret = wde_dle_imr_enable_8852b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]wde_dle_imr_enable %d\n", ret);
			return ret;
		}

		ret = ple_dle_imr_enable_8852b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]ple_dle_imr_enable %d\n", ret);
			return ret;
		}

		ret = pktin_imr_enable_8852b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]pktin_imr_enable %d\n", ret);
			return ret;
		}

		ret = host_dispatcher_imr_enable_8852b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]host_dispatcher_imr_enable %d\n", ret);
			return ret;
		}

		ret = cpu_dispatcher_imr_enable_8852b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]cpu_dispatcher_imr_enable %d\n", ret);
			return ret;
		}

		ret = dispatcher_dle_interface_imr_enable_8852b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]dispatcher_dle_interface_imr_enable %d\n", ret);
			return ret;
		}

		ret = dle_cpuio_imr_enable_8852b(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]dle_cpuio_imr_enable %d\n", ret);
			return ret;
		}

		ret = bbrpt_imr_enable_8852b(adapter);
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
#endif /* #if MAC_AX_8852B_SUPPORT */
