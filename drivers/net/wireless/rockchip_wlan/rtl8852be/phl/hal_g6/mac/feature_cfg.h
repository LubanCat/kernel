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

#ifndef _MAC_AX_FEATURE_CFG_H_
#define _MAC_AX_FEATURE_CFG_H_

#ifndef __cplusplus /* for win/linux driver */

/* [Note] Modify MakeFile to reduce code size */

/* Debug package for debugging */
#ifdef CONFIG_HAL_MAC_DBG
#define MAC_AX_FEATURE_DBGPKG	1
#else
#define MAC_AX_FEATURE_DBGPKG	0
#endif

/* Error Flag for HW debugging */
#if MAC_AX_FEATURE_DBGPKG
#define MAC_AX_FEATURE_ERR_FLAG	1
#else
#define MAC_AX_FEATURE_ERR_FLAG	0
#endif

/* Hadrware verification module */
#define MAC_AX_FEATURE_HV	0

/* FW offload reg read/write  */
#define MAC_AX_FW_REG_OFLD	0

/* Temp code for FPGA verification*/
#define MAC_AX_FPGA_TEST	0

/* temp for h2c alloc move to phl layer*/
#define MAC_AX_PHL_H2C		1

/* For USB IO Offload Accelerate enable*/
#if defined(CONFIG_FW_DBCC_OFLD_SUPPORT) || defined(CONFIG_PHL_IO_OFLD)
#define MAC_USB_IO_ACC_ON	1
#else
#define MAC_USB_IO_ACC_ON	0
#endif

/* For workaround temp usage */
#define MAC_AX_RTK_WARD		0

/* For workaround sdio issue temp usage */
#ifdef CONFIG_PHL_WKARD_TXEN_ISSUE
#define MAC_AX_RTK_WARD_TXEN_DATA_PLE_FULL	1
#else
#define MAC_AX_RTK_WARD_TXEN_DATA_PLE_FULL	0
#endif

/* For USB IO Offload Accelerate*/
#define MAC_USB_IO_ACC		0

#define MAC_SELF_DIAG_INFO	1

#ifdef PHL_FEATURE_NONE
#define MAC_AX_CONSOLE_EN	0
#elif MAC_SELF_DIAG_INFO
#define MAC_AX_CONSOLE_EN	1
#else
#define MAC_AX_CONSOLE_EN	0
#endif

#if defined(PHL_PLATFORM_WINDOWS) && defined(CONFIG_PHL_DIAGNOSE)
#define MAC_AX_PLDR_DIAGNOSE_EN	1
#else
#define MAC_AX_PLDR_DIAGNOSE_EN	0
#endif

#ifdef CONFIG_IOW_STATISTICS
#define MAC_TXRPT_STATISTIC	1
#endif

/* ------- Begin of Feature Flag  ------- */
#if defined(CONFIG_PHL_BCN_ERLY_RPT) || defined(CONFIG_SCANOFLD) || defined(CONFIG_PHL_CHSWOFLD) || defined(CONFIG_PHL_PKTOFLD) || MAC_USB_IO_ACC_ON
#define MAC_FEAT_FWOFLD		1
#else
#define MAC_FEAT_FWOFLD		0
#endif

#define MAC_FEAT_NLO		1

#ifdef CONFIG_PHL_P2PPS
#define MAC_FEAT_P2P		1
#else
#define MAC_FEAT_P2P		0
#endif

#define MAC_FEAT_RX_BCN_OFLD	1

#define MAC_FEAT_ULOFDMA	1

#define MAC_FEAT_DLOFDMA	1

#define MAC_FEAT_BFMER		1
#ifdef MAC_FEAT_BFMER //& wait yllinzo define F2PCMD
	#define MAC_FEAT_BFMER_F2PSND	1
#endif

#ifdef CONFIG_MCC_SUPPORT
#define MAC_FEAT_MCC		1
#else
#define MAC_FEAT_MCC		0
#endif

#ifdef CONFIG_POWER_SAVE
#define MAC_FEAT_LPS		1
#else
#define MAC_FEAT_LPS		0
#endif

#define MAC_FEAT_TWTAP		1 // TWT AP feature compile flag

#define MAC_FEAT_PSAP		1 // AP power save compile flag

#define MAC_FEAT_MUMIMO		1

#define MAC_FEAT_HWSSN		1

#define MAC_FEAT_HDR_CONV	1

#define MAC_FEAT_HWAMSDU	1

#define MAC_FEAT_NAN		1 // NAN feature compile flag

#define MAC_FEAT_SECURE_BOOT 1

#define MAC_FEAT_BFMEE		1 // sounding MEE compile flag

//FTM compile flag
#ifdef CONFIG_PHL_FTM
#define MAC_FEAT_FTM		1
#else
#define MAC_FEAT_FTM		0
#endif

#define MAC_FEAT_TX_BCN 1
#if MAC_FEAT_TX_BCN
#define MAC_FEAT_MBSSID 1
#define MAC_FEAT_TX_BCN_DM_TBTT_SHIFT 0
#define MAC_FEAT_TX_BCN_DM_IGNORE_EDCCA 1
#else
#define MAC_FEAT_MBSSID 0
#define MAC_FEAT_TX_BCN_DM_TBTT_SHIFT 0
#define MAC_FEAT_TX_BCN_DM_IGNORE_EDCCA 0
#endif

#define MAC_FEAT_LAMODE		1 // for BB LA mode debug usage

#define MAC_FEAT_F2PCMD		1

#ifdef CONFIG_PHL_TWT
#define MAC_FEAT_TWT_STA	1
#else
#define MAC_FEAT_TWT_STA	0
#endif

#define MAC_FEAT_PPDU_STS	1
#if defined(CONFIG_PHL_CHANNEL_INFO) || defined(CONFIG_PHL_WKARD_CHANNEL_INFO_ACK) ||\
defined(CONFIG_PHL_CHANNEL_INFO_DBG)
#define MAC_FEAT_CH_INFO	1
#else
#define MAC_FEAT_CH_INFO	0
#endif
#define MAC_FEAT_DFS		1
#if (MAC_FEAT_PPDU_STS || MAC_FEAT_CH_INFO || MAC_FEAT_DFS)
#define MAC_FEAT_PHY_RPT	1
#else
#define MAC_FEAT_PHY_RPT	0
#endif

#if defined(CONFIG_WOWLAN)
#define MAC_FEAT_WOWLAN		1
#else
#define MAC_FEAT_WOWLAN		0
#endif

#if defined(CONFIG_BCN_CNT)
#define MAC_FEAT_BCN_CNT	1
#else
#define MAC_FEAT_BCN_CNT	0
#endif

//Basic efuse R/W won't include in MAC_FEAT_EFUSE
#define MAC_FEAT_EFUSE		1
#if MAC_FEAT_EFUSE
#define MAC_FEAT_EFUSE_MP	1
#define MAC_FEAT_EFUSE_HV	0
#else
#define MAC_FEAT_EFUSE_MP	0
#define MAC_FEAT_EFUSE_HV	0
#endif

#define MAC_FEAT_COLLOCATION 1

#ifdef CONFIG_DBCC_SUPPORT
#define MAC_FEAT_DBCC		1
#else
#define MAC_FEAT_DBCC		0
#endif

/* COEX */
#ifdef CONFIG_BTCOEX
#define MAC_FEAT_COEX		1
#else
#define MAC_FEAT_COEX		0
#endif

/* For NINTENDO DRIVER*/
#define NINTENDO_EN		0

/* -------  End of Feature Flag  -------  */

#else /* for WD1 test program */

/* [Note] Modify MakeFile to reduce code size */

/* Debug package for debugging */
#define MAC_AX_FEATURE_DBGPKG	1

/* Error Flag for HW debugging */
#if MAC_AX_FEATURE_DBGPKG
#define MAC_AX_FEATURE_ERR_FLAG	1
#else
#define MAC_AX_FEATURE_ERR_FLAG	0
#endif

/* Debug package for debugging */
#define MAC_AX_FW_REG_OFLD	0

/* Hadrware verification module */
#define MAC_AX_FEATURE_HV	1

#define MAC_AX_FPGA_TEST	1

/* temp for h2c alloc move to phl layer */
#define MAC_AX_PHL_H2C		0

/* [HV] for SIC command generagor */
#define MAC_AX_HV_SIC_GEN	0

/* For workaround temp usage */
#define MAC_AX_RTK_WARD		0

/* For workaround sdio issue temp usage */
#define MAC_AX_RTK_WARD_TXEN_DATA_PLE_FULL	0

/* For USB IO Offload Accelerate*/
#define MAC_USB_IO_ACC		0

/* For USB IO Offload Accelerate enable*/
#define MAC_USB_IO_ACC_ON	1

#define MAC_SELF_DIAG_INFO	1

#define MAC_AX_CONSOLE_EN	0

#define MAC_AX_PLDR_DIAGNOSE_EN	0

/* ------- Begin of Feature Flag  ------- */
#define MAC_FEAT_FWOFLD		1

#define MAC_FEAT_NLO		1

#define MAC_FEAT_P2P		1 // P2P feature compile flag
#define MAC_FEAT_RX_BCN_OFLD	1 // RX beacon offload feature compile flag

#define MAC_FEAT_ULOFDMA	1

#define MAC_FEAT_BFMER		1
#ifdef MAC_FEAT_BFMER //& wait yllinzo define F2PCMD
	#define MAC_FEAT_BFMER_F2PSND	1
#endif

#define MAC_FEAT_MCC		1

#define MAC_FEAT_LPS		1

#define MAC_FEAT_TWTAP		1 // TWT AP feature compile flag

#define MAC_FEAT_PSAP		1 // AP power save compile flag

#define MAC_FEAT_MUMIMO		1

#define MAC_FEAT_HWSSN		1

#define MAC_FEAT_HDR_CONV	1

#define MAC_FEAT_HWAMSDU	1

#define MAC_FEAT_SECURE_BOOT 1

#define MAC_FEAT_BFMEE		1 // sounding MEE compile flag

// FTM compile flag
#ifdef CONFIG_PHL_FTM
#define MAC_FEAT_FTM		1
#else
#define MAC_FEAT_FTM		0
#endif

#define MAC_FEAT_TX_BCN 1
#if MAC_FEAT_TX_BCN
#define MAC_FEAT_MBSSID 1
#define MAC_FEAT_TX_BCN_DM_TBTT_SHIFT 0
#define MAC_FEAT_TX_BCN_DM_IGNORE_EDCCA 1
#else
#define MAC_FEAT_MBSSID 0
#define MAC_FEAT_TX_BCN_DM_TBTT_SHIFT 0
#define MAC_FEAT_TX_BCN_DM_IGNORE_EDCCA 0
#endif

#define MAC_FEAT_LAMODE		1 // for BB LA mode debug usage

#define MAC_FEAT_F2PCMD		1

#define MAC_FEAT_TWT_STA	1

#define MAC_FEAT_PPDU_STS	1
#define MAC_FEAT_CH_INFO	1
#define MAC_FEAT_DFS		1
#if (MAC_FEAT_PPDU_STS || MAC_FEAT_CH_INFO || MAC_FEAT_DFS)
#define MAC_FEAT_PHY_RPT	1
#else
#define MAC_FEAT_PHY_RPT	0
#endif

#define MAC_FEAT_WOWLAN		1

//Basic efuse R/W won't include in MAC_FEAT_EFUSE
#define MAC_FEAT_EFUSE		1
#if MAC_FEAT_EFUSE
#define MAC_FEAT_EFUSE_MP	1
#define MAC_FEAT_EFUSE_HV	1
#else
#define MAC_FEAT_EFUSE_MP	0
#define MAC_FEAT_EFUSE_HV	0
#endif

#define MAC_FEAT_COLLOCATION 1
#define MAC_FEAT_DBCC		1 // DBCC

#define MAC_FEAT_COEX		1 // COEX

/* For NINTENDO DRIVER*/
#define NINTENDO_EN		0
/* -------  End of Feature Flag  -------  */


#endif // #else /* for WD1 test program */


#endif

