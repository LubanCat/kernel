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

#include "fwdl.h"
#include "mac_priv.h"

/* Const String-Debug Boot Step */
static const char fwdl_dbg_step_0[11] = "MAIN START";
static const char fwdl_dbg_step_1[9] = "DBG DONE";
static const char fwdl_dbg_step_2[14] = "INIT HW PREDL";
static const char fwdl_dbg_step_3[12] = "AXIDMA DONE";
static const char fwdl_dbg_step_4[11] = "IDDMA DONE";
static const char fwdl_dbg_step_5[10] = "UART DONE";
static const char fwdl_dbg_step_6[14] = "HW PREDL DONE";
static const char fwdl_dbg_step_7[11] = "DLFW START";
static const char fwdl_dbg_step_8[14] = "DL FWHDR DONE";
static const char fwdl_dbg_step_9[10] = "DLFW DONE";
static const char fwdl_dbg_step_10[16] = "RXBD CHECK FAIL";
static const char fwdl_dbg_step_11[16] = "DSP CHKSUM FAIL";
static const char fwdl_dbg_step_12[17] = "SECURE BOOT FAIL";
static const char fwdl_dbg_step_13[19] = "SECURE CHKSUM FAIL";
static const char fwdl_dbg_step_14[11] = "RAM LOADED";
static const char fwdl_dbg_step_15[18] = "INIT SYSMIB ENTRY";
static const char fwdl_dbg_step_16[18] = "SYSMIB ENTRY DONE";
static const char fwdl_dbg_step_17[14] = "INIT FW ENTRY";
static const char fwdl_dbg_step_18[12] = "H2CPKT DONE";
static const char fwdl_dbg_step_19[12] = "C2HPKT DONE";
static const char fwdl_dbg_step_20[14] = "FW ENTRY DONE";
static const char fwdl_dbg_step_21[19] = "INIT SYSMIB POSTDL";
static const char fwdl_dbg_step_22[19] = "SYSMIB POSTDL DONE";
static const char fwdl_dbg_step_23[15] = "INIT HW POSTDL";
static const char fwdl_dbg_step_24[12] = "RXI300 DONE";
static const char fwdl_dbg_step_25[12] = "32KCAL DONE";
static const char fwdl_dbg_step_26[13] = "FREERUN DONE";
static const char fwdl_dbg_step_27[13] = "PSTIMER DONE";
static const char fwdl_dbg_step_28[17] = "INIT IOHANG DONE";
static const char fwdl_dbg_step_29[15] = "HW POSTDL DONE";
static const char fwdl_dbg_step_30[15] = "INIT FW POSTDL";
static const char fwdl_dbg_step_31[11] = "FWINT DONE";
static const char fwdl_dbg_step_32[12] = "H2CREG DONE";
static const char fwdl_dbg_step_33[12] = "C2HREG DONE";
static const char fwdl_dbg_step_34[11] = "TXRPT DONE";
static const char fwdl_dbg_step_35[13] = "DBG PKT DONE";
static const char fwdl_dbg_step_36[12] = "CPU MG DONE";
static const char fwdl_dbg_step_37[12] = "FWOFLD DONE";
static const char fwdl_dbg_step_38[8] = "PS DONE";
static const char fwdl_dbg_step_39[15] = "FW POSTDL DONE";
static const char fwdl_dbg_step_40[10] = "INIT WLAN";
static const char fwdl_dbg_step_41[10] = "WLAN DONE";
static const char fwdl_dbg_step_42[14] = "INIT IRQ DONE";
static const char fwdl_dbg_step_43[14] = "OS START DONE";
static const char fwdl_dbg_step_2_52a[18] = "INIT SYSMIB PREDL";
static const char fwdl_dbg_step_3_52a[18] = "SYSMIB PREDL DONE";
static const char fwdl_dbg_step_9_52a[14] = "INIT FW PREDL";
static const char fwdl_dbg_step_12_52a[14] = "FW PREDL DONE";

#if FWDL_SUPPORT_8852A
static const char *fwdl_8852a_debug_step[38] = {
	fwdl_dbg_step_0,
	fwdl_dbg_step_1,
	fwdl_dbg_step_2_52a,
	fwdl_dbg_step_3_52a,
	fwdl_dbg_step_2,
	fwdl_dbg_step_3,
	fwdl_dbg_step_4,
	fwdl_dbg_step_5,
	fwdl_dbg_step_6,
	fwdl_dbg_step_9_52a,
	fwdl_dbg_step_18,
	fwdl_dbg_step_19,
	fwdl_dbg_step_12_52a,
	fwdl_dbg_step_7,
	fwdl_dbg_step_9,
	fwdl_dbg_step_14,
	fwdl_dbg_step_21,
	fwdl_dbg_step_22,
	fwdl_dbg_step_23,
	fwdl_dbg_step_24,
	fwdl_dbg_step_25,
	fwdl_dbg_step_26,
	fwdl_dbg_step_27,
	fwdl_dbg_step_29,
	fwdl_dbg_step_30,
	fwdl_dbg_step_31,
	fwdl_dbg_step_32,
	fwdl_dbg_step_33,
	fwdl_dbg_step_34,
	fwdl_dbg_step_35,
	fwdl_dbg_step_36,
	fwdl_dbg_step_37,
	fwdl_dbg_step_38,
	fwdl_dbg_step_39,
	fwdl_dbg_step_40,
	fwdl_dbg_step_41,
	fwdl_dbg_step_28,
};
#endif

#if FWDL_SUPPORT_8852B || FWDL_SUPPORT_8851B || FWDL_SUPPORT_8852BT
static const char *fwdl_8852b_debug_step[38] = {
	fwdl_dbg_step_0,
	fwdl_dbg_step_1,
	fwdl_dbg_step_2,
	fwdl_dbg_step_3,
	fwdl_dbg_step_4,
	fwdl_dbg_step_5,
	fwdl_dbg_step_6,
	fwdl_dbg_step_7,
	fwdl_dbg_step_9,
	fwdl_dbg_step_14,
	fwdl_dbg_step_15,
	fwdl_dbg_step_16,
	fwdl_dbg_step_17,
	fwdl_dbg_step_18,
	fwdl_dbg_step_19,
	fwdl_dbg_step_20,
	fwdl_dbg_step_21,
	fwdl_dbg_step_22,
	fwdl_dbg_step_23,
	fwdl_dbg_step_24,
	fwdl_dbg_step_25,
	fwdl_dbg_step_26,
	fwdl_dbg_step_27,
	fwdl_dbg_step_29,
	fwdl_dbg_step_30,
	fwdl_dbg_step_31,
	fwdl_dbg_step_32,
	fwdl_dbg_step_33,
	fwdl_dbg_step_34,
	fwdl_dbg_step_35,
	fwdl_dbg_step_36,
	fwdl_dbg_step_37,
	fwdl_dbg_step_38,
	fwdl_dbg_step_39,
	fwdl_dbg_step_40,
	fwdl_dbg_step_41,
	fwdl_dbg_step_28,
};
#endif

#if FWDL_SUPPORT_8852C || FWDL_SUPPORT_8852D || FWDL_SUPPORT_8192XB
static const char *fwdl_8852c_debug_step[39] = {
	fwdl_dbg_step_0,
	fwdl_dbg_step_1,
	fwdl_dbg_step_2,
	fwdl_dbg_step_3,
	fwdl_dbg_step_4,
	fwdl_dbg_step_5,
	fwdl_dbg_step_6,
	fwdl_dbg_step_7,
	fwdl_dbg_step_9,
	fwdl_dbg_step_14,
	fwdl_dbg_step_15,
	fwdl_dbg_step_16,
	fwdl_dbg_step_17,
	fwdl_dbg_step_18,
	fwdl_dbg_step_19,
	fwdl_dbg_step_20,
	fwdl_dbg_step_21,
	fwdl_dbg_step_22,
	fwdl_dbg_step_23,
	fwdl_dbg_step_24,
	fwdl_dbg_step_25,
	fwdl_dbg_step_26,
	fwdl_dbg_step_27,
	fwdl_dbg_step_28,
	fwdl_dbg_step_29,
	fwdl_dbg_step_30,
	fwdl_dbg_step_31,
	fwdl_dbg_step_32,
	fwdl_dbg_step_33,
	fwdl_dbg_step_34,
	fwdl_dbg_step_35,
	fwdl_dbg_step_36,
	fwdl_dbg_step_37,
	fwdl_dbg_step_38,
	fwdl_dbg_step_39,
	fwdl_dbg_step_40,
	fwdl_dbg_step_41,
	fwdl_dbg_step_42,
	fwdl_dbg_step_43,
};
#endif

static inline u32 fwdl_precheck(struct mac_ax_adapter *adapter)
{
	u32 ple_que_ch0_empty;
	u32 ple_que_ch1_empty;
	u32 val32;
	u32 dma_idx;
	u32 cpu_idx;
	u32 ret;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	struct dle_dfi_qempty_t qempty = {0};

	qempty.dle_type = DLE_CTRL_TYPE_PLE;
	qempty.grpsel = 0;
	ret = dle_dfi_qempty(adapter, &qempty);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[FWDL][ERROR][%s] dle dfi empty group %d fail %d\n", __func__,
			      qempty.grpsel, ret);
		return MACDBGPORTDMP;
	}
	val32 = qempty.qempty;
	ple_que_ch0_empty = (val32 & (1 << 8));
	ple_que_ch1_empty = (val32 & (1 << 9));
	if (ple_que_ch0_empty == 0) {
		PLTFM_MSG_ERR("[FWDL][ERROR][%s] PLE queue for PAXIDMA ch0 is not empty\n",
			      __func__);
		return MACWQBUSY;
	}
	if (ple_que_ch1_empty == 0) {
		PLTFM_MSG_ERR("[FWDL][ERROR][%s] PLE queue for PAXIDMA ch1 is not empty\n",
			      __func__);
		return MACWQBUSY;
	}

#if FWDL_AX_MIPS_SUPPORT
	if ((FWDL_IS_AX_MIPS) && adapter->env_info.intf != MAC_AX_INTF_PCIE)
		PLTFM_MSG_TRACE("[FWDL]Skip HAXIDMA CH12 Check\n");
#endif

#if FWDL_AX_RISCV_SUPPORT
	if (FWDL_IS_AX_RISCV) {
		val32 = MAC_REG_R32(R_AX_CH12_TXBD_IDX);
		dma_idx = GET_FIELD(val32, B_AX_CH12_HW_IDX);
		cpu_idx = GET_FIELD(val32, B_AX_CH12_HOST_IDX);
		if (dma_idx != cpu_idx) {
			PLTFM_MSG_ERR("[FWDL][ERROR][%s]", __func__);
			PLTFM_MSG_ERR("HAXIDMA CH12 TX DMA IDX 0x%04X != CPU IDX 0x%04X\n",
				      dma_idx, cpu_idx);
			return MACWQBUSY;
		}
	}
#endif

	return MACSUCCESS;
}

void fwdl_time_parsing(struct mac_ax_adapter *adapter)
{
#if MAC_FWDL_TIME_PROFILING
	u32 rom_boot_time = FWDL_TIME_DELTA_US(FWDL_TIMESTAMP_BUFF.ts0,
					       FWDL_TIMESTAMP_BUFF.ts1);
	u32 header_dl_time = FWDL_TIME_DELTA_US(FWDL_TIMESTAMP_BUFF.ts1,
						FWDL_TIMESTAMP_BUFF.ts2);
	u32 setion_dl_time = FWDL_TIME_DELTA_US(FWDL_TIMESTAMP_BUFF.ts2,
						FWDL_TIMESTAMP_BUFF.ts3);
	u32 secure_check_time = FWDL_TIME_DELTA_US(FWDL_TIMESTAMP_BUFF.ts3,
						   FWDL_TIMESTAMP_BUFF.ts4);
	u32 post_boot_time = FWDL_TIME_DELTA_US(FWDL_TIMESTAMP_BUFF.ts4,
						FWDL_TIMESTAMP_BUFF.ts5);

	PLTFM_MSG_ALWAYS("[FWDL][INFO] ROM Boot Time = %uus\n",
			 rom_boot_time);
	PLTFM_MSG_ALWAYS("[FWDL][INFO] Header DL Time = %uus\n",
			 header_dl_time);
	PLTFM_MSG_ALWAYS("[FWDL][INFO] Section DL Time = %uus\n",
			 setion_dl_time);
	PLTFM_MSG_ALWAYS("[FWDL][INFO] Secure Check Time = %uus\n",
			 secure_check_time);
	PLTFM_MSG_ALWAYS("[FWDL][INFO] Post Boot Time = %uus\n",
			 post_boot_time);
#if NOUSE
	if (rom_boot_time > 1500 || header_dl_time > 15000 || setion_dl_time > 25000 ||
	    secure_check_time > 3000 || post_boot_time > 3000) {
		PLTFM_MSG_ALWAYS("[FWDL][WARN] %uus, %uus, %uus, %uus, %uus, %uus\n",
				 FWDL_TIMESTAMP_BUFF.ts0,
				 FWDL_TIMESTAMP_BUFF.ts1,
				 FWDL_TIMESTAMP_BUFF.ts2,
				 FWDL_TIMESTAMP_BUFF.ts3,
				 FWDL_TIMESTAMP_BUFF.ts4,
				 FWDL_TIMESTAMP_BUFF.ts5);
	}
#endif
#else
	(void *)adapter;
#endif
}

void idmem_share_mode_check(struct mac_ax_adapter *adapter, u8 *fw)
{

	u8 val8 = 0;
	u8 idmem_share_mode = 0;
	u32 val32 = 0;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	struct fwhdr_hdr_t *hdr = (struct fwhdr_hdr_t *)fw;

	/* AX Chip Can't Modify ROM Code, So We have to:
	 * 1. Set C00 SEC_IDMEM_SIZE_CONFIG if IS NOT Secure IC
	 * 2. Show ERROR if Previous IDMEM Share Mode != Current IDMEM Share Mode
	 * if IS secure IC
	 */
	if (!adapter->hw_info->is_sec_ic) {
		// 1. Set C00 SEC_IDMEM_SIZE_CONFIG if IS NOT Secure IC
		val32 = MAC_REG_R32(R_AX_SEC_CTRL);
		val8 = GET_FIELD(val32, B_SEC_IDMEM_SIZE_CONFIG);
		if (val8 != FWDL_IDMEM_SHARE_DEFAULT_MODE) {
			PLTFM_MSG_ALWAYS("[FWDL] Change IDMEM Share to Default Mode 0x%x\n",
					 FWDL_IDMEM_SHARE_DEFAULT_MODE);
			val32 = SET_CLR_WORD(val32, FWDL_IDMEM_SHARE_DEFAULT_MODE,
					     B_SEC_IDMEM_SIZE_CONFIG);
			MAC_REG_W32(R_AX_SEC_CTRL, val32);
		}
	} else {
		/* 2. Show WARNING if Previous IDMEM Share Mode != Default Mode
		 * if IS secure IC
		 */
		val32 = MAC_REG_R32(R_AX_WCPU_FW_CTRL);
		if (val32 & B_AX_IDMEM_SHARE_MODE_RECORD_VALID) {
			/* Record Is Valid, This may be second boot
			 * Now we will Check if Previous IDMEM Share Mode Match Current
			 */
			val8 = GET_FIELD(val32, B_AX_IDMEM_SHARE_MODE_RECORD);
			if (val8 != FWDL_IDMEM_SHARE_DEFAULT_MODE) {
				PLTFM_MSG_WARN("[FWDL][WARN][%s] IDMEM Share Mode Change\n",
						__func__);
				PLTFM_MSG_WARN("[FWDL][WARN][%s] Previous Mode = 0x%X\n",
						__func__, val8);
				PLTFM_MSG_WARN("[FWDL][WARN][%s] Default Mode = 0x%X\n",
						__func__, FWDL_IDMEM_SHARE_DEFAULT_MODE);
				PLTFM_MSG_WARN("[FWDL][WARN][%s] FWDL May Fail.\n",
						__func__);
			}
		}
		//Record Current IDMEM Share Mode
		idmem_share_mode = GET_FIELD(le32_to_cpu(hdr->dword7), FWHDR_FW_IDMEM_SHARE_MODE);
		val32 = SET_CLR_WORD(val32, idmem_share_mode, B_AX_IDMEM_SHARE_MODE_RECORD);
		val32 |= B_AX_IDMEM_SHARE_MODE_RECORD_VALID;
		MAC_REG_W32(R_AX_WCPU_FW_CTRL, val32);
	}
}

static inline void fwhdr_section_parser(struct mac_ax_adapter *adapter,
					struct fwhdr_section_t *section,
					struct fwhdr_section_info *info)
{
	u32 hdr_val;
	u32 section_len;

	hdr_val = le32_to_cpu(section->dword1);
	section_len = GET_FIELD(hdr_val, SECTION_INFO_SEC_SIZE);
	if (hdr_val & SECTION_INFO_CHECKSUM)
		section_len += FWDL_SECTION_CHKSUM_LEN;
	info->type = GET_FIELD(hdr_val, SECTION_INFO_SECTIONTYPE);
	if (info->type == FWDL_SECURITY_SECTION_TYPE)
		info->mssc = le32_to_cpu(section->dword2);
	else
		info->mssc = 0;
	info->len = section_len;
	info->redl = (hdr_val & SECTION_INFO_REDL) ? 1 : 0;

#if FWDL_AX_MIPS_SUPPORT
	if (FWDL_IS_AX_MIPS) {
		info->dladdr = (GET_FIELD(le32_to_cpu(section->dword0),
		SECTION_INFO_SEC_DL_ADDR)) & 0x1FFFFFFF;
	}
#endif
#if FWDL_AX_RISCV_SUPPORT
	if (FWDL_IS_AX_RISCV)
		info->dladdr = (GET_FIELD(le32_to_cpu(section->dword0),	SECTION_INFO_SEC_DL_ADDR));
#endif
}

static inline u32 fwhdr_hdr_parser(struct mac_ax_adapter *adapter, struct fwhdr_hdr_t *hdr,
				   struct fw_bin_info *info)
{
	u32 hdr_val;
	u32 val32;

	hdr_val = le32_to_cpu(hdr->dword6);
	info->section_num = GET_FIELD(hdr_val, FWHDR_SEC_NUM);

	hdr_val = le32_to_cpu(hdr->dword3);
	info->hdr_len = GET_FIELD(hdr_val, FWHDR_FWHDR_SZ);

	/* fill HALMAC information */
	hdr_val = le32_to_cpu(hdr->dword7);
	hdr_val = SET_CLR_WORD(hdr_val, FWDL_SECTION_PER_PKT_LEN,
			       FWHDR_FW_PART_SZ);
	info->dynamic_hdr_en = GET_FIELD(hdr_val, FWHDR_FW_DYN_HDR);
	hdr->dword7 = cpu_to_le32(hdr_val);

	if (info->dynamic_hdr_en) {
		info->dynamic_hdr_len = info->hdr_len - (FWHDR_HDR_LEN +
							 info->section_num * FWHDR_SECTION_LEN);
		val32 = le32_to_cpu(*(u32 *)(((u8 *)hdr) +
					     (FWHDR_HDR_LEN +
					      info->section_num * FWHDR_SECTION_LEN)));
		if (val32 != info->dynamic_hdr_len) {
			PLTFM_MSG_ERR("[ERR]%s Dynamic Header Len miss match\n", __func__);
			PLTFM_MSG_ERR("[ERR]Recorded Len (0x%X) != Calculated Len (0x%X)\n", val32,
				      info->dynamic_hdr_len);
			return MACCMP;
		}
	} else {
		val32 = FWHDR_HDR_LEN + info->section_num * FWHDR_SECTION_LEN;
		if (val32 != info->hdr_len) {
			PLTFM_MSG_ERR("[ERR]%s Static Header Len miss match\n", __func__);
			PLTFM_MSG_ERR("[ERR]Recorded Len (0x%X) != Calculated Len (0x%X)\n", val32,
				      info->hdr_len);
			return MACCMP;
		}
		info->dynamic_hdr_len = 0;
	}

	hdr_val = le32_to_cpu(hdr->dword2);
	info->git_idx = GET_FIELD(hdr_val, FWHDR_COMMITID);
	PLTFM_MSG_ALWAYS("[FWDL] Image Git SHA = %08X\n", info->git_idx);
	return MACSUCCESS;
}

static u32 get_ple_base(struct mac_ax_adapter *adapter)
{
	u32 ple_base = FWDL_PLE_BASE_ADDR;

	if (!ple_base)
		PLTFM_MSG_WARN("[FWDL][WARN][%s] Unknown Chip ID 0x%X's PLE Base\n", __func__,
			       adapter->hw_info->chip_id);
	return ple_base;
}

static inline u32 get_mss_keypool_index(struct mac_ax_adapter *adapter, u32 *mss_idx,
					u8 *secure_section_end, u32 *msskp_hdr_len,
					u32 *msskp_key_num)
{
	u8 b1, b2, mss_dev_type, mss_cust_idx, mss_key_num, sel_bit_idx, bit_idx;
	u8 end_bit;
	u8 *p_mss_pool, *rmp_tbl;
	u32 byte_idx;
	u32 sel_byte_idx;
	struct fwdl_mss_pool_hdr_v0 mss_hdr = { 0 };
	u8 mss_pool_magic_num[] = { 0x4D, 0x53, 0x53, 0x4B, 0x50, 0x4F, 0x4F, 0x4C }; //MSSKPOOL
	u32 rmp_tbl_size = 0;
	u32 mss_sel_idx = 0;
	u32 en_key_count = 0;
	enum mac_ax_efuse_bank bank = MAC_AX_EFUSE_BANK_WIFI;
	struct mac_ax_ops *ops = adapter_to_mac_ops(adapter);

	//MSS Pool Locate AFTER Secure Section, out of section headder recorded bound
	p_mss_pool = secure_section_end;

	//Compare to Magic Number
	if (PLTFM_MEMCMP(p_mss_pool, mss_pool_magic_num, FWDL_MSS_POOL_HDR_MGN_SIZE)) {
		PLTFM_MSG_ERR("[FWDL][ERR][%s] MSS Key Pool Magic Number Not Found!\n", __func__);
		return MACFWBIN;
	}

	//Store MSS Keypool Header
	PLTFM_MEMCPY(&mss_hdr, p_mss_pool, FWDL_MSS_POOL_HDR_LEN);
	// Handle Endian Issue
	ENDIAN_REVERTER(mss_hdr, FWDL_MSS_POOL_HDR_LEN);

	//Locate MSS Keypool Remap Table Size and Location
	if (mss_hdr.rmpfmt == MSS_POOL_RMP_TBL_BITMASK) {
		rmp_tbl_size = ((mss_hdr.msskey_num_max) * (mss_hdr.msscust_max) *
				(mss_hdr.mssdev_max)) >> 3;
		if (mss_hdr.defen)
			rmp_tbl_size += FWDL_MSS_POOL_DEFKEYSETS_SIZE;
		rmp_tbl = p_mss_pool + mss_hdr.mss_rmp_tbl_offset;
	} else {
		PLTFM_MSG_ERR("[FWDL][ERR][%s] MSS Key Pool Remap Table Format Unsupport:%X\n",
			      __func__, mss_hdr.rmpfmt);
		return MACNOITEM;
	}

	//Check MSS Keypool Header Format Correct
	if (rmp_tbl_size + FWDL_MSS_POOL_HDR_LEN != mss_hdr.mss_key_raw_offset) {
		PLTFM_MSG_ERR("[FWDL][ERR][%s] MSS Key Pool Format Error:0x%X + 0x%X != 0x%X\n",
			      __func__, rmp_tbl_size, FWDL_MSS_POOL_HDR_LEN,
			      mss_hdr.mss_key_raw_offset);
		return MACNOITEM;
	}

	//Get MSS Necessary Information From OTP
	if (ops->read_efuse(adapter, EFUSE_EXTERNALPN_ADDR, 1, &b1, bank) != 0) {
		PLTFM_MSG_ERR("[FWDL][ERR][%s] Read efuse 0x%X failed.\n", __func__,
			      EFUSE_EXTERNALPN_ADDR);
		return MACEFUSEREAD;
	}
	if (ops->read_efuse(adapter, EFUSE_CUSTOMER_ADDR, 1, &b2, bank) != 0) {
		PLTFM_MSG_ERR("[FWDL][ERR][%s] Read efuse 0x%X failed.\n", __func__,
			      EFUSE_CUSTOMER_ADDR);
		return MACEFUSEREAD;
	}
	b1 = le32_to_cpu(b1);
	b2 = le32_to_cpu(b2);
	PLTFM_MSG_TRACE("[FWDL][INFO] Read efuse EFUSE_EXTERNALPN_ADDR(0x%X) = 0x%X.\n",
			EFUSE_EXTERNALPN_ADDR, b1);
	PLTFM_MSG_TRACE("[FWDL][INFO] Read efuse EFUSE_CUSTOMER_ADDR(0x%X) = 0x%X.\n",
			EFUSE_CUSTOMER_ADDR, b2);

	//Transfer OTP Value into Device Type
	mss_dev_type = GET_FIELD(b1, EFUSE_MSSDEVTYPE);
	mss_cust_idx = 0x1F - (GET_FIELD(b1, EFUSE_MSSCUSTIDX0) |
			       (GET_FIELD(b2, EFUSE_MSSCUSTIDX1) << EFUSE_MSSCUSTIDX0_SH));
	mss_key_num = 0xF - GET_FIELD(b2, EFUSE_MSSKEYNUM);

	if ((FWDL_IS_8852B) && (b1 == 0xFF && b2 == 0x6E)) {
	// compatible old release
		mss_dev_type = MSS_DEV_TYPE_FWSEC_NONLIN_INBOX_NON_COB;
		mss_cust_idx = 0;
		mss_key_num = 0;
	}

	switch (mss_dev_type) {
	case MSS_DEV_TYPE_FWSEC_WINLIN_INBOX:
		mss_dev_type = 0x0;
		break;
	case MSS_DEV_TYPE_FWSEC_NONLIN_INBOX_NON_COB:
		mss_dev_type = 0x1;
		break;
	case MSS_DEV_TYPE_FWSEC_NONLIN_INBOX_COB:
		mss_dev_type = 0x2;
		break;
	case MSS_DEV_TYPE_FWSEC_NONWIN_INBOX:
		mss_dev_type = 0x3;
		break;
	case MSS_DEV_TYPE_FWSEC_DEF:
		break;
	default:
		mss_dev_type = 0xFF;
		PLTFM_MSG_ERR("[FWDL][ERR][%s] MSS Key Pool Unknown Device Type = 0x%X.\n",
			      __func__, mss_dev_type);
		return MACNOITEM;
	}

	//Calculate Relative MSS Key Pool Keypair Index
	if (mss_dev_type == MSS_DEV_TYPE_FWSEC_DEF) {
		if (mss_hdr.defen)
			mss_sel_idx = mss_cust_idx * (mss_hdr.msskey_num_max) + mss_key_num;
		else
			return MACNOITEM;
	} else {
		if (mss_hdr.defen)
			mss_sel_idx = (FWDL_MSS_POOL_DEFKEYSETS_SIZE << 3);
		else
			mss_sel_idx = 0;
		mss_sel_idx += (mss_dev_type * ((mss_hdr.msscust_max) * (mss_hdr.msskey_num_max)) +
				mss_cust_idx * (mss_hdr.msskey_num_max) + mss_key_num);
	}
	//Looking into Remap Table, Make sure OTP Selected Key is Also Enabled in Remap table
	sel_byte_idx = mss_sel_idx >> 3;
	sel_bit_idx = mss_sel_idx & 0x7;
	if (rmp_tbl[sel_byte_idx] & (0x1 << sel_bit_idx)) {
		/* Calculate the number of key is enable in Remap Table, before the selected key
		 * This Numner - 1 is the actual key offset of selected key in Image
		 */
		for (byte_idx = 0; byte_idx <= sel_byte_idx; byte_idx++) {
			end_bit = (byte_idx == sel_byte_idx) ? sel_bit_idx : 7;
			for (bit_idx = 0; bit_idx <= end_bit; bit_idx++) {
				if (rmp_tbl[byte_idx] & (0x1 << bit_idx))
					en_key_count++;
			}
		}
	} else {
		PLTFM_MSG_ERR("[FWDL][ERR][%s] MSS Key Pool Key offset 0x%X Not Enable.\n",
			      __func__, mss_sel_idx);
		return MACNOITEM;
	}
	*mss_idx = en_key_count - 1;
	//fwbin_info->img_info.mss_keypool_keypair_num = (u16)mss_hdr.keypair_num;
	//fwbin_info->img_info.mss_keypool_hdr_len = mss_hdr.mss_key_raw_offset;
	PLTFM_MSG_TRACE("[FWDL][INFO] mss_idx = 0x%X.\n", *mss_idx);
	PLTFM_MSG_TRACE("[FWDL][INFO] Keypair number = 0x%X.\n", mss_hdr.keypair_num);
	PLTFM_MSG_TRACE("[FWDL][INFO] mss_pool_hdr_len = 0x%X.\n",
			mss_hdr.mss_key_raw_offset);
	*msskp_hdr_len = mss_hdr.mss_key_raw_offset;
	*msskp_key_num = mss_hdr.keypair_num;
	return MACSUCCESS;
}

static s8 __mss_index(struct mac_ax_adapter *adapter)
{
	s8 ret = 0;
	u8 externalPN = 0;
	u8 customer = 0;
	u8 serialNum = 0;
	u8 b1;
	u8 b2;
	u8 i;
	enum mac_ax_efuse_bank bank = MAC_AX_EFUSE_BANK_WIFI;
	struct mac_ax_ops *ops = adapter_to_mac_ops(adapter);

	if (ops->read_efuse(adapter, EFUSE_EXTERNALPN_ADDR, 1, &b1, bank) != 0) {
		PLTFM_MSG_ERR("[ERR]%s: Read efuse 0x5EC failed.\n", __func__);
		return -1;
	}
	if (ops->read_efuse(adapter, EFUSE_CUSTOMER_ADDR, 1, &b2, bank) != 0) {
		PLTFM_MSG_ERR("[ERR]%s: Read efuse 0x5ED failed.\n", __func__);
		return -1;
	}
	b1 = le32_to_cpu(b1);
	b2 = le32_to_cpu(b2);
	externalPN = 0xFF - GET_FIELD(b1, EFUSE_EXTERNALPN);
	customer = 0xF - GET_FIELD(b2, EFUSE_CUSTOMER);
	serialNum = 0x7 - GET_FIELD(b2, EFUSE_SERIALNUM);
	PLTFM_MSG_TRACE("[DBG]%s: External PN %x\n", __func__, externalPN);
	PLTFM_MSG_TRACE("[DBG]%s: customer %x\n", __func__, customer);
	PLTFM_MSG_TRACE("[DBG]%s: Serial Num %x\n", __func__, serialNum);
	for (i = 0; i < OTP_KEY_INFO_NUM; i++) {
		if (externalPN == otp_key_info_externalPN[i] &&
		    customer == otp_key_info_customer[i] &&
		    serialNum == otp_key_info_serialNum[i]) {
			ret = (s8)i;
			break;
		}
	}
	PLTFM_MSG_TRACE("[DBG]%s: ret %d\n", __func__, ret);
	return ret;
}

static u32 fwhdr_parser(struct mac_ax_adapter *adapter, u8 *fw, u32 len,
			struct fw_bin_info *info)
{
	u32 i;
	u8 *fw_end = fw + len;
	u8 *bin_ptr;
	struct fwhdr_section_info *cur_section_info;
	u32 ret;
	u32 fwdl_ple_base;
	enum DLE_RSVD_INFO dle_info;
	u8 *mss_start;
	u8 *mss_selected;
	s8 mss_idx = 0;
	u32 msskp_idx = 0;
	u32 msskp_hdr_len;
	u32 msskp_key_num;

	if (!info) {
		PLTFM_MSG_ERR("[ERR]%s: *info = NULL\n", __func__);
		return MACNPTR;
	} else if (!fw) {
		PLTFM_MSG_ERR("[ERR]%s: *fw = NULL\n", __func__);
		return MACNOITEM;
	} else if (!len) {
		PLTFM_MSG_ERR("[ERR]%s: len = 0\n", __func__);
		return MACBUFSZ;
	}

	fwdl_ple_base = get_ple_base(adapter);
	ret = fwhdr_hdr_parser(adapter, (struct fwhdr_hdr_t *)fw, info);
	if (ret)
		return ret;
	bin_ptr = fw + info->hdr_len;

	/* jump to section header */
	fw += FWHDR_HDR_LEN;
	cur_section_info = info->section_info;
	info->is_fw_use_ple = 0;
	for (i = 0; i < info->section_num; i++) {
		fwhdr_section_parser(adapter, (struct fwhdr_section_t *)fw,
				     cur_section_info);
		if (FWDL_IS_8852B) {
			if (cur_section_info->type == FWDL_SECURITY_SECTION_TYPE) {
				cur_section_info->len = 2048;
				((struct fwhdr_section_t *)fw)->dword1 = 0x09000800; //0x800 = 2048
			}
		}
		cur_section_info->addr = bin_ptr;
		bin_ptr += cur_section_info->len;
		if (cur_section_info->dladdr == fwdl_ple_base)
			info->is_fw_use_ple = 1;
		if (cur_section_info->type == FWDL_SECURITY_SECTION_TYPE &&
		    cur_section_info->mssc > 0) {
			mss_start = cur_section_info->addr + FWDL_SECURITY_SECTION_CONSTANT;
			if ((cur_section_info->mssc & 0xFF) == 0xFF) {
				PLTFM_MSG_TRACE("[FWDL][INFO] New MSS Key Pool Enable\n");
				ret = get_mss_keypool_index(adapter, &msskp_idx, bin_ptr,
							    &msskp_hdr_len, &msskp_key_num);
				if (ret)
					return ret;
				mss_selected = bin_ptr + (((u64)msskp_idx) * FWDL_SECURITY_SIGLEN) +
					       msskp_hdr_len;
				fw_end -= (msskp_key_num * FWDL_SECURITY_SIGLEN + msskp_hdr_len);
			} else {
				PLTFM_MSG_TRACE("[FWDL][INFO] Multi Secure Section Enable\n");
				fw_end -= (cur_section_info->mssc * FWDL_SECURITY_SIGLEN);
				mss_idx = __mss_index(adapter);
				if (mss_idx < 0) {
					PLTFM_MSG_ERR("[ERR]%s:", __func__);
					PLTFM_MSG_ERR("Failed to get secure info offset\n");
					return MACFWBIN;
				}
				mss_selected = bin_ptr + (mss_idx * FWDL_SECURITY_SIGLEN);
			}
			PLTFM_MEMCPY(mss_start, mss_selected, FWDL_SECURITY_SIGLEN);
			if (FWDL_IS_8852B) {
				// Workaround for 1344 workaround,
				// Secure Boot CAN NOT have 1344 workaround
				// Assuming if mss_idx>0, than we are in secure boot.
				// Sub 1088 (1344-256) from cur_section_info->len
				if (mss_idx > 0 || msskp_idx > 0) {
					//cur_section_info->len should be 960
					cur_section_info->len = 960;
					//0x3C0=960
					((struct fwhdr_section_t *)fw)->dword1 = 0x090003C0;
				}
			}
		}
		fw += FWHDR_SECTION_LEN;
		cur_section_info++;
	}

	ret = get_dle_rsvd_info(adapter, (enum DLE_RSVD_INFO *)&dle_info);
	if (ret != MACSUCCESS)
		return ret;
	// Check DLE reserved quota for FW
	if (dle_info != DLE_RSVD_INFO_FW && info->is_fw_use_ple) {
		PLTFM_MSG_ERR("[ERR]%s: ", __func__);
		PLTFM_MSG_ERR("fw try to download something to ple illegally\n");
		return MACFWBIN;
	}
	adapter->fw_info.last_fw_use_ple = (u8)info->is_fw_use_ple;

	if (fw_end != bin_ptr) {
		PLTFM_MSG_ERR("[ERR]%s: ", __func__);
		PLTFM_MSG_ERR("fw bin size (0x%x) != fw size in fwhdr (0x%x)\n",
			      len, (u32)(bin_ptr - fw));
		return MACFWBIN;
	}

	return MACSUCCESS;
}

static inline u32 update_fw_ver(struct mac_ax_adapter *adapter,
				struct fwhdr_hdr_t *hdr)
{
	u32 hdr_val;
	struct mac_ax_fw_info *info = &adapter->fw_info;

	hdr_val = le32_to_cpu(hdr->dword1);
	info->major_ver = GET_FIELD(hdr_val, FWHDR_MAJORVER);
	info->minor_ver = GET_FIELD(hdr_val, FWHDR_MINORVER);
	info->sub_ver = GET_FIELD(hdr_val, FWHDR_SUBVERSION);
	info->sub_idx = GET_FIELD(hdr_val, FWHDR_SUBINDEX);

	hdr_val = le32_to_cpu(hdr->dword2);
	info->commit_id = GET_FIELD(hdr_val, FWHDR_COMMITID);

	hdr_val = le32_to_cpu(hdr->dword5);
	info->build_year = GET_FIELD(hdr_val, FWHDR_YEAR);

	hdr_val = le32_to_cpu(hdr->dword4);
	info->build_mon = GET_FIELD(hdr_val, FWHDR_MONTH);
	info->build_date = GET_FIELD(hdr_val, FWHDR_DATE);
	info->build_hour = GET_FIELD(hdr_val, FWHDR_HOUR);
	info->build_min = GET_FIELD(hdr_val, FWHDR_MIN);

	info->h2c_seq = 0;
	info->rec_seq = 0;

	return MACSUCCESS;
}

u32 mac_get_dynamic_hdr_ax(struct mac_ax_adapter *adapter, u8 *fw, u32 fw_len)
{
	u32 ret = MACSUCCESS;
	struct fw_bin_info info;
	u32 dynamic_hdr_len = 0;
	u32 dynamic_hdr_count = 0;
	u32 gitshainfo = 0;
	u16 dynamic_section_len = 0;
	u8 dynamic_section_type = 0;
	u32 val32 = 0;
	u32 total_len = 0;
	u8 *dynamic_hdr_content;
	u8 *content;

	adapter->fw_info.cap_size = 0;
	PLTFM_MEMSET(adapter->fw_info.cap_buff, 0, MAC_WLANFW_CAP_MAX_SIZE);

	if (!fw) {
		PLTFM_MSG_ERR("[ERR]%s: FW == NULL\n", __func__);
		return MACNOFW;
	}
	ret = fwhdr_hdr_parser(adapter, (struct fwhdr_hdr_t *)fw, &info);
	if (ret)
		return ret;
	if (!info.dynamic_hdr_en) {
		PLTFM_MSG_TRACE("[TRACE]%s: WCPU Dynamic Header Disabled\n", __func__);
		return MACSUCCESS;
	}
	dynamic_hdr_content = fw + (FWHDR_HDR_LEN + info.section_num * FWHDR_SECTION_LEN);
	dynamic_hdr_len = le32_to_cpu(*(u32 *)dynamic_hdr_content);
	dynamic_hdr_count = le32_to_cpu(*(u32 *)(dynamic_hdr_content + 4));
	dynamic_hdr_content += 8;
	total_len += 8;
	// Check Dynamic Header length match first, so we won't need to malloc and free if failed.
	for (val32 = 0; val32 < dynamic_hdr_count; val32++) {
		dynamic_section_len = le16_to_cpu(*(u16 *)(dynamic_hdr_content));
		dynamic_section_type = *(u8 *)(dynamic_hdr_content + 2);
		total_len += dynamic_section_len;
		dynamic_hdr_content += dynamic_section_len;
	}
	if (total_len % 16)
		total_len += (16 - (total_len % 16));
	if (total_len != dynamic_hdr_len) {
		PLTFM_MSG_ERR("[ERR]%s: Dynamic Hdr Len Compare Fail:0x%X!=0x%X\n",
			      __func__, total_len, dynamic_hdr_len);
		return MACCMP;
	}
	// Restore dynamic_hdr_content, reparsing from dynamic head again
	dynamic_hdr_content = fw + (FWHDR_HDR_LEN + info.section_num * FWHDR_SECTION_LEN);
	dynamic_hdr_content += 8;
	for (val32 = 0; val32 < dynamic_hdr_count; val32++) {
		dynamic_section_len = le16_to_cpu(*(u16 *)(dynamic_hdr_content));
		dynamic_section_type = *(u8 *)(dynamic_hdr_content + 2);
		content = dynamic_hdr_content + 4;
		//switch CPU
		switch (dynamic_section_type) {
		case FWDL_DYNAMIC_HDR_FWCAP:
			if (dynamic_section_len - 4 > MAC_WLANFW_CAP_MAX_SIZE) {
				PLTFM_MSG_ERR("[ERR]%s: Dynamic Hdr Len 0x%X Exceed Fix Len 0x%X\n",
					      __func__, dynamic_section_len - 4,
					      MAC_WLANFW_CAP_MAX_SIZE);
				return MACNOBUF;
			}
			PLTFM_MEMCPY(adapter->fw_info.cap_buff, content, dynamic_section_len - 4);
			adapter->fw_info.cap_size = dynamic_section_len - 4;
			break;
		case FWDL_DYNAMIC_HDR_OUTSRC_GIT_INFO:
			gitshainfo = le32_to_cpu(*(u32 *)content);
			adapter->fw_info.commit_id_outsrc_bb = gitshainfo;
			PLTFM_MSG_ALWAYS("[FWDL] BB Git SHA = %08X\n", gitshainfo);
			gitshainfo = le32_to_cpu(*(u32 *)(content + 4));
			adapter->fw_info.commit_id_outsrc_btc = gitshainfo;
			PLTFM_MSG_ALWAYS("[FWDL] BTC Git SHA = %08X\n", gitshainfo);
			gitshainfo = le32_to_cpu(*(u32 *)(content + 8));
			adapter->fw_info.commit_id_outsrc_rf = gitshainfo;
			PLTFM_MSG_ALWAYS("[FWDL] RF Git SHA = %08X\n", gitshainfo);
			break;
		case FWDL_DYNAMIC_HDR_NOUSE:
		case FWDL_DYNAMIC_HDR_MAX:
		default:
			PLTFM_MSG_ERR("[ERR]%s: Dynamic Hdr Type Unused or Undefind:0x%X\n",
				      __func__, dynamic_section_type);
			break;
		}
		dynamic_hdr_content += dynamic_section_len;
	}

	return ret;
}

static u32 __fwhdr_download(struct mac_ax_adapter *adapter,
			    u8 *fw, u32 hdr_len, u8 redl)
{
	u8 *buf;
	u32 ret = 0;
	#if MAC_AX_PHL_H2C
	struct rtw_h2c_pkt *h2cb;
	#else
	struct h2c_buf *h2cb;
	#endif

	h2cb = h2cb_alloc(adapter, H2CB_CLASS_DATA);
	if (!h2cb) {
		PLTFM_MSG_ERR("[ERR]%s: h2cb_alloc fail\n", __func__);
		return MACNPTR;
	}

	buf = h2cb_put(h2cb, hdr_len);
	if (!buf) {
		PLTFM_MSG_ERR("[ERR]%s: h2cb_put fail\n", __func__);
		ret = MACNOBUF;
		goto fail;
	}

	PLTFM_MEMCPY(buf, fw, hdr_len);

	if (redl) {
		ret = h2c_pkt_set_hdr_fwdl(adapter, h2cb,
					   FWCMD_TYPE_H2C, FWCMD_H2C_CAT_MAC,
					   FWCMD_H2C_CL_FWDL,
					   FWCMD_H2C_FUNC_FWHDR_REDL, 0, 0);
	} else {
		ret = h2c_pkt_set_hdr_fwdl(adapter, h2cb,
					   FWCMD_TYPE_H2C, FWCMD_H2C_CAT_MAC,
					   FWCMD_H2C_CL_FWDL,
					   FWCMD_H2C_FUNC_FWHDR_DL, 0, 0);
	}

	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: set h2c hdr fail\n", __func__);
		goto fail;
	}

	ret = h2c_pkt_build_txd(adapter, h2cb);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: build h2c txd fail\n", __func__);
		goto fail;
	}

	#if MAC_AX_PHL_H2C
	ret = PLTFM_TX(h2cb);
	#else
	ret = PLTFM_TX(h2cb->data, h2cb->len);
	#endif
	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: PLTFM_TX fail\n", __func__);
		goto fail;
	}

	h2cb_free(adapter, h2cb);

	return MACSUCCESS;
fail:
	h2cb_free(adapter, h2cb);

	PLTFM_MSG_ERR("[ERR]%s ret: %d\n", __func__, ret);

	return ret;
}

#if MAC_AX_PHL_H2C
static u32 __sections_build_txd(struct mac_ax_adapter *adapter,
				struct rtw_h2c_pkt *h2cb)
{
	u8 *buf;
	u32 ret;
	u32 txd_len;
	struct rtw_t_meta_data info = {0};
	struct mac_ax_ops *ops = adapter_to_mac_ops(adapter);

	info.type = RTW_PHL_PKT_TYPE_FWDL;
	info.pktlen = (u16)h2cb->data_len;
	txd_len = ops->txdesc_len(adapter, &info);

	buf = h2cb_push(h2cb, txd_len);
	if (!buf) {
		PLTFM_MSG_ERR("[ERR]%s: h2cb_push fail\n", __func__);
		return MACNPTR;
	}

	ret = ops->build_txdesc(adapter, &info, buf, txd_len);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: ", __func__);
		PLTFM_MSG_ERR("build_txdesc fail\n");
		return ret;
	}

	return MACSUCCESS;
}

static u32 __sections_push(struct rtw_h2c_pkt *h2cb)
{
#define section_push_len 8
	h2cb->vir_data -= section_push_len;
	h2cb->vir_tail -= section_push_len;

	return MACSUCCESS;
}

#else
static u32 __sections_build_txd(struct mac_ax_adapter *adapter,
				struct h2c_buf *h2cb)
{
	u8 *buf;
	u32 ret;
	u32 txd_len;
	struct rtw_t_meta_data info;
	struct mac_ax_ops *ops = adapter_to_mac_ops(adapter);

	info.type = RTW_PHL_PKT_TYPE_FWDL;
	info.pktlen = (u16)h2cb->len;
	txd_len = ops->txdesc_len(adapter, &info);

	buf = h2cb_push(h2cb, txd_len);
	if (!buf) {
		PLTFM_MSG_ERR("[ERR]%s: h2cb_push fail\n", __func__);
		return MACNPTR;
	}

	ret = ops->build_txdesc(adapter, &info, buf, txd_len);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: ", __func__);
		PLTFM_MSG_ERR("mac_build_txdesc fail\n");
		return ret;
	}

	return MACSUCCESS;
}
#endif
static u32 __sections_download(struct mac_ax_adapter *adapter,
			       struct fwhdr_section_info *info)
{
	u8 *section = info->addr;
	u32 residue_len = info->len;
	u32 pkt_len;
	u8 *buf;
	u32 ret = 0;
	#if MAC_AX_PHL_H2C
	struct rtw_h2c_pkt *h2cb;
	#else
	struct h2c_buf *h2cb;
	#endif

	while (residue_len) {
		if (residue_len >= FWDL_SECTION_PER_PKT_LEN)
			pkt_len = FWDL_SECTION_PER_PKT_LEN;
		else
			pkt_len = residue_len;

		h2cb = h2cb_alloc(adapter, H2CB_CLASS_LONG_DATA);
		if (!h2cb) {
			PLTFM_MSG_ERR("[ERR]%s: ", __func__);
			PLTFM_MSG_ERR("h2cb_alloc fail\n");
			return MACNPTR;
		}
		#if MAC_AX_PHL_H2C
		__sections_push(h2cb);
		#endif
		buf = h2cb_put(h2cb, pkt_len);
		if (!buf) {
			PLTFM_MSG_ERR("[ERR]%s: ", __func__);
			PLTFM_MSG_ERR("h2cb_put fail\n");
			ret = MACNOBUF;
			goto fail;
		}

		PLTFM_MEMCPY(buf, section, pkt_len);

		ret = __sections_build_txd(adapter, h2cb);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]%s: ", __func__);
			PLTFM_MSG_ERR("__sections_build_txd fail\n");
			goto fail;
		}
		#if MAC_AX_PHL_H2C
		ret = PLTFM_TX(h2cb);
		#else
		ret = PLTFM_TX(h2cb->data, h2cb->len);
		#endif
		if (ret) {
			PLTFM_MSG_ERR("[ERR]%s: PLTFM_TX fail\n", __func__);
			goto fail;
		}

		h2cb_free(adapter, h2cb);

		section += pkt_len;
		residue_len -= pkt_len;
	}

	return MACSUCCESS;
fail:
	h2cb_free(adapter, h2cb);

	PLTFM_MSG_ERR("[ERR]%s ret: %d\n", __func__, ret);

	return ret;
}

static u32 __write_memory(struct mac_ax_adapter *adapter,
			  u8 *buffer, u32 addr, u32 len)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u8 *content = NULL;
	u32 dl_size;
	u32 target_addr, write_addr;
	u32 seg_size, seg_bytes;
	u32 val32;
	u32 index = 0;
	u32 ret = MACSUCCESS;

	if (adapter->hw_info->is_sec_ic) {
		PLTFM_MSG_ERR("[ERR]security mode ind accees\n");
		PLTFM_MSG_ERR("[ERR]Abort %s.\n", __func__);
		return MACIOERRINSEC;
	}

	PLTFM_MSG_WARN("%s ind access start\n", __func__);
	PLTFM_MUTEX_LOCK(&adapter->lock_info.ind_access_lock);
	adapter->dbg_info.ind_aces_cnt++;

	MAC_REG_W32(R_AX_FILTER_MODEL_ADDR, addr);
	MAC_REG_W32(R_AX_INDIR_ACCESS_ENTRY, 0xAAAAAAAA);
	MAC_REG_W32(R_AX_INDIR_ACCESS_ENTRY + 4, 0xBBBBBBBB);

	val32 = MAC_REG_R32(R_AX_INDIR_ACCESS_ENTRY);
	if (val32 != 0xAAAAAAAA) {
		ret = MACMEMRO;
		goto ind_aces_end;
	}

	val32 = MAC_REG_R32(R_AX_INDIR_ACCESS_ENTRY + 4);
	if (val32 != 0xBBBBBBBB) {
		ret = MACMEMRO;
		goto ind_aces_end;
	}

ind_aces_end:
	adapter->dbg_info.ind_aces_cnt--;
	PLTFM_MUTEX_UNLOCK(&adapter->lock_info.ind_access_lock);
	PLTFM_MSG_WARN("%s ind access end\n", __func__);
	if (ret != MACSUCCESS)
		return ret;

	content = (u8 *)PLTFM_MALLOC(len);
	if (!content) {
		PLTFM_MSG_ERR("[ERR]%s: malloc fail\n", __func__);
		return MACNOBUF;
	}

	PLTFM_MEMCPY(content, buffer, len);

	dl_size = len;
	target_addr = addr;

	PLTFM_MSG_WARN("%s ind access trg 0x%X start\n", __func__, target_addr);
	PLTFM_MUTEX_LOCK(&adapter->lock_info.ind_access_lock);
	adapter->dbg_info.ind_aces_cnt++;
	while (dl_size != 0) {
		MAC_REG_W32(R_AX_FILTER_MODEL_ADDR, target_addr);
		write_addr = R_AX_INDIR_ACCESS_ENTRY;

		if (dl_size >= ROMDL_SEG_LEN)
			seg_size = ROMDL_SEG_LEN;
		else
			seg_size = dl_size;

		seg_bytes = seg_size;

		while (seg_bytes != 0) {
			val32 = *((u32 *)(content + index));
			MAC_REG_W32(write_addr,
				    cpu_to_le32(val32));

			seg_bytes -= 4;
			write_addr += 4;
			index += 4;
		}

		target_addr += seg_size;
		dl_size -= seg_size;
	}
	adapter->dbg_info.ind_aces_cnt--;
	PLTFM_MUTEX_UNLOCK(&adapter->lock_info.ind_access_lock);
	PLTFM_MSG_WARN("%s ind access trg 0x%X end\n", __func__, target_addr);

	PLTFM_FREE(content, len);

	return MACSUCCESS;
}

static u32 fwdl_phase0(struct mac_ax_adapter *adapter)
{
	u32 cnt = FWDL_WAIT_CNT;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	if (adapter->sm.fwdl != MAC_AX_FWDL_CPU_ON) {
		PLTFM_MSG_ERR("[ERR]%s: state != CPU_ON\n", __func__);
		return MACPROCERR;
	}

	while (--cnt) {
		if (MAC_REG_R8(R_AX_WCPU_FW_CTRL) & B_AX_H2C_PATH_RDY)
			break;
		PLTFM_DELAY_US(1);
	}

	if (!cnt) {
		PLTFM_MSG_ERR("[ERR]%s: poll 0x1E0[1] = 1 fail\n", __func__);
		return MACPOLLTO;
	}

	adapter->sm.fwdl = MAC_AX_FWDL_H2C_PATH_RDY;
#if MAC_FWDL_TIME_PROFILING
	FWDL_TIMESTAMP_BUFF.ts1 = FWDL_CURRENT_TIME_US();
#endif
	PLTFM_MSG_ALWAYS("[FWDL] WCPU PRE BOOT..................................DONE\n");

	return MACSUCCESS;
}

static u32 fwdl_phase1(struct mac_ax_adapter *adapter,
		       u8 *fw, u32 hdr_len, u8 redl)
{
	u32 ret;
	u32 cnt = FWDL_WAIT_CNT;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	if (adapter->sm.fwdl != MAC_AX_FWDL_H2C_PATH_RDY) {
		PLTFM_MSG_ERR("[ERR]%s: state != H2C_PATH_RDY\n", __func__);
		return MACPROCERR;
	}

	ret = __fwhdr_download(adapter, fw, hdr_len, redl);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: __fwhdr_download fail\n", __func__);
		return ret;
	}

	while (--cnt) {
		if (MAC_REG_R8(R_AX_WCPU_FW_CTRL) & B_AX_FWDL_PATH_RDY)
			break;
		PLTFM_DELAY_US(1);
	}

	if (!cnt) {
		PLTFM_MSG_ERR("[ERR]%s: poll 0x1E0[2] = 1 fail\n", __func__);
		return MACPOLLTO;
	}

	MAC_REG_W32(R_AX_HALT_H2C_CTRL, 0);
	MAC_REG_W32(R_AX_HALT_C2H_CTRL, 0);

	adapter->sm.fwdl = MAC_AX_FWDL_PATH_RDY;
#if MAC_FWDL_TIME_PROFILING
	FWDL_TIMESTAMP_BUFF.ts2 = FWDL_CURRENT_TIME_US();
#endif
	PLTFM_MSG_ALWAYS("[FWDL] WCPU IMAGE HEADER SEND........................DONE\n");

	return MACSUCCESS;
}

void check_ser_status(struct mac_ax_adapter *adapter, const char *caller)
{
	u32 val32;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852A) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8851E) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
#if (MAC_AX_8852A_SUPPORT || MAC_AX_8852B_SUPPORT || \
MAC_AX_8851E_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT)
		val32 = MAC_REG_R32(R_AX_HALT_C2H);
		if (val32) {
			PLTFM_MSG_WARN("[FWDL][WARN] L2 Debug not empty after %s\n", caller);
			PLTFM_MSG_WARN("[FWDL][WARN] (0x%X) = 0x%X\n", R_AX_HALT_C2H, val32);
		}

		val32 = MAC_REG_R32(R_AX_UDM1);
		if (val32) {
			PLTFM_MSG_WARN("[FWDL][WARN] L2 Debug not empty after %s\n", caller);
			PLTFM_MSG_WARN("[FWDL][WARN] (0x%X) = 0x%X\n", R_AX_UDM1, val32);
		}

		val32 = MAC_REG_R32(R_AX_UDM2);
		if (val32) {
			PLTFM_MSG_WARN("[FWDL][WARN] L2 Debug not empty after %s\n", caller);
			PLTFM_MSG_WARN("[FWDL][WARN] (0x%X) = 0x%X\n", R_AX_UDM2, val32);
		}

		val32 = MAC_REG_R32(R_AX_HD0ISR);
		if (val32) {
			PLTFM_MSG_WARN("[FWDL][WARN]R_AX_HD0ISR not empty after %s\n", caller);
			PLTFM_MSG_WARN("[FWDL][WARN] (0x%X) = 0x%X\n", R_AX_HD0ISR, val32);
			if (val32 & B_AX_WDT_PTFM_INT) {
				PLTFM_MSG_WARN("[FWDL][WARN] clear WDT\n");
				MAC_REG_W32(R_AX_HD0ISR, B_AX_WDT_PTFM_INT);
			}
		}
#endif
	} else if (is_chip_id(adapter, MAC_BE_CHIP_ID_1115E) ||
		   is_chip_id(adapter, MAC_BE_CHIP_ID_8922A) ||
		   is_chip_id(adapter, MAC_BE_CHIP_ID_8952A) ||
		   is_chip_id(adapter, MAC_AX_CHIP_ID_8852C) ||
		   is_chip_id(adapter, MAC_AX_CHIP_ID_8852D) ||
		   is_chip_id(adapter, MAC_AX_CHIP_ID_8192XB)) {
#if MAC_AX_8852C_SUPPORT || MAC_AX_8192XB_SUPPORT || MAC_AX_8852D_SUPPORT
		val32 = MAC_REG_R32(R_AX_HALT_C2H);
		if (val32) {
			PLTFM_MSG_WARN("[FWDL][WARN] L2 Debug not empty after %s\n", caller);
			PLTFM_MSG_WARN("[FWDL][WARN] (0x%X) = 0x%X\n", R_AX_HALT_C2H, val32);
		}

		val32 = MAC_REG_R32(R_AX_UDM1);
		if (val32) {
			PLTFM_MSG_WARN("[FWDL][WARN] L2 Debug not empty after %s\n", caller);
			PLTFM_MSG_WARN("[FWDL][WARN] (0x%X) = 0x%X\n", R_AX_UDM1, val32);
		}

		val32 = MAC_REG_R32(R_AX_UDM2);
		if (val32) {
			PLTFM_MSG_WARN("[FWDL][WARN] L2 Debug not empty after %s\n", caller);
			PLTFM_MSG_WARN("[FWDL][WARN] (0x%X) = 0x%X\n", R_AX_UDM2, val32);
		}

		val32 = MAC_REG_R32(R_AX_HISR0);
		if (val32) {
			PLTFM_MSG_WARN("[FWDL][WARN]R_AX_HISR0 not empty after %s\n", caller);
			PLTFM_MSG_WARN("[FWDL][WARN] (0x%X) = 0x%X\n", R_AX_HISR0, val32);
			if (val32 & B_AX_WDT_TIMEOUT_INT) {
				PLTFM_MSG_WARN("[FWDL][WARN] clear WDT\n");
				MAC_REG_W32(R_AX_HISR0, B_AX_WDT_TIMEOUT_INT);
			}
		}
#endif
#if MAC_BE_1115E_SUPPORT || MAC_BE_8922A_SUPPORT || MAC_BE_8952A_SUPPORT
		val32 = MAC_REG_R32(R_BE_HALT_C2H);
		if (val32) {
			PLTFM_MSG_WARN("[FWDL][WARN] L2 Debug not empty after %s\n", caller);
			PLTFM_MSG_WARN("[FWDL][WARN] (0x%X) = 0x%X\n", R_BE_HALT_C2H, val32);
		}

		val32 = MAC_REG_R32(R_BE_UDM1);
		if (val32) {
			PLTFM_MSG_WARN("[FWDL][WARN] L2 Debug not empty after %s\n", caller);
			PLTFM_MSG_WARN("[FWDL][WARN] (0x%X) = 0x%X\n", R_BE_UDM1, val32);
		}

		val32 = MAC_REG_R32(R_BE_UDM2);
		if (val32) {
			PLTFM_MSG_WARN("[FWDL][WARN] L2 Debug not empty after %s\n", caller);
			PLTFM_MSG_WARN("[FWDL][WARN] (0x%X) = 0x%X\n", R_BE_UDM2, val32);
		}

		val32 = MAC_REG_R32(R_BE_HISR0);
		if (val32) {
			PLTFM_MSG_WARN("[FWDL][WARN]R_BE_HISR0 not empty after %s\n", caller);
			PLTFM_MSG_WARN("[FWDL][WARN](0x%X) = 0x%X\n", R_BE_HISR0, val32);
			if (val32 & B_BE_WDT_TIMEOUT_INT) {
				PLTFM_MSG_WARN("[FWDL][WARN]clear WDT\n");
				MAC_REG_W32(R_BE_HISR0, B_BE_WDT_TIMEOUT_INT);
			}
		}
#endif
	}
}

static inline void fwdl_fw_debug_status_log(struct mac_ax_adapter *adapter, u16 debug_step)
{
	const char **cur_fwdl_debug_step;

	//For performance reason, do not change to if(IS_XXXX)
	switch (adapter->hw_info->chip_id) {
#if FWDL_SUPPORT_8852A
	case MAC_AX_CHIP_ID_8852A:
		cur_fwdl_debug_step = fwdl_8852a_debug_step;
		break;
#endif
#if FWDL_SUPPORT_8852B || FWDL_SUPPORT_8851B || FWDL_SUPPORT_8852BT
	case MAC_AX_CHIP_ID_8852B:
	case MAC_AX_CHIP_ID_8851B:
	case MAC_AX_CHIP_ID_8852BT:
		cur_fwdl_debug_step = fwdl_8852b_debug_step;
		break;
#endif
#if FWDL_SUPPORT_8852C || FWDL_SUPPORT_8852D || FWDL_SUPPORT_8192XB
	case MAC_AX_CHIP_ID_8852C:
	case MAC_AX_CHIP_ID_8852D:
	case MAC_AX_CHIP_ID_8192XB:
		cur_fwdl_debug_step = fwdl_8852c_debug_step;
		break;
#endif
#if FWDL_SUPPORT_8851E
	case MAC_AX_CHIP_ID_8851E:
		PLTFM_MSG_ALWAYS("[FWDL][FW Debug Step] 0x%X\n", debug_step);
		return;
#endif
	default:
		PLTFM_MSG_ALWAYS("[FWDL][FW Debug Step] 0x%X\n", debug_step);
		return;
	}

	PLTFM_MSG_ALWAYS("[FWDL] %s (0x%X)\n", cur_fwdl_debug_step[debug_step],
			 debug_step);
}

static void fwdl_show_fwdlpass_text(struct mac_ax_adapter *adapter)
{
	PLTFM_MSG_ALWAYS("[FWDL] WCPU RAM PATCH.................................DONE\n");
	PLTFM_MSG_ALWAYS(" _____      _____  _      ___  _   ___ ___\n");
	PLTFM_MSG_ALWAYS("| __\\ \\    / /   \\| |    | _ \\/_\\ / __/ __|\n");
	PLTFM_MSG_ALWAYS("| _| \\ \\/\\/ /| |) | |__  |  _/ _ \\\\__ \\__ \\\n");
	PLTFM_MSG_ALWAYS("|_|   \\_/\\_/ |___/|____| |_|/_/ \\_\\___/___/\n");
}

static u32 check_fw_rdy(struct mac_ax_adapter *adapter)
{
	u32 pre_val8;
	u32 val8 = FWDL_INITIAL_STATE;
	u32 cnt = FWDL_WAIT_CNT;
	u8 fwdl_done = 0;
	u32 pre_bootstep = 0, cur_bootstep = 0, pre_secure_step = 0, cur_secure_step = 0;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

#if FWDL_AX_MIPS_SUPPORT
	if (FWDL_IS_AX_MIPS) {
		pre_bootstep = GET_FIELD(MAC_REG_R32(R_AX_BOOT_DBG), B_AX_BOOT_STATUS);
		pre_secure_step = GET_FIELD(MAC_REG_R32(R_AX_BOOT_DBG), B_AX_SECUREBOOT_STATUS);
	}
#endif
#if FWDL_AX_RISCV_SUPPORT
	if (FWDL_IS_AX_RISCV) {
		pre_bootstep = GET_FIELD(MAC_REG_R32(R_AX_BOOT_DBG_V1), B_AX_BOOT_STATUS);
		pre_secure_step = GET_FIELD(MAC_REG_R32(R_AX_BOOT_DBG_V1), B_AX_SECUREBOOT_STATUS);
	}
#endif
	pre_val8 = GET_FIELD(MAC_REG_R8(R_AX_WCPU_FW_CTRL),
			     B_AX_WCPU_FWDL_STS);

	while (--cnt) {
#if FWDL_AX_MIPS_SUPPORT
		if (FWDL_IS_AX_MIPS) {
			cur_bootstep = GET_FIELD(MAC_REG_R32(R_AX_BOOT_DBG), B_AX_BOOT_STATUS);
			cur_secure_step = GET_FIELD(MAC_REG_R32(R_AX_BOOT_DBG),
						    B_AX_SECUREBOOT_STATUS);
		}
#endif
#if FWDL_AX_RISCV_SUPPORT
		if (FWDL_IS_AX_RISCV) {
			cur_bootstep = GET_FIELD(MAC_REG_R32(R_AX_BOOT_DBG_V1), B_AX_BOOT_STATUS);
			cur_secure_step = GET_FIELD(MAC_REG_R32(R_AX_BOOT_DBG_V1),
						    B_AX_SECUREBOOT_STATUS);
		}
#endif
		val8 = GET_FIELD(MAC_REG_R8(R_AX_WCPU_FW_CTRL),
				 B_AX_WCPU_FWDL_STS);
		if (cur_bootstep != pre_bootstep) {
			fwdl_fw_debug_status_log(adapter, (u16)cur_bootstep);
			pre_bootstep = cur_bootstep;
			if (pre_bootstep >= FWDL_DBGSTEP_FW_BOOT_RAM_LOADED && !fwdl_done) {
				fwdl_done = 1;
				fwdl_show_fwdlpass_text(adapter);
			}
#if MAC_FWDL_TIME_PROFILING
			if (!FWDL_TIMESTAMP_BUFF.ts3 &&
			    pre_bootstep >= FWDL_DBGSTEP_FW_BOOT_DLFW_DONE)
				FWDL_TIMESTAMP_BUFF.ts3 = FWDL_CURRENT_TIME_US();
			if (!FWDL_TIMESTAMP_BUFF.ts4 &&
			    pre_bootstep >= FWDL_DBGSTEP_FW_BOOT_RAM_LOADED)
				FWDL_TIMESTAMP_BUFF.ts4 = FWDL_CURRENT_TIME_US();
#endif
		}
		if (cur_secure_step != pre_secure_step) {
			PLTFM_MSG_TRACE("[TRACE]%s: SECURE_STEP 0x%x -> 0x%x\n",
					__func__, pre_secure_step, cur_secure_step);
			pre_secure_step = cur_secure_step;
		}
		if (val8 != pre_val8) {
			PLTFM_MSG_TRACE("[TRACE]%s: 0x1E0[7:5] 0x%x -> 0x%x\n",
					__func__, pre_val8, val8);
			pre_val8 = val8;
		}
		if (val8 == FWDL_WCPU_FW_INIT_RDY) {
#if MAC_FWDL_TIME_PROFILING
			FWDL_TIMESTAMP_BUFF.ts5 = FWDL_CURRENT_TIME_US();
#endif
			break;
		} else if (val8 == FWDL_CHECKSUM_FAIL) {
			PLTFM_MSG_ERR("[ERR]%s: FWDL_DISPATCHER_CHECKSUM_FAIL\n", __func__);
			PLTFM_MSG_ERR("[ERR]%s: 0x1E0[7:5] last value = %d\n", __func__, val8);
			return MACFWCHKSUM;
		} else if (val8 == FWDL_SECURITY_FAIL) {
			PLTFM_MSG_ERR("[ERR]%s: FWDL_SECURITY_FAIL\n", __func__);
			PLTFM_MSG_ERR("[ERR]%s: 0x1E0[7:5] last value = %d\n", __func__, val8);
			return MACFWSECBOOT;
		} else if (val8 == FWDL_CUT_NOT_MATCH) {
			PLTFM_MSG_ERR("[ERR]%s: FWDL_CUT_NOT_MATCH\n", __func__);
			PLTFM_MSG_ERR("[ERR]%s: 0x1E0[7:5] last value = %d\n", __func__, val8);
			return MACFWCUT;
		}
		PLTFM_DELAY_US(1);
	}

	if (!cnt) {
		PLTFM_MSG_ERR("[ERR]%s: Polling 0x1E0[7:5] == 7 Timeout\n", __func__);
		PLTFM_MSG_ERR("[ERR]%s: 0x1E0[7:5] last value = %d\n", __func__, val8);
		return MACPOLLTO;
	}

	check_ser_status(adapter, __func__);
	adapter->sm.fwdl = MAC_AX_FWDL_INIT_RDY;
	PLTFM_MSG_ALWAYS("[FWDL] WCPU FREERTOS INIT.............................DONE\n");
	PLTFM_MSG_ALWAYS("  ___ _____ ___  ___   ___  ___   ___ _____\n");
	PLTFM_MSG_ALWAYS(" | _ |_   _/ _ \\/ __| | _ )/ _ \\ / _ |_   _|\n");
	PLTFM_MSG_ALWAYS(" |   / | || (_) \\__ \\ | _ | (_) | (_) || |\n");
	PLTFM_MSG_ALWAYS(" |_|_\\ |_| \\___/|___/ |___/\\___/ \\___/ |_|\n");

	return MACSUCCESS;
}

static u32 fwdl_phase2(struct mac_ax_adapter *adapter, u8 *fw,
		       struct fw_bin_info *info, u8 redl)
{
	u32 ret;
	u32 section_num = info->section_num;
	u32 section_idx = 0;
	struct fwhdr_section_info *section_info = info->section_info;

	if (adapter->sm.fwdl != MAC_AX_FWDL_PATH_RDY) {
		PLTFM_MSG_ERR("[ERR]%s: state != FWDL_PATH_RDY\n", __func__);
		return MACPROCERR;
	}

	for (section_idx = 0; section_idx < section_num; section_idx++) {
		PLTFM_MSG_ALWAYS("[FWDL] SEC_%d:Len=0x%X,Addr=0x%X,Type=0x%X\n",
				 section_idx, section_info->len, section_info->dladdr,
				 section_info->type);
		if (!redl) {
			ret = __sections_download(adapter, section_info);
			if (ret) {
				PLTFM_MSG_ERR("[ERR]%s: ", __func__);
				PLTFM_MSG_ERR("__sections_download fail\n");
				return ret;
			}
		} else {
			if (section_info->redl) {
				ret = __sections_download(adapter,
							  section_info);
				if (ret) {
					PLTFM_MSG_ERR("[ERR]%s: ", __func__);
					PLTFM_MSG_ERR("__sections_download ");
					PLTFM_MSG_ERR("fail\n");
					return ret;
				}
			}
		}
		section_info++;
	}

	ret = check_fw_rdy(adapter);
	if (ret) {
		PLTFM_MSG_ERR("%s: check_fw_rdy fail\n", __func__);
		return ret;
	}
	return MACSUCCESS;
}

static void fwdl_fail_dump(struct mac_ax_adapter *adapter,
			   struct fw_bin_info *info, u32 ret)
{
	PLTFM_MSG_ERR("[ERR]fwdl ret = %d\n", ret);
	PLTFM_MSG_ERR("[ERR]fw git idx = 0x%x\n", info->git_idx);

	ret = fw_st_dbg_dump(adapter);
	if (ret != MACSUCCESS)
		PLTFM_MSG_ERR("fw st dump fail %d\n", ret);

	pltfm_dbg_dump(adapter);
}

u8 mac_fwredl_needed(struct mac_ax_adapter *adapter)
{
	if (adapter->sm.fwdl != MAC_AX_FWDL_INIT_RDY) {
		PLTFM_MSG_TRACE("[FWDL][INFO] Not Yet FWDL\n");
		return false;
	}
	//FWREDL function is available only on 8852B
	if (!FWDL_IS_SUPPORT_REDL) {
		PLTFM_MSG_TRACE("[FWDL][INFO] FWREDL available only on 8852B or 8851B or 8852BT\n");
		return false;
	}
	if (!adapter->fw_info.last_fw_use_ple) {
		PLTFM_MSG_TRACE("[FWDL][INFO] Last FW Not Use PLE Section, REDL No Need.\n");
		return false;
	}
	return true;
}

static u32 send_h2c_reset_idmem_mode(struct mac_ax_adapter *adapter)
{
	u32 ret = MACSUCCESS;
	struct h2c_info h2c_info = { 0 };
	struct fwcmd_fw_redl_cfg *content;

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_fw_redl_cfg);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_FW_INFO;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_FW_REDL_CFG;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	content = (struct fwcmd_fw_redl_cfg *)PLTFM_MALLOC(h2c_info.content_len);

	if (!content) {
		PLTFM_MSG_ERR("%s: malloc fail\n", __func__);
		return MACNPTR;
	}
	content->dword0 = cpu_to_le32(FWCMD_H2C_FW_REDL_CFG_STOP_WATCHDOG |
				      FWCMD_H2C_FW_REDL_CFG_RESET_IDMEM_MODE);

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)content);
	PLTFM_FREE(content, h2c_info.content_len);

	return ret;
}

u32 mac_fwredl(struct mac_ax_adapter *adapter, u8 *fw, u32 len)
{
	u32 val32;
	u32 ret;
	struct fw_bin_info info;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	//FWREDL funtion is available only on 8852B
	if (!FWDL_IS_SUPPORT_REDL) {
		PLTFM_MSG_ERR("%s: FWREDL is available only on 8852B/8851B/8852BT\n",
			      __func__);
		return MACSUCCESS;
	}

	ret = fwhdr_parser(adapter, fw, len, &info);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: fwhdr_parser fail\n", __func__);
		goto fwdl_err;
	}

	if (!info.is_fw_use_ple) {
		PLTFM_MSG_WARN("[WARN]%s: no need to redownload\n", __func__);
		return MACSUCCESS;
	}
	ret = update_fw_ver(adapter, (struct fwhdr_hdr_t *)fw);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: update_fw_ver fail\n", __func__);
		goto fwdl_err;
	}

	val32 = MAC_REG_R32(R_AX_WCPU_FW_CTRL);
	val32 &= ~(B_AX_WCPU_FWDL_EN | B_AX_H2C_PATH_RDY | B_AX_FWDL_PATH_RDY);
	val32 = SET_CLR_WORD(val32, FWDL_INITIAL_STATE,
			     B_AX_WCPU_FWDL_STS);
	MAC_REG_W32(R_AX_WCPU_FW_CTRL, val32);

	adapter->sm.fwdl = MAC_AX_FWDL_H2C_PATH_RDY;

	ret = fwdl_phase1(adapter, fw, info.hdr_len - info.dynamic_hdr_len, 1);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: fwdl_phase1 fail\n", __func__);
		goto fwdl_err;
	}

	ret = fwdl_phase2(adapter, fw, &info, 1);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: fwdl_phase2 fail\n", __func__);
		goto fwdl_err;
	}

	mac_scanofld_reset_state(adapter);
	return MACSUCCESS;

fwdl_err:
	fwdl_fail_dump(adapter, &info, ret);

	return ret;
}

u32 mac_fwdl(struct mac_ax_adapter *adapter, u8 *fw, u32 len)
{
	u8 retry_cnt;
	u32 ret;
	struct fw_bin_info info;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	ret = 0;
	retry_cnt = 0;
	MAC_REG_W32(R_AX_UDM1, 0);
	if (!fw) {
		PLTFM_MSG_ERR("[ERR]%s: no fw\n", __func__);
		ret = MACNOFW;
		PLTFM_MEMSET(&info, 0, sizeof(struct fw_bin_info));
		return ret;
	}
	/* Move this fuinction outside the retry loop may be buggy.
	 * Since we've reed efuse in this function.
	 */
	ret = fwhdr_parser(adapter, fw, len, &info);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: fwhdr_parser fail\n", __func__);
		return ret;
	}
	PLTFM_MSG_ALWAYS("[FWDL] WCPU FW BIN CHECK.............................DONE\n");

	ret = update_fw_ver(adapter, (struct fwhdr_hdr_t *)fw);
	if (ret)
		return ret;

	// FWDL retry, for 025 temp workaround
	while (retry_cnt < FWDL_TRY_CNT) {
		ret = fwdl_phase0(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]%s: fwdl_phase0 fail\n", __func__);
			goto fwdl_err;
		}

		ret = fwdl_phase1(adapter, fw, info.hdr_len - info.dynamic_hdr_len, 0);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]%s: fwdl_phase1 fail\n", __func__);
			goto fwdl_err;
		}

		ret = fwdl_phase2(adapter, fw, &info, 0);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]%s: fwdl_phase2 fail\n", __func__);
			goto fwdl_err;
		}

		ret = mac_get_dynamic_hdr_ax(adapter, fw, len);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]%s: mac_get_dynamic_hdr_ax fail\n", __func__);
			goto fwdl_err;
		}

#if MAC_FWDL_TIME_PROFILING
		fwdl_time_parsing(adapter);
#endif

		return MACSUCCESS;

fwdl_err:
		retry_cnt++;
		PLTFM_MSG_ERR("[ERR]%s: Retry FWDL count %d\n", __func__, retry_cnt);
		// At most retry 2 times
		if (retry_cnt < FWDL_TRY_CNT) {
			ret = mac_disable_cpu(adapter);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("[ERR]%s: mac_disable_cpu fail\n", __func__);
				fwdl_fail_dump(adapter, &info, ret);
				return ret;
			}

			ret = mac_enable_cpu(adapter, AX_BOOT_REASON_PWR_ON, 1);
			if (ret != MACSUCCESS) {
				PLTFM_MSG_ERR("[ERR]%s: mac_enable_cpu fail\n", __func__);
				fwdl_fail_dump(adapter, &info, ret);
				return ret;
			}
			MAC_REG_W32(R_AX_UDM1, retry_cnt);
		} else {
			break;
		}
	}

	fwdl_fail_dump(adapter, &info, ret);

	return ret;
}

u32 mac_enable_cpu(struct mac_ax_adapter *adapter, u8 boot_reason, u8 dlfw)
{
	u32 val32, ret;
	u16 val16;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	if (MAC_REG_R32(R_AX_PLATFORM_ENABLE) & B_AX_WCPU_EN)
		return MACCPUSTATE;

	if (adapter->sm.fwdl != MAC_AX_FWDL_IDLE) {
		PLTFM_MSG_ERR("[ERR]%s: state != FWDL_IDLE\n", __func__);
		return MACPROCERR;
	}

	//FW cannot support too much log. Reset R_AX_LDM for FW debug config
	MAC_REG_W32(R_AX_LDM, 0);

	//Default open the debug mode
	val32 = MAC_REG_R32(R_AX_UDM0);
	//val32 |= BIT(0);
	//MAC_REG_W32(R_AX_UDM0, val32);

	//Clear SER status
	MAC_REG_W32(R_AX_HALT_H2C_CTRL, 0);
	MAC_REG_W32(R_AX_HALT_C2H_CTRL, 0);
	MAC_REG_W32(R_AX_HALT_H2C, 0);
	MAC_REG_W32(R_AX_HALT_C2H, 0);
	//Clear SER status end

	// write 1 clear for R_AX_HISR0(HALT_C2H ISR)
	val32 = MAC_REG_R32(R_AX_HISR0);
	MAC_REG_W32(R_AX_HISR0, val32);

	MAC_REG_W32(R_AX_SYS_CLK_CTRL,
		    MAC_REG_R32(R_AX_SYS_CLK_CTRL) | B_AX_CPU_CLK_EN);

	val32 = MAC_REG_R32(R_AX_WCPU_FW_CTRL);
	val32 &= ~(B_AX_WCPU_FWDL_EN | B_AX_H2C_PATH_RDY | B_AX_FWDL_PATH_RDY);
	val32 = SET_CLR_WORD(val32, FWDL_INITIAL_STATE,
			     B_AX_WCPU_FWDL_STS);

	if (dlfw)
		val32 |= B_AX_WCPU_FWDL_EN;

	MAC_REG_W32(R_AX_WCPU_FW_CTRL, val32);

	val16 = MAC_REG_R16(R_AX_BOOT_REASON);
	val16 = SET_CLR_WORD(val16, boot_reason, B_AX_BOOT_REASON);
	MAC_REG_W16(R_AX_BOOT_REASON, val16);

	ret = fwdl_precheck(adapter);
	if (ret)
		return ret;

	val32 = MAC_REG_R32(R_AX_PLATFORM_ENABLE);
	MAC_REG_W32(R_AX_PLATFORM_ENABLE, val32 | B_AX_WCPU_EN);
#if MAC_FWDL_TIME_PROFILING
	FWDL_TIMESTAMP_BUFF.ts0 = FWDL_CURRENT_TIME_US();
#endif

	adapter->sm.fwdl = MAC_AX_FWDL_CPU_ON;

	if (!dlfw) {
		PLTFM_DELAY_MS(5);

		ret = check_fw_rdy(adapter);
		if (ret) {
			PLTFM_MSG_ERR("[ERR]%s: ", __func__);
			PLTFM_MSG_ERR("check_fw_rdy fail\n");
			return ret;
		}
	}

	// Prevent sequence number in HALMAC and FW mismatching
	reset_lps_seq_num(adapter);

	return MACSUCCESS;
}

u32 mac_disable_cpu(struct mac_ax_adapter *adapter)
{
	u32 val32;

	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	adapter->sm.fwdl = MAC_AX_FWDL_IDLE;
#if MAC_FWDL_TIME_PROFILING
	PLTFM_MEMSET(&FWDL_TIMESTAMP_BUFF, 0, sizeof(struct mac_fwdl_timestamp));
#endif
	//todo: need to check cpu in safe state before reset CPU

	val32 = MAC_REG_R32(R_AX_PLATFORM_ENABLE);
	MAC_REG_W32(R_AX_PLATFORM_ENABLE, val32 & ~B_AX_WCPU_EN);

	val32 = MAC_REG_R32(R_AX_WCPU_FW_CTRL);
	val32 &= ~(B_AX_WCPU_FWDL_EN | B_AX_H2C_PATH_RDY | B_AX_FWDL_PATH_RDY);
	MAC_REG_W32(R_AX_WCPU_FW_CTRL, val32);

	val32 = MAC_REG_R32(R_AX_SYS_CLK_CTRL);
	MAC_REG_W32(R_AX_SYS_CLK_CTRL, val32 & ~B_AX_CPU_CLK_EN);

	/* Disable WDT by Reset CPU CR
	 * Reset B_AX_APB_WRAP_EN will reset all CPU Local CR.
	 * Although reset B_AX_PLATFORM_EN will have same effect.
	 * This redundant flow represent as an explicit point,
	 * where we've reseted CPU Local CR, including WDT.
	 *
	 * After 52C, Disable WCPU Will Also Disable WDT
	 * So only 52A, 52B and 51B need to reset B_AX_APB_WRAP_EN
	 */
#if FWDL_RESET_APB_WRAP_SUPPORT
	if (FWDL_IS_RESET_APB_WRAP) {
		val32 = MAC_REG_R32(R_AX_PLATFORM_ENABLE);
		MAC_REG_W32(R_AX_PLATFORM_ENABLE, val32 & ~B_AX_APB_WRAP_EN);

		val32 = MAC_REG_R32(R_AX_PLATFORM_ENABLE);
		MAC_REG_W32(R_AX_PLATFORM_ENABLE, val32 | B_AX_APB_WRAP_EN);
	}
#endif

	adapter->sm.plat = MAC_AX_PLAT_OFF;

	if (chk_patch_pcie_hang(adapter)) {
		val32 = MAC_REG_R32(R_AX_PLATFORM_ENABLE);
		MAC_REG_W32(R_AX_PLATFORM_ENABLE, val32 & ~B_AX_PLATFORM_EN);

		val32 = MAC_REG_R32(R_AX_PLATFORM_ENABLE);
		MAC_REG_W32(R_AX_PLATFORM_ENABLE, val32 | B_AX_PLATFORM_EN);
	}

	adapter->sm.plat = MAC_AX_PLAT_ON;

	PLTFM_MSG_ALWAYS("[FWDL] WCPU OFF......................................DONE\n");
	return MACSUCCESS;
}

u32 mac_romdl(struct mac_ax_adapter *adapter, u8 *ROM, u32 ROM_addr, u32 len)
{
	u8 *content = NULL;
	u32 val32, ret;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	ret = mac_disable_cpu(adapter);
	if (ret)
		return ret;

	if (!ROM)
		return MACNOITEM;

	val32 = MAC_REG_R32(R_AX_SEC_CTRL);

	if (val32 & BIT(0)) {
		ret = __write_memory(adapter, ROM, ROM_addr, len);
		if (ret)
			return ret;
	} else {
		PLTFM_MSG_ERR("[ERR]%s: __write_memory fail\n", __func__);
		return MACSECUREON;
	}

	PLTFM_FREE(content, len);

	return MACSUCCESS;
}

u32 mac_ram_boot(struct mac_ax_adapter *adapter, u8 *fw, u32 len)
{
	u32 addr;
	u32 ret = 0, section_num = 1;
	struct fw_bin_info info = {0};
	struct fwhdr_section_info *section_info;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	if (adapter->hw_info->is_sec_ic) {
		PLTFM_MSG_ERR("[ERR]security mode ind accees\n");
		PLTFM_MSG_ERR("[ERR]Abort %s.\n", __func__);
		return MACIOERRINSEC;
	}

	ret = mac_disable_cpu(adapter);
	if (ret)
		goto fwdl_err;

	ret = fwhdr_parser(adapter, fw, len, &info);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: fwhdr_parser fail\n", __func__);
		goto fwdl_err;
	}

	ret = update_fw_ver(adapter, (struct fwhdr_hdr_t *)fw);
	if (ret)
		goto fwdl_err;

	section_num = info.section_num;
	section_info = info.section_info;

	while (section_num > 0) {
		ret = __write_memory(adapter, section_info->addr,
				     section_info->dladdr, section_info->len);
		if (ret)
			goto fwdl_err;

		section_info++;
		section_num--;
	}

	addr = (0xb8003000 + R_AX_CPU_BOOT_ADDR) & 0x1FFFFFFF;
	PLTFM_MSG_WARN("%s ind access 0x%X start\n", __func__, addr);
	PLTFM_MUTEX_LOCK(&adapter->lock_info.ind_access_lock);
	adapter->dbg_info.ind_aces_cnt++;
#if FWDL_AX_MIPS_SUPPORT
	if (FWDL_IS_AX_MIPS)
		MAC_REG_W32(R_AX_INDIR_ACCESS_ENTRY, 0xB8970000);
#endif
#if FWDL_AX_RISCV_SUPPORT
	if (FWDL_IS_AX_RISCV)
		MAC_REG_W32(R_AX_INDIR_ACCESS_ENTRY, 0x20100000);
#endif
	adapter->dbg_info.ind_aces_cnt--;
	PLTFM_MUTEX_UNLOCK(&adapter->lock_info.ind_access_lock);
	PLTFM_MSG_WARN("%s ind access 0x%X end\n", __func__, addr);

	ret = mac_enable_cpu(adapter, AX_BOOT_REASON_PWR_ON, 0);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: mac_enable_cpu fail\n", __func__);
		goto fwdl_err;
	}

	PLTFM_DELAY_MS(10);

	ret = check_fw_rdy(adapter);
	if (ret) {
		PLTFM_MSG_ERR("[ERR]%s: check_fw_rdy fail\n", __func__);
		goto fwdl_err;
	}
	return MACSUCCESS;

fwdl_err:
	fwdl_fail_dump(adapter, &info, ret);

	return ret;
}

u32 mac_enable_fw(struct mac_ax_adapter *adapter, enum rtw_fw_type cat)
{
	u32 ret = MACSUCCESS;
#if defined(PHL_FEATURE_AP) || defined(PHL_FEATURE_NIC)
	u32 fw_len = 0;
	u8 *fw = NULL;
	enum DLE_RSVD_INFO dle_info;

	PLTFM_MSG_ALWAYS("[FWDL] FW INIT......................................START\n");

	ret = get_dle_rsvd_info(adapter, (enum DLE_RSVD_INFO *)&dle_info);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s: ", __func__);
		PLTFM_MSG_ALWAYS("[FWDL] FW INIT.......................................FAIL\n");
		return ret;
	}

	PLTFM_MSG_ALWAYS("Downloading Chip Halmac ID: 0x%02X\n", adapter->hw_info->chip_id);

	ret = mac_query_fw_buff(adapter, cat, &fw, &fw_len);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s: fw selection fail\n", __func__);
		PLTFM_MSG_ALWAYS("[FWDL] FW INIT.......................................FAIL\n");
		return ret;
	}

	ret = mac_disable_cpu(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s: mac_disable_cpu fail\n", __func__);
		PLTFM_MSG_ALWAYS("[FWDL] FW INIT.......................................FAIL\n");
		return ret;
	}

	PLTFM_MSG_ALWAYS("[FWDL] WCPU FWDL Boot.\n");

	/* _patch_otp_power_issue checks whether OTP arbiter switchs to WLAN. */
	/* If return err, print log but not return ret. */
	/* Re-DLFW executed by following mac_fwdl_ax can handle this issue. */
	if (chk_patch_otp_power_issue(adapter)) {
		ret = _patch_otp_power_issue(adapter);
		if (ret != MACSUCCESS)
			PLTFM_MSG_WARN("[WARN]%s: _patch_otp_power_issue fail\n", __func__);
	}

	idmem_share_mode_check(adapter, fw);

	ret = mac_enable_cpu(adapter, AX_BOOT_REASON_PWR_ON, 1);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s: mac_enable_cpu fail\n", __func__);
		PLTFM_MSG_ALWAYS("[FWDL] FW INIT.......................................FAIL\n");
		return ret;
	}

	ret = mac_fwdl(adapter, fw, fw_len);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]%s: mac_enable_cpu fail\n", __func__);
		PLTFM_MSG_ALWAYS("[FWDL] FW INIT.......................................FAIL\n");
		return ret;
	}
	mac_scanofld_reset_state(adapter);

#endif /* #if defined(PHL_FEATURE_AP) || defined(PHL_FEATURE_NIC) */
	PLTFM_MSG_ALWAYS("[FWDL] FW INIT.......................................DONE\n");
	return ret;
}

u32 mac_query_fw_buff(struct mac_ax_adapter *adapter, enum rtw_fw_type cat, u8 **fw, u32 *fw_len)
{
	u32 ret = MACSUCCESS;
	u32 chip_id, cv;
	enum DLE_RSVD_INFO dle_info;

	chip_id = adapter->hw_info->chip_id;
	cv = adapter->hw_info->cv;
	PLTFM_MSG_ALWAYS("[FWDL] Query Internal FW Chip_ID = 0x%X\n", chip_id);
	PLTFM_MSG_ALWAYS("[FWDL] Query Internal FW CV = 0x%X\n", cv);
	PLTFM_MSG_ALWAYS("[FWDL] Query Internal FW cat = 0x%X\n", cat);

	ret = get_dle_rsvd_info(adapter, (enum DLE_RSVD_INFO *)&dle_info);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[FWDL][ERROR][%s] get_dle_rsvd_info Fail.\n", __func__);
		return ret;
	}

	*fw = FWDL_NO_INTERNAL_FW;
	*fw_len = FWDL_NO_INTERNAL_FW;

	// Due To Halmac Code Rule Checker, Do Not change siwtch (cv) to if,else case
	if (FWDL_IS_8852A) { /**************** 8852A Internal FW Block ****************/
		switch (cv) {
		case FWDL_CBV:
			if (cat == RTW_FW_AP) {
				*fw = INTERNAL_FW_CONTENT_8852A_CBV_AP;
				*fw_len = INTERNAL_FW_LEN_8852A_CBV_AP;
			} else if (cat == RTW_FW_NIC) {
				*fw = INTERNAL_FW_CONTENT_8852A_CBV_NIC;
				*fw_len = INTERNAL_FW_LEN_8852A_CBV_NIC;
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8852A_CBV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8852A_CBV_WOWLAN;
			}
			break;
		case FWDL_CCV:
			// fall through
		default:
			if (cat == RTW_FW_AP) {
				*fw = INTERNAL_FW_CONTENT_8852A_CCV_AP;
				*fw_len = INTERNAL_FW_LEN_8852A_CCV_AP;
			} else if (cat == RTW_FW_NIC) {
				*fw = INTERNAL_FW_CONTENT_8852A_CCV_NIC;
				*fw_len = INTERNAL_FW_LEN_8852A_CCV_NIC;
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8852A_CCV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8852A_CCV_WOWLAN;
			}
			break;
		}
	} else if (FWDL_IS_8852B) { /**************** 8852B Internal FW Block ****************/
		switch (cv) {
		case FWDL_CBV:
			if (cat == RTW_FW_NIC) {
				if (dle_info == DLE_RSVD_INFO_FW) {
					PLTFM_MSG_WARN("[FWDL][WARN] PLE FW is not in use\n");
					PLTFM_MSG_WARN("[FWDL][WARN] Fallback to NIC FW\n");
				}
				*fw = INTERNAL_FW_CONTENT_8852B_CBV_NIC;
				*fw_len = INTERNAL_FW_LEN_8852B_CBV_NIC;
				// RTL8852BP Special Case
				if (PLTFM_GET_CHIP_ID(void) == CHIP_WIFI6_8852BP) {
					PLTFM_MSG_ALWAYS("[FWDL] 8852B Use NIC BPLUS FW\n");
					*fw = INTERNAL_FW_CONTENT_8852B_CBV_NIC_BPLUS;
					*fw_len = INTERNAL_FW_LEN_8852B_CBV_NIC_BPLUS;
				}
			} else if (cat == RTW_FW_NIC_CE) {
				*fw = INTERNAL_FW_CONTENT_8852B_CBV_NICCE;
				*fw_len = INTERNAL_FW_LEN_8852B_CBV_NICCE;
				// RTL8852BP Special Case
				if (PLTFM_GET_CHIP_ID(void) == CHIP_WIFI6_8852BP) {
					PLTFM_MSG_ALWAYS("[FWDL] 8852B Use NICCE BPLUS FW\n");
					*fw = INTERNAL_FW_CONTENT_8852B_CBV_NICCE_BPLUS;
					*fw_len = INTERNAL_FW_LEN_8852B_CBV_NICCE_BPLUS;
				}
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8852B_CBV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8852B_CBV_WOWLAN;
				// RTL8852BP Special Case
				if (PLTFM_GET_CHIP_ID(void) == CHIP_WIFI6_8852BP) {
					PLTFM_MSG_ALWAYS("[FWDL] 8852B Use WOWLAN BPLUS FW\n");
					*fw = INTERNAL_FW_CONTENT_8852B_CBV_WOWLAN_BPLUS;
					*fw_len = INTERNAL_FW_LEN_8852B_CBV_WOWLAN_BPLUS;
				}
			}
			break;
		case FWDL_CCV:
			// fall through
		default:
			if (cat == RTW_FW_NIC) {
				if (dle_info == DLE_RSVD_INFO_FW) {
					PLTFM_MSG_WARN("[FWDL][WARN] PLE FW is not in use\n");
					PLTFM_MSG_WARN("[FWDL][WARN] Fallback to NIC FW\n");
				}
				*fw = INTERNAL_FW_CONTENT_8852B_CCV_NIC;
				*fw_len = INTERNAL_FW_LEN_8852B_CCV_NIC;
				// RTL8852BP Special Case
				if (PLTFM_GET_CHIP_ID(void) == CHIP_WIFI6_8852BP) {
					PLTFM_MSG_ALWAYS("[FWDL] 8852B Use NIC BPLUS FW\n");
					*fw = INTERNAL_FW_CONTENT_8852B_CCV_NIC_BPLUS;
					*fw_len = INTERNAL_FW_LEN_8852B_CCV_NIC_BPLUS;
				}
			} else if (cat == RTW_FW_NIC_CE) {
				*fw = INTERNAL_FW_CONTENT_8852B_CCV_NICCE;
				*fw_len = INTERNAL_FW_LEN_8852B_CCV_NICCE;
				// RTL8852BP Special Case
				if (PLTFM_GET_CHIP_ID(void) == CHIP_WIFI6_8852BP) {
					PLTFM_MSG_ALWAYS("[FWDL] 8852B Use NICCE BPLUS FW\n");
					*fw = INTERNAL_FW_CONTENT_8852B_CCV_NICCE_BPLUS;
					*fw_len = INTERNAL_FW_LEN_8852B_CCV_NICCE_BPLUS;
				}
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8852B_CCV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8852B_CCV_WOWLAN;
				// RTL8852BP Special Case
				if (PLTFM_GET_CHIP_ID(void) == CHIP_WIFI6_8852BP) {
					PLTFM_MSG_ALWAYS("[FWDL] 8852B Use WOWLAN BPLUS FW\n");
					*fw = INTERNAL_FW_CONTENT_8852B_CCV_WOWLAN_BPLUS;
					*fw_len = INTERNAL_FW_LEN_8852B_CCV_WOWLAN_BPLUS;
				}
			}
			break;
		}
	} else if (FWDL_IS_8852C) {
		switch (cv) {
		case FWDL_CAV:
			if (cat == RTW_FW_AP) {
				*fw = INTERNAL_FW_CONTENT_8852C_CAV_AP;
				*fw_len = INTERNAL_FW_LEN_8852C_CAV_AP;
			} else if (cat == RTW_FW_NIC) {
				*fw = INTERNAL_FW_CONTENT_8852C_CAV_NIC;
				*fw_len = INTERNAL_FW_LEN_8852C_CAV_NIC;
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8852C_CAV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8852C_CAV_WOWLAN;
			}
			break;
		case FWDL_CBV:
			// fall through
		default:
			if (cat == RTW_FW_AP) {
				*fw = INTERNAL_FW_CONTENT_8852C_CBV_AP;
				*fw_len = INTERNAL_FW_LEN_8852C_CBV_AP;
			} else if (cat == RTW_FW_NIC) {
				*fw = INTERNAL_FW_CONTENT_8852C_CBV_NIC;
				*fw_len = INTERNAL_FW_LEN_8852C_CBV_NIC;
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8852C_CBV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8852C_CBV_WOWLAN;
			}
			break;
		}
	} else if (FWDL_IS_8192XB) {
		switch (cv) {
		case FWDL_CAV:
			if (cat == RTW_FW_AP) {
				*fw = INTERNAL_FW_CONTENT_8192XB_CAV_AP;
				*fw_len = INTERNAL_FW_LEN_8192XB_CAV_AP;
			} else if (cat == RTW_FW_NIC) {
				*fw = INTERNAL_FW_CONTENT_8192XB_CAV_NIC;
				*fw_len = INTERNAL_FW_LEN_8192XB_CAV_NIC;
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8192XB_CAV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8192XB_CAV_WOWLAN;
			}
			break;
		case FWDL_CBV:
			// fall through
		default:
			if (cat == RTW_FW_AP) {
				*fw = INTERNAL_FW_CONTENT_8192XB_CBV_AP;
				*fw_len = INTERNAL_FW_LEN_8192XB_CBV_AP;
			} else if (cat == RTW_FW_NIC) {
				*fw = INTERNAL_FW_CONTENT_8192XB_CBV_NIC;
				*fw_len = INTERNAL_FW_LEN_8192XB_CBV_NIC;
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8192XB_CBV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8192XB_CBV_WOWLAN;
			}
			break;
		}
	} else if (FWDL_IS_8851B) {
		switch (cv) {
		case FWDL_CAV:
			if (cat == RTW_FW_NIC) {
				if (dle_info == DLE_RSVD_INFO_FW) {
					PLTFM_MSG_WARN("[FWDL][WARN] PLE FW is not in use\n");
					PLTFM_MSG_WARN("[FWDL][WARN] Fallback to NIC FW\n");
				}
				*fw = INTERNAL_FW_CONTENT_8851B_CAV_NIC;
				*fw_len = INTERNAL_FW_LEN_8851B_CAV_NIC;
			} else if (cat == RTW_FW_NIC_CE) {
				*fw = INTERNAL_FW_CONTENT_8851B_CAV_NICCE;
				*fw_len = INTERNAL_FW_LEN_8851B_CAV_NICCE;
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8851B_CAV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8851B_CAV_WOWLAN;
			}
			break;
		case FWDL_CBV:
			// fall through
		default:
			if (cat == RTW_FW_NIC) {
				if (dle_info == DLE_RSVD_INFO_FW) {
					PLTFM_MSG_WARN("[FWDL][WARN] PLE FW is not in use\n");
					PLTFM_MSG_WARN("[FWDL][WARN] Fallback to NIC FW\n");
				}
				*fw = INTERNAL_FW_CONTENT_8851B_CBV_NIC;
				*fw_len = INTERNAL_FW_LEN_8851B_CBV_NIC;
			} else if (cat == RTW_FW_NIC_CE) {
				*fw = INTERNAL_FW_CONTENT_8851B_CBV_NICCE;
				*fw_len = INTERNAL_FW_LEN_8851B_CBV_NICCE;
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8851B_CBV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8851B_CBV_WOWLAN;
			}
			break;
		}
	} else if (FWDL_IS_8851E) {
		switch (cv) {
		case FWDL_CAV:
			// fall through
		default:
			if (cat == RTW_FW_AP) {
				*fw = INTERNAL_FW_CONTENT_8851E_CAV_AP;
				*fw_len = INTERNAL_FW_LEN_8851E_CAV_AP;
			} else if (cat == RTW_FW_NIC) {
				*fw = INTERNAL_FW_CONTENT_8851E_CAV_NIC;
				*fw_len = INTERNAL_FW_LEN_8851E_CAV_NIC;
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8851E_CAV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8851E_CAV_WOWLAN;
			}
			break;
		}
	} else if (FWDL_IS_8852D) {
		switch (cv) {
		case FWDL_CAV:
			if (cat == RTW_FW_AP) {
				*fw = INTERNAL_FW_CONTENT_8852D_CAV_AP;
				*fw_len = INTERNAL_FW_LEN_8852D_CAV_AP;
			} else if (cat == RTW_FW_NIC) {
				*fw = INTERNAL_FW_CONTENT_8852D_CAV_NIC;
				*fw_len = INTERNAL_FW_LEN_8852D_CAV_NIC;
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8852D_CAV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8852D_CAV_WOWLAN;
			}
			break;
		case FWDL_CBV:
			// fall through
		default:
			if (cat == RTW_FW_AP) {
				*fw = INTERNAL_FW_CONTENT_8852D_CBV_AP;
				*fw_len = INTERNAL_FW_LEN_8852D_CBV_AP;
			} else if (cat == RTW_FW_NIC) {
				*fw = INTERNAL_FW_CONTENT_8852D_CBV_NIC;
				*fw_len = INTERNAL_FW_LEN_8852D_CBV_NIC;
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8852D_CBV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8852D_CBV_WOWLAN;
			}
			break;
		}
	} else if (FWDL_IS_8852BT) {
		switch (cv) {
		case FWDL_CAV:
			// fall through
		default:
			if (cat == RTW_FW_NIC) {
				if (dle_info == DLE_RSVD_INFO_FW) {
					PLTFM_MSG_WARN("[FWDL][WARN] PLE FW is not in use\n");
					PLTFM_MSG_WARN("[FWDL][WARN] Fallback to NIC FW\n");
				}
				*fw = INTERNAL_FW_CONTENT_8852BT_CAV_NIC;
				*fw_len = INTERNAL_FW_LEN_8852BT_CAV_NIC;
			} else if (cat == RTW_FW_NIC_CE) {
				*fw = INTERNAL_FW_CONTENT_8852BT_CAV_NICCE;
				*fw_len = INTERNAL_FW_LEN_8852BT_CAV_NICCE;
			} else if (cat == RTW_FW_WOWLAN) {
				*fw = INTERNAL_FW_CONTENT_8852BT_CAV_WOWLAN;
				*fw_len = INTERNAL_FW_LEN_8852BT_CAV_WOWLAN;
			}
			break;
		}
	}

	if (*fw == FWDL_NO_INTERNAL_FW || *fw_len == FWDL_NO_INTERNAL_FW) {
		PLTFM_MSG_ERR("[FWDL][ERROR][%s] Query Internal FW Fail.\n", __func__);
		ret = MACNOFW;
	}
	return ret;
}
