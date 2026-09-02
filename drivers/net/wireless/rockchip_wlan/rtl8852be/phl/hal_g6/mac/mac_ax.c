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

#include "mac_ax.h"
#include "mac_ax/mac_priv.h"

#define PID_HW_DEF_8852AS	0xA852
#define PID_HW_DEF_8852BS	0xB852
#define PID_HW_DEF_8852BSA	0xB85B
#define PID_HW_DEF_8852BSVQ	0xB853
#define PID_HW_DEF_8851ASA	0x8851
#define PID_HW_DEF_8852BPS	0xA85C
#define PID_HW_DEF_8852CS	0xC852
#define PID_HW_DEF_8192XBS	0x0192
#define PID_HW_DEF_8851BS	0xB851
#define PID_HW_DEF_8851BSM	0xB51A
#define PID_HW_DEF_8852DS	0xD852
#define PID_HW_DEF_8852BTS	0xB925
#define PID_HW_DEF_1115ES	0x892A

#define SDIO_FN0_CIS_PID	0x1004

#define SDIO_WAIT_CNT		50
#define MAC_AX_INDIR_CNT	23

#if MAC_AX_SDIO_SUPPORT
static u8 r8_indir_cmd52_sdio(void *drv_adapter,
			      struct mac_ax_pltfm_cb *pltfm_cb, u32 adr);
#endif

static u8 chk_pltfm_cb(void *drv_adapter, enum mac_ax_intf intf,
		       struct mac_ax_pltfm_cb *pltfm_cb)
{
	if (!pltfm_cb)
		return MACSUCCESS;

	if (!pltfm_cb->msg_print)
		return MACSUCCESS;

#if MAC_AX_SDIO_SUPPORT
	if (!pltfm_cb->sdio_cmd52_r8 || !pltfm_cb->sdio_cmd53_r8 ||
	    !pltfm_cb->sdio_cmd53_r16 || !pltfm_cb->sdio_cmd53_r32 ||
	    !pltfm_cb->sdio_cmd53_rn || !pltfm_cb->sdio_cmd52_w8 ||
	    !pltfm_cb->sdio_cmd53_w8 || !pltfm_cb->sdio_cmd53_w16 ||
	    !pltfm_cb->sdio_cmd53_w32 || !pltfm_cb->sdio_cmd53_wn ||
	    !pltfm_cb->sdio_cmd52_cia_r8) {
		pltfm_cb->msg_print(drv_adapter, _PHL_ERR_, "[ERR]CB-SDIO\n");
		return MACSUCCESS;
	}
#endif

#if (MAC_AX_USB_SUPPORT || MAC_AX_PCIE_SUPPORT)
	if (!pltfm_cb->reg_r8 || !pltfm_cb->reg_r16 ||
	    !pltfm_cb->reg_r32 || !pltfm_cb->reg_w8 ||
	    !pltfm_cb->reg_w16 || !pltfm_cb->reg_w32) {
		pltfm_cb->msg_print(drv_adapter, _PHL_ERR_, "[ERR]CB-USB or PCIE\n");
		return MACSUCCESS;
	}
#endif
	if (!pltfm_cb->rtl_free || !pltfm_cb->rtl_malloc ||
	    !pltfm_cb->rtl_memcpy || !pltfm_cb->rtl_memset ||
	    !pltfm_cb->rtl_delay_us || !pltfm_cb->rtl_delay_ms ||
	    !pltfm_cb->rtl_sleep_us || !pltfm_cb->rtl_sleep_ms ||
	    !pltfm_cb->rtl_get_current_times_ms || !pltfm_cb->rtl_get_current_times_us ||
	    !pltfm_cb->rtl_get_passing_times_ms || !pltfm_cb->rtl_get_passing_times_us ||
	    !pltfm_cb->rtl_mutex_init || !pltfm_cb->rtl_mutex_deinit ||
	    !pltfm_cb->rtl_mutex_lock || !pltfm_cb->rtl_mutex_unlock ||
	    !pltfm_cb->event_notify || !pltfm_cb->get_chip_id) {
		pltfm_cb->msg_print(drv_adapter, _PHL_ERR_, "[ERR]CB-OS\n");
		return MACSUCCESS;
	}

	return MACPFCB;
}

static u8 chk_pltfm_endian(void)
{
	u32 num = 1;
	u8 *num_ptr = (u8 *)&num;

	if (*num_ptr != PLATFOM_IS_LITTLE_ENDIAN)
		return MACSUCCESS;

	return MACPFCB;
}

static u8 get_analog_info(struct mac_ax_adapter *adapter)
{
#if MAC_AX_8852B_SUPPORT || MAC_AX_8851B_SUPPORT || MAC_AX_8852BT_SUPPORT
	u32 ret;
	u8 xtal_si_val;

	if (is_chip_id(adapter, MAC_AX_CHIP_ID_8852B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8851B) ||
	    is_chip_id(adapter, MAC_AX_CHIP_ID_8852BT)) {
		ret = mac_read_xtal_si(adapter, XTAL_SI_CV, &xtal_si_val);
		if (ret)
			PLTFM_MSG_ERR("Read XTAL_SI fail!\n");

		xtal_si_val = (xtal_si_val & acv_mask);
		adapter->hw_info->acv = xtal_si_val;
	}
#endif

	return MACSUCCESS;
}

#if MAC_AX_SDIO_SUPPORT
static u8 r8_indir_cmd52_sdio(void *drv_adapter,
			      struct mac_ax_pltfm_cb *pltfm_cb, u32 adr)
{
	u8 tmp;
	u32 cnt;

	pltfm_cb->sdio_cmd52_w8(drv_adapter, R_AX_SDIO_INDIRECT_ADDR,
				(u8)adr);
	pltfm_cb->sdio_cmd52_w8(drv_adapter, R_AX_SDIO_INDIRECT_ADDR + 1,
				(u8)(adr >> 8));
	pltfm_cb->sdio_cmd52_w8(drv_adapter, R_AX_SDIO_INDIRECT_ADDR + 2,
				(u8)(adr >> 16));
	pltfm_cb->sdio_cmd52_w8(drv_adapter, R_AX_SDIO_INDIRECT_ADDR + 3,
				(u8)((adr | B_AX_INDIRECT_RDY) >> 24));
	pltfm_cb->sdio_cmd52_w8(drv_adapter, R_AX_SDIO_INDIRECT_CTRL,
				(u8)B_AX_INDIRECT_REG_R);

	cnt = SDIO_WAIT_CNT;
	do {
		tmp = pltfm_cb->sdio_cmd52_r8(drv_adapter,
					      R_AX_SDIO_INDIRECT_ADDR + 3);
		cnt--;
	} while (((tmp & BIT(7)) == 0) && (cnt > 0));

	if (((tmp & BIT(7)) == 0) && cnt == 0)
		pltfm_cb->msg_print(drv_adapter, _PHL_ERR_,
				    "[ERR]sdio indirect CMD52 read\n");

	return pltfm_cb->sdio_cmd52_r8(drv_adapter, R_AX_SDIO_INDIRECT_DATA);
}

static u8 r8_dir_cmd52_sdio(void *drv_adapter,
			    struct mac_ax_pltfm_cb *pltfm_cb, u32 adr)
{
	/* Original: Use indirect access WLAN reg due to SDIO power off*/
	/* SDIO reg changes after 8852C (0x1000 -> 0x4000) */
	/* -> SDIO indirect access reg is uncertain before get_chip_info */
	/* Fix: CMD52 direct read WLAN reg and repeat several times */
	/* to sync SDIO reg with MAC reg */
	/* MAC reg is in MAC clk domain & SDIO reg is in SDIO clk domain */
	u32 cnt;

	cnt = MAC_AX_INDIR_CNT;
	do {
		pltfm_cb->sdio_cmd52_r8(drv_adapter, adr);
		cnt--;
	} while (cnt);

	return pltfm_cb->sdio_cmd52_r8(drv_adapter, adr);
}

static u8 get_chip_id_hw_def_sdio(void *drv_adapter,
				  struct mac_ax_pltfm_cb *pltfm_cb, u32 adr)
{
	u16 pid;

	pid = pltfm_cb->sdio_cmd52_cia_r8(drv_adapter, SDIO_FN0_CIS_PID) |
	      (pltfm_cb->sdio_cmd52_cia_r8(drv_adapter, SDIO_FN0_CIS_PID + 1) << 8);

	switch (pid) {
	case PID_HW_DEF_8852AS:
		return MAC_AX_CHIP_ID_8852A;
	case PID_HW_DEF_8852BS:
	case PID_HW_DEF_8852BSA:
	case PID_HW_DEF_8851ASA:
	case PID_HW_DEF_8852BPS:
	case PID_HW_DEF_8852BSVQ:
		return MAC_AX_CHIP_ID_8852B;
	case PID_HW_DEF_8852CS:
		return MAC_AX_CHIP_ID_8852C;
	case PID_HW_DEF_8192XBS:
		return MAC_AX_CHIP_ID_8192XB;
	case PID_HW_DEF_8851BS:
	case PID_HW_DEF_8851BSM:
		return MAC_AX_CHIP_ID_8851B;
	case PID_HW_DEF_8852DS:
		return MAC_AX_CHIP_ID_8852D;
	case PID_HW_DEF_1115ES:
		return MAC_BE_CHIP_ID_1115E;
	case PID_HW_DEF_8852BTS:
		return MAC_AX_CHIP_ID_8852BT;
	default:
		break;
	}

	return r8_dir_cmd52_sdio(drv_adapter, pltfm_cb, adr);
}
#endif
static bool chk_get_chip_info(u8 chip_id, u8 cv)
{
	switch (chip_id) {
	case MAC_AX_CHIP_ID_8852A:
		switch (cv) {
		case CAV:
			return PATCH_ENABLE;
		case CBV:
			return PATCH_ENABLE;
		case CCV:
			return PATCH_DISABLE;
		case CDV:
			//fall through
		default:
			return PATCH_DISABLE;
	}
	default:
		return PATCH_DISABLE;
	}
}

static u32 get_chip_info(void *drv_adapter, struct mac_ax_pltfm_cb *pltfm_cb,
			 enum mac_ax_intf intf, u8 *id, u8 *cv)
{
	u8 cur_id;
	u32 ret;

	if (!cv || !id)
		return MACNPTR;

	switch (intf) {
#if MAC_AX_SDIO_SUPPORT
	case MAC_AX_INTF_SDIO:
		cur_id = r8_indir_cmd52_sdio(drv_adapter, pltfm_cb,
					     R_AX_SYS_CHIPINFO);
		*cv = r8_indir_cmd52_sdio(drv_adapter, pltfm_cb,
					  R_AX_SYS_CFG1 + 1) >> 4;
		if (chk_get_chip_info(cur_id, *cv) == PATCH_ENABLE)
			*cv = CBV;
		break;
#endif
#if (MAC_AX_USB_SUPPORT || MAC_AX_PCIE_SUPPORT)
	case MAC_AX_INTF_USB:
	case MAC_AX_INTF_PCIE:
		cur_id = pltfm_cb->reg_r8(drv_adapter, R_AX_SYS_CHIPINFO);
		*cv = pltfm_cb->reg_r8(drv_adapter, R_AX_SYS_CFG1 + 1) >> 4;
		if (chk_get_chip_info(cur_id, *cv) == PATCH_ENABLE)
			*cv = CBV;
		break;
#endif
	default:
		return MACINTF;
	}

	ret = xlat_chip_id(cur_id, id);
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

u32 mac_get_wlanfw_cap(struct mac_ax_adapter *adapter, struct rtw_wcpu_cap_t *wcpu_cap)
{
	u32 offset = 0;
	u32 cap_len;
	u32 defined_cap_len;
	u32 len_to_cp;
	u32 sizeof_cur_blk;
	struct mac_wlanfw_cap_hdr *cap_hdr;
	u32 size = adapter->fw_info.cap_size;
	u8 *content = adapter->fw_info.cap_buff;
	u8 mod_id;
	u8 mod_idx;
	u8 *target_addr;

	PLTFM_MEMSET(wcpu_cap, 0, sizeof(struct rtw_wcpu_cap_t));
	if (size > MAC_WLANFW_CAP_MAX_SIZE) {
		PLTFM_MSG_ERR("[FwCap] Size (%d) exceeds def (%d)", size, MAC_WLANFW_CAP_MAX_SIZE);
		return MACBUFSZ;
	}
	while (offset < size) {
		sizeof_cur_blk = *content;
		content++;
		offset++;
		if (sizeof_cur_blk < sizeof(struct mac_wlanfw_cap_hdr)) {
			content += sizeof_cur_blk;
			offset += sizeof_cur_blk;
			continue;
		}
		cap_hdr = (struct mac_wlanfw_cap_hdr *)content;
		PLTFM_MSG_TRACE("[FwCap] magic_code (%x), num_mods (%d)\n",
				cap_hdr->magic_code, cap_hdr->num_mods);
		if (cap_hdr->magic_code != MAC_WLANFW_CAP_MAGIC_CODE) {
			content += sizeof_cur_blk;
			offset += sizeof_cur_blk;
			continue;
		}
		content += sizeof(struct mac_wlanfw_cap_hdr);
		sizeof_cur_blk -= sizeof(struct mac_wlanfw_cap_hdr);

		for (mod_idx = 0; mod_idx < cap_hdr->num_mods; mod_idx++) {
			if (sizeof_cur_blk < 2) {
				PLTFM_MSG_ERR("[FwCap] No enough space for modId and capLen\n");
				PLTFM_MEMSET(wcpu_cap, 0, sizeof(struct rtw_wcpu_cap_t));
				return MACNOITEM;
			}
			mod_id = *content++;
			cap_len = ((u32)*content++) * 4;
			sizeof_cur_blk -= 2;
			PLTFM_MSG_TRACE("[FwCap] mod (%x), capLen (%d) byte\n", mod_id, cap_len);
			if (cap_len > sizeof_cur_blk) {
				PLTFM_MSG_ERR("[FwCap] No enough space for mod (%x) * %d Byte\n",
					      mod_id, cap_len);
				PLTFM_MEMSET(wcpu_cap, 0, sizeof(struct rtw_wcpu_cap_t));
				return MACNOITEM;
			}

			switch (mod_id) {
			case MAC_WLANFW_MAC_CAP_SUBID:
				defined_cap_len = sizeof(wcpu_cap->mac_ofld_cap);
				target_addr = (u8 *)&wcpu_cap->mac_ofld_cap;
				break;
			case MAC_WLANFW_BB_CAP_SUBID:
				defined_cap_len = sizeof(wcpu_cap->bb_ofld_cap);
				target_addr = (u8 *)&wcpu_cap->bb_ofld_cap;
				break;
			case MAC_WLANFW_RF_CAP_SUBID:
				defined_cap_len = sizeof(wcpu_cap->rf_ofld_cap);
				target_addr = (u8 *)&wcpu_cap->rf_ofld_cap;
				break;
			case MAC_WLANFW_BTC_CAP_SUBID:
				defined_cap_len = sizeof(wcpu_cap->btc_ofld_cap);
				target_addr = (u8 *)&wcpu_cap->btc_ofld_cap;
				break;
			default:
				PLTFM_MSG_ERR("[FwCap] Unknown modId (%x), abort\n", mod_id);
				PLTFM_MEMSET(wcpu_cap, 0, sizeof(struct rtw_wcpu_cap_t));
				return MACNOITEM;
			}
			len_to_cp = (cap_len < defined_cap_len) ? cap_len : defined_cap_len;
			PLTFM_MSG_TRACE("[FwCap] cp %d bytes\n", len_to_cp);
			PLTFM_MEMCPY(target_addr, content, len_to_cp);
			content += cap_len;
			sizeof_cur_blk -= cap_len;
		}
		wcpu_cap->valid = 1;
		return MACSUCCESS;
	}
	PLTFM_MSG_WARN("[FwCap] wcpu cap not found.\n");
	return MACSUCCESS;
}

u32 mac_ax_ops_init(void *drv_adapter, struct mac_ax_pltfm_cb *pltfm_cb,
		    enum mac_ax_intf intf,
		    struct mac_ax_adapter **mac_adapter,
		    struct mac_ax_ops **mac_ops)
{
	u32 ret;
	u8 chip_id = 0;
	u8 cv = 0;
	struct mac_ax_adapter *adapter;
	struct mac_ax_priv_ops *p_ops;

	if (!chk_pltfm_cb(drv_adapter, intf, pltfm_cb))
		return MACPFCB;

	if (!chk_pltfm_endian())
		return MACPFED;

	pltfm_cb->msg_print(drv_adapter, _PHL_ERR_,
			    "MAC_AX_MAJOR_VER = %d\n"
			    "MAC_AX_PROTOTYPE_VER = %d\n"
			    "MAC_AX_SUB_VER = %d\n"
			    "MAC_AX_SUB_INDEX = %d\n",
			    MAC_AX_MAJOR_VER, MAC_AX_PROTOTYPE_VER,
			    MAC_AX_SUB_VER, MAC_AX_SUB_INDEX);

	ret = get_chip_info(drv_adapter, pltfm_cb, intf, &chip_id, &cv);
	if (ret)
		return ret;

	adapter = get_mac_ax_adapter(intf, chip_id, cv, drv_adapter,
				     pltfm_cb);
	if (!adapter) {
		pltfm_cb->msg_print(drv_adapter, _PHL_ERR_, "[ERR]Get MAC adapter\n");
		return MACADAPTER;
	}

	*mac_adapter = adapter;
	*mac_ops = adapter->ops;

#if MAC_AX_SDIO_SUPPORT
	/* should call halmac IO API before SDIO tbl (mutex) init*/
	ret = sdio_tbl_init(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]sdio tbl init %d\n", ret);
		return ret;
	}
#endif

#if MAC_AX_FEATURE_HV
	adapter->hv_ops = get_hv_ax_ops(adapter);
#endif

#if MAC_AX_PHL_H2C

#else
	ret = h2cb_init(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]h2cb init %d\n", ret);
		return ret;
	}
#endif

	ret = role_tbl_init(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]role tbl init %d\n", ret);
		return ret;
	}

	p_ops = adapter_to_priv_ops(adapter);
	if (!p_ops) {
		PLTFM_MSG_ERR("%s p_ops alloc failed\n", __func__);
		return MACBUFALLOC;
	}
	ret = p_ops->sec_info_tbl_init(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]sec info tbl init %d\n", ret);
		return ret;
	}

	ret = p_ops->efuse_info_init(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]efuse info init %d\n", ret);
		return ret;
	}

#if MAC_FEAT_P2P
	ret = p2p_info_init(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]p2p info init %d\n", ret);
		return ret;
	}
#endif

	ret = mport_info_init(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]mpinfo info init %d\n", ret);
		return ret;
	}

	ret = mix_info_init(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]mix info init %d\n", ret);
		return ret;
	}

	ret = hfc_info_init(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]hfc info init %d\n", ret);
		return ret;
	}

#if MAC_FEAT_TWT_STA || MAC_FEAT_TWTAP
	ret = twt_info_init(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]twt info init %d\n", ret);
		return ret;
	}
#endif

#if MAC_FEAT_DBCC
	ret = dbcc_info_init(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]dbcc info init %d\n", ret);
		return ret;
	}
#endif /* MAC_FEAT_DBCC */

	ret = get_analog_info(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]get_analog_info %d\n", ret);
		return ret;
	}

	return MACSUCCESS;
}

#if MAC_AX_PHL_H2C
u32 mac_ax_phl_init(void *phl_adapter, struct mac_ax_adapter *mac_adapter)
{
	struct mac_ax_adapter *adapter = mac_adapter;

	adapter->phl_adapter = phl_adapter;

	return MACSUCCESS;
}
#endif
u32 mac_ax_ops_exit(struct mac_ax_adapter *adapter)
{
	u32 ret;
	struct mac_ax_priv_ops *p_ops = adapter_to_priv_ops(adapter);
	struct mac_ax_cmd_ofld_info *ofld_info = &adapter->cmd_ofld_info;
	struct scan_chinfo_list *scan_list;
	u32 priv_size;
	u8 band_idx;

	ret = h2cb_exit(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]h2c buffer exit %d\n", ret);
		return ret;
	}

	if (ofld_info->buf) {
		PLTFM_FREE(ofld_info->buf, CMD_OFLD_MAX_LEN);
		ofld_info->buf = NULL;
	}

	ret = p_ops->free_sec_info_tbl(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]sec table exit %d\n", ret);
		return ret;
	}

	ret = role_tbl_exit(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]role table exit %d\n", ret);
		return ret;
	}

	ret = efuse_info_exit(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]efuse info exit %d\n", ret);
		return ret;
	}

#if MAC_FEAT_P2P
	ret = p2p_info_exit(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]p2p info exit %d\n", ret);
		return ret;
	}
#endif

	ret = mport_info_exit(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]mpinfo info exit %d\n", ret);
		return ret;
	}

	ret = mix_info_exit(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]mix info exit %d\n", ret);
		return ret;
	}

#if MAC_AX_SDIO_SUPPORT
	ret = sdio_tbl_exit(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]efuse table exit %d\n", ret);
		return ret;
	}
#endif

	ret = hfc_info_exit(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]hfc info exit %d\n", ret);
		return ret;
	}

#if MAC_FEAT_TWT_STA || MAC_FEAT_TWTAP
	ret = twt_info_exit(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]twt info exit %d\n", ret);
		return ret;
	}
#endif

#if MAC_FEAT_DBCC
	ret = dbcc_info_exit(adapter);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("[ERR]dbcc info exit %d\n", ret);
		return ret;
	}
#endif /* MAC_FEAT_DBCC */

#if MAC_FEAT_FWOFLD
	for (band_idx = 0; band_idx < MAC_AX_BAND_NUM; band_idx++) {
		scan_list = adapter->scanofld_info.list[band_idx];
		if (scan_list) {
			mac_scanofld_ch_list_clear(adapter, scan_list);
			PLTFM_FREE((u8 *)scan_list,
				   sizeof(struct scan_chinfo_list));
			adapter->scanofld_info.list[band_idx] = NULL;
		}
	}
#endif /* MAC_FEAT_FWOFLD */

	PLTFM_FREE(adapter->hw_info, sizeof(struct mac_ax_hw_info));

	priv_size = get_mac_ax_priv_size();
	PLTFM_FREE(adapter, sizeof(struct mac_ax_adapter) + priv_size);
	return MACSUCCESS;
}

u32 is_chip_id(struct mac_ax_adapter *adapter, enum mac_ax_chip_id id)
{
	return (id == adapter->drv_info->sw_chip_id ? 1 : 0);
}

u32 is_cv(struct mac_ax_adapter *adapter, enum rtw_cv cv)
{
	return (cv == adapter->hw_info->cv ? 1 : 0);
}

u32 xlat_chip_id(u8 hw_id, u8 *chip_id)
{
	switch (hw_id) {
#if MAC_AX_8852A_SUPPORT
	case D_DIE_CHIP_ID_8852A:
		*chip_id = MAC_AX_CHIP_ID_8852A;
		break;
#endif
#if MAC_AX_8852B_SUPPORT
	case D_DIE_CHIP_ID_8852B:
		*chip_id = MAC_AX_CHIP_ID_8852B;
		break;
#endif
#if MAC_AX_8852C_SUPPORT
	case D_DIE_CHIP_ID_8852C:
		*chip_id = MAC_AX_CHIP_ID_8852C;
		break;
#endif
#if MAC_AX_8192XB_SUPPORT
	case D_DIE_CHIP_ID_8192XB:
		*chip_id = MAC_AX_CHIP_ID_8192XB;
		break;
#endif
#if MAC_AX_8851B_SUPPORT
	case D_DIE_CHIP_ID_8851B:
		*chip_id = MAC_AX_CHIP_ID_8851B;
		break;
#endif
#if MAC_AX_8852D_SUPPORT
	case D_DIE_CHIP_ID_8852D:
		*chip_id = MAC_AX_CHIP_ID_8852D;
		break;
#endif
#if MAC_AX_8852BT_SUPPORT
	case D_DIE_CHIP_ID_8852BT:
		*chip_id = MAC_AX_CHIP_ID_8852BT;
		break;
#endif
	default:
		*chip_id = MAC_AX_CHIP_ID_INVALID;
		return MACCHIPID;
	}

	return MACSUCCESS;
}

#if MAC_AX_CONSOLE_EN || MAC_AX_PLDR_DIAGNOSE_EN
void mac_console_log(void *vadapter, s8 *prefix, s8 *fmt, ...)
{
	struct mac_ax_adapter *adapter = (struct mac_ax_adapter *)vadapter;
	char *output_tmp = adapter->fw_dbgcmd.buf;
	u32 output_len_tmp = adapter->fw_dbgcmd.out_len;
	u32 *used_tmp = &adapter->fw_dbgcmd.used;
	s32 ret;
	_os_va_list args;

	if (adapter->fw_dbgcmd.dbg_console_log_on) {
		PLTFM_MUTEX_LOCK(&adapter->lock_info.fw_dbgcmd_lock);
		if (*used_tmp < output_len_tmp) {
			ret = PLTFM_SNPRINTF(output_tmp + *used_tmp,
					     output_len_tmp - *used_tmp, prefix);
			if (ret >= 0)
				*used_tmp += ret;
		}
		if (*used_tmp < output_len_tmp) {
			_os_va_start(args, fmt);
			ret = PLTFM_VSNPRINTF(output_tmp + *used_tmp,
					      output_len_tmp - *used_tmp, fmt, args);
			_os_va_end(args);
			if (ret >= 0)
				*used_tmp += ret;
		}
		PLTFM_MUTEX_UNLOCK(&adapter->lock_info.fw_dbgcmd_lock);
	}
}
#else
void mac_console_log(void *vadapter, s8 *prefix, s8 *fmt, ...)
{
}
#endif /* MAC_AX_CONSOLE_EN */
