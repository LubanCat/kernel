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

#include "beacon.h"

#if MAC_FEAT_RX_BCN_OFLD

u8 _byte_rev(u8 in)
{
	u8 data = 0;
	u8 i;

	for (i = 0; i < 8; i++)
		data |= in & BIT(i) ? BIT(7 - i) : 0;
	return data;
}

u8 _crc8_htsig(u8 *mem, u32 len)
{
	u8 crc = 0xFF;
	u8 key = 0x07;
	u8 k;

	if (!mem)
		return 0xFF;

	while (len--) {
		crc ^= _byte_rev(*mem++);
		for (k = 0; k < 8; k++)
			crc = crc & 0x80 ? (crc << 1) ^ key : crc << 1;
	}

	return crc;
}

u32 mac_calc_crc(struct mac_ax_adapter *adapter, struct mac_calc_crc_info *info)
{
	if (!info->buf)
		return MACNPTR;

	if (!info->len)
		return MACBUFSZ;

	info->crc = (u32)_crc8_htsig(info->buf, info->len);

	return MACSUCCESS;
}

u32 mac_bcn_ofld_ctrl(struct mac_ax_adapter *adapter, struct mac_bcn_ofld_info *info)
{
	struct h2c_info h2c_info = {0};
	struct fwcmd_ie_cam *cmd;
	u32 ret, iecam_len, i, content_len;
	u8 *buf, *iecam_buf;
	u8 rst = 0, num = 0;

	switch (info->ctrl_type) {
	case MAC_BCN_OFLD_UPD_PARAM:
	case MAC_BCN_OFLD_DIS:
		num = 0;
		rst = 0;
		break;
	case MAC_BCN_OFLD_EN:
	case MAC_BCN_OFLD_UPD_CAM:
		if (!info->cam_num) {
			PLTFM_MSG_ERR("%s cam num is 0 when enable/update iecam\n", __func__);
			return MACFUNCINPUT;
		}

		rst = info->rst_iecam == MAC_AX_FUNC_EN ? 1 : 0;
		num = info->cam_num;
	}

	iecam_len = sizeof(struct mac_ie_cam_ent) * num;
	content_len = sizeof(struct fwcmd_ie_cam) - IECAM_INFO_SIZE + iecam_len;
	buf = (u8 *)PLTFM_MALLOC(content_len);
	if (!buf) {
		PLTFM_MSG_ERR("H2C bcn ofld ctrl no buffer\n");
		return MACNOBUF;
	}
	iecam_buf = buf + content_len - iecam_len;

	cmd = (struct fwcmd_ie_cam *)buf;
	cmd->dword0 =
		cpu_to_le32((info->band == MAC_AX_BAND_1 ? FWCMD_H2C_IE_CAM_BAND : 0) |
			    SET_WORD(info->port, FWCMD_H2C_IE_CAM_PORT) |
			    (info->ctrl_type == MAC_BCN_OFLD_DIS ?
			     0 : FWCMD_H2C_IE_CAM_CAM_EN) |
			    (info->hit_en == MAC_AX_FUNC_DIS ?
			     0 : FWCMD_H2C_IE_CAM_HIT_FRWD_EN) |
			    SET_WORD(info->hit_sel, FWCMD_H2C_IE_CAM_HIT_FRWD) |
			    (info->miss_en == MAC_AX_FUNC_DIS ?
			     0 : FWCMD_H2C_IE_CAM_MISS_FRWD_EN) |
			    SET_WORD(info->miss_sel, FWCMD_H2C_IE_CAM_MISS_FRWD) |
			    SET_WORD(num, FWCMD_H2C_IE_CAM_UPD_NUM) |
			    (rst ? FWCMD_H2C_IE_CAM_RST : 0));

	if (num) {
		for (i = 0; i < num; i++) {
			*(u32 *)iecam_buf = cpu_to_le32(info->cam_list[i].hdr.cam_idx);
			iecam_buf += 4;
			*(u32 *)iecam_buf = cpu_to_le32(info->cam_list[i].u.data.dw0);
			iecam_buf += 4;
			*(u32 *)iecam_buf = cpu_to_le32(info->cam_list[i].u.data.dw1);
			iecam_buf += 4;
		}
	}

	h2c_info.agg_en = 0;
	h2c_info.content_len = (u16)content_len;
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_IE_CAM;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_IE_CAM;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)buf);
	if (ret != MACSUCCESS)
		PLTFM_MSG_ERR("%s send h2c fail %d\n", __func__, ret);
	PLTFM_FREE(buf, content_len);
	return ret;
}

u32 mac_ie_cam_upd(struct mac_ax_adapter *adapter,
		   struct mac_ax_ie_cam_cmd_info *info)
{
	struct h2c_info h2c_info = {0};
	struct fwcmd_ie_cam *cmd;
	u8 *buf, *tmp_buf;
	u32 ret, content_len;

	if (adapter->sm.fwdl != MAC_AX_FWDL_INIT_RDY) {
		PLTFM_MSG_WARN("%s fw not ready\n", __func__);
		return MACFWNONRDY;
	}

	content_len = sizeof(struct fwcmd_ie_cam) + info->buf_len;
	buf = (u8 *)PLTFM_MALLOC(content_len);
	if (!buf) {
		PLTFM_MSG_ERR("H2C IE CAM no buffer\n");
		return MACNOBUF;
	}

	cmd = (struct fwcmd_ie_cam *)buf;
	cmd->dword0 =
		cpu_to_le32((info->en ? FWCMD_H2C_IE_CAM_CAM_EN : 0) |
			    (info->band ? FWCMD_H2C_IE_CAM_BAND : 0) |
			    (info->hit_en ? FWCMD_H2C_IE_CAM_HIT_FRWD_EN : 0) |
			    (info->miss_en ?
			     FWCMD_H2C_IE_CAM_MISS_FRWD_EN : 0) |
			    (info->rst ? FWCMD_H2C_IE_CAM_RST : 0) |
			    SET_WORD(info->port, FWCMD_H2C_IE_CAM_PORT) |
			    SET_WORD(info->hit_sel, FWCMD_H2C_IE_CAM_HIT_FRWD) |
			    SET_WORD(info->miss_sel,
				     FWCMD_H2C_IE_CAM_MISS_FRWD) |
			    SET_WORD(info->num, FWCMD_H2C_IE_CAM_UPD_NUM));

	tmp_buf = buf + sizeof(struct fwcmd_ie_cam);
	PLTFM_MEMCPY(tmp_buf, info->buf, info->buf_len);

	h2c_info.agg_en = 0;
	h2c_info.content_len = (u16)content_len;
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_IE_CAM;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_IE_CAM;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)buf);
	if (ret != MACSUCCESS)
		PLTFM_MSG_ERR("H2C IE CAM send fail %d\n", ret);
	PLTFM_FREE(buf, content_len);
	return ret;
}
#endif // #if MAC_FEAT_RX_BCN_OFLD

#if MAC_FEAT_TX_BCN_DM_IGNORE_EDCCA
u32 mac_set_bcn_ignore_edcca(struct mac_ax_adapter *adapter,
			     struct mac_ax_bcn_ignore_edcca *bcn_ignore_edcca)
{
	u32 ret = MACSUCCESS;
	struct h2c_info h2c_info = {0};
	struct fwcmd_bcn_ignore_edcca *content;

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_bcn_ignore_edcca);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_FR_EXCHG;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_BCN_IGNORE_EDCCA;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	content = (struct fwcmd_bcn_ignore_edcca *)PLTFM_MALLOC(h2c_info.content_len);
	if (!content)
		return MACBUFALLOC;

	PLTFM_MSG_ALWAYS("%s: port(%d), mbssid(%d ms), band(%d), en/dis(%d)\n",
			 __func__, bcn_ignore_edcca->port, bcn_ignore_edcca->mbssid,
			 bcn_ignore_edcca->band, bcn_ignore_edcca->ignore_edcca_en);

	content->dword0 =
		cpu_to_le32(SET_WORD(bcn_ignore_edcca->port, FWCMD_H2C_BCN_IGNORE_EDCCA_PORT) |
			SET_WORD(bcn_ignore_edcca->mbssid, FWCMD_H2C_BCN_IGNORE_EDCCA_MBSSID) |
			SET_WORD(bcn_ignore_edcca->band, FWCMD_H2C_BCN_IGNORE_EDCCA_BAND) |
			(bcn_ignore_edcca->ignore_edcca_en ?
			 FWCMD_H2C_BCN_IGNORE_EDCCA_IGNORE_EDCCA_EN : 0));

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)content);
	PLTFM_FREE(content, h2c_info.content_len);
	return ret;
}
#endif

#if MAC_FEAT_TX_BCN
u32 mac_send_bcn_h2c(struct mac_ax_adapter *adapter,
		     struct mac_ax_bcn_info *info)
{
	struct h2c_info h2c_info = {0};
	struct fwcmd_bcn_upd_v1 *sub_content;
	u8 portl;
	u32 ret = MACSUCCESS;
	u32 *content;

	if (info->pld_len > MAX_BCN_PLD_SIZE) {
		PLTFM_MSG_ERR("[ERR] BCN Payload len should not exceed %d bytes\n",
			      MAX_BCN_PLD_SIZE);
		return MACFUNCINPUT;
	}

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_bcn_upd_v1) + info->pld_len;
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_FR_EXCHG;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_BCN_UPD_V1;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;
	content = (u32 *)PLTFM_MALLOC(h2c_info.content_len);
	if (!content)
		return MACBUFALLOC;
	sub_content = (struct fwcmd_bcn_upd_v1 *)PLTFM_MALLOC(sizeof(struct fwcmd_bcn_upd_v1));
	if (!sub_content) {
		PLTFM_FREE(content, h2c_info.content_len);
		return MACBUFALLOC;
	}

	portl = (info->port == 0) ? info->mbssid : info->port + MAC_AX_P0_MBID_LAST - 1;
	if (info->band == 0) {
		adapter->hw_info->h2c_bcn_upd_sent_band0 = adapter->hw_info->h2c_bcn_upd_sent_band0
							   | BIT(portl);
		adapter->hw_info->bcn_drop_all_band0 = adapter->hw_info->bcn_drop_all_band0
						       & ~(u32)BIT(portl);
	} else if (info->band == 1) {
		adapter->hw_info->h2c_bcn_upd_sent_band1 = adapter->hw_info->h2c_bcn_upd_sent_band1
							   | BIT(portl);
		adapter->hw_info->bcn_drop_all_band1 = adapter->hw_info->bcn_drop_all_band1
						       & ~(u32)BIT(portl);
	}
	adapter->hw_info->bcn_pkt_drop = adapter->hw_info->bcn_pkt_drop & ~(u8)BIT(info->band);

	info->grp_ie_ofst |= info->grp_ie_ofst ? BCN_GRPIE_OFST_EN : 0;

	sub_content->dword0 =
		cpu_to_le32(SET_WORD(info->port,
				     FWCMD_H2C_BCN_UPD_V1_PORT) |
			    SET_WORD(info->mbssid,
				     FWCMD_H2C_BCN_UPD_V1_MBSSID) |
			    SET_WORD(info->band,
				     FWCMD_H2C_BCN_UPD_V1_BAND) |
			    SET_WORD(info->grp_ie_ofst,
				     FWCMD_H2C_BCN_UPD_V1_GRP_IE_OFST));

	sub_content->dword1 =
		cpu_to_le32(SET_WORD(info->macid,
				     FWCMD_H2C_BCN_UPD_V1_MACID) |
			    SET_WORD(info->ssn_sel,
				     FWCMD_H2C_BCN_UPD_V1_SSN_SEL) |
			    SET_WORD(info->ssn_mode,
				     FWCMD_H2C_BCN_UPD_V1_SSN_MODE) |
			    SET_WORD(info->rate_sel,
				     FWCMD_H2C_BCN_UPD_V1_RATE) |
			    SET_WORD(info->txpwr,
				     FWCMD_H2C_BCN_UPD_V1_TXPWR) |
			    FWCMD_H2C_BCN_UPD_V1_ECSA_SUPPORT |
			    ((info->protection_key_id & BIT(0)) ?
				     FWCMD_H2C_BCN_UPD_V1_PROTECTION_KEY_ID : 0) |
			    SET_WORD(info->sec_algo,
				     FWCMD_H2C_BCN_UPD_V1_SEC_ALGO) |
			    (info->sec_enable ?
				FWCMD_H2C_BCN_UPD_V1_SEC_ENABLE : 0) |
			    (info->pn_reset ?
				FWCMD_H2C_BCN_UPD_V1_PN_RESET : 0));

	sub_content->dword2 =
		cpu_to_le32((info->txinfo_ctrl_en ?
			     FWCMD_H2C_BCN_UPD_V1_TXINFO_CTRL_EN : 0) |
			    SET_WORD(info->ntx_path_en,
				     FWCMD_H2C_BCN_UPD_V1_NTX_PATH_EN) |
			    SET_WORD(info->path_map_a,
				     FWCMD_H2C_BCN_UPD_V1_PATH_MAP_A) |
			    SET_WORD(info->path_map_b,
				     FWCMD_H2C_BCN_UPD_V1_PATH_MAP_B) |
			    SET_WORD(info->path_map_c,
				     FWCMD_H2C_BCN_UPD_V1_PATH_MAP_C) |
			    SET_WORD(info->path_map_d,
				     FWCMD_H2C_BCN_UPD_V1_PATH_MAP_D) |
			    (info->antsel_a ?
			     FWCMD_H2C_BCN_UPD_V1_ANTSEL_A : 0) |
			    (info->antsel_b ?
			     FWCMD_H2C_BCN_UPD_V1_ANTSEL_B : 0) |
			    (info->antsel_c ?
			     FWCMD_H2C_BCN_UPD_V1_ANTSEL_C : 0) |
			    (info->antsel_d ?
			     FWCMD_H2C_BCN_UPD_V1_ANTSEL_D : 0) |
			     SET_WORD(info->csa_ofst,
				      FWCMD_H2C_BCN_UPD_V1_CSA_OFST));

	sub_content->dword3 =
		cpu_to_le32(SET_WORD(info->ecsa_ofst,
				     FWCMD_H2C_BCN_UPD_V1_ECSA_OFST) |
			    SET_WORD(info->bc_chg_ofst,
				     FWCMD_H2C_BCN_UPD_V1_BC_CHG_OFST));

	sub_content->dword4 =
		cpu_to_le32(SET_WORD(info->pn_low,
				     FWCMD_H2C_BCN_UPD_V1_PN_LOW));

	sub_content->dword5 =
		cpu_to_le32(SET_WORD(info->pn_high,
				     FWCMD_H2C_BCN_UPD_V1_PN_HIGH));

	PLTFM_MEMCPY(content, sub_content, sizeof(struct fwcmd_bcn_upd_v1));
	PLTFM_MEMCPY(content + sizeof(*sub_content) / sizeof(sub_content->dword0),
		     info->pld_buf, info->pld_len);
	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)content);
	PLTFM_FREE(content, h2c_info.content_len);
	PLTFM_FREE(sub_content, sizeof(struct fwcmd_bcn_upd_v1));
	return ret;
}
#endif

#if MAC_FEAT_TX_BCN_DM_TBTT_SHIFT
u32 mac_set_bcn_dynamic_mech(struct mac_ax_adapter *adapter,
			     struct mac_ax_bcn_dynamic_mech *bcn_dynamic_mech)
{
	u32 ret = MACSUCCESS;
	struct h2c_info h2c_info = {0};
	struct fwcmd_bcn_dynamic_mech *content;

	h2c_info.agg_en = 0;
	h2c_info.content_len = sizeof(struct fwcmd_bcn_dynamic_mech);
	h2c_info.h2c_cat = FWCMD_H2C_CAT_MAC;
	h2c_info.h2c_class = FWCMD_H2C_CL_FR_EXCHG;
	h2c_info.h2c_func = FWCMD_H2C_FUNC_BCN_DYNAMIC_MECH;
	h2c_info.rec_ack = 0;
	h2c_info.done_ack = 0;

	content = (struct fwcmd_bcn_dynamic_mech *)PLTFM_MALLOC(h2c_info.content_len);
	if (!content)
		return MACBUFALLOC;

	content->dword0 = cpu_to_le32((bcn_dynamic_mech->bcn_dm_tbtt_shft_en ?
				       FWCMD_H2C_BCN_DYNAMIC_MECH_BCN_DM_TBTT_SHFT_EN : 0));

	ret = mac_h2c_common(adapter, &h2c_info, (u32 *)content);
	PLTFM_FREE(content, h2c_info.content_len);
	return ret;
}
#endif
