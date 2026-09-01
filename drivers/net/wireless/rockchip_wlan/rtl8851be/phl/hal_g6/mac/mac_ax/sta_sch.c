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

#include "sta_sch.h"

static u32 poll_own_chk(struct mac_ax_adapter *adapter, u32 addr)
{
	u32 cnt = 100;
	u32 ck;
	u32 val32;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	if (addr == R_AX_SS_SRAM_CTRL_1 || addr == R_AX_SS_LINK_INFO ||
	    addr == R_AX_SS_SRAM_CTRL_0 || addr == R_AX_SS_LINK_ADD ||
	    addr == R_AX_SS_LINK_DEL || addr == R_AX_SS_LINK_SEARCH)
		ck = B_AX_SS_OWN;
	else
		ck = 0;

	while (--cnt) {
		val32 = MAC_REG_R32(addr);
		if ((val32 & ck) == 0)
			break;
		PLTFM_DELAY_US(1);
	}
	if (!cnt) {
		PLTFM_MSG_ERR("[STASCH]poll_own fail!!\n");
		return MACPOLLTO;
	}

	if (addr == R_AX_SS_SRAM_CTRL_1 || addr == R_AX_SS_LINK_INFO ||
	    addr == R_AX_SS_LINK_ADD || addr == R_AX_SS_LINK_DEL) {
		val32 = MAC_REG_R32(addr);
		if (val32 & (BIT(29) | BIT(30))) {
			PLTFM_MSG_ERR("[STASCH]poll status fail!!\n");
			return MACPROCERR;
		}
	}
	return MACSUCCESS;
}

static u32 poll_own_val(struct mac_ax_adapter *adapter, u32 addr, u32 *r_val)
{
	u32 cnt = 100;
	u32 ck;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	if (addr == R_AX_SS_SRAM_CTRL_1 || addr == R_AX_SS_LINK_INFO ||
	    addr == R_AX_SS_SRAM_CTRL_0 || addr == R_AX_SS_LINK_ADD ||
	    addr == R_AX_SS_LINK_DEL || addr == R_AX_SS_LINK_SEARCH)
		ck = B_AX_SS_OWN;
	else
		ck = 0;

	while (--cnt) {
		*r_val = MAC_REG_R32(addr);
		if ((*r_val & ck) == 0)
			break;
		PLTFM_DELAY_US(1);
	}
	if (!cnt) {
		PLTFM_MSG_ERR("[STASCH]poll_own fail!!\n");
		return MACPOLLTO;
	}

	if (addr == R_AX_SS_SRAM_CTRL_1 || addr == R_AX_SS_LINK_INFO ||
	    addr == R_AX_SS_LINK_ADD || addr == R_AX_SS_LINK_DEL) {
		if (*r_val & (BIT(29) | BIT(30))) {
			PLTFM_MSG_ERR("[STASCH]poll status fail!!\n");
			return MACPROCERR;
		}
	}
	return MACSUCCESS;
}

static u32 sram1_ctrl_val(struct mac_ax_adapter *adapter, u32 cmd, u32 *r_val)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	MAC_REG_W32(R_AX_SS_SRAM_CTRL_1, cmd);

	return poll_own_val(adapter, R_AX_SS_SRAM_CTRL_1, r_val);
}

static u32 get_link_info(struct mac_ax_adapter *adapter, u32 cmd, u32 *r_val)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	MAC_REG_W32(R_AX_SS_LINK_INFO, cmd);

	return poll_own_val(adapter, R_AX_SS_LINK_INFO, r_val);
}

static u32 add_to_link(struct mac_ax_adapter *adapter,
		       struct mac_ax_ss_link_info *link)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	MAC_REG_W32(R_AX_SS_LINK_ADD, B_AX_SS_OWN |
		    (link->ul ? B_AX_SS_UL : 0) |
		    SET_WORD(link->wmm, B_AX_SS_WMM) |
		    SET_WORD(link->ac, B_AX_SS_AC) |
		    SET_WORD(link->macid2, B_AX_SS_MACID_2) |
		    SET_WORD(link->macid1, B_AX_SS_MACID_1) |
		    SET_WORD(link->macid0, B_AX_SS_MACID_0));
	return poll_own_chk(adapter, R_AX_SS_LINK_ADD);
}

static u32 del_from_link(struct mac_ax_adapter *adapter,
			 struct mac_ax_ss_link_info *link)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	MAC_REG_W32(R_AX_SS_LINK_DEL, B_AX_SS_OWN |
		    (link->ul ? B_AX_SS_UL : 0) |
		    SET_WORD(link->wmm, B_AX_SS_WMM) |
		    SET_WORD(link->ac, B_AX_SS_AC) |
		    SET_WORD(link->macid2, B_AX_SS_MACID_2) |
		    SET_WORD(link->macid1, B_AX_SS_MACID_1) |
		    SET_WORD(link->macid0, B_AX_SS_MACID_0));
	return poll_own_chk(adapter, R_AX_SS_LINK_DEL);
}

static u32 clean_sta_link(struct mac_ax_adapter *adapter,
			  struct mac_ax_ss_link_info *link)
{
	u32 val32;
	u8 wmm, ac;
	u32 cnt = adapter->hw_info->macid_num + 1;
	u32 cmd;
	u32 ret;

	if (link->ul) {
		wmm = 0;
		ac = 0;
	} else {
		wmm = link->wmm;
		ac = link->ac;
	}

	do {
		cmd = B_AX_SS_OWN | (link->ul ? B_AX_SS_UL : 0) |
		      SET_WORD(wmm, B_AX_SS_WMM) | SET_WORD(ac, B_AX_SS_AC);
		ret = get_link_info(adapter, cmd, &val32);
		if (ret != MACSUCCESS)
			return ret;

		link->link_head = GET_FIELD(val32, B_AX_SS_LINK_HEAD);
		link->link_tail = GET_FIELD(val32, B_AX_SS_LINK_TAIL);
		link->link_len = GET_FIELD(val32, B_AX_SS_LINK_LEN);
		link->macid0 = link->link_head;

		if (link->macid0 != 0xFF) {
			ret = del_from_link(adapter, link);
			if (ret != MACSUCCESS)
				return ret;
			cnt--;
		} else {
			break;
		}
	} while (link->link_len || cnt);

	if (!cnt) {
		PLTFM_MSG_ERR("[STASCH]link error!!\n");
		return MACSSLINK;
	}

	return MACSUCCESS;
}

static u32 wmm_vld_chk(struct mac_ax_adapter *adapter, u8 *vld, u8 wmm)
{
	u8 wmm_num;

	switch (adapter->drv_info->sw_chip_id) {
	case MAC_AX_CHIP_ID_8852A:
		wmm_num = STA_SCH_WMM_NUM_8852A;
		break;
	case MAC_AX_CHIP_ID_8852B:
		wmm_num = STA_SCH_WMM_NUM_8852B;
		break;
	case MAC_AX_CHIP_ID_8852C:
		wmm_num = STA_SCH_WMM_NUM_8852C;
		break;
	case MAC_AX_CHIP_ID_8192XB:
		wmm_num = STA_SCH_WMM_NUM_8192XB;
		break;
	case MAC_AX_CHIP_ID_8851B:
		wmm_num = STA_SCH_WMM_NUM_8851B;
		break;
	case MAC_AX_CHIP_ID_8852D:
		wmm_num = STA_SCH_WMM_NUM_8852D;
		break;
	case MAC_AX_CHIP_ID_8852BT:
		wmm_num = STA_SCH_WMM_NUM_8852BT;
		break;
	default:
		return MACCHIPID;
	}

	if (wmm < wmm_num)
		*vld = 1;
	else
		*vld = 0;

	return MACSUCCESS;
}

static u32 ul_vld_chk(struct mac_ax_adapter *adapter, u8 *vld)
{
	u8 ul_support;

	switch (adapter->drv_info->sw_chip_id) {
	case MAC_AX_CHIP_ID_8852A:
		ul_support = STA_SCH_UL_SUPPORT_8852A;
		break;
	case MAC_AX_CHIP_ID_8852B:
		ul_support = STA_SCH_UL_SUPPORT_8852B;
		break;
	case MAC_AX_CHIP_ID_8852C:
		ul_support = STA_SCH_UL_SUPPORT_8852C;
		break;
	case MAC_AX_CHIP_ID_8192XB:
		ul_support = STA_SCH_UL_SUPPORT_8192XB;
		break;
	case MAC_AX_CHIP_ID_8851B:
		ul_support = STA_SCH_UL_SUPPORT_8851B;
		break;
	case MAC_AX_CHIP_ID_8852D:
		ul_support = STA_SCH_UL_SUPPORT_8852D;
		break;
	case MAC_AX_CHIP_ID_8852BT:
		ul_support = STA_SCH_UL_SUPPORT_8852BT;
		break;
	default:
		return MACCHIPID;
	}

	if (ul_support)
		*vld = 1;
	else
		*vld = 0;

	return MACSUCCESS;
}

static u32 get_sta_link(struct mac_ax_adapter *adapter,
			struct mac_ax_ss_link_info *link)
{
	u32 val32, i;
	u8 macid, wmm, ac;
	u32 ret;
	u32 cmd;
	u16 id_empty = STA_SCH_LEFT_SHIFT_ONE_SET_MSB(adapter->hw_info->macid_num);

	if (link->ul) {
		wmm = 0;
		ac = 0;
	} else {
		wmm = link->wmm;
		ac = link->ac;
	}

	cmd = B_AX_SS_OWN | (link->ul ? B_AX_SS_UL : 0) |
	      SET_WORD(wmm, B_AX_SS_WMM) | SET_WORD(ac, B_AX_SS_AC);
	ret = get_link_info(adapter, cmd, &val32);
	if (ret != MACSUCCESS)
		return ret;

	link->link_head = GET_FIELD(val32, B_AX_SS_LINK_HEAD);
	link->link_tail = GET_FIELD(val32, B_AX_SS_LINK_TAIL);
	link->link_len = GET_FIELD(val32, B_AX_SS_LINK_LEN);
	macid = link->link_head;

	if (link->link_head == id_empty) {
		if (link->link_len) {
			PLTFM_MSG_ERR("empty link_len error!!");
			return MACSSLINK;
		}
	} else {
		i = 0;
		do {
			link->link_list[i] = macid;
			link->link_bitmap[macid] = 1;
			cmd = B_AX_SS_OWN |
			      SET_WORD(CTRL1_R_NEXT_LINK, B_AX_SS_CMD_SEL) |
			      SET_WORD(ac, B_AX_SS_AC) |
			      (link->ul ? BIT(23) : 0) | macid;
			ret = sram1_ctrl_val(adapter, cmd, &val32);
			if (ret != MACSUCCESS)
				return ret;

			macid = GET_FIELD(val32, B_AX_SS_VALUE);
			if (macid == id_empty) {
				if (link->link_list[i] != link->link_tail) {
					PLTFM_MSG_ERR("[STASCH]link_tail error!!\n");
					return MACSSLINK;
				}
				if (i >= link->link_len) {
					PLTFM_MSG_ERR("[STASCH]link_len error!!\n");
					return MACSSLINK;
				}
				break;
			}
			i++;
		} while (i < SS_LINK_SIZE);
	}

	return MACSUCCESS;
}

u32 sta_link_cfg(struct mac_ax_adapter *adapter,
		 struct mac_ax_ss_link_info *link,
		 enum mac_ax_ss_link_cfg cfg)
{
	u32 ret;
	u8 vld;

	if (link->ul) {
		ret = ul_vld_chk(adapter, &vld);
		if (ret != MACSUCCESS)
			return ret;
		if (vld == 0)
			return MACCHIPID;
	}

	ret = wmm_vld_chk(adapter, &vld, link->wmm);
	if (ret != MACSUCCESS)
		return ret;
	if (vld == 0)
		return MACCHIPID;
	switch (cfg) {
	case MAC_AX_SS_LINK_CFG_GET:
		ret = get_sta_link(adapter, link);
		break;
	case MAC_AX_SS_LINK_CFG_ADD:
		ret = add_to_link(adapter, link);
		break;
	case MAC_AX_SS_LINK_CFG_DEL:
		ret = del_from_link(adapter, link);
		break;
	case MAC_AX_SS_LINK_CFG_CLEAN:
		ret = clean_sta_link(adapter, link);
		break;
	}

	return ret;
}

void set_ss_wmm_tbl(struct mac_ax_adapter *adapter,
		    struct mac_ax_ss_wmm_tbl_ctrl *ctrl)
{
	u32 val32;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	val32 = MAC_REG_R32(R_AX_SS_CTRL);
	switch (ctrl->wmm) {
	case 0:
		val32 = SET_CLR_WORD(val32, ctrl->wmm_mapping,
				     B_AX_SS_WMM_SEL_0);
		break;
	case 1:
		val32 = SET_CLR_WORD(val32, ctrl->wmm_mapping,
				     B_AX_SS_WMM_SEL_1);
		break;
	case 2:
		val32 = SET_CLR_WORD(val32, ctrl->wmm_mapping,
				     B_AX_SS_WMM_SEL_2);
		break;
	case 3:
		val32 = SET_CLR_WORD(val32, ctrl->wmm_mapping,
				     B_AX_SS_WMM_SEL_3);
		break;
	default:
		break;
	}
	MAC_REG_W32(R_AX_SS_CTRL, val32);
}

void ss_wmm_tbl_cfg(struct mac_ax_adapter *adapter,
		    struct mac_ax_ss_wmm_tbl_ctrl *ctrl,
		    enum mac_ax_ss_wmm_tbl_cfg cfg)
{
	set_ss_wmm_tbl(adapter, ctrl);
}

u32 switch_wmm_macid(struct mac_ax_adapter *adapter,
		     struct mac_ax_ss_link_info *link,
		     enum mac_ax_ss_wmm_tbl src_link,
		     enum mac_ax_ss_wmm_tbl dst_link)
{
	u32 ret;

	link->wmm = src_link;
	ret = sta_link_cfg(adapter, link, MAC_AX_SS_LINK_CFG_DEL);
	if (ret != MACSUCCESS)
		return ret;
	link->wmm = dst_link;
	ret = sta_link_cfg(adapter, link, MAC_AX_SS_LINK_CFG_ADD);
	if (ret != MACSUCCESS)
		return ret;

	return MACSUCCESS;
}

static u32 chk_role_wmm(struct mac_ax_adapter *adapter, u8 *pmacid, u8 src_wmm)
{
	struct mac_role_tbl *role;
	u8 wmm;

	role = role_srch_no_lock(adapter, *pmacid);
	if (!role)
		return MACNOITEM;

	wmm = role->info.dbcc_role ?
	      *(adapter->dbcc_info->dbcc_wmm_list + *pmacid) : role->wmm;

	if (wmm != src_wmm)
		*pmacid = 0xFF;

	return MACSUCCESS;
}

static u32 switch_wmm_link(struct mac_ax_adapter *adapter,
			   enum mac_ax_ss_wmm_tbl src_link,
			   enum mac_ax_ss_wmm_tbl dst_link,
			   enum mac_ax_ss_wmm src_wmm)
{
	struct mac_ax_ss_link_info link;
	u32 ret;
	u32 i;

	link.ul = 0;
	for (link.ac = 0; link.ac < 4; link.ac++) {
		PLTFM_MEMSET(link.link_list, 0xFF, SS_LINK_SIZE);
		link.wmm = src_link;
		ret = sta_link_cfg(adapter, &link, MAC_AX_SS_LINK_CFG_GET);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("sta_link_cfg %d\n", ret);
			return ret;
		}
		for (i = 0; i < link.link_len; i += 3) {
			link.macid0 = link.link_list[i];
			link.macid1 = link.link_list[i + 1];
			link.macid2 = link.link_list[i + 2];
			if (link.macid0 != 0xFF) {
				ret = chk_role_wmm(adapter, &link.macid0, src_wmm);
				if (ret != MACSUCCESS)
					return ret;
			}
			if (link.macid1 != 0xFF) {
				ret = chk_role_wmm(adapter, &link.macid1, src_wmm);
				if (ret != MACSUCCESS)
					return ret;
			}
			if (link.macid2 != 0xFF) {
				ret = chk_role_wmm(adapter, &link.macid2, src_wmm);
				if (ret != MACSUCCESS)
					return ret;
			}
			if (link.macid0 != 0xFF || link.macid1 != 0xFF ||
			    link.macid2 != 0xFF) {
				ret = switch_wmm_macid(adapter, &link, src_link,
						       dst_link);
				if (ret != MACSUCCESS)
					return ret;
			}
		}
	}
	return MACSUCCESS;
}

u32 mac_ss_wmm_sta_move(struct mac_ax_adapter *adapter,
			enum mac_ax_ss_wmm src_wmm,
			enum mac_ax_ss_wmm_tbl dst_link)
{
	struct mac_ax_ss_wmm_tbl_ctrl ctrl;
	u32 ret;

	switch (src_wmm) {
	case MAC_AX_SS_WMM0:
		ctrl.wmm = 0;
		break;
	case MAC_AX_SS_WMM1:
		ctrl.wmm = 1;
		break;
	case MAC_AX_SS_WMM2:
		ctrl.wmm = 2;
		break;
	case MAC_AX_SS_WMM3:
		ctrl.wmm = 3;
		break;
	default:
		return MACFUNCINPUT;
	}
	ret = get_ss_wmm_tbl(adapter, &ctrl);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("get_ss_wmm_tbl %d\n", ret);
		return ret;
	}

	if (ctrl.wmm_mapping == dst_link)
		return MACARDYDONE;

	ret = switch_wmm_link(adapter, ctrl.wmm_mapping, dst_link, src_wmm);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("switch_wmm_link %d\n", ret);
		return ret;
	}

	ctrl.wmm_mapping = dst_link;
	ss_wmm_tbl_cfg(adapter, &ctrl, MAC_AX_SS_WMM_TBL_SET);

	return MACSUCCESS;
}

u32 mac_ss_wmm_map_upd(struct mac_ax_adapter *adapter,
		       enum mac_ax_ss_wmm src_wmm,
		       enum mac_ax_ss_wmm_tbl dst_link,
		       u8 chk_emp)
{
	struct mac_ax_ss_wmm_tbl_ctrl ctrl;
	struct mac_ax_ss_link_info link;
	u32 ret;

	switch (src_wmm) {
	case MAC_AX_SS_WMM0:
		ctrl.wmm = 0;
		break;
	case MAC_AX_SS_WMM1:
		ctrl.wmm = 1;
		break;
	case MAC_AX_SS_WMM2:
		ctrl.wmm = 2;
		break;
	case MAC_AX_SS_WMM3:
		ctrl.wmm = 3;
		break;
	default:
		return MACFUNCINPUT;
	}
	ret = get_ss_wmm_tbl(adapter, &ctrl);
	if (ret != MACSUCCESS) {
		PLTFM_MSG_ERR("get_ss_wmm_tbl %d\n", ret);
		return ret;
	}

	if (ctrl.wmm_mapping == dst_link)
		return MACSUCCESS;

	link.ul = 0;
	for (link.ac = 0; link.ac < 4; link.ac++) {
		if (!chk_emp)
			break;
		PLTFM_MEMSET(link.link_list, 0xFF, SS_LINK_SIZE);
		link.wmm = ctrl.wmm_mapping;
		ret = sta_link_cfg(adapter, &link, MAC_AX_SS_LINK_CFG_GET);
		if (ret != MACSUCCESS) {
			PLTFM_MSG_ERR("sta_link_cfg %d\n", ret);
			return ret;
		}
		if (link.link_len) {
			PLTFM_MSG_ERR("wmm%d link%d non empty %d\n",
				      link.wmm, link.ac, link.link_len);
			return MACCMP;
		}
	}

	ctrl.wmm_mapping = dst_link;
	ss_wmm_tbl_cfg(adapter, &ctrl, MAC_AX_SS_WMM_TBL_SET);

	return MACSUCCESS;
}

static void get_dl_su_rpt(struct mac_ax_adapter *adapter,
			  struct mac_ax_ss_dl_rpt_info *info)
{
	u32 val32;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	val32 = MAC_REG_R32(R_AX_SS_DL_RPT_CRTL);
	info->wmm0_max = GET_FIELD(val32, B_AX_SS_MAX_SU_NUM_0);
	info->wmm1_max = GET_FIELD(val32, B_AX_SS_MAX_SU_NUM_1);
	info->twt_wmm0_max = GET_FIELD(val32, B_AX_SS_TWT_MAX_SU_NUM_0);
	info->twt_wmm1_max = GET_FIELD(val32, B_AX_SS_TWT_MAX_SU_NUM_1);
}

static void set_dl_su_rpt(struct mac_ax_adapter *adapter,
			  struct mac_ax_ss_dl_rpt_info *info)
{
	u32 val32;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	val32 = MAC_REG_R32(R_AX_SS_DL_RPT_CRTL);
	val32 = SET_CLR_WORD(val32, info->wmm0_max, B_AX_SS_MAX_SU_NUM_0);
	val32 = SET_CLR_WORD(val32, info->wmm1_max, B_AX_SS_MAX_SU_NUM_1);
	val32 =	SET_CLR_WORD(val32, info->twt_wmm0_max,
			     B_AX_SS_TWT_MAX_SU_NUM_0);
	val32 =	SET_CLR_WORD(val32, info->twt_wmm1_max,
			     B_AX_SS_TWT_MAX_SU_NUM_1);
	MAC_REG_W32(R_AX_SS_DL_RPT_CRTL, val32);
}

static void get_dl_mu_rpt(struct mac_ax_adapter *adapter,
			  struct mac_ax_ss_dl_rpt_info *info)
{
	u32 val32;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	val32 = MAC_REG_R32(R_AX_SS_DL_MU_RPT_CRTL);
	info->wmm0_max = GET_FIELD(val32, B_AX_SS_MAX_MU_NUM_0);
	info->wmm1_max = GET_FIELD(val32, B_AX_SS_MAX_MU_NUM_1);
	info->twt_wmm0_max = GET_FIELD(val32, B_AX_SS_TWT_MAX_MU_NUM_0);
	info->twt_wmm1_max = GET_FIELD(val32, B_AX_SS_TWT_MAX_MU_NUM_1);
}

static void set_dl_mu_rpt(struct mac_ax_adapter *adapter,
			  struct mac_ax_ss_dl_rpt_info *info)
{
	u32 val32;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	val32 = MAC_REG_R32(R_AX_SS_DL_MU_RPT_CRTL);
	val32 = SET_CLR_WORD(val32, info->wmm0_max, B_AX_SS_MAX_MU_NUM_0);
	val32 = SET_CLR_WORD(val32, info->wmm1_max, B_AX_SS_MAX_MU_NUM_1);
	val32 =	SET_CLR_WORD(val32, info->twt_wmm0_max,
			     B_AX_SS_TWT_MAX_MU_NUM_0);
	val32 =	SET_CLR_WORD(val32, info->twt_wmm1_max,
			     B_AX_SS_TWT_MAX_MU_NUM_1);
	MAC_REG_W32(R_AX_SS_DL_MU_RPT_CRTL, val32);
}

static void get_dl_ru_rpt(struct mac_ax_adapter *adapter,
			  struct mac_ax_ss_dl_rpt_info *info)
{
	u32 val32;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	val32 = MAC_REG_R32(R_AX_SS_DL_RU_RPT_CRTL);
	info->wmm0_max = GET_FIELD(val32, B_AX_SS_MAX_RU_NUM_0);
	info->wmm1_max = GET_FIELD(val32, B_AX_SS_MAX_RU_NUM_1);
	info->twt_wmm0_max = GET_FIELD(val32, B_AX_SS_TWT_MAX_RU_NUM_0);
	info->twt_wmm1_max = GET_FIELD(val32, B_AX_SS_TWT_MAX_RU_NUM_1);
}

static void set_dl_ru_rpt(struct mac_ax_adapter *adapter,
			  struct mac_ax_ss_dl_rpt_info *info)
{
	u32 val32;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	val32 = MAC_REG_R32(R_AX_SS_DL_RU_RPT_CRTL);
	val32 = SET_CLR_WORD(val32, info->wmm0_max, B_AX_SS_MAX_RU_NUM_0);
	val32 = SET_CLR_WORD(val32, info->wmm1_max, B_AX_SS_MAX_RU_NUM_1);
	val32 =	SET_CLR_WORD(val32, info->twt_wmm0_max,
			     B_AX_SS_TWT_MAX_RU_NUM_0);
	val32 =	SET_CLR_WORD(val32, info->twt_wmm1_max,
			     B_AX_SS_TWT_MAX_RU_NUM_1);
	MAC_REG_W32(R_AX_SS_DL_RU_RPT_CRTL, val32);
}

void set_delay_tx_cfg(struct mac_ax_adapter *adapter,
		      struct mac_ax_delay_tx_cfg *cfg)
{
	u32 val32;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	val32 = MAC_REG_R32(R_AX_SS_CTRL);
	val32 = SET_CLR_WORD(val32, cfg->en, B_AX_SS_DELAY_TX_BAND_SEL);
	MAC_REG_W32(R_AX_SS_CTRL, val32);

	val32 = SET_WORD(cfg->vovi_to_b0, B_AX_SS_VOVI_TO_0) |
		SET_WORD(cfg->bebk_to_b0, B_AX_SS_BEBK_TO_0) |
		SET_WORD(cfg->vovi_to_b1, B_AX_SS_VOVI_TO_1) |
		SET_WORD(cfg->bebk_to_b1, B_AX_SS_BEBK_TO_1);
	MAC_REG_W32(R_AX_SS_DELAYTX_TO, val32);

	val32 = SET_WORD(cfg->vovi_len_b0, B_AX_SS_VOVI_LEN_THR_0) |
		SET_WORD(cfg->bebk_len_b0, B_AX_SS_BEBK_LEN_THR_0) |
		SET_WORD(cfg->vovi_len_b1, B_AX_SS_VOVI_LEN_THR_1) |
		SET_WORD(cfg->bebk_len_b1, B_AX_SS_BEBK_LEN_THR_1);
	MAC_REG_W32(R_AX_SS_DELAYTX_LEN_THR, val32);
}

u32 set_ss_quota_mode(struct mac_ax_adapter *adapter,
		      struct mac_ax_ss_quota_mode_ctrl *ctrl)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val32_wmm, val32_ul, ret;

	val32_wmm = MAC_REG_R32(R_AX_SS_DL_QUOTA_CTRL);
	val32_ul = MAC_REG_R32(R_AX_SS_UL_QUOTA_CTRL);
	switch (ctrl->wmm) {
	case MAC_AX_SS_WMM0:
		if (ctrl->mode == MAC_AX_SS_QUOTA_MODE_CNT)
			MAC_REG_W32(R_AX_SS_DL_QUOTA_CTRL,
				    val32_wmm | B_AX_SS_QUOTA_MODE_0);
		else
			MAC_REG_W32(R_AX_SS_DL_QUOTA_CTRL,
				    val32_wmm & ~B_AX_SS_QUOTA_MODE_0);
		break;
	case MAC_AX_SS_WMM1:
		if (ctrl->mode == MAC_AX_SS_QUOTA_MODE_CNT)
			MAC_REG_W32(R_AX_SS_DL_QUOTA_CTRL,
				    val32_wmm | B_AX_SS_QUOTA_MODE_1);
		else
			MAC_REG_W32(R_AX_SS_DL_QUOTA_CTRL,
				    val32_wmm & ~B_AX_SS_QUOTA_MODE_1);
		break;
	case MAC_AX_SS_WMM2:
		if (ctrl->mode == MAC_AX_SS_QUOTA_MODE_CNT)
			MAC_REG_W32(R_AX_SS_DL_QUOTA_CTRL,
				    val32_wmm | B_AX_SS_QUOTA_MODE_2);
		else
			MAC_REG_W32(R_AX_SS_DL_QUOTA_CTRL,
				    val32_wmm & ~B_AX_SS_QUOTA_MODE_2);
		break;
	case MAC_AX_SS_WMM3:
		if (ctrl->mode == MAC_AX_SS_QUOTA_MODE_CNT)
			MAC_REG_W32(R_AX_SS_DL_QUOTA_CTRL,
				    val32_wmm | B_AX_SS_QUOTA_MODE_3);
		else
			MAC_REG_W32(R_AX_SS_DL_QUOTA_CTRL,
				    val32_wmm & ~B_AX_SS_QUOTA_MODE_3);
		break;
	case MAC_AX_SS_UL:
		if (ctrl->mode == MAC_AX_SS_QUOTA_MODE_CNT)
			MAC_REG_W32(R_AX_SS_UL_QUOTA_CTRL,
				    val32_ul | B_AX_SS_QUOTA_MODE_UL);
		else
			MAC_REG_W32(R_AX_SS_UL_QUOTA_CTRL,
				    val32_ul & ~B_AX_SS_QUOTA_MODE_UL);
		break;
	}
#if MAC_AX_FW_REG_OFLD
	if (adapter->sm.fwdl == MAC_AX_FWDL_INIT_RDY) {
		switch (ctrl->wmm) {
		case MAC_AX_SS_WMM0:
		case MAC_AX_SS_WMM1:
		case MAC_AX_SS_UL:
			ret = check_mac_en(adapter, 0, MAC_AX_CMAC_SEL);
			if (ret != MACSUCCESS)
				return ret;
			if (ctrl->mode == MAC_AX_SS_QUOTA_MODE_TIME) {
				ret = MAC_REG_W_OFLD(R_AX_PTCL_ATM,
						     B_AX_ATM_AIRTIME_EN, 1, 1);
				if (ret != MACSUCCESS) {
					PLTFM_MSG_ERR("%s: config fail\n",
						      __func__);
					return ret;
				}
			}
			break;
		case MAC_AX_SS_WMM2:
		case MAC_AX_SS_WMM3:
			ret = check_mac_en(adapter, 1, MAC_AX_CMAC_SEL);
			if (ret != MACSUCCESS)
				return ret;
			if (ctrl->mode == MAC_AX_SS_QUOTA_MODE_TIME) {
				ret = MAC_REG_W_OFLD(R_AX_PTCL_ATM_C1,
						     B_AX_ATM_AIRTIME_EN, 1, 1);
				if (ret != MACSUCCESS) {
					PLTFM_MSG_ERR("%s: config fail\n",
						      __func__);
					return ret;
				}
			}
			break;
		}

		return MACSUCCESS;
	}
#endif

	switch (ctrl->wmm) {
	case MAC_AX_SS_WMM0:
	case MAC_AX_SS_WMM1:
	case MAC_AX_SS_UL:
		ret = check_mac_en(adapter, 0, MAC_AX_CMAC_SEL);
		if (ret != MACSUCCESS)
			return ret;
		val32_wmm = MAC_REG_R32(R_AX_PTCL_ATM);
		if (ctrl->mode == MAC_AX_SS_QUOTA_MODE_TIME)
			MAC_REG_W32(R_AX_PTCL_ATM,
				    val32_wmm | B_AX_ATM_AIRTIME_EN);
		break;
	case MAC_AX_SS_WMM2:
	case MAC_AX_SS_WMM3:
		ret = check_mac_en(adapter, 1, MAC_AX_CMAC_SEL);
		if (ret != MACSUCCESS)
			return ret;
		val32_wmm = MAC_REG_R32(R_AX_PTCL_ATM_C1);
		if (ctrl->mode == MAC_AX_SS_QUOTA_MODE_TIME)
			MAC_REG_W32(R_AX_PTCL_ATM_C1,
				    val32_wmm | B_AX_ATM_AIRTIME_EN);
		break;
	}

	return MACSUCCESS;
}

u32 ss_set_quotasetting(struct mac_ax_adapter *adapter,
			struct mac_ax_ss_quota_setting *para)
{
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);
	u32 val32;
	u32 cnt = 1000;

	if (para->ul_dl == mac_ax_issue_ul) {
		val32 = (B_AX_SS_OWN |
			SET_WORD(SS_W_QUOTA_SETTING, B_AX_SS_CMD_SEL) |
			BIT(23) | SET_WORD(para->val, B_AX_SS_VALUE) |
			para->macid);
		MAC_REG_W32(R_AX_SS_SRAM_CTRL_1, val32);
	} else {
		val32 = (B_AX_SS_OWN |
			SET_WORD(SS_W_QUOTA_SETTING, B_AX_SS_CMD_SEL) |
			SET_WORD(para->ac_type, B_AX_SS_AC) |
			SET_WORD(para->val, B_AX_SS_VALUE) | para->macid);
		MAC_REG_W32(R_AX_SS_SRAM_CTRL_1, val32);
	}

	while (--cnt) {
		val32 = MAC_REG_R32(R_AX_SS_SRAM_CTRL_1);
		if ((val32 & B_AX_SS_OWN) == 0)
			break;
		PLTFM_DELAY_US(1);
	}

	if (!cnt) {
		PLTFM_MSG_ERR("SS Set quota setting fail!!\n");
		return MACPOLLTO;
	}

	return MACSUCCESS;
}

u32 get_ss_wmm_tbl(struct mac_ax_adapter *adapter,
		   struct mac_ax_ss_wmm_tbl_ctrl *ctrl)
{
	u32 val32;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	val32 = MAC_REG_R32(R_AX_SS_CTRL);
	switch (ctrl->wmm) {
	case 0:
		ctrl->wmm_mapping =
		(enum mac_ax_ss_wmm_tbl)GET_FIELD(val32, B_AX_SS_WMM_SEL_0);
		break;
	case 1:
		ctrl->wmm_mapping =
		(enum mac_ax_ss_wmm_tbl)GET_FIELD(val32, B_AX_SS_WMM_SEL_1);
		break;
	case 2:
		ctrl->wmm_mapping =
		(enum mac_ax_ss_wmm_tbl)GET_FIELD(val32, B_AX_SS_WMM_SEL_2);
		break;
	case 3:
		ctrl->wmm_mapping =
		(enum mac_ax_ss_wmm_tbl)GET_FIELD(val32, B_AX_SS_WMM_SEL_3);
		break;
	default:
		return MACNOITEM;
	}

	return MACSUCCESS;
}

void get_delay_tx_cfg(struct mac_ax_adapter *adapter,
		      struct mac_ax_delay_tx_cfg *cfg)
{
	u32 val32;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	val32 = MAC_REG_R32(R_AX_SS_CTRL);
	cfg->en =
		(enum mac_ax_delay_tx_en)GET_FIELD(val32,
						   B_AX_SS_DELAY_TX_BAND_SEL);

	val32 = MAC_REG_R32(R_AX_SS_DELAYTX_TO);
	cfg->vovi_to_b0 = GET_FIELD(val32, B_AX_SS_VOVI_TO_0);
	cfg->bebk_to_b0 = GET_FIELD(val32, B_AX_SS_BEBK_TO_0);
	cfg->vovi_to_b1 = GET_FIELD(val32, B_AX_SS_VOVI_TO_1);
	cfg->bebk_to_b1 = GET_FIELD(val32, B_AX_SS_BEBK_TO_1);

	val32 = MAC_REG_R32(R_AX_SS_DELAYTX_LEN_THR);
	cfg->vovi_len_b0 = GET_FIELD(val32, B_AX_SS_VOVI_LEN_THR_0);
	cfg->bebk_len_b0 = GET_FIELD(val32, B_AX_SS_BEBK_LEN_THR_0);
	cfg->vovi_len_b1 = GET_FIELD(val32, B_AX_SS_VOVI_LEN_THR_1);
	cfg->bebk_len_b1 = GET_FIELD(val32, B_AX_SS_BEBK_LEN_THR_1);
}

void mac_ss_dl_rpt_cfg(struct mac_ax_adapter *adapter,
		       struct mac_ax_ss_dl_rpt_info *info,
		       enum mac_ax_ss_rpt_cfg cfg)
{
	switch (cfg) {
	case MAC_AX_SS_DL_SU_RPT_CFG_GET:
		get_dl_su_rpt(adapter, info);
		break;
	case MAC_AX_SS_DL_SU_RPT_CFG_SET:
		set_dl_su_rpt(adapter, info);
		break;
	case MAC_AX_SS_DL_MU_RPT_CFG_GET:
		get_dl_mu_rpt(adapter, info);
		break;
	case MAC_AX_SS_DL_MU_RPT_CFG_SET:
		set_dl_mu_rpt(adapter, info);
		break;
	case MAC_AX_SS_DL_RU_RPT_CFG_GET:
		get_dl_ru_rpt(adapter, info);
		break;
	case MAC_AX_SS_DL_RU_RPT_CFG_SET:
		set_dl_ru_rpt(adapter, info);
		break;
	}
}
u32 mac_ss_stat_chk(struct mac_ax_adapter *adapter)
{
	u32 val32 = 0;
	u32 r_val32;
	u8 fw_vld;
	struct mac_ax_intf_ops *ops = adapter_to_intf_ops(adapter);

	switch (adapter->drv_info->sw_chip_id) {
	case MAC_AX_CHIP_ID_8852A:
		fw_vld = SS_FW_SUPPORT_8852A;
		break;
	case MAC_AX_CHIP_ID_8852B:
		fw_vld = SS_FW_SUPPORT_8852B;
		break;
	default:
		fw_vld = 0;
		break;
	}

	r_val32 = MAC_REG_R32(R_AX_SS_DBG_3);
	if (r_val32 & B_AX_SS_HW_DECR_LEN_UDN)
		val32 |= SS_TX_HW_LEN_UDN;
	if (r_val32 & B_AX_SS_SW_DECR_LEN_UDN)
		val32 |= SS_TX_SW_LEN_UDN;
	if (r_val32 & B_AX_SS_HW_ADD_LEN_OVF)
		val32 |= SS_TX_HW_LEN_OVF;

	r_val32 = MAC_REG_R32(R_AX_SS_DBG_2);
	if (fw_vld) {
		if (GET_FIELD(r_val32, B_AX_SS_FWTX_STAT) != 1)
			val32 |= SS_STAT_FWTX;
	}
	if (GET_FIELD(r_val32, B_AX_SS_RPTA_STAT) != 1)
		val32 |= SS_STAT_RPTA;
	if (GET_FIELD(r_val32, B_AX_SS_WDEA_STAT_V1) != 1)
		val32 |= SS_STAT_WDEA;
	if (GET_FIELD(r_val32, B_AX_SS_PLEA_STAT_V1) != 1)
		val32 |= SS_STAT_PLEA;

	r_val32 = MAC_REG_R32(R_AX_SS_DBG_1);
	if (GET_FIELD(r_val32, B_AX_SS_ULRU_STAT) > 1)
		val32 |= SS_STAT_ULRU;
	if (GET_FIELD(r_val32, B_AX_SS_DLTX_STAT) > 1)
		val32 |= SS_STAT_DLTX;

	return val32;
}

