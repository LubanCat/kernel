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
#ifndef _ERR_FLAG_AUTO_GEN_H_
#define _ERR_FLAG_AUTO_GEN_H_

/**
 * @enum mac_ax_err_flag_cat
 *
 * @brief mac_ax_err_flag_cat
 *
 * @var mac_ax_err_flag_cat::MAC_AX_ERR_FLAG_CAT_COMMON
 * Please Place Description here.
 * @var mac_ax_err_flag_cat::MAC_AX_ERR_FLAG_CAT_PCIE
 * Please Place Description here.
 * @var mac_ax_err_flag_cat::MAC_AX_ERR_FLAG_CAT_USB
 * Please Place Description here.
 * @var mac_ax_err_flag_cat::MAC_AX_ERR_FLAG_CAT_SDIO
 * Please Place Description here.
 * @var mac_ax_err_flag_cat::MAC_AX_ERR_FLAG_CAT_LAST
 * Please Place Description here.
 * @var mac_ax_err_flag_cat::MAC_AX_ERR_FLAG_CAT_MAX
 * Please Place Description here.
 * @var mac_ax_err_flag_cat::MAC_AX_ERR_FLAG_CAT_INVALID
 * Please Place Description here.
 */
enum mac_ax_err_flag_cat {
	MAC_AX_ERR_FLAG_CAT_COMMON = 0,
	MAC_AX_ERR_FLAG_CAT_PCIE,
	MAC_AX_ERR_FLAG_CAT_USB,
	MAC_AX_ERR_FLAG_CAT_SDIO,

	/* keep last */
	MAC_AX_ERR_FLAG_CAT_LAST,
	MAC_AX_ERR_FLAG_CAT_MAX = MAC_AX_ERR_FLAG_CAT_LAST,
	MAC_AX_ERR_FLAG_CAT_INVALID = MAC_AX_ERR_FLAG_CAT_LAST
};

/**
 * @enum mac_ax_err_dbg_lvl
 *
 * @brief mac_ax_err_dbg_lvl
 *
 * @var mac_ax_err_dbg_lvl::MAC_AX_ERR_FLAG_DBG_LVL_RUNTIME
 * Please Place Description here.
 * @var mac_ax_err_dbg_lvl::MAC_AX_ERR_FLAG_DBG_LVL_FULL
 * Please Place Description here.
 * @var mac_ax_err_dbg_lvl::MAC_AX_ERR_FLAG_DBG_LVL_LAST
 * Please Place Description here.
 * @var mac_ax_err_dbg_lvl::MAC_AX_ERR_FLAG_DBG_LVL_LAST
 * Please Place Description here.
 * @var mac_ax_err_dbg_lvl::MAC_AX_ERR_FLAG_DBG_LVL_LAST
 * Please Place Description here.
 */
enum mac_ax_err_dbg_lvl {
	MAC_AX_ERR_FLAG_DBG_LVL_RUNTIME = 0,
	MAC_AX_ERR_FLAG_DBG_LVL_FULL,

	/* keep last */
	MAC_AX_ERR_FLAG_DBG_LVL_LAST,
	MAC_AX_ERR_FLAG_DBG_LVL_MAX = MAC_AX_ERR_FLAG_DBG_LVL_LAST,
	MAC_AX_ERR_FLAG_DBG_LVL_INVALID = MAC_AX_ERR_FLAG_DBG_LVL_LAST
};

/**
 * @struct mac_ax_err_flag_sts
 * @brief mac_ax_err_flag_sts
 *
 * @var mac_ax_err_flag_sts::total_cnt
 * Please Place Description here.
 * @var mac_ax_err_flag_sts::fail_cnt
 * Please Place Description here.
 * @var mac_ax_err_flag_sts::dbg_lvl
 * Please Place Description here.
 */
struct mac_ax_err_flag_sts {
	u32 total_cnt;
	u32 fail_cnt;
	u32 dbg_lvl;
};


#endif/* ERR_FLAG_AUTO_GEN_H */