/******************************************************************************
 *
 * Copyright(c) 2007 - 2022 Realtek Corporation.
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
 *****************************************************************************/
#define _RTW_CHPLAN_C_

#include <drv_types.h>
#include "regdb/rtw_regdb.h"

bool rtw_chplan_init(void)
{
	if (!regdb_ops.init || regdb_ops.init())
		return true;

	rtw_warn_on(1);
	return false;
}

void rtw_chplan_deinit(void)
{
	if (regdb_ops.deinit)
		regdb_ops.deinit();
}

u8 rtw_chplan_get_default_regd_2g(u8 id)
{
	return regdb_ops.get_default_regd_2g(id);
}

#if CONFIG_IEEE80211_BAND_5GHZ
u8 rtw_chplan_get_default_regd_5g(u8 id)
{
	return regdb_ops.get_default_regd_5g(id);
}
#endif

bool rtw_is_channel_plan_valid(u8 id)
{
	return regdb_ops.is_domain_code_valid(id);
}

/*
 * Search the @param ch in chplan by given @param id
 * @id: the given channel plan id
 * @ch: the given channel number
 *
 * return the index of channel_num in channel_set, -1 if not found
 */
static bool rtw_chplan_get_ch(u8 id, u32 ch, u8 *flags)
{
	return regdb_ops.domain_get_ch(id, ch, flags);
}

#if CONFIG_IEEE80211_BAND_6GHZ
u8 rtw_chplan_get_default_regd_6g(u8 id)
{
	return regdb_ops.get_default_regd_6g(id);
}

bool rtw_is_channel_plan_6g_valid(u8 id)
{
	return regdb_ops.is_domain_code_6g_valid(id);
}

/*
 * Search the @param ch in chplan by given @param id
 * @id: the given channel plan id
 * @ch: the given channel number
 *
 * return the index of channel_num in channel_set, -1 if not found
 */
static bool rtw_chplan_6g_get_ch(u8 id, u32 ch, u8 *flags)
{
	return regdb_ops.domain_6g_get_ch(id, ch, flags);
}
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

void rtw_rfctl_addl_ch_disable_conf_init(struct rf_ctl_t *rfctl, struct registry_priv *regsty)
{
	int i;

	rfctl->dis_ch_flags = regsty->dis_ch_flags
		#if !CONFIG_DFS
		| RTW_CHF_DFS
		#endif
		;

	for (i = 0; i < MAX_CHANNEL_NUM_2G_5G; i++)
		rfctl->excl_chs[i] = regsty->excl_chs[i];
#if CONFIG_IEEE80211_BAND_6GHZ
	for (i = 0; i < MAX_CHANNEL_NUM_6G; i++)
		rfctl->excl_chs_6g[i] = regsty->excl_chs_6g[i];
#endif
}

static bool rtw_rfctl_is_excl_chs(struct rf_ctl_t *rfctl, u8 ch)
{
	int i;

	for (i = 0; i < MAX_CHANNEL_NUM_2G_5G; i++) {
		if (rfctl->excl_chs[i] == 0)
			break;
		if (rfctl->excl_chs[i] == ch)
			return true;
	}
	return false;
}

#if CONFIG_IEEE80211_BAND_6GHZ
static bool rtw_rfctl_is_excl_chs_6g(struct rf_ctl_t *rfctl, u8 ch)
{
	int i;

	for (i = 0; i < MAX_CHANNEL_NUM_6G; i++) {
		if (rfctl->excl_chs_6g[i] == 0)
			break;
		if (rfctl->excl_chs_6g[i] == ch)
			return true;
	}
	return false;
}
#endif

/*
 * Check if the @param ch, bw, offset is valid for the given @param id, id_6g
 * @ch_set: the given channel set
 * @ch: the given channel number
 * @bw: the given bandwidth
 * @offset: the given channel offset
 *
 * return valid (1) or not (0)
 */
u8 rtw_chplan_is_bchbw_valid(u8 id, u8 id_6g, enum band_type band, u8 ch, u8 bw, u8 offset
	, bool allow_primary_passive, bool allow_passive, struct rf_ctl_t *rfctl)
{
	u8 cch;
	u8 *op_chs;
	u8 op_ch_num;
	u8 op_ch;
	u8 valid = 0;
	int i;
	u8 flags;

	cch = rtw_get_center_ch_by_band(band, ch, bw, offset);

	if (!rtw_get_op_chs_by_bcch_bw(band, cch, bw, &op_chs, &op_ch_num))
		goto exit;

	for (i = 0; i < op_ch_num; i++) {
		op_ch = *(op_chs + i);
		if (0)
			RTW_INFO("%u,%u,%u - cch:%u, bw:%u, op_ch:%u\n", ch, bw, offset, cch, bw, op_ch);
		#if CONFIG_IEEE80211_BAND_6GHZ
		if (band == BAND_ON_6G) {
			if (!rtw_chplan_6g_get_ch(id_6g, op_ch, &flags)
				|| (rfctl && (rfctl->dis_ch_flags & flags))
				|| (rfctl && rtw_rfctl_is_excl_chs_6g(rfctl, op_ch) == _TRUE))
				break;
		} else
		#endif
		{
			if (!rtw_chplan_get_ch(id, op_ch, &flags)
				|| (rfctl && (rfctl->dis_ch_flags & flags))
				|| (rfctl && rtw_rfctl_is_excl_chs(rfctl, op_ch) == _TRUE))
				break;
		}
		if (flags & RTW_CHF_NO_IR) {
			if (!allow_passive
				|| (!allow_primary_passive && op_ch == ch))
				break;
		}
	}

	if (op_ch_num != 0 && i == op_ch_num)
		valid = 1;

exit:
	return valid;
}

const char *_regd_src_str[] = {
	[REGD_SRC_RTK_PRIV] = "RTK_PRIV",
	[REGD_SRC_OS] = "OS",
	[REGD_SRC_NUM] = "UNKNOWN",
};

static void rtw_chset_apply_from_rtk_priv(struct rtw_chset *chset, u8 chplan, u8 chplan_6g, u8 d_flags)
{
	RT_CHANNEL_INFO *chinfo;
	u8 i;
	u8 flags;
	bool chplan_valid = rtw_is_channel_plan_valid(chplan);
#if CONFIG_IEEE80211_BAND_6GHZ
	bool chplan_6g_valid = rtw_is_channel_plan_valid(chplan_6g);
#endif
	bool usable_ch;

	RTW_INFO("%s chplan:0x%02X chplan_6g:0x%02X\n", __func__, chplan, chplan_6g);
	rtw_warn_on(!chplan_valid);
#if CONFIG_IEEE80211_BAND_6GHZ
	rtw_warn_on(!chplan_6g_valid);
#endif

	for (i = 0; i < chset->chs_len; i++) {
		chinfo = &chset->chs[i];
		if (chinfo->flags & RTW_CHF_DIS)
			continue;

		if (chinfo->band == BAND_ON_24G
			#if CONFIG_IEEE80211_BAND_5GHZ
			|| chinfo->band == BAND_ON_5G
			#endif
		) {
			if (!chplan_valid)
				continue;
			usable_ch = rtw_chplan_get_ch(chplan, chinfo->ChannelNum, &flags);
			if (usable_ch && (flags & d_flags))
				usable_ch = false;
		}
		#if CONFIG_IEEE80211_BAND_6GHZ
		else if (chinfo->band == BAND_ON_6G) {
			if (!chplan_6g_valid)
				continue;
			usable_ch = rtw_chplan_6g_get_ch(chplan_6g, chinfo->ChannelNum, &flags);
			if (usable_ch && (flags & d_flags))
				usable_ch = false;
		}
		#endif
		else
			usable_ch = false;

		if (usable_ch)
			chinfo->flags |= flags;
		else
			chinfo->flags = RTW_CHF_DIS;
	}
}

static void rtw_rfctl_chset_apply_regd_reqs(struct rf_ctl_t *rfctl, u8 d_flags, bool req_lock)
{
	struct regd_req_t *req;
	_list *cur, *head;

	/* apply regd reqs */
	if (req_lock)
		_rtw_mutex_lock_interruptible(&rfctl->regd_req_mutex);

	head = &rfctl->regd_req_list;
	cur = get_next(head);
	while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
		req = LIST_CONTAINOR(cur, struct regd_req_t, list);
		cur = get_next(cur);
		if (!req->applied)
			continue;

		if (req->src == REGD_SRC_RTK_PRIV) {
			u8 domain_code_6g = RTW_CHPLAN_6G_NULL;

			#if CONFIG_IEEE80211_BAND_6GHZ
			domain_code_6g = req->chplan.domain_code_6g;
			#endif
			rtw_chset_apply_from_rtk_priv(&rfctl->chset, req->chplan.domain_code, domain_code_6g, d_flags);
		}
		#ifdef CONFIG_REGD_SRC_FROM_OS
		else if (req->src == REGD_SRC_OS)
			rtw_chset_apply_from_os(&rfctl->chset, d_flags);
		#endif
		else
			rtw_warn_on(1);
	}

	if (req_lock)
		_rtw_mutex_unlock(&rfctl->regd_req_mutex);
}

static void rtw_rfctl_chset_apply_regulatory(struct dvobj_priv *dvobj, bool req_lock)
{
	struct rf_ctl_t *rfctl = dvobj_to_rfctl(dvobj);
	struct rtw_chset *chset = &rfctl->chset;
	u8 d_band_bmp = 0;
	u8 d_flags = rfctl->dis_ch_flags;
	RT_CHANNEL_INFO *chinfo;
	int i;

	d_band_bmp |= !RFCTL_REG_EN_11A(rfctl) ? BAND_CAP_5G : 0;

	/* reset flags of all channels */
	for (i = 0; i < chset->chs_len; i++) {
		chinfo = &chset->chs[i];
		if ((band_to_band_cap(chinfo->band) & d_band_bmp)
			|| ((chinfo->band == BAND_ON_24G || chinfo->band == BAND_ON_5G)
				&& rtw_rfctl_is_excl_chs(rfctl, chinfo->ChannelNum) == true)
			#if CONFIG_IEEE80211_BAND_6GHZ
			|| (chinfo->band == BAND_ON_6G
				&& rtw_rfctl_is_excl_chs_6g(rfctl, chinfo->ChannelNum) == true)
			#endif
		)
			chinfo->flags = RTW_CHF_DIS;
		else
			chinfo->flags = 0;
	}

	rtw_rfctl_chset_apply_regd_reqs(rfctl, d_flags, req_lock);

	chset->enable_ch_num = 0;
	for (i = 0; i < BAND_MAX; i++)
		chset->enable_ch_num_of_band[i] = 0;

	for (i = 0; i < chset->chs_len; i++) {
		chinfo = &chset->chs[i];

		chinfo->reg_no_ir = !!(chinfo->flags & RTW_CHF_NO_IR);

		if (chinfo->flags & RTW_CHF_DIS)
			continue;
		chset->enable_ch_num++;
		chset->enable_ch_num_of_band[chinfo->band]++;

		if (chinfo->flags & RTW_CHF_NO_IR && CH_IS_BCN_HINT(chinfo)
			&& rtw_rfctl_reg_allow_beacon_hint(rfctl)
			&& rtw_chinfo_allow_beacon_hint(chinfo))
			chinfo->flags &= ~RTW_CHF_NO_IR;

		/* logs for channel with NO_IR but can't be cleared through beacon hint */
		if (chinfo->flags & RTW_CHF_NO_IR) {
			if (!rtw_rfctl_reg_allow_beacon_hint(rfctl) || !rtw_chinfo_allow_beacon_hint(chinfo))
				RTW_INFO("band:%s ch%u is NO_IR%s while beacon hint not allowed\n"
					, band_str(chinfo->band), chinfo->ChannelNum, chinfo->flags & RTW_CHF_DFS ? " DFS" : "");
		}
	}

	if (chset->enable_ch_num) {
		char buf[] = "xx,xx,xx"; /* buf for BAND_ON_24G ~ BAND_ON_6G */
		size_t buf_len = strlen(buf) + 1;
		char *pos = buf;

		for (i = 0; i < BAND_MAX; i++)
			pos += snprintf(pos, buf_len - (pos - buf), "%u,", chset->enable_ch_num_of_band[i]);
		if (i)
			*(pos - 1) = '\0';

		RTW_INFO("%s ch num:%d(%s)\n", __func__, chset->enable_ch_num, buf);
	} else
		RTW_WARN("%s final chset has no channel\n", __func__);
}

/* domain status specific beacon hint rules */
#ifndef RTW_CHPLAN_BEACON_HINT_SPECIFIC_COUNTRY
#define RTW_CHPLAN_BEACON_HINT_SPECIFIC_COUNTRY 0
#endif

bool rtw_rfctl_reg_allow_beacon_hint(struct rf_ctl_t *rfctl)
{
	return RTW_CHPLAN_BEACON_HINT_SPECIFIC_COUNTRY || RFCTL_REG_WORLDWIDE(rfctl)
		|| RFCTL_REG_ALPHA2_UNSPEC(rfctl) || RFCTL_REG_INTERSECTED(rfctl);
}

/* channel specific beacon hint rules */
#ifndef RTW_CHPLAN_BEACON_HINT_ON_2G_CH_1_11
#define RTW_CHPLAN_BEACON_HINT_ON_2G_CH_1_11 0
#endif
#ifndef RTW_CHPLAN_BEACON_HINT_ON_DFS_CH
#define RTW_CHPLAN_BEACON_HINT_ON_DFS_CH 0
#endif

bool rtw_chinfo_allow_beacon_hint(struct _RT_CHANNEL_INFO *chinfo)
{
	return (RTW_CHPLAN_BEACON_HINT_ON_2G_CH_1_11 || !(chinfo->band == BAND_ON_24G && chinfo->ChannelNum <= 11))
		&& (RTW_CHPLAN_BEACON_HINT_ON_DFS_CH || !(chinfo->flags & RTW_CHF_DFS));
}

u8 rtw_process_beacon_hint(struct rf_ctl_t *rfctl, struct wlan_network *network)
{
	u8 act_cnt = 0;

	if (rtw_network_chk_regu_ies(rfctl, network)) {
		struct rtw_chset *chset = &rfctl->chset;
		WLAN_BSSID_EX *bss = &network->network;
		enum band_type band = BSS_EX_OP_BAND(bss);
		u8 ch = BSS_EX_OP_CH(bss);
		RT_CHANNEL_INFO *chinfo = rtw_chset_get_chinfo_by_bch(chset, band, ch, false);

		if (!chinfo)
			goto exit;

		if (rfctl->regd_src == REGD_SRC_RTK_PRIV) {
			chinfo->bcn_hint_end_time = rtw_get_current_time() + rtw_ms_to_systime(rfctl->bcn_hint_valid_ms);
			if (chinfo->bcn_hint_end_time == RTW_BCN_HINT_STOPPED)
				chinfo->bcn_hint_end_time++;
		}

		if ((chinfo->flags & RTW_CHF_NO_IR)
			&& rtw_rfctl_reg_allow_beacon_hint(rfctl)
			&& rtw_chinfo_allow_beacon_hint(chinfo)
		) {
			RTW_INFO("%s: change band:%s ch:%d to active\n", __func__, band_str(band), ch);
			chinfo->flags &= ~RTW_CHF_NO_IR;
			act_cnt++;
		}
	}

exit:
	return act_cnt;
}

/* called at cmd handler */
void rtw_beacon_hint_ch_change_notifier(struct rf_ctl_t *rfctl)
{
	_adapter *adapter = dvobj_get_primary_adapter(rfctl_to_dvobj(rfctl));
	#ifdef CONFIG_IOCTL_CFG80211
	struct get_chplan_resp *chplan;

	if (rtw_get_chplan_cmd(adapter, RTW_CMDF_DIRECTLY, &chplan) != _SUCCESS
		|| rtw_regd_change_complete_async(adapter_to_wiphy(adapter), chplan) != _SUCCESS)
		rtw_warn_on(1);
	#endif

	op_class_pref_apply_regulatory(rfctl, REG_BEACON_HINT);
	rtw_nlrtw_reg_beacon_hint_event(adapter);
}

static void rtw_beacon_hint_expire(struct rf_ctl_t *rfctl)
{
	struct dvobj_priv *dvobj;
	struct rtw_chset *chset;
	RT_CHANNEL_INFO *chinfo;
	int i;
	bool deactivate = false;

	if (rfctl->regd_src != REGD_SRC_RTK_PRIV)
		return;

	dvobj = rfctl_to_dvobj(rfctl);
	chset = &rfctl->chset;

	/* don't expire when device is linking/linked/beaconing */
	for (i = HW_BAND_0; i < HW_BAND_MAX; i++)
		if (HWBAND_STA_LG_NUM(dvobj, i) || HWBAND_STA_LD_NUM(dvobj, i) || HWBAND_WPS_NUM(dvobj, i)
			|| HWBAND_AP_NUM(dvobj, i) || HWBAND_MESH_NUM(dvobj, i)|| HWBAND_ADHOC_NUM(dvobj, i))
			break;
	if (i < HW_BAND_MAX)
		return;

	for (i = 0; i < chset->chs_len; i++) {
		chinfo = &chset->chs[i];
		if (!CH_IS_BCN_HINT(chinfo) && !CH_IS_BCN_HINT_STOPPED(chinfo)) {
			chinfo->bcn_hint_end_time = RTW_BCN_HINT_STOPPED;
			if (chinfo->reg_no_ir && !(chinfo->flags & RTW_CHF_NO_IR)) {
				RTW_INFO("%s: change band:%s ch:%d to passive\n", __func__
					, band_str(chinfo->band), chinfo->ChannelNum);
				chinfo->flags |= RTW_CHF_NO_IR;
				deactivate = true;
			}
		}
	}

	if (deactivate)
		rtw_beacon_hint_ch_change_notifier(rfctl);
}

const char *const _regd_inr_str[] = {
	[RTW_REGD_SET_BY_INIT]		= "INIT",
	[RTW_REGD_SET_BY_USER]		= "USER",
	[RTW_REGD_SET_BY_COUNTRY_IE]	= "COUNTRY_IE",
	[RTW_REGD_SET_BY_EXTRA]		= "EXTRA",
	[RTW_REGD_SET_BY_DRIVER]	= "DRIVER",
	[RTW_REGD_SET_BY_CORE]		= "CORE",
	[RTW_REGD_SET_BY_NUM]		= "UNKNOWN",
};

const char *const _regd_str[] = {
	[RTW_REGD_NA]		= "NA",
	[RTW_REGD_FCC]		= "FCC",
	[RTW_REGD_MKK]		= "MKK",
	[RTW_REGD_ETSI]		= "ETSI",
	[RTW_REGD_IC]		= "IC",
	[RTW_REGD_KCC]		= "KCC",
	[RTW_REGD_NCC]		= "NCC",
	[RTW_REGD_ACMA]		= "ACMA",
	[RTW_REGD_CHILE]	= "CHILE",
	[RTW_REGD_MEX]		= "MEX",
	[RTW_REGD_WW]		= "WW",
};

const char *const _env_str[] = {
	[RTW_ENV_ANY]		= "ANY",
	[RTW_ENV_INDOOR]	= "IN",
	[RTW_ENV_OUTDOOR]	= "OUT",
};

const char *const _rtw_edcca_mode_str[] = {
	[RTW_EDCCA_NORM]	= "NORMAL",
	[RTW_EDCCA_CS]		= "CS",
	[RTW_EDCCA_ADAPT]	= "ADAPT",
	[RTW_EDCCA_CBP]		= "CBP",
};

const char *const _rtw_dfs_regd_str[] = {
	[RTW_DFS_REGD_NONE]	= "NONE",
	[RTW_DFS_REGD_FCC]	= "FCC",
	[RTW_DFS_REGD_MKK]	= "MKK",
	[RTW_DFS_REGD_ETSI]	= "ETSI",
	[RTW_DFS_REGD_KCC]	= "KCC",
};

const char *const _txpwr_lmt_str[] = {
	[TXPWR_LMT_NONE]	= "NONE",
	[TXPWR_LMT_FCC]		= "FCC",
	[TXPWR_LMT_MKK]		= "MKK",
	[TXPWR_LMT_ETSI]	= "ETSI",
	[TXPWR_LMT_IC]		= "IC",
	[TXPWR_LMT_KCC]		= "KCC",
	[TXPWR_LMT_NCC]		= "NCC",
	[TXPWR_LMT_ACMA]	= "ACMA",
	[TXPWR_LMT_CHILE]	= "CHILE",
	[TXPWR_LMT_UKRAINE]	= "UKRAINE",
	[TXPWR_LMT_MEXICO]	= "MEXICO",
	[TXPWR_LMT_CN]		= "CN",
	[TXPWR_LMT_QATAR]	= "QATAR",
	[TXPWR_LMT_UK]		= "UK",
	[TXPWR_LMT_THAILAND]	= "THAILAND",
	[TXPWR_LMT_WW]		= "WW",
	[TXPWR_LMT_NUM]		= NULL,
};

const REGULATION_TXPWR_LMT _txpwr_lmt_alternate[] = {
	[TXPWR_LMT_NONE]	= TXPWR_LMT_NONE,
	[TXPWR_LMT_FCC]		= TXPWR_LMT_FCC,
	[TXPWR_LMT_MKK]		= TXPWR_LMT_MKK,
	[TXPWR_LMT_ETSI]	= TXPWR_LMT_ETSI,
	[TXPWR_LMT_WW]		= TXPWR_LMT_WW,
	[TXPWR_LMT_NUM]		= TXPWR_LMT_NUM,

	[TXPWR_LMT_IC]		= TXPWR_LMT_FCC,
	[TXPWR_LMT_KCC]		= TXPWR_LMT_ETSI,
	[TXPWR_LMT_NCC]		= TXPWR_LMT_FCC,
	[TXPWR_LMT_ACMA]	= TXPWR_LMT_ETSI,
	[TXPWR_LMT_CHILE]	= TXPWR_LMT_FCC,
	[TXPWR_LMT_UKRAINE]	= TXPWR_LMT_ETSI,
	[TXPWR_LMT_MEXICO]	= TXPWR_LMT_FCC,
	[TXPWR_LMT_CN]		= TXPWR_LMT_ETSI,
	[TXPWR_LMT_QATAR]	= TXPWR_LMT_ETSI,
	[TXPWR_LMT_UK]		= TXPWR_LMT_ETSI,
	[TXPWR_LMT_THAILAND]	= TXPWR_LMT_ETSI,
};

#if CONFIG_IEEE80211_BAND_6GHZ
const char *const _txpwr_lmt_6g_cate_str[] = {
	[TXPWR_LMT_6G_CATE_VLP]	= "VLP",
	[TXPWR_LMT_6G_CATE_LPI]	= "LPI",
	[TXPWR_LMT_6G_CATE_STD]	= "STD",
	[TXPWR_LMT_6G_CATE_NUM]	= NULL,
};

#define TXPWR_LMT_6G_CATE_STR_DECLARE(reg) \
	[TXPWR_LMT_##reg][TXPWR_LMT_6G_CATE_VLP] = #reg"_VLP", \
	[TXPWR_LMT_##reg][TXPWR_LMT_6G_CATE_LPI] = #reg"_LPI", \
	[TXPWR_LMT_##reg][TXPWR_LMT_6G_CATE_STD] = #reg"_STD"

const char *const _txpwr_lmt_6g_str[][TXPWR_LMT_6G_CATE_NUM] = {
	[TXPWR_LMT_NONE][TXPWR_LMT_6G_CATE_VLP]	= "NONE",
	[TXPWR_LMT_NONE][TXPWR_LMT_6G_CATE_LPI]	= "NONE",
	[TXPWR_LMT_NONE][TXPWR_LMT_6G_CATE_STD]	= "NONE",
	[TXPWR_LMT_WW][TXPWR_LMT_6G_CATE_VLP]	= "WW",
	[TXPWR_LMT_WW][TXPWR_LMT_6G_CATE_LPI]	= "WW",
	[TXPWR_LMT_WW][TXPWR_LMT_6G_CATE_STD]	= "WW",
	TXPWR_LMT_6G_CATE_STR_DECLARE(FCC),
	TXPWR_LMT_6G_CATE_STR_DECLARE(MKK),
	TXPWR_LMT_6G_CATE_STR_DECLARE(ETSI),
	TXPWR_LMT_6G_CATE_STR_DECLARE(IC),
	TXPWR_LMT_6G_CATE_STR_DECLARE(KCC),
	TXPWR_LMT_6G_CATE_STR_DECLARE(NCC),
	TXPWR_LMT_6G_CATE_STR_DECLARE(ACMA),
	TXPWR_LMT_6G_CATE_STR_DECLARE(CHILE),
	TXPWR_LMT_6G_CATE_STR_DECLARE(UKRAINE),
	TXPWR_LMT_6G_CATE_STR_DECLARE(MEXICO),
	TXPWR_LMT_6G_CATE_STR_DECLARE(CN),
	TXPWR_LMT_6G_CATE_STR_DECLARE(QATAR),
	TXPWR_LMT_6G_CATE_STR_DECLARE(UK),
	TXPWR_LMT_6G_CATE_STR_DECLARE(THAILAND),
};

u8 _rtw_env_to_txpwr_lmt_6g_cate_map[RTW_ENV_NUM] = {
	[RTW_ENV_ANY]		= BIT(TXPWR_LMT_6G_CATE_VLP),
	[RTW_ENV_INDOOR]	= BIT(TXPWR_LMT_6G_CATE_VLP) | BIT(TXPWR_LMT_6G_CATE_LPI),
	[RTW_ENV_OUTDOOR]	= BIT(TXPWR_LMT_6G_CATE_VLP),
};

#define rtw_env_to_txpwr_lmt_6g_cate_map(env) (((env) >= RTW_ENV_NUM) ? 0 : _rtw_env_to_txpwr_lmt_6g_cate_map[(env)])

const char *const _chplan_6g_cate_str[] = {
	[CHPLAN_6G_CATE_VLP]	= "VLP",
	[CHPLAN_6G_CATE_LPI]	= "LPI",
	[CHPLAN_6G_CATE_STD]	= "STD",
};

#define chplan_6g_cate_str(cate) (((cate) >= CHPLAN_6G_CATE_NUM) ? "-" : _chplan_6g_cate_str[(cate)])

u8 _chplan_6g_cate_to_txpwr_lmt_6g_cate_map[] = {
	[CHPLAN_6G_CATE_VLP]	= BIT(TXPWR_LMT_6G_CATE_VLP),
	[CHPLAN_6G_CATE_LPI]	= BIT(TXPWR_LMT_6G_CATE_VLP) | BIT(TXPWR_LMT_6G_CATE_LPI),
	[CHPLAN_6G_CATE_STD]	= BIT(TXPWR_LMT_6G_CATE_VLP) | BIT(TXPWR_LMT_6G_CATE_LPI) | BIT(TXPWR_LMT_6G_CATE_STD),
};

#define chplan_6g_cate_to_txpwr_lmt_6g_cate_map(cate) (((cate) >= CHPLAN_6G_CATE_NUM) ? 0 : _chplan_6g_cate_to_txpwr_lmt_6g_cate_map[(cate)])

u8 _reg_info_to_chplan_6g_cate_map[] = {
	[CIS_6G_REG_IN_AP]	= BIT(CHPLAN_6G_CATE_VLP) | BIT(CHPLAN_6G_CATE_LPI),
	[CIS_6G_REG_SP_AP]	= BIT(CHPLAN_6G_CATE_VLP) | BIT(CHPLAN_6G_CATE_STD),
	[CIS_6G_REG_VLP_AP]	= BIT(CHPLAN_6G_CATE_VLP),
	[CIS_6G_REG_IN_EN_AP]	= BIT(CHPLAN_6G_CATE_VLP) | BIT(CHPLAN_6G_CATE_LPI),
	[CIS_6G_REG_IN_SP_AP]	= BIT(CHPLAN_6G_CATE_VLP) | BIT(CHPLAN_6G_CATE_STD),
};
#define reg_info_to_chplan_6g_cate_map(reg_info) (((reg_info) >= CIS_6G_REG_NUM) ? 0 : _reg_info_to_chplan_6g_cate_map[(reg_info)])
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

const enum rtw_edcca_mode_t _rtw_regd_to_edcca_mode[RTW_REGD_NUM] = {
	[RTW_REGD_NA]	= RTW_EDCCA_MODE_NUM,
	[RTW_REGD_MKK]	= RTW_EDCCA_CS,
	[RTW_REGD_ETSI]	= RTW_EDCCA_ADAPT,
	[RTW_REGD_WW]	= RTW_EDCCA_ADAPT,
};

#if CONFIG_IEEE80211_BAND_6GHZ
const enum rtw_edcca_mode_t _rtw_regd_to_edcca_mode_6g[RTW_REGD_NUM] = {
	[RTW_REGD_NA]	= RTW_EDCCA_MODE_NUM,
	[RTW_REGD_FCC]	= RTW_EDCCA_CBP,
	[RTW_REGD_MKK]	= RTW_EDCCA_CS,
	[RTW_REGD_ETSI]	= RTW_EDCCA_ADAPT,
	[RTW_REGD_WW]	= RTW_EDCCA_CBP,
};
#endif

const REGULATION_TXPWR_LMT _rtw_regd_to_txpwr_lmt[] = {
	[RTW_REGD_NA]		= TXPWR_LMT_NUM,
	[RTW_REGD_FCC]		= TXPWR_LMT_FCC,
	[RTW_REGD_MKK]		= TXPWR_LMT_MKK,
	[RTW_REGD_ETSI]		= TXPWR_LMT_ETSI,
	[RTW_REGD_IC]		= TXPWR_LMT_IC,
	[RTW_REGD_KCC]		= TXPWR_LMT_KCC,
	[RTW_REGD_NCC]		= TXPWR_LMT_NCC,
	[RTW_REGD_ACMA]		= TXPWR_LMT_ACMA,
	[RTW_REGD_CHILE]	= TXPWR_LMT_CHILE,
	[RTW_REGD_MEX]		= TXPWR_LMT_MEXICO,
	[RTW_REGD_WW]		= TXPWR_LMT_WW,
};

char *rtw_get_regd_inr_bmp_str(char *buf, u8 bmp)
{
	char *pos = buf;
	int i;

	for (i = 0; i < RTW_REGD_SET_BY_NUM; i++) {
		if (!(bmp & BIT(i)))
			continue;
		pos += snprintf(pos, REGD_INR_BMP_STR_LEN - (pos - buf), "%s%s"
			, pos == buf ? "" : " ", regd_inr_str(i));
		if (pos >= buf + REGD_INR_BMP_STR_LEN - 1)
			goto exit;
	}
	if (pos == buf)
		buf[0] = '\0';

exit:
	return buf;
}

#if CONFIG_IEEE80211_BAND_6GHZ
char *rtw_get_env_bmp_str(char *buf, u8 bmp)
{
	char *pos = buf;
	int i;

	for (i = 0; i < RTW_ENV_NUM; i++) {
		if (!(bmp & BIT(i)))
			continue;
		pos += snprintf(pos, ENV_BMP_STR_LEN - (pos - buf), "%s%s"
			, pos == buf ? "" : " ", env_str(i));
		if (pos >= buf + ENV_BMP_STR_LEN - 1)
			goto exit;
	}
	if (pos == buf)
		buf[0] = '\0';

exit:
	return buf;
}
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

char *rtw_get_edcca_mode_of_bands_str(char *buf, u8 mode_of_band[])
{
#define EDCCA_MODE_SEQ_COMPARE(result, operand) (result == RTW_EDCCA_MODE_NUM ? operand : (operand == RTW_EDCCA_MODE_NUM ? result : (result != operand ? -1 : result)))

	int mode = RTW_EDCCA_MODE_NUM;
	char *pos = buf;

	mode = EDCCA_MODE_SEQ_COMPARE(mode, mode_of_band[BAND_ON_24G]);
#if CONFIG_IEEE80211_BAND_5GHZ
	mode = EDCCA_MODE_SEQ_COMPARE(mode, mode_of_band[BAND_ON_5G]);
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	mode = EDCCA_MODE_SEQ_COMPARE(mode, mode_of_band[BAND_ON_6G]);
#endif

	if (mode != -1) { /* all available values are the same */
		pos += snprintf(pos, EDCCA_MODE_OF_BANDS_STR_LEN - (pos - buf), "%s(%u)", rtw_edcca_mode_str(mode), mode);
		if (pos >= buf + EDCCA_MODE_OF_BANDS_STR_LEN - 1)
			goto exit;
	} else {
		pos += snprintf(pos, EDCCA_MODE_OF_BANDS_STR_LEN - (pos - buf), "%s(%u)", rtw_edcca_mode_str(mode_of_band[BAND_ON_24G]), mode_of_band[BAND_ON_24G]);
		if (pos >= buf + EDCCA_MODE_OF_BANDS_STR_LEN - 1)
			goto exit;
		#if CONFIG_IEEE80211_BAND_5GHZ
		pos += snprintf(pos, EDCCA_MODE_OF_BANDS_STR_LEN - (pos - buf), " %s(%u)", rtw_edcca_mode_str(mode_of_band[BAND_ON_5G]), mode_of_band[BAND_ON_5G]);
		if (pos >= buf + EDCCA_MODE_OF_BANDS_STR_LEN - 1)
			goto exit;
		#endif
		#if CONFIG_IEEE80211_BAND_6GHZ
		pos += snprintf(pos, EDCCA_MODE_OF_BANDS_STR_LEN - (pos - buf), " %s(%u)", rtw_edcca_mode_str(mode_of_band[BAND_ON_6G]), mode_of_band[BAND_ON_6G]);
		if (pos >= buf + EDCCA_MODE_OF_BANDS_STR_LEN - 1)
			goto exit;
		#endif
	}

exit:
	return buf;
}

static enum rtw_edcca_mode_t rtw_edcca_mode_get_strictest(enum rtw_edcca_mode_t a, enum rtw_edcca_mode_t b)
{
	if (a >= RTW_EDCCA_MODE_NUM)
		return b < RTW_EDCCA_MODE_NUM ? b : RTW_EDCCA_MODE_NUM;
	if (b >= RTW_EDCCA_MODE_NUM)
		return a;
	return rtw_max(a,b);
}

static void rtw_edcca_mode_update_by_regd_reqs(struct dvobj_priv *dvobj, bool req_lock)
{
	struct rf_ctl_t *rfctl = dvobj_to_rfctl(dvobj);
	struct regd_req_t *req;
	struct country_chplan *chplan;
	_list *cur, *head;
	u8 mode[BAND_MAX];
	u8 band, tmp_mode;
	char buf[EDCCA_MODE_OF_BANDS_STR_LEN];

	for (band = 0; band < BAND_MAX; band++)
		mode[band] = RTW_EDCCA_MODE_NUM;

	if (req_lock)
		_rtw_mutex_lock_interruptible(&rfctl->regd_req_mutex);

	head = &rfctl->regd_req_list;
	cur = get_next(head);
	while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
		req = LIST_CONTAINOR(cur, struct regd_req_t, list);
		chplan = &req->chplan;
		cur = get_next(cur);
		if (!req->applied)
			continue;

		tmp_mode = COUNTRY_CHPLAN_EDCCA_2G_OVERRIDE(chplan) != RTW_EDCCA_DEF ? COUNTRY_CHPLAN_EDCCA_2G_OVERRIDE(chplan) :
			rtw_regd_to_edcca_mode(rtw_chplan_get_default_regd_2g(chplan->domain_code));
		mode[BAND_ON_24G] = rtw_edcca_mode_get_strictest(mode[BAND_ON_24G], tmp_mode);

		#if CONFIG_IEEE80211_BAND_5GHZ
		tmp_mode = COUNTRY_CHPLAN_EDCCA_5G_OVERRIDE(chplan) != RTW_EDCCA_DEF ? COUNTRY_CHPLAN_EDCCA_5G_OVERRIDE(chplan) :
			rtw_regd_to_edcca_mode(rtw_chplan_get_default_regd_5g(chplan->domain_code));
		mode[BAND_ON_5G] = rtw_edcca_mode_get_strictest(mode[BAND_ON_5G], tmp_mode);
		#endif

		#if CONFIG_IEEE80211_BAND_6GHZ
		tmp_mode = COUNTRY_CHPLAN_EDCCA_6G_OVERRIDE(chplan) != RTW_EDCCA_DEF ? COUNTRY_CHPLAN_EDCCA_6G_OVERRIDE(chplan) :
			rtw_regd_to_edcca_mode_6g(rtw_chplan_get_default_regd_6g(chplan->domain_code_6g));
		if (tmp_mode == RTW_EDCCA_CBP) {
			/* STD category needs no CBP */
			if (req->txpwr_lmt_6g_cate_map & BIT(TXPWR_LMT_6G_CATE_STD))
				tmp_mode = RTW_EDCCA_NORM;
		}
		mode[BAND_ON_6G] = rtw_edcca_mode_get_strictest(mode[BAND_ON_6G], tmp_mode);
		#endif
	}

	if (req_lock)
		_rtw_mutex_unlock(&rfctl->regd_req_mutex);

	rfctl->edcca_mode_2g = mode[BAND_ON_24G];
	#if CONFIG_IEEE80211_BAND_5GHZ
	rfctl->edcca_mode_5g = mode[BAND_ON_5G];
	#endif
	#if CONFIG_IEEE80211_BAND_6GHZ
	rfctl->edcca_mode_6g = mode[BAND_ON_6G];
	#endif

	RTW_PRINT("update edcca_mode:%s\n"
		, rtw_get_edcca_mode_of_bands_str(buf, mode)
	);
}

void rtw_edcca_mode_update(struct dvobj_priv *dvobj, bool req_lock)
{
	struct registry_priv *regsty = dvobj_to_regsty(dvobj);
	struct rf_ctl_t *rfctl = dvobj_to_rfctl(dvobj);

	if (regsty->edcca_mode_sel == RTW_EDCCA_NORM) {
		/* force disable */
		rfctl->edcca_mode_2g = RTW_EDCCA_NORM;
		#if CONFIG_IEEE80211_BAND_5GHZ
		rfctl->edcca_mode_5g = RTW_EDCCA_NORM;
		#endif
		#if CONFIG_IEEE80211_BAND_6GHZ
		rfctl->edcca_mode_6g = RTW_EDCCA_NORM;
		#endif

	} else if (regsty->edcca_mode_sel == RTW_EDCCA_CS) {
		/* carrier sense */
		rfctl->edcca_mode_2g = RTW_EDCCA_CS;
		#if CONFIG_IEEE80211_BAND_5GHZ
		rfctl->edcca_mode_5g = RTW_EDCCA_CS;
		#endif
		#if CONFIG_IEEE80211_BAND_6GHZ
		rfctl->edcca_mode_6g = RTW_EDCCA_CS;
		#endif

	} else if (regsty->edcca_mode_sel == RTW_EDCCA_ADAPT) {
		/* adaptivity */
		rfctl->edcca_mode_2g = RTW_EDCCA_ADAPT;
		#if CONFIG_IEEE80211_BAND_5GHZ
		rfctl->edcca_mode_5g = RTW_EDCCA_ADAPT;
		#endif
		#if CONFIG_IEEE80211_BAND_6GHZ
		rfctl->edcca_mode_6g = RTW_EDCCA_ADAPT;
		#endif

	} else if (regsty->edcca_mode_sel == RTW_EDCCA_CBP) {
		/* adaptivity */
		rfctl->edcca_mode_2g = RTW_EDCCA_NORM;
		#if CONFIG_IEEE80211_BAND_5GHZ
		rfctl->edcca_mode_5g = RTW_EDCCA_NORM;
		#endif
		#if CONFIG_IEEE80211_BAND_6GHZ
		rfctl->edcca_mode_6g = RTW_EDCCA_CBP;
		#endif

	} else {
		/* by regulatory setting */
		rtw_edcca_mode_update_by_regd_reqs(dvobj, req_lock);
	}

	rtw_edcca_hal_update(dvobj);
}

u8 rtw_get_edcca_mode(struct dvobj_priv *dvobj, enum band_type band)
{
	struct rf_ctl_t *rfctl = dvobj_to_rfctl(dvobj);
	u8 edcca_mode = RTW_EDCCA_NORM;

	if (band == BAND_ON_24G)
		edcca_mode = rfctl->edcca_mode_2g;
	#if CONFIG_IEEE80211_BAND_5GHZ
	else if (band == BAND_ON_5G)
		edcca_mode = rfctl->edcca_mode_5g;
	#endif
	#if CONFIG_IEEE80211_BAND_6GHZ
	else if (band == BAND_ON_6G)
		edcca_mode = rfctl->edcca_mode_6g;
	#endif

	return edcca_mode;
}

#if CONFIG_TXPWR_LIMIT
char *rtw_get_txpwr_lmt_name_of_bands_str(char *buf, const char *name_of_band[], u8 unknown_bmp)
{
#define NAME_DIFF ((void *)1)
/* input comes form organized database, string with same content will not have different pointer */
#define NAME_SEQ_COMPARE(result, operand) ( \
	(result) == NULL ? (operand) : \
	(operand) == NULL ? (result) : \
	(result) != (operand) ? NAME_DIFF : (result) \
	)

#define BOOL_TO_S8(b) ((b) ? 1 : 0)
#define BOOL_S_DIFF (-1)
#define BOOL_S_SEQ_COMPARE(result, operand) ((result) != BOOL_TO_S8(operand) ? BOOL_S_DIFF : (result))

	const char *name;
	s8 unknown;
	char *pos = buf;

	name = name_of_band[BAND_ON_24G];
	unknown = BOOL_TO_S8(!!(unknown_bmp & BIT(BAND_ON_24G)));
#if CONFIG_IEEE80211_BAND_5GHZ
	name = NAME_SEQ_COMPARE(name, name_of_band[BAND_ON_5G]);
	unknown = BOOL_S_SEQ_COMPARE(unknown, !!(unknown_bmp & BIT(BAND_ON_5G)));
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	name = NAME_SEQ_COMPARE(name, name_of_band[BAND_ON_6G]);
	unknown = BOOL_S_SEQ_COMPARE(unknown, !!(unknown_bmp & BIT(BAND_ON_6G)));
#endif

	if (name != NAME_DIFF && unknown != BOOL_S_DIFF) { /* all available values are the same */
		pos += snprintf(pos, TXPWR_NAME_OF_BANDS_STR_LEN - (pos - buf), "%s%s", (unknown_bmp & BIT(BAND_ON_24G)) ? "?" : "", name);
		if (pos >= buf + TXPWR_NAME_OF_BANDS_STR_LEN - 1)
			goto exit;
	} else {
		pos += snprintf(pos, TXPWR_NAME_OF_BANDS_STR_LEN - (pos - buf), "%s%s", (unknown_bmp & BIT(BAND_ON_24G)) ? "?" : "", name_of_band[BAND_ON_24G]);
		if (pos >= buf + TXPWR_NAME_OF_BANDS_STR_LEN - 1)
			goto exit;
		#if CONFIG_IEEE80211_BAND_5GHZ
		pos += snprintf(pos, TXPWR_NAME_OF_BANDS_STR_LEN - (pos - buf), " %s%s", (unknown_bmp & BIT(BAND_ON_5G)) ? "?" : "", name_of_band[BAND_ON_5G]);
		if (pos >= buf + TXPWR_NAME_OF_BANDS_STR_LEN - 1)
			goto exit;
		#endif
		#if CONFIG_IEEE80211_BAND_6GHZ
		pos += snprintf(pos, TXPWR_NAME_OF_BANDS_STR_LEN - (pos - buf), " %s%s", (unknown_bmp & BIT(BAND_ON_6G)) ? "?" : "", name_of_band[BAND_ON_6G]);
		if (pos >= buf + TXPWR_NAME_OF_BANDS_STR_LEN - 1)
			goto exit;
		#endif
	}

exit:
	return buf;
}

#if CONFIG_IEEE80211_BAND_6GHZ
static u8 regd_req_get_6g_cate_map(struct regd_req_t *req)
{
	return req->txpwr_lmt_6g_cate_map;
}
#endif

static void rtw_txpwr_apply_regd_req_reg_exc(struct rf_ctl_t *rfctl, struct regd_req_t *req
	, char *req_6g_name_buf, u8 req_6g_name_bsz
	, const char *name_of_band[], u8 *unknown_bmp)
{
	struct dvobj_priv *dvobj = rfctl_to_dvobj(rfctl);
	struct country_chplan *chplan = &req->chplan;
	enum txpwr_lmt_reg_exc_match exc = TXPWR_LMT_REG_EXC_MATCH_NONE;
	const char *name = NULL;
#if CONFIG_IEEE80211_BAND_6GHZ
	enum txpwr_lmt_reg_exc_match exc_6g = TXPWR_LMT_REG_EXC_MATCH_NONE;
	const char *name_6g = NULL;
#endif
	u8 band;

	if (*unknown_bmp & (BIT(BAND_ON_24G) | BIT(BAND_ON_5G)))
		exc = rtw_txpwr_hal_lmt_reg_exc_search(dvobj, chplan->alpha2, chplan->domain_code, &name);
#if CONFIG_IEEE80211_BAND_6GHZ
	if (*unknown_bmp & BIT(BAND_ON_6G))
		exc_6g = rtw_txpwr_hal_lmt_reg_exc_6g_search(dvobj, chplan->alpha2, chplan->domain_code_6g, &name_6g);
#endif

	if (exc
		#if CONFIG_IEEE80211_BAND_6GHZ
		|| exc_6g
		#endif
	) {
		char exc_msg[] = " country:XX domain:0xGG"
				#if CONFIG_IEEE80211_BAND_6GHZ
				" domain_6g:0xGG"
				#endif
				"\0";
		u8 exc_msg_len = sizeof(exc_msg);
		char *msg_p = exc_msg;
		char buf[TXPWR_NAME_OF_BANDS_STR_LEN];

		for (band = 0; band < BAND_MAX; band++) {
			if (!(*unknown_bmp & BIT(band)))
				continue;

			if ((band == BAND_ON_24G || band == BAND_ON_5G) && exc) {
				name_of_band[band] = name;
				if (strcmp(name, txpwr_lmt_str(TXPWR_LMT_NONE)) == 0
					|| strcmp(name, txpwr_lmt_str(TXPWR_LMT_WW)) == 0
					|| rtw_txpwr_hal_lmt_reg_search(dvobj, band, name))
					*unknown_bmp &= ~BIT(band);
			#if CONFIG_IEEE80211_BAND_6GHZ
			} else if (band == BAND_ON_6G && exc_6g) {
				if (strcmp(name_6g, txpwr_lmt_str(TXPWR_LMT_NONE)) == 0
					|| strcmp(name_6g, txpwr_lmt_str(TXPWR_LMT_WW)) == 0
				) {
					/* known limits without category */
					name_of_band[band] = name_6g;
					*unknown_bmp &= ~BIT(band);
				} else {
					u8 cate_map = regd_req_get_6g_cate_map(req);
					int i;

					name_of_band[band] = req_6g_name_buf;
					for (i = TXPWR_LMT_6G_CATE_NUM - 1; i >= TXPWR_LMT_6G_CATE_VLP ; i--) {
						if (!(cate_map & BIT(i)))
							continue;
						snprintf(req_6g_name_buf, req_6g_name_bsz, "%s_%s", name_6g, txpwr_lmt_6g_cate_str(i));
						if (rtw_txpwr_hal_lmt_reg_search(dvobj, band, req_6g_name_buf)) {
							*unknown_bmp &= ~BIT(band);
							break;
						}
					}
				}
			#endif
			}
		}

		if (exc == TXPWR_LMT_REG_EXC_MATCH_COUNTRY
			#if CONFIG_IEEE80211_BAND_6GHZ
			|| exc_6g == TXPWR_LMT_REG_EXC_MATCH_COUNTRY
			#endif
		) {
			msg_p += snprintf(msg_p, exc_msg_len - (msg_p - exc_msg), " country:"ALPHA2_FMT, ALPHA2_ARG(chplan->alpha2));
			if (msg_p >= exc_msg + exc_msg_len - 1)
				goto msg_dump;
		}
		if (exc == TXPWR_LMT_REG_EXC_MATCH_DOMAIN) {
			msg_p += snprintf(msg_p, exc_msg_len - (msg_p - exc_msg), " domain:0x%02x", chplan->domain_code);
			if (msg_p >= exc_msg + exc_msg_len - 1)
				goto msg_dump;
		}
		#if CONFIG_IEEE80211_BAND_6GHZ
		if (exc_6g == TXPWR_LMT_REG_EXC_MATCH_DOMAIN) {
			msg_p += snprintf(msg_p, exc_msg_len - (msg_p - exc_msg), " domain_6g:0x%02x", chplan->domain_code_6g);
			if (msg_p >= exc_msg + exc_msg_len - 1)
				goto msg_dump;
		}
		#endif

msg_dump:
		RTW_PRINT("exception%s applied, txpwr_lmt:%s\n", exc_msg
			, rtw_get_txpwr_lmt_name_of_bands_str(buf, name_of_band, *unknown_bmp));
	}
}

#ifdef CONFIG_REGD_SRC_FROM_OS
static void rtw_txpwr_apply_regd_req_from_os(struct rf_ctl_t *rfctl, struct regd_req_t *req
	, const char *req_alpha2_str, char *req_6g_name_buf, u8 req_6g_name_bsz
	, const char *name_of_band[], u8 *unknown_bmp)
{
	struct dvobj_priv *dvobj = rfctl_to_dvobj(rfctl);
	struct country_chplan *chplan = &req->chplan;

	if (rfctl->regd_src == REGD_SRC_OS) {
		bool alpha2_is_ww = IS_ALPHA2_WORLDWIDE(req_alpha2_str);
		const char *name = NULL;
		char buf[TXPWR_NAME_OF_BANDS_STR_LEN];
		u8 band;

		name = alpha2_is_ww ? txpwr_lmt_str(TXPWR_LMT_WW) : req_alpha2_str;

		for (band = 0; band < BAND_MAX; band++) {
			if (!(*unknown_bmp & BIT(band)))
				continue;

			if (alpha2_is_ww) {
				name_of_band[band] = name;
				*unknown_bmp &= ~BIT(band);
				continue;
			}

			#if CONFIG_IEEE80211_BAND_6GHZ
			if (band == BAND_ON_6G) {
				u8 cate_map = regd_req_get_6g_cate_map(req);
				int i;

				name_of_band[band] = req_6g_name_buf;
				for (i = TXPWR_LMT_6G_CATE_NUM - 1; i >= TXPWR_LMT_6G_CATE_VLP ; i--) {
					if (!(cate_map & BIT(i)))
						continue;
					snprintf(req_6g_name_buf, req_6g_name_bsz, "%s_%s", name, txpwr_lmt_6g_cate_str(i));
					if (rtw_txpwr_hal_lmt_reg_search(dvobj, band, req_6g_name_buf)) {
						*unknown_bmp &= ~BIT(band);
						break;
					}
				}
			} else
			#endif
			{
				name_of_band[band] = name;
				if (rtw_txpwr_hal_lmt_reg_search(dvobj, band, name_of_band[band]))
					*unknown_bmp &= ~BIT(band);
			}
		}

		RTW_PRINT("os country:"ALPHA2_FMT" applied, txpwr_lmt:%s\n"
			, ALPHA2_ARG(req_alpha2_str), rtw_get_txpwr_lmt_name_of_bands_str(buf, name_of_band, *unknown_bmp));

		if (*unknown_bmp && chplan->domain_code == RTW_CHPLAN_UNSPECIFIED) {
			for (band = 0; band < BAND_MAX; band++) {
				if (!(*unknown_bmp & BIT(band)))
					continue;
				name_of_band[band] = txpwr_lmt_str(TXPWR_LMT_WW);
				*unknown_bmp &= ~BIT(band);
			}

			RTW_PRINT("unsupported os country:"ALPHA2_FMT" applied, txpwr_lmt:%s\n"
				, ALPHA2_ARG(req_alpha2_str), rtw_get_txpwr_lmt_name_of_bands_str(buf, name_of_band, *unknown_bmp));
		}
	}
}
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
static bool rtw_txpwr_search_predef_6g_regd(struct rf_ctl_t *rfctl, struct regd_req_t *req
	, u8 txpwr_lmt, const char **name)
{
	struct dvobj_priv *dvobj = rfctl_to_dvobj(rfctl);
	u8 cate_map = regd_req_get_6g_cate_map(req);
	int i;

	for (i = TXPWR_LMT_6G_CATE_NUM - 1; i >= TXPWR_LMT_6G_CATE_VLP ; i--) {
		if (!(cate_map & BIT(i)))
			continue;
		*name = txpwr_lmt_6g_str(txpwr_lmt, i);
		if (!*name)
			continue;
		if (rtw_txpwr_hal_lmt_reg_search(dvobj, BAND_ON_6G, *name))
			return true;
	}

	return false;
}
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

static void rtw_txpwr_apply_predefined(struct rf_ctl_t *rfctl, struct regd_req_t *req, enum band_type band, u8 txpwr_lmt
	, const char *name_of_band[], u8 *unknown_bmp)
{
	struct dvobj_priv *dvobj = rfctl_to_dvobj(rfctl);

	if (txpwr_lmt == TXPWR_LMT_NONE || txpwr_lmt == TXPWR_LMT_WW || txpwr_lmt == TXPWR_LMT_NUM) {
		name_of_band[band] = txpwr_lmt_str(txpwr_lmt);
		*unknown_bmp &= ~BIT(band);
	} else if (band == BAND_ON_24G || band == BAND_ON_5G) {
		name_of_band[band] = txpwr_lmt_str(txpwr_lmt);
		if (name_of_band[band] && rtw_txpwr_hal_lmt_reg_search(dvobj, band, name_of_band[band]))
			*unknown_bmp &= ~BIT(band);
	#if CONFIG_IEEE80211_BAND_6GHZ
	} else if (band == BAND_ON_6G) {
		if (rtw_txpwr_search_predef_6g_regd(rfctl, req, txpwr_lmt, &name_of_band[band]))
			*unknown_bmp &= ~BIT(band);
	#endif
	}
}

static void rtw_txpwr_apply_regd_req_default(struct rf_ctl_t *rfctl, struct regd_req_t *req
	, const char *name_of_band[], u8 *unknown_bmp)
{
	struct country_chplan *chplan = &req->chplan;
	u8 txpwr_lmt[BAND_MAX];
	char buf[TXPWR_NAME_OF_BANDS_STR_LEN];
	u8 band;
	bool altenate_applied = 0;

	for (band = 0; band < BAND_MAX; band++)
		txpwr_lmt[band] = TXPWR_LMT_NONE;

	if (chplan->txpwr_lmt_override != TXPWR_LMT_DEF) {
		for (band = 0; band < BAND_MAX; band++) {
			if (!(*unknown_bmp & BIT(band)))
				continue;
			txpwr_lmt[band] = chplan->txpwr_lmt_override;
			rtw_txpwr_apply_predefined(rfctl, req, band, txpwr_lmt[band], name_of_band, unknown_bmp);
		}
		RTW_PRINT("default country:"ALPHA2_FMT" applied, txpwr_lmt:%s\n"
			, ALPHA2_ARG(chplan->alpha2), rtw_get_txpwr_lmt_name_of_bands_str(buf, name_of_band, *unknown_bmp));

	} else {
		if (*unknown_bmp & (BIT(BAND_ON_24G) | BIT(BAND_ON_5G))) {
			txpwr_lmt[BAND_ON_24G] = rtw_regd_to_txpwr_lmt(rtw_chplan_get_default_regd_2g(chplan->domain_code));
			#if CONFIG_IEEE80211_BAND_5GHZ
			txpwr_lmt[BAND_ON_5G] = rtw_regd_to_txpwr_lmt(rtw_chplan_get_default_regd_5g(chplan->domain_code));
			#endif
			for (band = 0; band < BAND_MAX; band++) {
				if (!(*unknown_bmp & BIT(band)) || (band != BAND_ON_24G && band != BAND_ON_5G))
					continue;
				rtw_txpwr_apply_predefined(rfctl, req, band, txpwr_lmt[band], name_of_band, unknown_bmp);
			}
			RTW_PRINT("default domain:0x%02x applied, txpwr_lmt:%s\n"
				, chplan->domain_code, rtw_get_txpwr_lmt_name_of_bands_str(buf, name_of_band, *unknown_bmp));
		}

		#if CONFIG_IEEE80211_BAND_6GHZ
		if (*unknown_bmp & BIT(BAND_ON_6G)) {
			txpwr_lmt[BAND_ON_6G] = rtw_regd_to_txpwr_lmt(rtw_chplan_get_default_regd_6g(chplan->domain_code_6g));
			rtw_txpwr_apply_predefined(rfctl, req, BAND_ON_6G, txpwr_lmt[BAND_ON_6G], name_of_band, unknown_bmp);
			RTW_PRINT("default domain_6g:0x%02x applied, txpwr_lmt:%s\n"
				, chplan->domain_code_6g, rtw_get_txpwr_lmt_name_of_bands_str(buf, name_of_band, *unknown_bmp));
		}
		#endif
	}

	if (*unknown_bmp == 0)
		return;

	for (band = 0; band < BAND_MAX; band++) {
		if (!(*unknown_bmp & BIT(band)))
			continue;
		if (TXPWR_LMT_ALTERNATE_DEFINED(txpwr_lmt[band])) {
			/*
			* To support older chips without new predefined txpwr_lmt:
			* - use txpwr_lmt_alternate() to get alternate if the selection is  not found
			*/
			altenate_applied = 1;
			txpwr_lmt[band] = txpwr_lmt_alternate(txpwr_lmt[band]);
			rtw_txpwr_apply_predefined(rfctl, req, band, txpwr_lmt[band], name_of_band, unknown_bmp);
		}
	}
	if (altenate_applied) {
		RTW_PRINT("alternate applied, txpwr_lmt:%s\n"
			, rtw_get_txpwr_lmt_name_of_bands_str(buf, name_of_band, *unknown_bmp));
		if (*unknown_bmp == 0)
			return;
	}

	for (band = 0; band < BAND_MAX; band++) {
		if (!(*unknown_bmp & BIT(band)))
			continue;
		txpwr_lmt[band] = TXPWR_LMT_WW;
		name_of_band[band] = txpwr_lmt_str(txpwr_lmt[band]);
		*unknown_bmp &= ~BIT(band);
	}
	RTW_PRINT("world wide applied, txpwr_lmt:%s\n"
		, rtw_get_txpwr_lmt_name_of_bands_str(buf, name_of_band, *unknown_bmp));
}

static void rtw_txpwr_apply_regd_req(struct rf_ctl_t *rfctl, struct regd_req_t *req
	, char *names_of_band[], int names_of_band_len[])
{
#if CONFIG_IEEE80211_BAND_6GHZ
#define REG_6G_NAME_BSZ (16 + 4) /* ex: reg_name(up to  16 char)  + "_VLP" */
#else
#define REG_6G_NAME_BSZ 0
#endif
	struct rtw_chset *chset = &rfctl->chset;
#ifdef CONFIG_REGD_SRC_FROM_OS
	struct country_chplan *chplan = &req->chplan;
	char req_alpha2_str[3] = {chplan->alpha2[0], chplan->alpha2[1], 0};
#endif
	char req_6g_name_buf[REG_6G_NAME_BSZ];
	const char *name_of_band[BAND_MAX];
	u8 unknown_bmp; /* unknown bitmap of name_of_band */
	u8 band;

	for (band = 0; band < BAND_MAX; band++)
		name_of_band[band] = NULL;

	unknown_bmp = 0
		| (chset->chs_of_band[BAND_ON_24G] ? BIT(BAND_ON_24G) : 0)
		#if CONFIG_IEEE80211_BAND_5GHZ
		| (chset->chs_of_band[BAND_ON_5G] ? BIT(BAND_ON_5G) : 0)
		#endif
		#if CONFIG_IEEE80211_BAND_6GHZ
		| (chset->chs_of_band[BAND_ON_6G] ? BIT(BAND_ON_6G) : 0)
		#endif
		;

	/* search from exception mapping */
	rtw_txpwr_apply_regd_req_reg_exc(rfctl, req
		, req_6g_name_buf, REG_6G_NAME_BSZ, name_of_band, &unknown_bmp);
	if (!unknown_bmp)
		goto exit;

#ifdef CONFIG_REGD_SRC_FROM_OS
	rtw_txpwr_apply_regd_req_from_os(rfctl, req, req_alpha2_str
		, req_6g_name_buf, REG_6G_NAME_BSZ, name_of_band, &unknown_bmp);
	if (!unknown_bmp)
		goto exit;
#endif

	/* follow default channel plan mapping */
	rtw_txpwr_apply_regd_req_default(rfctl, req, name_of_band, &unknown_bmp);

exit:
	for (band = 0; band < BAND_MAX; band++)
		ustrs_add(&names_of_band[band], &names_of_band_len[band], name_of_band[band]);
}

void rtw_txpwr_update_cur_lmt_regs(struct dvobj_priv *dvobj, bool req_lock)
{
	struct rf_ctl_t *rfctl = dvobj_to_rfctl(dvobj);
	struct regd_req_t *req;
	_list *cur, *head;
	char *names[BAND_MAX];
	int names_len[BAND_MAX];
	u8 band;

	_rtw_memset(names, 0, sizeof(names));
	_rtw_memset(names_len, 0, sizeof(names_len));

	if (req_lock)
		_rtw_mutex_lock_interruptible(&rfctl->regd_req_mutex);

	head = &rfctl->regd_req_list;
	cur = get_next(head);
	while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
		req = LIST_CONTAINOR(cur, struct regd_req_t, list);
		cur = get_next(cur);
		if (!req->applied)
			continue;
		rtw_txpwr_apply_regd_req(rfctl, req, names, names_len);
	}

	if (req_lock)
		_rtw_mutex_unlock(&rfctl->regd_req_mutex);

	/* set to tx power limit regulations to HAL */
	rtw_txpwr_hal_set_current_lmt_regs_by_name(dvobj, names, names_len);

	for (band = 0; band < BAND_MAX; band++)
		if (names[band] && names_len[band])
			rtw_mfree(names[band], names_len[band]);
}
#endif /* CONFIG_TXPWR_LIMIT */

static const struct country_chplan world_wide_chplan =
	COUNTRY_CHPLAN_ENT(WORLDWIDE_ALPHA2, RTW_CHPLAN_WORLDWIDE, RTW_CHPLAN_6G_WORLDWIDE, DEF, 1, 1, 1, 1, ___);

/*
* rtw_get_chplan_worldwide -
* @ent: the buf to copy country_chplan entry content
*/
void rtw_get_chplan_worldwide(struct country_chplan *ent)
{
	_rtw_memcpy(ent, &world_wide_chplan, sizeof(*ent));
}

struct alpha2_alternate_t {
	char alpha2[2];
	char alt[2];
};

#define ALPHA2_ALT_ENT(_alpha2, _alt) {.alpha2 = _alpha2, .alt = _alt}

static const struct alpha2_alternate_t alpha2_alt_map[] = {
	ALPHA2_ALT_ENT("EU", "DE"),
};

static bool rtw_get_chplan_from_alpha2_alt(const char *code, struct country_chplan *ent)
{
	int i;
	bool ret = false;

	for (i = 0; i < ARRAY_SIZE(alpha2_alt_map); i++) {
		if (code[0] == alpha2_alt_map[i].alpha2[0]
			&& code[1] == alpha2_alt_map[i].alpha2[1]
		) {
			ret = regdb_ops.get_chplan_from_alpha2(alpha2_alt_map[i].alt, ent);
			if (ret)
				break;
		}
	}

	return ret;
}

/*
* rtw_get_chplan_from_country -
* @country_code: string of country code
* @ent: the buf to copy country_chplan entry content
*
* Return _TRUE or _FALSE when unsupported country_code is given
*/
bool rtw_get_chplan_from_country(const char *country_code, struct country_chplan *ent)
{
	char code[2];
	bool ret;

	if (!is_alpha(country_code[0]) || !is_alpha(country_code[1]))
		return false;

	code[0] = alpha_to_upper(country_code[0]);
	code[1] = alpha_to_upper(country_code[1]);

	ret = regdb_ops.get_chplan_from_alpha2(code, ent);
	if (!ret)
		ret = rtw_get_chplan_from_alpha2_alt(code, ent);

	return ret;
}

void rtw_chplan_ioctl_input_mapping(u16 *chplan, u16 *chplan_6g)
{
	if (chplan) {
		if (*chplan == RTW_CHPLAN_IOCTL_UNSPECIFIED)
			*chplan = RTW_CHPLAN_UNSPECIFIED;
		else if (*chplan == RTW_CHPLAN_IOCTL_NULL)
			*chplan = RTW_CHPLAN_NULL;
	}

	if (chplan_6g) {
		if (*chplan_6g == RTW_CHPLAN_IOCTL_UNSPECIFIED)
			*chplan_6g = RTW_CHPLAN_6G_UNSPECIFIED;
		else if (*chplan_6g == RTW_CHPLAN_IOCTL_NULL)
			*chplan_6g = RTW_CHPLAN_6G_NULL;
	}
}

bool rtw_chplan_ids_is_world_wide(u8 chplan, u8 chplan_6g)
{
	return !(chplan == RTW_CHPLAN_NULL
				#if CONFIG_IEEE80211_BAND_6GHZ
				&& chplan_6g == RTW_CHPLAN_6G_NULL
				#endif
			)
			&& (chplan == RTW_CHPLAN_WORLDWIDE || chplan == RTW_CHPLAN_NULL)
			#if CONFIG_IEEE80211_BAND_6GHZ
			&& (chplan_6g == RTW_CHPLAN_6G_WORLDWIDE || chplan_6g == RTW_CHPLAN_6G_NULL)
			#endif
			;
}

/*
 * Check if the @param ch, bw, offset is valid for the given @param ent
 * @ent: the given country chplan ent
 * @band: the given band
 * @ch: the given channel number
 * @bw: the given bandwidth
 * @offset: the given channel offset
 * @rfctl: refer to addtional ch enable/disable configurations
 *
 * return valid (1) or not (0)
 */
u8 rtw_country_chplan_is_bchbw_valid(struct country_chplan *ent, enum band_type band, u8 ch, u8 bw, u8 offset
	, bool allow_primary_passive, bool allow_passive, struct rf_ctl_t *rfctl)
{
	u8 domain_code_6g = RTW_CHPLAN_6G_NULL;
	u8 valid = 0;

	if (band == BAND_ON_5G && !COUNTRY_CHPLAN_EN_11A(ent))
		goto exit;

	if (bw >= CHANNEL_WIDTH_80 && !COUNTRY_CHPLAN_EN_11AC(ent))
		goto exit;

	#if CONFIG_IEEE80211_BAND_6GHZ
	domain_code_6g = ent->domain_code_6g;
	#endif

	valid = rtw_chplan_is_bchbw_valid(ent->domain_code, domain_code_6g, band, ch, bw, offset
		, allow_primary_passive, allow_passive, rfctl);

exit:
	return valid;
}

static void rtw_country_chplan_get_edcca_mode_of_bands(const struct country_chplan *ent, u8 mode_of_band[])
{
	mode_of_band[BAND_ON_24G] =
		COUNTRY_CHPLAN_EDCCA_2G_OVERRIDE(ent) != RTW_EDCCA_DEF ? COUNTRY_CHPLAN_EDCCA_2G_OVERRIDE(ent) :
		rtw_regd_to_edcca_mode(rtw_chplan_get_default_regd_2g(ent->domain_code));
	#if CONFIG_IEEE80211_BAND_5GHZ
	mode_of_band[BAND_ON_5G] =
		COUNTRY_CHPLAN_EDCCA_5G_OVERRIDE(ent) != RTW_EDCCA_DEF ? COUNTRY_CHPLAN_EDCCA_5G_OVERRIDE(ent) :
		rtw_regd_to_edcca_mode(rtw_chplan_get_default_regd_5g(ent->domain_code));
	#endif
	#if CONFIG_IEEE80211_BAND_6GHZ
	mode_of_band[BAND_ON_6G] =
		COUNTRY_CHPLAN_EDCCA_6G_OVERRIDE(ent) != RTW_EDCCA_DEF ? COUNTRY_CHPLAN_EDCCA_6G_OVERRIDE(ent) :
		rtw_regd_to_edcca_mode_6g(rtw_chplan_get_default_regd_6g(ent->domain_code_6g));
	#endif
}

static void rtw_country_chplan_get_txpwr_lmt_of_bands(const struct country_chplan *ent, u8 txpwr_lmt_of_band[])
{
	txpwr_lmt_of_band[BAND_ON_24G] =
		ent->txpwr_lmt_override != TXPWR_LMT_DEF ? ent->txpwr_lmt_override :
		rtw_regd_to_txpwr_lmt(rtw_chplan_get_default_regd_2g(ent->domain_code));
	#if CONFIG_IEEE80211_BAND_5GHZ
	txpwr_lmt_of_band[BAND_ON_5G] =
		ent->txpwr_lmt_override != TXPWR_LMT_DEF ? ent->txpwr_lmt_override :
		rtw_regd_to_txpwr_lmt(rtw_chplan_get_default_regd_5g(ent->domain_code));
	#endif
	#if CONFIG_IEEE80211_BAND_6GHZ
	txpwr_lmt_of_band[BAND_ON_6G] =
		ent->txpwr_lmt_override != TXPWR_LMT_DEF ? ent->txpwr_lmt_override :
		rtw_regd_to_txpwr_lmt(rtw_chplan_get_default_regd_6g(ent->domain_code_6g));
	#endif
}

#ifdef CONFIG_80211D
static const char *const _cis_status_str[] = {
	[COUNTRY_IE_SLAVE_NOCOUNTRY]	= "NOCOUNTRY",
	[COUNTRY_IE_SLAVE_APPLICABLE]	= "APPLICABLE",
	[COUNTRY_IE_SLAVE_UNKNOWN]	= "UNKNOWN",
	[COUNTRY_IE_SLAVE_OPCH_NOEXIST]	= "OPCH_NOEXIST",
	[COUNTRY_IE_SLAVE_CATE_6G_NS]	= "CATE_6G_NS",
	[COUNTRY_IE_SLAVE_TPE_FORBID]	= "TPE_FORBID",
	[COUNTRY_IE_SLAVE_STATUS_NUM]	= "INVALID",
};

#define cis_status_str(s) (((s) >= COUNTRY_IE_SLAVE_STATUS_NUM) ? _cis_status_str[COUNTRY_IE_SLAVE_STATUS_NUM] : _cis_status_str[s])

const char _rtw_env_char[] = {
	[RTW_ENV_ANY]		= ' ',
	[RTW_ENV_INDOOR]	= 'I',
	[RTW_ENV_OUTDOOR]	= 'O',
};

#if CONFIG_IEEE80211_BAND_6GHZ
static const char *const _cis_6g_reg_info_str[] = {
	[CIS_6G_REG_IN_AP]	= "IN_AP",
	[CIS_6G_REG_SP_AP]	= "SP_AP",
	[CIS_6G_REG_VLP_AP]	= "VLP_AP",
	[CIS_6G_REG_IN_EN_AP]	= "IN_EN_AP",
	[CIS_6G_REG_IN_SP_AP]	= "IN_SP_AP",
	[CIS_6G_REG_NUM]	= "RSVD",
};

#define cis_6g_reg_info_str(r) (((r) >= CIS_6G_REG_NUM) ? _cis_6g_reg_info_str[CIS_6G_REG_NUM] : _cis_6g_reg_info_str[r])

static const enum rtw_env_t _reg_info_to_env[] = {
	[CIS_6G_REG_IN_AP]	= RTW_ENV_INDOOR,
	[CIS_6G_REG_SP_AP]	= RTW_ENV_OUTDOOR,
	[CIS_6G_REG_VLP_AP]	= RTW_ENV_ANY,
	[CIS_6G_REG_IN_EN_AP]	= RTW_ENV_INDOOR,
	[CIS_6G_REG_IN_SP_AP]	= RTW_ENV_INDOOR,
	[CIS_6G_REG_NUM]	= RTW_ENV_ANY,
};

#define reg_info_to_env(reg_info) (((reg_info) >= CIS_6G_REG_NUM) ? _reg_info_to_env[CIS_6G_REG_NUM] : _reg_info_to_env[(reg_info)])

static const char *_tpe_max_txpwr_intp_str[] = {
	[TPE_ELE_MAX_TXPWR_INTP_L_EIRP]		= "L_EIRP",
	[TPE_ELE_MAX_TXPWR_INTP_L_EIRP_PSD]	= "L_EIRP_PSD",
	[TPE_ELE_MAX_TXPWR_INTP_RC_EIRP]	= "RC_EIPR",
	[TPE_ELE_MAX_TXPWR_INTP_RC_EIRP_PSD]	= "RC_EIRP_PSD",
	[TPE_ELE_MAX_TXPWR_INTP_NUM]		= "RSVD",
};

#define tpe_max_txpwr_intp_str(intp) (intp) >= TPE_ELE_MAX_TXPWR_INTP_NUM ? _tpe_max_txpwr_intp_str[TPE_ELE_MAX_TXPWR_INTP_NUM] : _tpe_max_txpwr_intp_str[intp]

static const char *_tpe_max_txpwr_cate_str[] = {
	[TPE_ELE_MAX_TXPWR_CATE_DEF]	= "DEF",
	[TPE_ELE_MAX_TXPWR_CATE_SUB]	= "SUB",
	[TPE_ELE_MAX_TXPWR_CATE_NUM]	= "RSVD",
};

#define tpe_max_txpwr_cate_str(cate) (cate) >= TPE_ELE_MAX_TXPWR_CATE_NUM ? _tpe_max_txpwr_cate_str[TPE_ELE_MAX_TXPWR_CATE_NUM] : _tpe_max_txpwr_cate_str[cate]

static bool rtw_rfctl_cisr_has_valid_link_6g_cate(const struct rf_ctl_t *rfctl, const struct country_ie_slave_record *cisr)
{
#if CONFIG_80211D_ENV_BSS_MAJORITY
	struct cis_scan_stat_ent *m;
#endif

	if (cisr->band != BAND_ON_6G
		|| cisr->status == COUNTRY_IE_SLAVE_NOCOUNTRY)
		return false;
#if CONFIG_80211D_ENV_BSS_MAJORITY
	m = RFCTL_GET_CIS_MAJORITY(rfctl);
	if (m) {
		char alpha2[2] = {
			cisr->country_str[0],
			cisr->country_str[1],
		};

		/* skip country not same as majority */
		if (m->cisr.chplan.alpha2[0] != alpha2[0]
			|| m->cisr.chplan.alpha2[1] != alpha2[1])
			return false;
	}
#endif
	if (!rfctl->cis_enabled) {
		char alpha2[2] = {
			cisr->country_str[0],
			cisr->country_str[1],
		};

		/* skip country not same as INIT/USER */
		if (rfctl->user_req) {
			if (rfctl->user_req->chplan.alpha2[0] != alpha2[0]
				|| rfctl->user_req->chplan.alpha2[1] != alpha2[1])
				return false;
		} else {
			if (rfctl->init_req.chplan.alpha2[0] != alpha2[0]
				|| rfctl->init_req.chplan.alpha2[1] != alpha2[1])
				return false;
		}
	}
	return true;
}

static void max_txpwr_get_dbm_str(s8 idx, char dbm_str[], u8 dbm_str_len)
{
	if (idx > -2 && idx < 0) /* -0.5 */
		snprintf(dbm_str, dbm_str_len, " -0.5");
	else if (idx % 2) /* d.5 */
		snprintf(dbm_str, dbm_str_len, "%3d.5", idx / 2);
	else /* d */
		snprintf(dbm_str, dbm_str_len, "%5d", idx / 2);
}

static bool dump_tpes(void *sel, u8 indent, bool fl_indent, struct rtw_tpe_t *tpes)
{
	u8 i, j;
	char dbm_str[6]; /* "-xx.x" is longest str */

	for (i = 0; i < tpes->cnt; i++) {
		if (i > 0 || fl_indent) {
			for (j = 0; j < indent; j++)
				_RTW_PRINT_SEL(sel, " ");
		}
		_RTW_PRINT_SEL(sel, " %-11s %-4s"
			, tpe_max_txpwr_intp_str(tpes->ele[i].intp)
			, tpe_max_txpwr_cate_str(tpes->ele[i].cate));
		for (j = 0; j < tpes->ele[i].pwr_cnt; j++) {
			max_txpwr_get_dbm_str(tpes->ele[i].max_tx_pwr[j], dbm_str, ARRAY_SIZE(dbm_str));
			_RTW_PRINT_SEL(sel, " %s", dbm_str);
		}
		_RTW_PRINT_SEL(sel, "\n");
	}

	return i > 0;
}
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

static void cis_scan_stat_clr(struct cis_scan_stat_t *stat, bool lock)
{
	_list *list, *head;
	struct cis_scan_stat_ent *ent;

	if (lock)
		_rtw_mutex_lock(&stat->lock);

	head = &stat->ent;
	list = get_next(head);
	while (!rtw_end_of_queue_search(head, list)) {
		ent = LIST_CONTAINOR(list, struct cis_scan_stat_ent, list);
		list = get_next(list);
		rtw_list_delete(&ent->list);
		rtw_mfree(ent, sizeof(*ent));
	}

	stat->ent_num = 0;
	CIS_SCAN_STAT_SET_MAJORITY(stat, NULL);

	if (lock)
		_rtw_mutex_unlock(&stat->lock);
}

static void cis_scan_stat_add(struct cis_scan_stat_t *stat, const struct country_ie_slave_record *cisr)
{
	_list *list, *head;
	struct cis_scan_stat_ent *ent, *inc;

	head = &stat->ent;
	list = get_next(head);
	while (!rtw_end_of_queue_search(head, list)) {
		ent = LIST_CONTAINOR(list, struct cis_scan_stat_ent, list);

		if (ent->cisr.country_str[0] == cisr->country_str[0] && ent->cisr.country_str[1] == cisr->country_str[1]) {
			ent->count++;
			inc = ent;
			goto check_order;
		}

		list = get_next(list);
	}

	/* not match, add new ent and insert tail */
	ent = rtw_malloc(sizeof(*ent));
	if (!ent)
		return;

	_rtw_memcpy(&ent->cisr, cisr, sizeof(ent->cisr));
	ent->count = 1;

	rtw_list_insert_tail(&ent->list, head);
	stat->ent_num++;
	return;

check_order:
	/* keep decending by count */

	/* search ent with count >= inc's (or search to head) */
	list = get_prev(list);
	while (!rtw_end_of_queue_search(head, list)) {
		ent = LIST_CONTAINOR(list, struct cis_scan_stat_ent, list);
		if (ent->count >= inc->count)
			break;
		list = get_prev(list);
	}

	if (list != get_prev(&inc->list)) {
		rtw_list_delete(&inc->list);
		rtw_list_insert_head(&inc->list, list);
	}
}

#if CONFIG_80211D_ENV_BSS_MAJORITY
static void cis_scan_stat_update_majority(struct cis_scan_stat_t *stat)
{
	_list *list, *head;
	struct cis_scan_stat_ent *ent, *m = NULL;

	if (stat->ent_num == 0)
		goto update;

	head = &stat->ent;
	list = get_next(head);
	m = LIST_CONTAINOR(list, struct cis_scan_stat_ent, list);

	/* check if not single majority */
	list = get_next(list);
	if (!rtw_end_of_queue_search(head, list)) {
		ent = LIST_CONTAINOR(list, struct cis_scan_stat_ent, list);
		if (m->count == ent->count)
			m = NULL;
	}

update:
	CIS_SCAN_STAT_SET_MAJORITY(stat, m);
}
#endif

static void cis_scan_stat_init(struct cis_scan_stat_t *stat)
{
	_rtw_init_listhead(&stat->ent);
	_rtw_mutex_init(&stat->lock);
	stat->ent_num = 0;
	CIS_SCAN_STAT_SET_MAJORITY(stat, NULL);
}

static void cis_scan_stat_deinit(struct cis_scan_stat_t *stat)
{
	cis_scan_stat_clr(stat, true);
	_rtw_mutex_free(&stat->lock);
}

static void dump_cis_scan_stat(void *sel, struct rf_ctl_t *rfctl)
{
	struct cis_scan_stat_t *stat = &rfctl->cis_scan_stat;
	_list *list, *head;
	struct cis_scan_stat_ent *ent;

	RTW_PRINT_SEL(sel, "ENV_BSS (updated %u sec ago)\n"
		, rtw_get_passing_time_ms(rfctl->cis_scan_last_complete_time) / 1000);

	_rtw_mutex_lock(&stat->lock);

	head = &stat->ent;
	list = get_next(head);
	while (!rtw_end_of_queue_search(head, list)) {
		ent = LIST_CONTAINOR(list, struct cis_scan_stat_ent, list);
		list = get_next(list);
		RTW_PRINT_SEL(sel, "%c"ALPHA2_FMT" %u\n"
			, RFCTL_GET_CIS_MAJORITY(rfctl) == ent ? '*' : ' '
			, ALPHA2_ARG(ent->cisr.country_str), ent->count);
	}

	_rtw_mutex_unlock(&stat->lock);
}

void dump_country_ie_slave_records(void *sel, struct rf_ctl_t *rfctl, bool skip_noset, bool lock)
{
#define CISR_TITLE_FMT "%-6s %-4s %-4s"
#define CISR_TITLE_ARG , "alpha2", "band", "opch"
#define CISR_VALUE_FMT "     "ALPHA2_FMT" %4s %4u"
#define CISR_VALUE_ARG , ALPHA2_ARG(cisr->country_str), band_str(cisr->band), cisr->opch
#define CISR_INDENT 16
#if CONFIG_IEEE80211_BAND_6GHZ
#define CISR_TITLE_FMT_6G " %-3s %-8s %-7s"
#define CISR_TITLE_ARG_6G , "env", "6g_reg_i", "6g_cate"
#define CISR_VALUE_FMT_6G " %3c %8s %7s"
#define CISR_VALUE_ARG_6G , rtw_env_char(cisr->env), cisr->band == BAND_ON_6G ? cis_6g_reg_info_str(cisr->reg_info) : "" \
	, rtw_rfctl_cisr_has_valid_link_6g_cate(rfctl, cisr) ? chplan_6g_cate_str(cisr_get_largest_chplan_6g_cate(cisr)) : "-"
#define CISR_INDENT_6G 21
#else
#define CISR_TITLE_FMT_6G ""
#define CISR_TITLE_ARG_6G
#define CISR_VALUE_FMT_6G ""
#define CISR_VALUE_ARG_6G
#define CISR_INDENT_6G 0
#endif
#define CISR_TITLE_FMT_STATUS " %-12s"
#define CISR_VALUE_FMT_STATUS " %-12s"
#define CISR_TITLE_ARG_STATUS , "status"
#define CISR_VALUE_ARG_STATUS , cis_status_str(cisr->status)
#define CISR_INDENT_STATUS 13
#if CONFIG_IEEE80211_BAND_6GHZ
#define CISR_TITLE_FMT_TPES " %-4s"
#define CISR_TITLE_ARG_TPES , "tpes"
#else
#define CISR_TITLE_FMT_TPES ""
#define CISR_TITLE_ARG_TPES
#endif

	struct country_ie_slave_record *cisr;
	int i, j;

	RTW_PRINT_SEL(sel,
		"    "
		CISR_TITLE_FMT
		CISR_TITLE_FMT_6G
		CISR_TITLE_FMT_STATUS
		CISR_TITLE_FMT_TPES
		"\n"
		CISR_TITLE_ARG
		CISR_TITLE_ARG_6G
		CISR_TITLE_ARG_STATUS
		CISR_TITLE_ARG_TPES
	);

	if (lock)
		_rtw_mutex_lock_interruptible(&rfctl->regd_req_mutex);

	for (i = 0; i < CONFIG_IFACE_NUMBER; i++) {
		for (j = 0; j < RTW_RLINK_MAX; j++) {
			cisr = &rfctl->cisr[i][j];
			if (skip_noset && is_all_null(cisr->country_str, 3))
				continue;

			RTW_PRINT_SEL(sel,
				"%d %d"
				CISR_VALUE_FMT
				CISR_VALUE_FMT_6G
				CISR_VALUE_FMT_STATUS
				, i, j
				CISR_VALUE_ARG
				CISR_VALUE_ARG_6G
				CISR_VALUE_ARG_STATUS
			);

			#if CONFIG_IEEE80211_BAND_6GHZ
			if (!dump_tpes(sel, 4 + CISR_INDENT + CISR_INDENT_6G + CISR_INDENT_STATUS, false, &cisr->tpes))
			#endif
				_RTW_PRINT_SEL(sel, "\n");

		}
	}

#if CONFIG_IEEE80211_BAND_6GHZ
	RTW_PRINT_SEL(sel, "\n");
	RTW_PRINT_SEL(sel, "link_6g_cate_map_applied_num:%u\n", rfctl->txpwr_lmt_link_6g_cate_map_applied_num);
	RTW_PRINT_SEL(sel, "link_6g_cate_map:");
	for (i = TXPWR_LMT_6G_CATE_NUM - 1; i >= TXPWR_LMT_6G_CATE_VLP ; i--) {
		if (!(rfctl->txpwr_lmt_link_6g_cate_map & BIT(i)))
			continue;
		_RTW_PRINT_SEL(sel, "%s ", txpwr_lmt_6g_cate_str(i));
	}
	_RTW_PRINT_SEL(sel, "\n");
#endif

	if (rfctl->cis_flags & CISF_ENV_BSS) {
		RTW_PRINT_SEL(sel, "\n");
		dump_cis_scan_stat(sel, rfctl);
	}

	if (lock)
		_rtw_mutex_unlock(&rfctl->regd_req_mutex);
}

static void rtw_ies_get_country_str(const u8 *ies, uint ies_len, u8 *country_str)
{
	const u8 *country_ie;
	sint country_ie_len;

	country_ie = rtw_get_ie(ies, WLAN_EID_COUNTRY, &country_ie_len, ies_len);
	if (country_ie && country_ie_len >= 3) {
		country_str[0] = country_ie[2];
		country_str[1] = country_ie[3];
		country_str[2] = country_ie[4];
		return;
	}

	country_str[0] = 0;
	country_str[1] = 0;
	country_str[2] = 0;
}

#if CONFIG_IEEE80211_BAND_6GHZ
static enum rtw_env_t country_str_get_env(const char *country_str)
{
	if (country_str) {
		switch (country_str[2]) {
		case 'I':
			return RTW_ENV_INDOOR;
		case 'O':
			return RTW_ENV_OUTDOOR;
		case ' ':
		default:
			return RTW_ENV_ANY;
		}
	}
	return RTW_ENV_ANY;
}

static enum country_ie_slave_6g_reg_info rtw_ies_get_6g_reg_info(const u8 *ies, uint ies_len)
{
	const u8 *he_op_ie;
	sint he_op_ielen;

	he_op_ie = rtw_get_ext_ie(ies, WLAN_EID_EXTENSION_HE_OPERATION, &he_op_ielen, ies_len);
	if (he_op_ie && he_op_ielen >= 1 + HE_OPER_PARAMS_LEN && GET_HE_OP_PARA_6GHZ_OP_INFO_PRESENT(he_op_ie + 3)) {
		u8 offset = HE_OPER_PARAMS_LEN + HE_OPER_BSS_COLOR_INFO_LEN + HE_OPER_BASIC_MCS_LEN
			+ (GET_HE_OP_PARA_VHT_OP_INFO_PRESENT(he_op_ie + 3) ? HE_OPER_VHT_OPER_INFO_LEN : 0)
			+ (GET_HE_OP_PARA_CO_HOSTED_BSS(he_op_ie + 3) ? HE_OPER_MAX_COHOST_BSSID_LEN : 0);

		if (he_op_ielen >= offset)
			return GET_HE_OP_INFO_REG_INFO(he_op_ie + 3 + offset);
	}
	return CIS_6G_REG_RSVD;
}

static bool rtw_parse_tpe_ie(const u8 *cont, u8 len, struct rtw_tpe_ele_t *tpe)
{
	u8 intp, cate, cnt, i;

	/* info & at least one max tx power */
	if (len < 2)
		return false;

	intp = GET_TPE_ELE_MAX_TXPWR_INTP(cont);
	if (intp >= TPE_ELE_MAX_TXPWR_INTP_NUM)
		return false;

	cate = GET_TPE_ELE_MAX_TXPWR_CATE(cont);
	if (cate >= TPE_ELE_MAX_TXPWR_CATE_NUM)
		return false;

	cnt = GET_TPE_ELE_MAX_TXPWR_CNT(cont);

	switch (intp) {
	case TPE_ELE_MAX_TXPWR_INTP_L_EIRP:
	case TPE_ELE_MAX_TXPWR_INTP_RC_EIRP:
		if (cnt >= 4)
			return false;
		cnt = cnt + 1;
		break;

	case TPE_ELE_MAX_TXPWR_INTP_L_EIRP_PSD:
	case TPE_ELE_MAX_TXPWR_INTP_RC_EIRP_PSD:
		if (cnt >= 5)
			return false;
		if (cnt == 0)
			cnt = 1;
		else
			cnt = BIT(cnt - 1);
		break;
	}

	if (len < 1 + cnt)
		cnt = len - 1; /* adjust cnt according to ie len */

	tpe->intp = intp;
	tpe->cate = cate;
	tpe->pwr_cnt = cnt;
	_rtw_memset(tpe->max_tx_pwr, 0, TPE_TXPWR_MAX_CNT);
	for (i = 0; i < cnt; i++)
		tpe->max_tx_pwr[i] = GET_TPE_ELE_MAX_TXPWR_VAL(cont, i);

	return true;
}

static u8 rtw_ies_get_tpes(const u8 *ies, uint ies_len, struct rtw_tpe_t *tpes)
{
	const u8 *tpe_ie;
	sint tpe_ielen;
	u8 cnt = 0;

	tpe_ie = rtw_get_ie(ies, WLAN_EID_TX_POWER_ENVELOPE, &tpe_ielen, ies_len);
	while (tpe_ie) {
		if (cnt >= ARRAY_SIZE(tpes->ele)) {
			struct rtw_tpe_ele_t tmp;

			if (rtw_parse_tpe_ie(tpe_ie + 2, tpe_ielen, &tmp)) {
				RTW_WARN("%s array size of ele not enough\n", __func__);
				break;
			}
		}
		if (rtw_parse_tpe_ie(tpe_ie + 2, tpe_ielen, &tpes->ele[cnt]))
			cnt++;
		if ((tpe_ie + 2 + tpe_ielen) - ies >= ies_len - 2)
			break;
		tpe_ie = rtw_get_ie(tpe_ie + 2 + tpe_ielen, WLAN_EID_TX_POWER_ENVELOPE, &tpe_ielen, ies_len);
	}

	tpes->cnt = cnt;
	return cnt;
}
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

static void rtw_cisr_update_chplan_status(struct rf_ctl_t *rfctl, struct country_ie_slave_record *cisr)
{
	u8 domain_code_6g = RTW_CHPLAN_6G_NULL;
	struct country_chplan *ent = &cisr->chplan;
	enum country_ie_slave_status status;

#if CONFIG_IEEE80211_BAND_6GHZ
	cisr->env = country_str_get_env(cisr->country_str);
#endif

	_rtw_memset(ent, 0, sizeof(*ent));

	if (is_all_null(cisr->country_str, 3)
		|| (cisr->country_str[0] == 'X'
			&& cisr->country_str[1] == 'X')
	) {
		status = COUNTRY_IE_SLAVE_NOCOUNTRY;
		goto exit;
	}

	if (!rtw_get_chplan_from_country(cisr->country_str, ent)) {
		status = COUNTRY_IE_SLAVE_UNKNOWN;
		goto exit;
	}

	#if CONFIG_IEEE80211_BAND_6GHZ
	domain_code_6g = ent->domain_code_6g;
	#endif

	if ((cisr->band == BAND_ON_5G && !COUNTRY_CHPLAN_EN_11A(ent))
		|| !rtw_chplan_is_bchbw_valid(ent->domain_code, domain_code_6g, cisr->band, cisr->opch
			, CHANNEL_WIDTH_20, CHAN_OFFSET_NO_EXT, 1, 1, rfctl)
	) {
		status = COUNTRY_IE_SLAVE_OPCH_NOEXIST;
		goto exit;
	}

#if CONFIG_IEEE80211_BAND_6GHZ
	if (cisr->band == BAND_ON_6G) {
		if (!(ent->cate_6g_map & reg_info_to_chplan_6g_cate_map(cisr->reg_info))) {
			status = COUNTRY_IE_SLAVE_CATE_6G_NS;
			goto exit;
		}
		if (!rtw_txpwr_hal_tpe_allow(rfctl_to_dvobj(rfctl), cisr)) {
			status = COUNTRY_IE_SLAVE_TPE_FORBID;
			goto exit;
		}
	}
#endif
	status = COUNTRY_IE_SLAVE_APPLICABLE;

exit:
	cisr->status = status;
}

static void rtw_get_cisr_from_regu_settings(struct rf_ctl_t *rfctl
	, enum band_type band, u8 opch
	, const u8 *country_str, enum country_ie_slave_6g_reg_info reg_info
	, struct rtw_tpe_t *tpes
	, struct country_ie_slave_record *cisr)
{
	if (country_str) {
		cisr->country_str[0] = country_str[0];
		cisr->country_str[1] = country_str[1];
		cisr->country_str[2] = country_str[2];
	} else {
		cisr->country_str[0] = 0;
		cisr->country_str[1] = 0;
		cisr->country_str[2] = 0;
	}
	cisr->band = band;
	cisr->opch = opch;

#if CONFIG_IEEE80211_BAND_6GHZ
	cisr->reg_info = reg_info;
	if (tpes)
		cisr->tpes = *tpes;
	else
		cisr->tpes.cnt = 0;
#endif

	rtw_cisr_update_chplan_status(rfctl, cisr);
}

static void rtw_get_cisr_from_regu_ies(struct rf_ctl_t *rfctl
	, enum band_type band, u8 opch
	, const u8 *ies, uint ies_len, bool country_str
	, struct country_ie_slave_record *cisr)
{
	rtw_ies_get_country_str(ies, country_str ? ies_len : 0, cisr->country_str);

	cisr->band = band;
	cisr->opch = opch;

#if CONFIG_IEEE80211_BAND_6GHZ
	if (band == BAND_ON_6G) {
		cisr->reg_info = rtw_ies_get_6g_reg_info(ies, ies_len);
		rtw_ies_get_tpes(ies, ies_len, &cisr->tpes);
	} else {
		cisr->reg_info = CIS_6G_REG_RSVD;
		cisr->tpes.cnt = 0;
	}
#endif

	rtw_cisr_update_chplan_status(rfctl, cisr);
}
#endif /* CONFIG_80211D */

void dump_country_chplan(void *sel, const struct country_chplan *ent, bool regd_info)
{
	char buf[16];
	char *pos = buf;

	if (ent->domain_code == RTW_CHPLAN_UNSPECIFIED)
		pos += sprintf(pos, "UNSPEC");
	else
		pos += sprintf(pos, "0x%02X", ent->domain_code);

#if CONFIG_IEEE80211_BAND_6GHZ
	if (ent->domain_code_6g == RTW_CHPLAN_6G_UNSPECIFIED)
		pos += sprintf(pos, " UNSPEC");
	else
		pos += sprintf(pos, " 0x%02X", ent->domain_code_6g);
#endif

	RTW_PRINT_SEL(sel, "\"%c%c\", %s"
		, ent->alpha2[0], ent->alpha2[1], buf);

	if (regd_info) {
		u8 edcca_mode[BAND_MAX];
		u8 txpwr_lmt[BAND_MAX];

		rtw_country_chplan_get_edcca_mode_of_bands(ent, edcca_mode);
		_RTW_PRINT_SEL(sel, " {%-6s", rtw_edcca_mode_str(edcca_mode[BAND_ON_24G]));
		#if CONFIG_IEEE80211_BAND_5GHZ
		_RTW_PRINT_SEL(sel, " %-6s", rtw_edcca_mode_str(edcca_mode[BAND_ON_5G]));
		#endif
		#if CONFIG_IEEE80211_BAND_6GHZ
		_RTW_PRINT_SEL(sel, " %-6s", rtw_edcca_mode_str(edcca_mode[BAND_ON_6G]));
		#endif
		_RTW_PRINT_SEL(sel, "}");

		rtw_country_chplan_get_txpwr_lmt_of_bands(ent, txpwr_lmt);
		_RTW_PRINT_SEL(sel, " {%-8s", txpwr_lmt_str(txpwr_lmt[BAND_ON_24G]));
		#if CONFIG_IEEE80211_BAND_5GHZ
		_RTW_PRINT_SEL(sel, " %-8s", txpwr_lmt_str(txpwr_lmt[BAND_ON_5G]));
		#endif
		#if CONFIG_IEEE80211_BAND_6GHZ
		_RTW_PRINT_SEL(sel, " %-8s", txpwr_lmt_str(txpwr_lmt[BAND_ON_6G]));
		#endif
		_RTW_PRINT_SEL(sel, "}");
	}

	_RTW_PRINT_SEL(sel, " %s", COUNTRY_CHPLAN_EN_11BE(ent) ? "be" : "  ");
	_RTW_PRINT_SEL(sel, " %s", COUNTRY_CHPLAN_EN_11AX(ent) ? "ax" : "  ");
	_RTW_PRINT_SEL(sel, " %s", COUNTRY_CHPLAN_EN_11AC(ent) ? "ac" : "  ");
	_RTW_PRINT_SEL(sel, " %s", COUNTRY_CHPLAN_EN_11A(ent) ? "a" : " ");

	#if CONFIG_IEEE80211_BAND_6GHZ
	_RTW_PRINT_SEL(sel, " %c%c%c"
		, (ent->cate_6g_map & BIT(CHPLAN_6G_CATE_STD)) ? 'S' : '_'
		, (ent->cate_6g_map & BIT(CHPLAN_6G_CATE_LPI)) ? 'I' : '_'
		, (ent->cate_6g_map & BIT(CHPLAN_6G_CATE_VLP)) ? 'V' : '_'
	);
	#endif

	_RTW_PRINT_SEL(sel, "\n");
}

void dump_country_chplan_map(void *sel, bool regd_info)
{
	struct country_chplan ent;
	u8 code[2];

	rtw_get_chplan_worldwide(&ent);
	dump_country_chplan(sel, &ent, regd_info);

	for (code[0] = 'A'; code[0] <= 'Z'; code[0]++) {
		for (code[1] = 'A'; code[1] <= 'Z'; code[1]++) {
			if (!rtw_get_chplan_from_country(code, &ent))
				continue;

			dump_country_chplan(sel, &ent, regd_info);
		}
	}
}

void dump_country_list(void *sel)
{
	u8 code[2];

	RTW_PRINT_SEL(sel, "%s ", WORLDWIDE_ALPHA2);

	for (code[0] = 'A'; code[0] <= 'Z'; code[0]++) {
		for (code[1] = 'A'; code[1] <= 'Z'; code[1]++) {
			if (!rtw_get_chplan_from_country(code, NULL))
				continue;
			_RTW_PRINT_SEL(sel, "%c%c ", code[0], code[1]);
		}
	}
	_RTW_PRINT_SEL(sel, "\n");
}

void dump_chplan_id_list(void *sel)
{
	u8 id_search_max = 255;
	u8 first = 1;
	int i;

	for (i = 0; i <= id_search_max; i++) {
		if (!rtw_is_channel_plan_valid(i))
			continue;

		if (first) {
			RTW_PRINT_SEL(sel, "0x%02X ", i);
			first = 0;
		} else
			_RTW_PRINT_SEL(sel, "0x%02X ", i);
	}
	if (first == 0)
		_RTW_PRINT_SEL(sel, "\n");
}

void dump_chplan_country_list(void *sel)
{
	u8 id_search_max = 255;
	int i;

	for (i = 0; i <= id_search_max; i++) {
		struct country_chplan ent;
		u8 code[2];
		u8 first;

		if (!rtw_is_channel_plan_valid(i))
			continue;

		first = 1;
		for (code[0] = 'A'; code[0] <= 'Z'; code[0]++) {
			for (code[1] = 'A'; code[1] <= 'Z'; code[1]++) {
				if (!rtw_get_chplan_from_country(code, &ent) || ent.domain_code != i)
					continue;

				if (first) {
					RTW_PRINT_SEL(sel, "0x%02X %c%c ", i, code[0], code[1]);
					first = 0;
				} else
					_RTW_PRINT_SEL(sel, "%c%c ", code[0], code[1]);
			}
		}
		if (first == 0)
			_RTW_PRINT_SEL(sel, "\n");
	}
}

#if CONFIG_IEEE80211_BAND_6GHZ
void dump_chplan_6g_id_list(void *sel)
{
	u8 id_search_max = 255;
	u8 first = 1;
	int i;

	for (i = 0; i <= id_search_max; i++) {
		if (!rtw_is_channel_plan_6g_valid(i))
			continue;

		if (first) {
			RTW_PRINT_SEL(sel, "0x%02X ", i);
			first = 0;
		} else
			_RTW_PRINT_SEL(sel, "0x%02X ", i);
	}
	if (first == 0)
		_RTW_PRINT_SEL(sel, "\n");
}

void dump_chplan_6g_country_list(void *sel)
{
	u8 id_search_max = 255;
	int i;

	for (i = 0; i <= id_search_max; i++) {
		struct country_chplan ent;
		u8 code[2];
		u8 first;

		if (!rtw_is_channel_plan_6g_valid(i))
			continue;

		first = 1;
		for (code[0] = 'A'; code[0] <= 'Z'; code[0]++) {
			for (code[1] = 'A'; code[1] <= 'Z'; code[1]++) {
				if (!rtw_get_chplan_from_country(code, &ent) || ent.domain_code_6g != i)
					continue;

				if (first) {
					RTW_PRINT_SEL(sel, "0x%02X %c%c ", i, code[0], code[1]);
					first = 0;
				} else
					_RTW_PRINT_SEL(sel, "%c%c ", code[0], code[1]);
			}
		}
		if (first == 0)
			_RTW_PRINT_SEL(sel, "\n");
	}
}
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

#ifdef CONFIG_RTW_CHPLAN_DEV
static void dump_chplan_get_ch_2g_search_test(void *sel, u8 id, u8 *ch_array, size_t ch_array_sz, bool content, u32 times)
{
	u8 flags_buf;
	u8 *flags = content ? &flags_buf : NULL;
	u32 round = times / ch_array_sz;
	u32 i, r;
	sysptime start, end;

	start = rtw_sptime_get_raw();

	for (r = 0; r < round; r++)
		for (i = 0; i < ch_array_sz; i++)
			rtw_chplan_get_ch(id, ch_array[i], flags);

	end = rtw_sptime_get_raw();
	RTW_PRINT_SEL(sel, "2G_CHD times:%10u round:%10u %10lld ns\n", times, round, rtw_sptime_diff_ns(start, end));
}

#if CONFIG_IEEE80211_BAND_5GHZ
static void dump_chplan_get_ch_5g_search_test(void *sel, u8 id, u8 *ch_array, size_t ch_array_sz, bool content, u32 times)
{
	u8 flags_buf;
	u8 *flags = content ? &flags_buf : NULL;
	u32 round = times / ch_array_sz;
	u32 i, r;
	sysptime start, end;

	start = rtw_sptime_get_raw();

	for (r = 0; r < round; r++)
		for (i = 0; i < ch_array_sz; i++)
			rtw_chplan_get_ch(id, ch_array[i], flags);

	end = rtw_sptime_get_raw();
	RTW_PRINT_SEL(sel, "5G_CHD times:%10u round:%10u %10lld ns\n", times, round, rtw_sptime_diff_ns(start, end));
}
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
static void dump_chplan_get_ch_6g_search_test(void *sel, u8 id, u8 *ch_array, size_t ch_array_sz, bool content, u32 times)
{
	u8 flags_buf;
	u8 *flags = content ? &flags_buf : NULL;
	u32 round = times / ch_array_sz;
	u32 i, r;
	sysptime start, end;

	start = rtw_sptime_get_raw();

	for (r = 0; r < round; r++)
		for (i = 0; i < ch_array_sz; i++)
			rtw_chplan_6g_get_ch(id, ch_array[i], flags);

	end = rtw_sptime_get_raw();
	RTW_PRINT_SEL(sel, "6G_CHD times:%10u round:%10u %10lld ns\n", times, round, rtw_sptime_diff_ns(start, end));
}
#endif

static const char alpha2_list[][2] = {
	"AD", "AE", "AF", "AG", "AI", "AL", "AM", "AN", "AO", "AQ",
	"AR", "AS", "AT", "AU", "AW", "AZ", "BA", "BB", "BD", "BE",
	"BF", "BG", "BH", "BI", "BJ", "BM", "BN", "BO", "BR", "BS",
	"BT", "BV", "BW", "BY", "BZ", "CA", "CC", "CD", "CF", "CG",
	"CH", "CI", "CK", "CL", "CM", "CN", "CO", "CR", "CV", "CX",
	"CY", "CZ", "DE", "DJ", "DK", "DM", "DO", "DZ", "EC", "EE",
	"EG", "EH", "ER", "ES", "ET", "FI", "FJ", "FK", "FM", "FO",
	"FR", "GA", "GB", "GD", "GE", "GF", "GG", "GH", "GI", "GL",
	"GM", "GN", "GP", "GQ", "GR", "GS", "GT", "GU", "GW", "GY",
	"HK", "HM", "HN", "HR", "HT", "HU", "ID", "IE", "IL", "IM",
	"IN", "IO", "IQ", "IR", "IS", "IT", "JE", "JM", "JO", "JP",
	"KE", "KG", "KH", "KI", "KM", "KN", "KR", "KW", "KY", "KZ",
	"LA", "LB", "LC", "LI", "LK", "LR", "LS", "LT", "LU", "LV",
	"LY", "MA", "MC", "MD", "ME", "MF", "MG", "MH", "MK", "ML",
	"MM", "MN", "MO", "MP", "MQ", "MR", "MS", "MT", "MU", "MV",
	"MW", "MX", "MY", "MZ", "NA", "NC", "NE", "NF", "NG", "NI",
	"NL", "NO", "NP", "NR", "NU", "NZ", "OM", "PA", "PE", "PF",
	"PG", "PH", "PK", "PL", "PM", "PR", "PS", "PT", "PW", "PY",
	"QA", "RE", "RO", "RS", "RU", "RW", "SA", "SB", "SC", "SE",
	"SG", "SH", "SI", "SJ", "SK", "SL", "SM", "SN", "SO", "SR",
	"ST", "SV", "SX", "SZ", "TC", "TD", "TF", "TG", "TH", "TJ",
	"TK", "TM", "TN", "TO", "TR", "TT", "TV", "TW", "TZ", "UA",
	"UG", "US", "UY", "UZ", "VA", "VC", "VE", "VG", "VI", "VN",
	"VU", "WF", "WS", "XK", "YE", "YT", "ZA", "ZM", "ZW"
};

static void dump_country_chplan_search_test(void *sel, const char cc_array[][2], size_t cc_array_sz, bool content, u32 times)
{
	struct country_chplan ent_buf;
	struct country_chplan *ent = content ? &ent_buf : NULL;
	u32 round = times / cc_array_sz;
	u32 i, r;
	sysptime start, end;

	start = rtw_sptime_get_raw();

	for (r = 0; r < round; r++)
		for (i = 0; i < cc_array_sz; i++)
			rtw_get_chplan_from_country(cc_array[i], ent);

	end = rtw_sptime_get_raw();
	RTW_PRINT_SEL(sel, "CC_MAP times:%10u round:%10u %10lld ns\n", times, round, rtw_sptime_diff_ns(start, end));
}

void dump_chplan_test(void *sel)
{
	if (regdb_ops.dump_chplan_test)
		regdb_ops.dump_chplan_test(sel);

	dump_chplan_get_ch_2g_search_test(sel, 0x7F, center_ch_2g, ARRAY_SIZE(center_ch_2g), false, 100000);
	dump_chplan_get_ch_2g_search_test(sel, 0x7F, center_ch_2g, ARRAY_SIZE(center_ch_2g), true, 100000);
	dump_chplan_get_ch_2g_search_test(sel, 0x7F, center_ch_2g, 1, false, 100000);
	dump_chplan_get_ch_2g_search_test(sel, 0x7F, center_ch_2g, 1, true, 100000);

#if CONFIG_IEEE80211_BAND_5GHZ
	dump_chplan_get_ch_5g_search_test(sel, 0x7F, center_ch_5g_20m, ARRAY_SIZE(center_ch_5g_20m), false, 100000);
	dump_chplan_get_ch_5g_search_test(sel, 0x7F, center_ch_5g_20m, ARRAY_SIZE(center_ch_5g_20m), true, 100000);
	dump_chplan_get_ch_5g_search_test(sel, 0x7F, center_ch_5g_20m, 1, false, 100000);
	dump_chplan_get_ch_5g_search_test(sel, 0x7F, center_ch_5g_20m, 1, true, 100000);
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
	dump_chplan_get_ch_6g_search_test(sel, 0x7F, center_ch_6g_20m, ARRAY_SIZE(center_ch_6g_20m), false, 100000);
	dump_chplan_get_ch_6g_search_test(sel, 0x7F, center_ch_6g_20m, ARRAY_SIZE(center_ch_6g_20m), true, 100000);
	dump_chplan_get_ch_6g_search_test(sel, 0x7F, center_ch_6g_20m, 1, false, 100000);
	dump_chplan_get_ch_6g_search_test(sel, 0x7F, center_ch_6g_20m, 1, true, 100000);
#endif

	dump_country_chplan_search_test(sel, alpha2_list, ARRAY_SIZE(alpha2_list), false, 100000);
	dump_country_chplan_search_test(sel, alpha2_list, ARRAY_SIZE(alpha2_list), true, 100000);
	dump_country_chplan_search_test(sel, alpha2_list, 1, false, 100000);
	dump_country_chplan_search_test(sel, alpha2_list, 1, true, 100000);
}
#endif /* CONFIG_RTW_CHPLAN_DEV */

void dump_chplan_ver(void *sel)
{
	char buf[CHPLAN_VER_STR_BUF_LEN] = {0};

	regdb_ops.get_ver_str(buf, CHPLAN_VER_STR_BUF_LEN);
	RTW_PRINT_SEL(sel, "%s\n", buf);
}

static struct regd_req_t *rtw_regd_req_alloc(void)
{
	struct regd_req_t *req;

	req = rtw_zmalloc(sizeof(struct regd_req_t));
	if (req)
		_rtw_init_listhead(&req->list);

	return req;
}

static struct regd_req_t *rtw_regd_req_alloc_with_country_chplan(struct country_chplan *ent)
{
	struct regd_req_t *req;

	req = rtw_regd_req_alloc();
	if (req)
		_rtw_memcpy(&req->chplan, ent, sizeof(req->chplan));

	return req;
}

static void rtw_regd_req_free(struct regd_req_t *req)
{
	rtw_mfree(req, sizeof(*req));
}

static void rtw_regd_req_list_insert(struct rf_ctl_t *rfctl, struct regd_req_t *req)
{
	rtw_list_insert_tail(&req->list, &rfctl->regd_req_list);
	rfctl->regd_req_num++;
}

#if defined(CONFIG_80211D) || defined(CONFIG_REGD_SRC_FROM_OS)
static void rtw_regd_req_list_delete(struct rf_ctl_t *rfctl, struct regd_req_t *req)
{
	rtw_list_delete(&req->list);
	rfctl->regd_req_num--;
}
#endif

void rtw_regd_req_list_init(struct rf_ctl_t *rfctl, struct registry_priv *regsty)
{
	_rtw_mutex_init(&rfctl->regd_req_mutex);
	_rtw_init_listhead(&rfctl->regd_req_list);
	rfctl->init_regd_always_apply = regsty->init_regd_always_apply;
	rfctl->user_regd_always_apply = regsty->user_regd_always_apply;
}

void rtw_regd_req_list_free(struct rf_ctl_t *rfctl)
{
	struct regd_req_t *req;
	_list *cur, *head;

	_rtw_mutex_lock_interruptible(&rfctl->regd_req_mutex);

	head = &rfctl->regd_req_list;
	cur = get_next(head);

	while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
		req = LIST_CONTAINOR(cur, struct regd_req_t, list);
		cur = get_next(cur);
		rtw_list_delete(&req->list);
		if (req != &rfctl->init_req)
			rtw_regd_req_free(req);
	}
	rfctl->user_req = NULL;
	rfctl->extra_req = NULL;
	rfctl->regd_req_num = 0;

	_rtw_mutex_unlock(&rfctl->regd_req_mutex);

	_rtw_mutex_free(&rfctl->regd_req_mutex);
}

bool rtw_regd_watchdog_hdl(struct dvobj_priv *dvobj)
{
	struct rf_ctl_t *rfctl = dvobj_to_rfctl(dvobj);
	bool ret = false;

	rtw_beacon_hint_expire(rfctl);

#ifdef CONFIG_DFS_MASTER
	rtw_rfctl_chset_chk_non_ocp_finish(rfctl);
#endif

#ifdef CONFIG_80211D
	ret = rtw_cis_scan_needed(rfctl, NULL);
#endif

	return ret;
}

static enum channel_width rtw_regd_adjust_op_bw(struct rf_ctl_t *rfctl, bool slave_m
	, enum band_type band, u8 ch, enum channel_width bw, enum chan_offset offset)
{
#ifndef DBG_REGD_ADJUST_LINKING_BW
#define DBG_REGD_ADJUST_LINKING_BW 0
#endif

	struct rtw_chset *chset = &rfctl->chset;

	#if DBG_REGD_ADJUST_LINKING_BW
	RTW_INFO("%s %s ch:%u,%u,%u\n"
		, __func__, band_str(band), ch, bw, offset);
	#endif

	if (bw == CHANNEL_WIDTH_20)
		goto exit;

	for (; bw > CHANNEL_WIDTH_20; bw--) {
		if (rtw_chset_is_bchbw_non_ocp(chset, band, ch, bw, offset)) {
			#if DBG_REGD_ADJUST_LINKING_BW
			RTW_INFO("%s %s ch:%u,%u,%u not allowed by non_ocp\n", __func__, band_str(band), ch, bw, offset);
			#endif
			continue;
		}

		/* TODO: slave_m downgrade bw to avoid not covered by master of STA link */

		if (!rtw_chset_is_bchbw_valid(chset, band, ch, bw, offset, !slave_m , !slave_m)) {
			#if DBG_REGD_ADJUST_LINKING_BW
			RTW_INFO("%s %s ch:%u,%u,%u not allowed by chset\n", __func__, band_str(band), ch, bw, offset);
			#endif
			continue;
		}

		break;
	}

	if (bw == CHANNEL_WIDTH_20)
		offset = CHAN_OFFSET_NO_EXT;

exit:
	rtw_warn_on(!rtw_chset_is_bchbw_valid(chset, band, ch, bw, offset, true, true));
	rtw_warn_on(rtw_chset_is_bchbw_non_ocp(chset, band, ch, bw, offset));

	return bw;
}

enum channel_width alink_adjust_linking_bw_by_regd(struct _ADAPTER_LINK *alink
	, enum band_type band, u8 ch, enum channel_width bw, enum chan_offset offset)
{
	_adapter *adapter = alink->adapter;
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);

	return rtw_regd_adjust_op_bw(rfctl, false, band, ch, bw, offset);
}

enum channel_width adapter_adjust_linking_bw_by_regd(_adapter *adapter
	, enum band_type band, u8 ch, enum channel_width bw, enum chan_offset offset)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);

	return rtw_regd_adjust_op_bw(rfctl, false, band, ch, bw, offset);
}

#ifdef CONFIG_AP_MODE
enum channel_width alink_adjust_slave_m_bw_by_regd(struct _ADAPTER_LINK *alink
	, enum band_type band, u8 ch, enum channel_width bw, enum chan_offset offset)
{
	_adapter *adapter = alink->adapter;
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);

	return rtw_regd_adjust_op_bw(rfctl, true, band, ch, bw, offset);
}

enum channel_width adapter_adjust_slave_m_bw_by_regd(_adapter *adapter
	, enum band_type band, u8 ch, enum channel_width bw, enum chan_offset offset)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);

	return rtw_regd_adjust_op_bw(rfctl, true, band, ch, bw, offset);
}
#endif

static void rtw_chplan_rtk_priv_domain_code_get_country_chplan(struct country_chplan *chplan
	, u8 domain_code, u8 domain_code_6g)
{
	if (rtw_chplan_ids_is_world_wide(domain_code, domain_code_6g)) {
		rtw_get_chplan_worldwide(chplan);
		chplan->domain_code = domain_code;
		#if CONFIG_IEEE80211_BAND_6GHZ
		chplan->domain_code_6g = domain_code_6g;
		#endif
	} else {
		SET_UNSPEC_ALPHA2(chplan->alpha2);
		chplan->domain_code = domain_code;
		#if CONFIG_IEEE80211_BAND_6GHZ
		chplan->domain_code_6g = domain_code_6g;
		#endif

		#if CONFIG_COUNTRY_CHPLAN_EDCCA_OVERRIDE
		chplan->edcca_2g_override = RTW_EDCCA_DEF;
		#if CONFIG_IEEE80211_BAND_5GHZ
		chplan->edcca_5g_override = RTW_EDCCA_DEF;
		#endif
		#if CONFIG_IEEE80211_BAND_6GHZ
		chplan->edcca_6g_override = RTW_EDCCA_DEF;
		#endif
		#endif

		chplan->txpwr_lmt_override = TXPWR_LMT_DEF;
		#ifdef CONFIG_CHPLAN_PROTO_EN
		chplan->proto_en = CHPLAN_PROTO_EN_ALL;
		#endif
	}
}

static void rtw_rfctl_regd_status_init_by_req(struct rf_ctl_t *rfctl, struct regd_req_t *req)
{
	struct country_chplan *chplan = &req->chplan;

	rfctl->regd_src = req->src;
	rfctl->regd_inr_bmp = BIT(req->inr);
	_rtw_memcpy(rfctl->alpha2, chplan->alpha2, 2);
	rfctl->domain_code = chplan->domain_code;
#if CONFIG_IEEE80211_BAND_6GHZ
	rfctl->domain_code_6g = chplan->domain_code_6g;
	rfctl->env_bmp = BIT(req->env);
#endif
#ifdef CONFIG_CHPLAN_PROTO_EN
	rfctl->proto_en = chplan->proto_en;
#endif
}

static void rtw_rfctl_regd_status_update_by_req(struct rf_ctl_t *rfctl, struct regd_req_t *req)
{
	if (rfctl->regd_src == REGD_SRC_NUM) {
		/* first applied req */
		rtw_rfctl_regd_status_init_by_req(rfctl, req);

	} else if (rfctl->regd_src == req->src || req->inr == RTW_REGD_SET_BY_EXTRA) {
		struct country_chplan *chplan = &req->chplan;

		rfctl->regd_inr_bmp |= BIT(req->inr);
		#if CONFIG_IEEE80211_BAND_6GHZ
		rfctl->env_bmp |= BIT(req->env);
		#endif
		if (_rtw_memcmp(rfctl->alpha2, chplan->alpha2, 2) != _TRUE
			|| rfctl->domain_code != chplan->domain_code
			#if CONFIG_IEEE80211_BAND_6GHZ
			|| rfctl->domain_code_6g != chplan->domain_code_6g
			#endif
		)
			SET_INTERSECTEDC_ALPHA2(rfctl->alpha2);
		if (rfctl->domain_code != chplan->domain_code)
			rfctl->domain_code = RTW_CHPLAN_UNSPECIFIED;
		#if CONFIG_IEEE80211_BAND_6GHZ
		if (rfctl->domain_code_6g != chplan->domain_code_6g)
			rfctl->domain_code_6g = RTW_CHPLAN_UNSPECIFIED;
		#endif
		#ifdef CONFIG_CHPLAN_PROTO_EN
		rfctl->proto_en &= chplan->proto_en;
		#endif
	} else {
		RTW_WARN("%s req->src:%s != applied src:%s\n", __func__, regd_src_str(req->src), regd_src_str(rfctl->regd_src));
		rtw_warn_on(1);
	}
}

#ifdef CONFIG_80211D
static bool rtw_rfctl_is_init_user_req_world_wide(const struct rf_ctl_t *rfctl)
{
	return (!rfctl->user_req && IS_ALPHA2_WORLDWIDE(rfctl->init_req.chplan.alpha2)) /* INIT is WW (when no USER) */
		|| (rfctl->user_req && IS_ALPHA2_WORLDWIDE(rfctl->user_req->chplan.alpha2)); /* USER is WW */
}

static bool rtw_rfctl_should_cis_enable(const struct rf_ctl_t *rfctl)
{
	if (rtw_rfctl_is_disable_sw_channel_plan(rfctl_to_dvobj(rfctl)))
		return false;
	if (rfctl->cis_en_mode == CISEM_ENABLE)
		return true;
	if (rfctl->cis_en_mode == CISEM_ENABLE_WW)
		return rtw_rfctl_is_init_user_req_world_wide(rfctl);
	return false;
}

static bool rtw_rfctl_update_cis_enabled(struct rf_ctl_t *rfctl, bool init, const char *caller)
{
	bool enabled = rtw_rfctl_should_cis_enable(rfctl);

	if (init || rfctl->cis_enabled != enabled) {
		rfctl->cis_enabled = enabled;
		RTW_INFO("%s set cis_enabled to %d\n", caller, enabled);
		return true;
	}
	return false;
}

static bool rtw_rfctl_forbid_unknown_country_opch(const struct rf_ctl_t *rfctl)
{
	return !RFCTL_REGD_SRC_FROM_OS(rfctl);
}

static void rtw_rfctl_update_cisr_collect_mode(struct rf_ctl_t *rfctl, bool init, const char *caller)
{
	bool collect_link_cisr;
	bool collect_network_cisr;
	bool forbid_unknown_country_opch = rtw_rfctl_forbid_unknown_country_opch(rfctl);

	collect_link_cisr = (rfctl->cis_enabled && !RFCTL_REGD_SRC_FROM_OS(rfctl))
		|| rtw_chset_has_6g_enabled(&rfctl->chset);

	collect_network_cisr = (rfctl->cis_enabled && (rfctl->cis_flags & CISF_ENV_BSS))
		|| (rtw_txpwr_hal_is_txpwr_limit_needed(rfctl_to_dvobj(rfctl))
			&& (forbid_unknown_country_opch || rtw_chset_has_6g_enabled(&rfctl->chset)));

	if (init || rfctl->collect_link_cisr != collect_link_cisr) {
		rfctl->collect_link_cisr = collect_link_cisr;
		RTW_INFO("%s set collect_link_cisr to %d\n", caller, collect_link_cisr);
	}
	if (init || rfctl->collect_network_cisr != collect_network_cisr) {
		rfctl->collect_network_cisr = collect_network_cisr;
		RTW_INFO("%s set collect_network_cisr to %d\n", caller, collect_network_cisr);
	}
}

#if CONFIG_IEEE80211_BAND_6GHZ
/* 
* Update txpwr_lmt_6g_cate_map for 
* a. REGD_SRC_RTK_PRIV's INIT/USER/EXTRA req
* b. REGD_SRC_OS's all req
*/
static void rtw_rfctl_update_regd_req_txpwr_6g_cate_map(struct rf_ctl_t *rfctl)
{
	struct regd_req_t *req;
	_list *cur, *head;

	/* apply txpwr lmt 6g cate map of req */
	head = &rfctl->regd_req_list;
	cur = get_next(head);
	while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
		req = LIST_CONTAINOR(cur, struct regd_req_t, list);
		cur = get_next(cur);

		if (rfctl->regd_src == REGD_SRC_RTK_PRIV) {
			/* apply txpwr lmt 6g cate map only for INIT/USER/EXTRA req */
			if (req->inr > RTW_REGD_SET_BY_USER && req->inr != RTW_REGD_SET_BY_EXTRA)
				continue;
		}

		if (rfctl->txpwr_lmt_link_6g_cate_map_applied_num) /* from per_link cisr */
			req->txpwr_lmt_6g_cate_map = rfctl->txpwr_lmt_link_6g_cate_map;
		else {
			if (req->inr == RTW_REGD_SET_BY_COUNTRY_IE) /* REGD_SRC_OS, without 6G link */
				req->txpwr_lmt_6g_cate_map = BIT(TXPWR_LMT_6G_CATE_VLP);
			else /* get from self's env */
				req->txpwr_lmt_6g_cate_map = rtw_env_to_txpwr_lmt_6g_cate_map(req->env);
		}
	}
}
#endif
#endif /* CONFIG_80211D */

static bool rtw_rfctl_get_chplan_from_alpha2(struct country_chplan *ent, const char *alpha2)
{
	if (IS_ALPHA2_WORLDWIDE(alpha2)
		|| rtw_get_chplan_from_country(alpha2, ent)
	) {
		if (IS_ALPHA2_WORLDWIDE(alpha2))
			rtw_get_chplan_worldwide(ent);
		return true;
	}
	return false;
}

static bool rtw_rfctl_update_extra_alpha2_req(struct rf_ctl_t *rfctl, const char *alpha2)
{
	struct country_chplan ent;

	if (rtw_rfctl_get_chplan_from_alpha2(&ent, alpha2)) {
		if (!rfctl->extra_req) {
			rfctl->extra_req = rtw_regd_req_alloc_with_country_chplan(&ent);
			if (!rfctl->extra_req) {
				rtw_warn_on(1);
				return false;
			}
			rfctl->extra_req->src = REGD_SRC_RTK_PRIV;
			rfctl->extra_req->inr = RTW_REGD_SET_BY_EXTRA;
			#if CONFIG_IEEE80211_BAND_6GHZ
			rfctl->extra_req->env = RTW_ENV_NUM;
			#endif
			rtw_regd_req_list_insert(rfctl, rfctl->extra_req);
		} else {
			if (rfctl->extra_req->chplan.alpha2[0] == ent.alpha2[0]
				&& rfctl->extra_req->chplan.alpha2[1] == ent.alpha2[1])
				return false;
			rfctl->extra_req->chplan = ent;
		}

		return true;

	} else if (rfctl->extra_req) {
		rtw_regd_req_list_delete(rfctl, rfctl->extra_req);
		rtw_regd_req_free(rfctl->extra_req);
		rfctl->extra_req = NULL;
		return true;
	}

	return false;
}

static bool rtw_rfctl_extra_alpha2_req_needed(struct rf_ctl_t *rfctl)
{
	if (RFCTL_GET_CIS_MAJORITY(rfctl))
		return false;

	if (rfctl->regd_src == REGD_SRC_RTK_PRIV) {
		struct regd_req_t *req, *extra_req = rfctl->extra_req;
		_list *cur, *head;

		head = &rfctl->regd_req_list;
		cur = get_next(head);
		while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
			req = LIST_CONTAINOR(cur, struct regd_req_t, list);
			cur = get_next(cur);

			if (!req->applied || req->inr == RTW_REGD_SET_BY_EXTRA)
				continue;

			if  (IS_ALPHA2_WORLDWIDE(req->chplan.alpha2)
				|| (req->chplan.alpha2[0] == extra_req->chplan.alpha2[0]
					&& req->chplan.alpha2[1] == extra_req->chplan.alpha2[1]))
				return false;
		}
	}
	return true;
}

/*
 * Description:
 *	Use hardware(efuse), driver parameter(registry) and default channel plan
 *	to decide which one should be used.
 *
 * Parameters:
 *	rfctl				pointer of rfctl
 *	hw_alpha2		country code from HW (efuse/eeprom/mapfile)
 *	hw_chplan		domain code from HW (efuse/eeprom/mapfile)
 *	hw_chplan_6g	6g domain code from HW (efuse/eeprom/mapfile)
 *	hw_force_chplan	if forcing HW channel plan setting (efuse/eeprom/mapfile)
 *					will modified tif HW channel plan setting is invlid, will
 */
void rtw_rfctl_decide_init_chplan(struct rf_ctl_t *rfctl,
	const char *hw_alpha2, u8 hw_chplan, u8 hw_chplan_6g, u8 hw_force_chplan)
{
	struct registry_priv *regsty;
	char *sw_alpha2;
	const struct country_chplan *country_ent = NULL;
	struct country_chplan ent;
	int chplan = -1;
	int chplan_6g = -1;

	u8 sw_chplan;
	u8 def_chplan = RTW_CHPLAN_WORLDWIDE; /* worldwide,  used when HW, SW both invalid */
#if CONFIG_IEEE80211_BAND_6GHZ
	u8 sw_chplan_6g;
	u8 def_chplan_6g = RTW_CHPLAN_6G_WORLDWIDE; /* worldwide,  used when HW, SW both invalid */
#endif

	if (hw_alpha2) {
		if (strlen(hw_alpha2) != 2 || !is_alpha(hw_alpha2[0]) || is_alpha(hw_alpha2[1]))
			RTW_PRINT("%s hw_alpha2 is not valid alpha2\n", __func__);
		else if (rtw_get_chplan_from_country(hw_alpha2, &ent)) {
			/* get chplan from hw country code, by pass hw chplan setting */
			country_ent = &ent;
			chplan = ent.domain_code;
			#if CONFIG_IEEE80211_BAND_6GHZ
			chplan_6g = ent.domain_code_6g;
			#endif
			goto chk_sw_config;
		} else
			RTW_PRINT("%s unsupported hw_alpha2:\"%c%c\"\n", __func__, hw_alpha2[0], hw_alpha2[1]);
	}

	if (rtw_is_channel_plan_valid(hw_chplan))
		chplan = hw_chplan;
	else if (hw_force_chplan == _TRUE) {
		RTW_PRINT("%s unsupported hw_chplan:0x%02X\n", __func__, hw_chplan);
		/* hw infomaton invalid, refer to sw information */
		hw_force_chplan = _FALSE;
	}

#if CONFIG_IEEE80211_BAND_6GHZ
	if (rtw_is_channel_plan_6g_valid(hw_chplan_6g))
		chplan_6g = hw_chplan_6g;
	else if (hw_force_chplan == _TRUE) {
		RTW_PRINT("%s unsupported hw_chplan_6g:0x%02X\n", __func__, hw_chplan_6g);
		/* hw infomaton invalid, refer to sw information */
		hw_force_chplan = _FALSE;
	}
#endif

chk_sw_config:
	regsty = dvobj_to_regsty(rfctl_to_dvobj(rfctl));

	if (hw_force_chplan == _TRUE)
		goto done;

	sw_alpha2 = regsty->alpha2;
	sw_chplan = regsty->channel_plan;
	#if CONFIG_IEEE80211_BAND_6GHZ
	sw_chplan_6g = regsty->channel_plan_6g;
	#endif

	if (sw_alpha2 && !IS_ALPHA2_UNSPEC(sw_alpha2)) {
		if (rtw_rfctl_get_chplan_from_alpha2(&ent, sw_alpha2)) {
			/* get chplan from sw country code, by pass sw chplan setting */
			country_ent = &ent;
			chplan = ent.domain_code;
			#if CONFIG_IEEE80211_BAND_6GHZ
			chplan_6g = ent.domain_code_6g;
			#endif
			goto done;
		} else
			RTW_PRINT("%s unsupported sw_alpha2:\"%c%c\"\n", __func__, sw_alpha2[0], sw_alpha2[1]);
	}

	if (rtw_is_channel_plan_valid(sw_chplan)) {
		/* cancel hw_alpha2 because chplan is specified by sw_chplan */
		country_ent = NULL;
		chplan = sw_chplan;
	} else if (sw_chplan != RTW_CHPLAN_UNSPECIFIED)
		RTW_PRINT("%s unsupported sw_chplan:0x%02X\n", __func__, sw_chplan);

#if CONFIG_IEEE80211_BAND_6GHZ
	if (rtw_is_channel_plan_6g_valid(sw_chplan_6g)) {
		/* cancel hw_alpha2 because chplan_6g is specified by sw_chplan_6g */
		country_ent = NULL;
		chplan_6g = sw_chplan_6g;
	} else if (sw_chplan_6g != RTW_CHPLAN_6G_UNSPECIFIED)
		RTW_PRINT("%s unsupported sw_chplan_6g:0x%02X\n", __func__, sw_chplan_6g);
#endif

done:
	if (chplan == -1) {
		RTW_PRINT("%s use def_chplan:0x%02X\n", __func__, def_chplan);
		chplan = def_chplan;
	} else
		RTW_PRINT("%s chplan:0x%02X\n", __func__, chplan);

#if CONFIG_IEEE80211_BAND_6GHZ
	if (chplan_6g == -1) {
		RTW_PRINT("%s use def_chplan_6g:0x%02X\n", __func__, def_chplan_6g);
		chplan_6g = def_chplan_6g;
	} else
		RTW_PRINT("%s chplan_6g:0x%02X\n", __func__, chplan_6g);
#endif

	if (!country_ent)
		rtw_chplan_rtk_priv_domain_code_get_country_chplan(&ent, chplan, chplan_6g);
	else {
		RTW_PRINT("%s country code:\"%c%c\"\n", __func__
			, country_ent->alpha2[0], country_ent->alpha2[1]);
	}

#if CONFIG_IEEE80211_BAND_6GHZ
	RTW_PRINT("%s env:%s\n", __func__, env_str(regsty->env));
#endif

	rfctl->disable_sw_chplan = hw_force_chplan;

	rfctl->init_req.src = REGD_SRC_RTK_PRIV;
	rfctl->init_req.inr = RTW_REGD_SET_BY_INIT;
#if CONFIG_IEEE80211_BAND_6GHZ
	rfctl->init_req.env = regsty->env;
#endif
	rfctl->init_req.applied = true;
	_rtw_memcpy(&rfctl->init_req.chplan, &ent, sizeof(ent));
	rtw_regd_req_list_insert(rfctl, &rfctl->init_req);

	rtw_rfctl_regd_status_init_by_req(rfctl, &rfctl->init_req);

	if (!rfctl->disable_sw_chplan) {
		/* handle extra_alpha2 */
		sw_alpha2 = regsty->extra_alpha2;
		if (sw_alpha2 && !IS_ALPHA2_UNSPEC(sw_alpha2)) {
			bool added = rtw_rfctl_update_extra_alpha2_req(rfctl, sw_alpha2);

			RTW_PRINT("%s%s extra_alpha2:\"%c%c\"\n", __func__
				,  added ? "" :  " unsupported"
				, sw_alpha2[0], sw_alpha2[1]);

			if (added) {
				rfctl->extra_req->applied = rtw_rfctl_extra_alpha2_req_needed(rfctl);
				if (rfctl->extra_req->applied)
					rtw_rfctl_regd_status_update_by_req(rfctl, rfctl->extra_req);
			}
		}
	}
}

void rtw_rfctl_apply_init_chplan(struct rf_ctl_t *rfctl, bool req_lock)
{
	struct dvobj_priv *dvobj = rfctl_to_dvobj(rfctl);

	rtw_edcca_mode_update(dvobj, req_lock);
	rtw_rfctl_chset_apply_regulatory(dvobj, req_lock);

#ifdef CONFIG_80211D
	rtw_rfctl_update_cis_enabled(rfctl, true, __func__);
	rtw_rfctl_update_cisr_collect_mode(rfctl, true, __func__);
	rfctl->init_user_req_is_ww = rtw_rfctl_is_init_user_req_world_wide(rfctl);
	#if CONFIG_IEEE80211_BAND_6GHZ
	rtw_rfctl_update_regd_req_txpwr_6g_cate_map(rfctl);
	#endif
#endif
}

bool rtw_rfctl_is_disable_sw_channel_plan(struct dvobj_priv *dvobj)
{
	return dvobj_to_rfctl(dvobj)->disable_sw_chplan;
}

static void get_str_of_set_chplan_keys(char *buf, u8 buf_len, struct SetChannelPlan_param *param)
{
	char *pos = buf;

#ifdef CONFIG_80211D
#if CONFIG_IEEE80211_BAND_6GHZ
#define CHPLAN_KEYS_FMT_80211D_6G " env:%s reg_info:%s"
#define CHPLAN_KEYS_ARG_80211D_6G , env_str(param->cisr.env), cis_6g_reg_info_str(param->cisr.reg_info)
#else
#define CHPLAN_KEYS_FMT_80211D_6G
#define CHPLAN_KEYS_ARG_80211D_6G
#endif

	if (param->regd_src == REGD_SRC_RTK_PRIV && param->inr == RTW_REGD_SET_BY_COUNTRY_IE) {
		if (param->has_cisr) {
			pos += snprintf(pos, buf_len - (pos - buf), "alid:%c alpha2:"ALPHA2_FMT
				CHPLAN_KEYS_FMT_80211D_6G
				" %s"
				, param->cisr_alink_id >= RTW_RLINK_MAX ? '-' : '0' + param->cisr_alink_id
				, ALPHA2_ARG(param->cisr.country_str)
				CHPLAN_KEYS_ARG_80211D_6G
				, cis_status_str(param->cisr.status));
		} else
			*buf = '\0';
	} else
#endif
	{
		if (param->has_country)
			pos += snprintf(pos, buf_len - (pos - buf), "alpha2:"ALPHA2_FMT, ALPHA2_ARG(param->country_ent.alpha2));
		else {
			if (param->channel_plan != RTW_CHPLAN_UNSPECIFIED)
				pos += snprintf(pos, buf_len - (pos - buf), "chplan:0x%02X", param->channel_plan);

			#if CONFIG_IEEE80211_BAND_6GHZ
			if (param->channel_plan_6g != RTW_CHPLAN_6G_UNSPECIFIED)
				pos += snprintf(pos, buf_len - (pos - buf), " chplan_6g:0x%02X", param->channel_plan_6g);
			#endif
		}

		#if CONFIG_IEEE80211_BAND_6GHZ
		if (param->env < RTW_ENV_NUM)
			pos += snprintf(pos, buf_len - (pos - buf), " env:%s", env_str(param->env));
		if (param->force_txpwr_lmt_6g_cate < TXPWR_LMT_6G_CATE_NUM)
			pos += snprintf(pos, buf_len - (pos - buf), " force_txpwr_lmt_6g_cate:%s"
				, txpwr_lmt_6g_cate_str(param->force_txpwr_lmt_6g_cate));
		if (param->force_txpwr_lmt_6g_cate == TXPWR_LMT_6G_CATE_NONE)
			pos += snprintf(pos, buf_len - (pos - buf), " force_txpwr_lmt_6g_cate:AUTO");
		#endif
	}
}

#define EXCL_CHS_STR_LEN (MAX_CHANNEL_NUM_2G_5G * 4)
#define EXCL_CHS_6G_STR_LEN (MAX_CHANNEL_NUM_6G * 4)
#if defined(CONFIG_RTW_DEBUG) || defined(CONFIG_PROC_DEBUG)
static char *get_str_of_u8_array(char *buf, size_t buf_len, u8 array[], size_t array_len, char delim, bool zero_end)
{
	char *pos = buf;
	char d_str[2] = {delim, '\0'};
	int i;

	for (i = 0; i < array_len && (!zero_end || array[i]); i++) {
		pos += snprintf(pos, buf_len - (pos - buf), "%s%u"
			, pos == buf ? "" : d_str, array[i]);
		if (pos >= buf + buf_len - 1)
			break;
	}
	if (pos == buf)
		buf[0] = '\0';

	return buf;
}
#endif

#define RTW_PRIV_USER_SET_DOMAIN		BIT0
#define RTW_PRIV_USER_SET_DOMAIN_6G		BIT1
#define RTW_PRIV_USER_SET_COUNTRY		BIT2
#define RTW_PRIV_USER_SET_ENV			BIT3
#define RTW_PRIV_USER_SET_TXPWR_LMT_6G_CATE	BIT4

static int rtw_chplan_rtk_priv_req_prehdl_domain_code(struct rf_ctl_t *rfctl, struct SetChannelPlan_param *param, const char *caller)
{
	if (param->priv_user_set_bmp & RTW_PRIV_USER_SET_DOMAIN) {
		/* disallow invalid input */
		if (!rtw_is_channel_plan_valid(param->channel_plan)) {
			RTW_WARN("%s invalid chplan:0x%02X\n", caller, param->channel_plan);
			return _FAIL;
		}
	} else {
		/* use original value when unspecified */
		if (rfctl->user_req)
			param->channel_plan = rfctl->user_req->chplan.domain_code;
		else
			param->channel_plan = rfctl->init_req.chplan.domain_code;
	}

#if CONFIG_IEEE80211_BAND_6GHZ
	if (param->priv_user_set_bmp & RTW_PRIV_USER_SET_DOMAIN_6G) {
		if (!rtw_is_channel_plan_6g_valid(param->channel_plan_6g)) {
			RTW_WARN("%s invalid chplan_6g:0x%02X\n", caller, param->channel_plan_6g);
			return _FAIL;
		}
	} else {
		/* use original value when unspecified */
		if (rfctl->user_req)
			param->channel_plan_6g = rfctl->user_req->chplan.domain_code_6g;
		else
			param->channel_plan_6g = rfctl->init_req.chplan.domain_code_6g;
	}
#endif

	return _SUCCESS;
}

static void rtw_chplan_rtk_priv_req_prehdl_country_ent(struct rf_ctl_t *rfctl, struct SetChannelPlan_param *param)
{
	if (!param->has_country) {
		struct country_chplan *ent = &param->country_ent;

		if (!(param->priv_user_set_bmp & (RTW_PRIV_USER_SET_DOMAIN | RTW_PRIV_USER_SET_DOMAIN_6G))) {
			/* no domain/_6g specified, keep original country setting */
			if (rfctl->user_req)
				_rtw_memcpy(ent, &rfctl->user_req->chplan, sizeof(*ent));
			else
				_rtw_memcpy(ent, &rfctl->init_req.chplan, sizeof(*ent));
		} else {
			/* set world wide or unspecified */
			u8 chplan_6g = RTW_CHPLAN_6G_NULL;

			#if CONFIG_IEEE80211_BAND_6GHZ
			chplan_6g = param->channel_plan_6g;
			#endif

			rtw_chplan_rtk_priv_domain_code_get_country_chplan(ent, param->channel_plan, chplan_6g);
		}

		param->has_country = 1;
	}
}

#ifdef CONFIG_80211D
static u8 get_cis_scan_band_bmp(struct rf_ctl_t *rfctl, u8 band_bmp)
{
	struct rtw_chset *chset = &rfctl->chset;
	u8 bmp = 0;
	int i;

	for (i = 0; i < BAND_MAX; i++)
		if (chset->chs_of_band[i])
			bmp |= BIT(i);

	return bmp & band_bmp;
}

static u32 get_cis_scan_urgent_ms(u32 int_ms, u32 urgent_ms)
{
	if (int_ms && urgent_ms)
		return rtw_max(int_ms, urgent_ms);
	return 0;
}
#endif

static void rtw_chplan_rtk_priv_req_prehdl_confs(struct rf_ctl_t *rfctl, struct chplan_confs *confs, const char *caller)
{
	if (confs->set_types & BIT(CHPLAN_CONFS_DIS_CH_FLAGS)) {
		#if !CONFIG_DFS
		/* force disable DFS channel because no DFS capability */
		confs->dis_ch_flags |= RTW_CHF_DFS;
		#endif
		if (rfctl->dis_ch_flags != confs->dis_ch_flags) {
			char buf[RTW_CH_FLAGS_STR_LEN];

			rfctl->dis_ch_flags = confs->dis_ch_flags;
			RTW_INFO("%s set dis_ch_flags to %s\n", caller
				, rtw_get_ch_flags_str(buf, confs->dis_ch_flags, ','));
		} else
			confs->set_types &= ~BIT(CHPLAN_CONFS_DIS_CH_FLAGS);
	}

	if (confs->set_types & BIT(CHPLAN_CONFS_EXCL_CHS)) {
		if (_rtw_memcmp(rfctl->excl_chs, confs->excl_chs, sizeof(rfctl->excl_chs)) == _FALSE) {
			char buf[EXCL_CHS_STR_LEN];

			_rtw_memcpy(rfctl->excl_chs, confs->excl_chs, sizeof(rfctl->excl_chs));

			RTW_INFO("%s set excl_chs to %s\n", caller
				, get_str_of_u8_array(buf, sizeof(buf), confs->excl_chs, MAX_CHANNEL_NUM_2G_5G, ',', true));
		} else
			confs->set_types &= ~BIT(CHPLAN_CONFS_EXCL_CHS);
	}

#if CONFIG_IEEE80211_BAND_6GHZ
	if (confs->set_types & BIT(CHPLAN_CONFS_EXCL_CHS_6G)) {
		if (_rtw_memcmp(rfctl->excl_chs_6g, confs->excl_chs_6g, sizeof(rfctl->excl_chs_6g)) == _FALSE) {
			char buf[EXCL_CHS_6G_STR_LEN];

			_rtw_memcpy(rfctl->excl_chs_6g, confs->excl_chs_6g, sizeof(rfctl->excl_chs_6g));

			RTW_INFO("%s set excl_chs_6g to %s\n", caller
				, get_str_of_u8_array(buf, sizeof(buf), confs->excl_chs_6g, MAX_CHANNEL_NUM_6G, ',', true));
		} else
			confs->set_types &= ~BIT(CHPLAN_CONFS_EXCL_CHS_6G);
	}
#endif

	if (confs->set_types & BIT(CHPLAN_CONFS_INIT_REGD_ALWAYS_APPLY)) {
		bool val_changed = false;

		if (rfctl->init_regd_always_apply != confs->init_regd_always_apply) {
			rfctl->init_regd_always_apply = confs->init_regd_always_apply;
			val_changed = true;
			RTW_INFO("%s set init_regd_always_apply to %d\n"
				, caller, confs->init_regd_always_apply);
		}

		if (!val_changed || rfctl->regd_src != REGD_SRC_RTK_PRIV
			|| rfctl->regd_req_num == 1 /* only INIT */
		)
			confs->set_types &= ~BIT(CHPLAN_CONFS_INIT_REGD_ALWAYS_APPLY);
	}

	if (confs->set_types & BIT(CHPLAN_CONFS_USER_REGD_ALWAYS_APPLY)) {
		bool val_changed = false;

		if (rfctl->user_regd_always_apply != confs->user_regd_always_apply) {
			rfctl->user_regd_always_apply = confs->user_regd_always_apply;
			val_changed = true;
			RTW_INFO("%s set user_regd_always_apply to %d\n"
				, caller, confs->user_regd_always_apply);
		}

		if (!val_changed || rfctl->regd_src != REGD_SRC_RTK_PRIV
			|| !rfctl->user_req
			|| rfctl->regd_req_num == 2 /* only INIT & USER */
		)
			confs->set_types &= ~BIT(CHPLAN_CONFS_USER_REGD_ALWAYS_APPLY);
	}

	if (confs->set_types & BIT(CHPLAN_CONFS_BCN_HINT_VALID_MS)) {
		if (rfctl->bcn_hint_valid_ms != confs->bcn_hint_valid_ms) {
			rfctl->bcn_hint_valid_ms = confs->bcn_hint_valid_ms;
			RTW_INFO("%s set bcn_hint_valid_ms to %u\n", caller, confs->bcn_hint_valid_ms);
		}
		confs->set_types &= ~BIT(CHPLAN_CONFS_BCN_HINT_VALID_MS); /* setting done here */
	}

	if (confs->set_types & BIT(CHPLAN_CONFS_EXTRA_ALPHA2)) {
		if (!rtw_rfctl_update_extra_alpha2_req(rfctl, confs->extra_alpha2))
			confs->set_types &= ~BIT(CHPLAN_CONFS_EXTRA_ALPHA2); /* setting done here */
		else {
			if (rfctl->extra_req) {
				RTW_INFO("%s set extra_alpha2 to "ALPHA2_FMT"\n", caller
					, ALPHA2_ARG(rfctl->extra_req->chplan.alpha2));
			} else
				RTW_INFO("%s clear extra_alpha2\n", caller);
		}
	}

#ifdef CONFIG_80211D
	#ifdef CONFIG_REGD_SRC_FROM_OS
	if (RFCTL_REGD_SRC_FROM_OS(rfctl)) {
		/* regd_src from OS has its own 802.11d behavior, do not allow change */
		confs->set_types &= ~(BIT(CHPLAN_CONFS_CIS_EN_MODE)
					| BIT(CHPLAN_CONFS_CIS_FLAGS)
					| BIT(CHPLAN_CONFS_CIS_EN_ROLE)
					| BIT(CHPLAN_CONFS_CIS_EN_IFBMP)
					| BIT(CHPLAN_CONFS_CIS_SCAN_INT_MS)
					);
	}
	#endif
	if (confs->set_types & BIT(CHPLAN_CONFS_CIS_EN_MODE)) {
		if (CIS_EN_MODE_IS_VALID(confs->cis_en_mode)
			&& rfctl->cis_en_mode != confs->cis_en_mode
		) {
			rfctl->cis_en_mode = confs->cis_en_mode;
			RTW_INFO("%s set cis_en_mode to %u\n", caller, confs->cis_en_mode);
		} else
			confs->set_types &= ~BIT(CHPLAN_CONFS_CIS_EN_MODE);
	}

	if (confs->set_types & BIT(CHPLAN_CONFS_CIS_FLAGS)) {
		if (confs->cis_flags & CISF_ENV_BSS_MAJ)
			confs->cis_flags |= CISF_ENV_BSS;
		if (confs->cis_flags & ~CISF_VALIDS) {
			RTW_WARN("%s cis_flags:0x%02x has undefined bits, apply valid bits only\n", caller, confs->cis_flags);
			confs->cis_flags &= CISF_VALIDS;
		}
		if (rfctl->cis_flags != confs->cis_flags) {
			rfctl->cis_flags = confs->cis_flags;
			RTW_INFO("%s set cis_flags to 0x%02x\n", caller, confs->cis_flags);
		} else
			confs->set_types &= ~BIT(CHPLAN_CONFS_CIS_FLAGS);
	}

	if (confs->set_types & BIT(CHPLAN_CONFS_CIS_EN_ROLE)) {
		if (rfctl->cis_en_role != confs->cis_en_role) {
			rfctl->cis_en_role = confs->cis_en_role;
			RTW_INFO("%s set cis_en_role to 0x%02x\n", caller, confs->cis_en_role);
		}
		confs->set_types &= ~BIT(CHPLAN_CONFS_CIS_EN_ROLE); /* setting done here */
	}

	if (confs->set_types & BIT(CHPLAN_CONFS_CIS_EN_IFBMP)) {
		if (rfctl->cis_en_ifbmp != confs->cis_en_ifbmp) {
			rfctl->cis_en_ifbmp = confs->cis_en_ifbmp;
			RTW_INFO("%s set cis_en_ifbmp to 0x%02x\n", caller, confs->cis_en_ifbmp);
		}
		confs->set_types &= ~BIT(CHPLAN_CONFS_CIS_EN_IFBMP); /* setting done here */
	}

	if (confs->set_types & BIT(CHPLAN_CONFS_CIS_SCAN_BAND_BMP)) {
		if (rfctl->cis_scan_band_bmp != confs->cis_scan_band_bmp) {
			rfctl->cis_scan_band_bmp = get_cis_scan_band_bmp(rfctl, confs->cis_scan_band_bmp);
			RTW_INFO("%s set cis_scan_band_bmp to 0x%02x\n", caller, rfctl->cis_scan_band_bmp);
		}
		confs->set_types &= ~BIT(CHPLAN_CONFS_CIS_SCAN_BAND_BMP); /* setting done here */
	}

	if (confs->set_types & BIT(CHPLAN_CONFS_CIS_SCAN_INT_MS)) {
		if (rfctl->cis_scan_int_ms != confs->cis_scan_int_ms) {
			u32 urgent_ms;

			rfctl->cis_scan_int_ms = confs->cis_scan_int_ms;
			RTW_INFO("%s set cis_scan_int_ms to %u\n", caller, confs->cis_scan_int_ms);

			if (confs->set_types & BIT(CHPLAN_CONFS_CIS_SCAN_URGENT_MS))
				urgent_ms = get_cis_scan_urgent_ms(rfctl->cis_scan_int_ms, confs->cis_scan_urgent_ms);
			else
				urgent_ms = get_cis_scan_urgent_ms(rfctl->cis_scan_int_ms, rfctl->cis_scan_urgent_ms);

			if (rfctl->cis_scan_urgent_ms != urgent_ms) {
				rfctl->cis_scan_urgent_ms = urgent_ms;
				RTW_INFO("%s set cis_scan_urgent_ms to %u\n", caller, rfctl->cis_scan_urgent_ms);
			}
			confs->set_types &= ~BIT(CHPLAN_CONFS_CIS_SCAN_URGENT_MS); /* setting done here */
		}
		confs->set_types &= ~BIT(CHPLAN_CONFS_CIS_SCAN_INT_MS); /* setting done here */
	}

	if (confs->set_types & BIT(CHPLAN_CONFS_CIS_SCAN_URGENT_MS)) {
		if (rfctl->cis_scan_urgent_ms != confs->cis_scan_urgent_ms) {
			rfctl->cis_scan_urgent_ms = get_cis_scan_urgent_ms(rfctl->cis_scan_int_ms, confs->cis_scan_urgent_ms);
			RTW_INFO("%s set cis_scan_urgent_ms to %u\n", caller, rfctl->cis_scan_urgent_ms);
		}
		confs->set_types &= ~BIT(CHPLAN_CONFS_CIS_SCAN_URGENT_MS); /* setting done here */
	}
#endif
}

static bool rtw_chplan_rtk_priv_req_prehdl_user(struct rf_ctl_t *rfctl, struct SetChannelPlan_param *param, const char *caller)
{
	bool effected = false;

	if (param->channel_plan != RTW_CHPLAN_UNSPECIFIED)
		param->priv_user_set_bmp |= RTW_PRIV_USER_SET_DOMAIN;
	if (param->has_country)
		param->priv_user_set_bmp |= RTW_PRIV_USER_SET_COUNTRY;
#if CONFIG_IEEE80211_BAND_6GHZ
	if (param->channel_plan_6g != RTW_CHPLAN_6G_UNSPECIFIED)
		param->priv_user_set_bmp |= RTW_PRIV_USER_SET_DOMAIN_6G;
	if (param->env < RTW_ENV_NUM)
		param->priv_user_set_bmp |= RTW_PRIV_USER_SET_ENV;
	if (param->force_txpwr_lmt_6g_cate < TXPWR_LMT_6G_CATE_NUM
		|| param->force_txpwr_lmt_6g_cate == TXPWR_LMT_6G_CATE_NONE)
		param->priv_user_set_bmp |= RTW_PRIV_USER_SET_TXPWR_LMT_6G_CATE;
#endif

	if (!param->priv_user_set_bmp && !param->confs.set_types) {
		/* meaningless input */
		RTW_WARN("%s meaningless input\n", caller);
		goto exit;
	}

	/* check input parameter */
	if (rtw_chplan_rtk_priv_req_prehdl_domain_code(rfctl, param, __func__) != _SUCCESS)
		goto exit;

	if (param->priv_user_set_bmp & (RTW_PRIV_USER_SET_DOMAIN | RTW_PRIV_USER_SET_DOMAIN_6G
		| RTW_PRIV_USER_SET_COUNTRY | RTW_PRIV_USER_SET_ENV)
	) {
		rtw_chplan_rtk_priv_req_prehdl_country_ent(rfctl, param);

		/* allows only one user request */
		if (!rfctl->user_req) {
			rfctl->user_req = rtw_regd_req_alloc_with_country_chplan(&param->country_ent);
			if (rfctl->user_req) {
				rfctl->user_req->src = param->regd_src;
				rfctl->user_req->inr = param->inr;
				#if CONFIG_IEEE80211_BAND_6GHZ
				if (param->env >= RTW_ENV_NUM) /* no env specified by first user_req, get from init_req */
					rfctl->user_req->env = rfctl->init_req.env;
				else {
					rfctl->user_req->env = param->env;
					if (rfctl->init_req.env == param->env)
						param->priv_user_set_bmp &= ~RTW_PRIV_USER_SET_ENV;
				}
				#endif
				rtw_regd_req_list_insert(rfctl, rfctl->user_req);
				effected = true;
			} else
				rtw_warn_on(1);
		} else {
			if (!_rtw_memcmp(&rfctl->user_req->chplan, &param->country_ent, sizeof(param->country_ent))) {
				_rtw_memcpy(&rfctl->user_req->chplan, &param->country_ent, sizeof(param->country_ent));
				effected = true;
			}

			#if CONFIG_IEEE80211_BAND_6GHZ
			if (param->env < RTW_ENV_NUM && rfctl->user_req->env != param->env) {
				rfctl->user_req->env = param->env;
				effected = true;
			} else
				param->priv_user_set_bmp &= ~RTW_PRIV_USER_SET_ENV;
			#endif
		}
	}

#if CONFIG_IEEE80211_BAND_6GHZ
	if (param->priv_user_set_bmp & RTW_PRIV_USER_SET_TXPWR_LMT_6G_CATE) {
		if (rfctl->force_txpwr_lmt_6g_cate != param->force_txpwr_lmt_6g_cate) {
			rfctl->force_txpwr_lmt_6g_cate = param->force_txpwr_lmt_6g_cate;
			effected = true;
		} else
			param->priv_user_set_bmp &= ~RTW_PRIV_USER_SET_TXPWR_LMT_6G_CATE;
	}
#endif

	/* check and update confs */
	rtw_chplan_rtk_priv_req_prehdl_confs(rfctl, &param->confs, caller);

exit:
	return effected || param->confs.set_types;
}

#ifdef CONFIG_80211D
#if CONFIG_IEEE80211_BAND_6GHZ
u8 cisr_get_largest_chplan_6g_cate(const struct country_ie_slave_record *cisr)
{
	u8 cate_6g_map = cisr->chplan.cate_6g_map & reg_info_to_chplan_6g_cate_map(cisr->reg_info);

	if (cate_6g_map) {
		int i;

		for (i = CHPLAN_6G_CATE_NUM - 1; i >= 0; i--)
			if (cate_6g_map & BIT(i))
				break;
		if (i >= 0)
			return i;
	}
	return CHPLAN_6G_CATE_NUM;
}

static u8 cisr_get_txpwr_lmt_6g_cate_map(const struct country_ie_slave_record *cisr)
{
	if (cisr->band == BAND_ON_6G) {
		if (!IS_6G_REG_INFO_RSVD(cisr->reg_info)) {
			u8 chplan_6g_cate = cisr_get_largest_chplan_6g_cate(cisr);

			if (chplan_6g_cate < CHPLAN_6G_CATE_NUM)
				return chplan_6g_cate_to_txpwr_lmt_6g_cate_map(chplan_6g_cate);
		}
		return BIT(TXPWR_LMT_6G_CATE_VLP);
	}
	return 0;
}
#endif

enum cisr_match {
	CISR_MATCH		= 0, /* identically match */
	CISR_MATCH_CHPLAN	= 1, /* same chplan result (including txpwr_lmt_6g_cate_bmp, tpes) */
	CISR_DIFF		/* different (not above cases) */
};

static enum cisr_match rtw_cisr_compare(const struct country_ie_slave_record *a, const struct country_ie_slave_record *b)
{
	if (_rtw_memcmp(a, b, sizeof(*a)) == true)
		return CISR_MATCH;
	if (_rtw_memcmp(&a->chplan, &b->chplan, sizeof(a->chplan)) == true
		#if CONFIG_IEEE80211_BAND_6GHZ
		&& cisr_get_txpwr_lmt_6g_cate_map(a) == cisr_get_txpwr_lmt_6g_cate_map(b)
		&& _rtw_memcmp(&a->tpes, &b->tpes, sizeof(a->tpes)) == true
		#endif
	)
		return CISR_MATCH_CHPLAN;
	return CISR_DIFF;
}

#if CONFIG_IEEE80211_BAND_6GHZ
static void rtw_rfctl_update_link_6g_cate_map(struct rf_ctl_t *rfctl)
{
	if (rfctl->force_txpwr_lmt_6g_cate == TXPWR_LMT_6G_CATE_NONE) {
		int i, j;
		u8 bmp = 0;
		u8 applied_link_num = 0;

		for (i = 0; i < CONFIG_IFACE_NUMBER; i++) {
			for (j = 0; j < RTW_RLINK_MAX; j++) {
				if (!rtw_rfctl_cisr_has_valid_link_6g_cate(rfctl, &rfctl->cisr[i][j]))
					continue;
				bmp |= cisr_get_txpwr_lmt_6g_cate_map(&rfctl->cisr[i][j]);
				applied_link_num++;
			}
		}

		rfctl->txpwr_lmt_link_6g_cate_map = bmp;
		rfctl->txpwr_lmt_link_6g_cate_map_applied_num = applied_link_num;
		return;
	}

	/* below is for force_txpwr_lmt_6g_cate case */
	rfctl->txpwr_lmt_link_6g_cate_map = BIT(rfctl->force_txpwr_lmt_6g_cate);
	rfctl->txpwr_lmt_link_6g_cate_map_applied_num = 255;
}

static void rtw_rfctl_get_txpwr_lmt_6g_cate_map_from_init_user(struct rf_ctl_t *rfctl
	, enum rtw_env_t *r_env, enum country_ie_slave_6g_reg_info *r_reg_info
	, u8 *r_txpwr_lmt_6g_cate_map)
{
	struct regd_req_t *user_req = rfctl->user_req;
	enum rtw_env_t env = RTW_ENV_NUM;
	u8 bmp;

	if (user_req)
		env = user_req->env;
	if (!user_req || rfctl->init_regd_always_apply) {
		if (env == RTW_ENV_NUM || rfctl->init_req.env == RTW_ENV_ANY)
			env = rfctl->init_req.env;
		else if (env != rfctl->init_req.env)
			env = RTW_ENV_ANY;
	}

	bmp = rtw_env_to_txpwr_lmt_6g_cate_map(env);

	*r_env = env;
	*r_reg_info = CIS_6G_REG_RSVD;
	*r_txpwr_lmt_6g_cate_map = bmp;
}
#endif /*CONFIG_IEEE80211_BAND_6GHZ  */

static bool rtw_chplan_update_per_link_cisr(struct rf_ctl_t *rfctl, u8 iface_id
	, u8 cisr_alink_id, struct country_ie_slave_record *cisr)
{
	struct country_ie_slave_record ori_cisr_cont[RTW_RLINK_MAX];
	u8 alink_id_s, alink_id_e, alink_id;
	bool effected = false;

	if (cisr_alink_id < RTW_RLINK_MAX) {
		/* specific alink */
		alink_id_s = cisr_alink_id;
		alink_id_e = alink_id_s + 1;
	} else {
		/* all alinks of specific iface */
		alink_id_s = 0;
		alink_id_e = RTW_RLINK_MAX;
	}

	/* compare original record with same iface_id & spcified alink_id range */
	for (alink_id = alink_id_s; alink_id < alink_id_e; alink_id++)
		if (rtw_cisr_compare(&rfctl->cisr[iface_id][alink_id], cisr) != CISR_MATCH)
			break;
	if (alink_id >= alink_id_e) {
		/* record no change  */
		goto exit;
	}

	/* backup original content */
	for (alink_id = alink_id_s; alink_id < alink_id_e; alink_id++)
		_rtw_memcpy(&ori_cisr_cont[alink_id], &rfctl->cisr[iface_id][alink_id], sizeof(ori_cisr_cont[alink_id]));

	/* update record */
	for (alink_id = alink_id_s; alink_id < alink_id_e; alink_id++)
		_rtw_memcpy(&rfctl->cisr[iface_id][alink_id], cisr, sizeof(*cisr));

	/* compare original record with same iface_id & spcified alink_id range for chplan change */
	for (alink_id = alink_id_s; alink_id < alink_id_e; alink_id++)
		if (rtw_cisr_compare(&ori_cisr_cont[alink_id], cisr) > CISR_MATCH_CHPLAN)
			break;
	if (alink_id >= alink_id_e) {
		/* chplan no change  */
		goto exit;
	}

#if CONFIG_IEEE80211_BAND_6GHZ
	rtw_rfctl_update_link_6g_cate_map(rfctl);
#endif

	effected = true;

exit:
	return effected;
}

static bool rtw_regd_req_list_add_country_ie_req(struct rf_ctl_t *rfctl
	, struct country_ie_slave_record *cisr, bool link)
{
	struct country_chplan *chplan = &cisr->chplan;
	struct regd_req_t *req;
	_list *cur, *head;
	bool insert = false;
#if CONFIG_IEEE80211_BAND_6GHZ
	enum rtw_env_t env;
	enum country_ie_slave_6g_reg_info reg_info;
	u8 txpwr_lmt_6g_cate_map;

	if (link) {
		env = cisr->env;
		reg_info = cisr->reg_info;
		txpwr_lmt_6g_cate_map = cisr_get_txpwr_lmt_6g_cate_map(cisr);
	} else {
		/*  ENV_BSS doesn't affect env and reg_info, assign value acording to per link status or init/user req */
		if (rfctl->txpwr_lmt_link_6g_cate_map_applied_num) {
			env = RTW_ENV_NUM;
			reg_info = CIS_6G_REG_RSVD;
			txpwr_lmt_6g_cate_map = rfctl->txpwr_lmt_link_6g_cate_map;
		} else {
			rtw_rfctl_get_txpwr_lmt_6g_cate_map_from_init_user(rfctl
				, &env, &reg_info, &txpwr_lmt_6g_cate_map);
		}
	}
#endif

	head = &rfctl->regd_req_list;
	cur = get_next(head);
	while (rtw_end_of_queue_search(head, cur) == _FALSE) {
		req = LIST_CONTAINOR(cur, struct regd_req_t, list);
		if (req->inr == RTW_REGD_SET_BY_COUNTRY_IE
			&& _rtw_memcmp(&req->chplan, chplan, sizeof(*chplan)) == true
		) {
			#if CONFIG_IEEE80211_BAND_6GHZ
			/* same chplan, take union of txpwr_lmt_6g_cate_map */
			if (req->ref_cnt == 0) {
				req->_pre_txpwr_lmt_6g_cate_map = req->txpwr_lmt_6g_cate_map;
				req->txpwr_lmt_6g_cate_map = txpwr_lmt_6g_cate_map;
			} else
				req->txpwr_lmt_6g_cate_map |= txpwr_lmt_6g_cate_map;
			#endif
			req->ref_cnt++;
			break;
		}
		cur = get_next(cur);
	}
	if (rtw_end_of_queue_search(head, cur)) {
		req = rtw_regd_req_alloc_with_country_chplan(chplan);
		if (req) {
			req->src = REGD_SRC_RTK_PRIV;
			req->inr = RTW_REGD_SET_BY_COUNTRY_IE;
			#if CONFIG_IEEE80211_BAND_6GHZ
			req->env = env;
			req->reg_info = reg_info;
			req->txpwr_lmt_6g_cate_map = txpwr_lmt_6g_cate_map;
			#endif
			req->ref_cnt = 1;
			rtw_regd_req_list_insert(rfctl, req);
			insert = true;
		} else
			rtw_warn_on(1);
	}

	return insert;
}

static bool rtw_regd_req_list_add_country_ie_req_from_per_link_cisr(struct rf_ctl_t *rfctl)
{
	int i, j;
	bool effected = false;

	for (i = 0; i < CONFIG_IFACE_NUMBER; i++) {
		for (j = 0; j < RTW_RLINK_MAX; j++) {
			if (rfctl->cisr[i][j].status != COUNTRY_IE_SLAVE_APPLICABLE)
				continue;
			effected |= rtw_regd_req_list_add_country_ie_req(rfctl, &rfctl->cisr[i][j], true);
		}
	}

	return effected;
}

static bool rtw_regd_req_list_add_country_ie_req_from_scan_stat_cisr(struct rf_ctl_t *rfctl)
{
	struct cis_scan_stat_t *stat = &rfctl->cis_scan_stat;
	_list *list, *head;
	struct cis_scan_stat_ent *ent;
	bool effected = false;

	_rtw_mutex_lock(&stat->lock);

	/* follow all ENV BSS */
	head = &stat->ent;
	list = get_next(head);
	while (!rtw_end_of_queue_search(head, list)) {
		ent = LIST_CONTAINOR(list, struct cis_scan_stat_ent, list);
		list = get_next(list);

		if (ent->cisr.status == COUNTRY_IE_SLAVE_NOCOUNTRY
			|| ent->cisr.status == COUNTRY_IE_SLAVE_UNKNOWN)
			continue;

		effected |= rtw_regd_req_list_add_country_ie_req(rfctl, &ent->cisr, false);
	}

	_rtw_mutex_unlock(&stat->lock);

	return effected;
}

static void rtw_regd_req_list_clear_ref_cnt_by_inr(struct rf_ctl_t *rfctl, enum rtw_regd_inr inr)
{
	struct regd_req_t *req;
	_list *cur, *head;

	head = &rfctl->regd_req_list;
	cur = get_next(head);
	while (rtw_end_of_queue_search(head, cur) == _FALSE) {
		req = LIST_CONTAINOR(cur, struct regd_req_t, list);
		cur = get_next(cur);
		if (req->inr == inr)
			req->ref_cnt = 0;
	}
}

static bool rtw_regd_req_list_clear_zero_ref_req_by_inr(struct rf_ctl_t *rfctl, enum rtw_regd_inr inr)
{
	struct regd_req_t *req;
	_list *cur, *head;
	bool del = false;

	head = &rfctl->regd_req_list;
	cur = get_next(head);
	while (rtw_end_of_queue_search(head, cur) == _FALSE) {
		req = LIST_CONTAINOR(cur, struct regd_req_t, list);
		cur = get_next(cur);
		if (req->inr == inr && req->ref_cnt == 0) {
			rtw_regd_req_list_delete(rfctl, req);
			rtw_regd_req_free(req);
			del = true;
		}
	}
	return del;
}

#if CONFIG_IEEE80211_BAND_6GHZ
static bool rtw_regd_req_list_chk_txpwr_lmt_6g_cate_map_changed(struct rf_ctl_t *rfctl, enum rtw_regd_inr inr)
{
	struct regd_req_t *req;
	_list *cur, *head;

	head = &rfctl->regd_req_list;
	cur = get_next(head);
	while (rtw_end_of_queue_search(head, cur) == _FALSE) {
		req = LIST_CONTAINOR(cur, struct regd_req_t, list);
		cur = get_next(cur);
		if (req->inr != inr)
			continue;
		if (req->_pre_txpwr_lmt_6g_cate_map != req->txpwr_lmt_6g_cate_map)
			return true;
	}
	return false;
}
#endif

static bool rtw_chplan_rtk_priv_req_prehdl_country_ie(_adapter *adapter, struct SetChannelPlan_param *param, const char *caller)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	struct chplan_confs *confs = &param->confs;
	u16 apply_confs_bmp = BIT(CHPLAN_CONFS_CIS_EN_MODE) | BIT(CHPLAN_CONFS_CIS_FLAGS);
	bool confs_change = confs->set_types & apply_confs_bmp;
	bool effected = false;

	if (param->has_cisr) {
		bool per_link_cisr_changed;

		#ifdef CONFIG_RTW_DEBUG
		if (rtw_drv_log_level >= _DRV_DEBUG_) {
			RTW_PRINT("%s cisr before\n", __func__);
			dump_country_ie_slave_records(RTW_DBGDUMP, rfctl, false, false);
		}
		#endif

		per_link_cisr_changed = rtw_chplan_update_per_link_cisr(rfctl
			, adapter->iface_id, param->cisr_alink_id, &param->cisr);
		if (!per_link_cisr_changed && !confs_change)
			goto exit;

		if (!rfctl->cis_enabled || rfctl->regd_src != REGD_SRC_RTK_PRIV)
			effected = per_link_cisr_changed;
	}

	if (param->has_cisr) {
		#ifdef CONFIG_RTW_DEBUG
		if (rtw_drv_log_level >= _DRV_DEBUG_) {
			RTW_PRINT("%s cisr after\n", __func__);
			dump_country_ie_slave_records(RTW_DBGDUMP, rfctl, false, false);
		}
		#endif
	}

	if (rfctl->regd_src == REGD_SRC_RTK_PRIV) {
		rtw_regd_req_list_clear_ref_cnt_by_inr(rfctl, RTW_REGD_SET_BY_COUNTRY_IE);

		if (rfctl->cis_enabled) {
			if (RFCTL_GET_CIS_MAJORITY(rfctl))
				effected |= rtw_regd_req_list_add_country_ie_req(rfctl, &RFCTL_GET_CIS_MAJORITY(rfctl)->cisr, false);
			else {
				effected |= rtw_regd_req_list_add_country_ie_req_from_per_link_cisr(rfctl);
				if (rfctl->cis_flags & CISF_ENV_BSS)
					effected |= rtw_regd_req_list_add_country_ie_req_from_scan_stat_cisr(rfctl);
			}	
		}

		effected |= rtw_regd_req_list_clear_zero_ref_req_by_inr(rfctl, RTW_REGD_SET_BY_COUNTRY_IE);
		#if CONFIG_IEEE80211_BAND_6GHZ
		effected |= rtw_regd_req_list_chk_txpwr_lmt_6g_cate_map_changed(rfctl, RTW_REGD_SET_BY_COUNTRY_IE);
		#endif
	}
	confs->set_types &= ~apply_confs_bmp;

exit:
	return effected;
}
#endif /* CONFIG_80211D */

#ifdef CONFIG_REGD_SRC_FROM_OS
static bool rtw_chplan_req_prehdl_from_os(_adapter *adapter, struct SetChannelPlan_param *param, const char *caller)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	bool effected = false;
	struct regd_req_t *req;
	_list *cur, *head;

#ifdef CONFIG_80211D
	/* adjust cis settings */
	rfctl->cis_en_mode = rtw_os_get_cis_en_mode(adapter);
	rfctl->cis_flags = 0;
	rfctl->cis_en_role = COUNTRY_IE_SLAVE_EN_ROLE_STA | COUNTRY_IE_SLAVE_EN_ROLE_GC;
	rfctl->cis_en_ifbmp = 0xFF;
	rtw_rfctl_update_cis_enabled(rfctl, false, __func__);
	rtw_rfctl_update_cisr_collect_mode(rfctl, false, __func__);
#endif

	/*
	* two reqs only:
	* 1. init req and
	* 2. another recent req
	*/
	#if CONFIG_IEEE80211_BAND_6GHZ
	if (param->inr == RTW_REGD_SET_BY_USER
		&& param->channel_plan == RTW_CHPLAN_UNSPECIFIED
		&& param->channel_plan_6g == RTW_CHPLAN_6G_UNSPECIFIED
		&& !param->has_country
	) {
		/* special OS-USER req to set default environment */
		if (param->env < RTW_ENV_NUM) {
			bool has_country_ie = false;
			bool non_country_ie_env_changed = false;

			head = &rfctl->regd_req_list;
			cur = get_next(head);
			while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
				req = LIST_CONTAINOR(cur, struct regd_req_t, list);
				cur = get_next(cur);
				if (req->inr != RTW_REGD_SET_BY_COUNTRY_IE) {
					if (req->env != param->env) {
						req->env = param->env;
						non_country_ie_env_changed = true;
					}
				} else
					has_country_ie = true;
			}
			if (!has_country_ie && non_country_ie_env_changed) {
				effected = true;
				goto exit;
			}
		}
		goto exit;
	} else
	#endif
	{
		head = &rfctl->regd_req_list;
		cur = get_next(head);
		while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
			req = LIST_CONTAINOR(cur, struct regd_req_t, list);
			cur = get_next(cur);
			if (req->inr == RTW_REGD_SET_BY_INIT)
				continue;
			if (req->inr == RTW_REGD_SET_BY_USER)
				rfctl->user_req = NULL;
			rtw_regd_req_list_delete(rfctl, req);
			rtw_regd_req_free(req);
		}

		req = rtw_regd_req_alloc_with_country_chplan(&param->country_ent);
		if (req) {
			req->src = param->regd_src;
			req->inr = param->inr;
			#if CONFIG_IEEE80211_BAND_6GHZ
			if (req->inr == RTW_REGD_SET_BY_COUNTRY_IE && param->env < RTW_ENV_NUM)
				req->env = param->env;
			else
				req->env = rfctl->init_req.env;
			#endif
			rtw_regd_req_list_insert(rfctl, req);
			if (req->inr == RTW_REGD_SET_BY_USER)
				rfctl->user_req = req;
		} else {
			rtw_warn_on(1);
			goto exit;
		}
	}

	effected = true;

exit:
	return effected;
}
#endif /* CONFIG_REGD_SRC_FROM_OS */

static bool rtw_chplan_req_prehdl(_adapter *adapter, struct SetChannelPlan_param *param, const char *caller)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	bool effected = false;
	char buf[64];

	get_str_of_set_chplan_keys(buf, 64, param);
	RTW_INFO("%s iface_id:%u src:%s inr:%s %s\n", caller, adapter->iface_id
		, regd_src_str(param->regd_src), regd_inr_str(param->inr), buf);

	if (param->inr == RTW_REGD_SET_BY_INIT) {
		/* init should not done here */
		rtw_warn_on(1);
		goto exit;
	}

	if (param->regd_src == REGD_SRC_RTK_PRIV) {
		if (param->inr == RTW_REGD_SET_BY_USER) {
			#ifdef CONFIG_80211D
			struct chplan_confs *confs = &param->confs;
			#endif

			if (!rtw_chplan_rtk_priv_req_prehdl_user(rfctl, param, __func__))
				goto exit;

			#ifdef CONFIG_80211D
			/*
			* rfctl.regd_src is not allowed to change from REGD_SRC_OS to  REGD_SRC_RTK_PRIV
			* get new status and update to rfctl.cis_enabled here is safe and necessary for logic below
			*/
			if (rtw_rfctl_update_cis_enabled(rfctl, false, __func__)) {
				rtw_rfctl_update_cisr_collect_mode(rfctl, false, __func__);
				/* enable status change */
				confs->set_types |= BIT(CHPLAN_CONFS_CIS_EN_MODE);
			}

			if ((confs->set_types & BIT(CHPLAN_CONFS_CIS_FLAGS))
				|| (confs->set_types & BIT(CHPLAN_CONFS_CIS_EN_MODE))
				|| (param->priv_user_set_bmp & RTW_PRIV_USER_SET_ENV)
			) {
				if (!rtw_chplan_rtk_priv_req_prehdl_country_ie(adapter, param, __func__)
					&& !confs->set_types && !param->priv_user_set_bmp)
					goto exit;
			}
			#endif
		}
		#ifdef CONFIG_80211D
		else if (param->inr == RTW_REGD_SET_BY_COUNTRY_IE) {
			if (!rtw_chplan_rtk_priv_req_prehdl_country_ie(adapter, param, __func__))
				goto exit;
		}
		#endif
		else {
			rtw_warn_on(1);
			goto exit;
		}
	}
#ifdef CONFIG_REGD_SRC_FROM_OS
	else if (param->regd_src == REGD_SRC_OS) {
		if (!rtw_chplan_req_prehdl_from_os(adapter, param, caller))
			goto exit;
	}
#endif
	else {
		rtw_warn_on(1);
		goto exit;
	}

	effected = true;

exit:
#ifdef CONFIG_80211D
	rfctl->init_user_req_is_ww = rtw_rfctl_is_init_user_req_world_wide(rfctl);
	#if CONFIG_IEEE80211_BAND_6GHZ
	rtw_rfctl_update_link_6g_cate_map(rfctl);
	rtw_rfctl_update_regd_req_txpwr_6g_cate_map(rfctl);
	#endif
#endif

	return effected;
}

static void rtw_rfctl_regd_req_sel_and_status_update(struct rf_ctl_t *rfctl)
{
	struct regd_req_t *req;
	_list *cur, *head;
	enum regd_src_t applied_src = REGD_SRC_NUM;
	enum rtw_regd_inr applied_inr = RTW_REGD_SET_BY_NUM;

	/* decide applied_src (highest src) */
	head = &rfctl->regd_req_list;
	cur = get_next(head);
	while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
		req = LIST_CONTAINOR(cur, struct regd_req_t, list);
		cur = get_next(cur);
		if (req->inr == RTW_REGD_SET_BY_EXTRA)
			continue;
		if (applied_src == REGD_SRC_NUM || req->src > applied_src)
			applied_src = req->src;
	}

	/* decide applied_inr (highest inr) */
	head = &rfctl->regd_req_list;
	cur = get_next(head);
	while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
		req = LIST_CONTAINOR(cur, struct regd_req_t, list);
		cur = get_next(cur);
		if (req->src != applied_src)
			continue;
		if (req->inr == RTW_REGD_SET_BY_EXTRA)
			continue;
		if (applied_inr == RTW_REGD_SET_BY_NUM || req->inr > applied_inr)
			applied_inr = req->inr;
	}

	rfctl->regd_src = REGD_SRC_NUM;
	head = &rfctl->regd_req_list;
	cur = get_next(head);
	while ((rtw_end_of_queue_search(head, cur)) == _FALSE) {
		req = LIST_CONTAINOR(cur, struct regd_req_t, list);
		cur = get_next(cur);
		if (req->inr == RTW_REGD_SET_BY_EXTRA)
			continue;

		/* mark req with different src as not applied*/
		if (applied_src != req->src) {
			req->applied = false;
			continue;
		}
		/* mark req with target inr or always_apply INIT/USER req as applied*/
		req->applied = applied_inr == req->inr
			|| (req->inr == RTW_REGD_SET_BY_INIT && rfctl->init_regd_always_apply)
			|| (req->inr == RTW_REGD_SET_BY_USER && rfctl->user_regd_always_apply);
		if (!req->applied)
			continue;

		rtw_rfctl_regd_status_update_by_req(rfctl, req);
	}

	if (rfctl->extra_req) {
		rfctl->extra_req->applied = rtw_rfctl_extra_alpha2_req_needed(rfctl);
		if (rfctl->extra_req->applied)
			rtw_rfctl_regd_status_update_by_req(rfctl, rfctl->extra_req);
	}

	#ifdef CONFIG_80211D
	if (rtw_rfctl_update_cis_enabled(rfctl, false, __func__))
		rtw_rfctl_update_cisr_collect_mode(rfctl, false, __func__);
	#endif
}

#if defined(CONFIG_AP_MODE) && CONFIG_AP_REGU_FORBID
bool rtw_rfctl_is_regu_forbid_bss(struct rf_ctl_t *rfctl, enum band_type band)
{
	if (!rtw_txpwr_hal_is_txpwr_limit_needed(rfctl_to_dvobj(rfctl)))
		return false;

	/* forbid before 6G AP regulatory ready */
	return band == BAND_ON_6G;
}

static void rtw_bss_regu_status_update(struct rf_ctl_t *rfctl)
{
	struct dvobj_priv *dvobj = rfctl_to_dvobj(rfctl);
	u8 band_idx;

	/* TODO: update possible AP cate */

	for (band_idx = HW_BAND_0; band_idx < HW_BAND_MAX; band_idx++) {
		_adapter *iface;
		struct _ADAPTER_LINK *alink;
		int i, j;
		bool forbid;
		u8 ifbmp_m = rtw_mi_get_ap_mesh_ifbmp_by_hwband(dvobj, band_idx);
		u8 forbid_cnt = 0;
		bool sta_link = false;

		/* loop all bss to apply regu status (reg_info, forbid) */
		for (i = 0; i < dvobj->iface_nums; i++) {
			iface = dvobj->padapters[i];
			if (!iface || !(ifbmp_m & BIT(iface->iface_id)))
				continue;

			for (j = 0; j < ADAPTER_GET_LINK_NUM(iface); j++) {
				alink = GET_LINK(iface, j);
				if (ALINK_GET_HWBAND(alink) != band_idx)
					continue;

				forbid = rtw_rfctl_is_regu_forbid_bss(rfctl, ALINK_GET_BAND(alink));
				if (forbid)
					forbid_cnt++;

				/* TODO: update reg_info for 6G AP */

				/* update forbid status */
				rtw_ap_link_regu_forbid_apply(alink, forbid, false);
			}
		}

		if (!forbid_cnt || DEV_MCC_CAPABLE(dvobj))
			continue;

		/* check if having sta link exist */
		for (i = 0; i < dvobj->iface_nums; i++) {
			iface = dvobj->padapters[i];
			if (!iface || (ifbmp_m & BIT(iface->iface_id)) || !rtw_is_adapter_up(iface))
				continue;

			for (j = 0; j < ADAPTER_GET_LINK_NUM(iface); j++) {
				alink = GET_LINK(iface, j);
				if (ALINK_GET_HWBAND(alink) != band_idx)
					continue;
				if (rfctl->cisr[i][j].opch) {
					sta_link = true;
					i = dvobj->iface_nums;
					break;
				}
			}
		}

		if (!sta_link) {
			/* no sta link, trigger bss ch select to get out of forbid status */
			struct _ADAPTER_LINK *m_iflink = rtw_mi_get_ap_mesh_iflink_by_hwband(dvobj, band_idx);
			u8 ifbmp_s = rtw_mi_get_lgd_sta_ifbmp_by_hwband(dvobj, band_idx);

			RTW_INFO(FUNC_HWBAND_FMT" trigger ch select for forbid bss\n", FUNC_HWBAND_ARG(band_idx));

			if (!m_iflink) {
				rtw_warn_on(1);
				continue;
			}

			rtw_change_bss_bchbw_cmd(m_iflink->adapter, RTW_CMDF_DIRECTLY
				, ifbmp_m, ifbmp_s, REQ_BAND_NONE, REQ_CH_NONE, REQ_BW_ORI, REQ_OFFSET_NONE);
		}
	}
}
#endif /* defined(CONFIG_AP_MODE) && CONFIG_AP_REGU_FORBID */

u8 rtw_set_chplan_hdl(_adapter *adapter, u8 *pbuf)
{
	struct SetChannelPlan_param *param;
	struct dvobj_priv *dvobj = adapter_to_dvobj(adapter);
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
#ifdef CONFIG_IOCTL_CFG80211
	struct get_chplan_resp *chplan;
#endif

	if (!pbuf)
		return H2C_PARAMETERS_ERROR;

	param = (struct SetChannelPlan_param *)pbuf;

	_rtw_mutex_lock_interruptible(&rfctl->regd_req_mutex);

	if (!rtw_chplan_req_prehdl(adapter, param, __func__)) {
		_rtw_mutex_unlock(&rfctl->regd_req_mutex);
		goto exit;
	}

	rtw_rfctl_regd_req_sel_and_status_update(rfctl);

#if CONFIG_TXPWR_LIMIT
	rtw_txpwr_update_cur_lmt_regs(dvobj, false);
	#if CONFIG_IEEE80211_BAND_6GHZ
	rtw_txpwr_hal_set_tpe_infos(dvobj);
	#endif
#endif

	rtw_edcca_mode_update(dvobj, false);

	rtw_rfctl_chset_apply_regulatory(dvobj, false);
#ifdef CONFIG_80211D
	rtw_rfctl_update_cisr_collect_mode(rfctl, false, __func__);
#endif

	_rtw_mutex_unlock(&rfctl->regd_req_mutex);

	op_class_pref_apply_regulatory(rfctl, REG_CHANGE);
	init_channel_list(adapter);

#ifdef CONFIG_IOCTL_CFG80211
	if (rtw_get_chplan_cmd(adapter, RTW_CMDF_DIRECTLY, &chplan) == _SUCCESS) {
		if (!param->rtnl_lock_needed)
			rtw_regd_change_complete_sync(adapter_to_wiphy(adapter), chplan, 0);
		else
			rtw_warn_on(rtw_regd_change_complete_async(adapter_to_wiphy(adapter), chplan) != _SUCCESS);
	} else
		rtw_warn_on(1);
#endif

	rtw_nlrtw_reg_change_event(adapter);

	if (rtw_txpwr_hal_get_pwr_lmt_en(dvobj) && rtw_hw_is_init_completed(dvobj))
		rtw_update_txpwr_level(dvobj, HW_BAND_MAX);

exit:

#if defined(CONFIG_AP_MODE) && CONFIG_AP_REGU_FORBID
	rtw_bss_regu_status_update(rfctl);
#endif

	return	H2C_SUCCESS;
}

static u8 _rtw_set_chplan_cmd(_adapter *adapter, int flags
	, u8 chplan, u8 chplan_6g, const struct country_chplan *country_ent
	, enum rtw_env_t env, enum txpwr_lmt_6g_cate_t force_txpwr_lmt_6g_cate
	, enum regd_src_t regd_src, enum rtw_regd_inr inr
	, const struct country_ie_slave_record *cisr, u8 cisr_alink_id
	, struct chplan_confs *confs)
{
	struct cmd_obj *cmdobj;
	struct SetChannelPlan_param *parm;
	struct cmd_priv *pcmdpriv = &adapter_to_dvobj(adapter)->cmdpriv;
	struct submit_ctx sctx;
#ifdef PLATFORM_LINUX
	bool rtnl_lock_needed = rtw_rtnl_lock_needed(adapter_to_dvobj(adapter));
#endif
	u8 res = _SUCCESS;

	/* check if allow software config */
	if (rtw_rfctl_is_disable_sw_channel_plan(adapter_to_dvobj(adapter)) == _TRUE) {
		res = _FAIL;
		goto exit;
	}

	if (country_ent) {
		/* if country_entry is provided, replace chplan */
		chplan = country_ent->domain_code;
		#if CONFIG_IEEE80211_BAND_6GHZ
		chplan_6g = country_ent->domain_code_6g;
		#endif
	}

	/* prepare cmd parameter */
	parm = rtw_zmalloc(sizeof(*parm));
	if (parm == NULL) {
		res = _FAIL;
		goto exit;
	}
	parm->regd_src = regd_src;
	parm->inr = inr;
	if (country_ent) {
		_rtw_memcpy(&parm->country_ent, country_ent, sizeof(parm->country_ent));
		parm->has_country = true;
	}
	parm->channel_plan = chplan;
#if CONFIG_IEEE80211_BAND_6GHZ
	parm->channel_plan_6g = chplan_6g;
	parm->env = env;
	parm->force_txpwr_lmt_6g_cate = force_txpwr_lmt_6g_cate;
#endif
#ifdef CONFIG_80211D
	if (cisr) {
		_rtw_memcpy(&parm->cisr, cisr, sizeof(*cisr));
		parm->cisr_alink_id = cisr_alink_id;
		parm->has_cisr = true;
	}
#endif

	if (confs)
		_rtw_memcpy(&parm->confs, confs, sizeof(parm->confs));

#ifdef PLATFORM_LINUX
	if (flags & (RTW_CMDF_DIRECTLY | RTW_CMDF_WAIT_ACK))
		parm->rtnl_lock_needed = rtnl_lock_needed; /* synchronous call, follow caller's */
	else
		parm->rtnl_lock_needed = 1; /* asynchronous call, always needed */
#endif

	if (flags & RTW_CMDF_DIRECTLY) {
		/* no need to enqueue, do the cmd hdl directly and free cmd parameter */
		if (H2C_SUCCESS != rtw_set_chplan_hdl(adapter, (u8 *)parm))
			res = _FAIL;
		rtw_mfree(parm, sizeof(*parm));
	} else {
		/* need enqueue, prepare cmd_obj and enqueue */
		cmdobj = rtw_zmalloc(sizeof(*cmdobj));
		if (cmdobj == NULL) {
			res = _FAIL;
			rtw_mfree(parm, sizeof(*parm));
			goto exit;
		}
		cmdobj->padapter = adapter;

		init_h2fwcmd_w_parm_no_rsp(cmdobj, parm, CMD_SET_CHANPLAN);
		CMD_OBJ_SET_HWBAND(cmdobj, HW_BAND_0);

		if (flags & RTW_CMDF_WAIT_ACK) {
			cmdobj->sctx = &sctx;
			rtw_sctx_init(&sctx, 2000);
		}

		res = rtw_enqueue_cmd(pcmdpriv, cmdobj);

		if (res == _SUCCESS && (flags & RTW_CMDF_WAIT_ACK)) {
			rtw_sctx_wait(&sctx, __func__);
			_rtw_mutex_lock_interruptible(&pcmdpriv->sctx_mutex);
			if (sctx.status == RTW_SCTX_SUBMITTED)
				cmdobj->sctx = NULL;
			_rtw_mutex_unlock(&pcmdpriv->sctx_mutex);
			if (sctx.status != RTW_SCTX_DONE_SUCCESS)
				res = _FAIL;
		}

		/* allow set channel plan when cmd_thread is not running */
		if (res != _SUCCESS && (flags & RTW_CMDF_WAIT_ACK)) {
			parm = rtw_zmalloc(sizeof(*parm));
			if (parm == NULL) {
				res = _FAIL;
				goto exit;
			}
			parm->regd_src = regd_src;
			parm->inr = inr;
			if (country_ent) {
				_rtw_memcpy(&parm->country_ent, country_ent, sizeof(parm->country_ent));
				parm->has_country = true;
			}
			parm->channel_plan = chplan;
			#if CONFIG_IEEE80211_BAND_6GHZ
			parm->channel_plan_6g = chplan_6g;
			parm->env = env;
			parm->force_txpwr_lmt_6g_cate = force_txpwr_lmt_6g_cate;
			#endif
			#ifdef CONFIG_80211D
			if (cisr) {
				_rtw_memcpy(&parm->cisr, cisr, sizeof(*cisr));
				parm->cisr_alink_id = cisr_alink_id;
				parm->has_cisr = true;
			}
			#endif
			if (confs)
				_rtw_memcpy(&parm->confs, confs, sizeof(parm->confs));
			#ifdef PLATFORM_LINUX
			parm->rtnl_lock_needed = rtnl_lock_needed; /* synchronous call, follow caller's */
			#endif

			if (H2C_SUCCESS != rtw_set_chplan_hdl(adapter, (u8 *)parm))
				res = _FAIL;
			else
				res = _SUCCESS;
			rtw_mfree(parm, sizeof(*parm));
		}
	}

exit:
	return res;
}

u8 rtw_set_chplan_cmd(_adapter *adapter, int flags, u8 chplan, u8 chplan_6g
	, enum rtw_env_t env, enum rtw_regd_inr inr)
{
	return _rtw_set_chplan_cmd(adapter, flags
		, chplan, chplan_6g, NULL
		, env, TXPWR_LMT_6G_CATE_NUM
		, REGD_SRC_RTK_PRIV, inr
		, NULL, RTW_RLINK_MAX
		, NULL);
}

u8 rtw_set_country_cmd(_adapter *adapter, int flags, const char *country_code
	, enum rtw_env_t env, enum rtw_regd_inr inr)
{
	struct country_chplan ent;

	if (IS_ALPHA2_WORLDWIDE(country_code)) {
		rtw_get_chplan_worldwide(&ent);
		goto cmd;
	}

	if (is_alpha(country_code[0]) == _FALSE
	    || is_alpha(country_code[1]) == _FALSE
	   ) {
		RTW_PRINT("%s input country_code is not alpha2\n", __func__);
		return _FAIL;
	}

	if (!rtw_get_chplan_from_country(country_code, &ent)) {
		RTW_PRINT("%s unsupported country_code:\"%c%c\"\n", __func__, country_code[0], country_code[1]);
		return _FAIL;
	}

cmd:
	RTW_PRINT("%s country_code:\"%c%c\"\n", __func__, country_code[0], country_code[1]);

	return _rtw_set_chplan_cmd(adapter, flags
		, RTW_CHPLAN_UNSPECIFIED, RTW_CHPLAN_6G_UNSPECIFIED, &ent
		, env, TXPWR_LMT_6G_CATE_NUM
		, REGD_SRC_RTK_PRIV, inr
		, NULL, RTW_RLINK_MAX
		, NULL);
}

#if CONFIG_IEEE80211_BAND_6GHZ
u8 rtw_set_env_cmd(_adapter *adapter, int flags, enum rtw_env_t env
	, enum regd_src_t regd_src, enum rtw_regd_inr inr)
{
	return _rtw_set_chplan_cmd(adapter, flags
		, RTW_CHPLAN_UNSPECIFIED, RTW_CHPLAN_6G_UNSPECIFIED, NULL
		, env, TXPWR_LMT_6G_CATE_NUM
		, regd_src, inr
		, NULL, RTW_RLINK_MAX
		, NULL);
}

u8 rtw_set_force_txpwr_lmt_6g_cate_cmd(_adapter *adapter, int flags
	, enum txpwr_lmt_6g_cate_t force_txpwr_lmt_6g_cate
	, enum regd_src_t regd_src, enum rtw_regd_inr inr)
{
	return _rtw_set_chplan_cmd(adapter, flags
		, RTW_CHPLAN_UNSPECIFIED, RTW_CHPLAN_6G_UNSPECIFIED, NULL
		, RTW_ENV_NUM, force_txpwr_lmt_6g_cate
		, regd_src, inr
		, NULL, RTW_RLINK_MAX
		, NULL);
}
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

u8 rtw_set_chplan_confs_cmd(_adapter *adapter, int flags, struct chplan_confs *confs)
{
	return _rtw_set_chplan_cmd(adapter, flags
		, RTW_CHPLAN_UNSPECIFIED, RTW_CHPLAN_6G_UNSPECIFIED, NULL
		, RTW_ENV_NUM, TXPWR_LMT_6G_CATE_NUM
		, REGD_SRC_RTK_PRIV, RTW_REGD_SET_BY_USER
		, NULL, RTW_RLINK_MAX
		, confs);
}

#ifdef CONFIG_PROC_DEBUG
static const char *const chplan_confs_type_str[] = {
	[CHPLAN_CONFS_DIS_CH_FLAGS]		= "dis_ch_flags",
	[CHPLAN_CONFS_EXCL_CHS]			= "excl_chs",
	[CHPLAN_CONFS_EXCL_CHS_6G]		= "excl_chs_6g",
	[CHPLAN_CONFS_INIT_REGD_ALWAYS_APPLY]	= "init_regd_always_apply",
	[CHPLAN_CONFS_USER_REGD_ALWAYS_APPLY]	= "user_regd_always_apply",
	[CHPLAN_CONFS_EXTRA_ALPHA2]		= "extra_alpha2",
	[CHPLAN_CONFS_BCN_HINT_VALID_MS]	= "bcn_hint_valid_ms",
	[CHPLAN_CONFS_CIS_EN_MODE]		= "cis_en_mode",
	[CHPLAN_CONFS_CIS_FLAGS]		= "cis_flags",
	[CHPLAN_CONFS_CIS_EN_IFBMP]		= "cis_en_ifbmp",
	[CHPLAN_CONFS_CIS_EN_ROLE]		= "cis_en_role",
	[CHPLAN_CONFS_CIS_SCAN_BAND_BMP]	= "cis_scan_band_bmp",
	[CHPLAN_CONFS_CIS_SCAN_INT_MS]		= "cis_scan_int_ms",
	[CHPLAN_CONFS_CIS_SCAN_URGENT_MS]	= "cis_scan_urgent_ms",
};

static enum chplan_confs_type get_chplan_confs_type_from_str(const char *str, size_t str_len)
{
	u8 i;

	for (i = 0; i < CHPLAN_CONFS_NUM; i++)
		if (str_len == strlen(chplan_confs_type_str[i])
			&& strncmp(str, chplan_confs_type_str[i], str_len) == 0)
			return i;
	return CHPLAN_CONFS_NUM;
}

static void rtw_parse_chplan_confs_type_arg_str(struct chplan_confs *confs, enum chplan_confs_type type, char *str)
{
	if (type == CHPLAN_CONFS_DIS_CH_FLAGS) {
		confs->set_types |= BIT(type);
		if (strlen(str)) {
			char *c;
			enum rtw_ch_type ch_type;

			for (c = strsep(&str, ","); c; c = strsep(&str, ",")) {
				ch_type = get_ch_type_from_str(c, strlen(c));
				if (ch_type != RTW_CHT_NUM)
					confs->dis_ch_flags |= BIT(ch_type);
			}
		}

	} else if (type == CHPLAN_CONFS_EXCL_CHS) {
		confs->set_types |= BIT(type);
		if (strlen(str)) {
			char *c;
			int ch, ch_num = 0;

			for (c = strsep(&str, ","); c && ch_num <  MAX_CHANNEL_NUM_2G_5G; c = strsep(&str, ",")) {
				if (sscanf(c, "%d", &ch) == 1 && ch > 0 && ch < 256)
					confs->excl_chs[ch_num++] = ch;
			}
		}

#if CONFIG_IEEE80211_BAND_6GHZ
	} else if (type == CHPLAN_CONFS_EXCL_CHS_6G) {
		confs->set_types |= BIT(type);
		if (strlen(str)) {
			char *c;
			int ch, ch_num = 0;

			for (c = strsep(&str, ","); c && ch_num < MAX_CHANNEL_NUM_6G; c = strsep(&str, ",")) {
				if (sscanf(c, "%d", &ch) == 1 && ch > 0 && ch < 256)
					confs->excl_chs_6g[ch_num++] = ch;
			}
		}
#endif

	} else if (type == CHPLAN_CONFS_INIT_REGD_ALWAYS_APPLY) {
		if (strlen(str)) {
			int val;

			if (sscanf(str, "%d", &val) == 1) {
				confs->set_types |= BIT(type);
				confs->init_regd_always_apply = !!val;
			}
		}

	} else if (type == CHPLAN_CONFS_USER_REGD_ALWAYS_APPLY) {
		if (strlen(str)) {
			int val;

			if (sscanf(str, "%d", &val) == 1) {
				confs->set_types |= BIT(type);
				confs->user_regd_always_apply = !!val;
			}
		}

	} else if (type == CHPLAN_CONFS_BCN_HINT_VALID_MS) {
		if (strlen(str)) {
			u32 ms;

			if (sscanf(str, "%u", &ms) == 1) {
				confs->set_types |= BIT(type);
				confs->bcn_hint_valid_ms = ms;
			}
		}

	} else if (type == CHPLAN_CONFS_EXTRA_ALPHA2) {
		confs->set_types |= BIT(type);
		if (strlen(str)) {
			char alpha2[2];

			if (sscanf(str, "%c%c", &alpha2[0], &alpha2[1]) == 2) {
				confs->extra_alpha2[0] = alpha2[0];
				confs->extra_alpha2[1] = alpha2[1];
			}
		}

#ifdef CONFIG_80211D
	} else if (type == CHPLAN_CONFS_CIS_EN_MODE) {
		if (strlen(str)) {
			u8 mode;

			if (sscanf(str, "%hhu", &mode) == 1) {
				confs->set_types |= BIT(type);
				confs->cis_en_mode = mode;
			}
		}

	} else if (type == CHPLAN_CONFS_CIS_FLAGS) {
		if (strlen(str)) {
			u8 flags;

			if (sscanf(str, "%hhx", &flags) == 1) {
				confs->set_types |= BIT(type);
				confs->cis_flags = flags;
			}
		}

	} else if (type == CHPLAN_CONFS_CIS_EN_ROLE) {
		if (strlen(str)) {
			u8 role;

			if (sscanf(str, "%hhx", &role) == 1) {
				confs->set_types |= BIT(type);
				confs->cis_en_role = role;
			}
		}

	} else if (type == CHPLAN_CONFS_CIS_EN_IFBMP) {
		if (strlen(str)) {
			u8 ifbmp;

			if (sscanf(str, "%hhx", &ifbmp) == 1) {
				confs->set_types |= BIT(type);
				confs->cis_en_ifbmp = ifbmp;
			}
		}

	} else if (type == CHPLAN_CONFS_CIS_SCAN_BAND_BMP) {
		if (strlen(str)) {
			u8 bmp;

			if (sscanf(str, "%hhx", &bmp) == 1) {
				confs->set_types |= BIT(type);
				confs->cis_scan_band_bmp = bmp;
			}
		}

	} else if (type == CHPLAN_CONFS_CIS_SCAN_INT_MS) {
		if (strlen(str)) {
			u32 ms;

			if (sscanf(str, "%u", &ms) == 1) {
				confs->set_types |= BIT(type);
				confs->cis_scan_int_ms = ms;
			}
		}

	} else if (type == CHPLAN_CONFS_CIS_SCAN_URGENT_MS) {
		if (strlen(str)) {
			u32 ms;

			if (sscanf(str, "%u", &ms) == 1) {
				confs->set_types |= BIT(type);
				confs->cis_scan_urgent_ms = ms;
			}
		}
#endif /* CONFIG_80211D */

	}
}

u16 rtw_parse_chplan_confs_cmd_str(struct chplan_confs *confs, char *str)
{
	char *next = str, *c, *equal;
	enum chplan_confs_type type;

	_rtw_memset(confs, 0, sizeof(*confs));

	for (c = strsep(&next, " \t\n\r"); c; c = strsep(&next, " \t\n\r")) {
		equal = strchr(c, '=');
		if (!equal || c == equal)
			continue;
		type = get_chplan_confs_type_from_str(c, equal - c);
		if (type == CHPLAN_CONFS_NUM)
			continue;
		rtw_parse_chplan_confs_type_arg_str(confs, type, equal + 1);
	}

	return confs->set_types;
}
#endif /* CONFIG_PROC_DEBUG */

#ifdef CONFIG_80211D
static u8 rtw_alink_apply_recv_cisr_cmd(struct _ADAPTER_LINK *alink, int flags, struct country_ie_slave_record *cisr)
{
	return _rtw_set_chplan_cmd(alink->adapter, flags
		, RTW_CHPLAN_UNSPECIFIED, RTW_CHPLAN_6G_UNSPECIFIED, NULL
		, RTW_ENV_NUM, TXPWR_LMT_6G_CATE_NUM
		, REGD_SRC_RTK_PRIV, RTW_REGD_SET_BY_COUNTRY_IE
		, cisr, rtw_adapter_link_get_id(alink)
		, NULL);
}

static u8 rtw_apply_recv_cisr_cmd(_adapter *adapter, int flags, struct country_ie_slave_record *cisr)
{
	return _rtw_set_chplan_cmd(adapter, flags
		, RTW_CHPLAN_UNSPECIFIED, RTW_CHPLAN_6G_UNSPECIFIED, NULL
		, RTW_ENV_NUM, TXPWR_LMT_6G_CATE_NUM
		, REGD_SRC_RTK_PRIV, RTW_REGD_SET_BY_COUNTRY_IE
		, cisr, RTW_RLINK_MAX
		, NULL);
}

u8 rtw_alink_apply_recv_regu_parm_cmd(struct _ADAPTER_LINK *alink, int flags, enum band_type band,u8 opch
	, const u8 *country_str, enum country_ie_slave_6g_reg_info reg_info, struct rtw_tpe_t *tpes)
{
	struct country_ie_slave_record cisr;

	rtw_get_cisr_from_regu_settings(adapter_to_rfctl(alink->adapter), band, opch
		, country_str, reg_info, tpes
		, &cisr);

	return rtw_alink_apply_recv_cisr_cmd(alink, flags, &cisr);
}

u8 rtw_apply_recv_regu_parm_cmd(_adapter *adapter, int flags, enum band_type band,u8 opch
	, const u8 *country_str, enum country_ie_slave_6g_reg_info reg_info, struct rtw_tpe_t *tpes)
{
	struct country_ie_slave_record cisr;

	rtw_get_cisr_from_regu_settings(adapter_to_rfctl(adapter), band, opch
		, country_str, reg_info, tpes
		, &cisr);

	return rtw_apply_recv_cisr_cmd(adapter, flags, &cisr);
}

u8 rtw_alink_clear_recv_regu_cmd(struct _ADAPTER_LINK *alink, int flags)
{
	struct country_ie_slave_record cisr;

	rtw_get_cisr_from_regu_settings(adapter_to_rfctl(alink->adapter), 0, 0
		, NULL, CIS_6G_REG_RSVD, NULL
		, &cisr);

	return rtw_alink_apply_recv_cisr_cmd(alink, flags, &cisr);
}

u8 rtw_clear_recv_regu_cmd(_adapter *adapter, int flags)
{
	struct country_ie_slave_record cisr;

	rtw_get_cisr_from_regu_settings(adapter_to_rfctl(adapter), 0, 0
		, NULL, CIS_6G_REG_RSVD, NULL
		, &cisr);

	return rtw_apply_recv_cisr_cmd(adapter, flags, &cisr);
}

u8 rtw_apply_scan_network_country_ie_cmd(_adapter *adapter, int flags)
{
	return _rtw_set_chplan_cmd(adapter, flags
		, RTW_CHPLAN_UNSPECIFIED, RTW_CHPLAN_6G_UNSPECIFIED, NULL
		, RTW_ENV_NUM, TXPWR_LMT_6G_CATE_NUM
		, REGD_SRC_RTK_PRIV, RTW_REGD_SET_BY_COUNTRY_IE
		, NULL, RTW_RLINK_MAX
		, NULL);
}
#endif /* CONFIG_80211D */

#ifdef CONFIG_REGD_SRC_FROM_OS
u8 rtw_sync_os_regd_cmd(_adapter *adapter, int flags, const char *country_code
	, u8 dfs_region, enum rtw_env_t env, enum rtw_regd_inr inr)
{
	struct country_chplan ent;
	struct country_chplan rtk_ent;
	bool rtk_ent_exist;

	rtk_ent_exist = rtw_get_chplan_from_country(country_code, &rtk_ent);

	_rtw_memcpy(ent.alpha2, country_code, 2);

	/*
	* Regulation follows OS, the internal txpwr limit selection is searched by alpha2
	*     "00" => WW, others use string mapping
	* When  no matching txpwr limit selection is found, use
	*     1. txpwr lmit selection associated with alpha2 inside driver regulation database
	*     2. WW when driver has no support of this alpha2
	*/

	ent.domain_code = rtk_ent_exist ? rtk_ent.domain_code : RTW_CHPLAN_UNSPECIFIED;
	#if CONFIG_IEEE80211_BAND_6GHZ
	ent.domain_code_6g = rtk_ent_exist ? rtk_ent.domain_code_6g : RTW_CHPLAN_6G_UNSPECIFIED;
	#endif

	#if CONFIG_COUNTRY_CHPLAN_EDCCA_OVERRIDE
	ent.edcca_2g_override = rtk_ent_exist ? rtk_ent.edcca_2g_override : RTW_EDCCA_ADAPT;
	#if CONFIG_IEEE80211_BAND_5GHZ
	ent.edcca_5g_override = rtk_ent_exist ? rtk_ent.edcca_5g_override : RTW_EDCCA_ADAPT;
	#endif
	#if CONFIG_IEEE80211_BAND_6GHZ
	ent.edcca_6g_override = rtk_ent_exist ? rtk_ent.edcca_6g_override : RTW_EDCCA_ADAPT;
	#endif
	#endif

	ent.txpwr_lmt_override = rtk_ent_exist ? rtk_ent.txpwr_lmt_override : TXPWR_LMT_DEF;
	#ifdef CONFIG_CHPLAN_PROTO_EN
	ent.proto_en = CHPLAN_PROTO_EN_ALL;
	#endif

	/* TODO: dfs_region */

	return _rtw_set_chplan_cmd(adapter, flags
		, RTW_CHPLAN_UNSPECIFIED, RTW_CHPLAN_6G_UNSPECIFIED, &ent
		, env, TXPWR_LMT_6G_CATE_NUM
		, REGD_SRC_OS, inr
		, NULL, RTW_RLINK_MAX
		, NULL);
}
#endif /* CONFIG_REGD_SRC_FROM_OS */

u8 rtw_get_chplan_hdl(_adapter *adapter, u8 *pbuf)
{
	struct get_channel_plan_param *param;
	struct get_chplan_resp *chplan;
	struct rf_ctl_t *rfctl;
	struct rtw_chset *chset;
#if CONFIG_TXPWR_LIMIT
	char *tl_reg_names[BAND_MAX];
	int tl_reg_names_len[BAND_MAX];
#endif
	int tl_reg_names_len_total = 0;
	int i;

	if (!pbuf)
		return H2C_PARAMETERS_ERROR;

	rfctl = adapter_to_rfctl(adapter);
	chset = adapter_to_chset(adapter);
	param = (struct get_channel_plan_param *)pbuf;

#if CONFIG_TXPWR_LIMIT
	rtw_txpwr_hal_get_current_lmt_regs_name(adapter_to_dvobj(adapter), tl_reg_names, tl_reg_names_len);
	for (i = 0; i < BAND_MAX; i++)
		tl_reg_names_len_total += tl_reg_names_len[i];
#endif

	chplan = rtw_vmalloc(sizeof(struct get_chplan_resp) + sizeof(RT_CHANNEL_INFO) * chset->chs_len + tl_reg_names_len_total);
	if (!chplan)
		return H2C_CMD_FAIL;

	chplan->regd_src = rfctl->regd_src;
	chplan->regd_inr_bmp = rfctl->regd_inr_bmp;

	chplan->alpha2[0] = rfctl->alpha2[0];
	chplan->alpha2[1] = rfctl->alpha2[1];

	chplan->channel_plan = rfctl->domain_code;
#if CONFIG_IEEE80211_BAND_6GHZ
	chplan->chplan_6g = rfctl->domain_code_6g;
	chplan->env_bmp = rfctl->env_bmp;
	chplan->force_txpwr_lmt_6g_cate = rfctl->force_txpwr_lmt_6g_cate;
#endif
#if CONFIG_TXPWR_LIMIT
	chplan->txpwr_lmt_names_len_total = tl_reg_names_len_total;
	for (i = 0; i < BAND_MAX; i++) {
		if (i == 0)
			chplan->txpwr_lmt_names[i] = ((u8 *)(chplan->chs)) + sizeof(RT_CHANNEL_INFO) * chset->chs_len;
		else
			chplan->txpwr_lmt_names[i] = chplan->txpwr_lmt_names[i - 1] + chplan->txpwr_lmt_names_len[i - 1];

		chplan->txpwr_lmt_names_len[i] = tl_reg_names_len[i];
		if (tl_reg_names[i] && tl_reg_names_len[i]) {
			_rtw_memcpy((void *)chplan->txpwr_lmt_names[i], tl_reg_names[i], tl_reg_names_len[i]);
			rtw_mfree(tl_reg_names[i], tl_reg_names_len[i]);
		}
	}
#endif
	chplan->edcca_mode_2g = rfctl->edcca_mode_2g;
#if CONFIG_IEEE80211_BAND_5GHZ
	chplan->edcca_mode_5g = rfctl->edcca_mode_5g;
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	chplan->edcca_mode_6g = rfctl->edcca_mode_6g;
#endif
#ifdef CONFIG_DFS_MASTER
	chplan->dfs_domain = rtw_rfctl_get_dfs_domain(rfctl);
#endif

	chplan->proto_en = 0
		#ifdef CONFIG_CHPLAN_PROTO_EN
		| rfctl->proto_en
		#endif
		;

	chplan->confs.dis_ch_flags = rfctl->dis_ch_flags;
	_rtw_memcpy(chplan->confs.excl_chs, rfctl->excl_chs, MAX_CHANNEL_NUM_2G_5G);
#if CONFIG_IEEE80211_BAND_6GHZ
	_rtw_memcpy(chplan->confs.excl_chs_6g, rfctl->excl_chs_6g, MAX_CHANNEL_NUM_6G);
#endif
	chplan->confs.init_regd_always_apply = rfctl->init_regd_always_apply;
	chplan->confs.user_regd_always_apply = rfctl->user_regd_always_apply;
	chplan->confs.extra_alpha2[0] = rfctl->extra_req ? rfctl->extra_req->chplan.alpha2[0] : '\0';
	chplan->confs.extra_alpha2[1] = rfctl->extra_req ? rfctl->extra_req->chplan.alpha2[1] : '\0';
	chplan->confs.bcn_hint_valid_ms = rfctl->bcn_hint_valid_ms;
#ifdef CONFIG_80211D
	chplan->confs.cis_en_mode = rfctl->cis_en_mode;
	chplan->confs.cis_flags = rfctl->cis_flags;
	chplan->confs.cis_en_role = rfctl->cis_en_role;
	chplan->confs.cis_en_ifbmp = rfctl->cis_en_ifbmp;
	chplan->confs.cis_scan_band_bmp = rfctl->cis_scan_band_bmp;
	chplan->confs.cis_scan_int_ms = rfctl->cis_scan_int_ms;
	chplan->confs.cis_scan_urgent_ms = rfctl->cis_scan_urgent_ms;
#endif

	chplan->chs_len = chset->chs_len;
	_rtw_memcpy(chplan->chs, chset->chs, sizeof(RT_CHANNEL_INFO) * chset->chs_len);
	param->chplan = chplan;

	return	H2C_SUCCESS;
}

void rtw_get_chplan_callback(_adapter *adapter, struct cmd_obj *cmdobj)
{
	struct get_channel_plan_param *param = (struct get_channel_plan_param *)cmdobj->parmbuf;

	cmdobj->sctx_rsp_buf = param->chplan;
}

u8 rtw_get_chplan_cmd(_adapter *adapter, int flags, struct get_chplan_resp **chplan)
{
	struct cmd_obj *cmdobj;
	struct get_channel_plan_param *parm;
	struct cmd_priv *pcmdpriv = &adapter_to_dvobj(adapter)->cmdpriv;
	struct submit_ctx sctx;
	u8 res = _FAIL;

	if (!(flags & (RTW_CMDF_DIRECTLY | RTW_CMDF_WAIT_ACK))) {
		rtw_warn_on(1);
		goto exit;
	}

	/* prepare cmd parameter */
	parm = rtw_zmalloc(sizeof(*parm));
	if (parm == NULL)
		goto exit;

	if (flags & RTW_CMDF_DIRECTLY) {
		/* no need to enqueue, do the cmd hdl directly and free cmd parameter */
		if (H2C_SUCCESS == rtw_get_chplan_hdl(adapter, (u8 *)parm)) {
			*chplan = parm->chplan;
			res = _SUCCESS;
		}
		rtw_mfree((u8 *)parm, sizeof(*parm));
	} else { /* case of RTW_CMDF_WAIT_ACK */
		/* need enqueue, prepare cmd_obj and enqueue */
		cmdobj = (struct cmd_obj *)rtw_zmalloc(sizeof(*cmdobj));
		if (cmdobj == NULL) {
			rtw_mfree((u8 *)parm, sizeof(*parm));
			goto exit;
		}
		cmdobj->padapter = adapter;

		init_h2fwcmd_w_parm_no_rsp(cmdobj, parm, CMD_GET_CHANPLAN);
		CMD_OBJ_SET_HWBAND(cmdobj, HW_BAND_0);
		cmdobj->no_io = true;

		cmdobj->sctx = &sctx;
		rtw_sctx_init(&sctx, 2000);
		cmdobj->sctx_rsp_buf_free = (void *)rtw_free_get_chplan_resp;

		res = rtw_enqueue_cmd(pcmdpriv, cmdobj);

		if (res == _SUCCESS) {
			rtw_sctx_wait(&sctx, __func__);
			_rtw_mutex_lock_interruptible(&pcmdpriv->sctx_mutex);
			if (sctx.status == RTW_SCTX_SUBMITTED)
				cmdobj->sctx = NULL;
			_rtw_mutex_unlock(&pcmdpriv->sctx_mutex);
			if (sctx.status == RTW_SCTX_DONE_SUCCESS)
				*chplan = sctx.rsp;
			else
				res = _FAIL;
		}

		/* allow get channel plan when cmd_thread is not running */
		if (res != _SUCCESS && (flags & RTW_CMDF_WAIT_ACK)) {
			parm = rtw_zmalloc(sizeof(*parm));
			if (parm == NULL)
				goto exit;

			if (H2C_SUCCESS == rtw_get_chplan_hdl(adapter, (u8 *)parm)) {
				*chplan = parm->chplan;
				res = _SUCCESS;
			}

			rtw_mfree((u8 *)parm, sizeof(*parm));
		}
	}

exit:
	return res;
}

void rtw_free_get_chplan_resp(struct get_chplan_resp *chplan)
{
	size_t sz = sizeof(struct get_chplan_resp) + sizeof(RT_CHANNEL_INFO) * chplan->chs_len
		#if CONFIG_TXPWR_LIMIT
		+ chplan->txpwr_lmt_names_len_total
		#endif
		;

	rtw_vmfree(chplan, sz);
}

bool rtw_network_chk_opch_status_rsn(const struct rf_ctl_t *rfctl, const struct wlan_network *network
	, enum network_opch_status *rsn)
{
	RT_CHANNEL_INFO *chinfo;
	enum network_opch_status status = NETWORK_OPCH_USABLE;
	bool ret = false;

	chinfo = rtw_chset_get_chinfo_by_bch(&rfctl->chset
		, BSS_EX_OP_BAND(&network->network), BSS_EX_OP_CH(&network->network), false);
	if (!chinfo) {
		status = NETWORK_OPCH_DISABLE;
		goto exit;
	}

	if (CH_IS_NON_OCP(chinfo)) {
		status = NETWORK_OPCH_NOCP;
		goto exit;
	}

	ret = true;

exit:
	if (rsn)
		*rsn = status;

	return ret;
}

#ifdef CONFIG_80211D
static bool rtw_iface_accept_country_ie(_adapter *adapter)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);

	if (!(BIT(adapter->iface_id) & rfctl->cis_en_ifbmp))
		return false;
	if (!MLME_IS_STA(adapter))
		return false;
	if (!MLME_IS_GC(adapter)) {
		if (!(rfctl->cis_en_role & COUNTRY_IE_SLAVE_EN_ROLE_STA))
			return false;
	} else {
		if (!(rfctl->cis_en_role & COUNTRY_IE_SLAVE_EN_ROLE_GC))
			return false;
	}
	return true;
}

static bool rtw_rfctl_link_recv_regu_infos_not_empty(struct rf_ctl_t *rfctl, u8 iface_id, u8 alink_id)
{
	return rfctl->recv_country_str[iface_id][alink_id][0] != 0
		|| rfctl->recv_country_str[iface_id][alink_id][1] != 0
		|| rfctl->recv_country_str[iface_id][alink_id][2] != 0
		#if CONFIG_IEEE80211_BAND_6GHZ
		|| rfctl->recv_6g_reg_info[iface_id][alink_id] != CIS_6G_REG_RSVD
		|| rfctl->recv_tpes[iface_id][alink_id].cnt != 0
		#endif
		;
}

static void rtw_rfctl_reset_link_recv_regu_infos(struct rf_ctl_t *rfctl, u8 iface_id, u8 alink_id)
{
	rfctl->recv_country_str[iface_id][alink_id][0] = 0;
	rfctl->recv_country_str[iface_id][alink_id][1] = 0;
	rfctl->recv_country_str[iface_id][alink_id][2] = 0;
	#if CONFIG_IEEE80211_BAND_6GHZ
	rfctl->recv_6g_reg_info[iface_id][alink_id] = CIS_6G_REG_RSVD;
	rfctl->recv_tpes[iface_id][alink_id].cnt = 0;
	#endif
}

void rtw_alink_joinbss_update_regulatory(struct _ADAPTER_LINK *alink, const WLAN_BSSID_EX *network)
{
	_adapter *adapter = alink->adapter;
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	u8 iface_id = adapter->iface_id;
	u8 alink_id = rtw_adapter_link_get_id(alink);

	if (rfctl->collect_link_cisr) {
		struct country_ie_slave_record cisr;
		#if CONFIG_IEEE80211_BAND_6GHZ
		u8 i;
		#endif

		rtw_get_cisr_from_regu_ies(rfctl, BSS_EX_OP_BAND(network), BSS_EX_OP_CH(network)
			, BSS_EX_TLV_IES(network), BSS_EX_TLV_IES_LEN(network), rtw_iface_accept_country_ie(adapter)
			, &cisr);

		rfctl->recv_country_str[iface_id][alink_id][0] = cisr.country_str[0];
		rfctl->recv_country_str[iface_id][alink_id][1] = cisr.country_str[1];
		rfctl->recv_country_str[iface_id][alink_id][2] = cisr.country_str[2];
		#if CONFIG_IEEE80211_BAND_6GHZ
		rfctl->recv_6g_reg_info[iface_id][alink_id] = cisr.reg_info;
		for (i = 0; i < cisr.tpes.cnt; i++)
			rfctl->recv_tpes[iface_id][alink_id].ele[i] = cisr.tpes.ele[i];
		rfctl->recv_tpes[iface_id][alink_id].cnt = cisr.tpes.cnt;
		#endif

		if (rtw_alink_apply_recv_cisr_cmd(alink, RTW_CMDF_DIRECTLY, &cisr) != _SUCCESS)
			RTW_WARN(FUNC_ADPT_FMT" id:%u rtw_alink_apply_recv_cisr_cmd() fail\n", FUNC_ADPT_ARG(adapter), alink_id);
	} else {
		rtw_rfctl_reset_link_recv_regu_infos(rfctl, iface_id, alink_id);

		if (rtw_alink_apply_recv_regu_parm_cmd(alink, RTW_CMDF_DIRECTLY
				, BSS_EX_OP_BAND(network), BSS_EX_OP_CH(network)
				, rfctl->recv_country_str[iface_id][alink_id]
				, RFCTL_RECV_6G_REG_INFO(rfctl, iface_id, alink_id)
				, RFCTL_RECV_TPES(rfctl, iface_id, alink_id)) != _SUCCESS)
			RTW_WARN(FUNC_ADPT_FMT" id:%u rtw_alink_apply_recv_regu_parm_cmd() fail\n", FUNC_ADPT_ARG(adapter), alink_id);
	}
}

static void _rtw_alink_leavebss_update_regulatory(_adapter *adapter, u8 alink_id)
{
	if (alink_id < RTW_RLINK_MAX) {
		struct _ADAPTER_LINK * alink = GET_LINK(adapter, alink_id);

		if (rtw_alink_clear_recv_regu_cmd(alink, RTW_CMDF_DIRECTLY) != _SUCCESS)
			RTW_WARN(FUNC_ADPT_FMT" id:%u rtw_alink_clear_recv_regu_cmd() fail\n", FUNC_ADPT_ARG(adapter), alink_id);
	} else {
		if (rtw_clear_recv_regu_cmd(adapter, RTW_CMDF_DIRECTLY) != _SUCCESS)
			RTW_WARN(FUNC_ADPT_FMT" rtw_clear_recv_regu_cmd() fail\n", FUNC_ADPT_ARG(adapter));
	}
}

void rtw_alink_leavebss_update_regulatory(struct _ADAPTER_LINK * alink)
{
	_rtw_alink_leavebss_update_regulatory(alink->adapter, rtw_adapter_link_get_id(alink));
}

void rtw_alink_csa_update_regulatory(struct _ADAPTER_LINK *alink, enum band_type req_band, u8 req_ch)
{
	_adapter *adapter = alink->adapter;
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	u8 iface_id = adapter->iface_id;
	u8 alink_id = rtw_adapter_link_get_id(alink);

	if (!rfctl->collect_link_cisr)
		rtw_rfctl_reset_link_recv_regu_infos(rfctl, iface_id, alink_id);

	if (rtw_alink_apply_recv_regu_parm_cmd(alink, RTW_CMDF_DIRECTLY
			, req_band, req_ch, rfctl->recv_country_str[iface_id][alink_id]
			, RFCTL_RECV_6G_REG_INFO(rfctl, iface_id, alink_id)
			, RFCTL_RECV_TPES(rfctl, iface_id, alink_id)) != _SUCCESS)
		RTW_WARN(FUNC_ADPT_FMT" id:%u rtw_alink_apply_recv_regu_parm_cmd() fail\n", FUNC_ADPT_ARG(adapter), alink_id);
}

void alink_process_regu_ies(struct _ADAPTER_LINK *alink, u8 *ies, uint ies_len)
{
	_adapter *adapter = alink->adapter;
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	u8 iface_id;
	u8 alink_id;

#ifdef CONFIG_ECSA_PHL
	/* don't process country ie when under CSA processing */
	if (rtw_mr_is_ecsa_running(adapter))
		return;
#endif

	iface_id = adapter->iface_id;
	alink_id = rtw_adapter_link_get_id(alink);

	if (rfctl->collect_link_cisr) {
		struct country_ie_slave_record cisr;
		bool country_str_changed = false;
		#if CONFIG_IEEE80211_BAND_6GHZ
		bool reg_info_changed = false;
		bool tpe_changed = false;
		u8 i;
		#endif

		rtw_get_cisr_from_regu_ies(rfctl, ALINK_GET_BAND(alink), ALINK_GET_CH(alink)
			, ies, ies_len, rtw_iface_accept_country_ie(adapter)
			, &cisr);

		country_str_changed = _rtw_memcmp(rfctl->recv_country_str[iface_id][alink_id], cisr.country_str, 3) == _FALSE;
		#if CONFIG_IEEE80211_BAND_6GHZ
		reg_info_changed = rfctl->recv_6g_reg_info[iface_id][alink_id] != cisr.reg_info;
		if (rfctl->recv_tpes[iface_id][alink_id].cnt == cisr.tpes.cnt) {
			struct rtw_tpe_t *recv_tpes = &rfctl->recv_tpes[iface_id][alink_id];

			for (i = 0; i < cisr.tpes.cnt; i++) {
				if (_rtw_memcmp(&recv_tpes->ele[i], &cisr.tpes.ele[i], sizeof(cisr.tpes.ele[i])) == _FALSE) {
					tpe_changed = true;
					break;
				}
			}
		} else
			tpe_changed = true;
		#endif

		if (!country_str_changed
			#if CONFIG_IEEE80211_BAND_6GHZ
			&& !reg_info_changed
			&& !tpe_changed
			#endif
		)
			return;

		if (country_str_changed) {
			rfctl->recv_country_str[iface_id][alink_id][0] = cisr.country_str[0];
			rfctl->recv_country_str[iface_id][alink_id][1] = cisr.country_str[1];
			rfctl->recv_country_str[iface_id][alink_id][2] = cisr.country_str[2];
		}
		#if CONFIG_IEEE80211_BAND_6GHZ
		if (reg_info_changed)
			rfctl->recv_6g_reg_info[iface_id][alink_id] = cisr.reg_info;
		if (tpe_changed) {
			for (i = 0; i < cisr.tpes.cnt; i++)
				rfctl->recv_tpes[iface_id][alink_id].ele[i] = cisr.tpes.ele[i];
			rfctl->recv_tpes[iface_id][alink_id].cnt = cisr.tpes.cnt;
		}
		#endif

		RTW_INFO(FUNC_ADPT_FMT" id:%u%s%s%s changed\n", FUNC_ADPT_ARG(adapter), alink_id
			, country_str_changed ? " country_str" : ""
			#if CONFIG_IEEE80211_BAND_6GHZ
			, reg_info_changed ? " 6g_reg_info" : ""
			, tpe_changed ? " tpe" : ""
			#else
			, "", ""
			#endif
			);
		rtw_alink_apply_recv_cisr_cmd(alink, 0, &cisr);
	}
	else if (rtw_rfctl_link_recv_regu_infos_not_empty(rfctl, iface_id, alink_id)) {
		rtw_rfctl_reset_link_recv_regu_infos(rfctl, iface_id, alink_id);

		rtw_alink_apply_recv_regu_parm_cmd(alink, 0, ALINK_GET_BAND(alink), ALINK_GET_CH(alink)
			, rfctl->recv_country_str[iface_id][alink_id]
			, RFCTL_RECV_6G_REG_INFO(rfctl, iface_id, alink_id)
			, RFCTL_RECV_TPES(rfctl, iface_id, alink_id));
	}
}

void rtw_joinbss_update_regulatory(_adapter *adapter, const WLAN_BSSID_EX *network)
{
	rtw_alink_joinbss_update_regulatory(GET_PRIMARY_LINK(adapter), network);
}

void rtw_leavebss_update_regulatory(_adapter *adapter)
{
	_rtw_alink_leavebss_update_regulatory(adapter, RTW_RLINK_MAX);
}

void rtw_csa_update_regulatory(_adapter *adapter, enum band_type req_band, u8 req_ch)
{
	rtw_alink_csa_update_regulatory(GET_PRIMARY_LINK(adapter), req_band, req_ch);
}

void process_regu_ies(_adapter *adapter, u8 *ies, uint ies_len)
{
	alink_process_regu_ies(GET_PRIMARY_LINK(adapter), ies, ies_len);
}

void rtw_update_scanned_network_cisr(struct rf_ctl_t *rfctl, struct wlan_network *network)
{
	struct country_ie_slave_record *cisr = &network->cisr;
	enum band_type band = BSS_EX_OP_BAND(&network->network);
	u8 opch = BSS_EX_OP_CH(&network->network);

	if (rfctl->collect_network_cisr) {
		rtw_get_cisr_from_regu_ies(rfctl, band, opch
			, BSS_EX_TLV_IES(&network->network), BSS_EX_TLV_IES_LEN(&network->network), true
			, cisr);
	}
	else {
		rtw_get_cisr_from_regu_ies(rfctl, band, opch
			, NULL, 0, true
			, cisr);
	}
}

bool rtw_network_chk_regu_ies_rsn(const struct rf_ctl_t *rfctl, const struct wlan_network *network
	, enum country_ie_slave_status *rsn)
{
	enum country_ie_slave_status status = network->cisr.status;
	bool ret = false;

	if (!rtw_txpwr_hal_is_txpwr_limit_needed(rfctl_to_dvobj(rfctl)))
		goto bypass;

#if CONFIG_IEEE80211_BAND_6GHZ
	if (network->cisr.band == BAND_ON_6G) {
		if (status == COUNTRY_IE_SLAVE_CATE_6G_NS)
			goto exit;
		if (status == COUNTRY_IE_SLAVE_TPE_FORBID)
			goto exit;
	}
#endif

	if (rtw_rfctl_forbid_unknown_country_opch(rfctl)) {
		if (status == COUNTRY_IE_SLAVE_UNKNOWN
			|| status == COUNTRY_IE_SLAVE_OPCH_NOEXIST)
			goto exit;
	}

bypass:
	ret = true;

exit:
	if (rsn)
		*rsn = status;

	return ret;
}

bool rtw_cis_scan_needed(struct rf_ctl_t *rfctl, bool *urgent)
{
	if (rfctl->cis_enabled && (rfctl->cis_flags & CISF_ENV_BSS)
		&& rfctl->cis_scan_band_bmp
		&& rfctl->cis_scan_int_ms
		&& dvobj_get_netif_up_adapter(rfctl_to_dvobj(rfctl))
	) {
		u32 pt = rtw_get_passing_time_ms(rfctl->cis_scan_last_complete_time);

		if (pt >= rfctl->cis_scan_int_ms) {
			if (urgent)
				*urgent = rfctl->cis_scan_urgent_ms && pt >= rfctl->cis_scan_urgent_ms;
			return true;
		}
	}
	return false;
}

void rtw_cis_scan_idle_check(struct rf_ctl_t *rfctl)
{
	if (rtw_cis_scan_needed(rfctl, NULL)) {
		_adapter *adapter = dvobj_get_netif_up_adapter(rfctl_to_dvobj(rfctl));
		struct sitesurvey_parm *parm;

		if (!adapter)
			return;

		parm = rtw_malloc(sizeof(*parm));
		if (parm == NULL)
			return;

		rtw_init_sitesurvey_parm(parm);
		parm->reason = RTW_AUTO_SCAN_REASON_CIS_ENV_BSS;
		rtw_auto_scan_ch_list_init(parm, rfctl->cis_scan_band_bmp, RTW_IEEE80211_CHAN_PASSIVE_SCAN);

		rtw_sitesurvey_cmd(adapter, parm);

		rtw_mfree(parm, sizeof(*parm));
	}
}

static bool rtw_cis_scan_stat_update(_adapter *adapter)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	struct mlme_priv *mlme = &adapter->mlmepriv;
	_queue *queue = &mlme->scanned_queue;
	_list *list, *head;
	struct wlan_network *scanned;
	bool effected = false;
	struct cis_scan_stat_t *stat = &rfctl->cis_scan_stat;

	_rtw_mutex_lock(&stat->lock);

	cis_scan_stat_clr(stat, false);

	_rtw_spinlock_bh(&queue->lock);

	/* loop scan queue for stat */
	head = get_list_head(queue);
	list = get_next(head);
	while (!rtw_end_of_queue_search(head, list)) {
		scanned = LIST_CONTAINOR(list, struct wlan_network, list);
		list = get_next(list);

		if (scanned->cisr.status == COUNTRY_IE_SLAVE_NOCOUNTRY
			|| scanned->cisr.status == COUNTRY_IE_SLAVE_UNKNOWN)
			continue;

		cis_scan_stat_add(stat, &scanned->cisr);
	}
#if CONFIG_80211D_ENV_BSS_MAJORITY
	cis_scan_stat_update_majority(stat);
#endif

	_rtw_spinunlock_bh(&queue->lock);

	_rtw_mutex_unlock(&stat->lock);

	return effected;
}

void rtw_cis_scan_complete_hdl(_adapter *adapter)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);

	if (rfctl->cis_enabled && (rfctl->cis_flags & CISF_ENV_BSS)) {
		rtw_cis_scan_stat_update(adapter);
		rfctl->cis_scan_last_complete_time = rtw_get_current_time();

		/* 802.11d scan has done complete, trigger regulation selection */
		rtw_apply_scan_network_country_ie_cmd(adapter, RTW_CMDF_DIRECTLY);
	}
}

void rtw_rfctl_cis_init(struct rf_ctl_t *rfctl, struct registry_priv *regsty)
{
#if CONFIG_IEEE80211_BAND_6GHZ
	int i, j;
#endif

	rfctl->cis_en_mode = regsty->country_ie_slave_en_mode;
	if (!CIS_EN_MODE_IS_VALID(rfctl->cis_en_mode)) {
		RTW_WARN("%s cis_en_mode %u is not supported, set to disable\n", __func__, rfctl->cis_en_mode);
		rfctl->cis_en_mode = CISEM_DISABLE;
	}

	rfctl->cis_flags = regsty->country_ie_slave_flags;
	if (rfctl->cis_flags & CISF_ENV_BSS_MAJ)
		rfctl->cis_flags |= CISF_ENV_BSS;
	if (rfctl->cis_flags & ~CISF_VALIDS) {
		RTW_WARN("%s cis_flags:0x%02x has undefined bits, apply valid bits only\n", __func__, rfctl->cis_flags);
		rfctl->cis_flags &= CISF_VALIDS;
	}

	rfctl->cis_en_role = regsty->country_ie_slave_en_role;
	rfctl->cis_en_ifbmp = regsty->country_ie_slave_en_ifbmp;
	rfctl->cis_scan_band_bmp = get_cis_scan_band_bmp(rfctl, regsty->country_ie_slave_scan_band_bmp);
	rfctl->cis_scan_int_ms = regsty->country_ie_slave_scan_int_ms;
	rfctl->cis_scan_urgent_ms = get_cis_scan_urgent_ms(rfctl->cis_scan_int_ms, regsty->country_ie_slave_scan_urgent_ms);
	rfctl->cis_scan_last_complete_time = rtw_get_current_time() - rtw_ms_to_systime(rfctl->cis_scan_int_ms);

	cis_scan_stat_init(&rfctl->cis_scan_stat);

#if CONFIG_IEEE80211_BAND_6GHZ
	rfctl->force_txpwr_lmt_6g_cate = TXPWR_LMT_6G_CATE_NONE;

	for (i = 0; i < CONFIG_IFACE_NUMBER; i++) {
		for (j = 0; j < RTW_RLINK_MAX; j++) {
			rfctl->recv_6g_reg_info[i][j] = CIS_6G_REG_RSVD;
			rfctl->cisr[i][j].reg_info = CIS_6G_REG_RSVD;
		}
	}
#endif
}

void rtw_rfctl_cis_deinit(struct rf_ctl_t *rfctl)
{
	cis_scan_stat_deinit(&rfctl->cis_scan_stat);
}
#endif /* CONFIG_80211D */

#ifdef CONFIG_PROC_DEBUG
void dump_cur_chplan_confs(void *sel, struct rf_ctl_t *rfctl)
{
	struct dvobj_priv *dvobj = rfctl_to_dvobj(rfctl);
	struct get_chplan_resp *chplan;
	struct chplan_confs *confs;
	size_t buf_len = rtw_max(RTW_CH_FLAGS_STR_LEN, EXCL_CHS_STR_LEN);
	char *buf;

#if CONFIG_IEEE80211_BAND_6GHZ
	buf_len = rtw_max(buf_len, EXCL_CHS_6G_STR_LEN);
#endif

	buf = rtw_vmalloc(buf_len);
	if (!buf)
		return;

	if (rtw_get_chplan_cmd(dvobj_get_primary_adapter(dvobj), RTW_CMDF_WAIT_ACK, &chplan) == _FAIL)
		goto free_buf;

	confs = &chplan->confs;

	RTW_PRINT_SEL(sel, "dis_ch_flags=%s\n", rtw_get_ch_flags_str(buf, confs->dis_ch_flags, ','));
	RTW_PRINT_SEL(sel, "excl_chs=%s\n"
		, get_str_of_u8_array(buf, buf_len, confs->excl_chs, MAX_CHANNEL_NUM_2G_5G, ',', true));
#if CONFIG_IEEE80211_BAND_6GHZ
	RTW_PRINT_SEL(sel, "excl_chs_6g=%s\n"
		, get_str_of_u8_array(buf, buf_len, confs->excl_chs_6g, MAX_CHANNEL_NUM_6G, ',', true));
#endif
	RTW_PRINT_SEL(sel, "init_regd_always_apply=%d\n", confs->init_regd_always_apply);
	RTW_PRINT_SEL(sel, "user_regd_always_apply=%d\n", confs->user_regd_always_apply);
	RTW_PRINT_SEL(sel, "extra_alpha2="ALPHA2_FMT"\n", ALPHA2_ARG_EX(confs->extra_alpha2, ' '));
	RTW_PRINT_SEL(sel, "bcn_hint_valid_ms=%u\n", confs->bcn_hint_valid_ms);
#ifdef CONFIG_80211D
	RTW_PRINT_SEL(sel, "cis_en_mode=%u\n", confs->cis_en_mode);
	RTW_PRINT_SEL(sel, "cis_flags=0x%02x\n", confs->cis_flags);
	RTW_PRINT_SEL(sel, "cis_en_role=0x%02x\n", confs->cis_en_role);
	RTW_PRINT_SEL(sel, "cis_en_ifbmp=0x%02x\n", confs->cis_en_ifbmp);
	RTW_PRINT_SEL(sel, "cis_scan_band_bmp=0x%02x\n", confs->cis_scan_band_bmp);
	RTW_PRINT_SEL(sel, "cis_scan_int_ms=%u\n", confs->cis_scan_int_ms);
	RTW_PRINT_SEL(sel, "cis_scan_urgent_ms=%u\n", confs->cis_scan_urgent_ms);
#endif

	rtw_free_get_chplan_resp(chplan);

free_buf:
	rtw_vmfree(buf, buf_len);
}

static void dump_chplan_regd_inrs(void *sel, struct get_chplan_resp *chplan)
{
	char buf[REGD_INR_BMP_STR_LEN];

	RTW_PRINT_SEL(sel, "regd_inr:%s\n", rtw_get_regd_inr_bmp_str(buf, chplan->regd_inr_bmp));
}

#if CONFIG_IEEE80211_BAND_6GHZ
static void dump_chplan_envs(void *sel, struct get_chplan_resp *chplan)
{
	char buf[REGD_INR_BMP_STR_LEN];

	RTW_PRINT_SEL(sel, "env:%s\n", rtw_get_env_bmp_str(buf, chplan->env_bmp));
}
#endif

#if CONFIG_TXPWR_LIMIT
static void dump_chplan_txpwr_lmt_regs(void *sel, struct get_chplan_resp *chplan)
{
	int band;
	const char *names, *name;
	int names_len;

	for (band = 0; band < BAND_MAX; band++) {
		names = chplan->txpwr_lmt_names[band];
		names_len = chplan->txpwr_lmt_names_len[band];

		RTW_PRINT_SEL(sel, "txpwr_lmt[%s]:", band_str(band));
		ustrs_for_each_str(names, names_len, name)
			_RTW_PRINT_SEL(sel, "%s%s", name == names ? "" : " ", name);
		_RTW_PRINT_SEL(sel, "\n");
	}
}
#endif

static void dump_chplan_edcca_modes(void *sel, struct get_chplan_resp *chplan)
{
	u8 mode[BAND_MAX];
	char buf[EDCCA_MODE_OF_BANDS_STR_LEN];

	mode[BAND_ON_24G] = chplan->edcca_mode_2g;
#if CONFIG_IEEE80211_BAND_5GHZ
	mode[BAND_ON_5G] = chplan->edcca_mode_5g;
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	mode[BAND_ON_6G] = chplan->edcca_mode_6g;
#endif

	RTW_PRINT_SEL(sel, "edcca_mode:%s\n", rtw_get_edcca_mode_of_bands_str(buf, mode));
}

static void dump_addl_ch_disable_conf(void *sel, struct get_chplan_resp *chplan)
{
	struct chplan_confs *confs = &chplan->confs;

	if (confs->dis_ch_flags) {
		char buf[RTW_CH_FLAGS_STR_LEN];

		RTW_PRINT_SEL(sel, "dis_ch_flags:%s\n", rtw_get_ch_flags_str(buf, confs->dis_ch_flags, ' '));
	}

	if (confs->excl_chs[0] != 0) {
		char buf[EXCL_CHS_STR_LEN];

		RTW_PRINT_SEL(sel, "excl_chs:%s\n"
			, get_str_of_u8_array(buf, sizeof(buf), confs->excl_chs, MAX_CHANNEL_NUM_2G_5G, ' ', true));
	}

#if CONFIG_IEEE80211_BAND_6GHZ
	if (confs->excl_chs_6g[0] != 0) {
		char buf[EXCL_CHS_6G_STR_LEN];

		RTW_PRINT_SEL(sel, "excl_chs_6g:%s\n"
			, get_str_of_u8_array(buf, sizeof(buf), confs->excl_chs_6g, MAX_CHANNEL_NUM_6G, ' ', true));
	}
#endif
}

void dump_cur_country(void *sel, struct rf_ctl_t *rfctl)
{
	struct dvobj_priv *dvobj = rfctl_to_dvobj(rfctl);
	struct get_chplan_resp *chplan;

	if (rtw_get_chplan_cmd(dvobj_get_primary_adapter(dvobj), RTW_CMDF_WAIT_ACK, &chplan) == _FAIL)
		return;

	RTW_PRINT_SEL(sel, "%c%c\n", chplan->alpha2[0], chplan->alpha2[1]);

	rtw_free_get_chplan_resp(chplan);
}

void dump_cur_chplan(void *sel, struct rf_ctl_t *rfctl)
{
	struct dvobj_priv *dvobj = rfctl_to_dvobj(rfctl);
	struct get_chplan_resp *chplan;

	if (rtw_get_chplan_cmd(dvobj_get_primary_adapter(dvobj), RTW_CMDF_WAIT_ACK, &chplan) == _FAIL)
		return;

	RTW_PRINT_SEL(sel, "regd_src:%s(%d)\n", regd_src_str(chplan->regd_src), chplan->regd_src);
	dump_chplan_regd_inrs(sel, chplan);

	RTW_PRINT_SEL(sel, "alpha2:%c%c\n", chplan->alpha2[0], chplan->alpha2[1]);

#ifdef CONFIG_80211BE_EHT
	RTW_PRINT_SEL(sel, "be:%d\n", (chplan->proto_en & CHPLAN_PROTO_EN_BE) ? 1 : 0);
#endif
#ifdef CONFIG_80211AX_HE
	RTW_PRINT_SEL(sel, "ax:%d\n", (chplan->proto_en & CHPLAN_PROTO_EN_AX) ? 1 : 0);
#endif
#ifdef CONFIG_80211AC_VHT
	RTW_PRINT_SEL(sel, "ac:%d\n", (chplan->proto_en & CHPLAN_PROTO_EN_AC) ? 1 : 0);
#endif
#if CONFIG_IEEE80211_BAND_5GHZ
	RTW_PRINT_SEL(sel, "a:%d\n", (chplan->proto_en & CHPLAN_PROTO_EN_A) ? 1 : 0);
#endif

	if (chplan->channel_plan == RTW_CHPLAN_UNSPECIFIED)
		RTW_PRINT_SEL(sel, "chplan:UNSPEC\n");
	else
		RTW_PRINT_SEL(sel, "chplan:0x%02X\n", chplan->channel_plan);

#if CONFIG_IEEE80211_BAND_6GHZ
	if (chplan->chplan_6g == RTW_CHPLAN_6G_UNSPECIFIED)
		RTW_PRINT_SEL(sel, "chplan_6g:UNSPEC\n");
	else
		RTW_PRINT_SEL(sel, "chplan_6g:0x%02X\n", chplan->chplan_6g);

	dump_chplan_envs(sel, chplan);
#endif

#if CONFIG_TXPWR_LIMIT
	dump_chplan_txpwr_lmt_regs(sel, chplan);
#endif

	dump_chplan_edcca_modes(sel, chplan);

#ifdef CONFIG_DFS_MASTER
	RTW_PRINT_SEL(sel, "dfs_domain:%s(%u)\n", rtw_dfs_regd_str(chplan->dfs_domain), chplan->dfs_domain);
#endif

	dump_addl_ch_disable_conf(sel, chplan);

	dump_chinfos(sel, chplan->chs, chplan->chs_len);

	rtw_free_get_chplan_resp(chplan);
}

#if CONFIG_IEEE80211_BAND_6GHZ
void dump_cur_env(void *sel, struct rf_ctl_t *rfctl)
{
	struct dvobj_priv *dvobj = rfctl_to_dvobj(rfctl);
	struct get_chplan_resp *chplan;

	if (rtw_get_chplan_cmd(dvobj_get_primary_adapter(dvobj), RTW_CMDF_WAIT_ACK, &chplan) == _FAIL)
		return;

	dump_chplan_envs(sel, chplan);

	rtw_free_get_chplan_resp(chplan);
}
#endif
#endif /* CONFIG_PROC_DEBUG */
