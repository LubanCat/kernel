/*
 * This module is designed to collect internal diagnostic information.
 *
 *	Author: Cosa
 *	History: Created at 2023/02/03
 */
#define _PHL_DIAGNOSE_C_
#include "phl_headers.h"

#ifdef CONFIG_PHL_DIAGNOSE

static void _diag_evt_done(void* priv, struct phl_msg* msg)
{
	struct phl_info_t *phl_info = (struct phl_info_t *)priv;

	if (msg->inbuf && msg->inlen) {
		_os_kmem_free(phl_to_drvpriv(phl_info),
			msg->inbuf, msg->inlen);
	}
}

bool phl_send_diag_hub_msg(void *phl, u16 phl_evt, u8 sub_evt,
	u8 level, u8 ver, u8 *buf, u32 len)
{
	struct phl_msg msg = {0};
	struct phl_msg_attribute attr = {0};
	struct rtw_phl_diag_msg *dmsg = NULL;
	struct phl_info_t *phl_info = (struct phl_info_t *)phl;
	void *d = NULL;

	if (!phl_info || !buf)
		return false;

	if ((phl_evt >= PHL_DIAG_EVT_MAX) ||(level >= PHL_DIAG_LVL_MAX) ||
	    (len > MAX_PHL_DIAG_MSG_LEN) || !phl_info)  {
		return false;
	}

	d = phl_to_drvpriv(phl_info);
	if (!d)
		return false;

	dmsg = (struct rtw_phl_diag_msg *)_os_kmem_alloc(
			d, sizeof(struct rtw_phl_diag_msg));
	if (!dmsg)
		return false;

	SET_MSG_MDL_ID_FIELD(msg.msg_id, PHL_MDL_GENERAL);
	SET_MSG_EVT_ID_FIELD(msg.msg_id, MSG_EVT_DIAGNOSTIC);

	dmsg->type = phl_evt;
	dmsg->level = level;
	if (phl_evt == PHL_DIAG_EVT_MAC || phl_evt == PHL_DIAG_EVT_BB ||
	    phl_evt == PHL_DIAG_EVT_RF) {
		dmsg->sub_evt = sub_evt;
	} else {
		dmsg->sub_evt = INVALID_SUBMODULE_DIAG_EVT;
	}
	dmsg->ver = ver;
	dmsg->len = len;
	_os_mem_cpy(d, dmsg->buf, buf, len);

	msg.inbuf = (u8 *)dmsg;
	msg.inlen = sizeof(struct rtw_phl_diag_msg);
	attr.completion.completion = _diag_evt_done;
	attr.completion.priv = phl_info;

	if (phl_msg_hub_send(phl_info, &attr, &msg) != RTW_PHL_STATUS_SUCCESS) {
		_os_kmem_free(d, dmsg, sizeof(struct rtw_phl_diag_msg));
		return false;
	} else {
		return true;
	}
}


bool rtw_phl_send_diag_hub_msg(struct rtw_phl_com_t *phl_com,
		u16 phl_evt, u8 sub_evt, u8 level, u8 ver, u8 *buf, u32 len)
{
	struct phl_info_t *phl = NULL;

	if (!phl_com || !buf)
		return false;

	phl = phl_com->phl_priv;

	return phl_send_diag_hub_msg(phl, phl_evt, sub_evt, level, ver, buf, len);
}

bool rtw_phl_query_rf_diag_err_code(void *phl, u32 *err_code)
{
	struct phl_info_t *phl_info = (struct phl_info_t *)phl;

	if (!phl_info || !err_code)
		return false;

	if (RTW_HAL_STATUS_SUCCESS ==
		rtw_hal_rf_query_diag_err_code(phl_info->hal, err_code))
		return true;
	else
		return false;
}

bool rtw_phl_query_rf_diag_info_len(u32 *len)
{
	if (!len)
		return false;

	if (RTW_HAL_STATUS_SUCCESS ==
		rtw_hal_rf_query_diag_info_len(len))
		return true;
	else
		return false;
}

bool rtw_phl_query_bb_diag_info_len_ver(u32 *len, u8 *ver, u8 type)
{
	if (!len || !ver)
		return false;

	if (RTW_HAL_STATUS_SUCCESS ==
		rtw_hal_bb_query_diag_info_len_ver(len, ver, type))
		return true;
	else
		return false;
}

bool rtw_phl_query_rf_diag_info(void *phl,
	struct rtw_phl_diag_rf_info *info)
{
	struct phl_info_t *phl_info = (struct phl_info_t *)phl;

	if (!phl_info || !info)
		return false;

	if (RTW_HAL_STATUS_SUCCESS ==
		rtw_hal_rf_query_diag_info(phl_info->hal, info))
		return true;
	else
		return false;
}

enum rtw_phl_status rtw_phl_query_bb_diag_info(void *phl,
		struct rtw_wifi_role_link_t *rlink, u8 *bb_comp_buf,
		u32 len, enum rtw_phl_bb_dbg_type type)
{
	struct phl_info_t *phl_info = (struct phl_info_t *)phl;
	void *d = NULL;
	enum rtw_phl_status pstatus = RTW_PHL_STATUS_SUCCESS;

	if (!phl_info || !rlink || !bb_comp_buf)
		return RTW_PHL_STATUS_FAILURE;

	d = phl_to_drvpriv(phl_info);
	if (!d) {
		PHL_WARN("rtw_phl_query_bb_diag_info(): NULL D\n");
		return RTW_PHL_STATUS_FAILURE;
	}

	PHL_WARN("rtw_phl_query_bb_diag_info(): rlink hw_band %d\n", rlink->hw_band);
	if (type & PHL_BB_PMAC) {
		if (RTW_HAL_STATUS_SUCCESS != rtw_hal_get_pmac_info(
				phl_info->hal, rlink->hw_band, bb_comp_buf, len)) {
			pstatus	= RTW_PHL_STATUS_FAILURE;
			PHL_WARN("rtw_phl_query_bb_diag_info(): get pmac failure\n");
		}
	} else if (type & PHL_BB_UTILITY) {
		if (RTW_HAL_STATUS_SUCCESS != rtw_hal_get_utility_info(
				phl_info->hal, rlink->hw_band, bb_comp_buf, len)) {
			pstatus	= RTW_PHL_STATUS_FAILURE;
			PHL_WARN("rtw_phl_query_bb_diag_info(): get utility failure\n");
		}
	} else {
		pstatus = RTW_PHL_STATUS_FAILURE;
		PHL_WARN("rtw_phl_query_bb_diag_info(): unkonw query type\n");
	}

	return pstatus;
}


#endif

