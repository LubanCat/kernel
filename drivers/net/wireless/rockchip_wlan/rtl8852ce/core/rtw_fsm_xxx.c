/******************************************************************************
 *
 * Copyright(c) 2023 - 2024 Realtek Corporation.
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
#include <drv_types.h>

#ifdef CONFIG_RTW_FSM_XXX
/*
* State machine
*/
enum XXX_STATE_ST {
	XXX_ST_IDLE,
	XXX_ST_SCAN,
	XXX_ST_END,
	XXX_ST_MAX
};

enum XXX_EV_ID {
	XXX_EV_request_timer_expire,
	XXX_EV_start_scan,
	XXX_EV_survey_done,
	XXX_EV_recv_rep,
	XXX_EV_max
};

static int xxx_idle_st_hdl(void *obj, u16 event, void *param);
static int xxx_scan_st_hdl(void *obj, u16 event, void *param);
static int xxx_end_st_hdl(void *obj, u16 event, void *param);

/* STATE table */
static struct fsm_state_ent xxx_state_tbl[] = {
	ST_ENT(XXX_ST_IDLE, xxx_idle_st_hdl),
	ST_ENT(XXX_ST_SCAN, xxx_scan_st_hdl),
	ST_ENT(XXX_ST_END, xxx_end_st_hdl),
};

/* EVENT table */
static struct fsm_event_ent xxx_event_tbl[] = {
	EV_DBG(XXX_EV_request_timer_expire),
	EV_DBG(XXX_EV_start_scan),
	EV_DBG(XXX_EV_survey_done),
	EV_DBG(XXX_EV_recv_rep),
	EV_DBG(XXX_EV_max) /* EV_MAX for fsm safety checking */
};

struct xxx_obj {

	u32 cid;

	/* obj default */
	struct fsm_obj *obj;
	struct fsm_main *fsm;

	_list list;
};

/*
 * xxx sub function
 */


/*
 * XXX state handler
 */
static int xxx_idle_st_hdl(void *obj, u16 event, void *param)
{
	struct xxx_obj *pxxx = (struct xxx_obj *)obj;

	switch (event) {
	case FSM_EV_STATE_IN:
		break;
	case FSM_EV_ABORT:
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(pxxx);
		break;
	default:
		break;
	}

	return _SUCCESS;
}

static int xxx_scan_st_hdl(void *obj, u16 event, void *param)
{
	struct xxx_obj *pxxx = (struct xxx_obj *)obj;

	switch (event) {
	case FSM_EV_STATE_IN:
		break;
	case FSM_EV_ABORT:
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(pxxx);
		break;
	default:
		break;
	}

	return _SUCCESS;
}

static int xxx_end_st_hdl(void *obj, u16 event, void *param)
{
	struct xxx_obj *pxxx = (struct xxx_obj *)obj;

	switch (event) {
	case FSM_EV_STATE_IN:
		break;
	case FSM_EV_ABORT:
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(pxxx);
		break;
	default:
		break;
	}

	return _SUCCESS;
}

static void xxx_dump_obj_cb(void *obj, char *p, int *sz)
{
	/* nothing to do for now */
}

static void xxx_dump_fsm_cb(void *fsm, char *p, int *sz)
{
	/* nothing to do for now */
}

static int xxx_init_priv_cb(void *priv)
{
	struct xxx_priv *pxxxpriv = (struct xxx_priv *)priv;

	RTW_INFO("%s\n", __func__);

	return _SUCCESS;
}

static int xxx_deinit_priv_cb(void *priv)
{
	struct xxx_priv *pxxxpriv = (struct xxx_priv *)priv;

	RTW_INFO("%s\n", __func__);

	return _SUCCESS;
}

static void xxx_debug(void *obj, char input[][MAX_ARGV], u32 input_num,
	char *output, u32 *out_len)
{
	return;
}

/* For EXTERNAL application to create a xxx object
 * return
 * xxx_obj: ptr to new xxx object
 */
struct xxx_obj *rtw_xxx_new_obj(_adapter *a, struct sta_info *psta, u16 cid)
{
	struct fsm_main *fsm = a->fsmpriv.xxxpriv.fsm;
	struct fsm_obj *obj;
	struct xxx_obj *pxxx;

	obj = rtw_fsm_new_obj(fsm, psta, cid, (void **)&pxxx, sizeof(*pxxx));

	if (pxxx == NULL) {
		FSM_ERR_(fsm, "malloc obj fail\n");
		return NULL;
	}
	pxxx->fsm = fsm;
	pxxx->obj = obj;

	FSM_TRACE(pxxx, "%s\n", __func__);

	return pxxx;
}

/* For EXTERNAL application to abort xxx section
 */
void rtw_xxx_abort(struct xxx_priv *pxxxpriv)
{
	struct xxx_obj *pxxx = pxxxpriv->xxx;

	if (!pxxx)
		return;

	FSM_TRACE(pxxx, "%s\n", __func__);

	rtw_fsm_gen_msg(pxxx, NULL, 0, FSM_EV_ABORT);
}

/* For EXTERNAL application to create RRM FSM */
int rtw_xxx_reg_fsm(struct fsm_priv *fsmpriv)
{
	struct fsm_root *root = fsmpriv->root;
	struct fsm_main *fsm = NULL;
	struct rtw_fsm_tb tb;
	struct xxx_priv *xxxpriv = &fsmpriv->xxxpriv;

	memset(&tb, 0, sizeof(tb));
	tb.max_state = sizeof(xxx_state_tbl)/sizeof(xxx_state_tbl[0]);
	tb.max_event = sizeof(xxx_event_tbl)/sizeof(xxx_event_tbl[0]);
	tb.state_tbl = xxx_state_tbl;
	tb.evt_tbl = xxx_event_tbl;
	tb.priv = xxxpriv;
	tb.init_priv = xxx_init_priv_cb;
	tb.deinit_priv = xxx_deinit_priv_cb;
	tb.dump_obj = xxx_dump_obj_cb;
	tb.dump_fsm = xxx_dump_fsm_cb;
	tb.dbg_level = FSM_LV_TRACE;
	tb.evt_level = FSM_LV_TRACE;
	tb.debug = xxx_debug;

	fsm = rtw_fsm_register_fsm(root, "XXX", &tb);
	xxxpriv->fsm = fsm;

	if (!fsm)
		return _FAIL;

	return _SUCCESS;
}
#endif /* CONFIG_RTW_FSM_XXX */
