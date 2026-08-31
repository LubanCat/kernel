/******************************************************************************
 *
 * Copyright(c) 2019 - 2020 Realtek Corporation.
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
#ifndef __RTW_FSM_H__
#define __RTW_FSM_H__

#include "rtw_fsm_xxx.h" /* xxx_priv reference template */
#include "rtw_fsm_rrm.h" /* rrm_priv */
#include "rtw_fsm_wnm.h" /* btm_priv */

#ifdef CONFIG_RTW_FSM
u8 fsm_post_event_hdl(_adapter *padapter, u8 *pbuf);

/* #define RTW_DEBUG_FSM */
/* #define FSM_DBG_MEM_OVERWRITE */

#ifdef FSM_DBG_MEM_OVERWRITE
void *fsm_kmalloc(u32 sz);
void fsm_kfree(void *ptr, u32 sz);
#endif
#define FSM_NAME_LEN		32
#define CLOCK_UNIT              10 /* ms */

/* event map
 */
#define FSM_EV_MASK	0xff00
#define FSM_USR_EV_MASK	0x0100
#define FSM_INT_EV_MASK	0x0200
#define FSM_GBL_EV_MASK	0x0400
#define FSM_EV_UNKNOWN	0xffff

/* FSM EVENT */
enum FSM_EV_ID {
	/* Expose to all FSM service */
	FSM_INT_EV_MASK_ = FSM_INT_EV_MASK,
	FSM_EV_ABORT,
	FSM_EV_TIMER_EXPIRE,
	FSM_EV_END, /* for reference */

	FSM_EV_STATE_IN,
	FSM_EV_STATE_OUT,

	FSM_EV_NEW_OBJ,
	FSM_EV_DEL_OBJ,

	FSM_EV_INT_MAX,

	FSM_GBL_EV_MASK_ = FSM_GBL_EV_MASK,
	FSM_EV_CONNECTED,
	FSM_EV_CONNECT_FAIL,
	FSM_EV_DISCONNECTED,
	FSM_EV_SCAN_START,
	FSM_EV_SCAN_DONE,

	FSM_EV_GBL_MAX
};

struct fsm_root;
struct fsm_main;
struct fsm_obj;

struct fsm_priv {
	struct fsm_root *root;
#ifdef CONFIG_RTW_FSM_XXX
	struct xxx_priv xxxpriv;
#endif
#ifdef CONFIG_RTW_FSM_RRM
	struct rrm_priv rmpriv;
#endif
#ifdef CONFIG_RTW_FSM_BTM
	struct btm_priv btmpriv;
#endif
};

/* @oid: object id
 * @event: event id
 * @msg: additional message of the event
 * @msg_sz: message size
 */
struct fsm_msg {
	_os_list list;
	u32 oid; /* receiver */
	u16 cid; /* receiver */
	u16 event; /* event id */
	struct fsm_main *fsm;

	void *param;
	int param_sz;
};

enum fsm_dbg_level {
	FSM_LV_NONE,
	FSM_LV_PRINT,
	FSM_LV_ERR,
	FSM_LV_WARN,
	FSM_LV_INFO, /* dbg_level: dump normal info msg */
	FSM_LV_DBG, /* dbg_level: dump state change info */
	FSM_LV_TRACE,
	FSM_LV_MAX
};

#define EV_ENT(ev) {ev, #ev, FSM_LV_INFO}
#define EV_WRN(ev) {ev, #ev, FSM_LV_WARN}
#define EV_INF(ev) {ev, #ev, FSM_LV_INFO}
#define EV_DBG(ev) {ev, #ev, FSM_LV_DBG}

struct fsm_event_ent {
	u16 event;
	char *name;
	u8 evt_level;
};

#define ST_ENT(st, hdl) {st, #st, hdl}
struct fsm_state_ent {
	u8 state;
	char *name;
	int (*fsm_func)(void *priv, u16 event, void *param);
};

/* struct of rtw_fsm_register_fsm() */
struct rtw_fsm_tb {
	void *priv;
	u8 dbg_level;
	u8 evt_level;
	u8 max_state;
	u16 max_event;
	struct fsm_state_ent *state_tbl;
	struct fsm_event_ent *evt_tbl;

	int (*init_priv)(void *priv); /* optional */
	int (*deinit_priv)(void *priv); /* optional */
	int (*active_obj)(void *priv, void *custom_obj); /* optional */
	int (*deactive_obj)(void *priv, void *custom_obj); /* optional */
	/* debug function */
	void (*dump_obj)(void *obj, char *p, int *sz); /* optional */
	void (*dump_fsm)(void *fsm, char *p, int *sz); /* optional */
	void (*debug)(void *custom_obj, char input[][MAX_ARGV],
		      u32 input_num, char *output, u32 *out_len);
};

/* fsm init funciton */
int rtw_fsm_init(struct fsm_priv *fsmpriv, void *priv);
void rtw_fsm_deinit(struct fsm_priv *fsmpriv);
void rtw_fsm_start(struct fsm_priv *fsmpriv);
void rtw_fsm_stop(struct fsm_priv *fsmpriv);

struct fsm_main *rtw_fsm_register_fsm(struct fsm_root *root,
	const char *name, struct rtw_fsm_tb *tb);

void *rtw_fsm_new_obj(struct fsm_main *fsm,
	struct sta_info *psta, u16 cid, void **custom_obj, int obj_sz);
void fsm_activate_obj(struct fsm_obj *obj);
void fsm_deactivate_obj(struct fsm_obj *obj);
#ifdef RTW_DEBUG_FSM
void rtw_fsm_dbg(void *a, char input[][MAX_ARGV],
		      u32 input_num, char *output, u32 out_len);
#endif

/* fsm operating funciton */
struct fsm_msg *fsm_new_msg(struct fsm_obj *obj, u16 event);
int fsm_send_msg(struct fsm_main *fsm, struct fsm_msg *msg);
int fsm_abort_obj(struct fsm_obj *obj);
void fsm_state_goto(struct fsm_obj *obj, u8 new_state);
void fsm_set_alarm(struct fsm_obj *obj, int ms, u16 event);
void fsm_set_alarm_ext(struct fsm_obj *obj,
	int ms, u16 event, u8 id, void *priv);
void fsm_cancel_alarm(struct fsm_obj *obj);
void fsm_cancel_alarm_ext(struct fsm_obj *obj, u8 id);
void fsm_del_obj(struct fsm_obj *obj);
#if 0
void fsm_pause_alarm(struct fsm_obj *obj);
void fsm_pause_alarm_ext(struct fsm_obj *obj, u8 id);
void fsm_resume_alarm(struct fsm_obj *obj);
void fsm_resume_alarm_ext(struct fsm_obj *obj, u8 id);
bool fsm_is_alarm_off(struct fsm_obj *obj);
bool fsm_is_alarm_off_ext(struct fsm_obj *obj, u8 id);
void fsm_extend_alarm_ext(struct fsm_obj *obj, int ms, u8 id);
#endif
u8 fsm_dbg_lv(struct fsm_main *fsm, u8 level);
u8 fsm_evt_lv(struct fsm_main *fsm, u8 level);
int fsm_gen_msg(struct fsm_obj *obj, void *pbuf, u32 sz, u16 event);
int rtw_fsm_gen_cmd(struct fsm_main *fsm, void *pbuf, u32 sz, u16 event);

/* function to manipulate extra queue */
int rtw_fsm_enqueue_ext(struct fsm_main *fsm, struct fsm_msg *msg, u8 to_head);
struct fsm_msg *rtw_fsm_dequeue_ext(struct fsm_main *fsm);
int rtw_fsm_is_ext_queue_empty(struct fsm_main *fsm);

/* util function */
void *rtw_fsm_get_obj(struct fsm_main *fsm, u16 cid);
char *fsm_obj_name(struct fsm_obj *obj);
char *fsm_evt_name(struct fsm_main *fsm, u16 event);

void *fsm_to_priv(struct fsm_main *fsm);
struct sta_info *obj_to_sta(struct fsm_obj *obj);
_adapter *fsm_to_adapter(struct fsm_main *fsm);

/* Notify function */
void rtw_fsm_notify_connect(struct fsm_priv *fsmpriv, struct sta_info *psta, int res);
void rtw_fsm_notify_disconnect(struct fsm_priv *fsmpriv, struct sta_info *psta);
void rtw_fsm_notify_scan_start(struct fsm_priv *fsmpriv, struct sta_info *psta);
void rtw_fsm_notify_scan_done(struct fsm_priv *fsmpriv, struct sta_info *psta);

_adapter *fsm_to_adapter(struct fsm_main *fsm);

#define obj2adp(c) fsm_to_adapter(c->fsm)
#define obj2sta(c) obj_to_sta(c->obj)
#define obj2mac(c) obj_to_sta(c->obj)->phl_sta->mac_addr
#define obj2priv(c) fsm_to_priv(c->fsm)

#define rtw_fsm_st_goto(c, d) fsm_state_goto(c->obj, d)
#define rtw_fsm_set_alarm(c, d, e) fsm_set_alarm(c->obj, d, e)
#define rtw_fsm_cancel_alarm(c) fsm_cancel_alarm(c->obj)

#define rtw_fsm_abort_obj(c) fsm_abort_obj(c->obj)

#define rtw_fsm_new_msg(c, d) fsm_new_msg(c->obj, d)
#define rtw_fsm_send_msg(c, d) fsm_send_msg(c->fsm, d)
#define rtw_fsm_gen_msg(c, d, e, f) fsm_gen_msg(c->obj, d, e, f)
#define rtw_fsm_activate_obj(c) fsm_activate_obj(c->obj)
#define rtw_fsm_deactivate_obj(c) fsm_deactivate_obj(c->obj)
#define rtw_fsm_obj_name(c) fsm_obj_name(c->obj)
#define rtw_fsm_evt_name(c, d) fsm_evt_name(c->fsm, d)
#define rtw_fsm_cancel_alarm_ext(c, d) fsm_cancel_alarm_ext(c->obj, d)
#define rtw_fsm_set_alarm_ext(c, d, e, f, g) fsm_set_alarm_ext(c->obj, d, e, f, g)
#define rtw_fsm_del_obj(c) fsm_del_obj(c->obj)

#if 1
#define FSM_PRINT(fsm, fmt, ...) \
	do {\
		if (!fsm || fsm_dbg_lv(fsm, FSM_LV_PRINT)) \
			RTW_PRINT(fmt, ##__VA_ARGS__); \
	} while (0)

#define FSM_ERR_(fsm, fmt, ...) \
	do {\
		if (!fsm || fsm_dbg_lv(fsm, FSM_LV_ERR)) \
			RTW_ERR(fmt, ##__VA_ARGS__); \
	} while (0)

#define FSM_ERR(c, fmt, ...) \
	do {\
		if (!c->fsm || fsm_dbg_lv(c->fsm, FSM_LV_ERR)) \
			RTW_ERR("%s " fmt, fsm_obj_name(c->obj), ##__VA_ARGS__); \
	} while (0)

#define FSM_WARN_(fsm, fmt, ...) \
	do {\
		if (!fsm || fsm_dbg_lv(fsm, FSM_LV_WARN)) \
			RTW_WARN(fmt, ##__VA_ARGS__); \
	} while (0)

#define FSM_WARN(c, fmt, ...) \
	do {\
		if (!c->fsm || fsm_dbg_lv(c->fsm, FSM_LV_WARN)) \
			RTW_WARN("%s " fmt, fsm_obj_name(c->obj), ##__VA_ARGS__); \
	} while (0)

#define FSM_INFO_(fsm, fmt, ...) \
	do {\
		if (!fsm || fsm_dbg_lv(fsm, FSM_LV_INFO)) \
			RTW_INFO(fmt, ##__VA_ARGS__); \
	} while (0)

#define FSM_INFO(c, fmt, ...) \
	do {\
		if (!c->fsm || fsm_dbg_lv(c->fsm, FSM_LV_INFO)) \
			RTW_INFO("%s " fmt, fsm_obj_name(c->obj), ##__VA_ARGS__); \
	} while (0)

#define FSM_DBG_(fsm, fmt, ...) \
	do {\
		if (!fsm || fsm_dbg_lv(fsm, FSM_LV_DBG)) \
			RTW_INFO(fmt, ##__VA_ARGS__); \
	} while (0)

#define FSM_DBG(c, fmt, ...) \
	do {\
		if (!c->fsm || fsm_dbg_lv(c->fsm, FSM_LV_DBG)) \
			RTW_INFO("%s " fmt, fsm_obj_name(c->obj), ##__VA_ARGS__); \
	} while (0)

#define FSM_TRACE_(fsm, fmt, ...) \
	do {\
		if (!fsm || fsm_dbg_lv(fsm, FSM_LV_TRACE)) \
			RTW_INFO(fmt, ##__VA_ARGS__); \
	} while (0)

#define FSM_TRACE(c, fmt, ...) \
	do {\
		if (!c->fsm || fsm_dbg_lv(c->fsm, FSM_LV_TRACE)) \
			RTW_INFO("%s " fmt, fsm_obj_name(c->obj), ##__VA_ARGS__); \
	} while (0)

#define FSM_MSG_(fsm, level_, fmt, ...) \
	do {\
		if (!fsm || fsm_dbg_lv(fsm, level_)) \
			RTW_INFO(fmt, ##__VA_ARGS__); \
	} while (0)

#define FSM_MSG(c, level_, fmt, ...) \
	do {\
		if (!c->fsm || fsm_dbg_lv(fsm, level_)) \
			RTW_PRINT("%s ", fmt, fsm_obj_name(c->obj), ##__VA_ARGS__); \
	} while (0)

#define FSM_EV_MSG_(fsm, level_, fmt, ...) \
	do {\
		if (!fsm || fsm_evt_lv(fsm, level_)) \
			RTW_INFO(fmt, ##__VA_ARGS__); \
	} while (0)
#else
#undef FSM_PRINT
#define FSM_PRINT(fsm, fmt, ...)
#undef FSM_ERR
#define FSM_ERR(fsm, fmt, ...)
#undef FSM_WARN
#define FSM_WARN(fsm, fmt, ...)
#undef FSM_INFO
#define FSM_INFO(fsm, fmt, ...)
#undef FSM_DBG
#define FSM_DBG(fsm, fmt, ...)
#undef FSM_MSG
#define FSM_MSG(fsm, level, fmt, ...)
#undef FSM_EV_MSG
#define FSM_EV_MSG(fsm, level, fmt, ...)
#endif  /* CONFIG_PHL_WPP */
#endif /* CONFIG_RTW_FSM */
#endif /* __PHL_FSM_H__ */
