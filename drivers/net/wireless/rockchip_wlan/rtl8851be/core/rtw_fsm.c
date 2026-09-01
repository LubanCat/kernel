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
 * Author: vincent_fann@realtek.com
 *
 *****************************************************************************/
#include <drv_types.h>
#include <rtw_fsm.h>

#ifdef CONFIG_RTW_FSM

/* #define USE_PHL_CMD_DISPR */

#define CLOCK_NUM 3
#define CLOCK_UNIT 10
#define IS_CLK_OFF(clk) (clk->remain < 0) /* Negative value means disabled */
#define IS_CLK_ON(clk) (clk->remain >= 0)
#define IS_CLK_EXP(clk) (clk->remain < (CLOCK_UNIT >> 1)) /* expire */

#ifndef MAX
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#endif
#define pstr(s) (s +strlen((u8 *)s))
#define lstr(s, l) (size_t)(l - strlen((u8 *)s))

static struct fsm_obj *fsm_get_obj(struct fsm_main *fsm, u32 oid, u16 cid);

//#define FSM_DBG_MEM_OVERWRITE
#ifdef FSM_DBG_MEM_OVERWRITE
void *fsm_kmalloc(u32 sz)
{
	char *ptr;

	ptr = kmalloc(sz+4, GFP_KERNEL);
	memset(ptr+sz, 0xff, 4);
	RTW_INFO("+AA %p %d\n", ptr, sz);
	return ptr;
}

void fsm_kfree(void *ptr, u32 sz)
{
	u32 ptn = 0xffffffff;
	u32 *p = (u32 *)(ptr+sz);

	RTW_INFO("-AA %p %d", ptr, sz);
	if ((*p&ptn) != ptn) {
		RTW_ERR("- %p %d", ptr, sz);
		RTW_ERR("OVER WRITE %x\n", ptn);
	}
	kfree(ptr);
}
#define _os_kmem_alloc(a, b) fsm_kmalloc(b)
#define _os_kmem_free(a, b, c) fsm_kfree(b, c)
#endif

struct fsm_event_ent int_event_tbl[] = {
	EV_DBG(FSM_INT_EV_MASK),
	EV_DBG(FSM_EV_ABORT),
	EV_DBG(FSM_EV_TIMER_EXPIRE),
	EV_DBG(FSM_EV_END),

	//EV_DBG(FSM_EV_SWITCH_IN),
	//EV_DBG(FSM_EV_SWITCH_OUT),
	EV_DBG(FSM_EV_STATE_IN),
	EV_DBG(FSM_EV_STATE_OUT),

	EV_DBG(FSM_EV_NEW_OBJ),
	EV_DBG(FSM_EV_DEL_OBJ),

	EV_DBG(FSM_EV_INT_MAX),

	EV_DBG(FSM_GBL_EV_MASK),
	EV_DBG(FSM_EV_CONNECTED),
	EV_DBG(FSM_EV_CONNECT_FAIL),
	EV_DBG(FSM_EV_DISCONNECTED),
	EV_DBG(FSM_EV_SCAN_START),
	EV_DBG(FSM_EV_SCAN_DONE),

	EV_DBG(FSM_EV_GBL_MAX)
};

/*
 * FSM status
 */
enum FSM_STATUS {
	FSM_STATUS_NONE,	/* default value */

	FSM_STATUS_INITIALIZED, /* insert module ok,
				 * mem/queue/timer were allocated
				 */

	FSM_STATUS_ENABLE,	/* run msg */

	FSM_STATUS_DISABLE,	/* Does NOT run and receiv msg,
				 * thread will remove all NON INT event
				 * after dequeue msg
				 */
};

/* @obj: obj that will be infomred to when time's up
 * @counter: clock time period
 * @event: event that will delivered when time's up
 * @end: end time
 * @pause: stop countdown
 */
struct fsm_clk {
	u16 event;
	void *priv;
	u8 pause;
	u32 start;
	u32 end;
	int remain; /* ms */
};

struct fsm_queue {
	struct	list_head q;
	_lock lock;
};

struct fsm_main {
	struct list_head list;
	char name[FSM_NAME_LEN];
	u8 status;
	u8 obj_cnt;
	u8 en_clock_num;

	_timer fsm_timer; /* unit in ms */

	struct fsm_root *root;
	struct fsm_queue obj_queue;
	struct fsm_queue msg_queue;
	struct fsm_queue clk_queue;

	/* extra custom queue; for fsm private */
	struct fsm_queue ext_queue;

	struct rtw_fsm_tb tb;
};

/* @obj_id: object id
 * @state: current state
 * @prive: object's private date
 */
struct fsm_obj {
	struct list_head list;
	u32 oid;
	u16 cid; /* customer id */
	u8 state;
	u8 valid; /* framework */
	char name[FSM_NAME_LEN];
	struct fsm_clocks *clocks;
	struct fsm_main *fsm;

	void *custom_obj;
	int custom_len; /* custom obj length */

	struct sta_info *psta;
};

struct fsm_clocks {
	struct list_head list;
	struct fsm_clk clk[CLOCK_NUM];
	struct fsm_obj *obj;
};

/* Main structure to handle all standalone fsm */
struct fsm_root {
	struct task_struct * thread;
	struct list_head list;
	struct fsm_queue q_share_thd;
	u8 thread_should_stop;

	_adapter *a;
	u32 oid_seq; /* starts from 1 */

	_sema msg_ready;

	u32 status; /* refer to enum FSM_ROOT_STATUS_FLAGS */
};

/* Static function porto type */
static int fsm_handler(struct fsm_main *fsm);
static char *fsm_state_name(struct fsm_main *fsm, u8 state);
static u8 get_evt_level(struct fsm_main *fsm, u16 event);
static void fsm_destory_obj(struct fsm_obj *obj);
static int fsm_start_fsm(struct fsm_main *fsm);
static int fsm_stop_fsm(struct fsm_main *fsm);
static void fsm_user_evt_handler(struct fsm_main *fsm);
static int fsm_enqueue_list(struct fsm_queue *queue, struct list_head *list);

u8 fsm_post_event_hdl(_adapter *padapter, u8 *pbuf)
{
#ifdef USE_PHL_CMD_DISPR
	struct fsm_msg *msg = (struct fsm_msg *)pbuf;

	fsm_enqueue_list(&msg->fsm->msg_queue, &msg->list);
	fsm_user_evt_handler(msg->fsm);
#endif
	return H2C_SUCCESS;
}

static void fsm_status_set(struct fsm_main *fsm, enum FSM_STATUS status)
{
	fsm->status = status;
}

static enum FSM_STATUS fsm_status(struct fsm_main *fsm)
{
	return fsm->status;
}

/* unit ms */
u32 rtw_fsm_time_pass(u32 start)
{
	u32 now = rtw_systime_to_ms(rtw_get_current_time());
	u32 pass;

	if (start <= now)
		pass = now - start;
	else
		pass = 0xffffffff - start + now;

	return pass;
}

u32 rtw_fsm_time_left(u32 start, u32 end)
{
	u32 total, pass;
	int left = 0;

	pass = rtw_fsm_time_pass(start);

	if (end >= start)
		total = end - start;
	else
		total = 0xffffffff - start + end;

	left = total - pass;

	if (left < 0)
		left = 0;

	return (u32)left;
}

static struct fsm_msg *fsm_dequeue_msg(struct fsm_main *fsm)
{
	struct fsm_msg *msg;

	if (list_empty(&fsm->msg_queue.q))
		return NULL;

	_rtw_spinlock_bh(&fsm->msg_queue.lock);
	msg = list_first_entry(&fsm->msg_queue.q, struct fsm_msg, list);
	list_del(&msg->list);
	_rtw_spinunlock_bh(&fsm->msg_queue.lock);
	return msg;
}

#if 0
static struct fsm_clk *fsm_dequeue_clk(struct fsm_main *fsm)
{
	struct fsm_clk *clk;

	if (list_empty(&fsm->clk_queue.q))
		return NULL;

	_rtw_spinlock_bh(&fsm->clk_queue.lock);
	clk = list_first_entry(&fsm->clk_queue.q, struct fsm_clk, list);
	list_del(&clk->list);
	_rtw_spinunlock_bh(&fsm->clk_queue.lock);
	return clk;
}
#endif

/* For EXTERNAL application to get adapter (expose) */
void *fsm_to_priv(struct fsm_main *fsm)
{
	return fsm->tb.priv; /* xxx_priv */
}

struct sta_info *obj_to_sta(struct fsm_obj *obj)
{
	return obj->psta;
}

_adapter *fsm_to_adapter(struct fsm_main *fsm)
{
	return fsm->root->a;
}

/* For EXTERNAL application to enqueue message to extra queue (expose)
 *
 * @fsm: fsm that object belonged to
 * @msg: message to be enqueued
 * @to_head: enqueue message to the head
 */
int rtw_fsm_enqueue_ext(struct fsm_main *fsm, struct fsm_msg *msg, u8 to_head)
{
	struct fsm_queue *queue = &fsm->ext_queue;

	_rtw_spinlock_bh(&queue->lock);
	if (to_head)
		list_add(&msg->list, &queue->q);
	else
		list_add_tail(&msg->list, &queue->q);
	_rtw_spinunlock_bh(&queue->lock);

	return 0;
}

/* For EXTERNAL application to dequeue message from extra queue (expose)
 *
 * @fsm: fsm that object belonged to
 */
struct fsm_msg *rtw_fsm_dequeue_ext(struct fsm_main *fsm)
{
	struct fsm_msg *msg;

	if (list_empty(&fsm->ext_queue.q))
		return NULL;

	_rtw_spinlock_bh(&fsm->ext_queue.lock);
	msg = list_first_entry(&fsm->ext_queue.q, struct fsm_msg, list);
	list_del(&msg->list);
	_rtw_spinunlock_bh(&fsm->ext_queue.lock);
	return msg;
}

/* For EXTERNAL application to dequeue message from extra queue (expose)
 *
 * @fsm: fsm that object belonged to
 */
int rtw_fsm_is_ext_queue_empty(struct fsm_main *fsm)
{
	return list_empty(&fsm->ext_queue.q);
}

static u32 fsm_new_oid(struct fsm_main *fsm)
{
	/* TODO: how to handle redendent oid ? */
	do {
		fsm->root->oid_seq++;
	} while (fsm->root->oid_seq == 0);

	return fsm->root->oid_seq;
}

static int fsm_enqueue_list(struct fsm_queue *queue, struct list_head *list)
{
	_rtw_spinlock_bh(&queue->lock);
	list_add_tail(list, &queue->q);
	_rtw_spinunlock_bh(&queue->lock);
	return 0;
}

static int fsm_state_run(struct fsm_obj *obj, u16 event, void *param)
{
	struct fsm_main *fsm = obj->fsm;

	FSM_EV_MSG_(fsm, get_evt_level(fsm, event),
		"%s %-18s    %s\n", obj->name,
		fsm_state_name(fsm, obj->state), fsm_evt_name(obj->fsm, event));

	return fsm->tb.state_tbl[obj->state].fsm_func(obj->custom_obj,
		event, param);
}

static void fsm_remove_all_queuing_msg(struct fsm_main *fsm)
{
	struct fsm_msg *msg;
	//struct fsm_evt *evt;
	struct fsm_obj *obj;

	/* go through msg queue and free everything */
	while ((msg = fsm_dequeue_msg(fsm)) != NULL) {
		obj = fsm_get_obj(fsm, 0, msg->cid);
		if (msg->param)
			rtw_mfree((void *)msg->param, msg->param_sz);
		rtw_mfree((void *)msg, sizeof(*msg));
	}

	/* go through event queue and free everything */
#if 0
	while ((evt = fsm_dequeue_clk(fsm)) != NULL) {
		if (evt->param)
			rtw_mfree((void *)evt->param, evt->param_sz);
		rtw_mfree((void *)evt, sizeof(*evt));
	}
#endif

	/* go through ext queue and free everything */
	while ((msg = rtw_fsm_dequeue_ext(fsm)) != NULL) {
		if (msg->param)
			rtw_mfree((void *)msg->param, msg->param_sz);
		rtw_mfree((void *)msg, sizeof(*msg));
	}
}

static int fsm_abort_all_running_obj(struct fsm_main *fsm)
{
	struct fsm_obj *obj;

	if (list_empty(&fsm->obj_queue.q))
		return 0;

	list_for_each_entry(obj, &fsm->obj_queue.q, list) {
		fsm_gen_msg(obj, NULL, 0, FSM_EV_ABORT);
	}

	return 0;
}

u8 fsm_dbg_lv(struct fsm_main *fsm, u8 level)
{
	if (fsm->tb.dbg_level >= level)
		return fsm->tb.dbg_level;

	return 0;
}

u8 fsm_evt_lv(struct fsm_main *fsm, u8 level)
{
	if (fsm->tb.evt_level >= level)
		return fsm->tb.evt_level;

	return 0;
}

static u8 get_evt_level(struct fsm_main *fsm, u16 event)
{
	u16 ev;

	/* fsm global event */
	if (event & FSM_GBL_EV_MASK) {
		ev = (u8)((event & ~(FSM_GBL_EV_MASK)) + 1 +
			(FSM_EV_INT_MAX & ~(FSM_INT_EV_MASK)));
		return int_event_tbl[ev].evt_level;
	}

	/* fsm internal event */
	if (event & FSM_INT_EV_MASK) {
		ev = (u8)(event & ~(FSM_INT_EV_MASK));
		return int_event_tbl[ev].evt_level;
	}

	if (event == FSM_EV_UNKNOWN)
		return FSM_LV_INFO;

	if (event > fsm->tb.max_event)
		return FSM_LV_INFO;

	/* user event */
	return fsm->tb.evt_tbl[event].evt_level;
}

static void fsm_init_queue(struct fsm_queue *queue)
{
	INIT_LIST_HEAD(&queue->q);
	_rtw_spinlock_init(&queue->lock);
}

static void fsm_deinit_queue(struct fsm_queue *queue)
{
	_rtw_spinlock_free(&queue->lock);
}


#ifndef USE_PHL_CMD_DISPR
int fsm_thread_share(void *param)
{
	struct fsm_obj *obj;
	struct fsm_main *fsm, *fsm_t;
	struct fsm_root *root = (struct fsm_root *)param;
	int rtn;

	while (1) {

		//while (down_timeout(&root->msg_ready, 100)) {
		rtn = down_interruptible(&root->msg_ready);

		if (root->thread_should_stop)
			goto exit;

		list_for_each_entry(fsm, &root->q_share_thd.q, list) {
			fsm_handler(fsm);
		}
	}
exit:
	while (!kthread_should_stop()) {
		RTW_INFO("fsm: [FSM] thread wait stop\n");
		rtw_msleep_os(10);
	}
	RTW_INFO("fsm: [FSM] thread down\n");

	return 0;
}
#endif

/* cid customer cid */
static struct fsm_obj *fsm_get_obj(struct fsm_main *fsm, u32 oid, u16 cid)
{
	struct fsm_obj *obj, *obj_t;

	if (list_empty(&fsm->obj_queue.q))
		return NULL;

	_rtw_spinlock_bh(&fsm->obj_queue.lock);
	list_for_each_entry(obj, &fsm->obj_queue.q, list) {
		if ((oid && (oid == obj->oid)) ||
		    (cid && (cid == obj->cid))) {
			_rtw_spinunlock_bh(&fsm->obj_queue.lock);
			return obj;
		}
	}
	_rtw_spinunlock_bh(&fsm->obj_queue.lock);
	return NULL;
}

struct fsm_msg *fsm_new_msg(struct fsm_obj *obj, u16 event)
{
	struct fsm_msg *msg = NULL;

	msg = (struct fsm_msg *)rtw_zmalloc(sizeof(*msg));

	if (msg == NULL)
		return NULL;

	_rtw_memset(msg, 0, sizeof(*msg));
	msg->event = event;

	if (obj) {
		msg->fsm = obj->fsm;
		msg->oid = obj->oid;
		msg->cid = obj->cid;
	}
	return msg;
}

#ifdef USE_PHL_CMD_DISPR
static int rtw_fsm_send_cmd_msg(struct fsm_main *fsm, struct fsm_msg *msg)
{
	struct cmd_obj *pcmd = NULL;
	_adapter *padapter = fsm_to_adapter(fsm);
	struct cmd_priv *pcmdpriv = &adapter_to_dvobj(padapter)->cmdpriv;

	pcmd = (struct cmd_obj *)rtw_zmalloc(sizeof(struct cmd_obj));
	if (pcmd == NULL)
		goto fail;

	pcmd->padapter = padapter;
	init_h2fwcmd_w_parm_no_rsp(pcmd, msg, CMD_RM_POST_EVENT);
	return rtw_enqueue_cmd(pcmdpriv, pcmd);
fail:
	if (msg->param && msg->param_sz)
		rtw_mfree(msg->param, msg->param_sz);
	rtw_mfree(msg, sizeof(*msg));

	if (pcmd)
		rtw_mfree(pcmd, sizeof(*pcmd));
	return _FAIL;
}
#endif

int fsm_send_msg(struct fsm_main *fsm, struct fsm_msg *msg)
{

	if ((fsm_status(fsm) != FSM_STATUS_ENABLE) &&
		!(msg->event & FSM_INT_EV_MASK)) {
		RTW_WARN("fsm: %s is out of service, drop message %s!\n",
			fsm->name, fsm_evt_name(fsm, msg->event));
		if (msg->param && msg->param_sz)
			rtw_mfree(msg->param, msg->param_sz);
		rtw_mfree(msg, sizeof(*msg));
		return _FAIL;

	}
#ifdef USE_PHL_CMD_DISPR
	rtw_fsm_send_cmd_msg(fsm, msg);
#else
	fsm_enqueue_list(&fsm->msg_queue, &msg->list);
	_rtw_up_sema(&fsm->root->msg_ready);
#endif
	return _SUCCESS;
}

void fsm_timer_callback(void *context)
{
	struct fsm_main *fsm = (struct fsm_main *)context;
	struct fsm_clk *clk;
	struct fsm_clocks *clocks;
	int i = 0;

	_set_timer(&fsm->fsm_timer, CLOCK_UNIT);

	_rtw_spinlock_bh(&fsm->clk_queue.lock);

	if (fsm->en_clock_num == 0)
		goto done;

	/* go through clock and descrease timer
	 * if timer was expired, issue event
	 */
	list_for_each_entry(clocks, &fsm->clk_queue.q, list) {
		for (i = 0; i < CLOCK_NUM; i++) {

			clk = &clocks->clk[i];

			if (IS_CLK_OFF(clk) || clk->pause)
				continue;

			clk->remain = (int)rtw_fsm_time_left(clk->start, clk->end);

			/* timer expired */
			if (!IS_CLK_EXP(clk))
				continue;

			FSM_TRACE_(fsm, "%s: expire in %d ms\n",
				fsm_evt_name(clocks->obj->fsm, clk->event),
				rtw_fsm_time_pass(clk->start));

			clk->end = 0;
			clk->remain = -1;
			/* send message to obj */

			/* check fsm status before posting */
			if (fsm_status(fsm) == FSM_STATUS_ENABLE) {
				fsm_gen_msg(clocks->obj, NULL, 0, clk->event);
				if (--fsm->en_clock_num == 0)
					break;
			}
		}
	}
done:
	_rtw_spinunlock_bh(&fsm->clk_queue.lock);
}

/* allocate and init fsm resource */
struct fsm_main *rtw_fsm_register_fsm(struct fsm_root *root,
	const char *name, struct rtw_fsm_tb *tb)
{
	struct fsm_main *fsm;

	/* check event table */
	if (tb->evt_tbl[tb->max_event-1].event != tb->max_event-1) {
		RTW_ERR("Event mismatch ? Is max event = %d != %d ?\n",
			tb->evt_tbl[tb->max_event-1].event,
			tb->max_event-1);
		return NULL;
	}

	/* check state table */
	if (tb->state_tbl[tb->max_state-1].state != tb->max_state-1) {
		RTW_ERR("State mismatch ? Is max state = %d != %d) ?\n",
			tb->state_tbl[tb->max_state-1].state,
			tb->max_state-1);
		return NULL;
	}

	fsm = (struct fsm_main *)rtw_zmalloc(sizeof(*fsm));

	if (fsm == NULL)
		return NULL;

	_rtw_memset(fsm, 0, sizeof(*fsm));
	_rtw_memcpy(&fsm->tb, (void *)tb, sizeof(*tb));
	_rtw_memcpy(&fsm->name, (void *)name,
		MIN(FSM_NAME_LEN-1, strlen((u8 *)name)));

	fsm->root = root;
	fsm_init_queue(&(fsm->obj_queue));
	fsm_init_queue(&(fsm->msg_queue));
	fsm_init_queue(&(fsm->clk_queue));
	fsm_init_queue(&(fsm->ext_queue));

	if (tb->init_priv)
		tb->init_priv(tb->priv);

	rtw_init_timer(&fsm->fsm_timer, fsm_timer_callback, fsm);

	/* link fsm_main to fsm_root */
	fsm_enqueue_list(&root->q_share_thd, &fsm->list);

	FSM_INFO_(fsm, "fsm: [%s] %s()\n", fsm->name, __func__);
	fsm_status_set(fsm, FSM_STATUS_INITIALIZED);
	return fsm;
}

void fsm_deinit_fsm(struct fsm_main *fsm)
{
	_adapter *a;
	struct fsm_obj *obj, *obj_t;

	if (!fsm)
		return;

	_cancel_timer_ex(&fsm->fsm_timer);
	a = fsm_to_adapter(fsm);

	if (!list_empty(&fsm->obj_queue.q)) {
		list_for_each_entry_safe(obj, obj_t, &fsm->obj_queue.q, list) {
			fsm_destory_obj(obj);
		}
	}

	FSM_INFO_(fsm, "fsm: [%s] "FUNC_NDEV_FMT"\n", fsm->name, FUNC_NDEV_ARG(a->pnetdev));

	fsm_deinit_queue(&(fsm->obj_queue));
	fsm_deinit_queue(&(fsm->msg_queue));
	fsm_deinit_queue(&(fsm->clk_queue));
	fsm_deinit_queue(&(fsm->ext_queue));
	rtw_mfree(fsm, sizeof(*fsm));

	return;
}

char *fsm_evt_name(struct fsm_main *fsm, u16 event)
{
	u8 ev;

	/* fsm global event */
	if (event & FSM_GBL_EV_MASK) {
		ev = (u8)((event & ~(FSM_GBL_EV_MASK)) + 1 +
			(FSM_EV_INT_MAX & ~(FSM_INT_EV_MASK)));
		return int_event_tbl[ev].name;
	}

	/* fsm internal event */
	if (event & FSM_INT_EV_MASK) {
		ev = (u8)(event & ~(FSM_INT_EV_MASK));
		return int_event_tbl[ev].name;
	}

	if (event == FSM_EV_UNKNOWN)
		return "FSM_EV_UNKNOWN";

	if (event > fsm->tb.max_event)
		return "undefine";

	/* user event */
	return fsm->tb.evt_tbl[event].name;
}

static char *fsm_state_name(struct fsm_main *fsm, u8 state)
{
	if (state > fsm->tb.max_state)
		return "unknown";

	return fsm->tb.state_tbl[state].name;
}

/* For EXTERNAL application to get custom obj (expose)
 *
 * @obj: cid to get obj
 */
void *rtw_fsm_get_obj(struct fsm_main *fsm, u16 cid)
{
	struct fsm_obj *obj = NULL;

	obj = fsm_get_obj(fsm, 0, cid);

	if (obj)
		return obj->custom_obj;
	return NULL;
}

/**  init obj internal variable
 *
 * @fsm: fsm that object belonged to
 * default init to the 1st state in state_tbl

 */
static void fsm_obj_switch_in(struct fsm_obj *obj, struct fsm_msg *msg)
{
	struct fsm_main *fsm = obj->fsm;

	if (obj->fsm->tb.active_obj)
		obj->fsm->tb.active_obj(obj->fsm->tb.priv, obj->custom_obj);

	/* default init to the 1st state in state_tbl */
	obj->state = fsm->tb.state_tbl[0].state;

	FSM_DBG_(fsm, "%s %-21s %s(cid=0x%04x oid=0x%08x)\n", obj->name,
		fsm_state_name(fsm, obj->state), "new obj", obj->cid, obj->oid);

	obj->valid = 1;
	/* Make it alive! Hello obj! */
	fsm_state_run(obj, FSM_EV_STATE_IN, msg->param);
}

/**  deinit obj internal variable
 *
 * @fsm: fsm that object belonged to
 * default init to the 1st state in state_tbl

 */
static void fsm_obj_switch_out(struct fsm_obj *obj, struct fsm_msg *msg)
{
	struct fsm_main *fsm = obj->fsm;

	if (fsm->tb.deactive_obj)
		fsm->tb.deactive_obj(fsm->tb.priv, obj->custom_obj);

	fsm_state_run(obj, FSM_EV_STATE_OUT, msg->param);

	FSM_DBG_(fsm, "%s %-21s %s(cid=0x%04x oid=0x%08x)\n", obj->name,
		fsm_state_name(fsm, obj->state), "del obj", obj->cid, obj->oid);

	obj->valid = 0;
	/* Let it go! Goodbye obj! */
	fsm_destory_obj(obj);
}

/* For EXTERNAL application to new a fsm object (expose)
 *
 * @fsm: fsm that object belonged to
 * @fsm_obj: obj param when calling FSM framework function
 * @priv_len: custom obj length
 *
 * return value: NULL	:fail
 *		 other	:cusomer obj handler (success)
 */
void *rtw_fsm_new_obj(struct fsm_main *fsm,
	struct sta_info *psta, u16 cid, void **custom_obj, int sz)
{
	struct fsm_obj *obj;
	struct fsm_clocks *clocks;
	int i;

	*custom_obj = NULL;
	obj = (struct fsm_obj *)rtw_zmalloc(sizeof(*obj));
	if (obj == NULL)
		return NULL;
	_rtw_memset(obj, 0, sizeof(*obj));

	clocks = (struct fsm_clocks *)rtw_zmalloc(sizeof(*clocks));
	if (clocks == NULL) {
		rtw_mfree(obj, sizeof(*obj));
		return NULL;
	}
	_rtw_memset(clocks, 0, sizeof(*clocks));
	clocks->obj = obj;
	for (i = 0; i < CLOCK_NUM; i++)
		clocks->clk[i].remain = -1; /* Negative means disable */

	obj->custom_obj = rtw_zmalloc(sz);
	if (obj->custom_obj == NULL) {
		rtw_mfree(clocks, sizeof(*clocks));
		rtw_mfree(obj, sizeof(*obj));
		return NULL;
	}
	_rtw_memset(obj->custom_obj, 0, sz);
	obj->clocks = clocks;
	obj->custom_len = sz;
	obj->oid = fsm_new_oid(fsm);
	obj->cid = cid;
	obj->fsm = fsm;
	obj->state = fsm->tb.state_tbl[0].state;
	obj->psta = psta;

	_rtw_memset(obj->name, 0, FSM_NAME_LEN);
	snprintf(obj->name, FSM_NAME_LEN,
		"%s-%04x", obj->fsm->name, obj->cid);
	*custom_obj = obj->custom_obj;
	fsm_enqueue_list(&fsm->obj_queue, &obj->list);
	fsm_enqueue_list(&fsm->clk_queue, &clocks->list);

	return obj;
}

/* For EXTERNAL application to destory a fsm object (expose)
 *
 * @fsm_obj: obj param when calling FSM framework function
 */
static void fsm_destory_obj(struct fsm_obj *obj)
{
	struct fsm_main *fsm = obj->fsm;
	struct fsm_clocks *clocks = obj->clocks;

	list_del(&obj->list);

	/* free clocks */
	_rtw_spinlock_bh(&fsm->clk_queue.lock);
	list_del(&clocks->list);
	_rtw_spinunlock_bh(&fsm->clk_queue.lock);

	rtw_mfree(clocks, sizeof(struct fsm_clocks));

	/* free custom_obj */
	rtw_mfree(obj->custom_obj, obj->custom_len);

	/* free fsm_obj */
	rtw_mfree(obj, sizeof(*obj));
}

void fsm_del_obj(struct fsm_obj *obj)
{
	/* only allow to del an inactive obj */
	if (obj->valid != 0) {
		FSM_ERR_(obj->fsm, "%s: free an actived obj\n", fsm_obj_name(obj));
		return;
	}
	fsm_destory_obj(obj);
}

void fsm_activate_obj(struct fsm_obj *obj)
{
	fsm_gen_msg(obj, NULL, 0, FSM_EV_NEW_OBJ);
}

void fsm_deactivate_obj(struct fsm_obj *obj)
{
	fsm_gen_msg(obj, NULL, 0, FSM_EV_DEL_OBJ);
}

#if 0
bool fsm_is_alarm_off_ext(struct fsm_obj *obj, u8 id)
{
	struct fsm_clocks *clocks = obj->clocks;
	struct fsm_clk *clk = &clocks->clk[id];
	bool is_clk_off;

	_rtw_spinunlock_bh(&obj->fsm->clk_queue.lock);
	is_clk_off = IS_CLK_OFF(clk);
	_rtw_spinunlock_bh(&obj->fsm->clk_queue.lock);

	return is_clk_off;
}

bool fsm_is_alarm_off(struct fsm_obj *obj)
{
	struct fsm_clocks *clocks = obj->clocks;
	struct fsm_clk *clk = &clocks->clk[0];
	bool is_clk_off;

	_rtw_spinunlock_bh(&obj->fsm->clk_queue.lock);
	is_clk_off = IS_CLK_OFF(clk);
	_rtw_spinunlock_bh(&obj->fsm->clk_queue.lock);

	return is_clk_off;
}
#endif

static void _fsm_set_alarm(struct fsm_obj *obj, int ms,
	u16 event, u8 id, void *priv)
{
	struct fsm_clocks *clocks = obj->clocks;
	struct fsm_clk *clk = &clocks->clk[id];
	u32 now;

#if 0
	if (ms == 0)
		fsm_post_event(obj, event, priv);
#endif

	now = rtw_systime_to_ms(rtw_get_current_time());

	_rtw_spinlock_bh(&obj->fsm->clk_queue.lock);
	/* turn on clock from off */
	if (IS_CLK_OFF(clk))
		obj->fsm->en_clock_num++;

	clk->event = event;
	clk->priv = priv;
	clk->start = now;
	clk->end = now + ms;
	clk->remain = (int)rtw_fsm_time_left(clk->start, clk->end);
	_rtw_spinunlock_bh(&obj->fsm->clk_queue.lock);
#if 0
	FSM_DBG_(obj->fsm, "%s:%s now=0x%08x, end=0x%08x, remain=0x%08x\n",
		fsm_obj_name(obj), fsm_evt_name(obj->fsm, event),
		clk->start, clk->end, clk->remain);
#endif
}

/* For EXTERNAL application to extend alarm time (expose)
 *
 * @obj: obj param when calling FSM framework function
 * @event: alarm will issue this event while timer expired
 * @ms: time period for the alarm
 *	remain time does not less than 'ms'
 * @id: alarm id; start from 1
 */
#if 0
void fsm_extend_alarm_ext(struct fsm_obj *obj, int ms, u8 id)

{
	struct fsm_clocks *clocks = obj->clocks;
	struct fsm_clk *clk = &clocks->clk[id];
	int remain = ms;

	if (id == 0 || id >= CLOCK_NUM) {
		RTW_ERR("%s: %s_%d fail\n",
			fsm_obj_name(obj), __func__, id);
		return;
	}

	if (IS_CLK_OFF(clk))
		return;

	remain = MAX((int)rtw_fsm_time_left(clk->start, clk->end), ms);
	fsm_set_alarm_ext(obj, remain, clk->event, id, clk->priv);
}
#endif

/* For EXTERNAL application to setup alarm (expose)
 *
 * @obj: obj param when calling FSM framework function
 * @event: alarm will issue this event while timer expired
 * @ms: time period for the alarm
 * @id: alarm id; start from 1
 */
void fsm_set_alarm(struct fsm_obj *obj, int ms, u16 event)
{
	_fsm_set_alarm(obj, ms, event, 0, NULL);
}

/* For EXTERNAL application to setup alarm_ext (expose)
 *
 * @obj: obj param when calling FSM framework function
 * @event: alarm will issue this event while timer expired
 * @ms: time period for the alarm
 * @id: alarm id; start from 1
 * @priv: priv from caller
 */
void fsm_set_alarm_ext(struct fsm_obj *obj,
	int ms, u16 event, u8 id, void *priv)
{
	if (id >= CLOCK_NUM) {
		RTW_ERR("%s: set alarm_ext_%d to %d ms fail\n",
			fsm_obj_name(obj), id, ms);
		return;
	}
	_fsm_set_alarm(obj, ms, event, id, priv);
}

static void _fsm_cancel_alarm(struct fsm_obj *obj, u8 id)
{
	struct fsm_clocks *clocks = obj->clocks;
	struct fsm_clk *clk = &clocks->clk[id];

	_rtw_spinlock_bh(&obj->fsm->clk_queue.lock);
	/* turn off clock from on */
	if (IS_CLK_ON(clk))
		obj->fsm->en_clock_num--;

	clk->end = 0;
	clk->remain = -1;
	clk->pause = 0;
	_rtw_spinunlock_bh(&obj->fsm->clk_queue.lock);
}

/* For EXTERNAL application to cancel alarm (expose)
 *
 * @obj: obj param when calling FSM framework function
 */
void fsm_cancel_alarm(struct fsm_obj *obj)
{
	_fsm_cancel_alarm(obj, 0);
}

/* For EXTERNAL application to cancel alarm_ext (expose)
 *
 * @obj: obj param when calling FSM framework function
 * @id: alarm id; start from 1
 */
void fsm_cancel_alarm_ext(struct fsm_obj *obj, u8 id)
{
	if (id == 0 || id >= CLOCK_NUM) {
		RTW_ERR("%s: cancel alarm_ext_%d fail\n",
			fsm_obj_name(obj), id);
		return;
	}
	_fsm_cancel_alarm(obj, id);
}

#if 0
static void _fsm_pause_alarm(struct fsm_obj *obj, u8 id)
{
	struct fsm_clocks *clocks = obj->clocks;
	struct fsm_clk *clk = &clocks->clk[id];

	_rtw_spinlock_bh(&obj->fsm->clk_queue.lock);
	clk[id].pause = 1;
	_rtw_spinunlock_bh(&obj->fsm->clk_queue.lock);
}

/* For EXTERNAL application to pause alarm (expose)
 *
 * @obj: obj param when calling FSM framework function
 */
void fsm_pause_alarm(struct fsm_obj *obj)
{
	_fsm_pause_alarm(obj, 0);
}

/* For EXTERNAL application to pause alarm_ext (expose)
 *
 * @obj: obj param when calling FSM framework function
 * @id: alarm id; start from 1
 */
void fsm_pause_alarm_ext(struct fsm_obj *obj, u8 id)
{
	if (id == 0 || id >= CLOCK_NUM) {
		RTW_ERR("%s: pause alarm_%d fail\n", fsm_obj_name(obj), id);
		return;
	}
	_fsm_pause_alarm(obj, id);
}

static void _fsm_resume_alarm(struct fsm_obj *obj, u8 id)
{
	struct fsm_clocks *clocks = obj->clocks;
	struct fsm_clk *clk = &clocks->clk[id];
	u32 cur = rtw_systime_to_ms(rtw_get_current_time());

	/* extrend end time */
	_rtw_spinlock_bh(&obj->fsm->clk_queue.lock);
	clk->end = cur + clk->remain;
	clk->pause = 0;
	_rtw_spinunlock_bh(&obj->fsm->clk_queue.lock);
}

/* For EXTERNAL application to resume alarm (expose)
 *
 * @obj: obj param when calling FSM framework function
 */
void fsm_resume_alarm(struct fsm_obj *obj)
{
	_fsm_resume_alarm(obj, 0);
}

/* For EXTERNAL application to resume alarm_ext (expose)
 *
 * @obj: obj param when calling FSM framework function
 * @id: alarm id; start from 1
 */
void fsm_resume_alarm_ext(struct fsm_obj *obj, u8 id)
{
	if (id == 0 || id >= CLOCK_NUM) {
		RTW_ERR("%s: resume alarm_ext_%d fail\n",
			fsm_obj_name(obj), id);
		return;
	}
	_fsm_resume_alarm(obj, id);
}
#endif

/* For EXTERNAL application to change state (expose)
 *
 * @obj: obj that changes state
 * @new_state: new state
 */
void fsm_state_goto(struct fsm_obj *obj, u8 new_state)
{
	struct fsm_main *fsm = NULL;

	if (obj->state == new_state)
		return;

	fsm = obj->fsm;

	fsm_state_run(obj, FSM_EV_STATE_OUT, NULL);

	FSM_MSG_(fsm, FSM_LV_DBG, "\n");
	FSM_MSG_(fsm, FSM_LV_DBG, "%s %-18s -> %s\n", obj->name,
		fsm_state_name(fsm, obj->state),
		fsm_state_name(fsm, new_state));

	obj->state = new_state; /* new state */
	fsm_state_run(obj, FSM_EV_STATE_IN, NULL);
}

static int fsm_int_evt_handler(struct fsm_obj *obj, struct fsm_msg *msg)
{
	switch(msg->event) {
	case FSM_EV_NEW_OBJ:
		fsm_obj_switch_in(obj, msg);
		break;
	case FSM_EV_DEL_OBJ:
		fsm_obj_switch_out(obj, msg);
		break;
	default:
		FSM_WARN_(obj->fsm, "unexpect event %04x(%s)\n",
			msg->event, fsm_evt_name(obj->fsm, msg->event));
		return _FALSE;
		break;
	}
	return _SUCCESS;
}

static void fsm_free_msg(struct fsm_msg *msg)
{
	if (msg && msg->param && msg->param_sz)
		rtw_mfree((void *)msg->param, msg->param_sz);
#ifndef USE_PHL_CMD_DISPR
	if (msg)
		rtw_mfree((void *)msg, sizeof(*msg));
#endif

}

static void fsm_user_evt_handler(struct fsm_main *fsm)
{
	struct fsm_msg *msg;
	struct fsm_obj *obj;

	while ((msg = fsm_dequeue_msg(fsm)) != NULL) {

		obj = fsm_get_obj(fsm, msg->oid, 0);

		if (obj == NULL) {
			FSM_WARN_(fsm, "%s: obj (cid=0x%04x, oid=0x%8x) not found, ignore %s\n",
				fsm->name, msg->cid, msg->oid, fsm_evt_name(fsm, msg->event));
			fsm_free_msg(msg);
			continue;
		}

		if (msg->event & FSM_INT_EV_MASK) {
			/* Allow INT_EV even FSM_STATUS_DISABLE
			 * for ABORT and DEL_OBJ to run. Free ALL user and
			 * Global evt to empty msg queue.
			 */
			if (fsm_int_evt_handler(obj, msg)) {
				fsm_free_msg(msg);
				continue;
			}
		}

		if ((fsm_status(fsm) != FSM_STATUS_ENABLE)) {
			fsm_free_msg(msg);
			continue;
		}

		/* pre handle events */
		switch(msg->event) {
		case FSM_EV_CONNECTED:
		case FSM_EV_DISCONNECTED:
			if (msg->param && !msg->param_sz)
				obj->psta = (struct sta_info *)msg->param;
			break;
		default:
			break;
		}

		if (!obj->valid) {
			FSM_WARN_(fsm, "%s: invalid, ignore %s\n",
				fsm_obj_name(obj), fsm_evt_name(fsm, msg->event));
			fsm_free_msg(msg);
			continue;
		}

		/* run state machine */
		fsm_state_run(obj, msg->event, msg->param);
		fsm_free_msg(msg);
	}
	return;
}

static int fsm_handler(struct fsm_main *fsm)
{
	/* USER EVENT */
	if (list_empty(&fsm->msg_queue.q))
		return 0;

	fsm_user_evt_handler(fsm);

	return 0;
}

/* For EXTERNAL application to get fsm name (expose)
 * @fsm: fsm to be get name
 */
char *rtw_fsm_fsm_name(struct fsm_main *fsm)
{
	return fsm->name;
}

/* For EXTERNAL application to get obj name (expose)
 * @obj: obj to be get name
 * For example: scan-1 (sacn obj with object id 1)
 */
char *fsm_obj_name(struct fsm_obj *obj)
{
	return obj->name;
}

/* For EXTERNAL application to cancel sma (expose)
 * @obj: obj job will be cancelled
 */
int fsm_abort_obj(struct fsm_obj *obj)
{
	struct fsm_msg *msg;

	/* NEW message to cancel obj task */
	msg = fsm_new_msg(obj, FSM_EV_ABORT);
	if (msg == NULL) {
		RTW_ERR("%s: alloc msg fail\n", obj->fsm->name);
		return _FAIL;
	}
	return fsm_send_msg(obj->fsm, msg);
}

int fsm_init_custom(struct fsm_priv *fsmpriv)
{
	int rtn = _FAIL;

#ifdef CONFIG_RTW_FSM_XXX
	if (rtw_xxx_reg_fsm(fsmpriv) == _FAIL)
		goto exit;
#endif
#ifdef CONFIG_RTW_FSM_RRM
	if (rtw_rrm_reg_fsm(fsmpriv) == _FAIL)
		goto exit;
#endif
#ifdef CONFIG_RTW_FSM_BTM
	if (rtw_btm_reg_fsm(fsmpriv) == _FAIL)
		goto exit;
#endif
	rtn = _SUCCESS;;
exit:
	return rtn;
}

void fsm_deinit_custom(struct fsm_priv *fsmpriv)
{
	struct fsm_root *root = fsmpriv->root;
	struct fsm_main *fsm, *fsm_t;

	if (list_empty(&root->q_share_thd.q))
		return;

	list_for_each_entry_safe(fsm, fsm_t, &root->q_share_thd.q, list) {
		list_del(&fsm->list);
		/* csutom deinit fsm */
		if (fsm->tb.deinit_priv)
			fsm->tb.deinit_priv(fsm->tb.priv);
		fsm_deinit_fsm(fsm);
	}
}

/* For EXTERNAL application to init FSM framework (expose) */
/* @obj: obj job will be cancelled
 */
int rtw_fsm_init(struct fsm_priv *fsmpriv, void *priv)
{
	struct fsm_root *root;
	int max, i, g, size;

	/* check size of internal event table */
	i = FSM_EV_INT_MAX & ~FSM_INT_EV_MASK;
	g = FSM_EV_GBL_MAX & ~FSM_GBL_EV_MASK;
	max = i + g + 2;
	size = sizeof(int_event_tbl)/sizeof(int_event_tbl)[0];
	if (size != max) {
		RTW_ERR("fsm: int_event_tbl[%d] != %d size mismatch!!",
			size, max);
		WARN_ON(1);
		return _FAIL;
	}
	root = (struct fsm_root *)rtw_zmalloc(sizeof(*root));
	if (root == NULL)
		return _FAIL;

	_rtw_memset(root, 0, sizeof(*root));
	fsm_init_queue(&(root->q_share_thd));
	_rtw_init_sema(&root->msg_ready, 0);
	root->a = priv;
	root->oid_seq = 1;

	fsmpriv->root = root;

	RTW_INFO("fsm: [FSM] %s()\n", __func__);

	return fsm_init_custom(fsmpriv);
}

/* For EXTERNAL application to deinit FSM framework (expose)
 * @root: FSM framework handler
 */
void rtw_fsm_deinit(struct fsm_priv *fsmpriv)
{
	_adapter *a;
	void *c = NULL;
	struct fsm_root *root = fsmpriv->root;

	if (!root) {
		RTW_ERR("fsm: root hasn't initialized!\n");
		return;
	}
	fsm_deinit_custom(fsmpriv);
	a = (_adapter *)root->a;

	FSM_INFO_(c, "fsm: [FSM] "FUNC_NDEV_FMT"\n",FUNC_NDEV_ARG(a->pnetdev));

	fsm_deinit_queue(&(root->q_share_thd));
	_rtw_free_sema(&root->msg_ready);

	/* free fsm_root */
	rtw_mfree(root, sizeof(*root));
	fsmpriv->root = NULL;

}

void fsm_start_custom(struct fsm_priv *fsmpriv)
{
	struct fsm_root *root = fsmpriv->root;
	struct fsm_main *fsm, *fsm_t;

	if (list_empty(&root->q_share_thd.q))
		return;

	list_for_each_entry_safe(fsm, fsm_t, &root->q_share_thd.q, list) {
		fsm_start_fsm(fsm);
	}
}

void fsm_stop_custom(struct fsm_priv *fsmpriv)
{
	struct fsm_root *root = fsmpriv->root;
	struct fsm_main *fsm, *fsm_t;
	int sleep_count;

	if (list_empty(&root->q_share_thd.q))
		return;

	list_for_each_entry_safe(fsm, fsm_t, &root->q_share_thd.q, list) {
		fsm_stop_fsm(fsm);
	}
	list_for_each_entry_safe(fsm, fsm_t, &root->q_share_thd.q, list) {
		sleep_count = 100;
		/* Max wait 1000ms until queue is empty */
		while (!list_empty(&fsm->msg_queue.q) && sleep_count--)
			rtw_msleep_os(10);

		if (sleep_count <= 0) {
			FSM_WARN_(fsm, "msg queue is not empty!\n");
		}
		fsm_remove_all_queuing_msg(fsm);
	}
}

/* For EXTERNAL application to start fsm root (expose)
 * @fsm: see struct fsm_main
 */
void rtw_fsm_start(struct fsm_priv *fsmpriv)
{
	struct fsm_root *root = fsmpriv->root;
	_adapter *a = (_adapter *)root->a;

#ifndef USE_PHL_CMD_DISPR
	if (root->thread) {
		RTW_WARN("fsm: thread is running\n");
		return;
	}
	root->thread_should_stop = 0;
	root->thread = kthread_create(fsm_thread_share, root, "fsm_thread");
	wake_up_process(root->thread);
#endif
	RTW_INFO("fsm: [FSM] "FUNC_NDEV_FMT"\n", FUNC_NDEV_ARG(a->pnetdev));

	fsm_start_custom(fsmpriv);
	return;
}

/* For EXTERNAL application to stop fsm root (expose)
 * @fsm: see struct fsm_main
 */
void rtw_fsm_stop(struct fsm_priv *fsmpriv)
{
	void *c = NULL;
	struct fsm_root *root = fsmpriv->root;
	_adapter *a = (_adapter *)root->a;

#ifdef USE_PHL_CMD_DISPR
	fsm_stop_custom(fsmpriv);
#else
	if (!root->thread) {
		RTW_INFO("fsm: thread was down\n");
		return;
	}
	fsm_stop_custom(fsmpriv);

	root->thread_should_stop = 1;
	_rtw_up_sema(&root->msg_ready);

	kthread_stop(root->thread);
	root->thread = NULL;
#endif
	RTW_INFO("fsm: [FSM] "FUNC_NDEV_FMT"\n", FUNC_NDEV_ARG(a->pnetdev));

	return;
}

static int fsm_start_fsm(struct fsm_main *fsm)
{
	_adapter *a = fsm_to_adapter(fsm);

	_set_timer(&fsm->fsm_timer, CLOCK_UNIT);
	fsm_status_set(fsm, FSM_STATUS_ENABLE);

	FSM_INFO_(fsm, "fsm: [%s] "FUNC_NDEV_FMT"\n", fsm->name, FUNC_NDEV_ARG(a->pnetdev));

	return _SUCCESS;
}

static int fsm_stop_fsm(struct fsm_main *fsm)
{
	struct fsm_obj *obj;
	_adapter *a = fsm_to_adapter(fsm);

	_cancel_timer_ex(&fsm->fsm_timer);

	/* Stop receiving message */
	fsm_status_set(fsm, FSM_STATUS_DISABLE);

	/* ABORT all objs within fsm */
	fsm_abort_all_running_obj(fsm);

	FSM_INFO_(fsm, "fsm: [%s] "FUNC_NDEV_FMT"\n", fsm->name, FUNC_NDEV_ARG(a->pnetdev));

	return _SUCCESS;
}

/* For EXTERNAL application to generate message buffer (expose)
 * Generate message quickly and simply
 * @obj: fsm_obj (msg receiver)
 * @pbuf: message parameter
 * @sz: message parameter size
 * @event: event for the message
 */
int fsm_gen_msg(struct fsm_obj *obj, void *pbuf, u32 sz, u16 event)
{
	void *param = NULL;
	struct fsm_msg *msg;

	/* NEW mem for message */
	msg = fsm_new_msg(obj, event);
	if (msg == NULL) {
		FSM_ERR_(obj->fsm, "%s: alloc msg %s fail\n",
			fsm_obj_name(obj),
			fsm_evt_name(obj->fsm, event));
		goto msg_fail;
	}

	/* NEW mem for param */
	if (pbuf && sz) {
		param = rtw_zmalloc(sz);
		if (param == NULL) {
			FSM_ERR_(obj->fsm,
				"%s: alloc param %s fail\n",
				fsm_obj_name(obj),
				fsm_evt_name(obj->fsm, event));
			rtw_mfree(msg, sizeof(*msg));
			goto msg_fail;
		}
		_rtw_memcpy(param, pbuf, sz);
		msg->param = (void *)param;
		msg->param_sz = sz;
	} else {
		msg->param = (void *)pbuf;
		msg->param_sz = sz;
	}

	return fsm_send_msg(obj->fsm, msg);

msg_fail:
	return _FAIL;
}

static void fsm_bcast_msg(struct fsm_priv *fsmpriv,
	struct sta_info *psta, char *pbuf, u32 sz, u16 event)
{
	struct fsm_root *root = fsmpriv->root;
	struct fsm_main *fsm;
	struct fsm_obj *obj;

	list_for_each_entry(fsm, &root->q_share_thd.q, list) {
		if (fsm_status(fsm) != FSM_STATUS_ENABLE)
			continue;
		list_for_each_entry(obj, &fsm->obj_queue.q, list) {
			if (!psta || (psta && obj->psta == psta))
				fsm_gen_msg(obj, pbuf, sz,  event);
		}
	}
}

void rtw_fsm_notify_connect(struct fsm_priv *fsmpriv, struct sta_info *psta, int res)
{
	_adapter *a = fsmpriv->root->a;
	struct sta_info *pself;

	pself = rtw_get_stainfo(&a->stapriv, a->phl_role->mac_addr);
	if (res >= 0) /* success */
		fsm_bcast_msg(fsmpriv, pself, (char *)psta, 0, FSM_EV_CONNECTED);
	else
		fsm_bcast_msg(fsmpriv, pself, (char *)psta, 0, FSM_EV_CONNECT_FAIL);
}

void rtw_fsm_notify_disconnect(struct fsm_priv *fsmpriv, struct sta_info *psta)
{
	_adapter *a = fsmpriv->root->a;
	struct sta_info *pself;

	pself = rtw_get_stainfo(&a->stapriv, a->phl_role->mac_addr);
	if (pself)
		fsm_bcast_msg(fsmpriv, psta, (char *)pself, 0, FSM_EV_DISCONNECTED);
}

void rtw_fsm_notify_scan_start(struct fsm_priv *fsmpriv, struct sta_info *psta)
{
	fsm_bcast_msg(fsmpriv, psta, NULL, 0, FSM_EV_SCAN_START);
}

void rtw_fsm_notify_scan_done(struct fsm_priv *fsmpriv, struct sta_info *psta)
{
	fsm_bcast_msg(fsmpriv, psta, NULL, 0, FSM_EV_SCAN_DONE);
}

/** Debug funcitons
 *
 */
#ifdef RTW_DEBUG_FSM
static void fsm_dbg_dump_fsm_queue(struct fsm_queue *fsmq,
	char *s, int *sz,bool detail)
{
	struct fsm_main *fsm, *fsm_t;

	char *ptr = s;
	int len = *sz;

	list_for_each_entry(fsm, &fsmq->q, list) {
		_os_snprintf(pstr(ptr), lstr(ptr, len), "\t%4s : %s\n", fsm->name,
			fsm->tb.mode ? "STANDALONE":"SHARE");

		if (fsm->tb.dump_fsm && detail) {
			len = lstr(ptr, len);
			ptr = pstr(ptr);
			fsm->tb.dump_fsm(fsm, ptr, &len);
		}
	}
	*sz = len;
}

static void fsm_dbg_help(struct fsm_main *fsm, char *s, int *sz, bool detail);
static void fsm_dbg_dump_fsm(struct fsm_main *fsm,
	char *s, int *sz, bool detail)
{
	int len = *sz;
	char *ptr = s;

	_os_snprintf(pstr(ptr), lstr(ptr, len), "\t%4s : %s\n", fsm->name,
		fsm->tb.mode ? "STANDALONE":"SHARE");

	if (fsm->tb.dump_fsm && detail) {
		len = lstr(ptr, len);
		ptr = pstr(ptr);
		fsm->tb.dump_fsm(fsm, ptr, &len);
	}

}

static void fsm_dbg_dump_state(struct fsm_main *fsm,
	char *s, int *sz, bool detail)
{
	int i;
	int len = *sz;

	_os_snprintf(pstr(s), lstr(s, len),
		"[%s] state table\n", fsm->name);
	for (i = 0; i < fsm->tb.max_state; i++)
		_os_snprintf(pstr(s), lstr(s, len), "\t%4d : %s\n",
			i, fsm->tb.state_tbl[i].name);
	*sz = len;
}

static void fsm_dbg_dump_event(struct fsm_main *fsm,
	char *s, int *sz, bool detail)
{
	int i, max;
	int len = *sz;

	/* internal event */
	_os_snprintf(pstr(s), lstr(s, len), "[Internal] event table\n");

	max = FSM_EV_END & ~(int_event_tbl[0].event); /* FSM_INT_EV_MASK */
	for (i = 1; i < max; i++)
		_os_snprintf(pstr(s), lstr(s, len), "\t0x%4x : %s\n",
			int_event_tbl[i].event, int_event_tbl[i].name);

	/* user event */
	_os_snprintf(pstr(s), lstr(s, len), "\n[%s] event table max %d\n", fsm->name, fsm->tb.max_event);
	for (i = 0; i < fsm->tb.max_event-1; i++)
		_os_snprintf(pstr(s), lstr(s, len), "\t0x%4x : %s\n",
			fsm->tb.evt_tbl[i].event, fsm->tb.evt_tbl[i].name);
	*sz = len;
}

static void fsm_dbg_dump_obj(struct fsm_main *fsm,
	char *s, int *sz, bool detail)
{
	struct fsm_obj *obj, *obj_t;
	int len = *sz;
	char *ptr = s;

	list_for_each_entry(obj, &fsm->obj_queue.q, list) {

		_os_snprintf(pstr(ptr), lstr(ptr, len), "%s : state %s",
			obj->name, fsm_state_name(fsm, obj->state));

		if (fsm->tb.dump_obj && detail) {
			len = lstr(ptr, len);
			ptr = pstr(ptr);
			fsm->tb.dump_obj(obj->custom_obj, ptr, &len);
		}
	}
	*sz = len;
}

static void fsm_dbg_max(struct fsm_main *fsm, char *s, int *sz, bool detail)
{
	int len = *sz;

	_os_snprintf(pstr(s), lstr(s, len),
		"ERR: fsm %s sould not run to here!!\n", __func__);
	*sz = len;
}

struct fsm_debug_ent {
	char *opt;
	void (*func)(struct fsm_main *fsm, char *s, int *sz, bool detail);
	char *desc;
};

struct fsm_debug_ent debug_opt[] = {
	{"help", fsm_dbg_help, "help message"},
	{"fsm", fsm_dbg_dump_fsm, "all fsm name"},
	{"st", fsm_dbg_dump_state, "state name"},
	{"ev", fsm_dbg_dump_event, "event name"},
	{"obj", fsm_dbg_dump_obj, "obj detail"},
	{"max", fsm_dbg_max, "max_opt"}
};

static void _fsm_dbg_help(struct fsm_root *root, char *s, int *sz, bool detail)
{
	int i, max_opt;
	int len = *sz;
	char *ptr = s;

	_os_snprintf(pstr(ptr), lstr(ptr, len),
		"usage:\tfsm d <fsm_name> <option>\n");
	_os_snprintf(pstr(ptr), lstr(ptr, len),
		"\tfsm p,<obj_name> <priv_dbg_cmd> ....\n");
	_os_snprintf(pstr(ptr), lstr(ptr, len),
		"\tfsm s,<obj_name> <EVENT>\n");
	_os_snprintf(pstr(ptr), lstr(ptr, len),
		"\tfsm w,<fsm_name> <dbg_level|ev_level> <0-5(dbg)>\n");

	_os_snprintf(pstr(s), lstr(ptr, len), "\nfsm_name:\n");

	len = lstr(ptr, len);
	ptr = pstr(ptr);
	fsm_dbg_dump_fsm_queue(&root->q_share_thd, ptr, &len, detail);

	_os_snprintf(pstr(ptr), lstr(ptr, len), "\noption:\n");
	max_opt = sizeof(debug_opt)/sizeof(debug_opt[0]);
	for (i = 0; i < max_opt-1; i++)
		_os_snprintf(pstr(ptr), lstr(ptr, len), "%12s : %s\n",
			debug_opt[i].opt, debug_opt[i].desc);
	*sz = len;
}

static void fsm_dbg_help(struct fsm_main *fsm, char *s, int *sz, bool detail)
{
	_fsm_dbg_help(fsm->root, s, sz, false);
}

struct fsm_main *get_fsm_by_name(struct fsm_root *root, char *name)
{
	struct fsm_main *fsm, *fsm_t;
	u32 len = strlen((u8 *)name);

	if (len > FSM_NAME_LEN)
		return NULL;

	list_for_each_entry(fsm, &root->q_share_thd.q, list) {
		if (strlen((u8 *)fsm->name) == len &&
			_os_mem_cmp(d, fsm->name, name, len) == 0)
			return fsm;
	}

	return NULL;
}

static u16 fsm_get_evt_id(struct fsm_main *fsm, char *event)
{
	int i;
	u32 len = strlen((u8 *)event);

	/* internal event */
	for (i = 0; i < (sizeof(int_event_tbl)/sizeof(int_event_tbl[0])); i++) {
		if (strlen((u8 *)int_event_tbl[i].name) == len &&
			_os_mem_cmp(d, int_event_tbl[i].name, event, len) == 0)
			return int_event_tbl[i].event;
	}

	/* user event */
	for (i = 0; i < fsm->tb.max_event; i++) {
		if (strlen((u8 *)fsm->tb.evt_tbl[i].name) == len &&
			_os_mem_cmp(d,
				fsm->tb.evt_tbl[i].name, event, len) == 0)
			return fsm->tb.evt_tbl[i].event;
	}
	return FSM_EV_UNKNOWN;
}
#endif /* RTW_DEBUG_FSM */

/* For EXTERNAL application to debug fsm (expose)
 * @input: input cmd
 * @input_num: num of cmd param
 * @output: output buffer
 * @out_len: MAX output buffer len
 *
 * d: dump fsm info
 *	fsm <d> <fsm_name> <fsm|st|ev|obj>
 * p: private cmd to fsm module
 *	fsm <p> <obj_name> <cmd to fsm module>
 * s: send event to fsm
 *	fsm <s> <obj_name> <ev>
 * w: write debug level
 *	fsm <w> <fsm_name> <dbg_level|evt_level> <0-5>
 */
void rtw_fsm_dbg(void *a, char input[][MAX_ARGV],
		      u32 input_num, char *output, u32 out_len)
{
#ifdef RTW_DEBUG_FSM
	//struct phl_info_t *phl = (struct phl_info_t *)phl_info;
	struct fsm_root *root = a->fsmpriv.root;
	struct fsm_main *fsm = NULL;
	struct fsm_obj *obj = NULL;
	struct fsm_msg *msg;
	int i, max_opt, len = out_len;
	char fsm_name[FSM_NAME_LEN], opt[FSM_NAME_LEN], cmd[FSM_NAME_LEN];
	char c, *ptr, *sp;
	u8 obj_id = 0;
	u16 ev_id;

	ptr = output;
	/* fsm <cmd> <fsm_name> <opt> : fsm d cmd ev
	 * fsm <cmd> <fsm_name> <evt> : fsm s cmd-1 FSM_EV_ABORT
	 */
	if (input_num < 4)
		goto help;

	_rtw_memset(cmd, 0, FSM_NAME_LEN);
	_rtw_memcpy(cmd, input[1],
		MIN(strlen((u8 *)input[1]), FSM_NAME_LEN));

	_rtw_memset(fsm_name, 0, FSM_NAME_LEN);
	_rtw_memcpy(fsm_name, input[2],
		MIN(strlen((u8 *)input[2]), FSM_NAME_LEN));

	_rtw_memset(opt, 0, FSM_NAME_LEN);
	_rtw_memcpy(opt, input[3],
		MIN(strlen((u8 *)input[3]), FSM_NAME_LEN));

	c = (char)*cmd;
	/* read obj_id
	 * if fsm_name is "cmd-1" then obj number is "1"
	 */
	sp = _os_strchr((const char *)fsm_name, '-');

	if (sp != NULL) {
		*sp = '\0';
		if (_os_sscanf(sp+1, "%hhd", &obj_id) != 1) {
			_os_snprintf(pstr(ptr), lstr(ptr, len),
				"ERR: fsm[%s] miss obj_id\n", fsm_name);
			return;
		}
	} else
		obj_id = 1; /* assume obj-1 */

	/* search fsm by name */
	fsm = get_fsm_by_name(root, (char *)fsm_name);
	if (fsm == NULL) {
		_os_snprintf(pstr(ptr), lstr(ptr, len),
			"ERR: fsm[%s] not found\n", fsm_name);
		return;
	}

	obj = fsm_get_obj(fsm, obj_id, 0);
	if (obj == NULL) {
		_os_snprintf(pstr(ptr), lstr(ptr, len),
			"ERR: fsm[%s] miss obj_%d\n", fsm_name, obj_id);
		return;
	}

	switch (c) {
	case 'd':
		/* dump status */
		max_opt = sizeof(debug_opt)/sizeof(debug_opt)[0];
		for (i = 0; i < max_opt-1; i++) {
			if (strlen((u8 *)debug_opt[i].opt) == \
				strlen((u8 *)opt) &&
				_os_mem_cmp(d, debug_opt[i].opt, opt,
				strlen((u8 *)opt)) == 0) {

				len = lstr(ptr, len);
				ptr = pstr(ptr);
				debug_opt[i].func(fsm, ptr, &len, true);
				break;
			}
		}
		break;

	case 'p':
		/* call fsm private degug function */
		if ((fsm != NULL) && (obj != NULL) && (fsm->tb.debug != NULL)){
			len = lstr(ptr, len);
			ptr = pstr(ptr);
			fsm->tb.debug(obj->custom_obj, &input[3],
				(input_num - 3), ptr, (u32 *)&len);
		}
		break;

	case 's':
		/* get event id */
		ev_id = fsm_get_evt_id(fsm, (char *)opt);

		if (ev_id == FSM_EV_UNKNOWN) {
			_os_snprintf(pstr(ptr), lstr(ptr, len),
				"\n\nERR: fsm[%s] unknown event %s\n",
				fsm_name, opt);
			len = lstr(ptr, len);
			ptr = pstr(ptr);
			fsm_dbg_dump_event(fsm, ptr, &len, false);
			break;
		}

		if (obj != NULL) {
			msg = fsm_new_msg(obj, ev_id);

			/* send event */
			fsm_send_msg(obj->fsm, msg);
		}

		break;

	case 'w':
		/* write cfg */
		/* fsm w,<fsm_name>,<dbg_level|ev_level>,<0-5(dbg)> */

		sp = _os_strchr((const char *)opt, ',');
		if (sp == NULL)
			goto help;

		*sp = '\0';
		if (_os_sscanf(sp+1, "%d", &i) != 1)
			goto help;

		if ((i<0) || (i>5))
			goto help;

		if (!_os_strcmp(opt, "dbg_level")) {
			fsm->tb.dbg_level = (u8)i;
			_os_snprintf(pstr(ptr), lstr(ptr, len),
				"\n%s: set debug level to %d\n",
				rtw_fsm_fsm_name(fsm), i);
		} else if (!_os_strcmp(opt, "evt_level")) {
			_os_snprintf(pstr(ptr), lstr(ptr, len),
				"\n%s: set event level to %d\n",
				rtw_fsm_fsm_name(fsm), i);
		} else
			goto help;
		break;

	default:
		goto help;
	}
	return;
help:
	len = lstr(ptr, len);
	ptr = pstr(ptr);
	_fsm_dbg_help(fsm->root, ptr, &len, false);
#endif /* RTW_DEBUG_FSM */
}
#endif /*CONFIG_RTW_FSM*/
