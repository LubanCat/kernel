/******************************************************************************
 *
 * Copyright(c) 2019 - 2023 Realtek Corporation.
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

#ifdef CONFIG_RTW_FSM_RRM

#ifdef FSM_DBG_MEM_OVERWRITE
#define _os_kmem_alloc(a, b) fsm_kmalloc(b)
#define _os_kmem_free(a, b, c) fsm_kfree(b, c)
#endif

#define pstr(s) (s +strlen((u8 *)s))
#define lstr(s, l) (size_t)(l - strlen((u8 *)s))

#if 1
#define RM_SUPPORT_IWPRIV_DBG	1

#define DBG_BCN_REQ_DETAIL	0
#define DBG_BCN_REQ_WILDCARD	0
#define DBG_BCN_REQ_SSID	0
#define DBG_BCN_REQ_SSID_NAME	"RealKungFu"

#define RM_REQ_TIMEOUT		5000	/*  5 seconds */
#if CONFIG_IEEE80211_BAND_6GHZ
#define RM_MEAS_TIMEOUT		15000	/* 15 seconds */
#else
#define RM_MEAS_TIMEOUT		10000	/* 10 seconds */
#endif
#define RM_REPT_SCAN_INTVL	5000	/*  5 seconds */
#define RM_REPT_POLL_INTVL	2000	/*  2 seconds */
#define RM_COND_INTVL		2000	/*  2 seconds */
#define RM_BUSY_TRAFFIC_TIMES	2
#define RM_WAIT_BUSY_TIMEOUT	2000	/*  1 seconds */
#define RM_NB_REP_VALID_TIME	10000	/* 10 seconds */


#define MEAS_REP_MOD_LATE	BIT(0)
#define MEAS_REP_MOD_INCAP	BIT(1)
#define MEAS_REP_MOD_REFUSE	BIT(2)

#define RM_MASTER		0x0100	/* STA who issue meas_req */
#define RM_SLAVE		0x0000	/* STA who do measurement */

/* IEEE 802.11-2012 Table 8-59 Measurement Type definitions
*  for measurement request
*  modify rm_meas_type_req_name() when adding new type
*/
enum meas_type_of_req {
	basic_req,	/* spectrum measurement */
	cca_req,
	rpi_histo_req,
	ch_load_req,
	noise_histo_req,
	bcn_req,
	frame_req,
	sta_statis_req,
	lci_req,
	meas_type_req_max,
};

/* IEEE 802.11-2012 Table 8-81 Measurement Type definitions
*  for measurement report
*  modify rm_type_rep_name() when adding new type
*/
enum meas_type_of_rep {
	basic_rep,	/* spectrum measurement */
	cca_rep,
	rpi_histo_rep,
	ch_load_rep,	/* radio measurement */
	noise_histo_rep,
	bcn_rep,
	frame_rep,
	sta_statis_rep,	/* Radio measurement and WNM */
	lci_rep,
	meas_type_rep_max
};

/*
* Beacon request
*/
/* IEEE 802.11-2012 Table 8-64 Measurement mode for Beacon Request element */
enum bcn_req_meas_mode {
	bcn_req_passive,
	bcn_req_active,
	bcn_req_bcn_table
};

/* IEEE 802.11-2012 Table 8-65 optional subelement IDs for Beacon Request */
enum bcn_req_opt_sub_id{
	bcn_req_ssid = 0,		/* len 0-32 */
	bcn_req_rep_info = 1,		/* len 2 */
	bcn_req_rep_detail = 2,		/* len 1 */
	bcn_req_req = 10,		/* len 0-237 */
	bcn_req_ap_ch_rep = 51,		/* len 1-237 */
	bcn_req_last_bcn_rpt_ind = 164	/* len 1 */
};

/* IEEE 802.11-2012 Table 8-66 Reporting condition of Beacon Report */
enum bcn_rep_cound_id{
	bcn_rep_cond_immediately,	/* default */
	bcn_req_cond_rcpi_greater,
	bcn_req_cond_rcpi_less,
	bcn_req_cond_rsni_greater,
	bcn_req_cond_rsni_less,
	bcn_req_cond_max
};

struct opt_rep_info {
	u8 cond;
	u8 threshold;
};

#define	MAX_CH_NUM_IN_OP_CLASS	11
typedef struct _RT_OPERATING_CLASS {
	int	global_op_class;
	int	Len;
	u8	Channel[MAX_CH_NUM_IN_OP_CLASS];
} RT_OPERATING_CLASS, *PRT_OPERATING_CLASS;

#define BCN_REQ_OPT_MAX_NUM		16
#define BCN_REQ_REQ_OPT_MAX_NUM		16
#define BCN_REQ_OPT_AP_CH_RPT_MAX_NUM	12
struct bcn_req_opt {
	/* all req cmd id */
	u8 opt_id[BCN_REQ_OPT_MAX_NUM];
	u8 opt_id_num;
	u8 req_id_num;
	u8 req_id[BCN_REQ_REQ_OPT_MAX_NUM];
	u8 rep_detail;
	u8 rep_last_bcn_ind;
	NDIS_802_11_SSID ssid;

	/* bcn report condition */
	struct opt_rep_info rep_cond;

	u8 ap_ch_rpt_num;
	struct _RT_OPERATING_CLASS *ap_ch_rpt[BCN_REQ_OPT_AP_CH_RPT_MAX_NUM];

	/* 0:default(Report to be issued after each measurement) */
	u8 *req_start;	/*id : 10 request;start  */
	u8 req_len;	/*id : 10 request;length */
};

/*
* channel load
*/
/* IEEE 802.11-2012 Table 8-60 optional subelement IDs for channel load request */
enum ch_load_opt_sub_id{
	ch_load_rsvd,
	ch_load_rep_info
};

/* IEEE 802.11-2012 Table 8-61 Reporting condition for channel load Report */
enum ch_load_cound_id{
	ch_load_cond_immediately,	/* default */
	ch_load_cond_anpi_equal_greater,
	ch_load_cond_anpi_equal_less,
	ch_load_cond_max
};

/*
* Noise histogram
*/
/* IEEE 802.11-2012 Table 8-62 optional subelement IDs for noise histogram */
enum noise_histo_opt_sub_id{
	noise_histo_rsvd,
	noise_histo_rep_info
};

/* IEEE 802.11-2012 Table 8-63 Reporting condition for noise historgarm Report */
enum noise_histo_cound_id{
	noise_histo_cond_immediately,	/* default */
	noise_histo_cond_anpi_equal_greater,
	noise_histo_cond_anpi_equal_less,
	noise_histo_cond_max
};

struct meas_req_opt {
	/* report condition */
	struct opt_rep_info rep_cond;
};

#define MAX_RM_PKT_NUM	32
#define MAX_RM_AP_NUM	128
struct data_buf {
	u8 *pbuf;
	u16 len;
};

/*
* Measurement
*/
struct opt_subelement {
	u8 id;
	u8 length;
	u8 *data;
};

/* 802.11-2012 Table 8-206 Radio Measurment Action field */
enum rm_action_code {
	RM_ACT_RADIO_MEAS_REQ,
	RM_ACT_RADIO_MEAS_REP,
	RM_ACT_LINK_MEAS_REQ,
	RM_ACT_LINK_MEAS_REP,
	RM_ACT_NB_REP_REQ,	/* 4 */
	RM_ACT_NB_REP_RESP,
	RM_ACT_RESV,
	RM_ACT_MAX
};

/* 802.11-2012 Table 8-119 RM Enabled Capabilities definition */
enum rm_cap_en {
	RM_LINK_MEAS_CAP_EN,
	RM_NB_REP_CAP_EN,		/* neighbor report */
	RM_PARAL_MEAS_CAP_EN,		/* parallel report */
	RM_REPEAT_MEAS_CAP_EN,
	RM_BCN_PASSIVE_MEAS_CAP_EN,
	RM_BCN_ACTIVE_MEAS_CAP_EN,
	RM_BCN_TABLE_MEAS_CAP_EN,
	RM_BCN_MEAS_REP_COND_CAP_EN,	/* conditions */

	RM_FRAME_MEAS_CAP_EN,
	RM_CH_LOAD_CAP_EN,
	RM_NOISE_HISTO_CAP_EN,		/* noise historgram */
	RM_STATIS_MEAS_CAP_EN,		/* statistics */
	RM_LCI_MEAS_CAP_EN,		/* 12 */
	RM_LCI_AMIMUTH_CAP_EN,
	RM_TRANS_STREAM_CAT_MEAS_CAP_EN,
	RM_TRIG_TRANS_STREAM_CAT_MEAS_CAP_EN,

	RM_AP_CH_REP_CAP_EN,
	RM_RM_MIB_CAP_EN,
	RM_OP_CH_MAX_MEAS_DUR0,		/* 18-20 */
	RM_OP_CH_MAX_MEAS_DUR1,
	RM_OP_CH_MAX_MEAS_DUR2,
	RM_NONOP_CH_MAX_MEAS_DUR0,	/* 21-23 */
	RM_NONOP_CH_MAX_MEAS_DUR1,
	RM_NONOP_CH_MAX_MEAS_DUR2,

	RM_MEAS_PILOT_CAP0,		/* 24-26 */
	RM_MEAS_PILOT_CAP1,
	RM_MEAS_PILOT_CAP2,
	RM_MEAS_PILOT_TRANS_INFO_CAP_EN,
	RM_NB_REP_TSF_OFFSET_CAP_EN,
	RM_RCPI_MEAS_CAP_EN,		/* 29 */
	RM_RSNI_MEAS_CAP_EN,
	RM_BSS_AVG_ACCESS_DELAY_CAP_EN,

	RM_AVALB_ADMIS_CAPACITY_CAP_EN,
	RM_ANT_CAP_EN,
	RM_RSVD,			/* 34-39 */
	RM_MAX
};
#endif

/*
* State machine
*/
enum RRM_STATE_ST {
	RRM_ST_IDLE,
	RRM_ST_DO_MEAS,
	RRM_ST_WAIT_MEAS,
	RRM_ST_SEND_REPORT,
	RRM_ST_RECV_REPORT,
	RRM_ST_END,
	RRM_ST_MAX
};

enum RM_EV_ID {
	RRM_EV_busy_timer_expire,
	RRM_EV_delay_timer_expire,
	RRM_EV_meas_timer_expire,
	RRM_EV_retry_timer_expire,
	RRM_EV_repeat_delay_expire,
	RRM_EV_request_timer_expire,
	RRM_EV_start_meas,
	RRM_EV_recv_rep,
	RRM_EV_max
};

static int rrm_idle_st_hdl(void *obj, u16 event, void *param);
static int rrm_do_meas_st_hdl(void *obj, u16 event, void *param);
static int rrm_wait_meas_st_hdl(void *obj, u16 event, void *param);
static int rrm_send_report_st_hdl(void *obj, u16 event, void *param);
static int rrm_recv_report_st_hdl(void *obj, u16 event, void *param);
static int rrm_end_st_hdl(void *obj, u16 event, void *param);

/* STATE table */
static struct fsm_state_ent rrm_state_tbl[] = {
	ST_ENT(RRM_ST_IDLE, rrm_idle_st_hdl),
	ST_ENT(RRM_ST_DO_MEAS, rrm_do_meas_st_hdl),
	ST_ENT(RRM_ST_WAIT_MEAS, rrm_wait_meas_st_hdl),
	ST_ENT(RRM_ST_SEND_REPORT, rrm_send_report_st_hdl),
	ST_ENT(RRM_ST_RECV_REPORT, rrm_recv_report_st_hdl),
	ST_ENT(RRM_ST_END, rrm_end_st_hdl),
};

/* EVENT table */
static struct fsm_event_ent rrm_event_tbl[] = {
	EV_DBG(RRM_EV_busy_timer_expire),
	EV_DBG(RRM_EV_delay_timer_expire),
	EV_DBG(RRM_EV_meas_timer_expire),
	EV_DBG(RRM_EV_retry_timer_expire),
	EV_DBG(RRM_EV_repeat_delay_expire),
	EV_DBG(RRM_EV_request_timer_expire),
	EV_DBG(RRM_EV_start_meas),
	EV_DBG(RRM_EV_recv_rep),
	EV_DBG(RRM_EV_max) /* EV_MAX for fsm safety checking */
};

struct rm_meas_req {
	u8 category;
	u8 action_code;		/* T8-206  */
	u8 diag_token;
	u16 rpt;

	u8 e_id;
	u8 len;
	u8 m_token;
	u8 m_mode;		/* req:F8-105, rep:F8-141 */
	u8 m_type;		/* T8-59 */
	u8 op_class;
	u8 ch_num;
	u16 rand_intvl;		/* units of TU */
	u16 meas_dur;		/* units of TU */
	u8 s_mode;

	u8 bssid[6];		/* for bcn_req */

	u8 *pssid;
	u8 *opt_s_elem_start;
	int opt_s_elem_len;

	s8 tx_pwr_used;		/* for link measurement */
	s8 tx_pwr_max;		/* for link measurement */

	union {
		struct bcn_req_opt bcn;
		struct meas_req_opt clm;
		struct meas_req_opt nhm;
	}opt;

	struct rtw_ieee80211_channel ch_set[RTW_CHANNEL_SCAN_AMOUNT];
	u8 ch_set_ch_amount;
	s8 rx_pwr;		/* in dBm */
	u8 rx_bw;
	u8 rx_rsni;
	u16 rx_rate;
	u8 band;
};

struct rm_meas_rep {
	u8 category;
	u8 action_code;		/* T8-206  */
	u8 diag_token;

	u8 e_id;		/* T8-54, 38 request; 39 report */
	u8 len;
	u8 m_token;
	u8 m_mode;		/* req:F8-105, rep:F8-141 */
	u8 m_type;		/* T8-59 */
	u8 op_class;
	u8 ch_num;

	u8 ch_load;
	u8 anpi;
	u8 ipi[11];

	u16 rpt;
	u8 bssid[6];		/* for bcn_req */
	u8 band;
};

struct rrm_obj {

	u16 cid;

	enum RRM_STATE_ST state;
	struct rm_meas_req q;
	struct rm_meas_rep p;

	/* meas report */
	u64 meas_start_time;
	u64 meas_end_time;
	int wait_busy;
	u8 poll_mode;

	struct data_buf buf[MAX_RM_PKT_NUM];
	bool from_ioctl;
	struct fsm_obj *obj;
	struct fsm_main *fsm;

	struct wlan_network *ap[MAX_RM_AP_NUM];
	u8 ap_num;

	_list list;
};

static void rrm_upd_nb_ch_list(struct rrm_obj *prm, struct rrm_nb_rpt *pnb_rpt);
/*
 * rrm sub function
 */
void rrm_update_cap(u8 *frame_head, _adapter *pa, u32 pktlen, int offset)
{
	u8 *res;
	sint len;

	res = rtw_get_ie(frame_head + offset, _EID_RRM_EN_CAP_IE_, &len,
			 pktlen - offset);
	if (res != NULL)
		_rtw_memcpy((void *)pa->fsmpriv.rmpriv.rm_en_cap_def, (res + 2),
			    MIN(len, sizeof(pa->fsmpriv.rmpriv.rm_en_cap_def)));
}

struct cmd_meas_type_ {
	u8 id;
	char *name;
};

char *rm_type_req_name(u8 meas_type) {

	switch (meas_type) {
	case basic_req:
		return "basic_req";
	case cca_req:
		return "cca_req";
	case rpi_histo_req:
		return "rpi_histo_req";
	case ch_load_req:
		return "ch_load_req";
	case noise_histo_req:
		return "noise_histo_req";
	case bcn_req:
		return "bcn_req";
	case frame_req:
		return "frame_req";
	case sta_statis_req:
		return "sta_statis_req";
	}
	return "unknown_req";
};

char *rm_type_rep_name(u8 meas_type) {

	switch (meas_type) {
	case basic_rep:
		return "basic_rep";
	case cca_rep:
		return "cca_rep";
	case rpi_histo_rep:
		return "rpi_histo_rep";
	case ch_load_rep:
		return "ch_load_rep";
	case noise_histo_rep:
		return "noise_histo_rep";
	case bcn_rep:
		return "bcn_rep";
	case frame_rep:
		return "frame_rep";
	case sta_statis_rep:
		return "sta_statis_rep";
	}
	return "unknown_rep";
};

static char *rm_en_cap_name(enum rm_cap_en en)
{
	switch (en) {
	case RM_LINK_MEAS_CAP_EN:
		return "RM_LINK_MEAS_CAP_EN";
	case RM_NB_REP_CAP_EN:
		return "RM_NB_REP_CAP_EN";
	case RM_PARAL_MEAS_CAP_EN:
		return "RM_PARAL_MEAS_CAP_EN";
	case RM_REPEAT_MEAS_CAP_EN:
		return "RM_REPEAT_MEAS_CAP_EN";
	case RM_BCN_PASSIVE_MEAS_CAP_EN:
		return "RM_BCN_PASSIVE_MEAS_CAP_EN";
	case RM_BCN_ACTIVE_MEAS_CAP_EN:
		return "RM_BCN_ACTIVE_MEAS_CAP_EN";
	case RM_BCN_TABLE_MEAS_CAP_EN:
		return "RM_BCN_TABLE_MEAS_CAP_EN";
	case RM_BCN_MEAS_REP_COND_CAP_EN:
		return "RM_BCN_MEAS_REP_COND_CAP_EN";

	case RM_FRAME_MEAS_CAP_EN:
		return "RM_FRAME_MEAS_CAP_EN";
	case RM_CH_LOAD_CAP_EN:
		return "RM_CH_LOAD_CAP_EN";
	case RM_NOISE_HISTO_CAP_EN:
		return "RM_NOISE_HISTO_CAP_EN";
	case RM_STATIS_MEAS_CAP_EN:
		return "RM_STATIS_MEAS_CAP_EN";
	case RM_LCI_MEAS_CAP_EN:
		return "RM_LCI_MEAS_CAP_EN";
	case RM_LCI_AMIMUTH_CAP_EN:
		return "RM_LCI_AMIMUTH_CAP_EN";
	case RM_TRANS_STREAM_CAT_MEAS_CAP_EN:
		return "RM_TRANS_STREAM_CAT_MEAS_CAP_EN";
	case RM_TRIG_TRANS_STREAM_CAT_MEAS_CAP_EN:
		return "RM_TRIG_TRANS_STREAM_CAT_MEAS_CAP_EN";

	case RM_AP_CH_REP_CAP_EN:
		return "RM_AP_CH_REP_CAP_EN";
	case RM_RM_MIB_CAP_EN:
		return "RM_RM_MIB_CAP_EN";
	case RM_OP_CH_MAX_MEAS_DUR0:
		return "RM_OP_CH_MAX_MEAS_DUR0";
	case RM_OP_CH_MAX_MEAS_DUR1:
		return "RM_OP_CH_MAX_MEAS_DUR1";
	case RM_OP_CH_MAX_MEAS_DUR2:
		return "RM_OP_CH_MAX_MEAS_DUR2";
	case RM_NONOP_CH_MAX_MEAS_DUR0:
		return "RM_NONOP_CH_MAX_MEAS_DUR0";
	case RM_NONOP_CH_MAX_MEAS_DUR1:
		return "RM_NONOP_CH_MAX_MEAS_DUR1";
	case RM_NONOP_CH_MAX_MEAS_DUR2:
		return "RM_NONOP_CH_MAX_MEAS_DUR2";

	case RM_MEAS_PILOT_CAP0:
		return "RM_MEAS_PILOT_CAP0";		/* 24-26 */
	case RM_MEAS_PILOT_CAP1:
		return "RM_MEAS_PILOT_CAP1";
	case RM_MEAS_PILOT_CAP2:
		return "RM_MEAS_PILOT_CAP2";
	case RM_MEAS_PILOT_TRANS_INFO_CAP_EN:
		return "RM_MEAS_PILOT_TRANS_INFO_CAP_EN";
	case RM_NB_REP_TSF_OFFSET_CAP_EN:
		return "RM_NB_REP_TSF_OFFSET_CAP_EN";
	case RM_RCPI_MEAS_CAP_EN:
		return "RM_RCPI_MEAS_CAP_EN";		/* 29 */
	case RM_RSNI_MEAS_CAP_EN:
		return "RM_RSNI_MEAS_CAP_EN";
	case RM_BSS_AVG_ACCESS_DELAY_CAP_EN:
		return "RM_BSS_AVG_ACCESS_DELAY_CAP_EN";

	case RM_AVALB_ADMIS_CAPACITY_CAP_EN:
		return "RM_AVALB_ADMIS_CAPACITY_CAP_EN";
	case RM_ANT_CAP_EN:
		return "RM_ANT_CAP_EN";
	case RM_RSVD:
	case RM_MAX:
	default:
		break;
	}
	return "unknown";
}

static void rm_set_rep_mode(struct rrm_obj *prm, u8 mode)
{

	FSM_DBG(prm, "set %s\n",
		mode&MEAS_REP_MOD_INCAP?"INCAP":
		mode&MEAS_REP_MOD_REFUSE?"REFUSE":
		mode&MEAS_REP_MOD_LATE?"LATE":"");

	prm->p.m_mode |= mode;
}

int rm_en_cap_chk_and_set(struct rrm_obj *prm, enum rm_cap_en en)
{
	struct rrm_priv *rmpriv = (struct rrm_priv *)obj2priv(prm);
	int idx;
	u8 cap;


	if (en >= RM_MAX)
		return _FALSE;

	idx = en / 8;
	cap = rmpriv->rm_en_cap_def[idx];

	if (!(cap & BIT(en - (idx*8)))) {
		FSM_INFO(prm, "%s incapable\n",rm_en_cap_name(en));
		rm_set_rep_mode(prm, MEAS_REP_MOD_INCAP);
		return _FALSE;
	}
	return _SUCCESS;
}

static u8 rm_get_ch_set(u8 op_class, struct rtw_ieee80211_channel *pch_set, u8 pch_num)
{
	int i;
	const struct op_class_t *opc = get_opc_by_op_class(NULL, op_class);

	if (!opc) {
		RTW_INFO("%s can't get opc with id:%u\n", __func__, op_class);
		return 0;
	}

	if (pch_num < OPC_CH_LIST_LEN(opc)) {
		RTW_ERR("Invalid pch len %d < %d\n",pch_num,
			OPC_CH_LIST_LEN(opc));
		return 0;
	}

	for (i = 0; i < OPC_CH_LIST_LEN(opc); i++) {
		pch_set[i].hw_value = OPC_CH_LIST_CH(opc, i);
		pch_set[i].band = opc->band;
	}

	return i;
}

static enum band_type rm_get_band_by_opc(_adapter *padapter, u8 op_class)
{
	struct _ADAPTER_LINK *padapter_link = GET_PRIMARY_LINK(padapter);
	struct link_mlme_ext_priv *pmlmeext = &padapter_link->mlmeextpriv;
	char *alpha2 = NULL;

	if (is_alpha(pmlmeext->country[0]) != _FALSE &&
	    is_alpha(pmlmeext->country[1]) != _FALSE)
		alpha2 = pmlmeext->country;

	return  rtw_get_band_by_op_class(alpha2, op_class);
}

static u8 rm_get_ch_set_from_bcn_req_opt(struct rrm_obj *prm, struct bcn_req_opt *opt,
	struct rtw_ieee80211_channel *pch_set, u8 pch_num)
{
	_adapter *a = obj2adp(prm);
	int i,j,k,sz;
	struct _RT_OPERATING_CLASS *ap_ch_rpt;
	enum band_type band;
	u8 ch_amount = 0;

	k = 0;
	for (i = 0; i < opt->ap_ch_rpt_num; i++) {
		if (opt->ap_ch_rpt[i] == NULL)
			break;

		ap_ch_rpt = opt->ap_ch_rpt[i];
		band = rm_get_band_by_opc(a, ap_ch_rpt->global_op_class);

		if (band >= BAND_MAX) {
			FSM_WARN(prm, "%s: skip unknown opc:%d\n", __func__, ap_ch_rpt->global_op_class);
			continue;
		}

		if ((k + ap_ch_rpt->Len) > pch_num) {
			RTW_ERR("rrm: ch num exceed %d > %d\n", (k + ap_ch_rpt->Len), pch_num);
			return k;
		}

		for (j = 0; j < ap_ch_rpt->Len; j++) {
			pch_set[k].hw_value =
				ap_ch_rpt->Channel[j];
			pch_set[k].band = band;
			FSM_TRACE(prm, "meas_ch[%d].hw_value = %u\n",
				k, pch_set[k].hw_value);
			k++;
		}
	}
	return k;
}

static int is_wildcard_bssid(u8 *bssid)
{
	int i;
	u8 val8 = 0xff;


	for (i=0;i<6;i++)
		val8 &= bssid[i];

	if (val8 == 0xff)
		return _SUCCESS;
	return _FALSE;
}

static u8 translate_dbm_to_rcpi(s8 SignalPower)
{
	/* RCPI = Int{(Power in dBm + 110)*2} for 0dBm > Power > -110dBm
	 *    0	: power <= -110.0 dBm
	 *    1	: power =  -109.5 dBm
	 *    2	: power =  -109.0 dBm
	 */
	return (SignalPower + 110)*2;
}

static u8 translate_percentage_to_rcpi(u32 SignalStrengthIndex)
{
	/* Translate to dBm (x=y-100) */
	return translate_dbm_to_rcpi(SignalStrengthIndex - 100);
}

static u8 rm_get_bcn_rcpi(struct rrm_obj *prm, struct wlan_network *pnetwork)
{
	return translate_percentage_to_rcpi(
		pnetwork->network.PhyInfo.SignalStrength);
}

static u8 rm_get_frame_rsni(struct rrm_obj *prm, union recv_frame *pframe)
{
#if 0
	int i;
	u8 val8 = 0, snr;
	struct dvobj_priv *dvobj = adapter_to_dvobj(prm->psta->padapter);
	u8 rf_path = GET_HAL_RFPATH_NUM(dvobj);
	if (IS_CCK_RATE((hw_rate_to_m_rate(pframe->u.hdr.attrib.data_rate))))
		val8 = 255;
	else {
		snr = 0;
		for (i = 0; i < rf_path; i++)
			snr += pframe->u.hdr.attrib.phy_info.rx_snr[i];
		snr = snr / rf_path;
		val8 = (u8)(snr + 10)*2;
	}
#endif
	return 0;
}

static u8 rm_get_bcn_rsni(struct rrm_obj *prm, struct wlan_network *pnetwork)
{
	int i;
	u8 val8, snr;
	struct dvobj_priv *dvobj = adapter_to_dvobj(obj2adp(prm));
	u8 rf_path = GET_HAL_RFPATH_NUM(dvobj);


	if (pnetwork->network.PhyInfo.is_cck_rate) {
		/* current HW doesn't have CCK RSNI */
		/* 255 indicates RSNI is unavailable */
		val8 = 255;
	} else {
		snr = 0;
		for (i = 0; i < rf_path; i++) {
			snr += pnetwork->network.PhyInfo.rx_snr[i];
		}
		snr = snr / rf_path;
		val8 = (u8)(snr + 10)*2;
	}
	return val8;
}

/* output: pwr (unit dBm) */
static int rm_get_tx_power(_adapter *adapter, enum band_type band, enum MGN_RATE rate, s8 *pwr)
{
	struct dvobj_priv *devob = adapter_to_dvobj(adapter);
	u8 rs = mgn_rate_to_rs(rate);

	*pwr = rtw_phl_get_power_by_rate_band(GET_PHL_INFO(devob), HW_BAND_0, _rate_mrate2phl(rate),
					       IS_DCM_RATE_SECTION(rs) ? 1 : 0, 0, band);
	return 0;
}

static u8 rrm_gen_dialog_token(_adapter *padapter)
{
	struct mlme_ext_priv *pmlmeext = &(padapter->mlmeextpriv);
	struct mlme_ext_info *pmlmeinfo = &(pmlmeext->mlmext_info);

	do {
		pmlmeinfo->dialogToken++;
	} while (pmlmeinfo->dialogToken == 0); /* Valid range : 1-255*/
	return pmlmeinfo->dialogToken;
}

static u8 rrm_gen_meas_token(_adapter *padapter)
{
	struct rrm_priv *prmpriv = &(padapter->fsmpriv.rmpriv);

	do {
		prmpriv->meas_token++;
	} while (prmpriv->meas_token == 0);

	return prmpriv->meas_token;
}

static u16 rrm_cal_cid(u8 token, u16 role)
{
	u16 cid;

	cid = role | token;

	return cid;
}

static int rrm_get_rx_sensitivity(_adapter *adapter, enum channel_width bw, enum MGN_RATE rate, s8 *pwr)
{
	s8 rx_sensitivity = -110;

	switch(rate) {
	case MGN_1M:
		rx_sensitivity= -101;
		break;
	case MGN_2M:
		rx_sensitivity= -98;
		break;
	case MGN_5_5M:
		rx_sensitivity= -92;
		break;
	case MGN_11M:
		rx_sensitivity= -89;
		break;
	case MGN_6M:
	case MGN_9M:
	case MGN_12M:
		rx_sensitivity = -92;
		break;
	case MGN_18M:
		rx_sensitivity = -90;
		break;
	case MGN_24M:
		rx_sensitivity = -88;
		break;
	case MGN_36M:
		rx_sensitivity = -84;
		break;
	case MGN_48M:
		rx_sensitivity = -79;
		break;
	case MGN_54M:
		rx_sensitivity = -78;
		break;

	case MGN_MCS0:
	case MGN_MCS8:
	case MGN_MCS16:
	case MGN_MCS24:
	case MGN_VHT1SS_MCS0:
	case MGN_VHT2SS_MCS0:
	case MGN_VHT3SS_MCS0:
	case MGN_VHT4SS_MCS0:
		/* BW20 BPSK 1/2 */
		rx_sensitivity = -82;
		break;

	case MGN_MCS1:
	case MGN_MCS9:
	case MGN_MCS17:
	case MGN_MCS25:
	case MGN_VHT1SS_MCS1:
	case MGN_VHT2SS_MCS1:
	case MGN_VHT3SS_MCS1:
	case MGN_VHT4SS_MCS1:
		/* BW20 QPSK 1/2 */
		rx_sensitivity = -79;
		break;

	case MGN_MCS2:
	case MGN_MCS10:
	case MGN_MCS18:
	case MGN_MCS26:
	case MGN_VHT1SS_MCS2:
	case MGN_VHT2SS_MCS2:
	case MGN_VHT3SS_MCS2:
	case MGN_VHT4SS_MCS2:
		/* BW20 QPSK 3/4 */
		rx_sensitivity = -77;
		break;

	case MGN_MCS3:
	case MGN_MCS11:
	case MGN_MCS19:
	case MGN_MCS27:
	case MGN_VHT1SS_MCS3:
	case MGN_VHT2SS_MCS3:
	case MGN_VHT3SS_MCS3:
	case MGN_VHT4SS_MCS3:
		/* BW20 16-QAM 1/2 */
		rx_sensitivity = -74;
		break;

	case MGN_MCS4:
	case MGN_MCS12:
	case MGN_MCS20:
	case MGN_MCS28:
	case MGN_VHT1SS_MCS4:
	case MGN_VHT2SS_MCS4:
	case MGN_VHT3SS_MCS4:
	case MGN_VHT4SS_MCS4:
		/* BW20 16-QAM 3/4 */
		rx_sensitivity = -70;
		break;

	case MGN_MCS5:
	case MGN_MCS13:
	case MGN_MCS21:
	case MGN_MCS29:
	case MGN_VHT1SS_MCS5:
	case MGN_VHT2SS_MCS5:
	case MGN_VHT3SS_MCS5:
	case MGN_VHT4SS_MCS5:
		/* BW20 64-QAM 2/3 */
		rx_sensitivity = -66;
		break;

	case MGN_MCS6:
	case MGN_MCS14:
	case MGN_MCS22:
	case MGN_MCS30:
	case MGN_VHT1SS_MCS6:
	case MGN_VHT2SS_MCS6:
	case MGN_VHT3SS_MCS6:
	case MGN_VHT4SS_MCS6:
		/* BW20 64-QAM 3/4 */
		rx_sensitivity = -65;
		break;

	case MGN_MCS7:
	case MGN_MCS15:
	case MGN_MCS23:
	case MGN_MCS31:
	case MGN_VHT1SS_MCS7:
	case MGN_VHT2SS_MCS7:
	case MGN_VHT3SS_MCS7:
	case MGN_VHT4SS_MCS7:
		/* BW20 64-QAM 5/6 */
		rx_sensitivity = -64;
		break;

	case MGN_VHT1SS_MCS8:
	case MGN_VHT2SS_MCS8:
	case MGN_VHT3SS_MCS8:
	case MGN_VHT4SS_MCS8:
		/* BW20 256-QAM 3/4 */
		rx_sensitivity = -59;
		break;

	case MGN_VHT1SS_MCS9:
	case MGN_VHT2SS_MCS9:
	case MGN_VHT3SS_MCS9:
	case MGN_VHT4SS_MCS9:
		/* BW20 256-QAM 5/6 */
		rx_sensitivity = -57;
		break;

	default:
		return -1;
		break;

	}

	switch(bw) {
	case CHANNEL_WIDTH_20:
		break;
	case CHANNEL_WIDTH_40:
		rx_sensitivity -= 3;
		break;
	case CHANNEL_WIDTH_80:
		rx_sensitivity -= 6;
		break;
	case CHANNEL_WIDTH_160:
		rx_sensitivity -= 9;
		break;
	case CHANNEL_WIDTH_5:
	case CHANNEL_WIDTH_10:
	case CHANNEL_WIDTH_80_80:
	default:
		return -1;
		break;
	}
	*pwr = rx_sensitivity;

	return 0;
}

/* output: path_a max tx power in dBm */
static int rrm_get_path_a_max_tx_power(_adapter *adapter, s8 *path_a)
{
#if 0 /*GEORGIA_TODO_FIXIT*/

	struct dvobj_priv *dvobj = adapter_to_dvobj(adapter);
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(dvobj);
	HAL_DATA_TYPE *hal_data = GET_PHL_COM(dvobj);
	int path, tx_num, band, bw, ch, n, rs;
	u8 rate_num;
	s8 max_pwr[RF_PATH_MAX], pwr;


	band = hal_data->current_band_type;
	bw = hal_data->current_channel_bw;
	ch = hal_data->current_channel;

	for (path = 0; path < RF_PATH_MAX; path++) {
		if (!HAL_SPEC_CHK_RF_PATH(hal_spec, band, path))
			break;

		max_pwr[path] = -127; /* min value of s8 */
#if (RM_MORE_DBG_MSG)
		RTW_INFO("[%s][%c]\n", band_str(band), rf_path_char(path));
#endif
		for (rs = 0; rs < RATE_SECTION_NUM; rs++) {
			tx_num = rate_section_to_tx_num(rs);

			if (tx_num >= hal_spec->tx_nss_num)
				continue;

			if (band == BAND_ON_5G && IS_CCK_RATE_SECTION(rs))
				continue;

			if (IS_VHT_RATE_SECTION(rs) && !IS_HARDWARE_TYPE_JAGUAR_ALL(adapter))
				continue;

			rate_num = rate_section_rate_num(rs);

			/* get power by rate in db */
			for (n = rate_num - 1; n >= 0; n--) {
				pwr = phy_get_tx_power_final_absolute_value(adapter, path, rates_by_sections[rs].rates[n], bw, ch);
				max_pwr[path] = MAX(max_pwr[path], pwr);
#if (RM_MORE_DBG_MSG)
				RTW_INFO("%9s = %2d\n",
					MGN_RATE_STR(rates_by_sections[rs].rates[n]), pwr);
#endif
			}
		}
	}
#if (RM_MORE_DBG_MSG)
	RTW_INFO("path_a max_pwr=%ddBm\n", max_pwr[0]);
#endif
	*path_a = max_pwr[0];
#endif
	return 0;
}

static u8 *build_wlan_hdr(struct rrm_obj *prm, _adapter *padapter,
	struct xmit_frame *pmgntframe, struct sta_info *psta, u16 frame_type)
{
	u8 *pframe;
	u16 *fctrl;
	struct pkt_attrib *pattr;
	struct rtw_ieee80211_hdr *pwlanhdr;
	struct mlme_ext_priv *pmlmeext = &padapter->mlmeextpriv;
	struct _ADAPTER_LINK *padapter_link = psta->padapter_link;
	struct link_mlme_ext_info *pmlmeinfo = &padapter_link->mlmeextpriv.mlmext_info;


	/* update attribute */
	pattr = &pmgntframe->attrib;
	update_mgntframe_attrib(padapter, padapter_link, pattr);

	_rtw_memset(pmgntframe->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);

	pframe = (u8 *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;

	_rtw_memcpy(pwlanhdr->addr1, psta->phl_sta->mac_addr, ETH_ALEN);
	_rtw_memcpy(pwlanhdr->addr2, padapter_link->mac_addr, ETH_ALEN);
	_rtw_memcpy(pwlanhdr->addr3, get_my_bssid(&(pmlmeinfo->network)), ETH_ALEN);

	FSM_TRACE(prm, "dst = " MAC_FMT "\n", MAC_ARG(pwlanhdr->addr1));

	SetSeqNum(pwlanhdr, pmlmeext->mgnt_seq);
	pmlmeext->mgnt_seq++;
	SetFragNum(pframe, 0);

	set_frame_sub_type(pframe, WIFI_ACTION);

	pframe += sizeof(struct rtw_ieee80211_hdr_3addr);
	pattr->pktlen = sizeof(struct rtw_ieee80211_hdr_3addr);

	return pframe;
}

int rrm_issue_null_reply(struct rrm_obj *prm)
{
	int len=0, my_len;
	u8 *pframe, m_mode;
	_adapter *padapter = obj2adp(prm);
	struct pkt_attrib *pattr;
	struct xmit_frame *pmgntframe;
	struct xmit_priv *pxmitpriv = &(padapter->xmitpriv);

	m_mode = prm->p.m_mode;
	if (m_mode || prm->q.rpt == 0) {
		FSM_INFO(prm, "reply %s\n",
			m_mode&MEAS_REP_MOD_INCAP?"INCAP":
			m_mode&MEAS_REP_MOD_REFUSE?"REFUSE":
			m_mode&MEAS_REP_MOD_LATE?"LATE":"no content");
	}

	switch (prm->q.action_code) {
	case RM_ACT_RADIO_MEAS_REQ:
		len = 8;
		break;
	case RM_ACT_NB_REP_REQ:
		len = 3;
		break;
	case RM_ACT_LINK_MEAS_REQ:
		len = 3;
		break;
	default:
		break;
	}

	if (len==0)
		return _FALSE;

	pmgntframe = alloc_mgtxmitframe(pxmitpriv);
	if (pmgntframe == NULL) {
		FSM_ERR(prm, "%s alloc xmit_frame fail\n",__func__);
		return _FALSE;
	}
	pattr = &pmgntframe->attrib;
	pframe = build_wlan_hdr(prm, padapter, pmgntframe, obj2sta(prm), WIFI_ACTION);
	pframe = rtw_set_fixed_ie(pframe, 3, &prm->p.category, &pattr->pktlen);

	my_len = 0;
	if (len>5) {
		prm->p.len = len - 3 - 2;
		pframe = rtw_set_fixed_ie(pframe, len - 3,
			&prm->p.e_id, &my_len);
	}

	pattr->pktlen += my_len;
	pattr->last_txcmdsz = pattr->pktlen;
	dump_mgntframe(padapter, pmgntframe);

	return _SUCCESS;
}

static int rm_prepare_scan(struct rrm_obj *prm)
{
	_adapter *padapter = obj2adp(prm);
	u8 ssc_chk;

	ssc_chk = rtw_sitesurvey_condition_check(padapter, _FALSE);

	if (ssc_chk == SS_ALLOW) {
		return _SUCCESS;
	} else if (ssc_chk == SS_DENY_BUSY_TRAFFIC) {
		FSM_DBG(prm, "Don't care BusyTraffic\n");
		return _SUCCESS;
	}
	return _FALSE;
}

int rrm_get_chset(struct rrm_obj *prm)
{
	int i,meas_ch_amount=0;
	u8 op_class=0, val8;
	struct rtw_ieee80211_channel *pch_set;

	pch_set = &prm->q.ch_set[0];

	_rtw_memset(pch_set, 0,
		sizeof(struct rtw_ieee80211_channel) * RTW_CHANNEL_SCAN_AMOUNT);

	op_class = prm->q.op_class;
	if (prm->q.ch_num == 0) {
		/* ch_num=0   : scan all ch in operating class */
		meas_ch_amount = rm_get_ch_set(op_class, pch_set, RTW_CHANNEL_SCAN_AMOUNT);

	} else if (prm->q.ch_num == 255) {
		/* 802.11 p.1066 */
		/* ch_num=255 : If the Channel Number is 255 and includes
		 * AP Channel Report subelements
		 */
		meas_ch_amount = rm_get_ch_set_from_bcn_req_opt(prm,
			&prm->q.opt.bcn, pch_set, RTW_CHANNEL_SCAN_AMOUNT);
	} else {
		pch_set[0].hw_value = prm->q.ch_num;
		pch_set[0].band = prm->q.band;
		meas_ch_amount = 1;
		FSM_INFO(prm, "meas_ch->hw_value = %u\n", pch_set->hw_value);
	}

	/* get means channel */
	prm->q.ch_set_ch_amount = meas_ch_amount;

	return 0;
}

static int rrm_sitesurvey(struct rrm_obj *prm)
{
	int meas_ch_amount=0;
	struct rtw_ieee80211_channel *pch_set;
	struct sitesurvey_parm *parm;

	parm = rtw_malloc(sizeof(*parm));
	if (parm == NULL)
		return _FAIL;

	rrm_get_chset(prm);
	pch_set = &prm->q.ch_set[0];

	meas_ch_amount = MIN(prm->q.ch_set_ch_amount, RTW_CHANNEL_SCAN_AMOUNT);
	_rtw_memset(parm, 0, sizeof(struct sitesurvey_parm));
	_rtw_memcpy(parm->ch, pch_set, sizeof(struct rtw_ieee80211_channel) * meas_ch_amount);

	_rtw_memcpy(&parm->ssid[0], &prm->q.opt.bcn.ssid, IW_ESSID_MAX_SIZE);

	parm->ssid_num = 1;
	parm->scan_mode = prm->q.s_mode;
	parm->ch_num = meas_ch_amount;
	parm->duration = prm->q.meas_dur;
	parm->scan_type = RTW_SCAN_RRM;
	parm->psta = obj2sta(prm);
	/* parm.bw = BW_20M; */

	rtw_sitesurvey_cmd(obj2adp(prm), parm);
	rtw_mfree(parm, sizeof(*parm));

	if (prm->q.opt.bcn.ssid.SsidLength)
		FSM_INFO(prm, "%s search %s\n", __func__, prm->q.opt.bcn.ssid.Ssid);
	else
		FSM_INFO(prm, "%s\n", __func__);

	return _SUCCESS;
}

static int rrm_parse_ch_load_s_elem(struct rrm_obj *prm, u8 *pbody, int req_len)
{
	u8 *popt_id;
	int i, p=0; /* position */
	int len = req_len;


	prm->q.opt_s_elem_len = len;

	while (len) {

		switch (pbody[p]) {
		case ch_load_rep_info:
			/* check RM_EN */
			rm_en_cap_chk_and_set(prm, RM_CH_LOAD_CAP_EN);

			_rtw_memcpy(&(prm->q.opt.clm.rep_cond),
				&pbody[p+2], sizeof(prm->q.opt.clm.rep_cond));

			FSM_INFO(prm, "ch_load_rep_info=%u:%u\n",
				prm->q.opt.clm.rep_cond.cond,
				prm->q.opt.clm.rep_cond.threshold);
			break;
		default:
			break;

		}
		len = len - (int)pbody[p+1] - 2;
		p = p + (int)pbody[p+1] + 2;
	}
	return _SUCCESS;
}

static int rrm_parse_noise_histo_s_elem(struct rrm_obj *prm,
	u8 *pbody, int req_len)
{
	u8 *popt_id;
	int i, p=0; /* position */
	int len = req_len;


	prm->q.opt_s_elem_len = len;

	while (len) {

		switch (pbody[p]) {
		case noise_histo_rep_info:
			/* check RM_EN */
			rm_en_cap_chk_and_set(prm, RM_NOISE_HISTO_CAP_EN);

			_rtw_memcpy(&(prm->q.opt.nhm.rep_cond),
				&pbody[p+2], sizeof(prm->q.opt.nhm.rep_cond));

			FSM_INFO(prm, "noise_histo_rep_info=%u:%u\n",
				prm->q.opt.nhm.rep_cond.cond,
				prm->q.opt.nhm.rep_cond.threshold);
			break;
		default:
			break;

       		}
		len = len - (int)pbody[p+1] - 2;
		p = p + (int)pbody[p+1] + 2;
	}
	return _SUCCESS;
}

static int rrm_parse_bcn_req_s_elemm(struct rrm_obj *prm, u8 *pbody, int req_len)
{
	u8 *popt_id;
	int i, p=0; /* position */
	int len = req_len;
	int ap_ch_rpt_idx = 0;
	struct _RT_OPERATING_CLASS *op;


	/* opt length,2:pbody[0]+ pbody[1] */
	/* first opt id : pbody[18] */

	prm->q.opt_s_elem_len = len;

	popt_id = prm->q.opt.bcn.opt_id;
	while (len && prm->q.opt.bcn.opt_id_num < BCN_REQ_OPT_MAX_NUM) {

		switch (pbody[p]) {
		case bcn_req_ssid:
#if (DBG_BCN_REQ_WILDCARD)
			FSM_INFO(prm, "DBG set ssid to WILDCARD\n");
#else
#if (DBG_BCN_REQ_SSID)
			FSM_INFO(prm, "DBG set ssid to %s\n",DBG_BCN_REQ_SSID_NAME);
			i = strlen(DBG_BCN_REQ_SSID_NAME);
			prm->q.opt.bcn.ssid.SsidLength = i;
			_rtw_memcpy(&(prm->q.opt.bcn.ssid.Ssid), DBG_BCN_REQ_SSID_NAME,
				MIN(i, sizeof(prm->q.opt.bcn.ssid.Ssid)-1));

#else /* original */
			prm->q.opt.bcn.ssid.SsidLength = pbody[p+1];
			_rtw_memcpy(&(prm->q.opt.bcn.ssid.Ssid), &pbody[p+2],
				MIN(pbody[p+1], sizeof(prm->q.opt.bcn.ssid.Ssid)-1));
#endif
#endif
			FSM_DBG(prm, "bcn_req_ssid %s\n", prm->q.opt.bcn.ssid.Ssid);

			popt_id[prm->q.opt.bcn.opt_id_num++] = pbody[p];
			break;

		case bcn_req_rep_info:
			FSM_TRACE(prm, "bcn_req_rep_info\n");
			/* check RM_EN */
			rm_en_cap_chk_and_set(prm, RM_BCN_MEAS_REP_COND_CAP_EN);

			_rtw_memcpy(&(prm->q.opt.bcn.rep_cond),
				&pbody[p+2], sizeof(prm->q.opt.bcn.rep_cond));

			FSM_TRACE(prm, "bcn_req_rep_info=%u:%u\n",
				prm->q.opt.bcn.rep_cond.cond,
				prm->q.opt.bcn.rep_cond.threshold);

			/*popt_id[prm->q.opt.bcn.opt_id_num++] = pbody[p];*/
			break;

		case bcn_req_rep_detail:
			FSM_TRACE(prm, "bcn_req_rep_detail\n");
#if DBG_BCN_REQ_DETAIL
			prm->q.opt.bcn.rep_detail = 2; /* all IE in beacon */
#else
			prm->q.opt.bcn.rep_detail = pbody[p+2];
#endif
			popt_id[prm->q.opt.bcn.opt_id_num++] = pbody[p];

			FSM_TRACE(prm, "report_detail=%d\n",
				prm->q.opt.bcn.rep_detail);
			break;
		case bcn_req_last_bcn_rpt_ind:
			prm->q.opt.bcn.rep_last_bcn_ind = pbody[p+2];
#if DBG_BCN_REQ_DETAIL
			RTW_INFO("RM: bcn_req_last_bcn_rpt_ind=%d\n",
				prm->q.opt.bcn.rep_last_bcn_ind);
#endif
			break;

		case bcn_req_req:

			FSM_TRACE(prm, "bcn_req_req\n");

			prm->q.opt.bcn.req_start = rtw_malloc(pbody[p+1]);

			if (prm->q.opt.bcn.req_start == NULL) {
				FSM_ERR(prm, "req_start malloc fail!!\n");
				break;
			}

			for (i = 0; i < pbody[p+1]; i++)
				*((prm->q.opt.bcn.req_start)+i) =
					pbody[p+2+i];

			prm->q.opt.bcn.req_len = pbody[p+1];
			popt_id[prm->q.opt.bcn.opt_id_num++] = pbody[p];
			break;

		case bcn_req_ap_ch_rep:

			FSM_TRACE(prm, "rrm: bcn_req_ap_ch_rep\n");

			if (ap_ch_rpt_idx > BCN_REQ_OPT_AP_CH_RPT_MAX_NUM) {
				FSM_ERR(prm, "bcn_req_ap_ch_rep over size\n");
				break;
			}
			popt_id[prm->q.opt.bcn.opt_id_num++] = pbody[p];

			/* get channel list
			 * EID:len:op-class:ch-list
			 */
			op = rtw_malloc(sizeof (*op));
			op->global_op_class = pbody[p + 2];
			i = pbody[p + 1] - 1; /* ch list len; (-1) is op class */

			FSM_DBG(prm, "%d op class %d has %d ch\n",
				ap_ch_rpt_idx,op->global_op_class,i);

			op->Len = i;
			memcpy(op->Channel, &pbody[p + 3],
				MIN(i, MAX_CH_NUM_IN_OP_CLASS));
			prm->q.opt.bcn.ap_ch_rpt[ap_ch_rpt_idx++] = op;
			prm->q.opt.bcn.ap_ch_rpt_num = ap_ch_rpt_idx;
			break;

		default:
			break;

		}
		len = len - (int)pbody[p+1] - 2;
		p = p + (int)pbody[p+1] + 2;
	}

	return _SUCCESS;
}

static int rrm_parse_meas_req(struct rrm_obj *prm, u8 *pbody)
{
	_adapter *a = obj2adp(prm);
	int p; /* position */
	int req_len;

	req_len = (int)pbody[1];
	p = 5;

	prm->q.op_class = pbody[p++];
	prm->q.ch_num = pbody[p++];
	prm->q.band = (prm->q.op_class)?rm_get_band_by_opc(a, prm->q.op_class):BAND_MAX;

	if (prm->q.band >= BAND_MAX) {
		FSM_WARN(prm, "%s: unknown opc:%d\n", __func__, prm->q.op_class);
		return _FAIL;
	}

	prm->q.rand_intvl = le16_to_cpu(*(u16*)(&pbody[p]));
	p+=2;
	prm->q.meas_dur = le16_to_cpu(*(u16*)(&pbody[p]));
	p+=2;

	if (prm->q.m_type == bcn_req) {
		/*
		 * 0: passive
		 * 1: active
		 * 2: bcn_table
		 */
		prm->q.s_mode = pbody[p++];

		/* BSSID */
		_rtw_memcpy(&(prm->q.bssid), &pbody[p], 6);
		p+=6;

		/*
		 * default, used when Reporting detail subelement
		 * is not included in Beacon Request
		 */
		prm->q.opt.bcn.rep_detail = 2;
	}

	if (req_len-(p-2) <= 0) /* without sub-element */
		return _SUCCESS;

	switch (prm->q.m_type) {
	case bcn_req:
		rrm_parse_bcn_req_s_elemm(prm, &pbody[p], req_len-(p-2));
		break;
	case ch_load_req:
		rrm_parse_ch_load_s_elem(prm, &pbody[p], req_len-(p-2));
		break;
	case noise_histo_req:
		rrm_parse_noise_histo_s_elem(prm, &pbody[p], req_len-(p-2));
		break;
	default:
		break;
	}

	return _SUCCESS;
}

static int rrm_enqueue_rm(struct rrm_priv *prmpriv , struct rrm_obj *prm, u8 to_head)
{
	_queue *queue = &prmpriv->rm_queue;

	_rtw_spinlock_bh(&queue->lock);
	if (to_head)
		list_add(&prm->list, &queue->queue);
	else
		list_add_tail(&prm->list, &queue->queue);
	_rtw_spinunlock_bh(&queue->lock);

	return 0;
}

static int rrm_dequeue_rm(struct rrm_priv *prmpriv , struct rrm_obj *prm)
{
	_queue *queue = &prmpriv->rm_queue;

	_rtw_spinlock_bh(&queue->lock);
	list_del(&prm->list);
	_rtw_spinunlock_bh(&queue->lock);

	return 0;
}

static struct rrm_obj *rtw_rrm_new_obj(_adapter *a, struct sta_info *psta, u8 diag_token)
{
	struct fsm_main *fsm = a->fsmpriv.rmpriv.fsm;
	struct rrm_priv *prmpriv = &(a->fsmpriv.rmpriv);
	struct fsm_obj *obj;
	struct rrm_obj *prm;
	u16 cid;

	if (diag_token) {
		cid = rrm_cal_cid(diag_token, RM_SLAVE);
	} else {
		diag_token = rrm_gen_dialog_token(a);
		cid = rrm_cal_cid(diag_token, RM_MASTER);
	}

	/* check redundant obj */
	prm = rtw_fsm_get_obj(fsm, cid);
	if (prm) {
		FSM_WARN(prm, "%s obj exist!!\n", __func__);
		return NULL;
	}

	obj = rtw_fsm_new_obj(fsm, psta, cid, (void **)&prm, sizeof(*prm));

	if (prm == NULL) {
		FSM_ERR_(fsm, "rrm: malloc obj fail\n");
		return NULL;
	}

	prm->fsm = fsm;
	prm->obj = obj;
	prm->cid = cid;
	prm->p.diag_token = diag_token;
	prm->q.diag_token = diag_token;

	return prm;
}

/* receive measurement request */
int rrm_recv_radio_mens_req(_adapter *padapter,
	union recv_frame *precv_frame, struct sta_info *psta)
{
	struct rrm_priv *prmpriv = &padapter->fsmpriv.rmpriv;
	struct rrm_obj *prm;
	u8 *pdiag_body = (u8 *)(precv_frame->u.hdr.rx_data +
		sizeof(struct rtw_ieee80211_hdr_3addr));
	u8 ssc_chk, diag_token = pdiag_body[2];
	u8 *pmeas_body = &pdiag_body[5];
	u8 need_to_scan = 1;
	u16 cid;

#if 0
	if (pmeas_body[4] == bcn_req) {
		rtw_cfg80211_rx_rrm_action(padapter, precv_frame);
		return _SUCCESS;
	}
#endif

	/* search existing rrm_obj */
	cid = rrm_cal_cid(pdiag_body[2], RM_SLAVE);
	prm = rtw_fsm_get_obj(prmpriv->fsm, cid);
	if (prm) {
		FSM_INFO(prm, "Found an exist meas\n");
		return _FALSE;
	}

	prm = rtw_rrm_new_obj(padapter, psta, diag_token);
	if (prm == NULL) {
		RTW_ERR("rrm: unable to alloc rm obj for requeset\n");
		return _FALSE;
	}

	prm->q.diag_token = diag_token;
	prm->q.rpt = le16_to_cpu(*(u16*)(&pdiag_body[3]));

	/* Figure 8-104 Measurement Requested format */
	prm->q.e_id = pmeas_body[0];
	prm->q.m_token = pmeas_body[2];
	prm->q.m_mode = pmeas_body[3];
	prm->q.m_type = pmeas_body[4];

	//FSM_INFO(prm, "bssid " MAC_FMT "\n", MAC_ARG(obj2mac(prm)));

	FSM_TRACE(prm, "element_id = %d\n", prm->q.e_id);
	FSM_TRACE(prm, "length = %d\n", (int)pmeas_body[1]);
	FSM_TRACE(prm, "meas_token = %d\n", prm->q.m_token);
	FSM_TRACE(prm, "meas_mode = %d\n", prm->q.m_mode);
	FSM_TRACE(prm, "meas_type = %d\n", prm->q.m_type);

	if (prm->q.e_id != _MEAS_REQ_IE_) { /* 38 */
		rm_set_rep_mode(prm, MEAS_REP_MOD_INCAP);
		RTW_WARN("rrm: Unsupported Action element(%d!=%d)!\n",
			prm->q.e_id, _MEAS_REQ_IE_);
		goto fail;
	}

	FSM_INFO(prm, "rx %s from " MAC_FMT "\n",
		rm_type_req_name(prm->q.m_type), MAC_ARG(obj2mac(prm)));

	switch (prm->q.m_type) {
	case bcn_req:
		if (!rrm_parse_meas_req(prm, pmeas_body)) {
			rm_set_rep_mode(prm, MEAS_REP_MOD_INCAP);
			goto fail;
		}
		switch (prm->q.s_mode) {
		case bcn_req_passive:
			rm_en_cap_chk_and_set(prm, RM_BCN_PASSIVE_MEAS_CAP_EN);
			break;
		case bcn_req_active:
			rm_en_cap_chk_and_set(prm, RM_BCN_ACTIVE_MEAS_CAP_EN);
			break;
		case bcn_req_bcn_table:
			need_to_scan = 0;
			rm_en_cap_chk_and_set(prm, RM_BCN_TABLE_MEAS_CAP_EN);
			break;
		default:
			rm_set_rep_mode(prm, MEAS_REP_MOD_INCAP);
			goto fail;
		}
		break;
	case ch_load_req:
		rrm_parse_meas_req(prm, pmeas_body);
		rm_en_cap_chk_and_set(prm, RM_CH_LOAD_CAP_EN);
		break;
	case noise_histo_req:
		rrm_parse_meas_req(prm, pmeas_body);
		rm_en_cap_chk_and_set(prm, RM_NOISE_HISTO_CAP_EN);
		break;
	default:
		rm_set_rep_mode(prm, MEAS_REP_MOD_INCAP);
		goto fail;
	}

	if (need_to_scan) {
		ssc_chk = rtw_sitesurvey_condition_check(padapter, _FALSE);

		if ((ssc_chk != SS_ALLOW) && (ssc_chk != SS_DENY_BUSY_TRAFFIC)) {
			rm_set_rep_mode(prm, MEAS_REP_MOD_REFUSE);
			goto fail;
		}
	}

	rtw_fsm_activate_obj(prm);

	return _SUCCESS;
fail:
	/* reply incap */
	rrm_issue_null_reply(prm);

	/* del obj */
	rtw_fsm_del_obj(prm);

	return _FAIL;
}

void indicate_beacon_report(struct rrm_obj *prm, u8 *sta_addr,
	u8 n_measure_rpt, u32 elem_len, u8 *elem)
{
	FSM_INFO(prm, "recv bcn reprot from mac="MAC_FMT"\n", MAC_ARG(sta_addr));
}

/* receive measurement report */
int rrm_recv_radio_mens_rep(_adapter *padapter,
	union recv_frame *precv_frame, struct sta_info *psta)
{
	int ret = _FALSE;
	struct rrm_obj *prm;
	struct rrm_priv *prmpriv = &padapter->fsmpriv.rmpriv;
	u32 len;
	u16 cid;
	u8 *pdiag_body = (u8 *)(precv_frame->u.hdr.rx_data +
		sizeof(struct rtw_ieee80211_hdr_3addr));
	u8 *pmeas_body = &pdiag_body[3];

	cid = rrm_cal_cid(pdiag_body[2], RM_MASTER);
	prm = rtw_fsm_get_obj(prmpriv->fsm, cid);
	if (prm == NULL) {
		/* not belong to us, report to upper */
		rtw_cfg80211_rx_rrm_action(psta->padapter, precv_frame);
		return _TRUE;
	}

	prm->p.action_code = pdiag_body[1];
	prm->p.diag_token = pdiag_body[2];

	/* Figure 8-140 Measuremnt Report format */
	prm->p.e_id = pmeas_body[0];
	prm->p.m_token = pmeas_body[2];
	prm->p.m_mode = pmeas_body[3];
	prm->p.m_type = pmeas_body[4];

	FSM_INFO(prm, "bssid " MAC_FMT "\n", MAC_ARG(obj2mac(prm)));

	FSM_TRACE(prm, "element_id = %d\n", prm->p.e_id);
	FSM_TRACE(prm, "length = %d\n", (int)pmeas_body[1]);
	FSM_TRACE(prm, "meas_token = %d\n", prm->p.m_token);
	FSM_TRACE(prm, "meas_mode = %d\n", prm->p.m_mode);
	FSM_TRACE(prm, "meas_type = %d\n", prm->p.m_type);

	if (prm->p.e_id != _MEAS_RSP_IE_) /* 39 */
		return _FALSE;

	FSM_INFO(prm, "rx %s (" MAC_FMT ")\n",
		rm_type_rep_name(prm->p.m_type), MAC_ARG(obj2mac(prm)));

	rtw_fsm_gen_msg(prm, NULL, 0, RRM_EV_recv_rep);

	/* report to upper via ioctl */
	if ((prm->from_ioctl == true) &&
		prm->q.m_type == bcn_req) {
		len = pmeas_body[1] + 2; /* 2 : EID(1B)  length(1B) */
		indicate_beacon_report(prm, obj2mac(prm),
			1, len, pmeas_body);
	}

	return ret;
}

/* receive link measurement request */
int rrm_recv_link_mens_req(_adapter *padapter,
	union recv_frame *precv_frame, struct sta_info *psta)
{
	struct rrm_obj *prm;
	u8 *pdiag_body = (u8 *)(precv_frame->u.hdr.rx_data +
		sizeof(struct rtw_ieee80211_hdr_3addr));
	u8 diag_token = pdiag_body[2];
	u8 *pmeas_body = &pdiag_body[3];
	int i;

	prm = rtw_rrm_new_obj(padapter, psta, diag_token);

	if (prm == NULL) {
		RTW_ERR("rrm: unable to alloc rm obj for requeset\n");
		return _FALSE;
	}

	prm->q.action_code = pdiag_body[1];
	prm->q.diag_token = pdiag_body[2];
	prm->q.tx_pwr_used = pmeas_body[0];
	prm->q.tx_pwr_max = pmeas_body[1];
	prm->q.rx_pwr = precv_frame->u.hdr.attrib.phy_info.rx_power;
	prm->q.rx_rate = precv_frame->u.hdr.attrib.data_rate;
	prm->q.rx_bw = precv_frame->u.hdr.attrib.bw;
	prm->q.rx_rsni = rm_get_frame_rsni(prm, precv_frame);

	FSM_INFO(prm, "rx link_mens_req(" MAC_FMT ") rx_pwr=%ddBm, rate=%s\n",
		MAC_ARG(obj2mac(prm)), prm->q.rx_pwr,
		MGN_RATE_STR(prm->q.rx_rate));

	FSM_TRACE(prm, "tx_pwr_used =%d dBm\n", prm->q.tx_pwr_used);
	FSM_TRACE(prm, "tx_pwr_max  =%d dBm\n", prm->q.tx_pwr_max);

	rtw_fsm_activate_obj(prm);

	return _SUCCESS;
}

/* receive link measurement report */
int rrm_recv_link_mens_rep(_adapter *padapter,
	union recv_frame *precv_frame, struct sta_info *psta)
{
	struct rrm_priv *prmpriv = &padapter->fsmpriv.rmpriv;
	int ret = _FALSE;
	struct rrm_obj *prm;
	u16 cid;
	u8 *pdiag_body = (u8 *)(precv_frame->u.hdr.rx_data +
		sizeof(struct rtw_ieee80211_hdr_3addr));
	u8 *pmeas_body = pdiag_body + 3;
	s8 val;

	cid = rrm_cal_cid(pdiag_body[2], RM_MASTER);
	prm = rtw_fsm_get_obj(prmpriv->fsm, cid);
	if (prm == NULL) {
		/* not belong to us, report to upper */
		rtw_cfg80211_rx_rrm_action(padapter, precv_frame);
		return _TRUE;
	}

	//FSM_INFO(prm, "bssid " MAC_FMT "\n", MAC_ARG(obj2mac(prm)));

	prm->p.action_code = pdiag_body[1];
	prm->p.diag_token = pdiag_body[2];

	FSM_TRACE(prm, "action_code = %d\n", prm->p.action_code);
	FSM_TRACE(prm, "diag_token  = %d\n", prm->p.diag_token);
	FSM_TRACE(prm, "xmit_power  = %d dBm\n", pmeas_body[2]);
	FSM_TRACE(prm, "link_margin = %d dBm\n", pmeas_body[3]);
	FSM_TRACE(prm, "xmit_ant    = %d\n", pmeas_body[4]);
	FSM_TRACE(prm, "recv_ant    = %d\n", pmeas_body[5]);
	FSM_TRACE(prm, "RCPI        = %d\n", pmeas_body[6]);
	FSM_TRACE(prm, "RSNI        = %d\n", pmeas_body[7]);

	FSM_INFO(prm, "rx link_meas_report (" MAC_FMT ")\n", MAC_ARG(obj2mac(prm)));

	rtw_fsm_gen_msg(prm, NULL, 0, RRM_EV_recv_rep);

	return ret;
}

static void rtw_rrm_dump_nb_rpt(struct rrm_obj *prm, struct rrm_nb_rpt * nb_rpt)
{
	int i;
	struct rrm_nb_list *pnb_list = nb_rpt->nb_list;

	for (i = 0; i < nb_rpt->nb_list_num; i++) {
		FSM_INFO(prm, "NB %d("MAC_FMT") bss_info=0x%08X opc=0x%02X ch:%d "
			"phy_type=0x%02X pref=%d\n", i + 1,
			MAC_ARG(pnb_list[i].ent.bssid),
			pnb_list[i].ent.bss_info,
			pnb_list[i].ent.reg_class,
			pnb_list[i].ent.ch_num,
			pnb_list[i].ent.phy_type,
			pnb_list[i].preference);
	}
}

int rm_radio_mens_nb_rpt(_adapter *padapter,
	union recv_frame *precv_frame, struct sta_info *psta)
{
	struct rrm_priv *prmpriv = &padapter->fsmpriv.rmpriv;
	struct rrm_priv *rrmpriv;
	struct rrm_obj *prm;
	u8 *pdiag_body = (u8 *)(precv_frame->u.hdr.rx_data +
		sizeof(struct rtw_ieee80211_hdr_3addr));
	u8 *pmeas_body = &pdiag_body[3];
	u32 len = precv_frame->u.hdr.len;
	u16 cid;

	cid = rrm_cal_cid(pdiag_body[2], RM_MASTER);
	prm = rtw_fsm_get_obj(prmpriv->fsm, cid);
	if (prm == NULL) {
		/* not belong to us, report to upper */
		rtw_cfg80211_rx_rrm_action(padapter, precv_frame);
		RTW_WARN("%s() dialog token 0x%02x not found\n", __func__, pdiag_body[2]);
		return _TRUE;
	}

	prm->p.action_code = pdiag_body[1];
	prm->p.diag_token = pdiag_body[2];
	prm->p.e_id = pmeas_body[0];

	FSM_INFO(prm, "rx neighbor report (" MAC_FMT ")\n", MAC_ARG(obj2mac(prm)));

	rrmpriv = obj2priv(prm);
	rrm_parse_nb_list(&rrmpriv->nb_rpt, pdiag_body + 3,
		(len - sizeof(struct rtw_ieee80211_hdr_3addr)));
	rrm_sort_nb_list(&rrmpriv->nb_rpt);
	rrm_upd_nb_ch_list(prm, &rrmpriv->nb_rpt);
	//rtw_rrm_dump_nb_rpt(prm, &rrmpriv->nb_rpt);

	rtw_fsm_gen_msg(prm, NULL, 0, RRM_EV_recv_rep);
	rtw_cfg80211_rx_rrm_action(padapter, precv_frame);

	return _TRUE;
}

unsigned int rrm_on_action(_adapter *padapter, union recv_frame *precv_frame)
{
	struct mlme_ext_info *pmlmeinfo = &padapter->mlmeextpriv.mlmext_info;
	u32 ret = _FAIL;
	u8 *pframe = NULL;
	u8 *pframe_body = NULL;
	u8 action_code = 0;
	u8 diag_token = 0;
	struct rtw_ieee80211_hdr_3addr *whdr;
	struct sta_info *psta;


	if (!(pmlmeinfo->state & WIFI_FW_ASSOC_SUCCESS))
		goto exit;

	pframe = precv_frame->u.hdr.rx_data;

	/* check RA matches or not */
	if (!_rtw_memcmp(adapter_mac_addr(padapter),
		GetAddr1Ptr(pframe), ETH_ALEN))
		goto exit;

	whdr = (struct rtw_ieee80211_hdr_3addr *)pframe;
	RTW_DBG("rrm: %s bssid = " MAC_FMT "\n",
		__func__, MAC_ARG(whdr->addr2));

	psta = rtw_get_stainfo(&padapter->stapriv, whdr->addr2);

        if (!psta) {
		RTW_ERR("rrm: psta not found\n");
                goto exit;
        }

	pframe_body = (unsigned char *)(pframe +
		sizeof(struct rtw_ieee80211_hdr_3addr));

	/* Figure 8-438 radio measurement request frame Action field format */
	/* Category = pframe_body[0] = 5 (Radio Measurement) */
	action_code = pframe_body[1];
	diag_token = pframe_body[2];

	//RTW_DBG("RRM: %s radio_action=%x, diag_token=%x\n", __func__,
	//	action_code, diag_token);

	switch (action_code) {

	case RM_ACT_RADIO_MEAS_REQ:
		//RTW_INFO("RRM: RM_ACT_RADIO_MEAS_REQ\n");
		ret = rrm_recv_radio_mens_req(padapter, precv_frame, psta);
		break;

	case RM_ACT_RADIO_MEAS_REP:
		//RTW_INFO("RRM: RM_ACT_RADIO_MEAS_REP\n");
		ret = rrm_recv_radio_mens_rep(padapter, precv_frame, psta);
		break;

	case RM_ACT_LINK_MEAS_REQ:
		//RTW_INFO("RRM: RM_ACT_LINK_MEAS_REQ\n");
		ret = rrm_recv_link_mens_req(padapter, precv_frame, psta);
		break;

	case RM_ACT_LINK_MEAS_REP:
		//RTW_INFO("RRM: RM_ACT_LINK_MEAS_REP\n");
		ret = rrm_recv_link_mens_rep(padapter, precv_frame, psta);
		break;

	case RM_ACT_NB_REP_REQ:
		//RTW_INFO("RRM: RM_ACT_NB_REP_REQ\n");
		break;

	case RM_ACT_NB_REP_RESP:
		//RTW_INFO("RRM: RM_ACT_NB_REP_RESP\n");
		ret = rm_radio_mens_nb_rpt(padapter, precv_frame, psta);
		break;

	default:
		RTW_WARN("rrm: unknown specturm management action %2x\n",
			action_code);
		break;
	}
exit:
	return ret;
}

static u8 *rm_gen_bcn_detail_2_elem(_adapter *padapter, u8 *pframe, u8 frag_id,
	struct rrm_obj *prm, u8 **pie, u32 *ie_len, unsigned int *fr_len)
{
	u32 frag_max_len = 255 - *fr_len - 4; /* 4: frag_id */
	int len = *ie_len;
	u8 slen = 0, frag_len = 0, more = 0;
	u8 *ies = *pie;
	u8 id, val8;

	if (prm->q.opt.bcn.rep_last_bcn_ind)
		frag_max_len -= 3;

	/* report_detail = 2 */
	if (frag_id == 0) {
		frag_len += _FIXED_IE_LENGTH_;
		len -= _FIXED_IE_LENGTH_;
	}

	while (len >= 0) {

		id = ies[frag_len];
		slen = (unsigned int)ies[frag_len + 1] + 2;

		if ((frag_len + slen) >= frag_max_len ||
		    frag_len == *ie_len) {
			/* ID */
			val8 = 1; /* reported frame body */
			pframe = rtw_set_fixed_ie(pframe, 1, &val8, fr_len);

			/* LEN */
			pframe = rtw_set_fixed_ie(pframe, 1, &frag_len, fr_len);

			/* frame body */
			pframe = rtw_set_fixed_ie(pframe, frag_len, ies, fr_len);

			goto done;
		}

		len -= slen;
		frag_len += slen;
	}
done:
	*pie = ies + frag_len;

	if (len > 0)
		*ie_len = len; /* remain len */
	else
		*ie_len = 0;

	return pframe;
}

static u8 *rm_gen_bcn_detail_1_elem(u8 *pframe, u8 frag_id,
	struct rrm_obj *prm, u8 **ies, u32 *ie_len, unsigned int *fr_len)
{
	unsigned int my_len = 0;
	_adapter *padapter = obj2adp(prm);
	int j = 0, k, len;
	u8 *plen;
	u8 *ptr;
	u8 val8, eid;

	/* report_detail = 1 */

	/* ID */
	val8 = 1; /* 1:reported frame body */
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	plen = pframe;
	val8 = 0;
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	/* report_detail = 1
	 * All fixed-length fields and any requested elements
	 * in the Request element if present
	 */
	pframe = rtw_set_fixed_ie(pframe,
		_FIXED_IE_LENGTH_, *ies, &my_len);

	for (j = 0; j < prm->q.opt.bcn.opt_id_num; j++) {
		switch (prm->q.opt.bcn.opt_id[j]) {
		case bcn_req_ssid:
			/* SSID */
			FSM_TRACE(prm, "bcn_req_ssid\n");
			ptr = rtw_get_ie(*ies + _FIXED_IE_LENGTH_, _SSID_IE_,
				&len, *ie_len - _FIXED_IE_LENGTH_);

			if (ptr)
				pframe = rtw_set_ie(pframe, _SSID_IE_, len, ptr + 2, &my_len);
			break;
		case bcn_req_req:
			if (prm->q.opt.bcn.req_start == NULL)
				break;

			FSM_TRACE(prm, "bcn_req_req");

			for (k=0; k<prm->q.opt.bcn.req_len; k++) {
				eid = prm->q.opt.bcn.req_start[k];

				val8 = *ie_len - _FIXED_IE_LENGTH_;
				ptr = rtw_get_ie(*ies + _FIXED_IE_LENGTH_,
					eid, &len, val8);

				if (!ptr)
					continue;

				switch (eid) {
				case EID_SsId:
					FSM_TRACE(prm, "EID_SSID\n");
					break;
				case EID_QBSSLoad:
					FSM_TRACE(prm, "EID_QBSSLoad\n");
					break;
				case EID_HTCapability:
					FSM_TRACE(prm, "EID_HTCapability\n");
					break;
				case _MDIE_:
					FSM_TRACE(prm, "EID_MobilityDomain\n");
					break;
				case EID_Vendor:
					FSM_TRACE(prm, "EID_Vendor\n");
					break;
				default:
					FSM_TRACE(prm, "EID %d todo\n",eid);
					break;
				}
				pframe = rtw_set_ie(pframe, eid,
					len, ptr + 2, &my_len);
			} /* for() */
			break;
		case bcn_req_rep_detail:
			FSM_DBG(prm, "bcn_req_rep_detail\n");
			break;
		case bcn_req_ap_ch_rep:
			FSM_DBG(prm, "bcn_req_ap_ch_rep\n");
			break;
		default:
			FSM_INFO(prm, "OPT %d not support\n",prm->q.opt.bcn.opt_id[j]);
			break;
		}
	}
	/*
	 * update my length
	 * content length does NOT include ID and LEN
	 */
	val8 = my_len - 2;
	rtw_set_fixed_ie(plen, 1, &val8, &j);

	/* update length to caller */
	*fr_len += my_len;

	return pframe;
}

u8 rm_bcn_req_cond_mach(struct rrm_obj *prm, struct wlan_network *pnetwork)
{
	u8 val8;


	switch(prm->q.opt.bcn.rep_cond.cond) {
	case bcn_rep_cond_immediately:
		return _SUCCESS;
	case bcn_req_cond_rcpi_greater:
		val8 = rm_get_bcn_rcpi(prm, pnetwork);
		if (val8 > prm->q.opt.bcn.rep_cond.threshold)
			return _SUCCESS;
		break;
	case bcn_req_cond_rcpi_less:
		val8 = rm_get_bcn_rcpi(prm, pnetwork);
		if (val8 < prm->q.opt.bcn.rep_cond.threshold)
			return _SUCCESS;
		break;
	case bcn_req_cond_rsni_greater:
		val8 = rm_get_bcn_rsni(prm, pnetwork);
		if (val8 != 255 && val8 > prm->q.opt.bcn.rep_cond.threshold)
			return _SUCCESS;
		break;
	case bcn_req_cond_rsni_less:
		val8 = rm_get_bcn_rsni(prm, pnetwork);
		if (val8 != 255 && val8 < prm->q.opt.bcn.rep_cond.threshold)
			return _SUCCESS;
		break;
	default:
		RTW_ERR("rrm: bcn_req cond %u not support\n",
			prm->q.opt.bcn.rep_cond.cond);
		break;
	}
	return _FALSE;
}

static u8 *rm_gen_bcn_rep_ie (struct rrm_obj *prm, u8 last,
	u8 *pframe, struct wlan_network *pnetwork, unsigned int *fr_len)
{
	int snr, i = 0;
	u8 val8, *plen, *ies, frag_id = 0, more;
	u16 val16;
	u32 val32, ie_len;
	u64 val64;
	unsigned int my_len;
	_adapter *padapter = obj2adp(prm);
	u8 frag_bcn_idx[4] = {0x02, 0x2, 0x1, 0x0};
	u8 last_bcn_ind[3] = {0xa4, 0x1, 0x0};

	ies = pnetwork->network.IEs;

	ie_len = pnetwork->network.IELength;
more:
	my_len = 0;
	plen = pframe + 1;
	pframe = rtw_set_fixed_ie(pframe, 7, &prm->p.e_id, &my_len);

	/* Actual Measurement StartTime */
	val64 = cpu_to_le64(prm->meas_start_time);
	pframe = rtw_set_fixed_ie(pframe, 8, (u8 *)&val64, &my_len);

	/* Measurement Duration */
	val16 = prm->meas_end_time - prm->meas_start_time;
	val16 = cpu_to_le16(val16);
	pframe = rtw_set_fixed_ie(pframe, 2, (u8*)&val16, &my_len);

	/* TODO
	* ReportedFrameInformation:
	* 0 :beacon or probe rsp
	* 1 :pilot frame
	*/
	val8 = 0; /* report frame info */
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	/* RCPI */
	val8 = rm_get_bcn_rcpi(prm, pnetwork);
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	/* RSNI */
	val8 = rm_get_bcn_rsni(prm, pnetwork);
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	/* BSSID */
	pframe = rtw_set_fixed_ie(pframe, 6,
		(u8 *)&pnetwork->network.MacAddress, &my_len);

	/*
	 * AntennaID
	 * 0: unknown
	 * 255: multiple antenna (Diversity)
	 */
	val8 = 0;
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	/* ParentTSF */
	val32 = prm->meas_start_time + pnetwork->network.PhyInfo.free_cnt;
	pframe = rtw_set_fixed_ie(pframe, 4, (u8 *)&val32, &my_len);

	/* Generate Beacon detail */

	/* Reporting Detail values
	 * 0: No fixed length fields or elements
	 * 1: All fixed length fields and any requested elements
	 *    in the Request info element if present
	 * 2: All fixed length fields and elements
	 * 3-255: Reserved
	 */
	if (prm->q.opt.bcn.rep_detail == 1) {
		pframe = rm_gen_bcn_detail_1_elem(pframe, frag_id,
			prm, &ies, &ie_len, &my_len);

	} else if (prm->q.opt.bcn.rep_detail == 2) {
		pframe = rm_gen_bcn_detail_2_elem(padapter, pframe, frag_id,
			prm, &ies, &ie_len, &my_len);

		/* fragment id num */
		more = (ie_len > 0);
		val8 = more << 7 | frag_id;
		frag_bcn_idx[3] = val8;

		pframe = rtw_set_fixed_ie(pframe, sizeof(frag_bcn_idx), frag_bcn_idx, &my_len);

		if (prm->q.opt.bcn.rep_last_bcn_ind) {
			if (last && !more)
				last_bcn_ind[2] = 1;
			pframe = rtw_set_fixed_ie(pframe,
				sizeof(last_bcn_ind), last_bcn_ind, &my_len);
		}
	}

	/*
	* update my length
	* content length does NOT include ID and LEN
	*/
	val8 = my_len - 2;
	rtw_set_fixed_ie(plen, 1, &val8, &i);

	/* update length to caller */
	*fr_len += my_len;

	if (ie_len && prm->q.opt.bcn.rep_detail == 2 && frag_id < 3) {
		frag_id++;
		goto more;
	}

	return pframe;
}

static int rm_match_sub_elem(_adapter *padapter,
	struct rrm_obj *prm, struct wlan_network *pnetwork)
{
	WLAN_BSSID_EX *pbss = &pnetwork->network;
	unsigned int my_len;
	int i, j, k, len;
	u8 *plen;
	u8 *ptr;
	u8 val8, eid;


	my_len = 0;
	/* Reporting Detail values
	 * 0: No fixed length fields or elements
	 * 1: All fixed length fields and any requested elements
	 *    in the Request info element if present
	 * 2: All fixed length fields and elements
	 * 3-255: Reserved
	 */

	/* report_detail != 1  */
	if (prm->q.opt.bcn.rep_detail != 1)
		return _TRUE;

	/* report_detail = 1 */

	for (j = 0; j < prm->q.opt.bcn.opt_id_num; j++) {
		switch (prm->q.opt.bcn.opt_id[j]) {
		case bcn_req_ssid:
			/* SSID */

			FSM_DBG(prm, "bcn_req_ssid\n");

			if (pbss->Ssid.SsidLength == 0)
				return _FALSE;
			break;
		case bcn_req_req:
			if (prm->q.opt.bcn.req_start == NULL)
				break;

			FSM_DBG(prm, "RM: bcn_req_req");

			for (k=0; k<prm->q.opt.bcn.req_len; k++) {
				eid = prm->q.opt.bcn.req_start[k];

				i = pbss->IELength - _FIXED_IE_LENGTH_;
				ptr = rtw_get_ie(pbss->IEs + _FIXED_IE_LENGTH_,
					eid, &len, i);

				switch (eid) {
				case EID_SsId:
					FSM_TRACE(prm, "EID_SSID\n");
					break;
				case EID_QBSSLoad:
					FSM_TRACE(prm, "EID_QBSSLoad\n");
					break;
				case EID_HTCapability:
					FSM_TRACE(prm, "EID_HTCapability\n");
					break;
				case _MDIE_:
					FSM_TRACE(prm, "EID_MobilityDomain\n");
					break;
				case EID_Vendor:
					FSM_TRACE(prm, "EID_Vendor\n");
					break;
				default:
					FSM_TRACE(prm, "EID %d todo\n",eid);
					break;
				}
				if (!ptr) {
					FSM_INFO(prm, "EID %d not found\n",eid);
					return _FALSE;
				}
			} /* for() */
			break;
		case bcn_req_rep_detail:
			FSM_DBG(prm, "bcn_req_rep_detail\n");
			break;
		case bcn_req_ap_ch_rep:
			FSM_DBG(prm, "bcn_req_ap_ch_rep\n");
			break;
		default:
			FSM_INFO(prm, "OPT %d not found\n",prm->q.opt.bcn.opt_id[j]);
			break;
		}
	}
	return _TRUE;
}

static int retrieve_scan_result(struct rrm_obj *prm)
{
	_list *plist, *phead;
	_queue *queue;
	_adapter *padapter = obj2adp(prm);
	struct rtw_ieee80211_channel *pch_set;
	struct wlan_network *pnetwork = NULL;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	int i, j;
	PWLAN_BSSID_EX pbss;
	unsigned int matched_network;
	int len, my_len;
	u8 last = 0;
	u8 buf_idx, *pbuf = NULL, *tmp_buf = NULL;
	u16 xframe_ext_sz = SZ_XMITFRAME_EXT;


	tmp_buf = rtw_malloc(xframe_ext_sz);
	if (tmp_buf == NULL)
		return 0;

	matched_network = 0;
	queue = &(pmlmepriv->scanned_queue);

	_rtw_spinlock_bh(&(pmlmepriv->scanned_queue.lock));

	phead = get_list_head(queue);
	plist = get_next(phead);

	/* get requested measurement channel set */
	pch_set = prm->q.ch_set;

	prm->ap_num = 0;
	/* search scan queue to find requested SSID */
	while (1) {

		if (rtw_end_of_queue_search(phead, plist) == _TRUE)
			break;

		pnetwork = LIST_CONTAINOR(plist, struct wlan_network, list);
		pbss = &pnetwork->network;

#if 0
		RTW_INFO("RM: ch %u ssid %s bssid "MAC_FMT"\n",
			pbss->Configuration.DSConfig, pbss->Ssid.Ssid,
			MAC_ARG(pbss->MacAddress));
		/*
		* report network if requested channel set contains
		* the channel matchs selected network
		*/
		if (rtw_chset_search_ch(adapter_to_chset(padapter),
			pbss->Configuration.DSConfig) < 0) /* not match */
			goto next;

		if (rtw_mlme_band_check(padapter, pbss->Configuration.DSConfig)
			== _FALSE)
			goto next;
#endif
		if (rtw_validate_ssid(&(pbss->Ssid)) == _FALSE)
			goto next;

		/* match bssid */
		if (is_wildcard_bssid(prm->q.bssid) == _FALSE)
			if (_rtw_memcmp(prm->q.bssid,
				pbss->MacAddress, 6) == _FALSE)
				goto next;
		/*
		 * default wildcard SSID. wildcard SSID:
		 * A SSID value (null) used to represent all SSIDs
		 */

		/* match ssid */
		if ((prm->q.opt.bcn.ssid.SsidLength > 0) &&
			_rtw_memcmp(prm->q.opt.bcn.ssid.Ssid,
			pbss->Ssid.Ssid,
			prm->q.opt.bcn.ssid.SsidLength) == _FALSE)
			goto next;

		/* go through measurement requested channels */
		for (i = 0; i < prm->q.ch_set_ch_amount; i++) {
			if ((pch_set[i].hw_value) ==
				(pbss->Configuration.DSConfig)) /* match ch */
				break;
		}
		if (i >= prm->q.ch_set_ch_amount) /* channel mismatch */
			goto next;

		/* match condition */
		if (rm_bcn_req_cond_mach(prm, pnetwork) == _FALSE) {
			RTW_INFO("RM: condition mismatch ch %u ssid %s bssid "MAC_FMT"\n",
				pbss->Configuration.DSConfig, pbss->Ssid.Ssid,
				MAC_ARG(pbss->MacAddress));
			RTW_INFO("RM: condition %u:%u\n",
				prm->q.opt.bcn.rep_cond.cond,
				prm->q.opt.bcn.rep_cond.threshold);
			goto next;
		}

		/* match subelement */
		if (rm_match_sub_elem(padapter, prm, pnetwork) == _FALSE)
			goto next;

		/* Found a matched SSID */
		matched_network++;

		RTW_INFO("RM: ch %u Found %s bssid "MAC_FMT"\n",
			pbss->Configuration.DSConfig, pbss->Ssid.Ssid,
			MAC_ARG(pbss->MacAddress));

		prm->ap[prm->ap_num++] = pnetwork;
		if (prm->ap_num >= MAX_RM_AP_NUM)
			break;
next:
		plist = get_next(plist);
	} /* while() */

	/* generate packet */
	my_len = 0;
	buf_idx = 0;
	for (i = 0; i < prm->ap_num; i++) {
		len = 0;
		last = (i == (prm->ap_num - 1));
		_rtw_memset(tmp_buf, 0, xframe_ext_sz);
		pnetwork = prm->ap[i];
		rm_gen_bcn_rep_ie(prm, last, tmp_buf, pnetwork, &len);
		prm->ap[i] = NULL;
new_packet:
		if (my_len == 0) {
			pbuf = rtw_malloc(xframe_ext_sz);
			if (pbuf == NULL)
				goto fail;
			prm->buf[buf_idx].pbuf = pbuf;
		}

		if ((xframe_ext_sz - (my_len+len+24+4)) > 0) {
			pbuf = rtw_set_fixed_ie(pbuf,
				len, tmp_buf, &my_len);
			prm->buf[buf_idx].len = my_len;
		} else {
			if (my_len == 0) /* not enough space */
				goto fail;

			my_len = 0;
			buf_idx++;

			if (buf_idx >= MAX_RM_PKT_NUM) {
				for (j = i; j < prm->ap_num; j++)
					prm->ap[i] = NULL;
				break;
			}

			goto new_packet;
		}
	}

fail:
	_rtw_spinunlock_bh(&(pmlmepriv->scanned_queue.lock));

	if (tmp_buf)
		rtw_mfree(tmp_buf, xframe_ext_sz);

	RTW_INFO("RM: Found %d matched %s (%d)\n", matched_network,
		prm->q.opt.bcn.ssid.Ssid, buf_idx+1);

	if (prm->buf[buf_idx].pbuf)
		return buf_idx + 1;

	return 0;
}

int rrm_issue_beacon_rep(struct rrm_obj *prm)
{
	int i, my_len;
	u8 *pframe;
	_adapter *padapter = obj2adp(prm);
	struct pkt_attrib *pattr;
	struct xmit_frame *pmgntframe;
	struct xmit_priv *pxmitpriv = &(padapter->xmitpriv);
	int pkt_num;
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);
	u16 xframe_ext_sz = SZ_XMITFRAME_EXT;

	pkt_num = retrieve_scan_result(prm);

	if (pkt_num == 0) {
		rrm_issue_null_reply(prm);
		return _SUCCESS;
	}

	FSM_INFO(prm, "tx bcn_req report\n");

	for (i=0;i<pkt_num;i++) {

		pmgntframe = alloc_mgtxmitframe(pxmitpriv);
		if (pmgntframe == NULL) {
			FSM_ERR(prm, "%s alloc xmit_frame fail\n",__func__);
			goto fail;
		}
		pattr = &pmgntframe->attrib;
		pframe = build_wlan_hdr(prm, padapter,
			pmgntframe, obj2sta(prm), WIFI_ACTION);
		pframe = rtw_set_fixed_ie(pframe,
			3, &prm->p.category, &pattr->pktlen);

		my_len = 0;
		pframe = rtw_set_fixed_ie(pframe,
			prm->buf[i].len, prm->buf[i].pbuf, &my_len);

		pattr->pktlen += my_len;
		pattr->last_txcmdsz = pattr->pktlen;
		dump_mgntframe(padapter, pmgntframe);
	}
fail:
	/*GEORGIA_TODO_FIXIT*/
	for (i = 0; i < pkt_num; i++) {
		if (prm->buf[i].pbuf) {
			rtw_mfree(prm->buf[i].pbuf, xframe_ext_sz);
			prm->buf[i].pbuf = NULL;
			prm->buf[i].len = 0;
		}
	}
	return _SUCCESS;
}

/* neighbor request */
int rrm_issue_nb_req(struct rrm_obj *prm)
{
	_adapter *padapter = obj2adp(prm);
	struct sta_info *psta = obj2sta(prm);
	struct _ADAPTER_LINK *padapter_link = psta->padapter_link;
	struct link_mlme_priv *pmlmepriv = &padapter_link->mlmepriv;
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct xmit_frame *pmgntframe = NULL;
	struct pkt_attrib *pattr = NULL;
	u8 val8;
	u8 *pframe = NULL;


	FSM_DBG(prm, "%s\n", __func__);

	pmgntframe = alloc_mgtxmitframe(pxmitpriv);
	if (pmgntframe == NULL) {
		FSM_ERR(prm, "%s alloc xmit_frame fail\n",__func__);
		return _FALSE;
	}
	pattr = &pmgntframe->attrib;
	pframe = build_wlan_hdr(prm, padapter, pmgntframe, psta, WIFI_ACTION);
	pframe = rtw_set_fixed_ie(pframe,
		3, &prm->q.category, &pattr->pktlen);

	if (prm->q.pssid) {

		u8 sub_ie[64] = {0};
		u8 *pie = &sub_ie[2];

		FSM_INFO(prm, "tx neighbor request to search %s\n",
			pmlmepriv->cur_network.network.Ssid.Ssid);

		val8 = strlen(prm->q.pssid);
		sub_ie[0] = 0; /*SSID*/
		sub_ie[1] = val8;

		_rtw_memcpy(pie, prm->q.pssid, val8);

		pframe = rtw_set_fixed_ie(pframe, val8 + 2,
			sub_ie, &pattr->pktlen);
	} else {

		if (!pmlmepriv->cur_network.network.Ssid.SsidLength) {
			FSM_INFO(prm, "tx neighbor request\n");
		} else {
			u8 sub_ie[64] = {0};
			u8 *pie = &sub_ie[2];

			FSM_INFO(prm, "tx neighbor request to search %s\n",
				pmlmepriv->cur_network.network.Ssid.Ssid);

			sub_ie[0] = 0; /*SSID*/
			sub_ie[1] = pmlmepriv->cur_network.network.Ssid.SsidLength;

			_rtw_memcpy(pie, pmlmepriv->cur_network.network.Ssid.Ssid,
				pmlmepriv->cur_network.network.Ssid.SsidLength);

			pframe = rtw_set_fixed_ie(pframe,
				pmlmepriv->cur_network.network.Ssid.SsidLength + 2,
				sub_ie, &pattr->pktlen);
		}
	}

	pattr->last_txcmdsz = pattr->pktlen;
	dump_mgntframe(padapter, pmgntframe);

	return _SUCCESS;
}

/* issue link measurement request */
int rrm_issue_link_meas_req(struct rrm_obj *prm)
{
	_adapter *padapter = obj2adp(prm);
	struct _ADAPTER_LINK *padapter_link = GET_PRIMARY_LINK(padapter);
	struct sta_info *psta = obj2sta(prm);
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct xmit_frame *pmgntframe = NULL;
	struct pkt_attrib *pattr = NULL;
	u8 *pframe = NULL;
	s8 pwr_used = 0, path_a_pwr = 0;


	FSM_INFO(prm, "%s\n", __func__);

	pmgntframe = alloc_mgtxmitframe(pxmitpriv);
	if (pmgntframe == NULL) {
		FSM_ERR(prm, "%s alloc xmit_frame fail\n",__func__);
		return _FALSE;
	}
	pattr = &pmgntframe->attrib;
	pframe = build_wlan_hdr(prm, padapter, pmgntframe, psta, WIFI_ACTION);

	/* Category, Action code, Dialog token */
	pframe = rtw_set_fixed_ie(pframe,
		3, &prm->q.category, &pattr->pktlen);

	/* xmit power used */
	/* we don't know actual TX power due to RA may change TX rate;
	 * But if we fix TX rate then we can get specific tx power
	 */
	pattr->rate = MGN_6M;
	rm_get_tx_power(padapter, padapter_link->mlmeextpriv.chandef.band, MGN_6M, &pwr_used);
	pframe = rtw_set_fixed_ie(pframe,
		1, &pwr_used, &pattr->pktlen);

	/* Max xmit power */
	rrm_get_path_a_max_tx_power(padapter, &path_a_pwr);
	pframe = rtw_set_fixed_ie(pframe,
		1, &path_a_pwr, &pattr->pktlen);

	pattr->last_txcmdsz = pattr->pktlen;
	dump_mgntframe(padapter, pmgntframe);

	return _SUCCESS;
}

/* issue link measurement report */
int rrm_issue_link_meas_rep(struct rrm_obj *prm)
{
	u8 val8;
	u8 *pframe;
	unsigned int my_len;
	_adapter *padapter = obj2adp(prm);
	struct _ADAPTER_LINK *padapter_link = GET_PRIMARY_LINK(padapter);
	struct xmit_frame *pmgntframe;
	struct pkt_attrib *pattr;
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct sta_info *psta = obj2sta(prm);
	int i;
	u8 tpc[4];
	s8 pwr_used;


	FSM_INFO(prm, "%s\n", __func__);

	pmgntframe = alloc_mgtxmitframe(pxmitpriv);
	if (pmgntframe == NULL) {
		FSM_ERR(prm, "ERR %s alloc xmit_frame fail\n",__func__);
		return _FALSE;
	}
	pattr = &pmgntframe->attrib;
	pframe = build_wlan_hdr(prm, padapter, pmgntframe, psta, WIFI_ACTION);
	/* Category, action code, Dialog token */
	pframe = rtw_set_fixed_ie(pframe, 3,
		&prm->p.category, &pattr->pktlen);

	my_len = 0;

	/* TPC report */
	rm_get_tx_power(padapter, padapter_link->mlmeextpriv.chandef.band, MGN_6M, &pwr_used);
	tpc[0] = EID_TPC;
	tpc[1] = 2; /* length */

	/* TX power */
	tpc[2] = pwr_used;

	/* link margin */
	rrm_get_rx_sensitivity(padapter, prm->q.rx_bw, prm->q.rx_rate, &pwr_used);
	tpc[3] = prm->q.rx_pwr - pwr_used; /* RX sensitivity */
	pattr->rate = MGN_6M; /* use fix rate to get fixed RX sensitivity */

	FSM_TRACE(prm, "rx_pwr=%ddBm - rx_sensitivity=%ddBm = link_margin=%ddB\n",
		prm->q.rx_pwr, pwr_used, tpc[3]);

	pframe = rtw_set_fixed_ie(pframe, 4, tpc, &my_len);

	/* RECV antenna ID */
	val8 = 0; /* unknown antenna */
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	/* XMIT antenna ID */
	/* Fix rate 6M(1T) always use main antenna to TX */
	val8 = 1; /* main antenna */
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	/* RCPI */
	val8 = translate_dbm_to_rcpi(prm->q.rx_pwr);
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	/* RSNI */
	val8 = prm->q.rx_rsni;
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	/* length */
	//val8 = (u8)my_len-2;
	//rtw_set_fixed_ie(plen, 1, &val8, &i); /* use variable i to ignore it */

	pattr->pktlen += my_len;
	pattr->last_txcmdsz = pattr->pktlen;
	dump_mgntframe(padapter, pmgntframe);

	return _SUCCESS;
}

static u8 *rm_gen_bcn_req_s_elem(_adapter *padapter,
	struct rrm_obj *prm, u8 *pframe, unsigned int *fr_len)
{
	u8 val8, l;
	int i;
	unsigned int my_len = 0;
	struct _RT_OPERATING_CLASS *op;


	val8 = bcn_req_active; /* measurement mode T8-64 */
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	/* bssid */
	pframe = rtw_set_fixed_ie(pframe, 6, prm->q.bssid, &my_len);

	/*
	 * opt ssid (0)
	 */
	l = MIN(32, (int)prm->q.opt.bcn.ssid.SsidLength);

	if (l > 0 && l <= 32) {
		/* Type */
		val8 = bcn_req_ssid;
		pframe = rtw_set_fixed_ie(pframe, 1,
			&val8, &my_len);
		/* Len */
		pframe = rtw_set_fixed_ie(pframe, 1,
			&l, &my_len);
		/* Value */
		pframe = rtw_set_fixed_ie(pframe, l,
			prm->q.opt.bcn.ssid.Ssid, &my_len);
	}

	/*
	 * opt reporting detail (2)
	 */
	/* Type */
	val8 = bcn_req_rep_detail;
	pframe = rtw_set_fixed_ie(pframe, 1,
		&val8, &my_len);
	/* Len */
	l = 1;
	pframe = rtw_set_fixed_ie(pframe, 1,
		&l, &my_len);
	/* Value */
	pframe = rtw_set_fixed_ie(pframe, l,
		&prm->q.opt.bcn.rep_detail, &my_len);

	/*
	 * opt request (10)
	 */

	if (prm->q.opt.bcn.req_id_num > 0) {
		/* Type */
		val8 = bcn_req_req;
		pframe = rtw_set_fixed_ie(pframe, 1,
			&val8, &my_len);
		/* Len */
		l = prm->q.opt.bcn.req_id_num;
		pframe = rtw_set_fixed_ie(pframe, 1,
			&l, &my_len);
		/* Value */
		pframe = rtw_set_fixed_ie(pframe, l,
			prm->q.opt.bcn.req_id, &my_len);
	}

	/*
	 * opt ap channel report (51)
	 */
	for (i = 0; i < prm->q.opt.bcn.ap_ch_rpt_num; i++) {
		op = prm->q.opt.bcn.ap_ch_rpt[i];
		if (op == NULL)
			break;
		/* Type */
		val8 = bcn_req_ap_ch_rep;
		pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);
		l = (u8)op->Len + 1;
		/* length */
		pframe = rtw_set_fixed_ie(pframe, 1, &l, &my_len);

		/* op class */
		val8 = op->global_op_class;
		pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);
		/* channel */
		pframe = rtw_set_fixed_ie(pframe, op->Len, op->Channel, &my_len);
	}

	/* update length to caller */
	*fr_len += my_len;

	/* optional subelements */
	return pframe;
}

static u8 *rm_gen_ch_load_req_s_elem(_adapter *padapter,
	u8 *pframe, unsigned int *fr_len)
{
	u8 val8;
	unsigned int my_len = 0;


	val8 = 1; /* 1: channel load T8-60 */
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	val8 = 2; /* channel load length = 2 (extensible)  */
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	val8 = 0; /* channel load condition : 0 (issue when meas done) T8-61 */
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	val8 = 0; /* channel load reference value : 0 */
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	/* update length to caller */
	*fr_len += my_len;

	return pframe;
}

static u8 *rm_gen_noise_histo_req_s_elem(_adapter *padapter,
	u8 *pframe, unsigned int *fr_len)
{
	u8 val8;
	unsigned int my_len = 0;


	val8 = 1; /* 1: noise histogram T8-62 */
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	val8 = 2; /* noise histogram length = 2 (extensible)  */
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	val8 = 0; /* noise histogram condition : 0 (issue when meas done) T8-63 */
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	val8 = 0; /* noise histogram reference value : 0 */
	pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);

	/* update length to caller */
	*fr_len += my_len;

	return pframe;
}

int rrm_issue_radio_meas_req(struct rrm_obj *prm)
{
	u8 val8;
	u8 *pframe;
	u8 *plen;
	u16 val16;
	int my_len, i = 0;
	struct xmit_frame *pmgntframe;
	struct pkt_attrib *pattr;
	_adapter *padapter = obj2adp(prm);
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;


	FSM_INFO(prm, "%s - %s\n", __func__, rm_type_req_name(prm->q.m_type));

	pmgntframe = alloc_mgtxmitframe(pxmitpriv);
	if (pmgntframe == NULL) {
		FSM_ERR(prm, "%s alloc xmit_frame fail\n",__func__);
		return _FALSE;
	}
	pattr = &pmgntframe->attrib;
	pframe = build_wlan_hdr(prm, padapter, pmgntframe, obj2sta(prm), WIFI_ACTION);

	/* Category, Action code, Dialog token */
	pframe = rtw_set_fixed_ie(pframe, 3, &prm->q.category, &pattr->pktlen);

	/* repeat */
	val16 = cpu_to_le16(prm->q.rpt);
	pframe = rtw_set_fixed_ie(pframe, 2,
		(unsigned char *)&(val16), &pattr->pktlen);

	my_len = 0;
	plen = pframe + 1;

	/* Element ID, Length, Meas token, Meas Mode, Meas type, op class, ch */
	pframe = rtw_set_fixed_ie(pframe, 7, &prm->q.e_id, &my_len);

	/* random interval */
	val16 = cpu_to_le16(prm->q.rand_intvl); /* TU */
	pframe = rtw_set_fixed_ie(pframe, 2, (u8 *)&val16, &my_len);

	/* measurement duration */
	val16 = cpu_to_le16(prm->q.meas_dur);
	pframe = rtw_set_fixed_ie(pframe, 2, (u8 *)&val16, &my_len);

	/* optional subelement */
	switch (prm->q.m_type) {
	case bcn_req:
		pframe = rm_gen_bcn_req_s_elem(padapter, prm,  pframe, &my_len);
		break;
	case ch_load_req:
		pframe = rm_gen_ch_load_req_s_elem(padapter, pframe, &my_len);
		break;
	case noise_histo_req:
		pframe = rm_gen_noise_histo_req_s_elem(padapter,
			pframe, &my_len);
		break;
	case basic_req:
	default:
		break;
	}

	/* length */
	val8 = (u8)my_len - 2;
	rtw_set_fixed_ie(plen, 1, &val8, &i);

	pattr->pktlen += my_len;

	pattr->last_txcmdsz = pattr->pktlen;
	dump_mgntframe(padapter, pmgntframe);

	return _SUCCESS;
}

static int rrm_issue_meas_req(struct rrm_obj *prm)
{
	struct rrm_priv *rrmpriv = obj2priv(prm);

	switch (prm->q.action_code) {
	case RM_ACT_RADIO_MEAS_REQ:
		switch (prm->q.m_type) {
		case bcn_req:
		case ch_load_req:
		case noise_histo_req:
			rrm_issue_radio_meas_req(prm);
			break;
		default:
			break;
		} /* meas_type */
		break;
	case RM_ACT_NB_REP_REQ:
		/* invilid NB report */
		rrmpriv->nb_rpt.last_update = 0;
		/* issue NB report request */
		rrm_issue_nb_req(prm);
		break;
	case RM_ACT_LINK_MEAS_REQ:
		rrm_issue_link_meas_req(prm);
		break;
	default:
		return _FALSE;
	} /* action_code */

	return _SUCCESS;
}

int rm_radio_meas_report_cond(struct rrm_obj *prm)
{
	u8 val8;
	int i = 0, ret = _FAIL;


	switch (prm->q.m_type) {
	case ch_load_req:
		val8 = prm->p.ch_load;
		switch (prm->q.opt.clm.rep_cond.cond) {
		case ch_load_cond_immediately:
			ret = _SUCCESS;
			break;
		case ch_load_cond_anpi_equal_greater:
			if (val8 >= prm->q.opt.clm.rep_cond.threshold)
				ret = _SUCCESS;
			break;
		case ch_load_cond_anpi_equal_less:
			if (val8 <= prm->q.opt.clm.rep_cond.threshold)
				ret = _SUCCESS;
			break;
		default:
			break;
		}
		break;
	case noise_histo_req:
		val8 = prm->p.anpi;
		switch (prm->q.opt.nhm.rep_cond.cond) {
		case noise_histo_cond_immediately:
			ret = _SUCCESS;
			break;
		case noise_histo_cond_anpi_equal_greater:
			if (val8 >= prm->q.opt.nhm.rep_cond.threshold)
				ret = _SUCCESS;
			break;
		case noise_histo_cond_anpi_equal_less:
			if (val8 <= prm->q.opt.nhm.rep_cond.threshold)
				ret = _SUCCESS;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return ret;
}

int retrieve_radio_meas_result(struct rrm_obj *prm)
{
#ifdef CONFIG_RTW_ACS
	struct dvobj_priv *dvobj = adapter_to_dvobj(obj2adp(prm));
#endif
	int i;
	u8 val8;

#if 0
	int ch = -1;

	ch = rtw_chset_search_bch(adapter_to_chset(obj2adp(prm)),
		prm->q.band, prm->q.ch_num);

	if ((ch == -1) || (ch >= MAX_CHANNEL_NUM)) {
		FSM_ERR(prm, "get ch(CH:%d) fail\n", prm->q.ch_num);
		ch = 0;
	}
#endif
	switch (prm->q.m_type) {
	case ch_load_req:
#if 0 /* def CONFIG_RTW_ACS */
		val8 = hal_data->acs.clm_ratio[ch];
#else
		val8 = 0;
#endif
		prm->p.ch_load = val8;
		break;
	case noise_histo_req:
#if 0 /* def CONFIG_RTW_ACS */
		/* ANPI */
		prm->p.anpi = hal_data->acs.nhm_ratio[ch];

		/* IPI 0~10 */
		for (i=0;i<11;i++)
			prm->p.ipi[i] = hal_data->acs.nhm[ch][i];
#else
		val8 = 0;
		prm->p.anpi = val8;
		for (i=0;i<11;i++)
			prm->p.ipi[i] = val8;
#endif
		break;
	default:
		break;
	}
	return _SUCCESS;
}

int rrm_issue_radio_meas_rep(struct rrm_obj *prm)
{
	u8 val8;
	u8 *pframe;
	u8 *plen;
	u16 val16;
	u64 val64;
	unsigned int my_len;
	_adapter *padapter = obj2adp(prm);
	struct xmit_frame *pmgntframe;
	struct pkt_attrib *pattr;
	struct xmit_priv *pxmitpriv = &(padapter->xmitpriv);
	struct sta_info *psta = obj2sta(prm);
	int i = 0;


	FSM_INFO(prm, "%s\n", __func__);

	pmgntframe = alloc_mgtxmitframe(pxmitpriv);
	if (pmgntframe == NULL) {
		FSM_ERR(prm, "ERR %s alloc xmit_frame fail\n",__func__);
		return _FALSE;
	}
	pattr = &pmgntframe->attrib;
	pframe = build_wlan_hdr(prm, padapter, pmgntframe, psta, WIFI_ACTION);
	pframe = rtw_set_fixed_ie(pframe, 3,
		&prm->p.category, &pattr->pktlen);

	my_len = 0;
	plen = pframe + 1;
	pframe = rtw_set_fixed_ie(pframe, 7, &prm->p.e_id, &my_len);

	/* Actual Meas start time - 8 bytes */
	val64 = cpu_to_le64(prm->meas_start_time);
	pframe = rtw_set_fixed_ie(pframe, 8, (u8 *)&val64, &my_len);

	/* measurement duration */
	val16 = prm->meas_end_time - prm->meas_start_time;
	val16 = cpu_to_le16(val16);
	pframe = rtw_set_fixed_ie(pframe, 2, (u8 *)&val16, &my_len);

	/* optional subelement */
	switch (prm->q.m_type) {
	case ch_load_req:
		val8 = prm->p.ch_load;
		pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);
		break;
	case noise_histo_req:
		/*
		 * AntennaID
		 * 0: unknown
		 * 255: multiple antenna (Diversity)
		 */
		val8 = 0;
		pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);
		/* ANPI */
		val8 = prm->p.anpi;
		pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);
		/* IPI 0~10 */
		for (i=0;i<11;i++) {
			val8 = prm->p.ipi[i];
			pframe = rtw_set_fixed_ie(pframe, 1, &val8, &my_len);
		}
		break;
	default:
		break;
	}
	/* length */
	val8 = (u8)my_len-2;
	rtw_set_fixed_ie(plen, 1, &val8, &i); /* use variable i to ignore it */

	pattr->pktlen += my_len;
	pattr->last_txcmdsz = pattr->pktlen;
	dump_mgntframe(padapter, pmgntframe);

	return _SUCCESS;
}

void rtw_ap_parse_sta_rm_en_cap(_adapter *padapter,
	struct sta_info *psta, struct rtw_ieee802_11_elems *elem)
{
	if (elem->rm_en_cap) {
		RTW_INFO("rrm: assoc.rm_en_cap="RM_CAP_FMT"\n",
			RM_CAP_ARG(elem->rm_en_cap));

		_rtw_memcpy(psta->rm_en_cap, (elem->rm_en_cap),
			MIN(elem->rm_en_cap_len, sizeof(psta->rm_en_cap)));
	}
}

void RM_IE_handler(_adapter *padapter, PNDIS_802_11_VARIABLE_IEs pIE)
{
	int i;

	_rtw_memcpy(&padapter->fsmpriv.rmpriv.rm_en_cap_assoc, pIE->data,
		    MIN(pIE->Length, sizeof(padapter->fsmpriv.rmpriv.rm_en_cap_assoc)));
	RTW_INFO("rrm: assoc.rm_en_cap="RM_CAP_FMT"\n", RM_CAP_ARG(pIE->data));
}

/* Debug command */

#if (RM_SUPPORT_IWPRIV_DBG)
static int hex2num(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

int hex2byte(const char *hex)
{
	int a, b;
	a = hex2num(*hex++);
	if (a < 0)
		return -1;
	b = hex2num(*hex++);
	if (b < 0)
		return -1;
	return (a << 4) | b;
}

static char * hwaddr_parse(char *txt, u8 *addr)
{
	size_t i;

	for (i = 0; i < ETH_ALEN; i++) {
		int a;

		a = hex2byte(txt);
		if (a < 0)
			return NULL;
		txt += 2;
		addr[i] = a;
		if (i < ETH_ALEN - 1 && *txt++ != ':')
			return NULL;
	}
	return txt;
}

void rm_dbg_list_sta(_adapter *padapter, char *s)
{
	int i;
	struct sta_info *psta;
	struct sta_priv *pstapriv = &padapter->stapriv;
	_list *plist, *phead;

	sprintf(pstr(s), "\n");
	_rtw_spinlock_bh(&pstapriv->sta_hash_lock);
	for (i = 0; i < NUM_STA; i++) {
		phead = &(pstapriv->sta_hash[i]);
		plist = get_next(phead);

		while ((rtw_end_of_queue_search(phead, plist)) == _FALSE) {
			psta = LIST_CONTAINOR(plist,
				struct sta_info, hash_list);

			plist = get_next(plist);

			sprintf(pstr(s), "=========================================\n");
			sprintf(pstr(s), "mac=" MAC_FMT "\n",
				MAC_ARG(psta->phl_sta->mac_addr));
			sprintf(pstr(s), "state=0x%x, aid=%d, macid=%d\n",
				psta->state, psta->phl_sta->aid, psta->phl_sta->macid);
			sprintf(pstr(s), "rm_cap="RM_CAP_FMT"\n",
				RM_CAP_ARG(psta->rm_en_cap));
		}

	}
	_rtw_spinunlock_bh(&pstapriv->sta_hash_lock);
	sprintf(pstr(s), "=========================================\n");
}

#if 0
void rm_dbg_help(_adapter *padapter, char *s)
{
	int i;


	sprintf(pstr(s), "\n");
	sprintf(pstr(s), "rrm list_sta\n");
	sprintf(pstr(s), "rrm list_meas\n");

	sprintf(pstr(s), "rrm add_meas <aid=1|mac=>,m=<bcn|clm|nhm|nb|link>,rpt=\n");
	sprintf(pstr(s), "rrm run_meas <aid=1|evid=>\n");
	sprintf(pstr(s), "rrm del_meas\n");

	sprintf(pstr(s), "rrm run_meas cid=xxxx,ev=xx\n");
	sprintf(pstr(s), "rrm activate\n");

	for (i=0;i<RRM_EV_max;i++)
		sprintf(pstr(s), "\t%2d %s\n",i, rtw_fsm_event_name(i) );
	sprintf(pstr(s), "\n");
}
#endif

struct sta_info *rm_get_sta(_adapter *padapter, u16 aid, u8* pbssid)
{
	int i;
	struct sta_info *psta = NULL;
	struct sta_priv *pstapriv = &padapter->stapriv;
	_list *plist, *phead;


	_rtw_spinlock_bh(&pstapriv->sta_hash_lock);

	for (i = 0; i < NUM_STA; i++) {
		phead = &(pstapriv->sta_hash[i]);
		plist = get_next(phead);

		while ((rtw_end_of_queue_search(phead, plist)) == _FALSE) {
			psta = LIST_CONTAINOR(plist,
				struct sta_info, hash_list);

			plist = get_next(plist);

			if (psta->phl_sta->aid == aid)
				goto done;

			if (pbssid && _rtw_memcmp(psta->phl_sta->mac_addr,
				pbssid, 6))
				goto done;
		}

	}
	psta = NULL;
done:
	_rtw_spinunlock_bh(&pstapriv->sta_hash_lock);
	return psta;
}

static int rm_dbg_modify_meas(_adapter *padapter, char *s)
{
	struct rrm_priv *prmpriv = &padapter->fsmpriv.rmpriv;
	struct mlme_ext_info *pmlmeinfo = &padapter->mlmeextpriv.mlmext_info;
	struct rrm_obj *prm;
	struct sta_info *psta;
	char *pmac, *ptr, *paid, *prpt, *pnbp, *pclm, *pnhm, *pbcn, *plnk;
	unsigned val;
	u8 bssid[ETH_ALEN];
	int i;

	/* example :
	* rrm add_meas <aid=1|mac=>,m=<nb|clm|nhm|bcn|link>,<rept=>
	* rrm run_meas <aid=1|evid=>
	*/
	paid = strstr(s, "aid=");
	pmac = strstr(s, "mac=");
	pbcn = strstr(s, "m=bcn");
	pclm = strstr(s, "m=clm");
	pnhm = strstr(s, "m=nhm");
	pnbp = strstr(s, "m=nb");
	plnk = strstr(s, "m=link");
	prpt = strstr(s, "rpt=");

	/* set all ',' to NULL (end of line) */
	ptr = s;
	while (ptr) {
		ptr = strchr(ptr, ',');
		if (ptr) {
			*(ptr) = 0x0;
			ptr++;
		}
	}
	prm = (struct rrm_obj *)prmpriv->prm_sel;
	prm->q.m_token = rrm_gen_meas_token(padapter);
	psta = obj2sta(prm);

	for (i=0;i<6;i++)
		prm->q.bssid[i] = 0xff; /* wildcard bssid */
	if (paid) { /* find sta_info according to aid */
		paid += 4; /* skip aid= */
		sscanf(paid, "%u", &val); /* aid=x */
		psta = rm_get_sta(padapter, val, NULL);

	} else if (pmac) { /* find sta_info according to bssid */
		pmac += 4; /* skip mac= */
		if (hwaddr_parse(pmac, bssid) == NULL) {
			sprintf(pstr(s), "Err: \nincorrect mac format\n");
			return _FAIL;
		}
		psta = rm_get_sta(padapter, 0xff, bssid);
	}

	if (psta) {
		prm->q.diag_token = rrm_gen_dialog_token(padapter);
		prm->cid = rrm_cal_cid(prm->q.diag_token, RM_MASTER);
	} else
		return _FAIL;

	prm->q.action_code = RM_ACT_RADIO_MEAS_REQ;
	if (pbcn) {
		prm->q.m_type = bcn_req;
		prm->q.rand_intvl = le16_to_cpu(100);
		prm->q.meas_dur = le16_to_cpu(100);
	} else if (pnhm) {
		prm->q.m_type = noise_histo_req;
	} else if (pclm) {
		prm->q.m_type = ch_load_req;
	} else if (pnbp) {
		prm->q.action_code = RM_ACT_NB_REP_REQ;
	} else if (plnk) {
		prm->q.action_code = RM_ACT_LINK_MEAS_REQ;
	} else
		return _FAIL;

	if (prpt) {
		prpt += 4; /* skip rpt= */
		sscanf(prpt, "%u", &val);
		prm->q.rpt = (u8)val;
	}

	return _SUCCESS;
}

static void rm_dbg_activate_meas(_adapter *padapter, char *s)
{
	struct rrm_priv *prmpriv = &(padapter->fsmpriv.rmpriv);
	struct rrm_obj *prm;
	/* ToDo CONFIG_RTW_MLD: [currently primary link only] */
	struct _ADAPTER_LINK *padapter_link = GET_PRIMARY_LINK(padapter);


	if (prmpriv->prm_sel == NULL) {
		sprintf(pstr(s), "\nErr: No inActivate measurement\n");
		return;
	}
	prm = (struct rrm_obj *)prmpriv->prm_sel;

	/* measure current channel */
	prm->q.ch_num = padapter_link->mlmeextpriv.chandef.chan;
	prm->q.band = padapter_link->mlmeextpriv.chandef.band;
	prm->q.op_class = rtw_get_op_class_by_bchbw(prm->q.band,
		prm->q.ch_num, CHANNEL_WIDTH_20, CHAN_OFFSET_NO_EXT);

	rtw_fsm_activate_obj(prm);

#if 0
	sprintf(pstr(s), "\nActivate cid=%x, state=%s, meas_type=%s\n",
		prm->cid, rm_state_name(prm->pstate),
		rm_type_req_name(prm->q.m_type));
#endif

	sprintf(pstr(s), "aid=%d, mac=" MAC_FMT "\n",
		obj2sta(prm)->phl_sta->aid, MAC_ARG(obj2mac(prm)));

	/* clearn inActivate prm info */
	prmpriv->prm_sel = NULL;
}

/* for ioctl */
int rm_send_bcn_reqs(_adapter *padapter, u8 *sta_addr, u8 op_class, u8 ch,
	u16 measure_duration, u8 measure_mode, u8 *bssid, u8 *ssid,
	u8 reporting_detail,
	u8 n_ap_ch_rpt, struct _RT_OPERATING_CLASS *rpt,
	u8 n_elem_id, u8 *elem_id_list)
{
	struct rrm_obj *prm;
	char *pact;
	struct sta_info *psta;
	struct _RT_OPERATING_CLASS *prpt;
	void *ptr;
	int i,j,sz;
	u8 bcast[ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

	if (n_ap_ch_rpt > BCN_REQ_OPT_AP_CH_RPT_MAX_NUM) {
		RTW_ERR("rrm: chset num %d > %d\n",
			n_ap_ch_rpt, BCN_REQ_OPT_AP_CH_RPT_MAX_NUM);
		return -1;
	}
	/* dest sta */
	psta = rtw_get_stainfo(&padapter->stapriv, sta_addr);
        if (!psta) {
		RTW_ERR("rrm: psta not found\n");
		return -2;
        }
	prm = rtw_rrm_new_obj(padapter, psta, 0);
	if (prm == NULL) {
		RTW_ERR("rrm: unable to alloc rm obj for requeset\n");
		return -3;
	}

	prm->q.meas_dur = measure_duration;

	/* Figure 8-104 Measurement Requested format */
	prm->q.category = RTW_WLAN_CATEGORY_RADIO_MEAS;
	prm->q.action_code = RM_ACT_RADIO_MEAS_REQ;
	prm->q.m_mode = measure_mode; /* TODO */
	prm->q.m_type = bcn_req;
	prm->q.m_token = rrm_gen_meas_token(padapter);

	prm->q.e_id = _MEAS_REQ_IE_; /* 38 */
	prm->q.ch_num = ch;
	prm->q.op_class = op_class;
	prm->q.band = rm_get_band_by_opc(padapter, op_class);
	if (prm->q.band >= BAND_MAX) {
		FSM_WARN(prm, "%s: unknow opc:%d\n", __func__, op_class);
		return -4;
	}
	prm->from_ioctl = true;

	if (bssid != NULL)
		memcpy(prm->q.bssid, bssid, ETH_ALEN);
	else
		memcpy(prm->q.bssid, bcast, ETH_ALEN);

	if (ssid != NULL) {
		i = MIN(32, strlen(ssid));
		prm->q.opt.bcn.ssid.SsidLength = i;
		memcpy(prm->q.opt.bcn.ssid.Ssid, ssid, i);
	}

	if (n_ap_ch_rpt > 0) {
		prm->q.opt.bcn.ap_ch_rpt_num = n_ap_ch_rpt;
		j = 0;
		for (i = 0; i < n_ap_ch_rpt; i++) {
			prpt = rpt++;
			if (prpt == NULL)
				break;

			sz = sizeof(struct _RT_OPERATING_CLASS) * prpt->Len;
			ptr = rtw_malloc(sz);
			_rtw_memcpy(ptr, prpt, sz);
			prm->q.opt.bcn.ap_ch_rpt[i] = (struct _RT_OPERATING_CLASS *)ptr;
		}
	}
	prm->q.opt.bcn.rep_detail = reporting_detail;

	if ((n_elem_id > 0) && (n_elem_id < BCN_REQ_REQ_OPT_MAX_NUM)) {
		prm->q.opt.bcn.req_id_num = n_elem_id;
		_rtw_memcpy(prm->q.opt.bcn.req_id, elem_id_list, n_elem_id);
	}

	/* enquee rmobj */
	rtw_fsm_activate_obj(prm);

	FSM_INFO(prm, "\nAdd meas_type=%s ok\n",
		rm_type_req_name(prm->q.m_type));

	FSM_INFO(prm, "mac="MAC_FMT"\n", MAC_ARG(obj2mac(prm)));
	return 0;
}

static void rm_dbg_add_meas(_adapter *padapter, char *s)
{
#if 0
	struct rrm_priv *prmpriv = &(padapter->fsmpriv.rmpriv);
	struct rrm_obj *prm;
	char *pact;


	/* example :
	* rrm add_meas <aid=1|mac=>,m=<nb|clm|nhm|link>
	* rrm run_meas <aid=1|evid=>
	*/
	prm = (struct rrm_obj *)prmpriv->prm_sel;
	if (prm == NULL) {
		prm = rtw_rrm_new_obj(padapter, psta);
	}

	if (prm == NULL) {
		sprintf(pstr(s), "\nErr: alloc meas fail\n");
		return;
	}

	prm->obj = obj;
        prmpriv->prm_sel = prm;

	pact = strstr(s, "act");
	if (rm_dbg_modify_meas(padapter, s) == _FAIL) {

		sprintf(pstr(s), "\nErr: add meas fail\n");
		rtw_fsm_destory_obj(prm->obj);
		prmpriv->prm_sel = NULL;
		return;
	}
	prm->q.category = RTW_WLAN_CATEGORY_RADIO_MEAS;
	prm->q.e_id = _MEAS_REQ_IE_; /* 38 */

	sprintf(pstr(s), "\nAdd cid=%x, meas_type=%s ok\n",
		prm->cid, rm_type_req_name(prm->q.m_type));

	//if (prm->psta)
	sprintf(pstr(s), "mac="MAC_FMT"\n",
		MAC_ARG(obj2sta(prm)->phl_sta->mac_addr));

	if (pact)
#endif
		rm_dbg_activate_meas(padapter, pstr(s));
}

static void rm_dbg_del_meas(_adapter *padapter, char *s)
{
	struct rrm_priv *prmpriv = &padapter->fsmpriv.rmpriv;
	struct rrm_obj *prm = (struct rrm_obj *)prmpriv->prm_sel;


	if (prm) {
		sprintf(pstr(s), "\ndelete cid=%x\n",prm->cid);

		/* free inActivate meas - enqueue yet  */
		prmpriv->prm_sel = NULL;
		rtw_mfree(prmpriv->prm_sel, sizeof(struct rrm_obj));
	} else
		sprintf(pstr(s), "Err: no inActivate measurement\n");
}

static void rm_dbg_run_meas(_adapter *padapter, char *s)
{
	struct rrm_priv *prmpriv = &padapter->fsmpriv.rmpriv;
	struct rrm_obj *prm;
	char *pevid, *pcid;
	u32 evid;
	u16 cid;

	pcid = strstr(s, "cid="); /* hex */
	pevid = strstr(s, "evid="); /* dec */

	if (pcid && pevid) {
		pcid += 4; /* cid= */
		sscanf(pcid, "%hx", &cid);

		pevid += 5; /* evid= */
		sscanf(pevid, "%u", &evid);
	} else {
		sprintf(pstr(s), "\nErr: incorrect attribute\n");
		return;
	}

	prm = rtw_fsm_get_obj(prmpriv->fsm, cid);
	if (!prm) {
		sprintf(pstr(s), "\nErr: measurement not found\n");
		return;
	}

	if (evid >= RRM_EV_max) {
		sprintf(pstr(s), "\nErr: wrong event id\n");
		return;
	}

	rtw_fsm_gen_msg(prm, NULL, 0, evid);
	sprintf(pstr(s), "\npost %s to cid=%x\n", rtw_fsm_evt_name(prm, evid), cid);
}

static void rm_dbg_show_meas(struct rrm_obj *prm, char *s)
{
	struct sta_info *psta = obj2sta(prm);

	if (prm->q.action_code == RM_ACT_RADIO_MEAS_REQ) {

		sprintf(pstr(s), "\ncid=%x, meas_type=%s\n",
			prm->cid, rm_type_req_name(prm->q.m_type));

	} else  if (prm->q.action_code == RM_ACT_NB_REP_REQ) {

		sprintf(pstr(s), "\ncid=%x, action=neighbor_req\n",
			prm->cid);
	} else
		sprintf(pstr(s), "\ncid=%x, action=unknown\n",
			prm->cid);

	if (psta)
		sprintf(pstr(s), "aid=%d, mac="MAC_FMT"\n",
			psta->phl_sta->aid, MAC_ARG(obj2mac(prm)));

#if 0
	sprintf(pstr(s), "clock=%d, state=%s, rpt=%u/%u\n",
		(int)ATOMIC_READ(&prm->pclock->counter),
		rm_state_name(prm->pstate), prm->p.rpt, prm->q.rpt);
#endif
}

static void rm_dbg_list_meas(_adapter *padapter, char *s)
{
	int meas_amount;
	struct rrm_obj *prm;
	struct sta_info *psta;
	struct rrm_priv *prmpriv = &padapter->fsmpriv.rmpriv;
	_queue *queue = &prmpriv->rm_queue;
	_list *plist, *phead;
	unsigned long sp_flags;

	sprintf(pstr(s), "\n");
	_rtw_spinlock_irq(&queue->lock, &sp_flags);
	phead = get_list_head(queue);
	plist = get_next(phead);
	meas_amount = 0;

	while ((rtw_end_of_queue_search(phead, plist)) == _FALSE) {
		prm = LIST_CONTAINOR(plist, struct rrm_obj, list);
		meas_amount++;
		plist = get_next(plist);
		psta = obj2sta(prm);
		sprintf(pstr(s), "=========================================\n");

		rm_dbg_show_meas(prm, s);
	}
	_rtw_spinunlock_irq(&queue->lock, &sp_flags);

	sprintf(pstr(s), "=========================================\n");

	if (meas_amount==0) {
		sprintf(pstr(s), "No Activate measurement\n");
		sprintf(pstr(s), "=========================================\n");
	}

	if (prmpriv->prm_sel == NULL)
		sprintf(pstr(s), "\nNo inActivate measurement\n");
	else {
		sprintf(pstr(s), "\ninActivate measurement\n");
		rm_dbg_show_meas((struct rrm_obj *)prmpriv->prm_sel, s);
	}
}
#endif /* RM_SUPPORT_IWPRIV_DBG */

int verify_bcn_req(_adapter *padapter, struct sta_info *psta)
{
	char *bssid =  NULL;
	char ssid[] = "RealKungFu";
	u8 op_class = 0;
	u8 ch = 255;
	u16 measure_duration = 100;
	u8 reporting_detaial = 0;
	u8 n_ap_ch_rpt = 6;
	u8 measure_mode = bcn_req_active;
	u8 req[] = {1,2,3};
	u8 req_len = sizeof(req);

	/* TODO: use system OP class */
	static RT_OPERATING_CLASS US[] = {
	/* 0, OP_CLASS_NULL */	//{  0,  0, {}},
	/* 1, OP_CLASS_1 */	{115,  4, {36, 40, 44, 48}},
	/* 2, OP_CLASS_2 */	{118,  4, {52, 56, 60, 64}},
	/* 3, OP_CLASS_3 */	{124,  4, {149, 153, 157, 161}},
	/* 4, OP_CLASS_4 */	{121, 11, {100, 104, 108, 112, 116, 120, 124,
						128, 132, 136, 140}},
	/* 5, OP_CLASS_5 */	{125,  5, {149, 153, 157, 161, 165}},
	/* 6, OP_CLASS_12 */	{ 81, 11, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}}
	};

	rm_send_bcn_reqs(padapter, psta->phl_sta->mac_addr, op_class, ch,
		measure_duration, measure_mode, bssid, ssid,
		reporting_detaial, n_ap_ch_rpt, US, req_len, req);
	return 0;
}

/*
 * RM state machine
 */
static int rrm_idle_st_hdl(void *obj, u16 event, void *param)
{
	struct rrm_obj *prm = (struct rrm_obj *)obj;
	_adapter *padapter = obj2adp(prm);
	struct _ADAPTER_LINK *padapter_link = obj2sta(prm)->padapter_link;
	u8 val8;
	u32 val32;

	prm->p.category = RTW_WLAN_CATEGORY_RADIO_MEAS;

	switch (event) {
	case FSM_EV_STATE_IN:

		rrm_enqueue_rm(obj2priv(prm) ,prm, 0);

		prm->p.diag_token = prm->q.diag_token;
		switch (prm->q.action_code) {
		case RM_ACT_RADIO_MEAS_REQ:
			/* copy attrib from meas_req to meas_rep */
			prm->p.action_code = RM_ACT_RADIO_MEAS_REP;
			prm->p.e_id = _MEAS_RSP_IE_;
			prm->p.m_token = prm->q.m_token;
			prm->p.m_type = prm->q.m_type;
			prm->p.rpt = prm->q.rpt;
			prm->p.ch_num = prm->q.ch_num;
			prm->p.op_class = prm->q.op_class;
			prm->p.band = prm->q.band;

			if (prm->q.m_type == ch_load_req
				|| prm->q.m_type == noise_histo_req) {
				/*
				 * phydm measure current ch periodically
				 * scan current ch is not necessary
				 */
				val8 = padapter_link->mlmeextpriv.chandef.chan;
				if (prm->q.ch_num == val8)
					prm->poll_mode = 1;
			}
			FSM_DBG(prm, "%s repeat=%u\n",
				rm_type_req_name(prm->q.m_type),
				prm->q.rpt);
			break;
		case RM_ACT_NB_REP_REQ:
			prm->p.action_code = RM_ACT_NB_REP_RESP;
			FSM_DBG(prm, "Neighbor request\n");
			break;
		case RM_ACT_LINK_MEAS_REQ:
			prm->p.action_code = RM_ACT_LINK_MEAS_REP;
			FSM_DBG(prm, "Link meas\n");
			break;
		default:
			prm->p.action_code = prm->q.action_code;
			rm_set_rep_mode(prm, MEAS_REP_MOD_INCAP);
			FSM_WARN(prm, "recv unknown action%d\n", prm->p.action_code);
			break;
		} /* switch() */
		if (prm->cid & RM_MASTER) {
			if (rrm_issue_meas_req(prm) == _SUCCESS)
				rtw_fsm_st_goto(prm, RRM_ST_WAIT_MEAS);
			else
				rtw_fsm_st_goto(prm, RRM_ST_END);
			return _SUCCESS;
		} else {
			rtw_fsm_st_goto(prm, RRM_ST_DO_MEAS);
			return _SUCCESS;
		}

#if 0
		/* TODO below are dead code */
		if (prm->p.m_mode) {
			rrm_issue_null_reply(prm);
			rtw_fsm_st_goto(prm, RRM_ST_END);
			return _SUCCESS;
		}
		if (prm->q.rand_intvl) {
		#if 0 /*GEORGIA_TODO_REDEFINE_IO*/
			/* get low tsf to generate random interval */
			val32 = rtw_read32(padapter, REG_TSFTR);
		#endif
			val32 = rtw_hal_get_ltsf(padapter);
			val32 = val32 % prm->q.rand_intvl;
			FSM_INFO(prm, "rand_intval=%d, rand=%d\n", (int)prm->q.rand_intvl,val32);
			rtw_fsm_set_alarm(prm, prm->q.rand_intvl, RRM_EV_delay_timer_expire);
			return _SUCCESS;
		}
#endif
		break;
	case RRM_EV_delay_timer_expire:
		rtw_fsm_st_goto(prm, RRM_ST_DO_MEAS);
		break;
	case FSM_EV_ABORT:
	case FSM_EV_DISCONNECT_START:
		rtw_fsm_st_goto(prm, RRM_ST_END);
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(prm);
		break;
	default:
		break;
	}
	return _SUCCESS;
}

/* we do the measuring */
static int rrm_do_meas_st_hdl(void *obj, u16 event, void *param)
{
	struct rrm_obj *prm = (struct rrm_obj *)obj;
	_adapter *padapter = obj2adp(prm);
	u8 val8;
	u64 val64;


	switch (event) {
	case FSM_EV_STATE_IN:
		if (prm->q.action_code == RM_ACT_RADIO_MEAS_REQ) {
			switch (prm->q.m_type) {
			case bcn_req:
				if (prm->q.s_mode == bcn_req_bcn_table) {
					FSM_INFO(prm, "Beacon table\n");
					rrm_get_chset(prm);
					rtw_fsm_st_goto(prm, RRM_ST_SEND_REPORT);
					return _SUCCESS;
				}
				break;
			case ch_load_req:
			case noise_histo_req:
				/* TODO DO not send event */
				if (prm->poll_mode)
					rtw_fsm_gen_msg(prm, NULL, 0, FSM_EV_SCAN_DONE);
				return _SUCCESS;
			default:
				rtw_fsm_st_goto(prm, RRM_ST_END);
				return _SUCCESS;
			}

			if (!rm_prepare_scan(prm)) {
#if 0
				prm->wait_busy = RM_BUSY_TRAFFIC_TIMES;
				FSM_DBG(prm, "wait busy traffic - %d\n", prm->wait_busy);
				rtw_fsm_set_alarm(prm, RM_WAIT_BUSY_TIMEOUT, RRM_EV_busy_timer_expire);
#endif
				rm_set_rep_mode(prm, MEAS_REP_MOD_REFUSE);
				rrm_issue_null_reply(prm);
				rtw_fsm_st_goto(prm, RRM_ST_END);
				return _SUCCESS;
			}
		} else if (prm->q.action_code == RM_ACT_LINK_MEAS_REQ) {
			; /* do nothing */
			rtw_fsm_st_goto(prm, RRM_ST_SEND_REPORT);
			return _SUCCESS;
		}
		fallthrough;
	case RRM_EV_retry_timer_expire:
		/* expired due to meas condition mismatch, meas again */
		fallthrough;
	case RRM_EV_start_meas:
		if (prm->q.action_code == RM_ACT_RADIO_MEAS_REQ) {
			/* resotre measurement start time */
			prm->meas_start_time =
				rtw_hal_get_tsftr_by_port(padapter, rtw_hal_get_port(padapter));

			switch (prm->q.m_type) {
			case bcn_req:
				val8 = 1; /* Enable free run counter */
				rtw_hal_set_hwreg(padapter,
					HW_VAR_FREECNT, &val8);
				rrm_sitesurvey(prm);
				break;
			case ch_load_req:
			case noise_histo_req:
				rrm_sitesurvey(prm);
				break;
			default:
				rtw_fsm_st_goto(prm, RRM_ST_END);
				return _SUCCESS;
				break;
			}
		}
		/* handle measurement timeout */
		rtw_fsm_set_alarm(prm, RM_MEAS_TIMEOUT, RRM_EV_meas_timer_expire);
		break;
	case FSM_EV_SCAN_DONE:
		if (prm->q.action_code == RM_ACT_RADIO_MEAS_REQ) {
			switch (prm->q.m_type) {
			case bcn_req:
				rtw_fsm_st_goto(prm, RRM_ST_SEND_REPORT);
				return _SUCCESS;
			case ch_load_req:
			case noise_histo_req:
				retrieve_radio_meas_result(prm);

				if (rm_radio_meas_report_cond(prm) == _SUCCESS)
					rtw_fsm_st_goto(prm, RRM_ST_SEND_REPORT);
				else
					rtw_fsm_set_alarm(prm, RM_COND_INTVL,
						RRM_EV_retry_timer_expire);
				break;
			default:
				rtw_fsm_st_goto(prm, RRM_ST_END);
				return _SUCCESS;
			}
		}
		break;
	case RRM_EV_meas_timer_expire:
		FSM_INFO(prm, "measurement timeount\n");
		rm_set_rep_mode(prm, MEAS_REP_MOD_REFUSE);
		rrm_issue_null_reply(prm);
		rtw_fsm_st_goto(prm, RRM_ST_END);
		break;
#if 0
	case RRM_EV_busy_timer_expire:
		if (!rm_prepare_scan(prm) && prm->wait_busy--) {
			FSM_DBG(prm, "wait busy - %d\n",prm->wait_busy);
			rtw_fsm_set_alarm(prm, RM_WAIT_BUSY_TIMEOUT,
				RRM_EV_busy_timer_expire);
			break;
		}
		else if (prm->wait_busy <= 0) {
			FSM_DBG(prm, "wait busy timeout\n");
			rm_set_rep_mode(prm, MEAS_REP_MOD_REFUSE);
			rrm_issue_null_reply(prm);
			rtw_fsm_st_goto(prm, RRM_ST_END);
			return _SUCCESS;
		}
		rtw_fsm_gen_msg(prm, NULL, 0, RRM_EV_start_meas);
		break;
#endif
	case RRM_EV_request_timer_expire:
		rm_set_rep_mode(prm, MEAS_REP_MOD_REFUSE);
		rrm_issue_null_reply(prm);
		rtw_fsm_st_goto(prm, RRM_ST_END);
		break;
	case FSM_EV_ABORT:
	case FSM_EV_DISCONNECT_START:
		rm_set_rep_mode(prm, MEAS_REP_MOD_REFUSE);
		rtw_fsm_st_goto(prm, RRM_ST_END);
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(prm);
		/* resotre measurement end time */
		prm->meas_end_time = rtw_hal_get_tsftr_by_port(padapter
								, rtw_hal_get_port(padapter));

		val8 = 0; /* Disable free run counter */
		rtw_hal_set_hwreg(padapter, HW_VAR_FREECNT, &val8);
		break;
	default:
		break;
	}

	return _SUCCESS;
}

static int rrm_wait_meas_st_hdl(void *obj, u16 event, void *param)
{
	struct rrm_obj *prm = (struct rrm_obj *)obj;
	_adapter *a = obj2adp(prm);

	switch (event) {
	case FSM_EV_STATE_IN:
		/* we create meas_req, waiting for peer report */
		rtw_fsm_set_alarm(prm, RM_REQ_TIMEOUT,
			RRM_EV_request_timer_expire);
		break;
	case RRM_EV_recv_rep:
		rtw_fsm_st_goto(prm, RRM_ST_RECV_REPORT);
		break;
	case RRM_EV_request_timer_expire:
		FSM_INFO(prm, "request timeout\n");
		fallthrough;
	case FSM_EV_ABORT:
	case FSM_EV_DISCONNECT_START:
		rtw_fsm_st_goto(prm, RRM_ST_END);
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(prm);
		if (prm->q.action_code == RM_ACT_NB_REP_REQ) /* always make it valid */
			a->fsmpriv.rmpriv.nb_rpt.last_update = rtw_get_current_time();
		break;
	default:
		break;
	}
	return _SUCCESS;
}

static int rrm_send_report_st_hdl(void *obj, u16 event, void *param)
{
	struct rrm_obj *prm = (struct rrm_obj *)obj;
	u8 val8;


	switch (event) {
	case FSM_EV_STATE_IN:
		/* we have to issue report */
		if (prm->q.action_code == RM_ACT_RADIO_MEAS_REQ) {
			switch (prm->q.m_type) {
			case bcn_req:
				rrm_issue_beacon_rep(prm);
				break;
			case ch_load_req:
			case noise_histo_req:
				rrm_issue_radio_meas_rep(prm);
				break;
			default:
				rtw_fsm_st_goto(prm, RRM_ST_END);
				return _SUCCESS;
			}

		} else if (prm->q.action_code == RM_ACT_LINK_MEAS_REQ) {
			rrm_issue_link_meas_rep(prm);
			rtw_fsm_st_goto(prm, RRM_ST_END);
			return _SUCCESS;

		} else {
			rtw_fsm_st_goto(prm, RRM_ST_END);
			return _SUCCESS;
		}

		/* check repeat */
		if (prm->p.rpt) {
			FSM_INFO(prm, "repeat=%u/%u\n", prm->p.rpt, prm->q.rpt);
			prm->p.rpt--;
			/*
			* we recv meas_req,
			* delay for a wihile and than meas again
			*/
			if (prm->poll_mode)
				rtw_fsm_set_alarm(prm, RM_REPT_POLL_INTVL,
					RRM_EV_repeat_delay_expire);
			else
				rtw_fsm_set_alarm(prm, RM_REPT_SCAN_INTVL,
					RRM_EV_repeat_delay_expire);
			return _SUCCESS;
		}
		/* we are done */
		rtw_fsm_st_goto(prm, RRM_ST_END);
		break;
	case RRM_EV_repeat_delay_expire:
		rtw_fsm_st_goto(prm, RRM_ST_DO_MEAS);
		break;
	case FSM_EV_ABORT:
	case FSM_EV_DISCONNECT_START:
		rtw_fsm_st_goto(prm, RRM_ST_END);
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(prm);
		break;
	default:
		break;
	}
	return _SUCCESS;
}

static int rrm_recv_report_st_hdl(void *obj, u16 event, void *param)
{
	struct rrm_obj *prm = (struct rrm_obj *)obj;
	u8 val8;


	switch (event) {
	case FSM_EV_STATE_IN:
		/* we issue meas_req, got peer's meas report */
		switch (prm->p.action_code) {
		case RM_ACT_RADIO_MEAS_REP:
			/* check refuse, incapable and repeat */
			val8 = prm->p.m_mode;
			if (val8) {
				FSM_INFO(prm, "peer reject (%s repeat=%d)\n",
					val8&MEAS_REP_MOD_INCAP?"INCAP":
					val8&MEAS_REP_MOD_REFUSE?"REFUSE":
					val8&MEAS_REP_MOD_LATE?"LATE":"",
					prm->p.rpt);
				rtw_fsm_st_goto(prm, RRM_ST_END);
				return _SUCCESS;
			}
			break;
		case RM_ACT_NB_REP_RESP:
			/* report to upper layer if needing */
			rtw_fsm_st_goto(prm, RRM_ST_END);
			return _SUCCESS;
		default:
			rtw_fsm_st_goto(prm, RRM_ST_END);
			return _SUCCESS;
		}
		/* check repeat */
		if (prm->p.rpt) {
			FSM_INFO(prm, "repeat=%u/%u\n", prm->p.rpt, prm->q.rpt);
			prm->p.rpt--;
			/* waitting more report */
			rtw_fsm_st_goto(prm, RRM_ST_WAIT_MEAS);
			break;
		}
		/* we are done */
		rtw_fsm_st_goto(prm, RRM_ST_END);
		break;
	case FSM_EV_ABORT:
	case FSM_EV_DISCONNECT_START:
		rtw_fsm_st_goto(prm, RRM_ST_END);
		break;
	case FSM_EV_STATE_OUT:
		rtw_fsm_cancel_alarm(prm);
		break;
	default:
		break;
	}
	return _SUCCESS;
}

static int rrm_end_st_hdl(void *obj, u16 event, void *param)
{
	struct rrm_obj *prm = (struct rrm_obj *)obj;
	_adapter *a = obj2adp(prm);
	struct rrm_priv *prmpriv = &a->fsmpriv.rmpriv;
	int i;

	switch (event) {
	case FSM_EV_STATE_IN:

		rrm_dequeue_rm(obj2priv(prm), prm);

		if (prm->q.opt.bcn.req_start)
			rtw_mfree(prm->q.opt.bcn.req_start, prm->q.opt.bcn.req_len);
		for (i = 0; i < prm->q.opt.bcn.ap_ch_rpt_num; i++)
			if (prm->q.opt.bcn.ap_ch_rpt[i])
				rtw_mfree(prm->q.opt.bcn.ap_ch_rpt[i],
					sizeof(struct _RT_OPERATING_CLASS));
		rtw_fsm_deactivate_obj(prm);
		break;

	case FSM_EV_ABORT:
	case FSM_EV_STATE_OUT:
	default:
		break;
	}
	return _SUCCESS;
}

static void rrm_dump_obj_cb(void *obj, char *p, int *sz)
{
	/* nothing to do for now */
}

static void rrm_dump_fsm_cb(void *fsm, char *p, int *sz)
{
	/* nothing to do for now */
}

static void rm_dbg_help(void *obj, char *p, int *sz)
{
	struct rrm_obj *prm = (struct rrm_obj *)obj;
	int len = *sz;

	snprintf(pstr(p), lstr(p, len),
		"usage:\n\t<%s> <wdog>,<pause|resume>\n",
		rtw_fsm_obj_name(prm));
	*sz = len;
}

static void rm_debug(void *obj, char input[][MAX_ARGV], u32 input_num,
	char *output, u32 *out_len)
{
	struct rrm_obj *prm = (struct rrm_obj *)obj;
	char *ptr = output;
	int len = *out_len;

	if (input_num <  2) {
		rm_dbg_help(prm, ptr, &len);
		goto done;
	}

	if (!strcmp(input[0], "wdog")) {
		if (!strcmp(input[1], "pause")) {

			snprintf(pstr(ptr), lstr(ptr, len),
				"\n%s: pause watchdog\n",
				rtw_fsm_obj_name(prm));

		} else if (!strcmp(input[1], "resume")) {

			/* wdog,resume */
			snprintf(pstr(ptr), lstr(ptr, len),
				"\n%s: resume watchdog\n",
				rtw_fsm_obj_name(prm));
		}
	} else
		rm_dbg_help(prm, ptr, &len);
done:
	*out_len = len;
}

static int rrm_init_priv_cb(void *priv)
{
	struct rrm_priv *prmpriv = (struct rrm_priv *)priv;

	/* RRM private queue */
	_rtw_init_queue(&(prmpriv->rm_queue));

	/* bit 0-7 */
	prmpriv->rm_en_cap_def[0] = 0
		| BIT(RM_LINK_MEAS_CAP_EN)
		| BIT(RM_NB_REP_CAP_EN)
		/*| BIT(RM_PARAL_MEAS_CAP_EN)*/
		| BIT(RM_REPEAT_MEAS_CAP_EN)
		| BIT(RM_BCN_PASSIVE_MEAS_CAP_EN)
		| BIT(RM_BCN_ACTIVE_MEAS_CAP_EN)
		| BIT(RM_BCN_TABLE_MEAS_CAP_EN)
		| BIT(RM_BCN_MEAS_REP_COND_CAP_EN);

	/* bit  8-15 */
	prmpriv->rm_en_cap_def[1] = 0
		/*| BIT(RM_FRAME_MEAS_CAP_EN - 8)*/
#ifdef CONFIG_RTW_ACS
		| BIT(RM_CH_LOAD_CAP_EN - 8)
		| BIT(RM_NOISE_HISTO_CAP_EN - 8)
#endif
		/*| BIT(RM_STATIS_MEAS_CAP_EN - 8)*/
		/*| BIT(RM_LCI_MEAS_CAP_EN - 8)*/
		/*| BIT(RM_LCI_AMIMUTH_CAP_EN - 8)*/
		/*| BIT(RM_TRANS_STREAM_CAT_MEAS_CAP_EN - 8)*/
		/*| BIT(RM_TRIG_TRANS_STREAM_CAT_MEAS_CAP_EN - 8)*/;

	/* bit 16-23 */
	prmpriv->rm_en_cap_def[2] = 0
		/*| BIT(RM_AP_CH_REP_CAP_EN - 16)*/
		/*| BIT(RM_RM_MIB_CAP_EN - 16)*/
		/*| BIT(RM_OP_CH_MAX_MEAS_DUR0 - 16)*/
		/*| BIT(RM_OP_CH_MAX_MEAS_DUR1 - 16)*/
		/*| BIT(RM_OP_CH_MAX_MEAS_DUR2 - 16)*/
		/*| BIT(RM_NONOP_CH_MAX_MEAS_DUR0 - 16)*/
		/*| BIT(RM_NONOP_CH_MAX_MEAS_DUR1 - 16)*/
		/*| BIT(RM_NONOP_CH_MAX_MEAS_DUR2 - 16)*/;

	/* bit 24-31 */
	prmpriv->rm_en_cap_def[3] = 0
		/*| BIT(RM_MEAS_PILOT_CAP0 - 24)*/
		/*| BIT(RM_MEAS_PILOT_CAP1 - 24)*/
		/*| BIT(RM_MEAS_PILOT_CAP2 - 24)*/
		/*| BIT(RM_MEAS_PILOT_TRANS_INFO_CAP_EN - 24)*/
		/*| BIT(RM_NB_REP_TSF_OFFSET_CAP_EN - 24)*/
		| BIT(RM_RCPI_MEAS_CAP_EN - 24)
		| BIT(RM_RSNI_MEAS_CAP_EN - 24)
		/*| BIT(RM_BSS_AVG_ACCESS_DELAY_CAP_EN - 24)*/;

	/* bit 32-39 */
	prmpriv->rm_en_cap_def[4] = 0
		/*| BIT(RM_BSS_AVG_ACCESS_DELAY_CAP_EN - 32)*/
		/*| BIT(RM_AVALB_ADMIS_CAPACITY_CAP_EN - 32)*/
		/*| BIT(RM_ANT_CAP_EN - 32)*/;

	return _SUCCESS;
}

static int rrm_deinit_priv_cb(void *priv)
{
	struct rrm_priv *prmpriv = (struct rrm_priv *)priv;

	/* RRM private queue */
	_rtw_deinit_queue(&(prmpriv->rm_queue));

	return _SUCCESS;
}

u8 rm_add_nb_req(_adapter *a, struct sta_info *psta)
{
	struct rrm_obj *prm;

	prm = rtw_rrm_new_obj(a, psta, 0);

	if (prm == NULL) {
		RTW_ERR("rrm: unable to alloc rm obj for requeset\n");
		return _FALSE;
	}

	prm->q.category = RTW_WLAN_CATEGORY_RADIO_MEAS;
	prm->q.action_code = RM_ACT_NB_REP_REQ;

	rtw_fsm_activate_obj(prm);

	FSM_INFO(prm, "%s to " MAC_FMT "\n", __func__, MAC_ARG(obj2mac(prm)));

	return _SUCCESS;
}

static int rrm_is_nb_valid(_adapter *a)
{
	struct rrm_nb_rpt *nb_rpt = &a->fsmpriv.rmpriv.nb_rpt;

	if (nb_rpt->last_update == 0 ||
		rtw_get_passing_time_ms(nb_rpt->last_update) > RM_NB_REP_VALID_TIME)
		return _FAIL;

	return _SUCCESS;
}

int rtw_rrm_get_nb_rpt(_adapter *a, struct rrm_nb_rpt * nb_rpt)
{
	memcpy(nb_rpt, &a->fsmpriv.rmpriv.nb_rpt, sizeof(struct rrm_nb_rpt));

	if (rrm_is_nb_valid(a))
		return _SUCCESS; /* nb_rpt is valid */
	return _FAIL; /* nb_rpt is invalid */
}

/* For EXTERNAL application to create RRM FSM */
int rtw_rrm_reg_fsm(struct fsm_priv *fsmpriv)
{
	struct fsm_root *root = fsmpriv->root;
	struct rrm_priv *prmpriv = &fsmpriv->rmpriv;
	struct fsm_main *fsm = NULL;
	struct rtw_fsm_tb tb;

	memset(&tb, 0, sizeof(tb));
	tb.max_state = sizeof(rrm_state_tbl)/sizeof(rrm_state_tbl[0]);
	tb.max_event = sizeof(rrm_event_tbl)/sizeof(rrm_event_tbl[0]);
	tb.state_tbl = rrm_state_tbl;
	tb.evt_tbl = rrm_event_tbl;
	tb.priv = prmpriv;
	tb.init_priv = rrm_init_priv_cb;
	tb.deinit_priv = rrm_deinit_priv_cb;
	tb.dump_obj = rrm_dump_obj_cb;
	tb.dump_fsm = rrm_dump_fsm_cb;
	tb.dbg_level = FSM_LV_INFO;
	tb.evt_level = FSM_LV_INFO;
	tb.debug = rm_debug;

	fsm = rtw_fsm_register_fsm(root, "rrm", &tb);
	prmpriv->fsm = fsm;

	if (fsm)
		return _SUCCESS;
	return _FAIL;
}

/* parse neighbor report */
u32 rrm_parse_nb_list(struct rrm_nb_rpt *pnb_rpt, u8 *ie, u32 ie_len)
{
	int i;
	struct nb_rpt_hdr *pie;
	struct rrm_nb_list *pnb_list = pnb_rpt->nb_list;
	u8 *ptr, *pend, *op, preference;
	u32 elem_len, subelem_len, op_len;

	ptr = ie;
	pend = ie + ie_len;
	elem_len = ie_len;
	subelem_len = (u32)*(ie + 1);

	memset(pnb_rpt, 0, sizeof(struct rrm_nb_rpt));
	for (i = 0; i < RTW_MAX_NB_RPT_NUM; i++) {
		if (((ptr + 7) > pend) || (elem_len < subelem_len))
			break;

		if (*ptr != RTW_WLAN_ACTION_WNM_NB_RPT_ELEM) {
			RTW_INFO("WNM: end of data(0x%2x)!\n", *ptr);
			break;
		}

		pie = (struct nb_rpt_hdr *)ptr;
		_rtw_memcpy(&pnb_list[i].ent, pie, sizeof(struct nb_rpt_hdr));
#if 0 /* debug */
		/* TEST forse 1st ch to DFS 100 */
		if (i==0) {
			pnb_list[i].ent.ch_num = 100;
			pnb_list[i].ent.reg_class = 121;
		}
		/* TEST forse 2nd ch to DFS 104 */
		else if (i==1) {
			pnb_list[i].ent.ch_num = 104;
			pnb_list[i].ent.reg_class = 121;
		}
#endif
		op = rtw_get_ie((u8 *)(ptr + 15),
				WNM_BTM_CAND_PREF_SUBEID, &op_len, (subelem_len - 15));

		if (op && (op_len !=0)) {
			preference = *(op + 2);
			if (preference) /* 1-255 */
				pnb_list[i].preference = preference;
			else /* 0:exclude BSS */
				pnb_list[i].preference = -1; /* exclude BSS */
		} else {
			pnb_list[i].preference = 0; /* Does NOT specify preference */
		}
		ptr = (u8 *)(ptr + subelem_len + 2);
		elem_len -= (subelem_len + 2);
		subelem_len = *(ptr + 1);

		pnb_rpt->nb_list_num++;
	}

	return _SUCCESS;
}

static void _swap(struct rrm_nb_list *pnb_list, int idx1, int idx2)
{
	struct rrm_nb_list tmp;

	if (idx1 == idx2)
		return;

	memcpy(&tmp, &pnb_list[idx1], sizeof(struct rrm_nb_list));
	memcpy(&pnb_list[idx1], &pnb_list[idx2], sizeof(struct rrm_nb_list));
	memcpy(&pnb_list[idx2], &tmp, sizeof(struct rrm_nb_list));
}

/* sort nb list according to preference */
void rrm_sort_nb_list(struct rrm_nb_rpt *pnb_rpt)
{
	int i, j, p, idx1, idx2, ch;

	for (i = 0; i < pnb_rpt->nb_list_num; i++) {
		p = pnb_rpt->nb_list[i].preference;
		idx1 = idx2 = i;
		for (j = i; j < pnb_rpt->nb_list_num; j++) {
			if (pnb_rpt->nb_list[j].preference > p) {
				p = pnb_rpt->nb_list[j].preference; /* max */
				idx2 = j;
			}
		}
		_swap(pnb_rpt->nb_list, idx1, idx2);
	}
}

static void rrm_upd_nb_ch_list(struct rrm_obj *prm, struct rrm_nb_rpt *pnb_rpt)
{
	_adapter *a = obj2adp(prm);
	struct rtw_ieee80211_channel tmp_ch[RTW_CHANNEL_SCAN_AMOUNT] = {0};
	const struct op_class_t *opc;
	int i, j, tmp_ch_num = 0;
	u8 ch, band, op_class;

	for (i = 0; i < pnb_rpt->nb_list_num; i++) {
		ch = pnb_rpt->nb_list[i].ent.ch_num;
		op_class = pnb_rpt->nb_list[i].ent.reg_class;
		band = rm_get_band_by_opc(a, op_class);

		if (band >= BAND_MAX) {
			FSM_WARN(prm, "%s: skip unknown opc:%d\n", __func__, op_class);
			continue;
		}

		if (ch == 0) {
			/* get all channels in this op class */
			opc = get_opc_by_op_class(NULL, op_class);

			if (!opc)
				continue;

			for (j = 0; j < OPC_CH_LIST_LEN(opc); j++) {
				tmp_ch[tmp_ch_num].hw_value = OPC_CH_LIST_CH(opc, j);
				tmp_ch[tmp_ch_num].band = band;
				if (++tmp_ch_num == RTW_CHANNEL_SCAN_AMOUNT)
					goto full;
			}
		} else {
			tmp_ch[tmp_ch_num].hw_value = ch;
			tmp_ch[tmp_ch_num].band = band;
			if (++tmp_ch_num == RTW_CHANNEL_SCAN_AMOUNT)
				goto full;
		}
	}

full:
	/* remove redundant ch */
	for (i = 0; i < tmp_ch_num; i++) {
		if (tmp_ch[i].hw_value != 0) {
			for (j = i + 1; j < tmp_ch_num; j++) {
				if (tmp_ch[i].hw_value == tmp_ch[j].hw_value &&
					tmp_ch[i].band == tmp_ch[j].band) {
					tmp_ch[j].hw_value = 0; /* remove */
				}
			}
		}
	}

	/* copy final channels to pnb_rpt */
	for (i = 0; i < tmp_ch_num; i++) {
		if (tmp_ch[i].hw_value != 0) {
			pnb_rpt->ch_list[pnb_rpt->ch_list_num].hw_value = tmp_ch[i].hw_value;
			pnb_rpt->ch_list[pnb_rpt->ch_list_num].band = tmp_ch[i].band;
			pnb_rpt->ch_list_num++;
		}
	}
#if 0
	for (i = 0; i < pnb_rpt->ch_list_num; i++) {
		printk("%s(%d) %d/%d band = %d, ch = %d\n", __func__, __LINE__,
			i, pnb_rpt->ch_list_num,pnb_rpt->ch_list[i].band, pnb_rpt->ch_list[i].hw_value);
	}
#endif
}
#endif /* CONFIG_RTW_FSM_RRM */
