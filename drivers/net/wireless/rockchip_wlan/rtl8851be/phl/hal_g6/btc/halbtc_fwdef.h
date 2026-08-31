#ifndef __INC_BTC_FW_DEF_H__
#define __INC_BTC_FW_DEF_H__

#pragma pack(push)
#pragma pack(1)
/*
 * shared FW Definition
 */

//#define BTC_FW_STEP_DBG

#define CXMREG_MAX 20
#define FCXDEF_STEP 50			/* MUST fw_step size*/
#define BTC_CYCLE_SLOT_MAX 48		/* must be even number, non-zero */
#define BTC_WL_MAX_ROLE_NUMBER 6	/* Must = MAX_WIFI_ROLE_NUMBER */
#define BTC_DBG_MAX1 32

#define FCX_VER_FDDT 7		/* H2c refer to "struct btc_fddt_train_info" */
#define FCX_VER_TRX 7		/* H2c refer to "struct btc_trx_info" */
#define FCX_VER_INIT 7		/* H2C refer to "struct btc_init_info" */
#define FCX_VER_CTRL 7		/* H2C refer to "struct btc_ctrl" */
#define FCX_VER_ROLE 7		/* H2C refer to "struct btc_wl_role_info" */

#define FCX_VER_BTDEVINFO 7	/* C2H refer to "struct fbtc_btdevinfo" */
#define FCX_VER_BTAFH 7		/* C2H refer to "struct fbtc_btafh" */
#define FCX_VER_BTSCAN 7 	/* C2H refer to "struct fbtc_btscan" */
#define FCX_VER_BTVER 7		/* C2H refer to "struct fbtc_btver */
#define FCX_VER_NULLSTA 7 	/* C2H refer to "struct fbtc_cynullsta" */
#define FCX_VER_CYSTA 7 	/* C2H refer to "struct fbtc_cysta" */
#define FCX_VER_STEP 7		/* C2H refer to "struct fbtc_steps" */

#define FCX_VER_TDMA 7		/* H2C/C2H refer to "struct fbtc_tdma" */
#define FCX_VER_BTCRPT 8	/* H2C/C2H refer to "struct fbtc_rpt_ctrl" */
#define FCX_VER_SLOT 7		/* H2C/C2H refer to "struct fbtc_slots" */
#define FCX_VER_MREG 7		/* H2C/C2H refer to "struct fbtc_mreg_val */
#define FCX_VER_GPIODBG 8 	/* H2C/C2H refer to "struct fbtc_gpio_dbg" */
#define FCX_VER_TEST 1		/* H2C/C2H refer to "struct fbtc_testinfo" */

enum btc_bt_sta_counter {
	BTC_BCNT_RFK_REQ = 0,
	BTC_BCNT_HI_TX = 1,
	BTC_BCNT_HI_RX = 2,
	BTC_BCNT_LO_TX = 3,
	BTC_BCNT_LO_RX = 4,
	BTC_BCNT_POLUT = 5,
	BTC_BCNT_STA_MAX
};

/* ============== TDMA related ============== */
enum fbtc_tdma_template {
	CXTD_OFF = 0x0,
	CXTD_OFF_B2,
	CXTD_OFF_EXT,
	CXTD_FIX,
	CXTD_PFIX,
	CXTD_AUTO,
	CXTD_PAUTO,
	CXTD_AUTO2,
	CXTD_PAUTO2,
	CXTD_MAX
};

enum fbtc_tdma_type {
	CXTDMA_OFF = 0x0, /* tdma off */
	CXTDMA_FIX = 0x1, /* fixed slot */
	CXTDMA_AUTO = 0x2, /* auto slot */
	CXTDMA_AUTO2 = 0x3, /* extended auto slot */
	CXTDMA_MAX
};

enum fbtc_tdma_rx_flow_ctrl {
	CXFLC_OFF = 0x0,  /* rx flow off */
	CXFLC_NULLP = 0x1, /* Null/Null-P */
	CXFLC_QOSNULL = 0x2, /* QOS Null/Null-P */
	CXFLC_CTS = 0x3,  /* CTS to Self control */
	CXFLC_MAX
};

enum fbtc_tdma_wlan_tx_pause {
	CXTPS_OFF = 0x0,  /* no wl tx pause*/
	CXTPS_ON = 0x1,
	CXTPS_MAX
};

/* define if ext-ctrl-slot allowed while TDMA off */
enum fbtc_ext_ctrl_type {
	CXECTL_OFF = 0x0, /* tdma off */
	CXECTL_B2 = 0x1, /* allow B2 (beacon-early) */
	CXECTL_EXT = 0x2,
	CXECTL_MAX
};

union fbtc_rxflct {
	u8 val;
	u8 type: 3;
	u8 tgln_n: 5;
};

enum fbtc_option_ctrl_def {
	CXOPCTL_INST_EXEC = BIT(0), /* tdma instant execute */
	CXOPCTL_FDDT_ENABLE = BIT(1),/* info wl fw to enable fdd-train*/
	CXOPCTL_FDDT_RENEW = BIT(2), /* info wl fw to update fddt_info.train */
	CXOPCTL_FDDT_DEBUG = BIT(3), /* info wl fw to enter debug mode */
	CXOPCTL_LE_INIT_EXEC = BIT(4) /*le_interval < 100ms*/
};

 /* ============== SLOT related ============== */
 enum btc_slot_type { /* slot */
	CXST_OFF = 0x0,
	CXST_B2W = 0x1,
	CXST_W1 = 0x2,
	CXST_W2 = 0x3,
	CXST_W2B = 0x4,
	CXST_B1 = 0x5,
	CXST_B2 = 0x6,
	CXST_B3 = 0x7,
	CXST_B4 = 0x8,
	CXST_LK = 0x9,
	CXST_BLK = 0xa,
	CXST_E2G = 0xb, /* for ext-control-slot 2G*/
	CXST_E5G = 0xc, /* for ext-control-slot 5G*/
	CXST_EBT = 0xd, /* for ext-control-slot BT*/
	CXST_ENULL = 0xe, /* for ext-control-slot Null*/
	CXST_WLK = 0xf, /* for WL link slot */
	CXST_W1FDD = 0x10,
	CXST_B1FDD = 0x11,
	CXST_MAX = 0x12 /* The max slot must be even*/
};

enum btc_evnt_type {
	CXEVNT_TDMA_ENTRY = 0x0,
	CXEVNT_WL_TMR,
	CXEVNT_B1_TMR,
	CXEVNT_B2_TMR,
	CXEVNT_B3_TMR,
	CXEVNT_B4_TMR,
	CXEVNT_W2B_TMR,
	CXEVNT_B2W_TMR,
	CXEVNT_BCN_EARLY,
	CXEVNT_A2DP_EMPTY,
	CXEVNT_LK_END,
	CXEVNT_RX_ISR,
	CXEVNT_RX_FC0,
	CXEVNT_RX_FC1,
	CXEVNT_BT_RELINK,
	CXEVNT_BT_RETRY,
	CXEVNT_E2G,
	CXEVNT_E5G,
	CXEVNT_EBT,
	CXEVNT_ENULL,
	CXEVNT_DRV_WLK,
	CXEVNT_BCN_OK,
	CXEVNT_BT_CHANGE,
	CXEVNT_EBT_EXTEND,
	CXEVNT_E2G_NULL1,
	CXEVNT_B1FDD_TMR,
	CXEVNT_LE_INIT_END,
	CXEVNT_MAX
};

enum btc_bcn_type {
	CXBCN_ALL = 0x0,
	CXBCN_ALL_OK,
	CXBCN_BT_SLOT,
	CXBCN_BT_OK,
	CXBCN_MAX
};

enum btc_slot_iso_type {
	SLOT_MIX = 0x0, /* accept BT Lower-Pri Tx/Rx request 0x778 = 1 */
	SLOT_ISO = 0x1, /* no accept BT Lower-Pri Tx/Rx request 0x778 = d*/
	CXSTYPE_MAX
};

enum btc_wbslot_type { /* TIME */
	CXT_BT = 0x0,
	CXT_WL = 0x1,
	CXT_MAX
};

enum btc_flctrl_type { /* TIME-A2DP */
	CXT_FLCTRL_OFF = 0x0,
	CXT_FLCTRL_ON = 0x1,
	CXT_FLCTRL_MAX
};

enum btc_step_type { /* STEP TYPE */
	CXSTEP_NONE = 0x0,
	CXSTEP_EVNT = 0x1,
	CXSTEP_SLOT = 0x2,
	CXSTEP_MAX
};

enum btc_null_state {
	CXNULL_0 = 0,
	CXNULL_1 = 1,
	CXNULL_STATE_MAX = 2
};

enum btc_null_staus {
	CXNULL_FAIL = 0,
	CXNULL_OK = 1,
	CXNULL_LATE = 2,
	CXNULL_RETRY = 3,
	CXNULL_TX = 4,
	CXNULL_MAX = 5
};

enum btc_ble_scan_type {
	CXSCAN_BG = 0,
	CXSCAN_INIT,
	CXSCAN_LE,
	CXSCAN_MAX1
};

enum btc_bt_afh_type {
	BTC_RPT_BT_AFH_LEGACY = 0x10,
	BTC_RPT_BT_AFH_LE = 0x20
};

/* ========== H2C/C2H refer to "struct fbtc_tdma" ========== */
struct fbtc_tdma {
	u8 type; /* refer to fbtc_tdma_type*/
	u8 rxflctrl; /* refer to fbtc_tdma_rx_flow_ctrl */
	u8 txflctrl; /* If WL stop Tx while enter BT-slot */
	u8 rsvd;

	u8 leak_n; /* every leak_n cycle do leak detection */
	u8 ext_ctrl; /* refer to fbtc_ext_ctrl_type*/
	/* send rxflctrl to which role
	 * enum role_type, default: 0 for single-role
	 * if multi-role: [7:4] second-role, [3:0] fisrt-role
	 */
	u8 rxflctrl_role;
	u8 option_ctrl; /*bit0: execute immediately, no tdma cycle waitting */
};

struct fbtc_1tdma {
	u8 fver;
	u8 rsvd1;
	u8 rsvd2;
	u8 rsvd3;

	struct fbtc_tdma tdma;
};
/* ==========================================================================*/

/* ========== H2C refer to "struct btc_wl_role_info" ========== */
struct btc_wl_active_role { /* struct size must be n*4 bytes */
	u8 connected;
	u8 pid;
	u8 phy;
	u8 noa;

	u8 band; /* enum band_type RF band: 2.4G/5G/6G */
	u8 client_ps;
	u8 bw; /* enum channel_width */
	u8 role; /*enum role_type */

	u8 ch;
	u8 noa_dur; /* ms */
	u8 client_cnt;
	u8 rsvd2;
};

struct btc_wl_role_info { /* struct size must be n*4 bytes */
	u8 connect_cnt;
	u8 link_mode;
	u8 link_mode_chg;
	u8 p2p_2g;

	struct btc_wl_active_role active_role[BTC_WL_MAX_ROLE_NUMBER];

	u32 role_map;
	u32 mrole_type; /* btc_wl_mrole_type */
	u32 mrole_noa_duration; /* ms */
	u32 dbcc_en;
	u32 dbcc_chg;
	u32 dbcc_2g_phy; /* which phy operate in 2G, HW_PHY_0 or HW_PHY_1 */
};
/* ==========================================================================*/

/* ========== H2C refer to "struct btc_ctrl" ========== */
struct btc_ctrl {
	u8 manual;
	u8 igno_bt;
	u8 always_freerun;
	u8 rsvd;
};
/* ==========================================================================*/

/* ========== H2C/C2H refer to "struct fbtc_rpt_ctrl" ========== */
struct btc_rpt_ctrl_info {
	u16 cnt; /* fw report counter */
	u16 cnt_c2h; /* fw send c2h counter  */

	u16 cnt_h2c; /* fw recv h2c counter */
	u16 len_c2h; /* The total length of the last C2H  */

	u16 cnt_aoac_rf_on;  /* rf-on counter for aoac switch notify */
	u16 cnt_aoac_rf_off; /* rf-off counter for aoac switch notify */

	u32 cx_ver; /* match which driver's coex version */
	u32 fw_ver;
	u32 en; /* report map */
};

struct btc_rpt_ctrl_a2dp_empty {
	u32 cnt_empty; /* a2dp empty count */
	u32 cnt_flowctrl; /* a2dp empty flow control counter */
	u32 cnt_tx;
	u32 cnt_ack;
	u32 cnt_nack;
};

struct btc_rpt_ctrl_bt_mailbox {
	u32 cnt_send_ok; /* fw send mailbox ok counter */
	u32 cnt_send_fail; /* fw send mailbox fail counter */
	u32 cnt_recv; /* fw recv mailbox counter */
	struct btc_rpt_ctrl_a2dp_empty a2dp;
};

struct fbtc_rpt_ctrl {
	u8 fver;
	u8 ext_req_exist;
	u8 pta_owner;
	u8 rsvd2;

	u8 gnt_val[HW_PHY_MAX][4];
	u16 bt_cnt[BTC_BCNT_STA_MAX];

	struct btc_rpt_ctrl_info rpt_info;
	struct btc_rpt_ctrl_bt_mailbox bt_mbx_info;
};
/* ==========================================================================*/

/* ========== H2C refer to "struct btc_init_info" ========== */
struct btc_ant_info {
	u8 type;  /* shared, dedicated(non-shared) */
	u8 num;   /* antenna count  */
	u8 isolation;
	u8 single_pos;/* wifi 1ss-1ant at 0:S0 or 1:S1 */

	u8 diversity; /* only for wifi use 1-antenna */
	u8 btg_pos; /* btg-circuit at 0:S0/1:S1/others:all */
	u8 stream_cnt;  /* spatial_stream count: Tx[7:4], Rx[3:0] */
	u8 path_pos;    /* path pos: Tx[7:4], Rx[3:0] */
};

struct btc_module {
	u8 rfe_type;
	u8 kt_ver;
	u8 bt_solo;
	u8 bt_pos; /* wl-end view: get from efuse, must compare bt.btg_type*/

	u8 switch_type; /* WL/BT switch type: 0: internal, 1: external */
	u8 wa_type; /* WA type: 0:none, 1: 51B 5G_Hi-Ch_Rx */
	u8 kt_ver_adie;
	u8 rsvd;

	struct btc_ant_info ant;
};

struct btc_init_info {
	u8 wl_guard_ch;
	u8 wl_only;
	u8 wl_init_ok;
	u8 dbcc_en;

	u8 cx_other;
	u8 bt_only;
	u8 rsvd1;
	u8 rsvd2;

	struct btc_module module;
};
/* ==========================================================================*/

/* ========== H2C/C2H refer to "struct fbtc_gpio_dbg" ========== */
struct fbtc_gpio_dbg {
	u8 fver;
	u8 rsvd0;
	u8 rsvd1;
	u8 rsvd2;

	u8 gpio_map[BTC_DBG_MAX1]; /* debug signals to GPIO-Position mapping */

	u32 en_map; /* which debug signal (see btc_wl_gpio_debug) is enable */
	u32 pre_state; /* the debug signal is 1 or 0  */
};
/* ==========================================================================*/

/* ========== H2C/C2H refer to "struct fbtc_mreg_val" ========== */
struct fbtc_mreg_val {
	u8 fver;
	u8 reg_num;
	u8 rsvd0;
	u8 rsvd1;

	u32 mreg_val[CXMREG_MAX];
};

struct fbtc_mreg {
	u16 type;
	u16 bytes;
	u32 offset;
};
/* ==========================================================================*/

/* ========== H2C/C2H refer to "struct fbtc_slots" ========== */
struct fbtc_slot {
	u16 dur; /* slot duration */
	u16 cxtype;
	u32 cxtbl;
};

struct fbtc_slot_u16 {
	u16 dur; /* slot duration */
	u16 cxtype;
	u16 cxtbl_l16; /* coex table [15:0] */
	u16 cxtbl_h16; /* coex table [31:16] */
};

struct fbtc_1slot {
	u8 fver;
	u8 sid; /* slot id */
	u16 rsvd;
	struct fbtc_slot slot;
};

struct fbtc_slots {
	u8 fver;
	u8 slot_cnt;
	u8 rsvd0;
	u8 rsvd1;

	struct fbtc_slot_u16 slot[CXST_MAX];

	u32 update_map;
};
/* ==========================================================================*/

/* ========== C2H refer to "struct fbtc_steps" ========== */
struct fbtc_step {
	u8 type;
	u8 val;
	u8 difft_l8;
	u8 difft_h8;
};

struct fbtc_steps {
	u8 fver;
	u8 en;
	u8 rsvd0;
	u8 rsvd1;

#ifdef BTC_FW_STEP_DBG
	struct fbtc_step step[FCXDEF_STEP];
#endif

	u32 cnt;
};
/* ==========================================================================*/

/* ========== C2H refer to "struct fbtc_cysta" ========== */
struct fbtc_sta_time_info {
	u16 tavg[CXT_MAX]; /* avg wl/bt cycle time */
	u16 tmax[CXT_MAX]; /* max wl/bt cycle time */
};

struct fbtc_cycle_a2dptrx_info {
	u8 empty_cnt;
	u8 retry_cnt;
	u8 tx_rate;
	u8 tx_cnt;

	u8 ack_cnt;
	u8 nack_cnt;
	u8 no_empty_cnt;
	u8 rsvd;
};

struct fbtc_sta_a2dpept_info {
	u16 cnt; /* a2dp empty cnt */
	u16 cnt_timeout; /* a2dp empty timeout cnt*/
	u16 tavg; /* avg a2dp empty time */
	u16 tmax; /* max a2dp empty time */
};

struct fbtc_sta_leak_info {
	u16 tavg; /* avg leak-slot time */
	u16 tamx; /* max leak-slot time */
	u32 cnt_rximr; /* the rximr occur at leak slot  */
};

struct fbtc_cysta { /* statistics for cycles */
	u8 fver;
	u8 rsvd;
	u8 collision_cnt; /* counter for event/timer occur at the same time */
	u8 except_cnt;

	u8 wl_rx_err_ratio[BTC_CYCLE_SLOT_MAX];

	struct fbtc_cycle_a2dptrx_info a2dp_trx[BTC_CYCLE_SLOT_MAX];

	u16 skip_cnt;
	u16 cycles; /* total cycle number */

	u16 slot_step_time[BTC_CYCLE_SLOT_MAX]; /* record the wl/bt slot time */
	u16 slot_cnt[CXST_MAX]; /* slot count */
	u16 bcn_cnt[CXBCN_MAX];

	struct fbtc_sta_time_info cycle_time;
	struct fbtc_sta_a2dpept_info a2dp_ept;
	struct fbtc_sta_leak_info leak_slot;

	u32 except_map;
};
/* ==========================================================================*/

/* ========== C2H refer to "struct fbtc_cynullsta" ========== */
struct fbtc_cynullsta { /* cycle null statistics */
	u8 fver;
	u8 rsvd0;
	u8 rsvd1;
	u8 rsvd2;

	u32 tmax[CXNULL_STATE_MAX]; /* max_t for 0:null0/1:null1 */
	u32 tavg[CXNULL_STATE_MAX]; /* avg_t for 0:null0/1:null1 */
	/* result for null , 0:fail, 1:ok, 2:late, 3:retry */
	u32 result[CXNULL_STATE_MAX][CXNULL_MAX];
};
/* ==========================================================================*/

/* ========== C2H refer to "struct fbtc_btver" ========== */
struct fbtc_btver {
	u8 fver;
	u8 rsvd0;
	u8 rsvd1;
	u8 rsvd2;

	u32 coex_ver; /*bit[15:8]->shared, bit[7:0]->non-shared */
	u32 fw_ver;
	u32 feature;
};
/* ==========================================================================*/

/* ========== C2H refer to "struct fbtc_btscan" ========== */
struct btc_bt_scan_info {
	u16 win;
	u16 intvl;
};

struct fbtc_btscan {
	u8 fver;
	u8 type;
	u8 rsvd0;
	u8 rsvd1;

	struct btc_bt_scan_info para[CXSCAN_MAX1];
};
/* ==========================================================================*/

/* ========== C2H refer to "struct fbtc_btafh" ========== */
struct fbtc_btafh {
	u8 fver;
	u8 map_type;
	u8 rsvd0;
	u8 rsvd1;

	u8 afh_l[4]; /*bit0:2402, bit1: 2403.... bit31:2433 */
	u8 afh_m[4]; /*bit0:2434, bit1: 2435.... bit31:2465 */
	u8 afh_h[4]; /*bit0:2466, bit1:2467......bit14:2480 */

	u8 afh_le_a[4];
	u8 afh_le_b[4];
};
/* ==========================================================================*/

/* ========== H2c refer to "struct btc_trx_info" ========== */
struct btc_trx_info {
	u8 tx_lvl;
	u8 rx_lvl;
	u8 wl_rssi;
	u8 bt_rssi;

	s8 tx_power; /* absolute Tx power (dBm), 0xff-> no BTC control */
	s8 rx_gain;  /* rx gain table index (TBD.) */
	s8 bt_tx_power; /* decrease Tx power (dB) */
	s8 bt_rx_gain;  /* LNA constrain level */

	u8 cn; /* condition_num */
	s8 nhm;
	u8 bt_profile;
	u8 rsvd2;

	u16 tx_rate;
	u16 rx_rate;

	u32 tx_tp;
	u32 rx_tp;
	u32 rx_err_ratio;
};
/* ==========================================================================*/

/* ========== C2H refer to "struct fbtc_btdevinfo" ========== */
struct fbtc_btdevinfo {
	u8 fver;
	u8 rsvd0;
	u16 vendor_id;

	u32 dev_name; /* only 24 bits valid */
	u32 flush_time;
};
/* ==========================================================================*/

/* ========== C2H refer to "struct fbtc_testinfo" ========== */
struct fbtc_testinfo {
	u8 fver;
	u8 rsvd0;
	u8 rsvd1;
	u8 rsvd2;
	u8 rsvd3;
	u8 rsvd4;
	u8 rsvd5;
	u8 rsvd6;
};
/* ==========================================================================*/

/* ========== H2c refer to "struct btc_fddt_train_info" ========== */
struct fbtc_fddt_cycle_info {
	s8 tx_power; /* absolute Tx power (dBm), 0xff-> no BTC control */
	s8 bt_tx_power; /* decrease Tx power (dB) */
	s8 bt_rx_gain;  /* LNA constrain level */
	u8 no_empty_cnt;

	u8 rssi; /* [7:4] -> bt_rssi_level, [3:0]-> wl_rssi_level */
	u8 cn; /* condition_num */
	u8 train_status; /* [7:4]-> state, [3:0]-> phase */
	u8 train_result; /* response: break,pass..  */

	u8 phase_cycle; /* phase-cycle */
	u8 train_step;  /* train_step  */
	u8 tp_l8;       /* TP[7:0]     */
	u8 tp_h8;       /* TP[15:8]    */
};

struct fbtc_fddt_cell_status {
	s8 wl_tx_pwr;
	s8 bt_tx_pwr;
	s8 bt_rx_gain;
	u8 state_phase; /* train state(bit[7:4])_phase(bit[3:0]) */
};

struct fbtc_fddt_sta { /* statistics for cycles */
	u8 fver;
	u8 cycles_l8;
	u8 cycles_h8;
	u8 rsvd0;

	struct fbtc_fddt_cell_status fddt_cells[2][5][5];
	struct fbtc_fddt_cycle_info fddt_trx[BTC_CYCLE_SLOT_MAX];
};

struct btc_fddt_bt_stat {
	struct btc_rpt_ctrl_a2dp_empty a2dp_last;
	u32 retry_last;
};

struct btc_fddt_cell {
	s8 wl_pwr_min;
	s8 wl_pwr_max;
	s8 bt_pwr_dec_max;
	s8 bt_rx_gain;
};

struct btc_fddt_fail_check { /* for cell stay in training */
	u8 check_map;         /* check pass condition if bit-map = 1 */
	u8 bt_no_empty_cnt;   /* 0-fail if no bt-empty >= th in train_cycle */
	u8 wl_tp_ratio;       /* 1-fail if wl tp rise ratio < th */
	u8 wl_kpibtr_ratio;   /* 2-fail if phase_now_tp < phase_last_tp * kpibtr_ratio*/
};

struct btc_fddt_break_check { /* for cell stay in training or train-ok */
	u8 check_map;            /* check break condition if bit-map = 1 */
	u8 bt_no_empty_cnt;  /* 0-break if no empty count >= th  */
	u8 wl_tp_ratio;      /* 1-break if wl tp ratio < th (%)*/
	u8 wl_tp_low_bound;  /* 2-break if wl tp (in Mbps) < th */

	u8 cn;               /* 3-break if (cn >= cn_limit) >= th cycle */
	u8 cell_chg;         /* 4-break if non-matched-RSSI >= th cycle */
	s8 nhm_limit;        /* 5-break if nhm >= th --> ill-condition*/
	u8 cn_limit;         /* if condition number >= th --> ill-condition  */
};

struct btc_fddt_time_ctrl {
	/* 1 TDD cycle = w1 + b1, FDD 1cycle = w1fdd-slot + b1fdd-slot */
	u8 m_cycle; /* KPI Moving-Average-Cycle: 1~32 cycles */
	u8 w_cycle; /* Start to calcul WKPI after this if train-phase change */
	u8 k_cycle; /* Total kpi-estimate cycles for each training-step */
	u8 rsvd;
};

struct btc_fddt_train_info {
	struct btc_fddt_time_ctrl t_ctrl;
	struct btc_fddt_break_check b_chk;
	struct btc_fddt_fail_check f_chk;
	struct btc_fddt_cell cell_ul[5][5];
	struct btc_fddt_cell cell_dl[5][5];
};

struct btc_fddt_info {
	u8 type;         /* refer to enum btc_fddt_type */
	u8 result;       /* fw send fdd-training status by c2h  */
	u8 state;        /* refer to enum btc_fddt_state */

	u8 wl_iot[6];    /* wl bssid  */
	u16 bt_iot;      /* bt vendor-id */

	u32 nrsn_map;    /* the reason map for no-run fdd-traing */
	struct btc_fddt_bt_stat bt_stat;  /* bt statistics */
	struct btc_fddt_train_info train;
	struct btc_fddt_train_info train_now;
};
/* ==========================================================================*/

/*
 * End of FW Definition
 */
#pragma pack(pop)

#endif	/* __INC_BTC_FW_DEF_H__ */
