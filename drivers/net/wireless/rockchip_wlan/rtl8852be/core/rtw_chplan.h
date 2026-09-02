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
#ifndef __RTW_CHPLAN_H__
#define __RTW_CHPLAN_H__

#define RTW_CHPLAN_IOCTL_NULL		0xFFFE /* special value by ioctl: null(empty) chplan */
#define RTW_CHPLAN_IOCTL_UNSPECIFIED	0xFFFF /* special value by ioctl: no change (keep original) */

#define RTW_CHPLAN_NULL		0x0A
#define RTW_CHPLAN_WORLDWIDE	0x7F
#define RTW_CHPLAN_UNSPECIFIED	0xFF
#define RTW_CHPLAN_6G_NULL		0x00
#define RTW_CHPLAN_6G_WORLDWIDE		0x7F
#define RTW_CHPLAN_6G_UNSPECIFIED	0xFF

u8 rtw_chplan_get_default_regd_2g(u8 id);
#if CONFIG_IEEE80211_BAND_5GHZ
u8 rtw_chplan_get_default_regd_5g(u8 id);
#endif
bool rtw_is_channel_plan_valid(u8 id);

#if CONFIG_IEEE80211_BAND_6GHZ
u8 rtw_chplan_get_default_regd_6g(u8 id);
bool rtw_is_channel_plan_6g_valid(u8 id);
#endif

void rtw_rfctl_addl_ch_disable_conf_init(struct rf_ctl_t *rfctl, struct registry_priv *regsty);

u8 rtw_chplan_is_bchbw_valid(u8 id, u8 id_6g, enum band_type band, u8 ch, u8 bw, u8 offset
	, bool allow_primary_passive, bool allow_passive, struct rf_ctl_t *rfctl);

enum regd_src_t {
	REGD_SRC_RTK_PRIV = 0, /* Regulatory settings from Realtek framework (Realtek defined or customized) */
	REGD_SRC_OS = 1, /* Regulatory settings from OS */
	REGD_SRC_NUM,
};

#define regd_src_is_valid(src) ((src) < REGD_SRC_NUM)

extern const char *_regd_src_str[];
#define regd_src_str(src) ((src) >= REGD_SRC_NUM ? _regd_src_str[REGD_SRC_NUM] : _regd_src_str[src])

bool rtw_rfctl_reg_allow_beacon_hint(struct rf_ctl_t *rfctl);
bool rtw_chinfo_allow_beacon_hint(struct _RT_CHANNEL_INFO *chinfo);
u8 rtw_process_beacon_hint(struct rf_ctl_t *rfctl, struct wlan_network *network);
void rtw_beacon_hint_ch_change_notifier(struct rf_ctl_t *rfctl);

#define ALPHA2_FMT "%c%c"
#define ALPHA2_ARG(a2) ((is_alpha(a2[0]) || is_decimal(a2[0])) ? a2[0] : '-'), ((is_alpha(a2[1]) || is_decimal(a2[1])) ? a2[1] : '-')

#define WORLDWIDE_ALPHA2	"00"
#define UNSPEC_ALPHA2		"99"
#define INTERSECTED_ALPHA2	"98"

#define IS_ALPHA2_WORLDWIDE(_alpha2)	(strncmp(_alpha2, WORLDWIDE_ALPHA2, 2) == 0)
#define IS_ALPHA2_UNSPEC(_alpha2)	(strncmp(_alpha2, UNSPEC_ALPHA2, 2) == 0)
#define IS_ALPHA2_INTERSECTED(_alpha2)	(strncmp(_alpha2, INTERSECTED_ALPHA2, 2) == 0)
#define SET_UNSPEC_ALPHA2(_alpha2)		do { _rtw_memcpy(_alpha2, UNSPEC_ALPHA2, 2); } while (0)
#define SET_INTERSECTEDC_ALPHA2(_alpha2)	do { _rtw_memcpy(_alpha2, INTERSECTED_ALPHA2, 2); } while (0)

enum rtw_regd_inr {
	RTW_REGD_SET_BY_INIT = 0,
	RTW_REGD_SET_BY_USER = 1,
	RTW_REGD_SET_BY_COUNTRY_IE = 2,

	/* below is not used for REGD_SRC_RTK_PRIV */
	RTW_REGD_SET_BY_DRIVER = 3,
	RTW_REGD_SET_BY_CORE = 4,

	RTW_REGD_SET_BY_NUM,
};

extern const char *const _regd_inr_str[];
#define regd_inr_str(inr) (((inr) >= RTW_REGD_SET_BY_NUM) ? _regd_inr_str[RTW_REGD_SET_BY_NUM] : _regd_inr_str[(inr)])

enum rtw_regd {
	RTW_REGD_NA = 0,
	RTW_REGD_FCC = 1,
	RTW_REGD_MKK = 2,
	RTW_REGD_ETSI = 3,
	RTW_REGD_IC = 4,
	RTW_REGD_KCC = 5,
	RTW_REGD_NCC = 6,
	RTW_REGD_ACMA = 7,
	RTW_REGD_CHILE = 8,
	RTW_REGD_MEX = 9,
	RTW_REGD_WW,
	RTW_REGD_NUM,
};

extern const char *const _regd_str[];
#define regd_str(regd) (((regd) >= RTW_REGD_NUM) ? _regd_str[RTW_REGD_NA] : _regd_str[(regd)])

enum rtw_env_t {
	RTW_ENV_ANY	= 0,
	RTW_ENV_INDOOR	= 1,
	RTW_ENV_OUTDOOR	= 2,
	RTW_ENV_NUM,
};

extern const char *const _env_str[];
#define env_str(env) (((env) >= RTW_ENV_NUM) ? _env_str[RTW_ENV_ANY] : _env_str[(env)])

enum rtw_edcca_mode_t {
	RTW_EDCCA_NORM		= 0, /* normal */
	RTW_EDCCA_CS		= 1, /* carrier sense */
	RTW_EDCCA_ADAPT		= 2, /* adaptivity */
	RTW_EDCCA_CBP		= 3, /* contention based protocol */
	RTW_EDCCA_MODE_NUM,
	RTW_EDCCA_DEF		= RTW_EDCCA_MODE_NUM, /* default (ref to domain code), used at country chplan map's override field */
	RTW_EDCCA_AUTO		= 255, /* follow channel plan */
};

extern const char *const _rtw_edcca_mode_str[];
#define rtw_edcca_mode_str(mode) (((mode) >= RTW_EDCCA_MODE_NUM) ? _rtw_edcca_mode_str[RTW_EDCCA_NORM] : _rtw_edcca_mode_str[(mode)])

#define RTW_EDCCA_MODE_SEL_VALID(mode) ((mode) < RTW_EDCCA_MODE_NUM || (mode) == RTW_EDCCA_AUTO)
#define rtw_edcca_mode_sel_str(mode) (((mode) == RTW_EDCCA_AUTO) ? "AUTO" : rtw_edcca_mode_str(mode))

enum rtw_dfs_regd {
	RTW_DFS_REGD_NONE	= 0,
	RTW_DFS_REGD_FCC	= 1,
	RTW_DFS_REGD_MKK	= 2,
	RTW_DFS_REGD_ETSI	= 3,
	RTW_DFS_REGD_KCC	= 4,
	RTW_DFS_REGD_NUM,
	RTW_DFS_REGD_AUTO	= 0xFF, /* follow channel plan */
};

#define RTW_DFS_REGD_IS_UNKNOWN(regd) ((regd) == RTW_DFS_REGD_NONE || (regd) >= RTW_DFS_REGD_NUM)

extern const char *const _rtw_dfs_regd_str[];
#define rtw_dfs_regd_str(region) (((region) >= RTW_DFS_REGD_NUM) ? _rtw_dfs_regd_str[RTW_DFS_REGD_NONE] : _rtw_dfs_regd_str[(region)])

typedef enum _REGULATION_TXPWR_LMT {
	TXPWR_LMT_NONE = 0, /* no limit */
	TXPWR_LMT_FCC = 1,
	TXPWR_LMT_MKK = 2,
	TXPWR_LMT_ETSI = 3,
	TXPWR_LMT_IC = 4,
	TXPWR_LMT_KCC = 5,
	TXPWR_LMT_NCC = 6,
	TXPWR_LMT_ACMA = 7,
	TXPWR_LMT_CHILE = 8,
	TXPWR_LMT_UKRAINE = 9,
	TXPWR_LMT_MEXICO = 10,
	TXPWR_LMT_CN = 11,
	TXPWR_LMT_QATAR = 12,
	TXPWR_LMT_UK = 13,
	TXPWR_LMT_THAILAND = 14,
	TXPWR_LMT_WW, /* smallest of all available limit, keep last */

	TXPWR_LMT_NUM,
	TXPWR_LMT_DEF = TXPWR_LMT_NUM, /* default (ref to domain code), used at country chplan map's override field */
} REGULATION_TXPWR_LMT;

extern const char *const _txpwr_lmt_str[];
#define txpwr_lmt_str(regd) (((regd) >= TXPWR_LMT_NUM) ? _txpwr_lmt_str[TXPWR_LMT_NUM] : _txpwr_lmt_str[(regd)])

extern const REGULATION_TXPWR_LMT _txpwr_lmt_alternate[];
#define txpwr_lmt_alternate(ori) (((ori) > TXPWR_LMT_NUM) ? _txpwr_lmt_alternate[TXPWR_LMT_WW] : _txpwr_lmt_alternate[(ori)])

#define TXPWR_LMT_ALTERNATE_DEFINED(txpwr_lmt) (txpwr_lmt_alternate(txpwr_lmt) != txpwr_lmt)

#if CONFIG_IEEE80211_BAND_6GHZ
enum txpwr_lmt_6g_cate_t {
	TXPWR_LMT_6G_CATE_VLP,
	TXPWR_LMT_6G_CATE_LPI,
	TXPWR_LMT_6G_CATE_STD,
	TXPWR_LMT_6G_CATE_NUM,
};

extern const char *const _txpwr_lmt_6g_cate_str[];
#define txpwr_lmt_6g_cate_str(cate) (((cate) >= TXPWR_LMT_6G_CATE_NUM) ? _txpwr_lmt_6g_cate_str[TXPWR_LMT_6G_CATE_NUM] : _txpwr_lmt_6g_cate_str[(cate)])

extern const char *const _txpwr_lmt_6g_str[][TXPWR_LMT_6G_CATE_NUM];
#define txpwr_lmt_6g_str(regd, cate) (((regd) >= TXPWR_LMT_NUM || (cate) >= TXPWR_LMT_6G_CATE_NUM) ? _txpwr_lmt_str[TXPWR_LMT_NUM] : _txpwr_lmt_6g_str[(regd)][(cate)])
#endif /* CONFIG_IEEE80211_BAND_6GHZ */

extern const enum rtw_edcca_mode_t _rtw_regd_to_edcca_mode[RTW_REGD_NUM];
#define rtw_regd_to_edcca_mode(regd) (((regd) >= RTW_REGD_NUM) ? RTW_EDCCA_NORM : _rtw_regd_to_edcca_mode[(regd)])

#if CONFIG_IEEE80211_BAND_6GHZ
extern const enum rtw_edcca_mode_t _rtw_regd_to_edcca_mode_6g[RTW_REGD_NUM];
#define rtw_regd_to_edcca_mode_6g(regd) (((regd) >= RTW_REGD_NUM) ? RTW_EDCCA_NORM : _rtw_regd_to_edcca_mode_6g[(regd)])
#endif

extern const REGULATION_TXPWR_LMT _rtw_regd_to_txpwr_lmt[];
#define rtw_regd_to_txpwr_lmt(regd) (((regd) >= RTW_REGD_NUM) ? TXPWR_LMT_WW : _rtw_regd_to_txpwr_lmt[(regd)])

#define REGD_INR_BMP_STR_LEN (33)
char *rtw_get_regd_inr_bmp_str(char *buf, u8 bmp);

#define ENV_BMP_STR_LEN (11)
char *rtw_get_env_bmp_str(char *buf, u8 bmp);

#define EDCCA_MODE_OF_BANDS_STR_LEN (((6 + 3 + 1) * BAND_MAX) + 1)
char *rtw_get_edcca_mode_of_bands_str(char *buf, u8 mode_of_band[]);
void rtw_edcca_mode_update(struct dvobj_priv *dvobj, bool req_lock);
u8 rtw_get_edcca_mode(struct dvobj_priv *dvobj, enum band_type band);
void rtw_edcca_mode_sel_msg(void *sel, struct dvobj_priv *dvobj);

#if CONFIG_TXPWR_LIMIT
#define TXPWR_NAME_OF_BANDS_STR_LEN (((1 + 8 + 1) * BAND_MAX) + (4 * CONFIG_IEEE80211_BAND_6GHZ) + 1)
char *rtw_get_txpwr_lmt_name_of_bands_str(char *buf, const char *name_of_band[], u8 unknown_bmp);
void rtw_txpwr_update_cur_lmt_regs(struct dvobj_priv *dvobj, bool req_lock);
#endif

#define CHPLAN_PROTO_EN_A	BIT0
#define CHPLAN_PROTO_EN_AC	BIT1
#define CHPLAN_PROTO_EN_AX	BIT2
#define CHPLAN_PROTO_EN_BE	BIT3
#define CHPLAN_PROTO_EN_ALL	0xFF

#if defined(CONFIG_80211BE_EHT) || defined(CONFIG_80211AX_HE) || defined(CONFIG_80211AC_VHT) || CONFIG_IEEE80211_BAND_5GHZ
#define CONFIG_CHPLAN_PROTO_EN
#endif

#define CHPLAN_6G_CATE_VLP	BIT0
#define CHPLAN_6G_CATE_LPI	BIT1
#define CHPLAN_6G_CATE_STD	BIT2

struct country_chplan {
	char alpha2[2];

	u8 domain_code;
#if CONFIG_IEEE80211_BAND_6GHZ
	u8 domain_code_6g;
#endif

#if CONFIG_COUNTRY_CHPLAN_EDCCA_OVERRIDE
	/* will override edcca mode get by domain code (/6g) */
	u8 edcca_2g_override:3;
	#if CONFIG_IEEE80211_BAND_5GHZ
	u8 edcca_5g_override:3;
	#endif
	#if CONFIG_IEEE80211_BAND_6GHZ
	u8 edcca_6g_override:3;
	#endif
#endif

	/* will override txpwr_lmt get by domain code (/6g) */
	u8 txpwr_lmt_override;

#ifdef CONFIG_CHPLAN_PROTO_EN
	u8 proto_en;
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
	u8 cate_6g_map;
#endif
};

#if CONFIG_COUNTRY_CHPLAN_EDCCA_OVERRIDE
#define COUNTRY_CHPLAN_ASSIGN_EDCCA_2G_OVERRIDE(_val) , .edcca_2g_override = (_val)
#else
#define COUNTRY_CHPLAN_ASSIGN_EDCCA_2G_OVERRIDE(_val)
#endif

#if CONFIG_COUNTRY_CHPLAN_EDCCA_OVERRIDE && CONFIG_IEEE80211_BAND_5GHZ
#define COUNTRY_CHPLAN_ASSIGN_EDCCA_5G_OVERRIDE(_val) , .edcca_5g_override = (_val)
#else
#define COUNTRY_CHPLAN_ASSIGN_EDCCA_5G_OVERRIDE(_val)
#endif

#if CONFIG_COUNTRY_CHPLAN_EDCCA_OVERRIDE && CONFIG_IEEE80211_BAND_6GHZ
#define COUNTRY_CHPLAN_ASSIGN_EDCCA_6G_OVERRIDE(_val) , .edcca_6g_override = (_val)
#else
#define COUNTRY_CHPLAN_ASSIGN_EDCCA_6G_OVERRIDE(_val)
#endif

#define CHPLAN_6G_CATE_MAP____	0
#define CHPLAN_6G_CATE_MAP___V	CHPLAN_6G_CATE_VLP
#define CHPLAN_6G_CATE_MAP__I_	CHPLAN_6G_CATE_LPI
#define CHPLAN_6G_CATE_MAP__IV	(CHPLAN_6G_CATE_LPI | CHPLAN_6G_CATE_VLP)
#define CHPLAN_6G_CATE_MAP_S__	CHPLAN_6G_CATE_STD
#define CHPLAN_6G_CATE_MAP_S_V	(CHPLAN_6G_CATE_STD | CHPLAN_6G_CATE_VLP)
#define CHPLAN_6G_CATE_MAP_SI_	(CHPLAN_6G_CATE_STD | CHPLAN_6G_CATE_LPI)
#define CHPLAN_6G_CATE_MAP_SIV	(CHPLAN_6G_CATE_STD | CHPLAN_6G_CATE_LPI | CHPLAN_6G_CATE_VLP)

#if CONFIG_IEEE80211_BAND_6GHZ
#define COUNTRY_CHPLAN_ASSIGN_CHPLAN_6G(_val) , .domain_code_6g = (_val)
#define COUNTRY_CHPLAN_ASSIGN_6G_CATE_MAP(_val) , .cate_6g_map = CHPLAN_6G_CATE_MAP_##_val
#else
#define COUNTRY_CHPLAN_ASSIGN_CHPLAN_6G(_val)
#define COUNTRY_CHPLAN_ASSIGN_6G_CATE_MAP(_val)
#endif

#ifdef CONFIG_CHPLAN_PROTO_EN
#define COUNTRY_CHPLAN_ASSIGN_PROTO_EN(_val) , .proto_en = (_val)
#else
#define COUNTRY_CHPLAN_ASSIGN_PROTO_EN(_val)
#endif

#define COUNTRY_CHPLAN_ENT_ARGC_9(_alpha2, _chplan, _chplan_6g, _txpwr_lmt_override, _en_11be, _en_11ax, _en_11ac, _en_11a, _cate_6g_map) \
	{ \
		.alpha2 = (_alpha2), .domain_code = (_chplan) \
		COUNTRY_CHPLAN_ASSIGN_CHPLAN_6G(_chplan_6g) \
		COUNTRY_CHPLAN_ASSIGN_EDCCA_2G_OVERRIDE(RTW_EDCCA_DEF) \
		COUNTRY_CHPLAN_ASSIGN_EDCCA_5G_OVERRIDE(RTW_EDCCA_DEF) \
		COUNTRY_CHPLAN_ASSIGN_EDCCA_6G_OVERRIDE(RTW_EDCCA_DEF) \
		, .txpwr_lmt_override = TXPWR_LMT_##_txpwr_lmt_override \
		COUNTRY_CHPLAN_ASSIGN_PROTO_EN( \
			(_en_11be ? CHPLAN_PROTO_EN_BE : 0) | (_en_11ax ? CHPLAN_PROTO_EN_AX : 0) \
			| (_en_11ac ? CHPLAN_PROTO_EN_AC : 0) | (_en_11a ? CHPLAN_PROTO_EN_A : 0)) \
		COUNTRY_CHPLAN_ASSIGN_6G_CATE_MAP(_cate_6g_map) \
	}

#define COUNTRY_CHPLAN_ENT_ARGC_7(_alpha2, _chplan, _chplan_6g, _txpwr_lmt_override, _en_11ax, _en_11ac, _en_11a) \
	COUNTRY_CHPLAN_ENT_ARGC_9(_alpha2, _chplan, _chplan_6g, _txpwr_lmt_override, 0, _en_11ax, _en_11ac, _en_11a, ___)

#define __COUNTRY_CHPLAN_ENT_HDL(argc9, argc8, argc7, argc6, argc5, argc4, argc3, argc2, argc1, hdl, ...) hdl

#define COUNTRY_CHPLAN_ENT(...) \
	__COUNTRY_CHPLAN_ENT_HDL(__VA_ARGS__, \
		COUNTRY_CHPLAN_ENT_ARGC_9, \
		macro_argc8, \
		COUNTRY_CHPLAN_ENT_ARGC_7, \
		macro_argc6, \
		macro_argc5, \
		macro_argc4, \
		macro_argc3, \
		macro_argc2, \
		macro_argc1) \
		(__VA_ARGS__)

#if CONFIG_COUNTRY_CHPLAN_EDCCA_OVERRIDE
#define COUNTRY_CHPLAN_EDCCA_2G_OVERRIDE(_ent) (_ent)->edcca_2g_override
#else
#define COUNTRY_CHPLAN_EDCCA_2G_OVERRIDE(_ent) RTW_EDCCA_DEF
#endif

#if CONFIG_COUNTRY_CHPLAN_EDCCA_OVERRIDE && CONFIG_IEEE80211_BAND_5GHZ
#define COUNTRY_CHPLAN_EDCCA_5G_OVERRIDE(_ent) (_ent)->edcca_5g_override
#else
#define COUNTRY_CHPLAN_EDCCA_5G_OVERRIDE(_ent) RTW_EDCCA_DEF
#endif

#if CONFIG_COUNTRY_CHPLAN_EDCCA_OVERRIDE && CONFIG_IEEE80211_BAND_6GHZ
#define COUNTRY_CHPLAN_EDCCA_6G_OVERRIDE(_ent) (_ent)->edcca_6g_override
#else
#define COUNTRY_CHPLAN_EDCCA_6G_OVERRIDE(_ent) RTW_EDCCA_DEF
#endif

#if CONFIG_IEEE80211_BAND_5GHZ
#define COUNTRY_CHPLAN_EN_11A(_ent) (((_ent)->proto_en & CHPLAN_PROTO_EN_A) ? true : false)
#else
#define COUNTRY_CHPLAN_EN_11A(_ent) false
#endif

#ifdef CONFIG_80211AC_VHT
#define COUNTRY_CHPLAN_EN_11AC(_ent) (((_ent)->proto_en & CHPLAN_PROTO_EN_AC) ? true : false)
#else
#define COUNTRY_CHPLAN_EN_11AC(_ent) false
#endif

#ifdef CONFIG_80211AX_HE
#define COUNTRY_CHPLAN_EN_11AX(_ent) (((_ent)->proto_en & CHPLAN_PROTO_EN_AX) ? true : false)
#else
#define COUNTRY_CHPLAN_EN_11AX(_ent) false
#endif

#ifdef CONFIG_80211BE_EHT
#define COUNTRY_CHPLAN_EN_11BE(_ent) (((_ent)->proto_en & CHPLAN_PROTO_EN_BE) ? true : false)
#else
#define COUNTRY_CHPLAN_EN_11BE(_ent) false
#endif

void rtw_get_chplan_worldwide(struct country_chplan *ent);
bool rtw_get_chplan_from_country(const char *country_code, struct country_chplan *ent);

void rtw_chplan_ioctl_input_mapping(u16 *chplan, u16 *chplan_6g);
bool rtw_chplan_ids_is_world_wide(u8 chplan, u8 chplan_6g);

u8 rtw_country_chplan_is_bchbw_valid(struct country_chplan *ent, enum band_type band, u8 ch, u8 bw, u8 offset
	, bool allow_primary_passive, bool allow_passive, struct rf_ctl_t *rfctl);

enum country_ie_slave_en_mode {
	CISEM_DISABLE	= 0, /* disable */
	CISEM_ENABLE	= 1, /* enable */
	CISEM_ENABLE_WW	= 2, /* enable when INIT/USER set world wide mode */
	CISEM_NUM,
};

#define CIS_EN_MODE_IS_VALID(mode) ((mode) < CISEM_NUM)

enum country_ie_slave_flags {
	_CISF_RSVD_0	= BIT0, /* deprecated BIT0 */
	CISF_ENV_BSS	= BIT1, /* if not set, consider IEs of associated BSSs only */

	CISF_VALIDS = CISF_ENV_BSS,
};

enum country_ie_slave_6g_reg_info {
	CIS_6G_REG_IN_AP	= 0,
	CIS_6G_REG_SP_AP	= 1,
	CIS_6G_REG_VLP_AP	= 2,
	CIS_6G_REG_IN_EN_AP	= 3,
	CIS_6G_REG_IN_SP_AP	= 4,
	CIS_6G_REG_NUM,
	CIS_6G_REG_RSVD		= CIS_6G_REG_NUM,
};

#define IS_6G_REG_INFO_RSVD(reg) ((reg) >= CIS_6G_REG_NUM)

enum country_ie_slave_status {
	COUNTRY_IE_SLAVE_NOCOUNTRY	= 0, /* no country IE or 'XX' alpha2 */
	COUNTRY_IE_SLAVE_APPLICABLE	= 1,
	COUNTRY_IE_SLAVE_UNKNOWN	= 2, /* unknown country */
	COUNTRY_IE_SLAVE_OPCH_NOEXIST	= 3, /* bss's op ch not exist */
	COUNTRY_IE_SLAVE_CATE_6G_NS	= 4, /* 6G category not support */

	COUNTRY_IE_SLAVE_STATUS_NUM,
};

struct country_ie_slave_record {
	char alpha2[2]; /* country code get from connected AP of STA ifaces, "\x00\x00" is not set */
	enum band_type band;
	u8 opch;
#if CONFIG_IEEE80211_BAND_6GHZ
	enum rtw_env_t env;
	enum country_ie_slave_6g_reg_info reg_info;
#endif
	enum country_ie_slave_status status;
	struct country_chplan chplan;
};

#ifdef CONFIG_80211D
extern const char _rtw_env_char[];
#define rtw_env_char(e) (((e) >= RTW_ENV_NUM) ? _rtw_env_char[RTW_ENV_ANY] : _rtw_env_char[e])

void dump_country_ie_slave_records(void *sel, struct rf_ctl_t *rfctl, bool skip_noset);
#endif

void dump_country_chplan(void *sel, const struct country_chplan *ent, bool regd_info);
void dump_country_chplan_map(void *sel, bool regd_info);
void dump_country_list(void *sel);
void dump_chplan_id_list(void *sel);
void dump_chplan_country_list(void *sel);
#if CONFIG_IEEE80211_BAND_6GHZ
void dump_chplan_6g_id_list(void *sel);
void dump_chplan_6g_country_list(void *sel);
#endif
#ifdef CONFIG_RTW_DEBUG
void dump_chplan_test(void *sel);
#endif
void dump_chplan_ver(void *sel);

struct regd_req_t {
	_list list;
	int ref_cnt; /* used by RTK_PRIV's COUNTRY_IE req */
	bool applied;

	enum regd_src_t src;
	enum rtw_regd_inr inr;
#if CONFIG_IEEE80211_BAND_6GHZ
	enum rtw_env_t env;
	enum country_ie_slave_6g_reg_info reg_info;
	u8 txpwr_lmt_6g_cate_map;
#endif

	struct country_chplan chplan;
};

void rtw_regd_req_list_init(struct rf_ctl_t *rfctl, struct registry_priv *regsty);
void rtw_regd_req_list_free(struct rf_ctl_t *rfctl);

void rtw_regd_watchdog_hdl(struct dvobj_priv *dvobj);

struct _ADAPTER_LINK;
enum channel_width alink_adjust_linking_bw_by_regd(struct _ADAPTER_LINK *alink
	, enum band_type band, u8 ch, enum channel_width bw, enum chan_offset offset);
enum channel_width adapter_adjust_linking_bw_by_regd(_adapter *adapter
	, enum band_type band, u8 ch, enum channel_width bw, enum chan_offset offset);
#ifdef CONFIG_AP_MODE
enum channel_width alink_adjust_slave_m_bw_by_regd(struct _ADAPTER_LINK *alink
	, enum band_type band, u8 ch, enum channel_width bw, enum chan_offset offset);
enum channel_width adapter_adjust_slave_m_bw_by_regd(_adapter *adapter
	, enum band_type band, u8 ch, enum channel_width bw, enum chan_offset offset);
#endif

void rtw_rfctl_decide_init_chplan(struct rf_ctl_t *rfctl,
	const char *hw_alpha2, u8 hw_chplan, u8 hw_chplan_6g, u8 hw_force_chplan);
void rtw_rfctl_apply_init_chplan(struct rf_ctl_t *rfctl, bool req_lock);

bool rtw_rfctl_is_disable_sw_channel_plan(struct dvobj_priv *dvobj);

#define alink_regu_block_tx(alink) (LINK_MLME_IS_REGU_FORBID(alink) || alink_is_tx_blocked_by_ch_waiting(alink))
#define adapter_regu_block_tx(adapter) alink_regu_block_tx(GET_PRIMARY_LINK(adapter))

#if defined(CONFIG_AP_MODE) && CONFIG_AP_REGU_FORBID
bool rtw_rfctl_is_regu_forbid_bss(struct rf_ctl_t *rfctl, enum band_type band);
#else
#define rtw_rfctl_is_regu_forbid_bss(rfctl, band) false
#endif

enum chplan_confs_type {
	CHPLAN_CONFS_DIS_CH_FLAGS,
	CHPLAN_CONFS_EXCL_CHS,
	CHPLAN_CONFS_EXCL_CHS_6G,
	CHPLAN_CONFS_INIT_REGD_ALWAYS_APPLY,
	CHPLAN_CONFS_USER_REGD_ALWAYS_APPLY,
	CHPLAN_CONFS_BCN_HINT_VALID_MS,
	CHPLAN_CONFS_EDCCA_MODE_SEL,
	CHPLAN_CONFS_CIS_EN_MODE,
	CHPLAN_CONFS_CIS_FLAGS,
	CHPLAN_CONFS_CIS_EN_ROLE,
	CHPLAN_CONFS_CIS_EN_IFBMP,
	CHPLAN_CONFS_CIS_SCAN_INT_MS,
	CHPLAN_CONFS_NUM,
};

struct chplan_confs {
	u16 set_types; /* bitmap of chplan_confs_type */
	u8 dis_ch_flags;
	u8 excl_chs[MAX_CHANNEL_NUM_2G_5G];
#if CONFIG_IEEE80211_BAND_6GHZ
	u8 excl_chs_6g[MAX_CHANNEL_NUM_6G];
#endif
	bool init_regd_always_apply;
	bool user_regd_always_apply;
	u32 bcn_hint_valid_ms;
	u8 edcca_mode_sel;
#ifdef CONFIG_80211D
	u8 cis_en_mode;
	u8 cis_flags;
	u8 cis_en_role;
	u8 cis_en_ifbmp;
	u32 cis_scan_int_ms;
#endif
};

struct SetChannelPlan_param {
	enum regd_src_t regd_src;
	enum rtw_regd_inr inr;
	struct country_chplan country_ent;
	bool has_country;
	u8 channel_plan;
#if CONFIG_IEEE80211_BAND_6GHZ
	u8 channel_plan_6g;
	enum rtw_env_t env;
#endif
	/* used for regd_src == RTK_PRIV and inr == USER, bitmap of  RTW_PRIV_USER_SET_XXX */
	u8 priv_user_set_bmp;

#ifdef CONFIG_80211D
	/* used for regd_src == RTK_PRIV and inr == COUNTRY_IE */
	struct country_ie_slave_record cisr;
	u8 cisr_alink_id;
	bool has_cisr;
#endif

	struct chplan_confs confs;

#ifdef PLATFORM_LINUX
	bool rtnl_lock_needed;
#endif
};

u8 rtw_set_chplan_hdl(_adapter *adapter, u8 *pbuf);

u8 rtw_set_chplan_cmd(_adapter *adapter, int flags, u8 chplan, u8 chplan_6g
	, enum rtw_env_t env, enum rtw_regd_inr inr);
u8 rtw_set_country_cmd(_adapter *adapter, int flags, const char *country_code
	, enum rtw_env_t env, enum rtw_regd_inr inr);
#if CONFIG_IEEE80211_BAND_6GHZ
u8 rtw_set_env_cmd(_adapter *adapter, int flags, enum rtw_env_t env
	, enum regd_src_t regd_src, enum rtw_regd_inr inr);
#endif
#ifdef CONFIG_80211D
u8 rtw_alink_apply_recv_regu_ies_cmd(struct _ADAPTER_LINK *alink, int flags, enum band_type band,u8 opch
	, const u8 *country_ie, enum country_ie_slave_6g_reg_info reg_info);
u8 rtw_apply_recv_regu_ies_cmd(_adapter *adapter, int flags, enum band_type band,u8 opch
	, const u8 *country_ie, enum country_ie_slave_6g_reg_info reg_info);
u8 rtw_apply_scan_network_country_ie_cmd(_adapter *adapter, int flags);
#endif
#ifdef CONFIG_REGD_SRC_FROM_OS
u8 rtw_sync_os_regd_cmd(_adapter *adapter, int flags, const char *country_code
	, u8 dfs_region, enum rtw_env_t env, enum rtw_regd_inr inr);
#endif
u8 rtw_set_chplan_confs_cmd(_adapter *adapter, int flags, struct chplan_confs *confs);
#ifdef CONFIG_PROC_DEBUG
u16 rtw_parse_chplan_confs_cmd_str(struct chplan_confs *confs, char *str);
#endif

struct get_chplan_resp {
	enum regd_src_t regd_src;
	u8 regd_inr_bmp;
	char alpha2[2];
	u8 channel_plan;
#if CONFIG_IEEE80211_BAND_6GHZ
	u8 chplan_6g;
	u8 env_bmp;
#endif

#if CONFIG_TXPWR_LIMIT
	/* point to content of txpwr_lmt_names of specific band (after content of chset) */
	const char *txpwr_lmt_names[BAND_MAX];
	int txpwr_lmt_names_len[BAND_MAX];
	int txpwr_lmt_names_len_total; /* ease for free operation */
#endif
	u8 edcca_mode_2g;
#if CONFIG_IEEE80211_BAND_5GHZ
	u8 edcca_mode_5g;
#endif
#if CONFIG_IEEE80211_BAND_6GHZ
	u8 edcca_mode_6g;
#endif
#ifdef CONFIG_DFS_MASTER
	u8 dfs_domain;
#endif
	u8 proto_en;

	struct chplan_confs confs;

	u8 chs_len;
	RT_CHANNEL_INFO chs[];
};

struct get_channel_plan_param {
	struct get_chplan_resp *chplan;
};

u8 rtw_get_chplan_hdl(_adapter *adapter, u8 *pbuf);
struct cmd_obj;
void rtw_get_chplan_callback(_adapter *adapter, struct cmd_obj *cmdobj);

u8 rtw_get_chplan_cmd(_adapter *adapter, int flags, struct get_chplan_resp **chplan);
void rtw_free_get_chplan_resp(struct get_chplan_resp *chplan);

bool rtw_network_chk_opch_status(struct rf_ctl_t *rfctl, struct wlan_network *network);

#ifdef CONFIG_80211D
void rtw_alink_joinbss_update_regulatory(struct _ADAPTER_LINK *alink, const WLAN_BSSID_EX *network);
void rtw_alink_leavebss_update_regulatory(struct _ADAPTER_LINK * alink);
void rtw_alink_csa_update_regulatory(struct _ADAPTER_LINK *alink, enum band_type req_band, u8 req_ch);
void alink_process_regu_ies(struct _ADAPTER_LINK *alink, u8 *ies, uint ies_len);

void rtw_joinbss_update_regulatory(_adapter *adapter, const WLAN_BSSID_EX *network);
void rtw_leavebss_update_regulatory(_adapter *adapter);
void rtw_csa_update_regulatory(_adapter *adapter, enum band_type req_band, u8 req_ch);
void process_regu_ies(_adapter *adapter, u8 *ies, uint ies_len);

bool rtw_update_scanned_network_cisr(struct rf_ctl_t *rfctl, struct wlan_network *network);
bool rtw_network_chk_regu_ies(struct rf_ctl_t *rfctl, struct wlan_network *network);

void rtw_rfctl_cis_init(struct rf_ctl_t *rfctl, struct registry_priv *regsty);
void rtw_rfctl_cis_deinit(struct rf_ctl_t *rfctl);
#else
#define rtw_network_chk_regu_ies(rfctl, network) true
#endif /* CONFIG_80211D */

#ifdef CONFIG_PROC_DEBUG
void dump_cur_chplan_confs(void *sel, struct rf_ctl_t *rfctl);
void dump_cur_country(void *sel, struct rf_ctl_t *rfctl);
void dump_cur_chplan(void *sel, struct rf_ctl_t *rfctl);
#if CONFIG_IEEE80211_BAND_6GHZ
void dump_cur_env(void *sel, struct rf_ctl_t *rfctl);
#endif
#endif /* CONFIG_PROC_DEBUG */

#define CHPLAN_VER_STR_BUF_LEN 64

struct rtw_regdb_ops {
	u8 (*get_default_regd_2g)(u8 id);
#if CONFIG_IEEE80211_BAND_5GHZ
	u8 (*get_default_regd_5g)(u8 id);
#endif
	bool (*is_domain_code_valid)(u8 id);
	bool (*domain_get_ch)(u8 id, u32 ch, u8 *flags);
#if CONFIG_IEEE80211_BAND_6GHZ
	u8 (*get_default_regd_6g)(u8 id);
	bool (*is_domain_code_6g_valid)(u8 id);
	bool (*domain_6g_get_ch)(u8 id, u32 ch, u8 *flags);
#endif

	bool (*get_chplan_from_alpha2)(const char *alpha2, struct country_chplan *ent);

#ifdef CONFIG_RTW_DEBUG
	void (*dump_chplan_test)(void *sel);
#endif
	void (*get_ver_str)(char *buf, size_t buf_len);
};

#endif /* __RTW_CHPLAN_H__ */
