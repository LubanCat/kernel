/******************************************************************************
 *
 * Copyright(c) 2007 - 2024 Realtek Corporation.
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

#ifndef __RTW_REGDB_H__
#define __RTW_REGDB_H__

#define CHPLAN_VER_STR_BUF_LEN 64

struct rtw_regdb_ops {
	bool (*init)(void);
	void (*deinit)(void);

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

#ifdef CONFIG_RTW_CHPLAN_DEV
	void (*dump_chplan_test)(void *sel);
#endif
	void (*get_ver_str)(char *buf, size_t buf_len);
};

extern const struct rtw_regdb_ops regdb_ops;

#if CONFIG_IEEE80211_BAND_5GHZ
#define __REGDB_OPS_ASSIGN_5GHZ(_get_default_regd_5g) \
	.get_default_regd_5g = _get_default_regd_5g,

#define __REGDB_OPS_5GHZ_ASSERT \
	regdb_ops.get_default_regd_5g
#else
#define __REGDB_OPS_ASSIGN_5GHZ(_get_default_regd_5g)
#define __REGDB_OPS_5GHZ_ASSERT 1
#endif

#if CONFIG_IEEE80211_BAND_6GHZ
#define __REGDB_OPS_ASSIGN_6GHZ(_get_default_regd_6g, _is_domain_code_6g_valid, _domain_6g_get_ch) \
	.get_default_regd_6g = _get_default_regd_6g, \
	.is_domain_code_6g_valid = _is_domain_code_6g_valid, \
	.domain_6g_get_ch = _domain_6g_get_ch,

#define __REGDB_OPS_6GHZ_ASSERT \
	(regdb_ops.get_default_regd_6g && \
	regdb_ops.is_domain_code_6g_valid && \
	regdb_ops.domain_6g_get_ch)
#else
#define __REGDB_OPS_ASSIGN_6GHZ(_get_default_regd_6g, _is_domain_code_6g_valid, _domain_6g_get_ch)
#define __REGDB_OPS_6GHZ_ASSERT 1
#endif

#ifdef CONFIG_RTW_CHPLAN_DEV
#define __REGDB_OPS_ASSIGN_TEST(_dump_chplan_test) \
	.dump_chplan_test = _dump_chplan_test,
#else
#define __REGDB_OPS_ASSIGN_TEST(_dump_chplan_test)
#endif

#define DECL_REGDB_OPS( \
	_init, \
	_deinit, \
	_get_default_regd_2g, \
	_get_default_regd_5g, \
	_is_domain_code_valid, \
	_domain_get_ch, \
	_get_default_regd_6g, \
	_is_domain_code_6g_valid, \
	_domain_6g_get_ch, \
	_get_chplan_from_alpha2, \
	_dump_chplan_test, \
	_get_ver_str) \
const struct rtw_regdb_ops regdb_ops = { \
	.init = _init, \
	.deinit = _deinit, \
	.get_default_regd_2g = _get_default_regd_2g, \
	__REGDB_OPS_ASSIGN_5GHZ(_get_default_regd_5g) \
	.is_domain_code_valid = _is_domain_code_valid, \
	.domain_get_ch = _domain_get_ch, \
	__REGDB_OPS_ASSIGN_6GHZ(_get_default_regd_6g, _is_domain_code_6g_valid, _domain_6g_get_ch) \
	__REGDB_OPS_ASSIGN_TEST(_dump_chplan_test) \
	.get_chplan_from_alpha2 = _get_chplan_from_alpha2, \
	.get_ver_str = _get_ver_str, \
}; \
void assert_regdb_ops(void); \
void assert_regdb_ops(void) \
{ \
	BUILD_BUG_ON(!regdb_ops.get_default_regd_2g); \
	BUILD_BUG_ON(!__REGDB_OPS_5GHZ_ASSERT); \
	BUILD_BUG_ON(!regdb_ops.is_domain_code_valid);\
	BUILD_BUG_ON(!regdb_ops.domain_get_ch); \
	BUILD_BUG_ON(!__REGDB_OPS_6GHZ_ASSERT); \
	BUILD_BUG_ON(!regdb_ops.get_chplan_from_alpha2); \
	BUILD_BUG_ON(!regdb_ops.get_ver_str); \
}

#endif /* __RTW_REGDB_H__ */
