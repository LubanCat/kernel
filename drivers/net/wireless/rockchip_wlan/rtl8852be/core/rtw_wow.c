/******************************************************************************
 *
 * Copyright(c) 2007 - 2019 Realtek Corporation.
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
#define _RTW_WOW_C_

#include <drv_types.h>

#if defined(CONFIG_WOWLAN) || defined(CONFIG_AP_WOWLAN)
	#include <linux/inetdevice.h>
	#define ETH_TYPE_OFFSET	12
	#define PROTOCOL_OFFSET	23
	#define IP_OFFSET	30
	#define IPv6_OFFSET	38
	#define IPv6_PROTOCOL_OFFSET	20
#endif

#ifdef CONFIG_WOWLAN

void rtw_init_wow(_adapter *padapter)
{
	struct registry_priv  *registry_par = &padapter->registrypriv;
	struct pwrctrl_priv *pwrctrlpriv = adapter_to_pwrctl(padapter);
	struct wow_priv *wowpriv = adapter_to_wowlan(padapter);

#ifdef CONFIG_GPIO_WAKEUP
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);
	void *phl = GET_PHL_INFO(dvobj);
	enum rtw_phl_status status = RTW_PHL_STATUS_FAILURE;
	struct rtw_wow_gpio_info *wow_gpio = &wowpriv->wow_gpio;
	struct rtw_dev2hst_gpio_info *d2h_gpio_info = &wow_gpio->d2h_gpio_info;
	u8 toggle_pulse = CONFIG_TOGGLE_PULSE, gpio_time_unit = 1, gpio_pulse_count = CONFIG_PULSE_COUNT;
	u8 gpio_pulse_period = 20, gpio_pulse_dura = 10;
	u8 rsn_a_en = 0, rsn_a = 0, rsn_a_time_unit = 0, rsn_a_toggle_pulse = DEV2HST_TOGGLE;
	u8 rsn_a_pulse_count = 0, rsn_a_pulse_period = 0, rsn_a_pulse_duration = 0;
#endif
#ifdef CONFIG_WOW_PERIODIC_WAKE
	struct rtw_periodic_wake_info *wow_periodic_wake = &wowpriv->wow_periodic_wake;
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_ANDROID_POWER)
	pwrctrlpriv->early_suspend.suspend = NULL;
	rtw_register_early_suspend(pwrctrlpriv);
#endif /* CONFIG_HAS_EARLYSUSPEND || CONFIG_ANDROID_POWER */
	pwrctrlpriv->wowlan_mode = _FALSE;
	pwrctrlpriv->wowlan_ap_mode = _FALSE;
	pwrctrlpriv->wowlan_p2p_mode = _FALSE;
	pwrctrlpriv->wowlan_in_resume = _FALSE;

	wowpriv->wow_wake_reason = 0;
	wowpriv->wowlan_aoac_rpt.wow_pattern_idx = -1; /*avoid confusion with idx 0*/

#ifdef CONFIG_GPIO_WAKEUP
#ifdef PRIVATE_R
	rsn_a_en = 1;
	toggle_pulse = DEV2HST_PULSE;
	gpio_pulse_count = 1;
	gpio_pulse_dura = 20;
	gpio_pulse_period = 40;
	rsn_a_toggle_pulse = DEV2HST_PULSE;
	rsn_a_time_unit = 1;
	rsn_a_pulse_count = 1;
	rsn_a_pulse_duration = 60;
	rsn_a_pulse_period = 120;
	rsn_a = 0x21;
#endif /* PRIVATE_R */
	pwrctrlpriv->hst2dev_high_active = HIGH_ACTIVE_HST2DEV;
	/*default low active*/
	d2h_gpio_info->gpio_active = HIGH_ACTIVE_DEV2HST;

	d2h_gpio_info->toggle_pulse = toggle_pulse;
	d2h_gpio_info->gpio_time_unit = gpio_time_unit;
	d2h_gpio_info->gpio_pulse_dura = gpio_pulse_dura;
	d2h_gpio_info->gpio_pulse_period = gpio_pulse_period;
	d2h_gpio_info->gpio_pulse_count = gpio_pulse_count;

	wow_gpio->dev2hst_gpio = WAKEUP_GPIO_IDX;
	d2h_gpio_info->rsn_a_en = rsn_a_en;
	d2h_gpio_info->rsn_a_toggle_pulse = rsn_a_toggle_pulse;
	d2h_gpio_info->rsn_a_time_unit = rsn_a_time_unit;
	d2h_gpio_info->rsn_a = rsn_a;
	d2h_gpio_info->rsn_a_pulse_duration = rsn_a_pulse_duration;
	d2h_gpio_info->rsn_a_pulse_period = rsn_a_pulse_period;
	d2h_gpio_info->rsn_a_pulse_count = rsn_a_pulse_count;

#ifdef CONFIG_RTW_ONE_PIN_GPIO
	wow_gpio->dev2hst_gpio_mode = RTW_AX_SW_IO_MODE_INPUT;
	status = rtw_phl_cfg_wow_set_sw_gpio_mode(phl, wow_gpio);
#else
	#ifdef CONFIG_WAKEUP_GPIO_INPUT_MODE
	wow_gpio->dev2hst_gpio_mode = RTW_AX_SW_IO_MODE_OUTPUT_OD;
	#else
	wow_gpio->dev2hst_gpio_mode = RTW_AX_SW_IO_MODE_OUTPUT_PP;
	#endif /*CONFIG_WAKEUP_GPIO_INPUT_MODE*/
	/* switch GPIO to open-drain or push-pull */
	status = rtw_phl_cfg_wow_set_sw_gpio_mode(phl, wow_gpio);
	wow_gpio->dev2hst_high = d2h_gpio_info->gpio_active == 0 ? 1 : 0;
	status = rtw_phl_cfg_wow_sw_gpio_ctrl(phl, wow_gpio);
	RTW_INFO("%s: set GPIO_%d %d as default. status=%d\n",
		 __func__, WAKEUP_GPIO_IDX, wow_gpio->dev2hst_high, status);
#endif /* CONFIG_RTW_ONE_PIN_GPIO */
#endif /* CONFIG_GPIO_WAKEUP */

#ifdef CONFIG_WOWLAN
	if (!(registry_par->wakeup_event & BIT(3)))
		rtw_wow_pattern_clean(padapter, RTW_CUSTOMIZED_PATTERN);

	rtw_wow_pattern_clean(padapter, RTW_DEFAULT_PATTERN);

	_rtw_mutex_init(&pwrctrlpriv->wowlan_pattern_cam_mutex);

	pwrctrlpriv->wowlan_aoac_rpt_loc = 0;
#ifdef CONFIG_MDNS_OFFLOAD
	_rtw_memset(&wowpriv->mdns_ofld_info, 0,
			sizeof(struct rtw_mdns_ofld_info));
#endif
#ifdef CONFIG_APF
	rtw_init_apf(padapter);
#endif /* CONFIG_APF */
#endif /* CONFIG_WOWLAN */

#ifdef CONFIG_WOW_PERIODIC_WAKE
	wow_periodic_wake->wake_period = WOW_DEFAULT_WAKE_PERIOD;
	wow_periodic_wake->wake_duration = WOW_DEFAULT_WAKE_DURATION;
#endif
}

void rtw_free_wow(_adapter *adapter)
{
	struct pwrctrl_priv *pwrctrlpriv = adapter_to_pwrctl(adapter);

	_rtw_mutex_free(&pwrctrlpriv->wowlan_pattern_cam_mutex);

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_ANDROID_POWER)
	rtw_unregister_early_suspend(pwrctrlpriv);
#endif /* CONFIG_HAS_EARLYSUSPEND || CONFIG_ANDROID_POWER */
}

void rtw_wowlan_set_pattern_cast_type(_adapter *adapter, struct rtw_wowcam_upd_info *wowcam_info)
{
	if (is_broadcast_mac_addr(wowcam_info->ptrn))
		wowcam_info->bc = 1;
	else if (is_multicast_mac_addr(wowcam_info->ptrn))
		wowcam_info->mc = 1;
	else if (!memcmp(wowcam_info->ptrn, adapter_mac_addr(adapter), ETH_ALEN))
		wowcam_info->uc = 1;
}

bool rtw_wowlan_parser_pattern_cmd(u8 *input, char *pattern,
				   int *pattern_len, char *bit_mask)
{
	char *cp = NULL;
	size_t len = 0;
	int pos = 0, mask_pos = 0, res = 0;
	u8 member[2] = {0};

	/* To get the pattern string after "=", when we use :
	 * iwpriv wlanX pattern=XX:XX:..:XX
	 */
	cp = strchr(input, '=');
	if (cp) {
		*cp = 0;
		cp++;
		input = cp;
	}

	/* To take off the newline character '\n'(0x0a) at the end of pattern string,
	 * when we use echo xxxx > /proc/xxxx
	 */
	cp = strchr(input, '\n');
	if (cp)
		*cp = 0;

	while (input) {
		cp = strsep((char **)(&input), ":");

		if (bit_mask && (strcmp(cp, "-") == 0 ||
				 strcmp(cp, "xx") == 0 ||
				 strcmp(cp, "--") == 0)) {
			/* skip this byte and leave mask bit unset */
		} else {
			u8 hex;

			if (strlen(cp) != 2) {
				RTW_ERR("%s:[ERROR] hex len != 2, input=[%s]\n",
					__func__, cp);
				goto error;
			}

			if (hexstr2bin(cp, &hex, 1) < 0) {
				RTW_ERR("%s:[ERROR] pattern is invalid, input=[%s]\n",
					__func__, cp);
				goto error;
			}

			pattern[pos] = hex;
			mask_pos = pos / 8;
			if (bit_mask)
				bit_mask[mask_pos] |= 1 << (pos % 8);
		}

		pos++;
	}

	(*pattern_len) = pos;

	return _TRUE;
error:
	return _FALSE;
}

u8 rtw_wow_pattern_set(_adapter *adapter, struct rtw_wowcam_upd_info *wowcam_info,
		       enum pattern_type set_type)
{
	struct wow_priv *wowpriv = adapter_to_wowlan(adapter);
	struct dvobj_priv *dvobj = adapter_to_dvobj(adapter);
	void *phl = GET_PHL_INFO(dvobj);
	enum rtw_phl_status status = RTW_PHL_STATUS_FAILURE;
	u8 cam_idx = MAX_WKFM_CAM_NUM;

	rtw_wowlan_set_pattern_cast_type(adapter, wowcam_info);
	status = rtw_phl_add_wow_ptrn_info(phl, wowcam_info, &cam_idx);
	if (status != RTW_PHL_STATUS_SUCCESS) {
		RTW_INFO("Add wow pattern fail(%d)\n", status);
		return _FAIL;
	}

	wowpriv->wow_ptrn_valid[cam_idx] = set_type;

	RTW_INFO("wowcam_id: %d, type: %d\n", cam_idx, set_type);

	return _SUCCESS;
}

void rtw_wow_pattern_clean(_adapter *adapter, enum pattern_type clean_type)
{
	struct wow_priv *wowpriv = adapter_to_wowlan(adapter);
	struct dvobj_priv *dvobj = adapter_to_dvobj(adapter);
	void *phl = GET_PHL_INFO(dvobj);
	u8 cam_idx;

	for (cam_idx = 0; cam_idx < MAX_WKFM_CAM_NUM; cam_idx++) {
		if (wowpriv->wow_ptrn_valid[cam_idx] == clean_type) {
			rtw_phl_remove_wow_ptrn_info(phl, cam_idx);
			wowpriv->wow_ptrn_valid[cam_idx] = RTW_INVALID_PATTERN;
		}
	}
}

#ifdef CONFIG_GOOGLE_CAST_WAKEUP
#define rtw_user_wow_patten_elem(len, array, args...)	\
	{.conts_len = len,  .pconts = (u8[len]){array, ##args}}

void rtw_set_google_cast_mdns_wow_pattern(_adapter *adapter)
{
	struct pwrctrl_priv *pwrpriv = adapter_to_pwrctl(adapter);
	struct mlme_ext_priv *pmlmeext = &adapter->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &pmlmeext->mlmext_info;
	struct rtw_wowcam_upd_info wowcam_info = {0};
	u8 index = 0;
	u8 eth_dest_mac[ETH_ALEN] = {0x01, 0x00, 0x5e, 0x00, 0x00, 0xfb};
	u8 eth_protocol[2] = {0x08, 0x00};
	u8 ip_ver = 0x45;
	u8 ip_protocol = 0x11;
	u8 dest_port[2] = {0x14, 0xe9};
	u8 *ptr;
	struct pattern_cont_t {
		u8 conts_len;
		u8 *pconts;
	};
	struct pattern_cont_t conts[] = {
		rtw_user_wow_patten_elem(0x51, /*_%9E5E7C8F47989526C9BCD95D24084F6F0B27C5ED._sub._googlecast._tcp.local*/
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, 0x5F, 0x25, 0x39, 0x45, 0x35,
				0x45, 0x37, 0x43, 0x38, 0x46, 0x34, 0x37, 0x39, 0x38, 0x39, 0x35, 0x32, 0x36, 0x43, 0x39, 0x42,
				0x43, 0x44, 0x39, 0x35, 0x44, 0x32, 0x34, 0x30, 0x38, 0x34, 0x46, 0x36, 0x46, 0x30, 0x42, 0x32,
				0x37, 0x43, 0x35, 0x45, 0x44, 0x04, 0x5F, 0x73, 0x75, 0x62, 0x0B, 0x5F, 0x67, 0x6F, 0x6F, 0x67,
				0x6C, 0x65, 0x63, 0x61, 0x73, 0x74, 0x04, 0x5F, 0x74, 0x63, 0x70, 0x05, 0x6C, 0x6F, 0x63, 0x61,
				0x6C),
		rtw_user_wow_patten_elem(0x21, /*_googlecast._tcp.local*/
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x5F, 0x67, 0x6F, 0x6F, 0x67,
				0x6C, 0x65, 0x63, 0x61, 0x73, 0x74, 0x04, 0x5F, 0x74, 0x63, 0x70, 0x05, 0x6C, 0x6F, 0x63, 0x61,
				0x6C),
		rtw_user_wow_patten_elem(0x30, /*_233637DE._sub._googlecast._tcp.local*/
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x5F, 0x32, 0x33, 0x33, 0x36,
				0x33, 0x37, 0x44, 0x45, 0x04, 0x5F, 0x73, 0x75, 0x62, 0x0B, 0x5F, 0x67, 0x6F, 0x6F, 0x67, 0x6C,
				0x65, 0x63, 0x61, 0x73, 0x74, 0x04, 0x5F, 0x74, 0x63, 0x70, 0x05, 0x6C, 0x6F, 0x63, 0x61, 0x6C),
	};
	struct pattern_cont_t masks[] = {
		rtw_user_wow_patten_elem(MAX_WKFM_SIZE,
				0x3f, 0x70, 0x80, 0x00, 0x30, 0x30, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f),
		rtw_user_wow_patten_elem(10,
				0x3f, 0x70, 0x80, 0x00, 0x30, 0x30, 0xc0, 0xff, 0xff, 0x1f),
		rtw_user_wow_patten_elem(12,
				0x3f, 0x70, 0x80, 0x00, 0x30, 0x30, 0xc0, 0xff, 0xff, 0xff, 0xff, 0x0f),
	};

	u8 *target = NULL;

	for (index = 0 ; index < GOOGLE_CAST_PATTERN_NUM ; index++){
		_rtw_memset((void *)&wowcam_info, 0, sizeof(wowcam_info));

		target = wowcam_info.ptrn;

		_rtw_memcpy(target, eth_dest_mac, ETH_ALEN);
		target += ETH_TYPE_OFFSET;
		wowcam_info.ptrn_len += ETH_TYPE_OFFSET;

		_rtw_memcpy(target, eth_protocol, 2);
		target += 2;
		wowcam_info.ptrn_len += 2;

		*target = ip_ver;
		target += 1;
		wowcam_info.ptrn_len += 1;

		/*padding*/
		target += 8;
		wowcam_info.ptrn_len += 8;

		*target = ip_protocol;
		target += 1;
		wowcam_info.ptrn_len += 1;

		/*padding*/
		target += 2;
		wowcam_info.ptrn_len += 2;

		/*skip src & des ip*/
		target += (RTW_IP_ADDR_LEN * 2);
		wowcam_info.ptrn_len += (RTW_IP_ADDR_LEN * 2);

		/*skip src port*/
		target += 2;
		wowcam_info.ptrn_len += 2;

		_rtw_memcpy(target, dest_port, 2);
		target += 2;
		wowcam_info.ptrn_len += 2;

		/*padding*/
		target += 6;
		wowcam_info.ptrn_len += 6;

		_rtw_memcpy(target, conts[index].pconts, conts[index].conts_len);
		wowcam_info.ptrn_len += conts[index].conts_len;

		_rtw_memcpy(wowcam_info.mask, masks[index].pconts, masks[index].conts_len);

		rtw_wow_pattern_set(adapter, &wowcam_info, RTW_DEFAULT_PATTERN);
	}
}
#endif /*CONFIG_GOOGLE_CAST_WAKEUP*/

void rtw_set_default_pattern(_adapter *adapter)
{
	struct pwrctrl_priv *pwrpriv = adapter_to_pwrctl(adapter);
	struct mlme_ext_priv *pmlmeext = &adapter->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &pmlmeext->mlmext_info;
	struct rtw_wowcam_upd_info wowcam_info = {0};
	u8 index = 0;
	u8 multicast_addr[3] = {0x01, 0x00, 0x5e};
	u8 multicast_ip[4] = {0xe0, 0x28, 0x28, 0x2a};

	u8 unicast_mask[5] = {0x3f, 0x70, 0x80, 0xc0, 0x03};
	u8 icmpv6_mask[7] = {0x00, 0x70, 0x10, 0x00, 0xc0, 0xc0, 0x3f};
	u8 multicast_mask[5] = {0x07, 0x70, 0x80, 0xc0, 0x03};

	u8 ip_protocol[3] = {0x08, 0x00, 0x45};
	u8 ipv6_protocol[3] = {0x86, 0xdd, 0x60};

	u8 *target = NULL;

	/*TCP/ICMP unicast*/
	for (index = 0 ; index < DEFAULT_PATTERN_NUM ; index++) {
		_rtw_memset((void *)&wowcam_info, 0, sizeof(wowcam_info));

		switch (index) {
		case 0:
			target = wowcam_info.ptrn;
			_rtw_memcpy(target, adapter_mac_addr(adapter), ETH_ALEN);

			target += ETH_TYPE_OFFSET;
			_rtw_memcpy(target, &ip_protocol, sizeof(ip_protocol));

			/* TCP */
			target += (PROTOCOL_OFFSET - ETH_TYPE_OFFSET);
			_rtw_memset(target, 0x06, 1);

			target += (IP_OFFSET - PROTOCOL_OFFSET);

			_rtw_memcpy(target, pmlmeinfo->ip_addr, RTW_IP_ADDR_LEN);

			_rtw_memcpy(wowcam_info.mask,
				    &unicast_mask, sizeof(unicast_mask));

			wowcam_info.ptrn_len = IP_OFFSET + RTW_IP_ADDR_LEN;

			rtw_wow_pattern_set(adapter, &wowcam_info, RTW_DEFAULT_PATTERN);

			break;
		case 1:
			target = wowcam_info.ptrn;
			_rtw_memcpy(target, adapter_mac_addr(adapter), ETH_ALEN);

			target += ETH_TYPE_OFFSET;
			_rtw_memcpy(target, &ip_protocol, sizeof(ip_protocol));

			/* ICMP */
			target += (PROTOCOL_OFFSET - ETH_TYPE_OFFSET);
			_rtw_memset(target, 0x01, 1);

			target += (IP_OFFSET - PROTOCOL_OFFSET);
			_rtw_memcpy(target, pmlmeinfo->ip_addr, RTW_IP_ADDR_LEN);

			_rtw_memcpy(wowcam_info.mask,
				    &unicast_mask, sizeof(unicast_mask));

			wowcam_info.ptrn_len = IP_OFFSET + RTW_IP_ADDR_LEN;

			rtw_wow_pattern_set(adapter, &wowcam_info, RTW_DEFAULT_PATTERN);

			break;
#ifdef CONFIG_IPV6
		case 2:
			if (pwrpriv->wowlan_ns_offload_en == _TRUE) {
				target = wowcam_info.ptrn;
				target += ETH_TYPE_OFFSET;

				_rtw_memcpy(target, &ipv6_protocol,
					    sizeof(ipv6_protocol));

				/* ICMPv6 */
				target += (IPv6_PROTOCOL_OFFSET -
					   ETH_TYPE_OFFSET);
				_rtw_memset(target, 0x3a, 1);

				target += (IPv6_OFFSET - IPv6_PROTOCOL_OFFSET);
				_rtw_memcpy(target, pmlmeinfo->ip6_addr,
					    RTW_IPv6_ADDR_LEN);

				_rtw_memcpy(wowcam_info.mask,
					    &icmpv6_mask, sizeof(icmpv6_mask));

				wowcam_info.ptrn_len = IPv6_OFFSET + RTW_IPv6_ADDR_LEN;

				rtw_wow_pattern_set(adapter, &wowcam_info,
						    RTW_DEFAULT_PATTERN);
			}
			break;
#endif /*CONFIG_IPV6*/
		case 3:
			target = wowcam_info.ptrn;
			_rtw_memcpy(target, &multicast_addr, sizeof(multicast_addr));

			target += ETH_TYPE_OFFSET;
			_rtw_memcpy(target, &ip_protocol, sizeof(ip_protocol));

			/* UDP */
			target += (PROTOCOL_OFFSET - ETH_TYPE_OFFSET);
			_rtw_memset(target, 0x11, 1);

			target += (IP_OFFSET - PROTOCOL_OFFSET);
			_rtw_memcpy(target, &multicast_ip, sizeof(multicast_ip));

			_rtw_memcpy(wowcam_info.mask,
				    &multicast_mask, sizeof(multicast_mask));

			wowcam_info.ptrn_len = IP_OFFSET + sizeof(multicast_ip);

			rtw_wow_pattern_set(adapter, &wowcam_info, RTW_DEFAULT_PATTERN);

			break;
		default:
			break;
		}
	}

	return;
}

void rtw_dump_priv_pattern(_adapter *adapter, u8 idx)
{
	struct pwrctrl_priv *pwrctl = adapter_to_pwrctl(adapter);
	char str_1[128];
	char *p_str;
	u8 val8 = 0;
	int i = 0, j = 0, len = 0, max_len = 0;

	RTW_INFO("=========[%d]========\n", idx);

	RTW_INFO(">>>priv_pattern_content:\n");
	p_str = str_1;
	max_len = sizeof(str_1);
	for (i = 0 ; i < MAX_WKFM_PATTERN_SIZE / 8 ; i++) {
		_rtw_memset(p_str, 0, max_len);
		len = 0;
		for (j = 0 ; j < 8 ; j++) {
			val8 = pwrctl->patterns[idx].content[i * 8 + j];
			len += snprintf(p_str + len, max_len - len,
					"%02x ", val8);
		}
		RTW_INFO("%s\n", p_str);
	}

	RTW_INFO(">>>priv_pattern_mask:\n");
	for (i = 0 ; i < MAX_WKFM_SIZE / 8 ; i++) {
		_rtw_memset(p_str, 0, max_len);
		len = 0;
		for (j = 0 ; j < 8 ; j++) {
			val8 = pwrctl->patterns[idx].mask[i * 8 + j];
			len += snprintf(p_str + len, max_len - len,
					"%02x ", val8);
		}
		RTW_INFO("%s\n", p_str);
	}

	RTW_INFO(">>>priv_pattern_len:\n");
	RTW_INFO("%s: len: %d\n", __func__, pwrctl->patterns[idx].len);
}

void rtw_wow_pattern_sw_dump(_adapter *adapter)
{
	int i;

	RTW_INFO("********[RTK priv-patterns]*********\n");
	for (i = 0 ; i < MAX_WKFM_CAM_NUM; i++)
		rtw_dump_priv_pattern(adapter, i);
}

void rtw_get_sec_iv(_adapter *padapter, u8 *pcur_dot11txpn, u8 *StaAddr)
{
	struct sta_info		*psta;
	struct security_priv *psecpriv = &padapter->securitypriv;

	_rtw_memset(pcur_dot11txpn, 0, 8);
	if (NULL == StaAddr)
		return;
	psta = rtw_get_stainfo(&padapter->stapriv, StaAddr);
	RTW_INFO("%s(): StaAddr: %02x %02x %02x %02x %02x %02x\n",
		 __func__, StaAddr[0], StaAddr[1], StaAddr[2],
		 StaAddr[3], StaAddr[4], StaAddr[5]);

	if (psta) {
		if ((psecpriv->dot11PrivacyAlgrthm == _AES_) ||
		    (psecpriv->dot11PrivacyAlgrthm == _CCMP_256_))
			AES_IV(pcur_dot11txpn, psta->dot11txpn, 0);
		else if (psecpriv->dot11PrivacyAlgrthm == _TKIP_)
			TKIP_IV(pcur_dot11txpn, psta->dot11txpn, 0);
		else if ((psecpriv->dot11PrivacyAlgrthm == _GCMP_) ||
			 (psecpriv->dot11PrivacyAlgrthm == _GCMP_256_))
			GCMP_IV(pcur_dot11txpn, psta->dot11txpn, 0);
	}
}

void rtw_construct_remote_control_info(_adapter *adapter,
				       struct rtw_remote_wake_ctrl_info *ctrl_info)
{
	struct security_priv *securitypriv = &adapter->securitypriv;
	struct stainfo_rxcache *rxcache = NULL;
	struct sta_info *sta = NULL;
	u8 gtk_rx_iv[4][IV_LENGTH] = {0};
	u8 tid_id = 0;
	u8 i = 0;
	/* ToDo CONFIG_RTW_MLD: [currently primary link only] */
	struct _ADAPTER_LINK *adapter_link = GET_PRIMARY_LINK(adapter);
	struct link_security_priv *lsecuritypriv = &adapter_link->securitypriv;

	sta = rtw_get_stainfo(&adapter->stapriv, get_link_bssid(&adapter_link->mlmepriv));

	if (!sta) {
		rtw_warn_on(1);
		return;
	}

	rxcache = &sta->sta_recvpriv.rxcache;

	rtw_get_sec_iv(adapter, ctrl_info->ptk_tx_iv, get_link_bssid(&adapter_link->mlmepriv));
	RTW_INFO("[wow] ptk_tx_iv = " IV_FMT "\n", IV_ARG(ctrl_info->ptk_tx_iv));

	ctrl_info->valid_check = REMOTECTRL_INFO_VALID_CHECK;
	ctrl_info->symbol_check_en |= REMOTECTRL_INFO_SYMBOL_CHK_PTK |
				      REMOTECTRL_INFO_SYMBOL_CHK_GTK;

	ctrl_info->gtk_key_idx = lsecuritypriv->dot118021XGrpKeyid;
	RTW_INFO("[wow] gtk_key_idx = %d\n", ctrl_info->gtk_key_idx);

	tid_id = rxcache->last_tid;
	_rtw_memcpy(ctrl_info->ptk_rx_iv, rxcache->iv[tid_id], IV_LENGTH);
	RTW_INFO("[wow] ptk_rx_iv = " IV_FMT "\n", IV_ARG(ctrl_info->ptk_rx_iv));

	for (i = 0; i < 4; i++) {
		rtw_pn_to_iv(lsecuritypriv->iv_seq[i], gtk_rx_iv[i], i,
			     securitypriv->dot118021XGrpPrivacy);
		RTW_INFO("[wow] gtk_rx_iv[%u] = " IV_FMT "\n", i, IV_ARG(gtk_rx_iv[i]));
	}
	_rtw_memcpy(ctrl_info->gtk_rx_iv_idx0, gtk_rx_iv[0], IV_LENGTH);
	_rtw_memcpy(ctrl_info->gtk_rx_iv_idx1, gtk_rx_iv[1], IV_LENGTH);
	_rtw_memcpy(ctrl_info->gtk_rx_iv_idx2, gtk_rx_iv[2], IV_LENGTH);
	_rtw_memcpy(ctrl_info->gtk_rx_iv_idx3, gtk_rx_iv[3], IV_LENGTH);
}

void rtw_core_wow_handle_wake_up_rsn(void *drv_priv, u8 rsn)
{
	struct dvobj_priv *dvobj = (struct dvobj_priv *)drv_priv;
	struct wow_priv *wowpriv = dvobj_to_wowlan(dvobj);

	wowpriv->wow_wake_reason = rsn;
}

void rtw_core_wow_handle_wake_up_pattern_idx(void *drv_priv, u8 pattern_idx)
{
	struct dvobj_priv *dvobj = (struct dvobj_priv *)drv_priv;
	struct wow_priv *wowpriv = dvobj_to_wowlan(dvobj);

	wowpriv->wowlan_aoac_rpt.wow_pattern_idx = pattern_idx;
}

#ifdef CONFIG_MDNS_OFFLOAD
static struct rtw_mdns_resp_entry *
_rtw_get_mdns_resp_entry(_adapter *padapter, u8 index)
{
	struct wow_priv *wowpriv = adapter_to_wowlan(padapter);
	struct rtw_mdns_ofld_info *ofld_info = &wowpriv->mdns_ofld_info;
	struct rtw_mdns_resp_entry *resp_entry = NULL;

	if (index < MAX_MDNS_RESP_NUM)
		resp_entry = &ofld_info->resp_entry[index];

	return resp_entry;
}

int rtw_wow_add_mdns_resp(_adapter *padapter, u8 index, u8 *resp_content, u16 content_len)
{
	struct rtw_mdns_resp_entry *resp_entry = NULL;

	resp_entry = _rtw_get_mdns_resp_entry(padapter, index);
	if (resp_entry == NULL)
		return _FAIL;

	_rtw_memcpy(resp_entry->content, resp_content, content_len);
	resp_entry->content_len = content_len;

	return _SUCCESS;
}

int rtw_wow_del_mdns_resp(_adapter *padapter, u8 index)
{
	struct rtw_mdns_resp_entry *resp_entry = NULL;

	resp_entry = _rtw_get_mdns_resp_entry(padapter, index);
	if (resp_entry == NULL)
		return _FAIL;

	_rtw_memset(resp_entry, 0, sizeof(struct rtw_mdns_resp_entry));

	return _SUCCESS;
}

int rtw_wow_get_mdns_resp_ent(_adapter *padapter, u8 index, struct rtw_mdns_resp_entry **resp_entry)
{
	*resp_entry = _rtw_get_mdns_resp_entry(padapter, index);
	if (*resp_entry == NULL)
		return _FAIL;

	return _SUCCESS;
}

int rtw_wow_add_mdns_match_crit(_adapter *padapter, u8 index, u16 match_type, u16 name_offset, u16 name_len)
{
	struct rtw_mdns_resp_entry *resp_entry = NULL;
	struct rtw_mdns_match_criteria *match_ct = NULL;

	resp_entry = _rtw_get_mdns_resp_entry(padapter, index);
	if (resp_entry == NULL)
		return _FAIL;

	if (resp_entry->match_ct_num == MAX_MDNS_MATCH_CRITERIA_NUM)
		return _FAIL;

	match_ct = &resp_entry->match_ct[resp_entry->match_ct_num];

	match_ct->name_offset = name_offset;
	match_ct->type = match_type;
	match_ct->name_len = name_len;

	resp_entry->match_ct_num += 1;

	return _SUCCESS;
}

int rtw_wow_del_mdns_match_crit(_adapter *padapter, u8 index)
{
	struct rtw_mdns_resp_entry *resp_entry = NULL;
	struct rtw_mdns_match_criteria *match_ct = NULL;

	resp_entry = _rtw_get_mdns_resp_entry(padapter, index);
	if (resp_entry == NULL)
		return _FAIL;

	resp_entry->match_ct_num = 0;
	_rtw_memset(resp_entry->match_ct, 0, sizeof(resp_entry->match_ct));

	return _SUCCESS;
}

int rtw_wow_add_mdns_passthru_name(_adapter *padapter, u8 *name, u8 name_len)
{
	struct wow_priv *wowpriv = adapter_to_wowlan(padapter);
	struct rtw_mdns_ofld_info *ofld_info = &wowpriv->mdns_ofld_info;
	struct rtw_mdns_passthru_list *passthru_list = &ofld_info->passthru_list;
	struct rtw_mdns_passthru_name *passthru_name = NULL;

	if (passthru_list->passthru_name_num == MAX_MDNS_PASSTHRU_NAME_NUM)
		return _FAIL;

	passthru_name = &passthru_list->passthru_name[passthru_list->passthru_name_num];
	_rtw_memcpy(passthru_name->name, name, name_len);
	passthru_name->name_len = name_len;

	passthru_list->passthru_name_num += 1;

	return _SUCCESS;
}

void rtw_wow_clr_mdns_passthru_name(_adapter *padapter)
{
	struct wow_priv *wowpriv = adapter_to_wowlan(padapter);
	struct rtw_mdns_ofld_info *ofld_info = &wowpriv->mdns_ofld_info;
	struct rtw_mdns_passthru_list *passthru_list = &ofld_info->passthru_list;

	passthru_list->passthru_name_num = 0;
	_rtw_memset(passthru_list->passthru_name, 0,
		    sizeof(struct rtw_mdns_passthru_name) * MAX_MDNS_PASSTHRU_NAME_NUM);
}

void rtw_wow_get_mdns_passthru_list(_adapter *padapter, struct rtw_mdns_passthru_list **passthru_list)
{
	struct wow_priv *wowpriv = adapter_to_wowlan(padapter);
	struct rtw_mdns_ofld_info *ofld_info = &wowpriv->mdns_ofld_info;

	*passthru_list = &ofld_info->passthru_list;
}
#endif /* CONFIG_MDNS_OFFLOAD */

#ifdef CONFIG_APF
static inline bool _should_cmd_executed(enum rtw_screen_mode _mode, enum mlme_state _state,
						enum phl_apf_cmd _cmd, struct phl_apf_info *_info)
{
	bool _is_cmd_executed = true;
	if (_state == MLME_LINKED) {
		if (_cmd == PHL_APF_CMD_DISABLE) {
			_is_cmd_executed = false;
		} else if (_cmd == PHL_APF_CMD_ENABLE &&
			   (_info->ram.prog_len == 0 || _mode == SCREEN_ON)) {
			_is_cmd_executed = false;
		} else if (_cmd >= PHL_APF_CMD_DRV) {
			_is_cmd_executed = false;
		}
	}

#ifdef CONFIG_APF_DBG
	if (_is_cmd_executed == false && PHL_APF_CMD_CHECK != _cmd)
		RTW_INFO("[APFDBG] Should not execute apf cmd(%s) screen(%s) mstate(%u)\n",
			  _apf_cmd_to_str(_cmd), _mode == SCREEN_OFF ? "OFF" : "ON", _state);
#endif
	return _is_cmd_executed;
}

static inline bool _should_change_ampdu(enum phl_apf_cmd _cmd, struct phl_apf_info *_info)
{
	bool _chg_ampdu = false;
	if (_info->fw_mode == RTW_FW_NIC) {
		if (PHL_APF_CMD_ENABLE == _cmd) {
			if (PHL_APF_STANDBY == _info->apf_state || PHL_APF_DISABLE == _info->apf_state)
				_chg_ampdu = true;
		} else if (PHL_APF_CMD_STANDBY == _cmd) {
			if (PHL_APF_ENABLE == _info->apf_state)
				_chg_ampdu = true;
		}
	}
	return _chg_ampdu;
}
void rtw_init_apf(struct _ADAPTER *padapter)
{
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);
	void *phl = GET_PHL_INFO(dvobj);
	struct rtw_phl_com_t *phl_com = dvobj->phl_com;
	struct dev_cap_t *dev_cap = &phl_com->dev_cap;
	struct wow_priv *wowpriv = adapter_to_wowlan(padapter);
	struct phl_apf_info *info = &(wowpriv->apf_info);

	/* init wowpriv apf related parameter & apf_info */
	wowpriv->screen_mode = SCREEN_ON;
	wowpriv->mstate = MLME_NO_LINK;
	wowpriv->apf_active_tp_th = APF_ACTIVE_TP_TH;
	wowpriv->dump_apf_ram = 0;

	info->apf_cmd = PHL_APF_CMD_NONE;
	info->apf_state = PHL_APF_DISABLE;
	info->fw_mode = RTW_FW_NIC;
	info->macid = 0;
	info->apf_force_dis = PHL_APF_FORCE_IGNORE;

	/* pkt ofld related */
	info->ofld_idx = 0;
	info->is_ram_ofld = 0;
	info->ram_frag_num = 0;
	info->mac_chg = 0;
	_rtw_memset(info->apf_ram_frag_pktid, 0xFF, 8);
	info->ram.prog_len = 0;
	_rtw_memset(info->ram.buf, 0, MAX_APF_RAM_SIZE);

	/* security info for mac hdr */
	info->pairwise_sec_algo = _NO_PRIVACY_;
	info->group_sec_algo = _NO_PRIVACY_;
	info->hw_sec_iv = dev_cap->sec_cap.hw_sec_iv;
#ifdef CONFIG_APF_DBG
	info->apf_cmd_seq = 0;
#endif
	info->wait_start_time = 0;
	info->wait_period = 0;
	/* hook phl & halmac */
	rtw_phl_init_apf(phl, info);
}

void rtw_apf_cmd_hdl(struct _ADAPTER *padapter, enum phl_apf_cmd cmd)
{
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);
	void *phl = GET_PHL_INFO(dvobj);
	struct _ADAPTER_LINK *alink = GET_PRIMARY_LINK(padapter);
	struct rtw_phl_stainfo_t *phl_sta = NULL;
	struct security_priv *securitypriv = &padapter->securitypriv;
	enum mlme_state mstate;

	struct wow_priv *wowpriv = adapter_to_wowlan(padapter);
	struct phl_apf_info *info = &wowpriv->apf_info;
	struct phl_apf_cmd_blob *cmd_blob = NULL;
	u8 cmd_type = PHL_CMD_WAIT;
	u32 screen_mode = (u32)wowpriv->screen_mode;
	u32 apf_active_tp_th = wowpriv->apf_active_tp_th;
	u32 cur_tx_tp = dvobj->traffic_stat.cur_tx_tp;
	u32 cur_rx_tp = dvobj->traffic_stat.cur_rx_tp;
	u8 enable_ampdu = 1;

	if (!is_primary_adapter(padapter) || !MLME_IS_STA(padapter)) {
		RTW_INFO("%s() APF is used for primary adapter & STA only\n", __func__);
		return;
	}

	if (padapter->netif_up && dvobj->processing_dev_remove == _FALSE) {
		phl_sta = rtw_phl_get_stainfo_self(phl, alink->wrlink);
		if (!phl_sta) {
			RTW_ERR("%s() netif_up but phl_sta is NULL\n", __func__);
			return;
		}
		mstate = phl_sta->rlink->mstate;
		if (mstate != MLME_LINKED && cmd != PHL_APF_CMD_MEDIA_STATUS_CHG) {
			if (cmd != PHL_APF_CMD_CHECK)
				RTW_INFO("[APFDBG]%s() !!NOT CONNECTED!! manipulate APF while connected!!\n", __func__);

			return;
		}
	} else {
		return;
	}

#ifdef CONFIG_APF_DBG
	if (PHL_APF_CMD_CHECK != cmd)
		RTW_INFO("[APFDBG] %s() apf_cmd(%s) new_cmd(%s)\n",
			 __func__, _apf_cmd_to_str(info->apf_cmd), _apf_cmd_to_str(cmd));
#endif
	if (PHL_APF_CMD_CHECK == cmd && info->wait_period != 0) {
		if (info->wait_start_time == 0) {
			info->wait_start_time = rtw_get_current_time();
			return;
		} else {
			if (rtw_get_passing_time_ms(info->wait_start_time) < info->wait_period) {
				RTW_INFO("%s() Wait Period\n", __func__);
				return;
			} else {
				info->wait_start_time = 0;
				info->wait_period = 0;
			}
		}
	}
	if (info->apf_force_dis == PHL_APF_FORCE_DIS &&
		(cmd == PHL_APF_CMD_ENABLE || cmd == PHL_APF_CMD_CHECK)) {
		RTW_INFO("%s() FORCE DISABLE APF\n", __func__);
		if (info->apf_state == PHL_APF_STANDBY)
			return;
		cmd = PHL_APF_CMD_STANDBY;
	}


	if (PHL_APF_CMD_MEDIA_STATUS_CHG == cmd) {
		RTW_INFO("%s() Media Status Change\n", __func__);
		wowpriv->mstate = mstate;
		info->macid = phl_sta->macid;
		info->mac_chg = 1;
		cmd_type = PHL_CMD_DIRECTLY;
		if (mstate == MLME_LINKED) {
			cmd = PHL_APF_CMD_STANDBY;
			info->pairwise_sec_algo = rtw_sec_algo_drv2phl(securitypriv->dot11PrivacyAlgrthm);
			info->group_sec_algo = rtw_sec_algo_drv2phl(securitypriv->dot118021XGrpPrivacy);
			info->protect_bit = (info->pairwise_sec_algo == RTW_ENC_NONE ? 0 : 1);
			info->key_idx = (u8)securitypriv->dot11PrivacyKeyIndex;
			_rtw_memcpy(info->a1, phl_sta->mac_addr, MAC_ADDRESS_LENGTH);
			_rtw_memcpy(info->a2, phl_sta->wrole->mac_addr, MAC_ADDRESS_LENGTH);
			_rtw_memcpy( info->a3, phl_sta->mac_addr, MAC_ADDRESS_LENGTH);
		} else {
			cmd = PHL_APF_CMD_DISABLE;
			info->pairwise_sec_algo = rtw_sec_algo_drv2phl(_NO_PRIVACY_);
			info->group_sec_algo = rtw_sec_algo_drv2phl(_NO_PRIVACY_);
			info->protect_bit = 0;
			info->key_idx = 0;
			_rtw_memset(info->a1, 0xFF, MAC_ADDRESS_LENGTH);
			_rtw_memset(info->a2, 0xFF, MAC_ADDRESS_LENGTH);
			_rtw_memset(info->a3, 0xFF, MAC_ADDRESS_LENGTH);
			info->ram.prog_len = 0;
			_rtw_memset(info->ram.buf, 0, MAX_APF_RAM_SIZE);
		}
#ifdef CONFIG_APF_DBG
		RTW_INFO("[APFDBG] %s() mstate(%u) pairwise_sec(%u) group_sec(%u) key_idx(%u)\n",
			 __func__, wowpriv->mstate, info->pairwise_sec_algo,
			 info->group_sec_algo, info->key_idx);
#endif
		goto manipulate_apf;
	}

	if (PHL_APF_CMD_CHECK == cmd && info->ram.prog_len != 0) {
		cmd_type = PHL_CMD_DIRECTLY;
		if (screen_mode == SCREEN_OFF) {
			/* check TP and apf_state */
			if (cur_rx_tp + cur_tx_tp > apf_active_tp_th) {
				if (IS_STATE(info->apf_state, PHL_APF_ENABLE)) {
					RTW_INFO("%s() Traffic is not idle, standby APF\n", __func__);
					cmd = PHL_APF_CMD_STANDBY;
				}

			} else {
				if (info->apf_state == PHL_APF_STANDBY && info->ram.prog_len != 0) {
					RTW_INFO("%s() Traffic is idle, enable APF\n", __func__);
					cmd = PHL_APF_CMD_ENABLE;
				}
			}
		} else {
			if (IS_STATE(info->apf_state, PHL_APF_ENABLE)) {
				cmd = PHL_APF_CMD_STANDBY;
			}

		}
		/* if no need to switch apf enable/standby */
		if (PHL_APF_CMD_CHECK == cmd)
			return;

		goto manipulate_apf;
	}

	if (!_should_cmd_executed(screen_mode, mstate, cmd, info))
		return;

manipulate_apf:
#ifdef CONFIG_APF_DBG
	info->apf_cmd_seq++;
	RTW_INFO("[APFDBG] %s() cmd seq apf_cmd(%s) seq(%llu)\n",
		 __func__, _apf_cmd_to_str(cmd), info->apf_cmd_seq);
#endif
	cmd_blob = rtw_malloc(sizeof(struct phl_apf_cmd_blob));
	if (!cmd_blob) {
		RTW_ERR("%s() allocate memory for apf_cmd_blob fail\n", __func__);
		return;
	}

	/* APF cannot handle amsdu pkt, disable ampdu before enable APF */
	if (_should_change_ampdu(cmd, info)) {
		RTW_INFO("%s() %s AMPDU\n", __func__,
			 PHL_APF_CMD_ENABLE == cmd ? "Disable" : "Auto");
		enable_ampdu = PHL_APF_CMD_ENABLE == cmd ? 0 : RX_AMPDU_ACCEPT_INVALID;
		if (rtw_rx_ampdu_set_accept(padapter, enable_ampdu, RX_AMPDU_DRV_FIXED)) {
			RTW_INFO("Change AMPDU \n");
			rtw_rx_ampdu_apply(padapter);
		}
	}
	cmd_blob->cmd = cmd;
	rtw_phl_manipulate_apf_cmd(phl, cmd_blob, cmd_type);
}

#endif /* CONFIG_APF */

#ifdef CONFIG_GTK_OL
void _update_aoac_rpt_phase_0(_adapter *adapter, struct rtw_aoac_report *aoac_info)
{
	struct security_priv *securitypriv = &adapter->securitypriv;
	struct stainfo_rxcache *rxcache = NULL;
	struct sta_info *sta = NULL;
	u8 pn[8] = {0};
	u8 gtk_key_idx = 0;
	u8 i = 0;
	/* ToDo CONFIG_RTW_MLD: [currently primary link only] */
	struct _ADAPTER_LINK *adapter_link = GET_PRIMARY_LINK(adapter);
	struct link_security_priv *lsecuritypriv = &adapter_link->securitypriv;

	/* handle ptk rx iv */
	/* This Rx IV has no effect, the driver does not drop unicast packets
	 * due to different PNs. (Refer to recv_ucast_pn_decache)
	 */
	sta = rtw_get_stainfo(&adapter->stapriv, get_bssid(&adapter->mlmepriv));
	if (sta) {
		if (rtw_iv_to_pn(aoac_info->ptk_rx_iv, pn, NULL,
				 securitypriv->dot11PrivacyAlgrthm)) {
			rxcache = &sta->sta_recvpriv.rxcache;
			for (i = 0; i < TID_NUM; i++)
				_rtw_memcpy(rxcache->iv[i], aoac_info->ptk_rx_iv,
					    IV_LENGTH);
			sta->dot11rxpn.val = RTW_GET_LE64(pn);
			RTW_INFO("[wow] ptk_rx_pn = " PN_FMT "\n", PN_ARG(pn));
		}
	}

	/* handle gtk rx iv */
	gtk_key_idx = aoac_info->key_idx;
	if (rtw_iv_to_pn(aoac_info->gtk_rx_iv[gtk_key_idx], pn, NULL,
			 securitypriv->dot118021XGrpPrivacy)) {
		_rtw_memcpy(lsecuritypriv->iv_seq[gtk_key_idx], pn, 8);
		RTW_INFO("[wow] gtk_rx_pn[%u] = " PN_FMT "\n", gtk_key_idx, PN_ARG(pn));
	}

#ifdef CONFIG_IEEE80211W
	/* handle igtk rx ipn */
	if (SEC_IS_BIP_KEY_INSTALLED(lsecuritypriv)) {
		lsecuritypriv->dot11wBIPrxpn.val = RTW_GET_LE64(aoac_info->igtk_ipn);
		RTW_INFO("[wow] igtk_rx_pn = " PN_FMT "\n", PN_ARG(aoac_info->igtk_ipn));
	}
#endif
}

void _update_aoac_rpt_phase_1(_adapter *adapter, struct rtw_aoac_report *aoac_info)
{
	struct security_priv *securitypriv = &adapter->securitypriv;
	struct sta_info *sta = NULL;
	u8 pn[8] = {0};
	u8 gtk_key_idx = 0;
	u8 key_len = 0;
	u8 i = 0;
	/* ToDo CONFIG_RTW_MLD: [currently primary link only] */
	struct _ADAPTER_LINK *adapter_link = GET_PRIMARY_LINK(adapter);
	struct link_security_priv *lsecuritypriv = &adapter_link->securitypriv;

	/* handle ptk tx iv */
	sta = rtw_get_stainfo(&adapter->stapriv, get_link_bssid(&adapter_link->mlmepriv));
	if (sta) {
		if (rtw_iv_to_pn(aoac_info->ptk_tx_iv, pn, NULL,
				 securitypriv->dot11PrivacyAlgrthm)) {
			sta->dot11txpn.val = RTW_GET_LE64(pn);
			RTW_INFO("[wow] ptk_tx_pn = " PN_FMT "\n", PN_ARG(pn));
		}
	}

	if (aoac_info->rekey_ok) {
		/* update gtk key */
		gtk_key_idx = aoac_info->key_idx;
		lsecuritypriv->dot118021XGrpKeyid = gtk_key_idx;

		switch (securitypriv->dot118021XGrpPrivacy) {
		case _TKIP_:
		case _AES_:
		case _GCMP_:
			key_len = 16;
			break;
		case _GCMP_256_:
		case _CCMP_256_:
			key_len = 32;
			break;
		default:
			key_len = 0;
		}

		if (key_len)
			_rtw_memcpy(lsecuritypriv->dot118021XGrpKey[gtk_key_idx].skey,
					aoac_info->gtk, key_len);
		/* update tkip dot118021XGrptxmickey dot118021XGrprxmickey */
		if (securitypriv->dot118021XGrpPrivacy == _TKIP_) {
			/* The order of the GTK Tx/Rx mic keys in the AOAC report is
			 * reversed compared to the GTK Tx/Rx mic keys provided by
			 * wpa_supplicant.
			 */
			_rtw_memcpy(lsecuritypriv->dot118021XGrptxmickey[gtk_key_idx].skey,
					&aoac_info->gtk[24], 8);
			_rtw_memcpy(lsecuritypriv->dot118021XGrprxmickey[gtk_key_idx].skey,
					&aoac_info->gtk[16], 8);
		}
		rtw_set_key(adapter, adapter_link, gtk_key_idx, 1, _TRUE);

		/* update eapol replay_counter */
		_rtw_memcpy(sta->replay_ctr, aoac_info->eapol_key_replay_count,
			    RTW_REPLAY_CTR_LEN);
#ifdef CONFIG_IEEE80211W
		if (SEC_IS_BIP_KEY_INSTALLED(lsecuritypriv)) {
			switch (securitypriv->dot11wCipher) {
			case _BIP_CMAC_128_:
			case _BIP_GMAC_128_:
				key_len = 16;
				break;
			case _BIP_CMAC_256_:
			case _BIP_GMAC_256_:
				key_len = 32;
				break;
			default:
				key_len = 0;
			}

			/*
			 * The Rx IPN has already been updated in
			 * _update_aoac_rpt_phase_0(), so no update is done here
			 * unless WoWLAN FW updates the Rx IPN before sending
			 * out the phase1 AOAC report.
			 */
			lsecuritypriv->dot11wBIPKeyid = RTW_GET_LE32(aoac_info->igtk_key_id);
			_rtw_memcpy(lsecuritypriv->dot11wBIPKey[lsecuritypriv->dot11wBIPKeyid].skey,
				    &aoac_info->igtk, key_len);
		}
#endif
	} else {
		RTW_INFO("[wow] no rekey event\n");
	}

	for (i = 0; i < 4; i++) {
		if (rtw_iv_to_pn(aoac_info->gtk_rx_iv[i], pn, NULL,
				 securitypriv->dot118021XGrpPrivacy)) {
			_rtw_memcpy(lsecuritypriv->iv_seq[i], pn, 8);
			RTW_INFO("[wow] gtk_rx_pn[%u] = " PN_FMT "\n", i, PN_ARG(pn));
		}
	}
}

void rtw_update_gtk_ofld_info(void *drv_priv, struct rtw_aoac_report *aoac_info,
			      u8 aoac_report_get_ok, u8 rx_ready)
{
	struct dvobj_priv *dvobj = (struct dvobj_priv *)drv_priv;
	_adapter *adapter = dvobj_get_primary_adapter(dvobj);

	if (!rx_ready)
		_update_aoac_rpt_phase_0(adapter, aoac_info);
	else
		_update_aoac_rpt_phase_1(adapter, aoac_info);
}
#endif /* CONFIG_GTK_OL */

#ifdef CONFIG_WRC_WOW_MAGIC
u8 rtw_cfg_wrc_wol_magic(_adapter *padapter, u8 enable)
{
	struct registry_priv *registry_par = &padapter->registrypriv;
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);
	u8 iface_num = GET_IFACE_NUMS(padapter);
	_adapter *padapter_wrc = NULL;
	struct _ADAPTER_LINK *padapter_wrc_link = NULL;
	enum rtw_phl_status pstatus;
	u8 ret = _FAIL;
	u8 i;

	if (!registry_par->wakeup_event & BIT(0)) {
		RTW_INFO("[wow][wrc] no magic packet wake up\n");
		goto exit;
	}

	for (i = 0; i < iface_num; i++) {
		if (i != padapter->iface_id)
			padapter_wrc = GET_ADAPTER(padapter, i);
	}

	if (padapter_wrc == NULL) {
		RTW_ERR("[wow][wrc] no avaliable interface\n");
		goto exit;
	}

	/* ToDo CONFIG_RTW_MLD: currently primary link only */
	padapter_wrc_link = GET_PRIMARY_LINK(padapter_wrc);

	if (enable) {
		struct rtw_phl_stainfo_t *phl_sta;
		enum role_type rtype = PHL_RTYPE_AP;

		pstatus = rtw_phl_cmd_wrole_change(GET_PHL_INFO(dvobj),
						   padapter_wrc->phl_role,
						   padapter_wrc_link->wrlink,
						   WR_CHG_MADDR,
						   padapter->mac_addr,
						   MACADDRLEN,
						   PHL_CMD_DIRECTLY, 0);
		if (pstatus != RTW_PHL_STATUS_SUCCESS) {
			RTW_ERR("[wow][wrc] change role mac failed\n");
			goto exit;
		}

		phl_sta = rtw_phl_get_stainfo_self(GET_PHL_INFO(dvobj),
						   padapter_wrc_link->wrlink);
		if (!phl_sta) {
			RTW_ERR("[wow][wrc] change role type failed\n");
			goto exit;
		}

		phl_sta->wol_magic = _TRUE;
		phl_sta->hit_rule = _TRUE;

		pstatus = rtw_phl_cmd_wrole_change(GET_PHL_INFO(dvobj),
						   padapter_wrc->phl_role,
						   padapter_wrc_link->wrlink,
						   WR_CHG_TYPE, (u8*)&rtype,
						   sizeof(enum role_type),
						   PHL_CMD_DIRECTLY, 0);
		if (pstatus != RTW_PHL_STATUS_SUCCESS) {
			RTW_ERR("[wow][wrc] change role type failed\n");
			goto exit;
		}
	} else {
		u8 role_id;
		u8 *padapter_wrc_link_mac_addr[RTW_RLINK_MAX] = {NULL};

		rtw_phl_wifi_role_free(GET_PHL_INFO(dvobj),
				       padapter_wrc->phl_role->id);
		padapter_wrc->phl_role = NULL;

		rtw_collect_adapter_link_mac_addr(padapter_wrc,
						  padapter_wrc_link_mac_addr);

		role_id = rtw_phl_wifi_role_alloc(GET_PHL_INFO(dvobj),
						  padapter_wrc->mac_addr,
						  padapter_wrc_link_mac_addr,
						  PHL_RTYPE_STATION,
						  padapter_wrc->iface_id,
						  &padapter_wrc->phl_role,
						  DTYPE,
						  _FALSE);
		if (role_id == INVALID_WIFI_ROLE_IDX ||
		    padapter_wrc->phl_role == NULL) {
			RTW_ERR("[wow][wrc] rtw_phl_wifi_role_alloc failed\n");
			rtw_warn_on(1);
			goto exit;
		}
	}

	ret = _SUCCESS;

exit:
	return ret;
}
#endif /* CONFIG_WRC_WOW_MAGIC */
#endif /* CONFIG_WOWLAN */

#ifdef CONFIG_PNO_SUPPORT
#ifdef CONFIG_PNO_SECURITY_OFFLOAD
static void nlo_parse_cipher_list(struct rtw_nlo_info *wow_nlo, char *list_str)
{
	char *pch;
	char *pnext;
	char *pend;
	u8 index = 0;

	pch = list_str;
	while (strlen(pch) != 0) {
		pnext = strstr(pch, "key_mgmt=");
		if (pnext == NULL)
			break;

		pch = pnext + strlen(CIPHER_IE);
		pend = strstr(pch, "}");
		if (strncmp(pch, CIPHER_NONE, strlen(CIPHER_NONE)) == 0) {
			wow_nlo->chipertype[index] = NLO_CIPHER_OPEN;
		} else if (strncmp(pch, CIPHER_WPA_PSK, strlen(CIPHER_WPA_PSK)) == 0) {
			wow_nlo->chipertype[index] = NLO_CIPHER_WPA_TKIP |
						     NLO_CIPHER_WPA_AES |
						     NLO_CIPHER_WPA2_TKIP |
						     NLO_CIPHER_WPA2_AES;
		} else if (strncmp(pch, CIPHER_WPA_EAP, strlen(CIPHER_WPA_EAP)) == 0) {
			wow_nlo->chipertype[index] = NLO_CIPHER_WEP;
		}

		index++;
		pch = pend + 1;
	}
}

static void nlo_security_init(struct rtw_nlo_info *wow_nlo)
{
	struct file *fp;
	mm_segment_t fs;
	loff_t pos = 0;
	u8 *source = NULL;
	long len = 0;

	fp = filp_open("/data/misc/wifi/wpa_supplicant.conf", O_RDONLY,  0644);
	if (IS_ERR(fp)) {
		RTW_INFO("Error, wpa_supplicant.conf doesn't exist.\n");
		RTW_INFO("Error, cipher array using default value.\n");
		return;
	} else {
		RTW_INFO("Open wpa_supplicant.conf successfully.\n");
	}

	len = i_size_read(fp->f_path.dentry->d_inode);
	if (len < 0 || len > 2048) {
		RTW_INFO("Error, file size is bigger than 2048.\n");
		RTW_INFO("Error, cipher array using default value.\n");
		return;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);

	source = rtw_zmalloc(2048);

	if (source != NULL) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
		len = kernel_read(fp, source, len, &pos);
#else
		len = vfs_read(fp, source, len, &pos);
#endif
		nlo_parse_cipher_list(wow_nlo, source);
		rtw_mfree(source, 2048);
	}

	wow_nlo->compare_cipher_type = _TRUE;

	set_fs(fs);
	filp_close(fp, NULL);
}
#endif

static void nlo_scan_ch_init(struct rtw_nlo_info *wow_nlo,
			     struct ieee80211_channel **channels,
			     u32 n_channels)
{
	u8 i = 0;

	while (i < MAX_NLO_CHANNEL && i < n_channels) {
		wow_nlo->channel_list[i].bw = CHANNEL_WIDTH_20;
		wow_nlo->channel_list[i].center_chan = channels[i]->hw_value;
		wow_nlo->channel_list[i].chan = channels[i]->hw_value;
		wow_nlo->channel_list[i].period = 100;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
		if (channels[i]->flags & IEEE80211_CHAN_PASSIVE_SCAN)
#else
		if (channels[i]->flags & IEEE80211_CHAN_NO_IR)
#endif
			wow_nlo->channel_list[i].tx_pkt = 0;
		else
			wow_nlo->channel_list[i].tx_pkt = 1;
		wow_nlo->channel_list[i].tx_data_pause = 1;
		i++;
	}
	wow_nlo->channel_num = i;
}

static void nlo_ssid_init(struct rtw_nlo_info *wow_nlo,
			  struct cfg80211_ssid *ssids, int n_ssids)
{
	u8 i = 0;

	while (i < MAX_NLO_NUM && i < n_ssids) {
		_rtw_memcpy((void *)wow_nlo->ssid[i], (void *)ssids[i].ssid,
			    ssids[i].ssid_len);
		wow_nlo->ssidlen[i] = ssids[i].ssid_len;
		i++;
	}

	wow_nlo->num_of_networks = i;
	wow_nlo->num_of_hidden_ap = i;
}

int rtw_nlo_enable(struct net_device *net, struct cfg80211_ssid *ssids,
		   int n_ssids, struct ieee80211_channel **channels,
		   u32 n_channels, u32 delay, u32 interval, u32 iterations,
		   u32 slow_interval)
{
	_adapter *padapter = (_adapter *)rtw_netdev_priv(net);
	struct wow_priv *wowpriv = adapter_to_wowlan(padapter);
	struct rtw_nlo_info *wow_nlo = &wowpriv->wow_nlo;

	_rtw_memset((void *)wow_nlo, 0, sizeof(struct rtw_nlo_info));

	wow_nlo->nlo_en = _TRUE;

	nlo_scan_ch_init(wow_nlo, channels, n_channels);
	nlo_ssid_init(wow_nlo, ssids, n_ssids);
#ifdef CONFIG_PNO_SECURITY_OFFLOAD
	nlo_security_init(wow_nlo);
#endif

	wow_nlo->delay = delay * 1000;
	wow_nlo->period = interval * 1000;
	wow_nlo->cycle = iterations;
	wow_nlo->slow_period = slow_interval * 1000;
	wow_nlo->construct_pbreq = NULL;;

	rtw_nlo_debug(net);

	return 0;
}

int rtw_nlo_disable(struct net_device *net)
{
	_adapter *padapter = (_adapter *)rtw_netdev_priv(net);
	struct wow_priv *wowpriv = adapter_to_wowlan(padapter);
	struct rtw_nlo_info *wow_nlo = &wowpriv->wow_nlo;

	wow_nlo->nlo_en = _FALSE;

	return 0;
}

void rtw_nlo_debug(struct net_device *net)
{
	_adapter *padapter = (_adapter *)rtw_netdev_priv(net);
	struct wow_priv *wowpriv = adapter_to_wowlan(padapter);
	struct rtw_nlo_info *wow_nlo = &wowpriv->wow_nlo;
	struct pwrctrl_priv *pwrctl = adapter_to_pwrctl(padapter);
	int i;

	RTW_INFO("********NLO_INFO********\n");
	RTW_INFO("compare_cipher_type: %d\n", wow_nlo->compare_cipher_type);
	RTW_INFO("ssid_num: %d\n", wow_nlo->num_of_networks);
	for (i = 0; i < wow_nlo->num_of_networks; i++) {
		RTW_INFO("%d SSID (%s) length (%d) cipher (%#x)\n",
			 i, wow_nlo->ssid[i], wow_nlo->ssidlen[i],
			 wow_nlo->chipertype[i]);
	}
	RTW_INFO("delay: %d\n", wow_nlo->delay);
	RTW_INFO("fast_scan_iterations: %d\n", wow_nlo->cycle);
	RTW_INFO("fast_scan_period: %d\n", wow_nlo->period);
	RTW_INFO("slow_scan_period: %d\n", wow_nlo->slow_period);

	RTW_INFO("********SCAN_INFO*******\n");
	RTW_INFO("ch_num: %d\n", wow_nlo->channel_num);
	for (i = 0 ;i < wow_nlo->channel_num; i++) {
		RTW_INFO("[%02d] avtive:%d, timeout:%d, ch:%02d\n",
			 i, wow_nlo->channel_list[i].tx_pkt,
			 wow_nlo->channel_list[i].period,
			 wow_nlo->channel_list[i].chan);
	}
	RTW_INFO("************************\n");
}
#endif /* CONFIG_PNO_SUPPORT */

