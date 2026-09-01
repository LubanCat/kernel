/******************************************************************************
 *
 * Copyright(c) 2007 - 2021 Realtek Corporation.
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
#define _RTW_PWRCTRL_C_

#include <drv_types.h>
#ifdef CONFIG_RTW_IPS
bool rtw_core_set_ips_state(void *drv_priv, enum rtw_rf_state state)
{
        struct dvobj_priv *dvobj = (struct dvobj_priv *)drv_priv;
        enum rtw_phl_status pstatus = RTW_PHL_STATUS_FAILURE;

        if (state == RTW_RF_ON) {
                pstatus = rtw_phl_rf_on(dvobj->phl);
        } else if (state == RTW_RF_OFF) {
                pstatus = rtw_phl_rf_off(dvobj->phl);
        }

        if (RTW_PHL_STATUS_SUCCESS == pstatus)
                return true;
        else
                return false;
}
#endif

#ifdef CONFIG_RESUME_IN_WORKQUEUE
	static void resume_workitem_callback(struct work_struct *work);
#endif /* CONFIG_RESUME_IN_WORKQUEUE */

void rtw_init_pwrctrl_priv(_adapter *padapter)
{
	struct pwrctrl_priv *pwrctrlpriv = adapter_to_pwrctl(padapter);

#if defined(CONFIG_CONCURRENT_MODE)
	if (!is_primary_adapter(padapter))
		return;
#endif
	pwrctrlpriv->bInSuspend = _FALSE;
	pwrctrlpriv->bkeepfwalive = _FALSE;
#ifdef CONFIG_RESUME_IN_WORKQUEUE
	_init_workitem(&pwrctrlpriv->resume_work, resume_workitem_callback, NULL);
	pwrctrlpriv->rtw_workqueue = create_singlethread_workqueue("rtw_workqueue");
#endif /* CONFIG_RESUME_IN_WORKQUEUE */
}


void rtw_free_pwrctrl_priv(_adapter *adapter)
{
	struct pwrctrl_priv *pwrctrlpriv = adapter_to_pwrctl(adapter);

#if defined(CONFIG_CONCURRENT_MODE)
	if (!is_primary_adapter(adapter))
		return;
#endif


	/* _rtw_memset((unsigned char *)pwrctrlpriv, 0, sizeof(struct pwrctrl_priv)); */


#ifdef CONFIG_RESUME_IN_WORKQUEUE
	if (pwrctrlpriv->rtw_workqueue) {
		flush_workqueue(pwrctrlpriv->rtw_workqueue);
		destroy_workqueue(pwrctrlpriv->rtw_workqueue);
	}
#endif
}

#ifdef CONFIG_RESUME_IN_WORKQUEUE
static void resume_workitem_callback(struct work_struct *work)
{
	struct pwrctrl_priv *pwrpriv = container_of(work, struct pwrctrl_priv, resume_work);
	struct dvobj_priv *dvobj = pwrctl_to_dvobj(pwrpriv);
	_adapter *adapter = dvobj_get_primary_adapter(dvobj);

	RTW_INFO("%s\n", __FUNCTION__);

	rtw_resume_process(adapter);

	rtw_resume_unlock_suspend();
}

void rtw_resume_in_workqueue(struct pwrctrl_priv *pwrpriv)
{
	/* accquire system's suspend lock preventing from falliing asleep while resume in workqueue */
	/* rtw_lock_suspend(); */

	rtw_resume_lock_suspend();

#if 1
	queue_work(pwrpriv->rtw_workqueue, &pwrpriv->resume_work);
#else
	_set_workitem(&pwrpriv->resume_work);
#endif
}
#endif /* CONFIG_RESUME_IN_WORKQUEUE */

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_ANDROID_POWER)
inline bool rtw_is_earlysuspend_registered(struct pwrctrl_priv *pwrpriv)
{
	return (pwrpriv->early_suspend.suspend) ? _TRUE : _FALSE;
}

inline bool rtw_is_do_late_resume(struct pwrctrl_priv *pwrpriv)
{
	return (pwrpriv->do_late_resume) ? _TRUE : _FALSE;
}

inline void rtw_set_do_late_resume(struct pwrctrl_priv *pwrpriv, bool enable)
{
	pwrpriv->do_late_resume = enable;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void rtw_early_suspend(struct early_suspend *h)
{
	struct pwrctrl_priv *pwrpriv = container_of(h, struct pwrctrl_priv, early_suspend);
	RTW_INFO("%s\n", __FUNCTION__);

	rtw_set_do_late_resume(pwrpriv, _FALSE);
}

static void rtw_late_resume(struct early_suspend *h)
{
	struct pwrctrl_priv *pwrpriv = container_of(h, struct pwrctrl_priv, early_suspend);
	struct dvobj_priv *dvobj = pwrctl_to_dvobj(pwrpriv);
	_adapter *adapter = dvobj_get_primary_adapter(dvobj);

	RTW_INFO("%s\n", __FUNCTION__);

	if (pwrpriv->do_late_resume) {
		rtw_set_do_late_resume(pwrpriv, _FALSE);
		rtw_resume_process(adapter);
	}
}

void rtw_register_early_suspend(struct pwrctrl_priv *pwrpriv)
{
	RTW_INFO("%s\n", __FUNCTION__);

	/* jeff: set the early suspend level before blank screen, so we wll do late resume after scree is lit */
	pwrpriv->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 20;
	pwrpriv->early_suspend.suspend = rtw_early_suspend;
	pwrpriv->early_suspend.resume = rtw_late_resume;
	register_early_suspend(&pwrpriv->early_suspend);


}

void rtw_unregister_early_suspend(struct pwrctrl_priv *pwrpriv)
{
	RTW_INFO("%s\n", __FUNCTION__);

	rtw_set_do_late_resume(pwrpriv, _FALSE);

	if (pwrpriv->early_suspend.suspend)
		unregister_early_suspend(&pwrpriv->early_suspend);

	pwrpriv->early_suspend.suspend = NULL;
	pwrpriv->early_suspend.resume = NULL;
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

#ifdef CONFIG_ANDROID_POWER
static void rtw_early_suspend(android_early_suspend_t *h)
{
	struct pwrctrl_priv *pwrpriv = container_of(h, struct pwrctrl_priv, early_suspend);
	RTW_INFO("%s\n", __FUNCTION__);

	rtw_set_do_late_resume(pwrpriv, _FALSE);
}

static void rtw_late_resume(android_early_suspend_t *h)
{
	struct pwrctrl_priv *pwrpriv = container_of(h, struct pwrctrl_priv, early_suspend);
	struct dvobj_priv *dvobj = pwrctl_to_dvobj(pwrpriv);
	_adapter *adapter = dvobj_get_primary_adapter(dvobj);

	RTW_INFO("%s\n", __FUNCTION__);
	if (pwrpriv->do_late_resume) {
#if defined(CONFIG_USB_HCI) || defined(CONFIG_SDIO_HCI) || defined(CONFIG_GSPI_HCI)
		rtw_set_do_late_resume(pwrpriv, _FALSE);
		rtw_resume_process(adapter);
#endif
	}
}

void rtw_register_early_suspend(struct pwrctrl_priv *pwrpriv)
{
	RTW_INFO("%s\n", __FUNCTION__);

	/* jeff: set the early suspend level before blank screen, so we wll do late resume after scree is lit */
	pwrpriv->early_suspend.level = ANDROID_EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 20;
	pwrpriv->early_suspend.suspend = rtw_early_suspend;
	pwrpriv->early_suspend.resume = rtw_late_resume;
	android_register_early_suspend(&pwrpriv->early_suspend);
}

void rtw_unregister_early_suspend(struct pwrctrl_priv *pwrpriv)
{
	RTW_INFO("%s\n", __FUNCTION__);

	rtw_set_do_late_resume(pwrpriv, _FALSE);

	if (pwrpriv->early_suspend.suspend)
		android_unregister_early_suspend(&pwrpriv->early_suspend);

	pwrpriv->early_suspend.suspend = NULL;
	pwrpriv->early_suspend.resume = NULL;
}
#endif /* CONFIG_ANDROID_POWER */

static void _rtw_ssmps(_adapter *adapter, struct sta_info *sta)
{
	struct mlme_ext_priv *pmlmeext = &(adapter->mlmeextpriv);
	struct mlme_ext_info *pmlmeinfo = &(pmlmeext->mlmext_info);

	issue_action_SM_PS_wait_ack(adapter , sta->phl_sta->mac_addr,
			sta->phl_sta->asoc_cap.sm_ps, 3, 1);

	if (sta->phl_sta->asoc_cap.sm_ps == SM_PS_STATIC) {
		pmlmeext->txss_bk = sta->phl_sta->asoc_cap.nss_rx;
		rtw_phl_sta_assoc_cap_process(sta->phl_sta, _TRUE);
		sta->phl_sta->asoc_cap.nss_rx = 1;
		sta->phl_sta->asoc_cap.stbc_ht_rx = 0;
		sta->phl_sta->asoc_cap.stbc_vht_rx = 0;
		sta->phl_sta->asoc_cap.stbc_he_rx = 0;
	} else {
		sta->phl_sta->asoc_cap.nss_rx = pmlmeext->txss_bk;
		rtw_phl_sta_assoc_cap_process(sta->phl_sta, _FALSE);
	}

	rtw_sta_hal_ra_mask_update_cmd(adapter, sta, RTW_CMDF_DIRECTLY);
}

void rtw_ssmps_enter(_adapter *adapter, struct sta_info *sta)
{
	if (sta->phl_sta->asoc_cap.sm_ps == SM_PS_STATIC)
		return;

	RTW_INFO(ADPT_FMT" STA [" MAC_FMT "]\n", ADPT_ARG(adapter), MAC_ARG(sta->phl_sta->mac_addr));

	sta->phl_sta->asoc_cap.sm_ps = SM_PS_STATIC;
	_rtw_ssmps(adapter, sta);
}

void rtw_ssmps_leave(_adapter *adapter, struct sta_info *sta)
{
	if (sta->phl_sta->asoc_cap.sm_ps == SM_PS_DISABLE)
		return;

	RTW_INFO(ADPT_FMT" STA [" MAC_FMT "] \n", ADPT_ARG(adapter), MAC_ARG(sta->phl_sta->mac_addr));
	sta->phl_sta->asoc_cap.sm_ps = SM_PS_DISABLE;
	_rtw_ssmps(adapter, sta);
}

void rtw_update_ips_setting(int make_level, int ins_level, u8 *mode, u8 *cap, bool is_wow)
{
	if (ins_level != PS_IPS_MAX)
		make_level = ins_level;

	switch (make_level) {
	case PS_IPS_NONE:
		*mode = PS_OP_MODE_DISABLED;
		*cap = PS_CAP_PWRON;
		break;
	case PS_IPS_RF_OFF:
		*mode = is_wow? PS_OP_MODE_FORCE_ENABLED: PS_OP_MODE_AUTO;
		*cap = PS_CAP_PWRON | PS_CAP_RF_OFF;
		break;
	case PS_IPS_CLK_GATED:
		*mode = is_wow? PS_OP_MODE_FORCE_ENABLED: PS_OP_MODE_AUTO;
		*cap = PS_CAP_PWRON | PS_CAP_RF_OFF | PS_CAP_CLK_GATED;
		break;
	case PS_IPS_PWR_GATED:
		*mode = is_wow? PS_OP_MODE_FORCE_ENABLED: PS_OP_MODE_AUTO;
		*cap = PS_CAP_PWRON | PS_CAP_RF_OFF | PS_CAP_CLK_GATED | PS_CAP_PWR_GATED;
		break;
	case PS_PWR_OFF:
		*mode = is_wow? PS_OP_MODE_FORCE_ENABLED: PS_OP_MODE_AUTO;
		*cap = PS_CAP_PWR_OFF;
		break;
	default:
		RTW_ERR("%s ips mode level (%d) invalid\n", __func__, ins_level);
	}
}

void rtw_update_lps_setting(int make_level, int ins_level, u8 *mode, u8 *cap, bool is_wow)
{
	if (ins_level != PS_LPS_MAX)
		make_level = ins_level;

	switch (make_level) {
	case PS_LPS_NONE:
		*mode = PS_OP_MODE_DISABLED;
		*cap = PS_CAP_PWRON;
		break;
	case PS_LPS_RF_OFF:
		*mode = is_wow? PS_OP_MODE_FORCE_ENABLED: PS_OP_MODE_AUTO;
		*cap = PS_CAP_PWRON | PS_CAP_RF_OFF;
		break;
	case PS_LPS_CLK_GATED:
		*mode = is_wow? PS_OP_MODE_FORCE_ENABLED: PS_OP_MODE_AUTO;
		*cap = PS_CAP_PWRON | PS_CAP_RF_OFF | PS_CAP_CLK_GATED;
		break;
	case PS_LPS_PWR_GATED:
		*mode = is_wow? PS_OP_MODE_FORCE_ENABLED: PS_OP_MODE_AUTO;
		*cap = PS_CAP_PWRON | PS_CAP_RF_OFF | PS_CAP_CLK_GATED | PS_CAP_PWR_GATED;
		break;
	default:
		RTW_ERR("%s lps mode level (%d) invalid\n", __func__, ins_level);
	}
}

#if defined(CONFIG_RTW_LPS) || defined(CONFIG_RTW_LPS_WOW)
void rtw_update_lps_listen_beacon_mode(
	u8 bcn_mode, u8 awake_interval,
	enum rtw_lps_listen_bcn_mode *lps_bcn_mode, u8 *lps_awake_interval)
{
	if (bcn_mode == RTW_LPS_LISTEN_BCN_MAX) {
		*lps_bcn_mode = RTW_LPS_RLBM_MAX;
		*lps_awake_interval = 0;
	} else if (bcn_mode == RTW_LPS_RLBM_MIN ||
		   bcn_mode == RTW_LPS_RLBM_MAX) {
		*lps_bcn_mode = bcn_mode;
		*lps_awake_interval = 0;
		RTW_INFO(
			"%s: awake interval will not be applied, since beacon mode is not RTW_LPS_RLBM_USERDEFINE\n",
			__func__);
	} else if (bcn_mode == RTW_LPS_RLBM_USERDEFINE) {
		*lps_bcn_mode = bcn_mode;
		*lps_awake_interval = awake_interval;
	} else {
		RTW_ERR("%s lps listen beacon mode (%d) invalid\n", __func__,
			bcn_mode);
	}
}
#endif /* defined(CONFIG_RTW_LPS) || defined(CONFIG_RTW_LPS_WOW) */