/******************************************************************************
 *
 * Copyright(c) 2013 - 2017 Realtek Corporation.
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
#include <linux/rfkill-wlan.h>

extern unsigned int oob_irq;

#ifndef CONFIG_PLATFORM_OPS
/*
 * Return:
 *	0:	power on successfully
 *	others: power on failed
 */
int platform_wifi_power_on(void)
{
	int ret = 0;

	oob_irq = rockchip_wifi_get_oob_irq(); //这里对应dts的WIFI_WAKE_HOST PIN脚

	return ret;
}

void platform_wifi_power_off(void)
{
}
#endif /* !CONFIG_PLATFORM_OPS */
