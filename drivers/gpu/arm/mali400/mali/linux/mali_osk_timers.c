/*
 * Copyright (C) 2010-2014, 2016-2017 ARM Limited. All rights reserved.
 * 
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/**
 * @file mali_osk_timers.c
 * Implementation of the OS abstraction layer for the kernel device driver
 */

#include <linux/timer.h>
#include <linux/slab.h>
#include "mali_osk.h"
#include "mali_kernel_common.h"

typedef void (*timer_timeout_function_t)(unsigned long);

void _mali_osk_timer_init(_mali_osk_timer_t *tim, _mali_osk_timer_callback_t callback)
{
	_mali_osk_timer_t *t = tim;
	if (NULL != t)
		timer_setup(t,
			    (void (*)(struct timer_list *))callback, 0);
}

void _mali_osk_timer_add(_mali_osk_timer_t *tim, unsigned long ticks_to_expire)
{
	MALI_DEBUG_ASSERT_POINTER(tim);
	tim->expires = jiffies + ticks_to_expire;
	add_timer(tim);
}

void _mali_osk_timer_mod(_mali_osk_timer_t *tim, unsigned long ticks_to_expire)
{
	MALI_DEBUG_ASSERT_POINTER(tim);
	mod_timer(tim, jiffies + ticks_to_expire);
}

void _mali_osk_timer_del(_mali_osk_timer_t *tim)
{
	MALI_DEBUG_ASSERT_POINTER(tim);
	del_timer_sync(tim);
}

void _mali_osk_timer_del_async(_mali_osk_timer_t *tim)
{
	MALI_DEBUG_ASSERT_POINTER(tim);
	del_timer(tim);
}

mali_bool _mali_osk_timer_pending(_mali_osk_timer_t *tim)
{
	MALI_DEBUG_ASSERT_POINTER(tim);
	return 1 == timer_pending(tim);
}

