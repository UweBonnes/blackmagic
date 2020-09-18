/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2016  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "general.h"
#include "mbed_thread.h"

void platform_delay(uint32_t ms)
{
	thread_sleep_for(ms);
}

uint32_t platform_time_ms(void)
{
	return (uint32_t)get_ms_count();
}

void platform_timeout_set(platform_timeout *t, uint32_t ms)
{
	t->time = (uint32_t)get_ms_count() + ms;
}

bool platform_timeout_is_expired(platform_timeout *t)
{
	return (uint32_t)get_ms_count() >= t->time;
}
