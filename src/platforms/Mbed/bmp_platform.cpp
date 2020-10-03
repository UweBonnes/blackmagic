/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
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

/* This file implements the platform specific functions for the ST-Link
 * implementation.
 */
#include "mbed.h"
#include "blackmagic/src/platforms/Mbed/platform.h"
#include "gdb_if.h"

int cl_debuglevel;
AnalogIn	ainTargetVoltage(PA_6);
static char sVoltage[16];

extern "C" {
// uint16_t led_idle_run;
// uint16_t srst_pin;

int platform_hwversion(void)
{
	return 0;
}

void platform_init()
{
	printf("platform init()\n");
	
//	cl_debuglevel = BMP_DEBUG_INFO | BMP_DEBUG_GDB | BMP_DEBUG_TARGET | BMP_DEBUG_PROBE | BMP_DEBUG_WIRE | BMP_DEBUG_STDOUT ;
	cl_debuglevel = BMP_DEBUG_INFO | BMP_DEBUG_STDOUT;

	gdb_if_init();
}

void platform_srst_set_val(bool assert)
{
}

bool platform_srst_get_val()
{
	return false;
}

const char *platform_target_voltage(void)
{
	float sum = 0.0f;
	int nAverage = 16;
	float refVoltage = 3.3f;		// TODO: check reference voltage

	for (int i = 0; i < nAverage; i++) {
		sum += ainTargetVoltage;
	}
	float relVoltage = sum / nAverage;
	float absVoltage = relVoltage * refVoltage;	
	snprintf(sVoltage, sizeof(sVoltage), "%6.2f V", absVoltage);

	return sVoltage;
}

} // extern "C"
