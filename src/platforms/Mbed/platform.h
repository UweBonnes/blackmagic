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

/* This file implements the platform specific functions for the STM32
 * implementation.
 */
#ifndef __PLATFORM_H
#define __PLATFORM_H

#include "general.h"

#include "timing.h"
//#include "timing_stm32.h"
#include "version.h"


#ifdef ENABLE_DEBUG
# define PLATFORM_HAS_DEBUG
# define USBUART_DEBUG
extern bool debug_bmp;
int usbuart_debug_write(const char *buf, size_t len);
#endif

#define PLATFORM_IDENT()  "MBed hosted"
#define BOARD_IDENT       "Black Magic Probe (STLINK), (Firmware " FIRMWARE_VERSION ")"
#define BOARD_IDENT_DFU   "Black Magic (Upgrade) for STLink/Discovery, (Firmware " FIRMWARE_VERSION ")"
#define BOARD_IDENT_UPD   "Black Magic (DFU Upgrade) for STLink/Discovery, (Firmware " FIRMWARE_VERSION ")"
#define DFU_IDENT         "Black Magic Firmware Upgrade (STLINK)"
#define UPD_IFACE_STRING  "@Internal Flash   /0x08000000/8*001Kg"


//#define PLATFORM_HAS_TRACESWO	1
#define NUM_TRACE_PACKETS		(128)		/* This is an 8K buffer */
//#define TRACESWO_PROTOCOL		2			/* 1 = Manchester, 2 = NRZ / async */

#if 0
# define SWD_CR   GPIO_CRH(SWDIO_PORT)
# define SWD_CR_MULT (1 << ((14 - 8) << 2))

#define TMS_SET_MODE() \
	gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_50_MHZ, \
	              GPIO_CNF_OUTPUT_PUSHPULL, TMS_PIN);
#define SWDIO_MODE_FLOAT() 	do { \
	uint32_t cr = SWD_CR; \
	cr  &= ~(0xf * SWD_CR_MULT); \
	cr  |=  (0x4 * SWD_CR_MULT); \
	SWD_CR = cr; \
} while(0)
#define SWDIO_MODE_DRIVE() 	do { \
	uint32_t cr = SWD_CR; \
	cr  &= ~(0xf * SWD_CR_MULT); \
	cr  |=  (0x1 * SWD_CR_MULT); \
	SWD_CR = cr; \
} while(0)
#define UART_PIN_SETUP() \
	gpio_set_mode(USBUSART_PORT, GPIO_MODE_OUTPUT_2_MHZ, \
	              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, USBUSART_TX_PIN);

#endif /* if 0 */

#define LED_IDLE_RUN            
#define SET_RUN_STATE(state)	{}
//#define SET_RUN_STATE(state)	{running_status = (state);}
//#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, led_idle_run, state);}
#define SET_IDLE_STATE(state)	{}
#define SET_ERROR_STATE(x)

#endif

