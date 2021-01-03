/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2008  Black Sphere Technologies Ltd.
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

/* Low level JTAG implementation using FT2232 with libftdi.
 *
 */

#include "general.h"
#include <unistd.h>
#include <assert.h>
#include <ftdi.h>
#include "ftdi_bmp.h"


extern cable_desc_t *active_cable;
extern struct ftdi_context *ftdic;

static void jtagtap_reset(void);
static void jtagtap_tms_seq(uint32_t MS, int ticks);
static void jtagtap_tdi_seq(
	const uint8_t final_tms, const uint8_t *DI, int ticks);
static uint8_t jtagtap_next(uint8_t dTMS, uint8_t dTDI);

int libftdi_jtagtap_init(jtag_proc_t *jtag_proc)
{
	if ((active_cable->mpsse_swd_read.set_data_low == MPSSE_DO) &&
		(active_cable->mpsse_swd_write.set_data_low == MPSSE_DO)) {
		printf("Jtag not possible with resistor SWD!\n");
			return -1;
	}
	jtag_proc->jtagtap_reset = jtagtap_reset;
	jtag_proc->jtagtap_next =jtagtap_next;
	jtag_proc->jtagtap_tms_seq = jtagtap_tms_seq;
	jtag_proc->jtagtap_tdi_tdo_seq = libftdi_jtagtap_tdi_tdo_seq;
	jtag_proc->jtagtap_tdi_seq = jtagtap_tdi_seq;

	active_state.data_low  |=   active_cable->jtag.set_data_low |
		MPSSE_CS | MPSSE_DI | MPSSE_DO;
	active_state.data_low  &= ~(active_cable->jtag.clr_data_low | MPSSE_SK);
	active_state.ddr_low   |=   MPSSE_CS | MPSSE_DO | MPSSE_SK;
	active_state.ddr_low   &= ~(MPSSE_DI);
	active_state.data_high |=   active_cable->jtag.set_data_high;
	active_state.data_high &= ~(active_cable->jtag.clr_data_high);
	uint8_t gab[16];
	int garbage =  ftdi_read_data(ftdic, gab, sizeof(gab));
	if (garbage > 0) {
		DEBUG_WARN("FTDI JTAG init got garbage:");
		for (int i = 0; i < garbage; i++)
			DEBUG_WARN(" %02x", gab[i]);
		DEBUG_WARN("\n");
	}
	uint8_t cmd_write[16] = {
		SET_BITS_LOW,  active_state.data_low,
		active_state.ddr_low,
		SET_BITS_HIGH, active_state.data_high, active_state.ddr_high};
	libftdi_buffer_write(cmd_write, 6);
	libftdi_buffer_flush();
	/* Write out start condition and pull garbage from read buffer.
	 * FT2232D otherwise misbehaves on runs follwoing the first run.*/
	garbage =  ftdi_read_data(ftdic, cmd_write, sizeof(cmd_write));
	if (garbage > 0) {
		DEBUG_WARN("FTDI JTAG end init got garbage:");
		for (int i = 0; i < garbage; i++)
			DEBUG_WARN(" %02x", cmd_write[i]);
		DEBUG_WARN("\n");
	}
	jtag_proc->jtagtap_tms_seq(0xffffffff, 32);	/* Reset SW-DP by sending*/
	jtag_proc->jtagtap_tms_seq(0xffffffff, 18); /* 50 cycles hihh*/
	jtag_proc->jtagtap_tms_seq(0xffffE73C, 32);	/* SWD to JTAG sequence */

	return 0;
}

static void jtagtap_reset(void)
{
	jtagtap_soft_reset();
}

static void jtagtap_tms_seq(uint32_t MS, int ticks)
{
	DEBUG_PROBE("jtagtap_tms_seq TMS %8x, ticks %d\n", MS, ticks);
	uint8_t tmp[3] = {
		MPSSE_WRITE_TMS | MPSSE_LSB | MPSSE_BITMODE | MPSSE_WRITE_NEG, 0, 0};
	int current_ticks;
	while(ticks > 0) {
		/* Attention: Bug in FT2232L(D?, H not!).
		 *  With 7 bits TMS shift, static TDO
		 *  value gets set to TMS on last TCK edge*/
		current_ticks = (ticks < 7) ? ticks : 6;
		tmp[1] = current_ticks - 1;
		tmp[2] = 0x80 | MS ; /* KEEP TDO high */

		libftdi_buffer_write(tmp, 3);
		MS >>= current_ticks;
		ticks -= current_ticks;
	}
}

static void jtagtap_tdi_seq(
	const uint8_t final_tms, const uint8_t *DI, int ticks)
{
	return libftdi_jtagtap_tdi_tdo_seq(NULL,  final_tms, DI, ticks);
}

static uint8_t jtagtap_next(uint8_t dTMS, uint8_t dTDI)
{
	DEBUG_PROBE("jtagtap_next TMS %s, TDI %s\n", (dTMS)? "1" : "", (dTDI)? "1" : "");
	uint8_t ret;
	uint8_t tmp[3] = {MPSSE_WRITE_TMS | MPSSE_DO_READ | MPSSE_LSB |
					  MPSSE_BITMODE | MPSSE_WRITE_NEG, 0, 0};
	tmp[2] = (dTDI?0x80:0) | (dTMS?0x01:0);
	libftdi_buffer_write(tmp, 3);
	libftdi_buffer_read(&ret, 1);

	ret &= 0x80;

	return ret;
}
