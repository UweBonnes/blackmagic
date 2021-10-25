/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright(C) 2018 - 2021 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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

/* MPSSE bit-banging SW-DP interface over FTDI with loop unrolled.
 * Speed is sensible.
 */

#include "general.h"
#include <assert.h>

#include <ftdi.h>
#include "ftdi_bmp.h"

enum  swdio_status{
	SWDIO_STATUS_DRIVE = 0,
	SWDIO_STATUS_FLOAT,
};

static enum swdio_status olddir;
static bool do_mpsse;
static bool direct_bb_swd;

#define MPSSE_MASK (MPSSE_DO | MPSSE_DI | MPSSE_CS)
#define MPSSE_TD_MASK (MPSSE_DO | MPSSE_DI)
#define MPSSE_TMS_SHIFT (MPSSE_WRITE_TMS | MPSSE_LSB |\
						 MPSSE_BITMODE | MPSSE_WRITE_NEG)
#define MPSSE_TDO_SHIFT (MPSSE_DO_WRITE | MPSSE_LSB |\
						 MPSSE_BITMODE | MPSSE_WRITE_NEG)
static void swdptap_turnaround(enum swdio_status dir)
{
	if (dir == olddir)
		return;
	olddir = dir;
	uint8_t cmd[16], *p = cmd;
	DEBUG_PROBE("Turnaround %s: ", (dir == SWDIO_STATUS_FLOAT) ? "float": "drive");
	if (do_mpsse) {
		if (dir == SWDIO_STATUS_FLOAT)	/* SWDIO goes to input */ {
			active_state.data_low |=  active_cable->mpsse_swd_read.set_data_low | MPSSE_DO;
			active_state.data_low &= ~active_cable->mpsse_swd_read.clr_data_low;
			active_state.ddr_low &= ~MPSSE_DO;
			active_state.data_high |=  active_cable->mpsse_swd_read.set_data_high;
			active_state.data_high &= ~active_cable->mpsse_swd_read.clr_data_high;
			*p++ = SET_BITS_LOW;
			*p++ = active_state.data_low;
			*p++ = active_state.ddr_low;
			*p++ = SET_BITS_HIGH;
			*p++ = active_state.data_high;
			*p++ = active_state.ddr_high;
		}
		*p++ = MPSSE_TDO_SHIFT;
		*p++ = 0;
		*p++ = 0; /* One clock cycle */
		if (dir == SWDIO_STATUS_DRIVE)  /* SWDIO goes to output */ {
			active_state.data_low |=  active_cable->mpsse_swd_write.set_data_low | MPSSE_DO;
			active_state.data_low &= ~active_cable->mpsse_swd_write.clr_data_low;
			active_state.ddr_low |= MPSSE_DO;
			active_state.data_high |=  active_cable->mpsse_swd_write.set_data_high;
			active_state.data_high &= ~active_cable->mpsse_swd_write.clr_data_high;
			*p++ = SET_BITS_LOW;
			*p++ = active_state.data_low;
			*p++ = active_state.ddr_low;
			*p++ = SET_BITS_HIGH;
			*p++ = active_state.data_high;
			*p++ = active_state.ddr_high;
		}
	} else {
		if(dir == SWDIO_STATUS_FLOAT)	  { /* SWDIO goes to input */
			if (direct_bb_swd) {
				active_state.data_low |=  MPSSE_CS;
				active_state.ddr_low  &= ~MPSSE_CS;
			} else {
				active_state.data_low  |=  active_cable->bb_swd_read.set_data_low;
				active_state.data_low  &= ~active_cable->bb_swd_read.clr_data_low;
				active_state.data_high |=  active_cable->bb_swd_read.set_data_high;
				active_state.data_high &= ~active_cable->bb_swd_read.clr_data_high;
			}
			*p++ = SET_BITS_LOW;
			*p++ = active_state.data_low;
			*p++ = active_state.ddr_low;
			*p++ = SET_BITS_HIGH;
			*p++ = active_state.data_high;
			*p++ = active_state.ddr_high;
		}
		/* One clock cycle */
		*p++ = MPSSE_TMS_SHIFT;
		*p++ = 0;
		*p++ = 0;
		if (dir == SWDIO_STATUS_DRIVE) {
			if (direct_bb_swd) {
				active_state.data_low |=  MPSSE_CS;
				active_state.ddr_low  |=  MPSSE_CS;
			} else {
				active_state.data_low  |=  active_cable->bb_swd_write.set_data_low;
				active_state.data_low  &= ~active_cable->bb_swd_write.clr_data_low;
				active_state.data_high |=  active_cable->bb_swd_write.set_data_high;
				active_state.data_high &= ~active_cable->bb_swd_write.clr_data_high;
			}
			*p++ = SET_BITS_LOW;
			*p++ = active_state.data_low;
			*p++ = active_state.ddr_low;
			*p++ = SET_BITS_HIGH;
			*p++ = active_state.data_high;
			*p++ = active_state.ddr_high;
		}
	}
	libftdi_buffer_write(cmd, p - cmd);
}

static bool swdptap_seq_in_parity(uint32_t *res, int ticks);
static uint32_t swdptap_seq_in(int ticks);
static void swdptap_seq_out(uint32_t MS, int ticks);
static void swdptap_seq_out_parity(uint32_t MS, int ticks);

bool libftdi_swd_possible(bool *do_mpsse, bool *direct_bb_swd)
{
	bool swd_read =
		active_cable->mpsse_swd_read.set_data_low ||
		active_cable->mpsse_swd_read.clr_data_low ||
		active_cable->mpsse_swd_read.set_data_high ||
		active_cable->mpsse_swd_read.clr_data_high;
	bool swd_write =
		active_cable->mpsse_swd_write.set_data_low ||
		active_cable->mpsse_swd_write.clr_data_low ||
		active_cable->mpsse_swd_write.set_data_high ||
		active_cable->mpsse_swd_write.clr_data_high;
	bool mpsse = swd_read && swd_write;
	if (do_mpsse)
		*do_mpsse = mpsse;
	if (!mpsse) {
		bool bb_swd_read =
			active_cable->bb_swd_read.set_data_low ||
			active_cable->bb_swd_read.clr_data_low ||
			active_cable->bb_swd_read.set_data_high ||
			active_cable->bb_swd_read.clr_data_high;
		bool bb_swd_write =
			active_cable->bb_swd_write.set_data_low ||
			active_cable->bb_swd_write.clr_data_low ||
			active_cable->bb_swd_write.set_data_high ||
			active_cable->bb_swd_write.clr_data_high;
		bool bb_direct_possible =
			active_cable->bb_swdio_in_port_cmd == GET_BITS_LOW &&
			active_cable->bb_swdio_in_pin == MPSSE_CS;
		if (!bb_swd_read && !bb_swd_write) {
			if (!bb_direct_possible)
				return false;
		}
		if (direct_bb_swd)
			*direct_bb_swd = true;
	}
	return true;
}

/* Construct stream of MPSSE commands. Read back later
 * MPSSE: 29 Bytes per word, else 156 Byte
 */
static void mpsse_fast_swdp_low_read(uint16_t addr)
{
	swdptap_turnaround(SWDIO_STATUS_DRIVE); /* 9 */
	unsigned int request = make_packet_request(ADIV5_LOW_READ, addr);
	if (do_mpsse) {
		uint8_t cmd[16] = {
			MPSSE_DO_WRITE | MPSSE_WRITE_NEG | MPSSE_LSB,
			1 - 1,
			0,
			request
			};
		libftdi_buffer_write(cmd, 4); /* 4 Byte */
		swdptap_turnaround(SWDIO_STATUS_FLOAT); /* 9 Byte */
		unsigned int index = 0;
		cmd[index++] = MPSSE_DO_READ | MPSSE_LSB | MPSSE_BITMODE;
		cmd[index++] = 3 - 1;
		cmd[index++] = MPSSE_DO_READ | MPSSE_LSB;
		cmd[index++] = 4 - 1;
		cmd[index++] = 0;
		cmd[index++] = MPSSE_DO_READ | MPSSE_LSB | MPSSE_BITMODE;
		cmd[index++] = 1 - 1;
		libftdi_buffer_write(cmd, index); /* 7 Byte */
	} else {
		uint8_t cmd[16];
		unsigned int index = 0;
		cmd[index++] = MPSSE_TMS_SHIFT;
		cmd[index++] = 7 - 1;
		cmd[index++] = request & 0x7f;
		request >>= 7;
		cmd[index++] = MPSSE_TMS_SHIFT;
		cmd[index++] = 1 - 1;
		cmd[index++] = request & 0x7f;
		libftdi_buffer_write(cmd, index); /* 6 Byte */
		swdptap_turnaround(SWDIO_STATUS_FLOAT); /* 9 Byte*/
		cmd[0] = active_cable->bb_swdio_in_port_cmd;
		cmd[1] = MPSSE_TMS_SHIFT;
		cmd[2] = 0;
		cmd[3] = 0;
		int  bits = 36;
		while (bits--) {
			libftdi_buffer_write(cmd, sizeof(cmd)); /* 138 Bytes*/
		}
	}
}

static int eval(void *dest, int words)
{
	uint8_t buffer[4096], *p = buffer;
	uint32_t *data = (uint32_t*)dest;
	int ret = 0;
	if (do_mpsse) {
		int size = libftdi_buffer_read(buffer, words * 6);
		int items = size / 6; /* 1 Byte ack, 4 byte data, 1 byte parity */
		ret = items;
		if (size - ret * 6) {
			DEBUG_WARN("Invalid read size %d\n", size);
			return 0;
		}
		while (items--) {
			uint8_t ack = (*p++ >> 5);
			if (ack != 1) {
				DEBUG_WARN("Invalid ack %x\n", ack);
				return 0;
			}
			uint32_t word = *p++;
			word |= *p++ <<  8;
			word |= *p++ << 16;
			word |= *p++ << 24;
			int parity = (*p++ & 0x80) ? 1 : 0; /* parity */
			if ((__builtin_popcount(word) + parity) & 1) {
				DEBUG_WARN("Invalid parity %08" PRIx32": %x\n", word, parity);
				return 0;
			}
			*data++ = word;
		}
	} else {
		int size = libftdi_buffer_read(buffer, words * 36);
		int items = size / 36; /* 3 Byte ack, 32 byte data, 1 byte parity */
		ret = items;
		if (size - ret * 36) {
			DEBUG_WARN("Invalid read size %d\n", size);
			return 0;
		}
		while (items--) {
			uint8_t ack = (*p++ & 0x80);
			ack |= (*p++ & 0x80) << 1;
			ack |= (*p++ & 0x80) << 2;
			if (ack != 1) {
				DEBUG_WARN("Invalid ack %x\n", ack);
				return 0;
			}
			uint32_t word = 0;
			for (int i = 0; i < 32; i++)
				word += (*p++ & 0x80) ? (1 << i) : 0;
			*data++ = word;
			int parity = *p++ & 0x80;
			if ((__builtin_popcount(word) + parity) & 1) {
				DEBUG_WARN("Invalid parity %08" PRIx32": %x\n", word, parity);
				return 0;
			}
		}
	}
	return ret;
}

/* Do not evaluate the device ack on every word. */
static void mpsse_mem_read(ADIv5_AP_t *ap, void *dest, uint32_t src, size_t len)
{
	if ((src & 3) || (len & 3))
		return firmware_mem_read(ap, dest, src, len);
	len >>= ALIGN_WORD;
	uint32_t csw = ap->csw | ADIV5_AP_CSW_ADDRINC_SINGLE | ADIV5_AP_CSW_SIZE_WORD;
	ap->dp->ap_write(ap, ADIV5_AP_CSW, csw);
	ap->dp->low_access(ap->dp, ADIV5_LOW_WRITE, ADIV5_AP_TAR, src);
	ap->dp->low_access(ap->dp, ADIV5_LOW_READ, ADIV5_AP_DRW, 0);
	/* Check if adress causes fault*/
	ap->dp->low_access(ap->dp, ADIV5_LOW_READ, ADIV5_DP_RDBUFF, 0);
	if (ap->dp->fault)
		return;
	int words = 0;
	while (--len) {
		mpsse_fast_swdp_low_read(ADIV5_AP_DRW);
		src += (1 << ALIGN_WORD);
		words++;
		if (!do_mpsse && (words >= (4095 / 36))) {
				int items = eval(dest, words);
				if (!items) {
					ap->dp->fault = 1;
					return;
				}
				dest += items  * 4;
				words = 0;
		}
		/* Check for 10 bit address overflow */
		if (!(src & 0x3ff)) {
			if (words) {
				int items = eval(dest, words);
				if (!items) {
					ap->dp->fault = 1;
					return;
				}
				dest += items  * 4;
				words = 0;
			}
			adiv5_dp_low_access(ap->dp, ADIV5_LOW_WRITE, ADIV5_AP_TAR, src);
			adiv5_dp_low_access(ap->dp, ADIV5_LOW_READ, ADIV5_AP_DRW, 0);
			/* Check if adress causes fault*/
			ap->dp->low_access(ap->dp, ADIV5_LOW_READ, ADIV5_DP_RDBUFF, 0);
			if (ap->dp->fault)
				return;
		}
	}
	mpsse_fast_swdp_low_read(ADIV5_DP_RDBUFF);
	words++;
	int items = eval(dest, words);
	if (!items) {
		ap->dp->fault = 1;
		return;
	}
}

int libftdi_swdptap_init(ADIv5_DP_t *dp)
{
	if (!libftdi_swd_possible(&do_mpsse, &direct_bb_swd)) {
		DEBUG_WARN("SWD not possible or missing item in cable description.\n");
		return -1;
	}
	active_state.data_low |=   MPSSE_CS | MPSSE_DI | MPSSE_DO;
	active_state.data_low &=   MPSSE_SK;
	active_state.ddr_low  |=   MPSSE_SK;
	active_state.ddr_low  &= ~(MPSSE_CS | MPSSE_DI | MPSSE_DO);
	if (do_mpsse) {
		DEBUG_INFO("Using genuine MPSSE for SWD.\n");
		active_state.data_low  |=   active_cable->mpsse_swd_read.set_data_low;
		active_state.data_low  &= ~(active_cable->mpsse_swd_read.clr_data_low);
		active_state.data_high |=   active_cable->mpsse_swd_read.set_data_high;
		active_state.data_high &= ~(active_cable->mpsse_swd_read.clr_data_high);
	} else if (direct_bb_swd) {
		DEBUG_INFO("Using direct bitbang with SWDIO %cBUS%d.\n",
				   (active_cable->bb_swdio_in_port_cmd == GET_BITS_LOW) ? 'C' : 'D',
				   __builtin_ctz(active_cable->bb_swdio_in_pin));
	} else {
		DEBUG_INFO("Using switched bitbang for SWD.\n");
		active_state.data_low  |=   active_cable->bb_swd_read.set_data_low;
		active_state.data_low  &= ~(active_cable->bb_swd_read.clr_data_low);
		active_state.data_high |=   active_cable->bb_swd_read.set_data_high;
		active_state.data_high &= ~(active_cable->bb_swd_read.clr_data_high);
		active_state.ddr_low  |=  MPSSE_CS;
		if (active_cable->bb_swdio_in_port_cmd == GET_BITS_LOW)
			active_state.ddr_low  &= ~active_cable->bb_swdio_in_pin;
		else if (active_cable->bb_swdio_in_port_cmd == GET_BITS_HIGH)
			active_state.ddr_high &= ~active_cable->bb_swdio_in_pin;
	}
	uint8_t cmd_write[6] = {
		SET_BITS_LOW,  active_state.data_low,
		active_state.ddr_low,
		SET_BITS_HIGH, active_state.data_high, active_state.ddr_high};
	libftdi_buffer_write(cmd_write, 6);
	libftdi_buffer_flush();
	olddir = SWDIO_STATUS_FLOAT;

	dp->seq_in  = swdptap_seq_in;
	dp->seq_in_parity  = swdptap_seq_in_parity;
	dp->seq_out = swdptap_seq_out;
	dp->seq_out_parity  = swdptap_seq_out_parity;
	dp->dp_read = firmware_swdp_read;
	dp->error = firmware_swdp_error;
	dp->low_access = firmware_swdp_low_access;
	dp->abort = firmware_swdp_abort;
	dp->mem_read = mpsse_mem_read;
	return 0;
}

static bool swdptap_seq_in_parity(uint32_t *res, int ticks)
{
	assert(ticks == 32);
	swdptap_turnaround(SWDIO_STATUS_FLOAT);
	unsigned int parity = 0;
	unsigned int result = 0;
	if (do_mpsse) {
		uint8_t DO[5];
		libftdi_jtagtap_tdi_tdo_seq(DO, 0, NULL, ticks + 1);
		result = DO[0] + (DO[1] << 8) + (DO[2] << 16) + (DO[3] << 24);
		parity =  __builtin_parity(result & ((1LL << ticks) - 1)) & 1;
		parity ^= DO[4] & 1;
	} else {
		int index = ticks + 1;
		uint8_t cmd[4];

		cmd[0] = active_cable->bb_swdio_in_port_cmd;
		cmd[1] = MPSSE_TMS_SHIFT;
		cmd[2] = 0;
		cmd[3] = 0;
		while (index--) {
			libftdi_buffer_write(cmd, sizeof(cmd));
		}
		uint8_t data[33];
		libftdi_buffer_read(data, ticks + 1);
		if (data[ticks] & active_cable->bb_swdio_in_pin)
			parity ^= 1;
		index = ticks;
		while (index--) {
			if (data[index] & active_cable->bb_swdio_in_pin) {
				parity ^= 1;
				result |= (1 << index);
			}
		}
	}
	*res = result;
	return parity;
}

static uint32_t swdptap_seq_in(int ticks)
{
	if (!ticks)
		return 0;
	uint32_t result = 0;
	swdptap_turnaround(SWDIO_STATUS_FLOAT);
	if (do_mpsse) {
		uint8_t DO[4];
		libftdi_jtagtap_tdi_tdo_seq(DO, 0, NULL, ticks);
		int bytes = ticks >> 3;
		if (ticks & 7)
			bytes++;
		for (int i = 0; i < bytes; i++) {
			result |= DO[i] << (8 * i);
		}
	} else {
		int index = ticks;
		uint8_t cmd[4];

		cmd[0] = active_cable->bb_swdio_in_port_cmd;
		cmd[1] = MPSSE_TMS_SHIFT;
		cmd[2] = 0;
		cmd[3] = 0;

		while (index--) {
			libftdi_buffer_write(cmd, sizeof(cmd));
		}
		uint8_t data[32];
		libftdi_buffer_read(data, ticks);
		index = ticks;
		while (index--) {
			if (data[index] & active_cable->bb_swdio_in_pin)
				result |= (1 << index);
		}
	}
	return result;
}

static void swdptap_seq_out(uint32_t MS, int ticks)
{
	if (!ticks)
		return;
	swdptap_turnaround(SWDIO_STATUS_DRIVE);
	if (do_mpsse) {
		uint8_t DI[4];
		DI[0] = (MS >>  0) & 0xff;
		DI[1] = (MS >>  8) & 0xff;
		DI[2] = (MS >> 16) & 0xff;
		DI[3] = (MS >> 24) & 0xff;
		libftdi_jtagtap_tdi_tdo_seq(NULL, 0, DI, ticks);
	} else {
		uint8_t cmd[16];
		unsigned int index = 0;
		while (ticks) {
			cmd[index++] = MPSSE_TMS_SHIFT;
			if (ticks >= 7) {
				cmd[index++] = 6;
				cmd[index++] = MS & 0x7f;
				MS >>= 7;
				ticks -= 7;
			} else {
				cmd[index++] = ticks - 1;
				cmd[index++] = MS & 0x7f;
				ticks = 0;
			}
		}
		libftdi_buffer_write(cmd, index);
	}
}

/* ARM Debug Interface Architecture Specification ADIv5.0 to ADIv5.2
 * tells to clock the data through SW-DP to either :
 * - immediate start a new transaction
 * - continue to drive idle cycles
 * - or clock at least 8 idle cycles
 *
 * Implement last option to favour correctness over
 *   slight speed decrease
 */
static void swdptap_seq_out_parity(uint32_t MS, int ticks)
{
	(void) ticks;
	int parity = __builtin_parity(MS) & 1;
	unsigned int index = 0;
	swdptap_turnaround(SWDIO_STATUS_DRIVE);
	if (do_mpsse) {
		uint8_t DI[8];
		DI[0] = (MS >>  0) & 0xff;
		DI[1] = (MS >>  8) & 0xff;
		DI[2] = (MS >> 16) & 0xff;
		DI[3] = (MS >> 24) & 0xff;
		DI[4] = parity;
		DI[5] = 0;
		libftdi_jtagtap_tdi_tdo_seq(NULL, 0, DI, 32 + 1 + 8);
	} else {
		uint8_t cmd[32];
		int steps = ticks;
		while (steps) {
			cmd[index++] = MPSSE_TMS_SHIFT;
			cmd[index++] = 6;
			if (steps >= 7) {
				cmd[index++] = MS & 0x7f;
				MS >>= 7;
				steps -= 7;
			} else {
				cmd[index++] = (MS & 0x7f) | (parity << 4);
				steps = 0;
			}
		}
		cmd[index++] = MPSSE_TMS_SHIFT;
		cmd[index++] = 4;
		cmd[index++] = 0;
		libftdi_buffer_write(cmd, index);
	}
}
