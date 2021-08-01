/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2020, 2021 Uwe Bonnes bon@elektron.ikp.physik.tu-darmstadt.de
 *
  * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/* Implement basic support for Atmel AVR8.
 * Implemented:
 * - Flash, eeprom, fuses read and write
 * - Lock byte read
 * Missing:
 * - lots of idcodes
 * - size of EEPROM
 * - size of fuses and default values
 *
 * EEPROM is mapped at 0x100000. EEPROM write can only
 * program erased bits, but not erase programmed bits.
 * Erase EEPROM (and flash) by chip erase with EESAV unset
 *
 * FUSES are mapped at 0x200000, Lock Byte at 0x200003
 */
#include "general.h"
#include "jtag_scan.h"
#include "jtagtap.h"
#include "target.h"
#include "target_internal.h"

enum AVR8_JTAG_REGS {
	AVR8_EXTEST = 0,
	AVR8_IDCODE,
	AVR8_SAMPLE_PRELOAD,
	AVR8_REG_0X3,
	AVR8_PROG_ENABLE,
	AVR8_PROG_COMMANDS,
	AVR8_PROG_PAGELOAD,
	AVR8_PROG_PAGEREAD,
	AVR8_JTAG_INS_HALT,
	AVR8_JTAG_INS_RUN,
	AVR8_PRIVATE2,
	AVR8_PRIVATE3,
	AVR8_RESET,
	AVR8_REG_0XD,
	AVR8_REG_0XE,
	AVR8_BYPASS
};
#define AVR8_EEPROM_START 0x100000
#define AVR8_EEPROM_PAGE_SIZE    8
#define AVR8_FUSES_START  0x200000
#define AVR8_LOCK_START   (AVR8_FUSES_START + 3)

#define AVR8_REG_SIZE 39

#define AVR8_JTAGEN 0x40

#define AVR8_PAGE_SIZE 0x100
#define AVR8_PAGE_MASK (AVR8_PAGE_SIZE - 1)

#define AVR8_POLL_DONE                  0x0200

#define AVR8_CMD_PROG_ENABLE            0xa370

#define AVR8_CMD_CHIP_ERASE_SEQ0        0x2380
#define AVR8_CMD_CHIP_ERASE_SEQ1        0x3180
#define AVR8_CMD_CHIP_ERASE_POLL        0x3380

#define AVR8_CMD_LOAD_ADDR_EXT          0x0b00
#define AVR8_CMD_LOAD_ADDR_HIGH         0x0700
#define AVR8_CMD_LOAD_ADDR_LOW          0x0300
#define AVR8_CMD_ENTER_FLASH_READ       0x2302
#define AVR8_CMD_FLASH_READ_START       0x3200
#define AVR8_CMD_FLASH_READ_LOW_BYTE    0x3600
#define AVR8_CMD_FLASH_READ_HIGH_BYTE   0x3700

#define AVR8_CMD_ENTER_FLASH_WRITE      0x2310
#define AVR8_CMD_POLL_WRITE_TRIGGER     0x3500
#define AVR8_CMD_POLL_WRITE_COMPLETE    0x3700

#define AVR8_CMD_ENTER_FUSES_READ       0x2304
#define AVR8_CMD_FUSES_READ             0x3a00
#define AVR8_CMD_FUSES_READ_EXT         0x3e00
#define AVR8_CMD_FUSES_READ_HIGH        0x3200
#define AVR8_CMD_FUSES_READ_LOW         0x3600
#define AVR8_CMD_LOCK_READ              0x3700

#define AVR8_CMD_ENTER_FUSES_WRITE      0x2340
#define AVR8_CMD_FUSE_DATA              0x1300
#define AVR8_CMD_FUSE_EXT_WRITE_SEQ     0x3b00
#define AVR8_CMD_FUSE_EXT_WRITE_SEQ2    0x3900
#define AVR8_CMD_FUSE_HIGH_WRITE_SEQ    0x3700
#define AVR8_CMD_FUSE_HIGH_WRITE_SEQ2   0x3500
#define AVR8_CMD_FUSE_LOW_WRITE_SEQ     0x3300
#define AVR8_CMD_FUSE_LOW_WRITE_SEQ2    0x3100
#define AVR8_CMD_POLL_FUSE_COMPLETE     0x3300

#define AVR8_CMD_ENTER_EEPROM_READ      0x2303
#define AVR8_CMD_EEPROM_SEQ             0x3300
#define AVR8_CMD_EEPROM_SEQ2            0x3200

#define AVR8_CMD_ENTER_EEPROM_WRITE     0x2311
#define AVR8_CMD_EEPROM_DATA            0x1300
#define AVR8_CMD_EEPROM_DATA_SEQ        0x3700
#define AVR8_CMD_EEPROM_DATA_SEQ2       0x7700
#define AVR8_CMD_EEPROM_PAGE_SEQ        0x3300
#define AVR8_CMD_EEPROM_PAGE_SEQ2       0x3100
#define AVR8_CMD_EEPROM_PAGE_POLL       0x3300

#define AVR8_CMD_ENTER_LOCK_WRITE       0x2320
#define AVR8_CMD_LOCK_WRITE_SEQ         0x3300
#define AVR8_CMD_LOCK_WRITE_SEQ2        0x3100
#define AVR8_CMD_LOCK_WRITE_POLL        0x3100

static bool avr8_cmd_fuses (target* t, int argc, char** argv);
static bool avr8_cmd_eeprom     (target* t, int argc, char** argv);

static const struct command_s avr8_cmd_list[] = {
        { "option",             (cmd_handler) avr8_cmd_fuses,
          "Manipulate fuse bytes"}, /* All other devices use "option" */
        { "fuses ",             (cmd_handler) avr8_cmd_fuses,
          "Manipulate fuse bytes (Alias for option"},
        { "eeprom",             (cmd_handler) avr8_cmd_eeprom,
          "Manipulate EEPROM(NVM data) memory"},
        { NULL, NULL, NULL },
};

static uint16_t avr8_jtag_command_data(uint32_t jtag_index, uint16_t cmd_data)
{
	uint8_t res[2];
	uint8_t cmd[2] = {cmd_data & 0xff, cmd_data >> 8};
	jtag_dev_shift_dr(&jtag_proc, jtag_index, res, cmd, 15);
	return res[0] | (res[1] << 8);
}

static uint32_t avr8_jtag_command(uint32_t jtag_index,
								  enum AVR8_JTAG_REGS reg_nr, uint16_t data)
{
	DEBUG_PROBE("avr8_jtag_command reg %x, data 0x%04x\n", reg_nr, data);
	uint8_t val[2] = {data & 0xff, data >> 8};
	int len = 0;
	switch (reg_nr) {
	case AVR8_IDCODE:
		len = 32;
		break;
	case AVR8_PROG_ENABLE:
		len = 16;
		break;
	case AVR8_PROG_COMMANDS:
		len = 15;
		break;
	case AVR8_PROG_PAGEREAD:
		len = (AVR8_PAGE_SIZE + 1) * 8;
		break;
	case AVR8_PROG_PAGELOAD:
		len = (AVR8_PAGE_SIZE + 0) * 8;
		len = 8;
		break;
	case AVR8_RESET:
	case AVR8_BYPASS:
		len = 1;
		break;
	case AVR8_JTAG_INS_HALT:
	case AVR8_JTAG_INS_RUN:
		len = 0;
		break;
	default:
		DEBUG_WARN("Unhandled avr8 jtag register %x\n", reg_nr);
		return 1;
	}
	uint8_t res[4] = {0, 0, 0, 0};
	jtag_dev_shift_ir(&jtag_proc, jtag_index, reg_nr);
	if (len)
		jtag_dev_shift_dr(&jtag_proc, jtag_index, res, val, len);
	return res[0] | (res[1] << 8) | (res[1] << 16) | (res[1] << 24);
}

static void avr8_flash_read(uint32_t jtag_index, void *dest,
							target_addr src, size_t len)
{
	while (len) {
		unsigned int batchsize = AVR8_PAGE_SIZE;
		/* AT90CAN128 can start page read anywhere in the page.
		 * Check with older parts*/
		if (0 /*src & AVR8_PAGE_MASK*/) { /* Page read can not be used */
			batchsize -= (src & AVR8_PAGE_MASK);
			if (batchsize > len)
				batchsize = len;
			jtag_dev_shift_ir(&jtag_proc, jtag_index,
							  AVR8_PROG_COMMANDS);
			uint8_t cmd[2] = {AVR8_CMD_ENTER_FLASH_READ & 0xff,
							  AVR8_CMD_ENTER_FLASH_READ >> 8};
			jtag_dev_shift_dr(&jtag_proc, jtag_index, NULL,
								  cmd, 15);
			unsigned int runsize = batchsize;
			while (runsize) {
				unsigned int res;
				avr8_jtag_command_data(jtag_index, AVR8_CMD_FLASH_READ_START);
				avr8_jtag_command_data(jtag_index, src >> 9 |
									   AVR8_CMD_LOAD_ADDR_HIGH);
				avr8_jtag_command_data(jtag_index, src >> 1 |
									   AVR8_CMD_LOAD_ADDR_LOW);
				res = avr8_jtag_command_data(jtag_index,
											 AVR8_CMD_FLASH_READ_LOW_BYTE);
				if (!(src & 1)) {
					*(uint8_t *)dest = res & 0xff;
					dest += 1;
					src ++;
					runsize --;
				}
				res = avr8_jtag_command_data(jtag_index,
											 AVR8_CMD_FLASH_READ_HIGH_BYTE);
				if (runsize) {
					*(uint8_t *)dest = res & 0xff;
					dest += 1;
					runsize --;
				}
				src++;
			}
			len -= batchsize;
		} else {
			if (len < batchsize)
				batchsize = len;
			avr8_jtag_command(jtag_index, AVR8_PROG_COMMANDS,
							  AVR8_CMD_ENTER_FLASH_READ);
			avr8_jtag_command_data(jtag_index, ((src >> 9) & 0xff) |
								   AVR8_CMD_LOAD_ADDR_HIGH);
			avr8_jtag_command_data(jtag_index, ((src >> 1) & 0xff) |
								   AVR8_CMD_LOAD_ADDR_LOW);
			jtag_dev_shift_ir(&jtag_proc, jtag_index,
							  AVR8_PROG_PAGEREAD);
			if (src & 1) {
				DEBUG_WARN("odd\n");
				jtag_dev_shift_dr(&jtag_proc,jtag_index, NULL,
								  NULL, 8);
			}
			len -= batchsize;
			while (batchsize) {
				jtag_dev_shift_dr(&jtag_proc, jtag_index, dest,
								  (uint8_t*)&src, 8);
				batchsize--;
				src++;
				dest++;
			}
		}
	}
}

void avr8_mem_read_firmware(uint32_t jtag_index, void *dest,
							target_addr src, size_t len)
{
	DEBUG_TARGET("mem_read len %d from %04x\n", len, src);
	avr8_jtag_command(jtag_index, AVR8_PROG_ENABLE, AVR8_CMD_PROG_ENABLE);
	if (src < AVR8_EEPROM_START) {
		avr8_flash_read(jtag_index, dest, src, len);
	} else if (src < AVR8_FUSES_START) {
		avr8_jtag_command(jtag_index, AVR8_PROG_COMMANDS,
						  AVR8_CMD_ENTER_EEPROM_READ);
		while (len) {
			avr8_jtag_command_data(jtag_index, ((src >> 8) & 0xff) |
								   AVR8_CMD_LOAD_ADDR_HIGH);
			avr8_jtag_command_data(jtag_index, ((src >> 0) & 0xff) |
								   AVR8_CMD_LOAD_ADDR_LOW);
			avr8_jtag_command_data(jtag_index, ((src >> 0) & 0xff) |
								   AVR8_CMD_EEPROM_SEQ);
			avr8_jtag_command_data(jtag_index, AVR8_CMD_EEPROM_SEQ2);
			unsigned int res = avr8_jtag_command_data(jtag_index,
													  AVR8_CMD_EEPROM_SEQ);
			*(uint8_t *)dest = res & 0xff;
			dest++;
			src++;
			len --;
		}
	} else {
		uint8_t fuses[4];
		avr8_jtag_command(jtag_index, AVR8_PROG_COMMANDS,
						  AVR8_CMD_ENTER_FUSES_READ);
		avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSES_READ);
		fuses[0] = avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSES_READ_EXT);
		fuses[1] = avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSES_READ_HIGH);
		fuses[2] = avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSES_READ_LOW);
		fuses[3] = avr8_jtag_command_data(jtag_index, AVR8_CMD_LOCK_READ);
		if (src < AVR8_FUSES_START + 4)
			memcpy(dest, fuses + (src & 3), ((len - 1) & 3) + 1);
	}
	avr8_jtag_command(jtag_index, AVR8_PROG_ENABLE, 0);
}

static void avr8_mem_read(target *t, void *dest, target_addr src, size_t len)
{
	avr8_mem_read_firmware(t->jtag_index, dest, src, len);
}

static int avr8_fuses_write(uint32_t jtag_index, target_addr dest,
							const void *src, size_t len)
{
	DEBUG_TARGET("Fuse write len %ld\n", len);
	dest &= 3; /* clip */
	uint8_t fuses[3];
	if (len > (3 - dest))
		len = 3 - dest;
	if (len < 3) /* preload old value */
		avr8_mem_read_firmware(jtag_index, fuses, AVR8_FUSES_START, 3);
	memcpy(fuses + dest, src, len);
	if (fuses[1] & AVR8_JTAGEN) {
		DEBUG_WARN("Disable JTAG access denied\n");
		return -1;
	}
	DEBUG_TARGET("Setting Ext %x High %x Low %x\n",
			   fuses[0], fuses[1], fuses[2]);
	avr8_jtag_command(jtag_index, AVR8_PROG_ENABLE, AVR8_CMD_PROG_ENABLE);
	avr8_jtag_command(jtag_index,  AVR8_PROG_COMMANDS,
					  AVR8_CMD_ENTER_FUSES_WRITE);
	avr8_jtag_command_data(jtag_index, fuses[0] | AVR8_CMD_FUSE_DATA);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSE_EXT_WRITE_SEQ);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSE_EXT_WRITE_SEQ2);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSE_EXT_WRITE_SEQ);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSE_EXT_WRITE_SEQ);
	uint16_t res;
	do {
		res = avr8_jtag_command_data(jtag_index, AVR8_CMD_POLL_WRITE_COMPLETE);
	} while (!(res & AVR8_POLL_DONE));
	avr8_jtag_command_data(jtag_index, fuses[1] | AVR8_CMD_FUSE_DATA);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSE_HIGH_WRITE_SEQ);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSE_HIGH_WRITE_SEQ2);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSE_HIGH_WRITE_SEQ);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSE_HIGH_WRITE_SEQ);
	do {
		res = avr8_jtag_command_data(jtag_index, AVR8_CMD_POLL_WRITE_COMPLETE);
	} while (!(res & AVR8_POLL_DONE));
	avr8_jtag_command_data(jtag_index, fuses[2] | AVR8_CMD_FUSE_DATA);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSE_LOW_WRITE_SEQ);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSE_LOW_WRITE_SEQ2);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSE_LOW_WRITE_SEQ);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_FUSE_LOW_WRITE_SEQ);
	do {
		res = avr8_jtag_command_data(jtag_index, AVR8_CMD_POLL_FUSE_COMPLETE);
	} while (!(res & AVR8_POLL_DONE));
	avr8_jtag_command(jtag_index, AVR8_PROG_ENABLE, 0);
	return 0;
}

/* As we use buffered write, we only write full pages! */
void avr8_flash_write(uint32_t jtag_index, target_addr dest,
							const void *src, size_t len)
{
	avr8_jtag_command(jtag_index, AVR8_PROG_COMMANDS,
					  AVR8_CMD_ENTER_FLASH_WRITE);
	avr8_jtag_command_data(jtag_index, ((dest >> 9) & 0xff) |
						   AVR8_CMD_LOAD_ADDR_HIGH);
	avr8_jtag_command_data(jtag_index, ((dest >> 1) & 0xff) |
						   AVR8_CMD_LOAD_ADDR_LOW);
	jtag_dev_shift_ir(&jtag_proc, jtag_index, AVR8_PROG_PAGELOAD);
	size_t l = 0;
	for (; l < len; l++)
		jtag_dev_shift_dr(&jtag_proc, jtag_index, NULL,
						  src++, 8);
	avr8_jtag_command(jtag_index, AVR8_PROG_COMMANDS,
					  AVR8_CMD_POLL_WRITE_COMPLETE);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_POLL_WRITE_TRIGGER);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_POLL_WRITE_COMPLETE);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_POLL_WRITE_COMPLETE);
	uint16_t res;
	do {
		res = avr8_jtag_command_data(jtag_index,
									 AVR8_CMD_POLL_WRITE_COMPLETE);
	} while (!(res & AVR8_POLL_DONE));
}

static int avr8_lock_write(uint32_t jtag_index, target_addr dest,
							const void *src, size_t len)
{
	DEBUG_TARGET("avr8_lock_write len %d to %04x, data %x\n", len,
				 dest, *(uint8_t*) src);
	(void)len;
	(void)dest;
	uint8_t data = *(uint8_t *) src;
	if ((data & 0xc0) != 0xc0) {
		DEBUG_WARN("Unexpected Lock Byte %x\n", data);
		return -1;
	}
	avr8_jtag_command(jtag_index, AVR8_PROG_COMMANDS,
					  AVR8_CMD_ENTER_LOCK_WRITE);
	avr8_jtag_command_data(jtag_index, data | AVR8_CMD_FUSE_DATA);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_LOCK_WRITE_SEQ);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_LOCK_WRITE_SEQ2);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_LOCK_WRITE_SEQ);
	avr8_jtag_command_data(jtag_index, AVR8_CMD_LOCK_WRITE_SEQ);
	uint16_t res;
	do {
		res = avr8_jtag_command_data(jtag_index, AVR8_CMD_LOCK_WRITE_POLL);
	} while (!(res & AVR8_POLL_DONE));
	return 0;

}

/* "EEPROM" via JTAG behaves like a flash with a small page size,
 * typical 8 bytes.
 * There is no way to erase EEPROM via JTAG other than a full chip
 * erase with EESAV fuse unset
 */
static void avr8_eeprom_write(uint32_t jtag_index, target_addr dest,
							const void *src, size_t len)
{
	DEBUG_TARGET("avr8_eeprom_write len %d to %04x\n", len, dest & 0xffff);
	dest -= AVR8_EEPROM_START;
	avr8_jtag_command(jtag_index, AVR8_PROG_COMMANDS,
					  AVR8_CMD_ENTER_EEPROM_WRITE);
	while (len) {
		avr8_jtag_command_data(jtag_index, ((dest >> 8) & 0xff) |
							   AVR8_CMD_LOAD_ADDR_HIGH);
		do {
			avr8_jtag_command_data(jtag_index, ((dest >> 0) & 0xff) |
								   AVR8_CMD_LOAD_ADDR_LOW);
			avr8_jtag_command_data(jtag_index, (*(uint8_t *) src) |
								   AVR8_CMD_EEPROM_DATA);
			avr8_jtag_command_data(jtag_index, AVR8_CMD_EEPROM_DATA_SEQ);
			avr8_jtag_command_data(jtag_index, AVR8_CMD_EEPROM_DATA_SEQ2);
			avr8_jtag_command_data(jtag_index, AVR8_CMD_EEPROM_DATA_SEQ);
			dest++;
			src++;
			len--;
		} while (len && (dest & (AVR8_EEPROM_PAGE_SIZE - 1)));
		avr8_jtag_command_data(jtag_index, AVR8_CMD_EEPROM_PAGE_SEQ);
		avr8_jtag_command_data(jtag_index, AVR8_CMD_EEPROM_PAGE_SEQ2);
		avr8_jtag_command_data(jtag_index, AVR8_CMD_EEPROM_PAGE_SEQ);
		avr8_jtag_command_data(jtag_index, AVR8_CMD_EEPROM_PAGE_SEQ);
		uint16_t res;
		do {
			res = avr8_jtag_command_data(jtag_index, AVR8_CMD_EEPROM_PAGE_POLL);
		} while (!(res & AVR8_POLL_DONE));
	}
}

int avr8_mem_write_firmware(uint32_t jtag_index, target_addr dest,
							const void *src, size_t len)
{
	DEBUG_TARGET("mem_write len %d to %04x\n", len, dest);
	int res = 0;
	avr8_jtag_command(jtag_index, AVR8_PROG_ENABLE, AVR8_CMD_PROG_ENABLE);
	if (dest >= AVR8_LOCK_START)
		res = avr8_lock_write(jtag_index, dest, src, len);
	else if (dest >=  AVR8_FUSES_START)
		res = avr8_fuses_write(jtag_index, dest, src, len);
	else if (dest >=  AVR8_EEPROM_START)
		avr8_eeprom_write(jtag_index, dest, src, len);
	else
		avr8_flash_write(jtag_index, dest, src, len);
	avr8_jtag_command(jtag_index, AVR8_PROG_ENABLE, 0);
	return res;
}

/* As we use buffered write, we only write full pages! */
static int avr8_mem_write(struct target_flash *f, target_addr dest,
							const void *src, size_t len)
{
	DEBUG_TARGET("mem_write len %d to %04x\n", len, dest);
	if (len != f->blocksize) {
		DEBUG_WARN("Avr8_flash_write unexpected write\n");
		return -1;
	}
#if PC_HOSTED == 1
	void remote_avr8_mem_write(uint32_t jtag_index, target_addr dest,
								const void *src, size_t len);
	if (f->t->mem_read == avr8_mem_read)
		avr8_mem_write_firmware(f->t->jtag_index, dest, src,len);
	else
		remote_avr8_mem_write(f->t->jtag_index, dest, src,len);
#else
	avr8_mem_write_firmware(f->t->jtag_index, dest, src,len);
#endif
	return 0;
}

/* Bare desc without memory map. Map probably provided by avr-gdb.*/
static const char tdesc_avr8[] =
	"<?xml version=\"1.0\"?>\n"
	"<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\n"
	"<target version=\"1.0\">\n"
	"  <architecture>avr</architecture>\n"
	"</target>\n";

/* AVR is kept under reset as long as AVR_RESET is 1*/
static void avr8_reset(target *t)
{
	DEBUG_TARGET("AVR Reset\n");
	avr8_jtag_command(t->jtag_index, AVR8_RESET, 1);
	avr8_jtag_command(t->jtag_index, AVR8_RESET, 0);
}

/* No single stepping for now!*/
static void avr8_halt_resume(target *t, bool step)
{
	(void) step;
	DEBUG_TARGET("%s AVR HALT_RESUME%s\n", (step)? "Single Step": "");
	avr8_jtag_command(t->jtag_index, AVR8_RESET, 0);
}

/* Keep the target under reset while attached for now! */
static bool avr8_attach(target *t)
{
	avr8_jtag_command(t->jtag_index, AVR8_JTAG_INS_HALT, 0);
	return true;
}

/* Keep the target under resey while attached for now! */
static void avr8_detach(target *t)
{
	avr8_jtag_command(t->jtag_index, AVR8_JTAG_INS_RUN, 0);
}

static void avr8_regs_read(target *t, void *data)
{
	(void) t;
	memset(data, 0, AVR8_REG_SIZE);
}

static int avr8_flash_erase(struct target_flash *f,
							target_addr addr, size_t len)
{
	target *t = f->t;
	(void) addr;
	(void) len;
	DEBUG_TARGET("FLash erase add %05x len %05x\n", addr, len);
	if (addr >= AVR8_EEPROM_START) {
		DEBUG_INFO("Erase skipped for EEPROM/Fuses!\n");
		return -1;
	}
	avr8_jtag_command(t->jtag_index, AVR8_PROG_ENABLE, AVR8_CMD_PROG_ENABLE);
	avr8_jtag_command(t->jtag_index, AVR8_PROG_COMMANDS,
					  AVR8_CMD_CHIP_ERASE_SEQ0);
	avr8_jtag_command_data(t->jtag_index, AVR8_CMD_CHIP_ERASE_SEQ1);
	avr8_jtag_command_data(t->jtag_index, AVR8_CMD_CHIP_ERASE_POLL);
	avr8_jtag_command_data(t->jtag_index, AVR8_CMD_CHIP_ERASE_POLL);
	uint16_t res;
	do {
		res = avr8_jtag_command_data(t->jtag_index, AVR8_CMD_CHIP_ERASE_POLL);
	} while (!(res & AVR8_POLL_DONE));
	avr8_jtag_command(t->jtag_index, AVR8_PROG_ENABLE, 0);
	return 0;
}

static void avr8_add_flash(target *t, uint32_t addr, size_t length,
						   size_t blocksize)
{
	struct target_flash *f = calloc(1, sizeof(*f));
	if (!f) {			/* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return;
	}
	f->start = addr;
	f->length = length;
	f->erased = 0xff;
	f->blocksize = blocksize;
	f->write = avr8_mem_write;
	if (addr < AVR8_EEPROM_START)
		f->erase = avr8_flash_erase;
	target_add_flash(t, f);
}

void avr8_handler(jtag_dev_t *jd)
{
	target *t = target_new();
	if (!t) {
		return;
	}
	t->jtag_index = jd->jd_dev;
	size_t length;
	uint32_t blocksize;

	switch ((jd->jd_idcode >> 12) & 0xffff) {
	case 0x9403:
		t->driver = "Atmega16";
		length = 0x4000;
		blocksize = 128;
		break;
	case 0x9581:
		t->driver = "At90Can32";
		length = 0x8000;
		blocksize = 256;
		break;
	case 0x9681:
		t->driver = "At90Can64";
		blocksize = 256;
		length = 0x10000;
		break;
	case 0x9781:
		t->driver = "At90Can128";
		blocksize = 256;
		length = 0x20000;
		break;
	default:
		free(t);
		return;
	}
	avr8_add_flash(t, 0, length, blocksize);
	avr8_add_flash(t, AVR8_EEPROM_START , 0x4000, 8);
	avr8_add_flash(t, AVR8_FUSES_START  ,      3, 1);
	avr8_add_flash(t, AVR8_FUSES_START+3,      1, 1);
	t->core = "AVR8";
	t->reset = avr8_reset;
	t->halt_resume = avr8_halt_resume;
	t->attach = avr8_attach;
	t->detach = avr8_detach;
	t->tdesc = tdesc_avr8;
	t->regs_size = AVR8_REG_SIZE;
	t->regs_read = avr8_regs_read;
#if PC_HOSTED == 1
	void avr8_defaults(target *t);
	t->mem_read = 0;
	avr8_defaults(t);
	if (!t->mem_read)
		t->mem_read = avr8_mem_read;
#else
	t->mem_read = avr8_mem_read;
#endif
	uint8_t fuses[3];
	avr8_mem_read_firmware(t->jtag_index, fuses, AVR8_FUSES_START, 3);
	if (fuses[1] & 0x80)
		DEBUG_WARN("OCDEN disabled!\n");
	target_add_commands(t, avr8_cmd_list, "AVR8");
	return;
}

static bool avr8_cmd_fuses (target* t, int argc, char** argv)
{
	if (argc > 1) {
		if (!strncmp(argv[1], "help", 3)) {
			tc_printf(t, "usage: monitor option|fuses [EXT HIGH LOW]\n");
			tc_printf(t, "\t write option if 3 bytes are give, else read\n");

		} else if (!strncmp(argv[1], "wri", 3)) {
			if (argc != 5) {
				DEBUG_WARN("Invalid write data\n");
				return false;
			}
			uint16_t ext_byte, high_byte, low_byte;
			ext_byte  = strtol(argv[2], NULL, 0);
			high_byte = strtol(argv[3], NULL, 0);
			low_byte  = strtol(argv[4], NULL, 0);
			if ((ext_byte > 0xff) || (high_byte > 0xff) || (low_byte > 0xff)) {
				DEBUG_WARN("Invalid write values\n");
				return false;
			}
			if ((ext_byte & 0xf0) != 0xf0) {
				DEBUG_WARN("Invalid ext fuse value 0x%02x\n", ext_byte);
				return false;
			}
			uint8_t fuses[3];
			struct target_flash f;
			f.t = t;
			fuses[0] = ext_byte & 0xff;
			fuses[1] = high_byte & 0xff;
			fuses[2] = low_byte & 0xff;
			avr8_mem_write(&f, AVR8_FUSES_START, fuses, 3);
		}
	}
	/* Report current fuse setting */
	uint8_t fuses[4];
	avr8_mem_read(t, fuses, AVR8_FUSES_START, 4);
	tc_printf(t, "Ext %02x High %02x Low %02x lock %02x\n",
			   fuses[0], fuses[1], fuses[2], fuses[3]);
	return true;
}

static bool avr8_cmd_eeprom (target* t, int argc, char** argv)
{
	if (argc < 2) {
		tc_printf(t, "usage: monitor eeprom <addr> [<data>]\n");
		tc_printf(t, "\treads one byte at <addr> with no <data], "
				  "otherwise write\n");
		return false;
	}
	uint16_t addr = strtol(argv[1], NULL, 0);
	if (argc == 2) {
		uint8_t res;
		avr8_mem_read(t, &res,  AVR8_EEPROM_START + addr, 1);
		tc_printf(t, "EEPROM 0x%04x: 0x%02x\n", addr, res);
	} else {
		struct target_flash f;
		f.t = t;
		uint8_t data = strtol(argv[2], NULL, 0);
		avr8_mem_write(&f, AVR8_EEPROM_START + addr, &data, 1);
	}
	return true;
}
