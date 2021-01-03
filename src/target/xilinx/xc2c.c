/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2021 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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

#include "general.h"
#include "target_internal.h"
#include "xilinx.h"

#include <stdint.h>
#include <stdio.h>

/* This file handles XC2C deviced
 *
 * Similar to xc3sprog
 */
/* generate tables with
int main(void)
{
    int i, j;
    uint8_t c;
    printf("const uint8_t gray_code_table[256] = {\n");
    for (i=0; i<0x100; i++) {
        c= 0;
        if(i & 0x80)
            c |= 0x80;
        for (j=6; j>=0; j--) {
            int pat0 = 1<< j;
            int pat1 = 1<<(1+j);
            if (!(i & pat0 ) && (i & pat1))
                c |= (1<<j);
            if ( (i & pat0 ) && !(i & pat1))
                c |= (1<<j);
        }
        printf("0x%02x,%c", c, ((i & 0x7) == 0x7)? '\n': ' ');
    }
    printf("};\n");
    return 0;
}
*/
const uint8_t gray_code_table[256] = {
	0x00, 0x01, 0x03, 0x02, 0x06, 0x07, 0x05, 0x04,
	0x0c, 0x0d, 0x0f, 0x0e, 0x0a, 0x0b, 0x09, 0x08,
	0x18, 0x19, 0x1b, 0x1a, 0x1e, 0x1f, 0x1d, 0x1c,
	0x14, 0x15, 0x17, 0x16, 0x12, 0x13, 0x11, 0x10,
	0x30, 0x31, 0x33, 0x32, 0x36, 0x37, 0x35, 0x34,
	0x3c, 0x3d, 0x3f, 0x3e, 0x3a, 0x3b, 0x39, 0x38,
	0x28, 0x29, 0x2b, 0x2a, 0x2e, 0x2f, 0x2d, 0x2c,
	0x24, 0x25, 0x27, 0x26, 0x22, 0x23, 0x21, 0x20,
	0x60, 0x61, 0x63, 0x62, 0x66, 0x67, 0x65, 0x64,
	0x6c, 0x6d, 0x6f, 0x6e, 0x6a, 0x6b, 0x69, 0x68,
	0x78, 0x79, 0x7b, 0x7a, 0x7e, 0x7f, 0x7d, 0x7c,
	0x74, 0x75, 0x77, 0x76, 0x72, 0x73, 0x71, 0x70,
	0x50, 0x51, 0x53, 0x52, 0x56, 0x57, 0x55, 0x54,
	0x5c, 0x5d, 0x5f, 0x5e, 0x5a, 0x5b, 0x59, 0x58,
	0x48, 0x49, 0x4b, 0x4a, 0x4e, 0x4f, 0x4d, 0x4c,
	0x44, 0x45, 0x47, 0x46, 0x42, 0x43, 0x41, 0x40,
	0xc0, 0xc1, 0xc3, 0xc2, 0xc6, 0xc7, 0xc5, 0xc4,
	0xcc, 0xcd, 0xcf, 0xce, 0xca, 0xcb, 0xc9, 0xc8,
	0xd8, 0xd9, 0xdb, 0xda, 0xde, 0xdf, 0xdd, 0xdc,
	0xd4, 0xd5, 0xd7, 0xd6, 0xd2, 0xd3, 0xd1, 0xd0,
	0xf0, 0xf1, 0xf3, 0xf2, 0xf6, 0xf7, 0xf5, 0xf4,
	0xfc, 0xfd, 0xff, 0xfe, 0xfa, 0xfb, 0xf9, 0xf8,
	0xe8, 0xe9, 0xeb, 0xea, 0xee, 0xef, 0xed, 0xec,
	0xe4, 0xe5, 0xe7, 0xe6, 0xe2, 0xe3, 0xe1, 0xe0,
	0xa0, 0xa1, 0xa3, 0xa2, 0xa6, 0xa7, 0xa5, 0xa4,
	0xac, 0xad, 0xaf, 0xae, 0xaa, 0xab, 0xa9, 0xa8,
	0xb8, 0xb9, 0xbb, 0xba, 0xbe, 0xbf, 0xbd, 0xbc,
	0xb4, 0xb5, 0xb7, 0xb6, 0xb2, 0xb3, 0xb1, 0xb0,
	0x90, 0x91, 0x93, 0x92, 0x96, 0x97, 0x95, 0x94,
	0x9c, 0x9d, 0x9f, 0x9e, 0x9a, 0x9b, 0x99, 0x98,
	0x88, 0x89, 0x8b, 0x8a, 0x8e, 0x8f, 0x8d, 0x8c,
	0x84, 0x85, 0x87, 0x86, 0x82, 0x83, 0x81, 0x80,
};

static const unsigned char table[] = {
	0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
	0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
	0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
	0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
	0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
	0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
	0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
	0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
	0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
	0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
	0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
	0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
	0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
	0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
	0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
	0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
	0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
	0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
	0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
	0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
	0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
	0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
	0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
	0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
	0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
	0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
	0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
	0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
	0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
	0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
	0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
	0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
};

struct xc2c_priv {
	unsigned int row_size;
	unsigned int row_max;
	unsigned int addr_size;
};

enum XC2C_JTAG_REG {
	XC2C_IDCODE        = 0x01,
	XC2C_ISC_ENABLE_OTF= 0xe4,
	XC2C_ISC_ENABLE    = 0xe8,
	XC2C_ISC_SRAM_READ = 0xe7,
	XC2C_ISC_WRITE     = 0xe6,
	XC2C_ISC_ERASE     = 0xed,
	XC2C_ISC_ERASE_ALL = 0x14,
	XC2C_ISC_PROGRAM   = 0xea,
	XC2C_ISC_READ      = 0xee,
	XC2C_ISC_INIT      = 0xf0,
	XC2C_ISC_DISABLE   = 0xc0,
	XC2C_USERCODE      = 0xfd,
	XC2C_BYPASS        = 0xff,
};

const uint8_t ones[4] = {0xff, 0xff, 0xff, 0xff};

void jtag_dev_shift_dr_start(jtag_proc_t *jp, uint8_t jd_index, uint8_t *dout,
							 const uint8_t *din, int ticks)
{
	jtag_dev_t *d = &jtag_devs[jd_index];
	jtagtap_shift_dr();
	jp->jtagtap_tdi_seq(0, ones, d->dr_prescan);
	jp->jtagtap_tdi_tdo_seq((dout)? (void*)dout : NULL, 0, (void*)din, ticks);
}

void jtag_dev_shift_dr_finish(jtag_proc_t *jp, uint8_t jd_index, uint8_t *dout,
							  const uint8_t *din, int ticks)
{
	jtag_dev_t *d = &jtag_devs[jd_index];
	jp->jtagtap_tdi_tdo_seq((dout) ? (void*)dout : NULL,
							d->dr_postscan ? 0 : 1, (void*)din, ticks);
	if (d->dr_postscan)
		jp->jtagtap_tdi_seq(1, ones, d->dr_postscan);
	jtagtap_return_idle();
}

static void array_erase(target *t)
{
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_BYPASS);
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_ISC_ENABLE_OTF);
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_ISC_ERASE);
	platform_delay(100);
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_ISC_DISABLE);
}

static int xc2c_array_erase(struct target_flash *f,
							target_addr addr, size_t len)
{
	DEBUG_TARGET("Array erase from %5d len %d\n", addr, len);
	target *t = f->t;
	array_erase(t);
	return 0;
}

static bool xc2c_cmd_erase_mass(target *t, int argc, const char **argv)
{
	DEBUG_TARGET("Mass erase\n");
	(void) argc;
	(void) argv;
	array_erase(t);
	return true;
}

static bool xc2c_cmd_read_usercode(target* t, int argc, char** argv)
{
	(void) argc;
	(void) argv;
	uint8_t res[4];
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_ISC_ENABLE_OTF);
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_USERCODE);
	jtag_dev_shift_dr(&jtag_proc, t->jtag_index, res, NULL, sizeof(res) * 8);
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_ISC_DISABLE);
	tc_printf(t, "Usercode: 0x%02x%02x%02x%02x\n",
			  res[3], res[2], res[1], res[0]);
	return true;
}

static const struct command_s xc2c_cmd_list[] = {
	{ "usercode", (cmd_handler) xc2c_cmd_read_usercode, "Read usercode"},
	{ "erase_mass",  xc2c_cmd_erase_mass, "Erase entire chip"},
	{ NULL, NULL, NULL },
};

static int xc2c_array_write(struct target_flash *f, target_addr dest,
							const void *src, size_t len)
{
	DEBUG_TARGET("xc2c array write len %d to %04x: ", len, dest & 0xffff);
	target *t = f->t;
	struct xc2c_priv *priv = t->priv;
	uint8_t buf[len];
	const uint8_t *source = src;
	unsigned int row_start = (dest * 8) / priv->row_size;
	unsigned int row_end = ((dest + len) * 8) / priv->row_size;
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_ISC_ENABLE_OTF);
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_ISC_PROGRAM);
	/* Provide bit-reversed image of src*/
	for (size_t i = 0; i < len; i++)
		buf[i] = table[source[i]];
	int buff_bit_index = 0;
	for (unsigned int i = row_start; i < row_end; i++) {
		/* transfer row_size bits to dout */
		uint8_t din[(priv->row_size + 7) / 8];
		memset(din, f->erased, sizeof(din));
		for (unsigned int j = 0; j < priv->row_size; j++) {
			bool bit = buf[buff_bit_index / 8] & (1 << (buff_bit_index & 7));
			if (!bit)
				din[j / 8] &= ~(1 << (j & 7));
			buff_bit_index++;
		}
		uint8_t addr = table[gray_code_table[i]] >> (8 - priv->addr_size);
		jtag_dev_shift_dr_start(&jtag_proc, t->jtag_index, NULL, din,
								priv->row_size);
		jtag_dev_shift_dr_finish(&jtag_proc, t->jtag_index, NULL, &addr,
								 priv->addr_size);
		platform_delay(10);
	}
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_ISC_DISABLE);
	return 0;
}

static void xc2c_add_array(target *t, uint32_t addr, size_t length,
						   size_t blocksize)
{
	struct target_flash *f = calloc(1, sizeof(*f));
	if (!f) {			/* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return;
	}
	f->start = addr;
	f->length = length;
	f->erased = 0xff; /*?*/
	while (blocksize & 7)
		blocksize <<= 1;
	f->blocksize = blocksize / 8; /* Make sure we handle full byte */
	f->write = xc2c_array_write;
	f->erase = xc2c_array_erase;
	target_add_flash(t, f);
}

static void xc2c_array_read(target *t, void *dest, target_addr src, size_t len)
{
	DEBUG_TARGET("array_read, src %08x, len %d\n", src, len);
	struct xc2c_priv *priv = t->priv;
	uint8_t data[priv->row_size];
	unsigned int row_start = (src * 8) / priv->row_size;
	unsigned int row_end = ((src + len) * 8) / priv->row_size;
	unsigned int first_row_offset = (src * 8) -
		(row_start * priv->row_size);
	if ((row_start > priv->row_max) || (row_start > priv->row_max)) {
		DEBUG_WARN("xc2c_array_read outside of array\n");
	}
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_BYPASS);
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_ISC_ENABLE_OTF);
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_ISC_READ);
	data[0] =  table[gray_code_table[row_start]] >> (8 - priv->addr_size);
	jtag_dev_shift_dr(&jtag_proc, t->jtag_index, NULL, data, priv->addr_size);
	uint8_t buf[len];
	memset(buf, 0, (len * 8 + priv->row_size - 1) / 8);
	int buf_index = 0;
	for (unsigned int i = row_start; i <= row_end; i++) {
		uint8_t postdata;
		postdata = table[gray_code_table[(i + 1)]] >> (8 - priv->addr_size);
		jtag_dev_shift_dr_start(&jtag_proc, t->jtag_index, data, NULL,
								priv->row_size);
		jtag_dev_shift_dr_finish(&jtag_proc, t->jtag_index, NULL, &postdata,
								 priv->addr_size);
		/* bitwise transfer as start or end may not be on byte border*/
		for (unsigned int j = first_row_offset; j < priv->row_size; j++) {
			bool bit = data[j / 8]  & (1 << (j & 7));
			if (bit)
				buf[buf_index / 8] |= (1 << (buf_index & 7));
			buf_index++;
		}
		first_row_offset = 0;
	}
	jtag_dev_shift_ir(&jtag_proc, t->jtag_index, XC2C_ISC_DISABLE);
	uint8_t *dest_b = dest;
	for (size_t i = 0; i < len; i++)
		dest_b[i] = table[buf[i]];
}

void xc2c_handler(jtag_dev_t *jd)
{
	target *t = target_new();
	if (!t) {
		return;
	}
	struct xc2c_priv *priv = calloc(1, sizeof(*priv));
	if (!priv) {			/* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return;
	}
	t->priv = priv;
	t->priv_free = free;
	switch ((jd->jd_idcode >> 12) & 0xfff) {
	case 0xc1b:
		t->driver = "XC2C32A_QF32";
		break;
	case 0xc1c:
		t->driver ="XC2C32_VQ44";
		break;
	case 0xc1d:
		t->driver ="XC2C32_PC44/64";
		break;
	case 0xc5a:
		t->driver ="XC2C64-PC44";
		break;
	case 0xc5b:
		t->driver ="XC2C64-CP132";
		break;
	case 0xc5c:
		t->driver ="XC2C64-VQ100";
		break;
	case 0xc5d:
		t->driver ="XC2C64-CP56";
		break;
	case 0xc5e:
		t->driver ="XC2C64-VQ44";
		break;
	case 0xd1d:
		t->driver ="XC2C32A_PC44";
		break;
	case 0xd4a:
		t->driver ="XC2C256_VQ100";
		break;
	case 0xd4b:
		t->driver ="XC2C256_CP132";
		break;
	case 0xd4c:
		t->driver ="XC2C256_TQ144";
		break;
	case 0xd4d:
		t->driver ="XC2C256_PQ208";
		break;
	case 0xd4e:
		t->driver ="XC2C256_FT256";
		break;
	case 0xd5a:
		t->driver ="XC2C384_FG324";
		break;
	case 0xd5b:
		t->driver ="XC2C384_CP204";
		break;
	case 0xd5c:
		t->driver ="XC2C384_TQ144";
		break;
	case 0xd5d:
		t->driver ="XC2C384_PQ208";
		break;
	case 0xd5e:
		t->driver ="XC2C384_FT256";
		break;
	case 0xd7a:
		t->driver ="XC2C512_FG324";
		break;
	case 0xd7c:
		t->driver ="XC2C512_PQ208";
		break;
	case 0xd7e:
		t->driver ="XC2C512_FT256";
		break;
	case 0xd8a:
		t->driver ="XC2C128_VQ100";
		break;
	case 0xd8b:
		t->driver ="XC2C128_CP132";
		break;
	case 0xd8c:
		t->driver ="XC2C128_TQ144";
		break;
	case 0xd8e:
		t->driver ="XC2C128_FT256";
		break;
	case 0xe1b:
		t->driver ="XC2C32A_CP56";
		break;
	case 0xe1c:
		t->driver = "XC2C32A_VQ44";
		break;
	case 0xe1d:
		t->driver ="XC2C32A_PC44/64";
		break;
	case 0xe59:
		t->driver ="XC2C64A-QF48";
		break;
	case 0xe5a:
		t->driver ="XC2C64A-PC44";
		break;
	case 0xe5b:
		t->driver ="XC2C64A-CP132";
		break;
	case 0xe5c:
		t->driver ="XC2C64A-VQ100";
		break;
	case 0xe5d:
		t->driver ="XC2C64A-CP56";
		break;
	case 0xe5e:
		t->driver ="XC2C64A-VQ44";
		break;
	}
	switch ((jd->jd_idcode & 0x001f0000) >> 16) {
    case 0x01: /*XC2C32(A) */
		priv->row_size  = 260;
		priv->row_max   = 48;
		priv->addr_size = 6;
		break;
    case 0x5: /*XC2C64(A) */
		priv->row_size  = 274;
		priv->row_max   = 96;
		priv->addr_size = 7;
		break;
    case 0x18: /*XC2C128 */
		priv->row_size  = 752;
		priv->row_max   = 80;
		priv->addr_size = 7;
		break;
    case 0x14: /*XC2C256 */
		priv->row_size  = 1364;
		priv->row_max   = 96;
		priv->addr_size = 7;
		break;
    case 0x15: /*XC2C384 */
		priv->row_size  = 1868;
		priv->row_max   = 120;
		priv->addr_size = 7;
		break;
    case 0x17: /*XC2C512 */
		priv->row_size  = 1980;
		priv->row_max   = 160;
		priv->addr_size = 8;
		break;
	}
	/* there are two extra rows for security/done and usercode bits*/
	priv->row_max += 1;
	DEBUG_TARGET("XC2C: row_size %d # rows %d addr_size %d\n",
			   priv->row_size, priv->row_max, priv->addr_size);
	t->core = "XC2C/CPLD";
	t->jtag_index = jd->jd_dev;
	target_add_commands(t, xc2c_cmd_list, "XC2C");
	xc2c_add_array(t, 0, priv->row_max * priv->row_size / 8,
				   priv->row_size);
	t->mem_read = xc2c_array_read;
	return;
}
