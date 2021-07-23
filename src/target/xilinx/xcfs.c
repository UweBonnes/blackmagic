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

/* This file handles XCF deviced
 *
 * Base on code from xc3prog
 * Copyright (C) 2004-2011 Andrew Rogers
 *              2009 Uwe Bonnes
 */

struct xcfs_priv {
	unsigned int total_size;
	unsigned int block_size;
	bool use_optimized_algs;
};

enum XCFS_JTAG_REG {
	XCFS_SERASE                         = 0x0a,
	XCFS_ISCTESTSTATUS                  = 0xe3,
	XCFS_ISCTESTSTATUSISC_ENABLE        = 0xe8,
	XCFS_ISCTESTSTATUSISC_PROGRAM       = 0xea,
	XCFS_ISCTESTSTATUSISC_ADDRESS_SHIFT = 0xeb,
	XCFS_ISCTESTSTATUSISC_ERASE         = 0xec,
	XCFS_ISCTESTSTATUSISC_DATA_SHIFT    = 0xed,
	XCFS_ISCTESTSTATUSCONFIG            = 0xee,
	XCFS_ISCTESTSTATUSISC_READ          = 0xef,
	XCFS_ISCTESTSTATUSISC_DISABLE       = 0xf0,
	XCFS_ISCTESTSTATUSIDCODE            = 0xfe,
	XCFS_ISCTESTSTATUSBYPASS            = 0xff,
};

#define ISC_ENABLE_MAGIC 0x34

static void xcfs_add_array(target *t, struct xcfs_priv *priv)
{
	struct target_flash *f = calloc(1, sizeof(*f));
	if (!f) {			/* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return;
	}
	f->start = 0;
	f->length = priv->total_size >> 3;
	f->erased = 0xff; /*?*/
	f->blocksize = priv->block_size >> 3;
	DEBUG_WARN("block zize %d\n",f->blocksize );
//	f->write = xc2c_array_write;
//	f->erase = xc2c_array_erase;
	target_add_flash(t, f);
}

static void xcfs_mem_read(target *t, void *dest, target_addr src, size_t len)
{
	DEBUG_TARGET("mem_read len %d from %04x\n", len, src);
//	unsigned int block_address = 0;
	(void) t;
	(void) dest;
	(void) src;
	(void) len;

}

void xcfs_handler(jtag_dev_t *jd)
{
	target *t = target_new();
	if (!t) {
		return;
	}
	struct xcfs_priv *priv = calloc(1, sizeof(*priv));
	if (!priv) {			/* calloc failed: heap exhaustion */
		DEBUG_WARN("calloc: failed in %s\n", __func__);
		return;
	}
	int size_ind = (jd->jd_idcode & 0x000ef000) >> 12;
	t->priv = priv;
	t->priv_free = free;
	switch (size_ind) {
	case 0x23:
		t->driver = "XC18V512";
		priv->total_size = 1 << 19;
		priv->block_size = 2048;
		break;
	case 0x24:
		t->driver = "XC18V01";
		priv->total_size = 1 << 20;
		priv->block_size = 2048;
		break;
	case 0x25:
		t->driver = "XC18V02";
		priv->total_size = 2 << 20;
		priv->block_size = 4096;
		break;
	case 0x26:
		t->driver = "XC18V04";
		priv->total_size = 4 << 20;
		priv->block_size = 4096;
		break;
	case 0x44:
		t->driver = "XCF01S";
		priv->total_size = 1 << 20;
		priv->block_size = 2048;
		priv->use_optimized_algs = true;
		break;
	case 0x45:
		t->driver = "XCF02S";
		priv->total_size = 2 << 20;
		priv->block_size = 4096;
		priv->use_optimized_algs = true;
		break;
	case 0x46:
		t->driver = "XCF04S";
		priv->total_size = 4 << 20;
		priv->block_size = 4096;
		priv->use_optimized_algs = true;
		break;
	default:
		DEBUG_WARN("Unknown XCF device size code %x\n", size_ind);
		return;
	}
	t->core = "Serial Configuration";
	xcfs_add_array(t, priv);
	t->mem_read = xcfs_mem_read;
}
