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

/* This file tells about the connected XILINX device
 *
 * Similar to xc3sprog
 */

#include "general.h"
#include "target_internal.h"
#include "xilinx.h"

#include <stdint.h>
#include <stdio.h>

unsigned int xilinx(
	uint32_t product,void (**jd_handler)(jtag_dev_t *jd),
	jtag_dev_t *dev)
{
	unsigned int expected_irlen = (unsigned int)-1;
	if        (product > 0xa00) {
		expected_irlen = 9;
		dev->jd_descr = "Xilinx SystemACE";
	} else if (product > 0x900) {
		expected_irlen =  8;
		dev->jd_descr = "Xilinx XC95";
	} else if (product > 0x6d0) {
		expected_irlen =  8;
		dev->jd_descr = "XILINX XC2CxxA:";
		*jd_handler = &xc2c_handler;
	} else if (product > 0x6c0) {
		expected_irlen =  8;
		dev->jd_descr = "XILINX XC2Cxx:";
		*jd_handler = &xc2c_handler;
	} else if (product > 0x505) {
		dev->jd_descr = "XILINX XCFxxP";
		expected_irlen =  16;
		dev->jd_descr = "XC2FxxP";
	} else if (product > 0x503) {
		dev->jd_descr = "XILINX XCFxxS";
		expected_irlen =  8;
		*jd_handler = &xcfs_handler;
	} else if (product > 0x502) {
		dev->jd_descr = "XILINX XC18V";
		expected_irlen =  8;
	} else if (product > 0x450) {
		dev->jd_descr = "XILINX XC5VTXxxxT";
		expected_irlen =  10;
	} else if (product > 0x420) {
		dev->jd_descr = "XILINX XC6V";
		expected_irlen =  10;
	} else if (product > 0x400) {
		dev->jd_descr = "XILINX XC6VSL";
		expected_irlen =  10;
	} else if (product > 0x380) {
		dev->jd_descr = "XILINX XC3SD";
		expected_irlen =  6;
	} else if (product > 0x374) {
		dev->jd_descr = "XILINX XC7K";
		expected_irlen =  6;
	} else if (product > 0x372) {
		dev->jd_descr = "XILINX XC7Z";
		expected_irlen =  6;
	} else if (product > 0x366) {
		dev->jd_descr = "XILINX XC7V";
		expected_irlen =  6;
	} else if (product > 0x364) {
		dev->jd_descr = "XILINX XC7K";
		expected_irlen =  6;
	} else if (product > 0x362) {
		dev->jd_descr = "XILINX XC7A";
		expected_irlen =  6;
	} else if (product > 0x32d) {
		dev->jd_descr = "XILINX XC5VF";
		expected_irlen =  14;
	} else if (product > 0x320) {
		dev->jd_descr = "XILINX XC5VF";
		expected_irlen =  10;
	} else if (product > 0x2e0) {
		dev->jd_descr = "XILINX XC5SX";
		expected_irlen =  10;
	} else if (product > 0x280) {
		dev->jd_descr = "XILINX XC5SV";
		expected_irlen =  10;
	} else if (product > 0x220) {
		dev->jd_descr = "XILINX XC3SxxxAN";
		expected_irlen =   6;
	} else if (product > 0x200) {
		dev->jd_descr = "XILINX XC4VSX";
		expected_irlen =  10;
	} else if (product > 0x1e0) {
		dev->jd_descr = "XILINX XC4VFX";
		expected_irlen =  10;
	} else if (product > 0x1C0) {
		dev->jd_descr = "XILINX XC3SxxxE";
		expected_irlen =   6;
	} else if (product > 0x160) {
		dev->jd_descr = "XILINX XC4VLX";
		expected_irlen =  10;
	} else if (product > 0x140) {
		dev->jd_descr = "XILINX XC3S";
		expected_irlen =   6;
	} else if (product > 0x100) {
		expected_irlen =   6;
		dev->jd_descr = "XILINX XC2V";
	}
	if (!dev->jd_descr)
		dev->jd_descr = "XILINX";
	return expected_irlen;
}

