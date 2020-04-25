/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2019  Uwe Bonnes
 * Written by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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

/* This file implements the interface to command line command for PC-Hosted
 * platforms.
 */
#if !defined(__CL_UTILS_H)
#define __CL_UTILS_H

#define RESP_TIMEOUT (100)

enum bmp_cl_mode {
	BMP_MODE_DEBUG,
	BMP_MODE_TEST,
	BMP_MODE_RESET,
	BMP_MODE_FLASH_ERASE,
	BMP_MODE_FLASH_WRITE,
	BMP_MODE_FLASH_READ,
	BMP_MODE_FLASH_VERIFY
};

enum BMP_DEBUG {
	BMP_DEBUG_NONE = 0,
	BMP_DEBUG_INFO = 1,
	BMP_DEBUG_PLATFORM = 2,
	BMP_DEBUG_WIRE = 4
};

typedef struct BMP_CL_OPTIONS_s {
	enum bmp_cl_mode opt_mode;
	bool opt_usejtag;
	bool opt_tpwr;
	bool opt_connect_under_reset;
	char *opt_flash_file;
	char *opt_device;
	char *opt_serial;
	char *opt_ident_string;
	int  opt_position;
	char *opt_cable;
	int opt_debuglevel;
	int opt_target_dev;
	uint32_t opt_flash_start;
	size_t opt_flash_size;
	char     *opt_idstring;
}BMP_CL_OPTIONS_t;

extern int cl_debuglevel;
void cl_init(BMP_CL_OPTIONS_t *opt, int argc, char **argv);
int cl_execute(BMP_CL_OPTIONS_t *opt);
int serial_open(BMP_CL_OPTIONS_t *opt, char *serial);
void serial_close(void);
#endif
