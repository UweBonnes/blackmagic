/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2019 Uwe Bonnes
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.	 If not, see <http://www.gnu.org/licenses/>.
 */
#if !defined(__STLINKV2_H_)
#define STLINK_ERROR_FAIL -1
#define STLINK_ERROR_OK 0
#define STLINK_ERROR_WAIT 1

#define STLINK_DEBUG_PORT_ACCESS            0xffff

void stlink_init(int argc, char **argv);
void stlink_leave_state(void);
const char *stlink_target_voltage(void);
void stlink_srst_set_val(bool assert);
void stlink_enter_debug_swd(void);
uint32_t stlink_read_coreid(void);
int stlink_read_dp_register(uint16_t port, uint16_t addr, uint32_t *res);
int stlink_write_dp_register(uint16_t port, uint16_t addr, uint32_t val);
void stlink_open_ap(uint8_t ap);
void stlink_close_ap(uint8_t ap);
void stlink_readmem32(void *dest, uint32_t src, size_t len);
int stlink_usb_get_rw_status(void);
void stlink_writemem8 (uint32_t addr, size_t len, uint8_t  *buffer);
void stlink_writemem16(uint32_t addr, size_t len, uint16_t *buffer);
void stlink_writemem32(uint32_t addr, size_t len, uint32_t *buffer);
#endif
