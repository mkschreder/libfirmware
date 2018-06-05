/*
 * Copyright (C) 2017 Martin K. Schröder <mkschreder.uk@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#pragma once

#include <stdint.h>
#include <stddef.h>

typedef const struct eeprom_ops ** eeprom_t;

/**
 * EEPROM information struct
 */
struct eeprom_info {
	uint32_t page_size;
	uint32_t num_pages;
};

struct eeprom_ops {
	int (*write)(eeprom_t mem, uint32_t addr, const void *ptr, size_t size);
	int (*read)(eeprom_t mem, uint32_t addr, void *ptr, size_t size);
	int (*erase_page)(eeprom_t mem, uint32_t addr);
	int (*get_info)(eeprom_t mem, struct eeprom_info *info);
};

#define eeprom_read(s, addr, d, sz) (*(s))->read(s, addr, d, sz)
#define eeprom_write(s, addr, d, sz) (*(s))->write(s, addr, d, sz)
#define eeprom_erase_page(s, addr) (*(s))->erase_page(s, addr)
#define eeprom_get_info(s, info) (*(s))->get_info(s, info)

