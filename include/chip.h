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

int chip_get_uuid(uint32_t id[3]);

static inline uint32_t chip_get_uuid32(void){
	uint32_t id[3];
	chip_get_uuid(id);
	return id[0] ^ id[1] ^ id[2];
}

uint32_t chip_get_device_id(void);
uint16_t chip_get_flash_size_k(void);
uint32_t chip_get_ram_total(void);
uint32_t chip_get_data_size(void);

void chip_reset_to_bootloader(void);
void chip_reset(void);
