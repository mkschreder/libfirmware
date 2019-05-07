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
#include "list.h"
#include "driver.h"

#define serial_port_t serial_device_t
typedef const struct serial_device_ops ** serial_port_t;

struct serial_device_ops {
	int (*write)(serial_port_t port, const void *ptr, size_t size, uint32_t timeout_ms);
	int (*read)(serial_port_t port, void *ptr, size_t size, uint32_t timeout_ms);
};

#define serial_read(s, b, sz, t) (*(s))->read(s, b, sz, t)
#define serial_write(s, b, sz, t) (*(s))->write(s, b, sz, t)

int serial_set_printk_port(serial_port_t port);

DECLARE_DEVICE_CLASS(serial)


int serial_write_string(serial_port_t dev, const char *str, timeout_t to);
int serial_write_u32(serial_port_t dev, uint32_t value, timeout_t to);
int serial_write_i32(serial_port_t dev, int32_t value, timeout_t to);
