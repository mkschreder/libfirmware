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

typedef const struct serial_ops ** serial_port_t;

struct serial_ops {
	int (*write)(serial_port_t port, const void *ptr, size_t size, uint32_t timeout_ms);
	int (*read)(serial_port_t port, void *ptr, size_t size, uint32_t timeout_ms);
};

#define serial_read(s, b, sz, t) (*(s))->read(s, b, sz, t)
#define serial_write(s, b, sz, t) (*(s))->write(s, b, sz, t)

struct serial_device {
	struct list_head list;
	const struct serial_ops *ops;
	int fdt_node;
};

void serial_device_init(struct serial_device *self, int fdt_node, const struct serial_ops *ops);
int serial_device_register(struct serial_device *self);
serial_port_t serial_find(const char *dtb_path);
serial_port_t serial_find_by_node(void *fdt, int node);

int serial_set_printk_port(serial_port_t port);
