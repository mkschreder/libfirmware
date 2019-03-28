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

#include "timestamp.h"
#include "list.h"

typedef const struct i2c_device_ops ** i2c_device_t;

struct i2c_device_ops {
    int (*write)(i2c_device_t dev, uint8_t addr_, uint8_t reg_, const void *data, size_t len);
    int (*read)(i2c_device_t dev, uint8_t addr_, uint8_t reg_, void *data, size_t len);
};

struct i2c_device {
	struct list_head list;
	const struct i2c_device_ops *ops;
	void *fdt;
	int fdt_node;
};

void i2c_device_init(struct i2c_device *self, void *fdt, int fdt_node, const struct i2c_device_ops *ops);
int i2c_device_register(struct i2c_device *self);
i2c_device_t i2c_find(void *fdt, const char *dtb_path);
i2c_device_t i2c_find_by_node(void *fdt, int node);
i2c_device_t i2c_find_by_ref(void *fdt, int fdt_node, const char *ref_name);

#define i2c_write_buf(dev, addr, reg, buf, len) (*(dev))->write(dev, addr, reg, buf, len)
#define i2c_read_buf(dev, addr, reg, buf, len) (*(dev))->read(dev, addr, reg, buf, len)

int i2c_write_reg(i2c_device_t dev, uint8_t addr, uint8_t reg, const uint8_t data);
int i2c_read_reg(i2c_device_t dev, uint8_t addr, uint8_t reg, uint8_t *data);
