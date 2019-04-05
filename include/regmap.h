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

#include <sys/types.h>
#include "list.h"
#include "mutex.h"
#include "driver.h"

typedef enum {
    REG_UINT8 = (0 << 0),
    REG_INT8 = (1 << 0),
    REG_UINT16 = (2 << 0),
    REG_INT16 = (3 << 0),
    REG_INT32 = (4 << 0),
    REG_UINT32 = (5 << 0),
	REG_STRING = (7 << 0),
} regmap_value_type_t;

struct regmap_range {
	struct list_head list;
	uint32_t start, end;
	const struct regmap_range_ops *ops;
};

typedef const struct regmap_range_ops ** regmap_range_t;

struct regmap_range_ops {
	ssize_t (*read)(regmap_range_t range, uint32_t id, regmap_value_type_t type, void *value, size_t size);
	ssize_t (*write)(regmap_range_t range, uint32_t id, regmap_value_type_t type, const void *value, size_t size);
};

void regmap_range_init(struct regmap_range *self, uint32_t start, uint32_t end, const struct regmap_range_ops *ops);

typedef const struct regmap_device_ops ** regmap_device_t;

struct regmap_device_ops {
	int (*add)(regmap_device_t dev, struct regmap_range *range);
	int (*read)(regmap_device_t dev, uint32_t id, regmap_value_type_t get_as, void *value, size_t value_size);
	int (*write)(regmap_device_t dev, uint32_t id, regmap_value_type_t set_from, const void* value, size_t size);
};

#define regmap_add(dev, map) (*(dev))->add(dev, map)
#define regmap_read(dev, id, type, val, size) (*(dev))->read(dev, id, type, val, size)
#define regmap_write(dev, id, type, val, size) (*(dev))->write(dev, id, type, val, size)

DECLARE_DEVICE_CLASS(regmap)

int regmap_write_u8(regmap_device_t dev, uint32_t id, uint8_t value);
int regmap_write_i8(regmap_device_t dev, uint32_t id, int8_t value);
int regmap_write_u16(regmap_device_t dev, uint32_t id, uint16_t value);
int regmap_write_i16(regmap_device_t dev, uint32_t id, int16_t value);
int regmap_write_u32(regmap_device_t dev, uint32_t id, uint32_t value);
int regmap_write_i32(regmap_device_t dev, uint32_t id, int32_t value);
int regmap_write_string(regmap_device_t dev, uint32_t id, const char *str, size_t len);

int regmap_read_u32(regmap_device_t dev, uint32_t id, uint32_t *value);
int regmap_read_i32(regmap_device_t dev, uint32_t id, int32_t *value);
int regmap_read_u16(regmap_device_t dev, uint32_t id, uint16_t *value);
int regmap_read_i16(regmap_device_t dev, uint32_t id, int16_t *value);
int regmap_read_u8(regmap_device_t dev, uint32_t id, uint8_t *value);
int regmap_read_i8(regmap_device_t dev, uint32_t id, int8_t *value);
int regmap_read_string(regmap_device_t dev, uint32_t id, char *value, size_t max_len);

int regmap_convert_u32(uint32_t value, regmap_value_type_t type, void *data, size_t size);

int regmap_mem_to_u32(regmap_value_type_t type, const void *data, size_t size, uint32_t *value);
int regmap_mem_to_u16(regmap_value_type_t type, const void *data, size_t size, uint16_t *value);
int regmap_mem_to_u8(regmap_value_type_t type, const void *data, size_t size, uint8_t *value);
