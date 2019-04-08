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

#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include "thread.h"
#include "regmap.h"
#include "driver.h"

#include <libfdt/libfdt.h>

DEFINE_DEVICE_CLASS(regmap)

void regmap_range_init(struct regmap_range *self, uint32_t start, uint32_t end, const struct regmap_range_ops *ops){
	memset(self, 0, sizeof(*self));
	INIT_LIST_HEAD(&self->list);
	self->start = start;
	self->end = end;
	self->ops = ops;
}

int regmap_write_u8(regmap_device_t dev, uint32_t id, uint8_t value){
	return regmap_write(dev, id, REG_UINT8, &value, sizeof(value));
}

int regmap_write_i8(regmap_device_t dev, uint32_t id, int8_t value){
	return regmap_write(dev, id, REG_INT8, &value, sizeof(value));
}

int regmap_write_u16(regmap_device_t dev, uint32_t id, uint16_t value){
	return regmap_write(dev, id, REG_UINT16, &value, sizeof(value));
}

int regmap_write_i16(regmap_device_t dev, uint32_t id, int16_t value){
	return regmap_write(dev, id, REG_INT16, &value, sizeof(value));
}

int regmap_write_u32(regmap_device_t dev, uint32_t id, uint32_t value){
	return regmap_write(dev, id, REG_UINT32, &value, sizeof(value));
}

int regmap_write_i32(regmap_device_t dev, uint32_t id, int32_t value){
	return regmap_write(dev, id, REG_INT32, &value, sizeof(value));
}

int regmap_write_string(regmap_device_t dev, uint32_t id, const char *str, size_t len){
	return regmap_write(dev, id, REG_STRING, str, len);
}

int regmap_read_u32(regmap_device_t dev, uint32_t id, uint32_t *value){
	return regmap_read(dev, id, REG_UINT32, value, sizeof(*value));
}

int regmap_read_i32(regmap_device_t dev, uint32_t id, int32_t *value){
	return regmap_read(dev, id, REG_INT32, value, sizeof(*value));
}

int regmap_read_u16(regmap_device_t dev, uint32_t id, uint16_t *value){
	return regmap_read(dev, id, REG_UINT16, value, sizeof(*value));
}

int regmap_read_i16(regmap_device_t dev, uint32_t id, int16_t *value){
	return regmap_read(dev, id, REG_INT16, value, sizeof(*value));
}

int regmap_read_u8(regmap_device_t dev, uint32_t id, uint8_t *value){
	return regmap_read(dev, id, REG_UINT8, value, sizeof(*value));
}

int regmap_read_i8(regmap_device_t dev, uint32_t id, int8_t *value){
	return regmap_read(dev, id, REG_INT8, value, sizeof(*value));
}

int regmap_read_string(regmap_device_t dev, uint32_t id, char *value, size_t max_len){
	return regmap_read(dev, id, REG_STRING, value, max_len);
}

int regmap_convert_u32(uint32_t value, regmap_value_type_t type, void *data, size_t size){
	switch(type){
		case REG_UINT8:
			if(size < 1) return -EINVAL;
			*((uint8_t*)data) = (uint8_t)value;
			return 1;
		case REG_INT8:
			if(size < 1) return -EINVAL;
			*((int8_t*)data) = (int8_t)value;
			return 1;
		case REG_UINT16:
			if(size < 2) return -EINVAL;
			*((uint16_t*)data) = (uint16_t)value;
			return 2;
		case REG_INT16:
			if(size < 2) return -EINVAL;
			*((int16_t*)data) = (int16_t)value;
			return 2;
		case REG_INT32:
			if(size < 4) return -EINVAL;
			*((int32_t*)data) = (int32_t)value;
			return 4;
		case REG_UINT32:
			if(size < 4) return -EINVAL;
			*((uint32_t*)data) = (uint32_t)value;
			return 4;
		case REG_STRING:
			return snprintf(data, size, "%u", (unsigned int)value);
	}
	return -EINVAL;
}

int regmap_mem_to_u32(regmap_value_type_t type, const void *data, size_t size, uint32_t *value){
	switch(type){
		case REG_UINT8:
			*value = *(uint8_t*)data;
			return 1;
		case REG_INT8:
			*value = (uint32_t)*(int8_t*)data;
			return 1;
		case REG_UINT16:
			*value = (uint32_t)*(uint16_t*)data;
			return 2;
		case REG_INT16:
			*value = (uint32_t)*(int16_t*)data;
			return 2;
		case REG_INT32:
			*value = (uint32_t)*(int32_t*)data;
			return 4;
		case REG_UINT32:
			*value = *(uint32_t*)data;
			return 4;
		case REG_STRING:
			return -1;
	}
	return -1;
}

int regmap_mem_to_u16(regmap_value_type_t type, const void *data, size_t size, uint16_t *value){
	switch(type){
		case REG_UINT8:
			*value = (uint16_t)*((uint8_t*)data);
			return 1;
		case REG_INT8:
			*value = (uint16_t)*((int8_t*)data);
			return 1;
		case REG_UINT16:
			*value = (uint16_t)*((uint16_t*)data);
			return 2;
		case REG_INT16:
			*value = (uint16_t)*((int16_t*)data);
			return 2;
		case REG_INT32:
			*value = (uint16_t)*((int32_t*)data);
			return 4;
		case REG_UINT32:
			*value = *((uint16_t*)data);
			return 4;
		case REG_STRING:
			return -1;
	}
	return -1;
}

int regmap_mem_to_u8(regmap_value_type_t type, const void *data, size_t size, uint8_t *value){
	switch(type){
		case REG_UINT8:
			*value = (uint8_t)*(uint8_t*)data;
			return 1;
		case REG_INT8:
			*value = (uint8_t)*(int8_t*)data;
			return 1;
		case REG_UINT16:
			*value = (uint8_t)*(uint16_t*)data;
			return 2;
		case REG_INT16:
			*value = (uint8_t)*(int16_t*)data;
			return 2;
		case REG_INT32:
			*value = (uint8_t)*(int32_t*)data;
			return 4;
		case REG_UINT32:
			*value = *(uint8_t*)data;
			return 4;
		case REG_STRING:
			return -1;
	}
	return -1;
}

struct regmap {
	struct regmap_device dev;
	struct list_head ranges;
	struct mutex lock;
};

static int _regmap_add(regmap_device_t dev, struct regmap_range *range){
	struct regmap *self = container_of(dev, struct regmap, dev.ops);
	BUG_ON(!list_empty(&range->list));
	thread_mutex_lock(&self->lock);
	list_add(&range->list, &self->ranges);
	thread_mutex_unlock(&self->lock);
	return 0;
}

static int _regmap_read(regmap_device_t dev, uint32_t id, regmap_value_type_t get_as, void *value, size_t value_size){
	struct regmap *self = container_of(dev, struct regmap, dev.ops);

	thread_mutex_lock(&self->lock);

	struct regmap_range *entry;
	list_for_each_entry(entry, &self->ranges, list){
		if(id >= entry->start && id <= entry->end){
			// try all ranges until one read succeeds. This allow overlapping regions
			int ret = 0;
			if((ret = entry->ops->read(&entry->ops, id, get_as, value, value_size)) >= 0){
				thread_mutex_unlock(&self->lock);
				return ret;
			}
		}
	}

	thread_mutex_unlock(&self->lock);
	return -ENOENT;
}

static int _regmap_write(regmap_device_t dev, uint32_t id, regmap_value_type_t set_from, const void* value, size_t size){
	struct regmap *self = container_of(dev, struct regmap, dev.ops);

	thread_mutex_lock(&self->lock);

	struct regmap_range *entry;
	list_for_each_entry(entry, &self->ranges, list){
		if(id >= entry->start && id <= entry->end){
			// try all ranges until one write succeeds. This allow overlapping regions
			int ret = 0;
			if((ret = entry->ops->write(&entry->ops, id, set_from, value, size)) >= 0){
				thread_mutex_unlock(&self->lock);
				return ret;
			}
		}
	}

	thread_mutex_unlock(&self->lock);
	return -ENOENT;
}

struct regmap_device_ops _regmap_ops = {
	.add = _regmap_add,
	.read = _regmap_read,
	.write = _regmap_write
};

int _regmap_probe(void *fdt, int fdt_node){
	struct regmap *self = kzmalloc(sizeof(struct regmap));
	INIT_LIST_HEAD(&self->ranges);
	thread_mutex_init(&self->lock);

	regmap_device_init(&self->dev, fdt, fdt_node, &_regmap_ops);
	regmap_device_register(&self->dev);

	printk(PRINT_SUCCESS "regmap: ready\n");
	return 0;
}

int _regmap_remove(void *fdt, int fdt_node){
	return -1;
}

DEVICE_DRIVER(regmap, "fw,regmap", _regmap_probe, _regmap_remove)
