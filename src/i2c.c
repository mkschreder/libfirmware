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

#include <libfdt/libfdt.h>

#include "i2c.h"
#include "list.h"
#include "thread.h"
#include "driver.h"

#include <errno.h>

static LIST_HEAD(_i2c_ports);

void i2c_device_init(struct i2c_device *self, int fdt_node, const struct i2c_device_ops *ops){
	memset(self, 0, sizeof(*self));
	INIT_LIST_HEAD(&self->list);
	self->fdt_node = fdt_node;
	self->ops = ops;
}

int i2c_device_register(struct i2c_device *self){
	BUG_ON(!self);
	BUG_ON(!self->ops);
	BUG_ON(!self->ops->write);
	BUG_ON(!self->ops->read);
	list_add_tail(&self->list, &_i2c_ports);
	return 0;
}

i2c_device_t i2c_find_by_node(void *fdt, int node){
	struct i2c_device *dev;
    if(node < 0) return NULL;
    list_for_each_entry(dev, &_i2c_ports, list){
		if(dev->fdt_node == node) return &dev->ops;
	}
	return NULL;
}

i2c_device_t i2c_find(const char *dtb_path){
	int node = fdt_path_offset(_devicetree, dtb_path);
	if(node < 0) return NULL;
    return i2c_find_by_node(_devicetree, node);
}

int i2c_write_reg(i2c_device_t dev, uint8_t addr, uint8_t reg, const uint8_t data){
    return i2c_write_buf(dev, addr, reg, &data, 1);
}

int i2c_read_reg(i2c_device_t dev, uint8_t addr, uint8_t reg, uint8_t *data){
    uint8_t ch;
    if(!data) data = &ch;
    return i2c_read_buf(dev, addr, reg, data, 1);
}

