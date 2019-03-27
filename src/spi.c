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

#include "spi.h"
#include "list.h"
#include "thread.h"
#include "driver.h"

#include <errno.h>

static LIST_HEAD(_spi_ports);

void spi_device_init(struct spi_device *self, int fdt_node, const struct spi_device_ops *ops){
	memset(self, 0, sizeof(*self));
	INIT_LIST_HEAD(&self->list);
	self->fdt_node = fdt_node;
	self->ops = ops;
}

int spi_device_register(struct spi_device *self){
	BUG_ON(!self);
	BUG_ON(!self->ops);
	BUG_ON(!self->ops->transfer);
	list_add_tail(&self->list, &_spi_ports);
	return 0;
}

spi_device_t spi_find_by_node(void *fdt, int node){
	struct spi_device *dev;
    if(node < 0) return NULL;
    list_for_each_entry(dev, &_spi_ports, list){
		if(dev->fdt_node == node) return &dev->ops;
	}
	return NULL;
}

spi_device_t spi_find(const char *dtb_path){
	int node = fdt_path_offset(_devicetree, dtb_path);
	if(node < 0) return NULL;
    return spi_find_by_node(_devicetree, node);
}

spi_device_t spi_find_by_ref(void *fdt, int fdt_node, const char *ref_name){
	int node = fdt_find_node_by_ref(fdt, fdt_node, ref_name);
	if(node < 0){
		return 0;
	}

	spi_device_t dev = spi_find_by_node(fdt, node);
	if(!dev){
		return 0;
	}
	return dev;
}

