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

#include "adc.h"
#include "list.h"
#include "thread.h"
#include "driver.h"

#include <errno.h>

static LIST_HEAD(_adc_devices);

void adc_device_init(struct adc_device *self, int fdt_node, const struct adc_device_ops *ops){
	memset(self, 0, sizeof(*self));
	INIT_LIST_HEAD(&self->list);
	self->fdt_node = fdt_node;
	self->ops = ops;
}

int adc_device_register(struct adc_device *self){
	BUG_ON(!self);
	BUG_ON(!self->ops);
	BUG_ON(!self->ops->read);
	list_add_tail(&self->list, &_adc_devices);
	return 0;
}

adc_device_t adc_find_by_node(void *fdt, int node){
	struct adc_device *dev;
    if(node < 0) return NULL;
    list_for_each_entry(dev, &_adc_devices, list){
		if(dev->fdt_node == node) return &dev->ops;
	}
	return NULL;
}

adc_device_t adc_find(const char *dtb_path){
	int node = fdt_path_offset(_devicetree, dtb_path);
	if(node < 0) return NULL;
    return adc_find_by_node(_devicetree, node);
}

