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
#include <errno.h>

#include "serial.h"
#include "list.h"
#include "thread.h"
#include "driver.h"

static LIST_HEAD(_serial_ports);

void serial_device_init(struct serial_device *self, int fdt_node, const struct serial_ops *ops){
	memset(self, 0, sizeof(*self));
	INIT_LIST_HEAD(&self->list);
	self->fdt_node = fdt_node;
	self->ops = ops;
}

int serial_device_register(struct serial_device *self){
	BUG_ON(!self);
	BUG_ON(!self->ops);
	BUG_ON(!self->ops->write);
	BUG_ON(!self->ops->read);
	list_add_tail(&self->list, &_serial_ports);
	return 0;
}

serial_port_t serial_find(const char *dtb_path){
	struct serial_device *serial;
	int node = fdt_path_offset(_devicetree, dtb_path);
	if(node < 0) return NULL;
	list_for_each_entry(serial, &_serial_ports, list){
		if(serial->fdt_node == node) return &serial->ops;
	}
	return NULL;
}
