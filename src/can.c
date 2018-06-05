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

/**
 * CAN driver core subsystem design with possibility for can protocol drivers
 * to add listeners for incoming can packets. This way multiple protocol
 * drivers can be used with the same can interface.
 */

#include <libfdt/libfdt.h>
#include <errno.h>

#include "driver.h"
#include "can.h"
#include "list.h"
#include "thread.h"

static LIST_HEAD(_can_ports);

void can_listener_init(struct can_listener *self, void (*handler)(struct can_listener *self, struct can_message *msg)){
	memset(self, 0, sizeof(*self));
	INIT_LIST_HEAD(&self->list);
	self->handler = handler;
}

// default handler for device related messages which can be called from driver to avoid reimplementing same functionality
int can_device_handler(struct can_device *self, can_listen_cmd_t cmd, struct can_listener *listener){
	switch(cmd){
		case CAN_LISTENER_ADD: {
			thread_mutex_lock(&self->lock);
			list_add(&listener->list, &self->listeners);
			thread_mutex_unlock(&self->lock);
		} break;
		case CAN_LISTENER_REMOVE:{
			thread_mutex_lock(&self->lock);
			struct can_listener *l;
			list_for_each_entry(l, &self->listeners, list){
				if(l == listener){
					list_del_init(&l->list);
					break;
				}
			}
			thread_mutex_unlock(&self->lock);
		} break;
		default: return -EINVAL;
	}
	return 0;
}

/**
 * Usually called from driver dispatcher worker
 */
void can_device_dispatch_message(struct can_device *self, struct can_message *msg){
	struct can_listener *listener;
	thread_mutex_lock(&self->lock);
	list_for_each_entry(listener, &self->listeners, list){
		if(listener && listener->handler)
			listener->handler(listener, msg);
	}
	thread_mutex_unlock(&self->lock);
}

int can_register_device(void *fdt, int fdt_node, struct can_device *self, can_port_t port){
	self->port = port;
	self->fdt_node = (int)fdt_node;
	thread_mutex_init(&self->lock);
	INIT_LIST_HEAD(&self->list);
	INIT_LIST_HEAD(&self->listeners);

	list_add_tail(&self->list, &_can_ports);

	return 0;
}

int can_unregister_device(void *fdt, int fdt_node){
	struct can_device *can;
	list_for_each_entry(can, &_can_ports, list){
		if(can->fdt_node == fdt_node) {
			list_del(&can->list);
			break;
		}
	}
	return 0;
}

struct can_device *can_port_first(){
	if(list_empty(&_can_ports)) return NULL;
	return list_first_entry(&_can_ports, struct can_device, list);
}

can_port_t can_find(const char *dtb_path){
	struct can_device *can;
	int node = fdt_path_offset(_devicetree, dtb_path);
	if(node < 0) return NULL;
	list_for_each_entry(can, &_can_ports, list){
		if(can->fdt_node == node) return can->port;
	}
	return NULL;
}
