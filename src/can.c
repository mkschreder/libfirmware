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
 * CAN message dispatcher is an extension to device specific can implementation drivers that allows
 * multiple listeners to register and receive messages from the can interface.
 */

#include <libfdt/libfdt.h>
#include <errno.h>

#include "driver.h"
#include "can.h"
#include "list.h"
#include "thread.h"

DEFINE_DEVICE_CLASS(can)

void _can_dispatcher_loop(void *ptr){
	struct can_dispatcher *self = (struct can_dispatcher*)ptr;
	while(1){
		struct can_message cm;
		if(self->next_message(self->dev, &cm) > 0){
			struct can_listener *listener;
			thread_mutex_lock(&self->lock);
			list_for_each_entry(listener, &self->listeners, list){
				if(listener && listener->process_message){
					thread_mutex_unlock(&self->lock);
					listener->process_message(listener, self->dev, &cm);
					thread_mutex_lock(&self->lock);
				}
			}
			thread_mutex_unlock(&self->lock);
		}
	}
}

void can_dispatcher_init(struct can_dispatcher *self,
		can_device_t dev,
		int (*next_message)(can_device_t dev, struct can_message *msg)){
	memset(self, 0, sizeof(*self));
	BUG_ON(!dev);
	BUG_ON(!next_message);
	INIT_LIST_HEAD(&self->listeners);
	self->dev = dev;
	self->next_message = next_message;
	thread_mutex_init(&self->lock);
	thread_create(_can_dispatcher_loop, "can_disp", 400, self, 2, NULL);
}

void can_dispatcher_add_listener(struct can_dispatcher *self, struct can_listener *listener){
	thread_mutex_lock(&self->lock);
	list_add(&listener->list, &self->listeners);
	thread_mutex_unlock(&self->lock);
}

void can_listener_init(struct can_listener *self, void (*handler)(struct can_listener *self, can_device_t can, struct can_message *msg)){
	memset(self, 0, sizeof(*self));
	INIT_LIST_HEAD(&self->list);
	self->process_message = handler;
}

