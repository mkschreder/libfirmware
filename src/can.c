/** :ms-top-comment
* +-------------------------------------------------------------------------------+
* |                      ____                  _ _     _                          |
* |                     / ___|_      _____  __| (_)___| |__                       |
* |                     \___ \ \ /\ / / _ \/ _` | / __| '_ \                      |
* |                      ___) \ V  V /  __/ (_| | \__ \ | | |                     |
* |                     |____/ \_/\_/ \___|\__,_|_|___/_| |_|                     |
* |                                                                               |
* |               _____           _              _     _          _               |
* |              | ____|_ __ ___ | |__   ___  __| | __| | ___  __| |              |
* |              |  _| | '_ ` _ \| '_ \ / _ \/ _` |/ _` |/ _ \/ _` |              |
* |              | |___| | | | | | |_) |  __/ (_| | (_| |  __/ (_| |              |
* |              |_____|_| |_| |_|_.__/ \___|\__,_|\__,_|\___|\__,_|              |
* |                                                                               |
* |                       We design hardware and program it                       |
* |                                                                               |
* |               If you have software that needs to be developed or              |
* |                      hardware that needs to be designed,                      |
* |                               then get in touch!                              |
* |                                                                               |
* |                            info@swedishembedded.com                           |
* +-------------------------------------------------------------------------------+
*
*                       This file is part of TheBoss Project
*
* FILE ............... src/can.c
* AUTHOR ............. Martin K. Schröder
* VERSION ............ Not tagged
* DATE ............... 2019-05-26
* GIT ................ https://github.com/mkschreder/libfirmware
* WEBSITE ............ http://swedishembedded.com
* LICENSE ............ Swedish Embedded Open Source License
*
*                     Copyright (C) 2014-2019 Martin Schröder
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of
* this software and associated documentation files (the "Software"), to deal in
* the Software without restriction, including without limitation the rights to
* use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
* the Software, and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions:
*
* The above copyright notice and this text, in full, shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
* FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
* COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
* IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**/
/**
 * CAN message dispatcher is an extension to device specific can implementation drivers that allows
 * multiple listeners to register and receive messages from the can interface.
 */

#include <libfdt/libfdt.h>
#include <errno.h>

#include "driver.h"
#include "can.h"

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

