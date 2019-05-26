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
* FILE ............... src/events.c
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
#include "events.h"
#include "driver.h"
#include "mutex.h"
#include "queue.h"

DEFINE_DEVICE_CLASS(events)

struct events {
	struct list_head subscribers;
	struct events_device dev;
	struct mutex lock;
	struct thread_queue events;
	uint32_t timeout;
};

struct events_binding {
	struct list_head list;
	uint32_t ev;
	struct events_subscriber *sub;
};

static int _events_subscribe(events_device_t dev, uint32_t ev, struct events_subscriber *sub){
	struct events *self = container_of(dev, struct events, dev.ops);
	struct events_binding *evb = kzmalloc(sizeof(struct events_binding));
	INIT_LIST_HEAD(&evb->list);
	evb->ev = ev;
	evb->sub = sub;

	thread_mutex_lock(&self->lock);
	list_add(&evb->list, &self->subscribers);
	thread_mutex_unlock(&self->lock);

	return 0;
}

static int _events_publish(events_device_t dev, uint32_t ev){
	struct events *self = container_of(dev, struct events, dev.ops);
	return thread_queue_send(&self->events, &ev, self->timeout);
}

static int _events_publish_from_isr(events_device_t dev, uint32_t ev, int32_t *wake){
	struct events *self = container_of(dev, struct events, dev.ops);
	return thread_queue_send_from_isr(&self->events, &ev, wake);
}

static void _events_dispatcher(void *ptr){
	struct events *self = (struct events*)ptr;
	while(1){
		uint32_t ev = 0;
		if(thread_queue_recv(&self->events, &ev, self->timeout) == 1){
			//thread_mutex_lock(&self->lock);
			struct events_binding *evb;
			list_for_each_entry(evb, &self->subscribers, list){
				if(evb->ev == ev && evb->sub && evb->sub->notify){
					evb->sub->notify(evb->sub, ev);
				}
			}
			//thread_mutex_unlock(&self->lock);
		}
	}
}

static const struct events_device_ops _events_ops = {
	.subscribe = _events_subscribe,
	.publish = _events_publish,
	.publish_from_isr = _events_publish_from_isr
};

static int _events_probe(void *fdt, int fdt_node){
	uint8_t prio = (uint8_t)fdt_get_int_or_default(fdt, fdt_node, "priority", 1);
	uint32_t stack_size = (uint32_t)fdt_get_int_or_default(fdt, fdt_node, "stack_size", 450);
	uint32_t queue_size = (uint32_t)fdt_get_int_or_default(fdt, fdt_node, "queue_size", 8);
	uint32_t timeout = (uint32_t)fdt_get_int_or_default(fdt, fdt_node, "timeout", 100);

	struct events *self = kzmalloc(sizeof(struct events));
	thread_queue_init(&self->events, queue_size, sizeof(uint32_t));
	thread_mutex_init(&self->lock);
	INIT_LIST_HEAD(&self->subscribers);
	self->timeout = timeout;

	events_device_init(&self->dev, fdt, fdt_node, &_events_ops);
	events_device_register(&self->dev);

	thread_create(
			_events_dispatcher,
			"ev_disp",
			stack_size,
			self,
			prio,
			NULL);

	return 0;
}

static int _events_remove(void *fdt, int fdt_node){
	return -1;
}

DEVICE_DRIVER(events, "fw,events", _events_probe, _events_remove)
