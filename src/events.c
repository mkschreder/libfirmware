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
