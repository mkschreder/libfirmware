#pragma once

#include <stdbool.h>
#include "irq.h"
#include "timestamp.h"
#include "list.h"
#include "driver.h"

typedef const struct events_device_ops ** events_device_t;

struct events_subscriber {
	int (*notify)(struct events_subscriber *sub, uint32_t ev);
};

struct events_device_ops {
    int (*subscribe)(events_device_t dev, uint32_t ev, struct events_subscriber *sub);
    int (*publish)(events_device_t dev, uint32_t ev);
    int (*publish_from_isr)(events_device_t dev, uint32_t ev, int32_t *wake);
};

#define events_subscribe(dev, ev, sub) do { if(dev) (*(dev))->subscribe(dev, ev, sub); } while(0)
#define events_publish(dev, ev) do { if(dev) (*(dev))->publish(dev, ev); } while(0); 
#define events_publish_from_isr(dev, ev, wake) do { if(dev) (*(dev))->publish_from_isr(dev, ev, wake); } while(0); 

static inline void events_subscriber_init(struct events_subscriber *self,
	int (*notify)(struct events_subscriber *sub, uint32_t ev)){
	memset(self, 0, sizeof(*self));
	self->notify = notify;
}

DECLARE_DEVICE_CLASS(events)
