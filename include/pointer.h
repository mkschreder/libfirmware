#pragma once

#include "timestamp.h"
#include "list.h"

typedef const struct pointer_device_ops ** pointer_device_t;

struct pointer_reading {
    uint8_t caps;
    int16_t delta_x;
    int16_t delta_y;
};

struct pointer_device_ops {
    int (*read)(pointer_device_t dev, struct pointer_reading *data);
};

#define pointer_read(pointer, data) (*(pointer))->read(pointer, data)

struct pointer_device {
	struct list_head list;
	const struct pointer_device_ops *ops;
	int fdt_node;
};

void pointer_device_init(struct pointer_device *self, int fdt_node, const struct pointer_device_ops *ops);
int pointer_device_register(struct pointer_device *self);
pointer_device_t pointer_find(const char *dtb_path);
