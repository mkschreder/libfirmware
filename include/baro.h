#pragma once

#include "timestamp.h"
#include "list.h"

typedef const struct baro_device_ops ** baro_device_t;

enum {
    BARO_HAS_TEMPERATURE = 1,
    BARO_HAS_PRESSURE = (1 << 1)
};

struct baro_reading {
    uint8_t caps;
    float temperature;
    float pressure;
};

struct baro_device_ops {
    int (*read)(baro_device_t dev, struct baro_reading *data);
};

#define baro_read(baro, data) (*(baro))->read(baro, data)

struct baro_device {
	struct list_head list;
	const struct baro_device_ops *ops;
	int fdt_node;
};

void baro_device_init(struct baro_device *self, int fdt_node, const struct baro_device_ops *ops);
int baro_device_register(struct baro_device *self);
baro_device_t baro_find(const char *dtb_path);
