#pragma once

#include "timestamp.h"
#include "list.h"
#include "driver.h"

typedef const struct pointer_device_ops ** pointer_device_t;

struct pointer_reading {
    uint8_t caps;
	uint8_t quality;
    int16_t delta_x;
    int16_t delta_y;
};

struct pointer_device_ops {
    int (*read)(pointer_device_t dev, struct pointer_reading *data);
};

#define pointer_read(pointer, data) (*(pointer))->read(pointer, data)

DECLARE_DEVICE_CLASS(pointer)

