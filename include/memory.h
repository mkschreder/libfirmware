#pragma once

#include "timestamp.h"
#include "list.h"
#include "driver.h"

typedef const struct memory_device_ops ** memory_device_t;

struct memory_device_ops {
    int (*write)(memory_device_t dev, size_t offset, const void *data, size_t len);
    int (*read)(memory_device_t dev, size_t offset, void *data, size_t len);
};

#define memory_read(dev, off, dat, len) (*(dev))->read(dev, off, dat, len)
#define memory_write(dev, off, dat, len) (*(dev))->write(dev, off, dat, len)

DECLARE_DEVICE_CLASS(memory)
