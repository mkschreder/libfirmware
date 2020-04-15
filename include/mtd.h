#pragma once

#include "driver.h"
#include "list.h"
#include "timestamp.h"

typedef const struct mtd_device_ops **mtd_device_t;

struct mtd_device_info {
	uint8_t manufacturer;
	uint8_t type;
	uint32_t size;
	uint32_t erasesize;
	uint32_t writesize;
};

struct mtd_device_ops {
	int (*info)(mtd_device_t dev, struct mtd_device_info *info);
	int (*erase)(mtd_device_t dev, size_t offset, size_t len);
	int (*write)(mtd_device_t dev, size_t offset, const void *data, size_t len);
	int (*read)(mtd_device_t dev, size_t offset, void *data, size_t len);
};

#define mtd_info(dev, info) (*(dev))->read(dev, info)
#define mtd_erase(dev, off, len) (*(dev))->read(dev, off, len)
#define mtd_read(dev, off, dat, len) (*(dev))->read(dev, off, dat, len)
#define mtd_write(dev, off, dat, len) (*(dev))->write(dev, off, dat, len)

DECLARE_DEVICE_CLASS(mtd)
