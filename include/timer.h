#pragma once
#include "driver.h"

typedef const struct timer_device_ops ** timer_device_t;

struct timer_device_ops {
	int (*set_channel)(timer_device_t spi);
};

//#define spi_transfer(spi, gpio, cs_pin, tx, rx, sz, to) (*(spi))->transfer(spi, gpio, cs_pin, tx, rx, sz, to)

DECLARE_DEVICE_CLASS(timer)
