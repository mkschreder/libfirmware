/*
 * Copyright (C) 2017 Martin K. Schröder <mkschreder.uk@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#pragma once

#include "timestamp.h"
#include "list.h"
#include "gpio.h"

typedef const struct spi_device_ops ** spi_device_t;

struct spi_transfer {
	struct list_head list;
	const void *tx_buf;
	void *rx_buf;
	size_t len;
	uint32_t timeout_ms;
};

struct spi_device_ops {
	int (*transfer)(spi_device_t spi, gpio_device_t gpio, uint32_t cs_pin, const void *tx_buf, void *rx_buf, size_t len, uint32_t to);
};

#define spi_transfer(spi, gpio, cs_pin, tx, rx, sz, to) (*(spi))->transfer(spi, gpio, cs_pin, tx, rx, sz, to)

DECLARE_DEVICE_CLASS(spi)

void spi_transfer_init(struct spi_transfer *self);

