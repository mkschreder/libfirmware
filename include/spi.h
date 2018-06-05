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

#include "time.h"
#include "list.h"

typedef const struct spi_device_ops ** spi_device_t;

struct spi_device_ops {
	int (*transfer)(spi_device_t spi, const void *tx, void *rx, size_t size, timestamp_t timeout);
};

#define spi_transfer(spi, tx, rx, sz, to) (*(spi))->transfer(spi, tx, rx, sz, to)

struct spi_device {
	struct list_head list;
	const struct spi_device_ops *ops;
	int fdt_node;
};

void spi_device_init(struct spi_device *self, int fdt_node, const struct spi_device_ops *ops);
int spi_device_register(struct spi_device *self);
spi_device_t spi_find(const char *dtb_path);

