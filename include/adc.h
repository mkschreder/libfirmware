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

typedef const struct adc_device_ops ** adc_device_t;

struct adc_device_ops {
    int (*trigger)(adc_device_t dev);
    int (*read)(adc_device_t dev, unsigned int channel, int16_t *value);
};

struct adc_device {
	struct list_head list;
	const struct adc_device_ops *ops;
	int fdt_node;
};

void adc_device_init(struct adc_device *self, int fdt_node, const struct adc_device_ops *ops);
int adc_device_register(struct adc_device *self);
adc_device_t adc_find(const char *dtb_path);
adc_device_t adc_find_by_node(void *fdt, int node);

#define adc_read(dev, channel) (*(dev))->read(dev, channel)
#define adc_trigger(dev) (*(dev))->trigger(dev)

