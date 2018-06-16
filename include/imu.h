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

typedef const struct imu_device_ops ** imu_device_t;

struct imu_reading {
    uint8_t has_reading;
    float ax, ay, az;
    float gx, gy, gz;
};

struct imu_device_ops {
    int (*read)(imu_device_t dev, struct imu_reading *data);
};

struct imu_device {
	struct list_head list;
	const struct imu_device_ops *ops;
	int fdt_node;
};

void imu_device_init(struct imu_device *self, int fdt_node, const struct imu_device_ops *ops);
int imu_device_register(struct imu_device *self);
imu_device_t imu_find(const char *dtb_path);
imu_device_t imu_find_by_node(void *fdt, int node);

#define imu_read(imu, data) (*(imu))->read(imu, data)

