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

#include <stdint.h>
#include <stddef.h>
#include "work.h"
#include "mutex.h"
#include "dx_tracker.h"
#include "list.h"

typedef const struct encoder_ops ** encoder_t;

struct encoder_ops {
	int32_t (*read)(encoder_t encoder);
};

#define encoder_read(enc) (*(enc))->read(enc)

struct encoder_device {
	struct list_head list;
	const struct encoder_ops *ops;
	int fdt_node;

	struct mutex mx;
	uint32_t counts_per_unit;
	uint32_t unit_scale;
	int32_t last_raw;
	uint32_t last_update_micros;
	uint32_t update_interval_ms;
	float observer_gain;
	struct dx_tracker dx_track;

	struct work work;
};

void encoder_device_init(struct encoder_device *self, int fdt_node, const struct encoder_ops *ops);
int encoder_device_register(struct encoder_device *self);
encoder_t encoder_find(const char *dtb_path);

//! returns encoder position in meters relative to position when encoder was started
float encoder_get_position(encoder_t encoder);
float encoder_get_velocity(encoder_t encoder);
