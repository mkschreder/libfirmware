/*
 * Copyright (C) 2019 Martin K. Schröder <mkschreder.uk@gmail.com>
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
#include "driver.h"

typedef const struct analog_device_ops ** analog_device_t;

struct analog_device_ops {
    int (*write)(analog_device_t dev, unsigned int channel, float value);
    int (*read)(analog_device_t dev, unsigned int channel, float *value);
};

#define analog_read(dev, channel, value) ((dev)?(*(dev))->read(dev, channel, value):-EINVAL)
#define analog_write(dev, channel, value) ((dev)?(*(dev))->write(dev, channel, value):-EINVAL)

DECLARE_DEVICE_CLASS(analog)

