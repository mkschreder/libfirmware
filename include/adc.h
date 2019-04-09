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
#include "driver.h"

typedef const struct adc_device_ops ** adc_device_t;

struct adc_device_ops {
    int (*trigger)(adc_device_t dev);
    int (*read)(adc_device_t dev, unsigned int channel, uint16_t *value);
};

#define adc_read(dev, channel, value) (*(dev))->read(dev, channel, value)
#define adc_trigger(dev) (*(dev))->trigger(dev)

DECLARE_DEVICE_CLASS(adc)

