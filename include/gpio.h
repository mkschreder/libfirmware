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

#include <stdbool.h>
#include "irq.h"
#include "timestamp.h"
#include "list.h"
#include "driver.h"

typedef const struct gpio_device_ops ** gpio_device_t;

struct gpio_device_ops {
    int (*write_pin)(gpio_device_t dev, uint32_t pin, bool value);
    int (*read_pin)(gpio_device_t dev, uint32_t pin, bool *value);
};

#define gpio_set(gpio, pin) (*(gpio))->write_pin(gpio, pin, true)
#define gpio_reset(gpio, pin) (*(gpio))->write_pin(gpio, pin, false)
#define gpio_write(gpio, pin, val) (*(gpio))->write_pin(gpio, pin, val)

uint32_t gpio_read(gpio_device_t gpio, uint32_t pin);

DECLARE_DEVICE_CLASS(gpio)
