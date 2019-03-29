#pragma once

#include <stdint.h>
#include <stddef.h>

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


#include "list.h"
#include "driver.h"

#define led_controller_t leds_device_t

typedef const struct leds_device_ops ** led_controller_t;

struct leds_device_ops {
	void (*on)(led_controller_t leds, uint8_t led);
	void (*off)(led_controller_t leds, uint8_t led);
	void (*toggle)(led_controller_t leds, uint8_t led);
};

#define led_on(c, l) (*(c))->on(c, l)
#define led_off(c, l) (*(c))->off(c, l)
#define led_toggle(c, l) (*(c))->toggle(c, l)

DECLARE_DEVICE_CLASS(leds)

