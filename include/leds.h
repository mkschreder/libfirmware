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

typedef const struct led_controller_ops ** led_controller_t;
typedef const struct led_controller_ops ** leds_t;

struct led_controller_ops {
	void (*on)(led_controller_t leds, uint8_t led);
	void (*off)(led_controller_t leds, uint8_t led);
	void (*toggle)(led_controller_t leds, uint8_t led);
};

#define led_on(c, l) (*(c))->on(c, l)
#define led_off(c, l) (*(c))->off(c, l)
#define led_toggle(c, l) (*(c))->toggle(c, l)

struct leds_device {
	int fdt_node;
	led_controller_t ctrl;
	struct list_head list;
};

int leds_register(void *fdt, int fdt_node, led_controller_t ctrl);
led_controller_t leds_find(const char *dtb_path);

