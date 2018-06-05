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

typedef const struct gpio_pin_ops ** gpio_pin_t;

struct gpio_pin_ops {
	int (*set)(gpio_pin_t pin, bool value);
	bool (*get)(gpio_pin_t pin);
	int (*register_irq)(gpio_pin_t pin, irq_trigger_mode_t mode, struct irq *irq, irq_result_t (*handler)(struct irq *self, int *wake));
};

#define gpio_pin_set(pin) (*(pin))->set(pin, true)
#define gpio_pin_get(pin) (*(pin))->get(pin)
#define gpio_pin_reset(pin) (*(pin))->set(pin, false)
#define gpio_register_irq(pin, mode, irq, handler) (*(pin))->register_irq(pin, mode, irq, handler)
