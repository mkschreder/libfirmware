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

typedef const struct gpio_device_ops ** gpio_device_t;

struct gpio_device_ops {
    int (*write_pin)(gpio_device_t dev, uint32_t pin, bool value);
    int (*read_pin)(gpio_device_t dev, uint32_t pin, bool *value);
};

#define gpio_set(gpio, pin) (*(gpio))->write_pin(gpio, pin, true)
#define gpio_reset(gpio, pin) (*(gpio))->write_pin(gpio, pin, false)
static inline bool gpio_read(gpio_device_t gpio, uint32_t pin) {
	bool __val; 
	(*(gpio))->read_pin(gpio, pin, &__val);
	return __val;
}

struct gpio_device {
	struct list_head list;
	const struct gpio_device_ops *ops;
	void *fdt;
	int fdt_node;
};

void gpio_device_init(struct gpio_device *self, void *fdt, int fdt_node, const struct gpio_device_ops *ops);
int gpio_device_register(struct gpio_device *self);
gpio_device_t gpio_find(void *fdt, const char *dtb_path);
gpio_device_t gpio_find_by_node(void *fdt, int node);
gpio_device_t gpio_find_by_ref(void *fdt, int fdt_node, const char *ref_name);

// old pins ops
/*
struct gpio_pin_ops {
	int (*set)(gpio_pin_t pin, bool value);
	bool (*get)(gpio_pin_t pin);
	int (*register_irq)(gpio_pin_t pin, irq_trigger_mode_t mode, struct irq *irq, irq_result_t (*handler)(struct irq *self, int *wake));
};

#define gpio_pin_set(pin) (*(pin))->set(pin, true)
#define gpio_pin_get(pin) (*(pin))->get(pin)
#define gpio_pin_reset(pin) (*(pin))->set(pin, false)
#define gpio_register_irq(pin, mode, irq, handler) (*(pin))->register_irq(pin, mode, irq, handler)
*/
