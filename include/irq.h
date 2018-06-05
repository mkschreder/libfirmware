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

#include "list.h"

typedef enum {
	IRQ_TRIGGER_RISING = 0,
	IRQ_TRIGGER_FALLING,
	IRQ_TRIGGER_RISING_FALLING
} irq_trigger_mode_t;

typedef enum {
	IRQ_HANDLED,
	IRQ_NOT_HANDLED
} irq_result_t;

struct irq {
	struct list_head list;
	irq_result_t (*handler)(struct irq *self, int *wake);
};
