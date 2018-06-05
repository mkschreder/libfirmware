#pragma once

#include "thread.h"
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

struct device_driver {
	const char *name;
	const char *compatible;
	int (*probe)(void *fdt, int device_node);
	int (*remove)(void *fdt, int device_node);
} __attribute__((aligned(4)));

#define DEVICE_DRIVER(_name, _compatible, _probe, _remove) static const struct device_driver __attribute__((section(".drivers"),used,aligned(4))) _driver = { .name = _name, .compatible = _compatible, .probe = _probe, .remove = _remove }

int probe_device_drivers(void *fdt);
int remove_device_drivers(void *fdt);

extern const char *_devicetree;
