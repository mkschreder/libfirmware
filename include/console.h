#pragma once

// TODO: get rid of direct freertos dependency
#include "thread.h"
#include "queue.h"
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

#include "serial.h"
#include "list.h"
#include "driver.h"

typedef const struct console_device_ops ** console_device_t;

struct console;
struct console_command {
	char *name;
	int (*proc)(console_device_t dev, void *userptr, int argc, char **argv);
	char *options;
	char *description;
	void *userptr;
	struct list_head list;
};

struct console_device_ops {
	int (*add_command)(console_device_t dev, struct console_command *cmd);
	int (*printf)(console_device_t dev, const char *fmt, ...);
	int (*read)(console_device_t dev, char *data, size_t max_size, uint32_t timeout_ms);
};

int console_add_command(console_device_t con,
		void *userptr,
		const char *name,
		const char *description,
		const char *options,
		int (*proc)(console_device_t con, void *userptr, int argc, char **argv)
);

#define console_printf(dev, ...) (*(dev))->printf(dev, ##__VA_ARGS__)
#define console_read(dev, data, size, tout) (*(dev))->read(dev, data, size, tout)
//#define console_scanf(dev, ...) (*(dev))->scanf(dev, ##__VA_ARGS__)

DECLARE_DEVICE_CLASS(console)

