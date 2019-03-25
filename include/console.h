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

typedef const struct console_ops ** console_t;

struct console;
struct console_command {
	const char *name;
	int (*proc)(console_t dev, int argc, char **argv);
	const char *options;
	const char *description;
	struct list_head list;
};

void console_command_init(struct console_command *self,
		const char *name,
		const char *description,
		const char *options,
		int (*proc)(console_t con, int argc, char **argv)
);

struct console_ops {
	int (*add_command)(console_t dev, struct console_command *cmd);
};

#define console_add_command(dev, cmd) (*(dev))->add_command(dev, cmd)

struct console_device {
	struct list_head list;
	const struct console_ops *ops;
	void *fdt;
	int fdt_node;
};

void console_init(struct console_device *self, void *fdt, int fdt_node, const struct console_ops *ops);
int console_register(struct console_device *self);
console_t console_find(const char *dtb_path);
console_t console_find_by_node(void *fdt, int node);

