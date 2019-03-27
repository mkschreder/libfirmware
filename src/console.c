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

#include <errno.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <libfdt/libfdt.h>

#include "console.h"
#include "list.h"
#include "thread.h"
#include "mutex.h"
#include "driver.h"


static LIST_HEAD(_console_devs);

void console_init(struct console_device *self, void *fdt, int fdt_node, const struct console_ops *ops){
	memset(self, 0, sizeof(*self));
	INIT_LIST_HEAD(&self->list);
	self->fdt = fdt;
	self->fdt_node = fdt_node;
	self->ops = ops;
}

int console_register(struct console_device *self){
	BUG_ON(!self);
	BUG_ON(!self->ops);
	BUG_ON(!self->ops->add_command);
	list_add_tail(&self->list, &_console_devs);
	return 0;
}

console_t console_find(const char *dtb_path){
	struct console_device *console;
	int node = fdt_path_offset(_devicetree, dtb_path);
	if(node < 0) return NULL;
	list_for_each_entry(console, &_console_devs, list){
		if(console->fdt_node == node) return &console->ops;
	}
	return NULL;
}

console_t console_find_by_node(void *fdt, int node){
	struct console_device *dev;
    if(node < 0) return NULL;
    list_for_each_entry(dev, &_console_devs, list){
		if(dev->fdt_node == node) return &dev->ops;
	}
	return NULL;
}

console_t console_find_by_ref(void *fdt, int fdt_node, const char *ref_name){
	int node = fdt_find_node_by_ref(fdt, fdt_node, ref_name);
	if(node < 0){
		return 0;
	}

	console_t dev = console_find_by_node(fdt, node);
	if(!dev){
		return 0;
	}
	return dev;
}

int console_add_command(console_t con,
		void *userptr,
		const char *name,
		const char *description,
		const char *options,
		int (*proc)(console_t con, void *userptr, int argc, char **argv)
){
	struct console_command *self = kzmalloc(sizeof(struct console_command));
	self->name = kzmalloc(strlen(name) + 1);
	if(!self->name) return -ENOMEM;
	strcpy(self->name, name);
	self->description = kzmalloc(strlen(description) + 1);
	if(!self->description) return -ENOMEM;
	strcpy(self->description, description);
	self->options = kzmalloc(strlen(options) + 1);
	if(!self->options) return -ENOMEM;
	strcpy(self->options, options);
	self->proc = proc;
	self->userptr = userptr;
	return (*(con))->add_command(con, self);
}
