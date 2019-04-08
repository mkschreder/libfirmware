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

DEFINE_DEVICE_CLASS(console)

int console_add_command(console_device_t con,
		void *userptr,
		const char *name,
		const char *description,
		const char *options,
		int (*proc)(console_device_t con, void *userptr, int argc, char **argv)
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
