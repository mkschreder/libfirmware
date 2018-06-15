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


#include "serial.h"

#define CONSOLE_MAX_PS_TASKS 10
// TODO: make this configurable from the devicetree
#define CONSOLE_MAX_LINE 80
#define CONSOLE_MAX_ARGS 16

struct console {
	serial_port_t serial;
	struct console_command *commands;
	size_t ncommands;

	struct vardir *vars;

	//! this is used for displaying task stats
#if defined(FREERTOS) && CONFIG_CMD_PS == 1
	TaskStatus_t prev_status[CONSOLE_MAX_PS_TASKS];
#endif
	char printf_buffer[CONSOLE_MAX_LINE];
	char line[CONSOLE_MAX_LINE];
	char *argv[CONSOLE_MAX_ARGS];
};

struct console_command {
	const char *name;
	int (*proc)(struct console *self, int argc, char **argv);
	const char *options;
	const char *description;
};

void console_init(struct console *self, serial_port_t port, struct console_command *commands, size_t ncommands, struct vardir *vars);
void console_start(struct console *self);

void con_printf(struct console *self, const char *fmt, ...);
