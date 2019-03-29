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

#include "serial.h"
#include "list.h"
#include "thread.h"
#include "mutex.h"
#include "driver.h"

static serial_port_t _default_serial_port = 0;

DEFINE_DEVICE_CLASS(serial)

int serial_set_printk_port(serial_port_t port){
    _default_serial_port = port;
    return 0;
}

int printk(const char *fmt, ...){
    static bool _printk_mutex_inited = false;
    static struct mutex _printk_lock;
    static char buf[80];

	if(!_default_serial_port) return -1;
    if(!_printk_mutex_inited) {
        thread_mutex_init(&_printk_lock);
        _printk_mutex_inited = true;
    }

    thread_mutex_lock(&_printk_lock);

	va_list argptr;
	va_start(argptr, fmt);
	int len = vsnprintf(buf, sizeof(buf), fmt, argptr);
	va_end(argptr);

    thread_mutex_unlock(&_printk_lock);

	return serial_write(_default_serial_port, buf, (size_t)len, 10);
}

