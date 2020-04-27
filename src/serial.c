/** :ms-top-comment
* +-------------------------------------------------------------------------------+
* |                      ____                  _ _     _                          |
* |                     / ___|_      _____  __| (_)___| |__                       |
* |                     \___ \ \ /\ / / _ \/ _` | / __| '_ \                      |
* |                      ___) \ V  V /  __/ (_| | \__ \ | | |                     |
* |                     |____/ \_/\_/ \___|\__,_|_|___/_| |_|                     |
* |                                                                               |
* |               _____           _              _     _          _               |
* |              | ____|_ __ ___ | |__   ___  __| | __| | ___  __| |              |
* |              |  _| | '_ ` _ \| '_ \ / _ \/ _` |/ _` |/ _ \/ _` |              |
* |              | |___| | | | | | |_) |  __/ (_| | (_| |  __/ (_| |              |
* |              |_____|_| |_| |_|_.__/ \___|\__,_|\__,_|\___|\__,_|              |
* |                                                                               |
* |                       We design hardware and program it                       |
* |                                                                               |
* |               If you have software that needs to be developed or              |
* |                      hardware that needs to be designed,                      |
* |                               then get in touch!                              |
* |                                                                               |
* |                            info@swedishembedded.com                           |
* +-------------------------------------------------------------------------------+
*
*                       This file is part of TheBoss Project
*
* FILE ............... src/serial.c
* AUTHOR ............. Martin K. Schröder
* VERSION ............ Not tagged
* DATE ............... 2019-05-26
* GIT ................ https://github.com/mkschreder/libfirmware
* WEBSITE ............ http://swedishembedded.com
* LICENSE ............ Swedish Embedded Open Source License
*
*          Copyright (C) 2014-2019 Martin Schröder. All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of
* this software and associated documentation files (the "Software"), to deal in
* the Software without restriction, including without limitation the rights to
* use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
* the Software, and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions:
*
* The above copyright notice and this text, in full, shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
* FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
* COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
* IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**/
#include <errno.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <libfdt/libfdt.h>

#include "serial.h"
#include "types/list.h"
#include "thread/thread.h"
#include "thread/mutex.h"
#include "driver.h"

#define PRINTK_WRITE_TIMEOUT 100

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

	int ret = 0;
	if(serial_write(_default_serial_port, buf, (size_t)len, PRINTK_WRITE_TIMEOUT) < 0) ret = -1;
	if(serial_write(_default_serial_port, "\x1b[0m", 4, PRINTK_WRITE_TIMEOUT) < 0) ret = -1;

    thread_mutex_unlock(&_printk_lock);

	return ret;
}

int serial_write_string(serial_port_t dev, const char *str, timeout_t to){
	return serial_write(dev, str, strlen(str), to);
}

int serial_write_u32(serial_port_t dev, uint32_t value, timeout_t to){
	char buf[12];
	int len = snprintf(buf, sizeof(buf), "%u", (unsigned)value);
	if(len < 0) return -EINVAL;
	return serial_write(dev, buf, (size_t)len, to);
}

int serial_write_i32(serial_port_t dev, int32_t value, timeout_t to){
	char buf[12];
	int len = snprintf(buf, sizeof(buf), "%d", (signed)value);
	if(len < 0) return -EINVAL;
	return serial_write(dev, buf, (size_t)len, to);
}
