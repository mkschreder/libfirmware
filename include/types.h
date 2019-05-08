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

#include <stddef.h>
#include <stdint.h>

#define THREAD_SLEEP_MAX_DELAY 0xffffffff

typedef uint32_t timeout_t;

#define typecheck(type,x) \
(__extension__({  type __dummy; \
    typeof(x) __dummy2; \
    (void)(&__dummy == &__dummy2); \
    1; \
}))

#define container_of(ptr, type, member) (__extension__({ \
	const typeof( ((type *)0)->member ) \
		*__mptr = (ptr);\
		(type *)( (char *)__mptr - offsetof(type,member) );}))

#define call_member(x, n, ...) ((x)->n((x), ##__VA_ARGS__))

#define _STRING(expr) #expr
#define STRING(expr) _STRING(expr)

#define BUG_ON(expr) do { if(expr) { panic("BUG_ON " STRING(expr) " in " __FILE__ " line " STRING(__LINE__)); } } while(0)
#define COVERAGE_DUMMY() do { uint8_t __attribute__((unused)) uncovered = 0; } while(0)

void __attribute__((weak)) panic(const char *msg);

#ifndef __packed
#define __packed __attribute__((packed))
#endif
//#define __pure __attribute__((pure))
