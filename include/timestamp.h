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

#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include "types.h"

struct timeval;

typedef unsigned long timestamp_t;
typedef unsigned long usec_t;

// time_after(a,b) returns true if the time a is after time b.

#define time_after(a,b)		\
	(typecheck(unsigned long, a) && \
	 typecheck(unsigned long, b) && \
	 ((long)((b) - (a)) < 0))
#define time_before(a,b)	time_after(b,a)

#define time_after_eq(a,b)	\
	(typecheck(unsigned long, a) && \
	 typecheck(unsigned long, b) && \
	 ((long)((a) - (b)) >= 0))
#define time_before_eq(a,b)	time_after_eq(b,a)

/*
 * Calculate whether a is in the range of [b, c].
 */
#define time_in_range(a,b,c) \
	(time_after_eq(a,b) && \
	 time_before_eq(a,c))

int timeval_subtract(const struct timeval *x, const struct timeval *y, struct timeval *result);
//int time_diff(const struct timeval *x, const struct timeval *y, struct timeval *result);
void time_cpy(struct timeval *dst, const struct timeval *src);

void time_init(void);
int time_init_rtc(void);

timestamp_t micros(void);
void time_gettime(struct timeval *ts);
uint32_t time_get_hse_precision(void);
