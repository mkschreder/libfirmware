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

typedef uint32_t sec_t;
typedef int32_t sec_diff_t;
typedef uint32_t msec_t;
typedef int32_t msec_diff_t;
typedef uint32_t usec_t;
typedef int32_t usec_diff_t;

typedef struct {
	sec_t sec;
	usec_t usec;
} timestamp_t;

typedef struct {
	sec_diff_t sec;
	usec_diff_t usec;
} timestamp_diff_t;

//typedef unsigned long timestamp_t;
//typedef unsigned long usec_t;

int timestamp_before(const timestamp_t a, const timestamp_t b);
int timestamp_after(const timestamp_t a, const timestamp_t b);
int timestamp_expired(const timestamp_t a);
timestamp_t timestamp_add(const timestamp_t a, const timestamp_t b);
timestamp_diff_t timestamp_sub(const timestamp_t a, const timestamp_t b);
timestamp_t timestamp_add_ms(const timestamp_t a, const msec_t ms);
timestamp_t timestamp_add_us(const timestamp_t a, const usec_t us);
timestamp_t timestamp_from_now_us(usec_t us);
timestamp_t timestamp_from_now_ms(msec_t ms);
timestamp_t timestamp();

#if 0
#define time_before(a,b)		\
	(typecheck(uint32_t, a) && \
	 typecheck(uint32_t, b) && \
	 ((signed long)((a) - (b)) < 0))
#define time_after(a,b)	time_before(b,a)

#define time_after_eq(a,b)	\
	(typecheck(unsigned long, a) && \
	 typecheck(unsigned long, b) && \
	 (((long)(a) - (long)(b)) >= 0))
#define time_before_eq(a,b)	time_after_eq(b,a)
#endif

//uint32_t nanos();

int timeval_subtract(const struct timeval *x, const struct timeval *y, struct timeval *result);
//int time_diff(const struct timeval *x, const struct timeval *y, struct timeval *result);
void time_cpy(struct timeval *dst, const struct timeval *src);

void time_init(void);
int time_init_rtc(void);

usec_t micros(void);
void time_gettime(struct timeval *ts);
uint32_t time_get_hse_precision(void);

void delay_us(uint32_t us);
uint32_t time_get_clock_speed();
int time_cpu_clock_speed_exact();
