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
* FILE ............... include/timestamp.h
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
#pragma once

#include <time.h>

#if defined(HAVE_SYS_TIME_H)
#include <sys/time.h>
#endif

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
