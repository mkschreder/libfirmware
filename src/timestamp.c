<<<<<<< HEAD
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
* FILE ............... src/timestamp.c
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
=======
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

>>>>>>> license
#include "timestamp.h"

#define MILLION 1000000

int timeval_subtract(const struct timeval *x, const struct timeval *__y,
                     struct timeval *result) {
	struct timeval _y = {.tv_sec = __y->tv_sec, .tv_usec = __y->tv_usec};
	struct timeval *y = &_y;

	/* Perform the carry for the later subtraction by updating y. */
	if(x->tv_usec < y->tv_usec) {
		long int nsec = (y->tv_usec - x->tv_usec) / MILLION + 1;
		y->tv_usec -= MILLION * nsec;
		y->tv_sec += nsec;
	}
	if(x->tv_usec - y->tv_usec > MILLION) {
		long int nsec = (x->tv_usec - y->tv_usec) / MILLION;
		y->tv_usec += MILLION * nsec;
		y->tv_sec -= nsec;
	}

	/* Compute the time remaining to wait.
	   tv_usec is certainly positive. */
	result->tv_sec = x->tv_sec - y->tv_sec;
	result->tv_usec = x->tv_usec - y->tv_usec;

	/* Return 1 if result is negative. */
	return x->tv_sec < y->tv_sec;
}

void time_cpy(struct timeval *dst, const struct timeval *src) {
	dst->tv_sec = src->tv_sec;
	dst->tv_usec = src->tv_usec;
}

int timestamp_before(const timestamp_t a, const timestamp_t b) {
	timestamp_diff_t diff = timestamp_sub(a, b);
	return diff.sec < 0 || (diff.sec == 0 && diff.usec < 0);
}

int timestamp_after(const timestamp_t a, const timestamp_t b) {
	return timestamp_before(b, a);
}

int timestamp_expired(const timestamp_t a) {
	return timestamp_after(timestamp(), a);
}

timestamp_t timestamp_add(const timestamp_t a, const timestamp_t b) {
	timestamp_t r = {.sec = 0, .usec = 0};
	r.sec = a.sec + b.sec;
	r.usec = a.usec + b.usec;
	if(r.usec >= MILLION) {
		r.sec += r.usec / MILLION;
		r.usec = r.usec % MILLION;
	}
	return r;
}

timestamp_diff_t timestamp_sub(const timestamp_t _a, const timestamp_t _b) {
	timestamp_diff_t r = {.sec = 0, .usec = 0};
	uint64_t a = _a.sec * MILLION + _a.usec;
	uint64_t b = _b.sec * MILLION + _b.usec;
	int64_t res = (int64_t)(a - b);
	if(res < 0){
		r.sec = (sec_diff_t)(-((-res) / MILLION));
		r.usec = (usec_diff_t)(-((-res) % MILLION));
	} else {
		r.sec = (sec_diff_t)(res / MILLION);
		r.usec = (usec_diff_t)(res % MILLION);
	}
	return r;
}

timestamp_t timestamp_add_ms(const timestamp_t a, const msec_t ms) {
	timestamp_t d = {.sec = ms / 1000, .usec = (ms * 1000) % MILLION};
	return timestamp_add(a, d);
}

timestamp_t timestamp_add_us(const timestamp_t a, const usec_t us) {
	timestamp_t d = {.sec = us / MILLION, .usec = us % MILLION};
	return timestamp_add(a, d);
}

timestamp_t timestamp_from_now_us(usec_t us) {
	return timestamp_add_us(timestamp(), us);
}

timestamp_t timestamp_from_now_ms(msec_t ms) {
	return timestamp_add_ms(timestamp(), ms);
}
