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

#include "timestamp.h"

#define MILLION 1000000

int timeval_subtract(const struct timeval *x, const struct timeval *__y, struct timeval *result){
	struct timeval _y = {
		.tv_sec = __y->tv_sec,
		.tv_usec = __y->tv_usec
	};
	struct timeval *y = &_y;

	/* Perform the carry for the later subtraction by updating y. */
	if (x->tv_usec < y->tv_usec) {
		long int nsec = (y->tv_usec - x->tv_usec) / MILLION + 1;
		y->tv_usec -= MILLION * nsec;
		y->tv_sec += nsec;
	}
	if (x->tv_usec - y->tv_usec > MILLION) {
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

void time_cpy(struct timeval *dst, const struct timeval *src){
	dst->tv_sec = src->tv_sec;
	dst->tv_usec = src->tv_usec;
}

int timestamp_before(const timestamp_t a, const timestamp_t b){
	timestamp_diff_t diff = timestamp_sub(a, b);
	return diff.sec <= 0 || (diff.sec == 0 && diff.usec < 0);
}

int timestamp_after(const timestamp_t a, const timestamp_t b){
	return timestamp_before(b, a);
}

int timestamp_expired(const timestamp_t a){
	return timestamp_after(timestamp(), a);
}

timestamp_t timestamp_add(const timestamp_t a, const timestamp_t b){
	timestamp_t r = { .sec = 0, .usec = 0 };
	r.sec = a.sec + b.sec;
	r.usec = a.usec + b.usec;
	if(r.usec >= MILLION){
		r.sec += r.usec / MILLION;
		r.usec = r.usec % MILLION;
	}
	return r;
}

timestamp_diff_t timestamp_sub(const timestamp_t a, const timestamp_t b){
	timestamp_diff_t r = { .sec = 0, .usec = 0 };
	r.sec = (sec_diff_t)(a.sec - b.sec);
	r.usec = (usec_diff_t)(a.usec - b.usec);
	r.sec += r.usec / MILLION;
	if(r.usec < 0){
		r.usec = -((-r.usec) % MILLION);
	} else {
		r.usec = r.usec % MILLION;
	}
	return r;
}

timestamp_t timestamp_add_ms(const timestamp_t a, const msec_t ms){
	timestamp_t d = { .sec = ms / 1000, .usec = (ms * 1000) % MILLION };
	return timestamp_add(a, d);
}

timestamp_t timestamp_add_us(const timestamp_t a, const usec_t us){
	timestamp_t d = { .sec = us / MILLION, .usec = us % MILLION };
	return timestamp_add(a, d);
}

timestamp_t timestamp_from_now_us(usec_t us){
	return timestamp_add_us(timestamp(), us);
}

timestamp_t timestamp_from_now_ms(msec_t ms){
	return timestamp_add_ms(timestamp(), ms);
}
