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

#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846264338327950288)
#endif

#ifndef __cplusplus
//#include <cstdint>
//#include <cmath>
#define min(x,y) (__extension__({ \
    typeof(x) _x = (x); \
    typeof(x) _y = (y); \
    (void) (&_x == &_y);        \
    (typeof(x))(_x < _y ? _x : _y); }))

#define max(x,y) ({ \
    typeof(x) _x = (x); \
    typeof(y) _y = (y); \
    (void) (&_x == &_y);        \
    _x > _y ? _x : _y; })
#endif

#define signf(x) (float)(((float)(x) > 0) - ((float)(x) < 0))

#define RAD_360 (2 * M_PI)
#define RAD_120 (2 * M_PI / 3)
#define RAD_60 (M_PI / 3)
#define RAD_30 (M_PI / 6)

/* use several alternatives to make full use of the -Wconversion warning */

#define _CONSTRAIN(x, a, b) (((x) <= (a))?(a):(((x) > (b))?(b):(x)))
static inline uint32_t constrain_u32(uint32_t x, uint32_t a, uint32_t b){
	return _CONSTRAIN(x, a, b);
}

static inline int32_t constrain_i32(int32_t x, int32_t a, int32_t b){
	return _CONSTRAIN(x, a, b);
}

static inline uint16_t constrain_u16(uint16_t x, uint16_t a, uint16_t b){
	return _CONSTRAIN(x, a, b);
}

static inline int16_t constrain_i16(int16_t x, int16_t a, int16_t b){
	return _CONSTRAIN(x, a, b);
}

static inline uint8_t constrain_u8(uint8_t x, uint8_t a, uint8_t b){
	return _CONSTRAIN(x, a, b);
}

static inline int8_t constrain_i8(int8_t x, int8_t a, int8_t b){
	return _CONSTRAIN(x, a, b);
}

static inline float constrain_float(float x, float a, float b){
	return _CONSTRAIN(x, a, b);
}



#define scale_range32(x, a, b, c, d) ((int32_t)(x) * ((d) - (c)) / ((b) - (a)))

//#define abs(x) (((x) < 0)?(-(x)):(x))

#define deg2rad(x) ((float)(x) * 3.14f / 180.f)
#define rad2deg(x) ((float)(x) / (3.14f / 180.f))

#define FLOAT_EPSILON (1e-6f)

void clamp_rad360(float *angle);

static inline int16_t u16_diff(uint16_t a, uint16_t b){
	return (int16_t)((int16_t)a - (int16_t)b);
}

static inline int32_t u32_diff(uint32_t a, uint32_t b){
	return ((int32_t)a - (int32_t)b);
}

int32_t lowpass_i32(int32_t x, int32_t prev, uint16_t num, uint16_t denom);
uint32_t lowpass_phase32(uint32_t x, uint32_t prev, uint16_t num, uint16_t denom);
float lowpass_f32(float x, float prev, float frac_new);

uint8_t majority_filter(uint8_t emf, uint8_t last_known_sector, uint8_t prev_out);

int8_t fastsin_127deg(uint16_t x);
int8_t fastcos_127deg(uint16_t x);
int8_t fastsin_127u16(uint16_t x);
int8_t fastcos_127u16(uint16_t x);
int8_t fastsin_127u32(uint32_t x);
int8_t fastcos_127u32(uint32_t x);
float fastsinf(float x);
float fastcosf(float x);

