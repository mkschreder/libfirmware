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
* FILE ............... include/math.h
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

#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846264338327950288)
#endif

#ifndef __cplusplus
//#include <cstdint>
//#include <cmath>
#define min(x,y) (__extension__({ \
    __typeof__(x) _x = (x); \
    __typeof__(x) _y = (y); \
    (void) (&_x == &_y);        \
    (__typeof__(x))(_x < _y ? _x : _y); }))

#define max(x,y) ({ \
    __typeof__(x) _x = (x); \
    __typeof__(y) _y = (y); \
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
	return (uint8_t)_CONSTRAIN(x, a, b);
}

static inline int8_t constrain_i8(int8_t x, int8_t a, int8_t b){
	return (int8_t)_CONSTRAIN(x, a, b);
}

static inline float constrain_float(float x, float a, float b){
	return _CONSTRAIN(x, a, b);
}

#define normalize_angle(angle) atan2f(sinf(angle), cosf(angle))

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

