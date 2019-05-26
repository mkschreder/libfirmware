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
* FILE ............... include/atomic.h
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

// int would be fairly standard
typedef unsigned long __atomic_t;
typedef volatile __atomic_t atomic_t;

//! atomically adds other to self and returns previous value of self
static __attribute__((always_inline)) inline __atomic_t atomic_add(atomic_t *self, atomic_t other){
	return __sync_fetch_and_add(self, other);
}

//! atomically subtracts value from an atomic type and returns previous value of self
static __attribute__((always_inline)) inline __atomic_t atomic_sub(atomic_t *self, atomic_t other){
	return __sync_fetch_and_sub(self, other);
}

#define atomic_inc(x) atomic_add((x), 1)
#define atomic_dec(x) atomic_sub((x), 1)

static inline __atomic_t atomic_or(atomic_t *self, atomic_t other){
	return __sync_or_and_fetch(self, other);
}

static inline __atomic_t atomic_and(atomic_t *self, atomic_t other){
	return __sync_and_and_fetch(self, other);
}

static inline __atomic_t atomic_nand(atomic_t *self, atomic_t other){
	return __sync_nand_and_fetch(self, other);
}

static inline __atomic_t atomic_compare_and_swap(atomic_t *self, atomic_t tval, atomic_t sval){
	return __sync_val_compare_and_swap(self, tval, sval);
}

__atomic_t atomic_set(atomic_t *self, atomic_t other);

static inline __atomic_t atomic_get(atomic_t *self){
	// this one is kindof pointless but we use it for consistency
	return *self;
}

#define atomic_bit_set(a, b) atomic_or((a), (b))
#define atomic_bit_reset(a, b) atomic_and((a), (__atomic_t)~(b))
