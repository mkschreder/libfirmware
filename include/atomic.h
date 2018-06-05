/*
 * This file is part of Ninjaflight.
 *
 * Copyright (c) 2017 Martin Schröder <mkschreder.uk@gmail.com>
 *
 * Ninjaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ninjaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Ninjaflight.  If not, see <http://www.gnu.org/licenses/>.
 */


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
