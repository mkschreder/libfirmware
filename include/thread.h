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
#include <stdlib.h>
#include <string.h>

#define THREAD_WAIT_FOREVER 0xffffffff

struct thread_status {
	uint8_t dummy;
};

typedef void* thread_t;
typedef struct thread_status thread_status_t;

int thread_create(void (*thread)(void*), const char *name, uint32_t stack_words, void *ptr, uint8_t priority, thread_t *handle);
int thread_join(thread_t handle);

void thread_set_tag(thread_t thread, void *tag);
void thread_start_scheduler(void);
int thread_sleep_ms(uint32_t ms);
int thread_sleep_us(uint32_t us);

/**
 * Used in interrupt handlers to signal operating system that a reschedule may be necessary
 */
void thread_yield_from_isr(int32_t wake);

void thread_sched_suspend();
void thread_sched_resume();

void thread_hard_delay_us(uint32_t us);

void* kmalloc(size_t size);
void* kzmalloc(size_t size);
void kfree(void *ptr);

void thread_meminfo();

unsigned long thread_get_free_heap();
unsigned long thread_get_total_heap();
 
