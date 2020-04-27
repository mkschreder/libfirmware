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
* FILE ............... include/thread.h
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
int thread_sleep_ms_until(uint32_t *last, uint32_t ms);
void thread_yield();
void thread_suspend();

int thread_sleep_us(uint32_t us);
uint32_t thread_ticks_count();
uint32_t thread_ticks_from_us(uint32_t us);

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
 
void thread_update_clocks();
