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
* FILE ............... src/linux/thread.c
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
#include <pthread.h>
#include <stdlib.h>
#include <memory.h>
#include <stdint.h>
#include <unistd.h>

#include "thread.h"

struct thread_arg {
	void (*proc)(void*);
	void *ptr;
};

//! A wrapper for pthread signature for threads
void *_thread_handler(void *ptr){
	struct thread_arg *arg = (struct thread_arg*)ptr;
	arg->proc(arg->ptr);
	free(arg);
	return NULL;
}

int thread_create(void (*thread)(void*), const char *name, uint32_t stack_words, void *ptr, uint8_t priority, thread_t *handle){
	(void)name;
	(void)stack_words;
	(void)priority;
	pthread_t th;
	if(!handle) handle = (thread_t*)&th;
	struct thread_arg *arg = malloc(sizeof(struct thread_arg));
	arg->proc = thread;
	arg->ptr = ptr;
	pthread_create((pthread_t*)handle, NULL, _thread_handler, arg);
	return 0;
}

int thread_join(thread_t handle){
	pthread_join((pthread_t)handle, NULL);
    return 0;
}

void thread_suspend(){

}

void thread_sched_suspend(){

}

void thread_sched_resume(){

}

void thread_meminfo(){

}

unsigned long thread_get_total_heap(){
	return 0;
}

unsigned long thread_get_free_heap(){
	return 0;
}

uint32_t thread_ticks_count(){
	return micros() / 1000;
}

uint32_t thread_ticks_from_us(usec_t us){
	return us / 1000;
}

void thread_enter_critical(void){
	// TODO: see how we should do this
}

void thread_start_scheduler(void){
	// TODO: need to join with all threads
	while(1){
		thread_sleep_ms(1000);
	}
}

void thread_set_tag(thread_t thread, void *tag){
	(void)thread;
	(void)tag;
	// TODO: make this set the tag using pthread func
}

int thread_sleep_ms(msec_t ms){
	usleep(ms * 1000);
    return 0;
}

int thread_sleep_us(usec_t us){
	usleep(us);
    return 0;
}

int thread_sleep_ms_until(msec_t *until, msec_t ms){
	return thread_sleep_ms(ms);
}

void *kzmalloc(size_t size){
	void *m = malloc(size);
	if(!m) return 0;
	memset(m, 0, size);
	return m;
}

void *kmalloc(size_t size){
	return malloc(size);
}

void kfree(void *ptr){
	free(ptr);
}

