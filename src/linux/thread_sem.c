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
* FILE ............... src/linux/thread_sem.c
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
#include <memory.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdint.h>

#include <semaphore.h>

#include "sem.h"

int thread_sem_init(struct semaphore *self){
	if((self->sem = malloc(sizeof(sem_t))) == NULL) return -1;
	sem_init(self->sem, 0, 0);
	return 0;
}

int thread_sem_init_counting(struct semaphore *self, uint16_t max_count, uint16_t initial_count){
	if((self->sem = malloc(sizeof(sem_t))) == NULL) return -1;
	sem_init(self->sem, 0, initial_count);
	return 0;
}

int thread_sem_destroy(struct semaphore *self){
	return sem_destroy(self->sem);
}

int thread_sem_take(struct semaphore *self){
	return sem_wait(self->sem);
}

int thread_sem_take_wait(struct semaphore *self, uint32_t timeout){
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec += timeout / 1000;
	ts.tv_nsec += timeout * 1000000;
	if (ts.tv_nsec>=1000000000) {
		ts.tv_sec+=1;
		ts.tv_nsec-=1000000000;
	}
	return sem_timedwait(self->sem, &ts);
}

int thread_sem_give(struct semaphore *self){
	sem_post(self->sem);
	return 0;
}

int thread_sem_give_from_isr(struct semaphore *self, int32_t *wake){
	return thread_sem_give(self);
}

