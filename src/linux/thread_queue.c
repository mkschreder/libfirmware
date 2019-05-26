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
* FILE ............... src/linux/thread_queue.c
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
/**
 * Taken from StackOverflow thread: http://stackoverflow.com/questions/4577961/pthread-synchronized-blocking-queue
 *
 * Modified to fit the needs of portable message queue that we can implement
 * both on top of freertos and on top of pthreads and make sure that it behaves
 * in the same way.
 */

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdbool.h>

#include <sys/time.h>

#include "queue.h"
#include "thread.h"

struct threadmsg {
	void *data;
	struct threadmsg *next;
};

struct msglist {
	struct threadmsg msg;
	struct msglist *next;
};

struct linux_queue {
	pthread_cond_t cond;
	pthread_cond_t cond_not_full;
	pthread_mutex_t mutex;
	struct msglist *first, *last;
	struct msglist *msgpool;
	size_t length;
	size_t elem_size;
	size_t total_elems;
};

/*
static inline struct msglist *get_msglist(struct thread_queue *self){
	struct msglist *tmp;

	if(self->msgpool != NULL) {
		tmp = self->msgpool;
		self->msgpool = tmp->next;
		self->msgpool_length--;
	} else {
		tmp = malloc(sizeof *tmp);
	}

	return tmp;
}

static inline void release_msglist(struct thread_queue *self,struct msglist *node){
	if(self->msgpool_length > ( self->length/8 + MSGPOOL_SIZE)) {
		free(node);
	} else {
		node->msg.data = NULL;
		node->msg.msgtype = 0;
		node->next = self->msgpool;
		self->msgpool = node;
		self->msgpool_length++;
	}
	if(self->msgpool_length > (self->length/4 + MSGPOOL_SIZE*10)) {
		struct msglist *tmp = self->msgpool;
		self->msgpool = tmp->next;
		free(tmp);
		self->msgpool_length--;
	}
}
*/
int thread_queue_init(struct thread_queue *tq, size_t nelem, size_t elem_size){
	int ret = 0;
	if (!tq) {
		return -EINVAL;
	}
	memset(tq, 0, sizeof(struct thread_queue));
	struct linux_queue *self = malloc(sizeof(struct linux_queue));

	ret = pthread_cond_init(&self->cond, NULL);
	if (ret != 0) {
		return ret;
	}

	ret = pthread_cond_init(&self->cond_not_full, NULL);
	if (ret != 0) {
		return ret;
	}

	ret = pthread_mutex_init(&self->mutex, NULL);
	if (ret != 0) {
		pthread_cond_destroy(&self->cond);
		return ret;
	}

	self->total_elems = nelem;
	self->elem_size = elem_size;
	tq->handle = self;
	return 0;
}

static void _timespec_from_now_us(struct timespec *abstimeout, uint32_t tout_us){
	struct timeval now;

	unsigned long tv_sec = tout_us / 1000000;
	unsigned long tv_nsec = (tout_us % 1000000) * 1000;

	//printf("will wait for %lu %lu\n", tv_sec, tv_nsec);

	gettimeofday(&now, NULL);
	abstimeout->tv_sec = (suseconds_t)(now.tv_sec + (long int)tv_sec);
	abstimeout->tv_nsec = (suseconds_t)((now.tv_usec * 1000) + (long int)tv_nsec);
	if (abstimeout->tv_nsec >= 1000000000) {
		abstimeout->tv_sec++;
		abstimeout->tv_nsec -= 1000000000;
	}
}

static int _thread_queue_send(struct thread_queue *tq, const void *data, uint32_t tout_us, bool ovrwrt){
	struct msglist *newmsg;
	int ret = 0;
	struct linux_queue *self = (struct linux_queue*)tq->handle;

	struct timespec abstimeout;

	_timespec_from_now_us(&abstimeout, tout_us);

	pthread_mutex_lock(&self->mutex);

	// if we want to overwrite and the queue has an element then just do it and return
	if(ovrwrt && self->length != 0){
		memcpy(self->first->msg.data, data, self->elem_size);
		pthread_cond_broadcast(&self->cond);
		pthread_mutex_unlock(&self->mutex);
		return 1;
	}

	// while the queue is full we wait until some element becomes available
	while ((self->length == self->total_elems) && ret != ETIMEDOUT) {  //Need to loop to handle spurious wakeups
		if (tout_us != THREAD_WAIT_FOREVER) {
			ret = pthread_cond_timedwait(&self->cond_not_full, &self->mutex, &abstimeout);
		} else {
			pthread_cond_wait(&self->cond_not_full, &self->mutex);

		}
	}

	if(self->length == self->total_elems){
		pthread_mutex_unlock(&self->mutex);
		return -1;
	}

	// TODO: for now we just use malloc. This is not very fast though..
	newmsg = (struct msglist*)malloc(sizeof(struct msglist));
	if (newmsg == NULL) {
		pthread_mutex_unlock(&self->mutex);
		return ENOMEM;
	}
	newmsg->msg.data = malloc(self->elem_size);
	memcpy(newmsg->msg.data, data, self->elem_size);

	newmsg->next = NULL;
	if (self->last == NULL) {
		self->last = newmsg;
		self->first = newmsg;
	} else {
		self->last->next = newmsg;
		self->last = newmsg;
	}

	if(self->length == 0)
		pthread_cond_broadcast(&self->cond);

	self->length++;

	pthread_mutex_unlock(&self->mutex);

	return 1;
}

int thread_queue_send(struct thread_queue *self, const void *data, uint32_t tout_us){
	return _thread_queue_send(self, data, tout_us, false);
}

int thread_queue_send_from_isr(struct thread_queue *self, const void *data, int32_t *wait){
	return _thread_queue_send(self, data, 0, false);
}

int thread_queue_overwrite(struct thread_queue *self, void *data){
	return _thread_queue_send(self, data, 0, true);
}

static int _thread_queue_recv(struct thread_queue *tq, void *data, uint32_t tout_ms, bool remove){
	struct msglist *firstrec;
	int ret = 0;
	struct timespec abstimeout;

	if (tq == NULL || data == NULL) {
		return -EINVAL;
	}
	struct linux_queue *self = (struct linux_queue*)tq->handle;

	_timespec_from_now_us(&abstimeout, tout_ms);

	pthread_mutex_lock(&self->mutex);

	/* Will wait until awakened by a signal or broadcast */
	while (self->first == NULL && ret != ETIMEDOUT) {  //Need to loop to handle spurious wakeups
		if (tout_ms != THREAD_WAIT_FOREVER) {
			ret = pthread_cond_timedwait(&self->cond, &self->mutex, &abstimeout);
		} else {
			pthread_cond_wait(&self->cond, &self->mutex);
		}
	}

	if (ret == ETIMEDOUT) {
		pthread_mutex_unlock(&self->mutex);
		return -ETIMEDOUT;
	}

	if(remove){
		firstrec = self->first;
		self->first = self->first->next;
		self->length--;

		if (self->first == NULL) {
			self->last = NULL;	 // we know this since we hold the lock
			self->length = 0;
		}

		memcpy(data, firstrec->msg.data, self->elem_size);

		// since we have read the element and removed it, the queue is definitely not full
		pthread_cond_broadcast(&self->cond_not_full);

		free(firstrec);
	} else {
		memcpy(data, self->first->msg.data, self->elem_size);
	}

	pthread_mutex_unlock(&self->mutex);

	return 1;
}

int thread_queue_recv(struct thread_queue *queue, void *data, uint32_t tout_ms){
	return _thread_queue_recv(queue, data, tout_ms, true);
}

int thread_queue_peek(struct thread_queue *queue, void *data, uint32_t tout_ms){
	return _thread_queue_recv(queue, data, tout_ms, false);
}

//maybe caller should supply a callback for cleaning the elements ?
int thread_queue_cleanup(struct thread_queue *tq){
	struct msglist *rec;
	struct msglist *next;
	struct msglist *recs[2];
	int ret,i;
	if (tq == NULL) {
		return EINVAL;
	}
	struct linux_queue *self = (struct linux_queue*)tq->handle;
	pthread_mutex_lock(&self->mutex);
	recs[0] = self->first;
	recs[1] = self->msgpool;
	for(i = 0; i < 2 ; i++) {
		rec = recs[i];
		while (rec) {
			next = rec->next;
			//if (freedata) {
				free(rec->msg.data);
			//}
			free(rec);
			rec = next;
		}
	}

	pthread_mutex_unlock(&self->mutex);
	ret = pthread_mutex_destroy(&self->mutex);
	pthread_cond_destroy(&self->cond);

	return ret;

}

long thread_queue_length(struct thread_queue *tq){
	long counter;
	struct linux_queue *self = (struct linux_queue*)tq->handle;
	// get the length properly
	pthread_mutex_lock(&self->mutex);
	counter = (long)self->length;
	pthread_mutex_unlock(&self->mutex);
	return counter;
}
