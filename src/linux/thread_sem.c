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

