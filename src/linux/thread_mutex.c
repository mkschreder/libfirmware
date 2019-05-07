#include <pthread.h>
#include <stdlib.h>

#include "mutex.h"
#include "timestamp.h"

int thread_mutex_init(struct mutex *self){
	self->sem = malloc(sizeof(pthread_mutex_t));
	return pthread_mutex_init((pthread_mutex_t*)self->sem, NULL);
}

int thread_mutex_destroy(struct mutex *self){
	return pthread_mutex_destroy((pthread_mutex_t*)self->sem);
}

int thread_mutex_lock(struct mutex *self){
	return pthread_mutex_lock((pthread_mutex_t*)self->sem);
}

int thread_mutex_lock_wait(struct mutex *self, msec_t wait_time){
	return pthread_mutex_lock((pthread_mutex_t*)self->sem);
}

int thread_mutex_unlock(struct mutex *self){
	return pthread_mutex_unlock((pthread_mutex_t*)self->sem);
}

