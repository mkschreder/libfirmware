#include "kernel.h"
#include "mutex.h"
#include <string.h>
#include <errno.h>

int thread_mutex_init(struct mutex *self){
	memset(self, 0, sizeof(*self));
	SemaphoreHandle_t s = xSemaphoreCreateMutex();
	if(s == NULL) return -ENOMEM;
	self->sem = s;
    return 0;
}

int thread_mutex_lock_wait(struct mutex *self, int block_time_ms){
	if(!self->sem) return -EINVAL;
	TickType_t time = ((block_time_ms == -1)?portMAX_DELAY:(portTICK_PERIOD_MS * (long unsigned int)block_time_ms));
	if(xSemaphoreTake(self->sem, time) == pdTRUE) return 0;
	return -EAGAIN;
}

int thread_mutex_lock(struct mutex *self){
	if(!self->sem) return -EINVAL;
	return thread_mutex_lock_wait(self, -1);
}

int thread_mutex_unlock(struct mutex *self){
	if(!self->sem) return -EINVAL;
	if(xSemaphoreGive(self->sem) == pdTRUE) return 0;
	return -1;
}

