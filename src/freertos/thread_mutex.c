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

int thread_mutex_lock_wait(struct mutex *self, uint32_t block_time_ms){
	if(!self->sem) return -EINVAL;
	if(xSemaphoreTake(self->sem, (block_time_ms == THREAD_SLEEP_MAX_DELAY)?(portMAX_DELAY):(block_time_ms / portTICK_PERIOD_MS)) == pdTRUE) return 0;
	return -EAGAIN;
}

int thread_mutex_lock(struct mutex *self){
	if(!self->sem) return -EINVAL;
	return thread_mutex_lock_wait(self, THREAD_SLEEP_MAX_DELAY);
}

int thread_mutex_unlock(struct mutex *self){
	if(!self->sem) return -EINVAL;
	if(xSemaphoreGive(self->sem) == pdTRUE) return 0;
	return -1;
}

