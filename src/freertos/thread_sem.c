#include "kernel.h"
#include "sem.h"

int thread_sem_init(struct semaphore *self){
	if((self->sem = xSemaphoreCreateBinary()) == NULL) return -1;
	return 0;
}

int thread_sem_init_counting(struct semaphore *self, uint16_t max_count, uint16_t initial_count){
	if((self->sem = xSemaphoreCreateCounting(max_count, initial_count)) == NULL) return -1;
	return 0;
}

int thread_sem_take(struct semaphore *self){
	if(xSemaphoreTake(self->sem, portMAX_DELAY) == pdTRUE) return 0;
	return -1;
}

int thread_sem_take_wait(struct semaphore *self, uint32_t timeout){
	if(xSemaphoreTake(self->sem, timeout / portTICK_PERIOD_MS) == pdTRUE) return 0;
	return -1;
}

int thread_sem_give(struct semaphore *self){
	if(xSemaphoreGive(self->sem) == pdTRUE) return 0;
	return -1;
}

int thread_sem_give_from_isr(struct semaphore *self){
	BaseType_t wake = 0;
	if(xSemaphoreGiveFromISR(self->sem, &wake) == pdTRUE){
		if(wake) return 1;
		return 0;
	}
	return -1;
}

