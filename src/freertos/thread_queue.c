#include "kernel.h"
#include "queue.h"

int thread_queue_init(struct thread_queue *queue, size_t elems, size_t elem_size){
	queue->handle = xQueueCreate(elems, elem_size);
	if(!queue->handle) return -1;
    return 0;
}

int thread_queue_send(struct thread_queue *self, const void *data, uint32_t tout_ms){
	if(xQueueSend(self->handle, data, tout_ms) == pdTRUE) return 1;
	return -1;
}

int thread_queue_send_from_isr(struct thread_queue *self, const void *data, int32_t *wake){
	if(xQueueSendFromISR(self->handle, data, wake) == pdTRUE) return 1;
	return -1;
}

int thread_queue_recv(struct thread_queue *self, void *data, uint32_t tout_ms){
	if(xQueueReceive(self->handle, data, tout_ms) == pdTRUE) return 1;
	return -1;
}

int thread_queue_recv_from_isr(struct thread_queue *self, void *data, int32_t *wake){
	if(xQueueReceiveFromISR(self->handle, data, wake) == pdTRUE) return 1;
	return -1;
}


int thread_queue_peek(struct thread_queue *self, void *data, uint32_t tout_ms){
	if(xQueuePeek(self->handle, data, tout_ms) == pdTRUE) return 0;
	return -1;
}

int thread_queue_overwrite(struct thread_queue *self, void *data){
	if(xQueueOverwrite(self->handle, data) == pdTRUE) return 0;
	return -1;
}

long thread_queue_length( struct thread_queue *self){
	return (long)uxQueueMessagesWaiting(self->handle);
}

int thread_queue_cleanup(struct thread_queue *self){
	(void)self;
	// TODO: implement queue deletion
	//xQueueDelete(self->handle);
	return 0;
}

