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
* FILE ............... src/freertos/thread_queue.c
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
#include "kernel.h"
#include "thread/queue.h"

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

