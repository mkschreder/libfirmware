/*
 * Copyright (C) 2017 Martin K. Schröder <mkschreder.uk@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#pragma once

#include <stdint.h>

struct thread_queue {
	void* handle;
};

int thread_queue_init(struct thread_queue *queue, size_t elems, size_t elem_size);
int thread_queue_send(struct thread_queue *queue, const void *data, uint32_t tout_ms);
int thread_queue_send_from_isr(struct thread_queue *queue, const void *data, int32_t *need_reschedule);
int thread_queue_recv(struct thread_queue *queue, void *data, uint32_t tout_ms);
int thread_queue_recv_from_isr(struct thread_queue *queue, void *data, int32_t *need_reschedule);
int thread_queue_peek(struct thread_queue *queue, void *data, uint32_t tout_ms);
int thread_queue_overwrite(struct thread_queue *queue, void *data);
long thread_queue_length( struct thread_queue *queue );
int thread_queue_cleanup(struct thread_queue *queue);
