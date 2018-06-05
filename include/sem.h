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

struct semaphore {
	void *sem;
};

int thread_sem_init(struct semaphore *self);
int thread_sem_init_counting(struct semaphore *self, uint16_t max_count, uint16_t initial_count);
int thread_sem_destroy(struct semaphore *self);
int thread_sem_take(struct semaphore *self);
int thread_sem_take_wait(struct semaphore *self, uint32_t timeout);
int thread_sem_give(struct semaphore *self);
int thread_sem_give_from_isr(struct semaphore *self);
