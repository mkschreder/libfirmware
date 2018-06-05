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

#include "spinlock.h"
#include "list.h"

struct work {
	spinlock_t lock;
	struct list_head list;
	void (*handler)(struct work *work);
	void *priv;
};

void work_init(struct work *work, void (*handler)(struct work *));
void work_destroy(struct work *work);

//! queue work on the default work queue
int queue_work(struct work *work, uint32_t delay_ms);
int reschedule_work(struct work *self, uint32_t delay_ms);
int queue_work_from_isr(struct work *work, uint32_t delay_ms, int32_t *wake);
