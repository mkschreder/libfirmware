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

#include "types/cbuf.h"
#include <string.h>

void cbuf_init(struct cbuf *self, void *data, size_t item_size, size_t nitems) {
	memset(self, 0, sizeof(*self));
	self->head = self->tail = 0;
	self->data = data;
	self->nitems = nitems;
	self->item_size = item_size;
	self->full = false;
}

void cbuf_clear(struct cbuf *self) {
	self->head = self->tail = 0;
	self->full = false;
}

static void cbuf_advance(struct cbuf *self) {
	if(self->full)
		self->tail = (self->tail + 1) % self->nitems;
	self->head = (self->head + 1) % self->nitems;
	self->full = self->head == self->tail;
}

static void cbuf_retreat(struct cbuf *self) {
	self->full = false;
	self->tail = (self->tail + 1) % self->nitems;
}

int cbuf_put(struct cbuf *self, const void *item) {
	bool full = self->full;
	memcpy(((uint8_t *)self->data) + (self->item_size * self->head), item,
	       self->item_size);
	cbuf_advance(self);
	return full;
}

int cbuf_put_unless_full(struct cbuf *self, const void *item) {
	if(self->full)
		return -1;
	memcpy(((uint8_t *)self->data) + (self->item_size * self->head), item,
	       self->item_size);
	cbuf_advance(self);
	return 0;
}

int cbuf_get(struct cbuf *self, void *item) {
	if(cbuf_empty(self))
		return 0;
	memcpy(item, (uint8_t *)self->data + (self->item_size * self->tail),
	       self->item_size);
	cbuf_retreat(self);
	return 1;
}

bool cbuf_full(struct cbuf *self) {
	return self->full;
}

bool cbuf_empty(struct cbuf *self) {
	return !self->full && (self->head == self->tail);
}

size_t cbuf_capacity(struct cbuf *self) {
	return self->nitems;
}

size_t cbuf_size(struct cbuf *self) {
	if(self->full)
		return self->nitems;
	if(self->head >= self->tail)
		return self->head - self->tail;
	return self->nitems + self->head - self->tail;
}
