/** :ms-top-comment
 *               __        ___          _     _____ ____
 *               \ \      / (_)___  ___| |   | ____|  _ \
 *                \ \ /\ / /| / __|/ _ \ |   |  _| | | | |
 *                 \ V  V / | \__ \  __/ |___| |___| |_| |
 *                  \_/\_/  |_|___/\___|_____|_____|____/
 *
 * FILE ............... src/cbuf.c
 * AUTHOR ............. Martin K. Schröder
 * VERSION ............ Not tagged
 * DATE ............... 2019-06-30
 * WEBSITE ............ http://wiseled.com
 * LICENSE ............ WiseLED
 *
 * This file is part of WiseLED Project.
 *
 * WiseLED Project can not be copied and/or distributed without the express
 * permission of WiseLED.
 *
 * +----------------------------------------------------------------------+
 * |         Swedish Embedded - We design hardware and program it         |
 * +----------------------------------------------------------------------+
 * |  This file was refactored and improved for WiseLED, on contract, by  |
 * | Swedish Embedded. If you have software that needs to be developed or |
 * |        hardware that needs to be designed, then get in touch!        |
 * |                                                                      |
 * |                       Phone: (+46)733-38-76-94                       |
 * |                   Email: info@swedishembedded.com                    |
 * +----------------------------------------------------------------------+
 **/
#include "cbuf.h"
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
