/** :ms-top-comment
 *               __        ___          _     _____ ____
 *               \ \      / (_)___  ___| |   | ____|  _ \
 *                \ \ /\ / /| / __|/ _ \ |   |  _| | | | |
 *                 \ V  V / | \__ \  __/ |___| |___| |_| |
 *                  \_/\_/  |_|___/\___|_____|_____|____/
 *
 * FILE ............... include/cbuf.h
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
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

struct cbuf {
	size_t head, tail;
	void *data;
	size_t item_size, nitems;
	bool full;
};

void cbuf_init(struct cbuf *self, void *data, size_t item_size, size_t nitems);
void cbuf_clear(struct cbuf *self);
int cbuf_put(struct cbuf *self, const void *item);
int cbuf_put_unless_full(struct cbuf *self, const void *item);
int cbuf_get(struct cbuf *self, void *item);
size_t cbuf_capacity(struct cbuf *self);
size_t cbuf_size(struct cbuf *self);
bool cbuf_full(struct cbuf *self);
bool cbuf_empty(struct cbuf *self);
