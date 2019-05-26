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
* FILE ............... src/termui.c
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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "math.h"
#include "termui.h"

#define fabsf(x) ((x < 0)?(-(x)):(x))

void termui_init(struct termui *self, console_device_t console){
	memset(self, 0, sizeof(*self));
	self->console = console;
}

void termui_clear(struct termui *self){
	console_printf(self->console, "\033[2J");
}

void termui_home(struct termui *self){
	console_printf(self->console, "\033[H");
}

void termui_show_cursor(struct termui *self, bool show){
	if(show){
		console_printf(self->console, "\033[?25h");
	} else {
		console_printf(self->console, "\033[?25l");
	}
}

void termui_move_cursor(struct termui *self, unsigned x, unsigned y){
	console_printf(self->console, "\033[%d;%dH", y, x);
}

void termui_setattr(struct termui *self, termui_attr_t attr){
	console_printf(self->console, "\033[%dm", attr);
}

void termui_resetattr(struct termui *self){
	termui_setattr(self, ATTR_RESET);
}

void termui_draw_float(struct termui *self, unsigned width, float value){
	char str[12];
	snprintf(str, sizeof(str), "%d.%d", (int)(value), (int)(fabsf(value) * 1000) % 1000);

	termui_setattr(self, ATTR_BG_BLUE);
	termui_setattr(self, ATTR_FG_WHITE);
	size_t len = strlen(str);
	size_t half = (width - len) / 2;
	for(size_t c = 0; c < half; c++) console_printf(self->console, " ");
	console_printf(self->console, "%s", str);
	for(size_t c = 0; c < width - (half + len); c++) console_printf(self->console, " ");
	termui_resetattr(self);
}

void termui_draw_int(struct termui *self, unsigned width, int value){
	char str[12];
	snprintf(str, sizeof(str), "%d", (int)(value));

	termui_setattr(self, ATTR_BG_BLUE);
	termui_setattr(self, ATTR_FG_WHITE);
	size_t len = strlen(str);
	size_t half = (width - len) / 2;
	for(size_t c = 0; c < half; c++) console_printf(self->console, " ");
	console_printf(self->console, "%s", str);
	for(size_t c = 0; c < width - (half + len); c++) console_printf(self->console, " ");
	termui_resetattr(self);
}

void termui_draw_text_centered(struct termui *self, unsigned width, const char *str){
	size_t len = strlen(str);
	size_t half = (width - len) / 2;
	for(size_t c = 0; c < half; c++) console_printf(self->console, " ");
	console_printf(self->console, "%s", str);
	for(size_t c = 0; c < width - (half + len); c++) console_printf(self->console, " ");
}

void termui_draw_hex(struct termui *self, unsigned width, int value, int len){
	char str[8];
	snprintf(str, sizeof(str), "%0*x", len, value);

	termui_setattr(self, ATTR_BG_BLUE);
	termui_setattr(self, ATTR_FG_WHITE);

	termui_draw_text_centered(self, width, str);

	termui_resetattr(self);
}

void termui_draw_text(struct termui *self, unsigned width, const char *str){
	size_t len = strlen(str);
	size_t half = (width - len) / 2;
	for(size_t c = 0; c < half; c++) console_printf(self->console, " ");
	console_printf(self->console, "%s", str);
	for(size_t c = 0; c < width - (half + len); c++) console_printf(self->console, " ");
}

void termui_draw_label(struct termui *self, unsigned w, const char *text){
	char fmt[8];
	snprintf(fmt, sizeof(fmt), "%%-%ds", w);
	console_printf(self->console, fmt, text);
}

void termui_draw_frame(struct termui *self, unsigned x, unsigned y, unsigned w, unsigned h, const char *title){
	// draw the border
	termui_setattr(self, ATTR_DIM);
	termui_setattr(self, ATTR_FG_WHITE);

	termui_move_cursor(self, x - 1, y - 1);
	console_printf(self->console, "\033(0\x6c\033(B"); // corner
	console_printf(self->console, "\033(0\x71\033(B %s ", title);
	for(unsigned c = 0; c < w - 3 - strlen(title); c++){
		console_printf(self->console, "\033(0\x71\033(B");
	}
	console_printf(self->console, "\033(0\x6b\033(B"); // corner
	termui_move_cursor(self, x - 1, y + h);
	console_printf(self->console, "\033(0\x6d\033(B");
	for(unsigned c = 0; c < w; c++){
		console_printf(self->console, "\033(0\x71\033(B");
	}
	console_printf(self->console, "\033(0\x6a\033(B");

	// draw left and right border
	termui_move_cursor(self, x - 1, y);
	for(unsigned c = 0; c < h; c++){
		console_printf(self->console, "\033(0\x78\033(B\033[D\033[B");
	}
	termui_move_cursor(self, x + w, y);
	for(unsigned c = 0; c < h; c++){
		console_printf(self->console, "\033(0\x78\033(B\033[D\033[B");
	}

	// reset to default text format
	termui_setattr(self, ATTR_RESET);
}

void termui_draw_ind_float(struct termui *self, unsigned x, unsigned y, unsigned width, const char *name, float value){
	termui_move_cursor(self, x, y);
	termui_draw_float(self, width, value);
	termui_move_cursor(self, x, y + 1);
	termui_setattr(self, ATTR_FG_BLUE);
	termui_setattr(self, ATTR_BG_WHITE);
	termui_draw_text(self, width, name);
	termui_setattr(self, ATTR_RESET);
}

void termui_draw_ind_int(struct termui *self, unsigned x, unsigned y, unsigned width, const char *name, int value){
	termui_draw_ind_float(self, x, y, width, name, (float)value);
}

