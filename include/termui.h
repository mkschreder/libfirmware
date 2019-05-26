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
* FILE ............... include/termui.h
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
#pragma once

#include "console.h"

struct termui {
	console_device_t console;
};

typedef enum {
	ATTR_RESET = 0,
	ATTR_BRIGHT,
	ATTR_DIM,
	ATTR_UNDERSCORE,
	ATTR_BLINK,
	ATTR_REVERSE,
	ATTR_HIDDEN,
	ATTR_FG_BLACK = 30,
	ATTR_FG_RED,
	ATTR_FG_GREEN,
	ATTR_FG_YELLOW,
	ATTR_FG_BLUE,
	ATTR_FG_MAGENTA,
	ATTR_FG_CYAN,
	ATTR_FG_WHITE,
	ATTR_BG_BLACK = 40,
	ATTR_BG_RED,
	ATTR_BG_GREEN,
	ATTR_BG_YELLOW,
	ATTR_BG_BLUE,
	ATTR_BG_MAGENTA,
	ATTR_BG_CYAN,
	ATTR_BG_WHITE
} termui_attr_t;

void termui_init(struct termui *self, console_device_t console);
void termui_clear(struct termui *self);
void termui_home(struct termui *self);
void termui_show_cursor(struct termui *self, bool show);
void termui_move_cursor(struct termui *self, unsigned x, unsigned y);
void termui_setattr(struct termui *self, termui_attr_t attr);
void termui_resetattr(struct termui *self);
void termui_draw_float(struct termui *self, unsigned width, float value);
void termui_draw_int(struct termui *self, unsigned width, int value);
void termui_draw_hex(struct termui *self, unsigned width, int value, int len);
void termui_draw_text(struct termui *self, unsigned width, const char *text);
void termui_draw_label(struct termui *self, unsigned width, const char *str);
void termui_draw_frame(struct termui *self, unsigned x, unsigned y, unsigned w, unsigned h, const char *title);
void termui_draw_ind_float(struct termui *self, unsigned x, unsigned y, unsigned width, const char *name, float value);
void termui_draw_ind_int(struct termui *self, unsigned x, unsigned y, unsigned width, const char *name, int value);

