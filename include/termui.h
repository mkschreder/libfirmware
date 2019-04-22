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
void termui_draw_text(struct termui *self, unsigned width, const char *text);
void termui_draw_ind_float(struct termui *self, unsigned x, unsigned y, unsigned width, const char *name, float value);
void termui_draw_ind_int(struct termui *self, unsigned x, unsigned y, unsigned width, const char *name, int value);

