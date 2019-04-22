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
	termui_draw_float(self, width, (float)value);
}

void termui_draw_text(struct termui *self, unsigned width, const char *str){
	size_t len = strlen(str);
	size_t half = (width - len) / 2;
	for(size_t c = 0; c < half; c++) console_printf(self->console, " ");
	console_printf(self->console, "%s", str);
	for(size_t c = 0; c < width - (half + len); c++) console_printf(self->console, " ");
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

