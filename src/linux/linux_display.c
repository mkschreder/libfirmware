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
* FILE ............... src/linux/linux_display.c
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
#include <errno.h>
#include <unistd.h>

#include "driver.h"
#include "gpio.h"
#include "display.h"
#include "mutex.h"

#include <libfdt/libfdt.h>

#include <SDL2/SDL.h>

struct linux_display {
	struct display_device dev;
    SDL_Window *window;
    SDL_Renderer *renderer;
    int width, height, bpp, pixel_size;
	int fb_size;
	uint8_t *framebuffer;
	struct mutex lock;
};

void _linux_display_task(void *data){
    struct linux_display *self = (struct linux_display *)data;
    if (SDL_Init(SDL_INIT_VIDEO) == -1) {
        fprintf(stderr, "SDL error : %s\n", SDL_GetError());
        return;
    }


    SDL_Window *window = NULL;
    window = SDL_CreateWindow("Simulator",SDL_WINDOWPOS_UNDEFINED,SDL_WINDOWPOS_UNDEFINED, (int)(self->width * self->pixel_size), (int)(self->height * self->pixel_size),SDL_WINDOW_SHOWN);
    if(!window) {
        printf("linux_display: could not create window!\n");
        return;
    }

    SDL_Renderer *renderer = NULL;
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if(!renderer){
        printf("linux_display: could not create renderer\n");
        return;
    }

	self->window = window;
    self->renderer = renderer;

    SDL_Event event;

    while(1){
       while (SDL_PollEvent(&event)){
            if (event.type == SDL_QUIT){
                printf("LOG : Closing simulator\n");
				exit(1);
            }
        }

		SDL_SetRenderDrawColor(self->renderer, 0x0, 0x0, 0x0, 128);
		SDL_Rect rect;
		rect.x = 0;
		rect.y = 0;
		rect.w = self->width;
		rect.h = self->height;
		SDL_RenderFillRect(self->renderer, &rect);

		thread_mutex_lock(&self->lock);
		for(int x = 0; x < self->width; x++){
			for(int y = 0; y < self->height; y++){
                SDL_Rect pixel;
				pixel.x = (int)(x * self->pixel_size);
				pixel.y = (int)(y * self->pixel_size);
				pixel.w = self->pixel_size;
				pixel.h = self->pixel_size;

                int base = y * (self->width * self->bpp) + x * self->bpp;
                int byte = base / 8;
                int bit = base & 0x07;
                bool r = self->framebuffer[byte] & (1 << bit);
                base++;
                byte = base / 8;
                bit = base & 0x07;
                bool g = self->framebuffer[byte] & (1 << bit);

				SDL_SetRenderDrawColor(self->renderer, (r)?0xff:0, (g)?0xff:0, 0x0, 128);

				SDL_RenderFillRect(self->renderer, &pixel);
			}
        }
		thread_mutex_unlock(&self->lock);

		SDL_RenderPresent(self->renderer);

		usleep(2000);
	}
}

int _linux_display_write_pixel(display_device_t dev, int x, int y, color_t color){
	struct linux_display *self = container_of(dev, struct linux_display, dev.ops);

	if(x < 0 || x >= self->width || y < 0 || y >= self->height) return -1;

    for(int c = 0; c < 2; c++){
        int base = y * (self->width * self->bpp) + x * self->bpp + c;
        int byte = base / 8;
        int bit = base & 0x7;
		thread_mutex_lock(&self->lock);
        if(color & (color_t)(1 << c)){
            self->framebuffer[byte] = (uint8_t)(self->framebuffer[byte] | (1 << bit));
        } else {
            self->framebuffer[byte] = (uint8_t)(self->framebuffer[byte] & ~(1 << bit));
        }
		thread_mutex_unlock(&self->lock);
    }
    return 0;
}


static const struct display_device_ops _display_ops = {
	.write_pixel = _linux_display_write_pixel
};

int _linux_display_probe(void *fdt, int fdt_node){
    int width = fdt_get_int_or_default(fdt, fdt_node, "width", 64);
	int height = fdt_get_int_or_default(fdt, fdt_node, "height", 8);
	int bpp = fdt_get_int_or_default(fdt, fdt_node, "bpp", 3);
	int pixel_size = fdt_get_int_or_default(fdt, fdt_node, "pixel_size", 4);

    struct linux_display *self = kzmalloc(sizeof(struct linux_display));
	display_device_init(&self->dev, fdt_node, &_display_ops);

    self->bpp = bpp;
    self->width = width;
    self->height = height;
	self->pixel_size = pixel_size;
	self->fb_size = (width * height * bpp) / 8;
    self->framebuffer = kzmalloc((size_t)(self->fb_size));
	thread_mutex_init(&self->lock);

	display_device_register(&self->dev);

	thread_create(
		  _linux_display_task,
		  "display",
		  300,
		  self,
		  2,
		  NULL);

    (void)self;
    printf("linux_display: ok\n");
    return 0;
}

int _linux_display_remove(void *fdt, int fdt_node){
    return 0;
}

DEVICE_DRIVER(linux_display, "gnu,linux_display", _linux_display_probe, _linux_display_remove)
