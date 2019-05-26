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
* FILE ............... src/linux/linux_leds.c
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
#include <stdio.h>

#include <libfdt/libfdt.h>

#include "leds.h"
#include "driver.h"

#define MAX_LEDS 8
struct linux_leds {
    bool leds[MAX_LEDS];
	struct leds_device dev;
};

static void print_leds(struct linux_leds *self){
    printf("LEDS: ");
    for(int c = 0; c < MAX_LEDS; c++){
        if(self->leds[c]) printf("o");
        else printf("_");
    }
    printf("\r");
    fflush(stdout);
}

void _linux_leds_on(leds_device_t leds, uint8_t id){
	struct linux_leds *self = container_of(leds, struct linux_leds, dev.ops);
	if(id >= MAX_LEDS) return;
    self->leds[id] = true;
    print_leds(self);
}

void _linux_leds_off(leds_device_t leds, uint8_t id){
	struct linux_leds *self = container_of(leds, struct linux_leds, dev.ops);
	if(id >= MAX_LEDS) return;
    self->leds[id] = false;
    print_leds(self);
}

void _linux_leds_toggle(leds_device_t leds, uint8_t id){
	struct linux_leds *self = container_of(leds, struct linux_leds, dev.ops);
	if(id >= MAX_LEDS) return;
    self->leds[id] = !self->leds[id];
    print_leds(self);
}

static const struct leds_device_ops _led_ops = {
	.on = _linux_leds_on,
	.off = _linux_leds_off,
	.toggle = _linux_leds_toggle,
};

static int _linux_leds_probe(void *fdt, int fdt_node){
	struct linux_leds *self = kzmalloc(sizeof(struct linux_leds));

	leds_device_init(&self->dev, fdt, fdt_node, &_led_ops);
	leds_device_register(&self->dev);

	return 0;
}

static int _linux_leds_remove(void *fdt, int fdt_node){
	// TODO 
    return -1;
}

DEVICE_DRIVER(linux_leds, "gnu,linux_leds", _linux_leds_probe, _linux_leds_remove)
