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
* FILE ............... src/stm32f4xx/stm32_leds.c
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
#include "leds.h"
#include "driver.h"
#include <libfdt/libfdt.h>
#include <stm32f4xx_gpio.h>

#define MAX_LEDS 8
struct stm32_leds {
	struct {
		GPIO_TypeDef *gpio;
		uint16_t pin;
	} leds[MAX_LEDS];
	uint8_t count;
	bool inverted;
	struct leds_device dev;
};

void _stm32_leds_on(led_controller_t leds, uint8_t id){
	struct stm32_leds *self = container_of(leds, struct stm32_leds, dev.ops);
	if(id >= self->count) return;
	if(self->inverted){
		GPIO_ResetBits(self->leds[id].gpio, self->leds[id].pin);
	} else {
		GPIO_SetBits(self->leds[id].gpio, self->leds[id].pin);
	}
}

void _stm32_leds_off(led_controller_t leds, uint8_t id){
	struct stm32_leds *self = container_of(leds, struct stm32_leds, dev.ops);
	if(id >= self->count) return;
	if(self->inverted){
		GPIO_SetBits(self->leds[id].gpio, self->leds[id].pin);
	} else {
		GPIO_ResetBits(self->leds[id].gpio, self->leds[id].pin);
	}
}

void _stm32_leds_toggle(led_controller_t leds, uint8_t id){
	struct stm32_leds *self = container_of(leds, struct stm32_leds, dev.ops);
	if(id >= self->count) return;
	GPIO_ToggleBits(self->leds[id].gpio, self->leds[id].pin);
}

static const struct leds_device_ops _led_ops = {
	.on = _stm32_leds_on,
	.off = _stm32_leds_off,
	.toggle = _stm32_leds_toggle,
};

static int _stm32_leds_probe(void *fdt, int fdt_node){
	/* Find the node referenced by pins label and then parse out the pins of that node for gpio references */
	int len = 0;
	const fdt32_t *val = (const fdt32_t*)fdt_getprop(fdt, fdt_node, "pins", &len);
	if(len != 4) return -1;

	uint32_t pins_handle = (uint32_t)fdt32_to_cpu(*val);

	int node = fdt_node_offset_by_phandle(fdt, pins_handle);
	if(node < 0) return -1;

	val = (const fdt32_t*)fdt_getprop(fdt, node, "pinctrl", &len);
	if(len == 0 || !val || (len % 3) != 0) return -1;

	struct stm32_leds *self = kzmalloc(sizeof(struct stm32_leds));

	self->inverted = fdt_get_int_or_default(fdt, (int)fdt_node, "inverted", 0);
	self->count = (uint8_t)(len / 4 / 3);
	if(self->count > 8) self->count = 8;

	// save our led config for quick access
	for(int c = 0; c < self->count; c++){
		const fdt32_t *base = val + (3 * c);
		self->leds[c].gpio = (GPIO_TypeDef*)fdt32_to_cpu(*(base));
		self->leds[c].pin = (uint16_t)fdt32_to_cpu(*(base + 1));
	}

	leds_device_init(&self->dev, fdt, fdt_node, &_led_ops);
	leds_device_register(&self->dev);

	return 0;
}

static int _stm32_leds_remove(void *fdt, int fdt_node){
	// TODO 
    return -1;
}

DEVICE_DRIVER(stm32_leds, "st,stm32_leds", _stm32_leds_probe, _stm32_leds_remove)
