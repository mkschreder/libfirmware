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
* FILE ............... src/stm32f10x/stm32_gpio.c
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
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

#include <errno.h>

#include <libfdt/libfdt.h>

#include "driver.h"
#include "gpio.h"

#define gpio_debug no_printk

struct stm32_gpio_pin {
	GPIO_TypeDef *gpio;
	uint16_t pin;
};

struct stm32_gpio {
	struct gpio_device dev;
	struct stm32_gpio_pin *pins;
	uint8_t npins;
};

static int _stm32_gpio_write_pin(gpio_device_t dev, uint32_t pin, bool value) {
	struct stm32_gpio *self = container_of(dev, struct stm32_gpio, dev.ops);
	if(pin >= self->npins)
		return -EINVAL;
	gpio_debug("gpio pin %08x: %04x = %d\n", self->pins[pin].gpio, self->pins[pin].pin,
	           value);
	if(value) {
		GPIO_SetBits(self->pins[pin].gpio, self->pins[pin].pin);
	} else {
		GPIO_ResetBits(self->pins[pin].gpio, self->pins[pin].pin);
	}
	return 0;
}

static int _stm32_gpio_read_pin(gpio_device_t dev, uint32_t pin, bool *value) {
	struct stm32_gpio *self = container_of(dev, struct stm32_gpio, dev.ops);
	if(pin >= self->npins)
		return -EINVAL;
	*value = !!GPIO_ReadInputDataBit(self->pins[pin].gpio, self->pins[pin].pin);
	return 0;
}

static const struct gpio_device_ops _gpio_ops = {.read_pin = _stm32_gpio_read_pin,
                                                 .write_pin = _stm32_gpio_write_pin};

static int _stm32_gpio_setup_subnode(void *fdt, int fdt_node) {
	int len = 0, defs_len = 0;
	const fdt32_t *val = (const fdt32_t *)fdt_getprop(fdt, fdt_node, "pinctrl", &len);
	const fdt32_t *defs =
	    (const fdt32_t *)fdt_getprop(fdt, fdt_node, "defaults", &defs_len);

	if(defs && (len != (defs_len * 3))) {
		printk("gpio: defaults not supplied for all pins\n");
		return -1;
	}

	if(len == 0 || !val)
		return -1;

	uint8_t pin_count = (uint8_t)(len / 4 / 3);

	struct stm32_gpio *self = kzmalloc(sizeof(struct stm32_gpio));
	self->pins = kzmalloc(sizeof(struct stm32_gpio_pin) * pin_count);
	gpio_device_init(&self->dev, fdt, fdt_node, &_gpio_ops);
	self->npins = pin_count;

	for(uint8_t c = 0; c < pin_count; c++) {
		const fdt32_t *base = val + (3 * c);
		GPIO_TypeDef *GPIOx = (GPIO_TypeDef *)fdt32_to_cpu(*(base));
		uint16_t pin = (uint16_t)fdt32_to_cpu(*(base + 1));
		uint32_t opts = (uint32_t)fdt32_to_cpu(*(base + 2));

		self->pins[c].gpio = GPIOx;
		self->pins[c].pin = pin;

		GPIO_InitTypeDef gpio;
		GPIO_StructInit(&gpio);
		gpio.GPIO_Pin = pin;
		gpio.GPIO_Mode = (opts)&0xff;
		gpio.GPIO_Speed = (opts >> 8) & 0x3;
		if(!gpio.GPIO_Speed)
			gpio.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOx, &gpio);

		if((gpio.GPIO_Mode == GPIO_Mode_Out_PP || gpio.GPIO_Mode == GPIO_Mode_Out_OD) && defs) {
			uint16_t en = (uint16_t)fdt32_to_cpu(*(defs + c));
			GPIO_WriteBit(GPIOx, pin, en);
		}
		/*
		uint16_t idx = (uint16_t)ffs(pin);
		if(gpio.GPIO_Mode == GPIO_Mode_AF && idx != 0){
		    GPIO_PinAFConfig(GPIOx, (uint16_t)(idx - 1), (opts & 0xf));
		}
		*/
	}

	gpio_device_register(&self->dev);
	dbg_printk("gpio %s: ok, %d pins\n", fdt_get_name(fdt, fdt_node, NULL), pin_count);
	return 0;
}

static int _stm32_gpio_probe(void *fdt, int fdt_node) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	// check if we directly have a pinmux here
	int len = 0;
	const fdt32_t *val = (const fdt32_t *)fdt_getprop(fdt, fdt_node, "pinctrl", &len);
	if(val && len > 0) {
		if(_stm32_gpio_setup_subnode(fdt, fdt_node) < 0) {
			return -1;
		}
	}

	// otherwise scan all children
	int node;
	fdt_for_each_subnode(node, fdt, fdt_node) {
		if(_stm32_gpio_setup_subnode(fdt, node) < 0) {
			return -1;
		}
	}

	val = (const fdt32_t *)fdt_getprop(fdt, fdt_node, "remap", &len);
	for(uint8_t c = 0; c < (len / 4); c++) {
		const fdt32_t *base = val + c;
		uint32_t remap = (uint32_t)fdt32_to_cpu(*(base));
		GPIO_PinRemapConfig(remap, ENABLE);
		printk("gpio: remap %08x\n", remap);
	}

	return 0;
}

static int _stm32_gpio_remove(void *fdt, int fdt_node) {
	// TODO
	return 0;
}

DEVICE_DRIVER(stm32_gpio, "st,stm32_gpio", _stm32_gpio_probe, _stm32_gpio_remove)
