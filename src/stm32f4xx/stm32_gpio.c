<<<<<<< HEAD
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
* FILE ............... src/stm32f4xx/stm32_gpio.c
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
#include <string.h>
#include "driver.h"
#include "list.h"
#include <errno.h>
#include <libfdt/libfdt.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_syscfg.h>
#include <string.h>
//#include "stm32_gpio.h"

#include "gpio.h"

struct stm32_gpio_pin {
	GPIO_TypeDef *gpio;
	uint16_t pin;
};

struct stm32_gpio {
	struct gpio_device dev;
	struct stm32_gpio_pin *pins;
	uint8_t npins;
};

static LIST_HEAD(ext0_list);
static LIST_HEAD(ext1_list);
static LIST_HEAD(ext2_list);
static LIST_HEAD(ext3_list);
static LIST_HEAD(ext4_list);
static LIST_HEAD(ext9_5_list);
static LIST_HEAD(ext15_10_list);

/*
static int _stm32_gpio_set(gpio_pin_t pin, bool value){
  struct stm32_gpio_pin *self = container_of(pin, struct stm32_gpio_pin, ops);
  if(value){
    GPIO_SetBits(self->port, self->pin);
  } else {
    GPIO_ResetBits(self->port, self->pin);
  }
    return 0;
}

static bool _stm32_gpio_get(gpio_pin_t pin){
  struct stm32_gpio_pin *self = container_of(pin, struct stm32_gpio_pin, ops);
  return !!(GPIO_ReadInputData(self->port) & self->pin);
}

int _stm32_gpio_register_irq(gpio_pin_t pin, irq_trigger_mode_t mode, struct irq
*irq, irq_result_t (*handler)(struct irq *self, int *woken)){ struct stm32_gpio_pin
*self = container_of(pin, struct stm32_gpio_pin, ops); uint8_t port_source = 0;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  if(self->port == GPIOA){
    port_source = EXTI_PortSourceGPIOA;
  } else if(self->port == GPIOB){
    port_source = EXTI_PortSourceGPIOB;
  } else if(self->port == GPIOC){
    port_source = EXTI_PortSourceGPIOC;
  } else if(self->port == GPIOD){
    port_source = EXTI_PortSourceGPIOD;
  } else if(self->port == GPIOE){
    port_source = EXTI_PortSourceGPIOE;
  } else if(self->port == GPIOF){
    port_source = EXTI_PortSourceGPIOF;
  } else if(self->port == GPIOG){
    port_source = EXTI_PortSourceGPIOG;
  } else {
    return -EINVAL;
  };

  uint8_t pin_idx = 0xff;
  for(uint8_t c = 0; c < 16; c++){
    if(self->pin & (1 << c)){
      SYSCFG_EXTILineConfig(port_source, c);
      pin_idx = c;
      break;
    }
  }

  if(pin_idx == 0xff) return -EINVAL; // pin not specified

  EXTI_InitTypeDef exti;
  EXTI_StructInit(&exti);
  //exti.EXTI_Line = EXTI_Line6;
  exti.EXTI_Line = self->pin;
  exti.EXTI_Mode = EXTI_Mode_Interrupt;
  //exti.EXTI_Trigger = EXTI_Trigger_Rising;
  switch(mode){
    case IRQ_TRIGGER_RISING:
      exti.EXTI_Trigger = EXTI_Trigger_Rising;
      break;
    case IRQ_TRIGGER_FALLING:
      exti.EXTI_Trigger = EXTI_Trigger_Falling;
      break;
    case IRQ_TRIGGER_RISING_FALLING:
      exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
      break;
    default: return -EINVAL;
  }

  exti.EXTI_LineCmd = ENABLE;
  EXTI_Init(&exti);

  irq->handler = handler;

  NVIC_InitTypeDef nvic;
  nvic.NVIC_IRQChannelPreemptionPriority = 1;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;

  // add irq to the appropriate list
  struct {
    uint8_t irqn;
    struct list_head *list;
  } conf[] = {
    { .irqn = EXTI0_IRQn, .list = &ext0_list },
    { .irqn = EXTI1_IRQn, .list = &ext1_list },
    { .irqn = EXTI2_IRQn, .list = &ext2_list },
    { .irqn = EXTI3_IRQn, .list = &ext3_list },
    { .irqn = EXTI4_IRQn, .list = &ext4_list },
    { .irqn = EXTI9_5_IRQn, .list = &ext9_5_list },
    { .irqn = EXTI15_10_IRQn, .list = &ext15_10_list }
  };
  if(pin_idx <= 4){
    nvic.NVIC_IRQChannel = conf[pin_idx].irqn;
    list_add(&irq->list, conf[pin_idx].list);
  } else if(pin_idx >= 5 && pin_idx <= 9){
    nvic.NVIC_IRQChannel = conf[5].irqn;
    list_add(&irq->list, conf[5].list);
  } else if(pin_idx >= 10 && pin_idx <= 15){
    nvic.NVIC_IRQChannel = conf[6].irqn;
    list_add(&irq->list, conf[6].list);
  } else {
    return -EINVAL;
  }

  NVIC_Init(&nvic);

  return 0;
}

static const struct gpio_pin_ops gpio_pin_ops = {
  .set = &_stm32_gpio_set,
  .get = &_stm32_gpio_get,
  .register_irq = &_stm32_gpio_register_irq
};

gpio_pin_t stm32_gpio_pin_get_interface(struct stm32_gpio_pin *self){
  self->ops = &gpio_pin_ops;
  return &self->ops;
}

gpio_pin_t stm32_gpio_pin_init(struct stm32_gpio_pin *self, GPIO_TypeDef *port,
uint16_t pin){ memset(self, 0, sizeof(*self)); self->port = port; self->pin = pin;
  return stm32_gpio_pin_get_interface(self);
}
*/

static inline __attribute__((always_inline)) void
_handle_irq(uint32_t lines, struct list_head *list) {
	struct irq *self;
	int wake = 0;
	list_for_each_entry(self, list, list) {
		if(self->handler) {
			if(self->handler(self, &wake) == IRQ_HANDLED) {
				// TODO: this is not currently entirely correct because we also want to
				// support shared irqs. Need to store line number in irq struct!
				EXTI_ClearITPendingBit(lines);
				break;
			}
		}
	}
	// portEND_SWITCHING_ISR(wake);
}

void EXTI0_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line0) == RESET)
		return;
	_handle_irq(EXTI_Line0, &ext0_list);
}

void EXTI1_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line1) == RESET)
		return;
	_handle_irq(EXTI_Line1, &ext1_list);
}

void EXTI2_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line2) == RESET)
		return;
	_handle_irq(EXTI_Line2, &ext2_list);
}

void EXTI3_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line3) == RESET)
		return;
	_handle_irq(EXTI_Line3, &ext3_list);
}

void EXTI4_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line4) == RESET)
		return;
	_handle_irq(EXTI_Line4, &ext4_list);
}

void EXTI9_5_IRQHandler(void) {
	_handle_irq(EXTI_Line5 | EXTI_Line6 | EXTI_Line7 | EXTI_Line8 | EXTI_Line9,
	            &ext9_5_list);
}

void EXTI15_10_IRQHandler(void) {
	_handle_irq(EXTI_Line10 | EXTI_Line11 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14 |
	                EXTI_Line15,
	            &ext15_10_list);
}

static int _stm32_gpio_write_pin(gpio_device_t dev, uint32_t pin, bool value) {
	struct stm32_gpio *self = container_of(dev, struct stm32_gpio, dev.ops);
	if(pin >= self->npins)
		return -EINVAL;
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
	*value = (bool)GPIO_ReadInputDataBit(self->pins[pin].gpio, self->pins[pin].pin);
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

	if(len == 0 || !val) {
		printk("gpio: no pinctrl parameter\n");
		return -1;
	}

	int pin_count = (uint8_t)(len / 4 / 3);

	struct stm32_gpio *self = kzmalloc(sizeof(struct stm32_gpio));
	self->pins = kzmalloc(sizeof(struct stm32_gpio_pin) * (unsigned)pin_count);

	if(!self || !self->pins) {
		printk("gpio: nomem\n");
		return -ENOMEM;
	}

	gpio_device_init(&self->dev, fdt, fdt_node, &_gpio_ops);
	self->npins = (uint8_t)pin_count;

	for(int c = 0; c < pin_count; c++) {
		const fdt32_t *base = val + (3 * c);
		GPIO_TypeDef *GPIOx = (GPIO_TypeDef *)fdt32_to_cpu(*(base));
		uint16_t pin = (uint16_t)fdt32_to_cpu(*(base + 1));
		uint32_t opts = (uint32_t)fdt32_to_cpu(*(base + 2));

		self->pins[c].gpio = GPIOx;
		self->pins[c].pin = pin;

		GPIO_InitTypeDef gpio;
		GPIO_StructInit(&gpio);
		gpio.GPIO_Pin = pin;
		gpio.GPIO_Mode = (opts >> 4) & 0x3;
		gpio.GPIO_OType = (opts >> 6) & 0x1;
		gpio.GPIO_Speed = (opts >> 7) & 0x3;
		gpio.GPIO_PuPd = (opts >> 9) & 0x3;
		GPIO_Init(GPIOx, &gpio);

		uint16_t idx = (uint16_t)ffs(pin);
		if(gpio.GPIO_Mode == GPIO_Mode_AF && idx != 0) {
			GPIO_PinAFConfig(GPIOx, (uint16_t)(idx - 1), (opts & 0xf));
		} else if(gpio.GPIO_Mode == GPIO_Mode_OUT && defs) {
			uint16_t en = (uint16_t)fdt32_to_cpu(*(defs + c));
			GPIO_WriteBit(GPIOx, pin, en);
		}
	}

	gpio_device_register(&self->dev);
	dbg_printk("gpio %s: ok, %d pins\n", fdt_get_name(fdt, fdt_node, NULL), pin_count);
	return 0;
}

static int _stm32_gpio_probe(void *fdt, int fdt_node) {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

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

	return 0;
}

static int _stm32_gpio_remove(void *fdt, int fdt_node) {
	// TODO
	return -1;
}

DEVICE_DRIVER(stm32_gpio, "st,stm32_gpio", _stm32_gpio_probe, _stm32_gpio_remove)
