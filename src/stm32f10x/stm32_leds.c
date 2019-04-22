#include <stddef.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <misc.h>

#include <libfdt/libfdt.h>

#include "driver.h"
#include "leds.h"

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

void _stm32_leds_on(leds_device_t leds, uint8_t id){
	struct stm32_leds *self = container_of(leds, struct stm32_leds, dev.ops);
	if(id >= self->count) return;
	if(self->inverted){
		GPIO_ResetBits(self->leds[id].gpio, self->leds[id].pin);
	} else {
		GPIO_SetBits(self->leds[id].gpio, self->leds[id].pin);
	}
}

void _stm32_leds_off(leds_device_t leds, uint8_t id){
	struct stm32_leds *self = container_of(leds, struct stm32_leds, dev.ops);
	if(id >= self->count) return;
	if(self->inverted){
		GPIO_SetBits(self->leds[id].gpio, self->leds[id].pin);
	} else {
		GPIO_ResetBits(self->leds[id].gpio, self->leds[id].pin);
	}
}

void _stm32_leds_toggle(leds_device_t leds, uint8_t id){
	struct stm32_leds *self = container_of(leds, struct stm32_leds, dev.ops);
	if(id >= self->count) return;
	// Not available on 103
	//GPIO_ToggleBits(self->leds[id].gpio, self->leds[id].pin);
}

const struct leds_device_ops _led_ops = {
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
	if(!self) return -1;

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

    dbg_printk("leds: ok, %d leds\n", self->count);

	return 0;
}

static int _stm32_leds_remove(void *fdt, int fdt_node){
	// TODO 
    return 0;
}

DEVICE_DRIVER(stm32_leds, "st,stm32_leds", _stm32_leds_probe, _stm32_leds_remove)
