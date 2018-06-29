#include <stddef.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>

#include "stm32_leds.h"

struct stm32_leds_config {
	GPIO_TypeDef *gpio;
	uint16_t pin;
};

static const struct stm32_leds_config _leds[] = {
	{ .gpio = GPIOC, .pin = GPIO_Pin_13 },
	{ .gpio = GPIOB, .pin = GPIO_Pin_4 }
};

void _stm32_leds_on(led_controller_t dev, uint8_t id){
	if(id > (sizeof(_leds)/sizeof(_leds[0]))) return;
	GPIO_SetBits(_leds[id].gpio, _leds[id].pin);
}

void _stm32_leds_off(led_controller_t dev, uint8_t id){
	if(id > (sizeof(_leds)/sizeof(_leds[0]))) return;
	GPIO_ResetBits(_leds[id].gpio, _leds[id].pin);
}

void _stm32_leds_toggle(led_controller_t dev, uint8_t id){
	if(id > (sizeof(_leds)/sizeof(_leds[0]))) return;
	uint16_t d = GPIO_ReadOutputData(_leds[id].gpio);
	GPIO_WriteBit(_leds[id].gpio, _leds[id].pin, !(d & _leds[id].pin));
}

const struct led_controller_ops _led_ops = {
	.on = _stm32_leds_on,
	.off = _stm32_leds_off,
	.toggle = _stm32_leds_toggle,
};

void stm32_leds_init(struct stm32_leds *self){
	GPIO_InitTypeDef gpio;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);

	for(size_t c = 0; c < (sizeof(_leds)/sizeof(_leds[0])); c++){
				gpio.GPIO_Pin = _leds[c].pin;
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(_leds[c].gpio, &gpio);
	}

	self->ops = &_led_ops;
}

led_controller_t stm32_leds_get_interface(struct stm32_leds *self){
	return &self->ops;
}

