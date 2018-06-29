#pragma once
#include <stdint.h>

#include <firmware/leds.h>

struct stm32_leds {
	const struct led_controller_ops *ops;
};

void stm32_leds_init(struct stm32_leds *self);
led_controller_t stm32_leds_get_interface(struct stm32_leds *self);
