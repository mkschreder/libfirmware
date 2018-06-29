#pragma once

#include <stm32f30x_gpio.h>
#include <firmware/gpio.h>

struct stm32_gpio_pin {
	GPIO_TypeDef *port;
	uint16_t pin;
	const struct gpio_pin_ops *ops;
};

gpio_pin_t stm32_gpio_pin_init(struct stm32_gpio_pin *self, GPIO_TypeDef *port, uint16_t pin);
gpio_pin_t stm32_gpio_pin_get_interface(struct stm32_gpio_pin *self);
void stm32_gpio_pin_set_input_default(struct stm32_gpio_pin *self);
void stm32_gpio_pin_set_output_default(struct stm32_gpio_pin *self);
