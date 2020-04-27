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
* FILE ............... src/stm32f4xx/stm32_pwm_motor.c
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
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

#include "motor.h"
#include "math.h"
#include "pwm.h"
#include "types/types.h"

#define PL_DIR_STOPPED 0
#define PL_DIR_FORWARD 1
#define PL_DIR_REVERSE 2

struct motor_hw;
struct stm32_pwm_motor {
	const struct motor_hw *hw;
	const struct motor_ops *motor_ops;
};

struct motor_pin_hw {
	GPIO_TypeDef *gpio;
	uint16_t pin;
};

struct motor_hw {
	struct motor_pin_hw pin_in1;
	struct motor_pin_hw pin_in2;
	struct motor_pin_hw pin_stby;
	uint8_t pwm_chan;
};

static const struct motor_hw _motor_defs[] = {
	// front left MOTA
	{
		.pin_in1 = { .gpio = GPIOE, .pin = GPIO_Pin_10 },
		.pin_in2 = { .gpio = GPIOE, .pin = GPIO_Pin_7 },
		.pin_stby = { .gpio = GPIOE, .pin = GPIO_Pin_9 },
		.pwm_chan = 4
	},
	// back left MOTB
	{
		.pin_in1 = { .gpio = GPIOE, .pin = GPIO_Pin_11 },
		.pin_in2 = { .gpio = GPIOE, .pin = GPIO_Pin_8 },
		.pin_stby = { .gpio = GPIOE, .pin = GPIO_Pin_9 },
		.pwm_chan = 5
	},
	// front right MOTA
	{
		.pin_in1 = { .gpio = GPIOE, .pin = GPIO_Pin_4 },
		.pin_in2 = { .gpio = GPIOE, .pin = GPIO_Pin_5 },
		.pin_stby = { .gpio = GPIOC, .pin = GPIO_Pin_13 },
		.pwm_chan = 6
	},
	// back right MOTB
	{
		.pin_in1 = { .gpio = GPIOE, .pin = GPIO_Pin_2 },
		.pin_in2 = { .gpio = GPIOE, .pin = GPIO_Pin_6 },
		.pin_stby = { .gpio = GPIOC, .pin = GPIO_Pin_13 },
		.pwm_chan = 7
	}
};

static void _init_gpio(struct stm32_pwm_motor *self, const struct motor_hw *hw){
	(void)self;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitTypeDef gpio;

	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	gpio.GPIO_Pin = hw->pin_in1.pin;
	GPIO_Init(hw->pin_in1.gpio, &gpio);

	gpio.GPIO_Pin = hw->pin_in2.pin;
	GPIO_Init(hw->pin_in2.gpio, &gpio);

	gpio.GPIO_Pin = hw->pin_stby.pin;
	GPIO_Init(hw->pin_stby.gpio, &gpio);
}

static int _motor_direction(struct stm32_pwm_motor *self, uint8_t dir){
	const struct motor_hw *hw = self->hw;
	switch(dir){
		case PL_DIR_STOPPED:
			GPIO_SetBits(hw->pin_in1.gpio, hw->pin_in1.pin);
			GPIO_SetBits(hw->pin_in2.gpio, hw->pin_in2.pin);
			GPIO_ResetBits(hw->pin_stby.gpio, hw->pin_stby.pin);
			break;
		case PL_DIR_FORWARD:
			GPIO_SetBits(hw->pin_in1.gpio, hw->pin_in1.pin);
			GPIO_ResetBits(hw->pin_in2.gpio, hw->pin_in2.pin);
			GPIO_SetBits(hw->pin_stby.gpio, hw->pin_stby.pin);
			break;
		case PL_DIR_REVERSE:
			GPIO_ResetBits(hw->pin_in1.gpio, hw->pin_in1.pin);
			GPIO_SetBits(hw->pin_in2.gpio, hw->pin_in2.pin);
			GPIO_SetBits(hw->pin_stby.gpio, hw->pin_stby.pin);
			break;
	}
	return 0;
}

static int _motor_set_speed(motor_t motor, int8_t speed){
	struct stm32_pwm_motor *self = container_of(motor, struct stm32_pwm_motor, motor_ops);

	speed = constrain_i8(speed, -100, 100);
	if(speed < 0){
		pwm_set_duty(self->hw->pwm_chan, (uint8_t)-speed);
		_motor_direction(self, PL_DIR_REVERSE);
	} else if(speed > 0){
		pwm_set_duty(self->hw->pwm_chan, (uint8_t)speed);
		_motor_direction(self, PL_DIR_FORWARD);
	} else {
		pwm_set_duty(self->hw->pwm_chan, 0);
		_motor_direction(self, PL_DIR_STOPPED);
	}
	return 0;
}

static const struct motor_ops _motor_ops = {
	.set_speed = _motor_set_speed
};

void stm32_pwm_motor_init(struct stm32_pwm_motor *self, uint8_t id){
	memset(self, 0, sizeof(*self));

	self->motor_ops = &_motor_ops;

	if(id < (sizeof(_motor_defs) / sizeof(_motor_defs[0]))){
		_init_gpio(self, &_motor_defs[id]);
		self->hw = &_motor_defs[0];
	}

	_motor_set_speed(&self->motor_ops, 0);
}
motor_t stm32_pwm_motor_get_interface(struct stm32_pwm_motor *self){
	return &self->motor_ops;
}

