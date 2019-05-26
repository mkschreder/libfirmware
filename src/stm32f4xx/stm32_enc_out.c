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
* FILE ............... src/stm32f4xx/stm32_enc_out.c
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
/**
Encoder output driver.

This driver uses timer to set rate and dma channel to write waveform to the GPIO pins.

Example:

enc_out: enc_out {
    compatible = "st,stm32_enc_out";
    timer = <TIM4>;
    gpio = <GPIOC>;
    pin_a = <6>;
    pin_b = <7>;
};

*/
#include "driver.h"
#include "serial.h"
#include "console.h"
#include <libfdt/libfdt.h>

#include <stm32f4xx_tim.h>
#include <stm32f4xx_dma.h>

#include <stdlib.h>
#include <stdio.h>

struct stm32_enc_out {
	struct serial_device dev;
	TIM_TypeDef *hw;
	DMA_Stream_TypeDef *dma;
	uint32_t pattern_pos[4];
	uint32_t pattern_neg[4];
};

#define PIN_SET(pin) (uint32_t)(1 << (pin))
#define PIN_RESET(pin) (uint32_t)(1 << (pin + 16))

static void _stm32_enc_out_write(struct stm32_enc_out *self, int32_t counts){
	if(counts == 0) {
		self->hw->ARR = 0;
	} else if(counts < 0) {
		self->hw->ARR = (uint16_t)(1000000 / -counts / 2);
		DMA_Cmd(self->dma, DISABLE);
		self->dma->M0AR = (uint32_t)self->pattern_neg;
		DMA_Cmd(self->dma, ENABLE);
	} else {
		self->hw->ARR = (uint16_t)(1000000 / counts / 2);
		DMA_Cmd(self->dma, DISABLE);
		self->dma->M0AR = (uint32_t)self->pattern_pos;
		DMA_Cmd(self->dma, ENABLE);
	}
}

static int _serial_write(serial_port_t serial, const void *data, size_t size, uint32_t timeout) {
	struct stm32_enc_out *self = container_of(serial, struct stm32_enc_out, dev.ops);
	if(size != 4) return -1;

	int32_t counts = 0;
	memcpy(&counts, data, sizeof(counts));

	_stm32_enc_out_write(self, counts);

	return 4;
}

static int _serial_read(serial_port_t serial, void *data, size_t size, uint32_t timeout) {
	return -1;
}

static const struct serial_device_ops _serial_ops = {
	.read = _serial_read,
	.write = _serial_write
};

static int _stm32_encoder_out_cmd(console_device_t con, void *ptr, int argc, char **argv){
	struct stm32_enc_out *self = (struct stm32_enc_out*)ptr;
	if(argc == 2){
		int val = 0;
		sscanf(argv[1], "%d", &val);
		_stm32_enc_out_write(self, (int32_t)val);
	} else {

	}
	return 0;
}

static int _stm32_encoder_out_probe(void *fdt, int fdt_node) {
	TIM_TypeDef *hw = (TIM_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "timer", 0);
	GPIO_TypeDef *gpio = (GPIO_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "gpio", 0);
	int pin_a = fdt_get_int_or_default(fdt, (int)fdt_node, "pin_a", 0);
	int pin_b = fdt_get_int_or_default(fdt, (int)fdt_node, "pin_b", 1);
	console_device_t console = console_find_by_ref(fdt, fdt_node, "console");

	if(!hw){
		printk(PRINT_ERROR "stm32_enc_out: timer missing\n");
		return -EINVAL;
	}
	if(!gpio){
		printk(PRINT_ERROR "stm32_enc_out: gpio missing\n");
		return -EINVAL;
	}

	uint32_t channel = 0;
	DMA_Stream_TypeDef *stream = NULL;

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	uint32_t timebase = 0;

	if(hw == TIM1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		channel = DMA_Channel_6;
		stream = DMA2_Stream5;
		timebase = clocks.PCLK2_Frequency;
		printk("enc_out: TIM1, DMA2, Stream5, Channel6\n");
	} else if(hw == TIM2) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		channel = DMA_Channel_3;
		stream = DMA1_Stream1;
		timebase = clocks.PCLK1_Frequency;
		printk("enc_out: TIM2, DMA1 Stream7 Channel3\n");
	} else if(hw == TIM3) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		channel = DMA_Channel_5;
		stream = DMA1_Stream2;
		timebase = clocks.PCLK1_Frequency;
		printk("enc_out: TIM3, DMA1 Stream2 Channel5\n");
	} else if(hw == TIM4) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		channel = DMA_Channel_2;
		stream = DMA1_Stream6;
		timebase = clocks.PCLK1_Frequency;
		printk("enc_out: TIM4, DMA1 Stream6 Channel2\n");
	} else if(hw == TIM5) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		channel = DMA_Channel_6;
		stream = DMA1_Stream6;
		timebase = clocks.PCLK1_Frequency;
		printk("enc_out: TIM5, DMA1 Stream6 Channel6\n");
	} else {
		printk("enc_out: provided timer is not supported\n");
		return -1;
	}

	struct stm32_enc_out *self = kzmalloc(sizeof(struct stm32_enc_out));
	self->hw = hw;
	self->dma = stream;

	// fill out the patterns for set/reset register
	self->pattern_pos[0] = PIN_RESET(pin_a) | PIN_RESET(pin_b);
	self->pattern_pos[1] = PIN_RESET(pin_b) | PIN_SET(pin_a);
	self->pattern_pos[2] = PIN_SET(pin_a) | PIN_SET(pin_b);
	self->pattern_pos[3] = PIN_RESET(pin_a) | PIN_SET(pin_b);

	self->pattern_neg[0] = PIN_RESET(pin_a) | PIN_SET(pin_b);
	self->pattern_neg[1] = PIN_SET(pin_a) | PIN_SET(pin_b);
	self->pattern_neg[2] = PIN_RESET(pin_b) | PIN_SET(pin_a);
	self->pattern_neg[3] = PIN_RESET(pin_a) | PIN_RESET(pin_b);

	TIM_Cmd(hw, DISABLE);
	TIM_DeInit(hw);

	TIM_TimeBaseInitTypeDef tim;
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_Prescaler = (uint16_t)(timebase / 1000000);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 200;
	tim.TIM_ClockDivision = 0;
	tim.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(hw, &tim);

	TIM_DMACmd(hw, TIM_DMA_Update, ENABLE);

	DMA_Cmd(stream, DISABLE);
	while(DMA_GetCmdStatus(stream) == ENABLE);

	DMA_DeInit(stream);

	DMA_InitTypeDef dma;
	DMA_StructInit(&dma);
	dma.DMA_Channel = channel;
	dma.DMA_Memory0BaseAddr = (uint32_t)self->pattern_pos;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&GPIOC->BSRRL;
	dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dma.DMA_BufferSize = 4;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_High;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;

	DMA_Init(stream, &dma);

	DMA_Cmd(stream, ENABLE);

	TIM_Cmd(hw, ENABLE);

	if(console){
		console_add_command(console, self, fdt_get_name(fdt, fdt_node, NULL), "Encoder output control", "", _stm32_encoder_out_cmd);
	}

	serial_device_init(&self->dev, fdt, fdt_node, &_serial_ops);
	serial_device_register(&self->dev);

	printk(PRINT_SUCCESS "stm32_enc_out: ready\n");

	return 0;
}

static int _stm32_encoder_out_remove(void *fdt, int fdt_node) {
	return 0;
}

DEVICE_DRIVER(stm32_enc_out, "st,stm32_enc_out", _stm32_encoder_out_probe, _stm32_encoder_out_remove)
