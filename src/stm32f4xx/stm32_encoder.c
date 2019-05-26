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
* FILE ............... src/stm32f4xx/stm32_encoder.c
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
#include "driver.h"
#include "encoder.h"

#include <libfdt/libfdt.h>
#include <stm32f4xx_tim.h>

struct stm32_encoder {
	struct encoder_device dev;
	TIM_TypeDef *hw;
};

int32_t _stm32_encoder_read(encoder_device_t encoder){
	struct stm32_encoder *self = container_of(encoder, struct stm32_encoder, dev.ops);
	return (int32_t)(int16_t)self->hw->CNT;
}

const struct encoder_device_ops _encoder_ops = {
	.read = _stm32_encoder_read
};

static int _stm32_encoder_probe(void *fdt, int fdt_node){
	TIM_TypeDef *hw = (TIM_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "timer", 0);

	if(!hw){
		return -1;
	}

	if(hw == TIM1) RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	else if(hw == TIM3) RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	else if(hw == TIM4) RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	else if(hw == TIM5) RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	TIM_DeInit(hw);

	TIM_EncoderInterfaceConfig(hw, TIM_EncoderMode_TI12,
		TIM_ICPolarity_Rising,
		TIM_ICPolarity_Rising);
	TIM_SetAutoreload(hw, 0xffff);
	TIM_Cmd(hw, ENABLE);

	// set filter to 0x6
	//hw->CCMR1 |= 6 << 12 | 6 << 4;
	//hw->CCMR2 |= 6 << 4;


	struct stm32_encoder *enc = kzmalloc(sizeof(struct stm32_encoder));
	enc->hw = hw;

	encoder_device_init(&enc->dev, fdt, fdt_node, &_encoder_ops);
	encoder_device_register(&enc->dev);

	return 0;
}

static int _stm32_encoder_remove(void *fdt, int fdt_node){
	// TODO
	return -1;
}

DEVICE_DRIVER(stm32_enc, "st,stm32_enc", _stm32_encoder_probe, _stm32_encoder_remove)
