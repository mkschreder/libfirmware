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
* FILE ............... src/stm32f4xx/stm32_tim.c
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
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_adc.h>
#include <string.h>
#include <stdio.h>

#include <libfdt/libfdt.h>

#include "driver.h"
#include "thread.h"
#include "queue.h"
#include "sem.h"
#include "work.h"
#include "analog.h"
#include "encoder.h"

//#include "timer.h"
#include "mutex.h"
#include "atomic.h"
#include "events.h"

#include <errno.h>

struct stm32_tim {
    TIM_TypeDef *hw;
	struct analog_device analog;
	struct encoder_device enc_dev;
	events_device_t publisher;
	uint32_t update_event;
};

static struct stm32_tim *_timers[8] = {0};

static int _stm32_tim_init_hall(struct stm32_tim *self, uint16_t prescaler, uint32_t period){
	TIM_TimeBaseInitTypeDef tim;
	TIM_TypeDef *hw = self->hw;
    uint8_t irq_channel = 0;

    if(hw == TIM4){
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
        irq_channel = TIM4_IRQn;
    } else {
        return -1;
    }

	TIM_TimeBaseStructInit(&tim);
	tim.TIM_Prescaler = prescaler;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = period;
	tim.TIM_ClockDivision = 0;
	tim.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(hw, &tim);

	// Connects CH1-CH3 to TI1 using an XOR gate.
	// This generates edge every time hall sensor value changes
	TIM_SelectHallSensor(hw, ENABLE);

	// Set the timer input trigger to be edge detect of the TI1 signal
	TIM_SelectInputTrigger(hw, TIM_TS_TI1F_ED);

	//TIM_SelectOutputTrigger(hw, TIM_TRGOSource_Update);

	// Reset the timer on each toggle of the TI1 line (each hall sensor change)
	TIM_SelectSlaveMode(hw, TIM_SlaveMode_Reset);

	/*
	 * IC1 channel will thus be used for capturing time between two hall readings
	 */
	TIM_ICInitTypeDef ic;
	ic.TIM_Channel = TIM_Channel_1;
	ic.TIM_ICPolarity = TIM_ICPolarity_Rising; // capture both edges because we do 12 step commutation
	ic.TIM_ICSelection = TIM_ICSelection_TRC; // connect this to trigger capture?
	ic.TIM_ICPrescaler = TIM_ICPSC_DIV1; // capture every edge
	ic.TIM_ICFilter = 0xF; // noise filter
	TIM_ICInit(hw, &ic);

	/*
	 * Configure commutation delay using channel 2
	 */
	TIM_OCInitTypeDef oc;
	oc.TIM_OCMode = TIM_OCMode_PWM2; // according to manual this should be pwm2
	oc.TIM_OutputState = TIM_OutputState_Disable; // enable the output compare output
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_Pulse = 0xffff; // 1 = no delay, 65535 = full period delay
	TIM_OC2Init(hw, &oc);

	TIM_ClearFlag(hw, TIM_FLAG_CC1 | TIM_FLAG_CC2);

	/*
	 * Enable interrupts for cc1 and cc2 so that we can save the captured value and load next commutation step into tim1
	 */
	TIM_ITConfig(hw, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);
	TIM_ITConfig(hw, TIM_IT_Update, ENABLE);
	/*
	 * Select source for output trigger to be OC2 signal (after deadtime delay)
	 */
	//TIM_SelectOutputTrigger(hw, TIM_TRGOSource_OC2Ref);

	/*
	 * Configure nvic to deliver us the interrupts
	 */
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = irq_channel;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	/*
	 * Finally start the timer
	 */
	TIM_Cmd(hw, ENABLE);

    return 0;
}

void TIM4_IRQHandler(void){
    struct stm32_tim *self = _timers[3];
    if(!self) return;

	// this will be called whenever hall input is toggled and timer is reset
	if(TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET){
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
	}

	// this gets toggled after the small deadtime that is programmed after the timer is reset
	// so this gets called even at the very begining after the application starts running (and after timer wraps around)
	if(TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET){
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
	}

	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

void TIM1_UP_TIM10_IRQHandler(void){
	struct stm32_tim *self = _timers[0];
	int32_t wake = 0;
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET){
		GPIO_WriteBit(GPIOE, GPIO_Pin_0, 1);
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		events_publish_from_isr(self->publisher, self->update_event, &wake);
		GPIO_WriteBit(GPIOE, GPIO_Pin_0, 0);
    }
	thread_yield_from_isr(wake);
}

void TIM8_UP_TIM13_IRQHandler(){
	struct stm32_tim *self = _timers[7];
	int32_t wake = 0;
	if(TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET){
		GPIO_WriteBit(GPIOE, GPIO_Pin_1, 1);
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
		events_publish_from_isr(self->publisher, self->update_event, &wake);
		GPIO_WriteBit(GPIOE, GPIO_Pin_1, 0);
	}
	thread_yield_from_isr(wake);
}

static int _tim_analog_read(analog_device_t dev, unsigned int chan, float *value){
	return -EINVAL;
}

static int _tim_analog_write(analog_device_t dev, unsigned int chan, float value){
	struct stm32_tim *self = container_of(dev, struct stm32_tim, analog.ops);
	uint16_t arr = (uint16_t)((float)self->hw->ARR * value);
	// basic duty cycle limit TODO: this can be an issue
	if(!self->hw->ARR || arr < 2 || arr > (self->hw->ARR - 2)) return -1;
	switch(chan){
		case 0: TIM_SetCompare1(self->hw, arr); break;
		case 1: TIM_SetCompare2(self->hw, arr); break;
		case 2: TIM_SetCompare3(self->hw, arr); break;
		case 3: TIM_SetCompare4(self->hw, arr); break;
		default: return -EINVAL;
	}
	return 0;
}

static struct analog_device_ops _tim_analog_ops = {
	.read = _tim_analog_read,
	.write = _tim_analog_write
};

static int32_t _encoder_read(encoder_device_t dev){
	struct stm32_tim *self = container_of(dev, struct stm32_tim, enc_dev.ops);
	return (int16_t)self->hw->CNT;
}

static struct encoder_device_ops _encoder_ops = {
	.read = _encoder_read
};

static int _stm32_tim_probe(void *fdt, int fdt_node){
	TIM_TypeDef *TIMx = (TIM_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);
	int mode = fdt_get_int_or_default(fdt, (int)fdt_node, "mode", -1);
	int clock_div = fdt_get_int_or_default(fdt, (int)fdt_node, "clock_div", 0);
	int rep_count = fdt_get_int_or_default(fdt, (int)fdt_node, "rep_count", 0);
	uint32_t freq = (uint32_t)fdt_get_int_or_default(fdt, (int)fdt_node, "freq", 1);
	uint16_t master_mode = (uint16_t)fdt_get_int_or_default(fdt, (int)fdt_node, "master_mode", 0);
	uint16_t slave_mode = (uint16_t)fdt_get_int_or_default(fdt, (int)fdt_node, "slave_mode", 0);
	int output_trigger = fdt_get_int_or_default(fdt, (int)fdt_node, "output_trigger", -1);
	int input_trigger = fdt_get_int_or_default(fdt, (int)fdt_node, "input_trigger", -1);
	int enable = fdt_get_int_or_default(fdt, (int)fdt_node, "enable", 1);
	int events = fdt_subnode_offset(fdt, fdt_node, "events");

	events_device_t publisher = NULL;
	uint32_t update_event = 0;

	if(events > 0){
		publisher = events_find_by_ref(fdt, events, "publisher");
		if(!publisher){
			printk(PRINT_ERROR "tim: events node specified but no publisher\n");
			return -EINVAL;
		}
		update_event = (uint32_t)fdt_get_int_or_default(fdt, events, "update", 0);
	}

    int idx = -1;
    if(TIMx == TIM1) idx = 0;
    else if(TIMx == TIM2) idx = 1;
    else if(TIMx == TIM3) idx = 2;
    else if(TIMx == TIM4) idx = 3;
    else if(TIMx == TIM5) idx = 4;
    else if(TIMx == TIM6) idx = 5;
    else if(TIMx == TIM7) idx = 6;
    else if(TIMx == TIM8) idx = 7;
    else return -1;

    if(mode == -1) {
        printk("stm32_tim: mode not defined\n");
        return -1;
    }

    if(_timers[idx]) {
        printk("tim: tim%d already initialized!\n", idx);
        return -1;
    }

	if(freq == 0){
		printk("tim: must specify nonzero frequency!\n");
		return -1;
	}

    struct stm32_tim *self = kzmalloc(sizeof(struct stm32_tim));
    self->hw = TIMx;
	self->publisher = publisher;
	self->update_event = update_event;
    _timers[idx] = self;

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	const char *name = "";
	uint32_t base = 0;
	if(TIMx == TIM1){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		name = "tim1";
		base = clocks.PCLK2_Frequency;
	} else if(TIMx == TIM2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		name = "tim2";
		base = clocks.PCLK1_Frequency;
	} else if(TIMx == TIM3){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		name = "tim3";
		base = clocks.PCLK1_Frequency;
	} else if(TIMx == TIM4){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		name = "tim4";
		base = clocks.PCLK1_Frequency;
	} else if(TIMx == TIM5){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
		name = "tim5";
		base = clocks.PCLK1_Frequency;
	} else if(TIMx == TIM6){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
		name = "tim6";
		base = clocks.PCLK1_Frequency;
	} else if(TIMx == TIM7){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
		name = "tim7";
		base = clocks.PCLK1_Frequency;
	} else if(TIMx == TIM8){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		name = "tim8";
		base = clocks.PCLK2_Frequency;
	} else {
		printk("stm32_tim: unsupported timer\n");
		return -1;
	}

	TIM_DeInit(TIMx);

	if(mode == TIM_EncoderMode_TI12 || mode == TIM_EncoderMode_TI1 || mode == TIM_EncoderMode_TI2){
		TIM_EncoderInterfaceConfig(TIMx, (uint16_t)mode,
			TIM_ICPolarity_Rising,
			TIM_ICPolarity_Rising);
		TIM_SetAutoreload(TIMx, 0xffff);

		encoder_device_init(&self->enc_dev, fdt, fdt_node, &_encoder_ops);
		encoder_device_register(&self->enc_dev);

		printk("%s: encoder mode\n", name);
	} else {
		TIM_TimeBaseInitTypeDef tim;
		TIM_TimeBaseStructInit(&tim);
		tim.TIM_CounterMode = (uint16_t)mode;

		uint32_t presc = 1;
		uint32_t period = 0;
		while((period = (base / presc / (freq * 2))) > 0xFFFFUL){
			presc++;
		}

		tim.TIM_Prescaler = (uint16_t)(presc - 1);
		tim.TIM_Period = (uint16_t)period;
		tim.TIM_ClockDivision = (uint16_t)clock_div;
		tim.TIM_RepetitionCounter = (uint8_t)rep_count;
		TIM_TimeBaseInit(TIMx, &tim);

		if(master_mode){
			TIM_SelectMasterSlaveMode(TIMx, master_mode);
			if(output_trigger >= 0){
				TIM_SelectOutputTrigger(TIMx, (uint16_t)output_trigger);
				printk("tim: selecting output trigger %04x\n", output_trigger);
			}
		}

		if(slave_mode){
			TIM_SelectSlaveMode(TIMx, slave_mode);
			if(input_trigger >= 0){
				TIM_SelectInputTrigger(TIMx, (uint16_t)input_trigger);
				printk("tim: selecting input trigger %04x\n", input_trigger);
			}
		}

		printk("%s: counter mode base %d, presc %d, reload %d\n", name, base, presc, period);
	}

	for(int c = 1; c <= 4; c++){
		char chan_name[16];
		snprintf(chan_name, sizeof(chan_name), "channel%d", c);
		int node = fdt_subnode_offset(fdt, fdt_node, chan_name);
		if(node > 0){
			int oc_mode = fdt_get_int_or_default(fdt, (int)node, "mode", 0);
			int pout_en = fdt_get_int_or_default(fdt, (int)node, "pout_en", 0);
			int nout_en = fdt_get_int_or_default(fdt, (int)node, "nout_en", 0);
			int pulse = fdt_get_int_or_default(fdt, (int)node, "pulse", 0);
			int ppol = fdt_get_int_or_default(fdt, (int)node, "ppol", 0);
			int npol = fdt_get_int_or_default(fdt, (int)node, "npol", 0);
			int pidle = fdt_get_int_or_default(fdt, (int)node, "pidle", 0);
			int nidle = fdt_get_int_or_default(fdt, (int)node, "nidle", 0);
			
			TIM_OCInitTypeDef oc;
			TIM_OCStructInit(&oc);
			oc.TIM_OCMode = (uint16_t)oc_mode;
			oc.TIM_OutputState = (pout_en)?TIM_OutputState_Enable:TIM_OutputState_Disable;
			oc.TIM_OutputNState = (nout_en)?TIM_OutputNState_Enable:TIM_OutputNState_Disable;
			// pulse is now given in duty cycle between 0 and 1000 to make it agnostic to period
			oc.TIM_Pulse = (uint16_t)((uint32_t)pulse * TIMx->ARR / 1000);
			oc.TIM_OCPolarity = (ppol)?TIM_OCPolarity_High:TIM_OCPolarity_Low;
			oc.TIM_OCNPolarity = (npol)?TIM_OCNPolarity_High:TIM_OCNPolarity_Low;
			oc.TIM_OCIdleState = (pidle)?TIM_OCIdleState_Set:TIM_OCIdleState_Reset;
			oc.TIM_OCNIdleState = (nidle)?TIM_OCNIdleState_Set:TIM_OCNIdleState_Reset;
			switch(c){
				case 1: TIM_OC1Init(TIMx, &oc); break;
				case 2: TIM_OC2Init(TIMx, &oc); break;
				case 3: TIM_OC3Init(TIMx, &oc); break;
				case 4: TIM_OC4Init(TIMx, &oc); break;
			}

			printk("%s: OC%d pulse %d\n", name, c, pulse);
		}
	}

	if(TIMx == TIM1 || TIMx == TIM8){
		TIM_BDTRInitTypeDef dt;
		dt.TIM_OSSRState = TIM_OSSRState_Enable;
		dt.TIM_OSSIState = TIM_OSSIState_Enable;
		dt.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
		dt.TIM_DeadTime = 80;
		dt.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
		dt.TIM_Break = TIM_Break_Disable;
		TIM_BDTRConfig(TIMx, &dt);

		// register timer as an anlog device for pwm
		analog_device_init(&self->analog, fdt, fdt_node, &_tim_analog_ops);
		analog_device_register(&self->analog);

		NVIC_InitTypeDef nvic;
		if(TIMx == TIM8){
			nvic.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
		} else if(TIMx == TIM1){
			nvic.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
		}
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);

		TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
	}

	if(enable){
		TIM_Cmd(TIMx, ENABLE);
	}

	TIM_CtrlPWMOutputs(TIMx, ENABLE);

    return 0;
}

static int _stm32_tim_remove(void *fdt, int fdt_node){
    return 0;
}

DEVICE_DRIVER(stm32_tim, "st,stm32_tim", _stm32_tim_probe, _stm32_tim_remove)
