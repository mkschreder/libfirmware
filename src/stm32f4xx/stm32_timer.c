#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_adc.h>
#include <string.h>

#include <libfdt/libfdt.h>

#include "driver.h"
#include "thread.h"
#include "queue.h"
#include "sem.h"
#include "work.h"

//#include "timer.h"
#include "mutex.h"
#include "atomic.h"

#include <errno.h>

struct stm32_timer {
    TIM_TypeDef *hw;
};

static struct stm32_timer *_timers[8] = {0};

static int _stm32_timer_init_pwm_bldc(struct stm32_timer *self, uint32_t pwm_freq){
    TIM_TypeDef *hw = self->hw;
    if(!hw) return -EINVAL;

    uint8_t irq_chan = 0;
    if(hw == TIM1){
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
        irq_chan = TIM1_UP_TIM10_IRQn;
    } else {
        printk("tim: invalid timer for pwm!\n");
        return -EINVAL;
    }

    TIM_TimeBaseInitTypeDef tim;

	TIM_TimeBaseStructInit(&tim);
	tim.TIM_Prescaler = 0;
	tim.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	tim.TIM_Period = (uint16_t)(SystemCoreClock / pwm_freq);
	tim.TIM_ClockDivision = 0;
	//tim.TIM_RepetitionCounter = 17; // this gives about 500us update event period
	TIM_TimeBaseInit(hw, &tim);

	TIM_OCInitTypeDef oc;
	TIM_OCStructInit(&oc);
	oc.TIM_OCMode = TIM_OCMode_PWM1; // start in timing mode (disabled outputs)
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputNState_Enable;
	oc.TIM_Pulse = 200; // will be set later
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
	oc.TIM_OCIdleState = TIM_OCIdleState_Set;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(hw, &oc);
	TIM_OC2Init(hw, &oc);
	TIM_OC3Init(hw, &oc);
	TIM_OC4Init(hw, &oc);

	TIM_OC1PreloadConfig(hw, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(hw, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(hw, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(hw, TIM_OCPreload_Enable);

	/*
	 * Configure automatic output enable + deadtime
	 */
	TIM_BDTRInitTypeDef bdtr;
	TIM_BDTRStructInit(&bdtr);
	bdtr.TIM_OSSRState = TIM_OSSRState_Enable;
	bdtr.TIM_OSSIState = TIM_OSSIState_Enable;
	bdtr.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	bdtr.TIM_DeadTime = 80; // How many nanoseconds?
	bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	bdtr.TIM_Break = TIM_Break_Disable;
	TIM_BDTRConfig(hw, &bdtr);

	TIM_Cmd(hw, ENABLE);
	TIM_CtrlPWMOutputs(hw, ENABLE);

    TIM_ITConfig(hw, TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = irq_chan;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

    return 0;
}

static int _stm32_timer_init_hall(struct stm32_timer *self, uint16_t prescaler, uint32_t period){
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
    struct stm32_timer *self = _timers[3];
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
	//struct stm32_bldc *self = _timers[0];

	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

static int _stm32_timer_probe(void *fdt, int fdt_node){
	TIM_TypeDef *TIMx = (TIM_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);
	int mode = fdt_get_int_or_default(fdt, (int)fdt_node, "mode", 0);
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

    if(!mode) {
        printk("tim1: no mode supplied. Valid modes are:\n");
        printk("\t1: pwm 3phase\n");
        return -1;
    }

    if(_timers[idx]) {
        printk("tim: tim%d already initialized!\n", idx);
        return -1;
    }

    struct stm32_timer *self = kzmalloc(sizeof(struct stm32_timer));
    self->hw = TIMx;
    _timers[idx] = self;

    switch(mode){
        case 0: {
	        int pwm_freq = fdt_get_int_or_default(fdt, (int)fdt_node, "pwm_freq", 0);
            if(pwm_freq){
                _stm32_timer_init_pwm_bldc(self, (uint32_t)pwm_freq);
            }
        } break;
    }
    return 0;
}

static int _stm32_timer_remove(void *fdt, int fdt_node){
    return 0;
}

DEVICE_DRIVER(stm32_timer, "st,stm32_timer", _stm32_timer_probe, _stm32_timer_remove)
