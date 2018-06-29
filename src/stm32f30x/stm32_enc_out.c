#include <stm32f30x_tim.h>
#include "stm32_enc_out.h"

void stm32_enc_out_init(void){
	TIM_TypeDef *hw = TIM2;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	TIM_TimeBaseInitTypeDef tim;
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_Prescaler = (uint16_t)(SystemCoreClock / 1000000);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 1000;
	tim.TIM_ClockDivision = 0;
	tim.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(hw, &tim);

/*
	TIM_OCInitTypeDef oc;
	TIM_OCStructInit(&oc);
	oc.TIM_OCMode = TIM_OCMode_Toggle; // start in timing mode (disabled outputs)
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_Pulse = 500;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OCIdleState = TIM_OCIdleState_Set;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(hw, &oc);
	TIM_OC1PreloadConfig(hw, TIM_OCPreload_Enable);
	oc.TIM_Pulse = 200;
	TIM_OC2Init(hw, &oc);
	TIM_OC2PreloadConfig(hw, TIM_OCPreload_Enable);
*/
	TIM_Cmd(hw, ENABLE);
	TIM_CtrlPWMOutputs(hw, ENABLE);

	TIM_ITConfig(hw, TIM_IT_Update, ENABLE);

	GPIO_InitTypeDef gpio;

	// TIM1 CH1 CH2 and CH3
	gpio.GPIO_Pin = GPIO_Pin_0;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);
	gpio.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &gpio);

	/*
	 * Configure nvic to deliver us the interrupts
	 */
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}

void TIM2_IRQHandler(void){
	static const uint8_t enca[4] = {0, 0, 1, 1};
	static const uint8_t encb[4] = {0, 1, 1, 0};
	static uint8_t pos = 0;

	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		if(enca[pos])
			GPIO_SetBits(GPIOA, GPIO_Pin_0);
		else
			GPIO_ResetBits(GPIOA, GPIO_Pin_0);
		if(encb[pos])
			GPIO_SetBits(GPIOA, GPIO_Pin_1);
		else
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);
		pos = (pos + 1) & 0x3;
	}
}

void stm32_enc_out_set_counts_per_sec(uint32_t counts){
	if(!counts)
		TIM2->ARR = 0;
	else
		TIM2->ARR = (uint16_t)(1000000 / counts / 2);
}
