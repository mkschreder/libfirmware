#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "pwm.h"

static void _led_pwm_init(void){
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Clock for GPIOD */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Alternating functions for pins */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

	/* Set pins */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	/* Enable clock for TIM4 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
/*
	TIM4 is connected to APB1 bus, which has on F407 device 42MHz clock
	But, timer has internal PLL, which double this frequency for timer, up to 84MHz
	Remember: Not each timer is connected to APB1, there are also timers connected
	on APB2, which works at 84MHz by default, and internal PLL increase
	this to up to 168MHz

	Set timer prescaller
	Timer count frequency is set with

	timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)

	In our case, we want a max frequency for timer, so we set prescaller to 0
	And our timer will have tick frequency

	timer_tick_frequency = 84000000 / (0 + 1) = 84000000
*/
	TIM_BaseStruct.TIM_Prescaler = 0;
	/* Count up */
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
/*
	Set timer period when it have reset
	First you have to know max value for timer
	In our case it is 16bit = 65535
	To get your frequency for PWM, equation is simple

	PWM_frequency = timer_tick_frequency / (TIM_Period + 1)

	If you know your PWM frequency you want to have timer period set correct

	TIM_Period = timer_tick_frequency / PWM_frequency - 1

	In our case, for 10Khz PWM_frequency, set Period to

	TIM_Period = 84000000 / 10000 - 1 = 8399

	If you get TIM_Period larger than max timer value (in our case 65535),
	you have to choose larger prescaler and slow down timer tick frequency
*/
	TIM_BaseStruct.TIM_Period = 8399; /* 10kHz PWM */
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	/* Initialize TIM4 */
	TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
	/* Start count on TIM4 */
	TIM_Cmd(TIM4, ENABLE);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
}

void _motor_pwm_init(void){
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_BaseStruct.TIM_Prescaler = 0;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_BaseStruct.TIM_Period = 8399; /* 10kHz PWM */
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
	TIM_Cmd(TIM3, ENABLE);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	//TIM_ARRPreloadConfig(TIM3, ENABLE);
}

void pwm_init(void) {
	_led_pwm_init();
	_motor_pwm_init();
	for(uint8_t c = 0; c < 8; c++){
		pwm_set_duty(c, 0);
	}
}

int pwm_set_duty(uint8_t chan, uint8_t duty){
	TIM_OCInitTypeDef TIM_OCStruct;

	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OCStruct.TIM_Pulse = ((uint32_t)8399 * duty) / 100;

	switch(chan){
		case 0:
			TIM_OC1Init(TIM4, &TIM_OCStruct);
			break;
		case 1:
			TIM_OC2Init(TIM4, &TIM_OCStruct);
			break;
		case 2:
			TIM_OC3Init(TIM4, &TIM_OCStruct);
			break;
		case 3:
			TIM_OC4Init(TIM4, &TIM_OCStruct);
			break;
		case 4:
			TIM_OC1Init(TIM3, &TIM_OCStruct);
			break;
		case 5:
			TIM_OC2Init(TIM3, &TIM_OCStruct);
			break;
		case 6:
			TIM_OC3Init(TIM3, &TIM_OCStruct);
			break;
		case 7:
			TIM_OC4Init(TIM3, &TIM_OCStruct);
			break;
		default:
			return -1;
	}
	return 0;
}

