#include <stm32f10x_tim.h>

#include "timer.h"

int _stm32_tim_probe(void *fdt, int fdt_node) {
	TIM_TypeDef *TIMx =
	    (TIM_TypeDef *)fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);
	// int mode = fdt_get_int_or_default(fdt, (int)fdt_node, "mode", -1);
	uint16_t period =
	    (uint16_t)fdt_get_int_or_default(fdt, (int)fdt_node, "period", 1000);
	uint16_t prescaler =
	    (uint16_t)fdt_get_int_or_default(fdt, (int)fdt_node, "prescaler", 36);
	uint16_t mode = (uint16_t)fdt_get_int_or_default(fdt, (int)fdt_node, "mode",
	                                                 TIM_CounterMode_CenterAligned1);

	uint16_t enable = (uint16_t)fdt_get_int_or_default(fdt, (int)fdt_node, "enable", 1);

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	const char *name = "";
	uint32_t base = 0;
	if(TIMx == TIM1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		name = "tim1";
		base = clocks.PCLK2_Frequency;
	} else if(TIMx == TIM2) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		name = "tim2";
		base = clocks.PCLK1_Frequency;
	} else if(TIMx == TIM3) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		name = "tim3";
		base = clocks.PCLK1_Frequency;
	} else if(TIMx == TIM4) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		name = "tim4";
		base = clocks.PCLK1_Frequency;
	} else if(TIMx == TIM5) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
		name = "tim5";
		base = clocks.PCLK1_Frequency;
	} else if(TIMx == TIM6) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
		name = "tim6";
		base = clocks.PCLK1_Frequency;
	} else if(TIMx == TIM7) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
		name = "tim7";
		base = clocks.PCLK1_Frequency;
	} else if(TIMx == TIM8) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		name = "tim8";
		base = clocks.PCLK2_Frequency;
	} else {
		printk("stm32_tim: unsupported timer\n");
		return -1;
	}

	printk("%s: clock %u\n", name, base);

	TIM_TimeBaseInitTypeDef tim;
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_Prescaler = prescaler;
	tim.TIM_CounterMode = mode;
	tim.TIM_Period = period;
	tim.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIMx, &tim);

	// otherwise scan all children
	int child_node;
	fdt_for_each_subnode(child_node, fdt, fdt_node) {
		const char *oc_name = fdt_get_name(fdt, child_node, NULL);
		uint16_t oc_mode = (uint16_t)fdt_get_int_or_default(fdt, (int)child_node, "mode",
		                                                 TIM_OCMode_PWM1);
		uint16_t polarity = (uint16_t)fdt_get_int_or_default(
		    fdt, (int)child_node, "polarity", TIM_OCPolarity_High);
		uint16_t output_state = (uint16_t)fdt_get_int_or_default(
		    fdt, (int)child_node, "output_state", TIM_OutputState_Disable);
		uint16_t idle_state = (uint16_t)fdt_get_int_or_default(
		    fdt, (int)child_node, "idle_state", TIM_OCIdleState_Reset);
		uint16_t pulse =
		    (uint16_t)fdt_get_int_or_default(fdt, (int)child_node, "pulse", 0);

		TIM_OCInitTypeDef oc;
		TIM_OCStructInit(&oc);
		oc.TIM_OCMode = oc_mode;
		oc.TIM_OutputState = output_state;
		oc.TIM_Pulse = pulse;
		oc.TIM_OCPolarity = polarity;
		oc.TIM_OCIdleState = idle_state;

		if(strcmp(oc_name, "oc1") == 0) {
			TIM_OC1Init(TIMx, &oc);
			TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
		} else if(strcmp(oc_name, "oc2") == 0) {
			TIM_OC2Init(TIMx, &oc);
			TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
		} else if(strcmp(oc_name, "oc3") == 0) {
			TIM_OC3Init(TIMx, &oc);
			TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
		} else if(strcmp(oc_name, "oc4") == 0) {
			TIM_OC4Init(TIMx, &oc);
			TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
		} else {
			continue;
		}

		printk("%s: ok\n", oc_name);
	}

	if(enable){
		TIM_Cmd(TIMx, ENABLE);
		TIM_CtrlPWMOutputs(TIMx, ENABLE);
	}

	return 0;
}

int _stm32_tim_remove(void *fdt, int fdt_node) {
	return 0;
}

DEVICE_DRIVER(stm32_tim, "st,stm32_tim", _stm32_tim_probe, _stm32_tim_remove)
