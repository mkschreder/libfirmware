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
