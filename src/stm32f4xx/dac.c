#include <stm32f4xx_dac.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>

#if 0
int dac_write(struct dac *self, uint8_t chan, uint16_t value){
	(void)self;
	switch(chan){
		case 1:
			DAC_SetChannel1Data(DAC_Align_12b_R, value & 0xFFF);
			DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
			break;
		case 2:
			DAC_SetChannel2Data(DAC_Align_12b_R, value & 0xFFF);
			DAC_SoftwareTriggerCmd(DAC_Channel_2, ENABLE);
			break;
	}
	return 0;
}

int dac_init(struct dac *self, uint8_t hw_id){
	(void)self;
	(void)hw_id;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	gpio.GPIO_Mode = GPIO_Mode_AN;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio);

	DAC_InitTypeDef dac;
	DAC_StructInit(&dac);

	dac.DAC_Trigger = DAC_Trigger_None;
	dac.DAC_WaveGeneration = DAC_WaveGeneration_None;
	dac.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &dac);
	DAC_Init(DAC_Channel_2, &dac);

	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_Cmd(DAC_Channel_2, ENABLE);

	return 0;
}

#endif
