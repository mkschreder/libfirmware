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
* FILE ............... src/stm32f10x/stm32_adc.c
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
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_adc.h>
#include <string.h>

#include "adc.h"
#include "thread.h"
#include "sem.h"

#if 0
int _stm32_adc_read(adc_device_t dev, uint32_t *value){
    ADC_SoftwareStartConvCmd(ADCx, ENABLE);
}

int _stm32_adc_probe(void *fdt, int fdt_node){
    ADC_TypeDef *ADCx = (ADC_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);
	int irq_prio = fdt_get_int_or_default(fdt, (int)fdt_node, "irq_prio", 1);
	int n_channels = fdt_get_int_or_default(fdt, (int)fdt_node, "n_channels", 1);
    int len = 0;
	const fdt32_t *val = (const fdt32_t*)fdt_getprop(fdt, node, "channels", &len);
	if(len == 0 || !val || (len % 1) != 0) {
        dbg_printk("adc: nochan!\n");
        return -1;
    }

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADCx, ENABLE);

	ADC_DeInit(ADCx);

	ADC_InitTypeDef adc;
    ADC_StructInit(&adc);

	//adc.ADC_Resolution = ADC_Resolution_12b;
	adc.ADC_Mode = ADC_Mode_Independent;
	adc.ADC_ScanConvMode = ENABLE;
	adc.ADC_ContinuousConvMode = DISABLE;
	//adc.ADC_ExternalTriggerConvEdge = ADC_ExternalTrigConvEdge_Rising;
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	//adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC3;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_NbrOfChannel = self->n_channels;
	//adc.ADC_NmbrOfConversion = 1;
	ADC_Init(ADCx, &adc);
	ADC_Cmd(ADCx, ENABLE);

	//TODO: this can be a problem if for whatever reason it never completes.
	ADC_ResetCalibration(ADCx);
	while(ADC_GetResetCalibrationStatus(ADCx));
	ADC_StartCalibration(ADCx);
	while(ADC_GetCalibrationStatus(ADCx));

    static const int items_per_row = 2;
	int n_channels = (uint8_t)(len / 4 / items_per_row);
	if(n_channels > 8) n_channels = 8;

	// save our led config for quick access
	for(int c = 0; c < n_channels; c++){
		const fdt32_t *base = val + (items_per_row * c);
		uint8_t chan = (uint8_t)fdt32_to_cpu(*(base));
		uint8_t sample_time = (uint8_t)fdt32_to_cpu(*(base + 1));
	    ADC_RegularChannelConfig(ADCx, chan, c, sample_time);
	}

	ADC_ExternalTrigConvCmd(ADCx, ENABLE);

	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);

	// voltage sensing inputs
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &gpio);

	ADC_DMACmd(ADCx, ENABLE);

	//initialize adc dma
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel1);

    DMA_InitTypeDef  dma;
    dma.DMA_M2M = DMA_M2M_Disable; // don't need memory to memory
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = sizeof(self->irq_data.adc) / sizeof(self->irq_data.adc[0]);
    dma.DMA_PeripheralBaseAddr = (long)&ADCx->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)self->irq_data.adc;

    DMA_Init(DMA1_Channel1, &dma);

    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

    DMA_Cmd(DMA1_Channel1, ENABLE);

	NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = irq_prio;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    struct stm32_adc *self = kzmalloc(sizeof(struct stm32_adc));
	if(!self) return -EIO;

    return 0;
}

int _stm32_adc_remove(void *fdt, int fdt_node){
    return 0;
}

DEVICE_DRIVER(stm32_adc, "st,stm32_adc", _stm32_adc_probe, _stm32_adc_remove);
#endif
