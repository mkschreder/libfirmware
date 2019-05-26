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
* FILE ............... src/stm32f4xx/stm32_adc.c
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

#include <libfdt/libfdt.h>

#include "driver.h"
#include "thread.h"
#include "queue.h"
#include "sem.h"
#include "work.h"
#include "console.h"

#include "adc.h"
#include "mutex.h"
#include "atomic.h"

#include <errno.h>

struct stm32_adc {
    struct adc_device dev;
    volatile uint32_t *dma_buf;
    uint8_t n_channels;
	struct {
		atomic_t eoc;
	} cnt;
};

static struct stm32_adc *_adc1 = NULL;

void _adc_dma_configure(struct stm32_adc *self){
	DMA_InitTypeDef  dma;
	DMA_StructInit(&dma);

	dma.DMA_Channel = DMA_Channel_0;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_High;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = (uint32_t)self->n_channels;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR;
	dma.DMA_Memory0BaseAddr = (uint32_t)self->dma_buf;

	DMA_DeInit(DMA2_Stream4);
	DMA_Init(DMA2_Stream4, &dma);
	DMA_Cmd(DMA2_Stream4, ENABLE);
}

static int _stm32_adc_trigger(adc_device_t adc){
    // ADC 2 & 3 are triggered as slaves by ADC1
	//ADC_SoftwareStartConv(ADC1);
    return -EINVAL;
}

static int _stm32_adc_read(adc_device_t adc, unsigned int channel, uint16_t *value){
    struct stm32_adc *self = container_of(adc, struct stm32_adc, dev.ops);
    if(channel >= self->n_channels) return -EINVAL;
	if(ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR)){
		// can this even happen?
		printk(PRINT_ERROR "adc overrun\n");
		ADC_ClearFlag(ADC1, ADC_FLAG_OVR);
		_adc_dma_configure(self);
	}
    *value = (volatile uint16_t)(self->dma_buf[channel]);

    return 0;
}

static const struct adc_device_ops _adc_ops = {
    .trigger = _stm32_adc_trigger,
    .read = _stm32_adc_read
};

static int _stm32_adc_cmd(console_device_t con, void *ptr, int argc, char *argv[]){
	#define flag_str(f, name) (f)?"\033[30;47m" name "\033[0m":name
	struct stm32_adc *self = (struct stm32_adc *)ptr;
	if(argc == 2 && strcmp(argv[1], "status") == 0){
		console_printf(con, "No. conv: %d\n", self->cnt.eoc);
		console_printf(con, "Status: ");
		console_printf(con, "%s ", flag_str(ADC_GetFlagStatus(ADC1, ADC_FLAG_AWD), "AWD"));
		console_printf(con, "%s ", flag_str(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC), "EOC"));
		console_printf(con, "%s ", flag_str(ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC), "JEOC"));
		console_printf(con, "%s ", flag_str(ADC_GetFlagStatus(ADC1, ADC_FLAG_JSTRT), "JSTRT"));
		console_printf(con, "%s ", flag_str(ADC_GetFlagStatus(ADC1, ADC_FLAG_STRT), "STRT"));
		console_printf(con, "%s ", flag_str(ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR), "OVR"));
		console_printf(con, "\n");
	}
	return 0;
}

void ADC_IRQHandler(void){
	struct stm32_adc *self = _adc1;
	atomic_inc(&self->cnt.eoc);
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
}

static int _stm32_adc_probe(void *fdt, int fdt_node){
    int ch_count = 0;

	int len = 0;
    const fdt32_t *channels = (const fdt32_t*)fdt_getprop(fdt, fdt_node, "channels", &len);
	if(len != 0 && channels) {
        // each config line contains 4 fields of 32 bit each
        // <Device>, <Channel>, <Order>, <SampleTime>
		ch_count = (uint8_t)(len / 4 / 4);
    }
	uint32_t trigger = (uint32_t)fdt_get_int_or_default(fdt, fdt_node, "trigger", 0);
	int trigger_edge = fdt_get_int_or_default(fdt, fdt_node, "trigger_edge", -1);
	uint32_t prescaler = (uint32_t)fdt_get_int_or_default(fdt, fdt_node, "prescaler", ADC_Prescaler_Div2);
	console_device_t console = console_find_by_ref(fdt, fdt_node, "console");
	
	if(trigger > 0 && trigger_edge == -1){
		trigger_edge = ADC_ExternalTrigConvEdge_Rising;
	}

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	uint32_t adc_freq = clocks.PCLK2_Frequency;
	switch(prescaler){
		case ADC_Prescaler_Div2: adc_freq = adc_freq / 2; break;
		case ADC_Prescaler_Div4: adc_freq = adc_freq / 4; break;
		case ADC_Prescaler_Div6: adc_freq = adc_freq / 6; break;
		case ADC_Prescaler_Div8: adc_freq = adc_freq / 8; break;
		default: {
			printk(PRINT_ERROR "Invalid prescaler value\n");
			return -EINVAL;
		}
	}

	// TODO: make this dependent on chip type
	if(adc_freq > 36000000){
		printk(PRINT_ERROR "adc clock frequency too high (%d)\n", adc_freq);
		return -EINVAL;
	}

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	ADC_CommonInitTypeDef acom;
	ADC_CommonStructInit(&acom);
	acom.ADC_Mode = ADC_TripleMode_RegSimult;
	acom.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	acom.ADC_Prescaler = prescaler;
	ADC_CommonInit(&acom);

	ADC_InitTypeDef adc;
	ADC_StructInit(&adc);
	adc.ADC_Resolution = ADC_Resolution_12b;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_ExternalTrigConv = trigger;
	adc.ADC_ExternalTrigConvEdge = (uint32_t)trigger_edge;
	adc.ADC_ContinuousConvMode = DISABLE;
	adc.ADC_ScanConvMode = ENABLE;
	adc.ADC_NbrOfConversion = (uint8_t)(ch_count / 3);
	ADC_Init(ADC2, &adc);
	ADC_Init(ADC3, &adc);
	ADC_Init(ADC1, &adc);

	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);
	ADC_EOCOnEachRegularChannelCmd(ADC1, DISABLE);

    // load channel config
    for(int c = 0; c < ch_count; c++){
        const fdt32_t *base = channels + (4 * c);
        ADC_TypeDef *ADCx = (ADC_TypeDef*)fdt32_to_cpu(*(base));
        uint8_t chan = (uint8_t)fdt32_to_cpu(*(base + 1));
        uint8_t order = (uint8_t)fdt32_to_cpu(*(base + 2));
        uint8_t sample_time = (uint8_t)fdt32_to_cpu(*(base + 3));

        ADC_RegularChannelConfig(ADCx, chan, order, sample_time);
    }

	//initialize adc dma

	struct stm32_adc *self = kzmalloc(sizeof(struct stm32_adc));
    self->dma_buf = kzmalloc(sizeof(uint32_t) * (unsigned)ch_count);
    self->n_channels = (uint8_t)ch_count;
	_adc1 = self;

	_adc_dma_configure(self);

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = ADC_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);


	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	ADC_DMACmd(ADC1, ENABLE);

	ADC_Cmd(ADC3, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	if(console){
		console_add_command(console, self, "adc", "adc utilities", "", _stm32_adc_cmd);
	}

    adc_device_init(&self->dev, fdt, fdt_node, &_adc_ops);
    adc_device_register(&self->dev);

	printk("adc: ready %d channels, clock %dkHz\n", ch_count, adc_freq / 1000);

    return 0;
}

static int _stm32_adc_remove(void *fdt, int fdt_node){
    return 0;
}

DEVICE_DRIVER(stm32_adc, "st,stm32_adc", _stm32_adc_probe, _stm32_adc_remove)
