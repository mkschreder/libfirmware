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

#include "adc.h"
#include "mutex.h"
#include "atomic.h"

#include <errno.h>

struct stm32_adc {
    struct adc_device dev;
    volatile uint32_t *dma_buf;
    uint8_t n_channels;
};

static int _stm32_adc_trigger(adc_device_t adc){
    // ADC 2 & 3 are triggered as slaves by ADC1
	ADC_SoftwareStartConv(ADC1);
    return 0;
}

static int _stm32_adc_read(adc_device_t adc, unsigned int channel, int16_t *value){
    struct stm32_adc *self = container_of(adc, struct stm32_adc, dev.ops);
    if(channel >= self->n_channels) return -EINVAL;
    *value = (volatile int16_t)(self->dma_buf[channel]);
    return 0;
}

static const struct adc_device_ops _adc_ops = {
    .trigger = _stm32_adc_trigger,
    .read = _stm32_adc_read
};

void DMA2_Stream4_IRQHandler(void){
	DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_FEIF4);
	DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_DMEIF4);
	DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TEIF4);
	DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_HTIF4);
	DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4);
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

    //RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div6);
	//RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div6);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	ADC_CommonInitTypeDef acom;
	ADC_CommonStructInit(&acom);
	acom.ADC_Mode = ADC_TripleMode_RegSimult;
	acom.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	acom.ADC_Prescaler = ADC_Prescaler_Div2;

	ADC_CommonInit(&acom);

	ADC_InitTypeDef adc;
	ADC_StructInit(&adc);
	adc.ADC_Resolution = ADC_Resolution_12b;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_ExternalTrigConv = 0;
	adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	adc.ADC_ContinuousConvMode = DISABLE;
	adc.ADC_ScanConvMode = ENABLE;
	adc.ADC_NbrOfConversion = (uint8_t)(ch_count / 3);
	ADC_Init(ADC2, &adc);
	ADC_Init(ADC3, &adc);

	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_Init(ADC1, &adc);

	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	
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
	DMA_DeInit(DMA2_Stream4);

    uint32_t *dma_buf = kzmalloc(sizeof(uint32_t) * (unsigned)ch_count);

	DMA_InitTypeDef  dma;
	dma.DMA_Channel = DMA_Channel_0;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_High;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = (uint32_t)ch_count;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR;
	dma.DMA_Memory0BaseAddr = (uint32_t)dma_buf;

	DMA_Init(DMA2_Stream4, &dma);
	//DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream4, ENABLE);
/*
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = DMA2_Stream4_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
*/
    struct stm32_adc *self = kzmalloc(sizeof(struct stm32_adc));
    adc_device_init(&self->dev, fdt, fdt_node, &_adc_ops);
    self->dma_buf = dma_buf;
    self->n_channels = (uint8_t)ch_count;
    adc_device_register(&self->dev);

	ADC_Cmd(ADC3, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	printk("adc: ready %d channels\n", ch_count);

    return 0;
}

static int _stm32_adc_remove(void *fdt, int fdt_node){
    return 0;
}

DEVICE_DRIVER(stm32_adc, "st,stm32_adc", _stm32_adc_probe, _stm32_adc_remove)
