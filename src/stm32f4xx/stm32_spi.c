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
* FILE ............... src/stm32f4xx/stm32_spi.c
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
#include <string.h>
#include <errno.h>

#include "sem.h"
#include "thread.h"
#include "time.h"
#include "spi.h"
#include "driver.h"
#include "mutex.h"

#include <libfdt/libfdt.h>

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"

struct stm32_spi {
	SPI_TypeDef *hw;
	struct spi_device dev;
	char tx_dma[16];
	char rx_dma[16];
	bool cs_control;
	struct semaphore rx_sem;
	struct mutex lock;
};

struct stm32_spi *_devices[3] = {NULL};

// RX
void DMA2_Stream0_IRQHandler(void){
	struct stm32_spi *self = _devices[0];
	BUG_ON(!self);
    int32_t wake = 0;
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET){
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

		SPI_Cmd(SPI1, DISABLE);
		thread_sem_give_from_isr(&self->rx_sem, &wake);
	}
    thread_yield_from_isr(wake);
}

// TX
void DMA2_Stream3_IRQHandler(void){
	if(DMA_GetITStatus(DMA2_Stream3, DMA_IT_TCIF3) != RESET){
		DMA_ClearITPendingBit(DMA2_Stream3, DMA_IT_TCIF3);
	}
}

// RX
void DMA1_Stream3_IRQHandler(void){
	struct stm32_spi *self = _devices[1];
	BUG_ON(!self);
    int32_t wake = 0;
	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET){
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);

		thread_sem_give_from_isr(&self->rx_sem, &wake);
		SPI_Cmd(SPI2, DISABLE);
	}
    thread_yield_from_isr(wake);
}

// TX
void DMA1_Stream4_IRQHandler(void){
	if(DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4) != RESET){
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
	}
}

static void _dma_set_data(DMA_Stream_TypeDef *dma, uint32_t addr, size_t size){
	DMA_Cmd(dma, DISABLE);
    while (DMA_GetCmdStatus(dma) != DISABLE);
	dma->M0AR = addr;
	dma->NDTR = size;
	//DMA_ClearITPendingBit(dma, DMA_IT_TCIF0 | DMA_IT_HTIF0 | DMA_IT_FEIF0);
	//DMA_ClearITPendingBit(dma, DMA_IT_TCIF3 | DMA_IT_HTIF3 | DMA_IT_FEIF3);
	//DMA_ClearITPendingBit(dma, DMA_IT_TCIF4 | DMA_IT_HTIF4 | DMA_IT_FEIF4);
	if(dma == DMA1_Stream0) DMA_ClearFlag(dma, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0 | DMA_FLAG_TEIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_FEIF0);
	if(dma == DMA1_Stream3) DMA_ClearFlag(dma, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3);
	if(dma == DMA2_Stream3) DMA_ClearFlag(dma, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3);
	if(dma == DMA2_Stream4) DMA_ClearFlag(dma, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_DMEIF4 | DMA_FLAG_FEIF4);
	DMA_Cmd(dma, ENABLE);
	while (DMA_GetCmdStatus(dma) != ENABLE);
}

static int _stm32_spi_transfer(spi_device_t dev, gpio_device_t gpio, uint32_t cs_pin, const void *tx_data, void *rx_data, size_t size, msec_t timeout){
	struct stm32_spi *self = container_of(dev, struct stm32_spi, dev.ops);
	if(!self->hw) return -1;

	#if 0
	if(self->hw == SPI1){
		thread_mutex_lock(&self->lock);

		if(self->cs_control) gpio_reset(gpio, cs_pin);

		_dma_set_data(DMA2_Stream0, (uint32_t)rx_data, size);
		_dma_set_data(DMA2_Stream3, (uint32_t)tx_data, size);

		SPI_Cmd(self->hw, ENABLE);

		if(thread_sem_take_wait(&self->rx_sem, timeout) != 0){
			SPI_Cmd(self->hw, DISABLE);
			gpio_set(gpio, cs_pin);
			thread_mutex_unlock(&self->lock);
			return -ETIMEDOUT;
		}
		SPI_Cmd(self->hw, DISABLE);
		if(self->cs_control) gpio_set(gpio, cs_pin);
		thread_mutex_unlock(&self->lock);
	} else {
		#endif
		// Currently DMA does not seem to work. Need to debug it.
		//_dma_set_data(DMA1_Stream3, (uint32_t)rx_data, size);
		//_dma_set_data(DMA1_Stream4, (uint32_t)tx_data, size);

		// this is quick and dirty just to get it to work
		thread_mutex_lock(&self->lock);
		if(self->cs_control) gpio_reset(gpio, cs_pin);
		const uint8_t *tx = (const uint8_t*)tx_data;
		uint8_t *rx = (uint8_t*)rx_data;
		for(size_t c = 0; c < size; c++){
			while(!SPI_I2S_GetFlagStatus(self->hw, SPI_I2S_FLAG_TXE));
			SPI_I2S_SendData(self->hw, *tx++);
			while(!SPI_I2S_GetFlagStatus(self->hw, SPI_I2S_FLAG_RXNE));
			uint8_t dr = (uint8_t)SPI_I2S_ReceiveData(self->hw);
			*rx++ = dr;
			while(SPI_I2S_GetFlagStatus(self->hw, SPI_I2S_FLAG_BSY));
		}
		if(self->cs_control) gpio_set(gpio, cs_pin);
		thread_mutex_unlock(&self->lock);
	//}

	return 0;
}

static const struct spi_device_ops _ops = {
	.transfer = _stm32_spi_transfer
};

static int _stm32_spi_probe(void *fdt, int fdt_node){
	SPI_TypeDef *SPIx = (SPI_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);
	int cs_control = fdt_get_int_or_default(fdt, fdt_node, "cs_control", 1);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	struct stm32_spi *self = kzmalloc(sizeof(struct stm32_spi));
	self->cs_control = (bool)cs_control;

	SPI_I2S_DeInit(SPIx);

	SPI_InitTypeDef spi;
	SPI_StructInit(&spi);

	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_CPOL = SPI_CPOL_High;
	spi.SPI_CPHA = SPI_CPHA_2Edge;
	spi.SPI_NSS = SPI_NSS_Soft;
	spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	spi.SPI_FirstBit = SPI_FirstBit_MSB;

	SPI_Init(SPIx, &spi);
	SPI_CalculateCRC(SPIx, DISABLE);

	if(SPIx == SPI1){
		#if 0
		DMA_InitTypeDef dma;
		DMA_StructInit(&dma);

		// RX
		dma.DMA_Channel = DMA_Channel_3;
		dma.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
		dma.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_BufferSize = sizeof(self->rx_dma);
		dma.DMA_Memory0BaseAddr = (uint32_t)self->rx_dma;
		dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
		dma.DMA_Priority = DMA_Priority_High;
		dma.DMA_Mode = DMA_Mode_Normal;
		DMA_Init(DMA2_Stream0, &dma);

		// TX
		dma.DMA_BufferSize = sizeof(self->tx_dma);
		dma.DMA_Memory0BaseAddr = (uint32_t)self->tx_dma;
		dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		dma.DMA_Priority = DMA_Priority_High;
		dma.DMA_Mode = DMA_Mode_Normal;
		DMA_Init(DMA2_Stream3, &dma);

		DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
		DMA_ITConfig(DMA2_Stream3, DMA_IT_TC, ENABLE);

		NVIC_InitTypeDef nvic;
		nvic.NVIC_IRQChannel = DMA2_Stream0_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
		nvic.NVIC_IRQChannel = DMA2_Stream3_IRQn;
		NVIC_Init(&nvic);

		SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx, ENABLE);
		SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Tx, ENABLE);

		#endif
		printk("spi1: ready\n");
		_devices[0] = self;
	} else if(SPIx == SPI2){
		#if 0
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

		DMA_InitTypeDef dma;
		DMA_StructInit(&dma);

		// RX
		dma.DMA_Channel = DMA_Channel_0;
		dma.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
		dma.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_BufferSize = sizeof(self->rx_dma);
		dma.DMA_Memory0BaseAddr = (uint32_t)self->rx_dma;
		dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
		dma.DMA_Priority = DMA_Priority_High;
		dma.DMA_Mode = DMA_Mode_Normal;
		DMA_Init(DMA1_Stream3, &dma);

		// TX
		dma.DMA_BufferSize = sizeof(self->tx_dma);
		dma.DMA_Memory0BaseAddr = (uint32_t)self->tx_dma;
		dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		dma.DMA_Priority = DMA_Priority_High;
		dma.DMA_Mode = DMA_Mode_Normal;
		DMA_Init(DMA1_Stream4, &dma);

		DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
		DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);

		NVIC_InitTypeDef nvic;
		nvic.NVIC_IRQChannel = DMA1_Stream3_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
		nvic.NVIC_IRQChannel = DMA1_Stream4_IRQn;
		NVIC_Init(&nvic);

		SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx, ENABLE);
		SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Tx, ENABLE);

		#endif
		_devices[1] = self;

		printk("spi2: ready\n");
	} else if(SPIx == SPI3){
		_devices[2] = self;

		printk("spi3: ready\n");
	} else {
		return -EINVAL;
	}

	thread_sem_init(&self->rx_sem);
	self->hw = SPIx;
	thread_mutex_init(&self->lock);

	SPI_Cmd(self->hw, ENABLE);

	spi_device_init(&self->dev, fdt, fdt_node, &_ops);
	spi_device_register(&self->dev);

	return 0;
}

static int _stm32_spi_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(stm32_spi, "st,stm32_spi", _stm32_spi_probe, _stm32_spi_remove)
