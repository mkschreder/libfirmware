/** :ms-top-comment
<<<<<<< HEAD
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
* FILE ............... src/stm32f10x/stm32_spi.c
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
/**
 * Author: Martin Schröder 2017
 */
#include <errno.h>
#include <string.h>

#include "driver.h"
#include "gpio.h"
#include "sem.h"
#include "spi.h"
#include "thread.h"
#include "time.h"

#include <libfdt/libfdt.h>

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

struct stm32_spi {
	struct spi_device dev;
	SPI_TypeDef *hw;
	/*
	  char tx_dma[16];
	  char rx_dma[16];
	  struct semaphore rx_sem;
	*/
};

struct stm32_spi *_devices[2];
#if 0
// RX
void DMA2_Stream0_IRQHandler(void){
	struct stm32_spi *self = _devices[0];
    if(!self) return;

	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET){
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

		SPI_Cmd(SPI1, DISABLE);
		thread_sem_give_from_isr(&self->rx_sem);
	}
}

// TX
void DMA2_Stream3_IRQHandler(void){
	if(DMA_GetITStatus(DMA2_Stream3, DMA_IT_TCIF3) != RESET){
		DMA_ClearITPendingBit(DMA2_Stream3, DMA_IT_TCIF3);
	}
}

// RX
void DMA1_Stream3_IRQHandler(void){
	struct stm32_spi *self = &_devices[1];
    if(!self) return;

	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET){
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);

		thread_sem_give_from_isr(&self->rx_sem);
		SPI_Cmd(SPI2, DISABLE);
	}
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
#endif

#define SPI_TRANSFER_TIMEOUT 20000

int _stm32_spi_transfer(spi_device_t dev, gpio_device_t gpio, uint32_t cs_pin,
                        const void *tx_data, void *rx_data, size_t size,
                        msec_t timeout) {
	struct stm32_spi *self = container_of(dev, struct stm32_spi, dev.ops);
	if(!self->hw)
		return -1;

	// SPI_Cmd(self->hw, ENABLE);

	// gpio_reset(self->gpio, cs);
	gpio_reset(gpio, cs_pin);
	for(size_t c = 0; c < size; c++) {
		SPI_I2S_SendData(self->hw, ((const uint8_t *)tx_data)[c]);
		int tout = SPI_TRANSFER_TIMEOUT;
		while(SPI_I2S_GetFlagStatus(self->hw, SPI_I2S_FLAG_TXE) == RESET && --tout)
			asm volatile("nop");
		if(tout == 0) {
			dbg_printk("spi: etxe\n");
			goto timedout;
		}
		tout = SPI_TRANSFER_TIMEOUT;
		while(SPI_I2S_GetFlagStatus(self->hw, SPI_I2S_FLAG_RXNE) == RESET && --tout)
			asm volatile("nop");
		if(tout == 0) {
			dbg_printk("spi: erxne\n");
			goto timedout;
		}

		((uint8_t *)rx_data)[c] = (uint8_t)SPI_I2S_ReceiveData(self->hw);

		tout = SPI_TRANSFER_TIMEOUT;
		while(SPI_I2S_GetFlagStatus(self->hw, SPI_I2S_FLAG_BSY) == SET && --tout)
			asm volatile("nop");
		if(tout == 0) {
			dbg_printk("spi: ebsy\n");
			goto timedout;
		}
	}
	gpio_set(gpio, cs_pin);
	// gpio_set(self->gpio, cs);
	// SPI_Cmd(self->hw, DISABLE);
	return 0;
timedout:
	gpio_set(gpio, cs_pin);
	// gpio_set(self->gpio, cs);
	// SPI_Cmd(self->hw, DISABLE);
	return -ETIMEDOUT;
#if 0
	if(self->hw == SPI1){
		_dma_set_data(DMA2_Stream0, (uint32_t)rx_data, size);
		_dma_set_data(DMA2_Stream3, (uint32_t)tx_data, size);

		SPI_Cmd(self->hw, ENABLE);

		if(thread_sem_take_wait(&self->rx_sem, timeout) != 0){
			SPI_Cmd(self->hw, DISABLE);
			return -ETIMEDOUT;
		}
	} else if(self->hw == SPI2){
		// Currently DMA does not seem to work. Need to debug it.
		//_dma_set_data(DMA1_Stream3, (uint32_t)rx_data, size);
		//_dma_set_data(DMA1_Stream4, (uint32_t)tx_data, size);

		// this is quick and dirty just to get it to work
		SPI_Cmd(self->hw, ENABLE);
		for(size_t c = 0; c < size; c++){
			uint8_t *tx = (uint8_t*)tx_data;
			uint8_t *rx = (uint8_t*)rx_data;
			SPI_I2S_SendData(self->hw, tx[c]);
			while(!SPI_I2S_GetFlagStatus(self->hw, SPI_I2S_FLAG_TXE));
			while(!SPI_I2S_GetFlagStatus(self->hw, SPI_I2S_FLAG_RXNE));
			while(SPI_I2S_GetFlagStatus(self->hw, SPI_I2S_FLAG_BSY));
			rx[c] = (uint8_t)self->hw->DR;
		}
	}


	SPI_Cmd(self->hw, DISABLE);
#endif
	return 0;
}

const struct spi_device_ops _ops = {.transfer = _stm32_spi_transfer};

static int _stm32_spi_probe(void *fdt, int fdt_node) {
	SPI_TypeDef *SPIx =
	    (SPI_TypeDef *)fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);

	int idx = 0;
	if(SPIx == SPI1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
		idx = 0;
	} else if(SPIx == SPI2) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		idx = 1;
	} else {
		return -1;
	}

	struct stm32_spi *self = kzmalloc(sizeof(struct stm32_spi));
	self->hw = SPIx;
	_devices[idx] = self;

	SPI_Cmd(SPIx, DISABLE);

	SPI_InitTypeDef spi;
	SPI_StructInit(&spi);

	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_CPOL = SPI_CPOL_High;
	spi.SPI_CPHA = SPI_CPHA_2Edge;
	spi.SPI_NSS = SPI_NSS_Soft;
	spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	spi.SPI_FirstBit = SPI_FirstBit_MSB;

	SPI_Init(SPIx, &spi);
	SPI_CalculateCRC(SPIx, DISABLE);

	SPI_Cmd(SPIx, ENABLE);

	spi_device_init(&self->dev, fdt, fdt_node, &_ops);
	spi_device_register(&self->dev);

	dbg_printk("spi%d: ok\n", idx + 1);
#if 0

	if(SPIx == SPI1){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

		SPI_Cmd(SPIx, DISABLE);

		SPI_InitTypeDef spi;
		SPI_StructInit(&spi);

		spi.SPI_Mode = SPI_Mode_Master;
		spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		spi.SPI_DataSize = SPI_DataSize_8b;
		spi.SPI_CPOL = SPI_CPOL_High;
		spi.SPI_CPHA = SPI_CPHA_2Edge;
		spi.SPI_NSS = SPI_NSS_Soft;
		spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
		spi.SPI_FirstBit = SPI_FirstBit_MSB;

		SPI_Init(SPIx, &spi);
		SPI_CalculateCRC(SPIx, DISABLE);


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

		self = &_devices[0];
	} else if(SPIx == SPI2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

		SPI_Cmd(SPIx, DISABLE);

		SPI_InitTypeDef spi;
		SPI_StructInit(&spi);

		spi.SPI_Mode = SPI_Mode_Master;
		spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		spi.SPI_DataSize = SPI_DataSize_8b;
		spi.SPI_CPOL = SPI_CPOL_High;
		spi.SPI_CPHA = SPI_CPHA_2Edge;
		spi.SPI_NSS = SPI_NSS_Soft;
		spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
		spi.SPI_FirstBit = SPI_FirstBit_MSB;

		SPI_Init(SPIx, &spi);
		SPI_CalculateCRC(SPIx, DISABLE);

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

		self = &_devices[1];
	} else {
		return -EINVAL;
	}

	spi_device_init(&self->dev, fdt_node, &_ops);
	thread_sem_init(&self->rx_sem);
	self->hw = SPIx;

	spi_device_register(&self->dev);
#endif
	return 0;
}

static int _stm32_spi_remove(void *fdt, int fdt_node) {
	return 0;
}

DEVICE_DRIVER(stm32_spi, "st,stm32_spi", _stm32_spi_probe, _stm32_spi_remove)
#if 0
#include <errno.h>
#include <string.h>

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

#include "thread.h"

int _stm32_spi_probe(void *fdt, int node){
    SPI_InitTypeDef spi;
    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    SPI_Init(SPIx, &SPI_InitStructure);
}

int _stm32_spi_remove(void *fdt, int node){

}

DEVICE_DRIVER(stm32_spi, "st,stm32_spi", _stm32_spi_probe, _stm32_spi_remove)

static struct stm32_spi *_spi2 = 0;

void SPI2_IRQHandler(void){
	int32_t wake = 0;
	if(SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET){
		SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE);
		if(_spi2){
			//uint8_t data = (uint8_t)SPI2->DR;
			//thread_queue_send_from_isr(&_spi2->rx_queue, &data, &wake);
		}
	}
	thread_yield_from_isr(wake);
}

void EXTI15_10_IRQHandler(void){
	int32_t wake = 0;
	if(EXTI_GetITStatus(EXTI_Line12) != RESET){
		EXTI_ClearITPendingBit(EXTI_Line12);
		if(_spi2){
			//char *wr_addr = self->rx_dma + (uint32_t)(SPI_DMA_RX_SIZE - (uint32_t)DMA1_Channel4->CNDTR);
			// reset dma (must disbale first before setting values)
			//DMA_ClearFlag(DMA1_FLAG_GL4);
			//DMA_Cmd(DMA1_Channel4, DISABLE);
/*
			struct spi_message *msg = 0;
			thread_queue_recv_from_isr(&_spi2->free_queue, &msg, &wake);

			if(msg){
				for(uint32_t c = 0; c < rx_cnt; c++){
					thread_queue_send_from_isr(&_spi2->rx_queue, &_spi2->rx_dma[c], &wake);
				}
			}
*/
			//DMA1_Channel4->CNDTR = buf_size;
			
			// reset tx dma
			DMA_ClearFlag(DMA1_FLAG_GL5);
			DMA_Cmd(DMA1_Channel5, DISABLE);
			DMA1_Channel5->CNDTR = SPI_DMA_TX_SIZE;
			DMA_Cmd(DMA1_Channel5, ENABLE);

			//DMA_Cmd(DMA1_Channel4, ENABLE);

			// make sure first byte of next transfer is null
			SPI2->DR = 0;
		}
	}
	thread_yield_from_isr(wake);
}

static void _spi_init_slave(struct stm32_spi *self){
	GPIO_InitTypeDef gpio;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	SPI_I2S_DeInit(SPI2);
	SPI_InitTypeDef spi;
	SPI_StructInit(&spi);
	spi.SPI_Mode = SPI_Mode_Slave;
	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_CPOL = SPI_CPOL_Low;
	spi.SPI_CPHA = SPI_CPHA_1Edge;
	spi.SPI_NSS = SPI_NSS_Hard;
	spi.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI2, &spi);
	SPI_CalculateCRC(SPI2, DISABLE);

	//SPI_TIModeCmd(SPI2, DISABLE);
	//SPI_NSSPulseModeCmd(SPI2, DISABLE);

	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP; //GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_14;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &gpio);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel4);
	DMA_DeInit(DMA1_Channel5);

	DMA_InitTypeDef dma;
	DMA_StructInit(&dma);
	dma.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
	dma.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_M2M = DMA_M2M_Disable;

	dma.DMA_BufferSize = sizeof(self->rx_dma);
	dma.DMA_MemoryBaseAddr = (uint32_t)self->rx_dma;
	dma.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma.DMA_Priority = DMA_Priority_High;
	dma.DMA_Mode = DMA_Mode_Circular;
	DMA_Init(DMA1_Channel4, &dma);

	dma.DMA_BufferSize = sizeof(self->tx_dma);
	dma.DMA_MemoryBaseAddr = (uint32_t)self->tx_dma;
	dma.DMA_DIR = DMA_DIR_PeripheralDST;
	dma.DMA_Priority = DMA_Priority_Low;
	dma.DMA_Mode = DMA_Mode_Circular;
	DMA_Init(DMA1_Channel5, &dma);

	DMA_Cmd(DMA1_Channel4, ENABLE);
	DMA_Cmd(DMA1_Channel5, ENABLE);

	NVIC_InitTypeDef nvic = { 0 };
	nvic.NVIC_IRQChannel = SPI2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);

	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
	SPI_Cmd(SPI2, ENABLE);

	// configure NSS interrupt so we can reset dma
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);

	EXTI_InitTypeDef exti;
	exti.EXTI_Line = EXTI_Line12;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Rising;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	nvic.NVIC_IRQChannel = EXTI15_10_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}

int stm32_spi_init(struct stm32_spi *self, spi_mode_t mode, uint32_t speed){
	memset(self, 0, sizeof(*self));
	self->rd_addr = self->rx_dma;

	switch(mode){
		case SPI_MODE_MASTER: break;
		case SPI_MODE_SLAVE: {
			_spi_init_slave(self);
			_spi2 = self;
		} break;
	}

	return 0;
}

int stm32_spi_read(struct stm32_spi *self, void *buf, size_t size, uint32_t timeout_ms){
	size_t cnt = 0;
	char *wr_addr = self->rx_dma + (uint32_t)(SPI_DMA_RX_SIZE - (uint32_t)DMA1_Channel4->CNDTR);
	char *dst = (char*)buf;
	// TODO: this is a very stupid way to implement this kind of thing. Argh...
	/*
	while(timeout_ms && self->rd_addr == wr_addr){
		thread_sleep_ms(1);
		timeout_ms--;
	}
	*/
	while((self->rd_addr != wr_addr) && (cnt < size)){
		*dst = *self->rd_addr;
		cnt++; dst++; self->rd_addr++;
		if(self->rd_addr == (self->rx_dma + SPI_DMA_RX_SIZE)) self->rd_addr = self->rx_dma;
	}
	if(cnt == 0) return -EAGAIN;
	return (int)cnt;
}
#endif
