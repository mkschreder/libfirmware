/**
 * Author: Martin Schröder 2017
 */

#include <string.h>
#include <errno.h>

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

#include "thread.h"

#if 0
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
