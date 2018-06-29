/*
 * Author: Martin Schröder 2017
 */

#pragma once

#include <stdint.h>
#include <thread/queue.h>

#define SPI_DMA_RX_SIZE 1024
#define SPI_DMA_TX_SIZE 64
#define SPI_MESSAGE_BUFFER_SIZE 4

struct stm32_spi {
	//struct thread_queue rx_queue;
	char tx_dma[SPI_DMA_TX_SIZE];
	char rx_dma[SPI_DMA_RX_SIZE];
	char *rd_addr;
};

typedef enum {
	SPI_MODE_MASTER = 1,
	SPI_MODE_SLAVE
} spi_mode_t;

int stm32_spi_init(struct stm32_spi *self, spi_mode_t mode, uint32_t speed);
int stm32_spi_read(struct stm32_spi *self, void *buf, size_t size, uint32_t timeout_ms);
