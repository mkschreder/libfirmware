#include <stm32f4xx.h>
#include <misc.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_gpio.h>

#include <stdlib.h>
#include <string.h>

#include <errno.h>

#include "types.h"
#include "driver.h"
#include "serial.h"
#include "thread.h"
#include "queue.h"

#include <libfdt/libfdt.h>

struct stm32_uart {
	struct serial_device dev;
	USART_TypeDef *hw;
	struct thread_queue tx_queue;
	struct thread_queue rx_queue;
};

#define UART_NUM_DEVICES 6
static struct stm32_uart *_uart_ptr[UART_NUM_DEVICES] = {0, 0, 0, 0, 0, 0};

static int _serial_write(serial_port_t serial, const void *data, size_t size, uint32_t timeout){
	struct stm32_uart *self = container_of(serial, struct stm32_uart, dev.ops);
	if(!self) return -1;
	uint8_t *buf = (uint8_t*)data;
	int sent = 0;
	for(size_t c = 0; c < size; c++){
		if(thread_queue_send(&self->tx_queue, &buf[c], timeout) < 0){
			// on timeout we break and just return the number of bytes sent so far
			break;
		}
		// after putting the data into the queue, we enable the interrupt
		USART_ITConfig(self->hw, USART_IT_TXE, ENABLE);
		sent++;
	}
	return sent;
}

static int _serial_read(serial_port_t serial, void *data, size_t size, uint32_t timeout){
	struct stm32_uart *self = container_of(serial, struct stm32_uart, dev.ops);
	if(!self) return -1;
	char ch;
	char *buf = (char*)data;
	int pos = 0;
	// pop characters off the queue
	while(size && thread_queue_recv(&self->rx_queue, &ch, (pos == 0)?(timeout):0) == 0){
		*(buf + pos) = ch;
		pos++;
		size--;
	}
	if(pos == 0) return -ETIMEDOUT;
	return pos;
}

static inline struct stm32_uart *_get_hw(uint8_t id){
	if(id == 0 || id > UART_NUM_DEVICES) return NULL;
	return _uart_ptr[id - 1];
}

static int32_t _uart_irq(struct stm32_uart *self){
	if(!self){
		USART_ClearITPendingBit(self->hw, USART_IT_RXNE);
		USART_ClearITPendingBit(self->hw, USART_IT_TXE);
		return 0;
	}

	int32_t wake = 0;

	// we check for incoming data on this device, ack the interrupt and copy data into the queue
	if( USART_GetITStatus(self->hw, USART_IT_RXNE) ){
		USART_ClearITPendingBit(self->hw, USART_IT_RXNE);
		uint16_t t = self->hw->DR;
		thread_queue_send_from_isr(&self->rx_queue, &t, &wake);
	}

	// we check for transmission read on this device, ack the interrupt and either send next byte or turn off the interrupt
	if( USART_GetITStatus(self->hw, USART_IT_TXE) ){
		USART_ClearITPendingBit(self->hw, USART_IT_TXE);
		char ch;
        if(thread_queue_recv_from_isr(&self->tx_queue, &ch, &wake) == 0){
            self->hw->DR = ch;
            USART_ITConfig(self->hw, USART_IT_TXE, ENABLE);
        } else {
            USART_ITConfig(self->hw, USART_IT_TXE, DISABLE);
        }
	}
	return wake;
}

void USART1_IRQHandler(void){
	struct stm32_uart *hw = _get_hw(1);
	int32_t __unused wake = _uart_irq(hw);
	//portYIELD_FROM_ISR(wake);
}

void USART2_IRQHandler(void){
	struct stm32_uart *hw = _get_hw(2);
	int32_t __unused wake = _uart_irq(hw);
	//portYIELD_FROM_ISR(wake);
}

void USART3_IRQHandler(void){
	struct stm32_uart *hw = _get_hw(3);
	int32_t __unused wake = _uart_irq(hw);
	//portYIELD_FROM_ISR(wake);
}

void UART4_IRQHandler(void){
	struct stm32_uart *hw = _get_hw(4);
	int32_t __unused wake = _uart_irq(hw);
	//portYIELD_FROM_ISR(wake);
}

void UART5_IRQHandler(void){
	struct stm32_uart *hw = _get_hw(5);
	int32_t __unused wake = _uart_irq(hw);
	//portYIELD_FROM_ISR(wake);
}

static const struct serial_ops _serial_ops = {
	.read = _serial_read,
	.write = _serial_write
};

static int _stm32_uart_probe(void *fdt, int fdt_node){
	// TODO: move this so it's only done once
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

	int baud = fdt_get_int_or_default(fdt, (int)fdt_node, "baud", 9600);
	USART_TypeDef *UARTx = (USART_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);
	int irq = fdt_get_int_or_default(fdt, (int)fdt_node, "interrupt", -1);
	int irq_pre_prio = fdt_get_int_or_default(fdt, (int)fdt_node, "irq_prio", 1);
	int irq_sub_prio = fdt_get_int_or_default(fdt, (int)fdt_node, "irq_sub_prio", 0);
	int tx_queue = fdt_get_int_or_default(fdt, (int)fdt_node, "tx_queue", 64);
	int rx_queue = fdt_get_int_or_default(fdt, (int)fdt_node, "rx_queue", 64);

	if(UARTx == 0) {
		return -EINVAL;
	}

	int idx = -1;
	if(UARTx == USART1) {
		idx = 1;
	} else if(UARTx == USART2) {
		idx = 2;
	} else if(UARTx == USART3) {
		idx = 3;
	} else if(UARTx == UART4) {
		idx = 4;
	} else if(UARTx == UART5) {
		idx = 5;
	};

	if(idx == -1){
		return -EINVAL;
	}

	struct stm32_uart *self = kzmalloc(sizeof(struct stm32_uart));
	if(!self) return -1;

	serial_device_init(&self->dev, fdt_node, &_serial_ops);

    thread_queue_init(&self->tx_queue, (size_t)tx_queue, sizeof(char));
    thread_queue_init(&self->rx_queue, (size_t)rx_queue, sizeof(char));

	self->hw = UARTx;
	_uart_ptr[idx - 1] = self;

	USART_InitTypeDef conf;

	USART_StructInit(&conf);

	// uart itself
	conf.USART_BaudRate = (uint32_t)baud;
	conf.USART_WordLength = USART_WordLength_8b;
	conf.USART_StopBits = USART_StopBits_1;
	conf.USART_Parity = USART_Parity_No;
	conf.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	conf.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(UARTx, &conf);
	(void)irq;

	if(irq > 0){
		NVIC_InitTypeDef nvic;
		nvic.NVIC_IRQChannel = (uint8_t)irq;
		nvic.NVIC_IRQChannelPreemptionPriority = (uint8_t)irq_pre_prio;
		nvic.NVIC_IRQChannelSubPriority = (uint8_t)irq_sub_prio;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);

		// enable interrupts
		USART_ITConfig(UARTx, USART_IT_RXNE, ENABLE);
	}

	USART_Cmd(UARTx, ENABLE);

	if(serial_device_register(&self->dev) < 0){
		return -1;
	}

	return 0;
}

static int _stm32_uart_remove(void *fdt, int fdt_node){
	// TODO
    return -1;
}

DEVICE_DRIVER(stm32_uart, "st,stm32_uart", _stm32_uart_probe, _stm32_uart_remove)
