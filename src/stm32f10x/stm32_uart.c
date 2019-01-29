#include <stm32f10x.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>
#include <misc.h>

#include <stdlib.h>
#include <string.h>

#include <errno.h>

#include <libfdt/libfdt.h>

#include "thread.h"
#include "mutex.h"
#include "queue.h"
#include "serial.h"
#include "types.h"
#include "driver.h"


struct stm32_uart {
	struct serial_device dev;
	USART_TypeDef *hw;
	struct thread_queue tx_queue;
	struct thread_queue rx_queue;
    struct mutex wr_lock, rd_lock;
    int crlf;
};


#define UART_NUM_DEVICES 4
static struct stm32_uart *_uart_ptr[UART_NUM_DEVICES] = {0, 0, 0, 0};

static int _serial_write(serial_port_t serial, const void *data, size_t size, uint32_t timeout){
	struct stm32_uart *self = container_of(serial, struct stm32_uart, dev.ops);
	if(!self || !self->hw) return -1;
    thread_mutex_lock(&self->wr_lock);
	uint8_t *buf = (uint8_t*)data;
	int sent = 0;
	for(size_t c = 0; c < size; c++){
        if(self->crlf && buf[c] == '\n') {
            char ch = '\r';
            if(thread_queue_send(&self->tx_queue, &ch, timeout) < 0){
                break;
            }
        }
		if(thread_queue_send(&self->tx_queue, &buf[c], timeout) < 0){
			// on timeout we break and just return the number of bytes sent so far
			break;
		}
		// after putting the data into the queue, we enable the interrupt
		USART_ITConfig(self->hw, USART_IT_TXE, ENABLE);
		sent++;
	}
    thread_mutex_unlock(&self->wr_lock);
	return sent;
}

static int _serial_read(serial_port_t serial, void *data, size_t size, uint32_t timeout){
	struct stm32_uart *self = container_of(serial, struct stm32_uart, dev.ops);
	if(!self) return -1;
    thread_mutex_lock(&self->rd_lock);
	char ch;
	char *buf = (char*)data;
	int pos = 0;
	// pop characters off the queue
	while(size && thread_queue_recv(&self->rx_queue, &ch, (pos == 0)?(timeout):0) > 0){
		*(buf + pos) = ch;
		pos++;
		size--;
	}
	if(pos == 0) {
        thread_mutex_unlock(&self->rd_lock);
        return -ETIMEDOUT;
    }
    thread_mutex_unlock(&self->rd_lock);
	return pos;
}

static inline struct stm32_uart *_get_hw(uint8_t id){
	return _uart_ptr[id - 1];
}

static int32_t _uart_irq(struct stm32_uart *hw){
	if(hw == NULL) return 0;

	int32_t wake = 0;
	// we check for incoming data on this device, ack the interrupt and copy data into the queue
	if( USART_GetITStatus(hw->hw, USART_IT_RXNE) ){
		char t = (char)hw->hw->DR;
		thread_queue_send_from_isr(&hw->rx_queue, &t, &wake);
		USART_ClearITPendingBit(hw->hw, USART_IT_RXNE);
	}

	// we check for transmission read on this device, ack the interrupt and either send next byte or turn off the interrupt
	if( USART_GetITStatus(hw->hw, USART_IT_TXE) ){
		USART_ClearITPendingBit(hw->hw, USART_IT_TXE);
		char ch;
		if(hw) {
			if(thread_queue_recv_from_isr(&hw->tx_queue, &ch, &wake) > 0){
				hw->hw->DR = ch;
			} else {
				USART_ITConfig(hw->hw, USART_IT_TXE, DISABLE);
			}
		}
	}
	return wake;
}

void USART1_IRQHandler(void);
void USART1_IRQHandler(void){
	struct stm32_uart *hw = _get_hw(1);
	int32_t wake = _uart_irq(hw);
	thread_yield_from_isr(wake);
}

void USART2_IRQHandler(void);
void USART2_IRQHandler(void){
	struct stm32_uart *hw = _get_hw(2);
	int32_t wake = _uart_irq(hw);
	thread_yield_from_isr(wake);
}

void USART3_IRQHandler(void);
void USART3_IRQHandler(void){
	struct stm32_uart *hw = _get_hw(3);
	int32_t wake = _uart_irq(hw);
	thread_yield_from_isr(wake);
}

static const struct serial_ops _serial_ops = {
	.read = _serial_read,
	.write = _serial_write
};

static int _stm32_uart_probe(void *fdt, int fdt_node){
	int baud = fdt_get_int_or_default(fdt, (int)fdt_node, "baud", 9600);
	USART_TypeDef *UARTx = (USART_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);
	int irq = fdt_get_int_or_default(fdt, (int)fdt_node, "interrupt", -1);
	int irq_pre_prio = fdt_get_int_or_default(fdt, (int)fdt_node, "irq_prio", 1);
	int irq_sub_prio = fdt_get_int_or_default(fdt, (int)fdt_node, "irq_sub_prio", 0);
	int tx_queue = fdt_get_int_or_default(fdt, (int)fdt_node, "tx_queue", 64);
	int rx_queue = fdt_get_int_or_default(fdt, (int)fdt_node, "rx_queue", 64);
	int def_port = fdt_get_int_or_default(fdt, (int)fdt_node, "printk_port", 0);
	int crlf = fdt_get_int_or_default(fdt, (int)fdt_node, "insert-cr-before-lf", 1);

	if(UARTx == 0) {
		return -EINVAL;
	}

	int idx = -1;
	if(UARTx == USART1) {
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		idx = 1;
	} else if(UARTx == USART2) {
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		idx = 2;
	} else if(UARTx == USART3) {
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		idx = 3;
	} else if(UARTx == UART4) {
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		idx = 4;
	} else if(UARTx == UART5) {
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		idx = 5;
	};

	if(idx == -1){
		return -EINVAL;
	}

	struct stm32_uart *self = kzmalloc(sizeof(struct stm32_uart));
	if(!self) return -ENOMEM;

	serial_device_init(&self->dev, fdt_node, &_serial_ops);

    thread_queue_init(&self->tx_queue, (size_t)tx_queue, sizeof(char));
    thread_queue_init(&self->rx_queue, (size_t)rx_queue, sizeof(char));
    thread_mutex_init(&self->wr_lock);
    thread_mutex_init(&self->rd_lock);

    self->crlf = crlf;
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

    if(def_port) serial_set_printk_port(&self->dev.ops);

	return 0;
}

static int _stm32_uart_remove(void *fdt, int fdt_node){
	// TODO
    return -1;
}

DEVICE_DRIVER(stm32_uart, "st,stm32_uart", _stm32_uart_probe, _stm32_uart_remove)

