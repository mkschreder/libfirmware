#include <stm32f30x.h>
#include <stm32f30x_usart.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>

#include <stdlib.h>
#include <string.h>

#include <errno.h>

#include <thread/thread.h>
#include <thread/queue.h>

#include "stm32_uart.h"
#include <firmware/serial.h>
#include <firmware/types.h>

struct stm32_uart_hw {
	USART_TypeDef *hw;
	struct thread_queue tx_queue;
	struct thread_queue rx_queue;
};

#define UART_NUM_DEVICES 4
static volatile struct stm32_uart *_uart_ptr[UART_NUM_DEVICES] = {0, 0, 0, 0};
static struct stm32_uart_hw _uart_hw[3];

static void _uart1_init(void){
	USART_InitTypeDef uart;
	NVIC_InitTypeDef nvic;

	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// TX
	gpio.GPIO_Pin = GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_OD;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);

	// RX
	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio);

	USART_StructInit(&uart);

	// uart itself
	uart.USART_BaudRate = 9600;				// the baudrate is set to the value we passed into this init function
	uart.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	uart.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	uart.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &uart);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);

	// enable rx interrupt
	nvic.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	nvic.NVIC_IRQChannelPreemptionPriority = 2;// this sets the priority group of the USART1 interrupts
	nvic.NVIC_IRQChannelSubPriority = 2;		 // this sets the subpriority inside the group
	nvic.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&nvic);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

	// enable interrupts
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART1, ENABLE);
}

static void _uart2_init(void){
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef uart;
	NVIC_InitTypeDef nvic;

	GPIO_StructInit(&gpio);
	USART_StructInit(&uart);

	// clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	// TX
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);

	// RX
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Pin = GPIO_Pin_3; // PA2 tx, PA3 rx
	GPIO_Init(GPIOA, &gpio);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);

	// uart itself
	uart.USART_BaudRate = 9600;				// the baudrate is set to the value we passed into this init function
	uart.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	uart.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	uart.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART2, &uart);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting

	nvic.NVIC_IRQChannel = USART2_IRQn;		 // we want to configure the USART1 interrupts
	nvic.NVIC_IRQChannelPreemptionPriority = 2;// this sets the priority group of the USART1 interrupts
	nvic.NVIC_IRQChannelSubPriority = 2;		 // this sets the subpriority inside the group
	nvic.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&nvic);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

	// enable interrupts
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(USART2, ENABLE);
}

static void _uart3_init(void){
	// clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // TX: PB10, RX: PB11
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &gpio);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_7);

	// uart itself
	USART_InitTypeDef uart;
	USART_StructInit(&uart);
	uart.USART_BaudRate = 115200;				// the baudrate is set to the value we passed into this init function
	USART_Init(USART3, &uart);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting

	// enable interrupts
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART3, ENABLE);

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = USART3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}

static void _uart_send_direct(struct stm32_uart_hw *hw, char ch){
	while(USART_GetFlagStatus(hw->hw, USART_FLAG_TXE) != SET);
	USART_SendData(hw->hw, ch);
}

static int _serial_write(serial_port_t serial, const void *data, size_t size, uint32_t timeout){
	struct stm32_uart *self = container_of(serial, struct stm32_uart, serial_ops);
	struct stm32_uart_hw *hw = (struct stm32_uart_hw*)self->driver_data;
	if(!hw || !hw->hw) return -1;
	uint8_t *buf = (uint8_t*)data;
	int sent = 0;
	for(size_t c = 0; c < size; c++){
		_uart_send_direct(hw, buf[c]);
		/*
		if(thread_queue_send(&hw->tx_queue, &buf[c], timeout) == 1){
			sent++;
		} else {
			break;
		}
		USART_ITConfig(hw->hw, USART_IT_TXE, ENABLE);
		*/
	}
	return sent;
}

static int _serial_read(serial_port_t serial, void *data, size_t size, uint32_t timeout){
	struct stm32_uart *self = container_of(serial, struct stm32_uart, serial_ops);
	struct stm32_uart_hw *hw = (struct stm32_uart_hw*)self->driver_data;
	if(!hw) return -1;
	char ch;
	char *buf = (char*)data;
	int pos = 0;
	// pop characters off the queue
	while(size && thread_queue_recv(&hw->rx_queue, &ch, (pos == 0)?(timeout):0) > 0){
		*(buf + pos) = ch;
		pos++;
		size--;
	}
	if(pos == 0) return -ETIMEDOUT;
	return pos;
}

static inline struct stm32_uart_hw *_get_hw(uint8_t id){
	volatile struct stm32_uart *ptr = _uart_ptr[id - 1];
	if(!ptr) return NULL;
	struct stm32_uart_hw *hw = (struct stm32_uart_hw*)ptr->driver_data;
	return hw;
}

static int32_t _uart_irq(struct stm32_uart_hw *hw){
	if(hw == NULL) return 0;

	int32_t wake = 0;
	// we check for incoming data on this device, ack the interrupt and copy data into the queue
	if( USART_GetITStatus(hw->hw, USART_IT_RXNE) ){
		USART_ClearITPendingBit(hw->hw, USART_IT_RXNE);
		char t = (char)USART_ReceiveData(hw->hw);
		thread_queue_send_from_isr(&hw->rx_queue, &t, &wake);
	}

	// we check for transmission read on this device, ack the interrupt and either send next byte or turn off the interrupt
	if( USART_GetITStatus(hw->hw, USART_IT_TXE)){
		USART_ClearITPendingBit(hw->hw, USART_IT_TXE);
		char ch;
		if(thread_queue_recv_from_isr(&hw->tx_queue, &ch, &wake) == 1){
			USART_SendData(hw->hw, ch);
		} else {
			USART_ITConfig(hw->hw, USART_IT_TXE, DISABLE);
		}
	}

	if( USART_GetFlagStatus(hw->hw, USART_FLAG_ORE) ){
		USART_ClearFlag(hw->hw, USART_FLAG_ORE);
	}

	return wake;
}

void USART1_IRQHandler(void);
void USART1_IRQHandler(void){
	struct stm32_uart_hw *hw = _get_hw(1);
	int32_t wake = _uart_irq(hw);
	thread_yield_from_isr(wake);
}

void USART2_IRQHandler(void);
void USART2_IRQHandler(void){
	struct stm32_uart_hw *hw = _get_hw(2);
	int32_t wake = _uart_irq(hw);
	thread_yield_from_isr(wake);
}

void USART3_IRQHandler(void);
void USART3_IRQHandler(void){
	struct stm32_uart_hw *hw = _get_hw(3);
	int32_t wake = _uart_irq(hw);
	thread_yield_from_isr(wake);
}

int stm32_uart_configure(struct stm32_uart *self, uint32_t baud, uint8_t data_bits, uint8_t parity){
	struct stm32_uart_hw *hw = (struct stm32_uart_hw*)self->driver_data;
	if(!hw) return -1;

	USART_Cmd(hw->hw, DISABLE);

	USART_InitTypeDef uart;
	USART_StructInit(&uart);

	// uart itself
	uart.USART_BaudRate = baud;				// the baudrate is set to the value we passed into this init function
	uart.USART_WordLength = (data_bits == 9)?USART_WordLength_9b:USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	uart.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	if(parity == 0)
		uart.USART_Parity = USART_Parity_No;
	else if(parity == 1)
		uart.USART_Parity = USART_Parity_Odd;
	else if(parity == 2)
		uart.USART_Parity = USART_Parity_Even;
	uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(hw->hw, &uart);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting

	USART_Cmd(hw->hw, ENABLE);
	return 0;
}

static const struct serial_ops _serial_ops = {
	.read = _serial_read,
	.write = _serial_write
};


int stm32_uart_init(struct stm32_uart *self, uint8_t hw_id){
	memset(self, 0, sizeof(*self));

	struct stm32_uart_hw *hw = NULL;
	switch(hw_id){
		case 1:
			if(_uart_ptr[0]) return -1;
			_uart_ptr[0] = self;
			_uart1_init();
			hw = &_uart_hw[0];
			hw->hw = USART1;
			break;
		case 2:
			if(_uart_ptr[1]) return -1;
			_uart_ptr[1] = self;
			_uart2_init();
			hw = &_uart_hw[1];
			hw->hw = USART2;
			break;
		case 3:
			if(_uart_ptr[2]) return -1;
			_uart_ptr[2] = self;
			_uart3_init();
			hw = &_uart_hw[2];
			hw->hw = USART3;
			break;
		default:
			return -1;
	}

	self->serial_ops = &_serial_ops; 

	thread_queue_init(&hw->tx_queue, 64, sizeof(char));
	thread_queue_init(&hw->rx_queue, 128, sizeof(char));

	self->driver_data = hw;

	return 0;
}

serial_port_t stm32_uart_get_serial_interface(struct stm32_uart *self){
	return &self->serial_ops;
}


