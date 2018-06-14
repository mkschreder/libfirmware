#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_can.h>
#include <string.h>

#include "thread.h"
#include "sem.h"

#if 0

struct stm32_can_hw {
	struct thread_queue rx_queue;
	struct semaphore tx_sem;
};

static struct stm32_can_hw can1 = {{0},{0}};

static int _stm32_can_write(struct stm32_can *self, const struct can_message *in, uint32_t timeout){
	(void)self;
	static CanTxMsg msg;

	memset(&msg, 0, sizeof(msg));
	msg.ExtId = in->id;
	msg.RTR=CAN_RTR_DATA;
	msg.IDE=CAN_ID_EXT;
	msg.DLC = in->len;

	memcpy(msg.Data, in->data, sizeof(msg.Data));

	uint8_t mb = 0;
	CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);

	if((mb = CAN_Transmit(CAN1, &msg)) == CAN_NO_MB) {
		return -1;
	}

	if(CAN_TransmitStatus(CAN1, mb) == CAN_TxStatus_Failed)
		return -1;

	// wait until transmission is ready
	if(sem_take_wait(&can1.tx_sem, timeout) < 0)
		return -1;

	//while(CAN_TransmitStatus(CAN1, mb) == CAN_TxStatus_Pending);

	return 1;
}

static int _can_send(can_port_t port, const struct can_message *msg, uint32_t timeout_ms){
	struct stm32_can *self = container_of(port, struct stm32_can, can_ops);
	thread_mutex_lock(&self->lock);
	int r = _stm32_can_write(self, msg, timeout_ms);
	thread_mutex_unlock(&self->lock);
	return r;
}

static int _can_recv(can_port_t port, struct can_message *msg, uint32_t timeout_ms){
	if(thread_queue_recv(&can1.rx_queue, msg, timeout_ms) < 0)
		return -1;
	return 1;
}

static const struct can_ops _can_ops = {
	.send = _can_send,
	.recv = _can_recv
};

void stm32_can_init(struct stm32_can *self){
	memset(self, 0, sizeof(*self));

	thread_mutex_init(&self->lock);
	self->can_ops = &_can_ops;

	thread_queue_init(&can1.rx_queue, 2, sizeof(struct can_message));
	sem_init(&can1.tx_sem);
	sem_give(&can1.tx_sem);

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	GPIO_InitTypeDef  gpio;

	/* GPIO clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* Configure CAN pin: RX */
	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &gpio);

	/* Configure CAN pin: TX */
	gpio.GPIO_Pin = GPIO_Pin_12;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);

	//GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);

	/* CAN register init */
	CAN_DeInit(CAN1);

	CAN_InitTypeDef can;
	CAN_StructInit(&can);

	/* CAN cell init */
	can.CAN_TTCM = DISABLE;
	can.CAN_ABOM = DISABLE;
	can.CAN_AWUM = DISABLE;
	can.CAN_NART = DISABLE;
	can.CAN_RFLM = DISABLE;
	can.CAN_TXFP = DISABLE;
	can.CAN_Mode = CAN_Mode_Normal;

	can.CAN_SJW = CAN_SJW_3tq; // 2
	can.CAN_BS1 = CAN_BS1_12tq; // 11
	can.CAN_BS2 = CAN_BS2_5tq; // 4
	can.CAN_Prescaler = (uint16_t)(clocks.PCLK1_Frequency / (uint32_t)((can.CAN_BS1 + can.CAN_BS2 + can.CAN_SJW + 1) * 125000));

	CAN_Init(CAN1, &can);

	/* CAN filter init */
	CAN_FilterInitTypeDef filter;
	filter.CAN_FilterNumber = 0;
	filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	filter.CAN_FilterScale = CAN_FilterScale_16bit;
	filter.CAN_FilterIdHigh = 0x0000;
	filter.CAN_FilterIdLow = 0x0000;
	filter.CAN_FilterMaskIdHigh = 0x0000;
	filter.CAN_FilterMaskIdLow = 0x0000;
	filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
	filter.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&filter);

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
	NVIC_Init(&nvic);

	CAN_ITConfig(CAN1, CAN_IT_FMP0 | CAN_IT_TME, ENABLE);
}

void USB_HP_CAN1_TX_IRQHandler(void){
	if(CAN1->TSR & CAN_TSR_RQCP0){
		CAN1->TSR |= CAN_TSR_RQCP0;
		CAN1->IER &= ~CAN_IER_TMEIE;
		sem_give_from_isr(&can1.tx_sem);
	}
	if(CAN1->TSR & CAN_TSR_RQCP1){
		CAN1->TSR |= CAN_TSR_RQCP1;
		CAN1->IER &= ~CAN_IER_TMEIE;
		sem_give_from_isr(&can1.tx_sem);
	}
	if(CAN1->TSR & CAN_TSR_RQCP2){
		CAN1->TSR |= CAN_TSR_RQCP2;
		CAN1->IER &= ~CAN_IER_TMEIE;
		sem_give_from_isr(&can1.tx_sem);
	}
}

void USB_LP_CAN1_RX0_IRQHandler(void){
	CanRxMsg msg;
	int32_t wake = 0;
	if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET){
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &msg);
		struct can_message cm;
		if(msg.IDE == CAN_ID_STD){
			cm.id = msg.StdId;
		} else {
			cm.id = msg.ExtId;
		}
		cm.len = msg.DLC;
		memcpy(cm.data, msg.Data, sizeof(cm.data));
		thread_queue_send_from_isr(&can1.rx_queue, &cm, &wake);
	}
	portYIELD_FROM_ISR(wake);
}

can_port_t stm32_can_get_interface(struct stm32_can *self){
	return &self->can_ops;
}
#endif
