#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_can.h>
#include <string.h>

#include <libfdt/libfdt.h>

#include "driver.h"
#include "math.h"

#include "thread.h"
#include "queue.h"
#include "sem.h"
#include "work.h"

#include "can.h"
#include "mutex.h"
#include "atomic.h"

#include <errno.h>

struct stm32_can {
	struct mutex lock;
	struct can_device dev;
	/*
	struct {
		struct mutex lock;
		struct list_head listeners;
	} dispatcher;
	*/
	struct can_counters counters;
	struct work bh;
	struct thread_queue rx_queue;
	struct thread_queue tx_queue;
	//struct semaphore tx_sem;
	const struct can_ops *can_ops;
};

static struct stm32_can *interface[1] = {0};

int _stm32_can_write(struct stm32_can *self, const struct can_message *in, uint32_t timeout){
	(void)self;
	CanTxMsg msg;

	memset(&msg, 0, sizeof(msg));

	if(in->id > 0x7ff){
		msg.ExtId = in->id;
		msg.IDE=CAN_ID_EXT;
	} else {
		msg.StdId = in->id;
		msg.IDE=CAN_ID_STD;
	}

	msg.RTR=CAN_RTR_DATA;
	msg.DLC = in->len;

	memcpy(msg.Data, in->data, sizeof(msg.Data));

	if(thread_queue_send(&self->tx_queue, &msg, timeout) < 0){
		atomic_inc(&self->counters.txto);
	}

	CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);

	NVIC->STIR = CAN1_TX_IRQn;
/*
	// wait for a mailbox
	if(thread_sem_take_wait(&self->tx_sem, timeout) < 0)
		return -1;

	uint8_t mb = 0;
	CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);

	if((mb = CAN_Transmit(CAN1, &msg)) == CAN_NO_MB){
		// should never get here because we wait for semaphore above
		thread_sem_give(&self->tx_sem);
		return -EIO;
	}

	if(CAN_TransmitStatus(CAN1, mb) == CAN_TxStatus_Failed) {
		thread_sem_give(&self->tx_sem);
		return -EIO;
	}

	thread_sem_give(&self->tx_sem);
	*/
	return 1;
}

static int _can_send(can_port_t port, const struct can_message *msg, uint32_t timeout_ms){
	struct stm32_can *self = container_of(port, struct stm32_can, can_ops);
	thread_mutex_lock(&self->lock);
	int r = _stm32_can_write(self, msg, timeout_ms);
	thread_mutex_unlock(&self->lock);
	return r;
}

static int _can_handler(can_port_t can, can_listen_cmd_t cmd, struct can_listener *listener){
	struct stm32_can *self = container_of(can, struct stm32_can, can_ops);
	return can_device_handler(&self->dev, cmd, listener);
}

static int _can_control(can_port_t can, can_control_cmd_t cmd, struct can_control_param *p){
	struct stm32_can *self = container_of(can, struct stm32_can, can_ops);
	CAN_TypeDef *CANx = CAN1;
	switch(cmd){
		case CAN_CMD_GET_STATUS: {
			uint32_t esr = CANx->ESR;
			memset(p, 0, sizeof(*p));
			p->v.status.cnt.tec = (uint8_t)((esr >> 16) & 0xff);
			p->v.status.cnt.rec = (uint8_t)((esr >> 24) & 0xff);
			p->v.status.cnt.tme = atomic_get(&self->counters.tme);
			p->v.status.cnt.lec = atomic_get(&self->counters.lec);
			p->v.status.cnt.bof = atomic_get(&self->counters.bof);
			p->v.status.cnt.epv = atomic_get(&self->counters.epv);
			p->v.status.cnt.ewg = atomic_get(&self->counters.ewg);
			p->v.status.cnt.fov = atomic_get(&self->counters.fov);
			p->v.status.cnt.fmp0 = atomic_get(&self->counters.fmp0);
			p->v.status.cnt.fmp1 = atomic_get(&self->counters.fmp1);
			p->v.status.cnt.txto = atomic_get(&self->counters.txto);
			p->v.status.cnt.rxp = atomic_get(&self->counters.rxp);
			p->v.status.cnt.rxdrop = atomic_get(&self->counters.rxdrop);
			p->v.status.cnt.txdrop = atomic_get(&self->counters.txdrop);
			return 0;
		} break;
	}
	return -EINVAL;
}

static const struct can_ops _can_ops = {
	.send = _can_send,
	.handler = _can_handler,
	.control = _can_control
};

// subscribe to error interrupt and record error counts
void CAN1_SCE_IRQHandler(void){
	struct stm32_can *self = interface[0];
	if(!self) return;

	if(CAN_GetITStatus(CAN1, CAN_IT_ERR) != RESET){
		CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);

		// last error has been changed in last error code bits
		if(CAN_GetITStatus(CAN1, CAN_IT_LEC) != RESET){
			CAN_ClearITPendingBit(CAN1, CAN_IT_LEC);
		}
		// bus off event has occurred
		if(CAN_GetITStatus(CAN1, CAN_IT_BOF) != RESET){
			CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
			atomic_inc(&self->counters.bof);
		}
		// error passive
		if(CAN_GetITStatus(CAN1, CAN_IT_EPV) != RESET){
			CAN_ClearITPendingBit(CAN1, CAN_IT_EPV);
			atomic_inc(&self->counters.epv);
		}
		// error warning
		if(CAN_GetITStatus(CAN1, CAN_IT_EWG) != RESET){
			CAN_ClearITPendingBit(CAN1, CAN_IT_EWG);
			atomic_inc(&self->counters.ewg);
		}
		// fifo 1 overrun
		if(CAN_GetITStatus(CAN1, CAN_IT_FOV1) != RESET){
			CAN_ClearITPendingBit(CAN1, CAN_IT_FOV1);
			atomic_inc(&self->counters.fov);
		}
		if(CAN_GetITStatus(CAN1, CAN_IT_FOV0) != RESET){
			CAN_ClearITPendingBit(CAN1, CAN_IT_FOV0);
			atomic_inc(&self->counters.fov);
		}
	}
}

void CAN1_TX_IRQHandler(void){
	struct stm32_can *self = interface[0];
	BUG_ON(!self);

	//if(CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET){
	//	CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
		// if any mailboxes are empty then we send a message
		if(CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)){
			atomic_inc(&self->counters.tme);
			CanTxMsg msg;
			int32_t wake = 0;
			if(thread_queue_recv_from_isr(&self->tx_queue, &msg, &wake) <= 0){
				// disable interrupt if we don't have any other messages
				CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);
				goto done;
			}

			if(CAN_Transmit(CAN1, &msg) == CAN_NO_MB){
				atomic_inc(&self->counters.txdrop);
				// Log error?
				goto done;
			}
		} else {
		}
	//}
	//CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);
done:
	return;
/*
	if(CAN1->TSR & CAN_TSR_RQCP0){
		CAN1->TSR |= CAN_TSR_RQCP0;
		goto give;
	}
	if(CAN1->TSR & CAN_TSR_RQCP1){
		CAN1->TSR |= CAN_TSR_RQCP1;
		goto give;
	}
	if(CAN1->TSR & CAN_TSR_RQCP2){
		CAN1->TSR |= CAN_TSR_RQCP2;
		goto give;
	}
	return;
give:
	GPIO_ResetBits(GPIOD, GPIO_Pin_5);
	if(interface[0])
		thread_sem_give_from_isr(&interface[0]->tx_sem);
	*/
}

/** message dispatcher
 * This one is called in the context of the work processing thread.
 * It currently assumes that listener list is never modified after the interface has been set up
 */
void _dispatcher(void *ptr){
	struct stm32_can *self = (struct stm32_can*)ptr; //container_of(work, struct stm32_can, bh);
	while(1){
		// dispatch the recieved messages through driver callbacks. Now with interrupts enabled and in kernel context.
		struct can_message cm;
		if(thread_queue_recv(&self->rx_queue, &cm, 100) > 0){
			can_device_dispatch_message(&self->dev, &cm);
			atomic_inc(&self->counters.rxp);
		}
	}
}

static void _can1_process_message_isr(struct stm32_can *self, CanRxMsg *msg, int32_t *wake){
	struct can_message cm;
	if(msg->IDE == CAN_ID_STD){
		cm.id = msg->StdId;
	} else {
		cm.id = msg->ExtId;
	}
	cm.len = msg->DLC;
	memcpy(cm.data, msg->Data, sizeof(cm.data));
	if(interface[0]){
		if(thread_queue_send_from_isr(&interface[0]->rx_queue, &cm, wake) < 0){
			atomic_inc(&self->counters.rxdrop);
		} else {
			//queue_work_from_isr(&interface[0]->bh, 1, wake);
		}
	}
}

void CAN1_RX0_IRQHandler(void){
	struct stm32_can *self = interface[0];
	BUG_ON(!self);

	CanRxMsg msg;
	int32_t wake = 0;

	if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET){
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		while(CAN_GetFlagStatus(CAN1, CAN_FLAG_FMP0) != RESET){
			CAN_Receive(CAN1, CAN_FIFO0, &msg);

			atomic_inc(&self->counters.fmp0);

			_can1_process_message_isr(self, &msg, &wake);
		}
	}

	//thread_yield_from_isr(wake);
}

void CAN1_RX1_IRQHandler(void){
	struct stm32_can *self = interface[0];
	BUG_ON(!self);

	CanRxMsg msg;
	int32_t wake = 0;

	if(CAN_GetITStatus(CAN1, CAN_IT_FMP1) != RESET){
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP1);
		while(CAN_GetFlagStatus(CAN1, CAN_FLAG_FMP1) != RESET){
			CAN_Receive(CAN1, CAN_FIFO1, &msg);

			atomic_inc(&self->counters.fmp1);

			_can1_process_message_isr(self, &msg, &wake);
		}
	}
}

static int _stm32_can_probe(void *fdt, int fdt_node){
	//int baud = fdt_get_int_or_default(fdt, (int)fdt_node, "baud", 1000000);
	int sjw = fdt_get_int_or_default(fdt, (int)fdt_node, "sjw", 1);
	int bs1 = fdt_get_int_or_default(fdt, (int)fdt_node, "bs1", 6);
	int bs2 = fdt_get_int_or_default(fdt, (int)fdt_node, "bs2", 7);
	int prescaler = fdt_get_int_or_default(fdt, (int)fdt_node, "prescaler", 3);
	int rx_queue = fdt_get_int_or_default(fdt, (int)fdt_node, "rx_queue", 32);
	int tx_queue = fdt_get_int_or_default(fdt, (int)fdt_node, "tx_queue", 8);
	int reg = fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);
	int irq_pre_prio = fdt_get_int_or_default(fdt, (int)fdt_node, "irq_preempt_prio", 1);
	int irq_sub_prio = fdt_get_int_or_default(fdt, (int)fdt_node, "irq_sub_prio", 1);

	// we currently only support one interface
	if(reg > 0 || interface[reg]){
		return -1;
	}

	sjw = constrain_i32(sjw - 1, CAN_SJW_1tq, CAN_SJW_4tq);
	bs1 = constrain_i32(bs1 - 1, CAN_BS1_1tq, CAN_BS1_16tq);
	bs2 = constrain_i32(bs2 - 1, CAN_BS2_1tq, CAN_BS2_8tq);

	struct stm32_can *self = kzmalloc(sizeof(struct stm32_can));
	if(!self) return -1;

	thread_queue_init(&self->tx_queue, (size_t)tx_queue, sizeof(CanTxMsg));
	thread_queue_init(&self->rx_queue, (size_t)rx_queue, sizeof(struct can_message));

	thread_mutex_init(&self->lock);

	thread_create(_dispatcher, "can0_rx", 230, self, 2, NULL);
	//work_init(&self->bh, _dispatcher);

	self->can_ops = &_can_ops;
	interface[reg] = self;

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

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

	// choose time quanta so that prescaler is a whole number in relation to can clock
	can.CAN_SJW = (uint8_t)sjw;
	can.CAN_BS1 = (uint8_t)bs1;
	can.CAN_BS2 = (uint8_t)bs2;
	can.CAN_Prescaler = (uint8_t)prescaler; //(uint16_t)(clocks.PCLK1_Frequency / (uint32_t)((uint32_t)(can.CAN_BS1 + can.CAN_BS2 + can.CAN_SJW + 1) * (uint32_t)baud));

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
	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = (uint8_t)irq_pre_prio;
	nvic.NVIC_IRQChannelSubPriority = (uint8_t)irq_sub_prio;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = CAN1_RX1_IRQn;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = CAN1_SCE_IRQn;
	NVIC_Init(&nvic);

	CAN_ITConfig(CAN1, CAN_IT_FMP0 | CAN_IT_FMP1 | CAN_IT_TME |
		CAN_IT_ERR |
		CAN_IT_LEC |
		CAN_IT_BOF |
		CAN_IT_EPV |
		CAN_IT_EWG |
		CAN_IT_FOV1 |
		CAN_IT_FOV0,
	ENABLE);

	can_register_device(fdt, fdt_node, &self->dev, &self->can_ops);

	return 0;
}

static int _stm32_can_remove(void *fdt, int fdt_node){
	// TODO
    return -1;
}

DEVICE_DRIVER(stm32_can, "st,stm32_can", _stm32_can_probe, _stm32_can_remove)
