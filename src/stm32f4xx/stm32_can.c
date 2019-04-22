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
#include "memory.h"
#include "can.h"
#include "mutex.h"
#include "atomic.h"
#include "console.h"

#include <errno.h>

#define STM32_CAN_RX_WAKE_INTERVAL_MS THREAD_WAIT_FOREVER

struct stm32_can {
	struct mutex lock;
	struct can_device dev;
	struct memory_device mem;

	struct can_counters counters;
	struct work bh;
	struct thread_queue rx_queue;
	struct thread_queue tx_queue;

	CAN_TypeDef *hw;

	struct can_dispatcher dispatcher;
};

static struct stm32_can *_interface[3] = {0};

static void _stm32_can_tx_start(struct stm32_can *self){
	CAN_ITConfig(self->hw, CAN_IT_TME, ENABLE);

	if(self->hw == CAN1){
		NVIC->STIR = CAN1_TX_IRQn;
	} else if(self->hw == CAN2){
		NVIC->STIR = CAN2_TX_IRQn;
	}
}

int _stm32_can_write(struct stm32_can *self, const struct can_message *in, uint32_t timeout){
	(void)self;
	CanTxMsg msg;
	memset(&msg, 0, sizeof(msg));

	// if the bus is passive then try sending a zero message until the bus is available again
	if(CAN_GetFlagStatus(self->hw, CAN_FLAG_EPV)){
		msg.StdId = 0;
		msg.IDE=CAN_ID_STD;
		msg.RTR=CAN_RTR_DATA;
		msg.DLC = 0;

		thread_queue_send(&self->tx_queue, &msg, 0);
		_stm32_can_tx_start(self);

		return -EIO;
	}

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

	if(thread_queue_send(&self->tx_queue, &msg, timeout) <= 0){
		atomic_inc(&self->counters.txto);
		return -ETIMEDOUT;
	}

	// enable error interrupts
	CAN_ITConfig(self->hw, CAN_IT_BOF, ENABLE);
	CAN_ITConfig(self->hw, CAN_IT_EPV, ENABLE);
	CAN_ITConfig(self->hw, CAN_IT_EWG, ENABLE);

	_stm32_can_tx_start(self);

	return 1;
}

static int _can_send(can_device_t port, const struct can_message *msg, uint32_t timeout_ms){
	struct stm32_can *self = container_of(port, struct stm32_can, dev.ops);
	thread_mutex_lock(&self->lock);
	int r = _stm32_can_write(self, msg, timeout_ms);
	thread_mutex_unlock(&self->lock);
	return r;
}

static int _can_subscribe(can_device_t can, struct can_listener *listener){
	struct stm32_can *self = container_of(can, struct stm32_can, dev.ops);
	thread_mutex_lock(&self->lock);
	can_dispatcher_add_listener(&self->dispatcher, listener);
	thread_mutex_unlock(&self->lock);
	return 0;
}

static const struct can_device_ops _stm32_can_ops = {
	.send = _can_send,
	.subscribe = _can_subscribe
};

static void _can_sce_isr(struct stm32_can *self, int32_t *wake){
	if(CAN_GetITStatus(self->hw, CAN_IT_ERR) != RESET){
		CAN_ClearITPendingBit(self->hw, CAN_IT_ERR);

		// last error has been changed in last error code bits
		if(CAN_GetITStatus(self->hw, CAN_IT_LEC) != RESET){
			CAN_ClearITPendingBit(self->hw, CAN_IT_LEC);
		}
		// bus off event has occurred
		if(CAN_GetITStatus(self->hw, CAN_IT_BOF) != RESET){
			CAN_ClearITPendingBit(self->hw, CAN_IT_BOF);
			CAN_ITConfig(self->hw, CAN_IT_BOF, DISABLE);
			atomic_inc(&self->counters.bof);
		}
		// error passive
		if(CAN_GetITStatus(self->hw, CAN_IT_EPV) != RESET){
			CAN_ClearITPendingBit(self->hw, CAN_IT_EPV);
			CAN_ITConfig(self->hw, CAN_IT_EPV, DISABLE);
			atomic_inc(&self->counters.epv);
		}
		// error warning
		if(CAN_GetITStatus(self->hw, CAN_IT_EWG) != RESET){
			CAN_ClearITPendingBit(self->hw, CAN_IT_EWG);
			CAN_ITConfig(self->hw, CAN_IT_EWG, DISABLE);
			atomic_inc(&self->counters.ewg);
		}
		// fifo 1 overrun
		if(CAN_GetITStatus(self->hw, CAN_IT_FOV1) != RESET){
			CAN_ClearITPendingBit(self->hw, CAN_IT_FOV1);
			atomic_inc(&self->counters.fov);
		}
		if(CAN_GetITStatus(self->hw, CAN_IT_FOV0) != RESET){
			CAN_ClearITPendingBit(self->hw, CAN_IT_FOV0);
			atomic_inc(&self->counters.fov);
		}
	}
}


static void _can_tx_isr(struct stm32_can *self, int32_t *wake){
	/*
	if(CAN_GetITStatus(self->hw, CAN_IT_TME) != RESET){
		CAN_ClearITPendingBit(self->hw, CAN_IT_TME);
		*/
		if(self->hw->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)){
			atomic_inc(&self->counters.tme);
			CanTxMsg msg;

			if(thread_queue_recv_from_isr(&self->tx_queue, &msg, wake) <= 0){
				// disable interrupt if we don't have any other messages
				CAN_ITConfig(self->hw, CAN_IT_TME, DISABLE);
				goto done;
			}

			if(CAN_Transmit(self->hw, &msg) == CAN_NO_MB){
				atomic_inc(&self->counters.txdrop);
				// Log error?
				goto done;
			}
		}
//	}
done:
	return;
}

static void _can_process_message_isr(struct stm32_can *self, CanRxMsg *msg, int32_t *wake){
	struct can_message cm;
	if(msg->IDE == CAN_ID_STD){
		cm.id = msg->StdId;
	} else {
		cm.id = msg->ExtId;
	}
	cm.len = msg->DLC;
	memcpy(cm.data, msg->Data, sizeof(cm.data));
	if(thread_queue_send_from_isr(&self->rx_queue, &cm, wake) < 0){
		atomic_inc(&self->counters.rxdrop);
	}
}

static void _can_rx0_isr(struct stm32_can *self, int32_t *wake){
	CanRxMsg msg;

	if(CAN_GetITStatus(self->hw, CAN_IT_FMP0) != RESET){
		CAN_ClearITPendingBit(self->hw, CAN_IT_FMP0);
		while(CAN_GetFlagStatus(self->hw, CAN_FLAG_FMP0) != RESET){
			CAN_Receive(self->hw, CAN_FIFO0, &msg);

			atomic_inc(&self->counters.fmp0);

			_can_process_message_isr(self, &msg, wake);
		}
	}
}

static void _can_rx1_isr(struct stm32_can *self, int32_t *wake){
	CanRxMsg msg;

	if(CAN_GetITStatus(self->hw, CAN_IT_FMP1) != RESET){
		CAN_ClearITPendingBit(self->hw, CAN_IT_FMP1);
		while(CAN_GetFlagStatus(self->hw, CAN_FLAG_FMP1) != RESET){
			CAN_Receive(self->hw, CAN_FIFO1, &msg);

			atomic_inc(&self->counters.fmp1);

			_can_process_message_isr(self, &msg, wake);
		}
	}
}

void CAN1_TX_IRQHandler(void){
	struct stm32_can *self = _interface[0];
	BUG_ON(!self);
	int32_t wake;

	_can_tx_isr(self, &wake);

	thread_yield_from_isr(wake);
}

void CAN1_RX0_IRQHandler(void){
	struct stm32_can *self = _interface[0];
	BUG_ON(!self);
	int32_t wake = 0;

	_can_rx0_isr(self, &wake);

	thread_yield_from_isr(wake);
}

void CAN1_RX1_IRQHandler(void){
	struct stm32_can *self = _interface[0];
	BUG_ON(!self);
	int32_t wake = 0;

	_can_rx1_isr(self, &wake);

	thread_yield_from_isr(wake);
}

void CAN1_SCE_IRQHandler(void){
	struct stm32_can *self = _interface[0];
	if(!self) return;
	int32_t wake = 0;

	_can_sce_isr(self, &wake);

	thread_yield_from_isr(wake);
}

void CAN2_TX_IRQHandler(void){
	struct stm32_can *self = _interface[1];
	BUG_ON(!self);
	int32_t wake;

	_can_tx_isr(self, &wake);

	thread_yield_from_isr(wake);
}

void CAN2_RX0_IRQHandler(void){
	struct stm32_can *self = _interface[1];
	BUG_ON(!self);
	int32_t wake = 0;

	_can_rx0_isr(self, &wake);

	thread_yield_from_isr(wake);
}

void CAN2_RX1_IRQHandler(void){
	struct stm32_can *self = _interface[1];
	BUG_ON(!self);

	int32_t wake = 0;

	_can_rx1_isr(self, &wake);

	thread_yield_from_isr(wake);
}

void CAN2_SCE_IRQHandler(void){
	struct stm32_can *self = _interface[1];
	if(!self) return;
	int32_t wake = 0;

	_can_sce_isr(self, &wake);

	thread_yield_from_isr(wake);
}

static int _can_next_message(can_device_t dev, struct can_message *msg){
	struct stm32_can *self = container_of(dev, struct stm32_can, dev.ops);
	return thread_queue_recv(&self->rx_queue, msg, STM32_CAN_RX_WAKE_INTERVAL_MS);
}

static int _memory_read(memory_device_t dev, size_t offset, void *data, size_t size){
	struct stm32_can *self = container_of(dev, struct stm32_can, mem.ops);
	if(size != sizeof(struct can_counters) || offset != 0) return -EINVAL;
	//thread_mutex_lock(&self->lock);
	memcpy(data, &self->counters, sizeof(self->counters));
	//thread_mutex_unlock(&self->lock);
	return (int)size;
}

static int _memory_write(memory_device_t dev, size_t offs, const void *data, size_t size){
	return -EINVAL;
}

static struct memory_device_ops _mem_ops = {
	.read = _memory_read,
	.write = _memory_write
};

static int _stm32_can_cmd(console_device_t con, void *userptr, int argc, char **argv){
	struct stm32_can *self = (struct stm32_can*)userptr;

	if(argc == 2 && strcmp(argv[1], "info") == 0){
		struct can_counters *cnt = &self->counters;
		console_printf(con, "\tTX count: %d\n", cnt->tme); 
		console_printf(con, "\tTX dropped: %d\n", cnt->txdrop);
		console_printf(con, "\tRX count: %d\n", cnt->rxp); 
		console_printf(con, "\tRX dropped: %d\n", cnt->rxdrop);
		console_printf(con, "\tTX timeout: %d\n", cnt->txto);
		console_printf(con, "\tRX on FIFO0: %d\n", cnt->fmp0);
		console_printf(con, "\tRX on FIFO1: %d\n", cnt->fmp1);
		console_printf(con, "\tTotal errors: %d\n", cnt->lec);
		console_printf(con, "\tBus off errors: %d\n", cnt->bof);
		console_printf(con, "\tBus passive errors: %d\n", cnt->epv);
		console_printf(con, "\tBus errors warnings: %d\n", cnt->ewg);
		console_printf(con, "\tFIFO Overflow errors: %d\n", cnt->fov);
	}
	return 0;
}

static int _stm32_can_probe(void *fdt, int fdt_node){
	//int baud = fdt_get_int_or_default(fdt, (int)fdt_node, "baud", 1000000);
	CAN_TypeDef *CANx = (CAN_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);
	int sjw = fdt_get_int_or_default(fdt, (int)fdt_node, "sjw", 1);
	int bs1 = fdt_get_int_or_default(fdt, (int)fdt_node, "bs1", 6);
	int bs2 = fdt_get_int_or_default(fdt, (int)fdt_node, "bs2", 7);
	int prescaler = fdt_get_int_or_default(fdt, (int)fdt_node, "prescaler", 3);
	int rx_queue = fdt_get_int_or_default(fdt, (int)fdt_node, "rx_queue", 32);
	int tx_queue = fdt_get_int_or_default(fdt, (int)fdt_node, "tx_queue", 8);
	int irq_pre_prio = fdt_get_int_or_default(fdt, (int)fdt_node, "irq_preempt_prio", 1);
	int irq_sub_prio = fdt_get_int_or_default(fdt, (int)fdt_node, "irq_sub_prio", 1);
	console_device_t console = console_find_by_ref(fdt, fdt_node, "console");

	if(sjw < 1 || sjw > 4){
		printk(PRINT_ERROR "can: sjw must be 1-4\n");
		return -EINVAL;
	}
	if(bs1 < 1 || bs1 > 16){
		printk(PRINT_ERROR "can: bs1 must be 1-16\n");
		return -EINVAL;
	}
	if(bs2 < 1 || bs2 > 8){
		printk(PRINT_ERROR "can: bs1 must be 1-8\n");
		return -EINVAL;
	}
	if(prescaler < 1 || prescaler > 0x1ff){
		printk(PRINT_ERROR "can: prescaler must be 1-512\n");
		return -EINVAL;
	}

	uint8_t idx, irq_rx0, irq_rx1, irq_tx, irq_sce, filter_id;

	if(CANx == CAN1){
		idx = 0;
		filter_id = 0;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
		irq_rx0 = CAN1_RX0_IRQn;
		irq_rx1 = CAN1_RX1_IRQn;
		irq_tx = CAN1_TX_IRQn;
		irq_sce = CAN1_SCE_IRQn;
	} else if(CANx == CAN2){
		idx = 1;
		filter_id = 14;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
		irq_rx0 = CAN2_RX0_IRQn;
		irq_rx1 = CAN2_RX1_IRQn;
		irq_tx = CAN2_TX_IRQn;
		irq_sce = CAN2_SCE_IRQn;
	} else {
		printk(PRINT_ERROR "can: unsupported can interface (reg)\n");
		return -EINVAL;
	}

	struct stm32_can *self = kzmalloc(sizeof(struct stm32_can));
	if(!self) return -ENOMEM;
	self->hw = CANx;
	_interface[idx] = self;

	thread_mutex_init(&self->lock);

	thread_queue_init(&self->tx_queue, (size_t)tx_queue, sizeof(CanTxMsg));
	thread_queue_init(&self->rx_queue, (size_t)rx_queue, sizeof(struct can_message));

	can_dispatcher_init(&self->dispatcher, &self->dev.ops, _can_next_message);

	/* CAN register init */
	CAN_DeInit(CANx);

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
	can.CAN_SJW = (uint8_t)(sjw - 1);
	can.CAN_BS1 = (uint8_t)(bs1 - 1);
	can.CAN_BS2 = (uint8_t)(bs2 - 1);
	can.CAN_Prescaler = (uint16_t)(prescaler); //(uint16_t)(clocks.PCLK1_Frequency / (uint32_t)((uint32_t)(can.CAN_BS1 + can.CAN_BS2 + can.CAN_SJW + 1) * (uint32_t)baud));

	CAN_Init(CANx, &can);

	/* CAN filter init */
	CAN_FilterInitTypeDef filter;
	filter.CAN_FilterNumber = filter_id;
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
	nvic.NVIC_IRQChannel = irq_rx0;
	nvic.NVIC_IRQChannelPreemptionPriority = (uint8_t)irq_pre_prio;
	nvic.NVIC_IRQChannelSubPriority = (uint8_t)irq_sub_prio;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = irq_rx1;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = irq_tx;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = irq_sce;
	NVIC_Init(&nvic);

	CAN_ITConfig(CANx, CAN_IT_FMP0 | CAN_IT_FMP1 | CAN_IT_TME |
		CAN_IT_ERR |
		CAN_IT_LEC |
		CAN_IT_BOF |
		CAN_IT_EPV |
		CAN_IT_EWG |
		CAN_IT_FOV1 |
		CAN_IT_FOV0,
	ENABLE);

	can_device_init(&self->dev, fdt, fdt_node, &_stm32_can_ops);
	can_device_register(&self->dev);

	memory_device_init(&self->mem, fdt, fdt_node, &_mem_ops);
	memory_device_register(&self->mem);

	if(console){
		console_add_command(console, self, fdt_get_name(fdt, fdt_node, NULL), "STM32 CAN interface", "", _stm32_can_cmd);
	}

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	printk(PRINT_DEFAULT "can: sysclk %d, pclk1 %d, pclk2 %d, hclk %d\n",
			clocks.SYSCLK_Frequency, clocks.PCLK1_Frequency, clocks.PCLK2_Frequency, clocks.HCLK_Frequency);

	printk(PRINT_DEFAULT "can: BTR: %08x\n", self->hw->BTR);

	printk(PRINT_SUCCESS "can%d: ready\n", idx + 1);

	return 0;
}

static int _stm32_can_remove(void *fdt, int fdt_node){
	// TODO
    return -1;
}

DEVICE_DRIVER(stm32_can, "st,stm32_can", _stm32_can_probe, _stm32_can_remove)
