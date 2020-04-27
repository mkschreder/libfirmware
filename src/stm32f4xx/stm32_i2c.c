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
* FILE ............... src/stm32f4xx/stm32_i2c.c
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
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_i2c.h>
#include <string.h>
#include <stdio.h>

#include "i2c.h"
#include <libfdt/libfdt.h>

#include "driver.h"
#include "thread/thread.h"
#include "thread/mutex.h"
#include "thread/sem.h"
#include "atomic.h"
#include "console.h"

#include <errno.h>

struct stm32_i2c {
	struct i2c_device dev;
	I2C_TypeDef *hw;
	volatile uint8_t addr;
	volatile bool error;
	volatile size_t writing;
	volatile size_t reading;
	volatile const uint8_t *write_p;
	volatile uint8_t *read_p;
	struct mutex lock;
	struct semaphore complete;
	struct {
		atomic_t tcn, tcn_failed;
		atomic_t tout;
		atomic_t sent_bytes, recv_bytes;
		atomic_t err_berr;
		atomic_t err_arlo;
		atomic_t err_af;
		atomic_t err_ovr;
		atomic_t err_pec;
		atomic_t err_timeout;
		atomic_t err_smbalert;
	} cnt;
};

static struct stm32_i2c *_i2c_devices[3] = {NULL};

static int _stm32_i2c_transfer(i2c_device_t dev, uint8_t addr, const void *tx_data, size_t tx_len, void *rx_data, size_t rx_len, uint32_t timeout_ms){
	struct stm32_i2c *self = container_of(dev, struct stm32_i2c, dev.ops);
    int result = 0;

	if (thread_mutex_lock_wait(&self->lock, timeout_ms) != 0){
		atomic_inc(&self->cnt.tout);
		return -ETIMEDOUT;
	}

	self->addr    = addr;
	self->writing = tx_len;
	self->write_p = (uint8_t *)tx_data;
	self->reading = rx_len;
	self->read_p  = rx_data;

	I2C_GenerateSTART(self->hw, ENABLE);

	if (thread_sem_take_wait(&self->complete, timeout_ms) != 0) {
		atomic_inc(&self->cnt.tout);
		result = -ETIMEDOUT;
	}

	if (self->error) {
		self->error = false;
		atomic_inc(&self->cnt.tcn_failed);
		result = -EIO;
	} else {
		atomic_inc(&self->cnt.tcn);
	}

	thread_mutex_unlock(&self->lock);

    return result;
}

static void _i2c_event_irq_handler(struct stm32_i2c *self) {
    int32_t should_yield = 0;

    /* Read both status registers*/
	uint32_t events = I2C_GetLastEvent(self->hw);

    /* Start bit sent. */
	if(events & I2C_SR1_SB){
		I2C_Send7bitAddress(self->hw,
				(uint8_t)(self->addr << 1),
				(self->writing)?I2C_Direction_Transmitter:I2C_Direction_Receiver);
    }

    /* Address sent. */
	if(events & I2C_SR1_ADDR){
		if(self->writing){
			/* Send a byte off the write buffer. */
			self->hw->DR = *(self->write_p);
			self->write_p++;
			self->writing--;
		} else {
			if (self->reading > 1) {
				I2C_AcknowledgeConfig(self->hw, ENABLE);
			} else {
				I2C_GenerateSTOP(self->hw, ENABLE);
			}
		}
    }

    /* RX Not empty (got new byte) */
	if(events & I2C_SR1_RXNE){
        /* Read into read buffer. */
        *(self->read_p) = (uint8_t)I2C_ReceiveData(self->hw);
		atomic_inc(&self->cnt.recv_bytes);
        self->read_p++;
        self->reading--;

        if (self->reading == 1) {
			I2C_AcknowledgeConfig(self->hw, DISABLE);
			I2C_GenerateSTOP(self->hw, ENABLE);
        } else if (self->reading == 0) {
            thread_sem_give_from_isr(&self->complete, &should_yield);
        }
    }

	if(events & I2C_SR1_TXE && !(events & I2C_SR1_BTF)){
        if (self->writing) {
            /* send next byte from write buffer. */
			I2C_SendData(self->hw, *(self->write_p));
			atomic_inc(&self->cnt.sent_bytes);
            self->write_p++;
            self->writing--;
        } else {
            if (self->reading) {
                /* done writing, now reading: send repeated start */
				I2C_GenerateSTART(self->hw, ENABLE);
            } else {
                /* done reading: send stop */
				I2C_GenerateSTOP(self->hw, ENABLE);
                thread_sem_give_from_isr(&self->complete, &should_yield);
            }
        }
    }

	thread_yield_from_isr(should_yield);
}

static void _i2c_error_irq_handler(struct stm32_i2c *self) {
    int32_t should_yield = 0;

	if(I2C_GetFlagStatus(self->hw, I2C_FLAG_BERR)){
        atomic_inc(&self->cnt.err_berr);
    }

	if(I2C_GetFlagStatus(self->hw, I2C_FLAG_ARLO)){
        atomic_inc(&self->cnt.err_arlo);
    }

	if(I2C_GetFlagStatus(self->hw, I2C_FLAG_AF)){
        atomic_inc(&self->cnt.err_af);
    }

	if(I2C_GetFlagStatus(self->hw, I2C_FLAG_OVR)){
        atomic_inc(&self->cnt.err_ovr);
    }

	if(I2C_GetFlagStatus(self->hw, I2C_FLAG_PECERR)){
        atomic_inc(&self->cnt.err_pec);
    }

	if(I2C_GetFlagStatus(self->hw, I2C_FLAG_SMBALERT)){
        atomic_inc(&self->cnt.err_smbalert);
    }

	if(I2C_GetFlagStatus(self->hw, I2C_FLAG_TIMEOUT)){
        atomic_inc(&self->cnt.err_timeout);
    }

    /* Read SRs to clear them */
    self->hw->SR1;
    self->hw->SR2;

	// reset all writable bits (errors)
    self->hw->SR1 = 0;

    /* Send stop */
	I2C_GenerateSTOP(self->hw, ENABLE);

    self->error = 1;

    thread_sem_give_from_isr(&self->complete, &should_yield);

	thread_yield_from_isr(should_yield);
}

void I2C1_EV_IRQHandler (void) {
    _i2c_event_irq_handler(_i2c_devices[0]);
}

void I2C2_EV_IRQHandler (void) {
    _i2c_event_irq_handler(_i2c_devices[1]);
}

void I2C3_EV_IRQHandler (void) {
    _i2c_event_irq_handler(_i2c_devices[2]);
}

void I2C1_ER_IRQHandler (void) {
    _i2c_error_irq_handler(_i2c_devices[0]);
}

void I2C2_ER_IRQHandler (void) {
    _i2c_error_irq_handler(_i2c_devices[1]);
}

void I2C3_ER_IRQHandler (void) {
    _i2c_error_irq_handler(_i2c_devices[2]);
}

static struct i2c_device_ops _i2c_ops = {
	.transfer = _stm32_i2c_transfer
};

static int _stm32_i2c_cmd(console_device_t con, void *ptr, int argc, char **argv){
	struct stm32_i2c *self = (struct stm32_i2c*)ptr;
	if(argc == 2 && strcmp(argv[1], "status") == 0){
		console_printf(con, "%-16s%d\n", "TCN Complete:", self->cnt.tcn);
		console_printf(con, "%-16s%d\n", "TCN Failed:", self->cnt.tcn_failed);
		console_printf(con, "%-16s%d\n", "Timeouts:", self->cnt.tout);
		console_printf(con, "%-16s%d bytes\n", "Sent:", self->cnt.sent_bytes);
		console_printf(con, "%-16s%d bytes\n", "Received:", self->cnt.recv_bytes);
		console_printf(con, "%-16s%d\n", "BERR Errors:", self->cnt.err_berr);
		console_printf(con, "%-16s%d\n", "ARLO Errors:", self->cnt.err_arlo);
		console_printf(con, "%-16s%d\n", "AF Errors:", self->cnt.err_af);
		console_printf(con, "%-16s%d\n", "OVR Errors:", self->cnt.err_ovr);
		console_printf(con, "%-16s%d\n", "PEC Errors:", self->cnt.err_pec);
		console_printf(con, "%-16s%d\n", "TOUT Errors:", self->cnt.err_timeout);
		console_printf(con, "%-16s%d\n", "SMBALERT Errors:", self->cnt.err_smbalert);
	}
	return 0;
}

static int _stm32_i2c_probe(void *fdt, int fdt_node) {
	I2C_TypeDef *I2Cx = (I2C_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);
	uint32_t baud = (uint32_t)fdt_get_int_or_default(fdt, (int)fdt_node, "baud", 100000);
	console_device_t console = console_find_by_ref(fdt, fdt_node, "console");

	int idx = 0;
	uint8_t irq_er = 0, irq_ev = 0;
	if(I2Cx == I2C1){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
		irq_er = I2C1_ER_IRQn;
		irq_ev = I2C1_EV_IRQn;
		idx = 1;
	} else if(I2Cx == I2C2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		irq_er = I2C2_ER_IRQn;
		irq_ev = I2C2_EV_IRQn;
		idx = 2;
	} else if(I2Cx == I2C3){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
		irq_er = I2C3_ER_IRQn;
		irq_ev = I2C3_EV_IRQn;
		idx = 3;
	} else {
		printk("i2c: unsupported device\n");
		return -1;
	}

	I2C_DeInit(I2Cx);
	I2C_InitTypeDef i2c;
    I2C_StructInit(&i2c);

    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_16_9;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = (uint32_t)baud;

    I2C_Init(I2Cx, &i2c);

	struct stm32_i2c *self = kzmalloc(sizeof(struct stm32_i2c));
	self->hw = I2Cx;
	thread_mutex_init(&self->lock);
	thread_sem_init(&self->complete);

	_i2c_devices[idx - 1] = self;

	i2c_device_init(&self->dev, fdt, fdt_node, &_i2c_ops);
	i2c_device_register(&self->dev);

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = irq_ev;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = irq_er;
	NVIC_Init(&nvic);

	I2C_ITConfig(I2Cx, I2C_IT_ERR | I2C_IT_BUF | I2C_IT_EVT, ENABLE);

    I2C_Cmd(I2Cx, ENABLE);

	if(console){
		console_add_command(console, self, fdt_get_name(fdt, fdt_node, NULL), "i2c low level interface", "", _stm32_i2c_cmd);
	}

	printk(PRINT_SUCCESS "i2c%d: ready (speed %d)\n", idx, baud);

	return 0;
}

static int _stm32_i2c_remove(void *fdt, int fdt_node){
	return -1;
}

DEVICE_DRIVER(stm32_i2c, "st,stm32_i2c", _stm32_i2c_probe, _stm32_i2c_remove)

