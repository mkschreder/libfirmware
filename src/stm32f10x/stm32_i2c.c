/*
 * This file is part of libfirmware.
 *
 * Original impoementation based on cleanflight.
 *
 * This software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>

#include <libfdt/libfdt.h>

#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"

#include "driver.h"
#include "i2c.h"
#include "sem.h"

#define I2C_DEFAULT_TIMEOUT 50

// I2C2
// SCL  PB10
// SDA  PB11
// I2C1
// SCL  PB6
// SDA  PB7

enum {
    I2C_ERR_NONE,
    I2C_ERR_BUS_ERROR,
    I2C_ERR_ARB_LOST,
    I2C_ERR_ARB_FAILED,
    I2C_ERR_UNKNOWN_EVENT
};

struct stm32_i2c {
    struct i2c_device dev;
    I2C_TypeDef *hw;

	GPIO_TypeDef *gpio_scl, *gpio_sda;
	uint16_t gpio_scl_pin, gpio_sda_pin;

    struct {
        uint8_t addr;
        uint8_t reg;
        const uint8_t *wr_buf;
        uint8_t *rd_buf;
        int len;
        int cursor;
        bool stop_sent;
        bool reg_sent;
        int error;
    } isr;

    thread_sem_t done;

    /*
    uint16_t scl;
    uint16_t sda;
    uint8_t ev_irq;
    uint8_t er_irq;
    uint32_t peripheral;
    */
};

static struct stm32_i2c *_devices[2] = {0, 0};

static void _stm32_i2c_unstick(struct stm32_i2c *self){
	uint16_t scl = self->gpio_scl_pin;
	uint16_t sda = self->gpio_sda_pin;
	GPIO_TypeDef *GPIOscl = self->gpio_scl;
	GPIO_TypeDef *GPIOsda = self->gpio_sda;
	GPIO_SetBits(GPIOscl, scl);
	GPIO_SetBits(GPIOsda, sda);

	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = scl;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(self->gpio_scl, &gpio);
	gpio.GPIO_Pin = sda;
	GPIO_Init(self->gpio_sda, &gpio);

    // Analog Devices AN-686
	for(int c = 0; c < 9; c++){
		while(!GPIO_ReadInputDataBit(self->gpio_scl, scl));
		GPIO_ResetBits(self->gpio_scl, scl);
		thread_sleep_ms(1);
		GPIO_SetBits(self->gpio_scl, scl);
	}
	GPIO_ResetBits(self->gpio_scl, scl);
	thread_sleep_ms(1);
	GPIO_ResetBits(self->gpio_sda, sda);
	GPIO_SetBits(self->gpio_scl, scl);
	thread_sleep_ms(1);
	GPIO_SetBits(self->gpio_sda, sda);

	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Pin = scl;
	GPIO_Init(self->gpio_scl, &gpio);
	gpio.GPIO_Pin = sda;
	GPIO_Init(self->gpio_sda, &gpio);
}

static int _i2c_check_error(struct stm32_i2c *self){
    // Read the I2Cx status register
    uint32_t SR1Register = self->hw->SR1;
    if (SR1Register & 0x0700) {
        (void)self->hw->SR2;                                                // read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
        I2C_ITConfig(self->hw, I2C_IT_BUF, DISABLE);                        // disable the RXNE/TXE interrupt - prevent the ISR tailchaining onto the ER (hopefully)
        if (!(SR1Register & 0x0200) && !(self->hw->CR1 & 0x0200)) {         // if we dont have an ARLO error, ensure sending of a stop
            if (self->hw->CR1 & 0x0100) {                                   // We are currently trying to send a start, this is very bad as start, stop will hang the peripheral
                // TODO - busy waiting in highest priority IRQ. Maybe only set flag and handle it from main loop 
                while (self->hw->CR1 & 0x0100) { ; }                        // wait for any start to finish sending  
                I2C_GenerateSTOP(self->hw, ENABLE);                         // send stop to finalise bus transaction 
                while (self->hw->CR1 & 0x0200) { ; }                        // wait for stop to finish sending       
            } else {
                I2C_GenerateSTOP(self->hw, ENABLE);                         // stop to free up the bus
                I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_ERR, DISABLE);   // Disable EVT and ERR interrupts while bus inactive                                                                        
            }
        }
    }
    self->hw->SR1 = (uint16_t)(self->hw->SR1 & ~0x0F00);
	return 0;
}

static int _i2c_wait_flag_cleared(struct stm32_i2c *self, uint32_t flag){
    int timeout = 10;
	while(I2C_GetFlagStatus(self->hw, flag) == SET && --timeout > 0) thread_sleep_ms(1);
    if(timeout == 0){
        return -EBUSY;
    }
    return 0;
}

static int _i2c_wait_event(struct stm32_i2c *self, uint32_t ev){
    int timeout = 100;
	while(!I2C_CheckEvent(self->hw, ev) && --timeout > 0) thread_sleep_ms(1);
	_i2c_check_error(self);
    if(timeout == 0){
	    I2C_GenerateSTOP(self->hw, ENABLE);
        return -ETIMEDOUT;
    }
    return 0;
}

static int _stm32_i2c_write_buf(i2c_device_t dev, uint8_t addr, uint8_t reg, const void *buf, size_t len){
    struct stm32_i2c *self = container_of(dev, struct stm32_i2c, dev.ops);
    const uint8_t *data = (const uint8_t*)buf;

    return -1;

    if (!self->hw || !len)
        return -1;

	/* While the bus is busy */
	if(_i2c_wait_flag_cleared(self, I2C_FLAG_BUSY) < 0) {
        return -EBUSY;
    }

	/* Send START condition */
	I2C_GenerateSTART(self->hw, ENABLE);

	/* Test on EV5 and clear it */
    if(_i2c_wait_event(self, I2C_EVENT_MASTER_MODE_SELECT) < 0) {
        return -ETIMEDOUT;
    }

    // send device address
	I2C_Send7bitAddress(self->hw, addr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
    if(_i2c_wait_event(self, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) < 0) {
        return -ETIMEDOUT;
    }

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(self->hw, ENABLE);

    // send register address
	I2C_SendData(self->hw, reg);

    // wait until the byte was sent
    if(_i2c_wait_event(self, I2C_EVENT_MASTER_BYTE_TRANSMITTED) < 0) {
        return -ETIMEDOUT;
    }

#if 0
	/* Send repeated START condition */
	I2C_GenerateSTART(self->hw, ENABLE);

	/* Test on EV5 and clear it */
    if(_i2c_wait_event(self, I2C_EVENT_MASTER_MODE_SELECT) < 0) {
        return -ETIMEDOUT;
    }

    // send device address
	I2C_Send7bitAddress(self->hw, addr, I2C_Direction_Receiver);

	/* Test on EV6 and clear it */
    if(_i2c_wait_event(self, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) < 0) {
        return -ETIMEDOUT;
    }
#endif 
	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(self->hw, ENABLE);

    if(len == 0){
        I2C_AcknowledgeConfig(self->hw, DISABLE);
        I2C_GenerateSTOP(self->hw, ENABLE);
    }

	for(int c = 0; c < (int)len; c++){
	    if(c == ((int)len - 1)){
            I2C_AcknowledgeConfig(self->hw, DISABLE);
            I2C_GenerateSTOP(self->hw, ENABLE);
        } else {
            I2C_AcknowledgeConfig(self->hw, ENABLE);
        }
        // wait for a byte to be received
        if(_i2c_wait_event(self, I2C_EVENT_MASTER_BYTE_RECEIVED) < 0) {
            return -ETIMEDOUT;
        }
        I2C_SendData(self->hw, data[c]);
        // wait until the byte was sent
        if(_i2c_wait_event(self, I2C_EVENT_MASTER_BYTE_TRANSMITTED) < 0) {
            return -ETIMEDOUT;
        }
	}

    /*
    //thread_mutex_lock(&self->lock);
	int timeout = 10;
	while(I2C_GetFlagStatus(self->hw, I2C_FLAG_BUSY) == SET && --timeout > 0) thread_sleep_ms(1);
    if(timeout == 0){
        return -EBUSY;
    }

    I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);

    memset(&self->isr, 0, sizeof(self->isr));

    self->isr.addr = (uint8_t)(addr_ << 1);
    self->isr.reg = reg_;
    self->isr.wr_buf = (const uint8_t *)_data;
    self->isr.len = (int)len;

    I2C_GenerateSTART(self->hw, ENABLE);

    I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_ERR, ENABLE);

    if(thread_sem_take_wait(&self->done, I2C_DEFAULT_TIMEOUT) < 0){
        dbg_printk("i2c: tout\n");
        return -ETIMEDOUT;
    }

    if(self->isr.error){
        dbg_printk("i2c: fail %d\n", self->isr.error);
        return self->isr.error;
    }

    dbg_printk("i2c: ok\n");
*/
    return (int)len;
}

static int _stm32_i2c_read_buf(i2c_device_t dev, uint8_t addr, uint8_t reg, void* buf, size_t len){
    struct stm32_i2c *self = container_of(dev, struct stm32_i2c, dev.ops);
    uint8_t *data = (uint8_t*)buf;

    if (!self->hw || !len)
        return -1;

    addr = (uint8_t)(addr << 1);

    dbg_printk("i2c: bsy\n");

	if(_i2c_wait_flag_cleared(self, I2C_FLAG_BUSY) < 0) {
        return -EBUSY;
    }

    dbg_printk("i2c: start\n");

    I2C_AcknowledgeConfig(self->hw, ENABLE);

	/* Send START condition */
	I2C_GenerateSTART(self->hw, ENABLE);

	/* Test on EV5 and clear it */
    if(_i2c_wait_event(self, I2C_EVENT_MASTER_MODE_SELECT) < 0) {
        return -ETIMEDOUT;
    }

    dbg_printk("i2c: adr\n");
    // send device address
	I2C_Send7bitAddress(self->hw, addr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
    if(_i2c_wait_event(self, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) < 0) {
        return -ETIMEDOUT;
    }

    dbg_printk("i2c: reg\n");
	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(self->hw, ENABLE);

    // send register address
	I2C_SendData(self->hw, reg);

    // wait until the byte was sent
    if(_i2c_wait_event(self, I2C_EVENT_MASTER_BYTE_TRANSMITTED) < 0) {
        return -ETIMEDOUT;
    }

    dbg_printk("i2c: repst\n");
	/* Send repeated START condition */
	I2C_GenerateSTART(self->hw, ENABLE);

	/* Test on EV5 and clear it */
    if(_i2c_wait_event(self, I2C_EVENT_MASTER_MODE_SELECT) < 0) {
        return -ETIMEDOUT;
    }

    // send device address
	I2C_Send7bitAddress(self->hw, addr, I2C_Direction_Receiver);

	/* Test on EV6 and clear it */
    if(_i2c_wait_event(self, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) < 0) {
        return -ETIMEDOUT;
    }

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(self->hw, ENABLE);

    dbg_printk("i2c: rd\n");
	for(int c = 0; c < (int)len; c++){
    /*
	    if(c == ((int)len - 1)){
            I2C_AcknowledgeConfig(self->hw, DISABLE);
            I2C_GenerateSTOP(self->hw, ENABLE);
        } else {
            I2C_AcknowledgeConfig(self->hw, ENABLE);
        }
        */
        // wait for a byte to be received
        if(_i2c_wait_event(self, I2C_EVENT_MASTER_BYTE_RECEIVED) < 0) {
            return -ETIMEDOUT;
        }
        data[c] = I2C_ReceiveData(self->hw);
	}

    I2C_GenerateSTOP(self->hw, ENABLE);
/*
	int timeout = 10;
	while(I2C_GetFlagStatus(self->hw, I2C_FLAG_BUSY) == SET && --timeout > 0) thread_sleep_ms(1);
    if(timeout == 0){
        return -EBUSY;
    }

    I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);

    memset(&self->isr, 0, sizeof(self->isr));

    self->isr.addr = (uint8_t)(addr_ << 1);
    self->isr.reg = reg_;
    self->isr.rd_buf = (uint8_t*)buf;
    self->isr.len = (int)len;

    I2C_GenerateSTART(self->hw, ENABLE);

    I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_ERR, ENABLE);

    if(thread_sem_take_wait(&self->done, I2C_DEFAULT_TIMEOUT) < 0){
        dbg_printk("i2c: tout\n");
        return -ETIMEDOUT;
    }

    dbg_printk("i2c: sr1: %04x, sr2: %04x, cr1: %04x\n", self->hw->SR1, self->hw->SR2, self->hw->CR1);

    if(self->isr.error){
        dbg_printk("i2c: fail %d\n", self->isr.error);
        return self->isr.error;
    }

    dbg_printk("i2c: ok\n");
*/
    return (int)len;
}

static void i2c_er_handler(struct stm32_i2c *self){
    if(I2C_GetITStatus(self->hw, I2C_IT_BERR)){
        // bus error
        I2C_ClearITPendingBit(self->hw, I2C_IT_BERR);
        self->isr.error = -I2C_ERR_BUS_ERROR;
    }
    if(I2C_GetITStatus(self->hw, I2C_IT_ARLO)){
        // arbitration lost
        I2C_ClearITPendingBit(self->hw, I2C_IT_ARLO);
        self->isr.error = -I2C_ERR_ARB_LOST;
    }
    if(I2C_GetITStatus(self->hw, I2C_IT_AF)){
        // no ack received from slave (ie slave not connected)
        I2C_ClearITPendingBit(self->hw, I2C_IT_AF);
        // generate stop to release the bus
        I2C_GenerateSTOP(self->hw, ENABLE);
        self->isr.error = -I2C_ERR_ARB_FAILED;
    }
    // disable interrupts and release the device
    I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
	// reset all error bits
	self->hw->SR1 = (uint16_t)(self->hw->SR1 & ~0x0F00);
    thread_sem_give_from_isr(&self->done);
}

void i2c_ev_handler(struct stm32_i2c *self){
    // read the status register here
    switch(I2C_GetLastEvent(self->hw)){
		/** 
		  * @brief  Communication start
		  * 
		  * After sending the START condition (I2C_GenerateSTART() function) the master 
		  * has to wait for this event. It means that the Start condition has been correctly 
		  * released on the I2C bus (the bus is free, no other devices is communicating).
		  * 
		  */
		/* --EV5 */
        case I2C_EVENT_MASTER_MODE_SELECT: {
    		I2C_AcknowledgeConfig(self->hw, ENABLE);
			if(self->isr.reg_sent && self->isr.rd_buf){
                I2C_Send7bitAddress(self->hw, self->isr.addr, I2C_Direction_Receiver);
			} else {
                I2C_Send7bitAddress(self->hw, self->isr.addr, I2C_Direction_Transmitter);
			}
/*
            if(self->isr.len == 0){
                I2C_AcknowledgeConfig(self->hw, DISABLE);
                I2C_GenerateSTOP(self->hw, ENABLE);
                self->isr.stop_sent = true;
            }
*/
        } break;
		/** 
		  * @brief  Address Acknowledge
		  * 
		  * After checking on EV5 (start condition correctly released on the bus), the 
		  * master sends the address of the slave(s) with which it will communicate 
		  * (I2C_Send7bitAddress() function, it also determines the direction of the communication: 
		  * Master transmitter or Receiver). Then the master has to wait that a slave acknowledges 
		  * his address. If an acknowledge is sent on the bus, one of the following events will 
		  * be set:
		  * 
		  *  1) In case of Master Receiver (7-bit addressing): the I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED 
		  *     event is set.
		  *  
		  *  2) In case of Master Transmitter (7-bit addressing): the I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED 
		  *     is set
		  *  
		  *  3) In case of 10-Bit addressing mode, the master (just after generating the START 
		  *  and checking on EV5) has to send the header of 10-bit addressing mode (I2C_SendData() 
		  *  function). Then master should wait on EV9. It means that the 10-bit addressing 
		  *  header has been correctly sent on the bus. Then master should send the second part of 
		  *  the 10-bit address (LSB) using the function I2C_Send7bitAddress(). Then master 
		  *  should wait for event EV6. 
		  *     
		  */
		/* --EV6 */
        case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
        case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED: {
			// clear ev6 by setting PE bit
			//I2C_Cmd(self->hw, ENABLE);
			// if we only want one byte then we should request stop now
            if(self->isr.rd_buf && self->isr.reg_sent && self->isr.len == 1){
                I2C_AcknowledgeConfig(self->hw, DISABLE);
                I2C_GenerateSTOP(self->hw, ENABLE);
				//I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
                self->isr.stop_sent = true;
            }
		} break;
		case I2C_EVENT_MASTER_MODE_ADDRESS10: {
			// this is currently ignored!
		} break;
		/** 
		  * @brief Communication events
		  * 
		  * If a communication is established (START condition generated and slave address 
		  * acknowledged) then the master has to check on one of the following events for 
		  * communication procedures:
		  *  
		  * 1) Master Receiver mode: The master has to wait on the event EV7 then to read 
		  *    the data received from the slave (I2C_ReceiveData() function).
		  * 
		  * 2) Master Transmitter mode: The master has to send data (I2C_SendData() 
		  *    function) then to wait on event EV8 or EV8_2.
		  *    These two events are similar: 
		  *     - EV8 means that the data has been written in the data register and is 
		  *       being shifted out.
		  *     - EV8_2 means that the data has been physically shifted out and output 
		  *       on the bus.
		  *     In most cases, using EV8 is sufficient for the application.
		  *     Using EV8_2 leads to a slower communication but ensure more reliable test.
		  *     EV8_2 is also more suitable than EV8 for testing on the last data transmission 
		  *     (before Stop condition generation).
		  *     
		  *  @note In case the  user software does not guarantee that this event EV7 is 
		  *  managed before the current byte end of transfer, then user may check on EV7 
		  *  and BTF flag at the same time (ie. (I2C_EVENT_MASTER_BYTE_RECEIVED | I2C_FLAG_BTF)).
		  *  In this case the communication may be slower.
		  * 
		  */
		/* Master RECEIVER mode -----------------------------*/ 
		/* --EV7 */
        case I2C_EVENT_MASTER_BYTE_RECEIVED: {
            // always get the by from dr
            uint8_t ch = I2C_ReceiveData(self->hw);
            // store byte if there is space
            if(self->isr.rd_buf && self->isr.cursor < self->isr.len){
                self->isr.rd_buf[self->isr.cursor++] = ch;
                I2C_AcknowledgeConfig(self->hw, ENABLE);
                if(self->isr.cursor == self->isr.len){
                    // if next to last byte then generate stop
                    I2C_AcknowledgeConfig(self->hw, DISABLE);
                    I2C_GenerateSTOP(self->hw, ENABLE);
                    self->isr.stop_sent = true;
                }
            } else if(self->isr.stop_sent){
                // if last byte then signal that we are done
                I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
                thread_sem_give_from_isr(&self->done);
            }
        } break;
		/* Master TRANSMITTER mode --------------------------*/
		/* --EV8 */
        case I2C_EVENT_MASTER_BYTE_TRANSMITTING: {
            if(self->isr.cursor == -1){
				I2C_SendData(self->hw, self->isr.reg);
				self->isr.cursor++;
			} else if(self->isr.wr_buf && (self->isr.cursor < self->isr.len)){
                // if we have data left to send then send it
                I2C_SendData(self->hw, self->isr.wr_buf[self->isr.cursor++]);
            }
        } break;
		/* --EV8_2 */
        case I2C_EVENT_MASTER_BYTE_TRANSMITTED: {
            if(self->isr.rd_buf) {
                if(self->isr.len){
                    // we only end up here if we have sent the reg address and we should now switch into read mode
                    I2C_GenerateSTART(self->hw, ENABLE);
                } else {
                    // if we have sent reg address and no more data is to be sent
                    I2C_GenerateSTOP(self->hw, ENABLE);
                    self->isr.stop_sent = true;
                }
            } else if(self->isr.stop_sent){
                // if we are in write mode and stop has been sent then we are done
                I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
                thread_sem_give_from_isr(&self->done);
            }
             // we must wait for the start to clear, otherwise we get constant BTF
             while (self->hw->CR1 & 0x0100) {; }
        } break;
        default: {
            // an unknown event has occured so we have to send stop and notify that we are done.
            I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
            I2C_AcknowledgeConfig(self->hw, DISABLE);
            // dummy read to send nack + stop
            (void)I2C_ReceiveData(self->hw);
            I2C_GenerateSTOP(self->hw, ENABLE);
            self->isr.error = -I2C_ERR_UNKNOWN_EVENT;
            thread_sem_give_from_isr(&self->done);
        } break;
    }
}

void I2C1_ER_IRQHandler(void){
    struct stm32_i2c *dev = _devices[0];
    if(!dev) return;
    i2c_er_handler(dev);
}

void I2C1_EV_IRQHandler(void){
    struct stm32_i2c *dev = _devices[0];
    if(!dev) return;
    i2c_ev_handler(dev);
}

void I2C2_ER_IRQHandler(void){
    struct stm32_i2c *dev = _devices[1];
    if(!dev) return;
    i2c_er_handler(dev);
}

void I2C2_EV_IRQHandler(void){
    struct stm32_i2c *dev = _devices[1];
    if(!dev) return;
    i2c_ev_handler(dev);
}

static const struct i2c_device_ops _stm32_i2c_ops = {
    .write = _stm32_i2c_write_buf,
    .read = _stm32_i2c_read_buf
};

int _stm32_i2c_probe(void *fdt, int fdt_node){
    NVIC_InitTypeDef nvic;
    I2C_InitTypeDef i2c;

	I2C_TypeDef *I2Cx = (I2C_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);
	int speed = fdt_get_int_or_default(fdt, (int)fdt_node, "speed", 100000);
	int irq_prio = fdt_get_int_or_default(fdt, (int)fdt_node, "irq_prio", 1);

	if(I2Cx == 0) {
        dbg_printk("i2c: no reg!\n");
		return -EINVAL;
	}

	// find gpio and pins of the sda and scl because we need to be able to do unstick procedure with them
	int len = 0;
	const fdt32_t *val = (const fdt32_t*)fdt_getprop(fdt, fdt_node, "pins", &len);
	if(len != 4) {
		dbg_printk("i2c: nopins!\n");
		return -1;
	}

	int node = fdt_node_offset_by_phandle(fdt, (uint32_t)fdt32_to_cpu(*val));
	val = (const fdt32_t*)fdt_getprop(fdt, node, "pinctrl", &len);
	if(len == 0 || !val || (len % 3) != 0) {
		dbg_printk("i2c: invalid pins!\n");
		return -1;
	}

    uint8_t irq_ev = 0, irq_er = 0;
    uint8_t __unused idx = 0;
    if(I2Cx == I2C1){
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        irq_ev = I2C1_EV_IRQn;
        irq_er = I2C1_ER_IRQn;
        idx = 1;
    } else if(I2Cx == I2C2){
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
        irq_ev = I2C2_EV_IRQn;
        irq_er = I2C2_ER_IRQn;
        idx = 2;
    }

    struct stm32_i2c *self = kzmalloc(sizeof(struct stm32_i2c));

    if(!self){
        dbg_printk("i2c: nomem!\n");
        return -ENOMEM;
    }

	const fdt32_t *base = val;
	self->gpio_scl = (GPIO_TypeDef*)fdt32_to_cpu(*(base));
	self->gpio_scl_pin = (uint16_t)fdt32_to_cpu(*(base + 1));
	self->gpio_sda = (GPIO_TypeDef*)fdt32_to_cpu(*(base + 3));
	self->gpio_sda_pin = (uint16_t)fdt32_to_cpu(*(base + 3 + 1));

    i2c_device_init(&self->dev, fdt_node, &_stm32_i2c_ops);

    // Turn on peripheral clock, save device and index
    self->hw = I2Cx;

    thread_sem_init(&self->done);

    // diable I2C interrrupts first to avoid ER handler triggering
    I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);

    _devices[idx - 1] = self;

    // clock out stuff to make sure slaves arent stuck
    // This will also configure GPIO as AF_OD at the end
    _stm32_i2c_unstick(self);

    // Init I2C peripheral
    I2C_DeInit(self->hw);
    I2C_StructInit(&i2c);

    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = (uint32_t)speed;
	i2c.I2C_OwnAddress1 = 0;

    I2C_Cmd(self->hw, ENABLE);
    I2C_Init(self->hw, &i2c);

    I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);

    // I2C ER Interrupt
    nvic.NVIC_IRQChannel = irq_er;
    nvic.NVIC_IRQChannelPreemptionPriority = (uint8_t)irq_prio;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // I2C EV Interrupt
    nvic.NVIC_IRQChannel = irq_ev;
    nvic.NVIC_IRQChannelPreemptionPriority = (uint8_t)irq_prio;
    nvic.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic);

    i2c_device_register(&self->dev);

    dbg_printk("i2c%d: ok, %dhz\n", idx, speed);
    dbg_printk("i2c%d sr1: %04x, sr2: %04x, cr1: %04x\n", idx, self->hw->SR1, self->hw->SR2, self->hw->CR1);

    return 0;
}
static int _stm32_i2c_remove(void *fdt, int fdt_node){
	// TODO
    return -1;
}

DEVICE_DRIVER(stm32_i2c, "st,stm32_i2c", _stm32_i2c_probe, _stm32_i2c_remove)
