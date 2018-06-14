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

#define I2C_DEFAULT_TIMEOUT 10

// I2C2
// SCL  PB10
// SDA  PB11
// I2C1
// SCL  PB6
// SDA  PB7

struct stm32_i2c {
    struct i2c_device dev;
    I2C_TypeDef *hw;
    GPIO_TypeDef *gpio;
    struct {
        uint16_t status;
        bool error;
        bool busError;
        bool busy;

        uint8_t addr;
        uint8_t reg;
        uint8_t bytes;
        uint8_t writing;
        uint8_t reading;
        uint8_t* write_p;
        uint8_t* read_p;

        uint8_t subaddress_sent, final_stop;
        int8_t index;
    } isr;

    /*
    uint16_t scl;
    uint16_t sda;
    uint8_t ev_irq;
    uint8_t er_irq;
    uint32_t peripheral;
    */
};

static struct stm32_i2c *_devices[2] = {0, 0};

void _stm32_i2c_init(struct stm32_i2c *self);
/*
static const i2cDevice_t i2cHardwareMap[] = {
    { I2C1, GPIOB, Pin_6, Pin_7, I2C1_EV_IRQn, I2C1_ER_IRQn, RCC_APB1Periph_I2C1 },
    { I2C2, GPIOB, Pin_10, Pin_11, I2C2_EV_IRQn, I2C2_ER_IRQn, RCC_APB1Periph_I2C2 },
};
*/
/*
// Copy of peripheral address for IRQ routines
static I2C_TypeDef *self->hw = NULL;
// Copy of device index for reinit, etc purposes
static I2CDevice self->hw_index;
static bool i2cOverClock;
void i2cSetOverclock(uint8_t OverClock)
{
    i2cOverClock = (OverClock) ? true : false;
}

static volatile uint16_t i2cErrorCount = 0;
*/

static void _stm32_i2c_reinit_device(struct stm32_i2c *self){
    // reinit peripheral + clock out garbage
    if (self->isr.busError) {
        _stm32_i2c_init(self);
    }
}

static void _stm32_i2c_unstick(struct stm32_i2c *self){
/*
    int i;

    gpio_pin_set(self->gpio_scl);
    gpio_pin_set(self->gpio_sda);

    gpio_pin_control(self->gpio_scl, GPIO_PIN_CONTROL_SET, GPIO_MODE_OD);
    cfg.pin = scl | sda;
    cfg.speed = Speed_2MHz;
    cfg.mode = Mode_Out_OD;
    gpioInit(gpio, &cfg);

    // Analog Devices AN-686
    // We need 9 clock pulses + STOP condition
    for (i = 0; i < 9; i++) {
        // Wait for any clock stretching to finish
        while (!digitalIn(gpio, scl))
            usleep(5);

        // Pull low
        digitalLo(gpio, scl); // Set bus low
        usleep(5);
        // Release high again
        digitalHi(gpio, scl); // Set bus high
        usleep(5);
    }

    // Generate a start then stop condition
    // SCL  PB10
    // SDA  PB11
    digitalLo(gpio, scl); // Set bus scl low
    usleep(10);
    digitalLo(gpio, sda); // Set bus data low
    usleep(10);

    digitalHi(gpio, scl); // Set bus scl high
    usleep(10);
    digitalHi(gpio, sda); // Set bus sda high

    // Init pins
    cfg.pin = scl | sda;
    cfg.speed = Speed_2MHz;
    cfg.mode = Mode_AF_OD;
    gpioInit(gpio, &cfg);
    */
}


int _stm32_i2c_write_buf(i2c_device_t dev, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data){
    struct stm32_i2c *self = container_of(dev, struct stm32_i2c, dev.ops);
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    if (!self->hw)
        return -1;

    self->isr.addr = (uint8_t)(addr_ << 1);
    self->isr.reg = reg_;
    self->isr.writing = 1;
    self->isr.reading = 0;
    self->isr.write_p = data;
    self->isr.read_p = data;
    self->isr.bytes = len_;
    self->isr.busy = 1;
    self->isr.error = false;
    self->isr.busError = false;

    if (!(self->hw->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(self->hw->CR1 & 0x0100)) {                                    // ensure sending a start
            while (self->hw->CR1 & 0x0200 && --timeout > 0) { ; }           // wait for any stop to finish sending
            if (self->isr.error || timeout == 0) {
                _stm32_i2c_reinit_device(self);
                return -EIO;
            }
            I2C_GenerateSTART(self->hw, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    /*
    timeout = I2C_DEFAULT_TIMEOUT;
    while (self->isr.busy && --timeout > 0) { ; }
    if (timeout == 0) {
        return i2c_reinit_device(self);
    }
    */

    return 0;
}

int _stm32_i2c_write_reg(i2c_device_t dev, uint8_t addr_, uint8_t reg_, uint8_t data){
    return _stm32_i2c_write_buf(dev, addr_, reg_, 1, &data);
}

int _stm32_i2c_read_buf(i2c_device_t dev, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf){
    struct stm32_i2c *self = container_of(dev, struct stm32_i2c, dev.ops);

    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    if (!self->hw)
        return -1;

    self->isr.addr = (uint8_t)(addr_ << 1);
    self->isr.reg = reg_;
    self->isr.writing = 0;
    self->isr.reading = 1;
    self->isr.read_p = buf;
    self->isr.write_p = buf;
    self->isr.bytes = len;
    self->isr.busy = 1;
    self->isr.error = false;
    self->isr.busError = false;

    if (!(self->hw->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(self->hw->CR1 & 0x0100)) {                                    // ensure sending a start
            while (self->hw->CR1 & 0x0200 && --timeout > 0) { ; }           // wait for any stop to finish sending
            if (self->isr.error || timeout == 0){
                _stm32_i2c_reinit_device(self);
                return -EIO;
            }
            I2C_GenerateSTART(self->hw, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    /*
    timeout = I2C_DEFAULT_TIMEOUT;
    while (busy && --timeout > 0) { ; }
    if (timeout == 0)
        return i2cHandleHardwareFailure();
    */

    return 0;
}

static void i2c_er_handler(struct stm32_i2c *self){
    // Read the self->hw status register
    uint16_t status = self->hw->SR1;

    if (status & 0x0F00) {                                         // an error
        self->isr.error = true;

        // Only re-initialise bus if bus error indicated, don't reset bus when ARLO or AF
        if (status & I2C_SR1_BERR)
            self->isr.busError = true;
    }

    // If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
    if (status & 0x0700) {
        (void)self->hw->SR2;                                                // read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
        I2C_ITConfig(self->hw, I2C_IT_BUF, DISABLE);                        // disable the RXNE/TXE interrupt - prevent the ISR tailchaining onto the ER (hopefully)
        if (!(status & 0x0200) && !(self->hw->CR1 & 0x0200)) {         // if we dont have an ARLO error, ensure sending of a stop
            if (self->hw->CR1 & 0x0100) {                                   // We are currently trying to send a start, this is very bad as start, stop will hang the peripheral
                // TODO - busy waiting in highest priority IRQ. Maybe only set flag and handle it from main loop
                while (self->hw->CR1 & 0x0100) { ; }                        // wait for any start to finish sending
                I2C_GenerateSTOP(self->hw, ENABLE);                         // send stop to finalise bus transaction
                while (self->hw->CR1 & 0x0200) { ; }                        // wait for stop to finish sending
                _stm32_i2c_init(self);                                    // reset and configure the hardware
            } else {
                I2C_GenerateSTOP(self->hw, ENABLE);                         // stop to free up the bus
                I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_ERR, DISABLE);   // Disable EVT and ERR interrupts while bus inactive
            }
        }
    }

    // reset all the error bits to clear the interrupt
    self->hw->SR1 = (uint16_t)(self->hw->SR1 & (~0x0F00));
    self->isr.busy = 0;
}

void i2c_ev_handler(struct stm32_i2c *self)
{
    uint16_t status = self->hw->SR1;                                         // read the status register here

    if (status & 0x0001) {                                              // we just sent a start - EV5 in ref manual
        self->hw->CR1 = (uint16_t)(self->hw->CR1 & (~0x0800));                                           // reset the POS bit so ACK/NACK applied to the current byte
        I2C_AcknowledgeConfig(self->hw, ENABLE);                            // make sure ACK is on
        self->isr.index = 0;                                                      // reset the index
        if (self->isr.reading && (self->isr.subaddress_sent || 0xFF == self->isr.reg)) {              // we have sent the subaddr
            self->isr.subaddress_sent = 1;                                        // make sure this is set in case of no subaddress, so following code runs correctly
            if (self->isr.bytes == 2)
                self->hw->CR1 |= 0x0800;                                    // set the POS bit so NACK applied to the final byte in the two byte read
            I2C_Send7bitAddress(self->hw, self->isr.addr, I2C_Direction_Receiver);    // send the address and set hardware mode
        } else {                                                        // direction is Tx, or we havent sent the sub and rep start
            I2C_Send7bitAddress(self->hw, self->isr.addr, I2C_Direction_Transmitter); // send the address and set hardware mode
            if (self->isr.reg != 0xFF)                                            // 0xFF as subaddress means it will be ignored, in Tx or Rx mode
                self->isr.index = -1;                                             // send a subaddress
        }
    } else if (status & 0x0002) {                                       // we just sent the address - EV6 in ref manual
        // Read SR1,2 to clear ADDR
        __DMB();                                                        // memory fence to control hardware
        if (self->isr.bytes == 1 && self->isr.reading && self->isr.subaddress_sent) {                 // we are receiving 1 byte - EV6_3
            I2C_AcknowledgeConfig(self->hw, DISABLE);                       // turn off ACK
            __DMB();
            (void)self->hw->SR2;                                            // clear ADDR after ACK is turned off
            I2C_GenerateSTOP(self->hw, ENABLE);                             // program the stop
            self->isr.final_stop = 1;
            I2C_ITConfig(self->hw, I2C_IT_BUF, ENABLE);                     // allow us to have an EV7
        } else {                                                        // EV6 and EV6_1
            (void)self->hw->SR2;                                            // clear the ADDR here
            __DMB();
            if (self->isr.bytes == 2 && self->isr.reading && self->isr.subaddress_sent) {             // rx 2 bytes - EV6_1
                I2C_AcknowledgeConfig(self->hw, DISABLE);                   // turn off ACK
                I2C_ITConfig(self->hw, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to fill
            } else if (self->isr.bytes == 3 && self->isr.reading && self->isr.subaddress_sent)        // rx 3 bytes
                I2C_ITConfig(self->hw, I2C_IT_BUF, DISABLE);                // make sure RXNE disabled so we get a BTF in two bytes time
            else                                                        // receiving greater than three bytes, sending subaddress, or transmitting
                I2C_ITConfig(self->hw, I2C_IT_BUF, ENABLE);
        }
    } else if (status & 0x004) {                                        // Byte transfer finished - EV7_2, EV7_3 or EV8_2
        self->isr.final_stop = 1;
        if (self->isr.reading && self->isr.subaddress_sent) {                               // EV7_2, EV7_3
            if (self->isr.bytes > 2) {                                            // EV7_2
                I2C_AcknowledgeConfig(self->hw, DISABLE);                   // turn off ACK
                self->isr.read_p[self->isr.index++] = (uint8_t)self->hw->DR;                    // read data N-2
                I2C_GenerateSTOP(self->hw, ENABLE);                         // program the Stop
                self->isr.final_stop = 1;                                         // required to fix hardware
                self->isr.read_p[self->isr.index++] = (uint8_t)self->hw->DR;                    // read data N - 1
                I2C_ITConfig(self->hw, I2C_IT_BUF, ENABLE);                 // enable TXE to allow the final EV7
            } else {                                                    // EV7_3
                if (self->isr.final_stop)
                    I2C_GenerateSTOP(self->hw, ENABLE);                     // program the Stop
                else
                    I2C_GenerateSTART(self->hw, ENABLE);                    // program a rep start
                self->isr.read_p[self->isr.index++] = (uint8_t)self->hw->DR;                    // read data N - 1
                self->isr.read_p[self->isr.index++] = (uint8_t)self->hw->DR;                    // read data N
                self->isr.index++;                                                // to show job completed
            }
        } else {                                                        // EV8_2, which may be due to a subaddress sent or a write completion
            if (self->isr.subaddress_sent || (self->isr.writing)) {
                if (self->isr.final_stop)
                    I2C_GenerateSTOP(self->hw, ENABLE);                     // program the Stop
                else
                    I2C_GenerateSTART(self->hw, ENABLE);                    // program a rep start
                self->isr.index++;                                                // to show that the job is complete
            } else {                                                    // We need to send a subaddress
                I2C_GenerateSTART(self->hw, ENABLE);                        // program the repeated Start
                self->isr.subaddress_sent = 1;                                    // this is set back to zero upon completion of the current task
            }
        }
        // TODO - busy waiting in ISR
        // we must wait for the start to clear, otherwise we get constant BTF
        while (self->hw->CR1 & 0x0100) { ; }
    } else if (self->isr.status & 0x0040) {                                       // Byte received - EV7
        self->isr.read_p[self->isr.index++] = (uint8_t)self->hw->DR;
        if (self->isr.bytes == (self->isr.index + 3))
            I2C_ITConfig(self->hw, I2C_IT_BUF, DISABLE);                    // disable TXE to allow the buffer to flush so we can get an EV7_2
        if (self->isr.bytes == self->isr.index)                                             // We have completed a final EV7
            self->isr.index++;                                                    // to show job is complete
    } else if (status & 0x0080) {                                       // Byte transmitted EV8 / EV8_1
        if (self->isr.index != -1) {                                              // we dont have a subaddress to send
            self->hw->DR = self->isr.write_p[self->isr.index++];
            if (self->isr.bytes == self->isr.index)                                         // we have sent all the data
                I2C_ITConfig(self->hw, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        } else {
            self->isr.index++;
            self->hw->DR = self->isr.reg;                                             // send the subaddress
            if (self->isr.reading || !self->isr.bytes)                                      // if receiving or sending 0 bytes, flush now
                I2C_ITConfig(self->hw, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        }
    }
    if (self->isr.index == self->isr.bytes + 1) {                                           // we have completed the current job
        self->isr.subaddress_sent = 0;                                            // reset this here
        if (self->isr.final_stop)                                                 // If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
            I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_ERR, DISABLE);       // Disable EVT and ERR interrupts while bus inactive
        self->isr.busy = 0;
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

int _stm32_i2c_probe(void *fdt, int fdt_node){
    NVIC_InitTypeDef nvic;
    I2C_InitTypeDef i2c;

	I2C_TypeDef *I2Cx = (I2C_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);
	int speed = fdt_get_int_or_default(fdt, (int)fdt_node, "speed", 100000);
	int irq_prio = fdt_get_int_or_default(fdt, (int)fdt_node, "irq_prio", 1);

	if(I2Cx == 0) {
		return -EINVAL;
	}

    uint8_t irq_ev = 0, irq_er = 0;
    if(I2Cx == I2C1){
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        irq_ev = I2C1_EV_IRQn;
        irq_er = I2C1_ER_IRQn;
    } else if(I2Cx == I2C2){
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
        irq_ev = I2C2_EV_IRQn;
        irq_er = I2C2_ER_IRQn;
    }

    struct stm32_i2c *self = kzmalloc(sizeof(struct stm32_i2c));

    if(!self){
        return -ENOMEM;
    }

    // Turn on peripheral clock, save device and index
    self->hw = I2Cx;

    // diable I2C interrrupts first to avoid ER handler triggering
    I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

    // clock out stuff to make sure slaves arent stuck
    // This will also configure GPIO as AF_OD at the end
    _stm32_i2c_unstick(self);

    // Init I2C peripheral
    I2C_DeInit(self->hw);
    I2C_StructInit(&i2c);

    I2C_ITConfig(self->hw, I2C_IT_EVT | I2C_IT_ERR, DISABLE);               // Disable EVT and ERR interrupts - they are enabled by the first request
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = (uint32_t)speed;

    I2C_Cmd(self->hw, ENABLE);
    I2C_Init(self->hw, &i2c);

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

    return 0;
}
static int _stm32_i2c_remove(void *fdt, int fdt_node){
	// TODO
    return -1;
}

DEVICE_DRIVER(stm32_i2c, "st,stm32_i2c", _stm32_i2c_probe, _stm32_i2c_remove)
