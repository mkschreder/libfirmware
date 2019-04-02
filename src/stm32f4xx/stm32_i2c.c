#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_i2c.h>
#include <string.h>
#include <stdio.h>

#include "i2c.h"
#include <libfdt/libfdt.h>

#include "driver.h"
#include "thread.h"
#include "queue.h"
#include "sem.h"
#include "work.h"

//#include "timer.h"
#include "mutex.h"
#include "atomic.h"

#include <errno.h>

struct stm32_i2c {
	struct i2c_device dev;
	I2C_TypeDef *hw;
};
#if 0
int _stm32_i2c_read(i2c_device_t dev, uint8_t address, uint8_t reg, const void *data, size_t len) {
	struct stm32_i2c *self = container_of(dev, struct stm32_i2c, dev.ops);

	while(self->hw->SR2 & I2C_SR2_BUSY);		// Wait for BUSY line
	self->hw->CR1 |= I2C_CR1_START;				// Generate START condition

	while (!(self->hw->SR1 & I2C_SR1_SB)); 		// Wait for EV5
	self->hw->DR = address<<1;					// Write device address (W)
    (void)self->hw->SR2;						// Read SR2

	while (!(self->hw->SR1 & I2C_SR1_TXE));		// Wait for EV8_1
	self->hw->DR = registry;

	self->hw->CR1 |= I2C_CR1_STOP;				// Generate STOP condition

	self->hw->CR1 |= I2C_CR1_START;				// Generate START condition

	while (!(self->hw->SR1 & I2C_SR1_SB)); 		// Wait for EV5
	self->hw->DR = (address << 1 ) | 1;			// Write device address (R)

	while (!(self->hw->SR1 & I2C_SR1_ADDR));	// Wait for EV6
    self->hw->CR1 &= ~I2C_CR1_ACK;              // No ACK
    (void)self->hw->SR2;						// Read SR2

	while (!(self->hw->SR1 & I2C_SR1_RXNE));	// Wait for EV7_1
    uint8_t value = (uint8_t)self->hw->DR;      // Read value

    self->hw->CR1 |= I2C_CR1_STOP;			    // Generate STOP condition

	return value;
}
#endif
int _stm32_i2c_read(i2c_device_t dev, uint8_t addr, const void *wr_data, size_t wr_len, void *data, size_t len) {
	struct stm32_i2c *self = container_of(dev, struct stm32_i2c, dev.ops);
	I2C_TypeDef *I2Cx = self->hw;
	uint8_t *result = (uint8_t*)data;
	uint8_t *wr_buf = (uint8_t*)wr_data;

	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) != RESET);		// Wait for BUSY line

	I2C_GenerateSTART(I2Cx, ENABLE);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2Cx, (uint8_t)(addr << 1), I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	// write header with secondary address and any other command bytes
	for(size_t c = 0; c < wr_len; c++){
		I2C_SendData(I2Cx, *wr_buf++);

		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}

	I2C_GenerateSTOP(I2Cx, ENABLE);

	I2C_GenerateSTART(I2Cx, ENABLE);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2Cx, (uint8_t)(addr << 1), I2C_Direction_Receiver);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	for(size_t c = 0; c < len - 1; c++){
		I2C_AcknowledgeConfig(I2Cx, ENABLE);
		I2C_GenerateSTOP(I2Cx, DISABLE);
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
		*result++ = I2C_ReceiveData(I2Cx);
	}

	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
	*result++ = I2C_ReceiveData(I2Cx);

	return 0;
}

static int _stm32_i2c_write(i2c_device_t dev, uint8_t addr, const void *wr_data, size_t wr_len, const void *data, size_t len){
	struct stm32_i2c *self = container_of(dev, struct stm32_i2c, dev.ops);
	I2C_TypeDef *I2Cx = self->hw;
	const uint8_t *buf = (uint8_t*)data;
	const uint8_t *wr_buf = (uint8_t*)wr_data;

	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) != RESET);		// Wait for BUSY line

	I2C_GenerateSTART(I2Cx, ENABLE);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2Cx, (uint8_t)(addr << 1), I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	for(size_t c = 0; c < wr_len; c++){
		I2C_SendData(I2Cx, *wr_buf++);
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}

	for(size_t c = 0; c < len; c++){
		I2C_SendData(I2Cx, *buf++);
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}

	I2C_GenerateSTOP(I2Cx, ENABLE);

	return 0;
}

static struct i2c_device_ops _i2c_ops = {
	.read = _stm32_i2c_read,
	.write = _stm32_i2c_write
};

static int _stm32_i2c_probe(void *fdt, int fdt_node) {
	I2C_TypeDef *I2Cx = (I2C_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "reg", 0);
	uint32_t baud = (uint32_t)fdt_get_int_or_default(fdt, (int)fdt_node, "baud", 100000);
	int idx = 0;
	if(I2Cx == I2C1){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
		idx = 1;
	} else if(I2Cx == I2C2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
		idx = 2;
	} else if(I2Cx == I2C3){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C3, ENABLE);
		idx = 3;
	} else {
		printk("i2c: unsupported device\n");
		return -1;
	}

	thread_sleep_ms(50);

	I2C_DeInit(I2Cx);
	I2C_InitTypeDef i2c;
    I2C_StructInit(&i2c);

    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = (uint32_t)baud;
	i2c.I2C_OwnAddress1 = 0;

    I2C_Cmd(I2Cx, ENABLE);
    I2C_Init(I2Cx, &i2c);

	I2C_GenerateSTART(I2Cx, ENABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);

	struct stm32_i2c *self = kzmalloc(sizeof(struct stm32_i2c));
	self->hw = I2Cx;
	i2c_device_init(&self->dev, fdt, fdt_node, &_i2c_ops);
	i2c_device_register(&self->dev);
	printk("i2c%d: ready (speed %d)\n", idx, baud);

	thread_sleep_ms(50);

	return 0;
}

static int _stm32_i2c_remove(void *fdt, int fdt_node){
	return -1;
}

DEVICE_DRIVER(stm32_i2c, "st,stm32_i2c", _stm32_i2c_probe, _stm32_i2c_remove)

