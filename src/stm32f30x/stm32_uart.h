#pragma once

#include <stdint.h>
#include <stddef.h>

#include <firmware/serial.h>

struct stm32_uart {
	// leave this at the top!
	const struct serial_ops* serial_ops;

	void *driver_data;
};

int stm32_uart_init(struct stm32_uart *self, uint8_t hw_idx);
int stm32_uart_configure(struct stm32_uart *self, uint32_t baud, uint8_t data_bits, uint8_t parity);
serial_port_t stm32_uart_get_serial_interface(struct stm32_uart *self);

//int uart_write(struct uart *self, const void *data, size_t size, uint32_t timeout_ms);
//int uart_read(struct uart *self, void *data, size_t size, uint32_t timeout_ms);
