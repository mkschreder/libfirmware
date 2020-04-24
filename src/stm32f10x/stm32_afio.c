#include <stm32f10x.h>
#include <misc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

#include <errno.h>

#include <libfdt/libfdt.h>

#include "driver.h"

static int _stm32_afio_probe(void *fdt, int fdt_node) {
	uint32_t mapr = (uint32_t)fdt_get_int_or_default(fdt, (int)fdt_node, "mapr", 0);

	printk("afio: init\n");

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	AFIO->MAPR |= mapr;

	return 0;
}

static int _stm32_afio_remove(void *fdt, int fdt_node) {
	// TODO
	return 0;
}

DEVICE_DRIVER(stm32_afio, "st,stm32_afio", _stm32_afio_probe, _stm32_afio_remove)
