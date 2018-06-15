#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

#include <libfdt/libfdt.h>

#include "driver.h"

static int _stm32_gpio_setup_subnode(void *fdt, int fdt_node){
    int len = 0;
    const fdt32_t *val = (const fdt32_t*)fdt_getprop(fdt, fdt_node, "pinctrl", &len);

    if(len == 0 || !val) return -1;

    int pin_count = (uint8_t)(len / 4 / 3);

    for(int c = 0; c < pin_count; c++){
        const fdt32_t *base = val + (3 * c);
        GPIO_TypeDef *GPIOx = (GPIO_TypeDef*)fdt32_to_cpu(*(base));
        uint16_t pin = (uint16_t)fdt32_to_cpu(*(base + 1));
        uint32_t opts = (uint32_t)fdt32_to_cpu(*(base + 2));

        GPIO_InitTypeDef gpio;
        GPIO_StructInit(&gpio);
        gpio.GPIO_Pin = pin;
        gpio.GPIO_Mode = (opts) & 0xff;
        gpio.GPIO_Speed = (opts >> 8) & 0x3;
        if(!gpio.GPIO_Speed) gpio.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOx, &gpio);

        /*
        uint16_t idx = (uint16_t)ffs(pin);
        if(gpio.GPIO_Mode == GPIO_Mode_AF && idx != 0){
            GPIO_PinAFConfig(GPIOx, (uint16_t)(idx - 1), (opts & 0xf));
        }
        */
    }
    dbg_printk("gpio: ok, %d pins\n", pin_count);
    return 0;
}

static int _stm32_gpio_probe(void *fdt, int fdt_node){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // check if we directly have a pinmux here
    int len = 0;
	const fdt32_t *val = (const fdt32_t*)fdt_getprop(fdt, fdt_node, "pinctrl", &len);
    if(val && len > 0){
        if(_stm32_gpio_setup_subnode(fdt, fdt_node) < 0){
            return -1;
        }
    }

    // otherwise scan all children
	int node;
	fdt_for_each_subnode(node, fdt, fdt_node){
        if(_stm32_gpio_setup_subnode(fdt, node) < 0){
            return -1;
        }
	}

	return 0;
}

static int _stm32_gpio_remove(void *fdt, int fdt_node){
	// TODO
    return 0;
}

DEVICE_DRIVER(stm32_gpio, "st,stm32_gpio", _stm32_gpio_probe, _stm32_gpio_remove)
