#include <stm32f10x.h>

#include "chip.h"

int chip_get_uuid(uint32_t id[3]){
	volatile uint16_t *id0 = (volatile uint16_t*)0x1FFFF7E8;
	volatile uint16_t *id1 = (volatile uint16_t*)0x1FFFF7E8 + 0x02;
	volatile uint16_t *id2 = (volatile uint16_t*)0x1FFFF7E8 + 0x04;
	volatile uint16_t *id3 = (volatile uint16_t*)0x1FFFF7E8 + 0x06;
	volatile uint16_t *id4 = (volatile uint16_t*)0x1FFFF7E8 + 0x08;
	volatile uint16_t *id5 = (volatile uint16_t*)0x1FFFF7E8 + 0x0a;
	id[2] = (((uint32_t)*id5) << 16) + *id4;
	id[1] = (((uint32_t)*id3) << 16) + *id2;
	id[0] = (((uint32_t)*id1) << 16) + *id0;
	return 0;
}

uint32_t chip_get_device_id(void){
	uint32_t *id0 = (uint32_t*)0xE0042000;
	return *id0; 
}

uint16_t chip_get_flash_size_k(void){
	uint16_t *fsz = (uint16_t*)0x1FFFF7E0;
	return *fsz;
}

uint32_t chip_get_ram_total(void){
	return 0;
}

extern uint8_t _sdata;
extern uint8_t _edata;
extern uint8_t _sbss;
extern uint8_t _ebss;

uint32_t chip_get_data_size(void){
	uint8_t *s = &_sdata;
	uint8_t *e = &_edata;
	uint8_t *ss = &_sbss;
	uint8_t *es = &_ebss;
	return (uint32_t)((e - s) + (es - ss));
}

static void _prepare_reset(void){
	RCC_DeInit();
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	GPIO_DeInit(GPIOD);
	USART_DeInit(USART1);
	USART_DeInit(USART2);
	USART_DeInit(USART3);
}

// Activate the bootloader without BOOT* pins.
void chip_reset_to_bootloader(void) {
	__disable_irq();

	_prepare_reset();

	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	__set_PRIMASK(1);
	//__set_MSP(0x1FFFF000);
	__set_MSP(0x20001000);

	//((void (*)(void)) *((uint32_t*)0x1fff0004))();
	((void (*)(void)) *((uint32_t*)0x1FFFF004))();

    while (1);
}

void chip_reset(void){
	__DSB();                                                     /* Ensure all outstanding memory accesses included
															  buffered write are completed before reset */
	__disable_irq();

	_prepare_reset();

	SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)      |
				 (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
				 SCB_AIRCR_SYSRESETREQ_Msk);                   /* Keep priority group unchanged */
	__DSB();                                                     /* Ensure completion of memory access */
	while(1);                                                    /* wait until reset */
}


