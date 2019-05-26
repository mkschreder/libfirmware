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
* FILE ............... src/stm32f4xx/chip.c
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
#include <stm32f4xx.h>
#include "chip.h"

// Activate the bootloader without BOOT* pins.
void chip_reset_to_bootloader(void) {
	RCC_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	__set_PRIMASK(1);
	__set_MSP(0x20001000);

	((void (*)(void)) *((uint32_t*)0x1fff0004))();

    while (1);
}

void chip_reset(void){
	__DSB();                                                     /* Ensure all outstanding memory accesses included
															  buffered write are completed before reset */
	SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)      |
				 (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
				 SCB_AIRCR_SYSRESETREQ_Msk);                   /* Keep priority group unchanged */
	__DSB();                                                     /* Ensure completion of memory access */
	while(1);                                                    /* wait until reset */
}

int chip_get_uuid(uint32_t id[3]){
	volatile uint16_t *id0 = (volatile uint16_t*)0x1FFF7A10;
	volatile uint16_t *id1 = (volatile uint16_t*)0x1FFF7A10 + 0x02;
	volatile uint16_t *id2 = (volatile uint16_t*)0x1FFF7A10 + 0x04;
	volatile uint16_t *id3 = (volatile uint16_t*)0x1FFF7A10 + 0x06;
	volatile uint16_t *id4 = (volatile uint16_t*)0x1FFF7A10 + 0x08;
	volatile uint16_t *id5 = (volatile uint16_t*)0x1FFF7A10 + 0x0a;
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
	uint16_t *fsz = (uint16_t*)0x1FFF7A22;
	return *fsz;
}

uint32_t chip_get_data_size(void){
	extern uint8_t _sdata;
	extern uint8_t _edata;
	extern uint8_t _sbss;
	extern uint8_t _ebss;

	uint8_t *s = &_sdata;
	uint8_t *e = &_edata;
	uint8_t *ss = &_sbss;
	uint8_t *es = &_ebss;
	return (uint32_t)((e - s) + (es - ss));
}


