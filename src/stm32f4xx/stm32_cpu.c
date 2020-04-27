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
* FILE ............... src/stm32f4xx/stm32_cpu.c
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
#include "driver.h"
#include "thread/thread.h"
#include <libfdt/libfdt.h>
#include <stm32f4xx/stm32f4xx_rcc.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"

#include "console.h"
#include "chip.h"

#ifndef configSYSTICK_CLOCK_HZ
	#define configSYSTICK_CLOCK_HZ configCPU_CLOCK_HZ
	/* Ensure the SysTick is clocked at the same frequency as the core. */
	#define portNVIC_SYSTICK_CLK_BIT	( 1UL << 2UL )
#else
	/* The way the SysTick is clocked is not modified in case it is not the same
	as the core. */
	#define portNVIC_SYSTICK_CLK_BIT	( 0 )
#endif

#define portNVIC_SYSTICK_CTRL_REG			( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG			( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG	( * ( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSTICK_CAL_REG			( * ( ( volatile uint32_t * ) 0xe000e01c ) )
#define portNVIC_SYSPRI2_REG				( * ( ( volatile uint32_t * ) 0xe000ed20 ) )
#define portNVIC_SYSTICK_INT_BIT			( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT			( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT		( 1UL << 16UL )
#define portNVIC_PENDSVCLEAR_BIT 			( 1UL << 27UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT		( 1UL << 25UL )

void vPortSetupTimerInterrupt( void )
{
	/* Calculate the constants required to configure the tick interrupt. */
	#if configUSE_TICKLESS_IDLE == 1
	{
#error "Not supported"
		ulTimerCountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
		xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
		ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
	}
	#endif /* configUSE_TICKLESS_IDLE */

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	SysTick_Config(clocks.HCLK_Frequency / (configTICK_RATE_HZ));
/*
	portNVIC_SYSTICK_CTRL_REG &= ~( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT );
	portNVIC_SYSTICK_LOAD_REG = ( clocks.SYSCLK_Frequency / (configTICK_RATE_HZ * 2)) - 1UL;
	portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT );
	*/
}

static int _stm32_cpu_probe(void *fdt, int fdt_node){
	uint32_t hclk_div = (uint32_t)fdt_get_int_or_default(fdt, (int)fdt_node, "hclk_div", 2);
	uint32_t pclk1_div = (uint32_t)fdt_get_int_or_default(fdt, (int)fdt_node, "pclk1_div", 2);
	uint32_t pclk2_div = (uint32_t)fdt_get_int_or_default(fdt, (int)fdt_node, "pclk2_div", 1);
	uint32_t pllm = (uint32_t)fdt_get_int_or_default(fdt, (int)fdt_node, "pllm", 8);
	uint32_t plln = (uint32_t)fdt_get_int_or_default(fdt, (int)fdt_node, "plln", 200);
	uint32_t pllp = (uint32_t)fdt_get_int_or_default(fdt, (int)fdt_node, "pllp", 2);
	uint32_t pllq = (uint32_t)fdt_get_int_or_default(fdt, (int)fdt_node, "pllq", 4);
	//uint32_t hse_value = (uint32_t)fdt_get_int_or_default(fdt, (int)fdt_node, "hse_value", 8000000);
	int ready = 0;
	int timeout = 5;

	RCC->CR |= ((uint32_t)RCC_CR_HSEON);

	do {
		ready = RCC->CR & RCC_CR_HSERDY;
		timeout--;
		thread_sleep_ms(10);
	} while(!ready && timeout);

	if(ready) {
		RCC->APB1ENR |= RCC_APB1ENR_PWREN;
		PWR->CR |= PWR_CR_VOS;

        hclk_div = (hclk_div == 1)?0:((uint32_t)(0x80 | (((uint16_t)ffs((int)hclk_div) - 2) << 4)));
		pclk1_div = (pclk1_div == 1)?0:((uint32_t)(0x1000 | (((uint16_t)ffs((int)pclk1_div) - 2) << 10)));
		pclk2_div = (pclk2_div == 1)?0:((uint32_t)(0x8000 | (((uint16_t)ffs((int)pclk2_div) - 2) << 13)));

		RCC->CFGR |= hclk_div | pclk1_div | pclk2_div;
		RCC->PLLCFGR = pllm | (plln << 6) | (((pllp >> 1) -1) << 16) | RCC_PLLCFGR_PLLSRC_HSE | (pllq << 24);

		RCC->CR |= RCC_CR_PLLON;

		while((RCC->CR & RCC_CR_PLLRDY) == 0) { }

		FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_3WS;

		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_PLL;

		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL){}
	} else {
		RCC->CR |= ((uint32_t)RCC_CR_HSION);                     // Enable HSI
		while ((RCC->CR & RCC_CR_HSIRDY) == 0);                  // Wait for HSI Ready

		RCC->CFGR = RCC_CFGR_SW_HSI;                             // HSI is system clock
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Wait for HSI used as system clock

		FLASH->ACR  = FLASH_ACR_PRFTEN;                          // Enable Prefetch Buffer
		FLASH->ACR |= FLASH_ACR_ICEN;                            // Instruction cache enable
		FLASH->ACR |= FLASH_ACR_DCEN;                            // Data cache enable
		FLASH->ACR |= FLASH_ACR_LATENCY_5WS;                     // Flash 5 wait state

		RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // HCLK = SYSCLK
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;                        // APB1 = HCLK/4
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;                        // APB2 = HCLK/2

		RCC->CR &= ~RCC_CR_PLLON;                                // Disable PLL

		// PLL configuration:  VCO = HSI/M * N,  Sysclk = VCO/P
		RCC->PLLCFGR = ( 16ul                   |                // PLL_M =  16
					 (192ul <<  6)            |                // PLL_N = 384
					 (  2ul << 16)            |                // PLL_P =   8
					 (RCC_PLLCFGR_PLLSRC_HSI) |                // PLL_SRC = HSI
					 (  4ul << 24)             );              // PLL_Q =   8

		RCC->CR |= RCC_CR_PLLON;                                 // Enable PLL
		while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();           // Wait till PLL is ready

		RCC->CFGR &= ~RCC_CFGR_SW;                               // Select PLL as system clock source
		RCC->CFGR |=  RCC_CFGR_SW_PLL;
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait till PLL is system clock src
	}

	vPortSetupTimerInterrupt();

	return 0;
}

static int _stm32_cpu_remove(void *fdt, int fdt_node){
	return 0;
}

DEVICE_DRIVER(stm32_cpu, "st,stm32_cpu", _stm32_cpu_probe, _stm32_cpu_remove)

/* add console command for showing cpu info */

static int _stm32_cmd_cpuinfo(console_device_t con, void *userptr, int argc, char **argv){
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	int pllm = (RCC->PLLCFGR) & 0x1f;
	int plln = (RCC->PLLCFGR >> 6) & 0x1ff;
	int pllp = (RCC->PLLCFGR >> 16) & 0x3;
	int pllsrc = (RCC->PLLCFGR >> 22) & 0x1;
	int pllq = (RCC->PLLCFGR >> 24) & 0xf;
	int pllr = (RCC->PLLCFGR >> 28) & 0x7;
	switch(pllp){
		case 0: pllp = 2; break;
		case 1: pllp = 4; break;
		case 2: pllp = 6; break;
		case 3: pllp = 8; break;
	}
	console_printf(con, "RCC: pllm: %d, plln: %d, pllp: %d, pllsrc = %d, pllq = %d, pllr = %d\n", pllm, plln, pllp, pllsrc, pllq, pllr);
	console_printf(con, "CPU clock source: %s\n", (RCC->CR & RCC_CR_HSERDY)?"HSE":"HSI");
	console_printf(con, "PLL clock source: %s\n", (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC)?"HSE":"HSI");
	console_printf(con, "Processor clock speed: %d (%s)\n", time_get_clock_speed(), (time_cpu_clock_speed_exact())?"exact":"inexact");
	console_printf(con, "SYSCLK: %d, HCLK: %d, PCLK1: %d, PCLK2: %d, ",
			clocks.SYSCLK_Frequency,
			clocks.HCLK_Frequency,
			clocks.PCLK1_Frequency,
			clocks.PCLK2_Frequency);
	console_printf(con, "USB: %d\n", (int)((int)clocks.SYSCLK_Frequency * pllp / pllq));
	console_printf(con, "SysTick reload value: %d\n", (SysTick->LOAD & 0xffffff) + 1);
	return 0;
}

static int _stm32_cmd_reboot(console_device_t con, void *userptr, int argc, char **argv){
	(void)argc;
	(void)argv;

	if(argc == 2 && strcmp(argv[1], "b") == 0){
		console_printf(con, "Rebooting to bootloader..\n");
		thread_sleep_ms(100);
		chip_reset_to_bootloader();
	} else if(argc == 1){
		chip_reset();
	} else {
		console_printf(con, "invalid argument!\n");
	}

	return 0;
}


static int _stm32_cpuinfo_probe(void *fdt, int fdt_node){
	int node = fdt_find_node_by_ref(fdt, fdt_node, "console");
	if(node < 0){
		printk("cpuinfo: no console device\n");
		return -1;
	}
	console_device_t con = console_find_by_node(fdt, node);
	if(!con){
		printk("cpuinfo: console not found\n");
		return -1;
	}
	
	console_add_command(con, NULL, "cpuinfo", "show cpu info", "", _stm32_cmd_cpuinfo);
	console_add_command(con, NULL, "reboot", "reboot cpu", "", _stm32_cmd_reboot);

	return 0;
}

static int _stm32_cpuinfo_remove(void *fdt, int fdt_node){
	return -1;
}

DEVICE_DRIVER(stm32_cpuinfo, "st,stm32_cpuinfo", _stm32_cpuinfo_probe, _stm32_cpuinfo_remove)

