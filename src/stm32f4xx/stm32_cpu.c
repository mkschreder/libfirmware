#include "driver.h"
#include "thread.h"
#include <libfdt/libfdt.h>
#include <stm32f4xx/stm32f4xx_rcc.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"

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
		ulTimerCountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
		xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
		ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
	}
	#endif /* configUSE_TICKLESS_IDLE */

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	/* Configure SysTick to interrupt at the requested rate. */
	portNVIC_SYSTICK_CTRL_REG &= ~( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT );
	portNVIC_SYSTICK_LOAD_REG = ( clocks.HCLK_Frequency / configTICK_RATE_HZ ) - 1UL;
	portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT );
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
