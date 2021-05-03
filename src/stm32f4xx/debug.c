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
* FILE ............... src/stm32f4xx/debug.c
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
/**
 * CoreDebug management module.
 *
 * Author: Martin Schröder <mkschreder.uk@gmail.com>
 *
 * This module enables the CoreDebug ARM built-in debugging functionality such
 * that you can get a detailed debug trace on the SWO pin including exceptions,
 * data write (hardware breakpoints) and arbitrary trace messages. This uses
 * ARM cortex debugging features described in detail here:
 * http://infocenter.arm.com/help/topic/com.arm.doc.ihi0029d/IHI0029D_coresight_architecture_spec_v2_0.pdf.
 *
 * Note that debug output is started once you have flashed over st-link. TODO:
 * it seems to be impossible to enable debug trace without connecting a
 * debugger. (DEMCR writes would fail or something). This kindof sucks..
 */

#include "debug.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "arm_etm.h"
#include <stddef.h>

#if 0
void configure_watchpoint()
{
    /* This is an example of how to configure DWT to monitor a watchpoint.
       The data value is reported when the watchpoint is hit. */
    /* Monitor all accesses to GPIOC (range length 32 bytes) */
    DWT->COMP0 = (uint32_t)GPIOC;
    DWT->MASK0 = 5;
    DWT->FUNCTION0 = (2 << DWT_FUNCTION_FUNCTION_Pos) // Report data and addr on watchpoint hit
                   | (1 << DWT_FUNCTION_EMITRANGE_Pos);
    /* Monitor all accesses to globalCounter (range length 4 bytes) */
    DWT->COMP1 = (uint32_t)&globalCounter;
    DWT->MASK1 = 2;
    DWT->FUNCTION1 = (3 << DWT_FUNCTION_FUNCTION_Pos); // Report data and PC on watchpoint hit
}
#endif

void swo_init() {
	// enable debuggint. Note that there does not seem to be a way to enable this
	// unless debugger is connected.
	DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins

	uint32_t baud = 921600; // 6000kbps, default for JLinkSWOViewer

	// enable core debug trace
	CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk;

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	// set tpi baud rate
	TPI->ACPR = (clocks.SYSCLK_Frequency / baud) - 1;
	TPI->SPPR = 2;     // Pin protocol = NRZ/USART
	TPI->FFCR = 0x100; // TPIU packet framing enabled when bit 2 is set.
	                   // You can use 0x100 if you only need DWT/ITM and not ETM.

	// unlock the itm registers
	ITM->LAR = 0xC5ACCE55;
	//*((volatile unsigned *)(ITM_BASE + 0x00FB0)) = 0xC5ACCE55; // ITM Lock Access
	//Register, C5ACCE55 enables more write access to Control Register 0xE00 :: 0xFFC
	ITM->TCR = ITM_TCR_TraceBusID_Msk | // enable trace bus ids
	           ITM_TCR_SWOENA_Msk |     // enable swo
	           ITM_TCR_DWTENA_Msk |     // enable dwt output
	           ITM_TCR_SYNCENA_Msk |    // enable sync packets
	           ITM_TCR_ITMENA_Msk;      // Main enable for itm

	ITM->TPR = ITM_TPR_PRIVMASK_Msk; // ITM Trace Privilege Register
	ITM->TER = 0xFFFFFFFF;           // Enable all stimulus ports
	//*((volatile unsigned *)(ITM_BASE + 0x01000)) = 0x400003FE; // DWT_CTRL
	//*((volatile unsigned *)(ITM_BASE + 0x40304)) = 0x00000100; // Formatter and Flush
	//Control Register

	DWT->CTRL = (1 << DWT_CTRL_CYCTAP_Pos)       // Prescaler for PC sampling
	                                             // 0 = x32, 1 = x512
	            | (0 << DWT_CTRL_POSTPRESET_Pos) // Postscaler for PC sampling
	                                             // Divider = value + 1
	            //| (1 << DWT_CTRL_PCSAMPLENA_Pos) // Enable PC sampling
	            | (2 << DWT_CTRL_SYNCTAP_Pos) // Sync packet interval
	                                          // 0 = Off, 1 = Every 2^23 cycles,
	                                          // 2 = Every 2^25, 3 = Every 2^27
	            //| (1 << DWT_CTRL_EXCTRCENA_Pos)  // Enable exception trace
	            | (1 << DWT_CTRL_CYCCNTENA_Pos); // Enable cycle counter
	;

	// unlock ETM
	ETM->LAR = 0xC5ACCE55;
	ETM_SetupMode();
	ETM->CR = ETM_CR_ETMEN             // Enable ETM output port
	          | ETM_CR_STALL_PROCESSOR // Stall processor when fifo is full
	          | ETM_CR_BRANCH_OUTPUT;  // Report all branches
	;
	ETM->TRACEIDR = 2;              // Trace bus ID for TPIU
	ETM->TECR1 = ETM_TECR1_EXCLUDE; // Trace always enabled
	ETM->FFRR = ETM_FFRR_EXCLUDE;   // Stalling always enabled
	ETM->FFLR = 24; // Stall when less than N bytes free in FIFO (range 1..24)
	                // Larger values mean less latency in trace, but more stalls.

	// An example of how to monitor uart data register.
	DWT->COMP0 = (uint32_t)USART1 + offsetof(USART_TypeDef, DR);
	DWT->MASK0 = 0;
	DWT->FUNCTION0 =
	    (2 << DWT_FUNCTION_FUNCTION_Pos) | // Report data and addr on watchpoint hit
	    (1 << DWT_FUNCTION_EMITRANGE_Pos);
}

// may not be needed
void swo_deinit() {
	volatile uint32_t i = 0xFFF;
	while(ITM->PORT[0].u32 == 0 && i--)
		; // wait for any pending transmission
	CoreDebug->DEMCR = 0;
	*((volatile unsigned *)(ITM_BASE + 0x00FB0)) =
	    0xC5ACCE55; // ITM Lock Access Register, C5ACCE55 enables more write access to
	                // Control Register 0xE00 :: 0xFFC
	ITM->TCR = 0;
	ITM->TPR = 0;
	ITM->TER = 0;
}

void swo_etm_start(void) {
	ETM_TraceMode();
}

void swo_etm_stop(void) {
	ETM_SetupMode();
}

// function mostly copied from ITM_SendChar
void swo_sendchar(unsigned port, int x) {
	// Other tests
	//(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) &&    // if debugger is
	//attached (does it really work?) (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)
	//&&      // Trace enabled
	if((ITM->TCR & ITM_TCR_ITMENA_Msk) && /* ITM enabled */
	   (ITM->TER & (1UL << port))         /* ITM Port #0 enabled */
	) {
		while(ITM->PORT[port].u32 == 0)
			;                              /* Wait for available */
		ITM->PORT[port].u8 = (uint8_t)x; /* Send character */
	}
}
