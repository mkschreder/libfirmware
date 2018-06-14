#include <errno.h>

#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>

#include "timestamp.h"

// cycles per microsecond
static uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;

void time_init(void){
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	usTicks = clocks.SYSCLK_Frequency / 1000000;
    SysTick_Config(SystemCoreClock / 1000);
}

void vApplicationTickHook(void){
	__sync_fetch_and_add(&sysTickUptime, 1);
    //sysTickUptime++;
}

// Return system uptime in microseconds (rollover in 70minutes)
timestamp_t micros(void){
    register uint32_t ms;
	register uint32_t cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;

        /*
         * If the SysTick timer expired during the previous instruction, we need to give it a little time for that
         * interrupt to be delivered before we can recheck sysTickUptime:
         */
        asm volatile("\tnop\n");
    } while (ms != sysTickUptime);
    return (timestamp_t)((ms * 1000U) + (usTicks * 1000U - cycle_cnt) / usTicks);
}

void time_gettime(struct timeval *ts){
	// TODO: this is just dummy. Need to use rtc.
	ts->tv_sec = 0;
	ts->tv_usec = (suseconds_t)micros();
}

int time_init_rtc(){
	// TODO
	return -1;
}

