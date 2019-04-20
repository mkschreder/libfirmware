#include <errno.h>

#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>

#include "timestamp.h"

// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = (uint32_t)(-20000);

/*
void time_init(void){
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	usTicks = clocks.SYSCLK_Frequency / 1000000;
    SysTick_Config(SystemCoreClock / 1000);
}

*/

void vApplicationTickHook(void){
	__sync_fetch_and_add(&sysTickUptime, 1);
    //sysTickUptime++;
}

// Return system uptime in microseconds (rollover in 70minutes)
timestamp_t micros(void){
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	uint32_t usTicks = clocks.SYSCLK_Frequency / 1000000;

    register uint32_t ticks;
	register uint32_t cycle_cnt;
    do {
        ticks = sysTickUptime;
        cycle_cnt = SysTick->VAL;

        /*
         * If the SysTick timer expired during the previous instruction, we need to give it a little time for that
         * interrupt to be delivered before we can recheck sysTickUptime:
         */
        asm volatile("\tnop\n");
    } while (ticks != sysTickUptime);
	// micros = tick_period_counts + count / counts_per_us;
    return (timestamp_t)(((uint64_t)ticks * SysTick->LOAD + cycle_cnt) / usTicks);
}

void time_gettime(struct timeval *ts){
	// this is a dummy
	timestamp_t t = micros();
	ts->tv_sec = (time_t)(t / 1000000);
	ts->tv_usec = (time_t)(t % 1000000);
}

int time_init_rtc(){
	// TODO
	return -1;
}

