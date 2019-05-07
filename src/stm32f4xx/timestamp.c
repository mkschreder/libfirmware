#include <errno.h>

#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>

#include "timestamp.h"

// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
// force overflow ~10s after boot
static volatile uint32_t _sys_ticks = 0;

/*
void time_init(void){
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	usTicks = clocks.SYSCLK_Frequency / 1000000;
    SysTick_Config(SystemCoreClock / 1000);
}

*/

void vApplicationTickHook(void){
	__sync_fetch_and_add(&_sys_ticks, 1);
}

static void _get_ticks(uint32_t *_ticks, uint32_t *_cycles){
    register uint32_t ticks;
	register uint32_t cycle_cnt;
    do {
        ticks = _sys_ticks;
        cycle_cnt = SysTick->VAL;

        /*
         * If the SysTick timer expired during the previous instruction, we need to give it a little time for that
         * interrupt to be delivered before we can recheck _sys_ticks:
         */
        asm volatile("\tnop\n");
    } while (ticks != _sys_ticks);
    *_ticks = ticks;
    *_cycles = cycle_cnt;
}

// Return system uptime in microseconds (rollover in 70minutes)
usec_t micros(void){
    uint32_t ticks = 0;
	uint32_t cycle_cnt = 0;

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	uint32_t usTicks = clocks.SYSCLK_Frequency / 1000000;

	_get_ticks(&ticks, &cycle_cnt);

	// micros = tick_period_counts + count / counts_per_us;
    return (usec_t)(((uint64_t)ticks * SysTick->LOAD + cycle_cnt) / usTicks);
}

// TODO: fix the 49 days overflow in some smart way that doesn't require locks
timestamp_t timestamp(void){
    uint32_t sched_ticks = 0;
	uint32_t cycle_cnt = 0;

	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	uint32_t usTicks = clocks.SYSCLK_Frequency / 1000000;

	_get_ticks(&sched_ticks, &cycle_cnt);

	uint64_t us = ((uint64_t)sched_ticks * SysTick->LOAD + cycle_cnt) / usTicks;

	timestamp_t r = (timestamp_t) {
		.sec = (sec_t)(us / 1000000),
		.usec = (usec_t)(us % 1000000)
	};

	return r;
}

void time_gettime(struct timeval *ts){
	// this is a dummy
	usec_t t = micros();
	ts->tv_sec = (time_t)(t / 1000000);
	ts->tv_usec = (time_t)(t % 1000000);
}

int time_init_rtc(){
	// TODO
	return -1;
}

