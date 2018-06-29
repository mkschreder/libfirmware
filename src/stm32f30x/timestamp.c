#include <thread/thread.h>
#include <errno.h>

#include <stm32f30x_tim.h>
#include <stm32f30x_rcc.h>

#include <time.h>
#include <firmware/time.h>
#include <firmware/math.h>

// cycles per microsecond
static uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t _ms = 0;
static volatile uint32_t _rtc_calib = 0;
static volatile struct timeval _current_time;

void vApplicationTickHook(void);
void RTC_IRQHandler(void);

void time_init(void){
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
    SysTick_Config(clocks.HCLK_Frequency / 1000);
	usTicks = clocks.SYSCLK_Frequency / 1000000;
}

static int _rtc_wait_for_flag(uint8_t flag){
	for(volatile int t = 0; t < 10000000; t++){
		if(RCC_GetFlagStatus(flag) != RESET){
			return 0;
		}
	}
	return -1;
}

static uint32_t _rtc_get_time(void){
	RTC_TimeTypeDef time;
	RTC_GetTime(RTC_Format_BIN, &time);
	
	return (uint32_t)(time.RTC_Hours * 3600 + time.RTC_Minutes * 60 + time.RTC_Seconds);
}

int time_init_rtc(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_BackupAccessCmd(ENABLE); //Allow access to BKP Domain

	//BKP_DeInit();

	RCC_LSEConfig(RCC_LSE_ON); /* Enable LSE Enable LSE */
	// use LSI by default but try to start LSE if it is detected
	uint32_t clk_source = RCC_RTCCLKSource_LSI;
	if(_rtc_wait_for_flag(RCC_FLAG_LSERDY) == 0){
		clk_source = RCC_RTCCLKSource_LSE;
	} else {
		RCC_LSEConfig(RCC_LSE_OFF);
		return -1;
	}

	RCC_RTCCLKConfig(clk_source); /* Select LSE as RTC Clock Source */
	RCC_RTCCLKCmd(ENABLE); /* Enable RTC Clock */

	// wait until write is finished
	if(RTC_WaitForSynchro() == ERROR)
		return -3;

	RTC_ITConfig(RTC_IT_ALRA, ENABLE); /* Enable the RTC Second */

	if(RTC_WaitForSynchro() == ERROR)
		return -4;

	RTC_InitTypeDef rtc;
	rtc.RTC_AsynchPrediv = 0x7f;
	rtc.RTC_SynchPrediv = 0xff;
	rtc.RTC_HourFormat = RTC_HourFormat_24;
	RTC_Init(&rtc);

	// start with a reasonable time
	_current_time.tv_sec = (time_t)_rtc_get_time(); 
	_current_time.tv_usec = 0;

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = RTC_Alarm_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	return 0;
}

void vApplicationTickHook(void){
	__sync_fetch_and_add(&_ms, 1);
}

// Return system uptime in microseconds (rollover in 70minutes)
timestamp_t micros(void){
    register uint32_t ms, cycle_cnt;
    do {
        ms = _ms;
        cycle_cnt = SysTick->VAL;

        /*
         * If the SysTick timer expired during the previous instruction, we need to give it a little time for that
         * interrupt to be delivered before we can recheck _ms:
         */
        asm volatile("\tnop\n");
    } while (ms != _ms);
    usec_t usec = (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
	return usec;
}

void time_gettime(struct timeval *ts){
	RTC_ITConfig(RTC_IT_ALRA, DISABLE);

	// scale the elapsed micros against the measured length of one second and clamp the result to valid microsecond value
	ts->tv_sec = _current_time.tv_sec;
	ts->tv_usec = (suseconds_t)constrain_u32(
		(uint32_t)((int64_t)((int32_t)micros() - _current_time.tv_usec) * 1000000) / _rtc_calib,
		0, 999999);

	RTC_ITConfig(RTC_IT_ALRA, ENABLE);
}

uint32_t time_get_hse_precision(void){
	return _rtc_calib;
}

void RTC_Alarm_IRQHandler(void){
	if(RTC_GetITStatus(RTC_IT_ALRA) != RESET){
		RTC_ClearITPendingBit(RTC_IT_ALRA); /* Clear the RTC Second interrupt */
		_current_time.tv_sec = (time_t)_rtc_get_time();

		// TODO: figure our if using micros here is any good.
		usec_t us = micros();
		_rtc_calib = (uint32_t)((long)us - _current_time.tv_usec);

		_current_time.tv_usec = (suseconds_t)us;
	}
}

