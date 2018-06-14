#include <stm32f4xx_iwdg.h>

void wdg_init(uint32_t period_us){
	uint32_t pr;
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	if(period_us > 16384000UL){
		pr = 6;
		IWDG_SetPrescaler(IWDG_Prescaler_256);
	} else if(period_us > 8192000UL){
		pr = 5;
		IWDG_SetPrescaler(IWDG_Prescaler_128);
	} else if(period_us > 4096000UL){
		pr = 4;
		IWDG_SetPrescaler(IWDG_Prescaler_64);
	} else if(period_us > 2048000UL){
		pr = 3;
		IWDG_SetPrescaler(IWDG_Prescaler_32);
	} else if(period_us > 1024000UL){
		pr = 2;
		IWDG_SetPrescaler(IWDG_Prescaler_16);
	} else if(period_us > 512000UL){
		pr = 1;
		IWDG_SetPrescaler(IWDG_Prescaler_8);
	} else {
		pr = 0;
		IWDG_SetPrescaler(IWDG_Prescaler_4);
	}
	uint16_t clock = (uint16_t)(32000UL/(uint16_t)(0x04<<pr));
	IWDG_SetReload((uint16_t)(period_us * clock/1000000UL-1));
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
	IWDG_ReloadCounter();
	IWDG_Enable();
}

void wdg_kick(void){
	IWDG_ReloadCounter();
}


