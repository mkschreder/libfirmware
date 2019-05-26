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
* FILE ............... src/stm32f4xx/watchdog.c
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


