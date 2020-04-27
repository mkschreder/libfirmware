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
* FILE ............... src/freertos/thread_work.c
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
#include "thread/thread.h"
#include "thread/work.h"

#include <string.h>
#include <FreeRTOS.h>
#include <timers.h>

#define WORK_SETUP_BLOCK_TIME 10

struct work_priv {
	TimerHandle_t timer;
};

static void _run_work(TimerHandle_t timer){
	struct work *self = pvTimerGetTimerID(timer);
	if(self && self->handler)
		self->handler(self);
}

void work_init(struct work *self, void (*handler)(struct work *)){
	memset(self, 0, sizeof(*self));
	self->handler = handler;
	struct work_priv *priv = kzmalloc(sizeof(struct work_priv));
	priv->timer = xTimerCreate(
		"work",
		1,
		false,
		self,
		_run_work
	);
	vTimerSetTimerID(priv->timer, self);
	self->priv = priv;
}

int queue_work(struct work *self, uint32_t delay_ms){
	if(delay_ms < 1) delay_ms = 1;
	struct work_priv *priv = (struct work_priv*)self->priv;
	// if timer has already been started then the work is queued so we do not reque it
	if(xTimerIsTimerActive(priv->timer)) return 0;

	xTimerChangePeriod(priv->timer, delay_ms / portTICK_PERIOD_MS, WORK_SETUP_BLOCK_TIME / portTICK_PERIOD_MS);
	xTimerStart(priv->timer, WORK_SETUP_BLOCK_TIME / portTICK_PERIOD_MS);
	return 0;
}

int reschedule_work(struct work *self, uint32_t delay_ms){
	if(delay_ms < 1) delay_ms = 1;
	struct work_priv *priv = (struct work_priv*)self->priv;
	// force a reschedule of the timer to new timeout time
	xTimerStop(priv->timer, WORK_SETUP_BLOCK_TIME / portTICK_PERIOD_MS);
	xTimerChangePeriod(priv->timer, delay_ms / portTICK_PERIOD_MS, WORK_SETUP_BLOCK_TIME / portTICK_PERIOD_MS);
	xTimerStart(priv->timer, WORK_SETUP_BLOCK_TIME / portTICK_PERIOD_MS);
	return 0;
}

int queue_work_from_isr(struct work *self, uint32_t delay_ms, int32_t *_wake){
	struct work_priv *priv = (struct work_priv*)self->priv;
	BaseType_t wake;
	if(xTimerChangePeriodFromISR(priv->timer, delay_ms / portTICK_PERIOD_MS, &wake) == pdFAIL) return -1;
	if(xTimerStartFromISR(priv->timer, &wake) == pdFAIL) return -1;
	*_wake = wake;
	return 0;
}

