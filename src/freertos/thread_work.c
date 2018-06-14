#include "work.h"
#include "thread.h"

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

