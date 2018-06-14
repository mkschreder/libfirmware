#include <sys/signal.h>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

#include <string.h>
#include <pthread.h>

#include "work.h"
#include "thread.h"

#define WORK_SETUP_BLOCK_TIME 10

struct work_priv {
	pthread_mutex_t mx;
	timer_t timer;
	bool queued; // specifies if work has already been queued
};

static void _run_work(union sigval sigval){
	struct work *self = (struct work*)sigval.sival_ptr;
	if(!self) return;

	if(self->priv){
		struct work_priv *priv = (struct work_priv*)self->priv;
		pthread_mutex_lock(&priv->mx);
		priv->queued = false;
		pthread_mutex_unlock(&priv->mx);

		if(self->handler) self->handler(self);
	}
}

void work_init(struct work *self, void (*handler)(struct work *)){
	memset(self, 0, sizeof(*self));
	self->handler = handler;

	struct work_priv *priv = kzmalloc(sizeof(struct work_priv));
	memset(priv, 0, sizeof(struct work_priv));

	self->priv = priv;
	pthread_mutex_init(&priv->mx, NULL);

	struct sigevent sev;
	sev.sigev_notify = SIGEV_THREAD;
	sev.sigev_signo = SIGRTMIN;
	sev.sigev_value.sival_ptr = self;
	sev.sigev_notify_function = _run_work;
	sev.sigev_notify_attributes = NULL;

	if(timer_create(CLOCK_MONOTONIC, &sev, &priv->timer) < 0){
		printf("Could not create work queue timer!\n");
		goto cleanup;
	} else {
		printf("Created work queue\n");
	}
	return;
cleanup:
	kfree(priv);
	self->priv = 0;
}

void work_destroy(struct work *self){
	struct work_priv *priv = (struct work_priv*)self->priv;
	// disarm timer
	struct itimerspec ts;
	memset(&ts, 0, sizeof(ts));
	timer_settime(priv->timer, 0, &ts, NULL);
	signal(SIGRTMIN, SIG_IGN);

	// sync with the worker if it is still running..
	pthread_mutex_lock(&priv->mx);
	pthread_mutex_unlock(&priv->mx);

	if(priv){
		timer_delete(priv->timer);
		pthread_mutex_destroy(&priv->mx);
		kfree(priv);
		self->priv = 0;
	}
}

int reschedule_work(struct work *self, uint32_t delay_ms){
    return queue_work(self, delay_ms);
}

int queue_work(struct work *self, uint32_t delay_ms){
	if(delay_ms < 1) delay_ms = 1;
	struct work_priv *priv = (struct work_priv*)self->priv;
	if(!priv){
		printf("Warning: work item has not been properly initialized!\n");
		fflush(stdout);
		return -1;
	}
	// if the work has already been queued then we don't queue it again
	// work will then run ahead of time though.
	// perhaps we should add semantics that call the work again later? An array of timers perhaps?
	pthread_mutex_lock(&priv->mx);
	if(!priv->queued){
		priv->queued = true;
		struct itimerspec ts;
		memset(&ts, 0, sizeof(ts));
		ts.it_value.tv_sec = (time_t)(delay_ms / 1000);
		ts.it_value.tv_nsec = (suseconds_t)((delay_ms * 1000000) % 1000000000);
		if(timer_settime(priv->timer, 0, &ts, 0) < 0){
			printf("error queuing work!\n");
			pthread_mutex_unlock(&priv->mx);
			return -1;
		}
	}
	pthread_mutex_unlock(&priv->mx);
	return 0;
}

