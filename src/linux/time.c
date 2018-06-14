#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <pthread.h>

#include <stdint.h>
#include <stdlib.h>

#include "atomic.h"
#include "timestamp.h"

timestamp_t micros(){
	timestamp_t t = 0;

	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);

	static pthread_mutex_t _lock = PTHREAD_MUTEX_INITIALIZER;

	// TODO: this implementation could be quite bad for performance. On the other hand we never use it in production..
	pthread_mutex_lock(&_lock);
	static struct timespec start_ts = {0, 0};
	if(!start_ts.tv_sec) memcpy(&start_ts, &ts, sizeof(start_ts));
	t = (timestamp_t)((ts.tv_sec - start_ts.tv_sec) * 1000000 + ts.tv_nsec / 1000);
	pthread_mutex_unlock(&_lock);

	return t;
}

