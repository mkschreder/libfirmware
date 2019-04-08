#include "kernel.h"

#include "thread.h"
#include "kernel/include/FreeRTOS.h"
#include "kernel/include/queue.h"
#include "kernel/include/task.h"
#include "kernel/include/portable.h"
#include "driver.h"
#include "math.h"

#include <errno.h>

int thread_create(void (*thread)(void*), const char *name, uint32_t stack_words, void *ptr, uint8_t priority, thread_t *handle){
	if(xTaskCreate(thread, name, (uint16_t)stack_words, ptr, priority, handle) == pdPASS) return 0;
	return -ENOMEM;
}

void thread_enter_critical(void){
	portENTER_CRITICAL();
}
/*
void thread_set_tag(thread_t thread, TaskHookFunction_t tag){
	vTaskSetApplicationTaskTag( thread, tag );
}
*/
void thread_start_scheduler(void){
	vTaskStartScheduler();
}

int thread_sleep_ms(uint32_t ms){
	vTaskDelay((ms)/portTICK_PERIOD_MS);
	if(ucTaskDelayWasAborted()) return -EINTR;
	return 0;
}

int thread_sleep_ms_until(uint32_t *last, uint32_t ms){
	vTaskDelayUntil(last, ms/portTICK_PERIOD_MS);
	if(ucTaskDelayWasAborted()) return -EINTR;
	return 0;
}

uint32_t thread_ticks_count(){
	return xTaskGetTickCount();
}

uint32_t thread_ticks_from_us(uint32_t us){
	uint32_t tick_period_us = (1000000 / configTICK_RATE_HZ);
	return constrain_u32(us / tick_period_us, 1, 0xffffffff);
}

int thread_sleep_us(uint32_t us){
	delay_us(us);
	return 0;
}

void thread_yield_from_isr(int32_t wake){
	portYIELD_FROM_ISR(wake);
}

void thread_yield(){
	taskYIELD();
}

void thread_sched_suspend(){
	vTaskSuspendAll();
}

void thread_sched_resume(){
	xTaskResumeAll();
}

void* kmalloc(size_t size) {
	return pvPortMalloc(size);
}

void* kzmalloc(size_t size) {
	void *_memory = pvPortMalloc(size);
	if(!_memory) return 0;
	memset(_memory, 0, size);
	return _memory;
}

void kfree(void *ptr){
	vPortFree(ptr);
}

unsigned long thread_get_free_heap(){
    return xPortGetFreeHeapSize();
}

unsigned long thread_get_total_heap(){
    return configTOTAL_HEAP_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *name){
	printk("stack overflow in %s\n", name);
	panic("SOVF");
}

void vApplicationMallocFailedHook(void){
    panic("NOMEM");
}

// TODO: get rid of this ugliness
static int _compare_tasks(const void *a, const void *b){
	TaskStatus_t *ta = (TaskStatus_t*)a;
	TaskStatus_t *tb = (TaskStatus_t*)b;
	return (int)ta->xTaskNumber - (int)tb->xTaskNumber;
}

void thread_meminfo(){
    #define CONSOLE_MAX_PS_TASKS 12
	// realtime tasks
	static TaskStatus_t status[CONSOLE_MAX_PS_TASKS];
	static TaskStatus_t prev_status[CONSOLE_MAX_PS_TASKS];
	memset(status, 0, sizeof(status));
	uint32_t total_time;
	printk("== realtime tasks\n");
	UBaseType_t ret = uxTaskGetSystemState(status, sizeof(status)/sizeof(status[0]), &total_time);
	struct timeval tval;
	time_gettime(&tval);
	if(ret > 0){
		qsort(status, ret, sizeof(status[0]), _compare_tasks);
		uint32_t total_calculated = 0;
		uint32_t prev_total_calculated = 0;
		for(UBaseType_t c = 0; c < ret; c++){
			TaskStatus_t *stat = &status[c];
			TaskStatus_t *prev_stat = &prev_status[c];
			total_calculated += stat->ulRunTimeCounter;
			prev_total_calculated += prev_stat->ulRunTimeCounter;
		}
		printk("time elapsed: %u:%06u, micros: %u, time usr: %u\n", tval.tv_sec, tval.tv_usec, micros(), total_calculated);
		printk("heap: %lu free of %lu bytes\n", xPortGetFreeHeapSize(), configTOTAL_HEAP_SIZE);
		//printk("data: %d\n", chip_get_data_size());
		printk("%5s%5s%8s%8s%10s%8s%8s\n", "id", "prio", "name", "stack", "cpu (ms)", "cpu (%)", "load");
		for(UBaseType_t c = 0; c < ret; c++){
			TaskStatus_t *stat = &status[c];
			TaskStatus_t *prev_stat = &prev_status[c];
			uint32_t dtotal = total_calculated - prev_total_calculated;
			uint32_t run_time = stat->ulRunTimeCounter;
			uint32_t drun_time = stat->ulRunTimeCounter - prev_stat->ulRunTimeCounter;

			uint32_t cpu_percent = 0;
			uint32_t dcpu_percent = 0;

			if(total_calculated && dtotal){
				cpu_percent = (uint32_t)((uint64_t)run_time * 10000 / total_calculated);
				dcpu_percent = (uint32_t)(((uint64_t)drun_time * 10000) / dtotal);
			}

			uint32_t cpu_whole = (cpu_percent / 100) % 100;
			uint32_t cpu_frac = cpu_percent % 100;

			uint32_t dcpu_whole = (dcpu_percent / 100) % 100;
			uint32_t dcpu_frac = dcpu_percent % 100;

			printk("%5u%5u%8s%8u%10u%5u.%02u%5u.%02u\n",
					stat->xTaskNumber,
					stat->uxBasePriority,
					stat->pcTaskName,
					stat->usStackHighWaterMark,
					stat->ulRunTimeCounter / (portTICK_PERIOD_MS * 1000),
					cpu_whole, cpu_frac, dcpu_whole, dcpu_frac);
		}
		memcpy(prev_status, status, sizeof(prev_status));
	} else {
		printk("(none)\n");
	}
}

