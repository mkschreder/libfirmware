#include "kernel.h"

#include "thread.h"
#include "kernel/include/FreeRTOS.h"
#include "kernel/include/queue.h"
#include "kernel/include/task.h"
#include "kernel/include/portable.h"
#include "driver.h"

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

int thread_sleep_us(uint32_t us){
    uint32_t ms = us / 1000;
    if(ms == 0){
        uint32_t ticks = SystemCoreClock / 1000000 / 5 * us;
        asm volatile (  "MOV R0,%[loops]\n\t"\
            "1: \n\t"\
            "SUB R0, #1\n\t"\
            "CMP R0, #0\n\t"\
            "BNE 1b \n\t" : : [loops] "r" (ticks) : "memory"\
              );\
        return 0;
    }

    return thread_sleep_ms((ms == 0)?1:ms);
}

void thread_yield_from_isr(int32_t wake){
	portYIELD_FROM_ISR(wake);
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

void vApplicationStackOverflowHook(void){
	panic("Stack overflow!");
}

// TODO: get rid of this ugliness
static int _compare_tasks(const void *a, const void *b){
	TaskStatus_t *ta = (TaskStatus_t*)a;
	TaskStatus_t *tb = (TaskStatus_t*)b;
	return tb->xTaskNumber < ta->xTaskNumber;
}

void thread_meminfo(){
    #define CONSOLE_MAX_PS_TASKS 8
	// realtime tasks
	TaskStatus_t status[CONSOLE_MAX_PS_TASKS];
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
		printk("%5s%5s%8s%8s%10s%8s%8s\n", "id", "prio", "name", "stack", "cpu (us)", "cpu (%)", "load");
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

			uint32_t cpu_whole = cpu_percent / 100;
			uint32_t cpu_frac = cpu_percent % 100;

			uint32_t dcpu_whole = dcpu_percent / 100;
			uint32_t dcpu_frac = dcpu_percent % 100;

			printk("%5u%5u%8s%8u%10u%5u.%02u%5u.%02u\n",
					stat->xTaskNumber, stat->uxBasePriority, stat->pcTaskName, stat->usStackHighWaterMark, stat->ulRunTimeCounter, cpu_whole, cpu_frac, dcpu_whole, dcpu_frac);
		}
		memcpy(prev_status, status, sizeof(prev_status));
	} else {
		printk("(none)\n");
	}
}


