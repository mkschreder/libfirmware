#include "kernel.h"

#include <libfirmware/thread.h>
#include "kernel/include/FreeRTOS.h"
#include "kernel/include/queue.h"

#include <errno.h>

int thread_create(void (*thread)(void*), const char *name, uint32_t stack_words, void *ptr, uint8_t priority, thread_t *handle){
	xTaskCreate(thread, name, (uint16_t)stack_words, ptr, priority, handle);
	return 0;
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

void thread_yield_from_isr(int32_t wake){
	portYIELD_FROM_ISR(wake);
}

void thread_sched_suspend(){
	vTaskSuspendAll();
}

void thread_sched_resume(){
	xTaskResumeAll();
}

/**
 * Warning: do not ever use hard delay unless hardware requires precis short delay when nothing is allowed to interrupt it
 */
void thread_hard_delay_us(uint32_t us){
	thread_sched_suspend();
	timestamp_t tout = micros() + us;
	while(time_after(tout, micros())) asm("nop");
	thread_sched_resume();
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


