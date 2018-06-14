#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "chip.h"

static uint32_t cpuid[3] = { 0, 0, 0 };
bool initialized = 0;

void chip_reset_to_bootloader(void) {
	printf("Reset to bootloader is not implemented!\n");
}

void chip_reset(void){
	printf("Reset is not implemented!\n");
}


void __attribute__((constructor)) _init_id(){
	cpuid[0] = (uint32_t)rand();
	cpuid[1] = (uint32_t)rand();
	cpuid[2] = (uint32_t)rand();
}

int chip_get_uuid(uint32_t id[3]){
	id[2] = cpuid[0];
	id[1] = cpuid[1];
	id[0] = cpuid[2];
	return 0;
}

uint32_t chip_get_device_id(void){
	return cpuid[0];
}

uint16_t chip_get_flash_size_k(void){
	return 1024;
}


