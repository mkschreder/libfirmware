
#if 0
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_iwdg.h"
#include "timestamp.h"
#include "config_flash.h"
#include "debug.h"

// Activate the bootloader without BOOT* pins.
void sys_reset_to_bootloader(void) {
	portENTER_CRITICAL();
	RCC_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	__set_PRIMASK(1);
	__set_MSP(0x20001000);

	((void (*)(void)) *((uint32_t*)0x1fff0004))();
/*
    // arm-none-eabi-gcc 4.9.0 does not correctly inline this
    // MSP function, so we write it out explicitly here.
    //__set_MSP(*((uint32_t*) 0x00000000));
    __ASM volatile ("movs r3, #0\nldr r3, [r3, #0]\nMSR msp, r3\n" : : : "r3", "sp");

    ((void (*)(void)) *((uint32_t*) 0x00000004))();
*/
    while (1);
}

void sys_reset(void){
	__DSB();                                                     /* Ensure all outstanding memory accesses included
															  buffered write are completed before reset */
	SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)      |
				 (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
				 SCB_AIRCR_SYSRESETREQ_Msk);                   /* Keep priority group unchanged */
	__DSB();                                                     /* Ensure completion of memory access */
	while(1);                                                    /* wait until reset */
}

static sys_micros_t _micros(const struct system_calls_time *self){
	(void)self;
	return micros();
}

static int _eeprom_read(const struct system_calls_bdev *self, void *dst, uint16_t addr, size_t size){
    (void)self;
    return flash_read(addr, dst, size);
}

static int _eeprom_write(const struct system_calls_bdev *self, uint16_t addr, const void *data, size_t size){
    (void)self;
    return flash_write(addr, data, size);
}

static int _eeprom_erase_page(const struct system_calls_bdev *self, uint16_t addr){
    (void)self;
    return flash_erase_page(addr);
}

static void _eeprom_get_info(const struct system_calls_bdev *self, struct system_bdev_info *info){
    (void)self;
    info->page_size = flash_get_page_size();
    info->num_pages = flash_get_num_pages();
}

static void _wdg_kick(const struct system_calls_wdg *self){
	(void)self;
	wdg_kick();
}

static struct system_calls _system_calls = {
	.time = {
		.micros = _micros
	},
	.wdg = {
		.kick = _wdg_kick
	},
	.eeprom = {
		.read = _eeprom_read,
		.write = _eeprom_write,
		.erase_page = _eeprom_erase_page,
		.get_info = _eeprom_get_info
	}
};

struct system_calls* sys_init(void){
	//_enable_fpu();

	NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);

	swo_init();

	led_init();
	time_init();
	pwm_init();

	wdg_init(3000000);

	return &_system_calls;
}
#endif
