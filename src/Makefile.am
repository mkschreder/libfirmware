lib_LIBRARIES = libfirmware.a
libfirmware_a_SOURCES = \
	adc.c\
	analog.c\
	avl-cmp.c\
    avl.c\
	baro.c\
    can.c\
    console.c\
	display.c\
    dx_tracker.c\
    pointer.c\
    driver.c\
    encoder.c\
	gpio.c\
	i2c.c\
	imu.c\
    leds.c\
    math.c\
	memory.c\
    serial.c\
    spi.c\
    time.c\
    ulink.c\
	usb.c\
    vardir.c
libfirmware_a_CFLAGS = -I$(top_srcdir)/include -std=gnu99

if WITH_TARGET_LINUX
libfirmware_a_SOURCES+=linux/chip.c\
	linux/linux_display.c\
	linux/linux_uart.c\
	linux/linux_udpcan.c\
	linux/linux_vcan.c\
	linux/linux_leds.c\
	linux/thread.c\
	linux/thread_mutex.c\
	linux/thread_queue.c\
	linux/thread_sem.c\
	linux/thread_work.c\
	linux/time.c
libfirmware_a_CFLAGS+=-D_XOPEN_SOURCE=700 -D_GNU_SOURCE
endif

if WITH_TARGET_STM32F10X
libfirmware_a_SOURCES += \
    stm32f10x/stdperiph/misc.c\
    stm32f10x/stdperiph/stm32f10x_adc.c\
    stm32f10x/stdperiph/stm32f10x_bkp.c\
    stm32f10x/stdperiph/stm32f10x_can.c\
    stm32f10x/stdperiph/stm32f10x_cec.c\
    stm32f10x/stdperiph/stm32f10x_crc.c\
    stm32f10x/stdperiph/stm32f10x_dac.c\
    stm32f10x/stdperiph/stm32f10x_dbgmcu.c\
    stm32f10x/stdperiph/stm32f10x_dma.c\
    stm32f10x/stdperiph/stm32f10x_exti.c\
    stm32f10x/stdperiph/stm32f10x_flash.c\
    stm32f10x/stdperiph/stm32f10x_fsmc.c\
    stm32f10x/stdperiph/stm32f10x_gpio.c\
    stm32f10x/stdperiph/stm32f10x_i2c.c\
    stm32f10x/stdperiph/stm32f10x_iwdg.c\
    stm32f10x/stdperiph/stm32f10x_pwr.c\
    stm32f10x/stdperiph/stm32f10x_rcc.c\
    stm32f10x/stdperiph/stm32f10x_rtc.c\
    stm32f10x/stdperiph/stm32f10x_sdio.c\
    stm32f10x/stdperiph/stm32f10x_spi.c\
    stm32f10x/stdperiph/stm32f10x_tim.c\
    stm32f10x/stdperiph/stm32f10x_usart.c\
    stm32f10x/stdperiph/stm32f10x_wwdg.c\
    stm32f10x/usb-fs-core/usb_core.c\
    stm32f10x/usb-fs-core/usb_desc.c\
    stm32f10x/usb-fs-core/usb_endp.c\
    stm32f10x/usb-fs-core/usb_init.c\
    stm32f10x/usb-fs-core/usb_int.c\
    stm32f10x/usb-fs-core/usb_istr.c\
    stm32f10x/usb-fs-core/usb_mem.c\
    stm32f10x/usb-fs-core/usb_prop.c\
    stm32f10x/usb-fs-core/usb_pwr.c\
    stm32f10x/usb-fs-core/usb_regs.c\
    stm32f10x/usb-fs-core/usb_sil.c\
    stm32f10x/stm32_adc.c\
    stm32f10x/stm32_can.c\
    stm32f10x/stm32_chip.c\
    stm32f10x/stm32_enc_out.c\
    stm32f10x/stm32_gpio.c\
    stm32f10x/stm32_i2c.c\
    stm32f10x/stm32_leds.c\
    stm32f10x/stm32_spi.c\
    stm32f10x/stm32_uart.c\
    stm32f10x/stm32_usb.c\
    stm32f10x/system_stm32f10x.c\
    stm32f10x/timestamp.c\
	freertos/atomic.c\
	freertos/thread.c\
	freertos/thread_mutex.c\
	freertos/thread_queue.c\
	freertos/thread_sem.c\
	freertos/thread_work.c\
	freertos/kernel/croutine.c\
	freertos/kernel/event_groups.c\
	freertos/kernel/list.c\
	freertos/kernel/queue.c\
	freertos/kernel/tasks.c\
	freertos/kernel/timers.c\
	freertos/kernel/portable/MemMang/heap_1.c\
	freertos/kernel/portable/GCC/ARM_CM3/port.c
libfirmware_a_CFLAGS += -I$(srcdir)/freertos/kernel/include -I$(srcdir)/freertos/kernel/portable/GCC/ARM_CM3/ -I$(top_srcdir)/include/stm32f10x -I$(top_srcdir)/include/CMSIS -DSTM32F10X -DUSE_STDPERIPH_DRIVER
endif

STM32F4XX_FILES=stm32f4xx/stdperiph/misc.c\
    stm32f4xx/stdperiph/stm32f4xx_adc.c\
    stm32f4xx/stdperiph/stm32f4xx_can.c\
    stm32f4xx/stdperiph/stm32f4xx_crc.c\
    stm32f4xx/stdperiph/stm32f4xx_cryp.c\
    stm32f4xx/stdperiph/stm32f4xx_cryp_aes.c\
    stm32f4xx/stdperiph/stm32f4xx_cryp_des.c\
    stm32f4xx/stdperiph/stm32f4xx_cryp_tdes.c\
    stm32f4xx/stdperiph/stm32f4xx_dac.c\
    stm32f4xx/stdperiph/stm32f4xx_dbgmcu.c\
    stm32f4xx/stdperiph/stm32f4xx_dcmi.c\
    stm32f4xx/stdperiph/stm32f4xx_dma.c\
    stm32f4xx/stdperiph/stm32f4xx_dma2d.c\
    stm32f4xx/stdperiph/stm32f4xx_exti.c\
    stm32f4xx/stdperiph/stm32f4xx_flash.c\
    stm32f4xx/stdperiph/stm32f4xx_fmc.c\
    stm32f4xx/stdperiph/stm32f4xx_fsmc.c\
    stm32f4xx/stdperiph/stm32f4xx_gpio.c\
    stm32f4xx/stdperiph/stm32f4xx_hash.c\
    stm32f4xx/stdperiph/stm32f4xx_hash_md5.c\
    stm32f4xx/stdperiph/stm32f4xx_hash_sha1.c\
    stm32f4xx/stdperiph/stm32f4xx_i2c.c\
    stm32f4xx/stdperiph/stm32f4xx_iwdg.c\
    stm32f4xx/stdperiph/stm32f4xx_ltdc.c\
    stm32f4xx/stdperiph/stm32f4xx_pwr.c\
    stm32f4xx/stdperiph/stm32f4xx_rcc.c\
    stm32f4xx/stdperiph/stm32f4xx_rng.c\
    stm32f4xx/stdperiph/stm32f4xx_rtc.c\
    stm32f4xx/stdperiph/stm32f4xx_sai.c\
    stm32f4xx/stdperiph/stm32f4xx_sdio.c\
    stm32f4xx/stdperiph/stm32f4xx_spi.c\
    stm32f4xx/stdperiph/stm32f4xx_syscfg.c\
    stm32f4xx/stdperiph/stm32f4xx_tim.c\
    stm32f4xx/stdperiph/stm32f4xx_usart.c\
    stm32f4xx/stdperiph/stm32f4xx_wwdg.c\
    stm32f4xx/chip.c\
    stm32f4xx/dac.c\
    stm32f4xx/debug.c\
    stm32f4xx/interrupts.c\
    stm32f4xx/pwm.c\
	stm32f4xx/stm32_adc.c\
    stm32f4xx/stm32_can.c\
    stm32f4xx/stm32_cpu.c\
    stm32f4xx/stm32_encoder.c\
    stm32f4xx/stm32_gpio.c\
    stm32f4xx/stm32_i2c.c\
    stm32f4xx/stm32_leds.c\
    stm32f4xx/stm32_pwm_motor.c\
    stm32f4xx/stm32_spi.c\
    stm32f4xx/stm32_tim.c\
    stm32f4xx/stm32_uart.c\
    stm32f4xx/system_stm32f4xx.c\
    stm32f4xx/timestamp.c\
    stm32f4xx/watchdog.c \
	freertos/atomic.c\
	freertos/thread.c\
	freertos/thread_mutex.c\
	freertos/thread_queue.c\
	freertos/thread_sem.c\
	freertos/thread_work.c\
	freertos/kernel/croutine.c\
	freertos/kernel/event_groups.c\
	freertos/kernel/list.c\
	freertos/kernel/queue.c\
	freertos/kernel/tasks.c\
	freertos/kernel/timers.c\
	freertos/kernel/portable/MemMang/heap_2.c\
	freertos/kernel/portable/GCC/ARM_CM4F/port.c

if WITH_TARGET_STM32F427
libfirmware_a_SOURCES += $(STM32F4XX_FILES)
libfirmware_a_CFLAGS += -I$(srcdir)/freertos/kernel/include -I$(srcdir)/freertos/kernel/portable/GCC/ARM_CM4F/ -I$(top_srcdir)/include/stm32f4xx -I$(top_srcdir)/include/CMSIS -DSTM32F427_437xx -DUSE_STDPERIPH_DRIVER
endif

if WITH_TARGET_STM32F429_439XX
libfirmware_a_SOURCES += $(STM32F4XX_FILES)
libfirmware_a_CFLAGS += -I$(srcdir)/freertos/kernel/include -I$(srcdir)/freertos/kernel/portable/GCC/ARM_CM4F/ -I$(top_srcdir)/include/stm32f4xx -I$(top_srcdir)/include/CMSIS -DSTM32F429_439xx -DUSE_STDPERIPH_DRIVER
endif

if WITH_TARGET_STM32F40_41XXX
libfirmware_a_SOURCES += $(STM32F4XX_FILES)
libfirmware_a_CFLAGS += -I$(srcdir)/freertos/kernel/include -I$(srcdir)/freertos/kernel/portable/GCC/ARM_CM4F/ -I$(top_srcdir)/include/stm32f4xx -I$(top_srcdir)/include/CMSIS -DSTM32F40_41xxx -DUSE_STDPERIPH_DRIVER
endif

AUTOMAKE_OPTIONS = subdir-objects
