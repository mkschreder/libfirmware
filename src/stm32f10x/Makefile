define target/stm32f10x/configure
ARCH:=arm
TARGET_DEPENDS:=thread
TARGET_ARCH:=arm-none-eabi-
TARGET_COMMON_FLAGS:=\
	-specs=nosys.specs \
	-mcpu=cortex-m3\
	-mthumb
TARGET_CFLAGS:=\
	-DSTM32\
	-DSTM32F10X\
	-DFREERTOS\
	-DSTM32F10X_MD\
	-DUSE_STDPERIPH_DRIVER\
	-DHSE_VALUE=8000000\
	-DUSE_STDPERIPH_DRIVER\
	-DTARGET_CPU_FREQ=72000000\
	-DTARGET_HEAP_SIZE=8000\
	-I$(TARGET_DIR)/\
	-I$(TARGET_DIR)/stdperiph/\
	-I$(TOPDIR)/include/CMSIS
TARGET_SOURCES:=\
	$(wildcard $(TARGET_DIR)/*.c) \
	$(wildcard $(TARGET_DIR)/stdperiph/*.c)
endef

define target/stm32f10x/103_64k/configure
$(eval $(call target/stm32f10x/configure))
TARGET_CFLAGS+=-DSTM32F103
TARGET_SOURCES+=$$(TARGET_DIR)/startup_stm32f10x.s
TARGET_SOURCES+=$(wildcard $(TARGET_DIR)/103/*.c)
TARGET_LDFLAGS+=-Wl,-T,$(TARGET_DIR)/103/stm32f103_64k.ld
endef

define target/stm32f10x/103_128k/configure
$(eval $(call target/stm32f10x/configure))
TARGET_CFLAGS+=-DSTM32F103
TARGET_SOURCES+=$$(TARGET_DIR)/startup_stm32f10x.s
TARGET_SOURCES+=$(wildcard $(TARGET_DIR)/103/*.c)
TARGET_LDFLAGS+=-Wl,-T,$(TARGET_DIR)/103/stm32f103_128k.ld
endef

define target/stm32f10x/rfid-dev/configure
$(eval $(call target/stm32f10x/103_128k/configure))
TARGET_DEVICETREE:=$$(TARGET_DIR)/dts/rfid-dev.dts
endef

define target/stm32f10x/flash
	openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c "program $(1) verify reset";
endef

define target/stm32f4xx/debug
	(st-util) & pid=$$!; arm-none-eabi-gdb $(1) -ex="target remote localhost:4242"; kill -9 $$pid
endef

$(eval $(call DefineTarget,stm32f10x/103_64k))
$(eval $(call DefineTarget,stm32f10x/103_128k))
$(eval $(call DefineTarget,stm32f10x/rfid-dev))

