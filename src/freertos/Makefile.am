lib_LIBRARIES = libfirmware_freertos.a
libfirmware_freertos_a_SOURCES = \
    atomic.c\
    thread.c\
    thread_mutex.c\
    thread_queue.c\
    thread_sem.c\
    work.c\
    kernel/croutine.c\
    kernel/event_groups.c\
    kernel/list.c\
    kernel/queue.c\
    kernel/tasks.c\
    kernel/timers.c
libfirmware_freertos_a_CFLAGS = -I$(top_srcdir)/include -I$(srcdir)/kernel/include -std=gnu99

ifdef WITH_CORE_CM4F
    libfirmware_freertos_a_CFLAGS+=$(CM4F_CFLAGS)
    libfirmware_freertos_a_SOURCES+=\
        kernel/portable/GCC/ARM_CM4F/port.c\
        kernel/portable/MemMang/heap_3.c
endif

AUTOMAKE_OPTIONS = subdir-objects
