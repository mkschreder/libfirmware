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

AUTOMAKE_OPTIONS = subdir-objects
