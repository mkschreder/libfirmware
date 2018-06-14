lib_LIBRARIES = libfirmware_linux.a
libfirmware_linux_a_SOURCES = chip.c\
    linux_uart.c\
    linux_udpcan.c\
    linux_vcan.c\
    linux_leds.c\
    thread.c\
    thread_mutex.c\
    thread_queue.c\
    thread_sem.c\
    thread_work.c\
    time.c
libfirmware_linux_a_CFLAGS = -I../include -std=gnu99 -D_XOPEN_SOURCE=700 -D_GNU_SOURCE

