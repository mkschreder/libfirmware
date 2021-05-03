/** :ms-top-comment
* +-------------------------------------------------------------------------------+
* |                      ____                  _ _     _                          |
* |                     / ___|_      _____  __| (_)___| |__                       |
* |                     \___ \ \ /\ / / _ \/ _` | / __| '_ \                      |
* |                      ___) \ V  V /  __/ (_| | \__ \ | | |                     |
* |                     |____/ \_/\_/ \___|\__,_|_|___/_| |_|                     |
* |                                                                               |
* |               _____           _              _     _          _               |
* |              | ____|_ __ ___ | |__   ___  __| | __| | ___  __| |              |
* |              |  _| | '_ ` _ \| '_ \ / _ \/ _` |/ _` |/ _ \/ _` |              |
* |              | |___| | | | | | |_) |  __/ (_| | (_| |  __/ (_| |              |
* |              |_____|_| |_| |_|_.__/ \___|\__,_|\__,_|\___|\__,_|              |
* |                                                                               |
* |                       We design hardware and program it                       |
* |                                                                               |
* |               If you have software that needs to be developed or              |
* |                      hardware that needs to be designed,                      |
* |                               then get in touch!                              |
* |                                                                               |
* |                            info@swedishembedded.com                           |
* +-------------------------------------------------------------------------------+
*
*                       This file is part of TheBoss Project
*
* FILE ............... src/linux/linux_uart.c
* AUTHOR ............. Martin K. Schröder
* VERSION ............ Not tagged
* DATE ............... 2019-05-26
* GIT ................ https://github.com/mkschreder/libfirmware
* WEBSITE ............ http://swedishembedded.com
* LICENSE ............ Swedish Embedded Open Source License
*
*          Copyright (C) 2014-2019 Martin Schröder. All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of
* this software and associated documentation files (the "Software"), to deal in
* the Software without restriction, including without limitation the rights to
* use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
* the Software, and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions:
*
* The above copyright notice and this text, in full, shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
* FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
* COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
* IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**/
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/select.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>

#include "serial.h"
#include "types.h"
#include "driver.h"
#include "serial.h"

struct linux_uart {
	struct serial_device dev;
	int fd_read, fd_write;
	int fd_host_read, fd_host_write;
	int def_port;
};

int _pipe_write(serial_port_t serial, const void *_frame, size_t size, uint32_t tout){
	struct linux_uart *self = container_of(serial, struct linux_uart, dev.ops);

	if(self->def_port){
		if(write(1, _frame, size) < 0){
			perror("write");
		}
	}

    // check for hangup event and do not write to the terminal if the other end has hung up
    struct pollfd pfd = { .fd = self->fd_write, .events = POLLHUP };
    poll(&pfd, 1, 1 /* or other small timeout */);

    if (!(pfd.revents & POLLHUP)) {
        /* There is now a reader on the slave side */
        ssize_t written = 0;
        const uint8_t *frame = (const uint8_t *)_frame;
        while(written < (ssize_t)size){
            ssize_t ret = write(self->fd_write, frame + written, (size_t)(size - (size_t)written));
            if(ret <= 0) return (int)ret;
            written += ret;
        }
        return (int)written;
    }
    return -EOF;
}

int _pipe_read(serial_port_t serial, void *frame, size_t max_size, uint32_t tout_ms){
	struct linux_uart *self = container_of(serial, struct linux_uart, dev.ops);

	suseconds_t tv_sec = (suseconds_t)(tout_ms / 1000);
	suseconds_t tv_usec = (suseconds_t)((tout_ms % 1000) * 1000);

	while(1){
	struct timeval tv;
		fd_set fds;
		tv.tv_sec = tv_sec;
		tv.tv_usec = tv_usec;

		FD_ZERO(&fds);
		FD_SET(self->fd_read, &fds);

		if(tout_ms == THREAD_SLEEP_MAX_DELAY){
			select(self->fd_read+1, &fds, NULL, NULL, NULL);
		} else {
			select(self->fd_read+1, &fds, NULL, NULL, &tv);
		}

		if(FD_ISSET(self->fd_read, &fds)){
			int r = (int)read(self->fd_read, frame, max_size);
			if(r > 0) {
				return r;
			}
		}
	}

	return 0;
}

static const struct serial_device_ops _linux_serial = {
	.write = _pipe_write,
	.read = _pipe_read
};

/**
 * Initialize a uart object to read and write from an existing fd
 * @param self pointer to the uart object
 * @param fd file descriptor to bind to this uart object (must be a valid file descriptor)
 */
int linux_uart_init_fd(struct linux_uart *self, int fd){
	memset(self, 0, sizeof(*self));
	self->fd_read = fd;
	self->fd_write = fd;
	return 0;
}

static int linux_uart_init_pipe(struct linux_uart *self){
	memset(self, 0, sizeof(*self));

	int fds[2];
    memset(fds, -1, sizeof(fds));
	if(pipe(fds) < 0) {
        perror("pipe");
        return -1;
    }
	self->fd_read = fds[0];
	self->fd_host_write = fds[1];
    memset(fds, -1, sizeof(fds));
	if(pipe(fds) < 0){
        perror("pipe");
        return -1;
    }
	self->fd_host_read = fds[0];
	self->fd_write = fds[1];
	return 0;
}

int linux_uart_init(struct linux_uart *self){
	return linux_uart_init_pipe(self);
}

static int _linux_uart_probe(void *fdt, int fdt_node){
	int def_port = fdt_get_int_or_default(fdt, (int)fdt_node, "printk_port", 0);

	int fdm = posix_openpt(O_RDWR);
	if(fdm < 0){
		perror("linux_uart open");
		return -1;
	}

	int rc = grantpt(fdm); 
	if(rc != 0){
		perror("linux_art grantpt");
		return -1;
	}
	rc = unlockpt(fdm); 
	if(rc != 0){
		perror("linux_uart unlockpt");
		return -1;
	}

    // ensure that pollhup is set so we can check it when writing
    close(open(ptsname(fdm), O_RDWR | O_NOCTTY));

	struct linux_uart *self = kzmalloc(sizeof(struct linux_uart));
	linux_uart_init_fd(self, fdm);

	self->def_port = def_port;

	serial_device_init(&self->dev, fdt, fdt_node, &_linux_serial);
	serial_device_register(&self->dev);

    if(def_port) {
	    printf("linux_uart (%s): using as printk port\n", fdt_get_name(fdt, fdt_node, NULL));
	    serial_set_printk_port(&self->dev.ops);
	}

	printf("linux_uart: start pseudo terminal at %s\n", (const char*)ptsname(fdm));

	return 0;
}

static int _linux_uart_remove(void *fdt, int fdt_node){
	// TODO
    return -1;
}

DEVICE_DRIVER(linux_uart, "gnu,linux_uart", _linux_uart_probe, _linux_uart_remove)
