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
* FILE ............... src/linux/linux_vcan.c
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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <time.h>
#include <errno.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <libfdt/libfdt.h>

#include "driver.h"
#include "can.h"
#include "timestamp.h"

#include <pthread.h>

#define LINUX_CAN_TIMEOUT 100

struct linux_vcan {
	struct sockaddr_can addr;
	int sock;
	int debug;
	struct can_device dev;
	pthread_t thread;
	pthread_mutex_t lock;
	bool shutdown;
	struct can_dispatcher dispatcher;
};

static int _can_recv(can_device_t dev, struct can_message *can_msg){
	struct linux_vcan *self = container_of(dev, struct linux_vcan, dev.ops);

	struct msghdr msg;
	struct can_frame frame;
	struct iovec iov;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
	memset(&msg, 0, sizeof(msg));
	memset(&iov, 0, sizeof(iov));
	memset(ctrlmsg, 0, sizeof(ctrlmsg));

	int sock = self->sock;

	iov.iov_base = &frame;
	iov.iov_len = sizeof(frame);
	msg.msg_name = &self->addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;
	msg.msg_namelen = sizeof(self->addr);
	msg.msg_controllen = sizeof(ctrlmsg);  
	msg.msg_flags = 0;

	fd_set rdfs;
	FD_ZERO(&rdfs);
	FD_SET(sock, &rdfs);

	struct timeval tv;
	msec_t timeout_ms = LINUX_CAN_TIMEOUT;
	tv.tv_sec = (time_t)(timeout_ms / 1000);
	tv.tv_usec = (suseconds_t)((timeout_ms * 1000) % 1000000);
	if (select(sock + 1, &rdfs, NULL, NULL, &tv) <= 0) {
		return -1;
	}

	ssize_t nbytes = recvmsg(sock, &msg, 0);

	if (nbytes < 0) {
		if ((errno == ENETDOWN)) {
			fprintf(stderr, "can: interface down\n");
		}
		perror("read");
		return -1;
	}

	can_msg->id = frame.can_id;
	can_msg->len = frame.can_dlc;
	memcpy(can_msg->data, frame.data, can_msg->len);

	if(self->debug){
		pthread_mutex_lock(&self->lock);
		printf("MSGIN can_id: %08x, data: ", can_msg->id);
		for(int c = 0; c < can_msg->len; c++){
			printf("%02x ", can_msg->data[c]);
		}
		printf("\n");
		fflush(stdout);
		pthread_mutex_unlock(&self->lock);
	}

	return 1;
}

static int _can_send(can_device_t port, const struct can_message *msg, uint32_t timeout_ms){
	struct linux_vcan *self = container_of(port, struct linux_vcan, dev.ops);
	struct can_frame frame;
	memset(&frame, 0, sizeof(frame));

	frame.can_id = msg->id;
	frame.can_dlc = msg->len;
	memcpy(frame.data, msg->data, msg->len);

	pthread_mutex_lock(&self->lock);
	if(self->debug){
		printf("MSGOUT can_id: %08x, data: ", msg->id);
		for(int c = 0; c < msg->len; c++){
			printf("%02x ", msg->data[c]);
		}
		printf("\n");
		fflush(stdout);
	}
	pthread_mutex_unlock(&self->lock);

	if(write(self->sock, &frame, CAN_MTU) < 0){
		perror("write");
		return -1;
	}

	return 1;
}

static int _can_subscribe(can_device_t port, struct can_listener *listener){
	struct linux_vcan *self = container_of(port, struct linux_vcan, dev.ops);
	can_dispatcher_add_listener(&self->dispatcher, listener);
	return 0;
}

static const struct can_device_ops _can_ops = {
	.send = _can_send,
	.subscribe = _can_subscribe
};

void linux_vcan_init(struct linux_vcan *self, const char *device){
	memset(self, 0, sizeof(*self));

	self->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	self->addr.can_family = AF_CAN;
	can_dispatcher_init(&self->dispatcher, &self->dev.ops, _can_recv);
	pthread_mutex_init(&self->lock, NULL);

	struct ifreq ifr;
	memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
	strcpy(ifr.ifr_name, device);

	if (strcmp("any", ifr.ifr_name)) {
		if (ioctl(self->sock, SIOCGIFINDEX, &ifr) < 0) {
			perror("SIOCGIFINDEX");
			return;
		}
		self->addr.can_ifindex = ifr.ifr_ifindex;
	} else
		self->addr.can_ifindex = 0; /* any can interface */

	printf("linux_vcan: using interface %s (%d)\n", ifr.ifr_name, ifr.ifr_ifindex);

	if (bind(self->sock, (struct sockaddr *)&self->addr, sizeof(self->addr)) < 0) {
		perror("bind");
		return;
	}
}

void linux_vcan_destroy(struct linux_vcan *self){
	self->shutdown = true;
	pthread_join(self->thread, NULL);
	close(self->sock);
}

static int _linux_vcan_probe(void *fdt, int fdt_node){
	const char* device = fdt_get_string_or_default(fdt, (int)fdt_node, "interface", "can0");

	printf("linux_vcan: initializing device %s..\n", device);

	struct linux_vcan *self = kzmalloc(sizeof(struct linux_vcan));
	linux_vcan_init(self, device);

	self->debug = fdt_get_int_or_default(fdt, (int)fdt_node, "debug", 0);

	can_device_init(&self->dev, fdt, fdt_node, &_can_ops);
	can_device_register(&self->dev);

	return 0;
}

static int _linux_vcan_remove(void *fdt, int fdt_node){
	can_device_t port = can_find_by_node(fdt, fdt_node);
	if(!port) return -1;
	printf("linux_vcan: removing %s, %p\n", fdt_get_name(fdt, fdt_node, NULL), (void*)port);
	struct linux_vcan *self = container_of(port, struct linux_vcan, dev.ops);
	can_device_unregister(&self->dev);
	linux_vcan_destroy(self);
	kfree(self);
    return 0;
}

DEVICE_DRIVER(linux_vcan, "gnu,linux_vcan", _linux_vcan_probe, _linux_vcan_remove)

