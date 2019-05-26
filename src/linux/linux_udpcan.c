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
* FILE ............... src/linux/linux_udpcan.c
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
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <linux/in.h>

#include "types.h"
#include "driver.h"
#include "can.h"
#include "queue.h"
#include "timestamp.h"

#define LINUX_CAN_TIMEOUT 100

struct linux_udpcan {
	int socket;
	struct sockaddr_in addr;
	struct thread_queue rx_queue, tx_queue;
	struct can_device dev;
	struct can_dispatcher dispatcher;
};

static int _can_send(can_device_t can, const struct can_message *msg, uint32_t timeout_ms){
	(void)timeout_ms;
	struct linux_udpcan *self = container_of(can, struct linux_udpcan, dev.ops);
	// TODO: implement timeout (for the most part these alway succeed anyway)
	return (int)sendto(self->socket, msg, sizeof(*msg), 0, (struct sockaddr*)&self->addr, sizeof(struct sockaddr));
}

static int _can_recv(can_device_t can, struct can_message *msg){
	struct linux_udpcan *self = container_of(can, struct linux_udpcan, dev.ops);

	// use OS function for setting the timeout (we could have used poll() too)
	struct timeval tv;
	// timeout of 0 should not mean infinite
	msec_t timeout_ms = LINUX_CAN_TIMEOUT;
	if(timeout_ms == THREAD_WAIT_FOREVER) timeout_ms = 0;
	else if(timeout_ms == 0) timeout_ms = 1;
	unsigned long tus = timeout_ms * 1000;
	tv.tv_sec = (suseconds_t)(tus / 1000000UL);
	tv.tv_usec = (suseconds_t)(tus % 1000000UL);
	setsockopt(self->socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval));

	struct sockaddr_storage src_addr;
	socklen_t src_addr_len = sizeof(src_addr);
	ssize_t ret = recvfrom(self->socket, msg, sizeof(*msg), 0, (struct sockaddr*)&src_addr, &src_addr_len);

	if(ret < 0 && ret != -EWOULDBLOCK && ret != -EAGAIN){
		//perror("socket error");
		return (int)ret;
	} else if(ret > 0){
		// lookup the client and handle the packet
		// or create a new network client
		char from_ip[32];
		char key[32];
		struct sockaddr_in *sa = ((struct sockaddr_in*)&src_addr);
		inet_ntop(AF_INET, &sa->sin_addr, from_ip, sizeof(from_ip));
		snprintf(key, sizeof(key), "%s:%d", from_ip, ntohs(sa->sin_port));

		//printf("got datagram from %s, id: %d\n", key, msg->std_id);
		return 1;
	}
	return 0;
}

static int _can_subscribe(can_device_t can, struct can_listener *listener){
	struct linux_udpcan *self = container_of(can, struct linux_udpcan, dev.ops);
	can_dispatcher_add_listener(&self->dispatcher, listener);
	return 0;
}

static const struct can_device_ops _can_ops = {
	.send = _can_send,
	.subscribe = _can_subscribe
};

void linux_udpcan_init(struct linux_udpcan *self){
	memset(self, 0, sizeof(*self));
}

int linux_udpcan_bind(struct linux_udpcan *self, const char *ip, uint16_t port){
	struct addrinfo addr;
	addr.ai_family = AF_INET; // use ipv4
	addr.ai_socktype = SOCK_DGRAM;
	addr.ai_protocol = IPPROTO_UDP;
	addr.ai_flags = AI_PASSIVE | AI_ADDRCONFIG;
	struct addrinfo *res = NULL;
	char sport[64];
	snprintf(sport, sizeof(sport), "%d", port);
	if(getaddrinfo(ip, sport, &addr, &res) != 0 || res == NULL){
		perror("getting addrinfo");
		return -1;
	}

	int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(fd < 0){
		perror("create socket");
		goto end;
	}

	int option = 1;
	if(setsockopt(fd, SOL_SOCKET, (SO_REUSEPORT | SO_REUSEADDR), (char*)&option, sizeof(option)) < 0){
		perror("set sockopt");
		goto end;
	}

	if(bind(fd, res->ai_addr, res->ai_addrlen) < 0){
		perror("bind socket");
		goto end;
	}

	self->addr.sin_family = AF_INET;
	self->addr.sin_port = htons(port);
	self->addr.sin_addr.s_addr = inet_addr(ip);

	if(setsockopt(fd, IPPROTO_IP, IP_MULTICAST_IF | IP_MULTICAST_LOOP, &option, sizeof(option)) < 0){
		perror("enable multicast");
		goto end;
	}

	struct ip_mreq req;
	memset(&req, 0, sizeof(req));
	inet_aton(ip, &req.imr_multiaddr);
	if( setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req)) < 0 ) {
		perror("setsockopt IP_ADD_MEMBERSHIP");
		return -1;
	}

	self->socket = fd;
	can_dispatcher_init(&self->dispatcher, &self->dev.ops, _can_recv);

	printf("bound udp socket!\n");
end:
	free(res);
	return -1;
}

static int _linux_udpcan_probe(void *fdt, int fdt_node){
	printf("linux_udpcan needs to get devicetree support\n");

	struct linux_udpcan *self = malloc(sizeof(struct linux_udpcan));
	linux_udpcan_init(self);

	can_device_init(&self->dev, fdt, fdt_node, &_can_ops);
	can_device_register(&self->dev);

	return 0;
}

static int _linux_udpcan_remove(void *fdt, int fdt_node){
	// TODO
    return -1;
}

DEVICE_DRIVER(linux_udpcan, "gnu,linux_udpcan", _linux_udpcan_probe, _linux_udpcan_remove)
