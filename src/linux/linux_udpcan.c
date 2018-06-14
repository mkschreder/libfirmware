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

struct linux_udpcan {
	int socket;
	struct sockaddr_in addr;
	struct thread_queue rx_queue, tx_queue;
	struct can_device dev;
	const struct can_ops *can_ops;
};

static int _can_send(can_port_t can, const struct can_message *msg, uint32_t timeout_ms){
	(void)timeout_ms;
	struct linux_udpcan *self = container_of(can, struct linux_udpcan, can_ops);
	// TODO: implement timeout (for the most part these alway succeed anyway)
	return (int)sendto(self->socket, msg, sizeof(*msg), 0, (struct sockaddr*)&self->addr, sizeof(struct sockaddr));
}

#if 0
static int _can_recv(can_port_t can, struct can_message *msg, uint32_t timeout_ms){
	(void)timeout_ms;

	struct linux_udpcan *self = container_of(can, struct linux_udpcan, can_ops);

	// use OS function for setting the timeout (we could have used poll() too)
	struct timeval tv;
	// timeout of 0 should not mean infinite
	if(timeout_ms == THREAD_QUEUE_MAX_DELAY) timeout_ms = 0;
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
#endif

static int _can_handler(can_port_t can, can_listen_cmd_t cmd, struct can_listener *listener){
	struct linux_udpcan *self = container_of(can, struct linux_udpcan, can_ops);
	return can_device_handler(&self->dev, cmd, listener);
}

static const struct can_ops _can_ops = {
	.send = _can_send,
	.handler = _can_handler
};

void linux_udpcan_init(struct linux_udpcan *self){
	memset(self, 0, sizeof(*self));
	self->can_ops = &_can_ops;
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

	printf("bound udp socket!\n");
end:
	free(res);
	return -1;
}

can_port_t linux_udpcan_get_interface(struct linux_udpcan *self){
	return &self->can_ops;
}

static int _linux_udpcan_probe(void *fdt, int fdt_node){
	printf("linux_udpcan needs to get devicetree support\n");

	struct linux_udpcan *self = malloc(sizeof(struct linux_udpcan));
	linux_udpcan_init(self);

	can_register_device(fdt, fdt_node, &self->dev, &self->can_ops);

	return 0;
}

static int _linux_udpcan_remove(void *fdt, int fdt_node){
	// TODO
    return -1;
}

DEVICE_DRIVER(linux_udpcan, "gnu,linux_udpcan", _linux_udpcan_probe, _linux_udpcan_remove)
