/*
 * Copyright (C) 2017 Martin K. Schröder <mkschreder.uk@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/



#pragma once

#include "list.h"
#include "work.h"
#include "mutex.h"

#include <stdint.h>
#include <string.h>

typedef const struct can_ops ** can_port_t;

struct can_message {
	uint32_t id;  /*!< Specifies the standard identifier. This parameter can be a value between 0 to 0x7FF. */
	uint8_t data[8]; /*!< Contains the data to be transmitted. It ranges from 0 to 0xFF. */
	uint8_t len; /*!< Length of the data in bytes. */
};

struct can_listener {
	struct list_head list;
	void (*handler)(struct can_listener *self, struct can_message *msg);
};

typedef enum {
	CAN_CMD_GET_STATUS = 1
} can_control_cmd_t;

struct can_counters {
	// messages sent
	uint32_t tme;
	// number of total errors (last error changed)
	uint32_t lec;
	// number of bus off events
	uint32_t bof;
	// number of times bus went into passive error state
	uint32_t epv;
	// number of times error counter has reached warning level
	uint32_t ewg;
	// number of times fifo overflow occured
	uint32_t fov;
	// messages received on fifo0 and fifo1
	uint32_t fmp0, fmp1;
	// tx timeout counter
	uint32_t txto;
	// total messages dispatched from receive queue
	uint32_t rxp;
	// total number of messages that were received but discarded
	uint32_t rxdrop, txdrop;
	// current number of tx/rx errors
	uint8_t tec, rec;
};

struct can_status {
	struct can_counters cnt;
};

struct can_control_param {
	union {
		struct can_status status;
	} v;
};

void can_listener_init(struct can_listener *self, void (*handler)(struct can_listener *self, struct can_message *msg));

#define can_message_init(self) memset(self, 0, sizeof(*self))

typedef enum {
	CAN_LISTENER_ADD=1,
	CAN_LISTENER_REMOVE=2
} can_listen_cmd_t;

struct can_ops {
	//! send a can packet
	int (*send)(can_port_t can, const struct can_message *msg, uint32_t timeout_ms);
	//! register a packet processor
	int (*handler)(can_port_t can, can_listen_cmd_t cmd, struct can_listener *listener);
	//! get/set can port parameters
	int (*control)(can_port_t can, can_control_cmd_t cmd, struct can_control_param *param);
};

#define can_send(can, msg, tout) ((*can)->send(can, msg, tout))
#define can_register_listener(can, listener) ((*can)->handler(can, CAN_LISTENER_ADD, listener))
#define can_unregister_listener(can, listener) ((*can)->handler(can, CAN_LISTENER_REMOVE, listener))
#define can_get_status(can, param) ((*can)->control(can, CAN_CMD_GET_STATUS, param))

//#define can_recv(can, msg, tout) ((*can)->recv(can, msg, tout))

struct can_device {
	int fdt_node;
	can_port_t port;
	struct list_head list;

	struct mutex lock;
	struct list_head listeners;
};

int can_device_handler(struct can_device *self, can_listen_cmd_t cmd, struct can_listener *listener);
void can_device_dispatch_message(struct can_device *self, struct can_message *msg);

int can_register_device(void *fdt, int fdt_node, struct can_device *self, can_port_t port);
int can_unregister_device(void *fdt, int fdt_node);
can_port_t can_find(const char *dtb_path);
