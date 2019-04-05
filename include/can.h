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
#include "driver.h"

#include <stdint.h>
#include <string.h>

typedef const struct can_device_ops ** can_device_t;

struct can_message {
	uint32_t id;  /*!< Specifies the standard identifier. This parameter can be a value between 0 to 0x7FF. */
	uint8_t data[8]; /*!< Contains the data to be transmitted. It ranges from 0 to 0xFF. */
	uint8_t len; /*!< Length of the data in bytes. */
};

struct can_listener {
	struct list_head list;
	void (*process_message)(struct can_listener *self, can_device_t dev, struct can_message *msg);
};

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

void can_listener_init(struct can_listener *self, void (*handler)(struct can_listener *self, can_device_t can, struct can_message *msg));

#define can_message_init(self) memset(self, 0, sizeof(*self))

struct can_dispatcher {
	can_device_t dev;
	struct list_head listeners;
	struct mutex lock;
	int (*next_message)(can_device_t dev, struct can_message *msg);
};

void can_dispatcher_init(struct can_dispatcher *self, can_device_t dev, int (*)(can_device_t dev, struct can_message *msg));
void can_dispatcher_add_listener(struct can_dispatcher *self, struct can_listener *listener);
void can_dispatcher_process_message(struct can_dispatcher *self, struct can_message *msg);

struct can_device_ops {
	//! send a can packet
	int (*send)(can_device_t can, const struct can_message *msg, uint32_t timeout_ms);
	//! register a packet processor
	int (*subscribe)(can_device_t can, struct can_listener *listener);
	//! get/set can port parameters
	//int (*control)(can_device_t can, can_control_cmd_t cmd, struct can_control_param *param);
};

#define can_send(can, msg, tout) ((*can)->send(can, msg, tout))
#define can_subscribe(can, listener) ((*can)->subscribe(can, listener))

DECLARE_DEVICE_CLASS(can)

