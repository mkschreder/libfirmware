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
* FILE ............... include/can.h
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

