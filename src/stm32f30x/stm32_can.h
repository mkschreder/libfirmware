#pragma once

#if CONFIG_CAN == 1

#include <thread/queue.h>
#include <thread/mutex.h>
#include <firmware/can.h>

struct stm32_can {
	const struct can_ops *can_ops;
	struct mutex lock;
};

void stm32_can_init(struct stm32_can *self, uint32_t baud);
can_port_t stm32_can_get_interface(struct stm32_can *self);

#endif
