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

#include "timestamp.h"
#include "list.h"
#include "driver.h"

typedef const struct i2c_device_ops ** i2c_device_t;

struct i2c_device_ops {
    int (*transfer)(i2c_device_t dev, uint8_t addr_, const void *tx_data, size_t tx_len, void *rx_data, size_t rx_len, uint32_t timeout_ms);
};

#define i2c_transfer(dev, addr, wr_buf, wr_len, buf, len, timeout) (*(dev))->transfer(dev, addr, wr_buf, wr_len, buf, len, timeout)

DECLARE_DEVICE_CLASS(i2c)

