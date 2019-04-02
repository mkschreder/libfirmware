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
    int (*write)(i2c_device_t dev, uint8_t addr_, const void *wr_data, size_t wr_len, const void *data, size_t len);
    int (*read)(i2c_device_t dev, uint8_t addr_, const void *wr_data, size_t wr_len, void *data, size_t len);
};

#define i2c_write_xfer(dev, addr, wr_buf, wr_len, buf, len) (*(dev))->write(dev, addr, wr_buf, wr_len, buf, len)
#define i2c_read_xfer(dev, addr, wr_buf, wr_len, buf, len) (*(dev))->read(dev, addr, wr_buf, wr_len, buf, len)

DECLARE_DEVICE_CLASS(i2c)

int i2c_write8_buf(i2c_device_t dev, uint8_t addr, uint8_t reg, const void *data, size_t len);
int i2c_read8_buf(i2c_device_t dev, uint8_t addr, uint8_t reg, void *data, size_t len);

int i2c_write16_buf(i2c_device_t dev, uint8_t addr, uint16_t reg, const void *data, size_t len);
int i2c_read16_buf(i2c_device_t dev, uint8_t addr, uint16_t reg, void *data, size_t len);

int i2c_write8_reg8(i2c_device_t dev, uint8_t addr, uint8_t reg, const uint8_t data);
int i2c_read8_reg8(i2c_device_t dev, uint8_t addr, uint8_t reg, uint8_t *data);

int i2c_write16_reg8(i2c_device_t dev, uint8_t addr, uint16_t reg, const uint8_t data);
int i2c_read16_reg8(i2c_device_t dev, uint8_t addr, uint16_t reg, uint8_t *data);
