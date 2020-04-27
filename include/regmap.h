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
* FILE ............... include/regmap.h
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

#include <sys/types.h>
#include "driver.h"

typedef enum {
    REG_UINT8 = (0 << 0),
    REG_INT8 = (1 << 0),
    REG_UINT16 = (2 << 0),
    REG_INT16 = (3 << 0),
    REG_INT32 = (4 << 0),
    REG_UINT32 = (5 << 0),
	REG_STRING = (7 << 0),
} regmap_value_type_t;

struct regmap_range {
	struct list_head list;
	uint32_t start, end;
	const struct regmap_range_ops *ops;
};

typedef const struct regmap_range_ops ** regmap_range_t;

struct regmap_range_ops {
	ssize_t (*read)(regmap_range_t range, uint32_t id, regmap_value_type_t type, void *value, size_t size);
	ssize_t (*write)(regmap_range_t range, uint32_t id, regmap_value_type_t type, const void *value, size_t size);
};

void regmap_range_init(struct regmap_range *self, uint32_t start, uint32_t end, const struct regmap_range_ops *ops);

typedef const struct regmap_device_ops ** regmap_device_t;

struct regmap_device_ops {
	int (*add)(regmap_device_t dev, struct regmap_range *range);
	int (*read)(regmap_device_t dev, uint32_t id, regmap_value_type_t get_as, void *value, size_t value_size);
	int (*write)(regmap_device_t dev, uint32_t id, regmap_value_type_t set_from, const void* value, size_t size);
};

#define regmap_add(dev, map) (*(dev))->add(dev, map)
#define regmap_read(dev, id, type, val, size) (*(dev))->read(dev, id, type, val, size)
#define regmap_write(dev, id, type, val, size) (*(dev))->write(dev, id, type, val, size)

DECLARE_DEVICE_CLASS(regmap)

int regmap_write_u8(regmap_device_t dev, uint32_t id, uint8_t value);
int regmap_write_i8(regmap_device_t dev, uint32_t id, int8_t value);
int regmap_write_u16(regmap_device_t dev, uint32_t id, uint16_t value);
int regmap_write_i16(regmap_device_t dev, uint32_t id, int16_t value);
int regmap_write_u32(regmap_device_t dev, uint32_t id, uint32_t value);
int regmap_write_i32(regmap_device_t dev, uint32_t id, int32_t value);
int regmap_write_string(regmap_device_t dev, uint32_t id, const char *str, size_t len);

int regmap_read_u32(regmap_device_t dev, uint32_t id, uint32_t *value);
int regmap_read_i32(regmap_device_t dev, uint32_t id, int32_t *value);
int regmap_read_u16(regmap_device_t dev, uint32_t id, uint16_t *value);
int regmap_read_i16(regmap_device_t dev, uint32_t id, int16_t *value);
int regmap_read_u8(regmap_device_t dev, uint32_t id, uint8_t *value);
int regmap_read_i8(regmap_device_t dev, uint32_t id, int8_t *value);
int regmap_read_string(regmap_device_t dev, uint32_t id, char *value, size_t max_len);

int regmap_convert_u32(uint32_t value, regmap_value_type_t type, void *data, size_t size);
int regmap_convert_u16(uint16_t value, regmap_value_type_t type, void *data, size_t size);

int regmap_mem_to_u32(regmap_value_type_t type, const void *data, size_t size, uint32_t *value);
int regmap_mem_to_u16(regmap_value_type_t type, const void *data, size_t size, uint16_t *value);
int regmap_mem_to_u8(regmap_value_type_t type, const void *data, size_t size, uint8_t *value);
