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

#include <sys/types.h>
#include "list.h"
#include "mutex.h"

typedef enum {
    VAR_UINT8 = (0 << 0),
    VAR_INT8 = (1 << 0),
    VAR_UINT16 = (2 << 0),
    VAR_INT16 = (3 << 0),
    VAR_INT32 = (4 << 0),
    VAR_UINT32 = (5 << 0),
    VAR_FLOAT = (6 << 0),
	VAR_STRING = (7 << 0),
	VARDIR_VALUE_TYPE_MASK = 0x7,

    VAR_MODE_DIRECT = (0 << 3),
    VAR_MODE_LOOKUP = (1 << 3),
	VARDIR_VALUE_MODE_MASK = (1 << 3),

    VAR_LIMIT =		(1 << 4),
	VAR_CONSTANT =	(1 << 5),
	VAR_RDONLY =	(1 << 6),
	VAR_REALTIME = 	(1 << 7)
} vardir_value_type_t;

struct vardir_min_max_conf {
    const int32_t min;
    const int32_t max;
};

typedef enum {
    VARDIR_LOOKUP_ON_OFF = 0,
} vardir_lookup_idx_t;

struct vardir_lookup_entry {
    const char * const *values;
    const uint8_t count;
};

static const char * const _vardir_lookup_on_off[] = { "OFF", "ON" };
#define VARDIR_LOOKUP_ENTRY(x) { (x), sizeof(x) / sizeof(x[0]) }
static const struct vardir_lookup_entry _vardir_lookup_table[] = {
	VARDIR_LOOKUP_ENTRY(_vardir_lookup_on_off)
};

struct vardir_lookup {
    const int idx;
};

struct vardir_value_target {
	void *base;
	uint16_t offset;
};

struct vardir_value_target_const {
	const void *base;
	uint16_t offset;
};

#define VARDIR_ENTRY_VALUE(type, obj, member) { .base = (obj), .offset = offsetof(type, member) }
#define VARDIR_ENTRY_VALUE_CONST_PTR(type, obj, member) { .base = (obj), .offset = offsetof(type, member) }
#define VARDIR_ENTRY_VALUE_SIMPLE(ptr) { .base = (ptr), .offset = 0 }
#define VARDIR_ENTRY_CONST_INT(val) (val)

struct vardir_entry;
struct vardir_entry_ops {
	ssize_t (*read)(struct vardir_entry_ops **, struct vardir_entry *entry, uint32_t *value);
	ssize_t (*write)(struct vardir_entry_ops **, struct vardir_entry *entry, uint32_t value);
};

struct vardir_entry {
	struct mutex lock;
	// may want to use avl list for fast search. For now we just use list. There are ways to optimize this.
	struct list_head list;
	const uint32_t id;
	const char *name;
	const uint8_t type;

	const union {
		struct vardir_lookup lookup;
		struct vardir_min_max_conf minmax;
	} conf;

	union {
		uint32_t u32;
		float f32;
		const char *const_string;
		struct vardir_entry_ops **ops;
	} value;
};

struct vardir {
	struct mutex lock;
	struct list_head directory;
};

void vardir_init(struct vardir *self);
void vardir_destroy(struct vardir *self);

int vardir_set_int_by_name(struct vardir *self, const char *var_name, uint32_t value);
int vardir_set_int(struct vardir *self, uint32_t id, uint32_t value);
int vardir_set_float_by_name(struct vardir *self, const char *var_name, float value);
int vardir_set_float(struct vardir *self, uint32_t id, float value);
int vardir_set_value(struct vardir *self, const char *name, const char *value);

struct vardir_entry *vardir_find_entry_by_name(struct vardir *self, const char *var_name);
struct vardir_entry *vardir_find_entry_by_id(struct vardir *self, uint32_t id);
int vardir_entry_get_u32(struct vardir_entry *var, uint32_t *value);

int vardir_get_int_by_name(struct vardir *self, const char *var_name, uint32_t *value);
int vardir_get_u32(struct vardir *self, uint32_t id, uint32_t *value);
int vardir_get_i32(struct vardir *self, uint32_t id, int32_t *value);
int vardir_get_u16(struct vardir *self, uint32_t id, uint16_t *value);
int vardir_get_i16(struct vardir *self, uint32_t id, int16_t *value);
int vardir_get_u8(struct vardir *self, uint32_t id, uint8_t *value);
int vardir_get_i8(struct vardir *self, uint32_t id, int8_t *value);
int vardir_get_float_by_name(struct vardir *self, const char *var_name, float *value);
int vardir_get_float(struct vardir *self, uint32_t id, float *value);
int vardir_get_string(struct vardir *self, uint32_t id, char *value, size_t max_len);
int vardir_get_value(struct vardir *self, const char *name, const char **value);

int vardir_add_field(struct vardir *self, uint32_t id, const char *name, uint8_t type, uint32_t default_value);
int vardir_add_entry(struct vardir *self, uint32_t id, const char *name, uint8_t type, struct vardir_entry_ops **ops);
int vardir_add_string(struct vardir *self, uint32_t id, const char *name, const char *str);
//int vardir_add_member(struct vardir *self, uint16_t id, uint8_t sub, const char *name, uint8_t type, void *base, uint16_t offset);

//struct canopen_drive_profile;
//int vardir_add_drive_profile(struct vardir *self, struct canopen_drive_profile *profile);
#if 0
#define vardir_add_i8(self, id, sub, name, value) vardir_add_int(self, id, sub, name, VAR_INT8 | VAR_CONSTANT, value)
#define vardir_add_u8(self, id, sub, name, value) vardir_add_int(self, id, sub, name, VAR_UINT8 | VAR_CONSTANT, value)
#define vardir_add_i16(self, id, sub, name, value) vardir_add_int(self, id, sub, name, VAR_INT16 | VAR_CONSTANT, value)
#define vardir_add_u16(self, id, sub, name, value) vardir_add_int(self, id, sub, name, VAR_UINT16 | VAR_CONSTANT, value)
#define vardir_add_i32(self, id, sub, name, value) vardir_add_int(self, id, sub, name, VAR_INT32 | VAR_CONSTANT, value)
#define vardir_add_u32(self, id, sub, name, value) vardir_add_int(self, id, sub, name, VAR_UINT32 | VAR_CONSTANT, value)
#endif

int vardir_entry_to_string(struct vardir_entry *self, char *str, size_t size);
#define vardir_for_each_entry(dir, entry) list_for_each_entry(entry, &(dir)->directory, list)
