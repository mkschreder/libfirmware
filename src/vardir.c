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

#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include "thread.h"
#include "vardir.h"
//#include "canopen.h"

typedef union {
	int32_t int_value;
	float float_value;
} int_float_value_t;

void vardir_init(struct vardir *self){
	memset(self, 0, sizeof(*self));
	INIT_LIST_HEAD(&self->directory);
}

void vardir_destroy(struct vardir *self){
	struct vardir_entry *entry, *p;
	list_for_each_entry_safe(entry, p, &self->directory, list){
		list_del(&entry->list);
		kfree(entry);
	}
}

struct vardir_entry *vardir_find_entry_by_name(struct vardir *self, const char *var_name){
	struct vardir_entry *entry;
	list_for_each_entry(entry, &self->directory, list){
		if (entry->name && strncasecmp(var_name, entry->name, strlen(entry->name)) == 0 && strlen(var_name) == strlen(entry->name)) {
			return entry;
		}
	}
	return NULL;
}

struct vardir_entry *vardir_find_entry_by_id(struct vardir *self, uint32_t id){
	struct vardir_entry *entry;
	list_for_each_entry(entry, &self->directory, list){
		if(entry->id == id){
			return entry;
		}
	}
	return NULL;
}


static bool _var_value_valid(const struct vardir_entry *var, float value){
	if (var->type & VAR_LIMIT){
		if(value >= var->conf.minmax.min && value <= var->conf.minmax.max) {
			return true;
		} else {
			return false;
		}
	}
	return true;
}

static int _vardir_entry_set_int(struct vardir_entry *self, uint32_t value){
	if(!self || !_var_value_valid(self, (float)value)) return -1;

	if(self->type & VAR_CONSTANT){
		return -EROFS;
	} else if(self->type & VAR_REALTIME){
		struct vardir_entry_ops *ops = *(self->value.ops);
		if(ops && ops->write){
			return (int)ops->write(self->value.ops, self, value);
		}
		return -EROFS;
	} else {
		switch (self->type & VARDIR_VALUE_TYPE_MASK) {
			case VAR_UINT8:
			case VAR_INT8:
			case VAR_UINT16:
			case VAR_INT16:
			case VAR_INT32:
			case VAR_UINT32:
				self->value.u32 = value;
				break;
			default: return -1;
		}
	}
	return 0;
}

static int _vardir_set_int(struct vardir *self, struct vardir_entry *var, uint32_t value){
	if(!var) return -1;
	if((var->type & VARDIR_VALUE_TYPE_MASK) == VAR_FLOAT)
		return -1;

	switch (var->type & VARDIR_VALUE_MODE_MASK) {
		case VAR_MODE_DIRECT: {
			return _vardir_entry_set_int(var, value);
		} break;
	}

	return -1;
}

int vardir_set_int_by_name(struct vardir *self, const char *var_name, uint32_t value){
	return _vardir_set_int(self, vardir_find_entry_by_name(self, var_name), value);
}

int vardir_set_int(struct vardir *self, uint32_t id, uint32_t value){
	return _vardir_set_int(self, vardir_find_entry_by_id(self, id), value);
}

int _vardir_set_float(struct vardir *self, struct vardir_entry *var, float value){
	if(!var) return -1;
	if((var->type & VARDIR_VALUE_TYPE_MASK) == VAR_FLOAT)
		return -1;

	union {
		float f32;
		uint32_t u32;
	} v;
	v.f32 = value;

	switch (var->type & VARDIR_VALUE_MODE_MASK) {
		case VAR_MODE_DIRECT: {
			return _vardir_entry_set_int(var, v.u32);
		} break;
	}

	return -1;
}
int vardir_set_float_by_name(struct vardir *self, const char *var_name, float value){
	return _vardir_set_float(self, vardir_find_entry_by_name(self, var_name), value);
}

int vardir_set_float(struct vardir *self, uint32_t id, float value){
	return _vardir_set_float(self, vardir_find_entry_by_id(self, id), value);
}

int vardir_set_value(struct vardir *self, const char *name, const char *value){
	float valuef;
	long int valued;
	if(sscanf(value, "%ld", &valued) > 0){
		return vardir_set_int_by_name(self, name, (uint32_t)valued);
	} else if(sscanf(value, "%f", &valuef) > 0){
		return vardir_set_float_by_name(self, name, valuef);
	}
	return -1;
}

int vardir_entry_get_u32(struct vardir_entry *e, uint32_t *value){
	if(!e) return -1;

	if(e->type & VAR_REALTIME){
		struct vardir_entry_ops *ops = *e->value.ops;
		if(ops && ops->read){
			return (int)ops->read(e->value.ops, e, value);
		}
		return -1;
	} else {
		switch (e->type & VARDIR_VALUE_TYPE_MASK) {
			case VAR_UINT8:
			case VAR_INT8:
			case VAR_UINT16:
			case VAR_INT16:
			case VAR_INT32:
			case VAR_UINT32:
				*value = e->value.u32;
			break;
			default: return -1;
		};
	} 
	return 0;
}

int vardir_get_u32_by_name(struct vardir *self, const char *var_name, uint32_t *value){
	return vardir_entry_get_u32(vardir_find_entry_by_name(self, var_name), value);
}

int vardir_get_u32(struct vardir *self, uint32_t id, uint32_t *value){
	return vardir_entry_get_u32(vardir_find_entry_by_id(self, id), value);
}

int vardir_get_i32(struct vardir *self, uint32_t id, int32_t *value){
	uint32_t v;
	if(vardir_entry_get_u32(vardir_find_entry_by_id(self, id), &v) < 0) return -1;
	*value = (int32_t)v;
	return 0;
}

int vardir_get_u16(struct vardir *self, uint32_t id, uint16_t *value){
	uint32_t v;
	if(vardir_entry_get_u32(vardir_find_entry_by_id(self, id), &v) < 0) return -1;
	*value = (uint16_t)v;
	return 0;
}

int vardir_get_i16(struct vardir *self, uint32_t id, int16_t *value){
	uint32_t v;
	if(vardir_entry_get_u32(vardir_find_entry_by_id(self, id), &v) < 0) return -1;
	*value = (int16_t)v;
	return 0;
}

int vardir_get_u8(struct vardir *self, uint32_t id, uint8_t *value){
	uint32_t v;
	if(vardir_entry_get_u32(vardir_find_entry_by_id(self, id), &v) < 0) return -1;
	*value = (uint8_t)v;
	return 0;
}

int vardir_get_i8(struct vardir *self, uint32_t id, int8_t *value){
	uint32_t v;
	if(vardir_entry_get_u32(vardir_find_entry_by_id(self, id), &v) < 0) return -1;
	*value = (int8_t)v;
	return 0;
}

int vardir_get_float_by_name(struct vardir *self, const char *var_name, float *value){
	return -1;
}

int vardir_get_float(struct vardir *self, uint32_t id, float *value){
	return -1;
}

int vardir_get_value(struct vardir *self, const char *name, const char **value){
	return -1;
}


int vardir_entry_to_string(struct vardir_entry *self, char *str, size_t size){
	if(!self) return -1;
	uint32_t value = 0;
	if(vardir_entry_get_u32(self, &value) == 0){
		switch (self->type & VARDIR_VALUE_TYPE_MASK) {
			case VAR_UINT8:
			case VAR_INT8:
				snprintf(str, size, "%02x", (unsigned int)(value & 0xff));
				break;
			case VAR_UINT16:
			case VAR_INT16:
				snprintf(str, size, "%04x", (unsigned int)(value & 0xffff));
				break;
			case VAR_INT32:
			case VAR_UINT32:
				snprintf(str, size, "%08x", (unsigned int)value);
				break;
			case VAR_FLOAT:
				snprintf(str, size, "%f", (double)value);
				break;
			default: return -1;
		}
		return 0;
	} else if(self->type & VAR_STRING && self->value.const_string){
		strncpy(str, self->value.const_string, size);
		return 0;
	}
	return -1;
}

int vardir_get_string(struct vardir *self, uint32_t id, char *value, size_t max_len){
	return vardir_entry_to_string(vardir_find_entry_by_id(self, id), value, max_len);
}
/*
int vardir_add_field(struct vardir *self, uint16_t id, uint8_t sub, const char *name, uint8_t type, void *base, uint16_t offset){
	struct vardir_entry *entry = kzmalloc(sizeof(struct vardir_entry));
	if(!entry) return -ENOMEM;
	const struct vardir_entry e = {
		.id = id,
		.sub = sub,
		.name = name,
		.type = type,
		.value = (struct vardir_value_target){
			.base = base,
			.offset = offset
		}
	};
	memcpy(entry, &e, sizeof(struct vardir_entry));
	list_add(&entry->list, &self->directory);
	return 0;
}
*/

int vardir_add_field(struct vardir *self, uint32_t id, const char *name, uint8_t type, uint32_t default_value){
	struct vardir_entry *entry = kzmalloc(sizeof(struct vardir_entry));
	if(!entry) return -ENOMEM;
	const struct vardir_entry e = {
		.id = id,
		.name = name,
		.type = type,
		.value = {
			.u32 = default_value
		}
	};
	memcpy(entry, &e, sizeof(struct vardir_entry));
	list_add(&entry->list, &self->directory);
	return 0;
}

int vardir_add_entry(struct vardir *self, uint32_t id, const char *name, uint8_t type, struct vardir_entry_ops **ops){
	struct vardir_entry *entry = kzmalloc(sizeof(struct vardir_entry));
	if(!entry) return -ENOMEM;
	const struct vardir_entry e = {
		.id = id,
		.name = name,
		.type = (uint8_t)(type | VAR_REALTIME),
		.value = {
			.ops = ops
		}
	};
	memcpy(entry, &e, sizeof(struct vardir_entry));
	list_add(&entry->list, &self->directory);
	return 0;
}

int vardir_add_string(struct vardir *self, uint32_t id, const char *name, const char *str){
	struct vardir_entry *entry = kzmalloc(sizeof(struct vardir_entry));
	if(!entry) return -ENOMEM;
	const struct vardir_entry e = {
		.id = id,
		.name = name,
		.type = VAR_STRING | VAR_CONSTANT,
		.value = {
			.const_string = str
		}
	};
	memcpy(entry, &e, sizeof(struct vardir_entry));
	list_add(&entry->list, &self->directory);
	return 0;
}

