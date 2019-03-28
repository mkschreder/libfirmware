#pragma once

#include "thread.h"
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


#include "list.h"

struct device_driver {
	const char *name;
	const char *compatible;
	int (*probe)(void *fdt, int device_node);
	int (*remove)(void *fdt, int device_node);
    struct list_head list;
} __attribute__((aligned(4)));

#define DECLARE_DEVICE_CLASS(CLASS) \
struct CLASS ## _device {\
	struct list_head list;\
	const struct CLASS ## _device_ops *ops;\
	void *fdt;\
	int fdt_node;\
};\
void CLASS ## _device_init(struct CLASS ## _device *self,\
		void *fdt, int fdt_node, const struct CLASS ## _device_ops *ops);\
int CLASS ## _device_register(struct CLASS ## _device *self);\
CLASS ## _device_t CLASS ## _find(void *fdt, const char *dtb_path);\
CLASS ## _device_t CLASS ## _find_by_node(void *fdt, int node);\
CLASS ## _device_t CLASS ## _find_by_ref(void *fdt, int fdt_node, const char *ref_name);\

#define DEFINE_DEVICE_CLASS(CLASS)\
static LIST_HEAD(_ ## CLASS ## _drivers);\
void CLASS ## _device_init(struct CLASS ## _device *self, void *fdt, int fdt_node, const struct CLASS ## _device_ops *ops){\
	memset(self, 0, sizeof(*self));\
	INIT_LIST_HEAD(&self->list);\
	self->fdt = fdt;\
	self->fdt_node = fdt_node;\
	self->ops = ops;\
}\
\
int CLASS ## _device_register(struct CLASS ## _device *self){\
	BUG_ON(!self);\
	BUG_ON(!self->ops);\
	list_add_tail(&self->list, &_ ## CLASS ## _drivers);\
	return 0;\
}\
\
CLASS ## _device_t CLASS ## _find_by_node(void *fdt, int node){\
	struct CLASS ## _device *dev;\
    if(node < 0) return NULL;\
    list_for_each_entry(dev, &_ ## CLASS ## _drivers, list){\
		if(dev->fdt == fdt && dev->fdt_node == node) return &dev->ops;\
	}\
	return NULL;\
}\
\
CLASS ## _device_t CLASS ## _find(void *fdt, const char *dtb_path){\
	int node = fdt_path_offset(fdt, dtb_path);\
	if(node < 0) return NULL;\
    return CLASS ## _find_by_node(fdt, node);\
}\
\
CLASS ## _device_t CLASS ## _find_by_ref(void *fdt, int fdt_node, const char *ref_name){\
	int node = fdt_find_node_by_ref(fdt, fdt_node, ref_name);\
	if(node < 0){return 0;}\
	CLASS ## _device_t dev = CLASS ## _find_by_node(fdt, node);\
	if(!dev){ return 0; }\
	return dev;\
}

#define DEVICE_DRIVER(_name, _compatible, _probe, _remove) void __attribute__((__constructor__,used)) _name ## _ko(){ \
    static struct device_driver __attribute__((used,aligned(4))) _driver = { .name = #_name, .compatible = _compatible, .probe = _probe, .remove = _remove };\
    register_device_driver(&_driver);\
}

int probe_device_drivers(void *fdt);
int remove_device_drivers(void *fdt);
void register_device_driver(struct device_driver *self);

extern unsigned char _devicetree[];

// this is defined to use the default uart once it has been initialized
// this function is defined in serial.c but placed here because it is very often used in device drivers
int printk(const char *fmt, ...);
int printk_isr(const char *fmt, ...);
#define dbg_printk printk
#define no_printk(...) do {} while(0)
