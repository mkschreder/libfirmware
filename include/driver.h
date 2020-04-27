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
* FILE ............... include/driver.h
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
#include <errno.h>
#include <libfdt/libfdt.h>
#include "thread/thread.h"
#include "types/list.h"

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
int CLASS ## _device_unregister(struct CLASS ## _device *self);\
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
int CLASS ## _device_unregister(struct CLASS ## _device *self){\
	BUG_ON(!self);\
	BUG_ON(!self->ops);\
	list_del(&self->list);\
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
    static struct device_driver __attribute__((used,aligned(4))) _driver_ ## _name = { .name = #_name, .compatible = _compatible, .probe = _probe, .remove = _remove };\
    register_device_driver(&_driver_ ## _name);\
}

#define DEVICE_NAME(dev) fdt_get_name((dev)->fdt, (dev)->fdt_node, NULL)
#define DEVICE_REF(name, type, ref) do { \
		self->ref = type ## _find_by_ref(fdt, fdt_node, #ref); \
		if(!self->ref) { \
			printk(PRINT_ERROR name ": %s missing\n", #ref);\
			/*return -EINVAL;*/ \
		}\
	} while(0)

int probe_device_drivers(void *fdt);
int remove_device_drivers(void *fdt);
void register_device_driver(struct device_driver *self);

extern unsigned char _devicetree[];

#define PRINT_DEFAULT "\033[0m"
#define PRINT_ERROR "\033[31m"
#define PRINT_SUCCESS "\033[32m"
#define PRINT_SYSTEM "\033[33m"

// this is defined to use the default uart once it has been initialized
// this function is defined in serial.c but placed here because it is very often used in device drivers
int printk(const char *fmt, ...);
int printk_isr(const char *fmt, ...);
#define dbg_printk printk
#define no_printk(...) do {} while(0)
