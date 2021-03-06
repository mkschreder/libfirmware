<<<<<<< HEAD
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
* FILE ............... src/driver.c
* AUTHOR ............. Martin K. Schröder
* VERSION ............ Not tagged
* DATE ............... 2019-05-26
* GIT ................ https://github.com/mkschreder/libfirmware
* WEBSITE ............ http://swedishembedded.com
* LICENSE ............ Swedish Embedded Open Source License
*
*                     Copyright (C) 2014-2019 Martin Schröder
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
#include <libfdt/libfdt.h>
=======
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

>>>>>>> push latest changes
#include <errno.h>
#include <libfdt/libfdt.h>

#include "driver.h"

static struct list_head _driver_list = LIST_HEAD_INIT(_driver_list);

static int _init_subnodes(void *fdt, int root) {
	int node;
	fdt_for_each_subnode(node, fdt, root) {
		struct device_driver *driver;

		char path[32];
		fdt_get_path(fdt, node, path, sizeof(path));

		bool found_driver = false;

		list_for_each_entry(driver, &_driver_list, list) {
			if(!driver->compatible || !driver->probe)
				continue;
			int r = 0;
			r = fdt_node_check_compatible(fdt, node, driver->compatible);
			if(r == 1) {
			} else if(fdt_first_property_offset(fdt, root) >= 0 &&
			          r == -FDT_ERR_NOTFOUND) {
				// the node has properties but no compatible string (nodes without properties
				// are allowed not to have it)
				found_driver = true;
				break;
			} else if(r == 0) {
				found_driver = true;
				printk(PRINT_SYSTEM ">> %s\n", path);
				if((r = driver->probe(fdt, node)) < 0) {
					printk(PRINT_ERROR "devicetree: failed to probe %s (error(%d): %s)\n",
					       path, r, strerror(-r));
				}
				break;
			}
		}

		if(!found_driver) {
			printk(PRINT_ERROR
			       "devicetree: driver for %s was not included in the firmware\n",
			       path);
		}

		// initialize children recursively
		_init_subnodes(fdt, node);
	}
	return 0;
}

static int _deinit_subnodes(void *fdt, int root) {
	int node;
	fdt_for_each_subnode(node, fdt, root) {
		struct device_driver *driver;
		_deinit_subnodes(fdt, node);
		list_for_each_entry(driver, &_driver_list, list) {
			if(!driver->compatible || !driver->remove)
				continue;
			if(fdt_node_check_compatible(fdt, node, driver->compatible) == 0) {
				driver->remove(fdt, node);
				break;
			}
		}
	}
	return 0;
}

int probe_device_drivers(void *fdt) {
	if(fdt_check_header(fdt) != 0)
		return -EINVAL;

	return _init_subnodes(fdt, 0);
}

int remove_device_drivers(void *fdt) {
	if(fdt_check_header(fdt) != 0)
		return -EINVAL;

	return _deinit_subnodes(fdt, 0);
}

#include <stdio.h>
void register_device_driver(struct device_driver *self) {
	INIT_LIST_HEAD(&self->list);
	list_add(&self->list, &_driver_list);
}
