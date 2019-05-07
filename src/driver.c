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

#include <libfdt/libfdt.h>
#include <errno.h>

#include "driver.h"

static struct list_head _driver_list = LIST_HEAD_INIT(_driver_list);

static int _init_subnodes(void *fdt, int root){
	int node;
	fdt_for_each_subnode(node, fdt, root){
        struct device_driver *driver;

		char path[32];
		fdt_get_path(fdt, node, path, sizeof(path));

		printk(PRINT_SYSTEM ">> %s\n", path);

		bool found_driver = false;

        list_for_each_entry(driver, &_driver_list, list){
            if(!driver->compatible || !driver->probe) continue;
			int r = 0;
			r = fdt_node_check_compatible(fdt, node, driver->compatible);
			if(r == 1){
			} else if(fdt_first_property_offset(fdt, root) >= 0 && r == -FDT_ERR_NOTFOUND){
				// the node has properties but no compatible string (nodes without properties are allowed not to have it)
				found_driver = true;
				break;
			} else if(r == 0){
				found_driver = true;
                if((r = driver->probe(fdt, node)) < 0){
					printk(PRINT_ERROR "devicetree: failed to probe %s (error(%d): %s)\n", path, r, strerror(-r));
				}
                break;
            }
        }

		if(!found_driver){
			printk(PRINT_ERROR "devicetree: driver for %s was not included in the firmware\n", path);
		}
		// initialize children recursively
		_init_subnodes(fdt, node);
	}
	return 0;
}

static int _deinit_subnodes(void *fdt, int root){
	int node;
	fdt_for_each_subnode(node, fdt, root){
        struct device_driver *driver;
		_deinit_subnodes(fdt, node);
        list_for_each_entry(driver, &_driver_list, list){
			if(!driver->compatible || !driver->remove) continue;
			if(fdt_node_check_compatible(fdt, node, driver->compatible) == 0){
				driver->remove(fdt, node);
				break;
			}
		}
	}
	return 0;
}

int probe_device_drivers(void *fdt){
	if(fdt_check_header(fdt) != 0) return -EINVAL;

	return _init_subnodes(fdt, 0);
}

int remove_device_drivers(void *fdt){
	if(fdt_check_header(fdt) != 0) return -EINVAL;

	return _deinit_subnodes(fdt, 0);
}

#include <stdio.h>
void register_device_driver(struct device_driver *self){
    INIT_LIST_HEAD(&self->list);
    list_add(&self->list, &_driver_list);
}

