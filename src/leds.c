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

#include "thread.h"
#include "leds.h"
#include "list.h"
#include "driver.h"

#include <errno.h>

static LIST_HEAD(_leds);

int leds_register(void *fdt, int fdt_node, led_controller_t ctrl){
	struct leds_device *leds = kzmalloc(sizeof(struct leds_device));
	leds->ctrl = ctrl;
	leds->fdt_node = (int)fdt_node;
	list_add_tail(&leds->list, &_leds);
}

led_controller_t leds_find(const char *dtb_path){
	struct leds_device *leds;
	int node = fdt_path_offset(_devicetree, dtb_path);
	if(node < 0) return NULL;
	list_for_each_entry(leds, &_leds, list){
		if(leds->fdt_node == node) return leds->ctrl;
	}
	return NULL;
}
