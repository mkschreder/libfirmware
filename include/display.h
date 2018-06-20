#pragma once

#include "timestamp.h"
#include "list.h"

typedef const struct display_device_ops ** display_device_t;
typedef uint32_t color_t;

struct display_device_ops {
    int (*write_pixel)(display_device_t dev, int x, int y, color_t color);
};

#define display_write_pixel(display, x, y, color) (*(display))->write_pixel(display, x, y, color)

struct display_device {
	struct list_head list;
	const struct display_device_ops *ops;
	int fdt_node;
};

void display_device_init(struct display_device *self, int fdt_node, const struct display_device_ops *ops);
int display_device_register(struct display_device *self);
display_device_t display_find(const char *dtb_path);

#define RGB565(R, G, B) (color_rgb_565_t)(((uint16_t)((uint16_t)(R) >> 3) << 11) | ((uint16_t)(((uint16_t)G) >> 2) << 5) | ((uint16_t)(((uint16_t)B) >> 3)))
#define RGB888_TO_RGB565(col) RGB565((col >> 16) & 0xff, (col >> 8) & 0xff, (col & 0xff))

typedef uint8_t color_rgb_332_t;
typedef uint16_t color_rgb_565_t;
typedef uint32_t color_rgb_888_t;
typedef uint32_t color_rgba_888_t;
