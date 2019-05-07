#include <stdio.h>

#include <libfdt/libfdt.h>

#include "leds.h"
#include "driver.h"

#define MAX_LEDS 8
struct linux_leds {
    bool leds[MAX_LEDS];
	struct leds_device dev;
};

static void print_leds(struct linux_leds *self){
    printf("LEDS: ");
    for(int c = 0; c < MAX_LEDS; c++){
        if(self->leds[c]) printf("o");
        else printf("_");
    }
    printf("\r");
    fflush(stdout);
}

void _linux_leds_on(leds_device_t leds, uint8_t id){
	struct linux_leds *self = container_of(leds, struct linux_leds, dev.ops);
	if(id >= MAX_LEDS) return;
    self->leds[id] = true;
    print_leds(self);
}

void _linux_leds_off(leds_device_t leds, uint8_t id){
	struct linux_leds *self = container_of(leds, struct linux_leds, dev.ops);
	if(id >= MAX_LEDS) return;
    self->leds[id] = false;
    print_leds(self);
}

void _linux_leds_toggle(leds_device_t leds, uint8_t id){
	struct linux_leds *self = container_of(leds, struct linux_leds, dev.ops);
	if(id >= MAX_LEDS) return;
    self->leds[id] = !self->leds[id];
    print_leds(self);
}

static const struct leds_device_ops _led_ops = {
	.on = _linux_leds_on,
	.off = _linux_leds_off,
	.toggle = _linux_leds_toggle,
};

static int _linux_leds_probe(void *fdt, int fdt_node){
	struct linux_leds *self = kzmalloc(sizeof(struct linux_leds));

	leds_device_init(&self->dev, fdt, fdt_node, &_led_ops);
	leds_device_register(&self->dev);

	return 0;
}

static int _linux_leds_remove(void *fdt, int fdt_node){
	// TODO 
    return -1;
}

DEVICE_DRIVER(linux_leds, "gnu,linux_leds", _linux_leds_probe, _linux_leds_remove)
