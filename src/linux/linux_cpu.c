#include "linux_cpu.h"

int _x86_cpu_probe(void *fdt, int fdt_node){
	struct x86_cpu *self = kzmalloc(sizeof(struct x86_cpu));
	//serial_set_printk_port(&self->dev.ops);
	return 0;
}

DEVICE_DRIVER(x86_cpu, "cpu,x86", _x86_cpu_probe, _x86_cpu_remove);
