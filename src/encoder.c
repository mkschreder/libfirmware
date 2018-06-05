#include <libfdt/libfdt.h>
#include <errno.h>

#include "encoder.h"
#include "list.h"
#include "thread.h"
#include "work.h"
#include "time.h"
#include "math.h"
#include "driver.h"

static LIST_HEAD(_encoders);

void _encoder_update_estimate(struct work *work){
	struct encoder_device *self = container_of(work, struct encoder_device, work);
	int32_t raw = 0;
	if(self->ops && self->ops->read)
		raw = self->ops->read(&self->ops);
	float sensed_pos = (float)raw / ((float)self->counts_per_unit * (float)self->unit_scale);

	uint32_t t = (uint32_t)micros();
	float dt = constrain_float(1e-6 * (float)u32_diff(t, self->last_update_micros), 0, 0.2);
	self->last_update_micros = t;

	thread_mutex_lock(&self->mx);
	dx_tracker_update(&self->dx_track, sensed_pos, dt);
	thread_mutex_unlock(&self->mx);

	queue_work(work, self->update_interval_ms);
}

void encoder_device_init(struct encoder_device *self, int fdt_node, const struct encoder_ops *ops){
	memset(self, 0, sizeof(*self));
	INIT_LIST_HEAD(&self->list);
	self->fdt_node = fdt_node;
	self->ops = ops;

	self->counts_per_unit = (uint32_t)fdt_get_int_or_default(_devicetree, (int)fdt_node, "counts_per_unit", 10000);
	self->unit_scale = (uint32_t)fdt_get_int_or_default(_devicetree, (int)fdt_node, "unit_scale", 2);
	self->update_interval_ms = (uint32_t)fdt_get_int_or_default(_devicetree, (int)fdt_node, "update_interval", 1);
	self->observer_gain = (float)fdt_get_int_or_default(_devicetree, (int)fdt_node, "observer_gain", 200);

	dx_tracker_init(&self->dx_track, 200);

	thread_mutex_init(&self->mx);

	work_init(&self->work, _encoder_update_estimate);
}

int encoder_device_register(struct encoder_device *self){
	BUG_ON(!self);
	BUG_ON(!self->ops);
	BUG_ON(!self->ops->read);
	list_add_tail(&self->list, &_encoders);
	queue_work(&self->work, 0);
	return 0;
}

encoder_t encoder_find(const char *dtb_path){
	struct encoder_device *encoder;
	int node = fdt_path_offset(_devicetree, dtb_path);
	if(node < 0) return NULL;
	list_for_each_entry(encoder, &_encoders, list){
		if(encoder->fdt_node == node) return &encoder->ops;
	}
	return NULL;
}

float encoder_get_position(encoder_t encoder){
	struct encoder_device *self = container_of(encoder, struct encoder_device, ops);
	thread_mutex_lock(&self->mx);
	float p = dx_tracker_get_x(&self->dx_track);
	thread_mutex_unlock(&self->mx);
	return p;
}

float encoder_get_velocity(encoder_t encoder){
	struct encoder_device *self = container_of(encoder, struct encoder_device, ops);
	thread_mutex_lock(&self->mx);
	float v = dx_tracker_get_dx(&self->dx_track);
	thread_mutex_unlock(&self->mx);
	return v;
}
