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
* FILE ............... src/encoder.c
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
#include <errno.h>

#include "encoder.h"

DEFINE_DEVICE_CLASS(encoder)

#if 0
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
#endif
