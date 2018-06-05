/*
	Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ULINK_MAX_FRAME_SIZE 512

struct ulink_frame {
	char buf[ULINK_MAX_FRAME_SIZE]; 
	size_t size; 
	uint8_t state; 
}; 

//size_t ulink_pack_data(const void *data, size_t size, struct ulink_frame *frame);
size_t ulink_frame_pack(struct ulink_frame *self, const void *_data, size_t size);
size_t ulink_frame_try_parse(struct ulink_frame *frame, const void *data, size_t size);

void ulink_frame_init(struct ulink_frame *self); 
const void *ulink_frame_data(struct ulink_frame *self); 
size_t ulink_frame_size(struct ulink_frame *self); 
bool ulink_frame_valid(struct ulink_frame *self); 
size_t ulink_frame_to_buffer(struct ulink_frame *self, char *data, size_t max_size); 

// NOTE: make sure frame is initialized before calling this function for the first time
#define ulink_for_each_frame(frame, buffer, buffer_size) \
	for(size_t _rlen = 0; \
		(_rlen += ulink_frame_try_parse((frame), (buffer) + _rlen, (size_t)((size_t)(buffer_size) - _rlen))) && ulink_frame_valid(frame); \
		)

#ifdef __cplusplus
}
#endif


