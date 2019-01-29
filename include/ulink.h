/*
	Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.  Redistributions in binary
    form must reproduce the above copyright notice, this list of conditions and the
    following disclaimer in the documentation and/or other materials provided with
    the distribution.

    THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
    ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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


