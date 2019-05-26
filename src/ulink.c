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
* FILE ............... src/ulink.c
* AUTHOR ............. Martin K. Schröder
* VERSION ............ Not tagged
* DATE ............... 2019-05-26
* GIT ................ https://github.com/mkschreder/libfirmware
* WEBSITE ............ http://swedishembedded.com
* LICENSE ............ Swedish Embedded Open Source License
*
*          Copyright (C) 2014-2019 Martin Schröder. All rights reserved.
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
#include <errno.h>
#include <stddef.h>
#include <string.h>

#include "ulink.h"

#define ULINK_FRAME_BYTE 0x7e
#define ULINK_ESCAPE_BYTE 0x7d

static const uint16_t crc16_tab[] = {
		0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
		0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
		0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
		0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
		0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
		0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
		0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
		0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
		0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
		0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
		0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
		0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
		0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
		0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
		0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
		0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
		0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
		0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
		0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
		0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
		0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
		0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
		0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
		0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
		0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
		0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
		0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
		0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
		0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
		0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
		0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
		0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
};

static uint16_t _crc16(uint16_t crc_in, const void *buf, int size){
        const uint8_t *p = (const uint8_t*)buf;
        uint16_t crc = crc_in;
        while (size--)
                crc = crc16_tab[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
        return crc;
}

enum {
	ULINK_STATE_WAIT, 
	ULINK_STATE_READ, 
	ULINK_STATE_ESCAPE,
	ULINK_STATE_DONE
}; 

static size_t _ulink_pack_data(const void *_data, size_t size, void *_packet_buf, size_t *packet_size, size_t max_packet_size){
	uint8_t *data = (uint8_t*)_data;
	uint8_t *packet_buf = (uint8_t*)_packet_buf;
	size_t c; 
	size_t max_size = max_packet_size - *packet_size; 
	for(c = 0; c < size; c++){
		uint8_t ch = data[c];  
		switch(ch){
			case ULINK_FRAME_BYTE:
			case ULINK_ESCAPE_BYTE: {
				*packet_buf++ = ULINK_ESCAPE_BYTE; (*packet_size)++;  
				if(*packet_size == max_size) break;  
				*packet_buf++ = ch ^ 0x20; (*packet_size)++;  
				if(*packet_size == max_size) break; 
			} break; 
			default:  {	
				*packet_buf++ = ch; (*packet_size)++;  
				if(*packet_size == max_size) break;  
			} break; 
		}
	}
	return c; 
}

void ulink_frame_init(struct ulink_frame *self){
	memset(self, 0, sizeof(*self));
	self->state = ULINK_STATE_READ; 
}

size_t ulink_frame_pack(struct ulink_frame *self, const void *_data, size_t size){
	const char *data = (const char*)_data;
	self->size = 0; 
	// pack the data first (and escape it)
	size_t s = _ulink_pack_data(data, size, self->buf, &self->size, ULINK_MAX_FRAME_SIZE - 5); 
	// calculate crc of the escaped data
	uint16_t crc = _crc16(0, data, (int)s); 
	uint8_t cd[2] = { (uint8_t)(crc >> 8), (uint8_t)(crc & 0xff) }; 
	// also escape the crc in case it contains key characters
	_ulink_pack_data(cd, 2, self->buf + self->size, &self->size, ULINK_MAX_FRAME_SIZE);  
	// append frame ending byte
	self->buf[self->size++] = ULINK_FRAME_BYTE; 
	return s; 
}

bool ulink_frame_valid(struct ulink_frame *self){
	return self->state == ULINK_STATE_DONE && self->size > 0; 
}

const void *ulink_frame_data(struct ulink_frame *self){
	return self->buf; 
}

size_t ulink_frame_size(struct ulink_frame *self){
	return self->size; 
}

size_t ulink_frame_to_buffer(struct ulink_frame *self, char *data, size_t size){
	size_t s = self->size; 
	if(size < s) s = size; 
	memcpy(data, self->buf, s); 
	return s; 
}

size_t ulink_frame_try_parse(struct ulink_frame *self, const void *_packet, size_t size){
	const char *packet = (const char*)_packet;
	if(self->state == ULINK_STATE_DONE) {
		self->state = ULINK_STATE_READ; 
		self->size = 0; 
	}
	size_t c; 
	for(c = 0; c < size; c++){
		char byte = *packet++; 
		switch(byte){
			case ULINK_FRAME_BYTE: {
				// new frame so we validate the data in the parsed buffer and return number of bytes processed
				if(self->size < 2) goto error; 

				uint16_t packet_crc = (uint16_t)((((uint16_t)self->buf[self->size - 2] & 0xff) << 8) | (uint16_t)(self->buf[self->size - 1] & 0xff)); 
				uint16_t calc_crc = _crc16(0, self->buf, (int)(self->size - 2)); 

				if(calc_crc != packet_crc) goto error; 

				self->size -= 2; // subtract crc bytes
				self->state = ULINK_STATE_DONE; 

				return c + 1; 
			} break; 
			case ULINK_ESCAPE_BYTE: {
				self->state = ULINK_STATE_ESCAPE; 
			} break; 
			default: {
				switch(self->state){
					case ULINK_STATE_READ: {
						self->buf[self->size++] = byte; 
					} break; 
					case ULINK_STATE_ESCAPE: {
						// only 0x5d and 0x5e are valid escaped chars
						if(byte != 0x5d && byte != 0x5e) goto error; 
						self->buf[self->size++] = byte ^ 0x20; 
						self->state = ULINK_STATE_READ; 
					} break; 
				}
				if(self->size == ULINK_MAX_FRAME_SIZE) goto error; 
			} break; 
		}
		continue; 
error: 
		self->size = 0; 
		self->state = ULINK_STATE_READ; 
	}
	return c; 
}


