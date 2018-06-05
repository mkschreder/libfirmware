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

#include <math.h>
#include "math.h"

#define SINE_TABLE_LENGTH 512
static const int8_t _sine_table[SINE_TABLE_LENGTH] = {
0   ,1   ,3   ,4   ,6   ,7   ,9   ,10  ,12  ,13  ,15  ,17  ,18  ,20  ,21  ,23  ,24  ,26  ,27  ,29  ,30  ,32  ,33  ,35  ,36  ,38  ,39  ,41  ,42  ,44  ,45  ,47  ,
48  ,50  ,51  ,52  ,54  ,55  ,57  ,58  ,59  ,61  ,62  ,63  ,65  ,66  ,67  ,69  ,70  ,71  ,73  ,74  ,75  ,76  ,78  ,79  ,80  ,81  ,82  ,84  ,85  ,86  ,87  ,88  ,
89  ,90  ,91  ,93  ,94  ,95  ,96  ,97  ,98  ,99  ,100 ,101 ,102 ,102 ,103 ,104 ,105 ,106 ,107 ,108 ,108 ,109 ,110 ,111 ,112 ,112 ,113 ,114 ,114 ,115 ,116 ,116 ,
117 ,117 ,118 ,119 ,119 ,120 ,120 ,121 ,121 ,121 ,122 ,122 ,123 ,123 ,123 ,124 ,124 ,124 ,125 ,125 ,125 ,125 ,126 ,126 ,126 ,126 ,126 ,126 ,126 ,126 ,126 ,126 ,
127 ,126 ,126 ,126 ,126 ,126 ,126 ,126 ,126 ,126 ,126 ,125 ,125 ,125 ,125 ,124 ,124 ,124 ,123 ,123 ,123 ,122 ,122 ,121 ,121 ,121 ,120 ,120 ,119 ,119 ,118 ,117 ,
117 ,116 ,116 ,115 ,114 ,114 ,113 ,112 ,112 ,111 ,110 ,109 ,108 ,108 ,107 ,106 ,105 ,104 ,103 ,102 ,102 ,101 ,100 ,99  ,98  ,97  ,96  ,95  ,94  ,93  ,91  ,90  ,
89  ,88  ,87  ,86  ,85  ,84  ,82  ,81  ,80  ,79  ,78  ,76  ,75  ,74  ,73  ,71  ,70  ,69  ,67  ,66  ,65  ,63  ,62  ,61  ,59  ,58  ,57  ,55  ,54  ,52  ,51  ,50  ,
48  ,47  ,45  ,44  ,42  ,41  ,39  ,38  ,36  ,35  ,33  ,32  ,30  ,29  ,27  ,26  ,24  ,23  ,21  ,20  ,18  ,17  ,15  ,13  ,12  ,10  ,9   ,7   ,6   ,4   ,3   ,1   ,
0   ,-1  ,-3  ,-4  ,-6  ,-7  ,-9  ,-10 ,-12 ,-13 ,-15 ,-17 ,-18 ,-20 ,-21 ,-23 ,-24 ,-26 ,-27 ,-29 ,-30 ,-32 ,-33 ,-35 ,-36 ,-38 ,-39 ,-41 ,-42 ,-44 ,-45 ,-47 ,
-48 ,-50 ,-51 ,-52 ,-54 ,-55 ,-57 ,-58 ,-59 ,-61 ,-62 ,-63 ,-65 ,-66 ,-67 ,-69 ,-70 ,-71 ,-73 ,-74 ,-75 ,-76 ,-78 ,-79 ,-80 ,-81 ,-82 ,-84 ,-85 ,-86 ,-87 ,-88 ,
-89 ,-90 ,-91 ,-93 ,-94 ,-95 ,-96 ,-97 ,-98 ,-99 ,-100,-101,-102,-102,-103,-104,-105,-106,-107,-108,-108,-109,-110,-111,-112,-112,-113,-114,-114,-115,-116,-116,
-117,-117,-118,-119,-119,-120,-120,-121,-121,-121,-122,-122,-123,-123,-123,-124,-124,-124,-125,-125,-125,-125,-126,-126,-126,-126,-126,-126,-126,-126,-126,-126,
-127,-126,-126,-126,-126,-126,-126,-126,-126,-126,-126,-125,-125,-125,-125,-124,-124,-124,-123,-123,-123,-122,-122,-121,-121,-121,-120,-120,-119,-119,-118,-117,
-117,-116,-116,-115,-114,-114,-113,-112,-112,-111,-110,-109,-108,-108,-107,-106,-105,-104,-103,-102,-102,-101,-100,-99 ,-98 ,-97 ,-96 ,-95 ,-94 ,-93 ,-91 ,-90 ,
-89 ,-88 ,-87 ,-86 ,-85 ,-84 ,-82 ,-81 ,-80 ,-79 ,-78 ,-76 ,-75 ,-74 ,-73 ,-71 ,-70 ,-69 ,-67 ,-66 ,-65 ,-63 ,-62 ,-61 ,-59 ,-58 ,-57 ,-55 ,-54 ,-52 ,-51 ,-50 ,
-48 ,-47 ,-45 ,-44 ,-42 ,-41 ,-39 ,-38 ,-36 ,-35 ,-33 ,-32 ,-30 ,-29 ,-27 ,-26 ,-24 ,-23 ,-21 ,-20 ,-18 ,-17 ,-15 ,-13 ,-12 ,-10 ,-9  ,-7  ,-6  ,-4  ,-3  ,-1  
};

int8_t fastsin_127u16(uint16_t x){
	return _sine_table[(x) >> 7];
}

int8_t fastcos_127u16(uint16_t x){
	return _sine_table[(x + (0xffff / 4)) >> 7];
}

int8_t fastsin_127u32(uint32_t x){
	return _sine_table[(x) >> 23];
}

int8_t fastcos_127u32(uint32_t x){
	return _sine_table[(x + (0xffffffff / 4)) >> 23];
}

int8_t fastsin_127deg(uint16_t x){
	return (_sine_table[((x) * SINE_TABLE_LENGTH / 360) & (SINE_TABLE_LENGTH - 1)]);
}

int8_t fastcos_127deg(uint16_t x){
	return (_sine_table[(((x) + 90) * SINE_TABLE_LENGTH / 360) & (SINE_TABLE_LENGTH - 1)]);
}

float fastsinf(float x){
	return (_sine_table[(uint16_t)(x * (SINE_TABLE_LENGTH >> 1) / M_PI) & (SINE_TABLE_LENGTH - 1)] / 127.0f);
}

float fastcosf(float x){
	return (_sine_table[(uint16_t)((x * (SINE_TABLE_LENGTH >> 1) / M_PI) + (90 * 512 / 360)) & (SINE_TABLE_LENGTH - 1)] / 127.0f);
}

void clamp_rad360(float *a){
	// NOTE: this presupposes that overflow is smaller than one period
	if(*a > RAD_360)
		*a -= RAD_360;
	if(*a < 0)
		*a = RAD_360 + *a;
}

