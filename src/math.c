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
* FILE ............... src/math.c
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
	return (_sine_table[(uint16_t)((x * (SINE_TABLE_LENGTH >> 1) / M_PI) + (90 * SINE_TABLE_LENGTH / 360)) & (SINE_TABLE_LENGTH - 1)] / 127.0f);
}

void clamp_rad360(float *a){
	// NOTE: this presupposes that overflow is smaller than one period
	if(*a > RAD_360)
		*a -= RAD_360;
	if(*a < 0)
		*a = RAD_360 + *a;
}

