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

#include "dx_tracker.h"
#include <string.h>

void dx_tracker_init(struct dx_tracker *self, float gain){
	memset(self, 0, sizeof(*self));
	self->P[0][0] =  2.5000e-07;
	self->P[0][1] = -2.5000e-05;
	self->P[1][0] = -2.5000e-05;
	self->P[1][1] =  5.0000e-03;
	self->gain = gain;
}

void dx_tracker_update(struct dx_tracker *self, float x, float dt){
	// equations derived using sympy and octave
	#define P self->P
	float gamma = -self->gain;
	float dx = self->dx;
	float ddx = 
		-(P[0][0] * dx)/(P[0][0] * P[1][1] - P[0][1] * P[1][0]) + 
		(P[1][0] * (self->x - x)) / (P[0][0] * P[1][1] - P[0][1] * P[1][0]);
	float dp[2][2] = {
		{ gamma * P[0][0] - P[0][1] - P[1][0], gamma * P[0][1] - P[1][1] },
		{ gamma * P[1][0] - P[1][1], gamma * P[1][1] + 1 }
	};
	#undef P
	self->x += dx * dt;
	self->dx += ddx * dt;
	self->P[0][0] += dp[0][0] * dt;
	self->P[0][1] += dp[0][1] * dt;
	self->P[1][0] += dp[1][0] * dt;
	self->P[1][1] += dp[1][1] * dt;
}

float dx_tracker_get_x(struct dx_tracker *self){
	return self->x;
}

float dx_tracker_get_dx(struct dx_tracker *self){
	return self->dx;
}
