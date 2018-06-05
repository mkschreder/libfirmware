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


#pragma once

/**
 * Derivative tracker that implements a linear derivative tracker as described by Salim Ibrir
 * in paper Linear time derivative trackers 2002. The linear time varying system looks like this
 *
 * Martin Schröder 2018
 *
 * A = [0, 1; 0, 0]; B = [0; 1]; C = [1, 0]; P = [2.5000e-07, -2.5000e-05; -2.5000e-05, 5.0000e-03];
 * dx = Ad * x - Bd * Bd' * inv(P) * (x - Cd' * y);
 * dP = -gain * P - P * Ad' - Ad * P + Bd * Bd';
 */

struct dx_tracker {
	float P[2][2];
	float x, dx;
	float gain;
};

void dx_tracker_init(struct dx_tracker *self, float gain);
void dx_tracker_update(struct dx_tracker *self, float x, float dt);
float dx_tracker_get_x(struct dx_tracker *self);
float dx_tracker_get_dx(struct dx_tracker *self);
