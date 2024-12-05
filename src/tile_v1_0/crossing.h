/*
 * L-Cube Tile Software
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

// includeguard
#ifndef CROSSING_
#define CROSSING_

#include "mmath.h"

// performs a crossing calculation with two beams from different base stations
// returns -1 if failed, 0 if success
int perform_crossing(uint8_t station_index_1, uint32_t lfsr_index_1_1, uint32_t lfsr_index_1_2, uint8_t station_index_2, uint32_t lfsr_index_2_1, uint32_t lfsr_index_2_2, vec3d sensor_position);

#endif