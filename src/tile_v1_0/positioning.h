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
#ifndef POSITIONING_
#define POSITIONING_

#include <stdint.h>

// calculates a global position and heading out of the sensors positions
void calculate_global_position_and_heading(uint8_t omitted_sensor);

// performs positioning of the sensors after decoded beams
void perform_positioning();

// update dirty bits of sensors (get incremented every 20 ms to signal lifetime)
void update_dirty_bits();

#endif
