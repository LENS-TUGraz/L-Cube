/*
 * L-Cube Initial Test
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
#ifndef LOOKUP_
#define LOOKUP_

// 32 for all 16 stations (each base station contains 2 lookup tables)
#define MAX_LFSR_INDEX 2
#define MASK 0x1FFFF

// recovers lfsr index of a beam
int get_unique_index(uint64_t beam, uint32_t* p_index, uint8_t* p_lfsr);

// includeguard
#endif
