/*
 * L-Cube Calibration Record
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

#include <Arduino.h>
#include "lookup.h"
#include "table/lookup_tables.h"

/*
// rpi with arduino cannot do non aligned memory lookups like this!
plu = lookup_0 + ((beam & MASK) * 3);
pp = *((uint32_t*)plu);
*/

// shorten this array according to MAX_LFSR_INDEX define
const uint8_t* lookups[] = {lookup_0, lookup_1, lookup_2, lookup_3, lookup_4, lookup_5, lookup_6, lookup_7, lookup_8, lookup_9, lookup_10, lookup_11/*, lookup_12, lookup_13, lookup_14, lookup_15, lookup_16, lookup_17, lookup_18, lookup_19, lookup_20, lookup_21, lookup_22, lookup_23, lookup_24, lookup_25, lookup_26, lookup_27, lookup_28, lookup_29, lookup_30, lookup_31/**/};

// returnvalue of 1 indicates
int get_unique_index(uint64_t beam, uint32_t* p_index, uint8_t* p_lfsr)
{
  uint32_t index = 0;
  uint32_t index1 = 0;
  uint32_t index2 = 0;
  uint8_t lfsr = 0;

  for(int i = 0; i < MAX_LFSR_INDEX; ++i)
  {
    // check each lfsr for match
    index1 = 0;
    index1 += lookups[i][(beam & MASK) * 3] << 16;
    index1 += lookups[i][((beam & MASK) * 3) + 1] << 8;
    index1 += lookups[i][((beam & MASK) * 3) + 2];

    index2 = 0;
    index2 += lookups[i][((beam >> 17) & MASK) * 3] << 16;
    index2 += lookups[i][(((beam >> 17) & MASK) * 3) + 1] << 8;
    index2 += lookups[i][(((beam >> 17) & MASK) * 3) + 2];
    if((index1 - index2) == 17)
    {
      *p_lfsr = i;
      *p_index = index1;
      return 1;
    }
  }

  // no success in lookup
  return 0;
}
