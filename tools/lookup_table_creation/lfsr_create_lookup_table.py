#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  LFSR lookup table creation
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Script to create the lookup table header files used for decoding in tiles
"""

import numpy as np
from lfsr import LFSR

polys = [0x0001D258, 0x00017E04,
         0x0001FF6B, 0x00013F67,
         0x0001B9EE, 0x000198D1,
         0x000178C7, 0x00018A55,
         0x00015777, 0x0001D911,
         0x00015769, 0x0001991F,
         0x00012BD0, 0x0001CF73,
         0x0001365D, 0x000197F5,
         0x000194A0, 0x0001B279,
         0x00013A34, 0x0001AE41,
         0x000180D4, 0x00017891,
         0x00012E64, 0x00017C72,
         0x00019C6D, 0x00013F32,
         0x0001AE14, 0x00014E76,
         0x00013C97, 0x000130CB,
         0x00013750, 0x0001CB8D]

# Compute all LFSR
x = 32
lookups_ = [[0 for i in range(2**17)] for y in range(x)]

u = np.zeros((2**17, x), dtype=int)
for i in range(0, x):
    lfsr = LFSR(polys[i])
    for j in range(0, 2**17):
        u[j, i]=next(lfsr)
        lookups_[i][u[j,i]] = j
    print('LFSR ', i, ' calculated')

#uint8_entries -> would require 12MB in total
# save table for c
# table that contains byte values, to save space as otherwise lookup not possible
# -> (rp2040 memory access restriction) (cannot perform lookups on parts of 64 byte word)
help_lookups_ = [[0 for i in range((2**17)*3)] for y in range(x)]
for j in range(x):
    for i in range(2**17):
        help_lookups_[j][(i * 3)]     = ((lookups_[j][i] & 0x10000) >> 16)
        help_lookups_[j][(i * 3) + 1] = ((lookups_[j][i] & 0x0FF00) >> 8)
        help_lookups_[j][(i * 3) + 2] = ((lookups_[j][i] & 0x000FF))
        pass

for j in range(x):
    head = "const uint8_t lookup_" + str(j) + "[] = {"
    file = "lookup_" + str(j) + ".h"
    np.savetxt(file, [help_lookups_[j]], fmt='%d', delimiter=',', header=head, footer="};")

exit()

"""
#uint32_entries -> would require 16MB in total - not used in our project
# save table for c
for j in range(x):
    head = "const uint32_t lookup_" + str(j) + "[] = {"
    file = "lookup_" + str(j) + ".h"
    np.savetxt(file, [lookups_[j]], fmt='%d', delimiter=',', header=head, footer="};")
"""
