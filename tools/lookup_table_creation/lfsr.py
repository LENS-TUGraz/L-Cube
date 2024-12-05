#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  LFSR class
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
Contains class to represent a lfsr
Parts addapted from: https://github.com/mat-eng/LH2-positioning/blob/master/Signal_processing/lfsr.py
"""

class LFSR:
    def __init__(self, poly, start=1):
        self.state = self.start = start
        self.poly = poly

    def reset(self, start=1):
        self.state = start

    def enter_loop(self):
        for i in range(2**17):
            next(self)
        return self

    def parity(self, bb):
        b = bb
        b ^= b >> 16
        b ^= b >> 8
        b ^= b >> 4
        b ^= b >> 2
        b ^= b >> 1
        b &= 1
        return b

    def __next__(self):
        b = self.state & self.poly
        b = self.parity(b)
        self.state = (self.state << 1) | b
        self.state &= (1 << 17) - 1
        return self.state

    def __iter__(self):
        return self