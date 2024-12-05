#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube roll, pitch, yaw conversion
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
Script to convert forward and up vector to roll, pitch and yaw
"""

import math
import numpy as np

# vectors that contain heading
forward_vector = [1, 0, 0]
up_vector = [0, 0, 1]

# positive pitch is leaning forward (a drone flying forward)
# negative pitch is leaning backward (a drone flying backward)
# positive roll is leaning to right side (a drone flying right)
# negative roll is leaning to left side (a drone flying left)
# positive yaw is looking to left side (a drone turning left)
# negative yaw is looking to right side (a drone turning right)

def normalize_vector(vector):
    magnitude = np.linalg.norm(vector)
    if magnitude == 0:
        # Avoid division by zero if the vector is already a zero vector
        return vector
    return vector / magnitude

# normalize vectors first
forward_vector = normalize_vector(forward_vector)
up_vector = normalize_vector(up_vector)

#conversion to yaw pitch and roll
yaw = math.atan2(forward_vector[1], forward_vector[0])
pitch = -math.asin(forward_vector[2])
plane_right_x = math.sin(yaw)
plane_right_y = -math.cos(yaw)
roll = math.asin(up_vector[0]*plane_right_x + up_vector[1]*plane_right_y)

if up_vector[2] < 0:
    #print("sign adaption of roll")
    roll = np.sign(roll) * math.pi - roll

# convert to degree
yaw_degree = math.degrees(yaw)
pitch_degree = math.degrees(pitch)
roll_degree = math.degrees(roll)

print("pitch: ", pitch_degree)
print("roll: ", roll_degree)
print("yaw: ", yaw_degree)
