#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube visualization variables across modules
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
Contains the global variables that get received for visualization and used in the different modules
"""

# cube1
position = [0.0, 0.0, 0.0]
front_vector = [1.0, 0.0, 0.0]
up_vector = [0.0, 0.0, 1.0]
is_crossing = 0
# for recording of cube
data = []
# cube2
position2 = [0.3, 0.3, 0.0]
front_vector2 = [1.0, 0.0, 0.0]
up_vector2 = [0.0, 0.0, 1.0]
is_crossing2 = 0
# tile
position_tile = [-0.3, -0.3, 0.0]
front_vector_tile = [1.0, 0.0, 0.0]
up_vector_tile = [0.0, 0.0, 1.0]
is_crossing_tile = 0

# dataset for violin plot
violin_dataset = [0]

# state of button
button_state = False
# to stop receiving threads
stop = 0
