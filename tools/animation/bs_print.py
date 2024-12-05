#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube visualization base station drawer
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
Prints the base stations which have been configured beforehand in the configuration folder to the visualization
"""

import numpy as np
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform
import csv

def calculate_bs_cube_coords(position, fw_vector, up_vector):
    scale = 3.0
    width = 0.07
    depth = 0.06
    height = 0.08
    w_2 = width / 2 * scale
    d_2 = depth / 2 * scale
    h_2 = height / 2 * scale
    side_vector = np.cross(fw_vector, up_vector)
    # calc coordinates of edges
    c0 = position + fw_vector * d_2 + side_vector * w_2 - up_vector * h_2
    c1 = position + fw_vector * d_2 - side_vector * w_2 - up_vector * h_2
    c2 = position - fw_vector * d_2 + side_vector * w_2 - up_vector * h_2
    c3 = position - fw_vector * d_2 - side_vector * w_2 - up_vector * h_2
    c4 = position + fw_vector * d_2 + side_vector * w_2 + up_vector * h_2
    c5 = position + fw_vector * d_2 - side_vector * w_2 + up_vector * h_2
    c6 = position - fw_vector * d_2 + side_vector * w_2 + up_vector * h_2
    c7 = position - fw_vector * d_2 - side_vector * w_2 + up_vector * h_2
    return [c0, c1, c2, c3, c4, c5, c6, c7]


# cube base:
#    x
#  y o
# bottom:
#  0  1
#  2  3
# top:
#  4  5
#  6  7
def draw_bs(draw_axis, coords):
    c0 = coords[0]
    c1 = coords[1]
    c2 = coords[2]
    c3 = coords[3]
    c4 = coords[4]
    c5 = coords[5]
    c6 = coords[6]
    c7 = coords[7]

    # bottom
    draw_axis.plot_surface(np.array([[c0[0], c1[0]], [c2[0], c3[0]]]), np.array([[c0[1], c1[1]], [c2[1], c3[1]]]), np.array([[c0[2], c1[2]], [c2[2], c3[2]]]), color='gray', edgecolor='black')
    # top
    draw_axis.plot_surface(np.array([[c4[0], c5[0]], [c6[0], c7[0]]]), np.array([[c4[1], c5[1]], [c6[1], c7[1]]]), np.array([[c4[2], c5[2]], [c6[2], c7[2]]]), color='gray', edgecolor='black')
    # front
    draw_axis.plot_surface(np.array([[c0[0], c1[0]], [c4[0], c5[0]]]), np.array([[c0[1], c1[1]], [c4[1], c5[1]]]), np.array([[c0[2], c1[2]], [c4[2], c5[2]]]), color='gray', edgecolor='black')
    # back
    draw_axis.plot_surface(np.array([[c2[0], c3[0]], [c6[0], c7[0]]]), np.array([[c2[1], c3[1]], [c6[1], c7[1]]]), np.array([[c2[2], c3[2]], [c6[2], c7[2]]]), color='gray', edgecolor='black')
    # rside
    draw_axis.plot_surface(np.array([[c1[0], c3[0]], [c5[0], c7[0]]]), np.array([[c1[1], c3[1]], [c5[1], c7[1]]]), np.array([[c1[2], c3[2]], [c5[2], c7[2]]]), color='gray', edgecolor='black')
    # lside
    draw_axis.plot_surface(np.array([[c0[0], c2[0]], [c4[0], c6[0]]]), np.array([[c0[1], c2[1]], [c4[1], c6[1]]]), np.array([[c0[2], c2[2]], [c4[2], c6[2]]]), color='gray', edgecolor='black')

def read_configuration():

    with open('../configuration/basestation_configuration.csv', encoding='UTF8', newline='') as f:
        reader = csv.reader(f)

        #struct for cube coordinates
        cube_coords = []

        for row in reader:
            # check for header
            if row[0].isnumeric() is False:
                continue

            # read config line
            #index = int(row[0])
            position = np.array([float(row[1]), float(row[2]), float(row[3])])
            forward_vec = np.array([float(row[4]), float(row[7]), float(row[10])])
            up_vec = np.array([float(row[6]), float(row[9]), float(row[12])])
            #up_vec = np.array([float(row[10]), float(row[11]), float(row[12])])

            cube_coords.append(calculate_bs_cube_coords(position, forward_vec, up_vec))

        return cube_coords


# ============================================================================================
# class for direction arrows
class Arrow3D(FancyArrowPatch):
    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)
    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)
        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)
    def do_3d_projection(self, renderer=None):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)
        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        return np.min(zs)
# ============================================================================================


# for drawing of heading vectors of base stations
def plot_basestation_direction(axis, station, up_vector, front_vector):
    scale = 0.4
    # position of drone
    axis.scatter3D(station[0], station[1], station[2], c='y', marker='o')
    # front vector
    arrow_f = Arrow3D(station[0], station[1], station[2], station[3]*scale, station[6]*scale, station[9]*scale,
                      mutation_scale=2, ec='red')
    axis.add_artist(arrow_f)
    # side vector
    side_vector = np.cross(up_vector, front_vector)
    side_vector = side_vector / np.linalg.norm(side_vector)
    arrow_s = Arrow3D(station[0], station[1], station[2], station[4]*scale, station[7]*scale, station[10]*scale,
                      mutation_scale=2, ec='green')
    axis.add_artist(arrow_s)
    # up vector
    arrow_u = Arrow3D(station[0], station[1], station[2], station[5]*scale, station[8]*scale, station[11]*scale,
                      mutation_scale=2, ec='blue')
    axis.add_artist(arrow_u)


