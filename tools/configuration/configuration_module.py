#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube configuration module
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
Module that contains functions for performing pnp config algorithm and saving config data to file
"""

import math
import numpy as np
import pkg_resources
installed = {pkg.key for pkg in pkg_resources.working_set}
import cv2 as cv
import csv


"""
Class modified from bitcraze: used for angle representation of an azimuth, elevation pair
link: https://github.com/bitcraze/crazyflie-lib-python/blob/master/cflib/localization/lighthouse_bs_vector.py
"""
class AzimuthElevationAngles:
    # class used to represent an azimuth-elevation angle pair (used in radiant for pnp algorithm)
    def __init__(self, azimuth_angle: float, elevation_angle: float) -> None:
        # azimuth_angle:
        #   0° - straight forward, looking left (from base station) is positive, right is negative
        # elevation_angle:
        #   0° - straight forward, looking up (from base station) is positive, down is negative
        self.azimuth = azimuth_angle
        self.elevation = elevation_angle

"""
Class modified from bitcraze: position estimation of the base stations
(we use other reference coordinate system, our indexes, ...)
link: https://github.com/bitcraze/crazyflie-lib-python/blob/master/cflib/localization/lighthouse_bs_geo.py
"""
# global reference frame:
#
#           x
#           A
#           |
#           |
# y <-------0 z

# open cv reference frame:
#  2--------1
#  |   z    |
#  |   0 x  | ((x) y-axis into monitor)
#  |        |
#  3--------0
class BasestationGeometryEstimator:
    # class used to get a base stations position and heading

    def __init__(self, config_system: int = 0, x_offset: float = 0.0, y_offset: float = 0.0, square_size: float = 1.0):
        # config_system:
        #  - 0 for advanced configuration - 1m square
        #  - 1 for quick configuration - hardware sensor positions

        # set offset to coordinates
        self.xof = x_offset
        self.yof = y_offset
        self.square_length = square_size

        # initialize dictionary for yaw initial guess based on azimuth angle to sensors
        self._directions = {
            self._hash_sensor_order([0, 2, 3, 1]): math.radians(0),
            self._hash_sensor_order([0, 2, 1, 3]): math.radians(25),
            self._hash_sensor_order([0, 1, 2, 3]): math.radians(65),
            self._hash_sensor_order([1, 0, 2, 3]): math.radians(90),
            self._hash_sensor_order([1, 0, 3, 2]): math.radians(115),
            self._hash_sensor_order([1, 3, 0, 2]): math.radians(155),
            self._hash_sensor_order([3, 1, 0, 2]): math.radians(180),
            self._hash_sensor_order([3, 1, 2, 0]): math.radians(205),
            self._hash_sensor_order([3, 2, 1, 0]): math.radians(245),
            self._hash_sensor_order([2, 3, 1, 0]): math.radians(270),
            self._hash_sensor_order([2, 3, 0, 1]): math.radians(295),
            self._hash_sensor_order([2, 0, 3, 1]): math.radians(335),
        }

        # set the points which are used for pnp algorithm
        self._lighthouse_3d = []
        if config_system == 0:
            # configuration point positions in open cv coordinates
            # opencv: x and z are ground plane, y goes to bottom
            # (z is global x, x is negative y, y is negative z)
            # for comparison: global system: x forward, y left,
            # z high (when looking into x direction) (x-y is ground plane)
            #  2--------1
            #  |   z    |
            #  |   0 x  | ((x) y-axis into monitor)
            #  |        |
            #  3--------0
            self._lighthouse_3d = np.float32(
                [
                    # adapted configuration's positions
                    [0 - self.yof, 0, 0 + self.xof],
                    [0 - self.yof, 0, self.square_length + self.xof],
                    [-self.square_length - self.yof, 0, self.square_length + self.xof],
                    [-self.square_length - self.yof, 0, 0 + self.xof]
                ])
        else:
            # the receiver's sensor distance
            sensor_distance_width = 0.035
            sensor_distance_length = 0.035
            # quick configuration (sensor positions of hardware are used)
            # 0---1
            # |   |
            # 2---3
            self._lighthouse_3d = np.float32(
                [

                    [-sensor_distance_width / 2, 0, sensor_distance_length / 2],
                    [sensor_distance_width / 2, 0, sensor_distance_length / 2],
                    [-sensor_distance_width / 2, 0, -sensor_distance_length / 2],
                    [sensor_distance_width / 2, 0, -sensor_distance_length / 2]
                ])

        # camera matrix
        self._K = np.float64(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]
            ])

        # distance coefficient passed to pnp algorithm
        self._dist_coef = np.zeros(4)

    # function to call for estimation of geometry
    def estimate_geometry(self, bs_vectors):
        # estimate the geometry of a base station
        # bs_vectors: the four azimuth-elevation angle pairs in radiant, received on the configuration positions
        # rotation: Rotation matrix of the base station heading
        # position: Position vector of the base station

        # yaw gets guessed first to find initial guesses for rotation and position
        guess_yaw = self._find_initial_yaw_guess(bs_vectors)
        rvec_guess, tvec_guess = self._convert_yaw_to_open_cv(guess_yaw)
        rw_ocv, tw_ocv = self._estimate_position_pnp(bs_vectors, rvec_guess, tvec_guess)
        rotation, position = self._opencv_to_global(rw_ocv, tw_ocv)
        return rotation, position


    # guess initial yaw based on azimuth angles
    def _find_initial_yaw_guess(self, bs_vectors):
        # assumes base station is slightly looking down and sorts sensor's azimuth angles which are then looked up
        # in the dictionary, containing the approximated base station yaw angle for this order
        sweeps_x = {
            # as positions are changed for this configuration method
            0: bs_vectors[2].azimuth,
            1: bs_vectors[1].azimuth,
            2: bs_vectors[3].azimuth,
            3: bs_vectors[0].azimuth
        }

        ordered_map = {k: v for k, v in sorted(sweeps_x.items(), key=lambda item: item[1])}
        sensor_order = list(ordered_map.keys())

        # the base station is roughly looks in this direction (global reference frame)
        try:
            return self._directions[self._hash_sensor_order(sensor_order)]
        except KeyError:
            #print(sensor_order)
            return self._directions[self._hash_sensor_order([3, 1, 2, 0])]

    # hash sensor orders to create keys for lookup
    def _hash_sensor_order(self, order):
        hash = 0
        for i in range(4):
            hash += order[i] * 4 ** i
        return hash

    # creates initial guess based on estimated yaw and assuming the base station is mounted
    # 2m high and 1m distance to the configuration point
    def _convert_yaw_to_open_cv(self, yaw):
        # height of base station
        bs_h = 2.0
        # floor distance to base station
        bs_fd = 1.0
        # straight line distance to base station
        bs_dist = math.sqrt(bs_h ** 2 + bs_fd ** 2)
        # elevation angle to the assumed point
        elevation = math.atan2(bs_h, bs_fd)

        # initial position of the receiving hardware in camera coordinate system, open cv style
        # global x-axis (where base station is looking) is z-axis in open cv
        # assumes base station is directly looking to configuration square
        tvec_start = np.array([0, 0, bs_dist])

        # rotation matrix gets calculated by rotating around x and y-axis in open cv
        # (which are y and z-axis in global system)
        d_c = math.cos(-yaw + math.pi)
        d_s = math.sin(-yaw + math.pi)
        R_rot_y = np.array([
            [d_c, 0.0, d_s],
            [0.0, 1.0, 0.0],
            [-d_s, 0.0, d_c],
        ])
        e_c = math.cos(elevation)
        e_s = math.sin(elevation)
        R_rot_x = np.array([
            [1.0, 0.0, 0.0],
            [0.0, e_c, -e_s],
            [0.0, e_s, e_c],
        ])
        R = np.dot(R_rot_x, R_rot_y)
        rvec_start, _ = cv.Rodrigues(R)

        return rvec_start, tvec_start

    # pnp algorithm to estimate position and rotation of base station
    def _estimate_position_pnp(self, bs_vectors, rvec_start, tvec_start):
        # Sensors as seen by the "camera"
        lighthouse_image_projection = np.float32(
            [
                [-math.tan(bs_vectors[0].azimuth), -math.tan(bs_vectors[0].elevation)],
                [-math.tan(bs_vectors[1].azimuth), -math.tan(bs_vectors[1].elevation)],
                [-math.tan(bs_vectors[2].azimuth), -math.tan(bs_vectors[2].elevation)],
                [-math.tan(bs_vectors[3].azimuth), -math.tan(bs_vectors[3].elevation)]
            ])

        _ret, rvec_est, tvec_est = cv.solvePnP(
            self._lighthouse_3d,
            lighthouse_image_projection,
            self._K,
            self._dist_coef,
            flags=cv.SOLVEPNP_ITERATIVE,
            rvec=rvec_start,
            tvec=tvec_start,
            useExtrinsicGuess=True)

        if not _ret:
            raise Exception('No solution found')

        Rw_ocv, Tw_ocv = self._cam_to_world(rvec_est, tvec_est)
        return Rw_ocv, Tw_ocv

    # transform cv rodrigues vectors and rotations to regular coordinates and rotations
    def _cam_to_world(self, rvec_c, tvec_c):
        R_c, _ = cv.Rodrigues(rvec_c)
        R_w = np.linalg.inv(R_c)
        tvec_w = -np.matmul(R_w, tvec_c)
        return R_w, tvec_w

    # transform coordinates from opencv to global reference frame
    def _opencv_to_global(self, Rotation_cv, transition_cv):
        R_opencv_to_global = np.array([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ])

        R_global_to_opencv = np.array([
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ])

        transition_global = np.dot(R_opencv_to_global, transition_cv)
        Rotation_global = np.dot(R_opencv_to_global, np.dot(Rotation_cv, R_global_to_opencv))
        return Rotation_global, transition_global


# saves the configuration in csv file
def save_config(data_for_file):
    # write calculated position data to csv file
    header = ['station_index',
              'position_x', 'position_y', 'position_z',
              'rotation_00', 'rotation_01', 'rotation_02',
              'rotation_10', 'rotation_11', 'rotation_12',
              'rotation_20', 'rotation_21', 'rotation_22']

    with open('basestation_configuration.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)

        # write the header
        writer.writerow(header)
        for i in range(len(data_for_file)):
            writer.writerow(data_for_file[i])



