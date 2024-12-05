#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube tile quick configuration
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
THIS SCRIPT PERFORMS A QUICK CONFIGURATION WITH THE TILE AND WRITES THE RESULTING CONFIGURATION TO IT
- this script records angles on the tiles sensor positions and configures base stations with this data
  after performing the configuration, the script creates and saves a file which contains the configuration
  "basestation_configuration.csv" which is also sent back to the configuration device
"""

import struct
import serial
import time
import configuration_module

# geometry estimator with 1 for quick config
geo_estimator = configuration_module.BasestationGeometryEstimator(1)
# the angles which are passed to the pnp algorithm
bs_angles = [configuration_module.AzimuthElevationAngles(0, 0) for i in range(4)]

# time to wait for configuration angles [s]
configuration_time = 10

# flag if config for the specific station is already done
max_base_stations = 16
config_done = [0 for n in range(max_base_stations)]

# configuration data that was calculated
config_data = []

# serial port to tile
com_port = 7

# establish serial, be sure to wait until startup from tile is completed
arduino = serial.Serial(port='COM'+str(com_port), baudrate=115200, timeout=5)

# q for quick config
arduino.write("q\n".encode())

# timestamp for configuration time
start_time = time.time()

# record config angles until configuration time has passed
while 1:
    if (time.time() - start_time) > configuration_time:
        break

    # get a line with station index number and 8 angles - 4 sensors with azimuth and elevation each:
    # order: idx, azimuth 0, elevation 0, azimuth 1, ...
    # azimuth and elevation angle 0 is from top left sensor, whereas 3 is bottom right
    line_bytes = arduino.readline()
    # a complete data line has 36 bytes
    # (1 byte 'p' for python data + 1 byte station number + 4(sensors) * 2(azimuth&elevation) * 4(float) byte + \r\n= 36
    if len(line_bytes) != 36:
        continue

    command = line_bytes[0:1].decode()
    # check if right data
    if command != 'p':
        continue

    # data available, filter out values
    station_idx = int.from_bytes(line_bytes[1:2], 'big')

    # only perform configuration once for every station
    if config_done[station_idx] == 0:
        # get values for pnp
        float_elements = list(struct.unpack('ffffffff', line_bytes[2:34]))

        # just for debug (print angles in degree)
        #degree = [0 for i in range(8)]
        #for i in range(8):
        #    degree[i] = math.degrees(float_elements[i])
        #print("angles [°]:", degree)

        # perform pnp algorithm for configuration
        bs_angles[0].azimuth = float_elements[0]
        bs_angles[0].elevation = float_elements[1]
        bs_angles[1].azimuth = float_elements[2]
        bs_angles[1].elevation = float_elements[3]
        bs_angles[2].azimuth = float_elements[4]
        bs_angles[2].elevation = float_elements[5]
        bs_angles[3].azimuth = float_elements[6]
        bs_angles[3].elevation = float_elements[7]
        # estimate geometry of base station
        rotation, position = geo_estimator.estimate_geometry(bs_angles)

        # transfer config to tile
        # d - config data command, station index, station position, station rotation
        encoded_command = 'd'.encode()
        encoded_station_index = struct.pack('B', station_idx)
        encoded_position_heading = struct.pack('ffffffffffff', position[0], position[1], position[2],
                                               rotation[0][0], rotation[0][1], rotation[0][2],
                                               rotation[1][0], rotation[1][1], rotation[1][2],
                                               rotation[2][0], rotation[2][1], rotation[2][2])
        encoded_newline = '\n'.encode()
        configuration_data = encoded_command + encoded_station_index + encoded_position_heading + encoded_newline
        arduino.write(configuration_data)

        # save data to save it in config file on pc
        data_row = [station_idx, position[0], position[1], position[2], rotation[0][0], rotation[0][1], rotation[0][2],
                    rotation[1][0], rotation[1][1], rotation[1][2], rotation[2][0], rotation[2][1], rotation[2][2]]
        config_data.append(data_row)

        # print configuration
        print("Configuration for station", station_idx, "added!")
        print(data_row)

        # set flag for one time calculation done
        config_done[station_idx] = 1

# stop configuration and continue tile in normal mode
arduino.write("s\n".encode())

# close serial connection
arduino.close()

# save configuration in file
configuration_module.save_config(config_data)
