#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube write tile configuration serial
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
THIS SCRIPT WRITES AND EXISTING CONFIGURATION TO A TILE VIA SERIAL CONNECTION
- we do this by using the quick configuration procedure, but do not receive data, only transmit configuration
- tile sets led 1 to green when a configuration is received (0.5s per config)
"""

import struct
import serial
import csv
import time

# config file to write to tile
configuration_file = 'normal_basestation_configuration.csv'
# uncomment to set configuration data to 0
#configuration_file = 'reset_basestation_configuration.csv'

# port of tile
com_port = 7

#-------------------------------------------------------------------------------------
# establish serial, be sure to wait until startup from tile is completed
arduino = serial.Serial(port='COM'+str(com_port), baudrate=115200, timeout=5)

# q for quick config
arduino.write("q\n".encode())

# wait until tile changed mode
time.sleep(0.05)
# send d for config data
# send s for stop config

with open(configuration_file, encoding='UTF8', newline='') as f:
    reader = csv.reader(f)

    for row in reader:
        # check for header
        if row[0].isnumeric() is False:
            print(row)
            print("skipped")
            continue

        # transfer config to tile
        # d - for config data command - station index, station position, station rotation follow
        encoded_command = 'd'.encode()

        encoded_station_index = struct.pack('B', int(row[0]))
        encoded_position_heading = struct.pack('ffffffffffff',
                                               # position
                                               float(row[1]), float(row[2]), float(row[3]),
                                               # heading
                                               float(row[4]), float(row[5]), float(row[6]),
                                               float(row[7]), float(row[8]), float(row[9]),
                                               float(row[10]), float(row[11]), float(row[12]))
        encoded_newline = '\n'.encode()
        configuration_data = encoded_command + encoded_station_index + encoded_position_heading + encoded_newline
        arduino.write(configuration_data)
        print(row[0])

        # wait until tile processed data
        time.sleep(0.05)

# stop configuration and continue tile in normal mode
arduino.write("s\n".encode())

arduino.close()
