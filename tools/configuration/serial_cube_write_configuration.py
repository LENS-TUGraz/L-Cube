#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube write cube configuration serial
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
THIS SCRIPT WRITES AND EXISTING CONFIGURATION TO A CUBE VIA SERIAL CONNECTION
- tile sets led 1 to green when a configuration is received (0.5s per config line)
"""

import struct
import serial
import csv
import time

# config file to write to tile
configuration_file = 'basestation_configuration.csv'
# uncomment to reset configuration
#configuration_file = 'reset_basestation_configuration.csv'

# port of cube
com_port = 17

#-------------------------------------------------------------------------------------
# establish serial, be sure to wait until startup from tile is completed
arduino = serial.Serial(port='COM'+str(com_port), baudrate=115200, timeout=5)
# send d for config data
# send s for stop config
# in case serial dbg is activated -> cube waits for communication with other participant until startup
time.sleep(0.5)
# read system startup output if required
while arduino.inWaiting() > 0:
    arduino.readline()
    #print(arduino.readline())

# short delay
time.sleep(0.5)

with open(configuration_file, encoding='UTF8', newline='') as f:
    reader = csv.reader(f)

    for row in reader:
        # check for header
        if row[0].isnumeric() is False:
            continue

        # transfer config to tile
        # d - for data command, station index, station position, station rotation
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

        print("send config", int(row[0]))

        # wait until tile processed and forwarded data to tile
        time.sleep(0.3)

# stop configuration and continue tiles in normal mode
encoded_stop = 's'.encode()
encoded_newline = '\n'.encode()
# send stop command
stop_command = encoded_stop + encoded_newline
arduino.write(stop_command)
print("send stop")
# wait until stop is delivered
time.sleep(1)

# read output of cube if something is in queue
while arduino.inWaiting() > 0:
    arduino.readline()
    # print(arduino.readline())

arduino.close()
#-------------------------------------------------------------------------------------
