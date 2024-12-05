#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube tile normal configuration
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
THIS SCRIPT PERFORMS A NORMAL CONFIGURATION WITH THE TILE AND WRITES THE RESULTING CONFIGURATION TO IT
- this script records angles on four positions used for the base station configuration
- after performing the configuration, the script creates and saves a file which contains the configuration
  "basestation_configuration.csv" which is also sent back to the configuration device
- angles from sensor 0 are used only -> place sensor 0 on required positions
- if configuration is not on the origin (0,0) -> change xof and yof fo offsets in x and y direction
- if configuration square is not 1 meter -> change square_size to specific size
"""

import struct
import serial
import time
import configuration_module

# this functions records the received python data for every of the four position and saves the data into an array
def record_data(current_round):
    global received_angles
    while 1:
        # read data from tile
        line_bytes = arduino.readline()
        #print(line_bytes)

        # a complete data line has 12 bytes
        # (1 byte 'p' for python data + 1 byte station number + (2(azimuth&elevation) * 4(float)) byte + \r\n = 12
        if len(line_bytes) != 12:
            # r for ready for next position
            try:
                if (len(line_bytes) == 3) and (line_bytes[0:1].decode() == 'r'):
                    break

                # in case there is '\n' in the received numeric data
                if line_bytes[0:1].decode() == 'p':
                    missing_len = 12 - len(line_bytes)
                    missing_line_bytes = arduino.read(missing_len)
                    line_bytes = line_bytes + missing_line_bytes
                    #there is \n in data, receive next line and append
                    pass
                else:
                    continue
            except UnicodeDecodeError:
                print("[ERROR] - line not decodable")
                continue

        command = line_bytes[0:1].decode()
        # 'p'-command for python data
        if command != 'p':
            continue

        # data available, filter out values
        station_idx = int.from_bytes(line_bytes[1:2], 'big')
        #print(station_idx)

        # get values for pnp
        float_elements = list(struct.unpack('ff', line_bytes[2:10]))
        #print(float_elements)

        # save values into struct for configuration
        received_angles[station_idx][(current_round * 2)] = float_elements[0]
        received_angles[station_idx][(current_round * 2) + 1] = float_elements[1]
    return


########################################################################################################################
############################################################
# change if configuring at other position than (0,0)
square_size = 1
x_offset = -0.5
y_offset = -0.5
############################################################

# geometry estimator with 0 for normal config
geo_estimator = configuration_module.BasestationGeometryEstimator(0, x_offset, y_offset, square_size)

# time between sending a station's configuration data
t_between_sent_configurations = 0.5

# port of the tile
# 7 for our test tile
com_port = 7

# array that contains the calculatedconfiguration
config_data = []

# the struct where the angles are saved to, which are later passed to the pnp algorithm
max_base_stations = 16
received_angles = [[None for k in range(8)] for n in range(max_base_stations)]

# the angles which are passed to the pnp algorithm
bs_angles = [configuration_module.AzimuthElevationAngles(0, 0) for i in range(4)]

# establish serial, be sure to wait until startup from tile is completed
arduino = serial.Serial(port='COM'+str(com_port), baudrate=115200, timeout=5)

# 'c'-command for normal config
arduino.write("c\n".encode())

# wait a second to allow rp to switch to configuration mode
time.sleep(1)

print("Place sensor 0 on first position (", format(x_offset, '.2f'), ",", format(y_offset, '.2f'), ") and then press Enter")
input("Press Enter to continue...")
# send command to log on first position
arduino.write("1\n".encode())
# tile records angles on D4 for one second, averages them and returns the angle to the base station
record_data(0)

print("Place sensor 0 on second position (", format(x_offset+square_size, '.2f'), ",", format(y_offset, '.2f'), ") and then press Enter")
input("Press Enter to continue...")
# send command to log on second position
arduino.write("2\n".encode())
# tile records angles on D4 for one second, averages them and returns the angle to the base station
record_data(1)

print("Place sensor 0 on third position (", format(x_offset+square_size, '.2f'), ",", format(y_offset+square_size, '.2f'), ") and then press Enter")
input("Press Enter to continue...")
# send command to log on third position
arduino.write("3\n".encode())
# tile records angles on D4 for one second, averages them and returns the angle to the base station
record_data(2)

print("Place sensor 0 on fourth position (", format(x_offset, '.2f'), ",", format(y_offset+square_size, '.2f'), ") and then press Enter")
input("Press Enter to continue...")
# send command to log on fourth position
arduino.write("4\n".encode())
# tile records angles on D4 for one second, averages them and returns the angle to the base station
record_data(3)

# wait a second to switch arduino into waiting mode
time.sleep(1)

# set to state 5, so tile waits for configuration
arduino.write("5\n".encode())

for station_index in range(max_base_stations):
    # check for enough angles for configuration
    for k in range(8):
        if received_angles[station_index][k] is None:
            break

        # if this is reached, enough data is availabe
        if k == 7:
            # just for debug with station 0 (print angles in degree)
            #degree = [0 for i in range(8)]
            #for i in range(8):
            #    degree[i] = math.degrees(received_angles[0][i])
            #print("angles [degree]:", degree)

            # perform pnp algorithm for configuration
            bs_angles[0].azimuth = received_angles[station_index][0]
            bs_angles[0].elevation = received_angles[station_index][1]
            bs_angles[1].azimuth = received_angles[station_index][2]
            bs_angles[1].elevation = received_angles[station_index][3]
            bs_angles[2].azimuth = received_angles[station_index][4]
            bs_angles[2].elevation = received_angles[station_index][5]
            bs_angles[3].azimuth = received_angles[station_index][6]
            bs_angles[3].elevation = received_angles[station_index][7]
            rotation, position = geo_estimator.estimate_geometry(bs_angles)

            # transfer configuration to tile
            # d - for data command, station index, station position, station rotation
            encoded_command = 'd'.encode()
            encoded_station_index = struct.pack('B', station_index)
            encoded_position_heading = struct.pack('ffffffffffff', position[0], position[1], position[2],
                                                   rotation[0][0], rotation[0][1], rotation[0][2],
                                                   rotation[1][0], rotation[1][1], rotation[1][2],
                                                   rotation[2][0], rotation[2][1], rotation[2][2])
            encoded_newline = '\n'.encode()
            configuration_data = encoded_command + encoded_station_index + encoded_position_heading + encoded_newline
            arduino.write(configuration_data)
            # give tile a bit of time to save configuration
            time.sleep(t_between_sent_configurations)

            # save data to store in config file on pc
            data_row = [station_index, position[0], position[1], position[2], rotation[0][0], rotation[0][1],
                        rotation[0][2],
                        rotation[1][0], rotation[1][1], rotation[1][2], rotation[2][0], rotation[2][1], rotation[2][2]]

            # print the configuration
            print("Configuration for station", station_index, "sent!")
            print(data_row)
            config_data.append(data_row)


# stop configuration and continue tile in normal mode
arduino.write("s\n".encode())
# close serial connection
arduino.close()

# save data to file
configuration_module.save_config(config_data)

