#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube MQTT cube normal configuration
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
THIS SCRIPT PERFORMS A NORMAL CONFIGURATION WITH THE CUBE'S TOP TILE AND MQTT AND WRITES THE RESULTING CONFIGURATION TO
ALL TILES OF THE CUBE
- this script records angles on four positions used for the base station configuration
- after performing the configuration, the script creates and saves a file which contains the configuration
  "basestation_configuration.csv" which is also sent back to the configuration device
- angles from sensor 0 are used only -> place sensor 0 on required positions
- if configuration is not on the origin (0,0) -> change xof and yof fo offsets in x and y direction
- if configuration square is not 1 meter -> change square_size to specific size
- only supported with one mqtt client during configuration time
"""

import paho.mqtt.client as mqtt
import struct
import time
import math
import configuration_module

# MQTT connection
TOPIC = "#"
BROKER_ADDRESS = "172.20.10.2"
PORT = 1883

########################################################################################################################
############################################################
# change if configuring at other position than (0,0)
square_size = 1
x_offset = -0.5
y_offset = -0.5
############################################################

# geometry estimator with 0 for normal config
geo_estimator = configuration_module.BasestationGeometryEstimator(0, x_offset, y_offset, square_size)


# the angles which are passed to the pnp algorithm
bs_angles = [configuration_module.AzimuthElevationAngles(0, 0) for i in range(4)]

# current round of angle recording at specific configuration position
current_state = 0

# array that contains data for publishing calculated configurations
configuration_array = []

# flag that data was sent to cube
data_sent = 0

# flag that client is ready for next round (it has published all recorded angle pairs at current position)
client_ready = 0

# received recorded angles at positions
max_base_stations = 16
received_angles = [[None for k in range(8)] for n in range(max_base_stations)]

def on_message(client, userdata, message):
    print("message topic: ", message.topic)

    if(message.topic == "c") or (message.topic == "q"):
        # skip own sent data
        return

    global data_sent
    if(message.topic == "bs_data") or (message.topic == "s"):
        data_sent = 1

    global client_ready
    if message.topic == "r":
        client_ready = 1

    if message.topic == "p":
        # evaluate and save received configuration data
        # (1 byte 'p' for python data + 1 byte station number + 4(sensors) * 2(azimuth&elevation) = 10
        if len(message.payload) != 10:
            print("invalid config data length")
            return

        command = message.payload[0:1].decode()
        if command != 'p':
            return

        # data available, filter out values
        station_idx = int.from_bytes(message.payload[1:2], 'big')
        #print(station_idx)

        # get values for pnp
        float_elements = list(struct.unpack('ff', message.payload[2:10]))
        #print(float_elements)

        # save values
        global current_state
        received_angles[station_idx][((current_state - 1) * 2)] = float_elements[0]
        received_angles[station_idx][((current_state - 1) * 2) + 1] = float_elements[1]
        return


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        # success for connection
        print("Connected to MQTT Broker: " + BROKER_ADDRESS)
        # start subscriptions
        client.subscribe(TOPIC)
    else:
        print("Error connection")
        print(reason_code)


# contains calculated configurations
config_data = []

# time between calculated configurations that are sent to cube
t_between_sent_configurations = 0.5

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER_ADDRESS, PORT)

# if handling should be in this thread
#client.loop_forever()

# do mqtt stuff in other thread
client.loop_start()

# start by publishing to 'c' for normal config
client.publish("c", "1")

#let tile switch mode
time.sleep(t_between_sent_configurations)

print("Place tile on first position (", format(x_offset, '.2f'), ",", format(y_offset, '.2f'), ") and then press Enter")
input("Press Enter to continue...")
current_state = 1
# publish current round
client.publish("1", "1")

# tile now records angles on D4 for one second, averages them and returns the angles via mqtt callback
while client_ready == 0:
    time.sleep(t_between_sent_configurations)
client_ready = 0

print("Place tile on second position (", format(x_offset+square_size, '.2f'), ",", format(y_offset, '.2f'), ") and then press Enter")
input("Press Enter to continue...")
current_state = 2
# publish current round
client.publish("2", "1")

# tile now records angles on D4 for one second, averages them and returns the angles via mqtt callback
while client_ready == 0:
    time.sleep(t_between_sent_configurations)
client_ready = 0

print("Place tile on third position (", format(x_offset+square_size, '.2f'), ",", format(y_offset+square_size, '.2f'), ") and then press Enter")
input("Press Enter to continue...")
current_state = 3
# publish current round
client.publish("3", "1")

# tile now records angles on D4 for one second, averages them and returns the angles via mqtt callback
while client_ready == 0:
    time.sleep(t_between_sent_configurations)
client_ready = 0

print("Place tile on fourth position (", format(x_offset, '.2f'), ",", format(y_offset+square_size, '.2f'), ") and then press Enter")
input("Press Enter to continue...")
current_state = 4
# publish current round
client.publish("4", "1")

# tile now records angles on D4 for one second, averages them and returns the angles via mqtt callback
while client_ready == 0:
    time.sleep(t_between_sent_configurations)
client_ready = 0

# set tile to data-waiting-mode
client.publish("5", "1")

# calculate configuration
for station_index in range(max_base_stations):
    for k in range(8):
        if received_angles[station_index][k] is None:
            break

        if k == 7:
            # configuration is possible for this index, perform it
            bs_angles[0].azimuth = received_angles[station_index][0]
            bs_angles[0].elevation = received_angles[station_index][1]
            bs_angles[1].azimuth = received_angles[station_index][2]
            bs_angles[1].elevation = received_angles[station_index][3]
            bs_angles[2].azimuth = received_angles[station_index][4]
            bs_angles[2].elevation = received_angles[station_index][5]
            bs_angles[3].azimuth = received_angles[station_index][6]
            bs_angles[3].elevation = received_angles[station_index][7]
            rotation, position = geo_estimator.estimate_geometry(bs_angles)

            # transfer stream for config from cube to tile
            # d - for data command, station index, station position, station rotation
            encoded_command = 'd'.encode()
            encoded_station_index = struct.pack('B', station_index)
            encoded_position_heading = struct.pack('ffffffffffff', position[0], position[1], position[2],
                                                   rotation[0][0], rotation[0][1], rotation[0][2],
                                                   rotation[1][0], rotation[1][1], rotation[1][2],
                                                   rotation[2][0], rotation[2][1], rotation[2][2])
            encoded_newline = '\n'.encode()
            configuration_data = encoded_command + encoded_station_index + encoded_position_heading + encoded_newline

            print("Configuration for station", station_index, "added!")
            configuration_array.append(configuration_data)

            # save data to store in config file on pc
            data_row = [station_index, position[0], position[1], position[2], rotation[0][0], rotation[0][1],
                        rotation[0][2],
                        rotation[1][0], rotation[1][1], rotation[1][2], rotation[2][0], rotation[2][1], rotation[2][2]]

            # print configuration
            print(data_row)
            config_data.append(data_row)

# publish configuration data to cube
for i in range(len(configuration_array)):
    data_sent = 0
    client.publish("bs_data", configuration_array[i])

    # wait until command is sent
    while data_sent == 0:
        time.sleep(t_between_sent_configurations)

    # give esp time to distribute data
    time.sleep(t_between_sent_configurations)

# finish with stop command
data_sent = 0
client.publish("s", "1")
# wait until command is sent
while data_sent == 0:
    time.sleep(t_between_sent_configurations)

# disconnect from mqtt broker
client.disconnect()

#save file
configuration_module.save_config(config_data)
