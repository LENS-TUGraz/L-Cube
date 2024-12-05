#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube MQTT quick configuration
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
THIS SCRIPT PERFORMS A QUICK CONFIGURATION WITH THE CUBE AND WRITES THE RESULTING CONFIGURATION TO IT
- only supported with one mqtt client during configuration
"""

import paho.mqtt.client as mqtt
import struct
import time
import pkg_resources
import math
installed = {pkg.key for pkg in pkg_resources.working_set}
import configuration_module


# MQTT connection
TOPIC = "#"
BROKER_ADDRESS = "172.20.10.2"
PORT = 1883

# geometry estimator with 1 for quick config
geo_estimator = configuration_module.BasestationGeometryEstimator(1)

# the angles which are passed to the pnp algorithm
bs_angles = [configuration_module.AzimuthElevationAngles(0, 0) for i in range(4)]

# contrains configuration stream that is sent from cube to tile after they have been calculated
configuration_array = []

# flag that mqtt message was sent
data_sent = 0

def on_message(client, userdata, message):
    print("message topic: ", message.topic)

    if(message.topic == "c") or (message.topic == "q"):
        # skip own sent data
        return

    global data_sent
    if(message.topic == "bs_data") or (message.topic == "s"):
        data_sent = 1

    if message.topic == "p":
        # evaluate received data
        # (1 byte 'p' for python data + 1 byte station number + 4(sensors) * 2(azimuth&elevation) * 4(float) byte = 34
        if len(message.payload) != 34:
            print("[ERROR] invalid config data length")
            return

        command = message.payload[0:1].decode()
        if command != 'p':
            return

        # data available, filter out values
        station_idx = int.from_bytes(message.payload[1:2], 'big')

        if config_done[station_idx] == 0:
            # get values for pnp
            float_elements = list(struct.unpack('ffffffff', message.payload[2:34]))

            # just for debug (print angles in degree)
            #degree_elements = []
            #for i in range(8):
            #   degree_elements.append(math.degrees(float_elements[i]))
            #print("angles [°]:", degree_elements)

            # perform pnp algorithm for configuration
            bs_angles[0].azimuth = float_elements[0]
            bs_angles[0].elevation = float_elements[1]
            bs_angles[1].azimuth = float_elements[2]
            bs_angles[1].elevation = float_elements[3]
            bs_angles[2].azimuth = float_elements[4]
            bs_angles[2].elevation = float_elements[5]
            bs_angles[3].azimuth = float_elements[6]
            bs_angles[3].elevation = float_elements[7]
            rotation, position = geo_estimator.estimate_geometry(bs_angles)

            # transfer config to tile
            # d - for data command, station index, station position, station rotation
            encoded_command = 'd'.encode()
            encoded_station_index = struct.pack('B', station_idx)
            encoded_position_heading = struct.pack('ffffffffffff', position[0], position[1], position[2],
                                                   rotation[0][0], rotation[0][1], rotation[0][2],
                                                   rotation[1][0], rotation[1][1], rotation[1][2],
                                                   rotation[2][0], rotation[2][1], rotation[2][2])
            encoded_newline = '\n'.encode()
            configuration_data = encoded_command + encoded_station_index + encoded_position_heading + encoded_newline
            # save in configuration array to send datastream to cube and tile after config time has passed
            configuration_array.append(configuration_data)

            print("Configuration for station", station_idx, "added!")
            # save data to store in config file on pc
            data_row = [station_idx, position[0], position[1], position[2], rotation[0][0], rotation[0][1],
                        rotation[0][2],
                        rotation[1][0], rotation[1][1], rotation[1][2], rotation[2][0], rotation[2][1], rotation[2][2]]
            config_data.append(data_row)

            # set flag for one time calculation
            config_done[station_idx] = 1
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


# calculated configuration data
config_data = []

# time for performing configuration angle recording [s]
configuration_time = 10

# time between configurations are sent to cube
t_between_sent_configurations = 0.5

# one time calculation flag for configuration
max_base_stations = 16
config_done = [0 for n in range(max_base_stations)]

# start mqtt connection
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
# connect to broker
client.connect(BROKER_ADDRESS, PORT)

# if handling should be in this thread
#client.loop_forever()
# do mqtt stuff in other thread
client.loop_start()

# start by publishing to 'q' for quick config
client.publish("q", "1")

# tile switches to config mode now and records data
# this data gets received in callback
time.sleep(configuration_time)

# publish configuration data to cube now
for i in range(len(configuration_array)):

    data_sent = 0
    client.publish("bs_data", configuration_array[i])

    # wait until command is sent
    while data_sent == 0:
        time.sleep(t_between_sent_configurations)

    # give esp time to distribute data
    time.sleep(t_between_sent_configurations)

# finish with stop command to set tile and cube controller to normal mode again
data_sent = 0
client.publish("s", "1")
# wait until command is sent
while data_sent == 0:
    time.sleep(t_between_sent_configurations)

# disconnect from mqtt broker
client.disconnect()

#save file
configuration_module.save_config(config_data)
