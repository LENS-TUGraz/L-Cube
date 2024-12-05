#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube MQTT write configuration MQTT
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
THIS SCRIPT WRITES AND EXISTING CONFIGURATION TO THE CUBES VIA MQTT
"""

import time
import paho.mqtt.client as mqtt
import csv
import struct

# config file to write to tile
#configuration_file = 'basestation_configuration.csv'
# uncomment to set all to 0
configuration_file = 'reset_basestation_configuration.csv'

# subscribe to all topics
TOPIC = "#"
# MQTT connection
BROKER_ADDRESS = "172.20.10.2"
PORT = 1883

# flag to ensure mqtt commands get sent
data_sent = 0

# callback on subscribed topics
def on_message(client, userdata, message):
    # skip all received data
    #print("message topic: ", message.topic)

    # check that command gets sent
    global data_sent
    if (message.topic == "bs_data") or (message.topic == "s"):
        data_sent = 1


# subscribe to topic to wait until data is sent
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        # success for connection
        print("Connected to MQTT Broker: " + BROKER_ADDRESS)
        # start subscriptions
        client.subscribe(TOPIC)
    else:
        print("Error connection")
        print(reason_code)


# setup mqtt connection
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_ADDRESS, PORT)

# do mqtt handling in other thread
client.loop_start()

# read and send configuration file to cube via mqtt
with open(configuration_file, encoding='UTF8', newline='') as f:
    reader = csv.reader(f)

    for row in reader:
        # check for header
        if row[0].isnumeric() is False:
            continue

        # transfer config to tile
        # d - for data command, station index, station position, station rotation
        # -> this gets directly sent from cube to tile
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

        # publish data to cube
        client.publish("bs_data", configuration_data)

        # wait until tile processed data
        time.sleep(0.5)

# finish with stop command
data_sent = 0
client.publish("s", "1")
# wait until command is sent
while data_sent == 0:
    time.sleep(0.5)

# disconnect from mqtt broker
client.disconnect()
