#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube visualization receiving data module
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
Receives the data to be visualized from two cubes and one tile
"""

import math
import serial
import time
import struct
import var
import paho.mqtt.client as mqtt

# reference position for violin plot of cube 1
ref_position = [0.0, 0.0, 1.0]

###############################################################################################
# cube 1 serial
def thread_receiver_cube():
    port = 'COM15'
    # establish serial
    try:
        arduino = serial.Serial(port=port, baudrate=115200, timeout=5)
    except IOError:
        print("Connection to Cube1 could not be opened!")
        return

    # receive data ===============================================
    while 1:
        # read 37 + \r\n byte chunks (per line, including \r or \n inside the lines as data)
        line_bytes = arduino.read(39)
        if line_bytes[37:39] != b'\r\n':
            # read next line and try again
            skipped = arduino.readline()
            print("[cube1] skipped:", skipped)
            continue

        line_bytes = line_bytes[0:37]
        tile = line_bytes[0]

        # g for sweeping, h for crossing
        if (tile != ord('g')) and (tile != ord('h')):
            continue

        if tile == ord('g'):
            var.is_crossing = 0
        if tile == ord('h'):
            var.is_crossing = 1

        var.position = list(struct.unpack('fff', line_bytes[1:13]))
        var.front_vector = list(struct.unpack('fff', line_bytes[13:25]))
        var.up_vector = list(struct.unpack('fff', line_bytes[25:37]))

        # if button state is true, we need to calculate error to specific point
        if var.button_state is True:
            error = math.sqrt((var.position[0] - ref_position[0])**2 + (var.position[1]-ref_position[1])**2
                              + (var.position[2]-ref_position[2])**2)
            var.violin_dataset.append(error)

        var.data.append([tile, *var.position, *var.front_vector, *var.up_vector, time.time()])
        # ===============================================================

        if var.stop == 1:
            break

    arduino.close()

###############################################################################################
# cube 2 serial
def thread_receiver_cube2():
    port = 'COM17'
    # establish serial
    try:
        arduino = serial.Serial(port=port, baudrate=115200, timeout=5)
    except IOError:
        print("Connection to Cube2 could not be opened!")
        return

    # receive data ===============================================
    while 1:
        # read 37 + \r\n byte chunks (per line, including \r or \n inside the lines as data)
        line_bytes = arduino.read(39)
        if line_bytes[37:39] != b'\r\n':
            # read next line and try again
            skipped = arduino.readline()
            print("[cube1] skipped:", skipped)
            continue

        line_bytes = line_bytes[0:37]
        tile = line_bytes[0]

        # g for sweeping, h for crossing
        if (tile != ord('g')) and (tile != ord('h')):
            continue

        if tile == ord('g'):
            var.is_crossing2 = 0
        if tile == ord('h'):
            var.is_crossing2 = 1

        var.position2 = list(struct.unpack('fff', line_bytes[1:13]))
        var.front_vector2 = list(struct.unpack('fff', line_bytes[13:25]))
        var.up_vector2 = list(struct.unpack('fff', line_bytes[25:37]))
        # ===============================================================

        if var.stop == 1:
            break

    arduino.close()

###############################################################################################
# tile serial
def thread_receiver_tile():
    port = 'COM7'

    # establish serial
    try:
        arduino = serial.Serial(port=port, baudrate=115200, timeout=5)
    except IOError:
        print("Connection to Tile could not be opened!")
        return

    # receive data ===============================================
    while 1:
        # read 37 + \r\n byte chunks (per line, including \r or \n inside the lines as data)
        line_bytes = arduino.read(39)
        if line_bytes[37:39] != b'\r\n':
            # read next line and try again
            skipped = arduino.readline()
            print("[rec] skipped:", skipped)
            continue

        line_bytes = line_bytes[0:37]
        #print(line_bytes)

        # print("----- NEW DATA -----")
        line_bytes = line_bytes[0:134]

        tile = line_bytes[0]

        # s sweeping, t crossing
        if (tile != ord('s')) and (tile != ord('t')):
            continue

        if tile == ord('s'):
            var.is_crossing_tile = 0
        if tile == ord('t'):
            var.is_crossing_tile = 1

        var.position_tile = list(struct.unpack('fff', line_bytes[1:13]))
        var.front_vector_tile = list(struct.unpack('fff', line_bytes[13:25]))
        var.up_vector_tile = list(struct.unpack('fff', line_bytes[25:37]))

        if var.stop == 1:
            break

    arduino.close()


###############################################################################################
# cubes via mqtt
# MQTT connection
# subscribe to all and filter for needed
TOPIC = "#"
BROKER_ADDRESS = "172.20.10.2"
PORT = 1883


# callback for connect
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        # success for connection
        print("Connected to MQTT Broker: " + BROKER_ADDRESS)
        # subscribe to all topics
        client.subscribe(TOPIC)
    else:
        print("[ERROR] Error connection")
        print(reason_code)


# callback on new update of a topic
def on_message(client, userdata, message):
    # cube 1 -----------------------------------------------------------------------
    if message.topic == "LOST_CUBE_1/cube_position":
        if len(message.payload) != 12:
            print("[ERROR] invalid length LOST_CUBE_1/cube_position")
            return
        var.position = list(struct.unpack('fff', message.payload))

        # if button state is true, we need to calculate error to specific point
        if var.button_state is True:
            error = math.sqrt((var.position[0] - ref_position[0]) ** 2 + (var.position[1] - ref_position[1]) ** 2
                              + (var.position[2] - ref_position[2]) ** 2)
            var.violin_dataset.append(error)

    if message.topic == "LOST_CUBE_1/cube_mode":
        if len(message.payload) != 1:
            print("[ERROR] invalid length LOST_CUBE_1/cube_mode")
            return
        mode = message.payload.decode('ascii')
        if mode == 's':
            var.is_crossing = 0
        if mode == 'c':
            var.is_crossing = 1
        return

    if message.topic == "LOST_CUBE_1/cube_forward_vector":
        if len(message.payload) != 12:
            print("[ERROR] invalid length LOST_CUBE_1/cube_forward_vector")
            return
        var.front_vector = list(struct.unpack('fff', message.payload))
        return

    if message.topic == "LOST_CUBE_1/cube_up_vector":
        if len(message.payload) != 12:
            print("[ERROR] invalid length LOST_CUBE_1/cube_up_vector")
            return
        var.up_vector = list(struct.unpack('fff', message.payload))
        return

    # cube 2 -----------------------------------------------------------------------
    if message.topic == "LOST_CUBE_2/cube_position":
        if len(message.payload) != 12:
            print("[ERROR] invalid length LOST_CUBE_1/cube_position")
            return
        var.position2 = list(struct.unpack('fff', message.payload))

    if message.topic == "LOST_CUBE_2/cube_mode":
        if len(message.payload) != 1:
            print("[ERROR] invalid length LOST_CUBE_1/cube_mode")
            return
        mode = message.payload.decode('ascii')
        if mode == 's':
            var.is_crossing2 = 0
        if mode == 'c':
            var.is_crossing2 = 1
        return

    if message.topic == "LOST_CUBE_2/cube_forward_vector":
        if len(message.payload) != 12:
            print("[ERROR] invalid length LOST_CUBE_1/cube_forward_vector")
            return
        var.front_vector2 = list(struct.unpack('fff', message.payload))
        return

    if message.topic == "LOST_CUBE_2/cube_up_vector":
        if len(message.payload) != 12:
            print("[ERROR] invalid length LOST_CUBE_1/cube_up_vector")
            return
        var.up_vector2 = list(struct.unpack('fff', message.payload))
        return


def thread_receiver_cubes_mqtt():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER_ADDRESS, PORT)

    # do mqtt background stuff in other thread
    client.loop_start()

    # keep thread alive and wait for stop
    while var.stop == 0:
        time.sleep(1)

    # cancel other thread
    client.loop_stop()

    # disconnect from mqtt broker
    client.disconnect()
