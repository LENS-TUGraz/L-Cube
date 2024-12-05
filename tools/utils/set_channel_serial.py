#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  L-Cube set channel of lighthouse base station v2
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
Script to set a base station to the desired channel
- 1. power up base station and connect via micro-usb-cable to pc
- 2. set port first ("COM3", ... for Windows)
- 3. set channel number
- 4. enable port_set flag
- 5. start script
"""
"""
Addapted from Crazyflie
link: https://github.com/bitcraze/crazyflie-clients-python/blob/master/src/cfclient/ui/dialogs/basestation_mode_dialog.py
"""

import io
import serial
from serial.tools.list_ports import comports
import time

# list connected ports
com_ports = comports()
print("Connected PORTS:")
for port, desc, hwid in sorted(com_ports):
    print("{}: {} [{}]".format(port, desc, hwid))

# 1) set port of base station from list or from device manager
# 2) set channel
# 3) set port_set to 1 to write channel to base station
port = 'COM13'
channel = 1
port_set = 0

#check variables
if port_set == 0:
    exit("Port not set or flag not enabled")
if (channel < 1) or (channel > 16):
    exit("Incorrect channel number")

# start serial connection
ser = serial.Serial(port, timeout=0.4)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
# write channel number to device
sio.write("\r\nmode " + str(channel) + "\r\n")
sio.flush()
time.sleep(1)
# let device save the parameter
sio.write("\r\nparam save\r\n")
sio.flush()
time.sleep(1)
# check if channel was changed by requesting channel number
sio.write("\r\nmode\r\n")
sio.flush()
mode_confirm_lines = sio.readlines()
confirm_mode = None
for line in mode_confirm_lines:
    if line.startswith('Current mode: '):
        parts = line.split()
        confirm_mode = int(parts[2])
# check result
if confirm_mode is channel:
    print("SUCCESS!")
else:
    print("TRY AGAIN")
# close connection
ser.close()

