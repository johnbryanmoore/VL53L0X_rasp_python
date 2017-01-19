#!/usr/bin/python

# MIT License
#
# Copyright (c) 2017 John Bryan Moore
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import time
from ctypes import *
import smbus

VL53L0X_GOOD_ACCURACY_MODE      = 0   # Good Accuracy mode
VL53L0X_BETTER_ACCURACY_MODE    = 1   # Better Accuracy mode
VL53L0X_BEST_ACCURACY_MODE      = 2   # Best Accuracy mode
VL53L0X_LONG_RANGE_MODE         = 3   # Longe Range mode
VL53L0X_HIGH_SPEED_MODE         = 4   # High Speed mode

i2cbus = smbus.SMBus(1)

# i2c bus read callback
def i2c_read(address, reg, data_p, length):
    result = i2cbus.read_i2c_block_data(address, reg, length)

    for index in range(length):
        data_p[index] = result[index]

    return 0

# i2c bus write callback
def i2c_write(address, reg, data_p, length):
    data = []
    for index in range(length):
        data.append(data_p[index])

    i2cbus.write_i2c_block_data(address, reg, data)

    return 0

# Load VL53L0X shared lib 
tof = CDLL("../bin/vl53l0x_python.so")

# Create read function pointer
READFUNC = CFUNCTYPE(c_int, c_ubyte, c_ubyte, POINTER(c_ubyte), c_ubyte)
read_func = READFUNC(i2c_read)

# Create write function pointer
WRITEFUNC = CFUNCTYPE(c_int, c_ubyte, c_ubyte, POINTER(c_ubyte), c_ubyte)
write_func = WRITEFUNC(i2c_write)

# pass i2c read and write function pointers to VL53L0X library
tof.VL53L0X_set_i2c(read_func, write_func)

class VL53L0X(object):
    """VL53L0X ToF."""

    device_address = 0x29

    def __init__(self, address=0x29, **kwargs):
        """Initialize the VL53L0X ToF Sensor from ST"""
        self.device_address = address
        # If address passed is not 0x29, must change address
        # in the device itself

    def start_ranging(self, mode = VL53L0X_GOOD_ACCURACY_MODE):
        """Start VL53L0X ToF Sensor Ranging"""
        tof.startRanging(mode)

    def stop_ranging(self):
        """Stop VL53L0X ToF Sensor Ranging"""
        tof.stopRanging()

    def get_distance(self):
        """Get distance from VL53L0X ToF Sensor"""
        return tof.getDistance()

