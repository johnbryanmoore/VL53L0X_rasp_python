# VL53L0X Python interface on Raspberry Pi

This project provides a simplified python interface on Raspberry Pi to the ST VL53L0X API (ST Microelectronics).

Patterned after the cassou/VL53L0X_rasp repository (https://github.com/cassou/VL53L0X_rasp.git)

In order to be able to share the i2c bus with other python code that uses the i2c bus, this library implements the VL53L0X platform specific i2c functions through callbacks to the python smbus interface. 

Version 0.0.9:
- initial version and only supports 1 sensor with limited error checking.

Version 1.0.0:
- Add support for multiple sensors on the same bus utilizing the ST API call to change the address of the device.
- Add support for improved error checking such as I/O error detection on I2C access.

Notes on Multiple sensor support:
- In order to have multiple sensors on the same bus, you must have the shutdown pin of each sensor tied to individual GPIO's so that they can be individually enabled and the addresses set.
- Both the Adafruit and Pololu boards for VL53L0X have I2C pull ups on the board. Because of this, the number of boards that can be added will be limited to only about 5 or 6 before the pull-up becomes too strong.
- Changes to the platform and python_lib c code allow for up to 16 sensors.
- Address changes are volatile so setting the shutdown pin low or removing power will change the address back to the default 0x29.

(Please note that while the author is an embedded software engineer, this is a first attempt at extending python and the author is by no means a python expert so any improvement suggestions are appreciated).


### Installation


### Compilation

* To compile the lib on your raspberry pi:
```bash
API_DIR=path/to/the/api/dir make
```

* In the Python directory are 2 python files:

VL53L0X.py - This contains the python ctypes interface to the ST Library

VL53L0X_example.py - This contains an example that accesses a single sensor with the default address.

VL53L0X_multi_example.py - This contains an example that accesses 2 sensors, setting the first to address 0x2B an the second to address 0x2D. It uses GPIOs 20 and 16 connected to the shutdown pins on the 2 sensors to control sensor activation.

Note: Eventualy I will make this an installable extension.

