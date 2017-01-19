# VL53L0X Python interface on Raspberry Pi

This project provides a limited python interface on Raspberry Pi to the VL53L0X API (ST Microelectronics).

Patterned after the cassou/VL53L0X_rasp repository (https://github.com/cassou/VL53L0X_rasp.git)

In order to be able to share the i2c bus with other python code that uses the i2c bus, this library implements the VL53L0X platform specific i2c functions through callbacks to the python smbus interface. 

(Please note that while the author is an embedded software engineer, this is a first attempt at extending python and the author only started learning python less than 2 months ago so any improvement suggestions are appreciated).


### Installation


### Compilation

* To compile the lib on your raspberry pi:
```bash
API_DIR=path/to/the/api/dir make
```

* In the Python directory are 2 python files:

VL53L0X.py - This contains the python ctypes interface to the ST Library

VL53L0X_example.py - This contains an example program that uses the interface.

Eventualy I will make this an installable extension.

