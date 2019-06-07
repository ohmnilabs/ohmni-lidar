# OhmniLabs A2M8 2D Lidar module code

## File description
### LidarSerial
Module to handle serial communication between the Lidar module and host.

### LidarNode
Some sample high level functions from data reported from the serial port. LidarNode takes 2 arguments:
* Port string: USB path of connected Lidar, should be COM* or /dev/ttyUSB*.
* Determine array: you can follow the format for this under line 19 to 32. This array will determine which angle zone the Lidar will be scanning or ignoring.

## Requirments
* Require NodeJs and serialport module to run.
* USB path needs sufficient permission for modules to be able to read and write.
