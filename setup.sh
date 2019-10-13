#!/usr/bin/env bash
sudo chmod 777 /dev/ttyACM0
sudo xboxdrv --detach-kernel-driver --silent

cd ~/arduino-1.8.10/libraries
rm -r ros_lib
rosrun rosserial_arduino make_libraries.py .
