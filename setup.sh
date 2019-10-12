#!/usr/bin/env bash
sudo chmod 777 /dev/ttyACM0
sudo xboxdrv --detach-kernel-driver --silent

rm ~/arduino-1.8.10/libraries/ros_lib
