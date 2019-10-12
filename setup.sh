#!/usr/bin/env bash
sudo chmod 777 /dev/ttyACM1
sudo xboxdrv --detach-kernel-driver --silent
