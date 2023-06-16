#!/usr/bin/env python3

from servo_utils import dynamixel_utils

servos = dynamixel_utils('/dev/ttyUSB0', 1000000)
servos.disableAllServos()

