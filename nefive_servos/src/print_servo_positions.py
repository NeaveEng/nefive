#!/usr/bin/env python3

from servo_utils import dynamixel_utils
import time

servos = dynamixel_utils('/dev/ttyUSB0', 1000000)

while True:
    print(servos.readAllRadians())
    time.sleep(0.25)

