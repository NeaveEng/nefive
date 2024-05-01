#!/usr/bin/env python3

from servo_utils import dynamixel_utils
import time

servos = dynamixel_utils('/dev/ttyUSB0', 1000000)

# servos.pingServos()

while True:
    print(servos.readAllAngles())
    time.sleep(0.25)

