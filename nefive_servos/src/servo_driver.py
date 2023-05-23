#!/usr/bin/env python3

import rospy
import signal
import sys
import time
from datetime import datetime
from sensor_msgs.msg import JointState
from servo_utils import dynamixel_utils

# from dynamixel_sdk import * # Uses Dynamixel SDK library

# ADDR_OPERATING_MODE         = 11
# ADDR_HOMING_OFFSET          = 20
# ADDR_TORQUE_ENABLE          = 64
# ADDR_LED_RED                = 65
# ADDR_ERROR_STATUS           = 70
# ADDR_PID_POSITION_P         = 80
# ADDR_PID_POSITION_I         = 82
# ADDR_PID_POSITION_D         = 84
# ADDR_GOAL_CURRENT           = 102
# ADDR_GOAL_POSITION          = 116
# ADDR_PRESENT_POSITION       = 132
# ADDR_PRESENT_TEMPERATURE    = 146
# BAUDRATE                    = 1000000

# PROTOCOL_VERSION            = 2.0
# DEVICENAME                  = '/dev/ttyUSB0'

# portHandler = PortHandler(DEVICENAME)
# packetHandler = PacketHandler(PROTOCOL_VERSION)


# # Open port
# if portHandler.openPort():
#     print("Succeeded to open the port")
# else:
#     print("Failed to open the port")


# if portHandler.setBaudRate(BAUDRATE):
#     print("Succeeded to change the baudrate")
# else:
#     print("Failed to change the baudrate")

# def getCurrentPosition(servo_id):
#     position, result, error = \
#         packetHandler.readTxRx(portHandler, servo_id, ADDR_PRESENT_POSITION, 4)
#     if result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(result))
#         return
#     elif error != 0:
#         print("%s" % packetHandler.getRxPacketError(error))
#         return
#     data_array = bytes([position[0], position[1], position[2], position[3]])
#     return int.from_bytes(data_array, "little", signed=True)

servos = dynamixel_utils('/dev/ttyUSB0', 1000000)
servos.disableAllServos()
print(servos.pingServos())
print(servos.readAllTemperatures())
print(servos.readAllPositions())
