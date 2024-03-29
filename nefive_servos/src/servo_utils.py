#!/usr/bin/env python
# -*- coding: utf-8 -*-

#*******************************************************************************
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#*******************************************************************************


#*******************************************************************************
#***********************     Read and Write Example      ***********************
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example :
#    - Select the DYNAMIXEL in use at the MY_DXL in the example code. 
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
#  Author: Ryu Woon Jung (Leon)
#  Maintainer : Zerom, Will Son
# *******************************************************************************

import os, ctypes
from jointlist import ServoDetails
import time
from datetime import datetime
import signal
from dynamixel_sdk import *  # Uses Dynamixel SDK library

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class dynamixel_utils:
    # Control table address
    ADDR_OPERATING_MODE         = 11
    ADDR_HOMING_OFFSET          = 20
    ADDR_CURRENT_LIMIT          = 38
    ADDR_TORQUE_ENABLE          = 64
    ADDR_LED_RED                = 65
    ADDR_ERROR_STATUS           = 70
    ADDR_PID_POSITION_P         = 80
    ADDR_PID_POSITION_I         = 82
    ADDR_PID_POSITION_D         = 84
    ADDR_GOAL_CURRENT           = 102
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_CURRENT        = 126
    ADDR_PRESENT_VELOCITY       = 128
    ADDR_PRESENT_POSITION       = 132
    ADDR_PRESENT_TEMPERATURE    = 146
    PROTOCOL_VERSION            = 2.0

    TORQUE_ENABLE               = 1     # Value for enabling the torque
    TORQUE_DISABLE              = 0     # Value for disabling the torque
    LED_ENABLE                  = 1     # Value for enabling the torque
    LED_DISABLE                 = 0     # Value for disabling the torque
    
    POSITION_MODE               = 3
    CURRENT_BASED_POSITION_MODE = 5

    portHandler = None
    packetHandler = None

    lerpingInProgress = False

    def __init__(self, serial, baud):
        self.portHandler = PortHandler(serial)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print(f"Port open {serial}")
        else:
            print(f"Failed to open {serial}")
            
        # Set port baudrate
        if self.portHandler.setBaudRate(baud):
            print(f"Baudrate set to {baud}")
        else:
            print(f"Failed to set the baudrate to {baud}")
            
        error_states = self.readAllHardwareStatus()
        for servo_name, servo_id in ServoDetails.servo_details.items():
            if error_states[servo_id] != 0:
                print(f"[{servo_id}]: {error_states[servo_id]}")

                dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, servo_id)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

                print("[ID:%03d] reboot Succeeded\n" % servo_id)
        
        # Sleep for 0.1 second to let the port open
        time.sleep(0.1)
            
    # This is valid for four bit two's compliment values
    def fourBitToSigned(self, position):
        if position > 2147483647:
            position = position - 4294967296

        return position

    def setAllCurrentGoals(self, goals):
        addr = self.ADDR_GOAL_CURRENT
        groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, addr, 2)
        for _, servo_id in ServoDetails.servo_details.items():            
            if goals[servo_id] is not None:
                # print(f"setting {servo_id} to {positions[servo_id]}")
                position_array = goals[servo_id].to_bytes(2, "little")
                groupSyncWrite.addParam(servo_id, position_array)

        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

    def setAllPositions(self, positions):
        groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, 4)
        for servo_name, servo_id in ServoDetails.servo_details.items():
            # print(f"setting {servo_id} to {positions[servo_id]}")
            position_array = positions[servo_id].to_bytes(4, "little", signed=True)
            groupSyncWrite.addParam(servo_id, position_array)

        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()
        

    def setAllAngles(self, angles):
        positions = {}
        for servo_name, servo_id in ServoDetails.servo_details.items():
            positions[servo_id] = self.angleToPosition(angles[servo_id])

        self.setAllPositions(positions)


    def setAllRadians(self, radians):
        positions = {}
        for servo_name, servo_id in ServoDetails.servo_details.items():
            positions[servo_id] = self.radianToPosition(radians[servo_id])

        self.setAllPositions(positions)
        

    def readAllAngles(self):
        angles = {}
        positions = self.readAllPositions()
        if positions == None:
            return None
        
        for servo_name, servo_id in ServoDetails.servo_details.items():
            angles[servo_id] = self.positionToAngle(positions[servo_id])
        return angles
    
    
    def readAllRadians(self):
        radians = {}
        positions = self.readAllPositions()
        
        if positions == None:
            return None
        
        for servo_name, servo_id in ServoDetails.servo_details.items():
            radians[servo_id] = self.positionToRadian(positions[servo_id])
        return radians
    
    def readAllPositions(self): 
        groupSyncReadPosition = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_POSITION, 4)
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_addparam_result = groupSyncReadPosition.addParam(servo_id)
            if dxl_addparam_result != True:
                print(f"add_param: {dxl_addparam_result}")

        dxl_comm_result = groupSyncReadPosition.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        present_position = {}
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_getdata_result = groupSyncReadPosition.isAvailable(servo_id, self.ADDR_PRESENT_POSITION, 4)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % servo_id)
                # return None

            position = groupSyncReadPosition.getData(servo_id, self.ADDR_PRESENT_POSITION, 4)
            present_position[servo_id] = self.fourBitToSigned(position)

        return present_position


    def readAllCurrent(self):  
        groupSyncReadCurrent = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_CURRENT, 2)
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_addparam_result = groupSyncReadCurrent.addParam(servo_id)
            if dxl_addparam_result != True:
                print(f"add_param: {dxl_addparam_result}")

        dxl_comm_result = groupSyncReadCurrent.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        present_current = {}
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_getdata_result = groupSyncReadCurrent.isAvailable(servo_id, self.ADDR_PRESENT_CURRENT, 2)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % servo_id)
                # return None

            current = groupSyncReadCurrent.getData(servo_id, self.ADDR_PRESENT_CURRENT, 2)
            present_current[servo_id] = current / 1000

        return present_current


    def readAllEfforts(self):        
        groupSyncReadCurrent = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_CURRENT, 2)
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_addparam_result = self.groupSyncReadCurrent.addParam(servo_id)
            if dxl_addparam_result != True:
                print(f"add_param: {dxl_addparam_result}")

        dxl_comm_result = self.groupSyncReadCurrent.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        present_effort = {}
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_getdata_result = self.groupSyncReadCurrent.isAvailable(servo_id, self.ADDR_PRESENT_CURRENT, 2)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % servo_id)
                # return None

            current = self.groupSyncReadCurrent.getData(servo_id, self.ADDR_PRESENT_CURRENT, 2)
            present_effort[servo_id] = (current / 1000) * self.servoDetails.servo_torque_constants[servo_id]

        return present_effort
     
                
    def readAllCurrentLimits(self):        
        addr = self.ADDR_CURRENT_LIMIT
        groupSyncReadCurrent = GroupSyncRead(self.portHandler, self.packetHandler, addr, 2)
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_addparam_result = groupSyncReadCurrent.addParam(servo_id)
            if dxl_addparam_result != True:
                print(f"add_param: {dxl_addparam_result}")

        dxl_comm_result = groupSyncReadCurrent.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        current_limits = {}
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_getdata_result = groupSyncReadCurrent.isAvailable(servo_id,  addr, 2)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % servo_id)
                # return None

            current_limits[servo_id] = groupSyncReadCurrent.getData(servo_id,  addr, 2)
            
        return current_limits
     
                
    groupSyncReadVelocities = None
    def readAllVelocities(self):      
        if self.groupSyncReadVelocities == None:  
            self.groupSyncReadVelocities = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_VELOCITY, 4)
            for servo_name, servo_id in ServoDetails.servo_details.items():
                dxl_addparam_result = self.groupSyncReadVelocities.addParam(servo_id)
                if dxl_addparam_result != True:
                    print(f"add_param: {dxl_addparam_result}")

        dxl_comm_result = self.groupSyncReadVelocities.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        present_velocity = {}
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_getdata_result = self.groupSyncReadVelocities.isAvailable(servo_id, self.ADDR_PRESENT_VELOCITY, 4)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % servo_id)
                # return None

            servo_velocity = self.groupSyncReadVelocities.getData(servo_id, self.ADDR_PRESENT_VELOCITY, 4)
            servo_velocity = self.fourBitToSigned(servo_velocity)

            # units for velocity is 0.229 / minute and 1RPM is 0.10472 radians per second
            # optimising 0.229 * 0.10471 to 0.02398088
            velocity = servo_velocity * 0.02398088
    
            present_velocity[servo_id] = velocity
        return present_velocity

  
    def pingServos(self):
        # Try to broadcast ping the Dynamixel
        dxl_data_list, dxl_comm_result = self.packetHandler.broadcastPing(self.portHandler)
        if dxl_comm_result != COMM_SUCCESS:
            print("Error pinging: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        detected = []
        for dxl_id in dxl_data_list:
            detected.append(dxl_id)

        return detected


    def setOperatingMode(self, id, value):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_OPERATING_MODE, value)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"setOpMode for id {id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"setOpMode for id {id}: {self.packetHandler.getRxPacketError(dxl_error)}")


    def setCurrentGoal(self, id, value):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_GOAL_CURRENT, value)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"setCurrentGoal for id {id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"setCurrentGoal for id {id}: {self.packetHandler.getRxPacketError(dxl_error)}")


    def setCurrentLimit(self, id, value):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_CURRENT_LIMIT, value)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"setCurrentLimit for id {id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"setCurrentLimit for id {id}: {self.packetHandler.getRxPacketError(dxl_error)}")


    def setPositionPID(self, id, p, i, d):
        print(f"Setting PID for Servo {id} to {p}, {i}, {d}")
        
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_PID_POSITION_P, p)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"setPositionPID P for id {id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"setPositionPID P for id {id}: {self.packetHandler.getRxPacketError(dxl_error)}")

        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_PID_POSITION_I, i)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"setPositionPID I for id {id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"setPositionPID I for id {id}: {self.packetHandler.getRxPacketError(dxl_error)}")
            
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_PID_POSITION_D, d)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"setPositionPID D for id {id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"setPositionPID D for id {id}: {self.packetHandler.getRxPacketError(dxl_error)}")

    def setLED(self, id, value):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_LED_RED, value)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"setLED for id {id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"setLED for id {id}: {self.packetHandler.getRxPacketError(dxl_error)}")


    def getCurrentPosition(self, servo_id):
        print(f"Getting position of {servo_id}")
        position, result, error = \
            self.packetHandler.readTxRx(self.portHandler, servo_id, self.ADDR_PRESENT_POSITION, 4)
        if result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(result))
            return
        elif error != 0:
            print("%s" % self.packetHandler.getRxPacketError(error))
            return
        data_array = bytes([position[0], position[1], position[2], position[3]])
        return int.from_bytes(data_array, "little", signed=True)


    def getCurrentAngle(self, servo_id):
        position_signed = self.getCurrentPosition(servo_id)
        current_angle = self.positionToAngle(position_signed)
        return round(current_angle, 2)


    def readAllOperatingModes(self):
        addr = self.ADDR_OPERATING_MODE
        groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, addr, 1)
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_addparam_result = groupSyncRead.addParam(servo_id)

        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        operating_mode = {}
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_getdata_result = groupSyncRead.isAvailable(servo_id, addr, 1)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % servo_id)
                operating_mode[servo_id] = None
            else:
                operating_mode[servo_id] = int(groupSyncRead.getData(servo_id, addr, 1))

        return operating_mode

    def readAllTemperatures(self):
        groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_TEMPERATURE, 1)
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_addparam_result = groupSyncRead.addParam(servo_id)

        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        present_temperature = {}
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_getdata_result = groupSyncRead.isAvailable(servo_id, self.ADDR_PRESENT_TEMPERATURE, 1)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % servo_id)
                present_temperature[servo_id] = None
            else:
                present_temperature[servo_id] = int(groupSyncRead.getData(servo_id, self.ADDR_PRESENT_TEMPERATURE, 1))

        return present_temperature

    def readAllHardwareStatus(self):
        groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_ERROR_STATUS, 1)
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_addparam_result = groupSyncRead.addParam(servo_id)

        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        present_state = {}
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_getdata_result = groupSyncRead.isAvailable(servo_id, self.ADDR_ERROR_STATUS, 1)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % servo_id)
                present_state[servo_id] = None
            else:
                present_state[servo_id] = int(groupSyncRead.getData(servo_id, self.ADDR_ERROR_STATUS, 1))

        return present_state


    def positionToAngle(self, position):
        # -2048 normalises around the middle position of the servo
        # 0.088 is 4096 (encoder resolution) / 360 ie. 1 tick is 0.088 degress
        current_angle = (position - 2048) * 0.088
        return round(current_angle, 2)


    def positionToRadian(self, position):
        # -2048 normalises around the middle position of the servo
        # 0.088 is 4096 (encoder resolution) / 360 ie. 1 tick is 0.088 degress
        # 0.001535 is 1 tick in radians
        current_angle = (position - 2048) * 0.001535 
        return round(current_angle, 4)


    def angleToPosition(self, angle):
        # convert angle to position value, note this is float to int

        position = int(angle / 0.088) + 2048
        # print(f"{angle} angle converted to position {position}")
        return position


    def radianToPosition(self, radian):
        # convert angle to position value, note this is float to int
        # current_angle = (position - 2048) * 0.001535 
        
        position = int(radian / 0.001535) + 2048
        # print(f"{angle} angle converted to position {position}")
        return position


    def setAngle(self, servo_id, angle):
        position = self.angleToPosition(angle)
        # print(position)

        self.setPosition(servo_id, position)


    def setRadian(self, servo_id, radian):
        position = self.radianToPosition(radian)
        # print(position)

        self.setPosition(servo_id, position)


    def setPosition(self, servo_id, position):
        # print(f"{servo_id}, {position}")

        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, servo_id, self.ADDR_GOAL_POSITION, position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def setHomingOffset(self, servo_id, angle):
        # convert angle to position value, note this is float to int
        position = int(angle / 0.088)

        # position to byte array
        position_array = position.to_bytes(4, "little", signed=True)

        # send to servo_id
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, servo_id, self.ADDR_HOMING_OFFSET, position_array)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def setServoTorque(self, servo_id, value):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, servo_id, self.ADDR_TORQUE_ENABLE, value)
        if dxl_comm_result != COMM_SUCCESS:
            print("SetServo: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("SetServo: %s" % self.packetHandler.getRxPacketError(dxl_error))


    def readAllTorqueStates(self):
        groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_TORQUE_ENABLE, 1)
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_addparam_result = groupSyncRead.addParam(servo_id)

        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        present_torque_states = {}
        for servo_name, servo_id in ServoDetails.servo_details.items():
            dxl_getdata_result = groupSyncRead.isAvailable(servo_id, self.ADDR_TORQUE_ENABLE, 1)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % servo_id)
                present_torque_states[servo_id] = None
            else:
                present_torque_states[servo_id] = int(groupSyncRead.getData(servo_id, self.ADDR_TORQUE_ENABLE, 1))

        return present_torque_states


    # Disable all LEDs and servos
    def disableAllServos(self):
        for _, servo_id in ServoDetails.servo_details.items():
            self.setServoTorque(servo_id, self.TORQUE_DISABLE)
            self.setLED(servo_id, self.LED_DISABLE)


    def enableAllServos(self):
        states = self.readAllTorqueStates()
        for servo_name, servo_id in ServoDetails.servo_details.items():
            if states[servo_id] == 0:
                self.setServoTorque(servo_id, self.TORQUE_ENABLE)
                self.setLED(servo_id, self.LED_ENABLE)


    def lerp(self, start, end, ratio):
        return round((ratio * end) + ((1 - ratio) * start), 1)


    def lerpToRadians(self, radians, lerp_time):
        self.lerpingInProgress = True

        start_radians = self.readAllRadians()
        interval = lerp_time / 100
        print(f"Lerp interval: {interval}")
        for i in range(100):
            new_radians = {}
            for servo_name, servo_id in ServoDetails.servo_details.items():
                value = self.lerp(start_radians[servo_id], radians[servo_id], i / 100)
                if value == None:
                    print(f"Unable to set position for {servo_name}")
                    continue
                new_radians[servo_id] = value
            self.setAllRadians(new_radians)
            # print(new_radians)
            
            time.sleep(interval)
        
        self.lerpingInProgress = False

    def lerpToAngles(self, angles, lerp_time, start_angles = None):
        self.lerpingInProgress = True
        if(start_angles == None):
            start_angles = self.readAllAngles()

        interval = lerp_time / 100
        print(f"Lerp interval: {interval}")
        for i in range(100):
            new_angles = {}
            for servo_name, servo_id in ServoDetails.servo_details.items():
                value = self.lerp(start_angles[servo_id], angles[servo_id], i / 100)
                if value == None:
                    print(f"Unable to set position for {servo_name}")
                    continue
                new_angles[servo_id] = value
            self.setAllAngles(new_angles)
            # print(new_angles)
            
            time.sleep(interval)
        
        self.lerpingInProgress = False


    def lerpToPositions(self, positions, lerp_time):
        self.lerpingInProgress = True
        start_positions = self.readAllPositions()
        interval = lerp_time / 100
        print(f"Lerp interval: {interval}")
        for i in range(100):
            new_positions = {}
            for servo_name, servo_id in ServoDetails.servo_details.items():
                value = int(lerp(start_positions[servo_id], positions[servo_id], i / 100))
                if value == None:
                    print(f"Unable to set position for {servo_name}")
                    continue
                new_positions[servo_id] = value
            self.setAllPositions(new_positions)
            
            time.sleep(interval)
        self.lerpingInProgress = False

    # Read current positions for all servos and move home
    def moveToHome(self):
        self.enableAllServos()

        start_angles = self.readAllAngles()

        for i in range(100):
            angles = {}
            for servo_name, servo_id in ServoDetails.servo_details.items():
                value = self.lerp(start_angles[servo_id], self.home_positions[servo_id], i / 100)
                if value == None:
                    print(f"Unable to set position for {servo_name}")
                    continue
                angles[servo_id] = value
            print(angles)
            self.setAllAngles(angles)
            print(i)
            time.sleep(0.005)


    def getHomePositions(self):
        home_values = {}

        # Iterate through all servos, setting home location
        for servo_name, servo_id in ServoDetails.servo_details.items():
            print(f"[{servo_name}] please move to home position and press any key to continue or ESC to quit")
            if getch() == chr(0x1b):
                break

            # Read present position
            angle = self.getCurrentAngle(servo_id)
            self.setServoTorque(servo_id, 1)
            self.setLED(servo_id, 1)
            home_values[servo_id] = angle
            print(f"{servo_name} home angle is {angle}")

        print(f"Home positions: {home_values}")


        print("press any key to disable servos and exit")
        getch()

        # disableAllServos()
        # Close port


    def loopReadTemperature(self):
        log_file = open("temp_log.txt", "at")

        # Just print all the positions/temperatures
        while 1:
            current_time = datetime.now()
            line = "[{0}:{1}:{2}] : {3}\n".format(current_time.hour,
                                                current_time.minute,
                                                current_time.second,
                                                self.readAllTemperatures())
            print(line)
            log_file.write(line)

            time.sleep(4)
            # print(readAllPositions())

        log_file.close()


    def getLimits(self):
        servo_limits = {}
        min_value = 0
        max_value = 1

        for servo_name, servo_id in ServoDetails.servo_details.items():
            # each tuple is {min_value, max_value}
            servo_limits[servo_id] = [ 100000, -100000 ]

        while 1:
            current_positions = self.readAllPositions()
            for servo_name, servo_id in ServoDetails.servo_details.items():
                if current_positions[servo_id] >= servo_limits[servo_id][max_value]:
                    servo_limits[servo_id][max_value] = current_positions[servo_id]
                    
                if current_positions[servo_id] <= servo_limits[servo_id][min_value]:
                    servo_limits[servo_id][min_value] = current_positions[servo_id]

            print(f"{servo_limits}")

    
    # set all servos to current-position mode
    def setOperatingModes(self, value):
        operating_modes = self.readAllOperatingModes()
        torque_state = self.readAllTorqueStates()

        for servo_name, servo_id in ServoDetails.servo_details.items():
            # This requires the servo to have it's torque disabled, only update if needed
            if operating_modes[servo_id] != value:
                if torque_state[servo_id] == 1:
                    self.setServoTorque(servo_id, 0)
    
                self.setOperatingMode(servo_id, value)
    
                if torque_state[servo_id] == 1:
                    self.setServoTorque(servo_id, 1)


    def setAllCurrentLimits(self, current_limits):
        torque_states = self.readAllTorqueStates()
        existing_current_limit = self.readAllCurrentLimits()

        for _, servo_id in ServoDetails.servo_details.items():
            # This requires the servo to have it's torque disabled, only update if needed
            if existing_current_limit[servo_id] != current_limits[servo_id]:
                if torque_states[servo_id] == 1:
                    self.setServoTorque(servo_id, 0)
    
                self.setCurrentLimit(servo_id, current_limits[servo_id])
    
                if torque_states[servo_id] == 1:
                    self.setServoTorque(servo_id, 1)


    def learnPositions():
        positions_in_order = []
        ch = ""
        while True:
            
            print(f"Please move to new positions and press any key to continue or esc to finish")
            
            if getch() == chr(0x1b):
                break
            
            currentPositions = readAllPositions()
                # if currentPositions != None:
                #     print(f"setting positions: {currentPositions}")
                #     setAllPositions(currentPositions)
                
                    
            # Append present positions to list
            positions_in_order.append(currentPositions)
                    

        print(f"Home positions: {positions_in_order}")

        print("press any key to disable servos and exit")
        getch()

        disableAllServos()
        # Close port

