#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import signal
import sys
import time
from servo_utils import dynamixel_utils
from jointlist import servo_details, servo_torque_constants, home_positions


servos = dynamixel_utils('/dev/ttyUSB0', 1000000)

pospub = None

# Previous value for the trigger, used for debouncing
trigger_prev = False
lerpingInProgress = False

axesDict = {
    "lx": 0,
    "ly": 1,
    "lz": 2,
    "rx": 3,
    "ry": 4,
    "rz": 5,
    "encoder": 6 }

xboxAxesDict = {
    "lx": 0,
    "ly": 1,
    "lt": 2, # -1:1, zero at start then 1 at rest
    "rx": 3,
    "ry": 4,
    "rt": 5, # -1:1, zero at start then 1 at rest
    "d-lr": 6,
    "d-ud": 7 }

xboxButtonsDict = {
    "A": 0,
    "B": 1,
    "X": 2,
    "Y": 3,
    "LB": 4,
    "RB": 5,
    "View": 6,
    "Menu": 7,
    "Meatball": 8,
    "LS": 9,
    "RS": 10 }

buttonsDict = {
    "S1": 0,
    "S2": 1,
    "S3": 4,
    "ToggleDown": 2,
    "ToggleUp": 3,
    # "Encoder": 5,
    "Trigger": 5,
    "LeftStick": 6,
    "RightStick": 7 }


# stores the current positions of all the servos
radians = servos.readAllRadians()
 

def Lerp(start, end, ratio):
    if start == end:
        return start

    return (start + (end - start) * ratio)


def InitServos():
    servos.enableAllServos()
    # servos.setAllAngles(home_positions)


def DisableServos():
    servos.disableAllServos()


def handler(signum, frame):
    print("ctrl-c pressed, exiting")
    DisableServos()
    sys.exit()


def joy_callback(data):
    global trigger_prev, lerpingInProgress
    if lerpingInProgress == True:
        # print("lerping in progress")
        return
    
    radians = servos.readAllRadians()

    if(data.buttons[buttonsDict["ToggleUp"]] == 1):
        # Arm control engaged
        global positions, leftLastClicked, leftPrev, rightLastClicked, rightPrev
        # print(data.axes[0], data.axes[1], data.axes[2], data.axes[3])

        lx = data.axes[axesDict["lx"]]
        ly = data.axes[axesDict["ly"]]
        lz = data.axes[axesDict["lz"]]

        rx = data.axes[axesDict["rx"]]
        ry = data.axes[axesDict["ry"]]
        rz = data.axes[axesDict["rz"]]

        reduction = 25

        leftAxisX = scaleinput(lx, False, reduction)
        leftAxisY = scaleinput(ly, False, reduction)
        leftAxisZ = scaleinput(lz, False, reduction)

        rightAxisX = scaleinput(rx, False, reduction)
        rightAxisY = scaleinput(ry, False, reduction)
        rightAxisZ = scaleinput(rz, False, reduction)

        trigger_curr = data.buttons[buttonsDict["Trigger"]]

        leftStickButton = data.buttons[buttonsDict["LeftStick"]]
        leftControlArm = data.buttons[buttonsDict["ToggleUp"]]
        leftControlHand = data.buttons[buttonsDict["S1"]]

        rightStickButton = data.buttons[buttonsDict["RightStick"]]
        rightControlArm = data.buttons[buttonsDict["ToggleUp"]]
        rightControlHand = data.buttons[buttonsDict["S2"]]

        # check if the trigger has been pulled
        trigger_pulled = False
        if trigger_curr != trigger_prev:
            trigger_prev = trigger_curr
            trigger_pulled = trigger_curr


        # We're in left arm control mode, check if arm/hand control needed
        if(leftControlArm == 1):
            if(leftControlHand == 0):
                # control the arm, not the wrist
                        
                if(leftStickButton == 0):
                    # positions.servo1 -= leftAxisY 
                    radians[101] -= leftAxisY 
                else:
                    radians[103] -= leftAxisY
                radians[100] -= leftAxisZ      
                radians[102] += leftAxisX      
        
            if(leftControlHand == 1):
                # controlling the wrist/hand      
                # pan
                radians[105] += leftAxisX
                # tilt
                radians[106] += leftAxisY
                # roll
                radians[104] += leftAxisZ

        # We're in right arm control mode, check if arm/hand control needed
        if(rightControlArm == 1):
            if(rightControlHand == 0):                           
                if(rightStickButton == 0):
                    # positions.servo1 -= leftAxisY 
                    radians[109] -= rightAxisY 
                else:
                    radians[111] -= rightAxisY
                radians[108] -= rightAxisZ      
                radians[110] += rightAxisX      
        
            if(rightControlHand == 1):
                # controlling the wrist/hand      
                # pan
                radians[113] += rightAxisX
                # tilt
                radians[114] += rightAxisY
                # roll
                radians[112] += rightAxisZ

        print(radians)
        servos.setAllRadians(radians)


def scaleinput(input, invert, scale):
    if scale != 0:
        scaled = input / scale
    else:
        scaled = input

    if(invert):
        scaled = scaled * -1

    return scaled


def TriggerButton(input):
    if input < 0.75:
        return 1
    else:
        return 0

# Rotation is centred on 0.5 so need to map for that
def MapAxis(input):
    if((input >= 0.43) & (input <= 0.57)):
        return 0
    else:
        return input - 0.5


def listener():
    global pospub
    rospy.init_node('dynamixel_arm_driver', anonymous=True)
    rospy.Subscriber('joy', Joy, joy_callback)

    rospy.spin()

if __name__ == '__main__':
    print("Arm controller node online.")
    signal.signal(signal.SIGINT, handler)
    InitServos()
    listener()
