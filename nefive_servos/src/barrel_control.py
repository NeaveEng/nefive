#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import signal
import sys, os
import time
from servo_utils import dynamixel_utils
from jointlist import servo_details, servo_torque_constants, home_positions, barrel_grab

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

servos = dynamixel_utils('/dev/ttyUSB0', 1000000)
print("Initialising servos...")
time.sleep(1)
print("Initialising servos complete.")

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

joint_dict = {
    "left_flappy": 100,
    "left_foreaft": 101,
    "left_upper_rotate": 102,
    "left_elbow": 103,
    "left_wrist_rotate": 104,
    "left_wrist_pan": 105,
    "left_wrist_tilt": 106,
    "left_hand": 107,
    "right_flappy": 108,
    "right_foreaft": 109,
    "right_upper_rotate": 110,
    "right_elbow": 111,
    "right_wrist_rotate": 112,
    "right_wrist_pan": 113,
    "right_wrist_tilt": 114,
    "right_hand": 115,
    "head_pan": 116,
    "head_tilt": 117,
    "head_roll": 118
}

# stores the current positions of all the servos
angles = barrel_grab

# print(angles)
# getch()

def Lerp(start, end, ratio):
    if start == end:
        return start

    return (start + (end - start) * ratio)


def InitServos():
    servos.enableAllServos()

    # print(f"Current angles {servos.readAllAngles()}")
    # print(f"Target angles {home_positions}")
    # print("continue?")
    # getch()
    # servos.getHomePositions()
    servos.lerpToAngles(angles, 2)



def DisableServos():
    servos.disableAllServos()


def handler(signum, frame):
    print("ctrl-c pressed, exiting")
    # DisableServos()
    sys.exit()


trigger_pulled = False

leftButtonPrevious = False
leftButtonCurrent = False
leftPressed = False
rightButtonPrevious = False
rightButtonCurrent = False
rightPressed = False

leftHandOpen = True
rightHandOpen = True

def joy_callback(data):
    global angles, trigger_prev, lerpingInProgress, leftButtonPrevious, rightButtonPrevious, leftHandOpen, rightHandOpen
    if lerpingInProgress == True:
        # print("lerping in progress")
        return
    
    # positions = start_positions = servos.readAllAngles()

    if(data.buttons[buttonsDict["ToggleUp"]] == 1):
        # Arm control engaged
        global leftLastClicked, leftPrev, rightLastClicked, rightPrev
        # print(data.axes[0], data.axes[1], data.axes[2], data.axes[3])

        lx = data.axes[axesDict["lx"]]
        ly = data.axes[axesDict["ly"]]
        lz = data.axes[axesDict["lz"]]

        rx = data.axes[axesDict["rx"]]
        ry = data.axes[axesDict["ry"]]
        rz = data.axes[axesDict["rz"]]

        reduction = 0.5

        leftAxisX = scaleinput(lx, False, reduction)
        leftAxisY = scaleinput(ly, False, reduction)
        leftAxisZ = scaleinput(lz, False, reduction)

        rightAxisX = scaleinput(rx, False, reduction)
        rightAxisY = scaleinput(ry, False, reduction)
        rightAxisZ = scaleinput(rz, False, reduction)

        # print(f"{leftAxisX}, {leftAxisY}, {leftAxisZ}")

        trigger_curr = data.buttons[buttonsDict["Trigger"]]

        leftStickButton = data.buttons[buttonsDict["LeftStick"]]
        leftControlArm = data.buttons[buttonsDict["ToggleUp"]]
        leftControlHand = data.buttons[buttonsDict["S1"]]

        rightStickButton = data.buttons[buttonsDict["RightStick"]]
        rightControlArm = data.buttons[buttonsDict["ToggleUp"]]
        rightControlHand = data.buttons[buttonsDict["S2"]]


        leftPressed = False
        if leftStickButton != leftButtonPrevious:
            leftButtonPrevious = leftStickButton
            leftPressed = leftStickButton

        rightPressed = False
        if rightStickButton != leftButtonPrevious:
            rightButtonPrevious = rightStickButton
            rightPressed = rightStickButton

        # check if the trigger has been pulled
        trigger_pulled = False
        if trigger_curr != trigger_prev:
            trigger_prev = trigger_curr
            trigger_pulled = trigger_curr
        
        if trigger_pulled:
            print(angles)

        if leftPressed:
            print(leftHandOpen)
            if leftHandOpen:
                leftHandOpen = False
                angles[joint_dict["left_hand"]] = -13
            else: 
                leftHandOpen = True
                angles[joint_dict["left_hand"]] = 30

        if rightPressed:
            print(rightHandOpen)
            if rightHandOpen:
                rightHandOpen = False
                angles[joint_dict["right_hand"]] = -13
            else: 
                rightHandOpen = True
                angles[joint_dict["right_hand"]] = 30



        start_angles = angles

        # We're in left arm control mode, check if arm/hand control needed
        if(leftControlArm == 1):
            # if(leftStickButton == 0):
            #     # positions.servo1 -= leftAxisY 
            #     angles[joint_dict["left_foreaft"]] = round(angles[joint_dict["left_foreaft"]] - leftAxisY, 1) 
           
            # controlling the wrist/hand      
            # pan
            angles[joint_dict["left_wrist_pan"]] += leftAxisX
            # tilt
            angles[joint_dict["left_wrist_tilt"]] += leftAxisY
            # roll
            angles[joint_dict["left_wrist_rotate"]] += leftAxisZ

        # We're in right arm control mode, check if arm/hand control needed
        if(rightControlArm == 1):
            # if(rightStickButton == 0):
            #     # positions.servo1 -= leftAxisY 
            #     angles[joint_dict["right_foreaft"]] -= rightAxisY 
           
            # controlling the wrist/hand      
            # pan
            angles[joint_dict["right_wrist_pan"]] += rightAxisX
            # tilt
            angles[joint_dict["right_wrist_tilt"]] += rightAxisY
            # roll
            angles[joint_dict["right_wrist_rotate"]] += rightAxisZ

        # print(angles)   

        angle_changed = False

        if start_angles != angles:
            angle_changed = True

        # print("=====")
        # print(start_angles)
        # print(angles)
        # print(diff_dict)
        # print("=====")
        # getch()

        # print(f"changes? {angle_changed}")
        # if angle_changed == True:
        # print(f"100 actual: {servos.getCurrentPosition(100)}")
        # print(f"100 target: {servos.angleToPosition(angles[100])}")

        servos.setAllAngles(angles)


def scaleinput(input, invert, scale):
    if scale != 0:
        scaled = input / scale
    else:
        scaled = input

    if(invert):
        scaled = scaled * -1

    if scaled == -0.0:
        scaled = 0

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
