#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from nefive_servos.msg import arm_servos, servo_position
import signal
import sys, os
import time
from servo_utils import dynamixel_utils
from jointlist import ServoDetails
from enum import Enum



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
servoDetails = ServoDetails()
time.sleep(1)
print("Servo initialision complete.")

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

def createMsg(arm, positions):
    if arm != 0 | arm != 1:
        raise

    print(positions)

    offset = arm * 8
    msg = arm_servos()
    msg.arm = arm
    msg.shoulder_flappy = positions[100 + offset]
    msg.shoulder_foraft = positions[101 + offset]
    msg.upperarm_rotate = positions[102 + offset]
    msg.elbow           = positions[103 + offset]
    msg.wrist_rotate    = positions[104 + offset]
    msg.wrist_pan       = positions[105 + offset]
    msg.wrist_tilt      = positions[106 + offset]

    return msg


# Previous values
# left_ready_to_grip = {100: -4.29436801150441, 101: 64.2782050036639, 102: 0.9978429514914744, 103: -85.94753611288965, 104: 4.955641083493829, 105: -5.396310194730758, 106: -64.60023140214383, 107: -13.29 } 
# left_lifted = {100: -4.29436801150441, 101: 66.95306419961155, 102: 0.9978429514914744, 103: 20.227195417359468, 104: 4.955641083493829, 105: -5.396310194730758, 106: -64.60023140214383, 107: -13.29 }
# left_held_at_rest = {100: -3.34, 101: -89.75, 102: -0.38, 103: 34.23, 104: 4.17, 105: -2.20, 106: -69.08, 107: -28.06} 

# right_ready_to_grip = {108: 2.9693499980121842, 109: -67.74518650181591, 110: -10.91413763143122, 111: 74.63416039779781, 112: 3.2878335943818087, 113: -3.6549706425517794, 114: 63.84032794177532, 115: 35.83 }
# right_lifted = {108: 2.896395347043871, 109: -65.48065266914665, 110: -10.91413763143122, 111: -9.95228489652277, 112: 3.2878335943818087, 113: -3.6549706425517794, 114: 63.81009735442698, 115: 35.83}
# right_held_at_rest = {108: -4.33, 109: 80.26, 110: -6.15, 111: -36.55, 112: -6.09, 113: 2.9, 114: 67.11, 115: 35.83}

left_held_at_rest =  {100: 5.19, 101: 42.93, 102: 8.18, 103: 35.22, 104: 1.94, 105: -4.84, 106: -15.65}
right_held_at_rest = {108: -6.6, 109: -22.72, 110: -2.46, 111: -37.05, 112: 0.26, 113: -9.77, 114: 21.38}

left_ready_to_grip = {100: -5.28, 101: -51.44, 102: 8.18, 103: -87.45, 104: 1.94, 105: -4.84, 106: -47.41} 
right_ready_to_grip = {108: -4.84, 109: 60.95, 110: -2.19, 111: 79.68, 112: -0.77, 113: -5.35, 114: 61.63}

left_lifted = {100: -5.28, 101: -52.76, 102: 8.18, 103: -45.95, 104: 1.94, 105: -8.29, 106: -51.75} 
right_lifted = {108: -4.84, 109: 61.22, 110: -2.19, 111: 39.07, 112: -0.77, 113: -5.35, 114: 61.63}

leftLowered = createMsg(0, left_ready_to_grip)
leftRaised  = createMsg(0, left_lifted)
leftHome    = createMsg(0, left_held_at_rest)

rightLowered = createMsg(1, right_ready_to_grip)
rightRaised  = createMsg(1, right_lifted)
rightHome    = createMsg(1, right_held_at_rest)

# stores the current positions of all the servos
angles = servoDetails.home_positions


def Lerp(start, end, ratio):
    if start == end:
        return start

    return (start + (end - start) * ratio)


def InitServos():
    global leftArmState, leftArmTargetState, rightArmState, rightArmTargetState

    # Set servos to current-based position mode
    # servos.setOperatingModes(servos.CURRENT_BASED_POSITION_MODE)
    # servos.setAllCurrentLimits(servoDetails.current_limits)
    # servos.setAllCurrentGoals(servoDetails.current_goals)

    servos.enableAllServos()
    servos.lerpToAngles(angles, 2)

    leftArmTargetState = leftArmState = ArmState.HOME
    rightArmTargetState = rightArmState = ArmState.HOME


def DisableServos():
    currentGoals = servos.readAllCurrentLimits()

    for i in range(50):
        for id, goal in currentGoals.items():
            # move the goal towards 0 to ramp down the servos
            currentGoals[id] = int(goal * 0.75)
            
        servos.setAllCurrentGoals(currentGoals)
        time.sleep(2/50)

    servos.disableAllServos()


def handler(signum, frame):
    print("ctrl-c pressed, exiting")
    DisableServos()
    sys.exit()


pan_servo = 116
tilt_servo = 117
roll_servo = 118

pan = pan_centre = 0
tilt = tilt_centre = 0
roll = roll_centre = 0

pan_min = -57.3 + pan_centre
pan_max =  57.3 + pan_centre

tilt_min = -57.3 + tilt_centre
tilt_max =  27 + tilt_centre

roll_min = -35 + roll_centre
roll_max =  35 + roll_centre

def reset():
    global pan, tilt, roll, angles
    pan = pan_centre
    tilt = tilt_centre

    angles[pan_servo] = pan_centre
    angles[tilt_servo] = tilt_centre
    angles[roll_servo] = roll_centre
    
def scaleinput(input, invert, scale):
    if scale != 0:
        scaled = input / scale
    else:
        scaled = input

    if(invert):
        scaled = scaled * -1

    return scaled

leftButtonPrevious = False
leftButtonCurrent = False
leftPressed = False
rightButtonPrevious = False
rightButtonCurrent = False
rightPressed = False

leftHandOpen = True
rightHandOpen = True

leftHandAngleLimits  = [servos.positionToAngle(servoDetails.servo_limits[107][0]),
                        servos.positionToAngle(servoDetails.servo_limits[107][1])]

rightHandAngleLimits = [servos.positionToAngle(servoDetails.servo_limits[115][0]),
                        servos.positionToAngle(servoDetails.servo_limits[115][1])]


class ArmState(Enum):
        UNSET   = 0
        HOME    = 1
        RAISED  = 2
        LOWERED = 3

leftArmState        = ArmState.UNSET
leftArmTargetState  = ArmState.UNSET
rightArmState       = ArmState.UNSET
rightArmTargetState = ArmState.UNSET

def joy_callback(data: Joy):
    global angles, trigger_prev, leftButtonPrevious, rightButtonPrevious, leftHandOpen, rightHandOpen
    global leftArmState, leftArmTargetState, rightArmState, rightArmTargetState
    
    # Arm control engaged
    global leftLastClicked, leftPrev, rightLastClicked, rightPrev
    # print(data.axes[0], data.axes[1], data.axes[2], data.axes[3])

    trigger_curr = data.buttons[buttonsDict["Trigger"]]

    leftStickButton = data.buttons[buttonsDict["LeftStick"]]
    leftControlHand = data.buttons[buttonsDict["S1"]]

    rightStickButton = data.buttons[buttonsDict["RightStick"]]
    rightControlHand = data.buttons[buttonsDict["S2"]]
    
    leftPressed = False
    leftHeld = False
    if leftStickButton != leftButtonPrevious:
        leftButtonPrevious = leftStickButton
        leftPressed = leftStickButton

    if (leftStickButton == True) & (leftButtonPrevious == True):
        leftHeld = True

    if leftPressed == True:
        print("Left pressed this frame!")
    # elif leftHeld:
    #     print("Left held down!")

    rightPressed = False
    rightHeld = False
    if rightStickButton != rightButtonPrevious:
        rightButtonPrevious = rightStickButton
        rightPressed = rightStickButton

    if (rightStickButton == True) & (rightButtonPrevious == True):
        rightHeld = True

    if rightPressed == True:
        print("Right pressed this frame!")
    # elif rightHeld:
    #     print("Right held down!")

    # check if the trigger has been pulled
    trigger_pulled = False
    if trigger_curr != trigger_prev:
        trigger_prev = trigger_curr
        trigger_pulled = trigger_curr
    
    if trigger_pulled:
        print(angles)

    # We're in left arm control mode, check if arm/hand control needed
    if leftControlHand == 0:
        leftArmTargetState = ArmState.LOWERED
    else:
        leftArmTargetState = ArmState.HOME

    if rightControlHand == 0:                           
        rightArmTargetState = ArmState.LOWERED
    else:
        rightArmTargetState = ArmState.HOME
    
    if (leftArmTargetState != ArmState.UNSET) & (leftArmTargetState != leftArmState):
        if (leftArmState == ArmState.HOME) | (leftArmState == ArmState.LOWERED):
            # Move from home to raised
            armControl(leftRaised)

            # then from raised to the target
            target_msg = leftHome if leftArmTargetState == ArmState.HOME else leftLowered 
            armControl(target_msg)

            # finally set the state
            leftArmState = leftArmTargetState

    if (rightArmTargetState != ArmState.UNSET) & (rightArmTargetState != rightArmState):
        if (rightArmState == ArmState.HOME) | (rightArmState == ArmState.LOWERED):
            # Move from home to raised
            armControl(rightRaised)

            # then from raised to the target
            target_msg = rightHome if rightArmTargetState == ArmState.HOME else rightLowered 
            armControl(target_msg)

            # finally set the state
            rightArmState = rightArmTargetState 

    if leftPressed == True:
        if leftHandOpen == True:
            print("Closing left hand")
            servos.setCurrentGoal(107, -100)
            angles[107] = leftHandAngleLimits[1]
            leftHandOpen = False
        # close hand
        else:
            print("Opening left hand")
            servos.setCurrentGoal(107, 100)
            angles[107] = leftHandAngleLimits[0]
            leftHandOpen = True

    if rightPressed == True:
        if rightHandOpen == True:
            print("Closing right hand")
            servos.setCurrentGoal(115, -100)
            angles[115] = rightHandAngleLimits[0]
            rightHandOpen = False
        # close hand
        else:
            print("Opening right hand")
            servos.setCurrentGoal(115, 100)
            angles[115] = rightHandAngleLimits[1]
            rightHandOpen = True


    # head control using right stick
    pan_diff = scaleinput(data.axes[3], True, 0.25)
    tilt_diff = scaleinput(data.axes[4], True, 0.25)            
    roll_diff = scaleinput(data.axes[5], True, 0.5)

    pan = angles[116] + pan_diff
    tilt = angles[117] + tilt_diff
    roll = angles[118] + roll_diff

    if(pan > pan_max):
        pan = pan_max
    elif(pan < pan_min):
        pan = pan_min

    if(tilt > tilt_max):
        tilt = tilt_max
    elif(tilt < tilt_min):
        tilt = tilt_min

    if(roll > roll_max):
        roll = roll_max
    elif(roll < roll_min):
        roll = roll_min

    
    angles[pan_servo] = pan
    angles[tilt_servo] = tilt
    angles[roll_servo] = roll
    
    # print(pan, tilt, roll)
    
    if(servos.lerpingInProgress == False):
        servos.setAllAngles(angles)


def armControl(data: arm_servos):
    global angles

    print(f"arm msg: {data}")
    if data.arm == 0: # left arm
        angles[joint_dict["left_flappy"]] = data.shoulder_flappy
        angles[joint_dict["left_foreaft"]] = data.shoulder_foraft
        angles[joint_dict["left_upper_rotate"]] = data.upperarm_rotate
        angles[joint_dict["left_elbow"]] = data.elbow
        angles[joint_dict["left_wrist_pan"]] = data.wrist_pan
        angles[joint_dict["left_wrist_rotate"]] = data.wrist_rotate
        angles[joint_dict["left_wrist_tilt"]] = data.wrist_tilt

    elif data.arm == 1: # right arm
        angles[joint_dict["right_flappy"]] = data.shoulder_flappy
        angles[joint_dict["right_foreaft"]] = data.shoulder_foraft
        angles[joint_dict["right_upper_rotate"]] = data.upperarm_rotate
        angles[joint_dict["right_elbow"]] = data.elbow
        angles[joint_dict["right_wrist_pan"]] = data.wrist_pan
        angles[joint_dict["right_wrist_rotate"]] = data.wrist_rotate
        angles[joint_dict["right_wrist_tilt"]] = data.wrist_tilt

    else:
        print("ERROR! No valid arm specified!")
        return

    # this should only occur when the node starts up
    while lerpingInProgress == True:
        time.sleep(0.25)

    servos.lerpToAngles(angles, 1)
    

def hand_callback(data: servo_position):
    global angles
    print(f"hand msg: {data}")
    while lerpingInProgress == True:
        time.sleep(0.5)

    servos.setPosition(data.id, data.goal_pos)
    angles[data.id] = servos.positionToAngle(data.goal_pos)

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


def listener():
    global pospub
    rospy.init_node('dynamixel_arm_driver', anonymous=True)
    rospy.Subscriber('joy', Joy, joy_callback)
    # rospy.Subscriber('nefive_servos/arms', arm_servos, arm_callback)
    rospy.Subscriber('nefive_servos/servo_position', servo_position, hand_callback)

    rospy.spin()

if __name__ == '__main__':
    print("Arm controller node online.")
    signal.signal(signal.SIGINT, handler)
    InitServos()
    listener()
