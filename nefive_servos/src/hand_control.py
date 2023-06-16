#!/usr/bin/env python3

from re import L
import rospy
import redboard
from rosredboard.msg import Expander
import signal
import sys
import time
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Joy

#  Get the expander address
expander_address = rospy.get_param('/expander_address', 66)
rospy.loginfo("Expander address: " + str(expander_address))

rbex = redboard.PCA9685(address=expander_address)
rbex.frequency = 50

rbex.servo7_config = 700, 2400
rbex.servo15_config = 700, 2400

lerpingInProgress = False

left_hand_powers  = [0, 0, 0]
left_hand_powers_index = 0
right_hand_powers = [0, 0, 0]
right_hand_powers_index = 0

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
positions = Expander()
positions.servo7  = -0.5
positions.servo15 = -0.5

def Lerp(start, end, ratio):
    if start == end:
        return start

    return (start + (end - start) * ratio)

def AveragePowers(array):
    sumcurrent = 0
    for i in array:
        sumcurrent += i
    return sumcurrent / 3

def InitServos():
    global rbex, positions
    rbex.servo7 = positions.servo7
    rbex.servo15 = positions.servo15

def DisableServos():
    global rbex
    rbex.servo7 = None
    rbex.servo15 = None

def handler(signum, frame):
    print("ctrl-c pressed, exiting")
    DisableServos()
    sys.exit()

def TriggerButton(input):
    if input < 0.75:
        return 1
    else:
        return 0

# Set the position of each servo
def SetServos():
    global rbex, positions

    # left hand
    rbex.servo7 = positions.servo7
    
    # right hand
    rbex.servo15 = positions.servo15


def ReduceAxis(input, amount):
    return input / amount

def joy_callback(data):
    global trigger_prev, positions, lerpingInProgress
    if lerpingInProgress == True:
        # print("lerping in progress")
        return

    isXbox = False       #   Are we using an xbox controller?

    if(data.buttons[buttonsDict["ToggleUp"]] == 1) or (isXbox == True):
        # Arm control engaged
        global positions, leftLastClicked, leftPrev, rightLastClicked, rightPrev
        # print(data.axes[0], data.axes[1], data.axes[2], data.axes[3])

        if isXbox == True:
            rx =  data.axes[xboxAxesDict["rx"]]
        
            reduction = 50
            rightAxisX = ReduceAxis(rx, reduction)
        
            leftControlArm =  data.buttons[xboxButtonsDict["LB"]]
            leftControlHand = TriggerButton(data.axes[xboxAxesDict["lt"]])

            rightControlArm =  data.buttons[xboxButtonsDict["RB"]]
            rightControlHand = TriggerButton(data.axes[xboxAxesDict["rt"]])

        else:
            reduction = 15
            lz = data.axes[axesDict["lz"]]
            rz = data.axes[axesDict["rz"]]

            leftAxisZ = ReduceAxis(lz, reduction)
            rightAxisZ = ReduceAxis(rz, reduction)

            leftControlArm = rightControlArm = data.buttons[buttonsDict["ToggleUp"]]
            leftControlHand = data.buttons[buttonsDict["S1"]]
            rightControlHand = data.buttons[buttonsDict["S2"]]

        # We're in left arm control mode, check if arm/hand control needed
        if(leftControlArm == 1) & (leftControlHand == 1):
            if(isXbox == True):
                positions.servo7 += rightAxisX
            else:
                positions.servo7 += leftAxisZ

        # We're in right arm control mode, check if arm/hand control needed
        if(rightControlArm == 1) & (rightControlHand == 1):
            # controlling the wrist/hand
            
            if(isXbox == True):            
                positions.servo15 += rightAxisX
            else:
                positions.servo15 -= rightAxisZ

        SetServos()

# target positions, time in seconds
def LerpPositions(msg, time_to_move):
    global positions, rbex, lerpingInProgress
    global left_hand_powers, left_hand_powers_index, right_hand_powers, right_hand_powers_index

    # create a loop
    #   each iteration, move each position a bit closer
    # go with 50hz as a rate for finer control
    rate = 50
    delayPerLoop = time_to_move / rate
    iterations_needed = time_to_move * rate

    left_limited = False
    right_limited = False

    lerpingInProgress = True

    # how many iterations do we need?
    for x in range(iterations_needed + 1):
        if (left_limited == True)& (right_limited == True):
            print("Exiting for loop, limited")
            break

        ratio = x / iterations_needed

        if (not (msg.servo7 > 1)) & (left_limited == False):            
            averages = AveragePowers(left_hand_powers)
            # print(averages)
            
            # check average current usage and stop if high
            if (msg.servo7 > positions.servo7):
                # hand is closing, if average low continue. Else setpositions to current
                
                left_hand_limit = 500
                if (averages < left_hand_limit):
                    rbex.servo7 = Lerp(positions.servo7, msg.servo7, ratio)
                else:
                    print("current limiting left, backing off")
                    while left_limited == False:
                        averages = AveragePowers(left_hand_powers)
                        if averages > left_hand_limit:
                            rbex.servo7 = rbex.servo7 - 0.01
                            print(rbex.servo7)
                            time.sleep(0.05)
                        else:
                            print("Backed off left.")
                            left_limited = True
                            break
                        
            else:
                # hand opening
                rbex.servo7 = Lerp(positions.servo7, msg.servo7, ratio)

        if (not (msg.servo15 > 1.0)) & (right_limited == False):            
            averages = AveragePowers(right_hand_powers)
            # print(f'r avg:{averages:.4f}')
            
            # check average current usage and stop if high
            if (msg.servo15 > positions.servo15):
                # hand is closing, if average low continue. Else setpositions to current
                
                right_hand_limit = 500
                if (averages < right_hand_limit):
                    rbex.servo15 = Lerp(positions.servo15, msg.servo15, ratio)
                    # print(f'r mov:{ratio:.4f}')
                    
                else:
                    print("right current limiting, backing off")
                    while right_limited == False:
                        averages = AveragePowers(right_hand_powers)
                        if averages > right_hand_limit:
                            rbex.servo15 = rbex.servo15 - 0.02
                            print(f'right hand: {rbex.servo15:.2f}')
                            time.sleep(0.05)
                        else:
                            print("Backed off right.")
                            right_limited = True
                            break
            else:
                # hand opening
                rbex.servo15 = Lerp(positions.servo15, msg.servo15, ratio)

        time.sleep(delayPerLoop)

    # set positions array to current values
    positions.servo7 = rbex.servo7
    positions.servo15 = rbex.servo15

    lerpingInProgress = False

def expander_callback(msg):
    # rospy.loginfo(msg)
    LerpPositions(msg, 1)

def IncIndex(input):
    if input == 2:
        return 0
    else:
        return input + 1

def battery_callback(msg):
    global left_hand_powers, left_hand_powers_index, right_hand_powers, right_hand_powers_index

    if(msg.location=="LEFT_HAND"):
        left_hand_powers[left_hand_powers_index] = msg.current
        left_hand_powers_index = IncIndex(left_hand_powers_index)
    if(msg.location=="RIGHT_HAND"):
        right_hand_powers[right_hand_powers_index] = msg.current
        right_hand_powers_index = IncIndex(right_hand_powers_index)

def listener():
    rospy.init_node('hand_control_node', anonymous=True)
    rospy.Subscriber('/redboard/expander', Expander, expander_callback)
    rospy.Subscriber('joy', Joy, joy_callback)
    rospy.Subscriber('servo_power', BatteryState, battery_callback)

    rospy.spin()

if __name__ == '__main__':
    print("Hand control node listening...")
    signal.signal(signal.SIGINT, handler)
    InitServos()
    listener()
