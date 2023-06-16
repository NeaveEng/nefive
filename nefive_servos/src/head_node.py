#!/usr/bin/env python3

import rospy
from servo_utils import dynamixel_utils
from sensor_msgs.msg import Joy
import signal
import sys


servos = dynamixel_utils('/dev/ttyUSB0', 1000000)

pan_servo = 116
tilt_servo = 117
roll_servo = 118

pan = pan_centre = 0
tilt = tilt_centre = 0
roll = roll_centre = 0

pan_min = -1 + pan_centre
pan_max =  1 + pan_centre

tilt_min = -1 + tilt_centre
tilt_max =  0.5 + tilt_centre

roll_min = -0.4 + roll_centre
roll_max =  0.4 + roll_centre


def reset():
    global pan, tilt
    pan = pan_centre
    tilt = tilt_centre

    servos.setRadian(pan_servo, pan_centre)
    servos.setRadian(tilt_servo, tilt_centre)
    servos.setRadian(roll_servo, roll_centre)

def scaleinput(input, invert, scale):
    if scale != 0:
        scaled = input / scale
    else:
        scaled = input

    if(invert):
        scaled = scaled * -1

    return scaled

def callback(data):
    isXbox = False

    if isXbox == True:
        control_head = not ((data.buttons[4] == 1) or (data.buttons[5] == 1))
    else:
        control_head = data.buttons[2] ==1
    
    # 6 is trigger on NE-Thing and View on Xbox
    reset_head = data.buttons[6] == 1

    if(reset_head):
        print("Resetting head position")
        reset()
    else:
        if(control_head):
            # print(data.axes[3], data.axes[4])
            pan = scaleinput(data.axes[3], True, 9)
            tilt = scaleinput(data.axes[4], True, 9)
            
            if(isXbox == True):
                # roll will be controlled using A as -ve and B as +ve on Xbox
                roll = -scaleinput(data.buttons[0], False, 5) + scaleinput(data.buttons[1], False, 5)
            else:
                roll = scaleinput(data.axes[5], True, 20)
            setservos(pan, tilt, roll)

        
def setservos(pan_diff, tilt_diff, roll_diff):
    global pan, tilt, roll
    pan = pan + pan_diff
    tilt = tilt + tilt_diff
    roll = roll + roll_diff

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

    # print(f"{pan}, {tilt}, {roll}, {pan_diff}, {tilt_diff}, {roll_diff}")

    servos.setRadian(pan_servo, pan)
    servos.setRadian(tilt_servo, tilt)
    servos.setRadian(roll_servo, roll)
    

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('head_driver', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def handler(signum, frame):
    print("ctrl-c pressed, exiting")
    servos.disableAllServos()
    sys.exit()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)

    print("Head node listening...")
    servos.enableAllServos()
    reset()
    listener()
