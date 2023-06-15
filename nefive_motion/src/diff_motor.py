#!/usr/bin/env python3

import rospy
import redboard
import math
import time

from sensor_msgs.msg import Joy

# time in seconds in not recieving a message before stopping
# msg_timeout = rospy.Duration(0.5)
# last_msg_received = rospy.Time.from_sec(time.time())

rb = redboard.RedBoard()

motor0 = 0
motor1 = 0
motor2 = 0
motor3 = 0

rb.m0_invert = False         # front left 
rb.m1_invert = False        # front right
rb.m2_invert = True         # rear left
rb.m3_invert = True        # rear right

# from https://electronics.stackexchange.com/questions/19669/algorithm-for-mixing-2-axis-analog-input-to-control-a-differential-motor-drive
def steering(x, y):
    # convert to polar
    r = math.hypot(x, y)
    t = math.atan2(y, x)

    # rotate by 45 degrees
    t -= math.pi / 4

    # back to cartesian
    left = r * math.sin(t)
    right = r * math.cos(t)

    # rescale the new coords
    left = left * math.sqrt(2)
    right = right * math.sqrt(2)

    # clamp to -1/+1
    left = max(-1, min(left, 1))
    right = max(-1, min(right, 1))

    return right, left

def callback(data):
    # global last_msg_received
    # rospy.loginfo(rospy.get_caller_id() + 'RCVD: %s', data)
    # last_msg_received = rospy.Time.now()

    isXbox = False

    if isXbox == True:
        # If buffer buttons presed, arms being controlled so return
        if (data.buttons[4] == 1) or (data.buttons[5] == 1):
            return
        control_movement = True
        high_speed = data.buttons[2] == 0 # X
        x, y = -data.axes[0], data.axes[1]
    else:
        control_movement = data.buttons[2] == 1
        high_speed = data.buttons[4] == 0
        x, y = data.axes[0], data.axes[1]

    if control_movement == True:
        # print(data.axes[0], data.axes[1])
        left, right = steering(x, y)

        # Buttons are on when down so this makes sense in the physical world
        if(high_speed == True):
            # Low speed, halve values
            left = left * 0.25
            right = right * 0.25
        else:
            left = left * 0.5
            right = right * 0.5

        setmotors(left, right)
    else:
        #  print("Motors not enabled.")
        setmotors(0, 0)

def setmotors(m1, m2):
    rb.m0 = m1
    rb.m1 = m2
    rb.m2 = m1
    rb.m3 = m2

# def timeout_check(event):
#     timediff = rospy.Time.now() - last_msg_received
#     if(timediff > msg_timeout):
#         print("Timed out: " + str(rospy.Time.now()) + ", " + str(last_msg_received) + ", " + str(timediff))
#         setmotors(0, 0)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_driver', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)

    # Set up the timer-based timeout check
    # rospy.Timer(msg_timeout, timeout_check)

    rospy.spin()

if __name__ == '__main__':
    print("Motor node listening...")
    listener()
