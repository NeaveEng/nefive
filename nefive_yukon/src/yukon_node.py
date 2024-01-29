#!/usr/bin/env python3

import rospy
import sys
import signal
import serial
import struct
import time
from message_types import MessageTypes
import random
from sensor_msgs.msg import Joy

def handler(signum, frame):
    print("ctrl-c pressed, exiting")
    sys.exit()

message_types = MessageTypes()
last_msg_received = None

# from https://electronics.stackexchange.com/questions/19669/algorithm-for-mixing-2-axis-analog-input-to-control-a-differential-motor-drive
def steering(x, y, rot):
    # x = x * -1
    # y = y * -1
    # rot = rot * -1
    
    front_left = (x - y - rot) # * WHEEL_GEOMETRY) / WHEEL_RADIUS
    front_right = (x + y + rot) # * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_left = (x + y - rot) # * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_right = (x - y + rot) # * WHEEL_GEOMETRY) / WHEEL_RADIUS

    return front_left, front_right, back_left, back_right

def setmotors(front_left, front_right, back_left, back_right):
    ser.write(struct.pack('I', 0))
    data = [rostime.secs, rostime.nsecs, back_right, back_left, front_left,front_right]
    data_packed = struct.pack('IIffff', *data)           
    print(f"Sending: {data_packed}, length: {len(data_packed)}")
    ser.write(data_packed)
    ser.write('\n'.encode('utf-8'))

def callback(data):
    global last_msg_received
    # rospy.loginfo(rospy.get_caller_id() + 'RCVD: %s', data)
    last_msg_received = rospy.Time.now()

    isXbox = False

    if isXbox == True:
        # If buffer buttons presed, arms being controlled so return
        if (data.buttons[4] == 1) or (data.buttons[5] == 1):
            return

        control_movement = True
        x, y, z = data.axes[1], data.axes[3], data.axes[0]

    else:
        control_movement = data.buttons[2] == 1
        x, y, z, torso = -data.axes[1], data.axes[0], data.axes[2], data.axes[6]

    if(control_movement == True):
        # print(data.axes[0], data.axes[1])
        front_left, front_right, back_left, back_right = steering(x, y, z)
        # print(front_left, front_right, back_left, back_right)

        base_motor_speed = 1
        turbo_multiplier = 1.25

        # Buttons are on when down so this makes sense in the physical world
        if(data.buttons[4] == 1):
            # Low speed, halve values
            front_left = front_left     * base_motor_speed
            front_right = front_right   * base_motor_speed
            back_left = back_left       * base_motor_speed
            back_right = back_right     * base_motor_speed
        else:
            front_left = front_left     * base_motor_speed * turbo_multiplier
            front_right = front_right   * base_motor_speed * turbo_multiplier
            back_left = back_left       * base_motor_speed * turbo_multiplier
            back_right = back_right     * base_motor_speed * turbo_multiplier

        setmotors(front_left, front_right, back_left, back_right)

    # else:
        #  print("Motors not enabled.")
        # setmotors(0, 0, 0, 0)


if __name__ == '__main__':
    try:
        rospy.init_node('yukon_node', anonymous=True)
        signal.signal(signal.SIGINT, handler)
        ser = serial.Serial('/dev/ttyS0', 115200)
        ser.flush()

        rospy.Subscriber('ne_five/joy', Joy, callback)
    
        rate = rospy.Rate(1)

        # Loop until disconnected
        while not rospy.is_shutdown():
            rostime = rospy.get_rostime()
            ser.write(struct.pack('I', 2))
            data = [rostime.secs, rostime.nsecs]
            data_packed = struct.pack('II', *data)           
            print(f"Sending: {data_packed}, length: {len(data_packed)}")
            ser.write(data_packed)
            ser.write('\n'.encode('utf-8'))

            rate.sleep()
            
    except rospy.ROSInterruptException:
        print("exiting.")
        pass




