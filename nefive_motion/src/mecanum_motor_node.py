#!/usr/bin/env python3

import rospy
import time
import threading
import numpy as np 
import math 

from sensor_msgs.msg import Joy
from nefive_msgs.msg import Motors


# time in seconds in not recieving a message before stopping
msg_timeout = rospy.Duration(0.50)
last_msg_received = rospy.Time.from_sec(time.time())

pub = None

WHEEL_RADIUS = 30
TRACK_WIDTH  = 195 
TRACK_LENGTH = 150

# print(f"Calculating wheel speeds for {wz}, {vx}, {vy}")
l = TRACK_LENGTH / 2
w = TRACK_WIDTH / 2
r = WHEEL_RADIUS

kinematic_model = np.array([[-l-w, 1, -1],
                    [ l+w, 1,  1],
                    [ l+w, 1, -1],
                    [-l-w, 1,  1]]) / r

msg = Motors()

# From page 13: https://cdn.intechopen.com/pdfs/465/InTechOmnidirectional_mobile_robot_design_and_implementation.pdf
def steering(vx, vy, wz):
    input_array = np.array([wz,vx,vy])    
    input_array.shape = (3, 1)

    u = np.dot(kinematic_model, input_array)

    # The kinematic model assumes that the motors are laid out as follows:
    #  [3]    [0]
    #     --->
    #  [2]    [1]
    #
    # NE-Five is wired up as follows:
    #  [2]    [1]
    #     --->
    #  [3]    [0]
    
    return [u[1], u[0], u[3], u[2]]

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
        motor_speeds = steering(x, y, z)
        # print(front_left, front_right, back_left, back_right)

        base_motor_speed = 0.2
        turbo_multiplier = 1.25

        # Buttons are on when down so this makes sense in the physical world
        if(data.buttons[4] == 1):
            # Low speed, halve values
            motor_speeds = np.multiply(motor_speeds, base_motor_speed)
        else:
            motor_speeds = np.multiply(motor_speeds, base_motor_speed * turbo_multiplier)

        setmotors(motor_speeds)

    else:
        #  print("Motors not enabled.")
        setmotors([0, 0, 0, 0])

def setmotors(motor_speeds):
    time = rospy.get_time()
    msg.seconds = math.fabs(time)
    msg.nsec = (time - msg.seconds) * 1e9

def publish_adc():
    try:
        # Loop until disconnected
        while not rospy.is_shutdown():
            # Create message
            msg = ADC()
            msg.adc0 = rb.adc0
            msg.adc1 = rb.adc1
            msg.adc2 = rb.adc2
            msg.adc3 = rb.adc3

            # rospy.loginfo(msg)
            pub.publish(msg)

            rospy.sleep(1)

    except rospy.ROSInterruptException:
        pass


def timeout_check(event):
    timediff = rospy.Time.now() - last_msg_received
    if(timediff > msg_timeout):
        print("Timed out: " + str(rospy.Time.now()) + ", " + str(last_msg_received) + ", " + str(timediff))
        setmotors([0, 0, 0, 0])

def listener():
    global pub
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_driver', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)
    pub = rospy.Publisher('adc', ADC, queue_size=10)
    
    # Set up the timer-based timeout check
    rospy.Timer(msg_timeout, timeout_check)
    adcThread = threading.Thread(target=publish_adc)
    adcThread.start()
    adcThread.join()

    rospy.spin()

if __name__ == '__main__':
    print("Motor node listening...")
    listener()
