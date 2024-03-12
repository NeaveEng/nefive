#!/usr/bin/env python3

import rospy
import sys
import signal
import numpy as np
from sensor_msgs.msg import Joy
from nefive_msgs.msg import Motors
import time

def handler(signum, frame):
    print("ctrl-c pressed, exiting")
    sys.exit()

last_msg_received = None

WHEEL_RADIUS = 0.030
TRACK_WIDTH  = 0.195 
TRACK_LENGTH = 0.150

# print(f"Calculating wheel speeds for {wz}, {vx}, {vy}")
l = TRACK_LENGTH / 2
w = TRACK_WIDTH / 2
r = WHEEL_RADIUS

# The kinematic model assumes that the motors are laid out as follows:
#  [3]    [0]
#     ---> x
#  [2]    [1]
#
# NE-Five is wired up as follows:
#  [2]    [1]
#     ---> x
#  [3]    [0]

front_left  = [-l-w, 1,  1]
front_right = [ l+w, 1, -1]
rear_left   = [-l-w, 1, -1]
rear_right  = [ l+w, 1,  1]

kinematic_model = np.array([front_left, 
                            front_right,
                            rear_left,
                            rear_right]) / r 


# k_const = (l+w)
# kinematic_model = matrix([-k_const, 1,  1], [ k_const, 1, -1], [k_const, 1,  -1], [ -k_const, 1,  1]) * (1/r)
# inverse_kinematic_model = matrix([ 1,  1,  1,  1],[ 1, -1, -1, 1],[-k_const,  k_const,  -k_const, k_const]) * (r/4)

# def motor_speeds(self, vx, vy, wz):
#     input_array = matrix([wz],[vx],[vy])    
#     return self.kinematic_model * input_array

# From page 13: https://cdn.intechopen.com/pdfs/465/InTechOmnidirectional_mobile_robot_design_and_implementation.pdf
def steering(vx, vy, wz):
    input_array = np.array([vx,vy,wz])    
    input_array.shape = (3, 1)
    u = np.dot(kinematic_model, input_array)
    
    return u

motor_msg = Motors()

def setmotors(speeds):
    global pub
    front_left, front_right, back_left, back_right = speeds
    rostime = rospy.get_rostime()
    motor_msg.seconds = rostime.secs
    motor_msg.nsec = rostime.nsecs
    motor_msg.rostime = True
    motor_msg.motor1 = front_right
    motor_msg.motor2 = front_left
    motor_msg.motor3 = back_left
    motor_msg.motor4 = back_right
    
    print(f"Publishing: {motor_msg}")

    pub.publish(motor_msg)

def send_and_latch(msg, duration):
    for _ in range(20):
        setmotors(msg)
        time.sleep(duration / 20)



if __name__ == '__main__':
    try:
        rospy.init_node('yukon_node', anonymous=True)
        signal.signal(signal.SIGINT, handler)

        pub = rospy.Publisher('ne_five/motors', Motors, queue_size=10)
        rospy.loginfo("Publishing to ne_five/motors")
        
        message = Motors()
        speed = 0.15
        
        print("Going forward")
        send_and_latch(steering(speed, 0, 0), 4)
        send_and_latch(steering(0, 0, 0), 1)
        
        print("Going right")
        send_and_latch(steering(0, speed, 0), 4)
        send_and_latch(steering(0, 0, 0), 1)
                
        print("Going backwards")
        send_and_latch(steering(-speed, 0, 0), 4)
        send_and_latch(steering(0, 0, 0), 1)
        
        print("Going left")
        send_and_latch(steering(0, -speed, 0), 4)
        send_and_latch(steering(0, 0, 0), 1)
        
        rospy.signal_shutdown("Exiting.")
            
    except rospy.ROSInterruptException:
        print("exiting.")
        pass




