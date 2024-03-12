#!/usr/bin/env python3

import rospy
import sys
import signal
import numpy as np
from sensor_msgs.msg import Joy
from nefive_msgs.msg import Motors
from nav_msgs.msg import Odometry
from nefive_msgs.msg import Position

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

k_const = (l+w)

front_left  = [1,  1, -l-w]
front_right = [1, -1,  l+w]
rear_left   = [1, -1, -l-w]
rear_right  = [1 , 1,  l+w]

kinematic_model = np.array([front_left, 
                            front_right,
                            rear_left,
                            rear_right]) / r 

# From page 13: https://cdn.intechopen.com/pdfs/465/InTechOmnidirectional_mobile_robot_design_and_implementation.pdf
def steering(vx, vy, wz):
    input_array = np.array([vx, vy, wz])    
    input_array.shape = (3, 1)
    u = np.dot(kinematic_model, input_array)
    return u

motor_msg = Motors()
odom_msg = Odometry()

def setmotors(speeds):
    global motor_pub
    # front_left, front_right, back_left, back_right = speeds
    rostime = rospy.get_rostime()
    motor_msg.seconds = rostime.secs
    motor_msg.nsec = rostime.nsecs
    motor_msg.rostime = True
    motor_msg.motor1 = speeds[0]
    motor_msg.motor2 = speeds[1]
    motor_msg.motor3 = speeds[2]
    motor_msg.motor4 = speeds[3]

    motor_pub.publish(motor_msg)


def odom_callback(data):
    global odom_pub
    rostime = rospy.get_rostime()
    odom_msg.header.stamp.secs = rostime.secs
    odom_msg.header.stamp.nsecs = rostime.nsecs
    odom_msg.header.frame_id = data.frame_id
    odom_msg.child_frame_id = data.child_frame_id
    odom_msg.pose.pose.position.x = data.position_x
    odom_msg.pose.pose.position.y = data.position_y
    odom_msg.pose.pose.orientation.z = data.angle_z

    odom_pub.publish(odom_msg)


def joy_callback(data):
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
        x, y, z, torso = data.axes[1], data.axes[0], -data.axes[2], data.axes[6]

    if(control_movement == True):
        # print(data.axes[0], data.axes[1])
        # [front_left, front_right, back_left, back_right]
        # Rotation needs to be radians per second, multiply z by 3.15 which is 180 degress per second
        rot = z * 3.15
        speeds = steering(x, y, rot)
        # print(front_left, front_right, back_left, back_right)

        # From profiling the motors, they have a max speed of ~50 rad/s
        # With 60mm wheels this works out to a max speed of 1.47 m/s in a straight line
        # As a base speed, 0.5m/s seems reasonable
        base_motor_speed = 0.45
        turbo_multiplier = 1.25

        # Buttons are on when down so this makes sense in the physical world
        if(data.buttons[4] == 1):
            # Low speed, halve values
            speeds = np.multiply(speeds, base_motor_speed)
        else:
            speeds = np.multiply(speeds, base_motor_speed * turbo_multiplier)

        setmotors(speeds)

    else:
        rospy.logdebug("No message received. Stopping motors.")
        setmotors([0, 0, 0, 0])


if __name__ == '__main__':
    try:
        rospy.init_node('yukon_node', anonymous=True)
        signal.signal(signal.SIGINT, handler)

        rospy.Subscriber('ne_five/joy', Joy, joy_callback)    
        rospy.loginfo("Subscribed to ne_five/joy")
        rospy.Subscriber('ne_five/yukon_odom', Position, odom_callback)
        rospy.loginfo("Subscribed to ne_five/yukon_odom")    

        motor_pub = rospy.Publisher('ne_five/motors', Motors, queue_size=1)
        rospy.loginfo("Publishing to ne_five/motors")
        odom_pub = rospy.Publisher('ne_five/odom', Odometry, queue_size=1)
        rospy.loginfo("Publishing to ne_five/odom")
        
        rospy.spin()
            
    except rospy.ROSInterruptException:
        print("exiting.")
        pass




