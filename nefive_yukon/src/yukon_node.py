#!/usr/bin/env python3

import rospy
import sys
import signal
from sensor_msgs.msg import Joy
from nefive_msgs.msg import Motors

def handler(signum, frame):
    print("ctrl-c pressed, exiting")
    sys.exit()

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

motor_msg = Motors()

def setmotors(front_left, front_right, back_left, back_right):
    global pub
    rostime = rospy.get_rostime()
    motor_msg.seconds = rostime.secs
    motor_msg.nsec = rostime.nsecs
    motor_msg.rostime = True
    motor_msg.motor1 = front_right
    motor_msg.motor2 = front_left
    motor_msg.motor3 = back_left
    motor_msg.motor4 = back_right

    pub.publish(motor_msg)
    

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

        base_motor_speed = 0.5
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

        setmotors(front_right, front_left, back_left, back_right)

    # else:
        #  print("Motors not enabled.")
        # setmotors(0, 0, 0, 0)


if __name__ == '__main__':
    try:
        rospy.init_node('yukon_node', anonymous=True)
        signal.signal(signal.SIGINT, handler)

        rospy.Subscriber('ne_five/joy', Joy, callback)
        pub = rospy.Publisher('ne_five/motors', Motors, queue_size=1)
    
        rospy.spin()
            
    except rospy.ROSInterruptException:
        print("exiting.")
        pass




