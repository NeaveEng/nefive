#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from servo_utils import dynamixel_utils
from jointlist import servo_details, servo_torque_constants
import signal
import sys


if __name__ == '__main__':
    def handler(signum, frame):
        print("ctrl-c pressed, exiting")
        servos.disableAllServos()
        sys.exit()

    try:
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node('dynamixel_node', anonymous=True)

        servos = dynamixel_utils('/dev/ttyUSB0', 4000000)
        servos.disableAllServos()

        signal.signal(signal.SIGINT, handler)

        # Create the joint state msg and populate with default values
        msg = JointState()
        for joint_name, servo_id in servo_details.items():
            msg.name.append(joint_name)
            
        # Loop until disconnected
        while True:
            # Create message
            msg.header.stamp = rospy.Time.now()
            msg.position = []
            msg.velocity = []
            msg.effort = []

            radians = servos.readAllRadians()
            velocities = servos.readAllVelocities()
            efforts = servos.readAllEfforts()

            # Update for gearing, flappy is 24/20 (0.83333) and foreaft is 28/20 (0.71429)
            radians[100] = radians[100] * 0.83333
            radians[101] = radians[101] * 0.71429
            radians[108] = radians[108] * 0.83333
            radians[109] = radians[109] * 0.71429

            for joint_name, servo_id in servo_details.items():
                msg.position.append(radians[servo_id])
                msg.velocity.append(velocities[servo_id])
                msg.effort.append(efforts[servo_id])            
            
            # rospy.loginfo(msg)
            pub.publish(msg)
    

    except rospy.ROSInterruptException:
        servos.disableAllServos()
        print("Servos disabled, exiting.")
        pass

