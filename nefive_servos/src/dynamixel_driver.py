#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from servo_utils import dynamixel_utils
from jointlist import servo_details, servo_torque_constants


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node('dynamixel_node', anonymous=True)

        servos = dynamixel_utils('/dev/ttyUSB0', 1000000)
        servos.disableAllServos()

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

            positions = servos.readAllRadians()

            # Update for gearing, flappy is 24/20 (0.83333) and foreaft is 28/20 (0.71429)
            positions[100] = positions[100] * 0.83333
            positions[101] = positions[101] * 0.71429
            positions[108] = positions[108] * 0.83333
            positions[109] = positions[109] * 0.71429

            velocities = servos.readAllVelocities()
            currents = servos.readAllCurrent()

            for joint_name, servo_id in servo_details.items():
                msg.position.append(positions[servo_id])
                msg.velocity.append(velocities[servo_id])
                msg.effort.append(currents[servo_id] * servo_torque_constants[servo_id])            
            
            # rospy.loginfo(msg)
            pub.publish(msg)
    

    except rospy.ROSInterruptException:
        servos.disableAllServos()
        print("Servos disabled, exiting.")
        pass

