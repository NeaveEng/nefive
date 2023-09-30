#!/usr/bin/env python3
import rospy
from nefive_servos.msg import arm_servos, servo_position
from evdev import InputDevice, categorize, ecodes

rospy.init_node('barrel_sorting_node', anonymous=True)
arm_pub = rospy.Publisher('/nefive_servos/arms', arm_servos, queue_size=10, latch=False)
hand_pub = rospy.Publisher('/nefive_servos/servo_position', servo_position, queue_size=10, latch=False)

dev = InputDevice('/dev/input/event1')
print(dev)

# Key Codes
# values: up: 0, down: 1, held: 2
#
# Key	    Code
# Q	        16
# W	        17
# E	        18
# R	        19
# A	        30
# S	        31
# D	        32
# F	        33
# Z	        44
# X	        45
# Space	    57
# N	        49
# M	        50
# K	        37
# L	        38
# O	        24
# P	        25


def DoAction(event):
    global arm_pub, hand_pub

    if event.value == 1:
        arm_msg = arm_servos()
        hand_msg = servo_position()
        # left open         1715
        # left closed       2062

        # right open        2501
        # right close       2047

        left_ready_to_grip = {100: 29.456330368667842, 101: 30.146035868749024, 102: 51.733146739006045, 103: -56.28475747831166, 104: -26.600920701324938, 105: -42.656178196296096, 106: -48.81609113566577, 107: -13.29}
        left_gripped_ready_to_lift = {100: 29.456330368667842, 101: 30.146035868749024, 102: 51.733146739006045, 103: -56.28475747831166, 104: -26.600920701324938, 105: -42.656178196296096, 106: -48.81609113566577, 107: -13.29}
        left_lifted = {100: 29.456330368667842, 101: 65.27122517310083, 102: 51.733146739006045, 103: -56.28475747831166, 104: -26.600920701324938, 105: -42.656178196296096, 106: -49.53141417779028, 107: -13.29}
        left_held_at_rest = {100: -5.869651304185389, 101: -48.732719454839824, 102: -0.09916764795779898, 103: 26.942624656334516, 104: -0.17195779636502095, 105: -7.014784810468555, 106: -59.568178859129546, 107: -13.29}

        right_ready_to_grip = {108: -53.19365683853626, 109: -45.383102371990674, 110: -27.94508296228945, 111: 77.38949466519057, 112: 4.063222712501883, 113: 29.623704479634767, 114: 61.54889508984983, 115: 51.83}
        right_gripped_ready_to_lift = {108: -53.19365683853626, 109: -45.383102371990674, 110: -27.94508296228945, 111: 77.38949466519057, 112: 4.063222712501883, 113: 29.623704479634767, 114: 61.54889508984983, 115: 51.83}
        right_lifted = {108: -37.78211537100375, 109: -80.4172634088248, 110: -31.895054783448572, 111: 77.38949466519057, 112: -17.342242761850358, 113: -8.162528710812325, 114: 91.04760551914572, 115: 51.83}
        right_held_at_rest = {108: -2.3999906154721984, 109: 34.793914288580424, 110: -2.8339665772020766, 111: -33.01042610011996, 112: -4.283606642261148, 113: -8.162528710812325, 114: 35.09518693663179, 115: 51.83}    

        if event.code == 16:    #   Q
            print("left ready to grip")
            arm_msg.arm = 0
            arm_msg.shoulder_flappy = left_ready_to_grip[100]
            arm_msg.shoulder_foraft = left_ready_to_grip[101]
            arm_msg.upperarm_rotate = left_ready_to_grip[102]
            arm_msg.elbow           = left_ready_to_grip[103]
            arm_msg.wrist_rotate    = left_ready_to_grip[104]
            arm_msg.wrist_pan       = left_ready_to_grip[105]
            arm_msg.wrist_tilt      = left_ready_to_grip[106]

            arm_pub.publish(arm_msg)

            hand_msg.id = 107
            hand_msg.goal_pos = 1715
            hand_pub.publish(hand_msg)
            return
            

        if event.code == 17:    #   W
            print("left gripped ready to lift")
            
            hand_msg.id = 107
            hand_msg.goal_pos = 2062
            hand_pub.publish(hand_msg)
            return


        if event.code == 18:    #   E
            print("left lifted")
            arm_msg.arm = 0
            arm_msg.shoulder_flappy = left_lifted[100]
            arm_msg.shoulder_foraft = left_lifted[101]
            arm_msg.upperarm_rotate = left_lifted[102]
            arm_msg.elbow           = left_lifted[103]
            arm_msg.wrist_rotate    = left_lifted[104]
            arm_msg.wrist_pan       = left_lifted[105]
            arm_msg.wrist_tilt      = left_lifted[106]

            arm_pub.publish(arm_msg)


        if event.code == 19:    #   R
            print("left held at rest")
            arm_msg.arm = 0
            arm_msg.shoulder_flappy = left_held_at_rest[100]
            arm_msg.shoulder_foraft = left_held_at_rest[101]
            arm_msg.upperarm_rotate = left_held_at_rest[102]
            arm_msg.elbow           = left_held_at_rest[103]
            arm_msg.wrist_rotate    = left_held_at_rest[104]
            arm_msg.wrist_pan       = left_held_at_rest[105]
            arm_msg.wrist_tilt      = left_held_at_rest[106]

            arm_pub.publish(arm_msg)
            return

        if event.code == 30:    #   A
            print("right ready to grip")
            arm_msg.arm = 1
            arm_msg.shoulder_flappy = right_ready_to_grip[108]
            arm_msg.shoulder_foraft = right_ready_to_grip[109]
            arm_msg.upperarm_rotate = right_ready_to_grip[110]
            arm_msg.elbow           = right_ready_to_grip[111]
            arm_msg.wrist_rotate    = right_ready_to_grip[112]
            arm_msg.wrist_pan       = right_ready_to_grip[113]
            arm_msg.wrist_tilt      = right_ready_to_grip[114]
            
            arm_pub.publish(arm_msg)

            hand_msg.id = 115
            hand_msg.goal_pos = 2501
            hand_pub.publish(hand_msg)
            return

        if event.code == 31:    #   S
            print("right gripped ready to lift")

            hand_msg.id = 115
            hand_msg.goal_pos = 2047
            hand_pub.publish(hand_msg)
            return

        if event.code == 32:    #   D
            print("right lifted")
            arm_msg.arm = 1
            arm_msg.shoulder_flappy = right_lifted[108]
            arm_msg.shoulder_foraft = right_lifted[109]
            arm_msg.upperarm_rotate = right_lifted[110]
            arm_msg.elbow           = right_lifted[111]
            arm_msg.wrist_rotate    = right_lifted[112]
            arm_msg.wrist_pan       = right_lifted[113]
            arm_msg.wrist_tilt      = right_lifted[114]
            
            arm_pub.publish(arm_msg)
            return


        if event.code == 33:    #   F
            print("right held at rest")
            arm_msg.arm = 1
            arm_msg.shoulder_flappy = right_held_at_rest[108]
            arm_msg.shoulder_foraft = right_held_at_rest[109]
            arm_msg.upperarm_rotate = right_held_at_rest[110]
            arm_msg.elbow           = right_held_at_rest[111]
            arm_msg.wrist_rotate    = right_held_at_rest[112]
            arm_msg.wrist_pan       = right_held_at_rest[113]
            arm_msg.wrist_tilt      = right_held_at_rest[114]

            arm_pub.publish(arm_msg)
            return

for event in dev.read_loop():
    if event.type == ecodes.EV_KEY:
        DoAction(event)
