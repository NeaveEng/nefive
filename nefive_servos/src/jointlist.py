servo_details = {
    "left_arm_left_shoulder_flappy_joint":                  100,
    "left_shoulder_flappy_left_shoulder_foreaft_joint":     101,
    "left_shoulder_foreaft_left_shoulder_rotate_joint":     102,
    "left_shoulder_rotate_left_elbow_joint":                103,
    "left_forearm_left_wrist_rotate_joint":                 104,
    "left_wrist_rotate_left_wrist_pan_joint":               105,
    "left_wrist_pan_left_wrist_tilt_joint":                 106,
    "left_hand":                                            107,
    "right_arm_right_shoulder_flappy_joint":                108,
    "right_shoulder_flappy_right_shoulder_foreaft_joint":   109,
    "right_shoulder_foreaft_right_shoulder_rotate_joint":   110,
    "right_shoulder_rotate_right_elbow_joint":              111,
    "right_forearm_right_wrist_rotate_joint":               112,
    "right_wrist_rotate_right_wrist_pan_joint":             113,
    "right_wrist_pan_right_wrist_tilt_joint":               114,
    "right_hand":                                           115,
    "chest_box_head_pan_joint":                             116,
    "head_pan_head_tilt_joint":                             117,
    "head_tilt_head_roll_joint":                            118
}

servo_torque_constants = {
    100: 1.15,
    101: 1.15,
    102: 1.15,
    103: 1.15,
    104: 0.35,
    105: 0.35,
    106: 0.35,
    107: 0.35,
    108: 1.15,
    109: 1.15,
    110: 1.15,
    111: 1.15,
    112: 0.35,
    113: 0.35,
    114: 0.35,
    115: 0.35,
    116: 0.35,
    117: 0.35,
    118: 0.35 
}

home_positions = {  
    100: 182.51, 101: 116.69, 102: 190.43, 103: 220.88, 104: 189.9, 105: 193.42, 106: 177.23, 107: 141.77, 
    108: 177.85, 109: 89.58, 110: 173.54, 111: 138.16, 112: 179.26, 113: 175.91, 114: 196.86, 115: 226.78,
    116: 173.8, 117: 179.17, 118: 178.29}

current_position_current_limits = {  
    100: 150, 101: 75, 102: 75, 103: 150, 104: 70, 105: 40, 106: 50, 107: 50, 
    108: 100, 109: 50, 110: 75, 111: 150, 112: 70, 113: 40, 114: 50, 115: 50,
    116: 100, 117: 50, 118: 60}

position_mode_current_limits = {  
    100: 910, 101: 910, 102: 910, 103: 910, 104: 1700, 105: 1700, 106: 1700, 107: 1700, 
    108: 910, 109: 910, 110: 910, 111: 910, 112: 1700, 113: 1700, 114: 1700, 115: 1700,
    116: 1700, 117: 1700, 118: 1700}

    # {100: 186.38, 101: 257.58, 102: 101.29, 103: 59.4, 104: -2.73, 105: 104.63, 106: 348.13, 107: 76.82, 
    #  108: 345.58, 109: 147.84, 110: -17.34, 111: 294.89, 112: 348.92, 113: 265.32, 114: 192.81, 115: 7.57}


servo_limits = {100: [1512, 3652], 101: [-967, 4917], 102: [-610, 4350], 103: [-276, 2618], 104: [-824, 5978], 105: [918, 3203], 106: [1087, 3179], 107: [1380, 2497], 108: [1040, 3685], 109: [-1254, 5105], 110: [-39, 4860], 111: [1565, 4156], 112: [-1172, 6545], 113: [999, 3103], 114: [1036, 3085], 115: [1587, 2673]}
