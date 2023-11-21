class ServoDetails:        
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

    # home_positions = {  100: -14.4, 101: -39.83, 102: 13.2, 103: 37.66, 104: 7.92, 105: -4.14, 106: -20.94, 107: -13.29, 
    #                     108: 7.54, 109: 32.43, 110: -17.49, 111: -40.74, 112: -7.39, 113: 2.9, 114: 18.04, 115: 35.83, 
    #                     116: 0, 117: 0.79, 118: -0.62}
    # home_positions = {100: 131.65, 101: 254.5, 102: 178.29, 103: 210.94, 104: 185.86, 105: 163.5, 106: 133.85, 107: 166.67, 
    #                   108: 238.66, 109: 126.28, 110: 197.74, 111: 146.96, 112: 356.84, 113: 174.33, 114: 135.52, 115: 182.95, 
    #                   116: 178.46, 117: 176.88, 118: 178.29}
    
    # home_positions = {100: 185.86, 101: 234.7, 102: 188.5, 103: 215.78, 104: 177.94, 105: 182.34, 106: 156.73, 107: 145.11, 
    #                   108: 177.06, 109: 137.37, 110: 184.89, 111: 142.74, 112: 182.42, 113: 176.26, 114: 208.91, 115: 219.74, 
    #                   116: 178.11, 117: 179.78, 118: 178.82}

    home_positions = {100: -0.97, 101: 69.34, 102: 8.18, 103: 37.22, 104: 1.94, 105: -4.84, 106: -33.97, 107: -24.55, 
                      108: -6.6, 109: -35.38, 110: -2.46, 111: -37.05, 112: 0.26, 113: -9.77, 114: 19.36, 115: 25.08, 
                      116: 2.29, 117: 0.35, 118: -2.55}

    current_limits = {  
        100: 800, 101: 800, 102: 800, 103: 800, 104: 1250, 105: 1250, 106: 1250, 107: 1250, 
        108: 800, 109: 800, 110: 800, 111: 800, 112: 1250, 113: 1250, 114: 1250, 115: 1250,
        116: 1150, 117: 1150, 118: 1150}

    # default all servo limits to 85% of max
    current_goals = {
        100: None, 101:  425, 102:  250, 103:  275, 104: None, 105: None, 106: -300, 107:  100, 
        108: None, 109: -425, 110: -250, 111: -275, 112: None, 113: None, 114:  300, 115: -100,
        116: None, 117: None, 118: None}

    servo_limits = {100: [468, 2528], 101: [-967, 4917], 102: [-610, 4350], 103: [-276, 2618], 104: [-824, 5978], 105: [918, 3203], 106: [1087, 3179], 107: [1604, 2653],
                    108: [1040, 3685], 109: [-1254, 5105], 110: [-39, 4860], 111: [1565, 4156], 112: [-1172, 6545], 113: [999, 3103], 114: [1036, 3085], 115: [1425, 2494]}

    def __init__(self):
        for id, _ in self.current_limits.items():
            self.current_goals[id] = int(self.current_limits[id] * 0.85)

        # Set the and effectors to 50%
        self.current_goals[107] = int(self.current_limits[107] * 0.5)
        self.current_goals[115] = int(self.current_limits[115] * 0.5)
