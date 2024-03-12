from pimoroni import NORMAL_DIR  # , REVERSED_DIR
from pimoroni_yukon import Yukon

from pimoroni_yukon import SLOT1 as M1
from pimoroni_yukon import SLOT2 as SERVOS
from pimoroni_yukon import SLOT3 as M2
from pimoroni_yukon import SLOT4 as M3
from pimoroni_yukon import SLOT5 as LEDS
from pimoroni_yukon import SLOT6 as M4
from pimoroni_yukon.modules import BigMotorModule, QuadServoDirectModule, LEDStripModule

from pimoroni_yukon import logging

import math 

"""
Profiler Starting...
Motor 0 Max Forwards: 48.28781, Max Backwards: -50.59612
Motor 1 Max Forwards: 51.17347, Max Backwards: -49.7635
Motor 2 Max Forwards: 53.7041, Max Backwards: -51.14614
Motor 3 Max Forwards: 50.83044, Max Backwards: -53.28336
"""


"""
Profile the speed of a motor across its PWM duty cycle range using its attached encoder for feedback.
This uses a Big Motor + Encoder Module connected to Slot1.

Note that the returned readings will only be valid for a single input voltage.
"""

# Constants
GEAR_RATIO = 30                         # The gear ratio of the motor
ENCODER_CPR = 64                        # The number of counts a single encoder shaft revolution will produce
MOTOR_CPR = GEAR_RATIO * ENCODER_CPR    # The number of counts a single motor shaft revolution will produce

DIRECTION = NORMAL_DIR                  # The direction to spin the motor in. NORMAL_DIR (0), REVERSED_DIR (1)
ZERO_POINT = 0.0                        # The duty cycle that corresponds with zero speed when plotting the motor's speed as a straight line
DEAD_ZONE = 0.0                         # The duty cycle below which the motor's friction prevents it from moving

DUTY_STEPS = 100                        # How many duty cycle steps to sample the speed of
SETTLE_TIME = 0.1                       # How long to wait after changing motor duty cycle
CAPTURE_TIME = 0.1                      # How long to capture the motor's speed at each step

max_forwards  = [-100,-100,-100,-100]
max_backwards = [ 100, 100, 100, 100]

motor_directions = [1, 1, 1, 0]
encoder_directions = [1, 0, 0, 0]

motor_speed_min_max = [[-50.59612, 48.28781], [-49.7635, 51.17347], [-51.14614, 53.7041], [-53.28336, 50.83044]]
motor_speed_scales = []

for min_max in motor_speed_min_max:
    # subtracting the negative value will give a positive value, saves  
    motor_speed_scales.append((math.fabs(min_max[0]) + min_max[1])/2)

print(f"Motor Speed Scales: {motor_speed_scales}")

motors          = [None, None, None, None]
encoders        = [None, None, None, None]

MOTOR_SLOTS = [ M1, M2, M3, M4 ]

# Variables
yukon = Yukon(logging_level=logging.LOG_INFO)                                     # Create a new Yukon object
#module = BigMotorModule(counts_per_rev=MOTOR_CPR)   # Create a BigMotorModule object

# Wrap the code in a try block, to catch any exceptions (including KeyboardInterrupt)
try:
    
    # Configure the motor and encoder modules
    for i in range(4):
        print("Registering motor: ", i)
        motors[i] = BigMotorModule(counts_per_rev=MOTOR_CPR,
                                   encoder_pio=0,    # Create a BigMotorModule object, with details of the encoder
                                   encoder_sm=i,)
        yukon.register_with_slot(motors[i], MOTOR_SLOTS[i])      # Register the BigMotorModule object with the slot

    
    yukon.verify_and_initialise(allow_discrepencies=True, allow_unregistered=True)               # Verify that a BigMotorModule is attached to Yukon, and initialise it
    yukon.enable_main_output()                  # Turn on power to the module slots

    for i in range(4):
        # Set the motor's speed scale, zeropoint, and deadzone
        motors[i].motor.speed_scale(motor_speed_scales[i])
        motors[i].motor.zeropoint(ZERO_POINT)
        motors[i].motor.deadzone(DEAD_ZONE)

        # Set the motor and encoder's direction
        motors[i].motor.direction(motor_directions[i])
        motors[i].encoder.direction(encoder_directions[i])


   # Function that performs a single profiling step
    def profile_at_duty(duty):
        global max_forwards, max_backwards
        
        for motor in range(4):
            # Set the motor to a new duty cycle and wait for it to settle
            motors[motor].motor.duty(duty)

        yukon.monitored_sleep(SETTLE_TIME)

        for motor in range(4):
            # Perform a dummy capture to clear the encoder
            motors[motor].encoder.capture()

        # Wait for the capture time to pass
        yukon.monitored_sleep(CAPTURE_TIME)
        measured_speed = [0, 0, 0, 0]

        for motor in range(4):
            # Perform a capture and read the measured speed
            capture = motors[motor].encoder.capture()
            measured_speed[motor] = capture.radians_per_second
            
            if duty > 0:
                if measured_speed[motor] > max_forwards[motor]:
                    max_forwards[motor] = measured_speed[motor]
            else:   
                if measured_speed[motor] < max_backwards[motor]:
                    max_backwards[motor] = measured_speed[motor]
        
        print(f"{measured_speed[0]}, {measured_speed[1]}, {measured_speed[2]}, {measured_speed[3]}")
        # return motors[motor].read_current()

        # These are some alternate speed measurements from the encoder
        # measured_speed = capture.revolutions_per_minute
        # measured_speed = capture.degrees_per_second
        # measured_speed = capture.radians_per_second

        # Print out the expected and measured speeds, as well as their difference
        # print("Duty =", motor.duty(), end=", ")
        # print("Expected =", motor.speed(), end=", ")
        # print("Measured =", measured_speed, end=", ")
        # print("Diff =", motor.speed() - measured_speed)

    for i in range(4):
        motors[i].enable()       # Enable the motor to get started
        motors[i].motor.enable() # Enable the motor driver on the BigMotorModule

    print("Profiler Starting...")

    current = [0, 0, 0, 0]

    # # Profile from 0% up to one step below 100%
    for i in range(DUTY_STEPS):
        profile_at_duty(i / DUTY_STEPS)

    # Profile from 100% down to one step above 0%
    for i in range(DUTY_STEPS):
        profile_at_duty((DUTY_STEPS - i) / DUTY_STEPS)

    # Profile from 0% down to one step above -100%
    for i in range(DUTY_STEPS):
        profile_at_duty(-i / DUTY_STEPS)

    # Profile from -100% up to one step below 0%
    for i in range(DUTY_STEPS):
        profile_at_duty(-(DUTY_STEPS - i) / DUTY_STEPS)

    # Profile 0% again
    for i in range(4):
        profile_at_duty(0)
    
    for i in range(4):
        # Disable the motor now the profiler has finished
        print(f"Motor {i} Max Forwards: {max_forwards[i]}, Max Backwards: {max_backwards[i]}")
        
    for i in range(4):   
        motors[i].motor.disable()
        
    print("Profiler Finished...")


finally:
    # Put the board back into a safe state, regardless of how the program may have ended
    yukon.reset()