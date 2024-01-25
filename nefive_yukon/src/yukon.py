import math
from pimoroni_yukon import Yukon
from pimoroni_yukon import SLOT1 as M1
from pimoroni_yukon import SLOT2 as SERVOS
from pimoroni_yukon import SLOT3 as M2
from pimoroni_yukon import SLOT4 as M3
from pimoroni_yukon import SLOT5 as LEDS
from pimoroni_yukon import SLOT6 as M4
from pimoroni_yukon.modules import BigMotorModule, QuadServoDirectModule, LEDStripModule
from pimoroni_yukon.timing import ticks_ms, ticks_add
from servo import ServoCluster, Calibration
from machine import UART
import time
import uros
from std_msgs import ColorRGBA #message object ColorRGBA
import gc


gc.collect()

"""
Drive a single motor from a Big Motor + Encoder Module connected to Slot1.
A wave pattern will be played on the attached motor, and its speed printed out.

Press "Boot/User" to exit the program.
"""

# Constants
GEAR_RATIO = 30                         # The gear ratio of the motor
ENCODER_CPR = 64                        # The number of counts a single encoder shaft revolution will produce
MOTOR_CPR = GEAR_RATIO * ENCODER_CPR    # The number of counts a single motor shaft revolution will produce

# PIO and State Machine Usage
ENCODER_PIO         = 0                 # The PIO system to use (0 or 1) for the motor's encoder
ENCODER_SM          = 0                 # Four encoders so SM 0..3 are used

STRIP_PIO           = 1                 # Each strip uses 1 SM, we hopefully have three available
STRIP_SM            = 1

SERVO_CLUSTER_PIO   = 1
SERVO_CLUSTER_SM    = 0

SPEED = 0.005                           # How much to advance the motor phase offset by each update
UPDATES = 50                            # How many times to update the motors per second
SPEED_EXTENT = 0.5                      # How far from zero to drive the motors

STRIP_TYPE     = LEDStripModule.NEOPIXEL
LEDS_PER_STRIP = 32
BRIGHTNESS     = 0.75
SLEEP = 0.02                            # The time to sleep between each update
SPEED = 0.01                            # How much to advance the rainbow hue offset by each update
RAINBOW_SAT = 1.0                       # The saturation of the rainbow
RAINBOW_VAL = 1.0                       # The value (brightness) of the rainbow

MOTOR_SLOTS = [ M1, M2, M3, M4 ]

node = None
msg = None

# 0: Front Left, 1: Front Right, 2: Rear Right, 3: Rear Left

# Motor Directions: 0: Normal, 1: Reversed
motor_directions = [1, 1, 1, 0]
encoder_directions = [1, 0, 0, 0]

motors      = [None, None, None, None]
encoders    = [None, None, None, None]
led_strip   = None
servos      = None

hue_offset = 0

servo_module = None

# Variables
yukon = Yukon()                         # Create a new Yukon object
phase_offset = 0                        # The offset used to animate the motor

# import uros
# from std_msgs import ColorRGBA #message object ColorRGBA

# while True:
#     node.publish('Colorsh',msg) #publish data to node Colorsh
#     sleep(1)

# Function for applying a rainbow pattern to an LED strip
def update_rainbow(strip, offset):
    for led in range(strip.num_leds()):
        # Calculate a hue for the LED based on its position in the strip and the offset
        hue = (led / strip.num_leds()) + offset
        strip.set_hsv(led, hue, RAINBOW_SAT, RAINBOW_VAL)

    # Send the new colours to the LED strip
    strip.update()


def init_all_the_things():
    global motors, led_strip, servos, servo_module, serial_port, node, msg
    for i in range(4):
        print("Registering motor: ", i)
        motors[i] = BigMotorModule(encoder_pio=ENCODER_PIO,    # Create a BigMotorModule object, with details of the encoder
                        encoder_sm=i,
                        counts_per_rev=MOTOR_CPR,
                        init_motor=True,
                        init_encoder=True)
        yukon.register_with_slot(motors[i], MOTOR_SLOTS[i])      # Register the BigMotorModule object with the slot
    
        
    
    # init_servos=False means not all pins will be assigned to servos
    servo_module = QuadServoDirectModule(init_servos=False)        # Create a QuadServoDirectModule object
    yukon.register_with_slot(servo_module, SERVOS)  # Register the QuadServoDirectModule object with the slot

    led_strip = LEDStripModule(STRIP_TYPE,     # Create a LEDStripModule object, with the details of the attached strip(s)
                                STRIP_PIO,
                                STRIP_SM,
                                LEDS_PER_STRIP,
                                BRIGHTNESS)
            
    yukon.register_with_slot(led_strip, LEDS)  # Register the QuadServoDirectModule object with the slot

    yukon.verify_and_initialise()               # Verify that a BigMotorModule is attached to Yukon, and initialise it

    yukon.enable_main_output()                  # Turn on power to the module slots
    
    led_strip.enable()
        
    # UARTpins SERVOS.FAST1, SERVOS.FAST2, 

    msg=ColorRGBA() #msg object init
    msg.r=1 #values to variables assigned
    msg.g=3
    msg.b=4
    msg.a=1
    
    node=uros.NodeHandle(1, 115200, tx=SERVOS.FAST1, rx=SERVOS.FAST2) #node initialized, for tx2/rx2 and 115200 baudrate
    # serial_port = UART(1, 115200, tx=SERVOS.FAST1, rx=SERVOS.FAST2)
    # serial_port.init(115200, bits=8, parity=None, stop=1)

    servo_pins = [SERVOS.FAST3, SERVOS.FAST4]
    servos = ServoCluster(SERVO_CLUSTER_PIO, SERVO_CLUSTER_SM, servo_pins)
    servos.enable_all()
    print(servos.calibration(0))

    cal = Calibration()
    cal.apply_two_pairs(1000, 2000, 0, 1)
    servos.calibration(0, cal)
    print(servos.calibration(0))
    
    print(servos)
    # servos.calibration(1)
    
    for i in range(4):
        print("Enabling motor: ", i)
        motors[i].enable()
        motors[i].motor.direction(motor_directions[i])
        motors[i].encoder.direction(encoder_directions[i])

                            # Enable the motor driver on the BigMotorModule


def test_motors():
    run_loop = True
    
    i = 0

    # Loop until the BOOT/USER button is pressed
    while run_loop:
        current_time = ticks_ms()                   # Record the start time of the program loop   

        if yukon.is_boot_pressed():
            motors[i].motor.speed(0)
            if i == 3:
                # run_loop = False
                i = 0
            else:
                i = i + 1

            while yukon.is_boot_pressed():
                time.sleep(0.01)

#        phase = phase_offset * math.pi * 2
#        speed = math.sin(phase) * SPEED_EXTENT
        speed = 0.2
        
        encoders[i] = motors[i].encoder.capture()                  # Capture the state of the encoder
        motors[i].motor.speed(speed)
        print(f"Motor: {i}, Speed: {speed}, Encoder: {encoders[i].revolutions_per_second}")             
            
        # Advance the current time by a number of seconds
        current_time = ticks_add(current_time, int(1000 / UPDATES))

        # Monitor sensors until the current time is reached, recording the min, max, and average for each
        # This approach accounts for the updates taking a non-zero amount of time to complete
        yukon.monitor_until_ms(current_time)

def run_motors_forwards():
    global motors, encoders

    for i in range(4):
        speed = 0.2
        encoders[i] = motors[i].encoder.capture()                  # Capture the state of the encoder
        motors[i].motor.speed(speed)
        # print(f"Motor {i} is not enabled: {motors[i].read_fault()}")

    # print(f"Motor: {i}, Speed: {speed}, Encoder: {encoders[i].revolutions_per_second}")             
    print(f"{encoders[0].revolutions_per_second}, {encoders[1].revolutions_per_second}, {encoders[2].revolutions_per_second}, {encoders[3].revolutions_per_second}")  


def test_actuator():
    loop_min = 0.25
    loop_max = 0.9
    loop_current = None
    loop_direction = 1
    
    loop_diff = loop_max - loop_min
    loop_step = 1
    loop_unit = loop_diff / 100

    print("up")
    for i in range(0, 100, 1):
        position_voltage = servo_module.read_adc1(5)
        position = 1 - position_voltage / 3.3
        print(f"Position: {position}, current_loop: {i}, value: {servos.value(0)}")

        servos.value(0, i * loop_unit + loop_min)
        time.sleep(0.2)

    print("down")
    for i in range(100, 0, -1):
        position_voltage = servo_module.read_adc1(5)
        position = 1 - position_voltage / 3.3
        print(f"Position: {position}, current_loop: {i}, value: {servos.value(0)}")

        servos.value(0, i * loop_unit + loop_min)
        time.sleep(0.2)

# Wrap the code in a try block, to catch any exceptions (including KeyboardInterrupt)
try:
    init_all_the_things()

    position_voltage = servo_module.read_adc1(5)
    loop_current = position = 1 - position_voltage / 3.3
    print("Start position: ", position)

    # servos.value(0, 0)
    # time.sleep(10)

    # position_voltage = servo_module.read_adc1(5)
    # loop_current = position = 1 - position_voltage / 3.3
    # print("Lowest position: ", position)

    # servos.value(0, 1)
    # time.sleep(10)

    # position_voltage = servo_module.read_adc1(5)
    # loop_current = position = 1 - position_voltage / 3.3
    # print("Highest position: ", position)


    last_publish = ticks_ms()
    publish_interval = 1000

    # test_motors()

    while True:
        current_time = ticks_ms()                   # Record the start time of the program loop    
        # update_rainbow(led_strip.strip, hue_offset)

        # # Advance the hue offset, wrapping if it exceeds 1.0
        # hue_offset += SPEED
        # if hue_offset >= 1.0:
        #     hue_offset -= 1.0          

        for led in range(24):
            led_strip.strip.set_rgb(led, 255, 255, 255)


        for led in range(24, 32):
            led_strip.strip.set_rgb(led, 0, 255, 0)

        led_strip.strip.update()


        if msg is not None:
            if current_time > last_publish + publish_interval:
                #print(msg.r, msg.g, msg.b, msg.a)
                node.publish('Colorsh',msg)
                #serial_port.write(f"{yukon.read_input_voltage(5)}\n")
                last_publish = current_time

        run_motors_forwards()


        # print(servos.servos[0].calbration())
        # Advance the current time by a number of seconds
        current_time = ticks_add(current_time, int(1000 / UPDATES))

        # Monitor sensors until the current time is reached, recording the min, max, and average for each
        # This approach accounts for the updates taking a non-zero amount of time to complete
        yukon.monitor_until_ms(current_time)

finally:
    # Put the board back into a safe state, regardless of how the program may have ended
    node.shutdown()
    yukon.reset()
