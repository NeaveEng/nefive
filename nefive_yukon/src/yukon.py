import time
from message_types import MessageTypes
from machine import UART
import ustruct
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

from message_types import MessageTypes
from blinking import Blinking


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

message_types = MessageTypes()
blinking = Blinking()

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
serial_port = None

# Variables
yukon = Yukon()                         # Create a new Yukon object
phase_offset = 0                        # The offset used to animate the motor

ros_time = [-1, -1]                     # ROS time, in [seconds, nsec since seconds]       

def initialise():
    global motors, led_strip, servos, servo_module, serial_port
    
    # Configure the motor and encoder modules
    for i in range(4):
        print("Registering motor: ", i)
        motors[i] = BigMotorModule(encoder_pio=ENCODER_PIO,    # Create a BigMotorModule object, with details of the encoder
                        encoder_sm=i,
                        counts_per_rev=MOTOR_CPR,
                        init_motor=True,
                        init_encoder=True)
        yukon.register_with_slot(motors[i], MOTOR_SLOTS[i])      # Register the BigMotorModule object with the slot
    
    # Configure the servo module, double duty as UART
    # init_servos=False means not all pins will be assigned to servos to be used as UART
    servo_module = QuadServoDirectModule(init_servos=False)        # Create a QuadServoDirectModule object
    yukon.register_with_slot(servo_module, SERVOS)  # Register the QuadServoDirectModule object with the slot

    # Configure the LED strip module
    led_strip = LEDStripModule(STRIP_TYPE,     # Create a LEDStripModule object, with the details of the attached strip(s)
                                STRIP_PIO,
                                STRIP_SM,
                                LEDS_PER_STRIP,
                                BRIGHTNESS)
            

    led_strip = LEDStripModule(STRIP_TYPE, STRIP_PIO, STRIP_SM, LEDS_PER_STRIP, BRIGHTNESS)
    yukon.register_with_slot(led_strip, LEDS)  # Register the QuadServoDirectModule object with the slot

    yukon.verify_and_initialise()               # Verify that all modules are connected and initialise them
    yukon.enable_main_output()                  # Turn on power to the module slots

    # Enable the motor modules
    for i in range(4):
        print("Enabling motor: ", i)
        motors[i].enable()
        motors[i].motor.direction(motor_directions[i])
        motors[i].encoder.direction(encoder_directions[i])
    
    # Enable the LED strip module
    led_strip.enable()
    
    # Enable the servo module, start with UART
    serial_port = UART(1, 115200, tx=SERVOS.FAST1, rx=SERVOS.FAST2, rxbuf=1024) #node initialized, for tx2/rx2 and 115200 baudrate
    serial_port.init(115200, bits=8, parity=None, stop=1)
    serial_port.read()

    servo_pins = [SERVOS.FAST3, SERVOS.FAST4]
    servos = ServoCluster(SERVO_CLUSTER_PIO, SERVO_CLUSTER_SM, servo_pins)
    servos.enable_all()

    # Configure the servo for the linear actuator
    cal = Calibration()
    cal.apply_two_pairs(1000, 2000, 0, 1)
    servos.calibration(0, cal)


def send_motors():
    data = (ticks_ms()/1000, encoders[0].revolutions_per_second, encoders[1].revolutions_per_second, encoders[2].revolutions_per_second, encoders[3].revolutions_per_second)  # A tuple of data
    packed_data = ustruct.pack('fffff', *data)  # Pack the data into a binary format
    serial_port.write(int(0).to_bytes(1, 'big'))  # Send Motor data
    serial_port.write(packed_data)  # Send the packed data


def send_imu():
    data = [0.1, 0.2, 0.3,
            1.1, 1.2, 1.3,
            2.1, 2.2, 2.3]  # A tuple of data
    packed_data = ustruct.pack('fffffffff', *data)  # Pack the data into a binary format
    serial_port.write(int(1).to_bytes(1, 'big'))  # Send Motor data
    serial_port.write(packed_data)  # Send the packed data


def read_until_newline(serial_port):
    result = bytearray()
    while True:
        char = serial_port.read(1)
        if char == None:
            return None
            continue
        if char == b'\n':
            break
        result.extend(char)
    return bytes(result)


# Wrap the code in a try block, to catch any exceptions (including KeyboardInterrupt)
try:
    initialise()

    last_publish_motors = ticks_ms()
    publish_motors_interval = 50

    last_publish_imu = ticks_ms()
    publish_imu_interval = 10

    loop_ended = ticks_ms()
    
    # Clear the buffer
    read_until_newline(serial_port)
    for led in range(24, 32):
        led_strip.strip.set_rgb(led, 0, 75, 0)
    led_strip.strip.update()

    while True:
        current_time = ticks_ms()                   # Record the start time of the program loop    
        delta_time = current_time - loop_ended

        leds_to_update = blinking.update_eyelids(current_time)
        if leds_to_update is not None:
            # Update the LED strip
            for led in range(24):
                if led in leds_to_update:
                    led_strip.strip.set_rgb(led, 150, 150, 150)
                else:
                    led_strip.strip.set_rgb(led, 0, 0, 0)
            led_strip.strip.update()

        if serial_port is not None:
            in_waiting = serial_port.any()
            if in_waiting > 0:           
                data = read_until_newline(serial_port) 
                # print(f"Waiting: {in_waiting}")
                if data is None:
                    serial_port.read()
                    continue

                try:

                    # Read the first four bytes, this will be an int that represents the next packet of data
                    index = ustruct.unpack('I', data[0:4])[0]

                    # # if index not in message_types.message_ids:
                    # if index not in message_types.message_ids.values():
                    #     print(f"Unknown message type: {index}")
                    #     serial_port.read()
                    #     continue
                    # else:
                    #     print(f"Message type: {index}")

                    print(index)
                    # Read the data for the packet
                    if index == message_types.message_ids['time']:
                        fmt = message_types.message_types[index]['format']
                        size = message_types.message_types[index]['size']
                        new_ros_time = ustruct.unpack(fmt, data[4: 4+size])
                        # print(f"New time: {new_ros_time}, old time: {ros_time}")
                        ros_time = new_ros_time

                    if index == message_types.message_ids['motors']:
                        fmt = message_types.message_types[index]['format']
                        size = message_types.message_types[index]['size']
                        motor_msg = ustruct.unpack(fmt, data[4: 4+size])
                        print(f"motors: {motor_msg}")
                        motors[0].motor.speed(motor_msg[2])
                        motors[1].motor.speed(motor_msg[3])
                        motors[2].motor.speed(motor_msg[4])
                        motors[3].motor.speed(motor_msg[5])



                except ValueError as vs:
                    print(f"ValueError: {vs}")
                    continue
        
        # if current_time > last_publish_motors + publish_motors_interval:
        #     send_motors()
        #     last_publish_motors = current_time

        # if current_time > last_publish_imu + publish_imu_interval:
        #     send_imu()
        #     last_publish_imu = current_time

        # print(servos.servos[0].calbration())
        # Advance the current time by a number of seconds
        current_time = ticks_add(current_time, 5)

        # Monitor sensors until the current time is reached, recording the min, max, and average for each
        # This approach accounts for the updates taking a non-zero amount of time to complete
        yukon.monitor_until_ms(current_time)

        loop_ended = ticks_ms()
finally:
    # Put the board back into a safe state, regardless of how the program may have ended
    # if node is not None:
    #     node.shutdown()
    yukon.reset()
