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

from nefive_msgs import Status, Motors, Imu
from blinking import Blinking

import math
from RollingAverage import RollingAverage
import uros
import gc

# Frees up memory in case new code is run without a reset
gc.collect()

"""
All the Yukon functionality to control NE-Five
"""

class NEFive:
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

    uros = None

    servo_module = None
    serial_port = None

    # Variables
    yukon = Yukon()                         # Create a new Yukon object
    phase_offset = 0                        # The offset used to animate the motor

    time = [-1, -1]                     # Time, in [seconds, nsec since seconds]       
    time_updated = None                 # Time in millis() that ROS time was last updated       
    time_mode = 0

    power_update_interval = 1000            # How often to update the power status
    voltage_in_avg = RollingAverage(20)     # Rolling average of the input voltage

    mouth_leds_on = False

    def __init__(self):
        # Configure the motor and encoder modules
        for i in range(4):
            print("Registering motor: ", i)
            self.motors[i] = BigMotorModule(encoder_pio=self.ENCODER_PIO,    # Create a BigMotorModule object, with details of the encoder
                            encoder_sm=i,
                            counts_per_rev=self.MOTOR_CPR,
                            init_motor=True,
                            init_encoder=True)
            self.yukon.register_with_slot(self.motors[i], self.MOTOR_SLOTS[i])      # Register the BigMotorModule object with the slot
        
        # Configure the servo module, double duty as UART
        # init_servos=False means not all pins will be assigned to servos to be used as UART
        self.servo_module = QuadServoDirectModule(init_servos=False)        # Create a QuadServoDirectModule object
        self.yukon.register_with_slot(self.servo_module, SERVOS)  # Register the QuadServoDirectModule object with the slot

        # Configure the LED strip module
        self.led_strip = LEDStripModule( self.STRIP_TYPE,     # Create a LEDStripModule object, with the details of the attached strip(s)
                                    self.STRIP_PIO,
                                    self.STRIP_SM,
                                    self.LEDS_PER_STRIP,
                                    self.BRIGHTNESS)
                
        self.yukon.register_with_slot(self.led_strip, LEDS)  # Register the QuadServoDirectModule object with the slot

        self.yukon.verify_and_initialise(allow_discrepencies=True)               # Verify that all modules are connected and initialise them
        self.yukon.enable_main_output()                  # Turn on power to the module slots

        # Enable the motor modules
        for i in range(4):
            print("Enabling motor: ", i)
            self.motors[i].enable()
            self.motors[i].motor.direction(self.motor_directions[i])
            self.motors[i].encoder.direction(self.encoder_directions[i])
        
        # Enable the LED strip module
        self.led_strip.enable()
        
        # Enable the servo module, start with UART
        self.node=uros.NodeHandle(1, 115200, tx=SERVOS.FAST1, rx=SERVOS.FAST2) 
        self.status = Status()

        self.servo_pins = [SERVOS.FAST3, SERVOS.FAST4]
        self.servos = ServoCluster(self.SERVO_CLUSTER_PIO, self.SERVO_CLUSTER_SM, self.servo_pins)
        self.servos.enable_all()

        # Configure the servo for the linear actuator
        cal = Calibration()
        cal.apply_two_pairs(1000, 2000, 0, 1)
        self.servos.calibration(0, cal)


    def send_motor_speeds(self):
        data = (ticks_ms()/1000, 
                self.encoders[0].revolutions_per_second, 
                self.encoders[1].revolutions_per_second, 
                self.encoders[2].revolutions_per_second, 
                self.encoders[3].revolutions_per_second)
        
        packed_data = ustruct.pack('fffff', *data)  
        # self.serial_port.write(int(0).to_bytes(1, 'big'))  
        # self.serial_port.write(packed_data)  


    # Currently sending dummy data as not yet implmented
    def send_imu_data(self):
        data = [0.1, 0.2, 0.3,
                1.1, 1.2, 1.3,
                2.1, 2.2, 2.3]  
        packed_data = ustruct.pack('fffffffff', *data) 
        # self.serial_port.write(int(1).to_bytes(1, 'big'))
        # self.serial_port.write(packed_data)


    def read_until_newline(self):
        result = bytearray()
        while True:
            char = self.serial_port.read(1)
            if char == None:
                return None
            if char == b'\n':
                break
            result.extend(char)
        return bytes(result)


    def update_eye_leds(self):
            leds_to_update = self.blinking.update_eyelids(self.current_time)
            if leds_to_update is not None:
                # Update the LED strip
                for led in range(24):
                    if led in leds_to_update:
                        self.led_strip.strip.set_rgb(led, 150, 150, 150)
                    else:
                        self.led_strip.strip.set_rgb(led, 0, 0, 0)
                self.led_strip.strip.update()


    def update_mouth_leds(self):
        # 4S battery, Samgsung 25R cells
        min_voltage = 10.25     #   2.5 * 4 + margin
        max_voltage = 16.8      #   4.2 * 4
        self.voltage_in_avg.add(self.yukon.read_input_voltage())

        # Calculate the percentage of the battery remaining
        if self.voltage_in_avg.is_full():
            voltage = self.voltage_in_avg.average()
            percentage = (voltage - min_voltage) / (max_voltage - min_voltage)
            led_voltage_display = int(percentage * 8) + 24
            # print(f"SOC: {int(percentage * 100)}, voltage: {voltage}, LEDs: {led_voltage_display}")
            
            if percentage > 0.1:
                for led in range(24, 32):
                    if led < led_voltage_display:
                        self.led_strip.strip.set_rgb(led, 0, 255, 0)
                    elif led == led_voltage_display:
                        self.led_strip.strip.set_rgb(led, 255, 165, 0)
                    else:
                        self.led_strip.strip.set_rgb(led, 255, 0, 0)
            else:
                # If low on power, flash all the lights
                for led in range(24, 32):
                    if self.mouth_leds_on == True:
                        self.led_strip.strip.set_rgb(led, 0, 0, 0)
                    else:
                        self.led_strip.strip.set_rgb(led, 255, 0, 0)
                
                self.mouth_leds_on = not self.mouth_leds_on

            self.led_strip.strip.update()


    def report_status(self):
        voltage_in = self.yukon.read_input_voltage()
        voltage_out = self.yukon.read_output_voltage()

        # Read Yukon's current sensor, but only if the input voltage
        # is high enough to turn it on, otherwise set the value to zero
        current = self.yukon.read_current() if voltage_in > 2.5 else 0.0

        # Read Yukon's temperature sensor
        temperature = self.yukon.read_temperature()

        time = divmod(ticks_ms(), 1000)
        
        self.status.seconds = time[0]
        self.status.nsec = time[1]
        self.status.rostime = False
        self.status.voltage_in = voltage_in
        self.status.voltage_out = voltage_out
        self.status.current = current
        self.status.temperature = temperature

        self.node.publish("/status", self.status)


    def run(self):
        # Wrap the code in a try block, to catch any exceptions (including KeyboardInterrupt)
        try:
            
            self.last_publish_motors = ticks_ms()
            self.publish_motors_interval = 50

            self.last_publish_imu = ticks_ms()
            self.publish_imu_interval = 10

            loop_ended = ticks_ms()
            
            # Set the mouth to green
            for led in range(24, 32):
                self.led_strip.strip.set_rgb(led, 0, 75, 0)
            self.led_strip.strip.update()

            battery_updated = ticks_ms()
            battery_update_interval = 1000

            while True:
                self.current_time = ticks_ms()                   # Record the start time of the program loop    
                delta_time = self.current_time - loop_ended

                self.update_eye_leds()
                # self.check_for_messages()
                
                if self.current_time > battery_updated + battery_update_interval:
                    self.update_mouth_leds()
                    self.report_status()

                    battery_updated = self.current_time
                
                # if current_time > last_publish_motors + publish_motors_interval:
                #     send_motors()
                #     last_publish_motors = current_time

                # if current_time > last_publish_imu + publish_imu_interval:
                #     send_imu()
                #     last_publish_imu = current_time

                # Monitor sensors until the current time is reached, recording the min, max, and average for each
                # This approach accounts for the updates taking a non-zero amount of time to complete
                self.yukon.monitor_until_ms(ticks_add(self.current_time, 5))
                loop_ended = ticks_ms()
        finally:
            # Put the board back into a safe state, regardless of how the program may have ended
            self.node.shutdown()
            self.yukon.reset()
