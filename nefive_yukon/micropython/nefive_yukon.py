# This is a filthy hack to pre-allocate memory
# Each will be set to None and gc.collect() will be called to free it up before it's needed
space_for_servo_cluster = bytearray(14396)
space_for_uros = bytearray(5000)

import gc
import micropython

from pimoroni_yukon import Yukon
from pimoroni_yukon import SLOT1 as M1
from pimoroni_yukon import SLOT2 as SERVOS
from pimoroni_yukon import SLOT3 as M2
from pimoroni_yukon import SLOT4 as M3
from pimoroni_yukon import SLOT5 as LEDS
from pimoroni_yukon import SLOT6 as M4
from pimoroni_yukon.modules import BigMotorModule, QuadServoDirectModule, LEDStripModule
from pimoroni_yukon.timing import ticks_ms, ticks_add
from time import ticks_us
from pimoroni import PID
import pimoroni_yukon.logging as yukon_logging
from servo import ServoCluster, Calibration
from machine import Pin
from nefive_msgs import Status, Motors, Imu
from blinking import Blinking
import math
from RollingAverage import RollingAverage
from odometry import Odometry
import uros
from kinematics import Kinematics
from umatrix import * 

log_level = yukon_logging.LOG_INFO
# Frees up memory in case new code is run without a reset
gc.collect()

"""
All the Yukon functionality to control NE-Five
"""

class NEFive:
    global space_for_servo_cluster

    yukon = Yukon(logging_level=log_level)                         # Create a new Yukon object
       
    # Constants
    GEAR_RATIO = 30                         # The gear ratio of the motor
    ENCODER_CPR = 64                        # The number of counts a single encoder shaft revolution will produce
    MOTOR_CPR = GEAR_RATIO * ENCODER_CPR    # The number of counts a single motor shaft revolution will produce

    # PIO and State Machine Usage
    MOTOR_ENCODER_PIO         = 0                 # The PIO system to use (0 or 1) for the motor's encoder
    MOTOR_ENCODER_SM          = 0                 # Four encoders so SM 0..3 are used

    STRIP_PIO           = 1                 # Each strip uses 1 SM, we hopefully have three available
    STRIP_SM            = 0

    SERVO_CLUSTER_PIO   = 1
    SERVO_CLUSTER_SM    = 1

    STRIP_TYPE     = LEDStripModule.NEOPIXEL
    LEDS_PER_STRIP = 32
    BRIGHTNESS     = 0.75

    # PID values
    VEL_KP = 30.0                           # Velocity proportional (P) gain
    VEL_KI = 0.0                            # Velocity integral (I) gain
    VEL_KD = 0.4                            # Velocity derivative (D) gain

    MOTOR_SLOTS = [ M2, M1, M3, M4 ]
    MOTOR_UPDATES     = 30                           # How many times to update the motor per second
    MOTOR_UPDATE_RATE = 1 / MOTOR_UPDATES

    blinking = Blinking()
    total_duration = 0

    # 0: Front Left, 1: Front Right, 2: Rear Right, 3: Rear Left
    # Motor Directions: 0: Normal, 1: Reversed
    motor_directions = [1, 1, 1, 0]
    encoder_directions = [0, 1, 0, 0]

    # These are in radians per second, generated using the motor_profiler.py script
    motor_speed_min_max = [[-49.7635, 51.17347], [-50.59612, 48.28781], [-51.14614, 53.7041], [-53.28336, 50.83044]]
    motor_speed_scales = []

    for min_max in motor_speed_min_max:
        motor_speed_scales.append((math.fabs(min_max[0]) + min_max[1])/2)

    motors          = [None, None, None, None]
    motor_requested_speeds = [0, 0, 0, 0]
    encoders        = [None, None, None, None]
    vel_pids        = [None, None, None, None]  
    led_strip       = None
    servos          = None

    uros = None

    servo_module = None
    serial_port = None

    # Variables
    phase_offset = 0                        # The offset used to animate the motor

    time = [-1, -1]                     # Time, in [seconds, nsec since seconds]       
    time_updated = None                 # Time in millis() that ROS time was last updated       
    time_mode = 0

    power_update_interval = 1000            # How often to update the power status
    voltage_in_avg = RollingAverage(20)     # Rolling average of the input voltage

    mouth_leds_on = False
    motor_speeds = None
    
    durations = RollingAverage(20)
    

    def update_odom(self, duration):
        self.motor_speeds = matrix([self.encoders[0].radians_per_second], 
                              [self.encoders[1].radians_per_second], 
                              [self.encoders[2].radians_per_second], 
                              [self.encoders[3].radians_per_second])
        latest_odom = self.kinematics.odometry(self.motor_speeds, duration)
        
        self.odom += latest_odom
        self.total_duration += duration
        self.durations.add(duration)

        
    def motor_callback(self, msg):
        self.motors[0].motor.speed(msg.motor1)
        self.vel_pids[0].setpoint = msg.motor1
        self.motors[1].motor.speed(msg.motor2)
        self.vel_pids[1].setpoint = msg.motor2
        self.motors[2].motor.speed(msg.motor3)
        self.vel_pids[2].setpoint = msg.motor3
        self.motors[3].motor.speed(msg.motor4)
        self.vel_pids[3].setpoint = msg.motor4
        
    encoder_counts = 0
    
    def update_encoders(self):
        self.encoder_counts += 1
        for i in range(4):
            self.encoders[i] = self.motors[i].encoder.capture()
               

    def update_motor_speeds(self):
        self.update_encoders()

        for i in range(4):
            accel = self.vel_pids[i].calculate(self.encoders[i].radians_per_second)
            self.motors[i].motor.speed(self.motors[i].motor.speed() + (accel * self.MOTOR_UPDATE_RATE))
            

    def __init__(self):
        global space_for_servo_cluster, space_for_uros
        # Configure the motor and encoder modules
        for i in range(4):
            print("Registering motor: ", i)
            self.motors[i] = BigMotorModule(encoder_pio=self.MOTOR_ENCODER_PIO,    # Create a BigMotorModule object, with details of the encoder
                            encoder_sm=i,
                            counts_per_rev=self.MOTOR_CPR,
                            init_motor=True,
                            init_encoder=True)
            self.vel_pids[i] = PID(self.VEL_KP, self.VEL_KI, self.VEL_KD, self.MOTOR_UPDATE_RATE)  # Create a PID object for velocity control
            self.yukon.register_with_slot(self.motors[i], self.MOTOR_SLOTS[i])      # Register the BigMotorModule object with the slot

        # Configure the servo module, double duty as UART
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
            self.motors[i].motor.speed_scale(self.motor_speed_scales[i])
        
        # Enable the LED strip module
        self.led_strip.enable()
        
        micropython.mem_info()
        # Enable the servo module, start with UART
        space_for_uros = None
        gc.collect()
        self.node=uros.NodeHandle(1, 115200, tx=SERVOS.FAST1, rx=SERVOS.FAST2) 
        self.status = Status()
        self.node.subscribe('ne_five/motors', Motors, self.motor_callback, buffer_size=256)

        self.servo_pins = [SERVOS.FAST3, SERVOS.FAST4]

        print("Initialising servo cluster")
        
        micropython.mem_info()
        print("Clearing space for servo cluster")
        space_for_servo_cluster = None
        gc.collect()
        micropython.mem_info()

        self.servos = ServoCluster(1, 1, self.servo_pins)
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
        

    # Currently sending dummy data as not yet implmented
    def send_imu_data(self):
        data = [0.1, 0.2, 0.3,
                1.1, 1.2, 1.3,
                2.1, 2.2, 2.3]  


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
        min_voltage = 10.25
        max_voltage = 16.3
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

        print(f"Status: {self.status.seconds}, {self.status.nsec}, {self.status.voltage_in}, {self.status.voltage_out}, {self.status.current}, {self.status.temperature}")
        print(f"Odom: [{self.odom}], update_counts: {self.encoder_counts}")
        print(f"Motors: {self.encoders[0].radians_per_second}, {self.encoders[1].radians_per_second}, {self.encoders[2].radians_per_second}, {self.encoders[3].radians_per_second}")
        self.node.publish("/status", self.status, buffer_size=256)
       
    kinematics = Kinematics() 
    # Create kinematics objects at start point
    velocities = kinematics.motor_speeds(0, 0, 0)
    odom = kinematics.odometry(velocities, 0)
    
    
    def run(self):
        # Wrap the code in a try block, to catch any exceptions (including KeyboardInterrupt)
        try:            
            self.last_publish_motors = ticks_ms()
            self.publish_motors_interval = 50

            self.last_publish_imu = ticks_ms()
            self.publish_imu_interval = 10

            delta_time, loop_ended = None, None
            
            # Set the mouth to green
            for led in range(24, 32):
                self.led_strip.strip.set_rgb(led, 0, 75, 0)
            self.led_strip.strip.update()

            odom_updated = ticks_ms()
            odom_update_interval = 10
            
            battery_updated = ticks_ms()
            battery_update_interval = 1000

            motor_update_interval = 1000 / self.MOTOR_UPDATES
            last_motor_update = ticks_ms()

            # Set the torso actuator to upright
            self.servos.value(0, 0.75)

            while True:
                self.current_time = ticks_ms()                   # Record the start time of the program loop
                
                if loop_ended is not None:
                    delta_time = (ticks_us() - loop_ended) / 1000
                    
                self.update_eye_leds()                
                if self.current_time > battery_updated + battery_update_interval:
                    self.update_mouth_leds()
                    self.report_status()
                    
                    battery_updated = self.current_time
                
                if self.current_time > last_motor_update + motor_update_interval:
                    self.update_motor_speeds()
                    # print(f"delta_time: {delta_time:.9f}")
                    self.update_odom((self.current_time - last_motor_update) / 1000)
                    last_motor_update = self.current_time

                self.servos.value(0, 0.75)

                if delta_time is not None:
                    self.update_encoders()        
                    # self.update_odom(delta_time)
                
                # Monitor sensors until the current time is reached, recording the min, max, and average for each
                # This approach accounts for the updates taking a non-zero amount of time to complete
                self.yukon.monitor_until_ms(ticks_add(self.current_time, 5))
                loop_ended = ticks_us()
        finally:
            # Put the board back into a safe state, regardless of how the program may have ended
            self.node.shutdown()
            self.yukon.reset()
