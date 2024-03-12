from pimoroni_yukon import Yukon
from pimoroni_yukon.timing import ticks_ms, ticks_add
from time import ticks_us
from kinematics import Kinematics

"""
All the Yukon functionality to control NE-Five
"""

kinematics = Kinematics() 
# Create kinematics objects at start point
velocities = kinematics.motor_speeds(1, 0, 0)
odom = kinematics.odometry(velocities, 0)
interval = 1000

yukon = Yukon()                                 
try:            
    delta_time, loop_ended = None, None
    current_time = ticks_ms()
    last_thing = current_time
    
    while True:
        current_time = ticks_ms()                   # Record the start time of the program loop
        
        if loop_ended is not None:
            delta_time = ticks_us() - loop_ended
            
        if delta_time is not None:
            # print(f"{velocities}")
            odom += kinematics.odometry(velocities, delta_time / 1000000)
            # print(delta_time, delta_time / 1000000)
        
        if current_time > last_thing + interval:
            print(f"{delta_time / 1000000}")
            print(f"{odom}")
            last_thing = current_time
        
        # Monitor sensors until the current time is reached, recording the min, max, and average for each
        # This approach accounts for the updates taking a non-zero amount of time to complete
        yukon.monitor_until_ms(ticks_add(current_time, 5))
        loop_ended = ticks_us()
finally:
    yukon.reset()
