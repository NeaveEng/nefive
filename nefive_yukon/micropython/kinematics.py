from umatrix import *

class Kinematics:
    WHEEL_RADIUS = 0.030
    TRACK_WIDTH  = 0.188 
    TRACK_LENGTH = 0.147

    l = TRACK_LENGTH / 2
    w = TRACK_WIDTH / 2
    r = WHEEL_RADIUS

    k_const = (l+w)
    kinematic_model = matrix(
        [ 1,  1, -k_const], 
        [ 1, -1,  k_const], 
        [ 1, -1, -k_const], 
        [ 1,  1,  k_const]) * (1/r)
    
    inverse_kinematic_model = matrix(
        [          1,           1,             1,           1],
        [          1,          -1,            -1,           1],
        [-(1/k_const), (1/k_const),  -(1/k_const), (1/k_const)]) * (r/4)

    # NOTE:
    # The kinematic model assumes that the motors are laid out as follows:
    #  [3]     [1]
    #     ---> x
    #  [4]     [2]
    #
    # NE-Five is wired up as follows:
    #  [3]     [2]
    #     ---> x
    #  [4]     [1]
    #
    # As a result, we need to swap input elements around before using the models

    def motor_speeds(self, vx, vy, wz):
        input_array = matrix([vx],[vy],[wz])    
        return self.kinematic_model * input_array
        

    def velocities(self, speeds):     
        return self.inverse_kinematic_model * speeds
        
        
    def odometry(self, wheel_speeds, duration):
        u = self.inverse_kinematic_model * wheel_speeds    
        return u * duration   
        