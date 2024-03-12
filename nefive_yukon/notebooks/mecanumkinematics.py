import array 
import numpy as np

class MecanumKinematics:     
    def __init__(self):
        print("MecanumKinematics")
        self.fkm = self.matrix_scalar(self.fkm, 1/self.r)
        # self.ikm = self.matrix_scalar(self.ikm, self.r/4)

    print("MecanumKinematics")
    # Long lived variable to reduce memory allocations
    input_velocities = [[0, 0, 0]]
        
    def matrix_multiply(self, a, b):
        result = [[0 for _ in range(len(b[0]))] for _ in range(len(a))]

        for i in range(len(a)):
            for j in range(len(b[0])):
                for k in range(len(b)):
                    result[i][j] += a[i][k] * b[k][j]

        return result

    # This works on a shallow copy of A so the original is modified
    def matrix_scalar(self, a, b):
        for i in range(len(a)):
            for j in range(len(a[0])):
                a[i][j] = a[i][j] * b
        
        return a

    def calc_motor_speeds(self, vx, vy, wz):
        self.input_velocities[0][0] = wz
        self.input_velocities[0][1] = vx
        self.input_velocities[0][2] = vy

        # u = np.multiply(self.fkm, self.input_velocities)
        u = self.matrix_multiply(self.fkm, self.input_velocities)
        
        u = [u[0][0], u[1][0], u[2][0], u[3][0]]
        return u
    
    
    def calc_velocities(self, motor_speeds):
        print(self.ikm)
        print(motor_speeds)
        
        u = self.matrix_multiply(self.ikm, motor_speeds)
        # u = [u[0][0], u[1][0], u[2][0], u[3][0]]
        return u
    

    WHEEL_RADIUS = 0.030
    TRACK_WIDTH  = 0.195 
    TRACK_LENGTH = 0.150

    l = TRACK_LENGTH / 2
    w = TRACK_WIDTH / 2
    r = WHEEL_RADIUS

    front_left  = [-l-w, 1,  1]
    front_right = [ l+w, 1, -1]
    rear_left   = [-l-w, 1, -1]
    rear_right  = [ l+w, 1,  1]
    
    kinematic_model = np.array([front_left, 
                            front_right,
                            rear_left,
                            rear_right]) / r 
    
    # From page 13: https://cdn.intechopen.com/pdfs/465/InTechOmnidirectional_mobile_robot_design_and_implementation.pdf
    def steering(self, vx, vy, wz):
        input_array = np.array([wz,vx,vy])    
        input_array.shape = (3, 1)

        u = np.dot(self.kinematic_model, input_array)
        # u = np.dot(self.fkm, input_array)
        return u

    fwk_front_left  = array.array('f', [-l-w, 1,  1])
    fwk_front_right = array.array('f', [ l+w, 1, -1])
    fwk_rear_left   = array.array('f', [-l-w, 1, -1])
    fwk_rear_right  = array.array('f', [ l+w, 1,  1])

    fkm = [fwk_front_left, 
           fwk_front_right,
           fwk_rear_left,
           fwk_rear_right] 

    ikm = [array.array('f', [  fkm[0][1],   fkm[1][1],   fkm[2][1],   fkm[3][1]]),
           array.array('f', [  fkm[0][2],   fkm[1][2],   fkm[2][2],   fkm[3][2]]),
           array.array('f', [1/fkm[0][0], 1/fkm[1][0], 1/fkm[2][0], 1/fkm[3][0]])]

