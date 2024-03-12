import math

class Odometry:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

    def update_position(self, vx, vy, vz, dt):
        # Update orientation
        self.theta += vz * dt
        self.theta = self.theta % (2 * math.pi)  # Keep theta within [0, 2pi]

        # Update position
        self.x += vx * dt * math.cos(self.theta)
        self.y += vy * dt * math.sin(self.theta)