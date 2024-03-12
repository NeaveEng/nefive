from mecanum_kinematics import MecanumKinematics
mk = MecanumKinematics()

x, y, z = 1, 0, 0
speeds = mk.calc_motor_speeds(x, y, z)
print(f"Input: {x}m/s, {y}m/s, {z}rad/s: {speeds}")

velocities = mk.calc_velocities(speeds)
print(f"Input: {speeds}: {velocities}")