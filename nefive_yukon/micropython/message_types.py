#   Format      C Type                  Python type     Standard size
#   b           signed char             integer         1
#   B           unsigned char           integer         1
#   h           short                   integer         2
#   H           unsigned short          integer         2
#   i           int                     integer         4
#   I           unsigned int            integer         4
#   l           long                    integer         4
#   L           unsigned long           integer         4
#   q           long long               integer         8
#   Q           unsigned long long      integer         8
#   f           float                   float           4
#   d           double                  float           8
class MessageTypes:
    message_ids = {}

    def __init__(self):
        self.message_types = [
            {
                'id': 0,
                'name': 'motors',
                'format': 'IIffff',
                'size': 2 * 4 + 4 * 4,
                'fields': ['seconds', 'nsec', 'motor1', 'motor2', 'motor3', 'motor4'],
                'units': ['seconds', 'nsec', 'RPS', 'RPS', 'RPS', 'RPS'],
                'description': 'Motor speeds'
            },
            {
                'id': 1,
                'name': 'imu',
                'size': 2 * 4 + 9 * 4,
                'format': 'IIfffffffff',
                'fields': ['seconds', 'nsec', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'mag_y', 'mag_z'],
                'units': ['seconds', 'nsec', 'm/s^2', 'm/s^2', 'm/s^2', 'rad/s', 'rad/s', 'rad/s', 'uT', 'uT', 'uT'],
                'description': 'IMU data'
            },
            {
                'id': 2,
                'name': 'time',
                'format': 'II',
                'size': 2*4,
                'fields': ['seconds', 'nsec'],
                'units': ['seconds', 'nsec'],
                'description': 'ROS Time'
            },
            {
                'id': 3,
                'name': 'actuator',
                'format': 'IIIf',
                'size': 3 * 4 + 4,
                'fields': ['seconds', 'nsec', 'servo_id', 'value'],
                'units': ['seconds', 'nsec', 'index', '-1, 1'],
                'description': 'Actuator data'
            },
            {
                'id': 4,
                'name': 'joystick',
                'format': 'IIff',
                'size': 16,
                'fields': ['seconds', 'nsec', 'x', 'y'],
                'units': ['seconds', 'nsec', 'x', 'y'],
                'description': 'Joystick data for teleop. x and y are in the range [-1, 1]'
            }
        ]

        for message_type in self.message_types:
            print(message_type)
            self.message_ids[message_type['name']] = message_type['id']