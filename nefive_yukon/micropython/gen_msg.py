from ugenpy.message import MessageGenerator
msgs = ['lib/nefive_msgs/Imu.msg', 'lib/nefive_msgs/Motors.msg', 'lib/nefive_msgs/Status.msg', 'lib/nefive_msgs/Position.msg']
for msg in msgs:
    msg=MessageGenerator(msg)
    msg.create_message()