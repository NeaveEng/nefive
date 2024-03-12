from ugenpy.message import MessageGenerator
msgs = ['nefive_msgs/Imu.msg', 'nefive_msgs/Motors.msg', 'nefive_msgs/Status.msg']
for msg in msgs:
    msg=MessageGenerator(msg)
    msg.create_message()