import uros
from nefive_msgs import Motors #message object ColorRGBA
from time import sleep
from machine import Pin
from pimoroni_yukon.timing import ticks_ms

node=uros.NodeHandle(1, 115200, tx=Pin(4), rx=Pin(5)) #node initialized, for tx2/rx2 and 115200 baudrate

msg=Motors() #msg object init
msg.seconds = 0
msg.nsec = 0
msg.rostime = False
msg.motor1 = 0.1
msg.motor2 = 0.2
msg.motor3 = 0.3
msg.motor4 = 0.4

while True:
    time = ticks_ms()
    msg.seconds = int(time/1000)
    msg.nsec = int((time%1000)*1000000)
    node.publish('Motors',msg) #publish data to node Colorsh
    sleep(1)