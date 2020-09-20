#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3
import time

node = rospy.init_node("Sqaure_driver")
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

forward = Twist(Vector3(0.5, 0, 0,), Vector3(0, 0, 0))
stop = Twist(Vector3(0, 0, 0,), Vector3(0, 0, 0))
turn = Twist(Vector3(0, 0, 0,), Vector3(0, 0, 0.5))

def timing_turn():
    pub.publish(stop)
    rospy.sleep(0.5)
    print('FORWARD')
    pub.publish(forward)
    rospy.sleep(2)
    print('TURN')
    pub.publish(turn)
    rospy.sleep(3.23)

def odom_turn():
    pass


for step in range(4):
    timing_turn()

pub.publish(stop)
print('STOP')
