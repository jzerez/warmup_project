#!/usr/bin/env python3
import rospy
from geometry_msgs import Twist, Vector3

node = rospy.init_node("Sqaure-driver")
pub = rospy.Publisher('cmd_vel', Twist, 10)

forward = Twist(Vector3(0.5, 0, 0,), Vector3(0, 0, 0))
stop = Twist(Vector3(0, 0, 0,), Vector3(0, 0, 0))
turn = Twist(Vector3(0, 0, 0,), Vector3(0, 0, 0.5))

def timing_turn():
    pub.publish(forward)
    time.sleep(2.24)
    pub.publish(turn)
    time.sleep(7.75)

def odom_turn():
    pass


for step in range(4):
    timing_turn()

pub.publish(stop)
