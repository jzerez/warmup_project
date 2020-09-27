#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from teleop import Teleoperator
import numpy as np

class WallFollower():

    def __init__(self):
        self.node = rospy.init_node('WallFollower')
        self.follow_dist = 0.8
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.parse_lidar)


        self.empty_scans = 0
        self.k = 0.7
        self.steer = 0
        self.rate = rospy.Rate(3)

    def run(self):

        while not rospy.is_shutdown():
            msg = Twist(Vector3(0.2, 0, 0), Vector3(0,0,0))
            if not np.isfinite(self.steer):
                self.steer = 0

            print('steer command: ', self.steer)


            msg.angular = Vector3(0,0,self.steer + msg.angular.z)
            self.pub.publish(msg)
            self.rate.sleep()

    def parse_lidar(self, data):
        rs = np.array(data.ranges[:-1])
        thetas = np.linspace(0, 2*np.pi, 361)[:-1]

        # reject lidar scans that are completely empty
        if not sum(np.isfinite(rs)):
            self.empty_scans += 1
            return

        # Find the smallest 15 radius values and compare to desired wall follwing distance
        num_rs = 15
        smallest_rs = rs[np.argpartition(rs, num_rs)[:num_rs]]

        # if we're far enough from the wall, don't suggest a steer command
        if np.mean(smallest_rs) > self.follow_dist:
            self.steer = 0
            return

        # Focus on wall following on the right hand side of the neato
        center_angle = 270
        # scan +/- 45 degrees from 270
        range = 45

        # Sector 1 radii:
        # Non-inf, non-nan values only
        # from 270 degrees to 315 degrees
        s1 = self.filter_infs(rs[center_angle:center_angle+range])


        # Sector 2 radii:
        # Non-inf, non-nan values only
        # from 270 degrees to 225 degrees
        s2 = self.filter_infs(rs[center_angle-range:center_angle])

        # proportional control. Compare average radius of each sector to get error. scale by k.
        self.steer = (np.mean(s2) - np.mean(s1))*self.k

    def polar_to_cartesian(self, thetas, rs):
        xs = np.cos(thetas) * rs
        ys = np.sin(thetas) * rs
        return xs,ys

    def filter_infs(self, arr):
        """
        eliminate any inf and nan values in a given array, arr
        """
        return arr[np.isfinite(arr)]

if __name__ == "__main__":
    a = WallFollower()
    a.run()
