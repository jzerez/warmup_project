#!/usr/bin/env python3
import rospy
from person_follower import PersonFollower
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import numpy as np
import pdb

import tf2_ros


class ObstacleAvoider(PersonFollower):
    def __init__(self):
        self.node = init_node('obstacle_avoider')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.parse_lidar)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.person_radius = 0.5
        self.target_dist = self.person_radius + 0.3
        self.goal_coor = np.array([[6,6]]).T
        self.points = np.array([])

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                transformation = self.tf_buffer.lookup_transformation('base_footprint', 'odom', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            center, radius = self.circle_ransac(self.points, plot_mode=False)
            if center is None:
                rate.sleep()
                continue

            target_inds = set(self.find_membership(self.points, center, radius))
            repulsive_inds = [i for i in range(self.points) if i not in target_inds]
            repulsive_points = self.points[repulsive_inds]

            repulsion_force = self.calc_repulsion(repulsive_points)

            dist_to_target = np.linalg.norm(center)

            rate.sleep()

    def limit_input(self, input, limit):
        if input < limit:
            return input
        else:
            return limit

    def calc_repulsion(self, points):
        dists = np.linalg.norm(points, axis=1)
        mags = np.array([[k/dist**2 for dist in dists]])
        return numpy.matmul(mags, dists)
