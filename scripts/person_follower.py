#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from MarkerPub import MarkerPub
import numpy as np
import pdb
import matplotlib.pyplot as plt

class PersonFollower():
    def __init__(self):
        self.node = rospy.init_node('PersonFollower')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.parse_lidar)


        self.pub.publish(Twist(Vector3(0.1,0,0), Vector3(0,0,0)))
        rospy.sleep(1)
        print('done')
        self.target_radius = 0.47
        self.target_dist = self.target_radius + 0.3
        self.points = np.array([])
        self.marker_pub = MarkerPub(standalone=False)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            center, radius = self.circle_ransac(self.points, plot_mode=False)
            if center is None:
                r.sleep()
                print('no human found')
                continue
            self.marker_pub.marker_ping(Marker.ADD, center[0], center[1])

            dist_to_target = np.linalg.norm(center)
            vel_command = (dist_to_target - self.target_dist) * 0.3
            turn_command = np.arctan2(center[1], center[0]) * 0.8

            vel_lim = 0.7
            turn_lim = 1.3
            if abs(vel_command) > vel_lim:
                vel_command = np.sign(vel_command) * vel_lim

            if abs(turn_command) > turn_lim:
                turn_command = np.sign(turn_command) * turn_lim

            translate = Vector3(vel_command, 0, 0)
            rotate = Vector3(0,0,turn_command)

            move_msg = Twist(translate, rotate)
            print(move_msg)
            self.pub.publish(move_msg)
            r.sleep()


    def parse_lidar(self, data):
        rs = np.array(data.ranges[:-1])
        thetas = np.linspace(0, 2*np.pi, 361)[:-1]

        non_infs = np.isfinite(rs)
        rs = rs[non_infs]
        thetas = thetas[non_infs]

        if rs.size == 0:
            print('empty rs')
            self.points = np.array([])
            return

        xs,ys = self.polar_to_cartesian(rs, thetas)
        self.points = np.vstack((xs,ys)).T


    def filter_infs(self, arr):
        """
        eliminate any inf and nan values in a given array, arr
        """
        return arr[np.isfinite(arr)]

    def polar_to_cartesian(self, rs, thetas):
        xs = np.cos(thetas) * rs
        ys = np.sin(thetas) * rs
        return xs,ys

    def circle_ransac(self, points, plot_mode=False):
        """
        Take the points of a lidar scan and perform RANSAC to find the circle
        """
        if points.size == 0:
            print('no points')
            return None, None
        most_points = 0
        best_center = None
        best_radius = None

        for index, sample in enumerate(points):
            # If you wanted to choose random points. Probably would be better to choose 3 kind of consecutive points
            # indices = np.random.choice(points.shape[0], 3)

            # chooses 3 points 5 degrees apart
            indices = [index, (index + 5)%len(points), (index + 10)%len(points)]
            chosen_points = points[indices]

            center, radius = self.fit_circle(chosen_points[0], chosen_points[1], chosen_points[2])

            member_points = len(self.find_membership(points, center, radius))

            if member_points > most_points and abs(radius - self.target_radius) / self.target_radius < 0.1:
                most_points = member_points
                best_center = center
                best_radius = radius

        if plot_mode:
            print(points[:, 0])

            plt.figure()
            plt.scatter(points[:, 0], points[:, 1], c='b', marker='o')
            plt.scatter(best_center[0], best_center[1], c='r', marker='x')
            plt.show()

        print('RADIUS IS: ', best_radius)
        return best_center, best_radius

    def fit_circle(self, p1, p2, p3):
        """
        Given 3 (x,y) tuples, return the center and the radius of the circle that they define

        This code is from https://www.w3resource.com/python-exercises/basic/python-basic-1-exercise-39.php
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        c = (x1-x2)**2 + (y1-y2)**2
        a = (x2-x3)**2 + (y2-y3)**2
        b = (x3-x1)**2 + (y3-y1)**2
        s = 2*(a*b + b*c + c*a) - (a*a + b*b + c*c)
        px = (a*(b+c-a)*x1 + b*(c+a-b)*x2 + c*(a+b-c)*x3) / s
        py = (a*(b+c-a)*y1 + b*(c+a-b)*y2 + c*(a+b-c)*y3) / s
        ar = a**0.5
        br = b**0.5
        cr = c**0.5
        radius = ar*br*cr / ((ar+br+cr)*(-ar+br+cr)*(ar-br+cr)*(ar+br-cr))**0.5
        return np.array([px,py]), radius

    def find_membership(self, points, center, radius, tolerance=0.05):
        """
        Given a circle with a given center and radius, find the points in a given lidar scan that are within a certain tolerable distance to the edge of the circle
        """
        members = []
        for index, point in enumerate(points):
            d = np.linalg.norm(point - center)
            if abs(d-radius) < tolerance:
                members.append(index)
        return members

if __name__ == "__main__":
    person_follower = PersonFollower()
    person_follower.run()
