#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3, PointStamped, Point
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import numpy as np
import pdb
from person_follower import PersonFollower
from MarkerPub import MarkerPub
import tf2_ros
import tf2_geometry_msgs


class ObstacleAvoider(PersonFollower):
    def __init__(self):
        self.node = rospy.init_node('obstacle_avoider')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.parse_lidar)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.goal_coor = np.array([[5,5]])
        goal_point = Point(self.goal_coor[0][0], self.goal_coor[0][1], 0)
        header = Header(stamp=rospy.Time())

        self.goal_coor_point = PointStamped(header, goal_point)
        self.marker_pub = MarkerPub(standalone=False)

        self.points = np.array([])

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                trans = self.tf_buffer.lookup_transform('base_footprint', 'odom', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print('tf2 error')
                print(e)
                rate.sleep()
                continue

            if self.points.size == 0:
                repulsion_vec = np.zeros([1,2])
            else:
                repulsion_vec = self.calc_repulsion(self.points)

            goal_trans = tf2_geometry_msgs.do_transform_point(self.goal_coor_point, trans).point

            goal_dist = np.sqrt(goal_trans.x**2 + goal_trans.y**2)
            attraction_vec = np.array([[goal_trans.x, goal_trans.y]]) * 2
            print(np.linalg.norm(repulsion_vec) / np.linalg.norm(attraction_vec))
            goal_vec = attraction_vec + repulsion_vec

            if goal_dist > 0.5:
                vel = 0.2
            else:
                print('DONE')
                self.pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, ang)))
                break

            ang = 1.3 * np.arctan2(goal_vec[0][1], goal_vec[0][0])

            self.pub.publish(Twist(Vector3(vel, 0, 0), Vector3(0, 0, ang)))
            self.marker_pub.marker_ping(Marker.ADD, goal_trans.x, goal_trans.y)
            rate.sleep()

    def calc_repulsion(self, points):
        k = -0.05
        dists = np.linalg.norm(points, axis=1)
        mags = np.array([[k/(dist**1.7) for dist in dists]])
        res = np.matmul(mags, points)
        return res

if __name__ == "__main__":
    obstacle_avoider = ObstacleAvoider()
    obstacle_avoider.run()
