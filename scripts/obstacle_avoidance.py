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
    """
    Moves the NEATO towards a goal while automatically avoiding obstacles
    """
    def __init__(self, standalone=True):
        if standalone:
            self.node = rospy.init_node('obstacle_avoider')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # self.parse_lidar is inherited from PersonFollower
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.parse_lidar)

        # reference frame transform tools
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Set the goal in global coordinates
        self.goal_coor = np.array([[5,5]])
        goal_point = Point(self.goal_coor[0][0], self.goal_coor[0][1], 0)
        header = Header(stamp=rospy.Time())
        self.goal_coor_point = PointStamped(header, goal_point)

        self.marker_pub = MarkerPub(standalone=False)

        self.points = np.array([])

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # transformed goal coordinates from global frame to NEATO frame
            goal_trans = self.transform_goal()

            if not goal_trans:
                rate.sleep()
                continue

            # Mark goal's position
            self.marker_pub.marker_ping(Marker.ADD, goal_trans.x, goal_trans.y)

            # Calculate movement command message
            move_msg = self.avoid_obstacle_movement(goal_trans)
            self.pub.publish(move_msg)

            if move_msg.linear.x == 0:
                break
            rate.sleep()

    def calc_repulsion(self, points):
        """
        Calculates the total repulsion force from all of the LIDAR points

        Parameters:
            points (np.array): (x,y) coords of lidar points in meters

        Returns:
            repulsion (np.array): x,y for repulsion vector
        """
        k = -0.05
        dists = np.linalg.norm(points, axis=1)
        mags = np.array([[k/(dist**1.7) for dist in dists]])
        repulsion = np.matmul(mags, points)
        return repulsion

    def avoid_obstacle_movement(self, goal):
        """
        Calculates the movement command to avoid obstacles

        Parameters:
            goal (geometry_msgs.msg.Point): goal's coords in m. Neato frame.

        Returns:
            move_msg (geometry_msgs.msg.Twist): movement command
        """
        # find repulsion vector
        if self.points.size == 0:
            repulsion_vec = np.zeros([1,2])
        else:
            repulsion_vec = self.calc_repulsion(self.points)

        # find attraction vector
        attraction_vec = np.array([[goal.x, goal.y]]) * 2
        print(np.linalg.norm(repulsion_vec) / np.linalg.norm(attraction_vec))
        goal_vec = attraction_vec + repulsion_vec

        goal_dist = np.sqrt(goal.x**2 + goal.y**2)

        # stop moving if NEATO passes threshold
        if goal_dist > 0.5:
            vel = 0.2
        else:
            print('DONE')
            return Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        # angular velocity is proportional to heading of target wrt NEATO
        ang = 1.3 * np.arctan2(goal_vec[0][1], goal_vec[0][0])
        return Twist(Vector3(vel, 0, 0), Vector3(0, 0, ang))

    def transform_goal(self):
        """
        ransforms the goal's coordinate from global frame to the neato's frame
        """
        # Obtains refrence frame transform from global to neato frame
        try:
            trans = self.tf_buffer.lookup_transform('base_footprint', 'odom', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print('tf2 error')
            print(e)

            return None

        return tf2_geometry_msgs.do_transform_point(self.goal_coor_point, trans).point


if __name__ == "__main__":
    obstacle_avoider = ObstacleAvoider()
    obstacle_avoider.run()
