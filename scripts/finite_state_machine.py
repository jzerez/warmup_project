#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3, PointStamped, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from MarkerPub import MarkerPub
from obstacle_avoidance import ObstacleAvoider
import numpy as np
import pdb
import matplotlib.pyplot as plt

class SmartFollower(ObstacleAvoider):
    """
    Tries to identify a human target. If it can, it follows the human.
    Otherwise, navigate to set goal in the global frame while avoiding obstacles
    """
    def __init__(self):
        rospy.init_node('ObstacleAvoider')
        super().__init__(standalone=False)

        # radius for human target
        self.target_radius = 0.47
        # follow distance for human target
        self.target_dist = self.target_radius + 0.3
        self.state = 0
        self.r = rospy.Rate(10)
    def run(self):
        while not rospy.is_shutdown():
            center, radius = self.circle_ransac(self.points, plot_mode=False)

            # State machine:
            # If no human target is found:
            if center is None:
                print('GLOBAL TARGET')
                self.state = 0
                self.set_goal(np.array([[5,5]]))
                goal_trans = self.transform_goal()


                if not goal_trans:
                    self.r.sleep()
                    continue


                self.marker_pub.marker_ping(Marker.ADD, goal_trans.x, goal_trans.y)
                # avoid obstacles and navigate towards global target
                move_msg = self.avoid_obstacle_movement(goal_trans)
                self.pub.publish(move_msg)

                self.r.sleep()
            # Otherwise
            else:
                print('HUMAN TARGET')
                self.state = 1
                self.marker_pub.marker_ping(Marker.ADD, center[0], center[1])
                self.set_goal(np.expand_dims(center, 0))
                # navigate towards person
                move_msg = self.person_follow_movement(center)
                self.pub.publish(move_msg)
                self.r.sleep()


    def set_goal(self, goal):
        """
        sets the neato's goal (honestly probably not needed)
        """
        self.goal_coor = goal
        goal_point = Point(self.goal_coor[0][0], self.goal_coor[0][1], 0)
        header = Header(stamp=rospy.Time())
        self.goal_coor_point = PointStamped(header, goal_point)

if __name__ == "__main__":
    a = SmartFollower()
    a.run()
