#!/usr/bin/env python3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
import rospy

class MarkerPub:
    def __init__(self, standalone=True):
        if standalone:
            rospy.init_node('Goal Marker')
        self.publisher = rospy.Publisher('/test_sphere', Marker, queue_size=10)


    def marker_ping(self, operation, x, y):
        header = Header(stamp = rospy.Time.now(), frame_id="base_link")
        pose = Pose(Point(x,y,0), Quaternion(0,0,0,0))
        scale = Vector3(0.5,0.5,0.5)
        color = ColorRGBA(0, 1, 0.5, 0.5)

        marker = Marker(
            header=header,
            ns="test_message",
            id=0,
            type=Marker.SPHERE,
            action=operation,
            pose=pose,
            scale=scale,
            color=color
        )
        self.publisher.publish(marker)


    def run(self):
        rate = rospy.Rate(1)
        # print(self.rate)
        while not rospy.is_shutdown():
            self.marker_ping(Marker.ADD, 1, 2)
            print('ping')
            rate.sleep()
            self.marker_ping(Marker.DELETE, 1, 2)
            rate.sleep()
        print('rospy shutdown. Stopping pings.')

if __name__ == "__main__":
    apple = MarkerPub()
    apple.run()
