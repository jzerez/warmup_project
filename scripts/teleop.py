#!/usr/bin/env python3

# ROS dependencies
from geometry_msgs.msg import Twist, Vector3
import rospy

# Key reading dependencies
import tty
import select
import sys
import termios

class Teleoperator():
    """
    Publishes commands to the 'cmd_vel' topic in order to control the NEATO with
    the W-A-S-D keys. Hold down a key to move.
    """
    def __init__(self):
        rospy.init_node('teleoperator')
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Standard movement commands to publish
        self.forward = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
        self.back = Twist(Vector3(-0.5,0,0), Vector3(0,0,0))
        self.stop = Twist(Vector3(0,0,0), Vector3(0,0,0))
        self.left = Twist(Vector3(0,0,0), Vector3(0,0,0.5))
        self.right = Twist(Vector3(0,0,0), Vector3(0,0,-0.5))


    def run(self):
        """
        continually listens for key presses and sends the appropriate commands
        """
        while not rospy.is_shutdown() and self.key != '\x03':
            self.key = self.getKey()

            if self.key == 'w':
                self.pub.publish(self.forward)
            elif self.key == 'a':
                self.pub.publish(self.left)
            elif self.key == 's':
                self.pub.publish(self.back)
            elif self.key == 'd':
                self.pub.publish(self.right)
            else:
                self.pub.publish(self.stop)
        print('terminated.')


    def getKey(self):
        """
        Listens for key presses. Returns the char of the key pressed
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

if __name__ == "__main__":
    controller = Teleoperator()
    controller.run()
