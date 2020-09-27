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
    def __init__(self, standalone=True):
        if standalone:
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
        while not rospy.is_shutdown():
            msg = self.get_pub_message()
            if not msg:
                break
            self.pub.publish(msg)

        print('terminated.')

    def get_pub_message(self):
        self.key = self.getKey()
        if self.key == '\x03':
            return None
        elif self.key == 'w':
            return self.forward
        elif self.key == 'a':
            return self.left
        elif self.key == 's':
            return self.back
        elif self.key == 'd':
            return self.right
        else:
            return self.stop

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
