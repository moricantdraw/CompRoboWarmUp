import rclpy  # convenience python library for interacting with ROS2
from rclpy.node import Node  # generic Node class for interacting with ROS2
from neato2_interfaces.msg import Bump  # local package call for a Bump type message format
from geometry_msgs.msg import Twist  # ROS package call for a Twist type message format

import tty
import select
import sys
import termios

class TeleopNode(Node):

    def __init__(self):
        super().__init__("teleop_node")
        self.publisher = self.create_publisher(Twist,'cmd_vel', 10)
    
    def drive():
        vel = Twist()
        if key == 'a' :
            vel.linear.x = 0.1
            vel.angular.z = 
        elif key =='w' :
            vel.linear.x = 0.1
            vel.angular.z =
        elif key == 'd' :
            vel.linear.x = 0.1
            vel.angular.z = 
        else:
            vel.linear.x = 0.0
            vel.angular.z = 0.0
        
        self.publisher.publish(vel)
        
    def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    settings = termios.tcgetattr(sys.stdin)
    key = None

    while key != '\x03':
        key = getKey()
        print(key)


def main():
    rclpy.init(args=args) 
    node = TeleopNode()


if __name__ == '__main__':
    main()
