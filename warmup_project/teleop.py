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
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist,'cmd_vel', 10)
    
    def direction(self, key):
        vel = Twist()
        if key == 'a' :
            vel.linear.x = 0.1
            vel.angular.z = -0.1
        elif key =='w' :
            vel.linear.x = 0.1
            vel.angular.z = 0.0
        elif key == 'd' :
            vel.linear.x = 0.1
            vel.angular.z = 0.1
        else:
            vel.linear.x = 0.0
            vel.angular.z = 0.0
        return vel

    def drive(self, vel):    
        self.vel_pub.publish(vel)
        
    def getKey(self, settings):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


    def run_loop (self):
        settings = termios.tcgetattr(sys.stdin)
        key = None
        key = self.getKey(settings)
        print(key)
        vel = self.direction(key)
        self.drive(vel)

def main(args = None):
    rclpy.init(args=args) 
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
