import rclpy  # convenience python library for interacting with ROS2
import math
from rclpy.node import Node  # generic Node class for interacting with ROS2
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist  # ROS package call for a Twist type message format
from neato2_interfaces.msg import Bump
from sensor_msgs.msg import LaserScan

class WallFollowingNode(Node):
    def __init__(self):
        super().__init__('wall_following')

        # Timer to regularly run the loop
        self.create_timer(0.1, self.run_loop)

        # Subscription to the LaserScan topic
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)

        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize variables
        self.distance_to_wall = None
        self.ranges = None
        self.scan_angle_1 = [220, 260] # 240 degrees
        self.scan_angle_2 = [280, 320] # 300 degrees
        self.m = 0
        self.b = 0
        self.theta = 0
        self.r = 0
        self.good_angle1 = None
        self.good_angle2 = None 
        self.good_scan1 = None
        self.good_scan2 = None

        # Constants for control
        self.Kp_distance = 1.0
        self.target_distance = 1.2
        self.closest_distance = self.target_distance
        self.linear_vel = 0.2
        self.angular_vel_limit = 1.0
    
    def check_r(self, r):
        if r != 0 and not math.isinf(r):
            return True
        else: 
            return False
        
    def process_scan(self, msg: LaserScan):
        self.ranges = msg.ranges
        for i in range(self.scan_angle_1[0], self.scan_angle_1[1]):
            if self.check_r(self.ranges[i])== True:
                self.good_scan1 = self.ranges[i]
                self.good_angle1 = i
                break
        for i in range(self.scan_angle_2[0], self.scan_angle_2[1]):
            if self.check_r(self.ranges[i])== True:
                self.good_scan2 = self.ranges[i]
                self.good_angle2 = i
                break

        if self.good_angle1 != None:
        # Calculate the distance to the wall using specific angles (240° and 300°)
            x1 = self.good_scan1* math.cos(math.radians(270 - self.good_angle1))
            y1 = -1*self.good_scan1* math.sin(math.radians(270 - self.good_angle1))
            x2 = self.good_scan2* math.cos(math.radians(self.good_angle2 - 270))
            y2 = self.good_scan2 * math.sin(math.radians(self.good_angle2- 270))

            # Calculate slope and intercept of the wall
            if x1 != 0 and y1 != 0 and x2 != 0 and y2 != 0 and x1 - x2 != 0:
                self.m = (y1 - y2) / (x1 - x2)
                self.b = y1 - self.m * x1

            # If the slope isn't zero, calculate the angle relative to the wall
            if self.m != 0:
                self.theta = math.degrees(math.atan(1 / self.m))

            # Calculate the closest distance to the wall
            self.closest_distance = abs(self.b) / math.sqrt(1 + self.m * self.m)

    def compute_velocity_command(self):
        msg = Twist()

        # If no wall data, move forward at default speed
        if self.ranges is None:
            msg.linear.x = 0.1
            msg.angular.z = 0.0
            print("no wall seen")
        else:
            # Distance error for proportional control
            error_distance = self.target_distance - self.closest_distance

            # Target angular velocity based on distance error
            angular_vel = error_distance * self.Kp_distance

            # Set velocity based on control laws
            msg.linear.x = self.linear_vel
            print(f"linear vel: {self.linear_vel}")
            msg.angular.z = angular_vel
            print(f"angular vel: {angular_vel}")
        return msg

    def run_loop(self):
        # Calculate the velocity command and publish it
        msg = self.compute_velocity_command()
        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
