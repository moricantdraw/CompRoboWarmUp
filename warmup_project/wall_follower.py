import rclpy  # convenience python library for interacting with ROS2
from rclpy.node import Node  # generic Node class for interacting with ROS2
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult 
from rclpy.qos import qos_profile_sensor_data
from neato2_interfaces.msg import Bump  # local package call for a Bump type message format
from geometry_msgs.msg import Twist  # ROS package call for a Twist type message format
from sensor_msgs.msg import LaserScan 

class WallFollowingNode(Node):
    def __init__(self):
        super().__init__('wall_following')
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan,'scan', self.process_scan, qos_profile = qos_profile_sensor_data)
        self.vel_pub = self.create_publisher (Twist, 'cmd_vel', 10)
        self.distance_to_wall = None
        #consant to apply proportional error signal 
        self.Kp = 0.3
        self.wall_goal = self.get_parameter('wall goal').value
    
    def process_scan(self, msg: LaserScan):
        if msg.ranges[0] != 0.0:
            self.distance_to_wall = msg.ranges[0]
        
    
    def compute_velocity_command(self):
        msg = Twist()
        
        if self.distance_to_wall is None:
            msg.linear.x = 0.1  # Move forward if no wall detected
        else:
            error = self.compute_error(self.distance_to_wall, self.wall_goal)
            msg.linear.x = self.Kp * error  # Proportional control based on distance error
        
        return msg
    
    # Compute error between actual and target distance
    def compute_error(self, actual_distance: float, wall_goal: float) -> float:
        return actual_distance - wall_goal

    # Publish velocity message
    def publish_velocity(self, msg: Twist):
        self.vel_pub.publish(msg)

    def run_loop(self):
        msg = self.compute_velocity_command()
        self.publish_velocity(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()