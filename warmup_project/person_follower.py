import rclpy  # Python library for ROS 2
import math
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult 
from rclpy.qos import qos_profile_sensor_data
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan


class NeatoPet(Node):

    def __init__(self):
        super().__init__("Neato_pet")
        self.create_timer(0.1, self.run_loop)
        self.vel_msg = Twist()
        self.person_marker = Marker()
        self.pet_marker = Marker()

        #publishers 

        #velocity comands to control robot
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # marker that represnts person 
        self.person_marker_pub = self.create_publisher(Marker, 'Person_Marker', 10)
        # marker that represnts robot 
        self.pet_marker_pub = self.create_publisher(Marker, 'Pet_Marker', 10)

        #subscribers

        # subscribes to laser scan data, calls process_scan to process it (qos to make getting realtime data better)
        self.create_subscription(LaserScan, 'scan', self.read_scan, qos_profile=qos_profile_sensor_data)
        #subscribes to bump sensor, call self.read_bump to read it
        #self.create_subscription(Bump, 'bump', self.read_bump, qos_profile=qos_profile_sensor_data)
        
        self.ranges = None

        # movement parameters 
        #velocity limits
        self.linear_vel_limit = 0.2
        self.angular_vel_limit = 1.0
        #response to errors
        self.kp_angle = 0.05
        self.kp_distance = 0.5

        #person stuff
        self.range_threshold = 4
        self.target_distance = 0.2
        self.angle_of_legs = 0
        self.distance_from_person = 0
        self.size_min = 0.01
        self.size_max = 1.0

        #Stopping 
        self.stop = False

        #markers
        self.marker_robot = Marker()
        self.marker = Marker()
        
    #reading functions 

    def read_scan(self, data: LaserScan):
        self.ranges = data.ranges
        self.locate_person()

    #def read_bump(self, data):
        #self.stop = any([data.leftFront, data.leftSide, data.rightFront, data.rightSide])  
    
    def locate_person(self):
        #storage bucket for legs
        object_array = []
        prev_range = 0
        #looking for legs
        for angle in range(-90, 90):
            current_range = self.ranges[angle]
            #establishing sudden changes in distance as a leg like quality that merrits futher inspection 
            if current_range != 0 and not math.isinf(current_range) and current_range < 1.0 and prev_range - current_range > self.range_threshold:
                object_size_array = self.detect_leg_angle(angle)
                object_array.extend(object_size_array)
            prev_range = current_range
            #calls function to calculate leg distance once legs are identified 
            self.calculate_leg_position(object_array)
    
    def detect_leg_angle(self, angle):
        #MATH! for good leg sizes!!! 
        object_array = []
        for angle2 in range(angle + 1, 45):
            current_range2 = self.ranges[angle2]
            size = math.tan(math.radians(angle2 - angle)) * current_range2
            if self.size_min < size < self.size_max:
                #leg size information added to leg bucket 
                object_array.append([(angle + angle2) / 2.0, current_range2])

        return object_array


    def calculate_leg_position(self, object_array):
        if len(object_array) > 0:
            self.angle_of_legs = sum([obj[0] for obj in object_array]) / len(object_array)
            self.distance_from_person = sum([obj[1] for obj in object_array]) / len(object_array)
            print(f"distance from person: {self.distance_from_person}")
        else:
            self.angle_of_legs = 0
            self.distance_from_person = self.target_distance
        
    def Move(self):
        #if bump, stops 
        ''' if self.stop:
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
            return
        '''
        # where is the person?
        error_distance = self.distance_from_person - self.target_distance
        error_theta = self.angle_of_legs

        #how fast do pets travel?
        linear_vel = self.kp_distance * error_distance
        angular_vel = self.kp_angle * error_theta

        #DON"T GO TOO FAST, you'll hurt yourself 
        linear_vel = max(min(linear_vel, self.linear_vel_limit), -self.linear_vel_limit)
        angular_vel = max(min(angular_vel, self.angular_vel_limit), -self.angular_vel_limit)

        self.vel_msg.linear.x = linear_vel
        self.vel_msg.angular.z = angular_vel
    
    
    
    def draw_marker(self):
    #we can't fucking talk about it. I know this is the worst way to do this but it's how it's getting done. 
    #pretenting the legs are a cube at estimated position of person 
    #pretending robot is a sphere. Everyone wants a pet sphere 
        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.id = 1
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = math.cos(math.radians(self.angle_of_legs)) * self.distance_from_person
        self.marker.pose.position.y = math.sin(math.radians(self.angle_of_legs)) * self.distance_from_person
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.person_marker_pub.publish(self.person_marker)

        self.marker_robot = Marker()
        self.marker_robot.header.frame_id = "base_link"
        self.marker.id = 0
        self.marker_robot.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker_robot.pose.position.x = 0.0
        self.marker_robot.pose.position.y = 0.0
        self.marker_robot.scale.x = 0.2
        self.marker_robot.scale.y = 0.2
        self.marker_robot.scale.z = 0.2
        self.marker_robot.color.a = 1.0
        self.marker_robot.color.b = 1.0
        self.pet_marker_pub.publish(self.pet_marker)
    
    def run_loop(self):
        if self.ranges is not None:
            self.Move()
            self.vel_pub.publish(self.vel_msg)
            self.draw_marker()
            print(f"marker position x {math.cos(math.radians(self.angle_of_legs)) * self.distance_from_person}")
            print(f"marker position y {math.sin(math.radians(self.angle_of_legs)) * self.distance_from_person}")

def main(args=None):
    rclpy.init(args=args)
    pet_node = NeatoPet()
    rclpy.spin(pet_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
