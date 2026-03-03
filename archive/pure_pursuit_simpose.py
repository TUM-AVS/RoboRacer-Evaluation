import rclpy
from rclpy.node import Node
import tf2_geometry_msgs
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point
from tf2_geometry_msgs import PointStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import csv
import numpy as np
from visualization_msgs.msg import Marker
import tf2_ros
import sys, os
sys.path.append(os.getcwd() + '/src')
from ws_params import raceline_path

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.wheel_base = 0.335
        
        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publisher for control commands
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Parameters
        self.max_speed = 2.0  # maximum linear speed
        self.lookahead_distance = 0.5  # lookahead distance
        self.kp = 1.0  # proportional gain

        # Initialize variables
        self.current_pose = None
        self.waypoints = []
        self.next_waypoint_index = 0

        # Read waypoints from CSV file
        self.read_waypoints_from_csv(raceline_path)
        

        # Initialize Marker to visualize current waypoint target
        self.target_marker_pub = self.create_publisher(Marker, '/target_Marker', 5)
        self.target_marker = Marker()
        self.target_marker.ns ='test'
        self.target_marker.id = 0
        self.target_marker.header.frame_id = 'map'
        self.target_marker.header.stamp = self.get_clock().now().to_msg()
        self.target_marker.type = Marker.SPHERE
        self.target_marker.action = Marker.ADD
        self.target_marker.scale.x = 0.55
        self.target_marker.scale.y = 0.55
        self.target_marker.scale.z = 0.55
        self.target_marker.color.a = 1.0
        self.target_marker.color.r = 1.0
        
        # Set initial waypoint index
        self.set_initial_waypoint_index()
        
        # Subscribe to current pose
        self.pose_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)
        
    def tf_callback(self, msg):
        print("entered tf callback")
        print(msg)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

        # Calculate control commands
        ackermann_cmd = self.calculate_control_commands()
        # Publish control commands
        self.drive_pub.publish(ackermann_cmd)

    def calculate_control_commands(self):
        ackermann_cmd = AckermannDriveStamped()

        # Find the next waypoint and mark it
        if self.current_pose and self.waypoints:
            print("entered control calculation")
            next_waypoint = self.waypoints[self.next_waypoint_index]
            self.target_marker.pose.position.x = next_waypoint.x
            self.target_marker.pose.position.y = next_waypoint.y
            self.target_marker_pub.publish(self.target_marker)

            # Calculate distance and angle to the next waypoint
            L = np.sqrt((next_waypoint.x - self.current_pose.position.x)**2 + (next_waypoint.y - self.current_pose.position.y)**2)
            
            # create PointStamped from waypoint for transformation
            map_point = PointStamped()
            map_point.header.frame_id = "map"
            map_point.point.x = next_waypoint.x
            map_point.point.y = next_waypoint.y
            map_point.point.z = 0.0

            # Transform the point into the new frame
            while(True):
                try:
                    frames_string = self.tf_buffer.all_frames_as_string()
                    #self.get_logger().info("Available TF frames: \n%s" %frames_string)
                    #print(f'Coordinates before transform: x:{map_point.point.x}  y:{map_point.point.y}')
                    transform = self.tf_buffer.lookup_transform("ego_racecar/base_link", "map", rclpy.time.Time(), rclpy.duration.Duration(seconds=0.3))
                    car_point = tf2_geometry_msgs.do_transform_point(map_point, transform)
                    #print(f'Coordinates after transform: x:{car_point.point.x}  y:{car_point.point.y}')
                    break                    
                except (Exception, TypeError) as ex:
                    self.get_logger().error(str(ex))
            
            # Calculate angle error directly from quaternion
            y = car_point.point.y
            Radius = L**2 / (2 * np.abs(y))
            if y<0:
                ackermann_cmd.drive.steering_angle = -self.wheel_base / Radius
            else:
                ackermann_cmd.drive.steering_angle = self.wheel_base / Radius
            print(f"Steering angle: {ackermann_cmd.drive.steering_angle:.5f}")
            ackermann_cmd.drive.speed = 1.0

            # Check if the current waypoint is reached
            if L < 0.4:
                self.next_waypoint_index = (self.next_waypoint_index + 1) % len(self.waypoints)  # Move to the next waypoint or wrap around if at the end

        return ackermann_cmd


    def read_waypoints_from_csv(self, waypoints_file):
        with open(waypoints_file, 'r') as csv_file:
            csv_reader = csv.reader(csv_file)
            for row in csv_reader:
                x, y, z = map(float, row)
                self.waypoints.append(Point(x=x, y=y, z=z))
        print(f"Read {len(self.waypoints)} waypoints.")

    def set_initial_waypoint_index(self):
        # Find the index of the closest waypoint to the initial position of the vehicle
        if self.current_pose:
            min_distance = float('inf')
            closest_index = 0
            for i, waypoint in enumerate(self.waypoints):
                distance = np.sqrt((waypoint.x - self.current_pose.position.x)**2 + (waypoint.y - self.current_pose.position.y)**2)
                if distance < min_distance:
                    min_distance = distance
                    closest_index = i
            self.next_waypoint_index = closest_index

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_controller = PurePursuitController()
    rclpy.spin(pure_pursuit_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
