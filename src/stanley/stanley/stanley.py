import numpy as np
import rclpy
import math
from rclpy.node import Node
import sys, os
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
sys.path.append(os.getcwd() + '/src')
from ws_params import raceline_path

class StanleyController(Node):
    """
    Implements the hand-crafted, rule-based Stanley Controller.
    Code original implemneted by Felix Jahncke for torch, edited by Jonathan Mohr for ROS2 with numpy.
    """
    def __init__(self):
        super().__init__('stanley_controller')
        
        # Set constant velocity parameter
        self.velocity_goal = 0.7 # percentage of the target velocity
        # Set the Stanley parameters
        self.k_e = 1.8
        self.k_h = 1.3

        # Initialize vehicle parameters
        self.lf = 0.15875
        self.lr = 0.17145
        self.max_steering_angle = 0.4189

        # Load the waypoints
        self.raceline_data = np.genfromtxt(raceline_path, delimiter=";", comments='#')
        self.raceline_data[:, 3] = self.raceline_data[:, 3] - np.pi / 2
        self.raceline_data[:, 3] = np.remainder(self.raceline_data[:, 3], 2 * np.pi)

        # Initialize the steering angle
        self.steer = 0.0
        
        # Publisher for control commands
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
         # Subscribe to current pose
        self.pose_sub = self.create_subscription(Odometry, '/pf/pose/odom', self.pose_callback, 10)
        #self.initalpose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.intialpose_callback, 10)
        
     
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        """
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # pitch (y-axis rotation)
        sinp = math.sqrt(1 + 2 * (w * y - x * z))
        cosp = math.sqrt(1 - 2 * (w * y - x * z))
        pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2
        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp);   

        return roll, pitch, yaw # in radians
    
    def pose_callback(self, msg):
        """
        Implements the Stanley Controller for the F1Tenth car.
        """
        ackermann_cmd = AckermannDriveStamped()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]    
        pose = np.array([x, y, theta])
        crosstrack_err, heading_err, min_index = self.error_model(pose)

        # Set the velocity according to the waypoints and multiply it with a factor for stability
        velocity = self.raceline_data[min_index, 5] * self.velocity_goal

        # Calculate the steering angle according to the Stanley Controller's control law
        self.steer = self.k_h * heading_err + np.arctan(self.k_e * -crosstrack_err / (velocity + 0.00001))

        # Publish the control commands
        ackermann_cmd.header.stamp = self.get_clock().now().to_msg()
        ackermann_cmd.drive.steering_angle = np.clip(self.steer, -self.max_steering_angle, self.max_steering_angle)
        ackermann_cmd.drive.speed = velocity
        self.drive_pub.publish(ackermann_cmd)
    
    
    def error_model(self, pose):
        """
        Calculates the cross-track and heading error of an input pose to the closest waypoint.
        """
        # Convert the pose from the center of gravity to the front axle
        pose_cg_x = pose[0] + self.lf * np.sin(pose[2])
        pose_cg_y = pose[1] + self.lf * np.cos(pose[2])
        pose_cg_theta = np.remainder(pose[2] + np.pi, 2 * np.pi)

        # Calculate the Euclidean distance between the pose and all waypoints
        distance_to_waypoints = np.linalg.norm(self.raceline_data[:, 1:3] - np.stack([pose_cg_x, pose_cg_y], axis=-1), axis=1)
        min_index = np.argmin(distance_to_waypoints)
        distance_to_waypoint = np.stack([pose_cg_x - self.raceline_data[min_index, 1], pose_cg_y - self.raceline_data[min_index, 2]])
        front_axle_vector = np.stack([-np.cos(pose_cg_theta + np.pi / 2), -np.sin(pose_cg_theta + np.pi / 2)])

        # Compute the cross-track error
        crosstrack_error = np.dot(distance_to_waypoint, front_axle_vector)

        # Compute the heading error
        raw_heading_error = (self.raceline_data[min_index, 3] - pose_cg_theta)

        # Clamp the heading error between 0 and 2pi
        heading_error = np.remainder(raw_heading_error + np.pi, 2 * np.pi) - np.pi

        return crosstrack_error, heading_error, min_index



    
def main(args=None):
    rclpy.init(args=args)
    stanley_controller = StanleyController()
    rclpy.spin(stanley_controller)
    stanley_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
