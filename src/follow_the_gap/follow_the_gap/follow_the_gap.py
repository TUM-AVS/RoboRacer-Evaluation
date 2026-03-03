import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from scipy.ndimage import uniform_filter1d
import numpy as np

class FollowTheGapNode(Node):
    def __init__(self):
        # Initialize the node with the name 'follow_the_gap_node'
        super().__init__('follow_the_gap_node')

        # Declare parameters with default values
        self.declare_parameter('bubble_radius', 0.2)
        self.declare_parameter('smoothing_filter_size', 5)
        self.declare_parameter('truncated_coverage_angle', 160.0)
        self.declare_parameter('max_accepted_distance', 6.0)
        self.declare_parameter('error_based_velocities.low', 2.4)
        self.declare_parameter('error_based_velocities.medium', 1.7)
        self.declare_parameter('error_based_velocities.high', 1.0)
        self.declare_parameter('steering_angle_reactivity', 0.4)

        # Setup subscription to the lidar scans and publisher for the drive command
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        # Flag to check if the truncated indices have been computed
        self.truncated = False

    def apply_smoothing_filter(self, input_vector, smoothing_filter_size):
        # Apply a uniform 1D filter to smooth the input vector, which helps to mitigate noise in the lidar data
        return uniform_filter1d(input_vector, size=smoothing_filter_size, mode='nearest')

    def truncated_start_and_end_indices(self, scan_msg, truncation_angle_coverage):
        # Calculate start and end indices for the truncated view based on the desired coverage angle
        truncated_range_size = int(
            np.deg2rad(truncation_angle_coverage) / (scan_msg.angle_max - scan_msg.angle_min) * len(scan_msg.ranges))
        start_index = len(scan_msg.ranges) // 2 - truncated_range_size // 2
        end_index = len(scan_msg.ranges) // 2 + truncated_range_size // 2
        print(f'Start index {start_index} \t End index: {end_index}')
        print(f'Len(ranges): {len(scan_msg.ranges)}')
        return start_index, end_index

    def minimum_element_index(self, input_vector):
        # Find the index of the minimum element in the input vector
        return int(np.argmin(input_vector))

    def find_largest_nonzero_sequence(self, input_vector):
        # Find the start and end indices of the largest sequence of non-zero values in the input vector
        max_gap = 0
        max_start = 0
        current_start = None
        for i, value in enumerate(input_vector):
            if value > 0.1:  # If value is non-zero, potentially start a new sequence
                if current_start is None:
                    current_start = i
            else:  # If we encounter a zero, check if the current sequence is the largest
                if current_start is not None:
                    if i - current_start > max_gap:
                        max_gap = i - current_start
                        max_start = current_start
                    current_start = None
        # Check if the last sequence is the largest one and update accordingly
        if current_start is not None and len(input_vector) - current_start > max_gap:
            max_gap = len(input_vector) - current_start
            max_start = current_start
        return max_start, max_start + max_gap - 1

    def zero_out_safety_bubble(self, input_vector, center_index, bubble_radius):
        # Zero out the elements within the 'bubble_radius' around the 'center_index'
        # This creates a 'bubble' around the closest obstacle where no valid paths can exist
        center_point_distance = input_vector[center_index]
        input_vector[center_index] = 0.0

        # Expand the bubble to the right
        current_index = center_index
        while current_index < len(input_vector) - 1 and input_vector[
            current_index + 1] < center_point_distance + bubble_radius:
            current_index += 1
            input_vector[current_index] = 0.0

        # Expand the bubble to the left
        current_index = center_index
        while current_index > 0 and input_vector[current_index - 1] < center_point_distance + bubble_radius:
            current_index -= 1
            input_vector[current_index] = 0.0

    def preprocess_lidar_scan(self, scan_msg):
        # Preprocess the lidar scan data
        # Convert NaNs to 0 for processing and cap the max range to a set value
        ranges = np.array(scan_msg.ranges)
        ranges[np.isnan(ranges)] = 0.0
        ranges[ranges > self.get_parameter('max_accepted_distance').get_parameter_value().double_value] = \
            self.get_parameter('max_accepted_distance').get_parameter_value().double_value
        # Apply the smoothing filter
        return self.apply_smoothing_filter(ranges,
                                      self.get_parameter('smoothing_filter_size').get_parameter_value().integer_value)

    def get_best_point(self, filtered_ranges, start_index, end_index):
        # Determine the best point to aim for within the largest gap
        return (start_index + end_index) // 2

    def get_steering_angle_from_range_index(self, scan_msg, best_point_index, closest_value):
        # Convert the index of the best point in the range array to a steering angle
        increment = scan_msg.angle_increment
        num_ranges = len(scan_msg.ranges)
        mid_point = num_ranges // 2

        # Determine if the best point is to the left or right of the center and calculate the steering angle
        if best_point_index < mid_point:
            best_point_steering_angle = -increment * (mid_point - best_point_index)
        else:
            best_point_steering_angle = increment * (best_point_index - mid_point)

        # Reactivity adjustment: adjust steering angle based on the distance of the closest object
        steering_angle_reactivity = self.get_parameter('steering_angle_reactivity').get_parameter_value().double_value
        distance_compensated_steering_angle = np.clip(
            best_point_steering_angle * steering_angle_reactivity / closest_value, -1.57, 1.57)
        return distance_compensated_steering_angle

    def scan_callback(self, scan_msg):
        # Callback function for processing lidar scans
        # First time this is run, we calculate the truncated indices based on the desired angle coverage
        if not self.truncated:
            truncated_indices = self.truncated_start_and_end_indices(scan_msg, self.get_parameter(
                'truncated_coverage_angle').get_parameter_value().double_value)
            self.get_logger().info(f"Truncated Indices: {truncated_indices}")
            self.truncated_start_index, self.truncated_end_index = truncated_indices
            self.truncated = True

        # Process the lidar scan data
        filtered_ranges = self.preprocess_lidar_scan(scan_msg)
        # Find the closest obstacle
        closest_index = self.minimum_element_index(filtered_ranges[self.truncated_start_index:self.truncated_end_index])
        closest_index = closest_index +  + self.truncated_start_index
        closest_range = filtered_ranges[closest_index]

        # Zero out the safety bubble around the closest obstacle
        self.zero_out_safety_bubble(filtered_ranges, closest_index,
                               self.get_parameter('bubble_radius').get_parameter_value().double_value)

        # Find the largest gap in the scan data
        start_index, end_index = self.find_largest_nonzero_sequence(filtered_ranges[self.truncated_start_index:self.truncated_end_index])
        start_index = start_index + self.truncated_start_index
        end_index = end_index + self.truncated_start_index
        # Determine the best point within that gap to aim for
        best_point_index = self.get_best_point(filtered_ranges, start_index, end_index)

        # Get the steering angle that directs the car towards the best point
        steering_angle = self.get_steering_angle_from_range_index(scan_msg, best_point_index, closest_range)

        # Prepare the drive message with the steering angle and corresponding speed
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'laser'
        print(f'Steering angle: {steering_angle}')
        drive_msg.drive.steering_angle = steering_angle

        # Set speed based on steering angle using a simple comparison approach
        if abs(steering_angle) > 0.349:
            drive_msg.drive.speed = self.get_parameter('error_based_velocities.high').get_parameter_value().double_value
        elif abs(steering_angle) > 0.174:
            drive_msg.drive.speed = self.get_parameter('error_based_velocities.medium').get_parameter_value().double_value
        else:
            drive_msg.drive.speed = self.get_parameter('error_based_velocities.low').get_parameter_value().double_value

        # Publish the drive command
        self.drive_pub.publish(drive_msg)

def main(args=None):
    # Main function to initialize the ROS node and spin
    rclpy.init(args=args)
    node = FollowTheGapNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    # Entry point for the script
    main()
