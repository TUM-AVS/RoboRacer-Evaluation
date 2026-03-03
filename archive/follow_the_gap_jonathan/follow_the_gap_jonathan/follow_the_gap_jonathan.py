import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from scipy.ndimage import uniform_filter1d
import numpy as np

#Jonathans Version

class FollowTheGapNode(Node):
    def __init__(self):
        # Initialize the node with the name 'follow_the_gap_node'
        super().__init__('follow_the_gap_node')

        # Declare parameters with default values
        self.declare_parameter('bubble_radius', 0.3)
        self.declare_parameter('saftey_arc_length', 0.3)
        self.declare_parameter('smoothing_filter_size', 3)
        self.declare_parameter('truncated_coverage_angle', 100.0)
        self.declare_parameter('max_accepted_distance', 4.0)
        self.declare_parameter('error_based_velocities.low', 2.0)
        self.declare_parameter('error_based_velocities.medium', 1.5)
        self.declare_parameter('error_based_velocities.high', 0.8)
        self.declare_parameter('steering_angle_reactivity', 0.20)

        # Setup subscription to the lidar scans and publisher for the drive command
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        # Flag to check if the truncated indices have been computed
        self.truncated = False 


    def process_laserscan(self, scan_msg):
        fov_degrees = self.get_parameter('truncated_coverage_angle').get_parameter_value().double_value # Field of view in degrees
        fov_radians = np.deg2rad(fov_degrees)  # Convert degrees to radians

        # Calculate start and end indices based on the field of view
        start_index = int((len(scan_msg.ranges) - 1) / 2 - (fov_radians / (scan_msg.angle_max - scan_msg.angle_min)) * (len(scan_msg.ranges) - 1) / 2)
        end_index = int((len(scan_msg.ranges) - 1) / 2 + (fov_radians / (scan_msg.angle_max - scan_msg.angle_min)) * (len(scan_msg.ranges) - 1) / 2)

        # print(f' Start index: {start_index:,}  End index: {end_index:,}')
        # Set range values outside the field of view to 0
        for i in range(start_index):
            scan_msg.ranges[i] = 0.0
        for i in range(end_index +1, len(scan_msg.ranges)):
            scan_msg.ranges[i]= 0.0
        
        # Apply moving average filter
        #filter_size = 5
        filter_weights = [0.2, 0.2, 0.2, 0.2, 0.2]
        scan_msg.ranges = [x for x in (np.convolve(scan_msg.ranges, filter_weights, mode='same')).tolist()]

        # Clip range values between 0 and 4
        scan_msg.ranges = [x for x in (np.clip(scan_msg.ranges, 0.0, 4.0)).tolist()]

        # Find minimum range value and its angle
        closest_dist = np.min(scan_msg.ranges[start_index:(end_index + 1)])
        closest_idx = np.argmin(scan_msg.ranges[start_index:(end_index + 1)]) + start_index
        closest_angle = scan_msg.angle_min + closest_idx * scan_msg.angle_increment

        return closest_dist, closest_angle, closest_idx
    
    def find_largest_nonzero_sequence(self, scan_msg):
    # Find the start and end indices of the largest sequence of non-zero values in the input vector
        max_gap = 0
        max_start = 0
        current_start = None
        input_vector = np.array(scan_msg.ranges)
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
        
        #print(f'Gap start:{max_start} \t Gap end: {max_start + max_gap - 1}')
        return max_start, max_start + max_gap - 1

    def follow_the_gap_algorithm(self, scan_msg):
        # Step 1: Process LaserScan
        closest_dist, closest_angle, closest_idx = self.process_laserscan(scan_msg)
        
        # Step 2: Zero out ranges within radius of robot
        for i, range_val in enumerate(scan_msg.ranges):
            angle_i = scan_msg.angle_min + i * (scan_msg.angle_max - scan_msg.angle_min) / len(scan_msg.ranges)
            arc_length = closest_dist * abs(angle_i - closest_angle)
            #print(f' Arc Length: {arc_length}')
            #print(f'closest distance: {closest_dist:,} at angle: {closest_angle}')
            if abs(arc_length) < self.get_parameter('saftey_arc_length').get_parameter_value().double_value:
                scan_msg.ranges[i] = 0.0

        # Zero out the elements within the 'bubble_radius' around the 'center_index'
        # This creates a 'bubble' around the closest obstacle where no valid paths can exist
        center_point_distance = scan_msg.ranges[closest_idx]
        scan_msg.ranges[closest_idx] = 0.0

        # Expand the bubble to the right
        current_index = closest_idx
        while current_index < len(scan_msg.ranges) - 1 and scan_msg.ranges[
            current_index + 1] < center_point_distance + self.get_parameter('bubble_radius').get_parameter_value().double_value:
            current_index += 1
            scan_msg.ranges[current_index] = 0.0

        # Expand the bubble to the left
        current_index = closest_idx
        while current_index > 0 and scan_msg.ranges[current_index - 1] < center_point_distance + self.get_parameter('bubble_radius').get_parameter_value().double_value:
            current_index -= 1
            scan_msg.ranges[current_index] = 0.0
        
        # Step 3: Find max gap
        max_gap_start, max_gap_end = self.find_largest_nonzero_sequence(scan_msg)
        
        try:
            # Step 4: Find angle(s) with largest range value within max gap
            max_range_values = scan_msg.ranges[max_gap_start:max_gap_end+1]
            max_range_indices = np.where(max_range_values == np.max(max_range_values))[0]
            max_range_indices = max_range_indices + max_gap_start
            # Get index closest to the middle (only necessary steering)
            steering_idx = min(max_range_indices, key=lambda x: abs(x - len(scan_msg.ranges) // 2))
            print(f'Steering idx: {steering_idx}')
            # Convert index to steering angle
            steering_angle = (steering_idx - len(scan_msg.ranges) // 2) * scan_msg.angle_increment
            print(f'Steering angle: {steering_angle}')

            # Set speed based on steering angle using a simple comparison approach
            if abs(steering_angle) > 0.349:
                velocity = self.get_parameter('error_based_velocities.high').get_parameter_value().double_value
            elif abs(steering_angle) > 0.174:
                velocity = self.get_parameter('error_based_velocities.medium').get_parameter_value().double_value
            else:
                velocity = self.get_parameter('error_based_velocities.low').get_parameter_value().double_value
        except ValueError as e:
            print(f"Caught a ValueError: {e}. Stopping the vehicle.")
            steering_angle = 0.0
            velocity = 0.0

        return steering_angle, velocity


    def apply_smoothing_filter(self, input_vector, smoothing_filter_size):
        input_vector = np.array(input_vector)  # Convert input_vector to a numpy array
        input_vector[np.isnan(input_vector)] = 0.0
        input_vector[input_vector > self.get_parameter('max_accepted_distance').get_parameter_value().double_value] = \
            self.get_parameter('max_accepted_distance').get_parameter_value().double_value
        # Apply a uniform 1D filter to smooth the input vector, which helps to mitigate noise in the lidar data
        return uniform_filter1d(input_vector, size=smoothing_filter_size, mode='nearest')



    def scan_callback(self, scan_msg):
        # Callback function for processing lidar scans
        modified_scan = LaserScan()
        modified_scan.header = scan_msg.header
        modified_scan.angle_min = scan_msg.angle_min
        modified_scan.angle_max = scan_msg.angle_max
        modified_scan.angle_increment = scan_msg.angle_increment
        modified_scan.time_increment = scan_msg.time_increment
        modified_scan.scan_time = scan_msg.scan_time
        modified_scan.ranges = [ x for x in (self.apply_smoothing_filter(scan_msg.ranges,
                                      self.get_parameter('smoothing_filter_size').get_parameter_value().integer_value)).tolist()]
        modified_scan.range_min = scan_msg.range_min
        modified_scan.range_max = scan_msg.range_max

        steering_angle, velocity = self.follow_the_gap_algorithm(modified_scan)
      

        # Prepare the drive message with the steering angle and corresponding speed
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'laser'
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity

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
