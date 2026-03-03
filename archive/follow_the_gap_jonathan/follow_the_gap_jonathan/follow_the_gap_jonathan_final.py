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
        self.declare_parameter('smoothing_filter_size', 3)
        self.declare_parameter('truncated_coverage_angle', 140.0)
        self.declare_parameter('max_accepted_distance',7.0)
        self.bubble_radius_close = 0.5 # Bubble radius for closest obstacle
        self.bubble_radius_diff = 1.2 # Bubble radius for points with large difference between two following scans
        self.allowed_difference = 0.3
        self.vehicle_width =0.3 # preicse would be 0.296
        self.safety_margin_gapsize = 0.7 # how much larger gap must be than vehicle width
        self.safety_margin_gapside = 0.7 # how much distance from the edge of the gap shall be ensured
        self.start_index = 0
        self.end_index = 0
        self.no_steering_index = 0
        self.max_speed = 2.0
        

        # Setup subscription to the lidar scans and publisher for the drive command
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)


    def calculate_indexes(self, scan_msg):
        """Calculates start and end index based on FOV"""
        fov_degrees = self.get_parameter('truncated_coverage_angle').get_parameter_value().double_value # Field of view in degrees
        fov_radians = np.deg2rad(fov_degrees)  # Convert degrees to radians
        # Calculate start and end indices based on the field of view
        start_index = int((len(scan_msg.ranges) - 1) / 2 - (fov_radians / (scan_msg.angle_max - scan_msg.angle_min)) * (len(scan_msg.ranges) - 1) / 2)
        end_index = int((len(scan_msg.ranges) - 1) / 2 + (fov_radians / (scan_msg.angle_max - scan_msg.angle_min)) * (len(scan_msg.ranges) - 1) / 2)
        no_steering_index = int((len(scan_msg.ranges) - 1) / 2)
        #print(f' Start index: {start_index:,}  End index: {end_index:,}   No steering index: {no_steering_index:,}')
        return start_index, end_index, no_steering_index
    
    def scan2scan_distance(self, dist1, idx1, dist2, idx2, angle_increment):
        distance = np.sqrt((np.sin(idx1 * angle_increment) * dist1 - np.sin(idx2 * angle_increment) * dist2)**2 + (np.cos(idx1 * angle_increment) * dist1 - np.cos(idx2 * angle_increment) * dist2)**2)
        return distance
    
    def find_best_nonzero_sequence(self, scan_msg):
    # Find the start and end indices of the largest sequence of non-zero values in the input vector
        max_gap = 0
        max_start = 0
        max_mean = 0
        gap_space = 0.0
        current_start = None
        input_vector = np.array(scan_msg.ranges)
        for i, value in enumerate(input_vector):
            if value > 0.1:  # If value is non-zero, potentially start a new sequence
                if current_start is None:
                    current_start = i
            else:  # If we encounter a zero, check if the current sequence is the largest
                if current_start is not None:
                    # Calculate the width of the gap
                    gap_space = self.scan2scan_distance(scan_msg.ranges[current_start], current_start, scan_msg.ranges[i-1], i-1, scan_msg.angle_increment)
                    # Calculate if the gap is on average further then the best gap so far
                    if np.mean(input_vector[current_start:i]) > max_mean and gap_space > (self.vehicle_width + self.safety_margin_gapsize):
                        max_mean = np.mean(input_vector[current_start:i])
                        max_gap = i - current_start
                        max_start = current_start
                    current_start = None
                    gap_space = 0.0
        # Check if the last sequence is the largest one and update accordingly
        if current_start is not None and np.mean(input_vector[current_start:]) > max_mean:
            gap_space = self.scan2scan_distance(scan_msg.ranges[current_start], current_start, scan_msg.ranges[-1], len(scan_msg.ranges) - 1, scan_msg.angle_increment)
            if gap_space > (self.vehicle_width + self.safety_margin_gapsize):
                max_gap = len(input_vector) - current_start
                max_start = current_start
        # Eliminate all values that are too cloose to the edges of the gap
        for i in range(max_start, max_start + max_gap):
            if(self.scan2scan_distance(scan_msg.ranges[max_start], max_start, scan_msg.ranges[i], i, scan_msg.angle_increment) < self.vehicle_width + self.safety_margin_gapside):
                scan_msg.ranges[i] = 0.0
            elif(self.scan2scan_distance(scan_msg.ranges[max_start + max_gap - 1], max_start + max_gap - 1, scan_msg.ranges[i], i, scan_msg.angle_increment) < self.vehicle_width + self.safety_margin_gapside):
                scan_msg.ranges[i] = 0.0
        #print(f'Gap start:{max_start} \t Gap end: {max_start + max_gap - 1}')
        return max_start, max_start + max_gap - 1

    def bubble_algorithm(self, scan_msg, center_idx, bubble_radius):
        """Zero out the elements within the 'bubble_radius' around the 'center_index'. \n
        This creates a 'bubble' around the closest obstacle where no valid paths can exist \n
        Copied from Waterloo Follow The Gap code"""
        center_dist = scan_msg.ranges[center_idx]
        scan_msg.ranges[center_idx] = 0.0
        # Expand the bubble to the right
        current_index = center_idx
        while current_index < len(scan_msg.ranges) - 1 and \
            self.scan2scan_distance(center_dist, center_idx, scan_msg.ranges[current_index + 1], current_index + 1, scan_msg.angle_increment) < bubble_radius:
            current_index += 1
            scan_msg.ranges[current_index] = 0.0
        # Expand the bubble to the left
        current_index = center_idx
        while current_index > 0 and \
            self.scan2scan_distance(center_dist, center_idx, scan_msg.ranges[current_index - 1], current_index - 1, scan_msg.angle_increment) < bubble_radius:
            current_index -= 1
            scan_msg.ranges[current_index] = 0.0
        
    def follow_the_gap_algorithm(self, scan_msg):
        # Get the closest index before manipulating ranges where values differ to much
        closest_idx = np.argmin(scan_msg.ranges)        
        # At indexes where the difference between the distance of 2 neighboring scans is larger than the allowed_difference elimante all points with in the vehicle width of this difference
        diff_indexes = []
        # Find indexes with such a difference
        for i in range(len(scan_msg.ranges)-1):
            if(abs(scan_msg.ranges[i+1] - scan_msg.ranges[i]) > self.allowed_difference):
                diff_indexes.append(i)
                diff_indexes.append(i + 1)
        # Remove duplicates from list
        diff_indexes = list(dict.fromkeys(diff_indexes))
        #print(f'diff indexes: {diff_indexes}')
        # Create Bubbles around all Differences
        for elem in diff_indexes:
            self.bubble_algorithm(scan_msg, elem, self.bubble_radius_diff)
        # Create Bubble around closest obstacle
        self.bubble_algorithm(scan_msg, closest_idx, self.bubble_radius_close)
        # Find best gap
        max_gap_start, max_gap_end = self.find_best_nonzero_sequence(scan_msg)
        
        try:
            # Find angle(s) with largest range value within max gap
            max_range_values = scan_msg.ranges[max_gap_start:max_gap_end+1]
            max_range_indices = np.where(max_range_values == np.max(max_range_values))[0]
            max_range_indices = max_range_indices + max_gap_start
            max_range_indices = max_range_indices + self.start_index
            #print(f' Max Range indicies: {max_range_indices}')
            # Get index closest to the middle (only necessary steering)
            steering_idx = min(max_range_indices, key=lambda x: abs(x - self.no_steering_index))
            #print(f' Start index: {self.start_index:,}  End index: {self.end_index:,}  Steering index: {steering_idx:,}    No steering index: {self.no_steering_index:,}')
            # Convert index to steering angle
            steering_angle = (steering_idx - self.no_steering_index) * scan_msg.angle_increment
            #print(f'Steering angle: {np.rad2deg(steering_angle)}')

            # Set speed based on steering angle using a simple comparison approach
            velocity = self.max_speed 
        except ValueError as e:
            print(f"Caught a ValueError: {e}. Stopping the vehicle.")
            steering_angle = 0.0
            velocity = 0.0

        return steering_angle, velocity


    def preprocessing_scan(self, input_vector, smoothing_filter_size):
        """Setting values exceeding max accepted distance to its value and applying a uniform 1D filter with smoothing filter size for reducing noise"""
        input_vector = np.array(input_vector)  # Convert input_vector to a numpy array
        input_vector[np.isnan(input_vector)] = 0.0
        input_vector[input_vector > self.get_parameter('max_accepted_distance').get_parameter_value().double_value] = \
            self.get_parameter('max_accepted_distance').get_parameter_value().double_value
        # Apply a uniform 1D filter to smooth the input vector, which helps to mitigate noise in the lidar data
        return uniform_filter1d(input_vector, size=smoothing_filter_size, mode='nearest')



    def scan_callback(self, scan_msg):
        # Callback function for processing lidar scans
        if self.end_index == 0:
            self.start_index, self.end_index, self.no_steering_index = self.calculate_indexes(scan_msg)
        modified_scan = LaserScan()
        modified_scan.header = scan_msg.header
        modified_scan.angle_min = scan_msg.angle_min + self.start_index * scan_msg.angle_increment
        modified_scan.angle_max = scan_msg.angle_min + self.end_index * scan_msg.angle_increment
        modified_scan.angle_increment = scan_msg.angle_increment
        modified_scan.time_increment = scan_msg.time_increment
        modified_scan.scan_time = scan_msg.scan_time
        modified_scan.ranges = [ x for x in (self.preprocessing_scan(scan_msg.ranges[self.start_index:self.end_index+1],
                                      self.get_parameter('smoothing_filter_size').get_parameter_value().integer_value)).tolist()]
        steering_angle, velocity = self.follow_the_gap_algorithm(modified_scan)
      

        # Create drive message for publishing steering angle and corresponding speed
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
