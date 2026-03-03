import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import csv
import numpy as np
from visualization_msgs.msg import Marker
import time
from datetime import datetime

# Pure Pursuit Controller using the particle filter position, constant speed and the very basic algorithm without variable look ahead etc.
class BenchmarkingMetrics(Node):
    def __init__(self):
        super().__init__('benchmark_metrics_node')
        
        # Set path for results file
        date_time_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S_")
        self.csv_path = './benchmark_results/' + date_time_str + 'lap_stats.csv'
        
        # Write headers
        with open(self.csv_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Lap', 'Lap Time', 'Speed Smoothness', 'Steering Smoothness'])
        
        # Set coordinates of Start/Finish line
        self.start_p = Point()
        self.start_p.x = 0.5
        self.start_p.y = -1.0
        self.end_p = Point()
        self.end_p.x = 0.6
        self.end_p.y = 1.0
        
        # Helper values for determining wether the vehicle is crossing the finish line
        # Calculate slope and intercept for later checking if vehicle is crossing finish line
        self.fline_slope = (self.end_p.y - self.start_p.y) / (self.end_p.x - self.start_p.x)
        self.fline_intercept = self.start_p.y - self.start_p.x * self.fline_slope
        # Take coordinate direction with greater difference between start and end point of finish line
        if abs(self.end_p.x - self.start_p.x) > abs(self.end_p.y - self.start_p.y):
            self.fline_max = max(self.end_p.x, self.start_p.x)
            self.fline_min = min(self.end_p.x, self.start_p.x)
        else:
            self.fline_max = max(self.end_p.y, self.start_p.y)
            self.fline_min = min(self.end_p.y, self.start_p.y)
        
        
        
        # Create subscription to vehicle position (based on particle filter)
        self.sub_odom = self.create_subscription(Odometry, '/pf/pose/odom', self.pose_callback, 10)
        # Create subscription to drive commands
        self.sub_drive = self.create_subscription(AckermannDriveStamped, '/drive', self.drive_callback, 10)
        # Create Publisher for visualizing start finish line and visualize it
        self.pub_vis = self.create_publisher(Marker, 'start_finish_line', 10)
        
        # Create a timer to publish the finish line visualization
        self.timer = self.create_timer(0.5, self.publish_finish_line)
       
        # Initialize  variables
        self.lap_count = 0
        self.lap_time = 0
        self.speed_diffs = []
        self.steering_diffs = []
        self.previous_speed = 0.0
        self.previous_steering = 0.0
        self.last_drive_timestamp = None
        self.start_time = None


    def pose_callback(self, odom_msg):
        # Check if vehicle is crossing start finish line
        if (abs(odom_msg.pose.pose.position.x * self.fline_slope + self.fline_intercept - odom_msg.pose.pose.position.y) < 0.30 and 
            self.fline_min < odom_msg.pose.pose.position.y < self.fline_max):
            # Starting first lap
            if self.start_time == None:
                self.start_time = time.time()
                self.speed_diffs = []
                self.steering_diffs = []
            # Starting new lap, but only if time is realistic (otherwise same crossing dected mulitple times)
            elif time.time() - self.start_time > 5.0:
                self.lap_time = time.time() - self.start_time
                self.start_time = time.time()
                self.lap_count += 1
                self.write_lap_stats()
                self.speed_diffs = []
                self.steering_diffs = []                      
    
    def drive_callback(self, drive_msg):
        msg_timestamp = drive_msg.header.stamp.sec + drive_msg.header.stamp.nanosec * 1e-9
        if self.last_drive_timestamp != None:
            time_diff = msg_timestamp - self.last_drive_timestamp
            if time_diff > 0:
                speed_d = abs(drive_msg.drive.speed - self.previous_speed)
                if speed_d > 0:
                    self.speed_diffs.append(speed_d / time_diff)               
                steering_d = abs(drive_msg.drive.steering_angle - self.previous_steering) 
                if steering_d > 0:
                    self.steering_diffs.append(steering_d / time_diff)
                #self.get_logger().info(f'Current Speed Smoothness: {np.mean(self.speed_diffs)}  Steering Smoothness: {np.mean(self.steering_diffs)}')
            
        self.last_drive_timestamp = msg_timestamp
        self.previous_speed = drive_msg.drive.speed
        self.previous_steering = drive_msg.drive.steering_angle
        
    def publish_finish_line(self):
        finish_line = Marker()
        finish_line.header.frame_id = 'map'
        finish_line.type = Marker.LINE_STRIP
        finish_line.action = Marker.ADD
        finish_line.pose.orientation.w = 1.0
        finish_line.scale.x = 0.1  # Line width
        finish_line.color.r = 1.0
        finish_line.color.a = 0.8
        finish_line.points.append(self.start_p)
        finish_line.points.append(self.end_p)
        self.pub_vis.publish(finish_line)
    
    def write_lap_stats(self):
        # Append lap time to the CSV file
        with open(self.csv_path, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([self.lap_count, self.lap_time]) #np.mean(self.speed_diffs), np.mean(self.steering_diffs)])
        self.get_logger().info(f'Logged Lap {self.lap_count} with Lap time {self.lap_time}')
     
        
        

def main(args=None):
    rclpy.init(args=args)
    benchmark_metrics_node = BenchmarkingMetrics()
    rclpy.spin(benchmark_metrics_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
