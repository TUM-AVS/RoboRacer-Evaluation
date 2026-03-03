import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import sys, os
sys.path.append(os.getcwd() + '/src')
from ws_params import raceline_path

class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_visualizer_node')
        self.declare_parameter('rviz_waypoints_topic', '/raceline_waypoints')
        self.waypoints = []
        waypoints_path = raceline_path
        self.rviz_waypoints_topic = self.get_parameter('rviz_waypoints_topic').get_parameter_value().string_value

        self.vis_path_pub = self.create_publisher(MarkerArray, self.rviz_waypoints_topic, 1000)
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info('This node has been launched')
        self.download_waypoints(waypoints_path)
        

    def download_waypoints(self, waypoints_path):
        self.waypoints = np.genfromtxt(waypoints_path, delimiter=";", comments='#')[:, 1:3]
        print(f"Read {len(self.waypoints)} waypoints.")

    def visualize_points(self):
        marker_array = MarkerArray()
        for i, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.id = i
            marker_array.markers.append(marker)

        self.vis_path_pub.publish(marker_array)

    def timer_callback(self):
        self.visualize_points()
        #self.test_movingpoint_vis()

def main(args=None):
    rclpy.init(args=args)
    waypoint_visualizer = WaypointVisualizer()
    rclpy.spin(waypoint_visualizer)
    waypoint_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
