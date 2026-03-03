from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    pure_pursuit_node = Node(
        package='stanley',
        executable='stanley',
        name='stanley_node',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('stanley'), 'launch', 'stanley.rviz')]
    )
    raceline_visualizer_node = Node(
        package='raceline_visualization',
        executable='visualize',
        name='raceline_visualizer_node',
    )

    # finalize
    ld.add_action(rviz_node)
    ld.add_action(pure_pursuit_node)
    ld.add_action(raceline_visualizer_node)
    

    return ld