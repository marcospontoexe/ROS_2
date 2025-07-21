from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marcos_publisher',
            executable='simple_publisher_node',
            output='screen'),
    ])