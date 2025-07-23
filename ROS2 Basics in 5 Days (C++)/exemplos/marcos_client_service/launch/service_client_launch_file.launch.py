from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marcos_client_service',
            executable='service_client_node',
            output='screen'),
    ])