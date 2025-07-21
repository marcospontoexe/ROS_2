from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marcos',       # nome do pacote
            executable='simple_node',   # nome do executável gerado no momento da compilação (CMakeLists.txt)
            output='screen'),
    ])