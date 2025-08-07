import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    tb3_0_nav2_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'tb3_0_amcl_config.yaml')
    tb3_1_nav2_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'tb3_1_amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'turtlebot_area_two_robots_clean.yaml')

    controller_yaml_tb3_0 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'tb3_0_controller.yaml')
    planner_yaml_tb3_0 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'tb3_0_planner_server.yaml')
    recovery_yaml_tb3_0 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'tb3_0_recovery.yaml')
    bt_navigator_yaml_tb3_0 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'tb3_0_bt_navigator.yaml')

    controller_yaml_tb3_1 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'tb3_1_controller.yaml')
    planner_yaml_tb3_1 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'tb3_1_planner_server.yaml')
    recovery_yaml_tb3_1 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'tb3_1_recovery.yaml')
    bt_navigator_yaml_tb3_1 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'tb3_1_bt_navigator.yaml')
    
    
    return LaunchDescription([    
        ## map server e amcl
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'topic_name':"map"},
                        {'frame_id':"map"},
                        {'yaml_filename':map_file}]
        ),
            
        Node(
            namespace='tb3_0',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[tb3_0_nav2_yaml]
        ),
        Node(
            namespace='tb3_1',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[tb3_1_nav2_yaml]
        ),


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['map_server', 'tb3_0/amcl', 'tb3_1/amcl']}]
        ), 

        ## TB3_0             
        Node(
            namespace='tb3_0',
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml_tb3_0]),

        Node(
            namespace='tb3_0',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml_tb3_0]),
            
        Node(
            namespace='tb3_0',
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml_tb3_0],
            output='screen'),

        Node(
            namespace='tb3_0',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml_tb3_0]),

        # ## TB3_1             
        Node(
            namespace='tb3_1',
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml_tb3_1]),

        Node(
            namespace='tb3_1',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml_tb3_1]),
            
        Node(
            namespace='tb3_1',
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml_tb3_1],
            output='screen'),

        Node(
            namespace='tb3_1',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml_tb3_1]),


        ## LIFECICLE MANAGER
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            
            parameters=[{'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['tb3_0/controller_server',
                                        'tb3_0/planner_server',
                                        'tb3_0/recoveries_server',
                                        'tb3_0/bt_navigator',
                                        'tb3_1/controller_server',
                                        'tb3_1/planner_server',
                                        'tb3_1/recoveries_server',
                                        'tb3_1/bt_navigator',
                                        ]}])
    ])