
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


import os
import time
import yaml

def generate_launch_description():

    use_sim_time = True
    params_file = "/home/robot/ROS2/nav2_params/nav2_params.yaml"
    map_file = '/home/robot/ROS2/src/robot_core/maps/unknown_maze.yaml'

    base_tf = ExecuteProcess(
        cmd=[[
            'ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link'
        ]],
        shell=True
    )

    map_tf = ExecuteProcess(
        cmd=[[
            'ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom'
        ]],
        shell=True
    )


    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time
        }]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )


    navigation = ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 
             f'use_sim_time:={use_sim_time}', 
             f'params_file:={params_file}'
             ],
        output='screen'
    )

    nodes = [
        # base_tf,
        map_tf,
        map_server,
        lifecycle_manager,
        navigation,
    ]

    return LaunchDescription(nodes)
