"""
Autonomous mapping launch script using LiDAR and SLAM.
The robot will generate a map as you drive it around. Use: ros2 run teleop_twist_keyboard teleop_twist_keyboard
To save the map, open a new, privledged terminal and run: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

import os
import time
import yaml

def generate_launch_description():

    pkg_share = get_package_share_directory('studica_control')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # mapper_params_file = os.path.join(os.path.dirname(pkg_share), '..', '..', '..', 'nav2_params', 'mapper_params_online_sync.yaml')

 
    # manual_composition = Node(
    #     package='studica_control',
    #     executable='manual_composition',
    #     name='control_server',
    #     output='screen',
    #     parameters=[params_file]
    # )
    # foxglove = Node(
    #     package='foxglove_bridge',
    #     executable='foxglove_bridge',
    #     name='foxglove_bridge',
    #     output='screen'
    # )

    # Launch Foxglove to monitor data
    # foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])

    # foxglove_bridge = ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])

    # laser_tf = ExecuteProcess(
    #     cmd=[[
    #         'ros2 run tf2_ros static_transform_publisher --x 0.2 --y 0 --z 0.15 --qx 0 --qy 0 --qz -0.7071 --qw 0.7071 --frame-id base_link --child-frame-id laser_frame'
    #     ]],
    #     shell=True

    # )

    
    # laser_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='lidar_tf',
    #     arguments=[
    #         '0.20', '0.0', '0.15',      # x y z (meters)
    #         '0.0', '0.0', '0.0',        # roll pitch yaw (radians)
    #         'base_link',
    #         'laser_frame'
    #     ]
    # )

    base_tf = ExecuteProcess(
        cmd=[[
            'ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link'
        ]],
        shell=True
    )
    

    # lidar = ExecuteProcess(
    #     cmd=['ros2', 'launch', 'ydlidar_ros2_driver', 'ydlidar_launch.py'],
    #     output='screen'
    # )

    # slam = ExecuteProcess(
    #     cmd=['ros2', 'launch', 'slam_toolbox', 'online_sync_launch.py', 'use_sim_time:=false'],
    #     output='screen'
    # )

    navigation = ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'use_sim_time:=false', 
             'params_file:=/home/vmx/ROS2/nav2_params/nav2_params.yaml'
             ],
        output='screen'
    )

        # Run slam_toolbox with all parameters from YAML file
    # slam = Node(
    #     package='slam_toolbox',
    #     executable='sync_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen',
    #     parameters=[
    #         mapper_params_file
    #         # {'resolution': LaunchConfiguration('resolution')}  # Override resolution from launch arg
    #     ]
    # )

    # joy_node = ExecuteProcess(
    #     cmd=['ros2', 'run', 'joy', 'game_controller_node'],
    #     output='screen'
    # )

    nodes = [
        # foxglove_studio,
        # foxglove,
        # foxglove_bridge,
        # manual_composition,
        base_tf,
        # laser_tf,
        # lidar,
        # slam,
        navigation,
        # joy_node,
    ]

    return LaunchDescription(nodes)
