from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():


    localization = Node(
        package='robot_localization',
        executable='main',
        output='screen',
    )


    node = [ 
        localization, 
    ]

    return LaunchDescription( node )