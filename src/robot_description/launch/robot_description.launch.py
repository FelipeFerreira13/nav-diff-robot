from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    pkg_path = get_package_share_directory('robot_description')
  
    xacro = os.path.join(pkg_path, 'urdf', 'four_wheels', 'robot.urdf.xacro')


    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', xacro])
            }]
        ),

    ])
