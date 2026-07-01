from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('robot_control')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    control = Node(
        package='robot_control',
        executable='main',
        name='controller_node',
        output='screen',
        parameters=[params_file]
    )


    node = [ 
        control, 
    ]

    return LaunchDescription( node )