from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('studica_control')
    config_file = os.path.join(pkg_share, 'config', 'ydlidar.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=config_file,
            description='YDLIDAR parameters YAML file'
        ),

        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        ),

    #    Node(
    #        package='tf2_ros',
    #        executable='static_transform_publisher',
    #        name='lidar_tf',
    #        arguments=[
    #            '--x', '0.20',
    #            '--y', '0.0',
    #            '--z', '0.15',
    #            '--roll', '0.0',
    #            '--pitch', '0.0',
    #            '--yaw', '0.0',
    #         #    '--yaw', '-1.5708',
    #            '--frame-id', 'base_link',
    #            '--child-frame-id', 'laser_frame'
    #        ]
    #    )
    ])
